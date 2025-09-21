/*
 * SGI Demo at EL3 (FIQ-only path, open Group0, enable SGI0 only, with drains)
 */

#include <stdint.h>
#include <common/debug.h>
#include <lib/mmio.h>
#include <arch_helpers.h>
#include <bl31/interrupt_mgmt.h>

#include <drivers/arm/gic_common.h>
#include <drivers/arm/gicv3.h>
#include <lib/utils_def.h>
#include <platform_def.h>

/* ========== 选项：是否在本文件自行注册 EL3 handler（默认 0，建议用平台注册） ========== */
#ifndef SGI_DEMO_REGISTER_LOCAL_EL3_HANDLER
#define SGI_DEMO_REGISTER_LOCAL_EL3_HANDLER   0
#endif

#define SGI_ID       U(0)      /* SGI0 */
#define SGI_PRIORITY U(0x10)

/* 一个 GICR frame 的跨度 */
#define RD_STRIDE    (1U << GICR_V3_PCPUBASE_SHIFT)

/* --------- 小工具 --------- */
static inline uint32_t mpidr_to_affval(u_register_t mpidr)
{
	return (uint32_t)(((mpidr >> 32) & 0xffU) << 24 |
	                  ((mpidr >> 16) & 0xffU) << 16 |
	                  ((mpidr >>  8) & 0xffU) <<  8 |
	                  ((mpidr >>  0) & 0xffU) <<  0);
}

static uintptr_t find_my_rdist(void)
{
	u_register_t mpidr = read_mpidr_el1();
	uint32_t my_aff    = mpidr_to_affval(mpidr);

	uintptr_t rd = BASE_GICR_BASE;
	for (;;) {
		uint64_t typer = mmio_read_64(rd + GICR_TYPER);
		uint32_t aff   = (uint32_t)(typer >> TYPER_AFF_VAL_SHIFT);
		if (aff == my_aff) return rd;
		if (typer & TYPER_LAST_BIT) break;
		rd += RD_STRIDE;
	}
	return BASE_GICR_BASE; /* 兜底 */
}

static inline void gicr_write_ipriority(uintptr_t rdist_base, uint32_t intid, uint8_t prio)
{
	uintptr_t reg = rdist_base + GICR_IPRIORITYR + ((intid & ~U(3)) * 1U);
	uint32_t  sh  = (intid & U(3)) * 8U;
	uint32_t  v   = mmio_read_32(reg);
	v = (v & ~(0xFFu << sh)) | ((uint32_t)prio << sh);
	mmio_write_32(reg, v);
}

static void rdist_wakeup(uintptr_t my_rdist)
{
	uint32_t w = mmio_read_32(my_rdist + GICR_WAKER);
	if (w & BIT_32(WAKER_PS_SHIFT)) {
		mmio_write_32(my_rdist + GICR_WAKER, w & ~BIT_32(WAKER_PS_SHIFT));
		while (mmio_read_32(my_rdist + GICR_WAKER) & BIT_32(WAKER_CA_SHIFT)) { }
	}
}

/* 只把 SGI0 配成 Secure Group0，其他 SGI/PPI 全关并清 pending，防止杂音 */
static void config_sgi0_on_this_cpu(uintptr_t my_rdist)
{
	uint32_t v;

	/* Secure Group0 */
	v  = mmio_read_32(my_rdist + GICR_IGROUPR0);
	v &= ~BIT_32(SGI_ID);
	mmio_write_32(my_rdist + GICR_IGROUPR0, v);

	/* 非 1NS（即 Secure） */
	v  = mmio_read_32(my_rdist + GICR_IGRPMODR0);
	v &= ~BIT_32(SGI_ID);
	mmio_write_32(my_rdist + GICR_IGRPMODR0, v);

	gicr_write_ipriority(my_rdist, SGI_ID, (uint8_t)SGI_PRIORITY);

	/* 先全关 + 清 pending，再只开 SGI0 */
	mmio_write_32(my_rdist + GICR_ICENABLER0, 0xFFFFFFFFu);
	mmio_write_32(my_rdist + GICR_ICPENDR0,   0xFFFFFFFFu);
	mmio_write_32(my_rdist + GICR_ISENABLER0, BIT_32(SGI_ID));

	uint32_t en = mmio_read_32(my_rdist + GICR_ISENABLER0);
	NOTICE("GICR_ISENABLER0 after enable = 0x%x\n", en);
}
#if 0
/* 只把 FIQ 路由到 EL3（IRQ 仍给下层），避免把别的 IRQ 吸到 EL3 导致卡死 */
static void route_only_fiq_to_el3(void)
{
	u_register_t scr = read_scr_el3();
	scr |=  SCR_FIQ_BIT;   /* FIQ -> EL3 */
	scr &= ~SCR_IRQ_BIT;   /* IRQ 保持在下层 */
	write_scr_el3(scr);
	isb();
}

static void unmask_irqs(void)
{
    asm volatile("msr daifclr, #3" ::: "memory"); // 清除 IRQ(0x2) 和 FIQ(0x1)
}
/* 打通 EL3/G0（严格顺序：先 PMR=0、开 SRE、开 G0、只放开 FIQ、最后 PMR=0xFF） */
static void open_el3_g0_path(void)
{
	/* 1) 收紧 PMR，避免半开状态被抢断 */
	write_icc_pmr_el1(0x00);

	/* 2) 允许 EL3 使用 ICC 接口 */
	write_icc_sre_el3(read_icc_sre_el3() | (ICC_SRE_SRE_BIT|ICC_SRE_DIB_BIT|ICC_SRE_DFB_BIT));
	isb();

	/* 3) GICD: 开 Group0 + ARE_S */
	uint32_t dctlr = mmio_read_32(BASE_GICD_BASE + GICD_CTLR);
	dctlr |= (CTLR_ENABLE_G0_BIT | CTLR_ARE_S_BIT);
	mmio_write_32(BASE_GICD_BASE + GICD_CTLR, dctlr);

	/* 4) 开 EL3 的 Group0 接口 */
	write_icc_igrpen0_el1(1U);

	/* 5) 只路由 FIQ 到 EL3，并只解屏蔽 FIQ（不解 IRQ） */
	route_only_fiq_to_el3();
	asm volatile("msr daifclr, #1" ::: "memory");  /* 清 F 位，仅放开 FIQ */

	/* 6) 最后再放开 PMR */
	write_icc_pmr_el1(0xFF);

	NOTICE("DAIF=0x%lx, PMR=0x%x, IGRPEN0=0x%x\n",
	       read_daif(),
	       (unsigned)read_icc_pmr_el1(),
	       (unsigned)read_icc_igrpen0_el1());
}
#endif


static void unmask_irqs(void)
{
    asm volatile("msr daifclr, #3" ::: "memory"); // 清除 IRQ(0x2) 和 FIQ(0x1)
}


static void enable_el3_g0_paths(void)
{
    uint32_t dctlr = mmio_read_32(BASE_GICD_BASE + GICD_CTLR);
    dctlr |= (CTLR_ENABLE_G0_BIT | CTLR_ARE_S_BIT);
    mmio_write_32(BASE_GICD_BASE + GICD_CTLR, dctlr);

    write_icc_sre_el3(read_icc_sre_el3() |
                      (ICC_SRE_SRE_BIT | ICC_SRE_DIB_BIT | ICC_SRE_DFB_BIT));
    isb();

    write_icc_pmr_el1(0xFFU);
    write_icc_igrpen0_el1(1U);

    unmask_irqs();  // <--- 关键

    NOTICE("DAIF=0x%lx, PMR=0x%x, IGRPEN0=0x%x\n",
           read_daif(),
           (unsigned)read_icc_pmr_el1(),
           (unsigned)read_icc_igrpen0_el1());
}


/* 发送 EL3/Group0 的 SGI0 到“自己” */
static void send_sgi0_to_self(void)
{
	u_register_t mpidr = read_mpidr_el1();
	unsigned long aff0 =  mpidr        & 0xFFUL;
	unsigned long aff1 = (mpidr >>  8) & 0xFFUL;
	unsigned long aff2 = (mpidr >> 16) & 0xFFUL;
	unsigned long aff3 = (mpidr >> 32) & 0xFFUL;

	uint64_t sgi0r = GICV3_SGIR_VALUE(aff3, aff2, aff1, SGI_ID,
	                                  SGIR_IRM_TO_AFF, (1UL << aff0));
	write_icc_sgi0r_el1(sgi0r); /* EL3/Group0 用 SGI0R */
	isb();
	NOTICE("EL3: SGI%u sent to self (MPIDR 0x%lx)\n", SGI_ID, (unsigned long)mpidr);
}

/* 可选：本地 EL3 handler（默认不注册；若要本地注册，把宏置 1 即可） */
static __attribute__((used))
uint64_t sgi_demo_el3_handler(uint32_t id, uint32_t flags, void *h, void *cookie)
{
	uint32_t intid = gicv3_acknowledge_interrupt(); /* IAR0 */
	if (!gicv3_is_intr_id_special_identifier(intid)) {
		NOTICE("EL3(local): intid=%u (param id=%u)\n", intid, id);
		gicv3_end_of_interrupt(intid);               /* EOIR0 */
	}
	return 0U;
}

/* 入口：在 bl31 平台 setup 里调用（GIC 初始化之后） */
void sgi_demo_el3_init(void)
{
	NOTICE("==== SGI Demo (EL3/G0) Init -1 ====\n");

	uintptr_t my_rdist = find_my_rdist();
	rdist_wakeup(my_rdist);
	config_sgi0_on_this_cpu(my_rdist);

#if SGI_DEMO_REGISTER_LOCAL_EL3_HANDLER
	int rc = register_interrupt_type_handler(INTR_TYPE_EL3, sgi_demo_el3_handler, 0U);
	if (rc) NOTICE("register_interrupt_type_handler rc=%d\n", rc);
#endif

	//open_el3_g0_path();
    enable_el3_g0_paths();

	/* 打开后先排水一次（有杂音先清掉） */
	for (int i = 0; i < 16; ++i) {
		uint32_t id = gicv3_acknowledge_interrupt();
		if (!gicv3_is_intr_id_special_identifier(id)) {
			NOTICE("EL3: drain pending intid=%u\n", id);
			gicv3_end_of_interrupt(id);
		} else {
			break;
		}
	}

	/* 触发 SGI0（应以 FIQ 进入 EL3 handler） */
	send_sgi0_to_self();

	/* 可选：等一下，确保走异常路径 */
	dsbsy(); isb();
	NOTICE("EL3: waiting for SGI0 via WFI...\n");
	wfi();

	/* 兜底：再轮询一次，防止没走异常路径 */
	for (int i = 0; i < 16; ++i) {
		uint32_t id = gicv3_acknowledge_interrupt();
		if (!gicv3_is_intr_id_special_identifier(id)) {
			NOTICE("EL3: polled fallback intid=%u\n", id);
			gicv3_end_of_interrupt(id);
			break;
		}
	}

	NOTICE("==== SGI Demo (EL3/G0) Init Done ====\n");
}
