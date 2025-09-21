#include <stdint.h>
#include <common/debug.h>
#include <lib/mmio.h>
#include <arch_helpers.h>
#include <bl31/interrupt_mgmt.h>

#include <drivers/arm/gic_common.h>
#include <drivers/arm/gicv3.h>
#include <lib/utils_def.h>
#include <platform_def.h>

#define SGI_ID       U(0)      /* SGI0 */
#define SGI_PRIORITY U(0x10)

#if 0
static void route_secure_irqs_to_el3(void)
{
    u_register_t scr = read_scr_el3();
    scr |= (SCR_IRQ_BIT | SCR_FIQ_BIT);   // 允许 IRQ/FIQ 路由到 EL3
    write_scr_el3(scr);
    isb();
}
#endif
uint32_t plat_ic_get_pending_interrupt_type(void)
{
    /* 用 ICC_HPPIR{0,1} 判定：EL3/G0、S-EL1/G1S、或 NS/G1NS */
    return gicv3_get_pending_interrupt_type();
}

/* 不是必须，但补上更完整： */
uint32_t plat_ic_get_pending_interrupt_id(void)
{
    /* 只查询（不 Ack）。有时调试方便。 */
    return gicv3_get_pending_interrupt_id();
}

void plat_ic_end_of_interrupt(uint32_t id)
{
    /* 如果你在别处用到了 plat_ic_* 的 EOI，也给个转发 */
    gicv3_end_of_interrupt(id);
}


/* 按规范：一个 GICR frame 的跨度 */
#define RD_STRIDE    (1U << GICR_V3_PCPUBASE_SHIFT)

/* 根据 MPIDR 取 Aff3:Aff2:Aff1:Aff0 拼 32b 值 */
static inline uint32_t mpidr_to_affval(u_register_t mpidr)
{
    return (uint32_t)(((mpidr >> 32) & 0xffU) << 24 |
                      ((mpidr >> 16) & 0xffU) << 16 |
                      ((mpidr >>  8) & 0xffU) <<  8 |
                      ((mpidr >>  0) & 0xffU) <<  0);
}

/* 遍历 GICR frames，找到“我的” GICR 基址 */
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
    /* 没找到就退回起点（极少见） */
    return BASE_GICR_BASE;
}

static inline void gicr_write_ipriority(uintptr_t rdist_base, uint32_t intid, uint8_t prio)
{
    /* 注意：GICR_IPRIORITYR 已经包含 SGI 子块的偏移 */
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

static void config_sgi0_on_this_cpu(uintptr_t my_rdist)
{
    uint32_t v;

    /* 这些宏本身已经是 “my_rdist + SGI 子块 + 寄存器” 的偏移 */
    v  = mmio_read_32(my_rdist + GICR_IGROUPR0);
    v &= ~BIT_32(SGI_ID);                      /* Secure Group0 */
    mmio_write_32(my_rdist + GICR_IGROUPR0, v);

    v  = mmio_read_32(my_rdist + GICR_IGRPMODR0);
    v &= ~BIT_32(SGI_ID);                      /* 非 1NS，即 Secure */
    mmio_write_32(my_rdist + GICR_IGRPMODR0, v);

    gicr_write_ipriority(my_rdist, SGI_ID, (uint8_t)SGI_PRIORITY);

    /* 使能 SGI0 —— 直接在 my_rdist + GICR_ISENABLER0 */
    mmio_write_32(my_rdist + GICR_ISENABLER0, BIT_32(SGI_ID));

    /* 建议读回核对一下 */
    uint32_t en = mmio_read_32(my_rdist + GICR_ISENABLER0);
    NOTICE("GICR_ISENABLER0 after enable = 0x%x\n", en);
}

#if 0
static void enable_el3_g0_paths(void)
{
    /* Distributor：开 Group0 + ARE_S */
    uint32_t dctlr = mmio_read_32(BASE_GICD_BASE + GICD_CTLR);
    dctlr |= (CTLR_ENABLE_G0_BIT | CTLR_ARE_S_BIT);
    mmio_write_32(BASE_GICD_BASE + GICD_CTLR, dctlr);

    /* 允许 EL3 访问 ICC 接口 */
    write_icc_sre_el3(read_icc_sre_el3() | (ICC_SRE_SRE_BIT | ICC_SRE_DIB_BIT | ICC_SRE_DFB_BIT));
    isb();

    /* 放开优先级屏蔽，打开 G0 */
    write_icc_pmr_el1(0xFFU);
    write_icc_igrpen0_el1(1U);

    /* 取消 EL3 屏蔽位 */
    enable_fiq();
    enable_irq();
}
#endif

static void send_sgi0_to_self(void)
{
    u_register_t mpidr = read_mpidr_el1();
    unsigned long aff0 =  mpidr        & 0xFFUL;
    unsigned long aff1 = (mpidr >>  8) & 0xFFUL;
    unsigned long aff2 = (mpidr >> 16) & 0xFFUL;
    unsigned long aff3 = (mpidr >> 32) & 0xFFUL;

    uint64_t sgi0r = GICV3_SGIR_VALUE(aff3, aff2, aff1, SGI_ID,
                                      SGIR_IRM_TO_AFF, (1UL << aff0));
    write_icc_sgi0r_el1(sgi0r); /* 注意：EL3/Group0 用 SGI0R */
    isb();

    NOTICE("EL3: SGI%u sent to self (MPIDR 0x%lx)\n", SGI_ID, (unsigned long)mpidr);
}

static uint64_t el3_g0_handler(uint32_t id, uint32_t flags, void *h, void *cookie)
{
    uint32_t intid = gicv3_acknowledge_interrupt(); /* IAR0 */
    NOTICE("EL3: G0 IRQ received: intid=%u (param id=%u)\n", intid, id);
    gicv3_end_of_interrupt(intid);                  /* EOIR0 */
    return 0U;
}
#if 0
//可以用
void sgi_demo_el3_init(void)
{
    NOTICE("==== SGI Demo (EL3/G0) Init -1  ====\n");
    route_secure_irqs_to_el3();
    uintptr_t my_rdist = find_my_rdist();
    rdist_wakeup(my_rdist);
    config_sgi0_on_this_cpu(my_rdist);
    enable_el3_g0_paths();

    register_interrupt_type_handler(INTR_TYPE_EL3, el3_g0_handler, 0U);

    send_sgi0_to_self();
    NOTICE("DAIF=0x%lx, PMR=0x%x, IGRPEN0=0x%x\n",
       read_daif(),
       (unsigned)read_icc_pmr_el1(),
       (unsigned)read_icc_igrpen0_el1());

        // dsbsy();
        // isb();
        // wfi();   // 让出执行等待中断进来，进入向量表
        /* 关键：立刻等待这个中断，确保走异常向量路径进 handler */
    dsbsy();
    isb();
    NOTICE("EL3: waiting for SGI0 via WFI...\n");
    wfi();  /* 应该马上被 EL3/G0 中断打断，跳到 irq_aarch64/fiq_aarch64 -> handle_interrupt_exception -> el3_g0_handler */

    // /* 兜底：如果某些初始化时序导致没进异常路径，就主动“轮询+EOI”一次 */
    // for (int i = 0; i < 1000; ++i) {
    //     uint32_t intid = gicv3_acknowledge_interrupt();  /* 读 IAR0，会把该中断标记为 Active */
    //     if (!gicv3_is_intr_id_special_identifier(intid)) {
    //         NOTICE("EL3: G0 IRQ received (polled fallback): intid=%u\n", intid);
    //         gicv3_end_of_interrupt(intid);               /* EOIR0 */
    //         break;
    //     }
    // }
    #if 0
    /* just after send_sgi0_to_self(); */
    for (volatile unsigned i = 0; i < 1000000; ++i) {
        unsigned id = (unsigned)(read_icc_iar0_el1() & 0xFFFFFFu);
        if (id < 1020u) {  /* 0..1019 为有效 INTID */
            NOTICE("EL3: polled G0 IRQ: intid=%u\n", id);
            write_icc_eoir0_el1(id);
            break;
        }
    }
    #endif

    NOTICE("==== SGI Demo (EL3/G0) Init Done ====\n");
}
#endif

void sgi_demo_el3_init(void)
{
    NOTICE("==== SGI Demo (EL3/G0) Init ====\n");

    uintptr_t rd = find_my_rdist();
    rdist_wakeup(rd);
    config_sgi0_on_this_cpu(rd);      // 配好 SGI0，但先不放开

    // 1) 先安装 EL3 handler
    int rc = register_interrupt_type_handler(INTR_TYPE_EL3, el3_g0_handler, 0U);
    if (rc) NOTICE("register_interrupt_type_handler rc=%d\n", rc);

    // 2) 先收紧 PMR，防止“半开状态”中断立刻打进来
    write_icc_pmr_el1(0x00);

    // 3) 允许 ICC SRE & GICD G0/ARE_S
    uint32_t dctlr = mmio_read_32(BASE_GICD_BASE + GICD_CTLR);
    dctlr |= (CTLR_ENABLE_G0_BIT | CTLR_ARE_S_BIT);
    mmio_write_32(BASE_GICD_BASE + GICD_CTLR, dctlr);
    write_icc_sre_el3(read_icc_sre_el3() | (ICC_SRE_SRE_BIT|ICC_SRE_DIB_BIT|ICC_SRE_DFB_BIT));
    isb();

    // 4) 路由 Secure IRQ/FIQ 到 EL3
    u_register_t scr = read_scr_el3();
    scr |= (SCR_IRQ_BIT | SCR_FIQ_BIT);
    write_scr_el3(scr);
    isb();

    // 5) 打开 G0 接口 & 取消 EL3 屏蔽
    write_icc_igrpen0_el1(1U);
    enable_irq();
    enable_fiq();

    // 6) 最后再放开 PMR
    write_icc_pmr_el1(0xFF);

    // 7) 触发 SGI
    send_sgi0_to_self();

    NOTICE("==== SGI Demo (EL3/G0) Init Done ====\n");
}


