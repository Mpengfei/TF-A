#include <common/debug.h>
#include <stdint.h>
#include <lib/mmio.h>
#include <arch_helpers.h>   /* for read/write ICC_* system registers */

/* Redistributor base: core0 */
#define GICR_BASE        0x2f100000UL
#define GICR_CTLR        0x0000
#define GICR_TYPER       0x0008
#define GICR_WAKER       0x0014
#define GICR_IGROUPR0    0x0080
#define GICR_ISENABLER0  0x0100
#define GICR_ICENABLER0  0x0180

/* Distributor base */
#define GICD_BASE        0x2f000000UL
#define GICD_CTLR        0x0000

static inline void gic_cpuif_enable(void)
{
    /* 1. 使能系统寄存器接口 */
    write_icc_sre_el3(read_icc_sre_el3() | 0x7);
    isb();

    /* 2. 设置优先级掩码 (0xff = 允许所有中断) */
    write_icc_pmr_el1(0xff);

    /* 3. 允许 Group1 中断 */
    write_icc_igrpen1_el1(1);

    NOTICE("CPU interface enabled (SRE_EL3=0x%lx, PMR=0x%lx)\n",
           read_icc_sre_el3(), read_icc_pmr_el1());
}

void test_sgi_enable(void)
{
    NOTICE("==== SGI Init Test ====\n");

    /* 确保 Redistributor 唤醒 */
    uint32_t waker = mmio_read_32(GICR_BASE + GICR_WAKER);
    waker &= ~(1U << 1); /* Clear ProcessorSleep */
    mmio_write_32(GICR_BASE + GICR_WAKER, waker);
    while (mmio_read_32(GICR_BASE + GICR_WAKER) & (1U << 2)) {
        /* 等待 ChildrenAsleep 清零 */
    }
    NOTICE("GICR_WAKER after wakeup = 0x%x\n", mmio_read_32(GICR_BASE + GICR_WAKER));

    /* 设置 SGI 属于 Group0 */
    mmio_write_32(GICR_BASE + GICR_IGROUPR0, 0x0);

    /* 读 ISENABLER0 原值 */
    uint32_t val = mmio_read_32(GICR_BASE + GICR_ISENABLER0);
    NOTICE("Before: GICR_ISENABLER0 = 0x%x\n", val);

    /* 使能 SGI0 */
    mmio_write_32(GICR_BASE + GICR_ISENABLER0, 0x1);

    /* 再读回来 */
    val = mmio_read_32(GICR_BASE + GICR_ISENABLER0);
    NOTICE("After Enable: GICR_ISENABLER0 = 0x%x\n", val);

    /* 禁用 SGI0 */
    mmio_write_32(GICR_BASE + GICR_ICENABLER0, 0x1);
    val = mmio_read_32(GICR_BASE + GICR_ISENABLER0);
    NOTICE("After Disable: GICR_ISENABLER0 = 0x%x\n", val);

    /* 打开 CPU interface */
    gic_cpuif_enable();

    NOTICE("==== SGI Init Test Done ====\n");
}
