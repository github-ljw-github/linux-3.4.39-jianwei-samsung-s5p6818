/* linux/arch/arm/mach-exynos4/mct.c
 *
 * Copyright (c) 2011 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * EXYNOS4 MCT(Multi-Core Timer) support
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/clockchips.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/percpu.h>

#include <asm/hardware/gic.h>
#include <asm/localtimer.h>

#include <plat/cpu.h>

#include <mach/map.h>
#include <mach/irqs.h>
#include <mach/regs-mct.h>
#include <asm/mach/time.h>

#define TICK_BASE_CNT	1

enum {
	MCT_INT_SPI,
	MCT_INT_PPI
};

static unsigned long clk_rate;
static unsigned int mct_int_type;

struct mct_clock_event_device {
	struct clock_event_device *evt;
	void __iomem *base;
	char name[10];
};

static void exynos4_mct_write(unsigned int value, void *addr)
{
	void __iomem *stat_addr;
	u32 mask;
	u32 i;

	__raw_writel(value, addr);

	if (likely(addr >= EXYNOS4_MCT_L_BASE(0))) {
		u32 base = (u32) addr & EXYNOS4_MCT_L_MASK;
		switch ((u32) addr & ~EXYNOS4_MCT_L_MASK) {
		case (u32) MCT_L_TCON_OFFSET:
			stat_addr = (void __iomem *) base + MCT_L_WSTAT_OFFSET;
			mask = 1 << 3;		/* L_TCON write status */
			break;
		case (u32) MCT_L_ICNTB_OFFSET:
			stat_addr = (void __iomem *) base + MCT_L_WSTAT_OFFSET;
			mask = 1 << 1;		/* L_ICNTB write status */
			break;
		case (u32) MCT_L_TCNTB_OFFSET:
			stat_addr = (void __iomem *) base + MCT_L_WSTAT_OFFSET;
			mask = 1 << 0;		/* L_TCNTB write status */
			break;
		default:
			return;
		}
	} else {
		switch ((u32) addr) {
		case (u32) EXYNOS4_MCT_G_TCON:
			stat_addr = EXYNOS4_MCT_G_WSTAT;
			mask = 1 << 16;		/* G_TCON write status */
			break;
		case (u32) EXYNOS4_MCT_G_COMP0_L:
			stat_addr = EXYNOS4_MCT_G_WSTAT;
			mask = 1 << 0;		/* G_COMP0_L write status */
			break;
		case (u32) EXYNOS4_MCT_G_COMP0_U:
			stat_addr = EXYNOS4_MCT_G_WSTAT;
			mask = 1 << 1;		/* G_COMP0_U write status */
			break;
		case (u32) EXYNOS4_MCT_G_COMP0_ADD_INCR:
			stat_addr = EXYNOS4_MCT_G_WSTAT;
			mask = 1 << 2;		/* G_COMP0_ADD_INCR w status */
			break;
		case (u32) EXYNOS4_MCT_G_CNT_L:
			stat_addr = EXYNOS4_MCT_G_CNT_WSTAT;
			mask = 1 << 0;		/* G_CNT_L write status */
			break;
		case (u32) EXYNOS4_MCT_G_CNT_U:
			stat_addr = EXYNOS4_MCT_G_CNT_WSTAT;
			mask = 1 << 1;		/* G_CNT_U write status */
			break;
		default:
			return;
		}
	}

	/* Wait maximum 1 ms until written values are applied */
	for (i = 0; i < loops_per_jiffy / 1000 * HZ; i++)
		if (__raw_readl(stat_addr) & mask) {
			__raw_writel(mask, stat_addr);
			return;
		}

	panic("MCT hangs after writing %d (addr:0x%08x)\n", value, (u32)addr);
}

/* Clocksource handling */
static void exynos4_mct_frc_start(u32 hi, u32 lo)
{
	u32 reg;

	exynos4_mct_write(lo, EXYNOS4_MCT_G_CNT_L);
	exynos4_mct_write(hi, EXYNOS4_MCT_G_CNT_U);

	reg = __raw_readl(EXYNOS4_MCT_G_TCON);
	reg |= MCT_G_TCON_START;
	exynos4_mct_write(reg, EXYNOS4_MCT_G_TCON);
}

static cycle_t exynos4_frc_read(struct clocksource *cs)
{
	unsigned int lo, hi;
	u32 hi2 = __raw_readl(EXYNOS4_MCT_G_CNT_U);

	do {
		hi = hi2;
		lo = __raw_readl(EXYNOS4_MCT_G_CNT_L);
		hi2 = __raw_readl(EXYNOS4_MCT_G_CNT_U);
	} while (hi != hi2);

	return ((cycle_t)hi << 32) | lo;
}

static void exynos4_frc_resume(struct clocksource *cs)
{
	exynos4_mct_frc_start(0, 0);
}

struct clocksource mct_frc = {
	.name		= "mct-frc",
	.rating		= 400,
	.read		= exynos4_frc_read,
	.mask		= CLOCKSOURCE_MASK(64),
	.flags		= CLOCK_SOURCE_IS_CONTINUOUS,
	.resume		= exynos4_frc_resume,
};

static void __init exynos4_clocksource_init(void)
{
	exynos4_mct_frc_start(0, 0);

	if (clocksource_register_hz(&mct_frc, clk_rate))
		panic("%s: can't register clocksource\n", mct_frc.name);
}

static void exynos4_mct_comp0_stop(void)
{
	unsigned int tcon;

	tcon = __raw_readl(EXYNOS4_MCT_G_TCON);
	tcon &= ~(MCT_G_TCON_COMP0_ENABLE | MCT_G_TCON_COMP0_AUTO_INC);

	exynos4_mct_write(tcon, EXYNOS4_MCT_G_TCON);
	exynos4_mct_write(0, EXYNOS4_MCT_G_INT_ENB);
}

static void exynos4_mct_comp0_start(enum clock_event_mode mode,
				    unsigned long cycles)
{
	unsigned int tcon;
	cycle_t comp_cycle;

	tcon = __raw_readl(EXYNOS4_MCT_G_TCON);

	if (mode == CLOCK_EVT_MODE_PERIODIC) {
		tcon |= MCT_G_TCON_COMP0_AUTO_INC;
		exynos4_mct_write(cycles, EXYNOS4_MCT_G_COMP0_ADD_INCR);
	}

	comp_cycle = exynos4_frc_read(&mct_frc) + cycles;
	exynos4_mct_write((u32)comp_cycle, EXYNOS4_MCT_G_COMP0_L);
	exynos4_mct_write((u32)(comp_cycle >> 32), EXYNOS4_MCT_G_COMP0_U);

	exynos4_mct_write(0x1, EXYNOS4_MCT_G_INT_ENB);

	tcon |= MCT_G_TCON_COMP0_ENABLE;
	exynos4_mct_write(tcon , EXYNOS4_MCT_G_TCON);
}

static int exynos4_comp_set_next_event(unsigned long cycles,
				       struct clock_event_device *evt)
{
	exynos4_mct_comp0_start(evt->mode, cycles);

	return 0;
}

static void exynos4_comp_set_mode(enum clock_event_mode mode,
				  struct clock_event_device *evt)
{
	unsigned long cycles_per_jiffy;
	exynos4_mct_comp0_stop();

	switch (mode) {
	case CLOCK_EVT_MODE_PERIODIC:
		cycles_per_jiffy =
			(((unsigned long long) NSEC_PER_SEC / HZ * evt->mult) >> evt->shift);
		exynos4_mct_comp0_start(mode, cycles_per_jiffy);
		break;

	case CLOCK_EVT_MODE_ONESHOT:
	case CLOCK_EVT_MODE_UNUSED:
	case CLOCK_EVT_MODE_SHUTDOWN:
	case CLOCK_EVT_MODE_RESUME:
		break;
	}
}

static struct clock_event_device mct_comp_device = {
	.name		= "mct-comp",
	.features       = CLOCK_EVT_FEAT_PERIODIC | CLOCK_EVT_FEAT_ONESHOT,
	.rating		= 250,
	.set_next_event	= exynos4_comp_set_next_event,
	.set_mode	= exynos4_comp_set_mode,
};

static irqreturn_t exynos4_mct_comp_isr(int irq, void *dev_id)
{
	struct clock_event_device *evt = dev_id;

	exynos4_mct_write(0x1, EXYNOS4_MCT_G_INT_CSTAT);

	evt->event_handler(evt);

	return IRQ_HANDLED;
}

static struct irqaction mct_comp_event_irq = {
	.name		= "mct_comp_irq",
	.flags		= IRQF_TIMER | IRQF_IRQPOLL,
	.handler	= exynos4_mct_comp_isr,
	.dev_id		= &mct_comp_device,
};

static void exynos4_clockevent_init(void)
{
	clockevents_calc_mult_shift(&mct_comp_device, clk_rate, 5);
	mct_comp_device.max_delta_ns =
		clockevent_delta2ns(0xffffffff, &mct_comp_device);
	mct_comp_device.min_delta_ns =
		clockevent_delta2ns(0xf, &mct_comp_device);
	mct_comp_device.cpumask = cpumask_of(0);
	clockevents_register_device(&mct_comp_device);

	if (soc_is_exynos5250())
		setup_irq(EXYNOS5_IRQ_MCT_G0, &mct_comp_event_irq);
	else
		setup_irq(EXYNOS4_IRQ_MCT_G0, &mct_comp_event_irq);
}

#ifdef CONFIG_LOCAL_TIMERS

static DEFINE_PER_CPU(struct mct_clock_event_device, percpu_mct_tick);

/* Clock event handling */
static void exynos4_mct_tick_stop(struct mct_clock_event_device *mevt)
{
	unsigned long tmp;
	unsigned long mask = MCT_L_TCON_INT_START | MCT_L_TCON_TIMER_START;
	void __iomem *addr = mevt->base + MCT_L_TCON_OFFSET;

	tmp = __raw_readl(addr);
	if (tmp & mask) {
		tmp &= ~mask;
		exynos4_mct_write(tmp, addr);
	}
}

static void exynos4_mct_tick_start(unsigned long cycles,
				   struct mct_clock_event_device *mevt)
{
	unsigned long tmp;

	exynos4_mct_tick_stop(mevt);

	tmp = (1 << 31) | cycles;	/* MCT_L_UPDATE_ICNTB */

	/* update interrupt count buffer */
	exynos4_mct_write(tmp, mevt->base + MCT_L_ICNTB_OFFSET);

	/* enable MCT tick interrupt */
	exynos4_mct_write(0x1, mevt->base + MCT_L_INT_ENB_OFFSET);

	tmp = __raw_readl(mevt->base + MCT_L_TCON_OFFSET);
	tmp |= MCT_L_TCON_INT_START | MCT_L_TCON_TIMER_START |
	       MCT_L_TCON_INTERVAL_MODE;
	exynos4_mct_write(tmp, mevt->base + MCT_L_TCON_OFFSET);
}

static int exynos4_tick_set_next_event(unsigned long cycles,
				       struct clock_event_device *evt)
{
	struct mct_clock_event_device *mevt = this_cpu_ptr(&percpu_mct_tick);

	exynos4_mct_tick_start(cycles, mevt);

	return 0;
}

static inline void exynos4_tick_set_mode(enum clock_event_mode mode,
					 struct clock_event_device *evt)
{
	struct mct_clock_event_device *mevt = this_cpu_ptr(&percpu_mct_tick);
	unsigned long cycles_per_jiffy;

	exynos4_mct_tick_stop(mevt);

	switch (mode) {
	case CLOCK_EVT_MODE_PERIODIC:
		cycles_per_jiffy =
			(((unsigned long long) NSEC_PER_SEC / HZ * evt->mult) >> evt->shift);
		exynos4_mct_tick_start(cycles_per_jiffy, mevt);
		break;

	case CLOCK_EVT_MODE_ONESHOT:
	case CLOCK_EVT_MODE_UNUSED:
	case CLOCK_EVT_MODE_SHUTDOWN:
	case CLOCK_EVT_MODE_RESUME:
		break;
	}
}

static int exynos4_mct_tick_clear(struct mct_clock_event_device *mevt)
{
	struct clock_event_device *evt = mevt->evt;

	/*
	 * This is for supporting oneshot mode.
	 * Mct would generate interrupt periodically
	 * without explicit stopping.
	 */
	if (evt->mode != CLOCK_EVT_MODE_PERIODIC)
		exynos4_mct_tick_stop(mevt);

	/* Clear the MCT tick interrupt */
	if (__raw_readl(mevt->base + MCT_L_INT_CSTAT_OFFSET) & 1) {
		exynos4_mct_write(0x1, mevt->base + MCT_L_INT_CSTAT_OFFSET);
		return 1;
	} else {
		return 0;
	}
}

static irqreturn_t exynos4_mct_tick_isr(int irq, void *dev_id)
{
	struct mct_clock_event_device *mevt = dev_id;
	struct clock_event_device *evt = mevt->evt;

	exynos4_mct_tick_clear(mevt);

	evt->event_handler(evt);

	return IRQ_HANDLED;
}

static struct irqaction mct_tick0_event_irq = {
	.name		= "mct_tick0_irq",
	.flags		= IRQF_TIMER | IRQF_NOBALANCING,
	.handler	= exynos4_mct_tick_isr,
};

static struct irqaction mct_tick1_event_irq = {
	.name		= "mct_tick1_irq",
	.flags		= IRQF_TIMER | IRQF_NOBALANCING,
	.handler	= exynos4_mct_tick_isr,
};

static int __cpuinit exynos4_local_timer_setup(struct clock_event_device *evt)
{
	struct mct_clock_event_device *mevt;
	unsigned int cpu = smp_processor_id();

	mevt = this_cpu_ptr(&percpu_mct_tick);
	mevt->evt = evt;

	mevt->base = EXYNOS4_MCT_L_BASE(cpu);
	sprintf(mevt->name, "mct_tick%d", cpu);

	evt->name = mevt->name;
	evt->cpumask = cpumask_of(cpu);
	evt->set_next_event = exynos4_tick_set_next_event;
	evt->set_mode = exynos4_tick_set_mode;
	evt->features = CLOCK_EVT_FEAT_PERIODIC | CLOCK_EVT_FEAT_ONESHOT;
	evt->rating = 450;

	clockevents_calc_mult_shift(evt, clk_rate / (TICK_BASE_CNT + 1), 5);
	evt->max_delta_ns =
		clockevent_delta2ns(0x7fffffff, evt);
	evt->min_delta_ns =
		clockevent_delta2ns(0xf, evt);

	clockevents_register_device(evt);

	exynos4_mct_write(TICK_BASE_CNT, mevt->base + MCT_L_TCNTB_OFFSET);

	if (mct_int_type == MCT_INT_SPI) {
		if (cpu == 0) {
			mct_tick0_event_irq.dev_id = mevt;
			evt->irq = EXYNOS4_IRQ_MCT_L0;
			setup_irq(EXYNOS4_IRQ_MCT_L0, &mct_tick0_event_irq);
		} else {
			mct_tick1_event_irq.dev_id = mevt;
			evt->irq = EXYNOS4_IRQ_MCT_L1;
			setup_irq(EXYNOS4_IRQ_MCT_L1, &mct_tick1_event_irq);
			irq_set_affinity(EXYNOS4_IRQ_MCT_L1, cpumask_of(1));
		}
	} else {
		enable_percpu_irq(EXYNOS_IRQ_MCT_LOCALTIMER, 0);
	}

	return 0;
}

static void exynos4_local_timer_stop(struct clock_event_device *evt)
{
	unsigned int cpu = smp_processor_id();
	evt->set_mode(CLOCK_EVT_MODE_UNUSED, evt);
	if (mct_int_type == MCT_INT_SPI)
		if (cpu == 0)
			remove_irq(evt->irq, &mct_tick0_event_irq);
		else
			remove_irq(evt->irq, &mct_tick1_event_irq);
	else
		disable_percpu_irq(EXYNOS_IRQ_MCT_LOCALTIMER);
}

static struct local_timer_ops exynos4_mct_tick_ops __cpuinitdata = {
	.setup	= exynos4_local_timer_setup,
	.stop	= exynos4_local_timer_stop,
};
#endif /* CONFIG_LOCAL_TIMERS */

static void __init exynos4_timer_resources(void)
{
	struct clk *mct_clk;
	mct_clk = clk_get(NULL, "xtal");

	clk_rate = clk_get_rate(mct_clk);

#ifdef CONFIG_LOCAL_TIMERS
	if (mct_int_type == MCT_INT_PPI) {
		int err;

		err = request_percpu_irq(EXYNOS_IRQ_MCT_LOCALTIMER,
					 exynos4_mct_tick_isr, "MCT",
					 &percpu_mct_tick);
		WARN(err, "MCT: can't request IRQ %d (%d)\n",
		     EXYNOS_IRQ_MCT_LOCALTIMER, err);
	}

	local_timer_register(&exynos4_mct_tick_ops);
#endif /* CONFIG_LOCAL_TIMERS */
}

static void __init exynos4_timer_init(void)
{
#if 0
	if (soc_is_exynos4210())
		mct_int_type = MCT_INT_SPI;
	else
		mct_int_type = MCT_INT_PPI;

	exynos4_timer_resources();
	exynos4_clocksource_init();
	exynos4_clockevent_init();
#endif
}
/**********************************************************************************************************************************/
#ifndef __ASM_ARM_ARCH_IO_H
#define __ASM_ARM_ARCH_IO_H

/*
 * for resource IORESOURCE_IO
 * refer to "platform.c" and CONFIG_NEED_MACH_IO_H
 */
#define IO_SPACE_LIMIT          0xffffffff

/*
 * We don't actually have real ISA nor PCI buses, but there is so many
 * drivers out there that might just work if we fake them...
 */
#define __io(a)                         __typesafe_io(a)

#define __PB_IO_MAP_REGS_PHYS 0xC0000000
#define __PB_IO_MAP_REGS_VIRT 0xF0000000
#define IO_ADDRESS(x)   (x - __PB_IO_MAP_REGS_PHYS + __PB_IO_MAP_REGS_VIRT)
#define __io_address(n) ((void __iomem *)IO_ADDRESS(n))

#endif /* __ASM_ARM_ARCH_IO_H */

/* cpu */
#define DEV_NAME_UART           "nxp-uart"        // pl0115 (amba-pl011.c)
#define DEV_NAME_FB             "nxp-fb"
#define DEV_NAME_DISP           "nxp-disp"
#define DEV_NAME_LCD            "nxp-lcd"
#define DEV_NAME_LVDS           "nxp-lvds"
#define DEV_NAME_HDMI           "nxp-hdmi"
#define DEV_NAME_RESCONV        "nxp-resconv"
#define DEV_NAME_MIPI           "nxp-mipi"
#define DEV_NAME_PCM            "nxp-pcm"
#define DEV_NAME_I2S            "nxp-i2s"
#define DEV_NAME_SPDIF_TX       "nxp-spdif-tx"
#define DEV_NAME_SPDIF_RX       "nxp-spdif-rx"
#define DEV_NAME_I2C            "nxp-i2c"
#define DEV_NAME_NAND           "nxp-nand"
#define DEV_NAME_KEYPAD         "nxp-keypad"
#define DEV_NAME_SDHC           "nxp-sdhc"
#define DEV_NAME_PWM            "nxp-pwm"
#define DEV_NAME_TIMER          "nxp-timer"
#define DEV_NAME_SOC_PWM        "nxp-soc-pwm"
#define DEV_NAME_GPIO           "nxp-gpio"
#define DEV_NAME_RTC            "nxp-rtc"
#define DEV_NAME_GMAC           "nxp-gmac"
#define DEV_NAME_MPEGTSI        "nxp-mpegtsi"
#define DEV_NAME_VR             "nxp-vr"
#define DEV_NAME_DIT            "nxp-deinterlace"
#define DEV_NAME_PPM            "nxp-ppm"
#define DEV_NAME_VIP            "nxp-vip"
#define DEV_NAME_CODA           "nxp-coda"
#define DEV_NAME_USB2HOST       "nxp-usb2h"
#define DEV_NAME_CRYPTO         "nxp-crypto"
#define DEV_NAME_SCALER         "nxp-scaler"
#define DEV_NAME_PDM            "nxp-pdm"
#define DEV_NAME_SPI            "nxp-spi"
#define DEV_NAME_ADC            "nxp-adc"
#define DEV_NAME_CPUFREQ        "nxp-cpufreq"
#define DEV_NAME_USBOTG         "nxp-otg"
#define DEV_NAME_RFKILL         "nxp-rfkill"
#define DEV_NAME_WDT            "nxp-wdt"
#define DEV_NAME_TVOUT          "nxp-tvout"

#ifndef __S5P6818_INTR_H__
#define __S5P6818_INTR_H__

/*
 * GIC Interrupt (0 ~ 32), must be align 32
 */
#define IRQ_GIC_START			(0)
#define IRQ_GIC_PPI_START		(IRQ_GIC_START  +  16)
#define IRQ_GIC_PPI_PVT			(IRQ_GIC_START  +  29)
#define IRQ_GIC_PPI_WDT			(IRQ_GIC_START  +  30)
#define IRQ_GIC_PPI_VIC			(IRQ_GIC_START  +  31)
#define IRQ_GIC_END				(IRQ_GIC_START  +  32)

/*
 * Physical Interrupt Number 64 (0~63)
 */
#define IRQ_PHY_MCUSTOP					(0  + 32)
#define IRQ_PHY_DMA0					(1  + 32)
#define IRQ_PHY_DMA1					(2  + 32)
#define IRQ_PHY_CLKPWR_INTREQPWR		(3  + 32)
#define IRQ_PHY_CLKPWR_ALIVEIRQ			(4  + 32)
#define IRQ_PHY_CLKPWR_RTCIRQ			(5  + 32)
#define IRQ_PHY_UART1					(6	+ 32) // pl01115_Uart_modem
#define IRQ_PHY_UART0					(7	+ 32) // UART0_MODULE
#define IRQ_PHY_UART2					(8	+ 32) // UART1_MODULE
#define IRQ_PHY_UART3					(9	+ 32) // pl01115_Uart_nodma0
#define IRQ_PHY_UART4					(10 + 32)	// pl01115_Uart_nodma1
#define IRQ_PHY_UART5					(11 + 32)	// pl01115_Uart_nodma2
#define IRQ_PHY_SSP0					(12 + 32)
#define IRQ_PHY_SSP1					(13 + 32)
#define IRQ_PHY_SSP2					(14 + 32)
#define IRQ_PHY_I2C0					(15 + 32)
#define IRQ_PHY_I2C1					(16 + 32)
#define IRQ_PHY_I2C2					(17 + 32)
#define IRQ_PHY_DEINTERLACE				(18 + 32)
#define IRQ_PHY_SCALER					(19 + 32)
#define IRQ_PHY_AC97					(20 + 32)
#define IRQ_PHY_SPDIFRX					(21 + 32)
#define IRQ_PHY_SPDIFTX					(22 + 32)
#define IRQ_PHY_TIMER_INT0				(23 + 32)
#define IRQ_PHY_TIMER_INT1				(24 + 32)
#define IRQ_PHY_TIMER_INT2				(25 + 32)
#define IRQ_PHY_TIMER_INT3				(26 + 32)
#define IRQ_PHY_PWM_INT0				(27 + 32)
#define IRQ_PHY_PWM_INT1				(28 + 32)
#define IRQ_PHY_PWM_INT2				(29 + 32)
#define IRQ_PHY_PWM_INT3				(30 + 32)
#define IRQ_PHY_WDT						(31 + 32)
#define IRQ_PHY_MPEGTSI					(32 + 32)
#define IRQ_PHY_DPC_P					(33 + 32)
#define IRQ_PHY_DPC_S					(34 + 32)
#define IRQ_PHY_RESCONV					(35 + 32)
#define IRQ_PHY_HDMI					(36 + 32)
#define IRQ_PHY_VIP0					(37 + 32)
#define IRQ_PHY_VIP1					(38 + 32)
#define IRQ_PHY_MIPI					(39 + 32)
#define IRQ_PHY_VR						(40 + 32)
#define IRQ_PHY_ADC						(41 + 32)
#define IRQ_PHY_PPM						(42 + 32)
#define IRQ_PHY_SDMMC0					(43 + 32)
#define IRQ_PHY_SDMMC1					(44 + 32)
#define IRQ_PHY_SDMMC2					(45 + 32)
#define IRQ_PHY_CODA960_HOST			(46 + 32)
#define IRQ_PHY_CODA960_JPG				(47 + 32)
#define IRQ_PHY_GMAC					(48 + 32)
#define IRQ_PHY_USB20OTG				(49 + 32)
#define IRQ_PHY_USB20HOST				(50 + 32)
#define IRQ_PHY_CAN0					(51 + 32)
#define IRQ_PHY_CAN1					(52 + 32)
#define IRQ_PHY_GPIOA					(53 + 32)
#define IRQ_PHY_GPIOB					(54 + 32)
#define IRQ_PHY_GPIOC					(55 + 32)
#define IRQ_PHY_GPIOD					(56 + 32)
#define IRQ_PHY_GPIOE					(57 + 32)
#define IRQ_PHY_CRYPTO					(58 + 32)
#define IRQ_PHY_PDM						(59 + 32)
#define IRQ_PHY_TMU0                    (60 + 32)
#define IRQ_PHY_TMU1                    (61 + 32)
#define IRQ_PHY_VIP2					(72 + 32)

#define IRQ_PHY_MAX_COUNT       		(74 + 32) // ADD GIC IRQ

/*
 * GPIO Interrupt Number 160 (106~265)
 */
#define IRQ_GPIO_START			IRQ_PHY_MAX_COUNT
#define IRQ_GPIO_END			(IRQ_GPIO_START + 32 * 5)	// Group: A,B,C,D,E

#define IRQ_GPIO_A_START		(IRQ_GPIO_START + PAD_GPIO_A)
#define IRQ_GPIO_B_START		(IRQ_GPIO_START + PAD_GPIO_B)
#define IRQ_GPIO_C_START		(IRQ_GPIO_START + PAD_GPIO_C)
#define IRQ_GPIO_D_START		(IRQ_GPIO_START + PAD_GPIO_D)
#define IRQ_GPIO_E_START		(IRQ_GPIO_START + PAD_GPIO_E)

/*
 * ALIVE Interrupt Number 6 (266~271)
 */
#define IRQ_ALIVE_START			IRQ_GPIO_END
#define IRQ_ALIVE_END			(IRQ_ALIVE_START + 6)

#define IRQ_ALIVE_0				(IRQ_ALIVE_START + 0)
#define IRQ_ALIVE_1				(IRQ_ALIVE_START + 1)
#define IRQ_ALIVE_2				(IRQ_ALIVE_START + 2)
#define IRQ_ALIVE_3				(IRQ_ALIVE_START + 3)
#define IRQ_ALIVE_4				(IRQ_ALIVE_START + 4)
#define IRQ_ALIVE_5				(IRQ_ALIVE_START + 5)

/*
 * MAX(Physical+Virtual) Interrupt Number
 */
#define IRQ_SYSTEM_END			IRQ_ALIVE_END

#if defined (CONFIG_REGULATOR_NXE2000)
#define	IRQ_RESERVED_OFFSET		72		// refer NXE2000_NR_IRQS <linux/mfd/nxe2000.h>
#else
#define	IRQ_RESERVED_OFFSET		0
#endif
#define IRQ_SYSTEM_RESERVED		IRQ_RESERVED_OFFSET

#define IRQ_TOTAL_MAX_COUNT  	(IRQ_SYSTEM_END + IRQ_SYSTEM_RESERVED)
#endif
#ifndef __ASM_MACH_CLKDEV_H
#define __ASM_MACH_CLKDEV_H

struct clk {
        unsigned int rate;
};


#define __clk_get(clk) ({ 1; })
#define __clk_put(clk) do { } while (0)

/* clk id */
#define CORECLK_NAME_PLL0               "pll0"  /* cpu clock */
#define CORECLK_NAME_PLL1               "pll1"
#define CORECLK_NAME_PLL2               "pll2"
#define CORECLK_NAME_PLL3               "pll3"
#define CORECLK_NAME_FCLK               "fclk"
#define CORECLK_NAME_MCLK               "mclk"
#define CORECLK_NAME_BCLK               "bclk"
#define CORECLK_NAME_PCLK               "pclk"
#define CORECLK_NAME_HCLK               "hclk"

#define CORECLK_ID_PLL0                 0
#define CORECLK_ID_PLL1                 1
#define CORECLK_ID_PLL2                 2
#define CORECLK_ID_PLL3                 3
#define CORECLK_ID_FCLK                 4
#define CORECLK_ID_MCLK                 5
#define CORECLK_ID_BCLK                 6
#define CORECLK_ID_PCLK                 7
#define CORECLK_ID_HCLK                 8

#endif

typedef signed   char   S8;
typedef unsigned char   U8;
typedef signed   short  S16;
typedef unsigned short  U16;


typedef int32_t   S32;
typedef u_int32_t U32;

typedef struct _S64
{
    U32          Low;
    S32          High;
} S64;

typedef struct _U64
{
    U32          Low;
    U32          High;
} U64;


/****************************************************************************/
/*  Pointers                                                                */
/****************************************************************************/

typedef S8      *PS8;
typedef U8      *PU8;
typedef S16     *PS16;
typedef U16     *PU16;
typedef S32     *PS32;
typedef U32     *PU32;
typedef S64     *PS64;
typedef U64     *PU64;

#ifndef __S5P6818_H__
#define __S5P6818_H__

#define PHY_BASEADDR_DMA0     		(0xC0000000)
#define PHY_BASEADDR_DMA1     		(0xC0001000)
#define PHY_BASEADDR_INTC     		(0xC0008000)
#define PHY_BASEADDR_CLKPWR			(0xC0010000)
#define PHY_BASEADDR_RTC			(0xC0010C00)	// ???
#define PHY_BASEADDR_ALIVE			(0xC0010800)	// ???
#define PHY_BASEADDR_RSTCON			(0xC0012000)
#define PHY_BASEADDR_TIEOFF			(0xC0011000)
#define PHY_BASEADDR_PDM			(0xC0014000)	// ???
#define PHY_BASEADDR_CRYPTO			(0xC0015000)
#define PHY_BASEADDR_TIMER			(0xC0017000)
#define PHY_BASEADDR_PWM			(0xC0018000)
#define PHY_BASEADDR_WDT			(0xC0019000)
#define PHY_BASEADDR_GPIOA			(0xC001A000)
#define PHY_BASEADDR_GPIOB			(0xC001B000)
#define PHY_BASEADDR_GPIOC			(0xC001C000)
#define PHY_BASEADDR_GPIOD			(0xC001D000)
#define PHY_BASEADDR_GPIOE			(0xC001E000)
#define PHY_BASEADDR_OHCI			(0xC0020000)
#define PHY_BASEADDR_EHCI			(0xC0030000)
#define PHY_BASEADDR_HSOTG			(0xC0040000)
#define PHY_BASEADDR_ADC			(0xC0053000)
#define PHY_BASEADDR_PPM			(0xC0054000)
#define PHY_BASEADDR_I2S0     		(0xC0055000)
#define PHY_BASEADDR_I2S1     		(0xC0056000)
#define PHY_BASEADDR_I2S2     		(0xC0057000)
#define PHY_BASEADDR_AC97			(0xC0058000)
#define PHY_BASEADDR_SPDIF_TX		(0xC0059000)
#define PHY_BASEADDR_SPDIF_RX		(0xC005A000)
#define PHY_BASEADDR_SSP0			(0xC005B000)
#define PHY_BASEADDR_SSP1			(0xC005C000)
#define PHY_BASEADDR_SSP2			(0xC005F000)
#define PHY_BASEADDR_MPEGTSI		(0xC005D000)
#define PHY_BASEADDR_GMAC			(0xC0060000)
#define PHY_BASEADDR_VIP0			(0xC0063000)
#define PHY_BASEADDR_VIP1			(0xC0064000)
#define PHY_BASEADDR_VIP2			(0xC0099000)
#define PHY_BASEADDR_DEINTERLACE	(0xC0065000)
#define PHY_BASEADDR_SCALER			(0xC0066000)
#define PHY_BASEADDR_ECID			(0xC0067000)
#define PHY_BASEADDR_SDMMC0			(0xC0062000)
#define PHY_BASEADDR_SDMMC1			(0xC0068000)
#define PHY_BASEADDR_SDMMC2			(0xC0069000)
#define PHY_BASEADDR_VR				(0xC0070000)
#define PHY_BASEADDR_CODA_APB0		(0xC0080000)
#define PHY_BASEADDR_CODA_APB1		(0xC0081000)
#define PHY_BASEADDR_CODA_APB2		(0xC0082000)
#define PHY_BASEADDR_CODA_APB3		(0xC0083000)
#define PHY_BASEADDR_UART0         	(0xC00A1000)	// dma (O), modem(X), UART0_MODULE
#define PHY_BASEADDR_UART1         	(0xC00A0000)	// dma (O), modem(O), pl01115_Uart_modem_MODULE
#define PHY_BASEADDR_UART2         	(0xC00A2000)	// dma (O), modem(X), UART1_MODULE
#define PHY_BASEADDR_UART3         	(0xC00A3000)	// dma (X), modem(X), pl01115_Uart_nodma0_MODULE
#define PHY_BASEADDR_UART4         	(0xC006D000)	// dma (X), modem(X), pl01115_Uart_nodma1_MODULE
#define PHY_BASEADDR_UART5         	(0xC006F000)	// dma (X), modem(X), pl01115_Uart_nodma2_MODULE
#define PHY_BASEADDR_I2C0           (0xC00A4000)
#define PHY_BASEADDR_I2C1           (0xC00A5000)
#define PHY_BASEADDR_I2C2           (0xC00A6000)
#define PHY_BASEADDR_CAN0			(0xC00CE000)
#define PHY_BASEADDR_CAN1			(0xC00CF000)
#define PHY_BASEADDR_MIPI			(0xC00D0000)
#define PHY_BASEADDR_DISPLAYTOP		(0xC0100000)

#define PHY_BASEADDR_CLKGEN0		(0xC00BB000)	// TIMER_1
#define PHY_BASEADDR_CLKGEN1		(0xC00BC000)	// TIMER_2
#define PHY_BASEADDR_CLKGEN2		(0xC00BD000)	// TIMER_3
#define PHY_BASEADDR_CLKGEN3		(0xC00BE000)	// PWM_1
#define PHY_BASEADDR_CLKGEN4		(0xC00BF000)	// PWM_2
#define PHY_BASEADDR_CLKGEN5		(0xC00C0000)	// PWM_3
#define PHY_BASEADDR_CLKGEN6		(0xC00AE000)	// I2C_0
#define PHY_BASEADDR_CLKGEN7		(0xC00AF000)	// I2C_1
#define PHY_BASEADDR_CLKGEN8		(0xC00B0000)	// I2C_2
#define PHY_BASEADDR_CLKGEN9		(0xC00CA000)	// MIPI
#define PHY_BASEADDR_CLKGEN10		(0xC00C8000)	// GMAC
#define PHY_BASEADDR_CLKGEN11		(0xC00B8000)	// SPDIF_TX
#define PHY_BASEADDR_CLKGEN12		(0xC00B7000)	// MPEGTSI
#define PHY_BASEADDR_CLKGEN13		(0xC00BA000)	// PWM_0
#define PHY_BASEADDR_CLKGEN14		(0xC00B9000)	// TIMER_0
#define PHY_BASEADDR_CLKGEN15		(0xC00B2000)	// I2S_0
#define PHY_BASEADDR_CLKGEN16		(0xC00B3000)	// I2S_1
#define PHY_BASEADDR_CLKGEN17		(0xC00B4000)	// I2S_2
#define PHY_BASEADDR_CLKGEN18		(0xC00C5000)	// SDHC_0
#define PHY_BASEADDR_CLKGEN19		(0xC00CC000)	// SDHC_1
#define PHY_BASEADDR_CLKGEN20		(0xC00CD000)	// SDHC_2
#define PHY_BASEADDR_CLKGEN21		(0xC00C3000)	// VR
#define PHY_BASEADDR_CLKGEN22		(0xC00A9000)	// UART_0
#define PHY_BASEADDR_CLKGEN23		(0xC00AA000)	// UART_2
#define PHY_BASEADDR_CLKGEN24		(0xC00A8000)	// UART_1
#define PHY_BASEADDR_CLKGEN25		(0xC00AB000)	// UART_3
#define PHY_BASEADDR_CLKGEN26		(0xC006E000)	// UART_4
#define PHY_BASEADDR_CLKGEN27		(0xC00B1000)	// UART_5
#define PHY_BASEADDR_CLKGEN28		(0xC00B5000)	// DEINTERLACE
#define PHY_BASEADDR_CLKGEN29		(0xC00C4000)	// PPM
#define PHY_BASEADDR_CLKGEN30		(0xC00C1000)	// VIP_0
#define PHY_BASEADDR_CLKGEN31		(0xC00C2000)	// VIP_1
#define PHY_BASEADDR_CLKGEN32		(0xC006B000)	// USB2HOST
#define PHY_BASEADDR_CLKGEN33		(0xC00C7000)	// CODA
#define PHY_BASEADDR_CLKGEN34		(0xC00C6000)	// CRYPTO
#define PHY_BASEADDR_CLKGEN35		(0xC00B6000)	// SCALER
#define PHY_BASEADDR_CLKGEN36		(0xC00CB000)	// PDM
#define PHY_BASEADDR_CLKGEN37		(0xC00AC000)	// SPI0
#define PHY_BASEADDR_CLKGEN38		(0xC00AD000)	// SPI1
#define PHY_BASEADDR_CLKGEN39		(0xC00A7000)	// SPI2
#define PHY_BASEADDR_CLKGEN40		(0xC009A000)
#define PHY_BASEADDR_DREX			(0xC00E0000)


#define PHY_BASEADDR_CS_NAND		(0x2C000000)	// ????

	/*
	 * Nexell clock generator
	 */
	#define CLK_ID_TIMER_1			0
	#define CLK_ID_TIMER_2			1
	#define CLK_ID_TIMER_3			2
	#define CLK_ID_PWM_1			3
	#define CLK_ID_PWM_2			4
	#define CLK_ID_PWM_3			5
	#define CLK_ID_I2C_0			6
	#define CLK_ID_I2C_1			7
	#define CLK_ID_I2C_2			8
	#define CLK_ID_MIPI				9
	#define CLK_ID_GMAC				10	/* External Clock 1 */
	#define CLK_ID_SPDIF_TX			11
	#define CLK_ID_MPEGTSI			12
	#define CLK_ID_PWM_0			13
	#define CLK_ID_TIMER_0			14
	#define CLK_ID_I2S_0			15	/* External Clock 1 */
	#define CLK_ID_I2S_1			16	/* External Clock 1 */
	#define CLK_ID_I2S_2			17	/* External Clock 1 */
	#define CLK_ID_SDHC_0			18
	#define CLK_ID_SDHC_1			19
	#define CLK_ID_SDHC_2			20
	#define CLK_ID_VR				21
	#define CLK_ID_UART_0			22	/* UART0_MODULE */
	#define CLK_ID_UART_2			23	/* UART1_MODULE */
	#define CLK_ID_UART_1			24	/* pl01115_Uart_modem_MODULE  */
	#define CLK_ID_UART_3			25	/* pl01115_Uart_nodma0_MODULE */
	#define CLK_ID_UART_4			26	/* pl01115_Uart_nodma1_MODULE */
	#define CLK_ID_UART_5			27	/* pl01115_Uart_nodma2_MODULE */
	#define CLK_ID_DIT				28
	#define CLK_ID_PPM				29
	#define CLK_ID_VIP_0			30	/* External Clock 1 */
	#define CLK_ID_VIP_1			31	/* External Clock 1, 2 */
	#define CLK_ID_USB2HOST			32	/* External Clock 2 */
	#define CLK_ID_CODA				33
	#define CLK_ID_CRYPTO			34
	#define CLK_ID_SCALER			35
	#define CLK_ID_PDM				36
	#define CLK_ID_SPI_0			37
	#define CLK_ID_SPI_1			38
	#define CLK_ID_SPI_2			39
	#define CLK_ID_MAX				39
	#define CLK_ID_USBOTG			40	/* Shared with USB2HOST */

	/*
	 * Nexell Reset control
	 */
	#define	RESET_ID_AC97				0
	#define	RESET_ID_CPU1				1
	#define	RESET_ID_CPU2				2
	#define	RESET_ID_CPU3				3
	#define	RESET_ID_WD1				4
	#define	RESET_ID_WD2				5
	#define	RESET_ID_WD3				6
	#define	RESET_ID_CRYPTO				7
	#define	RESET_ID_DEINTERLACE		8
	#define	RESET_ID_DISP_TOP			9
	#define RESET_ID_DISPLAY			10	/* DualDisplay -: MLC0/1, DPC0/1 */
	#define RESET_ID_RESCONV			11
	#define RESET_ID_LCDIF				12
	#define RESET_ID_HDMI				13
	#define RESET_ID_HDMI_VIDEO			14
	#define RESET_ID_HDMI_SPDIF			15
	#define RESET_ID_HDMI_TMDS			16
	#define RESET_ID_HDMI_PHY			17
	#define RESET_ID_LVDS				18
	#define RESET_ID_ECID				19
	#define RESET_ID_I2C0				20
	#define RESET_ID_I2C1				21
	#define RESET_ID_I2C2				22
	#define RESET_ID_I2S0				23
	#define RESET_ID_I2S1				24
	#define RESET_ID_I2S2				25
	#define RESET_ID_DREX_C				26
	#define RESET_ID_DREX_A				27
	#define RESET_ID_DREX				28
	#define RESET_ID_MIPI				29
	#define RESET_ID_MIPI_DSI			30
	#define RESET_ID_MIPI_CSI			31
	#define RESET_ID_MIPI_PHY_S			32
	#define RESET_ID_MIPI_PHY_M			33
	#define	RESET_ID_MPEGTSI			34
	#define RESET_ID_PDM				35
	#define RESET_ID_TIMER				36
	#define RESET_ID_PWM				37
	#define RESET_ID_SCALER				38
	#define RESET_ID_SDMMC0				39
	#define RESET_ID_SDMMC1				40
	#define RESET_ID_SDMMC2				41
	#define RESET_ID_SPDIFRX			42
	#define RESET_ID_SPDIFTX			43
	#define	RESET_ID_SSP0_P				44
	#define RESET_ID_SSP0				45
	#define RESET_ID_SSP1_P				46
	#define RESET_ID_SSP1				47
	#define RESET_ID_SSP2_P				48
	#define RESET_ID_SSP2				49
	#define RESET_ID_UART0				50	/* UART1 */
	#define RESET_ID_UART1				51	/* pl01115_Uart_modem	*/
	#define RESET_ID_UART2				52	/* UART1 */
	#define RESET_ID_UART3				53	/* pl01115_Uart_nodma0 */
	#define RESET_ID_UART4				54	/* pl01115_Uart_nodma1 */
	#define RESET_ID_UART5				55	/* pl01115_Uart_nodma2 */
	#define RESET_ID_USB20HOST			56
	#define RESET_ID_USB20OTG			57
	#define RESET_ID_WDT				58
	#define RESET_ID_WDT_POR			59
	#define RESET_ID_ADC				60
	#define RESET_ID_CODA_A				61
	#define RESET_ID_CODA_P				62
	#define RESET_ID_CODA_C				63
	#define RESET_ID_DWC_GMAC			64
	#define RESET_ID_VR					65
	#define RESET_ID_PPM				66
	#define RESET_ID_VIP1				67
	#define RESET_ID_VIP0               68

	/* redefine */
	#define TIEOFF_UART0_USERSMC        TIEOFFINDEX_OF_UART0_USESMC
	#define TIEOFF_UART0_SMCTXENB       TIEOFFINDEX_OF_UART0_SMCTXENB
	#define TIEOFF_UART0_SMCRXENB       TIEOFFINDEX_OF_UART0_SMCRXENB
	#define TIEOFF_UART1_USERSMC        TIEOFFINDEX_OF_UART1_USESMC
	#define TIEOFF_UART1_SMCTXENB       TIEOFFINDEX_OF_UART1_SMCTXENB
	#define TIEOFF_UART1_SMCRXENB       TIEOFFINDEX_OF_UART1_SMCRXENB
	#define TIEOFF_UART2_USERSMC        TIEOFFINDEX_OF_UART2_USESMC
	#define TIEOFF_UART2_SMCTXENB       TIEOFFINDEX_OF_UART2_SMCTXENB
	#define TIEOFF_UART2_SMCRXENB       TIEOFFINDEX_OF_UART2_SMCRXENB
	#define TIEOFF_UART3_USERSMC        TIEOFFINDEX_OF_UART3_USESMC
	#define TIEOFF_UART3_SMCTXENB       TIEOFFINDEX_OF_UART3_SMCTXENB
	#define TIEOFF_UART3_SMCRXENB       TIEOFFINDEX_OF_UART3_SMCRXENB
	#define TIEOFF_UART4_USERSMC        TIEOFFINDEX_OF_UART4_USESMC
	#define TIEOFF_UART4_SMCTXENB       TIEOFFINDEX_OF_UART4_SMCTXENB
	#define TIEOFF_UART4_SMCRXENB       TIEOFFINDEX_OF_UART4_SMCRXENB
	#define TIEOFF_UART5_USERSMC        TIEOFFINDEX_OF_UART5_USESMC
	#define TIEOFF_UART5_SMCTXENB       TIEOFFINDEX_OF_UART5_SMCTXENB
	#define TIEOFF_UART5_SMCRXENB       TIEOFFINDEX_OF_UART5_SMCRXENB

/*
 * Interrupt
 */
/*#include "s5p6818_irq.h"
*/
/*
 * prototype header
 */
/*#ifndef __ASSEMBLY__

#include <nx_clkpwr.h>
#include <nx_clkgen.h>
#include <nx_rstcon.h>
#include <nx_tieoff.h>
#include <nx_mcus.h>
#include <nx_timer.h>
#include <nx_alive.h>
#include <nx_gpio.h>
#include <nx_displaytop.h>
#include <nx_disptop_clkgen.h>
#include <nx_dualdisplay.h>
#include <nx_mlc.h>
#include <nx_dpc.h>
#include <nx_lcdif.h>
#include <nx_resconv.h>
#include <nx_lvds.h>
#include <nx_hdmi.h>
#include <nx_mipi.h>
#include <nx_i2c.h>
#include <nx_i2s.h>
#include <nx_sdmmc.h>
#include <nx_ssp.h>
#include <nx_rtc.h>
#include <nx_mpegtsi.h>
#include <nx_vip.h>
#include <nx_adc.h>
#include <nx_ecid.h>
#include <nx_tmu.h>

#endif*/ /* 	__ASSEMBLY__ */
#endif /*	__S5P6818_H__ */

//------------------------------------------------------------------------------
// PLL input crystal
//------------------------------------------------------------------------------
#define CFG_SYS_PLLFIN							24000000UL

/*------------------------------------------------------------------------------
 * 	System Name
 */
#define	CFG_SYS_CPU_NAME						"s5p6818"
#define	CFG_SYS_BOARD_NAME						"GEC6818"

/*------------------------------------------------------------------------------
 * 	BUS config
 */
#define CFG_BUS_RECONFIG_ENB					1		/* if want bus reconfig, select this first */

#define CFG_BUS_RECONFIG_DREXQOS				0
#define CFG_BUS_RECONFIG_TOPBUSSI				0
#define CFG_BUS_RECONFIG_BOTTOMBUSSI			0
#define CFG_BUS_RECONFIG_BOTTOMBUSQOS			0
#define CFG_BUS_RECONFIG_DISPBUSSI				1

/*------------------------------------------------------------------------------
 * 	Uart
 */
#define CFG_UART_DEBUG_CH						0	/* For Low level debug */
#define	CFG_UART_DEBUG_BAUDRATE					115200
#define	CFG_UART_CLKGEN_CLOCK_HZ				50000000	/* 50000000 */

/*------------------------------------------------------------------------------
 * 	Timer List (SYS = Source, EVT = Event, WDT = WatchDog)
 */
#define	CFG_TIMER_SYS_TICK_CH					0
#define	CFG_TIMER_EVT_TICK_CH					1

/*------------------------------------------------------------------------------
 * 	Extern Ethernet
 */
#define CFG_ETHER_EXT_PHY_BASEADDR          	0x04000000	// DM9000: CS1
#define	CFG_ETHER_EXT_IRQ_NUM					(IRQ_GPIO_C_START + 26)

/*------------------------------------------------------------------------------
 * 	GMAC PHY
 */

//#define CFG_ETHER_LOOPBACK_MODE					0	/* 0: disable, 1: 10M, 2: 100M(x), 3: 1000M(x) */

/*for rtl8211*/	//hdc 20150430
#define	CFG_ETHER_GMAC_PHY_IRQ_NUM				(IRQ_GPIO_E_START + 23)
#define	CFG_ETHER_GMAC_PHY_RST_NUM				(PAD_GPIO_E + 22)

/*------------------------------------------------------------------------------
 * 	Nand (HWECC)
 */
#define CFG_NAND_ECC_BYTES 						1024
#define CFG_NAND_ECC_BITS               		40			/* 512 - 4,8,16,24 1024 - 24,40,60 */
//#define CFG_NAND_ECCIRQ_MODE

/*------------------------------------------------------------------------------
 *	Nand (GPIO)
 */
#define CFG_IO_NAND_nWP							(PAD_GPIO_C + 27)		/* GPIO */

/*------------------------------------------------------------------------------
 * 	Display (DPC and MLC)
 */
/*extern int CFG_DISP_PRI_SCREEN_LAYER;
extern int CFG_DISP_PRI_SCREEN_RGB_FORMAT;
extern int CFG_DISP_PRI_SCREEN_PIXEL_BYTE;
extern int CFG_DISP_PRI_SCREEN_COLOR_KEY;
extern int CFG_DISP_PRI_VIDEO_PRIORITY;
extern int CFG_DISP_PRI_BACK_GROUND_COLOR;
extern int CFG_DISP_PRI_MLC_INTERLACE;

extern int CFG_DISP_PRI_LCD_WIDTH_MM;
extern int CFG_DISP_PRI_LCD_HEIGHT_MM;
extern int CFG_DISP_PRI_RESOL_WIDTH;
extern int CFG_DISP_PRI_RESOL_HEIGHT;
extern int CFG_DISP_PRI_HSYNC_SYNC_WIDTH;
extern int CFG_DISP_PRI_HSYNC_BACK_PORCH;
extern int CFG_DISP_PRI_HSYNC_FRONT_PORCH;
extern int CFG_DISP_PRI_HSYNC_ACTIVE_HIGH;
extern int CFG_DISP_PRI_VSYNC_SYNC_WIDTH;
extern int CFG_DISP_PRI_VSYNC_BACK_PORCH;
extern int CFG_DISP_PRI_VSYNC_FRONT_PORCH;
extern int CFG_DISP_PRI_VSYNC_ACTIVE_HIGH;

extern int CFG_DISP_PRI_CLKGEN0_SOURCE;
extern int CFG_DISP_PRI_CLKGEN0_DIV;
extern int CFG_DISP_PRI_CLKGEN0_DELAY;
extern int CFG_DISP_PRI_CLKGEN0_INVERT;
extern int CFG_DISP_PRI_CLKGEN1_SOURCE;
extern int CFG_DISP_PRI_CLKGEN1_DIV;
extern int CFG_DISP_PRI_CLKGEN1_DELAY;
extern int CFG_DISP_PRI_CLKGEN1_INVERT;
extern int CFG_DISP_PRI_CLKSEL1_SELECT;
extern int CFG_DISP_PRI_PADCLKSEL;
extern int CFG_DISP_PRI_PIXEL_CLOCK;
extern int CFG_DISP_PRI_OUT_SWAPRB;
extern int CFG_DISP_PRI_OUT_FORMAT;
extern int CFG_DISP_PRI_OUT_YCORDER;
extern int CFG_DISP_PRI_OUT_INTERLACE;
extern int CFG_DISP_PRI_OUT_INVERT_FIELD;
extern int CFG_DISP_LCD_MPY_TYPE;
extern int CFG_DISP_LVDS_LCD_FORMAT;
extern int CFG_DISP_HDMI_USING;

extern int CFG_DISP_MIPI_PLLPMS;
extern int CFG_DISP_MIPI_BANDCTL;
extern int CFG_DISP_MIPI_PLLCTL;
extern int CFG_DISP_MIPI_DPHYCTL;
extern struct mipi_reg_val * CFG_DISP_MIPI_INIT_DATA;
*/
/*------------------------------------------------------------------------------
 * 	PWM
 */
#define CFG_LCD_PRI_PWM_CH						0
#define CFG_LCD_PRI_PWM_FREQ					10000
#define CFG_LCD_PRI_PWM_DUTYCYCLE				50		/* (%) */

#define CFG_PWM1_CLK_SRC						1		// 0:pclk 1:tclk
#define CFG_PWM3_CLK_SRC						1		// 0:pclk 1:tclk

/*------------------------------------------------------------------------------
 * 	PPM
 */
#define CFG_PPM_CLK                     4000000

/*------------------------------------------------------------------------------
 * 	Audio I2S (0, 1, 2)
 */
//#define CFG_AUDIO_I2S_SUPPLY_EXT_MCLK			1
#define CFG_EXT_MCLK_PWM_CH						3

#define	CFG_AUDIO_I2S0_MASTER_MODE				CTRUE	// CTRUE
#define	CFG_AUDIO_I2S0_TRANS_MODE				0		// 0:I2S, 1:Left 2:Right justified */
#define	CFG_AUDIO_I2S0_FRAME_BIT				32		// 32, 48
#define	CFG_AUDIO_I2S0_SAMPLE_RATE				48000
#define	CFG_AUDIO_I2S0_PRE_SUPPLY_MCLK			1

#define	CFG_AUDIO_I2S1_MASTER_MODE				CTRUE	// CTRUE
#define	CFG_AUDIO_I2S1_TRANS_MODE				0		// 0:I2S, 1:Left 2:Right justified */
#define	CFG_AUDIO_I2S1_FRAME_BIT				48		// 32, 48
#define	CFG_AUDIO_I2S1_SAMPLE_RATE				48000
#define	CFG_AUDIO_I2S1_PRE_SUPPLY_MCLK			0

#define	CFG_AUDIO_I2S2_MASTER_MODE				CTRUE	// CTRUE
#define	CFG_AUDIO_I2S2_TRANS_MODE				0		// 0:I2S, 1:Left 2:Right justified */
#define	CFG_AUDIO_I2S2_FRAME_BIT				48		// 32, 48
#define	CFG_AUDIO_I2S2_SAMPLE_RATE				48000
#define	CFG_AUDIO_I2S2_PRE_SUPPLY_MCLK			0

/*------------------------------------------------------------------------------
 * 	Audio SPDIF (TX/RX)
 */
#define	CFG_AUDIO_SPDIF_TX_HDMI_OUT					CTRUE
#define	CFG_AUDIO_SPDIF_TX_SAMPLE_RATE				48000
#define	CFG_AUDIO_SPDIF_RX_SAMPLE_RATE				48000

/*------------------------------------------------------------------------------
 * 	I2C
 */
//#define CFG_I2C0_CLK							100000
#define CFG_I2C0_CLK							20000
#define CFG_I2C1_CLK							200000	/* TOUCH */
#define CFG_I2C2_CLK							200000
#define CFG_I2C3_CLK							100000

/*------------------------------------------------------------------------------
 * 	SPI
 */
#define CFG_SPI0_CLK							10000000
#define CFG_SPI1_CLK							10000000
#define CFG_SPI2_CLK							10000000

#define CFG_SPI0_COM_MODE						0 /* available 0: INTERRUPT_TRANSFER, 1: POLLING_TRANSFER, 2: DMA_TRANSFER */
#define CFG_SPI1_COM_MODE						1 /* available 0: INTERRUPT_TRANSFER, 1: POLLING_TRANSFER, 2: DMA_TRANSFER */
#define CFG_SPI2_COM_MODE						1 /* available 0: INTERRUPT_TRANSFER, 1: POLLING_TRANSFER, 2: DMA_TRANSFER */

#define CFG_SPI0_CS_GPIO_MODE					1		/* 0 FSS CONTROL, 1: CS CONTRO GPIO MODE */
#define CFG_SPI1_CS_GPIO_MODE					1		/* 0 FSS CONTROL, 1: CS CONTRO GPIO MODE */
#define CFG_SPI2_CS_GPIO_MODE					0	/* 0 FSS CONTROL, 1: CS CONTRO GPIO MODE */

#define CFG_SPI0_CS							PAD_GPIO_C + 30	/* 0 FSS CONTROL, 1: CS CONTRO GPIO MODE */
/*------------------------------------------------------------------------------
 *  MPEGTSIF
 */
#define CFG_MPEGTS_MASTER_MODE					1 /* 0: slave, 1: master */
#define CFG_MPEGTS_SLAVE_MODE					0 /* 0: slave, 1: master */
#define CFG_MPEGTS_CLOCKPOL						1 /* 0: falling, 1: rising */
#define CFG_MPEGTS_DATAPOL						1 /* 0: data is low, 1: data is high */
#define CFG_MPEGTS_SYNCPOL						1 /* 0: falling, 1: rising */
#define CFG_MPEGTS_ERRORPOL						1 /* 0: falling, 1: rising */
#define CFG_MPEGTS_DATAWIDTH					0 /* 0: 8bit, 1: 1bit */
#define CFG_MPEGTS_WORDCNT						47 /* 1 ~ 64 */

/*------------------------------------------------------------------------------
 * 	Keypad
 */
#define CFG_KEYPAD_KEY_BUTTON					{ PAD_GPIO_B + 31, PAD_GPIO_B + 30, PAD_GPIO_ALV + 0, PAD_GPIO_B + 9, PAD_GPIO_A + 28}
#define CFG_KEYPAD_KEY_CODE						{ KEY_VOLUMEDOWN, KEY_VOLUMEUP, KEY_POWER, KEY_MENU, KEY_BACK}
#define CFG_KEYPAD_REPEAT						CFALSE /* 0: Repeat Off 1 : Repeat On */

/*------------------------------------------------------------------------------
 * 	SDHC
 */
#define	CFG_SDMMC0_DETECT_IO					(PAD_GPIO_ALV + 1)	/* external sd 0 */
#define	CFG_SDMMC1_DETECT_IO					(PAD_GPIO_B + 25)	/* external sd 1 */


/*------------------------------------------------------------------------------
 * 	PMIC
 */
/* AXP228 PMIC	*/
#define CFG_SW_UBC_ENABLE						(1)					/* S/W UBC Check */
#define CFG_USB_DET_FROM_PMIC_INT			(0)				/* 0 : GPIO interrupt (CFG_GPIO_PMIC_VUSB_DET)		1 : PMIC interrupt (FVUSBDETSINT) */
#define CFG_GPIO_OTG_USBID_DET				(-1)					/* USB ID Deteict */
#define CFG_GPIO_OTG_VBUS_DET					(PAD_GPIO_C + 28)	/* USB OTG Power Enable */
#define CFG_GPIO_PMIC_VUSB_DET				(PAD_GPIO_ALV + 2)	/* Choice for SW_UBC or Wake-up*/
#define CFG_GPIO_PMIC_LOWBAT_DET				(-1)					/* Critical low battery detect */
#define CFG_PMIC_BAT_CHG_SUPPORT				(1)

/* AXP228 PMIC	*/
#define	CFG_PMIC_I2_CBUS						3					/* i2c channel */
#define CFG_BATTERY_CAP							3000					/* Battery Capacity */

/* PMIC Common*/
#define CFG_GPIO_PMIC_INTR						(PAD_GPIO_ALV + 4)	/* PMIC Interrupt */


/*------------------------------------------------------------------------------
 * 	Suspend mode
 */

/* Wakeup Source : ALIVE [0~7] */
#define CFG_PWR_WAKEUP_SRC_ALIVE0				CTRUE					/* KEY */
#define CFG_PWR_WAKEUP_MOD_ALIVE0				PWR_DECT_FALLINGEDGE
#define CFG_PWR_WAKEUP_SRC_ALIVE1				CFALSE					/* External SD */
#define CFG_PWR_WAKEUP_MOD_ALIVE1				PWR_DECT_BOTHEDGE
#define CFG_PWR_WAKEUP_SRC_ALIVE2				CFALSE					/* PMIC - VUSB */
#define CFG_PWR_WAKEUP_MOD_ALIVE2				PWR_DECT_BOTHEDGE
#define CFG_PWR_WAKEUP_SRC_ALIVE3				CFALSE					/* PMIC - CRITICAL LOW BATTERY */
#define CFG_PWR_WAKEUP_MOD_ALIVE3				PWR_DECT_ASYNC_LOWLEVEL
#define CFG_PWR_WAKEUP_SRC_ALIVE4				CFALSE					/* PMIC INTR */
#define CFG_PWR_WAKEUP_MOD_ALIVE4				PWR_DECT_FALLINGEDGE
#define CFG_PWR_WAKEUP_SRC_ALIVE5				CFALSE
#define CFG_PWR_WAKEUP_MOD_ALIVE5				PWR_DECT_FALLINGEDGE

/*
 * Wakeup Source : RTC ALARM
 * ifndef Enable ALARM Wakeup
 */
#define	CFG_PWR_WAKEUP_SRC_ALARM				CFALSE

//------------------------------------------------------------------------------
// Static Bus #0 ~ #9, NAND, IDE configuration
//------------------------------------------------------------------------------
//	_BW	  : Staic Bus width for Static #0 ~ #9            : 8 or 16
//
//	_TACS : adress setup time before chip select          : 0 ~ 15
//	_TCOS : chip select setup time before nOE is asserted : 0 ~ 15
//	_TACC : access cycle                                  : 1 ~ 256
//	_TSACC: burst access cycle for Static #0 ~ #9 & IDE   : 1 ~ 256
//	_TOCH : chip select hold time after nOE not asserted  : 0 ~ 15
//	_TCAH : address hold time after nCS is not asserted   : 0 ~ 15
//
//	_WAITMODE : wait enable control for Static #0 ~ #9 & IDE : 1=disable, 2=Active High, 3=Active Low
//	_WBURST	  : burst write mode for Static #0 ~ #9          : 0=disable, 1=4byte, 2=8byte, 3=16byte
//	_RBURST   : burst  read mode for Static #0 ~ #9          : 0=disable, 1=4byte, 2=8byte, 3=16byte
//
//------------------------------------------------------------------------------
#define CFG_SYS_STATICBUS_CONFIG( _name_, bw, tACS, tCOS, tACC, tSACC, tCOH, tCAH, wm, rb, wb )	\
	enum {											\
		CFG_SYS_ ## _name_ ## _BW		= bw,		\
		CFG_SYS_ ## _name_ ## _TACS		= tACS,		\
		CFG_SYS_ ## _name_ ## _TCOS		= tCOS,		\
		CFG_SYS_ ## _name_ ## _TACC		= tACC,		\
		CFG_SYS_ ## _name_ ## _TSACC	= tSACC,	\
		CFG_SYS_ ## _name_ ## _TCOH		= tCOH,		\
		CFG_SYS_ ## _name_ ## _TCAH		= tCAH,		\
		CFG_SYS_ ## _name_ ## _WAITMODE	= wm, 		\
		CFG_SYS_ ## _name_ ## _RBURST	= rb, 		\
		CFG_SYS_ ## _name_ ## _WBURST	= wb		\
	};

//                      ( _name_ , bw, tACS tCOS tACC tSACC tOCH tCAH, wm, rb, wb )
CFG_SYS_STATICBUS_CONFIG( STATIC0 ,  8,    1,   1,   6,    6,   1,   1,  1,  0,  0 )		// 0x0000_0000
CFG_SYS_STATICBUS_CONFIG( STATIC1 ,  8,    6,   6,  32,   32,   6,   6,  1,  0,  0 )		// 0x0400_0000
CFG_SYS_STATICBUS_CONFIG(    NAND ,  8,    0,   3,   9,    1,   3,   0,  1,  0,  0 )		// 0x2C00_0000, tOCH, tCAH must be greter than 0

/*
#define pr_debug 	printk
*/
#define pr_debug 	printk

#define	TIMER_CLOCK_SOURCE_HZ	(10*1000000)	/* 1MHZ */
#define	TIMER_CLOCK_EVENT_HZ	(10*1000000)	/* 1MHZ */

/*
 * Timer HW
 */
#define	TIMER_CFG0		(0x00)
#define	TIMER_CFG1		(0x04)
#define	TIMER_TCON		(0x08)
#define	TIMER_CNTB		(0x0C)
#define	TIMER_CMPB		(0x10)
#define	TIMER_CNTO		(0x14)
#define	TIMER_STAT		(0x44)

#define	TCON_AUTO		(1<<3)
#define	TCON_INVT		(1<<2)
#define	TCON_UP			(1<<1)
#define	TCON_RUN		(1<<0)
#define CFG0_CH(ch)		(ch == 0 || ch == 1 ? 0 : 8)
#define CFG1_CH(ch)		(ch * 4)
#define TCON_CH(ch)		(ch ? ch * 4  + 4 : 0)
#define TINT_CH(ch)		(ch)
#define TINT_CS_CH(ch)	(ch + 5)
#define	TINT_CS_MASK	(0x1F)
#define TIMER_CH_OFFS	(0xC)

#define	TIMER_BASE		IO_ADDRESS(PHY_BASEADDR_TIMER)
#define	TIMER_READ(ch)	(readl(TIMER_BASE + TIMER_CNTO + (TIMER_CH_OFFS * ch)))

int __timer_sys_mux_val = 0;
int __timer_sys_scl_val = 0;
int __timer_sys_clk_clr = 0;

#if	  (CFG_TIMER_SYS_TICK_CH == 0)
#define	TIMER_SYS_CLKGEN		IO_ADDRESS(PHY_BASEADDR_CLKGEN14)
#elif (CFG_TIMER_SYS_TICK_CH == 1)
#define	TIMER_SYS_CLKGEN		IO_ADDRESS(PHY_BASEADDR_CLKGEN0)
#elif (CFG_TIMER_SYS_TICK_CH == 2)
#define	TIMER_SYS_CLKGEN		IO_ADDRESS(PHY_BASEADDR_CLKGEN1)
#elif (CFG_TIMER_SYS_TICK_CH == 3)
#define	TIMER_SYS_CLKGEN		IO_ADDRESS(PHY_BASEADDR_CLKGEN2)
#endif
#define	CLKGEN_ENB		(0x0)
#define	CLKGEN_CLR		(0x4)

#define IP_RESET_REGISTER_0 IO_ADDRESS(0xc0012000)
#define IP_RESET_REGISTER_1 IO_ADDRESS(0xc0012004)
#define IP_RESET_REGISTER_2 IO_ADDRESS(0xc0012008)
unsigned in32(unsigned addr)
{
	return (*((volatile unsigned*)addr));
}
void out32(unsigned addr, unsigned data)
{
	(*((volatile unsigned*)addr)) = data;
}
void sr32(unsigned addr, unsigned start_bit, unsigned bit_num, unsigned data)
{
	unsigned mask = ~(((1<<bit_num) - 1)<<start_bit);
	out32(addr, ((in32(addr))&mask) | (data<<start_bit));
}
static inline void timer_reset(int ch)
{
/*	if (!nxp_soc_peri_reset_status(RESET_ID_TIMER))
		nxp_soc_peri_reset_set(RESET_ID_TIMER);*/
	/*关闭复位状态*/
	sr32(IP_RESET_REGISTER_1, 3, 2, 0x3);
	sr32(IP_RESET_REGISTER_1, 3, 2, 0x3);
	sr32(IP_RESET_REGISTER_1, 3, 2, 0x3);	
}

static inline void timer_clock(int ch, int mux, int scl)
{
	volatile U32 val;

	val  = readl(TIMER_BASE + TIMER_CFG0);
	val &= ~(0xFF   << CFG0_CH(ch));
	val |=  ((scl-1)<< CFG0_CH(ch));
	writel(val, TIMER_BASE + TIMER_CFG0);

	val  = readl(TIMER_BASE + TIMER_CFG1);
	val &= ~(0xF << CFG1_CH(ch));
	val |=  (mux << CFG1_CH(ch));
	writel(val, TIMER_BASE + TIMER_CFG1);
}

static inline void timer_count(int ch, unsigned int cnt)
{
	writel((cnt-1), TIMER_BASE + TIMER_CNTB + (TIMER_CH_OFFS * ch));
	writel((cnt-1), TIMER_BASE + TIMER_CMPB + (TIMER_CH_OFFS * ch));
}

static inline void timer_start(int ch, int irqon)
{
	volatile U32 val;
	int on = irqon ? 1 : 0;
//0xC0017000 + 0x44
	val  = readl(TIMER_BASE + TIMER_STAT);
	val &= ~(TINT_CS_MASK<<5 | 0x1 << TINT_CH(ch));
	val |=  (0x1 << TINT_CS_CH(ch) | on << TINT_CH(ch));
	writel(val, TIMER_BASE + TIMER_STAT);

	val = readl(TIMER_BASE + TIMER_TCON);
	val &= ~(0xE << TCON_CH(ch));
	val |=  (TCON_UP << TCON_CH(ch));
	writel(val, TIMER_BASE + TIMER_TCON);

	val &= ~(TCON_UP << TCON_CH(ch));
	val |=  ((TCON_AUTO | TCON_RUN)  << TCON_CH(ch));
	writel(val, TIMER_BASE + TIMER_TCON);
}

static inline void timer_stop(int ch, int irqon)
{
	volatile U32 val;
	int on = irqon ? 1 : 0;

	val  = readl(TIMER_BASE + TIMER_STAT);
	val &= ~(TINT_CS_MASK<<5 | 0x1 << TINT_CH(ch));
	val |=  (0x1 << TINT_CS_CH(ch) | on << TINT_CH(ch));
	writel(val, TIMER_BASE + TIMER_STAT);

	val  = readl(TIMER_BASE + TIMER_TCON);
	val &= ~(TCON_RUN << TCON_CH(ch));
	writel(val, TIMER_BASE + TIMER_TCON);
}

static inline void timer_irq_clear(int ch)
{
	volatile U32 val;
	val  = readl(TIMER_BASE + TIMER_STAT);
	val &= ~(TINT_CS_MASK<<5);
	val |= (0x1 << TINT_CS_CH(ch));
	writel(val, TIMER_BASE + TIMER_STAT);
}

struct timer_info {
	int channel;
	int	irqno;
	struct clk *clk;
	unsigned long rate;
	int tmmux;
	int prescale;
	unsigned long tcount;
	unsigned long rcount;
	int in_tclk;
};

static struct timer_info 	timer_src = { 0, };
static struct timer_info 	timer_evt = { 0, };
#define	tm_source_info()	(&timer_src)
#define	tm_event_info()		(&timer_evt)

/*
 * Timer clock source
 */
static void timer_clock_select(struct timer_info *info, long frequency)
{
	struct clk *clk = NULL;
	char name[16] = CORECLK_NAME_PCLK;
	ulong rate, tout = 0;
	int tscl = 0, tmux = 5;
	int vers = 1;//nxp_cpu_version();

	pr_debug("%s\n", __func__);
#if 0
#if !defined(CONFIG_NXP_DFS_BCLK)
	int smux = 0, pscl = 0;
	ulong mout;
	ulong thz, delt = (-1UL);

	/* PCLK */
	pr_debug("%s:pclk clk_get\n", __func__);
	info->clk = clk_get(NULL, name);
	pr_debug("%s:pclk clk_get_rate\n", __func__);
   	rate = clk_get_rate(info->clk);
   	for (smux = 0; 5 > smux; smux++) {
   		mout = rate/(1<<smux), pscl = mout/frequency;
   		thz  = mout/(pscl?pscl:1);
   		if (!(mout % frequency) && 256 > pscl) {
   			tout = thz, tmux = smux, tscl = pscl;
   			break;
   		}
		if (pscl > 256)
			continue;
		if (abs(frequency-thz) >= delt)
			continue;
		tout = thz, tmux = smux, tscl = pscl;
		delt = abs(frequency-thz);
   	}
#endif

	/* CLKGEN */
	if (vers && tout != frequency) {
		sprintf(name, "%s.%d", DEV_NAME_TIMER, info->channel);
		pr_debug("%s:clk_get\n", __func__);
		clk  = clk_get(NULL, name);
		pr_debug("%s:clk_round_rate\n", __func__);
		rate = clk_round_rate(clk, frequency);
		if (abs(frequency-tout) >= abs(frequency-rate)) {
			tout = clk_set_rate(clk, rate);
			tmux = 5, tscl = 1;
			info->clk = clk, info->in_tclk = 1;
			clk_enable(info->clk);
		} else {
			clk_put(clk);
			rate = clk_get_rate(info->clk);	/* PCLK */
		}
	}
#endif
	info->tmmux = tmux;
	info->prescale = tscl;
	info->tcount = tout/HZ;
	info->rate = tout;
	info->rate = 1000;

	pr_debug("%s (ch:%d, mux=%d, scl=%d, rate=%ld, %s)\n",
		__func__, info->channel, tmux, tscl, tout, info->in_tclk?"TCLK":"PCLK");
	pr_debug("%s end\n", __func__);
}

static void timer_source_suspend(struct clocksource *cs)
{
	struct timer_info *info = tm_source_info();
	int ch = info->channel;

	info->rcount = (info->tcount - TIMER_READ(ch));
	timer_stop(ch, 0);
}

static void timer_source_resume(struct clocksource *cs)
{
	struct timer_info *info = tm_source_info();
	int ch = info->channel;
	ulong flags;

	local_irq_save(flags);

	if (info->in_tclk) {
		clk_set_rate(info->clk, info->rate);
		clk_enable(info->clk);
	}

	timer_reset(ch);
	timer_stop (ch, 0);
	timer_clock(ch, info->tmmux, info->prescale);
	timer_count(ch, info->rcount + 1);	/* restore count */
	timer_start(ch, 0);
	timer_count(ch, info->tcount + 1);	/* next count */

	local_irq_restore(flags);
}

static cycle_t timer_source_read(struct clocksource *cs)
{
	struct timer_info *info = tm_source_info();
	int ch = info->channel;

	info->rcount = (info->tcount - TIMER_READ(ch));
	return (cycle_t)info->rcount;
}

static struct clocksource tm_source_clk = {
	.name 		= "source timer",
 	.rating		= 300,
 	.read		= timer_source_read,
	.mask		= CLOCKSOURCE_MASK(32),
 	.shift 		= 20,
	.flags		= CLOCK_SOURCE_IS_CONTINUOUS,
	.suspend	= timer_source_suspend,
	.resume		= timer_source_resume,
};
void delay(int i)
{
	volatile int j = i;
	while(j--);
}
static int __init timer_source_init(int ch)
{
	struct clocksource *cs = &tm_source_clk;
	struct timer_info *info = tm_source_info();

	pr_debug("%s\n", __func__);
	info->channel = ch;
	info->irqno = -1;
	timer_clock_select(info, TIMER_CLOCK_SOURCE_HZ);

	/*
	 * register timer source
	 */
	clocksource_register_hz(cs, info->rate);

	pr_debug("timer.%d: source shift =%u  \n", ch, cs->shift);
	pr_debug("timer.%d: source mult  =%u  \n", ch, cs->mult);

	/*
	 * source timer run
	 */
	info->tcount = -1UL;
	timer_reset(ch);
	timer_stop (ch, 0);
	//timer_clock(ch, info->tmmux, info->prescale);
	//timer_count(ch, info->tcount + 1);
	timer_count(ch, 1000 + 1);
	timer_start(ch, 0);
	
	while(1)
	{
		static int i = 0;
		printk("%d\n", i++);
		delay(0xffffff);
		printk("addr=%x, tcon:%x\n", TIMER_BASE + TIMER_TCON + (TIMER_CH_OFFS * ch),readl(TIMER_BASE + TIMER_TCON + (TIMER_CH_OFFS * ch)));		
		printk("addr=%x, cntb:%x\n", TIMER_BASE + TIMER_CNTB + (TIMER_CH_OFFS * ch),readl(TIMER_BASE + TIMER_CNTB + (TIMER_CH_OFFS * ch)));		
		printk("addr=%x, cmpb:%x\n", TIMER_BASE + TIMER_CMPB + (TIMER_CH_OFFS * ch),readl(TIMER_BASE + TIMER_CMPB + (TIMER_CH_OFFS * ch)));		
		printk("addr=%x, cnto:%x\n", TIMER_BASE + TIMER_CNTO + (TIMER_CH_OFFS * ch),readl(TIMER_BASE + TIMER_CNTO + (TIMER_CH_OFFS * ch)));		
		printk("addr=%x, tcount:%x\n", TIMER_BASE + TIMER_CNTB + (TIMER_CH_OFFS * ch),info->tcount + 1);		
	}

	__timer_sys_mux_val = info->tmmux;
	__timer_sys_scl_val = info->prescale;
	__timer_sys_clk_clr = readl(TIMER_SYS_CLKGEN + CLKGEN_CLR);
	printk("timer.%d: source, %9lu(HZ:%d), mult:%u\n", ch, info->rate, HZ, cs->mult);
	pr_debug("%s done\n", __func__);
 	return 0;
}

/*
 * Timer clock event
 */
static inline void timer_event_resume(struct timer_info *info)
{
	int ch = info->channel;
	if (info->in_tclk) {
		clk_set_rate(info->clk, info->rate);
		clk_enable(info->clk);
	}
	timer_stop(ch, 1);
	timer_clock(ch, info->tmmux, info->prescale);
}

static void timer_event_set_mode(enum clock_event_mode mode, struct clock_event_device *evt)
{
	struct timer_info *info = tm_event_info();
	int ch = info->channel;
	unsigned long cnt = info->tcount;
	pr_debug("%s (ch:%d, mode:0x%x, cnt:%ld)\n", __func__, ch, mode, cnt);

	switch(mode) {
	case CLOCK_EVT_MODE_UNUSED:		// 0x0
	case CLOCK_EVT_MODE_ONESHOT:	// 0x3
		break;
	case CLOCK_EVT_MODE_SHUTDOWN:	// 0x1
		timer_stop(ch, 0);
		break;
	case CLOCK_EVT_MODE_RESUME:		// 0x4
		timer_event_resume(info);
	case CLOCK_EVT_MODE_PERIODIC:	// 0x2
		timer_stop (ch, 0);
		timer_count(ch, cnt);
		timer_start(ch, 1);
		break;
	default:
		break;
	}
}

static int timer_event_set_next(unsigned long delta, struct clock_event_device *evt)
{
	struct timer_info *info = tm_event_info();
	int ch = info->channel;
	ulong flags;

	pr_debug("%s (ch:%d,delta:%ld)\n", __func__, ch, delta);
	raw_local_irq_save(flags);

	timer_stop (ch, 0);
	timer_count(ch, delta);
	timer_start(ch, 1);

	raw_local_irq_restore(flags);
	return 0;
}

static struct clock_event_device tm_event_clk = {
	.name			= "event timer",
	.features       = CLOCK_EVT_FEAT_PERIODIC | CLOCK_EVT_FEAT_ONESHOT,
	.set_mode		= timer_event_set_mode,
	.set_next_event	= timer_event_set_next,
	.rating			= 250,
};

#define	TIMER_TICK_MSG(ch, cn) 	{	\
		static long count = 0;			\
		if (0 == (count++ % cn))		\
			printk("[cpu.%d ch.%d evt: %6ld]\n", smp_processor_id(), ch, count-1);	\
		}

static irqreturn_t timer_event_handler(int irq, void *dev_id)
{
	struct clock_event_device *evt = &tm_event_clk;
	struct timer_info *info= tm_event_info();
	int ch = info->channel;

	timer_irq_clear(ch);
	evt->event_handler(evt);

	return IRQ_HANDLED;
}

static struct irqaction timer_event_irqaction = {
	.name		= "Event Timer IRQ",
	.flags		= IRQF_DISABLED | IRQF_TIMER,
	.handler	= timer_event_handler,
};

static int __init timer_event_init(int ch)
{
	struct clock_event_device *evt = &tm_event_clk;
	struct timer_info *info = tm_event_info();

	info->channel = ch;
	info->irqno = IRQ_PHY_TIMER_INT0 + ch;

	timer_clock_select(info, TIMER_CLOCK_EVENT_HZ);

	/*
	 * setup timer as free-running clocksource
	 */
	timer_stop (ch, 1);
	timer_clock(ch, info->tmmux, info->prescale);

	/*
	 * Make irqs happen for the system timer
	 */
	setup_irq(info->irqno, &timer_event_irqaction);

	/*
	 * register timer event device
	 */
	clockevents_calc_mult_shift(evt, info->rate, 5);
	evt->max_delta_ns = clockevent_delta2ns(0xffffffff, evt);
	evt->min_delta_ns = clockevent_delta2ns(0xf, evt);
	evt->cpumask = cpumask_of(0);
	evt->irq = info->irqno;
	clockevents_register_device(evt);

	pr_debug("timer.%d: event  shift =%u  \n", ch, evt->shift);
	pr_debug("timer.%d: event  mult  =%u  \n", ch, evt->mult);
	pr_debug("timer.%d: event  max   =%lld\n", ch, evt->max_delta_ns);
	pr_debug("timer.%d: event  min   =%lld\n", ch, evt->min_delta_ns);

	printk("timer.%d: event , %9lu(HZ:%d), mult:%u\n",
		ch, info->rate, HZ, evt->mult);
	return 0;
}

static void __init timer_initialize(void)
{
	pr_debug("%s\n", __func__);
timer_reset(0);
timer_reset(1);
	timer_source_init(CFG_TIMER_SYS_TICK_CH);
	timer_event_init(CFG_TIMER_EVT_TICK_CH);
	
	pr_debug("%s done\n", __func__);
	return;
}

struct sys_timer exynos4_timer = {
	.init		= timer_initialize,
};
