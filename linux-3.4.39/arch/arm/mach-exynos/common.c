/*
 * Copyright (c) 2010-2011 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * Common Codes for EXYNOS
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/sched.h>
#include <linux/serial_core.h>
#include <linux/of.h>
#include <linux/of_irq.h>

#include <asm/proc-fns.h>
#include <asm/exception.h>
#include <asm/hardware/cache-l2x0.h>
#include <asm/hardware/gic.h>
#include <asm/mach/map.h>
#include <asm/mach/irq.h>
#include <asm/cacheflush.h>

#include <mach/regs-irq.h>
#include <mach/regs-pmu.h>
#include <mach/regs-gpio.h>
#include <mach/pmu.h>

#include <plat/cpu.h>
#include <plat/clock.h>
#include <plat/devs.h>
#include <plat/pm.h>
#include <plat/sdhci.h>
#include <plat/gpio-cfg.h>
#include <plat/adc-core.h>
#include <plat/fb-core.h>
#include <plat/fimc-core.h>
#include <plat/iic-core.h>
#include <plat/tv-core.h>
#include <plat/regs-serial.h>

#include "common.h"
#define L2_AUX_VAL 0x7C470001
#define L2_AUX_MASK 0xC200ffff

static const char name_exynos4210[] = "EXYNOS4210";
static const char name_exynos4212[] = "EXYNOS4212";
static const char name_exynos4412[] = "EXYNOS4412";
static const char name_exynos5250[] = "EXYNOS5250";

static void exynos4_map_io(void);
static void exynos5_map_io(void);
static void exynos4_init_clocks(int xtal);
static void exynos5_init_clocks(int xtal);
static void exynos_init_uarts(struct s3c2410_uartcfg *cfg, int no);
static int exynos_init(void);

static struct cpu_table cpu_ids[] __initdata = {
	{
		.idcode		= EXYNOS4210_CPU_ID,
		.idmask		= EXYNOS4_CPU_MASK,
		.map_io		= exynos4_map_io,
		.init_clocks	= exynos4_init_clocks,
		.init_uarts	= exynos_init_uarts,
		.init		= exynos_init,
		.name		= name_exynos4210,
	}, {
		.idcode		= EXYNOS4212_CPU_ID,
		.idmask		= EXYNOS4_CPU_MASK,
		.map_io		= exynos4_map_io,
		.init_clocks	= exynos4_init_clocks,
		.init_uarts	= exynos_init_uarts,
		.init		= exynos_init,
		.name		= name_exynos4212,
	}, {
		.idcode		= EXYNOS4412_CPU_ID,
		.idmask		= EXYNOS4_CPU_MASK,
		.map_io		= exynos4_map_io,
		.init_clocks	= exynos4_init_clocks,
		.init_uarts	= exynos_init_uarts,
		.init		= exynos_init,
		.name		= name_exynos4412,
	}, {
		.idcode		= EXYNOS5250_SOC_ID,
		.idmask		= EXYNOS5_SOC_MASK,
		.map_io		= exynos5_map_io,
		.init_clocks	= exynos5_init_clocks,
		.init_uarts	= exynos_init_uarts,
		.init		= exynos_init,
		.name		= name_exynos5250,
	},
};
#define PB_IO_MAP(_n_, _v_, _p_, _s_, _t_)      \
        {                                                                               \
                .virtual        = _v_,                                  \
                .pfn            = __phys_to_pfn(_p_),   \
                .length         = _s_,                                  \
                .type           = _t_                                   \
        },


/* Initial IO mappings */

static struct map_desc exynos_iodesc[] __initdata = {
/*
 *      Length must be aligned 1MB
 *
 *      Refer to mach/iomap.h
 *
 *      Physical : __PB_IO_MAP_ ## _n_ ## _PHYS
 *      Virtual  : __PB_IO_MAP_ ## _n_ ## _VIRT
 *
 *      name    .virtual,       .pfn,           .length,        .type
 */
	PB_IO_MAP(      REGS,   0xF0000000,     0xC0000000,     0x00300000,     MT_DEVICE )             /* NOMAL IO, Reserved */
	PB_IO_MAP(      CCI4,   0xF0300000,     0xE0000000,     0x00100000,     MT_DEVICE )             /* CCI-400 */
	PB_IO_MAP(      SRAM,   0xF0400000,     0xFFF00000,     0x00100000,     MT_DEVICE )             /* SRAM */
	PB_IO_MAP(      NAND,   0xF0500000,     0x2C000000,     0x00100000,     MT_DEVICE )             /* NAND  */
	PB_IO_MAP(      IROM,   0xF0600000,     0x00000000,     0x00100000,     MT_DEVICE )             /* IROM  */

};

static struct map_desc exynos4_iodesc[] __initdata = {
	{
		.virtual	= (unsigned long)S3C_VA_SYS,
		.pfn		= __phys_to_pfn(EXYNOS4_PA_SYSCON),
		.length		= SZ_64K,
		.type		= MT_DEVICE,
	}, {
		.virtual	= (unsigned long)S3C_VA_TIMER,
		.pfn		= __phys_to_pfn(EXYNOS4_PA_TIMER),
		.length		= SZ_16K,
		.type		= MT_DEVICE,
	}, {
		.virtual	= (unsigned long)S3C_VA_WATCHDOG,
		.pfn		= __phys_to_pfn(EXYNOS4_PA_WATCHDOG),
		.length		= SZ_4K,
		.type		= MT_DEVICE,
	}, {
		.virtual	= (unsigned long)S5P_VA_SROMC,
		.pfn		= __phys_to_pfn(EXYNOS4_PA_SROMC),
		.length		= SZ_4K,
		.type		= MT_DEVICE,
	}, {
		.virtual	= (unsigned long)S5P_VA_SYSTIMER,
		.pfn		= __phys_to_pfn(EXYNOS4_PA_SYSTIMER),
		.length		= SZ_4K,
		.type		= MT_DEVICE,
	}, {
		.virtual	= (unsigned long)S5P_VA_PMU,
		.pfn		= __phys_to_pfn(EXYNOS4_PA_PMU),
		.length		= SZ_64K,
		.type		= MT_DEVICE,
	}, {
		.virtual	= (unsigned long)S5P_VA_COMBINER_BASE,
		.pfn		= __phys_to_pfn(EXYNOS4_PA_COMBINER),
		.length		= SZ_4K,
		.type		= MT_DEVICE,
	}, {
		.virtual	= (unsigned long)S5P_VA_GIC_CPU,
		.pfn		= __phys_to_pfn(EXYNOS4_PA_GIC_CPU),
		.length		= SZ_64K,
		.type		= MT_DEVICE,
	}, {
		.virtual	= (unsigned long)S5P_VA_GIC_DIST,
		.pfn		= __phys_to_pfn(EXYNOS4_PA_GIC_DIST),
		.length		= SZ_64K,
		.type		= MT_DEVICE,
	}, {
		.virtual	= (unsigned long)S3C_VA_UART,
		.pfn		= __phys_to_pfn(EXYNOS4_PA_UART),
		.length		= SZ_512K,
		.type		= MT_DEVICE,
	}, {
		.virtual	= (unsigned long)S5P_VA_CMU,
		.pfn		= __phys_to_pfn(EXYNOS4_PA_CMU),
		.length		= SZ_128K,
		.type		= MT_DEVICE,
	}, {
		.virtual	= (unsigned long)S5P_VA_COREPERI_BASE,
		.pfn		= __phys_to_pfn(EXYNOS4_PA_COREPERI),
		.length		= SZ_8K,
		.type		= MT_DEVICE,
	}, {
		.virtual	= (unsigned long)S5P_VA_L2CC,
		.pfn		= __phys_to_pfn(EXYNOS4_PA_L2CC),
		.length		= SZ_4K,
		.type		= MT_DEVICE,
	}, {
		.virtual	= (unsigned long)S5P_VA_DMC0,
		.pfn		= __phys_to_pfn(EXYNOS4_PA_DMC0),
		.length		= SZ_64K,
		.type		= MT_DEVICE,
	}, {
		.virtual	= (unsigned long)S5P_VA_DMC1,
		.pfn		= __phys_to_pfn(EXYNOS4_PA_DMC1),
		.length		= SZ_64K,
		.type		= MT_DEVICE,
	}, {
		.virtual	= (unsigned long)S3C_VA_USB_HSPHY,
		.pfn		= __phys_to_pfn(EXYNOS4_PA_HSPHY),
		.length		= SZ_4K,
		.type		= MT_DEVICE,
	},
};

static struct map_desc exynos4_iodesc0[] __initdata = {
	{
		.virtual	= (unsigned long)S5P_VA_SYSRAM,
		.pfn		= __phys_to_pfn(EXYNOS4_PA_SYSRAM0),
		.length		= SZ_4K,
		.type		= MT_DEVICE,
	},
};

static struct map_desc exynos4_iodesc1[] __initdata = {
	{
		.virtual	= (unsigned long)S5P_VA_SYSRAM,
		.pfn		= __phys_to_pfn(EXYNOS4_PA_SYSRAM1),
		.length		= SZ_4K,
		.type		= MT_DEVICE,
	},
};

static struct map_desc exynos5_iodesc[] __initdata = {
	{
		.virtual	= (unsigned long)S3C_VA_SYS,
		.pfn		= __phys_to_pfn(EXYNOS5_PA_SYSCON),
		.length		= SZ_64K,
		.type		= MT_DEVICE,
	}, {
		.virtual	= (unsigned long)S3C_VA_TIMER,
		.pfn		= __phys_to_pfn(EXYNOS5_PA_TIMER),
		.length		= SZ_16K,
		.type		= MT_DEVICE,
	}, {
		.virtual	= (unsigned long)S3C_VA_WATCHDOG,
		.pfn		= __phys_to_pfn(EXYNOS5_PA_WATCHDOG),
		.length		= SZ_4K,
		.type		= MT_DEVICE,
	}, {
		.virtual	= (unsigned long)S5P_VA_SROMC,
		.pfn		= __phys_to_pfn(EXYNOS5_PA_SROMC),
		.length		= SZ_4K,
		.type		= MT_DEVICE,
	}, {
		.virtual	= (unsigned long)S5P_VA_SYSTIMER,
		.pfn		= __phys_to_pfn(EXYNOS5_PA_SYSTIMER),
		.length		= SZ_4K,
		.type		= MT_DEVICE,
	}, {
		.virtual	= (unsigned long)S5P_VA_SYSRAM,
		.pfn		= __phys_to_pfn(EXYNOS5_PA_SYSRAM),
		.length		= SZ_4K,
		.type		= MT_DEVICE,
	}, {
		.virtual	= (unsigned long)S5P_VA_CMU,
		.pfn		= __phys_to_pfn(EXYNOS5_PA_CMU),
		.length		= 144 * SZ_1K,
		.type		= MT_DEVICE,
	}, {
		.virtual	= (unsigned long)S5P_VA_PMU,
		.pfn		= __phys_to_pfn(EXYNOS5_PA_PMU),
		.length		= SZ_64K,
		.type		= MT_DEVICE,
	}, {
		.virtual	= (unsigned long)S5P_VA_COMBINER_BASE,
		.pfn		= __phys_to_pfn(EXYNOS5_PA_COMBINER),
		.length		= SZ_4K,
		.type		= MT_DEVICE,
	}, {
		.virtual	= (unsigned long)S3C_VA_UART,
		.pfn		= __phys_to_pfn(EXYNOS5_PA_UART),
		.length		= SZ_512K,
		.type		= MT_DEVICE,
	}, {
		.virtual	= (unsigned long)S5P_VA_GIC_CPU,
		.pfn		= __phys_to_pfn(EXYNOS5_PA_GIC_CPU),
		.length		= SZ_64K,
		.type		= MT_DEVICE,
	}, {
		.virtual	= (unsigned long)S5P_VA_GIC_DIST,
		.pfn		= __phys_to_pfn(EXYNOS5_PA_GIC_DIST),
		.length		= SZ_64K,
		.type		= MT_DEVICE,
	},
};

void exynos4_restart(char mode, const char *cmd)
{
/*	__raw_writel(0x1, S5P_SWRESET);*/
}

void exynos5_restart(char mode, const char *cmd)
{
	__raw_writel(0x1, EXYNOS_SWRESET);
}

/*
 * exynos_map_io
 *
 * register the standard cpu IO areas
 */
extern void __init early_print(const char *str, ...);
void __init exynos_init_io(struct map_desc *mach_desc, int size)
{
	/* initialize the io descriptors we need for initialization */
	iotable_init(exynos_iodesc, ARRAY_SIZE(exynos_iodesc));
	early_print("call map_io.\n");
	if (mach_desc)
		iotable_init(mach_desc, size);
	
	/* detect cpu id and rev. */
	/*s5p_init_cpu(S5P_VA_CHIPID);*/

	/*s3c_init_cpu(samsung_cpu_id, cpu_ids, ARRAY_SIZE(cpu_ids));*/
	early_print("end of exynos_init_io.\n");
}

static void __init exynos4_map_io(void)
{
	iotable_init(exynos4_iodesc, ARRAY_SIZE(exynos4_iodesc));

	if (soc_is_exynos4210() && samsung_rev() == EXYNOS4210_REV_0)
		iotable_init(exynos4_iodesc0, ARRAY_SIZE(exynos4_iodesc0));
	else
		iotable_init(exynos4_iodesc1, ARRAY_SIZE(exynos4_iodesc1));

	/* initialize device information early */
	exynos4_default_sdhci0();
	exynos4_default_sdhci1();
	exynos4_default_sdhci2();
	exynos4_default_sdhci3();

	s3c_adc_setname("samsung-adc-v3");

	s3c_fimc_setname(0, "exynos4-fimc");
	s3c_fimc_setname(1, "exynos4-fimc");
	s3c_fimc_setname(2, "exynos4-fimc");
	s3c_fimc_setname(3, "exynos4-fimc");

	s3c_sdhci_setname(0, "exynos4-sdhci");
	s3c_sdhci_setname(1, "exynos4-sdhci");
	s3c_sdhci_setname(2, "exynos4-sdhci");
	s3c_sdhci_setname(3, "exynos4-sdhci");

	/* The I2C bus controllers are directly compatible with s3c2440 */
	s3c_i2c0_setname("s3c2440-i2c");
	s3c_i2c1_setname("s3c2440-i2c");
	s3c_i2c2_setname("s3c2440-i2c");

	s5p_fb_setname(0, "exynos4-fb");
	s5p_hdmi_setname("exynos4-hdmi");
}

static void __init exynos5_map_io(void)
{
	iotable_init(exynos5_iodesc, ARRAY_SIZE(exynos5_iodesc));

	s3c_device_i2c0.resource[0].start = EXYNOS5_PA_IIC(0);
	s3c_device_i2c0.resource[0].end   = EXYNOS5_PA_IIC(0) + SZ_4K - 1;
	s3c_device_i2c0.resource[1].start = EXYNOS5_IRQ_IIC;
	s3c_device_i2c0.resource[1].end   = EXYNOS5_IRQ_IIC;

	s3c_sdhci_setname(0, "exynos4-sdhci");
	s3c_sdhci_setname(1, "exynos4-sdhci");
	s3c_sdhci_setname(2, "exynos4-sdhci");
	s3c_sdhci_setname(3, "exynos4-sdhci");

	/* The I2C bus controllers are directly compatible with s3c2440 */
	s3c_i2c0_setname("s3c2440-i2c");
	s3c_i2c1_setname("s3c2440-i2c");
	s3c_i2c2_setname("s3c2440-i2c");
}

static void __init exynos4_init_clocks(int xtal)
{
	printk(KERN_DEBUG "%s: initializing clocks\n", __func__);

	s3c24xx_register_baseclocks(xtal);
	s5p_register_clocks(xtal);

	if (soc_is_exynos4210())
		exynos4210_register_clocks();
	else if (soc_is_exynos4212() || soc_is_exynos4412())
		exynos4212_register_clocks();

	exynos4_register_clocks();
	exynos4_setup_clocks();
}

static void __init exynos5_init_clocks(int xtal)
{
	printk(KERN_DEBUG "%s: initializing clocks\n", __func__);

	s3c24xx_register_baseclocks(xtal);
	s5p_register_clocks(xtal);

	exynos5_register_clocks();
	exynos5_setup_clocks();
}

#define COMBINER_ENABLE_SET	0x0
#define COMBINER_ENABLE_CLEAR	0x4
#define COMBINER_INT_STATUS	0xC

static DEFINE_SPINLOCK(irq_controller_lock);

struct combiner_chip_data {
	unsigned int irq_offset;
	unsigned int irq_mask;
	void __iomem *base;
};

static struct combiner_chip_data combiner_data[MAX_COMBINER_NR];

static inline void __iomem *combiner_base(struct irq_data *data)
{
	struct combiner_chip_data *combiner_data =
		irq_data_get_irq_chip_data(data);

	return combiner_data->base;
}

static void combiner_mask_irq(struct irq_data *data)
{
	u32 mask = 1 << (data->irq % 32);

	__raw_writel(mask, combiner_base(data) + COMBINER_ENABLE_CLEAR);
}

static void combiner_unmask_irq(struct irq_data *data)
{
	u32 mask = 1 << (data->irq % 32);

	__raw_writel(mask, combiner_base(data) + COMBINER_ENABLE_SET);
}

static void combiner_handle_cascade_irq(unsigned int irq, struct irq_desc *desc)
{
	struct combiner_chip_data *chip_data = irq_get_handler_data(irq);
	struct irq_chip *chip = irq_get_chip(irq);
	unsigned int cascade_irq, combiner_irq;
	unsigned long status;

	chained_irq_enter(chip, desc);

	spin_lock(&irq_controller_lock);
	status = __raw_readl(chip_data->base + COMBINER_INT_STATUS);
	spin_unlock(&irq_controller_lock);
	status &= chip_data->irq_mask;

	if (status == 0)
		goto out;

	combiner_irq = __ffs(status);

	cascade_irq = combiner_irq + (chip_data->irq_offset & ~31);
	if (unlikely(cascade_irq >= NR_IRQS))
		do_bad_IRQ(cascade_irq, desc);
	else
		generic_handle_irq(cascade_irq);

 out:
	chained_irq_exit(chip, desc);
}

static struct irq_chip combiner_chip = {
	.name		= "COMBINER",
	.irq_mask	= combiner_mask_irq,
	.irq_unmask	= combiner_unmask_irq,
};

static void __init combiner_cascade_irq(unsigned int combiner_nr, unsigned int irq)
{
	unsigned int max_nr;

	if (soc_is_exynos5250())
		max_nr = EXYNOS5_MAX_COMBINER_NR;
	else
		max_nr = EXYNOS4_MAX_COMBINER_NR;

	if (combiner_nr >= max_nr)
		BUG();
	if (irq_set_handler_data(irq, &combiner_data[combiner_nr]) != 0)
		BUG();
	irq_set_chained_handler(irq, combiner_handle_cascade_irq);
}

static void __init combiner_init(unsigned int combiner_nr, void __iomem *base,
			  unsigned int irq_start)
{
	unsigned int i;
	unsigned int max_nr;

	if (soc_is_exynos5250())
		max_nr = EXYNOS5_MAX_COMBINER_NR;
	else
		max_nr = EXYNOS4_MAX_COMBINER_NR;

	if (combiner_nr >= max_nr)
		BUG();

	combiner_data[combiner_nr].base = base;
	combiner_data[combiner_nr].irq_offset = irq_start;
	combiner_data[combiner_nr].irq_mask = 0xff << ((combiner_nr % 4) << 3);

	/* Disable all interrupts */

	__raw_writel(combiner_data[combiner_nr].irq_mask,
		     base + COMBINER_ENABLE_CLEAR);

	/* Setup the Linux IRQ subsystem */

	for (i = irq_start; i < combiner_data[combiner_nr].irq_offset
				+ MAX_IRQ_IN_COMBINER; i++) {
		irq_set_chip_and_handler(i, &combiner_chip, handle_level_irq);
		irq_set_chip_data(i, &combiner_data[combiner_nr]);
		set_irq_flags(i, IRQF_VALID | IRQF_PROBE);
	}
}

#ifdef CONFIG_OF
static const struct of_device_id exynos4_dt_irq_match[] = {
	{ .compatible = "arm,cortex-a9-gic", .data = gic_of_init, },
	{},
};
#endif
/******************************************************************************************************************/
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

#define __PB_IO_MAP_REGS_PHYS 0xc0000000
#define __PB_IO_MAP_REGS_VIRT 0xf0000000
#define IO_ADDRESS(x)   (x - __PB_IO_MAP_REGS_PHYS + __PB_IO_MAP_REGS_VIRT)
#define __io_address(n) ((void __iomem *)IO_ADDRESS(n))

#endif /* __ASM_ARM_ARCH_IO_H */

#define GPIO_NUM_PER_BANK       32
#define ARCH_NR_GPIOS           (GPIO_NUM_PER_BANK * 6) /* For GPIO A, B, C, D, E, ALVIE */

extern const unsigned char gpio_alt_no[][GPIO_NUM_PER_BANK];
#define GET_GPIO_ALTFUNC(io,idx)    (gpio_alt_no[io][idx])


#ifndef __MACH_GPIO_DESC_H__
#define __MACH_GPIO_DESC_H__

enum {
	ALT_NO_0 = 0,
	ALT_NO_1 = 1,
	ALT_NO_2 = 2,
	ALT_NO_3 = 3,
};

#define	ALT_NO_GPIO_A	\
	  ALT_NO_0 , ALT_NO_0 , ALT_NO_0 , ALT_NO_0 , ALT_NO_0 , ALT_NO_0 , ALT_NO_0 , ALT_NO_0 ,	\
	  ALT_NO_0 , ALT_NO_0 , ALT_NO_0 , ALT_NO_0 , ALT_NO_0 , ALT_NO_0 , ALT_NO_0 , ALT_NO_0 ,	\
	  ALT_NO_0 , ALT_NO_0 , ALT_NO_0 , ALT_NO_0 , ALT_NO_0 , ALT_NO_0 , ALT_NO_0 , ALT_NO_0 ,	\
	  ALT_NO_0 , ALT_NO_0 , ALT_NO_0 , ALT_NO_0 , ALT_NO_0 , ALT_NO_0 , ALT_NO_0 , ALT_NO_0 	\

#define	ALT_NO_GPIO_B	\
	  ALT_NO_0 , ALT_NO_0 , ALT_NO_0 , ALT_NO_0 , ALT_NO_0 , ALT_NO_0 , ALT_NO_0 , ALT_NO_0 ,	\
	  ALT_NO_0 , ALT_NO_0 , ALT_NO_0 , ALT_NO_2 , ALT_NO_2 , ALT_NO_1 , ALT_NO_2 , ALT_NO_1 ,	\
	  ALT_NO_2 , ALT_NO_1 , ALT_NO_2 , ALT_NO_1 , ALT_NO_1 , ALT_NO_1 , ALT_NO_1 , ALT_NO_1 ,	\
	  ALT_NO_1 , ALT_NO_1 , ALT_NO_1 , ALT_NO_1 , ALT_NO_1 , ALT_NO_1 , ALT_NO_1 , ALT_NO_1		\

#define	ALT_NO_GPIO_C	\
	  ALT_NO_1 , ALT_NO_1 , ALT_NO_1 , ALT_NO_1 , ALT_NO_1 , ALT_NO_1 , ALT_NO_1 , ALT_NO_1 ,	\
	  ALT_NO_1 , ALT_NO_1 , ALT_NO_1 , ALT_NO_1 , ALT_NO_1 , ALT_NO_1 , ALT_NO_1 , ALT_NO_1 ,	\
	  ALT_NO_1 , ALT_NO_1 , ALT_NO_1 , ALT_NO_1 , ALT_NO_1 , ALT_NO_1 , ALT_NO_1 , ALT_NO_1 ,	\
	  ALT_NO_1 , ALT_NO_1 , ALT_NO_1 , ALT_NO_1,  ALT_NO_0 , ALT_NO_0 , ALT_NO_0 , ALT_NO_0		\

#define	ALT_NO_GPIO_D	\
	  ALT_NO_0 , ALT_NO_0 , ALT_NO_0 , ALT_NO_0 , ALT_NO_0 , ALT_NO_0 , ALT_NO_0 , ALT_NO_0 ,	\
	  ALT_NO_0 , ALT_NO_0 , ALT_NO_0 , ALT_NO_0 , ALT_NO_0 , ALT_NO_0 , ALT_NO_0 , ALT_NO_0 ,	\
	  ALT_NO_0 , ALT_NO_0 , ALT_NO_0 , ALT_NO_0 , ALT_NO_0 , ALT_NO_0 , ALT_NO_0 , ALT_NO_0 ,	\
	  ALT_NO_0 , ALT_NO_0 , ALT_NO_0 , ALT_NO_0 , ALT_NO_0 , ALT_NO_0 , ALT_NO_0 , ALT_NO_0		\

#define	ALT_NO_GPIO_E	\
	  ALT_NO_0 , ALT_NO_0 , ALT_NO_0 , ALT_NO_0 , ALT_NO_0 , ALT_NO_0 , ALT_NO_0 , ALT_NO_0 ,	\
	  ALT_NO_0 , ALT_NO_0 , ALT_NO_0 , ALT_NO_0 , ALT_NO_0 , ALT_NO_0 , ALT_NO_0 , ALT_NO_0 ,	\
	  ALT_NO_0 , ALT_NO_0 , ALT_NO_0 , ALT_NO_0 , ALT_NO_0 , ALT_NO_0 , ALT_NO_0 , ALT_NO_0 ,	\
	  ALT_NO_0 , ALT_NO_1 , ALT_NO_1 , ALT_NO_1 , ALT_NO_1 , ALT_NO_1 , ALT_NO_1 , ALT_NO_1		\

#define	ALT_NO_ALIVE		\
	  ALT_NO_0 , ALT_NO_0 , ALT_NO_0 , ALT_NO_0 , ALT_NO_0 , ALT_NO_0 , ALT_NO_0 , ALT_NO_0 ,	\
	  ALT_NO_0 , ALT_NO_0 , ALT_NO_0 , ALT_NO_0 , ALT_NO_0 , ALT_NO_0 , ALT_NO_0 , ALT_NO_0 ,	\
	  ALT_NO_0 , ALT_NO_0 , ALT_NO_0 , ALT_NO_0 , ALT_NO_0 , ALT_NO_0 , ALT_NO_0 , ALT_NO_0 ,	\
	  ALT_NO_0 , ALT_NO_0 , ALT_NO_0 , ALT_NO_0 , ALT_NO_0 , ALT_NO_0 , ALT_NO_0 , ALT_NO_0		\

#endif
const unsigned char gpio_alt_no[][GPIO_NUM_PER_BANK] = { 
        { ALT_NO_GPIO_A }, { ALT_NO_GPIO_B }, { ALT_NO_GPIO_C },
        { ALT_NO_GPIO_D }, { ALT_NO_GPIO_E }, { ALT_NO_ALIVE  },  
};

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
/*#include "s5p6818_irq.h"*/

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
#endif
#ifndef __NX_CHIP_P5430_H__
#define __NX_CHIP_P5430_H__

#ifdef __cplusplus
extern "C" {
#endif


//-----------
// NUMBER OF MODULES
//-----------

#define NUMBER_OF_nx01301_CORTEXA9MP_TOP_QUADL2C_MODULE     1
#define NUMBER_OF_MCUSTOP_MODULE                            1
#define NUMBER_OF_AXISRAM_MODULE                            1
#define NUMBER_OF_DMA_MODULE                                2
#define NUMBER_OF_DREX_MODULE                               1
#define NUMBER_OF_CLKPWR_MODULE                             1
#define NUMBER_OF_INTC_MODULE                               1
#define NUMBER_OF_pl01115_Uart_modem_MODULE                 1
#define NUMBER_OF_UART_MODULE                               2
#define NUMBER_OF_pl01115_Uart_nodma_MODULE                 3
#define NUMBER_OF_SSP_MODULE                                3
#define NUMBER_OF_I2C_MODULE                                3
#define NUMBER_OF_I2S_MODULE                                3
#define NUMBER_OF_DEINTERLACE_MODULE                        1
#define NUMBER_OF_SCALER_MODULE                             1
#define NUMBER_OF_AC97_MODULE                               1
#define NUMBER_OF_SPDIFRX_MODULE                            1
#define NUMBER_OF_SPDIFTX_MODULE                            1
#define NUMBER_OF_TIMER_MODULE                              1
#define NUMBER_OF_PWM_MODULE                                1
#define NUMBER_OF_CLKGEN_MODULE                             41
#define NUMBER_OF_WDT_MODULE                                1
#define NUMBER_OF_MPEGTSI_MODULE                            1
#define NUMBER_OF_DISPLAYTOP_MODULE                         1
#define NUMBER_OF_VIP_MODULE                                3
#define NUMBER_OF_MIPI_MODULE                               1
#define NUMBER_OF_MALI400_MODULE                            1
#define NUMBER_OF_ADC_MODULE                                1
#define NUMBER_OF_PPM_MODULE                                1
#define NUMBER_OF_ahb3x1_MODULE                             1
#define NUMBER_OF_SDMMC_MODULE                              3
#define NUMBER_OF_CODA960_MODULE                            1
#define NUMBER_OF_DWC_GMAC_MODULE                           1
#define NUMBER_OF_USB20OTG_MODULE                           1
#define NUMBER_OF_USB20HOST_MODULE                          1
#define NUMBER_OF_CAN_MODULE                                2
#define NUMBER_OF_ECID_MODULE                               1
#define NUMBER_OF_RSTCON_MODULE                             1
#define NUMBER_OF_A3BM_AXI_TOP_MASTER_BUS_MODULE            1
#define NUMBER_OF_A3BM_AXI_BOTTOM_MASTER_BUS_MODULE         1
#define NUMBER_OF_A3BM_AXI_PERI_BUS_MODULE                  1
#define NUMBER_OF_A3BM_AXI_DISPLAY_BUS_MODULE               1
#define NUMBER_OF_GPIO_MODULE                               5
#define NUMBER_OF_CRYPTO_MODULE                             1
#define NUMBER_OF_PDM_MODULE                                1
#define NUMBER_OF_TIEOFF_MODULE                             1
#define NUMBER_OF_TMCB_MODULE                               4
#define NUMBER_OF_TMU_MODULE                                2
#define NUMBER_OF_TZPC_MODULE                               7
#define NUMBER_OF_CCI400_MODULE                             1


//------------
// PHY BASEADDR
//------------

//--------------------------------------------------------------------------
//  SFR Base Address
//--------------------------------------------------------------------------
// @modified sei - nxp5430 base address
#define PHY_BASEADDR_DMA0_MODULE                            0xC0000000
#define PHY_BASEADDR_DMA1_MODULE                            0xC0001000

#define PHY_BASEADDR_INTC_MODULE                            0xC0008000

#define PHY_BASEADDR_GPIOA_MODULE                           0xC001A000
#define PHY_BASEADDR_GPIOB_MODULE                           0xC001B000
#define PHY_BASEADDR_GPIOC_MODULE                           0xC001C000
#define PHY_BASEADDR_GPIOD_MODULE                           0xC001D000
#define PHY_BASEADDR_GPIOE_MODULE                           0xC001E000

#define PHY_BASEADDR_MCUSTOP_MODULE                         0xC0051000

#define PHY_BASEADDR_DREX_MODULE_CH0_APB                    0xC00E0000		// drex
#define PHY_BASEADDR_DREX_MODULE_CH1_APB                    0xC00E1000		// ddr phy
#define PHY_BASEADDR_XIUA_DREX0_MODULE                      0xC00E2000
#define PHY_BASEADDR_XIUA_DREX1_MODULE                      0xC00E3000
#define PHY_BASEADDR_XIUA_DREX2_MODULE                      0xC00E4000
#define PHY_BASEADDR_DREX_TZ_MODULE                         0xC00E5000		// drex trustzone
#define PHY_BASEADDR_DREXTZASC_MODULE                       0xC0300000		// address space controller

#define PHY_BASEADDR_TIMER_MODULE                           0xC0017000
#define PHY_BASEADDR_PWM_MODULE                             0xC0018000

#define PHY_BASEADDR_WDT_MODULE                             0xC0019000

#define PHY_BASEADDR_TMU0_MODULE                            0xC0096000
#define PHY_BASEADDR_TMU1_MODULE                            0xC0097000
#define PHY_BASEADDR_TMU2_MODULE                            0xC0092800

#define PHY_BASEADDR_ADC_MODULE                             0xC0053000

#define PHY_BASEADDR_PPM_MODULE                             0xC0054000

#define PHY_BASEADDR_I2S0_MODULE                            0xC0055000
#define PHY_BASEADDR_I2S1_MODULE                            0xC0056000
#define PHY_BASEADDR_I2S2_MODULE                            0xC0057000

#define PHY_BASEADDR_AC97_MODULE                            0xC0058000

#define PHY_BASEADDR_SPDIFTX_MODULE                         0xC0059000
#define PHY_BASEADDR_SPDIFRX_MODULE                         0xC005A000

#define PHY_BASEADDR_PDM_MODULE                             0xC0014000

#define PHY_BASEADDR_pl01115_Uart_modem_MODULE              0xC00A0000
#define PHY_BASEADDR_UART0_MODULE                           0xC00A1000
#define PHY_BASEADDR_UART1_MODULE                           0xC00A2000
#define PHY_BASEADDR_pl01115_Uart_nodma0_MODULE             0xC00A3000
#define PHY_BASEADDR_pl01115_Uart_nodma1_MODULE             0xC006D000
#define PHY_BASEADDR_pl01115_Uart_nodma2_MODULE             0xC006F000

#define PHY_BASEADDR_SSP0_MODULE                            0xC005B000
#define PHY_BASEADDR_SSP1_MODULE                            0xC005C000
#define PHY_BASEADDR_SSP2_MODULE                            0xC005F000

#define PHY_BASEADDR_I2C0_MODULE                            0xC00A4000
#define PHY_BASEADDR_I2C1_MODULE                            0xC00A5000
#define PHY_BASEADDR_I2C2_MODULE                            0xC00A6000

#define PHY_BASEADDR_CAN0_MODULE                            0xC00CE000
#define PHY_BASEADDR_CAN1_MODULE                            0xC00CF000

#define PHY_BASEADDR_MPEGTSI_MODULE                         0xC005D000

#define PHY_BASEADDR_DWC_GMAC_MODULE_APB0                   0xC0060000
#define PHY_BASEADDR_DWC_GMAC_MODULE_APB1                   0xC0061000

#define PHY_BASEADDR_USB20HOST_MODULE_OHCI_S_ABH            0xC0020000
#define PHY_BASEADDR_USB20HOST_MODULE_EHCI_S_ABH            0xC0030000
#define PHY_BASEADDR_USB20HOST_MODULE_APB                   0xC006A000

#define PHY_BASEADDR_USB20OTG_MODULE_AHBS0                  0xC0040000
#define PHY_BASEADDR_USB20OTG_MODULE_APB                    0xC006C000

#define PHY_BASEADDR_SDMMC0_MODULE                          0xC0062000
#define PHY_BASEADDR_SDMMC1_MODULE                          0xC0068000
#define PHY_BASEADDR_SDMMC2_MODULE                          0xC0069000


#define PHY_BASEADDR_CODA960_MODULE_APB0                    0xC0080000
#define PHY_BASEADDR_CODA960_MODULE_APB1                    0xC0081000
#define PHY_BASEADDR_CODA960_MODULE_APB2                    0xC0082000
#define PHY_BASEADDR_CODA960_MODULE_APB3                    0xC0083000
#define PHY_BASEADDR_XIUA_CODA0_MODULE                      0xC0084000
#define PHY_BASEADDR_XIUA_CODA1_MODULE                      0xC0085000

#define PHY_BASEADDR_MALI400_MODULE                         0xC0070000
#define PHY_BASEADDR_XIUA_MALI0_MODULE                      0xC009C000
#define PHY_BASEADDR_XIUA_MALI1_MODULE                      0xC009D000

#define PHY_BASEADDR_CSSYS_MODULE                           0xC0308000
#define PHY_BASEADDR_XIUA_CSSYS_MODULE                      0xC009E000

#define PHY_BASEADDR_VIP0_MODULE                            0xC0063000
#define PHY_BASEADDR_VIP1_MODULE                            0xC0064000
#define PHY_BASEADDR_VIP2_MODULE                            0xC0099000

#define PHY_BASEADDR_DEINTERLACE_MODULE                     0xC0065000

#define PHY_BASEADDR_SCALER_MODULE                          0xC0066000

#define PHY_BASEADDR_MIPI_MODULE                            0xC00D0000

#define PHY_BASEADDR_HDMI_PHY_MODULE                        0xC00F0000

#define PHY_BASEADDR_DISPLAYTOP_MODULE                      0xC0100000


#define PHY_BASEADDR_CLKGEN0_MODULE                         0xC00BB000
#define PHY_BASEADDR_CLKGEN1_MODULE                         0xC00BC000
#define PHY_BASEADDR_CLKGEN2_MODULE                         0xC00BD000
#define PHY_BASEADDR_CLKGEN3_MODULE                         0xC00BE000
#define PHY_BASEADDR_CLKGEN4_MODULE                         0xC00BF000
#define PHY_BASEADDR_CLKGEN5_MODULE                         0xC00C0000
#define PHY_BASEADDR_CLKGEN6_MODULE                         0xC00AE000
#define PHY_BASEADDR_CLKGEN7_MODULE                         0xC00AF000
#define PHY_BASEADDR_CLKGEN8_MODULE                         0xC00B0000
#define PHY_BASEADDR_CLKGEN9_MODULE                         0xC00CA000
#define PHY_BASEADDR_CLKGEN10_MODULE                        0xC00C8000
#define PHY_BASEADDR_CLKGEN11_MODULE                        0xC00B8000
#define PHY_BASEADDR_CLKGEN12_MODULE                        0xC00B7000
#define PHY_BASEADDR_CLKGEN13_MODULE                        0xC00BA000
#define PHY_BASEADDR_CLKGEN14_MODULE                        0xC00B9000
#define PHY_BASEADDR_CLKGEN15_MODULE                        0xC00B2000
#define PHY_BASEADDR_CLKGEN16_MODULE                        0xC00B3000
#define PHY_BASEADDR_CLKGEN17_MODULE                        0xC00B4000
#define PHY_BASEADDR_CLKGEN18_MODULE                        0xC00C5000
#define PHY_BASEADDR_CLKGEN19_MODULE                        0xC00CC000
#define PHY_BASEADDR_CLKGEN20_MODULE                        0xC00CD000
#define PHY_BASEADDR_CLKGEN21_MODULE                        0xC00C3000
#define PHY_BASEADDR_CLKGEN22_MODULE                        0xC00A9000
#define PHY_BASEADDR_CLKGEN23_MODULE                        0xC00AA000
#define PHY_BASEADDR_CLKGEN24_MODULE                        0xC00A8000
#define PHY_BASEADDR_CLKGEN25_MODULE                        0xC00AB000
#define PHY_BASEADDR_CLKGEN26_MODULE                        0xC006E000
#define PHY_BASEADDR_CLKGEN27_MODULE                        0xC00B1000
#define PHY_BASEADDR_CLKGEN28_MODULE                        0xC00B5000
#define PHY_BASEADDR_CLKGEN29_MODULE                        0xC00C4000
#define PHY_BASEADDR_CLKGEN30_MODULE                        0xC00C1000
#define PHY_BASEADDR_CLKGEN31_MODULE                        0xC00C2000
#define PHY_BASEADDR_CLKGEN32_MODULE                        0xC006B000
#define PHY_BASEADDR_CLKGEN33_MODULE                        0xC00C7000
#define PHY_BASEADDR_CLKGEN34_MODULE                        0xC00C6000
#define PHY_BASEADDR_CLKGEN35_MODULE                        0xC00B6000
#define PHY_BASEADDR_CLKGEN36_MODULE                        0xC00CB000
#define PHY_BASEADDR_CLKGEN37_MODULE                        0xC00AC000
#define PHY_BASEADDR_CLKGEN38_MODULE                        0xC00AD000
#define PHY_BASEADDR_CLKGEN39_MODULE                        0xC00A7000
#define PHY_BASEADDR_CLKGEN40_MODULE                        0xC009A000



#define PHY_BASEADDR_CRYPTO_MODULE                          0xC0015000
#define PHY_BASEADDR_ECID_MODULE                            0xC0067000
#define PHY_BASEADDR_CLKPWR_MODULE                          0xC0010000

#define PHY_BASEADDR_TIEOFF_MODULE                          0xC0011000
#define PHY_BASEADDR_RSTCON_MODULE                          0xC0012000

#define PHY_BASEADDR_AXISRAM_MODULE                         0xFFFF0000

#define PHY_BASEADDR_IOPERI_BUS_MODULE                      0xC0090000
#define PHY_BASEADDR_BOT_BUS_MODULE                         0xC0091000
#define PHY_BASEADDR_TOP_BUS_MODULE                         0xC0092000
#define PHY_BASEADDR_DISP_BUS_MODULE                        0xC0093000
#define PHY_BASEADDR_SFR_BUS_MODULE                         0xC0094000
#define PHY_BASEADDR_STATIC_BUS_MODULE                      0xC0095000

#define PHY_BASEADDR_nx01301_CORTEXA9MP_TOP_QUADL2C_MODULE  0x40000000

#define PHY_BASEADDR_A3BM_AXI_TOP_MASTER_BUS_MODULE                     0xC0052000
#define PHY_BASEADDR_A3BM_AXI_BOTTOM_MASTER_BUS_MODULE_APB_BOTTOMBUS    0xC0050000
#define PHY_BASEADDR_A3BM_AXI_PERI_BUS_MODULE                           0xC0000000
#define PHY_BASEADDR_A3BM_AXI_DISPLAY_BUS_MODULE                        0xC005E000

#define PHY_BASEADDR_TZPC0_MODULE                           0xC0301000
#define PHY_BASEADDR_TZPC1_MODULE                           0xC0302000
#define PHY_BASEADDR_TZPC2_MODULE                           0xC0303000
#define PHY_BASEADDR_TZPC3_MODULE                           0xC0304000
#define PHY_BASEADDR_TZPC4_MODULE                           0xC0305000
#define PHY_BASEADDR_TZPC5_MODULE                           0xC0306000
#define PHY_BASEADDR_TZPC6_MODULE                           0xC0307000

#define PHY_BASEADDR_CCI400_MODULE                          0xE0090000

//------------------------------------------------------------------------------
// Interrupt Number of mudules for the interrupt controller.
//------------------------------------------------------------------------------
//  REF EXAMPLE :
//  enum {
//      INTNUM_OF_OJTDMA_MODULE_IRQ     = 0
//      ,INTNUM_OF_OJTDMA_MODULE_DMAIRQ = 1
//      };


#define NX_INTC_NUM_OF_INT  62

//------------------------------------------------------------------------------
// BAGL RENDERING
//------------------------------------------------------------------------------
enum {
      INTNUM_OF_MCUSTOP_MODULE                              = 0
     ,INTNUM_OF_DMA0_MODULE                                 = 1
     ,INTNUM_OF_DMA1_MODULE                                 = 2
     ,INTNUM_OF_CLKPWR_MODULE_INTREQPWR                     = 3
     ,INTNUM_OF_CLKPWR_MODULE_ALIVEIRQ                      = 4
     ,INTNUM_OF_CLKPWR_MODULE_RTCIRQ                        = 5
     ,INTNUM_OF_pl01115_Uart_modem_MODULE                   = 6
     ,INTNUM_OF_UART0_MODULE                                = 7
     ,INTNUM_OF_UART1_MODULE                                = 8
     ,INTNUM_OF_pl01115_Uart_nodma0_MODULE                  = 9
     ,INTNUM_OF_pl01115_Uart_nodma1_MODULE                  = 10
     ,INTNUM_OF_pl01115_Uart_nodma2_MODULE                  = 11
     ,INTNUM_OF_SSP0_MODULE                                 = 12
     ,INTNUM_OF_SSP1_MODULE                                 = 13
     ,INTNUM_OF_SSP2_MODULE                                 = 14
     ,INTNUM_OF_I2C0_MODULE                                 = 15
     ,INTNUM_OF_I2C1_MODULE                                 = 16
     ,INTNUM_OF_I2C2_MODULE                                 = 17
     ,INTNUM_OF_DEINTERLACE_MODULE                          = 18
     ,INTNUM_OF_SCALER_MODULE                               = 19
     ,INTNUM_OF_AC97_MODULE                                 = 20
     ,INTNUM_OF_SPDIFRX_MODULE                              = 21
     ,INTNUM_OF_SPDIFTX_MODULE                              = 22
     ,INTNUM_OF_TIMER_MODULE_INT0                           = 23
     ,INTNUM_OF_TIMER_MODULE_INT1                           = 24
     ,INTNUM_OF_TIMER_MODULE_INT2                           = 25
     ,INTNUM_OF_TIMER_MODULE_INT3                           = 26
     ,INTNUM_OF_PWM_MODULE_INT0                             = 27
     ,INTNUM_OF_PWM_MODULE_INT1                             = 28
     ,INTNUM_OF_PWM_MODULE_INT2                             = 29
     ,INTNUM_OF_PWM_MODULE_INT3                             = 30
     ,INTNUM_OF_WDT_MODULE                                  = 31
     ,INTNUM_OF_MPEGTSI_MODULE                              = 32
     ,INTNUM_OF_DISPLAYTOP_MODULE_DUALDISPLAY_PRIMIRQ       = 33
     ,INTNUM_OF_DISPLAYTOP_MODULE_DUALDISPLAY_SECONDIRQ     = 34
     ,INTNUM_OF_DISPLAYTOP_MODULE_RESCONV_IRQ               = 35
     ,INTNUM_OF_DISPLAYTOP_MODULE_HDMI_IRQ                  = 36
     ,INTNUM_OF_VIP0_MODULE                                 = 37
     ,INTNUM_OF_VIP1_MODULE                                 = 38
     ,INTNUM_OF_MIPI_MODULE                                 = 39
     ,INTNUM_OF_MALI400_MODULE                              = 40
     ,INTNUM_OF_ADC_MODULE                                  = 41
     ,INTNUM_OF_PPM_MODULE                                  = 42
     ,INTNUM_OF_SDMMC0_MODULE                               = 43
     ,INTNUM_OF_SDMMC1_MODULE                               = 44
     ,INTNUM_OF_SDMMC2_MODULE                               = 45
     ,INTNUM_OF_CODA960_MODULE_HOST_INTRPT                  = 46
     ,INTNUM_OF_CODA960_MODULE_JPG_INTRPT                   = 47
     ,INTNUM_OF_DWC_GMAC_MODULE                             = 48
     ,INTNUM_OF_USB20OTG_MODULE                             = 49
     ,INTNUM_OF_USB20HOST_MODULE                            = 50
     ,INTNUM_OF_CAN0_MODULE                                 = 51
     ,INTNUM_OF_CAN1_MODULE                                 = 52
     ,INTNUM_OF_GPIOA_MODULE                                = 53
     ,INTNUM_OF_GPIOB_MODULE                                = 54
     ,INTNUM_OF_GPIOC_MODULE                                = 55
     ,INTNUM_OF_GPIOD_MODULE                                = 56
     ,INTNUM_OF_GPIOE_MODULE                                = 57
     ,INTNUM_OF_CRYPTO_MODULE                               = 58
     ,INTNUM_OF_PDM_MODULE                                  = 59
     ,INTNUM_OF_TMU0_MODULE                                 = 60
     ,INTNUM_OF_TMU1_MODULE                                 = 61
     ,INTNUM_OF_VIP2_MODULE									= 72
    };
//------------------------------------------------------------------------------
// BAGL RENDERING END
//------------------------------------------------------------------------------



//------------------------------------------------------------------------------
// Reset Controller : Number of Reset
//------------------------------------------------------------------------------
#define NUMBER_OF_RESET_MODULE_PIN 69
enum {
// xl50200_AC97_cfg0
RESETINDEX_OF_AC97_MODULE_PRESETn = 0
,// nx01301_CORTEXA9MP_TOP_QUADL2C
RESETINDEX_OF_nx01301_CORTEXA9MP_TOP_QUADL2C_MODULE_nCPURESET1 = 1
,// nx01301_CORTEXA9MP_TOP_QUADL2C
RESETINDEX_OF_nx01301_CORTEXA9MP_TOP_QUADL2C_MODULE_nCPURESET2 = 2
,// nx01301_CORTEXA9MP_TOP_QUADL2C
RESETINDEX_OF_nx01301_CORTEXA9MP_TOP_QUADL2C_MODULE_nCPURESET3 = 3
,// nx01301_CORTEXA9MP_TOP_QUADL2C
RESETINDEX_OF_nx01301_CORTEXA9MP_TOP_QUADL2C_MODULE_nWDRESET1 = 4
,// nx01301_CORTEXA9MP_TOP_QUADL2C
RESETINDEX_OF_nx01301_CORTEXA9MP_TOP_QUADL2C_MODULE_nWDRESET2 = 5
,// nx01301_CORTEXA9MP_TOP_QUADL2C
RESETINDEX_OF_nx01301_CORTEXA9MP_TOP_QUADL2C_MODULE_nWDRESET3 = 6
,// nx02600_CRYPTO_cfg0
RESETINDEX_OF_CRYPTO_MODULE_i_nRST = 7
,// nx01501_DEINTERLACE_cfg0
RESETINDEX_OF_DEINTERLACE_MODULE_i_nRST = 8
,// nx71000_DisplayTop_cfg5
RESETINDEX_OF_DISPLAYTOP_MODULE_i_Top_nRST = 9
,// nx71000_DisplayTop_cfg5
RESETINDEX_OF_DISPLAYTOP_MODULE_i_DualDisplay_nRST = 10
,// nx71000_DisplayTop_cfg5
RESETINDEX_OF_DISPLAYTOP_MODULE_i_ResConv_nRST = 11
,// nx71000_DisplayTop_cfg5
RESETINDEX_OF_DISPLAYTOP_MODULE_i_LCDIF_nRST = 12
,// nx71000_DisplayTop_cfg5
RESETINDEX_OF_DISPLAYTOP_MODULE_i_HDMI_nRST = 13
,// nx71000_DisplayTop_cfg5
RESETINDEX_OF_DISPLAYTOP_MODULE_i_HDMI_VIDEO_nRST = 14
,// nx71000_DisplayTop_cfg5
RESETINDEX_OF_DISPLAYTOP_MODULE_i_HDMI_SPDIF_nRST = 15
,// nx71000_DisplayTop_cfg5
RESETINDEX_OF_DISPLAYTOP_MODULE_i_HDMI_TMDS_nRST = 16
,// nx71000_DisplayTop_cfg5
RESETINDEX_OF_DISPLAYTOP_MODULE_i_HDMI_PHY_nRST = 17
,// nx71000_DisplayTop_cfg5
RESETINDEX_OF_DISPLAYTOP_MODULE_i_LVDS_nRST = 18
,// nx50100_ECID_128bit
RESETINDEX_OF_ECID_MODULE_i_nRST = 19
,// xl00100_I2C_cfg0
RESETINDEX_OF_I2C0_MODULE_PRESETn = 20
,// xl00100_I2C_cfg0
RESETINDEX_OF_I2C1_MODULE_PRESETn = 21
,// xl00100_I2C_cfg0
RESETINDEX_OF_I2C2_MODULE_PRESETn = 22
,// xl00300_I2S_cfg1
RESETINDEX_OF_I2S0_MODULE_PRESETn = 23
,// xl00300_I2S_cfg1
RESETINDEX_OF_I2S1_MODULE_PRESETn = 24
,// xl00300_I2S_cfg1
RESETINDEX_OF_I2S2_MODULE_PRESETn = 25
,// xl00112_DREX_cfg1
RESETINDEX_OF_DREX_MODULE_CRESETn = 26
,// xl00112_DREX_cfg1
RESETINDEX_OF_DREX_MODULE_ARESETn = 27
,// xl00112_DREX_cfg1
RESETINDEX_OF_DREX_MODULE_nPRST = 28
,// nx71100_mipi_cfg1
RESETINDEX_OF_MIPI_MODULE_i_nRST = 29
,// nx71100_mipi_cfg1
RESETINDEX_OF_MIPI_MODULE_i_DSI_I_PRESETn = 30
,// nx71100_mipi_cfg1
RESETINDEX_OF_MIPI_MODULE_i_CSI_I_PRESETn = 31
,// nx71100_mipi_cfg1
RESETINDEX_OF_MIPI_MODULE_i_PHY_S_RESETN = 32
,// nx71100_mipi_cfg1
RESETINDEX_OF_MIPI_MODULE_i_PHY_M_RESETN = 33
,// nx01910_MPEGTSI_cfg0
RESETINDEX_OF_MPEGTSI_MODULE_i_nRST = 34
,// nx02500_PDM_cfg0
RESETINDEX_OF_PDM_MODULE_i_nRST = 35
,// xl50010_PWMTIMER_usetimer
RESETINDEX_OF_TIMER_MODULE_PRESETn = 36
,// xl50010_PWMTIMER_usepwm
RESETINDEX_OF_PWM_MODULE_PRESETn = 37
,// nx01400_SCALER_cfg0
RESETINDEX_OF_SCALER_MODULE_i_nRST = 38
,// xl00500_SDMMC_cfg0
RESETINDEX_OF_SDMMC0_MODULE_i_nRST = 39
,// xl00500_SDMMC_cfg0
RESETINDEX_OF_SDMMC1_MODULE_i_nRST = 40
,// xl00500_SDMMC_cfg0
RESETINDEX_OF_SDMMC2_MODULE_i_nRST = 41
,// nx01600_SPDIFRX_cfg0
RESETINDEX_OF_SPDIFRX_MODULE_PRESETn = 42
,// xl50300_SPDIFTX_hdmipinout
RESETINDEX_OF_SPDIFTX_MODULE_PRESETn = 43
,// pl02212_Ssp_cfg1
RESETINDEX_OF_SSP0_MODULE_PRESETn = 44
,// pl02212_Ssp_cfg1
RESETINDEX_OF_SSP0_MODULE_nSSPRST = 45
,// pl02212_Ssp_cfg1
RESETINDEX_OF_SSP1_MODULE_PRESETn = 46
,// pl02212_Ssp_cfg1
RESETINDEX_OF_SSP1_MODULE_nSSPRST = 47
,// pl02212_Ssp_cfg1
RESETINDEX_OF_SSP2_MODULE_PRESETn = 48
,// pl02212_Ssp_cfg1
RESETINDEX_OF_SSP2_MODULE_nSSPRST = 49
,// pl01115_Uart_cfg0
RESETINDEX_OF_UART0_MODULE_nUARTRST = 50
,// pl01115_Uart_modem
RESETINDEX_OF_pl01115_Uart_modem_MODULE_nUARTRST = 51
,// pl01115_Uart_cfg0
RESETINDEX_OF_UART1_MODULE_nUARTRST = 52
,// pl01115_Uart_nodma
RESETINDEX_OF_pl01115_Uart_nodma0_MODULE_nUARTRST = 53
,// pl01115_Uart_nodma
RESETINDEX_OF_pl01115_Uart_nodma1_MODULE_nUARTRST = 54
,// pl01115_Uart_nodma
RESETINDEX_OF_pl01115_Uart_nodma2_MODULE_nUARTRST = 55
,// xl00700_USB20HOST_cfg0
RESETINDEX_OF_USB20HOST_MODULE_i_nRST = 56
,// xl00600_USB20OTG_cfg0
RESETINDEX_OF_USB20OTG_MODULE_i_nRST = 57
,// xl50500_WDT_cfg0
RESETINDEX_OF_WDT_MODULE_PRESETn = 58
,// xl50500_WDT_cfg0
RESETINDEX_OF_WDT_MODULE_nPOR = 59
,// nx01700_adc_cfg0
RESETINDEX_OF_ADC_MODULE_nRST = 60
,// xl07000_coda960_cfg1
RESETINDEX_OF_CODA960_MODULE_i_areset_n = 61
,// xl07000_coda960_cfg1
RESETINDEX_OF_CODA960_MODULE_i_preset_n = 62
,// xl07000_coda960_cfg1
RESETINDEX_OF_CODA960_MODULE_i_creset_n = 63
,// xl50400_DWC_GMAC_RGMII
RESETINDEX_OF_DWC_GMAC_MODULE_aresetn_i = 64
,// xl06000_mali400_cfg1
RESETINDEX_OF_MALI400_MODULE_nRST = 65
,// nx02300_ppm_cfg0
RESETINDEX_OF_PPM_MODULE_i_nRST = 66
,// nx01800_vip_twopadwrapper
RESETINDEX_OF_VIP1_MODULE_i_nRST = 67
,// nx01800_vip_cfg3
RESETINDEX_OF_VIP0_MODULE_i_nRST = 68
,
RESETINDEX_OF_VIP2_MODULE_i_nRST = 69
};




//------------------------------------------------------------------------------
// Clock Generator : Module Index of Clock Generator
//------------------------------------------------------------------------------

enum {
 CLOCKINDEX_OF_Inst_TIMER01_MODULE          = 0
,CLOCKINDEX_OF_Inst_TIMER02_MODULE          = 1
,CLOCKINDEX_OF_Inst_TIMER03_MODULE          = 2
,CLOCKINDEX_OF_Inst_PWM01_MODULE            = 3
,CLOCKINDEX_OF_Inst_PWM02_MODULE            = 4
,CLOCKINDEX_OF_Inst_PWM03_MODULE            = 5
,CLOCKINDEX_OF_I2C0_MODULE                  = 6
,CLOCKINDEX_OF_I2C1_MODULE                  = 7
,CLOCKINDEX_OF_I2C2_MODULE                  = 8
,CLOCKINDEX_OF_MIPI_MODULE                  = 9
,CLOCKINDEX_OF_DWC_GMAC_MODULE              = 10
,CLOCKINDEX_OF_SPDIFTX_MODULE               = 11
,CLOCKINDEX_OF_MPEGTSI_MODULE               = 12
,CLOCKINDEX_OF_PWM_MODULE                   = 13
,CLOCKINDEX_OF_TIMER_MODULE                 = 14
,CLOCKINDEX_OF_I2S0_MODULE                  = 15
,CLOCKINDEX_OF_I2S1_MODULE                  = 16
,CLOCKINDEX_OF_I2S2_MODULE                  = 17
,CLOCKINDEX_OF_SDMMC0_MODULE                = 18
,CLOCKINDEX_OF_SDMMC1_MODULE                = 19
,CLOCKINDEX_OF_SDMMC2_MODULE                = 20
,CLOCKINDEX_OF_MALI400_MODULE               = 21
,CLOCKINDEX_OF_UART0_MODULE                 = 22
,CLOCKINDEX_OF_UART1_MODULE                 = 23
,CLOCKINDEX_OF_pl01115_Uart_modem_MODULE    = 24
,CLOCKINDEX_OF_pl01115_Uart_nodma0_MODULE   = 25
,CLOCKINDEX_OF_pl01115_Uart_nodma1_MODULE   = 26
,CLOCKINDEX_OF_pl01115_Uart_nodma2_MODULE   = 27
,CLOCKINDEX_OF_DEINTERLACE_MODULE           = 28
,CLOCKINDEX_OF_PPM_MODULE                   = 29
,CLOCKINDEX_OF_VIP0_MODULE                  = 30
,CLOCKINDEX_OF_VIP1_MODULE                  = 31
,CLOCKINDEX_OF_USB20HOST_MODULE             = 32
,CLOCKINDEX_OF_CODA960_MODULE               = 33
,CLOCKINDEX_OF_CRYPTO_MODULE                = 34
,CLOCKINDEX_OF_SCALER_MODULE                = 35
,CLOCKINDEX_OF_PDM_MODULE                   = 36
,CLOCKINDEX_OF_SSP0_MODULE                  = 37
,CLOCKINDEX_OF_SSP1_MODULE                  = 38
,CLOCKINDEX_OF_SSP2_MODULE                  = 39
,CLOCKINDEX_OF_VIP2_MODULE                  = 40
};




//------------------------------------------------------------------------------
// DMA peripheral index of modules for the DMA controller.
//------------------------------------------------------------------------------

enum {
//------------------------------------------------------------------------------
// BAGL RENDERING
//------------------------------------------------------------------------------

      DMAINDEX_OF_pl01115_Uart_modem_MODULE_UARTTXDMA   = 0
     ,DMAINDEX_OF_pl01115_Uart_modem_MODULE_UARTRXDMA   = 1
     ,DMAINDEX_OF_UART0_MODULE_UARTTXDMA                = 2
     ,DMAINDEX_OF_UART0_MODULE_UARTRXDMA                = 3
     ,DMAINDEX_OF_UART1_MODULE_UARTTXDMA                = 4
     ,DMAINDEX_OF_UART1_MODULE_UARTRXDMA                = 5
     ,DMAINDEX_OF_SSP0_MODULE_SSPTXDMA                  = 6
     ,DMAINDEX_OF_SSP0_MODULE_SSPRXDMA                  = 7
     ,DMAINDEX_OF_SSP1_MODULE_SSPTXDMA                  = 8
     ,DMAINDEX_OF_SSP1_MODULE_SSPRXDMA                  = 9
     ,DMAINDEX_OF_SSP2_MODULE_SSPTXDMA                  = 10
     ,DMAINDEX_OF_SSP2_MODULE_SSPRXDMA                  = 11
     ,DMAINDEX_OF_I2S0_MODULE_I2STXDMA                  = 12
     ,DMAINDEX_OF_I2S0_MODULE_I2SRXDMA                  = 13
     ,DMAINDEX_OF_I2S1_MODULE_I2STXDMA                  = 14
     ,DMAINDEX_OF_I2S1_MODULE_I2SRXDMA                  = 15
     ,DMAINDEX_OF_I2S2_MODULE_I2STXDMA                  = 16
     ,DMAINDEX_OF_I2S2_MODULE_I2SRXDMA                  = 17
     ,DMAINDEX_OF_AC97_MODULE_PCMOUTDMA                 = 18
     ,DMAINDEX_OF_AC97_MODULE_PCMINDMA                  = 19
     ,DMAINDEX_OF_AC97_MODULE_MICINDMA                  = 20
     ,DMAINDEX_OF_SPDIFRX_MODULE                        = 21
     ,DMAINDEX_OF_SPDIFTX_MODULE                        = 22
     ,DMAINDEX_OF_MPEGTSI_MODULE_MPTSIDMA0              = 23
     ,DMAINDEX_OF_MPEGTSI_MODULE_MPTSIDMA1              = 24
     ,DMAINDEX_OF_MPEGTSI_MODULE_MPTSIDMA2              = 25
     ,DMAINDEX_OF_MPEGTSI_MODULE_MPTSIDMA3              = 26
     ,DMAINDEX_OF_CRYPTO_MODULE_CRYPDMA_BR              = 27
     ,DMAINDEX_OF_CRYPTO_MODULE_CRYPDMA_BW              = 28
     ,DMAINDEX_OF_CRYPTO_MODULE_CRYPDMA_HR              = 29
     ,DMAINDEX_OF_PDM_MODULE                            = 30
    };
//------------------------------------------------------------------------------
// BAGL RENDERING END
//------------------------------------------------------------------------------






//------------------------------------------------------------------------------
// PAD INFORMATION ( design의 PAD 부분을 보고 렌더링 한다. )
//
// #define  PAD_CLKRST_MANAGER_i_BCLK               1
// #define  PAD_SSP_SSPCLK              5
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// BAGL RENDERING
//------------------------------------------------------------------------------

#define PADINDEX_OF_USB20OTG_i_IdPin                ( (0 << 16) | 0 )
#define PADINDEX_OF_USB20OTG_io_VBUS                ( (0 << 16) | 0 )
#define PADINDEX_OF_USB20OTG_io_DM              ( (0 << 16) | 0 )
#define PADINDEX_OF_USB20OTG_io_DP              ( (0 << 16) | 0 )
#define PADINDEX_OF_USB20OTG_io_RKELVIN             ( (0 << 16) | 0 )
#define PADINDEX_OF_USB20HOST_io_DP0                ( (0 << 16) | 0 )
#define PADINDEX_OF_USB20HOST_io_DM0                ( (0 << 16) | 0 )
#define PADINDEX_OF_USB20HOST_io_RKELVIN                ( (0 << 16) | 0 )
#define PADINDEX_OF_USB20HOST_io_STROBE1                ( (0 << 16) | 0 )
#define PADINDEX_OF_USB20HOST_io_DATA1              ( (0 << 16) | 0 )
#define PADINDEX_OF_GPIOA_GPIO_0_               ( (1 << 16) | (0 << 8) | (0 << 3) | 0 )
#define PADINDEX_OF_DISPLAYTOP_o_DualDisplay_PADPrimVCLK                ( (1 << 16) | (0 << 8) | (0 << 3) | 1 )
#define PADINDEX_OF_CLKPWR_i_PADTESTMODE_4_             ( (1 << 16) | (0 << 8) | (0 << 3) | 3 )
#define PADINDEX_OF_GPIOA_GPIO_1_               ( (1 << 16) | (0 << 8) | (1 << 3) | 0 )
#define PADINDEX_OF_DISPLAYTOP_DualDisplay_PrimPADRGB24_0_              ( (1 << 16) | (0 << 8) | (1 << 3) | 1 )
#define PADINDEX_OF_GPIOA_GPIO_2_               ( (1 << 16) | (0 << 8) | (2 << 3) | 0 )
#define PADINDEX_OF_DISPLAYTOP_DualDisplay_PrimPADRGB24_1_              ( (1 << 16) | (0 << 8) | (2 << 3) | 1 )
#define PADINDEX_OF_CLKPWR_i_PADTESTMODE_0_             ( (1 << 16) | (0 << 8) | (2 << 3) | 3 )
#define PADINDEX_OF_GPIOA_GPIO_3_               ( (1 << 16) | (0 << 8) | (3 << 3) | 0 )
#define PADINDEX_OF_DISPLAYTOP_DualDisplay_PrimPADRGB24_2_              ( (1 << 16) | (0 << 8) | (3 << 3) | 1 )
#define PADINDEX_OF_CLKPWR_i_PADTESTMODE_1_             ( (1 << 16) | (0 << 8) | (3 << 3) | 3 )
#define PADINDEX_OF_GPIOA_GPIO_4_               ( (1 << 16) | (0 << 8) | (4 << 3) | 0 )
#define PADINDEX_OF_DISPLAYTOP_DualDisplay_PrimPADRGB24_3_              ( (1 << 16) | (0 << 8) | (4 << 3) | 1 )
#define PADINDEX_OF_CLKPWR_i_PADTESTMODE_2_             ( (1 << 16) | (0 << 8) | (4 << 3) | 3 )
#define PADINDEX_OF_GPIOA_GPIO_5_               ( (1 << 16) | (0 << 8) | (5 << 3) | 0 )
#define PADINDEX_OF_DISPLAYTOP_DualDisplay_PrimPADRGB24_4_              ( (1 << 16) | (0 << 8) | (5 << 3) | 1 )
#define PADINDEX_OF_CLKPWR_i_PADTESTMODE_3_             ( (1 << 16) | (0 << 8) | (5 << 3) | 3 )
#define PADINDEX_OF_GPIOA_GPIO_6_               ( (1 << 16) | (0 << 8) | (6 << 3) | 0 )
#define PADINDEX_OF_DISPLAYTOP_DualDisplay_PrimPADRGB24_5_              ( (1 << 16) | (0 << 8) | (6 << 3) | 1 )
#define PADINDEX_OF_GPIOA_GPIO_7_               ( (1 << 16) | (0 << 8) | (7 << 3) | 0 )
#define PADINDEX_OF_DISPLAYTOP_DualDisplay_PrimPADRGB24_6_              ( (1 << 16) | (0 << 8) | (7 << 3) | 1 )
#define PADINDEX_OF_GPIOA_GPIO_8_               ( (1 << 16) | (0 << 8) | (8 << 3) | 0 )
#define PADINDEX_OF_DISPLAYTOP_DualDisplay_PrimPADRGB24_7_              ( (1 << 16) | (0 << 8) | (8 << 3) | 1 )
#define PADINDEX_OF_GPIOA_GPIO_9_               ( (1 << 16) | (0 << 8) | (9 << 3) | 0 )
#define PADINDEX_OF_DISPLAYTOP_DualDisplay_PrimPADRGB24_8_              ( (1 << 16) | (0 << 8) | (9 << 3) | 1 )
#define PADINDEX_OF_GPIOA_GPIO_10_              ( (1 << 16) | (0 << 8) | (10 << 3) | 0 )
#define PADINDEX_OF_DISPLAYTOP_DualDisplay_PrimPADRGB24_9_              ( (1 << 16) | (0 << 8) | (10 << 3) | 1 )
#define PADINDEX_OF_GPIOA_GPIO_11_              ( (1 << 16) | (0 << 8) | (11 << 3) | 0 )
#define PADINDEX_OF_DISPLAYTOP_DualDisplay_PrimPADRGB24_10_             ( (1 << 16) | (0 << 8) | (11 << 3) | 1 )
#define PADINDEX_OF_GPIOA_GPIO_12_              ( (1 << 16) | (0 << 8) | (12 << 3) | 0 )
#define PADINDEX_OF_DISPLAYTOP_DualDisplay_PrimPADRGB24_11_             ( (1 << 16) | (0 << 8) | (12 << 3) | 1 )
#define PADINDEX_OF_GPIOA_GPIO_13_              ( (1 << 16) | (0 << 8) | (13 << 3) | 0 )
#define PADINDEX_OF_DISPLAYTOP_DualDisplay_PrimPADRGB24_12_             ( (1 << 16) | (0 << 8) | (13 << 3) | 1 )
#define PADINDEX_OF_GPIOA_GPIO_14_              ( (1 << 16) | (0 << 8) | (14 << 3) | 0 )
#define PADINDEX_OF_DISPLAYTOP_DualDisplay_PrimPADRGB24_13_             ( (1 << 16) | (0 << 8) | (14 << 3) | 1 )
#define PADINDEX_OF_GPIOA_GPIO_15_              ( (1 << 16) | (0 << 8) | (15 << 3) | 0 )
#define PADINDEX_OF_DISPLAYTOP_DualDisplay_PrimPADRGB24_14_             ( (1 << 16) | (0 << 8) | (15 << 3) | 1 )
#define PADINDEX_OF_GPIOA_GPIO_16_              ( (1 << 16) | (0 << 8) | (16 << 3) | 0 )
#define PADINDEX_OF_DISPLAYTOP_DualDisplay_PrimPADRGB24_15_             ( (1 << 16) | (0 << 8) | (16 << 3) | 1 )
#define PADINDEX_OF_GPIOA_GPIO_17_              ( (1 << 16) | (0 << 8) | (17 << 3) | 0 )
#define PADINDEX_OF_DISPLAYTOP_DualDisplay_PrimPADRGB24_16_             ( (1 << 16) | (0 << 8) | (17 << 3) | 1 )
#define PADINDEX_OF_GPIOA_GPIO_18_              ( (1 << 16) | (0 << 8) | (18 << 3) | 0 )
#define PADINDEX_OF_DISPLAYTOP_DualDisplay_PrimPADRGB24_17_             ( (1 << 16) | (0 << 8) | (18 << 3) | 1 )
#define PADINDEX_OF_GPIOA_GPIO_19_              ( (1 << 16) | (0 << 8) | (19 << 3) | 0 )
#define PADINDEX_OF_DISPLAYTOP_DualDisplay_PrimPADRGB24_18_             ( (1 << 16) | (0 << 8) | (19 << 3) | 1 )
#define PADINDEX_OF_GPIOA_GPIO_20_              ( (1 << 16) | (0 << 8) | (20 << 3) | 0 )
#define PADINDEX_OF_DISPLAYTOP_DualDisplay_PrimPADRGB24_19_             ( (1 << 16) | (0 << 8) | (20 << 3) | 1 )
#define PADINDEX_OF_GPIOA_GPIO_21_              ( (1 << 16) | (0 << 8) | (21 << 3) | 0 )
#define PADINDEX_OF_DISPLAYTOP_DualDisplay_PrimPADRGB24_20_             ( (1 << 16) | (0 << 8) | (21 << 3) | 1 )
#define PADINDEX_OF_GPIOA_GPIO_22_              ( (1 << 16) | (0 << 8) | (22 << 3) | 0 )
#define PADINDEX_OF_DISPLAYTOP_DualDisplay_PrimPADRGB24_21_             ( (1 << 16) | (0 << 8) | (22 << 3) | 1 )
#define PADINDEX_OF_GPIOA_GPIO_23_              ( (1 << 16) | (0 << 8) | (23 << 3) | 0 )
#define PADINDEX_OF_DISPLAYTOP_DualDisplay_PrimPADRGB24_22_             ( (1 << 16) | (0 << 8) | (23 << 3) | 1 )
#define PADINDEX_OF_GPIOA_GPIO_24_              ( (1 << 16) | (0 << 8) | (24 << 3) | 0 )
#define PADINDEX_OF_DISPLAYTOP_DualDisplay_PrimPADRGB24_23_             ( (1 << 16) | (0 << 8) | (24 << 3) | 1 )
#define PADINDEX_OF_GPIOA_GPIO_25_              ( (1 << 16) | (0 << 8) | (25 << 3) | 0 )
#define PADINDEX_OF_DISPLAYTOP_o_DualDisplay_PrimPADnVSync              ( (1 << 16) | (0 << 8) | (25 << 3) | 1 )
#define PADINDEX_OF_GPIOA_GPIO_26_              ( (1 << 16) | (0 << 8) | (26 << 3) | 0 )
#define PADINDEX_OF_DISPLAYTOP_o_DualDisplay_PrimPADnHSync              ( (1 << 16) | (0 << 8) | (26 << 3) | 1 )
#define PADINDEX_OF_GPIOA_GPIO_27_              ( (1 << 16) | (0 << 8) | (27 << 3) | 0 )
#define PADINDEX_OF_DISPLAYTOP_o_DualDisplay_PrimPADDE              ( (1 << 16) | (0 << 8) | (27 << 3) | 1 )
#define PADINDEX_OF_GPIOA_GPIO_28_              ( (1 << 16) | (0 << 8) | (28 << 3) | 0 )
#define PADINDEX_OF_VIP1_i_ExtCLK               ( (1 << 16) | (0 << 8) | (28 << 3) | 1 )
#define PADINDEX_OF_I2S2_I2SCODCLK              ( (1 << 16) | (0 << 8) | (28 << 3) | 2 )
#define PADINDEX_OF_I2S1_I2SCODCLK              ( (1 << 16) | (0 << 8) | (28 << 3) | 3 )
#define PADINDEX_OF_GPIOA_GPIO_30_              ( (1 << 16) | (0 << 8) | (30 << 3) | 0 )
#define PADINDEX_OF_VIP1_i_VD_0_                ( (1 << 16) | (0 << 8) | (30 << 3) | 1 )
#define PADINDEX_OF_MCUSTOP_SDEX_0_             ( (1 << 16) | (0 << 8) | (30 << 3) | 2 )
#define PADINDEX_OF_I2S1_I2SBCLK                ( (1 << 16) | (0 << 8) | (30 << 3) | 3 )
#define PADINDEX_OF_GPIOB_GPIO_0_               ( (1 << 16) | (1 << 8) | (0 << 3) | 0 )
#define PADINDEX_OF_VIP1_i_VD_1_                ( (1 << 16) | (1 << 8) | (0 << 3) | 1 )
#define PADINDEX_OF_MCUSTOP_SDEX_1_             ( (1 << 16) | (1 << 8) | (0 << 3) | 2 )
#define PADINDEX_OF_I2S1_I2SLRCLK               ( (1 << 16) | (1 << 8) | (0 << 3) | 3 )
#define PADINDEX_OF_GPIOB_GPIO_2_               ( (1 << 16) | (1 << 8) | (2 << 3) | 0 )
#define PADINDEX_OF_VIP1_i_VD_2_                ( (1 << 16) | (1 << 8) | (2 << 3) | 1 )
#define PADINDEX_OF_MCUSTOP_SDEX_2_             ( (1 << 16) | (1 << 8) | (2 << 3) | 2 )
#define PADINDEX_OF_I2S2_I2SBCLK                ( (1 << 16) | (1 << 8) | (2 << 3) | 3 )
#define PADINDEX_OF_GPIOB_GPIO_4_               ( (1 << 16) | (1 << 8) | (4 << 3) | 0 )
#define PADINDEX_OF_VIP1_i_VD_3_                ( (1 << 16) | (1 << 8) | (4 << 3) | 1 )
#define PADINDEX_OF_MCUSTOP_SDEX_3_             ( (1 << 16) | (1 << 8) | (4 << 3) | 2 )
#define PADINDEX_OF_I2S2_I2SLRCLK               ( (1 << 16) | (1 << 8) | (4 << 3) | 3 )
#define PADINDEX_OF_GPIOB_GPIO_6_               ( (1 << 16) | (1 << 8) | (6 << 3) | 0 )
#define PADINDEX_OF_VIP1_i_VD_4_                ( (1 << 16) | (1 << 8) | (6 << 3) | 1 )
#define PADINDEX_OF_MCUSTOP_SDEX_4_             ( (1 << 16) | (1 << 8) | (6 << 3) | 2 )
#define PADINDEX_OF_I2S1_I2SSDO             ( (1 << 16) | (1 << 8) | (6 << 3) | 3 )
#define PADINDEX_OF_GPIOB_GPIO_8_               ( (1 << 16) | (1 << 8) | (8 << 3) | 0 )
#define PADINDEX_OF_VIP1_i_VD_5_                ( (1 << 16) | (1 << 8) | (8 << 3) | 1 )
#define PADINDEX_OF_MCUSTOP_SDEX_5_             ( (1 << 16) | (1 << 8) | (8 << 3) | 2 )
#define PADINDEX_OF_I2S2_I2SSDO             ( (1 << 16) | (1 << 8) | (8 << 3) | 3 )
#define PADINDEX_OF_GPIOB_GPIO_9_               ( (1 << 16) | (1 << 8) | (9 << 3) | 0 )
#define PADINDEX_OF_VIP1_i_VD_6_                ( (1 << 16) | (1 << 8) | (9 << 3) | 1 )
#define PADINDEX_OF_MCUSTOP_SDEX_6_             ( (1 << 16) | (1 << 8) | (9 << 3) | 2 )
#define PADINDEX_OF_I2S1_I2SSDI             ( (1 << 16) | (1 << 8) | (9 << 3) | 3 )
#define PADINDEX_OF_GPIOB_GPIO_10_              ( (1 << 16) | (1 << 8) | (10 << 3) | 0 )
#define PADINDEX_OF_VIP1_i_VD_7_                ( (1 << 16) | (1 << 8) | (10 << 3) | 1 )
#define PADINDEX_OF_MCUSTOP_SDEX_7_             ( (1 << 16) | (1 << 8) | (10 << 3) | 2 )
#define PADINDEX_OF_I2S2_I2SSDI             ( (1 << 16) | (1 << 8) | (10 << 3) | 3 )
#define PADINDEX_OF_GPIOA_GPIO_29_              ( (1 << 16) | (0 << 8) | (29 << 3) | 0 )
#define PADINDEX_OF_SDMMC0_SDMMC_CCLK               ( (1 << 16) | (0 << 8) | (29 << 3) | 1 )
#define PADINDEX_OF_GPIOA_GPIO_31_              ( (1 << 16) | (0 << 8) | (31 << 3) | 0 )
#define PADINDEX_OF_SDMMC0_SDMMC_CMD                ( (1 << 16) | (0 << 8) | (31 << 3) | 1 )
#define PADINDEX_OF_GPIOB_GPIO_1_               ( (1 << 16) | (1 << 8) | (1 << 3) | 0 )
#define PADINDEX_OF_SDMMC0_SDMMC_CDATA_0_               ( (1 << 16) | (1 << 8) | (1 << 3) | 1 )
#define PADINDEX_OF_GPIOB_GPIO_3_               ( (1 << 16) | (1 << 8) | (3 << 3) | 0 )
#define PADINDEX_OF_SDMMC0_SDMMC_CDATA_1_               ( (1 << 16) | (1 << 8) | (3 << 3) | 1 )
#define PADINDEX_OF_GPIOB_GPIO_5_               ( (1 << 16) | (1 << 8) | (5 << 3) | 0 )
#define PADINDEX_OF_SDMMC0_SDMMC_CDATA_2_               ( (1 << 16) | (1 << 8) | (5 << 3) | 1 )
#define PADINDEX_OF_GPIOB_GPIO_7_               ( (1 << 16) | (1 << 8) | (7 << 3) | 0 )
#define PADINDEX_OF_SDMMC0_SDMMC_CDATA_3_               ( (1 << 16) | (1 << 8) | (7 << 3) | 1 )
#define PADINDEX_OF_MCUSTOP_CLE             ( (1 << 16) | (1 << 8) | (11 << 3) | 0 )
#define PADINDEX_OF_MCUSTOP_CLE1                ( (1 << 16) | (1 << 8) | (11 << 3) | 1 )
#define PADINDEX_OF_GPIOB_GPIO_11_              ( (1 << 16) | (1 << 8) | (11 << 3) | 2 )
#define PADINDEX_OF_MCUSTOP_ALE             ( (1 << 16) | (1 << 8) | (12 << 3) | 0 )
#define PADINDEX_OF_MCUSTOP_ALE1                ( (1 << 16) | (1 << 8) | (12 << 3) | 1 )
#define PADINDEX_OF_GPIOB_GPIO_12_              ( (1 << 16) | (1 << 8) | (12 << 3) | 2 )
#define PADINDEX_OF_MCUSTOP_SD_0_               ( (1 << 16) | (1 << 8) | (13 << 3) | 0 )
#define PADINDEX_OF_GPIOB_GPIO_13_              ( (1 << 16) | (1 << 8) | (13 << 3) | 1 )
#define PADINDEX_OF_MCUSTOP_RnB             ( (1 << 16) | (1 << 8) | (14 << 3) | 0 )
#define PADINDEX_OF_MCUSTOP_RnB1                ( (1 << 16) | (1 << 8) | (14 << 3) | 1 )
#define PADINDEX_OF_GPIOB_GPIO_14_              ( (1 << 16) | (1 << 8) | (14 << 3) | 2 )
#define PADINDEX_OF_MCUSTOP_SD_1_               ( (1 << 16) | (1 << 8) | (15 << 3) | 0 )
#define PADINDEX_OF_GPIOB_GPIO_15_              ( (1 << 16) | (1 << 8) | (15 << 3) | 1 )
#define PADINDEX_OF_MCUSTOP_nNFOE               ( (1 << 16) | (1 << 8) | (16 << 3) | 0 )
#define PADINDEX_OF_MCUSTOP_nNFOE1              ( (1 << 16) | (1 << 8) | (16 << 3) | 1 )
#define PADINDEX_OF_GPIOB_GPIO_16_              ( (1 << 16) | (1 << 8) | (16 << 3) | 2 )
#define PADINDEX_OF_MCUSTOP_SD_2_               ( (1 << 16) | (1 << 8) | (17 << 3) | 0 )
#define PADINDEX_OF_GPIOB_GPIO_17_              ( (1 << 16) | (1 << 8) | (17 << 3) | 1 )
#define PADINDEX_OF_MCUSTOP_nNFWE               ( (1 << 16) | (1 << 8) | (18 << 3) | 0 )
#define PADINDEX_OF_MCUSTOP_nNFWE1              ( (1 << 16) | (1 << 8) | (18 << 3) | 1 )
#define PADINDEX_OF_GPIOB_GPIO_18_              ( (1 << 16) | (1 << 8) | (18 << 3) | 2 )
#define PADINDEX_OF_MCUSTOP_SD_3_               ( (1 << 16) | (1 << 8) | (19 << 3) | 0 )
#define PADINDEX_OF_GPIOB_GPIO_19_              ( (1 << 16) | (1 << 8) | (19 << 3) | 1 )
#define PADINDEX_OF_MCUSTOP_nNCS_0_             ( (0 << 16) | 0 )
#define PADINDEX_OF_MCUSTOP_SD_4_               ( (1 << 16) | (1 << 8) | (20 << 3) | 0 )
#define PADINDEX_OF_GPIOB_GPIO_20_              ( (1 << 16) | (1 << 8) | (20 << 3) | 1 )
#define PADINDEX_OF_MCUSTOP_nNCS_1_             ( (0 << 16) | 0 )
#define PADINDEX_OF_MCUSTOP_SD_5_               ( (1 << 16) | (1 << 8) | (21 << 3) | 0 )
#define PADINDEX_OF_GPIOB_GPIO_21_              ( (1 << 16) | (1 << 8) | (21 << 3) | 1 )
#define PADINDEX_OF_MCUSTOP_SD_6_               ( (1 << 16) | (1 << 8) | (22 << 3) | 0 )
#define PADINDEX_OF_GPIOB_GPIO_22_              ( (1 << 16) | (1 << 8) | (22 << 3) | 1 )
#define PADINDEX_OF_MCUSTOP_SD_7_               ( (1 << 16) | (1 << 8) | (23 << 3) | 0 )
#define PADINDEX_OF_GPIOB_GPIO_23_              ( (1 << 16) | (1 << 8) | (23 << 3) | 1 )
#define PADINDEX_OF_MCUSTOP_SD_8_               ( (1 << 16) | (1 << 8) | (24 << 3) | 0 )
#define PADINDEX_OF_GPIOB_GPIO_24_              ( (1 << 16) | (1 << 8) | (24 << 3) | 1 )
#define PADINDEX_OF_MPEGTSI_TDATA0_0_               ( (1 << 16) | (1 << 8) | (24 << 3) | 2 )
#define PADINDEX_OF_MCUSTOP_SD_9_               ( (1 << 16) | (1 << 8) | (25 << 3) | 0 )
#define PADINDEX_OF_GPIOB_GPIO_25_              ( (1 << 16) | (1 << 8) | (25 << 3) | 1 )
#define PADINDEX_OF_MPEGTSI_TDATA0_1_               ( (1 << 16) | (1 << 8) | (25 << 3) | 2 )
#define PADINDEX_OF_MCUSTOP_SD_10_              ( (1 << 16) | (1 << 8) | (26 << 3) | 0 )
#define PADINDEX_OF_GPIOB_GPIO_26_              ( (1 << 16) | (1 << 8) | (26 << 3) | 1 )
#define PADINDEX_OF_MPEGTSI_TDATA0_2_               ( (1 << 16) | (1 << 8) | (26 << 3) | 2 )
#define PADINDEX_OF_ECID_PAD_BONDING_ID_2_              ( (1 << 16) | (1 << 8) | (26 << 3) | 3 )
#define PADINDEX_OF_MCUSTOP_SD_11_              ( (1 << 16) | (1 << 8) | (27 << 3) | 0 )
#define PADINDEX_OF_GPIOB_GPIO_27_              ( (1 << 16) | (1 << 8) | (27 << 3) | 1 )
#define PADINDEX_OF_MPEGTSI_TDATA0_3_               ( (1 << 16) | (1 << 8) | (27 << 3) | 2 )
#define PADINDEX_OF_MCUSTOP_SD_12_              ( (1 << 16) | (1 << 8) | (28 << 3) | 0 )
#define PADINDEX_OF_GPIOB_GPIO_28_              ( (1 << 16) | (1 << 8) | (28 << 3) | 1 )
#define PADINDEX_OF_MPEGTSI_TDATA0_4_               ( (1 << 16) | (1 << 8) | (28 << 3) | 2 )
#define PADINDEX_OF_pl01115_Uart_nodma1_UARTRXD             ( (1 << 16) | (1 << 8) | (28 << 3) | 3 )
#define PADINDEX_OF_MCUSTOP_SD_13_              ( (1 << 16) | (1 << 8) | (29 << 3) | 0 )
#define PADINDEX_OF_GPIOB_GPIO_29_              ( (1 << 16) | (1 << 8) | (29 << 3) | 1 )
#define PADINDEX_OF_MPEGTSI_TDATA0_5_               ( (1 << 16) | (1 << 8) | (29 << 3) | 2 )
#define PADINDEX_OF_pl01115_Uart_nodma1_UARTTXD             ( (1 << 16) | (1 << 8) | (29 << 3) | 3 )
#define PADINDEX_OF_MCUSTOP_SD_14_              ( (1 << 16) | (1 << 8) | (30 << 3) | 0 )
#define PADINDEX_OF_GPIOB_GPIO_30_              ( (1 << 16) | (1 << 8) | (30 << 3) | 1 )
#define PADINDEX_OF_MPEGTSI_TDATA0_6_               ( (1 << 16) | (1 << 8) | (30 << 3) | 2 )
#define PADINDEX_OF_pl01115_Uart_nodma2_UARTRXD             ( (1 << 16) | (1 << 8) | (30 << 3) | 3 )
#define PADINDEX_OF_MCUSTOP_SD_15_              ( (1 << 16) | (1 << 8) | (31 << 3) | 0 )
#define PADINDEX_OF_GPIOB_GPIO_31_              ( (1 << 16) | (1 << 8) | (31 << 3) | 1 )
#define PADINDEX_OF_MPEGTSI_TDATA0_7_               ( (1 << 16) | (1 << 8) | (31 << 3) | 2 )
#define PADINDEX_OF_pl01115_Uart_nodma2_UARTTXD             ( (1 << 16) | (1 << 8) | (31 << 3) | 3 )

#define PADINDEX_OF_MCUSTOP_o_ADDR_0_               ( (1 << 16) | (2 << 8) | (0 << 3) | 0 )
#define PADINDEX_OF_GPIOC_GPIO_0_                   ( (1 << 16) | (2 << 8) | (0 << 3) | 1 )
// @added - mpegtsi eco (add error port)
#define PADINDEX_OF_MPEGTSI_TERR0                   ( (1 << 16) | (2 << 8) | (0 << 3) | 2 )

#define PADINDEX_OF_MCUSTOP_o_ADDR_1_               ( (1 << 16) | (2 << 8) | (1 << 3) | 0 )
#define PADINDEX_OF_GPIOC_GPIO_1_                   ( (1 << 16) | (2 << 8) | (1 << 3) | 1 )
// @added - mpegtsi eco (add error port)
#define PADINDEX_OF_MPEGTSI_TERR1                   ( (1 << 16) | (2 << 8) | (1 << 3) | 2 )

#define PADINDEX_OF_MCUSTOP_o_ADDR_2_               ( (1 << 16) | (2 << 8) | (2 << 3) | 0 )
#define PADINDEX_OF_GPIOC_GPIO_2_               ( (1 << 16) | (2 << 8) | (2 << 3) | 1 )
#define PADINDEX_OF_MCUSTOP_o_ADDR_3_               ( (1 << 16) | (2 << 8) | (3 << 3) | 0 )
#define PADINDEX_OF_GPIOC_GPIO_3_               ( (1 << 16) | (2 << 8) | (3 << 3) | 1 )
#define PADINDEX_OF_DISPLAYTOP_io_HDMI_CEC              ( (1 << 16) | (2 << 8) | (3 << 3) | 2 )
#define PADINDEX_OF_SDMMC0_SDMMC_nRST               ( (1 << 16) | (2 << 8) | (3 << 3) | 3 )
#define PADINDEX_OF_MCUSTOP_o_ADDR_4_               ( (1 << 16) | (2 << 8) | (4 << 3) | 0 )
#define PADINDEX_OF_GPIOC_GPIO_4_               ( (1 << 16) | (2 << 8) | (4 << 3) | 1 )
#define PADINDEX_OF_pl01115_Uart_modem_nUARTDCD             ( (1 << 16) | (2 << 8) | (4 << 3) | 2 )
#define PADINDEX_OF_SDMMC0_SDMMC_CARD_nInt              ( (1 << 16) | (2 << 8) | (4 << 3) | 3 )
#define PADINDEX_OF_MCUSTOP_o_ADDR_5_               ( (1 << 16) | (2 << 8) | (5 << 3) | 0 )
#define PADINDEX_OF_GPIOC_GPIO_5_               ( (1 << 16) | (2 << 8) | (5 << 3) | 1 )
#define PADINDEX_OF_pl01115_Uart_modem_nUARTCTS             ( (1 << 16) | (2 << 8) | (5 << 3) | 2 )
#define PADINDEX_OF_SDMMC0_SDMMC_CARD_WritePrt              ( (1 << 16) | (2 << 8) | (5 << 3) | 3 )
#define PADINDEX_OF_MCUSTOP_o_ADDR_6_               ( (1 << 16) | (2 << 8) | (6 << 3) | 0 )
#define PADINDEX_OF_GPIOC_GPIO_6_               ( (1 << 16) | (2 << 8) | (6 << 3) | 1 )
#define PADINDEX_OF_pl01115_Uart_modem_nUARTRTS             ( (1 << 16) | (2 << 8) | (6 << 3) | 2 )
#define PADINDEX_OF_SDMMC0_SDMMC_CARD_nDetect               ( (1 << 16) | (2 << 8) | (6 << 3) | 3 )
#define PADINDEX_OF_MCUSTOP_o_ADDR_7_               ( (1 << 16) | (2 << 8) | (7 << 3) | 0 )
#define PADINDEX_OF_GPIOC_GPIO_7_               ( (1 << 16) | (2 << 8) | (7 << 3) | 1 )
#define PADINDEX_OF_pl01115_Uart_modem_nUARTDSR             ( (1 << 16) | (2 << 8) | (7 << 3) | 2 )
#define PADINDEX_OF_SDMMC1_SDMMC_nRST               ( (1 << 16) | (2 << 8) | (7 << 3) | 3 )
#define PADINDEX_OF_MCUSTOP_o_ADDR_8_               ( (1 << 16) | (2 << 8) | (8 << 3) | 0 )
#define PADINDEX_OF_GPIOC_GPIO_8_               ( (1 << 16) | (2 << 8) | (8 << 3) | 1 )
#define PADINDEX_OF_pl01115_Uart_modem_nUARTDTR             ( (1 << 16) | (2 << 8) | (8 << 3) | 2 )
#define PADINDEX_OF_SDMMC1_SDMMC_CARD_nInt              ( (1 << 16) | (2 << 8) | (8 << 3) | 3 )
#define PADINDEX_OF_MCUSTOP_o_ADDR_9_               ( (1 << 16) | (2 << 8) | (9 << 3) | 0 )
#define PADINDEX_OF_GPIOC_GPIO_9_               ( (1 << 16) | (2 << 8) | (9 << 3) | 1 )
#define PADINDEX_OF_SSP2_SSPCLK_IO              ( (1 << 16) | (2 << 8) | (9 << 3) | 2 )
#define PADINDEX_OF_PDM_o_Strobe                ( (1 << 16) | (2 << 8) | (9 << 3) | 3 )
#define PADINDEX_OF_MCUSTOP_o_ADDR_10_              ( (1 << 16) | (2 << 8) | (10 << 3) | 0 )
#define PADINDEX_OF_GPIOC_GPIO_10_              ( (1 << 16) | (2 << 8) | (10 << 3) | 1 )
#define PADINDEX_OF_SSP2_SSPFSS             ( (1 << 16) | (2 << 8) | (10 << 3) | 2 )
#define PADINDEX_OF_MCUSTOP_nNCS_2_             ( (1 << 16) | (2 << 8) | (10 << 3) | 3 )
#define PADINDEX_OF_MCUSTOP_o_ADDR_11_              ( (1 << 16) | (2 << 8) | (11 << 3) | 0 )
#define PADINDEX_OF_GPIOC_GPIO_11_              ( (1 << 16) | (2 << 8) | (11 << 3) | 1 )
#define PADINDEX_OF_SSP2_SSPRXD             ( (1 << 16) | (2 << 8) | (11 << 3) | 2 )
#define PADINDEX_OF_USB20OTG_o_DrvVBUS              ( (1 << 16) | (2 << 8) | (11 << 3) | 3 )
#define PADINDEX_OF_MCUSTOP_o_ADDR_12_              ( (1 << 16) | (2 << 8) | (12 << 3) | 0 )
#define PADINDEX_OF_GPIOC_GPIO_12_              ( (1 << 16) | (2 << 8) | (12 << 3) | 1 )
#define PADINDEX_OF_SSP2_SSPTXD             ( (1 << 16) | (2 << 8) | (12 << 3) | 2 )
#define PADINDEX_OF_SDMMC2_SDMMC_nRST               ( (1 << 16) | (2 << 8) | (12 << 3) | 3 )
#define PADINDEX_OF_MCUSTOP_o_ADDR_13_              ( (1 << 16) | (2 << 8) | (13 << 3) | 0 )
#define PADINDEX_OF_GPIOC_GPIO_13_              ( (1 << 16) | (2 << 8) | (13 << 3) | 1 )
#define PADINDEX_OF_PWM_TOUT1               ( (1 << 16) | (2 << 8) | (13 << 3) | 2 )
#define PADINDEX_OF_SDMMC2_SDMMC_CARD_nInt              ( (1 << 16) | (2 << 8) | (13 << 3) | 3 )
#define PADINDEX_OF_GPIOD_GPIO_18_              ( (1 << 16) | (3 << 8) | (18 << 3) | 0 )
#define PADINDEX_OF_UART0_UARTTXD               ( (1 << 16) | (3 << 8) | (18 << 3) | 1 )
#define PADINDEX_OF_pl01115_Uart_nodma0_SMCAYEN             ( (1 << 16) | (3 << 8) | (18 << 3) | 2 )
#define PADINDEX_OF_SDMMC2_SDMMC_CARD_WritePrt              ( (1 << 16) | (3 << 8) | (18 << 3) | 3 )
#define PADINDEX_OF_GPIOD_GPIO_19_              ( (1 << 16) | (3 << 8) | (19 << 3) | 0 )
#define PADINDEX_OF_pl01115_Uart_modem_UARTTXD              ( (1 << 16) | (3 << 8) | (19 << 3) | 1 )
#define PADINDEX_OF_UART0_SMCAYEN               ( (1 << 16) | (3 << 8) | (19 << 3) | 2 )
#define PADINDEX_OF_SDMMC2_SDMMC_CARD_nDetect               ( (1 << 16) | (3 << 8) | (19 << 3) | 3 )
#define PADINDEX_OF_GPIOD_GPIO_20_              ( (1 << 16) | (3 << 8) | (20 << 3) | 0 )
#define PADINDEX_OF_UART1_UARTTXD               ( (1 << 16) | (3 << 8) | (20 << 3) | 1 )
#define PADINDEX_OF_CAN0_o_TX               ( (1 << 16) | (3 << 8) | (20 << 3) | 2 )
#define PADINDEX_OF_SDMMC1_SDMMC_CARD_WritePrt              ( (1 << 16) | (3 << 8) | (20 << 3) | 3 )
#define PADINDEX_OF_GPIOD_GPIO_21_              ( (1 << 16) | (3 << 8) | (21 << 3) | 0 )
#define PADINDEX_OF_pl01115_Uart_nodma0_UARTTXD             ( (1 << 16) | (3 << 8) | (21 << 3) | 1 )
#define PADINDEX_OF_CAN1_o_TX               ( (1 << 16) | (3 << 8) | (21 << 3) | 2 )
#define PADINDEX_OF_SDMMC1_SDMMC_CARD_nDetect               ( (1 << 16) | (3 << 8) | (21 << 3) | 3 )
#define PADINDEX_OF_GPIOD_GPIO_22_              ( (1 << 16) | (3 << 8) | (22 << 3) | 0 )
#define PADINDEX_OF_SDMMC1_SDMMC_CCLK               ( (1 << 16) | (3 << 8) | (22 << 3) | 1 )
#define PADINDEX_OF_GPIOD_GPIO_23_              ( (1 << 16) | (3 << 8) | (23 << 3) | 0 )
#define PADINDEX_OF_SDMMC1_SDMMC_CMD                ( (1 << 16) | (3 << 8) | (23 << 3) | 1 )
#define PADINDEX_OF_GPIOD_GPIO_24_              ( (1 << 16) | (3 << 8) | (24 << 3) | 0 )
#define PADINDEX_OF_SDMMC1_SDMMC_CDATA_0_               ( (1 << 16) | (3 << 8) | (24 << 3) | 1 )
#define PADINDEX_OF_GPIOD_GPIO_25_              ( (1 << 16) | (3 << 8) | (25 << 3) | 0 )
#define PADINDEX_OF_SDMMC1_SDMMC_CDATA_1_               ( (1 << 16) | (3 << 8) | (25 << 3) | 1 )
#define PADINDEX_OF_GPIOD_GPIO_26_              ( (1 << 16) | (3 << 8) | (26 << 3) | 0 )
#define PADINDEX_OF_SDMMC1_SDMMC_CDATA_2_               ( (1 << 16) | (3 << 8) | (26 << 3) | 1 )
#define PADINDEX_OF_GPIOD_GPIO_27_              ( (1 << 16) | (3 << 8) | (27 << 3) | 0 )
#define PADINDEX_OF_SDMMC1_SDMMC_CDATA_3_               ( (1 << 16) | (3 << 8) | (27 << 3) | 1 )
#define PADINDEX_OF_MCUSTOP_nSWAIT              ( (1 << 16) | (2 << 8) | (25 << 3) | 0 )
#define PADINDEX_OF_GPIOC_GPIO_25_              ( (1 << 16) | (2 << 8) | (25 << 3) | 1 )
#define PADINDEX_OF_SPDIFTX_SPDIF_DATA              ( (1 << 16) | (2 << 8) | (25 << 3) | 2 )
#define PADINDEX_OF_MCUSTOP_nSOE                ( (1 << 16) | (4 << 8) | (30 << 3) | 0 )
#define PADINDEX_OF_GPIOE_GPIO_30_              ( (1 << 16) | (4 << 8) | (30 << 3) | 1 )
#define PADINDEX_OF_MCUSTOP_nSWE                ( (1 << 16) | (4 << 8) | (31 << 3) | 0 )
#define PADINDEX_OF_GPIOE_GPIO_31_              ( (1 << 16) | (4 << 8) | (31 << 3) | 1 )
#define PADINDEX_OF_MCUSTOP_RDnWR               ( (1 << 16) | (2 << 8) | (26 << 3) | 0 )
#define PADINDEX_OF_GPIOC_GPIO_26_              ( (1 << 16) | (2 << 8) | (26 << 3) | 1 )
#define PADINDEX_OF_PDM_i_Data0             ( (1 << 16) | (2 << 8) | (26 << 3) | 2 )
#define PADINDEX_OF_MCUSTOP_nSDQM1              ( (1 << 16) | (2 << 8) | (27 << 3) | 0 )
#define PADINDEX_OF_GPIOC_GPIO_27_              ( (1 << 16) | (2 << 8) | (27 << 3) | 1 )
#define PADINDEX_OF_PDM_i_Data1             ( (1 << 16) | (2 << 8) | (27 << 3) | 2 )
#define PADINDEX_OF_MCUSTOP_nSCS_0_             ( (0 << 16) | 0 )
#define PADINDEX_OF_GPIOC_GPIO_28_              ( (1 << 16) | (2 << 8) | (28 << 3) | 0 )
#define PADINDEX_OF_MCUSTOP_nSCS_1_             ( (1 << 16) | (2 << 8) | (28 << 3) | 1 )
#define PADINDEX_OF_pl01115_Uart_modem_nUARTRI              ( (1 << 16) | (2 << 8) | (28 << 3) | 2 )
#define PADINDEX_OF_GPIOC_GPIO_29_              ( (1 << 16) | (2 << 8) | (29 << 3) | 0 )
#define PADINDEX_OF_SSP0_SSPCLK_IO              ( (1 << 16) | (2 << 8) | (29 << 3) | 1 )
#define PADINDEX_OF_GPIOC_GPIO_30_              ( (1 << 16) | (2 << 8) | (30 << 3) | 0 )
#define PADINDEX_OF_SSP0_SSPFSS             ( (1 << 16) | (2 << 8) | (30 << 3) | 1 )
#define PADINDEX_OF_GPIOC_GPIO_31_              ( (1 << 16) | (2 << 8) | (31 << 3) | 0 )
#define PADINDEX_OF_SSP0_SSPTXD             ( (1 << 16) | (2 << 8) | (31 << 3) | 1 )
#define PADINDEX_OF_GPIOD_GPIO_0_               ( (1 << 16) | (3 << 8) | (0 << 3) | 0 )
#define PADINDEX_OF_SSP0_SSPRXD             ( (1 << 16) | (3 << 8) | (0 << 3) | 1 )
#define PADINDEX_OF_PWM_TOUT3               ( (1 << 16) | (3 << 8) | (0 << 3) | 2 )
#define PADINDEX_OF_GPIOD_GPIO_1_               ( (1 << 16) | (3 << 8) | (1 << 3) | 0 )
#define PADINDEX_OF_PWM_TOUT0               ( (1 << 16) | (3 << 8) | (1 << 3) | 1 )
#define PADINDEX_OF_MCUSTOP_o_ADDR_25_              ( (1 << 16) | (3 << 8) | (1 << 3) | 2 )
#define PADINDEX_OF_GPIOD_GPIO_2_               ( (1 << 16) | (3 << 8) | (2 << 3) | 0 )
#define PADINDEX_OF_I2C0_SCL                ( (1 << 16) | (3 << 8) | (2 << 3) | 1 )
#define PADINDEX_OF_pl01115_Uart_nodma1_SMCAYEN             ( (1 << 16) | (3 << 8) | (2 << 3) | 2 )
#define PADINDEX_OF_GPIOD_GPIO_3_               ( (1 << 16) | (3 << 8) | (3 << 3) | 0 )
#define PADINDEX_OF_I2C0_SDA                ( (1 << 16) | (3 << 8) | (3 << 3) | 1 )
#define PADINDEX_OF_pl01115_Uart_nodma2_SMCAYEN             ( (1 << 16) | (3 << 8) | (3 << 3) | 2 )
#define PADINDEX_OF_GPIOD_GPIO_4_               ( (1 << 16) | (3 << 8) | (4 << 3) | 0 )
#define PADINDEX_OF_I2C1_SCL                ( (1 << 16) | (3 << 8) | (4 << 3) | 1 )
#define PADINDEX_OF_GPIOD_GPIO_5_               ( (1 << 16) | (3 << 8) | (5 << 3) | 0 )
#define PADINDEX_OF_I2C1_SDA                ( (1 << 16) | (3 << 8) | (5 << 3) | 1 )
#define PADINDEX_OF_GPIOD_GPIO_6_               ( (1 << 16) | (3 << 8) | (6 << 3) | 0 )
#define PADINDEX_OF_I2C2_SCL                ( (1 << 16) | (3 << 8) | (6 << 3) | 1 )
#define PADINDEX_OF_GPIOD_GPIO_7_               ( (1 << 16) | (3 << 8) | (7 << 3) | 0 )
#define PADINDEX_OF_I2C2_SDA                ( (1 << 16) | (3 << 8) | (7 << 3) | 1 )
#define PADINDEX_OF_GPIOD_GPIO_8_               ( (1 << 16) | (3 << 8) | (8 << 3) | 0 )
#define PADINDEX_OF_PPM_i_PPMIn             ( (1 << 16) | (3 << 8) | (8 << 3) | 1 )
#define PADINDEX_OF_GPIOD_GPIO_9_               ( (1 << 16) | (3 << 8) | (9 << 3) | 0 )
#define PADINDEX_OF_I2S0_I2SSDO             ( (1 << 16) | (3 << 8) | (9 << 3) | 1 )
#define PADINDEX_OF_AC97_ACSDATAOUT             ( (1 << 16) | (3 << 8) | (9 << 3) | 2 )
#define PADINDEX_OF_GPIOD_GPIO_10_              ( (1 << 16) | (3 << 8) | (10 << 3) | 0 )
#define PADINDEX_OF_I2S0_I2SBCLK                ( (1 << 16) | (3 << 8) | (10 << 3) | 1 )
#define PADINDEX_OF_AC97_ACBITCLK               ( (1 << 16) | (3 << 8) | (10 << 3) | 2 )
#define PADINDEX_OF_GPIOD_GPIO_11_              ( (1 << 16) | (3 << 8) | (11 << 3) | 0 )
#define PADINDEX_OF_I2S0_I2SSDI             ( (1 << 16) | (3 << 8) | (11 << 3) | 1 )
#define PADINDEX_OF_AC97_ACSDATAIN              ( (1 << 16) | (3 << 8) | (11 << 3) | 2 )
#define PADINDEX_OF_GPIOD_GPIO_12_              ( (1 << 16) | (3 << 8) | (12 << 3) | 0 )
#define PADINDEX_OF_I2S0_I2SLRCLK               ( (1 << 16) | (3 << 8) | (12 << 3) | 1 )
#define PADINDEX_OF_AC97_ACSYNC             ( (1 << 16) | (3 << 8) | (12 << 3) | 2 )
#define PADINDEX_OF_GPIOD_GPIO_13_              ( (1 << 16) | (3 << 8) | (13 << 3) | 0 )
#define PADINDEX_OF_I2S0_I2SCODCLK              ( (1 << 16) | (3 << 8) | (13 << 3) | 1 )
#define PADINDEX_OF_AC97_nACRESET               ( (1 << 16) | (3 << 8) | (13 << 3) | 2 )
#define PADINDEX_OF_GPIOD_GPIO_14_              ( (1 << 16) | (3 << 8) | (14 << 3) | 0 )
#define PADINDEX_OF_UART0_UARTRXD               ( (1 << 16) | (3 << 8) | (14 << 3) | 1 )
#define PADINDEX_OF_pl01115_Uart_modem_SMCAYEN              ( (1 << 16) | (3 << 8) | (14 << 3) | 2 )
#define PADINDEX_OF_GPIOD_GPIO_15_              ( (1 << 16) | (3 << 8) | (15 << 3) | 0 )
#define PADINDEX_OF_pl01115_Uart_modem_UARTRXD              ( (1 << 16) | (3 << 8) | (15 << 3) | 1 )
#define PADINDEX_OF_UART1_SMCAYEN               ( (1 << 16) | (3 << 8) | (15 << 3) | 2 )
#define PADINDEX_OF_GPIOD_GPIO_16_              ( (1 << 16) | (3 << 8) | (16 << 3) | 0 )
#define PADINDEX_OF_UART1_UARTRXD               ( (1 << 16) | (3 << 8) | (16 << 3) | 1 )
#define PADINDEX_OF_CAN0_i_RX               ( (1 << 16) | (3 << 8) | (16 << 3) | 2 )
#define PADINDEX_OF_GPIOD_GPIO_17_              ( (1 << 16) | (3 << 8) | (17 << 3) | 0 )
#define PADINDEX_OF_pl01115_Uart_nodma0_UARTRXD             ( (1 << 16) | (3 << 8) | (17 << 3) | 1 )
#define PADINDEX_OF_CAN1_i_RX               ( (1 << 16) | (3 << 8) | (17 << 3) | 2 )
#define PADINDEX_OF_USB20OTG_i_VBUS             ( (0 << 16) | 0 )
#define PADINDEX_OF_DISPLAYTOP_i_HDMI_hotplug_5V                ( (0 << 16) | 0 )
#define PADINDEX_OF_MCUSTOP_o_ADDR_14_              ( (1 << 16) | (2 << 8) | (14 << 3) | 0 )
#define PADINDEX_OF_GPIOC_GPIO_14_              ( (1 << 16) | (2 << 8) | (14 << 3) | 1 )
#define PADINDEX_OF_PWM_TOUT2               ( (1 << 16) | (2 << 8) | (14 << 3) | 2 )
#define PADINDEX_OF_VIP1_i_ExtCLK2              ( (1 << 16) | (2 << 8) | (14 << 3) | 3 )
#define PADINDEX_OF_MCUSTOP_o_ADDR_15_              ( (1 << 16) | (2 << 8) | (15 << 3) | 0 )
#define PADINDEX_OF_GPIOC_GPIO_15_              ( (1 << 16) | (2 << 8) | (15 << 3) | 1 )
#define PADINDEX_OF_MPEGTSI_TSCLK0              ( (1 << 16) | (2 << 8) | (15 << 3) | 2 )
#define PADINDEX_OF_VIP1_i_ExtHSYNC2                ( (1 << 16) | (2 << 8) | (15 << 3) | 3 )
#define PADINDEX_OF_MCUSTOP_o_ADDR_16_              ( (1 << 16) | (2 << 8) | (16 << 3) | 0 )
#define PADINDEX_OF_GPIOC_GPIO_16_              ( (1 << 16) | (2 << 8) | (16 << 3) | 1 )
#define PADINDEX_OF_MPEGTSI_TSYNC0              ( (1 << 16) | (2 << 8) | (16 << 3) | 2 )
#define PADINDEX_OF_VIP1_i_ExtVSYNC2                ( (1 << 16) | (2 << 8) | (16 << 3) | 3 )
#define PADINDEX_OF_MCUSTOP_o_ADDR_17_              ( (1 << 16) | (2 << 8) | (17 << 3) | 0 )
#define PADINDEX_OF_GPIOC_GPIO_17_              ( (1 << 16) | (2 << 8) | (17 << 3) | 1 )
#define PADINDEX_OF_MPEGTSI_TDP0                ( (1 << 16) | (2 << 8) | (17 << 3) | 2 )
#define PADINDEX_OF_VIP1_i_VD2_0_               ( (1 << 16) | (2 << 8) | (17 << 3) | 3 )
#define PADINDEX_OF_MCUSTOP_o_ADDR_18_              ( (1 << 16) | (2 << 8) | (18 << 3) | 0 )
#define PADINDEX_OF_GPIOC_GPIO_18_              ( (1 << 16) | (2 << 8) | (18 << 3) | 1 )
#define PADINDEX_OF_SDMMC2_SDMMC_CCLK           ( (1 << 16) | (2 << 8) | (18 << 3) | 2 )
#define PADINDEX_OF_VIP1_i_VD2_1_               ( (1 << 16) | (2 << 8) | (18 << 3) | 3 )
#define PADINDEX_OF_MCUSTOP_o_ADDR_19_          ( (1 << 16) | (2 << 8) | (19 << 3) | 0 )
#define PADINDEX_OF_GPIOC_GPIO_19_              ( (1 << 16) | (2 << 8) | (19 << 3) | 1 )
#define PADINDEX_OF_SDMMC2_SDMMC_CMD            ( (1 << 16) | (2 << 8) | (19 << 3) | 2 )
#define PADINDEX_OF_VIP1_i_VD2_2_               ( (1 << 16) | (2 << 8) | (19 << 3) | 3 )
#define PADINDEX_OF_MCUSTOP_o_ADDR_20_          ( (1 << 16) | (2 << 8) | (20 << 3) | 0 )
#define PADINDEX_OF_GPIOC_GPIO_20_              ( (1 << 16) | (2 << 8) | (20 << 3) | 1 )
#define PADINDEX_OF_SDMMC2_SDMMC_CDATA_0_       ( (1 << 16) | (2 << 8) | (20 << 3) | 2 )
#define PADINDEX_OF_VIP1_i_VD2_3_               ( (1 << 16) | (2 << 8) | (20 << 3) | 3 )
#define PADINDEX_OF_MCUSTOP_o_ADDR_21_          ( (1 << 16) | (2 << 8) | (21 << 3) | 0 )
#define PADINDEX_OF_GPIOC_GPIO_21_              ( (1 << 16) | (2 << 8) | (21 << 3) | 1 )
#define PADINDEX_OF_SDMMC2_SDMMC_CDATA_1_       ( (1 << 16) | (2 << 8) | (21 << 3) | 2 )
#define PADINDEX_OF_VIP1_i_VD2_4_               ( (1 << 16) | (2 << 8) | (21 << 3) | 3 )
#define PADINDEX_OF_MCUSTOP_o_ADDR_22_          ( (1 << 16) | (2 << 8) | (22 << 3) | 0 )
#define PADINDEX_OF_GPIOC_GPIO_22_              ( (1 << 16) | (2 << 8) | (22 << 3) | 1 )
#define PADINDEX_OF_SDMMC2_SDMMC_CDATA_2_       ( (1 << 16) | (2 << 8) | (22 << 3) | 2 )
#define PADINDEX_OF_VIP1_i_VD2_5_               ( (1 << 16) | (2 << 8) | (22 << 3) | 3 )
#define PADINDEX_OF_MCUSTOP_o_ADDR_23_          ( (1 << 16) | (2 << 8) | (23 << 3) | 0 )
#define PADINDEX_OF_GPIOC_GPIO_23_              ( (1 << 16) | (2 << 8) | (23 << 3) | 1 )
#define PADINDEX_OF_SDMMC2_SDMMC_CDATA_3_       ( (1 << 16) | (2 << 8) | (23 << 3) | 2 )
#define PADINDEX_OF_VIP1_i_VD2_6_               ( (1 << 16) | (2 << 8) | (23 << 3) | 3 )
#define PADINDEX_OF_MCUSTOP_LATADDR             ( (1 << 16) | (2 << 8) | (24 << 3) | 0 )
#define PADINDEX_OF_GPIOC_GPIO_24_              ( (1 << 16) | (2 << 8) | (24 << 3) | 1 )
#define PADINDEX_OF_SPDIFRX_SPDIFIN             ( (1 << 16) | (2 << 8) | (24 << 3) | 2 )
#define PADINDEX_OF_VIP1_i_VD2_7_               ( (1 << 16) | (2 << 8) | (24 << 3) | 3 )
#define PADINDEX_OF_GPIOD_GPIO_28_              ( (1 << 16) | (3 << 8) | (28 << 3) | 0 )
#define PADINDEX_OF_VIP0_i_VD_0_                ( (1 << 16) | (3 << 8) | (28 << 3) | 1 )
#define PADINDEX_OF_MPEGTSI_TDATA1_0_               ( (1 << 16) | (3 << 8) | (28 << 3) | 2 )
#define PADINDEX_OF_MCUSTOP_o_ADDR_24_              ( (1 << 16) | (3 << 8) | (28 << 3) | 3 )
#define PADINDEX_OF_GPIOD_GPIO_29_              ( (1 << 16) | (3 << 8) | (29 << 3) | 0 )
#define PADINDEX_OF_VIP0_i_VD_1_                ( (1 << 16) | (3 << 8) | (29 << 3) | 1 )
#define PADINDEX_OF_MPEGTSI_TDATA1_1_               ( (1 << 16) | (3 << 8) | (29 << 3) | 2 )
#define PADINDEX_OF_GPIOD_GPIO_30_              ( (1 << 16) | (3 << 8) | (30 << 3) | 0 )
#define PADINDEX_OF_VIP0_i_VD_2_                ( (1 << 16) | (3 << 8) | (30 << 3) | 1 )
#define PADINDEX_OF_MPEGTSI_TDATA1_2_               ( (1 << 16) | (3 << 8) | (30 << 3) | 2 )
#define PADINDEX_OF_GPIOD_GPIO_31_              ( (1 << 16) | (3 << 8) | (31 << 3) | 0 )
#define PADINDEX_OF_VIP0_i_VD_3_                ( (1 << 16) | (3 << 8) | (31 << 3) | 1 )
#define PADINDEX_OF_MPEGTSI_TDATA1_3_               ( (1 << 16) | (3 << 8) | (31 << 3) | 2 )
#define PADINDEX_OF_GPIOE_GPIO_0_               ( (1 << 16) | (4 << 8) | (0 << 3) | 0 )
#define PADINDEX_OF_VIP0_i_VD_4_                ( (1 << 16) | (4 << 8) | (0 << 3) | 1 )
#define PADINDEX_OF_MPEGTSI_TDATA1_4_               ( (1 << 16) | (4 << 8) | (0 << 3) | 2 )
#define PADINDEX_OF_GPIOE_GPIO_1_               ( (1 << 16) | (4 << 8) | (1 << 3) | 0 )
#define PADINDEX_OF_VIP0_i_VD_5_                ( (1 << 16) | (4 << 8) | (1 << 3) | 1 )
#define PADINDEX_OF_MPEGTSI_TDATA1_5_               ( (1 << 16) | (4 << 8) | (1 << 3) | 2 )
#define PADINDEX_OF_GPIOE_GPIO_2_               ( (1 << 16) | (4 << 8) | (2 << 3) | 0 )
#define PADINDEX_OF_VIP0_i_VD_6_                ( (1 << 16) | (4 << 8) | (2 << 3) | 1 )
#define PADINDEX_OF_MPEGTSI_TDATA1_6_               ( (1 << 16) | (4 << 8) | (2 << 3) | 2 )
#define PADINDEX_OF_GPIOE_GPIO_3_               ( (1 << 16) | (4 << 8) | (3 << 3) | 0 )
#define PADINDEX_OF_VIP0_i_VD_7_                ( (1 << 16) | (4 << 8) | (3 << 3) | 1 )
#define PADINDEX_OF_MPEGTSI_TDATA1_7_               ( (1 << 16) | (4 << 8) | (3 << 3) | 2 )
#define PADINDEX_OF_GPIOE_GPIO_4_               ( (1 << 16) | (4 << 8) | (4 << 3) | 0 )
#define PADINDEX_OF_VIP0_i_ExtCLK               ( (1 << 16) | (4 << 8) | (4 << 3) | 1 )
#define PADINDEX_OF_MPEGTSI_TSCLK1              ( (1 << 16) | (4 << 8) | (4 << 3) | 2 )
#define PADINDEX_OF_GPIOE_GPIO_5_               ( (1 << 16) | (4 << 8) | (5 << 3) | 0 )
#define PADINDEX_OF_VIP0_i_ExtHSYNC             ( (1 << 16) | (4 << 8) | (5 << 3) | 1 )
#define PADINDEX_OF_MPEGTSI_TSYNC1              ( (1 << 16) | (4 << 8) | (5 << 3) | 2 )
#define PADINDEX_OF_GPIOE_GPIO_6_               ( (1 << 16) | (4 << 8) | (6 << 3) | 0 )
#define PADINDEX_OF_VIP0_i_ExtVSYNC             ( (1 << 16) | (4 << 8) | (6 << 3) | 1 )
#define PADINDEX_OF_MPEGTSI_TDP1                ( (1 << 16) | (4 << 8) | (6 << 3) | 2 )
#define PADINDEX_OF_ECID_PAD_EFUSE_FSOURCE              ( (0 << 16) | 0 )
#define PADINDEX_OF_nx01301_CORTEXA9MP_TOP_QUADL2C_nTRST                ( (1 << 16) | (4 << 8) | (25 << 3) | 0 )
#define PADINDEX_OF_GPIOE_GPIO_25_              ( (1 << 16) | (4 << 8) | (25 << 3) | 1 )
#define PADINDEX_OF_nx01301_CORTEXA9MP_TOP_QUADL2C_TMS              ( (1 << 16) | (4 << 8) | (26 << 3) | 0 )
#define PADINDEX_OF_GPIOE_GPIO_26_              ( (1 << 16) | (4 << 8) | (26 << 3) | 1 )
#define PADINDEX_OF_nx01301_CORTEXA9MP_TOP_QUADL2C_TDI              ( (1 << 16) | (4 << 8) | (27 << 3) | 0 )
#define PADINDEX_OF_GPIOE_GPIO_27_              ( (1 << 16) | (4 << 8) | (27 << 3) | 1 )
#define PADINDEX_OF_nx01301_CORTEXA9MP_TOP_QUADL2C_TCLK             ( (1 << 16) | (4 << 8) | (28 << 3) | 0 )
#define PADINDEX_OF_GPIOE_GPIO_28_              ( (1 << 16) | (4 << 8) | (28 << 3) | 1 )
#define PADINDEX_OF_nx01301_CORTEXA9MP_TOP_QUADL2C_TDO              ( (1 << 16) | (4 << 8) | (29 << 3) | 0 )
#define PADINDEX_OF_GPIOE_GPIO_29_              ( (1 << 16) | (4 << 8) | (29 << 3) | 1 )
#define PADINDEX_OF_ECID_PAD_BONDING_ID_0_              ( (0 << 16) | 0 )
#define PADINDEX_OF_ECID_PAD_BONDING_ID_1_              ( (0 << 16) | 0 )
#define PADINDEX_OF_CLKPWR_nGRESETOUT               ( (0 << 16) | 0 )
#define PADINDEX_OF_CLKPWR_AliveGPIO_0_             ( (0 << 16) | 0 )
#define PADINDEX_OF_CLKPWR_AliveGPIO_1_             ( (0 << 16) | 0 )
#define PADINDEX_OF_CLKPWR_AliveGPIO_2_             ( (0 << 16) | 0 )
#define PADINDEX_OF_CLKPWR_AliveGPIO_3_             ( (0 << 16) | 0 )
#define PADINDEX_OF_CLKPWR_AliveGPIO_4_             ( (0 << 16) | 0 )
#define PADINDEX_OF_CLKPWR_AliveGPIO_5_             ( (0 << 16) | 0 )
#define PADINDEX_OF_CLKPWR_nRESET               ( (0 << 16) | 0 )
#define PADINDEX_OF_CLKPWR_TEST_EN              ( (0 << 16) | 0 )
#define PADINDEX_OF_CLKPWR_nBATF                ( (0 << 16) | 0 )
#define PADINDEX_OF_CLKPWR_VDDPWRON             ( (0 << 16) | 0 )
#define PADINDEX_OF_CLKPWR_VDDPWRON_DDR             ( (0 << 16) | 0 )
#define PADINDEX_OF_CLKPWR_nVDDPWRTOGGLE                ( (0 << 16) | 0 )
#define PADINDEX_OF_CLKPWR_XTIRTC               ( (0 << 16) | 0 )
#define PADINDEX_OF_ADC_AIN_0_              ( (0 << 16) | 0 )
#define PADINDEX_OF_ADC_AIN_1_              ( (0 << 16) | 0 )
#define PADINDEX_OF_ADC_AIN_2_              ( (0 << 16) | 0 )
#define PADINDEX_OF_ADC_AIN_3_              ( (0 << 16) | 0 )
#define PADINDEX_OF_ADC_AIN_4_              ( (0 << 16) | 0 )
#define PADINDEX_OF_ADC_AIN_5_              ( (0 << 16) | 0 )
#define PADINDEX_OF_ADC_AIN_6_              ( (0 << 16) | 0 )
#define PADINDEX_OF_ADC_AIN_7_              ( (0 << 16) | 0 )
#define PADINDEX_OF_ADC_VREF                ( (0 << 16) | 0 )
#define PADINDEX_OF_ADC_AGND                ( (0 << 16) | 0 )
#define PADINDEX_OF_DREX_io_ZQ              ( (0 << 16) | 0 )
#define PADINDEX_OF_DREX_io_DQ_31_              ( (0 << 16) | 0 )
#define PADINDEX_OF_DREX_io_DQ_30_              ( (0 << 16) | 0 )
#define PADINDEX_OF_DREX_io_DQ_29_              ( (0 << 16) | 0 )
#define PADINDEX_OF_DREX_io_DQ_28_              ( (0 << 16) | 0 )
#define PADINDEX_OF_DREX_io_DQ_27_              ( (0 << 16) | 0 )
#define PADINDEX_OF_DREX_io_DQ_26_              ( (0 << 16) | 0 )
#define PADINDEX_OF_DREX_io_DQ_25_              ( (0 << 16) | 0 )
#define PADINDEX_OF_DREX_io_DQ_24_              ( (0 << 16) | 0 )
#define PADINDEX_OF_DREX_io_PDQS_3_             ( (0 << 16) | 0 )
#define PADINDEX_OF_DREX_io_NDQS_3_             ( (0 << 16) | 0 )
#define PADINDEX_OF_DREX_o_DM_3_                ( (0 << 16) | 0 )
#define PADINDEX_OF_DREX_io_VREF1               ( (0 << 16) | 0 )
#define PADINDEX_OF_DREX_io_DQ_15_              ( (0 << 16) | 0 )
#define PADINDEX_OF_DREX_io_DQ_14_              ( (0 << 16) | 0 )
#define PADINDEX_OF_DREX_io_DQ_13_              ( (0 << 16) | 0 )
#define PADINDEX_OF_DREX_io_DQ_12_              ( (0 << 16) | 0 )
#define PADINDEX_OF_DREX_io_DQ_11_              ( (0 << 16) | 0 )
#define PADINDEX_OF_DREX_io_DQ_10_              ( (0 << 16) | 0 )
#define PADINDEX_OF_DREX_io_DQ_9_               ( (0 << 16) | 0 )
#define PADINDEX_OF_DREX_io_DQ_8_               ( (0 << 16) | 0 )
#define PADINDEX_OF_DREX_io_PDQS_1_             ( (0 << 16) | 0 )
#define PADINDEX_OF_DREX_io_NDQS_1_             ( (0 << 16) | 0 )
#define PADINDEX_OF_DREX_o_DM_1_                ( (0 << 16) | 0 )
#define PADINDEX_OF_DREX_o_WE               ( (0 << 16) | 0 )
#define PADINDEX_OF_DREX_o_CAS              ( (0 << 16) | 0 )
#define PADINDEX_OF_DREX_o_RAS              ( (0 << 16) | 0 )
#define PADINDEX_OF_DREX_o_ODT_0_               ( (0 << 16) | 0 )
#define PADINDEX_OF_DREX_o_ODT_1_               ( (0 << 16) | 0 )
#define PADINDEX_OF_DREX_o_ADDR_14_             ( (0 << 16) | 0 )
#define PADINDEX_OF_DREX_o_ADDR_15_             ( (0 << 16) | 0 )
#define PADINDEX_OF_DREX_o_ADDR_0_              ( (0 << 16) | 0 )
#define PADINDEX_OF_DREX_o_ADDR_1_              ( (0 << 16) | 0 )
#define PADINDEX_OF_DREX_o_ADDR_2_              ( (0 << 16) | 0 )
#define PADINDEX_OF_DREX_o_ADDR_3_              ( (0 << 16) | 0 )
#define PADINDEX_OF_DREX_o_ADDR_4_              ( (0 << 16) | 0 )
#define PADINDEX_OF_DREX_o_CKE_0_               ( (0 << 16) | 0 )
#define PADINDEX_OF_DREX_o_RESET                ( (0 << 16) | 0 )
#define PADINDEX_OF_DREX_o_CKE_1_               ( (0 << 16) | 0 )
#define PADINDEX_OF_DREX_o_CK               ( (0 << 16) | 0 )
#define PADINDEX_OF_DREX_o_CKB              ( (0 << 16) | 0 )
#define PADINDEX_OF_DREX_o_CS_0_                ( (0 << 16) | 0 )
#define PADINDEX_OF_DREX_o_CS_1_                ( (0 << 16) | 0 )
#define PADINDEX_OF_DREX_o_ADDR_5_              ( (0 << 16) | 0 )
#define PADINDEX_OF_DREX_o_ADDR_6_              ( (0 << 16) | 0 )
#define PADINDEX_OF_DREX_o_ADDR_7_              ( (0 << 16) | 0 )
#define PADINDEX_OF_DREX_o_ADDR_8_              ( (0 << 16) | 0 )
#define PADINDEX_OF_DREX_o_ADDR_9_              ( (0 << 16) | 0 )
#define PADINDEX_OF_DREX_o_ADDR_10_             ( (0 << 16) | 0 )
#define PADINDEX_OF_DREX_o_ADDR_11_             ( (0 << 16) | 0 )
#define PADINDEX_OF_DREX_o_ADDR_12_             ( (0 << 16) | 0 )
#define PADINDEX_OF_DREX_o_ADDR_13_             ( (0 << 16) | 0 )
#define PADINDEX_OF_DREX_o_BADDR_0_             ( (0 << 16) | 0 )
#define PADINDEX_OF_DREX_o_BADDR_1_             ( (0 << 16) | 0 )
#define PADINDEX_OF_DREX_o_BADDR_2_             ( (0 << 16) | 0 )
#define PADINDEX_OF_DREX_o_DM_0_                ( (0 << 16) | 0 )
#define PADINDEX_OF_DREX_io_NDQS_0_             ( (0 << 16) | 0 )
#define PADINDEX_OF_DREX_io_PDQS_0_             ( (0 << 16) | 0 )
#define PADINDEX_OF_DREX_io_DQ_7_               ( (0 << 16) | 0 )
#define PADINDEX_OF_DREX_io_DQ_6_               ( (0 << 16) | 0 )
#define PADINDEX_OF_DREX_io_DQ_5_               ( (0 << 16) | 0 )
#define PADINDEX_OF_DREX_io_DQ_4_               ( (0 << 16) | 0 )
#define PADINDEX_OF_DREX_io_DQ_3_               ( (0 << 16) | 0 )
#define PADINDEX_OF_DREX_io_DQ_2_               ( (0 << 16) | 0 )
#define PADINDEX_OF_DREX_io_DQ_1_               ( (0 << 16) | 0 )
#define PADINDEX_OF_DREX_io_DQ_0_               ( (0 << 16) | 0 )
#define PADINDEX_OF_DREX_io_VREF3               ( (0 << 16) | 0 )
#define PADINDEX_OF_DREX_o_DM_2_                ( (0 << 16) | 0 )
#define PADINDEX_OF_DREX_io_NDQS_2_             ( (0 << 16) | 0 )
#define PADINDEX_OF_DREX_io_PDQS_2_             ( (0 << 16) | 0 )
#define PADINDEX_OF_DREX_io_DQ_23_              ( (0 << 16) | 0 )
#define PADINDEX_OF_DREX_io_DQ_22_              ( (0 << 16) | 0 )
#define PADINDEX_OF_DREX_io_DQ_21_              ( (0 << 16) | 0 )
#define PADINDEX_OF_DREX_io_DQ_20_              ( (0 << 16) | 0 )
#define PADINDEX_OF_DREX_io_DQ_19_              ( (0 << 16) | 0 )
#define PADINDEX_OF_DREX_io_DQ_18_              ( (0 << 16) | 0 )
#define PADINDEX_OF_DREX_io_DQ_17_              ( (0 << 16) | 0 )
#define PADINDEX_OF_DREX_io_DQ_16_              ( (0 << 16) | 0 )
#define PADINDEX_OF_MIPI_io_PHY_S_DPDATA3               ( (0 << 16) | 0 )
#define PADINDEX_OF_MIPI_io_PHY_S_DNDATA3               ( (0 << 16) | 0 )
#define PADINDEX_OF_MIPI_io_PHY_S_DPDATA2               ( (0 << 16) | 0 )
#define PADINDEX_OF_MIPI_io_PHY_S_DNDATA2               ( (0 << 16) | 0 )
#define PADINDEX_OF_MIPI_io_PHY_S_DPCLK             ( (0 << 16) | 0 )
#define PADINDEX_OF_MIPI_io_PHY_S_DNCLK             ( (0 << 16) | 0 )
#define PADINDEX_OF_MIPI_io_PHY_S_DPDATA1               ( (0 << 16) | 0 )
#define PADINDEX_OF_MIPI_io_PHY_S_DNDATA1               ( (0 << 16) | 0 )
#define PADINDEX_OF_MIPI_io_PHY_S_DPDATA0               ( (0 << 16) | 0 )
#define PADINDEX_OF_MIPI_io_PHY_S_DNDATA0               ( (0 << 16) | 0 )
#define PADINDEX_OF_MIPI_io_PHY_M_DPDATA3               ( (0 << 16) | 0 )
#define PADINDEX_OF_MIPI_io_PHY_M_DNDATA3               ( (0 << 16) | 0 )
#define PADINDEX_OF_MIPI_io_PHY_M_DPDATA2               ( (0 << 16) | 0 )
#define PADINDEX_OF_MIPI_io_PHY_M_DNDATA2               ( (0 << 16) | 0 )
#define PADINDEX_OF_MIPI_io_PHY_M_DPCLK             ( (0 << 16) | 0 )
#define PADINDEX_OF_MIPI_io_PHY_M_DNCLK             ( (0 << 16) | 0 )
#define PADINDEX_OF_MIPI_io_PHY_M_VREG_0P4V             ( (0 << 16) | 0 )
#define PADINDEX_OF_MIPI_io_PHY_M_DPDATA1               ( (0 << 16) | 0 )
#define PADINDEX_OF_MIPI_io_PHY_M_DNDATA1               ( (0 << 16) | 0 )
#define PADINDEX_OF_MIPI_io_PHY_M_DPDATA0               ( (0 << 16) | 0 )
#define PADINDEX_OF_MIPI_io_PHY_M_DNDATA0               ( (0 << 16) | 0 )
#define PADINDEX_OF_DISPLAYTOP_LVDS_ROUT                ( (0 << 16) | 0 )
#define PADINDEX_OF_DISPLAYTOP_LVDS_TXN_A               ( (0 << 16) | 0 )
#define PADINDEX_OF_DISPLAYTOP_LVDS_TXP_A               ( (0 << 16) | 0 )
#define PADINDEX_OF_DISPLAYTOP_LVDS_TXN_B               ( (0 << 16) | 0 )
#define PADINDEX_OF_DISPLAYTOP_LVDS_TXP_B               ( (0 << 16) | 0 )
#define PADINDEX_OF_DISPLAYTOP_LVDS_TXN_C               ( (0 << 16) | 0 )
#define PADINDEX_OF_DISPLAYTOP_LVDS_TXP_C               ( (0 << 16) | 0 )
#define PADINDEX_OF_DISPLAYTOP_LVDS_TXN_CLK             ( (0 << 16) | 0 )
#define PADINDEX_OF_DISPLAYTOP_LVDS_TXP_CLK             ( (0 << 16) | 0 )
#define PADINDEX_OF_DISPLAYTOP_LVDS_TXN_D               ( (0 << 16) | 0 )
#define PADINDEX_OF_DISPLAYTOP_LVDS_TXP_D               ( (0 << 16) | 0 )
#define PADINDEX_OF_DISPLAYTOP_LVDS_TXN_E               ( (0 << 16) | 0 )
#define PADINDEX_OF_DISPLAYTOP_LVDS_TXP_E               ( (0 << 16) | 0 )
#define PADINDEX_OF_CLKPWR_XTI              ( (0 << 16) | 0 )
#define PADINDEX_OF_CLKPWR_XTI_not_used             ( (0 << 16) | 0 )
#define PADINDEX_OF_GPIOE_GPIO_7_               ( (1 << 16) | (4 << 8) | (7 << 3) | 0 )
#define PADINDEX_OF_DWC_GMAC_PHY_TXD_0_             ( (1 << 16) | (4 << 8) | (7 << 3) | 1 )
#define PADINDEX_OF_VIP1_i_ExtVSYNC             ( (1 << 16) | (4 << 8) | (7 << 3) | 2 )
#define PADINDEX_OF_GPIOE_GPIO_8_               ( (1 << 16) | (4 << 8) | (8 << 3) | 0 )
#define PADINDEX_OF_DWC_GMAC_PHY_TXD_1_             ( (1 << 16) | (4 << 8) | (8 << 3) | 1 )
#define PADINDEX_OF_GPIOE_GPIO_9_               ( (1 << 16) | (4 << 8) | (9 << 3) | 0 )
#define PADINDEX_OF_DWC_GMAC_PHY_TXD_2_             ( (1 << 16) | (4 << 8) | (9 << 3) | 1 )
#define PADINDEX_OF_GPIOE_GPIO_10_              ( (1 << 16) | (4 << 8) | (10 << 3) | 0 )
#define PADINDEX_OF_DWC_GMAC_PHY_TXD_3_             ( (1 << 16) | (4 << 8) | (10 << 3) | 1 )
#define PADINDEX_OF_GPIOE_GPIO_11_              ( (1 << 16) | (4 << 8) | (11 << 3) | 0 )
#define PADINDEX_OF_DWC_GMAC_PHY_TXEN               ( (1 << 16) | (4 << 8) | (11 << 3) | 1 )
#define PADINDEX_OF_GPIOE_GPIO_12_              ( (1 << 16) | (4 << 8) | (12 << 3) | 0 )
#define PADINDEX_OF_DWC_GMAC_PHY_TXER               ( (1 << 16) | (4 << 8) | (12 << 3) | 1 )
#define PADINDEX_OF_GPIOE_GPIO_13_              ( (1 << 16) | (4 << 8) | (13 << 3) | 0 )
#define PADINDEX_OF_DWC_GMAC_PHY_COL                ( (1 << 16) | (4 << 8) | (13 << 3) | 1 )
#define PADINDEX_OF_VIP1_i_ExtHSYNC             ( (1 << 16) | (4 << 8) | (13 << 3) | 2 )
#define PADINDEX_OF_GPIOE_GPIO_14_              ( (1 << 16) | (4 << 8) | (14 << 3) | 0 )
#define PADINDEX_OF_DWC_GMAC_PHY_RXD_0_             ( (1 << 16) | (4 << 8) | (14 << 3) | 1 )
#define PADINDEX_OF_SSP1_SSPCLK_IO              ( (1 << 16) | (4 << 8) | (14 << 3) | 2 )
#define PADINDEX_OF_GPIOE_GPIO_15_              ( (1 << 16) | (4 << 8) | (15 << 3) | 0 )
#define PADINDEX_OF_DWC_GMAC_PHY_RXD_1_             ( (1 << 16) | (4 << 8) | (15 << 3) | 1 )
#define PADINDEX_OF_SSP1_SSPFSS             ( (1 << 16) | (4 << 8) | (15 << 3) | 2 )
#define PADINDEX_OF_GPIOE_GPIO_16_              ( (1 << 16) | (4 << 8) | (16 << 3) | 0 )
#define PADINDEX_OF_DWC_GMAC_PHY_RXD_2_             ( (1 << 16) | (4 << 8) | (16 << 3) | 1 )
#define PADINDEX_OF_GPIOE_GPIO_17_              ( (1 << 16) | (4 << 8) | (17 << 3) | 0 )
#define PADINDEX_OF_DWC_GMAC_PHY_RXD_3_             ( (1 << 16) | (4 << 8) | (17 << 3) | 1 )
#define PADINDEX_OF_GPIOE_GPIO_18_              ( (1 << 16) | (4 << 8) | (18 << 3) | 0 )
#define PADINDEX_OF_DWC_GMAC_CLK_RX             ( (1 << 16) | (4 << 8) | (18 << 3) | 1 )
#define PADINDEX_OF_SSP1_SSPRXD             ( (1 << 16) | (4 << 8) | (18 << 3) | 2 )
#define PADINDEX_OF_GPIOE_GPIO_19_              ( (1 << 16) | (4 << 8) | (19 << 3) | 0 )
#define PADINDEX_OF_DWC_GMAC_PHY_RXDV               ( (1 << 16) | (4 << 8) | (19 << 3) | 1 )
#define PADINDEX_OF_SSP1_SSPTXD             ( (1 << 16) | (4 << 8) | (19 << 3) | 2 )
#define PADINDEX_OF_GPIOE_GPIO_20_              ( (1 << 16) | (4 << 8) | (20 << 3) | 0 )
#define PADINDEX_OF_DWC_GMAC_GMII_MDC               ( (1 << 16) | (4 << 8) | (20 << 3) | 1 )
#define PADINDEX_OF_GPIOE_GPIO_21_              ( (1 << 16) | (4 << 8) | (21 << 3) | 0 )
#define PADINDEX_OF_DWC_GMAC_GMII_MDI           ( (1 << 16) | (4 << 8) | (21 << 3) | 1 )
#define PADINDEX_OF_SDMMC2_SDMMC_CDATA_4_       ( (1 << 16) | (4 << 8) | (21 << 3) | 2 )
#define PADINDEX_OF_GPIOE_GPIO_22_              ( (1 << 16) | (4 << 8) | (22 << 3) | 0 )
#define PADINDEX_OF_DWC_GMAC_PHY_RXER           ( (1 << 16) | (4 << 8) | (22 << 3) | 1 )
#define PADINDEX_OF_SDMMC2_SDMMC_CDATA_5_       ( (1 << 16) | (4 << 8) | (22 << 3) | 2 )
#define PADINDEX_OF_GPIOE_GPIO_23_              ( (1 << 16) | (4 << 8) | (23 << 3) | 0 )
#define PADINDEX_OF_DWC_GMAC_PHY_CRS            ( (1 << 16) | (4 << 8) | (23 << 3) | 1 )
#define PADINDEX_OF_SDMMC2_SDMMC_CDATA_6_       ( (1 << 16) | (4 << 8) | (23 << 3) | 2 )
#define PADINDEX_OF_GPIOE_GPIO_24_              ( (1 << 16) | (4 << 8) | (24 << 3) | 0 )
#define PADINDEX_OF_DWC_GMAC_GTX_CLK            ( (1 << 16) | (4 << 8) | (24 << 3) | 1 )
#define PADINDEX_OF_SDMMC2_SDMMC_CDATA_7_       ( (1 << 16) | (4 << 8) | (24 << 3) | 2 )
#define PADINDEX_OF_DISPLAYTOP_io_HDMI_REXT             ( (0 << 16) | 0 )
#define PADINDEX_OF_DISPLAYTOP_o_HDMI_TX0P              ( (0 << 16) | 0 )
#define PADINDEX_OF_DISPLAYTOP_o_HDMI_TX0N              ( (0 << 16) | 0 )
#define PADINDEX_OF_DISPLAYTOP_o_HDMI_TX1P              ( (0 << 16) | 0 )
#define PADINDEX_OF_DISPLAYTOP_o_HDMI_TX1N              ( (0 << 16) | 0 )
#define PADINDEX_OF_DISPLAYTOP_o_HDMI_TX2P              ( (0 << 16) | 0 )
#define PADINDEX_OF_DISPLAYTOP_o_HDMI_TX2N              ( (0 << 16) | 0 )
#define PADINDEX_OF_DISPLAYTOP_o_HDMI_TXCP              ( (0 << 16) | 0 )
#define PADINDEX_OF_DISPLAYTOP_o_HDMI_TXCN              ( (0 << 16) | 0 )



#ifdef __cplusplus
}
#endif

#endif // __NX_CHIP_P5430_H__

    ///@brief    ALIVE Interrupts for interrupt interface
    enum
    {
        NX_ALIVE_INT_ALIVEGPIO0 = 0,    ///<    ALIVE GPIO 0 Interrupt
        NX_ALIVE_INT_ALIVEGPIO1 = 1,    ///<    ALIVE GPIO 1 Interrupt
        NX_ALIVE_INT_ALIVEGPIO2 = 2,    ///<    ALIVE GPIO 2 Interrupt
        NX_ALIVE_INT_ALIVEGPIO3 = 3,    ///<    ALIVE GPIO 3 Interrupt
        NX_ALIVE_INT_ALIVEGPIO4 = 4,    ///<    ALIVE GPIO 4 Interrupt
        NX_ALIVE_INT_ALIVEGPIO5 = 5,    ///<    ALIVE GPIO 5 Interrupt
        NX_ALIVE_INT_ALIVEGPIO6 = 6,    ///<    ALIVE GPIO 6 Interrupt
        NX_ALIVE_INT_ALIVEGPIO7 = 7     ///<    ALIVE GPIO 7 Interrupt
    };

    ///@brief    ALIVE GPIO Detect Mode
    typedef enum
    {
        NX_ALIVE_DETECTMODE_ASYNC_LOWLEVEL      = 0,    ///< Async Low Level Detect Mode
        NX_ALIVE_DETECTMODE_ASYNC_HIGHLEVEL     = 1,    ///< Async High Level Detect Mode
        NX_ALIVE_DETECTMODE_SYNC_FALLINGEDGE    = 2,    ///< Sync Falling Edge Detect Mode
        NX_ALIVE_DETECTMODE_SYNC_RISINGEDGE     = 3,    ///< Sync Rising Edge Detect Mode
        NX_ALIVE_DETECTMODE_SYNC_LOWLEVEL       = 4,    ///< Sync Low Level Detect Mode
        NX_ALIVE_DETECTMODE_SYNC_HIGHLEVEL      = 5     ///< Sync High Level Detect Mode

    } NX_ALIVE_DETECTMODE;

    /// @brief    Retention PAD Group
    typedef enum
    {
        NX_ALIVE_PADGROUP0 = 0,     ///< IO Power Group 0 ( RX0 ~ RX4 )
        NX_ALIVE_PADGROUP1 = 1,     ///< IO Power Group 1 ( USB VBUS )
        NX_ALIVE_PADGROUP2 = 2,     ///< IO Power Group 2 ( GPIO )
        NX_ALIVE_PADGROUP3 = 3,     ///< IO Power Group 2 ( GPIO )
        NX_ALIVE_PADGROUP_NUMBER = 4

    } NX_ALIVE_PADGROUP;

#define NX_ALIVE_NUMBER_OF_GPIO 6

	///@brief	GPIO Interrupts for interrupt interface
	enum
	{
		NX_GPIO_INT_GPIO0	= 0,	///<	GPIO 0 Interrupt
		NX_GPIO_INT_GPIO1	= 1,	///<	GPIO 1 Interrupt
		NX_GPIO_INT_GPIO2	= 2,	///<	GPIO 2 Interrupt
		NX_GPIO_INT_GPIO3	= 3,	///<	GPIO 3 Interrupt
		NX_GPIO_INT_GPIO4	= 4,	///<	GPIO 4 Interrupt
		NX_GPIO_INT_GPIO5	= 5,	///<	GPIO 5 Interrupt
		NX_GPIO_INT_GPIO6	= 6,	///<	GPIO 6 Interrupt
		NX_GPIO_INT_GPIO7	= 7,	///<	GPIO 7 Interrupt
		NX_GPIO_INT_GPIO8	= 8,	///<	GPIO 8 Interrupt
		NX_GPIO_INT_GPIO9	= 9,	///<	GPIO 9 Interrupt
		NX_GPIO_INT_GPIO10	= 10,	///<	GPIO 10 Interrupt
		NX_GPIO_INT_GPIO11	= 11,	///<	GPIO 11 Interrupt
		NX_GPIO_INT_GPIO12	= 12,	///<	GPIO 12 Interrupt
		NX_GPIO_INT_GPIO13	= 13,	///<	GPIO 13 Interrupt
		NX_GPIO_INT_GPIO14	= 14,	///<	GPIO 14 Interrupt
		NX_GPIO_INT_GPIO15	= 15,	///<	GPIO 15 Interrupt
		NX_GPIO_INT_GPIO16	= 16,	///<	GPIO 16 Interrupt
		NX_GPIO_INT_GPIO17	= 17,	///<	GPIO 17 Interrupt
		NX_GPIO_INT_GPIO18	= 18,	///<	GPIO 18 Interrupt
		NX_GPIO_INT_GPIO19	= 19,	///<	GPIO 19 Interrupt
		NX_GPIO_INT_GPIO20	= 20,	///<	GPIO 20 Interrupt
		NX_GPIO_INT_GPIO21	= 21,	///<	GPIO 21 Interrupt
		NX_GPIO_INT_GPIO22	= 22,	///<	GPIO 22 Interrupt
		NX_GPIO_INT_GPIO23	= 23,	///<	GPIO 23 Interrupt
		NX_GPIO_INT_GPIO24	= 24,	///<	GPIO 24 Interrupt
		NX_GPIO_INT_GPIO25	= 25,	///<	GPIO 25 Interrupt
		NX_GPIO_INT_GPIO26	= 26,	///<	GPIO 26 Interrupt
		NX_GPIO_INT_GPIO27	= 27,	///<	GPIO 27 Interrupt
		NX_GPIO_INT_GPIO28	= 28,	///<	GPIO 28 Interrupt
		NX_GPIO_INT_GPIO29	= 29,	///<	GPIO 29 Interrupt
		NX_GPIO_INT_GPIO30	= 30,	///<	GPIO 30 Interrupt
		NX_GPIO_INT_GPIO31	= 31	///<	GPIO 31 Interrupt
	};

	/// @brief	GPIO interrupt mode
	typedef enum
	{
		NX_GPIO_INTMODE_LOWLEVEL	= 0UL,		///< Low level detect
		NX_GPIO_INTMODE_HIGHLEVEL	= 1UL,		///< High level detect
		NX_GPIO_INTMODE_FALLINGEDGE = 2UL,		///< Falling edge detect
		NX_GPIO_INTMODE_RISINGEDGE	= 3UL,		///< Rising edge detect
		NX_GPIO_INTMODE_BOTHEDGE    = 4UL  		///< both (rise and falling) edge detect

	}NX_GPIO_INTMODE;

	/// @brief	I/O mode
	typedef enum
	{
		NX_GPIO_PADFUNC_0			= 0UL,			///< Alternate function 0
		NX_GPIO_PADFUNC_1			= 1UL,			///< Alternate function 1
		NX_GPIO_PADFUNC_2			= 2UL,			///< Alternate function 2
		NX_GPIO_PADFUNC_3			= 3UL			///< Alternate function 3

	}NX_GPIO_PADFUNC ;

	typedef enum
	{
		NX_GPIO_DRVSTRENGTH_0		= 0UL,
		NX_GPIO_DRVSTRENGTH_1		= 1UL,
		NX_GPIO_DRVSTRENGTH_2		= 2UL,
		NX_GPIO_DRVSTRENGTH_3		= 3UL

	}NX_GPIO_DRVSTRENGTH;

	typedef enum
	{
		NX_GPIO_PULL_DOWN		= 0UL,
		NX_GPIO_PULL_UP			= 1UL,
		NX_GPIO_PULL_OFF		= 2UL				// Add
	}NX_GPIO_PULL;
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

#endif //__S5P6818_INTR_H__
/*
#define pr_debug	printk
*/

#define	INTC_BASE		(void __iomem *)IO_ADDRESS(PHY_BASEADDR_INTC)
#define	GIC_PHY_OFFSET	(0)

//----------------------------------------------------------------------------
static void __init __gic_init(void __iomem *dist_base, void __iomem *cpu_base);
static void __init gpio_init(void __iomem *base, unsigned int irq_start,
							u32 irq_sources, u32 resume_sources);
static void __init alive_init(void __iomem *base, unsigned int irq_start,
							u32 irq_sources, u32 resume_sources);

/*----------------------------------------------------------------------------
 *  cpu irq handler
 */
#define GIC_DIST_BASE		(void __iomem *)(INTC_BASE + 0x00001000)		// 0xC0009000
#define GIC_CPUI_BASE		(void __iomem *)(INTC_BASE + 0x00002000)		// 0xC000a000

#define GPIO_INT_BASE		(void __iomem *)IO_ADDRESS(PHY_BASEADDR_GPIOA)
#define GPIO_BASE_OFFSET	(0x1000)
#define GPIO_INT_MASK		(0xFFFFFFFF)

#define ALIVE_INT_BASE		(void __iomem *)IO_ADDRESS(PHY_BASEADDR_CLKPWR_MODULE + 0x800)
#define ALIVE_INT_MASK		(0x000000FF)

/*
 *  cpu irq handler
 */
void __init nxp_cpu_irq_init(void)
{
	pr_debug("%s:%d\n", __func__, __LINE__);

	__gic_init(GIC_DIST_BASE, (void __iomem *)GIC_CPUI_BASE);
	gpio_init(GPIO_INT_BASE , IRQ_GPIO_START, GPIO_INT_MASK, 0);	/* 64 ~ 223 (A,B,C,D,E) */
	alive_init(ALIVE_INT_BASE, IRQ_ALIVE_START, ALIVE_INT_MASK, 0); /* 224 ~ 231 */

#ifdef CONFIG_FIQ
	init_FIQ();
#endif

	/* wake up source from idle */
	irq_set_irq_wake(IRQ_PHY_CLKPWR_ALIVEIRQ + GIC_PHY_OFFSET, 1);
#if PM_RTC_WAKE
	irq_set_irq_wake(IRQ_PHY_CLKPWR_RTCIRQ + GIC_PHY_OFFSET, 1);
#endif
}

static void __init __gic_init(void __iomem *dist_base, void __iomem *cpu_base)
{
	int irq = IRQ_GIC_PPI_VIC;

	printk(KERN_INFO "GIC   @%p: start %3d (gic %d)\n",
		dist_base, IRQ_GIC_START, (irq-IRQ_GIC_START));

	gic_init(0, IRQ_GIC_PPI_START, dist_base, cpu_base);
}

/*----------------------------------------------------------------------------
 *  ALIVE irq chain handler
 *  start  -> request_irq -> alive irq_unmask
 *  do IRQ -> alive handler -> alive irq_mask -> alive irq_ack -> driver handler -> alive irq_unmask ->
 *  end    -> disable
 ----------------------------------------------------------------------------*/
#define	ALIVE_MOD_REST		(0x04)	// detect mode reset
#define	ALIVE_MOD_SET		(0x08)	// detect mode
#define	ALIVE_MOD_READ		(0x0C)	// detect mode read

#define	ALIVE_DET_RESET		(0x4C)
#define	ALIVE_DET_SET		(0x50)
#define	ALIVE_DET_READ		(0x54)

#define	ALIVE_INT_RESET		(0x58)	// interrupt reset 	: disable
#define	ALIVE_INT_SET		(0x5C)	// interrupt set	: enable
#define	ALIVE_INT_SET_READ	(0x60)	// interrupt set read
#define	ALIVE_INT_STATUS	(0x64)	// interrupt detect pending and clear

#define	ALIVE_OUT_RESET		(0x74)
#define	ALIVE_OUT_SET		(0x78)
#define	ALIVE_OUT_READ		(0x7C)

static void alive_ack_irq(struct irq_data *d)
{
	void __iomem *base = irq_data_get_irq_chip_data(d);
	int bit = (d->irq - IRQ_ALIVE_START) & 0x1F;
	pr_debug("%s: alive irq = %d, io = %d\n", __func__, d->irq, bit);

	/* alive ack : irq pend clear */
	writel((1<<bit), base + ALIVE_INT_STATUS);
	readl(base + ALIVE_INT_STATUS);
}

static void alive_mask_irq(struct irq_data *d)
{
	void __iomem *base = irq_data_get_irq_chip_data(d);
	int bit = (d->irq - IRQ_ALIVE_START) & 0x1F;
	pr_debug("%s: alive irq = %d, io = %d\n", __func__, d->irq, bit);

	/* alive mask : irq reset (disable) */
	writel((1<<bit), base + ALIVE_INT_RESET);
}

static void alive_unmask_irq(struct irq_data *d)
{
	void __iomem *base = irq_data_get_irq_chip_data(d);
	int bit = (d->irq - IRQ_ALIVE_START) & 0x1F;
	pr_debug("%s: alive irq = %d, io = %d\n", __func__, d->irq, bit);

	/* alive unmask : irq set (enable) */
	writel((1<<bit), base + ALIVE_INT_SET);
	readl(base + ALIVE_INT_SET_READ);
}

static int alive_set_type_irq(struct irq_data *d, unsigned int type)
{
	void __iomem *base = irq_data_get_irq_chip_data(d);
	u32 reg = 0;
	int bit = (d->irq - IRQ_ALIVE_START) & 0x1F;
	int offs = 0, i = 0;
	NX_ALIVE_DETECTMODE mode = 0;

	pr_debug("%s: alive irq = %d, io = %d, type=0x%x\n",
		__func__, d->irq, bit, type);

	switch (type) {
	case IRQ_TYPE_NONE:	printk(KERN_WARNING "%s: No edge setting!\n", __func__);
		break;
	case IRQ_TYPE_EDGE_FALLING:	mode = NX_ALIVE_DETECTMODE_SYNC_FALLINGEDGE; break;
	case IRQ_TYPE_EDGE_RISING:	mode = NX_ALIVE_DETECTMODE_SYNC_RISINGEDGE;	break;
	case IRQ_TYPE_EDGE_BOTH:	mode = NX_ALIVE_DETECTMODE_SYNC_FALLINGEDGE; break;	/* and Rising Edge */
	case IRQ_TYPE_LEVEL_LOW:	mode = NX_ALIVE_DETECTMODE_ASYNC_LOWLEVEL; break;
	case IRQ_TYPE_LEVEL_HIGH:	mode = NX_ALIVE_DETECTMODE_ASYNC_HIGHLEVEL; break;
	default:
		printk(KERN_ERR "%s: No such irq type %d", __func__, type);
		return -1;
	}

	for ( ; 6 > i; i++, offs += 0x0C) {
		reg = (i == mode ? ALIVE_MOD_SET : ALIVE_MOD_REST);
		writel(1<<bit, (base + reg  + offs));	/* set o reset mode */
	}

	/*
	 * set risingedge mode for both edge
	 * 0x2C : Risingedge
	 */
	if (IRQ_TYPE_EDGE_BOTH == type)
		writel(1<<bit, (base + 0x2C));

	writel(1<<bit, base + ALIVE_DET_SET);
	writel(1<<bit, base + ALIVE_INT_SET);
	writel(1<<bit, base + ALIVE_OUT_RESET);

	return 0;
}

static int alive_set_wake(struct irq_data *d, unsigned int on)
{
#if (0)
	void __iomem *base = irq_data_get_irq_chip_data(d);
	int bit = (d->irq - IRQ_ALIVE_START) & 0x1F;

	pr_info("%s: alive irq = %d, io = %d wake %s\n",
		__func__, d->irq, bit, on?"on":"off");
#endif
	return 0;
}

static void alive_irq_enable(struct irq_data *d)
{
	void __iomem *base = irq_data_get_irq_chip_data(d);
	int bit = (d->irq - IRQ_ALIVE_START) & 0x1F;
	pr_debug("%s: alive irq = %d, io = %d\n", __func__, d->irq, bit);

	/* alive unmask : irq set (enable) */
	writel((1<<bit), base + ALIVE_INT_SET);
	readl(base + ALIVE_INT_SET_READ);
}

static void alive_irq_disable(struct irq_data *d)
{
	void __iomem *base = irq_data_get_irq_chip_data(d);
	int bit = (d->irq - IRQ_ALIVE_START) & 0x1F;
	pr_debug("%s: alive irq = %d, io = %d\n", __func__, d->irq, bit);

	/* alive mask : irq reset (disable) */
	writel((1<<bit), base + ALIVE_INT_RESET);
}

static struct irq_chip alive_chip = {
	.name			= "ALIVE",
	.irq_ack		= alive_ack_irq,
	.irq_mask		= alive_mask_irq,
	.irq_unmask		= alive_unmask_irq,
	.irq_set_type	= alive_set_type_irq,
	.irq_set_wake	= alive_set_wake,
	.irq_enable		= alive_irq_enable,
	.irq_disable	= alive_irq_disable,
};

static void alive_handler(unsigned int irq, struct irq_desc *desc)
{
	void __iomem *base = irq_desc_get_handler_data(desc);
	u32 stat, mask;
	int phy, bit;

	mask = readl(base + ALIVE_INT_SET_READ);
	stat = readl(base + ALIVE_INT_STATUS) & mask;
	bit  = ffs(stat) - 1;
	phy  = irq;

	pr_debug("%s: alive irq=%d [io=%d], stat=0x%02x, mask=0x%02x\n",
		__func__, phy, bit, stat, mask);

	if (-1 == bit) {
		printk(KERN_ERR "Unknown alive irq=%d, stat=0x%08x, mask=0x%02x\r\n",
			phy, stat, mask);
		writel(-1, (base + ALIVE_INT_STATUS));	/* clear alive status all */
		writel_relaxed(phy, GIC_CPUI_BASE + GIC_CPU_EOI);
		return;
	}

	/* alive descriptor */
	irq  = IRQ_ALIVE_START + bit;
	desc = irq_desc + irq;

	if (desc && desc->action) {
		desc->action->flags |= IRQF_DISABLED;	/* disable irq reentrant */
		generic_handle_irq_desc(irq, desc);
	} else {
		printk(KERN_ERR "Error, not registered alive interrupt=%d (%d.%d), disable !!!\n",
			irq, phy, bit);
		writel(readl(base + ALIVE_INT_SET) & ~(1<<bit), base + ALIVE_INT_SET);		/* alive mask : irq disable */
		writel(readl(base + ALIVE_INT_STATUS) | (1<<bit), base + ALIVE_INT_STATUS);	/* alive ack  : irq pend clear */
		readl(base + ALIVE_INT_STATUS);	/* Guarantee */
	}

	writel_relaxed(phy, GIC_CPUI_BASE + GIC_CPU_EOI);
	return;
}

static void __init alive_init(void __iomem *base, unsigned int irq_start,
		     u32 irq_sources, u32 resume_sources)
{
	int irq_alive = IRQ_PHY_CLKPWR_ALIVEIRQ + GIC_PHY_OFFSET;
	int num = IRQ_ALIVE_END - IRQ_ALIVE_START;
	int i = 0;

	printk(KERN_INFO "ALIVE @%p: start %3d, mask 0x%08x (alive %d, num %d)\n",
		base, irq_start, irq_sources, irq_alive, num);

	/* set alive irq handler */
	for (i = 0; num > i; i++) {
		if (irq_sources & (1 << i)) {
			int irq = irq_start + i;
			irq_set_chip_data(irq, base);
			irq_set_chip_and_handler(irq, &alive_chip, handle_level_irq);
			set_irq_flags(irq, IRQF_VALID | IRQF_PROBE);
		}
	}

	/* register alive irq handler data */
	irq_set_handler_data(irq_alive, base);

	/*
	 * call alive_mask_irq
	 * chip and chip data is registerd at gic_init
	 */
	irq_set_chained_handler(irq_alive, alive_handler);
}

/*----------------------------------------------------------------------------
 *  GPIO irq chain handler
 *  start  -> request_irq -> gpio irq_unmask
 *  do IRQ -> gpio handler -> gpio irq_mask -> gpio irq_ack -> driver handler -> gpio irq_unmask ->
 *  end    -> disable
 ----------------------------------------------------------------------------*/
static const char *io_name[] = { "GPIOA", "GPIOB", "GPIOC", "GPIOD", "GPIOE", };

#define	PIO_IRQ_BASE	IRQ_PHY_GPIOA
#define	VIO_IRQ_BASE	IRQ_GPIO_START
#define	VIO_NAME(i)		(io_name[(i-VIO_IRQ_BASE)/32])
#define	PIO_NAME(i)		(io_name[(i-PIO_IRQ_BASE)])

#define	GPIO_OUT_ENB		0x04
#define	GPIO_INT_MODE0		0x08	// 0x08,0x0C
#define	GPIO_INT_MODE1		0x28
#define	GPIO_INT_ENB		0x10
#define	GPIO_INT_STATUS		0x14
#define	GPIO_ALT_MODE		0x20	// 0x20,0x24
#define	GPIO_INT_DET		0x3C

static void gpio_ack_irq(struct irq_data *d)
{
	void __iomem *base = irq_data_get_irq_chip_data(d);
	int bit = (d->irq - IRQ_GPIO_START) & 0x1F;
	pr_debug("%s: gpio irq = %d, %s.%d\n", __func__, d->irq, VIO_NAME(d->irq), bit);

	/* gpio ack : irq pend clear */
	writel((1<<bit), base + GPIO_INT_STATUS);
	readl(base + GPIO_INT_STATUS);
}

static void gpio_mask_irq(struct irq_data *d)
{
	void __iomem *base = irq_data_get_irq_chip_data(d);
	int bit = (d->irq - IRQ_GPIO_START) & 0x1F;
	pr_debug("%s: gpio irq = %d, %s.%d\n", __func__, d->irq, VIO_NAME(d->irq), bit);

	/* gpio mask : irq disable */
	writel(readl(base + GPIO_INT_ENB) & ~(1<<bit), base + GPIO_INT_ENB);
	writel(readl(base + GPIO_INT_DET) & ~(1<<bit), base + GPIO_INT_DET);
}

static void gpio_unmask_irq(struct irq_data *d)
{
	void __iomem *base = irq_data_get_irq_chip_data(d);
	int bit = (d->irq - IRQ_GPIO_START) & 0x1F;
	pr_debug("%s: gpio irq = %d, %s.%d\n", __func__, d->irq, VIO_NAME(d->irq), bit);

	/* gpio unmask : irq enable */
	writel(readl(base + GPIO_INT_ENB) | (1<<bit), base + GPIO_INT_ENB);
	writel(readl(base + GPIO_INT_DET) | (1<<bit), base + GPIO_INT_DET);
	readl(base + GPIO_INT_ENB);
}

static int gpio_set_type_irq(struct irq_data *d, unsigned int type)
{
	void __iomem *base = irq_data_get_irq_chip_data(d);
	int bit = (d->irq - IRQ_GPIO_START) & 0x1F;
	unsigned int reg, val, alt;
	NX_GPIO_INTMODE mode = 0;
	pr_debug("%s: gpio irq = %d, %s.%d, type=0x%x\n",
		__func__, d->irq, VIO_NAME(d->irq), bit, type);

	switch (type) {
	case IRQ_TYPE_NONE:	printk(KERN_WARNING "%s: No edge setting!\n", __func__);
		break;
	case IRQ_TYPE_EDGE_RISING:	mode = NX_GPIO_INTMODE_RISINGEDGE;	break;
	case IRQ_TYPE_EDGE_FALLING:	mode = NX_GPIO_INTMODE_FALLINGEDGE;	break;
	case IRQ_TYPE_EDGE_BOTH:	mode = NX_GPIO_INTMODE_BOTHEDGE;	break;
	case IRQ_TYPE_LEVEL_LOW:	mode = NX_GPIO_INTMODE_LOWLEVEL;	break;
	case IRQ_TYPE_LEVEL_HIGH:	mode = NX_GPIO_INTMODE_HIGHLEVEL;	break;
	default:
		printk(KERN_ERR "%s: No such irq type %d", __func__, type);
		return -1;
	}

	/* gpio out : output disable */
	writel(readl(base + GPIO_OUT_ENB) & ~(1<<bit), base + GPIO_OUT_ENB);

	/* gpio mode : interrupt mode */
	reg  = (unsigned int)(base + GPIO_INT_MODE0 + (bit/16) * 4);
	val  = readl(reg) & ~(3<<((bit&0xf) * 2));
	val |= (mode&0x3) << ((bit&0xf) * 2);
	pr_debug("reg=0x%08x, val=0x%08x\n", reg, val);

	writel(val, reg);

	reg  = (unsigned int)(base + GPIO_INT_MODE1);
	val  = readl(reg) & ~(1<<bit);
	val |= ((mode>>2) & 0x1) << bit;
	pr_debug("reg=0x%08x, val=0x%08x\n", reg, val);

	writel(val, reg);

	/* gpio alt : gpio mode for irq */
	reg  = (unsigned int)(base + GPIO_ALT_MODE + (bit/16) * 4);
	val  = readl(reg) & ~(3<<((bit&0xf) * 2));
	alt  = gpio_alt_no[(d->irq-VIO_IRQ_BASE)/32][bit];
	val |= alt << ((bit&0xf) * 2);
	pr_debug("reg=0x%08x, val=0x%08x\n", reg, val);
	writel(val, reg);

	return 0;
}

static int gpio_set_wake(struct irq_data *d, unsigned int on)
{
#if (0)
	void __iomem *base = irq_data_get_irq_chip_data(d);
	int bit = (d->irq - IRQ_GPIO_START) & 0x1F;
	pr_debug("%s: gpio irq = %d, %s.%d wake %s\n",
		__func__, d->irq, VIO_NAME(d->irq), bit, on?"on":"off");
#endif
	return 0;
}

static void gpio_irq_enable(struct irq_data *d)
{
	void __iomem *base = irq_data_get_irq_chip_data(d);
	int bit = (d->irq - IRQ_GPIO_START) & 0x1F;
	pr_debug("%s: gpio irq = %d, %s.%d\n", __func__, d->irq, VIO_NAME(d->irq), bit);

	/* gpio unmask : irq enable */
	writel(readl(base + GPIO_INT_ENB) | (1<<bit), base + GPIO_INT_ENB);
	writel(readl(base + GPIO_INT_DET) | (1<<bit), base + GPIO_INT_DET);
}

static void gpio_irq_disable(struct irq_data *d)
{
	void __iomem *base = irq_data_get_irq_chip_data(d);
	int bit = (d->irq - IRQ_GPIO_START) & 0x1F;
	pr_debug("%s: gpio irq = %d, %s.%d\n", __func__, d->irq, VIO_NAME(d->irq), bit);

	/* gpio mask : irq disable */
	writel(readl(base + GPIO_INT_ENB) & ~(1<<bit), base + GPIO_INT_ENB);
	writel(readl(base + GPIO_INT_DET) & ~(1<<bit), base + GPIO_INT_DET);
}

static struct irq_chip gpio_chip = {
	.name			= "GPIO",
	.irq_ack		= gpio_ack_irq,
	.irq_mask		= gpio_mask_irq,
	.irq_unmask		= gpio_unmask_irq,
	.irq_set_type	= gpio_set_type_irq,
	.irq_set_wake	= gpio_set_wake,
	.irq_enable		= gpio_irq_enable,
	.irq_disable	= gpio_irq_disable,
};

static void gpio_handler(unsigned int irq, struct irq_desc *desc)
{
	void __iomem *base = irq_desc_get_handler_data(desc);
	u32 stat, mask;
	int phy, bit;

	mask = readl(base + GPIO_INT_ENB);
	stat = readl(base + GPIO_INT_STATUS) & mask;
	bit  = ffs(stat) - 1;
	phy  = irq;

	pr_debug("%s: gpio irq=%d [%s.%d], stat=0x%08x, mask=0x%08x\n",
		__func__, phy, PIO_NAME(phy), bit, stat, mask);

	if (-1 == bit) {
		printk(KERN_ERR "Unknown gpio phy irq=%d, status=0x%08x, mask=0x%08x\r\n",
			phy, stat, mask);
		writel(-1, (base + GPIO_INT_STATUS));	/* clear gpio status all */
		writel_relaxed(phy, GIC_CPUI_BASE + GIC_CPU_EOI);
		return;
	}

	/* gpio descriptor */
	irq  = (VIO_IRQ_BASE + bit + (32 * (phy - PIO_IRQ_BASE)));	// virtual irq
	desc = irq_desc + irq;

	if (desc && desc->action) {
		/* disable irq reentrant */
		desc->action->flags |= IRQF_DISABLED;
		generic_handle_irq_desc(irq, desc);
	} else {
		printk(KERN_ERR "Error, not registered gpio interrupt=%d (%s.%d), disable !!!\n",
			irq, PIO_NAME(phy), bit);
		writel(readl(base + GPIO_INT_ENB) & ~(1<<bit), base + GPIO_INT_ENB);		/* gpio mask : irq disable */
		writel(readl(base + GPIO_INT_STATUS) | (1<<bit), base + GPIO_INT_STATUS);	/* gpio ack  : irq pend clear */
		readl(base + GPIO_INT_STATUS);	/* Guarantee */
	}

	writel_relaxed(phy, GIC_CPUI_BASE + GIC_CPU_EOI);
	return;
}

static void __init gpio_init(void __iomem *base, unsigned int irq_start,
		     u32 irq_sources, u32 resume_sources)
{
	int irq_gpio = IRQ_PHY_GPIOA + GIC_PHY_OFFSET;
	int num = 5;  /* A,B,C,D,E */
	int ios = 32; /* GPIO 32 */
	int n = 0,i = 0;

	/* set gpio irq handler */
	for (n = 0; num > n; n++) {
		printk(KERN_INFO "GPIO  @%p: start %3d, mask 0x%08x (gpio %d)\n",
			base, irq_start, irq_sources, irq_gpio);

		for (i = 0; ios > i; i++) {
			if (irq_sources & (1 << i)) {
				int irq = irq_start + i;
				irq_set_chip_data(irq, base);
				irq_set_chip_and_handler(irq, &gpio_chip, handle_level_irq);
				set_irq_flags(irq, IRQF_VALID | IRQF_PROBE);
			}
		}
		/* init gpio irq register  */
		writel(0xFFFFFFFF, base + GPIO_INT_STATUS);
		writel(0x0, base + GPIO_INT_ENB);
		writel(0x0, base + GPIO_INT_DET);

		/* register gpio irq handler data */
		irq_set_handler_data(irq_gpio, base);

		/*
	 	 * call gpio_mask_irq
	 	 * chip and chip data is registerd at gic_init
	 	 */
		irq_set_chained_handler(irq_gpio, gpio_handler);

		/* next */
		irq_gpio++;
		irq_start += ios;
		base += GPIO_BASE_OFFSET;
	}
}

void __init exynos4_init_irq(void)
{

	nxp_cpu_irq_init();


/*	int irq;
	unsigned int gic_bank_offset;

	gic_bank_offset = soc_is_exynos4412() ? 0x4000 : 0x8000;

	if (!of_have_populated_dt())
		gic_init_bases(0, IRQ_PPI(0), S5P_VA_GIC_DIST, S5P_VA_GIC_CPU, gic_bank_offset, NULL);
#ifdef CONFIG_OF
	else
		of_irq_init(exynos4_dt_irq_match);
#endif

	for (irq = 0; irq < EXYNOS4_MAX_COMBINER_NR; irq++) {

		combiner_init(irq, (void __iomem *)S5P_VA_COMBINER(irq),
				COMBINER_IRQ(irq, 0));
		combiner_cascade_irq(irq, IRQ_SPI(irq));
	}
*/
	/*
	 * The parameters of s5p_init_irq() are for VIC init.
	 * Theses parameters should be NULL and 0 because EXYNOS4
	 * uses GIC instead of VIC.
	 */
/*	s5p_init_irq(NULL, 0);*/
}

void __init exynos5_init_irq(void)
{
	int irq;

#ifdef CONFIG_OF
	of_irq_init(exynos4_dt_irq_match);
#endif

	for (irq = 0; irq < EXYNOS5_MAX_COMBINER_NR; irq++) {
		combiner_init(irq, (void __iomem *)S5P_VA_COMBINER(irq),
				COMBINER_IRQ(irq, 0));
		combiner_cascade_irq(irq, IRQ_SPI(irq));
	}

	/*
	 * The parameters of s5p_init_irq() are for VIC init.
	 * Theses parameters should be NULL and 0 because EXYNOS4
	 * uses GIC instead of VIC.
	 */
	s5p_init_irq(NULL, 0);
}

struct bus_type exynos4_subsys = {
	.name		= "exynos4-core",
	.dev_name	= "exynos4-core",
};

struct bus_type exynos5_subsys = {
	.name		= "exynos5-core",
	.dev_name	= "exynos5-core",
};

static struct device exynos4_dev = {
	.bus	= &exynos4_subsys,
};

static struct device exynos5_dev = {
	.bus	= &exynos5_subsys,
};

static int __init exynos_core_init(void)
{
	if (soc_is_exynos5250())
		return subsys_system_register(&exynos5_subsys, NULL);
	else
		return subsys_system_register(&exynos4_subsys, NULL);
}
core_initcall(exynos_core_init);

#ifdef CONFIG_CACHE_L2X0
static int __init exynos4_l2x0_cache_init(void)
{
	int ret;

	if (soc_is_exynos5250())
		return 0;

	ret = l2x0_of_init(L2_AUX_VAL, L2_AUX_MASK);
	if (!ret) {
		l2x0_regs_phys = virt_to_phys(&l2x0_saved_regs);
		clean_dcache_area(&l2x0_regs_phys, sizeof(unsigned long));
		return 0;
	}

	if (!(__raw_readl(S5P_VA_L2CC + L2X0_CTRL) & 0x1)) {
		l2x0_saved_regs.phy_base = EXYNOS4_PA_L2CC;
		/* TAG, Data Latency Control: 2 cycles */
		l2x0_saved_regs.tag_latency = 0x110;

		if (soc_is_exynos4212() || soc_is_exynos4412())
			l2x0_saved_regs.data_latency = 0x120;
		else
			l2x0_saved_regs.data_latency = 0x110;

		l2x0_saved_regs.prefetch_ctrl = 0x30000007;
		l2x0_saved_regs.pwr_ctrl =
			(L2X0_DYNAMIC_CLK_GATING_EN | L2X0_STNDBY_MODE_EN);

		l2x0_regs_phys = virt_to_phys(&l2x0_saved_regs);

		__raw_writel(l2x0_saved_regs.tag_latency,
				S5P_VA_L2CC + L2X0_TAG_LATENCY_CTRL);
		__raw_writel(l2x0_saved_regs.data_latency,
				S5P_VA_L2CC + L2X0_DATA_LATENCY_CTRL);

		/* L2X0 Prefetch Control */
		__raw_writel(l2x0_saved_regs.prefetch_ctrl,
				S5P_VA_L2CC + L2X0_PREFETCH_CTRL);

		/* L2X0 Power Control */
		__raw_writel(l2x0_saved_regs.pwr_ctrl,
				S5P_VA_L2CC + L2X0_POWER_CTRL);

		clean_dcache_area(&l2x0_regs_phys, sizeof(unsigned long));
		clean_dcache_area(&l2x0_saved_regs, sizeof(struct l2x0_regs));
	}

	l2x0_init(S5P_VA_L2CC, L2_AUX_VAL, L2_AUX_MASK);
	return 0;
}
early_initcall(exynos4_l2x0_cache_init);
#endif

static int __init exynos5_l2_cache_init(void)
{
	unsigned int val;

	if (!soc_is_exynos5250())
		return 0;

	asm volatile("mrc p15, 0, %0, c1, c0, 0\n"
		     "bic %0, %0, #(1 << 2)\n"	/* cache disable */
		     "mcr p15, 0, %0, c1, c0, 0\n"
		     "mrc p15, 1, %0, c9, c0, 2\n"
		     : "=r"(val));

	val |= (1 << 9) | (1 << 5) | (2 << 6) | (2 << 0);

	asm volatile("mcr p15, 1, %0, c9, c0, 2\n" : : "r"(val));
	asm volatile("mrc p15, 0, %0, c1, c0, 0\n"
		     "orr %0, %0, #(1 << 2)\n"	/* cache enable */
		     "mcr p15, 0, %0, c1, c0, 0\n"
		     : : "r"(val));

	return 0;
}
early_initcall(exynos5_l2_cache_init);

static int __init exynos_init(void)
{
	printk(KERN_INFO "EXYNOS: Initializing architecture\n");

	if (soc_is_exynos5250())
		return device_register(&exynos5_dev);
	else
		return device_register(&exynos4_dev);
}

/* uart registration process */

static void __init exynos_init_uarts(struct s3c2410_uartcfg *cfg, int no)
{
	struct s3c2410_uartcfg *tcfg = cfg;
	u32 ucnt;

	for (ucnt = 0; ucnt < no; ucnt++, tcfg++)
		tcfg->has_fracval = 1;

	if (soc_is_exynos5250())
		s3c24xx_init_uartdevs("exynos4210-uart", exynos5_uart_resources, cfg, no);
	else
		s3c24xx_init_uartdevs("exynos4210-uart", exynos4_uart_resources, cfg, no);
}

static void __iomem *exynos_eint_base;

static DEFINE_SPINLOCK(eint_lock);

static unsigned int eint0_15_data[16];

static inline int exynos4_irq_to_gpio(unsigned int irq)
{
	if (irq < IRQ_EINT(0))
		return -EINVAL;

	irq -= IRQ_EINT(0);
	if (irq < 8)
		return EXYNOS4_GPX0(irq);

	irq -= 8;
	if (irq < 8)
		return EXYNOS4_GPX1(irq);

	irq -= 8;
	if (irq < 8)
		return EXYNOS4_GPX2(irq);

	irq -= 8;
	if (irq < 8)
		return EXYNOS4_GPX3(irq);

	return -EINVAL;
}

static inline int exynos5_irq_to_gpio(unsigned int irq)
{
	if (irq < IRQ_EINT(0))
		return -EINVAL;

	irq -= IRQ_EINT(0);
	if (irq < 8)
		return EXYNOS5_GPX0(irq);

	irq -= 8;
	if (irq < 8)
		return EXYNOS5_GPX1(irq);

	irq -= 8;
	if (irq < 8)
		return EXYNOS5_GPX2(irq);

	irq -= 8;
	if (irq < 8)
		return EXYNOS5_GPX3(irq);

	return -EINVAL;
}

static unsigned int exynos4_eint0_15_src_int[16] = {
	EXYNOS4_IRQ_EINT0,
	EXYNOS4_IRQ_EINT1,
	EXYNOS4_IRQ_EINT2,
	EXYNOS4_IRQ_EINT3,
	EXYNOS4_IRQ_EINT4,
	EXYNOS4_IRQ_EINT5,
	EXYNOS4_IRQ_EINT6,
	EXYNOS4_IRQ_EINT7,
	EXYNOS4_IRQ_EINT8,
	EXYNOS4_IRQ_EINT9,
	EXYNOS4_IRQ_EINT10,
	EXYNOS4_IRQ_EINT11,
	EXYNOS4_IRQ_EINT12,
	EXYNOS4_IRQ_EINT13,
	EXYNOS4_IRQ_EINT14,
	EXYNOS4_IRQ_EINT15,
};

static unsigned int exynos5_eint0_15_src_int[16] = {
	EXYNOS5_IRQ_EINT0,
	EXYNOS5_IRQ_EINT1,
	EXYNOS5_IRQ_EINT2,
	EXYNOS5_IRQ_EINT3,
	EXYNOS5_IRQ_EINT4,
	EXYNOS5_IRQ_EINT5,
	EXYNOS5_IRQ_EINT6,
	EXYNOS5_IRQ_EINT7,
	EXYNOS5_IRQ_EINT8,
	EXYNOS5_IRQ_EINT9,
	EXYNOS5_IRQ_EINT10,
	EXYNOS5_IRQ_EINT11,
	EXYNOS5_IRQ_EINT12,
	EXYNOS5_IRQ_EINT13,
	EXYNOS5_IRQ_EINT14,
	EXYNOS5_IRQ_EINT15,
};
static inline void exynos_irq_eint_mask(struct irq_data *data)
{
	u32 mask;

	spin_lock(&eint_lock);
	mask = __raw_readl(EINT_MASK(exynos_eint_base, data->irq));
	mask |= EINT_OFFSET_BIT(data->irq);
	__raw_writel(mask, EINT_MASK(exynos_eint_base, data->irq));
	spin_unlock(&eint_lock);
}

static void exynos_irq_eint_unmask(struct irq_data *data)
{
	u32 mask;

	spin_lock(&eint_lock);
	mask = __raw_readl(EINT_MASK(exynos_eint_base, data->irq));
	mask &= ~(EINT_OFFSET_BIT(data->irq));
	__raw_writel(mask, EINT_MASK(exynos_eint_base, data->irq));
	spin_unlock(&eint_lock);
}

static inline void exynos_irq_eint_ack(struct irq_data *data)
{
	__raw_writel(EINT_OFFSET_BIT(data->irq),
		     EINT_PEND(exynos_eint_base, data->irq));
}

static void exynos_irq_eint_maskack(struct irq_data *data)
{
	exynos_irq_eint_mask(data);
	exynos_irq_eint_ack(data);
}

static int exynos_irq_eint_set_type(struct irq_data *data, unsigned int type)
{
	int offs = EINT_OFFSET(data->irq);
	int shift;
	u32 ctrl, mask;
	u32 newvalue = 0;

	switch (type) {
	case IRQ_TYPE_EDGE_RISING:
		newvalue = S5P_IRQ_TYPE_EDGE_RISING;
		break;

	case IRQ_TYPE_EDGE_FALLING:
		newvalue = S5P_IRQ_TYPE_EDGE_FALLING;
		break;

	case IRQ_TYPE_EDGE_BOTH:
		newvalue = S5P_IRQ_TYPE_EDGE_BOTH;
		break;

	case IRQ_TYPE_LEVEL_LOW:
		newvalue = S5P_IRQ_TYPE_LEVEL_LOW;
		break;

	case IRQ_TYPE_LEVEL_HIGH:
		newvalue = S5P_IRQ_TYPE_LEVEL_HIGH;
		break;

	default:
		printk(KERN_ERR "No such irq type %d", type);
		return -EINVAL;
	}

	shift = (offs & 0x7) * 4;
	mask = 0x7 << shift;

	spin_lock(&eint_lock);
	ctrl = __raw_readl(EINT_CON(exynos_eint_base, data->irq));
	ctrl &= ~mask;
	ctrl |= newvalue << shift;
	__raw_writel(ctrl, EINT_CON(exynos_eint_base, data->irq));
	spin_unlock(&eint_lock);

	if (soc_is_exynos5250())
		s3c_gpio_cfgpin(exynos5_irq_to_gpio(data->irq), S3C_GPIO_SFN(0xf));
	else
		s3c_gpio_cfgpin(exynos4_irq_to_gpio(data->irq), S3C_GPIO_SFN(0xf));

	return 0;
}

static struct irq_chip exynos_irq_eint = {
	.name		= "exynos-eint",
	.irq_mask	= exynos_irq_eint_mask,
	.irq_unmask	= exynos_irq_eint_unmask,
	.irq_mask_ack	= exynos_irq_eint_maskack,
	.irq_ack	= exynos_irq_eint_ack,
	.irq_set_type	= exynos_irq_eint_set_type,
#ifdef CONFIG_PM
	.irq_set_wake	= s3c_irqext_wake,
#endif
};

/*
 * exynos4_irq_demux_eint
 *
 * This function demuxes the IRQ from from EINTs 16 to 31.
 * It is designed to be inlined into the specific handler
 * s5p_irq_demux_eintX_Y.
 *
 * Each EINT pend/mask registers handle eight of them.
 */
static inline void exynos_irq_demux_eint(unsigned int start)
{
	unsigned int irq;

	u32 status = __raw_readl(EINT_PEND(exynos_eint_base, start));
	u32 mask = __raw_readl(EINT_MASK(exynos_eint_base, start));

	status &= ~mask;
	status &= 0xff;

	while (status) {
		irq = fls(status) - 1;
		generic_handle_irq(irq + start);
		status &= ~(1 << irq);
	}
}

static void exynos_irq_demux_eint16_31(unsigned int irq, struct irq_desc *desc)
{
	struct irq_chip *chip = irq_get_chip(irq);
	chained_irq_enter(chip, desc);
	exynos_irq_demux_eint(IRQ_EINT(16));
	exynos_irq_demux_eint(IRQ_EINT(24));
	chained_irq_exit(chip, desc);
}

static void exynos_irq_eint0_15(unsigned int irq, struct irq_desc *desc)
{
	u32 *irq_data = irq_get_handler_data(irq);
	struct irq_chip *chip = irq_get_chip(irq);

	chained_irq_enter(chip, desc);
	chip->irq_mask(&desc->irq_data);

	if (chip->irq_ack)
		chip->irq_ack(&desc->irq_data);

	generic_handle_irq(*irq_data);

	chip->irq_unmask(&desc->irq_data);
	chained_irq_exit(chip, desc);
}

static int __init exynos_init_irq_eint(void)
{
	int irq;

	if (soc_is_exynos5250())
		exynos_eint_base = ioremap(EXYNOS5_PA_GPIO1, SZ_4K);
	else
		exynos_eint_base = ioremap(EXYNOS4_PA_GPIO2, SZ_4K);

	if (exynos_eint_base == NULL) {
		pr_err("unable to ioremap for EINT base address\n");
		return -ENOMEM;
	}

	for (irq = 0 ; irq <= 31 ; irq++) {
		irq_set_chip_and_handler(IRQ_EINT(irq), &exynos_irq_eint,
					 handle_level_irq);
		set_irq_flags(IRQ_EINT(irq), IRQF_VALID);
	}

	irq_set_chained_handler(EXYNOS_IRQ_EINT16_31, exynos_irq_demux_eint16_31);

	for (irq = 0 ; irq <= 15 ; irq++) {
		eint0_15_data[irq] = IRQ_EINT(irq);

		if (soc_is_exynos5250()) {
			irq_set_handler_data(exynos5_eint0_15_src_int[irq],
					     &eint0_15_data[irq]);
			irq_set_chained_handler(exynos5_eint0_15_src_int[irq],
						exynos_irq_eint0_15);
		} else {
			irq_set_handler_data(exynos4_eint0_15_src_int[irq],
					     &eint0_15_data[irq]);
			irq_set_chained_handler(exynos4_eint0_15_src_int[irq],
						exynos_irq_eint0_15);
		}
	}

	return 0;
}
arch_initcall(exynos_init_irq_eint);
