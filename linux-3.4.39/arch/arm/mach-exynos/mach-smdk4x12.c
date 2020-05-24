/*
 * linux/arch/arm/mach-exynos4/mach-smdk4x12.c
 *
 * Copyright (c) 2011 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/io.h>
#include <linux/mfd/max8997.h>
#include <linux/mmc/host.h>
#include <linux/platform_device.h>
#include <linux/pwm_backlight.h>
#include <linux/regulator/machine.h>
#include <linux/serial_core.h>

#include <asm/mach/arch.h>
#include <asm/hardware/gic.h>
#include <asm/mach-types.h>

#include <plat/backlight.h>
#include <plat/clock.h>
#include <plat/cpu.h>
#include <plat/devs.h>
#include <plat/gpio-cfg.h>
#include <plat/iic.h>
#include <plat/keypad.h>
#include <plat/regs-serial.h>
#include <plat/sdhci.h>

#include <mach/map.h>

#include "common.h"

/* Following are default values for UCON, ULCON and UFCON UART registers */
#define SMDK4X12_UCON_DEFAULT	(S3C2410_UCON_TXILEVEL |	\
				 S3C2410_UCON_RXILEVEL |	\
				 S3C2410_UCON_TXIRQMODE |	\
				 S3C2410_UCON_RXIRQMODE |	\
				 S3C2410_UCON_RXFIFO_TOI |	\
				 S3C2443_UCON_RXERR_IRQEN)

#define SMDK4X12_ULCON_DEFAULT	S3C2410_LCON_CS8

#define SMDK4X12_UFCON_DEFAULT	(S3C2410_UFCON_FIFOMODE |	\
				 S5PV210_UFCON_TXTRIG4 |	\
				 S5PV210_UFCON_RXTRIG4)

static struct s3c2410_uartcfg smdk4x12_uartcfgs[] __initdata = {
	[0] = {
		.hwport		= 0,
		.flags		= 0,
		.ucon		= SMDK4X12_UCON_DEFAULT,
		.ulcon		= SMDK4X12_ULCON_DEFAULT,
		.ufcon		= SMDK4X12_UFCON_DEFAULT,
	},
	[1] = {
		.hwport		= 1,
		.flags		= 0,
		.ucon		= SMDK4X12_UCON_DEFAULT,
		.ulcon		= SMDK4X12_ULCON_DEFAULT,
		.ufcon		= SMDK4X12_UFCON_DEFAULT,
	},
	[2] = {
		.hwport		= 2,
		.flags		= 0,
		.ucon		= SMDK4X12_UCON_DEFAULT,
		.ulcon		= SMDK4X12_ULCON_DEFAULT,
		.ufcon		= SMDK4X12_UFCON_DEFAULT,
	},
	[3] = {
		.hwport		= 3,
		.flags		= 0,
		.ucon		= SMDK4X12_UCON_DEFAULT,
		.ulcon		= SMDK4X12_ULCON_DEFAULT,
		.ufcon		= SMDK4X12_UFCON_DEFAULT,
	},
};

static struct s3c_sdhci_platdata smdk4x12_hsmmc2_pdata __initdata = {
	.cd_type		= S3C_SDHCI_CD_INTERNAL,
	.clk_type		= S3C_SDHCI_CLK_DIV_EXTERNAL,
#ifdef CONFIG_EXYNOS4_SDHCI_CH2_8BIT
	.max_width		= 8,
	.host_caps		= MMC_CAP_8_BIT_DATA,
#endif
};

static struct s3c_sdhci_platdata smdk4x12_hsmmc3_pdata __initdata = {
	.cd_type		= S3C_SDHCI_CD_INTERNAL,
	.clk_type		= S3C_SDHCI_CLK_DIV_EXTERNAL,
};

static struct regulator_consumer_supply max8997_buck1 =
	REGULATOR_SUPPLY("vdd_arm", NULL);

static struct regulator_consumer_supply max8997_buck2 =
	REGULATOR_SUPPLY("vdd_int", NULL);

static struct regulator_consumer_supply max8997_buck3 =
	REGULATOR_SUPPLY("vdd_g3d", NULL);

static struct regulator_init_data max8997_buck1_data = {
	.constraints	= {
		.name		= "VDD_ARM_SMDK4X12",
		.min_uV		= 925000,
		.max_uV		= 1350000,
		.always_on	= 1,
		.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE,
		.state_mem	= {
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &max8997_buck1,
};

static struct regulator_init_data max8997_buck2_data = {
	.constraints	= {
		.name		= "VDD_INT_SMDK4X12",
		.min_uV		= 950000,
		.max_uV		= 1150000,
		.always_on	= 1,
		.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE,
		.state_mem	= {
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &max8997_buck2,
};

static struct regulator_init_data max8997_buck3_data = {
	.constraints	= {
		.name		= "VDD_G3D_SMDK4X12",
		.min_uV		= 950000,
		.max_uV		= 1150000,
		.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE |
				  REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &max8997_buck3,
};

static struct max8997_regulator_data smdk4x12_max8997_regulators[] = {
	{ MAX8997_BUCK1, &max8997_buck1_data },
	{ MAX8997_BUCK2, &max8997_buck2_data },
	{ MAX8997_BUCK3, &max8997_buck3_data },
};

static struct max8997_platform_data smdk4x12_max8997_pdata = {
	.num_regulators	= ARRAY_SIZE(smdk4x12_max8997_regulators),
	.regulators	= smdk4x12_max8997_regulators,

	.buck1_voltage[0] = 1100000,	/* 1.1V */
	.buck1_voltage[1] = 1100000,	/* 1.1V */
	.buck1_voltage[2] = 1100000,	/* 1.1V */
	.buck1_voltage[3] = 1100000,	/* 1.1V */
	.buck1_voltage[4] = 1100000,	/* 1.1V */
	.buck1_voltage[5] = 1100000,	/* 1.1V */
	.buck1_voltage[6] = 1000000,	/* 1.0V */
	.buck1_voltage[7] = 950000,	/* 0.95V */

	.buck2_voltage[0] = 1100000,	/* 1.1V */
	.buck2_voltage[1] = 1000000,	/* 1.0V */
	.buck2_voltage[2] = 950000,	/* 0.95V */
	.buck2_voltage[3] = 900000,	/* 0.9V */
	.buck2_voltage[4] = 1100000,	/* 1.1V */
	.buck2_voltage[5] = 1000000,	/* 1.0V */
	.buck2_voltage[6] = 950000,	/* 0.95V */
	.buck2_voltage[7] = 900000,	/* 0.9V */

	.buck5_voltage[0] = 1100000,	/* 1.1V */
	.buck5_voltage[1] = 1100000,	/* 1.1V */
	.buck5_voltage[2] = 1100000,	/* 1.1V */
	.buck5_voltage[3] = 1100000,	/* 1.1V */
	.buck5_voltage[4] = 1100000,	/* 1.1V */
	.buck5_voltage[5] = 1100000,	/* 1.1V */
	.buck5_voltage[6] = 1100000,	/* 1.1V */
	.buck5_voltage[7] = 1100000,	/* 1.1V */
};

static struct i2c_board_info smdk4x12_i2c_devs0[] __initdata = {
	{
		I2C_BOARD_INFO("max8997", 0x66),
		.platform_data	= &smdk4x12_max8997_pdata,
	}
};

static struct i2c_board_info smdk4x12_i2c_devs1[] __initdata = {
	{ I2C_BOARD_INFO("wm8994", 0x1a), }
};

static struct i2c_board_info smdk4x12_i2c_devs3[] __initdata = {
	/* nothing here yet */
};

static struct i2c_board_info smdk4x12_i2c_devs7[] __initdata = {
	/* nothing here yet */
};

static struct samsung_bl_gpio_info smdk4x12_bl_gpio_info = {
	.no = EXYNOS4_GPD0(1),
	.func = S3C_GPIO_SFN(2),
};

static struct platform_pwm_backlight_data smdk4x12_bl_data = {
	.pwm_id = 1,
	.pwm_period_ns  = 1000,
};

static uint32_t smdk4x12_keymap[] __initdata = {
	/* KEY(row, col, keycode) */
	KEY(1, 0, KEY_D), KEY(1, 1, KEY_A), KEY(1, 2, KEY_B),
	KEY(1, 3, KEY_E), KEY(1, 4, KEY_C)
};

static struct matrix_keymap_data smdk4x12_keymap_data __initdata = {
	.keymap		= smdk4x12_keymap,
	.keymap_size	= ARRAY_SIZE(smdk4x12_keymap),
};

static struct samsung_keypad_platdata smdk4x12_keypad_data __initdata = {
	.keymap_data	= &smdk4x12_keymap_data,
	.rows		= 2,
	.cols		= 5,
};

static struct platform_device *smdk4x12_devices[] __initdata = {
	&s3c_device_hsmmc2,
	&s3c_device_hsmmc3,
	&s3c_device_i2c0,
	&s3c_device_i2c1,
	&s3c_device_i2c3,
	&s3c_device_i2c7,
	&s3c_device_rtc,
	&s3c_device_wdt,
	&samsung_device_keypad,
};
extern void __init early_print(const char *str, ...);
struct meminfo;
#include <asm/setup.h>
#ifndef __CFG_MEM_H__
#define __CFG_MEM_H__

/*------------------------------------------------------------------------------
 *       System memory map
 */
#define CFG_MEM_PHY_SYSTEM_BASE                 0x40000000      /* System, must be at an evne 2MB boundary (head.S) */
#define CFG_MEM_PHY_SYSTEM_SIZE                 0x3F000000      /* 0x3F000000:Total 1GB 0x7F000000:Total 2GB */

/*------------------------------------------------------------------------------
 *   DMA zone, if not defined DAM default size is 2M
 */
#define CFG_MEM_PHY_DMAZONE_SIZE        0x01000000  /* 16 MB DMA zone */

#endif /* __CFG_MEM_H__ */

/*------------------------------------------------------------------------------
 *      cpu initialize and io/memory map.
 *      procedure: fixup -> map_io -> init_irq -> timer init -> init_machine
 */
static void __init cpu_fixup(struct tag *tags, char **cmdline, struct meminfo *mi)
{
        early_print("%s\n", __func__);
        /*
         * system momory  = system mem size + dma zone size
         */
    mi->nr_banks      = 1;
        mi->bank[0].start = CFG_MEM_PHY_SYSTEM_BASE;
#if !defined(CFG_MEM_PHY_DMAZONE_SIZE)
        mi->bank[0].size  = CFG_MEM_PHY_SYSTEM_SIZE;
#else
    mi->bank[0].size  = CFG_MEM_PHY_SYSTEM_SIZE + CFG_MEM_PHY_DMAZONE_SIZE;
#endif
}

static void __init smdk4x12_map_io(void)
{
	clk_xusbxti.rate = 24000000;

	exynos_init_io(NULL, 0);
	early_print("call map_io.\n");
	/*s3c24xx_init_clocks(clk_xusbxti.rate);*/
	s3c24xx_init_uarts(smdk4x12_uartcfgs, ARRAY_SIZE(smdk4x12_uartcfgs));
}

static void __init smdk4x12_machine_init(void)
{
#if 0
	s3c_i2c0_set_platdata(NULL);
	i2c_register_board_info(0, smdk4x12_i2c_devs0,
				ARRAY_SIZE(smdk4x12_i2c_devs0));

	s3c_i2c1_set_platdata(NULL);
	i2c_register_board_info(1, smdk4x12_i2c_devs1,
				ARRAY_SIZE(smdk4x12_i2c_devs1));

	s3c_i2c3_set_platdata(NULL);
	i2c_register_board_info(3, smdk4x12_i2c_devs3,
				ARRAY_SIZE(smdk4x12_i2c_devs3));

	s3c_i2c7_set_platdata(NULL);
	i2c_register_board_info(7, smdk4x12_i2c_devs7,
				ARRAY_SIZE(smdk4x12_i2c_devs7));

	samsung_bl_set(&smdk4x12_bl_gpio_info, &smdk4x12_bl_data);

	samsung_keypad_set_platdata(&smdk4x12_keypad_data);

	s3c_sdhci2_set_platdata(&smdk4x12_hsmmc2_pdata);
	s3c_sdhci3_set_platdata(&smdk4x12_hsmmc3_pdata);

	platform_add_devices(smdk4x12_devices, ARRAY_SIZE(smdk4x12_devices));
#endif
}

MACHINE_START(SMDK4212, "SMDK4212")
	/* Maintainer: Kukjin Kim <kgene.kim@samsung.com> */
	.fixup		= cpu_fixup,
	.atag_offset	= 0x100,
	.init_irq	= exynos4_init_irq,
	.map_io		= smdk4x12_map_io,
	.handle_irq	= gic_handle_irq,
	.init_machine	= smdk4x12_machine_init,
	.timer		= &exynos4_timer,
	.restart	= exynos4_restart,
MACHINE_END

