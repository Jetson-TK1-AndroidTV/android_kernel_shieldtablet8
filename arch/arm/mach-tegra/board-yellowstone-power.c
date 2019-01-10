/*
 * arch/arm/mach-tegra/board-yellowstone-power.c
 *
 * Copyright (c) 2013-2014, NVIDIA CORPORATION. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/i2c.h>
#include <linux/i2c/pca953x.h>
#include <linux/platform_device.h>
#include <linux/resource.h>
#include <linux/io.h>
#include <mach/edp.h>
#include <mach/irqs.h>
#include <linux/edp.h>
#include <linux/platform_data/tegra_edp.h>
#include <linux/pid_thermal_gov.h>
#include <linux/regulator/fixed.h>
#include <linux/regulator/machine.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/mfd/palmas.h>
#include <linux/mfd/as3722-plat.h>
#include <linux/regulator/tegra-dfll-bypass-regulator.h>
#include <linux/regulator/tps51632-regulator.h>
#include <linux/tegra-fuse.h>
#include <linux/tegra-pmc.h>
#include <linux/power/power_supply_extcon.h>
#include <linux/power/bq2477x-charger.h>
#include <linux/delay.h>

#include <asm/mach-types.h>
#include <mach/pinmux-t12.h>

#include "pm.h"
#include "dvfs.h"
#include "board.h"
#include "common.h"
#include "tegra-board-id.h"
#include "board-pmu-defines.h"
#include "board-common.h"
#include "board-yellowstone.h"
#include "board-pmu-defines.h"
#include "devices.h"
#include "iomap.h"
#include "tegra_cl_dvfs.h"
#include "tegra11_soctherm.h"
#include "tegra3_tsensor.h"

#define E1735_EMULATE_E1767_SKU	1001

#define PMC_CTRL                0x0
#define PMC_CTRL_INTR_LOW       (1 << 17)

/************************ ARDBEG E1733 based regulators ***********/
static struct regulator_consumer_supply as3722_ldo0_supply[] = {
	REGULATOR_SUPPLY("avdd_pll_m", NULL),
	REGULATOR_SUPPLY("avdd_pll_ap_c2_c3", NULL),
	REGULATOR_SUPPLY("avdd_pll_cud2dpd", NULL),
	REGULATOR_SUPPLY("avdd_pll_c4", NULL),
	REGULATOR_SUPPLY("avdd_lvds0_io", NULL),
	REGULATOR_SUPPLY("vddio_ddr_hs", NULL),
	REGULATOR_SUPPLY("avdd_pll_erefe", NULL),
	REGULATOR_SUPPLY("avdd_pll_x", NULL),
	REGULATOR_SUPPLY("avdd_pll_cg", NULL),
};

static struct regulator_consumer_supply as3722_ldo1_supply[] = {
	REGULATOR_SUPPLY("vdd_cam1_1v8_cam", NULL),
	REGULATOR_SUPPLY("vdd_cam2_1v8_cam", NULL),
	REGULATOR_SUPPLY("vif", "2-0010"),
	REGULATOR_SUPPLY("vif", "2-0036"),
	REGULATOR_SUPPLY("vdd_i2c", "2-000c"),
	REGULATOR_SUPPLY("vi2c", "2-0030"),
	REGULATOR_SUPPLY("vif2", "2-0021"),
	REGULATOR_SUPPLY("dovdd", "2-0010"),
	REGULATOR_SUPPLY("vdd", "2-004a"),
	REGULATOR_SUPPLY("vif", "2-0048"),
};

static struct regulator_consumer_supply as3722_ldo2_supply[] = {
	REGULATOR_SUPPLY("vddio_hsic", "tegra-ehci.1"),
	REGULATOR_SUPPLY("vddio_hsic", "tegra-ehci.2"),
	REGULATOR_SUPPLY("vddio_hsic", "tegra-xhci"),
	REGULATOR_SUPPLY("avdd_dsi_csi", "tegradc.0"),
	REGULATOR_SUPPLY("avdd_dsi_csi", "tegradc.1"),
	REGULATOR_SUPPLY("avdd_dsi_csi", "vi.0"),
	REGULATOR_SUPPLY("avdd_dsi_csi", "vi.1"),
	REGULATOR_SUPPLY("pwrdet_mipi", NULL),
	REGULATOR_SUPPLY("avdd_hsic_com", NULL),
	REGULATOR_SUPPLY("avdd_hsic_mdm", NULL),
	REGULATOR_SUPPLY("vdd_lcd_bl", NULL),
};

static struct regulator_consumer_supply as3722_ldo3_supply[] = {
	REGULATOR_SUPPLY("vdd_rtc", NULL),
};

static struct regulator_consumer_supply as3722_ldo4_supply[] = {
	REGULATOR_SUPPLY("vdd_2v7_hv", NULL),
	REGULATOR_SUPPLY("avdd_cam1_cam", NULL),
	REGULATOR_SUPPLY("avdd_cam2_cam", NULL),
	REGULATOR_SUPPLY("avdd_cam3_cam", NULL),
	REGULATOR_SUPPLY("vana", "2-0010"),
	REGULATOR_SUPPLY("avdd_ov5693", "2-0010"),
};

static struct regulator_consumer_supply as3722_ldo5_supply[] = {
	REGULATOR_SUPPLY("vdd_1v2_cam", NULL),
	REGULATOR_SUPPLY("vdig", "2-0010"),
	REGULATOR_SUPPLY("vdig", "2-0036"),
};

static struct regulator_consumer_supply as3722_ldo6_supply[] = {
	REGULATOR_SUPPLY("vddio_sdmmc", "sdhci-tegra.2"),
	REGULATOR_SUPPLY("pwrdet_sdmmc3", NULL),
};

static struct regulator_consumer_supply as3722_ldo7_supply[] = {
	REGULATOR_SUPPLY("vdd_cam_1v1_cam", NULL),
	REGULATOR_SUPPLY("imx135_reg2", NULL),
	REGULATOR_SUPPLY("vdig_lv", "2-0010"),
	REGULATOR_SUPPLY("dvdd", "2-0010"),
};

static struct regulator_consumer_supply as3722_ldo9_supply[] = {
	REGULATOR_SUPPLY("avdd", "spi0.0"),
};

static struct regulator_consumer_supply as3722_ldo10_supply[] = {
	REGULATOR_SUPPLY("avdd_af1_cam", NULL),
	REGULATOR_SUPPLY("imx135_reg1", NULL),
	REGULATOR_SUPPLY("vdd", "2-000c"),
	REGULATOR_SUPPLY("vin", "2-0030"),
	REGULATOR_SUPPLY("vana", "2-0036"),
	REGULATOR_SUPPLY("vana", "2-0021"),
	REGULATOR_SUPPLY("vdd_af1", "2-0010"),
	REGULATOR_SUPPLY("vin", "2-004a"),
	REGULATOR_SUPPLY("vana", "2-0048"),
};

static struct regulator_consumer_supply as3722_ldo11_supply[] = {
	REGULATOR_SUPPLY("vpp_fuse", NULL),
};

static struct regulator_consumer_supply as3722_sd0_supply[] = {
	REGULATOR_SUPPLY("vdd_cpu", NULL),
};

static struct regulator_consumer_supply as3722_sd1_supply[] = {
	REGULATOR_SUPPLY("vdd_core", NULL),
};

static struct regulator_consumer_supply as3722_sd2_supply[] = {
	REGULATOR_SUPPLY("vddio_ddr", NULL),
	REGULATOR_SUPPLY("vddio_ddr_mclk", NULL),
	REGULATOR_SUPPLY("vddio_ddr3", NULL),
	REGULATOR_SUPPLY("vcore1_ddr3", NULL),
};

static struct regulator_consumer_supply as3722_sd4_supply[] = {
	REGULATOR_SUPPLY("avdd_hdmi", "tegradc.1"),
	REGULATOR_SUPPLY("avdd_hdmi_pll", "tegradc.1"),
#ifdef CONFIG_TEGRA_HDMI_PRIMARY
	REGULATOR_SUPPLY("avdd_hdmi", "tegradc.0"),
	REGULATOR_SUPPLY("avdd_hdmi_pll", "tegradc.0"),
#endif
	REGULATOR_SUPPLY("avdd_pex_pll", "tegra-pcie"),
	REGULATOR_SUPPLY("avddio_pex", "tegra-pcie"),
	REGULATOR_SUPPLY("dvddio_pex", "tegra-pcie"),
	REGULATOR_SUPPLY("avddio_usb", "tegra-xhci"),
	REGULATOR_SUPPLY("vddio_pex_sata", "tegra-sata.0"),
};

static struct regulator_consumer_supply as3722_sd5_supply[] = {
	REGULATOR_SUPPLY("dbvdd", "tegra-snd-rt5639.0"),
	REGULATOR_SUPPLY("dbvdd", "tegra-snd-rt5645.0"),
	REGULATOR_SUPPLY("avdd", "tegra-snd-rt5639.0"),
	REGULATOR_SUPPLY("avdd", "tegra-snd-rt5645.0"),
	REGULATOR_SUPPLY("dmicvdd", "tegra-snd-rt5639.0"),
	REGULATOR_SUPPLY("dmicvdd", "tegra-snd-rt5645.0"),
	REGULATOR_SUPPLY("avdd_osc", NULL),
	REGULATOR_SUPPLY("vddio_sys", NULL),
	REGULATOR_SUPPLY("vddio_sys_2", NULL),
	REGULATOR_SUPPLY("vddio_gmi", NULL),
	REGULATOR_SUPPLY("pwrdet_nand", NULL),
	REGULATOR_SUPPLY("vddio_sdmmc", "sdhci-tegra.0"),
	REGULATOR_SUPPLY("pwrdet_sdmmc1", NULL),
	REGULATOR_SUPPLY("vddio_sdmmc", "sdhci-tegra.3"),
	REGULATOR_SUPPLY("pwrdet_sdmmc4", NULL),
	REGULATOR_SUPPLY("avdd_pll_utmip", "tegra-udc.0"),
	REGULATOR_SUPPLY("avdd_pll_utmip", "tegra-ehci.0"),
	REGULATOR_SUPPLY("avdd_pll_utmip", "tegra-ehci.1"),
	REGULATOR_SUPPLY("avdd_pll_utmip", "tegra-ehci.2"),
	REGULATOR_SUPPLY("avdd_pll_utmip", "tegra-xhci"),
	REGULATOR_SUPPLY("vddio_audio", NULL),
	REGULATOR_SUPPLY("pwrdet_audio", NULL),
	REGULATOR_SUPPLY("vddio_uart", NULL),
	REGULATOR_SUPPLY("pwrdet_uart", NULL),
	REGULATOR_SUPPLY("vddio_bb", NULL),
	REGULATOR_SUPPLY("pwrdet_bb", NULL),
	REGULATOR_SUPPLY("vdd_1v8b", "0-0048"),
	REGULATOR_SUPPLY("vdd_dtv", NULL),
	REGULATOR_SUPPLY("vdd_1v8_eeprom", NULL),
	REGULATOR_SUPPLY("vddio_cam", "tegra_camera"),
	REGULATOR_SUPPLY("vddio_cam", "vi"),
	REGULATOR_SUPPLY("pwrdet_cam", NULL),
	REGULATOR_SUPPLY("dvdd", "spi0.0"),
	REGULATOR_SUPPLY("vlogic", "0-0069"),
	REGULATOR_SUPPLY("vid", "0-000c"),
	REGULATOR_SUPPLY("vddio", "0-0077"),
	REGULATOR_SUPPLY("vdd_sata", "tegra-sata.0"),
	REGULATOR_SUPPLY("avdd_sata", "tegra-sata.0"),
	REGULATOR_SUPPLY("avdd_sata_pll", "tegra-sata.0"),
};

static struct regulator_consumer_supply as3722_sd6_supply[] = {
	REGULATOR_SUPPLY("vdd_gpu", NULL),
};


AMS_PDATA_INIT(sd0, NULL, 700000, 1400000, 1, 1, 1, AS3722_EXT_CONTROL_ENABLE2);
AMS_PDATA_INIT(sd1, NULL, 700000, 1400000, 1, 1, 1, AS3722_EXT_CONTROL_ENABLE1);
AMS_PDATA_INIT(sd2, NULL, 1350000, 1350000, 1, 1, 1, 0);
AMS_PDATA_INIT(sd4, NULL, 1050000, 1050000, 0, 1, 1, 0);
AMS_PDATA_INIT(sd5, NULL, 1800000, 1800000, 1, 1, 1, 0);
AMS_PDATA_INIT(sd6, NULL, 650000, 1400000, 0, 1, 0, 0);
AMS_PDATA_INIT(ldo0, AS3722_SUPPLY(sd2), 1050000, 1250000, 1, 1, 1, AS3722_EXT_CONTROL_ENABLE1);
AMS_PDATA_INIT(ldo1, NULL, 1800000, 1800000, 0, 1, 1, 0);
AMS_PDATA_INIT(ldo2, AS3722_SUPPLY(sd5), 1200000, 1200000, 0, 0, 1, 0);
AMS_PDATA_INIT(ldo3, NULL, 800000, 800000, 1, 1, 1, 0);
AMS_PDATA_INIT(ldo4, NULL, 2700000, 2700000, 0, 0, 1, 0);
AMS_PDATA_INIT(ldo5, AS3722_SUPPLY(sd5), 1200000, 1200000, 0, 0, 1, 0);
AMS_PDATA_INIT(ldo6, NULL, 1800000, 3300000, 0, 0, 1, 0);
AMS_PDATA_INIT(ldo7, AS3722_SUPPLY(sd5), 1050000, 1050000, 0, 0, 1, 0);
AMS_PDATA_INIT(ldo9, NULL, 3300000, 3300000, 0, 0, 1, 0);
AMS_PDATA_INIT(ldo10, NULL, 2700000, 2700000, 0, 0, 1, 0);
AMS_PDATA_INIT(ldo11, NULL, 1800000, 1800000, 0, 0, 1, 0);

static struct as3722_pinctrl_platform_data as3722_pctrl_pdata[] = {
	AS3722_PIN_CONTROL("gpio0", "gpio", "pull-down", NULL, NULL, "output-low"),
	AS3722_PIN_CONTROL("gpio1", "gpio", NULL, NULL, NULL, "output-high"),
	AS3722_PIN_CONTROL("gpio2", "gpio", NULL, NULL, NULL, "output-high"),
	AS3722_PIN_CONTROL("gpio3", "gpio", NULL, NULL, "enabled", NULL),
	AS3722_PIN_CONTROL("gpio4", "gpio", NULL, NULL, NULL, "output-high"),
	AS3722_PIN_CONTROL("gpio5", "clk32k-out", NULL, NULL, NULL, NULL),
	AS3722_PIN_CONTROL("gpio6", "gpio", NULL, NULL, "enabled", NULL),
	AS3722_PIN_CONTROL("gpio7", "gpio", NULL, NULL, NULL, "output-low"),
};

static struct as3722_adc_extcon_platform_data as3722_adc_extcon_pdata = {
	.connection_name = "as3722-extcon",
	.enable_adc1_continuous_mode = true,
	.enable_low_voltage_range = true,
	.adc_channel = 12,
	.hi_threshold =  0x100,
	.low_threshold = 0x80,
};

static struct as3722_platform_data as3722_pdata = {
	.reg_pdata[AS3722_SD0] = &as3722_sd0_reg_pdata,
	.reg_pdata[AS3722_SD1] = &as3722_sd1_reg_pdata,
	.reg_pdata[AS3722_SD2] = &as3722_sd2_reg_pdata,
	.reg_pdata[AS3722_SD4] = &as3722_sd4_reg_pdata,
	.reg_pdata[AS3722_SD5] = &as3722_sd5_reg_pdata,
	.reg_pdata[AS3722_SD6] = &as3722_sd6_reg_pdata,
	.reg_pdata[AS3722_LDO0] = &as3722_ldo0_reg_pdata,
	.reg_pdata[AS3722_LDO1] = &as3722_ldo1_reg_pdata,
	.reg_pdata[AS3722_LDO2] = &as3722_ldo2_reg_pdata,
	.reg_pdata[AS3722_LDO3] = &as3722_ldo3_reg_pdata,
	.reg_pdata[AS3722_LDO4] = &as3722_ldo4_reg_pdata,
	.reg_pdata[AS3722_LDO5] = &as3722_ldo5_reg_pdata,
	.reg_pdata[AS3722_LDO6] = &as3722_ldo6_reg_pdata,
	.reg_pdata[AS3722_LDO7] = &as3722_ldo7_reg_pdata,
	.reg_pdata[AS3722_LDO9] = &as3722_ldo9_reg_pdata,
	.reg_pdata[AS3722_LDO10] = &as3722_ldo10_reg_pdata,
	.reg_pdata[AS3722_LDO11] = &as3722_ldo11_reg_pdata,
	.gpio_base = AS3722_GPIO_BASE,
	.irq_base = AS3722_IRQ_BASE,
	.use_internal_int_pullup = 0,
	.use_internal_i2c_pullup = 0,
	.pinctrl_pdata = as3722_pctrl_pdata,
	.num_pinctrl = ARRAY_SIZE(as3722_pctrl_pdata),
	.enable_clk32k_out = true,
	.use_power_off = true,
	.extcon_pdata = &as3722_adc_extcon_pdata,
};

static struct i2c_board_info __initdata as3722_regulators[] = {
	{
		I2C_BOARD_INFO("as3722", 0x40),
		.flags = I2C_CLIENT_WAKE,
		.irq = INT_EXTERNAL_PMU,
		.platform_data = &as3722_pdata,
	},
};

int __init ardbeg_as3722_regulator_init(void)
{
	void __iomem *pmc = IO_ADDRESS(TEGRA_PMC_BASE);
	u32 pmc_ctrl;
	struct board_info board_info;

	/* AS3722: Normal state of INT request line is LOW.
	 * configure the power management controller to trigger PMU
	 * interrupts when HIGH.
	 */
	pmc_ctrl = readl(pmc + PMC_CTRL);
	writel(pmc_ctrl | PMC_CTRL_INTR_LOW, pmc + PMC_CTRL);

	/* Set vdd_gpu init uV to 1V */
	as3722_sd6_reg_idata.constraints.init_uV = 900000;

	as3722_sd6_reg_idata.constraints.min_uA = 3500000;
	as3722_sd6_reg_idata.constraints.max_uA = 3500000;

	as3722_sd0_reg_idata.constraints.min_uA = 3500000;
	as3722_sd0_reg_idata.constraints.max_uA = 3500000;

	as3722_sd1_reg_idata.constraints.min_uA = 2500000;
	as3722_sd1_reg_idata.constraints.max_uA = 2500000;

	as3722_ldo3_reg_pdata.enable_tracking = true;
	as3722_ldo3_reg_pdata.disable_tracking_suspend = true;

	tegra_get_board_info(&board_info);
	if ((board_info.board_id == BOARD_E1792) ||
	    (board_info.board_id == BOARD_E1971) ||
	    (board_info.board_id == BOARD_E1973)) {
		/*Default DDR voltage is 1.35V but lpddr3 supports 1.2V*/
		as3722_sd2_reg_idata.constraints.min_uV = 1200000;
		as3722_sd2_reg_idata.constraints.max_uV = 1200000;
	}

	pr_info("%s: i2c_register_board_info\n", __func__);
	i2c_register_board_info(4, as3722_regulators,
			ARRAY_SIZE(as3722_regulators));
	return 0;
}

int __init ardbeg_ams_regulator_init(void)
{
	ardbeg_as3722_regulator_init();
	return 0;
}

/**************** ARDBEG-TI913 based regulator ************/
#define palmas_ti913_smps123_supply as3722_sd6_supply
#define palmas_ti913_smps45_supply as3722_sd1_supply
#define palmas_ti913_smps6_supply as3722_sd5_supply
#define palmas_ti913_smps7_supply as3722_sd2_supply
#define palmas_ti913_smps9_supply as3722_sd4_supply
#define palmas_ti913_ldo1_supply as3722_ldo0_supply
#define palmas_ti913_ldo2_supply as3722_ldo5_supply
#define palmas_ti913_ldo3_supply as3722_ldo9_supply
#define palmas_ti913_ldo4_supply as3722_ldo2_supply
#define palmas_ti913_ldo5_supply as3722_ldo4_supply
#define palmas_ti913_ldo6_supply as3722_ldo1_supply
#define palmas_ti913_ldo7_supply as3722_ldo10_supply
#define palmas_ti913_ldo8_supply as3722_ldo3_supply
#define palmas_ti913_ldo9_supply as3722_ldo6_supply
#define palmas_ti913_ldoln_supply as3722_ldo7_supply
#define palmas_ti913_ldousb_supply as3722_ldo11_supply

static struct regulator_consumer_supply palmas_ti913_regen1_supply[] = {
	REGULATOR_SUPPLY("micvdd", "tegra-snd-rt5645.0"),
#ifdef CONFIG_TEGRA_HDMI_PRIMARY
	REGULATOR_SUPPLY("vddio_hv", "tegradc.0"),
#endif
	REGULATOR_SUPPLY("vddio_hv", "tegradc.1"),
	REGULATOR_SUPPLY("pwrdet_hv", NULL),
	REGULATOR_SUPPLY("avdd_usb", "tegra-udc.0"),
	REGULATOR_SUPPLY("avdd_usb", "tegra-ehci.0"),
	REGULATOR_SUPPLY("avdd_usb", "tegra-ehci.1"),
	REGULATOR_SUPPLY("avdd_usb", "tegra-ehci.2"),
	REGULATOR_SUPPLY("hvdd_usb", "tegra-xhci"),
	REGULATOR_SUPPLY("vddio_sd_slot", "sdhci-tegra.3"),
	REGULATOR_SUPPLY("hvdd_pex", "tegra-pcie"),
	REGULATOR_SUPPLY("hvdd_pex_pll_e", "tegra-pcie"),
	REGULATOR_SUPPLY("vddio_pex_ctl", "tegra-pcie"),
	REGULATOR_SUPPLY("pwrdet_pex_ctl", NULL),
	REGULATOR_SUPPLY("vdd", "0-0069"),
	REGULATOR_SUPPLY("vdd", "0-0048"),
	REGULATOR_SUPPLY("vdd", "stm8t143.2"),
	REGULATOR_SUPPLY("vdd", "0-000c"),
	REGULATOR_SUPPLY("vdd", "0-0077"),
	REGULATOR_SUPPLY("hvdd_sata", "tegra-sata.0"),
	REGULATOR_SUPPLY("vdd", "1-004c"),
	REGULATOR_SUPPLY("vdd", "1-004d"),
	REGULATOR_SUPPLY("vcc", "1-0071"),
};

PALMAS_REGS_PDATA(ti913_smps123, 650, 1400, NULL, 0, 1, 1, NORMAL,
	0, 0, 0, 0, 0);
PALMAS_REGS_PDATA(ti913_smps45, 700, 1400, NULL, 1, 1, 1, NORMAL,
	0, PALMAS_EXT_CONTROL_NSLEEP, 0, 0, 0);
PALMAS_REGS_PDATA(ti913_smps6, 1800, 1800, NULL, 1, 1, 1, NORMAL,
	0, 0, 0, 0, 0);
PALMAS_REGS_PDATA(ti913_smps7, 900, 1350, NULL, 1, 1, 1, NORMAL,
	0, 0, 0, 0, 0);
PALMAS_REGS_PDATA(ti913_smps9, 1050, 1050, NULL, 0, 0, 0, NORMAL,
	0, 0, 0, 0, 0);
PALMAS_REGS_PDATA(ti913_ldo1, 1050, 1250, palmas_rails(ti913_smps7),
		1, 1, 1, 0, 0, PALMAS_EXT_CONTROL_NSLEEP, 0, 0, 0);
PALMAS_REGS_PDATA(ti913_ldo2, 1200, 1200, palmas_rails(ti913_smps6),
		0, 0, 1, 0, 0, 0, 0, 0, 0);
PALMAS_REGS_PDATA(ti913_ldo3, 3100, 3100, NULL, 0, 0, 1, 0, 0, 0, 0, 0, 0);
PALMAS_REGS_PDATA(ti913_ldo4, 1200, 1200, palmas_rails(ti913_smps6),
		0, 0, 1, 0, 0, 0, 0, 0, 0);
PALMAS_REGS_PDATA(ti913_ldo5, 2700, 2700, NULL, 0, 0, 1, 0, 0, 0, 0, 0, 0);
PALMAS_REGS_PDATA(ti913_ldo6, 1800, 1800, NULL, 1, 1, 1, 0, 0, 0, 0, 0, 0);
PALMAS_REGS_PDATA(ti913_ldo7, 2700, 2700, NULL, 0, 0, 1, 0, 0, 0, 0, 0, 0);
PALMAS_REGS_PDATA(ti913_ldo8, 800, 800, NULL, 1, 1, 1, 0, 0, 0, 0, 0, 0);
PALMAS_REGS_PDATA(ti913_ldo9, 1800, 3300, NULL, 0, 0, 1, 0, 0, 0, 0, 0, 0);
PALMAS_REGS_PDATA(ti913_ldoln, 1050, 1050, palmas_rails(ti913_smps6),
		0, 0, 1, 0, 0, 0, 0, 0, 0);
PALMAS_REGS_PDATA(ti913_ldousb, 1800, 1800, NULL, 1, 1, 1, 0, 0, 0, 0, 0, 0);
PALMAS_REGS_PDATA(ti913_regen1, 2800, 3300, NULL, 1, 1, 1, 0, 0, 0, 0, 0, 0);

#define PALMAS_REG_PDATA(_sname) &reg_idata_##_sname
static struct regulator_init_data *ardbeg_1735_reg_data[PALMAS_NUM_REGS] = {
	NULL,
	PALMAS_REG_PDATA(ti913_smps123),
	NULL,
	PALMAS_REG_PDATA(ti913_smps45),
	NULL,
	PALMAS_REG_PDATA(ti913_smps6),
	PALMAS_REG_PDATA(ti913_smps7),
	NULL,
	PALMAS_REG_PDATA(ti913_smps9),
	NULL,
	NULL,
	PALMAS_REG_PDATA(ti913_ldo1),
	PALMAS_REG_PDATA(ti913_ldo2),
	PALMAS_REG_PDATA(ti913_ldo3),
	PALMAS_REG_PDATA(ti913_ldo4),
	PALMAS_REG_PDATA(ti913_ldo5),
	PALMAS_REG_PDATA(ti913_ldo6),
	PALMAS_REG_PDATA(ti913_ldo7),
	PALMAS_REG_PDATA(ti913_ldo8),
	PALMAS_REG_PDATA(ti913_ldo9),
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	PALMAS_REG_PDATA(ti913_ldoln),
	PALMAS_REG_PDATA(ti913_ldousb),
	PALMAS_REG_PDATA(ti913_regen1),
	NULL,
	NULL,
	NULL,
	NULL,
};

#define PALMAS_REG_INIT_DATA(_sname) &reg_init_data_##_sname
static struct palmas_reg_init *ardbeg_1735_reg_init[PALMAS_NUM_REGS] = {
	NULL,
	PALMAS_REG_INIT_DATA(ti913_smps123),
	NULL,
	PALMAS_REG_INIT_DATA(ti913_smps45),
	NULL,
	PALMAS_REG_INIT_DATA(ti913_smps6),
	PALMAS_REG_INIT_DATA(ti913_smps7),
	NULL,
	PALMAS_REG_INIT_DATA(ti913_smps9),
	NULL,
	NULL,
	PALMAS_REG_INIT_DATA(ti913_ldo1),
	PALMAS_REG_INIT_DATA(ti913_ldo2),
	PALMAS_REG_INIT_DATA(ti913_ldo3),
	PALMAS_REG_INIT_DATA(ti913_ldo4),
	PALMAS_REG_INIT_DATA(ti913_ldo5),
	PALMAS_REG_INIT_DATA(ti913_ldo6),
	PALMAS_REG_INIT_DATA(ti913_ldo7),
	PALMAS_REG_INIT_DATA(ti913_ldo8),
	PALMAS_REG_INIT_DATA(ti913_ldo9),
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	PALMAS_REG_INIT_DATA(ti913_ldoln),
	PALMAS_REG_INIT_DATA(ti913_ldousb),
	PALMAS_REG_INIT_DATA(ti913_regen1),
	NULL,
	NULL,
	NULL,
	NULL,
};

struct palmas_clk32k_init_data palmas_ti913_clk32k_idata[] = {
	{
		.clk32k_id = PALMAS_CLOCK32KG,
		.enable = true,
	}, {
		.clk32k_id = PALMAS_CLOCK32KG_AUDIO,
		.enable = true,
	},
};

static struct palmas_extcon_platform_data palmas_extcon_pdata = {
	.connection_name = "palmas-extcon",
	.enable_vbus_detection = true,
	.enable_id_pin_detection = true,
};

static struct palmas_pinctrl_config palmas_ti913_pincfg[] = {
	PALMAS_PINMUX("powergood", "powergood", NULL, NULL),
	PALMAS_PINMUX("vac", "vac", NULL, NULL),
	PALMAS_PINMUX("gpio0", "id", "pull-up", NULL),
	PALMAS_PINMUX("gpio1", "vbus_det", NULL, NULL),
	PALMAS_PINMUX("gpio2", "gpio", NULL, NULL),
	PALMAS_PINMUX("gpio3", "gpio", NULL, NULL),
	PALMAS_PINMUX("gpio4", "gpio", NULL, NULL),
	PALMAS_PINMUX("gpio5", "clk32kgaudio", NULL, NULL),
	PALMAS_PINMUX("gpio6", "gpio", NULL, NULL),
	PALMAS_PINMUX("gpio7", "gpio", NULL, NULL),
};

static struct palmas_pinctrl_platform_data palmas_ti913_pinctrl_pdata = {
	.pincfg = palmas_ti913_pincfg,
	.num_pinctrl = ARRAY_SIZE(palmas_ti913_pincfg),
	.dvfs1_enable = true,
	.dvfs2_enable = false,
};

static struct palmas_pmic_platform_data pmic_ti913_platform = {
};

static struct palmas_pm_platform_data palmas_pm_pdata = {
	.use_power_off = true,
	.use_power_reset = true,
};

static struct palmas_platform_data palmas_ti913_pdata = {
	.gpio_base = PALMAS_TEGRA_GPIO_BASE,
	.irq_base = PALMAS_TEGRA_IRQ_BASE,
	.pmic_pdata = &pmic_ti913_platform,
	.pinctrl_pdata = &palmas_ti913_pinctrl_pdata,
	.clk32k_init_data =  palmas_ti913_clk32k_idata,
	.clk32k_init_data_size = ARRAY_SIZE(palmas_ti913_clk32k_idata),
	.extcon_pdata = &palmas_extcon_pdata,
	.pm_pdata = &palmas_pm_pdata,
};

static struct i2c_board_info palma_ti913_device[] = {
	{
		I2C_BOARD_INFO("tps65913", 0x58),
		.irq            = INT_EXTERNAL_PMU,
		.platform_data  = &palmas_ti913_pdata,
	},
};

int __init ardbeg_tps65913_regulator_init(void)
{
	void __iomem *pmc = IO_ADDRESS(TEGRA_PMC_BASE);
	u32 pmc_ctrl;
	int i;
	struct board_info board_info;

	/* TPS65913: Normal state of INT request line is LOW.
	 * configure the power management controller to trigger PMU
	 * interrupts when HIGH.
	 */
	pmc_ctrl = readl(pmc + PMC_CTRL);
	writel(pmc_ctrl | PMC_CTRL_INTR_LOW, pmc + PMC_CTRL);

	/* Tracking configuration */
	reg_init_data_ti913_ldo8.config_flags =
		PALMAS_REGULATOR_CONFIG_TRACKING_ENABLE;

	for (i = 0; i < PALMAS_NUM_REGS ; i++) {
		pmic_ti913_platform.reg_data[i] = ardbeg_1735_reg_data[i];
		pmic_ti913_platform.reg_init[i] = ardbeg_1735_reg_init[i];
	}

	/* Set vdd_gpu init uV to 1V */
	reg_idata_ti913_smps123.constraints.init_uV = 900000;

	tegra_get_board_info(&board_info);
	if ((board_info.board_id == BOARD_E1792) ||
	    (board_info.board_id == BOARD_E1971) ||
	    (board_info.board_id == BOARD_E1973)) {
		/*Default DDR voltage is 1.35V but lpddr3 supports 1.2V*/
		reg_idata_ti913_smps7.constraints.min_uV = 1200000;
		reg_idata_ti913_smps7.constraints.max_uV = 1200000;
	}

	i2c_register_board_info(4, palma_ti913_device,
			ARRAY_SIZE(palma_ti913_device));
	return 0;
}

static struct pca953x_platform_data tca6408_pdata = {
	.gpio_base = PMU_TCA6416_GPIO_BASE,
	.irq_base = -1,
};

static const struct i2c_board_info tca6408_expander[] = {
	{
		I2C_BOARD_INFO("tca6408", 0x20),
		.platform_data = &tca6408_pdata,
	},
};

static u32 tegra_chip_id;
static struct tegra_suspend_platform_data yellowstone_suspend_data = {
	.cpu_timer      = 500,
	.cpu_off_timer  = 300,
	.suspend_mode   = TEGRA_SUSPEND_LP0,
	.core_timer     = 0x157e,
	.core_off_timer = 10,
	.corereq_high   = true,
	.sysclkreq_high = true,
	.cpu_lp2_min_residency = 1000,
	.min_residency_vmin_fmin = 1000,
	.min_residency_ncpu_fast = 8000,
	.min_residency_ncpu_slow = 5000,
	.min_residency_mclk_stop = 5000,
	.min_residency_crail = 20000,
};

/* Working theory. 5V -> 12V converter operating at 92% efficiency.
** According to TI WEBENCH, TPS55330 boost 5V in -> 12V out.
** Assuming 90% efficency for the boost converter
** Assuming 90% efficency for the charger converting 12V to 8700
** (2 batteries rated at 4.35v each = 8.7volts)
** Empirical observations.
** iin > ichg or the charger won't limit system current properly NOT
** iin >= ~512ma as according to bs24770.pdf page 21 wrt Setting Input Current.
** When programming over I2C, current steps are by 64ma so we & with ~0x3F
*/
static struct bq2477x_charge_zone yellowstone_bq2477x_charge_zones[3] = {
	/* Cold temperature charging. */
	{
		.min_temp = -150,	/* -15.0 C */
		.max_temp = 20,		/* 2.0 C */
		.charge_voltage = 0,	/* 0V */
		.charge_current = 0,	/* 0mA */
	},
	/* Nominal temperature charging. */
	{
		.min_temp = 20,		/* 2.0 C */
		.max_temp = 450,	/* 45.0 C */
		.charge_voltage = 8704, /* 8.704V */
		.charge_current = 2240, /* 2240mA */
	},
	/* Overtemp shutdown. */
	{
		.min_temp = 450,	/* 45.0 C */
		.max_temp = 999,	/* 99.9 C */
		.charge_voltage = 0,	/* 0V */
		.charge_current = 0,	/* 0mA */
	}
};

struct bq2477x_platform_data yellowstone_bq2477x_pdata = {
	.dac_v			= 8704,
	.dac_minsv		= 6144,
	.extcon_dock_name	= "power_bq2477x_extcon",
	.max_charge_ua		= 2440000,
	.dock_max_ua		= 2000000,
	.wdt_refresh_timeout	= 40,
	.disable_vbus_12v_boost_gpio = TEGRA_GPIO_PBB7,	/* 12v boost disable */
	.dock_12v_gpio = TEGRA_GPIO_PS0,
	.acok_gpio = TEGRA_GPIO_PJ0,
	.charge_table = yellowstone_bq2477x_charge_zones,
};

static struct platform_device yellowstone_bq2477x_extcon = {
	.name	= "power_bq2477x_extcon",
	.id	= -1,
	.dev	= {
		.platform_data = &yellowstone_bq2477x_pdata,
	},
};
static struct i2c_board_info __initdata bq2477x_boardinfo[] = {
	{
		I2C_BOARD_INFO("bq2477x", 0x6A),
		.platform_data	= &yellowstone_bq2477x_pdata,
	},
};

static struct power_supply_extcon_plat_data extcon_pdata = {
	.extcon_name = "tegra-udc",
};

static struct platform_device power_supply_extcon_device = {
	.name	= "power-supply-extcon",
	.id	= -1,
	.dev	= {
		.platform_data = &extcon_pdata,
	},
};

/************************ ARDBEG CL-DVFS DATA *********************/
#define E1735_CPU_VDD_MAP_SIZE		33
#define E1735_CPU_VDD_MIN_UV		752000
#define E1735_CPU_VDD_STEP_UV		16000
#define E1735_CPU_VDD_STEP_US		80
#define E1735_CPU_VDD_BOOT_UV		1248000
#define E1735_CPU_VDD_IDLE_MA		5000
#define YELLOWSTONE_DEFAULT_CVB_ALIGNMENT	10000

#ifdef CONFIG_ARCH_TEGRA_HAS_CL_DVFS
/* E1735 board parameters for cpu dfll */
static struct tegra_cl_dvfs_cfg_param e1735_cl_dvfs_param = {
	.sample_rate = 50000,

	.force_mode = TEGRA_CL_DVFS_FORCE_FIXED,
	.cf = 10,
	.ci = 0,
	.cg = 2,

	.droop_cut_value = 0xF,
	.droop_restore_ramp = 0x0,
	.scale_out_ramp = 0x0,
};

/* E1735 dfll bypass device for legacy dvfs control */
static struct regulator_consumer_supply e1735_dfll_bypass_consumers[] = {
	REGULATOR_SUPPLY("vdd_cpu", NULL),
};
DFLL_BYPASS(e1735,
	    E1735_CPU_VDD_MIN_UV, E1735_CPU_VDD_STEP_UV, E1735_CPU_VDD_BOOT_UV,
	    E1735_CPU_VDD_MAP_SIZE, E1735_CPU_VDD_STEP_US, TEGRA_GPIO_PX2);

static struct tegra_cl_dvfs_platform_data e1735_cl_dvfs_data = {
	.dfll_clk_name = "dfll_cpu",
	.pmu_if = TEGRA_CL_DVFS_PMU_PWM,
	.u.pmu_pwm = {
		.pwm_rate = 12750000,
		.min_uV = E1735_CPU_VDD_MIN_UV,
		.step_uV = E1735_CPU_VDD_STEP_UV,
		.pwm_pingroup = TEGRA_PINGROUP_DVFS_PWM,
		.out_gpio = TEGRA_GPIO_PS5,
		.out_enable_high = false,
#ifdef CONFIG_REGULATOR_TEGRA_DFLL_BYPASS
		.dfll_bypass_dev = &e1735_dfll_bypass_dev,
#endif
	},

	.cfg_param = &e1735_cl_dvfs_param,
};

static void e1735_suspend_dfll_bypass(void)
{
	__gpio_set_value(TEGRA_GPIO_PS5, 1); /* tristate external PWM buffer */
}

static void e1735_resume_dfll_bypass(void)
{
	__gpio_set_value(TEGRA_GPIO_PS5, 0); /* enable PWM buffer operations */
}

static void e1767_suspend_dfll_bypass(void)
{
	tegra_pinmux_set_tristate(TEGRA_PINGROUP_DVFS_PWM, TEGRA_TRI_TRISTATE);
}

static void e1767_resume_dfll_bypass(void)
{
	 tegra_pinmux_set_tristate(TEGRA_PINGROUP_DVFS_PWM, TEGRA_TRI_NORMAL);
}

static void __init yellowstone_tweak_E1767_dt(void)
{
	struct device_node *dn = NULL;
	struct property *pp = NULL;

	/*
	 *  Update E1735 DT for E1767 prototype. To be removed when
	 *  E1767 is productized with its own DT.
	 */
	dn = of_find_node_with_property(dn, "pwm-1wire-buffer");
	if (dn) {
		pp = of_find_property(dn, "pwm-1wire-buffer", NULL);
		if (pp)
			pp->name = "pwm-1wire-direct";
		of_node_put(dn);
	}
	if (!dn || !pp)
		WARN(1, "Failed update DT for PMU E1767 prototype\n");
}

static int __init yellowstone_cl_dvfs_init(struct board_info *pmu_board_info)
{
	u16 pmu_board_id = pmu_board_info->board_id;
	struct tegra_cl_dvfs_platform_data *data = NULL;

	if (pmu_board_id == BOARD_E1735) {
		bool e1767 = pmu_board_info->sku == E1735_EMULATE_E1767_SKU;
		struct device_node *dn = of_find_compatible_node(
			NULL, NULL, "nvidia,tegra124-dfll");
		/*
		 * Ardbeg platforms with E1735 PMIC module maybe used with
		 * different DT variants. Some of them include CL-DVFS data
		 * in DT, some - not. Check DT here, and continue with platform
		 * device registration only if DT DFLL node is not present.
		 */
		if (dn) {
			bool available = of_device_is_available(dn);
			of_node_put(dn);

			if (available) {
				if (e1767)
					yellowstone_tweak_E1767_dt();
				return 0;
			}
		}

		data = &e1735_cl_dvfs_data;

		data->u.pmu_pwm.pwm_bus = e1767 ?
			TEGRA_CL_DVFS_PWM_1WIRE_DIRECT :
			TEGRA_CL_DVFS_PWM_1WIRE_BUFFER;

		if (data->u.pmu_pwm.dfll_bypass_dev) {
			platform_device_register(
				data->u.pmu_pwm.dfll_bypass_dev);
		} else {
			(void)e1735_dfll_bypass_dev;
		}
	}

	if (data) {
		data->flags = TEGRA_CL_DVFS_DYN_OUTPUT_CFG;
		tegra_cl_dvfs_device.dev.platform_data = data;
		platform_device_register(&tegra_cl_dvfs_device);
	}
	return 0;
}
#else
static inline int yellowstone_cl_dvfs_init(struct board_info *pmu_board_info)
{ return 0; }
#endif

static void yellowstone_charger_init(void)
{
	int ret = 0;

	if (get_power_supply_type() == POWER_SUPPLY_TYPE_BATTERY) {
		ret = gpio_request(TEGRA_GPIO_PK5, "bq2477x-charger");
		if (ret < 0) {
			pr_err("%s: charger_enable TEGRA_GPIO_PK5 request failed\n",
				__func__);
		} else {
			ret = gpio_direction_output(TEGRA_GPIO_PK5, 1);
			if (ret < 0)
				pr_err("%s: TEGRA_GPIO_PK5 direction failed\n",
					__func__);
		}
		msleep(20);

		platform_device_register(&yellowstone_bq2477x_extcon);

		i2c_register_board_info(1, bq2477x_boardinfo,
			ARRAY_SIZE(bq2477x_boardinfo));
	}
}


int __init yellowstone_rail_alignment_init(void)
{
        struct board_info pmu_board_info;

        tegra_get_pmu_board_info(&pmu_board_info);

        if (pmu_board_info.board_id == BOARD_E1735)
                tegra12x_vdd_cpu_align(E1735_CPU_VDD_STEP_UV,
                                       E1735_CPU_VDD_MIN_UV);
        else
                tegra12x_vdd_cpu_align(YELLOWSTONE_DEFAULT_CVB_ALIGNMENT, 0);
        return 0;
}

int __init yellowstone_regulator_init(void)
{
        struct board_info pmu_board_info;

        tegra_get_pmu_board_info(&pmu_board_info);
        regulator_has_full_constraints();
	ardbeg_tps65913_regulator_init();
#ifdef CONFIG_REGULATOR_TEGRA_DFLL_BYPASS
        tegra_init_cpu_reg_mode_limits(
                E1735_CPU_VDD_IDLE_MA, REGULATOR_MODE_IDLE);
#endif
        platform_device_register(&power_supply_extcon_device);
        yellowstone_charger_init();

        yellowstone_cl_dvfs_init(&pmu_board_info);
        return 0;
}

int __init yellowstone_suspend_init(void)
{
	struct board_info pmu_board_info;

	tegra_get_pmu_board_info(&pmu_board_info);

	if (pmu_board_info.board_id == BOARD_E1735) {
		struct tegra_suspend_platform_data *data = &yellowstone_suspend_data;
		if (pmu_board_info.sku != E1735_EMULATE_E1767_SKU) {
			data->cpu_timer = 2000;
			data->crail_up_early = true;
#ifdef CONFIG_REGULATOR_TEGRA_DFLL_BYPASS
			data->suspend_dfll_bypass = e1735_suspend_dfll_bypass;
			data->resume_dfll_bypass = e1735_resume_dfll_bypass;
		} else {
			data->suspend_dfll_bypass = e1767_suspend_dfll_bypass;
			data->resume_dfll_bypass = e1767_resume_dfll_bypass;
#endif
		}
	}

	tegra_init_suspend(&yellowstone_suspend_data);
	return 0;
}

int __init yellowstone_edp_init(void)
{
	unsigned int regulator_mA;
	struct board_info pmu_board_info;

	tegra_get_pmu_board_info(&pmu_board_info);

	regulator_mA = get_maximum_cpu_current_supported();
	if (!regulator_mA) {
		if (pmu_board_info.board_id == BOARD_E1936)
			regulator_mA = 16800;
		else if (pmu_board_info.board_id == BOARD_PM374)
			regulator_mA = 32000;
		else
			regulator_mA = 14000;
	}

	pr_info("%s: CPU regulator %d mA\n", __func__, regulator_mA);
	tegra_init_cpu_edp_limits(regulator_mA);

	/* gpu maximum current */
	if (pmu_board_info.board_id == BOARD_E1936)
		regulator_mA = 11200;
	else if (pmu_board_info.board_id == BOARD_PM374)
		regulator_mA = 16000;
	else
		regulator_mA = 12000;

	pr_info("%s: GPU regulator %d mA\n", __func__, regulator_mA);
	tegra_init_gpu_edp_limits(regulator_mA);

	return 0;
}


static struct pid_thermal_gov_params soctherm_pid_params = {
	.max_err_temp = 9000,
	.max_err_gain = 1000,

	.gain_p = 1000,
	.gain_d = 0,

	.up_compensation = 20,
	.down_compensation = 20,
};

static struct thermal_zone_params soctherm_tzp = {
	.governor_name = "pid_thermal_gov",
	.governor_params = &soctherm_pid_params,
};

static struct tegra_tsensor_pmu_data tpdata_palmas = {
	.reset_tegra = 1,
	.pmu_16bit_ops = 0,
	.controller_type = 0,
	.pmu_i2c_addr = 0x58,
	.i2c_controller_id = 4,
	.poweroff_reg_addr = 0xa0,
	.poweroff_reg_data = 0x0,
};

static struct tegra_tsensor_pmu_data tpdata_as3722 = {
	.reset_tegra = 1,
	.pmu_16bit_ops = 0,
	.controller_type = 0,
	.pmu_i2c_addr = 0x40,
	.i2c_controller_id = 4,
	.poweroff_reg_addr = 0x36,
	.poweroff_reg_data = 0x2,
};

static struct soctherm_therm yellowstone_therm_pop[THERM_SIZE] = {
	[THERM_CPU] = {
		.zone_enable = true,
		.passive_delay = 1000,
		.hotspot_offset = 6000,
		.num_trips = 3,
		.trips = {
			{
				.cdev_type = "tegra-shutdown",
				.trip_temp = 97000,
				.trip_type = THERMAL_TRIP_CRITICAL,
				.upper = THERMAL_NO_LIMIT,
				.lower = THERMAL_NO_LIMIT,
			},
			{
				.cdev_type = "tegra-heavy",
				.trip_temp = 94000,
				.trip_type = THERMAL_TRIP_HOT,
				.upper = THERMAL_NO_LIMIT,
				.lower = THERMAL_NO_LIMIT,
			},
			{
				.cdev_type = "cpu-balanced",
				.trip_temp = 84000,
				.trip_type = THERMAL_TRIP_PASSIVE,
				.upper = THERMAL_NO_LIMIT,
				.lower = THERMAL_NO_LIMIT,
			},
		},
		.tzp = &soctherm_tzp,
	},
	[THERM_GPU] = {
		.zone_enable = true,
		.passive_delay = 1000,
		.hotspot_offset = 6000,
		.num_trips = 3,
		.trips = {
			{
				.cdev_type = "tegra-shutdown",
				.trip_temp = 93000,
				.trip_type = THERMAL_TRIP_CRITICAL,
				.upper = THERMAL_NO_LIMIT,
				.lower = THERMAL_NO_LIMIT,
			},
			{
				.cdev_type = "tegra-heavy",
				.trip_temp = 91000,
				.trip_type = THERMAL_TRIP_HOT,
				.upper = THERMAL_NO_LIMIT,
				.lower = THERMAL_NO_LIMIT,
			},
			{
				.cdev_type = "gpu-balanced",
				.trip_temp = 81000,
				.trip_type = THERMAL_TRIP_PASSIVE,
				.upper = THERMAL_NO_LIMIT,
				.lower = THERMAL_NO_LIMIT,
			},
		},
		.tzp = &soctherm_tzp,
	},
	[THERM_MEM] = {
		.zone_enable = true,
		.num_trips = 1,
		.trips = {
			{
				.cdev_type = "tegra-shutdown",
				.trip_temp = 93000, /* = GPU shut */
				.trip_type = THERMAL_TRIP_CRITICAL,
				.upper = THERMAL_NO_LIMIT,
				.lower = THERMAL_NO_LIMIT,
			},
		},
		.tzp = &soctherm_tzp,
	},
	[THERM_PLL] = {
		.zone_enable = true,
		.num_trips = 1,
		.trips = {
			{
				.cdev_type = "tegra-dram",
				.trip_temp = 78000,
				.trip_type = THERMAL_TRIP_ACTIVE,
				.upper = 1,
				.lower = 1,
			},
		},
		.tzp = &soctherm_tzp,
	},
};

/*
 * @PSKIP_CONFIG_NOTE: For T132, throttling config of PSKIP is no longer
 * done in soctherm registers. These settings are now done via registers in
 * denver:ccroc module which are at a different register offset. More
 * importantly, there are _only_ three levels of throttling: 'low',
 * 'medium' and 'heavy' and are selected via the 'throttling_depth' field
 * in the throttle->devs[] section of the soctherm config. Since the depth
 * specification is per device, it is necessary to manually make sure the
 * depths specified alongwith a given level are the same across all devs,
 * otherwise it will overwrite a previously set depth with a different
 * depth. We will refer to this comment at each relevant location in the
 * config sections below.
 */
static struct soctherm_platform_data yellowstone_soctherm_data = {
	.oc_irq_base = TEGRA_SOC_OC_IRQ_BASE,
	.num_oc_irqs = TEGRA_SOC_OC_NUM_IRQ,
	.therm = {
		[THERM_CPU] = {
			.zone_enable = true,
			.passive_delay = 1000,
			.hotspot_offset = 6000,
			.num_trips = 3,
			.trips = {
				{
					.cdev_type = "tegra-shutdown",
					.trip_temp = 101000,
					.trip_type = THERMAL_TRIP_CRITICAL,
					.upper = THERMAL_NO_LIMIT,
					.lower = THERMAL_NO_LIMIT,
				},
				{
					.cdev_type = "tegra-heavy",
					.trip_temp = 99000,
					.trip_type = THERMAL_TRIP_HOT,
					.upper = THERMAL_NO_LIMIT,
					.lower = THERMAL_NO_LIMIT,
				},
				{
					.cdev_type = "cpu-balanced",
					.trip_temp = 90000,
					.trip_type = THERMAL_TRIP_PASSIVE,
					.upper = THERMAL_NO_LIMIT,
					.lower = THERMAL_NO_LIMIT,
				},
			},
			.tzp = &soctherm_tzp,
		},
		[THERM_GPU] = {
			.zone_enable = true,
			.passive_delay = 1000,
			.hotspot_offset = 6000,
			.num_trips = 3,
			.trips = {
				{
					.cdev_type = "tegra-shutdown",
					.trip_temp = 101000,
					.trip_type = THERMAL_TRIP_CRITICAL,
					.upper = THERMAL_NO_LIMIT,
					.lower = THERMAL_NO_LIMIT,
				},
				{
					.cdev_type = "tegra-heavy",
					.trip_temp = 99000,
					.trip_type = THERMAL_TRIP_HOT,
					.upper = THERMAL_NO_LIMIT,
					.lower = THERMAL_NO_LIMIT,
				},
				{
					.cdev_type = "gpu-balanced",
					.trip_temp = 90000,
					.trip_type = THERMAL_TRIP_PASSIVE,
					.upper = THERMAL_NO_LIMIT,
					.lower = THERMAL_NO_LIMIT,
				},
			},
			.tzp = &soctherm_tzp,
		},
		[THERM_MEM] = {
			.zone_enable = true,
			.num_trips = 1,
			.trips = {
				{
					.cdev_type = "tegra-shutdown",
					.trip_temp = 101000, /* = GPU shut */
					.trip_type = THERMAL_TRIP_CRITICAL,
					.upper = THERMAL_NO_LIMIT,
					.lower = THERMAL_NO_LIMIT,
				},
			},
			.tzp = &soctherm_tzp,
		},
		[THERM_PLL] = {
			.zone_enable = true,
			.tzp = &soctherm_tzp,
		},
	},
	.throttle = {
		[THROTTLE_HEAVY] = {
			.priority = 100,
			.devs = {
				[THROTTLE_DEV_CPU] = {
					.enable = true,
					.depth = 80,
					/* see @PSKIP_CONFIG_NOTE */
					.throttling_depth = "heavy_throttling",
				},
				[THROTTLE_DEV_GPU] = {
					.enable = true,
					.throttling_depth = "heavy_throttling",
				},
			},
		},
	},
};

/* Only the diffs from yellowstone_soctherm_data structure */
static struct soctherm_platform_data t132ref_v1_soctherm_data = {
	.therm = {
		[THERM_CPU] = {
			.zone_enable = true,
			.passive_delay = 1000,
			.hotspot_offset = 10000,
		},
		[THERM_PLL] = {
			.zone_enable = true,
			.passive_delay = 1000,
			.num_trips = 3,
			.trips = {
				{
					.cdev_type = "tegra-shutdown",
					.trip_temp = 97000,
					.trip_type = THERMAL_TRIP_CRITICAL,
					.upper = THERMAL_NO_LIMIT,
					.lower = THERMAL_NO_LIMIT,
				},
				{
					.cdev_type = "tegra-heavy",
					.trip_temp = 94000,
					.trip_type = THERMAL_TRIP_HOT,
					.upper = THERMAL_NO_LIMIT,
					.lower = THERMAL_NO_LIMIT,
				},
				{
					.cdev_type = "cpu-balanced",
					.trip_temp = 84000,
					.trip_type = THERMAL_TRIP_PASSIVE,
					.upper = THERMAL_NO_LIMIT,
					.lower = THERMAL_NO_LIMIT,
				},
			},
			.tzp = &soctherm_tzp,
		},
	},
};

/* Only the diffs from yellowstone_soctherm_data structure */
static struct soctherm_platform_data t132ref_v2_soctherm_data = {
	.therm = {
		[THERM_CPU] = {
			.zone_enable = true,
			.passive_delay = 1000,
			.hotspot_offset = 10000,
			.num_trips = 3,
			.trips = {
				{
					.cdev_type = "tegra-shutdown",
					.trip_temp = 105000,
					.trip_type = THERMAL_TRIP_CRITICAL,
					.upper = THERMAL_NO_LIMIT,
					.lower = THERMAL_NO_LIMIT,
				},
				{
					.cdev_type = "tegra-heavy",
					.trip_temp = 102000,
					.trip_type = THERMAL_TRIP_HOT,
					.upper = THERMAL_NO_LIMIT,
					.lower = THERMAL_NO_LIMIT,
				},
				{
					.cdev_type = "cpu-balanced",
					.trip_temp = 92000,
					.trip_type = THERMAL_TRIP_PASSIVE,
					.upper = THERMAL_NO_LIMIT,
					.lower = THERMAL_NO_LIMIT,
				},
			},
			.tzp = &soctherm_tzp,
		},
		[THERM_GPU] = {
			.zone_enable = true,
			.passive_delay = 1000,
			.hotspot_offset = 5000,
			.num_trips = 3,
			.trips = {
				{
					.cdev_type = "tegra-shutdown",
					.trip_temp = 101000,
					.trip_type = THERMAL_TRIP_CRITICAL,
					.upper = THERMAL_NO_LIMIT,
					.lower = THERMAL_NO_LIMIT,
				},
				{
					.cdev_type = "tegra-heavy",
					.trip_temp = 99000,
					.trip_type = THERMAL_TRIP_HOT,
					.upper = THERMAL_NO_LIMIT,
					.lower = THERMAL_NO_LIMIT,
				},
				{
					.cdev_type = "gpu-balanced",
					.trip_temp = 89000,
					.trip_type = THERMAL_TRIP_PASSIVE,
					.upper = THERMAL_NO_LIMIT,
					.lower = THERMAL_NO_LIMIT,
				},
			},
			.tzp = &soctherm_tzp,
		},
	},
};

static struct soctherm_throttle battery_oc_throttle_t13x = {
	.throt_mode = BRIEF,
	.polarity = SOCTHERM_ACTIVE_LOW,
	.priority = 50,
	.intr = true,
	.alarm_cnt_threshold = 15,
	.alarm_filter = 5100000,
	.devs = {
		[THROTTLE_DEV_CPU] = {
			.enable = true,
			.depth = 50,
			/* see @PSKIP_CONFIG_NOTE */
			.throttling_depth = "low_throttling",
		},
		[THROTTLE_DEV_GPU] = {
			.enable = true,
			.throttling_depth = "medium_throttling",
		},
	},
};

static struct soctherm_throttle battery_oc_throttle_t12x = {
	.throt_mode = BRIEF,
	.polarity = SOCTHERM_ACTIVE_LOW,
	.priority = 50,
	.intr = true,
	.alarm_cnt_threshold = 15,
	.alarm_filter = 5100000,
	.devs = {
		[THROTTLE_DEV_CPU] = {
			.enable = true,
			.depth = 50,
		},
		[THROTTLE_DEV_GPU] = {
			.enable = true,
			.throttling_depth = "medium_throttling",
		},
	},
};

static struct soctherm_throttle voltmon_throttle_t13x = {
	.throt_mode = BRIEF,
	.polarity = SOCTHERM_ACTIVE_LOW,
	.priority = 50,
	.intr = true,
	.alarm_cnt_threshold = 100,
	.alarm_filter = 5100000,
	.devs = {
		[THROTTLE_DEV_CPU] = {
			.enable = true,
			/* throttle depth 75% with 3.76us ramp rate */
			.dividend = 63,
			.divisor = 255,
			.duration = 0,
			.step = 0,
			/* see @PSKIP_CONFIG_NOTE */
			.throttling_depth = "medium_throttling",
		},
		[THROTTLE_DEV_GPU] = {
			.enable = true,
			.throttling_depth = "medium_throttling",
		},
	},
};

static struct soctherm_throttle voltmon_throttle_t12x = {
	.throt_mode = BRIEF,
	.polarity = SOCTHERM_ACTIVE_LOW,
	.priority = 50,
	.intr = true,
	.alarm_cnt_threshold = 100,
	.alarm_filter = 5100000,
	.devs = {
		[THROTTLE_DEV_CPU] = {
			.enable = true,
			/* throttle depth 75% with 3.76us ramp rate */
			.dividend = 63,
			.divisor = 255,
			.duration = 0,
			.step = 0,
		},
		[THROTTLE_DEV_GPU] = {
			.enable = true,
			.throttling_depth = "medium_throttling",
		},
	},
};

struct soctherm_throttle baseband_throttle = {
	.throt_mode = BRIEF,
	.polarity = SOCTHERM_ACTIVE_HIGH,
	.priority = 50,
	.devs = {
		[THROTTLE_DEV_CPU] = {
			.enable = true,
			.depth = 50,
		},
		[THROTTLE_DEV_GPU] = {
			.enable = true,
			.throttling_depth = "medium_throttling",
		},
	},
};

int __init yellowstone_soctherm_init(void)
{
	const int t12x_edp_temp_margin = 7000,
		t13x_cpu_edp_temp_margin = 5000,
		t13x_gpu_edp_temp_margin = 6000;
	int cpu_edp_temp_margin, gpu_edp_temp_margin;
	int cp_rev, ft_rev;
	struct board_info pmu_board_info;
	struct board_info board_info;
	enum soctherm_therm_id therm_cpu = THERM_CPU;

	tegra_get_board_info(&board_info);
	tegra_chip_id = tegra_get_chip_id();

	if (board_info.board_id == BOARD_E1923 ||
			board_info.board_id == BOARD_E1922) {
		memcpy(yellowstone_soctherm_data.therm,
				yellowstone_therm_pop, sizeof(yellowstone_therm_pop));
	}

	cp_rev = tegra_fuse_calib_base_get_cp(NULL, NULL);
	ft_rev = tegra_fuse_calib_base_get_ft(NULL, NULL);

	/* Bowmore and P1761 are T132 platforms */
	if (board_info.board_id == BOARD_E1971 ||
			board_info.board_id == BOARD_P1761 ||
			board_info.board_id == BOARD_P1765 ||
			board_info.board_id == BOARD_E1991) {
		cpu_edp_temp_margin = t13x_cpu_edp_temp_margin;
		gpu_edp_temp_margin = t13x_gpu_edp_temp_margin;
		if (!cp_rev) {
			/* ATE rev is NEW: use v2 table */
			yellowstone_soctherm_data.therm[THERM_CPU] =
				t132ref_v2_soctherm_data.therm[THERM_CPU];
			yellowstone_soctherm_data.therm[THERM_GPU] =
				t132ref_v2_soctherm_data.therm[THERM_GPU];
		} else {
			/* ATE rev is Old or Mid: use PLLx sensor only */
			yellowstone_soctherm_data.therm[THERM_CPU] =
				t132ref_v1_soctherm_data.therm[THERM_CPU];
			yellowstone_soctherm_data.therm[THERM_PLL] =
				t132ref_v1_soctherm_data.therm[THERM_PLL];
			therm_cpu = THERM_PLL; /* override CPU with PLL zone */
		}
	} else {
		cpu_edp_temp_margin = t12x_edp_temp_margin;
		gpu_edp_temp_margin = t12x_edp_temp_margin;
	}

	/* do this only for supported CP,FT fuses */
	if ((cp_rev >= 0) && (ft_rev >= 0)) {
		tegra_platform_edp_init(
			yellowstone_soctherm_data.therm[therm_cpu].trips,
			&yellowstone_soctherm_data.therm[therm_cpu].num_trips,
			cpu_edp_temp_margin);
		tegra_platform_gpu_edp_init(
			yellowstone_soctherm_data.therm[THERM_GPU].trips,
			&yellowstone_soctherm_data.therm[THERM_GPU].num_trips,
			gpu_edp_temp_margin);
		tegra_add_cpu_vmax_trips(
			yellowstone_soctherm_data.therm[therm_cpu].trips,
			&yellowstone_soctherm_data.therm[therm_cpu].num_trips);
		tegra_add_tgpu_trips(
			yellowstone_soctherm_data.therm[THERM_GPU].trips,
			&yellowstone_soctherm_data.therm[THERM_GPU].num_trips);
		tegra_add_vc_trips(
			yellowstone_soctherm_data.therm[therm_cpu].trips,
			&yellowstone_soctherm_data.therm[therm_cpu].num_trips);
		tegra_add_core_vmax_trips(
			yellowstone_soctherm_data.therm[THERM_PLL].trips,
			&yellowstone_soctherm_data.therm[THERM_PLL].num_trips);
	}

	if (board_info.board_id == BOARD_P1761 ||
		board_info.board_id == BOARD_P1765 ||
		board_info.board_id == BOARD_E1784 ||
		board_info.board_id == BOARD_E1971 ||
		board_info.board_id == BOARD_E1991 ||
		board_info.board_id == BOARD_E1922) {
		tegra_add_cpu_vmin_trips(
			yellowstone_soctherm_data.therm[therm_cpu].trips,
			&yellowstone_soctherm_data.therm[therm_cpu].num_trips);
		tegra_add_gpu_vmin_trips(
			yellowstone_soctherm_data.therm[THERM_GPU].trips,
			&yellowstone_soctherm_data.therm[THERM_GPU].num_trips);
		tegra_add_core_vmin_trips(
			yellowstone_soctherm_data.therm[THERM_PLL].trips,
			&yellowstone_soctherm_data.therm[THERM_PLL].num_trips);
	}

	tegra_get_pmu_board_info(&pmu_board_info);

	if ((pmu_board_info.board_id == BOARD_E1733) ||
		(pmu_board_info.board_id == BOARD_E1734))
		yellowstone_soctherm_data.tshut_pmu_trip_data = &tpdata_as3722;
	else if (pmu_board_info.board_id == BOARD_E1735 ||
		 pmu_board_info.board_id == BOARD_E1736 ||
		 pmu_board_info.board_id == BOARD_E1769 ||
		 pmu_board_info.board_id == BOARD_P1761 ||
		 pmu_board_info.board_id == BOARD_P1765 ||
		 pmu_board_info.board_id == BOARD_E1936)
		yellowstone_soctherm_data.tshut_pmu_trip_data = &tpdata_palmas;

	else
		pr_warn("soctherm THERMTRIP not supported on PMU (BOARD_E%d)\n",
			pmu_board_info.board_id);

	/* Enable soc_therm OC throttling on selected platforms */
	switch (board_info.board_id) {
	case BOARD_E1971:
		memcpy(&yellowstone_soctherm_data.throttle[THROTTLE_OC4],
		       &battery_oc_throttle_t13x,
		       sizeof(battery_oc_throttle_t13x));
		break;
	case BOARD_P1761:
	case BOARD_E1936:
	case BOARD_P1765:
		if (tegra_chip_id == TEGRA_CHIPID_TEGRA13) {
			memcpy(&yellowstone_soctherm_data.throttle[THROTTLE_OC4],
				   &battery_oc_throttle_t13x,
				   sizeof(battery_oc_throttle_t13x));
			memcpy(&yellowstone_soctherm_data.throttle[THROTTLE_OC1],
				   &voltmon_throttle_t13x,
				   sizeof(voltmon_throttle_t13x));
		} else {
			memcpy(&yellowstone_soctherm_data.throttle[THROTTLE_OC4],
				   &battery_oc_throttle_t12x,
				   sizeof(battery_oc_throttle_t12x));
			memcpy(&yellowstone_soctherm_data.throttle[THROTTLE_OC1],
				   &voltmon_throttle_t12x,
				   sizeof(voltmon_throttle_t12x));
		}


		break;
	default:
		break;
	}

	/* enable baseband OC if Bruce modem is enabled */
	if (tegra_get_modem_id() == TEGRA_BB_BRUCE) {
		/* enable baseband OC unless board has voltage comparator */
		int board_has_vc;

		board_has_vc = (pmu_board_info.board_id == BOARD_P1761)
			&& (pmu_board_info.fab >= BOARD_FAB_A02);

		if (!board_has_vc)
			memcpy(&yellowstone_soctherm_data.throttle[THROTTLE_OC3],
			       &baseband_throttle,
			       sizeof(baseband_throttle));
	}

	return tegra11_soctherm_init(&yellowstone_soctherm_data);
}
