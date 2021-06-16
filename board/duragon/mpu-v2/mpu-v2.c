/*
 * (C) Copyright 2016 Rockchip Electronics Co., Ltd
 *
 * SPDX-License-Identifier:     GPL-2.0+
 */

#include <common.h>
#include <dm.h>
#include <misc.h>
#include <ram.h>
#include <dm/pinctrl.h>
#include <dm/uclass-internal.h>
#include <asm/setup.h>
#include <asm/arch/periph.h>
#include <power/regulator.h>
#include <u-boot/sha256.h>
#include <usb.h>
#include <dwc3-uboot.h>
#include <spl.h>
#include "../common/duragon.h"

DECLARE_GLOBAL_DATA_PTR;

#define RK3399_CPUID_OFF  0x7
#define RK3399_CPUID_LEN  0x10

int rk_board_init(void)
{
	struct udevice *pinctrl, *regulator;
	int ret;

	debug("%s: Initialization\n", __func__);
	/*
	 * The PWM does not have decicated interrupt number in dts and can
	 * not get periph_id by pinctrl framework, so let's init them here.
	 * The PWM2 and PWM3 are for pwm regulators.
	 */
	ret = uclass_get_device(UCLASS_PINCTRL, 0, &pinctrl);
	if (ret) {
		debug("%s: Cannot find pinctrl device, %d\n", __func__, ret);
		goto host;
	}

	ret = pinctrl_request_noflags(pinctrl, PERIPH_ID_PWM2);
	if (ret) {
		debug("%s PWM2 pinctrl init fail!, %d\n", __func__, ret);
		goto host;
	}

	ret = pinctrl_request_noflags(pinctrl, PERIPH_ID_PWM3);
	if (ret) {
		debug("%s PWM3 pinctrl init fail!, %d\n", __func__, ret);
		goto host;
	}

host:
	/* Type C USB host power control */
	ret = regulator_get_by_platname("vcc5v0_host", &regulator);
	if (ret) {
		debug("%s vcc5v0_host init fail!, %d\n", __func__, ret);
		goto ext;
	}

	ret = regulator_set_enable(regulator, true);
	if (ret) {
		debug("%s vcc5v0-host-en set fail!, %d\n", __func__, ret);
		goto ext;
	}
	debug("%s: USB Host Power Enabled\n", __func__);
ext:
	/* External Power Control */
	ret = regulator_get_by_platname("vcc12v0_ext", &regulator);
	if (ret) {
		debug("%s vcc12v0_ext init fail!, %d\n", __func__, ret);
		goto out;
	}

	ret = regulator_set_enable(regulator, true);
	if (ret) {
		debug("%s vcc12v0_ext set fail!\n", __func__);
		goto out;
	}
	debug("%s: External Power Enabled\n", __func__);
	/* BDC Power Control */
	ret = regulator_get_by_platname("vcc_bdc", &regulator);
	if (ret) {
		debug("%s vcc_bdc init fail! ret %d\n", __func__, ret);
		goto out;
	}

	ret = regulator_set_enable(regulator, true);
	if (ret) {
		debug("%s vcc_bdc set fail!\n", __func__);
		goto out;
	}
	debug("%s: BDC Power Enabled\n", __func__);
out:
	return 0;
}


int rk_board_late_init(void)
{
	printf("rk_board_late_init\n");
	
	setup_fpdlink_iii();
	return 0;
}


static void setup_macaddr(void)
{
#if CONFIG_IS_ENABLED(CMD_NET)
	int ret;
	const char *cpuid = env_get("cpuid#");
	u8 hash[SHA256_SUM_LEN];
	int size = sizeof(hash);
	u8 mac_addr[6];

	/* Only generate a MAC address, if none is set in the environment */
	if (env_get("ethaddr"))
		return;

	if (!cpuid) {
		debug("%s: could not retrieve 'cpuid#'\n", __func__);
		return;
	}

	ret = hash_block("sha256", (void *)cpuid, strlen(cpuid), hash, &size);
	if (ret) {
		debug("%s: failed to calculate SHA256\n", __func__);
		return;
	}

	/* Copy 6 bytes of the hash to base the MAC address on */
	memcpy(mac_addr, hash, 6);

	/* Make this a valid MAC address and set it */
	mac_addr[0] &= 0xfe;  /* clear multicast bit */
	mac_addr[0] |= 0x02;  /* set local assignment bit (IEEE802) */
	eth_env_set_enetaddr("ethaddr", mac_addr);
#endif

	return;
}

static void setup_serial(void)
{
#if CONFIG_IS_ENABLED(ROCKCHIP_EFUSE)
	struct udevice *dev;
	int ret, i;
	u8 cpuid[RK3399_CPUID_LEN];
	u8 low[RK3399_CPUID_LEN/2], high[RK3399_CPUID_LEN/2];
	char cpuid_str[RK3399_CPUID_LEN * 2 + 1];
	u64 serialno;
	char serialno_str[16];

	/* retrieve the device */
	ret = uclass_get_device_by_driver(UCLASS_MISC,
					  DM_GET_DRIVER(rockchip_efuse), &dev);
	if (ret) {
		debug("%s: could not find efuse device\n", __func__);
		return;
	}

	/* read the cpu_id range from the efuses */
	ret = misc_read(dev, RK3399_CPUID_OFF, &cpuid, sizeof(cpuid));
	if (ret) {
		debug("%s: reading cpuid from the efuses failed\n",
		      __func__);
		return;
	}

	memset(cpuid_str, 0, sizeof(cpuid_str));
	for (i = 0; i < 16; i++)
		sprintf(&cpuid_str[i * 2], "%02x", cpuid[i]);

	/*
	 * Mix the cpuid bytes using the same rules as in
	 *   ${linux}/drivers/soc/rockchip/rockchip-cpuinfo.c
	 */
	for (i = 0; i < 8; i++) {
		low[i] = cpuid[1 + (i << 1)];
		high[i] = cpuid[i << 1];
	}

	serialno = crc32_no_comp(0, low, 8);
	serialno |= (u64)crc32_no_comp(serialno, high, 8) << 32;
	snprintf(serialno_str, sizeof(serialno_str), "%llx", serialno);

	env_set("cpuid#", cpuid_str);
	env_set("serial#", serialno_str);
#endif

	return;
}

int misc_init_r(void)
{
	setup_serial();
	setup_macaddr();
	board_temperature();
	// setup_fpdlink_iii();

	return 0;
}

#ifdef CONFIG_SERIAL_TAG
void get_board_serial(struct tag_serialnr *serialnr)
{
	char *serial_string;
	u64 serial = 0;

	serial_string = env_get("serial#");

	if (serial_string)
		serial = simple_strtoull(serial_string, NULL, 16);

	serialnr->high = (u32)(serial >> 32);
	serialnr->low = (u32)(serial & 0xffffffff);
}
#endif

#ifdef CONFIG_USB_DWC3
static struct dwc3_device dwc3_device_data = {
	.maximum_speed = USB_SPEED_HIGH,
	.base = 0xfe800000,
	.dr_mode = USB_DR_MODE_PERIPHERAL,
	.index = 0,
	.dis_u2_susphy_quirk = 1,
	.usb2_phyif_utmi_width = 16,
};

int usb_gadget_handle_interrupts(void)
{
	dwc3_uboot_handle_interrupt(0);
	return 0;
}

int board_usb_init(int index, enum usb_init_type init)
{
	return dwc3_uboot_init(&dwc3_device_data);
}
#endif

#if defined(CONFIG_DISPLAY_CPUINFO)
int print_cpuinfo(void)
{

	debug("CPU: RK3399\n");
	return 0;
}
#endif
