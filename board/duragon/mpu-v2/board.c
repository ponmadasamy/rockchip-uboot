#define DEBUG
#undef CONFIG_LOGLEVEL
#define CONFIG_LOGLEVEL 8

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
#include <asm/io.h>
#include <asm/gpio.h>

#include "../common/duragon.h"

DECLARE_GLOBAL_DATA_PTR;

#define RK3399_CPUID_OFF  0x7
#define RK3399_CPUID_LEN  0x10

/*
 * ID info:
 *  ID : Volts : ADC value :   Bucket
 *  ==   =====   =========   ===========
 *   0 : 0.102V:        58 :    0 -   81
 *   1 : 0.211V:       120 :   82 -  150
 *   2 : 0.319V:       181 :  151 -  211	SOM-RK3399
 *   3 : 0.427V:       242 :  212 -  274	SOM-RK3399v2
 *   4 : 0.542V:       307 :  275 -  342
 *   5 : 0.666V:       378 :  343 -  411
 *   6 : 0.781V:       444 :  412 -  477
 *   7 : 0.900V:       511 :  478 -  545
 *   8 : 1.023V:       581 :  546 -  613
 *   9 : 1.137V:       646 :  614 -  675
 *  10 : 1.240V:       704 :  676 -  733
 *  11 : 1.343V:       763 :  734 -  795
 *  12 : 1.457V:       828 :  796 -  861
 *  13 : 1.576V:       895 :  862 -  925
 *  14 : 1.684V:       956 :  926 -  989
 *  15 : 1.800V:      1023 :  990 - 1023
 */
static const int id_readings[] = {
	 81, 150, 211, 274, 342, 411, 477, 545,
	613, 675, 733, 795, 861, 925, 989, 1023
};

static int cached_board_id = -1;

#define SARADC_BASE		0xFF100000
#define SARADC_DATA		(SARADC_BASE + 0)
#define SARADC_CTRL		(SARADC_BASE + 8)

static u32 get_saradc_value(int chn)
{
	int timeout = 0;
	u32 adc_value = 0;

	writel(0, SARADC_CTRL);
	udelay(2);

	writel(0x28 | chn, SARADC_CTRL);
	udelay(50);

	timeout = 0;
	do {
		if (readl(SARADC_CTRL) & 0x40) {
			adc_value = readl(SARADC_DATA) & 0x3FF;
			goto stop_adc;
		}

		udelay(10);
	} while (timeout++ < 100);

stop_adc:
	writel(0, SARADC_CTRL);

	return adc_value;
}

static uint32_t get_board_id(void)
{
	int i;
	int adc_reading;

	if (cached_board_id != -1)
		return cached_board_id;

	adc_reading = get_saradc_value(4); /* ADC_IN4 */
	for (i = 0; i < ARRAY_SIZE(id_readings); i++) {
		if (adc_reading <= id_readings[i]) {
			debug("ADC reading %d, ID %d\n", adc_reading, i);
			cached_board_id = i;
			return i;
		}
	}

	/* should die for impossible value */
	return 0;
}

int rk_board_init(void)
{
	struct udevice *pinctrl, *regulator;
	int ret;

	debug("%s\n", __func__);
	debug("%s: SOM ID: %d\n", __func__, get_board_id());
	/*
	 * The PWM does not have decicated interrupt number in dts and can
	 * not get periph_id by pinctrl framework, so let's init them here.
	 * The PWM2 and PWM3 are for pwm regulators.
	 */
	ret = uclass_get_device(UCLASS_PINCTRL, 0, &pinctrl);
	if (ret) {
		debug("%s: Cannot find pinctrl device, %d\n", __func__, ret);
		// goto host;
	}

	ret = pinctrl_request_noflags(pinctrl, PERIPH_ID_PWM2);
	if (ret) {
		debug("%s PWM2 pinctrl init fail!, %d\n", __func__, ret);
		// goto host;
	}

	ret = pinctrl_request_noflags(pinctrl, PERIPH_ID_PWM3);
	if (ret) {
		debug("%s PWM3 pinctrl init fail!, %d\n", __func__, ret);
		// goto host;
	}

// host:
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
	debug("%s\n", __func__);
	
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
#endif	/* #if defined(CONFIG_DISPLAY_CPUINFO) */


/*
 * Board revision list: <GPIO4_D1 | GPIO4_D0>
 *  0b00 - NanoPC-T4
 *  0b01 - NanoPi M4
 *
 * Extended by ADC_IN4
 * Group A:
 *  0x04 - NanoPi NEO4
 *  0x06 - SOC-RK3399
 *
 * Group B:
 *  0x21 - NanoPi M4 Ver2.0
 */
static int som_rev = -1;

void bd_hwrev_init(void)
{
#define GPIO4_BASE	0xff790000
	struct rockchip_gpio_regs *regs = (void *)GPIO4_BASE;

	if (som_rev >= 0)
		return;

	/* D1, D0: input mode */
	clrbits_le32(&regs->swport_ddr, (0x3 << 24));
	som_rev = (readl(&regs->ext_port) >> 24) & 0x3;

	if (som_rev == 0x3) {
		/* Revision group A: 0x04 ~ 0x13 */
		som_rev = 0x4 + get_board_id();

	} else if (som_rev == 0x1) {
		int idx = get_board_id();

		/* Revision group B: 0x21 ~ 0x2f */
		if (idx > 0) {
			som_rev = 0x20 + idx;
		}
	}
}

#ifdef CONFIG_REVISION_TAG
u32 get_board_rev(void)
{
	debug("%s\n", __func__);
	return som_rev;
}
#endif /* #ifdef CONFIG_REVISION_TAG */