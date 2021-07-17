/*
 * (C) Copyright 2021 Duragon Systems Co., Ltd
 *
 * SPDX-License-Identifier:     GPL-2.0+
 */

#ifndef __MPU_V2_H
#define __MPU_V2_H

#include <configs/rk3399_common.h>

#ifndef CONFIG_SPL_BUILD
#undef CONFIG_BOOTCOMMAND
#define CONFIG_BOOTCOMMAND RKIMG_BOOTCOMMAND
#endif

#define CONFIG_MMC_SDHCI_SDMA
#define CONFIG_SYS_MMC_ENV_DEV 0

#define SDRAM_BANK_SIZE			(2UL << 30)
#define CONFIG_MISC_INIT_R
#define CONFIG_SERIAL_TAG
#define CONFIG_ENV_OVERWRITE
#define CONFIG_REVISION_TAG

#define CONFIG_BMP_16BPP
#define CONFIG_BMP_24BPP
#define CONFIG_BMP_32BPP

#define ROCKCHIP_DEVICE_SETTINGS \
		"stdout=serial,vidconsole\0" \
		"stderr=serial,vidconsole\0"

#ifndef CONFIG_SPL_BUILD

#ifdef ENV_MEM_LAYOUT_SETTINGS
#undef ENV_MEM_LAYOUT_SETTINGS
#endif	/* ENV_MEM_LAYOUT_SETTINGS */

#define ENV_MEM_LAYOUT_SETTINGS \
	"scriptaddr=0x00500000\0" \
	"pxefile_addr_r=0x00600000\0" \
	"fdt_addr_r=0x08300000\0" \
	"kernel_addr_r=0x00280000\0" \
	"ramdisk_addr_r=0x0a200000\0"

#ifdef CONFIG_EXTRA_ENV_SETTINGS
#undef CONFIG_EXTRA_ENV_SETTINGS
#endif	/* CONFIG_EXTRA_ENV_SETTINGS */

#define PARTS_DURAGON \
	"uuid_disk=${uuid_gpt_disk};" \
	"name=spl,start=32K,size=4000K,uuid=${uuid_gpt_loader1};" \
	"name=uboot,start=8MB,size=8MB,uuid=${uuid_gpt_loader2};" \
	"name=boot,start=16MB,size=112MB,bootable,uuid=${uuid_gpt_boot};" \
	"name=rootfs,start=128MB,size=-,uuid="ROOT_UUID

#define PARTS_DURAGON_EXT \
	"uuid_disk=${uuid_gpt_disk};" \
	"name=loader1,start=32K,size=3552K,uuid=${uuid_gpt_loader1};" \
	"name=vendor,start=3584K,size=256K,uuid=${uuid_gpt_vendor};" \
	"name=unused,start=3840K,size=192K,uuid=${uuid_gpt_unused};" \
	"name=reserved1,start=4034K,size=64K,uuid=${uuid_gpt_reserved1};" \
	"name=env,start=4064K,size=32K,uuid=${uuid_gpt_env};" \
	"name=reserved2,start=4096K,size=4MB,uuid=${uuid_gpt_reserved2};" \
	"name=loader2,start=8MB,size=8MB,uuid=${uuid_gpt_loader2};" \
	"name=boot,start=16MB,size=112MB,bootable,uuid=${uuid_gpt_boot};" \
	"name=rootfs,start=128MB,size=-,uuid="ROOT_UUID

#include <config_distro_bootcmd.h>
#define CONFIG_EXTRA_ENV_SETTINGS \
	ENV_MEM_LAYOUT_SETTINGS \
	"partitions=" PARTS_DURAGON \
	ROCKCHIP_DEVICE_SETTINGS \
	RKIMG_DET_BOOTDEV \
	BOOTENV

#endif	/* CONFIG_SPL_BUILD */

#endif	/* __MPU_V2_H */
