#
# SPDX-License-Identifier:	GPL-2.0+
#

duragon += \
	mpu-v2.dtb

targets += $(duragon)

# Add any required device tree compiler flags here
DTC_FLAGS +=

PHONY += dtbs
dtbs: $(addprefix $(obj)/, $(duragon))
	@:

clean-files := *.dtb
