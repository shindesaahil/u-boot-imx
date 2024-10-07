#
# Copyright 2022 NXP
# Copyright 2023 Trucrux Ltd.
#
# SPDX-License-Identifier:      GPL-2.0+
#

obj-y += imx93_trucrux.o
obj-$(CONFIG_TARGET_IMX93_TRUCRUX) += ../common/imx9_eeprom.o

ifdef CONFIG_SPL_BUILD
obj-y += spl.o
obj-$(CONFIG_TARGET_IMX93_TRUCRUX) += lpddr4x_timing.o
else
obj-y += ../common/eth.o
obj-y += ../common/mmc.o
obj-$(CONFIG_EXTCON_PTN5150) += ../common/extcon-ptn5150.o
endif
