/*
 * Copyright 2018 NXP
 * Copyright 2018-2020 Trucrux Ltd.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <efi_loader.h>
#include <env.h>
#include <init.h>
#include <asm/io.h>
#include <asm/global_data.h>
#include <asm/mach-imx/iomux-v3.h>
#include <asm-generic/gpio.h>
#include <asm/arch/imx8mm_pins.h>
#include <asm/arch/sys_proto.h>
#include <asm/mach-imx/gpio.h>
#include <asm/mach-imx/mxc_i2c.h>
#include <asm/arch/clock.h>
#include <usb.h>
#include <dm.h>
#include <linux/arm-smccc.h>

#include "../common/extcon-ptn5150.h"
#include "../common/imx8_eeprom.h"
#include "imx8mm_trux_dart.h"

DECLARE_GLOBAL_DATA_PTR;

extern int trux_setup_mac(struct trux_eeprom *eeprom);

#define GPIO_PAD_CTRL	(PAD_CTL_DSE6 | PAD_CTL_FSEL1 | PAD_CTL_PUE | PAD_CTL_PE)

#ifdef CONFIG_SPL_BUILD
#define ID_GPIO 	IMX_GPIO_NR(2, 11)

static iomux_v3_cfg_t const id_pads[] = {
	IMX8MM_PAD_SD1_STROBE_GPIO2_IO11 | MUX_PAD_CTRL(GPIO_PAD_CTRL),
};

int get_board_id(void)
{
	static int board_id = UNKNOWN_BOARD;

	if (board_id != UNKNOWN_BOARD)
		return board_id;

	imx_iomux_v3_setup_multiple_pads(id_pads, ARRAY_SIZE(id_pads));
	gpio_request(ID_GPIO, "board_id");
	gpio_direction_input(ID_GPIO);

	board_id = gpio_get_value(ID_GPIO) ? TRUX_MX8M_MINI : TRUX_MX8M_MINI;

	return board_id;
}
#else
int get_board_id(void)
{
	static int board_id = UNKNOWN_BOARD;

	if (board_id != UNKNOWN_BOARD)
		return board_id;
		
	if (of_machine_is_compatible("trucrux,imx8mm-trux-q01"))
		board_id = TRUX_MX8M_MINI;
	else
		board_id = UNKNOWN_BOARD;

	return board_id;
}
#endif

int trux_get_som_rev(struct trux_eeprom *ep)
{
	switch (ep->somrev) {
	case 0:
		return SOM_REV_10;
	case 1:
		return SOM_REV_11;
	case 2:
		return SOM_REV_12;
	case 3:
		return SOM_REV_13;
	default:
		return UNKNOWN_REV;
	}
}

#define UART_PAD_CTRL	(PAD_CTL_DSE6 | PAD_CTL_FSEL1)
#define WDOG_PAD_CTRL	(PAD_CTL_DSE6 | PAD_CTL_ODE | PAD_CTL_PUE | PAD_CTL_PE)

static iomux_v3_cfg_t const uart1_pads[] = {
	IMX8MM_PAD_UART1_RXD_UART1_RX | MUX_PAD_CTRL(UART_PAD_CTRL),
	IMX8MM_PAD_UART1_TXD_UART1_TX | MUX_PAD_CTRL(UART_PAD_CTRL),
};

static iomux_v3_cfg_t const uart4_pads[] = {
	IMX8MM_PAD_UART4_RXD_UART4_RX | MUX_PAD_CTRL(UART_PAD_CTRL),
	IMX8MM_PAD_UART4_TXD_UART4_TX | MUX_PAD_CTRL(UART_PAD_CTRL),
};

static iomux_v3_cfg_t const wdog_pads[] = {
	IMX8MM_PAD_GPIO1_IO02_WDOG1_WDOG_B  | MUX_PAD_CTRL(WDOG_PAD_CTRL),
};

extern struct mxc_uart *mxc_base;

#if CONFIG_IS_ENABLED(EFI_HAVE_CAPSULE_SUPPORT)
struct efi_fw_image fw_images[] = {
	{
		.image_type_id = IMX_BOOT_IMAGE_GUID,
		.fw_name = u"IMX8MM-TRUCRUX-RAW",
		.image_index = 1,
	},
};

struct efi_capsule_update_info update_info = {
	.dfu_string = "mmc 2=1 raw 0x40 0x1000",
	.images = fw_images,
};

u8 num_image_type_guids = ARRAY_SIZE(fw_images);
#endif /* EFI_HAVE_CAPSULE_SUPPORT */

int board_early_init_f(void)
{
	int id;
	struct wdog_regs *wdog = (struct wdog_regs *)WDOG1_BASE_ADDR;

	imx_iomux_v3_setup_multiple_pads(wdog_pads, ARRAY_SIZE(wdog_pads));

	set_wdog_reset(wdog);

	id = get_board_id();

	if (id == TRUX_MX8M_MINI) {
		init_uart_clk(0);
		imx_iomux_v3_setup_multiple_pads(uart1_pads, ARRAY_SIZE(uart1_pads));
	}

	return 0;
}

#ifdef CONFIG_FEC_MXC
static int setup_fec(void)
{
	struct iomuxc_gpr_base_regs *gpr =
		(struct iomuxc_gpr_base_regs *)IOMUXC_GPR_BASE_ADDR;

	/* Use 125M anatop REF_CLK1 for ENET1, not from external */
	clrsetbits_le32(&gpr->gpr[1], 0x2000, 0);

	return 0;
}
#endif

#ifdef CONFIG_CI_UDC

#ifdef CONFIG_EXTCON_PTN5150
static struct extcon_ptn5150 usb_ptn5150;
#endif

int board_usb_init(int index, enum usb_init_type init)
{
	imx8m_usb_power(index, true);

#if (!defined(CONFIG_SPL_BUILD) && defined(CONFIG_EXTCON_PTN5150))
	if (index == 0) {
		/* Verify port is in proper mode */
		int phy_mode = extcon_ptn5150_phy_mode(&usb_ptn5150);

		//Only verify phy_mode if ptn5150 is initialized
		if (phy_mode >= 0 && phy_mode != init)
			return -ENODEV;
	}
#endif

	return 0;
}

int board_usb_cleanup(int index, enum usb_init_type init)
{
	imx8m_usb_power(index, false);

	return 0;
}

#ifdef CONFIG_EXTCON_PTN5150
int board_ehci_usb_phy_mode(struct udevice *dev)
{
	int usb_phy_mode = extcon_ptn5150_phy_mode(&usb_ptn5150);

	/* Default to host mode if not connected */
	if (usb_phy_mode < 0)
		usb_phy_mode = USB_INIT_HOST;

	return usb_phy_mode;
}
#endif
#endif

int board_init(void)
{
#ifdef CONFIG_FEC_MXC
	setup_fec();
#endif

	return 0;
}

#define SDRAM_SIZE_STR_LEN 5
int board_late_init(void)
{
	int som_rev;
	char sdram_size_str[SDRAM_SIZE_STR_LEN];
	int id = get_board_id();
	struct trux_eeprom *ep = TRUX_EEPROM_DATA;
	struct trux_carrier_eeprom carrier_eeprom;
	char carrier_rev[CARRIER_REV_LEN] = {0};

#ifdef CONFIG_EXTCON_PTN5150
	extcon_ptn5150_setup(&usb_ptn5150);
#endif

#ifdef CONFIG_FEC_MXC
	trux_setup_mac(ep);
#endif
	trux_eeprom_print_prod_info(ep);

	som_rev = trux_get_som_rev(ep);

	snprintf(sdram_size_str, SDRAM_SIZE_STR_LEN, "%d", (int) (gd->ram_size / 1024 / 1024));
	env_set("sdram_size", sdram_size_str);

	if (id == TRUX_MX8M_MINI) {
		env_set("board_name", "TRUX-MX8M-MINI");

		trux_carrier_eeprom_read(CARRIER_EEPROM_BUS_TRUX, CARRIER_EEPROM_ADDR, &carrier_eeprom);
		trux_carrier_eeprom_get_revision(&carrier_eeprom, carrier_rev, sizeof(carrier_rev));
		env_set("carrier_rev", carrier_rev);
	}

#ifdef CONFIG_ENV_IS_IN_MMC
	board_late_mmc_env_init();
#endif

	return 0;
}

#ifdef CONFIG_FSL_FASTBOOT
#ifdef CONFIG_ANDROID_RECOVERY

#define BACK_KEY IMX_GPIO_NR(4, 6)
#define BACK_PAD_CTRL	(PAD_CTL_DSE6 | PAD_CTL_HYS | PAD_CTL_PUE | PAD_CTL_PE)

static iomux_v3_cfg_t const back_pads[] = {
	IMX8MM_PAD_SAI1_RXD4_GPIO4_IO6 | MUX_PAD_CTRL(BACK_PAD_CTRL),
};

int is_recovery_key_pressing(void)
{
	return 0;
}
#endif /*CONFIG_ANDROID_RECOVERY*/
#endif /*CONFIG_FSL_FASTBOOT*/
