/*
 * Copyright (C) 2023 Trucrux Ltd.
 *
 * SPDX-License-Identifier: GPL-2.0+
 */
#include <common.h>
#include <command.h>
#include <dm.h>
#include <i2c.h>
#include <asm/io.h>
#include <cpu_func.h>
#include <u-boot/crc.h>
#include <asm/arch-imx9/ddr.h>

#include "imx9_eeprom.h"

static int trux_eeprom_get_dev(struct udevice **devp)
{
	int ret;
	struct udevice *bus;

	ret = uclass_get_device_by_name(UCLASS_I2C, TRUX_SOM_EEPROM_I2C_NAME, &bus);
	if (ret) {
		debug("%s: No EEPROM I2C bus '%s'\n", __func__, TRUX_SOM_EEPROM_I2C_NAME);
		return ret;
	}

	ret = dm_i2c_probe(bus, TRUX_SOM_EEPROM_I2C_ADDR, 0, devp);
	if (ret) {
		debug("%s: I2C EEPROM probe failed\n", __func__);
		return ret;
	}

	i2c_set_chip_offset_len(*devp, 1);
	i2c_set_chip_addr_offset_mask(*devp, 1);

	return 0;
}

int trux_eeprom_read_header(struct trux_eeprom *e)
{
	int ret;
	struct udevice *dev;

	ret = trux_eeprom_get_dev(&dev);
	if (ret) {
		debug("%s: Failed to detect I2C EEPROM\n", __func__);
		return ret;
	}

	/* Read EEPROM header to memory */
	ret = dm_i2c_read(dev, 0, (void *)e, sizeof(*e));
	if (ret) {
		debug("%s: EEPROM read failed, ret=%d\n", __func__, ret);
		return ret;
	}

	return 0;
}

int trux_eeprom_get_mac(struct trux_eeprom *ep, u8 *mac)
{
	flush_dcache_all();
	if (!trux_eeprom_is_valid(ep))
		return -1;

	memcpy(mac, ep->mac, sizeof(ep->mac));

	return 0;
}

int trux_eeprom_get_dram_size(struct trux_eeprom *ep, phys_size_t *size)
{
	/* No data in EEPROM - return default DRAM size */
	if (!trux_eeprom_is_valid(ep)) {
		*size = DEFAULT_SDRAM_SIZE;
		return 0;
	}

	*size = (ep->dramsize * 128UL) << 20;
	return 0;
}

#ifndef CONFIG_SPL_BUILD
void trux_eeprom_print_prod_info(struct trux_eeprom *ep)
{
	flush_dcache_all();

	if (!trux_eeprom_is_valid(ep))
		return;

#ifdef CONFIG_TARGET_IMX93_TRUX_SOM
	printf("\nPart number: VSM-MX93-%.*s\n", (int)sizeof(ep->partnum), ep->partnum);
#endif

	printf("Assembly: AS%.*s\n", (int)sizeof(ep->assembly), (char *)ep->assembly);

	printf("Production date: %.*s %.*s %.*s\n",
			4, /* YYYY */
			(char *)ep->date,
			3, /* MMM */
			((char *)ep->date) + 4,
			2, /* DD */
			((char *)ep->date) + 4 + 3);

	printf("Serial Number: %02x:%02x:%02x:%02x:%02x:%02x\n",
		ep->mac[0], ep->mac[1], ep->mac[2], ep->mac[3], ep->mac[4], ep->mac[5]);

	debug("EEPROM version: 0x%x\n", ep->version);
	debug("SOM features: 0x%x\n", ep->features);
	printf("SOM revision: 0x%x\n", ep->somrev);
	printf("DRAM PN: VIC-%04d\n", ep->ddr_vic);
	debug("DRAM size: %d GiB\n\n", (ep->dramsize * 128) / 1024);
}
#endif

int trux_carrier_eeprom_read(const char * bus_name, int addr, struct trux_carrier_eeprom *ep)
{
	int ret;
	struct udevice *bus;
	struct udevice *dev;

	ret = uclass_get_device_by_name(UCLASS_I2C, bus_name, &bus);
	if (ret) {
		debug("%s: No bus '%s'\n", __func__, bus_name);
		return ret;
	}

	ret = dm_i2c_probe(bus, addr, 0, &dev);
	if (ret) {
		debug("%s: Carrier EEPROM I2C probe failed\n", __func__);
		return ret;
	}

	/* Read EEPROM to memory */
	ret = dm_i2c_read(dev, 0, (void *)ep, sizeof(*ep));
	if (ret) {
		debug("%s: Carrier EEPROM read failed, ret=%d\n", __func__, ret);
		return ret;
	}

	return 0;
}

int trux_carrier_eeprom_is_valid(struct trux_carrier_eeprom *ep)
{
	u32 crc, crc_offset = offsetof(struct trux_carrier_eeprom, crc);

	if (htons(ep->magic) != TRUX_CARRIER_EEPROM_MAGIC) {
		debug("Invalid carrier EEPROM magic 0x%hx, expected 0x%hx\n",
			htons(ep->magic), TRUX_CARRIER_EEPROM_MAGIC);
		return 0;
	}

	if (ep->struct_ver < 1) {
		printf("Invalid carrier EEPROM version 0x%hx\n", ep->struct_ver);
		return 0;
	}

	if (ep->struct_ver == 1)
		return 1;

	/* Only EEPROM structure above version 1 has CRC field */
	crc = crc32(0, (void *)ep, crc_offset);

	if (crc != ep->crc) {
		printf("Carrier EEPROM CRC mismatch (%08x != %08x)\n",
			crc, be32_to_cpu(ep->crc));
		return 0;
	}

	return 1;
}

/* Returns carrier board revision string via 'rev' argument.
 * For legacy carrier board revisions the "legacy" string is returned.
 * For new carrier board revisions the actual carrier revision is returned.
 * Symphony-Board 1.4 and below are legacy, 1.4a and above are new.
 * DT8MCustomBoard 1.4 and below are legacy, 2.0 and above are new.
 */
void trux_carrier_eeprom_get_revision(struct trux_carrier_eeprom *ep, char *rev, size_t size)
{
	if (trux_carrier_eeprom_is_valid(ep))
		strncpy(rev, (const char *)ep->carrier_rev, size);
	else
		strncpy(rev, "legacy", size);
}
