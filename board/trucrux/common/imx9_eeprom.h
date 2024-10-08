/*
 * Copyright (C) 2023 Trucrux Ltd.
 *
 * SPDX-License-Identifier: GPL-2.0+
 */

#ifndef _MX9_TRUX_EEPROM_H_
#define _MX9_TRUX_EEPROM_H_

#ifdef CONFIG_ARCH_IMX9
#include <asm/arch-imx9/ddr.h>
#endif

#define TRUX_SOM_EEPROM_MAGIC   0x4D58 /* == HEX("MX") */

#define TRUX_SOM_EEPROM_I2C_ADDR 0x52

/* Optional SOM features */
#define TRUX_EEPROM_F_WIFI              (1 << 0)
#define TRUX_EEPROM_F_ETH               (1 << 1)
#define TRUX_EEPROM_F_AUDIO             (1 << 2)

/* SOM storage types */
enum som_storage {
        SOM_STORAGE_EMMC,
        SOM_STORAGE_NAND,
        SOM_STORAGE_UNDEFINED,
};

/* Number of DRAM adjustment tables */
#define DRAM_TABLE_NUM 7

struct __attribute__((packed)) trux_eeprom
{
        u16 magic;                      /* 00-0x00 - magic number       */
        u8 partnum[8];                  /* 02-0x02 - part number        */
        u8 assembly[10];                /* 10-0x0a - assembly number    */
        u8 date[9];                     /* 20-0x14 - build date         */
        u8 mac[6];                      /* 29-0x1d - MAC address        */
        u8 somrev;                      /* 35-0x23 - SOM revision       */
        u8 version;                     /* 36-0x24 - EEPROM version     */
        u8 features;                    /* 37-0x25 - SOM features       */
        u8 dramsize;                    /* 38-0x26 - DRAM size          */
        u8 reserved[5];                 /* 39-0x27 - reserved           */
        u32 ddr_crc32;                  /* 44-0x2c - CRC32 of DDR DATA  */
        u16 ddr_vic;                    /* 48-0x30 - DDR VIC PN         */
        u16 off[DRAM_TABLE_NUM+1];      /* 50-0x32 - DRAM table offsets */
};

#define TRUX_EEPROM_DATA ((struct trux_eeprom *)TRUX_EEPROM_DRAM_START)

#define TRUX_CARRIER_EEPROM_MAGIC       0x5643 /* == HEX("VC") */

#define CARRIER_REV_LEN 16
struct __attribute__((packed)) trux_carrier_eeprom
{
        u16 magic;                          /* 00-0x00 - magic number           */
        u8 struct_ver;                      /* 01-0x01 - EEPROM structure version */
        u8 carrier_rev[CARRIER_REV_LEN];    /* 02-0x02 - carrier board revision */
        u32 crc;                            /* 10-0x0a - checksum               */
};

static inline int trux_eeprom_is_valid(struct trux_eeprom *ep)
{
        if (htons(ep->magic) != TRUX_SOM_EEPROM_MAGIC) {
                debug("Invalid EEPROM magic 0x%hx, expected 0x%hx\n",
                        htons(ep->magic), TRUX_SOM_EEPROM_MAGIC);
                return 0;
        }

        return 1;
}

int trux_eeprom_read_header(struct trux_eeprom *e);
int trux_eeprom_get_dram_size(struct trux_eeprom *e, phys_size_t *size);
int trux_eeprom_get_mac(struct trux_eeprom *e, u8 *mac);
void trux_eeprom_print_prod_info(struct trux_eeprom *e);

int trux_carrier_eeprom_read(const char * bus_name, int addr, struct trux_carrier_eeprom *ep);
int trux_carrier_eeprom_is_valid(struct trux_carrier_eeprom *ep);
void trux_carrier_eeprom_get_revision(struct trux_carrier_eeprom *ep, char *rev, size_t size);

#endif /* _MX9_TRUX_EEPROM_H_ */

