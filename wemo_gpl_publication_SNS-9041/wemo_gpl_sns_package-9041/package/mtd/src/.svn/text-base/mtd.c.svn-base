/*
 * mtd - simple memory technology device manipulation tool
 *
 * Copyright (C) 2005      Waldemar Brodkorb <wbx@dass-it.de>,
 * Copyright (C) 2005-2009 Felix Fietkau <nbd@openwrt.org>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License v2
 * as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 *
 *
 * The code is based on the linux-mtd examples.
 */

#include <limits.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <signal.h>
#include <sys/ioctl.h>
#include <sys/syscall.h>
#include <fcntl.h>
#include <errno.h>
#include <error.h>
#include <time.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/param.h>
#include <sys/mount.h>
#include <sys/stat.h>
#include <sys/reboot.h>
#include <linux/reboot.h>
#include "mtd-api.h"
#include "fis.h"
#include "mtd.h"
#include "crc32.h"

size_t gOffset = 0;

static char *dataBuf = NULL;

#if defined (PRODUCT_WeMo_InsightCR)
char *factoryItem[] = {
    "wl_mac",
    "ssid",
    "model",
    "serial",
    "country",
    "fw_ver",
    "hw_ver",
    "zgb_mac",
    "csl_off",
    NULL
};
#else
char *factoryItem[] = {
    "serial",
    "region",
    "country",
    "model",
    "wl_mac",
    NULL
};
#endif

const unsigned long crc_table[256] = {
  0x00000000L, 0x77073096L, 0xee0e612cL, 0x990951baL, 0x076dc419L,
  0x706af48fL, 0xe963a535L, 0x9e6495a3L, 0x0edb8832L, 0x79dcb8a4L,
  0xe0d5e91eL, 0x97d2d988L, 0x09b64c2bL, 0x7eb17cbdL, 0xe7b82d07L,
  0x90bf1d91L, 0x1db71064L, 0x6ab020f2L, 0xf3b97148L, 0x84be41deL,
  0x1adad47dL, 0x6ddde4ebL, 0xf4d4b551L, 0x83d385c7L, 0x136c9856L,
  0x646ba8c0L, 0xfd62f97aL, 0x8a65c9ecL, 0x14015c4fL, 0x63066cd9L,
  0xfa0f3d63L, 0x8d080df5L, 0x3b6e20c8L, 0x4c69105eL, 0xd56041e4L,
  0xa2677172L, 0x3c03e4d1L, 0x4b04d447L, 0xd20d85fdL, 0xa50ab56bL,
  0x35b5a8faL, 0x42b2986cL, 0xdbbbc9d6L, 0xacbcf940L, 0x32d86ce3L,
  0x45df5c75L, 0xdcd60dcfL, 0xabd13d59L, 0x26d930acL, 0x51de003aL,
  0xc8d75180L, 0xbfd06116L, 0x21b4f4b5L, 0x56b3c423L, 0xcfba9599L,
  0xb8bda50fL, 0x2802b89eL, 0x5f058808L, 0xc60cd9b2L, 0xb10be924L,
  0x2f6f7c87L, 0x58684c11L, 0xc1611dabL, 0xb6662d3dL, 0x76dc4190L,
  0x01db7106L, 0x98d220bcL, 0xefd5102aL, 0x71b18589L, 0x06b6b51fL,
  0x9fbfe4a5L, 0xe8b8d433L, 0x7807c9a2L, 0x0f00f934L, 0x9609a88eL,
  0xe10e9818L, 0x7f6a0dbbL, 0x086d3d2dL, 0x91646c97L, 0xe6635c01L,
  0x6b6b51f4L, 0x1c6c6162L, 0x856530d8L, 0xf262004eL, 0x6c0695edL,
  0x1b01a57bL, 0x8208f4c1L, 0xf50fc457L, 0x65b0d9c6L, 0x12b7e950L,
  0x8bbeb8eaL, 0xfcb9887cL, 0x62dd1ddfL, 0x15da2d49L, 0x8cd37cf3L,
  0xfbd44c65L, 0x4db26158L, 0x3ab551ceL, 0xa3bc0074L, 0xd4bb30e2L,
  0x4adfa541L, 0x3dd895d7L, 0xa4d1c46dL, 0xd3d6f4fbL, 0x4369e96aL,
  0x346ed9fcL, 0xad678846L, 0xda60b8d0L, 0x44042d73L, 0x33031de5L,
  0xaa0a4c5fL, 0xdd0d7cc9L, 0x5005713cL, 0x270241aaL, 0xbe0b1010L,
  0xc90c2086L, 0x5768b525L, 0x206f85b3L, 0xb966d409L, 0xce61e49fL,
  0x5edef90eL, 0x29d9c998L, 0xb0d09822L, 0xc7d7a8b4L, 0x59b33d17L,
  0x2eb40d81L, 0xb7bd5c3bL, 0xc0ba6cadL, 0xedb88320L, 0x9abfb3b6L,
  0x03b6e20cL, 0x74b1d29aL, 0xead54739L, 0x9dd277afL, 0x04db2615L,
  0x73dc1683L, 0xe3630b12L, 0x94643b84L, 0x0d6d6a3eL, 0x7a6a5aa8L,
  0xe40ecf0bL, 0x9309ff9dL, 0x0a00ae27L, 0x7d079eb1L, 0xf00f9344L,
  0x8708a3d2L, 0x1e01f268L, 0x6906c2feL, 0xf762575dL, 0x806567cbL,
  0x196c3671L, 0x6e6b06e7L, 0xfed41b76L, 0x89d32be0L, 0x10da7a5aL,
  0x67dd4accL, 0xf9b9df6fL, 0x8ebeeff9L, 0x17b7be43L, 0x60b08ed5L,
  0xd6d6a3e8L, 0xa1d1937eL, 0x38d8c2c4L, 0x4fdff252L, 0xd1bb67f1L,
  0xa6bc5767L, 0x3fb506ddL, 0x48b2364bL, 0xd80d2bdaL, 0xaf0a1b4cL,
  0x36034af6L, 0x41047a60L, 0xdf60efc3L, 0xa867df55L, 0x316e8eefL,
  0x4669be79L, 0xcb61b38cL, 0xbc66831aL, 0x256fd2a0L, 0x5268e236L,
  0xcc0c7795L, 0xbb0b4703L, 0x220216b9L, 0x5505262fL, 0xc5ba3bbeL,
  0xb2bd0b28L, 0x2bb45a92L, 0x5cb36a04L, 0xc2d7ffa7L, 0xb5d0cf31L,
  0x2cd99e8bL, 0x5bdeae1dL, 0x9b64c2b0L, 0xec63f226L, 0x756aa39cL,
  0x026d930aL, 0x9c0906a9L, 0xeb0e363fL, 0x72076785L, 0x05005713L,
  0x95bf4a82L, 0xe2b87a14L, 0x7bb12baeL, 0x0cb61b38L, 0x92d28e9bL,
  0xe5d5be0dL, 0x7cdcefb7L, 0x0bdbdf21L, 0x86d3d2d4L, 0xf1d4e242L,
  0x68ddb3f8L, 0x1fda836eL, 0x81be16cdL, 0xf6b9265bL, 0x6fb077e1L,
  0x18b74777L, 0x88085ae6L, 0xff0f6a70L, 0x66063bcaL, 0x11010b5cL,
  0x8f659effL, 0xf862ae69L, 0x616bffd3L, 0x166ccf45L, 0xa00ae278L,
  0xd70dd2eeL, 0x4e048354L, 0x3903b3c2L, 0xa7672661L, 0xd06016f7L,
  0x4969474dL, 0x3e6e77dbL, 0xaed16a4aL, 0xd9d65adcL, 0x40df0b66L,
  0x37d83bf0L, 0xa9bcae53L, 0xdebb9ec5L, 0x47b2cf7fL, 0x30b5ffe9L,
  0xbdbdf21cL, 0xcabac28aL, 0x53b39330L, 0x24b4a3a6L, 0xbad03605L,
  0xcdd70693L, 0x54de5729L, 0x23d967bfL, 0xb3667a2eL, 0xc4614ab8L,
  0x5d681b02L, 0x2a6f2b94L, 0xb40bbe37L, 0xc30c8ea1L, 0x5a05df1bL,
  0x2d02ef8dL
};

#define DO1(buf) crc = crc_table[((int)crc ^ (*buf++)) & 0xff] ^ (crc >> 8);
#define DO2(buf)  DO1(buf); DO1(buf);
#define DO4(buf)  DO2(buf); DO2(buf);
#define DO8(buf)  DO4(buf); DO4(buf);

#if defined (PRODUCT_WeMo_InsightCR)

#define FACTORY_PARTITION       "/dev/mtd3"
#define SERIAL_NUMBER_LENGTH     (16)
#define MODEL_NAME_LENGTH        (16)
#define COUNTRY_CODE_LENGTH      (2)
#define WIFI_SSID_LENGTH         (32)
#define ETHER_ADDR_LEN           (6)
#define ZIGBEE_EUI64_LENGTH      (16)
#define FW_VERSION_LENGTH        (16)
#define HW_VERSION_LENGTH        (16)
#define MAC_ADDRESS_LENGTH       (17)
#define CONSOLE_DISABLE_LENGTH   (1)
#define FACTORY_PARTITION_OFFSET (0x40000)
#define FACTORY_PARTITION_SIZE   (0x10000)
#define WIFI_MAC_ADDR_OFFSET     (0x4)
#define SERIAL_NUMBER_OFFSET     (0x160)
#define MODEL_NAME_OFFSET        (SERIAL_NUMBER_OFFSET + SERIAL_NUMBER_LENGTH)
#define COUNTRY_CODE_OFFSET      (MODEL_NAME_OFFSET + MODEL_NAME_LENGTH )
#define WIFI_SSID_OFFSET         (COUNTRY_CODE_OFFSET + COUNTRY_CODE_LENGTH)
#define FW_VERSION_OFFSET        (WIFI_SSID_OFFSET + WIFI_SSID_LENGTH)
#define ZIGBEE_EUI64_OFFSET      (FW_VERSION_OFFSET + FW_VERSION_LENGTH)
#define HW_VERSION_OFFSET        (ZIGBEE_EUI64_OFFSET + ZIGBEE_EUI64_LENGTH)
#define CONSOLE_DISABLE_OFFSET   (HW_VERSION_OFFSET + HW_VERSION_LENGTH)

#else

#define MASIC_STRING             "FACTORY"
#define FACTORY_PARTITION        "/dev/mtd7" /* User_Factory */
#define SERIAL_NUMBER_LENGTH     (16)
#define MODEL_NAME_LENGTH        (16)
#define COUNTRY_REGION_LENGTH    (16)
#define TARGET_COUNTRY_LENGTH    (16)
#define ETHER_ADDR_LEN           (6)
#define FACTORY_PARTITION_OFFSET (0xFE0000)
#define FACTORY_PARTITION_SIZE   (0x10000)

#define SERIAL_NUMBER_OFFSET     (0x11)
#define COUNTRY_REGION_OFFSET    (SERIAL_NUMBER_OFFSET + SERIAL_NUMBER_LENGTH )
#define TARGET_COUNTRY_OFFSET    (COUNTRY_REGION_OFFSET + COUNTRY_REGION_LENGTH )
#define MODEL_NAME_OFFSET        (TARGET_COUNTRY_OFFSET + TARGET_COUNTRY_LENGTH)

#define WIFI_MAC_ADDR_OFFSET     (0x77)

#endif

#define MAX_ARGS 8
#define JFFS2_DEFAULT_DIR	"" /* directory name without /, empty means root dir */

#if __BYTE_ORDER == __BIG_ENDIAN
#define STORE32_LE(X)           ((((X) & 0x000000FF) << 24) | (((X) & 0x0000FF00) << 8) | (((X) & 0x00FF0000) >> 8) | (((X) & 0xFF000000) >> 24))
#elif __BYTE_ORDER == __LITTLE_ENDIAN
#define STORE32_LE(X)           (X)
#else
#error unkown endianness!
#endif

#if defined (PRODUCT_WeMo_InsightCR)

enum {
    ITEM_MAC,
    ITEM_SSID,
    ITEM_MODEL,
    ITEM_SERIAL,
    ITEM_COUNTRY,
    ITEM_FW_VER,
    ITEM_HW_VER,
    ITEM_ZGB_MAC,
    ITEM_CSL_OFF,
    ITEM_TOTAL
};

char factoryData[ITEM_TOTAL][WIFI_SSID_LENGTH+1];

#else

enum {
    ITEM_SERIAL,
    ITEM_REGION,
    ITEM_COUNTRY,
    ITEM_MODEL,
    ITEM_MAC,
    ITEM_TOTAL
};

char factoryData[ITEM_TOTAL][SERIAL_NUMBER_LENGTH+1];

#endif

ssize_t pread(int fd, void *buf, size_t count, off_t offset);
ssize_t pwrite(int fd, const void *buf, size_t count, off_t offset);
unsigned long cal_crc32(unsigned long crc, const unsigned char *buf, unsigned int len);

#define TRX_MAGIC       0x30524448      /* "HDR0" */
struct trx_header {
	uint32_t magic;		/* "HDR0" */
	uint32_t len;		/* Length of file including header */
	uint32_t crc32;		/* 32-bit CRC from flag_version to end of file */
	uint32_t flag_version;	/* 0:15 flags, 16:31 version */
	uint32_t offsets[3];    /* Offsets of partitions from start of header */
};

static char *buf = NULL;
static char *imagefile = NULL;
static char *jffs2file = NULL, *jffs2dir = JFFS2_DEFAULT_DIR;
static int buflen = 0;
int quiet;
int mtdsize = 0;
int erasesize = 0;

int mtd_open(const char *mtd, bool block)
{
	FILE *fp;
	char dev[PATH_MAX];
	int i;
	int ret;
	int flags = O_RDWR | O_SYNC;

	if ((fp = fopen("/proc/mtd", "r"))) {
		while (fgets(dev, sizeof(dev), fp)) {
			if (sscanf(dev, "mtd%d:", &i) && strstr(dev, mtd)) {
				snprintf(dev, sizeof(dev), "/dev/mtd%s/%d", (block ? "block" : ""), i);
				if ((ret=open(dev, flags))<0) {
					snprintf(dev, sizeof(dev), "/dev/mtd%s%d", (block ? "block" : ""), i);
					ret=open(dev, flags);
				}
				fclose(fp);
				return ret;
			}
		}
		fclose(fp);
	}

	return open(mtd, flags);
}

int mtd_check_open(const char *mtd)
{
	struct mtd_info_user mtdInfo;
	int fd;

	fd = mtd_open(mtd, false);
	if(fd < 0) {
		fprintf(stderr, "Could not open mtd device: %s\n", mtd);
		return -1;
	}

	if(ioctl(fd, MEMGETINFO, &mtdInfo)) {
		fprintf(stderr, "Could not get MTD device info from %s\n", mtd);
		close(fd);
		return -1;
	}
	mtdsize = mtdInfo.size;
	erasesize = mtdInfo.erasesize;

   if((gOffset % erasesize) != 0) {
      fprintf(stderr,"Error offset 0x%x is not aligned with erase boundary\n",
              gOffset);
      close(fd);
      return -1;
   }

   if(gOffset != 0) {
      if(lseek(fd, (off_t) gOffset,SEEK_SET) == (off_t) -1) {
         fprintf(stderr, "Could not seek MTD device %s\n", mtd);
         close(fd);
         return -1;
      }
   }
	return fd;
}

int mtd_erase_block(int fd, int offset)
{
	struct erase_info_user mtdEraseInfo;

   mtdEraseInfo.start = offset + gOffset;
	mtdEraseInfo.length = erasesize;
	ioctl(fd, MEMUNLOCK, &mtdEraseInfo);
	if (ioctl (fd, MEMERASE, &mtdEraseInfo) < 0)
		return -1;

	return 0;
}

int mtd_write_buffer(int fd, const char *buf, int offset, int length)
{
	lseek(fd, offset, SEEK_SET);
	write(fd, buf, length);
	return 0;
}


static int
image_check(int imagefd, const char *mtd)
{
	int ret = 1;
#ifdef target_brcm
	ret = trx_check(imagefd, mtd, buf, &buflen);
#endif
	return ret;
}

static int mtd_check(const char *mtd)
{
	char *next = NULL;
	char *str = NULL;
	int fd;

	if (strchr(mtd, ':')) {
		str = strdup(mtd);
		mtd = str;
	}

	do {
		next = strchr(mtd, ':');
		if (next) {
			*next = 0;
			next++;
		}

		fd = mtd_check_open(mtd);
		if (fd < 0)
			return 0;

		if (!buf)
			buf = malloc(erasesize);

		close(fd);
		mtd = next;
	} while (next);

	if (str)
		free(str);

	return 1;
}

static int
mtd_unlock(const char *mtd)
{
	struct erase_info_user mtdLockInfo;
	char *next = NULL;
	char *str = NULL;
	int fd;

	if (strchr(mtd, ':')) {
		str = strdup(mtd);
		mtd = str;
	}

	do {
		next = strchr(mtd, ':');
		if (next) {
			*next = 0;
			next++;
		}

		fd = mtd_check_open(mtd);
		if(fd < 0) {
			fprintf(stderr, "Could not open mtd device: %s\n", mtd);
			exit(1);
		}

//		if (quiet < 2)
//			fprintf(stderr, "Unlocking %s ...\n", mtd);

		mtdLockInfo.start = 0;
		mtdLockInfo.length = mtdsize;
		ioctl(fd, MEMUNLOCK, &mtdLockInfo);
		close(fd);
		mtd = next;
	} while (next);

	if (str)
		free(str);

	return 0;
}

static int
mtd_erase(const char *mtd)
{
	int fd;
	struct erase_info_user mtdEraseInfo;

	if (quiet < 2)
		fprintf(stderr, "Erasing %s ...\n", mtd);

	fd = mtd_check_open(mtd);
	if(fd < 0) {
		fprintf(stderr, "Could not open mtd device: %s\n", mtd);
		exit(1);
	}

	mtdEraseInfo.length = erasesize;

	for (mtdEraseInfo.start = 0;
		 mtdEraseInfo.start < mtdsize;
		 mtdEraseInfo.start += erasesize) {

		ioctl(fd, MEMUNLOCK, &mtdEraseInfo);
		if(ioctl(fd, MEMERASE, &mtdEraseInfo))
			fprintf(stderr, "Failed to erase block on %s at 0x%x\n", mtd, mtdEraseInfo.start);
	}

	close(fd);
	return 0;

}

static int
mtd_fixtrx(const char *mtd, size_t offset)
{
	int fd;
	struct trx_header *trx;
	char *buf;
	ssize_t res;
	size_t block_offset;

	if (quiet < 2)
		fprintf(stderr, "Trying to fix trx header in %s at 0x%x...\n", mtd, offset);

	block_offset = offset & ~(erasesize - 1);
	offset -= block_offset;

	fd = mtd_check_open(mtd);
	if(fd < 0) {
		fprintf(stderr, "Could not open mtd device: %s\n", mtd);
		exit(1);
	}

	if (block_offset + erasesize > mtdsize) {
		fprintf(stderr, "Offset too large, device size 0x%x\n", mtdsize);
		exit(1);
	}

	buf = malloc(erasesize);
	if (!buf) {
		perror("malloc");
		exit(1);
	}

	res = pread(fd, buf, erasesize, block_offset);
	if (res != erasesize) {
		perror("pread");
		exit(1);
	}

	trx = (struct trx_header *) (buf + offset);
	if (trx->magic != STORE32_LE(0x30524448)) {
		fprintf(stderr, "No trx magic found\n");
		exit(1);
	}

	if (trx->len == STORE32_LE(erasesize - offset)) {
		if (quiet < 2)
			fprintf(stderr, "Header already fixed, exiting\n");
		close(fd);
		return 0;
	}

	trx->len = STORE32_LE(erasesize - offset);

	trx->crc32 = STORE32_LE(crc32buf((char*) &trx->flag_version, erasesize - offset - 3*4));
	if (mtd_erase_block(fd, block_offset)) {
		fprintf(stderr, "Can't erease block at 0x%x (%s)\n", block_offset, strerror(errno));
		exit(1);
	}

	if (quiet < 2)
		fprintf(stderr, "New crc32: 0x%x, rewriting block\n", trx->crc32);

	if (pwrite(fd, buf, erasesize, block_offset) != erasesize) {
		fprintf(stderr, "Error writing block (%s)\n", strerror(errno));
		exit(1);
	}

	if (quiet < 2)
		fprintf(stderr, "Done.\n");

	close (fd);
	sync();
	return 0;

}

static int
mtd_refresh(const char *mtd)
{
	int fd;

	if (quiet < 2)
		fprintf(stderr, "Refreshing mtd partition %s ... ", mtd);

	fd = mtd_check_open(mtd);
	if(fd < 0) {
		fprintf(stderr, "Could not open mtd device: %s\n", mtd);
		exit(1);
	}

	if (ioctl(fd, MTDREFRESH, NULL)) {
		fprintf(stderr, "Failed to refresh the MTD device\n");
		close(fd);
		exit(1);
	}
	close(fd);

	if (quiet < 2)
		fprintf(stderr, "\n");

	return 0;
}

static void
mtd_read_core(const char* mtd, char *data)
{
	int fd = mtd_check_open(mtd);
	if(fd < 0) {
		fprintf(stderr, "Could not open mtd device: %s\n", mtd);
		return;
	}
    read(fd, data, mtdsize);
    close(fd);
}

static void
mtd_read(const char *mtd)
{
    if(!dataBuf)
        dataBuf = (char*)malloc(sizeof(char) * mtdsize);

    if(!dataBuf) {
		fprintf(stderr, "Could not alloc data array\n");
		return;
	}

    mtd_read_core(mtd, dataBuf);

    printf("%s:\n", mtd );

    int i = 0;
    for( i = 0; i < mtdsize; i++) {
        if(i>0 && i%16 == 0)
            printf("\n");
        printf("%02X ", dataBuf[i]&0xFF );
    }

    free(dataBuf);
    dataBuf = NULL;
}

#if defined (PRODUCT_WeMo_InsightCR)

static void
mtd_factory_get(const char *item)
{
    mtdsize = FACTORY_PARTITION_SIZE;

    if(!dataBuf)
        dataBuf = (char*)malloc(sizeof(char) * mtdsize);

    if(!dataBuf) {
		fprintf(stderr, "Could not alloc data array\n");
		return;
	}

    mtd_read_core(FACTORY_PARTITION, dataBuf);

    int i = 0;
    if(!strcmp(item, "all")) {
        char prtBuf[50];
        char *ptr = NULL;
        memset(prtBuf, 0, sizeof(prtBuf));
        // ITEM_MAC:
        printf("       WiFi MAC: %02X:%02X:%02X:%02X:%02X:%02X\n", 
                dataBuf[WIFI_MAC_ADDR_OFFSET] & 0xFF,
                dataBuf[WIFI_MAC_ADDR_OFFSET + 1] & 0xFF,
                dataBuf[WIFI_MAC_ADDR_OFFSET + 2] & 0xFF,
                dataBuf[WIFI_MAC_ADDR_OFFSET + 3] & 0xFF,
                dataBuf[WIFI_MAC_ADDR_OFFSET + 4] & 0xFF,
                dataBuf[WIFI_MAC_ADDR_OFFSET + 5] & 0xFF );

        // ITEM_SSID:
        memset(prtBuf, 0, sizeof(prtBuf));
        ptr = &dataBuf[0] + WIFI_SSID_OFFSET;
        for(i = 0; (*ptr&0xFF) != 0xFF && i < WIFI_SSID_LENGTH; i++, ptr++)
            prtBuf[i] = *ptr;
        prtBuf[WIFI_SSID_LENGTH] = '\0';
        printf("      WiFi SSID: %s\n", prtBuf );

        // ITEM_MODEL:
        memset(prtBuf, 0, sizeof(prtBuf));
        ptr = &dataBuf[0] + MODEL_NAME_OFFSET;
        for(i = 0; (*ptr&0xFF) != 0xFF && i < MODEL_NAME_LENGTH; i++, ptr++)
            prtBuf[i] = *ptr;
        prtBuf[MODEL_NAME_LENGTH] = '\0';
        printf("     Model name: %s\n", prtBuf );

        // ITEM_SERIAL:
        memset(prtBuf, 0, sizeof(prtBuf));
        ptr = &dataBuf[0] + SERIAL_NUMBER_OFFSET;
        for(i = 0; (*ptr&0xFF) != 0xFF && i < SERIAL_NUMBER_LENGTH; i++, ptr++)
            prtBuf[i] = *ptr;
        prtBuf[SERIAL_NUMBER_LENGTH] = '\0';
        printf("  Serial number: %s\n", prtBuf );

        // ITEM_COUNTRY:
        memset(prtBuf, 0, sizeof(prtBuf));
        ptr = &dataBuf[0] + COUNTRY_CODE_OFFSET;
        for(i = 0; (*ptr&0xFF) != 0xFF && i < COUNTRY_CODE_LENGTH; i++, ptr++)
            prtBuf[i] = *ptr;
        prtBuf[COUNTRY_CODE_LENGTH] = '\0';
        printf("        Country: %s\n", prtBuf );

        // ITEM_FW_VER:
        memset(prtBuf, 0, sizeof(prtBuf));
        ptr = &dataBuf[0] + FW_VERSION_OFFSET;
        for(i = 0; (*ptr&0xFF) != 0xFF && i < FW_VERSION_LENGTH; i++, ptr++)
            prtBuf[i] = *ptr;
        prtBuf[FW_VERSION_LENGTH] = '\0';
        printf("     FW Version: %s\n", prtBuf );

        // ITEM_HW_VER:
        memset(prtBuf, 0, sizeof(prtBuf));
        ptr = &dataBuf[0] + HW_VERSION_OFFSET;
        for(i = 0; (*ptr&0xFF) != 0xFF && i < HW_VERSION_LENGTH; i++, ptr++)
            prtBuf[i] = *ptr;
        prtBuf[HW_VERSION_LENGTH] = '\0';
        printf("     HW Version: %s\n", prtBuf );

        // ITEM_ZGB_MAC:
        memset(prtBuf, 0, sizeof(prtBuf));
        ptr = &dataBuf[0] + ZIGBEE_EUI64_OFFSET;
        for(i = 0; (*ptr&0xFF) != 0xFF && i < ZIGBEE_EUI64_LENGTH; i++, ptr++)
            prtBuf[i] = *ptr;
        prtBuf[ZIGBEE_EUI64_LENGTH] = '\0';
        printf("     Zigbee MAC: %s\n", prtBuf );

        // ITEM_CSL_OFF:
        memset(prtBuf, 0, sizeof(prtBuf));
        ptr = &dataBuf[0] + CONSOLE_DISABLE_OFFSET;
        for(i = 0; (*ptr&0xFF) != 0xFF && i < CONSOLE_DISABLE_LENGTH; i++, ptr++)
            prtBuf[i] = *ptr;
        prtBuf[CONSOLE_DISABLE_LENGTH] = '\0';
        printf("Console disable: %s\n", prtBuf );
        return;
    }

    for(i = 0; i < ITEM_TOTAL; i++) {
        if(!strcmp(factoryItem[i], item)) {
            char prtBuf[50];
            char *ptr = NULL;
            memset(prtBuf, 0, sizeof(prtBuf));
            switch(i)
            {
                case ITEM_MAC:
                    sprintf(prtBuf, "%02X:%02X:%02X:%02X:%02X:%02X", 
                            dataBuf[WIFI_MAC_ADDR_OFFSET] & 0xFF,
                            dataBuf[WIFI_MAC_ADDR_OFFSET + 1] & 0xFF,
                            dataBuf[WIFI_MAC_ADDR_OFFSET + 2] & 0xFF,
                            dataBuf[WIFI_MAC_ADDR_OFFSET + 3] & 0xFF,
                            dataBuf[WIFI_MAC_ADDR_OFFSET + 4] & 0xFF,
                            dataBuf[WIFI_MAC_ADDR_OFFSET + 5] & 0xFF );
                    break;
                case ITEM_SSID:
                    ptr = &dataBuf[0] + WIFI_SSID_OFFSET;
                    for(i = 0; (*ptr&0xFF) != 0xFF && i < WIFI_SSID_LENGTH; i++, ptr++)
                        prtBuf[i] = *ptr;
                    prtBuf[WIFI_SSID_LENGTH] = '\0';
                    break;
                case ITEM_MODEL:
                    ptr = &dataBuf[0] + MODEL_NAME_OFFSET;
                    for(i = 0; (*ptr&0xFF) != 0xFF && i < MODEL_NAME_LENGTH; i++, ptr++)
                        prtBuf[i] = *ptr;
                    prtBuf[MODEL_NAME_LENGTH] = '\0';
                    break;
                case ITEM_SERIAL:
                    ptr = &dataBuf[0] + SERIAL_NUMBER_OFFSET;
                    for(i = 0; (*ptr&0xFF) != 0xFF && i < SERIAL_NUMBER_LENGTH; i++, ptr++)
                        prtBuf[i] = *ptr;
                    prtBuf[SERIAL_NUMBER_LENGTH] = '\0';
                    break;
                case ITEM_COUNTRY:
                    ptr = &dataBuf[0] + COUNTRY_CODE_OFFSET;
                    for(i = 0; (*ptr&0xFF) != 0xFF && i < COUNTRY_CODE_LENGTH; i++, ptr++)
                        prtBuf[i] = *ptr;
                    prtBuf[COUNTRY_CODE_LENGTH] = '\0';
                    break;
                case ITEM_FW_VER:
                    ptr = &dataBuf[0] + FW_VERSION_OFFSET;
                    for(i = 0; (*ptr&0xFF) != 0xFF && i < FW_VERSION_LENGTH; i++, ptr++)
                        prtBuf[i] = *ptr;
                    prtBuf[FW_VERSION_LENGTH] = '\0';
                    break;
                case ITEM_HW_VER:
                    ptr = &dataBuf[0] + HW_VERSION_OFFSET;
                    for(i = 0; (*ptr&0xFF) != 0xFF && i < HW_VERSION_LENGTH; i++, ptr++)
                        prtBuf[i] = *ptr;
                    prtBuf[HW_VERSION_LENGTH] = '\0';
                    break;
                case ITEM_ZGB_MAC:
                    ptr = &dataBuf[0] + ZIGBEE_EUI64_OFFSET;
                    for(i = 0; (*ptr&0xFF) != 0xFF && i < ZIGBEE_EUI64_LENGTH; i++, ptr++)
                        prtBuf[i] = *ptr;
                    prtBuf[ZIGBEE_EUI64_LENGTH] = '\0';
                    break;
                case ITEM_CSL_OFF:
                    ptr = &dataBuf[0] + CONSOLE_DISABLE_OFFSET;
                    for(i = 0; (*ptr&0xFF) != 0xFF && i < CONSOLE_DISABLE_LENGTH; i++, ptr++)
                        prtBuf[i] = *ptr;
                    prtBuf[CONSOLE_DISABLE_LENGTH] = '\0';
                    break;
                default:
                    break;
            }
            printf("%s\n", prtBuf );
            break;
        } 
    }
    free(dataBuf);
    dataBuf = NULL;
}

#else

static void
mtd_factory_get(const char *item)
{
    mtdsize = FACTORY_PARTITION_SIZE;

    if(!dataBuf)
        dataBuf = (char*)malloc(sizeof(char) * mtdsize);

    if(!dataBuf) {
		fprintf(stderr, "Could not alloc data array\n");
		return;
	}

    mtd_read_core(FACTORY_PARTITION, dataBuf);

    int i = 0;
    if(!strcmp(item, "all")) {
        char prtBuf[50];
        char *ptr = NULL;
        memset(prtBuf, 0, sizeof(prtBuf));
        // ITEM_MAC:
        printf("       WiFi MAC: %02X:%02X:%02X:%02X:%02X:%02X\n", 
                dataBuf[WIFI_MAC_ADDR_OFFSET] & 0xFF,
                dataBuf[WIFI_MAC_ADDR_OFFSET + 1] & 0xFF,
                dataBuf[WIFI_MAC_ADDR_OFFSET + 2] & 0xFF,
                dataBuf[WIFI_MAC_ADDR_OFFSET + 3] & 0xFF,
                dataBuf[WIFI_MAC_ADDR_OFFSET + 4] & 0xFF,
                dataBuf[WIFI_MAC_ADDR_OFFSET + 5] & 0xFF );

        // ITEM_MODEL:
        memset(prtBuf, 0, sizeof(prtBuf));
        ptr = &dataBuf[0] + MODEL_NAME_OFFSET;
        for(i = 0; (*ptr&0xFF) != 0xFF && i < MODEL_NAME_LENGTH; i++, ptr++)
            prtBuf[i] = *ptr;
        prtBuf[MODEL_NAME_LENGTH] = '\0';
        printf("     Model name: %s\n", prtBuf );

        // ITEM_SERIAL:
        memset(prtBuf, 0, sizeof(prtBuf));
        ptr = &dataBuf[0] + SERIAL_NUMBER_OFFSET;
        for(i = 0; (*ptr&0xFF) != 0xFF && i < SERIAL_NUMBER_LENGTH; i++, ptr++)
            prtBuf[i] = *ptr;
        prtBuf[SERIAL_NUMBER_LENGTH] = '\0';
        printf("  Serial number: %s\n", prtBuf );

        // ITEM_REGION:
        memset(prtBuf, 0, sizeof(prtBuf));
        ptr = &dataBuf[0] + COUNTRY_REGION_OFFSET;
        for(i = 0; (*ptr&0xFF) != 0xFF && i < COUNTRY_REGION_LENGTH; i++, ptr++)
            prtBuf[i] = *ptr;
        prtBuf[COUNTRY_REGION_LENGTH] = '\0';
        printf(" Country Region: %s\n", prtBuf );

        // ITEM_COUNTRY:
        memset(prtBuf, 0, sizeof(prtBuf));
        ptr = &dataBuf[0] + TARGET_COUNTRY_OFFSET;
        for(i = 0; (*ptr&0xFF) != 0xFF && i < TARGET_COUNTRY_LENGTH; i++, ptr++)
            prtBuf[i] = *ptr;
        prtBuf[TARGET_COUNTRY_LENGTH] = '\0';
        printf(" Target Country: %s\n", prtBuf );
    }

    for(i = 0; i < ITEM_TOTAL; i++) {
        if(!strcmp(factoryItem[i], item)) {
            char prtBuf[50];
            char *ptr = NULL;
            memset(prtBuf, 0, sizeof(prtBuf));
            switch(i)
            {
                case ITEM_MAC:
                    sprintf(prtBuf, "%02X:%02X:%02X:%02X:%02X:%02X", 
                            dataBuf[WIFI_MAC_ADDR_OFFSET] & 0xFF,
                            dataBuf[WIFI_MAC_ADDR_OFFSET + 1] & 0xFF,
                            dataBuf[WIFI_MAC_ADDR_OFFSET + 2] & 0xFF,
                            dataBuf[WIFI_MAC_ADDR_OFFSET + 3] & 0xFF,
                            dataBuf[WIFI_MAC_ADDR_OFFSET + 4] & 0xFF,
                            dataBuf[WIFI_MAC_ADDR_OFFSET + 5] & 0xFF );
                    break;
                case ITEM_SERIAL:
                    ptr = &dataBuf[0] + SERIAL_NUMBER_OFFSET;
                    for(i = 0; (*ptr&0xFF) != 0xFF && i < SERIAL_NUMBER_LENGTH; i++, ptr++)
                        prtBuf[i] = *ptr;
                    prtBuf[SERIAL_NUMBER_LENGTH] = '\0';
                    break;
                case ITEM_REGION:
                    ptr = &dataBuf[0] + COUNTRY_REGION_OFFSET;
                    for(i = 0; (*ptr&0xFF) != 0xFF && i < COUNTRY_REGION_LENGTH; i++, ptr++)
                        prtBuf[i] = *ptr;
                    prtBuf[COUNTRY_REGION_LENGTH] = '\0';
                    break;
                case ITEM_COUNTRY:
                    ptr = &dataBuf[0] + TARGET_COUNTRY_OFFSET;
                    for(i = 0; (*ptr&0xFF) != 0xFF && i < TARGET_COUNTRY_LENGTH; i++, ptr++)
                        prtBuf[i] = *ptr;
                    prtBuf[TARGET_COUNTRY_LENGTH] = '\0';
                    break;
                case ITEM_MODEL:
                    ptr = &dataBuf[0] + MODEL_NAME_OFFSET;
                    for(i = 0; (*ptr&0xFF) != 0xFF && i < MODEL_NAME_LENGTH; i++, ptr++)
                        prtBuf[i] = *ptr;
                    prtBuf[MODEL_NAME_LENGTH] = '\0';
                    break;
                default:
                    break;
            }
            printf("%s\n", prtBuf );
            break;
        } 
    }
    free(dataBuf);
    dataBuf = NULL;
}

#endif

static int
mtd_write(int imagefd, const char *mtd, char *fis_layout)
{
	char *next = NULL;
	char *str = NULL;
	int fd, result;
	ssize_t r, w, e;
	uint32_t offset = 0;
	char *strtok_r_temp;

#ifdef FIS_SUPPORT
	static struct fis_part new_parts[MAX_ARGS];
	static struct fis_part old_parts[MAX_ARGS];
	int n_new = 0, n_old = 0;

	if (fis_layout) {
		const char *tmp = mtd;
		char *word, *brkt;
		int ret;

		memset(&old_parts, 0, sizeof(old_parts));
		memset(&new_parts, 0, sizeof(new_parts));

		do {
			next = strchr(tmp, ':');
			if (!next)
				next = (char *) tmp + strlen(tmp);

			memcpy(old_parts[n_old].name, tmp, next - tmp);

			n_old++;
			tmp = next + 1;
		} while(*next);

		for (word = strtok_r(fis_layout, ",", &brkt);
		     word;
			 word = strtok_r(NULL, ",", &brkt)) {

			tmp = strtok_r(word, ":",&strtok_r_temp);
			strncpy((char *) new_parts[n_new].name, tmp, sizeof(new_parts[n_new].name) - 1);

			tmp = strtok_r(NULL, ":",&strtok_r_temp);
			if (!tmp)
				goto next;

			new_parts[n_new].size = strtoul(tmp, NULL, 0);

			tmp = strtok_r(NULL, ":",&strtok_r_temp);
			if (!tmp)
				goto next;

			new_parts[n_new].loadaddr = strtoul(tmp, NULL, 16);
next:
			n_new++;
		}
		ret = fis_validate(old_parts, n_old, new_parts, n_new);
		if (ret < 0) {
			fprintf(stderr, "Failed to validate the new FIS partition table\n");
			exit(1);
		}
		if (ret == 0)
			fis_layout = NULL;
	}
#endif

	if (strchr(mtd, ':')) {
		str = strdup(mtd);
		mtd = str;
	}

	r = 0;

resume:
	next = strchr(mtd, ':');
	if (next) {
		*next = 0;
		next++;
	}

	fd = mtd_check_open(mtd);
	if(fd < 0) {
		fprintf(stderr, "Could not open mtd device: %s\n", mtd);
		exit(1);
	}

	if (quiet < 2)
		fprintf(stderr, "Writing from %s to %s ... ", imagefile, mtd);

	w = e = 0;
	if (!quiet)
		fprintf(stderr, " [ ]");

	for (;;) {
		/* buffer may contain data already (from trx check or last mtd partition write attempt) */
		while (buflen < erasesize) {
			r = read(imagefd, buf + buflen, erasesize - buflen);
			if (r < 0) {
				if ((errno == EINTR) || (errno == EAGAIN))
					continue;
				else {
					perror("read");
					break;
				}
			}

			if (r == 0)
				break;

			buflen += r;
		}

		if (buflen == 0)
			break;

		if (jffs2file) {
			if (memcmp(buf, JFFS2_EOF, sizeof(JFFS2_EOF) - 1) == 0) {
				if (!quiet)
					fprintf(stderr, "\b\b\b   ");
				if (quiet < 2)
					fprintf(stderr, "\nAppending jffs2 data from %s to %s...", jffs2file, mtd);
				/* got an EOF marker - this is the place to add some jffs2 data */
				mtd_replace_jffs2(mtd, fd, e, jffs2file);
				goto done;
			}
			/* no EOF marker, make sure we figure out the last inode number
			 * before appending some data */
			mtd_parse_jffs2data(buf, jffs2dir);
		}

		/* need to erase the next block before writing data to it */
		while (w + buflen > e) {
			if (!quiet)
				fprintf(stderr, "\b\b\b[e]");


			if (mtd_erase_block(fd, e) < 0) {
				if (next) {
					if (w < e) {
						write(fd, buf + offset, e - w);
						offset = e - w;
					}
					w = 0;
					e = 0;
					close(fd);
					mtd = next;
					fprintf(stderr, "\b\b\b   \n");
					goto resume;
				} else {
					fprintf(stderr, "Failed to erase block\n");
					exit(1);
				}
			}

			/* erase the chunk */
			e += erasesize;
		}

		if (!quiet)
			fprintf(stderr, "\b\b\b[w]");

		if ((result = write(fd, buf + offset, buflen)) < buflen) {
			if (result < 0) {
				fprintf(stderr, "Error writing image.\n");
				exit(1);
			} else {
				fprintf(stderr, "Insufficient space.\n");
				exit(1);
			}
		}
		w += buflen;

		buflen = 0;
		offset = 0;
	}

	if (!quiet)
		fprintf(stderr, "\b\b\b\b    ");

done:
	if (quiet < 2)
		fprintf(stderr, "\n");

#ifdef FIS_SUPPORT
	if (fis_layout) {
		if (fis_remap(old_parts, n_old, new_parts, n_new) < 0)
			fprintf(stderr, "Failed to update the FIS partition table\n");
	}
#endif

	close(fd);
	return 0;
}

static int ether_atoe(const char *a, unsigned char *e)
{
    char *c = (char *) a;
    char buf[3] = { '\0', '\0', '\0' };
    int i = 0;  
                    
    memset(e, 0, ETHER_ADDR_LEN);
    while(1) {          
        buf[0] = *c;        
        buf[1] = *(c+1);
        e[i++] = (unsigned char) strtoul(buf, NULL, 16);
        c+=2;
        if (!*c || i == ETHER_ADDR_LEN)
            break;
    }
    return (i == ETHER_ADDR_LEN);
}

#if defined (PRODUCT_WeMo_InsightCR)

static int
mtd_factory_set(char* parameter)
{
    mtdsize = FACTORY_PARTITION_SIZE;

    if(!dataBuf)
        dataBuf = (char*)malloc(sizeof(char) * mtdsize);

    if(!dataBuf) {
		fprintf(stderr, "Could not alloc data array\n");
		return 1;
	}
    mtd_read_core(FACTORY_PARTITION, dataBuf);

    int i = 0;
    char *ptr = dataBuf;
    if(factoryData[ITEM_MAC][0]) {
        fprintf(stderr, "Found ITEM_MAC     (%s), update to buffer...\n", factoryData[ITEM_MAC]);
        ptr = &dataBuf[0] + WIFI_MAC_ADDR_OFFSET;
        unsigned char hwaddr[6];
        ether_atoe(factoryData[ITEM_MAC], hwaddr);
        for(i = 0; i < ETHER_ADDR_LEN; i++, ptr++)
            *ptr = hwaddr[i];
    } 
    if(factoryData[ITEM_SSID][0]) {
        fprintf(stderr, "Found ITEM_SSID    (%s), update to buffer...\n", factoryData[ITEM_SSID]);
        ptr = &dataBuf[0] + WIFI_SSID_OFFSET;
        for(i = 0; i < WIFI_SSID_LENGTH; i++, ptr++) {
            if(i < strlen(factoryData[ITEM_SSID]))
                *ptr = factoryData[ITEM_SSID][i];
            else
                *ptr = 0xFF;
        }
    } 
    if(factoryData[ITEM_MODEL][0]) {
        fprintf(stderr, "Found ITEM_MODEL   (%s), update to buffer...\n", factoryData[ITEM_MODEL]);
        ptr = &dataBuf[0] + MODEL_NAME_OFFSET;
        for(i = 0; i < MODEL_NAME_LENGTH; i++, ptr++) {
            if(i < strlen(factoryData[ITEM_MODEL]))
                *ptr = factoryData[ITEM_MODEL][i];
            else
                *ptr = 0xFF;
        }
    } 
    if(factoryData[ITEM_SERIAL][0]) {
        fprintf(stderr, "Found ITEM_SERIAL  (%s), update to buffer...\n", factoryData[ITEM_SERIAL]);
        ptr = &dataBuf[0] + SERIAL_NUMBER_OFFSET;
        for(i = 0; i < SERIAL_NUMBER_LENGTH; i++, ptr++) {
            if(i < strlen(factoryData[ITEM_SERIAL]))
                *ptr = factoryData[ITEM_SERIAL][i];
            else
                *ptr = 0xFF;
        }
    } 
    if(factoryData[ITEM_COUNTRY][0]) {
        fprintf(stderr, "Found ITEM_COUNTRY (%s), update to buffer...\n", factoryData[ITEM_COUNTRY]);
        ptr = &dataBuf[0] + COUNTRY_CODE_OFFSET;
        for(i = 0; i < COUNTRY_CODE_LENGTH; i++, ptr++) {
            if(i < strlen(factoryData[ITEM_COUNTRY]))
                *ptr = factoryData[ITEM_COUNTRY][i];
            else
                *ptr = 0xFF;
        }
    } 
    if(factoryData[ITEM_FW_VER][0]) {
        fprintf(stderr, "Found ITEM_FW_VER  (%s), update to buffer...\n", factoryData[ITEM_FW_VER]);
        ptr = &dataBuf[0] + FW_VERSION_OFFSET;
        for(i = 0; i < FW_VERSION_LENGTH; i++, ptr++) {
            if(i < strlen(factoryData[ITEM_FW_VER]))
                *ptr = factoryData[ITEM_FW_VER][i];
            else
                *ptr = 0xFF;
        }
    } 
    if(factoryData[ITEM_HW_VER][0]) {
        fprintf(stderr, "Found ITEM_HW_VER  (%s), update to buffer...\n", factoryData[ITEM_HW_VER]);
        ptr = &dataBuf[0] + HW_VERSION_OFFSET;
        for(i = 0; i < HW_VERSION_LENGTH; i++, ptr++) {
            if(i < strlen(factoryData[ITEM_HW_VER]))
                *ptr = factoryData[ITEM_HW_VER][i];
            else
                *ptr = 0xFF;
        }
    } 
    if(factoryData[ITEM_ZGB_MAC][0]) {
        fprintf(stderr, "Found ITEM_ZGB_MAC (%s), update to buffer...\n", factoryData[ITEM_ZGB_MAC]);
        ptr = &dataBuf[0] + ZIGBEE_EUI64_OFFSET;
        for(i = 0; i < ZIGBEE_EUI64_LENGTH; i++, ptr++) {
            if(i < strlen(factoryData[ITEM_ZGB_MAC]))
                *ptr = factoryData[ITEM_ZGB_MAC][i];
            else
                *ptr = 0xFF;
        }
    } 
    if(factoryData[ITEM_CSL_OFF][0]) {
        fprintf(stderr, "Found ITEM_CSL_OFF (%s), update to buffer...\n", factoryData[ITEM_CSL_OFF]);
        ptr = &dataBuf[0] + CONSOLE_DISABLE_OFFSET;
        for(i = 0; i < CONSOLE_DISABLE_LENGTH; i++, ptr++) {
            if(i < strlen(factoryData[ITEM_CSL_OFF]))
                *ptr = factoryData[ITEM_CSL_OFF][i];
            else
                *ptr = 0xFF;
        }
    }

    /* write to factory */
	int fd = mtd_check_open(FACTORY_PARTITION);
    fprintf(stderr, "Write buffer to %s...\n", FACTORY_PARTITION );
	if(fd < 0) {
		fprintf(stderr, "Could not open mtd device: %s\n", FACTORY_PARTITION);
		return 1;
	}

	if (mtd_erase_block(fd, 0)) {
		fprintf(stderr, "Can't erease block at 0x%x (%s)\n", 0, strerror(errno));
        return 1;
	}

    mtd_write_buffer(fd, dataBuf, 0, mtdsize);
    close(fd);

    if(dataBuf) {
        free(dataBuf);
        dataBuf = NULL;
    }
	return 0;
}

#else

static int
mtd_factory_set(char* parameter)
{
    mtdsize = FACTORY_PARTITION_SIZE;

    if(!dataBuf)
        dataBuf = (char*)malloc(sizeof(char) * mtdsize);

    if(!dataBuf) {
		fprintf(stderr, "Could not alloc data array\n");
		return 1;
	}
    mtd_read_core(FACTORY_PARTITION, dataBuf);

    int i = 0;
    char *ptr = dataBuf;
    if(factoryData[ITEM_SERIAL][0]) {
        fprintf(stderr, "Found ITEM_SERIAL  (%s), update to buffer...\n", factoryData[ITEM_SERIAL]);
        ptr = &dataBuf[0] + SERIAL_NUMBER_OFFSET;
        for(i = 0; i < SERIAL_NUMBER_LENGTH; i++, ptr++) {
            if(i < strlen(factoryData[ITEM_SERIAL]))
                *ptr = factoryData[ITEM_SERIAL][i];
            else
                *ptr = 0x00;
        }
    } 
    if(factoryData[ITEM_REGION][0]) {
        fprintf(stderr, "Found ITEM_REGION (%s), update to buffer...\n", factoryData[ITEM_REGION]);
        ptr = &dataBuf[0] + COUNTRY_REGION_OFFSET;
        for(i = 0; i < COUNTRY_REGION_LENGTH; i++, ptr++) {
            if(i < strlen(factoryData[ITEM_REGION]))
                *ptr = factoryData[ITEM_REGION][i];
            else
                *ptr = 0x00;
        }
    } 
    if(factoryData[ITEM_COUNTRY][0]) {
        fprintf(stderr, "Found ITEM_COUNTRY (%s), update to buffer...\n", factoryData[ITEM_COUNTRY]);
        ptr = &dataBuf[0] + TARGET_COUNTRY_OFFSET;
        for(i = 0; i < TARGET_COUNTRY_LENGTH; i++, ptr++) {
            if(i < strlen(factoryData[ITEM_COUNTRY]))
                *ptr = factoryData[ITEM_COUNTRY][i];
            else
                *ptr = 0x00;
        }
    } 
    if(factoryData[ITEM_MODEL][0]) {
        fprintf(stderr, "Found ITEM_MODEL   (%s), update to buffer...\n", factoryData[ITEM_MODEL]);
        ptr = &dataBuf[0] + MODEL_NAME_OFFSET;
        for(i = 0; i < MODEL_NAME_LENGTH; i++, ptr++) {
            if(i < strlen(factoryData[ITEM_MODEL]))
                *ptr = factoryData[ITEM_MODEL][i];
            else
                *ptr = 0x00;
        }
    } 
    if(factoryData[ITEM_MAC][0]) {
        fprintf(stderr, "Found ITEM_MAC     (%s), update to buffer...\n", factoryData[ITEM_MAC]);
        ptr = &dataBuf[0] + WIFI_MAC_ADDR_OFFSET;
        unsigned char hwaddr[6];
        ether_atoe(factoryData[ITEM_MAC], hwaddr);
        for(i = 0; i < ETHER_ADDR_LEN; i++, ptr++)
            *ptr = hwaddr[i];
    } 

    /* write to factory */
	int fd = mtd_check_open(FACTORY_PARTITION);
    fprintf(stderr, "Write buffer to %s...\n", FACTORY_PARTITION );
	if(fd < 0) {
		fprintf(stderr, "Could not open mtd device: %s\n", FACTORY_PARTITION);
		return 1;
	}

	if (mtd_erase_block(fd, 0)) {
		fprintf(stderr, "Can't erease block at 0x%x (%s)\n", 0, strerror(errno));
        return 1;
	}

    mtd_write_buffer(fd, dataBuf, 0, mtdsize);
    close(fd);

    if(dataBuf) {
        free(dataBuf);
        dataBuf = NULL;
    }
	return 0;
}

#endif

static void usage(void)
{
	fprintf(stderr, "Usage: mtd [<options> ...] <command> [<arguments> ...] <device>[:<device>...]\n\n"
	"The device is in the format of mtdX (eg: mtd4) or its label.\n"
	"mtd recognizes these commands:\n"
	"        unlock                          unlock the device\n"
	"        refresh                         refresh mtd partition\n"
	"        erase                           erase all data on device\n"
	"        read                            read partition and print to stdout\n"
	"        write <imagefile>|-             write <imagefile> (use - for stdin) to device\n"
	"        factory_get <item>              get value of <item> from factory partition\n"
#if defined (PRODUCT_WeMo_InsightCR)
	"        factory_set \"1;2;3;4;5;6;7;8;9\" set <value> to factory partition\n"
#else
	"        factory_set \"1;2;3;4;5\"       set <value> to factory partition\n"
#endif
	"        crc32                           calculate crc32 of given partition\n"
	"        jffs2write <file>               append <file> to the jffs2 partition on the device\n"
	"        fixtrx                          fix the checksum in a trx header on first boot\n"
	"Following options are available:\n"
	"        -q                              quiet mode (once: no [w] on writing,\n"
	"                                                  twice: no status messages)\n"
	"        -r                              reboot after successful command\n"
	"        -f                              force write without trx checks\n"
	"        -e <device>                     erase <device> before executing the command\n"
	"        -d <name>                       directory for jffs2write, defaults to \"tmp\"\n"
	"        -j <name>                       integrate <file> into jffs2 data when writing an image\n"
	"        -o offset                       offset of the trx header in the partition (for fixtrx)\n"
#ifdef FIS_SUPPORT
	"        -F <part>[:<size>[:<entrypoint>]][,<part>...]\n"
	"                                        alter the fis partition table to create new partitions replacing\n"
	"                                        the partitions provided as argument to the write command\n"
	"                                        (only valid together with the write command)\n"
#endif
	"\n");
    fprintf(stderr, "<item> of factory_get must be one of these:\n");
    int i = 0;
    while(factoryItem[i] != NULL) {
        fprintf(stderr,"         %s\n", factoryItem[i] );
        i++;
    }
    fprintf(stderr, "\n\n");
    fprintf(stderr, "<value> of factory_set must be format like this:\n");
    fprintf(stderr, "         \"");
    i = 0;
    while(factoryItem[i] != NULL) {
        fprintf(stderr,"%s;", factoryItem[i] );
        i++;
    }
    fprintf(stderr, "\"\n\n");
#if defined (PRODUCT_WeMo_InsightCR)
    fprintf(stderr, "         Example 1, set SSID and serial:\n");
    fprintf(stderr, "         mtd factory_set \";WeMo.Bridge.666;;231346B0100666;;;;;;\"\n\n");
    fprintf(stderr, "         Example 2, set country and console disable:\n");
    fprintf(stderr, "         mtd factory_set \";;;;US;;;;0;\"\n\n");
    fprintf(stderr, "         Example 3, set all values:\n");
    fprintf(stderr, "         mtd factory_set \"000B6B123456;WeMo.Bridge.666;WeMo Lighting;231346B0100666;US;1.0.00.006;48SHR1L1.SGC;1A1ACC02006F0D00;0;\"\n\n");
#else
    fprintf(stderr, "         Example 1, set serial and country:\n");
    fprintf(stderr, "         mtd factory_set \"221201K0101AF6;;US;;;\"\n\n");
    fprintf(stderr, "         Example 2, set all values:\n");
    fprintf(stderr, "         mtd factory_set \"221201K0101AF6;0;US;231346B0100666;US;08863B6BF3C8;\"\n\n");
#endif
	fprintf(stderr, "\nExample: To write linux.trx to mtd4 labeled as linux and reboot afterwards\n"
	"         mtd -r write linux.trx linux\n\n");
	exit(1);
}

static void do_reboot(void)
{
	fprintf(stderr, "Rebooting ...\n");
	fflush(stderr);

	/* try regular reboot method first */
	system("/sbin/reboot");
	sleep(2);

	/* if we're still alive at this point, force the kernel to reboot */
	syscall(SYS_reboot,LINUX_REBOOT_MAGIC1,LINUX_REBOOT_MAGIC2,LINUX_REBOOT_CMD_RESTART,NULL);
}

int parseParameter(char *input)
{
    int i = 0;
    int n = 0;
    char *ptr = NULL;
    char delim = ';';

    memset(factoryData, 0, sizeof(factoryData));

    if(!input)
        return 1;

    while(i < strlen(input)) {
        if(input[i] == ';')
            n++;
        i++;
    }

    if(n != ITEM_TOTAL)
        return 1;
    
    i = 0;
    n = 1;
    while(1)
    {
        if(i>0)
            input = ptr+n;
        ptr = strchr(input, delim);
        if(!ptr || i==ITEM_TOTAL)
            break;
        strncpy(factoryData[i], input, ptr-input);
        factoryData[i][ptr-input+1]='\0';
        i++;
    }
    return 0;
}

unsigned long cal_crc32(unsigned long crc, const unsigned char *buf, unsigned int len)
{
    crc = crc ^ 0xffffffffL;
    while (len >= 8)
    {
        DO8(buf);
        len -= 8;
    }
    if (len) do {
        DO1(buf);
    } while (--len);
    return crc ^ 0xffffffffL;
}

void mtd_calculate_crc32(const char* mtd)
{
    if(!dataBuf)
        dataBuf = (char*)malloc(sizeof(char) * mtdsize);

    if(!dataBuf) {
		fprintf(stderr, "Could not alloc data array\n");
		return;
	}

    mtd_read_core(mtd, dataBuf);

    printf("CRC32 of %s:  %lu\n", mtd, cal_crc32(0, dataBuf, mtdsize));

    free(dataBuf);
    dataBuf = NULL;
}

int main (int argc, char **argv)
{
	int ch, i, boot, imagefd = 0, force, unlocked;
	char *erase[MAX_ARGS], *device = NULL, *value = NULL, *parameter = NULL;
	char *fis_layout = NULL;
	enum {
		CMD_ERASE,
		CMD_READ,
		CMD_WRITE,
		CMD_FACTORY_GET,
		CMD_FACTORY_SET,
        CMD_CRC32,
		CMD_UNLOCK,
		CMD_REFRESH,
		CMD_JFFS2WRITE,
		CMD_FIXTRX,
	} cmd = -1;

	erase[0] = NULL;
	boot = 0;
	force = 0;
	buflen = 0;
	quiet = 0;

	while ((ch = getopt(argc, argv,
#ifdef FIS_SUPPORT
			"F:"
#endif
			"frqe:d:j:o:")) != -1)
		switch (ch) {
			case 'f':
				force = 1;
				break;
			case 'r':
				boot = 1;
				break;
			case 'j':
				jffs2file = optarg;
				break;
			case 'q':
				quiet++;
				break;
			case 'e':
				i = 0;
				while ((erase[i] != NULL) && ((i + 1) < MAX_ARGS))
					i++;

				erase[i++] = optarg;
				erase[i] = NULL;
				break;
			case 'd':
				jffs2dir = optarg;
				break;
			case 'o':
				errno = 0;
            gOffset = strtoul(optarg, 0, 0);
				if (errno) {
					fprintf(stderr, "-o: illegal numeric string\n");
					usage();
				}
				break;
#ifdef FIS_SUPPORT
			case 'F':
				fis_layout = optarg;
				break;
#endif
			case '?':
			default:
				usage();
		}
	argc -= optind;
	argv += optind;

	if (argc < 2)
		usage();

	if ((strcmp(argv[0], "unlock") == 0) && (argc == 2)) {
		cmd = CMD_UNLOCK;
		device = argv[1];
	} else if ((strcmp(argv[0], "refresh") == 0) && (argc == 2)) {
		cmd = CMD_REFRESH;
		device = argv[1];
	} else if ((strcmp(argv[0], "erase") == 0) && (argc == 2)) {
		cmd = CMD_ERASE;
		device = argv[1];
	} else if ((strcmp(argv[0], "fixtrx") == 0) && (argc == 2)) {
		cmd = CMD_FIXTRX;
		device = argv[1];
    } else if ((strcmp(argv[0], "read") == 0) && (argc == 2)) {
		cmd = CMD_READ;
		device = argv[1];
	} else if ((strcmp(argv[0], "write") == 0) && (argc == 3)) {
		cmd = CMD_WRITE;
		device = argv[2];

		if (strcmp(argv[1], "-") == 0) {
			imagefile = "<stdin>";
			imagefd = 0;
		} else {
			imagefile = argv[1];
			if ((imagefd = open(argv[1], O_RDONLY)) < 0) {
				fprintf(stderr, "Couldn't open image file: %s!\n", imagefile);
				exit(1);
			}
		}

		if (!mtd_check(device)) {
			fprintf(stderr, "Can't open device for writing!\n");
			exit(1);
		}
		/* check trx file before erasing or writing anything */
		if (!image_check(imagefd, device) && !force) {
			fprintf(stderr, "Image check failed.\n");
			exit(1);
		}
	} else if ((strcmp(argv[0], "factory_get") == 0) && (argc == 2)) {
		cmd = CMD_FACTORY_GET;
        parameter = argv[1];

		if (!mtd_check(FACTORY_PARTITION)) {
			fprintf(stderr, "Can't open %s for writing!\n", FACTORY_PARTITION);
			exit(1);
		}
	} else if ((strcmp(argv[0], "factory_set") == 0) && (argc == 2)) {
		cmd = CMD_FACTORY_SET;
        parameter = argv[1];

		if (!mtd_check(FACTORY_PARTITION)) {
			fprintf(stderr, "Can't open %s for writing!\n", FACTORY_PARTITION);
			exit(1);
		}
        if(parseParameter(parameter)) {
            int i = 0;
            fprintf(stderr, "<value> of factory_set must be format like this:\n");
            fprintf(stderr, "\"");
            while(factoryItem[i] != NULL) {
                fprintf(stderr,"%s;", factoryItem[i] );
                i++;
            }
            fprintf(stderr, "\"\n\n");
#if defined (PRODUCT_WeMo_InsightCR)
            fprintf(stderr, "Example 1, set SSID and serial:\n");
            fprintf(stderr, "mtd factory_set \";WeMo.Bridge.666;;231346B0100666;;;;;;\"\n\n");
            fprintf(stderr, "Example 2, set country and console disable:\n");
            fprintf(stderr, "mtd factory_set \";;;;US;;;;0;\"\n\n");
            fprintf(stderr, "Example 3, set all values:\n");
            fprintf(stderr, "mtd factory_set \"000B6B123456;WeMo.Bridge.666;WeMo Lighting;231346B0100666;US;1.0.00.006;48SHR1L1.SGC;1A1ACC02006F0D00;0;\"\n\n");
#else
            fprintf(stderr, "Example 1, set serial and country:\n");
            fprintf(stderr, "mtd factory_set \"221201K0101AF6;;US;;;\"\n\n");
            fprintf(stderr, "Example 2, set all values:\n");
            fprintf(stderr, "mtd factory_set \"221201K0101AF6;0;US;231346B0100666;US;08863B6BF3C8;\"\n\n");
#endif			
            exit(1);
        }
	} else if ((strcmp(argv[0], "crc32") == 0) && (argc == 2)) {
		cmd = CMD_CRC32;
		device = argv[1];

		if (!mtd_check(device)) {
			fprintf(stderr, "Can't open device for writing!\n");
			exit(1);
		}
	} else if ((strcmp(argv[0], "jffs2write") == 0) && (argc == 3)) {
		cmd = CMD_JFFS2WRITE;
		device = argv[2];

		imagefile = argv[1];
		if (!mtd_check(device)) {
			fprintf(stderr, "Can't open device for writing!\n");
			exit(1);
		}
	} else {
		usage();
	}

	sync();

	i = 0;
	unlocked = 0;
	while (erase[i] != NULL) {
		mtd_unlock(erase[i]);
		mtd_erase(erase[i]);
		if (strcmp(erase[i], device) == 0)
			unlocked = 1;
		i++;
	}

	switch (cmd) {
		case CMD_UNLOCK:
			if (!unlocked)
				mtd_unlock(device);
			break;
		case CMD_ERASE:
			if (!unlocked)
				mtd_unlock(device);
			mtd_erase(device);
			break;
        case CMD_READ:
			if (!unlocked)
				mtd_unlock(device);
			mtd_read(device);
            break;
		case CMD_WRITE:
			if (!unlocked)
				mtd_unlock(device);
			mtd_write(imagefd, device, fis_layout);
			break;
		case CMD_FACTORY_GET:
			if (!unlocked)
				mtd_unlock(FACTORY_PARTITION);
			mtd_factory_get(parameter);
			break;
		case CMD_FACTORY_SET:
			if (!unlocked)
				mtd_unlock(FACTORY_PARTITION);
			mtd_factory_set(parameter);
			break;
        case CMD_CRC32:
			if (!unlocked)
				mtd_unlock(device);
			mtd_calculate_crc32(device);
            break;
		case CMD_JFFS2WRITE:
			if (!unlocked)
				mtd_unlock(device);
			mtd_write_jffs2(device, imagefile, jffs2dir);
			break;
		case CMD_REFRESH:
			mtd_refresh(device);
			break;
		case CMD_FIXTRX:
         mtd_fixtrx(device, gOffset);
			break;
	}

	sync();

	if (boot)
		do_reboot();

	return 0;
}

