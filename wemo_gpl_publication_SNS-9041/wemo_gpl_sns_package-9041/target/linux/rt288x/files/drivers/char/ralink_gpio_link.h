/*
 ***************************************************************************
 * Ralink Tech Inc.
 * 4F, No. 2 Technology 5th Rd.
 * Science-based Industrial Park
 * Hsin-chu, Taiwan, R.O.C.
 *
 * (c) Copyright, Ralink Technology, Inc.
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *  THIS  SOFTWARE  IS PROVIDED   ``AS  IS'' AND   ANY  EXPRESS OR IMPLIED
 *  WARRANTIES,   INCLUDING, BUT NOT  LIMITED  TO, THE IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN
 *  NO  EVENT  SHALL   THE AUTHOR  BE    LIABLE FOR ANY   DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 *  NOT LIMITED   TO, PROCUREMENT OF  SUBSTITUTE GOODS  OR SERVICES; LOSS OF
 *  USE, DATA,  OR PROFITS; OR  BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 *  ANY THEORY OF LIABILITY, WHETHER IN  CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 *  THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  You should have received a copy of the  GNU General Public License along
 *  with this program; if not, write  to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 *
 *
 ***************************************************************************
 */

#ifndef __RALINK_GPIO_LINK_H__
#define __RALINK_GPIO_LINK_H__

#include <asm/rt2880/rt_mmap.h>

#define PLUGIN_BOARD_NEW			1

#define RALINK_GPIO_HAS_5124		1
#define RALINK_GPIO_LED_LOW_ACT		1

/*
 * ioctl commands
 */
#define	RALINK_GPIO_SET_DIR		0x01
#define RALINK_GPIO_SET_DIR_IN		0x11
#define RALINK_GPIO_SET_DIR_OUT		0x12
#define	RALINK_GPIO_READ		0x02
#define	RALINK_GPIO_WRITE		0x03
#define	RALINK_GPIO_SET			0x21
#define	RALINK_GPIO_CLEAR		0x31
#define	RALINK_GPIO_READ_BIT		0x04
#define	RALINK_GPIO_WRITE_BIT		0x05
#define	RALINK_GPIO_READ_BYTE		0x06
#define	RALINK_GPIO_WRITE_BYTE		0x07
#define	RALINK_GPIO_READ_INT		0x02 //same as read
#define	RALINK_GPIO_WRITE_INT		0x03 //same as write
#define	RALINK_GPIO_SET_INT		0x21 //same as set
#define	RALINK_GPIO_CLEAR_INT		0x31 //same as clear
#define RALINK_GPIO_ENABLE_INTP		0x08
#define RALINK_GPIO_DISABLE_INTP	0x09
#define RALINK_GPIO_REG_IRQ		0x0A
#define RALINK_GPIO_LED_SET		0x41
#define RALINK_GENERAL_READ		0x50
//WNC0_Ruei, 20130801
#define  RALINK_GPIO_RESET_EM357	0x57
//WNC0

#if defined (RALINK_GPIO_HAS_2722)

#define	RALINK_GPIO2722_SET_DIR		0x51
#define RALINK_GPIO2722_SET_DIR_IN	0x13
#define RALINK_GPIO2722_SET_DIR_OUT	0x14
#define	RALINK_GPIO2722_READ		0x52
#define	RALINK_GPIO2722_WRITE		0x53
#define	RALINK_GPIO2722_SET		0x22
#define	RALINK_GPIO2722_CLEAR		0x32

#elif defined (RALINK_GPIO_HAS_4524)

#define	RALINK_GPIO3924_SET_DIR		0x51
#define RALINK_GPIO3924_SET_DIR_IN	0x13
#define RALINK_GPIO3924_SET_DIR_OUT	0x14
#define	RALINK_GPIO3924_READ		0x52
#define	RALINK_GPIO3924_WRITE		0x53
#define	RALINK_GPIO3924_SET		0x22
#define	RALINK_GPIO3924_CLEAR		0x32

#define	RALINK_GPIO4540_SET_DIR		0x61
#define RALINK_GPIO4540_SET_DIR_IN	0x15
#define RALINK_GPIO4540_SET_DIR_OUT	0x16
#define	RALINK_GPIO4540_READ		0x62
#define	RALINK_GPIO4540_WRITE		0x63
#define	RALINK_GPIO4540_SET		0x23
#define	RALINK_GPIO4540_CLEAR		0x33

#elif defined (RALINK_GPIO_HAS_5124)

#define	RALINK_GPIO3924_SET_DIR		0x51
#define RALINK_GPIO3924_SET_DIR_IN	0x13
#define RALINK_GPIO3924_SET_DIR_OUT	0x14
#define	RALINK_GPIO3924_READ		0x52
#define	RALINK_GPIO3924_WRITE		0x53
#define	RALINK_GPIO3924_SET		0x22
#define	RALINK_GPIO3924_CLEAR		0x32

#define	RALINK_GPIO5140_SET_DIR		0x61
#define RALINK_GPIO5140_SET_DIR_IN	0x15
#define RALINK_GPIO5140_SET_DIR_OUT	0x16
#define	RALINK_GPIO5140_READ		0x62
#define	RALINK_GPIO5140_WRITE		0x63
#define	RALINK_GPIO5140_SET		0x23
#define	RALINK_GPIO5140_CLEAR		0x33

#elif defined (RALINK_GPIO_HAS_9524) || defined (RALINK_GPIO_HAS_7224)

#define	RALINK_GPIO3924_SET_DIR		0x51
#define RALINK_GPIO3924_SET_DIR_IN	0x13
#define RALINK_GPIO3924_SET_DIR_OUT	0x14
#define	RALINK_GPIO3924_READ		0x52
#define	RALINK_GPIO3924_WRITE		0x53
#define	RALINK_GPIO3924_SET		0x22
#define	RALINK_GPIO3924_CLEAR		0x32

#define	RALINK_GPIO7140_SET_DIR		0x61
#define RALINK_GPIO7140_SET_DIR_IN	0x15
#define RALINK_GPIO7140_SET_DIR_OUT	0x16
#define	RALINK_GPIO7140_READ		0x62
#define	RALINK_GPIO7140_WRITE		0x63
#define	RALINK_GPIO7140_SET		0x23
#define	RALINK_GPIO7140_CLEAR		0x33

#if defined (RALINK_GPIO_HAS_7224)
#define	RALINK_GPIO72_SET_DIR		0x71
#define RALINK_GPIO72_SET_DIR_IN	0x17
#define RALINK_GPIO72_SET_DIR_OUT	0x18
#define	RALINK_GPIO72_READ		0x72
#define	RALINK_GPIO72_WRITE		0x73
#define	RALINK_GPIO72_SET		0x24
#define	RALINK_GPIO72_CLEAR		0x34
#else
#define	RALINK_GPIO9572_SET_DIR		0x71
#define RALINK_GPIO9572_SET_DIR_IN	0x17
#define RALINK_GPIO9572_SET_DIR_OUT	0x18
#define	RALINK_GPIO9572_READ		0x72
#define	RALINK_GPIO9572_WRITE		0x73
#define	RALINK_GPIO9572_SET		0x24
#define	RALINK_GPIO9572_CLEAR		0x34
#endif

#elif defined (RALINK_GPIO_HAS_9532)

#define	RALINK_GPIO6332_SET_DIR		0x51
#define RALINK_GPIO6332_SET_DIR_IN	0x13
#define RALINK_GPIO6332_SET_DIR_OUT	0x14
#define	RALINK_GPIO6332_READ		0x52
#define	RALINK_GPIO6332_WRITE		0x53
#define	RALINK_GPIO6332_SET		0x22
#define	RALINK_GPIO6332_CLEAR		0x32

#define	RALINK_GPIO9564_SET_DIR		0x61
#define RALINK_GPIO9564_SET_DIR_IN	0x15
#define RALINK_GPIO9564_SET_DIR_OUT	0x16
#define	RALINK_GPIO9564_READ		0x62
#define	RALINK_GPIO9564_WRITE		0x63
#define	RALINK_GPIO9564_SET		0x23
#define	RALINK_GPIO9564_CLEAR		0x33

#endif

/*
 * Address of RALINK_ Registers
 */
#define RALINK_SYSCTL_ADDR		RALINK_SYSCTL_BASE	// system control
#define RALINK_REG_GPIOMODE		(RALINK_SYSCTL_ADDR + 0x60)
#define RALINK_REG_PULLEN		(RALINK_SYSCTL_ADDR + 0x14)
#define RALINK_IRQ_ADDR			RALINK_INTCL_BASE
#define RALINK_PRGIO_ADDR		RALINK_PIO_BASE // Programmable I/O

#if defined (CONFIG_RALINK_MT7621)
#define RALINK_REG_INTENA		(RALINK_IRQ_ADDR   + 0x80)
#define RALINK_REG_INTDIS		(RALINK_IRQ_ADDR   + 0x78)

#define RALINK_REG_PIOINT		(RALINK_PRGIO_ADDR + 0x90)
#define RALINK_REG_PIOEDGE		(RALINK_PRGIO_ADDR + 0xA0)
#define RALINK_REG_PIORENA		(RALINK_PRGIO_ADDR + 0x50)
#define RALINK_REG_PIOFENA		(RALINK_PRGIO_ADDR + 0x60)
#define RALINK_REG_PIODATA		(RALINK_PRGIO_ADDR + 0x20)
#define RALINK_REG_PIODIR		(RALINK_PRGIO_ADDR + 0x00)
#define RALINK_REG_PIOSET		(RALINK_PRGIO_ADDR + 0x30)
#define RALINK_REG_PIORESET		(RALINK_PRGIO_ADDR + 0x40)
#else
#define RALINK_REG_INTENA		(RALINK_IRQ_ADDR   + 0x34)
#define RALINK_REG_INTDIS		(RALINK_IRQ_ADDR   + 0x38)

#define RALINK_REG_PIOINT		(RALINK_PRGIO_ADDR + 0x00)
#define RALINK_REG_PIOEDGE		(RALINK_PRGIO_ADDR + 0x04)
#define RALINK_REG_PIORENA		(RALINK_PRGIO_ADDR + 0x08)
#define RALINK_REG_PIOFENA		(RALINK_PRGIO_ADDR + 0x0C)
#define RALINK_REG_PIODATA		(RALINK_PRGIO_ADDR + 0x20)
#define RALINK_REG_PIODIR		(RALINK_PRGIO_ADDR + 0x24)
#define RALINK_REG_PIOSET		(RALINK_PRGIO_ADDR + 0x2C)
#define RALINK_REG_PIORESET		(RALINK_PRGIO_ADDR + 0x30)
#define RALINK_REG_PIOTOGGLE		(RALINK_PRGIO_ADDR + 0x34)
#endif

#if defined (RALINK_GPIO_HAS_2722)

#define RALINK_REG_PIO2722INT		(RALINK_PRGIO_ADDR + 0x60)
#define RALINK_REG_PIO2722EDGE		(RALINK_PRGIO_ADDR + 0x64)
#define RALINK_REG_PIO2722RENA		(RALINK_PRGIO_ADDR + 0x68)
#define RALINK_REG_PIO2722FENA		(RALINK_PRGIO_ADDR + 0x6C)
#define RALINK_REG_PIO2722DATA		(RALINK_PRGIO_ADDR + 0x70)
#define RALINK_REG_PIO2722DIR		(RALINK_PRGIO_ADDR + 0x74)
#define RALINK_REG_PIO2722SET		(RALINK_PRGIO_ADDR + 0x7C)
#define RALINK_REG_PIO2722RESET		(RALINK_PRGIO_ADDR + 0x80)
#define RALINK_REG_PIO2722TOGGLE	(RALINK_PRGIO_ADDR + 0x84)

#elif defined (RALINK_GPIO_HAS_4524)

#define RALINK_REG_PIO3924INT		(RALINK_PRGIO_ADDR + 0x38)
#define RALINK_REG_PIO3924EDGE		(RALINK_PRGIO_ADDR + 0x3C)
#define RALINK_REG_PIO3924RENA		(RALINK_PRGIO_ADDR + 0x40)
#define RALINK_REG_PIO3924FENA		(RALINK_PRGIO_ADDR + 0x44)
#define RALINK_REG_PIO3924DATA		(RALINK_PRGIO_ADDR + 0x48)
#define RALINK_REG_PIO3924DIR		(RALINK_PRGIO_ADDR + 0x4C)
#define RALINK_REG_PIO3924SET		(RALINK_PRGIO_ADDR + 0x54)
#define RALINK_REG_PIO3924RESET		(RALINK_PRGIO_ADDR + 0x58)
#define RALINK_REG_PIO3924TOGGLE	(RALINK_PRGIO_ADDR + 0x5C)

#define RALINK_REG_PIO4540INT		(RALINK_PRGIO_ADDR + 0x60)
#define RALINK_REG_PIO4540EDGE		(RALINK_PRGIO_ADDR + 0x64)
#define RALINK_REG_PIO4540RENA		(RALINK_PRGIO_ADDR + 0x68)
#define RALINK_REG_PIO4540FENA		(RALINK_PRGIO_ADDR + 0x6C)
#define RALINK_REG_PIO4540DATA		(RALINK_PRGIO_ADDR + 0x70)
#define RALINK_REG_PIO4540DIR		(RALINK_PRGIO_ADDR + 0x74)
#define RALINK_REG_PIO4540SET		(RALINK_PRGIO_ADDR + 0x7C)
#define RALINK_REG_PIO4540RESET		(RALINK_PRGIO_ADDR + 0x80)
#define RALINK_REG_PIO4540TOGGLE	(RALINK_PRGIO_ADDR + 0x84)

#elif defined (RALINK_GPIO_HAS_5124)

#define RALINK_REG_PIO3924INT		(RALINK_PRGIO_ADDR + 0x38)
#define RALINK_REG_PIO3924EDGE		(RALINK_PRGIO_ADDR + 0x3C)
#define RALINK_REG_PIO3924RENA		(RALINK_PRGIO_ADDR + 0x40)
#define RALINK_REG_PIO3924FENA		(RALINK_PRGIO_ADDR + 0x44)
#define RALINK_REG_PIO3924DATA		(RALINK_PRGIO_ADDR + 0x48)
#define RALINK_REG_PIO3924DIR		(RALINK_PRGIO_ADDR + 0x4C)
#define RALINK_REG_PIO3924SET		(RALINK_PRGIO_ADDR + 0x54)
#define RALINK_REG_PIO3924RESET		(RALINK_PRGIO_ADDR + 0x58)
#define RALINK_REG_PIO3924TOGGLE	(RALINK_PRGIO_ADDR + 0x5C)

#define RALINK_REG_PIO5140INT		(RALINK_PRGIO_ADDR + 0x60)
#define RALINK_REG_PIO5140EDGE		(RALINK_PRGIO_ADDR + 0x64)
#define RALINK_REG_PIO5140RENA		(RALINK_PRGIO_ADDR + 0x68)
#define RALINK_REG_PIO5140FENA		(RALINK_PRGIO_ADDR + 0x6C)
#define RALINK_REG_PIO5140DATA		(RALINK_PRGIO_ADDR + 0x70)
#define RALINK_REG_PIO5140DIR		(RALINK_PRGIO_ADDR + 0x74)
#define RALINK_REG_PIO5140SET		(RALINK_PRGIO_ADDR + 0x7C)
#define RALINK_REG_PIO5140RESET		(RALINK_PRGIO_ADDR + 0x80)
#define RALINK_REG_PIO5140TOGGLE	(RALINK_PRGIO_ADDR + 0x84)

#elif defined (RALINK_GPIO_HAS_7224)

#define RALINK_REG_PIO3924INT		(RALINK_PRGIO_ADDR + 0x38)
#define RALINK_REG_PIO3924EDGE		(RALINK_PRGIO_ADDR + 0x3C)
#define RALINK_REG_PIO3924RENA		(RALINK_PRGIO_ADDR + 0x40)
#define RALINK_REG_PIO3924FENA		(RALINK_PRGIO_ADDR + 0x44)
#define RALINK_REG_PIO3924DATA		(RALINK_PRGIO_ADDR + 0x48)
#define RALINK_REG_PIO3924DIR		(RALINK_PRGIO_ADDR + 0x4C)
#define RALINK_REG_PIO3924POL		(RALINK_PRGIO_ADDR + 0x50)
#define RALINK_REG_PIO3924SET		(RALINK_PRGIO_ADDR + 0x54)
#define RALINK_REG_PIO3924RESET		(RALINK_PRGIO_ADDR + 0x58)
#define RALINK_REG_PIO3924TOGGLE	(RALINK_PRGIO_ADDR + 0x5C)

#define RALINK_REG_PIO7140INT		(RALINK_PRGIO_ADDR + 0x60)
#define RALINK_REG_PIO7140EDGE		(RALINK_PRGIO_ADDR + 0x64)
#define RALINK_REG_PIO7140RENA		(RALINK_PRGIO_ADDR + 0x68)
#define RALINK_REG_PIO7140FENA		(RALINK_PRGIO_ADDR + 0x6C)
#define RALINK_REG_PIO7140DATA		(RALINK_PRGIO_ADDR + 0x70)
#define RALINK_REG_PIO7140DIR		(RALINK_PRGIO_ADDR + 0x74)
#define RALINK_REG_PIO7140POL		(RALINK_PRGIO_ADDR + 0x78)
#define RALINK_REG_PIO7140SET		(RALINK_PRGIO_ADDR + 0x7C)
#define RALINK_REG_PIO7140RESET		(RALINK_PRGIO_ADDR + 0x80)
#define RALINK_REG_PIO7140TOGGLE	(RALINK_PRGIO_ADDR + 0x84)

#define RALINK_REG_PIO72INT		(RALINK_PRGIO_ADDR + 0x88)
#define RALINK_REG_PIO72EDGE		(RALINK_PRGIO_ADDR + 0x8C)
#define RALINK_REG_PIO72RENA		(RALINK_PRGIO_ADDR + 0x90)
#define RALINK_REG_PIO72FENA		(RALINK_PRGIO_ADDR + 0x94)
#define RALINK_REG_PIO72DATA		(RALINK_PRGIO_ADDR + 0x98)
#define RALINK_REG_PIO72DIR		(RALINK_PRGIO_ADDR + 0x9C)
#define RALINK_REG_PIO72POL		(RALINK_PRGIO_ADDR + 0xA0)
#define RALINK_REG_PIO72SET		(RALINK_PRGIO_ADDR + 0xA4)
#define RALINK_REG_PIO72RESET		(RALINK_PRGIO_ADDR + 0xA8)
#define RALINK_REG_PIO72TOGGLE		(RALINK_PRGIO_ADDR + 0xAC)

#elif defined (RALINK_GPIO_HAS_9524)

#define RALINK_REG_PIO3924INT		(RALINK_PRGIO_ADDR + 0x38)
#define RALINK_REG_PIO3924EDGE		(RALINK_PRGIO_ADDR + 0x3C)
#define RALINK_REG_PIO3924RENA		(RALINK_PRGIO_ADDR + 0x40)
#define RALINK_REG_PIO3924FENA		(RALINK_PRGIO_ADDR + 0x44)
#define RALINK_REG_PIO3924DATA		(RALINK_PRGIO_ADDR + 0x48)
#define RALINK_REG_PIO3924DIR		(RALINK_PRGIO_ADDR + 0x4C)
#define RALINK_REG_PIO3924SET		(RALINK_PRGIO_ADDR + 0x54)
#define RALINK_REG_PIO3924RESET		(RALINK_PRGIO_ADDR + 0x58)
#define RALINK_REG_PIO3924TOGGLE	(RALINK_PRGIO_ADDR + 0x5C)

#define RALINK_REG_PIO7140INT		(RALINK_PRGIO_ADDR + 0x60)
#define RALINK_REG_PIO7140EDGE		(RALINK_PRGIO_ADDR + 0x64)
#define RALINK_REG_PIO7140RENA		(RALINK_PRGIO_ADDR + 0x68)
#define RALINK_REG_PIO7140FENA		(RALINK_PRGIO_ADDR + 0x6C)
#define RALINK_REG_PIO7140DATA		(RALINK_PRGIO_ADDR + 0x70)
#define RALINK_REG_PIO7140DIR		(RALINK_PRGIO_ADDR + 0x74)
#define RALINK_REG_PIO7140SET		(RALINK_PRGIO_ADDR + 0x7C)
#define RALINK_REG_PIO7140RESET		(RALINK_PRGIO_ADDR + 0x80)
#define RALINK_REG_PIO7140TOGGLE	(RALINK_PRGIO_ADDR + 0x84)

#define RALINK_REG_PIO9572INT		(RALINK_PRGIO_ADDR + 0x88)
#define RALINK_REG_PIO9572EDGE		(RALINK_PRGIO_ADDR + 0x8C)
#define RALINK_REG_PIO9572RENA		(RALINK_PRGIO_ADDR + 0x90)
#define RALINK_REG_PIO9572FENA		(RALINK_PRGIO_ADDR + 0x94)
#define RALINK_REG_PIO9572DATA		(RALINK_PRGIO_ADDR + 0x98)
#define RALINK_REG_PIO9572DIR		(RALINK_PRGIO_ADDR + 0x9C)
#define RALINK_REG_PIO9572SET		(RALINK_PRGIO_ADDR + 0xA0)
#define RALINK_REG_PIO9572RESET		(RALINK_PRGIO_ADDR + 0xA4)
#define RALINK_REG_PIO9572TOGGLE	(RALINK_PRGIO_ADDR + 0xA8)

#elif defined (RALINK_GPIO_HAS_9532)

#define RALINK_REG_PIO6332INT		(RALINK_PRGIO_ADDR + 0x94)
#define RALINK_REG_PIO6332EDGE		(RALINK_PRGIO_ADDR + 0xA4)
#define RALINK_REG_PIO6332RENA		(RALINK_PRGIO_ADDR + 0x54)
#define RALINK_REG_PIO6332FENA		(RALINK_PRGIO_ADDR + 0x64)
#define RALINK_REG_PIO6332DATA		(RALINK_PRGIO_ADDR + 0x24)
#define RALINK_REG_PIO6332DIR		(RALINK_PRGIO_ADDR + 0x04)
#define RALINK_REG_PIO6332SET		(RALINK_PRGIO_ADDR + 0x34)
#define RALINK_REG_PIO6332RESET		(RALINK_PRGIO_ADDR + 0x44)

#define RALINK_REG_PIO9564INT		(RALINK_PRGIO_ADDR + 0x98)
#define RALINK_REG_PIO9564EDGE		(RALINK_PRGIO_ADDR + 0xA8)
#define RALINK_REG_PIO9564RENA		(RALINK_PRGIO_ADDR + 0x58)
#define RALINK_REG_PIO9564FENA		(RALINK_PRGIO_ADDR + 0x68)
#define RALINK_REG_PIO9564DATA		(RALINK_PRGIO_ADDR + 0x28)
#define RALINK_REG_PIO9564DIR		(RALINK_PRGIO_ADDR + 0x08)
#define RALINK_REG_PIO9564SET		(RALINK_PRGIO_ADDR + 0x38)
#define RALINK_REG_PIO9564RESET		(RALINK_PRGIO_ADDR + 0x48)

#endif

/*
 * Values for the GPIOMODE Register
 */
#if defined (CONFIG_RALINK_RT2880)
#define RALINK_GPIOMODE_I2C		0x01
#define RALINK_GPIOMODE_UARTF		0x02
#define RALINK_GPIOMODE_SPI		0x04
#define RALINK_GPIOMODE_UARTL		0x08
#define RALINK_GPIOMODE_JTAG		0x10
#define RALINK_GPIOMODE_MDIO		0x20
#define RALINK_GPIOMODE_SDRAM		0x40
#define RALINK_GPIOMODE_PCI		0x80

#elif defined (CONFIG_RALINK_RT3052) || defined (CONFIG_RALINK_RT2883)

#define RALINK_GPIOMODE_I2C		0x01
#define RALINK_GPIOMODE_SPI		0x02
#define RALINK_GPIOMODE_UARTF		0x1C
#define RALINK_GPIOMODE_UARTL		0x20
#define RALINK_GPIOMODE_JTAG		0x40
#define RALINK_GPIOMODE_MDIO		0x80
#define RALINK_GPIOMODE_SDRAM		0x100
#define RALINK_GPIOMODE_RGMII		0x200

#elif defined (CONFIG_RALINK_RT3352)

#define RALINK_GPIOMODE_I2C		0x01
#define RALINK_GPIOMODE_SPI		0x02
#define RALINK_GPIOMODE_UARTF		0x1C
#define RALINK_GPIOMODE_UARTL		0x20
#define RALINK_GPIOMODE_JTAG		0x40
#define RALINK_GPIOMODE_MDIO		0x80
#define RALINK_GPIOMODE_GE1		0x200
#define RALINK_GPIOMODE_EPHY		0x4000
#define RALINK_GPIOMODE_LNA_G		0x40000
#define RALINK_GPIOMODE_PA_G		0x100000
#define RALINK_GPIOMODE_SPI_CS1		0x400000

#elif defined (CONFIG_RALINK_RT5350)

#define RALINK_GPIOMODE_I2C		0x01
#define RALINK_GPIOMODE_SPI		0x02
//WNC0_tsunghao Mon 09 Sep 2013 06:56:50 PM CST, customized define
#if WNC_DEL
#define RALINK_GPIOMODE_UARTF		0x1C
#endif
#define RALINK_GPIOMODE_UARTF		0x15
//WNC0
#define RALINK_GPIOMODE_UARTL		0x20
#define RALINK_GPIOMODE_JTAG		0x40
#define RALINK_GPIOMODE_EPHY		0x4000
#define RALINK_GPIOMODE_SPI_CS1		0x400000

#elif defined (CONFIG_RALINK_RT3883)

#define RALINK_GPIOMODE_I2C		0x01
#define RALINK_GPIOMODE_SPI		0x02
#define RALINK_GPIOMODE_UARTF		0x1C
#define RALINK_GPIOMODE_UARTL		0x20
#define RALINK_GPIOMODE_JTAG		0x40
#define RALINK_GPIOMODE_MDIO		0x80
#define RALINK_GPIOMODE_GE1		0x200
#define RALINK_GPIOMODE_GE2		0x400
#define RALINK_GPIOMODE_PCI		0x1800
#define RALINK_GPIOMODE_LNA_A		0x30000
#define RALINK_GPIOMODE_LNA_G		0xC0000

#elif defined (CONFIG_RALINK_RT6855)

#define RALINK_GPIOMODE_I2C		0x01
#define RALINK_GPIOMODE_UARTF		0x1C
#define RALINK_GPIOMODE_UARTL		0x20
#define RALINK_GPIOMODE_JTAG		0x40
#define RALINK_GPIOMODE_MDIO		0x80
#define RALINK_GPIOMODE_GE1		0x200
#define RALINK_GPIOMODE_NAND_SPI	0x3800
#define RALINK_GPIOMODE_EPHY		0x4000
#define RALINK_GPIOMODE_PERST		0x20000
//#define RALINK_GPIOMODE_SPI_CS1		0xC00000
//#define RALINK_GPIOMODE_SUTIF_SHARE	0x10000000

#elif defined (CONFIG_RALINK_MT7620)

#define RALINK_GPIOMODE_I2C		0x01
#define RALINK_GPIOMODE_UARTF		0x1C
#define RALINK_GPIOMODE_UARTL		0x20
//#define RALINK_GPIOMODE_JTAG		0x40
#define RALINK_GPIOMODE_MDIO		0x180
#define RALINK_GPIOMODE_GE1		0x200
#define RALINK_GPIOMODE_GE2		0x400
#define RALINK_GPIOMODE_SPI		0x800
#define RALINK_GPIOMODE_SPI_REFCLK	0x1000
#define RALINK_GPIOMODE_WLED		0x2000
#define RALINK_GPIOMODE_EPHY		0x8000
#define RALINK_GPIOMODE_PERST		0x20000
#define RALINK_GPIOMODE_ND_SD		0x80000
#define RALINK_GPIOMODE_PA_G		0x100000
#define RALINK_GPIOMODE_WDT		0x400000


#elif defined (CONFIG_RALINK_MT7621)

#define RALINK_GPIOMODE_I2C		0x01
#define RALINK_GPIOMODE_UARTF		0x1C
#define RALINK_GPIOMODE_UARTL		0x20
#define RALINK_GPIOMODE_JTAG		0x40
#define RALINK_GPIOMODE_MDIO		0x80
#define RALINK_GPIOMODE_GE1		0x200
#define RALINK_GPIOMODE_GE2		0x400
#define RALINK_GPIOMODE_SPI		0x800
#define RALINK_GPIOMODE_SPI_REFCLK	0x1000
#define RALINK_GPIOMODE_WLED		0x2000
#define RALINK_GPIOMODE_EPHY		0x8000
#define RALINK_GPIOMODE_PA_G		0x100000

#else
#error Please Choose System Type
#endif

// if you would like to enable GPIO mode for other pins, please modify this value
// !! Warning: changing this value may make other features(MDIO, PCI, etc) lose efficacy
#if defined (CONFIG_RALINK_MT7621) //FIXME
#define RALINK_GPIOMODE_DFT		(RALINK_GPIOMODE_I2C | RALINK_GPIOMODE_PA_G) 
#elif defined (CONFIG_RALINK_MT7620) //FIXME
#define RALINK_GPIOMODE_DFT		(RALINK_GPIOMODE_I2C) | (RALINK_GPIOMODE_UARTF)
#else
#define RALINK_GPIOMODE_DFT		(RALINK_GPIOMODE_UARTF)
#endif

/*
 * bit is the unit of length
 */
#if defined (RALINK_GPIO_HAS_2722)
#define RALINK_GPIO_NUMBER		28
#elif defined (RALINK_GPIO_HAS_4524)
#define RALINK_GPIO_NUMBER		46
#elif defined (RALINK_GPIO_HAS_5124)
#define RALINK_GPIO_NUMBER		52
#elif defined (RALINK_GPIO_HAS_7224)
#define RALINK_GPIO_NUMBER		73
#elif defined (RALINK_GPIO_HAS_9524)
#define RALINK_GPIO_NUMBER		96
#elif defined (RALINK_GPIO_HAS_9532)
#define RALINK_GPIO_NUMBER		73
#else
#define RALINK_GPIO_NUMBER		24
#endif


#if defined (CONFIG_RALINK_MT7621)
#define RALINK_GPIO_DATA_MASK		0xFFFFFFFF
#define RALINK_GPIO_DATA_LEN		24
#define RALINK_GPIO_DIR_IN		0
#define RALINK_GPIO_DIR_OUT		1
#define RALINK_GPIO_DIR_ALLIN		0
#define RALINK_GPIO_DIR_ALLOUT		0xFFFFFFFF
#else
#define RALINK_GPIO_DATA_MASK		0x00FFFFFF
#define RALINK_GPIO_DATA_LEN		24
#define RALINK_GPIO_DIR_IN		0
#define RALINK_GPIO_DIR_OUT		1
#define RALINK_GPIO_DIR_ALLIN		0
#define RALINK_GPIO_DIR_ALLOUT		0x00FFFFFF
#endif

/*
 * structure used at regsitration
 */
typedef struct {
	unsigned int irq;		//request irq pin number
	pid_t pid;			//process id to notify
} ralink_gpio_reg_info;

#define RALINK_GPIO_LED_INFINITY	4000
typedef struct {
	int gpio;			//gpio number (0 ~ 23)
	unsigned int on;		//interval of led on
	unsigned int off;		//interval of led off
	unsigned int blinks;		//number of blinking cycles
	unsigned int rests;		//number of break cycles
	unsigned int times;		//blinking times
} ralink_gpio_led_info;

#define SUPPORT_RESET_BUTTON		1
// Daniel Changed _start
/*
#define SUPPORT_WPS_BUTTON      1
#define SUPPORT_WPS_LED         1
#define SUPPORT_GEMTEK_MP_TEST  1
*/
#define SUPPORT_WPS_BUTTON      0
#define SUPPORT_WPS_LED         0
#define SUPPORT_GEMTEK_MP_TEST  0
#define SUPPORT_MOTION_SENSOR   0
// Daniel Changed _end

#define GPIO_STAT_OUT_LOW       0
#define GPIO_STAT_OUT_HIGH      1
#define GPIO_STAT_IN_LOW        2
#define GPIO_STAT_IN_HIGH       3

/*First GPIO Register GPIO0~GPIO21 can be used*/
#define PORT0				0x00000001	
#define PORT1				0x00000002
#define PORT2				0x00000004
#define PORT3				0x00000008
#define PORT4				0x00000010	
#define PORT5				0x00000020	
#define PORT6				0x00000040	
#define PORT7				0x00000080 //Watch Dog Input
#define PORT8				0x00000100	
#define PORT9				0x00000200	
#define PORT10			0x00000400 //SW Reset	
#define PORT11			0x00000800 //Watch Dog Enable	
#define PORT12			0x00001000	
#define PORT13			0x00002000 //WPS PBC	
#define PORT14			0x00004000	
#define PORT15			0x00008000
#define PORT16			0x00010000
#define PORT17			0x00020000	
#define PORT18			0x00040000	
#define PORT19			0x00080000
#define PORT20			0x00100000	
#define PORT21			0x00200000

/*Second GPIO Register GPI22~GPIO27 can be used*/	
#define PORT22			0x00000001 //Small Green LED
#define PORT23 	    0x00000002
#define PORT24      0x00000004 //Big Amber LED
#define PORT25      0x00000008 //Small Amber LED
#define PORT26      0x00000010 //Big Green LED 
#define PORT27      0x00000020

#define RALINK_GPIO(x)			(1 << x)

// Daniel Changed _start
/*
#define WATCH_DOG_INPUT  PORT7
#define WATCH_DOG_ENABLE PORT11
#define RESET_BUTTON     PORT10
#define WPS_BUTTON       PORT13

#define SMALL_GREEN_LED  PORT22
#define BIG_AMBER_LED    PORT24
#define SMALL_AMBER_LED  PORT25
#define BIG_GREEN_LED    PORT26

 //#define WIFI_AMBER_LED	PORT7
//#define WIFI_BLUE_LED	PORT11
#define WIFI_AMBER_LED    PORT11
#define WIFI_BLUE_LED     PORT7
#define POWER_BLUE_LED    PORT9
#define POWER_BUTTON    PORT12
#define RELAY_CONTROL	PORT13
//#define RESET_BUTTON    PORT10
#define MOTION_SENSOR	PORT14
*/

#define RESET_BUTTON     PORT0
#define WPS_BUTTON       PORT14

#define SMALL_GREEN_LED  PORT22
#define BIG_AMBER_LED    PORT24
#define SMALL_AMBER_LED  PORT25
#define BIG_GREEN_LED    PORT26

#define WIFI_AMBER_LED	PORT1
#define WIFI_BLUE_LED	PORT13
#define POWER_BLUE_LED  PORT13
#define POWER_BUTTON    PORT14
#define RELAY_CONTROL	PORT14
#define MOTION_SENSOR	PORT14

#define AMBER_LED_ON    1
#define AMBER_LED_OFF   0
#define BLUE_LED_ON    0
#define BLUE_LED_OFF   1
// Daniel Changed _end

#define RESET_TIME			 10
#define LED_ON            0
#define LED_OFF           1

#define WatchD_ON         1
#define WatchD_OFF        0

#define GMTK_CLOSE_SPEED_METER 0
#define GMTK_SYSTEM_READY 1
#define GMTK_SYSTEM_BOOT 2
#define GMTK_SPEED_METER_BOOT_UP_READY 3
#define GMTK_SPEED_METER_BOOT_UP 4
#define GMTK_WL_SEC_DISABLED 5
#define GMTK_WL_SEC_ENABLED 6
#define GMTK_LAN_LINK_DOWN 7
#define GMTK_LAN_LINK_UP 8
#define GMTK_LAN_LINK_ERROR 9
#define GMTK_WAN_MODEM_DOWN 10
#define GMTK_WAN_MODEM_UP 11
#define GMTK_WAN_MODEM_ERROR 12
#define GMTK_WAN_INTERNET_DOWN 13
#define GMTK_WAN_INTERNET_CONNECTING 14
#define GMTK_WAN_INTERNET_CONNECTED 15
#define GMTK_WAN_INTERNET_ERROR 16


#endif	// __RALINK_GPIO_H__

