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
 *
 */
#include <linux/init.h>
#include <linux/version.h>
#include <linux/module.h>
#include <linux/config.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/fs.h>
#include <linux/sched.h>
#ifdef CONFIG_RALINK_GPIO_LED
#include <linux/timer.h>
#endif
#include <asm/uaccess.h>
#include "product_gpio_cfg.h"
#include "ralink_gpio_link.h"
#include "../../arch/mips/rt2880/serial_rt2880.h"

#include <asm/rt2880/surfboardint.h>

#ifdef  CONFIG_DEVFS_FS
#include <linux/devfs_fs_kernel.h>
static  devfs_handle_t devfs_handle;
#endif


#define NAME			"ralink_gpio"
#define RALINK_GPIO_DEVNAME	"gpio"
int ralink_gpio_major = 252;
int ralink_gpio_irqnum = 0;
u32 ralink_gpio_intp = 0;
u32 ralink_gpio_edge = 0;
#if defined (RALINK_GPIO_HAS_2722)
u32 ralink_gpio2722_intp = 0;
u32 ralink_gpio2722_edge = 0;
#elif defined (RALINK_GPIO_HAS_4524)
u32 ralink_gpio3924_intp = 0;
u32 ralink_gpio3924_edge = 0;
u32 ralink_gpio4540_intp = 0;
u32 ralink_gpio4540_edge = 0;
#elif defined (RALINK_GPIO_HAS_5124)
u32 ralink_gpio3924_intp = 0;
u32 ralink_gpio3924_edge = 0;
u32 ralink_gpio5140_intp = 0;
u32 ralink_gpio5140_edge = 0;
#elif defined (RALINK_GPIO_HAS_9524) || defined (RALINK_GPIO_HAS_7224)
u32 ralink_gpio3924_intp = 0;
u32 ralink_gpio3924_edge = 0;
u32 ralink_gpio7140_intp = 0;
u32 ralink_gpio7140_edge = 0;
#if defined (RALINK_GPIO_HAS_7224)
u32 ralink_gpio72_intp = 0;
u32 ralink_gpio72_edge = 0;
#else
u32 ralink_gpio9572_intp = 0;
u32 ralink_gpio9572_edge = 0;
#endif
#elif defined (RALINK_GPIO_HAS_9532)
u32 ralink_gpio6332_intp = 0;
u32 ralink_gpio6332_edge = 0;
u32 ralink_gpio9564_intp = 0;
u32 ralink_gpio9564_edge = 0;
#endif
ralink_gpio_reg_info ralink_gpio_info[RALINK_GPIO_NUMBER];
extern unsigned long volatile jiffies;

#ifdef CONFIG_RALINK_GPIO_LED
#define RALINK_LED_DEBUG 0
#define RALINK_GPIO_LED_FREQ (HZ/10)
struct timer_list ralink_gpio_led_timer;
ralink_gpio_led_info ralink_gpio_led_data[RALINK_GPIO_NUMBER];

u32 ra_gpio_led_set = 0;
u32 ra_gpio_led_clr = 0;
#if defined (RALINK_GPIO_HAS_2722)
u32 ra_gpio2722_led_set = 0;
u32 ra_gpio2722_led_clr = 0;
#elif defined (RALINK_GPIO_HAS_4524)
u32 ra_gpio3924_led_set = 0;
u32 ra_gpio3924_led_clr = 0;
u32 ra_gpio4540_led_set = 0;
u32 ra_gpio4540_led_clr = 0;
#elif defined (RALINK_GPIO_HAS_5124)
u32 ra_gpio3924_led_set = 0;
u32 ra_gpio3924_led_clr = 0;
u32 ra_gpio5140_led_set = 0;
u32 ra_gpio5140_led_clr = 0;
#elif defined (RALINK_GPIO_HAS_9524) || defined (RALINK_GPIO_HAS_7224)
u32 ra_gpio3924_led_set = 0;
u32 ra_gpio3924_led_clr = 0;
u32 ra_gpio7140_led_set = 0;
u32 ra_gpio7140_led_clr = 0;
#if defined (RALINK_GPIO_HAS_7224)
u32 ra_gpio72_led_set = 0;
u32 ra_gpio72_led_clr = 0;
#else
u32 ra_gpio9572_led_set = 0;
u32 ra_gpio9572_led_clr = 0;
#endif
#elif defined (RALINK_GPIO_HAS_9532)
u32 ra_gpio6332_led_set = 0;
u32 ra_gpio6332_led_clr = 0;
u32 ra_gpio9564_led_set = 0;
u32 ra_gpio9564_led_clr = 0;
#endif
struct ralink_gpio_led_status_t {
	int ticks;
	unsigned int ons;
	unsigned int offs;
	unsigned int resting;
	unsigned int times;
} ralink_gpio_led_stat[RALINK_GPIO_NUMBER];
#endif

MODULE_DESCRIPTION("Ralink SoC GPIO Driver");
MODULE_AUTHOR("Winfred Lu <winfred_lu@ralinktech.com.tw>");
MODULE_LICENSE("GPL");
ralink_gpio_reg_info info;


int motion_sensor_status = 0;
int motion_sensor_delay = 0;
int motion_sensor_sensitivity = 0;
int motion_count = 0;
int motion_delay_timer_count = 0;
int restoreValue;

static int __init LED_init(void);

int ralink_gpio_led_set(ralink_gpio_led_info led)
{
#ifdef CONFIG_RALINK_GPIO_LED
	unsigned long tmp;
	if (0 <= led.gpio && led.gpio < RALINK_GPIO_NUMBER) {
		if (led.on > RALINK_GPIO_LED_INFINITY)
			led.on = RALINK_GPIO_LED_INFINITY;
		if (led.off > RALINK_GPIO_LED_INFINITY)
			led.off = RALINK_GPIO_LED_INFINITY;
		if (led.blinks > RALINK_GPIO_LED_INFINITY)
			led.blinks = RALINK_GPIO_LED_INFINITY;
		if (led.rests > RALINK_GPIO_LED_INFINITY)
			led.rests = RALINK_GPIO_LED_INFINITY;
		if (led.times > RALINK_GPIO_LED_INFINITY)
			led.times = RALINK_GPIO_LED_INFINITY;
		if (led.on == 0 && led.off == 0 && led.blinks == 0 &&
				led.rests == 0) {
			ralink_gpio_led_data[led.gpio].gpio = -1; //stop it
			return 0;
		}
		//register led data
		ralink_gpio_led_data[led.gpio].gpio = led.gpio;
		ralink_gpio_led_data[led.gpio].on = (led.on == 0)? 1 : led.on;
		ralink_gpio_led_data[led.gpio].off = (led.off == 0)? 1 : led.off;
		ralink_gpio_led_data[led.gpio].blinks = (led.blinks == 0)? 1 : led.blinks;
		ralink_gpio_led_data[led.gpio].rests = (led.rests == 0)? 1 : led.rests;
		ralink_gpio_led_data[led.gpio].times = (led.times == 0)? 1 : led.times;

		//clear previous led status
		ralink_gpio_led_stat[led.gpio].ticks = -1;
		ralink_gpio_led_stat[led.gpio].ons = 0;
		ralink_gpio_led_stat[led.gpio].offs = 0;
		ralink_gpio_led_stat[led.gpio].resting = 0;
		ralink_gpio_led_stat[led.gpio].times = 0;

		printk(KERN_EMERG "led=%d, on=%d, off=%d, blinks,=%d, reset=%d, time=%d\n",
				ralink_gpio_led_data[led.gpio].gpio,
				ralink_gpio_led_data[led.gpio].on,
				ralink_gpio_led_data[led.gpio].off,
				ralink_gpio_led_data[led.gpio].blinks,
				ralink_gpio_led_data[led.gpio].rests,
				ralink_gpio_led_data[led.gpio].times);
		//set gpio direction to 'out'
#if defined (RALINK_GPIO_HAS_2722)
		if (led.gpio <= 21) {
			tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIODIR));
			tmp |= RALINK_GPIO(led.gpio);
			*(volatile u32 *)(RALINK_REG_PIODIR) = tmp;
		}
		else {
			tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO2722DIR));
			tmp |= RALINK_GPIO((led.gpio-22));
			*(volatile u32 *)(RALINK_REG_PIO2722DIR) = tmp;
		}
#elif defined (RALINK_GPIO_HAS_9532)
		if (led.gpio <= 31) {
			tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIODIR));
			tmp |= RALINK_GPIO(led.gpio);
			*(volatile u32 *)(RALINK_REG_PIODIR) = tmp;
		} else if (led.gpio <= 63) {
			tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO6332DIR));
			tmp |= RALINK_GPIO((led.gpio-32));
			*(volatile u32 *)(RALINK_REG_PIO6332DIR) = tmp;
		} else {
			tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO9564DIR));
			tmp |= RALINK_GPIO((led.gpio-64));
			*(volatile u32 *)(RALINK_REG_PIO9564DIR) = tmp;
		}
#else
		if (led.gpio <= 23) {
			tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIODIR));
			tmp |= RALINK_GPIO(led.gpio);
			*(volatile u32 *)(RALINK_REG_PIODIR) = tmp;
		}
#if defined (RALINK_GPIO_HAS_4524)
		else if (led.gpio <= 39) {
			tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO3924DIR));
			tmp |= RALINK_GPIO((led.gpio-24));
			*(volatile u32 *)(RALINK_REG_PIO3924DIR) = tmp;
		}
		else {
			tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO4540DIR));
			tmp |= RALINK_GPIO((led.gpio-40));
			*(volatile u32 *)(RALINK_REG_PIO4540DIR) = tmp;
		}
#elif defined (RALINK_GPIO_HAS_5124)
		else if (led.gpio <= 39) {
			tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO3924DIR));
			tmp |= RALINK_GPIO((led.gpio-24));
			*(volatile u32 *)(RALINK_REG_PIO3924DIR) = tmp;
		}
		else {
			tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO5140DIR));
			tmp |= RALINK_GPIO((led.gpio-40));
			*(volatile u32 *)(RALINK_REG_PIO5140DIR) = tmp;
		}
#elif defined (RALINK_GPIO_HAS_9524) || defined (RALINK_GPIO_HAS_7224)
		else if (led.gpio <= 39) {
			tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO3924DIR));
			tmp |= RALINK_GPIO((led.gpio-24));
			*(volatile u32 *)(RALINK_REG_PIO3924DIR) = tmp;
		}
		else if (led.gpio <= 71) {
			tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO7140DIR));
			tmp |= RALINK_GPIO((led.gpio-40));
			*(volatile u32 *)(RALINK_REG_PIO7140DIR) = tmp;
		}
		else {
#if defined (RALINK_GPIO_HAS_7224)
			tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO72DIR));
			tmp |= RALINK_GPIO((led.gpio-72));
			*(volatile u32 *)(RALINK_REG_PIO72DIR) = tmp;
#else
			tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO9572DIR));
			tmp |= RALINK_GPIO((led.gpio-72));
			*(volatile u32 *)(RALINK_REG_PIO9572DIR) = tmp;
#endif
		}
#endif
#endif
#if RALINK_LED_DEBUG
		printk("dir_%x gpio_%d - %d %d %d %d %d\n", tmp,
				led.gpio, led.on, led.off, led.blinks,
				led.rests, led.times);
#endif
	}
	else {
		printk(KERN_ERR NAME ": gpio(%d) out of range\n", led.gpio);
		return -1;
	}
	return 0;
#else
	printk(KERN_ERR NAME ": gpio led support not built\n");
	return -1;
#endif
}
EXPORT_SYMBOL(ralink_gpio_led_set);

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,35)
long ralink_gpio_ioctl(struct file *file, unsigned int req,
		unsigned long arg)
#else
int ralink_gpio_ioctl(struct inode *inode, struct file *file, unsigned int req,
		unsigned long arg)
#endif
{
	unsigned long idx, tmp;
	ralink_gpio_reg_info info;
#ifdef CONFIG_RALINK_GPIO_LED
	ralink_gpio_led_info led;
#endif

	idx = (req >> RALINK_GPIO_DATA_LEN) & 0xFFL;
	req &= RALINK_GPIO_DATA_MASK;

    u32 gpio_ioctl;
    unsigned long my_arg;

	switch(req) {
	case RALINK_GPIO_SET_DIR:
		*(volatile u32 *)(RALINK_REG_PIODIR) = cpu_to_le32(arg);
		break;
	case RALINK_GPIO_SET_DIR_IN:
		tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIODIR));
		tmp &= ~cpu_to_le32(arg);
		*(volatile u32 *)(RALINK_REG_PIODIR) = tmp;
		break;
	case RALINK_GPIO_SET_DIR_OUT:
		tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIODIR));
		tmp |= cpu_to_le32(arg);
		*(volatile u32 *)(RALINK_REG_PIODIR) = tmp;
		break;
	case RALINK_GPIO_READ: //RALINK_GPIO_READ_INT
		tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIODATA));
		put_user(tmp, (int __user *)arg);
		break;
	case RALINK_GPIO_WRITE: //RALINK_GPIO_WRITE_INT
		*(volatile u32 *)(RALINK_REG_PIODATA) = cpu_to_le32(arg);
		break;
	//WNC0_Ruei, 20130801		
	case RALINK_GPIO_RESET_EM357: //RALINK_GPIO_RESET_EM357_INT
		*(volatile u32 *)(RALINK_REG_PIODATA) &=~(1<<2);
		gpio_ioctl = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIODATA));
		printk(KERN_EMERG "\n[reset] 111 gpio_ioctl = [0x%08X]\n", gpio_ioctl);
		usleep(500*1000);
		
		*(volatile u32 *)(RALINK_REG_PIODATA) |=(1<<2);
		gpio_ioctl = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIODATA));
		printk(KERN_EMERG "\n[reset] 222 gpio_ioctl = [0x%08X]\n", gpio_ioctl);
		break;
	//WNC0
	case RALINK_GPIO_SET: //RALINK_GPIO_SET_INT
		*(volatile u32 *)(RALINK_REG_PIOSET) = cpu_to_le32(arg);
		break;
	case RALINK_GPIO_CLEAR: //RALINK_GPIO_CLEAR_INT
		*(volatile u32 *)(RALINK_REG_PIORESET) = cpu_to_le32(arg);
		break;
	case RALINK_GPIO_READ_BIT:
		tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIODATA));
		if (0L <= idx && idx < RALINK_GPIO_DATA_LEN) {
			tmp = (tmp >> idx) & 1L;
			put_user(tmp, (int __user *)arg);
		}
		else
			return -EINVAL;
		break;
	case RALINK_GPIO_WRITE_BIT:
		if (0L <= idx && idx < RALINK_GPIO_DATA_LEN) {
			tmp =le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIODATA));
			if (arg & 1L)
				tmp |= (1L << idx);
			else
				tmp &= ~(1L << idx);
			*(volatile u32 *)(RALINK_REG_PIODATA)= cpu_to_le32(tmp);
		}
		else
			return -EINVAL;
		break;
	case RALINK_GPIO_READ_BYTE:
		tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIODATA));
		if (0L <= idx && idx < RALINK_GPIO_DATA_LEN/8) {
			tmp = (tmp >> idx*8) & 0xFFL;
			put_user(tmp, (int __user *)arg);
		}
		else
			return -EINVAL;
		break;
	case RALINK_GPIO_WRITE_BYTE:
		if (0L <= idx && idx < RALINK_GPIO_DATA_LEN/8) {
			tmp =le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIODATA));
			tmp &= ~(0xFFL << idx*8);
			tmp |= ((arg & 0xFFL) << idx*8);
			*(volatile u32 *)(RALINK_REG_PIODATA)= cpu_to_le32(tmp);
		}
		else
			return -EINVAL;
		break;
	case RALINK_GPIO_ENABLE_INTP:
		*(volatile u32 *)(RALINK_REG_INTENA) = cpu_to_le32(RALINK_INTCTL_PIO);
		break;
	case RALINK_GPIO_DISABLE_INTP:
		*(volatile u32 *)(RALINK_REG_INTDIS) = cpu_to_le32(RALINK_INTCTL_PIO);
		break;
	case RALINK_GPIO_REG_IRQ:
		copy_from_user(&info, (ralink_gpio_reg_info *)arg, sizeof(info));
		if (0 <= info.irq && info.irq < RALINK_GPIO_NUMBER) {
			ralink_gpio_info[info.irq].pid = info.pid;
#if defined (RALINK_GPIO_HAS_2722)
			if (info.irq <= 21) {
				tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIORENA));
				tmp |= (0x1 << info.irq);
				*(volatile u32 *)(RALINK_REG_PIORENA) = cpu_to_le32(tmp);
				tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIOFENA));
				tmp |= (0x1 << info.irq);
				*(volatile u32 *)(RALINK_REG_PIOFENA) = cpu_to_le32(tmp);
			}
			else {
				tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO2722RENA));
				tmp |= (0x1 << (info.irq-22));
				*(volatile u32 *)(RALINK_REG_PIO2722RENA) = cpu_to_le32(tmp);
				tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO2722FENA));
				tmp |= (0x1 << (info.irq-22));
				*(volatile u32 *)(RALINK_REG_PIO2722FENA) = cpu_to_le32(tmp);
			}
#elif defined (RALINK_GPIO_HAS_9532)
			if (info.irq <= 31) {
				tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIORENA));
				tmp |= (0x1 << info.irq);
				*(volatile u32 *)(RALINK_REG_PIORENA) = cpu_to_le32(tmp);
				tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIOFENA));
				tmp |= (0x1 << info.irq);
				*(volatile u32 *)(RALINK_REG_PIOFENA) = cpu_to_le32(tmp);
			} else if (info.irq <= 63) {
				tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO6332RENA));
				tmp |= (0x1 << (info.irq-32));
				*(volatile u32 *)(RALINK_REG_PIO6332RENA) = cpu_to_le32(tmp);
				tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO6332FENA));
				tmp |= (0x1 << (info.irq-32));
				*(volatile u32 *)(RALINK_REG_PIO6332FENA) = cpu_to_le32(tmp);
			} else {
				tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO9564RENA));
				tmp |= (0x1 << (info.irq-64));
				*(volatile u32 *)(RALINK_REG_PIO9564RENA) = cpu_to_le32(tmp);
				tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO9564FENA));
				tmp |= (0x1 << (info.irq-64));
				*(volatile u32 *)(RALINK_REG_PIO9564FENA) = cpu_to_le32(tmp);
			}
#else
			if (info.irq <= 23) {
				tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIORENA));
				tmp |= (0x1 << info.irq);
				*(volatile u32 *)(RALINK_REG_PIORENA) = cpu_to_le32(tmp);
				tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIOFENA));
				tmp |= (0x1 << info.irq);
				*(volatile u32 *)(RALINK_REG_PIOFENA) = cpu_to_le32(tmp);
			}
#if defined (RALINK_GPIO_HAS_4524)
			else if (info.irq <= 39) {
				tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO3924RENA));
				tmp |= (0x1 << (info.irq-24));
				*(volatile u32 *)(RALINK_REG_PIO3924RENA) = cpu_to_le32(tmp);
				tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO3924FENA));
				tmp |= (0x1 << (info.irq-24));
				*(volatile u32 *)(RALINK_REG_PIO3924FENA) = cpu_to_le32(tmp);
			}
			else {
				tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO4540RENA));
				tmp |= (0x1 << (info.irq-40));
				*(volatile u32 *)(RALINK_REG_PIO4540RENA) = cpu_to_le32(tmp);
				tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO4540FENA));
				tmp |= (0x1 << (info.irq-40));
				*(volatile u32 *)(RALINK_REG_PIO4540FENA) = cpu_to_le32(tmp);
			}
#elif defined (RALINK_GPIO_HAS_5124)
			else if (info.irq <= 39) {
				tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO3924RENA));
				tmp |= (0x1 << (info.irq-24));
				*(volatile u32 *)(RALINK_REG_PIO3924RENA) = cpu_to_le32(tmp);
				tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO3924FENA));
				tmp |= (0x1 << (info.irq-24));
				*(volatile u32 *)(RALINK_REG_PIO3924FENA) = cpu_to_le32(tmp);
			}
			else {
				tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO5140RENA));
				tmp |= (0x1 << (info.irq-40));
				*(volatile u32 *)(RALINK_REG_PIO5140RENA) = cpu_to_le32(tmp);
				tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO5140FENA));
				tmp |= (0x1 << (info.irq-40));
				*(volatile u32 *)(RALINK_REG_PIO5140FENA) = cpu_to_le32(tmp);
			}
#elif defined (RALINK_GPIO_HAS_9524) || defined (RALINK_GPIO_HAS_7224)
			else if (info.irq <= 39) {
				tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO3924RENA));
				tmp |= (0x1 << (info.irq-24));
				*(volatile u32 *)(RALINK_REG_PIO3924RENA) = cpu_to_le32(tmp);
				tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO3924FENA));
				tmp |= (0x1 << (info.irq-24));
				*(volatile u32 *)(RALINK_REG_PIO3924FENA) = cpu_to_le32(tmp);
			}
			else if (info.irq <= 71) {
				tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO7140RENA));
				tmp |= (0x1 << (info.irq-40));
				*(volatile u32 *)(RALINK_REG_PIO7140RENA) = cpu_to_le32(tmp);
				tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO7140FENA));
				tmp |= (0x1 << (info.irq-40));
				*(volatile u32 *)(RALINK_REG_PIO7140FENA) = cpu_to_le32(tmp);
			}
			else {
#if defined (RALINK_GPIO_HAS_7224)
				tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO72RENA));
				tmp |= (0x1 << (info.irq-72));
				*(volatile u32 *)(RALINK_REG_PIO72RENA) = cpu_to_le32(tmp);
				tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO72FENA));
				tmp |= (0x1 << (info.irq-72));
				*(volatile u32 *)(RALINK_REG_PIO72FENA) = cpu_to_le32(tmp);
#else
				tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO9572RENA));
				tmp |= (0x1 << (info.irq-72));
				*(volatile u32 *)(RALINK_REG_PIO9572RENA) = cpu_to_le32(tmp);
				tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO9572FENA));
				tmp |= (0x1 << (info.irq-72));
				*(volatile u32 *)(RALINK_REG_PIO9572FENA) = cpu_to_le32(tmp);
#endif
			}
#endif
#endif
		}
		else
			printk(KERN_ERR NAME ": irq number(%d) out of range\n",
					info.irq);
		break;

#if defined (RALINK_GPIO_HAS_2722)
	case RALINK_GPIO2722_SET_DIR:
		*(volatile u32 *)(RALINK_REG_PIO2722DIR) = cpu_to_le32(arg);
		break;
	case RALINK_GPIO2722_SET_DIR_IN:
		tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO2722DIR));
		tmp &= ~cpu_to_le32(arg);
		*(volatile u32 *)(RALINK_REG_PIO2722DIR) = tmp;
		break;
	case RALINK_GPIO2722_SET_DIR_OUT:
		tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO2722DIR));
		tmp |= cpu_to_le32(arg);
		*(volatile u32 *)(RALINK_REG_PIO2722DIR) = tmp;
		break;
	case RALINK_GPIO2722_READ:
		tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO2722DATA));
		put_user(tmp, (int __user *)arg);
		break;
	case RALINK_GPIO2722_WRITE:
		*(volatile u32 *)(RALINK_REG_PIO2722DATA) = cpu_to_le32(arg);
		break;
	case RALINK_GPIO2722_SET:
		*(volatile u32 *)(RALINK_REG_PIO2722SET) = cpu_to_le32(arg);
		break;
	case RALINK_GPIO2722_CLEAR:
		*(volatile u32 *)(RALINK_REG_PIO2722SET) = cpu_to_le32(arg);
		break;
#elif defined (RALINK_GPIO_HAS_9532)
	case RALINK_GPIO6332_SET_DIR:
		*(volatile u32 *)(RALINK_REG_PIO6332DIR) = cpu_to_le32(arg);
		break;
	case RALINK_GPIO6332_SET_DIR_IN:
		tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO6332DIR));
		tmp &= ~cpu_to_le32(arg);
		*(volatile u32 *)(RALINK_REG_PIO6332DIR) = tmp;
		break;
	case RALINK_GPIO6332_SET_DIR_OUT:
		tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO6332DIR));
		tmp |= cpu_to_le32(arg);
		*(volatile u32 *)(RALINK_REG_PIO6332DIR) = tmp;
		break;
	case RALINK_GPIO6332_READ:
		tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO6332DATA));
		put_user(tmp, (int __user *)arg);
		break;
	case RALINK_GPIO6332_WRITE:
		*(volatile u32 *)(RALINK_REG_PIO6332DATA) = cpu_to_le32(arg);
		break;
	case RALINK_GPIO6332_SET:
		*(volatile u32 *)(RALINK_REG_PIO6332SET) = cpu_to_le32(arg);
		break;
	case RALINK_GPIO6332_CLEAR:
		*(volatile u32 *)(RALINK_REG_PIO6332SET) = cpu_to_le32(arg);
		break;
	case RALINK_GPIO9564_SET_DIR:
		*(volatile u32 *)(RALINK_REG_PIO9564DIR) = cpu_to_le32(arg);
		break;
	case RALINK_GPIO9564_SET_DIR_IN:
		tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO9564DIR));
		tmp &= ~cpu_to_le32(arg);
		*(volatile u32 *)(RALINK_REG_PIO9564DIR) = tmp;
		break;
	case RALINK_GPIO9564_SET_DIR_OUT:
		tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO9564DIR));
		tmp |= cpu_to_le32(arg);
		*(volatile u32 *)(RALINK_REG_PIO9564DIR) = tmp;
		break;
	case RALINK_GPIO9564_READ:
		tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO9564DATA));
		put_user(tmp, (int __user *)arg);
		break;
	case RALINK_GPIO9564_WRITE:
		*(volatile u32 *)(RALINK_REG_PIO9564DATA) = cpu_to_le32(arg);
		break;
	case RALINK_GPIO9564_SET:
		*(volatile u32 *)(RALINK_REG_PIO9564SET) = cpu_to_le32(arg);
		break;
	case RALINK_GPIO9564_CLEAR:
		*(volatile u32 *)(RALINK_REG_PIO9564SET) = cpu_to_le32(arg);
		break;
#elif defined (RALINK_GPIO_HAS_4524)
	case RALINK_GPIO3924_SET_DIR:
		*(volatile u32 *)(RALINK_REG_PIO3924DIR) = cpu_to_le32(arg);
		break;
	case RALINK_GPIO3924_SET_DIR_IN:
		tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO3924DIR));
		tmp &= ~cpu_to_le32(arg);
		*(volatile u32 *)(RALINK_REG_PIO3924DIR) = tmp;
		break;
	case RALINK_GPIO3924_SET_DIR_OUT:
		tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO3924DIR));
		tmp |= cpu_to_le32(arg);
		*(volatile u32 *)(RALINK_REG_PIO3924DIR) = tmp;
		break;
	case RALINK_GPIO3924_READ:
		tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO3924DATA));
		put_user(tmp, (int __user *)arg);
		break;
	case RALINK_GPIO3924_WRITE:
		*(volatile u32 *)(RALINK_REG_PIO3924DATA) = cpu_to_le32(arg);
		break;
	case RALINK_GPIO3924_SET:
		*(volatile u32 *)(RALINK_REG_PIO3924SET) = cpu_to_le32(arg);
		break;
	case RALINK_GPIO3924_CLEAR:
		*(volatile u32 *)(RALINK_REG_PIO3924SET) = cpu_to_le32(arg);
		break;

	case RALINK_GPIO4540_SET_DIR:
		*(volatile u32 *)(RALINK_REG_PIO4540DIR) = cpu_to_le32(arg);
		break;
	case RALINK_GPIO4540_SET_DIR_IN:
		tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO4540DIR));
		tmp &= ~cpu_to_le32(arg);
		*(volatile u32 *)(RALINK_REG_PIO4540DIR) = tmp;
		break;
	case RALINK_GPIO4540_SET_DIR_OUT:
		tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO4540DIR));
		tmp |= cpu_to_le32(arg);
		*(volatile u32 *)(RALINK_REG_PIO4540DIR) = tmp;
		break;
	case RALINK_GPIO4540_READ:
		tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO4540DATA));
		put_user(tmp, (int __user *)arg);
		break;
	case RALINK_GPIO4540_WRITE:
		*(volatile u32 *)(RALINK_REG_PIO4540DATA) = cpu_to_le32(arg);
		break;
	case RALINK_GPIO4540_SET:
		*(volatile u32 *)(RALINK_REG_PIO4540SET) = cpu_to_le32(arg);
		break;
	case RALINK_GPIO4540_CLEAR:
		*(volatile u32 *)(RALINK_REG_PIO4540SET) = cpu_to_le32(arg);
		break;
#elif defined (RALINK_GPIO_HAS_5124)
	case RALINK_GPIO3924_SET_DIR:
		*(volatile u32 *)(RALINK_REG_PIO3924DIR) = cpu_to_le32(arg);
		break;
	case RALINK_GPIO3924_SET_DIR_IN:
		tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO3924DIR));
		tmp &= ~cpu_to_le32(arg);
		*(volatile u32 *)(RALINK_REG_PIO3924DIR) = tmp;
		break;
	case RALINK_GPIO3924_SET_DIR_OUT:
		tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO3924DIR));
		tmp |= cpu_to_le32(arg);
		*(volatile u32 *)(RALINK_REG_PIO3924DIR) = tmp;
		break;
	case RALINK_GPIO3924_READ:
		tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO3924DATA));
		put_user(tmp, (int __user *)arg);
		break;
	case RALINK_GPIO3924_WRITE:
		*(volatile u32 *)(RALINK_REG_PIO3924DATA) = cpu_to_le32(arg);
		break;
	case RALINK_GPIO3924_SET:
		*(volatile u32 *)(RALINK_REG_PIO3924SET) = cpu_to_le32(arg);
		break;
	case RALINK_GPIO3924_CLEAR:
		*(volatile u32 *)(RALINK_REG_PIO3924SET) = cpu_to_le32(arg);
		break;

	case RALINK_GPIO5140_SET_DIR:
		*(volatile u32 *)(RALINK_REG_PIO5140DIR) = cpu_to_le32(arg);
		break;
	case RALINK_GPIO5140_SET_DIR_IN:
		tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO5140DIR));
		tmp &= ~cpu_to_le32(arg);
		*(volatile u32 *)(RALINK_REG_PIO5140DIR) = tmp;
		break;
	case RALINK_GPIO5140_SET_DIR_OUT:
		tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO5140DIR));
		tmp |= cpu_to_le32(arg);
		*(volatile u32 *)(RALINK_REG_PIO5140DIR) = tmp;
		break;
	case RALINK_GPIO5140_READ:
		tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO5140DATA));
		put_user(tmp, (int __user *)arg);
		break;
	case RALINK_GPIO5140_WRITE:
		*(volatile u32 *)(RALINK_REG_PIO5140DATA) = cpu_to_le32(arg);
		break;
	case RALINK_GPIO5140_SET:
		*(volatile u32 *)(RALINK_REG_PIO5140SET) = cpu_to_le32(arg);
		break;
	case RALINK_GPIO5140_CLEAR:
		*(volatile u32 *)(RALINK_REG_PIO5140SET) = cpu_to_le32(arg);
		break;
#elif defined (RALINK_GPIO_HAS_9524) || defined (RALINK_GPIO_HAS_7224)
	case RALINK_GPIO3924_SET_DIR:
		*(volatile u32 *)(RALINK_REG_PIO3924DIR) = cpu_to_le32(arg);
		break;
	case RALINK_GPIO3924_SET_DIR_IN:
		tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO3924DIR));
		tmp &= ~cpu_to_le32(arg);
		*(volatile u32 *)(RALINK_REG_PIO3924DIR) = tmp;
		break;
	case RALINK_GPIO3924_SET_DIR_OUT:
		tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO3924DIR));
		tmp |= cpu_to_le32(arg);
		*(volatile u32 *)(RALINK_REG_PIO3924DIR) = tmp;
		break;
	case RALINK_GPIO3924_READ:
		tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO3924DATA));
		put_user(tmp, (int __user *)arg);
		break;
	case RALINK_GPIO3924_WRITE:
		*(volatile u32 *)(RALINK_REG_PIO3924DATA) = cpu_to_le32(arg);
		break;
	case RALINK_GPIO3924_SET:
		*(volatile u32 *)(RALINK_REG_PIO3924SET) = cpu_to_le32(arg);
		break;
	case RALINK_GPIO3924_CLEAR:
		*(volatile u32 *)(RALINK_REG_PIO3924SET) = cpu_to_le32(arg);
		break;

	case RALINK_GPIO7140_SET_DIR:
		*(volatile u32 *)(RALINK_REG_PIO7140DIR) = cpu_to_le32(arg);
		break;
	case RALINK_GPIO7140_SET_DIR_IN:
		tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO7140DIR));
		tmp &= ~cpu_to_le32(arg);
		*(volatile u32 *)(RALINK_REG_PIO7140DIR) = tmp;
		break;
	case RALINK_GPIO7140_SET_DIR_OUT:
		tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO7140DIR));
		tmp |= cpu_to_le32(arg);
		*(volatile u32 *)(RALINK_REG_PIO7140DIR) = tmp;
		break;
	case RALINK_GPIO7140_READ:
		tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO7140DATA));
		put_user(tmp, (int __user *)arg);
		break;
	case RALINK_GPIO7140_WRITE:
		*(volatile u32 *)(RALINK_REG_PIO7140DATA) = cpu_to_le32(arg);
		break;
	case RALINK_GPIO7140_SET:
		*(volatile u32 *)(RALINK_REG_PIO7140SET) = cpu_to_le32(arg);
		break;
	case RALINK_GPIO7140_CLEAR:
		*(volatile u32 *)(RALINK_REG_PIO7140SET) = cpu_to_le32(arg);
		break;
#if defined (RALINK_GPIO_HAS_7224)
	case RALINK_GPIO72_SET_DIR:
		*(volatile u32 *)(RALINK_REG_PIO72DIR) = cpu_to_le32(arg);
		break;
	case RALINK_GPIO72_SET_DIR_IN:
		tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO72DIR));
		tmp &= ~cpu_to_le32(arg);
		*(volatile u32 *)(RALINK_REG_PIO72DIR) = tmp;
		break;
	case RALINK_GPIO72_SET_DIR_OUT:
		tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO72DIR));
		tmp |= cpu_to_le32(arg);
		*(volatile u32 *)(RALINK_REG_PIO72DIR) = tmp;
		break;
	case RALINK_GPIO72_READ:
		tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO72DATA));
		put_user(tmp, (int __user *)arg);
		break;
	case RALINK_GPIO72_WRITE:
		*(volatile u32 *)(RALINK_REG_PIO72DATA) = cpu_to_le32(arg);
		break;
	case RALINK_GPIO72_SET:
		*(volatile u32 *)(RALINK_REG_PIO72SET) = cpu_to_le32(arg);
		break;
	case RALINK_GPIO72_CLEAR:
		*(volatile u32 *)(RALINK_REG_PIO72SET) = cpu_to_le32(arg);
		break;
#else
	case RALINK_GPIO9572_SET_DIR:
		*(volatile u32 *)(RALINK_REG_PIO9572DIR) = cpu_to_le32(arg);
		break;
	case RALINK_GPIO9572_SET_DIR_IN:
		tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO9572DIR));
		tmp &= ~cpu_to_le32(arg);
		*(volatile u32 *)(RALINK_REG_PIO9572DIR) = tmp;
		break;
	case RALINK_GPIO9572_SET_DIR_OUT:
		tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO9572DIR));
		tmp |= cpu_to_le32(arg);
		*(volatile u32 *)(RALINK_REG_PIO9572DIR) = tmp;
		break;
	case RALINK_GPIO9572_READ:
		tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO9572DATA));
		put_user(tmp, (int __user *)arg);
		break;
	case RALINK_GPIO9572_WRITE:
		*(volatile u32 *)(RALINK_REG_PIO9572DATA) = cpu_to_le32(arg);
		break;
	case RALINK_GPIO9572_SET:
		*(volatile u32 *)(RALINK_REG_PIO9572SET) = cpu_to_le32(arg);
		break;
	case RALINK_GPIO9572_CLEAR:
		*(volatile u32 *)(RALINK_REG_PIO9572SET) = cpu_to_le32(arg);
		break;
#endif
#endif

	case RALINK_GPIO_LED_SET:
#ifdef CONFIG_RALINK_GPIO_LED
		copy_from_user(&led, (ralink_gpio_led_info *)arg, sizeof(led));
		ralink_gpio_led_set(led);
#else
		printk(KERN_ERR NAME ": gpio led support not built\n");
#endif
		break;
	case RALINK_GENERAL_READ:
		put_user(restoreValue, (int __user *)arg);
		break;
	default:
		return -ENOIOCTLCMD;
	}
	return 0;
}

int ralink_gpio_open(struct inode *inode, struct file *file)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0)
	MOD_INC_USE_COUNT;
#else
	try_module_get(THIS_MODULE);
#endif
	return 0;
}

int ralink_gpio_release(struct inode *inode, struct file *file)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0)
	MOD_DEC_USE_COUNT;
#else
	module_put(THIS_MODULE);
#endif
	return 0;
}

struct file_operations ralink_gpio_fops =
{
	owner:		THIS_MODULE,
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,35)
	unlocked_ioctl:	ralink_gpio_ioctl,
#else
	ioctl:		ralink_gpio_ioctl,
#endif
	open:		ralink_gpio_open,
	release:	ralink_gpio_release,
};

#ifdef CONFIG_RALINK_GPIO_LED

#if RALINK_GPIO_LED_LOW_ACT
#define __LED_ON(gpio)      ra_gpio_led_clr |= RALINK_GPIO(gpio);
#define __LED_OFF(gpio)     ra_gpio_led_set |= RALINK_GPIO(gpio);
#define __LED2722_ON(gpio)  ra_gpio2722_led_clr |= RALINK_GPIO((gpio-22));
#define __LED2722_OFF(gpio) ra_gpio2722_led_set |= RALINK_GPIO((gpio-22));
#define __LED3924_ON(gpio)  ra_gpio3924_led_clr |= RALINK_GPIO((gpio-24));
#define __LED3924_OFF(gpio) ra_gpio3924_led_set |= RALINK_GPIO((gpio-24));
#define __LED4540_ON(gpio)  ra_gpio4540_led_clr |= RALINK_GPIO((gpio-40));
#define __LED4540_OFF(gpio) ra_gpio4540_led_set |= RALINK_GPIO((gpio-40));
#define __LED5140_ON(gpio)  ra_gpio5140_led_clr |= RALINK_GPIO((gpio-40));
#define __LED5140_OFF(gpio) ra_gpio5140_led_set |= RALINK_GPIO((gpio-40));
#define __LED7140_ON(gpio)  ra_gpio7140_led_clr |= RALINK_GPIO((gpio-40));
#define __LED7140_OFF(gpio) ra_gpio7140_led_set |= RALINK_GPIO((gpio-40));
#if defined (RALINK_GPIO_HAS_7224)
#define __LED72_ON(gpio)  ra_gpio72_led_clr |= RALINK_GPIO((gpio-72));
#define __LED72_OFF(gpio) ra_gpio72_led_set |= RALINK_GPIO((gpio-72));
#else
#define __LED9572_ON(gpio)  ra_gpio9572_led_clr |= RALINK_GPIO((gpio-72));
#define __LED9572_OFF(gpio) ra_gpio9572_led_set |= RALINK_GPIO((gpio-72));
#endif
#else
#define __LED_ON(gpio)      ra_gpio_led_set |= RALINK_GPIO(gpio);
#define __LED_OFF(gpio)     ra_gpio_led_clr |= RALINK_GPIO(gpio);
#define __LED2722_ON(gpio)  ra_gpio2722_led_set |= RALINK_GPIO((gpio-22));
#define __LED2722_OFF(gpio) ra_gpio2722_led_clr |= RALINK_GPIO((gpio-22));
#define __LED3924_ON(gpio)  ra_gpio3924_led_set |= RALINK_GPIO((gpio-24));
#define __LED3924_OFF(gpio) ra_gpio3924_led_clr |= RALINK_GPIO((gpio-24));
#define __LED4540_ON(gpio)  ra_gpio4540_led_set |= RALINK_GPIO((gpio-40));
#define __LED4540_OFF(gpio) ra_gpio4540_led_clr |= RALINK_GPIO((gpio-40));
#define __LED5140_ON(gpio)  ra_gpio5140_led_set |= RALINK_GPIO((gpio-40));
#define __LED5140_OFF(gpio) ra_gpio5140_led_clr |= RALINK_GPIO((gpio-40));
#define __LED7140_ON(gpio)  ra_gpio7140_led_set |= RALINK_GPIO((gpio-40));
#define __LED7140_OFF(gpio) ra_gpio7140_led_clr |= RALINK_GPIO((gpio-40));
#if defined (RALINK_GPIO_HAS_7224)
#define __LED72_ON(gpio)  ra_gpio72_led_set |= RALINK_GPIO((gpio-72));
#define __LED72_OFF(gpio) ra_gpio72_led_clr |= RALINK_GPIO((gpio-72));
#else
#define __LED9572_ON(gpio)  ra_gpio9572_led_set |= RALINK_GPIO((gpio-72));
#define __LED9572_OFF(gpio) ra_gpio9572_led_clr |= RALINK_GPIO((gpio-72));
#endif
#endif

static void ralink_gpio_led_do_timer(unsigned long unused)
{
	int i;
	unsigned int x;

#if defined (RALINK_GPIO_HAS_2722)
	for (i = 0; i < 22; i++) {
		ralink_gpio_led_stat[i].ticks++;
		if (ralink_gpio_led_data[i].gpio == -1) //-1 means unused
			continue;
		if (ralink_gpio_led_data[i].on == RALINK_GPIO_LED_INFINITY ||
				ralink_gpio_led_data[i].off == 0) { //always on
			__LED_ON(i);
			continue;
		}
		if (ralink_gpio_led_data[i].off == RALINK_GPIO_LED_INFINITY ||
				ralink_gpio_led_data[i].rests == RALINK_GPIO_LED_INFINITY ||
				ralink_gpio_led_data[i].on == 0 ||
				ralink_gpio_led_data[i].blinks == 0 ||
				ralink_gpio_led_data[i].times == 0) { //always off
			__LED_OFF(i);
			continue;
		}

		//led turn on or off
		if (ralink_gpio_led_data[i].blinks == RALINK_GPIO_LED_INFINITY ||
				ralink_gpio_led_data[i].rests == 0) { //always blinking
			x = ralink_gpio_led_stat[i].ticks % (ralink_gpio_led_data[i].on
					+ ralink_gpio_led_data[i].off);
		}
		else {
			unsigned int a, b, c, d, o, t;
			a = ralink_gpio_led_data[i].blinks / 2;
			b = ralink_gpio_led_data[i].rests / 2;
			c = ralink_gpio_led_data[i].blinks % 2;
			d = ralink_gpio_led_data[i].rests % 2;
			o = ralink_gpio_led_data[i].on + ralink_gpio_led_data[i].off;
			//t = blinking ticks
			t = a * o + ralink_gpio_led_data[i].on * c;
			//x = ticks % (blinking ticks + resting ticks)
			x = ralink_gpio_led_stat[i].ticks %
				(t + b * o + ralink_gpio_led_data[i].on * d);
			//starts from 0 at resting cycles
			if (x >= t)
				x -= t;
			x %= o;
		}
		if (x < ralink_gpio_led_data[i].on) {
			__LED_ON(i);
			if (ralink_gpio_led_stat[i].ticks && x == 0)
				ralink_gpio_led_stat[i].offs++;
#if RALINK_LED_DEBUG
			printk("t%d gpio%d on,", ralink_gpio_led_stat[i].ticks, i);
#endif
		}
		else {
			__LED_OFF(i);
			if (x == ralink_gpio_led_data[i].on)
				ralink_gpio_led_stat[i].ons++;
#if RALINK_LED_DEBUG
			printk("t%d gpio%d off,", ralink_gpio_led_stat[i].ticks, i);
#endif
		}

		//blinking or resting
		if (ralink_gpio_led_data[i].blinks == RALINK_GPIO_LED_INFINITY ||
				ralink_gpio_led_data[i].rests == 0) { //always blinking
			continue;
		}
		else {
			x = ralink_gpio_led_stat[i].ons + ralink_gpio_led_stat[i].offs;
			if (!ralink_gpio_led_stat[i].resting) {
				if (x == ralink_gpio_led_data[i].blinks) {
					ralink_gpio_led_stat[i].resting = 1;
					ralink_gpio_led_stat[i].ons = 0;
					ralink_gpio_led_stat[i].offs = 0;
					ralink_gpio_led_stat[i].times++;
				}
			}
			else {
				if (x == ralink_gpio_led_data[i].rests) {
					ralink_gpio_led_stat[i].resting = 0;
					ralink_gpio_led_stat[i].ons = 0;
					ralink_gpio_led_stat[i].offs = 0;
				}
			}
		}
		if (ralink_gpio_led_stat[i].resting) {
			__LED_OFF(i);
#if RALINK_LED_DEBUG
			printk("resting,");
		} else {
			printk("blinking,");
#endif
		}

		//number of times
		if (ralink_gpio_led_data[i].times != RALINK_GPIO_LED_INFINITY)
		{
			if (ralink_gpio_led_stat[i].times ==
					ralink_gpio_led_data[i].times) {
				__LED_OFF(i);
				ralink_gpio_led_data[i].gpio = -1; //stop
			}
#if RALINK_LED_DEBUG
			printk("T%d\n", ralink_gpio_led_stat[i].times);
		} else {
			printk("T@\n");
#endif
		}
	}

	for (i = 22; i <  RALINK_GPIO_NUMBER; i++) {
		ralink_gpio_led_stat[i].ticks++;
		if (ralink_gpio_led_data[i].gpio == -1) //-1 means unused
			continue;
		if (ralink_gpio_led_data[i].on == RALINK_GPIO_LED_INFINITY ||
				ralink_gpio_led_data[i].off == 0) { //always on
			__LED2722_ON(i);
			continue;
		}
		if (ralink_gpio_led_data[i].off == RALINK_GPIO_LED_INFINITY ||
				ralink_gpio_led_data[i].rests == RALINK_GPIO_LED_INFINITY ||
				ralink_gpio_led_data[i].on == 0 ||
				ralink_gpio_led_data[i].blinks == 0 ||
				ralink_gpio_led_data[i].times == 0) { //always off
			__LED2722_OFF(i);
			continue;
		}

		//led turn on or off
		if (ralink_gpio_led_data[i].blinks == RALINK_GPIO_LED_INFINITY ||
				ralink_gpio_led_data[i].rests == 0) { //always blinking
			x = ralink_gpio_led_stat[i].ticks % (ralink_gpio_led_data[i].on
					+ ralink_gpio_led_data[i].off);
		}
		else {
			unsigned int a, b, c, d, o, t;
			a = ralink_gpio_led_data[i].blinks / 2;
			b = ralink_gpio_led_data[i].rests / 2;
			c = ralink_gpio_led_data[i].blinks % 2;
			d = ralink_gpio_led_data[i].rests % 2;
			o = ralink_gpio_led_data[i].on + ralink_gpio_led_data[i].off;
			//t = blinking ticks
			t = a * o + ralink_gpio_led_data[i].on * c;
			//x = ticks % (blinking ticks + resting ticks)
			x = ralink_gpio_led_stat[i].ticks %
				(t + b * o + ralink_gpio_led_data[i].on * d);
			//starts from 0 at resting cycles
			if (x >= t)
				x -= t;
			x %= o;
		}
		if (x < ralink_gpio_led_data[i].on) {
			__LED2722_ON(i);
			if (ralink_gpio_led_stat[i].ticks && x == 0)
				ralink_gpio_led_stat[i].offs++;
#if RALINK_LED_DEBUG
			printk("t%d gpio%d on,", ralink_gpio_led_stat[i].ticks, i);
#endif
		}
		else {
			__LED2722_OFF(i);
			if (x == ralink_gpio_led_data[i].on)
				ralink_gpio_led_stat[i].ons++;
#if RALINK_LED_DEBUG
			printk("t%d gpio%d off,", ralink_gpio_led_stat[i].ticks, i);
#endif
		}

		//blinking or resting
		if (ralink_gpio_led_data[i].blinks == RALINK_GPIO_LED_INFINITY ||
				ralink_gpio_led_data[i].rests == 0) { //always blinking
			continue;
		}
		else {
			x = ralink_gpio_led_stat[i].ons + ralink_gpio_led_stat[i].offs;
			if (!ralink_gpio_led_stat[i].resting) {
				if (x == ralink_gpio_led_data[i].blinks) {
					ralink_gpio_led_stat[i].resting = 1;
					ralink_gpio_led_stat[i].ons = 0;
					ralink_gpio_led_stat[i].offs = 0;
					ralink_gpio_led_stat[i].times++;
				}
			}
			else {
				if (x == ralink_gpio_led_data[i].rests) {
					ralink_gpio_led_stat[i].resting = 0;
					ralink_gpio_led_stat[i].ons = 0;
					ralink_gpio_led_stat[i].offs = 0;
				}
			}
		}
		if (ralink_gpio_led_stat[i].resting) {
			__LED2722_OFF(i);
#if RALINK_LED_DEBUG
			printk("resting,");
		} else {
			printk("blinking,");
#endif
		}

		//number of times
		if (ralink_gpio_led_data[i].times != RALINK_GPIO_LED_INFINITY)
		{
			if (ralink_gpio_led_stat[i].times ==
					ralink_gpio_led_data[i].times) {
				__LED2722_OFF(i);
				ralink_gpio_led_data[i].gpio = -1; //stop
			}
#if RALINK_LED_DEBUG
			printk("T%d\n", ralink_gpio_led_stat[i].times);
		} else {
			printk("T@\n");
#endif
		}
	}
#else
	for (i = 0; i < 24; i++) {
		ralink_gpio_led_stat[i].ticks++;
		if (ralink_gpio_led_data[i].gpio == -1) //-1 means unused
			continue;
		if (ralink_gpio_led_data[i].on == RALINK_GPIO_LED_INFINITY ||
				ralink_gpio_led_data[i].off == 0) { //always on
			__LED_ON(i);
			continue;
		}
		if (ralink_gpio_led_data[i].off == RALINK_GPIO_LED_INFINITY ||
				ralink_gpio_led_data[i].rests == RALINK_GPIO_LED_INFINITY ||
				ralink_gpio_led_data[i].on == 0 ||
				ralink_gpio_led_data[i].blinks == 0 ||
				ralink_gpio_led_data[i].times == 0) { //always off
			__LED_OFF(i);
			continue;
		}

		//led turn on or off
		if (ralink_gpio_led_data[i].blinks == RALINK_GPIO_LED_INFINITY ||
				ralink_gpio_led_data[i].rests == 0) { //always blinking
			x = ralink_gpio_led_stat[i].ticks % (ralink_gpio_led_data[i].on
					+ ralink_gpio_led_data[i].off);
		}
		else {
			unsigned int a, b, c, d, o, t;
			a = ralink_gpio_led_data[i].blinks / 2;
			b = ralink_gpio_led_data[i].rests / 2;
			c = ralink_gpio_led_data[i].blinks % 2;
			d = ralink_gpio_led_data[i].rests % 2;
			o = ralink_gpio_led_data[i].on + ralink_gpio_led_data[i].off;
			//t = blinking ticks
			t = a * o + ralink_gpio_led_data[i].on * c;
			//x = ticks % (blinking ticks + resting ticks)
			x = ralink_gpio_led_stat[i].ticks %
				(t + b * o + ralink_gpio_led_data[i].on * d);
			//starts from 0 at resting cycles
			if (x >= t)
				x -= t;
			x %= o;
		}
		if (x < ralink_gpio_led_data[i].on) {
			__LED_ON(i);
			if (ralink_gpio_led_stat[i].ticks && x == 0)
				ralink_gpio_led_stat[i].offs++;
#if RALINK_LED_DEBUG
			printk("t%d gpio%d on,", ralink_gpio_led_stat[i].ticks, i);
#endif
		}
		else {
			__LED_OFF(i);
			if (x == ralink_gpio_led_data[i].on)
				ralink_gpio_led_stat[i].ons++;
#if RALINK_LED_DEBUG
			printk("t%d gpio%d off,", ralink_gpio_led_stat[i].ticks, i);
#endif
		}

		//blinking or resting
		if (ralink_gpio_led_data[i].blinks == RALINK_GPIO_LED_INFINITY ||
				ralink_gpio_led_data[i].rests == 0) { //always blinking
			continue;
		}
		else {
			x = ralink_gpio_led_stat[i].ons + ralink_gpio_led_stat[i].offs;
			if (!ralink_gpio_led_stat[i].resting) {
				if (x == ralink_gpio_led_data[i].blinks) {
					ralink_gpio_led_stat[i].resting = 1;
					ralink_gpio_led_stat[i].ons = 0;
					ralink_gpio_led_stat[i].offs = 0;
					ralink_gpio_led_stat[i].times++;
				}
			}
			else {
				if (x == ralink_gpio_led_data[i].rests) {
					ralink_gpio_led_stat[i].resting = 0;
					ralink_gpio_led_stat[i].ons = 0;
					ralink_gpio_led_stat[i].offs = 0;
				}
			}
		}
		if (ralink_gpio_led_stat[i].resting) {
			__LED_OFF(i);
#if RALINK_LED_DEBUG
			printk("resting,");
		} else {
			printk("blinking,");
#endif
		}

		//number of times
		if (ralink_gpio_led_data[i].times != RALINK_GPIO_LED_INFINITY)
		{
			if (ralink_gpio_led_stat[i].times ==
					ralink_gpio_led_data[i].times) {
				__LED_OFF(i);
				ralink_gpio_led_data[i].gpio = -1; //stop
			}
#if RALINK_LED_DEBUG
			printk("T%d\n", ralink_gpio_led_stat[i].times);
		} else {
			printk("T@\n");
#endif
		}
	}

#if defined (RALINK_GPIO_HAS_4524)
	for (i = 24; i < 40; i++) {
		ralink_gpio_led_stat[i].ticks++;
		if (ralink_gpio_led_data[i].gpio == -1) //-1 means unused
			continue;
		if (ralink_gpio_led_data[i].on == RALINK_GPIO_LED_INFINITY ||
				ralink_gpio_led_data[i].off == 0) { //always on
			__LED3924_ON(i);
			continue;
		}
		if (ralink_gpio_led_data[i].off == RALINK_GPIO_LED_INFINITY ||
				ralink_gpio_led_data[i].rests == RALINK_GPIO_LED_INFINITY ||
				ralink_gpio_led_data[i].on == 0 ||
				ralink_gpio_led_data[i].blinks == 0 ||
				ralink_gpio_led_data[i].times == 0) { //always off
			__LED3924_OFF(i);
			continue;
		}

		//led turn on or off
		if (ralink_gpio_led_data[i].blinks == RALINK_GPIO_LED_INFINITY ||
				ralink_gpio_led_data[i].rests == 0) { //always blinking
			x = ralink_gpio_led_stat[i].ticks % (ralink_gpio_led_data[i].on
					+ ralink_gpio_led_data[i].off);
		}
		else {
			unsigned int a, b, c, d, o, t;
			a = ralink_gpio_led_data[i].blinks / 2;
			b = ralink_gpio_led_data[i].rests / 2;
			c = ralink_gpio_led_data[i].blinks % 2;
			d = ralink_gpio_led_data[i].rests % 2;
			o = ralink_gpio_led_data[i].on + ralink_gpio_led_data[i].off;
			//t = blinking ticks
			t = a * o + ralink_gpio_led_data[i].on * c;
			//x = ticks % (blinking ticks + resting ticks)
			x = ralink_gpio_led_stat[i].ticks %
				(t + b * o + ralink_gpio_led_data[i].on * d);
			//starts from 0 at resting cycles
			if (x >= t)
				x -= t;
			x %= o;
		}
		if (x < ralink_gpio_led_data[i].on) {
			__LED3924_ON(i);
			if (ralink_gpio_led_stat[i].ticks && x == 0)
				ralink_gpio_led_stat[i].offs++;
#if RALINK_LED_DEBUG
			printk("t%d gpio%d on,", ralink_gpio_led_stat[i].ticks, i);
#endif
		}
		else {
			__LED3924_OFF(i);
			if (x == ralink_gpio_led_data[i].on)
				ralink_gpio_led_stat[i].ons++;
#if RALINK_LED_DEBUG
			printk("t%d gpio%d off,", ralink_gpio_led_stat[i].ticks, i);
#endif
		}

		//blinking or resting
		if (ralink_gpio_led_data[i].blinks == RALINK_GPIO_LED_INFINITY ||
				ralink_gpio_led_data[i].rests == 0) { //always blinking
			continue;
		}
		else {
			x = ralink_gpio_led_stat[i].ons + ralink_gpio_led_stat[i].offs;
			if (!ralink_gpio_led_stat[i].resting) {
				if (x == ralink_gpio_led_data[i].blinks) {
					ralink_gpio_led_stat[i].resting = 1;
					ralink_gpio_led_stat[i].ons = 0;
					ralink_gpio_led_stat[i].offs = 0;
					ralink_gpio_led_stat[i].times++;
				}
			}
			else {
				if (x == ralink_gpio_led_data[i].rests) {
					ralink_gpio_led_stat[i].resting = 0;
					ralink_gpio_led_stat[i].ons = 0;
					ralink_gpio_led_stat[i].offs = 0;
				}
			}
		}
		if (ralink_gpio_led_stat[i].resting) {
			__LED3924_OFF(i);
#if RALINK_LED_DEBUG
			printk("resting,");
		} else {
			printk("blinking,");
#endif
		}

		//number of times
		if (ralink_gpio_led_data[i].times != RALINK_GPIO_LED_INFINITY)
		{
			if (ralink_gpio_led_stat[i].times ==
					ralink_gpio_led_data[i].times) {
				__LED3924_OFF(i);
				ralink_gpio_led_data[i].gpio = -1; //stop
			}
#if RALINK_LED_DEBUG
			printk("T%d\n", ralink_gpio_led_stat[i].times);
		} else {
			printk("T@\n");
#endif
		}
	}

	for (i = 40; i <  RALINK_GPIO_NUMBER; i++) {
		ralink_gpio_led_stat[i].ticks++;
		if (ralink_gpio_led_data[i].gpio == -1) //-1 means unused
			continue;
		if (ralink_gpio_led_data[i].on == RALINK_GPIO_LED_INFINITY ||
				ralink_gpio_led_data[i].off == 0) { //always on
			__LED4540_ON(i);
			continue;
		}
		if (ralink_gpio_led_data[i].off == RALINK_GPIO_LED_INFINITY ||
				ralink_gpio_led_data[i].rests == RALINK_GPIO_LED_INFINITY ||
				ralink_gpio_led_data[i].on == 0 ||
				ralink_gpio_led_data[i].blinks == 0 ||
				ralink_gpio_led_data[i].times == 0) { //always off
			__LED4540_OFF(i);
			continue;
		}

		//led turn on or off
		if (ralink_gpio_led_data[i].blinks == RALINK_GPIO_LED_INFINITY ||
				ralink_gpio_led_data[i].rests == 0) { //always blinking
			x = ralink_gpio_led_stat[i].ticks % (ralink_gpio_led_data[i].on
					+ ralink_gpio_led_data[i].off);
		}
		else {
			unsigned int a, b, c, d, o, t;
			a = ralink_gpio_led_data[i].blinks / 2;
			b = ralink_gpio_led_data[i].rests / 2;
			c = ralink_gpio_led_data[i].blinks % 2;
			d = ralink_gpio_led_data[i].rests % 2;
			o = ralink_gpio_led_data[i].on + ralink_gpio_led_data[i].off;
			//t = blinking ticks
			t = a * o + ralink_gpio_led_data[i].on * c;
			//x = ticks % (blinking ticks + resting ticks)
			x = ralink_gpio_led_stat[i].ticks %
				(t + b * o + ralink_gpio_led_data[i].on * d);
			//starts from 0 at resting cycles
			if (x >= t)
				x -= t;
			x %= o;
		}
		if (x < ralink_gpio_led_data[i].on) {
			__LED4540_ON(i);
			if (ralink_gpio_led_stat[i].ticks && x == 0)
				ralink_gpio_led_stat[i].offs++;
#if RALINK_LED_DEBUG
			printk("t%d gpio%d on,", ralink_gpio_led_stat[i].ticks, i);
#endif
		}
		else {
			__LED4540_OFF(i);
			if (x == ralink_gpio_led_data[i].on)
				ralink_gpio_led_stat[i].ons++;
#if RALINK_LED_DEBUG
			printk("t%d gpio%d off,", ralink_gpio_led_stat[i].ticks, i);
#endif
		}

		//blinking or resting
		if (ralink_gpio_led_data[i].blinks == RALINK_GPIO_LED_INFINITY ||
				ralink_gpio_led_data[i].rests == 0) { //always blinking
			continue;
		}
		else {
			x = ralink_gpio_led_stat[i].ons + ralink_gpio_led_stat[i].offs;
			if (!ralink_gpio_led_stat[i].resting) {
				if (x == ralink_gpio_led_data[i].blinks) {
					ralink_gpio_led_stat[i].resting = 1;
					ralink_gpio_led_stat[i].ons = 0;
					ralink_gpio_led_stat[i].offs = 0;
					ralink_gpio_led_stat[i].times++;
				}
			}
			else {
				if (x == ralink_gpio_led_data[i].rests) {
					ralink_gpio_led_stat[i].resting = 0;
					ralink_gpio_led_stat[i].ons = 0;
					ralink_gpio_led_stat[i].offs = 0;
				}
			}
		}
		if (ralink_gpio_led_stat[i].resting) {
			__LED4540_OFF(i);
#if RALINK_LED_DEBUG
			printk("resting,");
		} else {
			printk("blinking,");
#endif
		}

		//number of times
		if (ralink_gpio_led_data[i].times != RALINK_GPIO_LED_INFINITY)
		{
			if (ralink_gpio_led_stat[i].times ==
					ralink_gpio_led_data[i].times) {
				__LED4540_OFF(i);
				ralink_gpio_led_data[i].gpio = -1; //stop
			}
#if RALINK_LED_DEBUG
			printk("T%d\n", ralink_gpio_led_stat[i].times);
		} else {
			printk("T@\n");
#endif
		}
	}
#elif defined (RALINK_GPIO_HAS_5124)
	for (i = 24; i < 40; i++) {
		ralink_gpio_led_stat[i].ticks++;
		if (ralink_gpio_led_data[i].gpio == -1) //-1 means unused
			continue;
		if (ralink_gpio_led_data[i].on == RALINK_GPIO_LED_INFINITY ||
				ralink_gpio_led_data[i].off == 0) { //always on
			__LED3924_ON(i);
			continue;
		}
		if (ralink_gpio_led_data[i].off == RALINK_GPIO_LED_INFINITY ||
				ralink_gpio_led_data[i].rests == RALINK_GPIO_LED_INFINITY ||
				ralink_gpio_led_data[i].on == 0 ||
				ralink_gpio_led_data[i].blinks == 0 ||
				ralink_gpio_led_data[i].times == 0) { //always off
			__LED3924_OFF(i);
			continue;
		}

		//led turn on or off
		if (ralink_gpio_led_data[i].blinks == RALINK_GPIO_LED_INFINITY ||
				ralink_gpio_led_data[i].rests == 0) { //always blinking
			x = ralink_gpio_led_stat[i].ticks % (ralink_gpio_led_data[i].on
					+ ralink_gpio_led_data[i].off);
		}
		else {
			unsigned int a, b, c, d, o, t;
			a = ralink_gpio_led_data[i].blinks / 2;
			b = ralink_gpio_led_data[i].rests / 2;
			c = ralink_gpio_led_data[i].blinks % 2;
			d = ralink_gpio_led_data[i].rests % 2;
			o = ralink_gpio_led_data[i].on + ralink_gpio_led_data[i].off;
			//t = blinking ticks
			t = a * o + ralink_gpio_led_data[i].on * c;
			//x = ticks % (blinking ticks + resting ticks)
			x = ralink_gpio_led_stat[i].ticks %
				(t + b * o + ralink_gpio_led_data[i].on * d);
			//starts from 0 at resting cycles
			if (x >= t)
				x -= t;
			x %= o;
		}
		if (x < ralink_gpio_led_data[i].on) {
			__LED3924_ON(i);
			if (ralink_gpio_led_stat[i].ticks && x == 0)
				ralink_gpio_led_stat[i].offs++;
#if RALINK_LED_DEBUG
			printk("t%d gpio%d on,", ralink_gpio_led_stat[i].ticks, i);
#endif
		}
		else {
			__LED3924_OFF(i);
			if (x == ralink_gpio_led_data[i].on)
				ralink_gpio_led_stat[i].ons++;
#if RALINK_LED_DEBUG
			printk("t%d gpio%d off,", ralink_gpio_led_stat[i].ticks, i);
#endif
		}

		//blinking or resting
		if (ralink_gpio_led_data[i].blinks == RALINK_GPIO_LED_INFINITY ||
				ralink_gpio_led_data[i].rests == 0) { //always blinking
			continue;
		}
		else {
			x = ralink_gpio_led_stat[i].ons + ralink_gpio_led_stat[i].offs;
			if (!ralink_gpio_led_stat[i].resting) {
				if (x == ralink_gpio_led_data[i].blinks) {
					ralink_gpio_led_stat[i].resting = 1;
					ralink_gpio_led_stat[i].ons = 0;
					ralink_gpio_led_stat[i].offs = 0;
					ralink_gpio_led_stat[i].times++;
				}
			}
			else {
				if (x == ralink_gpio_led_data[i].rests) {
					ralink_gpio_led_stat[i].resting = 0;
					ralink_gpio_led_stat[i].ons = 0;
					ralink_gpio_led_stat[i].offs = 0;
				}
			}
		}
		if (ralink_gpio_led_stat[i].resting) {
			__LED3924_OFF(i);
#if RALINK_LED_DEBUG
			printk("resting,");
		} else {
			printk("blinking,");
#endif
		}

		//number of times
		if (ralink_gpio_led_data[i].times != RALINK_GPIO_LED_INFINITY)
		{
			if (ralink_gpio_led_stat[i].times ==
					ralink_gpio_led_data[i].times) {
				__LED3924_OFF(i);
				ralink_gpio_led_data[i].gpio = -1; //stop
			}
#if RALINK_LED_DEBUG
			printk("T%d\n", ralink_gpio_led_stat[i].times);
		} else {
			printk("T@\n");
#endif
		}
	}

	for (i = 40; i < RALINK_GPIO_NUMBER; i++) {
		ralink_gpio_led_stat[i].ticks++;
		if (ralink_gpio_led_data[i].gpio == -1) //-1 means unused
			continue;
		if (ralink_gpio_led_data[i].on == RALINK_GPIO_LED_INFINITY ||
				ralink_gpio_led_data[i].off == 0) { //always on
			__LED5140_ON(i);
			continue;
		}
		if (ralink_gpio_led_data[i].off == RALINK_GPIO_LED_INFINITY ||
				ralink_gpio_led_data[i].rests == RALINK_GPIO_LED_INFINITY ||
				ralink_gpio_led_data[i].on == 0 ||
				ralink_gpio_led_data[i].blinks == 0 ||
				ralink_gpio_led_data[i].times == 0) { //always off
			__LED5140_OFF(i);
			continue;
		}

		//led turn on or off
		if (ralink_gpio_led_data[i].blinks == RALINK_GPIO_LED_INFINITY ||
				ralink_gpio_led_data[i].rests == 0) { //always blinking
			x = ralink_gpio_led_stat[i].ticks % (ralink_gpio_led_data[i].on
					+ ralink_gpio_led_data[i].off);
		}
		else {
			unsigned int a, b, c, d, o, t;
			a = ralink_gpio_led_data[i].blinks / 2;
			b = ralink_gpio_led_data[i].rests / 2;
			c = ralink_gpio_led_data[i].blinks % 2;
			d = ralink_gpio_led_data[i].rests % 2;
			o = ralink_gpio_led_data[i].on + ralink_gpio_led_data[i].off;
			//t = blinking ticks
			t = a * o + ralink_gpio_led_data[i].on * c;
			//x = ticks % (blinking ticks + resting ticks)
			x = ralink_gpio_led_stat[i].ticks %
				(t + b * o + ralink_gpio_led_data[i].on * d);
			//starts from 0 at resting cycles
			if (x >= t)
				x -= t;
			x %= o;
		}
		if (x < ralink_gpio_led_data[i].on) {
			__LED5140_ON(i);
			if (ralink_gpio_led_stat[i].ticks && x == 0)
				ralink_gpio_led_stat[i].offs++;
#if RALINK_LED_DEBUG
			printk("t%d gpio%d on,", ralink_gpio_led_stat[i].ticks, i);
#endif
		}
		else {
			__LED5140_OFF(i);
			if (x == ralink_gpio_led_data[i].on)
				ralink_gpio_led_stat[i].ons++;
#if RALINK_LED_DEBUG
			printk("t%d gpio%d off,", ralink_gpio_led_stat[i].ticks, i);
#endif
		}

		//blinking or resting
		if (ralink_gpio_led_data[i].blinks == RALINK_GPIO_LED_INFINITY ||
				ralink_gpio_led_data[i].rests == 0) { //always blinking
			continue;
		}
		else {
			x = ralink_gpio_led_stat[i].ons + ralink_gpio_led_stat[i].offs;
			if (!ralink_gpio_led_stat[i].resting) {
				if (x == ralink_gpio_led_data[i].blinks) {
					ralink_gpio_led_stat[i].resting = 1;
					ralink_gpio_led_stat[i].ons = 0;
					ralink_gpio_led_stat[i].offs = 0;
					ralink_gpio_led_stat[i].times++;
				}
			}
			else {
				if (x == ralink_gpio_led_data[i].rests) {
					ralink_gpio_led_stat[i].resting = 0;
					ralink_gpio_led_stat[i].ons = 0;
					ralink_gpio_led_stat[i].offs = 0;
				}
			}
		}
		if (ralink_gpio_led_stat[i].resting) {
			__LED5140_OFF(i);
#if RALINK_LED_DEBUG
			printk("resting,");
		} else {
			printk("blinking,");
#endif
		}

		//number of times
		if (ralink_gpio_led_data[i].times != RALINK_GPIO_LED_INFINITY)
		{
			if (ralink_gpio_led_stat[i].times ==
					ralink_gpio_led_data[i].times) {
				__LED5140_OFF(i);
				ralink_gpio_led_data[i].gpio = -1; //stop
			}
#if RALINK_LED_DEBUG
			printk("T%d\n", ralink_gpio_led_stat[i].times);
		} else {
			printk("T@\n");
#endif
		}
	}
#elif defined (RALINK_GPIO_HAS_9524) || defined (RALINK_GPIO_HAS_7224)
	for (i = 24; i < 40; i++) {
		ralink_gpio_led_stat[i].ticks++;
		if (ralink_gpio_led_data[i].gpio == -1) //-1 means unused
			continue;
		if (ralink_gpio_led_data[i].on == RALINK_GPIO_LED_INFINITY ||
				ralink_gpio_led_data[i].off == 0) { //always on
			__LED3924_ON(i);
			continue;
		}
		if (ralink_gpio_led_data[i].off == RALINK_GPIO_LED_INFINITY ||
				ralink_gpio_led_data[i].rests == RALINK_GPIO_LED_INFINITY ||
				ralink_gpio_led_data[i].on == 0 ||
				ralink_gpio_led_data[i].blinks == 0 ||
				ralink_gpio_led_data[i].times == 0) { //always off
			__LED3924_OFF(i);
			continue;
		}

		//led turn on or off
		if (ralink_gpio_led_data[i].blinks == RALINK_GPIO_LED_INFINITY ||
				ralink_gpio_led_data[i].rests == 0) { //always blinking
			x = ralink_gpio_led_stat[i].ticks % (ralink_gpio_led_data[i].on
					+ ralink_gpio_led_data[i].off);
		}
		else {
			unsigned int a, b, c, d, o, t;
			a = ralink_gpio_led_data[i].blinks / 2;
			b = ralink_gpio_led_data[i].rests / 2;
			c = ralink_gpio_led_data[i].blinks % 2;
			d = ralink_gpio_led_data[i].rests % 2;
			o = ralink_gpio_led_data[i].on + ralink_gpio_led_data[i].off;
			//t = blinking ticks
			t = a * o + ralink_gpio_led_data[i].on * c;
			//x = ticks % (blinking ticks + resting ticks)
			x = ralink_gpio_led_stat[i].ticks %
				(t + b * o + ralink_gpio_led_data[i].on * d);
			//starts from 0 at resting cycles
			if (x >= t)
				x -= t;
			x %= o;
		}
		if (x < ralink_gpio_led_data[i].on) {
			__LED3924_ON(i);
			if (ralink_gpio_led_stat[i].ticks && x == 0)
				ralink_gpio_led_stat[i].offs++;
#if RALINK_LED_DEBUG
			printk("t%d gpio%d on,", ralink_gpio_led_stat[i].ticks, i);
#endif
		}
		else {
			__LED3924_OFF(i);
			if (x == ralink_gpio_led_data[i].on)
				ralink_gpio_led_stat[i].ons++;
#if RALINK_LED_DEBUG
			printk("t%d gpio%d off,", ralink_gpio_led_stat[i].ticks, i);
#endif
		}

		//blinking or resting
		if (ralink_gpio_led_data[i].blinks == RALINK_GPIO_LED_INFINITY ||
				ralink_gpio_led_data[i].rests == 0) { //always blinking
			continue;
		}
		else {
			x = ralink_gpio_led_stat[i].ons + ralink_gpio_led_stat[i].offs;
			if (!ralink_gpio_led_stat[i].resting) {
				if (x == ralink_gpio_led_data[i].blinks) {
					ralink_gpio_led_stat[i].resting = 1;
					ralink_gpio_led_stat[i].ons = 0;
					ralink_gpio_led_stat[i].offs = 0;
					ralink_gpio_led_stat[i].times++;
				}
			}
			else {
				if (x == ralink_gpio_led_data[i].rests) {
					ralink_gpio_led_stat[i].resting = 0;
					ralink_gpio_led_stat[i].ons = 0;
					ralink_gpio_led_stat[i].offs = 0;
				}
			}
		}
		if (ralink_gpio_led_stat[i].resting) {
			__LED3924_OFF(i);
#if RALINK_LED_DEBUG
			printk("resting,");
		} else {
			printk("blinking,");
#endif
		}

		//number of times
		if (ralink_gpio_led_data[i].times != RALINK_GPIO_LED_INFINITY)
		{
			if (ralink_gpio_led_stat[i].times ==
					ralink_gpio_led_data[i].times) {
				__LED3924_OFF(i);
				ralink_gpio_led_data[i].gpio = -1; //stop
			}
#if RALINK_LED_DEBUG
			printk("T%d\n", ralink_gpio_led_stat[i].times);
		} else {
			printk("T@\n");
#endif
		}
	}

	for (i = 40; i < 72; i++) {
		ralink_gpio_led_stat[i].ticks++;
		if (ralink_gpio_led_data[i].gpio == -1) //-1 means unused
			continue;
		if (ralink_gpio_led_data[i].on == RALINK_GPIO_LED_INFINITY ||
				ralink_gpio_led_data[i].off == 0) { //always on
			__LED7140_ON(i);
			continue;
		}
		if (ralink_gpio_led_data[i].off == RALINK_GPIO_LED_INFINITY ||
				ralink_gpio_led_data[i].rests == RALINK_GPIO_LED_INFINITY ||
				ralink_gpio_led_data[i].on == 0 ||
				ralink_gpio_led_data[i].blinks == 0 ||
				ralink_gpio_led_data[i].times == 0) { //always off
			__LED7140_OFF(i);
			continue;
		}

		//led turn on or off
		if (ralink_gpio_led_data[i].blinks == RALINK_GPIO_LED_INFINITY ||
				ralink_gpio_led_data[i].rests == 0) { //always blinking
			x = ralink_gpio_led_stat[i].ticks % (ralink_gpio_led_data[i].on
					+ ralink_gpio_led_data[i].off);
		}
		else {
			unsigned int a, b, c, d, o, t;
			a = ralink_gpio_led_data[i].blinks / 2;
			b = ralink_gpio_led_data[i].rests / 2;
			c = ralink_gpio_led_data[i].blinks % 2;
			d = ralink_gpio_led_data[i].rests % 2;
			o = ralink_gpio_led_data[i].on + ralink_gpio_led_data[i].off;
			//t = blinking ticks
			t = a * o + ralink_gpio_led_data[i].on * c;
			//x = ticks % (blinking ticks + resting ticks)
			x = ralink_gpio_led_stat[i].ticks %
				(t + b * o + ralink_gpio_led_data[i].on * d);
			//starts from 0 at resting cycles
			if (x >= t)
				x -= t;
			x %= o;
		}
		if (x < ralink_gpio_led_data[i].on) {
			__LED7140_ON(i);
			if (ralink_gpio_led_stat[i].ticks && x == 0)
				ralink_gpio_led_stat[i].offs++;
#if RALINK_LED_DEBUG
			printk("t%d gpio%d on,", ralink_gpio_led_stat[i].ticks, i);
#endif
		}
		else {
			__LED7140_OFF(i);
			if (x == ralink_gpio_led_data[i].on)
				ralink_gpio_led_stat[i].ons++;
#if RALINK_LED_DEBUG
			printk("t%d gpio%d off,", ralink_gpio_led_stat[i].ticks, i);
#endif
		}

		//blinking or resting
		if (ralink_gpio_led_data[i].blinks == RALINK_GPIO_LED_INFINITY ||
				ralink_gpio_led_data[i].rests == 0) { //always blinking
			continue;
		}
		else {
			x = ralink_gpio_led_stat[i].ons + ralink_gpio_led_stat[i].offs;
			if (!ralink_gpio_led_stat[i].resting) {
				if (x == ralink_gpio_led_data[i].blinks) {
					ralink_gpio_led_stat[i].resting = 1;
					ralink_gpio_led_stat[i].ons = 0;
					ralink_gpio_led_stat[i].offs = 0;
					ralink_gpio_led_stat[i].times++;
				}
			}
			else {
				if (x == ralink_gpio_led_data[i].rests) {
					ralink_gpio_led_stat[i].resting = 0;
					ralink_gpio_led_stat[i].ons = 0;
					ralink_gpio_led_stat[i].offs = 0;
				}
			}
		}
		if (ralink_gpio_led_stat[i].resting) {
			__LED7140_OFF(i);
#if RALINK_LED_DEBUG
			printk("resting,");
		} else {
			printk("blinking,");
#endif
		}

		//number of times
		if (ralink_gpio_led_data[i].times != RALINK_GPIO_LED_INFINITY)
		{
			if (ralink_gpio_led_stat[i].times ==
					ralink_gpio_led_data[i].times) {
				__LED7140_OFF(i);
				ralink_gpio_led_data[i].gpio = -1; //stop
			}
#if RALINK_LED_DEBUG
			printk("T%d\n", ralink_gpio_led_stat[i].times);
		} else {
			printk("T@\n");
#endif
		}
	}

	for (i = 72; i < RALINK_GPIO_NUMBER; i++) {
		ralink_gpio_led_stat[i].ticks++;
		if (ralink_gpio_led_data[i].gpio == -1) //-1 means unused
			continue;
		if (ralink_gpio_led_data[i].on == RALINK_GPIO_LED_INFINITY ||
				ralink_gpio_led_data[i].off == 0) { //always on
#if defined (RALINK_GPIO_HAS_7224)
			__LED72_ON(i);
#else
			__LED9572_ON(i);
#endif
			continue;
		}
		if (ralink_gpio_led_data[i].off == RALINK_GPIO_LED_INFINITY ||
				ralink_gpio_led_data[i].rests == RALINK_GPIO_LED_INFINITY ||
				ralink_gpio_led_data[i].on == 0 ||
				ralink_gpio_led_data[i].blinks == 0 ||
				ralink_gpio_led_data[i].times == 0) { //always off
#if defined (RALINK_GPIO_HAS_7224)
			__LED72_OFF(i);
#else
			__LED9572_OFF(i);
#endif
			continue;
		}

		//led turn on or off
		if (ralink_gpio_led_data[i].blinks == RALINK_GPIO_LED_INFINITY ||
				ralink_gpio_led_data[i].rests == 0) { //always blinking
			x = ralink_gpio_led_stat[i].ticks % (ralink_gpio_led_data[i].on
					+ ralink_gpio_led_data[i].off);
		}
		else {
			unsigned int a, b, c, d, o, t;
			a = ralink_gpio_led_data[i].blinks / 2;
			b = ralink_gpio_led_data[i].rests / 2;
			c = ralink_gpio_led_data[i].blinks % 2;
			d = ralink_gpio_led_data[i].rests % 2;
			o = ralink_gpio_led_data[i].on + ralink_gpio_led_data[i].off;
			//t = blinking ticks
			t = a * o + ralink_gpio_led_data[i].on * c;
			//x = ticks % (blinking ticks + resting ticks)
			x = ralink_gpio_led_stat[i].ticks %
				(t + b * o + ralink_gpio_led_data[i].on * d);
			//starts from 0 at resting cycles
			if (x >= t)
				x -= t;
			x %= o;
		}
		if (x < ralink_gpio_led_data[i].on) {
#if defined (RALINK_GPIO_HAS_7224)
			__LED72_ON(i);
#else
			__LED9572_ON(i);
#endif
			if (ralink_gpio_led_stat[i].ticks && x == 0)
				ralink_gpio_led_stat[i].offs++;
#if RALINK_LED_DEBUG
			printk("t%d gpio%d on,", ralink_gpio_led_stat[i].ticks, i);
#endif
		}
		else {
#if defined (RALINK_GPIO_HAS_7224)
			__LED72_OFF(i);
#else
			__LED9572_OFF(i);
#endif
			if (x == ralink_gpio_led_data[i].on)
				ralink_gpio_led_stat[i].ons++;
#if RALINK_LED_DEBUG
			printk("t%d gpio%d off,", ralink_gpio_led_stat[i].ticks, i);
#endif
		}

		//blinking or resting
		if (ralink_gpio_led_data[i].blinks == RALINK_GPIO_LED_INFINITY ||
				ralink_gpio_led_data[i].rests == 0) { //always blinking
			continue;
		}
		else {
			x = ralink_gpio_led_stat[i].ons + ralink_gpio_led_stat[i].offs;
			if (!ralink_gpio_led_stat[i].resting) {
				if (x == ralink_gpio_led_data[i].blinks) {
					ralink_gpio_led_stat[i].resting = 1;
					ralink_gpio_led_stat[i].ons = 0;
					ralink_gpio_led_stat[i].offs = 0;
					ralink_gpio_led_stat[i].times++;
				}
			}
			else {
				if (x == ralink_gpio_led_data[i].rests) {
					ralink_gpio_led_stat[i].resting = 0;
					ralink_gpio_led_stat[i].ons = 0;
					ralink_gpio_led_stat[i].offs = 0;
				}
			}
		}
		if (ralink_gpio_led_stat[i].resting) {
#if defined (RALINK_GPIO_HAS_7224)
			__LED72_OFF(i);
#else
			__LED9572_OFF(i);
#endif
#if RALINK_LED_DEBUG
			printk("resting,");
		} else {
			printk("blinking,");
#endif
		}

		//number of times
		if (ralink_gpio_led_data[i].times != RALINK_GPIO_LED_INFINITY)
		{
			if (ralink_gpio_led_stat[i].times ==
					ralink_gpio_led_data[i].times) {
#if defined (RALINK_GPIO_HAS_7224)
				__LED72_OFF(i);
#else
				__LED9572_OFF(i);
#endif
				ralink_gpio_led_data[i].gpio = -1; //stop
			}
#if RALINK_LED_DEBUG
			printk("T%d\n", ralink_gpio_led_stat[i].times);
		} else {
			printk("T@\n");
#endif
		}
	}
#endif
#endif

	//always turn the power LED on
#ifdef CONFIG_RALINK_RT2880
	__LED_ON(12);
#elif defined (CONFIG_RALINK_RT3052) || defined (CONFIG_RALINK_RT2883)
	__LED_ON(9);
#endif

	*(volatile u32 *)(RALINK_REG_PIORESET) = ra_gpio_led_clr;
	*(volatile u32 *)(RALINK_REG_PIOSET) = ra_gpio_led_set;
#if defined (RALINK_GPIO_HAS_2722)
	*(volatile u32 *)(RALINK_REG_PIO2722RESET) = ra_gpio2722_led_clr;
	*(volatile u32 *)(RALINK_REG_PIO2722SET) = ra_gpio2722_led_set;
#elif defined (RALINK_GPIO_HAS_4524)
	*(volatile u32 *)(RALINK_REG_PIO3924RESET) = ra_gpio3924_led_clr;
	*(volatile u32 *)(RALINK_REG_PIO3924SET) = ra_gpio3924_led_set;
	*(volatile u32 *)(RALINK_REG_PIO4540RESET) = ra_gpio4540_led_clr;
	*(volatile u32 *)(RALINK_REG_PIO4540SET) = ra_gpio4540_led_set;
#elif defined (RALINK_GPIO_HAS_5124)
	*(volatile u32 *)(RALINK_REG_PIO3924RESET) = ra_gpio3924_led_clr;
	*(volatile u32 *)(RALINK_REG_PIO3924SET) = ra_gpio3924_led_set;
	*(volatile u32 *)(RALINK_REG_PIO5140RESET) = ra_gpio5140_led_clr;
	*(volatile u32 *)(RALINK_REG_PIO5140SET) = ra_gpio5140_led_set;
#elif defined (RALINK_GPIO_HAS_9524) || defined (RALINK_GPIO_HAS_7224)
	*(volatile u32 *)(RALINK_REG_PIO3924RESET) = ra_gpio3924_led_clr;
	*(volatile u32 *)(RALINK_REG_PIO3924SET) = ra_gpio3924_led_set;
	*(volatile u32 *)(RALINK_REG_PIO7140RESET) = ra_gpio7140_led_clr;
	*(volatile u32 *)(RALINK_REG_PIO7140SET) = ra_gpio7140_led_set;
#if defined (RALINK_GPIO_HAS_7224)
	*(volatile u32 *)(RALINK_REG_PIO72RESET) = ra_gpio72_led_clr;
	*(volatile u32 *)(RALINK_REG_PIO72SET) = ra_gpio72_led_set;
#else
	*(volatile u32 *)(RALINK_REG_PIO9572RESET) = ra_gpio9572_led_clr;
	*(volatile u32 *)(RALINK_REG_PIO9572SET) = ra_gpio9572_led_set;
#endif
#elif defined (RALINK_GPIO_HAS_9532)
	*(volatile u32 *)(RALINK_REG_PIO6332RESET) = ra_gpio6332_led_clr;
	*(volatile u32 *)(RALINK_REG_PIO6332SET) = ra_gpio6332_led_set;
	*(volatile u32 *)(RALINK_REG_PIO9564RESET) = ra_gpio9564_led_clr;
	*(volatile u32 *)(RALINK_REG_PIO9564SET) = ra_gpio9564_led_set;
#endif

#if RALINK_LED_DEBUG
	printk("led_set= %x, led_clr= %x\n", ra_gpio_led_set, ra_gpio_led_clr);
#if defined (RALINK_GPIO_HAS_2722)
	printk("led2722_set= %x, led2722_clr= %x\n", ra_gpio2722_led_set, ra_gpio2722_led_clr);
#elif defined (RALINK_GPIO_HAS_4524)
	printk("led3924_set= %x, led3924_clr= %x\n", ra_gpio3924_led_set, ra_gpio3924_led_clr);
	printk("led4540_set= %x, led4540_clr= %x\n", ra_gpio4540_led_set, ra_gpio4540_led_set);
#elif defined (RALINK_GPIO_HAS_5124)
	printk("led3924_set= %x, led3924_clr= %x\n", ra_gpio3924_led_set, ra_gpio3924_led_clr);
	printk("led5140_set= %x, led5140_clr= %x\n", ra_gpio5140_led_set, ra_gpio5140_led_set);
#elif defined (RALINK_GPIO_HAS_9524) || defined (RALINK_GPIO_HAS_7224)
	printk("led3924_set= %x, led3924_clr= %x\n", ra_gpio3924_led_set, ra_gpio3924_led_clr);
	printk("led7140_set= %x, led7140_clr= %x\n", ra_gpio7140_led_set, ra_gpio7140_led_set);
#if defined (RALINK_GPIO_HAS_7224)
	printk("led72_set= %x, led72_clr= %x\n", ra_gpio72_led_set, ra_gpio72_led_set);
#else
	printk("led9572_set= %x, led9572_clr= %x\n", ra_gpio9572_led_set, ra_gpio9572_led_set);
#endif
#elif defined (RALINK_GPIO_HAS_9532)
	printk("led6332_set= %x, led6332_clr= %x\n", ra_gpio6332_led_set, ra_gpio6332_led_clr);
	printk("led9564_set= %x, led9564_clr= %x\n", ra_gpio9564_led_set, ra_gpio9564_led_set);
#endif
#endif

	ra_gpio_led_set = ra_gpio_led_clr = 0;
#if defined (RALINK_GPIO_HAS_2722)
	ra_gpio2722_led_set = ra_gpio2722_led_clr = 0;
#elif defined (RALINK_GPIO_HAS_4524)
	ra_gpio3924_led_set = ra_gpio3924_led_clr = 0;
	ra_gpio4540_led_set = ra_gpio4540_led_clr = 0;
#elif defined (RALINK_GPIO_HAS_5124)
	ra_gpio3924_led_set = ra_gpio3924_led_clr = 0;
	ra_gpio5140_led_set = ra_gpio5140_led_clr = 0;
#elif defined (RALINK_GPIO_HAS_9524) || defined (RALINK_GPIO_HAS_7224)
	ra_gpio3924_led_set = ra_gpio3924_led_clr = 0;
	ra_gpio7140_led_set = ra_gpio7140_led_clr = 0;
#if defined (RALINK_GPIO_HAS_7224)
	ra_gpio72_led_set = ra_gpio72_led_clr = 0;
#else
	ra_gpio9572_led_set = ra_gpio9572_led_clr = 0;
#endif
#elif defined (RALINK_GPIO_HAS_9532)
	ra_gpio6332_led_set = ra_gpio6332_led_clr = 0;
	ra_gpio9564_led_set = ra_gpio9564_led_clr = 0;
#endif

	init_timer(&ralink_gpio_led_timer);
	ralink_gpio_led_timer.expires = jiffies + RALINK_GPIO_LED_FREQ;
	add_timer(&ralink_gpio_led_timer);
}

void ralink_gpio_led_init_timer(void)
{
	int i;

	for (i = 0; i < RALINK_GPIO_NUMBER; i++)
		ralink_gpio_led_data[i].gpio = -1; //-1 means unused
#if RALINK_GPIO_LED_LOW_ACT
	ra_gpio_led_set = 0xffffff;
#if defined (RALINK_GPIO_HAS_2722)
	ra_gpio2722_led_set = 0xff;
#elif defined (RALINK_GPIO_HAS_4524)
	ra_gpio3924_led_set = 0xffff;
	ra_gpio4540_led_set = 0xff;
#elif defined (RALINK_GPIO_HAS_5124)
	ra_gpio3924_led_set = 0xffff;
	ra_gpio5140_led_set = 0xfff;
#elif defined (RALINK_GPIO_HAS_9524) || defined (RALINK_GPIO_HAS_7224)
	ra_gpio3924_led_set = 0xffff;
	ra_gpio7140_led_set = 0xffffffff;
#if defined (RALINK_GPIO_HAS_7224)
	ra_gpio72_led_set = 0xffffff;
#else
	ra_gpio9572_led_set = 0xffffff;
#endif
#elif defined (RALINK_GPIO_HAS_9532)
	ra_gpio6332_led_set = 0xffffffff;
	ra_gpio9564_led_set = 0xffffffff;
#endif
#else // RALINK_GPIO_LED_LOW_ACT //
//WNC0_tsunghao Mon 09 Sep 2013 07:43:40 PM CST, HW define is difference from RT5350 EVB (WNC HW is Low active)
#if WNC_DEL
	ra_gpio_led_clr = 0xffffff;
#endif
//WNC0
#if defined (RALINK_GPIO_HAS_2722)
//WNC0_tsunghao Mon 09 Sep 2013 07:43:40 PM CST, HW define is difference from RT5350 EVB (WNC HW is Low active)
#if WNC_DEL
	ra_gpio2722_led_clr = 0xff;
#endif
//WNC0
#elif defined (RALINK_GPIO_HAS_4524)
	ra_gpio3924_led_clr = 0xffff;
	ra_gpio4540_led_clr = 0xff;
#elif defined (RALINK_GPIO_HAS_5124)
	ra_gpio3924_led_clr = 0xffff;
	ra_gpio5140_led_clr = 0xfff;
#elif defined (RALINK_GPIO_HAS_9524) || defined (RALINK_GPIO_HAS_7224)
	ra_gpio3924_led_clr = 0xffff;
	ra_gpio7140_led_clr = 0xffffffff;
#if defined (RALINK_GPIO_HAS_7224)
	ra_gpio72_led_clr = 0xffffff;
#else
	ra_gpio9572_led_clr = 0xffffff;
#endif
#elif defined (RALINK_GPIO_HAS_9532)
	ra_gpio6332_led_clr = 0xffffffff;
	ra_gpio9564_led_clr = 0xffffffff;
#endif
#endif // RALINK_GPIO_LED_LOW_ACT //

	init_timer(&ralink_gpio_led_timer);
	ralink_gpio_led_timer.function = ralink_gpio_led_do_timer;
	ralink_gpio_led_timer.expires = jiffies + RALINK_GPIO_LED_FREQ;
	add_timer(&ralink_gpio_led_timer);
}
#endif

int __init ralink_gpio_init(void)
{
	unsigned int i;
	u32 gpiomode;

#ifdef  CONFIG_DEVFS_FS
	if (devfs_register_chrdev(ralink_gpio_major, RALINK_GPIO_DEVNAME,
				&ralink_gpio_fops)) {
		printk(KERN_ERR NAME ": unable to register character device\n");
		return -EIO;
	}
	devfs_handle = devfs_register(NULL, RALINK_GPIO_DEVNAME,
			DEVFS_FL_DEFAULT, ralink_gpio_major, 0,
			S_IFCHR | S_IRUGO | S_IWUGO, &ralink_gpio_fops, NULL);
#else
	int r = 0;
	r = register_chrdev(ralink_gpio_major, RALINK_GPIO_DEVNAME,
			&ralink_gpio_fops);
	if (r < 0) {
		printk(KERN_ERR NAME ": unable to register character device\n");
		return r;
	}
	if (ralink_gpio_major == 0) {
		ralink_gpio_major = r;
		printk(KERN_DEBUG NAME ": got dynamic major %d\n", r);
	}
#endif

	//config these pins to gpio mode
	gpiomode = le32_to_cpu(*(volatile u32 *)(RALINK_REG_GPIOMODE));
	// printk(KERN_EMERG "111 gpiomode = [0x%08X]\n", gpiomode);
#if !defined (CONFIG_RALINK_RT2880)
	gpiomode &= ~0x1C;  //clear bit[2:4]UARTF_SHARE_MODE
#endif
#if defined (CONFIG_RALINK_MT7620)
	gpiomode &= ~0x2000;  //clear bit[13] WLAN_LED
#endif
	gpiomode |= RALINK_GPIOMODE_DFT;
	*(volatile u32 *)(RALINK_REG_GPIOMODE) = cpu_to_le32(gpiomode);

	gpiomode = le32_to_cpu(*(volatile u32 *)(RALINK_REG_GPIOMODE));
	// printk(KERN_EMERG "222 gpiomode = [0x%08X]\n", gpiomode);

//WNC0_tsunghao Wed 21 Aug 2013 07:15:43 AM CST
#if 0
	*(volatile u32 *)(RALINK_REG_GPIOMODE) |= (1<<0);
#endif

	gpiomode = le32_to_cpu(*(volatile u32 *)(RALINK_REG_GPIOMODE));
	// printk(KERN_EMERG "333 gpiomode = [0x%08X]\n", gpiomode);

	//enable gpio interrupt
	*(volatile u32 *)(RALINK_REG_INTENA) = cpu_to_le32(RALINK_INTCTL_PIO);
	for (i = 0; i < RALINK_GPIO_NUMBER; i++) {
		ralink_gpio_info[i].irq = i;
		ralink_gpio_info[i].pid = 0;
	}

#ifdef CONFIG_RALINK_GPIO_LED
	ralink_gpio_led_init_timer();
#endif
	gpiomode = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIODIR));
	gpiomode &= ~(1<<11);
	*(volatile u32 *)(RALINK_REG_PIODIR) = cpu_to_le32(gpiomode);
	printk(KERN_EMERG "[GPIO_INIT] piodata = [0x%08X]\n", gpiomode);
	printk("Ralink gpio driver initialized\n");
	LED_init();                                                                        
	return 0;
}

void __exit ralink_gpio_exit(void)
{
#ifdef  CONFIG_DEVFS_FS
	devfs_unregister_chrdev(ralink_gpio_major, RALINK_GPIO_DEVNAME);
	devfs_unregister(devfs_handle);
#else
	unregister_chrdev(ralink_gpio_major, RALINK_GPIO_DEVNAME);
#endif

	//config these pins to normal mode
	*(volatile u32 *)(RALINK_REG_GPIOMODE) &= ~RALINK_GPIOMODE_DFT;
	//disable gpio interrupt
	*(volatile u32 *)(RALINK_REG_INTDIS) = cpu_to_le32(RALINK_INTCTL_PIO);
#ifdef CONFIG_RALINK_GPIO_LED
	del_timer(&ralink_gpio_led_timer);
#endif
	printk("Ralink gpio driver exited\n");
}

/*
 * send a signal(SIGUSR1) to the registered user process whenever any gpio
 * interrupt comes
 * (called by interrupt handler)
 */
void ralink_gpio_notify_user(int usr)
{
	struct task_struct *p = NULL;

	if (ralink_gpio_irqnum < 0 || RALINK_GPIO_NUMBER <= ralink_gpio_irqnum) {
		printk(KERN_ERR NAME ": gpio irq number out of range\n");
		return;
	}

	//don't send any signal if pid is 0 or 1
	if ((int)ralink_gpio_info[ralink_gpio_irqnum].pid < 2)
		return;
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,35)
	p = find_task_by_vpid(ralink_gpio_info[ralink_gpio_irqnum].pid);
#else
	p = find_task_by_pid(ralink_gpio_info[ralink_gpio_irqnum].pid);
#endif

	if (NULL == p) {
		printk(KERN_ERR NAME ": no registered process to notify\n");
		return;
	}

	if (usr == 1) {
		printk(KERN_NOTICE NAME ": sending a SIGUSR1 to process %d\n",
				ralink_gpio_info[ralink_gpio_irqnum].pid);
		send_sig(SIGUSR1, p, 0);
	}
	else if (usr == 2) {
		printk(KERN_NOTICE NAME ": sending a SIGUSR2 to process %d\n",
				ralink_gpio_info[ralink_gpio_irqnum].pid);
		send_sig(SIGUSR2, p, 0);
	}
}

/*
 * 1. save the PIOINT and PIOEDGE value
 * 2. clear PIOINT by writing 1
 * (called by interrupt handler)
 */
void ralink_gpio_save_clear_intp(void)
{
	ralink_gpio_intp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIOINT));
	ralink_gpio_edge = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIOEDGE));
	*(volatile u32 *)(RALINK_REG_PIOINT) = cpu_to_le32(0x00FFFFFF);
	*(volatile u32 *)(RALINK_REG_PIOEDGE) = cpu_to_le32(0x00FFFFFF);
#if defined (RALINK_GPIO_HAS_2722)
	ralink_gpio2722_intp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO2722INT));
	ralink_gpio2722_edge = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO2722EDGE));
	*(volatile u32 *)(RALINK_REG_PIO2722INT) = cpu_to_le32(0x0000FFFF);
	*(volatile u32 *)(RALINK_REG_PIO2722EDGE) = cpu_to_le32(0x0000FFFF);
#elif defined (RALINK_GPIO_HAS_4524)
	ralink_gpio3924_intp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO3924INT));
	ralink_gpio3924_edge = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO3924EDGE));
	*(volatile u32 *)(RALINK_REG_PIO3924INT) = cpu_to_le32(0x0000FFFF);
	*(volatile u32 *)(RALINK_REG_PIO3924EDGE) = cpu_to_le32(0x0000FFFF);
	ralink_gpio4540_intp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO4540INT));
	ralink_gpio4540_edge = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO4540EDGE));
	*(volatile u32 *)(RALINK_REG_PIO4540INT) = cpu_to_le32(0x00000FFF);
	*(volatile u32 *)(RALINK_REG_PIO4540EDGE) = cpu_to_le32(0x00000FFF);
#elif defined (RALINK_GPIO_HAS_5124)
	ralink_gpio3924_intp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO3924INT));
	ralink_gpio3924_edge = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO3924EDGE));
	*(volatile u32 *)(RALINK_REG_PIO3924INT) = cpu_to_le32(0x0000FFFF);
	*(volatile u32 *)(RALINK_REG_PIO3924EDGE) = cpu_to_le32(0x0000FFFF);
	ralink_gpio5140_intp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO5140INT));
	ralink_gpio5140_edge = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO5140EDGE));
	*(volatile u32 *)(RALINK_REG_PIO5140INT) = cpu_to_le32(0x00000FFF);
	*(volatile u32 *)(RALINK_REG_PIO5140EDGE) = cpu_to_le32(0x00000FFF);
#elif defined (RALINK_GPIO_HAS_9524) || defined (RALINK_GPIO_HAS_7224)
	ralink_gpio3924_intp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO3924INT));
	ralink_gpio3924_edge = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO3924EDGE));
	*(volatile u32 *)(RALINK_REG_PIO3924INT) = cpu_to_le32(0x0000FFFF);
	*(volatile u32 *)(RALINK_REG_PIO3924EDGE) = cpu_to_le32(0x0000FFFF);
	ralink_gpio7140_intp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO7140INT));
	ralink_gpio7140_edge = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO7140EDGE));
	*(volatile u32 *)(RALINK_REG_PIO7140INT) = cpu_to_le32(0xFFFFFFFF);
	*(volatile u32 *)(RALINK_REG_PIO7140EDGE) = cpu_to_le32(0xFFFFFFFF);
#if defined (RALINK_GPIO_HAS_7224)
	ralink_gpio72_intp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO72INT));
	ralink_gpio72_edge = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO72EDGE));
	*(volatile u32 *)(RALINK_REG_PIO72INT) = cpu_to_le32(0x00FFFFFF);
	*(volatile u32 *)(RALINK_REG_PIO72EDGE) = cpu_to_le32(0x00FFFFFF);
#else
	ralink_gpio9572_intp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO9572INT));
	ralink_gpio9572_edge = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO9572EDGE));
	*(volatile u32 *)(RALINK_REG_PIO9572INT) = cpu_to_le32(0x00FFFFFF);
	*(volatile u32 *)(RALINK_REG_PIO9572EDGE) = cpu_to_le32(0x00FFFFFF);
#endif
#elif defined (RALINK_GPIO_HAS_9532)
	ralink_gpio6332_intp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO6332INT));
	ralink_gpio6332_edge = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO6332EDGE));
	*(volatile u32 *)(RALINK_REG_PIO6332INT) = cpu_to_le32(0xFFFFFFFF);
	*(volatile u32 *)(RALINK_REG_PIO6332EDGE) = cpu_to_le32(0xFFFFFFFF);
	ralink_gpio6332_intp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO6332INT));
	ralink_gpio6332_edge = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO6332EDGE));

	ralink_gpio9564_intp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO9564INT));
	ralink_gpio9564_edge = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO9564EDGE));
	*(volatile u32 *)(RALINK_REG_PIO9564INT) = cpu_to_le32(0xFFFFFFFF);
	*(volatile u32 *)(RALINK_REG_PIO9564EDGE) = cpu_to_le32(0xFFFFFFFF);
	ralink_gpio9564_intp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO9564INT));
	ralink_gpio9564_edge = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO9564EDGE));
#endif
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0)
void ralink_gpio_irq_handler(unsigned int irq, struct irqaction *irqaction)
#else
irqreturn_t ralink_gpio_irq_handler(int irq, void *irqaction)
#endif
{
	struct gpio_time_record {
		unsigned long falling;
		unsigned long rising;
	};
	static struct gpio_time_record record[RALINK_GPIO_NUMBER];
	unsigned long now;
	int i;

	ralink_gpio_save_clear_intp();
	now = jiffies;
#if defined (RALINK_GPIO_HAS_2722)
	for (i = 0; i < 22; i++) {
		if (! (ralink_gpio_intp & (1 << i)))
			continue;
		ralink_gpio_irqnum = i;
		if (ralink_gpio_edge & (1 << i)) { //rising edge
			if (record[i].rising != 0 && time_before_eq(now,
						record[i].rising + 40L)) {
				/*
				 * If the interrupt comes in a short period,
				 * it might be floating. We ignore it.
				 */
			}
			else {
				record[i].rising = now;
				if (time_before(now, record[i].falling + 200L)) {
					//one click
					ralink_gpio_notify_user(1);
				}
				else {
					//press for several seconds
					ralink_gpio_notify_user(2);
				}
			}
		}
		else { //falling edge
			record[i].falling = now;
		}
		break;
	}
	for (i = 22; i < 28; i++) {
		if (! (ralink_gpio2722_intp & (1 << (i - 22))))
			continue;
		ralink_gpio_irqnum = i;
		if (ralink_gpio2722_edge & (1 << (i - 22))) {
			if (record[i].rising != 0 && time_before_eq(now,
						record[i].rising + 40L)) {
			}
			else {
				record[i].rising = now;
				if (time_before(now, record[i].falling + 200L)) {
					ralink_gpio_notify_user(1);
				}
				else {
					ralink_gpio_notify_user(2);
				}
			}
		}
		else {
			record[i].falling = now;
		}
		break;
	}
#elif defined (RALINK_GPIO_HAS_9532)
	for (i = 0; i < 32; i++) {
		if (! (ralink_gpio_intp & (1 << i)))
			continue;
		ralink_gpio_irqnum = i;
		if (ralink_gpio_edge & (1 << i)) { //rising edge
			if (record[i].rising != 0 && time_before_eq(now,
						record[i].rising + 40L)) {
				/*
				 * If the interrupt comes in a short period,
				 * it might be floating. We ignore it.
				 */
			}
			else {
				record[i].rising = now;
				if (time_before(now, record[i].falling + 200L)) {
					//one click
					ralink_gpio_notify_user(1);
				}
				else {
					//press for several seconds
					ralink_gpio_notify_user(2);
				}
			}
		}
		else { //falling edge
			record[i].falling = now;
		}
		break;
	}
	for (i = 32; i < 64; i++) {
		if (! (ralink_gpio6332_intp & (1 << (i - 32))))
			continue;
		ralink_gpio_irqnum = i;
		if (ralink_gpio6332_edge & (1 << (i - 32))) {
			if (record[i].rising != 0 && time_before_eq(now,
						record[i].rising + 40L)) {
			}
			else {
				record[i].rising = now;
				if (time_before(now, record[i].falling + 200L)) {
					ralink_gpio_notify_user(1);
				}
				else {
					ralink_gpio_notify_user(2);
				}
			}
		}
		else {
			record[i].falling = now;
		}
		break;
	}
	for (i = 64; i < RALINK_GPIO_NUMBER; i++) {
		if (! (ralink_gpio9564_intp & (1 << (i - 64))))
			continue;
		ralink_gpio_irqnum = i;
		if (ralink_gpio9564_edge & (1 << (i - 64))) {
			if (record[i].rising != 0 && time_before_eq(now,
						record[i].rising + 40L)) {
			}
			else {
				record[i].rising = now;
				if (time_before(now, record[i].falling + 200L)) {
					ralink_gpio_notify_user(1);
				}
				else {
					ralink_gpio_notify_user(2);
				}
			}
		}
		else {
			record[i].falling = now;
		}
		break;
	}
#else
	for (i = 0; i < 24; i++) {
		if (! (ralink_gpio_intp & (1 << i)))
			continue;
		ralink_gpio_irqnum = i;
		if (ralink_gpio_edge & (1 << i)) { //rising edge
			if (record[i].rising != 0 && time_before_eq(now,
						record[i].rising + 40L)) {
				/*
				 * If the interrupt comes in a short period,
				 * it might be floating. We ignore it.
				 */
			}
			else {
				record[i].rising = now;
				if (time_before(now, record[i].falling + 200L)) {
					//one click
					printk("i=%d, one click\n", i);
					ralink_gpio_notify_user(1);
				}
				else {
					//press for several seconds
					printk("i=%d, push several seconds\n", i);
					ralink_gpio_notify_user(2);
				}
			}
		}
		else { //falling edge
			record[i].falling = now;
		}
		break;
	}
#if defined (RALINK_GPIO_HAS_4524)
	for (i = 24; i < 40; i++) {
		if (! (ralink_gpio3924_intp & (1 << (i - 24))))
			continue;
		ralink_gpio_irqnum = i;
		if (ralink_gpio3924_edge & (1 << (i - 24))) {
			if (record[i].rising != 0 && time_before_eq(now,
						record[i].rising + 40L)) {
			}
			else {
				record[i].rising = now;
				if (time_before(now, record[i].falling + 200L)) {
					ralink_gpio_notify_user(1);
				}
				else {
					ralink_gpio_notify_user(2);
				}
			}
		}
		else {
			record[i].falling = now;
		}
		break;
	}
	for (i = 40; i < RALINK_GPIO_NUMBER; i++) {
		if (! (ralink_gpio4540_intp & (1 << (i - 40))))
			continue;
		ralink_gpio_irqnum = i;
		if (ralink_gpio4540_edge & (1 << (i - 40))) {
			if (record[i].rising != 0 && time_before_eq(now,
						record[i].rising + 40L)) {
			}
			else {
				record[i].rising = now;
				if (time_before(now, record[i].falling + 200L)) {
					ralink_gpio_notify_user(1);
				}
				else {
					ralink_gpio_notify_user(2);
				}
			}
		}
		else {
			record[i].falling = now;
		}
		break;
	}
#elif defined (RALINK_GPIO_HAS_5124)
	for (i = 24; i < 40; i++) {
		if (! (ralink_gpio3924_intp & (1 << (i - 24))))
			continue;
		ralink_gpio_irqnum = i;
		if (ralink_gpio3924_edge & (1 << (i - 24))) {
			if (record[i].rising != 0 && time_before_eq(now,
						record[i].rising + 40L)) {
			}
			else {
				record[i].rising = now;
				if (time_before(now, record[i].falling + 200L)) {
					ralink_gpio_notify_user(1);
				}
				else {
					ralink_gpio_notify_user(2);
				}
			}
		}
		else {
			record[i].falling = now;
		}
		break;
	}
	for (i = 40; i < RALINK_GPIO_NUMBER; i++) {
		if (! (ralink_gpio5140_intp & (1 << (i - 40))))
			continue;
		ralink_gpio_irqnum = i;
		if (ralink_gpio5140_edge & (1 << (i - 40))) {
			if (record[i].rising != 0 && time_before_eq(now,
						record[i].rising + 40L)) {
			}
			else {
				record[i].rising = now;
				if (time_before(now, record[i].falling + 200L)) {
					ralink_gpio_notify_user(1);
				}
				else {
					ralink_gpio_notify_user(2);
				}
			}
		}
		else {
			record[i].falling = now;
		}
		break;
	}
#elif defined (RALINK_GPIO_HAS_9524) || defined (RALINK_GPIO_HAS_7224)
	for (i = 24; i < 40; i++) {
		if (! (ralink_gpio3924_intp & (1 << (i - 24))))
			continue;
		ralink_gpio_irqnum = i;
		if (ralink_gpio3924_edge & (1 << (i - 24))) {
			if (record[i].rising != 0 && time_before_eq(now,
						record[i].rising + 40L)) {
			}
			else {
				record[i].rising = now;
				if (time_before(now, record[i].falling + 200L)) {
					printk("i=%d, one click\n", i);
					ralink_gpio_notify_user(1);
				}
				else {
					printk("i=%d, push several seconds\n", i);
					ralink_gpio_notify_user(2);
				}
			}
		}
		else {
			record[i].falling = now;
		}
		break;
	}
	for (i = 40; i < 72; i++) {
		if (! (ralink_gpio7140_intp & (1 << (i - 40))))
			continue;
		ralink_gpio_irqnum = i;
		if (ralink_gpio7140_edge & (1 << (i - 40))) {
			if (record[i].rising != 0 && time_before_eq(now,
						record[i].rising + 40L)) {
			}
			else {
				record[i].rising = now;
				if (time_before(now, record[i].falling + 200L)) {
					printk("i=%d, one click\n", i);
					ralink_gpio_notify_user(1);
				}
				else {
					printk("i=%d, push several seconds\n", i);
					ralink_gpio_notify_user(2);
				}
			}
		}
		else {
			record[i].falling = now;
		}
		break;
	}
#if defined (RALINK_GPIO_HAS_7224)
	for (i = 72; i < RALINK_GPIO_NUMBER; i++) {
		if (! (ralink_gpio72_intp & (1 << (i - 72))))
			continue;
		ralink_gpio_irqnum = i;
		if (ralink_gpio72_edge & (1 << (i - 72))) {
			if (record[i].rising != 0 && time_before_eq(now,
						record[i].rising + 40L)) {
			}
			else {
				record[i].rising = now;
				if (time_before(now, record[i].falling + 200L)) {
					printk("i=%d, one click\n", i);
					ralink_gpio_notify_user(1);
				}
				else {
					printk("i=%d, push several seconds\n", i);
					ralink_gpio_notify_user(2);
				}
			}
		}
		else {
			record[i].falling = now;
		}
		break;
	}
#else
	for (i = 72; i < RALINK_GPIO_NUMBER; i++) {
		if (! (ralink_gpio9572_intp & (1 << (i - 72))))
			continue;
		ralink_gpio_irqnum = i;
		if (ralink_gpio9572_edge & (1 << (i - 72))) {
			if (record[i].rising != 0 && time_before_eq(now,
						record[i].rising + 40L)) {
			}
			else {
				record[i].rising = now;
				if (time_before(now, record[i].falling + 200L)) {
					printk("i=%d, one click\n", i);
					ralink_gpio_notify_user(1);
				}
				else {
					printk("i=%d, push several seconds\n", i);
					ralink_gpio_notify_user(2);
				}
			}
		}
		else {
			record[i].falling = now;
		}
		break;
	}
#endif
#endif
#endif

	return IRQ_HANDLED;
}

struct irqaction ralink_gpio_irqaction = {
	.handler = ralink_gpio_irq_handler,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,35)
	.flags = IRQF_DISABLED,
#else
	.flags = SA_INTERRUPT,
#endif
	.name = "ralink_gpio",
};

void __init ralink_gpio_init_irq(void)
{
    printk(KERN_EMERG NAME ": SURFBOARDINT_GPIO %d\n", SURFBOARDINT_GPIO );
	setup_irq(SURFBOARDINT_GPIO, &ralink_gpio_irqaction);
}

module_init(ralink_gpio_init);
module_exit(ralink_gpio_exit);
/* Gemtek GPIO start*/

#include <linux/proc_fs.h>
#include <linux/timer.h>

#if 0
#define DEBUGP(format, args...) printk(KERN_EMERG "%s:%s: " format, __FILE__, __FUNCTION__, ## args)
#else
#define DEBUGP(x, args...)
#endif

#define Reg1_GPIO_Start 0
#define Reg1_GPIO_End   21
#define Reg2_GPIO_Start 22
#define Reg2_GPIO_End   27

#define GPIO_TOTAL 28 

#define GPIO_TEXT_LEN 10

struct GPIO_data_d {
	char GPIO_NAME[GPIO_TEXT_LEN];
	u32 GPIO_PIN_NUM;
}GPIO_data[GPIO_TOTAL];

unsigned char GPIO_LIST_NAME[GPIO_TOTAL][GPIO_TEXT_LEN] =
{
	"GPIO0" ,
	"GPIO1" ,
	"GPIO2" ,
	"GPIO3" ,
	"GPIO4" ,
	"GPIO5" ,
	"GPIO6" ,
	"GPIO7" ,
	"GPIO8" ,
	"GPIO9" ,
	"GPIO10",
	"GPIO11",
	"GPIO12",
	"GPIO13",
	"GPIO14",
	"GPIO15",
	"GPIO16",
	"GPIO17",
	"GPIO18",
	"GPIO19",
	"GPIO20",
	"GPIO21",
	"GPIO22",
	"GPIO23",
	"GPIO24",
	"GPIO25",
	"GPIO26",
	"GPIO27"
};

u32 GPIO_LIST_PIN[GPIO_TOTAL] =
{
	PORT0,
	PORT1,
	PORT2,
	PORT3,
	PORT4,
	PORT5,
	PORT6,
	PORT7,
	PORT8,
	PORT9,
	PORT10,
	PORT11,
	PORT12,
	PORT13,
	PORT14,
	PORT15,
	PORT16,
	PORT17,
	PORT18,
	PORT19,
	PORT20,
	PORT21,
	PORT22,
	PORT23,
	PORT24,
	PORT25,
	PORT26,
	PORT27
};

int GPIO_read_proc(char *buf, char **start, off_t off, unsigned long count, int *eof, void *data);
int GPIO_write_proc(struct file *file, const char __user *buffer,  unsigned long count, void *data);
int GPIO_read_proc_22_27(char *buf, char **start, off_t off, unsigned long count, int *eof, void *data);
int GPIO_write_proc_22_27(struct file *file, const char __user *buffer,  unsigned long count, void *data);

char btn_action[12]="", easy_link_btn_action[12] = "0";

 

struct proc_dir_entry *create_proc_read_write_entry(const char *name,mode_t mode, struct proc_dir_entry *base, read_proc_t *read_proc, write_proc_t *write_proc, void * data)
{
	struct proc_dir_entry *res=create_proc_entry(name,0x1FF,base);

	if (res) {
		res->read_proc=read_proc;
		res->write_proc=write_proc;
		res->data=data;
	}
	return res;
}

static int GPIO_init(void) 
{
	int i;
	
	DEBUGP("Gemtek_GPIO_init\n");	
	

	for(i = Reg1_GPIO_Start;i <= Reg1_GPIO_End; i++)
	{
		strcpy(GPIO_data[i].GPIO_NAME, GPIO_LIST_NAME[i]);
		GPIO_data[i].GPIO_PIN_NUM = GPIO_LIST_PIN[i];
		if(create_proc_read_write_entry(GPIO_data[i].GPIO_NAME,0,NULL,GPIO_read_proc,GPIO_write_proc,(void *)&GPIO_data[i]) == NULL)
		{
			DEBUGP("%s : fail\n", GPIO_data[i].GPIO_NAME);
		}
		else
		{
			DEBUGP("%s : success\n", GPIO_data[i].GPIO_NAME);			
		}
	}
	
	for(i = Reg2_GPIO_Start;i <= Reg2_GPIO_End; i++)
	{
		strcpy(GPIO_data[i].GPIO_NAME, GPIO_LIST_NAME[i]);
		GPIO_data[i].GPIO_PIN_NUM = GPIO_LIST_PIN[i];
		if(create_proc_read_write_entry(GPIO_data[i].GPIO_NAME,0,NULL,GPIO_read_proc_22_27,GPIO_write_proc_22_27,(void *)&GPIO_data[i]) == NULL)
		{
			DEBUGP("%s : fail\n", GPIO_data[i].GPIO_NAME);
		}
		else
		{
			DEBUGP("%s : success\n", GPIO_data[i].GPIO_NAME);			
		}
	}
		
	return 0;		
}


struct timer_list BIG_LED_GPIO;
struct timer_list PLUGIN_LED_GPIO;
struct timer_list MOTION_SENSOR_GPIO;
int plugin_gpio_led = LED_OFF ;
struct timer_list SMALL_LED_GPIO;
struct timer_list NET_GPIO;
int ledcount=0;
int net_gpio_led = LED_OFF ;
int big_gpio_led = LED_OFF ;
int small_gpio_led = LED_OFF ;
#if defined (RALINK_GPIO_HAS_2722)
void BIG_LED_GREEN_BLINK(unsigned long data){

	if(big_gpio_led == LED_OFF){
		GPIO_write_bit_22_27(BIG_GREEN_LED,LED_ON);	
		big_gpio_led = LED_ON;
	}
	else{
		GPIO_write_bit_22_27(BIG_GREEN_LED,LED_OFF);	
		big_gpio_led = LED_OFF;
	}

		BIG_LED_GPIO.expires = jiffies + (1 * (HZ/4));
		add_timer(&BIG_LED_GPIO);		
		return ;
}
void BIG_LED_AMBER_BLINK(unsigned long data){

	if(big_gpio_led == LED_OFF){
		GPIO_write_bit_22_27(BIG_AMBER_LED,LED_ON);	
		big_gpio_led = LED_ON;
	}
	else{
		GPIO_write_bit_22_27(BIG_AMBER_LED,LED_OFF);	
		big_gpio_led = LED_OFF;
	}

		BIG_LED_GPIO.expires = jiffies + (1 * (HZ/4));
		add_timer(&BIG_LED_GPIO);		
		return ;
}
#endif
void WIFI_LED_BLUE_BLINK(unsigned long data){

	if(plugin_gpio_led == LED_OFF){
		GPIO_write_bit(WIFI_BLUE_LED, BLUE_LED_ON);	
		plugin_gpio_led = LED_ON;
	}
	else{
		GPIO_write_bit(WIFI_BLUE_LED, BLUE_LED_OFF);	
		plugin_gpio_led = LED_OFF;
	}

		PLUGIN_LED_GPIO.expires = jiffies + (7 * (HZ/10));
		add_timer(&PLUGIN_LED_GPIO);		
		return ;
}
int last_status=0;
void Detect_Motion_Scan(unsigned long data){

	u32 ret = 0,prevret;
	GPIO_read_bit(MOTION_SENSOR, &ret);
	//printk("Motion=[%X]\n", ret);
	
	if(motion_sensor_delay <= 0)
	{
		motion_sensor_status = ret; // No delay, give PIN status directly
		printk("motion_sensor_status=[%d], Motion=[%d]\n", motion_sensor_status, ret);
	}
	else
	{// Delay Enabled
//		printk("motion_sensor_status=[%d], Motion=[%d]\n", motion_sensor_status, ret);
		motion_delay_timer_count++;
		if(ret == 1)
			motion_count++;
			
		if(motion_delay_timer_count >= 2*motion_sensor_delay) // 500ms per detect, so *2
		{
			motion_delay_timer_count = 0; // Reset timer
//			printk(KERN_EMERG "motion_count=[%d], motion_sensor_delay=[%d], percent=[%d/%d], motion_sensor_sensitivity=[%d]\n", motion_count, motion_sensor_delay, motion_count, 2*motion_sensor_delay, motion_sensor_sensitivity);
			if( (motion_count*100) >= (motion_sensor_sensitivity*2*motion_sensor_delay) )
			{
				motion_sensor_status = 1;
				if(0==last_status)
					printk(KERN_EMERG "----------------- MOTION -----------------\n");
				last_status = 1;
			}
			else
			{
				motion_sensor_status = 0;
				if(1==last_status)
					printk(KERN_EMERG "----------------- NO MOTION -----------------\n");
				last_status = 0;
			}
			
			motion_count = 0; // Reset Motion Count
		}
	}

	
	/* Do Scan every 500 ms */
	MOTION_SENSOR_GPIO.expires = jiffies + (1 * (HZ/2));
	MOTION_SENSOR_GPIO.function = Detect_Motion_Scan;	
	add_timer(&MOTION_SENSOR_GPIO);		
	
	return ;
}

void WIFI_LED_BLUE_OFF(unsigned long data){

//	if(plugin_gpio_led == LED_OFF){
//		GPIO_write_bit(WIFI_BLUE_LED, BLUE_LED_ON);	
//		plugin_gpio_led = LED_ON;
//	}
//	else{
		GPIO_write_bit(WIFI_BLUE_LED, BLUE_LED_OFF);	
//		plugin_gpio_led = LED_OFF;
//	}

//		PLUGIN_LED_GPIO.expires = jiffies + (7 * (HZ/10));
//		add_timer(&PLUGIN_LED_GPIO);		
		return ;
}

void WIFI_LED_AMBER_BLINK(unsigned long data){

	if(plugin_gpio_led == LED_OFF){
		GPIO_write_bit(WIFI_AMBER_LED, AMBER_LED_ON);	
		plugin_gpio_led = LED_ON;
	}
	else{
		GPIO_write_bit(WIFI_AMBER_LED, AMBER_LED_OFF);	
		plugin_gpio_led = LED_OFF;
	}

		PLUGIN_LED_GPIO.expires = jiffies + (7 * (HZ/10));
		add_timer(&PLUGIN_LED_GPIO);		
		return ;
}

void WIFI_LED_AMBER_BLINK_300(unsigned long data){

	if(plugin_gpio_led == LED_OFF){
		GPIO_write_bit(WIFI_AMBER_LED, AMBER_LED_ON);	
		plugin_gpio_led = LED_ON;
	}
	else{
		GPIO_write_bit(WIFI_AMBER_LED, AMBER_LED_OFF);	
		plugin_gpio_led = LED_OFF;
	}

		PLUGIN_LED_GPIO.expires = jiffies + (3 * (HZ/10));
		add_timer(&PLUGIN_LED_GPIO);		
		return ;
}

void WIFI_LED_CHANGE_BLINK(unsigned long data){

	if(plugin_gpio_led == LED_OFF){
		GPIO_write_bit(WIFI_AMBER_LED, AMBER_LED_OFF);
		GPIO_write_bit(WIFI_BLUE_LED, BLUE_LED_ON);		
		plugin_gpio_led = LED_ON;
	}
	else{
		GPIO_write_bit(WIFI_BLUE_LED, BLUE_LED_OFF);
		GPIO_write_bit(WIFI_AMBER_LED, AMBER_LED_ON);		
		plugin_gpio_led = LED_OFF;
	}

		PLUGIN_LED_GPIO.expires = jiffies + (7 * (HZ/10));
		add_timer(&PLUGIN_LED_GPIO);		
		return ;
}
#if defined (RALINK_GPIO_HAS_2722)
void SMALL_LED_GREEN_BLINK(unsigned long data){

	if(small_gpio_led == LED_OFF){
		GPIO_write_bit_22_27(SMALL_GREEN_LED,LED_ON);	
		small_gpio_led = LED_ON;
	}
	else{
		GPIO_write_bit_22_27(SMALL_GREEN_LED,LED_OFF);	
		small_gpio_led = LED_OFF;
	}

		SMALL_LED_GPIO.expires = jiffies + (1 * (HZ/4));
		add_timer(&SMALL_LED_GPIO);		
		return ;
}
void SMALL_LED_AMBER_BLINK(unsigned long data){

	if(small_gpio_led == LED_OFF){
		GPIO_write_bit_22_27(SMALL_AMBER_LED  ,LED_ON);	
		small_gpio_led = LED_ON;
	}
	else{
		GPIO_write_bit_22_27(SMALL_AMBER_LED  ,LED_OFF);	
		small_gpio_led = LED_OFF;
	}

		SMALL_LED_GPIO.expires = jiffies + (1 * (HZ/4));
		add_timer(&SMALL_LED_GPIO);		
		return ;
}
#endif
/*
big_led_set

0	Big LED Green Solid
1	Big LED Green Blink
2	Big LED Amber Solid
3	Big LED Amber Blink
4	No LED
*/

void big_led_set(int ledcount)	{
return;
#if defined (RALINK_GPIO_HAS_2722)
	del_timer_sync(&BIG_LED_GPIO);
	
	if(ledcount==0)
	{
		GPIO_write_bit_22_27(BIG_GREEN_LED,LED_ON);
		GPIO_write_bit_22_27(BIG_AMBER_LED,LED_OFF);				
	}	
	if(ledcount==1)
	{
		GPIO_write_bit_22_27(BIG_GREEN_LED,LED_OFF);
		GPIO_write_bit_22_27(BIG_AMBER_LED,LED_OFF);	
		
		big_gpio_led = LED_OFF ;

		BIG_LED_GPIO.expires = jiffies + (1 * (HZ/2));
		BIG_LED_GPIO.function = BIG_LED_GREEN_BLINK;		
		add_timer(&BIG_LED_GPIO);

	}
	else if(ledcount==2) 
	{
		GPIO_write_bit_22_27(BIG_GREEN_LED,LED_OFF);
		GPIO_write_bit_22_27(BIG_AMBER_LED,LED_ON);	
	}
	else if(ledcount==3) 
	{
		GPIO_write_bit_22_27(BIG_GREEN_LED,LED_OFF);
		GPIO_write_bit_22_27(BIG_AMBER_LED,LED_OFF);	
				
		big_gpio_led = LED_OFF ;

		BIG_LED_GPIO.expires = jiffies + (1 * (HZ/2));
		BIG_LED_GPIO.function = BIG_LED_AMBER_BLINK;		
		add_timer(&BIG_LED_GPIO);
	}
	else if(ledcount==4) 
	{
		GPIO_write_bit_22_27(BIG_GREEN_LED,LED_OFF);
		GPIO_write_bit_22_27(BIG_AMBER_LED,LED_OFF);	
	}
	else if(ledcount==5){ //For Easytest
		GPIO_write_bit_22_27(BIG_GREEN_LED,LED_ON);
		GPIO_write_bit_22_27(BIG_AMBER_LED,LED_ON);
	}
#endif
}

/*
plugin_led_set

0	LED Blue Blink, 700 ms ON, 700 ms OFF
1	LED Blue 30s ON, then OFF
2	LED Amber Solid ON
3	LED Amber Blink, 700 ms ON, 700 ms OFF
4	No LED
5	700 ms LED Blue, 700 ms LED Amber
*/

void plugin_led_set(int ledcount)
{

	
	del_timer_sync(&PLUGIN_LED_GPIO);
	
	if(ledcount==0)
	{
		GPIO_write_bit(WIFI_AMBER_LED, AMBER_LED_OFF);
		GPIO_write_bit(WIFI_BLUE_LED, BLUE_LED_OFF);	
		
		plugin_gpio_led = LED_OFF ;

		PLUGIN_LED_GPIO.expires = jiffies + (1 * (HZ/2));
		PLUGIN_LED_GPIO.function = WIFI_LED_BLUE_BLINK;		
		add_timer(&PLUGIN_LED_GPIO);

	}
	else if(ledcount==1) 
	{
		GPIO_write_bit(WIFI_AMBER_LED, AMBER_LED_OFF);
		GPIO_write_bit(WIFI_BLUE_LED, BLUE_LED_OFF);	
		
		plugin_gpio_led = LED_OFF ;
		
		GPIO_write_bit(WIFI_BLUE_LED, BLUE_LED_ON);	
		PLUGIN_LED_GPIO.expires = jiffies + (30 * HZ);
		PLUGIN_LED_GPIO.function = WIFI_LED_BLUE_OFF;		
		add_timer(&PLUGIN_LED_GPIO);	
	}
	else if(ledcount==2) 
	{
		GPIO_write_bit(WIFI_BLUE_LED, BLUE_LED_OFF);
		GPIO_write_bit(WIFI_AMBER_LED, AMBER_LED_ON);
	}
	else if(ledcount==3) 
	{
		GPIO_write_bit(WIFI_AMBER_LED, AMBER_LED_OFF);
		GPIO_write_bit(WIFI_BLUE_LED, BLUE_LED_OFF);	
		
		plugin_gpio_led = LED_OFF ;

		PLUGIN_LED_GPIO.expires = jiffies + (1 * (HZ/2));
		PLUGIN_LED_GPIO.function = WIFI_LED_AMBER_BLINK;		
		add_timer(&PLUGIN_LED_GPIO);
	}
	else if(ledcount==4)
	{
		GPIO_write_bit(WIFI_BLUE_LED, BLUE_LED_OFF);
		GPIO_write_bit(WIFI_AMBER_LED, AMBER_LED_OFF);
	}
	else if(ledcount==5) 
	{
		GPIO_write_bit(WIFI_AMBER_LED, AMBER_LED_OFF);
		GPIO_write_bit(WIFI_BLUE_LED, BLUE_LED_OFF);	
		
		plugin_gpio_led = LED_OFF ;

		PLUGIN_LED_GPIO.expires = jiffies + (1 * (HZ/2));
		PLUGIN_LED_GPIO.function = WIFI_LED_CHANGE_BLINK;		
		add_timer(&PLUGIN_LED_GPIO);
	}
	else if(ledcount==6) 
	{
		GPIO_write_bit(WIFI_AMBER_LED, AMBER_LED_OFF);
		GPIO_write_bit(WIFI_BLUE_LED, BLUE_LED_OFF);	
		
		plugin_gpio_led = LED_OFF ;

		PLUGIN_LED_GPIO.expires = jiffies + (1 * (HZ/2));
		PLUGIN_LED_GPIO.function = WIFI_LED_AMBER_BLINK_300;		
		add_timer(&PLUGIN_LED_GPIO);
	}
	else if(ledcount==7)
        {
                GPIO_write_bit(WIFI_BLUE_LED, BLUE_LED_ON);
                GPIO_write_bit(WIFI_AMBER_LED, AMBER_LED_ON);
        }
	
}

/*
small_led_set

0	Small LED Blue Solid
1	Small LED Blue Blink
2	Small LED Amber Solid
3	Small LED Amber Blink
4	No LED
*/

void small_led_set(int ledcount)	{
return;	
#if defined (RALINK_GPIO_HAS_2722)
	del_timer_sync(&SMALL_LED_GPIO);
	
	if(ledcount==0)
	{
		GPIO_write_bit_22_27(SMALL_GREEN_LED,LED_ON);
		GPIO_write_bit_22_27(SMALL_AMBER_LED,LED_OFF);				
	}	
	if(ledcount==1)
	{
		GPIO_write_bit_22_27(SMALL_GREEN_LED,LED_OFF);
		GPIO_write_bit_22_27(SMALL_AMBER_LED,LED_OFF);	
		
		small_gpio_led = LED_OFF ;

		SMALL_LED_GPIO.expires = jiffies + (1 * (HZ/2));
		SMALL_LED_GPIO.function = SMALL_LED_GREEN_BLINK;		
		add_timer(&SMALL_LED_GPIO);

	}
	else if(ledcount==2) 
	{
		GPIO_write_bit_22_27(SMALL_GREEN_LED,LED_OFF);
		GPIO_write_bit_22_27(SMALL_AMBER_LED,LED_ON);	
	}
	else if(ledcount==3) 
	{
		GPIO_write_bit_22_27(SMALL_GREEN_LED,LED_OFF);
		GPIO_write_bit_22_27(SMALL_AMBER_LED,LED_OFF);	
				
		small_gpio_led = LED_OFF ;

		SMALL_LED_GPIO.expires = jiffies + (1 * (HZ/2));
		SMALL_LED_GPIO.function = SMALL_LED_AMBER_BLINK;		
		add_timer(&SMALL_LED_GPIO);
	}
	else if(ledcount==4) 
	{
		GPIO_write_bit_22_27(SMALL_GREEN_LED,LED_OFF);
		GPIO_write_bit_22_27(SMALL_AMBER_LED,LED_OFF);	
	}
	else if(ledcount==5){
		GPIO_write_bit_22_27(SMALL_GREEN_LED,LED_ON);
		GPIO_write_bit_22_27(SMALL_AMBER_LED,LED_ON);
	}
#endif
}

int BIG_LED_GPIO_proc(struct file *file, const char __user *buffer,  unsigned long count, void *data)
{
	int len = 0;
	char buf[20]={0};

	len = count;

	if(copy_from_user(buf, buffer, len))
	{
		DEBUGP("fail.\n");		
		return -EFAULT;
	}
	sscanf(buf,"%d",&ledcount);
	DEBUGP("BIG_LED_GPIO_proc ledcount=%d\n",ledcount);
	
	big_led_set(ledcount);

	return len;
} 

int PLUGIN_LED_GPIO_proc(struct file *file, const char __user *buffer,  unsigned long count, void *data)
{
	int len = 0;
	char buf[20]={0};

	len = count;

	if(copy_from_user(buf, buffer, len))
	{
		DEBUGP("fail.\n");		
		return -EFAULT;
	}
	sscanf(buf,"%d",&ledcount);
	//DEBUGP("PLUGIN_LED_GPIO_proc ledcount=%d\n",ledcount);
	printk(KERN_EMERG "PLUGIN_LED_GPIO_proc ledcount=%d\n",ledcount);
	
	plugin_led_set(ledcount);

	return len;
} 

int MOTION_SENSOR_STATUS_read_proc(char *buf, char **start, off_t off, int count, int *eof, void *data)
{
	int len = 0;

	len = sprintf(buf, "%d", motion_sensor_status);
	
	return len;
} 

int MOTION_SENSOR_STATUS_write_proc(struct file *file, const char __user *buffer,  unsigned long count, void *data)
{
	int len = 0;
	int sensor_enable = 0;
	char buf[20]={0};

	len = count;

	if(copy_from_user(buf, buffer, len))
	{
		DEBUGP("fail.\n");		
		return -EFAULT;
	}
	sscanf(buf,"%d",&sensor_enable);	

	del_timer_sync(&MOTION_SENSOR_GPIO);

	motion_sensor_status = 0;
	
	if(sensor_enable==0)
	{
		printk(KERN_EMERG "MOTION_SENSOR_STATUS_write_proc: Stop Motion Detect!!\n");	
	}
	else if(sensor_enable==1)
	{
		printk(KERN_EMERG "MOTION_SENSOR_STATUS_write_proc: Start Motion Detect!!\n");	
		MOTION_SENSOR_GPIO.expires = jiffies + (1 * (HZ/2));
		MOTION_SENSOR_GPIO.function = Detect_Motion_Scan;		
		add_timer(&MOTION_SENSOR_GPIO);
	}
	else	printk(KERN_EMERG "MOTION_SENSOR_STATUS_write_proc: ERROR!!\n");

	return len;
} 

int MOTION_SENSOR_SET_DELAY_proc(struct file *file, const char __user *buffer,  unsigned long count, void *data)
{
	int len = 0;
	char buf[20]={0};

	len = count;

	if(copy_from_user(buf, buffer, len))
	{
		DEBUGP("fail.\n");		
		return -EFAULT;
	}
	sscanf(buf,"%d",&motion_sensor_delay);
	
	if(motion_sensor_delay < 0)
		printk(KERN_EMERG "MOTION_SENSOR_SET_DELAY_proc ERROR!! motion_sensor_delay=[%d]\n", motion_sensor_delay);
	else
		printk(KERN_EMERG "MOTION_SENSOR_SET_DELAY_proc SUCCESS!! motion_sensor_delay=[%d]\n",motion_sensor_delay);
return len;	

	del_timer_sync(&MOTION_SENSOR_GPIO);
	
	if(ledcount==0)
	{
		printk(KERN_EMERG "Do nothing :p\n");	
	}
	else if(ledcount==1)
	{
		MOTION_SENSOR_GPIO.expires = jiffies + (1 * (HZ/2));
		MOTION_SENSOR_GPIO.function = Detect_Motion_Scan;		
		add_timer(&MOTION_SENSOR_GPIO);
	}

	return len;
} 

int MOTION_SENSOR_SET_SENSITIVITY_proc(struct file *file, const char __user *buffer,  unsigned long count, void *data)
{
	int len = 0;
	char buf[20]={0};

	len = count;

	if(copy_from_user(buf, buffer, len))
	{
		DEBUGP("fail.\n");		
		return -EFAULT;
	}
	sscanf(buf,"%d",&motion_sensor_sensitivity);
	
	if(motion_sensor_sensitivity < 0 || motion_sensor_sensitivity > 100)
		printk(KERN_EMERG "MOTION_SENSOR_SET_DELAY_proc ERROR!! motion_sensor_sensitivity=[%d]\n", motion_sensor_sensitivity);
	else
		printk(KERN_EMERG "MOTION_SENSOR_SET_DELAY_proc SUCCESS!! motion_sensor_sensitivity=[%d]\n",motion_sensor_sensitivity);
return len;	
	
	del_timer_sync(&MOTION_SENSOR_GPIO);
	
	if(ledcount==0)
	{
		printk(KERN_EMERG "Do nothing :p\n");	
	}
	else if(ledcount==1)
	{
		MOTION_SENSOR_GPIO.expires = jiffies + (1 * (HZ/2));
		MOTION_SENSOR_GPIO.function = Detect_Motion_Scan;		
		add_timer(&MOTION_SENSOR_GPIO);
	}

	return len;
} 

int SMALL_LED_GPIO_proc(struct file *file, const char __user *buffer,  unsigned long count, void *data)
{
	int len = 0;
	char buf[20]={0};

	len = count;

	if(copy_from_user(buf, buffer, len))
	{
		DEBUGP("fail.\n");		
		return -EFAULT;
	}
	sscanf(buf,"%d",&ledcount);
	DEBUGP("SMALL_LED_GPIO_proc ledcount=%d\n",ledcount);
	
	small_led_set(ledcount);

	return len;
}



int GPIO_read_proc(char *buf, char **start, off_t off, unsigned long count,  int *eof, void *data)
{
	u32 ret = 0;
	struct GPIO_data_d *gpio = (struct GPIO_data_d *)data;
	GPIO_read_bit(gpio->GPIO_PIN_NUM, &ret );

	sprintf(buf, "%d\n", ret);
	return 1;
}

int GPIO_read_proc_22_27(char *buf, char **start, off_t off, unsigned long count,  int *eof, void *data)
{
	u32 ret = 0;
	struct GPIO_data_d *gpio = (struct GPIO_data_d *)data;
	GPIO_write_bit_22_27(gpio->GPIO_PIN_NUM, &ret );
	
	sprintf(buf, "%d\n", ret);
	return 1;
}

int GPIO_write_proc(struct file *file, const char __user *buffer,  unsigned long count, void *data)
{
	int len = 0;
	char buf[10];
	struct GPIO_data_d *gpio = (struct GPIO_data_d *)data;

	if(count >= 10)
	{
		len = 9;
	}
	else
	{
		len = count;
	}
	
	if(copy_from_user(buf, buffer, len))
	{		
		return -EFAULT;
	}	 
	if(buf[0]=='1')
		GPIO_write_bit(gpio->GPIO_PIN_NUM, 1);
	else
		GPIO_write_bit(gpio->GPIO_PIN_NUM, 0);

	return len;	
}

int GPIO_write_proc_22_27(struct file *file, const char __user *buffer,  unsigned long count, void *data)
{
	int len = 0;
	char buf[10];
	struct GPIO_data_d *gpio = (struct GPIO_data_d *)data;
	if(count >= 10)
	{
		len = 9;
	}
	else
	{
		len = count;
	}
	
	if(copy_from_user(buf, buffer, len))
	{		
		return -EFAULT;
	}	 
	if(buf[0]=='1')
		GPIO_write_bit_22_27(gpio->GPIO_PIN_NUM, 1);
	else
		GPIO_write_bit_22_27(gpio->GPIO_PIN_NUM, 0);

	return len;	
}  

int GPIO_setdir(u32 port,int dir_value)
{
		u32 value,tmp;
		
		tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIODIR));

		if(dir_value==RALINK_GPIO_DIR_OUT)
			value=tmp | port;
		else
			value=tmp & (0xffffffff^port);

		
		*(volatile u32 *)(RALINK_REG_PIODIR) = cpu_to_le32(value);
		return 1;
}

int GPIO_setdir_22_27(u32 port,int dir_value)
{
		u32 value,tmp;
		
		tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO5140DIR));

		if(dir_value==RALINK_GPIO_DIR_OUT)
			value=tmp | port;
		else
			value=tmp & (0xffffffff^port);

		
		*(volatile u32 *)(RALINK_REG_PIO5140DIR) = cpu_to_le32(value);
		return 1;
}

int GPIO_write_bit(u32 port,u32 set_value)
{	
		u32 value,tmp;

		GPIO_setdir(port,RALINK_GPIO_DIR_OUT);		
		tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIODATA));

		if(set_value==1)
			value=tmp | port;
		else
			value=tmp & (0xffffffff^port);
		
		*(volatile u32 *)(RALINK_REG_PIODATA) = cpu_to_le32(value);	
		return 1;	
}

int GPIO_write_bit_22_27(u32 port,u32 set_value)
{	
		u32 value,tmp;

		GPIO_setdir_22_27(port,RALINK_GPIO_DIR_OUT);
		tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO5140DATA));
		if(set_value==1)
			value=tmp | port;
		else
			value=tmp & (0xffffffff^port);
		
		*(volatile u32 *)(RALINK_REG_PIO5140DATA) = cpu_to_le32(value);	
		return 1;	
}

int GPIO_read_bit(u32 port,u32 *get_value)
{
		GPIO_setdir(port,RALINK_GPIO_DIR_IN);
		u32 value,tmp;
		//read gpio port
		*get_value=0;
		tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIODATA));
		*get_value=tmp & port;
		if(*get_value>0)
			*get_value=1;
		else
			*get_value=0;
		
		return 1;	
}

int GPIO_read_bit_22_27(u32 port,u32 *get_value)
{
		GPIO_setdir_22_27(port,RALINK_GPIO_DIR_IN);
		u32 value,tmp;
		//read gpio port	
		*get_value=0;
		tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO5140DATA));
		*get_value=tmp & port;
		if(*get_value>0)
			*get_value=1;
		else
			*get_value=0;
		
		return 1;	
}

/* Gemtek GPIO end*/


int set_GPIO_status(u32 pin_num, char value)
{
	if(value == GPIO_STAT_OUT_LOW)
	{
		GPIO_write_bit(pin_num,GPIO_STAT_OUT_LOW);
		//DEBUGP("set GPIO Line 0x%04x to OUT LOW.\n", pin_num);
	}
	else if(value == GPIO_STAT_OUT_HIGH)
	{
		GPIO_write_bit(pin_num,GPIO_STAT_OUT_HIGH);
		//DEBUGP("set GPIO Line 0x%04x to OUT HIGH.\n", pin_num);
	}
	else if(value == GPIO_STAT_IN_LOW)
	{		
		//DEBUGP("[Not implement]set GPIO Line 0x%04x to IN LOW.\n", pin_num);
	}
	else if(value == GPIO_STAT_IN_HIGH)
	{
		//DEBUGP("[Not implement]set GPIO Line 0x%04x to IN HIGH.\n", pin_num);
	}
	else
	{
		//DEBUGP("GPIO %d, Unknown value[0x%04x]...do nothing.\n", pin_num, value);
		return -1;
	}	
	
	return 0;
}


#if SUPPORT_WPS_BUTTON
struct timer_list EasyLink;
int	EasyLinkCount = 0;

/* Gemtek EasyLink button init */
static int easylink_btn_read_proc(char *buf, char **start, off_t off, int count, int *eof, void *data)
{
	int len = 0;

	len = sprintf(buf, easy_link_btn_action);

	sprintf(easy_link_btn_action, "0");
	
	return len;
}

void EASY_LINKScan(void)
{
	unsigned int ret = 0;
	
  GPIO_read_bit(WPS_BUTTON, &ret);
  
	if(!ret) //Button is pressed.
	{
		EasyLinkCount++;
		printk(KERN_EMERG "easylink_btn : easylink_btn is %d...press\n", EasyLinkCount);
		if(EasyLinkCount>=5)
		{ 
		    printk(KERN_EMERG "easylink_btn : easylink_btn is %d > 5...press\n", EasyLinkCount);
		    sprintf(easy_link_btn_action, "1");
		}
	}
	else //Button is released.
	{
		//printk(KERN_EMERG "easylink_btn : easylink_btn is %d..release\n", EasyLinkCount);
		if(EasyLinkCount > 0 && EasyLinkCount < RESET_TIME)
		{
			//printk(KERN_EMERG "easylink_btn : easylink_btn reboot\n");
			sprintf(easy_link_btn_action, "2");
		}
		EasyLinkCount = 0;
	}
	return;	
}

void EasyLinkCheck(unsigned long data)
{
	EASY_LINKScan();
	EasyLink.expires = jiffies + HZ;
	add_timer(&EasyLink);
	return;
}
/* Gemtek EasyLink button init End */
#endif

#if SUPPORT_WPS_LED	
struct timer_list WPSLED;

#define WPS_FAIL '0'
#define WPS_SUCCESS '1'
#define WPS_SUCCESS_NONE '2'
#define WPS_IN_PROGRESS '3'
#define WPS_SESSION_OVERLAP '4'
#define WPS_TIMEOUT '5'
#define WPS_TIMEOUT_NONE '6'
#define WPS_RESET '8'
#define WPS_RESET_NONE '9'
#define WPS_UNKNOWN 'x'
#define WPS_LED_STOP 0
#define WPS_LED_REPEAT 99999
#define WPS_LED_SOLID_BLUE -99999

#define WPS_LED_ARRAY_SIZE 100
int LED_array[WPS_LED_ARRAY_SIZE] = {0};
int LED_index = 0;

void print_LED_array(void)
{
	int i = 0;
	for(i = 0;i< WPS_LED_ARRAY_SIZE;i++)
	{
		DEBUGP("LED_array[%d] : %d\n", i, LED_array[i]);
	}
}

char WPS_LED_STATUS = WPS_UNKNOWN;

void WPS_check_LED(unsigned long data)
{
	int WPS_LED_PERIOD = 0;
	if(LED_index < WPS_LED_ARRAY_SIZE)
	{
		if(LED_array[LED_index] != WPS_LED_STOP)
		{
			if(LED_array[LED_index] == WPS_LED_REPEAT)
			{
				LED_index = 0;
			}

			if( (WPS_LED_STATUS == WPS_FAIL) || (WPS_LED_STATUS == WPS_TIMEOUT) || (WPS_LED_STATUS == WPS_TIMEOUT_NONE) || (WPS_LED_STATUS == WPS_SESSION_OVERLAP))
			{
				if(LED_array[LED_index] == WPS_LED_SOLID_BLUE)
				{
					WPS_LED_PERIOD = 1;	
				}
				else if(LED_array[LED_index] > 0)
				{
          small_led_set(2);																					
					WPS_LED_PERIOD = LED_array[LED_index] * 10;
				}
				else if(LED_array[LED_index] < 0)
				{
          small_led_set(4);																						
					WPS_LED_PERIOD = LED_array[LED_index] *  -10;
				}
			}
			else
			{
				if(LED_array[LED_index] == WPS_LED_SOLID_BLUE)
				{
					WPS_LED_PERIOD = 1;
				}				
				else if(LED_array[LED_index] > 0)
				{
          small_led_set(0);																							
					WPS_LED_PERIOD = LED_array[LED_index] * 10;
				}
				else if(LED_array[LED_index] < 0)
				{
          small_led_set(4);	
					WPS_LED_PERIOD = LED_array[LED_index] * -10;
				}
			}
			DEBUGP(KERN_EMERG "*******WPS_check_LED : Set WPSLED : %d, period : %d[%d]\n", LED_index, WPS_LED_PERIOD, LED_array[LED_index]);
			WPSLED.expires = jiffies + WPS_LED_PERIOD * (HZ/10) ;
			LED_index++;
			WPSLED.function = WPS_check_LED;	
			add_timer(&WPSLED);	
		}
		else
		{
			DEBUGP("WPS_check_LED : Stop\n");
      small_led_set(4);
			del_timer_sync(&WPSLED);	
		}
	}
	return;
}
void WPS_init_LED(void)
{
		DEBUGP("WPS_init_LED : Stop all WPSLED\n");
		
		printk(KERN_EMERG "WPS BUTTON press!!\n");

		del_timer_sync(&WPSLED);
		LED_index = 0;
		print_LED_array();
		WPS_check_LED(0);
}

int wps_write_proc(struct file *file, const char __user *buffer, unsigned long count, void *data)
{
	char buf[10] = {0};
	int len = 0;
	int x = 0;
	DEBUGP("WPS WRITE\n");

	if(count >= 10)
	{
		len = 9;
	}
	else
	{
		len = count;
	}
		
	if(copy_from_user(buf, buffer, len))
	{
		DEBUGP("WPS WRITE : fail.\n");		
		return -EFAULT;
	}

	DEBUGP("WPS WRITE : get data...[%s].\n", buf);

	if((buf[0] == WPS_FAIL) && (WPS_LED_STATUS != WPS_FAIL))
	{
		
    memset(LED_array,0,sizeof(int)*100);
    WPS_LED_STATUS = WPS_FAIL;
		LED_array[0] = 1;
		LED_array[1] = -1;
		LED_array[2] = WPS_LED_REPEAT;
		WPS_init_LED();		
	}
	else if((buf[0] == WPS_SUCCESS) && (WPS_LED_STATUS != WPS_SUCCESS))
	{
		memset(LED_array,0,sizeof(int)*100);
    WPS_LED_STATUS = WPS_SUCCESS;
		//LED_array[0] = 3000;
		LED_array[0] = 30;
		LED_array[1] = WPS_LED_STOP;
		WPS_init_LED();
	}	
	else if((buf[0] == WPS_SUCCESS_NONE) && (WPS_LED_STATUS != WPS_SUCCESS_NONE))
	{
		memset(LED_array,0,sizeof(int)*100);
    WPS_LED_STATUS = WPS_SUCCESS_NONE;
		LED_array[0] = 3000;
		LED_array[1] = -1;
		LED_array[2] = WPS_LED_STOP;
		WPS_init_LED();
	}		
	else if((buf[0] == WPS_IN_PROGRESS) && (WPS_LED_STATUS != WPS_IN_PROGRESS))
	{
	  memset(LED_array,0,sizeof(int)*100);
  	WPS_LED_STATUS = WPS_IN_PROGRESS;
		LED_array[0] = 2;
		LED_array[1] = -1;
		LED_array[2] = WPS_LED_REPEAT;
		WPS_init_LED();
	}	
	else if((buf[0] == WPS_SESSION_OVERLAP) && (WPS_LED_STATUS != WPS_SESSION_OVERLAP))
	{
		memset(LED_array,0,sizeof(int)*100);
    WPS_LED_STATUS = WPS_SESSION_OVERLAP;
		
		for(x=0;x<50;x++)
		{
			if((x%2) == 0)
			{
				LED_array[x] = 1;
			}
			else
			{
				LED_array[x] = -1;
			}
		}
		LED_array[50] = -1; 
    LED_array[51] = WPS_LED_STOP; 
		WPS_init_LED();			
	}
	else if((buf[0] == WPS_TIMEOUT) && (WPS_LED_STATUS != WPS_TIMEOUT))
	{
    memset(LED_array,0,sizeof(int)*100);
    WPS_LED_STATUS = WPS_TIMEOUT;		
	
		for(x=0;x<50;x++)
		{
			if((x%2) == 0)
			{
				LED_array[x] = 1;
			}
			else
			{
				LED_array[x] = -1;
			}
		}
		LED_array[50] = -1; 
    LED_array[51] = WPS_LED_STOP; 
	
		WPS_init_LED();		
	}	
	else if((buf[0] == WPS_TIMEOUT_NONE) && (WPS_LED_STATUS != WPS_TIMEOUT_NONE))
	{
	  memset(LED_array,0,sizeof(int)*100);
    WPS_LED_STATUS = WPS_TIMEOUT_NONE;
    LED_array[0] = 1;
		LED_array[1] = -1;
		LED_array[2] = WPS_LED_REPEAT;


		for(x=0;x<50;x++)
		{
			if((x%2) == 0)
			{
				LED_array[x] = 1;
			}
			else
			{
				LED_array[x] = -1;
			}
		}
		LED_array[50] = -1;
		LED_array[51] = WPS_LED_STOP;

		WPS_init_LED();		
	}		
	else if((buf[0] == WPS_RESET) && (WPS_LED_STATUS != WPS_RESET))
	{
		memset(LED_array,0,sizeof(int)*100);
    WPS_LED_STATUS = WPS_RESET;
		LED_array[0] = -1;
		LED_array[0] = WPS_LED_STOP;
		WPS_init_LED();			
	}	
	else if((buf[0] == WPS_RESET_NONE) && (WPS_LED_STATUS != WPS_RESET_NONE))
	{
		memset(LED_array,0,sizeof(int)*100);
    WPS_LED_STATUS = WPS_RESET_NONE;
		LED_array[0] = -1;
		LED_array[1] = WPS_LED_STOP;
		WPS_init_LED();			
	}		
	return len;
}  
			   
          
int wps_read_proc(char *buf, char **start, off_t off, int count, int *eof, void *data)
{
	int ret = 0;
	int len = 0;
		
	DEBUGP("WPS READ\n");

	len = sprintf(buf, "%d\n", ret);	
	return len;
}
#endif

#if SUPPORT_RESET_BUTTON

/* Gemtek Reset button init */
struct timer_list Reset;
int	ResetCount = 0;

static int reset_btn_read_proc(char *buf, char **start, off_t off, int count, int *eof, void *data)
{
	int len = 0;

	len = sprintf(buf, btn_action);

	return len;
}

void ResetScan(void)
{
	unsigned int ret = 0;

  GPIO_read_bit(RESET_BUTTON,&ret);

	if(!ret) //Button is pressed.
	{
		big_led_set(2);
		ResetCount++;
		printk( "reset_btn : reset_btn is %d...press\n", ResetCount);
		
		if(ResetCount>=RESET_TIME)
		{ 
		    printk(KERN_EMERG "reset_btn : reset_btn is %d > %d...press\n", ResetCount,RESET_TIME);
		    sprintf(btn_action, "RESET NOW!");
		}
	}
	else //Button is released.
	{
		if(ResetCount > 0 && ResetCount < RESET_TIME)
		{
 			sprintf(btn_action, "REBOOT NOW!");
		}
		ResetCount = 0;
	}

	return;
}

void ResetCheck(unsigned long data)
{
	ResetScan();
	Reset.expires = jiffies + HZ;
	add_timer(&Reset);
	return;
}
/* Gemtek Reset button init End*/
#endif

#if SUPPORT_GEMTEK_MP_TEST	
struct timer_list GTK_MP;
struct timer_list GTK_MP_PLUG;

void GEMTEK_MP_Scan(void)
{
	unsigned int ret = 0, ret1 = 0;

	ret = 0;
  	GPIO_read_bit(POWER_BUTTON, &ret);

	ret1 = 0;
	GPIO_read_bit(RESET_BUTTON,&ret1);
	
	if((ret == 1) && (ret1 == 0)) //RESET Button is pressed.
	{
		printk(KERN_EMERG "RESET BUTTON press!! - Dark all LED\n");
		plugin_led_set(7);
		plugin_led_set(4);
		GPIO_write_bit(POWER_BLUE_LED, 0);
		GPIO_write_bit(POWER_BLUE_LED, 1);
		
	}
	else if((ret == 0) && (ret1 == 1)) //POWER Button is pressed.
	{	
		printk(KERN_EMERG "POWER BUTTON press!! - Lite up all LED\n");
		plugin_led_set(4);
		plugin_led_set(7);
		GPIO_write_bit(POWER_BLUE_LED, 1);
		GPIO_write_bit(POWER_BLUE_LED, 0);
	}
	
/*	
  	ret = 0;
  	GPIO_read_bit(WPS_BUTTON, &ret);

	ret1 = 0;
	GPIO_read_bit(RESET_BUTTON,&ret1);
	  
	if((ret == 1) && (ret1 == 0)) //RESET Button is pressed.
	{
		printk(KERN_EMERG "RESET BUTTON press!! - Dark all LED\n");
		big_led_set(4);
		printk(KERN_EMERG "RESET BUTTON press!! - Dark all LED\n");
		small_led_set(4);
	}
	else if((ret == 0) && (ret1 == 1)) //WPS Button is pressed.
	{	

		printk(KERN_EMERG "WPS BUTTON press!! - Lite up all LED\n");
		big_led_set(5);
		printk(KERN_EMERG "WPS BUTTON press!! - Lite up all LED\n");
		small_led_set(5);
	}
	else //Other
	{
	}
*/	
	return;	
}

void GEMTEK_MP_PLUG_Scan(void)
{
	unsigned int ret = 0;

	ret = 0;
  	GPIO_read_bit(MOTION_SENSOR, &ret);
	
	if(ret == 1) //MOTION SENSOR detect
	{
		printk(KERN_EMERG "MOTION SENSOR detect!! - Lite up all LED\n");
		plugin_led_set(4);
		plugin_led_set(7);
		GPIO_write_bit(POWER_BLUE_LED, 1);
		GPIO_write_bit(POWER_BLUE_LED, 0);
	}
	else
	{
		printk(KERN_EMERG "RMOTION SENSOR no detect!! - Dark all LED\n");
		plugin_led_set(7);
		plugin_led_set(4);
		GPIO_write_bit(POWER_BLUE_LED, 0);
		GPIO_write_bit(POWER_BLUE_LED, 1);
	}

	return;	
}

void GEMTEK_MP_Check(unsigned long data)
{
	del_timer_sync(&GTK_MP);/* Fix Kernel Panic */

	GEMTEK_MP_Scan();
	GTK_MP.expires = jiffies + HZ;
	add_timer(&GTK_MP);
	return;
}

void GEMTEK_MP_PLUG_Check(unsigned long data)
{
	del_timer_sync(&GTK_MP_PLUG);/* Fix Kernel Panic */

	GEMTEK_MP_PLUG_Scan();
	GTK_MP_PLUG.expires = jiffies + HZ;
	add_timer(&GTK_MP_PLUG);
	return;
}

int gemtek_test_write_proc(struct file *file, const char __user *buffer, unsigned long count, void *data)
{
	return 0;
}

int gemtek_test_plug_write_proc(struct file *file, const char __user *buffer, unsigned long count, void *data)
{
	return 0;
}

int gemtek_test_read_proc(char *buf, char **start, off_t off, int count, int *eof, void *data)
{
	int ret = 0;
	int len = 0;
	int i = 0;
	
	DEBUGP("GEMTEK TEST\n");
	
	remove_proc_entry("resetDefault", NULL);
  	remove_proc_entry("easylink", NULL);
 	remove_proc_entry("WPS_LED", NULL);

	for(i = 0;i < GPIO_TOTAL; i++)
	{
		remove_proc_entry(GPIO_LIST_NAME[i], NULL);
	}

		remove_proc_entry("SPEED", NULL);
	
#if SUPPORT_RESET_BUTTON	
	del_timer_sync(&Reset);
#endif
#if SUPPORT_WPS_BUTTON	
	del_timer_sync(&EasyLink);
#endif
#if SUPPORT_WPS_LED	
	del_timer_sync(&WPSLED);
#endif

	del_timer_sync(&GTK_MP);/* Fix Kernel Panic */

	GTK_MP.expires = jiffies + HZ;
	GTK_MP.function = GEMTEK_MP_Check;	
	add_timer(&GTK_MP);
	
	len = sprintf(buf, "GEMTEK TEST\n");	
	return len;
}

int gemtek_test_plug_read_proc(char *buf, char **start, off_t off, int count, int *eof, void *data)
{
	int ret = 0;
	int len = 0;
	int i = 0;
	
	DEBUGP("GEMTEK TEST MOTION SENSOR\n");
	
	remove_proc_entry("resetDefault", NULL);
  	remove_proc_entry("easylink", NULL);
 	remove_proc_entry("WPS_LED", NULL);

	for(i = 0;i < GPIO_TOTAL; i++)
	{
		remove_proc_entry(GPIO_LIST_NAME[i], NULL);
	}

		remove_proc_entry("SPEED", NULL);
	
#if SUPPORT_RESET_BUTTON	
	del_timer_sync(&Reset);
#endif
#if SUPPORT_WPS_BUTTON	
	del_timer_sync(&EasyLink);
#endif
#if SUPPORT_WPS_LED	
	del_timer_sync(&WPSLED);
#endif

	del_timer_sync(&GTK_MP_PLUG);/* Fix Kernel Panic */

	GTK_MP_PLUG.expires = jiffies + HZ;
	GTK_MP_PLUG.function = GEMTEK_MP_PLUG_Check;	
	add_timer(&GTK_MP_PLUG);
	
	len = sprintf(buf, "GEMTEK TEST\n");	
	return len;
}

#endif


/* Gemtek LED start*/
static int __init LED_init(void)
{	
	int i=0;

	printk(KERN_EMERG "Gemtek LED init...\n");

#if SUPPORT_RESET_BUTTON	
	/* Gemtek Reset button init */
	create_proc_read_entry("resetDefault",0,NULL,reset_btn_read_proc,NULL);
	init_timer(&Reset);
//	Reset.expires = jiffies + HZ;
//	Reset.function = ResetCheck;
//	add_timer(&Reset);
	/* Gemtek Reset button init End */
#endif	

#if SUPPORT_WPS_BUTTON	
		/* Gemtek EasyLink button init */
		create_proc_read_entry("easylink",0,NULL,easylink_btn_read_proc,NULL);	
//		init_timer(&EasyLink);	
//		EasyLink.expires = jiffies + HZ;
//		EasyLink.function = EasyLinkCheck;	
//		add_timer(&EasyLink);
		/* Gemtek EasyLink button init End */
#endif
	
#if SUPPORT_WPS_LED		
		if(create_proc_read_write_entry("WPS_LED",0,NULL,wps_read_proc,wps_write_proc,NULL) == NULL)
		{
			DEBUGP("WPS LED : fail\n");
		}
		else
		{
			DEBUGP("WPS LED : success\n");			
		}
			init_timer(&WPSLED);	
			WPSLED.function = WPS_check_LED;	
#endif
	
#if SUPPORT_GEMTEK_MP_TEST	
		init_timer(&GTK_MP);
		if(create_proc_read_write_entry("GEMTEK_BTN_TEST",0,NULL,gemtek_test_read_proc,gemtek_test_write_proc,NULL) == NULL)
		{
			DEBUGP("GEMTEK TEST : fail\n");
		}
		else
		{
			DEBUGP("GEMTEK TEST : success\n");			
		}
#endif			
	
		/* Gemtek GPIO */
		GPIO_init();
		
		/* Gemtek BELKIN BIG LED GPIO init */
		if(create_proc_read_write_entry("BIG_LED_GPIO",0,NULL,NULL,BIG_LED_GPIO_proc,NULL))
			DEBUGP("create_proc_read_write_entry  BIG_LED_GPIO: fail\n");
		else
			DEBUGP("create_proc_read_write_entry   BIG_LED_GPIO: success\n");
			
		  init_timer(&BIG_LED_GPIO);
		/* Gemtek BELKIN BIG LED GPIO End */
		
		/* Gemtek BELKIN SMALL LED GPIO init */
		if(create_proc_read_write_entry("SMALL_LED_GPIO",0,NULL,NULL,SMALL_LED_GPIO_proc,NULL))
			DEBUGP("create_proc_read_write_entry  SMALL_LED_GPIO: fail\n");
		else
			DEBUGP("create_proc_read_write_entry   SMALL_LED_GPIO: success\n");
			
		init_timer(&SMALL_LED_GPIO);
		/* Gemtek BELKIN SMALL LED GPIO End */
    	
		//GPIO_write_bit(WATCH_DOG_ENABLE, WatchD_OFF);
		//GPIO_write_bit(WATCH_DOG_INPUT, WatchD_OFF);

		/* Gemtek GPIO End */
		
		GPIO_write_bit_22_27(BIG_GREEN_LED,LED_OFF);
		GPIO_write_bit_22_27(BIG_AMBER_LED,LED_ON);

#ifdef PLUGIN_BOARD_NEW		
		/* PLUGIN LED GPIO init */
		if(create_proc_read_write_entry("PLUGIN_LED_GPIO",0,NULL,NULL,PLUGIN_LED_GPIO_proc,NULL))
			DEBUGP("create_proc_read_write_entry  PLUGIN_LED_GPIO: fail\n");
		else
			DEBUGP("create_proc_read_write_entry  PLUGIN_LED_GPIO: success\n");
			
		  init_timer(&PLUGIN_LED_GPIO);
		/* PLUGIN LED GPIO End */
		
		plugin_led_set(0);
		
		unsigned int ret = 0;
  		GPIO_read_bit(RESET_BUTTON,&ret);
  		//GPIO_read_bit(POWER_BUTTON,&ret);// Reset Button not Ready, use Power Button to simulate.
		if(!ret) //Button is pressed.
		{
			plugin_led_set(6);
			printk(KERN_EMERG "################## Restore to Factory Defaults ###################\n");
			restoreValue = 1;
			printk("################## restoreValue= %d ##################\n", restoreValue);
			sprintf(btn_action, "RESET NOW!");
			//printk(KERN_EMERG "################## Restore to Factory Defaults, btn_action=[%s] ###################\n", btn_action);
		}
		else {
			printk(KERN_EMERG "################## Don't Restore to Factory Defaults ###################\n");
			restoreValue = 0;
			printk("################## restoreValue= %d ##################\n", restoreValue);
		}
		
#if SUPPORT_MOTION_SENSOR	
		//MOTION_SENSOR_GPIO
		if(create_proc_read_write_entry("MOTION_SENSOR_STATUS",0,NULL,MOTION_SENSOR_STATUS_read_proc,MOTION_SENSOR_STATUS_write_proc,NULL))
			DEBUGP("create_proc_read_write_entry  MOTION_SENSOR_STATUS: fail\n");
		else
			DEBUGP("create_proc_read_write_entry  MOTION_SENSOR_STATUS: success\n");
			
		if(create_proc_read_write_entry("MOTION_SENSOR_SET_DELAY",0,NULL,NULL,MOTION_SENSOR_SET_DELAY_proc,NULL))
			DEBUGP("create_proc_read_write_entry  MOTION_SENSOR_SET_DELAY: fail\n");
		else
			DEBUGP("create_proc_read_write_entry  MOTION_SENSOR_SET_DELAY: success\n");
		
		if(create_proc_read_write_entry("MOTION_SENSOR_SET_SENSITIVITY",0,NULL,NULL,MOTION_SENSOR_SET_SENSITIVITY_proc,NULL))
			DEBUGP("create_proc_read_write_entry  MOTION_SENSOR_SET_SENSITIVITY: fail\n");
		else
			DEBUGP("create_proc_read_write_entry  MOTION_SENSOR_SET_SENSITIVITY: success\n");
			
		init_timer(&MOTION_SENSOR_GPIO);
#endif
		
#if SUPPORT_GEMTEK_MP_TEST	
		init_timer(&GTK_MP_PLUG);
		if(create_proc_read_write_entry("GEMTEK_BTN_TEST_PLUG",0,NULL,gemtek_test_plug_read_proc,gemtek_test_plug_write_proc,NULL) == NULL)
		{
			DEBUGP("GEMTEK TEST PLUG: fail\n");
		}
		else
		{
			DEBUGP("GEMTEK TEST PLUG: success\n");			
		}
#endif

#endif

	return 1;
}
