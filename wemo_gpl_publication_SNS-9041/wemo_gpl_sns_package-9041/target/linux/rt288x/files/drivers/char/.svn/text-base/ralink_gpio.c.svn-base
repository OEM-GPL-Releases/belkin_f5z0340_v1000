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
#include "ralink_gpio.h"
#include "../../arch/mips/rt2880/serial_rt2880.h"

#include <asm/rt2880/surfboardint.h>

#ifdef  CONFIG_DEVFS_FS
#include <linux/devfs_fs_kernel.h>
static  devfs_handle_t devfs_handle;
#endif

#include "product_gpio_cfg.h"

#ifndef FALSE
#define FALSE 0
#endif

#ifndef TRUE
#define TRUE   (!FALSE)
#endif

#define LED_ON    1
#define LED_OFF   0

#define Reg1_GPIO_Start 0
#define Reg1_GPIO_End   21
#define Reg2_GPIO_Start 22
#define Reg2_GPIO_End   27

#if GPIO_DEBUG_ENABLE
#define PRINTK(format, ...) printk(format, ## __VA_ARGS__)
#else
#define PRINTK(format, ...)
#endif


#if defined(MIN_OUT_DELAY) && defined(RELAY_ON_TIME)
#error "Error: MIN_OUT_DELAY and RELAY_ON_TIME are mutually exclusive features, select just one"
#endif

#if defined(POWER_DBL_CLICK) || defined(MIN_OUT_DELAY) || \
    defined(RELAY_ON_TIME) || defined(PWM_SERIAL_PORT) || \
    defined(HAS_WASP_RESET)
   #define WEMO_TIMER_PROC    1
#endif

#if defined(MIN_OUT_DELAY) || defined(RELAY_ON_TIME) || \
    defined(PWM_SERIAL_PORT)
   #define UINT_PROC_ENTRIES  1
#endif

#ifdef POWER_DBL_CLICK
typedef enum {
   IDLE = 0,   // Nothing happening yet

   CLICKED,    // Button released for > MaxClickDelay in CLICKED_W state

   DBL_CLICKED,// Button pressed for > MinClickJiffies & < MaxClickJiffies
               // in PRESSED_2 state
   HELD,       // Button has been pressed for > MaxClickTime

   PRESSED_1,  // Button just pressed the first time

   CLICKED_W,  // Button pressed for > MinClickJiffies & < MaxClickJiffies
               // waiting for MaxClickDelay before declaring single click

   PRESSED_2,  // Button pressed the second time

} BUTTON_STATE;

BUTTON_STATE PowerButtonState = IDLE;
// Reported to user:
//    0 - button inactive
//    1 - button clicked
//    2 - button double clicked
//    3 - button held a "long" time
int ReportedButtonState = IDLE;

// The default double click time in windows is 1/2 second, i.e.
// All times are in jiffies
static unsigned int MinClickTime = 2;  // debounce, clicks less than this are ignored
static unsigned int MaxClickTime = HZ; // button held longer than this is being HELD
static unsigned int DoubleClickTime = HZ / 2;
static unsigned int ClickClearDelay = HZ; // length of time to report key status


static int PowerButtonLast = 1;
static u64 ButtonReportedTime = 0;  // time ReportedButtonState last changed
static u64 LastChangeTime = 0;      // time button or PowerButtonState last changed
static u64 ClickStarted = 0;

static void LogBadState(void);
static void LogInvalidState(void);
static int PowerButtonStateReadProc(char *buf,char **start,off_t off,int count,int *eof,void *data);
#endif   // POWER_DBL_CLICK

#ifdef MIN_OUT_DELAY
static unsigned int MinOutDelay = MIN_OUT_DELAY;
static u64 LastRelayChange = 0;  // time relay last changed state
static int RelayState = 0;
static int DesiredRelayState = 0;
#endif   // MIN_OUT_DELAY

#if HAS_WASP_RESET
static int WaspResetCount = 0;
#endif

#ifdef RELAY_ON_TIME
static unsigned int RelayOnTime = RELAY_ON_TIME;
static u64 LastRelayChange = 0;  // time relay last changed state
static int RelayState = 0;
#endif   // RELAY_ON_TIME

#ifdef PWM_SERIAL_PORT
// #define DEBUG_PWM          1

static u64 LastFadeChange = 0;   // fade was stepped
static unsigned int FadeTime = 15;
static int BacklightTarget = -1;
static int BacklightCurrent = -1;
static int BacklightLast = -1;
static void *pUartPort = NULL;

// Mode:
//    0 - No fading, update LED level via BacklightCurrent
//    1 - fade BacklightCurrent to BacklightTarget
//    2 - Fade up and down continuedly (test mode)
static int BacklightMode = 0;

#define WEMO_PWM_STOP_TX      0
#define WEMO_PWM_START_TX     1
#define WEMO_PWM_SEND_BREAK   2
#define WEMO_PWM_END_BREAK    3

// Actions:
//    WEMO_PWM_STOP_TX    - Stop Tx (LED on full)
//    WEMO_PWM_START_TX   - Start Tx (LED on, but not at full)
//    WEMO_PWM_SEND_BREAK - Stop ending, send Break (LED off)
//    WEMO_PWM_END_BREAK  - End Break, start sending (LED turning back on)
void WemoPwmLED(void *p,int Action,int Level);  // in .../drivers/serial/8250.c

void UpdatePwmState(void);
int GetPwmByte(unsigned int iobase);
int WemoPwmLEDInit(void *p,int iobase);
static int BacklightWriteProc(struct file *,const char __user *,unsigned long,
                              void *);

#endif   // PWM_SERIAL_PORT

#ifdef UINT_PROC_ENTRIES
static int UIntReadProc(char *buf,char **start,off_t off,int count,int *eof,void *data);
static int IntReadProc(char *buf,char **start,off_t off,int count,int *eof,void *data);
static int UIntWriteProc(struct file *file, const char __user *buffer, unsigned long count, void *data);
static int IntWriteProc(struct file *file, const char __user *buffer, unsigned long count, void *data);
#endif

#define NAME         "ralink_gpio"
#define RALINK_GPIO_DEVNAME   "gpio"
int ralink_gpio_major = 252;
int ralink_gpio_irqnum = 0;
u32 ralink_gpio_intp = 0;
u32 ralink_gpio_edge = 0;
#ifdef RALINK_GPIO_HAS_5124
u32 ralink_gpio3924_intp = 0;
u32 ralink_gpio3924_edge = 0;
u32 ralink_gpio5140_intp = 0;
u32 ralink_gpio5140_edge = 0;
#endif
#ifdef RALINK_GPIO_HAS_9524
u32 ralink_gpio3924_intp = 0;
u32 ralink_gpio3924_edge = 0;
u32 ralink_gpio7140_intp = 0;
u32 ralink_gpio7140_edge = 0;
u32 ralink_gpio9572_intp = 0;
u32 ralink_gpio9572_edge = 0;
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
#ifdef RALINK_GPIO_HAS_5124
u32 ra_gpio3924_led_set = 0;
u32 ra_gpio3924_led_clr = 0;
u32 ra_gpio5140_led_set = 0;
u32 ra_gpio5140_led_clr = 0;
#endif
#ifdef RALINK_GPIO_HAS_9524
u32 ra_gpio3924_led_set = 0;
u32 ra_gpio3924_led_clr = 0;
u32 ra_gpio7140_led_set = 0;
u32 ra_gpio7140_led_clr = 0;
u32 ra_gpio9572_led_set = 0;
u32 ra_gpio9572_led_clr = 0;
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


#if HAS_MOTION_SENSOR
int motion_sensor_status = 0;
int motion_sensor_delay = 0;
int motion_sensor_sensitivity = 0;
int motion_count = 0;
int motion_delay_timer_count = 0;
#endif


#ifdef WEMO_TIMER_PROC
static void WemoTimerProc(unsigned long unused);
static struct timer_list WemoTimer;
#endif

#include <linux/proc_fs.h>

void GPIO_read_bit(u32 Bit,u32 *get_value);
void GPIO_write_bit(u32 Bit,u32 set_value);
int GPIO_read_proc(char *buf, char **start, off_t off, int count, int *eof, void *data);
int GPIO_write_proc(struct file *file, const char __user *buffer,  unsigned long count, void *data);

static int __init LED_init(void);

struct {
   const char *ProcName;
   read_proc_t *ReadProc;
   write_proc_t *WriteProc;
   void *Data;
} WeMoProcs[] = {
#ifdef POWER_DBL_CLICK
   {"POWER_BUTTON_STATE",PowerButtonStateReadProc,(write_proc_t *) NULL},
#endif
#ifdef MIN_OUT_DELAY
   {"MIN_OUT_RATE",UIntReadProc,UIntWriteProc,&MinOutDelay},
#endif
#ifdef RELAY_ON_TIME
   {"RELAY_ON_TIME",UIntReadProc,UIntWriteProc,&RelayOnTime},
#endif

#ifdef PWM_SERIAL_PORT
   {"FADE_TIME",UIntReadProc,UIntWriteProc,&FadeTime},
   {"BACKLIGHT_MODE",IntReadProc,BacklightWriteProc,&BacklightMode},
   {"BACKLIGHT_TARGET",IntReadProc,BacklightWriteProc,&BacklightTarget},
   {"BACKLIGHT_CURRENT",IntReadProc,BacklightWriteProc,&BacklightCurrent},
#endif
#if HAS_POWER_BUTTON
   {"POWER_BUTTON",GPIO_read_proc,(write_proc_t *) NULL,(void *)POWER_BUTTON},
#endif
#if HAS_POWER_RELAY
   {"POWER_RELAY",GPIO_read_proc,GPIO_write_proc,(void*) POWER_RELAY},
#endif
#if HAS_RELAY_LED
   {"RELAY_LED",GPIO_read_proc,GPIO_write_proc,(void*) RELAY_LED},
#endif
#if HAS_WASP_RESET
   {"WASP_RESET",GPIO_read_proc,GPIO_write_proc,(void*) WASP_RESET},
#endif
   {NULL}
};

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

      //set gpio direction to 'out'
      if (led.gpio <= 23) {
         tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIODIR));
         tmp |= RALINK_GPIO(led.gpio);
         *(volatile u32 *)(RALINK_REG_PIODIR) = tmp;
      }
#ifdef RALINK_GPIO_HAS_5124
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
#endif
#ifdef RALINK_GPIO_HAS_9524
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
         tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO9572DIR));
         tmp |= RALINK_GPIO((led.gpio-72));
         *(volatile u32 *)(RALINK_REG_PIO9572DIR) = tmp;
      }
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

//#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,35)
//long ralink_gpio_ioctl(struct file *file, unsigned int req,
//    unsigned long arg)
//#else
int ralink_gpio_ioctl(struct inode *inode, struct file *file, unsigned int req,
      unsigned long arg)
//#endif
{
   unsigned long idx, tmp;
   ralink_gpio_reg_info info;
#ifdef CONFIG_RALINK_GPIO_LED
   ralink_gpio_led_info led;
#endif

   idx = (req >> RALINK_GPIO_DATA_LEN) & 0xFFL;
   req &= RALINK_GPIO_DATA_MASK;

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
         if (info.irq <= 23) {
            tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIORENA));
            tmp |= (0x1 << info.irq);
            *(volatile u32 *)(RALINK_REG_PIORENA) = cpu_to_le32(tmp);
            tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIOFENA));
            tmp |= (0x1 << info.irq);
            *(volatile u32 *)(RALINK_REG_PIOFENA) = cpu_to_le32(tmp);
         }
#ifdef RALINK_GPIO_HAS_5124
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
#endif
#ifdef RALINK_GPIO_HAS_9524
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
            tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO9572RENA));
            tmp |= (0x1 << (info.irq-72));
            *(volatile u32 *)(RALINK_REG_PIO9572RENA) = cpu_to_le32(tmp);
            tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO9572FENA));
            tmp |= (0x1 << (info.irq-72));
            *(volatile u32 *)(RALINK_REG_PIO9572FENA) = cpu_to_le32(tmp);
         }
#endif
      }
      else
         printk(KERN_ERR NAME ": irq number(%d) out of range\n",
               info.irq);
      break;

#ifdef RALINK_GPIO_HAS_5124
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
#endif
#ifdef RALINK_GPIO_HAS_9524
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

   case RALINK_GPIO_LED_SET:
#ifdef CONFIG_RALINK_GPIO_LED
      copy_from_user(&led, (ralink_gpio_led_info *)arg, sizeof(led));
      ralink_gpio_led_set(led);
#else
      printk(KERN_ERR NAME ": gpio led support not built\n");
#endif
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
   owner:      THIS_MODULE,
//#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,35)
// unlocked_ioctl:   ralink_gpio_ioctl,
//#else
   ioctl:      ralink_gpio_ioctl,
//#endif
   open:    ralink_gpio_open,
   release: ralink_gpio_release,
};

#ifdef CONFIG_RALINK_GPIO_LED

#if RALINK_GPIO_LED_LOW_ACT
#define __LED_ON(gpio)      ra_gpio_led_clr |= RALINK_GPIO(gpio);
#define __LED_OFF(gpio)     ra_gpio_led_set |= RALINK_GPIO(gpio);
#define __LED3924_ON(gpio)  ra_gpio3924_led_clr |= RALINK_GPIO((gpio-24));
#define __LED3924_OFF(gpio) ra_gpio3924_led_set |= RALINK_GPIO((gpio-24));
#define __LED5140_ON(gpio)  ra_gpio5140_led_clr |= RALINK_GPIO((gpio-40));
#define __LED5140_OFF(gpio) ra_gpio5140_led_set |= RALINK_GPIO((gpio-40));
#define __LED7140_ON(gpio)  ra_gpio7140_led_clr |= RALINK_GPIO((gpio-40));
#define __LED7140_OFF(gpio) ra_gpio7140_led_set |= RALINK_GPIO((gpio-40));
#define __LED9572_ON(gpio)  ra_gpio9572_led_clr |= RALINK_GPIO((gpio-72));
#define __LED9572_OFF(gpio) ra_gpio9572_led_set |= RALINK_GPIO((gpio-72));
#else
#define __LED_ON(gpio)      ra_gpio_led_set |= RALINK_GPIO(gpio);
#define __LED_OFF(gpio)     ra_gpio_led_clr |= RALINK_GPIO(gpio);
#define __LED3924_ON(gpio)  ra_gpio3924_led_set |= RALINK_GPIO((gpio-24));
#define __LED3924_OFF(gpio) ra_gpio3924_led_clr |= RALINK_GPIO((gpio-24));
#define __LED5140_ON(gpio)  ra_gpio5140_led_set |= RALINK_GPIO((gpio-40));
#define __LED5140_OFF(gpio) ra_gpio5140_led_clr |= RALINK_GPIO((gpio-40));
#define __LED7140_ON(gpio)  ra_gpio7140_led_set |= RALINK_GPIO((gpio-40));
#define __LED7140_OFF(gpio) ra_gpio7140_led_clr |= RALINK_GPIO((gpio-40));
#define __LED9572_ON(gpio)  ra_gpio9572_led_set |= RALINK_GPIO((gpio-72));
#define __LED9572_OFF(gpio) ra_gpio9572_led_clr |= RALINK_GPIO((gpio-72));
#endif

static void ralink_gpio_led_do_timer(unsigned long unused)
{
   int i;
   unsigned int x;

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

#ifdef RALINK_GPIO_HAS_5124
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

   for (i = 40; i < 52; i++) {
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
#endif // RALINK_GPIO_HAS_5124 //
#ifdef RALINK_GPIO_HAS_9524
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

   for (i = 72; i < 96; i++) {
      ralink_gpio_led_stat[i].ticks++;
      if (ralink_gpio_led_data[i].gpio == -1) //-1 means unused
         continue;
      if (ralink_gpio_led_data[i].on == RALINK_GPIO_LED_INFINITY ||
            ralink_gpio_led_data[i].off == 0) { //always on
         __LED9572_ON(i);
         continue;
      }
      if (ralink_gpio_led_data[i].off == RALINK_GPIO_LED_INFINITY ||
            ralink_gpio_led_data[i].rests == RALINK_GPIO_LED_INFINITY ||
            ralink_gpio_led_data[i].on == 0 ||
            ralink_gpio_led_data[i].blinks == 0 ||
            ralink_gpio_led_data[i].times == 0) { //always off
         __LED9572_OFF(i);
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
         __LED9572_ON(i);
         if (ralink_gpio_led_stat[i].ticks && x == 0)
            ralink_gpio_led_stat[i].offs++;
#if RALINK_LED_DEBUG
         printk("t%d gpio%d on,", ralink_gpio_led_stat[i].ticks, i);
#endif
      }
      else {
         __LED9572_OFF(i);
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
         __LED9572_OFF(i);
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
            __LED9572_OFF(i);
            ralink_gpio_led_data[i].gpio = -1; //stop
         }
#if RALINK_LED_DEBUG
         printk("T%d\n", ralink_gpio_led_stat[i].times);
      } else {
         printk("T@\n");
#endif
      }
   }
#endif // RALINK_GPIO_HAS_9524 //

   //always turn the power LED on
#ifdef CONFIG_RALINK_RT2880
   __LED_ON(12);
#elif defined (CONFIG_RALINK_RT3052) || defined (CONFIG_RALINK_RT2883)
   __LED_ON(9);
#endif

#if RALINK_GPIO_LED_LOW_ACT
   *(volatile u32 *)(RALINK_REG_PIORESET) = ra_gpio_led_clr;
   *(volatile u32 *)(RALINK_REG_PIOSET) = ra_gpio_led_set;
#ifdef RALINK_GPIO_HAS_5124
   *(volatile u32 *)(RALINK_REG_PIO3924RESET) = ra_gpio3924_led_clr;
   *(volatile u32 *)(RALINK_REG_PIO3924SET) = ra_gpio3924_led_set;
   *(volatile u32 *)(RALINK_REG_PIO5140RESET) = ra_gpio5140_led_clr;
   *(volatile u32 *)(RALINK_REG_PIO5140SET) = ra_gpio5140_led_set;
#endif
#ifdef RALINK_GPIO_HAS_9524
   *(volatile u32 *)(RALINK_REG_PIO3924RESET) = ra_gpio3924_led_clr;
   *(volatile u32 *)(RALINK_REG_PIO3924SET) = ra_gpio3924_led_set;
   *(volatile u32 *)(RALINK_REG_PIO7140RESET) = ra_gpio7140_led_clr;
   *(volatile u32 *)(RALINK_REG_PIO7140SET) = ra_gpio7140_led_set;
   *(volatile u32 *)(RALINK_REG_PIO9572RESET) = ra_gpio9572_led_clr;
   *(volatile u32 *)(RALINK_REG_PIO9572SET) = ra_gpio9572_led_set;
#endif
#else // RALINK_GPIO_LED_LOW_ACT //
   *(volatile u32 *)(RALINK_REG_PIOSET) = ra_gpio_led_set;
   *(volatile u32 *)(RALINK_REG_PIORESET) = ra_gpio_led_clr;
#ifdef RALINK_GPIO_HAS_5124
   *(volatile u32 *)(RALINK_REG_PIO3924SET) = ra_gpio3924_led_set;
   *(volatile u32 *)(RALINK_REG_PIO3924RESET) = ra_gpio3924_led_clr;
   *(volatile u32 *)(RALINK_REG_PIO5140SET) = ra_gpio5140_led_set;
   *(volatile u32 *)(RALINK_REG_PIO5140RESET) = ra_gpio5140_led_clr;
#endif
#ifdef RALINK_GPIO_HAS_9524
   *(volatile u32 *)(RALINK_REG_PIO3924SET) = ra_gpio3924_led_set;
   *(volatile u32 *)(RALINK_REG_PIO3924RESET) = ra_gpio3924_led_clr;
   *(volatile u32 *)(RALINK_REG_PIO7140SET) = ra_gpio7140_led_set;
   *(volatile u32 *)(RALINK_REG_PIO7140RESET) = ra_gpio7140_led_clr;
   *(volatile u32 *)(RALINK_REG_PIO9572SET) = ra_gpio9572_led_set;
   *(volatile u32 *)(RALINK_REG_PIO9572RESET) = ra_gpio9572_led_clr;
#endif
#endif // RALINK_GPIO_LED_LOW_ACT //

#if RALINK_LED_DEBUG
   printk("led_set= %x, led_clr= %x\n", ra_gpio_led_set, ra_gpio_led_clr);
#ifdef RALINK_GPIO_HAS_5124
   printk("led3924_set= %x, led3924_clr= %x\n", ra_gpio3924_led_set, ra_gpio3924_led_clr);
   printk("led5140_set= %x, led5140_clr= %x\n", ra_gpio5140_led_set, ra_gpio5140_led_set);
#endif
#ifdef RALINK_GPIO_HAS_9524
   printk("led3924_set= %x, led3924_clr= %x\n", ra_gpio3924_led_set, ra_gpio3924_led_clr);
   printk("led7140_set= %x, led7140_clr= %x\n", ra_gpio7140_led_set, ra_gpio7140_led_set);
   printk("led9572_set= %x, led9572_clr= %x\n", ra_gpio9572_led_set, ra_gpio9572_led_set);
#endif
#endif

   ra_gpio_led_set = ra_gpio_led_clr = 0;
#ifdef RALINK_GPIO_HAS_5124
   ra_gpio3924_led_set = ra_gpio3924_led_clr = 0;
   ra_gpio5140_led_set = ra_gpio5140_led_clr = 0;
#endif
#ifdef RALINK_GPIO_HAS_9524
   ra_gpio3924_led_set = ra_gpio3924_led_clr = 0;
   ra_gpio7140_led_set = ra_gpio7140_led_clr = 0;
   ra_gpio9572_led_set = ra_gpio9572_led_clr = 0;
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
#ifdef RALINK_GPIO_HAS_5124
   ra_gpio3924_led_set = 0xffff;
   ra_gpio5140_led_set = 0xfff;
#endif
#else
        ra_gpio_led_clr = 0xffffff;
//#ifdef RALINK_GPIO_HAS_9524
// ra_gpio3924_led_set = 0xffff;
// ra_gpio7140_led_set = 0xffffffff;
// ra_gpio9572_led_set = 0xffffff;
//#endif
//#else // RALINK_GPIO_LED_LOW_ACT //
// ra_gpio_led_clr = 0xffffff;
#ifdef RALINK_GPIO_HAS_5124
   ra_gpio3924_led_clr = 0xffff;
   ra_gpio5140_led_clr = 0xffff;
#endif
//#ifdef RALINK_GPIO_HAS_9524
// ra_gpio3924_led_clr = 0xffff;
// ra_gpio7140_led_clr = 0xffffffff;
// ra_gpio9572_led_clr = 0xffffff;
#endif
//#endif // RALINK_GPIO_LED_LOW_ACT //

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
   u32 reg1_gpio_dir;
   u32 Bits;

   printk(DRIVER_CONFIG " GPIO Driver initializing\n");

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
   {
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
   }
#endif

   //config these pins to gpio mode
   gpiomode = le32_to_cpu(*(volatile u32 *)(RALINK_REG_GPIOMODE));

#if defined (CONFIG_RALINK_RT3052) || defined (CONFIG_RALINK_RT2883) || defined(CONFIG_RALINK_RT3883) || defined(CONFIG_RALINK_RT5350)
   // Select GPIO & UARTF mode
   gpiomode &= ~RALINK_GPIOMODE_UARTF;
#if ENABLE_UARTF
   gpiomode |= RALINK_GPIOMODE_GPIO_UARTF;
#endif
#endif

#if ENABLE_I2C
   gpiomode &= ~RALINK_GPIOMODE_I2C;
#else
   gpiomode |= RALINK_GPIOMODE_I2C;
#endif

#if ENABLE_JTAG
   gpiomode &= ~RALINK_GPIOMODE_JTAG;
#else
   gpiomode |= RALINK_GPIOMODE_JTAG;
#endif

   *(volatile u32 *)(RALINK_REG_GPIOMODE) = cpu_to_le32(gpiomode);
   printk(KERN_EMERG "RALINK_REG_GPIOMODE = [0x%08X]\n", gpiomode);

   //enable gpio interrupt
   *(volatile u32 *)(RALINK_REG_INTENA) = cpu_to_le32(RALINK_INTCTL_PIO);
   for (i = 0; i < RALINK_GPIO_NUMBER; i++) {
      ralink_gpio_info[i].irq = i;
      ralink_gpio_info[i].pid = 0;
   }

   reg1_gpio_dir = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIODIR));
   PRINTK(KERN_EMERG "RALINK_REG_PIODIR before init 0x%08X\n", reg1_gpio_dir);

   Bits = GPIO_OUTPUTS & 0x3fffff;
   reg1_gpio_dir |= Bits;  // set output bits to output mode

   Bits = GPIO_INPUTS & 0x3fffff;
   reg1_gpio_dir &= ~Bits;    // set input bits to input mode
   *(volatile u32 *)(RALINK_REG_PIODIR) = cpu_to_le32(reg1_gpio_dir);
   reg1_gpio_dir = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIODIR));
   PRINTK(KERN_EMERG "RALINK_REG_PIODIR after init  0x%08X\n", reg1_gpio_dir);

   reg1_gpio_dir = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO5140DIR));
   PRINTK(KERN_EMERG "RALINK_REG_PIO5140DIR before init 0x%08X\n", reg1_gpio_dir);

   Bits = (GPIO_OUTPUTS >> Reg2_GPIO_Start) & 0x3f;
   reg1_gpio_dir |= Bits;  // set output bits to output mode

   Bits = (GPIO_INPUTS >> Reg2_GPIO_Start) & 0x3f;
   reg1_gpio_dir &= ~Bits;    // set input bits to input mode
   *(volatile u32 *)(RALINK_REG_PIO5140DIR) = cpu_to_le32(reg1_gpio_dir);
   reg1_gpio_dir = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO5140DIR));
   PRINTK(KERN_EMERG "RALINK_REG_PIO5140DIR after init  0x%08X\n", reg1_gpio_dir);

   // initialize outputs
   Bits = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIODATA));
   PRINTK(KERN_EMERG "RALINK_REG_PIODATA before init 0x%08X\n",Bits);
   Bits |= GPIO_HIGH_INIT;
   Bits &= ~GPIO_LOW_INIT;
   *(volatile u32 *)(RALINK_REG_PIODATA) = cpu_to_le32(Bits) & 0x3fffff;
   PRINTK(KERN_EMERG "RALINK_REG_PIODATA after init 0x%08X\n",Bits);

   Bits = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO5140DATA));
   PRINTK(KERN_EMERG "RALINK_REG_PIO5140DATA before init 0x%08X\n",Bits);
   Bits |= (GPIO_HIGH_INIT >> Reg2_GPIO_Start);
   Bits &= ~(GPIO_HIGH_INIT >> Reg2_GPIO_Start);
   *(volatile u32 *)(RALINK_REG_PIO5140DATA) = cpu_to_le32(Bits) & 0x3f;
   PRINTK(KERN_EMERG "RALINK_REG_PIO5140DATA after init 0x%08X\n",Bits);

#ifdef CONFIG_RALINK_GPIO_LED
   ralink_gpio_led_init_timer();
#endif
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

   {
      int i;
      for(i = 0; WeMoProcs[i].ProcName != NULL; i++) {
         remove_proc_entry(WeMoProcs[i].ProcName, NULL);
      }
   }

#ifdef WEMO_TIMER_PROC
   del_timer(&WemoTimer);
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
//#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,35)
// p = find_task_by_vpid(ralink_gpio_info[ralink_gpio_irqnum].pid);
//#else
   p = find_task_by_pid(ralink_gpio_info[ralink_gpio_irqnum].pid);
//#endif

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
#ifdef RALINK_GPIO_HAS_5124
   ralink_gpio3924_intp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO3924INT));
   ralink_gpio3924_edge = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO3924EDGE));
   *(volatile u32 *)(RALINK_REG_PIO3924INT) = cpu_to_le32(0x0000FFFF);
   *(volatile u32 *)(RALINK_REG_PIO3924EDGE) = cpu_to_le32(0x0000FFFF);
   ralink_gpio5140_intp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO5140INT));
   ralink_gpio5140_edge = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO5140EDGE));
   *(volatile u32 *)(RALINK_REG_PIO5140INT) = cpu_to_le32(0x00000FFF);
   *(volatile u32 *)(RALINK_REG_PIO5140EDGE) = cpu_to_le32(0x00000FFF);
#endif
#ifdef RALINK_GPIO_HAS_9524
   ralink_gpio3924_intp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO3924INT));
   ralink_gpio3924_edge = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO3924EDGE));
   *(volatile u32 *)(RALINK_REG_PIO3924INT) = cpu_to_le32(0x0000FFFF);
   *(volatile u32 *)(RALINK_REG_PIO3924EDGE) = cpu_to_le32(0x0000FFFF);
   ralink_gpio7140_intp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO7140INT));
   ralink_gpio7140_edge = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO7140EDGE));
   *(volatile u32 *)(RALINK_REG_PIO7140INT) = cpu_to_le32(0xFFFFFFFF);
   *(volatile u32 *)(RALINK_REG_PIO7140EDGE) = cpu_to_le32(0xFFFFFFFF);
   ralink_gpio9572_intp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO9572INT));
   ralink_gpio9572_edge = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO9572EDGE));
   *(volatile u32 *)(RALINK_REG_PIO9572INT) = cpu_to_le32(0x00FFFFFF);
   *(volatile u32 *)(RALINK_REG_PIO9572EDGE) = cpu_to_le32(0x00FFFFFF);
#endif
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0)
void ralink_gpio_irq_handler(unsigned int irq, struct irqaction *irqaction)
#else
irqreturn_t ralink_gpio_irq_handler(int irq, void *irqaction)
#endif
{
//+++Eric add
   extern unsigned long volatile jiffies;
//---Eric add
   struct gpio_time_record {
      unsigned long falling;
      unsigned long rising;
   };
   static struct gpio_time_record record[RALINK_GPIO_NUMBER];
   unsigned long now;
   int i;

   ralink_gpio_save_clear_intp();
   now = jiffies;
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
#ifdef RALINK_GPIO_HAS_5124
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
   for (i = 40; i < 52; i++) {
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
#endif
#ifdef RALINK_GPIO_HAS_9524
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
   for (i = 72; i < 96; i++) {
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
#endif

   return IRQ_HANDLED;
}

struct irqaction ralink_gpio_irqaction = {
   .handler = ralink_gpio_irq_handler,
//#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,35)
// .flags = IRQF_DISABLED,
//#else
   .flags = SA_INTERRUPT,
//#endif
   .name = "ralink_gpio",
};

void __init ralink_gpio_init_irq(void)
{
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


#define GPIO_TOTAL 28

#define GPIO_TEXT_LEN 10

#if HAS_POWER_BUTTON
int PowerButtonReadProc(char *buf,char **start,off_t off,int count,int *eof,void *data);
#endif

#if HAS_POWER_RELAY
int PowerRelayReadProc(char *buf,char **start,off_t off,int count,int *eof,void *data);
#endif



char btn_action[12]="", easy_link_btn_action[12] = "0";


struct proc_dir_entry *create_proc_read_write_entry(
   const char *name,
   mode_t mode,
   struct proc_dir_entry *base,
   read_proc_t *read_proc,
   write_proc_t *write_proc,
   void * data)
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
   char ProcName[32];
   struct proc_dir_entry *p;

   DEBUGP("Gemtek_GPIO_init\n");


   for(i = Reg1_GPIO_Start;i <= Reg2_GPIO_End; i++) {
      snprintf(ProcName,sizeof(ProcName),"GPIO%d",i);
      p = create_proc_read_write_entry(ProcName,0,NULL,
                                       GPIO_read_proc,
                                       GPIO_write_proc,
                                       (void *)i);

      if(p == NULL) {
         DEBUGP("%s : fail\n",ProcName);
      }
      else {
         DEBUGP("%s : success\n",ProcName);
      }
   }

   for(i = 0; WeMoProcs[i].ProcName != NULL; i++) {
      p = create_proc_read_write_entry(WeMoProcs[i].ProcName,0,NULL,
                                       WeMoProcs[i].ReadProc,
                                       WeMoProcs[i].WriteProc,
                                       WeMoProcs[i].Data);
      if(p == NULL) {
         printk(KERN_EMERG "%s: failed to register WeMoProc %s\n",
                __FUNCTION__,WeMoProcs[i].ProcName);
      }
      else {
         printk(KERN_NOTICE "%s: registered WeMoProc %s\n",
                __FUNCTION__,WeMoProcs[i].ProcName);
      }
   }

#ifdef WEMO_TIMER_PROC
// Start the WemoTimer
   init_timer(&WemoTimer);
   WemoTimer.expires = jiffies + 1;
   WemoTimer.function = WemoTimerProc;
   add_timer(&WemoTimer);
#endif
   return 0;
}


struct timer_list PLUGIN_LED_GPIO;

#if HAS_MOTION_SENSOR
struct timer_list MOTION_SENSOR_GPIO;
#endif

int plugin_gpio_led = LED_OFF ;
struct timer_list NET_GPIO;
int ledcount=0;

void WIFI_LED_BLUE_BLINK(unsigned long data)
{
#if HAS_WIFI_LEDS
   if(plugin_gpio_led == LED_OFF) {
      GPIO_write_bit(WIFI_BLUE_LED,WIFI_BLUE_LED_ON);
      plugin_gpio_led = LED_ON;
   }
   else {
      GPIO_write_bit(WIFI_BLUE_LED,WIFI_BLUE_LED_OFF);
      plugin_gpio_led = LED_OFF;
   }
#endif

   PLUGIN_LED_GPIO.expires = jiffies + (7 * (HZ/10));
   add_timer(&PLUGIN_LED_GPIO);
}

#if HAS_MOTION_SENSOR
void Detect_Motion_Scan(unsigned long data)
{
   u32 ret = 0;
   GPIO_read_bit(BACKLIGHT_LED, &ret);
   //printk("Motion=[%X]\n", ret);

   if(motion_sensor_delay <= 0)
   {
      motion_sensor_status = ret; // No delay, give PIN status directly
      printk("motion_sensor_status=[%d], Motion=[%d]\n", motion_sensor_status, ret);
   }
   else
   {// Delay Enabled
      printk("motion_sensor_status=[%d], Motion=[%d]\n", motion_sensor_status, ret);
      motion_delay_timer_count++;
      if(ret == 1)
         motion_count++;

      if(motion_delay_timer_count >= 2*motion_sensor_delay) // 500ms per detect, so *2
      {
         motion_delay_timer_count = 0; // Reset timer
         printk(KERN_EMERG "motion_count=[%d], motion_sensor_delay=[%d], percent=[%d/%d], motion_sensor_sensitivity=[%d]\n", motion_count, motion_sensor_delay, motion_count, 2*motion_sensor_delay, motion_sensor_sensitivity);
         if( (motion_count*100) >= (motion_sensor_sensitivity*2*motion_sensor_delay) )
         {
            motion_sensor_status = 1;
            printk(KERN_EMERG "----------------- MOTION -----------------\n");
         }
         else
         {
            motion_sensor_status = 0;
            printk(KERN_EMERG "----------------- NO MOTION -----------------\n");
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
#endif

#if HAS_WIFI_LEDS
void WIFI_LED_BLUE_OFF(unsigned long data)
{
   GPIO_write_bit(WIFI_BLUE_LED,WIFI_BLUE_LED_OFF);
}

void WIFI_LED_AMBER_BLINK(unsigned long data)
{
   if(plugin_gpio_led == LED_OFF) {
      GPIO_write_bit(WIFI_AMBER_LED,WIFI_AMBER_LED_ON);
      plugin_gpio_led = LED_ON;
   }
   else {
      GPIO_write_bit(WIFI_AMBER_LED,WIFI_AMBER_LED_OFF);
      plugin_gpio_led = LED_OFF;
   }
   PLUGIN_LED_GPIO.expires = jiffies + (7 * (HZ/10));
   add_timer(&PLUGIN_LED_GPIO);
}

void WIFI_LED_AMBER_BLINK_300(unsigned long data)
{
   if(plugin_gpio_led == LED_OFF) {
      GPIO_write_bit(WIFI_AMBER_LED,WIFI_AMBER_LED_ON);
      plugin_gpio_led = LED_ON;
   }
   else {
      GPIO_write_bit(WIFI_AMBER_LED,WIFI_AMBER_LED_OFF);
      plugin_gpio_led = LED_OFF;
   }
   PLUGIN_LED_GPIO.expires = jiffies + (3 * (HZ/10));
   add_timer(&PLUGIN_LED_GPIO);
}

void WIFI_LED_AMBER_BLINK_100(unsigned long data)
{
   if(plugin_gpio_led == LED_OFF) {
      GPIO_write_bit(WIFI_AMBER_LED,WIFI_AMBER_LED_ON);
      plugin_gpio_led = LED_ON;
   }
   else {
      GPIO_write_bit(WIFI_AMBER_LED,WIFI_AMBER_LED_OFF);
      plugin_gpio_led = LED_OFF;
   }
   PLUGIN_LED_GPIO.expires = jiffies + (1 * (HZ/10));
   add_timer(&PLUGIN_LED_GPIO);
}


void WIFI_LED_CHANGE_BLINK(unsigned long data)
{
   if(plugin_gpio_led == LED_OFF) {
      GPIO_write_bit(WIFI_AMBER_LED,WIFI_AMBER_LED_OFF);
      GPIO_write_bit(WIFI_BLUE_LED,WIFI_BLUE_LED_ON);
      plugin_gpio_led = LED_ON;
   }
   else {
      GPIO_write_bit(WIFI_AMBER_LED,WIFI_AMBER_LED_ON);
      GPIO_write_bit(WIFI_BLUE_LED,WIFI_BLUE_LED_OFF);
      plugin_gpio_led = LED_OFF;
   }
   PLUGIN_LED_GPIO.expires = jiffies + (7 * (HZ/10));
   add_timer(&PLUGIN_LED_GPIO);
}
#endif


/*
plugin_led_set

//  For reference original SNS LED behavour:
0: Startup/Connecting     - LED Blue Blink, 700 ms ON, 700 ms OFF
1: Connection established - LED Blue 30s ON, then OFF
2: Poor connection        - LED Amber Solid ON
3: No Connection          - LED Amber Blink, 700 ms ON, 700 ms OFF
4: On & Ok                - No LED
5: Setup mode             - 700 ms LED Blue, 700 ms LED Amber


//  Wemo Smart Module LED behavour (2/11/2013 spec):
// Note - read "Blue" as "Green" in the code below for this product
0: Startup/Connecting     - LED Green Blink, 700 ms ON, 700 ms OFF
1: Connection established - LED Green ON
2: Poor connection        - LED Amber Solid ON
3: No Connection          - LED Amber Blink, 700 ms ON, 700 ms OFF
4: On & Ok                - LED Green ON
5: Setup mode             - 700 ms LED Green, 700 ms LED Amber

6: Factory reset button pressed - LED Amber Blink, 300 ms ON, 300 ms OFF
7: Factory test
8: WASP communications failure - LED Amber Blink, 100 ms ON, 100 ms OFF
*/
void plugin_led_set(int ledcount)
{
#if HAS_WIFI_LEDS
   del_timer_sync(&PLUGIN_LED_GPIO);

   switch(ledcount) {
      case 0:
         // Startup/Connecting
         GPIO_write_bit(WIFI_AMBER_LED,WIFI_AMBER_LED_OFF);
         GPIO_write_bit(WIFI_BLUE_LED,WIFI_BLUE_LED_OFF);

         plugin_gpio_led = LED_OFF ;

         PLUGIN_LED_GPIO.expires = jiffies + (1 * (HZ/2));
         PLUGIN_LED_GPIO.function = WIFI_LED_BLUE_BLINK;
         add_timer(&PLUGIN_LED_GPIO);
         break;

      case 1:
         // Connection established
         GPIO_write_bit(WIFI_AMBER_LED,WIFI_AMBER_LED_OFF);
         GPIO_write_bit(WIFI_BLUE_LED,WIFI_BLUE_LED_ON);
         break;

      case 2:
         // Poor connection
         GPIO_write_bit(WIFI_BLUE_LED,WIFI_BLUE_LED_OFF);
         GPIO_write_bit(WIFI_AMBER_LED,WIFI_AMBER_LED_ON);
         break;

      case 3:
         // No Connection
         GPIO_write_bit(WIFI_AMBER_LED,WIFI_AMBER_LED_OFF);
         GPIO_write_bit(WIFI_BLUE_LED,WIFI_BLUE_LED_OFF);

         plugin_gpio_led = LED_OFF ;

         PLUGIN_LED_GPIO.expires = jiffies + (1 * (HZ/2));
         PLUGIN_LED_GPIO.function = WIFI_LED_AMBER_BLINK;
         add_timer(&PLUGIN_LED_GPIO);
         break;

      case 4:
         // On & Ok
         GPIO_write_bit(WIFI_BLUE_LED,WIFI_BLUE_LED_ON);
         GPIO_write_bit(WIFI_AMBER_LED,WIFI_AMBER_LED_OFF);
         break;

      case 5:
         // Setup mode
         GPIO_write_bit(WIFI_AMBER_LED,WIFI_AMBER_LED_OFF);
         GPIO_write_bit(WIFI_BLUE_LED,WIFI_BLUE_LED_OFF);

         plugin_gpio_led = LED_OFF ;

         PLUGIN_LED_GPIO.expires = jiffies + (1 * (HZ/2));
         PLUGIN_LED_GPIO.function = WIFI_LED_CHANGE_BLINK;
         add_timer(&PLUGIN_LED_GPIO);
         break;

      case 6:
         // Factory reset button pressed
         GPIO_write_bit(WIFI_AMBER_LED,WIFI_AMBER_LED_OFF);
         GPIO_write_bit(WIFI_BLUE_LED,WIFI_BLUE_LED_OFF);

         plugin_gpio_led = LED_OFF ;

         PLUGIN_LED_GPIO.expires = jiffies + (1 * (HZ/2));
         PLUGIN_LED_GPIO.function = WIFI_LED_AMBER_BLINK_300;
         add_timer(&PLUGIN_LED_GPIO);
         break;

      case 7:
         // Factory test
         GPIO_write_bit(WIFI_BLUE_LED,WIFI_BLUE_LED_ON);
         GPIO_write_bit(WIFI_AMBER_LED,WIFI_AMBER_LED_ON);
         break;

      case 8:
         // Communications with WASP interface card failed
         GPIO_write_bit(WIFI_AMBER_LED,WIFI_AMBER_LED_OFF);
         GPIO_write_bit(WIFI_BLUE_LED,WIFI_BLUE_LED_OFF);

         plugin_gpio_led = LED_OFF ;

         PLUGIN_LED_GPIO.expires = jiffies + (1 * (HZ/2));
         PLUGIN_LED_GPIO.function = WIFI_LED_AMBER_BLINK_100;
         add_timer(&PLUGIN_LED_GPIO);
         break;

   }
#endif
}

// Dummy to avoid link error
void small_led_set(int ledcount)
{
}


int PLUGIN_LED_GPIO_proc(struct file *file, const char __user *buffer,  unsigned long count, void *data)
{
   int len = 0;
   char buf[20]={0};
   int Ledtemp;

   len = count;

   if(copy_from_user(buf, buffer, len))
   {
      DEBUGP("fail.\n");
      return -EFAULT;
   }
   sscanf(buf,"%d",&Ledtemp);

   if(Ledtemp == -1) {
      ledcount = Ledtemp = 0;
   }

   if(ledcount == 8) {
   // Protect WASP communications failure from being over written by
   // a less important state
      printk(KERN_EMERG "PLUGIN_LED_GPIO_proc ignored attempt to set "
             "state %d in state 8\n",ledcount);
   }
   else {
      ledcount = Ledtemp;
      printk(KERN_EMERG "PLUGIN_LED_GPIO_proc ledcount=%d\n",ledcount);
      plugin_led_set(ledcount);
   }

   return len;
}

#if HAS_MOTION_SENSOR
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
   else  printk(KERN_EMERG "MOTION_SENSOR_STATUS_write_proc: ERROR!!\n");

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
#endif

int GPIO_read_proc(char *buf,char **start,off_t off,int count,int *eof,void *data)
{
   u32 ret = 0;
   int Bit = (int) data;

   GPIO_read_bit(Bit,&ret);

   return sprintf(buf,"%d\n",ret);
}


int GPIO_write_proc(struct file *file,const char __user *buffer,unsigned long count,void *data)
{
   int len = 0;
   char buf[10];
   int Bit = (int) data;

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

#if GPIO_DEBUG_ENABLE
   printk("%s: %c->GPIO%d\n",__FUNCTION__,buf[0],Bit);
#endif

   do {
#ifdef RELAY_ON_TIME
      if(Bit == RELAY_CONTROL) {
      // Special handling for the relay to pulsing
         if(buf[0]=='1' && RelayState != 1) {
         // Request to turn on the relay and it's not on already
            RelayState = 1;
            LastRelayChange = jiffies;
            GPIO_write_bit(Bit,1);
         }
         break;
      }
#elif defined(MIN_OUT_DELAY)
      if(Bit == RELAY_CONTROL) {
      // Special handling for the relay to limit rate change
         if(buf[0]=='1') {
            DesiredRelayState = 1;
         }
         else if(buf[0]=='0') {
            DesiredRelayState = 0;
         }

         if(RelayState != DesiredRelayState  &&
            (jiffies - LastRelayChange) >= MinOutDelay)
         {
            RelayState = DesiredRelayState;
            GPIO_write_bit(Bit,buf[0]=='1' ? 1:0);
            LastRelayChange = jiffies;
         }
      }
      break;
#endif

#if HAS_WASP_RESET
      if(Bit == WASP_RESET) {
      // Special handling to pulse reset line low
         if(buf[0]=='1') {
            WaspResetCount = (WASP_RESET_TIME * HZ) / 1000;
            buf[0] = '0';
         }
         else {
         // Ignore attempts to write 0
            break;
         }
      }
#endif

      GPIO_write_bit(Bit,buf[0]=='1' ? 1:0);
   } while(FALSE);

   return len;
}


void GPIO_setdir(u32 Bit,int dir_value)
{
      u32 tmp;
      u32 Mask = 1 << Bit;
      u32 DirReg;
      u32 Value;


      if(Bit < Reg2_GPIO_Start) {
         DirReg = RALINK_REG_PIODIR;
      }
      else {
         DirReg = RALINK_REG_PIO5140DIR;
         Mask >>= Reg2_GPIO_Start;
      }

      tmp = le32_to_cpu(*(volatile u32 *)(DirReg));

      if(dir_value==RALINK_GPIO_DIR_OUT) {
         Value = tmp | Mask;
      }
      else {
         Value = tmp & ~Mask;
      }

      if(Value != tmp) {
         *(volatile u32 *)(DirReg) = cpu_to_le32(Value);
#if GPIO_DEBUG_ENABLE
         printk("%s: Changed GPIO%d to %s mode\n",__FUNCTION__,Bit,
            dir_value ? "output" : "input");
#endif
      }
}

void GPIO_write_bit(u32 Bit,u32 set_value)
{
   u32 tmp;
   u32 Mask = 1 << Bit;
   u32 DataReg;
   u32 Value;

   if(Bit < Reg2_GPIO_Start) {
      DataReg = RALINK_REG_PIODATA;
   }
   else {
      DataReg = RALINK_REG_PIO5140DATA;
      Mask >>= Reg2_GPIO_Start;
   }

   GPIO_setdir(Bit,RALINK_GPIO_DIR_OUT);
   tmp = le32_to_cpu(*(volatile u32 *)(DataReg));

   if(set_value==1) {
      Value = tmp | Mask;
   }
   else {
      Value = tmp & ~Mask;
   }

   if(Value != tmp) {
      *(volatile u32 *)(DataReg) = cpu_to_le32(Value);
#if GPIO_DEBUG_ENABLE
      printk("%s: %d->GPIO%d\n",__FUNCTION__,set_value,Bit);
#endif
   }
}

void GPIO_read_bit(u32 Bit,u32 *get_value)
{
   u32 tmp;

   //GPIO_setdir(Bit,RALINK_GPIO_DIR_IN);
   //read gpio port
   if(Bit < Reg2_GPIO_Start) {
      tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIODATA));
   }
   else {
      tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO5140DATA));
      tmp <<= Reg2_GPIO_Start;
   }

   if(tmp & (1 << Bit)) {
      *get_value = 1;
   }
   else {
      *get_value = 0;
   }
#if GPIO_DEBUG_ENABLE
   printk("%s: GPIO%d: %d\n",__FUNCTION__,Bit,*get_value);
#endif
}

/* Gemtek GPIO end*/


#if SUPPORT_RESET_BUTTON

int   ResetCount = 0;

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
      GPIO_write_bit(POWER_BLUE_LED, LED_ON);
      GPIO_write_bit(POWER_BLUE_LED, LED_OFF);
      GPIO_write_bit(BACKLIGHT_LED, LED_ON);
      GPIO_write_bit(BACKLIGHT_LED, LED_OFF);

      GPIO_write_bit(RELAY_CONTROL, 1);
      GPIO_write_bit(RELAY_CONTROL, 0);


   }
   else if((ret == 0) && (ret1 == 1)) //POWER Button is pressed.
   {
      printk(KERN_EMERG "POWER BUTTON press!! - Lite up all LED\n");
      plugin_led_set(4);
      plugin_led_set(7);
      GPIO_write_bit(POWER_BLUE_LED, LED_OFF);
      GPIO_write_bit(POWER_BLUE_LED, LED_ON);
      GPIO_write_bit(BACKLIGHT_LED, LED_OFF);
      GPIO_write_bit(BACKLIGHT_LED, LED_ON);

      GPIO_write_bit(RELAY_CONTROL, 0);
      GPIO_write_bit(RELAY_CONTROL, 1);
   }

   return;
}
void GEMTEK_MP_Check(unsigned long data)
{
   del_timer_sync(&GTK_MP);/* Fix Kernel Panic */

   GEMTEK_MP_Scan();
   GTK_MP.expires = jiffies + (HZ/10);
   add_timer(&GTK_MP);
   return;
}

int gemtek_test_write_proc(struct file *file, const char __user *buffer, unsigned long count, void *data)
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
   remove_proc_entry("WPS_LED", NULL);

   for(i = 0;i < GPIO_TOTAL; i++)
   {
      remove_proc_entry(GPIO_LIST_NAME[i], NULL);
   }

      remove_proc_entry("SPEED", NULL);

   del_timer_sync(&GTK_MP);/* Fix Kernel Panic */

   GTK_MP.expires = jiffies + HZ;
   GTK_MP.function = GEMTEK_MP_Check;
   add_timer(&GTK_MP);

   len = sprintf(buf, "GEMTEK TEST\n");
   return len;
}
#endif


/* Gemtek LED start*/
static int __init LED_init(void)
{
   unsigned int ret = 0;
   printk(KERN_EMERG "Gemtek LED init...\n");

#if SUPPORT_RESET_BUTTON
   create_proc_read_entry("resetDefault",0,NULL,reset_btn_read_proc,NULL);
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

      /* Gemtek GPIO End */

      /* PLUGIN LED GPIO init */
      if(create_proc_read_write_entry("PLUGIN_LED_GPIO",0,NULL,NULL,PLUGIN_LED_GPIO_proc,NULL))
         DEBUGP("create_proc_read_write_entry  PLUGIN_LED_GPIO: fail\n");
      else
         DEBUGP("create_proc_read_write_entry  PLUGIN_LED_GPIO: success\n");

        init_timer(&PLUGIN_LED_GPIO);
      /* PLUGIN LED GPIO End */

      plugin_led_set(0);

      GPIO_read_bit(RESET_BUTTON,&ret);


      //GPIO_read_bit(POWER_BUTTON,&ret);// Reset Button not Ready, use Power Button to simulate.
      if(!ret) //Button is pressed.
      {
         plugin_led_set(6);
         printk(KERN_EMERG "################## Restore to Factory Defaults ###################\n");
         sprintf(btn_action, "RESET NOW!");
         //printk(KERN_EMERG "################## Restore to Factory Defaults, btn_action=[%s] ###################\n", btn_action);
      }
      else
         printk(KERN_EMERG "################## Don't Restore to Factory Defaults ###################\n");

#if HAS_MOTION_SENSOR
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


   return 1;
}

#ifdef POWER_DBL_CLICK
static void LogBadState()
{
   printk(KERN_ERR "Error: PowerButtonLast is %d in PowerButtonState %d\n",
          PowerButtonLast,PowerButtonState);
}

static void LogInvalidState()
{
   printk(KERN_ERR "Error: Invalid PowerButtonState %d\n",
          PowerButtonState);
   PowerButtonState = IDLE;
}

static int PowerButtonStateReadProc(char *buf,char **start,off_t off,int count,int *eof,void *data)
{
   buf[0] = ReportedButtonState + '0';
   buf[1] = '\n';
   buf[2] = 0;
   return 2;
}
#endif   // POWER_DBL_CLICK

#ifdef WEMO_TIMER_PROC
static void WemoTimerProc(unsigned long unused)
{
#ifdef POWER_DBL_CLICK
   int Current;

   GPIO_read_bit(POWER_BUTTON,&Current);

   if(PowerButtonLast != Current) {
   // Button has changed state
      unsigned int DeltaT = jiffies - LastChangeTime;
      PowerButtonLast = Current;
      LastChangeTime = jiffies;
      if(Current) {
      // Button released
         switch(PowerButtonState) {
         case IDLE:
         case CLICKED:
         case DBL_CLICKED:
         // shouldn't happen
            LogBadState();
            break;

         case PRESSED_1:
            if(DeltaT >= MinClickTime) {
            // A possible single click
               PowerButtonState = CLICKED_W;
            }
            else {
            // Clicked too fast, ignore it
               PowerButtonState = IDLE;
            }
            break;

         case HELD:
            PowerButtonState = IDLE;
            if((jiffies - ButtonReportedTime) > ClickClearDelay) {
            // Clear HELD state if it has been present long enough to be noticed
               ReportedButtonState = IDLE;
            }
            break;

         case PRESSED_2:
            if((jiffies - ClickStarted) < DoubleClickTime) {
            // A double click
               PowerButtonState = DBL_CLICKED;
               ReportedButtonState = DBL_CLICKED;
               ButtonReportedTime = jiffies;
            }
            else {
               PowerButtonState = IDLE;
            }
            break;

         case CLICKED_W:
            break;

         default:
            LogInvalidState();
            break;
         }
      }
      else {
      // Button pressed
         switch(PowerButtonState) {
         case IDLE:
         case CLICKED:
         case DBL_CLICKED:
         // New sequence starting
            ClickStarted = jiffies;
            PowerButtonState = PRESSED_1;
            break;

         case PRESSED_1:
         case HELD:
         case PRESSED_2:
         // shouldn't happen
            LogBadState();
            break;

         case CLICKED_W:
         // Not a single click, either a double click or a long hold
            PowerButtonState = PRESSED_2;
            break;

         default:
            LogInvalidState();
            break;
         }
      }
   }
   else if(!Current) {
   // Button has not changed state and it's pressed
      switch(PowerButtonState) {
         case PRESSED_1:
         case PRESSED_2:
            if((jiffies - ClickStarted) > MaxClickTime) {
            // Button is being held
               PowerButtonState = HELD;
               ReportedButtonState = HELD;
               ButtonReportedTime = jiffies;
            }
            break;

      case IDLE:
      case CLICKED_W:
      case CLICKED:
      case DBL_CLICKED:
      // shouldn't happen
         LogBadState();
         break;

      case HELD:
         break;

      default:
         LogInvalidState();
         break;
      }
   }
   else {
   // Button has not changed state, it's not pressed
      switch(PowerButtonState) {
      case IDLE:
      case HELD:
      case CLICKED:
      case DBL_CLICKED:
         break;

      case PRESSED_1:
      case PRESSED_2:
      // shouldn't happen
         LogBadState();
         break;

      case CLICKED_W:
         if((jiffies - ClickStarted) >= DoubleClickTime) {
         // Too late to be a double click
            PowerButtonState = CLICKED;
            ReportedButtonState = CLICKED;
            ButtonReportedTime = jiffies;
         }
         break;

      default:
         LogInvalidState();
         break;
      }
   }


   switch(ReportedButtonState) {
      case IDLE:
         break;

      case HELD:
         if(Current && (jiffies - ButtonReportedTime) > ClickClearDelay) {
            ReportedButtonState = IDLE;
         }
         break;

      case CLICKED:
      case DBL_CLICKED:
         if((jiffies - ButtonReportedTime) > ClickClearDelay) {
         // Clear last actions
            ReportedButtonState = IDLE;
         }
         break;

      case PRESSED_1:
      case PRESSED_2:
      case CLICKED_W:
      // shouldn't happen
         LogBadState();
         break;

      default:
         LogInvalidState();
         break;
   }
#endif   // POWER_DBL_CLICK

#ifdef MIN_OUT_DELAY
   if(RelayState != DesiredRelayState  &&
      (jiffies - LastRelayChange) >= MinOutDelay)
   {
      RelayState = DesiredRelayState;
      GPIO_write_bit(RELAY_CONTROL,RelayState);
      LastRelayChange = jiffies;
   }
#endif   // MIN_OUT_DELAY

#ifdef RELAY_ON_TIME
   if(RelayState && (jiffies - LastRelayChange) >= RelayOnTime) {
   // Time to turn off the relay
      RelayState = 0;
      GPIO_write_bit(RELAY_CONTROL,0);
      LastRelayChange = jiffies;
   }
#endif   // RELAY_ON_TIME

#ifdef PWM_SERIAL_PORT
   if(BacklightMode != 0 &&
      BacklightTarget != BacklightCurrent &&
      (jiffies - LastFadeChange) >= FadeTime)
   {  // Time to change the LED level
      if(BacklightTarget > BacklightCurrent) {
         BacklightCurrent++;
      }
      else {
         BacklightCurrent--;
      }
      UpdatePwmState();
      LastFadeChange = jiffies;
   }

   if(BacklightMode == 2 &&
      BacklightTarget == BacklightCurrent &&
      (jiffies - LastFadeChange) >= FadeTime)
   {  // Time to change direction of ramp
      if(BacklightTarget == 0) {
         BacklightTarget = 10;
      }
      else {
         BacklightTarget = 0;
      }
      UpdatePwmState();
      LastFadeChange = jiffies;
   }
#endif

#ifdef HAS_WASP_RESET
   if(WaspResetCount > 0) {
      if(--WaspResetCount == 0) {
      // Remove reset
         GPIO_write_bit(WASP_RESET,1);
      }
   }
#endif

   WemoTimer.expires = jiffies + 1;
   add_timer(&WemoTimer);
}
#endif   // WEMO_TIMER_PROC

#ifdef UINT_PROC_ENTRIES
static int UIntReadProc(char *buf,char **start,off_t off,int count,int *eof,void *data)
{
   return sprintf(buf,"%u\n",*((unsigned int *) data));
}

static int IntReadProc(char *buf,char **start,off_t off,int count,int *eof,void *data)
{
   return sprintf(buf,"%d\n",*((int *) data));
}

static int UIntWriteProc(
   struct file *file,
   const char __user *buffer,
   unsigned long count,
   void *data)
{
   char buf[10] = {0};
   int len = count;

   do {
      if(len > sizeof(buf)) {
         len = sizeof(buf) - 1;
      }

      if(copy_from_user(buf, buffer, len)) {
         len = -EFAULT;
         break;
      }

      sscanf(buf,"%u",(unsigned int *) data);
   } while(FALSE);

   return len;
}

static int IntWriteProc(
   struct file *file,
   const char __user *buffer,
   unsigned long count,
   void *data)
{
   char buf[10] = {0};
   int len = count;

   do {
      if(len > sizeof(buf)) {
         len = sizeof(buf) - 1;
      }

      if(copy_from_user(buf, buffer, len)) {
         len = -EFAULT;
         break;
      }

      sscanf(buf,"%d",(int *) data);
   } while(FALSE);

   return len;
}

#endif   // UINT_PROC_ENTRIES


#ifdef PWM_SERIAL_PORT

void UpdatePwmState()
{
#ifdef DEBUG_PWM
   printk("%s: Last %d, Current %d, Target %d\n",__FUNCTION__,
          BacklightLast,BacklightCurrent,BacklightTarget);
#endif

   if(BacklightLast == 0) {
   // We are sending a break, we need to end the break
      WemoPwmLED(pUartPort,WEMO_PWM_END_BREAK,BacklightCurrent);
   }

   if(BacklightCurrent >= 0) {
   // We're in PWM mode
      if(BacklightCurrent == 10) {
      // We need to stop sending to turn the LED on full
         WemoPwmLED(pUartPort,WEMO_PWM_STOP_TX,BacklightCurrent);
      }
      else if(BacklightCurrent == 0) {
      // We need to start sending a brake to turn the LED off
         WemoPwmLED(pUartPort,WEMO_PWM_SEND_BREAK,BacklightCurrent);
      }
      else if(BacklightLast < 0 || BacklightLast == 10) {
      // We weren't sending before, we need to start
         WemoPwmLED(pUartPort,WEMO_PWM_START_TX,BacklightCurrent);
      }
   }
   BacklightLast = BacklightCurrent;
}

int GetPwmByte(unsigned int iobase)
{
   int Ret = -1;
   if(iobase == PWM_SERIAL_PORT) {
      if(BacklightCurrent != BacklightLast) {
         UpdatePwmState();
      }
      Ret = BacklightCurrent;
   }

   return Ret;
}

// Return 1 if this is the PWM port
int WemoPwmLEDInit(void *p,int iobase)
{
   int Ret = 0;
   if(iobase == PWM_SERIAL_PORT) {
   // Save pointer to serial port for later
      pUartPort = p;
      Ret = 1;
   }

   return Ret;
}

static int BacklightWriteProc(
   struct file *file,
   const char __user *buffer,
   unsigned long count,
   void *data)
{
   int len = IntWriteProc(file,buffer,count,data);
   int *pInt = (int *) data;

// Force BacklightTarget, BacklightCurrent into legal range
   if(BacklightTarget < 0) {
      BacklightTarget = 0;
   }
   else if(BacklightTarget > 10) {
      BacklightTarget = 10;
   }

   if(BacklightCurrent < 0) {
      BacklightCurrent = -1;
   }
   else if(BacklightCurrent > 10) {
      BacklightCurrent = 10;
   }

   if(pInt == &BacklightCurrent) {
   // Force BacklightCurrent into legal range

      if(BacklightCurrent != BacklightLast) {
      // just in case we need to start/stop a break or whatever
         UpdatePwmState();
      }
   }
   else if(pInt == &BacklightMode) {
      if(BacklightMode < 0) {
         BacklightMode = 0;
      }
      else if(BacklightMode > 2) {
         BacklightMode = 2;
      }

      switch(BacklightMode) {
      case 0:
      // Disable PWM
         BacklightCurrent = -1;
         break;

      case 1:
      // Enable PWM, start with LED off
         BacklightCurrent = 0;
         BacklightTarget = 0;
         break;

      case 2:
      // Start a new ramp
         BacklightCurrent = 0;
         BacklightTarget = 10;
         break;
      }
      UpdatePwmState();
   }

   return len;
}

#endif   // PWM_SERIAL_PORT
