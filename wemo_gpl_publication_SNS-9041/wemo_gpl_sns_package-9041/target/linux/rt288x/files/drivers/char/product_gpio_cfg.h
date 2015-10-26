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


#ifndef _PRODUCT_GPIO_CFG_H_

// These defines configure the features compiled into ralink_gpio.c

#define DRIVER_CONFIG      "WeMo Smart Module"

#define HAS_MOTION_SENSOR  0
#define HAS_POWER_RELAY    0
#define HAS_RELAY_LED      0
#define HAS_POWER_BUTTON   0
#define HAS_WASP_RESET     1
#define HAS_WIFI_LEDS      1
#define GPIO_DEBUG_ENABLE  0

// set if GPIO1 and GPIO2 are used by the I2C interface
#define ENABLE_I2C         0

// set if GPIO 7 to GPIO 10 are used by the UARTF for RTS, TXD, CTS and RXD
#define ENABLE_UARTF       1

// set if GPIO17...GPIO21 are used by the JTAG interface
#define ENABLE_JTAG        0


// WiFi Logo Green LED, active high (out) (previously BLUE on the SNS)
// pin 6 - GPIO18
// NB: Some documentation shows GPIO0 which is WRONG.  It's actually GPIO18.
#define WIFI_BLUE_LED      18
#define WIFI_BLUE_LED_ON   1
#define WIFI_BLUE_LED_OFF  0

// WiFi Logo Green LED, active high (out)
// pin 7 - GPIO1
#define WIFI_AMBER_LED     1
#define WIFI_AMBER_LED_ON  1
#define WIFI_AMBER_LED_OFF 0

// Reset hardware interface card, active low (out)
// pin 8 - GPIO2
#define WASP_RESET         2
#define WASP_RESET_TIME    100   // pulse width in milliseconds

// WeMo factor reset/restore button, active low (in)
// pin 9 - GPIO11
#define RESET_BUTTON       11

#define RESET_TIME         10

#define POWER_BUTTON       13 // pin 10, not used
#define RELAY_LED          27 // pin 11, not used

#define GPIO_OUTPUTS       ((1 << WIFI_AMBER_LED) | (1 << WIFI_BLUE_LED) | \
                            (1 << WASP_RESET))
#define GPIO_INPUTS        (1 << RESET_BUTTON)

// Initialize these outputs high
#define GPIO_HIGH_INIT     (1 << WASP_RESET)

// Initialize these outputs low
#define GPIO_LOW_INIT      (0)


// Define POWER_DBL_CLICK to create a /proc/POWER_BUTTON_STATE
// to provide double click support for the power button
//
// /proc/POWER_BUTTON_STATE:
// 0: button not pressed
// 1: button was clicked once
// 2: button was double clicked
// 3: button was held for a "long" time.

// #define POWER_DBL_CLICK 1

// Define MIN_OUT_DELAY to limit number of times the output relay can
// be turned on and off a second.  Measured in Jiffies.
// i.e. HZ/2 means the output can be turned on or off twice a second.

// #define MIN_OUT_DELAY      HZ/2

// Define RELAY_ON_TIME to cause the relay to turn on for
// RELAY_ON_TIME and then turn off.
// When this mode is defined writes of a 1 to the relay control port
// turn on the relay and starts the timer, writes of a 0 are ignored.
// Writes of a 1 while the timer is running are ignored.
// The value of the define sets the default relay on time in jiffies.
// time can be changed by writing an new value to /proc/RELAY_ON_TIME
// i.e. to "# echo 800 >  /proc/RELAY_ON_TIME"

//#define RELAY_ON_TIME 10*HZ // 10 seconds (time is measured in jiffies)

// Define PWM_SERIAL_PORT to use a serial port as a poor man's PWM to
// allow the backlight LED to be dimmed.
//
// LED is controlled using the following /proc files:
//    /proc/BACKLIGHT_MODE
//       0 - PWM disabled, serial port acts normally
//       1 - PWM mode
//       2 - PWM debug/demo mode, continuous ramping up and down
//    /proc/BACKLIGHT_TARGET
//       Desired LED intensity value 0 (off) to 10 (full on) in 10% steps
//    /proc/BACKLIGHT_CURRENT
//       Current LED intensity value 0 (off) to 10 (full on) in 10% steps
//    /proc/FADE_TIME
//       Number of system ticks per step when fading from one LED intensity
//       to the next.  To disable fading set BACKLIGHT_MODE to 0 and
//       set BACKLIGHT_CURRENT to the desired level.
//
// Set PWM_SERIAL_PORT to the address of the LED's serial port.
//
// ttyS0: 0xb0000500
// ttyS1: 0xb0000c00

//#define PWM_SERIAL_PORT  0xb0000c00


#endif   // _PRODUCT_GPIO_CFG_H_
