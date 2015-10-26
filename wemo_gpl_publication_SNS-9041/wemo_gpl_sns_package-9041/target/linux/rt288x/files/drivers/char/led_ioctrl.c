                                                             #include <linux/init.h>
#include <linux/version.h>
#include <linux/config.h>
#include <linux/module.h>
#include <linux/kernel.h>   
#include <asm/uaccess.h>

#ifdef  CONFIG_DEVFS_FS
#include <linux/devfs_fs_kernel.h>
#endif

#include <linux/proc_fs.h>

#define RDM_DEVNAME	    "ledctrl"
#ifdef  CONFIG_DEVFS_FS
static devfs_handle_t devfs_handle;
#endif
int led_ioctrl_major =  23;

#include "ralink_gpio_sns.h"

int led_ioctrl_open(struct inode *inode, struct file *filp)
{
	return 0;
}

int led_ioctrl_release(struct inode *inode, struct file *filp)
{
	return 0;
}

int led_ioctrl_ioctl(struct inode *inode, struct file *filp, int cmd,
		unsigned long arg)
{
	int rc = 0;
	int i ;
	int reg_addr, reg_value = 0;
	
	switch (cmd) {
		case GMTK_SYSTEM_READY:
			printk(KERN_EMERG"GTK LED Status: GMTK_SYSTEM_READY   \n");
			break;

		case GMTK_SYSTEM_BOOT:
			printk(KERN_EMERG"GTK LED Status: GMTK_SYSTEM_BOOT   \n");
			big_led_set(1);
			small_led_set(4);
			break;

		case GMTK_LAN_LINK_UP:		
			printk(KERN_EMERG"GTK LED Status: GMTK_LAN_LINK_UP   \n");
			break;

		case GMTK_LAN_LINK_ERROR:	
			printk(KERN_EMERG"GTK LED Status: GMTK_LAN_LINK_ERROR   \n");
			break;

		case GMTK_WAN_INTERNET_DOWN:
			printk(KERN_EMERG"GTK LED Status: GMTK_WAN_INTERNET_DOWN   \n");
			big_led_set(3);
			small_led_set(4);
			break;
			
		case GMTK_WAN_MODEM_DOWN:
			printk(KERN_EMERG"GTK LED Status:  GMTK_WAN_MODEM_DOWN \n");
			big_led_set(3);
			small_led_set(4);
			break;

		case GMTK_WAN_INTERNET_CONNECTING:
			printk(KERN_EMERG"GTK LED Status: GMTK_WAN_INTERNET_CONNECTING   \n");
			big_led_set(2);
			small_led_set(4);
			break;

		case GMTK_WAN_INTERNET_CONNECTED:
			printk(KERN_EMERG"GTK LED Status: GMTK_WAN_INTERNET_CONNECTED   \n");
			big_led_set(0);
			small_led_set(4);
			break;	

		case GMTK_WAN_INTERNET_ERROR:
			printk(KERN_EMERG"GTK LED Status: GMTK_WAN_INTERNET_ERROR   \n");
			big_led_set(2);
			small_led_set(4);
			break;
	}
	return rc;
}

static const struct file_operations led_ioctrl_fops = {
	.owner = THIS_MODULE,
	.ioctl = led_ioctrl_ioctl,
	.open = led_ioctrl_open,
	.release = led_ioctrl_release,
};

static int led_ioctrl_init(void)
{
#ifdef  CONFIG_DEVFS_FS
	if (devfs_register_chrdev(led_ioctrl_major, RDM_DEVNAME , &led_ioctrl_fops)) {
		return -EIO;
	}

	devfs_handle = devfs_register(NULL, RDM_DEVNAME, DEVFS_FL_DEFAULT,
			led_ioctrl_major, 0, S_IFCHR | S_IRUGO | S_IWUGO, &led_ioctrl_fops,
			NULL);
#else
	int result=0;
	result = register_chrdev(led_ioctrl_major, RDM_DEVNAME, &led_ioctrl_fops);
	if (result < 0) {
		return result;
	}

	if (led_ioctrl_major == 0) {
		led_ioctrl_major = result; /* dynamic */
	}
#endif

	return 0;
}

static void led_ioctrl_exit(void)
{
	//printk(KERN_EMERG "gemtek_ledctrl_exit\n");
#ifdef  CONFIG_DEVFS_FS
	devfs_unregister_chrdev(led_ioctrl_major, RDM_DEVNAME);
	devfs_unregister(devfs_handle);
#else
	unregister_chrdev(led_ioctrl_major, RDM_DEVNAME);
#endif

}

module_init(led_ioctrl_init);
module_exit(led_ioctrl_exit);

#if LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,12)
MODULE_PARM (led_ioctrl_major, "i");
#else
module_param (led_ioctrl_major, int, 0);
#endif

MODULE_LICENSE("GPL");
