/*
 * MSE, ml-motofelica@nttd-mse.com
 * 04/27/2012 First Release of FeliCa device driver
 * 05/22/2012 Release of CEN, INT, RFS function
 * 05/29/2012 Update FeliCa driver
 * 06/05/2012 Fix of pointing out checkpatch.pl
 * 07/12/2012 Improvement of receiving push even if CPU is deep sleep
 *
 * Copyright(C) 2012 NTT DATA MSE CORPORATION. All right reserved.
 *
 * @file felica.c
 * @brief FeliCa code
 *
 * @date 2012/04/27
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 */

/*
 * include files
 */
#include <linux/err.h>
#include <linux/string.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/timer.h>
#include <linux/param.h>
#include <linux/poll.h>
#include <linux/delay.h>
#include <linux/wait.h>
#include <linux/vmalloc.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/fcntl.h>
#include <linux/proc_fs.h>
#include <linux/cdev.h>
#include <linux/gpio.h>
#include <linux/uaccess.h>
#include <asm/ioctls.h>
#include <linux/types.h>
#include <linux/syscalls.h>
#include <linux/sysfs.h>
#include <linux/ctype.h>
#include <linux/i2c.h>
#include <mach/gpiomux.h>
#include "felica.h"

/*
 * Function prottype
 */
/* External if */
static int __init FeliCa_init(void);
static void __exit FeliCa_exit(void);
static int FeliCa_dev_pon_open(struct inode *, struct file *);
static int FeliCa_dev_cen_open(struct inode *, struct file *);
static int FeliCa_dev_rfs_open(struct inode *, struct file *);
static int FeliCa_dev_itr_open(struct inode *, struct file *);
static int FeliCa_dev_pon_close(struct inode *, struct file *);
static int FeliCa_dev_cen_close(struct inode *, struct file *);
static int FeliCa_dev_rfs_close(struct inode *, struct file *);
static int FeliCa_dev_itr_close(struct inode *, struct file *);
static unsigned int FeliCa_dev_itr_poll(
				struct file *, struct poll_table_struct *);
static ssize_t FeliCa_dev_cen_read(struct file *, char *, size_t, loff_t *);
static ssize_t FeliCa_dev_itr_read(struct file *, char *, size_t, loff_t *);
static ssize_t FeliCa_dev_rfs_read(struct file *, char *, size_t, loff_t *);
static ssize_t FeliCa_dev_cen_write(
				struct file *, const char *, size_t, loff_t *);
static ssize_t FeliCa_dev_pon_write(
				struct file *, const char *, size_t, loff_t *);
static long FeliCa_dev_itr_ioctl(struct file * , unsigned int, unsigned long);
static int __devinit FeliCa_probe(
				struct i2c_client *client,
				const struct i2c_device_id *id);
static __devexit int FeliCa_remove(struct i2c_client *client);

/* Interrupt handler */
static irqreturn_t FeliCa_int_INT(int, void *);
static void FeliCa_timer_hnd_general(unsigned long);

/* Internal function */
static int FeliCa_ctrl_enable(struct FELICA_TIMER *);
static int FeliCa_ctrl_unenable(struct FELICA_TIMER *, struct FELICA_TIMER *);
static int FeliCa_ctrl_online(struct FELICA_TIMER *);
static int FeliCa_ctrl_offline(void);
static int FeliCa_term_cen_read(unsigned char *);
static int FeliCa_term_rfs_read(unsigned char *);
static int FeliCa_term_int_read(unsigned char *);
static int FeliCa_term_cen_write(unsigned char);
static int FeliCa_term_pon_write(unsigned char);
static void FeliCa_drv_timer_data_init(struct FELICA_TIMER *);
static void FeliCa_drv_init_cleanup(int);
static int FeliCa_i2c_read(
				unsigned char sub_addr,
				unsigned char *value,
				unsigned int conut);
static int FeliCa_i2c_write(unsigned char sub_addr, unsigned char value);


/*
 * static paramater & global
 */
static struct class *felica_class; /* device class information */
static struct i2c_client *felica_i2c_client;/* i2c class information */
static struct FELICA_DEV_INFO gPonCtrl;	/* device control info(PON) */
static struct FELICA_DEV_INFO gCenCtrl;	/* device control info(CEN) */
static struct FELICA_DEV_INFO gRfsCtrl;	/* device control info(RFS) */
static struct FELICA_DEV_INFO gItrCtrl;	/* device control info(ITR) */
static spinlock_t itr_lock; /* spin_lock param          */
static unsigned long itr_lock_flag; /* spin_lock flag           */
static struct cdev g_cdev_fpon; /* charactor device of PON  */
static struct cdev g_cdev_fcen; /* charactor device of CEN  */
static struct cdev g_cdev_frfs;	 /* charactor device of RFS  */
static struct cdev g_cdev_fitr; /* charactor device of ITR  */
static struct device *pon_device;
static dev_t dev_id_pon; /* Major/Minor Number(PON)  */
static dev_t dev_id_cen; /* Major/Minor Number(CEN)  */
static dev_t dev_id_rfs; /* Major/Minor Number(RFS)  */
static dev_t dev_id_itr; /* Major/Minor Number(ITR)  */
static dev_t dev_id_uart; /* Major/Minor Number(UART) */
EXPORT_SYMBOL_GPL(dev_id_uart);
static dev_t dev_id_rws; /* Major/Minor Number(RWS)  */
EXPORT_SYMBOL_GPL(dev_id_rws);

/* Interrupt Control Informaion */
static struct FELICA_ITR_INFO gFeliCaINTItrInfo = {
	.irq     = 0,
	.handler = FeliCa_int_INT,
	.flags   = IRQF_DISABLED|IRQF_TRIGGER_FALLING,
	.name    = FELICA_CTRL_ITR_STR_INT,
	.dev     = NULL
};

/*
 * Regist External if
 */
static const struct file_operations felica_cen_fops = {
	.owner			= THIS_MODULE,
	.read			= FeliCa_dev_cen_read,
	.write			= FeliCa_dev_cen_write,
	.open			= FeliCa_dev_cen_open,
	.release		= FeliCa_dev_cen_close
};
static const struct file_operations felica_pon_fops = {
	.owner			= THIS_MODULE,
	.write			= FeliCa_dev_pon_write,
	.open			= FeliCa_dev_pon_open,
	.release		= FeliCa_dev_pon_close
};
static const struct file_operations felica_itr_fops = {
	.owner			= THIS_MODULE,
	.poll			= FeliCa_dev_itr_poll,
	.read			= FeliCa_dev_itr_read,
	.unlocked_ioctl	= FeliCa_dev_itr_ioctl,
	.open			= FeliCa_dev_itr_open,
	.release		= FeliCa_dev_itr_close
};
static const struct file_operations felica_rfs_fops = {
	.owner			= THIS_MODULE,
	.read			= FeliCa_dev_rfs_read,
	.open			= FeliCa_dev_rfs_open,
	.release		= FeliCa_dev_rfs_close
};

static struct gpiomux_setting felica_rfs_reset_suspend_config = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_UP,
};

static struct gpiomux_setting felica_rfs_reset_active_config = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_UP,
};

static struct msm_gpiomux_config felica_rfs_reset_gpio_config = {
	.gpio = FELICA_GPIO_PORT_RFS,
	.settings = {
		[GPIOMUX_SUSPENDED] = &felica_rfs_reset_suspend_config,
		[GPIOMUX_ACTIVE] = &felica_rfs_reset_active_config,
	},
};


/*
 * struct of I2C driver
 */

static struct i2c_device_id felica_id_table[] = {
	{ "felica", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, felica_id_table);


static struct i2c_driver felica_i2c_driver = {
	.id_table = felica_id_table,
	.probe = FeliCa_probe,
	.remove = FeliCa_remove,
	.driver = {
		.owner	= THIS_MODULE,
		.name = "felica",
	},
};


static int FeliCa_gpio_request_input(int gpio_no, const unsigned char *name)
{
	int rc = 0;

	rc = gpio_request(gpio_no, name);
	if (rc) {
		pr_err("%s: Failed to request gpio %d\n", __func__, gpio_no);
		return rc;
	}

	rc = gpio_direction_input(gpio_no);
	if (rc)
		pr_err("%s: Failed to direction_input gpio %d\n",
						__func__, gpio_no);

	return rc;
}


static int FeliCa_gpio_request_output(
						int gpio_no,
						const unsigned char *name,
						int value)
{
	int rc = 0;

	rc = gpio_request(gpio_no, name);
	if (rc) {
		pr_err("%s: Failed to request gpio %d\n", __func__, gpio_no);
		return rc;
	}

	rc = gpio_direction_output(gpio_no, value);
	if (rc)
		pr_err("%s: Failed to direction_output gpio %d\n",
						__func__, gpio_no);

	return rc;
}

/* gpio initial */
static int FeliCa_gpio_init(void)
{
	int ret = 0;

	/* GPIO initial setting */
	ret = FeliCa_gpio_request_output(
				FELICA_GPIO_PORT_PON, FELICA_GPIO_STR_PON, 0);
	if (ret)
		return ret;

	ret = FeliCa_gpio_request_output(
				FELICA_GPIO_PORT_LDO_EN,
				FELICA_GPIO_STR_LDO_EN, 1);
	if (ret)
		return ret;

	ret = FeliCa_gpio_request_output(
				FELICA_GPIO_PORT_LOCKCONT,
				FELICA_GPIO_STR_LOCKCONT, 0);
	if (ret)
		return ret;

	msm_gpiomux_install(&felica_rfs_reset_gpio_config, 1);
	ret = FeliCa_gpio_request_input(
				FELICA_GPIO_PORT_RFS
				, FELICA_GPIO_STR_RFS);
	if (ret)
		return ret;

	ret = FeliCa_gpio_request_input(
				FELICA_GPIO_PORT_INT
				, FELICA_GPIO_STR_INT);
	if (ret)
		return ret;

	return ret;
}


/* gpio terminate */
static void FeliCa_gpio_exit(void)
{
	gpio_free(FELICA_GPIO_PORT_PON);
	gpio_free(FELICA_GPIO_PORT_LDO_EN);
	gpio_free(FELICA_GPIO_PORT_LOCKCONT);
	gpio_free(FELICA_GPIO_PORT_RFS);
	gpio_free(FELICA_GPIO_PORT_INT);

	return;
}


/*
 * Functions
 */

/**
 *   @brief Initialize module function
 *
 *   @par   Outline:\n
 *          Register FeliCa device driver, \n
 *          and relate system call and the I/F function
 *
 *   @param none
 *
 *   @retval 0   Normal end
 *   @retval <0  Error no of register device
 *
 *   @par Special note
 *     - none
 **/
static int __init FeliCa_init(void)
{
	int ret = 0;
	int cleanup = DFD_CLUP_NONE;
	struct device *devt;

	memset(&gPonCtrl, 0x00, sizeof(gPonCtrl));
	memset(&gCenCtrl, 0x00, sizeof(gCenCtrl));
	memset(&gRfsCtrl, 0x00, sizeof(gRfsCtrl));
	memset(&gItrCtrl, 0x00, sizeof(gItrCtrl));

	/* init gpio */
	ret = FeliCa_gpio_init();
	if (ret < 0)
		return ret;

	/* init Interruput struct */
	gFeliCaINTItrInfo.irq = MSM_GPIO_TO_INT(FELICA_GPIO_PORT_INT);

	do {
		/* register_chrdev_region() for felica */
		ret = alloc_chrdev_region(&dev_id_pon,
						FELICA_CTRL_MINOR_NO_PON,
						FELICA_DEV_NUM,
						FELICA_DEV_NAME_CTRL_PON);
		if (ret < 0)
			break;

		cdev_init(&g_cdev_fpon, &felica_pon_fops);
		g_cdev_fpon.owner = THIS_MODULE;
		ret = cdev_add(
				&g_cdev_fpon,
				dev_id_pon,
				FELICA_DEV_NUM);
		if (ret < 0) {
			cleanup = DFD_CLUP_UNREG_CDEV_PON;
			break;
		}
		ret = alloc_chrdev_region(&dev_id_cen,
						FELICA_CTRL_MINOR_NO_CEN,
						FELICA_DEV_NUM,
						FELICA_DEV_NAME_CTRL_CEN);
		if (ret < 0) {
			cleanup = DFD_CLUP_CDEV_DEL_PON;
			break;
		}
		cdev_init(&g_cdev_fcen, &felica_cen_fops);
		g_cdev_fcen.owner = THIS_MODULE;
		ret = cdev_add(
				&g_cdev_fcen,
				dev_id_cen,
				FELICA_DEV_NUM);
		if (ret < 0) {
			cleanup = DFD_CLUP_UNREG_CDEV_CEN;
			break;
		}
		ret = alloc_chrdev_region(&dev_id_rfs,
						FELICA_CTRL_MINOR_NO_RFS,
						FELICA_DEV_NUM,
						FELICA_DEV_NAME_CTRL_RFS);
		if (ret < 0) {
			cleanup = DFD_CLUP_CDEV_DEL_CEN;
			break;
		}
		cdev_init(&g_cdev_frfs, &felica_rfs_fops);
		g_cdev_frfs.owner = THIS_MODULE;
		ret = cdev_add(
				&g_cdev_frfs,
				dev_id_rfs,
				FELICA_DEV_NUM);
		if (ret < 0) {
			cleanup = DFD_CLUP_UNREG_CDEV_RFS;
			break;
		}
		ret = alloc_chrdev_region(&dev_id_itr,
						FELICA_CTRL_MINOR_NO_ITR,
						FELICA_DEV_NUM,
						FELICA_DEV_NAME_CTRL_ITR);
		if (ret < 0) {
			cleanup = DFD_CLUP_CDEV_DEL_RFS;
			break;
		}
		cdev_init(&g_cdev_fitr, &felica_itr_fops);
		g_cdev_fitr.owner = THIS_MODULE;
		ret = cdev_add(
				&g_cdev_fitr,
				dev_id_itr,
				FELICA_DEV_NUM);
		if (ret < 0) {
			cleanup = DFD_CLUP_UNREG_CDEV_ITR;
			break;
		}
		ret = alloc_chrdev_region(&dev_id_uart,
						FELICA_COMM_MINOR_NO_COMM,
						FELICA_DEV_NUM,
						FELICA_DEV_NAME_COMM);
		if (ret < 0) {
			unregister_chrdev_region(dev_id_uart, FELICA_DEV_NUM);
			break;
		}
		ret = alloc_chrdev_region(&dev_id_rws,
						FELICA_RWS_MINOR_NO_RWS,
						FELICA_DEV_NUM,
						FELICA_DEV_NAME_RWS);
		if (ret < 0) {
			unregister_chrdev_region(dev_id_rws, FELICA_DEV_NUM);
			break;
		}

		/* class_create() for felica */
		felica_class = class_create(THIS_MODULE, FELICA_DEV_NAME);
		if (IS_ERR(felica_class)) {
			ret = PTR_ERR(felica_class);
			cleanup = DFD_CLUP_CDEV_DEL_ITR;
			break;
		}

		/* device_create() for felica */
		devt = device_create(
			felica_class,
			NULL,
			dev_id_pon,
			NULL,
			FELICA_DEV_NAME_CTRL_PON);
		if (IS_ERR(devt)) {
			ret = PTR_ERR(devt);
			cleanup = DFD_CLUP_CLASS_DESTORY;
			break;
		}
		pon_device = devt;
		devt = device_create(
				felica_class,
				NULL,
				dev_id_cen,
				NULL,
				FELICA_DEV_NAME_CTRL_CEN);
		if (IS_ERR(devt)) {
			ret = PTR_ERR(devt);
			cleanup = DFD_CLUP_DEV_DESTORY_PON;
			break;
		}
		devt = device_create(
				felica_class,
				NULL,
				dev_id_rfs,
				NULL,
				FELICA_DEV_NAME_CTRL_RFS);
		if (IS_ERR(devt)) {
			ret = PTR_ERR(devt);
			cleanup = DFD_CLUP_DEV_DESTORY_CEN;
			break;
		}
		devt = device_create(
				felica_class,
				NULL,
				dev_id_itr,
				NULL,
				FELICA_DEV_NAME_CTRL_ITR);
		if (IS_ERR(devt)) {
			ret = PTR_ERR(devt);
			cleanup = DFD_CLUP_DEV_DESTORY_RFS;
			break;
		}
		/* device_create() for felica */
		devt = device_create(
				felica_class,
				NULL,
				dev_id_uart,
				NULL,
				FELICA_DEV_NAME_COMM);
		if (IS_ERR(devt)) {
			ret = PTR_ERR(devt);
			cleanup = DFD_CLUP_DEV_DESTORY_ITR;
			break;
		}
		/* device_create() for felica_rws */
		devt = device_create(
				felica_class,
				NULL,
				dev_id_rws,
				NULL,
				FELICA_DEV_NAME_RWS);
		if (IS_ERR(devt)) {
			ret = PTR_ERR(devt);
			cleanup = DFD_CLUP_DEV_DESTORY_COMM;
			break;
		}

	} while (0);

	/* cleanup device driver when failing in the initialization f*/
	FeliCa_drv_init_cleanup(cleanup);

	if (ret != 0) {
		pr_err("FeliCa : device registration failed ret = %d\n", ret);
		return ret;
	}

	/* I2C driver initialize */
	ret = i2c_add_driver(&felica_i2c_driver);
	if (ret) {
		pr_err("FeliCa : i2c driver registration failed\n");
		return ret;
	}

	return ret;
}
module_init(FeliCa_init);

/**
 *   @brief Exit module function
 *
 *   @par   Outline:\n
 *          Erases subscription of device driver
 *
 *   @param none
 *
 *   @retval none
 *
 *   @par Special note
 *     - none
 **/
static void __exit FeliCa_exit(void)
{
	FeliCa_drv_init_cleanup(DFD_CLUP_ALL);
	FeliCa_gpio_exit();

	/* I2C driver delete */
	i2c_del_driver(&felica_i2c_driver);

	return;
}
module_exit(FeliCa_exit);

/**
 *   @brief poll function
 *
 *   @par   Outline:\n
 *          Acquires whether or not it is writable to the device,\n
 *          and whether or not it is readable to it
 *
 *   @param[in]     *pfile  pointer of file infomation struct
 *   @param[in/out] *pwait  polling table structure address
 *
 *   @retval Success\n
 *             Returns in the OR value of the reading, the writing in\n
 *               case reading\n
 *                 0                    block's there being\n
 *                 POLLIN | POLLRDNORM  not in the block\n
 *               case writing\n
 *                 0                    block's there being\n
 *                 POLLOUT | POLLWRNORM not in the block\n
 *           Failure\n
 *                 POLLHUP | POLLERR    not in the block of reading and writing
 *
 *   @par Special note
 *     - As for the writing in, because it doesn't use, \n
 *       it always specifies block prosecution.
 *
 **/
static unsigned int FeliCa_dev_itr_poll(
						struct file *pfile,
						struct poll_table_struct *pwait)
{
	unsigned int ret = 0;
	struct FELICA_CTRLDEV_ITR_INFO *pItr = NULL;

	if (pfile == NULL) {
		ret = (POLLHUP | POLLERR);
		return ret;
	}

	pItr = (struct FELICA_CTRLDEV_ITR_INFO *)gItrCtrl.Info;
	if (NULL == pItr) {
		ret = (POLLHUP | POLLERR);
		return ret;
	}

	poll_wait(pfile, &pItr->RcvWaitQ, pwait);

	/* Readable check */
	if (FELICA_EDGE_NON != pItr->INT_Flag)
		ret |= (POLLIN  | POLLRDNORM);


	/* Writeable is allways not block */
	ret |= (POLLOUT | POLLWRNORM);

	return ret;
}

/**
 *   @brief Writes felica drivers function
 *
 *   @par   Outline:\n
 *          Writes data in the device from the user space
 *
 *   @param[in] *pfile  pointer of file infomation struct
 *   @param[in] *buff   user buffer address which maintains the data to write
 *   @param[in]  count  The request data transfer size
 *   @param[in] *poff   The file position of the access by the user
 *
 *   @retval >0       Wrote data size
 *   @retval -EINVAL  Invalid argument
 *   @retval -ENODEV  No such device
 *   @retval -EIO     I/O error
 *   @retval -EFAULT  Bad address
 *
 *   @par Special note
 *     - none
 *
 **/
static ssize_t FeliCa_dev_cen_write(
				struct file	*pfile,
				const char	*buff,
				size_t		count,
				loff_t		*poff)
{
	ssize_t ret = 0;
	unsigned long ret_cpy = 0;
	int ret_sub = 0;
	struct FELICA_CTRLDEV_CEN_INFO *pCenCtrl = NULL;
	unsigned char local_buff[FELICA_OUT_SIZE];

	if ((pfile == NULL) || (buff == NULL))
		return -EINVAL;


	pCenCtrl = (struct FELICA_CTRLDEV_CEN_INFO *)gCenCtrl.Info;
	if (NULL == pCenCtrl)
		return -EINVAL;


	if (count != FELICA_OUT_SIZE)
		return -EINVAL;


	ret_cpy = copy_from_user(local_buff, buff, FELICA_OUT_SIZE);
	if (ret_cpy != 0)
		return -EFAULT;


	if (FELICA_OUT_L == local_buff[0]) {
		ret_sub = FeliCa_ctrl_unenable(
					  &pCenCtrl->PON_Timer
					, &pCenCtrl->CEN_Timer);
	} else if (FELICA_OUT_H == local_buff[0]) {
		ret_sub = FeliCa_ctrl_enable(
					  &pCenCtrl->CEN_Timer);
	} else {
		ret_sub = -EINVAL;
	}

	if (ret_sub == 0)
		ret = FELICA_OUT_SIZE;
	else
		ret = ret_sub;


	return ret;
}

/**
 *   @brief Writes felica drivers function
 *
 *   @par   Outline:\n
 *          Writes data in the device from the user space
 *
 *   @param[in] *pfile  pointer of file infomation struct
 *   @param[in] *buff   user buffer address which maintains the data to write
 *   @param[in]  count  The request data transfer size
 *   @param[in] *poff   The file position of the access by the user
 *
 *   @retval >0       Wrote data size
 *   @retval -EINVAL  Invalid argument
 *   @retval -ENODEV  No such device
 *   @retval -EIO     I/O error
 *   @retval -EFAULT  Bad address
 *
 *   @par Special note
 *     - none
 *
 **/
static ssize_t FeliCa_dev_pon_write(
				struct file	*pfile,
				const char	*buff,
				size_t		count,
				loff_t		*poff)
{
	ssize_t ret = 0;
	unsigned long ret_cpy = 0;
	int ret_sub = 0;
	struct FELICA_CTRLDEV_PON_INFO *pPonCtrl = NULL;
	unsigned char local_buff[FELICA_OUT_SIZE];

	if ((pfile == NULL) || (buff == NULL))
		return -EINVAL;


	pPonCtrl = (struct FELICA_CTRLDEV_PON_INFO *)gPonCtrl.Info;
	if (NULL == pPonCtrl)
		return -EINVAL;


	if (count != FELICA_OUT_SIZE)
		return -EINVAL;


	ret_cpy = copy_from_user(local_buff, buff, FELICA_OUT_SIZE);
	if (ret_cpy != 0)
		return -EFAULT;


	if (FELICA_OUT_L == local_buff[0])
		ret_sub = FeliCa_ctrl_offline();
	else if (FELICA_OUT_H == local_buff[0])
		ret_sub = FeliCa_ctrl_online(&(pPonCtrl->PON_Timer));
	else
		ret_sub = -EINVAL;


	if (ret_sub == 0)
		ret = FELICA_OUT_SIZE;
	else
		ret = ret_sub;

	return ret;
}

/**
 *   @brief Read felica drivers function
 *
 *   @par   Outline:\n
 *          Read data from the device to the user space
 *
 *   @param[in] *pfile  pointer of file infomation struct
 *   @param[in] *buff   user buffer address which maintains the data to read
 *   @param[in]  count  The request data transfer size
 *   @param[in] *poff   The file position of the access by the user
 *
 *   @retval >0        Read data size
 *   @retval -EINVAL   Invalid argument
 *   @retval -EFAULT   Bad address
 *   @retval -EBADF    Bad file number
 *   @retval -ENODEV   No such device
 *
 *   @par Special note
 *     - none
 *
 **/
static ssize_t FeliCa_dev_cen_read(
				struct file	*pfile,
				char		*buff,
				size_t		count,
				loff_t		*poff)
{
	ssize_t			ret = 0;
	unsigned char	local_buff[max(FELICA_OUT_SIZE, FELICA_EDGE_OUT_SIZE)];
	int				ret_sub = 0;
	unsigned char	w_rdata = 0;
	unsigned long	ret_cpy = 0;

	if ((pfile == NULL) || (buff == NULL))
		return -EINVAL;


	if (count != FELICA_OUT_SIZE)
		return -EINVAL;


	ret_sub = FeliCa_term_cen_read(&w_rdata);
	if (ret_sub != 0)
		return ret_sub;

	switch (w_rdata) {
	case DFD_OUT_H:
		local_buff[0] = FELICA_OUT_H;
		break;
	case DFD_OUT_L:
	default:
		local_buff[0] = FELICA_OUT_L;
		break;
	}

	ret_cpy = copy_to_user(buff, local_buff, FELICA_OUT_SIZE);
	if (ret_cpy != 0)
		return -EFAULT;
	else
		ret = FELICA_OUT_SIZE;


	return ret;
}

/**
 *   @brief Read felica drivers function
 *
 *   @par   Outline:\n
 *          Read data from the device to the user space
 *
 *   @param[in] *pfile  pointer of file infomation struct
 *   @param[in] *buff   user buffer address which maintains the data to read
 *   @param[in]  count  The request data transfer size
 *   @param[in] *poff   The file position of the access by the user
 *
 *   @retval >0        Read data size
 *   @retval -EINVAL   Invalid argument
 *   @retval -EFAULT   Bad address
 *   @retval -EBADF    Bad file number
 *   @retval -ENODEV   No such device
 *
 *   @par Special note
 *     - none
 *
 **/
static ssize_t FeliCa_dev_itr_read(
				struct file	*pfile,
				char		*buff,
				size_t		count,
				loff_t		*poff)
{
	ssize_t			ret = 0;
	unsigned char	local_buff[max(FELICA_OUT_SIZE, FELICA_EDGE_OUT_SIZE)];
	unsigned long	ret_cpy = 0;
	struct FELICA_CTRLDEV_ITR_INFO *pItr;

	if ((pfile == NULL) || (buff == NULL))
		return -EINVAL;


	pItr = (struct FELICA_CTRLDEV_ITR_INFO *)gItrCtrl.Info;
	if (NULL == pItr)
		return -EINVAL;


	if (count != FELICA_EDGE_OUT_SIZE)
		return -EINVAL;


	/* Begin disabling interrupts for protecting INT Flag */
	spin_lock_irqsave(&itr_lock, itr_lock_flag);

	local_buff[0] = pItr->Reserve_Flag;
	local_buff[1] = pItr->INT_Flag;
	pItr->Reserve_Flag = FELICA_EDGE_NON;
	pItr->INT_Flag = FELICA_EDGE_NON;

	/* End disabling interrupts for protecting INT Flag */
	spin_unlock_irqrestore(&itr_lock, itr_lock_flag);

	ret_cpy = copy_to_user(buff, local_buff, FELICA_EDGE_OUT_SIZE);
	if (ret_cpy != 0)
		return -EFAULT;
	else
		ret = FELICA_EDGE_OUT_SIZE;


	return ret;
}
/**
 *   @brief Read felica drivers function
 *
 *   @par   Outline:\n
 *          Read data from the device to the user space
 *
 *   @param[in] *pfile  pointer of file infomation struct
 *   @param[in] *buff   user buffer address which maintains the data to read
 *   @param[in]  count  The request data transfer size
 *   @param[in] *poff   The file position of the access by the user
 *
 *   @retval >0        Read data size
 *   @retval -EINVAL   Invalid argument
 *   @retval -EFAULT   Bad address
 *   @retval -EBADF    Bad file number
 *   @retval -ENODEV   No such device
 *
 *   @par Special note
 *     - none
 *
 **/
static ssize_t FeliCa_dev_rfs_read(
				struct file	*pfile,
				char		*buff,
				size_t		count,
				loff_t		*poff)
{
	ssize_t			ret = 0;
	unsigned char	local_buff[max(FELICA_OUT_SIZE, FELICA_EDGE_OUT_SIZE)];
	int				ret_sub = 0;
	unsigned char	w_rdata = 0;
	unsigned long	ret_cpy = 0;

	if ((pfile == NULL) || (buff == NULL))
		return -EINVAL;


	if (count != FELICA_OUT_SIZE)
		return -EINVAL;


	ret_sub = FeliCa_term_rfs_read(&w_rdata);
	if (ret_sub != 0)
		return ret_sub;


	switch (w_rdata) {
	case DFD_OUT_L:
		local_buff[0] = FELICA_OUT_RFS_L;
		break;
	case DFD_OUT_H:
	default:
		local_buff[0] = FELICA_OUT_RFS_H;
		break;
	}

	ret_cpy = copy_to_user(buff, local_buff, FELICA_OUT_SIZE);
	if (ret_cpy != 0)
		return -EFAULT;
	else
		ret = FELICA_OUT_SIZE;


	return ret;
}

static long FeliCa_dev_itr_ioctl(
						struct file *pfile,
						unsigned int cmd,
						unsigned long arg)
{
	unsigned char	rdata = 0;
	int				ret_data = 0;
	int				ret = 0;
	int				ret_sub;

	if (NULL == pfile)
		return -EINVAL;


	if (cmd == FIONREAD) { /* Test */
		ret_sub = FeliCa_term_int_read(&rdata);
		ret_data = (int)rdata;

		if (copy_to_user((void *)arg, &ret_data, sizeof(int)) != 0)
			return -EFAULT;

	} else {
		pr_err("FeliCa_ioctl cannot use command!\n");
		return -ENOSYS;
	}

	return ret;
}

/**
 *   @brief INT interrupts handler
 *
 *   @par   Outline:\n
 *          Called when INT interrups from FeliCa device
 *
 *   @param[in]  i_req     IRQ number
 *   @param[in] *pv_devid  All-round pointer\n
 *                         (The argument which was set
 *                         at the function "request_irq()")
 *   @retval IRQ_HANDLED  Interrupt was handled by this device
 *
 *   @par Special note
 *     - none
 *
 **/
static irqreturn_t FeliCa_int_INT(int i_req, void *pv_devid)
{
	struct FELICA_CTRLDEV_ITR_INFO *pItr =
				(struct FELICA_CTRLDEV_ITR_INFO *)gItrCtrl.Info;
	int ret_sub = 0;
	unsigned char rdata = DFD_OUT_H;

	if (NULL == pItr)
		return IRQ_HANDLED;


	ret_sub = FeliCa_term_int_read(&rdata);

	if ((ret_sub == 0) && (rdata == DFD_OUT_L)) {
		pItr->INT_Flag = FELICA_EDGE_L;
		wake_up_interruptible(&(pItr->RcvWaitQ));
	}

	return IRQ_HANDLED;
}


/**
 *   @brief Generic timer handler
 *
 *   @par   Outline:\n
 *          Called when timeout for generic
 *
 *   @param[in] p  Address of the timer structure
 *
 *   @retval none
 *
 *   @par Special note
 *     - none
 *
 **/
static void FeliCa_timer_hnd_general(unsigned long p)
{
	struct FELICA_TIMER *pTimer;

	pTimer = (struct FELICA_TIMER *)p;
	if (pTimer != NULL)
		wake_up_interruptible(&pTimer->wait);


	return;
}


/**
 *   @brief Enable FeliCa control
 *
 *   @param[in] *pTimer  pointer of timer data(CEN terminal control timer)
 *
 *   @retval 0        Normal end
 *   @retval -EINVAL  Invalid argument
 *   @retval -EBUSY   Device or resource busy
 *
 *   @par Special note
 *     - none
 **/
static int FeliCa_ctrl_enable(struct FELICA_TIMER *pTimer)
{
	int ret = 0;
	int ret_sub = 0;

	if (NULL == pTimer)
		return -EINVAL;


	ret_sub = timer_pending(&pTimer->Timer);

	if (ret_sub != 0)
		return -EBUSY;


	ret = FeliCa_term_cen_write(DFD_OUT_H);

	if (ret >= 0) {
		/* wait cen timer */
		pTimer->Timer.function = FeliCa_timer_hnd_general;
		pTimer->Timer.data	   = (unsigned long)pTimer;
		mod_timer(&pTimer->Timer,
				jiffies + FELICA_TIMER_CEN_TERMINAL_WAIT);

		wait_event_interruptible(
			pTimer->wait,
			(timer_pending(&pTimer->Timer) == 0));

		ret_sub = timer_pending(&pTimer->Timer);
		if (ret_sub == 1)
			del_timer(&pTimer->Timer);

	}

	return ret;
}

/**
 *   @brief Disable FeliCa control
 *
 *   @param[in] *pPonTimer  pointer of timer data(PON terminal control timer)
 *   @param[in] *pCenTimer  pointer of timer data(PON terminal control timer)
 *
 *   @retval 0        Normal end
 *   @retval -EINVAL  Invalid argument
 *   @retval -EBUSY   Device or resource busy
 *
 *   @par Special note
 *     - none
 **/
static int FeliCa_ctrl_unenable(
					struct FELICA_TIMER *pPonTimer,
					struct FELICA_TIMER *pCenTimer)
{
	int ret = 0;
	int ret_sub = 0;

	if ((NULL == pPonTimer) || (NULL == pCenTimer))
		return -EINVAL;


	(void)FeliCa_ctrl_offline();

	/* wait pon timer */
	pPonTimer->Timer.function = FeliCa_timer_hnd_general;
	pPonTimer->Timer.data	   = (unsigned long)pPonTimer;
	mod_timer(&pPonTimer->Timer,
				jiffies + FELICA_TIMER_PON_TERMINAL_WAIT);

	wait_event_interruptible(
		pPonTimer->wait,
		(timer_pending(&pPonTimer->Timer) == 0));

	ret_sub = timer_pending(&pPonTimer->Timer);
	if (ret_sub == 1)
		del_timer(&pPonTimer->Timer);


	ret = FeliCa_term_cen_write(DFD_OUT_L);

	if (ret == 0) {
		pCenTimer->Timer.function = FeliCa_timer_hnd_general;
		pCenTimer->Timer.data	   = (unsigned long)pCenTimer;
		mod_timer(&pCenTimer->Timer,
				jiffies + FELICA_TIMER_CEN_TERMINAL_WAIT);

		wait_event_interruptible(
			pCenTimer->wait,
			(timer_pending(&pCenTimer->Timer) == 0));

		ret_sub = timer_pending(&pCenTimer->Timer);
		if (ret_sub == 1)
			del_timer(&pCenTimer->Timer);

	}

	return ret;
}

/**
 *   @brief Enable FeliCa IC function
 *
 *   @param[in] *pTimer  pointer of timer data(PON terminal control timer)
 *
 *   @retval 0     Normal end
 *   @retval -EIO  I/O error
 *
 *   @par Special note
 *     - none
 **/
static int FeliCa_ctrl_online(struct FELICA_TIMER *pTimer)
{
	int ret = 0;
	int ret_sub = 0;
	unsigned char rdata;

	if (NULL == pTimer)
		return -EINVAL;


	ret_sub = FeliCa_term_cen_read(&rdata);
	if ((ret_sub != 0) || (rdata == DFD_OUT_L))
		ret = -EIO;


	if (ret == 0) {
		ret_sub = FeliCa_term_pon_write(DFD_OUT_H);
		if (ret_sub != 0)
			ret = ret_sub;


	}
	if (ret == 0) {
		pTimer->Timer.function = FeliCa_timer_hnd_general;
		pTimer->Timer.data     = (unsigned long)pTimer;
		mod_timer(&pTimer->Timer,
				jiffies + FELICA_TIMER_PON_TERMINAL_WAIT);

		wait_event_interruptible(
			pTimer->wait,
			(timer_pending(&pTimer->Timer) == 0));

		ret_sub = timer_pending(&pTimer->Timer);
		if (ret_sub == 1)
			del_timer(&pTimer->Timer);

	}

	return ret;
}

/**
 *   @brief Disable FeliCa IC function
 *
 *   @param  none
 *
 *   @retval 0     Normal end
 *   @retval -EIO  I/O error
 *
 *   @par Special note
 *     - none
 **/
static int FeliCa_ctrl_offline(void)
{
	int ret = 0;
	int ret_sub = 0;
	unsigned char rdata;

	ret_sub = FeliCa_term_cen_read(&rdata);
	if ((ret_sub != 0) || (rdata == DFD_OUT_L))
		ret = -EIO;

	if (ret == 0) {
		ret_sub = FeliCa_term_pon_write(DFD_OUT_L);

		if (ret_sub != 0)
			ret = ret_sub;

	}

	return ret;
}

/**
 *   @brief Read CEN Terminal settings
 *
 *   @param[out] cen_data  Status of the CEN terminal
 *
 *   @retval 0        Normal end
 *   @retval -EINVAL  Invalid argument
 *   @retval -EIO     I/O error
 *
 *   @par Special note
 *     - none
 **/
static int FeliCa_term_cen_read(unsigned char *cen_data)
{
	int ret = 0;
	char buf = 0x00;
	char enable_flg = 0;

	ret = FeliCa_i2c_read(FELICA_LOCKCONT_LOCK_ADDR, &buf, 1);

	enable_flg = (buf & FELICA_LOCKCONT_LOCKEN_BIT);

	if (ret == 0) {
		if (enable_flg == 1)
			*cen_data = DFD_OUT_H;
		else
			*cen_data = DFD_OUT_L;

	}

	return ret;
}

/**
 *   @brief Read RFS Terminal settings
 *
 *   @param[out] rfs_data  Status of the RFS terminal
 *
 *   @retval 0        Normal end
 *   @retval -EINVAL  Invalid argument
 *   @retval -EIO     I/O error
 *
 *   @par Special note
 *     - none
 **/
static int FeliCa_term_rfs_read(unsigned char *rfs_data)
{

	int ret = 0;
	int w_rdata;

	if (NULL == rfs_data)
		return -EINVAL;


	w_rdata = gpio_get_value(FELICA_GPIO_PORT_RFS);

	if (w_rdata == 0)
		*rfs_data = DFD_OUT_L;
	else
		*rfs_data = DFD_OUT_H;


	return ret;
}

/**
 *   @brief Read INT Terminal settings
 *
 *   @param[out] int_data  Status of the INT terminal
 *
 *   @retval 0        Normal end
 *   @retval -EINVAL  Invalid argument
 *
 *   @par Special note
 *     - none
 **/
static int FeliCa_term_int_read(unsigned char *int_data)
{
	int ret = 0;
	int w_rdata;

	if (NULL == int_data)
		return -EINVAL;

	w_rdata = gpio_get_value(FELICA_GPIO_PORT_INT);

	if (w_rdata == 0)
		*int_data = DFD_OUT_L;
	else
		*int_data = DFD_OUT_H;


	return ret;
}

/**
 *   @brief Write CEN Terminal settings
 *
 *   @param[in] cen_data  Value of the CEN terminal to set
 *
 *   @retval 0        Normal end
 *   @retval -EINVAL  Invalid argument
 *   @retval -EIO     I/O error
 *
 *   @par Special note
 *     - none
 **/
static int FeliCa_term_cen_write(unsigned char cen_data)
{
	int ret = 0;
	unsigned char w_wdata = 0;
	unsigned char rdata = 0;

	switch (cen_data) {
	case DFD_OUT_H:
		w_wdata = 0x01;
		break;
	case DFD_OUT_L:
		w_wdata = 0x00;
		break;
	default:
		ret = -EINVAL;
		break;
	}

	/* Check "busy bit" */
	ret = FeliCa_i2c_read(FELICA_LOCKCONT_EEPCNT_ADDR, &rdata, 1);

	/* Check only "busy bit" */
	rdata = rdata & FELICA_LOCKCONT_BUSY_BIT;

	if (rdata == FELICA_LOCKCONT_BUSY_BIT) {
		/* I2C is busy, so read retry */
		ret = FeliCa_i2c_read(FELICA_LOCKCONT_EEPCNT_ADDR, &rdata, 1);

	}

	/* Set SET2EEP bit */
	w_wdata = (w_wdata | FELICA_LOCKCONT_SET2EEP_BIT);

	ret = FeliCa_i2c_write(FELICA_LOCKCONT_LOCK_ADDR, w_wdata);

	return ret;
}

/**
 *   @brief Write PON Terminal settings
 *
 *   @param[in] pon_data  Value of the PON terminal to set
 *
 *   @retval 0        Normal end
 *   @retval -EINVAL  Invalid argument
 *   @retval -EIO     I/O error
 *
 *   @par Special note
 *     - none
 **/
static int FeliCa_term_pon_write(unsigned char pon_data)
{
	int ret = 0;
	int ret_sub = 0;
	u8 w_wdata = 0;

	switch (pon_data) {
	case DFD_OUT_L:
		w_wdata = 0;
		break;
	case DFD_OUT_H:
		w_wdata = 1;
		break;
	default:
		ret = -EINVAL;
		break;
	}

	if (ret == 0) {
		ret_sub = gpio_direction_output(FELICA_GPIO_PORT_PON, w_wdata);
		if (ret_sub != 0)
			ret = -EIO;

	}

	return ret;
}

/**
 *   @brief Initialize timer data
 *
 *   @param[in] pTimer  timer data
 *
 *   @retval none
 *
 *   @par Special note
 *     - none
 **/
static void FeliCa_drv_timer_data_init(struct FELICA_TIMER *pTimer)
{

	if (pTimer != NULL) {
		init_timer(&pTimer->Timer);
		init_waitqueue_head(&pTimer->wait);
	}

	return;
}


/**
 *   @brief cleanup driver setup
 *
 *   @par   Outline:\n
 *          For cleaning device driver setup info
 *
 *   @param[in] cleanup  cleanup point
 *
 *   @retval none
 *
 *   @par Special note
 *     - none
 **/
static void FeliCa_drv_init_cleanup(int cleanup)
{

	/* cleanup */
	switch (cleanup) {
	case DFD_CLUP_ALL:
	case DFD_CLUP_DEV_DESTORY_RWS:
		device_destroy(
			felica_class,
			dev_id_rws);
	case DFD_CLUP_DEV_DESTORY_COMM:
		device_destroy(
			felica_class,
			dev_id_uart);
	case DFD_CLUP_DEV_DESTORY_ITR:
		device_destroy(
			felica_class,
			dev_id_itr);
	case DFD_CLUP_DEV_DESTORY_RFS:
		device_destroy(
			felica_class,
			dev_id_rfs);
	case DFD_CLUP_DEV_DESTORY_CEN:
		device_destroy(
			felica_class,
			dev_id_cen);
	case DFD_CLUP_DEV_DESTORY_PON:
		device_destroy(
			felica_class,
			dev_id_pon);
	case DFD_CLUP_CLASS_DESTORY:
		class_destroy(felica_class);
	case DFD_CLUP_CDEV_DEL_ITR:
		cdev_del(&g_cdev_fitr);
	case DFD_CLUP_UNREG_CDEV_ITR:
		unregister_chrdev_region(
			dev_id_itr,
			FELICA_DEV_NUM);
	case DFD_CLUP_CDEV_DEL_RFS:
		cdev_del(&g_cdev_frfs);
	case DFD_CLUP_UNREG_CDEV_RFS:
		unregister_chrdev_region(
			dev_id_rfs,
			FELICA_DEV_NUM);
	case DFD_CLUP_CDEV_DEL_CEN:
		cdev_del(&g_cdev_fcen);
	case DFD_CLUP_UNREG_CDEV_CEN:
		unregister_chrdev_region(
			dev_id_cen,
			FELICA_DEV_NUM);
	case DFD_CLUP_CDEV_DEL_PON:
		cdev_del(&g_cdev_fpon);
	case DFD_CLUP_UNREG_CDEV_PON:
		unregister_chrdev_region(
			dev_id_pon,
			FELICA_DEV_NUM);
	case DFD_CLUP_NONE:
	default:
		break;
	}

	return;
}

/**
 *   @brief Open PON device contorl driver of FeliCa
 *
 *   @par   Outline:\n
 *          Initialize internal infomation, and count up open counter
 *
 *   @param[in] *pinode  pointer of inode infomation struct
 *   @param[in] *pfile   pointer of file infomation struct
 *
 *   @retval 0        Normal end
 *   @retval -EINVAL  Invalid argument
 *   @retval -EACCES  Permission denied
 *   @retval -ENOMEM  Out of memory
 *
 *   @par Special note
 *     - none
 **/
static int FeliCa_dev_pon_open(struct inode *pinode, struct file *pfile)
{
	int ret = 0;
	struct FELICA_CTRLDEV_PON_INFO *pCtrl = NULL;

	if ((NULL == pfile) || (NULL == pinode))
		return -EINVAL;


	if ((pfile->f_flags & O_ACCMODE) == O_RDONLY)
		return -EACCES;
	else if ((pfile->f_flags & O_ACCMODE) != O_WRONLY)
		return -EINVAL;

	if ((gPonCtrl.w_cnt == 0) && (gPonCtrl.Info != NULL)) {
		kfree(gPonCtrl.Info);
		gPonCtrl.Info = NULL;
	}

	pCtrl = gPonCtrl.Info;

	if (NULL == pCtrl) {
		pCtrl = kmalloc(sizeof(*pCtrl), GFP_KERNEL);
		if (NULL == pCtrl)
			return -ENOMEM;

		gPonCtrl.Info = pCtrl;
		FeliCa_drv_timer_data_init(&pCtrl->PON_Timer);

		gPonCtrl.w_cnt = 0;
	}

	gPonCtrl.w_cnt++;

	return ret;
}

/**
 *   @brief Open CEN device contorl driver of FeliCa
 *
 *   @par   Outline:\n
 *          Initialize internal infomation, and count up open counter
 *
 *   @param[in] *pinode  pointer of inode infomation struct
 *   @param[in] *pfile   pointer of file infomation struct
 *
 *   @retval 0        Normal end
 *   @retval -EINVAL  Invalid argument
 *   @retval -EACCES  Permission denied
 *   @retval -ENOMEM  Out of memory
 *
 *   @par Special note
 *     - none
 **/
static int FeliCa_dev_cen_open(struct inode *pinode, struct file *pfile)
{
	int ret = 0;
	struct FELICA_CTRLDEV_CEN_INFO *pCtrl = NULL;

	if ((NULL == pfile) || (NULL == pinode))
		return -EINVAL;


	if (((pfile->f_flags & O_ACCMODE) != O_RDONLY)
		&& ((pfile->f_flags & O_ACCMODE) != O_WRONLY)) {
		return -EINVAL;
	}

	if ((gCenCtrl.w_cnt == 0)
				&& (gCenCtrl.r_cnt == 0)
				&& (NULL != gCenCtrl.Info)) {
		kfree(gCenCtrl.Info);
		gCenCtrl.Info = NULL;
	}

	pCtrl = gCenCtrl.Info;

	if (NULL == pCtrl) {
		pCtrl = kmalloc(sizeof(*pCtrl), GFP_KERNEL);
		if (NULL == pCtrl)
			return -ENOMEM;

		gCenCtrl.Info = pCtrl;
		FeliCa_drv_timer_data_init(&pCtrl->PON_Timer);
		FeliCa_drv_timer_data_init(&pCtrl->CEN_Timer);

		gCenCtrl.r_cnt = 0;
		gCenCtrl.w_cnt = 0;
	}

	if ((pfile->f_flags & O_ACCMODE) == O_RDONLY)
		gCenCtrl.r_cnt++;
	else
		gCenCtrl.w_cnt++;


	return ret;
}

/**
 *   @brief Open RFS device contorl driver of FeliCa
 *
 *   @par   Outline:\n
 *          Initialize internal infomation, and count up open counter
 *
 *   @param[in] *pinode  pointer of inode infomation struct
 *   @param[in] *pfile   pointer of file infomation struct
 *
 *   @retval 0        Normal end
 *   @retval -EINVAL  Invalid argument
 *   @retval -EACCES  Permission denied
 *
 *   @par Special note
 *     - none
 **/
static int FeliCa_dev_rfs_open(struct inode *pinode, struct file *pfile)
{
	int ret = 0;

	if ((NULL == pfile) || (NULL == pinode))
		return -EINVAL;


	if ((pfile->f_flags & O_ACCMODE) == O_WRONLY)
		return -EACCES;
	else if ((pfile->f_flags & O_ACCMODE) != O_RDONLY)
		return -EINVAL;

	gRfsCtrl.r_cnt++;

	return ret;
}

/**
 *   @brief Open ITR device contorl driver of FeliCa
 *
 *   @par   Outline:\n
 *          Initialize internal infomation, and count up open counter
 *
 *   @param[in] *pinode  pointer of inode infomation struct
 *   @param[in] *pfile   pointer of file infomation struct
 *
 *   @retval 0        Normal end
 *   @retval -EINVAL  Invalid argument
 *   @retval -EACCES  Permission denied
 *   @retval -ENOMEM  Out of memory
 *
 *   @par Special note
 *     - none
 **/
static int FeliCa_dev_itr_open(struct inode *pinode, struct file *pfile)
{
	int ret = 0;
	struct FELICA_CTRLDEV_ITR_INFO *pCtrl = NULL;

	if ((NULL == pfile) || (NULL == pinode))
		return -EINVAL;


	if ((pfile->f_flags & O_ACCMODE) == O_WRONLY)
		return -EACCES;
	else if ((pfile->f_flags & O_ACCMODE) != O_RDONLY)
		return -EINVAL;

	if ((gItrCtrl.r_cnt == 0) && (gItrCtrl.Info != NULL)) {
		kfree(gItrCtrl.Info);
		gItrCtrl.Info = NULL;
	}

	pCtrl = gItrCtrl.Info;

	if (NULL == pCtrl) {
		pCtrl = kmalloc(sizeof(*pCtrl), GFP_KERNEL);
		if (NULL == pCtrl)
			return -ENOMEM;

		gItrCtrl.Info = pCtrl;
		init_waitqueue_head(&pCtrl->RcvWaitQ);
		spin_lock_init(&itr_lock);
		gItrCtrl.r_cnt = 0;
		gItrCtrl.w_cnt = 0;
		pCtrl->INT_Flag = FELICA_EDGE_NON;
		pCtrl->Reserve_Flag = FELICA_EDGE_NON;
		gFeliCaINTItrInfo.dev = pCtrl;
	}

	if (gItrCtrl.r_cnt == 0) {
		ret = request_irq(
					gFeliCaINTItrInfo.irq,
					gFeliCaINTItrInfo.handler,
					gFeliCaINTItrInfo.flags,
					gFeliCaINTItrInfo.name,
					gFeliCaINTItrInfo.dev);

		if (ret == 0) {
			ret = enable_irq_wake(gFeliCaINTItrInfo.irq);

			if (ret != 0)
				free_irq(
					gFeliCaINTItrInfo.irq,
					gFeliCaINTItrInfo.dev);
		}
	}

	if (ret == 0) {
		gItrCtrl.r_cnt++;
	} else if (gItrCtrl.r_cnt == 0) {
		kfree(gItrCtrl.Info);
		gItrCtrl.Info = NULL;
	}

	return ret;
}

/**
 *   @brief Close PON device contorl driver of FeliCa
 *
 *   @par   Outline:\n
 *          Free internal infomation, and count down open counter
 *
 *   @param[in] *pinode  pointer of inode infomation struct
 *   @param[in] *pfile   pointer of file infomation struct
 *
 *   @retval 0        Normal end
 *   @retval -EINVAL  Invalid argument
 *
 *   @par Special note
 *     - none
 **/
static int FeliCa_dev_pon_close(struct inode *pinode, struct file *pfile)
{
	int    ret = 0;
	struct FELICA_CTRLDEV_PON_INFO *pCtrl =
		(struct FELICA_CTRLDEV_PON_INFO *)gPonCtrl.Info;

	if ((NULL == pfile) || (NULL == pinode))
		return -EINVAL;


	if (NULL == pCtrl)
		return -EINVAL;


	if (((pfile->f_flags & O_ACCMODE) == O_WRONLY) && (gPonCtrl.w_cnt > 0))
		gPonCtrl.w_cnt--;
	else
		return -EINVAL;

	if (gPonCtrl.w_cnt == 0) {
		if (timer_pending(&pCtrl->PON_Timer.Timer) == 1) {
			del_timer(&pCtrl->PON_Timer.Timer);
			wake_up_interruptible(&pCtrl->PON_Timer.wait);
		}

		(void)FeliCa_term_pon_write(DFD_OUT_L);

		kfree(gPonCtrl.Info);
		gPonCtrl.Info = NULL;
	}

	return ret;
}

/**
 *   @brief Close CEN device contorl driver of FeliCa
 *
 *   @par   Outline:\n
 *          Free internal infomation, and count down open counter
 *
 *   @param[in] *pinode  pointer of inode infomation struct
 *   @param[in] *pfile   pointer of file infomation struct
 *
 *   @retval 0        Normal end
 *   @retval -EINVAL  Invalid argument
 *
 *   @par Special note
 *     - none
 **/
static int FeliCa_dev_cen_close(struct inode *pinode, struct file *pfile)
{
	int    ret = 0;
	struct FELICA_CTRLDEV_CEN_INFO *pCtrl =
				(struct FELICA_CTRLDEV_CEN_INFO *)gCenCtrl.Info;

	if ((NULL == pfile) || (NULL == pinode))
		return -EINVAL;


	if (NULL == pCtrl)
		return -EINVAL;


	if (((pfile->f_flags & O_ACCMODE) == O_RDONLY)
						&& (gCenCtrl.r_cnt > 0)) {
		gCenCtrl.r_cnt--;
	} else if (((pfile->f_flags & O_ACCMODE) == O_WRONLY)
						&& (gCenCtrl.w_cnt > 0)) {
		gCenCtrl.w_cnt--;
	} else {
		return -EINVAL;
	}

	if ((gCenCtrl.r_cnt == 0) && (gCenCtrl.w_cnt == 0)) {
		if (timer_pending(&pCtrl->PON_Timer.Timer) == 1) {
			del_timer(&pCtrl->PON_Timer.Timer);
			wake_up_interruptible(&pCtrl->PON_Timer.wait);
		}

		if (timer_pending(&pCtrl->CEN_Timer.Timer) == 1) {
			del_timer(&pCtrl->CEN_Timer.Timer);
			wake_up_interruptible(&pCtrl->CEN_Timer.wait);
		}

		kfree(gCenCtrl.Info);
		gCenCtrl.Info = NULL;
	}

	return ret;
}

/**
 *   @brief Close RFS device contorl driver of FeliCa
 *
 *   @par   Outline:\n
 *          Free internal infomation, and count down open counter
 *
 *   @param[in] *pinode  pointer of inode infomation struct
 *   @param[in] *pfile   pointer of file infomation struct
 *
 *   @retval 0        Normal end
 *   @retval -EINVAL  Invalid argument
 *
 *   @par Special note
 *     - none
 **/
static int FeliCa_dev_rfs_close(
					struct inode *pinode,
					struct file  *pfile)
{
	int ret = 0;

	if ((NULL == pfile) || (NULL == pinode))
		return -EINVAL;


	if (((pfile->f_flags & O_ACCMODE) == O_RDONLY)
						&& (gRfsCtrl.r_cnt > 0)) {
		gRfsCtrl.r_cnt--;
	} else {
		return -EINVAL;
	}

	return ret;
}

/**
 *   @brief Close ITR device contorl driver of FeliCa
 *
 *   @par   Outline:\n
 *          Free internal infomation, and count down open counter
 *
 *   @param[in] *pinode  pointer of inode infomation struct
 *   @param[in] *pfile   pointer of file infomation struct
 *
 *   @retval 0        Normal end
 *   @retval -EINVAL  Invalid argument
 *
 *   @par Special note
 *     - none
 **/
static int FeliCa_dev_itr_close(
					struct inode *pinode,
					struct file  *pfile)
{
	int ret = 0;
	struct FELICA_CTRLDEV_ITR_INFO *pCtrl =
				(struct FELICA_CTRLDEV_ITR_INFO *)gItrCtrl.Info;

	if ((NULL == pfile) || (NULL == pinode))
		return -EINVAL;


	if (NULL == pCtrl)
		return -EINVAL;


	if (((pfile->f_flags & O_ACCMODE) == O_RDONLY)
		&& (gItrCtrl.r_cnt > 0)
		&& (gItrCtrl.r_cnt <= FELICA_DEV_ITR_MAX_OPEN_NUM)) {
		gItrCtrl.r_cnt--;
	} else {
		return -EINVAL;
	}

	wake_up_interruptible(&pCtrl->RcvWaitQ);

	if (gItrCtrl.r_cnt == 0) {
		disable_irq_wake(gFeliCaINTItrInfo.irq);
		free_irq(
			gFeliCaINTItrInfo.irq,
			gFeliCaINTItrInfo.dev);
		kfree(gItrCtrl.Info);
		gItrCtrl.Info = NULL;
	}

	return ret;
}

static int FeliCa_probe(struct i2c_client *client,
				 const struct i2c_device_id *id)
{

	if (client == NULL) {
		pr_err("FeliCa_probe client is NULL!\n");
		return 0;
	}

	felica_i2c_client = client;

	return 0;
}

static __devexit int FeliCa_remove(struct i2c_client *client)
{

	felica_i2c_client = NULL;

	return 0;
}

/**
 *   @brief I2C Read Module - FeliCa Utility
 *
 *   @param[in] sub_addr  register address
 *   @param[out] *value  read buffer
 *   @param[in] count  read count
 *
 *   @retval 0        Normal end
 *   @retval -EINVAL  Invalid argument
 *   @retval -EIO     I/O error
 *
 *   @par Special note
 *     - none
 **/
static int FeliCa_i2c_read(
						unsigned char sub_addr,
						unsigned char *value,
						unsigned int conut)
{
	int ret;
	struct i2c_msg msg[2];

	/* send sub address */
	msg[0].addr = felica_i2c_client->addr;
	msg[0].len = 1;
	msg[0].flags = 0;			/* Write the register value */
	msg[0].buf = &sub_addr;

	/* data receive settings */
	msg[1].addr = felica_i2c_client->addr;
	msg[1].len = conut;	/* only n bytes */
	msg[1].flags = I2C_M_RD;	/* Read the register value */
	msg[1].buf = value;

	ret = i2c_transfer(felica_i2c_client->adapter, msg, 2);

	/* i2c_transfer returns number of messages transferred */
	if (ret != 2) {
		pr_err("%s: i2c_read failed to transfer all messages\n",
								"FeliCa Driver");
		if (ret < 0)
			return ret;
		else
			return -EIO;
	} else {
		return 0;
	}
}

/**
 *   @brief I2C Write Module - FeliCa Utility
 *
 *   @param[in] sub_addr  register address
 *   @param[in] value  write value
 *
 *   @retval 0        Normal end
 *   @retval -EINVAL  Invalid argument
 *   @retval -EIO     I/O error
 *
 *   @par Special note
 *     - none
 **/
static int FeliCa_i2c_write(unsigned char sub_addr, unsigned char value)
{
	int ret;
	unsigned char w_wdata[2];

	if (felica_i2c_client == NULL) {
		pr_err("i2c send error end: %d\n", -EIO);
		return -EIO;
	}

	w_wdata[0] = sub_addr;
	w_wdata[1] = value;

	ret = i2c_master_send(felica_i2c_client, (const char *)&w_wdata, 2);

	if (ret == -EREMOTEIO) { /* Retry, chip was in standby */
		usleep_range(6000, 10000);
		ret = i2c_master_send(felica_i2c_client,
						(const char *)&w_wdata, 2);
		pr_err("retry FeliCa_i2c_write: %d\n", ret);
	}

	/* i2c_master_send returns number of complete writing buffers */
	if (ret != 2) {
		pr_err("%s: i2c_write failed to write all messages\n",
						"FeliCa Driver");
		if (ret < 0)
			return ret;
		else
			return -EIO;
	} else {
		return 0;
	}
}

MODULE_AUTHOR("NTT DATA MSE Co., Ltd.");
MODULE_DESCRIPTION("FeliCa Driver");
MODULE_LICENSE("GPL");
