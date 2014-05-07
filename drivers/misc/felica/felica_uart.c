/*
 * MSE, ml-motofelica@nttd-mse.com
 * 04/27/2012 First Release of FeliCa device driver
 * 05/29/2012 Update FeliCa driver
 * 06/05/2012 Fix of pointing out checkpatch.pl
 * 06/21/2012 Modify for SW tuning value
 *
 * Copyright(C) 2012 NTT DATA MSE CORPORATION. All right reserved.
 *
 * @file felica_uart.c
 * @brief FeliCa Driver
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
 *
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
#include <linux/slab.h>
#include <linux/timer.h>
#include <linux/param.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/wait.h>
#include <linux/vmalloc.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/fcntl.h>
#include <linux/cdev.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <asm/ioctls.h>
#include <linux/types.h>
#include <linux/syscalls.h>
#include <linux/device.h>
#include <linux/termios.h>
#include <linux/sysfs.h>
#include <linux/ctype.h>
#include "felica_uart.h"

/*
 * Function prottype
 */
static int     __init FeliCaUART_init(void);
static void    __exit FeliCaUART_exit(void);
static int     FeliCaUART_open(struct inode *, struct file *);
static int     FeliCaUART_close(struct inode *, struct file *);
static ssize_t FeliCaUART_write(struct file *, const char *, size_t, loff_t *);
static ssize_t FeliCaUART_read(struct file *, char *, size_t, loff_t *);
static long    FeliCaUART_ioctl(struct file * , unsigned int, unsigned long);
static int     FeliCaUART_fsync(struct file*, int);
static int  FeliCaUART_tty_open(void);
static void FeliCaUART_tty_close(void);
static int  FeliCaUART_tty_write(const char *, size_t);
static int  FeliCaUART_tty_read(char *, size_t);
static int  FeliCaUART_tty_fsync(void);
static int  FeliCaUART_tty_available(int *);

/*
 * sturut of reginsting External if
 */
static const struct file_operations felica_fops = {
	.owner			= THIS_MODULE,
	.read			= FeliCaUART_read,
	.write			= FeliCaUART_write,
	.unlocked_ioctl = FeliCaUART_ioctl,
	.fsync			= FeliCaUART_fsync,
	.open			= FeliCaUART_open,
	.release		= FeliCaUART_close
};

/*
 * static paramater & global
 */
static struct cdev g_cdev_fuart;
static spinlock_t timer_lock;
static struct file *uart_tty_filp;
static int uart_tty_open_count;
static unsigned char uart_tty_readbuff[UART_TTY_READBUFF_SIZE];
static unsigned char uart_tty_writebuff[UART_TTY_WRITEBUFF_SIZE];







static int FeliCaUART_tty_open(void)
{
	mm_segment_t	oldfs;
	struct file		*filp;
	struct inode	*inode = NULL;
	struct termios	options;


	oldfs = get_fs();
	set_fs(KERNEL_DS);

	filp = filp_open(TTY_FILE_PATH,
		O_RDWR,
		S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH | S_IWOTH);

	if (IS_ERR(filp)) {
		pr_err("filp_open error\n");
		set_fs(oldfs);
		return -EIO;
	}

	inode = filp->f_path.dentry->d_inode;
	if (!filp->f_op || !inode) {
		filp_close(filp, NULL);
		set_fs(oldfs);
		return -EIO;
	}

	if (filp->f_op->unlocked_ioctl(filp,
				(unsigned int)TCFLSH,
				(unsigned long)TCIOFLUSH) < 0) {
		filp_close(filp, NULL);
		set_fs(oldfs);
		return -EIO;
	}

	if (filp->f_op->unlocked_ioctl(filp,
				(unsigned int)TCGETS,
				(unsigned long)&options) < 0) {
		filp_close(filp, NULL);
		set_fs(oldfs);
		return -EIO;
	}

	options.c_cc[VTIME] = 0;
	options.c_cc[VMIN]  = 0;
	options.c_cflag &= ~CSIZE;
	options.c_cflag |= (CS8 | CLOCAL | CREAD | CRTSCTS);
	options.c_iflag = IGNPAR;
	options.c_oflag = 0;
	options.c_lflag = 0;
	options.c_cflag = (options.c_cflag & ~CBAUD) | (B460800 & CBAUD);
	if (filp->f_op->unlocked_ioctl(filp,
				(unsigned int)TCSETS,
				(unsigned long)&options) < 0) {
		filp_close(filp, NULL);
		set_fs(oldfs);
		return -EIO;
	}

	set_fs(oldfs);
	uart_tty_filp  = filp;
	return 0;
}


static void FeliCaUART_tty_close(void)
{
	filp_close(uart_tty_filp, NULL);
	uart_tty_filp = NULL;

}


static int FeliCaUART_tty_write(const char *buff, size_t count)
{
	struct file		*filp;
	int				ret = 0;
	mm_segment_t	oldfs;


	/* parameter check */
	if (NULL == buff) {
		pr_err("FeliCaUART_tty_write parameter error\n");
		return -EIO;
	}

	if (count == 0) {
		pr_err("FeliCaUART_tty_write count 0\n");
		return 0;
	}

	filp = uart_tty_filp;
	filp->f_pos = 0;

	oldfs = get_fs();
	set_fs(KERNEL_DS);

	ret = filp->f_op->write(filp, buff, count, &filp->f_pos);
	set_fs(oldfs);
	if (ret < count) {
		pr_err("FeliCaUART_tty_write write error %d\n", ret);
		return -EIO;
	}

	return ret;
}


static int FeliCaUART_tty_read(char *buff, size_t count)
{
	struct file		*filp;
	int				ret = 0;
	mm_segment_t	oldfs;


	/* parameter check */
	if (NULL == buff) {
		pr_err("FeliCaUART_tty_read parameter error\n");
		return -EIO;
	}
	if (count == 0) {
		pr_err("FeliCaUART_tty_read count 0\n");
		return 0;
	}

	filp = uart_tty_filp;
	filp->f_pos = 0;

	oldfs = get_fs();
	set_fs(KERNEL_DS);

	ret = filp->f_op->read(filp, buff, count, &filp->f_pos);
	set_fs(oldfs);

	if (ret < 0) {
		pr_err("FeliCaUART_tty_read read error %d\n", ret);
		return ret;
	}

	return ret;
}


static int FeliCaUART_tty_fsync(void)
{
	struct file		*filp;
	int				ret = 0;
	mm_segment_t	oldfs;
	int				size;
	int				i = 0;


	filp = uart_tty_filp;
	filp->f_pos = 0;

	oldfs = get_fs();
	set_fs(KERNEL_DS);

	/* output-buffer-size check */
	ret = filp->f_op->unlocked_ioctl(filp,
				(unsigned int)TIOCOUTQ,
				(unsigned long)&size);
	set_fs(oldfs);
	while (size != 0) { /* become zero until continue  */
		oldfs = get_fs();
		set_fs(KERNEL_DS);
		ret = filp->f_op->unlocked_ioctl(filp,
				(unsigned int)TIOCOUTQ,
				(unsigned long)&size);
		set_fs(oldfs);
		msleep(20);
		i++;
		if (i > 1000) { /* gaurd counter */
			pr_err("fysnc-counter-overflow\n");
			break;
		}
	}

	return ret;
}


static int FeliCaUART_tty_available(int *value)
{
	struct file		*filp;
	int				ret = 0;
	mm_segment_t	oldfs;
	int				size;


	filp = uart_tty_filp;
	filp->f_pos = 0;

	oldfs = get_fs();
	set_fs(KERNEL_DS);

	ret = filp->f_op->unlocked_ioctl(filp,
				(unsigned int)FIONREAD,
				(unsigned long)&size);
	set_fs(oldfs);

	if (ret < 0) {
		pr_err("FIONREAD error %d\n", ret);
		size = 0;
	}
	*value = size;

	return ret;
}


static int FeliCaUART_open(struct inode *pinode, struct file  *pfile)
{
	int ret				= 0;
	unsigned char minor	= 0;


	if ((pfile == NULL) || (pinode == NULL))
		return -EINVAL;


	minor = MINOR(pinode->i_rdev);

	if (minor == FELICA_COMM_MINOR_NO_COMM) {
		if (uart_tty_open_count == 0) {
			if (FeliCaUART_tty_open() < 0)
				return -EACCES;

		}
		uart_tty_open_count++;
	} else {
		ret = -ENODEV;
	}

	return ret;
}


static int FeliCaUART_close(struct inode *pinode, struct file *pfile)
{
	int ret				= 0;
	unsigned char minor	= 0;


	if ((pfile == NULL) || (pinode == NULL))
		return -EINVAL;


	minor = MINOR(pinode->i_rdev);

	if (minor == FELICA_COMM_MINOR_NO_COMM) {
		uart_tty_open_count--;
		if (uart_tty_open_count == 0)
			FeliCaUART_tty_close();

	} else {
		ret = -ENODEV;
	}

	return ret;
}


static ssize_t FeliCaUART_write(
						struct file	*pfile,
						const char	*buff,
						size_t		count,
						loff_t		*poff)
{
	ssize_t			ret = 0;
	unsigned char	minor = 0;
	unsigned long	ret_w;
	int				write_ret;


	if ((pfile == NULL) || (buff == NULL))
		return (ssize_t)(-EINVAL);


	minor = MINOR(pfile->f_dentry->d_inode->i_rdev);

	if (FELICA_COMM_MINOR_NO_COMM == minor) {
		if (count == 0) {
			pr_err("count 0\n");
			return 0;
		}

		if (count > UART_TTY_WRITEBUFF_SIZE)
			count = UART_TTY_WRITEBUFF_SIZE;


		ret_w = copy_from_user(uart_tty_writebuff,
				(void *)buff,
				(unsigned int)count);
		if (ret_w == 0) {
			write_ret = FeliCaUART_tty_write(
					(const char *)uart_tty_writebuff,
					count);
			if (write_ret < 0)
				return (ssize_t)(-EFAULT);

			return write_ret;
		} else {
			pr_err("copy_from_user error\n");
			return (ssize_t)(-EFAULT);
		}
	} else {
		ret = (ssize_t)(-ENODEV);
	}

	return ret;
}


static ssize_t FeliCaUART_read(
							struct file	*pfile,
							char		*buff,
							size_t		count,
							loff_t		*poff)
{
	ssize_t				ret = 0;
	unsigned char		minor = 0;


	if ((pfile == NULL) || (buff == NULL))
		return (ssize_t)(-EINVAL);


	minor = MINOR(pfile->f_dentry->d_inode->i_rdev);

	if (FELICA_COMM_MINOR_NO_COMM == minor) {

		if (count > UART_TTY_READBUFF_SIZE)
			count = UART_TTY_READBUFF_SIZE;


		ret = FeliCaUART_tty_read(uart_tty_readbuff, count);
		if (ret < 0)
			return (ssize_t)(-EFAULT);

		if ((copy_to_user((void *)buff,
				(void *)uart_tty_readbuff, ret)) != 0) {
			return (ssize_t)(-EFAULT);
		}
	} else {
		return (ssize_t)(-ENODEV);
	}

	return ret;
}


static long FeliCaUART_ioctl(struct file *pfile,
						unsigned int cmd,
						unsigned long arg)
{
	int				ret = 0;
	int				ret_sub;
	unsigned char	minor = 0;


	if (NULL == pfile)
		return -EINVAL;


	minor = MINOR(pfile->f_dentry->d_inode->i_rdev);

	if (FELICA_COMM_MINOR_NO_COMM == minor) {
			if (cmd == FIONREAD) {
				ret = FeliCaUART_tty_available(&ret_sub);
				if (ret < 0)
					return -EIO;

				if (copy_to_user((void *)arg,
						&ret_sub, sizeof(int)) != 0) {
					return -EFAULT;
				}

			} else {
				ret = -ENOSYS;
			}
	} else {
			ret = -ENODEV;
	}

	return ret;
}


static int FeliCaUART_fsync(struct file *pfile, int datasync)
{
	int				ret = 0;
	unsigned char	minor;


	if (NULL == pfile)
		return -EINVAL;


	minor = MINOR(pfile->f_dentry->d_inode->i_rdev);

	if (FELICA_COMM_MINOR_NO_COMM == minor) {
			ret = FeliCaUART_tty_fsync();
			if (ret < 0)
				ret = (ssize_t)(-EIO);

	} else {
			ret = -ENODEV;
	}

	return ret;
}


static int __init FeliCaUART_init(void)
{
	int ret;


	spin_lock_init(&timer_lock);

	cdev_init(&g_cdev_fuart, &felica_fops);
	g_cdev_fuart.owner = THIS_MODULE;
	ret = cdev_add(&g_cdev_fuart, dev_id_uart, FELICA_DEV_NUM);
	if (ret < 0)
		pr_err("cdev_init(UART) failed.");


	return ret;
}
module_init(FeliCaUART_init);


static void __exit FeliCaUART_exit(void)
{
	cdev_del(&g_cdev_fuart);
	unregister_chrdev_region(dev_id_uart, FELICA_DEV_NUM);

	return;
}
module_exit(FeliCaUART_exit);

MODULE_AUTHOR("NTT DATA MSE Co., Ltd.");
MODULE_DESCRIPTION("FeliCa UART Driver");
MODULE_LICENSE("GPL");
