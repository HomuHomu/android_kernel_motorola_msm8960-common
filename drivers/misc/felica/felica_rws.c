/*
 * MSE, ml-motofelica@nttd-mse.com
 * 04/27/2012 First Release of FeliCa device driver
 * 05/29/2012 Update FeliCa driver
 * 06/05/2012 Fix of pointing out checkpatch.pl
 *
 * Copyright(C) 2012 NTT DATA MSE CORPORATION. All right reserved.
 *
 * @file felica_rws.c
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
#include <linux/fcntl.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include "felica_rws.h"

/*
 * static paramater & global
 */
static struct cdev g_cdev_frws;
static unsigned char g_rws_info = FELICA_OUT_RWS_AVAILABLE;

/*
 * Function prottype
 */
static int  __init FeliCaRWS_init(void);
static void __exit FeliCaRWS_exit(void);
static int  FeliCaRWS_open(struct inode *, struct file *);
static int  FeliCaRWS_close(struct inode *, struct file *);
static ssize_t FeliCaRWS_read(struct file *, char *, size_t, loff_t *);

/*
 * sturut of reginsting External if
 */
static const struct file_operations felica_fops = {
	.owner   = THIS_MODULE,
	.read    = FeliCaRWS_read,
	.open    = FeliCaRWS_open,
	.release = FeliCaRWS_close
};







static int __init FeliCaRWS_init(void)
{
	int ret;

	g_rws_info = FELICA_OUT_RWS_AVAILABLE;

	cdev_init(&g_cdev_frws, &felica_fops);
	g_cdev_frws.owner = THIS_MODULE;
	ret = cdev_add(
			&g_cdev_frws,
			dev_id_rws,
			FELICA_DEV_NUM);
	if (ret < 0)
		pr_err("cdev_init(RWS) failed.");


	return ret;
}
module_init(FeliCaRWS_init);


static void __exit FeliCaRWS_exit(void)
{
	cdev_del(&g_cdev_frws);
	unregister_chrdev_region(dev_id_rws, FELICA_DEV_NUM);
	return;
}
module_exit(FeliCaRWS_exit);


static int FeliCaRWS_open(struct inode *pinode, struct file *pfile)
{
	int ret				= 0;
	unsigned char minor	= 0;

	if ((pfile == NULL) || (pinode == NULL))
		return -EINVAL;


	minor = MINOR(pinode->i_rdev);

	if (minor != FELICA_RWS_MINOR_NO_RWS)
		ret = -ENODEV;


	return ret;
}


static int FeliCaRWS_close(struct inode *pinode, struct file *pfile)
{
	int ret				= 0;
	unsigned char minor	= 0;

	if ((pfile == NULL) || (pinode == NULL))
		return -EINVAL;


	minor = MINOR(pinode->i_rdev);

	if (minor != FELICA_RWS_MINOR_NO_RWS)
		ret = -ENODEV;


	return ret;
}


static ssize_t FeliCaRWS_read(
							struct file *pfile,
							char		*buff,
							size_t		count,
							loff_t		*poff)
{
	ssize_t			ret = 0;
	unsigned char	minor = 0;
	unsigned char	local_buff[FELICA_OUT_RWS_SIZE];
	unsigned long	ret_cpy = 0;

	if ((pfile == NULL) || (buff == NULL))
		return (ssize_t)(-EINVAL);


	minor = MINOR(pfile->f_dentry->d_inode->i_rdev);

	if (FELICA_RWS_MINOR_NO_RWS == minor) {
		if (count != FELICA_OUT_RWS_SIZE)
			return (ssize_t)(-EINVAL);

		local_buff[0] = g_rws_info;
		ret_cpy = copy_to_user(buff, local_buff, FELICA_OUT_RWS_SIZE);
		if (ret_cpy != 0)
			return (ssize_t)(-EFAULT);
		else
			ret = FELICA_OUT_RWS_SIZE;

	} else {
		ret = (ssize_t)(-ENODEV);
	}

	return ret;
}

MODULE_AUTHOR("NTT DATA MSE Co., Ltd.");
MODULE_DESCRIPTION("FeliCa RWS Driver");
MODULE_LICENSE("GPL");
