#include <linux/fs.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/utsname.h>

static int version_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, linux_proc_banner,
		utsname()->sysname,
		utsname()->release,
		utsname()->version);
	return 0;
}

static int version_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, version_proc_show, NULL);
}

static const struct file_operations version_proc_fops = {
	.open		= version_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int __init proc_version_init(void)
{
	proc_create("version", 0, NULL, &version_proc_fops);
	return 0;
}
module_init(proc_version_init);

//BEGIN, MSE, ml-motofelica@nttd-mse.com 11/30/2012 for TOMOYO patch
static int __init ccs_show_version(void)
{
	printk(KERN_INFO "Hook version: 3.0.46 2012/10/13\n");
	return 0;
}
module_init(ccs_show_version);
//END, MSE, ml-motofelica@nttd-mse.com 11/30/2012 for TOMOYO patch
