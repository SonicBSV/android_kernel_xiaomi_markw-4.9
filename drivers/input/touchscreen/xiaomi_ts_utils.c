#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <linux/semaphore.h>

#define PROC_BASE_DIR	"gesture"

bool xiaomi_ts_probed = false;
EXPORT_SYMBOL(xiaomi_ts_probed);

DEFINE_SEMAPHORE(xiaomi_ts_probe_sem);
EXPORT_SYMBOL(xiaomi_ts_probe_sem);

static int gesture_on_off = 0;
static void *gesture_on_off_priv = NULL;
static void (*gesture_on_off_callback)(void*, int) = NULL;


static ssize_t proc_gesture_onoff_read(struct file *file,
					char __user *page,
					size_t count, loff_t *ppos)
{
	int num;

	if (*ppos)
		return 0;

	num = sprintf(page, "%d\n", gesture_on_off);
	*ppos += num;
	return num;
}

static ssize_t proc_gesture_onoff_write(struct file *file,
					const char __user *buffer,
					size_t count, loff_t *ppos)
{
	char buf[10];

	if (copy_from_user(buf, buffer, sizeof(buf)))
		return -EFAULT;

	sscanf(buf, "%d", &gesture_on_off);
	if (gesture_on_off_callback != NULL)
		gesture_on_off_callback(gesture_on_off_priv, gesture_on_off);

	return count;
}

static const struct file_operations gesture_onoff_proc_fops = {
	.write = proc_gesture_onoff_write,
	.read = proc_gesture_onoff_read,
	.open = simple_open,
	.owner = THIS_MODULE,
};

static int gesture_on_off_proc_setup(void)
{
	struct proc_dir_entry	*dir = NULL;
	struct proc_dir_entry	*onoff = NULL;

	dir = proc_mkdir(PROC_BASE_DIR, NULL);
	if (!dir)
		return -ENOMEM;

	onoff = proc_create("onoff", 0660, dir, &gesture_onoff_proc_fops);
	if (onoff == NULL){
		remove_proc_subtree(PROC_BASE_DIR, NULL);
		return -ENOMEM;
	}

	return 0;
}

void xiaomi_ts_set_gesture_on_off_callback(void (*callback)(void*, int), void *priv)
{
	if (callback == NULL)
		priv = NULL;

	gesture_on_off_callback = callback;
	gesture_on_off_priv = priv;

	if (gesture_on_off_callback != NULL)
		gesture_on_off_callback(gesture_on_off_priv, gesture_on_off);
}
EXPORT_SYMBOL(xiaomi_ts_set_gesture_on_off_callback);

static int __init xiaomi_ts_util_init(void)
{
	return gesture_on_off_proc_setup();
}

static void __exit xiaomi_ts_util_exit(void)
{
	remove_proc_subtree(PROC_BASE_DIR, NULL);
}

module_init(xiaomi_ts_util_init);
module_exit(xiaomi_ts_util_exit);

MODULE_DESCRIPTION("Xiaomi ts utils");
MODULE_LICENSE("GPL v2");
