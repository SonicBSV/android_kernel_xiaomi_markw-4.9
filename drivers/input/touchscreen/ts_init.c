#include <linux/module.h>
#include <xiaomi/touchscreen.h>

/*
#include <xiaomi/touchscreen.h>

	if (xiaomi_touchscreen_is_probed)
		return -ENODEV;

	xiaomi_touchscreen_is_probed = true;
*/
bool xiaomi_touchscreen_is_probed = false;
EXPORT_SYMBOL(xiaomi_touchscreen_is_probed);

#if IS_ENABLED(CONFIG_TOUCHSCREEN_ATMEL308U)
extern int xiaomi_touchscreen_mxt_init(void);
#endif
#if IS_ENABLED(CONFIG_TOUCHSCREEN_FTS)
extern int xiaomi_touchscreen_fts_ts_init(void);
#endif

static int __init xiaomi_touchscreen_init(void)
{
#if IS_ENABLED(CONFIG_TOUCHSCREEN_ATMEL308U)
	xiaomi_touchscreen_mxt_init();
#endif
#if IS_ENABLED(CONFIG_TOUCHSCREEN_FTS)
	xiaomi_touchscreen_fts_ts_init();
#endif

	return 0;
}
module_init(xiaomi_touchscreen_init);
