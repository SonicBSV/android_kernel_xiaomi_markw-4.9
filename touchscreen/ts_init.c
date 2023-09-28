#include <xiaomi/touchscreen.h>

/*
#include <xiaomi/touchscreen.h>
	if (xiaomi_touchscreen_is_probed)
		return -ENODEV;
	xiaomi_touchscreen_is_probed = true;
*/
bool xiaomi_touchscreen_is_probed = false;
EXPORT_SYMBOL(xiaomi_touchscreen_is_probed);
