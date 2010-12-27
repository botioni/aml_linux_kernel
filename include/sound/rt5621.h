/**
 * struct rt5621_platform_data - platform-specific RT5621 data
 * @is_hp_unpluged:          HP Detect
 */

#ifndef _RT5621_H_
#define _RT5621_H_

struct rt5621_platform_data {
    int (*is_hp_pluged)(void);
};

#endif
