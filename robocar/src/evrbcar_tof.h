#ifndef __EVRBCAR_TOF_H__
#define __EVRBCAR_TOF_H__

#include <vl53l0x_api.h>
#include <vl53l0x_platform.h>

#define VERSION_REQUIRED_MAJOR 1
#define VERSION_REQUIRED_MINOR 0
#define VERSION_REQUIRED_BUILD 1

int evrbcar_tof_init(VL53L0X_Dev_t* tofctx, char* dev, uint8_t addr);

#endif//__EVRBCAR_TOF_H__

