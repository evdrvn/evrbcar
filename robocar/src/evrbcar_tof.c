#include <vl53l0x_api.h>
#include <vl53l0x_platform.h>
#include "evrbcar_tof.h"
#include "evrbcar_elog.h"

int evrbcar_tof_init(VL53L0X_Dev_t* tofctx, char* dev, uint8_t addr){
    tofctx->I2cDevAddr = addr;
    tofctx->fd = VL53L0X_i2c_init(dev, tofctx->I2cDevAddr);
    if(tofctx->fd < 0){
        push_event_log("tof init error !!");
        return tofctx->fd;
    }
    return 0;
}
