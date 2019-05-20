#include <bno055-i2c.h>
#include "evrbcar_elog.h"

int evrbcar_imu_init(bno055_conn_t* conn, const char* i2cdev, uint8_t address){
    int ret = 0;
    
    uint8_t calibstat = 0;

    bno055_open(conn, i2cdev, address, 2);
    while(0 == bno055_readcalibstat(conn, &calibstat)){

    }

    return ret;
}
