#include <stdio.h>
#include <stdlib.h>
#include <bno055-i2c.h>
#include "evrbcar_elog.h"

#define EVRBCAR_IMU_CALIBDAT_FILENAME "imucalib.dat"
#define EVRBCAR_IMU_CALIBDAT_MASK (char)(0b00111111)
#define EVRBCAR_IMU_CALIBDAT_SIZE (MAG_RADIUS_MSB_ADDR - ACCEL_OFFSET_X_LSB_ADDR + 1) 
int evrbcar_imu_init(bno055_conn_t* conn, const char* i2cdev, uint8_t address){
    int ret = 0;
    int i = 0;
    int j = 0;
    bno055_param_t params[64];
    uint8_t calibstat = 0;
    uint8_t calibdat[EVRBCAR_IMU_CALIBDAT_SIZE];
    FILE* fp;

    bno055_open(conn, i2cdev, address, 2);
    if(0 < ret) {
        push_event_log("imu open error!! %d", ret);
        return ret;
    }

    i = 0;
    j = 0;
    params[i].reg = BNO055_AXIS_MAP_CONFIG_ADDR;
    params[i++].value = 0x24;
    //params[i++].value = 0x21;
    params[i].reg = BNO055_AXIS_MAP_SIGN_ADDR;
    params[i++].value = 0x00;
    //params[1++].value = 0x01;
    
    fp = fopen(EVRBCAR_IMU_CALIBDAT_FILENAME, "rb" );
    if(fp != NULL){
        fread(calibdat, sizeof(calibdat[0]), EVRBCAR_IMU_CALIBDAT_SIZE, fp);  
        fclose(fp);
        for(j = 0; j < EVRBCAR_IMU_CALIBDAT_SIZE; j++){
            params[i + j].reg = ACCEL_OFFSET_X_LSB_ADDR + j; 
            params[i + j].value = calibdat[j]; 
        }
    }
    
    if(0 > bno055_init(conn, 0x81, params, i + j)) push_event_log("imu init error");

    while(ret >= 0 && calibstat != 0xFF){
        push_event_log("calibrating... calibstat = 0x%x", calibstat);
        ret = bno055_readcalibstat(conn, &calibstat);
        sleep(1);
    }
    if(calibstat != 0xFF) push_event_log("imu calibration error !! ret = %d, calibstat = 0x%x", ret, calibstat);
    else push_event_log("imu calibration completed !! ret = %d, calibstat = 0x%x", ret, calibstat);

    bno055_readbytes(conn, ACCEL_OFFSET_X_LSB_ADDR, calibdat, sizeof(calibdat));

    fp = fopen(EVRBCAR_IMU_CALIBDAT_FILENAME, "wb" );
    if(fp == NULL){
        push_event_log("imu clibdat file open error!!");
        return -1;
    }

    fwrite(calibdat, 1, sizeof(calibdat), fp);

    if(fclose(fp) == EOF){
        push_event_log("imu clibdat file close error!!");
        return -1;
    }

    return ret;
}