#ifndef __EVRBCAR_IMU_H__
#define __EVRBCAR_IMU_H__

extern int evrbcar_imu_init(bno055_conn_t* conn, const char* i2cdev, uint8_t address);
extern int evrbcar_imu_destroy(bno055_conn_t* conn);
extern int evrbcar_imu_measure(bno055_conn_t* conn, double *result);

#endif//__EVRBCAR_IMU_H__


