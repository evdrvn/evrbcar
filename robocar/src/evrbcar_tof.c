#include <vl53l0x_api.h>
#include <vl53l0x_platform.h>
#include "evrbcar_tof.h"
#include "evrbcar_elog.h"

int evrbcar_tof_init(VL53L0X_Dev_t* tofctx, char* dev, uint8_t addr){
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    uint32_t refSpadCount;
    uint8_t isApertureSpads;
    uint8_t VhvSettings;
    uint8_t PhaseCal;

    tofctx->I2cDevAddr = addr;
    tofctx->fd = VL53L0X_i2c_init(dev, tofctx->I2cDevAddr);
    if(tofctx->fd < 0){
        push_event_log("fail to init VL53L0X");
        Status = VL53L0X_ERROR_CONTROL_INTERFACE;
    }
    if(Status == VL53L0X_ERROR_NONE)
    {
        push_event_log("Call of VL53L0X_DataInit");
        Status = VL53L0X_DataInit(tofctx);
    }
    if(Status == VL53L0X_ERROR_NONE)
    {
        push_event_log("Call of VL53L0X_StaticInit");
        Status = VL53L0X_StaticInit(tofctx); 
    }
    if(Status == VL53L0X_ERROR_NONE)
    {
        push_event_log("Call of VL53L0X_PerformRefCalibration");
        Status = VL53L0X_PerformRefCalibration(tofctx,
                &VhvSettings, &PhaseCal);
    }
    if(Status == VL53L0X_ERROR_NONE)
    {
        push_event_log("Call of VL53L0X_PerformRefSpadManagement");
        Status = VL53L0X_PerformRefSpadManagement(tofctx,
                &refSpadCount, &isApertureSpads);
    }
    if(Status == VL53L0X_ERROR_NONE)
    {
        push_event_log("Call of VL53L0X_SetDeviceMode");
        Status = VL53L0X_SetDeviceMode(tofctx, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING); // Setup in single ranging mode
    }
    if (Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_SetLimitCheckEnable(tofctx,
        		VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1);
    }
    if (Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_SetLimitCheckEnable(tofctx,
        		VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
    }
    if (Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_SetLimitCheckValue(tofctx,
        		VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,
        		(FixPoint1616_t)(0.1*65536));
	}			
    if (Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_SetLimitCheckValue(tofctx,
        		VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE,
        		(FixPoint1616_t)(60*65536));			
    }
    if (Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(tofctx,
        		33000);
	}
    if (Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_SetVcselPulsePeriod(tofctx, 
		        VL53L0X_VCSEL_PERIOD_PRE_RANGE, 18);
    }
    if (Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_SetVcselPulsePeriod(tofctx, 
		        VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 14);
    }
    if(Status == VL53L0X_ERROR_NONE)
    {
        push_event_log("Call of VL53L0X_StartMeasurement");
        Status = VL53L0X_StartMeasurement(tofctx);
    }

    return (int)Status;
}

int evrbcar_tof_measure(VL53L0X_Dev_t *tofctx, uint32_t *range){
    VL53L0X_RangingMeasurementData_t    RangingMeasurementData;
    VL53L0X_RangingMeasurementData_t   *pRangingMeasurementData    = &RangingMeasurementData;
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;

    Status = VL53L0X_GetRangingMeasurementData(tofctx, pRangingMeasurementData);
    *range = pRangingMeasurementData->RangeMilliMeter;
    VL53L0X_ClearInterruptMask(tofctx, VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY);

    return (int)Status;
}

VL53L0X_Error WaitStopCompleted(VL53L0X_DEV Dev) {
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    uint32_t StopCompleted=0;
    uint32_t LoopNb;

    // Wait until it finished
    // use timeout to avoid deadlock
    if (Status == VL53L0X_ERROR_NONE) {
        LoopNb = 0;
        do {
            Status = VL53L0X_GetStopCompletedStatus(Dev, &StopCompleted);
            if ((StopCompleted == 0x00) || Status != VL53L0X_ERROR_NONE) {
                break;
            }
            LoopNb = LoopNb + 1;
            VL53L0X_PollingDelay(Dev);
        } while (LoopNb < VL53L0X_DEFAULT_MAX_LOOP);

        if (LoopNb >= VL53L0X_DEFAULT_MAX_LOOP) {
            Status = VL53L0X_ERROR_TIME_OUT;
        }
	
    }

    return Status;
}

int evrbcar_tof_destroy(VL53L0X_Dev_t* tofctx){
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    
    if(Status == VL53L0X_ERROR_NONE)
    {
        push_event_log("Call of VL53L0X_StopMeasurement");
        Status = VL53L0X_StopMeasurement(tofctx);
    }
    if(Status == VL53L0X_ERROR_NONE)
    {
        push_event_log("Wait Stop to be competed");
        Status = WaitStopCompleted(tofctx);
    }
    if(Status == VL53L0X_ERROR_NONE)
	Status = VL53L0X_ClearInterruptMask(tofctx,
		VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY);

    return (int)Status;
}

