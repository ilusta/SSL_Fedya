#pragma once

#include "Arduino.h"
#include "Errors.h"
#include "Updatable.h"

#include "MPU9250_WE.h"

class IMU : public Updatable
{
public:
    IMU(int cs) : imu(&SPI, cs, true) {}
    ERROR_TYPE init()
    {
        imu.init();
        delay(1000);
        imu.autoOffsets();
        
        //imu.setAccOffsets(-14240.0, 18220.0, -17280.0, 15590.0, -20930.0, 12080.0);
        //imu.setGyrOffsets(45.0, 145.0, -105.0);
        imu.enableGyrDLPF();
        //imu.disableGyrDLPF(MPU9250_BW_WO_DLPF_8800); // bandwdith without DLPF
        imu.setGyrDLPF(MPU9250_DLPF_6);
        imu.setSampleRateDivider(5);
        imu.setGyrRange(MPU9250_GYRO_RANGE_250);
        imu.setAccRange(MPU9250_ACC_RANGE_2G);
        imu.enableAccDLPF(true);
        imu.setAccDLPF(MPU9250_DLPF_6);
        //imu.enableAccAxes(MPU9250_ENABLE_XYZ);
        //imu.enableGyrAxes(MPU9250_ENABLE_XYZ);
        imu.setMagOpMode(AK8963_CONT_MODE_100HZ);
        delay(200);
        return NO_ERRORS;
    }
    ERROR_TYPE update() override;

    float getYawRate() { return gyr.z; }
    
    bool isFlat()
    {
        return orientation == MPU9250_FLAT;
    }

protected:
    MPU9250_WE imu;

    xyzFloat gValue;
    xyzFloat gyr;
    xyzFloat magValue;
    MPU9250_orientation orientation;
    float temp;
    float resultantG;
};

ERROR_TYPE IMU::update()
{
  gValue = imu.getGValues();
  gyr = imu.getGyrValues() * M_PI / 180.0;
  magValue = imu.getMagValues();
  temp = imu.getTemperature();
  resultantG = imu.getResultantG(gValue);
  orientation = imu.getOrientation();

  return NO_ERRORS;
}