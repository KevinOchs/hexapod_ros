#include "phidgets_api/imu.h"

namespace phidgets {

Imu::Imu():
  Phidget(),
  imu_handle_(0)
{
  // create the handle
  CPhidgetSpatial_create(&imu_handle_);

  // pass handle to base class
  Phidget::init((CPhidgetHandle)imu_handle_);

  // register base class callbacks
  Phidget::registerHandlers();
  
  // register imu data callback
	CPhidgetSpatial_set_OnSpatialData_Handler(imu_handle_, SpatialDataHandler, this);
}

void Imu::setDataRate(int rate)
{
	CPhidgetSpatial_setDataRate(imu_handle_, rate);
}

void Imu::zero()
{
  // zero (calibrate) gyro
  CPhidgetSpatial_zeroGyro(imu_handle_);
}

int Imu::SpatialDataHandler(CPhidgetSpatialHandle handle, void *userptr, CPhidgetSpatial_SpatialEventDataHandle *data, int count)
{
  ((Imu*)userptr)->dataHandler(data, count);
  return 0;
}

void Imu::dataHandler(CPhidgetSpatial_SpatialEventDataHandle *data, int count)
{
  printf("Empty data handler\n");
}

} //namespace phidgets

