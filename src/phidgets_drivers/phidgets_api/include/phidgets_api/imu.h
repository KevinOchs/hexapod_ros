#ifndef PHIDGETS_API_IMU_H
#define PHIDGETS_API_IMU_H

#include "phidgets_api/phidget.h"

namespace phidgets {

class Imu: public Phidget
{
  public:

    Imu();

  protected:
 
    CPhidgetSpatialHandle imu_handle_;
    
    void zero();
    void setDataRate(int rate);

    virtual void dataHandler(CPhidgetSpatial_SpatialEventDataHandle *data, int count);

  private:

    static int SpatialDataHandler(CPhidgetSpatialHandle spatial, void *userptr, CPhidgetSpatial_SpatialEventDataHandle *data, int count);
};

} //namespace phidgets

#endif // PHIDGETS_API_IMU_H
