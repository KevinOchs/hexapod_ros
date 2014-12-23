#include "phidgets_api/phidget.h"

namespace phidgets {

Phidget::Phidget()
{

}

Phidget::~Phidget()
{
  //close(); // segfaults, why?
  CPhidget_delete(handle_);
}

void Phidget::registerHandlers()
{
  CPhidget_set_OnAttach_Handler(handle_, &Phidget::AttachHandler, this); 
  CPhidget_set_OnDetach_Handler(handle_, &Phidget::DetachHandler, this); 
	CPhidget_set_OnError_Handler (handle_, &Phidget::ErrorHandler,  this);
}

void Phidget::init(CPhidgetHandle handle)
{
  handle_ = handle;
}

int Phidget::open(int serial_number)
{
  return CPhidget_open(handle_, serial_number);
}

int Phidget::close()
{
  return CPhidget_close(handle_);
}

int Phidget::waitForAttachment(int timeout)
{
  return CPhidget_waitForAttachment(handle_, timeout);
}

std::string Phidget::getDeviceType()
{
  char a[1000];
  const char * deviceptr = a;
  CPhidget_getDeviceType(handle_, &deviceptr);
  return std::string(deviceptr);
}

std::string Phidget::getDeviceName()
{
  char a[1000];
  const char * deviceptr = a;
  CPhidget_getDeviceName(handle_, &deviceptr);
  return std::string(deviceptr);
}

std::string Phidget::getDeviceLabel()
{
  char a[1000];
  const char * deviceptr = a;
  CPhidget_getDeviceType(handle_, &deviceptr);
  return std::string(deviceptr);
}

std::string Phidget::getLibraryVersion(){
  char a[1000];
  const char * deviceptr = a;
  CPhidget_getLibraryVersion(&deviceptr);
  return std::string(deviceptr);
}

int Phidget::getDeviceSerialNumber()
{
  int sernum;
  CPhidget_getSerialNumber(handle_, &sernum);
  return sernum;
}

int Phidget::getDeviceVersion()
{
  int version;
  CPhidget_getDeviceVersion(handle_, &version);
  return version;
}

std::string Phidget::getErrorDescription(int errorCode)
{
  char a[1000];
  const char * errorPtr = a;
  CPhidget_getErrorDescription(errorCode, &errorPtr);
  return std::string(errorPtr);
}

void Phidget::attachHandler()
{
	printf("Phidget attached (serial# %d)\n", getDeviceSerialNumber());
}

void Phidget::detachHandler()
{
	printf("Phidget detached (serial# %d)\n", getDeviceSerialNumber());
}

void Phidget::errorHandler(int error)
{
	printf("Phidget error [%d]: %s\n", error, getErrorDescription(error).c_str());
}

int Phidget::AttachHandler(CPhidgetHandle handle, void *userptr)
{
  ((Phidget*)userptr)->attachHandler();
  return 0;
}

int Phidget::DetachHandler(CPhidgetHandle handle, void *userptr)
{
  ((Phidget*)userptr)->detachHandler();
  return 0;
}

int Phidget::ErrorHandler(CPhidgetHandle handle, void *userptr, int ErrorCode, const char *unknown)
{
  ((Phidget*)userptr)->errorHandler(ErrorCode);
  return 0;
}

} //namespace phidgets
