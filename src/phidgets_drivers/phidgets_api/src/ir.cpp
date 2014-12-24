#include "phidgets_api/ir.h"

namespace phidgets {

IR::IR():
  Phidget(),
  ir_handle_(0)
{
  // create the handle
  CPhidgetIR_create(&ir_handle_);

  // pass handle to base class
  Phidget::init((CPhidgetHandle)ir_handle_);

  // register base class callbacks
  Phidget::registerHandlers();
  
  // register ir data callback
	CPhidgetIR_set_OnCode_Handler(ir_handle_, CodeHandler, this);
}


int IR::CodeHandler(CPhidgetIRHandle ir, void *userptr, unsigned char *data, int dataLength, int bitCount, int repeat)
{
  ((IR*)userptr)->codeHandler(data, dataLength, bitCount, repeat);
  return 0;
}

void IR::codeHandler(unsigned char *data, int dataLength, int bitCount, int repeat)
{
	int i;
	printf("DataLength: %d, Bit Count: %d, Repeat: %d\n", dataLength, bitCount, repeat);
	printf("Code: ");
	for(i = 0; i < dataLength; i++)
	{
		printf("%02x", data[i]); 
	}
	printf("\n");
}

} // namespace phidgets
