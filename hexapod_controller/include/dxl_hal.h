#ifndef _DYNAMIXEL_HAL_HEADER
#define _DYNAMIXEL_HAL_HEADER


#ifdef __cplusplus
extern "C" {
#endif


int dxl_hal_open( int devIndex, float baudrate );
void dxl_hal_close();
void dxl_hal_clear();
int dxl_hal_tx( unsigned char *pPacket, int numPacket );
int dxl_hal_rx( unsigned char *pPacket, int numPacket );
void dxl_hal_set_timeout( int NumRcvByte );
int dxl_hal_timeout();



#ifdef __cplusplus
}
#endif

#endif
