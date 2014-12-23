#ifndef _DYNAMIXEL_HEADER
#define _DYNAMIXEL_HEADER


#ifdef __cplusplus
extern "C" {
#endif


///////////// device control methods ////////////////////////
int dxl_initialize( int deviceIndex, int baudnum );
void dxl_terminate();


///////////// set/get packet methods //////////////////////////
#define MAXNUM_TXPARAM      (150)
#define MAXNUM_RXPARAM      (225)

void dxl_set_txpacket_id( int id );
#define BROADCAST_ID        (254)

void dxl_set_txpacket_instruction( int instruction );
#define INST_PING           (1)
#define INST_READ           (2)
#define INST_WRITE          (3)
#define INST_REG_WRITE      (4)
#define INST_ACTION         (5)
#define INST_RESET          (6)
#define INST_SYNC_WRITE     (131)
#define INST_SYNC_READ      (132)

void dxl_set_txpacket_parameter( int index, int value );
void dxl_set_txpacket_length( int length );

int dxl_get_rxpacket_error( int errbit );
#define ERRBIT_VOLTAGE      (1)
#define ERRBIT_ANGLE        (2)
#define ERRBIT_OVERHEAT     (4)
#define ERRBIT_RANGE        (8)
#define ERRBIT_CHECKSUM     (16)
#define ERRBIT_OVERLOAD     (32)
#define ERRBIT_INSTRUCTION  (64)

int dxl_get_rxpacket_length( void );
int dxl_get_rxpacket_parameter( int index );


// utility for value
int dxl_makeword( int lowbyte, int highbyte );
int dxl_get_lowbyte( int word );
int dxl_get_highbyte( int word );


////////// packet communication methods ///////////////////////
void dxl_tx_packet( void );
void dxl_rx_packet( void );
void dxl_txrx_packet( void );

int dxl_get_result( void );
#define COMM_TXSUCCESS      (0)
#define COMM_RXSUCCESS      (1)
#define COMM_TXFAIL     (2)
#define COMM_RXFAIL     (3)
#define COMM_TXERROR        (4)
#define COMM_RXWAITING      (5)
#define COMM_RXTIMEOUT      (6)
#define COMM_RXCORRUPT      (7)


//////////// high communication methods ///////////////////////
void dxl_ping( int id );
int dxl_read_byte( int id, int address );
void dxl_write_byte( int id, int address, int value );
int dxl_read_word( int id, int address );
void dxl_write_word( int id, int address, int value );

//////////// Synchroneous communication methods ///////////////////////
void dxl_sync_write_start( int address, int data_length );
void dxl_sync_write_push_id( int id );
void dxl_sync_write_push_byte( int value );
void dxl_sync_write_push_word( int value );
void dxl_sync_write_send();

void dxl_sync_read_start( int address, int data_length );
void dxl_sync_read_push_id( int id );
void dxl_sync_read_send();
//void dxl_sync_read_noblock_send();
//void dxl_sync_read_noblock_receive();
int dxl_sync_read_pop_byte();
int dxl_sync_read_pop_word();


#ifdef __cplusplus
}
#endif

#endif
