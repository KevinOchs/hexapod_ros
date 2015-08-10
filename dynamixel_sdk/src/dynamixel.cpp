#include <dxl_hal.h>
#include <dynamixel.h>

#define ID					(2)
#define LENGTH				(3)
#define INSTRUCTION			(4)
#define ERRBIT				(4)
#define PARAMETER			(5)
#define DEFAULT_BAUDNUMBER	(1)

unsigned char gbInstructionPacket[MAXNUM_TXPARAM+10] = {0};
unsigned char gbStatusPacket[MAXNUM_RXPARAM+10] = {0};
unsigned char gbRxPacketLength = 0;
unsigned char gbRxGetLength = 0;
int gbCommStatus = COMM_RXSUCCESS;
int giBusUsing = 0;


int dxl_initialize( int devIndex, int baudnum )
{
	float baudrate;	
	baudrate = 2000000.0f / (float)(baudnum + 1);
	
	if( dxl_hal_open(devIndex, baudrate) == 0 )
		return 0;

	gbCommStatus = COMM_RXSUCCESS;
	giBusUsing = 0;
	return 1;
}

void dxl_terminate()
{
	dxl_hal_close();
}

void dxl_tx_packet()
{
	unsigned char i;
	unsigned char TxNumByte, RealTxNumByte;
	unsigned char checksum = 0;

	if( giBusUsing == 1 )
		return;
	
	giBusUsing = 1;

	if( gbInstructionPacket[LENGTH] > (MAXNUM_TXPARAM+2) )
	{
		gbCommStatus = COMM_TXERROR;
		giBusUsing = 0;
		return;
	}
	
	if( gbInstructionPacket[INSTRUCTION] != INST_PING
		&& gbInstructionPacket[INSTRUCTION] != INST_READ
		&& gbInstructionPacket[INSTRUCTION] != INST_WRITE
		&& gbInstructionPacket[INSTRUCTION] != INST_REG_WRITE
		&& gbInstructionPacket[INSTRUCTION] != INST_ACTION
		&& gbInstructionPacket[INSTRUCTION] != INST_RESET
		&& gbInstructionPacket[INSTRUCTION] != INST_SYNC_WRITE
		&& gbInstructionPacket[INSTRUCTION] != INST_SYNC_READ)
	{
		gbCommStatus = COMM_TXERROR;
		giBusUsing = 0;
		return;
	}
	
	gbInstructionPacket[0] = 0xff;
	gbInstructionPacket[1] = 0xff;
	for( i=0; i<(gbInstructionPacket[LENGTH]+1); i++ )
		checksum += gbInstructionPacket[i+2];
	gbInstructionPacket[gbInstructionPacket[LENGTH]+3] = ~checksum;
	
	if( gbCommStatus == COMM_RXTIMEOUT || gbCommStatus == COMM_RXCORRUPT )
		dxl_hal_clear();

	TxNumByte = gbInstructionPacket[LENGTH] + 4;
	RealTxNumByte = dxl_hal_tx( (unsigned char*)gbInstructionPacket, TxNumByte );

	if( TxNumByte != RealTxNumByte )
	{
		gbCommStatus = COMM_TXFAIL;
		giBusUsing = 0;
		return;
	}

	if( gbInstructionPacket[INSTRUCTION] == INST_READ )
		dxl_hal_set_timeout( gbInstructionPacket[PARAMETER+1] + 24 );
	else if ( gbInstructionPacket[INSTRUCTION] == INST_SYNC_READ )
        dxl_hal_set_timeout( gbInstructionPacket[PARAMETER+1] + 200 );
    else
		dxl_hal_set_timeout( 24 );

	gbCommStatus = COMM_TXSUCCESS;
}

void dxl_rx_packet()
{
	unsigned char i, j, nRead;
	unsigned char checksum = 0;

	if( giBusUsing == 0 )
		return;

	if( gbInstructionPacket[ID] == BROADCAST_ID )
	{
		gbCommStatus = COMM_RXSUCCESS;
		giBusUsing = 0;
		return;
	}
	
	if( gbCommStatus == COMM_TXSUCCESS )
	{
		gbRxGetLength = 0;
		gbRxPacketLength = 6;
	}
	
	nRead = dxl_hal_rx( (unsigned char*)&gbStatusPacket[gbRxGetLength], gbRxPacketLength - gbRxGetLength );
	gbRxGetLength += nRead;
	if( gbRxGetLength < gbRxPacketLength )
	{
		if( dxl_hal_timeout() == 1 )
		{
			if(gbRxGetLength == 0)
				gbCommStatus = COMM_RXTIMEOUT;
			else
				gbCommStatus = COMM_RXCORRUPT;
			giBusUsing = 0;
			return;
		}
	}
	
	// Find packet header
	for( i=0; i<(gbRxGetLength-1); i++ )
	{
		if( gbStatusPacket[i] == 0xff && gbStatusPacket[i+1] == 0xff )
		{
			break;
		}
		else if( i == gbRxGetLength-2 && gbStatusPacket[gbRxGetLength-1] == 0xff )
		{
			break;
		}
	}	
	if( i > 0 )
	{
		for( j=0; j<(gbRxGetLength-i); j++ )
			gbStatusPacket[j] = gbStatusPacket[j + i];
			
		gbRxGetLength -= i;		
	}

	if( gbRxGetLength < gbRxPacketLength )
	{
		gbCommStatus = COMM_RXWAITING;
		return;
	}

	// Check id pairing
	if( gbInstructionPacket[ID] != gbStatusPacket[ID])
	{
		gbCommStatus = COMM_RXCORRUPT;
		giBusUsing = 0;
		return;
	}
	
	gbRxPacketLength = gbStatusPacket[LENGTH] + 4;
	if( gbRxGetLength < gbRxPacketLength )
	{
		nRead = dxl_hal_rx( (unsigned char*)&gbStatusPacket[gbRxGetLength], gbRxPacketLength - gbRxGetLength );
		gbRxGetLength += nRead;
		if( gbRxGetLength < gbRxPacketLength )
		{
			gbCommStatus = COMM_RXWAITING;
			return;
		}
	}

	// Check checksum
	for( i=0; i<(gbStatusPacket[LENGTH]+1); i++ )
		checksum += gbStatusPacket[i+2];
	checksum = ~checksum;

	if( gbStatusPacket[gbStatusPacket[LENGTH]+3] != checksum )
	{
		gbCommStatus = COMM_RXCORRUPT;
		giBusUsing = 0;
		return;
	}
	
	gbCommStatus = COMM_RXSUCCESS;
	giBusUsing = 0;
}

void dxl_txrx_packet()
{
	dxl_tx_packet();

	if( gbCommStatus != COMM_TXSUCCESS )
		return;	
	
	do{
		dxl_rx_packet();		
	}while( gbCommStatus == COMM_RXWAITING );	
}

int dxl_get_result()
{
	return gbCommStatus;
}

void dxl_set_txpacket_id( int id )
{
	gbInstructionPacket[ID] = (unsigned char)id;
}

void dxl_set_txpacket_instruction( int instruction )
{
	gbInstructionPacket[INSTRUCTION] = (unsigned char)instruction;
}

void dxl_set_txpacket_parameter( int index, int value )
{
	gbInstructionPacket[PARAMETER+index] = (unsigned char)value;
}

void dxl_set_txpacket_length( int length )
{
	gbInstructionPacket[LENGTH] = (unsigned char)length;
}

int dxl_get_rxpacket_error( int errbit )
{
	if( gbStatusPacket[ERRBIT] & (unsigned char)errbit )
		return 1;

	return 0;
}

int dxl_get_rxpacket_length()
{
	return (int)gbStatusPacket[LENGTH];
}

int dxl_get_rxpacket_parameter( int index )
{
	return (int)gbStatusPacket[PARAMETER+index];
}

int dxl_makeword( int lowbyte, int highbyte )
{
	unsigned short word;

	word = highbyte;
	word = word << 8;
	word = word + lowbyte;
	return (int)word;
}

int dxl_get_lowbyte( int word )
{
	unsigned short temp;

	temp = word & 0xff;
	return (int)temp;
}

int dxl_get_highbyte( int word )
{
	unsigned short temp;

	temp = word & 0xff00;
	temp = temp >> 8;
	return (int)temp;
}

void dxl_ping( int id )
{
	while(giBusUsing);

	gbInstructionPacket[ID] = (unsigned char)id;
	gbInstructionPacket[INSTRUCTION] = INST_PING;
	gbInstructionPacket[LENGTH] = 2;
	
	dxl_txrx_packet();
}

int dxl_read_byte( int id, int address )
{
	while(giBusUsing);

	gbInstructionPacket[ID] = (unsigned char)id;
	gbInstructionPacket[INSTRUCTION] = INST_READ;
	gbInstructionPacket[PARAMETER] = (unsigned char)address;
	gbInstructionPacket[PARAMETER+1] = 1;
	gbInstructionPacket[LENGTH] = 4;
	
	dxl_txrx_packet();

	return (int)gbStatusPacket[PARAMETER];
}

void dxl_write_byte( int id, int address, int value )
{
	while(giBusUsing);

	gbInstructionPacket[ID] = (unsigned char)id;
	gbInstructionPacket[INSTRUCTION] = INST_WRITE;
	gbInstructionPacket[PARAMETER] = (unsigned char)address;
	gbInstructionPacket[PARAMETER+1] = (unsigned char)value;
	gbInstructionPacket[LENGTH] = 4;
	
	dxl_txrx_packet();
}

int dxl_read_word( int id, int address )
{
	while(giBusUsing);

	gbInstructionPacket[ID] = (unsigned char)id;
	gbInstructionPacket[INSTRUCTION] = INST_READ;
	gbInstructionPacket[PARAMETER] = (unsigned char)address;
	gbInstructionPacket[PARAMETER+1] = 2;
	gbInstructionPacket[LENGTH] = 4;
	
	dxl_txrx_packet();

	return dxl_makeword((int)gbStatusPacket[PARAMETER], (int)gbStatusPacket[PARAMETER+1]);
}

void dxl_write_word( int id, int address, int value )
{
	while(giBusUsing);

	gbInstructionPacket[ID] = (unsigned char)id;
	gbInstructionPacket[INSTRUCTION] = INST_WRITE;
	gbInstructionPacket[PARAMETER] = (unsigned char)address;
	gbInstructionPacket[PARAMETER+1] = (unsigned char)dxl_get_lowbyte(value);
	gbInstructionPacket[PARAMETER+2] = (unsigned char)dxl_get_highbyte(value);
	gbInstructionPacket[LENGTH] = 5;
	
	dxl_txrx_packet();
}


unsigned char gbSyncNbParam;

void dxl_sync_write_start( int address, int data_length )
{
	while(giBusUsing); // needs to be done before touching the TX buffer as it is used until the end of RX.
	
	gbInstructionPacket[ID] = BROADCAST_ID; // use the device ID of the USB2AX instead of the broadcast ID to avoid some modifications to the RX code. 
	gbInstructionPacket[INSTRUCTION] = INST_SYNC_WRITE;
	gbInstructionPacket[PARAMETER] = (unsigned char)address;
	gbInstructionPacket[PARAMETER+1] = (unsigned char)data_length;
	gbSyncNbParam = 2;
}

void dxl_sync_write_push_id( int id )
{
    while(giBusUsing); // needs to be done before touching the TX buffer as it is used until the end of RX.	
	
    if ( gbSyncNbParam > MAXNUM_TXPARAM )
    {
        return;
    }
	
    gbInstructionPacket[PARAMETER+gbSyncNbParam++] = (unsigned char)id;
}

void dxl_sync_write_push_byte( int value )
{
    while(giBusUsing); // needs to be done before touching the TX buffer as it is used until the end of RX.	
	
    if ( gbSyncNbParam > MAXNUM_TXPARAM )
    {
       return;
    }
	
    gbInstructionPacket[PARAMETER+gbSyncNbParam++] = (unsigned char)value;
}

void dxl_sync_write_push_word( int value )
{
    while(giBusUsing); // needs to be done before touching the TX buffer as it is used until the end of RX.	
	
    if ( gbSyncNbParam > MAXNUM_TXPARAM )
    {
        return;
    }
	
    gbInstructionPacket[PARAMETER+gbSyncNbParam++] = (unsigned char)dxl_get_lowbyte(value);
	gbInstructionPacket[PARAMETER+gbSyncNbParam++] = (unsigned char)dxl_get_highbyte(value);
}

void dxl_sync_write_send()
{
	while(giBusUsing); // needs to be done before touching the TX buffer as it is used until the end of RX.	
	
    gbInstructionPacket[LENGTH] = gbSyncNbParam + 2;
    
	dxl_txrx_packet();
}


void dxl_sync_read_start( int address, int data_length )
{
    while(giBusUsing); // needs to be done before touching the TX buffer as it is used until the end of RX.	
	
    gbInstructionPacket[ID] = 0XFD; // use the device ID of the USB2AX instead of the broadcast ID to avoid some modifications to the rx code. 
	gbInstructionPacket[INSTRUCTION] = INST_SYNC_READ;
	gbInstructionPacket[PARAMETER] = (unsigned char)address;
	gbInstructionPacket[PARAMETER+1] = (unsigned char)data_length;
	gbSyncNbParam = 2;
}

void dxl_sync_read_push_id( int id )
{
    while(giBusUsing); // needs to be done before touching the TX buffer as it is used until the end of RX.	
	
    if ( gbSyncNbParam > MAXNUM_TXPARAM )
    {
        return;
    }
	
    gbInstructionPacket[PARAMETER+gbSyncNbParam++] = (unsigned char)id;
}

void dxl_sync_read_send()
{
    while(giBusUsing); // needs to be done before touching the TX buffer as it is used until the end of RX.	
	
    gbInstructionPacket[LENGTH] = gbSyncNbParam + 2;
    gbSyncNbParam = 0;

	dxl_txrx_packet();
}

//// you will need to make a noblock_receive before anything else, because it will not allow any other command before it is done.
//void dxl_sync_read_noblock_send(){
//    while(giBusUsing); // needs to be done before touching the TX buffer as it is used until the end of RX.	
//	
//    gbInstructionPacket[LENGTH] = gbSyncNbParam + 2;
//    gbSyncNbParam = 0;
//    
//    dxl_tx_packet();
//
//	if( gbCommStatus != COMM_TXSUCCESS )
//		return;	
//	
//	do{
//		dxl_rx_packet();		
//	}while( gbCommStatus == COMM_RXWAITING );	
//
//    
//}
//
//void dxl_sync_read_noblock_receive(){
//    TODO
//}


int dxl_sync_read_pop_byte()
{
    while(giBusUsing); // needs to be done before touching the TX buffer as it is used until the end of RX.	
	
    if ( gbSyncNbParam >= gbStatusPacket[LENGTH] - 2  )
    {
        return -1;
    }
    
    return (int)gbStatusPacket[PARAMETER+gbSyncNbParam++];
}

int dxl_sync_read_pop_word()
{
	int b0, b1;

    while(giBusUsing); // needs to be done before touching the TX buffer as it is used until the end of RX.	
	
    if ( gbSyncNbParam >= gbStatusPacket[LENGTH] - 3 )
    {
        return -1;
	}

	b0 = gbStatusPacket[PARAMETER + gbSyncNbParam++];
	b1 = gbStatusPacket[PARAMETER + gbSyncNbParam++];

    return dxl_makeword( b0, b1 );
}




