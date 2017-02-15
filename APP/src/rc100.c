/*
 * rc100.c

 *
 *  Created on: 14 feb. 2017
 *      Author: kalle
 */

#include "rc100.h"
#include "typedefs.h"

unsigned char gb_rcv_packet[6];
unsigned char gb_rcv_packet_num;
unsigned short gw_rcv_data;
unsigned char gb_rcv_flag;

volatile byte                   gb_packet_wr_pointer = 0;
volatile byte                   gb_packet_rd_pointer = 0;
volatile byte                   gb_packet_pointer = 0;
volatile byte                   gb_packet_data_buffer[16+1+16];
volatile byte                   gb_packet[PACKET_LENGTH+2];
volatile byte                   gb_new_packet;

void RxD2Interrupt(void)
{
	/* Interrupt handler for the UART5 port (zigbee) */
	if(USART_GetITStatus(UART5, USART_IT_RXNE) != RESET)
	{
		word temp;
		temp = USART_ReceiveData(UART5);

		gb_packet_data_buffer[gb_packet_wr_pointer] = temp;
		gb_packet_wr_pointer++;
		//Simple modulo
		gb_packet_wr_pointer &= 0x1F;
	}
}

int rc100_has_new_data() {
	return gb_packet_rd_pointer == gb_packet_wr_pointer ? 0 : 1;
}

void rc100_init(void) {
	gb_rcv_flag = 0;
	gw_rcv_data = 0;
	gb_rcv_packet_num = 0;
	GPIO_ResetBits(PORT_ZIGBEE_RESET, PIN_ZIGBEE_RESET);
}


int rc100_receive( unsigned char *packet, int num_packets) {
	unsigned char i;
	for( i=0 ; i<num_packets ; i++ )
	{
		if (rc100_has_new_data()) {
			packet[i] = gb_packet_data_buffer[gb_packet_rd_pointer++];
		} else
			return i;
	}

	return num_packets;

}

int rc100_transmit( unsigned char *packet, int num_packets )
{
	unsigned char i;
	for(i=0 ; i<num_packets; i++  ) {
			USART_SendData(USART1, packet[i]);
			while( USART_GetFlagStatus(USART1, USART_FLAG_TC)==RESET );
	}


	return num_packets;
}


int rc100_check(void) {
	int rcv_num;
	unsigned char checksum;
	int i, j;

	if(gb_rcv_flag)
		return 1;

	// Fill packet buffer
	if(gb_rcv_packet_num < 6)
	{
		rcv_num = rc100_receive( &gb_rcv_packet[gb_rcv_packet_num], (6 - gb_rcv_packet_num) );
		if( rcv_num != -1 )
			gb_rcv_packet_num += rcv_num;
	}

	// Find header
	if(gb_rcv_packet_num >= 2)
	{
		for( i=0; i<gb_rcv_packet_num; i++ )
		{
			if(gb_rcv_packet[i] == 0xff)
			{
				if(i <= (gb_rcv_packet_num - 2))
				{
					if(gb_rcv_packet[i+1] == 0x55)
						break;
				}
			}
		}

		if(i > 0)
		{
			if(i == gb_rcv_packet_num)
			{
				// Can not find header
				if(gb_rcv_packet[i - 1] == 0xff)
					i--;
			}

			// Remove data before header
			for( j=i; j<gb_rcv_packet_num; j++)
			{
				gb_rcv_packet[j - i] = gb_rcv_packet[j];
			}
			gb_rcv_packet_num -= i;
		}
	}

	// Verify packet
	if(gb_rcv_packet_num == 6)
	{
		if(gb_rcv_packet[0] == 0xff && gb_rcv_packet[1] == 0x55)
		{
			checksum = ~gb_rcv_packet[3];
			if(gb_rcv_packet[2] == checksum)
			{
				checksum = ~gb_rcv_packet[5];
				if(gb_rcv_packet[4] == checksum)
				{
					gw_rcv_data = (unsigned short)((gb_rcv_packet[4] << 8) & 0xff00);
					gw_rcv_data += gb_rcv_packet[2];
					gb_rcv_flag = 1;
				}
			}
		}

		gb_rcv_packet[0] = 0x00;
		gb_rcv_packet_num = 0;
	}

	return gb_rcv_flag;



}

int rc100_send_data(int data) {
	unsigned char snd_packet[6];
	unsigned short word = (unsigned short)data;
	unsigned char lowbyte = (unsigned char)(word & 0xff);
	unsigned char highbyte = (unsigned char)((word >> 8) & 0xff);

	snd_packet[0] = 0xff;
	snd_packet[1] = 0x55;
	snd_packet[2] = lowbyte;
	snd_packet[3] = ~lowbyte;
	snd_packet[4] = highbyte;
	snd_packet[5] = ~highbyte;

	if( rc100_transmit( snd_packet, 6 ) != 6 )
		return 0;

	return 1;

}

int rc100_read_data(void) {
	gb_rcv_flag = 0;
	return (int)gw_rcv_data;

}

