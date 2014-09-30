/*                      */
/******************************************************************************
 NetBurner simple UDP send/receive example.

 This example program will allow you to send and receive UDP packets to
 another device or host computer. To run the example, connect a serial
 port to the debug serial port on your NetBurner device and run a dumb
 terminal program such as MTTTY. On the PC, run the NetBurner UDP Terminal
 (be sure to set the IP address and port numbers to match). You will then
 be able to type characters in the UDP Termainal and see them in MTTTY,
 and vice versa.

 You will be prompted for the port number to send/receive data and the
 destination IP address of the other device or host. Note that the application
 uses the same port number to send and receive data, but you can change this
 to any other port number you wish.

 The application will create a thread to recive packets and display them
 on the debug port, while the main task will take any data you type in
 to the MTTTY dumb terminal and send it as a udp packet to the destination
 IP address.

 *****************************************************************************/

/*********Modified by Tanmay******************/

//This Code sends Data to IP 157.182.196.87:80 and receives data from IP 157.182.196.83:80

#include "predef.h"
#include <stdio.h>
#include <ctype.h>
#include <startnet.h>
#include <ucos.h>
#include <udp.h>
//#include <autoupdate.h>
#include <dhcpclient.h>
#include <pitr_sem.h>//for PIT SEM

extern "C" {
	void UserMain(void * pd);
}


const char *AppName = "UDP Send/Receive Example";
BYTE* rec_buff=0;

// Allocate task stack for UDP listen task
DWORD   UdpTestStk[USER_TASK_STK_SIZE];



/*-------------------------------------------------------------------
 UDP Server task will wait for incoming packets on the
 designated port number, which is passed as a OSTaskCreate()
 void * parameter.
 -------------------------------------------------------------------*/
void UdpReaderMain(void * pd)
{
	int port = (int)pd;	  // cast void * param as int port number

	//iprintf("Listening on UDP port: %d\r\n", port);

	// Create a FIFO for the UDP packet and initialize it
	OS_FIFO fifo;
	OSFifoInit(&fifo);

	// Register to listen for UDP packets on port number 'port'
	RegisterUDPFifo(port, &fifo);

	while (1)
	{
		// Construct a UDP packet object using the previously
		// declared FIFO. The UDP constructor will only return
		// when a packet has been received. The second parameter
		// is a timeout value (timeticks). A value of 0 will
		// wait forever. The TICKS_PER_SECOND definition can
		// be used for code readability.
		UDPPacket upkt(&fifo, 0 * TICKS_PER_SECOND);

		// Did we get a valid packet, or just time out?
		if (upkt.Validate())
		{
			WORD len = upkt.GetDataSize();
			//iprintf("Received UDP packet with %d bytes from: ", (int)len);
			//ShowIP(upkt.GetSourceAddress());	// show ip address
			//iprintf("\r\n");

			//ShowData(upkt.GetDataBuffer(), len); // hex dump function
			rec_buff=upkt.GetDataBuffer();
			//for (int i=0;i<len;i++)
			iprintf("Packet_Type=%d,Counter=%d\n",rec_buff[3],rec_buff[19]);
			iprintf("\r\n");
		}
	}
}



/*-------------------------------------------------------------------
 UserMain
 -------------------------------------------------------------------*/
void UserMain(void * pd)
{
	int 	portnum=80;
	char 	c_addr[20]="157.182.196.87";
	BYTE	buffer[20]={0xFF};
	buffer[0]=0xAA;
	buffer[1]=0xAB;
	buffer[2]=0xBB;
	buffer[3]=0x02;




 	InitializeStack();

	//EnableAutoUpdate();
	OSChangePrio(MAIN_PRIO);

	iprintf("Starting NetBurner UDP Example \r\n");
	IPADDR ipaddr = AsciiToIp(c_addr);

	OS_SEM PitSem1;//Time Sem
    OSSemInit( &PitSem1, 0 );
    // Init for timer 1, at 20ms second intervals
    InitPitOSSem( 1, &PitSem1, 50 );

	// Create UDP listen task
	OSTaskCreate(UdpReaderMain,
				(void  *)portnum,
				&UdpTestStk[USER_TASK_STK_SIZE] ,
				UdpTestStk,
				MAIN_PRIO - 1);	// lower priority than send task


	// Main while loop will take any user input and send it as a
	// UDP datagram.
	while(1)
	{
		BYTE status = OSSemPend( &PitSem1, TICKS_PER_SECOND * 5 );
		if ( status == OS_NO_ERR ){
			{
				UDPPacket pkt;
				pkt.SetSourcePort(portnum);
				pkt.SetDestinationPort(portnum);
				pkt.AddData(buffer,20);
				//pkt.AddDataByte(0);
				pkt.Send(ipaddr);
				buffer[19]=buffer[19]+1;
			}
		}
	}
}
