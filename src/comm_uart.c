/*
	Copyright 2015 Benjamin Vedder	benjamin@vedder.se

	This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

/*
 * comm_uart.c
 *
 *  Created on: 17 aug 2015
 *      Author: benjamin
 *  Modified on: 20 MAR 2017
 *      By: Ryan Owens
 *          - Modified for use with any linux serial device
 *          - Send and receive packets over serial port
 *          - Removed all code for STM32F4 Discovery
 *          - Uses a single rx function instead of processing threads.
 *            This allows more control of received data from
 *            multiple VESCs over CAN by calling rx only when
 *            application is ready to process data for each VESC.
 */

#include "vesc_uart/comm_uart.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <poll.h>
#include <sys/time.h>
#include <fcntl.h>
#include <termios.h>
#include "vesc_uart/bldc_interface_uart.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

// Settings
#define BAUDRATE B115200   // Change as needed, keep B

#define _POSIX_SOURCE 1 /* POSIX compliant source */

// Maximum receive buffer size
#define SERIAL_RX_BUFFER_SIZE 256

// Private functions
static void send_packet(unsigned char *data, unsigned int len);

// File Descriptor
static int fd;

/*
 * This function should be called every time data is requested from VESC.
 * Call this function at least 10ms after calling a get function.
 * Can receive and process multiple packets with a single call
 * as long as SERIAL_RX_BUFFER_SIZE is not exceeded.
 * Returns 1 to indicate that buffer has been read and sent to packet handler.
 */
int receive_packet() {
	int ret, i;
	static uint8_t buffer[SERIAL_RX_BUFFER_SIZE];
	// Read characters from Serial and put them in a buffer
	ret = read(fd, buffer, SERIAL_RX_BUFFER_SIZE);
	// Loop through the buffer and send each byte to the
	// packet handler
	for (i = 0; i < ret; i++) {
		bldc_interface_uart_process_byte(buffer[i]);
	}
	// clear any data on Serial Rx that was not read
	tcflush(fd, TCIFLUSH);
	return 1;
}

/**
 * Callback that the packet handler uses to send an assembled packet.
 *
 * @param data
 * Data array pointer
 * @param len
 * Data array length
 */
static void send_packet(unsigned char *data, unsigned int len) {
	if (len > (PACKET_MAX_PL_LEN + 5)) {
		return;
	}

	// Wait for the previous transmission to finish.
	tcdrain(fd);

	// Copy this data to a new buffer in case the provided one is re-used
	// after this function returns.
	static uint8_t buffer[PACKET_MAX_PL_LEN + 5];
	memcpy(buffer, data, len);

	// Send the data over Serial
	write(fd, buffer, len);
}

/*
 * Call this function once from main program to initialize 
 * serial port for reading (non-blocking) and writing.
 */
void comm_uart_init(char* modemDevice) {
	struct termios newtio;
	
	bzero(&newtio, sizeof(newtio)); /* clear struct for new port settings */

    /* BAUDRATE: Set bps rate. You could also use cfsetispeed and cfsetospeed.
       CS8     : 8n1 (8bit,no parity,1 stopbit)
       CLOCAL  : local connection, no modem contol
       CREAD   : enable receiving characters */
    newtio.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD;

    /* IGNPAR  : ignore bytes with parity errors
       otherwise make device raw (no other input processing) */
    newtio.c_iflag = IGNPAR;

    /*  Raw output  */
    newtio.c_oflag = 0;
    
    newtio.c_cc[VTIME] = 0;
    newtio.c_cc[VMIN]  = 1;

    /* ICANON  : enable canonical input
       disable all echo functionality, and don't send signals to calling program */
    newtio.c_lflag = 0;
	
    // Load the pin configuration
	/* Open modem device for reading and writing and not as controlling tty
    because we don't want to get killed if linenoise sends CTRL-C. */
    fd = open(modemDevice, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) { perror(modemDevice); exit(-1); }
    
    /* now clean the modem line and activate the settings for the port */
    tcflush(fd, TCIFLUSH);
    tcsetattr(fd,TCSANOW,&newtio);
    
	// Initialize the bldc interface and provide a send function
	bldc_interface_uart_init(send_packet);
}

/*
 * Call this function at end of main program to close 
 * serial port file descriptor.
 */
void comm_uart_close(void) {
	close(fd);
}