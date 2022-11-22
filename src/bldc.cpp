/*
	Copyright 2017 Ryan Owens

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
 * bldc.cpp
 *
 *  Created on: 19 mar 2017
 *      Author: Ryan
 */
#include "vesc_uart/bldc.h"
#include "vesc_uart/comm_uart.h"
#include "vesc_uart/bldc_interface.h"
#include "vesc_uart/bldc_interface_uart.h"
#include <unistd.h>
#include <stdio.h>
#include "vesc_uart/timers.h" // for receiving data. must compile with -lrt flag.
#include <vector> // for read data state machine
#include <iostream>

// Create new timer for reading data
timer_t* timerid = new_timer();

// Constant declarations
const int timerSP = 20; // timer set-point in milliseconds.
	                    // adjust for frequency of read data.
	                    // setting too low may cause packets to be dropped.
                        // 10ms is the lowest I have observed to work
                        // reliably on beaglebone black.
const RxData zeroRxData = {}; // rxData = 0

// global variables
static std::vector<RxMotor> motorList; // dynamic list to receive and store motor data
static std::vector<RxMotor>::iterator it; // iterator for motorList
static enum {REQUEST, READ} readMode = REQUEST; // data sampling state-machine

// Rx values and position callback functions
// BLDC interface will call these functions implicitly
// whenever the appropriate packet is received
static void bldc_val_received(mc_values *val);
static void bldc_pos_received(float pos); // should work after VESC firmware mod

//********************************************************************************
// Pre: rx_rotor_pos_func callback has been set
// Post: Rotor position in degrees is printed to console.
//       Will add pos data member instead of print after VESC firmware mod complete.
//********************************************************************************
void bldc_pos_received(float pos) {
	printf("\r\n");
	printf("Rotor position: %.2f degrees\r\n", pos);
	printf("\r\n");
}
//********************************************************************************
// Pre: rx_value_func callback has been set.
// Post: Motor data is stored in global Serial Rx buffer and
//       flag has been set indicating that data is available in buffer.
//********************************************************************************
void bldc_val_received(mc_values *val) {
	// set current iteration of motorList rxData from serial data
	it->rxData.voltageIn = val->v_in;
	it->rxData.tempPCB = val->temp_pcb;
	it->rxData.tempMOS1 = val->temp_mos1;
	it->rxData.tempMOS2 = val->temp_mos2;
	it->rxData.tempMOS3 = val->temp_mos3;
	it->rxData.tempMOS4 = val->temp_mos4;
	it->rxData.tempMOS5 = val->temp_mos5;
	it->rxData.tempMOS6 = val->temp_mos6;
	it->rxData.currentMotor = val->current_motor;
	it->rxData.currentIn = val->current_in;
	it->rxData.rpm = val->rpm;
	it->rxData.duty = val->duty_now;
	it->rxData.ampHours = val->amp_hours;
	it->rxData.ampHoursCharged = val->amp_hours_charged;
	it->rxData.wattHours = val->watt_hours;
	it->rxData.wattHoursCharged = val->watt_hours_charged;
	it->rxData.tachometer = val->tachometer;
	it->rxData.tachometerAbs = val->tachometer_abs;
	it->rxData.faultCode = bldc_interface_fault_to_string(val->fault_code);
	// printf("read\r\n");
}
//********************************************************************************
// Pre: Serial port is enabled
// Post: Serial interface is initialized.
//********************************************************************************
void BLDC::init(char* serialPort) {
	// Initialize Serial
    comm_uart_init(serialPort);
	// Set rx data callback functions
	bldc_interface_set_rx_value_func(bldc_val_received);
	// Won't work unless VESC firmware modified
	bldc_interface_set_rx_rotor_pos_func(bldc_pos_received);
}
//********************************************************************************
// Pre: Serial fd is open.
// Post: Serial fd is closed for reading and writing.
//********************************************************************************
void BLDC::close() {
	comm_uart_close();
}
//********************************************************************************
// Pre: None
// Post: Total Number of BLDC objects is returned.
//********************************************************************************
int BLDC::num_Motors(void) {
	return motorList.size();
}
//********************************************************************************
// Pre: At least one motor object is instantiated
// Post: Data for all VESCs is sampled from serial port
//********************************************************************************
bool BLDC::sample_Data() {
	bool ret = false;
	bldc_interface_uart_run_timer();
	// Non-blocking read state-machine
	// A state-machine is necessary for multiple VESCs over CAN because
	// data packets do not identify which controller sent them.
	// Therefore, a timer based state-machine approach is used to guarantee that 
	// all of the requested data is available before attempting to read, and then
	// moving on to the next VESC.
	switch(readMode) {
		case REQUEST:
			request_Values(it->canId); // Send the serial command requesting data from VESC
			start_timer(timerid, timerSP); 
			readMode = READ;
			// printf("request\n");
			break;
		case READ:
			// If timer SP is reached, read data from serial port
			if (check_timer(timerid)) {
				receive_packet();
				readMode = REQUEST;
				it++; // move to the next VESC for reading
				if (it == motorList.end())
					it = motorList.begin();
				ret = true;
			}
			// printf("read\n");
			break;
	}
	return ret;
}
//********************************************************************************
// Pre: Serial port is initialized. vescID value is input.
// Post: Motor object instantiated with ID, zero rxData, and motor config.
//********************************************************************************
BLDC::BLDC(VescID vescID, Motor_Config motorConfig) : id(vescID), config(motorConfig) {
	RxMotor init = {vescID, zeroRxData};
	dataPos = motorList.size(); // store position in motorList to use for accessing data
	motorList.push_back(init); // add to motorList with vesc ID and zero data
	it = motorList.begin(); // reset the motorList iterator to beginning
}
//********************************************************************************
//********************************************************************************
BLDC::~BLDC() {
}
//********************************************************************************
// Pre: Serial port is initialized. Speed in RPM is input.
// Post: Speed of motor has been set in RPM.
//********************************************************************************
void BLDC::set_Speed(int rpm) {
    // brake if value is within neutral range
    if (rpm > -1*config.Min_Erpm && rpm < config.Min_Erpm) {
		// apply brake if braking enabled
		if (config.enable_brake) {
			bldc_interface_set_forward_can(id);
        	apply_Brake(config.Brake_Current);
        	return;
		}
		else
			rpm = 0;
	}
    // set controller id and send the packet
    bldc_interface_set_forward_can(id);
    bldc_interface_set_rpm(rpm);
}
//********************************************************************************
// Pre: Serial port is initialized. Unscaled Analog/Digital value is input.
// Post: Speed of motor has been set in RPM.
//********************************************************************************
void BLDC::set_Speed_Unscaled(float val) {
    int rpm;
    // scale val to rpm value
    rpm = scale_To_Int(val, -1*config.Max_Erpm, config.Max_Erpm);
    set_Speed(rpm);
}
//********************************************************************************
// Pre: Serial port is initialized. Value in amps is input.
// Post: Current of motor has been set in Amps.
//********************************************************************************
void BLDC::set_Current(float amps) {
    // brake if value is within neutral range
    if (amps > -1*config.Min_Amps && amps < config.Min_Amps) {
		std::cout << "kokokokoko" << std::endl;
		// apply brake if braking enabled
		if (config.enable_brake) {
			bldc_interface_set_forward_can(id);
        	apply_Brake(config.Brake_Current);
        	return;
		}
		else
			amps = 0;
	}
    // set controller id and send the packet
    bldc_interface_set_forward_can(id);
    bldc_interface_set_current(amps);
}
//********************************************************************************
// Pre: Serial port is initialized. Unscaled Analog/Digital value is input.
// Post: Current of motor has been set in Amps.
//********************************************************************************
void BLDC::set_Current_Unscaled(float val) {
    float amps;
    // scale val to current value
    amps = scale_To_Float(val, -1*config.Max_Amps, config.Max_Amps);
    set_Current(amps);
}
//********************************************************************************
// Pre: Serial port is initialized. Brake current in amps is input.
// Post: Braking has been applied to motor with given current.
//********************************************************************************
void BLDC::apply_Brake(float brake) {
    // No scaling required
    bldc_interface_set_forward_can(id);
    bldc_interface_set_current_brake(brake);
}
//********************************************************************************
// Pre: Serial port is initialized. Duty cycle is input as percentage.
// Post: Duty cycle of motor has been set (percentage).
//********************************************************************************
void BLDC::set_Duty(float duty) {
    // brake if value is within neutral range
    if (duty > -1*config.Min_Duty && duty < config.Min_Duty) {
		// apply brake if braking enabled
		if (config.enable_brake) {
			bldc_interface_set_forward_can(id);
        	apply_Brake(config.Brake_Current);
        	return;
		}
		else
			duty = 0;
    }   
    // set controller id and send the packet
    bldc_interface_set_forward_can(id);
    bldc_interface_set_duty_cycle(duty);
}
//********************************************************************************
// Pre: Serial port is initialized. Unscaled Analog/Digital value is input.
// Post: Duty cycle of motor has been set (percentage).
//********************************************************************************
void BLDC::set_Duty_Unscaled(float val) {
	float duty;
	duty = scale_To_Float(duty, -1*config.Max_Duty, config.Max_Duty);
	set_Duty(duty);
}
//********************************************************************************
// Pre: Serial port is initialized. Position 0-360 degrees is input.
// Post: Position of motor has been set in in degrees.
//********************************************************************************
void BLDC::set_Pos(float pos) {
    // set controller id and send the packet
    bldc_interface_set_forward_can(id);
    bldc_interface_set_pos(pos);
}
//********************************************************************************
// Pre: Rx callback functions have been set.
// Post: Data has been read from Serial and stored in rxData.
//********************************************************************************
void BLDC::request_Values(int canId) {
	bldc_interface_set_forward_can(canId);
	bldc_interface_get_values();
}
//********************************************************************************
// Pre: Rx callback functions have been set. Also change VESC firmware.
// Post: Rotor position callback function is invoked (pos printed to console).
//********************************************************************************
void BLDC::request_Pos(void) {
	// Position read over Serial requires non-standard firmware
	bldc_interface_set_forward_can(id);
	bldc_interface_get_rotor_pos();
}
//********************************************************************************
// Pre: Motor is spinning and command has not been given before timeout occurs.
// Post: VESC timer has been reset and motor keeps spinning.
//********************************************************************************
void BLDC::send_Alive(void) {
	bldc_interface_set_forward_can(id);
	bldc_interface_send_alive();
}
//********************************************************************************
// Pre: BLDC object initialized.
// Post: rxData rof current object returned as RxData struct.
//********************************************************************************
RxData BLDC::get_Values(void) {
	return motorList[dataPos].rxData;
}
//********************************************************************************
// Pre: BLDC object initialized.
// Post: Rotor position returned as float.
//********************************************************************************
float BLDC::get_Pos(void) {
	// TODO
}
//********************************************************************************
// Pre: None.
// Post: rxData printed to console.
//********************************************************************************
void BLDC::print_Data(void) {
	printf("\r\n");
	printf("Input voltage: %.2f V\r\n", motorList[dataPos].rxData.voltageIn);
	printf("Temp PCB:      %.2f degC\r\n", motorList[dataPos].rxData.tempPCB);
	printf("Temp MOSFET1:  %.2f degC\r\n", motorList[dataPos].rxData.tempMOS1);
	printf("Temp MOSFET2:  %.2f degC\r\n", motorList[dataPos].rxData.tempMOS2);
	printf("Temp MOSFET3:  %.2f degC\r\n", motorList[dataPos].rxData.tempMOS3);
	printf("Temp MOSFET4:  %.2f degC\r\n", motorList[dataPos].rxData.tempMOS4);
	printf("Temp MOSFET5:  %.2f degC\r\n", motorList[dataPos].rxData.tempMOS5);
	printf("Temp MOSFET6:  %.2f degC\r\n", motorList[dataPos].rxData.tempMOS6);
	printf("Current motor: %.2f A\r\n", motorList[dataPos].rxData.currentMotor);
	printf("Current in:    %.2f A\r\n", motorList[dataPos].rxData.currentIn);
	printf("RPM:           %.1f RPM\r\n", motorList[dataPos].rxData.rpm);
	printf("Duty cycle:    %.1f %%\r\n", motorList[dataPos].rxData.duty);
	printf("Ah Drawn:      %.4f Ah\r\n", motorList[dataPos].rxData.ampHours);
	printf("Ah Regen:      %.4f Ah\r\n", motorList[dataPos].rxData.ampHoursCharged);
	printf("Wh Drawn:      %.4f Wh\r\n", motorList[dataPos].rxData.wattHours);
	printf("Wh Regen:      %.4f Wh\r\n", motorList[dataPos].rxData.wattHoursCharged);
	printf("Tacho:         %i counts\r\n", motorList[dataPos].rxData.tachometer);
	printf("Tacho ABS:     %i counts\r\n", motorList[dataPos].rxData.tachometerAbs);
	printf("Fault Code:    %s\r\n", motorList[dataPos].rxData.faultCode.c_str());
	printf("\r\n");
}

float BLDC::get_Current(void) {
	// current?
	return motorList[dataPos].rxData.tempMOS4*0.1;
}

//********************************************************************************
// Pre: Value to scale, min and max values for mapping have been input.
// Post: Value has been scaled and returned as float.
//********************************************************************************
float BLDC::scale_To_Float(float val, float min, float max) {
    return (max-min)/(config.Scale_Max-config.Scale_Min) * val;
}
//********************************************************************************
// Pre: Value to scale, min and max values for mapping have been input.
// Post: Value has been scaled to and returned as integer.
//********************************************************************************
int BLDC::scale_To_Int(float val, int min, int max) {
    return ((max-min)/(config.Scale_Max-config.Scale_Min)) * val;
}