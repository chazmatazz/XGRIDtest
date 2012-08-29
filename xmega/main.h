/************************************************************************/
/* xgrid                                                                */
/*                                                                      */
/* main.h                                                               */
/*                                                                      */
/* Alex Forencich <alex@alexforencich.com>                              */
/*                                                                      */
/* Copyright (c) 2011 Alex Forencich                                    */
/*                                                                      */
/* Permission is hereby granted, free of charge, to any person          */
/* obtaining a copy of this software and associated documentation       */
/* files(the "Software"), to deal in the Software without restriction,  */
/* including without limitation the rights to use, copy, modify, merge, */
/* publish, distribute, sublicense, and/or sell copies of the Software, */
/* and to permit persons to whom the Software is furnished to do so,    */
/* subject to the following conditions:                                 */
/*                                                                      */
/* The above copyright notice and this permission notice shall be       */
/* included in all copies or substantial portions of the Software.      */
/*                                                                      */
/* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,      */
/* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF   */
/* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND                */
/* NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS  */
/* BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN   */
/* ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN    */
/* CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE     */
/* SOFTWARE.                                                            */
/*                                                                      */
/************************************************************************/

#ifndef __MAIN_H
#define __MAIN_H

#include <math.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#include <stdio.h>

#include "board.h"
#include "usart.h"
#include "spi.h"
#include "i2c.h"
#include "eeprom.h"
#include "xgrid.h"
#include "servo.h"
#include "../xboot/xbootapi.h"

// Build information
extern char   __BUILD_DATE;
extern char   __BUILD_NUMBER;

// defines

// typedefs

// Prototypes
void init(void);
int main(void);
uint8_t SP_ReadCalibrationByte( uint8_t index );
uint8_t SP_ReadUserSigRow( uint8_t index );

#ifndef ADCACAL0_offset

#define ADCACAL0_offset 0x20
#define ADCACAL1_offset 0x21
#define ADCBCAL0_offset 0x24
#define ADCBCAL1_offset 0x25

#endif

#endif // __MAIN_H

//##########################################################################
//##########################################################################
//##########################################################################

#define prt_flt3(f)	(int16_t)f,(int16_t)((fabs(f)-(int16_t)(fabs(f)))*1000)	

#define PI 3.14159
#define NUM_NEIGHBORS 6

//input port number					//output 
#define BOTTOM_LEFT				1	//0b00000010
#define BOTTOM_RIGHT			0	//0b00000001
#define LEFT_TOP				3	//0b00001000   
#define LEFT_BOTTOM				2	//0b00000100
#define RIGHT_BOTTOM			4	//0b00010000
#define RIGHT_TOP				5	//0b00100000 

#define MESSAGE_NUMDATA 0
#define MESSAGE_COMMAND 1
#define ALL_DIRECTION 0x3F
#define ALL 20

//##########################################################################

struct OBJ1
{
	float px, py, vx, vy, hd;
	float neix[NUM_NEIGHBORS], neiy[NUM_NEIGHBORS];
} agent0, agent1;

struct OBJ2
{
	float tim1, tim2;
	bool flg;
} agent2, agent3;

struct point
{
	float x1, y1;
	float x0, y0;
} mdata;

//##########################################################################

//===== common =====
volatile unsigned long jiffies = 0, temp_time, cnt4sensor = 0;

bool connected[NUM_NEIGHBORS];
bool sonar_attached = false;
bool dblchk = true, trichk = true;
uint16_t sensor_value, sensor_value_now, sensor_value_dblchk, sensor_value_trichk;
uint16_t sum_dbl = 0, sum_tri = 0;


int sec_counter = 0;
float decay_tim;
float global_amp;

bool communication_on = false;

bool sendmessage_fast = false;
bool sendmessage_slow = false;

bool display = false;
bool display_on = false;

bool reboot_on = false;
bool servo_motor_on = false;

bool speedup_on = false;
bool use_sensor_data_on = false;

bool special = false;	//marker of "Bottom-Left" module

//===== for "Ken's model" =====
bool sync = true;

//===== for "wave" and "column swing" =====
bool wave_flg = false,  wave_ping = false;
bool column_flg = false;
uint8_t wave_port;

//===== for "rhythm" =====
bool rhythm_on = false;

// ===== PROTOTYPE DECLARATION =====
void send_message(uint8_t, uint8_t, int, const char[]);
void servo_motor_control(float);

//##### SENSOR RANGE ######
#define RANGE1 50
#define RANGE2 150
#define RANGE3 250
#define IGNORE_DISTANCE 400

//##### TIMER ######
#define STGtime1 0		//sync
#define STGtime2 150	//FM
#define STGtime3 210	//AM
#define STGtime4 270	//break
#define STGtime5 300	//sync-chaos
#define STGtime6 360	//column swing
#define STGtime7 420	//plane wave
#define STGtime8 480	//avalanche
#define STGtime9 570	//break
#define LASTtime 630