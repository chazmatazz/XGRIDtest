/************************************************************************/
/* avr-xgrid                                                            */
/*                                                                      */
/* main.cpp                                                             */
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

#include "main.h"

// USART

#define USART_TX_BUF_SIZE 64
#define USART_RX_BUF_SIZE 64
char usart_txbuf[USART_TX_BUF_SIZE];
char usart_rxbuf[USART_RX_BUF_SIZE];
CREATE_USART(usart, UART_DEVICE_PORT);
FILE usart_stream;

#define NODE_TX_BUF_SIZE 32
#define NODE_RX_BUF_SIZE 64
char usart_n0_txbuf[NODE_TX_BUF_SIZE];
char usart_n0_rxbuf[NODE_RX_BUF_SIZE];
CREATE_USART(usart_n0, USART_N0_DEVICE_PORT);
char usart_n1_txbuf[NODE_TX_BUF_SIZE];
char usart_n1_rxbuf[NODE_RX_BUF_SIZE];
CREATE_USART(usart_n1, USART_N1_DEVICE_PORT);
char usart_n2_txbuf[NODE_TX_BUF_SIZE];
char usart_n2_rxbuf[NODE_RX_BUF_SIZE];
CREATE_USART(usart_n2, USART_N2_DEVICE_PORT);
char usart_n3_txbuf[NODE_TX_BUF_SIZE];
char usart_n3_rxbuf[NODE_RX_BUF_SIZE];
CREATE_USART(usart_n3, USART_N3_DEVICE_PORT);
char usart_n4_txbuf[NODE_TX_BUF_SIZE];
char usart_n4_rxbuf[NODE_RX_BUF_SIZE];
CREATE_USART(usart_n4, USART_N4_DEVICE_PORT);
char usart_n5_txbuf[NODE_TX_BUF_SIZE];
char usart_n5_rxbuf[NODE_RX_BUF_SIZE];
CREATE_USART(usart_n5, USART_N5_DEVICE_PORT);

Usart *usart_n[6];

Xgrid xgrid;

// SPI

Spi spi(&SPI_DEV);

// I2C

I2c i2c(&I2C_DEV);

// Timer

volatile unsigned long jiffies = 0;

// KENS CODE

void swarm_initialization();
void swarm_communication();
void swarm_calculation();
void swarm_interaction(int i, int j, int nei);

#define QUICK_SERVO 0
#define SMART_SERVO 1
#define SERVO_MODE QUICK_SERVO

int16_t smart_servo_pos_deg;

						// connect RTC clock source to 32.768 kHz (taken from 32.768 kHz oscillator)
	
void set_servo_position(int16_t degrees);
void set_servo_position(int16_t degrees)
{
	uint16_t compare_value;		// this value will ultimately fill the PER register of the timer/counter

	compare_value = 785 + degrees*5; // ** CALIBRATED (w/ opt.lvl -O1) for +/- 90 degrees and ZEROED when servo_position_deg==0 **
	// compare_value = 760 + (int16_t)degrees*2.7777; // ** CORRECT, but NOT CALIBRATED **
	
	TCE1.CCABUF = compare_value;	// fill the clock-compare A-buffer with the servo position val (for pin OC1A)
									// CCABUF will be loaded into CCB on the next UPDATE event (counter value = BOTTOM)
	
	TCE1.CCBBUF = compare_value;	// fill the clock-compare B-buffer with the servo position val (for pin OC1B)
									// CCBBUF will be loaded into CCB on the next UPDATE event (counter value = BOTTOM)
}	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	

// Production signature row access
uint8_t SP_ReadCalibrationByte( uint8_t index )
{
        uint8_t result;
        NVM_CMD = NVM_CMD_READ_CALIB_ROW_gc;
        result = pgm_read_byte(index);
        NVM_CMD = NVM_CMD_NO_OPERATION_gc;
        return result;
}

// User signature row access
uint8_t SP_ReadUserSigRow( uint8_t index )
{
        uint8_t result;
        NVM_CMD = NVM_CMD_READ_USER_SIG_ROW_gc;
        result = pgm_read_byte(index);
        NVM_CMD = NVM_CMD_NO_OPERATION_gc;
        return result;
}

// Timer tick ISR (1 kHz)
ISR(TCC0_OVF_vect)
{
        // Timers
        jiffies++;
        
        if (jiffies % 50 == 0)
			LED_PORT.OUTTGL = LED_USR_0_PIN_bm;		// toggle red light every 50 ms (20 Hz)
        
		if (jiffies % 100 == 0)
		{
			swarm_communication();
			swarm_calculation();
		}		
       
	    xgrid.process();
}

void rx_pkt(Xgrid::Packet *pkt)
{
        usart.write_string("RX: ");
        usart.write(pkt->data, pkt->data_len);
        usart.put('\r');
		usart.put('\n');
        LED_PORT.OUTTGL = LED_USR_2_PIN_bm;		// toggle green light when receive packet?
}

// Init everything
void init(void)
{
        // clock
        OSC.CTRL |= OSC_RC32MEN_bm; // turn on 32 MHz oscillator
        while (!(OSC.STATUS & OSC_RC32MRDY_bm)) { }; // wait for it to start
        CCP = CCP_IOREG_gc;
        CLK.CTRL = CLK_SCLKSEL_RC32M_gc; // switch osc
        DFLLRC32M.CTRL = DFLL_ENABLE_bm; // turn on DFLL
        
        // disable JTAG
        CCP = CCP_IOREG_gc;
        MCU.MCUCR = 1;
        
        // Init pins
        LED_PORT.OUTCLR = LED_USR_0_PIN_bm | LED_USR_1_PIN_bm | LED_USR_2_PIN_bm;
        LED_PORT.DIRSET = LED_USR_0_PIN_bm | LED_USR_1_PIN_bm | LED_USR_2_PIN_bm;
        
        // Init buttons
        BTN_PORT.DIRCLR = BTN_PIN_bm;
        
        // UARTs
        usart.set_tx_buffer(usart_txbuf, USART_TX_BUF_SIZE);
        usart.set_rx_buffer(usart_rxbuf, USART_RX_BUF_SIZE);
        usart.begin(UART_BAUD_RATE);
        usart.setup_stream(&usart_stream);
        
        usart_n0.set_tx_buffer(usart_n0_txbuf, NODE_TX_BUF_SIZE);
        usart_n0.set_rx_buffer(usart_n0_rxbuf, NODE_RX_BUF_SIZE);
        usart_n0.begin(NODE_BAUD_RATE);
        usart_n[0] = &usart_n0;
        xgrid.add_node(&usart_n0);
        usart_n1.set_tx_buffer(usart_n1_txbuf, NODE_TX_BUF_SIZE);
        usart_n1.set_rx_buffer(usart_n1_rxbuf, NODE_RX_BUF_SIZE);
        usart_n1.begin(NODE_BAUD_RATE);
        usart_n[1] = &usart_n1;
        xgrid.add_node(&usart_n1);
        usart_n2.set_tx_buffer(usart_n2_txbuf, NODE_TX_BUF_SIZE);
        usart_n2.set_rx_buffer(usart_n2_rxbuf, NODE_RX_BUF_SIZE);
        usart_n2.begin(NODE_BAUD_RATE);
        usart_n[2] = &usart_n2;
        xgrid.add_node(&usart_n2);
        usart_n3.set_tx_buffer(usart_n3_txbuf, NODE_TX_BUF_SIZE);
        usart_n3.set_rx_buffer(usart_n3_rxbuf, NODE_RX_BUF_SIZE);
        usart_n3.begin(NODE_BAUD_RATE);
        usart_n[3] = &usart_n3;
        xgrid.add_node(&usart_n3);
        usart_n4.set_tx_buffer(usart_n4_txbuf, NODE_TX_BUF_SIZE);
        usart_n4.set_rx_buffer(usart_n4_rxbuf, NODE_RX_BUF_SIZE);
        usart_n4.begin(NODE_BAUD_RATE);
        usart_n[4] = &usart_n4;
        xgrid.add_node(&usart_n4);
        usart_n5.set_tx_buffer(usart_n5_txbuf, NODE_TX_BUF_SIZE);
        usart_n5.set_rx_buffer(usart_n5_rxbuf, NODE_RX_BUF_SIZE);
        usart_n5.begin(NODE_BAUD_RATE);
        usart_n[5] = &usart_n5;
        xgrid.add_node(&usart_n5);
        
        // ADC setup
        ADCA.CTRLA = ADC_DMASEL_OFF_gc | ADC_FLUSH_bm;
        ADCA.CTRLB = ADC_CONMODE_bm | ADC_RESOLUTION_12BIT_gc;
        ADCA.REFCTRL = ADC_REFSEL_INT1V_gc | ADC_BANDGAP_bm;
        ADCA.EVCTRL = ADC_SWEEP_0123_gc | ADC_EVSEL_0123_gc | ADC_EVACT_SWEEP_gc;
        ADCA.PRESCALER = ADC_PRESCALER_DIV64_gc;
        ADCA.CALL = SP_ReadCalibrationByte(PROD_SIGNATURES_START + ADCACAL0_offset);
        ADCA.CALH = SP_ReadCalibrationByte(PROD_SIGNATURES_START + ADCACAL1_offset);
        
        ADCA.CH0.CTRL = ADC_CH_GAIN_1X_gc | ADC_CH_INPUTMODE_SINGLEENDED_gc;
        ADCA.CH0.MUXCTRL = ADC_CH_MUXPOS_PIN1_gc;
        ADCA.CH0.INTCTRL = ADC_CH_INTMODE_COMPLETE_gc | ADC_CH_INTLVL_LO_gc;
        ADCA.CH0.RES = 0;
        
        ADCA.CH1.CTRL = ADC_CH_GAIN_1X_gc | ADC_CH_INPUTMODE_SINGLEENDED_gc;
        ADCA.CH1.MUXCTRL = ADC_CH_MUXPOS_PIN2_gc;
        ADCA.CH1.INTCTRL = ADC_CH_INTMODE_COMPLETE_gc | ADC_CH_INTLVL_LO_gc;
        ADCA.CH1.RES = 0;
        
        ADCA.CH2.CTRL = ADC_CH_GAIN_1X_gc | ADC_CH_INPUTMODE_SINGLEENDED_gc;
        ADCA.CH2.MUXCTRL = ADC_CH_MUXPOS_PIN3_gc;
        ADCA.CH2.INTCTRL = ADC_CH_INTMODE_COMPLETE_gc | ADC_CH_INTLVL_LO_gc;
        ADCA.CH2.RES = 0;
        
        ADCA.CH3.CTRL = ADC_CH_GAIN_1X_gc | ADC_CH_INPUTMODE_SINGLEENDED_gc;
        ADCA.CH3.MUXCTRL = ADC_CH_MUXPOS_PIN4_gc;
        ADCA.CH3.INTCTRL = ADC_CH_INTMODE_COMPLETE_gc | ADC_CH_INTLVL_LO_gc;
        ADCA.CH3.RES = 0;
        
        //ADCA.CTRLA |= ADC_ENABLE_bm;
        //ADCA.CTRLB |= ADC_FREERUN_bm;
        
        // TCC
        TCC0.CTRLA = TC_CLKSEL_DIV256_gc;
        TCC0.CTRLB = 0;
        TCC0.CTRLC = 0;
        TCC0.CTRLD = 0;
        TCC0.CTRLE = 0;
        TCC0.INTCTRLA = TC_OVFINTLVL_LO_gc;
        TCC0.INTCTRLB = 0;
        TCC0.CNT = 0;
        TCC0.PER = 125;
        
        // ADC trigger on TCC0 overflow
        //EVSYS.CH0MUX = EVSYS_CHMUX_TCC0_OVF_gc;
        //EVSYS.CH0CTRL = 0;
        
        // I2C
        //i2c.begin(400000L);
        
        // SPI
        //spi.begin(SPI_MODE_2_gc, SPI_PRESCALER_DIV4_gc, 1);
        
        // CS line
        //SPI_CS_PORT.OUTSET = SPI_CS_DEV_PIN_bm;
        //SPI_CS_PORT.DIRSET = SPI_CS_DEV_PIN_bm;
        
        // Interrupts
        PMIC.CTRL = PMIC_LOLVLEN_bm | PMIC_MEDLVLEN_bm;
        
        sei();
		
		//LED_PORT.OUTTGL = LED_USR_1_PIN_bm;
}

// KEN'S CODE	============================================================================================

float ld = 20.0;		// virtual distance between the nodes
float dt = 0.05;		// time step for difference equation
float acc = 10.0;		// self-propelling force
float gmma =5.0;		// viscosity
float ka = 0.1;			// spring constant
float d = 0.5;			// strength of anisotropy
float cf = 100.0;		// strength of interaction with neighbors
float rc = 20.0;		// optimum distance between agents
float tau = 1.0;		// relaxation time of heading dynamics
float forcex, forcey;
//int delayX = 10;	(converted to #def)

int i, j, t;	// moved from swarm_initialization() to global

#define PI 3.14159
#define RADtoDEG 57.29578
#define delayX 10
#define SIZEX 10
#define SIZEY 10

struct OBJ{
	float px, py, vx, vy, hd, dt, vpx, vpy;
	float mempx[delayX], mempy[delayX];
	float neix[4], neiy[4];
	int mempt;
} mchip;

void swarm_initialization()
{
	// SERVO: initialize the two servo data-pins as output pins
	
		PORTE.DIRSET = PIN4_bm; // (PWM_1) make pin E4 writeable, by default all initialize as readable
		PORTE.DIRSET = PIN5_bm; // (PWM_2) make pin E5 writeable, by default all initialize as readable

	if(SERVO_MODE == QUICK_SERVO)
	{
	// SERVO: Begin a timer-counter that will reach TOP (overflow) every 20ms (servo signal is 50 Hz)
	
		TCE1.PER = 10126; // Set period (10126 ticks@~500KHz -> ~20.0ms) ** CALIBRATED (w/ opt.lvl -O1), meas abt. 506.27 ticks/ms **
		//TC_SetPeriod( &TCE1, 9999 );  // Set period (10000 ticks@500KHz = 20ms)  ** CORRECT, but NOT CALIBRATED **
		
		TCE1.CCA = 785;	//initially fill the CCA register with a value that is calibrated to zero the servo (~1.5 ms on time)
		TCE1.CCB = 785;	//initially fill the CCB register with a value that is calibrated to zero the servo (~1.5 ms on time)
		TCE1.CTRLA |= 0b00000101;		// Set clock and prescaler, 32MHz/64 = 500KHz
		TCE1.CTRLB |= 0b00000011;		// enable Single Slope PWM (Waveform Generation Mode)
		TCE1.CTRLB |= 0b00010000;		// enable waveform output on OC1A (setting this in WGM of operation overrides the port output register for this output pin: portEpin4)
		TCE1.CTRLB |= 0b00100000;		// enable waveform output on OC1B (setting this in WGM of operation overrides the port output register for this output pin: portEpin5)
	}	
	
	else if(SERVO_MODE == SMART_SERVO)
	{
	// SERVO: Begin a clock counter that will trigger an overflow-interrupt (TCE1) every 20ms -> (50 Hz)
	
		TCE1.PER = 10126;  // Set period (10126 ticks@~500KHz -> ~20.0ms) ** CALIBRATED (w/ opt.lvl -O1), meas abt. 506.27 ticks/ms **
		
		TCE1.INTCTRLA = 0b00000010; // set a medium-level interrupt on overflow
		
		TCE1.CTRLA = 0b00000101; // Set clock and prescaler, 32MHz/64 = 500KHz
		
		TCE1.PER = 10126;  // Set period (10126 ticks@~500KHz -> ~20.0ms) ** CALIBRATED (w/ opt.lvl -O1), meas abt. 506.27 ticks/ms **
		
		TCE1.INTCTRLB |= 0b00000010;  // set a medium-level interrupt on CCA compare
		
	}

//**** INITIALIZE REAL TIME COUNTER (RTC) / REAL TIME CLOCK *********************************************************************************	
		
		
	//	OSC.CTRL |= 0b00000100;								// enable the internal 32.768 kHz oscillator
	//	do {} while ( (OSC.STATUS & 0b00000100) == 0 );		// wait for oscillator to stabilize
	//	CLK.RTCCTRL |= 0b00000100;							// connect RTC clock source to 1.024 kHz (taken from 32.768 kHz oscillator)
	//	//CLK.RTCCTRL |= 0b00001010;	
	
	mchip.px=0;
	mchip.py=0;
	mchip.vpx=0.0;
	mchip.vpy=0.0;

	for(t=0;t<delayX;t++){
		mchip.mempx[t]=0;
		mchip.mempy[t]=0;
	}
	mchip.mempt=0;

	mchip.vx = 1.0;
	mchip.vy = 1.0;
	mchip.hd = PI/4.0;

	for(t=0;t<4;t++){
		mchip.neix[t]=0;
		mchip.neiy[t]=0;
	}
	
	
}

void swarm_communication()
{
	/*
	Here I assume:
		Send: the value of 'mchip.vpx' and 'mchip.vpy' to the neighbors
		Receive and store:
			(mchip.neix[0], mchip.neiy[0]) <- bottom Chip's (.vpx, .vpy)
			(mchip.neix[1], mchip.neiy[1]) <- left Chip's (.vpx, .vpy)
			(mchip.neix[2], mchip.neiy[2]) <- right Chip's (.vpx, .vpy)
			(mchip.neix[3], mchip.neiy[3]) <- top Chip's (.vpx, .vpy)
	*/
	
	
}

void swarm_calculation()
{
	//int check;	// unused
	float dvx, dvy, lx, ly, vabs, ds, fx, fy;
	float dir = mchip.hd;
	float cvx = mchip.vx;
	float cvy = mchip.vy;

	// ===== calculation of forces =====
	//self-propel force and viscosity
	dvx = acc * cos(dir) - gmma * cvx;
	dvy = acc * sin(dir) - gmma * cvy;

	//interaction force with 4 neighbors
	//It contains the process for boundaries.
	forcex=0; forcey=0;
	if(i!=0)  			swarm_interaction(i,j,1);
	if(i!=SIZEX-1) 		swarm_interaction(i,j,2);
	if(j!=0)  			swarm_interaction(i,j,3);
	if(j!=SIZEY-1)		swarm_interaction(i,j,0);
	dvx = dvx + forcex;
	dvy = dvy + forcey;

	//spring term (Zero-length spring). fixed point = (0,0)
	lx = - mchip.px;
	ly = - mchip.py;
	dvx = dvx + ka * lx;
	dvy = dvy + ka * ly;

	// ===== update =====
	//direction
	vabs = sqrt(cvx * cvx + cvy * cvy);
	fx = cvx / vabs;
	fy = cvy / vabs;
	ds = -1.0 / tau * (sin(dir)*fx-cos(dir)*fy); // get sin value by cross product
	mchip.hd += ds * dt;

	if(mchip.hd > 2.0 * PI)		mchip.hd -= 2.0 * PI;
	if(mchip.hd < 0 ) 			mchip.hd += 2.0 * PI;

	//velocity
	mchip.vx += dvx * dt;
	mchip.vy += dvy * dt;

	//position
	mchip.px += mchip.vx * dt;
	mchip.py += mchip.vy * dt;

	//This memory is for intentional delay effect
	mchip.mempx[mchip.mempt] = mchip.px;
	mchip.mempy[mchip.mempt] = mchip.py;

	mchip.vpx = mchip.mempx[mchip.mempt];
	mchip.vpy = mchip.mempy[mchip.mempt];

	mchip.mempt++;
	if(mchip.mempt == delayX)	mchip.mempt = 0;

}

//==============================================

void swarm_interaction(int i, int j, int nei)
{
	float dirx, diry, disx, disy, dis1, dis2, alph, force;
	int di,dj;

	dirx = cos(mchip.hd);
	diry = sin(mchip.hd);

	switch(nei){
		case 0: di=0; dj=+1; break;
		case 1: di=-1; dj=0; break;
		case 2: di=+1; dj=0; break;
		case 3: di=0; dj=-1; break;
	}
	
	disx = mchip.neix[nei] + ld * di - mchip.px;
	disy = mchip.neiy[nei] + ld * dj - mchip.py;

	dis2 = disx * disx + disy * disy;
	dis1 = sqrt(dis2);

	alph = 1.0 + d * (disx * dirx + disy * diry) / dis1; //inner product
	force = -cf * (rc / dis1 - 1.0) * rc * rc / dis2;
	forcex = forcex + alph * force * disx / dis1;
	forcey = forcey + alph * force * disy / dis1;
}

void servo_motor_control();
void servo_motor_control()
{
	int servo_pos_deg;
	
	// Angle of axis Aa is calculated by
	// Aa = 2.0 * cos(mchip.hd)

	// This is just an example. There are many ways to translate
	// virtual agent's movement to the servo's angle.

	// mchip.hd (range: 0 - 2*PI)

	servo_pos_deg = (mchip.hd-PI)*RADtoDEG;

	if(SERVO_MODE == QUICK_SERVO) {set_servo_position((int)mchip.hd*10);}

	
	
	
	fprintf_P(&usart_stream, PSTR("hd:%li, %i\r\n"), (long int)(mchip.hd*100000), servo_pos_deg);

}

// END KEN'S CODE	============================================================================================

ISR(RTC_OVF_vect)
{
	if(smart_servo_pos_deg < 0)
	{
		smart_servo_pos_deg = 45;
	}	
		
	else
	{
		smart_servo_pos_deg = -45;
	}	
}


ISR(TCE1_OVF_vect)
{
	PORTB.OUT |= PIN1_bm; //(YELLOW LED) turn the bit on (for debugging, remove this line when complete)
	
	// SERVO: Begin the HIGH-pulse to the servo:
		
		PORTE.OUT |= PIN4_bm; // (PWM_1) set pin on
		PORTE.OUT |= PIN5_bm; // (PWM_2) set pin on
	
	// SERVO: calculate how long the HIGH-pulse to the servo should last
	
	set_servo_position(smart_servo_pos_deg);
	
}

ISR(TCE1_CCA_vect)
{
    PORTB.OUT &= ~PIN1_bm; //(YELLOW LED) turn the bit off (for debugging, remove this line when complete)
	
	// SERVO: End the HIGH-pulse to the servo, clear the interrupt, stop counting on TCD1 :
	
		PORTE.OUT &= ~PIN4_bm; // (PWM_1) set pin off
		PORTE.OUT &= ~PIN5_bm; // (PWM_2) set pin off
	
		//TC1_SetOverflowIntLevel( &TCE1, TC_OVFINTLVL_OFF_gc ); // Clear Interrupt on Overflow (is this necessary??)
		//TC1_ConfigClockSource( &TCE1, TC_CLKSEL_OFF_gc );      // Stop clock
}


int main(void)
{
        //char old_btn = 0;
        //char btn;
        uint32_t j = 0;
        
		char input_char, first_char;
		
        _delay_ms(50);
        
        init();
        
        xgrid.rx_pkt = &rx_pkt;
        
        LED_PORT.OUT = LED_USR_0_PIN_bm;
        
        fprintf_P(&usart_stream, PSTR("avr-xgrid build %ld\r\n"), (unsigned long) &__BUILD_NUMBER);
        
		/*
		char str[] = "boot up";
		Xgrid::Packet pkt;
		pkt.type = 0;
		pkt.flags = 0;
		pkt.radius = 1;
		pkt.data = (uint8_t *)str;
		pkt.data_len = 4;
                        
		xgrid.send_packet(&pkt);
		*/
		
		// KEN'S CODE 
		
		swarm_initialization();
		
		int16_t servo_position_deg = 0;	// servo position in degrees
		
		// END KEN'S CODE
		
		if (usart.available())
		{
			first_char = usart.get();
		}	
		
        while (1)
        {
                j = jiffies + 10;	// from here, you have 10 ms to reach the bottom of this loop
				
				if (usart.available())
				{
					input_char = usart.get();
				}	
							
				// main loop
                if (input_char == 0x1b)
                        xboot_reset();
				
				else if(input_char != 0x00)
				{
					fprintf_P(&usart_stream, PSTR("CPU: %c**\r\n"), input_char);
					
					if (input_char == 'a')
					{
						char str[] = "A";
                        Xgrid::Packet pkt;		// Packet is defined on line 72 of xgrid.h
                        pkt.type = 0;
                        pkt.flags = 0;
                        pkt.radius = 1;
                        pkt.data = (uint8_t *)str;
                        pkt.data_len = 4;
                        
                        xgrid.send_packet(&pkt);
					}
					
					else if(input_char == 'y')
					{
						LED_PORT.OUTTGL = LED_USR_1_PIN_bm;
					}
					
					input_char = 0x00;	// clear the most recent computer input
				}
				
				// KEN'S CODE:
				
				//swarm_communication();
				//swarm_calculation();
				servo_motor_control();
				
		
                // END KEN'S CODE
				
				if(jiffies > j)	// if TRUE, then we took too long to get here, halt program
				{
					cli();	//disable_interrupts
					LED_PORT.OUTSET = LED_USR_0_PIN_bm;		// turn red light on and keep it on
					LED_PORT.OUTSET = LED_USR_1_PIN_bm;		// turn yellow light on and keep it on
					while(1==1)
					{};
				}					
				
                while (j > jiffies) { };
        }  
}








/*  CODE REMOVED FROM MAIN, 1st level code in while(1)
				//fprintf_P(&usart_stream, PSTR("%c"), 'g');					
                
                // check for button press
				btn = !(BTN_PORT.IN & BTN_PIN_bm);
                
				if (btn && (btn != old_btn))
				{
                        char str[] = "test";
                        Xgrid::Packet pkt;
                        pkt.type = 0;
                        pkt.flags = 0;
                        pkt.radius = 1;
                        pkt.data = (uint8_t *)str;
                        pkt.data_len = 4;
                        
                        xgrid.send_packet(&pkt);
                        
                        LED_PORT.OUTTGL = LED_USR_1_PIN_bm;
                }
                
                old_btn = btn;	
				
				
				
*/
                

