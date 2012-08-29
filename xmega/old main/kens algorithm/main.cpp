/************************************************************************/
/* xgrid                                                                */
/*                                                                      */
/* xgrid.cpp                                                            */
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

#define prt_flt3(f)	(int16_t)f,(int16_t)((fabs(f)-(int16_t)(fabs(f)))*1000)			// use %i.%i inside print_f in place of %f


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

Xgrid xgrid;

// SPI

Spi spi(&SPI_DEV);

// I2C

I2c i2c(&I2C_DEV);

// Timer

volatile unsigned long jiffies = 0;

// KENS CODE	============================================================================================

void swarm_initialization();
void swarm_communication();
void swarm_calculation();
void swarm_interaction(int i, int j, int nei);
void servo_motor_control();

// END KENS CODE	============================================================================================

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
        
        if (jiffies % 25 == 0)
		{
			LED_PORT.OUTTGL = LED_USR_0_PIN_bm;		// toggle red light every 50 ms (20 Hz)
			//servo_motor_control();
			swarm_calculation();
			servo_motor_control();
		}		
		
		if (jiffies % 100 == 0)
		{

			//swarm_communication();
			swarm_calculation();
			
			//servo_motor_control();
			
			
			

		}		
       
		if (jiffies % 200 == 0)
		{

			swarm_communication();
			//servo_motor_control();
			

		}
	   
	   
	    xgrid.process();
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
        xgrid.add_node(&usart_n0);
        
		usart_n1.set_tx_buffer(usart_n1_txbuf, NODE_TX_BUF_SIZE);
        usart_n1.set_rx_buffer(usart_n1_rxbuf, NODE_RX_BUF_SIZE);
        usart_n1.begin(NODE_BAUD_RATE);
        xgrid.add_node(&usart_n1);
        
		usart_n2.set_tx_buffer(usart_n2_txbuf, NODE_TX_BUF_SIZE);
        usart_n2.set_rx_buffer(usart_n2_rxbuf, NODE_RX_BUF_SIZE);
        usart_n2.begin(NODE_BAUD_RATE);
        xgrid.add_node(&usart_n2);
        
		usart_n3.set_tx_buffer(usart_n3_txbuf, NODE_TX_BUF_SIZE);
        usart_n3.set_rx_buffer(usart_n3_rxbuf, NODE_RX_BUF_SIZE);
        usart_n3.begin(NODE_BAUD_RATE);
        xgrid.add_node(&usart_n3);
        
		usart_n4.set_tx_buffer(usart_n4_txbuf, NODE_TX_BUF_SIZE);
        usart_n4.set_rx_buffer(usart_n4_rxbuf, NODE_RX_BUF_SIZE);
        usart_n4.begin(NODE_BAUD_RATE);
        xgrid.add_node(&usart_n4);
        
		usart_n5.set_tx_buffer(usart_n5_txbuf, NODE_TX_BUF_SIZE);
        usart_n5.set_rx_buffer(usart_n5_rxbuf, NODE_RX_BUF_SIZE);
        usart_n5.begin(NODE_BAUD_RATE);
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
float forcex, forcey;   // force
//int delayX = 10;	(converted to #def)

int i, j, t;	// moved from swarm_initialization() to global

#define PI 3.14159
#define RADtoDEG 57.29578
#define delayX 10
#define SIZEX 10
#define SIZEY 10
#define NUM_NEIGHBORS 4

bool print_servo_info = false;

struct OBJ{
	// px, py point
	// vx, vy velocity of ?
	// hd heading (radians)
	// dt delta time
	// vpx, vpy velocity of point
	float px, py, vx, vy, hd, dt, vpx, vpy;
	// ?
	float mempx[delayX], mempy[delayX];
	// neighbors
	float neix[NUM_NEIGHBORS], neiy[NUM_NEIGHBORS];
	int mempt;
} mchip;


/**
 * Data transmitted to neighbors
 */
struct point {
	float x, y;
};

point mdata;
/*
 * Receive KEN's data
 */
void rx_pkt(Xgrid::Packet *pkt)
{
		point* pt_ptr = (point*) pkt->data;
		mchip.neix[pkt->source_id] = pt_ptr->x;
		mchip.neiy[pkt->source_id] = pt_ptr->y;
        LED_PORT.OUTTGL = LED_USR_2_PIN_bm;		// toggle green light when receive packet?
}

void swarm_initialization()
{
	init_servo();

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

	for(t=0;t<NUM_NEIGHBORS;t++){
		mchip.neix[t]=0;
		mchip.neiy[t]=0;
	}
}

/*
 * Just sending
 */
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
	mdata.x = mchip.vpx;
	mdata.y = mchip.vpy;
	Xgrid::Packet pkt;
	pkt.type = 0;
	pkt.flags = 0;	
	pkt.radius = 1;
	pkt.data = (uint8_t *)&mdata;
	pkt.data_len = sizeof(point);
											//									
	xgrid.send_packet(&pkt,0b00100111);		// do not send packets in the 'horizontal' directions
											//								
	/* optional bit mask, to send to select ports
		mask			xgrid node name		datasheet	slikscreen	direction
		-----------------------------------------------------------------------
		0b000000001	=	send to node 0		[USART C0,	P1/P1b		(bottom-left)	]
		0b000000010	=	send to node 1		[USART C1,	P2/P2b		(bottom-right)	]
		0b000000100	=	send to node 2		[USART D0,	P3/P3b		(left-top)		]
		0b000001000	=	send to node 3		[USART D1,	P4/P4b		(left-bottom)	]
		0b000010000	=	send to node 4		[USART E0,	P5/P5b		(right-bottom)	]
		0b000100000	=	send to node 5		[USART E1,	P6/P6b		(right-top)		]

		note: connector H7 is not treated as a node by xgrid
	*/
}

void swarm_calculation()
{
	//int check;	// unused
	// dvx, dvy delta velocity
	// lx, ly current position
	// ds ?
	// fx, fy ?
	float dvx, dvy, lx, ly, vabs, ds, fx, fy;
	// dir new direction
	float dir = mchip.hd;
	// cvx, cvy new velocity (?)
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
/**
 * i, j current position
 * nei neighbor index
 */
void swarm_interaction(int i, int j, int nei)
{
	// dirx, diry direction (?)
	// disx, disy ?
	// dis1
	// dis2
	// alph
	// force
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

void servo_motor_control()
{
	float servo_pos_flt;
	
	// Angle of axis Aa is calculated by
	// Aa = 2.0 * cos(mchip.hd)

	// This is just an example. There are many ways to translate
	// virtual agent's movement to the servo's angle.

	// mchip.hd (range: 0 - 2*PI)

	//servo_pos_flt = (mchip.hd-PI)*RADtoDEG;
	
	servo_pos_flt = 90*cos(mchip.hd);		// x = r*cos(theta)
	
	if(print_servo_info)
		fprintf_P(&usart_stream, PSTR("hd: %i.%i, deg: %i.%i\r\n"), prt_flt3(mchip.hd), prt_flt3(servo_pos_flt));
		//fprintf_P(&usart_stream, PSTR("hd: %f, deg: %f\r\n"), mchip.hd, servo_pos_flt);

	set_servo_position(servo_pos_flt);

}

// END KEN'S CODE	============================================================================================


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
                j = jiffies + 20;	// from here, you have 20 ms to reach the bottom of this loop
				
				if (usart.available())
				{
					input_char = usart.get();
				}	
							
				// main loop
                if (input_char == 0x1b)
                        xboot_reset();
				
				else if(input_char != 0x00)
				{
					fprintf_P(&usart_stream, PSTR("CPU: %c\r\n"), input_char);
					
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
					
					else if(input_char == 's')
					{
						if(print_servo_info)
							print_servo_info = false;
						else
							print_servo_info = true;
					}
					
					else if(input_char == 'v')
					{
						fprintf_P(&usart_stream, PSTR("avr-xgrid build %ld\r\n"), (unsigned long) &__BUILD_NUMBER);
					}
					
					else if(input_char == 'y')
					{
						LED_PORT.OUTTGL = LED_USR_1_PIN_bm;		// toggle yellow LED
					}
					
					input_char = 0x00;	// clear the most recent computer input
				}
				
				// KEN'S CODE:
				
				//swarm_communication();	// moved to ISR(TCC0_OVF_vect) where it is executed every 100 ms
				//swarm_calculation();		// moved to ISR(TCC0_OVF_vect) where it is executed every 100 ms
				//servo_motor_control();
				
		
                // END KEN'S CODE
				
				if(jiffies > j)	// if TRUE, then we took too long to get here, halt program
				{
					cli();	//disable_interrupts
					LED_PORT.OUTSET &= ~LED_USR_2_PIN_bm;	// turn green light off and keep it off
					LED_PORT.OUTSET = LED_USR_0_PIN_bm;		// turn red light on and keep it on
					LED_PORT.OUTSET = LED_USR_1_PIN_bm;		// turn yellow light on and keep it on
					while(1==1)
					{};
				}					
				
                while (j > jiffies) { };
        }  
}


