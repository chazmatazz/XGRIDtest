

#define prt_flt3(f)	(int16_t)f,(int16_t)((fabs(f)-(int16_t)(fabs(f)))*1000)			// use %i.%i inside print_f in place of %f

#if PROGMEM_SIZE > 0x010000
#define PGM_READ_BYTE pgm_read_byte_far
#else
#define PGM_READ_BYTE pgm_read_byte_near
#endif

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
void normalize_2D(float &x, float &y);
void get_x_y_from_rand(float &x, float &y, int rand1, int rand2);
void get_new_rands(int& rand1, int& rand2);

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
        
        if (jiffies % 50 == 0)
		{
			LED_PORT.OUTTGL = LED_USR_0_PIN_bm;		// toggle red light every 50 ms (20 Hz)
			
			//swarm_calculation();
			//servo_motor_control();
		}		
		
		if (jiffies % 100 == 0)
		{
			//swarm_communication();
			//swarm_calculation();		// looks for new commands	
			servo_motor_control();
		}		
       
		if (jiffies % 200 == 0)
		{
			//swarm_communication();
			swarm_calculation();		// looks for new commands	
			//servo_motor_control();
		}
		
		if (jiffies % 500 == 0)
		{
			swarm_communication();
			//swarm_calculation();		// looks for new commands	
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




#define MESSAGE_TYPE_XY 0
#define MESSAGE_TYPE_COMMAND 1

#define SWARM_MODE_RANDOM 0
//#define SWARM_MODE_AVERAGE 1
#define SWARM_MODE_ORIGIN 2
#define SWARM_MODE_MIRROR_MASTER 3
#define SWARM_MODE_MIRROR_SLAVE 4
#define SWARM_MODE_LINE_MASTER 5
#define SWARM_MODE_LINE_SLAVE 6

//#define INIT_MODE SWARM_MODE_RANDOM
#define INIT_MODE SWARM_MODE_ORIGIN

int i, j, t;	// moved from swarm_initialization() to global

#define PI 3.14159
#define RADtoDEG 57.29578
#define delayX 10
#define SIZEX 10
#define SIZEY 10
#define NUM_NEIGHBORS 6

#define BOTTOM_LEFT 1	// was 0
#define BOTTOM_RIGHT 0  // was 1
#define LEFT_TOP 3      // was 2
#define LEFT_BOTTOM 2   // was 3
#define RIGHT_BOTTOM 4  // was 4
#define RIGHT_TOP 5     // was 5

#define BOTTOM_LEFT_MASK		0b00000010 // old is 0b00000001
#define BOTTOM_RIGHT_MASK		0b00000001 // old is 0b00000010
#define LEFT_TOP_MASK			0b00001000 // old is 0b00000100
#define LEFT_BOTTOM_MASK		0b00000100 // old is 0b00001000
#define RIGHT_BOTTOM_MASK		0b00010000 // old is 0b00010000
#define RIGHT_TOP_MASK			0b00100000 // old is 0b00100000

float theta;
float my_x, my_y;
int swarm_id;
int rand1, rand2;
int swarm_mode;

bool command_received;
char command;
uint8_t command_source;
uint8_t message_distance;

bool print_servo_info = false;
bool update_allowed = false;

uint32_t message_count[NUM_NEIGHBORS];
uint8_t line_direction;
uint8_t send_mask;
uint8_t receive_mask;	// NOT USED

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
		LED_PORT.OUTTGL = LED_USR_2_PIN_bm;		// toggle green light when receive packet
		
		if((pkt->rx_node >=0)&&(pkt->rx_node < NUM_NEIGHBORS))
			message_count[pkt->rx_node]++;
		else
			fprintf_P(&usart_stream, PSTR("UNKNOWN packet source: %i\n\r"), pkt->rx_node);
		
		if(pkt->type == MESSAGE_TYPE_XY)
		{
			point* pt_ptr = (point*) pkt->data;
			//mchip.neix[pkt->source_id] = pt_ptr->x;
			//mchip.neiy[pkt->source_id] = pt_ptr->y;
			
			if(swarm_mode == SWARM_MODE_MIRROR_SLAVE)
			{
				my_x += pt_ptr->x;
				my_y += pt_ptr->y;
				normalize_2D(my_x,my_y);
			}
		
			else if(swarm_mode == SWARM_MODE_LINE_SLAVE)
			{
				fprintf_P(&usart_stream, PSTR("packet source: "));
				switch(pkt->rx_node)
				{
					case BOTTOM_LEFT:	fprintf_P(&usart_stream, PSTR("BOTTOM LEFT\n\r"));	break;
					case BOTTOM_RIGHT:	fprintf_P(&usart_stream, PSTR("BOTTOM RIGHT\n\r"));	break;
					case LEFT_TOP:		fprintf_P(&usart_stream, PSTR("LEFT TOP\n\r"));		break;
					case LEFT_BOTTOM:	fprintf_P(&usart_stream, PSTR("LEFT BOTTOM\n\r"));	break;
					case RIGHT_BOTTOM:	fprintf_P(&usart_stream, PSTR("RIGHT BOTTOM\n\r"));	break;
					case RIGHT_TOP:		fprintf_P(&usart_stream, PSTR("RIGHT TOP\n\r"));	break;
					default:			fprintf_P(&usart_stream, PSTR("UNKNOWN\n\r"));		break;
				}
				
				fprintf_P(&usart_stream, PSTR("contents x: %i.%i, y: %i.%i\n\r"), prt_flt3(pt_ptr->x), prt_flt3(pt_ptr->y));	
				
				if(pkt->rx_node == command_source)
				{
					my_x = pt_ptr->x;
					my_y = pt_ptr->y;
					fprintf_P(&usart_stream, PSTR("ACCEPTED "));
				}
				fprintf_P(&usart_stream, PSTR("my_x: %i.%i, my_y: %i.%i\n\r"), prt_flt3(my_x), prt_flt3(my_y));
			}
		}
		
		else if(pkt->type == MESSAGE_TYPE_COMMAND)
		{
			char* char_ptr = (char*) pkt->data;
			command_received = true;
			command = *char_ptr;
			command_source = pkt->rx_node;
			
			fprintf_P(&usart_stream, PSTR("command source: %i "), command_source);
			switch(command_source)
			{
				case BOTTOM_LEFT:	fprintf_P(&usart_stream, PSTR("BOTTOM LEFT\n\r"));	break;
				case BOTTOM_RIGHT:	fprintf_P(&usart_stream, PSTR("BOTTOM RIGHT\n\r"));	break;
				case LEFT_TOP:		fprintf_P(&usart_stream, PSTR("LEFT TOP\n\r"));		break;
				case LEFT_BOTTOM:	fprintf_P(&usart_stream, PSTR("LEFT BOTTOM\n\r"));	break;
				case RIGHT_BOTTOM:	fprintf_P(&usart_stream, PSTR("RIGHT BOTTOM\n\r"));	break;
				case RIGHT_TOP:		fprintf_P(&usart_stream, PSTR("RIGHT TOP\n\r"));	break;
				default:			fprintf_P(&usart_stream, PSTR("UNKNOWN\n\r"));		break;
			}
		}
		
		else
		{
			fprintf_P(&usart_stream, PSTR("UNKNOWN packet type received: %i\n\r"), pkt->type);
		}
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
	
	uint32_t b = 0;
	uint32_t crc = 0;
	
	// calculate local id
	// simply crc of user sig row
	// likely to be unique and constant for each chip
	NVM_CMD = NVM_CMD_READ_CALIB_ROW_gc;
	
	for (uint32_t i = 0x08; i <= 0x15; i++)
	{
			b = PGM_READ_BYTE(i);
			//fprintf_P(&usart_stream, PSTR("%i:%i\r\n"),i,b);
			crc = _crc16_update(crc, b);
	}
	
	 NVM_CMD = NVM_CMD_NO_OPERATION_gc;
	
	swarm_id = crc;
	
	if(swarm_id < 0)
		swarm_id*=-1;
	
	srand(swarm_id);
	
	//NEW
	get_new_rands(rand1, rand2);
	
	if(INIT_MODE == SWARM_MODE_RANDOM)
	{
		swarm_mode = SWARM_MODE_RANDOM;
		
		get_x_y_from_rand(my_x, my_y, rand1, rand2);
	}
	
	else /*if(INIT_MODE == SWARM_MODE_ORIGIN)*/
	{
		swarm_mode = SWARM_MODE_ORIGIN;
		
		my_x = 0;
		my_y = 1;
	}
	
	// blindly assume my neighbors have the same value that I do
	for(t=0;t<NUM_NEIGHBORS;t++)
	{
		mchip.neix[t]=my_x;
		mchip.neiy[t]=my_y;
		message_count[t]=0;
	}
	
	command_received = false;
	
	message_distance = 1;
	line_direction = 0;
}

/*
 * Just sending
 */
void swarm_communication()
{
	//mdata.x = mchip.vpx;
	//mdata.y = mchip.vpy;
	
	//NEW
	mdata.x = my_x;
	mdata.y = my_y;
	
	Xgrid::Packet pkt;
	pkt.type = MESSAGE_TYPE_XY;
	pkt.flags = 0;	
	pkt.radius = 1;
	pkt.data = (uint8_t *)&mdata;
	pkt.data_len = sizeof(point);
	
	if(swarm_mode == SWARM_MODE_LINE_SLAVE)
		xgrid.send_packet(&pkt, send_mask);
	else
		xgrid.send_packet(&pkt);
	
}

void swarm_calculation()
{
	if(command_received)
	{
		if(command == 'O')
		{
			swarm_mode = SWARM_MODE_ORIGIN;
			my_x = 0;
			my_y = 1;
			fprintf_P(&usart_stream, PSTR("GOTO origin my_x: %i.%i, my_y: %i.%i\n\r"), prt_flt3(my_x), prt_flt3(my_y));		
		}
		
		else if(command == 'R')
		{
			swarm_mode = SWARM_MODE_RANDOM;
			get_x_y_from_rand(my_x, my_y, rand1, rand2);
			fprintf_P(&usart_stream, PSTR("GOTO random my_x: %i.%i, my_y: %i.%i\n\r"), prt_flt3(my_x), prt_flt3(my_y));
			fprintf_P(&usart_stream, PSTR("(rand1: %i, rand2: %i)\n\r"), rand1, rand2);
		}
		
		else if(command == 'M')
		{
			swarm_mode = SWARM_MODE_MIRROR_SLAVE;
			fprintf_P(&usart_stream, PSTR("GOTO mirror slave\n\r"));
		}
		
		else if(command == 'L')
		{
			swarm_mode = SWARM_MODE_LINE_SLAVE;
			fprintf_P(&usart_stream, PSTR("GOTO line slave\n\r"));
			
			if(command_source == BOTTOM_LEFT)			{send_mask = RIGHT_TOP_MASK; line_direction = RIGHT_TOP;}
			else if(command_source == BOTTOM_RIGHT)		{send_mask = LEFT_TOP_MASK; line_direction = LEFT_TOP;}
			else if(command_source == LEFT_TOP)			{send_mask = BOTTOM_RIGHT_MASK; line_direction = BOTTOM_RIGHT;}
			else if(command_source == LEFT_BOTTOM)		{send_mask = RIGHT_BOTTOM_MASK; line_direction = RIGHT_BOTTOM;}
			else if(command_source == RIGHT_BOTTOM)		{send_mask = LEFT_BOTTOM_MASK; line_direction = LEFT_BOTTOM;}
			else if(command_source == RIGHT_TOP)		{send_mask = BOTTOM_LEFT_MASK; line_direction = BOTTOM_LEFT;}
		
			fprintf_P(&usart_stream, PSTR("line direction: "));
			switch(line_direction)
			{
				case BOTTOM_LEFT:	fprintf_P(&usart_stream, PSTR("BOTTOM LEFT\n\r"));	break;
				case BOTTOM_RIGHT:	fprintf_P(&usart_stream, PSTR("BOTTOM RIGHT\n\r"));	break;
				case LEFT_TOP:		fprintf_P(&usart_stream, PSTR("LEFT TOP\n\r"));		break;
				case LEFT_BOTTOM:	fprintf_P(&usart_stream, PSTR("LEFT BOTTOM\n\r"));	break;
				case RIGHT_BOTTOM:	fprintf_P(&usart_stream, PSTR("RIGHT BOTTOM\n\r"));	break;
				case RIGHT_TOP:		fprintf_P(&usart_stream, PSTR("RIGHT TOP\n\r"));	break;
				default:			fprintf_P(&usart_stream, PSTR("UNKNOWN\n\r"));		break;
			}
		}
		
		else
		{
			fprintf_P(&usart_stream, PSTR("UNKNOWN command received: %c\n\r"), command);
		}

		command_received = false;
	}

	// ===== calculation of forces =====
	
	//fprintf_P(&usart_stream, PSTR("#my_x: %i.%i, my_y: %i.%i\r\n"), prt_flt3(my_x), prt_flt3(my_y));

	// ===== update =====
	if(update_allowed)
	{
		float new_x = 0;
		float new_y = 0;
	
		for(t=0;t<NUM_NEIGHBORS;t++)
		{
			new_x += mchip.neix[t];
			new_y += mchip.neiy[t];
		}
	
		my_x += new_x/NUM_NEIGHBORS;
		my_y += new_y/NUM_NEIGHBORS;
	}
	
	normalize_2D(my_x, my_y);
	
	theta = atan2(my_y,my_x);
}

//==============================================
/**
 * i, j current position
 * nei neighbor index
 */
void swarm_interaction(int i, int j, int nei)
{
	// how are boundary conditions used?
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
	
	theta = atan2(my_y,my_x);
	
	servo_pos_flt = 90*cos(theta);
	
	if(print_servo_info)
		fprintf_P(&usart_stream, PSTR("theta: %i.%i, deg: %i.%i, my_x: %i, my_y: %i\n\r"), prt_flt3(theta), prt_flt3(servo_pos_flt), my_x, my_y);

	set_servo_position(servo_pos_flt);
}

// END KEN'S CODE	============================================================================================

void normalize_2D(float &x, float &y)
{
	float len = sqrt(x*x + y*y);
	
	x = x/len;
	y = y/len;
}

void get_x_y_from_rand(float &x, float &y, int rand1, int rand2)
{
	//x = (-4+(rand1%7))*1.0;
	//y = (-4+(rand2%7))*1.0;
	
	x = -0.5 + (float)rand1/(float)RAND_MAX;
	y = -0.5 + (float)rand2/(float)RAND_MAX;
	
	normalize_2D(x, y);
}

void get_new_rands(int& rand1, int& rand2)
{
	rand1 = rand();
	rand2 = rand();
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
		
		swarm_initialization();
		
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
					
					if (input_char == 'c')
					{
						fprintf_P(&usart_stream, PSTR("message count:\r\n"));
						fprintf_P(&usart_stream, PSTR("0 [BOTTOM LEFT] : %i\r\n"),message_count[0]);
						LED_PORT.OUTTGL = LED_USR_1_PIN_bm;
						_delay_ms(1);
						fprintf_P(&usart_stream, PSTR("1 [BOTTOM RIGHT]: %i\r\n"),message_count[1]);
						LED_PORT.OUTTGL = LED_USR_1_PIN_bm;
						_delay_ms(1);
						fprintf_P(&usart_stream, PSTR("2 [LEFT TOP]	   : %i\r\n"),message_count[2]);
						LED_PORT.OUTTGL = LED_USR_1_PIN_bm;
						_delay_ms(1);
						fprintf_P(&usart_stream, PSTR("3 [LEFT BOTTOM] : %i\r\n"),message_count[3]);
						LED_PORT.OUTTGL = LED_USR_1_PIN_bm;
						_delay_ms(1);
						fprintf_P(&usart_stream, PSTR("4 [RIGHT BOTTOM]: %i\r\n"),message_count[4]);
						LED_PORT.OUTTGL = LED_USR_1_PIN_bm;
						_delay_ms(1);
						fprintf_P(&usart_stream, PSTR("5 [RIGHT TOP]   : %i\r\n"),message_count[5]);
						LED_PORT.OUTTGL = LED_USR_1_PIN_bm;
						_delay_ms(1);
					}
					
					if (input_char == 'd')
					{
						message_distance++;
						
						if(message_distance > 7)
							message_distance = 1;
						
						fprintf_P(&usart_stream, PSTR("message distance:%i\n\r"), message_distance);
					}

					else if (input_char == 'i')
					{
						//fprintf_P(&usart_stream, PSTR("my info:\r\n"));
						
						fprintf_P(&usart_stream, PSTR("swarm_id:%i\r\n"),swarm_id);
					}
					
					else if (input_char == 'j')
					{
						fprintf_P(&usart_stream, PSTR("my calibration byte:\r\n"));
						fprintf_P(&usart_stream, PSTR("0:%i\r\n"),SP_ReadCalibrationByte(0));
						fprintf_P(&usart_stream, PSTR("1:%i\r\n"),SP_ReadCalibrationByte(1));
						fprintf_P(&usart_stream, PSTR("2:%i\r\n"),SP_ReadCalibrationByte(2));
						fprintf_P(&usart_stream, PSTR("3:%i\r\n"),SP_ReadCalibrationByte(3));
						fprintf_P(&usart_stream, PSTR("4:%i\r\n"),SP_ReadCalibrationByte(4));
					}
					
					else if(input_char == 'l')
					{
						line_direction++;
						
						if(line_direction >= NUM_NEIGHBORS)
							line_direction = 0;
						
						fprintf_P(&usart_stream, PSTR("line direction: "));
						switch(line_direction)
						{
							case BOTTOM_LEFT:	fprintf_P(&usart_stream, PSTR("BOTTOM LEFT\n\r"));	break;
							case BOTTOM_RIGHT:	fprintf_P(&usart_stream, PSTR("BOTTOM RIGHT\n\r"));	break;
							case LEFT_TOP:		fprintf_P(&usart_stream, PSTR("LEFT TOP\n\r"));		break;
							case LEFT_BOTTOM:	fprintf_P(&usart_stream, PSTR("LEFT BOTTOM\n\r"));	break;
							case RIGHT_BOTTOM:	fprintf_P(&usart_stream, PSTR("RIGHT BOTTOM\n\r"));	break;
							case RIGHT_TOP:		fprintf_P(&usart_stream, PSTR("RIGHT TOP\n\r"));	break;
							default:			fprintf_P(&usart_stream, PSTR("UNKNOWN\n\r"));		break;
						}
					}
					
					else if(input_char == 'L')
					{
						swarm_mode = SWARM_MODE_LINE_MASTER;
						fprintf_P(&usart_stream, PSTR("BECAME line MASTER, direction: "));
						
						switch(line_direction)
						{
							case BOTTOM_LEFT:	/*command_source = RIGHT_TOP;*/		send_mask = BOTTOM_LEFT_MASK;	fprintf_P(&usart_stream, PSTR("BOTTOM LEFT\n\r"));	break;
							case BOTTOM_RIGHT:	/*command_source = LEFT_TOP;*/ 		send_mask = BOTTOM_RIGHT_MASK;	fprintf_P(&usart_stream, PSTR("BOTTOM RIGHT\n\r"));	break;
							case LEFT_TOP:		/*command_source = BOTTOM_RIGHT;*/ 	send_mask = LEFT_TOP_MASK;		fprintf_P(&usart_stream, PSTR("LEFT TOP\n\r"));		break;
							case LEFT_BOTTOM:	/*command_source = RIGHT_BOTTOM;*/ 	send_mask = LEFT_BOTTOM_MASK;	fprintf_P(&usart_stream, PSTR("LEFT BOTTOM\n\r"));	break;
							case RIGHT_BOTTOM:	/*command_source = LEFT_BOTTOM;*/ 	send_mask = RIGHT_BOTTOM_MASK;	fprintf_P(&usart_stream, PSTR("RIGHT BOTTOM\n\r"));	break;
							case RIGHT_TOP:		/*command_source = BOTTOM_LEFT;*/ 	send_mask = RIGHT_TOP_MASK;		fprintf_P(&usart_stream, PSTR("RIGHT TOP\n\r"));	break;
							default:			fprintf_P(&usart_stream, PSTR("UNKNOWN\n\r"));		break;
						}
						
						char str[] = "L";
                        Xgrid::Packet pkt;
                        pkt.type = MESSAGE_TYPE_COMMAND;
                        pkt.flags = 0;
                        pkt.radius = message_distance;
                        pkt.data = (uint8_t *)str;
                        pkt.data_len = 1;
                        
                        xgrid.send_packet(&pkt, send_mask);
					}
					
					else if(input_char == 'm')
					{
						fprintf_P(&usart_stream, PSTR("SWARM MODE: %i (0=rand, 1=merge, 2=origin, 3=Mmaster, 4=Mslave)\r\n"), swarm_mode);
					}
					
					else if(input_char == 'M')
					{
						swarm_mode = SWARM_MODE_MIRROR_MASTER;
						fprintf_P(&usart_stream, PSTR("BECAME mirror MASTER\n\r"));
						
						char str[] = "M";
                        Xgrid::Packet pkt;
                        pkt.type = MESSAGE_TYPE_COMMAND;
                        pkt.flags = 0;
                        pkt.radius = message_distance;
                        pkt.data = (uint8_t *)str;
                        pkt.data_len = 1;
                        
                        xgrid.send_packet(&pkt);
					}
					
					else if(input_char == 'o')
					{
						if(swarm_mode == SWARM_MODE_ORIGIN)
						{
							fprintf_P(&usart_stream, PSTR("AT origin my_x: %i.%i, my_y: %i.%i\n\r"), prt_flt3(my_x), prt_flt3(my_y));
						}
						
						else
						{
							swarm_mode = SWARM_MODE_ORIGIN;
							my_x = 0;
							my_y = 1;
							fprintf_P(&usart_stream, PSTR("GOTO origin my_x: %i.%i, my_y: %i.%i\n\r"), prt_flt3(my_x), prt_flt3(my_y));
						}
					}
					
					else if(input_char == 'O')
					{
						swarm_mode = SWARM_MODE_ORIGIN;
						my_x = 0;
						my_y = 1;
						fprintf_P(&usart_stream, PSTR("GOTO origin my_x: %i.%i, my_y: %i.%i\n\r"), prt_flt3(my_x), prt_flt3(my_y));
						
						char str[] = "O";
                        Xgrid::Packet pkt;
                        pkt.type = MESSAGE_TYPE_COMMAND;
                        pkt.flags = 0;
                        pkt.radius = message_distance;
                        pkt.data = (uint8_t *)str;
                        pkt.data_len = 1;
                        
                        xgrid.send_packet(&pkt);
					}
					
					else if(input_char == 'p')
					{
						fprintf_P(&usart_stream, PSTR("my_x: %i.%i, my_y: %i.%i, rand1: %i, rand2: %i\n\r"), prt_flt3(my_x), prt_flt3(my_y), rand1, rand2);
					}
					
					else if(input_char == 'r')
					{
						if(swarm_mode == SWARM_MODE_RANDOM)
						{
							fprintf_P(&usart_stream, PSTR("AT random my_x: %i.%i, my_y: %i.%i\n\r"), prt_flt3(my_x), prt_flt3(my_y));
							fprintf_P(&usart_stream, PSTR("(rand1: %i, rand2: %i)\n\r"), rand1, rand2);
						}
						
						else
						{
							swarm_mode = SWARM_MODE_RANDOM;
							get_x_y_from_rand(my_x, my_y, rand1, rand2);
							fprintf_P(&usart_stream, PSTR("GOTO random my_x: %i.%i, my_y: %i.%i\n\r"), prt_flt3(my_x), prt_flt3(my_y));
							fprintf_P(&usart_stream, PSTR("(rand1: %i, rand2: %i)\n\r"), rand1, rand2);
						}
					}
					
					else if(input_char == 'R')
					{
						swarm_mode = SWARM_MODE_RANDOM;
						get_x_y_from_rand(my_x, my_y, rand1, rand2);
						fprintf_P(&usart_stream, PSTR("GOTO random my_x: %i.%i, my_y: %i.%i\n\r"), prt_flt3(my_x), prt_flt3(my_y));
						fprintf_P(&usart_stream, PSTR("(rand1: %i, rand2: %i)\n\r"), rand1, rand2);
					
						char str[] = "R";
                        Xgrid::Packet pkt;
                        pkt.type = MESSAGE_TYPE_COMMAND;
                        pkt.flags = 0;
                        pkt.radius = message_distance;
                        pkt.data = (uint8_t *)str;
                        pkt.data_len = 1;
                        
                        xgrid.send_packet(&pkt);			
					}
					
					else if(input_char == 's')
					{
						if(print_servo_info)
							print_servo_info = false;
						else
							print_servo_info = true;
					}
					
					else if(input_char == 'u')
					{
						if(update_allowed)
							update_allowed = false;
						else
							update_allowed = true;
							
						if(update_allowed)
							fprintf_P(&usart_stream, PSTR("update allowed"));
						else
							fprintf_P(&usart_stream, PSTR("update NOT allowed"));
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


