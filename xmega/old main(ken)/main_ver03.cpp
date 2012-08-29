#if PROGMEM_SIZE > 0x010000
#define PGM_READ_BYTE pgm_read_byte_far
#else
#define PGM_READ_BYTE pgm_read_byte_near
#endif

#define prt_flt3(f)	(int16_t)f,(int16_t)((fabs(f)-(int16_t)(fabs(f)))*1000)	

#include "main.h"
#include "board_init.c"

#define PI 3.14159
#define NUM_NEIGHBORS 6

#define BOTTOM_LEFT				1	
#define BOTTOM_RIGHT			0  
#define LEFT_TOP				3      
#define LEFT_BOTTOM				2   
#define RIGHT_BOTTOM			4  
#define RIGHT_TOP				5    

#define BOTTOM_LEFT_MASK		0b00000010
#define BOTTOM_RIGHT_MASK		0b00000001
#define LEFT_TOP_MASK			0b00001000
#define LEFT_BOTTOM_MASK		0b00000100
#define RIGHT_BOTTOM_MASK		0b00010000
#define RIGHT_TOP_MASK			0b00100000

// ============================================================================================
// prototype declaration
// ============================================================================================
void init_common(void);
void swarm_initialization1(void);
void swarm_calculation1(void);
void swarm_interaction1(int);

void send_message(uint8_t, uint8_t);
void servo_motor_control(float);

void swarm_initialization2(void);
void swarm_calculation2(void);

void swarm_initialization3(void);
void swarm_calculation3(void);
void RESTART_PROCESS(void);
// ============================================================================================

bool sendmessage_fast = false;
bool sendmessage_slow = false;
bool calculation_on = false;
bool servo_motor_on = false;
bool sensor = false;
bool osensor = false;
bool display;
bool restart_on = false;
bool speedup_on = false;

volatile unsigned long jiffies = 0;

uint8_t servo_tick = 0;

uint8_t my_counter = 1, my_program_number = 1, program_number;

uint8_t received_counter[NUM_NEIGHBORS];
uint8_t received_pnumber[NUM_NEIGHBORS];

int temp1, temp2;

struct OBJ{
	float px, py, vx, vy, hd, hd_diff, hd_memo;
	float tim1, tim2;
	float neix[NUM_NEIGHBORS], neiy[NUM_NEIGHBORS];
	float oneix[NUM_NEIGHBORS], oneiy[NUM_NEIGHBORS];
	bool open[NUM_NEIGHBORS];
	bool flag;
} mchip;

struct point {
	float x, y;
	uint8_t pnum;	//5bit for count, 3bit for program num
} mdata;

// ============================================================================================
// variables
// ============================================================================================
float dt = 0.05;		// time step for differential equation (no relation with real time)

float ld = 20.0;		// virtual distance between the nodes
float acc = 20.0;		// self-propelling force
float gmma =5.0;		// viscosity
float ka = 0.1;			// spring constant
float d = 0.5;			// strength of anisotropy
float cf = 100.0;		// strength of interaction with neighbors
float rc = 20.0;		// optimum distance between agents
float tau = 1.0;		// relaxation time of heading dynamics
float forcex, forcey;   // force

float ep = 0.2;
float dl = 0.5;
float alpha = 0.5;

// ============================================================================================
// Packet
// ============================================================================================
void rx_pkt(Xgrid::Packet *pkt)
{
		mchip.open[pkt->rx_node] = true;

		point* pt_ptr = (point*) pkt->data;
		mchip.neix[pkt->rx_node] = pt_ptr->x;
		mchip.neiy[pkt->rx_node] = pt_ptr->y;
		received_pnumber[pkt->rx_node] = (pt_ptr->pnum) & 0b00000111;
		received_counter[pkt->rx_node] = (pt_ptr->pnum) >> 3;

		LED_PORT.OUTTGL = LED_USR_2_PIN_bm;	//green LED
}

// ============================================================================================
// Timer tick ISR (1 kHz)
// ============================================================================================
ISR(TCC0_OVF_vect)
{
		int i;

		jiffies++;	// Timers

		// if restart_on is set, board just sends reset signal.
		if(restart_on){
			if(jiffies%500 == 0) send_message(1, 0b00111111);
		}
		else{

			if(jiffies%50  == 0)  servo_motor_on   = true;

			if(jiffies%100 == 0){
				sendmessage_fast = true;	
				if(speedup_on) LED_PORT.OUTTGL = LED_USR_1_PIN_bm;
				calculation_on = true;
			}
		
			if(jiffies%500 == 0){
				sendmessage_slow = true;
				if(!speedup_on) LED_PORT.OUTTGL = LED_USR_1_PIN_bm;
			}

			//if(jiffies%1000== 0)  LED_PORT.OUTTGL = LED_USR_1_PIN_bm;	//yellow LED
		}

		xgrid.process();
}

// ============================================================================================
// RESTART PROCESS
// ============================================================================================
void RESTART_PROCESS()
{
	unsigned long count = jiffies + 3000;

	restart_on = true;

	mdata.x = 0.0; mdata.y = 0.0; 
	program_number = 0; my_counter = 0;
	
	while(count > jiffies);
	
	xboot_reset();
}

// ============================================================================================
// KEY INPUT PROCESS
// ============================================================================================
void key_input()
{
	unsigned long count;
	char input_char;

	if (usart.available()) input_char = usart.get();
		
	if (input_char == 0x1b) RESTART_PROCESS();	// ESC key restarts the board
	
	// COMM SPEED CHANGE
	switch(input_char){
	case 'f':	speedup_on = true;	program_number = 7; my_counter++; 
				break;
	case 's':	speedup_on = false; program_number = 6;	my_counter++; 
				break;
	case 'z':	sensor = true; osensor = true;
				break;
	case 'x':	sensor = false;
				break;
	case 'd':	if(display) display = false; else display = true;
				break;
	case '1':	my_program_number = 1; program_number = 1; my_counter++; swarm_initialization1(); 
				break;
	case '2':	my_program_number = 2; program_number = 2; my_counter++; swarm_initialization2(); 
				break;
	case '3':	my_program_number = 3; program_number = 3; my_counter++; swarm_initialization3(); 
				break;
	}

	if(my_counter>31) my_counter = 1;

	input_char = 0;
}

// ============================================================================================
// MAIN FUNCTION
// ============================================================================================
int main(void)
{
	unsigned long count;		//same type as jiffies
	uint8_t counter, pnumber;
	bool flag;
	float angle, speed = 12.0;
	int i;

	_delay_ms(50);
       
    init();

	my_counter = 1;
	my_program_number = 1;
	program_number = 1;

	init_common();

    xgrid.rx_pkt = &rx_pkt;

    //LED_PORT.OUT = LED_USR_0_PIN_bm;

	while (1){

		key_input();

		// ========== MESSAGE PASSING ==========
		for(i=0;i<NUM_NEIGHBORS;i++){
			if(!restart_on && mchip.open[i] && received_pnumber[i] == 0) RESTART_PROCESS();
			if(mchip.open[i]){
				counter = received_counter[i];
				pnumber = received_pnumber[i];
				flag = false;
				if((counter > my_counter) && (counter - my_counter)<5) flag = true;
				if((counter + 31 > my_counter ) && (counter + 31 - my_counter)<5) flag = true;
				if(flag){
					my_counter = counter;
					switch(pnumber){
						case 7: speedup_on = true;  break;
						case 6: speedup_on = false; break;
						case 1: my_program_number = pnumber; swarm_initialization1(); break;
						case 2: my_program_number = pnumber; swarm_initialization2(); break;
						case 3: my_program_number = pnumber; swarm_initialization3(); break;
					}
					program_number = pnumber;
				}
			}
		}

		// ========== SEND MESSAGE ==========
		if((speedup_on && sendmessage_fast) || (!speedup_on && sendmessage_slow))
		{
			switch(my_program_number){
				case 1:	send_message(1, 0b00111111); break;
				case 2: send_message(1, 0b00111111); break;
				case 3:	if(mchip.py==0.0) send_message(1,0b00100000);
						if(mchip.py==1.0) send_message(1,0b00001000);
						break;
			}

			sendmessage_fast = false;
			sendmessage_slow = false;

			if(display) fprintf_P(&usart_stream, PSTR("pnum=%d: cnt=%d\n\r"), my_program_number, my_counter);
		}

		// ========== CALCULATION ==========
		if(calculation_on)
		{
			switch(my_program_number){
				case 1: swarm_calculation1(); break;
				case 2: swarm_calculation2(); break;
				case 3: swarm_calculation3(); break;
			}
			calculation_on = false;			
		}

		// ========== SERVO MOTOR CONTROL ==========
		if(servo_motor_on){
			switch(my_program_number){
				case 1: angle = 90.0 * cos(mchip.hd * speed); break;
				case 2: angle = 90.0 * mchip.hd; break;
				case 3: angle = 90.0 * mchip.hd; break;
			}

			set_servo_position(angle);
			
			servo_motor_on = false;
		}

		// in order to refresh connection status
		for(i=0;i<NUM_NEIGHBORS;i++) mchip.open[i] = false;
	}

	return 0;
}

// ============================================================================================
// send messege
// ============================================================================================
void send_message(uint8_t distance, uint8_t direction)
{
	mdata.x = mchip.px;
	mdata.y = mchip.py;
	mdata.pnum = (my_counter << 3) + program_number;

	Xgrid::Packet pkt;
	pkt.type = 0;
	pkt.flags = 0;	
	pkt.radius = distance;
	pkt.data = (uint8_t *)&mdata;
	pkt.data_len = sizeof(point);
									
	xgrid.send_packet(&pkt, direction);		// send to all neighbors
	LED_PORT.OUTTGL = LED_USR_2_PIN_bm;		//green LED
}

// ============================================================================================
// SERVO MOTOR
// ============================================================================================
void servo_motor_control(float angle)
{
	float servo_pos_flt, speed = 2.0;
	
	servo_pos_flt = 90*cos(angle * speed);

	set_servo_position(servo_pos_flt);
}

// ============================================================================================
// for initialization of swarm dynamics function
// ============================================================================================
void init_common()
{
	int i;
	init_servo();
	for(i=0;i<NUM_NEIGHBORS;i++){
		mchip.neix[i] = 0.0;
		mchip.neiy[i] = 0.0;
		mchip.oneix[i] = 0.0;
		mchip.open[i] = false;
	}
}

// ============================================================================================
// ============================================================================================
// SWARM DYNAMICS 1 --- Ken's Swarm Dynamics ---
// ============================================================================================
// ============================================================================================
void swarm_initialization1()
{
	init_common();

	mchip.px = 2.7;
	mchip.py = 75.6;
	mchip.vx = 3.8;
	mchip.vy = 1.3;
	mchip.hd = 0.0;
	mchip.hd_diff = 0;
}
// --------------------------------------------------------------------------------------------
void swarm_calculation1()
{
	int i;
	float dvx, dvy, lx, ly, vabs, ds, fx, fy, tmp;
	float dir = mchip.hd;
	float cvx = mchip.vx;
	float cvy = mchip.vy;

	// ===== calculation of forces =====
	//self-propel force and viscosity
	dvx = acc * cos(dir) - gmma * cvx;
	dvy = acc * sin(dir) - gmma * cvy;

	//interaction force with 6 neighbors
	forcex=0; forcey=0;
	for(i=0;i<6;i++){
		if(mchip.open[i]) swarm_interaction1(i);
	}

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
}
// --------------------------------------------------------------------------------------------
void swarm_interaction1(int nei)
{
	float dirx, diry, disx, disy, dis1, dis2, alph, force;
	float di, dj;
	bool flag=true;

	dirx = cos(mchip.hd);
	diry = sin(mchip.hd);

	switch(nei){
		case BOTTOM_RIGHT: di= 0.866; dj= 0.500; break;
		case BOTTOM_LEFT : di=-0.866; dj= 0.500; break;
		case LEFT_BOTTOM : di=-1.000; dj= 0.000; break;
		case LEFT_TOP    : di=-0.866; dj=-0.500; break;
		case RIGHT_BOTTOM: di= 1.000; dj= 0.000; break;
		case RIGHT_TOP   : di= 0.866; dj=-0.500; break;
		default: flag=false;
	}

	if(flag){
		disx = mchip.neix[nei] + ld * di - mchip.px;
		disy = mchip.neiy[nei] + ld * dj - mchip.py;

		dis2 = disx * disx + disy * disy;
		dis1 = sqrt(dis2);

		alph = 1.0 + d * (disx * dirx + disy * diry) / dis1; //inner product
		force = -cf * (rc / dis1 - 1.0) * rc * rc / dis2;
		forcex += alph * force * disx / dis1;
		forcey += alph * force * disy / dis1;
	}
}

// ============================================================================================
// ============================================================================================
// SWARM DYNAMICS 2 --- propagation ---
// ============================================================================================
// ============================================================================================
void swarm_initialization2()
{
	init_common();

	mchip.tim1 = 15.0;
	mchip.tim2 = 15.0;
	mchip.flag = false;
}

// --------------------------------------------------------------------------------------------
void swarm_calculation2()
{
	int i;
	float sum=0.0, alpha, angle;

	for(i=0;i<NUM_NEIGHBORS;i++){
		//if(mchip.open[i]) 
		sum += mchip.neix[i];
	}

	if(sensor) sum=1.0;

	if(sum != 0 && mchip.flag == false){
		mchip.tim1 = 0; 
		mchip.tim2 = 0; 
		mchip.flag = true;
	}

	if(mchip.tim2 < 15.0){
		mchip.tim1 += 0.5; 	
		mchip.tim2 += 0.1;

		if(mchip.tim2 > 3.0 && mchip.tim2 < 3.5) mchip.px = 1.0;
		else mchip.px = 0.0;

		if(mchip.tim2 > 12.0) mchip.flag = false;

		alpha = exp(-0.5 * mchip.tim2);
		angle = alpha * sin(mchip.tim1);

		//indication of flag condition
		if(mchip.flag)  LED_PORT.OUT |= LED_USR_0_PIN_bm;
		else			LED_PORT.OUT &= !LED_USR_0_PIN_bm;
	}

	mchip.hd = angle;
}

// ============================================================================================
// ============================================================================================
// SWARM DYNAMICS 3 --- vertical sync ---
// ============================================================================================
// ============================================================================================
void swarm_initialization3()
{
	init_common();

	mchip.tim1 = 0.0;
	mchip.tim2 = 10.0;
	mchip.flag = false;
}

// --------------------------------------------------------------------------------------------
void swarm_calculation3()
{
	int i;
	float sum=0, alpha, angle = 0;
	float pth = 0.0;

	if(!mchip.flag){
		if(sensor){
			mchip.tim2 = 0; mchip.py = 1.0; //mchip.flag = true;
		}

		if(mchip.neix[0]==1.0){
			mchip.tim2 = 0; mchip.py = 0.0; //mchip.flag = true;
		}

		if(mchip.neix[1]==1.0){
			mchip.tim2 = 0; mchip.py = 1.0; //mchip.flag = true;
		}
	}

	// === to keep condition
	if(osensor!=sensor) mchip.flag = true;
	if(mchip.oneix[0]==1.0 && mchip.neix[0]==0.0) mchip.flag = true;
	if(mchip.oneix[1]==1.0 && mchip.neix[1]==0.0) mchip.flag = true;
	osensor = sensor;
	mchip.oneix[0] = mchip.neix[0];
	mchip.oneix[1] = mchip.neix[1];
	// ===

	if(mchip.tim1>6.283) mchip.tim1 -= 6.283; 

	if(mchip.tim2 < 10.0){
		mchip.tim1 += 0.5; 	
		mchip.tim2 += 0.1;

		if(mchip.tim2 > pth && mchip.tim2 < pth + 2.0) mchip.px = 1.0;
		else mchip.px = 0.0;

		alpha = exp(-0.5 * mchip.tim2);
		angle = alpha * sin(mchip.tim1);

		if(mchip.tim2 >9.8){
			mchip.flag = false;
		}

		//indication of flag condition
		if(mchip.flag)  LED_PORT.OUT |= LED_USR_0_PIN_bm;
		else			LED_PORT.OUT &= !LED_USR_0_PIN_bm;
	}
	else mchip.flag = false;

	mchip.hd = angle;
}
