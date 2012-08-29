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

#define BOTTOM_LEFT 1	
#define BOTTOM_RIGHT 0  
#define LEFT_TOP 3      
#define LEFT_BOTTOM 2   
#define RIGHT_BOTTOM 4  
#define RIGHT_TOP 5    

// ============================================================================================
// prototype declaration
// ============================================================================================
void swarm_initialization1(void);
void swarm_calculation1(void);
void swarm_interaction1(int);

void communication(void);
void servo_motor_control(float);
// ============================================================================================

bool communication_on = false;
bool calculation_on = false;
bool servo_motor_on = false;
bool sensor = false;

volatile unsigned long jiffies = 0;

uint8_t program_num = 1;
uint8_t servo_tick = 0;

struct OBJ{

	float px, py, vx, vy, hd, hd_diff, hd_memo;
	float tim1, tim2;
	float neix[NUM_NEIGHBORS], neiy[NUM_NEIGHBORS];
	bool open[NUM_NEIGHBORS];
	bool flag;
} mchip;

struct point {
	float x, y;
	uint8_t pnum;
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
		program_num = pt_ptr->pnum;
			
		LED_PORT.OUTTGL = LED_USR_2_PIN_bm;	//green LED
}

// ============================================================================================
// Timer tick ISR (1 kHz)
// ============================================================================================
ISR(TCC0_OVF_vect)
{
        jiffies++;	// Timers

		if(jiffies%20 == 0) servo_motor_on = true;

		if(jiffies%100 ==0) servo_tick++;
		
		if(jiffies%500 == 0){
			communication_on = true;
			calculation_on = true;
		}

	    xgrid.process();
}

// ============================================================================================
// MAIN FUNCTION
// ============================================================================================
int main(void)
{      
	float angle;
	char input_char;
	
	_delay_ms(50);
       
    init();

    xgrid.rx_pkt = &rx_pkt;

    LED_PORT.OUT = LED_USR_0_PIN_bm;

	swarm_initialization2();

	while (1){		

		if (usart.available()) input_char = usart.get();
							
		// main loop
		if (input_char == 0x1b) xboot_reset();
		if (input_char == 's') sensor = true;
		if (input_char == ' ') sensor = false;

		if(communication_on){
			communication();
			communication_on = false;
		}

		if(calculation_on){
			/*
			mchip.hd_memo = mchip.hd;
			swarm_calculation1();
			mchip.hd_diff = (mchip.hd - mchip.hd_memo) / 5.0;	//divide into 5 step
			*/
			servo_tick = 0;
			swarm_calculation2();
			calculation_on = false;
		}

		if(servo_motor_on){
			//angle = mchip.hd_memo + (float)servo_tick * mchip.hd_diff;
			//servo_motor_control(angle);
			//servo_motor_on = false;
		}
	}

	return 0;
}

// ============================================================================================
// send messege
// ============================================================================================
void communication()
{
	mdata.x = mchip.px;
	mdata.y = mchip.py;
	mdata.pnum = program_num;

	Xgrid::Packet pkt;
	pkt.type = 0;
	pkt.flags = 0;	
	pkt.radius = 1;
	pkt.data = (uint8_t *)&mdata;
	pkt.data_len = sizeof(point);
									
	xgrid.send_packet(&pkt,0b00111111);		// send to all neighbors
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

	mchip.tim1 = 3.0*PI;
	mchip.tim2 = 3.0*PI;
	mchip.flag = false;
}

// --------------------------------------------------------------------------------------------
void swarm_calculation2()
{
	int i;
	float sum=0, vals, valc, th=0.1, alpha, angle;

	for(i=0;i<NUM_NEIGHBORS;i++){
		if(mchip.open[i]) sum += mchip.neix[i];
	}

	if(sensor) sum=1;

	if(sum != 0 && mchip.flag == false){
		mchip.tim1 = 0; 
		mchip.tim2 = 0; 
		mchip.flag = true;
	}

	if(mchip.tim2<=1.0*PI) mchip.tim1 += 0.1; 
	else mchip.tim1=0;
	
	mchip.tim2+=0.01;
	
	vals=sin(mchip.tim2);
	valc=cos(mchip.tim2);

	if(vals>0 && vals<th && valc<0 && mchip.flag) mchip.px=1.0; else mchip.px = 0.0;

	if(mchip.tim2>2.0*PI && vals>0 && valc<0) mchip.flag = false;

	if(mchip.tim2>=3.0*PI) mchip.tim2=3.0*PI;

	alpha=sin(mchip.tim2);
	if(alpha<0) alpha=0; 

	angle = (10.0 * alpha) * sin(mchip.tim1);

	servo_motor_control(angle);

}