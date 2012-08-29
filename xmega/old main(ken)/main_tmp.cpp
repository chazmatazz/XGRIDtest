#if PROGMEM_SIZE > 0x010000
#define PGM_READ_BYTE pgm_read_byte_far
#else
#define PGM_READ_BYTE pgm_read_byte_near
#endif

#define prt_flt3(f)	(int16_t)f,(int16_t)((fabs(f)-(int16_t)(fabs(f)))*1000)	

#include "main.h"
#include "board_init.c"
#include "sonar.c"

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

void send_message(void);
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
bool display = false;
bool restart_on = false;
bool speedup_on = false;
bool decay_on = false;
bool test = false;


volatile unsigned long jiffies = 0;

bool measure_on = false;
uint16_t sensor_value = 0;

uint8_t sensor_in[NUM_NEIGHBORS], sensor_oin[NUM_NEIGHBORS], sensor_out, sensor_data;

uint8_t my_counter = 1, my_program_number = 1, program_number;

uint8_t received_counter[NUM_NEIGHBORS];
uint8_t received_pnumber[NUM_NEIGHBORS];

bool connected[NUM_NEIGHBORS];

uint8_t HopStock[16];
uint8_t MesStock[16];
uint8_t StockCnt = 0;
uint8_t StockCnt4s = 0;

struct SNS{
	uint8_t hopcount, measuredata;
} SensorDataIn, SensorDataOut, SensorData[14];


struct OBJ1{
	float px, py, vx, vy, hd;
	float neix[NUM_NEIGHBORS], neiy[NUM_NEIGHBORS];
	int coeff;
} agent1;

struct OBJ2{
	float hd;
	float tim1, tim2;
	bool flag, sig_out, dir_out;
	bool dir[NUM_NEIGHBORS], sig[NUM_NEIGHBORS], osig[NUM_NEIGHBORS];
	int coeff;
} agent2, agent3;

struct point {
	float x, y;
	uint8_t pnum;				//4bit for count, 4bit for program num
	uint8_t signal;
	uint8_t hopCount, measureData;
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

// ============================================================================================
// Received Packet
// ============================================================================================
void rx_pkt(Xgrid::Packet *pkt)
{
		uint8_t MD;

		connected[pkt->rx_node] = true;

		point* pt_ptr = (point*) pkt->data;

		received_pnumber[pkt->rx_node] = (pt_ptr->pnum) & 0b00001111;
		received_counter[pkt->rx_node] = (pt_ptr->pnum) >> 4;

		agent1.neix[pkt->rx_node] = pt_ptr->x;
		agent1.neiy[pkt->rx_node] = pt_ptr->y;
	
		agent2.sig[pkt->rx_node] = ((pt_ptr->signal)&0b10000000) >> 7;

		agent3.sig[pkt->rx_node] = ((pt_ptr->signal)&0b01000000) >> 6;
		agent3.dir[pkt->rx_node] = ((pt_ptr->signal)&0b00100000) >> 5;

		if(connected[pkt->rx_node]){
			MD = pt_ptr->measureData;
			if(MD!=0){
				HopStock[StockCnt] = pt_ptr->hopCount;
				MesStock[StockCnt] = pt_ptr->measureData;
				StockCnt++; if(StockCnt>15) StockCnt=0;
			}
		}

		LED_PORT.OUTTGL = LED_USR_2_PIN_bm;	//green LED
}

// ============================================================================================
// send messege
// ============================================================================================
void send_message()
{
	mdata.x = agent1.px;
	mdata.y = agent1.py;

	mdata.pnum = (my_counter << 4) + program_number;

	mdata.signal = (agent2.sig_out << 7) + (agent3.sig_out << 6) + (agent3.dir_out << 5);

	mdata.hopCount = SensorDataOut.hopcount;
	mdata.measureData = SensorDataOut.measuredata;

	Xgrid::Packet pkt;
	pkt.type = 0;
	pkt.flags = 0;	
	pkt.radius = 1;
	pkt.data = (uint8_t *)&mdata;
	pkt.data_len = sizeof(point);
									
	xgrid.send_packet(&pkt, 0b00111111);	// send to all neighbors
	LED_PORT.OUTTGL = LED_USR_2_PIN_bm;		//green LED
}

// ============================================================================================
// Timer tick ISR (1 kHz)
// ============================================================================================
ISR(TCC0_OVF_vect)
{

		jiffies++;	// Timers

		// restarting status: if restart_on is set, board just sends reset signal every 500ms.
		if(restart_on)
		{
			if(jiffies%500 == 0) send_message();
		}
		// normal status: just setting bit flags, no function.
		else
		{
			if(jiffies%50  == 0) servo_motor_on   = true;

			if(jiffies%100 == 0)
			{
				measure_on = true;
				sendmessage_fast = true;	
				if(speedup_on) LED_PORT.OUTTGL = LED_USR_1_PIN_bm;
			}
		
			if(jiffies%500 == 0)
			{
				decay_on = true;
				sendmessage_slow = true;
				if(!speedup_on) LED_PORT.OUTTGL = LED_USR_1_PIN_bm;
			}
		}

		xgrid.process();
}

// ============================================================================================
// RESTART PROCESS
// ============================================================================================
void RESTART_PROCESS()
{
	unsigned long count = jiffies + 3000;

	mdata.x = 0.0; mdata.y = 0.0; 
	program_number = 0; my_counter = 0;

	restart_on = true;

	while(count > jiffies);
	
	xboot_reset();
}

// ============================================================================================
// KEY INPUT PROCESS
// ============================================================================================
void key_input()
{
	char input_char;
	int distance_sim;

	if (usart.available()) input_char = usart.get();
	
	if (input_char == 0x1b) RESTART_PROCESS();	// ESC key restarts the board

	distance_sim = input_char - '0';
	if(distance_sim>0 && distance_sim<10){ sensor_value=distance_sim; my_counter++;}
	if(distance_sim==0){sensor_value=10; my_counter++; test=true;}
	
	switch(input_char)
	{
	case 'f':	speedup_on = true;	program_number = 7; my_counter++; 
				break;
	case 's':	speedup_on = false; program_number = 6;	my_counter++; 
				break;
	case 'z':	sensor = true;
				break;
	case 'x':	sensor = false;
				break;
	case 'd':	if(display) display = false; else display = true;
				break;

	//case '9': sensor_value=9; my_counter++; break; case '8': sensor_value=8; my_counter++; break; case '7': sensor_value=7; my_counter++; break;
	//case '6': sensor_value=6; my_counter++; break; case '5': sensor_value=5; my_counter++; break; case '4': sensor_value=4; my_counter++; break;
	//case '3': sensor_value=3; my_counter++; break; case '2': sensor_value=2; my_counter++; break; case '1': sensor_value=1; my_counter++; break;
	//case '0': sensor_value=10; my_counter++; break;
	}

	if(my_counter>15) my_counter = 1;

	input_char = 0;
}

// ============================================================================================
// MAIN FUNCTION
// ============================================================================================
int main(void)
{
	uint8_t counter, pnumber;
	uint16_t temp=0;
	bool flag;
	float angle, speed = 12.0;
	int i, mcount=0;

	_delay_ms(50);
       
    init();
	init_sonar();

	my_counter = 1;
	my_program_number = 1;
	program_number = 1;

	init_common();

    xgrid.rx_pkt = &rx_pkt;

	while (1)
	{
		/*
		key_input();

		if(measure_on){
			temp += get_sonar_value();
			mcount++;
			if(mcount>4){
				sensor_value = temp / mcount;
				temp = 0;
				mcount = 0;
				
				if(sensor_value < 1000){
					SensorDataOut.measuredata = sensor_value;
					SensorDataOut.hopcount = 15;
				}
				else{
					SensorDataOut.measuredata = MesStock[StockCnt4s];
					SensorDataOut.hopcount = HopStock[StockCnt4s]--;
					StockCnt4s++;
					if(StockCnt4s>15) StockCnt4s=0;
				}
			}
			measure_on = false;
		}
		*/
		display=true;
		if(display){
			if(sensor_value<1000) 
				fprintf_P(&usart_stream, PSTR("check\r\n"));//, sensor_value);
			else{
				for(i=0;i<8;i++) fprintf_P(&usart_stream, PSTR("%d(%d): "), MesStock[i], HopStock[i]);
				fprintf_P(&usart_stream, PSTR("\n\r"));
				for(i=8;i<16;i++) fprintf_P(&usart_stream, PSTR("%d(%d): "), MesStock[i], HopStock[i]);
				fprintf_P(&usart_stream, PSTR("\n\r\n\r"));
			}
			//fprintf_P(&usart_stream, PSTR("\n\r\n\r"));
		}

/*
		// ========== MESSAGE PASSING ==========
		for(i=0;i<NUM_NEIGHBORS;i++)
		{
			if(!restart_on && connected[i] && received_pnumber[i] == 0) RESTART_PROCESS();

			if(connected[i]){
				counter = received_counter[i];
				pnumber = received_pnumber[i];
				flag = false;
				if((counter > my_counter) && (counter - my_counter) < 5) flag = true;
				if((counter + 15 > my_counter ) && (counter + 15 - my_counter) < 5) flag = true;
				if(flag){
					my_counter = counter;
					switch(pnumber){
						case 6: speedup_on = false; break;
						case 7: speedup_on = true;  break;
					}
					program_number = pnumber;


				}
			}
		}
/*
if((speedup_on && sendmessage_fast) || (!speedup_on && sendmessage_slow))
{
send_message();
sendmessage_fast = false;
sendmessage_slow = false;
}
*/		

/*
		// ========== CALCULATION & SEND MESSAGE  ==========
		if((speedup_on && sendmessage_fast) || (!speedup_on && sendmessage_slow))
		{
			swarm_calculation1();
			swarm_calculation2();
			swarm_calculation3();

			// sensor 
			if(sensor_value == 0){				//no sensor
				for(i=0;i<NUM_NEIGHBORS;i++){
					if(sensor_in[i] != sensor_oin[i]) sensor_data = sensor_in[i];	//detect change
					sensor_oin[i] = sensor_in[i];
				}
			}
			else sensor_data = sensor_value;	//sensor

			sensor_out = sensor_data;

			send_message();
			sendmessage_fast = false;
			sendmessage_slow = false;

			if(display){
				for(i=0;i<6;i++){
					fprintf_P(&usart_stream, PSTR("%d "), agent3.sig[i]);
				}
				fprintf_P(&usart_stream, PSTR(": %d\n\r"), sensor_value);
			}
		}

		// ========== SERVO MOTOR CONTROL ==========
		if(servo_motor_on){

			if(sensor_data > 9) angle = 0;
			else
			{
				if(sensor_data > 6) angle = 90.0 * cos(agent1.hd * speed);
				else if(sensor_data < 4) angle = 90.0 * agent3.hd;
				else angle = 90.0 * agent2.hd;
			}

			set_servo_position(angle);
			
			servo_motor_on = false;
		}
*/
		// in order to refresh connection status
		for(i=0;i<NUM_NEIGHBORS;i++) connected[i] = false;
	}
	
	return 0;
}



// ============================================================================================
// for initialization of swarm dynamics function
// ============================================================================================
void init_common()
{
	int i;
	init_servo();

	for(i=0;i<NUM_NEIGHBORS;i++){
		agent1.neix[i] = 0.0;
		agent1.neiy[i] = 0.0;
		connected[i] = false;
		agent2.sig[i] = false;
		agent3.sig[i] = false;
		agent3.osig[i] = false;
		sensor_in[i] = 0;
		sensor_oin[i] = 0;
	}

	agent1.px = 2.7; agent1.py = 75.6;
	agent1.vx = 3.8; agent1.vy = 1.3;
	agent1.hd = 0.0;

	agent2.tim1 = 15.0;
	agent2.tim2 = 15.0;
	agent2.flag = false;

	agent3.tim1 = 0.0;
	agent3.tim2 = 10.0;
	agent3.flag = false;

}

// ============================================================================================
// ============================================================================================
// SWARM DYNAMICS 1 --- Ken's Swarm Dynamics ---
// ============================================================================================
// ============================================================================================
void swarm_calculation1()
{
	int i;
	float dvx, dvy, lx, ly, vabs, ds, fx, fy, tmp;
	float dir = agent1.hd;
	float cvx = agent1.vx;
	float cvy = agent1.vy;

	// ===== calculation of forces =====
	//self-propel force and viscosity
	dvx = acc * cos(dir) - gmma * cvx;
	dvy = acc * sin(dir) - gmma * cvy;

	//interaction force with 6 neighbors
	forcex=0; forcey=0;
	for(i=0;i<NUM_NEIGHBORS;i++)
	{
		if(connected[i]) swarm_interaction1(i);
	}

	dvx = dvx + forcex;
	dvy = dvy + forcey;

	//spring term (Zero-length spring). fixed point = (0,0)
	lx = - agent1.px;
	ly = - agent1.py;
	dvx = dvx + ka * lx;
	dvy = dvy + ka * ly;

	// ===== update =====
	//direction
	vabs = sqrt(cvx * cvx + cvy * cvy);
	fx = cvx / vabs;
	fy = cvy / vabs;
	ds = -1.0 / tau * (sin(dir)*fx-cos(dir)*fy); // get sin value by cross product

	agent1.hd += ds * dt;

	if(agent1.hd > 2.0 * PI)	agent1.hd -= 2.0 * PI;
	if(agent1.hd < 0 ) 			agent1.hd += 2.0 * PI;

	//velocity
	agent1.vx += dvx * dt;
	agent1.vy += dvy * dt;

	//position
	agent1.px += agent1.vx * dt;
	agent1.py += agent1.vy * dt;
}
// --------------------------------------------------------------------------------------------
void swarm_interaction1(int nei)
{
	float dirx, diry, disx, disy, dis1, dis2, alph, force;
	float di, dj;
	bool flag = true;

	dirx = cos(agent1.hd);
	diry = sin(agent1.hd);

	switch(nei)
	{
		case BOTTOM_RIGHT: di= 0.866; dj= 0.500; break;
		case BOTTOM_LEFT : di=-0.866; dj= 0.500; break;
		case LEFT_BOTTOM : di=-1.000; dj= 0.000; break;
		case LEFT_TOP    : di=-0.866; dj=-0.500; break;
		case RIGHT_BOTTOM: di= 1.000; dj= 0.000; break;
		case RIGHT_TOP   : di= 0.866; dj=-0.500; break;
		default: flag = false;
	}

	if(flag)
	{
		disx = agent1.neix[nei] + ld * di - agent1.px;
		disy = agent1.neiy[nei] + ld * dj - agent1.py;

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
void swarm_calculation2()
{
	int i;
	bool flg = false;
	float alpha, angle;

	for(i=0;i<NUM_NEIGHBORS;i++)
	{
		if(agent2.sig[i]) flg = true; 
	}

	if(sensor_value>=4 && sensor_value<=6) flg = true;

	if(flg && !agent2.flag){
		agent2.tim1 = 0; 
		agent2.tim2 = 0; 
		agent2.flag = true;
	}

	if(agent2.tim2 < 15.0){
		agent2.tim1 += 0.5; 	
		agent2.tim2 += 0.1;

		if(agent2.tim2 > 3.0 && agent2.tim2 < 3.5) agent2.sig_out = 1;
		else agent2.sig_out = 0;

		if(agent2.tim2 > 12.0) agent2.flag = false;

		alpha = exp(-0.5 * agent2.tim2);
		angle = alpha * sin(agent2.tim1);

		//indication of flag condition
		//if(mchip.flag)  LED_PORT.OUT |= LED_USR_0_PIN_bm;
		//else			LED_PORT.OUT &= !LED_USR_0_PIN_bm;
	}

	agent2.hd = angle;
}

// ============================================================================================
// ============================================================================================
// SWARM DYNAMICS 3 --- vertical sync ---
// ============================================================================================
// ============================================================================================
void swarm_calculation3()
{
	int i;
	float sum=0, alpha, angle = 0;
	float pth = 0.0;

	if(!agent3.flag){
		if(sensor_value >=1 && sensor_value <=3){
			agent3.tim2 = 0; agent3.dir_out = true;
		}

		if(agent3.sig[0] && !agent3.dir[0]){
			agent3.tim2 = 0; agent3.dir_out = true;
		}

		if(agent3.sig[1] && agent3.dir[1]){
			agent3.tim2 = 0; agent3.dir_out = false;
		}
	}

	// === to keep condition
	// sensor
	if(sensor_value >=1 && sensor_value <=3) agent3.flag = false;
	if(sensor_value >=4) agent3.flag = true;
	//no sensor
	if(sensor_value == 0)
	{
		if(!agent3.sig[0] && !agent3.sig[1]) agent3.flag = true;
	}

	if(agent3.tim1>6.283) agent3.tim1 -= 6.283; 

	if(agent3.tim2 < 10.0)
	{
		agent3.tim1 += 0.5; 	
		agent3.tim2 += 0.1;

		if(agent3.tim2 > pth && agent3.tim2 < pth + 2.0) agent3.sig_out = true;
		else agent3.sig_out = false;

		alpha = exp(-0.5 * agent3.tim2);
		angle = alpha * sin(agent3.tim1);

		if(agent3.tim2 >9.8){
			agent3.flag = false;
		}

		//indication of flag condition
		//if(mchip.flag)  LED_PORT.OUT |= LED_USR_0_PIN_bm;
		//else			LED_PORT.OUT &= !LED_USR_0_PIN_bm;
	}
	else agent3.flag = false;

	agent3.hd = angle;
}
