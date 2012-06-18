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

#define RESTART_CODE 0x07
#define SPEED_UP 0x06
#define SPEED_DOWN 0x05
#define INIT_CODE 0x04
#define STOP_SEND 0x03
#define START_SEND 0x02

#define MESSAGE_COMMAND 0x0F
#define MESSAGE_DATA 0xE

// ============================================================================================
// prototype declaration
// ============================================================================================
void init_common(void);
void swarm_initialization1(void);
void swarm_calculation1(void);
void swarm_interaction1(int);

void send_message(int);
void servo_motor_control(float);

void swarm_initialization2(void);
void swarm_calculation2(void);

void swarm_initialization3(void);
void swarm_calculation3(void);
void sensing(void);

void RESTART_PROCESS(void);
// ============================================================================================

bool sendmessage_fast = false;
bool sendmessage_slow = false;
bool sendmessage_stop = false;
bool calculation_on = false;
bool servo_motor_on = false;

bool display = false;
bool restart_on = false;
bool speedup_on = false;

bool sonar_attached = false;
bool measure_on = false;

uint16_t measure_temp = 0;
uint8_t my_sensor_value=0, mcount = 0;
uint8_t neinum;

volatile unsigned long jiffies = 0;

uint8_t sensor_in[NUM_NEIGHBORS], sensor_oin[NUM_NEIGHBORS], sensor_out, sensor_data;

uint8_t my_counter = 1, my_command = 0;

uint8_t received_counter[NUM_NEIGHBORS];
uint8_t received_command[NUM_NEIGHBORS];

int zzz=0;

uint8_t packet_type;
char command;

bool connected[NUM_NEIGHBORS];

struct OBJ1{
	float px, py, vx, vy, hd;
	float neix[NUM_NEIGHBORS], neiy[NUM_NEIGHBORS];
} agent1;

struct OBJ2{
	float hd;
	float tim1, tim2;
	bool work, flag, chk1, chk2, sig_out, dir_out;
	bool dir[NUM_NEIGHBORS], sig[NUM_NEIGHBORS], osig[NUM_NEIGHBORS];
} agent2, agent3;

struct point {
	float x, y;
	uint8_t cntcmd;		//4bit for count, 4bit for command
	uint8_t signal, sensor;
} mdata;

// ============================================================================================
// variables
// ============================================================================================
float dt = 0.01;		// time step for differential equation (no relation with real time)

float ld = 20.0;		// virtual distance between the nodes
float acc = 20.0;		// self-propelling force
float gmma =1.0;		// viscosity
float ka = 0.1;			// spring constant
float d = 1.0;			// strength of anisotropy 0.0
float cf = 100.0;		// strength of interaction with neighbors 100.0
float rc = 25.0;		// optimum distance between agents 20.0
float tau = 0.1;		// relaxation time of heading dynamics 1.0
float forcex, forcey;   // force

// ============================================================================================
// Received Packet
// ============================================================================================
void rx_pkt(Xgrid::Packet *pkt)
{
	//pkt->rx_node : port number. See top.
	if(pkt->rx_node >= 0 && pkt->rx_node < NUM_NEIGHBORS){ 

		connected[pkt->rx_node] = true;
	
		switch(pkt->type)
		{
			case MESSAGE_COMMAND:
				char* char_ptr = (char*) pkt->data;
				command = *char_ptr;
				break;

			case MESSAGE_DATA:
				point* pt_ptr = (point*) pkt->data;
				uint8_t port = pkt->rx_node;

				received_counter[pkt->rx_node] = (pt_ptr->cntcmd) >> 4;

				agent1.neix[port] = pt_ptr->x;
				agent1.neiy[port] = pt_ptr->y;
				sensor_in[port]  = pt_ptr->sensor;
				agent2.sig[port] = ((pt_ptr->signal)&0b10000000) >> 7;
				agent3.sig[port] = ((pt_ptr->signal)&0b01000000) >> 6;
				agent3.dir[port] = ((pt_ptr->signal)&0b00100000) >> 5;
				break;
		}

		LED_PORT.OUTTGL = LED_USR_2_PIN_bm;	//green LED
	}
}

// ============================================================================================
// send message
// ============================================================================================
void send_message()
{
	if(sendmessage_stop) return;

	mdata.x = agent1.px;
	mdata.y = agent1.py;

	mdata.signal = (agent2.sig_out << 7) + (agent3.sig_out << 6) + (agent3.dir_out << 5);
	mdata.sensor = sensor_out;

	Xgrid::Packet pkt;
	pkt.type = MESSAGE_DATA;
	pkt.flags = 0;	
	pkt.radius = 1;
	pkt.data = (uint8_t *)&mdata;
	pkt.data_len = sizeof(point);
									
	xgrid.send_packet(&pkt, 0b00111111);	// send to all neighbors
}

// ============================================================================================
// Timer tick ISR (1 kHz)
// ============================================================================================
ISR(TCC0_OVF_vect)
{
		jiffies++;	// Timers

		// restarting status: if restart_on is set, board just sends reset signal every 500ms.
		//if(restart_on)
		//{
		//	if(jiffies%500 == 0) send_message(20);
		//}
		// normal status: just setting bit flags, no function.
		//else
		//{
			if(jiffies%50  == 0)  servo_motor_on   = true;

			if(jiffies%100 == 0)
			{
				sendmessage_fast = true;	
				if(speedup_on) LED_PORT.OUTTGL = LED_USR_1_PIN_bm;
//LED_PORT.OUTTGL = LED_USR_0_PIN_bm;
			}

			if(jiffies%500 == 0)
			{
				if(!speedup_on) LED_PORT.OUTTGL = LED_USR_1_PIN_bm;
			}

			if(jiffies%1000 == 0)
			{
LED_PORT.OUTTGL = LED_USR_0_PIN_bm;
				
				sendmessage_slow = true;
				
				if(sonar_attached){
					measure_on = true;
					sensing();
				}
			}
		//}

		xgrid.process();
}

// ============================================================================================
// RESTART PROCESS
// ============================================================================================
void RESTART_PROCESS()
{
	unsigned long count = jiffies + 3000;	//send RESTART_CODE for 3 sec

	Xgrid::Packet pkt;
	
	pkt.type = MESSAGE_COMMAND;
	pkt.flags = 0;
	pkt.radius = 20;
	pkt.data = (uint8_t *)str;
	pkt.data_len = 1;

	xgrid.send_packet(&pkt);

	while(count > jiffies);
	
	xboot_reset();
}

// ============================================================================================
// INITIALIZATION PROCESS
// ============================================================================================
void INIT_PROCESS()
{
	unsigned long count = jiffies + 1000;	//send INIT_CODE for 1 sec

	mdata.x = 0.0; mdata.y = 0.0; 
	my_command = INIT_CODE; 
	my_counter = 0;

	restart_on = true;

	while(count > jiffies);
	
	restart_on = false;
}


// ============================================================================================
// KEY INPUT PROCESS
// ============================================================================================
void key_input()
{
	char input_char = 0;

	if (usart.available()) input_char = usart.get();
	
						
	switch(input_char)
	{
	case 0x1b:	RESTART_PROCESS();
				break;
	case 'i':	INIT_PROCESS();
				break;
	case 'f':	speedup_on = true;	my_counter++; my_command = SPEED_UP; send_message(20);
				break;
	case 's':	speedup_on = false; my_counter++; my_command = SPEED_DOWN; send_message(20);
				break;
	case 'd':	if(display) display = false; else display = true;
				break;
	case 'q':	my_counter++; my_command = STOP_SEND; send_message(20);
				break;
	case 'g':	my_counter++; my_command = START_SEND; send_message(20);
				break;
	}

	if(my_counter>15) my_counter = 0;
}

// ============================================================================================
// SENSING PROCESS
// ============================================================================================
void sensing()
{
	uint16_t value = get_sonar_value();
		
	measure_temp += value;
	mcount++;

	//my_sensor_value is calculated as an average of 3 times (update every 3sec)
	// to treat as 8 bit data, divide by 2
	if(mcount>3){
		measure_temp = measure_temp / mcount;
		if(measure_temp > 255) measure_temp = 255;		//Max range = 510 cm
		my_sensor_value = (uint8_t) measure_temp;
		measure_temp = 0;
		mcount = 0;
	}	
	LED_PORT.OUTTGL = LED_USR_0_PIN_bm;		
}

// ============================================================================================
// SENSOR DATA PROCESS
// ============================================================================================
void SensorDataProcess()
{
	int i;

	if(sonar_attached)
	{
		sensor_data = my_sensor_value;

		for(i=0;i<NUM_NEIGHBORS;i++)
		{
			if(connected[i] && sensor_in[i] < my_sensor_value) sensor_data = sensor_in[i];
		}
	}
	else
	{
		for(i=0;i<NUM_NEIGHBORS;i++)
		{
			if(sensor_in[i] != sensor_oin[i]) sensor_data = sensor_in[i];	//detect change
			sensor_oin[i] = sensor_in[i];
		}
	}
	
	sensor_out = sensor_data;
}

// ============================================================================================
// RECEIVED MESSAGE PROCESS
// ============================================================================================
void ReceivedMessageProcess()
{
	uint8_t i, counter, command;
	bool flag;

	for(i=0;i<NUM_NEIGHBORS;i++)
	{
		if(connected[i])
		{

			if(!restart_on && received_command[i] == RESTART_CODE) RESTART_PROCESS();

			if(received_command[i] == INIT_CODE)
			{
				INIT_PROCESS();
				init_common();
			}

			if(received_command[i] == STOP_SEND) sendmessage_stop = true;
			if(received_command[i] == START_SEND) sendmessage_stop = false;

			counter = received_counter[i];
			command = received_command[i];

			// Find latest command and execute
			flag = false;
			if((counter > my_counter) && (counter - my_counter) < 5) flag = true;
			if((counter + 15 > my_counter ) && (counter + 15 - my_counter) < 5) flag = true;

			if(flag)
			{
				my_counter = counter;
				switch(command)
				{
					case SPEED_DOWN:	speedup_on = false; break;
					case SPEED_UP  :	speedup_on = true;  break;
				}
				my_command = command;
			}
		}
	}
}

// ============================================================================================
// SERVO CONTROL
// ============================================================================================
void ServoControl()
{
	int d=sensor_data, a1 = 0, a2 = 0, a3 = 0;
	float angle, speed = 100.0;
	
	if(d<50) a1=90;
	if(d>=50 && d<100) a1=180-90*d/50;
	if(d>=100) a1=0;

	if(d<50) a2=0;
	if(d>50 && d<100) a2=90*(d-50)/50;
	if(d>=100 && d<150) a2=90-90*(d-100)/50;
	if(d>=150) a2=0;

	if(d<100) a3=0;
	if(d>=100 && d<150) a3=90*(d-100)/50;
	if(d>=150) a3=90;
	//if(d>=150 && d<250) a3=90-90*(d-150)/100;
	//if(d>=250) a3=0;

	if(agent2.work){a1=0; a2=90; a3=0;}
	if(agent3.work){a1=90; a2=0; a3=0;}

	angle = a1 * agent3.hd + a2 * agent2.hd + a3 * cos(agent1.hd * speed);

	set_servo_position(angle);
}

// ============================================================================================
// MAIN FUNCTION
// ============================================================================================
int main(void)
{
	int i;

	_delay_ms(50);

	// ========== INITIALIZATION ==========
    init();				//for board
	init_sonar();		//for sensor
	init_common();		//for program

	sonar_attached = check_sonar_attached();

	_delay_ms(500);

	for(i=0;i<10;i++){
		if(sonar_attached) LED_PORT.OUTTGL = LED_USR_2_PIN_bm;
		else LED_PORT.OUTTGL = LED_USR_0_PIN_bm;
		_delay_ms(100);
	}

    xgrid.rx_pkt = &rx_pkt;

	my_counter = 0;
	my_command = 0;

	while (1)
	{
		// ========== KEY INPUT ==========
		key_input();

		// ========== SENSING ==========
		if(measure_on)
		{
			//sensing();
			measure_on = false;
		}

		// ========== MESSAGE PASSING ==========
		ReceivedMessageProcess();

		// ========== CALCULATION & SENSOR DATA & SEND MESSAGE & DISPLAY ==========
		if((speedup_on && sendmessage_fast) || (!speedup_on && sendmessage_slow))
		{
			swarm_calculation1();
			swarm_calculation2();
			swarm_calculation3();

			SensorDataProcess();

			// send message
			send_message(1);
			sendmessage_fast = false;
			sendmessage_slow = false;

			if(display)
			{
				//fprintf_P(&usart_stream, PSTR("%d%d%d%d%d%d\r\n"),connected[0],connected[1],connected[2],connected[3],connected[4],connected[5]);
				fprintf_P(&usart_stream, PSTR("%d %d\r\n"),my_sensor_value, sensor_data);
				fprintf_P(&usart_stream, PSTR("hd: %i.%i\n\r"), prt_flt3(agent1.hd));
			}
		}

		// ========== SERVO MOTOR CONTROL ==========
		if(servo_motor_on)
		{
			ServoControl();
			servo_motor_on = false;
		}
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

	for(i=0;i<NUM_NEIGHBORS;i++)
	{
		agent1.neix[i] = 0.0;
		agent1.neiy[i] = 0.0;
		connected[i] = false;
		agent2.sig[i] = false;
		agent3.sig[i] = false;
		agent3.osig[i] = false;
		sensor_in[i] = 0;
		sensor_oin[i] = 0;
	}

	agent1.px = 2.7; 
	agent1.py = 75.6;
	
	agent1.vx = 3.8; 
	agent1.vy = 1.3;
	
	agent1.hd = 0.0;

	agent2.tim1 = 15.0;
	agent2.tim2 = 15.0;
	agent2.flag = false;

	agent3.tim1 = 0.0;
	agent3.tim2 = 10.0;
	agent3.flag = false;
	agent3.chk1 = false;

}

// ============================================================================================
// ============================================================================================
// SWARM DYNAMICS 1 --- Ken's Swarm Dynamics ---
// ============================================================================================
// ============================================================================================
void swarm_calculation1()
{
	int i, sum;
	float dvx, dvy, lx, ly, vabs, ds, fx, fy;
	float dir = agent1.hd;

	// ===== calculation of forces =====
	//self-propel force and viscosity
	dvx = acc * cos(dir) - gmma * agent1.vx;
	dvy = acc * sin(dir) - gmma * agent1.vy;

	//interaction force with 6 neighbors
	forcex=0.0; forcey=0.0;

	for(i=0;i<NUM_NEIGHBORS;i++)
	{
		if(connected[i]){
			swarm_interaction1(i);
			connected[i]=false;	
		}
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
	vabs = sqrt(agent1.vx * agent1.vx + agent1.vy * agent1.vy);

	fx = agent1.vx / vabs;
	fy = agent1.vy / vabs;
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
	
	//fprintf_P(&usart_stream, PSTR("xy:(%i.%i,%i.%i)\n\r"), prt_flt3(agent1.px), prt_flt3(agent1.py));

}

// --------------------------------------------------------------------------------------------
void swarm_interaction1(int nei)
{
	float disx, disy, dis1, dis2, alph, force;
	float di, dj;
	bool flag = true;

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
		disx = agent1.neix[nei] - agent1.px + ld * di;
		disy = agent1.neiy[nei] - agent1.py + ld * dj;

		dis2 = disx * disx + disy * disy;
		dis1 = sqrt(dis2);

		if(dis2!=0.0)
		{
			alph = 1.0 + d * (disx * cos(agent1.hd) + disy * sin(agent1.hd)) / dis1; //inner product
			force = -cf * (rc / dis1 - 1.0) * rc * rc / dis2;
			
			forcex += alph * force * disx / dis1;
			forcey += alph * force * disy / dis1;
		}

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
	float alpha, angle = 0;
	
	float period = 10.0;
	float decay = 2.0;

	if(!sonar_attached){
		for(i=0;i<NUM_NEIGHBORS;i++)
		{
			if(agent2.sig[i]) flg = true; 
		}
	}

	if(my_sensor_value>=50 && my_sensor_value<=100) flg = true;	//If sensor is connected

	/*if(my_sensor_value >=50 && my_sensor_value <=150){
		flg = true;
		if(my_sensor_value<100){
			period = 15.0;
			decay = 0.6;
		}
		else{
			period = 10.0;
			decay =1.2;
		}
	}*/
	
	if(flg && !agent2.flag)
	{
		agent2.tim1 = 0; 
		agent2.tim2 = 0; 
		agent2.flag = true;
	}

	if(agent2.tim2 < period)
	{
		agent2.tim1 += 0.5; 	
		agent2.tim2 += 0.1;

		if(agent2.tim2 > period / 5.0 && agent2.tim2 < period / 5.0 + 0.5) agent2.sig_out = 1;
		else agent2.sig_out = 0;

		if(agent2.tim2 > period-1.0) agent2.flag = false;

		//alpha = sin(0.2 * agent2.tim2) * sin(0.2 * agent2.tim2) * exp(-decay * agent2.tim2) * 20.0;
		
		alpha = sin(0.2 * agent2.tim2) * exp(-decay * agent2.tim2) * 27.0;
		angle = alpha * sin(agent2.tim1);

		agent2.work = true;
	}
	else agent2.work = false;

	agent2.hd = angle;
}

// ============================================================================================
// ============================================================================================
// SWARM DYNAMICS 3 --- vertical sync ---
// ============================================================================================
// ============================================================================================
void swarm_calculation3()
{
	float alpha, angle = 0;

	/*
	if(!sonar_attached) agent3.chk1 = false;

	if(!agent3.chk1 && sonar_attached)
	{
		agent3.flag = false;
		agent3.chk1 = true;
	}
	*/
	if(!sonar_attached && !((agent3.sig[0]&&!agent3.dir[0])||(agent3.sig[1]&&agent3.dir[1]))) 
		agent3.chk2 = false;

	if(!agent3.chk2 && (agent3.sig[0]&&!agent3.dir[0])||(agent3.sig[1]&&agent3.dir[1]))
	{
		agent3.flag = false;
		agent3.chk2 = true;
	}

	if(!agent3.flag)
	{
		//sensor
		if(my_sensor_value >=10 && my_sensor_value < 50)
		{ 
			agent3.sig_out = true;
			agent3.dir_out = true;
			agent3.tim2 = 0;
			agent3.chk1 = 1;
		}
		if(my_sensor_value >=50)
		{
			agent3.sig_out = false;
			agent3.dir_out = false;
			agent3.flag = true;
		}

		//no sensor
		if(!sonar_attached)
		{
			if((agent3.sig[0]&&!agent3.dir[0])||(agent3.sig[1]&&agent3.dir[1]))
			{
				agent3.tim2 = 0;

				if(agent3.sig[0]&&!agent3.dir[0])
				{
					agent3.sig_out = true;
					agent3.dir_out = true;
				}
				if(agent3.sig[1]&&agent3.dir[1])
				{
					agent3.sig_out = true;
					agent3.dir_out = false;
				}
			}
			else{
				agent3.sig_out = false;
				agent3.dir_out = false;
				agent3.flag = true;
			}
		}
	}

	if(sonar_attached && agent3.flag && agent3.tim2!=0.0) agent3.flag=false;

	if(agent3.tim1 > 6.283) agent3.tim1 -= 6.283;

	if(agent3.tim2 < 10.0)
	{
		agent3.work = true;
		agent3.tim1 += 0.5; 	
		agent3.tim2 += 0.1;

		alpha = exp(-0.5 * agent3.tim2);
		angle = alpha * sin(agent3.tim1);

		if(agent3.tim2 >9.8)
		{
			agent3.flag = false;
			agent3.sig_out = false;
			agent3.dir_out = false;
		}
	}
	else agent3.work = false;

	agent3.hd = angle;
}
