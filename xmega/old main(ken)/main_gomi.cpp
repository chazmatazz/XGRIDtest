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

#define BOTTOM_RIGHT			0
#define BOTTOM_LEFT				1
#define LEFT_BOTTOM				2
#define LEFT_TOP				3
#define RIGHT_BOTTOM			4
#define RIGHT_TOP				5

#define BOTTOM_LEFT_MASK		0b00000010
#define BOTTOM_RIGHT_MASK		0b00000001
#define LEFT_TOP_MASK			0b00001000
#define LEFT_BOTTOM_MASK		0b00000100
#define RIGHT_BOTTOM_MASK		0b00010000
#define RIGHT_TOP_MASK			0b00100000

#define MESSAGE_NUMDATA 0
#define MESSAGE_COMMAND 1
#define MESSAGE_USENSOR 2

#define RUNTIME1 120
#define RESTTIME 20

// ============================================================================================
// prototype declaration
// ============================================================================================
void init_common(void);
void swarm_initialization1(void);

void swarm_calculation0(void);
void swarm_interaction0(int);

void swarm_calculation1(void);
void swarm_interaction1(int);

void servo_motor_control(float);

void swarm_initialization2(void);
void swarm_calculation2(void);

void swarm_initialization3(void);
void swarm_calculation3(void);

void RESTART_PROCESS(void);
// ============================================================================================
bool reboot_on = false;

bool sendmessage_fast = false;
bool sendmessage_slow = false;
bool sendmessage_stop = false;

bool calculation_on = false;
bool servo_motor_on = false;
bool communication_on = true;

bool display = false;
bool display_on = false;
bool restart_on = false;
bool speedup_on = false;
bool mode = true;
bool sonar_attached = false;
bool send_sensor_value_on = false;
uint8_t my_sensor_value, my_sensor_data, no_sens_sec = 0;

float ratio = 0.0;

uint8_t phase = 1;
int sec_counter;
float decay_tim;
float global_amp;

volatile unsigned long jiffies = 0, temp_time;

uint8_t sensor_in[NUM_NEIGHBORS];

bool connected[NUM_NEIGHBORS];

struct OBJ1{
	float px, py, vx, vy, hd;
	float neix[NUM_NEIGHBORS], neiy[NUM_NEIGHBORS];
} agent1, agent0;

struct OBJ2{
	float hd;
	float tim1, tim2;
	bool work, flag, chk1, chk2, sig_out, dir_out;
	bool dir[NUM_NEIGHBORS], sig[NUM_NEIGHBORS], osig[NUM_NEIGHBORS];
} agent2, agent3;

struct point {
	float x, y;
	float x0,y0;
	uint8_t ctrl;
} mdata;

// ============================================================================================
// variables
// ============================================================================================
float dt = 0.01;		// time step for differential equation (no relation with real time)
float ld = 20.0, acc = 20.0, cf = 100.0;
float gmma1 = 5.0, ka1 = 0.1, d1 = 0.0, rc1 = 20.0, tau1 = 1.0;
float gmma0 = 1.0, ka0 = 1.0, d0 = 1.0, rc0 = 25.0, tau0 = 0.1;
float forcex, forcey;

// ============================================================================================
// send message
// ============================================================================================
void send_message(uint8_t MessageType)
{
	Xgrid::Packet pkt;
	pkt.type = MessageType;
	pkt.flags = 0;	

	switch(MessageType)
	{
		case MESSAGE_NUMDATA:
			mdata.x = agent1.px;
			mdata.y = agent1.py;
			mdata.x0 = agent0.px;
			mdata.y0 = agent0.py;
			
			mdata.ctrl = (agent2.sig_out << 7) + (agent3.sig_out << 6) + (agent3.dir_out << 5);

			pkt.data = (uint8_t *)&mdata;
			pkt.radius = 1;
			pkt.data_len = sizeof(point);
			break;

		case MESSAGE_USENSOR:
			pkt.data = (uint8_t*) my_sensor_value;
			pkt.radius = 20;
			pkt.data_len = 1;
			break;
	}
									
	xgrid.send_packet(&pkt, 0b00111111);	// send to all neighbors
	LED_PORT.OUTTGL = LED_USR_2_PIN_bm;		//green LED
}


// ============================================================================================
// Received Packet
// ============================================================================================
void rx_pkt(Xgrid::Packet *pkt)
{
	uint8_t port = pkt->rx_node;	//pkt->rx_node : port number. See top.
	char command;

	if(pkt->rx_node >= 0 && pkt->rx_node < NUM_NEIGHBORS){

		if(pkt->type == MESSAGE_COMMAND)
		{
			char* char_ptr = (char*) pkt->data;
			command = *char_ptr;
			
			switch(command)
			{
				case 'b': ratio += 0.1; if(ratio>1.0) ratio=1.0; break;
				case 'n': ratio -= 0.1; if(ratio<0.1) ratio=0.0; break;
				case 'c': mode = false;		break;
				case 'v': mode = true;		break;
				case 'Z': reboot_on = true;		break;
				case 'i': init_common();		break;
				case 'f': speedup_on = true;	break;
				case 's': speedup_on = false;	break;
				case 'q': communication_on = false;	break;
				case 'g': communication_on = true;	break;
			}
		}

		if(pkt->type == MESSAGE_NUMDATA)
		{
			connected[port] = true;

			point* pt_ptr = (point*) pkt->data;

			agent0.neix[port] = pt_ptr->x0;
			agent0.neiy[port] = pt_ptr->y0;
			
			agent1.neix[port] = pt_ptr->x;
			agent1.neiy[port] = pt_ptr->y;
	
			agent2.sig[port] = ((pt_ptr->ctrl)&0b10000000) >> 7;

			agent3.sig[port] = ((pt_ptr->ctrl)&0b01000000) >> 6;
			agent3.dir[port] = ((pt_ptr->ctrl)&0b00100000) >> 5;
		}

		if(pkt->type == MESSAGE_USENSOR)
		{
			uint8_t* sensor_ptr = (uint8_t*) pkt->data;
			sensor_in[port] = *sensor_ptr;
		}

		LED_PORT.OUTTGL = LED_USR_2_PIN_bm;	//green LED
	}
}


// ============================================================================================
// KEY INPUT PROCESS
// ============================================================================================
void key_input()
{
	char input_char = 0;
	
	if (usart.available()) input_char = usart.get();
	
	if(input_char == 0x1b) xboot_reset();

	if(input_char == 'c')
	{
		char str[] = "c";	Xgrid::Packet pkt;	pkt.type = MESSAGE_COMMAND;	pkt.flags = 0;
		pkt.radius = 20;	pkt.data = (uint8_t *)str;	pkt.data_len = 20;	xgrid.send_packet(&pkt);
		mode = false;
	}
	
	if(input_char == 'v')
	{
		char str[] = "v";
        Xgrid::Packet pkt;	Xgrid::Packet pkt;	pkt.type = MESSAGE_COMMAND;	pkt.flags = 0;
		pkt.radius = 20;	pkt.data = (uint8_t *)str;	pkt.data_len = 20;	xgrid.send_packet(&pkt);
		mode = true;
	}
	
	if(input_char == 'b')
	{
		char str[] = "b";
        Xgrid::Packet pkt;	Xgrid::Packet pkt;	pkt.type = MESSAGE_COMMAND;	pkt.flags = 0;
		pkt.radius = 20;	pkt.data = (uint8_t *)str;	pkt.data_len = 20;	xgrid.send_packet(&pkt);
		ratio += 0.1; if(ratio>1.0) ratio=1.0;
	}
	
	if(input_char == 'n')
	{
		char str[] = "n";	Xgrid::Packet pkt;	pkt.type = MESSAGE_COMMAND;	pkt.flags = 0;
		pkt.radius = 20;	pkt.data = (uint8_t *)str;	pkt.data_len = 20;	xgrid.send_packet(&pkt);
		ratio-= 0.1; if(ratio<0.1) ratio=0.0;
	}
	
	if(input_char == 'f')
	{
		char str[] = "f";
        Xgrid::Packet pkt;	Xgrid::Packet pkt;	pkt.type = MESSAGE_COMMAND;	pkt.flags = 0;
		pkt.radius = 20;	pkt.data = (uint8_t *)str;	pkt.data_len = 20;	xgrid.send_packet(&pkt);
		speedup_on = true;
	}
				
	if(input_char == 's')
	{
		char str[] = "s";	Xgrid::Packet pkt;	pkt.type = MESSAGE_COMMAND;	pkt.flags = 0;
		pkt.radius = 20;	pkt.data = (uint8_t *)str;	pkt.data_len = 20;	xgrid.send_packet(&pkt);
		speedup_on = false;
	}
	
	if(input_char == 'Z')
	{
        reboot_on = true;
		char str[] = "Z";	Xgrid::Packet pkt;	pkt.type = MESSAGE_COMMAND;	pkt.flags = 0;
		pkt.radius = 20;	pkt.data = (uint8_t *)str;	pkt.data_len = 20;	xgrid.send_packet(&pkt);
		
		temp_time = jiffies + 3000;
		while(jiffies < temp_time){};
		
		xboot_reset();
	}

	if(input_char == 'i')
	{
		char str[] = "i";	Xgrid::Packet pkt;	pkt.type = MESSAGE_COMMAND;	pkt.flags = 0;
		pkt.radius = 20;	pkt.data = (uint8_t *)str;	pkt.data_len = 20;	xgrid.send_packet(&pkt);
		init_common();
	}	

	if(input_char == 'q')
	{
		char str[] = "q";	Xgrid::Packet pkt;	pkt.type = MESSAGE_COMMAND;	pkt.flags = 0;
		pkt.radius = 20;	pkt.data = (uint8_t *)str;	pkt.data_len = 20;	xgrid.send_packet(&pkt);
		communication_on = false;
	}	

	if(input_char == 'g')
	{
		char str[] = "g";	Xgrid::Packet pkt;	pkt.type = MESSAGE_COMMAND;	pkt.flags = 0;
		pkt.radius = 20;	pkt.data = (uint8_t *)str;	pkt.data_len = 20;	xgrid.send_packet(&pkt);
		communication_on = true;
	}	

	if(input_char == 'd'){
		if(display) display = false; else display = true;
	}
}

// ============================================================================================
// SENSING PROCESS (sensor is attached)
// ============================================================================================
uint8_t sensing()
{
	uint16_t value = get_sonar_value();
	return (uint8_t) value;
}

// ============================================================================================
// SENSOR DATA PROCESS
// ============================================================================================
uint8_t SensorDataProcess()
{
	uint8_t i, min = 255;
	bool flg = false;
	
	for(i=0;i<NUM_NEIGHBORS;i++)
	{
		if(connected[i] && sensor_in[i]!=0){
			if(sensor_in[i] < min) min = sensor_in[i];
		}
	}

	return min;
}

// ============================================================================================
// SERVO CONTROL
// ============================================================================================
void ServoControl()
{
	bool servo_sw = true, phase;
	int a1 = 0, a2 = 0, a3 = 0;
	float angle, speed = 100.0;
	
	if(sec_counter == 0){init_common(); sec_counter++; }
	global_amp += 0.005;
	if(global_amp>1.0) global_amp =1.0;
	
	if(sec_counter < RUNTIME1)	decay_tim = 0.0;
	if(sec_counter >= RUNTIME1 && sec_counter < RUNTIME1 + 10)
	{
		decay_tim+=0.05;
		global_amp = exp(-0.4 * decay_tim);
	}
	if(sec_counter >= RUNTIME1 + 10 && sec_counter < RUNTIME1 + 10 + RESTTIME)
	{
		servo_sw = false; global_amp = 0;
		disable_servo();
	}
	if(sec_counter >= RUNTIME1 + RESTTIME + 10)
	{
		init_common();
		char str[] = "i";
        Xgrid::Packet pkt;
        pkt.type = MESSAGE_COMMAND;		pkt.flags = 0;	pkt.radius = 20;	pkt.data = (uint8_t *)str;		
		pkt.data_len = 1;				xgrid.send_packet(&pkt);
	}

	angle = global_amp * 90 * (ratio * cos(agent1.hd * speed) + (1.0 - ratio) * cos(agent0.hd * speed));

	if(servo_sw) set_servo_position(angle);
}

// ============================================================================================
// Timer tick ISR (1 kHz)
// ============================================================================================
ISR(TCC0_OVF_vect)
{
	jiffies++;	// Timers

	if(reboot_on){
		/*if(jiffies%500==0)
		{
			char str[] = "Z";
			Xgrid::Packet pkt;
			pkt.type = MESSAGE_COMMAND;		pkt.flags = 0;	pkt.radius = 20;	pkt.data = (uint8_t *)str;		
			pkt.data_len = 20;				xgrid.send_packet(&pkt);
		}
		*/
	}
	else
	{
		//if(jiffies%50  == 0)  servo_motor_on   = true;

		if(jiffies%100 == 0)
		{
			servo_motor_on   = true;
			sendmessage_fast = true;	

//LED_PORT.OUTTGL = LED_USR_0_PIN_bm;
LED_PORT.OUTTGL = LED_USR_1_PIN_bm;
		}

		if(jiffies%999 == 0)
		{
			sec_counter++;
//LED_PORT.OUTTGL = LED_USR_1_PIN_bm;
			sendmessage_slow = true;

			display_on = true;

			if(sonar_attached) my_sensor_value = sensing();
			send_sensor_value_on = true;
			
			if(my_sensor_data == 255) no_sens_sec++;
			else no_sens_sec= 0;
		}
	}
		xgrid.process();
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
	init_servo();		//for servo
	init_common();		//for program
	init_sonar();		//for sensor
	sonar_attached = check_sonar_attached();	//1:attached, 0:no
	
	// ===== INDICATION: SONAR ATTCHED? (attached: GREEN LED, not: RED LED) =====
	for(i=0;i<10;i++){
		if(sonar_attached)
		{
			LED_PORT.OUTTGL = LED_USR_2_PIN_bm;
			_delay_ms(500);
		}
		else LED_PORT.OUTTGL = LED_USR_0_PIN_bm;
		_delay_ms(100);
	}

    xgrid.rx_pkt = &rx_pkt;

	while (1)
	{
		if(reboot_on){
			temp_time = jiffies + 3000;
			while(jiffies < temp_time){LED_PORT.OUTTGL = LED_USR_1_PIN_bm; _delay_ms(100);}
			xboot_reset();	
		}
			
		// ========== KEY INPUT ==========
		key_input();

		// ========== SENSOR ==========
		//SENSOR: works every 1 sec (See ISR() function)
		if(sonar_attached && send_sensor_value_on)
		{
			if(communication_on) send_message(MESSAGE_USENSOR,"");
			send_sensor_value_on = false;
		}
		my_sensor_data = SensorDataProcess();

		// ========== CALCULATION & SENSOR DATA ==========
		if((speedup_on && sendmessage_fast) || (!speedup_on && sendmessage_slow))
		{
			if(communication_on)
			{
				swarm_calculation0();
				swarm_calculation1();
				swarm_calculation2();
				swarm_calculation3();
				send_message(MESSAGE_NUMDATA,"");
			}
			
			sendmessage_fast = false;
			sendmessage_slow = false;
		}
		// ==========  DISPLAY ==========
		if(display_on && display)
		{
			fprintf_P(&usart_stream, PSTR("%d %i.%i\r\n"),sec_counter, prt_flt3(ratio));
			//fprintf_P(&usart_stream, PSTR("%d %d\r\n"),my_sensor_value, sensor_out);
			//fprintf_P(&usart_stream, PSTR("hd: %i.%i\n\r"), prt_flt3(agent1.hd));
			display_on = false;
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
	
	for(i=0;i<NUM_NEIGHBORS;i++)
	{
		agent0.neix[i] = 0.0;
		agent0.neiy[i] = 0.0;
		agent1.neix[i] = 0.0;
		agent1.neiy[i] = 0.0;
		connected[i] = false;
		agent2.sig[i] = false;
		agent3.sig[i] = false;
		agent3.osig[i] = false;
		sensor_in[i] = 0;
	}

	agent0.px = 2.7; 
	agent0.py = 75.6;
	
	agent0.vx = 3.8; 
	agent0.vy = 1.3;
	
	agent0.hd = 0.0;
	
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

	global_amp = 0;
	sec_counter = 0;
	enable_servo();
}

// ============================================================================================
// ============================================================================================
// SWARM DYNAMICS 1 --- Ken's Swarm Dynamics ---
// ============================================================================================
// ============================================================================================
void swarm_calculation1()
{
	int i;
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
//########################################################################################
void swarm_calculation0()
{
	int i;
	float dvx, dvy, lx, ly, vabs, ds, fx, fy;
	float dir = agent0.hd;

	// ===== calculation of forces =====
	//self-propel force and viscosity
	dvx = acc * cos(dir) - gmma0 * agent0.vx;
	dvy = acc * sin(dir) - gmma0 * agent0.vy;

	//interaction force with 6 neighbors
	forcex=0.0; forcey=0.0;

	for(i=0;i<NUM_NEIGHBORS;i++)
	{
		if(connected[i]){
			swarm_interaction0(i);
		}
	}

	dvx = dvx + forcex;
	dvy = dvy + forcey;

	//spring term (Zero-length spring). fixed point = (0,0)
	lx = - agent0.px;
	ly = - agent0.py;
	dvx = dvx + ka0 * lx;
	dvy = dvy + ka0 * ly;

	// ===== update =====
	//direction
	vabs = sqrt(agent0.vx * agent0.vx + agent0.vy * agent0.vy);

	fx = agent0.vx / vabs;
	fy = agent0.vy / vabs;
	ds = -1.0 / tau0 * (sin(dir)*fx-cos(dir)*fy); // get sin value by cross product

	agent0.hd += ds * dt;

	if(agent0.hd > 2.0 * PI)	agent0.hd -= 2.0 * PI;
	if(agent0.hd < 0 ) 			agent0.hd += 2.0 * PI;

	//velocity
	agent0.vx += dvx * dt;
	agent0.vy += dvy * dt;


	//position
	agent0.px += agent0.vx * dt;
	agent0.py += agent0.vy * dt;
}

// --------------------------------------------------------------------------------------------
void swarm_interaction0(int nei)
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
		disx = agent0.neix[nei] - agent0.px + ld * di;
		disy = agent0.neiy[nei] - agent0.py + ld * dj;

		dis2 = disx * disx + disy * disy;
		dis1 = sqrt(dis2);

		if(dis2!=0.0)
		{
			alph = 1.0 + d0 * (disx * cos(agent0.hd) + disy * sin(agent0.hd)) / dis1; //inner product
			force = -cf * (rc0 / dis1 - 1.0) * rc0 * rc0 / dis2;
			
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
	float decay = 1.0;

	if(!sonar_attached){
		for(i=0;i<NUM_NEIGHBORS;i++)
		{
			if(agent2.sig[i]) flg = true; 
		}
	}

	if(my_sensor_value>=50 && my_sensor_value<=100) flg = true;	//If sensor is connected
	
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
		
		alpha = agent2.tim2 * exp(-decay * agent2.tim2) * 2.7;
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
