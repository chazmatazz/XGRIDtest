// ============================================================================================
// variables
// ============================================================================================
float dt = 0.01;		// time step for differential equation (no relation with real time)
float ld=20.0, acc=20.0, cf=100.0;
float gmma1=5.0, ka1=0.1, d1=0.0, rc1=20.0, tau1=1.0;	//for synchro motion
float gmma0=1.0, ka0=1.0, d0=1.0, rc0=25.0, tau0=0.1;	//for chaotic motion
float forcex, forcey;

// ============================================================================================
// for initialization of swarm dynamics function
// ============================================================================================
void init_variables()
{
	int i;

	if(special) 
		send_message(MESSAGE_COMMAND, ALL_DIRECTION, ALL, "i");

	for(i=0;i<NUM_NEIGHBORS;i++)
	{
		agent0.neix[i] = 0.0;	agent0.neiy[i] = 0.0;
		agent1.neix[i] = 0.0;	agent1.neiy[i] = 0.0;
		connected[i] = false;
	}

	// for chaotic motion
	agent0.px = 22.0; agent0.py = 40.0; agent0.vx = 0.186; agent0.vy = -4.8; agent0.hd = 4.75;

	//for ordered motion
	if(sec_counter < STGtime4)
	{
		agent1.px =  2.7; agent1.py = 75.6; agent1.vx = 3.8;   agent1.vy = 1.3;	 agent1.hd = 0.0;
	}
	else
	{
		agent1.px = 22.0; agent1.py = 40.0; agent1.vx = 0.186; agent1.vy = -4.8; agent1.hd = 4.75;
	}

	wave_flg = false; 
	wave_ping = false;
	column_flg = false;
	rhythm_on = false;

	agent2.flg = false;

	agent3.tim2 = 20.0;

	global_amp = 0;
	
	enable_servo();
	
	set_servo_position(0);
}

// ============================================================================================
// ============================================================================================
// SWARM DYNAMICS 5 --- rhythm ---
// ============================================================================================
// ============================================================================================
float rhythm(float stp)
{
	float self = 0, shift = 1.0;

	if(special)
	{
		self = agent0.py;
		agent0.px = agent0.py - shift;
		send_message(MESSAGE_NUMDATA, 0b00011100, NULL, "");

		agent0.py += stp;
		if(agent0.py > 2.0 * PI) agent0.py -= 2.0 * PI;
	}
	else
	{
		if(agent0.neix[0]!=0)
		{
			self = agent0.neiy[0];

			agent0.py = self;
			agent0.px = self - shift;
			send_message(MESSAGE_NUMDATA, 0b00011100, NULL, "");	
		}
		else if(agent0.neix[2]!=0)
		{
			self = agent0.neix[2];
			agent0.px = self - shift;
			send_message(MESSAGE_NUMDATA, 0b00010000, NULL, "");
		}
		else if(agent0.neix[4]!=0)
		{
			self = agent0.neix[4];
			agent0.px = self - shift;
			send_message(MESSAGE_NUMDATA, 0b00000100, NULL, "");
		}
	}

	return sin(self);
}

// --------------------------------------------------------------------------------------------
float rhythm_control1()	//Amplitude Modulation
{
	static int timestep = 0;
	float tim, dtim = 2.0 * PI / 30.0;		// 1cycle = 30 sec
	
	tim = (float)timestep * dtim;
	global_amp = sin(tim) * sin(tim);
	if(global_amp < 0.1) global_amp = 0.1; // to avoid complete stop

	if(rhythm_on)
	{
		timestep++;
		rhythm_on = false;
	}

	return global_amp * rhythm(0.6);
}

// --------------------------------------------------------------------------------------------
float rhythm_control2()	//Frequency Modulation
{
	static int timestep = 0;
	float stp, tim, dtim = 2.0 * PI / 30.0;	// 1cycle = 30 sec
	
	tim = (float)timestep * dtim;
	stp = 0.6 + 0.4 * cos(tim);

	if(rhythm_on)
	{
		timestep++;
		rhythm_on = false;
	}

	return rhythm(stp);
}

// --------------------------------------------------------------------------------------------
float rhythm_control3()	//Frequency Modulation
{
	static int timestep = 0;
	float stp, tim, dtim = 2.0 * PI / 30.0;	// 1cycle = 30 sec
	
	tim = (float)timestep * dtim;
	stp = 0.2;

	if(rhythm_on)
	{
		timestep++;
		rhythm_on = false;
	}

	return rhythm(stp);
}

// ============================================================================================
// ============================================================================================
// SWARM DYNAMICS 4 --- plane wave ---
// ============================================================================================
// ============================================================================================
float plane_wave()
{
	float period, decay = 1.0;
	float ampl, angle;
	uint8_t wave_dir;

	if(!agent2.flg)
	{
		agent2.tim1 = 0;
		agent2.tim2 = 0;
		wave_ping = false;

		// sensor is connected
		if(sonar_attached)
		{
			//if received horizontal signal 'a' 
			if(wave_flg) agent2.flg = true;
			//detect something
			else if(sensor_value_trichk >= RANGE1 && sensor_value_trichk <= RANGE3)
			{
				column_flg = true;
				agent2.flg = true;
				_delay_ms(500);
				send_message(MESSAGE_COMMAND, 0b00001000, 1, "b");
			}
		}
		//  no sensor
		else
		{
			//received vertical signal 'b'
			if(column_flg)
			{
				agent2.flg = true;
				send_message(MESSAGE_COMMAND, 0b00001000, 1, "b");
			}

			//received horizontal signal 'a' 
			if(wave_flg) agent2.flg = true;
		}
	}
	else
	{
		if(sonar_attached) period = 7.0; else period = 7.0;

		if(agent2.tim2 < period)
		{
			agent2.tim1 += 0.5;
			agent2.tim2 += 0.1;

			//threshold time 1: send message to neighbors
			if(agent2.tim2 > period / 4.0)
			{
				if(!wave_ping)
				{
					wave_ping = true; //send message only once

					//sensor board: send signal both side
					if(column_flg) wave_dir = 0b00010100;
					else
					{
						if(wave_port==2)	wave_dir = 0b00010000;
						else				wave_dir = 0b00000100;
					}
					send_message(MESSAGE_COMMAND, wave_dir, 1, "a");	
				}
			}
		}
		if(agent2.tim2 >= period)
		{
			agent2.flg = false;
			wave_flg = false;
			column_flg = false;
		}
	}

	ampl = agent2.tim2 * exp(-decay * agent2.tim2) * 2.7;
	angle = ampl * sin(agent2.tim1);
			
	return angle;
}


// ============================================================================================
// ============================================================================================
// SWARM DYNAMICS 3 --- swing column ---
// ============================================================================================
// ============================================================================================
float column()
{
	float decay = 1.0;
	float ampl, angle;

	if(sonar_attached)
	{
		if(sensor_value_dblchk >= RANGE1 && sensor_value_dblchk <= RANGE3)
		{
			// send command every 300ms
			if(use_sensor_data_on)
			{
				send_message(MESSAGE_COMMAND, 0b00001000, 1, "b");
				agent3.tim2 = 0;
				use_sensor_data_on = false;
			}
		}
	}
	// no sensor, but received 'b' 
	if(column_flg)
	{
		agent3.tim2 = 0;

		//reset column_flg every 300ms
		if(use_sensor_data_on)
		{
			column_flg = false;
			use_sensor_data_on = false;
			send_message(MESSAGE_COMMAND, 0b00001000, 1, "b");
		}
	}

	ampl = exp(-decay * agent3.tim2);
	angle = ampl * sin(agent3.tim1);

	agent3.tim1 += 1.0; if(agent3.tim1 > 2.0*PI) agent3.tim1 -= 2.0*PI;
	agent3.tim2 += 0.1; if(agent3.tim2 > 20.0) agent3.tim2 = 20.0;

	return angle;
}


// ============================================================================================
// ============================================================================================
// SWARM DYNAMICS 2 --- wave ---
// ============================================================================================
// ============================================================================================
float wave()
{
	float period, decay = 1.0;
	float ampl, angle;
	
	if(!agent2.flg)
	{
		agent2.tim1 = 0;
		agent2.tim2 = 0;
		wave_ping = false;

		// received 'a' from the neighbor
		if(wave_flg) agent2.flg = true;
		else if(sonar_attached) //not receive 'a', but detect audience by myself
		{
			if(sensor_value_dblchk >= RANGE2 && sensor_value_dblchk <= RANGE3) agent2.flg = true;
		}	
	}
	else
	{
		if(sonar_attached) period = 8.0; else period = 7.0;

		if(agent2.tim2 < period)
		{
			agent2.tim1 += 0.5;
			agent2.tim2 += 0.1;

			//threshold time 1: send message to neighbors
			if(agent2.tim2 > period / 4.0)
			{
				if(!wave_ping)
				{
					send_message(MESSAGE_COMMAND, ALL_DIRECTION, 1, "a");
					wave_ping = true;
				}
			}
		}

		if(agent2.tim2 >= period)
		{
			agent2.flg = false;
			wave_flg = false;
		}
	}

	ampl = agent2.tim2 * exp(-decay * agent2.tim2) * 2.7;
	angle = ampl * sin(agent2.tim1);

	return angle;
}


// ============================================================================================
// ============================================================================================
// SWARM DYNAMICS 1 --- Ken's Swarm Dynamics ---
// ============================================================================================
// ============================================================================================

// ################### DISORDERED BEHAVIOR ####################
void swarm_interaction0(int nei)
{
	float disx, disy, dis1, dis2, alph, force;
	float di, dj;
	bool flag = true;

	switch(nei)
	{
		case LEFT_BOTTOM : di=-1.000; dj= 0.000; break;
		case RIGHT_BOTTOM: di= 1.000; dj= 0.000; break;
		case BOTTOM_RIGHT: di= 0.000; dj= 1.000; break;
		case LEFT_TOP    : di= 0.000; dj=-1.000; break;
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
// --------------------------------------------------------------------------------------------
float swarm_disordered()
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

	return agent0.hd;
}

// ################### SYNCHRONIZED BEHAVIOR ####################
void swarm_interaction1(int nei)
{
	float disx, disy, dis1, dis2, alph, force;
	float di, dj;
	bool flag = true;

	switch(nei)
	{/* //in case of 6 neighbors
		case BOTTOM_RIGHT: di= 0.866; dj= 0.500; break;
		case BOTTOM_LEFT : di=-0.866; dj= 0.500; break;
		case LEFT_BOTTOM : di=-1.000; dj= 0.000; break;
		case LEFT_TOP    : di=-0.866; dj=-0.500; break;
		case RIGHT_BOTTOM: di= 1.000; dj= 0.000; break;
		case RIGHT_TOP   : di= 0.866; dj=-0.500; break;
	*/
		//in case of 4 neighbors
		case LEFT_BOTTOM : di=-1.000; dj= 0.000; break;
		case RIGHT_BOTTOM: di= 1.000; dj= 0.000; break;
		case BOTTOM_RIGHT: di= 0.000; dj= 1.000; break;
		case LEFT_TOP    : di= 0.000; dj=-1.000; break;
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
			alph = 1.0 + d1 * (disx * cos(agent1.hd) + disy * sin(agent1.hd)) / dis1; //inner product
			force = -cf * (rc1 / dis1 - 1.0) * rc1 * rc1 / dis2;
			
			forcex += alph * force * disx / dis1;
			forcey += alph * force * disy / dis1;
		}
	}
}
// --------------------------------------------------------------------------------------------
float swarm_synchronized()
{
	int i;
	float dvx, dvy, lx, ly, vabs, ds, fx, fy;
	float dir = agent1.hd;

	// ===== calculation of forces =====
	//self-propel force and viscosity
	dvx = acc * cos(dir) - gmma1 * agent1.vx;
	dvy = acc * sin(dir) - gmma1 * agent1.vy;

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
	dvx = dvx + ka1 * lx;
	dvy = dvy + ka1 * ly;

	// ===== update =====
	//direction
	vabs = sqrt(agent1.vx * agent1.vx + agent1.vy * agent1.vy);

	fx = agent1.vx / vabs;
	fy = agent1.vy / vabs;
	ds = -1.0 / tau1 * (sin(dir)*fx-cos(dir)*fy); // get sin value by cross product

	agent1.hd += ds * dt;

	if(agent1.hd > 2.0 * PI)	agent1.hd -= 2.0 * PI;
	if(agent1.hd < 0 ) 			agent1.hd += 2.0 * PI;

	//velocity
	agent1.vx += dvx * dt;
	agent1.vy += dvy * dt;


	//position
	agent1.px += agent1.vx * dt;
	agent1.py += agent1.vy * dt;

	return agent1.hd;
}

// ################### COMBINATION OF SYNC AND DISORDER BEHAVIOR ####################
float Sync_Disorder(int ClosingTime)
{
	float dir1, dir2, angle, speed = 150.0;
	int range;

	dir1 = swarm_synchronized();
	dir2 = swarm_disordered();
	send_message(MESSAGE_NUMDATA, ALL_DIRECTION, NULL, "");

	// The amplitude gradually increases, and decay in last 10 sec 
	if(sec_counter < ClosingTime-15)
	{
		decay_tim = 0;
		global_amp += 0.005; 
		if(global_amp > 1.0) global_amp = 1.0;
	}
	else
	{
		decay_tim += 0.05;
		global_amp = exp(-0.4 * decay_tim);
	}

	if(sonar_attached && use_sensor_data_on && sec_counter > STGtime4)
	{
		if(sensor_value_trichk >= RANGE1 && sensor_value_trichk < RANGE2)
		{
			send_message(MESSAGE_COMMAND, ALL_DIRECTION, 4, "c");
			sync = false;
		}
		if(sensor_value_trichk >= RANGE2 && sensor_value_trichk < RANGE3)
		{
			send_message(MESSAGE_COMMAND, ALL_DIRECTION, 2, "c");
			sync = false;
		}
		use_sensor_data_on = false;
	}

	if(sync) 	angle = global_amp * cos(dir1 * speed);
	else 		angle = global_amp * cos(dir2 * speed);

	return angle;
}

// ============================================================================================
// ============================================================================================
// SWARM DYNAMICS 6 --- Avalanche like behavior based on Ken's Swarm Dynamics ---
// ============================================================================================
// ============================================================================================
float avalanche()
{
	float angle, dir, threshold = 0.999, speed1 = 150.0, speed2 = 50.0;

	dir = swarm_disordered();
	send_message(MESSAGE_NUMDATA, ALL_DIRECTION, NULL, "");	//It should be included...

	if(sonar_attached && use_sensor_data_on)
	{
		if(sensor_value_dblchk >= RANGE1 && sensor_value_dblchk < RANGE3)
		{
			send_message(MESSAGE_COMMAND, ALL_DIRECTION, ALL, "c");
			sync = false;
		}
		use_sensor_data_on = false;
	}

	if(!sync) angle = cos(dir * speed1);
	else
	{
		angle = cos(dir * speed2);
		if(fabs(angle) < threshold) angle = 0;
	}

	return angle;
}

/*
float avalanche()
{
	static int timestep = 0;
	float tim, dir, threshold, speed = 50.0;
	
	dir = cos(swarm_disordered() * speed);

	if(timestep < 15) threshold = 0.999;
	else threshold = 0.999 - 0.005 * (timestep-15) * (timestep-15);

	if(timestep > 30) timestep = 0;

	if(rhythm_on)
	{
		timestep++;
		rhythm_on = false;
	}

	if((dir > threshold) || (dir < -threshold)) return dir;
	else return 0;
}
*/
