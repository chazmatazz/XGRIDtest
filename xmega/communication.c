// ============================================================================================
// send packet
// ============================================================================================
void send_message(uint8_t MessageType, uint8_t direction, int dist, const char str[])
{
	Xgrid::Packet pkt;
	pkt.type = MessageType;
	pkt.flags = 0;	

	switch(MessageType)
	{
		case MESSAGE_COMMAND:
			pkt.data = (uint8_t*)str;
			pkt.radius = dist;
			pkt.data_len = sizeof(str);
			break;

		case MESSAGE_NUMDATA:
			mdata.x0 = agent0.px;	mdata.y0 = agent0.py;
			mdata.x1 = agent1.px;	mdata.y1 = agent1.py;

			pkt.data = (uint8_t *)&mdata;
			pkt.radius = dist;
			pkt.data_len = sizeof(point);
			break;
	}
									
	xgrid.send_packet(&pkt, direction);	// send to all neighbors

	//LED_PORT.OUTTGL = LED_USR_2_PIN_bm;	//green LED
}

// ============================================================================================
// receive packet
// ============================================================================================
void rx_pkt(Xgrid::Packet *pkt)
{
	uint8_t port = pkt->rx_node;	//pkt->rx_node
	char command;

	if(port >= 0 && port < NUM_NEIGHBORS){

		if(pkt->type == MESSAGE_COMMAND)
		{
			char* char_ptr = (char*) pkt->data;
			command = *char_ptr;
			
			connected[port] = true;

			switch(command)
			{
				case 'Z': reboot_on = true;

				case 'D':	speedup_on = false;	communication_on = false;	
							disable_servo();	break;
				case 'R':	speedup_on = true;	communication_on = true;
							enable_servo();		break;

				case 'i':	init_variables();	break;
				case 'r':	sec_counter = 0;	break;
				case 'I':	init_variables();	sec_counter = 0;	break;

				case 'a':	wave_flg = true;	wave_port = port;	break;
				case 'b':	column_flg = true;	break;

				case 'c':	sync = false;		break;

				case '1':	sec_counter = STGtime1;	init_variables();	break;
				case '2':	sec_counter = STGtime2;	init_variables();	break;
				case '3':	sec_counter = STGtime3;	init_variables();	break;
				case '4':	sec_counter = STGtime4;	init_variables();	break;
				case '5':	sec_counter = STGtime5;	init_variables();	break;
				case '6':	sec_counter = STGtime6;	init_variables();	break;
				case '7':	sec_counter = STGtime7;	init_variables();	break;
				case '8':	sec_counter = STGtime8;	init_variables();	break;
				case '9':	sec_counter = STGtime9;	init_variables();	break;
				case '0':	sec_counter = LASTtime;	init_variables();	break;

			}
		}
		
		if(pkt->type == MESSAGE_NUMDATA)
		{
			connected[port] = true;

			point* pt_ptr = (point*) pkt->data;

			agent0.neix[port] = pt_ptr->x0;
			agent0.neiy[port] = pt_ptr->y0;
			
			agent1.neix[port] = pt_ptr->x1;
			agent1.neiy[port] = pt_ptr->y1;
		}
		
		//LED_PORT.OUTTGL = LED_USR_2_PIN_bm;	//green LED
	}
}

// ============================================================================================
// KEY INPUT PROCESS
// ============================================================================================
void key_input()
{
	char input_char = 0;
	
	if (usart.available()) input_char = usart.get();
	
	if(input_char == 0x1b) xboot_reset(); //reboot the board

	if(input_char == 'Z')	//Reboot whole system
	{
		send_message(MESSAGE_COMMAND, ALL_DIRECTION, ALL, "Z");
		temp_time = jiffies + 3000;
		while(jiffies < temp_time)
		{
			LED_PORT.OUTTGL = LED_USR_1_PIN_bm; _delay_ms(100);
		}
		xboot_reset();
	}

	if(input_char == 'D')	//Debug Mode
	{
		speedup_on = false;
		communication_on = false;
		send_message(MESSAGE_COMMAND, ALL_DIRECTION, ALL, "D");
	}

	if(input_char == 'R')	//Run Mode
	{
		speedup_on = true;
		communication_on = true;
		send_message(MESSAGE_COMMAND, ALL_DIRECTION, ALL, "R");
	}

	if(input_char == 'v')
		fprintf_P(&usart_stream, PSTR("build number = %ld\r\n"), (unsigned long) &__BUILD_NUMBER);

	if(input_char == 'd')
	{
		if(display) display = false;
		else		display = true;
	}

	if(input_char == 'I')	//initialization
	{
		send_message(MESSAGE_COMMAND, ALL_DIRECTION, ALL, "I");	
		init_variables();
		sec_counter = 0;
	}

	if(input_char == 'i')	//reset variables
	{
		send_message(MESSAGE_COMMAND, ALL_DIRECTION, ALL, "i");	
		init_variables();
	}

	if(input_char == 'r')	//set sec_counter as 0
	{
		send_message(MESSAGE_COMMAND, ALL_DIRECTION, ALL, "r");	
		sec_counter = 0;
	}

	if(input_char == '1')
	{
		send_message(MESSAGE_COMMAND, ALL_DIRECTION, ALL, "1");	
		init_variables();
		sec_counter = STGtime1;
	}

	if(input_char == '2')
	{
		send_message(MESSAGE_COMMAND, ALL_DIRECTION, ALL, "2");	
		init_variables();
		sec_counter = STGtime2;
	}

	if(input_char == '3')
	{
		send_message(MESSAGE_COMMAND, ALL_DIRECTION, ALL, "3");	
		init_variables();
		sec_counter = STGtime3;
	}

	if(input_char == '4')	//initialization
	{
		send_message(MESSAGE_COMMAND, ALL_DIRECTION, ALL, "4");	
		init_variables();
		sec_counter = STGtime4;
	}

	if(input_char == '5')	//initialization
	{
		send_message(MESSAGE_COMMAND, ALL_DIRECTION, ALL, "5");	
		init_variables();
		sec_counter = STGtime5;
	}

	if(input_char == '6')	//initialization
	{
		send_message(MESSAGE_COMMAND, ALL_DIRECTION, ALL, "6");	
		init_variables();
		sec_counter = STGtime6;
	}

	if(input_char == '7')	//initialization
	{
		send_message(MESSAGE_COMMAND, ALL_DIRECTION, ALL, "7");	
		init_variables();
		sec_counter = STGtime7;
	}

	if(input_char == '8')	//initialization
	{
		send_message(MESSAGE_COMMAND, ALL_DIRECTION, ALL, "8");	
		init_variables();
		sec_counter = STGtime8;
	}

	if(input_char == '9')	//initialization
	{
		send_message(MESSAGE_COMMAND, ALL_DIRECTION, ALL, "9");	
		init_variables();
		sec_counter = STGtime9;
	}

	if(input_char == '0')	//initialization
	{
		send_message(MESSAGE_COMMAND, ALL_DIRECTION, ALL, "0");	
		init_variables();
		sec_counter = LASTtime;
	}
}
