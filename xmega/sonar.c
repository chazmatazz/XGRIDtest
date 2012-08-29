// sonar.c

#include "avr_compiler.h"
#include "sonar.h"

void init_sonar()
{
	//**** INITIALIZE I2C *********************************************************************************
	
	//	SDA is connected to portCpin0 (Pin 16)	-> this 3.3v signal is then fed through logic level converter to 5v -> SDA5
	//	SCL is connected to portCpin1 (Pin 17)	-> this 3.3v signal is then fed through logic level converter to 5v -> SCL5
	//	SDA3 is connected to portEpin0 (Pin 36)
	//	SCL3 is connected to portEpin1 (Pin 37)
	
	/* comments on I2C protocol **		
										Ref: www.robot-electronics.co.uk/acatalog/I2C_Tutorial.html
										Ref: Fairchild Application Note 794		google:fan794.pdf
	
		Common I2C bus speeds are:			(arbitrarily low clock frequencies are also allowed)
			10 kbit/s "low-speed mode"
			100 kbit/s "standard mode"
			400 kbit/s "fast mode" (recent revision of I2C protocol, slave device may not support this) 
	
		I2C Data is transferred in sequences of 8 bits.
		The bits are placed on the SDA line starting with the MSB (Most Significant Bit).
		For every 8 bits transferred, the device receiving the data sends back an acknowledge (ACK) bit, 
		so there are actually 9 SCL clock pulses to transfer each 8 bit byte of data.
		If the receiving device sends back a low ACK bit (0), then it has received the data and is ready to accept another byte. (active response, SDA line is normally high)
		If the receiving device sends back a high ACK bit (1), then it is indicating it cannot accept any further data 
		and the master should terminate the transfer by sending a stop sequence.  
	
		I2C Device Addressing:
			Virtually all I2C addresses are 7 bits (10 bits rare). 
			It is possible to have up to 128 devices on the I2C bus, since a 7-bit number can be from 0 to 127.
			When sending out the 7-bit address, the protocol is to still always send 8 bits.
			The extra bit is used to inform the slave if the master is writing to it (0) or reading from it (1).
			The 7-bit address is placed in the upper 7 bits of the byte and the Read/Write (R/W) bit is in the LSB (Least Significant Bit).
			
			SDA:	A6		A5		A4		A3		A2		A1		A0		R/W		ACK		(address bits)
			SCL:	1		2		3		4		5		6		7		8		9		(clock pulses)
	
			The placement of the 7 bit address in the upper 7 bits of the byte may be a source of confusion. 
			(e.g. to write to address 21, you must actually send out 42 which is 21 moved over by 1 bit)
			The following alternative description may also be used: 
				I2C bus addresses are 8 bit addresses, with even addresses as write only, 
				and the odd addresses are the read address for the same device.
				
		The I2C Software Protocol:
			The first thing that will happen is that the master will send out a start sequence.
			This will alert all the slave devices on the bus that a transaction is starting and they should listen in case it is for them.
			Next the master will send out the device address.
			The slave that matches this address will continue with the transaction, any others will ignore the rest of this transaction and wait for the next.
			Having addressed the slave device the master must now send out the internal location or register number inside the slave that it wishes to write to or read from.
			This number is obviously dependent on what the slave actually is and how many internal registers it has.
			Some very simple devices do not have any, but most do.
			Having sent the I2C address and the internal register address the master can now send the data byte(s).
			The master can continue to send data bytes to the slave and these will normally be placed in the sequentially increasing registers.
			The slave should automatically increment the internal register address after each byte. 
			When the master has finished writing all data to the slave, it sends a stop sequence which completes the transaction. So to write to a slave device: 

				TO WRITE TO A SLAVE:
				1. Send a start sequence
				2. Send the I2C address of the slave with the R/W bit low (0)
				3. Send the internal register number you want to write to
				4. Send the data byte
				5. [Optionally, send any further data bytes]
				6. Send the stop sequence.	
	
			Before reading data from the slave device, you must tell it which of its internal addresses you want to read.
			So a read of the slave actually starts off by writing to it. 
			This is the same as when you want to write to it: You send the start sequence, 
			the I2C address of the slave with the R/W bit low (0), and the internal register number you want to write to. 
			Now you send another start sequence (sometimes called a restart) and the I2C address again - this time with the read bit high (1). 
			You then read as many data bytes as you wish and terminate the transaction with a stop sequence.
			
				TO READ FROM A SLAVE:
				1. Send a start sequence
				2. Send the I2C address of the slave with the R/W bit low (0)
				3. Send the internal register number you want to read from
				4. Send a start sequence again (repeated start)
				5. Send the I2C address of the slave with the R/W bit high (1)
				6. Read data byte
				7. Send the stop sequence.


	*/ // End comments on I2C protocol
	
	//TWIC.MASTER.CTRLB |= TWI_MASTER_SMEN_bm;   // Enable smart mode
	//TWIC.MASTER.CTRLB |= TWI_MASTER_QCEN_bm;   // Enable quick command
	
	TWIC.MASTER.BAUD = 155;                      // 100 kHz (Ref: doc8077.pdf, pg 221, [eqn. 2])
	//TWIC.MASTER.BAUD = 35;                     // 400 kHz
	
	TWIC.MASTER.CTRLA |= TWI_MASTER_ENABLE_bm; // Master enable
	TWIC.MASTER.STATUS |= 0x01;                // Set bus idle
}

uint16_t get_sonar_value(void)
{	
	uint8_t range_HIGH;		// high byte
	uint8_t range_LOW;		// low byte
	uint16_t range_FULL;
	
	// The default shipped address of the SRF02 is 0xE0
	// Only register location 0 can be written to
	
	//Send START + SLAVE DEVICE ADDRESS + WRITE BIT: 
	TWIC.MASTER.ADDR = 0xE0;		// 0xE0 = 0b11100000
	while (!(TWIC.MASTER.STATUS & TWI_MASTER_CLKHOLD_bm));	//wait for the outbound message to complete

/*	//IF SLAVE RETURNED NACK OR OR DID NOT REPLY AT ALL:
		//SEND ADDRESS UNTIL SLAVE RETURNS ACK.
        while(TWIC.MASTER.STATUS & TWI_MASTER_RXACK_bm)
        {
			TWIC.MASTER.ADDR = 0xE0;          // 0xE0 = 0b11100000
			while(!(TWIC.MASTER.STATUS & TWI_MASTER_CLKHOLD_bm));               
		}       */

	// send the internal register number to write to
	TWIC.MASTER.DATA = 0x00;		// 0x00 = 0b00000000
	while (!(TWIC.MASTER.STATUS & TWI_MASTER_CLKHOLD_bm));	//wait for the outbound message to complete
	
	// command the sonar to start a measurement 
	//	(Real Ranging Mode - Result in inches		- command: 0x50)
	//	(Real Ranging Mode - Result in centimeters	- command: 0x51)
	//	(Real Ranging Mode - Result in microseconds - command: 0x52)
	TWIC.MASTER.DATA = 0x51;		// 0x51 = 0b1010001
	while (!(TWIC.MASTER.STATUS & TWI_MASTER_CLKHOLD_bm));	//wait for the outbound message to complete
	
	TWIC.MASTER.CTRLC = 0x03; // issue a STOP condition

	// kill some time while sonar is out

	// CHECK FOR COMPLETION OF RANGING:

	/* Need to implement this 
		see the datasheet: www.robot-electronics.co.uk/htm/srf02techI2C.htm
		
	*/

	// READ THE RANGE DATA VIA I2C:

	// Send START + SLAVE DEVICE ADDRESS + WRITE BIT: 
	TWIC.MASTER.ADDR = 0xE0;		// 0xE0 = 0b11100000
	while (!(TWIC.MASTER.STATUS & TWI_MASTER_CLKHOLD_bm));	//wait for the outbound message to complete
	
	// send the internal register number to read from
	TWIC.MASTER.DATA = 0x02;		// 0x02 = 0b00000010
	while (!(TWIC.MASTER.STATUS & TWI_MASTER_CLKHOLD_bm));	//wait for the outbound  message to complete

	TWIC.MASTER.CTRLC = 0x03; // issue a STOP condition

	// Send START + SLAVE DEVICE ADDRESS + READ BIT: 
	TWIC.MASTER.ADDR = 0xE1;		// 0xE1 = 0b11100001
	while (!(TWIC.MASTER.STATUS & TWI_MASTER_CLKHOLD_bm));	//wait for the outbound message to complete ??
	
	// an inbound message will come in now
	
	range_HIGH = TWIC.MASTER.DATA;
	
	TWIC.MASTER.CTRLC = 0x02; // send an ACK to receive another byte

	// an inbound message will come in now
	while (!(TWIC.MASTER.STATUS & TWI_MASTER_RIF_bm));	//wait for the inbound message to complete

	range_LOW = TWIC.MASTER.DATA;
	
	TWIC.MASTER.CTRLC = 0b00000111; // send NACK followed by STOP condition	(0b00000111 = TWI_MASTER_ACKACT_bm | TWI_MASTER_CMD_STOP_gc)

	range_FULL = range_HIGH*256;	// read in the high byte
	range_FULL += range_LOW;		// append with the low byte

	return range_FULL;
}

bool check_sonar_attached(void)
{	
	uint32_t i;
	uint8_t range_HIGH;		// high byte
	uint8_t range_LOW;		// low byte
	uint16_t range_FULL;
	
	// The default shipped address of the SRF02 is 0xE0
	// Only register location 0 can be written to
	
	//Send START + SLAVE DEVICE ADDRESS + WRITE BIT: 
	TWIC.MASTER.ADDR = 0xE0;		// 0xE0 = 0b11100000
	while (!(TWIC.MASTER.STATUS & TWI_MASTER_CLKHOLD_bm));	//wait for the outbound message to complete

/*	//IF SLAVE RETURNED NACK OR OR DID NOT REPLY AT ALL:
		//SEND ADDRESS UNTIL SLAVE RETURNS ACK.
        while(TWIC.MASTER.STATUS & TWI_MASTER_RXACK_bm)
        {
			TWIC.MASTER.ADDR = 0xE0;          // 0xE0 = 0b11100000
			while(!(TWIC.MASTER.STATUS & TWI_MASTER_CLKHOLD_bm));               
		}       */

	// send the internal register number to write to
	TWIC.MASTER.DATA = 0x00;		// 0x00 = 0b00000000
	while (!(TWIC.MASTER.STATUS & TWI_MASTER_CLKHOLD_bm));	//wait for the outbound message to complete
	
	// command the sonar to start a measurement 
	//	(Real Ranging Mode - Result in inches		- command: 0x50)
	//	(Real Ranging Mode - Result in centimeters	- command: 0x51)
	//	(Real Ranging Mode - Result in microseconds - command: 0x52)
	TWIC.MASTER.DATA = 0x51;		// 0x51 = 0b1010001
	while (!(TWIC.MASTER.STATUS & TWI_MASTER_CLKHOLD_bm));	//wait for the outbound message to complete
	
	TWIC.MASTER.CTRLC = 0x03; // issue a STOP condition

	// kill some time while sonar is out

	// CHECK FOR COMPLETION OF RANGING:

	/* Need to implement this 
		see the datasheet: www.robot-electronics.co.uk/htm/srf02techI2C.htm
		
	*/

	// READ THE RANGE DATA VIA I2C:

	// Send START + SLAVE DEVICE ADDRESS + WRITE BIT: 
	TWIC.MASTER.ADDR = 0xE0;		// 0xE0 = 0b11100000
	while (!(TWIC.MASTER.STATUS & TWI_MASTER_CLKHOLD_bm));	//wait for the outbound message to complete
	
	// send the internal register number to read from
	TWIC.MASTER.DATA = 0x02;		// 0x02 = 0b00000010
	while (!(TWIC.MASTER.STATUS & TWI_MASTER_CLKHOLD_bm));	//wait for the outbound  message to complete

	TWIC.MASTER.CTRLC = 0x03; // issue a STOP condition

	// Send START + SLAVE DEVICE ADDRESS + READ BIT: 
	TWIC.MASTER.ADDR = 0xE1;		// 0xE1 = 0b11100001
	while (!(TWIC.MASTER.STATUS & TWI_MASTER_CLKHOLD_bm));	//wait for the outbound message to complete ??
	
	// an inbound message will come in now
	
	range_HIGH = TWIC.MASTER.DATA;
	
	TWIC.MASTER.CTRLC = 0x02; // send an ACK to receive another byte
	
	//i=jiffies+5; // wait 50ms
	i=0;
	// an inbound message will come in now
	while (!(TWIC.MASTER.STATUS & TWI_MASTER_RIF_bm)&& i++<1600000);	//wait for the inbound message to complete
	
	//if(jiffies>=i) return 0;

	range_LOW = TWIC.MASTER.DATA;
	
	TWIC.MASTER.CTRLC = 0b00000111; // send NACK followed by STOP condition	(0b00000111 = TWI_MASTER_ACKACT_bm | TWI_MASTER_CMD_STOP_gc)

	range_FULL = range_HIGH*256;	// read in the high byte
	range_FULL += range_LOW;		// append with the low byte
	
	if(i<1600000) return 1;
	else return 0;
}
