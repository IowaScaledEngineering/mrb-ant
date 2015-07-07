/*************************************************************************
Title:    MRBus Analog Throttle Module
Authors:  Nathan D. Holmes <maverick@drgw.net>
File:     $Id: $
License:  GNU General Public License v3

LICENSE:
    Copyright (C) 2015 Nathan Holmes

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

*************************************************************************/

/* This version of the MRB-ANT code is designed to be used with v1.x of the hardware */

#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include "mrbus.h"
#include "avr-i2c-master.h"

uint8_t mrbus_dev_addr = 0;

uint8_t pkt_count = 0;

// ******** Start 100 Hz Timer - Very Accurate Version

// Initialize a 100Hz timer for use in triggering events.
// If you need the timer resources back, you can remove this, but I find it
// rather handy in triggering things like periodic status transmissions.
// If you do remove it, be sure to yank the interrupt handler and ticks/secs as well
// and the call to this function in the main function

volatile uint8_t ticks;
volatile uint16_t decisecs=0;
volatile uint16_t update_decisecs=10;
volatile uint8_t globalTicker=0;

uint8_t basePwm1 = 0;
uint8_t basePwm2 = 0;
volatile uint8_t pulseCyclesRemaining1 = 0;
volatile uint8_t pulseCyclesRemaining2 = 0;
uint8_t pulseAmplitude1 = 0;
uint8_t pulseAmplitude2 = 0;
uint8_t pulseWidth1 = 0;
uint8_t pulseWidth2 = 0;

void initialize100HzTimer(void)
{
	// Set up timer 1 for 100Hz interrupts
	TCCR1A = 0;
	TCCR1B = _BV(CS11) | _BV(CS10);
	TCCR1C = 0;
	TIMSK1 = _BV(TOIE1);
	ticks = 0;
	decisecs = 0;
}

volatile uint8_t i2cStateMachineEnable = 0;
volatile int8_t tempSensorA = 0;
volatile int8_t tempSensorB = 0;

#define I2C_ADDR_PCA9536  0x82
#define I2C_ADDR_TC74_U8  0x90
#define I2C_ADDR_TC74_U9  0x92

typedef enum
{
	LED_OFF = 0x00,
	LED_GREEN,
	LED_RED,
	LED_YELLOW,
	LED_GREEN_SLOWBLINK,
	LED_GREEN_FASTBLINK,
	LED_RED_SLOWBLINK,
	LED_RED_FASTBLINK,
	LED_YELLOW_SLOWBLINK,
	LED_YELLOW_FASTBLINK

} LEDStatus;

LEDStatus ledChA, ledChB;

#define CH_A_GREEN 0x02
#define CH_A_RED   0x01
#define CH_B_GREEN 0x08
#define CH_B_RED   0x04

void writeLEDStatus()
{
	uint8_t i2cBuf[4];
	i2cBuf[0] = I2C_ADDR_PCA9536;
	i2cBuf[1] = 0x01; // 9536 output register
	i2cBuf[2] = 0;

	switch(ledChA)
	{
		case LED_GREEN:
		case LED_GREEN_SLOWBLINK:
		case LED_GREEN_FASTBLINK:
			i2cBuf[2] |= CH_A_GREEN;
			break;

		case LED_RED:
		case LED_RED_SLOWBLINK:
		case LED_RED_FASTBLINK:
			i2cBuf[2] |= CH_A_RED;
			break;

		case LED_YELLOW:
		case LED_YELLOW_SLOWBLINK:
		case LED_YELLOW_FASTBLINK:
			i2cBuf[2] |= CH_A_GREEN | CH_A_RED;
			break;

		case LED_OFF:
		default:
			break;
	}

	switch(ledChA)
	{

		case LED_GREEN_SLOWBLINK:
		case LED_RED_SLOWBLINK:
		case LED_YELLOW_SLOWBLINK:
			if (0 == (globalTicker & 0x04))
				i2cBuf[2] &= ~(CH_A_GREEN | CH_A_RED);
			break;

		case LED_GREEN_FASTBLINK:
		case LED_RED_FASTBLINK:
		case LED_YELLOW_FASTBLINK:
			if (0 == (globalTicker & 0x02))
				i2cBuf[2] &= ~(CH_A_GREEN | CH_A_RED);
			break;


		default:
			break;
	}

	switch(ledChB)
	{
		case LED_GREEN:
		case LED_GREEN_SLOWBLINK:
		case LED_GREEN_FASTBLINK:
			i2cBuf[2] |= CH_B_GREEN;
			break;

		case LED_RED:
		case LED_RED_SLOWBLINK:
		case LED_RED_FASTBLINK:
			i2cBuf[2] |= CH_B_RED;
			break;

		case LED_YELLOW:
		case LED_YELLOW_SLOWBLINK:
		case LED_YELLOW_FASTBLINK:
			i2cBuf[2] |= CH_B_GREEN | CH_B_RED;
			break;

		case LED_OFF:
		default:
			break;
	}

	switch(ledChB)
	{

		case LED_GREEN_SLOWBLINK:
		case LED_RED_SLOWBLINK:
		case LED_YELLOW_SLOWBLINK:
			if (0 == (globalTicker & 0x04))
				i2cBuf[2] &= ~(CH_B_GREEN | CH_B_RED);
			break;

		case LED_GREEN_FASTBLINK:
		case LED_RED_FASTBLINK:
		case LED_YELLOW_FASTBLINK:
			if (0 == (globalTicker & 0x02))
				i2cBuf[2] &= ~(CH_B_GREEN | CH_B_RED);
			break;


		default:
			break;
	}
	
	// Got it - send to part
	i2c_transmit(i2cBuf, 3, 1);
	while(i2c_busy());
}

void indicatorsConfigure()
{
	uint8_t i2cBuf[4];
	i2cBuf[0] = I2C_ADDR_PCA9536;
	i2cBuf[1] = 0x03; // 9536 configuration register
	i2cBuf[2] = 0; // All outputs
	i2c_transmit(i2cBuf, 3, 1);
	while(i2c_busy());

	i2cBuf[0] = I2C_ADDR_PCA9536;
	i2cBuf[1] = 0x01; // 9536 configuration register
	i2cBuf[2] = 0x00; // All outputs
	i2c_transmit(i2cBuf, 3, 1);
	while(i2c_busy());

}


void tempSensorConfigure(uint8_t sensorAddr)
{
	uint8_t i2cBuf[4];
	i2cBuf[0] = sensorAddr;
	i2cBuf[1] = 0x01; // Configuration register
	i2cBuf[2] = 0x00; // Take it out of standby, just in case
	i2c_transmit(i2cBuf, 3, 1);
	while(i2c_busy());

	i2cBuf[0] = sensorAddr;
	i2cBuf[1] = 0x00; // Set address back to data register
	i2c_transmit(i2cBuf, 2, 1);
	while(i2c_busy());
}

void i2cDevicesConfigure()
{
	indicatorsConfigure();
	tempSensorConfigure(I2C_ADDR_TC74_U8);
	tempSensorConfigure(I2C_ADDR_TC74_U9);
}


ISR(TIMER1_OVF_vect)
{
	static uint8_t nextI2CState = 0;
	TCNT1 += 0xF3CB;
	
	if (pulseAmplitude1 && pulseWidth1)
		pulseCyclesRemaining1 = pulseWidth1;

	if (pulseAmplitude2 && pulseWidth2)
		pulseCyclesRemaining2 = pulseWidth2;		
	
	if (++ticks >= 10)
	{
		ticks = 0;
		decisecs++;
		globalTicker++;
		if (0 == i2cStateMachineEnable)
			i2cStateMachineEnable = 1 + (0x07 & (nextI2CState++));
	}


}

#define VMAX_FULL_PWM 180  // In decivolts

uint8_t decivoltsToPwm(uint8_t decivolts)
{
	return((uint16_t)255 * decivolts) / VMAX_FULL_PWM;
}

uint8_t pwmToDecivolts(uint8_t pwm)
{
	return ((uint16_t)pwm * VMAX_FULL_PWM) / 255;
}

void speedToVoltsAndPWM(uint8_t speed, uint8_t vMax, uint8_t vPwmCutoff, uint8_t* basePwm, uint8_t* pulseAmplitude, uint8_t* pulseWidth)
{
	uint8_t pwmMax = 255;
	uint8_t vPwmMax=0;

	// Sanity tests - bound things into sane ranges
	vPwmMax = vMax = min(vMax, VMAX_FULL_PWM);
	speed = min(127, speed);

	pwmMax = decivoltsToPwm(vMax);

	//Momentum calculations go here
	
	// Speed is between 0-0x7F to match 128-step DCC
	*basePwm = ((uint16_t)pwmMax * speed) / 127;

	if (0 == *basePwm)
	{
		*pulseAmplitude = 0;
		*pulseWidth = 0;
	}
	else if (*basePwm < decivoltsToPwm(vPwmCutoff)) // Below pulse cutoff voltage
	{
		uint8_t vOut = pwmToDecivolts(*basePwm);
		uint8_t vPulse = vOut * (vPwmCutoff - vPwmMax) / vPwmCutoff + vPwmMax; // <- Makes the pulses go to equality with output voltage at cutoff
		
		*pulseAmplitude = decivoltsToPwm(vPulse);
		*pulseWidth = 75;
	}
	else
	{
		*pulseAmplitude = 0;
		*pulseWidth = 0;
	}
}

#define CS_DAC (PB0)

#define DIR_RELAY_A  (PC2)
#define DIR_RELAY_B  (PC3)
#define ACC_RELAY_A  (PB1)
#define ACC_RELAY_B  (PB2)


void initializeOutput(void)
{
	// Set up timer 0 for PWM operations
	TCNT0 = 0;
	TCCR0A = 0;
	TCCR0B = _BV(CS00);
	TIMSK0 = _BV(TOIE0);

	DDRB |= _BV(CS_DAC) | _BV(PB3) | _BV(PB5) | _BV(ACC_RELAY_A) | _BV(ACC_RELAY_B);
	PORTB |= _BV(CS_DAC);
	PORTB &= ~(_BV(ACC_RELAY_A) | _BV(ACC_RELAY_B));

	DDRC |= _BV(DIR_RELAY_A) | _BV(DIR_RELAY_B);
	PORTC &= ~(_BV(DIR_RELAY_A) | _BV(DIR_RELAY_B));

	DDRD &= ~(_BV(PD3) | _BV(PD4) | _BV(PD5) | _BV(PD6) | _BV(PD7));

//	SPSR = _BV(SPI2X);
	SPCR = _BV(SPE) | _BV(MSTR) | _BV(SPR0);
}



ISR(TIMER0_OVF_vect)
{
	static uint8_t phase = 0;
	static uint8_t dacVal = 0;

	switch(phase & 0x03)
	{
		case 0:
			PORTB |= _BV(CS_DAC); // Raise /CS on DAC 2
			PORTB &= ~_BV(CS_DAC); // Drop /CS on DAC 1
			dacVal = (pulseCyclesRemaining1)?pulseAmplitude1:basePwm1;
			SPDR = 0b00110000 | (0x0F & (dacVal>>4));
			break;
		case 1:
			SPDR = (0xF0 & (dacVal<<4));
			break;

		case 2:
			PORTB |= _BV(CS_DAC); // Raise /CS on DAC 1
			PORTB &= ~_BV(CS_DAC); // Drop /CS on DAC 2
			dacVal = (pulseCyclesRemaining2)?pulseAmplitude2:basePwm2;
			SPDR = 0b00110000 | (0x0F & (dacVal>>4));
			break;

		case 3:
			SPDR = (0xF0 & (dacVal<<4));
			break;
	}
	
	phase++;

	if (0 != pulseCyclesRemaining1)
		pulseCyclesRemaining1--;

	if (0 != pulseCyclesRemaining2)
		pulseCyclesRemaining2--;		
}

// **** Bus Voltage Monitor
/*
// Uncomment this block (and the ADC initialization in the init() function) if you want to continuously monitor bus voltage

volatile uint8_t busVoltage=0;

ISR(ADC_vect)
{
	static uint16_t busVoltageAccum=0;
	static uint8_t busVoltageCount=0;

	busVoltageAccum += ADC;
	if (++busVoltageCount >= 64)
	{
		busVoltageAccum = busVoltageAccum / 64;
        //At this point, we're at (Vbus/6) / 5 * 1024
        //So multiply by 300, divide by 1024, or multiply by 150 and divide by 512
        busVoltage = ((uint32_t)busVoltageAccum * 150) / 512;
		busVoltageAccum = 0;
		busVoltageCount = 0;
	}
}
*/

uint8_t requestedSpeed1 = 0;
uint8_t requestedSpeed2 = 0;

uint8_t requestedDirection1 = 0;
uint8_t requestedDirection2 = 0;

#define UPDATE_SPEED1 0
#define UPDATE_SPEED2 1

uint8_t updates = 0;

void PktHandler(void)
{
	uint16_t crc = 0;
	uint8_t i;
	uint8_t rxBuffer[MRBUS_BUFFER_SIZE];
	uint8_t txBuffer[MRBUS_BUFFER_SIZE];

	if (0 == mrbusPktQueuePop(&mrbusRxQueue, rxBuffer, sizeof(rxBuffer)))
		return;


	//*************** PACKET FILTER ***************
	// Loopback Test - did we send it?  If so, we probably want to ignore it
	if (rxBuffer[MRBUS_PKT_SRC] == mrbus_dev_addr) 
		goto	PktIgnore;

	// Destination Test - is this for us or broadcast?  If not, ignore
	if (0xFF != rxBuffer[MRBUS_PKT_DEST] && mrbus_dev_addr != rxBuffer[MRBUS_PKT_DEST]) 
		goto	PktIgnore;
	
	// CRC16 Test - is the packet intact?
	for(i=0; i<rxBuffer[MRBUS_PKT_LEN]; i++)
	{
		if ((i != MRBUS_PKT_CRC_H) && (i != MRBUS_PKT_CRC_L)) 
			crc = mrbusCRC16Update(crc, rxBuffer[i]);
	}
	if ((UINT16_HIGH_BYTE(crc) != rxBuffer[MRBUS_PKT_CRC_H]) || (UINT16_LOW_BYTE(crc) != rxBuffer[MRBUS_PKT_CRC_L]))
		goto	PktIgnore;
		
	//*************** END PACKET FILTER ***************


	//*************** PACKET HANDLER - PROCESS HERE ***************

	// Just smash the transmit buffer if we happen to see a packet directed to us
	// that requires an immediate response
	//
	// If we're in here, then either we're transmitting, then we can't be 
	// receiving from someone else, or we failed to transmit whatever we were sending
	// and we're waiting to try again.  Either way, we're not going to corrupt an
	// in-progress transmission.
	//
	// All other non-immediate transmissions (such as scheduled status updates)
	// should be sent out of the main loop so that they don't step on things in
	// the transmit buffer
	
	if ('A' == rxBuffer[MRBUS_PKT_TYPE])
	{
		// PING packet
		txBuffer[MRBUS_PKT_DEST] = rxBuffer[MRBUS_PKT_SRC];
		txBuffer[MRBUS_PKT_SRC] = mrbus_dev_addr;
		txBuffer[MRBUS_PKT_LEN] = 6;
		txBuffer[MRBUS_PKT_TYPE] = 'a';
		mrbusPktQueuePush(&mrbusTxQueue, txBuffer, txBuffer[MRBUS_PKT_LEN]);
		goto PktIgnore;
	} 
	else if ('W' == rxBuffer[MRBUS_PKT_TYPE]) 
	{
		// EEPROM WRITE Packet
		txBuffer[MRBUS_PKT_DEST] = rxBuffer[MRBUS_PKT_SRC];
		txBuffer[MRBUS_PKT_LEN] = 8;			
		txBuffer[MRBUS_PKT_TYPE] = 'w';
		eeprom_write_byte((uint8_t*)(uint16_t)rxBuffer[6], rxBuffer[7]);
		txBuffer[6] = rxBuffer[6];
		txBuffer[7] = rxBuffer[7];
		if (MRBUS_EE_DEVICE_ADDR == rxBuffer[6])
			mrbus_dev_addr = eeprom_read_byte((uint8_t*)MRBUS_EE_DEVICE_ADDR);
		txBuffer[MRBUS_PKT_SRC] = mrbus_dev_addr;
		mrbusPktQueuePush(&mrbusTxQueue, txBuffer, txBuffer[MRBUS_PKT_LEN]);
		goto PktIgnore;	
	}
	else if ('R' == rxBuffer[MRBUS_PKT_TYPE]) 
	{
		// EEPROM READ Packet
		txBuffer[MRBUS_PKT_DEST] = rxBuffer[MRBUS_PKT_SRC];
		txBuffer[MRBUS_PKT_SRC] = mrbus_dev_addr;
		txBuffer[MRBUS_PKT_LEN] = 8;			
		txBuffer[MRBUS_PKT_TYPE] = 'r';
		txBuffer[6] = rxBuffer[6];
		txBuffer[7] = eeprom_read_byte((uint8_t*)(uint16_t)rxBuffer[6]);			
		mrbusPktQueuePush(&mrbusTxQueue, txBuffer, txBuffer[MRBUS_PKT_LEN]);
		goto PktIgnore;
	}
	else if ('V' == rxBuffer[MRBUS_PKT_TYPE]) 
	{
		// Version
		txBuffer[MRBUS_PKT_DEST] = rxBuffer[MRBUS_PKT_SRC];
		txBuffer[MRBUS_PKT_SRC] = mrbus_dev_addr;
		txBuffer[MRBUS_PKT_LEN] = 16;
		txBuffer[MRBUS_PKT_TYPE] = 'v';
		txBuffer[6]  = MRBUS_VERSION_WIRED;
		txBuffer[7]  = 0; // Software Revision
		txBuffer[8]  = 0; // Software Revision
		txBuffer[9]  = 0; // Software Revision
		txBuffer[10]  = 0; // Hardware Major Revision
		txBuffer[11]  = 0; // Hardware Minor Revision
		txBuffer[12] = 'A';
		txBuffer[13] = 'N';
		txBuffer[14] = 'T';
		txBuffer[15] = ' ';
		mrbusPktQueuePush(&mrbusTxQueue, txBuffer, txBuffer[MRBUS_PKT_LEN]);
		goto PktIgnore;
	}
	else if ('X' == rxBuffer[MRBUS_PKT_TYPE]) 
	{
		// Reset
		cli();
		wdt_reset();
		MCUSR &= ~(_BV(WDRF));
		WDTCSR |= _BV(WDE) | _BV(WDCE);
		WDTCSR = _BV(WDE);
		while(1);  // Force a watchdog reset
		sei();
	}
	else if ('C' == rxBuffer[MRBUS_PKT_TYPE] && 0xFF != rxBuffer[MRBUS_PKT_SRC])
	{
		if (rxBuffer[MRBUS_PKT_SRC] == eeprom_read_byte((uint8_t*)0x10))
		{
			requestedSpeed1 = rxBuffer[7];
			requestedDirection1 = rxBuffer[6];
			updates |= _BV(UPDATE_SPEED1);
		} 
		else if (rxBuffer[MRBUS_PKT_SRC] == eeprom_read_byte((uint8_t*)0x11))
		{
			requestedSpeed2 = rxBuffer[7];
			requestedDirection2 = rxBuffer[6];
			updates |= _BV(UPDATE_SPEED2);
		}
	}

	// FIXME:  Insert code here to handle incoming packets specific
	// to the device.

	//*************** END PACKET HANDLER  ***************

	
	//*************** RECEIVE CLEANUP ***************
PktIgnore:
	// Yes, I hate gotos as well, but sometimes they're a really handy and efficient
	// way to jump to a common block of cleanup code at the end of a function 

	// This section resets anything that needs to be reset in order to allow us to receive
	// another packet.  Typically, that's just clearing the MRBUS_RX_PKT_READY flag to 
	// indicate to the core library that the mrbus_rx_buffer is clear.
	return;	
}

void init(void)
{
	// FIXME:  Do any initialization you need to do here.
	
	// Clear watchdog (in the case of an 'X' packet reset)
	MCUSR = 0;
#ifdef ENABLE_WATCHDOG
	// If you don't want the watchdog to do system reset, remove this chunk of code
	wdt_reset();
	WDTCSR |= _BV(WDE) | _BV(WDCE);
	WDTCSR = _BV(WDE) | _BV(WDP2) | _BV(WDP1); // Set the WDT to system reset and 1s timeout
	wdt_reset();
#else
	wdt_reset();
	wdt_disable();
#endif	

	pkt_count = 0;

	// Initialize MRBus address from EEPROM
	mrbus_dev_addr = eeprom_read_byte((uint8_t*)MRBUS_EE_DEVICE_ADDR);
	// Bogus addresses, fix to default address
	if (0xFF == mrbus_dev_addr || 0x00 == mrbus_dev_addr)
	{
		mrbus_dev_addr = 0x03;
		eeprom_write_byte((uint8_t*)MRBUS_EE_DEVICE_ADDR, mrbus_dev_addr);
	}
	
	update_decisecs = (uint16_t)eeprom_read_byte((uint8_t*)MRBUS_EE_DEVICE_UPDATE_L) 
		| (((uint16_t)eeprom_read_byte((uint8_t*)MRBUS_EE_DEVICE_UPDATE_H)) << 8);

	// This assures that update_decisecs is at least 0.1s and a maximum of 4.0s
	update_decisecs = max(1, update_decisecs);
	update_decisecs = min(40, update_decisecs);


/*
// Uncomment this block to set up the ADC to continuously monitor the bus voltage using a 3:1 divider tied into the ADC7 input
// You also need to uncomment the ADC ISR near the top of the file
	// Setup ADC
	ADMUX  = 0x47;  // AVCC reference; ADC7 input
	ADCSRA = _BV(ADATE) | _BV(ADIF) | _BV(ADPS2) | _BV(ADPS1); // 128 prescaler
	ADCSRB = 0x00;
	DIDR0  = 0x00;  // No digitals were harmed in the making of this ADC

	busVoltage = 0;
	ADCSRA |= _BV(ADEN) | _BV(ADSC) | _BV(ADIE) | _BV(ADIF);
*/
}

#define MRBUS_TX_BUFFER_DEPTH 4
#define MRBUS_RX_BUFFER_DEPTH 8

MRBusPacket mrbusTxPktBufferArray[MRBUS_TX_BUFFER_DEPTH];
MRBusPacket mrbusRxPktBufferArray[MRBUS_RX_BUFFER_DEPTH];


int8_t tempChA;
int8_t tempChB;


int8_t readTempSensor(uint8_t sensorAddress)
{
	uint8_t i2cBuf[2];

	i2cBuf[0] = sensorAddress;
	i2cBuf[1] = 0x00;
	i2c_transmit(i2cBuf, 2, 0);
	while(i2c_busy());

	i2cBuf[0] = sensorAddress | 0x01;
	i2cBuf[1] = 0x00;
	i2c_transmit(i2cBuf, 2, 1);
	while(i2c_busy());
	if(i2c_receive(i2cBuf, 2))
		return((int8_t)i2cBuf[1]);
	
	return(0);
}


int main(void)
{
	// Application initialization
	init();

	// Initialize a 100 Hz timer.  See the definition for this function - you can
	// remove it if you don't use it.
	initialize100HzTimer();
	
	initializeOutput();
	i2c_master_init();
	// Initialize MRBus core
	mrbusPktQueueInitialize(&mrbusTxQueue, mrbusTxPktBufferArray, MRBUS_TX_BUFFER_DEPTH);
	mrbusPktQueueInitialize(&mrbusRxQueue, mrbusRxPktBufferArray, MRBUS_RX_BUFFER_DEPTH);
	mrbusInit();

	sei();	

#define VMAX         180
#define VPWM_CUTOFF   40
	i2cDevicesConfigure();

	ledChA = LED_RED;
	ledChB = LED_RED;
	while (1)
	{
		wdt_reset();

		if (i2cStateMachineEnable)	
		{
			// Start I2C stuff based on phase
			switch(i2cStateMachineEnable)
			{
				case 1:
				case 3:
				case 5:
				case 7:
					writeLEDStatus();
					break;

				case 2:
					tempChA = readTempSensor(I2C_ADDR_TC74_U8);
					break;

				case 6:
					tempChB = readTempSensor(I2C_ADDR_TC74_U9);
					break;
		

				default:
					break;
			}

			i2cStateMachineEnable = 0;
		}

		// Handle any packets that may have come in
		if (mrbusPktQueueDepth(&mrbusRxQueue))
			PktHandler();
			
		// FIXME: Do any module-specific behaviours here in the loop.
		
		if (updates & _BV(UPDATE_SPEED1))
		{
			updates &= ~_BV(UPDATE_SPEED1);
			if (0 == requestedDirection1)
			{
				requestedSpeed1 = 0;
			}
			else if (requestedDirection1 & 0x01)
			{
				PORTC |= _BV(DIR_RELAY_A);
			}
			else if (requestedDirection1 & 0x02)
			{
				PORTC &= ~_BV(DIR_RELAY_A);
			}
			
			speedToVoltsAndPWM(requestedSpeed1, VMAX, VPWM_CUTOFF, &basePwm1, &pulseAmplitude1, &pulseWidth1);
		}
		
		if (updates & _BV(UPDATE_SPEED2))
		{
			updates &= ~_BV(UPDATE_SPEED2);
			if (0 == requestedDirection2)
			{
				requestedSpeed2 = 0;
			}
			else if (requestedDirection2 & 0x01)
			{
				PORTC |= _BV(DIR_RELAY_B);
			}
			else if (requestedDirection2 & 0x02)
			{
				PORTC &= ~_BV(DIR_RELAY_B);
			}
			
			speedToVoltsAndPWM(requestedSpeed2, VMAX, VPWM_CUTOFF, &basePwm2, &pulseAmplitude2, &pulseWidth2);
			

		}

		
		
		if (decisecs >= update_decisecs && !(mrbusPktQueueFull(&mrbusTxQueue)))
		{
			uint8_t txBuffer[MRBUS_BUFFER_SIZE];

			txBuffer[MRBUS_PKT_SRC] = mrbus_dev_addr;
			txBuffer[MRBUS_PKT_DEST] = 0xFF;
			txBuffer[MRBUS_PKT_LEN] = 10;
			txBuffer[5] = 'S';
//			txBuffer[6] = requestedSpeed1;
//			txBuffer[7] = requestedDirection1;
			txBuffer[6] = tempChA;
			txBuffer[7] = tempChB;
			txBuffer[8] = requestedSpeed2;
			txBuffer[9] = requestedDirection2;
			mrbusPktQueuePush(&mrbusTxQueue, txBuffer, txBuffer[MRBUS_PKT_LEN]);
			decisecs = 0;
		}	

		if (mrbusPktQueueDepth(&mrbusTxQueue))
		{
			uint8_t fail = mrbusTransmit();

			// If we're here, we failed to start transmission due to somebody else transmitting
			// Given that our transmit buffer is full, priority one should be getting that data onto
			// the bus so we can start using our tx buffer again.  So we stay in the while loop, trying
			// to get bus time.

			// We want to wait 20ms before we try a retransmit to avoid hammering the bus
			// Because MRBus has a minimum packet size of 6 bytes @ 57.6kbps,
			// need to check roughly every millisecond to see if we have a new packet
			// so that we don't miss things we're receiving while waiting to transmit
			if (fail)
			{
				uint8_t bus_countdown = 20;
				while (bus_countdown-- > 0 && !mrbusIsBusIdle())
				{
					wdt_reset();
					_delay_ms(1);
					if (mrbusPktQueueDepth(&mrbusRxQueue))
						PktHandler();
				}
			}
		}
	}
}



