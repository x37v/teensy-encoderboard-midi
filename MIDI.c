/*
 * Modified by Alex Norman 6/3/2009 to work with the Encoder Board
 LUFA Library
 Copyright (C) Dean Camera, 2009.

 dean [at] fourwalledcubicle [dot] com
 www.fourwalledcubicle.com
 */

/*
	Copyright 2009  Dean Camera (dean [at] fourwalledcubicle [dot] com)

	Permission to use, copy, modify, and distribute this software
	and its documentation for any purpose and without fee is hereby
	granted, provided that the above copyright notice appear in all
	copies and that both that the copyright notice and this
	permission notice and warranty disclaimer appear in supporting
	documentation, and that the name of the author not be used in
	advertising or publicity pertaining to distribution of the
	software without specific, written prior permission.

	The author disclaim all warranties with regard to this
	software, including all implied warranties of merchantability
	and fitness.  In no event shall the author be liable for any
	special, indirect or consequential damages or any damages
	whatsoever resulting from loss of use, data or profits, whether
	in an action of contract, negligence or other tortious action,
	arising out of or in connection with the use or performance of
	this software.
	*/

/** \file
 *
 *  Main source file for the MIDI input demo. This file contains the main tasks of the demo and
 *  is responsible for the initial application hardware configuration.
 */

#include "MIDI.h"
#include "RingBuff.h"
#include <util/delay.h>
#include <avr/eeprom.h>

/* Scheduler Task List */
TASK_LIST
{
	{ .Task = USB_USBTask          , .TaskStatus = TASK_STOP },
		{ .Task = USB_MIDI_Task        , .TaskStatus = TASK_STOP },
		{ .Task = SHIFT_REG_Task        , .TaskStatus = TASK_STOP },
};

//hold the data to send
RingBuff_t Tx_Buffer;

#define HISTORY 4
#define NUMBOARDS 2
#define ENC_BTN_CC_OFFSET 16
#define BTN_CC_OFFSET 32

volatile uint8_t encoder_hist[2 * NUMBOARDS][HISTORY];
volatile uint8_t button_hist[2 * NUMBOARDS][HISTORY];
volatile uint8_t history = 0;

volatile uint8_t encoder_last[2 * NUMBOARDS];
volatile uint8_t button_last[2 * NUMBOARDS];

//eeprom stuff
button_t EEMEM button_settings[4 * NUMBOARDS];
encoder_t EEMEM encoder_settings[8 * NUMBOARDS];

void tick_clock(void){
	PORTD &= ~_BV(PORTD1);
	PORTD |= _BV(PORTD1);
}

void latch_data(void){
	//allow parallel in
	PORTD &= ~_BV(PORTD4);
	//disable parallel in
	PORTD |= _BV(PORTD4);
}

typedef enum {ENC, ENC_BTN, BTN} controller_t;

uint8_t cc_num(controller_t t, uint8_t index, uint8_t board){
	switch(t){
		case ENC:
			//remap
			//odd indicies are 2nd row.
			if (index & 0x1)
				return ((NUMBOARDS + board) << 2) + (index >> 1);
			else
				return (index >> 1) + board * 4;
		case ENC_BTN:
			if (index & 0x1)
				return ((NUMBOARDS + board) << 2) + (index >> 1) + ENC_BTN_CC_OFFSET;
			else
				return (index >> 1) + (board << 2) + ENC_BTN_CC_OFFSET;
		case BTN:
			return index + (board << 2) + BTN_CC_OFFSET;
		default:
			break;
	}
	return 127;
}

int8_t decode(uint8_t enc_current, uint8_t enc_last){

	// -> 00 -> 01 -> 11 -> 10 -> 00
	// enc_last1 enc_last0 enc_current1 enc_current0
	switch((enc_current & 0x3) | ((enc_last & 0x3) << 2))
	{
		case 0x0: // 0000 0
		case 0x5: // 0101 5
		case 0xa: // 1010 a
		case 0xf: // 1111 f
			return 0;

		case 0x1: // 0001 1
		case 0x7: // 0111 7
		case 0xe: // 1110 e
		case 0x8: // 1000 8
			return 1;

		case 0x2: // 0010 2
		case 0xb: // 1011 b
		case 0xd: // 1101 d
		case 0x4: // 0100 4
			return -1;

		case 0x3: // 0011 3
		case 0xc: // 1100 c
		case 0x6: // 0110 6
		case 0x9: // 1001 9
			//XXX ERROR
			return 0;
	}
	return 0;
}

/** Main program entry point. This routine configures the hardware required by the application, then
 *  starts the scheduler to run the application tasks.
 */
int main(void)
{
	uint8_t i;

	_delay_ms(100);
	/* Disable watchdog if enabled by bootloader/fuses */
	MCUSR &= ~(1 << WDRF);
	wdt_disable();

	/* Disable clock division */
	clock_prescale_set(clock_div_1);

	//init ringbuffer
	Buffer_Initialize(&Tx_Buffer);

	//init ports [d6 is the LED]
	DDRD |= _BV(PIND1) | _BV(PIND4) | _BV(PIND6);
	DDRD &= ~_BV(PIND0);

	//initially everything is 'up'
	for(i = 0; i < (2 * NUMBOARDS); i++)
		encoder_last[i] = button_last[i] = 0xFF;

	for(i = 0; i < 4 * NUMBOARDS; i++){
		button_t setting;
		setting.chan = i;
		setting.num = i;

		eeprom_busy_wait();
		//XXX the src + dst changes.. i'm using libc 1.6.2
		eeprom_write_block((void *)(&(button_settings[i])), (void *)&setting, sizeof(button_t));
	}


	/* Indicate USB not ready */
	UpdateStatus(Status_USBNotReady);

	/* Initialize Scheduler so that it can be used */
	Scheduler_Init();

	/* Initialize USB Subsystem */
	USB_Init();

	/* Scheduling - routine never returns, so put this last in the main function */
	Scheduler_Start();
}

/** Event handler for the USB_Connect event. This indicates that the device is enumerating via the status LEDs. */
EVENT_HANDLER(USB_Connect)
{
	/* Start USB management task */
	Scheduler_SetTaskMode(USB_USBTask, TASK_RUN);

	/* Indicate USB enumerating */
	UpdateStatus(Status_USBEnumerating);
}

/** Event handler for the USB_Disconnect event. This indicates that the device is no longer connected to a host via
 *  the status LEDs, disables the sample update and PWM output timers and stops the USB and MIDI management tasks.
 */
EVENT_HANDLER(USB_Disconnect)
{
	/* Stop running audio and USB management tasks */
	Scheduler_SetTaskMode(USB_MIDI_Task, TASK_STOP);
	Scheduler_SetTaskMode(USB_USBTask, TASK_STOP);
	Scheduler_SetTaskMode(SHIFT_REG_Task, TASK_STOP);

	/* Indicate USB not ready */
	UpdateStatus(Status_USBNotReady);
}

/** Event handler for the USB_ConfigurationChanged event. This is fired when the host set the current configuration
 *  of the USB device after enumeration - the device endpoints are configured and the MIDI management task started.
 */
EVENT_HANDLER(USB_ConfigurationChanged)
{
	/* Setup MIDI stream endpoints */
	Endpoint_ConfigureEndpoint(MIDI_STREAM_OUT_EPNUM, EP_TYPE_BULK,
			ENDPOINT_DIR_OUT, MIDI_STREAM_EPSIZE,
			ENDPOINT_BANK_SINGLE);

	Endpoint_ConfigureEndpoint(MIDI_STREAM_IN_EPNUM, EP_TYPE_BULK,
			ENDPOINT_DIR_IN, MIDI_STREAM_EPSIZE,
			ENDPOINT_BANK_SINGLE);

	/* Indicate USB connected and ready */
	UpdateStatus(Status_USBReady);

	/* Start MIDI task */
	Scheduler_SetTaskMode(USB_MIDI_Task, TASK_RUN);
	Scheduler_SetTaskMode(SHIFT_REG_Task, TASK_RUN);
}

/** Task to handle the generation of MIDI note change events in response to presses of the board joystick, and send them
 *  to the host.
 */
TASK(USB_MIDI_Task)
{
	/* Select the MIDI IN stream */
	Endpoint_SelectEndpoint(MIDI_STREAM_IN_EPNUM);

	/* Check if endpoint is ready to be written to */
	if (Endpoint_IsINReady())
	{
		if (Tx_Buffer.Elements > 2){
			/* Wait until Serial Tx Endpoint Ready for Read/Write */
			while (!(Endpoint_IsINReady()));
			if(Endpoint_BytesInEndpoint() < MIDI_STREAM_EPSIZE){
				uint8_t chan = Buffer_GetElement(&Tx_Buffer);
				//the channel always has the top bit set.. 
				//this way we make sure to line up data if data is dropped
				//XXX could lead to lockup, need a better way
				while(!(chan & 0x80))
					chan = Buffer_GetElement(&Tx_Buffer);
				uint8_t addr = Buffer_GetElement(&Tx_Buffer);
				uint8_t val = Buffer_GetElement(&Tx_Buffer);
				SendMIDICC(addr & 0x7F, val & 0x7F, 0, chan & 0x0F);
			}
		}
	}

	/* Select the MIDI OUT stream */
	Endpoint_SelectEndpoint(MIDI_STREAM_OUT_EPNUM);

	/* Check if endpoint is ready to be read from, if so discard its (unused) data */
	if (Endpoint_IsOUTReceived())
		Endpoint_ClearOUT();
}

TASK(SHIFT_REG_Task)
{
	uint8_t i, j, board;
	latch_data();

	for(board = 0; board < NUMBOARDS; board++){
		//read in the encoder data
		for(i = 0; i < 2; i++){
			encoder_hist[i + board * 2][history] = 0;
			for(j = 0; j < 8; j++){
				encoder_hist[i + board * 2][history] |= ((PIND & _BV(PIND0)) << (7 - j));
				tick_clock();
			}
		}
		//read in the button data
		for(i = 0; i < 2; i++){
			button_hist[i + board * 2][history] = 0;
			for(j = 0; j < 8; j++){
				button_hist[i + board * 2][history] |= ((PIND & _BV(PIND0)) << (7 - j));
				tick_clock();
			}
		}

		//debounce buttons
		//encoders
		for(i = 0; i < 8; i++){
			bool consistent = true;
			uint8_t shift = (i * 2) % 8;
			uint8_t bank = i / 4;
			uint8_t state = (encoder_hist[bank + board * 2][0] >> shift) & 0x3;
			for(j = 1; j < HISTORY; j++){
				if(state != ((encoder_hist[bank + board * 2][0] >> shift) & 0x3)){
					consistent = false;
					break;
				}
			}
			if(consistent){
				uint8_t last_state = (encoder_last[bank + board * 2] >> shift) & 0x3;
				if(state != last_state){
					//send data only if:
					//we are not excluding non detent data
					//OR [it is implied we are only sending detent data], we are on a detent
					if(//(encoder_settings[i + board * 8].flags & ENC_DETENT_ONLY) ||
							((state == 0x0) || (state == 0x3))){

						Buffer_StoreElement(&Tx_Buffer, 0x80);
						switch(decode(state, last_state)){
							case 1:
								//addr
								Buffer_StoreElement(&Tx_Buffer, cc_num(ENC, i, board));
								//value
								Buffer_StoreElement(&Tx_Buffer, 65);
								break;
							case -1:
								//addr
								Buffer_StoreElement(&Tx_Buffer, cc_num(ENC, i, board));
								//value
								Buffer_StoreElement(&Tx_Buffer, 63);
								break;
							default:
								break;
						}

					}

					//store the new state data
					encoder_last[bank + board * 2] = (encoder_last[bank + board * 2] & ~(0x3 << shift)) | (state << shift);
				}
			}
		}

		//encoder buttons
		for(i = 0; i < 8; i++){
			bool consistent = true;
			uint8_t mask = (0x1 << i);

			//read through history
			bool up = (bool)(button_hist[board * 2][0] & mask);
			for(j = 1; j < HISTORY; j++){
				if(up != (bool)(button_hist[board * 2][j] & mask)){
					consistent = false;
					break;
				}
			}

			if(consistent){
				//has the state changed?
				if((button_last[board * 2] & mask) != (button_hist[board * 2][0] & mask)){

					//send the CC index
					Buffer_StoreElement(&Tx_Buffer, 0x80);
					Buffer_StoreElement(&Tx_Buffer, cc_num(ENC_BTN, i, board));

					//send the CC value
					if(up)
						Buffer_StoreElement(&Tx_Buffer, 0);
					else
						Buffer_StoreElement(&Tx_Buffer, 127);

					//store the new state
					button_last[board * 2] = (button_last[board * 2] & ~mask) | (button_hist[board * 2][0] & mask);
				}
			}
		}
		//other buttons
		for(i = 0; i < 4; i++){
			bool consistent = true;
			uint8_t mask = (0x1 << i);

			//read through history
			bool up = (bool)(button_hist[1 + board * 2][0] & mask);
			for(j = 1; j < HISTORY; j++){
				if(up != (bool)(button_hist[1 + board * 2][j] & mask)){
					consistent = false;
					break;
				}
			}
			if(consistent){
				//has the state changed?
				if((button_last[1 + board * 2] & mask) != (button_hist[1 + board * 2][0] & mask)){
					button_t setting;
					eeprom_busy_wait();
					//eeprom_read_block((void *)&setting, (const void *)((i + board * 4) * sizeof(button_t)), sizeof(button_t));
					eeprom_read_block((void *)&setting, (void *)(&button_settings[i + board * 4]), sizeof(button_t));

					//send the channel and CC index
					Buffer_StoreElement(&Tx_Buffer, 0x80 | setting.chan);
					Buffer_StoreElement(&Tx_Buffer, setting.num);

					//send the CC value
					if(up)
						Buffer_StoreElement(&Tx_Buffer, 0);
					else
						Buffer_StoreElement(&Tx_Buffer, 127);

					//store the new state
					button_last[1 + board * 2] = (button_last[1 + board * 2] & ~mask) | (button_hist[1 + board * 2][0] & mask);
				}
			}
		}
	}

	//increment the history index
	history = (history + 1) % HISTORY;
}

/** Function to manage status updates to the user. This is done via LEDs on the given board, if available, but may be changed to
 *  log to a serial port, or anything else that is suitable for status updates.
 *
 *  \param CurrentStatus  Current status of the system, from the MIDI_StatusCodes_t enum
 */
void UpdateStatus(uint8_t CurrentStatus)
{
	//by default turn off the LED
	PORTD |= _BV(PORTD6);

	switch (CurrentStatus)
	{
		case Status_USBNotReady:
			break;
		case Status_USBEnumerating:
			break;
		case Status_USBReady:
			//Turn on the LED when we are ready
			PORTD &= ~_BV(PORTD6);
			break;
	}
}

/** Sends a MIDI note change event (note on or off) to the MIDI output jack, on the given virtual cable ID and channel.
 *
 *  \param Pitch    Pitch of the note to turn on or off
 *  \param OnOff    Set to true if the note is on (being held down), or false otherwise
 *  \param CableID  ID of the virtual cable to send the note change to
 *  \param Channel  MIDI channel number to send the note change event to
 */
void SendMIDINoteChange(const uint8_t Pitch, const bool OnOff, const uint8_t CableID, const uint8_t Channel)
{
	/* Wait until endpoint ready for more data */
	while (!(Endpoint_IsReadWriteAllowed()));

	/* Check if the message should be a Note On or Note Off command */
	uint8_t Command = ((OnOff)? MIDI_COMMAND_NOTE_ON : MIDI_COMMAND_NOTE_OFF);

	/* Write the Packet Header to the endpoint */
	Endpoint_Write_Byte((CableID << 4) | (Command >> 4));

	/* Write the Note On/Off command with the specified channel, pitch and velocity */
	Endpoint_Write_Byte(Command | Channel);
	Endpoint_Write_Byte(Pitch);
	Endpoint_Write_Byte(MIDI_STANDARD_VELOCITY);

	/* Send the data in the endpoint to the host */
	Endpoint_ClearIN();
}

void SendMIDICC(const uint8_t num, const uint8_t val, const uint8_t CableID, const uint8_t Channel)
{
	/* Wait until endpoint ready for more data */
	while (!(Endpoint_IsReadWriteAllowed()));

	/* Check if the message should be a Note On or Note Off command */
	uint8_t Command = MIDI_COMMAND_CC;

	/* Write the Packet Header to the endpoint */
	Endpoint_Write_Byte((CableID << 4) | (Command >> 4));

	Endpoint_Write_Byte(Command | Channel);
	Endpoint_Write_Byte(num);
	Endpoint_Write_Byte(val);

	/* Send the data in the endpoint to the host */
	Endpoint_ClearIN();
}
