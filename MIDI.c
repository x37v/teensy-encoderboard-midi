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
#include <avr/power.h>

#define ADC_SMOOTHING_AMT 2
//mask off the mux selection
#define ADC_MUX_MASK 0xE0

/* Scheduler Task List */
TASK_LIST
{
	{ .Task = USB_USBTask          , .TaskStatus = TASK_STOP },
		{ .Task = USB_MIDI_Task        , .TaskStatus = TASK_STOP },
		{ .Task = SHIFT_REG_Task        , .TaskStatus = TASK_STOP },
		{ .Task = ADC_Task        , .TaskStatus = TASK_STOP },
};

//hold the data to send
RingBuff_t midiout_buf;
typedef enum {
	SEND_ENCODER,
	SEND_BUTTON,
	SEND_ALL
} command_t;
//hold commands [send sysex data dumps]
RingBuff_t cmd_buf;

volatile uint8_t encoder_hist[2 * NUMBOARDS][HISTORY];
volatile uint8_t button_hist[2 * NUMBOARDS][HISTORY];
volatile uint8_t history = 0;

volatile uint8_t encoder_last[2 * NUMBOARDS];
volatile uint8_t button_last[2 * NUMBOARDS];

//flags to indicate that a button is down
volatile uint8_t enc_btn_down[NUMBOARDS];
volatile uint8_t enc_value[8 * NUMBOARDS];

//this holds the channel and cc number for all the encoders, 
//so we can quickly match incoming values
volatile midi_cc_t enc_chan_num[8 * NUMBOARDS];

volatile bool send_ack;
volatile bool send_version;
volatile bool send_num_boards;
volatile bool sysex_in;
volatile uint8_t sysex_in_cnt;
volatile sysex_t sysex_in_type;
//this indexes an encoder or button based on the type set above
volatile uint8_t sysex_setting_index;

//header + command_code + index + data
//used for buttons as well
volatile uint8_t sysex_out_buffer[SYSEX_HEADER_SIZE + 7];

//eeprom stuff
midi_cc_t EEMEM button_settings[4 * NUMBOARDS];
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

uint8_t enc_index(uint8_t index, uint8_t board){
	if (index & 0x1)
		return ((NUMBOARDS + board) << 2) + (index >> 1);
	else
		return (index >> 1) + board * 4;
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

volatile uint8_t adc[4];
volatile uint8_t adc_index;

int main(void)
{
	uint8_t i;

	_delay_ms(100);
	/* Disable watchdog if enabled by bootloader/fuses */
	MCUSR &= ~(1 << WDRF);
	wdt_disable();

	/* Disable clock division */
	//doesn't work on 32u4
	//clock_prescale_set(clock_div_1);

	//init ringbuffers
	Buffer_Initialize(&midiout_buf);
	Buffer_Initialize(&cmd_buf);

	//init ports [d6 is the LED]
	DDRD |= _BV(PIND1) | _BV(PIND4) | _BV(PIND6);
	DDRD &= ~_BV(PIND0);

	send_num_boards = send_version = send_ack = false;
	sysex_in = false;
	sysex_in_cnt = 0;
	sysex_in_type = SYSEX_INVALID;
	sysex_setting_index = 0;

	//initially everything is 'up'
	for(i = 0; i < (2 * NUMBOARDS); i++)
		encoder_last[i] = button_last[i] = 0xFF;

	//initially all the encoder buttons are up
	for(i = 0; i < NUMBOARDS; i++)
		enc_btn_down[i] = 0;

	for(i = 0; i < 8 * NUMBOARDS; i++){
		//set the initial value for the encoder [absolute]
		enc_value[i] = 0;
		eeprom_busy_wait();
		enc_chan_num[i].chan = eeprom_read_byte((void *)&(encoder_settings[i].chan));
		eeprom_busy_wait();
		enc_chan_num[i].num = eeprom_read_byte((void *)&(encoder_settings[i].num));

		encoder_t setting;
		//see if stuff is already set, test for CHAN == 0xFF
		eeprom_busy_wait();
		if(eeprom_read_byte((void *)&(encoder_settings[i].chan)) == 0xFF)
			continue;

		//set the settings
		setting.flags = ENC_DETENT_ONLY;
		setting.chan = 0;
		setting.num = i;

		setting.btn.chan = 0;
		setting.btn.num = i + ENC_BTN_CC_OFFSET;
		
		//the multiplier
		setting.flags |= ENC_BUTTON_MUL;
		setting.btn.num = 8;

		//write to eeprom
		eeprom_busy_wait();
		//XXX the src + dst changes.. i'm using libc 1.6.2
		eeprom_write_block((void *)(&(encoder_settings[i])), (void *)&setting, sizeof(encoder_t));
	}

	for(i = 0; i < 4 * NUMBOARDS; i++){
		midi_cc_t setting;

		//see if stuff is already set, test for CHAN == 0xFF
		eeprom_busy_wait();
		if(eeprom_read_byte((void *)&(button_settings[i].chan)) == 0xFF)
			continue;

		setting.chan = 0;
		setting.num = i + BTN_CC_OFFSET;

		//write to eeprom
		eeprom_busy_wait();
		//XXX the src + dst changes.. i'm using libc 1.6.2
		eeprom_write_block((void *)(&(button_settings[i])), (void *)&setting, sizeof(midi_cc_t));
	}

	//init the sysex out for button and encoders for sending button and encoder data
	for(i = 0; i < SYSEX_HEADER_SIZE; i++)
		sysex_out_buffer[i] = sysex_header[i];


	/* Indicate USB not ready */
	UpdateStatus(Status_USBNotReady);

	//set up ADC
	DIDR0 = 0xFF;
	//left adjust, channel 0
	ADMUX = _BV(ADLAR) | _BV(REFS0);
	//enable 
	ADCSRA = _BV(ADEN) | _BV(ADPS1) | _BV(ADPS2);

	for(adc_index = 0; adc_index < 4; adc_index++){
		if (adc_index < 2)
			ADMUX = (ADMUX & ADC_MUX_MASK) | adc_index;
		else if (adc_index == 2)
			ADMUX = (ADMUX & ADC_MUX_MASK) | 0x4;
		else
			ADMUX = (ADMUX & ADC_MUX_MASK) | 0x5;
		//start a conversion
		ADCSRA |= _BV(ADSC);

		//wait for conversion to be done
		while(!(ADCSRA & _BV(ADIF)));

		//grab it
		adc[adc_index] = ADCH >> 1;
	}
	adc_index = 0;


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
	Scheduler_SetTaskMode(ADC_Task, TASK_STOP);

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
	Scheduler_SetTaskMode(ADC_Task, TASK_RUN);
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
		if(send_ack){
			send_ack = false;
			while (!(Endpoint_IsINReady()));
			SendSysex(sysex_ack, SYSEX_ACK_SIZE, 0);
		} else if (send_num_boards){
			send_num_boards = false;
			while (!(Endpoint_IsINReady()));
			SendSysex(sysex_boards, SYSEX_BOARD_CNT_SIZE, 0);
		} else if(send_version){
			send_version = false;
			while (!(Endpoint_IsINReady()));
			SendSysex(sysex_version, SYSEX_VERSION_SIZE, 0);
		}
		//deal with commands
		if(cmd_buf.Elements > 1){
			while (!(Endpoint_IsINReady()));
			if(Endpoint_BytesInEndpoint() < MIDI_STREAM_EPSIZE){
				uint8_t cmd = Buffer_GetElement(&cmd_buf);
				uint8_t index;
				//the cmd always has the top bit set
				if(cmd & 0x80){
					switch(cmd & 0x7F){
						case SEND_ENCODER:
							index = Buffer_GetElement(&cmd_buf);
							if(index < NUMBOARDS * 8){
								//fill and send the sysex out buffer
								sysex_out_buffer[SYSEX_HEADER_SIZE] = RET_ENCODER_DATA;
								sysex_out_buffer[SYSEX_HEADER_SIZE + 1] = index;
								eeprom_busy_wait();
								sysex_out_buffer[SYSEX_HEADER_SIZE + 2] = 
									eeprom_read_byte((void *)&(encoder_settings[index].flags));
								sysex_out_buffer[SYSEX_HEADER_SIZE + 3] = enc_chan_num[index].chan;
								sysex_out_buffer[SYSEX_HEADER_SIZE + 4] = enc_chan_num[index].num;
								eeprom_busy_wait();
								sysex_out_buffer[SYSEX_HEADER_SIZE + 5] = 
									eeprom_read_byte((void *)&(encoder_settings[index].btn.chan));
								eeprom_busy_wait();
								sysex_out_buffer[SYSEX_HEADER_SIZE + 6] = 
									eeprom_read_byte((void *)&(encoder_settings[index].btn.num));
								while (!(Endpoint_IsINReady()));
								SendSysex(sysex_out_buffer, SYSEX_HEADER_SIZE + 7, 0);
							}
							break;
						case SEND_BUTTON:
							index = Buffer_GetElement(&cmd_buf);
							if(index < NUMBOARDS * 4){
								//fill and send the sysex out buffer
								sysex_out_buffer[SYSEX_HEADER_SIZE] = RET_BUTTON_DATA;
								sysex_out_buffer[SYSEX_HEADER_SIZE + 1] = index;
								eeprom_busy_wait();
								sysex_out_buffer[SYSEX_HEADER_SIZE + 2] = 
									eeprom_read_byte((void *)&(button_settings[index].chan));
								eeprom_busy_wait();
								sysex_out_buffer[SYSEX_HEADER_SIZE + 3] = 
									eeprom_read_byte((void *)&(button_settings[index].num));
								while (!(Endpoint_IsINReady()));
								SendSysex(sysex_out_buffer, SYSEX_HEADER_SIZE + 4, 0);
							}
							break;
						default:
							break;
					};
				}
			}
		}
		if (midiout_buf.Elements > 2){
			/* Wait until Serial Tx Endpoint Ready for Read/Write */
			while (!(Endpoint_IsINReady()));
			if(Endpoint_BytesInEndpoint() < MIDI_STREAM_EPSIZE){
				uint8_t chan = Buffer_GetElement(&midiout_buf);
				//the channel always has the top bit set.. 
				//this way we make sure to line up data if data is dropped
				//so if we don't have the top bit set, don't grab more data..
				//we can catch up next time
				if(chan & 0x80){
					uint8_t addr = Buffer_GetElement(&midiout_buf);
					uint8_t val = Buffer_GetElement(&midiout_buf);
					SendMIDICC(addr & 0x7F, val & 0x7F, 0, chan & 0x0F);
				}
			}
		}
	}

	/* Select the MIDI OUT stream */
	Endpoint_SelectEndpoint(MIDI_STREAM_OUT_EPNUM);

	if (Endpoint_IsOUTReceived()){
		while (Endpoint_BytesInEndpoint()){
			uint8_t i;
			//always comes in packets of 4 bytes
			Endpoint_Read_Byte();
			//ditch the first byte and grab the next 3
			for(i = 0; i < 3; i++){
				uint8_t byte = Endpoint_Read_Byte();
				if(byte == SYSEX_BEGIN) {
					sysex_in = true;
					sysex_in_cnt = 0;
				} else if(byte == SYSEX_END){
					//if we were in sysex mode and we just got a header [just a ping]
					//send an ack
					if(sysex_in && sysex_in_cnt == SYSEX_HEADER_SIZE)
						send_ack = true;
					sysex_in = false;
				} else if(byte & 0x80){
					//if the top bit is set then we leave sysex mode
					sysex_in = false;
				} else if(sysex_in){
					//match the header
					if(sysex_in_cnt < SYSEX_HEADER_SIZE){
						if(!sysex_header[sysex_in_cnt] == byte)
							sysex_in = false;
					} else {
						//here we have matched the header and we're parsing input data
						uint8_t index = sysex_in_cnt - SYSEX_HEADER_SIZE;
						//if the index is 0 then we're matching the type
						if(index == 0){
							if(byte == GET_VERSION){
								send_version = true;
								sysex_in = false;
							} else if (byte == GET_NUM_BOARDS){
								send_num_boards = true;
								sysex_in = false;
								//XXX should deal with a dump all here
							} else if (byte < SYSEX_INVALID){
								sysex_in_type = byte;
							} else {
								sysex_in_type = SYSEX_INVALID;
								sysex_in = false;
							}
						} else if(index == 1){
							if (sysex_in_type == SET_ENCODER_DATA || sysex_in_type == SET_BUTTON_DATA)
								sysex_setting_index = byte;
							else if(sysex_in_type == GET_ENCODER_DATA){
								if(byte < (8 * NUMBOARDS)){
									Buffer_StoreElement(&cmd_buf, 0x80 | SEND_ENCODER);
									Buffer_StoreElement(&cmd_buf, byte);
								}
								sysex_in = false;
								sysex_in_type = SYSEX_INVALID;
							} else if(sysex_in_type == GET_BUTTON_DATA){
								if(byte < (4 * NUMBOARDS)){
									Buffer_StoreElement(&cmd_buf, 0x80 | SEND_BUTTON);
									Buffer_StoreElement(&cmd_buf, byte);
								}
								sysex_in = false;
								sysex_in_type = SYSEX_INVALID;
							}
						} else if(index > 1) { 
							if(sysex_in_type == SET_ENCODER_DATA){
								//make sure we're in range
								if(sysex_setting_index < (8 * NUMBOARDS)){
									switch(index - 2){
										case 0:
											//channel
											eeprom_busy_wait();
											eeprom_write_byte((void *)(&(encoder_settings[sysex_setting_index].flags)), byte & ENC_FLAGS);
											break;
										case 1:
											//chan
											eeprom_busy_wait();
											enc_chan_num[sysex_setting_index].chan = byte & 0x0F;
											eeprom_write_byte((void *)(&(encoder_settings[sysex_setting_index].chan)), byte & 0x0F);
											break;
										case 2:
											//num
											eeprom_busy_wait();
											enc_chan_num[sysex_setting_index].num = byte & 0x7F;
											eeprom_write_byte((void *)(&(encoder_settings[sysex_setting_index].num)), byte & 0x7F);
											break;
										case 3:
											//button chan
											eeprom_busy_wait();
											eeprom_write_byte((void *)(&(encoder_settings[sysex_setting_index].btn.chan)), byte & 0x0F);
											break;
										case 4:
											//button num
											eeprom_busy_wait();
											eeprom_write_byte((void *)(&(encoder_settings[sysex_setting_index].btn.num)), byte & 0x7F);
											send_ack = true;
											break;
										default:
											sysex_in = false;
											sysex_in_type = SYSEX_INVALID;
											break;
									}
								} else {
									sysex_in = false;
									sysex_in_type = SYSEX_INVALID;
								}
							} else if(sysex_in_type == SET_BUTTON_DATA){
								//make sure we're in range
								if(sysex_setting_index < (4 * NUMBOARDS)){
									switch(index - 2){
										case 0:
											//channel
											eeprom_busy_wait();
											eeprom_write_byte((void *)(&(button_settings[sysex_setting_index].chan)), byte & 0x0f);
											break;
										case 1:
											//num
											eeprom_busy_wait();
											eeprom_write_byte((void *)(&(button_settings[sysex_setting_index].num)), byte & 0x7f);
											send_ack = true;
											break;
										default:
											//if there is more data, ditch it
											sysex_in = false;
											sysex_in_type = SYSEX_INVALID;
											break;
									} 
								} else {
									sysex_in = false;
									sysex_in_type = SYSEX_INVALID;
								}
							} else {
								sysex_in = false;
								sysex_in_type = SYSEX_INVALID;
							}
						}
					}
					sysex_in_cnt++;
				}
			}
		}
		// Clear the endpoint buffer
		Endpoint_ClearOUT();
	}
}

TASK(ADC_Task)
{
	//if an conversion is complete
	if(ADCSRA & _BV(ADIF)){
		uint8_t index = adc_index;
		adc_index = (adc_index + 1) % 4;
		//we are using 0,1,3,4 inputs
		if (adc_index < 2)
			ADMUX = (ADMUX & ADC_MUX_MASK) | adc_index;
		else if (adc_index == 2)
			ADMUX = (ADMUX & ADC_MUX_MASK) | 0x4;
		else
			ADMUX = (ADMUX & ADC_MUX_MASK) | 0x5;
		//read in and smooth
		uint16_t new_adc = ADCH + ADC_SMOOTHING_AMT * (uint16_t)(adc[index] << 1);
		new_adc /= (ADC_SMOOTHING_AMT + 1);
		//if the LS bit is set and we'd have 0x7D if we shifted, make it 0x7F
		//otherwise just shift the result and mask.. if we don't do this we never
		//make it to 0x7F, but we also never seem to get 0x7E
		if(new_adc == 0xFB)
			new_adc = 0x7F;
		else
			new_adc = (new_adc >> 1) & 0x7F;
		if(new_adc != adc[index]){
			//XXX make direction, index and channel configurable
			Buffer_StoreElement(&midiout_buf, 0x80 | 10);
			Buffer_StoreElement(&midiout_buf, index + 64);
			Buffer_StoreElement(&midiout_buf, 127 - new_adc);
			adc[index] = new_adc;
		}
		//enable and start another conversion
		ADCSRA |= _BV(ADSC);
	}
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
				//we use a weird indexing scheme because we are cascading boards..
				//calculate the index
				uint8_t index = enc_index(i, board);
				eeprom_busy_wait();
				uint8_t flags = eeprom_read_byte((void *)&(encoder_settings[index].flags));
				uint8_t last_state = (encoder_last[bank + board * 2] >> shift) & 0x3;
				if(state != last_state){
					//send data only if:
					//we are not excluding non detent data
					//OR [it is implied we are only sending detent data], we are on a detent
					if(!(flags & ENC_DETENT_ONLY) ||
							((state == 0x0) || (state == 0x3))){
						int8_t enc_offset = decode(state, last_state);
						if(enc_offset != 0){
							uint8_t chan = enc_chan_num[index].chan;
							uint8_t num = enc_chan_num[index].num;

							//if the button is doing multiplying, do that
							if(flags & ENC_BUTTON_MUL){
								uint8_t mul;
								//get the multiplier
								eeprom_busy_wait();
								mul = eeprom_read_byte((void *)&(encoder_settings[index].btn.num));
								if(enc_btn_down[board] & (1 << i))
									enc_offset *= mul;
							}

							//are we doing absolute values?
							if(flags & ENC_ABSOLUTE){
								uint8_t out_value;
								uint8_t last_value = enc_value[index];
								//make sure to stay in range
								if(enc_offset + (int8_t)last_value < 0)
									out_value = 0;
								else if (last_value + enc_offset > 127)
									out_value = 127;
								else
									out_value = last_value + enc_offset;

								//only send/store if we have a new value
								if(out_value != last_value){
									Buffer_StoreElement(&midiout_buf, 0x80 | chan);
									Buffer_StoreElement(&midiout_buf, num);
									Buffer_StoreElement(&midiout_buf, out_value);
									//store this value
									enc_value[index] = out_value;
								}

							} else {
								Buffer_StoreElement(&midiout_buf, 0x80 | chan);
								Buffer_StoreElement(&midiout_buf, num);
								Buffer_StoreElement(&midiout_buf, 64 + enc_offset);
							}
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
				uint8_t index = enc_index(i, board);
				eeprom_busy_wait();
				uint8_t flags = eeprom_read_byte((void *)&(encoder_settings[index].flags));
				//has the state changed?
				if((button_last[board * 2] & mask) != (button_hist[board * 2][0] & mask)){

					//set the down state
					if(up)
						enc_btn_down[board] &= ~(1 << i);
					else
						enc_btn_down[board] |= (1 << i);

					//if we're doing a mul then we just set the down state
					if(!(flags & ENC_BUTTON_MUL)){

						//get the cc chan and num
						eeprom_busy_wait();
						uint8_t chan = eeprom_read_byte((void *)&(encoder_settings[index].btn.chan));
						eeprom_busy_wait();
						uint8_t num = eeprom_read_byte((void *)&(encoder_settings[index].btn.num));

						//send the CC chan
						Buffer_StoreElement(&midiout_buf, 0x80 | (chan & 0x0F));
						//send the CC index
						Buffer_StoreElement(&midiout_buf, num);

						//send the CC value
						if(up)
							Buffer_StoreElement(&midiout_buf, 0);
						else
							Buffer_StoreElement(&midiout_buf, 127);
					}

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
					midi_cc_t setting;
					eeprom_busy_wait();
					//eeprom_read_block((void *)&setting, (const void *)((i + board * 4) * sizeof(midi_cc_t)), sizeof(midi_cc_t));
					eeprom_read_block((void *)&setting, (void *)(&button_settings[i + board * 4]), sizeof(midi_cc_t));

					//send the channel and CC index
					Buffer_StoreElement(&midiout_buf, 0x80 | setting.chan);
					Buffer_StoreElement(&midiout_buf, setting.num);

					//send the CC value
					if(up)
						Buffer_StoreElement(&midiout_buf, 0);
					else
						Buffer_StoreElement(&midiout_buf, 127);

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

void SendSysex(const uint8_t * buf, const uint8_t len, const uint8_t CableID)
{
	if(len == 0)
		return;
	else if(len == 1){
		/* Wait until endpoint ready for more data */
		while (!(Endpoint_IsReadWriteAllowed()));
		/* Write the Packet Header to the endpoint */
		Endpoint_Write_Byte((CableID << 4) | 0x7);
		Endpoint_Write_Byte(SYSEX_BEGIN);
		Endpoint_Write_Byte(buf[0]);
		Endpoint_Write_Byte(SYSEX_END);
		/* Send the data in the endpoint to the host */
		Endpoint_ClearIN();
	} else {
		uint8_t i;
		//write the first packet
		
		// Wait until endpoint ready for more data
		while (!(Endpoint_IsReadWriteAllowed()));
		// Write the Packet Header to the endpoint
		Endpoint_Write_Byte((CableID << 4) | 0x4);
		Endpoint_Write_Byte(SYSEX_BEGIN);
		Endpoint_Write_Byte(buf[0]);
		Endpoint_Write_Byte(buf[1]);
		// Send the data in the endpoint to the host
		Endpoint_ClearIN();

		//write intermediate bytes
		for(i = 2; (i + 2) < len; i += 3){
			// Wait until endpoint ready for more data
			while (!(Endpoint_IsReadWriteAllowed()));
			// Write the Packet Header to the endpoint
			Endpoint_Write_Byte((CableID << 4) | 0x4);
			Endpoint_Write_Byte(buf[i]);
			Endpoint_Write_Byte(buf[i + 1]);
			Endpoint_Write_Byte(buf[i + 2]);
			// Send the data in the endpoint to the host
			Endpoint_ClearIN();
		}

		// Wait until endpoint ready for more data
		while (!(Endpoint_IsReadWriteAllowed()));
		// Write the Packet Header to the endpoint
		switch((len - 2) % 3){
			case 0:
				Endpoint_Write_Byte((CableID << 4) | 0x5);
				Endpoint_Write_Byte(SYSEX_END);
				Endpoint_Write_Byte(0);
				Endpoint_Write_Byte(0);
				break;
			case 1:
				Endpoint_Write_Byte((CableID << 4) | 0x6);
				Endpoint_Write_Byte(buf[len - 1]);
				Endpoint_Write_Byte(SYSEX_END);
				Endpoint_Write_Byte(0);
				break;
			case 2:
				Endpoint_Write_Byte((CableID << 4) | 0x7);
				Endpoint_Write_Byte(buf[len - 2]);
				Endpoint_Write_Byte(buf[len - 1]);
				Endpoint_Write_Byte(SYSEX_END);
				break;
		}
		// Send the data in the endpoint to the host
		Endpoint_ClearIN();
	}
}
