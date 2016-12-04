#include "ManchesterRF.h"	// https://github.com/cano64/ManchesterRF
#include "EEPROM.h"
#include "EEPROMAnything.h"
#include <TimerOne.h>

//#define DEBUG

// Configuration format
// --------------------
//
// CHANNEL-LOW-HIGH-SPEED
// 8       8   8    8

#define CONFIG_ADDRESS	0x00	// EEPROM address to store configuration
#define CONFIG_SIZE		0x0F 	// configuration array size in bytes
#define CONFIG_BACKUPS 	3 		// number of configuration copies to store

typedef struct
{
	uint8_t b[CONFIG_SIZE];
	byte crc;
} rx_config_t;

rx_config_t rx_config;

#define RX_PIN 11				// TX connects to this pin
#define DIM_PIN 9 				// dimmer output pin
#define LED_PIN 13				// LED connects to this pin
#define	PROG_ENABLE_PIN 4 		// if this pin is LOW, enable TX programming mode. Pins 5-9 are used to set channel
#define PROG_PIN_COUNT 5 		// number of programming pins

// Dimmer 3-byte message format
// ----------------------------
//
// HEADER-MSG_TYPE-CHANNEL-DATA-TRAILER
// 4      3        5       8    4

// 4 bits at the beginning and at the end of message
#define DATA_START 		0xF0	// packet start
#define DATA_STOP 		0x0A	// packet end

// 3 bits for different message types
#define MSG_BRIGHTNESS	0x00	// brightness adjustment
#define MSG_SET_LOW		0x01	// set low dimmable brightness value (multiplied by 10 at RX)
#define MSG_SET_HIGH	0x02	// set high dimmable brightness value (multiplied by 10 at RX)
#define MSG_SET_SPEED	0x03	// set dimming speed, 0-255, 0 = instant switching

#define BRIGHTNESS_HIGH	0xAA	// value to be sent for high brightness
#define BRIGHTNESS_LOW	0x3C	// value to be sent for low brightness

uint8_t channel = 0;			// channel to transmit information, 5 bits are used (31 channels supported, channel value 0 is used to detect EEPROM read failures)
uint8_t dim_low = 0;			// low dimmable brightness value (multiplied by 10 at RX)
uint8_t dim_high = 100;			// high dimmable brightness value (multiplied by 10 at RX)
uint8_t dim_speed = 10;			// dimming speed, 0-255, 0 = instant switching
uint8_t last_brightness = BRIGHTNESS_LOW;
uint8_t current_brightness = BRIGHTNESS_LOW;
boolean timer_initialized = false;

ManchesterRF rf(MAN_2400);		// link speed

//CRC-8 - based on the CRC8 formulas by Dallas/Maxim
//code released under the therms of the GNU GPL 3.0 license
byte crc8(const byte *data, byte len)
{
	byte crc = 0x00;
	while (len--)
	{
		byte extract = *data++;
		for (byte tempI = 8; tempI; tempI--)
		{
			byte sum = (crc ^ extract) & 0x01;
			crc >>= 1;
			if (sum)
			{
				crc ^= 0x8C;
			}
			extract >>= 1;
		}
	}
	return crc;
}

void zero_unused_config()
{
	for (uint8_t i = 5; i < CONFIG_SIZE; i++)
	{
		rx_config.b[i] = 0;
	}
}

void write_config(boolean read_channel)
{
	uint8_t j = 0;
	if (read_channel)
	{
		for (uint8_t i = (PROG_PIN_COUNT + PROG_ENABLE_PIN); i > PROG_ENABLE_PIN; i--)
		{
			pinMode(i, INPUT_PULLUP);
			bitWrite(channel, j, ((digitalRead(i) == HIGH) ? 0 : 1));
#ifdef DEBUG
			Serial.print("write_config() bit=");
			Serial.print(j);
			Serial.print(" value=");
			Serial.println(((digitalRead(i) == HIGH) ? 0 : 1));
#endif
			j++;
		}
	}
	rx_config.b[0] = channel;
	rx_config.b[1] = dim_low;
	rx_config.b[2] = dim_high;
	rx_config.b[3] = dim_speed;
	rx_config.b[4] = last_brightness;
	zero_unused_config();
	rx_config.crc = crc8(rx_config.b, CONFIG_SIZE);
#ifdef DEBUG
	Serial.print("write_config()");
	for (uint8_t i = 0; i < 5; i++)
	{
		Serial.print(" rx_config.b[");
		Serial.print(i);
		Serial.print("]=");
		Serial.print(rx_config.b[i]);
	}
	Serial.print(" rx_config.crc=");
	Serial.println(rx_config.crc);
#endif
	for (uint8_t i = 0; i < CONFIG_BACKUPS; i++)
	{
		EEPROM_writeAnything(CONFIG_ADDRESS + (i * (CONFIG_SIZE + 1)), rx_config);
	}
	//channel = 0;
}

void read_config()
{
	for (uint8_t i = 0; i < CONFIG_BACKUPS; i++)
	{
		EEPROM_readAnything(CONFIG_ADDRESS + (i * (CONFIG_SIZE + 1)), rx_config);
#ifdef DEBUG
		Serial.print("read_config() attempt ");
		Serial.print(i);
		for (uint8_t i = 0; i < 5; i++)
		{
			Serial.print(" rx_config.b[");
			Serial.print(i);
			Serial.print("]=");
			Serial.print(rx_config.b[i]);
		}
		Serial.print(" rx_config.crc=");
		Serial.print(rx_config.crc);
		Serial.print(" crc8()=");
		Serial.println(crc8(rx_config.b, CONFIG_SIZE));
#endif
		if (crc8(rx_config.b, CONFIG_SIZE) == rx_config.crc)
		{
			channel = rx_config.b[0];
			dim_low = rx_config.b[1];
			dim_high = rx_config.b[2];
			dim_speed = rx_config.b[3];
			last_brightness = rx_config.b[4];
#ifndef DEBUG
			break;
#endif
		}
	}

	if (channel == 0)	// CONFIG_BACKUPS attempts to read EEPROM were ALL unsuccessful => EEPROM corrupted or not programmed
	{
#ifdef DEBUG
		Serial.println("read_config() failure");
#endif
		while (true)
		{
			digitalWrite(LED_PIN, LOW);
			delay(100);
			digitalWrite(LED_PIN, HIGH);
			delay(100);
		}
	}
}

void dim_to(uint8_t value)
{
	uint16_t pwm_freq;

	if (!timer_initialized)
	{
		if (dim_speed > 0)
		{
			Timer1.initialize(10000); // set a timer of length 100000 microseconds (or 0.1 sec - or 10Hz => the led will blink 5 times, 5 cycles of on-and-off, per second)
			timer_initialized = true;
		}
	}

	if (dim_speed > 0)
	{
		if (value > last_brightness)		// dimming up
		{
			for (uint8_t i = dim_low; i <= dim_high; i++)
			{
				pwm_freq = i * 10;
				Timer1.pwm(DIM_PIN, pwm_freq);
				delay(dim_speed);
#ifdef DEBUG
				Serial.print("dim_to() pwm_freq=");
				Serial.println(pwm_freq);
#endif
			}
		}
		else if (value < last_brightness)	// dimming down
		{
			for (uint8_t i = dim_high; i > dim_low; i--)
			{
				pwm_freq = i * 10;
				Timer1.pwm(DIM_PIN, pwm_freq);
				delay(dim_speed);
#ifdef DEBUG
				Serial.print("dim_to() pwm_freq=");
				Serial.println(pwm_freq);
#endif
			}
		}
	}

	switch (value)
	{
	case BRIGHTNESS_LOW:
	{
#ifdef DEBUG
		Serial.println("dim_to() setting pin LOW");
#endif
		digitalWrite(DIM_PIN, LOW);
		break;
	}
	case BRIGHTNESS_HIGH:
	{
#ifdef DEBUG
		Serial.println("dim_to() setting pin HIGH");
#endif
		digitalWrite(DIM_PIN, HIGH);
		break;
	}
	default:
	{
		digitalWrite(DIM_PIN, LOW);
#ifdef DEBUG
		Serial.print("dim_to() invalid value=");
		Serial.println(value);
#endif
		break;
	}
	}
	Timer1.disablePwm(DIM_PIN);
	last_brightness = value;
}

void setup()
{
#ifdef DEBUG
	Serial.begin(57600);
#endif
	pinMode(LED_PIN, OUTPUT);
	pinMode(PROG_ENABLE_PIN, INPUT_PULLUP);
	if (digitalRead(PROG_ENABLE_PIN) == LOW)
	{
		write_config(true);
	}
	rf.RXInit(RX_PIN);
	read_config();
	pinMode(DIM_PIN, OUTPUT);
#ifdef DEBUG
	Serial.print("setup() last_brightness=");
	Serial.println(last_brightness);
#endif
	switch (last_brightness)
	{
	case BRIGHTNESS_LOW:
	{
		digitalWrite(DIM_PIN, LOW);
		break;
	}
	case BRIGHTNESS_HIGH:
	{
		digitalWrite(DIM_PIN, HIGH);
		break;
	}
	default:
	{
		digitalWrite(DIM_PIN, LOW);
#ifdef DEBUG
		Serial.print("setup() invalid last_brightness=");
		Serial.println(last_brightness);
#endif
		break;
	}
	}
}

void process_rx()
{
	uint8_t b1, b2, b3;				// ManchesterRF 3-byte receiveByte() bytes;
	uint8_t value;					// received value
	if (rf.receiveByte(b1, b2, b3))
	{
		if ((b1 & DATA_START) && (b3 & DATA_STOP))	// valid message format
		{
			digitalWrite(LED_PIN, HIGH);
			if ((((b1 << 4) & 0x1F) | (b2 >> 4)) == channel)	// message channel matches our channel
			{
				value = ((b2 << 4) | (b3 >> 4));
				switch ((b1 & 0x0E) >> 1)								// message type
				{
				case MSG_BRIGHTNESS:
				{
					if (value != last_brightness)
					{
						dim_to(value);
						write_config(false);
					}
#ifdef DEBUG
					else
					{
						Serial.print("process_rx() requested brightness=");
						Serial.print(value);
						Serial.print(" already is ");
						Serial.println(last_brightness);
					}
#endif
					break;
				}
				case MSG_SET_LOW:
				{
					dim_low = value;
					write_config(false);
					break;
				}
				case MSG_SET_HIGH:
				{
					dim_high = value;
					write_config(false);
					break;
				}
				case MSG_SET_SPEED:
				{
					dim_speed = value;
					write_config(false);
					break;
				}
#ifdef DEBUG
				default:
				{
					Serial.print("process_rx() invalid message type=");
					Serial.println((b1 & 0x0E) >> 1);
					break;
				}
#endif
				}
			}
#ifdef DEBUG
			else
			{
				Serial.print("process_rx() channel mismatch, received=");
				Serial.print(((b1 << 4) & 0x1F) | (b2 >> 4));
				Serial.print(" configured=");
				Serial.println(channel);
			}
#endif
			digitalWrite(LED_PIN, LOW);
		}
#ifdef DEBUG
		else
		{
			Serial.print("process_rx() invalid message format, b1=");
			Serial.print(b1, BIN);
			Serial.print(" b2=");
			Serial.print(b2, BIN);
			Serial.print(" b3=");
			Serial.println(b3);
		}
#endif
	}

}

void loop()
{
	if (rf.available())   //something is in RX buffer
	{
		process_rx();
	}
}


