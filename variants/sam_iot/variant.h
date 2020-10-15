/*
  Copyright (c) 2014-2015 Arduino LLC.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#pragma once

// The definitions here needs a SAMD core >=1.6.10
#define ARDUINO_SAMD_VARIANT_COMPLIANCE 10610


#include <WVariant.h>

// General definitions
// -------------------

// Frequency of the board main oscillator
#define VARIANT_MAINOSC (32768ul)

// Master clock frequency
#define VARIANT_MCK     (48000000ul)

/*----------------------------------------------------------------------------
 *        Pins
 *----------------------------------------------------------------------------*/

// Number of pins defined in PinDescription array
#ifdef __cplusplus
extern "C" unsigned int PINCOUNT_fn();
#endif
#define PINS_COUNT           (PINCOUNT_fn())
#define NUM_DIGITAL_PINS     (18u)
#define NUM_ANALOG_INPUTS    (2u)
#define NUM_ANALOG_OUTPUTS   (1u)
#define analogInputToDigitalPin(p)  ((p < 7u) ? (p) + 15u : -1)

// Low-level pin register query macros
// -----------------------------------
#define digitalPinToPort(P)      (&(PORT->Group[g_APinDescription[P].ulPort]))
#define digitalPinToBitMask(P)   (1 << g_APinDescription[P].ulPin)
//#define analogInPinToBit(P)    ()
#define portOutputRegister(port) (&(port->OUT.reg))
#define portInputRegister(port)  (&(port->IN.reg))
#define portModeRegister(port)   (&(port->DIR.reg))
#define digitalPinHasPWM(P)      (g_APinDescription[P].ulPWMChannel != NOT_ON_PWM || g_APinDescription[P].ulTCChannel != NOT_ON_TIMER)

/*
 * digitalPinToTimer(..) is AVR-specific and is not defined for SAMD
 * architecture. If you need to check if a pin supports PWM you must
 * use digitalPinHasPWM(..).
 *
 * https://github.com/arduino/Arduino/issues/1833
 */
// #define digitalPinToTimer(P)

// LEDs
// ----
//#define PIN_LED     (6u)
//#define LED_BUILTIN PIN_LED

/*
 * Analog pins
 */
#define PIN_A0   (0u)
#define PIN_A1   (1u)
#define PIN_DAC0 (16u)

static const uint8_t A0  = PIN_A0;
static const uint8_t A1  = PIN_A1;
static const uint8_t DAC0 = PIN_DAC0;
#define ADC_RESOLUTION		12

// SPI Interfaces
// --------------
#define SPI_INTERFACES_COUNT 2

// SPI
#define PIN_SPI_MOSI  (4u)
#define PIN_SPI_SCK   (5u)
#define PIN_SPI_SS    (6u)
#define PIN_SPI_MISO  (7u)
#define PERIPH_SPI    sercom0
#define PAD_SPI_TX    SPI_PAD_0_SCK_1
#define PAD_SPI_RX    SERCOM_RX_PAD_3
static const uint8_t SS   = PIN_SPI_SS;   // SPI Slave SS not used. Set here only for reference.
static const uint8_t MOSI = PIN_SPI_MOSI;
static const uint8_t MISO = PIN_SPI_MISO;
static const uint8_t SCK  = PIN_SPI_SCK;

// SPI1: Connected to WINC1501B
#define PIN_SPI1_MOSI (8u)
#define PIN_SPI1_SCK  (9u)
#define PIN_SPI1_SS   (10u)
#define PIN_SPI1_MISO (11u)
#define PERIPH_SPI1   sercom4
#define PAD_SPI1_TX   SPI_PAD_0_SCK_1
#define PAD_SPI1_RX   SERCOM_RX_PAD_3
static const uint8_t SS1   = PIN_SPI1_SS;
static const uint8_t MOSI1 = PIN_SPI1_MOSI;
static const uint8_t MISO1 = PIN_SPI1_MISO;
static const uint8_t SCK1  = PIN_SPI1_SCK;

// Wire Interfaces
// ---------------
#define WIRE_INTERFACES_COUNT 1

// Wire
#define PIN_WIRE_SDA        (2u)
#define PIN_WIRE_SCL        (3u)
#define PERIPH_WIRE         sercom3
#define WIRE_IT_HANDLER     SERCOM3_Handler

static const uint8_t SDA = PIN_WIRE_SDA;
static const uint8_t SCL = PIN_WIRE_SCL;

// USB
// ---
#define PIN_USB_DM          (22ul)
#define PIN_USB_DP          (23ul)
#define PIN_USB_HOST_ENABLE (24ul)

// I2S Interfaces
// --------------
//#define I2S_INTERFACES_COUNT 1

//#define I2S_DEVICE          0
//#define I2S_CLOCK_GENERATOR 3
//#define PIN_I2S_SD          (PIN_A6)
//#define PIN_I2S_SCK         (2u)
//#define PIN_I2S_FS          (3u)

// Needed for WINC1501B (WiFi101) library
// --------------------------------------
#define WINC1501_RESET_PIN   (12u)
#define WINC1501_CHIP_EN_PIN (13u)
#define WINC1501_INTN_PIN    (15u)
#define WINC1501_SPI         SPI1
#define WINC1501_SPI_CS_PIN  PIN_SPI1_SS


// Serial ports
// ------------
#ifdef __cplusplus
#include "SERCOM.h"
#include "Uart.h"

/*	=========================
 *	===== SERCOM DEFINITION
 *	=========================
*/
extern SERCOM sercom0;
extern SERCOM sercom1;
extern SERCOM sercom2;
extern SERCOM sercom3;
extern SERCOM sercom4;
extern SERCOM sercom5;

// Serial
extern Uart Serial;
#define PIN_SERIAL_RX (16ul)
#define PIN_SERIAL_TX (17ul)
#define PAD_SERIAL_TX (UART_TX_PAD_0)
#define PAD_SERIAL_RX (SERCOM_RX_PAD_1)
#endif // __cplusplus

#ifdef __cplusplus
extern "C" {
#endif
unsigned int PINCOUNT_fn();
#ifdef __cplusplus
}
#endif

// These serial port names are intended to allow libraries and architecture-neutral
// sketches to automatically default to the correct port name for a particular type
// of use.  For example, a GPS module would normally connect to SERIAL_PORT_HARDWARE_OPEN,
// the first hardware serial port whose RX/TX pins are not dedicated to another use.
//
// SERIAL_PORT_MONITOR        Port which normally prints to the Arduino Serial Monitor
//
// SERIAL_PORT_USBVIRTUAL     Port which is USB virtual serial
//
// SERIAL_PORT_LINUXBRIDGE    Port which connects to a Linux system via Bridge library
//
// SERIAL_PORT_HARDWARE       Hardware serial port, physical RX & TX pins.
//
// SERIAL_PORT_HARDWARE_OPEN  Hardware serial ports which are open for use.  Their RX & TX
//                            pins are NOT connected to anything by default.
#define SERIAL_PORT_USBVIRTUAL      Serial
#define SERIAL_PORT_MONITOR         Serial
#define SERIAL_PORT_HARDWARE        Serial
#define SERIAL_PORT_HARDWARE_OPEN   Serial

