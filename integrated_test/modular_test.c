#include <p24FJ128GB206.h>
#include <stdint.h>
#include "config.h"
#include "common.h"
#include "ui.h"
#include "usb.h"
#include "pin.h"
#include "spi.h"

#include "timer.h"
#include "oc.h"
#include "motor.h"
#include "encoder.h"
#include "usb2.h"

//#define TOGGLE_LED1         1
//#define TOGGLE_LED2         2
//#define READ_SW1            3
//#define READ_SW2            9
//#define READ_SW3            10
//#define REG_MAG_ADDR        0x3FFE

#define ENC_READ_ANG        1
#define TOGGLE_LED         2

void read_enc(){
	WORD result = enc_getAngle(&enc);
	BD[EP0IN].address[0] = result.b[0];
	BD[EP0IN].address[1] = result.b[1];
	BD[EP0IN].bytecount = 2;         // set EP0 IN byte count to 1
	BD[EP0IN].status = 0xC8;    
}

//void toggle_led(){
//	WORD led_idx = USB_setup.wValue;
//
//}

void registerUSBEvents(){
	registerUSBEvent(read_enc, ENC_READ_ANG);
}

int16_t main(void) {
	init_clock();
	init_ui();
	init_pin();
	init_spi();

	init_timer();
	init_oc();

	init_motors();
	init_enc();

	registerUSBEvents();

	InitUSB();                              // initialize the USB registers and serial interface engine
	while (USB_USWSTAT!=CONFIG_STATE) {     // while the peripheral is not configured...
		ServiceUSB();
	}
	while (1) {
		ServiceUSB();                       // service any pending USB requests
		drive(&m1, !sw_read(&sw1));
		led_write(&led3, !sw_read(&sw1));
	}
}
