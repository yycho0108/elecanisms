#include <p24FJ128GB206.h>
#include "config.h"
#include "common.h"
#include "usb.h"
#include "pin.h"
#include "uart.h"
#include <stdio.h>

#include "ui.h"
#include "oc.h"

#define HELLO       0   // Vendor request that prints "Hello World!"
#define SET_VALS    1   // Vendor request that receives 2 unsigned integer values
#define GET_VALS    2   // Vendor request that returns 2 unsigned integer values
#define PRINT_VALS  3   // Vendor request that prints 2 unsigned integer values 

#define TOGGLE_LED 4
#define SET_DUTY 5
#define GET_DUTY 6

uint16_t val1, val2;

//void ClassRequests(void) {
//    switch (USB_setup.bRequest) {
//        default:
//            USB_error_flags |= 0x01;                    // set Request Error Flag
//    }
//}

typedef enum{FALSE,TRUE} boolean;

typedef struct __LED{
	_LED* led;
	boolean status; // ON/OFF
	float duty;
} LED;

LED led_1 = {&led1, FALSE, 0.0};


void VendorRequests(void) {
    WORD temp;

    switch (USB_setup.bRequest) {
        case HELLO:
            printf("Hello World!\n");
            BD[EP0IN].bytecount = 0;    // set EP0 IN byte count to 0 
            BD[EP0IN].status = 0xC8;    // send packet as DATA1, set UOWN bit
            break;
        case SET_VALS: // master is setting vals, I'm "getting" vlas
            val1 = USB_setup.wValue.w;
            val2 = USB_setup.wIndex.w;
            BD[EP0IN].bytecount = 0;    // set EP0 IN byte count to 0 
            BD[EP0IN].status = 0xC8;    // send packet as DATA1, set UOWN bit
            break;
        case GET_VALS:// master is getting vals, I'm "setting" vals
            temp.w = val1;
            BD[EP0IN].address[0] = temp.b[0];
            BD[EP0IN].address[1] = temp.b[1];
            temp.w = val2;
            BD[EP0IN].address[2] = temp.b[0];
            BD[EP0IN].address[3] = temp.b[1];
            BD[EP0IN].bytecount = 4;    // set EP0 IN byte count to 4
            BD[EP0IN].status = 0xC8;    // send packet as DATA1, set UOWN bit
            break;            
        case PRINT_VALS:
            printf("val1 = %u, val2 = %u\n", val1, val2);
            BD[EP0IN].bytecount = 0;    // set EP0 IN byte count to 0
            BD[EP0IN].status = 0xC8;    // send packet as DATA1, set UOWN bit
            break;
		case TOGGLE_LED:
			led_toggle(&led1);
            BD[EP0IN].bytecount = 0;    // set EP0 IN byte count to 0 
            BD[EP0IN].status = 0xC8;    // send packet as DATA1, set UOWN bit
			break;
		case SET_DUTY:
			{
				uint16_t duty = USB_setup.wValue.w;
				pin_write(&D[13],duty);
				BD[EP0IN].bytecount = 0;    // set EP0 IN byte count to 0 
				BD[EP0IN].status = 0xC8;    // send packet as DATA1, set UOWN bit
			}
			break;
		case GET_DUTY:
			temp.w = pin_read(&D[13]);
			BD[EP0IN].address[0] = temp.b[0];
			BD[EP0IN].address[1] = temp.b[1];
			BD[EP0IN].bytecount = 2;
            BD[EP0IN].status = 0xC8;    // send packet as DATA1, set UOWN bit
			break;
        default:
            USB_error_flags |= 0x01;    // set Request Error Flag
    }
}

void VendorRequestsIn(void) {
    switch (USB_request.setup.bRequest) {
        default:
            USB_error_flags |= 0x01;                    // set Request Error Flag
    }
}

void VendorRequestsOut(void) {
    switch (USB_request.setup.bRequest) {
        default:
            USB_error_flags |= 0x01;                    // set Request Error Flag
    }
}

int16_t main(void) {
    init_clock();
	init_ui();
	init_pin();
	init_oc();

	//oc_pwm(&oc1,&D[13],
    init_uart();

    val1 = 0;
    val2 = 0;

    InitUSB();                              // initialize the USB registers and serial interface engine
    while (USB_USWSTAT!=CONFIG_STATE) {     // while the peripheral is not configured...
        ServiceUSB();                       // ...service USB requests
    }
    while (1) {
        ServiceUSB();                       // service any pending USB requests
    }
}

