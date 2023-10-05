/* tuxctl-ioctl.c
 *
 * Driver (skeleton) for the mp2 tuxcontrollers for ECE391 at UIUC.
 *
 * Mark Murphy 2006
 * Andrew Ofisher 2007
 * Steve Lumetta 12-13 Sep 2009
 * Puskar Naha 2013
 */

#include <asm/current.h>
#include <asm/uaccess.h>

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/file.h>
#include <linux/miscdevice.h>
#include <linux/kdev_t.h>
#include <linux/tty.h>
#include <linux/spinlock.h>

#include "tuxctl-ld.h"
#include "tuxctl-ioctl.h"
#include "mtcp.h"

#define debug(str, ...) \
	printk(KERN_DEBUG "%s: " str, __FUNCTION__, ## __VA_ARGS__)

unsigned char global_buffer[2]; // global buffer to hold button interupt data from handle_packet
unsigned char previous_state[6]; // holds previous led state
int spam_flag = 0;
/************************ Protocol Implementation *************************/

/* tuxctl_handle_packet()
 * IMPORTANT : Read the header for tuxctl_ldisc_data_callback() in 
 * tuxctl-ld.c. It calls this function, so all warnings there apply 
 * here as well.
 */
void tuxctl_handle_packet (struct tty_struct* tty, unsigned char* packet)
{
    unsigned a, b, c;

    a = packet[0]; /* Avoid printk() sign extending the 8-bit */
    b = packet[1]; /* values when printing them. */
    c = packet[2];

	switch(a) {
		case(MTCP_ACK):
			spam_flag = 0;
			return;
		case(MTCP_BIOC_EVENT):
			global_buffer[0] = b;
			global_buffer[1] = c;
		case(MTCP_RESET):
			tuxctl_ioctl_init(tty);
			tuxctl_ldisc_put(tty, previous_state, 6);
		default:
			return;
	}
    printk("packet : %x %x %x\n", a, b, c);
}

/******** IMPORTANT NOTE: READ THIS BEFORE IMPLEMENTING THE IOCTLS ************
 *                                                                            *
 * The ioctls should not spend any time waiting for responses to the commands *
 * they send to the controller. The data is sent over the serial line at      *
 * 9600 BAUD. At this rate, a byte takes approximately 1 millisecond to       *
 * transmit; this means that there will be about 9 milliseconds between       *
 * the time you request that the low-level serial driver send the             *
 * 6-byte SET_LEDS packet and the time the 3-byte ACK packet finishes         *
 * arriving. This is far too long a time for a system call to take. The       *
 * ioctls should return immediately with success if their parameters are      *
 * valid.                                                                     *
 *                                                                            *
 ******************************************************************************/
int 
tuxctl_ioctl (struct tty_struct* tty, struct file* file, 
	      unsigned cmd, unsigned long arg)
{
 
    switch (cmd) {
		case TUX_INIT:
			tuxctl_ioctl_init(tty); // call initialization helper
			return 0; // success
		case TUX_BUTTONS:
			tuxctl_ioctl_buttons(tty, arg); // call button ioctl 
			return 0; // success
		case TUX_SET_LED:
			if(spam_flag) { // if spam flag is on right now
				return 0; 
			}
			spam_flag = 1; // set spam flag true
			tuxctl_ioctl_set_led(tty, arg); // call set_led ioctl
			return 0; 
		case TUX_LED_ACK:
		case TUX_LED_REQUEST:
		case TUX_READ_LED:
		default:
			return -EINVAL;
    }
}
/*
tuxctl_ioctl_init
	initializes the TUX driver with 
		button interrupt mode 
		and led user mode so they can be modified with the led_set opcode
*/
int
tuxctl_ioctl_init(struct tty_struct *tty) {
	
	unsigned char buf[2]; // two commands so size = 2 command buffer
	spam_flag = 1; // set spam flag true 
	buf[0] = MTCP_BIOC_ON; // sets the buttons to interrupt-on-change 
	buf[1] = MTCP_LED_USR; // sets the leds into user mode so they can be modified by the LED_SET opcode
	
	tuxctl_ldisc_put(tty, buf, 2); // writes our buffer to the tux 

	return 0; //success
}

	//arg here is pointer to 32 bit integer 
	// arg[7:0] is set corresponding to currently pressed buttons
	// | right | left | down | up | c | b | a | start |

/*
tuxctl_ioctl_buttons:
	inputs: arg -- a pointer to a user level unsigned long which is to be updated corresponding to the button presses
	outputs arg pointer data is updated according to button interrups
*/
int
tuxctl_ioctl_buttons(struct tty_struct *tty, unsigned long arg) {
	
	unsigned char b = global_buffer[0]; // low 4 bits of c -> low 4 bits of b shifted up
	unsigned char c = global_buffer[1];
	unsigned char b5, b6, argbits;
	// 0x0F = 0000 1111, bitmask for bottom 4 bits
	b = (b & 0x0F); // clear top 4 bits of b
	c = (c & 0x0F) << 4;  // clear top 4 bits of c and then shift up to be in XXXX 0000

	b5 = c & 0x20; // extracts b6 of c: ands with 0010 0000 = 0x20
	b6 = c & 0x40; // extracts b6 of c: ands with 0100 0000 = 0x40

	b5 = b5 << 1; // swap b5 and b6
	b6 = b6 >> 1;  
	argbits = c | b ; // value to be put in arg[7:0] but down and left swapped
									// | right | down | left | up | c | b | a | start |
	
	argbits = ( argbits & 0x9F ) | b5 | b6; 	// bitwise and argbits with 1001 1111 = 0x9F to clear 5th and 6th bit

	// argbits ^= 0xFF;

	(void)copy_to_user((unsigned int*)arg, &argbits, sizeof(argbits)); // copies argbits into lower 8 bits of the arg pointer
	
	return 0; // success
}
/* tuxtl_ioctl_set_led() 
 * 		arg is 32 bit int: 
 * 			arg[15:0] - Low 16 bits specify number to be displayed on the 7-segment displays
 * 			arg[19:16] - low 4 bits of the third byte specify which leds should be on
 * 			arg[27:24] - specify decimal points to be turned on
 * 		outputs: updates the leds with the specified arg parameters
*/ 
int 
tuxctl_ioctl_set_led(struct tty_struct *tty, unsigned long arg) {

	unsigned char buf[6]; // first byte is opcode, second byte selects leds, 
	int led_buffer[4]; // holds the 4 values for each led
	char led_on, decimal_on; // hold the 4 bits describing the led and decimal on or off
	unsigned char buffer_byte, led_decimal_bitmask; // buffer byte is byte added to command packet
													// led_decimal_bitmask is used to check if led/decimal is on 
	int i;  // iterator int

	unsigned char led[16] = 
	{ 
		0xE7, 0x06, 0xCB, 0x8F, 0x2E, 0xAD, 0xED, 0x86, 0xEF, 0xAE, 0xEE, 0x6D, 0xE1, 0x4F, 0xE9, 0xE8
	}; // 7 segment display bytes describing which leds to turn on for 0-9, A-F

	buf[0] = MTCP_LED_SET; 
	previous_state[0] = MTCP_LED_SET;
	buf[1] = 0xF; // set all LEDs on to be included in the select 0000 1111
	previous_state[1] = 0xF;
	for(i = 0; i < 4; i++) { 
		led_buffer[i] = arg >> i*4 & 0xF; //extracts the last 4 bits of arg while shifting each 4 bit chunk into that spot
	}

	//printk("TIME: %u%u:%u%u \n", led_buffer[0], led_buffer[1], led_buffer[2], led_buffer[3]);
	led_on = (char) ((arg >> 16) & 0x0F); 	// have to get arg[19:16] so right shift by 16 to move 19:16 into 3:0, then bitmask with 0x0F to get the low 4 bits 
	decimal_on = (char) ((arg >> 24) & 0x0F); 	// want arg[27:24], right shift by 24 to move 27:24 into 3:0, then bitmask with 0x0F to get the low 4 bits  
	//printk("decimals on: %x \n", decimal_on);
	//printk("leds on: %x \n", decimal_on);

	
	led_decimal_bitmask = 0x01; // 0000 0001 bit mask for checking if led or decimal on or not 
	
	for(i = 0; i < 4; i++) { // bytes need to be sent from led0 to led3  
		if(led_on & led_decimal_bitmask) { 
			buffer_byte = led[led_buffer[i]];
			
		}
		else{
			buffer_byte = 0x00; // if led not on set buffer_byte to 00
		}
		if(decimal_on & led_decimal_bitmask) {
				buffer_byte = buffer_byte | 0x10; // sets the decimal bit on if decimal chosen to be on
		}

		buf[i + 2] = buffer_byte;
		previous_state[i + 2] = buffer_byte;

		led_decimal_bitmask = led_decimal_bitmask << 1; // shift bitmask
	}
	tuxctl_ldisc_put(tty, buf, 6);
	return 0;
}
