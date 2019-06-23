#include <stdint.h>
#include "hal.h"
#include "report.h"
#include "host.h"
#include "timer.h"
#include "print.h"
#include "debug.h"
#include "pointing_device.h"

static report_mouse_t mouseReport = {};

#ifdef DEBUG_MOUSE_SCAN_RATE
uint32_t mouse_poll_timer;
uint32_t mouse_poll_scan_count;
#endif

void pointing_device_init(void){
    #ifdef DEBUG_MOUSE_SCAN_RATE
      mouse_poll_timer = timer_read32();
      mouse_poll_scan_count = 0;
    #endif
}

void pointing_device_send(void){
    //If you need to do other things, like debugging, this is the place to do it.
    host_mouse_send(&mouseReport);
	//send it and 0 it out except for buttons, so those stay until they are explicity over-ridden using update_pointing_device
	mouseReport.x = 0;
	mouseReport.y = 0;
	mouseReport.v = 0;
	mouseReport.h = 0;
}

void pointing_device_task(void){
    #ifdef DEBUG_MOUSE_SCAN_RATE
      mouse_poll_scan_count++;

      uint32_t timer_now = timer_read32();
      if (TIMER_DIFF_32(timer_now, mouse_poll_timer)>1000) {
        dprintf("mouse poll frequency: %u\n", mouse_poll_scan_count);

        mouse_poll_timer = timer_now;
        mouse_poll_scan_count = 0;
      }
    #endif

    //gather info and put it in:
    //mouseReport.x = 127 max -127 min
    //mouseReport.y = 127 max -127 min
    //mouseReport.v = 127 max -127 min (scroll vertical)
    //mouseReport.h = 127 max -127 min (scroll horizontal)
    //mouseReport.buttons = 0x1F (decimal 31, binary 00011111) max (bitmask for mouse buttons 1-5, 1 is rightmost, 5 is leftmost) 0x00 min
    //send the report
    if (false){ //only send if there is movement/clicks as it wakes up the pc otherwise
        pointing_device_send();
    }
}