/*
** DVS GEN3 (hVGA) Viewer Example for Linux
** by CWS @ Samsung Electronics
** Maintainer: Eric Mitchell (eric.m1@samsung.com)
*/

#include <stdio.h>
#include <stdlib.h>
#include <ctime>
#include <stdint.h>
#include "cyusb.h"
#include <sys/time.h>

const float MAX_EVENT_DT = 4; //4 maximum time between event updates
const int MAX_EVENT_COUNT = 10000000; // maximum number of events per packet

struct timeval t;
double ctime() {
  gettimeofday(&t, NULL);
  return t.tv_sec * 1000.0 + t.tv_usec / 1000.0;
}

void add_event(uint32_t *outbuf, uint32_t count, uint32_t x, uint32_t y, uint32_t pol, uint32_t time) {
  outbuf[4*count] = x;
  outbuf[4*count+1] = y;
  outbuf[4*count+2] = pol;
  outbuf[4*count+3] = time;
}

extern "C" {
  int read(float period_fps,
	   void(*event_callback)(uint32_t*, int),
	   bool(*period_callback)(int)) {   
    static cyusb_handle *h1 = NULL;                               // to read from USB
    unsigned char	      *buf;                                     // to store USB output
    uint32_t                  *outbuf;                                     // to send USB output
    int	                      buflen, transferred = 0;                 // length of data read
    const int	              timeout = 1000;                          // CyUSB timeout
    uint32_t	              posX=0, posY0=0, grpAddr=0, pol=0, header; // packet contents
    
    unsigned int           wrapitup_flag =0;    
    uint32_t               remember_col=0;
    unsigned int           new_frame_flag=0;

    unsigned int	       longTs = 0;                              // * 10 usec
    unsigned int	       shortTs = 0;                             // * 1 msec
    unsigned int	       timeStamp = 0;                           // * usec
    struct timeval       structTime;
    double	       last_period_update, last_event_update, current_time; // for measuring FPS
    int                  chunk_counter = 0;                       // keep track of chunk
    int                  outcount = 0;
    int                  r, n;                                    // return values
    
    const int EVENT_SIZE = 4; // number of values (uint32_t) that comprise a message
    uint32_t event_count = 0;
    
    float DT = 1000.f / period_fps;
    r = cyusb_open();
    
    if ( r < 0 ) {
      printf("Error opening library\n");
      return -1;
    } else if ( r == 0 ) {
      printf("No device found\n");
      return 0;
    }
    
    if ( r > 1 ) {
      printf("More than 1 devices of interest found. Disconnect unwanted devices\n");
      return 0;
    }
    
    h1 = cyusb_gethandle(0);
    if ( cyusb_getvendor(h1) != 0x04b4 ) {
      printf("Cypress chipset not detected\n");
      cyusb_close();
      return 0;
    }
    r = cyusb_kernel_driver_active(h1, 0);
    if ( r != 0 ) {
      printf("kernel driver active. Exitting\n");
      cyusb_close();
      return 0;
    }
    r = cyusb_claim_interface(h1, 0);
    if ( r != 0 ) {
      printf("Error in claiming interface\n");
      cyusb_close();
      return 0;
    }
    
    buflen = cyusb_get_max_packet_size(h1, 0x81);
    printf("buffer size=%d\n", buflen);
    buf = (unsigned char *) malloc (buflen + 1);
    outbuf = (uint32_t *) malloc ((sizeof(uint32_t) * EVENT_SIZE) * MAX_EVENT_COUNT);
    
    last_period_update = ctime();
    last_event_update = ctime();
    
    while (true) {
      r = cyusb_bulk_transfer(h1, 0x81, buf, buflen, &transferred, timeout);
      if (transferred % 4) transferred = (transferred / 4) * 4;
      
      for(int i=0;i<transferred; i+=4){
	header = buf[i] & 0x7C;
	//id = buf[i] & 0x03;
	if (buf[i] & 0x80) {	// Group Packet
	  grpAddr = (buf[i+1] & 0xFC) >> 2;

	  if (buf[i + 3]) {
	    posY0 = grpAddr << 3;
	    pol = (buf[i+1] & 0x01)? 255 : 0;
	    for (n=0; n<8; n++) {
	      if ((buf[i+3] >> n) & 0x01) {
		add_event(outbuf, event_count, posX, posY0 + n, pol, timeStamp);
		event_count++;
		if (event_count == MAX_EVENT_COUNT) {
		  //wrapitup_flag=1;
          printf("this should never be run\n");
          event_callback(outbuf, event_count);
		  event_count = 0;
		}
	      }
	    }
	  }
	  
	  if (buf[i + 2]) {
	    grpAddr += (header >> 2);	// Offset
	    posY0 = grpAddr << 3;
	    pol = (buf[i+1] & 0x02)? 255 : 0;
	    for (n=0; n<8; n++) {
	      if ((buf[i+2] >> n) & 0x01) {
		add_event(outbuf, event_count, posX, posY0 + n, pol, timeStamp);
		event_count++;
		if (event_count == MAX_EVENT_COUNT) {
		  wrapitup_flag=1;
            //event_callback(outbuf, event_count);
		  //event_count = 0;
		}
	      }
	    }
	  }
	} else {					// Normal Packet
	  switch (header) {
	  case (0x04) :	// 0000 01** | --ST TTTT | TTTT T-CC | CCCC CCCC	Column Address (10) + SubTimestamp (10)
	    shortTs = ((buf[i+1] & 0x1F) << 5) | ((buf[i+2] & 0xF8) >> 3);
	    timeStamp = longTs + shortTs;
	    //posX = (((buf[i + 2] & 0x03) << 8) | (buf[i + 3] & 0xFF));		// Original
	    posX = 319 - (((buf[i + 2] & 0x03) << 8) | (buf[i + 3] & 0xFF));		// Rotation
        if ( (posX-remember_col) << 0 ){
        new_frame_flag = 1;
        } else {
        new_frame_flag = 0;
        }
        remember_col = posX;
	    break;
	  case (0x08) :	// 0000 01** | --TT TTTT | TTTT TTTT | TTTT TTTT	Reference Timestamp (22)
	    longTs = (((buf[i + 1] & 0x3F) << 16) | ((buf[i + 2] & 0xFF) << 8) | (buf[i + 3] & 0xFF)) * 1000;
	    break;
	  case (0x40) :	// 0100 00** | --II IIII | IIII IIII | IIII IIII	Packet ID (22)
	    // Packet ID is used to check packet loss
	    //packetID = ((buf[i + 1] & 0x3F) << 26) | ((buf[i + 2] & 0xFF) << 18) | ((buf[i + 3] & 0xFF) << 10);
	    break;
	  case (0x00) :	// 0000 0000 | 0000 0000 | 0000 0000 | 0000 0000	Padding
	    //i = dataLen;	// ignore all the remaining packet data
	    break;
	  default :		// This should not happen
	    break;
	  } // switch
	} // else (not group packet)
      } // for bytes in buf
      
      chunk_counter++;
     // printf("%d\n", chunk_counter);
      current_time = ctime();

      if (current_time - last_event_update > MAX_EVENT_DT) {
	wrapitup_flag = 1;
      }

      if ( wrapitup_flag == 1 && new_frame_flag == 1 ) {
	last_event_update = current_time;
	event_callback(outbuf, event_count);
    printf("%d\n", outcount);
    outcount++;
	event_count = 0;
    wrapitup_flag = 0;
    new_frame_flag = 0;
      }

      current_time = ctime();
      if (current_time - last_period_update > DT){
	    last_period_update = current_time;
	    if (!period_callback(chunk_counter)) {
            break;
	    }
        
      }
    if ((chunk_counter==100000)) {
	        //break;
	    }
    }
    
    cyusb_close();
    return 0;
  }
}

int main(int argc, char **argv) { }
