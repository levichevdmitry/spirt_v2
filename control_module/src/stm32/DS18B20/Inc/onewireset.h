
#ifndef		_ONEWIRESET_H_
#define		_ONEWIRESET_H_

#include 	"settings.h"
#include 	"uart_log.h"

// on/off debug one wire information
#if DEBUG == 1
	#define  	ONEWIRE_DEBUG	 		0 		// set 1 or 0 for on or off debug one wire, if global debug is on
	char debug_buf[5];
#else
	#define		ONEWIRE_DEBUG	 		0 		// set off debug, if global debug is off
#endif


#endif
