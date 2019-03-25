#ifndef _ROS_H_
#define _ROS_H_

#include "ros/node_handle.h"
#include "ArduinoHardware.h"

namespace ros
{

	/*
	This set our NodeHandle to have 10 Publishers, 15 Subscriber, 128 bytes for input buffer and 256 bytes for output buffer.

	Number of publishers, number of subscribers, input buffer num bytes, output buffer num bytes.
	*/


  // default is 25, 25, 512, 512  
  typedef NodeHandle_<ArduinoHardware, 2, 2, 128, 256> NodeHandle;


  // This is legal too and will use the default 25, 25, 512, 512
  //typedef NodeHandle_<ArduinoHardware> NodeHandle;
}

#endif







// #ifndef _ROS_H_
// #define _ROS_H_

// #include "ros/node_handle.h"

// #if defined(ESP8266) or defined(ROSSERIAL_ARDUINO_TCP)
//   #include "ArduinoTcpHardware.h"
// #else
//   #include "ArduinoHardware.h"
// #endif

// namespace ros
// {
// #if defined(__AVR_ATmega8__) or defined(__AVR_ATmega168__)
//   /* downsize our buffers */
//   typedef NodeHandle_<ArduinoHardware, 6, 6, 150, 150> NodeHandle;

// #elif defined(__AVR_ATmega328P__)

//   typedef NodeHandle_<ArduinoHardware, 25, 25, 280, 280> NodeHandle;

// #elif defined(SPARK)

//   typedef NodeHandle_<ArduinoHardware, 10, 10, 2048, 2048> NodeHandle;

// #else

//   typedef NodeHandle_<ArduinoHardware> NodeHandle; // default 25, 25, 512, 512

// #endif   
// }

// #endif
