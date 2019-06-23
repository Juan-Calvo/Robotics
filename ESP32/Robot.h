#ifndef robot_h
#define robot_h
	//BASIC HEADERS
	#include <stdio.h>
	#include<string.h>

	//FREERTOS
	#include <freertos/FreeRTOS.h>
	#include <freertos/task.h>
	#include <freertos/event_groups.h>
	#include <esp_log.h>
	
	//LWIP
	#include <lwip/err.h>
	#include <lwip/sockets.h>
	#include <lwip/sys.h>
	#include <lwip/netdb.h>
	#include <lwip/dns.h>

	//TIMER
	#include <soc/timer_group_struct.h>
	#include <soc/timer_group_reg.h>
  #include <driver/timer.h>

  //WIFI
	#include <esp_task_wdt.h>
	#include <esp_wifi.h>
	#include <esp_system.h>
	#include <esp_event.h>
	#include <esp_event_loop.h>
	#include <tcpip_adapter.h>
  #include <nvs_flash.h>

  //PERSONAL HEADERS
  #include "Variables.h"
  #include "POINTS.h"
  #include "TB6560.h"
  #include "Config_wifi_esp32.h"
  #include "Timer_config.h"

#endif
