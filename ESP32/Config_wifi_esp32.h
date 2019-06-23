/***********************************************************************
 *   WIFI FUNCTIONS                                                    *
 ***********************************************************************/
#ifndef Config_wifi_esp32
#define Config_wifi_esp32

  #define SSIDA "SSIDA"
  #define PASSPHARSE "PASSWORD"
  #define MESSAGE "o"
  #define TCPServerIP "IP"
  #define PORT 8090
  
  static EventGroupHandle_t wifi_event_group;
  int CONNECTED_BIT;
  static const char *TAG="tcp_client";
  void wifi_connect();
  static esp_err_t event_handler(void *ctx, system_event_t *event);
  static void initialise_wifi(void);
  bool wifi_work = false;  

#endif
