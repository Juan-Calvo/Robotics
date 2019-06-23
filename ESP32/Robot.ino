/***********************************************************************
 *   ROBOTIC ARM 5-DOF                                                 *
 *   Version: 1.0                                                      *
 ***********************************************************************/
#include "Robot.h"

int  sock, sock_read;
char recv_buf[64];
char *p;

char x[20];
char y[20];
char z[20];
char w[20];
char u[20];
char t[20];

const TickType_t xTicksToWait = ( TickType_t )0xff;
struct sockaddr_in tcpServerAddr;

TaskHandle_t Core0;
QueueHandle_t     TCP_POINTS_Readings;
POINTS Segment;  

void setup() {
  CONNECTED_BIT = BIT0;
  p= recv_buf;
  wifi_event_group = xEventGroupCreate();
  ESP_ERROR_CHECK( esp_event_loop_init(event_handler, NULL) );
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK( ret );
  while(true){
    initialise_wifi();
    if (wifi_work){break;}
  }
  if ( TCP_POINTS_Readings == 0){ Serial.println("Error"); }
  tcpServerAddr.sin_addr.s_addr = inet_addr(TCPServerIP);
  tcpServerAddr.sin_family = AF_INET;
  tcpServerAddr.sin_port = htons( PORT );
  TCP_POINTS_Readings  = xQueueCreate( 20 , sizeof(POINTS) );
  xTaskCreatePinnedToCore(
    Core0_main,     
    "Core0_main",   
    100000,         
    NULL,          
    1,             
    &Core0,         
    0
  );
  delay(1000);
}

//CORE 1
void loop() {
  for( ;; ){ 
    xEventGroupWaitBits(wifi_event_group,CONNECTED_BIT,false,true,portMAX_DELAY);
    sock = socket(AF_INET, SOCK_STREAM, 0);
    if(sock < 0) { continue; }
    if(connect(sock, (struct sockaddr *)&tcpServerAddr, sizeof(tcpServerAddr)) != 0) {
      close(sock);
      continue;
    }
    while(1){
      bzero(recv_buf, sizeof(recv_buf));
      bzero(x, sizeof(x));
      bzero(y, sizeof(y));
      bzero(z, sizeof(z));
      bzero(w, sizeof(w));
      bzero(u, sizeof(u));
      bzero(t, sizeof(t));
      sock_read = read(sock, recv_buf, sizeof(recv_buf)-1);
      write(sock , MESSAGE , strlen(MESSAGE));
      if(sock_read==0){ break; }
      int inc=0;
      if( *(p)== 'x'){
        for (int j=0; j<sock_read; j++){
          inc++;
          if(*(p + j+1)== 'y'){ break; }
          x[j]=*(p +j+1);
        }
      }
      int ae=0;
      if( *(p +inc)== 'y'){
        for (int j=inc; j<sock_read; j++){
          inc++;
          if(*(p + j+1)== 'z'){ ae=0; break; }
            y[ae]=*(p +j+1);
            ae++;
        }
      }
      if( *(p +inc)== 'z'){
        for (int j=inc; j<sock_read; j++){
          inc++;
          if(*(p + j+1)== 'w'){ ae=0; break; }
          z[ae]=*(p +j+1);
          ae++;
        }
      }
       if( *(p +inc)== 'w'){
        for (int j=inc; j<sock_read; j++){
          inc++;
          if(*(p + j+1)== 'u'){ ae=0; break; }
          w[ae]=*(p +j+1);
          ae++;
        }
      }
      if( *(p +inc)== 'u'){
        for (int j=inc; j<sock_read; j++){
          inc++;
          if(*(p + j+1)== 't'){ ae=0; break; }
          u[ae]=*(p +j+1);
          ae++;
        }
      }
      if( *(p +inc)== 't'){
        for (int j=inc; j<sock_read; j++){
          if(*(p + j+1)== ';'){ ae=0; break; }
          t[ae]=*(p +j+1);
          ae++;
        }
      }
      Segment.setx(atoi(x));
      Segment.sety(atoi(y));
      Segment.setz(atoi(z));
      Segment.setw(atoi(w));
      Segment.setu(atoi(u));
      Segment.sett(atoi(t));
      if(xQueueSendToBack( TCP_POINTS_Readings ,&Segment , portMAX_DELAY )!= pdPASS){ Serial.println("Error"); }
    }
    close(sock);
  }
}
 
//CORE 0
void Core0_main( void * pvParameters ){
  timer_init_1(3000);
  while(true){
    TIMERG0.wdt_wprotect = TIMG_WDT_WKEY_VALUE;
    TIMERG0.wdt_feed     = 1;
    TIMERG0.wdt_wprotect = 0;
  }
  vTaskDelete(Core0);
}
