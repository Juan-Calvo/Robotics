/***********************************************************************
 *   WIFI FUNCTIONS                                                    *
 ***********************************************************************/
#include "Robot.h"
void wifi_connect(){
    wifi_config_t cfg = {
        .sta = {
            {.ssid = SSIDA},
            {.password = PASSPHARSE},
        },
    };
    ESP_ERROR_CHECK( esp_wifi_disconnect() );
    ESP_ERROR_CHECK( esp_wifi_set_config(ESP_IF_WIFI_STA, &cfg) );
    ESP_ERROR_CHECK( esp_wifi_connect() );
}

static esp_err_t event_handler(void *ctx, system_event_t *event){
    switch(event->event_id) {
    case SYSTEM_EVENT_STA_START:
    Serial.println("wifi connect");
        wifi_connect();
        break;
    case SYSTEM_EVENT_STA_GOT_IP:
    {Serial.println("got ip");
        xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
        wifi_work = true; 
        break;}
    case SYSTEM_EVENT_STA_DISCONNECTED:
        esp_wifi_connect();
        xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
        break;
    default:
        break;
    }
    return ESP_OK;
}

static void initialise_wifi(void){
    esp_log_level_set("wifi", ESP_LOG_NONE);
    tcpip_adapter_init();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    cfg.wifi_task_core_id=1;
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK( esp_wifi_start() );
}
