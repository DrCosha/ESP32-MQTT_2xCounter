
/*******************************************************************
          Файл с описанием локальных параметров WiFi сети 
                     и доступа к MQTT серверу.

                  !!! не синхронизируется с Git !!! 
********************************************************************/

// начальные параметры устройства для подключения к WiFi и MQTT в рамках отладки 

#define P_WIFI_SSID "iot_ls"                      // SSID нашей локальной сети  
#define P_WIFI_PASSWORD "vvssoft40"               // пароль к нашей локальной сети
#define P_MQTT_USER "mqtt_user"                   // имя пользователя для подключения к MQTT серверу
#define P_MQTT_PWD "vvssoft40"                    // пароль для подключения к MQTT серверу
#define P_MQTT_HOST IPAddress(192, 168, 10, 100)  // адрес нашего Mosquito MQTT сервера
#define P_MQTT_PORT 1883                          // порт нашего Mosquito MQTT сервера
