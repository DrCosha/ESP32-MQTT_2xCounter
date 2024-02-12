/*
************************************************************************
*         Firmware для управления контроллером подсчёта импульсов
*                         (с) 2024, by Dr@Cosha
*                               ver 1.3b                    
************************************************************************


Модуль может использоватся для подсчёта внешних импульсов от счётчиков тепла/электричества/воды. 
Подсчёт ведется путем замыкания входных клемм [INP1] и [INP2]. Замыкание соответсвующего вывода индицируется светодиодом LED1 и LED2.
Переход от не замкнутого положения к замкнутому добавляет один импульс в значению соответсвующего счётчика. 
Модуль имеет детектор пропадания питания, при этом энергии в накопительных конденсаторах хватает для записи значения счётчиков в энергонезависимую FLASH память . 
Это сохраняет значения и минимизирует потери импульсов при перезагрузке устройства по питанию.
При этом есть есть отдельный счётчик, который ведет подсчёт количества пропаданий питания со времени своего последнего сброса.

При старте устройства текущие значения счётчиков вычитываются их FLASH памяти. Значения счётчиков могут быть сброшены принудительно, нажатием на 
кнопку CLEAR на плате. Однократное нажатие сбрасывает счётчик 1, двойное - счётчик 2.
Для полного сброса всех настроек модуля к базовым, необходимо более чем на 1 секунду нажать одновременно кнопки CLEAR и FLASH.

По умолчанию, модуль подключается к WiFi сети с настройками прошитыми в FLASH памяти. Там же хранятся настройки подключения и описания топиков MQTT сервера, 
через которые можно получить доступ к значениям счётчиков.

Если при старте модуля он не смог установить соединение с WiFi сетью вашего роутера, то будет поднята собственная точка доступа с именем CNTR_xxxx, 
где хххх - последние цифры MAC адреса ESP32s. После поднятия точки доступа, можно соединится со страницей настроек модуля,
которая будет доступна по адресу default gateway точки доступа. Эти же настройки доступны и при успешном подключении модуля к WiFi сети. Адрес при этом будет 
получен динамически от вашего маршрутизатора. Поддерживается только сеть 2.4МГц (это ограничение самого ESP32). Отдельным пунктом меню можно обнулить оба счётчика по отдельности. 

Встроенный WEB сервер можно так же использовать для получения значения счётчиков в текущий момент времени, или их установки. 
Для этого нужно:

- для получения данных обратится по адресу [адрес_модуля]/get_data?cntr=х - где х - номер счётчика, значение которого мы хотим получить 0..2 (0 - счётчик перезагрузок).
- для задания значений счётчиков обратится по адресу [адрес_модуля]/set_data?cntr=х&value=nnn - где х - номер счётчика, значение которого мы хотим установить 0..2 (0 - счётчик перезагрузок), 
  а nnn - новое значение этого счётчика;

Доступ к модулю через MQTT возможен при правильной настройке параметров подключения.  При этом это может быть как локальный, так и глобальный MQTT сервер. 
Работа с сервером идет через три топика:
- топик команд [SET] ;
- топик состояния подключения устройства [LWT];
- топик рапортов о текущих заначениях [STATUS];

Если модуль получает в топике [SET] команду {report}, то сразу публикует в топике [STATUS] своё текущее состояние. Иначе, своё текущее состояние модуль публикует каждые 60 минут.


Ниже приведены команды, которые будут исполнены при помещении их в топик [SET]:


{"report"} 			              - команда немедленной генерации отчёта в топик [STATUS]
{"reboot"} 			              - команда перезагрузки модуля с сохранением текущих счётчиков
{"clear":"config"} 		        - сброс текущей конфигурации до начальной и перезагрузки
{"clear":"cnt01"} 		        - сброс счётчика №1
{"clear":"cnt02"} 		        - сброс счётчика №2
{"clear":"reboot"}		        - сброс счётчика перезагрузок (считает от момента прошлого сброса)
{"set_value_1":<значение>}	  - установка значения счётчика №1*
{"set_value_2":<значение>}	  - установка значения счётчика №2*

	* допустимое значение счётчика от 0 до 4 294 967 295 (0xFFFFFFFF)


Ниже приведен пример отчета в JSON формате, генерируемого модулем в топик [STATUS]:


{"cnt01":<значение1>,"cnt02":<значение2>,"cnt_reboot":<значение3>,"ip":<xx.xx.xx.xx>}  - где:

	- <значение1>, <значение2>	- текущие значения счётчиков №1 и №2;
	- <значение3> 			- значение счётчика перезагрузок;
	- <xx.xx.xx.xx>			- текущий IP модуля для облегчения доступа к его страницам настроек.
*/


#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <EEPROM.h>

extern "C" {
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "freertos/semphr.h"
#include "esp_mac.h"
#include "soc/rtc_wdt.h"
}

#include "GyverButton.h"
#include <AsyncMqttClient.h>
#include <ArduinoJson.h>

#include "webPageConst.h"                         // сюда вынесены все константные строки для генерации WEB страниц

// устанавливаем режим отладки
// #define DEBUG_LEVEL_PORT                          // устанавливаем режим отладки через порт

#define FW_VERSION "v1.3b"                        // версия ПО

// определение пинов подключения переферии
#define PIN_INP_CH1 25                            // пин подключения кнопки POWER 
#define PIN_INP_CH2 26                            // пин подключения кнопки SELECTOR
#define PIN_INP_AC_CUTOFF 27                      // пин подключения кнопки UV_LIGHT 

#define LED_RED_PIN 2                             // пин LED power RED 
#define LED_BLUE_PIN 4                            // пин LED power BLUE

#define BTN_FLASH_PIN 0                           // пин подключения кнопки FLASH
#define BTN_CLEAR_PIN 15                          // пин подключения кнопки CLEAR


// определяем константы для задержек
#define C_WIFI_CONNECT_TIMEOUT 60000              // задержка для установления WiFi соединения (60 сек)
#define C_MQTT_CONNECT_TIMEOUT 30000              // задержка для установления MQTT соединения (30 сек)
#define C_WIFI_AP_WAIT 180000                     // таймуат поднятой AP без соединения с клиентами (после этого опять пытаемся подключится как клиент) (180 сек)
#define C_WIFI_CYCLE_WAIT 10000                   // таймуат цикла переустановки соединения с WiFi (10 сек)
#define C_BLINKER_DELAY 300                       // задержка для мигания индикаторными светодиодами
#define C_COUNTER_DELAY 50                        // задержка для подавления дребезга на счётных входах 50ms - нижняя граница пропускания 20Гц

// задержки в формировании MQTT отчета
#define C_REPORT_DELAY  3600000                   // 1 час между репортами

// начальные параметры устройства для подключения к WiFi и MQTT
#ifdef DEBUG_LEVEL_PORT
#include "local_secrets.h"                        // при отладке включаем локальные параметры доступа к WiFi и MQTT
#else
#define P_WIFI_SSID "wifi_ssid"                   // SSID нашей локальной сети  
#define P_WIFI_PASSWORD "wifi_pwd"                // пароль к нашей локальной сети
#define P_MQTT_USER "mqtt_user_name"              // имя пользователя для подключения к MQTT серверу
#define P_MQTT_PWD "mqtt_user_pwd"                // пароль для подключения к MQTT серверу
#define P_MQTT_HOST "192.168.1.1"                 // адрес нашего MQTT сервера
#define P_MQTT_PORT 1883                          // порт нашего MQTT сервера
#endif
#define DEF_WIFI_CHANNEL  13                      // канал WiFi по умолчанию

#define C_MAX_WIFI_FAILED_TRYS 3                  // количество попыток повтора поднятия AP точки перед выключением WIFI
#define C_MAX_MQTT_FAILED_TRYS 100                // количество попыток соединения с MQTT сервером перед тем как начать поднимать AP точку

// определяем топики для работы устройства по MQTT
#define P_LWT_TOPIC    "diy/wtr_cntr/LWT"         // топик публикации доступности устройства
#define P_SET_TOPIC    "diy/wtr_cntr/set"         // топик публикации команд для устройства
#define P_STATE_TOPIC  "diy/wtr_cntr/state"       // топик публикации основных параметров состояния устройства


// определяем константы для параметров и команд JSON формата в MQTT
// --- имена команд ---
#define jc_REPORT         "report"                // команда принудительного формирования отчета в топик
#define jc_REBOOT         "reboot"                // команда принудительной удаленной перезагрузки
#define jc_RESET          "reset"                 // команда принудительной удаленной перезагрузки

// --- имена ключей ---
#define jk_CLEAR          "clear"                 // ключ описания команды очистки (счётчиков или конфигурации)
#define jk_SET_VALUE_01   "set_value_1"           // ключ описания установки значения счётчика 1
#define jk_SET_VALUE_02   "set_value_2"           // ключ описания установки значения счётчика 2
#define jk_COUNTER_01     "cnt01"                 // ключ описания значения счётчика 1
#define jk_COUNTER_02     "cnt02"                 // ключ описания значения счётчика 2
#define jk_COUNTER_RB     "cnt_reboot"            // ключ описания значения счётчика перезагрузок
#define jk_IP             "ip"                    // ключ описания ip адреса

// --- значения ключей и команд ---
#define jv_ONLINE         "online"                // 
#define jv_OFFLINE        "offline"               //
#define jv_COUNTER_01     "cnt01"                 //
#define jv_COUNTER_02     "cnt02"                 //
#define jv_COUNTER_RB     "reboot"                //
#define jv_CONFIG         "config"                //

// тип описывающий режим работы WIFI - работа с самим WiFi и MQTT 
enum WiFi_mode_t : uint8_t {
  WF_UNKNOWN,                                     // режим работы WiFi еще не определен
  WF_OFF,                                         // при пакете ошибок при работе с WIFI - выключение WIFI и выключение режима ESP.NOW  
  WF_AP,                                          // поднятие собственной точки доступа со страничкой настройки   
  WF_CLIENT,                                      // включение WIFI в режиме клиента 
  WF_MQTT,                                        // соединение с MQTT сервером
  WF_IN_WORK,                                     // все хорошо, работаем
  WF_WITHOUT_MQTT                                 // работаем, но без MQTT
};  

// перечислимый тип описания счётчиков
enum Counters_t : uint8_t {
  CN_REBOOT = 0,                                  // счётчик перезагрузки
  CN_CNT01,                                       // счётчик 01
  CN_CNT02                                        // счётчик 02
};

// структура данных хранимых в EEPROM
struct GlobalParams {
// параметры режима работы усилителя
  uint32_t        counter_01;                     // значение счётчика №1
  uint32_t        counter_02;                     // значение счётчика №2
  uint16_t        counter_reboot;                 // значение счётчика перезагрузок
// параметры подключения к MQTT и WiFi  
  char            wifi_ssid[40];                  // строка SSID сети WiFi
  char            wifi_pwd[40];                   // пароль к WiFi сети
  char            mqtt_usr[40];                   // имя пользователя MQTT сервера
  char            mqtt_pwd[40];                   // пароль к MQTT серверу
  char            mqtt_host_s[80];                // адрес сервера MQTT
  uint16_t        mqtt_port;                      // порт подключения к MQTT серверу
// параметры очередей MQTT
  char            command_topic[80];              // топик получения команд
  char            report_topic[80];               // топик отправки текущего состояния устройства
  char            lwt_topic[80];                  // топик доступности устройства
// контрольная сумма блока для EEPROM
  uint16_t        simple_crc16;                   // контрольная сумма блока параметров
};

// объявляем текущие переменные состояния
bool s_EnableEEPROM = false;                    // глобальная переменная разрешения работы с EEPROM
WiFi_mode_t s_CurrentWIFIMode = WF_UNKNOWN;     // текущий режим работы WiFI
uint8_t count_GetWiFiConfig = 0;                // счётчик повторов попыток соединения c WIFI точкой
uint8_t count_GetMQTTConfig = 0;                // счётчик повторов попыток соединения с MQTT сервером

// временные моменты наступления контрольных событий в миллисекундах 
uint32_t tm_LastFireInp01 = 0;                  // момент сработки входа №1
uint32_t tm_LastFireInp02 = 0;                  // момент сработки входа №2
uint32_t tm_LastBlink = 0;                      // момент последнего срабатывания переключения мигания
uint32_t tm_LastReportToMQTT = 0;               // момент последнего отчета по MQTT

// общие флаги программы - команды и изменения 
bool f_Blinker = false;                         // флаг "мигания" - переключается с задержкой C_BLINKER_DELAY
bool f_WEB_Server_Enable = false;               // флаг разрешения работы встроенного WEB сервера
bool f_Has_WEB_Server_Connect = false;          // флаг обнаружения соединения с WEB страницей встроенного WEB сервера
bool f_HasMQTTCommand = false;                  // флаг наличия команды по MQTT
bool f_Has_Report = false;                      // флаг необходимости вывода отчета
bool f_FireInp01 = false;                       // флаг не обработанного прерывания по входу 01
bool f_FireInp02 = false;                       // флаг не обработанного прерывания по входу 02
bool f_FireCutOff = false;                      // флаг сработки прерывания у сенсора пропажи питания

// создаем буфера и структуры данных
GlobalParams   curConfig;                       // набор параметров управляющих текущей конфигурацией

// создаем и инициализируем объекты - кнопки
GButton bttn_flash(BTN_FLASH_PIN, HIGH_PULL, NORM_OPEN);      // инициализируем кнопку FLASH
GButton bttn_clear(BTN_CLEAR_PIN, HIGH_PULL, NORM_OPEN);      // инициализируем кнопку CLEAR

// объявляем объект MQTT клиент 
AsyncMqttClient   mqttClient;                  // MQTT клиент

// объявляем объект локальный WEB сервер
WebServer WEB_Server;

// создаем объект - JSON документ для приема/передачи данных через MQTT
StaticJsonDocument<512> InputJSONdoc,          // создаем входящий json документ с буфером в 512 байт 
                        OutputJSONdoc;         // создаем исходящий json документ с буфером в 512 байт 

// создаем мьютексы для синхронизации доступа к данным
SemaphoreHandle_t sem_InputJSONdoc = xSemaphoreCreateBinary();                           // создаем двоичный семафор для доступа к JSON документу 
SemaphoreHandle_t sem_CurConfigWrite = xSemaphoreCreateBinary();                         // создаем двоичный семафор для блокирования конфигурации при записи в EEPROM  

// наименование 
String ControllerName = "CNTR_";                                                         // имя нашего контроллера

// =============================== общие процедуры и функции ==================================

uint16_t GetCrc16Simple( uint8_t * data, uint16_t len ) { // процедура упрощенного расчета CRC16 для блока данных
  uint8_t lo;
  union // представляем crc как слово и как верхний и нижний байт
  {
    uint16_t value;
    struct { uint8_t lo, hi; } bytes;
  } crc;
 
  crc.value = 0xFFFF;  // начальное значение для расчета
  while ( len-- )
    {
        lo = crc.bytes.lo;
        crc.bytes.lo = crc.bytes.hi;
        crc.bytes.hi = lo ^ *data++;   
        uint8_t mask = 1;
        if ( crc.bytes.hi & mask ) crc.value ^= 0x0240;
        if ( crc.bytes.hi & ( mask << 1 ) ) crc.value ^= 0x0480;
        if ( crc.bytes.hi & ( mask << 2 ) ) crc.bytes.hi ^= 0x09;
        if ( crc.bytes.hi & ( mask << 3 ) ) crc.bytes.hi ^= 0x12;
        if ( crc.bytes.hi & ( mask << 4 ) ) crc.bytes.hi ^= 0x24;
        if ( crc.bytes.hi & ( mask << 5 ) ) crc.bytes.hi ^= 0x48;
        if ( crc.bytes.hi & ( mask << 6 ) ) crc.bytes.hi ^= 0x90;
        if ( crc.bytes.hi & ( mask << 7 ) ) crc.value ^= 0x2001;
    }
     return crc.value;
}

static void Halt(const char *msg) { //  процедура аварийного останова контроллера при критических ошибках в ходе выполнения
#ifdef DEBUG_LEVEL_PORT       // вывод в порт при отладке кода
  Serial.println(msg);        // выводим сообщение
  Serial.flush();
#endif
  esp_deep_sleep_start();     // останавливаем контроллер
}

void SetConfigByDefault() { // устанавливаем значения в блоке конфигурации по умолчанию
      memset((void*)&curConfig,0,sizeof(curConfig));    // обнуляем область памяти и заполняем ее значениями по умолчанию
      curConfig.counter_01 = 0;                                                       // счётчик 1 = 0
      curConfig.counter_02 = 0;                                                       // счётчик 2 = 0
      curConfig.counter_reboot = 0;                                                   // счётчик перезагрузок = 0
      memcpy(curConfig.wifi_ssid,P_WIFI_SSID,sizeof(P_WIFI_SSID));                    // сохраняем имя WiFi сети по умолчанию      
      memcpy(curConfig.wifi_pwd,P_WIFI_PASSWORD,sizeof(P_WIFI_PASSWORD));             // сохраняем пароль к WiFi сети по умолчанию
      memcpy(curConfig.mqtt_usr,P_MQTT_USER,sizeof(P_MQTT_USER));                     // сохраняем имя пользователя MQTT сервера по умолчанию
      memcpy(curConfig.mqtt_pwd,P_MQTT_PWD,sizeof(P_MQTT_PWD));                       // сохраняем пароль к MQTT серверу по умолчанию
      memcpy(curConfig.mqtt_host_s,P_MQTT_HOST,sizeof(P_MQTT_HOST));                  // сохраняем наименование хоста MQTT
      memcpy(curConfig.command_topic,P_SET_TOPIC,sizeof(P_SET_TOPIC));                // сохраняем наименование командного топика
      memcpy(curConfig.report_topic,P_STATE_TOPIC,sizeof(P_STATE_TOPIC));             // сохраняем наименование топика состояния
      memcpy(curConfig.lwt_topic,P_LWT_TOPIC,sizeof(P_LWT_TOPIC));                    // сохраняем наименование топика доступности
      curConfig.mqtt_port = P_MQTT_PORT;
       // расчитываем контрольную сумму блока данных
      curConfig.simple_crc16 = GetCrc16Simple((uint8_t*)&curConfig, sizeof(curConfig)-4);     // считаем CRC16      
}

bool ReadEEPROMConfig (){ // чтение конфигурации из EEPROM в буфер curConfig
  uint16_t tmp_CRC;

  EEPROM.get(0,curConfig);                                                 // читаем блок конфигурации из EEPROM
  tmp_CRC = GetCrc16Simple((uint8_t*)&curConfig, sizeof(curConfig)-4);     // считаем CRC16
  return (tmp_CRC==curConfig.simple_crc16);                                // возвращаем сошлась ли CRC16 
}

void CheckAndUpdateEEPROM() { // проверяем конфигурацию и в случае необходимости - записываем новую
  GlobalParams  oldConfig;        // это старый сохраненный конфиг
  uint16_t cur_CRC, old_CRC;      // это переменные для рассчета CRC сохраненного и текущего конфига

  if (!s_EnableEEPROM) return;    // если работаем без EEPROM - выходим сразу
  // иначе читаем старый конфиг в oldConfig, сравниваем его с текущим curConfig и если нужно, записываем в EEPROM
  EEPROM.get(0,oldConfig);                                                  // читаем блок конфигурации из EEPROM
  old_CRC = GetCrc16Simple((uint8_t*)&oldConfig, sizeof(oldConfig)-4);      // считаем CRC16 для него
  cur_CRC = GetCrc16Simple((uint8_t*)&curConfig, sizeof(curConfig)-4);      // считаем CRC16 для текущих параметров
  curConfig.simple_crc16 = cur_CRC;                                         // сохраняем CRC16 текущего блока параметров  
#ifdef DEBUG_LEVEL_PORT                
  if (cur_CRC != old_CRC) { //  если конфигурации отличаются - сохраняем новую
      EEPROM.put(0,curConfig);     
      if (EEPROM.commit()) Serial.println("EPPROM update successful.");
         else Serial.println("Error for update configuration in EPPROM.");
    } else { 
        Serial.println(" EPPROM is not need to updated."); 
      }
#else
  if (cur_CRC != old_CRC) { //  если конфигурации отличаются - сохраняем новую
      EEPROM.put(0,curConfig);     
      EEPROM.commit();
  }    
#endif      
}

bool isNumeric(String str, bool isInt) { // проверка, что строка содержит числo
// проверяем, что в строке содержится число и флаг, должно ли оно быть целым
    unsigned int stringLength = str.length(); 
    if (stringLength == 0) {
        return false;
    } 
    for(unsigned int i = 0; i < stringLength; ++i) {
        if (isDigit(str.charAt(i))) {
            continue;
        }
        if (str.charAt(i) == '.') {
            if (isInt) {
                return false;
            }
            isInt = true;
            continue;
        }
        return false;
    }
    return true;
}

// ------------------------ команды, которые обрабатываются в рамках получения событий ---------------------

void cmdReset() { // команда сброса конфигурации до состояния по умолчанию и перезагрузка
  #ifdef DEBUG_LEVEL_PORT       // вывод в порт при отладке кода 
  Serial.println("!!! Start rebooting process !!!");
  #endif    
  CheckAndUpdateEEPROM();                                                                    // проверяем конфигурацию и в случае необходимости - записываем новую
  if (mqttClient.connected()) mqttClient.publish(curConfig.lwt_topic, 0, true, jv_OFFLINE);  // публикуем в топик LWT_TOPIC событие об отключении
  vTaskDelay(pdMS_TO_TICKS(500));                                                            // задержка для публикации  
  ESP.restart();                                                                             // перезагружаемся  
}

void cmdClearConfig_Reset() { // команда сброса конфигурации до состояния по умолчанию и перезагрузка
  if (s_EnableEEPROM) { // если EEPROM разрешен и есть             
      SetConfigByDefault();                                                                   // в конфигурацию записываем значения по умолчанию
      curConfig.simple_crc16 = GetCrc16Simple((uint8_t*)&curConfig, sizeof(curConfig)-4);     // считаем CRC16 для конфигурации
  }  
  cmdReset();                                                                                 // перезагружаемся  
}

void cmdSetCounterValue(const Counters_t Cntr, uint32_t CntrValue) { // функция принудительной установки значения счётчика
  #ifdef DEBUG_LEVEL_PORT       // вывод в порт при отладке кода 
  Serial.printf("Set counter [%u] = [%u] \n",(uint8_t)Cntr,CntrValue);
  #endif   
  switch (Cntr) {
  case CN_REBOOT:
    curConfig.counter_reboot = CntrValue;    
    break;
  case CN_CNT01:
    curConfig.counter_01 = CntrValue;    
    break;
  case CN_CNT02:
    curConfig.counter_02 = CntrValue;    
    break;
  }
  curConfig.simple_crc16 = GetCrc16Simple((uint8_t*)&curConfig, sizeof(curConfig)-4);     // считаем CRC16 для текущих параметров  
  f_Has_Report = true; 
}

// ------------------------- обработка событий по генерации страниц WEB сервера -------------------------------

void handleRootPage() { // процедура генерации основной страницы сервера
  String tmpStr; 
  String out_http_text = CSW_PAGE_TITLE;
  out_http_text += ControllerName + " values</title>" + CSW_PAGE_STYLE + R"=====(<script> function wl(f){window.addEventListener('load',f);}function gv(count_num) {var xhttp = new XMLHttpRequest();	xhttp.onreadystatechange = function() {
 if (this.readyState == 4 && this.status == 200){	document.getElementById("in"+count_num).value = this.responseText;}};	xhttp.open("GET", "get_data?cntr="+count_num, true); xhttp.send();}			
 function sv(count_num) {var xhttp = new XMLHttpRequest(); xhttp.onreadystatechange = function() { if (this.readyState == 4 && this.status == 200) { if (this.responseText.length == 0) {	alert ("Wrong value for counter!"); window.location='/'; }
 else { if (this.responseText.startsWith("Error")) { alert (this.responseText); window.location='/'; } else document.getElementById("in"+count_num).value = this.responseText;}}}; 
 xhttp.open("GET", "set_data?cntr="+count_num+"&value="+document.getElementById("in"+count_num).value, true);	xhttp.send();} function jd(){ var t=0, i=document.querySelectorAll('input,button,textarea,select');	while(i.length>=t){ 
 if(i[t]){ i[t]['name']=(i[t].hasAttribute('id')&&(!i[t].hasAttribute('name')))?i[t]['id']:i[t]['name'];} t++;}} wl(jd);</script></head>
 <body><div style="text-align:left;display:inline-block;color:#eaeaff;min-width:340px;"><div style="text-align:center;color:#eaeaea;"><noscript>To use this page, please enable JavaScript<br></noscript><h3>Signal counting module:</h3><h2>)=====";
  out_http_text += ControllerName + R"=====(</h2><h4 style="color: #8f8f8f;">firmware )=====" + FW_VERSION + R"=====(</h4></div><fieldset><legend><b>&nbsp;Counter values&nbsp;</b></legend><p><b>Counter for input 01</b><br><input id="in1" placeholder=" " value=")=====";
  tmpStr = String(curConfig.counter_01);
  out_http_text += tmpStr + R"=====(" name="in1"><div/> <button style="width:48%;" name="" onclick="gv(1)">Load current</button> <button class="button bgrn" style="width:48%;" name="" onclick="sv(1)">Set value</button><hr></p><p>
 <b>Counter for input 02</b><br><input id="in2" placeholder=" " value=")=====";
  tmpStr = String(curConfig.counter_02);
  out_http_text += tmpStr + R"=====(" name="in2"><div/> <button style="width:48%;" name="" onclick="gv(2)">Load current</button> <button class="button bgrn" style="width:48%;" name="" onclick="sv(2)">Set value</button><hr></p><p>
 <b>Reboot counter</b><br><input id="in0" placeholder=" " value=")=====";
  tmpStr = String(curConfig.counter_reboot);
  out_http_text += tmpStr + R"=====(" name="in0"><div/><button class="button bgrn" style="width:100%;" name="" onclick="sv(0)">Set value</button></p></fieldset><div></div><p></p><form action="config" method="get">
 <button style="width:100%;">Configuration</button> <div></div></form><hr><form action="reboot" method="get"><div></div> <button class="button bred" name="">Reset</button>)=====" + CSW_PAGE_FOOTER;
  #ifdef DEBUG_LEVEL_PORT       // вывод в порт при отладке кода 
  Serial.println("WEB >>> index page");    
  #endif  
  WEB_Server.send ( 200, "text/html", out_http_text );
  f_Has_WEB_Server_Connect = true;                                            // взводим флаг наличия изменений
}

void handleConfigPage() { // процедура генерации страницы с конфигурацией 
  String tmpStr; 
  String out_http_text = CSW_PAGE_TITLE;
  out_http_text += ControllerName +
 R"=====( config</title><script> var x=null,lt,to,tp,pc='';function eb(s){return document.getElementById(s);}function qs(s){return document.querySelector(s);}
 function sp(i){eb(i).type=(eb(i).type==='text'?'password':'text');}function wl(f){window.addEventListener('load',f);}function jd(){var t=0,i=document.querySelectorAll('input,button,textarea,select'); 
 while(i.length>=t){ if(i[t]){i[t]['name']=(i[t].hasAttribute('id')&&(!i[t].hasAttribute('name')))?i[t]['id']:i[t]['name'];}t++;}} wl(jd); </script>)=====" + CSW_PAGE_STYLE +
 R"=====( </head><body> <div style="text-align:left;display:inline-block;color:#eaeaff;min-width:340px;"> <div style="text-align:center;color:#eaeaea;"> <noscript>To use this page, please enable JavaScript<br></noscript>
 <h3>Configuration for signal counting module:</h3><h2>)=====";
  out_http_text += ControllerName +
 R"=====(</h2></div><fieldset><legend><b>&nbsp;Network parameters&nbsp;</b></legend>
 <form method="get" action="applay"><p><b>WiFi SSID</b> [)=====";
  tmpStr = String(curConfig.wifi_ssid);
  out_http_text += tmpStr +
 R"=====(]<br><input id="wn" placeholder=" " value=")=====";
  out_http_text += tmpStr +
 R"=====(" name="wn"></p><p><b>WiFi password</b><input type="checkbox" onclick="sp(&quot;wp&quot;)" name=""><br>
 <input id="wp" type="password" placeholder="Password" value="****" name="wp"></p><p><b>IP for MQTT host</b> [)=====";
  tmpStr = String(curConfig.mqtt_host_s);
  out_http_text += tmpStr + R"=====(]<br><input id="mh" placeholder=" " value=")=====";
  out_http_text += tmpStr + R"=====(" name="mh"></p><p><b>Port</b> [)=====";
  tmpStr = String(curConfig.mqtt_port);
  out_http_text += tmpStr + R"=====(]<br><input id="ms" placeholder=")=====";
  out_http_text += tmpStr + R"=====(" value=")=====";
  out_http_text += tmpStr + R"=====(" name="ms"></p><p><b>MQTT User</b> [)=====";
  tmpStr = String(curConfig.mqtt_usr);
  out_http_text += tmpStr + R"=====(]<br><input id="mu" placeholder="MQTT_USER" value=")=====";
  out_http_text += tmpStr + R"=====(" name="mu"></p><p><b>MQTT user password</b><input type="checkbox" onclick="sp(&quot;mp&quot;)" name=""><br>
 <input id="mp" type="password" placeholder="Password" value="****" name="mp"></p><p><b>Set topic</b> [)=====";
  tmpStr = String(curConfig.command_topic);
  out_http_text += tmpStr + R"=====(]<br><input id="ts" placeholder=")=====";
  out_http_text += tmpStr + R"=====(" value=")=====";
  out_http_text += tmpStr + R"=====(" name="ts"></p><p><b>State topic</b> [)=====";
  tmpStr = String(curConfig.report_topic);
  out_http_text += tmpStr + R"=====(]<br><input id="tr" placeholder=")=====";
  out_http_text += tmpStr + R"=====(" value=")=====";
  out_http_text += tmpStr + R"=====(" name="tr"></p><p><b>LWT topic</b> [)=====";
  tmpStr = String(curConfig.lwt_topic);
  out_http_text += tmpStr + R"=====(]<br><input id="tl" placeholder=")=====";
  out_http_text += tmpStr + R"=====(" value=")=====";
  out_http_text += tmpStr + R"=====(" name="tl"></p><br><button name="save" type="submit" class="button bgrn">Save</button></form></fieldset> 
 <p></p><form action="config" method="get"><div></div><button name="">Reload current</button></form><div></div><form action="/" method="get">
 <button name="">Main page</button><div></div></form><hr><form action="reboot" method="get"><div></div><button class="button bred" name="">Reset</button>
  )=====" + CSW_PAGE_FOOTER;
  #ifdef DEBUG_LEVEL_PORT       // вывод в порт при отладке кода 
  Serial.println("WEB >>> config page");    
  #endif  
  WEB_Server.send ( 200, "text/html", out_http_text );
  f_Has_WEB_Server_Connect = true;                                            // взводим флаг наличия изменений
}

void handleRebootPage() { // процедура обработки страницы c ожидания
  String message_str = " Please wait for restart...";
  String out_http_text = CSW_PAGE_TITLE;  
  out_http_text += ControllerName + " reboot</title>" + CSW_PAGE_STYLE +
 R"=====(<script>setInterval(function(){getData();},1000);function getData() {var xhttp = new XMLHttpRequest(); xhttp.onreadystatechange=function() {if (this.readyState == 4 && this.status == 200) {
 if (this.responseText=="alive"){window.location='/';}}};xhttp.open("GET","alive",true);xhttp.send();}</script>	
 </head><body><div style='text-align:left;display:inline-block;color:#eaeaea;min-width:340px;'><div style="text-align:center;color:#eaeaea;"><h3>Signal counting module:</h3><h2>)=====";
  out_http_text += ControllerName + R"=====(</h2><br><noscript>To use this page, please enable JavaScript<br></noscript><br><div><a id="blink">)=====";
  if (f_ApplayChanges) out_http_text += "Changes applied." + message_str; 
    else out_http_text += "Reset and reboot." + message_str; 
  out_http_text += R"=====(</a></div><br><div></div><p><form action='/' method='get'><button>Main page</button>)=====" + CSW_PAGE_FOOTER;
  #ifdef DEBUG_LEVEL_PORT       // вывод в порт при отладке кода 
  Serial.println("WEB >>> reboot page");    
  #endif  
  f_ApplayChanges = false;
  WEB_Server.send(200, "text/html", out_http_text);
  vTaskDelay(pdMS_TO_TICKS(500));                                   // делаем задержку перед перезагрузкой чтобы сервер успел отправить страницы
  cmdReset();
} 

void handleNotFoundPage() { // процедура генерации страницы сервера c 404-й ошибкой
  String out_http_text = CSW_PAGE_TITLE;
  out_http_text += ControllerName +" - Page not found</title>" + CSW_PAGE_STYLE;
  out_http_text += R"=====(</head><body><div style='text-align:left;display:inline-block;color:#eaeaea;min-width:340px;'>
 <div style="text-align:center;color:#eaeaea;"><h3>Signal counting module:</h3><h2>)=====";
  out_http_text += ControllerName + R"=====(</h2><div><a id="blink" style="font-size:2em" > 404! Page not found...</a>
 </div><br><div></div><p><form action='/' method='get'><button>Main page</button>)=====" + CSW_PAGE_FOOTER;
   #ifdef DEBUG_LEVEL_PORT       // вывод в порт при отладке кода 
  Serial.println("WEB >>> not found page");    
  #endif  
  WEB_Server.send ( 404, "text/html", out_http_text );
}

void handleApplayPage() { // обработка страницы с приемом данных в контроллер со страницы клиента
  String ArgName  = "";
  String ArgValue = "";
  uint16_t _Int = 0;
  if (WEB_Server.args() > 0) {                                                  // если параметры переданы - то занимаемся их обработкой  
    for (size_t i = 0; i < WEB_Server.args(); i++) {                            // идем по списку переданных на страницу значений и обрабатываем их 
      ArgName = WEB_Server.argName(i);                                          // имя текущего параметра        
      ArgValue = WEB_Server.arg(i);                                             // значение текущего параметра  
      ArgValue.trim();                                                          // чистим от пробелов     
      // Аргумент [wn] >> SSID WiFi сети
      if (ArgName.equals("wn") and !ArgValue.isEmpty()) {                       // валидно не пустое значение
        strcpy(curConfig.wifi_ssid, ArgValue.c_str());                          // присваиваем новый SSID сети
        #ifdef DEBUG_LEVEL_PORT       // вывод в порт при отладке кода 
        Serial.printf("Argument [%s] >> curConfig.wifi_ssid = [%s]\n",ArgName, curConfig.wifi_ssid);
        #endif  
      }
      // Аргумент [wp] >> пароль для WiFi сети
      if (ArgName.equals("wp") and !ArgValue.equals("****")) {                  // проверяем на то, что в поле есть актуальное значение отличное от [****] 
        strcpy(curConfig.wifi_pwd, ArgValue.c_str());                           // присваиваем новый пароль сети
        #ifdef DEBUG_LEVEL_PORT       // вывод в порт при отладке кода 
        Serial.printf("Argument [%s] >> curConfig.wifi_pwd = [%s]\n",ArgName, curConfig.wifi_pwd);
        #endif  
      }
      // Аргумент [mh] >> хост для доступа к MQTT серверу
      if (ArgName.equals("mh") and !ArgValue.isEmpty()) {                       // валидно не пустое значение
        strcpy(curConfig.mqtt_host_s, ArgValue.c_str());                        // присваиваем имя MQTT хоста
        #ifdef DEBUG_LEVEL_PORT       // вывод в порт при отладке кода 
        Serial.printf("Argument [%s] >> curConfig.mqtt_host_s = [%s]\n",ArgName, curConfig.mqtt_host_s);
        #endif  
      }
      // Аргумент [ms] >> порт для доступа к MQTT
      if (ArgName.equals("ms") and !ArgValue.isEmpty()) {                       // проверяем на то, что в поле есть актуальное значение от 1..0xFFFF
        _Int = ArgValue.toInt();
        if (_Int>1 and _Int < 0xFFFF) {                                         // если это валидное значение порта, то присваиваем конфигурации
          curConfig.mqtt_port = _Int;                           
          #ifdef DEBUG_LEVEL_PORT       // вывод в порт при отладке кода 
          Serial.printf("Argument [%s] >> curConfig.mqtt_port = [%u]\n",ArgName, curConfig.mqtt_port);
          #endif  
        }
      }
      // Аргумент [mu] >> MQTT user
      if (ArgName.equals("mu") and !ArgValue.isEmpty()) {                       // валидно не пустое значение
        strcpy(curConfig.mqtt_usr, ArgValue.c_str());                           // присваиваем новое имя MQTT пользователя
        #ifdef DEBUG_LEVEL_PORT       // вывод в порт при отладке кода 
        Serial.printf("Argument [%s] >> curConfig.mqtt_usr = [%s]\n",ArgName, curConfig.mqtt_usr);
        #endif  
      }
      // Аргумент [mp] >> пароль для MQTT сервера
      if (ArgName.equals("mp") and !ArgValue.equals("****")) {                  // проверяем на то, что в поле есть актуальное значение отличное от [****] 
        strcpy(curConfig.mqtt_pwd, ArgValue.c_str());                           // присваиваем новый пароль MQTT пользователю
        #ifdef DEBUG_LEVEL_PORT       // вывод в порт при отладке кода 
        Serial.printf("Argument [%s] >> curConfig.mqtt_pwd = [%s]\n",ArgName, curConfig.mqtt_pwd);
        #endif  
      }
      // Аргумент [ts] >> MQTT топик для приема команд
      if (ArgName.equals("ts") and !ArgValue.isEmpty()) {                       // проверяем на то, что в поле есть актуальное значение от 1..0xFFFF      
        if (ArgValue.endsWith("/")) ArgValue.remove(ArgValue.length()-1,1);     // если есть обратная косая черта - удаляем
        strcpy(curConfig.command_topic, ArgValue.c_str());                      // присваиваем значение переменной 
        #ifdef DEBUG_LEVEL_PORT       // вывод в порт при отладке кода 
        Serial.printf("Argument [%s] >> curConfig.command_topic = [%s]\n",ArgName, curConfig.command_topic);
        #endif  
      }
      // Аргумент [tr] >> MQTT топик для основного отчета
      if (ArgName.equals("tr") and !ArgValue.isEmpty()) {                       // проверяем на то, что в поле есть актуальное значение
        if (ArgValue.endsWith("/")) ArgValue.remove(ArgValue.length()-1,1);     // если есть обратная косая черта - удаляем
        strcpy(curConfig.report_topic, ArgValue.c_str());                       // присваиваем значение переменной 
        #ifdef DEBUG_LEVEL_PORT       // вывод в порт при отладке кода 
        Serial.printf("Argument [%s] >> curConfig.report_topic = [%s]\n",ArgName, curConfig.report_topic);
        #endif  
      }
      // Аргумент [tl] >> MQTT топик для LWT
      if (ArgName.equals("tl") and !ArgValue.isEmpty()) {                       // проверяем на то, что в поле есть актуальное значение     
        if (ArgValue.endsWith("/")) ArgValue.remove(ArgValue.length()-1,1);     // если есть обратная косая черта - удаляем
        strcpy(curConfig.lwt_topic, ArgValue.c_str());                          // присваиваем значение переменной 
        #ifdef DEBUG_LEVEL_PORT       // вывод в порт при отладке кода 
        Serial.printf("Argument [%s] >> curConfig.lwt_topic = [%s]\n",ArgName, curConfig.lwt_topic);
        #endif  
      }
    }  
    CheckAndUpdateEEPROM();                                     // проверяем конфигурацию и в случае необходимости - записываем новую
    f_ApplayChanges = true;                                     // взводим флаг изменений для правильного вывода сообщения на странице перезагрузки
  }
  #ifdef DEBUG_LEVEL_PORT                                       // вывод в порт при отладке кода 
  Serial.println("WEB <<< Get and applay changes...");    
  #endif  
  handleRebootPage();                                           // отражаем страницу перезагрузки и перегружаем устройство
}

void handleCheckAlivePage() { // процедура проверки статуса контроллера и возврат данных на страницу ожидания (reboot и applay)
  #ifdef DEBUG_LEVEL_PORT       // вывод в порт при отладке кода 
  Serial.println("WEB >>> Send alive status");
  #endif
  WEB_Server.send(200, "text/plane", "alive");
}

void handleGetDataPage() { // получения данных счётчика через WEB
  // передать данные о счётчике номер которого указан в строке запроса
  String ArgName  = "";
  String ArgValue = "";
  String CntrResult = String(curConfig.counter_reboot);
  uint8_t _Int = 0;
  if (WEB_Server.args() > 0) {                                                // если параметры переданы - то занимаемся их обработкой  
    ArgName = WEB_Server.argName(0);                                          // имя первого параметра        
    ArgValue = WEB_Server.arg(0);                                             // значение первого параметра  
    if (ArgName.equals("cntr") and !ArgValue.isEmpty()) {                     // проверяем на то, что в поле есть актуальное значение
      _Int = ArgValue.toInt();
      switch (_Int) {
        case 1:
          CntrResult = String(curConfig.counter_01);
          break;        
        case 2:
          CntrResult = String(curConfig.counter_02);        
          break;        
      } 
    }  
  }
  #ifdef DEBUG_LEVEL_PORT       // вывод в порт при отладке кода 
  Serial.printf("WEB >>> Get by name [%s=%s] value = [%s]\n",ArgName,ArgValue,CntrResult);
  #endif
  WEB_Server.send(200, "text/plane", CntrResult);
}

void handleSetDataPage() { // установить значение счётчика через WEB
  // установить значение счётчика номер которого указан в строке запроса 
  String ArgName  = "";
  String ArgValue = "";
  uint8_t _CntrNum = 0;
  uint32_t _CntrValue = 0;
  String ResultValue = "";
  if (WEB_Server.args() > 1) {                                                // если параметры переданы - то занимаемся их обработкой  
    ArgName = WEB_Server.argName(0);                                          // имя первого параметра - "cntr"
    ArgValue = WEB_Server.arg(0);                                             // значение первого параметра - это номер счётчика        
    ArgValue.trim();                                                          // чистим от пробелов
    if (ArgName.equals("cntr") and isNumeric(ArgValue,true)) {                // проверяем на то, что в поле есть актуальное значение
      _CntrNum = ArgValue.toInt();
      ArgName = WEB_Server.argName(1);                                        // второй параметр это должно быть - "value"
      ArgValue = WEB_Server.arg(1);                                           // а это значение, которое нужно присвоить
      ArgValue.trim();           
      if (ArgName.equals("value") and isNumeric(ArgValue,true)) {             // если имя аргумента совпало и значение его - число, то 
        _CntrValue = ArgValue.toInt();                                        // собственно запоминаем нужное значение
        // и присваиваем его нужному счётчику
        switch (_CntrNum) {
          case 0:
            cmdSetCounterValue(CN_REBOOT, _CntrValue);            
            break;        
          case 1:
            cmdSetCounterValue(CN_CNT01, _CntrValue);
            break;        
          case 2:
            cmdSetCounterValue(CN_CNT02, _CntrValue);            
            break;        
          default:
          ResultValue = "Error !!! Can't assign value ["+ArgValue+"] to counter ["+String(_CntrNum)+"].";            
        } 
        // если результирующая строка пуста - присвоение прошло успешно
        if (ResultValue.isEmpty()) {
          ResultValue = String(_CntrValue);
          CheckAndUpdateEEPROM();                                     // проверяем конфигурацию и записываем новые значения
          #ifdef DEBUG_LEVEL_PORT       // вывод в порт при отладке кода 
          Serial.printf("Set counter [%u] = [%u]\n",_CntrNum, _CntrValue);
          #endif  
        }
      }  
    }
  }
  #ifdef DEBUG_LEVEL_PORT       // вывод в порт при отладке кода 
  Serial.printf("WEB <<< Set counter. Result = [%s] \n",ResultValue);
  #endif
  WEB_Server.send(200, "text/plane", ResultValue);
}

// -------------------------- описание call-back функции MQTT клиента ------------------------------------

void onMqttConnect(bool sessionPresent) { // обработчик подключения к MQTT
  #ifdef DEBUG_LEVEL_PORT                                    
    Serial.println("Connected to MQTT.");  //  "Подключились по MQTT."
  #endif                
  // далее подписываем ESP32 на набор необходимых для управления топиков:
  uint16_t packetIdSub = mqttClient.subscribe(curConfig.command_topic, 0);  // подписываем ESP32 на топик SET_TOPIC
  #ifdef DEBUG_LEVEL_PORT                                      
    Serial.printf("Subscribing at QoS 0, packetId: %d on topic :[%s]\n", packetIdSub, curConfig.command_topic);
  #endif                  
  // сразу публикуем событие о своей активности
  mqttClient.publish(curConfig.lwt_topic, 0, true, jv_ONLINE);           // публикуем в топик LWT_TOPIC событие о своей жизнеспособности
  #ifdef DEBUG_LEVEL_PORT                                      
    Serial.printf("Publishing LWT state in [%s]. QoS 0. ", curConfig.lwt_topic); 
  #endif                     
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) { // обработчик отключения от MQTT
  #ifdef DEBUG_LEVEL_PORT                                                                           
    Serial.println("Disconnected from MQTT.");                      // если отключились от MQTT
  #endif         
  s_CurrentWIFIMode = WF_UNKNOWN;                                   // переходим в режим полного реконнекта по WiFi
}

void onMqttSubscribe(uint16_t packetId, uint8_t qos) { // обработка подтверждения подписки на топик
  #ifdef DEBUG_LEVEL_PORT   
    Serial.printf("Subscribe acknowledged. \n  packetId: %d\n  qos: %d\n", packetId, qos);  
  #endif         
}

void onMqttUnsubscribe(uint16_t packetId) { // обработка подтверждения отписки от топика
  #ifdef DEBUG_LEVEL_PORT     
    Serial.printf("Unsubscribe acknowledged.\n  packetId: %d\n", packetId); 
  #endif                     
}

void onMqttPublish(uint16_t packetId) { // обработка подтверждения публикации
  #ifdef DEBUG_LEVEL_PORT     
    Serial.printf("Publish acknowledged.\n  packetId: %d\n", packetId);   
  #endif                     
}

void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) { // в этой функции обрабатываем события получения данных в управляющем топике SET_TOPIC
  String messageTemp;
  
  for (int i = 0; i < len; i++) {                       // преобразуем полученные в сообщении данные в строку при этом выкидываем символы кавычек
      messageTemp += (char)payload[i];
  }
  messageTemp[len] = '\0';  
  // проверяем, что мы получили MQTT сообщение в командном топике
  if (strcmp(topic, curConfig.command_topic) == 0) {
    // разбираем MQTT сообщение и подготавливаем буфер с изменениями для формирования команд    
    // для этого получаем семафор обработки входного JSON документа
    while ( xSemaphoreTake(sem_InputJSONdoc,(TickType_t) 10) != pdTRUE ) vTaskDelay(1/portTICK_PERIOD_MS);         
    DeserializationError err = deserializeJson(InputJSONdoc, messageTemp);                    // десерилизуем сообщение и взводим признак готовности к обработке
    if (err) {
      #ifdef DEBUG_LEVEL_PORT         
      Serial.print(F("Error of deserializeJson(): "));
      Serial.println(err.c_str());
      #endif
      f_HasMQTTCommand = false;                                                              // сбрасываем флаг получения команды по MQTT
      // далее проверяем, если это короткие сообщения - то сами достраиваем объект документ
      if (strstr(payload,jc_REPORT) != NULL ) {
        InputJSONdoc[jc_REPORT] = true;
        f_HasMQTTCommand = true;                                                              
      }  
      if (strstr(payload,jc_REBOOT) != NULL ) {
        InputJSONdoc[jc_REBOOT] = true;
        f_HasMQTTCommand = true;                                                              
      }  
      if (strstr(payload,jc_RESET) != NULL ) {
        InputJSONdoc[jc_REBOOT] = true;
        f_HasMQTTCommand = true;                                                              
      }  
      if (!f_HasMQTTCommand) xSemaphoreGive ( sem_InputJSONdoc );                            // отдаем семафор, если MQTT обработки не будет
      }
    else f_HasMQTTCommand = true;                                                            // взводим флаг получения команды по MQTT
    // отдадим семафор обработки документа только после преобразования JSON документа в команды
  }
  #ifdef DEBUG_LEVEL_PORT         
  Serial.printf("Publish received.\n  topic: %s\n  message: [", topic);
  Serial.print(messageTemp); Serial.println("]");
  #endif
}

// ========================= коммуникационные задачи времени выполнения ==================================

void webServerTask(void *pvParam) { // задача по обслуживанию WEB сервера модуля
// присваиваем ресурсы (страницы) нашему WEB серверу - страницы объявлены заранее и являются статическими
  WEB_Server.on("/", handleRootPage);		                              // корневая страница с данными счётчиков
  WEB_Server.on("/config", handleConfigPage);		                      // страница изменения конфигурации
  WEB_Server.on("/applay",handleApplayPage);                          // страница, применения изменений - на котрую передаются данные для новой конфигурации
  WEB_Server.on("/reboot",handleRebootPage);                          // страница автоматической перезагрузки контроллера 
  WEB_Server.on("/alive",handleCheckAlivePage);                       // страница для проверки стстуса контроллера и перенаправления на основную страницу
  WEB_Server.on("/get_data",handleGetDataPage);                       // передать данные о счётчике номер которого указан в строке запроса
  WEB_Server.on("/set_data",handleSetDataPage);                       // установить значение счётчика номер которого указан в строке запроса  
  WEB_Server.onNotFound(handleNotFoundPage);		                      // страница с 404-й ошибкой   

  bool _FirstTime = true;
  while (true) {
    if (f_WEB_Server_Enable) {  // если разрешена работа WEB сервера
    // если мы отдали страницу клиенту, и timeout по ее обработке не наступил, то f_Has_WEB_Server_Connect = true; 
      if (_FirstTime ) {
          WEB_Server.begin();                                               // регистрируем сервер 
          _FirstTime = false;
      }    
      WEB_Server.handleClient();
      } 
    else {
      _FirstTime = true;
      WEB_Server.close();  
    }
    vTaskDelay(1/portTICK_PERIOD_MS);                              // делаем задержку в чтении следующего цикла
  }
}

void wifiTask(void *pvParam) { // задача установления и поддержания WiFi соединения
  uint32_t  StartWiFiCycle = 0;                                     // стартовый момент цикла в обработчике WiFi
  uint32_t  StartMQTTCycle = 0;                                     // стартовый момент цикла подключения к MQTT
  uint8_t   APClientCount   = 0;                                    // количество подключенных клиентов в режиме AP
  WiFi.hostname(ControllerName);
  s_CurrentWIFIMode = WF_UNKNOWN;
  while (true) {    
    switch (s_CurrentWIFIMode) {
    case WF_UNKNOWN:
      // начальное подключение WiFi - сброс всех соединений и новый цикл их поднятия 
      f_WEB_Server_Enable = false;                                  // WEB сервер не доступен  
      f_Has_WEB_Server_Connect = false;                             // и коннектов к нему нет    
      count_GetWiFiConfig++;                                        // инкрементируем счётчик попыток 
      mqttClient.disconnect(true);                                  // принудительно отсоединяемся от MQTT       
      WiFi.persistent(false);
      WiFi.mode(WIFI_STA);
      WiFi.disconnect();
      #ifdef DEBUG_LEVEL_PORT       // вывод в порт при отладке кода
      Serial.printf("Try to connect WiFi: %s[",curConfig.wifi_ssid);
      #endif
      StartWiFiCycle = millis();
      // изначально пытаемся подключится в качестве клиента к существующей сети с грантами из конфигурации
      WiFi.begin(curConfig.wifi_ssid,curConfig.wifi_pwd);
      while ((! WiFi.isConnected()) && (millis() - StartWiFiCycle < C_WIFI_CONNECT_TIMEOUT)) { // ожидаем соединения с необходимой WiFi сеткой
        vTaskDelay(pdMS_TO_TICKS(1000)); // рисуем точку каждую секунду
        #ifdef DEBUG_LEVEL_PORT       // вывод в порт при отладке кода
        Serial.print(".");
        #endif
      } 
      #ifdef DEBUG_LEVEL_PORT       // вывод в порт при отладке кода
      Serial.println("]");
      #endif    
      // цикл окончен, проверяем соеденились или нет
      if (WiFi.isConnected()) {
          s_CurrentWIFIMode = WF_CLIENT;                          // если да - мы соеденились в режиме клиента
          f_WEB_Server_Enable = true;                             // WEB сервер становится доступен      
        } 
      else s_CurrentWIFIMode = WF_AP;                             // соеденится как клиент не смогли - нужно поднимать точку доступа
      #ifdef DEBUG_LEVEL_PORT       // вывод в порт при отладке кода 
      if (WiFi.isConnected()) {
          Serial.print("Connected with IP: ");
          Serial.println(WiFi.localIP());
        }
        else Serial.println("Fail.");
      #endif    
      break;
    case WF_OFF:   
      // WiFi принудительно выключен при получении ошибок при работе с WIFI 
      if (count_GetWiFiConfig == C_MAX_WIFI_FAILED_TRYS) {          // если превышено количество попыток соединения (делаем это действие 1 раз)
           f_WEB_Server_Enable = false;                             // WEB сервер не доступен            
           mqttClient.disconnect(true);                             // принудительно отсоединяемся от MQTT 
           WiFi.persistent(false);                                  // принудительно отсоединяемся от WiFi 
           WiFi.disconnect();
           count_GetWiFiConfig++;                                   // это для того, чтобы код условия выполнился один раз
           #ifdef DEBUG_LEVEL_PORT                                            
           Serial.println("!!! WiFi module is OFF !!!");
           #endif
        }   
      vTaskDelay(pdMS_TO_TICKS(C_WIFI_CYCLE_WAIT));                 // ждем цикл перед еще одной проверкой           
      break;    
    case WF_CLIENT:
      // включение WIFI в режиме клиента 
      #ifdef DEBUG_LEVEL_PORT       // вывод в порт при отладке кода
      Serial.printf("Try to connect MQTT: %s \n", curConfig.mqtt_host_s); 
      #endif     
      s_CurrentWIFIMode = WF_MQTT;
      break;    
    case WF_MQTT:
      // соединение с MQTT сервером
      StartMQTTCycle = millis();
      // пытаемся подключится к MQTT серверу в качестве клиента
      mqttClient.connect();
      while ((!mqttClient.connected()) && (millis()-StartMQTTCycle < C_MQTT_CONNECT_TIMEOUT)) { // ожидаем соединения с MQTT сервером
        vTaskDelay(pdMS_TO_TICKS(500)); 
      } 
      // цикл окончен, проверяем есть ли соединение с MQTT
      if (mqttClient.connected()) {                                   // если да - то переходим в режим нормальной работы
          s_CurrentWIFIMode = WF_IN_WORK;  
          f_Has_Report = true;                                        // рапортуем в MQTT текущим состоянием
          count_GetMQTTConfig = 0;                                    // обнуляем количество попыток неуспешного доступа к MQTT
        }  
        else {
          // код дальше заставляет сделать C_MAX_MQTT_FAILED_TRYS попыток соеденится с MQTT. При этом модуль доступен по адресу в указанной WiFi сети как WEB сервер, и можно 
          // поменять конфигурацию на его странице. Если это не получается - уходим в работу без MQTT.
          if (!f_Has_WEB_Server_Connect) count_GetMQTTConfig++;  
          #ifdef DEBUG_LEVEL_PORT       // вывод в порт при отладке кода
          Serial.printf("count_GetMQTTConfig = %u of %u \n",count_GetMQTTConfig,C_MAX_MQTT_FAILED_TRYS);
          #endif    
          if (count_GetMQTTConfig==C_MAX_MQTT_FAILED_TRYS) s_CurrentWIFIMode = WF_WITHOUT_MQTT;        // если есть проблема c ответом MQTT - переходим в работу без него
            else s_CurrentWIFIMode = WF_CLIENT;                       // иначе - уходим на еще один цикл подключения к MQTT
                                                                      // либо - нужно менять конфигурацию, либо ожидать поднятия MQTT сервера                                                                      
          #ifdef DEBUG_LEVEL_PORT       // вывод в порт при отладке кода
          Serial.println("MQTT connection timeout..."); 
          #endif    
        }  
      break;    
    case WF_IN_WORK:  // состояние в котором ничего не делаем, так как все нужные соединения установлены
      count_GetWiFiConfig = 0;                                           // при успешном соединении сбрасываем счётчик попыток повтора 
      if (!mqttClient.connected() or !WiFi.isConnected()) {              // проверяем, что соединения всё еще есть. Если они пропали, делаем таймаут на цикл C_WIFI_CYCLE_WAIT и переустанавливаем соединение
        #ifdef DEBUG_LEVEL_PORT       // вывод в порт при отладке кода
        if (!mqttClient.connected()) Serial.println("MQTT conneсtion lost.");
        if (!WiFi.isConnected()) Serial.println("WiFi conneсtion lost.");
        #endif
        vTaskDelay(pdMS_TO_TICKS(C_WIFI_CYCLE_WAIT)); 
        s_CurrentWIFIMode = WF_UNKNOWN;                                  // уходим на пересоединение с WIFI
      }
      break;    
    case WF_WITHOUT_MQTT:  // состояние в котором ничего не делаем, MQTT отсутсвует, и работаем без него
      count_GetWiFiConfig = 0;                                           // при успешном соединении сбрасываем счётчик попыток повтора 
      if (!WiFi.isConnected()) {                                         // проверяем, что соединение c WiFi еще есть. Если нет, делаем таймаут на цикл C_WIFI_CYCLE_WAIT и переустанавливаем соединение
        #ifdef DEBUG_LEVEL_PORT       // вывод в порт при отладке кода
        if (!WiFi.isConnected()) Serial.println("WiFi conneсtion lost.");
        #endif
        vTaskDelay(pdMS_TO_TICKS(C_WIFI_CYCLE_WAIT)); 
        s_CurrentWIFIMode = WF_UNKNOWN;                                  // уходим на пересоединение с WIFI
      }
      break;              
    case WF_AP:
      // поднятие собственной точки доступа с доступом к странице настройки
      count_GetMQTTConfig = 0;                              // обнуляем количество попыток неуспешного доступа к MQTT
      WiFi.persistent(false);
      WiFi.mode(WIFI_AP);
      WiFi.disconnect();      
      #ifdef DEBUG_LEVEL_PORT       // вывод в порт при отладке кода 
      Serial.printf("Create AP with SSID: %s\n", ControllerName);
      #endif    
      if (WiFi.softAP(ControllerName,"",DEF_WIFI_CHANNEL)) {     // собственно создаем точку доступа на дефолтном канале 
        #ifdef DEBUG_LEVEL_PORT       // вывод в порт при отладке кода 
        Serial.print("AP created with IP: ");
        Serial.println(WiFi.softAPIP());
        #endif 
        StartWiFiCycle = millis();                          // даем отсечку по времени для поднятия точки доступа        
        APClientCount = 0;
        f_Has_WEB_Server_Connect = false;                   // сбрасываем флаг коннектов к WEB серверу
        // если точку доступа удалось поднять, то даем ей работать до тех пор пока не кончился таймаут C_WIFI_AP_WAIT, или есть коннекты к точке доступа       
        while ((WiFi.softAPgetStationNum()>0) or ((millis()-StartWiFiCycle < C_WIFI_AP_WAIT))) {
          // В цикле только выводим количество подключенных к AP клиентов. Основная работа по обслуживанию запросов идет по ой цикл пуст, так как 
          f_WEB_Server_Enable = true;                       // поднимаем флаг доступности WEB сервера
          if (APClientCount!=WiFi.softAPgetStationNum()) {
            APClientCount = WiFi.softAPgetStationNum();
            #ifdef DEBUG_LEVEL_PORT       // вывод в порт при отладке кода 
            Serial.printf("К точке доступа [%s] подключено: %d клиентов \n", ControllerName,APClientCount);            
            #endif 
          }
          vTaskDelay(pdMS_TO_TICKS(500));                   // отдаем управление и ждем 0.5 секунды перед следующей проверкой
        }
      }
      if (count_GetWiFiConfig == C_MAX_WIFI_FAILED_TRYS) s_CurrentWIFIMode = WF_OFF;       // если достигнуто количество попыток соединения для получения конфигурации по WIFi - выключаем WIFI
        else s_CurrentWIFIMode = WF_UNKNOWN;                                          // если нет - переключаемся в режим попытки установления связи с роутером
      break; 
    }
    // запоминаем точку конца цикла
    StartWiFiCycle = millis();
    vTaskDelay(1/portTICK_PERIOD_MS); 
  }  
}

// ====================== обработчики прерываний для счётчиков и сенсора питания =========================

void IRAM_ATTR ISR_handler_counter01() { // описание обработчика прерывания для счётчика 01
// срабатывание происходит при переходе с высокого на низкий уровень (замыкание внешнего контакта)
  if (!f_FireInp01 and !f_FireCutOff) {   // если это новое срабатывание до снятия флага обработки и это не момент снятия питания
    tm_LastFireInp01 = millis();          // запоминаем время сработки 
    f_FireInp01 = true;                   // взводим флаг необходимости обработки
  }
}

void IRAM_ATTR ISR_handler_counter02() { // описание обработчика прерывания для счётчика 02
// срабатывание происходит при переходе с высокого на низкий уровень (замыкание внешнего контакта)
  if (!f_FireInp02 and !f_FireCutOff) {   // если это новое срабатывание до снятия флага обработки и это не момент снятия питания
    tm_LastFireInp02 = millis();          // запоминаем время сработки 
    f_FireInp02 = true;                   // взводим флаг необходимости обработки
  }
 }

void IRAM_ATTR ISR_handler_cutoff_sensor() { // описание обработчика прерывания для датчика пропадания питания
  // срабатывание происходит при переходе с низкого на высокий уровень
  f_FireCutOff = true;
}

// ================================== основные задачи времени выполнения =================================

void countingTask(void *pvParam) { // задача основной обработки по подсчёту импульсов с подавлением дребезга и сохранением данных при потере питания
  while (true) {
    // обработка входа 01
    if (f_FireInp01 and ((millis()-C_COUNTER_DELAY)>tm_LastFireInp01)) { // обработка импульса по входу 01 с подавлением дребезга выше 20Гц
      if (!digitalRead(PIN_INP_CH1)) {                                   // если сигнал на входе в низком уровне - фиксируем импульс
        curConfig.counter_01 ++;
        curConfig.simple_crc16 = GetCrc16Simple((uint8_t*)&curConfig, sizeof(curConfig)-4);      // считаем CRC
      }
      f_FireInp01 = false;
    }
    // обработка входа 02    
    if (f_FireInp02 and ((millis()-C_COUNTER_DELAY)>tm_LastFireInp02)) { // обработка импульса по входу 02 с подавлением дребезга выше 20Гц
      if (!digitalRead(PIN_INP_CH2)) {                                   // если сигнал на входе в низком уровне - фиксируем импульс
        curConfig.counter_02 ++;
        curConfig.simple_crc16 = GetCrc16Simple((uint8_t*)&curConfig, sizeof(curConfig)-4);      // считаем CRC
      }
      f_FireInp02 = false;
    }
    // обработка сигнала пропадания питания Cut-Off
    if (f_FireCutOff) {  
      #ifdef DEBUG_LEVEL_PORT
      Serial.println("Power cut-off detected. Save values."); 
      #endif   
      if (s_EnableEEPROM) {                                                                    // если EEPROM разрешен - просто его записываем
        EEPROM.put(0,curConfig);                                                               // пишем EEPROM
        EEPROM.commit();                                                                       // коммитим изменения 
      }    
      #ifdef DEBUG_LEVEL_PORT
      Serial.println("Save & commit complete."); 
      #endif 
 	    // публикуем событие о том, что мы померли
      if (mqttClient.connected()) {
        mqttClient.publish(curConfig.lwt_topic, 0, true, jv_OFFLINE);                          // публикуем в топик LWT_TOPIC событие о своей смерти
        #ifdef DEBUG_LEVEL_PORT                                      
        Serial.printf("Publishing LWT offline state in [%s]. QoS 0. \n", curConfig.lwt_topic); 
        #endif                     
      }  
      vTaskDelay(C_REPORT_DELAY);                                                             // вгоняем чип в задержку до конца питания      
      ESP.restart();                                                                          // и если мы еще живы, когда дошли до этого места - перезагружаемся (защита от дребезга по 220v -
                                                                                              // возможен вариант потери полупериода-периода питания, датчик сработает, а питание восстановится)
    }
    vTaskDelay(1/portTICK_PERIOD_MS);     
  }
}

void eventHandlerTask (void *pvParam) { // задача обработки событий получения команды от датчика, таймера, MQTT, OneWire, кнопок
  uint8_t         new_light_mode = 0;                       // новый режим работы подсветки индикатора    
  while (true) {
    //-------------------- обработка команд задержки и таймера ---------------------------------------
    // обработка флага мигания
    if (millis()-tm_LastBlink > C_BLINKER_DELAY) {  // переключаем блинкер по задержке 
      f_Blinker = !f_Blinker;
      tm_LastBlink = millis();
    } 
    //-------------------- обработка событий получения MQTT команд в приложение ----------------------
    if ( f_HasMQTTCommand ) {                                         // превращаем события MQTT в команды для отработки приложением    
      // защищаем секцию работы с Static JSON DOC с помощью мьютекса
      // MQTT: report
      if (InputJSONdoc.containsKey(jc_REPORT) and InputJSONdoc[jc_REPORT])  {   // послана команда принудительного отчета
        f_Has_Report = true;                                         // взводим флаг, что отчёт нужен сейчас
      }
      // MQTT: reboot
      if (InputJSONdoc.containsKey(jc_REBOOT) and InputJSONdoc[jc_REBOOT])  {   // послана команда принудительной перезагрузки
        CheckAndUpdateEEPROM();
        cmdReset();
      }
      // MQTT: сброс конфигурации/значения счётчиков
      if (InputJSONdoc.containsKey(jk_CLEAR))  {   // послана команда сброса
        if (InputJSONdoc[jk_CLEAR] == jv_COUNTER_01) cmdSetCounterValue(CN_CNT01,0);          // команда сброса счётчика 1
        if (InputJSONdoc[jk_CLEAR] == jv_COUNTER_02) cmdSetCounterValue(CN_CNT02,0);          // команда сброса счётчика 2
        if (InputJSONdoc[jk_CLEAR] == jv_COUNTER_RB) cmdSetCounterValue(CN_REBOOT,0);         // команда сброса счётчика перезагрузок
        if (InputJSONdoc[jk_CLEAR] == jv_CONFIG)     cmdClearConfig_Reset();                  // команда сброса конфигурации и обнуления счётчиков
      }
      // MQTT: установка значений счётчиков - 1
      if (InputJSONdoc.containsKey(jk_SET_VALUE_01))  {   // послана команда установки значения счётчика
        uint32_t _cntr_value1 = 0;
        if (InputJSONdoc[jk_SET_VALUE_01].is<int32_t>()) {
          _cntr_value1 = InputJSONdoc[jk_SET_VALUE_01];
          cmdSetCounterValue(CN_CNT01,_cntr_value1);        // устанавливаем значение для счётчика 1
        }  
        else {
          #ifdef DEBUG_LEVEL_PORT                                            
          Serial.printf("!!! Value can't be assigned for counter 01 !!!\n");
          #endif
        }
      }
      // MQTT: установка значений счётчиков - 2
      if (InputJSONdoc.containsKey(jk_SET_VALUE_02))  {   // послана команда установки значения счётчика
        uint32_t _cntr_value2 = 0;
        if (InputJSONdoc[jk_SET_VALUE_02].is<int32_t>()) {
          _cntr_value2 = InputJSONdoc[jk_SET_VALUE_02];
          cmdSetCounterValue(CN_CNT02,_cntr_value2);        // устанавливаем значение для счётчика 1
        }  
        else {
          #ifdef DEBUG_LEVEL_PORT                                            
          Serial.printf("!!! Value can't be assigned for counter 02 !!!\n");
          #endif
        }
      }
      // обработка входного JSON закончена - отпускаем семафор
      f_HasMQTTCommand = false;                                       // сбрасываем флаг наличия изменений через MQTT 
      xSemaphoreGive(sem_InputJSONdoc);                               // отпускаем семафор обработки входного сообщения
    }
    //--------------------- опрос кнопок - получение команд ------------------------
    bttn_clear.tick();                                                // опрашиваем кнопку CLEAR
    bttn_flash.tick();                                                // опрашиваем кнопку FLASH
    // однократное нажатие на кнопку CLEAR - обнуление счётчика №1
    if (bttn_clear.isClick()) {        
        cmdSetCounterValue(CN_CNT01,0);
    }
    // двухкратное нажатие на кнопку CLEAR - обнуление счётчика №2
    if (bttn_clear.isDouble()) {        
        cmdSetCounterValue(CN_CNT02,0);
    }
    // одновременное нажатие и удержание кнопок CLEAR и FLASH - команда сброса конфигурации до заводских параметров и перезагрузка
    if (bttn_clear.isHold() and bttn_flash.isHold()) {        
        cmdClearConfig_Reset();
    }
    // отдаем управление ядру FreeRT OS
    vTaskDelay(1/portTICK_PERIOD_MS); 
  }
}

void applayChangesTask (void *pvParam) { // применяем изменения состояния
// здесь отражаются внутренние изменения, команды выполняются в процедуре обработки команд eventHandlerTask
  while (true) {

  // TODO: включаем/выключаем индикацию
    digitalWrite(LED_RED_PIN,!digitalRead(PIN_INP_CH1));
    digitalWrite(LED_BLUE_PIN,!digitalRead(PIN_INP_CH2));


    vTaskDelay(1/portTICK_PERIOD_MS); 
  }
}

void reportTask (void *pvParam) { // репортим о текущем состоянии в MQTT и если отладка то и в Serial
  while (true) {
    if (((millis()-tm_LastReportToMQTT)>C_REPORT_DELAY) || f_Has_Report) {  // если наступило время отчёта или взведен флаг наличия отчета
      if (mqttClient.connected()) {  // если есть связь с MQTT - репорт в топик
        // ---------------------------------------------------------------------------------
        // рапортуем в главный топик статуса [curConfig.report_topic]
        // чистим документ
        OutputJSONdoc.clear(); 
        // добавляем поля в документ
        OutputJSONdoc[jk_COUNTER_01] = curConfig.counter_01;                                        // значение счётчика 1
        OutputJSONdoc[jk_COUNTER_02] = curConfig.counter_02;                                        // значение счётчика 2
        OutputJSONdoc[jk_COUNTER_RB] = curConfig.counter_reboot;                                    // значение счётчика перезагрузок
        OutputJSONdoc[jk_IP] = WiFi.localIP().toString();                                           // выводим значение IP адреса подключения
        // серилизуем в строку
        String tmpPayload;
        serializeJson(OutputJSONdoc, tmpPayload);
        // публикуем в топик P_STATE_TOPIC серилизованный json через буфер buffer
        char buffer1[ tmpPayload.length()+1 ];
        tmpPayload.toCharArray(buffer1, sizeof(buffer1));   
        mqttClient.publish(curConfig.report_topic, 0, true, buffer1 );
      }
      #ifdef DEBUG_LEVEL_PORT 
        Serial.println();
        Serial.println("<<<< Current state report >>>>");
        Serial.printf("%s : %u\n", jk_COUNTER_01, curConfig.counter_01);
        Serial.printf("%s : %u\n", jk_COUNTER_02, curConfig.counter_02);
        Serial.printf("%s : %u\n", jk_COUNTER_RB, curConfig.counter_reboot);    
        Serial.println("---");            
        Serial.printf("inp1 : %u\n", digitalRead(PIN_INP_CH1));
        Serial.printf("inp2 : %u\n", digitalRead(PIN_INP_CH2));
        Serial.printf("cut-off : %u\n", digitalRead(PIN_INP_AC_CUTOFF));
        Serial.println("<<<< End of current report >>>>");
      #endif                
      tm_LastReportToMQTT = millis();           // взводим интервал отсчёта
      f_Has_Report = false;                     // сбрасываем флаг
    }
    vTaskDelay(1/portTICK_PERIOD_MS);         
  }
}

// =================================== инициализация контроллера и программных модулей ======================================
// начальная инициализация программы - выполняется при подаче дежурного питания.
// дальнейшее включение усилителя - уже в рамках работающей программы
void setup() { // инициализация контроллера и программных модулей
  uint8_t MacAddress[8];                        // временная переменная для MAC адреса текущей ESP 
  String  Mac_Postfix;                          // строка для создания постфикса имени из MAC

  #ifdef DEBUG_LEVEL_PORT                       // вывод в порт при отладке кода
  // инициализируем порт отладки 
  Serial.begin(115200);
  Serial.println();
  #endif

 // инициализация входов и выходов  
  pinMode(LED_RED_PIN, OUTPUT);               // инициализируем pin светодиода канала 1
  pinMode(LED_BLUE_PIN, OUTPUT);              // инициализируем pin светодиода канала 2 
  pinMode(PIN_INP_CH1, INPUT);                // инициализируем вход канала 1
  pinMode(PIN_INP_CH2, INPUT);                // инициализируем вход канала 2
  pinMode(PIN_INP_AC_CUTOFF, INPUT);          // инициализируем вход датчика наличия напряжения

  // инициализация генератора случайных чисел MAC адресом
  // и генерация уникального имени контроллера из его MAC-а 
  if (esp_efuse_mac_get_default(MacAddress) == ESP_OK) {
    Mac_Postfix = String(MacAddress[4], HEX) + String(MacAddress[5], HEX);
    Mac_Postfix.toUpperCase();
    randomSeed(MacAddress[5]);
    }
  else {
    Mac_Postfix = "0000";
    randomSeed(millis());
  }
  // имя нашего контроллера из префикса и постфикса
  ControllerName += Mac_Postfix;

    // включаем индикацию
  digitalWrite(LED_BLUE_PIN, HIGH);
  digitalWrite(LED_RED_PIN, HIGH);
  
  // задержка для контроля индикации 
  vTaskDelay(pdMS_TO_TICKS(500));

  // гасим всю индикацию
  digitalWrite(LED_BLUE_PIN, LOW);
  digitalWrite(LED_RED_PIN, LOW);
  

  // инициализируем кнопку CLEAR
  bttn_clear.setDebounce(50);        // настройка антидребезга (по умолчанию 80 мс)  
  bttn_clear.setTimeout(500);        // настройка таймаута на удержание (по умолчанию 500 мс)
  bttn_clear.setClickTimeout(200);   // настройка таймаута между кликами (по умолчанию 300 мс)
  
  // инициализируем кнопку FLASH
  bttn_flash.setDebounce(50);        // настройка антидребезга (по умолчанию 80 мс)  
  bttn_flash.setTimeout(500);        // настройка таймаута на удержание (по умолчанию 500 мс)
  bttn_flash.setClickTimeout(200);   // настройка таймаута между кликами (по умолчанию 300 мс)

  // инициализируем блок конфигурации значениями по умолчанию
  SetConfigByDefault();

  // инициализация работы с EEPROM
  s_EnableEEPROM = EEPROM.begin(sizeof(curConfig));   // инициализируем работу с EEPROM 

  #ifdef DEBUG_LEVEL_PORT    
  if (s_EnableEEPROM) {  // если инициализация успешна - то:   
    if (ReadEEPROMConfig()) { // читаем конфигурацию из EEPROM и проверяем контрольную сумму для блока данных
      // контрольная сумма блока данных верна - используем его в качестве конфигурации      
      Serial.println("EEPROM config is valid. Applay config.");
      }
    else { // контрольная сумма данных не сошлась - считаем блок испорченным, инициализируем и перезаписываем его
      Serial.println("EEPROM config broken - try to rewrite.");    
      SetConfigByDefault();
      EEPROM.put(0,curConfig);     
      if (EEPROM.commit()) Serial.println("EPPROM update success.");
        else Serial.println("Error in write to EPPROM.");
    }
    Serial.printf("\n--- Инициализация блока управления прошла со следующими параметрами: ---\n");
    Serial.printf("  WiFi SSid: %s\n", curConfig.wifi_ssid);    
    Serial.printf("  WiFi pwd: %s\n", curConfig.wifi_pwd);
    Serial.printf("  MQTT usr: %s\n", curConfig.mqtt_usr);
    Serial.printf("  MQTT pwd: %s\n", curConfig.mqtt_pwd);
    Serial.printf("  MQTT host: %s\n", curConfig.mqtt_host_s); 
    Serial.printf("  MQTT port: %d\n", curConfig.mqtt_port);
    Serial.println("---");    
    Serial.printf("  COMMAND topic: %s\n", curConfig.command_topic);
    Serial.printf("  REPORT topic: %s\n", curConfig.report_topic);
    Serial.printf("  LWT topic: %s\n", curConfig.lwt_topic);
    Serial.println("---");    
    Serial.printf("  Counter 01: %u\n", curConfig.counter_01);
    Serial.printf("  Counter 02: %u\n", curConfig.counter_02);
    Serial.printf("  Reboot counter: %u\n", curConfig.counter_reboot);
    Serial.printf("---\n\n");
    }
  else { Serial.println("Warning! Блок работает без сохранения конфигурации !!!"); 
  } 
  #else
  if (s_EnableEEPROM) {  // если инициализация успешна - то:   
    if (!ReadEEPROMConfig()) { // читаем конфигурацию из EEPROM и проверяем контрольную сумму для блока данных
      // если контрольная сумма данных не сошлась - считаем блок испорченным, инициализируем и перезаписываем его
      SetConfigByDefault();
      EEPROM.put(0,curConfig);     
      EEPROM.commit();
    }
  }  
  #endif  

  // увеличиваем счетчик перезагрузок 
  curConfig.counter_reboot++;
  curConfig.simple_crc16 = GetCrc16Simple((uint8_t*)&curConfig, sizeof(curConfig)-4);   // и сразу пересчитываем CRC

  // настраиваем MQTT клиента
  mqttClient.setCredentials(curConfig.mqtt_usr,curConfig.mqtt_pwd);
  mqttClient.setServer(curConfig.mqtt_host_s, curConfig.mqtt_port);
  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onSubscribe(onMqttSubscribe);
  mqttClient.onUnsubscribe(onMqttUnsubscribe);
  mqttClient.onMessage(onMqttMessage);
  mqttClient.onPublish(onMqttPublish);

  // настраиваем семафоры - сбрасываем их
  xSemaphoreGive(sem_InputJSONdoc);
  xSemaphoreGiveFromISR(sem_CurConfigWrite,NULL);
  
  // создаем отдельные параллельные задачи, выполняющие группы функций  
  // стартуем основные задачи
  if (xTaskCreate(eventHandlerTask, "events", 4096, NULL, 1, NULL) != pdPASS) Halt("Error: Event handler task not created!");     // все плохо, задачу не создали
  if (xTaskCreate(applayChangesTask, "applay", 4096, NULL, 1, NULL) != pdPASS) Halt("Error: Applay changes task not created!");   // все плохо, задачу не создали
  if (xTaskCreate(reportTask, "report", 4096, NULL, 1, NULL) != pdPASS) Halt("Error: Report task not created!");                  // все плохо, задачу не создали
  if (xTaskCreate(countingTask, "count", 4096, NULL, 1, NULL) != pdPASS) Halt("Error: Report task not created!");                 // все плохо, задачу не создали
  // стартуем коммуникационные задачи
  if (xTaskCreate(wifiTask, "wifi", 4096*2, NULL, 1, NULL) != pdPASS) Halt("Error: WiFi communication task not created!");        // все плохо, задачу не создали
  if (xTaskCreate(webServerTask, "web", 4096*2, NULL, 1, NULL) != pdPASS) Halt("Error: Web server task not created!");            // все плохо, задачу не создали

}

void loop() { // не используемый основной цикл
  // присваиваем обработчики прерываний на счётчики и датчик питания
  attachInterrupt(PIN_INP_CH1,&ISR_handler_counter01,FALLING);			      // назначаем прерывание на GPIO входа #1 по ниспадающему фронту
  attachInterrupt(PIN_INP_CH2,&ISR_handler_counter02,FALLING);			      // назначаем прерывание на GPIO входа #2 по ниспадающему фронту
  attachInterrupt(PIN_INP_AC_CUTOFF,&ISR_handler_cutoff_sensor,RISING);		// назначаем прерывание на GPIO датчика пропажи питания по восходящему фронту
  vTaskDelete(NULL);   // удаляем не нужную задачу loop()  
}