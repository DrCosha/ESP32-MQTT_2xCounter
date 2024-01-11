# ESP32-MQTT_2xCounter
Firware for ESP32s module with AC power-in sensor.  Module count  incoming pulses in 2 counters and storing this values for external access. For access to counters value you can use MQTT and HTTP. Initial settings and counters start value set via built-in WEB server. Module have AC power-in sensor and save data in FLASH memory if his cut off. 

### Модуль счётчиков внешних импульсов на ESP32s с датчиком отключения питания.

<br> Данный модуль может использоватся для подсчёта внешних импульсов от счётчиков тепла/электричества/воды. Подсчёт ведется путем замыкания входных клемм __[INP1]__ и __[INP2]__. Замыкание соответсвующего вывода индицируется светодиодом __LED1__ и __LED2__.
Переход от ***не замкнутого*** положения к ***замкнутому*** добавляет один импульс в значению соответсвующего счётчика. </br> 
Модуль имеет детектор пропадания питания, при этом энергии в накопительных конденсаторах хватает для записи значения счётчиков в энергонезависимую FLASH память . Это сохраняет значения и минимизирует потери импульсов при перезагрузке устройства по питанию.
При этом есть есть отдельный счётчик, который ведет подсчёт количества пропаданий питания со времени своего последнего сброса.

При старте устройства текущие значения счётчиков вычитываются их FLASH памяти. Значения счётчиков могут быть сброшены принудительно, нажатием на кнопку CLEAR на плате. Однократное нажатие сбрасывает счётчик 1, двойное - счётчик 2.
Для полного сброса всех настроек модуля к базовым, необходимо более чем на 1 секунду нажать одновременно кнопки CLEAR и FLASH.

По умолчанию, модуль подключается к WiFi сети с настройками прошитыми в FLASH памяти. Там же хранятся настройки подключения и описания топиков MQTT сервера, через которые можно получить доступ к значениям счётчиков.


### Коммуникация с модулем

Если при старте модуля он не смог установить соединение с WiFi сетью вашего роутера, то будет поднята собственная точка доступа с именем CNTR_xxxx, где хххх - последние цифры MAC адреса ESP32s. После поднятия точки доступа, можно соединится со страницей настроек модуля,
которая будет доступна по адресу default gateway точки доступа. Эти же настройки доступны и при успешном подключении модуля к WiFi сети. Адрес при этом будет получен динамически от вашего маршрутизатора. Поддерживается только сеть 2.4МГц (это ограничение самого ESP32).
Общий вид страницы настроек приведен на изображении ниже. Отдельным пунктом меню можно обнулить оба счётчика по отдельности. 
Встроенный WEB сервер можно так же использовать для получения значения счётчиков в текущий момент времени. 

Для этого нужно обратится по адресу __[адрес_модуля]/get_data?cnt=х__ - где х - номер счётчика, значение которого мы хотим получить.

Доступ к модулю через MQTT возможен при правильной настройке параметров подключения.  При этом это может быть как локальный, так и глобальный MQTT сервер. 
Работа с сервером идет через три топика:
- топик команд **[SET]** ;
- топик состояния подключения устройства **[LWT]**;
- топик рапортов о текущих заначениях **[STATUS]**;

Если модуль получает в топике **[SET]** команду {report}, то сразу публикует в топике **[STATUS]** своё текущее состояние. Иначе, своё текущее состояние модуль публикует каждые 60 минут.


### Команды и статусы

Ниже приведены команды, которые будут исполнены при помещении их в топик **[SET]**:

```
{"report"} 			- команда немедленной генерации отчёта в топик [STATUS]
{"clear":"config"} 		- сброс текущей конфигурации до начальной и перезагрузки
{"clear":"cnt#1"} 		- сброс счётчика №1
{"clear":"cnt#2"} 		- сброс счётчика №2
{"clear":"reboot"}		- сброс счётчика перезагрузок (считает от момента прошлого сброса)
{"set_value_1":<значение>}	- установка значения счётчика №1*
{"set_value_2":<значение>}	- установка значения счётчика №2*

	* допустимое значение счётчика от 0 до 4 294 967 295 (0xFFFFFFFF)
```

Ниже приведен пример отчета в JSON формате, генерируемого модулем в топик **[STATUS]**:

```

{"cnt#1":<значение1>,"cnt#2":<значение2>,"cnt_reboot":<значение3>,"ip":<xx.xx.xx.xx>}  - где:

	- <значение1>, <значение2>	- текущие значения счётчиков №1 и №2;
	- <значение3> 			- значение счётчика перезагрузок;
	- <xx.xx.xx.xx>			- текущий IP модуля для облегчения доступа к его страницам настроек.
```

<br/>
<br/>

# Hardware часть

Общая принципиальная схема модуля представлена ниже.

![Schematic of ESP32-MQTT_2xCounter rev1](https://github.com/DrCosha/ESP32-MQTT_2xCounter/blob/main/images/Common_circuit_v1.png)

