# ESP32_drone

##Connections
| Led | Pin |
| --- | --- |
| Yellow  | 19  |
| Red  | 23  |
| Green  | 18  |

##Features
* 3 tasks created with the same function
* Struct passed as pvParameter

``C
struct led {
	uint8_t pinNumber;
	uint16_t tickPeriod;
} led;
``

##Status
**OK**
