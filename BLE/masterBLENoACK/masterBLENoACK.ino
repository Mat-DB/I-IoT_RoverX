#include <Wire.h>
#include <Arduino.h>
#include <bluefruit.h>
#include "nrf.h"
#include "nrf_gpio.h"
#include <Adafruit_FlashTransport.h>

#define I2C_SLAVE_ADDR   0x08
#define MANUFACTURER_ID  0x1234
#define WAKEUP_PIN      2

Adafruit_FlashTransport_QSPI flashTransport;

static char receivedData[60] = "";
static char lastReceivedData[60] = "";
static bool newData = false;
static uint32_t startTime = 0;

void disableI2CPullups() {
   nrf_gpio_cfg(
       digitalPinToPinName(PIN_WIRE_SDA),
       NRF_GPIO_PIN_DIR_INPUT,
       NRF_GPIO_PIN_INPUT_CONNECT,
       NRF_GPIO_PIN_NOPULL,
       NRF_GPIO_PIN_S0S1,
       NRF_GPIO_PIN_NOSENSE
   );
   
   nrf_gpio_cfg(
       digitalPinToPinName(PIN_WIRE_SCL),
       NRF_GPIO_PIN_DIR_INPUT,
       NRF_GPIO_PIN_INPUT_CONNECT,
       NRF_GPIO_PIN_NOPULL,
       NRF_GPIO_PIN_S0S1,
       NRF_GPIO_PIN_NOSENSE
   );
   
   Serial.println("I2C pull-up resistors disabled");
}

void scan_callback(ble_gap_evt_adv_report_t* report) {
    uint8_t buffer[31];
    int len = Bluefruit.Scanner.parseReportByType(report,
                BLE_GAP_AD_TYPE_MANUFACTURER_SPECIFIC_DATA,
                buffer, sizeof(buffer));
                
    if (len >= 2) {
        uint16_t cid = buffer[0] | (buffer[1] << 8);
        if (cid == MANUFACTURER_ID) {
            int msgLen = len - 2;
            
          
            char tempData[60];
            memcpy(tempData, &buffer[2], msgLen);
            tempData[msgLen] = '\0';
            
            
            if (strcmp(tempData, lastReceivedData) != 0) {
                strcpy(receivedData, tempData);
                strcpy(lastReceivedData, tempData);  
                newData = true;
                startTime = millis();
                
                Serial.print("New data received via BLE: ");
                Serial.println(receivedData);
            }
        }
    }
    Bluefruit.Scanner.resume();
}

void requestEvent() {
   if (newData) {
       uint32_t elapsed = millis() - startTime;
       
       char dataWithBothTimes[60];
       snprintf(dataWithBothTimes, sizeof(dataWithBothTimes), "%s_%lu", receivedData, elapsed);
       
       Wire.write(dataWithBothTimes, strlen(dataWithBothTimes));
       
       Serial.print("Original data with both times: ");
       Serial.println(dataWithBothTimes);
       Serial.print("Our processing time: ");
       Serial.print(elapsed);
       Serial.println(" ms");
       
       newData = false;
       //delay(4000);
       //enterDeepSleep();
   }
}

static void enterDeepSleep() {
   Serial.println("Entering deep sleep...");
   Serial.flush();
   nrf_gpio_cfg_sense_input(digitalPinToPinName(WAKEUP_PIN),
                         NRF_GPIO_PIN_PULLUP,
                         NRF_GPIO_PIN_SENSE_LOW);
                        
   sd_softdevice_disable();
   NRF_POWER->SYSTEMOFF = 1;
   while(1);
}

void setup() {
   Serial.println("also awake?!");
   Serial.begin(115200);
   
   //disableI2CPullups();
   
   Wire.begin(I2C_SLAVE_ADDR);
   Wire.onRequest(requestEvent);
   
   Bluefruit.begin();
   Bluefruit.setName("MasterRoverBLE");
   
   Bluefruit.Scanner.setRxCallback(scan_callback);
   Bluefruit.Scanner.setInterval(32, 32);
   Bluefruit.Scanner.useActiveScan(true);
   Bluefruit.Scanner.start(0.13);
   pinMode(WAKEUP_PIN, INPUT_PULLUP);
}

void loop() {
   delay(1);
}
