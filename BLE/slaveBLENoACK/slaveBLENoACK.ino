#include <Wire.h>
#include <Arduino.h>
#include <bluefruit.h>
#include "nrf.h"
#include "nrf_gpio.h"

#define I2C_SLAVE_ADDR   0x08
#define MANUFACTURER_ID  0x1234
#define WAKEUP_PIN      2

static char data[60] = "";
static bool newData = false;
static uint32_t startTime = 0;

void enterDeepSleep() {
   Serial.println("Entering deep sleep...");
   Serial.flush();
   // Configure wake pin
   nrf_gpio_cfg_sense_input(digitalPinToPinName(WAKEUP_PIN),
                         NRF_GPIO_PIN_PULLUP,
                         NRF_GPIO_PIN_SENSE_LOW);
                        
   sd_softdevice_disable();
   NRF_POWER->SYSTEMOFF = 1;
   while(1);
}

void receiveEvent(int howMany) {
   char buffer[64];
   int i = 0;
   while (Wire.available() && i < sizeof(buffer) - 1) {
       buffer[i++] = Wire.read();
   }
   buffer[i] = '\0';
   
   strcpy(data, buffer);
   newData = true;
   startTime = millis(); 
   Serial.print("Got from STM: ");
   Serial.println(data);
}

void advertiseData() {
  Bluefruit.Advertising.stop();
  Bluefruit.Advertising.clearData();
  
  uint32_t elapsed = millis() - startTime;
  

  char dataWithTime[60]; // if you see '_' the number behind is the time in ms
  snprintf(dataWithTime, sizeof(dataWithTime), "%s_%lu", data, elapsed);
  
  uint8_t msd[31] = {0};
  msd[0] = MANUFACTURER_ID & 0xFF;
  msd[1] = (MANUFACTURER_ID >> 8) & 0xFF;
  memcpy(&msd[2], dataWithTime, strlen(dataWithTime));
  
  Bluefruit.Advertising.addData(BLE_GAP_AD_TYPE_MANUFACTURER_SPECIFIC_DATA,
                             msd, strlen(dataWithTime) + 2);
  
  Serial.print("Time from I2C to BLE: ");
  Serial.print(elapsed);
  Serial.println(" ms");
  Serial.print("Sending data: ");
  Serial.println(dataWithTime);
  
  Bluefruit.Advertising.setInterval(32, 32);
  Bluefruit.Advertising.start(0.06);
  delay(10);
}

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

void setup() {
  //disableI2CPullups();
  while(!Serial) {}
   Serial.println("awake!!");
   Serial.begin(115200);
   
   Wire.begin(I2C_SLAVE_ADDR);
   Wire.onReceive(receiveEvent);
   
   Bluefruit.begin();
   Bluefruit.setName("SlaveRoverBLE");
   pinMode(WAKEUP_PIN, INPUT_PULLUP);
}

void loop() {
   if (newData) {
       advertiseData();
       newData = false;
       delay(100);
       enterDeepSleep();
   }
   delay(10);
}