#include <Wire.h>
#include <WireSlave.h>

#include <LovyanGFX.hpp>
#include <LGFX_AUTODETECT.hpp>

#include "config.h"
#include "common.h"

void receiveEvent(int howMany);
void requestEvent();

void I2CTask(void *parameter) {
  bool success = WireSlave.begin(SDA_PIN, SCL_PIN, I2C_SLAVE_ADDR, 200, 100);

  if (!success) {
    lcd.println("I2C slave init failed");
    while (1) delay(100);
  }
  WireSlave.onReceive(receiveEvent);
  WireSlave.onRequest(requestEvent);
  while (true) {
    WireSlave.update();
    delay(1);  // let I2C and other ESP32 peripherals interrupts work
  }
}

void setup(){
  // For display
  lcd.init();
  lcd.setRotation(lcd_rotation);
  // lcd.clear();
  // lcd.setTextSize(1.5);
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);

  Serial.begin(115200);

  lcd.println("Wait for I2C input.");
#ifdef USE_GROVE
  printColorText("\x1b[31mGROVE\x1b[39m Mode\n");
#else
  printColorText("\x1b[31mNOT GROVE\x1b[39m Mode\n");
#endif
  char log_msg[50];
  sprintf(log_msg, "I2C address \x1b[33m0x%02x\x1b[39m", I2C_SLAVE_ADDR);
  printColorText(log_msg);

  lastReceiveTime = millis();

  xTaskCreatePinnedToCore(I2CTask, "I2C Task", 2048, NULL, 24, NULL, 0);
}

void loop() {
  if (millis() - lastReceiveTime > receiveTimeout) {
    lcd.fillScreen(lcd.color565(255, 0, 0));  // Fill the screen with red
    lcd.setCursor(0, 0);
    lcd.println("No data received.");
    delay(500);  // Show message for a short time
  }
}

void receiveEvent(int howMany) {
  lastReceiveTime = millis();  // Update the last received time
  String str;
  while (0 < WireSlave.available()) {
    char c = WireSlave.read();  // receive byte as a character
    str += c;
  }

  // Draw
  lcd.fillScreen(lcd.color565(0, 0, 0));
  lcd.setCursor(0, 0);
  printColorText(str);
}

void requestEvent() {
  WireSlave.write(0);
}
