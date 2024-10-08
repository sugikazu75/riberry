#include "M5AtomS3.h"
#include "ArduinoHWCDCHardware.h"
#include "ros/node_handle.h"

#include <std_msgs/Float32.h>

// #define LGFX_M5ATOMS3
// #include <LovyanGFX.hpp>
// #include <LGFX_AUTODETECT.hpp>

#include "config.h"
#include "common.h"

#ifndef BatteryDisplay_h
#define BatteryDisplay_h

// voltage unit: V
class BatteryDisplay
{
public:
  BatteryDisplay(float maxVoltage, float minVoltage);

public:
  void setVoltageRange(float newMaxVoltage, float newMinVoltage);
  void displayFrame();
  void updateVoltage(float voltage);

private:
  float maxVoltage_;
  float minVoltage_;
  float Voltage_;
};

#endif

#define LCD_H M5.Lcd.height()
#define LCD_W M5.Lcd.width()


BatteryDisplay::BatteryDisplay(float maxVoltage, float minVoltage)
  :maxVoltage_(maxVoltage), minVoltage_(minVoltage) {}

void BatteryDisplay::setVoltageRange(float newMaxVoltage, float newMinVoltage)
{
  maxVoltage_ = newMaxVoltage;
  minVoltage_ = newMinVoltage;
}

void BatteryDisplay::displayFrame()
{
  // Show title.
  M5.Lcd.fillRect(0, 0, LCD_W, 16, MAROON);
  M5.Lcd.setTextSize(2);
  M5.Lcd.setTextColor(WHITE);
  M5.Lcd.drawString("Voltage", 0, 0, 1);

  // Show units.
  M5.Lcd.drawRect(0, 19, LCD_W, 19, YELLOW);
  M5.Lcd.drawLine(LCD_W/2+12, 19, LCD_W/2+12, 37, YELLOW);
  M5.Lcd.drawString("V", LCD_W/2-1, 22, 1);
  M5.Lcd.drawString("%", LCD_W-12, 22, 1);
}

void BatteryDisplay::updateVoltage(float voltage)
{
  float voltageRatio = (voltage - minVoltage_) / (maxVoltage_ - minVoltage_);
  voltageRatio = constrain(voltageRatio, 0.0f, 1.0f);

  // Erase screen.
  M5.Lcd.fillRect(1, 20, 60, 16, BLACK);
  M5.Lcd.fillRect(LCD_W/2+17, 20, 32, 16, BLACK);

  // Show voltage.
  M5.Lcd.setTextColor(WHITE);
  M5.Lcd.setCursor(2, 21);
  M5.Lcd.printf("%0.2f", voltage);
  M5.Lcd.setCursor(LCD_W/2+15, 21);
  M5.Lcd.print((uint8_t)(voltageRatio*100));

  // Show Meter
  int32_t rect_x = 0;
  int32_t rect_h = 7;
  int32_t rect_w = LCD_W;
  int32_t radius = 3;
  uint8_t barNum = 10;
  for(byte k = 0; k < barNum; k++)
  {
    int32_t rect_y = LCD_H - rect_h - (rect_h + 2) * k;
    uint16_t color = M5.Lcd.color565(16,16,16);
    if(voltageRatio > float(k+1) / barNum)
    {
      color = M5.Lcd.color565(
        (uint8_t)(255 - 255 * (k / float(barNum-1))),
        (uint8_t)(255 * (k / float(barNum-1))), 0);
    }
    M5.Lcd.fillRoundRect(rect_x, rect_y, rect_w, rect_h, radius, color);
  }
}
// void BatteryDisplay::VoltageSub(const std_msgs::Float32& voltage_msg){
//     Voltage_ = voltage_msg.data;
//     updateVoltage(Voltage_);
// }


ros::NodeHandle_<ArduinoHardware> nh;

float battery_voltage_;
void batteryVoltageCallback(const std_msgs::Float32& msg){
  battery_voltage_ = msg.data;
}

ros::Subscriber<std_msgs::Float32> battery_voltage_sub_("battery_voltage_status", &batteryVoltageCallback);

void setup()
{
  // For display
  lcd.init();
  lcd.setRotation(lcd_rotation);
  lcd.clear();
  lcd.setTextSize(1.5);

  lcd.println("waiting for rosserial connection");

  nh.initNode();
  nh.subscribe(battery_voltage_sub_);

  while (!nh.connected()) {
    nh.spinOnce();
    delay(100);
  }
  lcd.println("rosserial init done!");

  delay(1000);
}

void loop()
{
  nh.spinOnce();

  lcd.clear();
  lcd.setCursor(0, 0);
  if(!nh.connected())
    {
      lcd.fillScreen(lcd.color565(255, 0, 0));
    }
  else
    {
      lcd.setTextSize(4);
      lcd.println(battery_voltage_);
    }
  delay(500);
}
