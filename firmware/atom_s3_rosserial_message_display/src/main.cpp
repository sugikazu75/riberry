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
  void VoltageSub(const std_msgs::Float32& voltage_msg);

private:
  float maxVoltage_;
  float minVoltage_;
  float Voltage_;
  ros::Subscriber<std_msgs::Float32> voltage_sub_;
};

#endif


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
