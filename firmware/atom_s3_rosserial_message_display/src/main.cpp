#include "M5AtomS3.h"
#include "ArduinoHWCDCHardware.h"
#include "ros/node_handle.h"

namespace ros{
  typedef NodeHandle_<ArduinoHardware> NodeHandle;
}

#include <std_msgs/Float32.h>

// #define LGFX_M5ATOMS3
// #include <LovyanGFX.hpp>
// #include <LGFX_AUTODETECT.hpp>

#include "config.h"
#include "common.h"
#include "battery_display.h"


ros::NodeHandle nh;

int bat_cell;
float battery_voltage_;
void batteryVoltageCallback(const std_msgs::Float32& msg){
  battery_voltage_ = msg.data;
}

ros::Subscriber<std_msgs::Float32> battery_voltage_sub_("battery_voltage_status", &batteryVoltageCallback);

BatteryDisplay batDisp;

void setup()
{
  // For display
  M5.Lcd.init();
  M5.Lcd.setRotation(lcd_rotation);
  M5.Lcd.clear();
  M5.Lcd.setTextSize(1.5);

  M5.Lcd.println("waiting for rosserial connection");

  nh.initNode();
  nh.subscribe(battery_voltage_sub_);

  M5.begin();
  batDisp.displayFrame();


  while (!nh.connected()) {
    nh.spinOnce();
    delay(100);
  }
  nh.getParam("~bat_cell", &bat_cell);
  batDisp.SetBatcell(bat_cell);
  M5.Lcd.printf("bat_cell is %d", bat_cell);
  M5.Lcd.println();
  M5.Lcd.println("rosserial init done!");

  delay(1000);
}

void loop()
{
  nh.spinOnce();

  M5.Lcd.clear();
  M5.Lcd.setCursor(0, 0);
  if(!nh.connected())
    {
      M5.Lcd.fillScreen(M5.Lcd.color565(255, 0, 0));
    }
  else
    {
      batDisp.updateVoltage(battery_voltage_);
    }
  delay(500);
}
