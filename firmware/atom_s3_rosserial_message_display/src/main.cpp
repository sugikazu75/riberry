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


#define LCD_H M5.Lcd.height()
#define LCD_W M5.Lcd.width()


BatteryDisplay::BatteryDisplay()
{
}
void BatteryDisplay::SetBatcell(int bat_cell)
{
  bat_cell_ = bat_cell;
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

float BatteryDisplay::calcPercentage(float voltage){
  float average_voltage = voltage / bat_cell_;
  float input_cell = voltage / VOLTAGE_100P;
  float percentage = 0;
  if(average_voltage  > VOLTAGE_90P) percentage = (average_voltage - VOLTAGE_90P) / (VOLTAGE_100P - VOLTAGE_90P) * 10 + 90;
  else if (average_voltage  > VOLTAGE_80P) percentage = (average_voltage - VOLTAGE_80P) / (VOLTAGE_90P - VOLTAGE_80P) * 10 + 80;
  else if (average_voltage  > VOLTAGE_70P) percentage = (average_voltage - VOLTAGE_70P) / (VOLTAGE_80P - VOLTAGE_70P) * 10 + 70;
  else if (average_voltage  > VOLTAGE_60P) percentage = (average_voltage - VOLTAGE_60P) / (VOLTAGE_70P - VOLTAGE_60P) * 10 + 60;
  else if (average_voltage  > VOLTAGE_50P) percentage = (average_voltage - VOLTAGE_50P) / (VOLTAGE_60P - VOLTAGE_50P) * 10 + 50;
  else if (average_voltage  > VOLTAGE_40P) percentage = (average_voltage - VOLTAGE_40P) / (VOLTAGE_50P - VOLTAGE_40P) * 10 + 40;
  else if (average_voltage  > VOLTAGE_30P) percentage = (average_voltage - VOLTAGE_30P) / (VOLTAGE_40P - VOLTAGE_30P) * 10 + 30;
  else if (average_voltage  > VOLTAGE_20P) percentage = (average_voltage - VOLTAGE_20P) / (VOLTAGE_30P - VOLTAGE_20P) * 10 + 20;
  else if (average_voltage  > VOLTAGE_10P) percentage = (average_voltage - VOLTAGE_10P) / (VOLTAGE_20P - VOLTAGE_10P) * 10 + 10;
  else percentage = (average_voltage - VOLTAGE_0P) / (VOLTAGE_10P - VOLTAGE_0P) * 10;
  return percentage;
}

void BatteryDisplay::updateVoltage(float voltage)
{
  float voltageRatio = calcPercentage(voltage)/100;
  voltageRatio = constrain(voltageRatio, 0.0f, 1.0f);

  // Erase screen.
  M5.Lcd.fillRect(1, 20, 60, 16, BLACK);
  M5.Lcd.fillRect(LCD_W/2+17, 20, 32, 16, BLACK);

  // Show voltage.
  M5.Lcd.setTextColor(WHITE);
  M5.Lcd.setCursor(2, 21);
  M5.Lcd.printf("%0.2f", voltage);
  M5.Lcd.drawString("V", LCD_W/2, 22, 1);
  M5.Lcd.setCursor(LCD_W/2+25, 21);
  M5.Lcd.print((uint8_t)(voltageRatio*100));
  M5.Lcd.drawString("%", LCD_W-12, 22, 1);

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
