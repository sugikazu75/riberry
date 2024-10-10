#ifndef BatteryDisplay_h
#define BatteryDisplay_h

// voltage unit: V
class BatteryDisplay
{
public:
  BatteryDisplay();

public:
  void SetBatcell(int bat_cell);
  void displayFrame();
  void updateVoltage(float voltage);
  float calcPercentage(float voltage);
  static constexpr float VOLTAGE_100P =  4.2;
  static constexpr float VOLTAGE_90P =  4.085;
  static constexpr float VOLTAGE_80P =  3.999;
  static constexpr float VOLTAGE_70P =  3.936;
  static constexpr float VOLTAGE_60P =  3.883;
  static constexpr float VOLTAGE_50P =  3.839;
  static constexpr float VOLTAGE_40P =  3.812;
  static constexpr float VOLTAGE_30P =  3.791;
  static constexpr float VOLTAGE_20P =  3.747;
  static constexpr float VOLTAGE_10P =  3.1;
  static constexpr float VOLTAGE_0P =  3.0;

private:
  float Voltage_;
  int bat_cell_;
};

#endif
