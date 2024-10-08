static M5GFX lcd;
LGFX_Sprite sprite;
#define TFT_BL 10

#ifdef USE_GROVE
constexpr int SDA_PIN = 2;
constexpr int SCL_PIN = 1;
#else
constexpr int SDA_PIN = 38;
constexpr int SCL_PIN = 39;
#endif

#ifdef LCD_ROTATION
constexpr int lcd_rotation = LCD_ROTATION;
#else
constexpr int lcd_rotation = 1;
#endif

#ifdef I2C_ADDR
constexpr int I2C_SLAVE_ADDR = I2C_ADDR;
#else
constexpr int I2C_SLAVE_ADDR = 0x42;
#endif
constexpr int image_width = 139;
constexpr int image_height = 139;

unsigned long lastReceiveTime = 0;
const unsigned long receiveTimeout = 15000;  // Timeout in milliseconds (15 seconds)
