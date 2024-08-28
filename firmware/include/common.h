uint16_t colorMap(int code, bool isBackground = false) {
  if (isBackground) {
    switch (code) {
    case 40: return lcd.color565(0, 0, 0);     // Black
    case 41: return lcd.color565(255, 0, 0);   // Red
    case 42: return lcd.color565(0, 255, 0);   // Green
    case 43: return lcd.color565(255, 255, 0); // Yellow
    case 44: return lcd.color565(0, 0, 255);   // Blue
    case 45: return lcd.color565(255, 0, 255); // Magenta
    case 46: return lcd.color565(0, 255, 255); // Cyan
    case 47: return lcd.color565(255, 255, 255); // White
    case 49: return lcd.color565(0, 0, 0);     // Default (Black)
    default: return lcd.color565(0, 0, 0);
    }
  } else {
    switch (code) {
    case 30: return lcd.color565(0, 0, 0);     // Black
    case 31: return lcd.color565(255, 0, 0);   // Red
    case 32: return lcd.color565(0, 255, 0);   // Green
    case 33: return lcd.color565(255, 255, 0); // Yellow
    case 34: return lcd.color565(0, 0, 255);   // Blue
    case 35: return lcd.color565(255, 0, 255); // Magenta
    case 36: return lcd.color565(0, 255, 255); // Cyan
    case 37: return lcd.color565(255, 255, 255); // White
    case 39: return lcd.color565(255, 255, 255); // Default (White)
    default: return lcd.color565(255, 255, 255);
    }
  }
}

void printColorText(const String& input) {
  String text = input;
  uint16_t textColor = lcd.color565(255, 255, 255); // Default text color: white
  uint16_t bgColor = lcd.color565(0, 0, 0);         // Default background color: black
  int index = 0;

  while (index < text.length()) {
    if (text.charAt(index) == '\x1b' && text.charAt(index + 1) == '[') {
      int mIndex = text.indexOf('m', index);
      if (mIndex != -1) {
        String seq = text.substring(index + 2, mIndex);
        int code = seq.toInt();
        if (seq.startsWith("4")) {
          bgColor = colorMap(code, true);
        } else if (seq.startsWith("3")) {
          textColor = colorMap(code, false);
        }
        text.remove(index, mIndex - index + 1);
        continue;
      }
    }
    lcd.setTextColor(textColor, bgColor);
    lcd.print(text.charAt(index));
    index++;
  }
}

