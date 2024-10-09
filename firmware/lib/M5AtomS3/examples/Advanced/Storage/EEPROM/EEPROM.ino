/*
*******************************************************************************
* Copyright (c) 2021 by M5Stack
*                  Equipped with M5AtomS3 sample source code
*                          配套  M5AtomS3 示例源代码
* Visit for more information: https://docs.m5stack.com/en/core/AtomS3
* 获取更多资料请访问: https://docs.m5stack.com/zh_CN/core/AtomS3
*
* Describe: EEPROM Read & Write.
* Date: 2022/12/20
*******************************************************************************
The values stored in the EEPROM will remain in the EEPROM even after the
AtomS3 is disconnected. When a new program is uploaded to the AtomS3, the values
stored in the EEPROM can still be called or modified by the new program.
储存于EEPROM的数值即使在断开 AtomS3电源后仍会保存在EEPROM中
当新程序上传到 AtomS3后，储存于EEPROM中的数值仍然可以被新的程序调用或者修改
*/

#include <M5AtomS3.h>
#include <EEPROM.h>

int addr = 0;  // EEPROM Start number of an ADDRESS.  EEPROM地址起始编号
#define SIZE 16  // define the size of EEPROM(Byte).  定义EEPROM的大小(字节)

void setup() {
    AtomS3.begin();             // Init Atom.  初始化Atom
    if (!EEPROM.begin(SIZE)) {  // Request storage of SIZE size(success return
                                // 1).  申请SIZE大小的存储(成功返回1)
        AtomS3.Lcd.println(
            "\nFailed to initialise EEPROM!");  // 串口输出格式化字符串.  Serial
                                                //  output format string
        delay(1000000);
    }
    AtomS3.Lcd.println("\nRead data from EEPROM. Values are:");
    for (int i = 0; i < SIZE; i++) {
        AtomS3.Lcd.printf(
            "%d ",
            EEPROM.read(i));  // Reads data from 0 to SIZE in EEPROM.
                              // 读取EEPROM中从0到SIZE中的数据
    }
    AtomS3.Lcd.println("\n\nPress Btn to Write EEPROM");
}

void loop() {
    AtomS3.update();  // Check button down state.  检测按键按下状态
    if (AtomS3.BtnA.isPressed()) {  // if the button is Pressed.  如果按键按下
        AtomS3.Lcd.setCursor(0, 0);
        AtomS3.Lcd.fillRect(0, 0, 128, 128, BLACK);
        AtomS3.Lcd.printf("\n%d Bytes datas written on EEPROM.\nValues are:\n",
                          SIZE);
        for (int i = 0; i < SIZE; i++) {
            int val = random(
                256);  // Integer values to be stored in the EEPROM (EEPROM can
                       // store one byte per memory address, and can only store
                       // numbers from 0 to 255. Therefore, if you want to use
                       // EEPROM to store the numeric value read by the analog
                       // input pin, divide the numeric value by 4.
            // 将要存储于EEPROM的整数数值(EEPROM每一个存储地址可以储存一个字节，只能存储0-255的数.故如果要使用EEPROM存储模拟输入引脚所读取到的数值需要将该数值除以4)
            EEPROM.write(addr,
                         val);  // Writes the specified data to the specified
                                // address.  向指定地址写入指定数据
            AtomS3.Lcd.printf("%d ", val);
            addr += 1;  // Go to the next storage address.  转入下一存储地址
        }
        // When the storage address sequence number reaches the end of the
        // storage space of the EEPROM, return to.
        // 当存储地址序列号达到EEPROM的存储空间结尾，返回到EEPROM开始地址
        addr = 0;
        AtomS3.Lcd.println("\n\nRead form EEPROM. Values are:");
        for (int i = 0; i < SIZE; i++) {
            AtomS3.Lcd.printf("%d ", EEPROM.read(i));
        }
        AtomS3.Lcd.println("\n-------------------------------------\n");
    }
    delay(150);
}