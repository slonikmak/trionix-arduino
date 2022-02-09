/*
Протокол из Arduino:
#0 [msg]- сервисное сообщение
#1 [1 23 44 55] - значение датчиков

Протокол в Arduino (режимы работы)
$1 - старт стриминга данных датчиков
$2 - калибровка
$3 - установка моторов

Калибровка https://github.com/hideakitai/MPU9250/blob/master/examples/calibration_eeprom/calibration_eeprom.ino
*/

#include <Arduino.h>
#include <SerialParser.h>
#include "MPU9250.h"
#include <GyverOS.h>
#include "eeprom_utils.h"
#include "MS5837.h"
#include <Servo_Hardware_PWM.h>

#define PARSE_AMOUNT 6

//front
#define pin1 6
//right
#define pin2 7
//back
#define pin3 8
//left
#define pin4 5
//servo front
#define s_pin1 11
//servo right
#define s_pin2 10
//servo back
#define s_pin3 12
//servo left
#define s_pin4 13
//light
#define ledPin 9

Servo m1, m2, m3, m4;
Servo s1, s2, s3, s4;

int m1_val, m2_val, m3_val, m4_val;

int ledValue = 0;

GyverOS<3> OS; // указать макс. количество задач
MPU9250 mpu;
SerialParser parser(PARSE_AMOUNT);

//БЛОК параметров датчика давления
MS5837 sensor;
float depth_cal = 0; //калибровочное значение глубины

int mode = 1; // 1 - streaming,  2 - calibration

void attach_pins()
{
  m1.attach(pin2);
  m1.writeMicroseconds(1500); // send "stop" signal to ESC.
  m2.attach(pin3);
  m2.writeMicroseconds(1500);
  m3.attach(pin1);
  m3.writeMicroseconds(1500);
  m4.attach(pin4);
  m4.writeMicroseconds(1500);

  //    s1.attach(s_pin1);
  //    s2.attach(s_pin2);
  //    s3.attach(s_pin3);
  //    s4.attach(s_pin4);

  pinMode(ledPin, OUTPUT);
}

void setDefaultIMUValues()
{
    mpu.setAccBias(0., 0., 0.);
    mpu.setGyroBias(0., 0., 0.);
    mpu.setMagBias(0., 0., 0.);
    mpu.setMagScale(1., 1., 1.);
}

void printServiceMsg(String msg)
{
    Serial.print("#0 " + msg + ";");
    
}

void stopStreaming()
{
    OS.stop(0);
    OS.stop(1);
}

void straeming()
{
    mode = 1;
    OS.start(0);
    OS.start(1);
}

void printData()
{
        String answer = "#1 "
                    // heading
                    + String(mpu.getYaw()) + " "
                    // pitch
                    + String(mpu.getPitch()) + " "
                    // roll
                    + String(mpu.getRoll()) + " "
                    // depth
                    + String(sensor.depth() - depth_cal) + " "
                    // temp
                    + String(sensor.temperature()) + ";";

        Serial.print(answer);
    
}

void updateDepth()
{
    sensor.read();
}

void updateIMU()
{
    mpu.update();
}

void setup()
{
    Wire.begin();
    Serial.begin(115200);

    attach_pins();

    analogWrite(ledPin, 120);
    
    delay(2000);
    if (!mpu.setup(0x68))
    { // change to your own address
        while (1)
        {
            printServiceMsg("MPU_connection_failed.Please_check_your_connection_with_`connection_check`_example.");
            delay(1000);
        }
    }

    while (!sensor.init())
    {
        printServiceMsg("Depth_sensor_init_failed!");
        delay(1000);
    }

#if defined(ESP_PLATFORM) || defined(ESP8266)
    EEPROM.begin(0x80);
#endif

    // delay(5000);

    sensor.setModel(MS5837::MS5837_30BA);
    sensor.setFluidDensity(997); // kg/m^3 (freshwater, 1029 for seawater)

    sensor.read();
    depth_cal = sensor.depth(); //калибровка глубины в самом начале работы

    printServiceMsg("Started");

    if (isCalibrated())
    {
        printServiceMsg("IMU_calibrated");
    }
    else
    {
        printServiceMsg("IMU_not_calibrated");
    }

    delay(1000);

    analogWrite(ledPin, 0);

    OS.attach(0, updateDepth, 100);
    OS.attach(1, printData, 50);

    // straeming();
}

void calibrateIMU()
{
    mode = 2;
    // calibrate anytime you want to
    printServiceMsg("Calibration_started");
    printServiceMsg("Accel_Gyro_calibration_will_start_in_5sec.");
    printServiceMsg("Please_leave_the_device_still_on_the_flat_plane.");
    delay(5000);
    mpu.calibrateAccelGyro();

    printServiceMsg("Mag_calibration_will_start_in_5sec.");
    printServiceMsg("Please_Wave_device_in_a_figure_eight_until_done.");
    delay(5000);
    mpu.calibrateMag();
    mpu.verbose(false);

    // save to eeprom
    printServiceMsg("Write_calibrated_parameters_to_EEPROM");
    saveCalibration();

    // load from eeprom
    printServiceMsg("Load_calibrated_parameters_from_EEPROM");
    loadCalibration();
    printServiceMsg("Calibration_done");
}

void setMotors()
{
    int *intData = parser.getData();
    int m1_val_new = map(intData[1], -100, 100, 1100, 1900);
    int m2_val_new = map(intData[2], -100, 100, 1100, 1900);
    int m3_val_new = map(intData[3], -100, 100, 1100, 1900);
    int m4_val_new = map(intData[4], -100, 100, 1100, 1900);

    if (intData[5] != ledValue) {
        ledValue = intData[5];
        analogWrite(ledPin, ledValue);
    }


//        int s1_val = dataArray[4];
//        int s2_val = dataArray[5];
//        int s3_val = dataArray[6];
//        int s4_val = dataArray[7];
//
//        int l1_val = dataArray[8];

        m1.writeMicroseconds(m1_val_new);

        m2.writeMicroseconds(m2_val_new);

        m3.writeMicroseconds(m3_val_new);

        m4.writeMicroseconds(m4_val_new);
}

void loop()
{
    parser.update();

    if (parser.received())
    {
        int comand = parser.getData()[0];

        if (comand == 2 && mode == 1)
        {
            stopStreaming();
            calibrateIMU();
        }
        else if (comand == 1 && mode == 2)
        {
            straeming();
        }

        else if (comand == 3) {
            setMotors();
        }

    }
    updateIMU();

    OS.tick();

    
}
