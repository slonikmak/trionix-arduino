/*
Протокол из Arduino:
#0 [msg]- сервисное сообщение
#1 [1 23 44 55] - значение датчиков

Протокол в Arduino (режимы работы)
$1 - отправка данных датчиков
$2 - калибровка
$3 - к приёму готов

Калибровка https://github.com/hideakitai/MPU9250/blob/master/examples/calibration_eeprom/calibration_eeprom.ino
*/

#include <Arduino.h>
#include <SerialParser.h>
#include "MPU9250.h"
#include <GyverOS.h>
#include "eeprom_utils.h"
#include "MS5837.h"

#define PARSE_AMOUNT 1

GyverOS<3> OS; // указать макс. количество задач
MPU9250 mpu;
SerialParser parser(PARSE_AMOUNT);

//БЛОК параметров датчика давления
MS5837 sensor;
float depth_cal = 0; //калибровочное значение глубины

int mode = 1; // 1 - streaming,  2 - calibration

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

    OS.attach(0, updateDepth, 50);
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
            ready = true;
        }
    }
    updateIMU();

    OS.tick();

    
}
