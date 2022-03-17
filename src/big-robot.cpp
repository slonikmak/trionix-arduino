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
//#include <Parser.h>
#include "MPU9250.h"
#include "eeprom_utils.h"
#include "MS5837.h"
#include <Servo_Hardware_PWM.h> //использует 3 4 и 5 таймеры для аппаратного ШИМ


#define PARSE_AMOUNT 9        // число значений в массиве, который хотим получить
#define INPUT_AMOUNT 100      // максимальное количество символов в пакете, который идёт в сериал
char inputData[INPUT_AMOUNT]; // массив входных значений (СИМВОЛЫ)
int intData[PARSE_AMOUNT];    // массив численных значений после парсинга
boolean recievedFlag;
boolean getStarted;
byte index;
String string_convert;

//front_left
#define pin1 5
//front_right
#define pin2 2
//back_left
#define pin3 3
//back_right
#define pin4 6
//back_vertical
#define pin5 7
//back_horizontal
#define pin6 8


//servo
#define s_pin1 44
//light
#define ledPin 45
//claw
#define s_pin2 46

Servo m1, m2, m3, m4, m5, m6; //моторы
Servo s1,s2; //серва и клешня


int m1_val, m2_val, m3_val, m4_val, m5_val, m6_val;

int ledValue = 0;

int BatMeasPin = A0;
float BatMeasVal = 0;
#define VREF 2.5
#define DIV_R3 13000
#define DIV_R4 2000

const int BUFFER_SIZE = 100;
char buf[BUFFER_SIZE];

MPU9250 mpu;

//БЛОК параметров датчика давления
MS5837 sensor;
float depth_cal = 0; //калибровочное значение глубины

int mode = 1; // 1 - streaming,  2 - calibration

void attach_pins()
{
    m1.attach(pin1);
    m1.writeMicroseconds(1500); // send "stop" signal to ESC.
    m2.attach(pin2);
    m2.writeMicroseconds(1500);
    m3.attach(pin3);
    m3.writeMicroseconds(1500);
    m4.attach(pin4);
    m4.writeMicroseconds(1500);
    m5.attach(pin5);
    m5.writeMicroseconds(1500);
    m6.attach(pin6);
    m6.writeMicroseconds(1500);


    s1.attach(s_pin1);
    s1.write(90);  // центральное положение сервы
    
    s2.attach(s_pin2);
    s2.writeMicroseconds(1500); // нейтральное положение клешни

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

void printDebugMsg(String msg)
{
    Serial.print("#2 " + msg + ";");
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
                    + String(sensor.temperature()) + " "
                    // Battery Voltage
                    + String(BatMeasVal) + ";";

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
    analogReference(EXTERNAL);

    attach_pins();

    delay(8000); //ожидание загрузки бутлоадера raspberry

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

    sensor.full_read();
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

    int m1_val_new = map(intData[1], -100, 100, 1100, 1900);
    int m2_val_new = map(intData[2], -100, 100, 1100, 1900);
    int m3_val_new = map(intData[3], -100, 100, 1100, 1900);
    int m4_val_new = map(intData[4], -100, 100, 1100, 1900);
    int m5_val_new = map(intData[5], -100, 100, 1100, 1900);
    int m6_val_new = map(intData[6], -100, 100, 1100, 1900);

    int servo_val_new = map(intData[8], -90, 90, 0, 180);
    int claw_val_new = map(intData[9], -100, 100, 1100, 1900);

    if (intData[7] != ledValue)
    {
        ledValue = intData[7];
        analogWrite(ledPin, ledValue);
    }

    m1.writeMicroseconds(m1_val_new);

    m2.writeMicroseconds(m2_val_new);

    m3.writeMicroseconds(m3_val_new);

    m4.writeMicroseconds(m4_val_new);

    m5.writeMicroseconds(m5_val_new);

    m6.writeMicroseconds(m6_val_new);

    s1.write(servo_val_new);

    s2.writeMicroseconds(claw_val_new);


}

void parsing()
{
    while (Serial.available() > 0)
    {
        char incomingByte = Serial.read(); // обязательно ЧИТАЕМ входящий символ
        if (incomingByte == '$')
        {                      // если это $
            getStarted = true; // поднимаем флаг, что можно парсить
        }
        else if (incomingByte != ';' && getStarted)
        { // пока это не ;
            // в общем происходит всякая магия, парсинг осуществляется функцией strtok_r
            inputData[index] = incomingByte;
            index++;
            inputData[index] = NULL;
        }
        else
        {
            if (getStarted)
            {
                char *p = inputData;
                char *str;
                index = 0;
                String value = "";
                while ((str = strtok_r(p, " ", &p)) != NULL)
                {
                    string_convert = str;
                    intData[index] = string_convert.toInt();
                    index++;
                }
                index = 0;
            }
        }
        if (incomingByte == ';')
        { // если таки приняли ; - конец парсинга
            getStarted = false;
            recievedFlag = true;
        }
    }
}

void loop()
{


    // if (mpu.update())
    // {
    //     static uint32_t prev_ms = millis();
    //     if (millis() > prev_ms + 30)
    //     {
    //         printData();
    //         prev_ms = millis();
    //     }
    // }

    parsing(); // функция парсинга
    if (recievedFlag)
    { // если получены данные
        if (recievedFlag)
        {
            int com_type = intData[0];

            if (com_type == 2 && mode == 1)
            {
                calibrateIMU();
                printServiceMsg("calibrate imu");
            }

            else if (com_type == 3)
            {
                setMotors();
                printData();
            }
        }
        recievedFlag = false;
    }
    sensor.read(); //чтение глубины
    BatMeasVal = (float)analogRead(0) * VREF * ((DIV_R3 + DIV_R4) / DIV_R4) / 1024;
}
