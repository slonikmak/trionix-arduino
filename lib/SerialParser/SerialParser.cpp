#include "Arduino.h"
#include "SerialParser.h"

SerialParser::SerialParser(int parseAmount)
{
    _parseAmount = parseAmount;
    receivedFlag = false;
    array = new int[parseAmount];
    string_convert = "";
}

boolean SerialParser::received()
{
    return receivedFlag;
}

int *SerialParser::getData()
{
    receivedFlag = false;
    return array;
}

void SerialParser::update()
{
    if (Serial.available() > 0)
    {
        char incomingByte = Serial.read(); // обязательно ЧИТАЕМ входящий символ
        if (getStarted)
        { // если приняли начальный символ (парсинг разрешён)
            if (incomingByte != ' ' && incomingByte != ';')
            {                                   // если это не пробел И не конец
                string_convert += incomingByte; // складываем в строку
            }
            else
            {                                          // если это пробел или ; конец пакета
                array[index] = string_convert.toInt(); // преобразуем строку в int и кладём в массив
                string_convert = "";                   // очищаем строку
                index++;                               // переходим к парсингу следующего элемента массива
            }
        }
        if (incomingByte == '$')
        {                        // если это $
            getStarted = true;   // поднимаем флаг, что можно парсить
            index = 0;           // сбрасываем индекс
            string_convert = ""; // очищаем строку
        }
        if (incomingByte == ';')
        {                        // если таки приняли ; - конец парсинга
            getStarted = false;  // сброс
            receivedFlag = true; // флаг на принятие
        }
    }
}

SerialParser::~SerialParser() {
    delete[] array;
}