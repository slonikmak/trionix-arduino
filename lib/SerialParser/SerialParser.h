#ifndef SerialParser_h
#define SerialParser_h

#include <Arduino.h>

class SerialParser
{
    public:
        SerialParser(int parseAmount);
        boolean received();
        void update();
        int *getData();
        ~SerialParser();
    private:
        int *array;
        int _parseAmount;
        boolean receivedFlag;
        boolean getStarted;
        String string_convert;
        byte index;
};


#endif