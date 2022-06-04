#ifndef __SENSORTEMPERATURAAGUA_H__
#define __SENSORTEMPERATURAAGUA_H__
#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>

class sensorTemperaturaAgua
{
private:
    int _oneWirePin;
    OneWire _onWireBus;
    DallasTemperature sensor;
public:
    sensorTemperaturaAgua(int pin);
    ~sensorTemperaturaAgua();
    float getTemp();
};




#endif // __SENSORTEMPERATURAAGUA_H__