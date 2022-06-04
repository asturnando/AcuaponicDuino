#ifndef __TEMPERATURAHUMEDAD_H__
#define __TEMPERATURAHUMEDAD_H__
#include <Arduino.h>

class temperaturaHumedad
{
private:
    float _VREF = 5.0;
    int _tempPin = A1;
    int _humPin = A0;
    int _adcResolution=1024;

public:
    temperaturaHumedad();
    temperaturaHumedad(int tempPin, int humPin);
    temperaturaHumedad(int tempPin, int humPin, int adcResolution);
    temperaturaHumedad(int tempPin, int humPin, int adcResolution, float vref);
    ~temperaturaHumedad();
    float getTemp();
    float getHum();
};

#endif // __TEMPERATURAHUMEDAD_H__