#include <temperaturaHumedad.h>

temperaturaHumedad::temperaturaHumedad(/* args */)
{
}

temperaturaHumedad::temperaturaHumedad(int tempPin, int humPin)
{
    _tempPin = tempPin;
    _humPin = humPin;
}

temperaturaHumedad::temperaturaHumedad(int tempPin, int humPin, int adcResolution)
{
    _tempPin = tempPin;
    _humPin = humPin;
    _adcResolution = adcResolution;
}

temperaturaHumedad::temperaturaHumedad(int tempPin, int humPin, int adcResolution, float vref)
{
    _tempPin = tempPin;
    _humPin = humPin;
    _adcResolution = adcResolution;
    _VREF = vref;
}

temperaturaHumedad::~temperaturaHumedad()
{
}

float temperaturaHumedad::getTemp()
{
    float Tc, analogVolt;
    analogVolt = (float)analogRead(_tempPin) / _adcResolution * _VREF;
    // Convert voltage to temperature (℃, centigrade)
    Tc = -66.875 + 72.917 * analogVolt;
    // Convert voltage to temperature (°F, fahrenheit )
    // Tf = -88.375 + 131.25 * analogVolt;
    return Tc;
}

float temperaturaHumedad::getHum()
{
    float analogVolt, RH;
    analogVolt = (float)analogRead(_humPin) / _adcResolution * _VREF;
    // Convert voltage to relative humidity (%)
    RH = -12.5 + 41.667 * analogVolt;
    return RH;
}
