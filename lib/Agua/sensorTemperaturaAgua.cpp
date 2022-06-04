#include <sensorTemperaturaAgua.h>

sensorTemperaturaAgua::sensorTemperaturaAgua(int pin)
{
  _oneWirePin = pin;
  _onWireBus.begin(_oneWirePin);
  sensor.setOneWire(&_onWireBus);
  sensor.begin();
}
sensorTemperaturaAgua::~sensorTemperaturaAgua()
{
}

float sensorTemperaturaAgua::getTemp()
{
  this->sensor.requestTemperatures();
  return this->sensor.getTempCByIndex(0);
}
