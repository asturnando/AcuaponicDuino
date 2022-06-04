# # AcuaponicDuino - Automatización y control de cultivos acuapónicos con Arduino y Node-RED sobre protocolo MQTT
Codigo perteneciente al TFG del Grado de Ingenieria Informatica de la **Universitat Oberta de Catalunya**

- Autor: **Fernando Suarez Rodriguez**
- Fecha: **2022/06/06**
- Version: **1.0**
- Descripcion: Programa que controlara el funcionamiento de la placa de control del sistema de acuaponia (Arduino Mega 2560 rev3). El programa esta realizado en FreeRTOS, y se encargara de la automatizacion de las diferentes tareas del sistema asi como de la recepcion de datos de los sensores. Los datos recibidos de los sensores, seran enviados por el puerto serie hacia un ESP8266 que sera el encargado de transmitir los datos a la red MQTT para luego ser leidos en Node-RED.
