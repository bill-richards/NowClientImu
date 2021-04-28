#include "DataTaskRunner.h"

bool DataTaskRunner::datagramIsValid()  { return true; }

void DataTaskRunner::sendData()
{
  PREPARE_DATAGRAM(NowTransmittedDataTypes::IMU, _boardId)

  Serial.println("Sending data");
  
  _datagram.imu.pitch = _imuReader.pitch();
  _datagram.imu.roll = _imuReader.roll();

  if(datagramIsValid() )
  { 
    SEND_MESSAGE_USING_ESP_NOW(local_broadcastAddress, _datagram) 
  }
    
  _exitTime = micros(); 
}

void DataTaskRunner::run()
{
    const ulong currentMillis = micros();
    if (currentMillis - _exitTime < _interval) return;
    _imuReader.run();
    sendData(); 
}

void task(void * parameter)
{
    Serial.println("Starting IMU monitor task");
    _runner = (DataTaskRunner*)parameter;    
    Serial.println("Calibrating");
    _runner->calibrateImu();
    Serial.println("IMU Calibrated");

    for( ;; )
    {
        _runner->run();
        portYIELD();
        vTaskDelay(100);
    }
}