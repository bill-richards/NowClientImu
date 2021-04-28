#ifndef _ImuTaskRunner_h_
#define _ImuTaskRunner_h_

#include <NowCommon.h>
#include <TaskRunner.h>

#include "InertialMeasurmentUnitReader.h"

class DataTaskRunner
{
private:
    String _serverMacAddress;
    char _boardId[16];
    uint _taskPriority;
    esp_datagram _datagram;
    const ulong _interval = 4000;
    unsigned long _exitTime     = 0; // Stores last time temperature was published
    InertialMeasurmentUnitReader _imuReader;

    inline bool datagramIsValid();

public:
    DataTaskRunner(const char * board_id, String server_mac, uint task_priority) 
    { 
        _serverMacAddress = server_mac;
        _taskPriority = task_priority;
        memcpy(&_boardId, board_id, 32); 
    }
    void begin() 
    { 
        xTaskCreatePinnedToCore(task, "IMU_TSK", 10000, this, _taskPriority, NULL, 1);
    }

    inline void calibrateImu() { _imuReader.calibrate(); }
    inline float getPitch() { return _imuReader.pitch(); }
    inline float getRoll() { return _imuReader.roll(); }

    void run();
    void sendData();
};

DataTaskRunner* _runner;

#endif