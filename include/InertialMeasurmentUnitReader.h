#ifndef _InertialMeasurmentUnitReader_h_
#define _InertialMeasurmentUnitReader_h_

class InertialMeasurmentUnitReader
{
private:
    int gyro_x, gyro_y, gyro_z;
    long gyro_x_cal, gyro_y_cal, gyro_z_cal;
    bool set_gyro_angles;
    
    long acc_x, acc_y, acc_z, acc_total_vector;
    float angle_roll_acc, angle_pitch_acc;
    
    float angle_pitch, angle_roll;
    int angle_pitch_buffer, angle_roll_buffer;
    float angle_pitch_output, angle_roll_output;
    
    int temp;

    void setupRegisters();
    void read();
public:
    void calibrate();
    void run();
    inline float pitch() { return angle_pitch_output; }
    inline float roll() { return angle_roll_output; }
};

#endif

