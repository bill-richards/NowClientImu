#include <Arduino.h>
#include <Wire.h>

#include "InertialMeasurmentUnitReader.h"

void InertialMeasurmentUnitReader::setupRegisters()
{
    Wire.beginTransmission(0x68); 
    Wire.write(0x6B);               //Send the requested starting register
    Wire.write(0x00);               //Set the requested starting register
    Wire.endTransmission(); 
                                                
    //Configure the accelerometer (+/-8g)
    Wire.beginTransmission(0x68); 
    Wire.write(0x1C);   
    Wire.write(0x10); 
    Wire.endTransmission(); 
                                                
    //Configure the gyro (500dps full scale)
    Wire.beginTransmission(0x68);
    Wire.write(0x1B);
    Wire.write(0x08); 
    Wire.endTransmission(); 
}

void InertialMeasurmentUnitReader::calibrate()
{
    setupRegisters();
    for (int cal_int = 0; cal_int < 1000 ; cal_int ++)
    {
        read();
        gyro_x_cal += gyro_x;   //Add the gyro x offset to the gyro_x_cal variable
        gyro_y_cal += gyro_y;   //Add the gyro y offset to the gyro_y_cal variable
        gyro_z_cal += gyro_z;   //Add the gyro z offset to the gyro_z_cal variable
        vTaskDelay(3);          //Delay 3us to have 250Hz for-loop
    }
 
    // Divide all results by 1000 to get average offset
    gyro_x_cal /= 1000;                                                 
    gyro_y_cal /= 1000;                                                 
    gyro_z_cal /= 1000;
}

void InertialMeasurmentUnitReader::read()
{
    Wire.beginTransmission(0x68);   //Start communicating with the MPU-6050
    Wire.write(0x3B);               //Send the requested starting register
    Wire.endTransmission();         //End the transmission
    Wire.requestFrom(0x68,14);      //Request 14 bytes from the MPU-6050
    while(Wire.available() < 14);   //Wait until all the bytes are received
    
    //Following statements left shift 8 bits, then bitwise OR.  
    //Turns two 8-bit values into one 16-bit value                                       
    acc_x = Wire.read()<<8|Wire.read();                                  
    acc_y = Wire.read()<<8|Wire.read();                                  
    acc_z = Wire.read()<<8|Wire.read();                                  
    temp = Wire.read()<<8|Wire.read();                                   
    gyro_x = Wire.read()<<8|Wire.read();                                 
    gyro_y = Wire.read()<<8|Wire.read();                                 
    gyro_z = Wire.read()<<8|Wire.read();     
}

void InertialMeasurmentUnitReader::run()
{
    read();

    gyro_x -= gyro_x_cal;                                                
    gyro_y -= gyro_y_cal;                                                
    gyro_z -= gyro_z_cal;     

    // Gyro angle calculations . Note 0.0000611 = 1 / (250Hz x 65.5)
    // 0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians                                
    
    
    angle_pitch += gyro_x * 0.0000611;                      // Calculate the traveled pitch angle and add this to the angle_pitch variable
    angle_roll += gyro_y * 0.0000611;                       // Calculate the traveled roll angle and add this to the angle_roll variable                                        
    
    angle_pitch += angle_roll * sin(gyro_z * 0.000001066);  // If the IMU has yawed transfer the roll angle to the pitch angle
    angle_roll -= angle_pitch * sin(gyro_z * 0.000001066);  // If the IMU has yawed transfer the pitch angle to the roll angle
    
    // Accelerometer angle calculations
    // 57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians    
    
    acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z)); // Calculate the total accelerometer vector
    angle_pitch_acc = asin((float)acc_y/acc_total_vector)* 57.296;      // Calculate the pitch angle
    angle_roll_acc = asin((float)acc_x/acc_total_vector)* -57.296;      // Calculate the roll angle
        
    angle_pitch_acc -= 0.0; // Accelerometer calibration value for pitch
    angle_roll_acc -= 0.0;  //Accelerometer calibration value for roll
    
    if(set_gyro_angles)
    { 
        // If the IMU has been running then we need to: 
            
        angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004; // correct the drift of the gyro pitch angle with the accelerometer pitch angle
        angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;    // correct the drift of the gyro roll angle with the accelerometer roll angle
    }
    else
    { 
        // When the IMU has just started :
        
        angle_pitch = angle_pitch_acc;  //Set the gyro pitch angle equal to the accelerometer pitch angle
        angle_roll = angle_roll_acc;    //Set the gyro roll angle equal to the accelerometer roll angle
        set_gyro_angles = true;         //Set the IMU started flag
    }
    
    // To dampen the pitch and roll angles a complementary filter is used

    angle_pitch_output = angle_pitch_output * 0.9 + angle_pitch * 0.1;  // Take 90% of the output pitch value and add 10% of the raw pitch value
    angle_roll_output = angle_roll_output * 0.9 + angle_roll * 0.1;     // Take 90% of the output roll value and add 10% of the raw roll value 
    
    // Print to Serial Monitor   
    //Serial.print(" | Angle  = "); Serial.println(angle_pitch_output);
}