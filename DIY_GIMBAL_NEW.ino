#include "I2Cdev.h"
#include <Servo.h>
#include <Wire.h>
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;
Servo SvRoll, SvPitch, SvYaw, Stest;

#define OUTPUT_READABLE_YAWPITCHROLL

bool dmpReady = false; 
uint8_t mpuIntStatus;  
uint8_t devStatus;     
uint16_t packetSize;   
uint16_t fifoCount;     
uint8_t fifoBuffer[64]; 

Quaternion q;          
VectorInt16 aa;         
VectorInt16 aaReal;     
VectorInt16 aaWorld;    
VectorFloat gravity;    
float euler[3];        
float ypr[3];          

int SdriveRoll, SdrivePitch, SdriveYaw;
int RollOffset = 0, PitchOffset = -2, YawOffset = 0;
int YawInitial = 0, YawSetInit = 0;


volatile bool mpuInterrupt = false;    
void dmpDataReady() 
{
    mpuInterrupt = true;
}

void setup() 
{
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24;
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    Serial.begin(115200);
    Serial.println(F("Initializing Gyro sensor devices..."));
    mpu.initialize();
 
    Serial.println(F("Gyro device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); 

    
    if (devStatus == 0) {
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);
        
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;
    
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
       
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    SvRoll.attach(3);
    SvPitch.attach(6);
    SvYaw.attach(9);

    SvRoll.write(80);
    SvPitch.write(120);
    SvYaw.write(90);

}

void loop() 
{
  
    if (!dmpReady) return;

    while (!mpuInterrupt && fifoCount < packetSize) 
    {
      //Do something    
    }

    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    fifoCount = mpu.getFIFOCount();

    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    } else if (mpuIntStatus & 0x02) {
    
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            
            while(YawSetInit = 0)
            {
                YawInitial = int(ypr[0] * 180/M_PI);
                YawSetInit = 1; 
            }
            
            SdriveRoll = int(ypr[2] * 180/M_PI);
            SdrivePitch = int(ypr[1] * 180/M_PI);
            SdriveYaw = (int(ypr[0] * 180/M_PI)) - YawInitial;
            
            SvRoll.write(1.00*(80-SdriveRoll+RollOffset));
            SvPitch.write(1.00*(120+SdrivePitch+PitchOffset));
           

            Serial.println(SdriveRoll);
            Serial.print("\t");
            Serial.print(SdrivePitch);
            Serial.print("\t");
            Serial.print(SdriveYaw);
            Serial.print("\n");
  
        #endif
    }
}
