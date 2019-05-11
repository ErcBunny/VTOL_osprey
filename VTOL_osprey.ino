//macros
#define DEBUG

#define TILT_SERVO 9

#define CH440_ENA 12
#define CH440_SWITCH 13

#define COPTER 0
#define PLANE 1
#define COPTER_TO_PLANE 0
#define PLANE_TO_COPTER 1
#define NONE 2

#define SBUS_PERIOD 14
#define SBUS_MIN 306
#define SBUS_MAX 1694
#define DATA_MIN 1000
#define DATA_MAX 2000
#define CHANNEL_MODEL 8
#define CHANNEL_APM_MODE 9
#define CHANNEL_DEBUG 10

#define CONTROL_PERIOD 14

//headers
#include "SBUS.h"
#include "MPU6050.h"
#include "MsTimer2.h"
#include "Wire.h"
#include "Servo.h"

//objects
SBUS rx(Serial1);
SBUS copter(Serial2);
SBUS plane(Serial3);

Servo tiltServo;

//global variables
float channel_rx[16];
float channel_copter[16];
float channel_plane[16];
bool failSafe;
bool lostFrame;
int model;
int transition;

//channel commands
const float channel_copter_forward[16];
const float channel_plane_cruise[16];

//main structure
void setup()
{
#ifdef DEBUG
    Serial.begin(115200);
#endif
    //setup sbus coms
    rx.setEndPoints(2, 182, 1694);
    rx.setEndPoints(9, 292, 1692);
    rx.begin();
    copter.setEndPoints(2, 182, 1694);
    copter.setEndPoints(9, 292, 1692);
    copter.begin();
    plane.setEndPoints(2, 182, 1694);
    plane.setEndPoints(9, 292, 1692);
    plane.begin();
    //setup I/O pins
    pinMode(CH440_ENA, OUTPUT);
    pinMode(CH440_SWITCH, OUTPUT);
    digitalWrite(CH440_ENA, 0);
    //init servo library
    tiltServo.attach(TILT_SERVO);
    //init timers
    MsTimer2::set(CONTROL_PERIOD, control);
    MsTimer2::start();
}

void loop()
{
    readSbus();
    sendSbus();
    delay(SBUS_PERIOD);
#ifdef DEBUG
    for (int i = 0; i <= 15; i++)
    {
        Serial.print(channel_copter[i]);
        Serial.print("\t");
    }
    Serial.println();
    Serial.print(transition);
    Serial.print("\t");
    Serial.println(model);
#endif
}

void control()
{
    if (transition == NONE)
    {
        if (model == COPTER)
        {
            memcpy(channel_copter, channel_rx, sizeof(float) * 16);
            //TODO: memcpy throttle and arm channel
            digitalWrite(CH440_SWITCH, COPTER);
        }
        else if (model == PLANE)
        {
            memcpy(channel_copter, channel_rx, sizeof(float) * 16);
            //TODO: memcpy throttle and arm channel
            tiltServo.write(90 * (1 + channel_rx[CHANNEL_DEBUG]));
            digitalWrite(CH440_SWITCH, PLANE);
        }
    }
    if (transition == COPTER_TO_PLANE)
    {
        bool skipTrans = false;
        MsTimer2::stop();
        for (int i = 0; i < 100; i++)
        {
            readSbus();
            if (transition == PLANE_TO_COPTER)
            {
                skipTrans = true;
                break;
            }
            memcpy(channel_copter, channel_copter_forward, sizeof(float) * 16);
            //TODO: modify throttle value for the other
            sendSbus();
        }
        if (!skipTrans)
        {
            for (int i = 90; i > 0; i-=10)
            {
                digitalWrite(CH440_SWITCH, PLANE);
                tiltServo.write(i);
                readSbus();
                if (transition == PLANE_TO_COPTER)
                {
                    tiltServo.write(90);
                    break;
                }
                memcpy(channel_plane, channel_rx, sizeof(float) * 16);
                //TODO: throttle and arm for the other
                sendSbus();
            }
        }
        transition = NONE;
        MsTimer2::start();
    }
    if (transition == PLANE_TO_COPTER)
    {
        MsTimer2::stop();
        transition = NONE;
        MsTimer2::start();
    }
}

//component functions
/*
    void readSbus(void):
    read channel values from receiver and store them in channel_rx
    decide whether it is a transition or not and tell the model is copter or plane
*/
//transition value is constantly overwritten (problem)
void readSbus()
{
    static float previousModelValue = 0;
    rx.readCal(channel_rx, &failSafe, &lostFrame);
    if (abs(channel_rx[CHANNEL_MODEL] - previousModelValue) > 1)
    {
        if (channel_rx[CHANNEL_MODEL] < 0)
        {
            transition = PLANE_TO_COPTER;
            model = COPTER;
        }
        else
        {
            transition = COPTER_TO_PLANE;
            model = PLANE;
        }
    }
    else
    {
        if (channel_rx[CHANNEL_MODEL] < 0)
        {
            model = COPTER;
        }
        else
        {
            model = PLANE;
        }
    }
    previousModelValue = channel_rx[CHANNEL_MODEL];
}

/*
    void sendSbus(void):
    simply send packets to different flight controllers
    cannot be put into the function control
*/
void sendSbus()
{
    copter.writeCal(channel_copter);
    plane.writeCal(channel_plane);
}
