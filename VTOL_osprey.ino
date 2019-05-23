//headers
#include "./mavlink/ardupilotmega/mavlink.h"
#include "config.h"
#include "SBUS.h"
#include "Servo.h"
#include "Chrono.h"

//objects
SBUS rx_f4(Serial1);
SBUS apm(Serial2);

Chrono debugIntvl;
Chrono sbusIntvl_f4;
Chrono sbusIntvl_apm;
Chrono tempCounter;

Servo tiltServo;

//global variables
float channel_rx[16];
float channel_f4[16];
float channel_apm[16];
bool failSafe;
bool lostFrame;
int model;
int transition;
mavlink_attitude_t attitude;
mavlink_vfr_hud_t hudInfo;

//main structure
void setup()
{
    //setup mavlink and debug serial
    SERIAL_MAVLINK.begin(57600, SERIAL_8N1);
    #ifdef DEBUG
        Serial.begin(115200);
    #endif
    //setup sbus comms
    rx_f4.setEndPoints(2, 182, 1694);
    rx_f4.setEndPoints(9, 292, 1692);
    rx_f4.begin();
    apm.setEndPoints(2, 182, 1694);
    apm.setEndPoints(9, 292, 1692);
    apm.begin();
    //setup I/O pins
    pinMode(CH440_ENA, OUTPUT);
    pinMode(CH440_SWITCH, OUTPUT);
    digitalWrite(CH440_ENA, 0);
    //init servo
    tiltServo.attach(TILT_SERVO);
    //stop Chrono counting in background
    tempCounter.stop();
}

void loop()
{
    readSbus();
    switch (transition)
    {
        case NONE:
            switch (model)
            {
                case COPTER:
                    memcpy(channel_f4, channel_rx, sizeof(float) * 16);
                    //TODO: tell apm not to move servos
                    digitalWrite(CH440_SWITCH, COPTER);
                    break;
                case PLANE:
                    memcpy(channel_apm, channel_rx, sizeof(float) * 16);
                    tiltServo.write(90 * (1 + channel_rx[CHANNEL_DEBUG]));
                    digitalWrite(CH440_SWITCH, PLANE);
                    break;
                default:
                    break;
            }
            sendSbus();
            break;
        case COPTER_TO_PLANE:
            transition_copter_to_plane();
            #if defined DEBUG
                Serial.println("changed to plane");
            #endif
            transition = NONE;
            break;
        case PLANE_TO_COPTER:
            transition_plane_to_copter();
            #if defined DEBUG
                Serial.println("changed to copter");
            #endif
            transition = NONE;
            break;
        default:
            break;
    }
    #if defined DEBUG
        printDebug();
    #endif
}

//functions
void readSbus()
{
    static float previousModelValue = 0;
    rx_f4.readCal(channel_rx, &failSafe, &lostFrame);
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

void sendSbus()
{
    if (sbusIntvl_f4.hasPassed(14))
    {
        sbusIntvl_f4.restart();
        rx_f4.writeCal(channel_f4);
    }
    if (sbusIntvl_apm.hasPassed(14))
    {
        sbusIntvl_apm.restart();
        apm.writeCal(channel_apm);
    }
}

void readMavlink()
{
    //init
    const int num_hbs = 60;                            // # of heartbeats to wait before activating STREAMS from APM. 60 = one minute.
    static unsigned long previousMillisMAVLink = 0;    // will store last time MAVLink was transmitted and listened
    static unsigned long next_interval_MAVLink = 1000; // next interval to count
    static int num_hbs_past = num_hbs;
    unsigned long currentMillisMAVLink = millis();
    //heartbeat
    mav_heartbeat_pack();
    //request data
    if (currentMillisMAVLink - previousMillisMAVLink >= next_interval_MAVLink)
    {
        previousMillisMAVLink = currentMillisMAVLink;
        num_hbs_past++;
        if (num_hbs_past >= num_hbs)
        {
            mav_Request_Data();
            num_hbs_past = 0;
        }
    }
    //extract info
    mav_receive();
}

void mav_heartbeat_pack()
{
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    mavlink_msg_heartbeat_pack(sysid, compid, &msg, type, autopilot_type, system_mode, custom_mode, system_state);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    SERIAL_MAVLINK.write(buf, len);
}

void mav_Request_Data()
{
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    mavlink_msg_request_data_stream_pack(sysid, compid, &msg, 1, 0, MAV_DATA_STREAM_ALL, RATE_MSG, 1);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    SERIAL_MAVLINK.write(buf, len);
}

void mav_receive()
{
    mavlink_message_t msg;
    mavlink_status_t status;
    while (SERIAL_MAVLINK.available())
    {
        uint8_t c = SERIAL_MAVLINK.read();
        if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status))
        {
            switch (msg.msgid)
            {
                case MAVLINK_MSG_ID_ATTITUDE:
                    mavlink_msg_attitude_decode(&msg, &attitude);
                    break;
                case MAVLINK_MSG_ID_VFR_HUD:
                    mavlink_msg_vfr_hud_decode(&msg, &hudInfo);
                    break;
            }
            
        }
    }
}

void transition_copter_to_plane()
{
    bool skipTrans = false;
    bool recover = false;
    tempCounter.restart();
    while (!tempCounter.hasPassed(TIME_CONFIRM_TRANS))
    {
        readSbus();
        if (transition == PLANE_TO_COPTER)
        {
            skipTrans = true;
            break;
        }
        memcpy(channel_f4, channel_rx, sizeof(float) * 16);
        //TODO: modify CHANNEL_PITCH to go forward and tell apm not to move servos
        sendSbus();
    }
    tempCounter.stop();
    if (!skipTrans)
    {
        int currentAngle = TILT_MAX;
        int destAngle = TILT_MIN;
        int step = -TILT_STEP;
        tiltServo.write(currentAngle);
        digitalWrite(CH440_SWITCH, PLANE);
        while (currentAngle != destAngle)
        {
            tiltServo.write(currentAngle);
            readSbus();
            switch (transition)
            {
                case PLANE_TO_COPTER:
                    destAngle = TILT_MAX;
                    step = TILT_STEP;
                    break;
                case COPTER_TO_PLANE:
                    destAngle = TILT_MIN;
                    step = -TILT_STEP;
                    break;
                default:
                    break;
            }
            memcpy(channel_apm, channel_rx, sizeof(float) * 16);
            //TODO: details
            sendSbus();
            currentAngle += step;
        }
        if (model == COPTER)
        {
            digitalWrite(CH440_SWITCH, COPTER);
        }
    }
}

void transition_plane_to_copter()
{

}

void printDebug()
{
    if (debugIntvl.hasPassed(100))
    {
        debugIntvl.restart();
        Serial.print(attitude.roll);
        Serial.print("\t");
        Serial.print(attitude.pitch);
        Serial.print("\t");
        Serial.print(attitude.yaw);
        Serial.print("\t");
        Serial.println(hudInfo.alt);
        /*
            for (int i = 0; i <= 15; i++)
            {
                Serial.print(channel_apm[i]);
                Serial.print("\t");
            }
            Serial.println();
            Serial.print(transition);
            Serial.print("\t");
            Serial.println(model);
            */
    }
}