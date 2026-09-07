#include "Can.h"

extern pthread_mutex_t st_ControlToCAN;
extern pthread_cond_t st_DataTemporalcond;
extern CONTROL_DATA_t st_ControlData;
extern SENSOR_DATA_t st_SensorData;
extern DEAD_RECKONING_DATA_t st_DeadReckoningData;
extern CAN_DATA_t st_CANData;
extern LOGIC_HZ_t st_LogicHz;

extern bool b_Running;
extern bool b_CANMODE;
extern int32_t s32_HZContolNum;
extern uint64_t u64_StartLabTime;


extern bool goActive , stopActive , pitStopActive , slowOnActive , slowOffActive ;
extern bool Go , Stop , PitStop , SlowOn , SlowOff ;


bool b_CANMode = true;

void CanProcessing()
{
    auto u64_StartTime_ms = std::chrono::high_resolution_clock::now();

    auto goStartTime = std::chrono::steady_clock::now();
    auto stopStartTime = std::chrono::steady_clock::now();
    auto pitStopStartTime = std::chrono::steady_clock::now();
    auto slowOnStartTime = std::chrono::steady_clock::now();
    auto slowOffStartTime = std::chrono::steady_clock::now();


    while (b_Running)
    {
        st_CANData.u64_CanSync++;

        float32_t f32_TargetSteer_rad;
        float32_t f32_TargetSteer_deg;
        float32_t f32_TargetAccel;
        pthread_mutex_lock(&st_ControlToCAN);
        f32_TargetSteer_rad = st_ControlData.f32_TargetSteer_rad * 12.9;
        f32_TargetSteer_deg = st_ControlData.f32_TargetSteer_deg * 12.9;
        f32_TargetAccel = st_ControlData.f32_TargetThrottle;
        pthread_mutex_unlock(&st_ControlToCAN);

        st_CANData.f32_TargetSteering = -f32_TargetSteer_deg;
        st_CANData.f32_TargetSpeed = f32_TargetAccel;


        if (b_CANMode)
        {
            ReadCan();
        }        
        else
        {
            ReadPCan();
        }        

        st_CANData.u8_Alive_Cnt++;

        auto u64_EndTime_ms = std::chrono::high_resolution_clock::now();
        std::chrono::duration<float64_t, std::micro> elapsed_ms = u64_EndTime_ms - u64_StartTime_ms;
        if (elapsed_ms.count() > 49999)
        {
            if (b_CANMode)
            {
                WriteCan();
            }        
            else
            {
                WritePCan();
            }

            // u8_SIG_GO 로직
            if (Go == true && !goActive)
            {
                goActive = true;
                goStartTime = std::chrono::steady_clock::now();
            }
            if (goActive)
            {
                auto goElapsedTime = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - goStartTime).count();
                st_CANData.u8_SIG_GO = 1;

                if (goElapsedTime >= 5)
                {
                    st_CANData.u8_SIG_GO = 0;
                    goActive = false;
                    Go =false;
                }
            }

            // u8_SIG_STOP 로직
            if (Stop == true && !stopActive)
            {
                stopActive = true;
                stopStartTime = std::chrono::steady_clock::now();
            }
            if (stopActive)
            {
                auto stopElapsedTime = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - stopStartTime).count();
                st_CANData.u8_SIG_STOP = 1;

                if (stopElapsedTime >= 5)
                {
                    st_CANData.u8_SIG_STOP = 0;
                    stopActive = false;
                    Stop =false;
                }
            }

            // u8_SIG_PIT_STOP 로직
            if (PitStop == true && !pitStopActive)
            {
                pitStopActive = true;
                pitStopStartTime = std::chrono::steady_clock::now();
            }
            if (pitStopActive)
            {
                auto pitStopElapsedTime = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - pitStopStartTime).count();
                st_CANData.u8_SIG_PIT_STOP = 1;

                if (pitStopElapsedTime >= 5)
                {
                    st_CANData.u8_SIG_PIT_STOP = 0;
                    pitStopActive = false;
                    PitStop = false;
                }
            }

            // u8_SIG_SLOW_ON 로직
            if (SlowOn == true && !slowOnActive)
            {
                slowOnActive = true;
                slowOnStartTime = std::chrono::steady_clock::now();
            }
            if (slowOnActive)
            {
                auto slowOnElapsedTime = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - slowOnStartTime).count();
                st_CANData.u8_SIG_SLOW_ON = 1;

                if (slowOnElapsedTime >= 5)
                {
                    st_CANData.u8_SIG_SLOW_ON = 0;
                    slowOnActive = false;
                    SlowOn = false;
                }
            }

            // u8_SIG_SLOW_OFF 로직
            if (SlowOff == true && !slowOffActive)
            {
                slowOffActive = true;
                slowOffStartTime = std::chrono::steady_clock::now();
            }
            if (slowOffActive)
            {
                auto slowOffElapsedTime = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - slowOffStartTime).count();
                st_CANData.u8_SIG_SLOW_OFF = 1;

                if (slowOffElapsedTime >= 5)
                {
                    st_CANData.u8_SIG_SLOW_OFF = 0;
                    slowOffActive = false;
                    SlowOff = false;
                }
            }

            pthread_mutex_lock(&st_SensorData.st_MutexCAN);
            st_SensorData.st_RawCAN.u64_Timestamp = getMillisecond();
            st_SensorData.st_RawCAN.s32_Num = sizeof(CAN_DATA_t);
            memcpy(st_SensorData.st_RawCAN.arc_Buffer, &st_CANData, sizeof(CAN_DATA_t));
            pthread_mutex_unlock(&st_SensorData.st_MutexCAN);

            pthread_cond_signal(&st_DataTemporalcond);
            float64_t hz = 1000000.0 / elapsed_ms.count();
            u64_StartTime_ms = u64_EndTime_ms;
        }
    }
}

void ReadCan()
{

    bool b_0x710 = false;
    bool b_0x711 = false;
    bool b_0x712 = false;
    bool b_0x713 = false;

    bool b_0x124 = false;
    bool b_0x125 = false;

    st_CANData.st_Stat = canReadWait(st_CANData.st_Handle,
                                     &(st_CANData.ID), &st_CANData.msg,
                                     &(st_CANData.u32_DLC), &(st_CANData.u32_Flag),
                                     &(st_CANData.u32_Time), 0xFFFFFFFF);

    // 타임아웃 파라미터로
    switch (st_CANData.ID)
    {
    case 0x710: // 1808
        ReadMsgEPS(st_CANData.msg);
        b_0x710 = true;
        break;

    case 0x711: // 1809
        ReadMsgACC(st_CANData.msg);
        b_0x711 = true;
        break;

    case 0x712: // //1810
        ReadMsgWS(st_CANData.msg);
        b_0x712 = true;
        break;
    case 0x713:
        ReadMsgSTAT(st_CANData.msg);
        b_0x713 = true;
        break;
    case 0x124:
        ReadMsgSIG(st_CANData.msg);
        b_0x124 = true;
        break;
    case 0x125:
        ReadMsgPITZONE(st_CANData.msg);
        b_0x125 = true;
        break;
    default:
        break;
    }
}

void ReadPCan()
{
    st_CANData.st_PStat = CAN_Read(PCAN_USBBUS1, &st_CANData.message, &(st_CANData.timestamp));
    switch (st_CANData.message.ID)
    {
    case 0x710: // 1808
        ReadMsgEPS(st_CANData.message.DATA);
        break;

    case 0x711: // 1809
        ReadMsgACC(st_CANData.message.DATA);
        break;

    case 0x712: // //1810
        ReadMsgWS(st_CANData.message.DATA);
        break;
    case 0x713:
        ReadMsgSTAT(st_CANData.message.DATA);
        break;
    case 0x124:
        ReadMsgSIG(st_CANData.message.DATA);
        break;
    case 0x125:
        ReadMsgPITZONE(st_CANData.message.DATA);
        break;
    default:
        break;
    }

}

void ReadMsgEPS(uint8_t* msg)
{

    st_CANData.u8_EPS_En_Status = (msg[0] & 0b00000001);
    st_CANData.u8_EPS_Control_Board_Status = (msg[0] & 0b00001110) >> 1;
    st_CANData.u8_EPS_USER_CAN_ERR = (msg[0] & 0b00010000) >> 4;
    st_CANData.u8_EPS_ERR = (msg[0] & 0b00100000) >> 5;
    st_CANData.u8_EPS_Veh_CAN_ERR = (msg[0] & 0b01000000) >> 6;
    st_CANData.u8_EPS_SAS_ERR = (msg[0] & 0b10000000) >> 7;
    st_CANData.u8_EPS_Control_Status = (msg[1] & 0b00001111);
    st_CANData.u8_Override_Ignore_Status = (msg[1] & 0b00010000) >> 4;
    st_CANData.u8_Override_Status = (msg[1] & 0b00100000) >> 5;
    st_CANData.f32_StrAng = 0.1 * int16_t((msg[3] << 8) | msg[2]);
    st_CANData.f32_Str_Drv_Tq = 0.01 * (int16_t(msg[4] | (msg[5] & 0b00001111)) - 2048);
    st_CANData.f32_Str_Out_Tq = 0.1 * (int16_t((msg[5] & 0b11110000) >> 4) | (msg[6] << 4) - 2048);
    st_CANData.u8_EPS_Alive_Cnt = msg[7];

    st_CANData.f32_SteerAngle_deg = st_CANData.f32_StrAng;
    st_CANData.f32_SteerAngle_rad = deg2rad(st_CANData.f32_StrAng);
}

void ReadMsgACC(uint8_t* msg)
{

    st_CANData.u8_ACC_En_Status = (msg[0] & 0b00000001);
    st_CANData.u8_ACC_Control_Board_Status = (msg[0] & 0b00001110) >> 1;
    st_CANData.u8_ACC_USER_CAN_ERR = (msg[0] & 0b00010000) >> 4;
    st_CANData.u8_ACC_Veh_ERR = (msg[0] & 0b01000000) >> 6;
    st_CANData.u8_ACC_ERR = (msg[0] & 0b10000000) >> 7;
    st_CANData.u8_ACC_Control_Status = (msg[1] & 0b00001111);
    st_CANData.u8_VS = msg[2];
    st_CANData.f32_LONG_ACCEL = 0.01 * (int16_t(msg[4] | ((msg[5] & 0b00000111) << 8)) - 1023);
    st_CANData.u8_Turn_Right_En = (msg[6] & 0b00000001);
    st_CANData.u8_Hazard_En = (msg[6] & 0b00000010) >> 1;
    st_CANData.u8_Turn_Left_En = (msg[6] & 0b00000100) >> 2;
    st_CANData.u8_G_SEL_DISP = (msg[6] & 0b11110000) >> 4;
    st_CANData.u8_ACC_Alive_Cnt = msg[7];

    st_CANData.f32_Speed_kph = st_CANData.u8_VS;
    st_CANData.f32_Speed_m_s = st_CANData.u8_VS / 3.6;
}
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 
void ReadMsgWS(uint8_t* msg)
{
    st_CANData.f32_WHEEL_SPD_FL = 0.03152 * int16_t(msg[0] | (msg[1] << 8));
    st_CANData.f32_WHEEL_SPD_FR = 0.03152 * int16_t(msg[2] | (msg[3] << 8));
    st_CANData.f32_WHEEL_SPD_RL = 0.03152 * int16_t(msg[4] | (msg[5] << 8));
    st_CANData.f32_WHEEL_SPD_RR = 0.03152 * int16_t(msg[6] | (msg[7] << 8));
}

void ReadMsgSTAT(uint8_t* msg)
{
    st_CANData.f32_LAT_ACCEL = 0.01 * (int16_t(msg[0] | (msg[1] << 8)) - 1023);
    st_CANData.f32_Long_ACCEL = 0.01 * (int16_t(msg[2] | (msg[3] << 8)) - 1023);


    st_CANData.f32_YAW_RATE = 0.01 * (int16_t(msg[4] | (msg[5] << 8)) - 4095);
    st_CANData.f32_BRK_CYLINDER = 0.1 * (int16_t(msg[6] | (msg[7] << 8)) - 1023);
}

void ReadMsgSIG(uint8_t* msg)
{
    st_CANData.u8_SIG_GO = (msg[0] & 0b10000000) >> 7;
    st_CANData.u8_SIG_STOP = (msg[0] & 0b01000000) >> 6;
    st_CANData.u8_SIG_PIT_STOP = (msg[0] & 0b00100000) >> 5;
    st_CANData.u8_SIG_SLOW_ON = (msg[0] & 0b00010000) >> 4;
    st_CANData.u8_SIG_SLOW_OFF = (msg[0] & 0b00001000) >> 3;
}

void ReadMsgPITZONE(uint8_t* msg)
{
    st_CANData.f32_PITZONE_LAT = 0.0000001 * (msg[0] | (msg[1] << 8) | (msg[2] << 16) | (msg[3] << 24));
    st_CANData.f32_PITZONE_LONG = 0.0000001 * (msg[4] | (msg[5] << 8) | (msg[6] << 16) | (msg[7] << 24));

}

void MsgRequest()
{
    st_CANData.Request_Msg[0] = (st_CANData.u8_EPS_En & 0b00000001) | ((st_CANData.u8_EPS_Override_ignore & 0b00000001) << 2);
    st_CANData.Request_Msg[1] = st_CANData.u8_EPS_Speed;
    st_CANData.Request_Msg[2] = (st_CANData.u8_ACC_En & 0b00000001) | ((st_CANData.u8_AEB_En & 0b00000001) << 6);
    st_CANData.Request_Msg[5] = (st_CANData.u8_Hazard & 0b00000001) | ((st_CANData.u8_Turn_sig_right & 0b00000001) << 1) | ((st_CANData.u8_Turn_sig_left & 0b00000001) << 2);
    st_CANData.Request_Msg[6] = st_CANData.u8_AEB_decel_value;
    st_CANData.Request_Msg[7] = st_CANData.u8_Alive_Cnt;
}

void MsgControl()
{
    st_CANData.s16_EPS_Cmd = uint16_t(10 * st_CANData.f32_TargetSteering);
    st_CANData.s16_ACC_Cmd = uint16_t((st_CANData.f32_TargetSpeed + 10.23) * 100);

    st_CANData.Control_Msg[0] = st_CANData.s16_EPS_Cmd;
    st_CANData.Control_Msg[1] = uint8_t(st_CANData.s16_EPS_Cmd >> 8);
    st_CANData.Control_Msg[3] = st_CANData.s16_ACC_Cmd;
    st_CANData.Control_Msg[4] = uint8_t(st_CANData.s16_ACC_Cmd >> 8);
}

void MsgREPAIR()
{
    st_CANData.REPAIR_Msg[0] = st_CANData.u8_SIG_REPAIR;
}

void WriteCan()
{
    MsgRequest();
    st_CANData.st_Stat = canWrite(st_CANData.st_Handle, 0x156, st_CANData.Request_Msg, 8, canMSG_STD);
    MsgControl();
    st_CANData.st_Stat = canWrite(st_CANData.st_Handle, 0x157, st_CANData.Control_Msg, 8, canMSG_STD);
    MsgREPAIR();
    st_CANData.st_Stat = canWrite(st_CANData.st_Handle, 0x126, st_CANData.REPAIR_Msg, 8, canMSG_STD);
}

void WritePCan()
{
    st_CANData.PRequest_Msg.ID = 0x156;
    st_CANData.PRequest_Msg.MSGTYPE = PCAN_MESSAGE_STANDARD;
    st_CANData.PRequest_Msg.LEN = 8;
    MsgRequest();
    memcpy(st_CANData.PRequest_Msg.DATA, st_CANData.Request_Msg, sizeof(st_CANData.Request_Msg));
    st_CANData.st_PStat = CAN_Write(PCAN_USBBUS1, &st_CANData.PRequest_Msg);

    st_CANData.PControl_Msg.ID = 0x157;
    st_CANData.PControl_Msg.MSGTYPE = PCAN_MESSAGE_STANDARD;
    st_CANData.PControl_Msg.LEN = 8;
    MsgControl();
    memcpy(st_CANData.PControl_Msg.DATA, st_CANData.Control_Msg, sizeof(st_CANData.Control_Msg));
    st_CANData.st_PStat = CAN_Write(PCAN_USBBUS1, &st_CANData.PControl_Msg);

    st_CANData.PREPAIR_Msg.ID = 0x157;
    st_CANData.PREPAIR_Msg.MSGTYPE = PCAN_MESSAGE_STANDARD;
    st_CANData.PREPAIR_Msg.LEN = 8;
    MsgREPAIR();
    memcpy(st_CANData.PREPAIR_Msg.DATA, st_CANData.REPAIR_Msg, sizeof(st_CANData.REPAIR_Msg));
    st_CANData.st_PStat = CAN_Write(PCAN_USBBUS1, &st_CANData.PREPAIR_Msg);
}

void CANParser_Callback(const morai_msgs::EgoVehicleStatus::ConstPtr &st_Msg)
{
    CAN_DATA_t st_CAN = {0};
    float32_t f32_Speed_m_s = getDistance3d(0, 0, 0, st_Msg->velocity.x, st_Msg->velocity.y, st_Msg->velocity.z);
    float32_t f32_Speed_kph = ms2kph(f32_Speed_m_s);
    float32_t f32_SteerAngle_deg = st_Msg->wheel_angle;
    float32_t f32_SteerAngle_rad = deg2rad(st_Msg->wheel_angle);
    float32_t f32_Heading_deg = st_Msg->heading;
    float32_t f32_Heading_rad = deg2rad(st_Msg->heading);

    st_CAN.f32_Speed_m_s = f32_Speed_m_s;
    st_CAN.f32_Speed_kph = f32_Speed_kph;
    st_CAN.f32_SteerAngle_rad = f32_SteerAngle_rad;
    st_CAN.f32_SteerAngle_deg = f32_SteerAngle_deg;
    st_CAN.f32_Heading_rad = f32_Heading_rad;
    st_CAN.f32_Heading_deg = f32_Heading_deg;

    st_CAN.f32_Long_ACCEL = st_Msg->acceleration.x;
    st_CAN.f32_LAT_ACCEL = st_Msg->acceleration.y;

    pthread_mutex_lock(&st_SensorData.st_MutexCAN);
    st_SensorData.st_RawCAN.u64_Timestamp = getMillisecond();
    st_SensorData.st_RawCAN.s32_Num = sizeof(CAN_DATA_t);
    memcpy(st_SensorData.st_RawCAN.arc_Buffer, &st_CAN, sizeof(CAN_DATA_t));
    pthread_mutex_unlock(&st_SensorData.st_MutexCAN);
}

void CANParser_ROS(ros::NodeHandle *pst_NodeHandle)
{
    ros::Subscriber st_SubscribeCAN = pst_NodeHandle->subscribe<morai_msgs::EgoVehicleStatus>("/Ego_topic", 1, CANParser_Callback);
    ros::Publisher st_PublishCAN = pst_NodeHandle->advertise<morai_msgs::CtrlCmd>("ctrl_cmd", 1);
    morai_msgs::CtrlCmd msg;

    uint64_t u64_StartTime_ms = 0;
    auto start_time = std::chrono::steady_clock::now();
    auto end_time = start_time;

    auto goStartTime = std::chrono::steady_clock::now();
    auto stopStartTime = std::chrono::steady_clock::now();
    auto pitStopStartTime = std::chrono::steady_clock::now();
    auto slowOnStartTime = std::chrono::steady_clock::now();
    auto slowOffStartTime = std::chrono::steady_clock::now();
    
    while (b_Running)
    {
        ros::spinOnce();

        // u8_SIG_GO 로직
        if (Go == true && !goActive)
        {
            goActive = true;
            goStartTime = std::chrono::steady_clock::now();
        }
        if (goActive)
        {
            auto goElapsedTime = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - goStartTime).count();
            st_CANData.u8_SIG_GO = 1;

            if (goElapsedTime >= 5)
            {
                st_CANData.u8_SIG_GO = 0;
                goActive = false;
                Go =false;
            }
        }

        // u8_SIG_STOP 로직
        if (Stop == true && !stopActive)
        {
            stopActive = true;
            stopStartTime = std::chrono::steady_clock::now();
        }
        if (stopActive)
        {
            auto stopElapsedTime = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - stopStartTime).count();
            st_CANData.u8_SIG_STOP = 1;

            if (stopElapsedTime >= 5)
            {
                st_CANData.u8_SIG_STOP = 0;
                stopActive = false;
                Stop =false;
            }
        }

        // u8_SIG_PIT_STOP 로직
        if (PitStop == true && !pitStopActive)
        {
            pitStopActive = true;
            pitStopStartTime = std::chrono::steady_clock::now();
        }
        if (pitStopActive)
        {
            auto pitStopElapsedTime = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - pitStopStartTime).count();
            st_CANData.u8_SIG_PIT_STOP = 1;

            if (pitStopElapsedTime >= 5)
            {
                st_CANData.u8_SIG_PIT_STOP = 0;
                pitStopActive = false;
                PitStop = false;
            }
        }

        // u8_SIG_SLOW_ON 로직
        if (SlowOn == true && !slowOnActive)
        {
            slowOnActive = true;
            slowOnStartTime = std::chrono::steady_clock::now();
        }
        if (slowOnActive)
        {
            auto slowOnElapsedTime = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - slowOnStartTime).count();
            st_CANData.u8_SIG_SLOW_ON = 1;

            if (slowOnElapsedTime >= 5)
            {
                st_CANData.u8_SIG_SLOW_ON = 0;
                slowOnActive = false;
                SlowOn = false;
            }
        }

        // u8_SIG_SLOW_OFF 로직
        if (SlowOff == true && !slowOffActive)
        {
            slowOffActive = true;
            slowOffStartTime = std::chrono::steady_clock::now();
        }
        if (slowOffActive)
        {
            auto slowOffElapsedTime = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - slowOffStartTime).count();
            st_CANData.u8_SIG_SLOW_OFF = 1;

            if (slowOffElapsedTime >= 5)
            {
                st_CANData.u8_SIG_SLOW_OFF = 0;
                slowOffActive = false;
                SlowOff = false;
            }
        }

        u64_StartTime_ms = getMillisecond();
        msg.longlCmdType = 1;
        if (st_ControlData.f32_TargetThrottle > 0)
        {
            msg.accel = st_ControlData.f32_TargetThrottle / 2.f;
            msg.brake = 0;
        }
        else
        {
            msg.accel = 0;
            msg.brake = fmaxf( -0.3f, -st_ControlData.f32_TargetThrottle / 3.0f);
        }

        msg.steering = -st_ControlData.f32_TargetSteer_rad;
        st_PublishCAN.publish(msg);

        end_time = std::chrono::steady_clock::now();
        auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();

        if (elapsed_time >= 100)
        {
            pthread_cond_signal(&st_DataTemporalcond);
            start_time = end_time; // 시작 시간을 갱신
            st_LogicHz.u64_CANHz = getMillisecond() -u64_StartTime_ms;
        }
    }
}
void *CANParserWrapper(void *p_Arg)
{
    ros::NodeHandle *pst_NodeHandle = (ros::NodeHandle *)p_Arg;
    int32_t s32_CANPort = 0;
    int32_t s32_CANMode = 0;
    uint64_t u64_StartTime_ms = 0;

    std::string s_CANAddress = "";
    CAN_DATA_t st_TemporalCanData;
    YAML::Node st_Config = YAML::LoadFile("configuration.yaml");

    s32_CANMode = st_Config["CAN_Mode"].as<int32_t>();
    s_CANAddress = st_Config["CAN_Address"].as<std::string>();
    s32_CANPort = st_Config["CAN_Port"].as<int32_t>();

    if (s32_CANMode == c_PARSING_UDP)
    {
        while (b_Running)
        {
            u64_StartTime_ms = getMillisecond();
            usleep(50000);
            pthread_cond_signal(&st_DataTemporalcond);
            st_LogicHz.u64_CANHz = getMillisecond() - u64_StartTime_ms;
        }
    }
    else if (s32_CANMode == c_PARSING_ROS)
    {
        CANParser_ROS(pst_NodeHandle);
    }
    else if (s32_CANMode == c_PARSING_SERIAL)
    {
        ;
    }
    else if (s32_CANMode == 3)
    {
        b_CANMode = true;
        canInitializeLibrary();
        st_CANData.st_Handle = canOpenChannel(0, canOPEN_EXCLUSIVE);
        if (st_CANData.st_Handle < 0)
        {
            printf("NO CHANNEL\n");
        }
        st_CANData.st_Stat = canSetBusParams(st_CANData.st_Handle, canBITRATE_500K, 0, 0, 0, 0, 0);
        st_CANData.st_Stat = canBusOn(st_CANData.st_Handle);
        u64_StartTime_ms = getMillisecond();
        CanProcessing();
        st_LogicHz.u64_CANHz = getMillisecond() - u64_StartTime_ms;
    }
    else if (s32_CANMode == 4)
    {
        b_CANMode = false;
        st_CANData.st_PStat = CAN_Initialize(PCAN_USBBUS1, PCAN_BAUD_500K);
        if (st_CANData.st_PStat != PCAN_ERROR_OK)
        {
            // An error occurred, get a text describing the error and show it
            CAN_GetErrorText(st_CANData.st_PStat, 0, st_CANData.strMsg);
            std::wcout<<(st_CANData.strMsg);
        }
        else
        {
            std::wcout<<("PCAN-USB (Ch-1) was initialized");
        }

        u64_StartTime_ms = getMillisecond();
        CanProcessing();
        st_LogicHz.u64_CANHz = getMillisecond() - u64_StartTime_ms;
    }
}
