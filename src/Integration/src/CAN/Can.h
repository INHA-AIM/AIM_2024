#ifndef CAN_H
#define CAN_H

#include "Global/global.h"

void CanProcessing();

void ReadCan();

void ReadPCan();

void ReadMsgEPS(uint8_t* msg);

void ReadMsgACC(uint8_t* msg);

void ReadMsgWS(uint8_t* msg);

void ReadMsgSTAT(uint8_t* msg);

void ReadMsgSIG(uint8_t* msg);

void ReadMsgPITZONE(uint8_t* msg);

void MsgRequest(uint8_t* msg);

void MsgControl(uint8_t* msg);

void MsgREPAIR(uint8_t* msg);

void WriteCan();

void WritePCan();

void CANParser_Callback(const morai_msgs::EgoVehicleStatus::ConstPtr &st_Msg);

void CANParser_ROS(ros::NodeHandle *pst_NodeHandle);

void *CANParserWrapper(void *p_Arg);

#endif
