/**********************************************************************
* Copyright (C) 2020  NATORS£¬All Rights Reserved
*
* File name: NTControl.h
* Author   : Nators
* Version  : 1.4.7
*
* This is the software interface to the Nano Positioning System.
* Please refer to the Programmer's Guide for a detailed documentation.
*
* THIS  SOFTWARE, DOCUMENTS, FILES AND INFORMATION ARE PROVIDED 'AS IS'
* WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING,
* BUT  NOT  LIMITED  TO,  THE  IMPLIED  WARRANTIES  OF MERCHANTABILITY,
* FITNESS FOR A PURPOSE, OR THE WARRANTY OF NON-INFRINGEMENT.
* THE  ENTIRE  RISK  ARISING OUT OF USE OR PERFORMANCE OF THIS SOFTWARE
* REMAINS WITH YOU.
* IN  NO  EVENT  SHALL  THE  NATORS  BE  LIABLE  FOR ANY DIRECT,
* INDIRECT, SPECIAL, INCIDENTAL, CONSEQUENTIAL OR OTHER DAMAGES ARISING
* OUT OF THE USE OR INABILITY TO USE THIS SOFTWARE.
**********************************************************************/
#ifndef NPSCONTROL_H
#define NPSCONTROL_H

#ifdef NPSCONTROL_EXPORTS
	#define NPSCONTROL_API __declspec(dllexport)
#else
	#define NPSCONTROL_API __declspec(dllimport)
#endif

#define NPSCONTROL_CC  __cdecl


typedef unsigned int NT_STATUS;
typedef unsigned int NT_INDEX;
typedef unsigned int NT_PACKET_TYPE;

typedef struct NT_packet {
	NT_PACKET_TYPE packetType; // type of packet
	NT_INDEX channelIndex; // source channel
	unsigned int data1; // data field
	signed int data2; // data field
	signed int data3; // data field
	unsigned int data4; // data field
} NT_PACKET;


// function status return values
#define NT_OK                                       0
#define NT_INITIALIZATION_ERROR                     1
#define NT_NOT_INITIALIZED_ERROR                    2
#define NT_NO_SYSTEMS_FOUND_ERROR                   3
#define NT_TOO_MANY_SYSTEMS_ERROR                   4
#define NT_INVALID_SYSTEM_INDEX_ERROR               5
#define NT_INVALID_CHANNEL_INDEX_ERROR              6
#define NT_TRANSMIT_ERROR                           7
#define NT_WRITE_ERROR                              8
#define NT_INVALID_PARAMETER_ERROR                  9
#define NT_READ_ERROR                               10
#define NT_INTERNAL_ERROR                           12
#define NT_WRONG_MODE_ERROR                         13
#define NT_PROTOCOL_ERROR                           14
#define NT_TIMEOUT_ERROR                            15
#define NT_ID_LIST_TOO_SMALL_ERROR                  17
#define NT_SYSTEM_ALREADY_ADDED_ERROR               18
#define NT_WRONG_CHANNEL_TYPE_ERROR                 19
#define NT_CANCELED_ERROR                           20
#define NT_INVALID_SYSTEM_LOCATOR_ERROR             21
#define NT_INPUT_BUFFER_OVERFLOW_ERROR              22
#define NT_QUERYBUFFER_SIZE_ERROR                   23
#define NT_DRIVER_ERROR                             24
#define NT_NO_SENSOR_PRESENT_ERROR                  129
#define NT_AMPLITUDE_TOO_LOW_ERROR                  130
#define NT_AMPLITUDE_TOO_HIGH_ERROR                 131
#define NT_FREQUENCY_TOO_LOW_ERROR                  132
#define NT_FREQUENCY_TOO_HIGH_ERROR                 133
#define NT_SCAN_TARGET_TOO_HIGH_ERROR               135
#define NT_SCAN_SPEED_TOO_LOW_ERROR                 136
#define NT_SCAN_SPEED_TOO_HIGH_ERROR                137
#define NT_SENSOR_DISABLED_ERROR                    140
#define NT_COMMAND_OVERRIDDEN_ERROR                 141
#define NT_END_STOP_REACHED_ERROR                   142
#define NT_WRONG_SENSOR_TYPE_ERROR                  143
#define NT_COULD_NOT_FIND_REF_ERROR                 144
#define NT_WRONG_END_EFFECTOR_TYPE_ERROR            145
#define NT_MOVEMENT_LOCKED_ERROR                    146
#define NT_RANGE_LIMIT_REACHED_ERROR                147
#define NT_PHYSICAL_POSITION_UNKNOWN_ERROR          148
#define NT_OUTPUT_BUFFER_OVERFLOW_ERROR             149
#define NT_COMMAND_NOT_PROCESSABLE_ERROR            150
#define NT_WAITING_FOR_TRIGGER_ERROR                151
#define NT_COMMAND_NOT_TRIGGERABLE_ERROR            152
#define NT_COMMAND_QUEUE_FULL_ERROR                 153
#define NT_INVALID_COMPONENT_ERROR                  154
#define NT_INVALID_SUB_COMPONENT_ERROR              155
#define NT_INVALID_PROPERTY_ERROR                   156
#define NT_PERMISSION_DENIED_ERROR                  157
#define NT_CALIBRATION_FAILED_ERROR                 160
#define NT_UNKNOWN_COMMAND_ERROR                    240
#define NT_OTHER_ERROR                              255

// general definitions
#define NT_UNDEFINED                                0
#define NT_FALSE                                    0
#define NT_TRUE                                     1
#define NT_DISABLED                                 0
#define NT_ENABLED                                  1
#define NT_FALLING_EDGE                             0
#define NT_RISING_EDGE                              1
#define NT_FORWARD                                  0
#define NT_BACKWARD                                 1


// configuration flags for NT_InitDevices
#define NT_SYNCHRONOUS_COMMUNICATION                0
#define NT_ASYNCHRONOUS_COMMUNICATION               1
#define NT_HARDWARE_RESET                           2

// return values from NT_GetInitState
#define NT_INIT_STATE_NONE                          0
#define NT_INIT_STATE_SYNC                          1
#define NT_INIT_STATE_ASYNC                         2

// Hand Control Module modes for NT_SetHCMEnabled
#define NT_HCM_DISABLED                             0
#define NT_HCM_ENABLED                              1

// configuration values for NT_SetAccumulateRelativePositions_X
#define NT_NO_ACCUMULATE_RELATIVE_POSITIONS         0
#define NT_ACCUMULATE_RELATIVE_POSITIONS            1

// configuration values for NT_SetSensorEnabled_X
#define NT_SENSOR_DISABLED                          0
#define NT_SENSOR_ENABLED                           1

// infinite timeout for functions that wait
#define NT_TIMEOUT_INFINITE                         0xFFFFFFFF

// packet types for asynchronous mode
#define NT_NO_PACKET_TYPE                           0
#define NT_ERROR_PACKET_TYPE                        1
#define NT_POSITION_PACKET_TYPE                     2
#define NT_COMPLETED_PACKET_TYPE                    3
#define NT_STATUS_PACKET_TYPE                       4
#define NT_ANGLE_PACKET_TYPE                        5
#define NT_VOLTAGE_LEVEL_PACKET_TYPE                6
#define NT_SENSOR_TYPE_PACKET_TYPE                  7
#define NT_SENSOR_ENABLED_PACKET_TYPE               8
#define NT_END_EFFECTOR_TYPE_PACKET_TYPE            9
#define NT_GRIPPER_OPENING_PACKET_TYPE              10
#define NT_FORCE_PACKET_TYPE                        11
#define NT_MOVE_SPEED_PACKET_TYPE                   12
#define NT_PHYSICAL_POSITION_KNOWN_PACKET_TYPE      13
#define NT_POSITION_LIMIT_PACKET_TYPE               14
#define NT_ANGLE_LIMIT_PACKET_TYPE                  15
#define NT_SAFE_DIRECTION_PACKET_TYPE               16
#define NT_SCALE_PACKET_TYPE                        17
#define NT_MOVE_ACCELERATION_PACKET_TYPE            18
#define NT_CHANNEL_PROPERTY_PACKET_TYPE             19
#define NT_CAPTURE_BUFFER_PACKET_TYPE               20
#define NT_TRIGGERED_PACKET_TYPE                    21
#define NT_INVALID_PACKET_TYPE                      255

// channel status codes
#define NT_STOPPED_STATUS                           0
#define NT_STEPPING_STATUS                          1
#define NT_SCANNING_STATUS                          2
#define NT_HOLDING_STATUS                           3
#define NT_TARGET_STATUS                            4
#define NT_SENSOR_CLOSED_STATUS                     5
#define NT_LIMIT_POSITION_STATUS                    10
#define NT_SHORT_CIRCUIT_STATUS                     11


#ifdef __cplusplus
extern "C" {
#endif
	/***********************************************************************
	General note:
	All functions have a return value of NT_STATUS
	indicating success (NT_OK) or failure of execution. See the above
	definitions for a list of error codes.
	***********************************************************************/

	/************************************************************************
	*************************************************************************
	**                 Section I: Initialization Functions                 **
	*************************************************************************
	************************************************************************/

	NPSCONTROL_API
		NT_STATUS NPSCONTROL_CC NT_GetDLLVersion(unsigned int* version);

	NPSCONTROL_API
		NT_STATUS NPSCONTROL_CC NT_OpenSystem(NT_INDEX* systemIndex, const char* systemLocator, const char* options);

	NPSCONTROL_API
		NT_STATUS NPSCONTROL_CC NT_CloseSystem(NT_INDEX systemIndex);

	NPSCONTROL_API
		NT_STATUS NPSCONTROL_CC NT_FindSystems(const char* options, char* outBuffer, unsigned int* ioBufferSize);

	NPSCONTROL_API
		NT_STATUS NPSCONTROL_CC NT_GetSystemLocator(NT_INDEX systemIndex, char* outBuffer, unsigned int* ioBufferSize);

	NPSCONTROL_API
		NT_STATUS NPSCONTROL_CC NT_SetHCMEnabled(NT_INDEX systemIndex, unsigned int enabled);

	NPSCONTROL_API
		NT_STATUS NPSCONTROL_CC NT_GetNumberOfChannels(NT_INDEX systemIndex, unsigned int* channels);

	/************************************************************************
	*************************************************************************
	**        Section IIa:  Functions for SYNCHRONOUS communication        **
	*************************************************************************
	************************************************************************/
	NPSCONTROL_API
		NT_STATUS NPSCONTROL_CC NT_StepMove_S(NT_INDEX systemIndex, NT_INDEX channelIndex, signed int steps, unsigned int amplitude, unsigned int frequency);

	NPSCONTROL_API
		NT_STATUS NPSCONTROL_CC NT_Stop_S(NT_INDEX systemIndex, NT_INDEX channelIndex);

	NPSCONTROL_API
		NT_STATUS NPSCONTROL_CC NT_GotoPositionAbsolute_S(NT_INDEX systemIndex, NT_INDEX channelIndex, signed int position, unsigned int holdTime);

	NPSCONTROL_API
		NT_STATUS NPSCONTROL_CC NT_GotoPositionRelative_S(NT_INDEX systemIndex, NT_INDEX channelIndex, signed int diff, unsigned int holdTime);

	NPSCONTROL_API
		NT_STATUS NPSCONTROL_CC NT_GetStatus_S(NT_INDEX systemIndex, NT_INDEX channelIndex, unsigned int* status);

	NPSCONTROL_API
		NT_STATUS NPSCONTROL_CC NT_SetPosition_S(NT_INDEX systemIndex, NT_INDEX channelIndex, signed int position);

	NPSCONTROL_API
		NT_STATUS NPSCONTROL_CC NT_GetPosition_S(NT_INDEX systemIndex, NT_INDEX channelIndex, signed int* position);

	NPSCONTROL_API
		NT_STATUS NPSCONTROL_CC NT_SetSensorEnabled_S(NT_INDEX systemIndex, unsigned int enabled);

	NPSCONTROL_API
		NT_STATUS NPSCONTROL_CC NT_GetSensorEnabled_S(NT_INDEX systemIndex, unsigned int* enabled);

	NPSCONTROL_API
		NT_STATUS NPSCONTROL_CC NT_GetVoltageLevel_S(NT_INDEX systemIndex, NT_INDEX channelIndex, unsigned int* level);

	NPSCONTROL_API
		NT_STATUS NPSCONTROL_CC NT_ScanMoveRelative_S(NT_INDEX systemIndex, NT_INDEX channelIndex, signed int diff, unsigned int scanSpeed);

	NPSCONTROL_API
		NT_STATUS NPSCONTROL_CC NT_ScanMoveAbsolute_S(NT_INDEX systemIndex, NT_INDEX channelIndex, unsigned int target, unsigned int scanSpeed);

	NPSCONTROL_API
		NT_STATUS NPSCONTROL_CC NT_SetAccumulateRelativePositions_S(NT_INDEX systemIndex, NT_INDEX channelIndex, unsigned int accumulate);

	/************************************************************************
	*************************************************************************
	**       Section IIb:  Functions for ASYNCHRONOUS communication        **
	*************************************************************************
	************************************************************************/
	NPSCONTROL_API
		NT_STATUS NPSCONTROL_CC NT_StepMove_A(NT_INDEX systemIndex, NT_INDEX channelIndex, signed int steps, unsigned int amplitude, unsigned int frequency);

	NPSCONTROL_API
		NT_STATUS NPSCONTROL_CC NT_Stop_A(NT_INDEX systemIndex, NT_INDEX channelIndex);

	NPSCONTROL_API
		NT_STATUS NPSCONTROL_CC NT_GetStatus_A(NT_INDEX systemIndex, NT_INDEX channelIndex);

	NPSCONTROL_API
		NT_STATUS NPSCONTROL_CC NT_GotoPositionRelative_A(NT_INDEX systemIndex, NT_INDEX channelIndex, signed int diff, unsigned int holdTime);

	NPSCONTROL_API
		NT_STATUS NPSCONTROL_CC NT_GotoPositionAbsolute_A(NT_INDEX systemIndex, NT_INDEX channelIndex, signed int position, unsigned int holdTime);
		
	NPSCONTROL_API
		NT_STATUS NPSCONTROL_CC NT_GetPosition_A(NT_INDEX systemIndex, NT_INDEX channelIndex);

	NPSCONTROL_API
		NT_STATUS NPSCONTROL_CC NT_SetPosition_A(NT_INDEX systemIndex, NT_INDEX channelIndex, signed int position);

	NPSCONTROL_API
		NT_STATUS NPSCONTROL_CC NT_SetSensorEnabled_A(NT_INDEX systemIndex, unsigned int enabled);

	NPSCONTROL_API
		NT_STATUS NPSCONTROL_CC NT_GetSensorEnabled_A(NT_INDEX systemIndex);

	NPSCONTROL_API
		NT_STATUS NPSCONTROL_CC NT_GetVoltageLevel_A(NT_INDEX systemIndex, NT_INDEX channelIndex);

	NPSCONTROL_API
		NT_STATUS NPSCONTROL_CC NT_ScanMoveRelative_A(NT_INDEX systemIndex, NT_INDEX channelIndex, signed int diff, unsigned int scanSpeed);
		
	NPSCONTROL_API
		NT_STATUS NPSCONTROL_CC NT_ScanMoveAbsolute_A(NT_INDEX systemIndex, NT_INDEX channelIndex, unsigned int target, unsigned int scanSpeed);

	NPSCONTROL_API
		NT_STATUS NPSCONTROL_CC NT_SetAccumulateRelativePositions_A(NT_INDEX systemIndex, NT_INDEX channelIndex, unsigned int accumulate);

	NPSCONTROL_API
		NT_STATUS NPSCONTROL_CC NT_ReceiveNextPacket_A(NT_INDEX systemIndex, unsigned int timeout, NT_PACKET* packet);

	NPSCONTROL_API
		NT_STATUS NPSCONTROL_CC NT_CancelWaitForPacket_A(NT_INDEX systemIndex);

	NPSCONTROL_API
		NT_STATUS NPSCONTROL_CC NT_DiscardPacket_A(NT_INDEX systemIndex);

#ifdef __cplusplus
}
#endif 

#endif   /* NTCONTROL_H */ 