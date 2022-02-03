#pragma once
//=========================================================
// Servo register definitions
//=========================================================

#include <stdint.h>
#include <stddef.h>
#include <map>
//_________________________________________________________

namespace dynamixel {

	typedef int motor_id;

	typedef enum {
		None = 0,

		AX_12A = 12,
		AX_12W = 300,
		AX_18A = 18,

		RX_10 = 10,
		RX_24F = 24,
		RX_28 = 28,
		RX_64 = 64,

		EX_106 = 107,

		MX_12W = 360,
		MX_28 = 29,
		MX_28_2 = 30,
		MX_64 = 310,
		MX_64_2 = 311,
		MX_106 = 320,
		MX_106_2 = 321,

		XL_320 = 350,
		XL430_W250 = 1060,

		XL430_W250_2 = 1090, // 2XL

		XC430_W150 = 1070,
		XC430_W240 = 1080,

		XM430_W210 = 1030,
		XM430_W350 = 1020,

		XM540_W150 = 1130,
		XM540_W270 = 1120,

		XH430_W210 = 1010,
		XH430_W350 = 1000,
		XH430_V210 = 1050,
		XH430_V350 = 1040,

		XH540_W150 = 1110,
		XH540_W270 = 1100,
		XH540_V150 = 1150,
		XH540_V270 = 1140,

		XW540_T260 = 1170,
		XW540_T140 = 1180,

		PRO_L42_10_S300_R = 35072,
		PRO_L54_30_S400_R = 37928,
		PRO_L54_30_S500_R = 37896,
		PRO_L54_50_S290_R = 38176,
		PRO_L54_50_S500_R = 38152,

		PRO_M42_10_S260_R = 43288,
		PRO_M54_40_S250_R = 46096,
		PRO_M54_60_S250_R = 46352,

		PRO_H42_20_S300_R = 51200,
		PRO_H54_100_S500_R = 53768,
		PRO_H54_200_S500_R = 54024,

		PRO_M42_10_S260_R_A = 43289,
		PRO_M54_40_S250_R_A = 46097,
		PRO_M54_60_S250_R_A = 46353,

		PRO_H42_20_S300_R_A = 51201,
		PRO_H54_100_S500_R_A = 53769,
		PRO_H54_200_S500_R_A = 54025,

		PRO_PLUS_M42P_010_S260_R = 2100,
		PRO_PLUS_M54P_040_S250_R = 2110,
		PRO_PLUS_M54P_060_S250_R = 2120,

		PRO_PLUS_H42P_020_S300_R = 2000,
		PRO_PLUS_H54P_100_S500_R = 2010,
		PRO_PLUS_H54P_200_S500_R = 2020,

		RH_P12_RN = 35073,
		RH_P12_RN_A = 35074
	} model_t;

	typedef struct
	{
	float rpm;

	int64_t value_of_min_radian_position;
	int64_t value_of_zero_radian_position;
	int64_t value_of_max_radian_position;

	float  min_radian;
	float  max_radian;
	} ModelInfo;


	struct field {
		uint16_t address;
		uint16_t data_length;
		bool operator <(const field &b) const {
			return address < b.address || (address == b.address && data_length<b.data_length);
		};
	};

	typedef std::string field_name;
	struct model_info {
		const model_t type;

		//used to calculate the rpm:
		// value*rpm_factor = rpm [1/min]
		const float rpm_factor;

		//used to calculate the position from values
		const int64_t min_value;
		const int64_t max_value;
		const float  min_radian;
		const float  max_radian;
		
		// All available commands for this actuator
		const std::map<field_name,field> * const controll_table;
	};


	namespace command {
		static const std::string  Acceleration_Limit = "Acceleration_Limit";
		static const std::string  Alarm_LED = "Alarm_LED";
		static const std::string  Baud_Rate = "Baud_Rate";
		static const std::string  Bus_Watchdog = "Bus_Watchdog";
		static const std::string  CCW_Angle_Limit = "CCW_Angle_Limit";
		static const std::string  CCW_Compliance_Margin = "CCW_Compliance_Margin";
		static const std::string  CCW_Compliance_Slope = "CCW_Compliance_Slope";
		static const std::string  Control_Mode = "Control_Mode";
		static const std::string  Current = "Current";
		static const std::string  Current_Limit = "Current_Limit";
		static const std::string  CW_Angle_Limit = "CW_Angle_Limit";
		static const std::string  CW_Compliance_Margin = "CW_Compliance_Margin";
		static const std::string  CW_Compliance_Slope = "CW_Compliance_Slope";
		static const std::string  D_gain = "D_gain";
		static const std::string  Drive_Mode = "Drive_Mode";
		static const std::string  External_Port_Mode_1 = "External_Port_Mode_1";
		static const std::string  External_Port_Mode_2 = "External_Port_Mode_2";
		static const std::string  External_Port_Mode_3 = "External_Port_Mode_3";
		static const std::string  External_Port_Mode_4 = "External_Port_Mode_4";
		static const std::string  Feedforward_1st_Gain = "Feedforward_1st_Gain";
		static const std::string  Feedforward_2nd_Gain = "Feedforward_2nd_Gain";
		static const std::string  Firmware_Version = "Firmware_Version";
		static const std::string  Goal_Acceleration = "Goal_Acceleration";
		static const std::string  Goal_Current = "Goal_Current";
		static const std::string  Goal_Position = "Goal_Position";
		static const std::string  Goal_PWM = "Goal_PWM";
		static const std::string  Goal_Torque = "Goal_Torque";
		static const std::string  Goal_Velocity = "Goal_Velocity";
		static const std::string  Hardware_Error_Status = "Hardware_Error_Status";
		static const std::string  Homing_Offset = "Homing_Offset";
		static const std::string  I_gain = "I_gain";
		static const std::string  ID = "ID";
		static const std::string  LED = "LED";
		static const std::string  LED_BLUE = "LED_BLUE";
		static const std::string  LED_GREEN = "LED_GREEN";
		static const std::string  LED_RED = "LED_RED";
		static const std::string  Lock = "Lock";
		static const std::string  Max_Position_Limit = "Max_Position_Limit";
		static const std::string  Max_Torque = "Max_Torque";
		static const std::string  Max_Voltage_Limit = "Max_Voltage_Limit";
		static const std::string  Min_Position_Limit = "Min_Position_Limit";
		static const std::string  Min_Voltage_Limit = "Min_Voltage_Limit";
		static const std::string  Model_Number = "Model_Number";
		static const std::string  Moving = "Moving";
		static const std::string  Moving_Speed = "Moving_Speed";
		static const std::string  Moving_Status = "Moving_Status";
		static const std::string  Moving_Threshold = "Moving_Threshold";
		static const std::string  Multi_Turn_Offset = "Multi_Turn_Offset";
		static const std::string  Operating_Mode = "Operating_Mode";
		static const std::string  P_gain = "P_gain";
		static const std::string  Position_D_Gain = "Position_D_Gain";
		static const std::string  Position_I_Gain = "Position_I_Gain";
		static const std::string  Position_P_Gain = "Position_P_Gain";
		static const std::string  Position_Trajectory = "Position_Trajectory";
		static const std::string  Present_Current = "Present_Current";
		static const std::string  Present_Input = "Present_Input";
		static const std::string  Present_Input_Voltage = "Present_Input_Voltage";
		static const std::string  Present_Load = "Present_Load";
		static const std::string  Present_Position = "Present_Position";
		static const std::string  Present_PWM = "Present_PWM";
		static const std::string  Present_Speed = "Present_Speed";
		static const std::string  Present_Temperature = "Present_Temperature";
		static const std::string  Present_Velocity = "Present_Velocity";
		static const std::string  Present_Voltage = "Present_Voltage";
		static const std::string  Profile_Acceleration = "Profile_Acceleration";
		static const std::string  Profile_Velocity = "Profile_Velocity";
		static const std::string  Protocol_Version = "Protocol_Version";
		static const std::string  Punch = "Punch";
		static const std::string  PWM_Limit = "PWM_Limit";
		static const std::string  Realtime_Tick = "Realtime_Tick";
		static const std::string  Registered = "Registered";
		static const std::string  Registered_Instruction = "Registered_Instruction";
		static const std::string  Resolution_Divider = "Resolution_Divider";
		static const std::string  Return_Delay_Time = "Return_Delay_Time";
		static const std::string  Secondary_ID = "Secondary_ID";
		static const std::string  Sensored_Current = "Sensored_Current";
		static const std::string  Shutdown = "Shutdown";
		static const std::string  Status_Return_Level = "Status_Return_Level";
		static const std::string  Temperature_Limit = "Temperature_Limit";
		static const std::string  Torque_Control_Mode_Enable = "Torque_Control_Mode_Enable";
		static const std::string  Torque_Enable = "Torque_Enable";
		static const std::string  Torque_Limit = "Torque_Limit";
		static const std::string  Velocity_I_Gain = "Velocity_I_Gain";
		static const std::string  Velocity_Limit = "Velocity_Limit";
		static const std::string  Velocity_P_Gain = "Velocity_P_Gain";
		static const std::string  Velocity_Trajectory = "Velocity_Trajectory";
	}
	//_________________________________________________________
	using namespace command;

	//https://emanual.robotis.com/docs/en/dxl/x/xl430-w250/#hardware-error-status
	enum hardware_status {
		Input_Voltage_Error 	= 1<<0,	//0b00000001;
		OverHeating_Error 		= 1<<2,	//0b00000100;
		Motor_Encoder_Error 	= 1<<3,	//0b00001000;
		Electrical_Shock_Error = 1<<4,	//0b00010000;
		Overload_Error 		= 1<<5,	//0b00100000;
	};

	//---------------------------------------------------------
	// AX servos - (num == AX_12A || num == AX_12W || num == AX_18A)
	//---------------------------------------------------------

	static std::map<field_name,field> items_AX{
	    {Model_Number, {0, 2}},
	    {Firmware_Version, {2, 1}},
	    {ID, {3, 1}},
	    {Baud_Rate, {4, 1}},
	    {Return_Delay_Time, {5, 1}},
	    {CW_Angle_Limit, {6, 2}},
	    {CCW_Angle_Limit, {8, 2}},
	    {Temperature_Limit, {11, 1}},
	    {Min_Voltage_Limit, {12, 1}},
	    {Max_Voltage_Limit, {13, 1}},
	    {Max_Torque, {14, 2}},
	    {Status_Return_Level, {16, 1}},
	    {Alarm_LED, {17, 1}},
	    {Shutdown, {18, 1}},

	    {Torque_Enable, {24, 1}},
	    {LED, {25, 1}},
	    {CW_Compliance_Margin, {26, 1}},
	    {CCW_Compliance_Margin, {27, 1}},
	    {CW_Compliance_Slope, {28, 1}},
	    {CCW_Compliance_Slope, {29, 1}},
	    {Goal_Position, {30, 2}},
	    {Moving_Speed, {32, 2}},
	    {Torque_Limit, {34, 2}},
	    {Present_Position, {36, 2}},
	    {Present_Speed, {38, 2}},
	    {Present_Load, {40, 2}},
	    {Present_Voltage, {42, 1}},
	    {Present_Temperature, {43, 1}},
	    {Registered, {44, 1}},
	    {Moving, {46, 1}},
	    {Lock, {47, 1}},
	    {Punch, {48, 2}}};
    #define COUNT_AX_ITEMS (sizeof(items_AX) / sizeof(items_AX[0]))

    static const ModelInfo info_AX = {0.11,
							 0,
							 512,
							 1024,
							 -2.61799, 
							 2.61799};
    //---------------------------------------------------------
    // RX servos - (num == RX_10 || num == RX_24F || num == RX_28 || num == RX_64)
    //---------------------------------------------------------


    static std::map<field_name,field>  items_RX = {
	    {Model_Number, {0, 2}},
	    {Firmware_Version, {2, 1}},
	    {ID, {3, 1}},
	    {Baud_Rate, {4, 1}},
	    {Return_Delay_Time, {5, 1}},
	    {CW_Angle_Limit, {6, 2}},
	    {CCW_Angle_Limit, {8, 2}},
	    {Temperature_Limit, {11, 1}},
	    {Min_Voltage_Limit, {12, 1}},
	    {Max_Voltage_Limit, {13, 1}},
	    {Max_Torque, {14, 2}},
	    {Status_Return_Level, {16, 1}},
	    {Alarm_LED, {17, 1}},
	    {Shutdown, {18, 1}},

	    {Torque_Enable, {24, 1}},
	    {LED, {25, 1}},
	    {CW_Compliance_Margin, {26, 1}},
	    {CCW_Compliance_Margin, {27, 1}},
	    {CW_Compliance_Slope, {28, 1}},
	    {CCW_Compliance_Slope, {29, 1}},
	    {Goal_Position, {30, 2}},
	    {Moving_Speed, {32, 2}},
	    {Torque_Limit, {34, 2}},
	    {Present_Position, {36, 2}},
	    {Present_Speed, {38, 2}},
	    {Present_Load, {40, 2}},
	    {Present_Voltage, {42, 1}},
	    {Present_Temperature, {43, 1}},
	    {Registered, {44, 1}},
	    {Moving, {46, 1}},
	    {Lock, {47, 1}},
	    {Punch, {48, 2}}};

    #define COUNT_RX_ITEMS (sizeof(items_RX) / sizeof(items_RX[0]))

    static const ModelInfo info_RX = {0.11,
							 0,
							 512,
							 1024,
							 -2.61799, 
							 2.61799};


    //---------------------------------------------------------
    // EX servos - (num == EX_106)
    //---------------------------------------------------------
    static std::map<field_name,field>  items_EX = {
	    {Model_Number, {0, 2}},
	    {Firmware_Version, {2, 1}},
	    {ID, {3, 1}},
	    {Baud_Rate, {4, 1}},
	    {Return_Delay_Time, {5, 1}},
	    {CW_Angle_Limit, {6, 2}},
	    {CCW_Angle_Limit, {8, 2}},
	    {Drive_Mode, {10, 1}},
	    {Temperature_Limit, {11, 1}},
	    {Min_Voltage_Limit, {12, 1}},
	    {Max_Voltage_Limit, {13, 1}},
	    {Max_Torque, {14, 2}},
	    {Status_Return_Level, {16, 1}},
	    {Alarm_LED, {17, 1}},
	    {Shutdown, {18, 1}},

	    {Torque_Enable, {24, 1}},
	    {LED, {25, 1}},
	    {CW_Compliance_Margin, {26, 1}},
	    {CCW_Compliance_Margin, {27, 1}},
	    {CW_Compliance_Slope, {28, 1}},
	    {CCW_Compliance_Slope, {29, 1}},
	    {Goal_Position, {30, 2}},
	    {Moving_Speed, {34, 2}},
	    {Torque_Limit, {35, 2}},
	    {Present_Position, {36, 2}},
	    {Present_Speed, {38, 2}},
	    {Present_Load, {40, 2}},
	    {Present_Voltage, {42, 1}},
	    {Present_Temperature, {43, 1}},
	    {Registered, {44, 1}},
	    {Moving, {46, 1}},
	    {Lock, {47, 1}},
	    {Punch, {48, 2}},
	    {Sensored_Current, {56, 2}}};

    #define COUNT_EX_ITEMS (sizeof(items_EX) / sizeof(items_EX[0]))

    static const ModelInfo info_EX = {0.11,
							 0,
							 2048,
							 4096,
							 -2.18969008, 
							 2.18969008};


    //---------------------------------------------------------
    // MX Protocol 1 servos - (num == MX_12W || num == MX_28)
    //---------------------------------------------------------
    static std::map<field_name,field>  items_MX = {
	    {Model_Number, {0, 2}},
	    {Firmware_Version, {2, 1}},
	    {ID, {3, 1}},
	    {Baud_Rate, {4, 1}},
	    {Return_Delay_Time, {5, 1}},
	    {CW_Angle_Limit, {6, 2}},
	    {CCW_Angle_Limit, {8, 2}},
	    {Temperature_Limit, {11, 1}},
	    {Min_Voltage_Limit, {12, 1}},
	    {Max_Voltage_Limit, {13, 1}},
	    {Max_Torque, {14, 2}},
	    {Status_Return_Level, {16, 1}},
	    {Alarm_LED, {17, 1}},
	    {Shutdown, {18, 1}},
	    {Multi_Turn_Offset, {20, 2}},
	    {Resolution_Divider, {22, 1}},

	    {Torque_Enable, {24, 1}},
	    {LED, {25, 1}},
	    {D_gain, {26, 1}},
	    {I_gain, {27, 1}},
	    {P_gain, {28, 1}},
	    {Goal_Position, {30, 2}},
	    {Moving_Speed, {32, 2}},
	    {Torque_Limit, {34, 2}},
	    {Present_Position, {36, 2}},
	    {Present_Speed, {38, 2}},
	    {Present_Load, {40, 2}},
	    {Present_Voltage, {42, 1}},
	    {Present_Temperature, {43, 1}},
	    {Registered, {44, 1}},
	    {Moving, {46, 1}},
	    {Lock, {47, 1}},
	    {Punch, {48, 2}},
	    {Goal_Acceleration, {73, 1}}};

    #define COUNT_MX_ITEMS (sizeof(items_MX) / sizeof(items_MX[0]))

    static const ModelInfo info_MX = {0.11,
							 0,
							 2048,
							 4096,
							 -3.14159265, 
							 3.14159265};

    //---------------------------------------------------------
    // MX Protocol 2 servos - (num == MX_28_2)
    //---------------------------------------------------------
    static std::map<field_name,field>  items_MX2 = {
	    {Model_Number, {0, 2}},
	    {Firmware_Version, {6, 1}},
	    {ID, {7, 1}},
	    {Baud_Rate, {8, 1}},
	    {Return_Delay_Time, {9, 1}},
	    {Drive_Mode, {10, 1}},
	    {Operating_Mode, {11, 1}},
	    {Secondary_ID, {12, 1}},
	    {Protocol_Version, {13, 1}},
	    {Homing_Offset, {20, 4}},
	    {Moving_Threshold, {24, 4}},
	    {Temperature_Limit, {31, 1}},
	    {Max_Voltage_Limit, {32, 2}},
	    {Min_Voltage_Limit, {34, 2}},
	    {PWM_Limit, {36, 2}},
	    {Acceleration_Limit, {40, 4}},
	    {Velocity_Limit, {44, 4}},
	    {Max_Position_Limit, {48, 4}},
	    {Min_Position_Limit, {52, 4}},
	    {Shutdown, {63, 1}},

	    {Torque_Enable, {64, 1}},
	    {LED, {65, 1}},
	    {Status_Return_Level, {68, 1}},
	    {Registered_Instruction, {69, 1}},
	    {Hardware_Error_Status, {70, 1}},
	    {Velocity_I_Gain, {76, 2}},
	    {Velocity_P_Gain, {78, 2}},
	    {Position_D_Gain, {80, 2}},
	    {Position_I_Gain, {82, 2}},
	    {Position_P_Gain, {84, 2}},
	    {Feedforward_2nd_Gain, {88, 2}},
	    {Feedforward_1st_Gain, {90, 2}},
	    {Bus_Watchdog, {98, 1}},
	    {Goal_PWM, {100, 2}},
	    {Goal_Velocity, {104, 4}},
	    {Profile_Acceleration, {108, 4}},
	    {Profile_Velocity, {112, 4}},
	    {Goal_Position, {116, 4}},
	    {Realtime_Tick, {120, 2}},
	    {Moving, {122, 1}},
	    {Moving_Status, {123, 1}},
	    {Present_PWM, {124, 2}},
	    {Present_Load, {126, 2}},
	    {Present_Velocity, {128, 4}},
	    {Present_Position, {132, 4}},
	    {Velocity_Trajectory, {136, 4}},
	    {Position_Trajectory, {140, 4}},
	    {Present_Input_Voltage, {144, 2}},
	    {Present_Temperature, {146, 1}}};

    #define COUNT_MX2_ITEMS (sizeof(items_MX2) / sizeof(items_MX2[0]))

    static const ModelInfo info_MX2 = {0.229,
							 0,
							 2048,
							 4096,
							 -3.14159265, 
							 3.14159265};

    //---------------------------------------------------------
    // EXT MX Protocol 1 servos - (num == MX_64 || num == MX_106)
    //---------------------------------------------------------
    static std::map<field_name,field>  items_EXTMX = {
	    {Model_Number, {0, 2}},
	    {Firmware_Version, {2, 1}},
	    {ID, {3, 1}},
	    {Baud_Rate, {4, 1}},
	    {Return_Delay_Time, {5, 1}},
	    {CW_Angle_Limit, {6, 2}},
	    {CCW_Angle_Limit, {8, 2}},
	    {Temperature_Limit, {11, 1}},
	    {Min_Voltage_Limit, {12, 1}},
	    {Max_Voltage_Limit, {13, 1}},
	    {Max_Torque, {14, 2}},
	    {Status_Return_Level, {16, 1}},
	    {Alarm_LED, {17, 1}},
	    {Shutdown, {18, 1}},
	    {Multi_Turn_Offset, {20, 2}},
	    {Resolution_Divider, {22, 1}},

	    {Torque_Enable, {24, 1}},
	    {LED, {25, 1}},
	    {D_gain, {26, 1}},
	    {I_gain, {27, 1}},
	    {P_gain, {28, 1}},
	    {Goal_Position, {30, 2}},
	    {Moving_Speed, {32, 2}},
	    {Torque_Limit, {34, 2}},
	    {Present_Position, {36, 2}},
	    {Present_Speed, {38, 2}},
	    {Present_Load, {40, 2}},
	    {Present_Voltage, {42, 1}},
	    {Present_Temperature, {43, 1}},
	    {Registered, {44, 1}},
	    {Moving, {46, 1}},
	    {Lock, {47, 1}},
	    {Punch, {48, 2}},
	    {Current, {68, 2}},
	    {Torque_Control_Mode_Enable, {70, 1}},
	    {Goal_Torque, {71, 2}},
	    {Goal_Acceleration, {73, 1}}};

    #define COUNT_EXTMX_ITEMS (sizeof(items_EXTMX) / sizeof(items_EXTMX[0]))

    static const ModelInfo info_EXTMX = {0.11,
							 0,
							 2048,
							 4096,
							 -3.14159265, 
							 3.14159265};

    //---------------------------------------------------------
    // EXT MX Protocol 2 Servos - (num == MX_64_2 || num == MX_106_2)
    //---------------------------------------------------------
    static std::map<field_name,field>  items_EXTMX2 = {
	    {Model_Number, {0, 2}},
	    {Firmware_Version, {6, 1}},
	    {ID, {7, 1}},
	    {Baud_Rate, {8, 1}},
	    {Return_Delay_Time, {9, 1}},
	    {Drive_Mode, {10, 1}},
	    {Operating_Mode, {11, 1}},
	    {Secondary_ID, {12, 1}},
	    {Protocol_Version, {13, 1}},
	    {Homing_Offset, {20, 4}},
	    {Moving_Threshold, {24, 4}},
	    {Temperature_Limit, {31, 1}},
	    {Max_Voltage_Limit, {32, 2}},
	    {Min_Voltage_Limit, {34, 2}},
	    {PWM_Limit, {36, 2}},
	    {Current_Limit, {38, 2}},
	    {Acceleration_Limit, {40, 4}},
	    {Velocity_Limit, {44, 4}},
	    {Max_Position_Limit, {48, 4}},
	    {Min_Position_Limit, {52, 4}},
	    {Shutdown, {63, 1}},

	    {Torque_Enable, {64, 1}},
	    {LED, {65, 1}},
	    {Status_Return_Level, {68, 1}},
	    {Registered_Instruction, {69, 1}},
	    {Hardware_Error_Status, {70, 1}},
	    {Velocity_I_Gain, {76, 2}},
	    {Velocity_P_Gain, {78, 2}},
	    {Position_D_Gain, {80, 2}},
	    {Position_I_Gain, {82, 2}},
	    {Position_P_Gain, {84, 2}},
	    {Feedforward_2nd_Gain, {88, 2}},
	    {Feedforward_1st_Gain, {90, 2}},
	    {Bus_Watchdog, {98, 1}},
	    {Goal_PWM, {100, 2}},
	    {Goal_Current, {102, 2}},
	    {Goal_Velocity, {104, 4}},
	    {Profile_Acceleration, {108, 4}},
	    {Profile_Velocity, {112, 4}},
	    {Goal_Position, {116, 4}},
	    {Realtime_Tick, {120, 2}},
	    {Moving, {122, 1}},
	    {Moving_Status, {123, 1}},
	    {Present_PWM, {124, 2}},
	    {Present_Current, {126, 2}},
	    {Present_Velocity, {128, 4}},
	    {Present_Position, {132, 4}},
	    {Velocity_Trajectory, {136, 4}},
	    {Position_Trajectory, {140, 4}},
	    {Present_Input_Voltage, {144, 2}},
	    {Present_Temperature, {146, 1}}};

    #define COUNT_EXTMX2_ITEMS (sizeof(items_EXTMX2) / sizeof(items_EXTMX2[0]))

    static const ModelInfo info_EXTMX2 = {0.229,
							 0,
							 2048,
							 4096,
							 -3.14159265, 
							 3.14159265};

    //---------------------------------------------------------
    // XL320 - (num == XL_320)
    //---------------------------------------------------------
    static std::map<field_name,field>  items_XL320 = {
	    {Model_Number, {0, 2}},
	    {Firmware_Version, {2, 1}},
	    {ID, {3, 1}},
	    {Baud_Rate, {4, 1}},
	    {Return_Delay_Time, {5, 1}},
	    {CW_Angle_Limit, {6, 2}},
	    {CCW_Angle_Limit, {8, 2}},
	    {Control_Mode, {11, 1}},
	    {Temperature_Limit, {12, 1}},
	    {Min_Voltage_Limit, {13, 1}},
	    {Max_Voltage_Limit, {14, 1}},
	    {Max_Torque, {15, 2}},
	    {Status_Return_Level, {17, 1}},
	    {Shutdown, {18, 1}},

	    {Torque_Enable, {24, 1}},
	    {LED, {25, 1}},
	    {D_gain, {27, 1}},
	    {I_gain, {28, 1}},
	    {P_gain, {29, 1}},
	    {Goal_Position, {30, 2}},
	    {Moving_Speed, {32, 2}},
	    {Torque_Limit, {34, 2}},
	    {Present_Position, {37, 2}},
	    {Present_Speed, {39, 2}},
	    {Present_Load, {41, 2}},
	    {Present_Voltage, {45, 1}},
	    {Present_Temperature, {46, 1}},
	    {Registered, {47, 1}},
	    {Moving, {49, 1}},
	    {Hardware_Error_Status, {50, 1}},
	    {Punch, {51, 2}}};

    #define COUNT_XL320_ITEMS (sizeof(items_XL320) / sizeof(items_XL320[0]))

    static const ModelInfo info_XL320 = {0.11,
							 0,
							 512,
							 1024,
							 -2.61799, 
							 2.61799};


	//---------------------------------------------------------
	// XL - (num == XL430_W250, XL430_W250_2, XC430_W150, XC430_W240)
	//---------------------------------------------------------
	static std::map<field_name,field>  items_XL = {
	    {Model_Number, {0, 2}},
	    {Firmware_Version, {6, 1}},
	    {ID, {7, 1}},
	    {Baud_Rate, {8, 1}},
	    {Return_Delay_Time, {9, 1}},
	    {Drive_Mode, {10, 1}},
	    {Operating_Mode, {11, 1}},
	    {Secondary_ID, {12, 1}},
	    {Protocol_Version, {13, 1}},
	    {Homing_Offset, {20, 4}},
	    {Moving_Threshold, {24, 4}},
	    {Temperature_Limit, {31, 1}},
	    {Max_Voltage_Limit, {32, 2}},
	    {Min_Voltage_Limit, {34, 2}},
	    {PWM_Limit, {36, 2}},
	    {Acceleration_Limit, {40, 4}},
	    {Velocity_Limit, {44, 4}},
	    {Max_Position_Limit, {48, 4}},
	    {Min_Position_Limit, {52, 4}},
	    {Shutdown, {63, 1}},

	    {Torque_Enable, {64, 1}},
	    {LED, {65, 1}},
	    {Status_Return_Level, {68, 1}},
	    {Registered_Instruction, {69, 1}},
	    {Hardware_Error_Status, {70, 1}},
	    {Velocity_I_Gain, {76, 2}},
	    {Velocity_P_Gain, {78, 2}},
	    {Position_D_Gain, {80, 2}},
	    {Position_I_Gain, {82, 2}},
	    {Position_P_Gain, {84, 2}},
	    {Feedforward_2nd_Gain, {88, 2}},
	    {Feedforward_1st_Gain, {90, 2}},
	    {Bus_Watchdog, {98, 1}},
	    {Goal_PWM, {100, 2}},
	    {Goal_Velocity, {104, 4}},
	    {Profile_Acceleration, {108, 4}},
	    {Profile_Velocity, {112, 4}},
	    {Goal_Position, {116, 4}},
	    {Realtime_Tick, {120, 2}},
	    {Moving, {122, 1}},
	    {Moving_Status, {123, 1}},
	    {Present_PWM, {124, 2}},
	    {Present_Load, {126, 2}},
	    {Present_Velocity, {128, 4}},
	    {Present_Position, {132, 4}},
	    {Velocity_Trajectory, {136, 4}},
	    {Position_Trajectory, {140, 4}},
	    {Present_Input_Voltage, {144, 2}},
	    {Present_Temperature, {146, 1}}};

    	static const ModelInfo info_XL = {0.229,
							 0,
							 2048,
							 4096,
							 -3.14159265, 
							 3.14159265};


    //---------------------------------------------------------
    // XM - (num == XM430_W210 || num == XM430_W350)
    //---------------------------------------------------------
    static std::map<field_name,field>  items_XM = {
	    {Model_Number, {0, 2}},
	    {Firmware_Version, {6, 1}},
	    {ID, {7, 1}},
	    {Baud_Rate, {8, 1}},
	    {Return_Delay_Time, {9, 1}},
	    {Drive_Mode, {10, 1}},
	    {Operating_Mode, {11, 1}},
	    {Secondary_ID, {12, 1}},
	    {Protocol_Version, {13, 1}},
	    {Homing_Offset, {20, 4}},
	    {Moving_Threshold, {24, 4}},
	    {Temperature_Limit, {31, 1}},
	    {Max_Voltage_Limit, {32, 2}},
	    {Min_Voltage_Limit, {34, 2}},
	    {PWM_Limit, {36, 2}},
	    {Current_Limit, {38, 2}},
	    {Acceleration_Limit, {40, 4}},
	    {Velocity_Limit, {44, 4}},
	    {Max_Position_Limit, {48, 4}},
	    {Min_Position_Limit, {52, 4}},
	    {Shutdown, {63, 1}},

	    {Torque_Enable, {64, 1}},
	    {LED, {65, 1}},
	    {Status_Return_Level, {68, 1}},
	    {Registered_Instruction, {69, 1}},
	    {Hardware_Error_Status, {70, 1}},
	    {Velocity_I_Gain, {76, 2}},
	    {Velocity_P_Gain, {78, 2}},
	    {Position_D_Gain, {80, 2}},
	    {Position_I_Gain, {82, 2}},
	    {Position_P_Gain, {84, 2}},
	    {Feedforward_2nd_Gain, {88, 2}},
	    {Feedforward_1st_Gain, {90, 2}},
	    {Bus_Watchdog, {98, 1}},
	    {Goal_PWM, {100, 2}},
	    {Goal_Current, {102, 2}},
	    {Goal_Velocity, {104, 4}},
	    {Profile_Acceleration, {108, 4}},
	    {Profile_Velocity, {112, 4}},
	    {Goal_Position, {116, 4}},
	    {Realtime_Tick, {120, 2}},
	    {Moving, {122, 1}},
	    {Moving_Status, {123, 1}},
	    {Present_PWM, {124, 2}},
	    {Present_Current, {126, 2}},
	    {Present_Velocity, {128, 4}},
	    {Present_Position, {132, 4}},
	    {Velocity_Trajectory, {136, 4}},
	    {Position_Trajectory, {140, 4}},
	    {Present_Input_Voltage, {144, 2}},
	    {Present_Temperature, {146, 1}}};

    #define COUNT_XM_ITEMS (sizeof(items_XM) / sizeof(items_XM[0]))

    static const ModelInfo info_XM = {0.229,
							 0,
							 2048,
							 4096,
							 -3.14159265, 
							 3.14159265};

    //---------------------------------------------------------
    // EXTXM - (num == XM540_W150 || num == XM540_W270)
    //---------------------------------------------------------
    static std::map<field_name,field>  items_EXTXM = {
	    {Model_Number, {0, 2}},
	    {Firmware_Version, {6, 1}},
	    {ID, {7, 1}},
	    {Baud_Rate, {8, 1}},
	    {Return_Delay_Time, {9, 1}},
	    {Drive_Mode, {10, 1}},
	    {Operating_Mode, {11, 1}},
	    {Secondary_ID, {12, 1}},
	    {Protocol_Version, {13, 1}},
	    {Homing_Offset, {20, 4}},
	    {Moving_Threshold, {24, 4}},
	    {Temperature_Limit, {31, 1}},
	    {Max_Voltage_Limit, {32, 2}},
	    {Min_Voltage_Limit, {34, 2}},
	    {PWM_Limit, {36, 2}},
	    {Current_Limit, {38, 2}},
	    {Acceleration_Limit, {40, 4}},
	    {Velocity_Limit, {44, 4}},
	    {Max_Position_Limit, {48, 4}},
	    {Min_Position_Limit, {52, 4}},
	    {External_Port_Mode_1, {56, 1}},
	    {External_Port_Mode_2, {57, 1}},
	    {External_Port_Mode_3, {58, 1}},
	    {Shutdown, {63, 1}},

	    {Torque_Enable, {64, 1}},
	    {LED, {65, 1}},
	    {Status_Return_Level, {68, 1}},
	    {Registered_Instruction, {69, 1}},
	    {Hardware_Error_Status, {70, 1}},
	    {Velocity_I_Gain, {76, 2}},
	    {Velocity_P_Gain, {78, 2}},
	    {Position_D_Gain, {80, 2}},
	    {Position_I_Gain, {82, 2}},
	    {Position_P_Gain, {84, 2}},
	    {Feedforward_2nd_Gain, {88, 2}},
	    {Feedforward_1st_Gain, {90, 2}},
	    {Bus_Watchdog, {98, 1}},
	    {Goal_PWM, {100, 2}},
	    {Goal_Current, {102, 2}},
	    {Goal_Velocity, {104, 4}},
	    {Profile_Acceleration, {108, 4}},
	    {Profile_Velocity, {112, 4}},
	    {Goal_Position, {116, 4}},
	    {Realtime_Tick, {120, 2}},
	    {Moving, {122, 1}},
	    {Moving_Status, {123, 1}},
	    {Present_PWM, {124, 2}},
	    {Present_Current, {126, 2}},
	    {Present_Velocity, {128, 4}},
	    {Present_Position, {132, 4}},
	    {Velocity_Trajectory, {136, 4}},
	    {Position_Trajectory, {140, 4}},
	    {Present_Input_Voltage, {144, 2}},
	    {Present_Temperature, {146, 1}}};

    #define COUNT_EXTXM_ITEMS (sizeof(items_EXTXM) / sizeof(items_EXTXM[0]))

    static const ModelInfo info_EXTXM = {0.229,
							 0,
							 2048,
							 4096,
							 -3.14159265, 
							 3.14159265};

    //---------------------------------------------------------
    // XH - (num == XH430_V210 || num == XH430_V350 || num == XH430_W210 || num == XH430_W350)
    //---------------------------------------------------------
    static std::map<field_name,field>  items_XH = {
	    {Model_Number, {0, 2}},
	    {Firmware_Version, {6, 1}},
	    {ID, {7, 1}},
	    {Baud_Rate, {8, 1}},
	    {Return_Delay_Time, {9, 1}},
	    {Drive_Mode, {10, 1}},
	    {Operating_Mode, {11, 1}},
	    {Secondary_ID, {12, 1}},
	    {Protocol_Version, {13, 1}},
	    {Homing_Offset, {20, 4}},
	    {Moving_Threshold, {24, 4}},
	    {Temperature_Limit, {31, 1}},
	    {Max_Voltage_Limit, {32, 2}},
	    {Min_Voltage_Limit, {34, 2}},
	    {PWM_Limit, {36, 2}},
	    {Current_Limit, {38, 2}},
	    {Acceleration_Limit, {40, 4}},
	    {Velocity_Limit, {44, 4}},
	    {Max_Position_Limit, {48, 4}},
	    {Min_Position_Limit, {52, 4}},
	    {Shutdown, {63, 1}},

	    {Torque_Enable, {64, 1}},
	    {LED, {65, 1}},
	    {Status_Return_Level, {68, 1}},
	    {Registered_Instruction, {69, 1}},
	    {Hardware_Error_Status, {70, 1}},
	    {Velocity_I_Gain, {76, 2}},
	    {Velocity_P_Gain, {78, 2}},
	    {Position_D_Gain, {80, 2}},
	    {Position_I_Gain, {82, 2}},
	    {Position_P_Gain, {84, 2}},
	    {Feedforward_2nd_Gain, {88, 2}},
	    {Feedforward_1st_Gain, {90, 2}},
	    {Bus_Watchdog, {98, 1}},
	    {Goal_PWM, {100, 2}},
	    {Goal_Current, {102, 2}},
	    {Goal_Velocity, {104, 4}},
	    {Profile_Acceleration, {108, 4}},
	    {Profile_Velocity, {112, 4}},
	    {Goal_Position, {116, 4}},
	    {Realtime_Tick, {120, 2}},
	    {Moving, {122, 1}},
	    {Moving_Status, {123, 1}},
	    {Present_PWM, {124, 2}},
	    {Present_Current, {126, 2}},
	    {Present_Velocity, {128, 4}},
	    {Present_Position, {132, 4}},
	    {Velocity_Trajectory, {136, 4}},
	    {Position_Trajectory, {140, 4}},
	    {Present_Input_Voltage, {144, 2}},
	    {Present_Temperature, {146, 1}}};

    #define COUNT_XH_ITEMS (sizeof(items_XH) / sizeof(items_XH[0]))

    static const ModelInfo info_XH = {0.229,
							 0,
							 2048,
							 4096,
							 -3.14159265, 
							 3.14159265};

    //---------------------------------------------------------
    // EXTXH - (num == XH540_W150 || num == XH540_W270 || num == XH540_V150 || num == XH540_V270)
    //---------------------------------------------------------
    static std::map<field_name,field>  items_EXTXH = {
	    {Model_Number, {      0, 2}},
	    {Firmware_Version, {  6, 1}},
	    {ID, {                7, 1}},
	    {Baud_Rate, {         8, 1}},
	    {Return_Delay_Time, { 9, 1}},
	    {Drive_Mode, {        10, 1}},
	    {Operating_Mode, {    11, 1}},
	    {Secondary_ID, {      12, 1}},
	    {Protocol_Version, {  13, 1}},
	    {Homing_Offset, {     20, 4}},
	    {Moving_Threshold, {  24, 4}},
	    {Temperature_Limit, { 31, 1}},
	    {Max_Voltage_Limit, { 32, 2}},
	    {Min_Voltage_Limit, { 34, 2}},
	    {PWM_Limit, {         36, 2}},
	    {Current_Limit, {     38, 2}},
	    {Acceleration_Limit, {40, 4}},
	    {Velocity_Limit, {    44, 4}},
	    {Max_Position_Limit, {48, 4}},
	    {Min_Position_Limit, {52, 4}},
	    {Shutdown, {          63, 1}},

	    {Torque_Enable, {         64, 1}},
	    {LED, {                   65, 1}},
	    {Status_Return_Level, {   68, 1}},
	    {Registered_Instruction, {69, 1}},
	    {Hardware_Error_Status, { 70, 1}},
	    {Velocity_I_Gain, {       76, 2}},
	    {Velocity_P_Gain, {       78, 2}},
	    {Position_D_Gain, {       80, 2}},
	    {Position_I_Gain, {       82, 2}},
	    {Position_P_Gain, {       84, 2}},
	    {Feedforward_2nd_Gain, {  88, 2}},
	    {Feedforward_1st_Gain, {  90, 2}},
	    {Bus_Watchdog, {          98, 1}},
	    {Goal_PWM, {              100, 2}},
	    {Goal_Current, {          102, 2}},
	    {Goal_Velocity, {         104, 4}},
	    {Profile_Acceleration, {  108, 4}},
	    {Profile_Velocity, {      112, 4}},
	    {Goal_Position, {         116, 4}},
	    {Realtime_Tick, {         120, 2}},
	    {Moving, {                122, 1}},
	    {Moving_Status, {         123, 1}},
	    {Present_PWM, {           124, 2}},
	    {Present_Current, {       126, 2}},
	    {Present_Velocity, {      128, 4}},
	    {Present_Position, {      132, 4}},
	    {Velocity_Trajectory, {   136, 4}},
	    {Position_Trajectory, {   140, 4}},
	    {Present_Input_Voltage, { 144, 2}},
	    {Present_Temperature, {   146, 1}}};

    #define COUNT_EXTXH_ITEMS (sizeof(items_EXTXH) / sizeof(items_EXTXH[0]))

    static const ModelInfo info_EXTXH = {0.229,
							 0,
							 2048,
							 4096,
							 -3.14159265, 
							 3.14159265};

    //---------------------------------------------------------
    // XW - (num == XW540_T260 || XW540_T140)
    //---------------------------------------------------------
    static std::map<field_name,field>  items_XW = {
	    {Model_Number, {      0, 2}},
	    {Firmware_Version, {  6, 1}},
	    {ID, {                7, 1}},
	    {Baud_Rate, {         8, 1}},
	    {Return_Delay_Time, { 9, 1}},
	    {Drive_Mode, {        10, 1}},
	    {Operating_Mode, {    11, 1}},
	    {Secondary_ID, {      12, 1}},
	    {Protocol_Version, {  13, 1}},
	    {Homing_Offset, {     20, 4}},
	    {Moving_Threshold, {  24, 4}},
	    {Temperature_Limit, { 31, 1}},
	    {Max_Voltage_Limit, { 32, 2}},
	    {Min_Voltage_Limit, { 34, 2}},
	    {PWM_Limit, {         36, 2}},
	    {Current_Limit, {     38, 2}},
	    {Acceleration_Limit, {40, 4}},
	    {Velocity_Limit, {    44, 4}},
	    {Max_Position_Limit, {48, 4}},
	    {Min_Position_Limit, {52, 4}},
	    {Shutdown, {          63, 1}},

	    {Torque_Enable, {         64, 1}},
	    {Status_Return_Level, {   68, 1}},
	    {Registered_Instruction, {69, 1}},
	    {Hardware_Error_Status, { 70, 1}},
	    {Velocity_I_Gain, {       76, 2}},
	    {Velocity_P_Gain, {       78, 2}},
	    {Position_D_Gain, {       80, 2}},
	    {Position_I_Gain, {       82, 2}},
	    {Position_P_Gain, {       84, 2}},
	    {Feedforward_2nd_Gain, {  88, 2}},
	    {Feedforward_1st_Gain, {  90, 2}},
	    {Bus_Watchdog, {          98, 1}},
	    {Goal_PWM, {              100, 2}},
	    {Goal_Current, {          102, 2}},
	    {Goal_Velocity, {         104, 4}},
	    {Profile_Acceleration, {  108, 4}},
	    {Profile_Velocity, {      112, 4}},
	    {Goal_Position, {         116, 4}},
	    {Realtime_Tick, {         120, 2}},
	    {Moving, {                122, 1}},
	    {Moving_Status, {         123, 1}},
	    {Present_PWM, {           124, 2}},
	    {Present_Current, {       126, 2}},
	    {Present_Velocity, {      128, 4}},
	    {Present_Position, {      132, 4}},
	    {Velocity_Trajectory, {   136, 4}},
	    {Position_Trajectory, {   140, 4}},
	    {Present_Input_Voltage, { 144, 2}},
	    {Present_Temperature, {   146, 1}}};

    #define COUNT_XW_ITEMS (sizeof(items_XW) / sizeof(items_XW[0]))

    static const ModelInfo info_XW = {0.229,
							 0,
							 2048,
							 4096,
							 -3.14159265,
							 3.14159265};

    //---------------------------------------------------------
    // PRO - (num == PRO_L42_10_S300_R)
    //---------------------------------------------------------
    static std::map<field_name,field>  items_PRO = {
	    {Model_Number, {0, 2}},
	    {Firmware_Version, {6, 1}},
	    {ID, {7, 1}},
	    {Baud_Rate, {8, 1}},
	    {Return_Delay_Time, {9, 1}},
	    {Operating_Mode, {11, 1}},
	    {Moving_Threshold, {17, 4}},
	    {Temperature_Limit, {21, 1}},
	    {Max_Voltage_Limit, {22, 2}},
	    {Min_Voltage_Limit, {24, 2}},
	    {Acceleration_Limit, {26, 4}},
	    {Torque_Limit, {30, 2}},
	    {Velocity_Limit, {32, 4}},
	    {Max_Position_Limit, {36, 4}},
	    {Min_Position_Limit, {40, 4}},
	    {External_Port_Mode_1, {44, 1}},
	    {External_Port_Mode_2, {45, 1}},
	    {External_Port_Mode_3, {46, 1}},
	    {External_Port_Mode_4, {47, 1}},
	    {Shutdown, {48, 1}},

	    {Torque_Enable, {562, 1}},
	    {LED_RED, {563, 1}},
	    {LED_GREEN, {564, 1}},
	    {LED_BLUE, {565, 1}},
	    {Velocity_I_Gain, {586, 2}},
	    {Velocity_P_Gain, {588, 2}},
	    {Position_P_Gain, {594, 2}},
	    {Goal_Position, {596, 4}},
	    {Goal_Velocity, {600, 4}},
	    {Goal_Torque, {604, 2}},
	    {Goal_Acceleration, {606, 4}},
	    {Moving, {610, 1}},
	    {Present_Position, {611, 4}},
	    {Present_Velocity, {615, 4}},
	    {Present_Current, {621, 2}},
	    {Present_Input_Voltage, {623, 2}},
	    {Present_Temperature, {625, 1}},
	    {External_Port_Mode_1, {626, 2}},
	    {External_Port_Mode_2, {628, 2}},
	    {External_Port_Mode_3, {630, 2}},
	    {External_Port_Mode_4, {632, 2}},
	    {Registered_Instruction, {890, 1}},
	    {Status_Return_Level, {891, 1}},
	    {Hardware_Error_Status, {892, 1}}};

    #define COUNT_PRO_ITEMS (sizeof(items_PRO) / sizeof(items_PRO[0]))

    static const ModelInfo info_PRO = {0.114,
							 -2048,
							 0,
							 2048,
							 -3.14159265, 
							 3.14159265};
	
    //---------------------------------------------------------
    // EXT PRO - All Other Pros...
    //---------------------------------------------------------
    static std::map<field_name,field>  items_EXTPRO = {
	    {Model_Number, {0, 2}},
	    {Firmware_Version, {6, 1}},
	    {ID, {7, 1}},
	    {Baud_Rate, {8, 1}},
	    {Return_Delay_Time, {9, 1}},
	    {Operating_Mode, {11, 1}},
	    {Homing_Offset, {13, 4}},
	    {Moving_Threshold, {17, 4}},
	    {Temperature_Limit, {21, 1}},
	    {Max_Voltage_Limit, {22, 2}},
	    {Min_Voltage_Limit, {24, 2}},
	    {Acceleration_Limit, {26, 4}},
	    {Torque_Limit, {30, 2}},
	    {Velocity_Limit, {32, 4}},
	    {Max_Position_Limit, {36, 4}},
	    {Min_Position_Limit, {40, 4}},
	    {External_Port_Mode_1, {44, 1}},
	    {External_Port_Mode_2, {45, 1}},
	    {External_Port_Mode_3, {46, 1}},
	    {External_Port_Mode_4, {47, 1}},
	    {Shutdown, {48, 1}},

	    {Torque_Enable, {562, 1}},
	    {LED_RED, {563, 1}},
	    {LED_GREEN, {564, 1}},
	    {LED_BLUE, {565, 1}},
	    {Velocity_I_Gain, {586, 2}},
	    {Velocity_P_Gain, {588, 2}},
	    {Position_P_Gain, {594, 2}},
	    {Goal_Position, {596, 4}},
	    {Goal_Velocity, {600, 4}},
	    {Goal_Torque, {604, 2}},
	    {Goal_Acceleration, {606, 4}},
	    {Moving, {610, 1}},
	    {Present_Position, {611, 4}},
	    {Present_Velocity, {615, 4}},
	    {Present_Current, {621, 2}},
	    {Present_Input_Voltage, {623, 2}},
	    {Present_Temperature, {625, 1}},
	    {External_Port_Mode_1, {626, 2}},
	    {External_Port_Mode_2, {628, 2}},
	    {External_Port_Mode_3, {630, 2}},
	    {External_Port_Mode_4, {632, 2}},
	    {Registered_Instruction, {890, 1}},
	    {Status_Return_Level, {891, 1}},
	    {Hardware_Error_Status, {892, 1}}};

    #define COUNT_EXTPRO_ITEMS (sizeof(items_EXTPRO) / sizeof(items_EXTPRO[0]))

    static const ModelInfo info_EXTPRO[] = {
	   {0.00249657, -144197, 0, 144197, -3.14159265, 3.14159265},  // PRO_L54_30_S400_R
	   {0.00199234, -180692, 0, 180692, -3.14159265, 3.14159265},  // PRO_L54_30_S500_R, PRO_L54_50_S500_R
	   {0.00346667, -103846, 0, 103846, -3.14159265, 3.14159265},  // PRO_L54_50_S290_R
	   {0.00389076, -131593, 0, 131593, -3.14159265, 3.14159265},  // PRO_M42_10_S260_R
	   {0.00397746, -125708, 0, 125708, -3.14159265, 3.14159265},  // PRO_M54_40_S250_R, PRO_M54_60_S250_R
	   {0.00329218, -151875, 0, 151875, -3.14159265, 3.14159265},  // PRO_H42_20_S300_R
	   {0.00199234, -250961, 0, 250961, -3.14159265, 3.14159265}}; // PRO_H54_100_S500_R, PRO_H54_200_S500_R

    //---------------------------------------------------------
    // EXT PRO (A Firmware_Version) 
    //---------------------------------------------------------
    static std::map<field_name,field>  items_EXTPRO_A = {
	    {Model_Number, {        0, 2}},
	    {Firmware_Version, {    6, 1}},
	    {ID, {                  7, 1}},
	    {Baud_Rate, {           8, 1}},
	    {Return_Delay_Time, {   9, 1}},
	    {Operating_Mode, {      11, 1}},
	    {Homing_Offset, {       20, 4}},
	    {Moving_Threshold, {    24, 4}},
	    {Temperature_Limit, {   31, 1}},
	    {Max_Voltage_Limit, {   32, 2}},
	    {Min_Voltage_Limit, {   34, 2}},
	    {Current_Limit, {       38, 2}},
	    {Acceleration_Limit, {  40, 4}},
	    {Velocity_Limit, {      44, 4}},
	    {Max_Position_Limit, {  48, 4}},
	    {Min_Position_Limit, {  52, 4}},
	    {External_Port_Mode_1, {56, 1}},
	    {External_Port_Mode_2, {57, 1}},
	    {External_Port_Mode_3, {58, 1}},
	    {External_Port_Mode_4, {59, 1}},
	    {Shutdown, {            63, 1}},

	    {Torque_Enable, {         512, 1}},
	    {LED_RED, {               513, 1}},
	    {LED_GREEN, {             514, 1}},
	    {LED_BLUE, {              515, 1}},
	    {Velocity_I_Gain, {       524, 2}},
	    {Velocity_P_Gain, {       526, 2}},
	    {Position_D_Gain, {       528, 2}},
	    {Position_P_Gain, {       532, 2}},
	    {Position_I_Gain, {       530, 2}},
	    {Goal_Position, {         564, 4}},
	    {Goal_Velocity, {         552, 4}},
	    {Goal_Current, {          604, 2}},
	    {Profile_Acceleration, {  556, 4}},
	    {Profile_Velocity, {      560, 4}},
	    {Moving, {                570, 1}},
	    {Present_Position, {      580, 4}},
	    {Present_Velocity, {      576, 4}},
	    {Present_Current, {       574, 2}},
	    {Present_Input_Voltage, { 592, 2}},
	    {Present_Temperature, {   594, 1}},
	    {External_Port_Mode_1, {  600, 2}},
	    {External_Port_Mode_2, {  602, 2}},
	    {External_Port_Mode_3, {  604, 2}},
	    {External_Port_Mode_4, {  606, 2}}};

    #define COUNT_EXTPRO_A_ITEMS (sizeof(items_EXTPRO_A) / sizeof(items_EXTPRO_A[0]))

    static const ModelInfo info_EXTPRO_A[] = {
	   {0.00389076, -131593, 0, 131593, -3.14159265, 3.14159265},  // PRO_M42_10_S260_R_A
	   {0.00397746, -125708, 0, 125708, -3.14159265, 3.14159265},  // PRO_M54_40_S250_R_A, PRO_M54_60_S250_R_A
	   {0.00329218, -151875, 0, 151875, -3.14159265, 3.14159265},  // PRO_H42_20_S300_R_A
	   {0.00199234, -250961, 0, 250961, -3.14159265, 3.14159265}}; // PRO_H54_100_S500_R_A, PRO_H54_200_S500_R_A



    //---------------------------------------------------------
    // PRO PLUS - (num == PRO_H42P_020_S300_R, PRO_H54P_100_S500_R, PRO_H54P_200_S500_R)
    //---------------------------------------------------------
    static std::map<field_name,field>  items_PRO_PLUS = {
	    {Model_Number, {0, 2}},
	    {Firmware_Version, {6, 1}},
	    {ID, {7, 1}},
	    {Baud_Rate, {8, 1}},
	    {Return_Delay_Time, {9, 1}},
	    {Drive_Mode, {10, 1}},
	    {Operating_Mode, {11, 1}},
	    {Secondary_ID, {12, 1}},
	    {Homing_Offset, {20, 4}},
	    {Moving_Threshold, {24, 4}},
	    {Temperature_Limit, {31, 1}},
	    {Max_Voltage_Limit, {32, 2}},
	    {Min_Voltage_Limit, {34, 2}},
	    {PWM_Limit, {36, 2}},
	    {Current_Limit, {38, 2}},
	    {Acceleration_Limit, {40, 4}},
	    {Velocity_Limit, {44, 4}},
	    {Max_Position_Limit, {48, 4}},
	    {Min_Position_Limit, {52, 4}},
	    {External_Port_Mode_1, {56, 1}},
	    {External_Port_Mode_2, {57, 1}},
	    {External_Port_Mode_3, {58, 1}},
	    {External_Port_Mode_4, {59, 1}},
	    {Shutdown, {63, 1}},

	    {Torque_Enable, {512, 1}},
	    {LED_RED, {513, 1}},
	    {LED_GREEN, {514, 1}},
	    {LED_BLUE, {515, 1}},
	    {Status_Return_Level, {516, 1}},
	    {Registered_Instruction, {517, 1}},
	    {Hardware_Error_Status, {518, 1}},
	    {Velocity_I_Gain, {524, 2}},
	    {Velocity_P_Gain, {526, 2}},
	    {Position_D_Gain, {528, 2}},
	    {Position_I_Gain, {530, 2}},
	    {Position_P_Gain, {532, 2}},
	    {Feedforward_2nd_Gain, {536, 2}},
	    {Feedforward_1st_Gain, {538, 2}},
	    {Bus_Watchdog, {546, 1}},
	    {Goal_PWM, {548, 2}},
	    {Goal_Current, {550, 2}},
	    {Goal_Velocity, {552, 4}},
	    {Profile_Acceleration, {556, 4}},
	    {Profile_Velocity, {560, 4}},
	    {Goal_Position, {564, 4}},    
	    {Realtime_Tick, {568, 2}},
	    {Moving, {570, 1}},
	    {Moving_Status, {571, 1}},
	    {Present_PWM, {572, 2}},
	    {Present_Current, {574, 2}},
	    {Present_Velocity, {576, 4}},
	    {Present_Position, {580, 4}},
	    {Velocity_Trajectory, {584, 4}},
	    {Position_Trajectory, {588, 4}},    
	    {Present_Input_Voltage, {592, 2}},
	    {Present_Temperature, {594, 1}},
	    {External_Port_Mode_1, {600, 2}},
	    {External_Port_Mode_2, {602, 2}},
	    {External_Port_Mode_3, {604, 2}},
	    {External_Port_Mode_4, {606, 2}}};

    #define COUNT_EXTPRO_PLUS_ITEMS (sizeof(items_PRO_PLUS) / sizeof(items_PRO_PLUS[0]))

    static const ModelInfo info_PRO_PLUS[] = {
	   {0.01, -251173, 0, 251173, -3.14159265, 3.14159265},  // PRO_PLUS_M42P_010_S260_R
	   {0.01, -251173, 0, 251173, -3.14159265, 3.14159265},  // PRO_PLUS_M54P_040_S250_R
	   {0.01, -262931, 0, 262931, -3.14159265, 3.14159265},  // PRO_PLUS_M54P_060_S250_R
	   {0.01, -303454, 0, 303454, -3.14159265, 3.14159265},  // PRO_PLUS_H42P_020_S300_R
	   {0.01, -501433, 0, 501433, -3.14159265, 3.14159265},  // PRO_PLUS_H54P_100_S500_R
	   {0.01, -501433, 0, 501433, -3.14159265, 3.14159265}}; // PRO_PLUS_H54P_200_S500_R



    //---------------------------------------------------------
    // Gripper - (num == RH_P12_RN)
    //---------------------------------------------------------
    static std::map<field_name,field>  items_Gripper = {
	    {Model_Number, {0, 2}},
	    {Firmware_Version, {6, 1}},
	    {ID, {7, 1}},
	    {Baud_Rate, {8, 1}},
	    {Return_Delay_Time, {9, 1}},
	    {Operating_Mode, {11, 1}},
	    {Homing_Offset, {13, 4}},
	    {Moving_Threshold, {17, 4}},
	    {Temperature_Limit, {21, 1}},
	    {Max_Voltage_Limit, {22, 2}},
	    {Min_Voltage_Limit, {24, 2}},
	    {Acceleration_Limit, {26, 4}},
	    {Torque_Limit, {30, 2}},
	    {Velocity_Limit, {32, 4}},
	    {Max_Position_Limit, {36, 4}},
	    {Min_Position_Limit, {40, 4}},
	    {External_Port_Mode_1, {44, 1}},
	    {External_Port_Mode_2, {45, 1}},
	    {External_Port_Mode_3, {46, 1}},
	    {External_Port_Mode_4, {47, 1}},
	    {Shutdown, {48, 1}},

	    {Torque_Enable, {562, 1}},
	    {LED_RED, {563, 1}},
	    {LED_GREEN, {564, 1}},
	    {LED_BLUE, {565, 1}},
	    {Velocity_I_Gain, {586, 2}},
	    {Velocity_P_Gain, {588, 2}},
	    {Position_P_Gain, {594, 2}},
	    {Goal_Position, {596, 4}},
	    {Goal_Velocity, {600, 4}},
	    {Goal_Torque, {604, 2}},
	    {Goal_Acceleration, {606, 4}},
	    {Moving, {610, 1}},
	    {Present_Position, {611, 4}},
	    {Present_Velocity, {615, 4}},
	    {Present_Current, {621, 2}},
	    {Present_Input_Voltage, {623, 2}},
	    {Present_Temperature, {625, 1}},
	    {External_Port_Mode_1, {626, 2}},
	    {External_Port_Mode_2, {628, 2}},
	    {External_Port_Mode_3, {630, 2}},
	    {External_Port_Mode_4, {632, 2}},
	    {Registered_Instruction, {890, 1}},
	    {Status_Return_Level, {891, 1}},
	    {Hardware_Error_Status, {892, 1}}};
    #define COUNT_Gripper_ITEMS (sizeof(items_Gripper) / sizeof(items_Gripper[0]))

    static const ModelInfo info_Gripper = {0.01,
								0,
								0,
								1150,
								0, 
								1.7631835937};

    //---------------------------------------------------------
    // Gripper A Firmware - (num == RH_P12_RN_A)
    //---------------------------------------------------------
    static std::map<field_name,field>  items_EXTGripper = {
	    {Model_Number, {0, 2}},
	    {Firmware_Version, {6, 1}},
	    {ID, {7, 1}},
	    {Baud_Rate, {8, 1}},
	    {Return_Delay_Time, {9, 1}},
	    {Drive_Mode, {10, 1}},
	    {Operating_Mode, {11, 1}},
	    {Secondary_ID, {12, 1}},
	    {Homing_Offset, {20, 4}},
	    {Moving_Threshold, {24, 4}},
	    {Temperature_Limit, {31, 1}},
	    {Max_Voltage_Limit, {32, 2}},
	    {Min_Voltage_Limit, {34, 2}},
	    {PWM_Limit, {36, 2}},
	    {Current_Limit, {38, 2}},
	    {Acceleration_Limit, {40, 4}},
	    {Velocity_Limit, {44, 4}},
	    {Max_Position_Limit, {48, 4}},
	    {Min_Position_Limit, {52, 4}},
	    {External_Port_Mode_1, {56, 1}},
	    {External_Port_Mode_2, {57, 1}},
	    {External_Port_Mode_3, {58, 1}},
	    {External_Port_Mode_4, {59, 1}},
	    {Shutdown, {63, 1}},

	    {Torque_Enable, {512, 1}},
	    {LED_RED, {513, 1}},
	    {LED_GREEN, {514, 1}},
	    {LED_BLUE, {515, 1}},
	    {Status_Return_Level, {516, 1}},
	    {Registered_Instruction, {517, 1}},
	    {Hardware_Error_Status, {518, 1}},
	    {Velocity_I_Gain, {524, 2}},
	    {Velocity_P_Gain, {526, 2}},
	    {Position_D_Gain, {528, 2}},
	    {Position_I_Gain, {530, 2}},
	    {Position_P_Gain, {532, 2}},
	    {Feedforward_2nd_Gain, {536, 2}},
	    {Feedforward_1st_Gain, {538, 2}},
	    {Bus_Watchdog, {546, 1}},
	    {Goal_PWM, {548, 2}},
	    {Goal_Current, {550, 2}},
	    {Goal_Velocity, {552, 4}},
	    {Profile_Acceleration, {556, 4}},
	    {Profile_Velocity, {560, 4}},
	    {Goal_Position, {564, 4}},    
	    {Realtime_Tick, {568, 2}},
	    {Moving, {570, 1}},
	    {Moving_Status, {571, 1}},
	    {Present_PWM, {572, 2}},
	    {Present_Current, {574, 2}},
	    {Present_Velocity, {576, 4}},
	    {Present_Position, {580, 4}},
	    {Velocity_Trajectory, {584, 4}},
	    {Position_Trajectory, {588, 4}},    
	    {Present_Input_Voltage, {592, 2}},
	    {Present_Temperature, {594, 1}},
	    {External_Port_Mode_1, {600, 2}},
	    {External_Port_Mode_2, {602, 2}},
	    {External_Port_Mode_3, {604, 2}},
	    {External_Port_Mode_4, {606, 2}}
    };




	
	static const std::map<model_t,model_info> model_infos = {
		{AX_12A,
			{AX_12A,0.11,0,1024,-2.61799,2.61799,&items_AX}},
		{AX_12W,
			{AX_12W,0.11,0,1024,-2.61799,2.61799,&items_AX}},
		{AX_18A,
			{AX_18A,0.11,0,1024,-2.61799,2.61799,&items_AX}},
		{XL430_W250,
			{XL430_W250,0.229,0,4096,-3.14159265,3.14159265,&items_XL}},
		{XL430_W250_2,	
			{XL430_W250_2,	0.229,0,4096,-3.14159265,3.14159265,&items_XL}},
		{XC430_W150,	
			{XC430_W150,0.229,0,4096,-3.14159265,3.14159265,&items_XL}},
		{XC430_W240,	
			{XC430_W240,0.229,0,4096,-3.14159265,3.14159265,&items_XL}},
	};

	/*static const std::map<const model_t,const model_info*> model_infos = {
		{AX_12A,&infos_AX_12A},
		{AX_12W,&infos_AX_12W},
		{AX_18A,&infos_AX_18A},

		{XL430_W250,&infos_XL430_W250},
		{XL430_W250_2,&infos_XL430_W250_2},
		{XC430_W150,&infos_XC430_W150},
		{XC430_W240,&infos_XC430_W240}
	};*/

		/*	std::map<field_name,field>* get_command_table() {
				switch(type) {
					case AX_12A:
					case AX_12W:
					case AX_18A:
						return &items_AX;
					case RX_10:
					case RX_24F:
					case RX_28:
					case RX_64:
						return &items_RX;
					case EX_106:
						return &items_EX;
					case MX_12W:
					case MX_28:
						return &items_MX;
					case MX_28_2:
						return &items_MX2;
					case XL_320:
						return &items_XL320;	
					case XL430_W250: 
					case XL430_W250_2:
					case XC430_W150:
					case XC430_W240:
						return &items_XL;
					case XM430_W210:
					case XM430_W350:
						return &items_XM;
					case XM540_W150:
					case XM540_W270:
						return &items_EXTXM;
					case XH430_V210:
					case XH430_V350:
						return &items_XH;
					case XH540_W150: 
					case XH540_W270:
					case XH540_V150:
					case XH540_V270:
						return &items_EXTXH;
					case XW540_T260:
					case XW540_T140:
						return &items_XW;
					case PRO_L42_10_S300_R:
						return &items_PRO;
					//TODO add missing dynamixels types
					default:
						return 0;
				}
			}*/

		    /*  
		  std::map<uint32_t,dynamixel::ModelInfo*> info_tables;
		  dynamixel::info_tables.insert(std::pair<uint32_t,dynamixel::ModelInfo*>(AX_12A,&dynamixel::info_AX)));
		  dynamixel::info_tables.insert(std::pair<uint32_t,dynamixel::ModelInfo*>(AX_12W,&dynamixel::info_AX));
		  dynamixel::info_tables.insert(std::pair<uint32_t,dynamixel::ModelInfo*>(AX_18A,&dynamixel::info_AX));

		  dynamixel::info_tables.insert(std::pair<uint32_t,dynamixel::ModelInfo*>(RX_10,&dynamixel::info_RX));
		  dynamixel::info_tables.insert(std::pair<uint32_t,dynamixel::ModelInfo*>(RX_24F,&dynamixel::info_RX));
		  dynamixel::info_tables.insert(std::pair<uint32_t,dynamixel::ModelInfo*>(RX_28,&dynamixel::info_RX));
		  dynamixel::info_tables.insert(std::pair<uint32_t,dynamixel::ModelInfo*>(RX_64,&dynamixel::info_RX));

		  dynamixel::info_tables.insert(std::pair<uint32_t,dynamixel::ModelInfo*>(EX_106,&dynamixel::info_EX));

		  dynamixel::info_tables.insert(std::pair<uint32_t,dynamixel::ModelInfo*>(MX_12W,&dynamixel::info_MX));
		  dynamixel::info_tables.insert(std::pair<uint32_t,dynamixel::ModelInfo*>(MX_28,&dynamixel::info_MX));

		  dynamixel::info_tables.insert(std::pair<uint32_t,dynamixel::ModelInfo*>(MX_28_2,&dynamixel::info_MX2));

		  dynamixel::info_tables.insert(std::pair<uint32_t,dynamixel::ModelInfo*>(MX_64,&dynamixel::info_EXTMX));
		  dynamixel::info_tables.insert(std::pair<uint32_t,dynamixel::ModelInfo*>(MX_106,&dynamixel::info_EXTMX));

		  dynamixel::info_tables.insert(std::pair<uint32_t,dynamixel::ModelInfo*>(MX_64_2,&dynamixel::info_EXTMX2));
		  dynamixel::info_tables.insert(std::pair<uint32_t,dynamixel::ModelInfo*>(MX_106_2,&dynamixel::info_EXTMX2));

		  dynamixel::info_tables.insert(std::pair<uint32_t,dynamixel::ModelInfo*>(XL_320,&dynamixel::info_XL320));

		  dynamixel::info_tables.insert(std::pair<uint32_t,dynamixel::ModelInfo*>(XL430_W250,&dynamixel::info_XL));
		  dynamixel::info_tables.insert(std::pair<uint32_t,dynamixel::ModelInfo*>(XL430_W250_2,&dynamixel::info_XL));
		  dynamixel::info_tables.insert(std::pair<uint32_t,dynamixel::ModelInfo*>(XC430_W150,&dynamixel::info_XL));
		  dynamixel::info_tables.insert(std::pair<uint32_t,dynamixel::ModelInfo*>(XC430_W240,&dynamixel::info_XL));

		  dynamixel::info_tables.insert(std::pair<uint32_t,dynamixel::ModelInfo*>(XM430_W210,&dynamixel::info_XM));
		  dynamixel::info_tables.insert(std::pair<uint32_t,dynamixel::ModelInfo*>(XM430_W350,&dynamixel::info_XM));

		  dynamixel::info_tables.insert(std::pair<uint32_t,dynamixel::ModelInfo*>(XM540_W150,&dynamixel::info_EXTXM));
		  dynamixel::info_tables.insert(std::pair<uint32_t,dynamixel::ModelInfo*>(XM540_W270,&dynamixel::info_EXTXM));

		  dynamixel::info_tables.insert(std::pair<uint32_t,dynamixel::ModelInfo*>(XH430_V210,&dynamixel::info_XH));
		  dynamixel::info_tables.insert(std::pair<uint32_t,dynamixel::ModelInfo*>(XH430_V350,&dynamixel::info_XH));
		  dynamixel::info_tables.insert(std::pair<uint32_t,dynamixel::ModelInfo*>(XH430_W210,&dynamixel::info_XH));
		  dynamixel::info_tables.insert(std::pair<uint32_t,dynamixel::ModelInfo*>(XH430_W350,&dynamixel::info_XH));

		  dynamixel::info_tables.insert(std::pair<uint32_t,dynamixel::ModelInfo*>(XH540_W150,&dynamixel::info_XH));
		  dynamixel::info_tables.insert(std::pair<uint32_t,dynamixel::ModelInfo*>(XH540_W270,&dynamixel::info_XH));
		  dynamixel::info_tables.insert(std::pair<uint32_t,dynamixel::ModelInfo*>(XH540_V150,&dynamixel::info_XH));
		  dynamixel::info_tables.insert(std::pair<uint32_t,dynamixel::ModelInfo*>(XH540_W270,&dynamixel::info_XH));

		  dynamixel::info_tables.insert(std::pair<uint32_t,dynamixel::ModelInfo*>(XW540_T260,&dynamixel::info_XW));
		  dynamixel::info_tables.insert(std::pair<uint32_t,dynamixel::ModelInfo*>(XW540_T140,&dynamixel::info_XW));

		  dynamixel::info_tables.insert(std::pair<uint32_t,dynamixel::ModelInfo*>(PRO_L42_10_S300_R,&dynamixel::info_PRO));

		  dynamixel::info_tables.insert(std::pair<uint32_t,dynamixel::ModelInfo*>(PRO_L54_30_S400_R,&dynamixel::info_EXTPRO));
		  dynamixel::info_tables.insert(std::pair<uint32_t,dynamixel::ModelInfo*>(PRO_L54_30_S500_R,&dynamixel::info_EXTPRO));
		  dynamixel::info_tables.insert(std::pair<uint32_t,dynamixel::ModelInfo*>(PRO_L54_50_S290_R,&dynamixel::info_EXTPRO));
		  dynamixel::info_tables.insert(std::pair<uint32_t,dynamixel::ModelInfo*>(PRO_L54_50_S500_R,&dynamixel::info_EXTPRO));

		  dynamixel::info_tables.insert(std::pair<uint32_t,dynamixel::ModelInfo*>(PRO_M42_10_S260_R,&dynamixel::info_EXTPRO));
		  dynamixel::info_tables.insert(std::pair<uint32_t,dynamixel::ModelInfo*>(PRO_M54_40_S250_R,&dynamixel::info_EXTPRO));
		  dynamixel::info_tables.insert(std::pair<uint32_t,dynamixel::ModelInfo*>(PRO_M54_60_S250_R,&dynamixel::info_EXTPRO));

		  dynamixel::info_tables.insert(std::pair<uint32_t,dynamixel::ModelInfo*>(PRO_H42_20_S300_R,&dynamixel::info_EXTPRO));
		  dynamixel::info_tables.insert(std::pair<uint32_t,dynamixel::ModelInfo*>(PRO_H54_100_S500_R,&dynamixel::info_EXTPRO));
		  dynamixel::info_tables.insert(std::pair<uint32_t,dynamixel::ModelInfo*>(PRO_H54_200_S500_R,&dynamixel::info_EXTPRO));

		  dynamixel::info_tables.insert(std::pair<uint32_t,dynamixel::ModelInfo*>(PRO_M42_10_S260_R_A,&dynamixel::info_EXTPRO_A));
		  dynamixel::info_tables.insert(std::pair<uint32_t,dynamixel::ModelInfo*>(PRO_M54_40_S250_R_A,&dynamixel::info_EXTPRO_A));
		  dynamixel::info_tables.insert(std::pair<uint32_t,dynamixel::ModelInfo*>(PRO_M54_60_S250_R_A,&dynamixel::info_EXTPRO_A));

		  dynamixel::info_tables.insert(std::pair<uint32_t,dynamixel::ModelInfo*>(PRO_H42_20_S300_R_A,&dynamixel::info_EXTPRO_A));
		  dynamixel::info_tables.insert(std::pair<uint32_t,dynamixel::ModelInfo*>(PRO_H54_100_S500_R_A,&dynamixel::info_EXTPRO_A));
		  dynamixel::info_tables.insert(std::pair<uint32_t,dynamixel::ModelInfo*>(PRO_H54_200_S500_R_A,&dynamixel::info_EXTPRO_A));

		  dynamixel::info_tables.insert(std::pair<uint32_t,dynamixel::ModelInfo*>(PRO_PLUS_M42P_010_S260_R,&dynamixel::info_EXTPRO));
		  dynamixel::info_tables.insert(std::pair<uint32_t,dynamixel::ModelInfo*>(PRO_PLUS_M54P_040_S250_R,&dynamixel::info_EXTPRO));
		  dynamixel::info_tables.insert(std::pair<uint32_t,dynamixel::ModelInfo*>(PRO_PLUS_M54P_060_S250_R,&dynamixel::info_EXTPRO));

		  dynamixel::info_tables.insert(std::pair<uint32_t,dynamixel::ModelInfo*>(PRO_PLUS_H42P_020_S300_R,&dynamixel::info_PRO_PLUS));
		  dynamixel::info_tables.insert(std::pair<uint32_t,dynamixel::ModelInfo*>(PRO_PLUS_H54P_100_S500_R,&dynamixel::info_PRO_PLUS));
		  dynamixel::info_tables.insert(std::pair<uint32_t,dynamixel::ModelInfo*>(PRO_PLUS_H54P_200_S500_R,&dynamixel::info_PRO_PLUS));

		  dynamixel::info_tables.insert(std::pair<uint32_t,dynamixel::ModelInfo*>(RH_P12_RN,&dynamixel::info_Gripper));

		  dynamixel::info_tables.insert(std::pair<uint32_t,dynamixel::ModelInfo*>(RH_P12_RN_A,&dynamixel::info_Gripper));

		  std::map<dynamixel::model_t,std::map<std::string,dynamixel::field>*> control_tables;
		  control_tables.insert(std::pair<dynamixel::model_t,std::map<std::string,dynamixel::field>*>(AX_12A,&dynamixel::items_AX));
		  control_tables.insert(std::pair<dynamixel::model_t,std::map<std::string,dynamixel::field>*>(AX_12W,&dynamixel::items_AX));
		  control_tables.insert(std::pair<dynamixel::model_t,std::map<std::string,dynamixel::field>*>(AX_18A,&dynamixel::items_AX));

		  control_tables.insert(std::pair<dynamixel::model_t,std::map<std::string,dynamixel::field>*>(RX_10,&dynamixel::items_RX));
		  control_tables.insert(std::pair<dynamixel::model_t,std::map<std::string,dynamixel::field>*>(RX_24F,&dynamixel::items_RX));
		  control_tables.insert(std::pair<dynamixel::model_t,std::map<std::string,dynamixel::field>*>(RX_28,&dynamixel::items_RX));
		  control_tables.insert(std::pair<dynamixel::model_t,std::map<std::string,dynamixel::field>*>(RX_64,&dynamixel::items_RX));

		  control_tables.insert(std::pair<dynamixel::model_t,std::map<std::string,dynamixel::field>*>(EX_106,&dynamixel::items_EX));

		  control_tables.insert(std::pair<dynamixel::model_t,std::map<std::string,dynamixel::field>*>(MX_12W,&dynamixel::items_MX));
		  control_tables.insert(std::pair<dynamixel::model_t,std::map<std::string,dynamixel::field>*>(MX_28,&dynamixel::items_MX));

		  control_tables.insert(std::pair<dynamixel::model_t,std::map<std::string,dynamixel::field>*>(MX_28_2,&dynamixel::items_MX2));

		  control_tables.insert(std::pair<dynamixel::model_t,std::map<std::string,dynamixel::field>*>(MX_64,&dynamixel::items_EXTMX));
		  control_tables.insert(std::pair<dynamixel::model_t,std::map<std::string,dynamixel::field>*>(MX_106,&dynamixel::items_EXTMX));

		  control_tables.insert(std::pair<dynamixel::model_t,std::map<std::string,dynamixel::field>*>(MX_64_2,&dynamixel::items_EXTMX2));
		  control_tables.insert(std::pair<dynamixel::model_t,std::map<std::string,dynamixel::field>*>(MX_106_2,&dynamixel::items_EXTMX2));

		  control_tables.insert(std::pair<dynamixel::model_t,std::map<std::string,dynamixel::field>*>(XL_320,&dynamixel::items_XL320));

		  control_tables.insert(std::pair<dynamixel::model_t,std::map<std::string,dynamixel::field>*>(XL430_W250,&dynamixel::items_XL));
		  control_tables.insert(std::pair<dynamixel::model_t,std::map<std::string,dynamixel::field>*>(XL430_W250_2,&dynamixel::items_XL));
		  control_tables.insert(std::pair<dynamixel::model_t,std::map<std::string,dynamixel::field>*>(XC430_W150,&dynamixel::items_XL));
		  control_tables.insert(std::pair<dynamixel::model_t,std::map<std::string,dynamixel::field>*>(XC430_W240,&dynamixel::items_XL));

		  control_tables.insert(std::pair<dynamixel::model_t,std::map<std::string,dynamixel::field>*>(XM430_W210,&dynamixel::items_XM));
		  control_tables.insert(std::pair<dynamixel::model_t,std::map<std::string,dynamixel::field>*>(XM430_W350,&dynamixel::items_XM));

		  control_tables.insert(std::pair<dynamixel::model_t,std::map<std::string,dynamixel::field>*>(XM540_W150,&dynamixel::items_EXTXM));
		  control_tables.insert(std::pair<dynamixel::model_t,std::map<std::string,dynamixel::field>*>(XM540_W270,&dynamixel::items_EXTXM));

		  control_tables.insert(std::pair<dynamixel::model_t,std::map<std::string,dynamixel::field>*>(XH430_V210,&dynamixel::items_XH));
		  control_tables.insert(std::pair<dynamixel::model_t,std::map<std::string,dynamixel::field>*>(XH430_V350,&dynamixel::items_XH));
		  control_tables.insert(std::pair<dynamixel::model_t,std::map<std::string,dynamixel::field>*>(XH430_W210,&dynamixel::items_XH));
		  control_tables.insert(std::pair<dynamixel::model_t,std::map<std::string,dynamixel::field>*>(XH430_W350,&dynamixel::items_XH));

		  control_tables.insert(std::pair<dynamixel::model_t,std::map<std::string,dynamixel::field>*>(XH540_W150,&dynamixel::items_XH));
		  control_tables.insert(std::pair<dynamixel::model_t,std::map<std::string,dynamixel::field>*>(XH540_W270,&dynamixel::items_XH));
		  control_tables.insert(std::pair<dynamixel::model_t,std::map<std::string,dynamixel::field>*>(XH540_V150,&dynamixel::items_XH));
		  control_tables.insert(std::pair<dynamixel::model_t,std::map<std::string,dynamixel::field>*>(XH540_W270,&dynamixel::items_XH));

		  control_tables.insert(std::pair<dynamixel::model_t,std::map<std::string,dynamixel::field>*>(XW540_T260,&dynamixel::items_XW));
		  control_tables.insert(std::pair<dynamixel::model_t,std::map<std::string,dynamixel::field>*>(XW540_T140,&dynamixel::items_XW));

		  control_tables.insert(std::pair<dynamixel::model_t,std::map<std::string,dynamixel::field>*>(PRO_L42_10_S300_R,&dynamixel::items_PRO));

		  control_tables.insert(std::pair<dynamixel::model_t,std::map<std::string,dynamixel::field>*>(PRO_L54_30_S400_R,&dynamixel::items_EXTPRO));
		  control_tables.insert(std::pair<dynamixel::model_t,std::map<std::string,dynamixel::field>*>(PRO_L54_30_S500_R,&dynamixel::items_EXTPRO));
		  control_tables.insert(std::pair<dynamixel::model_t,std::map<std::string,dynamixel::field>*>(PRO_L54_50_S290_R,&dynamixel::items_EXTPRO));
		  control_tables.insert(std::pair<dynamixel::model_t,std::map<std::string,dynamixel::field>*>(PRO_L54_50_S500_R,&dynamixel::items_EXTPRO));

		  control_tables.insert(std::pair<dynamixel::model_t,std::map<std::string,dynamixel::field>*>(PRO_M42_10_S260_R,&dynamixel::items_EXTPRO));
		  control_tables.insert(std::pair<dynamixel::model_t,std::map<std::string,dynamixel::field>*>(PRO_M54_40_S250_R,&dynamixel::items_EXTPRO));
		  control_tables.insert(std::pair<dynamixel::model_t,std::map<std::string,dynamixel::field>*>(PRO_M54_60_S250_R,&dynamixel::items_EXTPRO));

		  control_tables.insert(std::pair<dynamixel::model_t,std::map<std::string,dynamixel::field>*>(PRO_H42_20_S300_R,&dynamixel::items_EXTPRO));
		  control_tables.insert(std::pair<dynamixel::model_t,std::map<std::string,dynamixel::field>*>(PRO_H54_100_S500_R,&dynamixel::items_EXTPRO));
		  control_tables.insert(std::pair<dynamixel::model_t,std::map<std::string,dynamixel::field>*>(PRO_H54_200_S500_R,&dynamixel::items_EXTPRO));

		  control_tables.insert(std::pair<dynamixel::model_t,std::map<std::string,dynamixel::field>*>(PRO_M42_10_S260_R_A,&dynamixel::items_EXTPRO_A));
		  control_tables.insert(std::pair<dynamixel::model_t,std::map<std::string,dynamixel::field>*>(PRO_M54_40_S250_R_A,&dynamixel::items_EXTPRO_A));
		  control_tables.insert(std::pair<dynamixel::model_t,std::map<std::string,dynamixel::field>*>(PRO_M54_60_S250_R_A,&dynamixel::items_EXTPRO_A));

		  control_tables.insert(std::pair<dynamixel::model_t,std::map<std::string,dynamixel::field>*>(PRO_H42_20_S300_R_A,&dynamixel::items_EXTPRO_A));
		  control_tables.insert(std::pair<dynamixel::model_t,std::map<std::string,dynamixel::field>*>(PRO_H54_100_S500_R_A,&dynamixel::items_EXTPRO_A));
		  control_tables.insert(std::pair<dynamixel::model_t,std::map<std::string,dynamixel::field>*>(PRO_H54_200_S500_R_A,&dynamixel::items_EXTPRO_A));

		  control_tables.insert(std::pair<dynamixel::model_t,std::map<std::string,dynamixel::field>*>(PRO_PLUS_M42P_010_S260_R,&dynamixel::items_EXTPRO));
		  control_tables.insert(std::pair<dynamixel::model_t,std::map<std::string,dynamixel::field>*>(PRO_PLUS_M54P_040_S250_R,&dynamixel::items_EXTPRO));
		  control_tables.insert(std::pair<dynamixel::model_t,std::map<std::string,dynamixel::field>*>(PRO_PLUS_M54P_060_S250_R,&dynamixel::items_EXTPRO));

		  control_tables.insert(std::pair<dynamixel::model_t,std::map<std::string,dynamixel::field>*>(PRO_PLUS_H42P_020_S300_R,&dynamixel::items_PRO_PLUS));
		  control_tables.insert(std::pair<dynamixel::model_t,std::map<std::string,dynamixel::field>*>(PRO_PLUS_H54P_100_S500_R,&dynamixel::items_PRO_PLUS));
		  control_tables.insert(std::pair<dynamixel::model_t,std::map<std::string,dynamixel::field>*>(PRO_PLUS_H54P_200_S500_R,&dynamixel::items_PRO_PLUS));

		  control_tables.insert(std::pair<dynamixel::model_t,std::map<std::string,dynamixel::field>*>(RH_P12_RN,&dynamixel::items_Gripper));

		  control_tables.insert(std::pair<dynamixel::model_t,std::map<std::string,dynamixel::field>*>(RH_P12_RN_A,&dynamixel::items_EXTGripper));
*/
}