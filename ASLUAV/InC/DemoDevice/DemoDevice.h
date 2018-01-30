
#pragma once

#include "BlackBox/Host.h"
#include <uchar.h>
INLINER size_t strlen16(register const char16_t * string)
{
    if(!string) return 0;
    register size_t len = 0;
    while(string[len])len++;
    return len;
}
Pack * c_LoopBackDemoChannel_new_HEARTBEAT_0();
Pack * c_LoopBackDemoChannel_ADV_new_HEARTBEAT_0();
extern void c_LoopBackDemoChannel_on_HEARTBEAT_0(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_SYS_STATUS_1();
Pack * c_LoopBackDemoChannel_ADV_new_SYS_STATUS_1();
extern void c_LoopBackDemoChannel_on_SYS_STATUS_1(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_SYSTEM_TIME_2();
Pack * c_LoopBackDemoChannel_ADV_new_SYSTEM_TIME_2();
extern void c_LoopBackDemoChannel_on_SYSTEM_TIME_2(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_POSITION_TARGET_LOCAL_NED_3();
Pack * c_LoopBackDemoChannel_ADV_new_POSITION_TARGET_LOCAL_NED_3();
extern void c_LoopBackDemoChannel_on_POSITION_TARGET_LOCAL_NED_3(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_PING_4();
Pack * c_LoopBackDemoChannel_ADV_new_PING_4();
extern void c_LoopBackDemoChannel_on_PING_4(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_CHANGE_OPERATOR_CONTROL_5();
Pack * c_LoopBackDemoChannel_ADV_new_CHANGE_OPERATOR_CONTROL_5();
extern void c_LoopBackDemoChannel_on_CHANGE_OPERATOR_CONTROL_5(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_CHANGE_OPERATOR_CONTROL_ACK_6();
Pack * c_LoopBackDemoChannel_ADV_new_CHANGE_OPERATOR_CONTROL_ACK_6();
extern void c_LoopBackDemoChannel_on_CHANGE_OPERATOR_CONTROL_ACK_6(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_AUTH_KEY_7();
Pack * c_LoopBackDemoChannel_ADV_new_AUTH_KEY_7();
extern void c_LoopBackDemoChannel_on_AUTH_KEY_7(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_SET_MODE_11();
Pack * c_LoopBackDemoChannel_ADV_new_SET_MODE_11();
extern void c_LoopBackDemoChannel_on_SET_MODE_11(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_PARAM_REQUEST_READ_20();
Pack * c_LoopBackDemoChannel_ADV_new_PARAM_REQUEST_READ_20();
extern void c_LoopBackDemoChannel_on_PARAM_REQUEST_READ_20(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_PARAM_REQUEST_LIST_21();
Pack * c_LoopBackDemoChannel_ADV_new_PARAM_REQUEST_LIST_21();
extern void c_LoopBackDemoChannel_on_PARAM_REQUEST_LIST_21(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_PARAM_VALUE_22();
Pack * c_LoopBackDemoChannel_ADV_new_PARAM_VALUE_22();
extern void c_LoopBackDemoChannel_on_PARAM_VALUE_22(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_PARAM_SET_23();
Pack * c_LoopBackDemoChannel_ADV_new_PARAM_SET_23();
extern void c_LoopBackDemoChannel_on_PARAM_SET_23(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_GPS_RAW_INT_24();
Pack * c_LoopBackDemoChannel_ADV_new_GPS_RAW_INT_24();
extern void c_LoopBackDemoChannel_on_GPS_RAW_INT_24(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_GPS_STATUS_25();
Pack * c_LoopBackDemoChannel_ADV_new_GPS_STATUS_25();
extern void c_LoopBackDemoChannel_on_GPS_STATUS_25(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_SCALED_IMU_26();
Pack * c_LoopBackDemoChannel_ADV_new_SCALED_IMU_26();
extern void c_LoopBackDemoChannel_on_SCALED_IMU_26(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_RAW_IMU_27();
Pack * c_LoopBackDemoChannel_ADV_new_RAW_IMU_27();
extern void c_LoopBackDemoChannel_on_RAW_IMU_27(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_RAW_PRESSURE_28();
Pack * c_LoopBackDemoChannel_ADV_new_RAW_PRESSURE_28();
extern void c_LoopBackDemoChannel_on_RAW_PRESSURE_28(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_SCALED_PRESSURE_29();
Pack * c_LoopBackDemoChannel_ADV_new_SCALED_PRESSURE_29();
extern void c_LoopBackDemoChannel_on_SCALED_PRESSURE_29(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_ATTITUDE_30();
Pack * c_LoopBackDemoChannel_ADV_new_ATTITUDE_30();
extern void c_LoopBackDemoChannel_on_ATTITUDE_30(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_ATTITUDE_QUATERNION_31();
Pack * c_LoopBackDemoChannel_ADV_new_ATTITUDE_QUATERNION_31();
extern void c_LoopBackDemoChannel_on_ATTITUDE_QUATERNION_31(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_LOCAL_POSITION_NED_32();
Pack * c_LoopBackDemoChannel_ADV_new_LOCAL_POSITION_NED_32();
extern void c_LoopBackDemoChannel_on_LOCAL_POSITION_NED_32(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_GLOBAL_POSITION_INT_33();
Pack * c_LoopBackDemoChannel_ADV_new_GLOBAL_POSITION_INT_33();
extern void c_LoopBackDemoChannel_on_GLOBAL_POSITION_INT_33(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_RC_CHANNELS_SCALED_34();
Pack * c_LoopBackDemoChannel_ADV_new_RC_CHANNELS_SCALED_34();
extern void c_LoopBackDemoChannel_on_RC_CHANNELS_SCALED_34(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_RC_CHANNELS_RAW_35();
Pack * c_LoopBackDemoChannel_ADV_new_RC_CHANNELS_RAW_35();
extern void c_LoopBackDemoChannel_on_RC_CHANNELS_RAW_35(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_SERVO_OUTPUT_RAW_36();
Pack * c_LoopBackDemoChannel_ADV_new_SERVO_OUTPUT_RAW_36();
extern void c_LoopBackDemoChannel_on_SERVO_OUTPUT_RAW_36(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_MISSION_REQUEST_PARTIAL_LIST_37();
Pack * c_LoopBackDemoChannel_ADV_new_MISSION_REQUEST_PARTIAL_LIST_37();
extern void c_LoopBackDemoChannel_on_MISSION_REQUEST_PARTIAL_LIST_37(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_MISSION_WRITE_PARTIAL_LIST_38();
Pack * c_LoopBackDemoChannel_ADV_new_MISSION_WRITE_PARTIAL_LIST_38();
extern void c_LoopBackDemoChannel_on_MISSION_WRITE_PARTIAL_LIST_38(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_MISSION_ITEM_39();
Pack * c_LoopBackDemoChannel_ADV_new_MISSION_ITEM_39();
extern void c_LoopBackDemoChannel_on_MISSION_ITEM_39(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_MISSION_REQUEST_40();
Pack * c_LoopBackDemoChannel_ADV_new_MISSION_REQUEST_40();
extern void c_LoopBackDemoChannel_on_MISSION_REQUEST_40(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_MISSION_SET_CURRENT_41();
Pack * c_LoopBackDemoChannel_ADV_new_MISSION_SET_CURRENT_41();
extern void c_LoopBackDemoChannel_on_MISSION_SET_CURRENT_41(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_MISSION_CURRENT_42();
Pack * c_LoopBackDemoChannel_ADV_new_MISSION_CURRENT_42();
extern void c_LoopBackDemoChannel_on_MISSION_CURRENT_42(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_MISSION_REQUEST_LIST_43();
Pack * c_LoopBackDemoChannel_ADV_new_MISSION_REQUEST_LIST_43();
extern void c_LoopBackDemoChannel_on_MISSION_REQUEST_LIST_43(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_MISSION_COUNT_44();
Pack * c_LoopBackDemoChannel_ADV_new_MISSION_COUNT_44();
extern void c_LoopBackDemoChannel_on_MISSION_COUNT_44(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_MISSION_CLEAR_ALL_45();
Pack * c_LoopBackDemoChannel_ADV_new_MISSION_CLEAR_ALL_45();
extern void c_LoopBackDemoChannel_on_MISSION_CLEAR_ALL_45(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_MISSION_ITEM_REACHED_46();
Pack * c_LoopBackDemoChannel_ADV_new_MISSION_ITEM_REACHED_46();
extern void c_LoopBackDemoChannel_on_MISSION_ITEM_REACHED_46(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_MISSION_ACK_47();
Pack * c_LoopBackDemoChannel_ADV_new_MISSION_ACK_47();
extern void c_LoopBackDemoChannel_on_MISSION_ACK_47(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_SET_GPS_GLOBAL_ORIGIN_48();
Pack * c_LoopBackDemoChannel_ADV_new_SET_GPS_GLOBAL_ORIGIN_48();
extern void c_LoopBackDemoChannel_on_SET_GPS_GLOBAL_ORIGIN_48(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_GPS_GLOBAL_ORIGIN_49();
Pack * c_LoopBackDemoChannel_ADV_new_GPS_GLOBAL_ORIGIN_49();
extern void c_LoopBackDemoChannel_on_GPS_GLOBAL_ORIGIN_49(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_PARAM_MAP_RC_50();
Pack * c_LoopBackDemoChannel_ADV_new_PARAM_MAP_RC_50();
extern void c_LoopBackDemoChannel_on_PARAM_MAP_RC_50(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_MISSION_REQUEST_INT_51();
Pack * c_LoopBackDemoChannel_ADV_new_MISSION_REQUEST_INT_51();
extern void c_LoopBackDemoChannel_on_MISSION_REQUEST_INT_51(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_SAFETY_SET_ALLOWED_AREA_54();
Pack * c_LoopBackDemoChannel_ADV_new_SAFETY_SET_ALLOWED_AREA_54();
extern void c_LoopBackDemoChannel_on_SAFETY_SET_ALLOWED_AREA_54(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_SAFETY_ALLOWED_AREA_55();
Pack * c_LoopBackDemoChannel_ADV_new_SAFETY_ALLOWED_AREA_55();
extern void c_LoopBackDemoChannel_on_SAFETY_ALLOWED_AREA_55(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_ATTITUDE_QUATERNION_COV_61();
Pack * c_LoopBackDemoChannel_ADV_new_ATTITUDE_QUATERNION_COV_61();
extern void c_LoopBackDemoChannel_on_ATTITUDE_QUATERNION_COV_61(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_NAV_CONTROLLER_OUTPUT_62();
Pack * c_LoopBackDemoChannel_ADV_new_NAV_CONTROLLER_OUTPUT_62();
extern void c_LoopBackDemoChannel_on_NAV_CONTROLLER_OUTPUT_62(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_GLOBAL_POSITION_INT_COV_63();
Pack * c_LoopBackDemoChannel_ADV_new_GLOBAL_POSITION_INT_COV_63();
extern void c_LoopBackDemoChannel_on_GLOBAL_POSITION_INT_COV_63(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_LOCAL_POSITION_NED_COV_64();
Pack * c_LoopBackDemoChannel_ADV_new_LOCAL_POSITION_NED_COV_64();
extern void c_LoopBackDemoChannel_on_LOCAL_POSITION_NED_COV_64(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_RC_CHANNELS_65();
Pack * c_LoopBackDemoChannel_ADV_new_RC_CHANNELS_65();
extern void c_LoopBackDemoChannel_on_RC_CHANNELS_65(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_REQUEST_DATA_STREAM_66();
Pack * c_LoopBackDemoChannel_ADV_new_REQUEST_DATA_STREAM_66();
extern void c_LoopBackDemoChannel_on_REQUEST_DATA_STREAM_66(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_DATA_STREAM_67();
Pack * c_LoopBackDemoChannel_ADV_new_DATA_STREAM_67();
extern void c_LoopBackDemoChannel_on_DATA_STREAM_67(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_MANUAL_CONTROL_69();
Pack * c_LoopBackDemoChannel_ADV_new_MANUAL_CONTROL_69();
extern void c_LoopBackDemoChannel_on_MANUAL_CONTROL_69(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_RC_CHANNELS_OVERRIDE_70();
Pack * c_LoopBackDemoChannel_ADV_new_RC_CHANNELS_OVERRIDE_70();
extern void c_LoopBackDemoChannel_on_RC_CHANNELS_OVERRIDE_70(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_MISSION_ITEM_INT_73();
Pack * c_LoopBackDemoChannel_ADV_new_MISSION_ITEM_INT_73();
extern void c_LoopBackDemoChannel_on_MISSION_ITEM_INT_73(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_VFR_HUD_74();
Pack * c_LoopBackDemoChannel_ADV_new_VFR_HUD_74();
extern void c_LoopBackDemoChannel_on_VFR_HUD_74(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_COMMAND_INT_75();
Pack * c_LoopBackDemoChannel_ADV_new_COMMAND_INT_75();
extern void c_LoopBackDemoChannel_on_COMMAND_INT_75(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_COMMAND_LONG_76();
Pack * c_LoopBackDemoChannel_ADV_new_COMMAND_LONG_76();
extern void c_LoopBackDemoChannel_on_COMMAND_LONG_76(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_COMMAND_ACK_77();
Pack * c_LoopBackDemoChannel_ADV_new_COMMAND_ACK_77();
extern void c_LoopBackDemoChannel_on_COMMAND_ACK_77(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_MANUAL_SETPOINT_81();
Pack * c_LoopBackDemoChannel_ADV_new_MANUAL_SETPOINT_81();
extern void c_LoopBackDemoChannel_on_MANUAL_SETPOINT_81(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_SET_ATTITUDE_TARGET_82();
Pack * c_LoopBackDemoChannel_ADV_new_SET_ATTITUDE_TARGET_82();
extern void c_LoopBackDemoChannel_on_SET_ATTITUDE_TARGET_82(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_ATTITUDE_TARGET_83();
Pack * c_LoopBackDemoChannel_ADV_new_ATTITUDE_TARGET_83();
extern void c_LoopBackDemoChannel_on_ATTITUDE_TARGET_83(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_SET_POSITION_TARGET_LOCAL_NED_84();
Pack * c_LoopBackDemoChannel_ADV_new_SET_POSITION_TARGET_LOCAL_NED_84();
extern void c_LoopBackDemoChannel_on_SET_POSITION_TARGET_LOCAL_NED_84(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_SET_POSITION_TARGET_GLOBAL_INT_86();
Pack * c_LoopBackDemoChannel_ADV_new_SET_POSITION_TARGET_GLOBAL_INT_86();
extern void c_LoopBackDemoChannel_on_SET_POSITION_TARGET_GLOBAL_INT_86(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_POSITION_TARGET_GLOBAL_INT_87();
Pack * c_LoopBackDemoChannel_ADV_new_POSITION_TARGET_GLOBAL_INT_87();
extern void c_LoopBackDemoChannel_on_POSITION_TARGET_GLOBAL_INT_87(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_89();
Pack * c_LoopBackDemoChannel_ADV_new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_89();
extern void c_LoopBackDemoChannel_on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_89(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_HIL_STATE_90();
Pack * c_LoopBackDemoChannel_ADV_new_HIL_STATE_90();
extern void c_LoopBackDemoChannel_on_HIL_STATE_90(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_HIL_CONTROLS_91();
Pack * c_LoopBackDemoChannel_ADV_new_HIL_CONTROLS_91();
extern void c_LoopBackDemoChannel_on_HIL_CONTROLS_91(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_HIL_RC_INPUTS_RAW_92();
Pack * c_LoopBackDemoChannel_ADV_new_HIL_RC_INPUTS_RAW_92();
extern void c_LoopBackDemoChannel_on_HIL_RC_INPUTS_RAW_92(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_HIL_ACTUATOR_CONTROLS_93();
Pack * c_LoopBackDemoChannel_ADV_new_HIL_ACTUATOR_CONTROLS_93();
extern void c_LoopBackDemoChannel_on_HIL_ACTUATOR_CONTROLS_93(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_OPTICAL_FLOW_100();
Pack * c_LoopBackDemoChannel_ADV_new_OPTICAL_FLOW_100();
extern void c_LoopBackDemoChannel_on_OPTICAL_FLOW_100(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_GLOBAL_VISION_POSITION_ESTIMATE_101();
Pack * c_LoopBackDemoChannel_ADV_new_GLOBAL_VISION_POSITION_ESTIMATE_101();
extern void c_LoopBackDemoChannel_on_GLOBAL_VISION_POSITION_ESTIMATE_101(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_VISION_POSITION_ESTIMATE_102();
Pack * c_LoopBackDemoChannel_ADV_new_VISION_POSITION_ESTIMATE_102();
extern void c_LoopBackDemoChannel_on_VISION_POSITION_ESTIMATE_102(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_VISION_SPEED_ESTIMATE_103();
Pack * c_LoopBackDemoChannel_ADV_new_VISION_SPEED_ESTIMATE_103();
extern void c_LoopBackDemoChannel_on_VISION_SPEED_ESTIMATE_103(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_VICON_POSITION_ESTIMATE_104();
Pack * c_LoopBackDemoChannel_ADV_new_VICON_POSITION_ESTIMATE_104();
extern void c_LoopBackDemoChannel_on_VICON_POSITION_ESTIMATE_104(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_HIGHRES_IMU_105();
Pack * c_LoopBackDemoChannel_ADV_new_HIGHRES_IMU_105();
extern void c_LoopBackDemoChannel_on_HIGHRES_IMU_105(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_OPTICAL_FLOW_RAD_106();
Pack * c_LoopBackDemoChannel_ADV_new_OPTICAL_FLOW_RAD_106();
extern void c_LoopBackDemoChannel_on_OPTICAL_FLOW_RAD_106(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_HIL_SENSOR_107();
Pack * c_LoopBackDemoChannel_ADV_new_HIL_SENSOR_107();
extern void c_LoopBackDemoChannel_on_HIL_SENSOR_107(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_SIM_STATE_108();
Pack * c_LoopBackDemoChannel_ADV_new_SIM_STATE_108();
extern void c_LoopBackDemoChannel_on_SIM_STATE_108(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_RADIO_STATUS_109();
Pack * c_LoopBackDemoChannel_ADV_new_RADIO_STATUS_109();
extern void c_LoopBackDemoChannel_on_RADIO_STATUS_109(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_FILE_TRANSFER_PROTOCOL_110();
Pack * c_LoopBackDemoChannel_ADV_new_FILE_TRANSFER_PROTOCOL_110();
extern void c_LoopBackDemoChannel_on_FILE_TRANSFER_PROTOCOL_110(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_TIMESYNC_111();
Pack * c_LoopBackDemoChannel_ADV_new_TIMESYNC_111();
extern void c_LoopBackDemoChannel_on_TIMESYNC_111(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_CAMERA_TRIGGER_112();
Pack * c_LoopBackDemoChannel_ADV_new_CAMERA_TRIGGER_112();
extern void c_LoopBackDemoChannel_on_CAMERA_TRIGGER_112(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_HIL_GPS_113();
Pack * c_LoopBackDemoChannel_ADV_new_HIL_GPS_113();
extern void c_LoopBackDemoChannel_on_HIL_GPS_113(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_HIL_OPTICAL_FLOW_114();
Pack * c_LoopBackDemoChannel_ADV_new_HIL_OPTICAL_FLOW_114();
extern void c_LoopBackDemoChannel_on_HIL_OPTICAL_FLOW_114(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_HIL_STATE_QUATERNION_115();
Pack * c_LoopBackDemoChannel_ADV_new_HIL_STATE_QUATERNION_115();
extern void c_LoopBackDemoChannel_on_HIL_STATE_QUATERNION_115(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_SCALED_IMU2_116();
Pack * c_LoopBackDemoChannel_ADV_new_SCALED_IMU2_116();
extern void c_LoopBackDemoChannel_on_SCALED_IMU2_116(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_LOG_REQUEST_LIST_117();
Pack * c_LoopBackDemoChannel_ADV_new_LOG_REQUEST_LIST_117();
extern void c_LoopBackDemoChannel_on_LOG_REQUEST_LIST_117(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_LOG_ENTRY_118();
Pack * c_LoopBackDemoChannel_ADV_new_LOG_ENTRY_118();
extern void c_LoopBackDemoChannel_on_LOG_ENTRY_118(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_LOG_REQUEST_DATA_119();
Pack * c_LoopBackDemoChannel_ADV_new_LOG_REQUEST_DATA_119();
extern void c_LoopBackDemoChannel_on_LOG_REQUEST_DATA_119(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_LOG_DATA_120();
Pack * c_LoopBackDemoChannel_ADV_new_LOG_DATA_120();
extern void c_LoopBackDemoChannel_on_LOG_DATA_120(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_LOG_ERASE_121();
Pack * c_LoopBackDemoChannel_ADV_new_LOG_ERASE_121();
extern void c_LoopBackDemoChannel_on_LOG_ERASE_121(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_LOG_REQUEST_END_122();
Pack * c_LoopBackDemoChannel_ADV_new_LOG_REQUEST_END_122();
extern void c_LoopBackDemoChannel_on_LOG_REQUEST_END_122(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_GPS_INJECT_DATA_123();
Pack * c_LoopBackDemoChannel_ADV_new_GPS_INJECT_DATA_123();
extern void c_LoopBackDemoChannel_on_GPS_INJECT_DATA_123(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_GPS2_RAW_124();
Pack * c_LoopBackDemoChannel_ADV_new_GPS2_RAW_124();
extern void c_LoopBackDemoChannel_on_GPS2_RAW_124(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_POWER_STATUS_125();
Pack * c_LoopBackDemoChannel_ADV_new_POWER_STATUS_125();
extern void c_LoopBackDemoChannel_on_POWER_STATUS_125(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_SERIAL_CONTROL_126();
Pack * c_LoopBackDemoChannel_ADV_new_SERIAL_CONTROL_126();
extern void c_LoopBackDemoChannel_on_SERIAL_CONTROL_126(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_GPS_RTK_127();
Pack * c_LoopBackDemoChannel_ADV_new_GPS_RTK_127();
extern void c_LoopBackDemoChannel_on_GPS_RTK_127(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_GPS2_RTK_128();
Pack * c_LoopBackDemoChannel_ADV_new_GPS2_RTK_128();
extern void c_LoopBackDemoChannel_on_GPS2_RTK_128(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_SCALED_IMU3_129();
Pack * c_LoopBackDemoChannel_ADV_new_SCALED_IMU3_129();
extern void c_LoopBackDemoChannel_on_SCALED_IMU3_129(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_DATA_TRANSMISSION_HANDSHAKE_130();
Pack * c_LoopBackDemoChannel_ADV_new_DATA_TRANSMISSION_HANDSHAKE_130();
extern void c_LoopBackDemoChannel_on_DATA_TRANSMISSION_HANDSHAKE_130(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_ENCAPSULATED_DATA_131();
Pack * c_LoopBackDemoChannel_ADV_new_ENCAPSULATED_DATA_131();
extern void c_LoopBackDemoChannel_on_ENCAPSULATED_DATA_131(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_DISTANCE_SENSOR_132();
Pack * c_LoopBackDemoChannel_ADV_new_DISTANCE_SENSOR_132();
extern void c_LoopBackDemoChannel_on_DISTANCE_SENSOR_132(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_TERRAIN_REQUEST_133();
Pack * c_LoopBackDemoChannel_ADV_new_TERRAIN_REQUEST_133();
extern void c_LoopBackDemoChannel_on_TERRAIN_REQUEST_133(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_TERRAIN_DATA_134();
Pack * c_LoopBackDemoChannel_ADV_new_TERRAIN_DATA_134();
extern void c_LoopBackDemoChannel_on_TERRAIN_DATA_134(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_TERRAIN_CHECK_135();
Pack * c_LoopBackDemoChannel_ADV_new_TERRAIN_CHECK_135();
extern void c_LoopBackDemoChannel_on_TERRAIN_CHECK_135(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_TERRAIN_REPORT_136();
Pack * c_LoopBackDemoChannel_ADV_new_TERRAIN_REPORT_136();
extern void c_LoopBackDemoChannel_on_TERRAIN_REPORT_136(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_SCALED_PRESSURE2_137();
Pack * c_LoopBackDemoChannel_ADV_new_SCALED_PRESSURE2_137();
extern void c_LoopBackDemoChannel_on_SCALED_PRESSURE2_137(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_ATT_POS_MOCAP_138();
Pack * c_LoopBackDemoChannel_ADV_new_ATT_POS_MOCAP_138();
extern void c_LoopBackDemoChannel_on_ATT_POS_MOCAP_138(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_SET_ACTUATOR_CONTROL_TARGET_139();
Pack * c_LoopBackDemoChannel_ADV_new_SET_ACTUATOR_CONTROL_TARGET_139();
extern void c_LoopBackDemoChannel_on_SET_ACTUATOR_CONTROL_TARGET_139(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_ACTUATOR_CONTROL_TARGET_140();
Pack * c_LoopBackDemoChannel_ADV_new_ACTUATOR_CONTROL_TARGET_140();
extern void c_LoopBackDemoChannel_on_ACTUATOR_CONTROL_TARGET_140(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_ALTITUDE_141();
Pack * c_LoopBackDemoChannel_ADV_new_ALTITUDE_141();
extern void c_LoopBackDemoChannel_on_ALTITUDE_141(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_RESOURCE_REQUEST_142();
Pack * c_LoopBackDemoChannel_ADV_new_RESOURCE_REQUEST_142();
extern void c_LoopBackDemoChannel_on_RESOURCE_REQUEST_142(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_SCALED_PRESSURE3_143();
Pack * c_LoopBackDemoChannel_ADV_new_SCALED_PRESSURE3_143();
extern void c_LoopBackDemoChannel_on_SCALED_PRESSURE3_143(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_FOLLOW_TARGET_144();
Pack * c_LoopBackDemoChannel_ADV_new_FOLLOW_TARGET_144();
extern void c_LoopBackDemoChannel_on_FOLLOW_TARGET_144(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_CONTROL_SYSTEM_STATE_146();
Pack * c_LoopBackDemoChannel_ADV_new_CONTROL_SYSTEM_STATE_146();
extern void c_LoopBackDemoChannel_on_CONTROL_SYSTEM_STATE_146(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_BATTERY_STATUS_147();
Pack * c_LoopBackDemoChannel_ADV_new_BATTERY_STATUS_147();
extern void c_LoopBackDemoChannel_on_BATTERY_STATUS_147(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_AUTOPILOT_VERSION_148();
Pack * c_LoopBackDemoChannel_ADV_new_AUTOPILOT_VERSION_148();
extern void c_LoopBackDemoChannel_on_AUTOPILOT_VERSION_148(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_LANDING_TARGET_149();
Pack * c_LoopBackDemoChannel_ADV_new_LANDING_TARGET_149();
extern void c_LoopBackDemoChannel_on_LANDING_TARGET_149(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_SENS_POWER_201();
Pack * c_LoopBackDemoChannel_ADV_new_SENS_POWER_201();
extern void c_LoopBackDemoChannel_on_SENS_POWER_201(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_SENS_MPPT_202();
Pack * c_LoopBackDemoChannel_ADV_new_SENS_MPPT_202();
extern void c_LoopBackDemoChannel_on_SENS_MPPT_202(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_ASLCTRL_DATA_203();
Pack * c_LoopBackDemoChannel_ADV_new_ASLCTRL_DATA_203();
extern void c_LoopBackDemoChannel_on_ASLCTRL_DATA_203(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_ASLCTRL_DEBUG_204();
Pack * c_LoopBackDemoChannel_ADV_new_ASLCTRL_DEBUG_204();
extern void c_LoopBackDemoChannel_on_ASLCTRL_DEBUG_204(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_ASLUAV_STATUS_205();
Pack * c_LoopBackDemoChannel_ADV_new_ASLUAV_STATUS_205();
extern void c_LoopBackDemoChannel_on_ASLUAV_STATUS_205(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_EKF_EXT_206();
Pack * c_LoopBackDemoChannel_ADV_new_EKF_EXT_206();
extern void c_LoopBackDemoChannel_on_EKF_EXT_206(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_ASL_OBCTRL_207();
Pack * c_LoopBackDemoChannel_ADV_new_ASL_OBCTRL_207();
extern void c_LoopBackDemoChannel_on_ASL_OBCTRL_207(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_SENS_ATMOS_208();
Pack * c_LoopBackDemoChannel_ADV_new_SENS_ATMOS_208();
extern void c_LoopBackDemoChannel_on_SENS_ATMOS_208(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_SENS_BATMON_209();
Pack * c_LoopBackDemoChannel_ADV_new_SENS_BATMON_209();
extern void c_LoopBackDemoChannel_on_SENS_BATMON_209(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_FW_SOARING_DATA_210();
Pack * c_LoopBackDemoChannel_ADV_new_FW_SOARING_DATA_210();
extern void c_LoopBackDemoChannel_on_FW_SOARING_DATA_210(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_SENSORPOD_STATUS_211();
Pack * c_LoopBackDemoChannel_ADV_new_SENSORPOD_STATUS_211();
extern void c_LoopBackDemoChannel_on_SENSORPOD_STATUS_211(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_SENS_POWER_BOARD_212();
Pack * c_LoopBackDemoChannel_ADV_new_SENS_POWER_BOARD_212();
extern void c_LoopBackDemoChannel_on_SENS_POWER_BOARD_212(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_ESTIMATOR_STATUS_230();
Pack * c_LoopBackDemoChannel_ADV_new_ESTIMATOR_STATUS_230();
extern void c_LoopBackDemoChannel_on_ESTIMATOR_STATUS_230(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_WIND_COV_231();
Pack * c_LoopBackDemoChannel_ADV_new_WIND_COV_231();
extern void c_LoopBackDemoChannel_on_WIND_COV_231(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_GPS_INPUT_232();
Pack * c_LoopBackDemoChannel_ADV_new_GPS_INPUT_232();
extern void c_LoopBackDemoChannel_on_GPS_INPUT_232(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_GPS_RTCM_DATA_233();
Pack * c_LoopBackDemoChannel_ADV_new_GPS_RTCM_DATA_233();
extern void c_LoopBackDemoChannel_on_GPS_RTCM_DATA_233(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_HIGH_LATENCY_234();
Pack * c_LoopBackDemoChannel_ADV_new_HIGH_LATENCY_234();
extern void c_LoopBackDemoChannel_on_HIGH_LATENCY_234(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_VIBRATION_241();
Pack * c_LoopBackDemoChannel_ADV_new_VIBRATION_241();
extern void c_LoopBackDemoChannel_on_VIBRATION_241(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_HOME_POSITION_242();
Pack * c_LoopBackDemoChannel_ADV_new_HOME_POSITION_242();
extern void c_LoopBackDemoChannel_on_HOME_POSITION_242(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_SET_HOME_POSITION_243();
Pack * c_LoopBackDemoChannel_ADV_new_SET_HOME_POSITION_243();
extern void c_LoopBackDemoChannel_on_SET_HOME_POSITION_243(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_MESSAGE_INTERVAL_244();
Pack * c_LoopBackDemoChannel_ADV_new_MESSAGE_INTERVAL_244();
extern void c_LoopBackDemoChannel_on_MESSAGE_INTERVAL_244(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_EXTENDED_SYS_STATE_245();
Pack * c_LoopBackDemoChannel_ADV_new_EXTENDED_SYS_STATE_245();
extern void c_LoopBackDemoChannel_on_EXTENDED_SYS_STATE_245(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_ADSB_VEHICLE_246();
Pack * c_LoopBackDemoChannel_ADV_new_ADSB_VEHICLE_246();
extern void c_LoopBackDemoChannel_on_ADSB_VEHICLE_246(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_COLLISION_247();
Pack * c_LoopBackDemoChannel_ADV_new_COLLISION_247();
extern void c_LoopBackDemoChannel_on_COLLISION_247(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_V2_EXTENSION_248();
Pack * c_LoopBackDemoChannel_ADV_new_V2_EXTENSION_248();
extern void c_LoopBackDemoChannel_on_V2_EXTENSION_248(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_MEMORY_VECT_249();
Pack * c_LoopBackDemoChannel_ADV_new_MEMORY_VECT_249();
extern void c_LoopBackDemoChannel_on_MEMORY_VECT_249(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_DEBUG_VECT_250();
Pack * c_LoopBackDemoChannel_ADV_new_DEBUG_VECT_250();
extern void c_LoopBackDemoChannel_on_DEBUG_VECT_250(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_NAMED_VALUE_FLOAT_251();
Pack * c_LoopBackDemoChannel_ADV_new_NAMED_VALUE_FLOAT_251();
extern void c_LoopBackDemoChannel_on_NAMED_VALUE_FLOAT_251(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_NAMED_VALUE_INT_252();
Pack * c_LoopBackDemoChannel_ADV_new_NAMED_VALUE_INT_252();
extern void c_LoopBackDemoChannel_on_NAMED_VALUE_INT_252(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_STATUSTEXT_253();
Pack * c_LoopBackDemoChannel_ADV_new_STATUSTEXT_253();
extern void c_LoopBackDemoChannel_on_STATUSTEXT_253(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_DEBUG_254();
Pack * c_LoopBackDemoChannel_ADV_new_DEBUG_254();
extern void c_LoopBackDemoChannel_on_DEBUG_254(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_SETUP_SIGNING_256();
Pack * c_LoopBackDemoChannel_ADV_new_SETUP_SIGNING_256();
extern void c_LoopBackDemoChannel_on_SETUP_SIGNING_256(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_BUTTON_CHANGE_257();
Pack * c_LoopBackDemoChannel_ADV_new_BUTTON_CHANGE_257();
extern void c_LoopBackDemoChannel_on_BUTTON_CHANGE_257(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_PLAY_TUNE_258();
Pack * c_LoopBackDemoChannel_ADV_new_PLAY_TUNE_258();
extern void c_LoopBackDemoChannel_on_PLAY_TUNE_258(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_CAMERA_INFORMATION_259();
Pack * c_LoopBackDemoChannel_ADV_new_CAMERA_INFORMATION_259();
extern void c_LoopBackDemoChannel_on_CAMERA_INFORMATION_259(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_CAMERA_SETTINGS_260();
Pack * c_LoopBackDemoChannel_ADV_new_CAMERA_SETTINGS_260();
extern void c_LoopBackDemoChannel_on_CAMERA_SETTINGS_260(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_STORAGE_INFORMATION_261();
Pack * c_LoopBackDemoChannel_ADV_new_STORAGE_INFORMATION_261();
extern void c_LoopBackDemoChannel_on_STORAGE_INFORMATION_261(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_CAMERA_CAPTURE_STATUS_262();
Pack * c_LoopBackDemoChannel_ADV_new_CAMERA_CAPTURE_STATUS_262();
extern void c_LoopBackDemoChannel_on_CAMERA_CAPTURE_STATUS_262(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_CAMERA_IMAGE_CAPTURED_263();
Pack * c_LoopBackDemoChannel_ADV_new_CAMERA_IMAGE_CAPTURED_263();
extern void c_LoopBackDemoChannel_on_CAMERA_IMAGE_CAPTURED_263(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_FLIGHT_INFORMATION_264();
Pack * c_LoopBackDemoChannel_ADV_new_FLIGHT_INFORMATION_264();
extern void c_LoopBackDemoChannel_on_FLIGHT_INFORMATION_264(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_MOUNT_ORIENTATION_265();
Pack * c_LoopBackDemoChannel_ADV_new_MOUNT_ORIENTATION_265();
extern void c_LoopBackDemoChannel_on_MOUNT_ORIENTATION_265(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_LOGGING_DATA_266();
Pack * c_LoopBackDemoChannel_ADV_new_LOGGING_DATA_266();
extern void c_LoopBackDemoChannel_on_LOGGING_DATA_266(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_LOGGING_DATA_ACKED_267();
Pack * c_LoopBackDemoChannel_ADV_new_LOGGING_DATA_ACKED_267();
extern void c_LoopBackDemoChannel_on_LOGGING_DATA_ACKED_267(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_LOGGING_ACK_268();
Pack * c_LoopBackDemoChannel_ADV_new_LOGGING_ACK_268();
extern void c_LoopBackDemoChannel_on_LOGGING_ACK_268(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_VIDEO_STREAM_INFORMATION_269();
Pack * c_LoopBackDemoChannel_ADV_new_VIDEO_STREAM_INFORMATION_269();
extern void c_LoopBackDemoChannel_on_VIDEO_STREAM_INFORMATION_269(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_SET_VIDEO_STREAM_SETTINGS_270();
Pack * c_LoopBackDemoChannel_ADV_new_SET_VIDEO_STREAM_SETTINGS_270();
extern void c_LoopBackDemoChannel_on_SET_VIDEO_STREAM_SETTINGS_270(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_WIFI_CONFIG_AP_299();
Pack * c_LoopBackDemoChannel_ADV_new_WIFI_CONFIG_AP_299();
extern void c_LoopBackDemoChannel_on_WIFI_CONFIG_AP_299(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_PROTOCOL_VERSION_300();
Pack * c_LoopBackDemoChannel_ADV_new_PROTOCOL_VERSION_300();
extern void c_LoopBackDemoChannel_on_PROTOCOL_VERSION_300(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_UAVCAN_NODE_STATUS_310();
Pack * c_LoopBackDemoChannel_ADV_new_UAVCAN_NODE_STATUS_310();
extern void c_LoopBackDemoChannel_on_UAVCAN_NODE_STATUS_310(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_UAVCAN_NODE_INFO_311();
Pack * c_LoopBackDemoChannel_ADV_new_UAVCAN_NODE_INFO_311();
extern void c_LoopBackDemoChannel_on_UAVCAN_NODE_INFO_311(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_PARAM_EXT_REQUEST_READ_320();
Pack * c_LoopBackDemoChannel_ADV_new_PARAM_EXT_REQUEST_READ_320();
extern void c_LoopBackDemoChannel_on_PARAM_EXT_REQUEST_READ_320(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_PARAM_EXT_REQUEST_LIST_321();
Pack * c_LoopBackDemoChannel_ADV_new_PARAM_EXT_REQUEST_LIST_321();
extern void c_LoopBackDemoChannel_on_PARAM_EXT_REQUEST_LIST_321(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_PARAM_EXT_VALUE_322();
Pack * c_LoopBackDemoChannel_ADV_new_PARAM_EXT_VALUE_322();
extern void c_LoopBackDemoChannel_on_PARAM_EXT_VALUE_322(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_PARAM_EXT_SET_323();
Pack * c_LoopBackDemoChannel_ADV_new_PARAM_EXT_SET_323();
extern void c_LoopBackDemoChannel_on_PARAM_EXT_SET_323(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_PARAM_EXT_ACK_324();
Pack * c_LoopBackDemoChannel_ADV_new_PARAM_EXT_ACK_324();
extern void c_LoopBackDemoChannel_on_PARAM_EXT_ACK_324(Bounds_Inside * bi, Pack * pack);
Pack * c_LoopBackDemoChannel_new_OBSTACLE_DISTANCE_330();
Pack * c_LoopBackDemoChannel_ADV_new_OBSTACLE_DISTANCE_330();
extern void c_LoopBackDemoChannel_on_OBSTACLE_DISTANCE_330(Bounds_Inside * bi, Pack * pack);

extern Channel c_LoopBackDemoChannel;
INLINER void c_LoopBackDemoChannel_send(Pack * pack) {c_LoopBackDemoChannel.process(pack, PROCESS_HOST_REQEST);}
INLINER int32_t  c_LoopBackDemoChannel_input_bytes(uint8_t* dst, int32_t bytes) { return input_bytes(&c_LoopBackDemoChannel, dst, bytes);}

//------------------------
INLINER void  c_LoopBackDemoChannel_output_bytes(uint8_t* src, int32_t bytes) {  output_bytes(&c_LoopBackDemoChannel, src, bytes);} //pass received from a network bytes to the channel
INLINER void c_LoopBackDemoChannel_process_received() { c_LoopBackDemoChannel.process(NULL, PROCESS_RECEIVED_PACKS);}//dispatch received packs to its handlers

extern Channel c_LoopBackDemoChannel_ADV;
INLINER void c_LoopBackDemoChannel_ADV_send(Pack * pack) {c_LoopBackDemoChannel_ADV.process(pack, PROCESS_HOST_REQEST);}
INLINER int32_t  c_LoopBackDemoChannel_ADV_input_bytes(uint8_t* dst, int32_t bytes) { return input_bytes_adv(&c_LoopBackDemoChannel_ADV, dst, bytes);}

; /*
				                                              */
typedef  enum
{
    e_MAV_TYPE_MAV_TYPE_GENERIC = 0,
    e_MAV_TYPE_MAV_TYPE_FIXED_WING = 1,
    e_MAV_TYPE_MAV_TYPE_QUADROTOR = 2,
    e_MAV_TYPE_MAV_TYPE_COAXIAL = 3,
    e_MAV_TYPE_MAV_TYPE_HELICOPTER = 4,
    e_MAV_TYPE_MAV_TYPE_ANTENNA_TRACKER = 5,
    e_MAV_TYPE_MAV_TYPE_GCS = 6,
    e_MAV_TYPE_MAV_TYPE_AIRSHIP = 7,
    e_MAV_TYPE_MAV_TYPE_FREE_BALLOON = 8,
    e_MAV_TYPE_MAV_TYPE_ROCKET = 9,
    e_MAV_TYPE_MAV_TYPE_GROUND_ROVER = 10,
    e_MAV_TYPE_MAV_TYPE_SURFACE_BOAT = 11,
    e_MAV_TYPE_MAV_TYPE_SUBMARINE = 12,
    e_MAV_TYPE_MAV_TYPE_HEXAROTOR = 13,
    e_MAV_TYPE_MAV_TYPE_OCTOROTOR = 14,
    e_MAV_TYPE_MAV_TYPE_TRICOPTER = 15,
    e_MAV_TYPE_MAV_TYPE_FLAPPING_WING = 16,
    e_MAV_TYPE_MAV_TYPE_KITE = 17,
    e_MAV_TYPE_MAV_TYPE_ONBOARD_CONTROLLER = 18,
    e_MAV_TYPE_MAV_TYPE_VTOL_DUOROTOR = 19,
    e_MAV_TYPE_MAV_TYPE_VTOL_QUADROTOR = 20,
    e_MAV_TYPE_MAV_TYPE_VTOL_TILTROTOR = 21,
    e_MAV_TYPE_MAV_TYPE_VTOL_RESERVED2 = 22,
    e_MAV_TYPE_MAV_TYPE_VTOL_RESERVED3 = 23,
    e_MAV_TYPE_MAV_TYPE_VTOL_RESERVED4 = 24,
    e_MAV_TYPE_MAV_TYPE_VTOL_RESERVED5 = 25,
    e_MAV_TYPE_MAV_TYPE_GIMBAL = 26,
    e_MAV_TYPE_MAV_TYPE_ADSB = 27,
    e_MAV_TYPE_MAV_TYPE_PARAFOIL = 28
} e_MAV_TYPE;
; /*
				                                              */
typedef  enum
{
    e_MAV_AUTOPILOT_MAV_AUTOPILOT_GENERIC = 0,
    e_MAV_AUTOPILOT_MAV_AUTOPILOT_RESERVED = 1,
    e_MAV_AUTOPILOT_MAV_AUTOPILOT_SLUGS = 2,
    e_MAV_AUTOPILOT_MAV_AUTOPILOT_ARDUPILOTMEGA = 3,
    e_MAV_AUTOPILOT_MAV_AUTOPILOT_OPENPILOT = 4,
    e_MAV_AUTOPILOT_MAV_AUTOPILOT_GENERIC_WAYPOINTS_ONLY = 5,
    e_MAV_AUTOPILOT_MAV_AUTOPILOT_GENERIC_WAYPOINTS_AND_SIMPLE_NAVIGATION_ONLY = 6,
    e_MAV_AUTOPILOT_MAV_AUTOPILOT_GENERIC_MISSION_FULL = 7,
    e_MAV_AUTOPILOT_MAV_AUTOPILOT_INVALID = 8,
    e_MAV_AUTOPILOT_MAV_AUTOPILOT_PPZ = 9,
    e_MAV_AUTOPILOT_MAV_AUTOPILOT_UDB = 10,
    e_MAV_AUTOPILOT_MAV_AUTOPILOT_FP = 11,
    e_MAV_AUTOPILOT_MAV_AUTOPILOT_PX4 = 12,
    e_MAV_AUTOPILOT_MAV_AUTOPILOT_SMACCMPILOT = 13,
    e_MAV_AUTOPILOT_MAV_AUTOPILOT_AUTOQUAD = 14,
    e_MAV_AUTOPILOT_MAV_AUTOPILOT_ARMAZILA = 15,
    e_MAV_AUTOPILOT_MAV_AUTOPILOT_AEROB = 16,
    e_MAV_AUTOPILOT_MAV_AUTOPILOT_ASLUAV = 17,
    e_MAV_AUTOPILOT_MAV_AUTOPILOT_SMARTAP = 18
} e_MAV_AUTOPILOT;
; /*
				                                              */
typedef  enum
{
    e_MAV_MODE_FLAG_MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1,
    e_MAV_MODE_FLAG_MAV_MODE_FLAG_TEST_ENABLED = 2,
    e_MAV_MODE_FLAG_MAV_MODE_FLAG_AUTO_ENABLED = 4,
    e_MAV_MODE_FLAG_MAV_MODE_FLAG_GUIDED_ENABLED = 8,
    e_MAV_MODE_FLAG_MAV_MODE_FLAG_STABILIZE_ENABLED = 16,
    e_MAV_MODE_FLAG_MAV_MODE_FLAG_HIL_ENABLED = 32,
    e_MAV_MODE_FLAG_MAV_MODE_FLAG_MANUAL_INPUT_ENABLED = 64,
    e_MAV_MODE_FLAG_MAV_MODE_FLAG_SAFETY_ARMED = 128
} e_MAV_MODE_FLAG;
; /*
				                                              */
typedef  enum
{
    e_MAV_STATE_MAV_STATE_UNINIT = 0,
    e_MAV_STATE_MAV_STATE_BOOT = 1,
    e_MAV_STATE_MAV_STATE_CALIBRATING = 2,
    e_MAV_STATE_MAV_STATE_STANDBY = 3,
    e_MAV_STATE_MAV_STATE_ACTIVE = 4,
    e_MAV_STATE_MAV_STATE_CRITICAL = 5,
    e_MAV_STATE_MAV_STATE_EMERGENCY = 6,
    e_MAV_STATE_MAV_STATE_POWEROFF = 7,
    e_MAV_STATE_MAV_STATE_FLIGHT_TERMINATION = 8
} e_MAV_STATE;
; /*
				                                              */
typedef  enum
{
    e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_GYRO = 1,
    e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_ACCEL = 2,
    e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_MAG = 4,
    e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE = 8,
    e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE = 16,
    e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_GPS = 32,
    e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW = 64,
    e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_VISION_POSITION = 128,
    e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_LASER_POSITION = 256,
    e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH = 512,
    e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL = 1024,
    e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION = 2048,
    e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_YAW_POSITION = 4096,
    e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL = 8192,
    e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL = 16384,
    e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS = 32768,
    e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_RC_RECEIVER = 65536,
    e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_GYRO2 = 131072,
    e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_ACCEL2 = 262144,
    e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_MAG2 = 524288,
    e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_GEOFENCE = 1048576,
    e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_AHRS = 2097152,
    e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_TERRAIN = 4194304,
    e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_REVERSE_MOTOR = 8388608,
    e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_LOGGING = 16777216,
    e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_BATTERY = 33554432
} e_MAV_SYS_STATUS_SENSOR;
; /*
				                                              */
typedef  enum
{
    e_MAV_FRAME_MAV_FRAME_GLOBAL = 0,
    e_MAV_FRAME_MAV_FRAME_LOCAL_NED = 1,
    e_MAV_FRAME_MAV_FRAME_MISSION = 2,
    e_MAV_FRAME_MAV_FRAME_GLOBAL_RELATIVE_ALT = 3,
    e_MAV_FRAME_MAV_FRAME_LOCAL_ENU = 4,
    e_MAV_FRAME_MAV_FRAME_GLOBAL_INT = 5,
    e_MAV_FRAME_MAV_FRAME_GLOBAL_RELATIVE_ALT_INT = 6,
    e_MAV_FRAME_MAV_FRAME_LOCAL_OFFSET_NED = 7,
    e_MAV_FRAME_MAV_FRAME_BODY_NED = 8,
    e_MAV_FRAME_MAV_FRAME_BODY_OFFSET_NED = 9,
    e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT = 10,
    e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT_INT = 11
} e_MAV_FRAME;
; /*
				                                              */
typedef  enum
{
    e_MAV_MODE_MAV_MODE_PREFLIGHT = 0,
    e_MAV_MODE_MAV_MODE_MANUAL_DISARMED = 64,
    e_MAV_MODE_MAV_MODE_TEST_DISARMED = 66,
    e_MAV_MODE_MAV_MODE_STABILIZE_DISARMED = 80,
    e_MAV_MODE_MAV_MODE_GUIDED_DISARMED = 88,
    e_MAV_MODE_MAV_MODE_AUTO_DISARMED = 92,
    e_MAV_MODE_MAV_MODE_MANUAL_ARMED = 192,
    e_MAV_MODE_MAV_MODE_TEST_ARMED = 194,
    e_MAV_MODE_MAV_MODE_STABILIZE_ARMED = 208,
    e_MAV_MODE_MAV_MODE_GUIDED_ARMED = 216,
    e_MAV_MODE_MAV_MODE_AUTO_ARMED = 220
} e_MAV_MODE;
; /*
				                                              */
typedef  enum
{
    e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_UINT8 = 1,
    e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_INT8 = 2,
    e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_UINT16 = 3,
    e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_INT16 = 4,
    e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_UINT32 = 5,
    e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_INT32 = 6,
    e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_UINT64 = 7,
    e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_INT64 = 8,
    e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_REAL32 = 9,
    e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_REAL64 = 10
} e_MAV_PARAM_TYPE;
; /*
				                                              */
typedef  enum
{
    e_GPS_FIX_TYPE_GPS_FIX_TYPE_NO_GPS = 0,
    e_GPS_FIX_TYPE_GPS_FIX_TYPE_NO_FIX = 1,
    e_GPS_FIX_TYPE_GPS_FIX_TYPE_2D_FIX = 2,
    e_GPS_FIX_TYPE_GPS_FIX_TYPE_3D_FIX = 3,
    e_GPS_FIX_TYPE_GPS_FIX_TYPE_DGPS = 4,
    e_GPS_FIX_TYPE_GPS_FIX_TYPE_RTK_FLOAT = 5,
    e_GPS_FIX_TYPE_GPS_FIX_TYPE_RTK_FIXED = 6,
    e_GPS_FIX_TYPE_GPS_FIX_TYPE_STATIC = 7,
    e_GPS_FIX_TYPE_GPS_FIX_TYPE_PPP = 8
} e_GPS_FIX_TYPE;
; /*
				                                              */
typedef  enum
{
    e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION = 0,
    e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE = 1,
    e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY = 2,
    e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL = 255
} e_MAV_MISSION_TYPE;
; /*
				                                              */
typedef  enum
{
    e_MAV_CMD_MAV_CMD_NAV_WAYPOINT = 16,
    e_MAV_CMD_MAV_CMD_NAV_LOITER_UNLIM = 17,
    e_MAV_CMD_MAV_CMD_NAV_LOITER_TURNS = 18,
    e_MAV_CMD_MAV_CMD_NAV_LOITER_TIME = 19,
    e_MAV_CMD_MAV_CMD_NAV_RETURN_TO_LAUNCH = 20,
    e_MAV_CMD_MAV_CMD_NAV_LAND = 21,
    e_MAV_CMD_MAV_CMD_NAV_TAKEOFF = 22,
    e_MAV_CMD_MAV_CMD_NAV_LAND_LOCAL = 23,
    e_MAV_CMD_MAV_CMD_NAV_TAKEOFF_LOCAL = 24,
    e_MAV_CMD_MAV_CMD_NAV_FOLLOW = 25,
    e_MAV_CMD_MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT = 30,
    e_MAV_CMD_MAV_CMD_NAV_LOITER_TO_ALT = 31,
    e_MAV_CMD_MAV_CMD_DO_FOLLOW = 32,
    e_MAV_CMD_MAV_CMD_DO_FOLLOW_REPOSITION = 33,
    e_MAV_CMD_MAV_CMD_NAV_ROI = 80,
    e_MAV_CMD_MAV_CMD_NAV_PATHPLANNING = 81,
    e_MAV_CMD_MAV_CMD_NAV_SPLINE_WAYPOINT = 82,
    e_MAV_CMD_MAV_CMD_NAV_VTOL_TAKEOFF = 84,
    e_MAV_CMD_MAV_CMD_NAV_VTOL_LAND = 85,
    e_MAV_CMD_MAV_CMD_NAV_GUIDED_ENABLE = 92,
    e_MAV_CMD_MAV_CMD_NAV_DELAY = 93,
    e_MAV_CMD_MAV_CMD_NAV_PAYLOAD_PLACE = 94,
    e_MAV_CMD_MAV_CMD_NAV_LAST = 95,
    e_MAV_CMD_MAV_CMD_CONDITION_DELAY = 112,
    e_MAV_CMD_MAV_CMD_CONDITION_CHANGE_ALT = 113,
    e_MAV_CMD_MAV_CMD_CONDITION_DISTANCE = 114,
    e_MAV_CMD_MAV_CMD_CONDITION_YAW = 115,
    e_MAV_CMD_MAV_CMD_CONDITION_LAST = 159,
    e_MAV_CMD_MAV_CMD_DO_SET_MODE = 176,
    e_MAV_CMD_MAV_CMD_DO_JUMP = 177,
    e_MAV_CMD_MAV_CMD_DO_CHANGE_SPEED = 178,
    e_MAV_CMD_MAV_CMD_DO_SET_HOME = 179,
    e_MAV_CMD_MAV_CMD_DO_SET_PARAMETER = 180,
    e_MAV_CMD_MAV_CMD_DO_SET_RELAY = 181,
    e_MAV_CMD_MAV_CMD_DO_REPEAT_RELAY = 182,
    e_MAV_CMD_MAV_CMD_DO_SET_SERVO = 183,
    e_MAV_CMD_MAV_CMD_DO_REPEAT_SERVO = 184,
    e_MAV_CMD_MAV_CMD_DO_FLIGHTTERMINATION = 185,
    e_MAV_CMD_MAV_CMD_DO_CHANGE_ALTITUDE = 186,
    e_MAV_CMD_MAV_CMD_DO_LAND_START = 189,
    e_MAV_CMD_MAV_CMD_DO_RALLY_LAND = 190,
    e_MAV_CMD_MAV_CMD_DO_GO_AROUND = 191,
    e_MAV_CMD_MAV_CMD_DO_REPOSITION = 192,
    e_MAV_CMD_MAV_CMD_DO_PAUSE_CONTINUE = 193,
    e_MAV_CMD_MAV_CMD_DO_SET_REVERSE = 194,
    e_MAV_CMD_MAV_CMD_DO_CONTROL_VIDEO = 200,
    e_MAV_CMD_MAV_CMD_DO_SET_ROI = 201,
    e_MAV_CMD_MAV_CMD_DO_DIGICAM_CONFIGURE = 202,
    e_MAV_CMD_MAV_CMD_DO_DIGICAM_CONTROL = 203,
    e_MAV_CMD_MAV_CMD_DO_MOUNT_CONFIGURE = 204,
    e_MAV_CMD_MAV_CMD_DO_MOUNT_CONTROL = 205,
    e_MAV_CMD_MAV_CMD_DO_SET_CAM_TRIGG_DIST = 206,
    e_MAV_CMD_MAV_CMD_DO_FENCE_ENABLE = 207,
    e_MAV_CMD_MAV_CMD_DO_PARACHUTE = 208,
    e_MAV_CMD_MAV_CMD_DO_MOTOR_TEST = 209,
    e_MAV_CMD_MAV_CMD_DO_INVERTED_FLIGHT = 210,
    e_MAV_CMD_MAV_CMD_NAV_SET_YAW_SPEED = 213,
    e_MAV_CMD_MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL = 214,
    e_MAV_CMD_MAV_CMD_DO_MOUNT_CONTROL_QUAT = 220,
    e_MAV_CMD_MAV_CMD_DO_GUIDED_MASTER = 221,
    e_MAV_CMD_MAV_CMD_DO_GUIDED_LIMITS = 222,
    e_MAV_CMD_MAV_CMD_DO_ENGINE_CONTROL = 223,
    e_MAV_CMD_MAV_CMD_DO_LAST = 240,
    e_MAV_CMD_MAV_CMD_PREFLIGHT_CALIBRATION = 241,
    e_MAV_CMD_MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS = 242,
    e_MAV_CMD_MAV_CMD_PREFLIGHT_UAVCAN = 243,
    e_MAV_CMD_MAV_CMD_PREFLIGHT_STORAGE = 245,
    e_MAV_CMD_MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN = 246,
    e_MAV_CMD_MAV_CMD_OVERRIDE_GOTO = 252,
    e_MAV_CMD_MAV_CMD_MISSION_START = 300,
    e_MAV_CMD_MAV_CMD_COMPONENT_ARM_DISARM = 400,
    e_MAV_CMD_MAV_CMD_GET_HOME_POSITION = 410,
    e_MAV_CMD_MAV_CMD_START_RX_PAIR = 500,
    e_MAV_CMD_MAV_CMD_GET_MESSAGE_INTERVAL = 510,
    e_MAV_CMD_MAV_CMD_SET_MESSAGE_INTERVAL = 511,
    e_MAV_CMD_MAV_CMD_REQUEST_PROTOCOL_VERSION = 519,
    e_MAV_CMD_MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES = 520,
    e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_INFORMATION = 521,
    e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_SETTINGS = 522,
    e_MAV_CMD_MAV_CMD_REQUEST_STORAGE_INFORMATION = 525,
    e_MAV_CMD_MAV_CMD_STORAGE_FORMAT = 526,
    e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS = 527,
    e_MAV_CMD_MAV_CMD_REQUEST_FLIGHT_INFORMATION = 528,
    e_MAV_CMD_MAV_CMD_RESET_CAMERA_SETTINGS = 529,
    e_MAV_CMD_MAV_CMD_SET_CAMERA_MODE = 530,
    e_MAV_CMD_MAV_CMD_IMAGE_START_CAPTURE = 2000,
    e_MAV_CMD_MAV_CMD_IMAGE_STOP_CAPTURE = 2001,
    e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE = 2002,
    e_MAV_CMD_MAV_CMD_DO_TRIGGER_CONTROL = 2003,
    e_MAV_CMD_MAV_CMD_VIDEO_START_CAPTURE = 2500,
    e_MAV_CMD_MAV_CMD_VIDEO_STOP_CAPTURE = 2501,
    e_MAV_CMD_MAV_CMD_VIDEO_START_STREAMING = 2502,
    e_MAV_CMD_MAV_CMD_VIDEO_STOP_STREAMING = 2503,
    e_MAV_CMD_MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION = 2504,
    e_MAV_CMD_MAV_CMD_LOGGING_START = 2510,
    e_MAV_CMD_MAV_CMD_LOGGING_STOP = 2511,
    e_MAV_CMD_MAV_CMD_AIRFRAME_CONFIGURATION = 2520,
    e_MAV_CMD_MAV_CMD_PANORAMA_CREATE = 2800,
    e_MAV_CMD_MAV_CMD_DO_VTOL_TRANSITION = 3000,
    e_MAV_CMD_MAV_CMD_ARM_AUTHORIZATION_REQUEST = 3001,
    e_MAV_CMD_MAV_CMD_SET_GUIDED_SUBMODE_STANDARD = 4000,
    e_MAV_CMD_MAV_CMD_SET_GUIDED_SUBMODE_CIRCLE = 4001,
    e_MAV_CMD_MAV_CMD_CONDITION_GATE = 4501,
    e_MAV_CMD_MAV_CMD_NAV_FENCE_RETURN_POINT = 5000,
    e_MAV_CMD_MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION = 5001,
    e_MAV_CMD_MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION = 5002,
    e_MAV_CMD_MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION = 5003,
    e_MAV_CMD_MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION = 5004,
    e_MAV_CMD_MAV_CMD_NAV_RALLY_POINT = 5100,
    e_MAV_CMD_MAV_CMD_UAVCAN_GET_NODE_INFO = 5200,
    e_MAV_CMD_MAV_CMD_PAYLOAD_PREPARE_DEPLOY = 30001,
    e_MAV_CMD_MAV_CMD_PAYLOAD_CONTROL_DEPLOY = 30002,
    e_MAV_CMD_MAV_CMD_WAYPOINT_USER_1 = 31000,
    e_MAV_CMD_MAV_CMD_WAYPOINT_USER_2 = 31001,
    e_MAV_CMD_MAV_CMD_WAYPOINT_USER_3 = 31002,
    e_MAV_CMD_MAV_CMD_WAYPOINT_USER_4 = 31003,
    e_MAV_CMD_MAV_CMD_WAYPOINT_USER_5 = 31004,
    e_MAV_CMD_MAV_CMD_SPATIAL_USER_1 = 31005,
    e_MAV_CMD_MAV_CMD_SPATIAL_USER_2 = 31006,
    e_MAV_CMD_MAV_CMD_SPATIAL_USER_3 = 31007,
    e_MAV_CMD_MAV_CMD_SPATIAL_USER_4 = 31008,
    e_MAV_CMD_MAV_CMD_SPATIAL_USER_5 = 31009,
    e_MAV_CMD_MAV_CMD_USER_1 = 31010,
    e_MAV_CMD_MAV_CMD_USER_2 = 31011,
    e_MAV_CMD_MAV_CMD_USER_3 = 31012,
    e_MAV_CMD_MAV_CMD_USER_4 = 31013,
    e_MAV_CMD_MAV_CMD_USER_5 = 31014,
    e_MAV_CMD_MAV_CMD_RESET_MPPT = 40001,
    e_MAV_CMD_MAV_CMD_PAYLOAD_CONTROL = 40002
} e_MAV_CMD;
; /*
				                                              */
typedef  enum
{
    e_MAV_MISSION_RESULT_MAV_MISSION_ACCEPTED = 0,
    e_MAV_MISSION_RESULT_MAV_MISSION_ERROR = 1,
    e_MAV_MISSION_RESULT_MAV_MISSION_UNSUPPORTED_FRAME = 2,
    e_MAV_MISSION_RESULT_MAV_MISSION_UNSUPPORTED = 3,
    e_MAV_MISSION_RESULT_MAV_MISSION_NO_SPACE = 4,
    e_MAV_MISSION_RESULT_MAV_MISSION_INVALID = 5,
    e_MAV_MISSION_RESULT_MAV_MISSION_INVALID_PARAM1 = 6,
    e_MAV_MISSION_RESULT_MAV_MISSION_INVALID_PARAM2 = 7,
    e_MAV_MISSION_RESULT_MAV_MISSION_INVALID_PARAM3 = 8,
    e_MAV_MISSION_RESULT_MAV_MISSION_INVALID_PARAM4 = 9,
    e_MAV_MISSION_RESULT_MAV_MISSION_INVALID_PARAM5_X = 10,
    e_MAV_MISSION_RESULT_MAV_MISSION_INVALID_PARAM6_Y = 11,
    e_MAV_MISSION_RESULT_MAV_MISSION_INVALID_PARAM7 = 12,
    e_MAV_MISSION_RESULT_MAV_MISSION_INVALID_SEQUENCE = 13,
    e_MAV_MISSION_RESULT_MAV_MISSION_DENIED = 14
} e_MAV_MISSION_RESULT;
; /*
				                                              */
typedef  enum
{
    e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_NAIVE = 1,
    e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_VISION = 2,
    e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_VIO = 3,
    e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_GPS = 4,
    e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_GPS_INS = 5
} e_MAV_ESTIMATOR_TYPE;
; /*
				                                              */
typedef  enum
{
    e_MAV_RESULT_MAV_RESULT_ACCEPTED = 0,
    e_MAV_RESULT_MAV_RESULT_TEMPORARILY_REJECTED = 1,
    e_MAV_RESULT_MAV_RESULT_DENIED = 2,
    e_MAV_RESULT_MAV_RESULT_UNSUPPORTED = 3,
    e_MAV_RESULT_MAV_RESULT_FAILED = 4,
    e_MAV_RESULT_MAV_RESULT_IN_PROGRESS = 5
} e_MAV_RESULT;
; /*
				                                              */
typedef  enum
{
    e_MAV_POWER_STATUS_MAV_POWER_STATUS_BRICK_VALID = 1,
    e_MAV_POWER_STATUS_MAV_POWER_STATUS_SERVO_VALID = 2,
    e_MAV_POWER_STATUS_MAV_POWER_STATUS_USB_CONNECTED = 4,
    e_MAV_POWER_STATUS_MAV_POWER_STATUS_PERIPH_OVERCURRENT = 8,
    e_MAV_POWER_STATUS_MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT = 16,
    e_MAV_POWER_STATUS_MAV_POWER_STATUS_CHANGED = 32
} e_MAV_POWER_STATUS;
; /*
				                                              */
typedef  enum
{
    e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_TELEM1 = 0,
    e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_TELEM2 = 1,
    e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_GPS1 = 2,
    e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_GPS2 = 3,
    e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_SHELL = 10
} e_SERIAL_CONTROL_DEV;
; /*
				                                              */
typedef  enum
{
    e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_REPLY = 1,
    e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_RESPOND = 2,
    e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_EXCLUSIVE = 4,
    e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_BLOCKING = 8,
    e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_MULTI = 16
} e_SERIAL_CONTROL_FLAG;
; /*
				                                              */
typedef  enum
{
    e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_LASER = 0,
    e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_ULTRASOUND = 1,
    e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_INFRARED = 2,
    e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_RADAR = 3,
    e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_UNKNOWN = 4
} e_MAV_DISTANCE_SENSOR;
; /*
				                                              */
typedef  enum
{
    e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_NONE = 0,
    e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_YAW_45 = 1,
    e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_YAW_90 = 2,
    e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_YAW_135 = 3,
    e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_YAW_180 = 4,
    e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_YAW_225 = 5,
    e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_YAW_270 = 6,
    e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_YAW_315 = 7,
    e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_ROLL_180 = 8,
    e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_ROLL_180_YAW_45 = 9,
    e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_ROLL_180_YAW_90 = 10,
    e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_ROLL_180_YAW_135 = 11,
    e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_PITCH_180 = 12,
    e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_ROLL_180_YAW_225 = 13,
    e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_ROLL_180_YAW_270 = 14,
    e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_ROLL_180_YAW_315 = 15,
    e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_ROLL_90 = 16,
    e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_ROLL_90_YAW_45 = 17,
    e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_ROLL_90_YAW_90 = 18,
    e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_ROLL_90_YAW_135 = 19,
    e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_ROLL_270 = 20,
    e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_ROLL_270_YAW_45 = 21,
    e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_ROLL_270_YAW_90 = 22,
    e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_ROLL_270_YAW_135 = 23,
    e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_PITCH_90 = 24,
    e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_PITCH_270 = 25,
    e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_PITCH_180_YAW_90 = 26,
    e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_PITCH_180_YAW_270 = 27,
    e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_ROLL_90_PITCH_90 = 28,
    e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_ROLL_180_PITCH_90 = 29,
    e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_ROLL_270_PITCH_90 = 30,
    e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_ROLL_90_PITCH_180 = 31,
    e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_ROLL_270_PITCH_180 = 32,
    e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_ROLL_90_PITCH_270 = 33,
    e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_ROLL_180_PITCH_270 = 34,
    e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_ROLL_270_PITCH_270 = 35,
    e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_ROLL_90_PITCH_180_YAW_90 = 36,
    e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_ROLL_90_YAW_270 = 37,
    e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_ROLL_315_PITCH_315_YAW_315 = 38
} e_MAV_SENSOR_ORIENTATION;
; /*
				                                              */
typedef  enum
{
    e_MAV_BATTERY_FUNCTION_MAV_BATTERY_FUNCTION_UNKNOWN = 0,
    e_MAV_BATTERY_FUNCTION_MAV_BATTERY_FUNCTION_ALL = 1,
    e_MAV_BATTERY_FUNCTION_MAV_BATTERY_FUNCTION_PROPULSION = 2,
    e_MAV_BATTERY_FUNCTION_MAV_BATTERY_FUNCTION_AVIONICS = 3,
    e_MAV_BATTERY_FUNCTION_MAV_BATTERY_TYPE_PAYLOAD = 4
} e_MAV_BATTERY_FUNCTION;
; /*
				                                              */
typedef  enum
{
    e_MAV_BATTERY_TYPE_MAV_BATTERY_TYPE_UNKNOWN = 0,
    e_MAV_BATTERY_TYPE_MAV_BATTERY_TYPE_LIPO = 1,
    e_MAV_BATTERY_TYPE_MAV_BATTERY_TYPE_LIFE = 2,
    e_MAV_BATTERY_TYPE_MAV_BATTERY_TYPE_LION = 3,
    e_MAV_BATTERY_TYPE_MAV_BATTERY_TYPE_NIMH = 4
} e_MAV_BATTERY_TYPE;
; /*
				                                              */
typedef  enum
{
    e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT = 1,
    e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT = 2,
    e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_MISSION_INT = 4,
    e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_COMMAND_INT = 8,
    e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_PARAM_UNION = 16,
    e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_FTP = 32,
    e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET = 64,
    e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED = 128,
    e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT = 256,
    e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_TERRAIN = 512,
    e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_SET_ACTUATOR_TARGET = 1024,
    e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION = 2048,
    e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION = 4096,
    e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_MAVLINK2 = 8192,
    e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_MISSION_FENCE = 16384,
    e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_MISSION_RALLY = 32768,
    e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_FLIGHT_INFORMATION = 65536
} e_MAV_PROTOCOL_CAPABILITY;
; /*
				                                              */
typedef  enum
{
    e_LANDING_TARGET_TYPE_LANDING_TARGET_TYPE_LIGHT_BEACON = 0,
    e_LANDING_TARGET_TYPE_LANDING_TARGET_TYPE_RADIO_BEACON = 1,
    e_LANDING_TARGET_TYPE_LANDING_TARGET_TYPE_VISION_FIDUCIAL = 2,
    e_LANDING_TARGET_TYPE_LANDING_TARGET_TYPE_VISION_OTHER = 3
} e_LANDING_TARGET_TYPE;
; /*
				                                              */
typedef  enum
{
    e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_ATTITUDE = 1,
    e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_VELOCITY_HORIZ = 2,
    e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_VELOCITY_VERT = 4,
    e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_POS_HORIZ_REL = 8,
    e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_POS_HORIZ_ABS = 16,
    e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_POS_VERT_ABS = 32,
    e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_POS_VERT_AGL = 64,
    e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_CONST_POS_MODE = 128,
    e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_PRED_POS_HORIZ_REL = 256,
    e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_PRED_POS_HORIZ_ABS = 512,
    e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_GPS_GLITCH = 1024
} e_ESTIMATOR_STATUS_FLAGS;
; /*
				                                              */
typedef  enum
{
    e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_ALT = 1,
    e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_HDOP = 2,
    e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_VDOP = 4,
    e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_VEL_HORIZ = 8,
    e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_VEL_VERT = 16,
    e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY = 32,
    e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY = 64,
    e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY = 128
} e_GPS_INPUT_IGNORE_FLAGS;
; /*
				                                              */
typedef  enum
{
    e_MAV_LANDED_STATE_MAV_LANDED_STATE_UNDEFINED = 0,
    e_MAV_LANDED_STATE_MAV_LANDED_STATE_ON_GROUND = 1,
    e_MAV_LANDED_STATE_MAV_LANDED_STATE_IN_AIR = 2,
    e_MAV_LANDED_STATE_MAV_LANDED_STATE_TAKEOFF = 3,
    e_MAV_LANDED_STATE_MAV_LANDED_STATE_LANDING = 4
} e_MAV_LANDED_STATE;
; /*
				                                              */
typedef  enum
{
    e_MAV_VTOL_STATE_MAV_VTOL_STATE_UNDEFINED = 0,
    e_MAV_VTOL_STATE_MAV_VTOL_STATE_TRANSITION_TO_FW = 1,
    e_MAV_VTOL_STATE_MAV_VTOL_STATE_TRANSITION_TO_MC = 2,
    e_MAV_VTOL_STATE_MAV_VTOL_STATE_MC = 3,
    e_MAV_VTOL_STATE_MAV_VTOL_STATE_FW = 4
} e_MAV_VTOL_STATE;
; /*
				                                              */
typedef  enum
{
    e_ADSB_ALTITUDE_TYPE_ADSB_ALTITUDE_TYPE_PRESSURE_QNH = 0,
    e_ADSB_ALTITUDE_TYPE_ADSB_ALTITUDE_TYPE_GEOMETRIC = 1
} e_ADSB_ALTITUDE_TYPE;
; /*
				                                              */
typedef  enum
{
    e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_NO_INFO = 0,
    e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_LIGHT = 1,
    e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_SMALL = 2,
    e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_LARGE = 3,
    e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_HIGH_VORTEX_LARGE = 4,
    e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_HEAVY = 5,
    e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_HIGHLY_MANUV = 6,
    e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_ROTOCRAFT = 7,
    e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_UNASSIGNED = 8,
    e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_GLIDER = 9,
    e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_LIGHTER_AIR = 10,
    e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_PARACHUTE = 11,
    e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_ULTRA_LIGHT = 12,
    e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_UNASSIGNED2 = 13,
    e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_UAV = 14,
    e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_SPACE = 15,
    e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_UNASSGINED3 = 16,
    e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_EMERGENCY_SURFACE = 17,
    e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_SERVICE_SURFACE = 18,
    e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_POINT_OBSTACLE = 19
} e_ADSB_EMITTER_TYPE;
; /*
				                                              */
typedef  enum
{
    e_ADSB_FLAGS_ADSB_FLAGS_VALID_COORDS = 1,
    e_ADSB_FLAGS_ADSB_FLAGS_VALID_ALTITUDE = 2,
    e_ADSB_FLAGS_ADSB_FLAGS_VALID_HEADING = 4,
    e_ADSB_FLAGS_ADSB_FLAGS_VALID_VELOCITY = 8,
    e_ADSB_FLAGS_ADSB_FLAGS_VALID_CALLSIGN = 16,
    e_ADSB_FLAGS_ADSB_FLAGS_VALID_SQUAWK = 32,
    e_ADSB_FLAGS_ADSB_FLAGS_SIMULATED = 64
} e_ADSB_FLAGS;
; /*
				                                              */
typedef  enum
{
    e_MAV_COLLISION_SRC_MAV_COLLISION_SRC_ADSB = 0,
    e_MAV_COLLISION_SRC_MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT = 1
} e_MAV_COLLISION_SRC;
; /*
				                                              */
typedef  enum
{
    e_MAV_COLLISION_ACTION_MAV_COLLISION_ACTION_NONE = 0,
    e_MAV_COLLISION_ACTION_MAV_COLLISION_ACTION_REPORT = 1,
    e_MAV_COLLISION_ACTION_MAV_COLLISION_ACTION_ASCEND_OR_DESCEND = 2,
    e_MAV_COLLISION_ACTION_MAV_COLLISION_ACTION_MOVE_HORIZONTALLY = 3,
    e_MAV_COLLISION_ACTION_MAV_COLLISION_ACTION_MOVE_PERPENDICULAR = 4,
    e_MAV_COLLISION_ACTION_MAV_COLLISION_ACTION_RTL = 5,
    e_MAV_COLLISION_ACTION_MAV_COLLISION_ACTION_HOVER = 6
} e_MAV_COLLISION_ACTION;
; /*
				                                              */
typedef  enum
{
    e_MAV_COLLISION_THREAT_LEVEL_MAV_COLLISION_THREAT_LEVEL_NONE = 0,
    e_MAV_COLLISION_THREAT_LEVEL_MAV_COLLISION_THREAT_LEVEL_LOW = 1,
    e_MAV_COLLISION_THREAT_LEVEL_MAV_COLLISION_THREAT_LEVEL_HIGH = 2
} e_MAV_COLLISION_THREAT_LEVEL;
; /*
				                                              */
typedef  enum
{
    e_MAV_SEVERITY_MAV_SEVERITY_EMERGENCY = 0,
    e_MAV_SEVERITY_MAV_SEVERITY_ALERT = 1,
    e_MAV_SEVERITY_MAV_SEVERITY_CRITICAL = 2,
    e_MAV_SEVERITY_MAV_SEVERITY_ERROR = 3,
    e_MAV_SEVERITY_MAV_SEVERITY_WARNING = 4,
    e_MAV_SEVERITY_MAV_SEVERITY_NOTICE = 5,
    e_MAV_SEVERITY_MAV_SEVERITY_INFO = 6,
    e_MAV_SEVERITY_MAV_SEVERITY_DEBUG = 7
} e_MAV_SEVERITY;
; /*
				                                              */
typedef  enum
{
    e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_CAPTURE_VIDEO = 1,
    e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_CAPTURE_IMAGE = 2,
    e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_HAS_MODES = 4,
    e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE = 8,
    e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE = 16,
    e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE = 32
} e_CAMERA_CAP_FLAGS;
; /*
				                                              */
typedef  enum
{
    e_CAMERA_MODE_CAMERA_MODE_IMAGE = 0,
    e_CAMERA_MODE_CAMERA_MODE_VIDEO = 1,
    e_CAMERA_MODE_CAMERA_MODE_IMAGE_SURVEY = 2
} e_CAMERA_MODE;
; /*
				                                              */
typedef  enum
{
    e_UAVCAN_NODE_HEALTH_UAVCAN_NODE_HEALTH_OK = 0,
    e_UAVCAN_NODE_HEALTH_UAVCAN_NODE_HEALTH_WARNING = 1,
    e_UAVCAN_NODE_HEALTH_UAVCAN_NODE_HEALTH_ERROR = 2,
    e_UAVCAN_NODE_HEALTH_UAVCAN_NODE_HEALTH_CRITICAL = 3
} e_UAVCAN_NODE_HEALTH;
; /*
				                                              */
typedef  enum
{
    e_UAVCAN_NODE_MODE_UAVCAN_NODE_MODE_OPERATIONAL = 0,
    e_UAVCAN_NODE_MODE_UAVCAN_NODE_MODE_INITIALIZATION = 1,
    e_UAVCAN_NODE_MODE_UAVCAN_NODE_MODE_MAINTENANCE = 2,
    e_UAVCAN_NODE_MODE_UAVCAN_NODE_MODE_SOFTWARE_UPDATE = 3,
    e_UAVCAN_NODE_MODE_UAVCAN_NODE_MODE_OFFLINE = 7
} e_UAVCAN_NODE_MODE;
; /*
				                                              */
typedef  enum
{
    e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_UINT8 = 1,
    e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_INT8 = 2,
    e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_UINT16 = 3,
    e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_INT16 = 4,
    e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_UINT32 = 5,
    e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_INT32 = 6,
    e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_UINT64 = 7,
    e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_INT64 = 8,
    e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_REAL32 = 9,
    e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_REAL64 = 10,
    e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_CUSTOM = 11
} e_MAV_PARAM_EXT_TYPE;
; /*
				                                              */
typedef  enum
{
    e_PARAM_ACK_PARAM_ACK_ACCEPTED = 0,
    e_PARAM_ACK_PARAM_ACK_VALUE_UNSUPPORTED = 1,
    e_PARAM_ACK_PARAM_ACK_FAILED = 2,
    e_PARAM_ACK_PARAM_ACK_IN_PROGRESS = 3
} e_PARAM_ACK;

INLINER uint32_t p0_custom_mode_GET(Pack * src)//A bitfield for use for autopilot-specific flags
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 4)));
}
INLINER void p0_custom_mode_SET(uint32_t  src, Pack * dst)//A bitfield for use for autopilot-specific flags
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  0);
}
INLINER uint8_t p0_mavlink_version_GET(Pack * src)//MAVLink version, not writable by user, gets added by protocol because of magic data type: uint8_t_mavlink_versi
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  4, 1)));
}
INLINER void p0_mavlink_version_SET(uint8_t  src, Pack * dst)//MAVLink version, not writable by user, gets added by protocol because of magic data type: uint8_t_mavlink_versi
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  4);
}
INLINER e_MAV_TYPE p0_type_GET(Pack * src)//Type of the MAV (quadrotor, helicopter, etc., up to 15 types, defined in MAV_TYPE ENUM
{
    uint8_t * data = src->data;
    return  0 + (int)get_bits(data, 40, 5);
}
INLINER void p0_type_SET(e_MAV_TYPE  src, Pack * dst)//Type of the MAV (quadrotor, helicopter, etc., up to 15 types, defined in MAV_TYPE ENUM
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 5, data, 40);
}
INLINER e_MAV_AUTOPILOT p0_autopilot_GET(Pack * src)//Autopilot type / class. defined in MAV_AUTOPILOT ENU
{
    uint8_t * data = src->data;
    return  0 + (int)get_bits(data, 45, 5);
}
INLINER void p0_autopilot_SET(e_MAV_AUTOPILOT  src, Pack * dst)//Autopilot type / class. defined in MAV_AUTOPILOT ENU
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 5, data, 45);
}
INLINER e_MAV_MODE_FLAG p0_base_mode_GET(Pack * src)//System mode bitfield, see MAV_MODE_FLAG ENUM in mavlink/include/mavlink_types.
{
    uint8_t * data = src->data;
    switch(get_bits(data, 50, 4))
    {
        case 0:
            return e_MAV_MODE_FLAG_MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
        case 1:
            return e_MAV_MODE_FLAG_MAV_MODE_FLAG_TEST_ENABLED;
        case 2:
            return e_MAV_MODE_FLAG_MAV_MODE_FLAG_AUTO_ENABLED;
        case 3:
            return e_MAV_MODE_FLAG_MAV_MODE_FLAG_GUIDED_ENABLED;
        case 4:
            return e_MAV_MODE_FLAG_MAV_MODE_FLAG_STABILIZE_ENABLED;
        case 5:
            return e_MAV_MODE_FLAG_MAV_MODE_FLAG_HIL_ENABLED;
        case 6:
            return e_MAV_MODE_FLAG_MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
        case 7:
            return e_MAV_MODE_FLAG_MAV_MODE_FLAG_SAFETY_ARMED;
        default: ;//assert(false);//("Unknown enum ID " + id);
    }
}
INLINER void p0_base_mode_SET(e_MAV_MODE_FLAG  src, Pack * dst)//System mode bitfield, see MAV_MODE_FLAG ENUM in mavlink/include/mavlink_types.
{
    uint8_t * data = dst->data;
    int32_t id;
    switch(src)
    {
        case e_MAV_MODE_FLAG_MAV_MODE_FLAG_CUSTOM_MODE_ENABLED:
            id = 0;
            break;
        case e_MAV_MODE_FLAG_MAV_MODE_FLAG_TEST_ENABLED:
            id = 1;
            break;
        case e_MAV_MODE_FLAG_MAV_MODE_FLAG_AUTO_ENABLED:
            id = 2;
            break;
        case e_MAV_MODE_FLAG_MAV_MODE_FLAG_GUIDED_ENABLED:
            id = 3;
            break;
        case e_MAV_MODE_FLAG_MAV_MODE_FLAG_STABILIZE_ENABLED:
            id = 4;
            break;
        case e_MAV_MODE_FLAG_MAV_MODE_FLAG_HIL_ENABLED:
            id = 5;
            break;
        case e_MAV_MODE_FLAG_MAV_MODE_FLAG_MANUAL_INPUT_ENABLED:
            id = 6;
            break;
        case e_MAV_MODE_FLAG_MAV_MODE_FLAG_SAFETY_ARMED:
            id = 7;
            break;
        default: ;// assert(false);//("Unknown enum" + id);
    }
    set_bits(id, 4, data, 50);
}
INLINER e_MAV_STATE p0_system_status_GET(Pack * src)//System status flag, see MAV_STATE ENU
{
    uint8_t * data = src->data;
    return  0 + (int)get_bits(data, 54, 4);
}
INLINER void p0_system_status_SET(e_MAV_STATE  src, Pack * dst)//System status flag, see MAV_STATE ENU
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 4, data, 54);
}
INLINER uint16_t p1_load_GET(Pack * src)//Maximum usage in percent of the mainloop time, (0%: 0, 100%: 1000) should be always below 100
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER void p1_load_SET(uint16_t  src, Pack * dst)//Maximum usage in percent of the mainloop time, (0%: 0, 100%: 1000) should be always below 100
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER uint16_t p1_voltage_battery_GET(Pack * src)//Battery voltage, in millivolts (1 = 1 millivolt
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 2)));
}
INLINER void p1_voltage_battery_SET(uint16_t  src, Pack * dst)//Battery voltage, in millivolts (1 = 1 millivolt
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  2);
}
INLINER uint16_t p1_drop_rate_comm_GET(Pack * src)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  4, 2)));
}
INLINER void p1_drop_rate_comm_SET(uint16_t  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  4);
}
INLINER uint16_t p1_errors_comm_GET(Pack * src)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  6, 2)));
}
INLINER void p1_errors_comm_SET(uint16_t  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  6);
}
INLINER uint16_t p1_errors_count1_GET(Pack * src)//Autopilot-specific error
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  8, 2)));
}
INLINER void p1_errors_count1_SET(uint16_t  src, Pack * dst)//Autopilot-specific error
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  8);
}
INLINER uint16_t p1_errors_count2_GET(Pack * src)//Autopilot-specific error
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  10, 2)));
}
INLINER void p1_errors_count2_SET(uint16_t  src, Pack * dst)//Autopilot-specific error
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  10);
}
INLINER uint16_t p1_errors_count3_GET(Pack * src)//Autopilot-specific error
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  12, 2)));
}
INLINER void p1_errors_count3_SET(uint16_t  src, Pack * dst)//Autopilot-specific error
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  12);
}
INLINER uint16_t p1_errors_count4_GET(Pack * src)//Autopilot-specific error
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  14, 2)));
}
INLINER void p1_errors_count4_SET(uint16_t  src, Pack * dst)//Autopilot-specific error
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  14);
}
INLINER int16_t p1_current_battery_GET(Pack * src)//Battery current, in 10*milliamperes (1 = 10 milliampere), -1: autopilot does not measure the curre
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  16, 2)));
}
INLINER void p1_current_battery_SET(int16_t  src, Pack * dst)//Battery current, in 10*milliamperes (1 = 10 milliampere), -1: autopilot does not measure the curre
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  16);
}
INLINER int8_t p1_battery_remaining_GET(Pack * src)//Remaining battery energy: (0%: 0, 100%: 100), -1: autopilot estimate the remaining batter
{
    uint8_t * data = src->data;
    return ((int8_t)(get_bytes(data,  18, 1)));
}
INLINER void p1_battery_remaining_SET(int8_t  src, Pack * dst)//Remaining battery energy: (0%: 0, 100%: 100), -1: autopilot estimate the remaining batter
{
    uint8_t * data = dst->data;
    set_bytes((uint8_t)(src), 1, data,  18);
}
INLINER e_MAV_SYS_STATUS_SENSOR p1_onboard_control_sensors_present_GET(Pack * src)
{
    uint8_t * data = src->data;
    switch(get_bits(data, 152, 5))
    {
        case 0:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_GYRO;
        case 1:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_ACCEL;
        case 2:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_MAG;
        case 3:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE;
        case 4:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE;
        case 5:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_GPS;
        case 6:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW;
        case 7:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_VISION_POSITION;
        case 8:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_LASER_POSITION;
        case 9:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH;
        case 10:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL;
        case 11:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION;
        case 12:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_YAW_POSITION;
        case 13:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL;
        case 14:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL;
        case 15:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS;
        case 16:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_RC_RECEIVER;
        case 17:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_GYRO2;
        case 18:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_ACCEL2;
        case 19:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_MAG2;
        case 20:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_GEOFENCE;
        case 21:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_AHRS;
        case 22:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_TERRAIN;
        case 23:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_REVERSE_MOTOR;
        case 24:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_LOGGING;
        case 25:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_BATTERY;
        default: ;//assert(false);//("Unknown enum ID " + id);
    }
}
INLINER void p1_onboard_control_sensors_present_SET(e_MAV_SYS_STATUS_SENSOR  src, Pack * dst)
{
    uint8_t * data = dst->data;
    int32_t id;
    switch(src)
    {
        case e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_GYRO:
            id = 0;
            break;
        case e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_ACCEL:
            id = 1;
            break;
        case e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_MAG:
            id = 2;
            break;
        case e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE:
            id = 3;
            break;
        case e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE:
            id = 4;
            break;
        case e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_GPS:
            id = 5;
            break;
        case e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW:
            id = 6;
            break;
        case e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_VISION_POSITION:
            id = 7;
            break;
        case e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_LASER_POSITION:
            id = 8;
            break;
        case e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH:
            id = 9;
            break;
        case e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL:
            id = 10;
            break;
        case e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION:
            id = 11;
            break;
        case e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_YAW_POSITION:
            id = 12;
            break;
        case e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL:
            id = 13;
            break;
        case e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL:
            id = 14;
            break;
        case e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS:
            id = 15;
            break;
        case e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_RC_RECEIVER:
            id = 16;
            break;
        case e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_GYRO2:
            id = 17;
            break;
        case e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_ACCEL2:
            id = 18;
            break;
        case e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_MAG2:
            id = 19;
            break;
        case e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_GEOFENCE:
            id = 20;
            break;
        case e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_AHRS:
            id = 21;
            break;
        case e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_TERRAIN:
            id = 22;
            break;
        case e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_REVERSE_MOTOR:
            id = 23;
            break;
        case e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_LOGGING:
            id = 24;
            break;
        case e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_BATTERY:
            id = 25;
            break;
        default: ;// assert(false);//("Unknown enum" + id);
    }
    set_bits(id, 5, data, 152);
}
INLINER e_MAV_SYS_STATUS_SENSOR p1_onboard_control_sensors_enabled_GET(Pack * src)
{
    uint8_t * data = src->data;
    switch(get_bits(data, 157, 5))
    {
        case 0:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_GYRO;
        case 1:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_ACCEL;
        case 2:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_MAG;
        case 3:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE;
        case 4:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE;
        case 5:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_GPS;
        case 6:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW;
        case 7:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_VISION_POSITION;
        case 8:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_LASER_POSITION;
        case 9:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH;
        case 10:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL;
        case 11:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION;
        case 12:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_YAW_POSITION;
        case 13:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL;
        case 14:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL;
        case 15:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS;
        case 16:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_RC_RECEIVER;
        case 17:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_GYRO2;
        case 18:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_ACCEL2;
        case 19:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_MAG2;
        case 20:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_GEOFENCE;
        case 21:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_AHRS;
        case 22:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_TERRAIN;
        case 23:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_REVERSE_MOTOR;
        case 24:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_LOGGING;
        case 25:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_BATTERY;
        default: ;//assert(false);//("Unknown enum ID " + id);
    }
}
INLINER void p1_onboard_control_sensors_enabled_SET(e_MAV_SYS_STATUS_SENSOR  src, Pack * dst)
{
    uint8_t * data = dst->data;
    int32_t id;
    switch(src)
    {
        case e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_GYRO:
            id = 0;
            break;
        case e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_ACCEL:
            id = 1;
            break;
        case e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_MAG:
            id = 2;
            break;
        case e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE:
            id = 3;
            break;
        case e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE:
            id = 4;
            break;
        case e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_GPS:
            id = 5;
            break;
        case e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW:
            id = 6;
            break;
        case e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_VISION_POSITION:
            id = 7;
            break;
        case e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_LASER_POSITION:
            id = 8;
            break;
        case e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH:
            id = 9;
            break;
        case e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL:
            id = 10;
            break;
        case e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION:
            id = 11;
            break;
        case e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_YAW_POSITION:
            id = 12;
            break;
        case e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL:
            id = 13;
            break;
        case e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL:
            id = 14;
            break;
        case e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS:
            id = 15;
            break;
        case e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_RC_RECEIVER:
            id = 16;
            break;
        case e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_GYRO2:
            id = 17;
            break;
        case e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_ACCEL2:
            id = 18;
            break;
        case e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_MAG2:
            id = 19;
            break;
        case e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_GEOFENCE:
            id = 20;
            break;
        case e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_AHRS:
            id = 21;
            break;
        case e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_TERRAIN:
            id = 22;
            break;
        case e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_REVERSE_MOTOR:
            id = 23;
            break;
        case e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_LOGGING:
            id = 24;
            break;
        case e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_BATTERY:
            id = 25;
            break;
        default: ;// assert(false);//("Unknown enum" + id);
    }
    set_bits(id, 5, data, 157);
}
INLINER e_MAV_SYS_STATUS_SENSOR p1_onboard_control_sensors_health_GET(Pack * src)
{
    uint8_t * data = src->data;
    switch(get_bits(data, 162, 5))
    {
        case 0:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_GYRO;
        case 1:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_ACCEL;
        case 2:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_MAG;
        case 3:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE;
        case 4:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE;
        case 5:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_GPS;
        case 6:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW;
        case 7:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_VISION_POSITION;
        case 8:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_LASER_POSITION;
        case 9:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH;
        case 10:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL;
        case 11:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION;
        case 12:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_YAW_POSITION;
        case 13:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL;
        case 14:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL;
        case 15:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS;
        case 16:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_RC_RECEIVER;
        case 17:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_GYRO2;
        case 18:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_ACCEL2;
        case 19:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_MAG2;
        case 20:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_GEOFENCE;
        case 21:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_AHRS;
        case 22:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_TERRAIN;
        case 23:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_REVERSE_MOTOR;
        case 24:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_LOGGING;
        case 25:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_BATTERY;
        default: ;//assert(false);//("Unknown enum ID " + id);
    }
}
INLINER void p1_onboard_control_sensors_health_SET(e_MAV_SYS_STATUS_SENSOR  src, Pack * dst)
{
    uint8_t * data = dst->data;
    int32_t id;
    switch(src)
    {
        case e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_GYRO:
            id = 0;
            break;
        case e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_ACCEL:
            id = 1;
            break;
        case e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_MAG:
            id = 2;
            break;
        case e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE:
            id = 3;
            break;
        case e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE:
            id = 4;
            break;
        case e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_GPS:
            id = 5;
            break;
        case e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW:
            id = 6;
            break;
        case e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_VISION_POSITION:
            id = 7;
            break;
        case e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_LASER_POSITION:
            id = 8;
            break;
        case e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH:
            id = 9;
            break;
        case e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL:
            id = 10;
            break;
        case e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION:
            id = 11;
            break;
        case e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_YAW_POSITION:
            id = 12;
            break;
        case e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL:
            id = 13;
            break;
        case e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL:
            id = 14;
            break;
        case e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS:
            id = 15;
            break;
        case e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_RC_RECEIVER:
            id = 16;
            break;
        case e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_GYRO2:
            id = 17;
            break;
        case e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_ACCEL2:
            id = 18;
            break;
        case e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_MAG2:
            id = 19;
            break;
        case e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_GEOFENCE:
            id = 20;
            break;
        case e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_AHRS:
            id = 21;
            break;
        case e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_TERRAIN:
            id = 22;
            break;
        case e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_REVERSE_MOTOR:
            id = 23;
            break;
        case e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_LOGGING:
            id = 24;
            break;
        case e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_BATTERY:
            id = 25;
            break;
        default: ;// assert(false);//("Unknown enum" + id);
    }
    set_bits(id, 5, data, 162);
}
INLINER uint32_t p2_time_boot_ms_GET(Pack * src)//Timestamp of the component clock since boot time in milliseconds
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 4)));
}
INLINER void p2_time_boot_ms_SET(uint32_t  src, Pack * dst)//Timestamp of the component clock since boot time in milliseconds
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  0);
}
INLINER uint64_t p2_time_unix_usec_GET(Pack * src)//Timestamp of the master clock in microseconds since UNIX epoch
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  4, 8)));
}
INLINER void p2_time_unix_usec_SET(uint64_t  src, Pack * dst)//Timestamp of the master clock in microseconds since UNIX epoch
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  4);
}
INLINER uint16_t p3_type_mask_GET(Pack * src)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER void p3_type_mask_SET(uint16_t  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER uint32_t p3_time_boot_ms_GET(Pack * src)//Timestamp in milliseconds since system boo
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 4)));
}
INLINER void p3_time_boot_ms_SET(uint32_t  src, Pack * dst)//Timestamp in milliseconds since system boo
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  2);
}
INLINER float p3_x_GET(Pack * src)//X Position in NED frame in meter
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  6, 4)));
}
INLINER void p3_x_SET(float  src, Pack * dst)//X Position in NED frame in meter
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  6);
}
INLINER float p3_y_GET(Pack * src)//Y Position in NED frame in meter
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  10, 4)));
}
INLINER void p3_y_SET(float  src, Pack * dst)//Y Position in NED frame in meter
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  10);
}
INLINER float p3_z_GET(Pack * src)//Z Position in NED frame in meters (note, altitude is negative in NED
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  14, 4)));
}
INLINER void p3_z_SET(float  src, Pack * dst)//Z Position in NED frame in meters (note, altitude is negative in NED
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  14);
}
INLINER float p3_vx_GET(Pack * src)//X velocity in NED frame in meter /
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  18, 4)));
}
INLINER void p3_vx_SET(float  src, Pack * dst)//X velocity in NED frame in meter /
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  18);
}
INLINER float p3_vy_GET(Pack * src)//Y velocity in NED frame in meter /
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  22, 4)));
}
INLINER void p3_vy_SET(float  src, Pack * dst)//Y velocity in NED frame in meter /
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  22);
}
INLINER float p3_vz_GET(Pack * src)//Z velocity in NED frame in meter /
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  26, 4)));
}
INLINER void p3_vz_SET(float  src, Pack * dst)//Z velocity in NED frame in meter /
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  26);
}
INLINER float p3_afx_GET(Pack * src)//X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  30, 4)));
}
INLINER void p3_afx_SET(float  src, Pack * dst)//X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  30);
}
INLINER float p3_afy_GET(Pack * src)//Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  34, 4)));
}
INLINER void p3_afy_SET(float  src, Pack * dst)//Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  34);
}
INLINER float p3_afz_GET(Pack * src)//Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  38, 4)));
}
INLINER void p3_afz_SET(float  src, Pack * dst)//Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  38);
}
INLINER float p3_yaw_GET(Pack * src)//yaw setpoint in ra
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  42, 4)));
}
INLINER void p3_yaw_SET(float  src, Pack * dst)//yaw setpoint in ra
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  42);
}
INLINER float p3_yaw_rate_GET(Pack * src)//yaw rate setpoint in rad/
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  46, 4)));
}
INLINER void p3_yaw_rate_SET(float  src, Pack * dst)//yaw rate setpoint in rad/
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  46);
}
INLINER e_MAV_FRAME p3_coordinate_frame_GET(Pack * src)
{
    uint8_t * data = src->data;
    return  0 + (int)get_bits(data, 400, 4);
}
INLINER void p3_coordinate_frame_SET(e_MAV_FRAME  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 4, data, 400);
}
INLINER uint32_t p4_seq_GET(Pack * src)//PING sequenc
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 4)));
}
INLINER void p4_seq_SET(uint32_t  src, Pack * dst)//PING sequenc
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  0);
}
INLINER uint64_t p4_time_usec_GET(Pack * src)//Unix timestamp in microseconds or since system boot if smaller than MAVLink epoch (1.1.2009
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  4, 8)));
}
INLINER void p4_time_usec_SET(uint64_t  src, Pack * dst)//Unix timestamp in microseconds or since system boot if smaller than MAVLink epoch (1.1.2009
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  4);
}
INLINER uint8_t p4_target_system_GET(Pack * src)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  12, 1)));
}
INLINER void p4_target_system_SET(uint8_t  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  12);
}
INLINER uint8_t p4_target_component_GET(Pack * src)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  13, 1)));
}
INLINER void p4_target_component_SET(uint8_t  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  13);
}
INLINER uint8_t p5_target_system_GET(Pack * src)//System the GCS requests control fo
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 1)));
}
INLINER void p5_target_system_SET(uint8_t  src, Pack * dst)//System the GCS requests control fo
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  0);
}
INLINER uint8_t p5_control_request_GET(Pack * src)//0: request control of this MAV, 1: Release control of this MA
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  1, 1)));
}
INLINER void p5_control_request_SET(uint8_t  src, Pack * dst)//0: request control of this MAV, 1: Release control of this MA
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  1);
}
INLINER uint8_t p5_version_GET(Pack * src)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 1)));
}
INLINER void p5_version_SET(uint8_t  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  2);
}
INLINER char16_t * p5_passkey_GET(Bounds_Inside * src, char16_t *  dst, int32_t pos)
{
    uint8_t * data = src->base.pack->data;
    for(int32_t BYTE = src->BYTE, dst_max = pos + src->items; pos < dst_max ; pos++,  BYTE += 2)
        dst[pos] = ((get_bytes(data,  BYTE, 2)));
    return dst;
}
INLINER int32_t p5_passkey_LEN(Bounds_Inside * src)
{
    return (src->base.field_bit !=  24 && !try_visit_field(src, 24)  ||  !try_visit_item(src, 0)) ? 0 : src->items;
}
INLINER  char16_t* p5_passkey_TRY_(Bounds_Inside * src)
{
    if(src->base.field_bit !=  24 && !try_visit_field(src, 24)  ||  !try_visit_item(src, 0)) return NULL;
    char16_t * ret = p5_passkey_GET(src, malloc((src->items + 1) * 2), 0);
    ret[src->items] = 0;//the terminating null character.
    return ret;
}
INLINER void p5_passkey_SET(char16_t *  src, int32_t pos, int32_t items, Bounds_Inside * dst)
{
    if(dst->base.field_bit != 24 && insert_field(dst, 24, items) ||
            ! try_visit_item(dst, 0)) insert_item(dst, 0, items);
    uint8_t * data = dst->base.pack->data;
    for(int32_t BYTE =  dst->BYTE, src_max = pos + dst->items; pos < src_max; pos++, BYTE += 2)
        set_bytes((uint16_t)(src[pos]), 2, data,  BYTE);
}
INLINER void p5_passkey_SET_(char16_t*  src, Bounds_Inside * dst) {p5_passkey_SET(src, 0, strlen16(src), dst);}
INLINER uint8_t p6_gcs_system_id_GET(Pack * src)//ID of the GCS this message
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 1)));
}
INLINER void p6_gcs_system_id_SET(uint8_t  src, Pack * dst)//ID of the GCS this message
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  0);
}
INLINER uint8_t p6_control_request_GET(Pack * src)//0: request control of this MAV, 1: Release control of this MA
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  1, 1)));
}
INLINER void p6_control_request_SET(uint8_t  src, Pack * dst)//0: request control of this MAV, 1: Release control of this MA
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  1);
}
INLINER uint8_t p6_ack_GET(Pack * src)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 1)));
}
INLINER void p6_ack_SET(uint8_t  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  2);
}
INLINER char16_t * p7_key_GET(Bounds_Inside * src, char16_t *  dst, int32_t pos) //ke
{
    uint8_t * data = src->base.pack->data;
    for(int32_t BYTE = src->BYTE, dst_max = pos + src->items; pos < dst_max ; pos++,  BYTE += 2)
        dst[pos] = ((get_bytes(data,  BYTE, 2)));
    return dst;
}
INLINER int32_t p7_key_LEN(Bounds_Inside * src)
{
    return (src->base.field_bit !=  0 && !try_visit_field(src, 0)  ||  !try_visit_item(src, 0)) ? 0 : src->items;
}
INLINER  char16_t* p7_key_TRY_(Bounds_Inside * src)
{
    if(src->base.field_bit !=  0 && !try_visit_field(src, 0)  ||  !try_visit_item(src, 0)) return NULL;
    char16_t * ret = p7_key_GET(src, malloc((src->items + 1) * 2), 0);
    ret[src->items] = 0;//the terminating null character.
    return ret;
}
INLINER void p7_key_SET(char16_t *  src, int32_t pos, int32_t items, Bounds_Inside * dst) //ke
{
    if(dst->base.field_bit != 0 && insert_field(dst, 0, items) ||
            ! try_visit_item(dst, 0)) insert_item(dst, 0, items);
    uint8_t * data = dst->base.pack->data;
    for(int32_t BYTE =  dst->BYTE, src_max = pos + dst->items; pos < src_max; pos++, BYTE += 2)
        set_bytes((uint16_t)(src[pos]), 2, data,  BYTE);
}
INLINER void p7_key_SET_(char16_t*  src, Bounds_Inside * dst) {p7_key_SET(src, 0, strlen16(src), dst);}
INLINER uint32_t p11_custom_mode_GET(Pack * src)//The new autopilot-specific mode. This field can be ignored by an autopilot
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 4)));
}
INLINER void p11_custom_mode_SET(uint32_t  src, Pack * dst)//The new autopilot-specific mode. This field can be ignored by an autopilot
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  0);
}
INLINER uint8_t p11_target_system_GET(Pack * src)//The system setting the mod
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  4, 1)));
}
INLINER void p11_target_system_SET(uint8_t  src, Pack * dst)//The system setting the mod
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  4);
}
INLINER e_MAV_MODE p11_base_mode_GET(Pack * src)//The new base mod
{
    uint8_t * data = src->data;
    switch(get_bits(data, 40, 4))
    {
        case 0:
            return e_MAV_MODE_MAV_MODE_PREFLIGHT;
        case 1:
            return e_MAV_MODE_MAV_MODE_MANUAL_DISARMED;
        case 2:
            return e_MAV_MODE_MAV_MODE_TEST_DISARMED;
        case 3:
            return e_MAV_MODE_MAV_MODE_STABILIZE_DISARMED;
        case 4:
            return e_MAV_MODE_MAV_MODE_GUIDED_DISARMED;
        case 5:
            return e_MAV_MODE_MAV_MODE_AUTO_DISARMED;
        case 6:
            return e_MAV_MODE_MAV_MODE_MANUAL_ARMED;
        case 7:
            return e_MAV_MODE_MAV_MODE_TEST_ARMED;
        case 8:
            return e_MAV_MODE_MAV_MODE_STABILIZE_ARMED;
        case 9:
            return e_MAV_MODE_MAV_MODE_GUIDED_ARMED;
        case 10:
            return e_MAV_MODE_MAV_MODE_AUTO_ARMED;
        default: ;//assert(false);//("Unknown enum ID " + id);
    }
}
INLINER void p11_base_mode_SET(e_MAV_MODE  src, Pack * dst)//The new base mod
{
    uint8_t * data = dst->data;
    int32_t id;
    switch(src)
    {
        case e_MAV_MODE_MAV_MODE_PREFLIGHT:
            id = 0;
            break;
        case e_MAV_MODE_MAV_MODE_MANUAL_DISARMED:
            id = 1;
            break;
        case e_MAV_MODE_MAV_MODE_TEST_DISARMED:
            id = 2;
            break;
        case e_MAV_MODE_MAV_MODE_STABILIZE_DISARMED:
            id = 3;
            break;
        case e_MAV_MODE_MAV_MODE_GUIDED_DISARMED:
            id = 4;
            break;
        case e_MAV_MODE_MAV_MODE_AUTO_DISARMED:
            id = 5;
            break;
        case e_MAV_MODE_MAV_MODE_MANUAL_ARMED:
            id = 6;
            break;
        case e_MAV_MODE_MAV_MODE_TEST_ARMED:
            id = 7;
            break;
        case e_MAV_MODE_MAV_MODE_STABILIZE_ARMED:
            id = 8;
            break;
        case e_MAV_MODE_MAV_MODE_GUIDED_ARMED:
            id = 9;
            break;
        case e_MAV_MODE_MAV_MODE_AUTO_ARMED:
            id = 10;
            break;
        default: ;// assert(false);//("Unknown enum" + id);
    }
    set_bits(id, 4, data, 40);
}
INLINER uint8_t p20_target_system_GET(Pack * src)//System I
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 1)));
}
INLINER void p20_target_system_SET(uint8_t  src, Pack * dst)//System I
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  0);
}
INLINER uint8_t p20_target_component_GET(Pack * src)//Component I
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  1, 1)));
}
INLINER void p20_target_component_SET(uint8_t  src, Pack * dst)//Component I
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  1);
}
INLINER int16_t p20_param_index_GET(Pack * src)//Parameter index. Send -1 to use the param ID field as identifier (else the param id will be ignore
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  2, 2)));
}
INLINER void p20_param_index_SET(int16_t  src, Pack * dst)//Parameter index. Send -1 to use the param ID field as identifier (else the param id will be ignore
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  2);
}
INLINER char16_t * p20_param_id_GET(Bounds_Inside * src, char16_t *  dst, int32_t pos)
{
    uint8_t * data = src->base.pack->data;
    for(int32_t BYTE = src->BYTE, dst_max = pos + src->items; pos < dst_max ; pos++,  BYTE += 2)
        dst[pos] = ((get_bytes(data,  BYTE, 2)));
    return dst;
}
INLINER int32_t p20_param_id_LEN(Bounds_Inside * src)
{
    return (src->base.field_bit !=  32 && !try_visit_field(src, 32)  ||  !try_visit_item(src, 0)) ? 0 : src->items;
}
INLINER  char16_t* p20_param_id_TRY_(Bounds_Inside * src)
{
    if(src->base.field_bit !=  32 && !try_visit_field(src, 32)  ||  !try_visit_item(src, 0)) return NULL;
    char16_t * ret = p20_param_id_GET(src, malloc((src->items + 1) * 2), 0);
    ret[src->items] = 0;//the terminating null character.
    return ret;
}
INLINER void p20_param_id_SET(char16_t *  src, int32_t pos, int32_t items, Bounds_Inside * dst)
{
    if(dst->base.field_bit != 32 && insert_field(dst, 32, items) ||
            ! try_visit_item(dst, 0)) insert_item(dst, 0, items);
    uint8_t * data = dst->base.pack->data;
    for(int32_t BYTE =  dst->BYTE, src_max = pos + dst->items; pos < src_max; pos++, BYTE += 2)
        set_bytes((uint16_t)(src[pos]), 2, data,  BYTE);
}
INLINER void p20_param_id_SET_(char16_t*  src, Bounds_Inside * dst) {p20_param_id_SET(src, 0, strlen16(src), dst);}
INLINER uint8_t p21_target_system_GET(Pack * src)//System I
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 1)));
}
INLINER void p21_target_system_SET(uint8_t  src, Pack * dst)//System I
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  0);
}
INLINER uint8_t p21_target_component_GET(Pack * src)//Component I
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  1, 1)));
}
INLINER void p21_target_component_SET(uint8_t  src, Pack * dst)//Component I
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  1);
}
INLINER uint16_t p22_param_count_GET(Pack * src)//Total number of onboard parameter
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER void p22_param_count_SET(uint16_t  src, Pack * dst)//Total number of onboard parameter
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER uint16_t p22_param_index_GET(Pack * src)//Index of this onboard paramete
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 2)));
}
INLINER void p22_param_index_SET(uint16_t  src, Pack * dst)//Index of this onboard paramete
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  2);
}
INLINER float p22_param_value_GET(Pack * src)//Onboard parameter valu
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  4, 4)));
}
INLINER void p22_param_value_SET(float  src, Pack * dst)//Onboard parameter valu
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  4);
}
INLINER e_MAV_PARAM_TYPE p22_param_type_GET(Pack * src)//Onboard parameter type: see the MAV_PARAM_TYPE enum for supported data types
{
    uint8_t * data = src->data;
    return  1 + (int)get_bits(data, 64, 4);
}
INLINER void p22_param_type_SET(e_MAV_PARAM_TYPE  src, Pack * dst)//Onboard parameter type: see the MAV_PARAM_TYPE enum for supported data types
{
    uint8_t * data = dst->data;
    set_bits(- 1 +   src, 4, data, 64);
}
INLINER char16_t * p22_param_id_GET(Bounds_Inside * src, char16_t *  dst, int32_t pos)
{
    uint8_t * data = src->base.pack->data;
    for(int32_t BYTE = src->BYTE, dst_max = pos + src->items; pos < dst_max ; pos++,  BYTE += 2)
        dst[pos] = ((get_bytes(data,  BYTE, 2)));
    return dst;
}
INLINER int32_t p22_param_id_LEN(Bounds_Inside * src)
{
    return (src->base.field_bit !=  68 && !try_visit_field(src, 68)  ||  !try_visit_item(src, 0)) ? 0 : src->items;
}
INLINER  char16_t* p22_param_id_TRY_(Bounds_Inside * src)
{
    if(src->base.field_bit !=  68 && !try_visit_field(src, 68)  ||  !try_visit_item(src, 0)) return NULL;
    char16_t * ret = p22_param_id_GET(src, malloc((src->items + 1) * 2), 0);
    ret[src->items] = 0;//the terminating null character.
    return ret;
}
INLINER void p22_param_id_SET(char16_t *  src, int32_t pos, int32_t items, Bounds_Inside * dst)
{
    if(dst->base.field_bit != 68 && insert_field(dst, 68, items) ||
            ! try_visit_item(dst, 0)) insert_item(dst, 0, items);
    uint8_t * data = dst->base.pack->data;
    for(int32_t BYTE =  dst->BYTE, src_max = pos + dst->items; pos < src_max; pos++, BYTE += 2)
        set_bytes((uint16_t)(src[pos]), 2, data,  BYTE);
}
INLINER void p22_param_id_SET_(char16_t*  src, Bounds_Inside * dst) {p22_param_id_SET(src, 0, strlen16(src), dst);}
INLINER uint8_t p23_target_system_GET(Pack * src)//System I
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 1)));
}
INLINER void p23_target_system_SET(uint8_t  src, Pack * dst)//System I
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  0);
}
INLINER uint8_t p23_target_component_GET(Pack * src)//Component I
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  1, 1)));
}
INLINER void p23_target_component_SET(uint8_t  src, Pack * dst)//Component I
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  1);
}
INLINER float p23_param_value_GET(Pack * src)//Onboard parameter valu
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  2, 4)));
}
INLINER void p23_param_value_SET(float  src, Pack * dst)//Onboard parameter valu
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  2);
}
INLINER e_MAV_PARAM_TYPE p23_param_type_GET(Pack * src)//Onboard parameter type: see the MAV_PARAM_TYPE enum for supported data types
{
    uint8_t * data = src->data;
    return  1 + (int)get_bits(data, 48, 4);
}
INLINER void p23_param_type_SET(e_MAV_PARAM_TYPE  src, Pack * dst)//Onboard parameter type: see the MAV_PARAM_TYPE enum for supported data types
{
    uint8_t * data = dst->data;
    set_bits(- 1 +   src, 4, data, 48);
}
INLINER char16_t * p23_param_id_GET(Bounds_Inside * src, char16_t *  dst, int32_t pos)
{
    uint8_t * data = src->base.pack->data;
    for(int32_t BYTE = src->BYTE, dst_max = pos + src->items; pos < dst_max ; pos++,  BYTE += 2)
        dst[pos] = ((get_bytes(data,  BYTE, 2)));
    return dst;
}
INLINER int32_t p23_param_id_LEN(Bounds_Inside * src)
{
    return (src->base.field_bit !=  52 && !try_visit_field(src, 52)  ||  !try_visit_item(src, 0)) ? 0 : src->items;
}
INLINER  char16_t* p23_param_id_TRY_(Bounds_Inside * src)
{
    if(src->base.field_bit !=  52 && !try_visit_field(src, 52)  ||  !try_visit_item(src, 0)) return NULL;
    char16_t * ret = p23_param_id_GET(src, malloc((src->items + 1) * 2), 0);
    ret[src->items] = 0;//the terminating null character.
    return ret;
}
INLINER void p23_param_id_SET(char16_t *  src, int32_t pos, int32_t items, Bounds_Inside * dst)
{
    if(dst->base.field_bit != 52 && insert_field(dst, 52, items) ||
            ! try_visit_item(dst, 0)) insert_item(dst, 0, items);
    uint8_t * data = dst->base.pack->data;
    for(int32_t BYTE =  dst->BYTE, src_max = pos + dst->items; pos < src_max; pos++, BYTE += 2)
        set_bytes((uint16_t)(src[pos]), 2, data,  BYTE);
}
INLINER void p23_param_id_SET_(char16_t*  src, Bounds_Inside * dst) {p23_param_id_SET(src, 0, strlen16(src), dst);}
INLINER uint16_t p24_eph_GET(Pack * src)//GPS HDOP horizontal dilution of position (unitless). If unknown, set to: UINT16_MA
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER void p24_eph_SET(uint16_t  src, Pack * dst)//GPS HDOP horizontal dilution of position (unitless). If unknown, set to: UINT16_MA
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER uint16_t p24_epv_GET(Pack * src)//GPS VDOP vertical dilution of position (unitless). If unknown, set to: UINT16_MA
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 2)));
}
INLINER void p24_epv_SET(uint16_t  src, Pack * dst)//GPS VDOP vertical dilution of position (unitless). If unknown, set to: UINT16_MA
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  2);
}
INLINER uint16_t p24_vel_GET(Pack * src)//GPS ground speed (m/s * 100). If unknown, set to: UINT16_MA
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  4, 2)));
}
INLINER void p24_vel_SET(uint16_t  src, Pack * dst)//GPS ground speed (m/s * 100). If unknown, set to: UINT16_MA
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  4);
}
INLINER uint16_t p24_cog_GET(Pack * src)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  6, 2)));
}
INLINER void p24_cog_SET(uint16_t  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  6);
}
INLINER uint64_t p24_time_usec_GET(Pack * src)//Timestamp (microseconds since UNIX epoch or microseconds since system boot
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  8, 8)));
}
INLINER void p24_time_usec_SET(uint64_t  src, Pack * dst)//Timestamp (microseconds since UNIX epoch or microseconds since system boot
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  8);
}
INLINER int32_t p24_lat_GET(Pack * src)//Latitude (WGS84, EGM96 ellipsoid), in degrees * 1E
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  16, 4)));
}
INLINER void p24_lat_SET(int32_t  src, Pack * dst)//Latitude (WGS84, EGM96 ellipsoid), in degrees * 1E
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  16);
}
INLINER int32_t p24_lon_GET(Pack * src)//Longitude (WGS84, EGM96 ellipsoid), in degrees * 1E
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  20, 4)));
}
INLINER void p24_lon_SET(int32_t  src, Pack * dst)//Longitude (WGS84, EGM96 ellipsoid), in degrees * 1E
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  20);
}
INLINER int32_t p24_alt_GET(Pack * src)
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  24, 4)));
}
INLINER void p24_alt_SET(int32_t  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  24);
}
INLINER uint8_t p24_satellites_visible_GET(Pack * src)//Number of satellites visible. If unknown, set to 25
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  28, 1)));
}
INLINER void p24_satellites_visible_SET(uint8_t  src, Pack * dst)//Number of satellites visible. If unknown, set to 25
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  28);
}
INLINER e_GPS_FIX_TYPE p24_fix_type_GET(Pack * src)//See the GPS_FIX_TYPE enum
{
    uint8_t * data = src->data;
    return  0 + (int)get_bits(data, 232, 4);
}
INLINER void p24_fix_type_SET(e_GPS_FIX_TYPE  src, Pack * dst)//See the GPS_FIX_TYPE enum
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 4, data, 232);
}
INLINER int32_t  p24_alt_ellipsoid_TRY(Bounds_Inside * src)
{
    if(src->base.field_bit !=  236 && !try_visit_field(src, 236)) return 0;
    uint8_t * data = src->base.pack->data;
    return ((int32_t)(get_bytes(data,  src->BYTE, 4)));
}
INLINER void p24_alt_ellipsoid_SET(int32_t  src, Bounds_Inside * dst)//Altitude (above WGS84, EGM96 ellipsoid), in meters * 1000 (positive for up)
{
    if(dst->base.field_bit != 236)insert_field(dst, 236, 0);
    uint8_t * data = dst->base.pack->data;
    set_bytes((uint32_t)(src), 4, data,  dst->BYTE);
}
INLINER uint32_t  p24_h_acc_TRY(Bounds_Inside * src)
{
    if(src->base.field_bit !=  237 && !try_visit_field(src, 237)) return 0;
    uint8_t * data = src->base.pack->data;
    return ((get_bytes(data,  src->BYTE, 4)));
}
INLINER void p24_h_acc_SET(uint32_t  src, Bounds_Inside * dst)//Position uncertainty in meters * 1000 (positive for up)
{
    if(dst->base.field_bit != 237)insert_field(dst, 237, 0);
    uint8_t * data = dst->base.pack->data;
    set_bytes((src), 4, data,  dst->BYTE);
}
INLINER uint32_t  p24_v_acc_TRY(Bounds_Inside * src)
{
    if(src->base.field_bit !=  238 && !try_visit_field(src, 238)) return 0;
    uint8_t * data = src->base.pack->data;
    return ((get_bytes(data,  src->BYTE, 4)));
}
INLINER void p24_v_acc_SET(uint32_t  src, Bounds_Inside * dst)//Altitude uncertainty in meters * 1000 (positive for up)
{
    if(dst->base.field_bit != 238)insert_field(dst, 238, 0);
    uint8_t * data = dst->base.pack->data;
    set_bytes((src), 4, data,  dst->BYTE);
}
INLINER uint32_t  p24_vel_acc_TRY(Bounds_Inside * src)
{
    if(src->base.field_bit !=  239 && !try_visit_field(src, 239)) return 0;
    uint8_t * data = src->base.pack->data;
    return ((get_bytes(data,  src->BYTE, 4)));
}
INLINER void p24_vel_acc_SET(uint32_t  src, Bounds_Inside * dst)//Speed uncertainty in meters * 1000 (positive for up)
{
    if(dst->base.field_bit != 239)insert_field(dst, 239, 0);
    uint8_t * data = dst->base.pack->data;
    set_bytes((src), 4, data,  dst->BYTE);
}
INLINER uint32_t  p24_hdg_acc_TRY(Bounds_Inside * src)
{
    if(src->base.field_bit !=  240 && !try_visit_field(src, 240)) return 0;
    uint8_t * data = src->base.pack->data;
    return ((get_bytes(data,  src->BYTE, 4)));
}
INLINER void p24_hdg_acc_SET(uint32_t  src, Bounds_Inside * dst)//Heading / track uncertainty in degrees * 1e5
{
    if(dst->base.field_bit != 240)insert_field(dst, 240, 0);
    uint8_t * data = dst->base.pack->data;
    set_bytes((src), 4, data,  dst->BYTE);
}
INLINER uint8_t p25_satellites_visible_GET(Pack * src)//Number of satellites visibl
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 1)));
}
INLINER void p25_satellites_visible_SET(uint8_t  src, Pack * dst)//Number of satellites visibl
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  0);
}
INLINER uint8_t* p25_satellite_prn_GET(Pack * src, uint8_t*  dst, int32_t pos) //Global satellite I
{
    uint8_t * data = src->data;
    for(int32_t BYTE = 1, dst_max = pos + 20; pos < dst_max ; pos++,  BYTE += 1)
        dst[pos] = ((get_bytes(data,  BYTE, 1)));
    return dst;
}

static const  uint32_t p25_satellite_prn_LEN = 20; //return array length

INLINER  uint8_t*  p25_satellite_prn_GET_(Pack * src) {return p25_satellite_prn_GET(src, malloc(20 * sizeof(uint8_t)), 0);}
INLINER void p25_satellite_prn_SET(uint8_t*  src, int32_t pos, Pack * dst) //Global satellite I
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  1, src_max = pos + 20; pos < src_max; pos++, BYTE += 1)
        set_bytes((src[pos]), 1, data,  BYTE);
}
INLINER uint8_t* p25_satellite_used_GET(Pack * src, uint8_t*  dst, int32_t pos) //0: Satellite not used, 1: used for localizatio
{
    uint8_t * data = src->data;
    for(int32_t BYTE = 21, dst_max = pos + 20; pos < dst_max ; pos++,  BYTE += 1)
        dst[pos] = ((get_bytes(data,  BYTE, 1)));
    return dst;
}

static const  uint32_t p25_satellite_used_LEN = 20; //return array length

INLINER  uint8_t*  p25_satellite_used_GET_(Pack * src) {return p25_satellite_used_GET(src, malloc(20 * sizeof(uint8_t)), 0);}
INLINER void p25_satellite_used_SET(uint8_t*  src, int32_t pos, Pack * dst) //0: Satellite not used, 1: used for localizatio
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  21, src_max = pos + 20; pos < src_max; pos++, BYTE += 1)
        set_bytes((src[pos]), 1, data,  BYTE);
}
INLINER uint8_t* p25_satellite_elevation_GET(Pack * src, uint8_t*  dst, int32_t pos) //Elevation (0: right on top of receiver, 90: on the horizon) of satellit
{
    uint8_t * data = src->data;
    for(int32_t BYTE = 41, dst_max = pos + 20; pos < dst_max ; pos++,  BYTE += 1)
        dst[pos] = ((get_bytes(data,  BYTE, 1)));
    return dst;
}

static const  uint32_t p25_satellite_elevation_LEN = 20; //return array length

INLINER  uint8_t*  p25_satellite_elevation_GET_(Pack * src) {return p25_satellite_elevation_GET(src, malloc(20 * sizeof(uint8_t)), 0);}
INLINER void p25_satellite_elevation_SET(uint8_t*  src, int32_t pos, Pack * dst) //Elevation (0: right on top of receiver, 90: on the horizon) of satellit
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  41, src_max = pos + 20; pos < src_max; pos++, BYTE += 1)
        set_bytes((src[pos]), 1, data,  BYTE);
}
INLINER uint8_t* p25_satellite_azimuth_GET(Pack * src, uint8_t*  dst, int32_t pos) //Direction of satellite, 0: 0 deg, 255: 360 deg
{
    uint8_t * data = src->data;
    for(int32_t BYTE = 61, dst_max = pos + 20; pos < dst_max ; pos++,  BYTE += 1)
        dst[pos] = ((get_bytes(data,  BYTE, 1)));
    return dst;
}

static const  uint32_t p25_satellite_azimuth_LEN = 20; //return array length

INLINER  uint8_t*  p25_satellite_azimuth_GET_(Pack * src) {return p25_satellite_azimuth_GET(src, malloc(20 * sizeof(uint8_t)), 0);}
INLINER void p25_satellite_azimuth_SET(uint8_t*  src, int32_t pos, Pack * dst) //Direction of satellite, 0: 0 deg, 255: 360 deg
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  61, src_max = pos + 20; pos < src_max; pos++, BYTE += 1)
        set_bytes((src[pos]), 1, data,  BYTE);
}
INLINER uint8_t* p25_satellite_snr_GET(Pack * src, uint8_t*  dst, int32_t pos) //Signal to noise ratio of satellit
{
    uint8_t * data = src->data;
    for(int32_t BYTE = 81, dst_max = pos + 20; pos < dst_max ; pos++,  BYTE += 1)
        dst[pos] = ((get_bytes(data,  BYTE, 1)));
    return dst;
}

static const  uint32_t p25_satellite_snr_LEN = 20; //return array length

INLINER  uint8_t*  p25_satellite_snr_GET_(Pack * src) {return p25_satellite_snr_GET(src, malloc(20 * sizeof(uint8_t)), 0);}
INLINER void p25_satellite_snr_SET(uint8_t*  src, int32_t pos, Pack * dst) //Signal to noise ratio of satellit
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  81, src_max = pos + 20; pos < src_max; pos++, BYTE += 1)
        set_bytes((src[pos]), 1, data,  BYTE);
}
INLINER uint32_t p26_time_boot_ms_GET(Pack * src)//Timestamp (milliseconds since system boot
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 4)));
}
INLINER void p26_time_boot_ms_SET(uint32_t  src, Pack * dst)//Timestamp (milliseconds since system boot
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  0);
}
INLINER int16_t p26_xacc_GET(Pack * src)//X acceleration (mg
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  4, 2)));
}
INLINER void p26_xacc_SET(int16_t  src, Pack * dst)//X acceleration (mg
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  4);
}
INLINER int16_t p26_yacc_GET(Pack * src)//Y acceleration (mg
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  6, 2)));
}
INLINER void p26_yacc_SET(int16_t  src, Pack * dst)//Y acceleration (mg
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  6);
}
INLINER int16_t p26_zacc_GET(Pack * src)//Z acceleration (mg
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  8, 2)));
}
INLINER void p26_zacc_SET(int16_t  src, Pack * dst)//Z acceleration (mg
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  8);
}
INLINER int16_t p26_xgyro_GET(Pack * src)//Angular speed around X axis (millirad /sec
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  10, 2)));
}
INLINER void p26_xgyro_SET(int16_t  src, Pack * dst)//Angular speed around X axis (millirad /sec
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  10);
}
INLINER int16_t p26_ygyro_GET(Pack * src)//Angular speed around Y axis (millirad /sec
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  12, 2)));
}
INLINER void p26_ygyro_SET(int16_t  src, Pack * dst)//Angular speed around Y axis (millirad /sec
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  12);
}
INLINER int16_t p26_zgyro_GET(Pack * src)//Angular speed around Z axis (millirad /sec
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  14, 2)));
}
INLINER void p26_zgyro_SET(int16_t  src, Pack * dst)//Angular speed around Z axis (millirad /sec
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  14);
}
INLINER int16_t p26_xmag_GET(Pack * src)//X Magnetic field (milli tesla
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  16, 2)));
}
INLINER void p26_xmag_SET(int16_t  src, Pack * dst)//X Magnetic field (milli tesla
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  16);
}
INLINER int16_t p26_ymag_GET(Pack * src)//Y Magnetic field (milli tesla
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  18, 2)));
}
INLINER void p26_ymag_SET(int16_t  src, Pack * dst)//Y Magnetic field (milli tesla
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  18);
}
INLINER int16_t p26_zmag_GET(Pack * src)//Z Magnetic field (milli tesla
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  20, 2)));
}
INLINER void p26_zmag_SET(int16_t  src, Pack * dst)//Z Magnetic field (milli tesla
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  20);
}
INLINER uint64_t p27_time_usec_GET(Pack * src)//Timestamp (microseconds since UNIX epoch or microseconds since system boot
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 8)));
}
INLINER void p27_time_usec_SET(uint64_t  src, Pack * dst)//Timestamp (microseconds since UNIX epoch or microseconds since system boot
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  0);
}
INLINER int16_t p27_xacc_GET(Pack * src)//X acceleration (raw
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  8, 2)));
}
INLINER void p27_xacc_SET(int16_t  src, Pack * dst)//X acceleration (raw
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  8);
}
INLINER int16_t p27_yacc_GET(Pack * src)//Y acceleration (raw
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  10, 2)));
}
INLINER void p27_yacc_SET(int16_t  src, Pack * dst)//Y acceleration (raw
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  10);
}
INLINER int16_t p27_zacc_GET(Pack * src)//Z acceleration (raw
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  12, 2)));
}
INLINER void p27_zacc_SET(int16_t  src, Pack * dst)//Z acceleration (raw
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  12);
}
INLINER int16_t p27_xgyro_GET(Pack * src)//Angular speed around X axis (raw
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  14, 2)));
}
INLINER void p27_xgyro_SET(int16_t  src, Pack * dst)//Angular speed around X axis (raw
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  14);
}
INLINER int16_t p27_ygyro_GET(Pack * src)//Angular speed around Y axis (raw
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  16, 2)));
}
INLINER void p27_ygyro_SET(int16_t  src, Pack * dst)//Angular speed around Y axis (raw
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  16);
}
INLINER int16_t p27_zgyro_GET(Pack * src)//Angular speed around Z axis (raw
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  18, 2)));
}
INLINER void p27_zgyro_SET(int16_t  src, Pack * dst)//Angular speed around Z axis (raw
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  18);
}
INLINER int16_t p27_xmag_GET(Pack * src)//X Magnetic field (raw
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  20, 2)));
}
INLINER void p27_xmag_SET(int16_t  src, Pack * dst)//X Magnetic field (raw
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  20);
}
INLINER int16_t p27_ymag_GET(Pack * src)//Y Magnetic field (raw
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  22, 2)));
}
INLINER void p27_ymag_SET(int16_t  src, Pack * dst)//Y Magnetic field (raw
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  22);
}
INLINER int16_t p27_zmag_GET(Pack * src)//Z Magnetic field (raw
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  24, 2)));
}
INLINER void p27_zmag_SET(int16_t  src, Pack * dst)//Z Magnetic field (raw
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  24);
}
INLINER uint64_t p28_time_usec_GET(Pack * src)//Timestamp (microseconds since UNIX epoch or microseconds since system boot
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 8)));
}
INLINER void p28_time_usec_SET(uint64_t  src, Pack * dst)//Timestamp (microseconds since UNIX epoch or microseconds since system boot
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  0);
}
INLINER int16_t p28_press_abs_GET(Pack * src)//Absolute pressure (raw
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  8, 2)));
}
INLINER void p28_press_abs_SET(int16_t  src, Pack * dst)//Absolute pressure (raw
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  8);
}
INLINER int16_t p28_press_diff1_GET(Pack * src)//Differential pressure 1 (raw, 0 if nonexistant
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  10, 2)));
}
INLINER void p28_press_diff1_SET(int16_t  src, Pack * dst)//Differential pressure 1 (raw, 0 if nonexistant
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  10);
}
INLINER int16_t p28_press_diff2_GET(Pack * src)//Differential pressure 2 (raw, 0 if nonexistant
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  12, 2)));
}
INLINER void p28_press_diff2_SET(int16_t  src, Pack * dst)//Differential pressure 2 (raw, 0 if nonexistant
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  12);
}
INLINER int16_t p28_temperature_GET(Pack * src)//Raw Temperature measurement (raw
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  14, 2)));
}
INLINER void p28_temperature_SET(int16_t  src, Pack * dst)//Raw Temperature measurement (raw
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  14);
}
INLINER uint32_t p29_time_boot_ms_GET(Pack * src)//Timestamp (milliseconds since system boot
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 4)));
}
INLINER void p29_time_boot_ms_SET(uint32_t  src, Pack * dst)//Timestamp (milliseconds since system boot
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  0);
}
INLINER float p29_press_abs_GET(Pack * src)//Absolute pressure (hectopascal
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  4, 4)));
}
INLINER void p29_press_abs_SET(float  src, Pack * dst)//Absolute pressure (hectopascal
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  4);
}
INLINER float p29_press_diff_GET(Pack * src)//Differential pressure 1 (hectopascal
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  8, 4)));
}
INLINER void p29_press_diff_SET(float  src, Pack * dst)//Differential pressure 1 (hectopascal
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  8);
}
INLINER int16_t p29_temperature_GET(Pack * src)//Temperature measurement (0.01 degrees celsius
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  12, 2)));
}
INLINER void p29_temperature_SET(int16_t  src, Pack * dst)//Temperature measurement (0.01 degrees celsius
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  12);
}
INLINER uint32_t p30_time_boot_ms_GET(Pack * src)//Timestamp (milliseconds since system boot
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 4)));
}
INLINER void p30_time_boot_ms_SET(uint32_t  src, Pack * dst)//Timestamp (milliseconds since system boot
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  0);
}
INLINER float p30_roll_GET(Pack * src)//Roll angle (rad, -pi..+pi
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  4, 4)));
}
INLINER void p30_roll_SET(float  src, Pack * dst)//Roll angle (rad, -pi..+pi
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  4);
}
INLINER float p30_pitch_GET(Pack * src)//Pitch angle (rad, -pi..+pi
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  8, 4)));
}
INLINER void p30_pitch_SET(float  src, Pack * dst)//Pitch angle (rad, -pi..+pi
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  8);
}
INLINER float p30_yaw_GET(Pack * src)//Yaw angle (rad, -pi..+pi
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  12, 4)));
}
INLINER void p30_yaw_SET(float  src, Pack * dst)//Yaw angle (rad, -pi..+pi
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  12);
}
INLINER float p30_rollspeed_GET(Pack * src)//Roll angular speed (rad/s
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  16, 4)));
}
INLINER void p30_rollspeed_SET(float  src, Pack * dst)//Roll angular speed (rad/s
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  16);
}
INLINER float p30_pitchspeed_GET(Pack * src)//Pitch angular speed (rad/s
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  20, 4)));
}
INLINER void p30_pitchspeed_SET(float  src, Pack * dst)//Pitch angular speed (rad/s
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  20);
}
INLINER float p30_yawspeed_GET(Pack * src)//Yaw angular speed (rad/s
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  24, 4)));
}
INLINER void p30_yawspeed_SET(float  src, Pack * dst)//Yaw angular speed (rad/s
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  24);
}
INLINER uint32_t p31_time_boot_ms_GET(Pack * src)//Timestamp (milliseconds since system boot
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 4)));
}
INLINER void p31_time_boot_ms_SET(uint32_t  src, Pack * dst)//Timestamp (milliseconds since system boot
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  0);
}
INLINER float p31_q1_GET(Pack * src)//Quaternion component 1, w (1 in null-rotation
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  4, 4)));
}
INLINER void p31_q1_SET(float  src, Pack * dst)//Quaternion component 1, w (1 in null-rotation
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  4);
}
INLINER float p31_q2_GET(Pack * src)//Quaternion component 2, x (0 in null-rotation
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  8, 4)));
}
INLINER void p31_q2_SET(float  src, Pack * dst)//Quaternion component 2, x (0 in null-rotation
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  8);
}
INLINER float p31_q3_GET(Pack * src)//Quaternion component 3, y (0 in null-rotation
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  12, 4)));
}
INLINER void p31_q3_SET(float  src, Pack * dst)//Quaternion component 3, y (0 in null-rotation
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  12);
}
INLINER float p31_q4_GET(Pack * src)//Quaternion component 4, z (0 in null-rotation
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  16, 4)));
}
INLINER void p31_q4_SET(float  src, Pack * dst)//Quaternion component 4, z (0 in null-rotation
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  16);
}
INLINER float p31_rollspeed_GET(Pack * src)//Roll angular speed (rad/s
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  20, 4)));
}
INLINER void p31_rollspeed_SET(float  src, Pack * dst)//Roll angular speed (rad/s
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  20);
}
INLINER float p31_pitchspeed_GET(Pack * src)//Pitch angular speed (rad/s
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  24, 4)));
}
INLINER void p31_pitchspeed_SET(float  src, Pack * dst)//Pitch angular speed (rad/s
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  24);
}
INLINER float p31_yawspeed_GET(Pack * src)//Yaw angular speed (rad/s
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  28, 4)));
}
INLINER void p31_yawspeed_SET(float  src, Pack * dst)//Yaw angular speed (rad/s
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  28);
}
INLINER uint32_t p32_time_boot_ms_GET(Pack * src)//Timestamp (milliseconds since system boot
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 4)));
}
INLINER void p32_time_boot_ms_SET(uint32_t  src, Pack * dst)//Timestamp (milliseconds since system boot
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  0);
}
INLINER float p32_x_GET(Pack * src)//X Positio
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  4, 4)));
}
INLINER void p32_x_SET(float  src, Pack * dst)//X Positio
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  4);
}
INLINER float p32_y_GET(Pack * src)//Y Positio
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  8, 4)));
}
INLINER void p32_y_SET(float  src, Pack * dst)//Y Positio
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  8);
}
INLINER float p32_z_GET(Pack * src)//Z Positio
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  12, 4)));
}
INLINER void p32_z_SET(float  src, Pack * dst)//Z Positio
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  12);
}
INLINER float p32_vx_GET(Pack * src)//X Spee
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  16, 4)));
}
INLINER void p32_vx_SET(float  src, Pack * dst)//X Spee
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  16);
}
INLINER float p32_vy_GET(Pack * src)//Y Spee
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  20, 4)));
}
INLINER void p32_vy_SET(float  src, Pack * dst)//Y Spee
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  20);
}
INLINER float p32_vz_GET(Pack * src)//Z Spee
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  24, 4)));
}
INLINER void p32_vz_SET(float  src, Pack * dst)//Z Spee
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  24);
}
INLINER uint16_t p33_hdg_GET(Pack * src)//Vehicle heading (yaw angle) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MA
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER void p33_hdg_SET(uint16_t  src, Pack * dst)//Vehicle heading (yaw angle) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MA
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER uint32_t p33_time_boot_ms_GET(Pack * src)//Timestamp (milliseconds since system boot
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 4)));
}
INLINER void p33_time_boot_ms_SET(uint32_t  src, Pack * dst)//Timestamp (milliseconds since system boot
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  2);
}
INLINER int32_t p33_lat_GET(Pack * src)//Latitude, expressed as degrees * 1E
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  6, 4)));
}
INLINER void p33_lat_SET(int32_t  src, Pack * dst)//Latitude, expressed as degrees * 1E
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  6);
}
INLINER int32_t p33_lon_GET(Pack * src)//Longitude, expressed as degrees * 1E
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  10, 4)));
}
INLINER void p33_lon_SET(int32_t  src, Pack * dst)//Longitude, expressed as degrees * 1E
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  10);
}
INLINER int32_t p33_alt_GET(Pack * src)
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  14, 4)));
}
INLINER void p33_alt_SET(int32_t  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  14);
}
INLINER int32_t p33_relative_alt_GET(Pack * src)//Altitude above ground in meters, expressed as * 1000 (millimeters
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  18, 4)));
}
INLINER void p33_relative_alt_SET(int32_t  src, Pack * dst)//Altitude above ground in meters, expressed as * 1000 (millimeters
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  18);
}
INLINER int16_t p33_vx_GET(Pack * src)//Ground X Speed (Latitude, positive north), expressed as m/s * 10
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  22, 2)));
}
INLINER void p33_vx_SET(int16_t  src, Pack * dst)//Ground X Speed (Latitude, positive north), expressed as m/s * 10
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  22);
}
INLINER int16_t p33_vy_GET(Pack * src)//Ground Y Speed (Longitude, positive east), expressed as m/s * 10
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  24, 2)));
}
INLINER void p33_vy_SET(int16_t  src, Pack * dst)//Ground Y Speed (Longitude, positive east), expressed as m/s * 10
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  24);
}
INLINER int16_t p33_vz_GET(Pack * src)//Ground Z Speed (Altitude, positive down), expressed as m/s * 10
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  26, 2)));
}
INLINER void p33_vz_SET(int16_t  src, Pack * dst)//Ground Z Speed (Altitude, positive down), expressed as m/s * 10
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  26);
}
INLINER uint32_t p34_time_boot_ms_GET(Pack * src)//Timestamp (milliseconds since system boot
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 4)));
}
INLINER void p34_time_boot_ms_SET(uint32_t  src, Pack * dst)//Timestamp (milliseconds since system boot
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  0);
}
INLINER uint8_t p34_port_GET(Pack * src)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  4, 1)));
}
INLINER void p34_port_SET(uint8_t  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  4);
}
INLINER int16_t p34_chan1_scaled_GET(Pack * src)//RC channel 1 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  5, 2)));
}
INLINER void p34_chan1_scaled_SET(int16_t  src, Pack * dst)//RC channel 1 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  5);
}
INLINER int16_t p34_chan2_scaled_GET(Pack * src)//RC channel 2 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  7, 2)));
}
INLINER void p34_chan2_scaled_SET(int16_t  src, Pack * dst)//RC channel 2 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  7);
}
INLINER int16_t p34_chan3_scaled_GET(Pack * src)//RC channel 3 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  9, 2)));
}
INLINER void p34_chan3_scaled_SET(int16_t  src, Pack * dst)//RC channel 3 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  9);
}
INLINER int16_t p34_chan4_scaled_GET(Pack * src)//RC channel 4 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  11, 2)));
}
INLINER void p34_chan4_scaled_SET(int16_t  src, Pack * dst)//RC channel 4 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  11);
}
INLINER int16_t p34_chan5_scaled_GET(Pack * src)//RC channel 5 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  13, 2)));
}
INLINER void p34_chan5_scaled_SET(int16_t  src, Pack * dst)//RC channel 5 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  13);
}
INLINER int16_t p34_chan6_scaled_GET(Pack * src)//RC channel 6 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  15, 2)));
}
INLINER void p34_chan6_scaled_SET(int16_t  src, Pack * dst)//RC channel 6 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  15);
}
INLINER int16_t p34_chan7_scaled_GET(Pack * src)//RC channel 7 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  17, 2)));
}
INLINER void p34_chan7_scaled_SET(int16_t  src, Pack * dst)//RC channel 7 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  17);
}
INLINER int16_t p34_chan8_scaled_GET(Pack * src)//RC channel 8 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  19, 2)));
}
INLINER void p34_chan8_scaled_SET(int16_t  src, Pack * dst)//RC channel 8 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  19);
}
INLINER uint8_t p34_rssi_GET(Pack * src)//Receive signal strength indicator, 0: 0%, 100: 100%, 255: invalid/unknown
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  21, 1)));
}
INLINER void p34_rssi_SET(uint8_t  src, Pack * dst)//Receive signal strength indicator, 0: 0%, 100: 100%, 255: invalid/unknown
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  21);
}
INLINER uint16_t p35_chan1_raw_GET(Pack * src)//RC channel 1 value, in microseconds. A value of UINT16_MAX implies the channel is unused
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER void p35_chan1_raw_SET(uint16_t  src, Pack * dst)//RC channel 1 value, in microseconds. A value of UINT16_MAX implies the channel is unused
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER uint16_t p35_chan2_raw_GET(Pack * src)//RC channel 2 value, in microseconds. A value of UINT16_MAX implies the channel is unused
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 2)));
}
INLINER void p35_chan2_raw_SET(uint16_t  src, Pack * dst)//RC channel 2 value, in microseconds. A value of UINT16_MAX implies the channel is unused
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  2);
}
INLINER uint16_t p35_chan3_raw_GET(Pack * src)//RC channel 3 value, in microseconds. A value of UINT16_MAX implies the channel is unused
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  4, 2)));
}
INLINER void p35_chan3_raw_SET(uint16_t  src, Pack * dst)//RC channel 3 value, in microseconds. A value of UINT16_MAX implies the channel is unused
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  4);
}
INLINER uint16_t p35_chan4_raw_GET(Pack * src)//RC channel 4 value, in microseconds. A value of UINT16_MAX implies the channel is unused
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  6, 2)));
}
INLINER void p35_chan4_raw_SET(uint16_t  src, Pack * dst)//RC channel 4 value, in microseconds. A value of UINT16_MAX implies the channel is unused
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  6);
}
INLINER uint16_t p35_chan5_raw_GET(Pack * src)//RC channel 5 value, in microseconds. A value of UINT16_MAX implies the channel is unused
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  8, 2)));
}
INLINER void p35_chan5_raw_SET(uint16_t  src, Pack * dst)//RC channel 5 value, in microseconds. A value of UINT16_MAX implies the channel is unused
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  8);
}
INLINER uint16_t p35_chan6_raw_GET(Pack * src)//RC channel 6 value, in microseconds. A value of UINT16_MAX implies the channel is unused
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  10, 2)));
}
INLINER void p35_chan6_raw_SET(uint16_t  src, Pack * dst)//RC channel 6 value, in microseconds. A value of UINT16_MAX implies the channel is unused
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  10);
}
INLINER uint16_t p35_chan7_raw_GET(Pack * src)//RC channel 7 value, in microseconds. A value of UINT16_MAX implies the channel is unused
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  12, 2)));
}
INLINER void p35_chan7_raw_SET(uint16_t  src, Pack * dst)//RC channel 7 value, in microseconds. A value of UINT16_MAX implies the channel is unused
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  12);
}
INLINER uint16_t p35_chan8_raw_GET(Pack * src)//RC channel 8 value, in microseconds. A value of UINT16_MAX implies the channel is unused
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  14, 2)));
}
INLINER void p35_chan8_raw_SET(uint16_t  src, Pack * dst)//RC channel 8 value, in microseconds. A value of UINT16_MAX implies the channel is unused
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  14);
}
INLINER uint32_t p35_time_boot_ms_GET(Pack * src)//Timestamp (milliseconds since system boot
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  16, 4)));
}
INLINER void p35_time_boot_ms_SET(uint32_t  src, Pack * dst)//Timestamp (milliseconds since system boot
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  16);
}
INLINER uint8_t p35_port_GET(Pack * src)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  20, 1)));
}
INLINER void p35_port_SET(uint8_t  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  20);
}
INLINER uint8_t p35_rssi_GET(Pack * src)//Receive signal strength indicator, 0: 0%, 100: 100%, 255: invalid/unknown
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  21, 1)));
}
INLINER void p35_rssi_SET(uint8_t  src, Pack * dst)//Receive signal strength indicator, 0: 0%, 100: 100%, 255: invalid/unknown
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  21);
}
INLINER uint16_t p36_servo1_raw_GET(Pack * src)//Servo output 1 value, in microsecond
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER void p36_servo1_raw_SET(uint16_t  src, Pack * dst)//Servo output 1 value, in microsecond
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER uint16_t p36_servo2_raw_GET(Pack * src)//Servo output 2 value, in microsecond
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 2)));
}
INLINER void p36_servo2_raw_SET(uint16_t  src, Pack * dst)//Servo output 2 value, in microsecond
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  2);
}
INLINER uint16_t p36_servo3_raw_GET(Pack * src)//Servo output 3 value, in microsecond
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  4, 2)));
}
INLINER void p36_servo3_raw_SET(uint16_t  src, Pack * dst)//Servo output 3 value, in microsecond
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  4);
}
INLINER uint16_t p36_servo4_raw_GET(Pack * src)//Servo output 4 value, in microsecond
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  6, 2)));
}
INLINER void p36_servo4_raw_SET(uint16_t  src, Pack * dst)//Servo output 4 value, in microsecond
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  6);
}
INLINER uint16_t p36_servo5_raw_GET(Pack * src)//Servo output 5 value, in microsecond
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  8, 2)));
}
INLINER void p36_servo5_raw_SET(uint16_t  src, Pack * dst)//Servo output 5 value, in microsecond
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  8);
}
INLINER uint16_t p36_servo6_raw_GET(Pack * src)//Servo output 6 value, in microsecond
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  10, 2)));
}
INLINER void p36_servo6_raw_SET(uint16_t  src, Pack * dst)//Servo output 6 value, in microsecond
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  10);
}
INLINER uint16_t p36_servo7_raw_GET(Pack * src)//Servo output 7 value, in microsecond
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  12, 2)));
}
INLINER void p36_servo7_raw_SET(uint16_t  src, Pack * dst)//Servo output 7 value, in microsecond
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  12);
}
INLINER uint16_t p36_servo8_raw_GET(Pack * src)//Servo output 8 value, in microsecond
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  14, 2)));
}
INLINER void p36_servo8_raw_SET(uint16_t  src, Pack * dst)//Servo output 8 value, in microsecond
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  14);
}
INLINER uint32_t p36_time_usec_GET(Pack * src)//Timestamp (microseconds since system boot
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  16, 4)));
}
INLINER void p36_time_usec_SET(uint32_t  src, Pack * dst)//Timestamp (microseconds since system boot
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  16);
}
INLINER uint8_t p36_port_GET(Pack * src)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  20, 1)));
}
INLINER void p36_port_SET(uint8_t  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  20);
}
INLINER uint16_t  p36_servo9_raw_TRY(Bounds_Inside * src)
{
    if(src->base.field_bit !=  168 && !try_visit_field(src, 168)) return 0;
    uint8_t * data = src->base.pack->data;
    return ((get_bytes(data,  src->BYTE, 2)));
}
INLINER void p36_servo9_raw_SET(uint16_t  src, Bounds_Inside * dst)//Servo output 9 value, in microsecond
{
    if(dst->base.field_bit != 168)insert_field(dst, 168, 0);
    uint8_t * data = dst->base.pack->data;
    set_bytes((src), 2, data,  dst->BYTE);
}
INLINER uint16_t  p36_servo10_raw_TRY(Bounds_Inside * src)
{
    if(src->base.field_bit !=  169 && !try_visit_field(src, 169)) return 0;
    uint8_t * data = src->base.pack->data;
    return ((get_bytes(data,  src->BYTE, 2)));
}
INLINER void p36_servo10_raw_SET(uint16_t  src, Bounds_Inside * dst)//Servo output 10 value, in microsecond
{
    if(dst->base.field_bit != 169)insert_field(dst, 169, 0);
    uint8_t * data = dst->base.pack->data;
    set_bytes((src), 2, data,  dst->BYTE);
}
INLINER uint16_t  p36_servo11_raw_TRY(Bounds_Inside * src)
{
    if(src->base.field_bit !=  170 && !try_visit_field(src, 170)) return 0;
    uint8_t * data = src->base.pack->data;
    return ((get_bytes(data,  src->BYTE, 2)));
}
INLINER void p36_servo11_raw_SET(uint16_t  src, Bounds_Inside * dst)//Servo output 11 value, in microsecond
{
    if(dst->base.field_bit != 170)insert_field(dst, 170, 0);
    uint8_t * data = dst->base.pack->data;
    set_bytes((src), 2, data,  dst->BYTE);
}
INLINER uint16_t  p36_servo12_raw_TRY(Bounds_Inside * src)
{
    if(src->base.field_bit !=  171 && !try_visit_field(src, 171)) return 0;
    uint8_t * data = src->base.pack->data;
    return ((get_bytes(data,  src->BYTE, 2)));
}
INLINER void p36_servo12_raw_SET(uint16_t  src, Bounds_Inside * dst)//Servo output 12 value, in microsecond
{
    if(dst->base.field_bit != 171)insert_field(dst, 171, 0);
    uint8_t * data = dst->base.pack->data;
    set_bytes((src), 2, data,  dst->BYTE);
}
INLINER uint16_t  p36_servo13_raw_TRY(Bounds_Inside * src)
{
    if(src->base.field_bit !=  172 && !try_visit_field(src, 172)) return 0;
    uint8_t * data = src->base.pack->data;
    return ((get_bytes(data,  src->BYTE, 2)));
}
INLINER void p36_servo13_raw_SET(uint16_t  src, Bounds_Inside * dst)//Servo output 13 value, in microsecond
{
    if(dst->base.field_bit != 172)insert_field(dst, 172, 0);
    uint8_t * data = dst->base.pack->data;
    set_bytes((src), 2, data,  dst->BYTE);
}
INLINER uint16_t  p36_servo14_raw_TRY(Bounds_Inside * src)
{
    if(src->base.field_bit !=  173 && !try_visit_field(src, 173)) return 0;
    uint8_t * data = src->base.pack->data;
    return ((get_bytes(data,  src->BYTE, 2)));
}
INLINER void p36_servo14_raw_SET(uint16_t  src, Bounds_Inside * dst)//Servo output 14 value, in microsecond
{
    if(dst->base.field_bit != 173)insert_field(dst, 173, 0);
    uint8_t * data = dst->base.pack->data;
    set_bytes((src), 2, data,  dst->BYTE);
}
INLINER uint16_t  p36_servo15_raw_TRY(Bounds_Inside * src)
{
    if(src->base.field_bit !=  174 && !try_visit_field(src, 174)) return 0;
    uint8_t * data = src->base.pack->data;
    return ((get_bytes(data,  src->BYTE, 2)));
}
INLINER void p36_servo15_raw_SET(uint16_t  src, Bounds_Inside * dst)//Servo output 15 value, in microsecond
{
    if(dst->base.field_bit != 174)insert_field(dst, 174, 0);
    uint8_t * data = dst->base.pack->data;
    set_bytes((src), 2, data,  dst->BYTE);
}
INLINER uint16_t  p36_servo16_raw_TRY(Bounds_Inside * src)
{
    if(src->base.field_bit !=  175 && !try_visit_field(src, 175)) return 0;
    uint8_t * data = src->base.pack->data;
    return ((get_bytes(data,  src->BYTE, 2)));
}
INLINER void p36_servo16_raw_SET(uint16_t  src, Bounds_Inside * dst)//Servo output 16 value, in microsecond
{
    if(dst->base.field_bit != 175)insert_field(dst, 175, 0);
    uint8_t * data = dst->base.pack->data;
    set_bytes((src), 2, data,  dst->BYTE);
}
INLINER uint8_t p37_target_system_GET(Pack * src)//System I
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 1)));
}
INLINER void p37_target_system_SET(uint8_t  src, Pack * dst)//System I
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  0);
}
INLINER uint8_t p37_target_component_GET(Pack * src)//Component I
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  1, 1)));
}
INLINER void p37_target_component_SET(uint8_t  src, Pack * dst)//Component I
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  1);
}
INLINER int16_t p37_start_index_GET(Pack * src)//Start index, 0 by defaul
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  2, 2)));
}
INLINER void p37_start_index_SET(int16_t  src, Pack * dst)//Start index, 0 by defaul
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  2);
}
INLINER int16_t p37_end_index_GET(Pack * src)//End index, -1 by default (-1: send list to end). Else a valid index of the lis
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  4, 2)));
}
INLINER void p37_end_index_SET(int16_t  src, Pack * dst)//End index, -1 by default (-1: send list to end). Else a valid index of the lis
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  4);
}
INLINER e_MAV_MISSION_TYPE p37_mission_type_GET(Pack * src)//Mission type, see MAV_MISSION_TYP
{
    uint8_t * data = src->data;
    switch(get_bits(data, 48, 3))
    {
        case 0:
            return e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION;
        case 1:
            return e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE;
        case 2:
            return e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY;
        case 3:
            return e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL;
        default: ;//assert(false);//("Unknown enum ID " + id);
    }
}
INLINER void p37_mission_type_SET(e_MAV_MISSION_TYPE  src, Pack * dst)//Mission type, see MAV_MISSION_TYP
{
    uint8_t * data = dst->data;
    int32_t id;
    switch(src)
    {
        case e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION:
            id = 0;
            break;
        case e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE:
            id = 1;
            break;
        case e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY:
            id = 2;
            break;
        case e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL:
            id = 3;
            break;
        default: ;// assert(false);//("Unknown enum" + id);
    }
    set_bits(id, 3, data, 48);
}
INLINER uint8_t p38_target_system_GET(Pack * src)//System I
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 1)));
}
INLINER void p38_target_system_SET(uint8_t  src, Pack * dst)//System I
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  0);
}
INLINER uint8_t p38_target_component_GET(Pack * src)//Component I
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  1, 1)));
}
INLINER void p38_target_component_SET(uint8_t  src, Pack * dst)//Component I
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  1);
}
INLINER int16_t p38_start_index_GET(Pack * src)//Start index, 0 by default and smaller / equal to the largest index of the current onboard list
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  2, 2)));
}
INLINER void p38_start_index_SET(int16_t  src, Pack * dst)//Start index, 0 by default and smaller / equal to the largest index of the current onboard list
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  2);
}
INLINER int16_t p38_end_index_GET(Pack * src)//End index, equal or greater than start index
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  4, 2)));
}
INLINER void p38_end_index_SET(int16_t  src, Pack * dst)//End index, equal or greater than start index
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  4);
}
INLINER e_MAV_MISSION_TYPE p38_mission_type_GET(Pack * src)//Mission type, see MAV_MISSION_TYP
{
    uint8_t * data = src->data;
    switch(get_bits(data, 48, 3))
    {
        case 0:
            return e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION;
        case 1:
            return e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE;
        case 2:
            return e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY;
        case 3:
            return e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL;
        default: ;//assert(false);//("Unknown enum ID " + id);
    }
}
INLINER void p38_mission_type_SET(e_MAV_MISSION_TYPE  src, Pack * dst)//Mission type, see MAV_MISSION_TYP
{
    uint8_t * data = dst->data;
    int32_t id;
    switch(src)
    {
        case e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION:
            id = 0;
            break;
        case e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE:
            id = 1;
            break;
        case e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY:
            id = 2;
            break;
        case e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL:
            id = 3;
            break;
        default: ;// assert(false);//("Unknown enum" + id);
    }
    set_bits(id, 3, data, 48);
}
INLINER uint16_t p39_seq_GET(Pack * src)//Sequenc
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER void p39_seq_SET(uint16_t  src, Pack * dst)//Sequenc
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER uint8_t p39_target_system_GET(Pack * src)//System I
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 1)));
}
INLINER void p39_target_system_SET(uint8_t  src, Pack * dst)//System I
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  2);
}
INLINER uint8_t p39_target_component_GET(Pack * src)//Component I
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  3, 1)));
}
INLINER void p39_target_component_SET(uint8_t  src, Pack * dst)//Component I
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  3);
}
INLINER uint8_t p39_current_GET(Pack * src)//false:0, true:
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  4, 1)));
}
INLINER void p39_current_SET(uint8_t  src, Pack * dst)//false:0, true:
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  4);
}
INLINER uint8_t p39_autocontinue_GET(Pack * src)//autocontinue to next w
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  5, 1)));
}
INLINER void p39_autocontinue_SET(uint8_t  src, Pack * dst)//autocontinue to next w
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  5);
}
INLINER float p39_param1_GET(Pack * src)//PARAM1, see MAV_CMD enu
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  6, 4)));
}
INLINER void p39_param1_SET(float  src, Pack * dst)//PARAM1, see MAV_CMD enu
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  6);
}
INLINER float p39_param2_GET(Pack * src)//PARAM2, see MAV_CMD enu
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  10, 4)));
}
INLINER void p39_param2_SET(float  src, Pack * dst)//PARAM2, see MAV_CMD enu
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  10);
}
INLINER float p39_param3_GET(Pack * src)//PARAM3, see MAV_CMD enu
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  14, 4)));
}
INLINER void p39_param3_SET(float  src, Pack * dst)//PARAM3, see MAV_CMD enu
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  14);
}
INLINER float p39_param4_GET(Pack * src)//PARAM4, see MAV_CMD enu
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  18, 4)));
}
INLINER void p39_param4_SET(float  src, Pack * dst)//PARAM4, see MAV_CMD enu
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  18);
}
INLINER float p39_x_GET(Pack * src)//PARAM5 / local: x position, global: latitud
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  22, 4)));
}
INLINER void p39_x_SET(float  src, Pack * dst)//PARAM5 / local: x position, global: latitud
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  22);
}
INLINER float p39_y_GET(Pack * src)//PARAM6 / y position: global: longitud
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  26, 4)));
}
INLINER void p39_y_SET(float  src, Pack * dst)//PARAM6 / y position: global: longitud
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  26);
}
INLINER float p39_z_GET(Pack * src)//PARAM7 / z position: global: altitude (relative or absolute, depending on frame
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  30, 4)));
}
INLINER void p39_z_SET(float  src, Pack * dst)//PARAM7 / z position: global: altitude (relative or absolute, depending on frame
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  30);
}
INLINER e_MAV_FRAME p39_frame_GET(Pack * src)//The coordinate system of the waypoint. see MAV_FRAME in mavlink_types.
{
    uint8_t * data = src->data;
    return  0 + (int)get_bits(data, 272, 4);
}
INLINER void p39_frame_SET(e_MAV_FRAME  src, Pack * dst)//The coordinate system of the waypoint. see MAV_FRAME in mavlink_types.
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 4, data, 272);
}
INLINER e_MAV_CMD p39_command_GET(Pack * src)//The scheduled action for the waypoint. see MAV_CMD in common.xml MAVLink spec
{
    uint8_t * data = src->data;
    switch(get_bits(data, 276, 8))
    {
        case 0:
            return e_MAV_CMD_MAV_CMD_NAV_WAYPOINT;
        case 1:
            return e_MAV_CMD_MAV_CMD_NAV_LOITER_UNLIM;
        case 2:
            return e_MAV_CMD_MAV_CMD_NAV_LOITER_TURNS;
        case 3:
            return e_MAV_CMD_MAV_CMD_NAV_LOITER_TIME;
        case 4:
            return e_MAV_CMD_MAV_CMD_NAV_RETURN_TO_LAUNCH;
        case 5:
            return e_MAV_CMD_MAV_CMD_NAV_LAND;
        case 6:
            return e_MAV_CMD_MAV_CMD_NAV_TAKEOFF;
        case 7:
            return e_MAV_CMD_MAV_CMD_NAV_LAND_LOCAL;
        case 8:
            return e_MAV_CMD_MAV_CMD_NAV_TAKEOFF_LOCAL;
        case 9:
            return e_MAV_CMD_MAV_CMD_NAV_FOLLOW;
        case 10:
            return e_MAV_CMD_MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT;
        case 11:
            return e_MAV_CMD_MAV_CMD_NAV_LOITER_TO_ALT;
        case 12:
            return e_MAV_CMD_MAV_CMD_DO_FOLLOW;
        case 13:
            return e_MAV_CMD_MAV_CMD_DO_FOLLOW_REPOSITION;
        case 14:
            return e_MAV_CMD_MAV_CMD_NAV_ROI;
        case 15:
            return e_MAV_CMD_MAV_CMD_NAV_PATHPLANNING;
        case 16:
            return e_MAV_CMD_MAV_CMD_NAV_SPLINE_WAYPOINT;
        case 17:
            return e_MAV_CMD_MAV_CMD_NAV_VTOL_TAKEOFF;
        case 18:
            return e_MAV_CMD_MAV_CMD_NAV_VTOL_LAND;
        case 19:
            return e_MAV_CMD_MAV_CMD_NAV_GUIDED_ENABLE;
        case 20:
            return e_MAV_CMD_MAV_CMD_NAV_DELAY;
        case 21:
            return e_MAV_CMD_MAV_CMD_NAV_PAYLOAD_PLACE;
        case 22:
            return e_MAV_CMD_MAV_CMD_NAV_LAST;
        case 23:
            return e_MAV_CMD_MAV_CMD_CONDITION_DELAY;
        case 24:
            return e_MAV_CMD_MAV_CMD_CONDITION_CHANGE_ALT;
        case 25:
            return e_MAV_CMD_MAV_CMD_CONDITION_DISTANCE;
        case 26:
            return e_MAV_CMD_MAV_CMD_CONDITION_YAW;
        case 27:
            return e_MAV_CMD_MAV_CMD_CONDITION_LAST;
        case 28:
            return e_MAV_CMD_MAV_CMD_DO_SET_MODE;
        case 29:
            return e_MAV_CMD_MAV_CMD_DO_JUMP;
        case 30:
            return e_MAV_CMD_MAV_CMD_DO_CHANGE_SPEED;
        case 31:
            return e_MAV_CMD_MAV_CMD_DO_SET_HOME;
        case 32:
            return e_MAV_CMD_MAV_CMD_DO_SET_PARAMETER;
        case 33:
            return e_MAV_CMD_MAV_CMD_DO_SET_RELAY;
        case 34:
            return e_MAV_CMD_MAV_CMD_DO_REPEAT_RELAY;
        case 35:
            return e_MAV_CMD_MAV_CMD_DO_SET_SERVO;
        case 36:
            return e_MAV_CMD_MAV_CMD_DO_REPEAT_SERVO;
        case 37:
            return e_MAV_CMD_MAV_CMD_DO_FLIGHTTERMINATION;
        case 38:
            return e_MAV_CMD_MAV_CMD_DO_CHANGE_ALTITUDE;
        case 39:
            return e_MAV_CMD_MAV_CMD_DO_LAND_START;
        case 40:
            return e_MAV_CMD_MAV_CMD_DO_RALLY_LAND;
        case 41:
            return e_MAV_CMD_MAV_CMD_DO_GO_AROUND;
        case 42:
            return e_MAV_CMD_MAV_CMD_DO_REPOSITION;
        case 43:
            return e_MAV_CMD_MAV_CMD_DO_PAUSE_CONTINUE;
        case 44:
            return e_MAV_CMD_MAV_CMD_DO_SET_REVERSE;
        case 45:
            return e_MAV_CMD_MAV_CMD_DO_CONTROL_VIDEO;
        case 46:
            return e_MAV_CMD_MAV_CMD_DO_SET_ROI;
        case 47:
            return e_MAV_CMD_MAV_CMD_DO_DIGICAM_CONFIGURE;
        case 48:
            return e_MAV_CMD_MAV_CMD_DO_DIGICAM_CONTROL;
        case 49:
            return e_MAV_CMD_MAV_CMD_DO_MOUNT_CONFIGURE;
        case 50:
            return e_MAV_CMD_MAV_CMD_DO_MOUNT_CONTROL;
        case 51:
            return e_MAV_CMD_MAV_CMD_DO_SET_CAM_TRIGG_DIST;
        case 52:
            return e_MAV_CMD_MAV_CMD_DO_FENCE_ENABLE;
        case 53:
            return e_MAV_CMD_MAV_CMD_DO_PARACHUTE;
        case 54:
            return e_MAV_CMD_MAV_CMD_DO_MOTOR_TEST;
        case 55:
            return e_MAV_CMD_MAV_CMD_DO_INVERTED_FLIGHT;
        case 56:
            return e_MAV_CMD_MAV_CMD_NAV_SET_YAW_SPEED;
        case 57:
            return e_MAV_CMD_MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL;
        case 58:
            return e_MAV_CMD_MAV_CMD_DO_MOUNT_CONTROL_QUAT;
        case 59:
            return e_MAV_CMD_MAV_CMD_DO_GUIDED_MASTER;
        case 60:
            return e_MAV_CMD_MAV_CMD_DO_GUIDED_LIMITS;
        case 61:
            return e_MAV_CMD_MAV_CMD_DO_ENGINE_CONTROL;
        case 62:
            return e_MAV_CMD_MAV_CMD_DO_LAST;
        case 63:
            return e_MAV_CMD_MAV_CMD_PREFLIGHT_CALIBRATION;
        case 64:
            return e_MAV_CMD_MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS;
        case 65:
            return e_MAV_CMD_MAV_CMD_PREFLIGHT_UAVCAN;
        case 66:
            return e_MAV_CMD_MAV_CMD_PREFLIGHT_STORAGE;
        case 67:
            return e_MAV_CMD_MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN;
        case 68:
            return e_MAV_CMD_MAV_CMD_OVERRIDE_GOTO;
        case 69:
            return e_MAV_CMD_MAV_CMD_MISSION_START;
        case 70:
            return e_MAV_CMD_MAV_CMD_COMPONENT_ARM_DISARM;
        case 71:
            return e_MAV_CMD_MAV_CMD_GET_HOME_POSITION;
        case 72:
            return e_MAV_CMD_MAV_CMD_START_RX_PAIR;
        case 73:
            return e_MAV_CMD_MAV_CMD_GET_MESSAGE_INTERVAL;
        case 74:
            return e_MAV_CMD_MAV_CMD_SET_MESSAGE_INTERVAL;
        case 75:
            return e_MAV_CMD_MAV_CMD_REQUEST_PROTOCOL_VERSION;
        case 76:
            return e_MAV_CMD_MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES;
        case 77:
            return e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_INFORMATION;
        case 78:
            return e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_SETTINGS;
        case 79:
            return e_MAV_CMD_MAV_CMD_REQUEST_STORAGE_INFORMATION;
        case 80:
            return e_MAV_CMD_MAV_CMD_STORAGE_FORMAT;
        case 81:
            return e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS;
        case 82:
            return e_MAV_CMD_MAV_CMD_REQUEST_FLIGHT_INFORMATION;
        case 83:
            return e_MAV_CMD_MAV_CMD_RESET_CAMERA_SETTINGS;
        case 84:
            return e_MAV_CMD_MAV_CMD_SET_CAMERA_MODE;
        case 85:
            return e_MAV_CMD_MAV_CMD_IMAGE_START_CAPTURE;
        case 86:
            return e_MAV_CMD_MAV_CMD_IMAGE_STOP_CAPTURE;
        case 87:
            return e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE;
        case 88:
            return e_MAV_CMD_MAV_CMD_DO_TRIGGER_CONTROL;
        case 89:
            return e_MAV_CMD_MAV_CMD_VIDEO_START_CAPTURE;
        case 90:
            return e_MAV_CMD_MAV_CMD_VIDEO_STOP_CAPTURE;
        case 91:
            return e_MAV_CMD_MAV_CMD_VIDEO_START_STREAMING;
        case 92:
            return e_MAV_CMD_MAV_CMD_VIDEO_STOP_STREAMING;
        case 93:
            return e_MAV_CMD_MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION;
        case 94:
            return e_MAV_CMD_MAV_CMD_LOGGING_START;
        case 95:
            return e_MAV_CMD_MAV_CMD_LOGGING_STOP;
        case 96:
            return e_MAV_CMD_MAV_CMD_AIRFRAME_CONFIGURATION;
        case 97:
            return e_MAV_CMD_MAV_CMD_PANORAMA_CREATE;
        case 98:
            return e_MAV_CMD_MAV_CMD_DO_VTOL_TRANSITION;
        case 99:
            return e_MAV_CMD_MAV_CMD_ARM_AUTHORIZATION_REQUEST;
        case 100:
            return e_MAV_CMD_MAV_CMD_SET_GUIDED_SUBMODE_STANDARD;
        case 101:
            return e_MAV_CMD_MAV_CMD_SET_GUIDED_SUBMODE_CIRCLE;
        case 102:
            return e_MAV_CMD_MAV_CMD_CONDITION_GATE;
        case 103:
            return e_MAV_CMD_MAV_CMD_NAV_FENCE_RETURN_POINT;
        case 104:
            return e_MAV_CMD_MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION;
        case 105:
            return e_MAV_CMD_MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION;
        case 106:
            return e_MAV_CMD_MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION;
        case 107:
            return e_MAV_CMD_MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION;
        case 108:
            return e_MAV_CMD_MAV_CMD_NAV_RALLY_POINT;
        case 109:
            return e_MAV_CMD_MAV_CMD_UAVCAN_GET_NODE_INFO;
        case 110:
            return e_MAV_CMD_MAV_CMD_PAYLOAD_PREPARE_DEPLOY;
        case 111:
            return e_MAV_CMD_MAV_CMD_PAYLOAD_CONTROL_DEPLOY;
        case 112:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_1;
        case 113:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_2;
        case 114:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_3;
        case 115:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_4;
        case 116:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_5;
        case 117:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_1;
        case 118:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_2;
        case 119:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_3;
        case 120:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_4;
        case 121:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_5;
        case 122:
            return e_MAV_CMD_MAV_CMD_USER_1;
        case 123:
            return e_MAV_CMD_MAV_CMD_USER_2;
        case 124:
            return e_MAV_CMD_MAV_CMD_USER_3;
        case 125:
            return e_MAV_CMD_MAV_CMD_USER_4;
        case 126:
            return e_MAV_CMD_MAV_CMD_USER_5;
        case 127:
            return e_MAV_CMD_MAV_CMD_RESET_MPPT;
        case 128:
            return e_MAV_CMD_MAV_CMD_PAYLOAD_CONTROL;
        default: ;//assert(false);//("Unknown enum ID " + id);
    }
}
INLINER void p39_command_SET(e_MAV_CMD  src, Pack * dst)//The scheduled action for the waypoint. see MAV_CMD in common.xml MAVLink spec
{
    uint8_t * data = dst->data;
    int32_t id;
    switch(src)
    {
        case e_MAV_CMD_MAV_CMD_NAV_WAYPOINT:
            id = 0;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_LOITER_UNLIM:
            id = 1;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_LOITER_TURNS:
            id = 2;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_LOITER_TIME:
            id = 3;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_RETURN_TO_LAUNCH:
            id = 4;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_LAND:
            id = 5;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_TAKEOFF:
            id = 6;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_LAND_LOCAL:
            id = 7;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_TAKEOFF_LOCAL:
            id = 8;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_FOLLOW:
            id = 9;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT:
            id = 10;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_LOITER_TO_ALT:
            id = 11;
            break;
        case e_MAV_CMD_MAV_CMD_DO_FOLLOW:
            id = 12;
            break;
        case e_MAV_CMD_MAV_CMD_DO_FOLLOW_REPOSITION:
            id = 13;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_ROI:
            id = 14;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_PATHPLANNING:
            id = 15;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_SPLINE_WAYPOINT:
            id = 16;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_VTOL_TAKEOFF:
            id = 17;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_VTOL_LAND:
            id = 18;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_GUIDED_ENABLE:
            id = 19;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_DELAY:
            id = 20;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_PAYLOAD_PLACE:
            id = 21;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_LAST:
            id = 22;
            break;
        case e_MAV_CMD_MAV_CMD_CONDITION_DELAY:
            id = 23;
            break;
        case e_MAV_CMD_MAV_CMD_CONDITION_CHANGE_ALT:
            id = 24;
            break;
        case e_MAV_CMD_MAV_CMD_CONDITION_DISTANCE:
            id = 25;
            break;
        case e_MAV_CMD_MAV_CMD_CONDITION_YAW:
            id = 26;
            break;
        case e_MAV_CMD_MAV_CMD_CONDITION_LAST:
            id = 27;
            break;
        case e_MAV_CMD_MAV_CMD_DO_SET_MODE:
            id = 28;
            break;
        case e_MAV_CMD_MAV_CMD_DO_JUMP:
            id = 29;
            break;
        case e_MAV_CMD_MAV_CMD_DO_CHANGE_SPEED:
            id = 30;
            break;
        case e_MAV_CMD_MAV_CMD_DO_SET_HOME:
            id = 31;
            break;
        case e_MAV_CMD_MAV_CMD_DO_SET_PARAMETER:
            id = 32;
            break;
        case e_MAV_CMD_MAV_CMD_DO_SET_RELAY:
            id = 33;
            break;
        case e_MAV_CMD_MAV_CMD_DO_REPEAT_RELAY:
            id = 34;
            break;
        case e_MAV_CMD_MAV_CMD_DO_SET_SERVO:
            id = 35;
            break;
        case e_MAV_CMD_MAV_CMD_DO_REPEAT_SERVO:
            id = 36;
            break;
        case e_MAV_CMD_MAV_CMD_DO_FLIGHTTERMINATION:
            id = 37;
            break;
        case e_MAV_CMD_MAV_CMD_DO_CHANGE_ALTITUDE:
            id = 38;
            break;
        case e_MAV_CMD_MAV_CMD_DO_LAND_START:
            id = 39;
            break;
        case e_MAV_CMD_MAV_CMD_DO_RALLY_LAND:
            id = 40;
            break;
        case e_MAV_CMD_MAV_CMD_DO_GO_AROUND:
            id = 41;
            break;
        case e_MAV_CMD_MAV_CMD_DO_REPOSITION:
            id = 42;
            break;
        case e_MAV_CMD_MAV_CMD_DO_PAUSE_CONTINUE:
            id = 43;
            break;
        case e_MAV_CMD_MAV_CMD_DO_SET_REVERSE:
            id = 44;
            break;
        case e_MAV_CMD_MAV_CMD_DO_CONTROL_VIDEO:
            id = 45;
            break;
        case e_MAV_CMD_MAV_CMD_DO_SET_ROI:
            id = 46;
            break;
        case e_MAV_CMD_MAV_CMD_DO_DIGICAM_CONFIGURE:
            id = 47;
            break;
        case e_MAV_CMD_MAV_CMD_DO_DIGICAM_CONTROL:
            id = 48;
            break;
        case e_MAV_CMD_MAV_CMD_DO_MOUNT_CONFIGURE:
            id = 49;
            break;
        case e_MAV_CMD_MAV_CMD_DO_MOUNT_CONTROL:
            id = 50;
            break;
        case e_MAV_CMD_MAV_CMD_DO_SET_CAM_TRIGG_DIST:
            id = 51;
            break;
        case e_MAV_CMD_MAV_CMD_DO_FENCE_ENABLE:
            id = 52;
            break;
        case e_MAV_CMD_MAV_CMD_DO_PARACHUTE:
            id = 53;
            break;
        case e_MAV_CMD_MAV_CMD_DO_MOTOR_TEST:
            id = 54;
            break;
        case e_MAV_CMD_MAV_CMD_DO_INVERTED_FLIGHT:
            id = 55;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_SET_YAW_SPEED:
            id = 56;
            break;
        case e_MAV_CMD_MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL:
            id = 57;
            break;
        case e_MAV_CMD_MAV_CMD_DO_MOUNT_CONTROL_QUAT:
            id = 58;
            break;
        case e_MAV_CMD_MAV_CMD_DO_GUIDED_MASTER:
            id = 59;
            break;
        case e_MAV_CMD_MAV_CMD_DO_GUIDED_LIMITS:
            id = 60;
            break;
        case e_MAV_CMD_MAV_CMD_DO_ENGINE_CONTROL:
            id = 61;
            break;
        case e_MAV_CMD_MAV_CMD_DO_LAST:
            id = 62;
            break;
        case e_MAV_CMD_MAV_CMD_PREFLIGHT_CALIBRATION:
            id = 63;
            break;
        case e_MAV_CMD_MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS:
            id = 64;
            break;
        case e_MAV_CMD_MAV_CMD_PREFLIGHT_UAVCAN:
            id = 65;
            break;
        case e_MAV_CMD_MAV_CMD_PREFLIGHT_STORAGE:
            id = 66;
            break;
        case e_MAV_CMD_MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
            id = 67;
            break;
        case e_MAV_CMD_MAV_CMD_OVERRIDE_GOTO:
            id = 68;
            break;
        case e_MAV_CMD_MAV_CMD_MISSION_START:
            id = 69;
            break;
        case e_MAV_CMD_MAV_CMD_COMPONENT_ARM_DISARM:
            id = 70;
            break;
        case e_MAV_CMD_MAV_CMD_GET_HOME_POSITION:
            id = 71;
            break;
        case e_MAV_CMD_MAV_CMD_START_RX_PAIR:
            id = 72;
            break;
        case e_MAV_CMD_MAV_CMD_GET_MESSAGE_INTERVAL:
            id = 73;
            break;
        case e_MAV_CMD_MAV_CMD_SET_MESSAGE_INTERVAL:
            id = 74;
            break;
        case e_MAV_CMD_MAV_CMD_REQUEST_PROTOCOL_VERSION:
            id = 75;
            break;
        case e_MAV_CMD_MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES:
            id = 76;
            break;
        case e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_INFORMATION:
            id = 77;
            break;
        case e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_SETTINGS:
            id = 78;
            break;
        case e_MAV_CMD_MAV_CMD_REQUEST_STORAGE_INFORMATION:
            id = 79;
            break;
        case e_MAV_CMD_MAV_CMD_STORAGE_FORMAT:
            id = 80;
            break;
        case e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS:
            id = 81;
            break;
        case e_MAV_CMD_MAV_CMD_REQUEST_FLIGHT_INFORMATION:
            id = 82;
            break;
        case e_MAV_CMD_MAV_CMD_RESET_CAMERA_SETTINGS:
            id = 83;
            break;
        case e_MAV_CMD_MAV_CMD_SET_CAMERA_MODE:
            id = 84;
            break;
        case e_MAV_CMD_MAV_CMD_IMAGE_START_CAPTURE:
            id = 85;
            break;
        case e_MAV_CMD_MAV_CMD_IMAGE_STOP_CAPTURE:
            id = 86;
            break;
        case e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE:
            id = 87;
            break;
        case e_MAV_CMD_MAV_CMD_DO_TRIGGER_CONTROL:
            id = 88;
            break;
        case e_MAV_CMD_MAV_CMD_VIDEO_START_CAPTURE:
            id = 89;
            break;
        case e_MAV_CMD_MAV_CMD_VIDEO_STOP_CAPTURE:
            id = 90;
            break;
        case e_MAV_CMD_MAV_CMD_VIDEO_START_STREAMING:
            id = 91;
            break;
        case e_MAV_CMD_MAV_CMD_VIDEO_STOP_STREAMING:
            id = 92;
            break;
        case e_MAV_CMD_MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION:
            id = 93;
            break;
        case e_MAV_CMD_MAV_CMD_LOGGING_START:
            id = 94;
            break;
        case e_MAV_CMD_MAV_CMD_LOGGING_STOP:
            id = 95;
            break;
        case e_MAV_CMD_MAV_CMD_AIRFRAME_CONFIGURATION:
            id = 96;
            break;
        case e_MAV_CMD_MAV_CMD_PANORAMA_CREATE:
            id = 97;
            break;
        case e_MAV_CMD_MAV_CMD_DO_VTOL_TRANSITION:
            id = 98;
            break;
        case e_MAV_CMD_MAV_CMD_ARM_AUTHORIZATION_REQUEST:
            id = 99;
            break;
        case e_MAV_CMD_MAV_CMD_SET_GUIDED_SUBMODE_STANDARD:
            id = 100;
            break;
        case e_MAV_CMD_MAV_CMD_SET_GUIDED_SUBMODE_CIRCLE:
            id = 101;
            break;
        case e_MAV_CMD_MAV_CMD_CONDITION_GATE:
            id = 102;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_FENCE_RETURN_POINT:
            id = 103;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION:
            id = 104;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION:
            id = 105;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION:
            id = 106;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION:
            id = 107;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_RALLY_POINT:
            id = 108;
            break;
        case e_MAV_CMD_MAV_CMD_UAVCAN_GET_NODE_INFO:
            id = 109;
            break;
        case e_MAV_CMD_MAV_CMD_PAYLOAD_PREPARE_DEPLOY:
            id = 110;
            break;
        case e_MAV_CMD_MAV_CMD_PAYLOAD_CONTROL_DEPLOY:
            id = 111;
            break;
        case e_MAV_CMD_MAV_CMD_WAYPOINT_USER_1:
            id = 112;
            break;
        case e_MAV_CMD_MAV_CMD_WAYPOINT_USER_2:
            id = 113;
            break;
        case e_MAV_CMD_MAV_CMD_WAYPOINT_USER_3:
            id = 114;
            break;
        case e_MAV_CMD_MAV_CMD_WAYPOINT_USER_4:
            id = 115;
            break;
        case e_MAV_CMD_MAV_CMD_WAYPOINT_USER_5:
            id = 116;
            break;
        case e_MAV_CMD_MAV_CMD_SPATIAL_USER_1:
            id = 117;
            break;
        case e_MAV_CMD_MAV_CMD_SPATIAL_USER_2:
            id = 118;
            break;
        case e_MAV_CMD_MAV_CMD_SPATIAL_USER_3:
            id = 119;
            break;
        case e_MAV_CMD_MAV_CMD_SPATIAL_USER_4:
            id = 120;
            break;
        case e_MAV_CMD_MAV_CMD_SPATIAL_USER_5:
            id = 121;
            break;
        case e_MAV_CMD_MAV_CMD_USER_1:
            id = 122;
            break;
        case e_MAV_CMD_MAV_CMD_USER_2:
            id = 123;
            break;
        case e_MAV_CMD_MAV_CMD_USER_3:
            id = 124;
            break;
        case e_MAV_CMD_MAV_CMD_USER_4:
            id = 125;
            break;
        case e_MAV_CMD_MAV_CMD_USER_5:
            id = 126;
            break;
        case e_MAV_CMD_MAV_CMD_RESET_MPPT:
            id = 127;
            break;
        case e_MAV_CMD_MAV_CMD_PAYLOAD_CONTROL:
            id = 128;
            break;
        default: ;// assert(false);//("Unknown enum" + id);
    }
    set_bits(id, 8, data, 276);
}
INLINER e_MAV_MISSION_TYPE p39_mission_type_GET(Pack * src)//Mission type, see MAV_MISSION_TYP
{
    uint8_t * data = src->data;
    switch(get_bits(data, 284, 3))
    {
        case 0:
            return e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION;
        case 1:
            return e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE;
        case 2:
            return e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY;
        case 3:
            return e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL;
        default: ;//assert(false);//("Unknown enum ID " + id);
    }
}
INLINER void p39_mission_type_SET(e_MAV_MISSION_TYPE  src, Pack * dst)//Mission type, see MAV_MISSION_TYP
{
    uint8_t * data = dst->data;
    int32_t id;
    switch(src)
    {
        case e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION:
            id = 0;
            break;
        case e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE:
            id = 1;
            break;
        case e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY:
            id = 2;
            break;
        case e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL:
            id = 3;
            break;
        default: ;// assert(false);//("Unknown enum" + id);
    }
    set_bits(id, 3, data, 284);
}
INLINER uint16_t p40_seq_GET(Pack * src)//Sequenc
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER void p40_seq_SET(uint16_t  src, Pack * dst)//Sequenc
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER uint8_t p40_target_system_GET(Pack * src)//System I
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 1)));
}
INLINER void p40_target_system_SET(uint8_t  src, Pack * dst)//System I
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  2);
}
INLINER uint8_t p40_target_component_GET(Pack * src)//Component I
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  3, 1)));
}
INLINER void p40_target_component_SET(uint8_t  src, Pack * dst)//Component I
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  3);
}
INLINER e_MAV_MISSION_TYPE p40_mission_type_GET(Pack * src)//Mission type, see MAV_MISSION_TYP
{
    uint8_t * data = src->data;
    switch(get_bits(data, 32, 3))
    {
        case 0:
            return e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION;
        case 1:
            return e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE;
        case 2:
            return e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY;
        case 3:
            return e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL;
        default: ;//assert(false);//("Unknown enum ID " + id);
    }
}
INLINER void p40_mission_type_SET(e_MAV_MISSION_TYPE  src, Pack * dst)//Mission type, see MAV_MISSION_TYP
{
    uint8_t * data = dst->data;
    int32_t id;
    switch(src)
    {
        case e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION:
            id = 0;
            break;
        case e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE:
            id = 1;
            break;
        case e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY:
            id = 2;
            break;
        case e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL:
            id = 3;
            break;
        default: ;// assert(false);//("Unknown enum" + id);
    }
    set_bits(id, 3, data, 32);
}
INLINER uint16_t p41_seq_GET(Pack * src)//Sequenc
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER void p41_seq_SET(uint16_t  src, Pack * dst)//Sequenc
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER uint8_t p41_target_system_GET(Pack * src)//System I
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 1)));
}
INLINER void p41_target_system_SET(uint8_t  src, Pack * dst)//System I
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  2);
}
INLINER uint8_t p41_target_component_GET(Pack * src)//Component I
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  3, 1)));
}
INLINER void p41_target_component_SET(uint8_t  src, Pack * dst)//Component I
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  3);
}
INLINER uint16_t p42_seq_GET(Pack * src)//Sequenc
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER void p42_seq_SET(uint16_t  src, Pack * dst)//Sequenc
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER uint8_t p43_target_system_GET(Pack * src)//System I
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 1)));
}
INLINER void p43_target_system_SET(uint8_t  src, Pack * dst)//System I
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  0);
}
INLINER uint8_t p43_target_component_GET(Pack * src)//Component I
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  1, 1)));
}
INLINER void p43_target_component_SET(uint8_t  src, Pack * dst)//Component I
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  1);
}
INLINER e_MAV_MISSION_TYPE p43_mission_type_GET(Pack * src)//Mission type, see MAV_MISSION_TYP
{
    uint8_t * data = src->data;
    switch(get_bits(data, 16, 3))
    {
        case 0:
            return e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION;
        case 1:
            return e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE;
        case 2:
            return e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY;
        case 3:
            return e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL;
        default: ;//assert(false);//("Unknown enum ID " + id);
    }
}
INLINER void p43_mission_type_SET(e_MAV_MISSION_TYPE  src, Pack * dst)//Mission type, see MAV_MISSION_TYP
{
    uint8_t * data = dst->data;
    int32_t id;
    switch(src)
    {
        case e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION:
            id = 0;
            break;
        case e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE:
            id = 1;
            break;
        case e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY:
            id = 2;
            break;
        case e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL:
            id = 3;
            break;
        default: ;// assert(false);//("Unknown enum" + id);
    }
    set_bits(id, 3, data, 16);
}
INLINER uint16_t p44_count_GET(Pack * src)//Number of mission items in the sequenc
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER void p44_count_SET(uint16_t  src, Pack * dst)//Number of mission items in the sequenc
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER uint8_t p44_target_system_GET(Pack * src)//System I
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 1)));
}
INLINER void p44_target_system_SET(uint8_t  src, Pack * dst)//System I
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  2);
}
INLINER uint8_t p44_target_component_GET(Pack * src)//Component I
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  3, 1)));
}
INLINER void p44_target_component_SET(uint8_t  src, Pack * dst)//Component I
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  3);
}
INLINER e_MAV_MISSION_TYPE p44_mission_type_GET(Pack * src)//Mission type, see MAV_MISSION_TYP
{
    uint8_t * data = src->data;
    switch(get_bits(data, 32, 3))
    {
        case 0:
            return e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION;
        case 1:
            return e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE;
        case 2:
            return e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY;
        case 3:
            return e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL;
        default: ;//assert(false);//("Unknown enum ID " + id);
    }
}
INLINER void p44_mission_type_SET(e_MAV_MISSION_TYPE  src, Pack * dst)//Mission type, see MAV_MISSION_TYP
{
    uint8_t * data = dst->data;
    int32_t id;
    switch(src)
    {
        case e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION:
            id = 0;
            break;
        case e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE:
            id = 1;
            break;
        case e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY:
            id = 2;
            break;
        case e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL:
            id = 3;
            break;
        default: ;// assert(false);//("Unknown enum" + id);
    }
    set_bits(id, 3, data, 32);
}
INLINER uint8_t p45_target_system_GET(Pack * src)//System I
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 1)));
}
INLINER void p45_target_system_SET(uint8_t  src, Pack * dst)//System I
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  0);
}
INLINER uint8_t p45_target_component_GET(Pack * src)//Component I
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  1, 1)));
}
INLINER void p45_target_component_SET(uint8_t  src, Pack * dst)//Component I
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  1);
}
INLINER e_MAV_MISSION_TYPE p45_mission_type_GET(Pack * src)//Mission type, see MAV_MISSION_TYP
{
    uint8_t * data = src->data;
    switch(get_bits(data, 16, 3))
    {
        case 0:
            return e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION;
        case 1:
            return e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE;
        case 2:
            return e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY;
        case 3:
            return e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL;
        default: ;//assert(false);//("Unknown enum ID " + id);
    }
}
INLINER void p45_mission_type_SET(e_MAV_MISSION_TYPE  src, Pack * dst)//Mission type, see MAV_MISSION_TYP
{
    uint8_t * data = dst->data;
    int32_t id;
    switch(src)
    {
        case e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION:
            id = 0;
            break;
        case e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE:
            id = 1;
            break;
        case e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY:
            id = 2;
            break;
        case e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL:
            id = 3;
            break;
        default: ;// assert(false);//("Unknown enum" + id);
    }
    set_bits(id, 3, data, 16);
}
INLINER uint16_t p46_seq_GET(Pack * src)//Sequenc
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER void p46_seq_SET(uint16_t  src, Pack * dst)//Sequenc
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER uint8_t p47_target_system_GET(Pack * src)//System I
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 1)));
}
INLINER void p47_target_system_SET(uint8_t  src, Pack * dst)//System I
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  0);
}
INLINER uint8_t p47_target_component_GET(Pack * src)//Component I
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  1, 1)));
}
INLINER void p47_target_component_SET(uint8_t  src, Pack * dst)//Component I
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  1);
}
INLINER e_MAV_MISSION_RESULT p47_type_GET(Pack * src)//See MAV_MISSION_RESULT enu
{
    uint8_t * data = src->data;
    return  0 + (int)get_bits(data, 16, 4);
}
INLINER void p47_type_SET(e_MAV_MISSION_RESULT  src, Pack * dst)//See MAV_MISSION_RESULT enu
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 4, data, 16);
}
INLINER e_MAV_MISSION_TYPE p47_mission_type_GET(Pack * src)//Mission type, see MAV_MISSION_TYP
{
    uint8_t * data = src->data;
    switch(get_bits(data, 20, 3))
    {
        case 0:
            return e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION;
        case 1:
            return e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE;
        case 2:
            return e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY;
        case 3:
            return e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL;
        default: ;//assert(false);//("Unknown enum ID " + id);
    }
}
INLINER void p47_mission_type_SET(e_MAV_MISSION_TYPE  src, Pack * dst)//Mission type, see MAV_MISSION_TYP
{
    uint8_t * data = dst->data;
    int32_t id;
    switch(src)
    {
        case e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION:
            id = 0;
            break;
        case e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE:
            id = 1;
            break;
        case e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY:
            id = 2;
            break;
        case e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL:
            id = 3;
            break;
        default: ;// assert(false);//("Unknown enum" + id);
    }
    set_bits(id, 3, data, 20);
}
INLINER uint8_t p48_target_system_GET(Pack * src)//System I
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 1)));
}
INLINER void p48_target_system_SET(uint8_t  src, Pack * dst)//System I
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  0);
}
INLINER int32_t p48_latitude_GET(Pack * src)//Latitude (WGS84), in degrees * 1E
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  1, 4)));
}
INLINER void p48_latitude_SET(int32_t  src, Pack * dst)//Latitude (WGS84), in degrees * 1E
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  1);
}
INLINER int32_t p48_longitude_GET(Pack * src)//Longitude (WGS84, in degrees * 1E
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  5, 4)));
}
INLINER void p48_longitude_SET(int32_t  src, Pack * dst)//Longitude (WGS84, in degrees * 1E
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  5);
}
INLINER int32_t p48_altitude_GET(Pack * src)//Altitude (AMSL), in meters * 1000 (positive for up
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  9, 4)));
}
INLINER void p48_altitude_SET(int32_t  src, Pack * dst)//Altitude (AMSL), in meters * 1000 (positive for up
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  9);
}
INLINER uint64_t  p48_time_usec_TRY(Bounds_Inside * src)
{
    if(src->base.field_bit !=  104 && !try_visit_field(src, 104)) return 0;
    uint8_t * data = src->base.pack->data;
    return ((get_bytes(data,  src->BYTE, 8)));
}
INLINER void p48_time_usec_SET(uint64_t  src, Bounds_Inside * dst)//Timestamp (microseconds since UNIX epoch or microseconds since system boot
{
    if(dst->base.field_bit != 104)insert_field(dst, 104, 0);
    uint8_t * data = dst->base.pack->data;
    set_bytes((src), 8, data,  dst->BYTE);
}
INLINER int32_t p49_latitude_GET(Pack * src)//Latitude (WGS84), in degrees * 1E
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  0, 4)));
}
INLINER void p49_latitude_SET(int32_t  src, Pack * dst)//Latitude (WGS84), in degrees * 1E
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  0);
}
INLINER int32_t p49_longitude_GET(Pack * src)//Longitude (WGS84), in degrees * 1E
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  4, 4)));
}
INLINER void p49_longitude_SET(int32_t  src, Pack * dst)//Longitude (WGS84), in degrees * 1E
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  4);
}
INLINER int32_t p49_altitude_GET(Pack * src)//Altitude (AMSL), in meters * 1000 (positive for up
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  8, 4)));
}
INLINER void p49_altitude_SET(int32_t  src, Pack * dst)//Altitude (AMSL), in meters * 1000 (positive for up
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  8);
}
INLINER uint64_t  p49_time_usec_TRY(Bounds_Inside * src)
{
    if(src->base.field_bit !=  96 && !try_visit_field(src, 96)) return 0;
    uint8_t * data = src->base.pack->data;
    return ((get_bytes(data,  src->BYTE, 8)));
}
INLINER void p49_time_usec_SET(uint64_t  src, Bounds_Inside * dst)//Timestamp (microseconds since UNIX epoch or microseconds since system boot
{
    if(dst->base.field_bit != 96)insert_field(dst, 96, 0);
    uint8_t * data = dst->base.pack->data;
    set_bytes((src), 8, data,  dst->BYTE);
}
INLINER uint8_t p50_target_system_GET(Pack * src)//System I
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 1)));
}
INLINER void p50_target_system_SET(uint8_t  src, Pack * dst)//System I
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  0);
}
INLINER uint8_t p50_target_component_GET(Pack * src)//Component I
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  1, 1)));
}
INLINER void p50_target_component_SET(uint8_t  src, Pack * dst)//Component I
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  1);
}
INLINER int16_t p50_param_index_GET(Pack * src)
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  2, 2)));
}
INLINER void p50_param_index_SET(int16_t  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  2);
}
INLINER uint8_t p50_parameter_rc_channel_index_GET(Pack * src)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  4, 1)));
}
INLINER void p50_parameter_rc_channel_index_SET(uint8_t  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  4);
}
INLINER float p50_param_value0_GET(Pack * src)//Initial parameter valu
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  5, 4)));
}
INLINER void p50_param_value0_SET(float  src, Pack * dst)//Initial parameter valu
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  5);
}
INLINER float p50_scale_GET(Pack * src)//Scale, maps the RC range [-1, 1] to a parameter valu
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  9, 4)));
}
INLINER void p50_scale_SET(float  src, Pack * dst)//Scale, maps the RC range [-1, 1] to a parameter valu
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  9);
}
INLINER float p50_param_value_min_GET(Pack * src)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  13, 4)));
}
INLINER void p50_param_value_min_SET(float  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  13);
}
INLINER float p50_param_value_max_GET(Pack * src)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  17, 4)));
}
INLINER void p50_param_value_max_SET(float  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  17);
}
INLINER char16_t * p50_param_id_GET(Bounds_Inside * src, char16_t *  dst, int32_t pos)
{
    uint8_t * data = src->base.pack->data;
    for(int32_t BYTE = src->BYTE, dst_max = pos + src->items; pos < dst_max ; pos++,  BYTE += 2)
        dst[pos] = ((get_bytes(data,  BYTE, 2)));
    return dst;
}
INLINER int32_t p50_param_id_LEN(Bounds_Inside * src)
{
    return (src->base.field_bit !=  168 && !try_visit_field(src, 168)  ||  !try_visit_item(src, 0)) ? 0 : src->items;
}
INLINER  char16_t* p50_param_id_TRY_(Bounds_Inside * src)
{
    if(src->base.field_bit !=  168 && !try_visit_field(src, 168)  ||  !try_visit_item(src, 0)) return NULL;
    char16_t * ret = p50_param_id_GET(src, malloc((src->items + 1) * 2), 0);
    ret[src->items] = 0;//the terminating null character.
    return ret;
}
INLINER void p50_param_id_SET(char16_t *  src, int32_t pos, int32_t items, Bounds_Inside * dst)
{
    if(dst->base.field_bit != 168 && insert_field(dst, 168, items) ||
            ! try_visit_item(dst, 0)) insert_item(dst, 0, items);
    uint8_t * data = dst->base.pack->data;
    for(int32_t BYTE =  dst->BYTE, src_max = pos + dst->items; pos < src_max; pos++, BYTE += 2)
        set_bytes((uint16_t)(src[pos]), 2, data,  BYTE);
}
INLINER void p50_param_id_SET_(char16_t*  src, Bounds_Inside * dst) {p50_param_id_SET(src, 0, strlen16(src), dst);}
INLINER uint16_t p51_seq_GET(Pack * src)//Sequenc
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER void p51_seq_SET(uint16_t  src, Pack * dst)//Sequenc
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER uint8_t p51_target_system_GET(Pack * src)//System I
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 1)));
}
INLINER void p51_target_system_SET(uint8_t  src, Pack * dst)//System I
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  2);
}
INLINER uint8_t p51_target_component_GET(Pack * src)//Component I
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  3, 1)));
}
INLINER void p51_target_component_SET(uint8_t  src, Pack * dst)//Component I
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  3);
}
INLINER e_MAV_MISSION_TYPE p51_mission_type_GET(Pack * src)//Mission type, see MAV_MISSION_TYP
{
    uint8_t * data = src->data;
    switch(get_bits(data, 32, 3))
    {
        case 0:
            return e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION;
        case 1:
            return e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE;
        case 2:
            return e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY;
        case 3:
            return e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL;
        default: ;//assert(false);//("Unknown enum ID " + id);
    }
}
INLINER void p51_mission_type_SET(e_MAV_MISSION_TYPE  src, Pack * dst)//Mission type, see MAV_MISSION_TYP
{
    uint8_t * data = dst->data;
    int32_t id;
    switch(src)
    {
        case e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION:
            id = 0;
            break;
        case e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE:
            id = 1;
            break;
        case e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY:
            id = 2;
            break;
        case e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL:
            id = 3;
            break;
        default: ;// assert(false);//("Unknown enum" + id);
    }
    set_bits(id, 3, data, 32);
}
INLINER uint8_t p54_target_system_GET(Pack * src)//System I
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 1)));
}
INLINER void p54_target_system_SET(uint8_t  src, Pack * dst)//System I
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  0);
}
INLINER uint8_t p54_target_component_GET(Pack * src)//Component I
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  1, 1)));
}
INLINER void p54_target_component_SET(uint8_t  src, Pack * dst)//Component I
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  1);
}
INLINER float p54_p1x_GET(Pack * src)//x position 1 / Latitude
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  2, 4)));
}
INLINER void p54_p1x_SET(float  src, Pack * dst)//x position 1 / Latitude
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  2);
}
INLINER float p54_p1y_GET(Pack * src)//y position 1 / Longitude
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  6, 4)));
}
INLINER void p54_p1y_SET(float  src, Pack * dst)//y position 1 / Longitude
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  6);
}
INLINER float p54_p1z_GET(Pack * src)//z position 1 / Altitude
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  10, 4)));
}
INLINER void p54_p1z_SET(float  src, Pack * dst)//z position 1 / Altitude
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  10);
}
INLINER float p54_p2x_GET(Pack * src)//x position 2 / Latitude
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  14, 4)));
}
INLINER void p54_p2x_SET(float  src, Pack * dst)//x position 2 / Latitude
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  14);
}
INLINER float p54_p2y_GET(Pack * src)//y position 2 / Longitude
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  18, 4)));
}
INLINER void p54_p2y_SET(float  src, Pack * dst)//y position 2 / Longitude
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  18);
}
INLINER float p54_p2z_GET(Pack * src)//z position 2 / Altitude
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  22, 4)));
}
INLINER void p54_p2z_SET(float  src, Pack * dst)//z position 2 / Altitude
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  22);
}
INLINER e_MAV_FRAME p54_frame_GET(Pack * src)
{
    uint8_t * data = src->data;
    return  0 + (int)get_bits(data, 208, 4);
}
INLINER void p54_frame_SET(e_MAV_FRAME  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 4, data, 208);
}
INLINER float p55_p1x_GET(Pack * src)//x position 1 / Latitude
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  0, 4)));
}
INLINER void p55_p1x_SET(float  src, Pack * dst)//x position 1 / Latitude
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  0);
}
INLINER float p55_p1y_GET(Pack * src)//y position 1 / Longitude
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  4, 4)));
}
INLINER void p55_p1y_SET(float  src, Pack * dst)//y position 1 / Longitude
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  4);
}
INLINER float p55_p1z_GET(Pack * src)//z position 1 / Altitude
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  8, 4)));
}
INLINER void p55_p1z_SET(float  src, Pack * dst)//z position 1 / Altitude
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  8);
}
INLINER float p55_p2x_GET(Pack * src)//x position 2 / Latitude
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  12, 4)));
}
INLINER void p55_p2x_SET(float  src, Pack * dst)//x position 2 / Latitude
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  12);
}
INLINER float p55_p2y_GET(Pack * src)//y position 2 / Longitude
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  16, 4)));
}
INLINER void p55_p2y_SET(float  src, Pack * dst)//y position 2 / Longitude
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  16);
}
INLINER float p55_p2z_GET(Pack * src)//z position 2 / Altitude
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  20, 4)));
}
INLINER void p55_p2z_SET(float  src, Pack * dst)//z position 2 / Altitude
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  20);
}
INLINER e_MAV_FRAME p55_frame_GET(Pack * src)
{
    uint8_t * data = src->data;
    return  0 + (int)get_bits(data, 192, 4);
}
INLINER void p55_frame_SET(e_MAV_FRAME  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 4, data, 192);
}
INLINER uint64_t p61_time_usec_GET(Pack * src)//Timestamp (microseconds since system boot or since UNIX epoch
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 8)));
}
INLINER void p61_time_usec_SET(uint64_t  src, Pack * dst)//Timestamp (microseconds since system boot or since UNIX epoch
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  0);
}
INLINER float* p61_q_GET(Pack * src, float*  dst, int32_t pos) //Quaternion components, w, x, y, z (1 0 0 0 is the null-rotation
{
    uint8_t * data = src->data;
    for(int32_t BYTE = 8, dst_max = pos + 4; pos < dst_max ; pos++,  BYTE += 4)
        dst[pos] = (intBitsToFloat(get_bytes(data,  BYTE, 4)));
    return dst;
}

static const  uint32_t p61_q_LEN = 4; //return array length

INLINER  float*  p61_q_GET_(Pack * src) {return p61_q_GET(src, malloc(4 * sizeof(float)), 0);}
INLINER void p61_q_SET(float*  src, int32_t pos, Pack * dst) //Quaternion components, w, x, y, z (1 0 0 0 is the null-rotation
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  8, src_max = pos + 4; pos < src_max; pos++, BYTE += 4)
        set_bytes(floatToIntBits(src[pos]), 4, data,  BYTE);
}
INLINER float p61_rollspeed_GET(Pack * src)//Roll angular speed (rad/s
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  24, 4)));
}
INLINER void p61_rollspeed_SET(float  src, Pack * dst)//Roll angular speed (rad/s
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  24);
}
INLINER float p61_pitchspeed_GET(Pack * src)//Pitch angular speed (rad/s
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  28, 4)));
}
INLINER void p61_pitchspeed_SET(float  src, Pack * dst)//Pitch angular speed (rad/s
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  28);
}
INLINER float p61_yawspeed_GET(Pack * src)//Yaw angular speed (rad/s
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  32, 4)));
}
INLINER void p61_yawspeed_SET(float  src, Pack * dst)//Yaw angular speed (rad/s
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  32);
}
INLINER float* p61_covariance_GET(Pack * src, float*  dst, int32_t pos) //Attitude covarianc
{
    uint8_t * data = src->data;
    for(int32_t BYTE = 36, dst_max = pos + 9; pos < dst_max ; pos++,  BYTE += 4)
        dst[pos] = (intBitsToFloat(get_bytes(data,  BYTE, 4)));
    return dst;
}

static const  uint32_t p61_covariance_LEN = 9; //return array length

INLINER  float*  p61_covariance_GET_(Pack * src) {return p61_covariance_GET(src, malloc(9 * sizeof(float)), 0);}
INLINER void p61_covariance_SET(float*  src, int32_t pos, Pack * dst) //Attitude covarianc
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  36, src_max = pos + 9; pos < src_max; pos++, BYTE += 4)
        set_bytes(floatToIntBits(src[pos]), 4, data,  BYTE);
}
INLINER uint16_t p62_wp_dist_GET(Pack * src)//Distance to active waypoint in meter
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER void p62_wp_dist_SET(uint16_t  src, Pack * dst)//Distance to active waypoint in meter
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER float p62_nav_roll_GET(Pack * src)//Current desired roll in degree
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  2, 4)));
}
INLINER void p62_nav_roll_SET(float  src, Pack * dst)//Current desired roll in degree
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  2);
}
INLINER float p62_nav_pitch_GET(Pack * src)//Current desired pitch in degree
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  6, 4)));
}
INLINER void p62_nav_pitch_SET(float  src, Pack * dst)//Current desired pitch in degree
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  6);
}
INLINER int16_t p62_nav_bearing_GET(Pack * src)//Current desired heading in degree
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  10, 2)));
}
INLINER void p62_nav_bearing_SET(int16_t  src, Pack * dst)//Current desired heading in degree
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  10);
}
INLINER int16_t p62_target_bearing_GET(Pack * src)//Bearing to current waypoint/target in degree
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  12, 2)));
}
INLINER void p62_target_bearing_SET(int16_t  src, Pack * dst)//Bearing to current waypoint/target in degree
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  12);
}
INLINER float p62_alt_error_GET(Pack * src)//Current altitude error in meter
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  14, 4)));
}
INLINER void p62_alt_error_SET(float  src, Pack * dst)//Current altitude error in meter
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  14);
}
INLINER float p62_aspd_error_GET(Pack * src)//Current airspeed error in meters/secon
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  18, 4)));
}
INLINER void p62_aspd_error_SET(float  src, Pack * dst)//Current airspeed error in meters/secon
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  18);
}
INLINER float p62_xtrack_error_GET(Pack * src)//Current crosstrack error on x-y plane in meter
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  22, 4)));
}
INLINER void p62_xtrack_error_SET(float  src, Pack * dst)//Current crosstrack error on x-y plane in meter
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  22);
}
INLINER uint64_t p63_time_usec_GET(Pack * src)//Timestamp (microseconds since system boot or since UNIX epoch
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 8)));
}
INLINER void p63_time_usec_SET(uint64_t  src, Pack * dst)//Timestamp (microseconds since system boot or since UNIX epoch
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  0);
}
INLINER int32_t p63_lat_GET(Pack * src)//Latitude, expressed as degrees * 1E
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  8, 4)));
}
INLINER void p63_lat_SET(int32_t  src, Pack * dst)//Latitude, expressed as degrees * 1E
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  8);
}
INLINER int32_t p63_lon_GET(Pack * src)//Longitude, expressed as degrees * 1E
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  12, 4)));
}
INLINER void p63_lon_SET(int32_t  src, Pack * dst)//Longitude, expressed as degrees * 1E
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  12);
}
INLINER int32_t p63_alt_GET(Pack * src)//Altitude in meters, expressed as * 1000 (millimeters), above MS
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  16, 4)));
}
INLINER void p63_alt_SET(int32_t  src, Pack * dst)//Altitude in meters, expressed as * 1000 (millimeters), above MS
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  16);
}
INLINER int32_t p63_relative_alt_GET(Pack * src)//Altitude above ground in meters, expressed as * 1000 (millimeters
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  20, 4)));
}
INLINER void p63_relative_alt_SET(int32_t  src, Pack * dst)//Altitude above ground in meters, expressed as * 1000 (millimeters
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  20);
}
INLINER float p63_vx_GET(Pack * src)//Ground X Speed (Latitude), expressed as m/
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  24, 4)));
}
INLINER void p63_vx_SET(float  src, Pack * dst)//Ground X Speed (Latitude), expressed as m/
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  24);
}
INLINER float p63_vy_GET(Pack * src)//Ground Y Speed (Longitude), expressed as m/
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  28, 4)));
}
INLINER void p63_vy_SET(float  src, Pack * dst)//Ground Y Speed (Longitude), expressed as m/
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  28);
}
INLINER float p63_vz_GET(Pack * src)//Ground Z Speed (Altitude), expressed as m/
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  32, 4)));
}
INLINER void p63_vz_SET(float  src, Pack * dst)//Ground Z Speed (Altitude), expressed as m/
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  32);
}
INLINER float* p63_covariance_GET(Pack * src, float*  dst, int32_t pos) //Covariance matrix (first six entries are the first ROW, next six entries are the second row, etc.
{
    uint8_t * data = src->data;
    for(int32_t BYTE = 36, dst_max = pos + 36; pos < dst_max ; pos++,  BYTE += 4)
        dst[pos] = (intBitsToFloat(get_bytes(data,  BYTE, 4)));
    return dst;
}

static const  uint32_t p63_covariance_LEN = 36; //return array length

INLINER  float*  p63_covariance_GET_(Pack * src) {return p63_covariance_GET(src, malloc(36 * sizeof(float)), 0);}
INLINER void p63_covariance_SET(float*  src, int32_t pos, Pack * dst) //Covariance matrix (first six entries are the first ROW, next six entries are the second row, etc.
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  36, src_max = pos + 36; pos < src_max; pos++, BYTE += 4)
        set_bytes(floatToIntBits(src[pos]), 4, data,  BYTE);
}
INLINER e_MAV_ESTIMATOR_TYPE p63_estimator_type_GET(Pack * src)//Class id of the estimator this estimate originated from
{
    uint8_t * data = src->data;
    return  1 + (int)get_bits(data, 1440, 3);
}
INLINER void p63_estimator_type_SET(e_MAV_ESTIMATOR_TYPE  src, Pack * dst)//Class id of the estimator this estimate originated from
{
    uint8_t * data = dst->data;
    set_bits(- 1 +   src, 3, data, 1440);
}
INLINER uint64_t p64_time_usec_GET(Pack * src)//Timestamp (microseconds since system boot or since UNIX epoch
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 8)));
}
INLINER void p64_time_usec_SET(uint64_t  src, Pack * dst)//Timestamp (microseconds since system boot or since UNIX epoch
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  0);
}
INLINER float p64_x_GET(Pack * src)//X Positio
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  8, 4)));
}
INLINER void p64_x_SET(float  src, Pack * dst)//X Positio
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  8);
}
INLINER float p64_y_GET(Pack * src)//Y Positio
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  12, 4)));
}
INLINER void p64_y_SET(float  src, Pack * dst)//Y Positio
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  12);
}
INLINER float p64_z_GET(Pack * src)//Z Positio
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  16, 4)));
}
INLINER void p64_z_SET(float  src, Pack * dst)//Z Positio
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  16);
}
INLINER float p64_vx_GET(Pack * src)//X Speed (m/s
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  20, 4)));
}
INLINER void p64_vx_SET(float  src, Pack * dst)//X Speed (m/s
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  20);
}
INLINER float p64_vy_GET(Pack * src)//Y Speed (m/s
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  24, 4)));
}
INLINER void p64_vy_SET(float  src, Pack * dst)//Y Speed (m/s
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  24);
}
INLINER float p64_vz_GET(Pack * src)//Z Speed (m/s
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  28, 4)));
}
INLINER void p64_vz_SET(float  src, Pack * dst)//Z Speed (m/s
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  28);
}
INLINER float p64_ax_GET(Pack * src)//X Acceleration (m/s^2
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  32, 4)));
}
INLINER void p64_ax_SET(float  src, Pack * dst)//X Acceleration (m/s^2
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  32);
}
INLINER float p64_ay_GET(Pack * src)//Y Acceleration (m/s^2
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  36, 4)));
}
INLINER void p64_ay_SET(float  src, Pack * dst)//Y Acceleration (m/s^2
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  36);
}
INLINER float p64_az_GET(Pack * src)//Z Acceleration (m/s^2
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  40, 4)));
}
INLINER void p64_az_SET(float  src, Pack * dst)//Z Acceleration (m/s^2
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  40);
}
INLINER float* p64_covariance_GET(Pack * src, float*  dst, int32_t pos)
{
    uint8_t * data = src->data;
    for(int32_t BYTE = 44, dst_max = pos + 45; pos < dst_max ; pos++,  BYTE += 4)
        dst[pos] = (intBitsToFloat(get_bytes(data,  BYTE, 4)));
    return dst;
}

static const  uint32_t p64_covariance_LEN = 45; //return array length

INLINER  float*  p64_covariance_GET_(Pack * src) {return p64_covariance_GET(src, malloc(45 * sizeof(float)), 0);}
INLINER void p64_covariance_SET(float*  src, int32_t pos, Pack * dst)
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  44, src_max = pos + 45; pos < src_max; pos++, BYTE += 4)
        set_bytes(floatToIntBits(src[pos]), 4, data,  BYTE);
}
INLINER e_MAV_ESTIMATOR_TYPE p64_estimator_type_GET(Pack * src)//Class id of the estimator this estimate originated from
{
    uint8_t * data = src->data;
    return  1 + (int)get_bits(data, 1792, 3);
}
INLINER void p64_estimator_type_SET(e_MAV_ESTIMATOR_TYPE  src, Pack * dst)//Class id of the estimator this estimate originated from
{
    uint8_t * data = dst->data;
    set_bits(- 1 +   src, 3, data, 1792);
}
INLINER uint16_t p65_chan1_raw_GET(Pack * src)//RC channel 1 value, in microseconds. A value of UINT16_MAX implies the channel is unused
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER void p65_chan1_raw_SET(uint16_t  src, Pack * dst)//RC channel 1 value, in microseconds. A value of UINT16_MAX implies the channel is unused
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER uint16_t p65_chan2_raw_GET(Pack * src)//RC channel 2 value, in microseconds. A value of UINT16_MAX implies the channel is unused
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 2)));
}
INLINER void p65_chan2_raw_SET(uint16_t  src, Pack * dst)//RC channel 2 value, in microseconds. A value of UINT16_MAX implies the channel is unused
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  2);
}
INLINER uint16_t p65_chan3_raw_GET(Pack * src)//RC channel 3 value, in microseconds. A value of UINT16_MAX implies the channel is unused
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  4, 2)));
}
INLINER void p65_chan3_raw_SET(uint16_t  src, Pack * dst)//RC channel 3 value, in microseconds. A value of UINT16_MAX implies the channel is unused
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  4);
}
INLINER uint16_t p65_chan4_raw_GET(Pack * src)//RC channel 4 value, in microseconds. A value of UINT16_MAX implies the channel is unused
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  6, 2)));
}
INLINER void p65_chan4_raw_SET(uint16_t  src, Pack * dst)//RC channel 4 value, in microseconds. A value of UINT16_MAX implies the channel is unused
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  6);
}
INLINER uint16_t p65_chan5_raw_GET(Pack * src)//RC channel 5 value, in microseconds. A value of UINT16_MAX implies the channel is unused
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  8, 2)));
}
INLINER void p65_chan5_raw_SET(uint16_t  src, Pack * dst)//RC channel 5 value, in microseconds. A value of UINT16_MAX implies the channel is unused
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  8);
}
INLINER uint16_t p65_chan6_raw_GET(Pack * src)//RC channel 6 value, in microseconds. A value of UINT16_MAX implies the channel is unused
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  10, 2)));
}
INLINER void p65_chan6_raw_SET(uint16_t  src, Pack * dst)//RC channel 6 value, in microseconds. A value of UINT16_MAX implies the channel is unused
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  10);
}
INLINER uint16_t p65_chan7_raw_GET(Pack * src)//RC channel 7 value, in microseconds. A value of UINT16_MAX implies the channel is unused
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  12, 2)));
}
INLINER void p65_chan7_raw_SET(uint16_t  src, Pack * dst)//RC channel 7 value, in microseconds. A value of UINT16_MAX implies the channel is unused
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  12);
}
INLINER uint16_t p65_chan8_raw_GET(Pack * src)//RC channel 8 value, in microseconds. A value of UINT16_MAX implies the channel is unused
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  14, 2)));
}
INLINER void p65_chan8_raw_SET(uint16_t  src, Pack * dst)//RC channel 8 value, in microseconds. A value of UINT16_MAX implies the channel is unused
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  14);
}
INLINER uint16_t p65_chan9_raw_GET(Pack * src)//RC channel 9 value, in microseconds. A value of UINT16_MAX implies the channel is unused
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  16, 2)));
}
INLINER void p65_chan9_raw_SET(uint16_t  src, Pack * dst)//RC channel 9 value, in microseconds. A value of UINT16_MAX implies the channel is unused
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  16);
}
INLINER uint16_t p65_chan10_raw_GET(Pack * src)//RC channel 10 value, in microseconds. A value of UINT16_MAX implies the channel is unused
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  18, 2)));
}
INLINER void p65_chan10_raw_SET(uint16_t  src, Pack * dst)//RC channel 10 value, in microseconds. A value of UINT16_MAX implies the channel is unused
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  18);
}
INLINER uint16_t p65_chan11_raw_GET(Pack * src)//RC channel 11 value, in microseconds. A value of UINT16_MAX implies the channel is unused
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  20, 2)));
}
INLINER void p65_chan11_raw_SET(uint16_t  src, Pack * dst)//RC channel 11 value, in microseconds. A value of UINT16_MAX implies the channel is unused
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  20);
}
INLINER uint16_t p65_chan12_raw_GET(Pack * src)//RC channel 12 value, in microseconds. A value of UINT16_MAX implies the channel is unused
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  22, 2)));
}
INLINER void p65_chan12_raw_SET(uint16_t  src, Pack * dst)//RC channel 12 value, in microseconds. A value of UINT16_MAX implies the channel is unused
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  22);
}
INLINER uint16_t p65_chan13_raw_GET(Pack * src)//RC channel 13 value, in microseconds. A value of UINT16_MAX implies the channel is unused
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  24, 2)));
}
INLINER void p65_chan13_raw_SET(uint16_t  src, Pack * dst)//RC channel 13 value, in microseconds. A value of UINT16_MAX implies the channel is unused
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  24);
}
INLINER uint16_t p65_chan14_raw_GET(Pack * src)//RC channel 14 value, in microseconds. A value of UINT16_MAX implies the channel is unused
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  26, 2)));
}
INLINER void p65_chan14_raw_SET(uint16_t  src, Pack * dst)//RC channel 14 value, in microseconds. A value of UINT16_MAX implies the channel is unused
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  26);
}
INLINER uint16_t p65_chan15_raw_GET(Pack * src)//RC channel 15 value, in microseconds. A value of UINT16_MAX implies the channel is unused
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  28, 2)));
}
INLINER void p65_chan15_raw_SET(uint16_t  src, Pack * dst)//RC channel 15 value, in microseconds. A value of UINT16_MAX implies the channel is unused
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  28);
}
INLINER uint16_t p65_chan16_raw_GET(Pack * src)//RC channel 16 value, in microseconds. A value of UINT16_MAX implies the channel is unused
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  30, 2)));
}
INLINER void p65_chan16_raw_SET(uint16_t  src, Pack * dst)//RC channel 16 value, in microseconds. A value of UINT16_MAX implies the channel is unused
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  30);
}
INLINER uint16_t p65_chan17_raw_GET(Pack * src)//RC channel 17 value, in microseconds. A value of UINT16_MAX implies the channel is unused
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  32, 2)));
}
INLINER void p65_chan17_raw_SET(uint16_t  src, Pack * dst)//RC channel 17 value, in microseconds. A value of UINT16_MAX implies the channel is unused
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  32);
}
INLINER uint16_t p65_chan18_raw_GET(Pack * src)//RC channel 18 value, in microseconds. A value of UINT16_MAX implies the channel is unused
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  34, 2)));
}
INLINER void p65_chan18_raw_SET(uint16_t  src, Pack * dst)//RC channel 18 value, in microseconds. A value of UINT16_MAX implies the channel is unused
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  34);
}
INLINER uint32_t p65_time_boot_ms_GET(Pack * src)//Timestamp (milliseconds since system boot
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  36, 4)));
}
INLINER void p65_time_boot_ms_SET(uint32_t  src, Pack * dst)//Timestamp (milliseconds since system boot
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  36);
}
INLINER uint8_t p65_chancount_GET(Pack * src)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  40, 1)));
}
INLINER void p65_chancount_SET(uint8_t  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  40);
}
INLINER uint8_t p65_rssi_GET(Pack * src)//Receive signal strength indicator, 0: 0%, 100: 100%, 255: invalid/unknown
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  41, 1)));
}
INLINER void p65_rssi_SET(uint8_t  src, Pack * dst)//Receive signal strength indicator, 0: 0%, 100: 100%, 255: invalid/unknown
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  41);
}
INLINER uint16_t p66_req_message_rate_GET(Pack * src)//The requested message rat
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER void p66_req_message_rate_SET(uint16_t  src, Pack * dst)//The requested message rat
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER uint8_t p66_target_system_GET(Pack * src)//The target requested to send the message stream
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 1)));
}
INLINER void p66_target_system_SET(uint8_t  src, Pack * dst)//The target requested to send the message stream
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  2);
}
INLINER uint8_t p66_target_component_GET(Pack * src)//The target requested to send the message stream
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  3, 1)));
}
INLINER void p66_target_component_SET(uint8_t  src, Pack * dst)//The target requested to send the message stream
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  3);
}
INLINER uint8_t p66_req_stream_id_GET(Pack * src)//The ID of the requested data strea
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  4, 1)));
}
INLINER void p66_req_stream_id_SET(uint8_t  src, Pack * dst)//The ID of the requested data strea
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  4);
}
INLINER uint8_t p66_start_stop_GET(Pack * src)//1 to start sending, 0 to stop sending
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  5, 1)));
}
INLINER void p66_start_stop_SET(uint8_t  src, Pack * dst)//1 to start sending, 0 to stop sending
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  5);
}
INLINER uint16_t p67_message_rate_GET(Pack * src)//The message rat
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER void p67_message_rate_SET(uint16_t  src, Pack * dst)//The message rat
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER uint8_t p67_stream_id_GET(Pack * src)//The ID of the requested data strea
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 1)));
}
INLINER void p67_stream_id_SET(uint8_t  src, Pack * dst)//The ID of the requested data strea
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  2);
}
INLINER uint8_t p67_on_off_GET(Pack * src)//1 stream is enabled, 0 stream is stopped
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  3, 1)));
}
INLINER void p67_on_off_SET(uint8_t  src, Pack * dst)//1 stream is enabled, 0 stream is stopped
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  3);
}
INLINER uint16_t p69_buttons_GET(Pack * src)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER void p69_buttons_SET(uint16_t  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER uint8_t p69_target_GET(Pack * src)//The system to be controlled
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 1)));
}
INLINER void p69_target_SET(uint8_t  src, Pack * dst)//The system to be controlled
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  2);
}
INLINER int16_t p69_x_GET(Pack * src)
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  3, 2)));
}
INLINER void p69_x_SET(int16_t  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  3);
}
INLINER int16_t p69_y_GET(Pack * src)
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  5, 2)));
}
INLINER void p69_y_SET(int16_t  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  5);
}
INLINER int16_t p69_z_GET(Pack * src)
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  7, 2)));
}
INLINER void p69_z_SET(int16_t  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  7);
}
INLINER int16_t p69_r_GET(Pack * src)
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  9, 2)));
}
INLINER void p69_r_SET(int16_t  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  9);
}
INLINER uint16_t p70_chan1_raw_GET(Pack * src)//RC channel 1 value, in microseconds. A value of UINT16_MAX means to ignore this field
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER void p70_chan1_raw_SET(uint16_t  src, Pack * dst)//RC channel 1 value, in microseconds. A value of UINT16_MAX means to ignore this field
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER uint16_t p70_chan2_raw_GET(Pack * src)//RC channel 2 value, in microseconds. A value of UINT16_MAX means to ignore this field
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 2)));
}
INLINER void p70_chan2_raw_SET(uint16_t  src, Pack * dst)//RC channel 2 value, in microseconds. A value of UINT16_MAX means to ignore this field
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  2);
}
INLINER uint16_t p70_chan3_raw_GET(Pack * src)//RC channel 3 value, in microseconds. A value of UINT16_MAX means to ignore this field
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  4, 2)));
}
INLINER void p70_chan3_raw_SET(uint16_t  src, Pack * dst)//RC channel 3 value, in microseconds. A value of UINT16_MAX means to ignore this field
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  4);
}
INLINER uint16_t p70_chan4_raw_GET(Pack * src)//RC channel 4 value, in microseconds. A value of UINT16_MAX means to ignore this field
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  6, 2)));
}
INLINER void p70_chan4_raw_SET(uint16_t  src, Pack * dst)//RC channel 4 value, in microseconds. A value of UINT16_MAX means to ignore this field
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  6);
}
INLINER uint16_t p70_chan5_raw_GET(Pack * src)//RC channel 5 value, in microseconds. A value of UINT16_MAX means to ignore this field
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  8, 2)));
}
INLINER void p70_chan5_raw_SET(uint16_t  src, Pack * dst)//RC channel 5 value, in microseconds. A value of UINT16_MAX means to ignore this field
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  8);
}
INLINER uint16_t p70_chan6_raw_GET(Pack * src)//RC channel 6 value, in microseconds. A value of UINT16_MAX means to ignore this field
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  10, 2)));
}
INLINER void p70_chan6_raw_SET(uint16_t  src, Pack * dst)//RC channel 6 value, in microseconds. A value of UINT16_MAX means to ignore this field
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  10);
}
INLINER uint16_t p70_chan7_raw_GET(Pack * src)//RC channel 7 value, in microseconds. A value of UINT16_MAX means to ignore this field
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  12, 2)));
}
INLINER void p70_chan7_raw_SET(uint16_t  src, Pack * dst)//RC channel 7 value, in microseconds. A value of UINT16_MAX means to ignore this field
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  12);
}
INLINER uint16_t p70_chan8_raw_GET(Pack * src)//RC channel 8 value, in microseconds. A value of UINT16_MAX means to ignore this field
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  14, 2)));
}
INLINER void p70_chan8_raw_SET(uint16_t  src, Pack * dst)//RC channel 8 value, in microseconds. A value of UINT16_MAX means to ignore this field
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  14);
}
INLINER uint8_t p70_target_system_GET(Pack * src)//System I
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  16, 1)));
}
INLINER void p70_target_system_SET(uint8_t  src, Pack * dst)//System I
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  16);
}
INLINER uint8_t p70_target_component_GET(Pack * src)//Component I
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  17, 1)));
}
INLINER void p70_target_component_SET(uint8_t  src, Pack * dst)//Component I
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  17);
}
INLINER uint16_t p73_seq_GET(Pack * src)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER void p73_seq_SET(uint16_t  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER uint8_t p73_target_system_GET(Pack * src)//System I
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 1)));
}
INLINER void p73_target_system_SET(uint8_t  src, Pack * dst)//System I
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  2);
}
INLINER uint8_t p73_target_component_GET(Pack * src)//Component I
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  3, 1)));
}
INLINER void p73_target_component_SET(uint8_t  src, Pack * dst)//Component I
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  3);
}
INLINER uint8_t p73_current_GET(Pack * src)//false:0, true:
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  4, 1)));
}
INLINER void p73_current_SET(uint8_t  src, Pack * dst)//false:0, true:
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  4);
}
INLINER uint8_t p73_autocontinue_GET(Pack * src)//autocontinue to next w
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  5, 1)));
}
INLINER void p73_autocontinue_SET(uint8_t  src, Pack * dst)//autocontinue to next w
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  5);
}
INLINER float p73_param1_GET(Pack * src)//PARAM1, see MAV_CMD enu
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  6, 4)));
}
INLINER void p73_param1_SET(float  src, Pack * dst)//PARAM1, see MAV_CMD enu
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  6);
}
INLINER float p73_param2_GET(Pack * src)//PARAM2, see MAV_CMD enu
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  10, 4)));
}
INLINER void p73_param2_SET(float  src, Pack * dst)//PARAM2, see MAV_CMD enu
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  10);
}
INLINER float p73_param3_GET(Pack * src)//PARAM3, see MAV_CMD enu
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  14, 4)));
}
INLINER void p73_param3_SET(float  src, Pack * dst)//PARAM3, see MAV_CMD enu
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  14);
}
INLINER float p73_param4_GET(Pack * src)//PARAM4, see MAV_CMD enu
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  18, 4)));
}
INLINER void p73_param4_SET(float  src, Pack * dst)//PARAM4, see MAV_CMD enu
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  18);
}
INLINER int32_t p73_x_GET(Pack * src)//PARAM5 / local: x position in meters * 1e4, global: latitude in degrees * 10^
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  22, 4)));
}
INLINER void p73_x_SET(int32_t  src, Pack * dst)//PARAM5 / local: x position in meters * 1e4, global: latitude in degrees * 10^
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  22);
}
INLINER int32_t p73_y_GET(Pack * src)//PARAM6 / y position: local: x position in meters * 1e4, global: longitude in degrees *10^
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  26, 4)));
}
INLINER void p73_y_SET(int32_t  src, Pack * dst)//PARAM6 / y position: local: x position in meters * 1e4, global: longitude in degrees *10^
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  26);
}
INLINER float p73_z_GET(Pack * src)//PARAM7 / z position: global: altitude in meters (relative or absolute, depending on frame
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  30, 4)));
}
INLINER void p73_z_SET(float  src, Pack * dst)//PARAM7 / z position: global: altitude in meters (relative or absolute, depending on frame
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  30);
}
INLINER e_MAV_FRAME p73_frame_GET(Pack * src)//The coordinate system of the waypoint. see MAV_FRAME in mavlink_types.
{
    uint8_t * data = src->data;
    return  0 + (int)get_bits(data, 272, 4);
}
INLINER void p73_frame_SET(e_MAV_FRAME  src, Pack * dst)//The coordinate system of the waypoint. see MAV_FRAME in mavlink_types.
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 4, data, 272);
}
INLINER e_MAV_CMD p73_command_GET(Pack * src)//The scheduled action for the waypoint. see MAV_CMD in common.xml MAVLink spec
{
    uint8_t * data = src->data;
    switch(get_bits(data, 276, 8))
    {
        case 0:
            return e_MAV_CMD_MAV_CMD_NAV_WAYPOINT;
        case 1:
            return e_MAV_CMD_MAV_CMD_NAV_LOITER_UNLIM;
        case 2:
            return e_MAV_CMD_MAV_CMD_NAV_LOITER_TURNS;
        case 3:
            return e_MAV_CMD_MAV_CMD_NAV_LOITER_TIME;
        case 4:
            return e_MAV_CMD_MAV_CMD_NAV_RETURN_TO_LAUNCH;
        case 5:
            return e_MAV_CMD_MAV_CMD_NAV_LAND;
        case 6:
            return e_MAV_CMD_MAV_CMD_NAV_TAKEOFF;
        case 7:
            return e_MAV_CMD_MAV_CMD_NAV_LAND_LOCAL;
        case 8:
            return e_MAV_CMD_MAV_CMD_NAV_TAKEOFF_LOCAL;
        case 9:
            return e_MAV_CMD_MAV_CMD_NAV_FOLLOW;
        case 10:
            return e_MAV_CMD_MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT;
        case 11:
            return e_MAV_CMD_MAV_CMD_NAV_LOITER_TO_ALT;
        case 12:
            return e_MAV_CMD_MAV_CMD_DO_FOLLOW;
        case 13:
            return e_MAV_CMD_MAV_CMD_DO_FOLLOW_REPOSITION;
        case 14:
            return e_MAV_CMD_MAV_CMD_NAV_ROI;
        case 15:
            return e_MAV_CMD_MAV_CMD_NAV_PATHPLANNING;
        case 16:
            return e_MAV_CMD_MAV_CMD_NAV_SPLINE_WAYPOINT;
        case 17:
            return e_MAV_CMD_MAV_CMD_NAV_VTOL_TAKEOFF;
        case 18:
            return e_MAV_CMD_MAV_CMD_NAV_VTOL_LAND;
        case 19:
            return e_MAV_CMD_MAV_CMD_NAV_GUIDED_ENABLE;
        case 20:
            return e_MAV_CMD_MAV_CMD_NAV_DELAY;
        case 21:
            return e_MAV_CMD_MAV_CMD_NAV_PAYLOAD_PLACE;
        case 22:
            return e_MAV_CMD_MAV_CMD_NAV_LAST;
        case 23:
            return e_MAV_CMD_MAV_CMD_CONDITION_DELAY;
        case 24:
            return e_MAV_CMD_MAV_CMD_CONDITION_CHANGE_ALT;
        case 25:
            return e_MAV_CMD_MAV_CMD_CONDITION_DISTANCE;
        case 26:
            return e_MAV_CMD_MAV_CMD_CONDITION_YAW;
        case 27:
            return e_MAV_CMD_MAV_CMD_CONDITION_LAST;
        case 28:
            return e_MAV_CMD_MAV_CMD_DO_SET_MODE;
        case 29:
            return e_MAV_CMD_MAV_CMD_DO_JUMP;
        case 30:
            return e_MAV_CMD_MAV_CMD_DO_CHANGE_SPEED;
        case 31:
            return e_MAV_CMD_MAV_CMD_DO_SET_HOME;
        case 32:
            return e_MAV_CMD_MAV_CMD_DO_SET_PARAMETER;
        case 33:
            return e_MAV_CMD_MAV_CMD_DO_SET_RELAY;
        case 34:
            return e_MAV_CMD_MAV_CMD_DO_REPEAT_RELAY;
        case 35:
            return e_MAV_CMD_MAV_CMD_DO_SET_SERVO;
        case 36:
            return e_MAV_CMD_MAV_CMD_DO_REPEAT_SERVO;
        case 37:
            return e_MAV_CMD_MAV_CMD_DO_FLIGHTTERMINATION;
        case 38:
            return e_MAV_CMD_MAV_CMD_DO_CHANGE_ALTITUDE;
        case 39:
            return e_MAV_CMD_MAV_CMD_DO_LAND_START;
        case 40:
            return e_MAV_CMD_MAV_CMD_DO_RALLY_LAND;
        case 41:
            return e_MAV_CMD_MAV_CMD_DO_GO_AROUND;
        case 42:
            return e_MAV_CMD_MAV_CMD_DO_REPOSITION;
        case 43:
            return e_MAV_CMD_MAV_CMD_DO_PAUSE_CONTINUE;
        case 44:
            return e_MAV_CMD_MAV_CMD_DO_SET_REVERSE;
        case 45:
            return e_MAV_CMD_MAV_CMD_DO_CONTROL_VIDEO;
        case 46:
            return e_MAV_CMD_MAV_CMD_DO_SET_ROI;
        case 47:
            return e_MAV_CMD_MAV_CMD_DO_DIGICAM_CONFIGURE;
        case 48:
            return e_MAV_CMD_MAV_CMD_DO_DIGICAM_CONTROL;
        case 49:
            return e_MAV_CMD_MAV_CMD_DO_MOUNT_CONFIGURE;
        case 50:
            return e_MAV_CMD_MAV_CMD_DO_MOUNT_CONTROL;
        case 51:
            return e_MAV_CMD_MAV_CMD_DO_SET_CAM_TRIGG_DIST;
        case 52:
            return e_MAV_CMD_MAV_CMD_DO_FENCE_ENABLE;
        case 53:
            return e_MAV_CMD_MAV_CMD_DO_PARACHUTE;
        case 54:
            return e_MAV_CMD_MAV_CMD_DO_MOTOR_TEST;
        case 55:
            return e_MAV_CMD_MAV_CMD_DO_INVERTED_FLIGHT;
        case 56:
            return e_MAV_CMD_MAV_CMD_NAV_SET_YAW_SPEED;
        case 57:
            return e_MAV_CMD_MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL;
        case 58:
            return e_MAV_CMD_MAV_CMD_DO_MOUNT_CONTROL_QUAT;
        case 59:
            return e_MAV_CMD_MAV_CMD_DO_GUIDED_MASTER;
        case 60:
            return e_MAV_CMD_MAV_CMD_DO_GUIDED_LIMITS;
        case 61:
            return e_MAV_CMD_MAV_CMD_DO_ENGINE_CONTROL;
        case 62:
            return e_MAV_CMD_MAV_CMD_DO_LAST;
        case 63:
            return e_MAV_CMD_MAV_CMD_PREFLIGHT_CALIBRATION;
        case 64:
            return e_MAV_CMD_MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS;
        case 65:
            return e_MAV_CMD_MAV_CMD_PREFLIGHT_UAVCAN;
        case 66:
            return e_MAV_CMD_MAV_CMD_PREFLIGHT_STORAGE;
        case 67:
            return e_MAV_CMD_MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN;
        case 68:
            return e_MAV_CMD_MAV_CMD_OVERRIDE_GOTO;
        case 69:
            return e_MAV_CMD_MAV_CMD_MISSION_START;
        case 70:
            return e_MAV_CMD_MAV_CMD_COMPONENT_ARM_DISARM;
        case 71:
            return e_MAV_CMD_MAV_CMD_GET_HOME_POSITION;
        case 72:
            return e_MAV_CMD_MAV_CMD_START_RX_PAIR;
        case 73:
            return e_MAV_CMD_MAV_CMD_GET_MESSAGE_INTERVAL;
        case 74:
            return e_MAV_CMD_MAV_CMD_SET_MESSAGE_INTERVAL;
        case 75:
            return e_MAV_CMD_MAV_CMD_REQUEST_PROTOCOL_VERSION;
        case 76:
            return e_MAV_CMD_MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES;
        case 77:
            return e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_INFORMATION;
        case 78:
            return e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_SETTINGS;
        case 79:
            return e_MAV_CMD_MAV_CMD_REQUEST_STORAGE_INFORMATION;
        case 80:
            return e_MAV_CMD_MAV_CMD_STORAGE_FORMAT;
        case 81:
            return e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS;
        case 82:
            return e_MAV_CMD_MAV_CMD_REQUEST_FLIGHT_INFORMATION;
        case 83:
            return e_MAV_CMD_MAV_CMD_RESET_CAMERA_SETTINGS;
        case 84:
            return e_MAV_CMD_MAV_CMD_SET_CAMERA_MODE;
        case 85:
            return e_MAV_CMD_MAV_CMD_IMAGE_START_CAPTURE;
        case 86:
            return e_MAV_CMD_MAV_CMD_IMAGE_STOP_CAPTURE;
        case 87:
            return e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE;
        case 88:
            return e_MAV_CMD_MAV_CMD_DO_TRIGGER_CONTROL;
        case 89:
            return e_MAV_CMD_MAV_CMD_VIDEO_START_CAPTURE;
        case 90:
            return e_MAV_CMD_MAV_CMD_VIDEO_STOP_CAPTURE;
        case 91:
            return e_MAV_CMD_MAV_CMD_VIDEO_START_STREAMING;
        case 92:
            return e_MAV_CMD_MAV_CMD_VIDEO_STOP_STREAMING;
        case 93:
            return e_MAV_CMD_MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION;
        case 94:
            return e_MAV_CMD_MAV_CMD_LOGGING_START;
        case 95:
            return e_MAV_CMD_MAV_CMD_LOGGING_STOP;
        case 96:
            return e_MAV_CMD_MAV_CMD_AIRFRAME_CONFIGURATION;
        case 97:
            return e_MAV_CMD_MAV_CMD_PANORAMA_CREATE;
        case 98:
            return e_MAV_CMD_MAV_CMD_DO_VTOL_TRANSITION;
        case 99:
            return e_MAV_CMD_MAV_CMD_ARM_AUTHORIZATION_REQUEST;
        case 100:
            return e_MAV_CMD_MAV_CMD_SET_GUIDED_SUBMODE_STANDARD;
        case 101:
            return e_MAV_CMD_MAV_CMD_SET_GUIDED_SUBMODE_CIRCLE;
        case 102:
            return e_MAV_CMD_MAV_CMD_CONDITION_GATE;
        case 103:
            return e_MAV_CMD_MAV_CMD_NAV_FENCE_RETURN_POINT;
        case 104:
            return e_MAV_CMD_MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION;
        case 105:
            return e_MAV_CMD_MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION;
        case 106:
            return e_MAV_CMD_MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION;
        case 107:
            return e_MAV_CMD_MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION;
        case 108:
            return e_MAV_CMD_MAV_CMD_NAV_RALLY_POINT;
        case 109:
            return e_MAV_CMD_MAV_CMD_UAVCAN_GET_NODE_INFO;
        case 110:
            return e_MAV_CMD_MAV_CMD_PAYLOAD_PREPARE_DEPLOY;
        case 111:
            return e_MAV_CMD_MAV_CMD_PAYLOAD_CONTROL_DEPLOY;
        case 112:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_1;
        case 113:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_2;
        case 114:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_3;
        case 115:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_4;
        case 116:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_5;
        case 117:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_1;
        case 118:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_2;
        case 119:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_3;
        case 120:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_4;
        case 121:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_5;
        case 122:
            return e_MAV_CMD_MAV_CMD_USER_1;
        case 123:
            return e_MAV_CMD_MAV_CMD_USER_2;
        case 124:
            return e_MAV_CMD_MAV_CMD_USER_3;
        case 125:
            return e_MAV_CMD_MAV_CMD_USER_4;
        case 126:
            return e_MAV_CMD_MAV_CMD_USER_5;
        case 127:
            return e_MAV_CMD_MAV_CMD_RESET_MPPT;
        case 128:
            return e_MAV_CMD_MAV_CMD_PAYLOAD_CONTROL;
        default: ;//assert(false);//("Unknown enum ID " + id);
    }
}
INLINER void p73_command_SET(e_MAV_CMD  src, Pack * dst)//The scheduled action for the waypoint. see MAV_CMD in common.xml MAVLink spec
{
    uint8_t * data = dst->data;
    int32_t id;
    switch(src)
    {
        case e_MAV_CMD_MAV_CMD_NAV_WAYPOINT:
            id = 0;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_LOITER_UNLIM:
            id = 1;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_LOITER_TURNS:
            id = 2;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_LOITER_TIME:
            id = 3;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_RETURN_TO_LAUNCH:
            id = 4;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_LAND:
            id = 5;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_TAKEOFF:
            id = 6;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_LAND_LOCAL:
            id = 7;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_TAKEOFF_LOCAL:
            id = 8;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_FOLLOW:
            id = 9;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT:
            id = 10;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_LOITER_TO_ALT:
            id = 11;
            break;
        case e_MAV_CMD_MAV_CMD_DO_FOLLOW:
            id = 12;
            break;
        case e_MAV_CMD_MAV_CMD_DO_FOLLOW_REPOSITION:
            id = 13;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_ROI:
            id = 14;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_PATHPLANNING:
            id = 15;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_SPLINE_WAYPOINT:
            id = 16;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_VTOL_TAKEOFF:
            id = 17;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_VTOL_LAND:
            id = 18;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_GUIDED_ENABLE:
            id = 19;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_DELAY:
            id = 20;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_PAYLOAD_PLACE:
            id = 21;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_LAST:
            id = 22;
            break;
        case e_MAV_CMD_MAV_CMD_CONDITION_DELAY:
            id = 23;
            break;
        case e_MAV_CMD_MAV_CMD_CONDITION_CHANGE_ALT:
            id = 24;
            break;
        case e_MAV_CMD_MAV_CMD_CONDITION_DISTANCE:
            id = 25;
            break;
        case e_MAV_CMD_MAV_CMD_CONDITION_YAW:
            id = 26;
            break;
        case e_MAV_CMD_MAV_CMD_CONDITION_LAST:
            id = 27;
            break;
        case e_MAV_CMD_MAV_CMD_DO_SET_MODE:
            id = 28;
            break;
        case e_MAV_CMD_MAV_CMD_DO_JUMP:
            id = 29;
            break;
        case e_MAV_CMD_MAV_CMD_DO_CHANGE_SPEED:
            id = 30;
            break;
        case e_MAV_CMD_MAV_CMD_DO_SET_HOME:
            id = 31;
            break;
        case e_MAV_CMD_MAV_CMD_DO_SET_PARAMETER:
            id = 32;
            break;
        case e_MAV_CMD_MAV_CMD_DO_SET_RELAY:
            id = 33;
            break;
        case e_MAV_CMD_MAV_CMD_DO_REPEAT_RELAY:
            id = 34;
            break;
        case e_MAV_CMD_MAV_CMD_DO_SET_SERVO:
            id = 35;
            break;
        case e_MAV_CMD_MAV_CMD_DO_REPEAT_SERVO:
            id = 36;
            break;
        case e_MAV_CMD_MAV_CMD_DO_FLIGHTTERMINATION:
            id = 37;
            break;
        case e_MAV_CMD_MAV_CMD_DO_CHANGE_ALTITUDE:
            id = 38;
            break;
        case e_MAV_CMD_MAV_CMD_DO_LAND_START:
            id = 39;
            break;
        case e_MAV_CMD_MAV_CMD_DO_RALLY_LAND:
            id = 40;
            break;
        case e_MAV_CMD_MAV_CMD_DO_GO_AROUND:
            id = 41;
            break;
        case e_MAV_CMD_MAV_CMD_DO_REPOSITION:
            id = 42;
            break;
        case e_MAV_CMD_MAV_CMD_DO_PAUSE_CONTINUE:
            id = 43;
            break;
        case e_MAV_CMD_MAV_CMD_DO_SET_REVERSE:
            id = 44;
            break;
        case e_MAV_CMD_MAV_CMD_DO_CONTROL_VIDEO:
            id = 45;
            break;
        case e_MAV_CMD_MAV_CMD_DO_SET_ROI:
            id = 46;
            break;
        case e_MAV_CMD_MAV_CMD_DO_DIGICAM_CONFIGURE:
            id = 47;
            break;
        case e_MAV_CMD_MAV_CMD_DO_DIGICAM_CONTROL:
            id = 48;
            break;
        case e_MAV_CMD_MAV_CMD_DO_MOUNT_CONFIGURE:
            id = 49;
            break;
        case e_MAV_CMD_MAV_CMD_DO_MOUNT_CONTROL:
            id = 50;
            break;
        case e_MAV_CMD_MAV_CMD_DO_SET_CAM_TRIGG_DIST:
            id = 51;
            break;
        case e_MAV_CMD_MAV_CMD_DO_FENCE_ENABLE:
            id = 52;
            break;
        case e_MAV_CMD_MAV_CMD_DO_PARACHUTE:
            id = 53;
            break;
        case e_MAV_CMD_MAV_CMD_DO_MOTOR_TEST:
            id = 54;
            break;
        case e_MAV_CMD_MAV_CMD_DO_INVERTED_FLIGHT:
            id = 55;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_SET_YAW_SPEED:
            id = 56;
            break;
        case e_MAV_CMD_MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL:
            id = 57;
            break;
        case e_MAV_CMD_MAV_CMD_DO_MOUNT_CONTROL_QUAT:
            id = 58;
            break;
        case e_MAV_CMD_MAV_CMD_DO_GUIDED_MASTER:
            id = 59;
            break;
        case e_MAV_CMD_MAV_CMD_DO_GUIDED_LIMITS:
            id = 60;
            break;
        case e_MAV_CMD_MAV_CMD_DO_ENGINE_CONTROL:
            id = 61;
            break;
        case e_MAV_CMD_MAV_CMD_DO_LAST:
            id = 62;
            break;
        case e_MAV_CMD_MAV_CMD_PREFLIGHT_CALIBRATION:
            id = 63;
            break;
        case e_MAV_CMD_MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS:
            id = 64;
            break;
        case e_MAV_CMD_MAV_CMD_PREFLIGHT_UAVCAN:
            id = 65;
            break;
        case e_MAV_CMD_MAV_CMD_PREFLIGHT_STORAGE:
            id = 66;
            break;
        case e_MAV_CMD_MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
            id = 67;
            break;
        case e_MAV_CMD_MAV_CMD_OVERRIDE_GOTO:
            id = 68;
            break;
        case e_MAV_CMD_MAV_CMD_MISSION_START:
            id = 69;
            break;
        case e_MAV_CMD_MAV_CMD_COMPONENT_ARM_DISARM:
            id = 70;
            break;
        case e_MAV_CMD_MAV_CMD_GET_HOME_POSITION:
            id = 71;
            break;
        case e_MAV_CMD_MAV_CMD_START_RX_PAIR:
            id = 72;
            break;
        case e_MAV_CMD_MAV_CMD_GET_MESSAGE_INTERVAL:
            id = 73;
            break;
        case e_MAV_CMD_MAV_CMD_SET_MESSAGE_INTERVAL:
            id = 74;
            break;
        case e_MAV_CMD_MAV_CMD_REQUEST_PROTOCOL_VERSION:
            id = 75;
            break;
        case e_MAV_CMD_MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES:
            id = 76;
            break;
        case e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_INFORMATION:
            id = 77;
            break;
        case e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_SETTINGS:
            id = 78;
            break;
        case e_MAV_CMD_MAV_CMD_REQUEST_STORAGE_INFORMATION:
            id = 79;
            break;
        case e_MAV_CMD_MAV_CMD_STORAGE_FORMAT:
            id = 80;
            break;
        case e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS:
            id = 81;
            break;
        case e_MAV_CMD_MAV_CMD_REQUEST_FLIGHT_INFORMATION:
            id = 82;
            break;
        case e_MAV_CMD_MAV_CMD_RESET_CAMERA_SETTINGS:
            id = 83;
            break;
        case e_MAV_CMD_MAV_CMD_SET_CAMERA_MODE:
            id = 84;
            break;
        case e_MAV_CMD_MAV_CMD_IMAGE_START_CAPTURE:
            id = 85;
            break;
        case e_MAV_CMD_MAV_CMD_IMAGE_STOP_CAPTURE:
            id = 86;
            break;
        case e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE:
            id = 87;
            break;
        case e_MAV_CMD_MAV_CMD_DO_TRIGGER_CONTROL:
            id = 88;
            break;
        case e_MAV_CMD_MAV_CMD_VIDEO_START_CAPTURE:
            id = 89;
            break;
        case e_MAV_CMD_MAV_CMD_VIDEO_STOP_CAPTURE:
            id = 90;
            break;
        case e_MAV_CMD_MAV_CMD_VIDEO_START_STREAMING:
            id = 91;
            break;
        case e_MAV_CMD_MAV_CMD_VIDEO_STOP_STREAMING:
            id = 92;
            break;
        case e_MAV_CMD_MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION:
            id = 93;
            break;
        case e_MAV_CMD_MAV_CMD_LOGGING_START:
            id = 94;
            break;
        case e_MAV_CMD_MAV_CMD_LOGGING_STOP:
            id = 95;
            break;
        case e_MAV_CMD_MAV_CMD_AIRFRAME_CONFIGURATION:
            id = 96;
            break;
        case e_MAV_CMD_MAV_CMD_PANORAMA_CREATE:
            id = 97;
            break;
        case e_MAV_CMD_MAV_CMD_DO_VTOL_TRANSITION:
            id = 98;
            break;
        case e_MAV_CMD_MAV_CMD_ARM_AUTHORIZATION_REQUEST:
            id = 99;
            break;
        case e_MAV_CMD_MAV_CMD_SET_GUIDED_SUBMODE_STANDARD:
            id = 100;
            break;
        case e_MAV_CMD_MAV_CMD_SET_GUIDED_SUBMODE_CIRCLE:
            id = 101;
            break;
        case e_MAV_CMD_MAV_CMD_CONDITION_GATE:
            id = 102;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_FENCE_RETURN_POINT:
            id = 103;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION:
            id = 104;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION:
            id = 105;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION:
            id = 106;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION:
            id = 107;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_RALLY_POINT:
            id = 108;
            break;
        case e_MAV_CMD_MAV_CMD_UAVCAN_GET_NODE_INFO:
            id = 109;
            break;
        case e_MAV_CMD_MAV_CMD_PAYLOAD_PREPARE_DEPLOY:
            id = 110;
            break;
        case e_MAV_CMD_MAV_CMD_PAYLOAD_CONTROL_DEPLOY:
            id = 111;
            break;
        case e_MAV_CMD_MAV_CMD_WAYPOINT_USER_1:
            id = 112;
            break;
        case e_MAV_CMD_MAV_CMD_WAYPOINT_USER_2:
            id = 113;
            break;
        case e_MAV_CMD_MAV_CMD_WAYPOINT_USER_3:
            id = 114;
            break;
        case e_MAV_CMD_MAV_CMD_WAYPOINT_USER_4:
            id = 115;
            break;
        case e_MAV_CMD_MAV_CMD_WAYPOINT_USER_5:
            id = 116;
            break;
        case e_MAV_CMD_MAV_CMD_SPATIAL_USER_1:
            id = 117;
            break;
        case e_MAV_CMD_MAV_CMD_SPATIAL_USER_2:
            id = 118;
            break;
        case e_MAV_CMD_MAV_CMD_SPATIAL_USER_3:
            id = 119;
            break;
        case e_MAV_CMD_MAV_CMD_SPATIAL_USER_4:
            id = 120;
            break;
        case e_MAV_CMD_MAV_CMD_SPATIAL_USER_5:
            id = 121;
            break;
        case e_MAV_CMD_MAV_CMD_USER_1:
            id = 122;
            break;
        case e_MAV_CMD_MAV_CMD_USER_2:
            id = 123;
            break;
        case e_MAV_CMD_MAV_CMD_USER_3:
            id = 124;
            break;
        case e_MAV_CMD_MAV_CMD_USER_4:
            id = 125;
            break;
        case e_MAV_CMD_MAV_CMD_USER_5:
            id = 126;
            break;
        case e_MAV_CMD_MAV_CMD_RESET_MPPT:
            id = 127;
            break;
        case e_MAV_CMD_MAV_CMD_PAYLOAD_CONTROL:
            id = 128;
            break;
        default: ;// assert(false);//("Unknown enum" + id);
    }
    set_bits(id, 8, data, 276);
}
INLINER e_MAV_MISSION_TYPE p73_mission_type_GET(Pack * src)//Mission type, see MAV_MISSION_TYP
{
    uint8_t * data = src->data;
    switch(get_bits(data, 284, 3))
    {
        case 0:
            return e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION;
        case 1:
            return e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE;
        case 2:
            return e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY;
        case 3:
            return e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL;
        default: ;//assert(false);//("Unknown enum ID " + id);
    }
}
INLINER void p73_mission_type_SET(e_MAV_MISSION_TYPE  src, Pack * dst)//Mission type, see MAV_MISSION_TYP
{
    uint8_t * data = dst->data;
    int32_t id;
    switch(src)
    {
        case e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION:
            id = 0;
            break;
        case e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE:
            id = 1;
            break;
        case e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY:
            id = 2;
            break;
        case e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL:
            id = 3;
            break;
        default: ;// assert(false);//("Unknown enum" + id);
    }
    set_bits(id, 3, data, 284);
}
INLINER uint16_t p74_throttle_GET(Pack * src)//Current throttle setting in integer percent, 0 to 10
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER void p74_throttle_SET(uint16_t  src, Pack * dst)//Current throttle setting in integer percent, 0 to 10
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER float p74_airspeed_GET(Pack * src)//Current airspeed in m/
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  2, 4)));
}
INLINER void p74_airspeed_SET(float  src, Pack * dst)//Current airspeed in m/
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  2);
}
INLINER float p74_groundspeed_GET(Pack * src)//Current ground speed in m/
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  6, 4)));
}
INLINER void p74_groundspeed_SET(float  src, Pack * dst)//Current ground speed in m/
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  6);
}
INLINER int16_t p74_heading_GET(Pack * src)//Current heading in degrees, in compass units (0..360, 0=north
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  10, 2)));
}
INLINER void p74_heading_SET(int16_t  src, Pack * dst)//Current heading in degrees, in compass units (0..360, 0=north
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  10);
}
INLINER float p74_alt_GET(Pack * src)//Current altitude (MSL), in meter
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  12, 4)));
}
INLINER void p74_alt_SET(float  src, Pack * dst)//Current altitude (MSL), in meter
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  12);
}
INLINER float p74_climb_GET(Pack * src)//Current climb rate in meters/secon
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  16, 4)));
}
INLINER void p74_climb_SET(float  src, Pack * dst)//Current climb rate in meters/secon
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  16);
}
INLINER uint8_t p75_target_system_GET(Pack * src)//System I
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 1)));
}
INLINER void p75_target_system_SET(uint8_t  src, Pack * dst)//System I
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  0);
}
INLINER uint8_t p75_target_component_GET(Pack * src)//Component I
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  1, 1)));
}
INLINER void p75_target_component_SET(uint8_t  src, Pack * dst)//Component I
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  1);
}
INLINER uint8_t p75_current_GET(Pack * src)//false:0, true:
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 1)));
}
INLINER void p75_current_SET(uint8_t  src, Pack * dst)//false:0, true:
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  2);
}
INLINER uint8_t p75_autocontinue_GET(Pack * src)//autocontinue to next w
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  3, 1)));
}
INLINER void p75_autocontinue_SET(uint8_t  src, Pack * dst)//autocontinue to next w
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  3);
}
INLINER float p75_param1_GET(Pack * src)//PARAM1, see MAV_CMD enu
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  4, 4)));
}
INLINER void p75_param1_SET(float  src, Pack * dst)//PARAM1, see MAV_CMD enu
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  4);
}
INLINER float p75_param2_GET(Pack * src)//PARAM2, see MAV_CMD enu
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  8, 4)));
}
INLINER void p75_param2_SET(float  src, Pack * dst)//PARAM2, see MAV_CMD enu
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  8);
}
INLINER float p75_param3_GET(Pack * src)//PARAM3, see MAV_CMD enu
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  12, 4)));
}
INLINER void p75_param3_SET(float  src, Pack * dst)//PARAM3, see MAV_CMD enu
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  12);
}
INLINER float p75_param4_GET(Pack * src)//PARAM4, see MAV_CMD enu
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  16, 4)));
}
INLINER void p75_param4_SET(float  src, Pack * dst)//PARAM4, see MAV_CMD enu
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  16);
}
INLINER int32_t p75_x_GET(Pack * src)//PARAM5 / local: x position in meters * 1e4, global: latitude in degrees * 10^
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  20, 4)));
}
INLINER void p75_x_SET(int32_t  src, Pack * dst)//PARAM5 / local: x position in meters * 1e4, global: latitude in degrees * 10^
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  20);
}
INLINER int32_t p75_y_GET(Pack * src)//PARAM6 / local: y position in meters * 1e4, global: longitude in degrees * 10^
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  24, 4)));
}
INLINER void p75_y_SET(int32_t  src, Pack * dst)//PARAM6 / local: y position in meters * 1e4, global: longitude in degrees * 10^
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  24);
}
INLINER float p75_z_GET(Pack * src)//PARAM7 / z position: global: altitude in meters (relative or absolute, depending on frame
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  28, 4)));
}
INLINER void p75_z_SET(float  src, Pack * dst)//PARAM7 / z position: global: altitude in meters (relative or absolute, depending on frame
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  28);
}
INLINER e_MAV_FRAME p75_frame_GET(Pack * src)//The coordinate system of the COMMAND. see MAV_FRAME in mavlink_types.
{
    uint8_t * data = src->data;
    return  0 + (int)get_bits(data, 256, 4);
}
INLINER void p75_frame_SET(e_MAV_FRAME  src, Pack * dst)//The coordinate system of the COMMAND. see MAV_FRAME in mavlink_types.
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 4, data, 256);
}
INLINER e_MAV_CMD p75_command_GET(Pack * src)//The scheduled action for the mission item. see MAV_CMD in common.xml MAVLink spec
{
    uint8_t * data = src->data;
    switch(get_bits(data, 260, 8))
    {
        case 0:
            return e_MAV_CMD_MAV_CMD_NAV_WAYPOINT;
        case 1:
            return e_MAV_CMD_MAV_CMD_NAV_LOITER_UNLIM;
        case 2:
            return e_MAV_CMD_MAV_CMD_NAV_LOITER_TURNS;
        case 3:
            return e_MAV_CMD_MAV_CMD_NAV_LOITER_TIME;
        case 4:
            return e_MAV_CMD_MAV_CMD_NAV_RETURN_TO_LAUNCH;
        case 5:
            return e_MAV_CMD_MAV_CMD_NAV_LAND;
        case 6:
            return e_MAV_CMD_MAV_CMD_NAV_TAKEOFF;
        case 7:
            return e_MAV_CMD_MAV_CMD_NAV_LAND_LOCAL;
        case 8:
            return e_MAV_CMD_MAV_CMD_NAV_TAKEOFF_LOCAL;
        case 9:
            return e_MAV_CMD_MAV_CMD_NAV_FOLLOW;
        case 10:
            return e_MAV_CMD_MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT;
        case 11:
            return e_MAV_CMD_MAV_CMD_NAV_LOITER_TO_ALT;
        case 12:
            return e_MAV_CMD_MAV_CMD_DO_FOLLOW;
        case 13:
            return e_MAV_CMD_MAV_CMD_DO_FOLLOW_REPOSITION;
        case 14:
            return e_MAV_CMD_MAV_CMD_NAV_ROI;
        case 15:
            return e_MAV_CMD_MAV_CMD_NAV_PATHPLANNING;
        case 16:
            return e_MAV_CMD_MAV_CMD_NAV_SPLINE_WAYPOINT;
        case 17:
            return e_MAV_CMD_MAV_CMD_NAV_VTOL_TAKEOFF;
        case 18:
            return e_MAV_CMD_MAV_CMD_NAV_VTOL_LAND;
        case 19:
            return e_MAV_CMD_MAV_CMD_NAV_GUIDED_ENABLE;
        case 20:
            return e_MAV_CMD_MAV_CMD_NAV_DELAY;
        case 21:
            return e_MAV_CMD_MAV_CMD_NAV_PAYLOAD_PLACE;
        case 22:
            return e_MAV_CMD_MAV_CMD_NAV_LAST;
        case 23:
            return e_MAV_CMD_MAV_CMD_CONDITION_DELAY;
        case 24:
            return e_MAV_CMD_MAV_CMD_CONDITION_CHANGE_ALT;
        case 25:
            return e_MAV_CMD_MAV_CMD_CONDITION_DISTANCE;
        case 26:
            return e_MAV_CMD_MAV_CMD_CONDITION_YAW;
        case 27:
            return e_MAV_CMD_MAV_CMD_CONDITION_LAST;
        case 28:
            return e_MAV_CMD_MAV_CMD_DO_SET_MODE;
        case 29:
            return e_MAV_CMD_MAV_CMD_DO_JUMP;
        case 30:
            return e_MAV_CMD_MAV_CMD_DO_CHANGE_SPEED;
        case 31:
            return e_MAV_CMD_MAV_CMD_DO_SET_HOME;
        case 32:
            return e_MAV_CMD_MAV_CMD_DO_SET_PARAMETER;
        case 33:
            return e_MAV_CMD_MAV_CMD_DO_SET_RELAY;
        case 34:
            return e_MAV_CMD_MAV_CMD_DO_REPEAT_RELAY;
        case 35:
            return e_MAV_CMD_MAV_CMD_DO_SET_SERVO;
        case 36:
            return e_MAV_CMD_MAV_CMD_DO_REPEAT_SERVO;
        case 37:
            return e_MAV_CMD_MAV_CMD_DO_FLIGHTTERMINATION;
        case 38:
            return e_MAV_CMD_MAV_CMD_DO_CHANGE_ALTITUDE;
        case 39:
            return e_MAV_CMD_MAV_CMD_DO_LAND_START;
        case 40:
            return e_MAV_CMD_MAV_CMD_DO_RALLY_LAND;
        case 41:
            return e_MAV_CMD_MAV_CMD_DO_GO_AROUND;
        case 42:
            return e_MAV_CMD_MAV_CMD_DO_REPOSITION;
        case 43:
            return e_MAV_CMD_MAV_CMD_DO_PAUSE_CONTINUE;
        case 44:
            return e_MAV_CMD_MAV_CMD_DO_SET_REVERSE;
        case 45:
            return e_MAV_CMD_MAV_CMD_DO_CONTROL_VIDEO;
        case 46:
            return e_MAV_CMD_MAV_CMD_DO_SET_ROI;
        case 47:
            return e_MAV_CMD_MAV_CMD_DO_DIGICAM_CONFIGURE;
        case 48:
            return e_MAV_CMD_MAV_CMD_DO_DIGICAM_CONTROL;
        case 49:
            return e_MAV_CMD_MAV_CMD_DO_MOUNT_CONFIGURE;
        case 50:
            return e_MAV_CMD_MAV_CMD_DO_MOUNT_CONTROL;
        case 51:
            return e_MAV_CMD_MAV_CMD_DO_SET_CAM_TRIGG_DIST;
        case 52:
            return e_MAV_CMD_MAV_CMD_DO_FENCE_ENABLE;
        case 53:
            return e_MAV_CMD_MAV_CMD_DO_PARACHUTE;
        case 54:
            return e_MAV_CMD_MAV_CMD_DO_MOTOR_TEST;
        case 55:
            return e_MAV_CMD_MAV_CMD_DO_INVERTED_FLIGHT;
        case 56:
            return e_MAV_CMD_MAV_CMD_NAV_SET_YAW_SPEED;
        case 57:
            return e_MAV_CMD_MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL;
        case 58:
            return e_MAV_CMD_MAV_CMD_DO_MOUNT_CONTROL_QUAT;
        case 59:
            return e_MAV_CMD_MAV_CMD_DO_GUIDED_MASTER;
        case 60:
            return e_MAV_CMD_MAV_CMD_DO_GUIDED_LIMITS;
        case 61:
            return e_MAV_CMD_MAV_CMD_DO_ENGINE_CONTROL;
        case 62:
            return e_MAV_CMD_MAV_CMD_DO_LAST;
        case 63:
            return e_MAV_CMD_MAV_CMD_PREFLIGHT_CALIBRATION;
        case 64:
            return e_MAV_CMD_MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS;
        case 65:
            return e_MAV_CMD_MAV_CMD_PREFLIGHT_UAVCAN;
        case 66:
            return e_MAV_CMD_MAV_CMD_PREFLIGHT_STORAGE;
        case 67:
            return e_MAV_CMD_MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN;
        case 68:
            return e_MAV_CMD_MAV_CMD_OVERRIDE_GOTO;
        case 69:
            return e_MAV_CMD_MAV_CMD_MISSION_START;
        case 70:
            return e_MAV_CMD_MAV_CMD_COMPONENT_ARM_DISARM;
        case 71:
            return e_MAV_CMD_MAV_CMD_GET_HOME_POSITION;
        case 72:
            return e_MAV_CMD_MAV_CMD_START_RX_PAIR;
        case 73:
            return e_MAV_CMD_MAV_CMD_GET_MESSAGE_INTERVAL;
        case 74:
            return e_MAV_CMD_MAV_CMD_SET_MESSAGE_INTERVAL;
        case 75:
            return e_MAV_CMD_MAV_CMD_REQUEST_PROTOCOL_VERSION;
        case 76:
            return e_MAV_CMD_MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES;
        case 77:
            return e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_INFORMATION;
        case 78:
            return e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_SETTINGS;
        case 79:
            return e_MAV_CMD_MAV_CMD_REQUEST_STORAGE_INFORMATION;
        case 80:
            return e_MAV_CMD_MAV_CMD_STORAGE_FORMAT;
        case 81:
            return e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS;
        case 82:
            return e_MAV_CMD_MAV_CMD_REQUEST_FLIGHT_INFORMATION;
        case 83:
            return e_MAV_CMD_MAV_CMD_RESET_CAMERA_SETTINGS;
        case 84:
            return e_MAV_CMD_MAV_CMD_SET_CAMERA_MODE;
        case 85:
            return e_MAV_CMD_MAV_CMD_IMAGE_START_CAPTURE;
        case 86:
            return e_MAV_CMD_MAV_CMD_IMAGE_STOP_CAPTURE;
        case 87:
            return e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE;
        case 88:
            return e_MAV_CMD_MAV_CMD_DO_TRIGGER_CONTROL;
        case 89:
            return e_MAV_CMD_MAV_CMD_VIDEO_START_CAPTURE;
        case 90:
            return e_MAV_CMD_MAV_CMD_VIDEO_STOP_CAPTURE;
        case 91:
            return e_MAV_CMD_MAV_CMD_VIDEO_START_STREAMING;
        case 92:
            return e_MAV_CMD_MAV_CMD_VIDEO_STOP_STREAMING;
        case 93:
            return e_MAV_CMD_MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION;
        case 94:
            return e_MAV_CMD_MAV_CMD_LOGGING_START;
        case 95:
            return e_MAV_CMD_MAV_CMD_LOGGING_STOP;
        case 96:
            return e_MAV_CMD_MAV_CMD_AIRFRAME_CONFIGURATION;
        case 97:
            return e_MAV_CMD_MAV_CMD_PANORAMA_CREATE;
        case 98:
            return e_MAV_CMD_MAV_CMD_DO_VTOL_TRANSITION;
        case 99:
            return e_MAV_CMD_MAV_CMD_ARM_AUTHORIZATION_REQUEST;
        case 100:
            return e_MAV_CMD_MAV_CMD_SET_GUIDED_SUBMODE_STANDARD;
        case 101:
            return e_MAV_CMD_MAV_CMD_SET_GUIDED_SUBMODE_CIRCLE;
        case 102:
            return e_MAV_CMD_MAV_CMD_CONDITION_GATE;
        case 103:
            return e_MAV_CMD_MAV_CMD_NAV_FENCE_RETURN_POINT;
        case 104:
            return e_MAV_CMD_MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION;
        case 105:
            return e_MAV_CMD_MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION;
        case 106:
            return e_MAV_CMD_MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION;
        case 107:
            return e_MAV_CMD_MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION;
        case 108:
            return e_MAV_CMD_MAV_CMD_NAV_RALLY_POINT;
        case 109:
            return e_MAV_CMD_MAV_CMD_UAVCAN_GET_NODE_INFO;
        case 110:
            return e_MAV_CMD_MAV_CMD_PAYLOAD_PREPARE_DEPLOY;
        case 111:
            return e_MAV_CMD_MAV_CMD_PAYLOAD_CONTROL_DEPLOY;
        case 112:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_1;
        case 113:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_2;
        case 114:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_3;
        case 115:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_4;
        case 116:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_5;
        case 117:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_1;
        case 118:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_2;
        case 119:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_3;
        case 120:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_4;
        case 121:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_5;
        case 122:
            return e_MAV_CMD_MAV_CMD_USER_1;
        case 123:
            return e_MAV_CMD_MAV_CMD_USER_2;
        case 124:
            return e_MAV_CMD_MAV_CMD_USER_3;
        case 125:
            return e_MAV_CMD_MAV_CMD_USER_4;
        case 126:
            return e_MAV_CMD_MAV_CMD_USER_5;
        case 127:
            return e_MAV_CMD_MAV_CMD_RESET_MPPT;
        case 128:
            return e_MAV_CMD_MAV_CMD_PAYLOAD_CONTROL;
        default: ;//assert(false);//("Unknown enum ID " + id);
    }
}
INLINER void p75_command_SET(e_MAV_CMD  src, Pack * dst)//The scheduled action for the mission item. see MAV_CMD in common.xml MAVLink spec
{
    uint8_t * data = dst->data;
    int32_t id;
    switch(src)
    {
        case e_MAV_CMD_MAV_CMD_NAV_WAYPOINT:
            id = 0;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_LOITER_UNLIM:
            id = 1;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_LOITER_TURNS:
            id = 2;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_LOITER_TIME:
            id = 3;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_RETURN_TO_LAUNCH:
            id = 4;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_LAND:
            id = 5;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_TAKEOFF:
            id = 6;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_LAND_LOCAL:
            id = 7;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_TAKEOFF_LOCAL:
            id = 8;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_FOLLOW:
            id = 9;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT:
            id = 10;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_LOITER_TO_ALT:
            id = 11;
            break;
        case e_MAV_CMD_MAV_CMD_DO_FOLLOW:
            id = 12;
            break;
        case e_MAV_CMD_MAV_CMD_DO_FOLLOW_REPOSITION:
            id = 13;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_ROI:
            id = 14;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_PATHPLANNING:
            id = 15;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_SPLINE_WAYPOINT:
            id = 16;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_VTOL_TAKEOFF:
            id = 17;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_VTOL_LAND:
            id = 18;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_GUIDED_ENABLE:
            id = 19;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_DELAY:
            id = 20;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_PAYLOAD_PLACE:
            id = 21;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_LAST:
            id = 22;
            break;
        case e_MAV_CMD_MAV_CMD_CONDITION_DELAY:
            id = 23;
            break;
        case e_MAV_CMD_MAV_CMD_CONDITION_CHANGE_ALT:
            id = 24;
            break;
        case e_MAV_CMD_MAV_CMD_CONDITION_DISTANCE:
            id = 25;
            break;
        case e_MAV_CMD_MAV_CMD_CONDITION_YAW:
            id = 26;
            break;
        case e_MAV_CMD_MAV_CMD_CONDITION_LAST:
            id = 27;
            break;
        case e_MAV_CMD_MAV_CMD_DO_SET_MODE:
            id = 28;
            break;
        case e_MAV_CMD_MAV_CMD_DO_JUMP:
            id = 29;
            break;
        case e_MAV_CMD_MAV_CMD_DO_CHANGE_SPEED:
            id = 30;
            break;
        case e_MAV_CMD_MAV_CMD_DO_SET_HOME:
            id = 31;
            break;
        case e_MAV_CMD_MAV_CMD_DO_SET_PARAMETER:
            id = 32;
            break;
        case e_MAV_CMD_MAV_CMD_DO_SET_RELAY:
            id = 33;
            break;
        case e_MAV_CMD_MAV_CMD_DO_REPEAT_RELAY:
            id = 34;
            break;
        case e_MAV_CMD_MAV_CMD_DO_SET_SERVO:
            id = 35;
            break;
        case e_MAV_CMD_MAV_CMD_DO_REPEAT_SERVO:
            id = 36;
            break;
        case e_MAV_CMD_MAV_CMD_DO_FLIGHTTERMINATION:
            id = 37;
            break;
        case e_MAV_CMD_MAV_CMD_DO_CHANGE_ALTITUDE:
            id = 38;
            break;
        case e_MAV_CMD_MAV_CMD_DO_LAND_START:
            id = 39;
            break;
        case e_MAV_CMD_MAV_CMD_DO_RALLY_LAND:
            id = 40;
            break;
        case e_MAV_CMD_MAV_CMD_DO_GO_AROUND:
            id = 41;
            break;
        case e_MAV_CMD_MAV_CMD_DO_REPOSITION:
            id = 42;
            break;
        case e_MAV_CMD_MAV_CMD_DO_PAUSE_CONTINUE:
            id = 43;
            break;
        case e_MAV_CMD_MAV_CMD_DO_SET_REVERSE:
            id = 44;
            break;
        case e_MAV_CMD_MAV_CMD_DO_CONTROL_VIDEO:
            id = 45;
            break;
        case e_MAV_CMD_MAV_CMD_DO_SET_ROI:
            id = 46;
            break;
        case e_MAV_CMD_MAV_CMD_DO_DIGICAM_CONFIGURE:
            id = 47;
            break;
        case e_MAV_CMD_MAV_CMD_DO_DIGICAM_CONTROL:
            id = 48;
            break;
        case e_MAV_CMD_MAV_CMD_DO_MOUNT_CONFIGURE:
            id = 49;
            break;
        case e_MAV_CMD_MAV_CMD_DO_MOUNT_CONTROL:
            id = 50;
            break;
        case e_MAV_CMD_MAV_CMD_DO_SET_CAM_TRIGG_DIST:
            id = 51;
            break;
        case e_MAV_CMD_MAV_CMD_DO_FENCE_ENABLE:
            id = 52;
            break;
        case e_MAV_CMD_MAV_CMD_DO_PARACHUTE:
            id = 53;
            break;
        case e_MAV_CMD_MAV_CMD_DO_MOTOR_TEST:
            id = 54;
            break;
        case e_MAV_CMD_MAV_CMD_DO_INVERTED_FLIGHT:
            id = 55;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_SET_YAW_SPEED:
            id = 56;
            break;
        case e_MAV_CMD_MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL:
            id = 57;
            break;
        case e_MAV_CMD_MAV_CMD_DO_MOUNT_CONTROL_QUAT:
            id = 58;
            break;
        case e_MAV_CMD_MAV_CMD_DO_GUIDED_MASTER:
            id = 59;
            break;
        case e_MAV_CMD_MAV_CMD_DO_GUIDED_LIMITS:
            id = 60;
            break;
        case e_MAV_CMD_MAV_CMD_DO_ENGINE_CONTROL:
            id = 61;
            break;
        case e_MAV_CMD_MAV_CMD_DO_LAST:
            id = 62;
            break;
        case e_MAV_CMD_MAV_CMD_PREFLIGHT_CALIBRATION:
            id = 63;
            break;
        case e_MAV_CMD_MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS:
            id = 64;
            break;
        case e_MAV_CMD_MAV_CMD_PREFLIGHT_UAVCAN:
            id = 65;
            break;
        case e_MAV_CMD_MAV_CMD_PREFLIGHT_STORAGE:
            id = 66;
            break;
        case e_MAV_CMD_MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
            id = 67;
            break;
        case e_MAV_CMD_MAV_CMD_OVERRIDE_GOTO:
            id = 68;
            break;
        case e_MAV_CMD_MAV_CMD_MISSION_START:
            id = 69;
            break;
        case e_MAV_CMD_MAV_CMD_COMPONENT_ARM_DISARM:
            id = 70;
            break;
        case e_MAV_CMD_MAV_CMD_GET_HOME_POSITION:
            id = 71;
            break;
        case e_MAV_CMD_MAV_CMD_START_RX_PAIR:
            id = 72;
            break;
        case e_MAV_CMD_MAV_CMD_GET_MESSAGE_INTERVAL:
            id = 73;
            break;
        case e_MAV_CMD_MAV_CMD_SET_MESSAGE_INTERVAL:
            id = 74;
            break;
        case e_MAV_CMD_MAV_CMD_REQUEST_PROTOCOL_VERSION:
            id = 75;
            break;
        case e_MAV_CMD_MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES:
            id = 76;
            break;
        case e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_INFORMATION:
            id = 77;
            break;
        case e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_SETTINGS:
            id = 78;
            break;
        case e_MAV_CMD_MAV_CMD_REQUEST_STORAGE_INFORMATION:
            id = 79;
            break;
        case e_MAV_CMD_MAV_CMD_STORAGE_FORMAT:
            id = 80;
            break;
        case e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS:
            id = 81;
            break;
        case e_MAV_CMD_MAV_CMD_REQUEST_FLIGHT_INFORMATION:
            id = 82;
            break;
        case e_MAV_CMD_MAV_CMD_RESET_CAMERA_SETTINGS:
            id = 83;
            break;
        case e_MAV_CMD_MAV_CMD_SET_CAMERA_MODE:
            id = 84;
            break;
        case e_MAV_CMD_MAV_CMD_IMAGE_START_CAPTURE:
            id = 85;
            break;
        case e_MAV_CMD_MAV_CMD_IMAGE_STOP_CAPTURE:
            id = 86;
            break;
        case e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE:
            id = 87;
            break;
        case e_MAV_CMD_MAV_CMD_DO_TRIGGER_CONTROL:
            id = 88;
            break;
        case e_MAV_CMD_MAV_CMD_VIDEO_START_CAPTURE:
            id = 89;
            break;
        case e_MAV_CMD_MAV_CMD_VIDEO_STOP_CAPTURE:
            id = 90;
            break;
        case e_MAV_CMD_MAV_CMD_VIDEO_START_STREAMING:
            id = 91;
            break;
        case e_MAV_CMD_MAV_CMD_VIDEO_STOP_STREAMING:
            id = 92;
            break;
        case e_MAV_CMD_MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION:
            id = 93;
            break;
        case e_MAV_CMD_MAV_CMD_LOGGING_START:
            id = 94;
            break;
        case e_MAV_CMD_MAV_CMD_LOGGING_STOP:
            id = 95;
            break;
        case e_MAV_CMD_MAV_CMD_AIRFRAME_CONFIGURATION:
            id = 96;
            break;
        case e_MAV_CMD_MAV_CMD_PANORAMA_CREATE:
            id = 97;
            break;
        case e_MAV_CMD_MAV_CMD_DO_VTOL_TRANSITION:
            id = 98;
            break;
        case e_MAV_CMD_MAV_CMD_ARM_AUTHORIZATION_REQUEST:
            id = 99;
            break;
        case e_MAV_CMD_MAV_CMD_SET_GUIDED_SUBMODE_STANDARD:
            id = 100;
            break;
        case e_MAV_CMD_MAV_CMD_SET_GUIDED_SUBMODE_CIRCLE:
            id = 101;
            break;
        case e_MAV_CMD_MAV_CMD_CONDITION_GATE:
            id = 102;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_FENCE_RETURN_POINT:
            id = 103;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION:
            id = 104;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION:
            id = 105;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION:
            id = 106;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION:
            id = 107;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_RALLY_POINT:
            id = 108;
            break;
        case e_MAV_CMD_MAV_CMD_UAVCAN_GET_NODE_INFO:
            id = 109;
            break;
        case e_MAV_CMD_MAV_CMD_PAYLOAD_PREPARE_DEPLOY:
            id = 110;
            break;
        case e_MAV_CMD_MAV_CMD_PAYLOAD_CONTROL_DEPLOY:
            id = 111;
            break;
        case e_MAV_CMD_MAV_CMD_WAYPOINT_USER_1:
            id = 112;
            break;
        case e_MAV_CMD_MAV_CMD_WAYPOINT_USER_2:
            id = 113;
            break;
        case e_MAV_CMD_MAV_CMD_WAYPOINT_USER_3:
            id = 114;
            break;
        case e_MAV_CMD_MAV_CMD_WAYPOINT_USER_4:
            id = 115;
            break;
        case e_MAV_CMD_MAV_CMD_WAYPOINT_USER_5:
            id = 116;
            break;
        case e_MAV_CMD_MAV_CMD_SPATIAL_USER_1:
            id = 117;
            break;
        case e_MAV_CMD_MAV_CMD_SPATIAL_USER_2:
            id = 118;
            break;
        case e_MAV_CMD_MAV_CMD_SPATIAL_USER_3:
            id = 119;
            break;
        case e_MAV_CMD_MAV_CMD_SPATIAL_USER_4:
            id = 120;
            break;
        case e_MAV_CMD_MAV_CMD_SPATIAL_USER_5:
            id = 121;
            break;
        case e_MAV_CMD_MAV_CMD_USER_1:
            id = 122;
            break;
        case e_MAV_CMD_MAV_CMD_USER_2:
            id = 123;
            break;
        case e_MAV_CMD_MAV_CMD_USER_3:
            id = 124;
            break;
        case e_MAV_CMD_MAV_CMD_USER_4:
            id = 125;
            break;
        case e_MAV_CMD_MAV_CMD_USER_5:
            id = 126;
            break;
        case e_MAV_CMD_MAV_CMD_RESET_MPPT:
            id = 127;
            break;
        case e_MAV_CMD_MAV_CMD_PAYLOAD_CONTROL:
            id = 128;
            break;
        default: ;// assert(false);//("Unknown enum" + id);
    }
    set_bits(id, 8, data, 260);
}
INLINER uint8_t p76_target_system_GET(Pack * src)//System which should execute the comman
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 1)));
}
INLINER void p76_target_system_SET(uint8_t  src, Pack * dst)//System which should execute the comman
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  0);
}
INLINER uint8_t p76_target_component_GET(Pack * src)//Component which should execute the command, 0 for all component
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  1, 1)));
}
INLINER void p76_target_component_SET(uint8_t  src, Pack * dst)//Component which should execute the command, 0 for all component
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  1);
}
INLINER uint8_t p76_confirmation_GET(Pack * src)//0: First transmission of this command. 1-255: Confirmation transmissions (e.g. for kill command
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 1)));
}
INLINER void p76_confirmation_SET(uint8_t  src, Pack * dst)//0: First transmission of this command. 1-255: Confirmation transmissions (e.g. for kill command
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  2);
}
INLINER float p76_param1_GET(Pack * src)//Parameter 1, as defined by MAV_CMD enum
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  3, 4)));
}
INLINER void p76_param1_SET(float  src, Pack * dst)//Parameter 1, as defined by MAV_CMD enum
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  3);
}
INLINER float p76_param2_GET(Pack * src)//Parameter 2, as defined by MAV_CMD enum
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  7, 4)));
}
INLINER void p76_param2_SET(float  src, Pack * dst)//Parameter 2, as defined by MAV_CMD enum
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  7);
}
INLINER float p76_param3_GET(Pack * src)//Parameter 3, as defined by MAV_CMD enum
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  11, 4)));
}
INLINER void p76_param3_SET(float  src, Pack * dst)//Parameter 3, as defined by MAV_CMD enum
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  11);
}
INLINER float p76_param4_GET(Pack * src)//Parameter 4, as defined by MAV_CMD enum
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  15, 4)));
}
INLINER void p76_param4_SET(float  src, Pack * dst)//Parameter 4, as defined by MAV_CMD enum
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  15);
}
INLINER float p76_param5_GET(Pack * src)//Parameter 5, as defined by MAV_CMD enum
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  19, 4)));
}
INLINER void p76_param5_SET(float  src, Pack * dst)//Parameter 5, as defined by MAV_CMD enum
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  19);
}
INLINER float p76_param6_GET(Pack * src)//Parameter 6, as defined by MAV_CMD enum
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  23, 4)));
}
INLINER void p76_param6_SET(float  src, Pack * dst)//Parameter 6, as defined by MAV_CMD enum
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  23);
}
INLINER float p76_param7_GET(Pack * src)//Parameter 7, as defined by MAV_CMD enum
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  27, 4)));
}
INLINER void p76_param7_SET(float  src, Pack * dst)//Parameter 7, as defined by MAV_CMD enum
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  27);
}
INLINER e_MAV_CMD p76_command_GET(Pack * src)//Command ID, as defined by MAV_CMD enum
{
    uint8_t * data = src->data;
    switch(get_bits(data, 248, 8))
    {
        case 0:
            return e_MAV_CMD_MAV_CMD_NAV_WAYPOINT;
        case 1:
            return e_MAV_CMD_MAV_CMD_NAV_LOITER_UNLIM;
        case 2:
            return e_MAV_CMD_MAV_CMD_NAV_LOITER_TURNS;
        case 3:
            return e_MAV_CMD_MAV_CMD_NAV_LOITER_TIME;
        case 4:
            return e_MAV_CMD_MAV_CMD_NAV_RETURN_TO_LAUNCH;
        case 5:
            return e_MAV_CMD_MAV_CMD_NAV_LAND;
        case 6:
            return e_MAV_CMD_MAV_CMD_NAV_TAKEOFF;
        case 7:
            return e_MAV_CMD_MAV_CMD_NAV_LAND_LOCAL;
        case 8:
            return e_MAV_CMD_MAV_CMD_NAV_TAKEOFF_LOCAL;
        case 9:
            return e_MAV_CMD_MAV_CMD_NAV_FOLLOW;
        case 10:
            return e_MAV_CMD_MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT;
        case 11:
            return e_MAV_CMD_MAV_CMD_NAV_LOITER_TO_ALT;
        case 12:
            return e_MAV_CMD_MAV_CMD_DO_FOLLOW;
        case 13:
            return e_MAV_CMD_MAV_CMD_DO_FOLLOW_REPOSITION;
        case 14:
            return e_MAV_CMD_MAV_CMD_NAV_ROI;
        case 15:
            return e_MAV_CMD_MAV_CMD_NAV_PATHPLANNING;
        case 16:
            return e_MAV_CMD_MAV_CMD_NAV_SPLINE_WAYPOINT;
        case 17:
            return e_MAV_CMD_MAV_CMD_NAV_VTOL_TAKEOFF;
        case 18:
            return e_MAV_CMD_MAV_CMD_NAV_VTOL_LAND;
        case 19:
            return e_MAV_CMD_MAV_CMD_NAV_GUIDED_ENABLE;
        case 20:
            return e_MAV_CMD_MAV_CMD_NAV_DELAY;
        case 21:
            return e_MAV_CMD_MAV_CMD_NAV_PAYLOAD_PLACE;
        case 22:
            return e_MAV_CMD_MAV_CMD_NAV_LAST;
        case 23:
            return e_MAV_CMD_MAV_CMD_CONDITION_DELAY;
        case 24:
            return e_MAV_CMD_MAV_CMD_CONDITION_CHANGE_ALT;
        case 25:
            return e_MAV_CMD_MAV_CMD_CONDITION_DISTANCE;
        case 26:
            return e_MAV_CMD_MAV_CMD_CONDITION_YAW;
        case 27:
            return e_MAV_CMD_MAV_CMD_CONDITION_LAST;
        case 28:
            return e_MAV_CMD_MAV_CMD_DO_SET_MODE;
        case 29:
            return e_MAV_CMD_MAV_CMD_DO_JUMP;
        case 30:
            return e_MAV_CMD_MAV_CMD_DO_CHANGE_SPEED;
        case 31:
            return e_MAV_CMD_MAV_CMD_DO_SET_HOME;
        case 32:
            return e_MAV_CMD_MAV_CMD_DO_SET_PARAMETER;
        case 33:
            return e_MAV_CMD_MAV_CMD_DO_SET_RELAY;
        case 34:
            return e_MAV_CMD_MAV_CMD_DO_REPEAT_RELAY;
        case 35:
            return e_MAV_CMD_MAV_CMD_DO_SET_SERVO;
        case 36:
            return e_MAV_CMD_MAV_CMD_DO_REPEAT_SERVO;
        case 37:
            return e_MAV_CMD_MAV_CMD_DO_FLIGHTTERMINATION;
        case 38:
            return e_MAV_CMD_MAV_CMD_DO_CHANGE_ALTITUDE;
        case 39:
            return e_MAV_CMD_MAV_CMD_DO_LAND_START;
        case 40:
            return e_MAV_CMD_MAV_CMD_DO_RALLY_LAND;
        case 41:
            return e_MAV_CMD_MAV_CMD_DO_GO_AROUND;
        case 42:
            return e_MAV_CMD_MAV_CMD_DO_REPOSITION;
        case 43:
            return e_MAV_CMD_MAV_CMD_DO_PAUSE_CONTINUE;
        case 44:
            return e_MAV_CMD_MAV_CMD_DO_SET_REVERSE;
        case 45:
            return e_MAV_CMD_MAV_CMD_DO_CONTROL_VIDEO;
        case 46:
            return e_MAV_CMD_MAV_CMD_DO_SET_ROI;
        case 47:
            return e_MAV_CMD_MAV_CMD_DO_DIGICAM_CONFIGURE;
        case 48:
            return e_MAV_CMD_MAV_CMD_DO_DIGICAM_CONTROL;
        case 49:
            return e_MAV_CMD_MAV_CMD_DO_MOUNT_CONFIGURE;
        case 50:
            return e_MAV_CMD_MAV_CMD_DO_MOUNT_CONTROL;
        case 51:
            return e_MAV_CMD_MAV_CMD_DO_SET_CAM_TRIGG_DIST;
        case 52:
            return e_MAV_CMD_MAV_CMD_DO_FENCE_ENABLE;
        case 53:
            return e_MAV_CMD_MAV_CMD_DO_PARACHUTE;
        case 54:
            return e_MAV_CMD_MAV_CMD_DO_MOTOR_TEST;
        case 55:
            return e_MAV_CMD_MAV_CMD_DO_INVERTED_FLIGHT;
        case 56:
            return e_MAV_CMD_MAV_CMD_NAV_SET_YAW_SPEED;
        case 57:
            return e_MAV_CMD_MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL;
        case 58:
            return e_MAV_CMD_MAV_CMD_DO_MOUNT_CONTROL_QUAT;
        case 59:
            return e_MAV_CMD_MAV_CMD_DO_GUIDED_MASTER;
        case 60:
            return e_MAV_CMD_MAV_CMD_DO_GUIDED_LIMITS;
        case 61:
            return e_MAV_CMD_MAV_CMD_DO_ENGINE_CONTROL;
        case 62:
            return e_MAV_CMD_MAV_CMD_DO_LAST;
        case 63:
            return e_MAV_CMD_MAV_CMD_PREFLIGHT_CALIBRATION;
        case 64:
            return e_MAV_CMD_MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS;
        case 65:
            return e_MAV_CMD_MAV_CMD_PREFLIGHT_UAVCAN;
        case 66:
            return e_MAV_CMD_MAV_CMD_PREFLIGHT_STORAGE;
        case 67:
            return e_MAV_CMD_MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN;
        case 68:
            return e_MAV_CMD_MAV_CMD_OVERRIDE_GOTO;
        case 69:
            return e_MAV_CMD_MAV_CMD_MISSION_START;
        case 70:
            return e_MAV_CMD_MAV_CMD_COMPONENT_ARM_DISARM;
        case 71:
            return e_MAV_CMD_MAV_CMD_GET_HOME_POSITION;
        case 72:
            return e_MAV_CMD_MAV_CMD_START_RX_PAIR;
        case 73:
            return e_MAV_CMD_MAV_CMD_GET_MESSAGE_INTERVAL;
        case 74:
            return e_MAV_CMD_MAV_CMD_SET_MESSAGE_INTERVAL;
        case 75:
            return e_MAV_CMD_MAV_CMD_REQUEST_PROTOCOL_VERSION;
        case 76:
            return e_MAV_CMD_MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES;
        case 77:
            return e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_INFORMATION;
        case 78:
            return e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_SETTINGS;
        case 79:
            return e_MAV_CMD_MAV_CMD_REQUEST_STORAGE_INFORMATION;
        case 80:
            return e_MAV_CMD_MAV_CMD_STORAGE_FORMAT;
        case 81:
            return e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS;
        case 82:
            return e_MAV_CMD_MAV_CMD_REQUEST_FLIGHT_INFORMATION;
        case 83:
            return e_MAV_CMD_MAV_CMD_RESET_CAMERA_SETTINGS;
        case 84:
            return e_MAV_CMD_MAV_CMD_SET_CAMERA_MODE;
        case 85:
            return e_MAV_CMD_MAV_CMD_IMAGE_START_CAPTURE;
        case 86:
            return e_MAV_CMD_MAV_CMD_IMAGE_STOP_CAPTURE;
        case 87:
            return e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE;
        case 88:
            return e_MAV_CMD_MAV_CMD_DO_TRIGGER_CONTROL;
        case 89:
            return e_MAV_CMD_MAV_CMD_VIDEO_START_CAPTURE;
        case 90:
            return e_MAV_CMD_MAV_CMD_VIDEO_STOP_CAPTURE;
        case 91:
            return e_MAV_CMD_MAV_CMD_VIDEO_START_STREAMING;
        case 92:
            return e_MAV_CMD_MAV_CMD_VIDEO_STOP_STREAMING;
        case 93:
            return e_MAV_CMD_MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION;
        case 94:
            return e_MAV_CMD_MAV_CMD_LOGGING_START;
        case 95:
            return e_MAV_CMD_MAV_CMD_LOGGING_STOP;
        case 96:
            return e_MAV_CMD_MAV_CMD_AIRFRAME_CONFIGURATION;
        case 97:
            return e_MAV_CMD_MAV_CMD_PANORAMA_CREATE;
        case 98:
            return e_MAV_CMD_MAV_CMD_DO_VTOL_TRANSITION;
        case 99:
            return e_MAV_CMD_MAV_CMD_ARM_AUTHORIZATION_REQUEST;
        case 100:
            return e_MAV_CMD_MAV_CMD_SET_GUIDED_SUBMODE_STANDARD;
        case 101:
            return e_MAV_CMD_MAV_CMD_SET_GUIDED_SUBMODE_CIRCLE;
        case 102:
            return e_MAV_CMD_MAV_CMD_CONDITION_GATE;
        case 103:
            return e_MAV_CMD_MAV_CMD_NAV_FENCE_RETURN_POINT;
        case 104:
            return e_MAV_CMD_MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION;
        case 105:
            return e_MAV_CMD_MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION;
        case 106:
            return e_MAV_CMD_MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION;
        case 107:
            return e_MAV_CMD_MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION;
        case 108:
            return e_MAV_CMD_MAV_CMD_NAV_RALLY_POINT;
        case 109:
            return e_MAV_CMD_MAV_CMD_UAVCAN_GET_NODE_INFO;
        case 110:
            return e_MAV_CMD_MAV_CMD_PAYLOAD_PREPARE_DEPLOY;
        case 111:
            return e_MAV_CMD_MAV_CMD_PAYLOAD_CONTROL_DEPLOY;
        case 112:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_1;
        case 113:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_2;
        case 114:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_3;
        case 115:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_4;
        case 116:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_5;
        case 117:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_1;
        case 118:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_2;
        case 119:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_3;
        case 120:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_4;
        case 121:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_5;
        case 122:
            return e_MAV_CMD_MAV_CMD_USER_1;
        case 123:
            return e_MAV_CMD_MAV_CMD_USER_2;
        case 124:
            return e_MAV_CMD_MAV_CMD_USER_3;
        case 125:
            return e_MAV_CMD_MAV_CMD_USER_4;
        case 126:
            return e_MAV_CMD_MAV_CMD_USER_5;
        case 127:
            return e_MAV_CMD_MAV_CMD_RESET_MPPT;
        case 128:
            return e_MAV_CMD_MAV_CMD_PAYLOAD_CONTROL;
        default: ;//assert(false);//("Unknown enum ID " + id);
    }
}
INLINER void p76_command_SET(e_MAV_CMD  src, Pack * dst)//Command ID, as defined by MAV_CMD enum
{
    uint8_t * data = dst->data;
    int32_t id;
    switch(src)
    {
        case e_MAV_CMD_MAV_CMD_NAV_WAYPOINT:
            id = 0;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_LOITER_UNLIM:
            id = 1;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_LOITER_TURNS:
            id = 2;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_LOITER_TIME:
            id = 3;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_RETURN_TO_LAUNCH:
            id = 4;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_LAND:
            id = 5;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_TAKEOFF:
            id = 6;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_LAND_LOCAL:
            id = 7;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_TAKEOFF_LOCAL:
            id = 8;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_FOLLOW:
            id = 9;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT:
            id = 10;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_LOITER_TO_ALT:
            id = 11;
            break;
        case e_MAV_CMD_MAV_CMD_DO_FOLLOW:
            id = 12;
            break;
        case e_MAV_CMD_MAV_CMD_DO_FOLLOW_REPOSITION:
            id = 13;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_ROI:
            id = 14;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_PATHPLANNING:
            id = 15;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_SPLINE_WAYPOINT:
            id = 16;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_VTOL_TAKEOFF:
            id = 17;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_VTOL_LAND:
            id = 18;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_GUIDED_ENABLE:
            id = 19;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_DELAY:
            id = 20;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_PAYLOAD_PLACE:
            id = 21;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_LAST:
            id = 22;
            break;
        case e_MAV_CMD_MAV_CMD_CONDITION_DELAY:
            id = 23;
            break;
        case e_MAV_CMD_MAV_CMD_CONDITION_CHANGE_ALT:
            id = 24;
            break;
        case e_MAV_CMD_MAV_CMD_CONDITION_DISTANCE:
            id = 25;
            break;
        case e_MAV_CMD_MAV_CMD_CONDITION_YAW:
            id = 26;
            break;
        case e_MAV_CMD_MAV_CMD_CONDITION_LAST:
            id = 27;
            break;
        case e_MAV_CMD_MAV_CMD_DO_SET_MODE:
            id = 28;
            break;
        case e_MAV_CMD_MAV_CMD_DO_JUMP:
            id = 29;
            break;
        case e_MAV_CMD_MAV_CMD_DO_CHANGE_SPEED:
            id = 30;
            break;
        case e_MAV_CMD_MAV_CMD_DO_SET_HOME:
            id = 31;
            break;
        case e_MAV_CMD_MAV_CMD_DO_SET_PARAMETER:
            id = 32;
            break;
        case e_MAV_CMD_MAV_CMD_DO_SET_RELAY:
            id = 33;
            break;
        case e_MAV_CMD_MAV_CMD_DO_REPEAT_RELAY:
            id = 34;
            break;
        case e_MAV_CMD_MAV_CMD_DO_SET_SERVO:
            id = 35;
            break;
        case e_MAV_CMD_MAV_CMD_DO_REPEAT_SERVO:
            id = 36;
            break;
        case e_MAV_CMD_MAV_CMD_DO_FLIGHTTERMINATION:
            id = 37;
            break;
        case e_MAV_CMD_MAV_CMD_DO_CHANGE_ALTITUDE:
            id = 38;
            break;
        case e_MAV_CMD_MAV_CMD_DO_LAND_START:
            id = 39;
            break;
        case e_MAV_CMD_MAV_CMD_DO_RALLY_LAND:
            id = 40;
            break;
        case e_MAV_CMD_MAV_CMD_DO_GO_AROUND:
            id = 41;
            break;
        case e_MAV_CMD_MAV_CMD_DO_REPOSITION:
            id = 42;
            break;
        case e_MAV_CMD_MAV_CMD_DO_PAUSE_CONTINUE:
            id = 43;
            break;
        case e_MAV_CMD_MAV_CMD_DO_SET_REVERSE:
            id = 44;
            break;
        case e_MAV_CMD_MAV_CMD_DO_CONTROL_VIDEO:
            id = 45;
            break;
        case e_MAV_CMD_MAV_CMD_DO_SET_ROI:
            id = 46;
            break;
        case e_MAV_CMD_MAV_CMD_DO_DIGICAM_CONFIGURE:
            id = 47;
            break;
        case e_MAV_CMD_MAV_CMD_DO_DIGICAM_CONTROL:
            id = 48;
            break;
        case e_MAV_CMD_MAV_CMD_DO_MOUNT_CONFIGURE:
            id = 49;
            break;
        case e_MAV_CMD_MAV_CMD_DO_MOUNT_CONTROL:
            id = 50;
            break;
        case e_MAV_CMD_MAV_CMD_DO_SET_CAM_TRIGG_DIST:
            id = 51;
            break;
        case e_MAV_CMD_MAV_CMD_DO_FENCE_ENABLE:
            id = 52;
            break;
        case e_MAV_CMD_MAV_CMD_DO_PARACHUTE:
            id = 53;
            break;
        case e_MAV_CMD_MAV_CMD_DO_MOTOR_TEST:
            id = 54;
            break;
        case e_MAV_CMD_MAV_CMD_DO_INVERTED_FLIGHT:
            id = 55;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_SET_YAW_SPEED:
            id = 56;
            break;
        case e_MAV_CMD_MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL:
            id = 57;
            break;
        case e_MAV_CMD_MAV_CMD_DO_MOUNT_CONTROL_QUAT:
            id = 58;
            break;
        case e_MAV_CMD_MAV_CMD_DO_GUIDED_MASTER:
            id = 59;
            break;
        case e_MAV_CMD_MAV_CMD_DO_GUIDED_LIMITS:
            id = 60;
            break;
        case e_MAV_CMD_MAV_CMD_DO_ENGINE_CONTROL:
            id = 61;
            break;
        case e_MAV_CMD_MAV_CMD_DO_LAST:
            id = 62;
            break;
        case e_MAV_CMD_MAV_CMD_PREFLIGHT_CALIBRATION:
            id = 63;
            break;
        case e_MAV_CMD_MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS:
            id = 64;
            break;
        case e_MAV_CMD_MAV_CMD_PREFLIGHT_UAVCAN:
            id = 65;
            break;
        case e_MAV_CMD_MAV_CMD_PREFLIGHT_STORAGE:
            id = 66;
            break;
        case e_MAV_CMD_MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
            id = 67;
            break;
        case e_MAV_CMD_MAV_CMD_OVERRIDE_GOTO:
            id = 68;
            break;
        case e_MAV_CMD_MAV_CMD_MISSION_START:
            id = 69;
            break;
        case e_MAV_CMD_MAV_CMD_COMPONENT_ARM_DISARM:
            id = 70;
            break;
        case e_MAV_CMD_MAV_CMD_GET_HOME_POSITION:
            id = 71;
            break;
        case e_MAV_CMD_MAV_CMD_START_RX_PAIR:
            id = 72;
            break;
        case e_MAV_CMD_MAV_CMD_GET_MESSAGE_INTERVAL:
            id = 73;
            break;
        case e_MAV_CMD_MAV_CMD_SET_MESSAGE_INTERVAL:
            id = 74;
            break;
        case e_MAV_CMD_MAV_CMD_REQUEST_PROTOCOL_VERSION:
            id = 75;
            break;
        case e_MAV_CMD_MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES:
            id = 76;
            break;
        case e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_INFORMATION:
            id = 77;
            break;
        case e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_SETTINGS:
            id = 78;
            break;
        case e_MAV_CMD_MAV_CMD_REQUEST_STORAGE_INFORMATION:
            id = 79;
            break;
        case e_MAV_CMD_MAV_CMD_STORAGE_FORMAT:
            id = 80;
            break;
        case e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS:
            id = 81;
            break;
        case e_MAV_CMD_MAV_CMD_REQUEST_FLIGHT_INFORMATION:
            id = 82;
            break;
        case e_MAV_CMD_MAV_CMD_RESET_CAMERA_SETTINGS:
            id = 83;
            break;
        case e_MAV_CMD_MAV_CMD_SET_CAMERA_MODE:
            id = 84;
            break;
        case e_MAV_CMD_MAV_CMD_IMAGE_START_CAPTURE:
            id = 85;
            break;
        case e_MAV_CMD_MAV_CMD_IMAGE_STOP_CAPTURE:
            id = 86;
            break;
        case e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE:
            id = 87;
            break;
        case e_MAV_CMD_MAV_CMD_DO_TRIGGER_CONTROL:
            id = 88;
            break;
        case e_MAV_CMD_MAV_CMD_VIDEO_START_CAPTURE:
            id = 89;
            break;
        case e_MAV_CMD_MAV_CMD_VIDEO_STOP_CAPTURE:
            id = 90;
            break;
        case e_MAV_CMD_MAV_CMD_VIDEO_START_STREAMING:
            id = 91;
            break;
        case e_MAV_CMD_MAV_CMD_VIDEO_STOP_STREAMING:
            id = 92;
            break;
        case e_MAV_CMD_MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION:
            id = 93;
            break;
        case e_MAV_CMD_MAV_CMD_LOGGING_START:
            id = 94;
            break;
        case e_MAV_CMD_MAV_CMD_LOGGING_STOP:
            id = 95;
            break;
        case e_MAV_CMD_MAV_CMD_AIRFRAME_CONFIGURATION:
            id = 96;
            break;
        case e_MAV_CMD_MAV_CMD_PANORAMA_CREATE:
            id = 97;
            break;
        case e_MAV_CMD_MAV_CMD_DO_VTOL_TRANSITION:
            id = 98;
            break;
        case e_MAV_CMD_MAV_CMD_ARM_AUTHORIZATION_REQUEST:
            id = 99;
            break;
        case e_MAV_CMD_MAV_CMD_SET_GUIDED_SUBMODE_STANDARD:
            id = 100;
            break;
        case e_MAV_CMD_MAV_CMD_SET_GUIDED_SUBMODE_CIRCLE:
            id = 101;
            break;
        case e_MAV_CMD_MAV_CMD_CONDITION_GATE:
            id = 102;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_FENCE_RETURN_POINT:
            id = 103;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION:
            id = 104;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION:
            id = 105;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION:
            id = 106;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION:
            id = 107;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_RALLY_POINT:
            id = 108;
            break;
        case e_MAV_CMD_MAV_CMD_UAVCAN_GET_NODE_INFO:
            id = 109;
            break;
        case e_MAV_CMD_MAV_CMD_PAYLOAD_PREPARE_DEPLOY:
            id = 110;
            break;
        case e_MAV_CMD_MAV_CMD_PAYLOAD_CONTROL_DEPLOY:
            id = 111;
            break;
        case e_MAV_CMD_MAV_CMD_WAYPOINT_USER_1:
            id = 112;
            break;
        case e_MAV_CMD_MAV_CMD_WAYPOINT_USER_2:
            id = 113;
            break;
        case e_MAV_CMD_MAV_CMD_WAYPOINT_USER_3:
            id = 114;
            break;
        case e_MAV_CMD_MAV_CMD_WAYPOINT_USER_4:
            id = 115;
            break;
        case e_MAV_CMD_MAV_CMD_WAYPOINT_USER_5:
            id = 116;
            break;
        case e_MAV_CMD_MAV_CMD_SPATIAL_USER_1:
            id = 117;
            break;
        case e_MAV_CMD_MAV_CMD_SPATIAL_USER_2:
            id = 118;
            break;
        case e_MAV_CMD_MAV_CMD_SPATIAL_USER_3:
            id = 119;
            break;
        case e_MAV_CMD_MAV_CMD_SPATIAL_USER_4:
            id = 120;
            break;
        case e_MAV_CMD_MAV_CMD_SPATIAL_USER_5:
            id = 121;
            break;
        case e_MAV_CMD_MAV_CMD_USER_1:
            id = 122;
            break;
        case e_MAV_CMD_MAV_CMD_USER_2:
            id = 123;
            break;
        case e_MAV_CMD_MAV_CMD_USER_3:
            id = 124;
            break;
        case e_MAV_CMD_MAV_CMD_USER_4:
            id = 125;
            break;
        case e_MAV_CMD_MAV_CMD_USER_5:
            id = 126;
            break;
        case e_MAV_CMD_MAV_CMD_RESET_MPPT:
            id = 127;
            break;
        case e_MAV_CMD_MAV_CMD_PAYLOAD_CONTROL:
            id = 128;
            break;
        default: ;// assert(false);//("Unknown enum" + id);
    }
    set_bits(id, 8, data, 248);
}
INLINER e_MAV_CMD p77_command_GET(Pack * src)//Command ID, as defined by MAV_CMD enum
{
    uint8_t * data = src->data;
    switch(get_bits(data, 0, 8))
    {
        case 0:
            return e_MAV_CMD_MAV_CMD_NAV_WAYPOINT;
        case 1:
            return e_MAV_CMD_MAV_CMD_NAV_LOITER_UNLIM;
        case 2:
            return e_MAV_CMD_MAV_CMD_NAV_LOITER_TURNS;
        case 3:
            return e_MAV_CMD_MAV_CMD_NAV_LOITER_TIME;
        case 4:
            return e_MAV_CMD_MAV_CMD_NAV_RETURN_TO_LAUNCH;
        case 5:
            return e_MAV_CMD_MAV_CMD_NAV_LAND;
        case 6:
            return e_MAV_CMD_MAV_CMD_NAV_TAKEOFF;
        case 7:
            return e_MAV_CMD_MAV_CMD_NAV_LAND_LOCAL;
        case 8:
            return e_MAV_CMD_MAV_CMD_NAV_TAKEOFF_LOCAL;
        case 9:
            return e_MAV_CMD_MAV_CMD_NAV_FOLLOW;
        case 10:
            return e_MAV_CMD_MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT;
        case 11:
            return e_MAV_CMD_MAV_CMD_NAV_LOITER_TO_ALT;
        case 12:
            return e_MAV_CMD_MAV_CMD_DO_FOLLOW;
        case 13:
            return e_MAV_CMD_MAV_CMD_DO_FOLLOW_REPOSITION;
        case 14:
            return e_MAV_CMD_MAV_CMD_NAV_ROI;
        case 15:
            return e_MAV_CMD_MAV_CMD_NAV_PATHPLANNING;
        case 16:
            return e_MAV_CMD_MAV_CMD_NAV_SPLINE_WAYPOINT;
        case 17:
            return e_MAV_CMD_MAV_CMD_NAV_VTOL_TAKEOFF;
        case 18:
            return e_MAV_CMD_MAV_CMD_NAV_VTOL_LAND;
        case 19:
            return e_MAV_CMD_MAV_CMD_NAV_GUIDED_ENABLE;
        case 20:
            return e_MAV_CMD_MAV_CMD_NAV_DELAY;
        case 21:
            return e_MAV_CMD_MAV_CMD_NAV_PAYLOAD_PLACE;
        case 22:
            return e_MAV_CMD_MAV_CMD_NAV_LAST;
        case 23:
            return e_MAV_CMD_MAV_CMD_CONDITION_DELAY;
        case 24:
            return e_MAV_CMD_MAV_CMD_CONDITION_CHANGE_ALT;
        case 25:
            return e_MAV_CMD_MAV_CMD_CONDITION_DISTANCE;
        case 26:
            return e_MAV_CMD_MAV_CMD_CONDITION_YAW;
        case 27:
            return e_MAV_CMD_MAV_CMD_CONDITION_LAST;
        case 28:
            return e_MAV_CMD_MAV_CMD_DO_SET_MODE;
        case 29:
            return e_MAV_CMD_MAV_CMD_DO_JUMP;
        case 30:
            return e_MAV_CMD_MAV_CMD_DO_CHANGE_SPEED;
        case 31:
            return e_MAV_CMD_MAV_CMD_DO_SET_HOME;
        case 32:
            return e_MAV_CMD_MAV_CMD_DO_SET_PARAMETER;
        case 33:
            return e_MAV_CMD_MAV_CMD_DO_SET_RELAY;
        case 34:
            return e_MAV_CMD_MAV_CMD_DO_REPEAT_RELAY;
        case 35:
            return e_MAV_CMD_MAV_CMD_DO_SET_SERVO;
        case 36:
            return e_MAV_CMD_MAV_CMD_DO_REPEAT_SERVO;
        case 37:
            return e_MAV_CMD_MAV_CMD_DO_FLIGHTTERMINATION;
        case 38:
            return e_MAV_CMD_MAV_CMD_DO_CHANGE_ALTITUDE;
        case 39:
            return e_MAV_CMD_MAV_CMD_DO_LAND_START;
        case 40:
            return e_MAV_CMD_MAV_CMD_DO_RALLY_LAND;
        case 41:
            return e_MAV_CMD_MAV_CMD_DO_GO_AROUND;
        case 42:
            return e_MAV_CMD_MAV_CMD_DO_REPOSITION;
        case 43:
            return e_MAV_CMD_MAV_CMD_DO_PAUSE_CONTINUE;
        case 44:
            return e_MAV_CMD_MAV_CMD_DO_SET_REVERSE;
        case 45:
            return e_MAV_CMD_MAV_CMD_DO_CONTROL_VIDEO;
        case 46:
            return e_MAV_CMD_MAV_CMD_DO_SET_ROI;
        case 47:
            return e_MAV_CMD_MAV_CMD_DO_DIGICAM_CONFIGURE;
        case 48:
            return e_MAV_CMD_MAV_CMD_DO_DIGICAM_CONTROL;
        case 49:
            return e_MAV_CMD_MAV_CMD_DO_MOUNT_CONFIGURE;
        case 50:
            return e_MAV_CMD_MAV_CMD_DO_MOUNT_CONTROL;
        case 51:
            return e_MAV_CMD_MAV_CMD_DO_SET_CAM_TRIGG_DIST;
        case 52:
            return e_MAV_CMD_MAV_CMD_DO_FENCE_ENABLE;
        case 53:
            return e_MAV_CMD_MAV_CMD_DO_PARACHUTE;
        case 54:
            return e_MAV_CMD_MAV_CMD_DO_MOTOR_TEST;
        case 55:
            return e_MAV_CMD_MAV_CMD_DO_INVERTED_FLIGHT;
        case 56:
            return e_MAV_CMD_MAV_CMD_NAV_SET_YAW_SPEED;
        case 57:
            return e_MAV_CMD_MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL;
        case 58:
            return e_MAV_CMD_MAV_CMD_DO_MOUNT_CONTROL_QUAT;
        case 59:
            return e_MAV_CMD_MAV_CMD_DO_GUIDED_MASTER;
        case 60:
            return e_MAV_CMD_MAV_CMD_DO_GUIDED_LIMITS;
        case 61:
            return e_MAV_CMD_MAV_CMD_DO_ENGINE_CONTROL;
        case 62:
            return e_MAV_CMD_MAV_CMD_DO_LAST;
        case 63:
            return e_MAV_CMD_MAV_CMD_PREFLIGHT_CALIBRATION;
        case 64:
            return e_MAV_CMD_MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS;
        case 65:
            return e_MAV_CMD_MAV_CMD_PREFLIGHT_UAVCAN;
        case 66:
            return e_MAV_CMD_MAV_CMD_PREFLIGHT_STORAGE;
        case 67:
            return e_MAV_CMD_MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN;
        case 68:
            return e_MAV_CMD_MAV_CMD_OVERRIDE_GOTO;
        case 69:
            return e_MAV_CMD_MAV_CMD_MISSION_START;
        case 70:
            return e_MAV_CMD_MAV_CMD_COMPONENT_ARM_DISARM;
        case 71:
            return e_MAV_CMD_MAV_CMD_GET_HOME_POSITION;
        case 72:
            return e_MAV_CMD_MAV_CMD_START_RX_PAIR;
        case 73:
            return e_MAV_CMD_MAV_CMD_GET_MESSAGE_INTERVAL;
        case 74:
            return e_MAV_CMD_MAV_CMD_SET_MESSAGE_INTERVAL;
        case 75:
            return e_MAV_CMD_MAV_CMD_REQUEST_PROTOCOL_VERSION;
        case 76:
            return e_MAV_CMD_MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES;
        case 77:
            return e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_INFORMATION;
        case 78:
            return e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_SETTINGS;
        case 79:
            return e_MAV_CMD_MAV_CMD_REQUEST_STORAGE_INFORMATION;
        case 80:
            return e_MAV_CMD_MAV_CMD_STORAGE_FORMAT;
        case 81:
            return e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS;
        case 82:
            return e_MAV_CMD_MAV_CMD_REQUEST_FLIGHT_INFORMATION;
        case 83:
            return e_MAV_CMD_MAV_CMD_RESET_CAMERA_SETTINGS;
        case 84:
            return e_MAV_CMD_MAV_CMD_SET_CAMERA_MODE;
        case 85:
            return e_MAV_CMD_MAV_CMD_IMAGE_START_CAPTURE;
        case 86:
            return e_MAV_CMD_MAV_CMD_IMAGE_STOP_CAPTURE;
        case 87:
            return e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE;
        case 88:
            return e_MAV_CMD_MAV_CMD_DO_TRIGGER_CONTROL;
        case 89:
            return e_MAV_CMD_MAV_CMD_VIDEO_START_CAPTURE;
        case 90:
            return e_MAV_CMD_MAV_CMD_VIDEO_STOP_CAPTURE;
        case 91:
            return e_MAV_CMD_MAV_CMD_VIDEO_START_STREAMING;
        case 92:
            return e_MAV_CMD_MAV_CMD_VIDEO_STOP_STREAMING;
        case 93:
            return e_MAV_CMD_MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION;
        case 94:
            return e_MAV_CMD_MAV_CMD_LOGGING_START;
        case 95:
            return e_MAV_CMD_MAV_CMD_LOGGING_STOP;
        case 96:
            return e_MAV_CMD_MAV_CMD_AIRFRAME_CONFIGURATION;
        case 97:
            return e_MAV_CMD_MAV_CMD_PANORAMA_CREATE;
        case 98:
            return e_MAV_CMD_MAV_CMD_DO_VTOL_TRANSITION;
        case 99:
            return e_MAV_CMD_MAV_CMD_ARM_AUTHORIZATION_REQUEST;
        case 100:
            return e_MAV_CMD_MAV_CMD_SET_GUIDED_SUBMODE_STANDARD;
        case 101:
            return e_MAV_CMD_MAV_CMD_SET_GUIDED_SUBMODE_CIRCLE;
        case 102:
            return e_MAV_CMD_MAV_CMD_CONDITION_GATE;
        case 103:
            return e_MAV_CMD_MAV_CMD_NAV_FENCE_RETURN_POINT;
        case 104:
            return e_MAV_CMD_MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION;
        case 105:
            return e_MAV_CMD_MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION;
        case 106:
            return e_MAV_CMD_MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION;
        case 107:
            return e_MAV_CMD_MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION;
        case 108:
            return e_MAV_CMD_MAV_CMD_NAV_RALLY_POINT;
        case 109:
            return e_MAV_CMD_MAV_CMD_UAVCAN_GET_NODE_INFO;
        case 110:
            return e_MAV_CMD_MAV_CMD_PAYLOAD_PREPARE_DEPLOY;
        case 111:
            return e_MAV_CMD_MAV_CMD_PAYLOAD_CONTROL_DEPLOY;
        case 112:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_1;
        case 113:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_2;
        case 114:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_3;
        case 115:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_4;
        case 116:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_5;
        case 117:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_1;
        case 118:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_2;
        case 119:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_3;
        case 120:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_4;
        case 121:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_5;
        case 122:
            return e_MAV_CMD_MAV_CMD_USER_1;
        case 123:
            return e_MAV_CMD_MAV_CMD_USER_2;
        case 124:
            return e_MAV_CMD_MAV_CMD_USER_3;
        case 125:
            return e_MAV_CMD_MAV_CMD_USER_4;
        case 126:
            return e_MAV_CMD_MAV_CMD_USER_5;
        case 127:
            return e_MAV_CMD_MAV_CMD_RESET_MPPT;
        case 128:
            return e_MAV_CMD_MAV_CMD_PAYLOAD_CONTROL;
        default: ;//assert(false);//("Unknown enum ID " + id);
    }
}
INLINER void p77_command_SET(e_MAV_CMD  src, Pack * dst)//Command ID, as defined by MAV_CMD enum
{
    uint8_t * data = dst->data;
    int32_t id;
    switch(src)
    {
        case e_MAV_CMD_MAV_CMD_NAV_WAYPOINT:
            id = 0;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_LOITER_UNLIM:
            id = 1;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_LOITER_TURNS:
            id = 2;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_LOITER_TIME:
            id = 3;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_RETURN_TO_LAUNCH:
            id = 4;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_LAND:
            id = 5;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_TAKEOFF:
            id = 6;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_LAND_LOCAL:
            id = 7;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_TAKEOFF_LOCAL:
            id = 8;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_FOLLOW:
            id = 9;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT:
            id = 10;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_LOITER_TO_ALT:
            id = 11;
            break;
        case e_MAV_CMD_MAV_CMD_DO_FOLLOW:
            id = 12;
            break;
        case e_MAV_CMD_MAV_CMD_DO_FOLLOW_REPOSITION:
            id = 13;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_ROI:
            id = 14;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_PATHPLANNING:
            id = 15;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_SPLINE_WAYPOINT:
            id = 16;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_VTOL_TAKEOFF:
            id = 17;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_VTOL_LAND:
            id = 18;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_GUIDED_ENABLE:
            id = 19;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_DELAY:
            id = 20;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_PAYLOAD_PLACE:
            id = 21;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_LAST:
            id = 22;
            break;
        case e_MAV_CMD_MAV_CMD_CONDITION_DELAY:
            id = 23;
            break;
        case e_MAV_CMD_MAV_CMD_CONDITION_CHANGE_ALT:
            id = 24;
            break;
        case e_MAV_CMD_MAV_CMD_CONDITION_DISTANCE:
            id = 25;
            break;
        case e_MAV_CMD_MAV_CMD_CONDITION_YAW:
            id = 26;
            break;
        case e_MAV_CMD_MAV_CMD_CONDITION_LAST:
            id = 27;
            break;
        case e_MAV_CMD_MAV_CMD_DO_SET_MODE:
            id = 28;
            break;
        case e_MAV_CMD_MAV_CMD_DO_JUMP:
            id = 29;
            break;
        case e_MAV_CMD_MAV_CMD_DO_CHANGE_SPEED:
            id = 30;
            break;
        case e_MAV_CMD_MAV_CMD_DO_SET_HOME:
            id = 31;
            break;
        case e_MAV_CMD_MAV_CMD_DO_SET_PARAMETER:
            id = 32;
            break;
        case e_MAV_CMD_MAV_CMD_DO_SET_RELAY:
            id = 33;
            break;
        case e_MAV_CMD_MAV_CMD_DO_REPEAT_RELAY:
            id = 34;
            break;
        case e_MAV_CMD_MAV_CMD_DO_SET_SERVO:
            id = 35;
            break;
        case e_MAV_CMD_MAV_CMD_DO_REPEAT_SERVO:
            id = 36;
            break;
        case e_MAV_CMD_MAV_CMD_DO_FLIGHTTERMINATION:
            id = 37;
            break;
        case e_MAV_CMD_MAV_CMD_DO_CHANGE_ALTITUDE:
            id = 38;
            break;
        case e_MAV_CMD_MAV_CMD_DO_LAND_START:
            id = 39;
            break;
        case e_MAV_CMD_MAV_CMD_DO_RALLY_LAND:
            id = 40;
            break;
        case e_MAV_CMD_MAV_CMD_DO_GO_AROUND:
            id = 41;
            break;
        case e_MAV_CMD_MAV_CMD_DO_REPOSITION:
            id = 42;
            break;
        case e_MAV_CMD_MAV_CMD_DO_PAUSE_CONTINUE:
            id = 43;
            break;
        case e_MAV_CMD_MAV_CMD_DO_SET_REVERSE:
            id = 44;
            break;
        case e_MAV_CMD_MAV_CMD_DO_CONTROL_VIDEO:
            id = 45;
            break;
        case e_MAV_CMD_MAV_CMD_DO_SET_ROI:
            id = 46;
            break;
        case e_MAV_CMD_MAV_CMD_DO_DIGICAM_CONFIGURE:
            id = 47;
            break;
        case e_MAV_CMD_MAV_CMD_DO_DIGICAM_CONTROL:
            id = 48;
            break;
        case e_MAV_CMD_MAV_CMD_DO_MOUNT_CONFIGURE:
            id = 49;
            break;
        case e_MAV_CMD_MAV_CMD_DO_MOUNT_CONTROL:
            id = 50;
            break;
        case e_MAV_CMD_MAV_CMD_DO_SET_CAM_TRIGG_DIST:
            id = 51;
            break;
        case e_MAV_CMD_MAV_CMD_DO_FENCE_ENABLE:
            id = 52;
            break;
        case e_MAV_CMD_MAV_CMD_DO_PARACHUTE:
            id = 53;
            break;
        case e_MAV_CMD_MAV_CMD_DO_MOTOR_TEST:
            id = 54;
            break;
        case e_MAV_CMD_MAV_CMD_DO_INVERTED_FLIGHT:
            id = 55;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_SET_YAW_SPEED:
            id = 56;
            break;
        case e_MAV_CMD_MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL:
            id = 57;
            break;
        case e_MAV_CMD_MAV_CMD_DO_MOUNT_CONTROL_QUAT:
            id = 58;
            break;
        case e_MAV_CMD_MAV_CMD_DO_GUIDED_MASTER:
            id = 59;
            break;
        case e_MAV_CMD_MAV_CMD_DO_GUIDED_LIMITS:
            id = 60;
            break;
        case e_MAV_CMD_MAV_CMD_DO_ENGINE_CONTROL:
            id = 61;
            break;
        case e_MAV_CMD_MAV_CMD_DO_LAST:
            id = 62;
            break;
        case e_MAV_CMD_MAV_CMD_PREFLIGHT_CALIBRATION:
            id = 63;
            break;
        case e_MAV_CMD_MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS:
            id = 64;
            break;
        case e_MAV_CMD_MAV_CMD_PREFLIGHT_UAVCAN:
            id = 65;
            break;
        case e_MAV_CMD_MAV_CMD_PREFLIGHT_STORAGE:
            id = 66;
            break;
        case e_MAV_CMD_MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
            id = 67;
            break;
        case e_MAV_CMD_MAV_CMD_OVERRIDE_GOTO:
            id = 68;
            break;
        case e_MAV_CMD_MAV_CMD_MISSION_START:
            id = 69;
            break;
        case e_MAV_CMD_MAV_CMD_COMPONENT_ARM_DISARM:
            id = 70;
            break;
        case e_MAV_CMD_MAV_CMD_GET_HOME_POSITION:
            id = 71;
            break;
        case e_MAV_CMD_MAV_CMD_START_RX_PAIR:
            id = 72;
            break;
        case e_MAV_CMD_MAV_CMD_GET_MESSAGE_INTERVAL:
            id = 73;
            break;
        case e_MAV_CMD_MAV_CMD_SET_MESSAGE_INTERVAL:
            id = 74;
            break;
        case e_MAV_CMD_MAV_CMD_REQUEST_PROTOCOL_VERSION:
            id = 75;
            break;
        case e_MAV_CMD_MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES:
            id = 76;
            break;
        case e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_INFORMATION:
            id = 77;
            break;
        case e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_SETTINGS:
            id = 78;
            break;
        case e_MAV_CMD_MAV_CMD_REQUEST_STORAGE_INFORMATION:
            id = 79;
            break;
        case e_MAV_CMD_MAV_CMD_STORAGE_FORMAT:
            id = 80;
            break;
        case e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS:
            id = 81;
            break;
        case e_MAV_CMD_MAV_CMD_REQUEST_FLIGHT_INFORMATION:
            id = 82;
            break;
        case e_MAV_CMD_MAV_CMD_RESET_CAMERA_SETTINGS:
            id = 83;
            break;
        case e_MAV_CMD_MAV_CMD_SET_CAMERA_MODE:
            id = 84;
            break;
        case e_MAV_CMD_MAV_CMD_IMAGE_START_CAPTURE:
            id = 85;
            break;
        case e_MAV_CMD_MAV_CMD_IMAGE_STOP_CAPTURE:
            id = 86;
            break;
        case e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE:
            id = 87;
            break;
        case e_MAV_CMD_MAV_CMD_DO_TRIGGER_CONTROL:
            id = 88;
            break;
        case e_MAV_CMD_MAV_CMD_VIDEO_START_CAPTURE:
            id = 89;
            break;
        case e_MAV_CMD_MAV_CMD_VIDEO_STOP_CAPTURE:
            id = 90;
            break;
        case e_MAV_CMD_MAV_CMD_VIDEO_START_STREAMING:
            id = 91;
            break;
        case e_MAV_CMD_MAV_CMD_VIDEO_STOP_STREAMING:
            id = 92;
            break;
        case e_MAV_CMD_MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION:
            id = 93;
            break;
        case e_MAV_CMD_MAV_CMD_LOGGING_START:
            id = 94;
            break;
        case e_MAV_CMD_MAV_CMD_LOGGING_STOP:
            id = 95;
            break;
        case e_MAV_CMD_MAV_CMD_AIRFRAME_CONFIGURATION:
            id = 96;
            break;
        case e_MAV_CMD_MAV_CMD_PANORAMA_CREATE:
            id = 97;
            break;
        case e_MAV_CMD_MAV_CMD_DO_VTOL_TRANSITION:
            id = 98;
            break;
        case e_MAV_CMD_MAV_CMD_ARM_AUTHORIZATION_REQUEST:
            id = 99;
            break;
        case e_MAV_CMD_MAV_CMD_SET_GUIDED_SUBMODE_STANDARD:
            id = 100;
            break;
        case e_MAV_CMD_MAV_CMD_SET_GUIDED_SUBMODE_CIRCLE:
            id = 101;
            break;
        case e_MAV_CMD_MAV_CMD_CONDITION_GATE:
            id = 102;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_FENCE_RETURN_POINT:
            id = 103;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION:
            id = 104;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION:
            id = 105;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION:
            id = 106;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION:
            id = 107;
            break;
        case e_MAV_CMD_MAV_CMD_NAV_RALLY_POINT:
            id = 108;
            break;
        case e_MAV_CMD_MAV_CMD_UAVCAN_GET_NODE_INFO:
            id = 109;
            break;
        case e_MAV_CMD_MAV_CMD_PAYLOAD_PREPARE_DEPLOY:
            id = 110;
            break;
        case e_MAV_CMD_MAV_CMD_PAYLOAD_CONTROL_DEPLOY:
            id = 111;
            break;
        case e_MAV_CMD_MAV_CMD_WAYPOINT_USER_1:
            id = 112;
            break;
        case e_MAV_CMD_MAV_CMD_WAYPOINT_USER_2:
            id = 113;
            break;
        case e_MAV_CMD_MAV_CMD_WAYPOINT_USER_3:
            id = 114;
            break;
        case e_MAV_CMD_MAV_CMD_WAYPOINT_USER_4:
            id = 115;
            break;
        case e_MAV_CMD_MAV_CMD_WAYPOINT_USER_5:
            id = 116;
            break;
        case e_MAV_CMD_MAV_CMD_SPATIAL_USER_1:
            id = 117;
            break;
        case e_MAV_CMD_MAV_CMD_SPATIAL_USER_2:
            id = 118;
            break;
        case e_MAV_CMD_MAV_CMD_SPATIAL_USER_3:
            id = 119;
            break;
        case e_MAV_CMD_MAV_CMD_SPATIAL_USER_4:
            id = 120;
            break;
        case e_MAV_CMD_MAV_CMD_SPATIAL_USER_5:
            id = 121;
            break;
        case e_MAV_CMD_MAV_CMD_USER_1:
            id = 122;
            break;
        case e_MAV_CMD_MAV_CMD_USER_2:
            id = 123;
            break;
        case e_MAV_CMD_MAV_CMD_USER_3:
            id = 124;
            break;
        case e_MAV_CMD_MAV_CMD_USER_4:
            id = 125;
            break;
        case e_MAV_CMD_MAV_CMD_USER_5:
            id = 126;
            break;
        case e_MAV_CMD_MAV_CMD_RESET_MPPT:
            id = 127;
            break;
        case e_MAV_CMD_MAV_CMD_PAYLOAD_CONTROL:
            id = 128;
            break;
        default: ;// assert(false);//("Unknown enum" + id);
    }
    set_bits(id, 8, data, 0);
}
INLINER e_MAV_RESULT p77_result_GET(Pack * src)//See MAV_RESULT enu
{
    uint8_t * data = src->data;
    return  0 + (int)get_bits(data, 8, 3);
}
INLINER void p77_result_SET(e_MAV_RESULT  src, Pack * dst)//See MAV_RESULT enu
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 3, data, 8);
}
INLINER uint8_t  p77_progress_TRY(Bounds_Inside * src)
{
    if(src->base.field_bit !=  11 && !try_visit_field(src, 11)) return 0;
    uint8_t * data = src->base.pack->data;
    return ((get_bytes(data,  src->BYTE, 1)));
}
INLINER void p77_progress_SET(uint8_t  src, Bounds_Inside * dst)
{
    if(dst->base.field_bit != 11)insert_field(dst, 11, 0);
    uint8_t * data = dst->base.pack->data;
    set_bytes((src), 1, data,  dst->BYTE);
}
INLINER int32_t  p77_result_param2_TRY(Bounds_Inside * src)
{
    if(src->base.field_bit !=  12 && !try_visit_field(src, 12)) return 0;
    uint8_t * data = src->base.pack->data;
    return ((int32_t)(get_bytes(data,  src->BYTE, 4)));
}
INLINER void p77_result_param2_SET(int32_t  src, Bounds_Inside * dst)
{
    if(dst->base.field_bit != 12)insert_field(dst, 12, 0);
    uint8_t * data = dst->base.pack->data;
    set_bytes((uint32_t)(src), 4, data,  dst->BYTE);
}
INLINER uint8_t  p77_target_system_TRY(Bounds_Inside * src)
{
    if(src->base.field_bit !=  13 && !try_visit_field(src, 13)) return 0;
    uint8_t * data = src->base.pack->data;
    return ((get_bytes(data,  src->BYTE, 1)));
}
INLINER void p77_target_system_SET(uint8_t  src, Bounds_Inside * dst)//WIP: System which requested the command to be execute
{
    if(dst->base.field_bit != 13)insert_field(dst, 13, 0);
    uint8_t * data = dst->base.pack->data;
    set_bytes((src), 1, data,  dst->BYTE);
}
INLINER uint8_t  p77_target_component_TRY(Bounds_Inside * src)
{
    if(src->base.field_bit !=  14 && !try_visit_field(src, 14)) return 0;
    uint8_t * data = src->base.pack->data;
    return ((get_bytes(data,  src->BYTE, 1)));
}
INLINER void p77_target_component_SET(uint8_t  src, Bounds_Inside * dst)//WIP: Component which requested the command to be execute
{
    if(dst->base.field_bit != 14)insert_field(dst, 14, 0);
    uint8_t * data = dst->base.pack->data;
    set_bytes((src), 1, data,  dst->BYTE);
}
INLINER uint32_t p81_time_boot_ms_GET(Pack * src)//Timestamp in milliseconds since system boo
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 4)));
}
INLINER void p81_time_boot_ms_SET(uint32_t  src, Pack * dst)//Timestamp in milliseconds since system boo
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  0);
}
INLINER float p81_roll_GET(Pack * src)//Desired roll rate in radians per secon
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  4, 4)));
}
INLINER void p81_roll_SET(float  src, Pack * dst)//Desired roll rate in radians per secon
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  4);
}
INLINER float p81_pitch_GET(Pack * src)//Desired pitch rate in radians per secon
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  8, 4)));
}
INLINER void p81_pitch_SET(float  src, Pack * dst)//Desired pitch rate in radians per secon
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  8);
}
INLINER float p81_yaw_GET(Pack * src)//Desired yaw rate in radians per secon
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  12, 4)));
}
INLINER void p81_yaw_SET(float  src, Pack * dst)//Desired yaw rate in radians per secon
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  12);
}
INLINER float p81_thrust_GET(Pack * src)//Collective thrust, normalized to 0 ..
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  16, 4)));
}
INLINER void p81_thrust_SET(float  src, Pack * dst)//Collective thrust, normalized to 0 ..
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  16);
}
INLINER uint8_t p81_mode_switch_GET(Pack * src)//Flight mode switch position, 0.. 25
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  20, 1)));
}
INLINER void p81_mode_switch_SET(uint8_t  src, Pack * dst)//Flight mode switch position, 0.. 25
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  20);
}
INLINER uint8_t p81_manual_override_switch_GET(Pack * src)//Override mode switch position, 0.. 25
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  21, 1)));
}
INLINER void p81_manual_override_switch_SET(uint8_t  src, Pack * dst)//Override mode switch position, 0.. 25
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  21);
}
INLINER uint32_t p82_time_boot_ms_GET(Pack * src)//Timestamp in milliseconds since system boo
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 4)));
}
INLINER void p82_time_boot_ms_SET(uint32_t  src, Pack * dst)//Timestamp in milliseconds since system boo
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  0);
}
INLINER uint8_t p82_target_system_GET(Pack * src)//System I
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  4, 1)));
}
INLINER void p82_target_system_SET(uint8_t  src, Pack * dst)//System I
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  4);
}
INLINER uint8_t p82_target_component_GET(Pack * src)//Component I
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  5, 1)));
}
INLINER void p82_target_component_SET(uint8_t  src, Pack * dst)//Component I
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  5);
}
INLINER uint8_t p82_type_mask_GET(Pack * src)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  6, 1)));
}
INLINER void p82_type_mask_SET(uint8_t  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  6);
}
INLINER float* p82_q_GET(Pack * src, float*  dst, int32_t pos) //Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0
{
    uint8_t * data = src->data;
    for(int32_t BYTE = 7, dst_max = pos + 4; pos < dst_max ; pos++,  BYTE += 4)
        dst[pos] = (intBitsToFloat(get_bytes(data,  BYTE, 4)));
    return dst;
}

static const  uint32_t p82_q_LEN = 4; //return array length

INLINER  float*  p82_q_GET_(Pack * src) {return p82_q_GET(src, malloc(4 * sizeof(float)), 0);}
INLINER void p82_q_SET(float*  src, int32_t pos, Pack * dst) //Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  7, src_max = pos + 4; pos < src_max; pos++, BYTE += 4)
        set_bytes(floatToIntBits(src[pos]), 4, data,  BYTE);
}
INLINER float p82_body_roll_rate_GET(Pack * src)//Body roll rate in radians per secon
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  23, 4)));
}
INLINER void p82_body_roll_rate_SET(float  src, Pack * dst)//Body roll rate in radians per secon
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  23);
}
INLINER float p82_body_pitch_rate_GET(Pack * src)//Body roll rate in radians per secon
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  27, 4)));
}
INLINER void p82_body_pitch_rate_SET(float  src, Pack * dst)//Body roll rate in radians per secon
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  27);
}
INLINER float p82_body_yaw_rate_GET(Pack * src)//Body roll rate in radians per secon
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  31, 4)));
}
INLINER void p82_body_yaw_rate_SET(float  src, Pack * dst)//Body roll rate in radians per secon
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  31);
}
INLINER float p82_thrust_GET(Pack * src)//Collective thrust, normalized to 0 .. 1 (-1 .. 1 for vehicles capable of reverse trust
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  35, 4)));
}
INLINER void p82_thrust_SET(float  src, Pack * dst)//Collective thrust, normalized to 0 .. 1 (-1 .. 1 for vehicles capable of reverse trust
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  35);
}
INLINER uint32_t p83_time_boot_ms_GET(Pack * src)//Timestamp in milliseconds since system boo
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 4)));
}
INLINER void p83_time_boot_ms_SET(uint32_t  src, Pack * dst)//Timestamp in milliseconds since system boo
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  0);
}
INLINER uint8_t p83_type_mask_GET(Pack * src)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  4, 1)));
}
INLINER void p83_type_mask_SET(uint8_t  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  4);
}
INLINER float* p83_q_GET(Pack * src, float*  dst, int32_t pos) //Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0
{
    uint8_t * data = src->data;
    for(int32_t BYTE = 5, dst_max = pos + 4; pos < dst_max ; pos++,  BYTE += 4)
        dst[pos] = (intBitsToFloat(get_bytes(data,  BYTE, 4)));
    return dst;
}

static const  uint32_t p83_q_LEN = 4; //return array length

INLINER  float*  p83_q_GET_(Pack * src) {return p83_q_GET(src, malloc(4 * sizeof(float)), 0);}
INLINER void p83_q_SET(float*  src, int32_t pos, Pack * dst) //Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  5, src_max = pos + 4; pos < src_max; pos++, BYTE += 4)
        set_bytes(floatToIntBits(src[pos]), 4, data,  BYTE);
}
INLINER float p83_body_roll_rate_GET(Pack * src)//Body roll rate in radians per secon
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  21, 4)));
}
INLINER void p83_body_roll_rate_SET(float  src, Pack * dst)//Body roll rate in radians per secon
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  21);
}
INLINER float p83_body_pitch_rate_GET(Pack * src)//Body pitch rate in radians per secon
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  25, 4)));
}
INLINER void p83_body_pitch_rate_SET(float  src, Pack * dst)//Body pitch rate in radians per secon
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  25);
}
INLINER float p83_body_yaw_rate_GET(Pack * src)//Body yaw rate in radians per secon
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  29, 4)));
}
INLINER void p83_body_yaw_rate_SET(float  src, Pack * dst)//Body yaw rate in radians per secon
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  29);
}
INLINER float p83_thrust_GET(Pack * src)//Collective thrust, normalized to 0 .. 1 (-1 .. 1 for vehicles capable of reverse trust
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  33, 4)));
}
INLINER void p83_thrust_SET(float  src, Pack * dst)//Collective thrust, normalized to 0 .. 1 (-1 .. 1 for vehicles capable of reverse trust
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  33);
}
INLINER uint16_t p84_type_mask_GET(Pack * src)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER void p84_type_mask_SET(uint16_t  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER uint32_t p84_time_boot_ms_GET(Pack * src)//Timestamp in milliseconds since system boo
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 4)));
}
INLINER void p84_time_boot_ms_SET(uint32_t  src, Pack * dst)//Timestamp in milliseconds since system boo
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  2);
}
INLINER uint8_t p84_target_system_GET(Pack * src)//System I
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  6, 1)));
}
INLINER void p84_target_system_SET(uint8_t  src, Pack * dst)//System I
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  6);
}
INLINER uint8_t p84_target_component_GET(Pack * src)//Component I
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  7, 1)));
}
INLINER void p84_target_component_SET(uint8_t  src, Pack * dst)//Component I
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  7);
}
INLINER float p84_x_GET(Pack * src)//X Position in NED frame in meter
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  8, 4)));
}
INLINER void p84_x_SET(float  src, Pack * dst)//X Position in NED frame in meter
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  8);
}
INLINER float p84_y_GET(Pack * src)//Y Position in NED frame in meter
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  12, 4)));
}
INLINER void p84_y_SET(float  src, Pack * dst)//Y Position in NED frame in meter
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  12);
}
INLINER float p84_z_GET(Pack * src)//Z Position in NED frame in meters (note, altitude is negative in NED
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  16, 4)));
}
INLINER void p84_z_SET(float  src, Pack * dst)//Z Position in NED frame in meters (note, altitude is negative in NED
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  16);
}
INLINER float p84_vx_GET(Pack * src)//X velocity in NED frame in meter /
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  20, 4)));
}
INLINER void p84_vx_SET(float  src, Pack * dst)//X velocity in NED frame in meter /
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  20);
}
INLINER float p84_vy_GET(Pack * src)//Y velocity in NED frame in meter /
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  24, 4)));
}
INLINER void p84_vy_SET(float  src, Pack * dst)//Y velocity in NED frame in meter /
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  24);
}
INLINER float p84_vz_GET(Pack * src)//Z velocity in NED frame in meter /
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  28, 4)));
}
INLINER void p84_vz_SET(float  src, Pack * dst)//Z velocity in NED frame in meter /
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  28);
}
INLINER float p84_afx_GET(Pack * src)//X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  32, 4)));
}
INLINER void p84_afx_SET(float  src, Pack * dst)//X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  32);
}
INLINER float p84_afy_GET(Pack * src)//Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  36, 4)));
}
INLINER void p84_afy_SET(float  src, Pack * dst)//Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  36);
}
INLINER float p84_afz_GET(Pack * src)//Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  40, 4)));
}
INLINER void p84_afz_SET(float  src, Pack * dst)//Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  40);
}
INLINER float p84_yaw_GET(Pack * src)//yaw setpoint in ra
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  44, 4)));
}
INLINER void p84_yaw_SET(float  src, Pack * dst)//yaw setpoint in ra
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  44);
}
INLINER float p84_yaw_rate_GET(Pack * src)//yaw rate setpoint in rad/
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  48, 4)));
}
INLINER void p84_yaw_rate_SET(float  src, Pack * dst)//yaw rate setpoint in rad/
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  48);
}
INLINER e_MAV_FRAME p84_coordinate_frame_GET(Pack * src)
{
    uint8_t * data = src->data;
    return  0 + (int)get_bits(data, 416, 4);
}
INLINER void p84_coordinate_frame_SET(e_MAV_FRAME  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 4, data, 416);
}
INLINER uint16_t p86_type_mask_GET(Pack * src)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER void p86_type_mask_SET(uint16_t  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER uint32_t p86_time_boot_ms_GET(Pack * src)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 4)));
}
INLINER void p86_time_boot_ms_SET(uint32_t  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  2);
}
INLINER uint8_t p86_target_system_GET(Pack * src)//System I
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  6, 1)));
}
INLINER void p86_target_system_SET(uint8_t  src, Pack * dst)//System I
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  6);
}
INLINER uint8_t p86_target_component_GET(Pack * src)//Component I
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  7, 1)));
}
INLINER void p86_target_component_SET(uint8_t  src, Pack * dst)//Component I
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  7);
}
INLINER int32_t p86_lat_int_GET(Pack * src)//X Position in WGS84 frame in 1e7 * meter
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  8, 4)));
}
INLINER void p86_lat_int_SET(int32_t  src, Pack * dst)//X Position in WGS84 frame in 1e7 * meter
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  8);
}
INLINER int32_t p86_lon_int_GET(Pack * src)//Y Position in WGS84 frame in 1e7 * meter
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  12, 4)));
}
INLINER void p86_lon_int_SET(int32_t  src, Pack * dst)//Y Position in WGS84 frame in 1e7 * meter
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  12);
}
INLINER float p86_alt_GET(Pack * src)//Altitude in meters in AMSL altitude, not WGS84 if absolute or relative, above terrain if GLOBAL_TERRAIN_ALT_I
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  16, 4)));
}
INLINER void p86_alt_SET(float  src, Pack * dst)//Altitude in meters in AMSL altitude, not WGS84 if absolute or relative, above terrain if GLOBAL_TERRAIN_ALT_I
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  16);
}
INLINER float p86_vx_GET(Pack * src)//X velocity in NED frame in meter /
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  20, 4)));
}
INLINER void p86_vx_SET(float  src, Pack * dst)//X velocity in NED frame in meter /
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  20);
}
INLINER float p86_vy_GET(Pack * src)//Y velocity in NED frame in meter /
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  24, 4)));
}
INLINER void p86_vy_SET(float  src, Pack * dst)//Y velocity in NED frame in meter /
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  24);
}
INLINER float p86_vz_GET(Pack * src)//Z velocity in NED frame in meter /
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  28, 4)));
}
INLINER void p86_vz_SET(float  src, Pack * dst)//Z velocity in NED frame in meter /
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  28);
}
INLINER float p86_afx_GET(Pack * src)//X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  32, 4)));
}
INLINER void p86_afx_SET(float  src, Pack * dst)//X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  32);
}
INLINER float p86_afy_GET(Pack * src)//Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  36, 4)));
}
INLINER void p86_afy_SET(float  src, Pack * dst)//Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  36);
}
INLINER float p86_afz_GET(Pack * src)//Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  40, 4)));
}
INLINER void p86_afz_SET(float  src, Pack * dst)//Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  40);
}
INLINER float p86_yaw_GET(Pack * src)//yaw setpoint in ra
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  44, 4)));
}
INLINER void p86_yaw_SET(float  src, Pack * dst)//yaw setpoint in ra
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  44);
}
INLINER float p86_yaw_rate_GET(Pack * src)//yaw rate setpoint in rad/
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  48, 4)));
}
INLINER void p86_yaw_rate_SET(float  src, Pack * dst)//yaw rate setpoint in rad/
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  48);
}
INLINER e_MAV_FRAME p86_coordinate_frame_GET(Pack * src)
{
    uint8_t * data = src->data;
    return  0 + (int)get_bits(data, 416, 4);
}
INLINER void p86_coordinate_frame_SET(e_MAV_FRAME  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 4, data, 416);
}
INLINER uint16_t p87_type_mask_GET(Pack * src)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER void p87_type_mask_SET(uint16_t  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER uint32_t p87_time_boot_ms_GET(Pack * src)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 4)));
}
INLINER void p87_time_boot_ms_SET(uint32_t  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  2);
}
INLINER int32_t p87_lat_int_GET(Pack * src)//X Position in WGS84 frame in 1e7 * meter
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  6, 4)));
}
INLINER void p87_lat_int_SET(int32_t  src, Pack * dst)//X Position in WGS84 frame in 1e7 * meter
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  6);
}
INLINER int32_t p87_lon_int_GET(Pack * src)//Y Position in WGS84 frame in 1e7 * meter
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  10, 4)));
}
INLINER void p87_lon_int_SET(int32_t  src, Pack * dst)//Y Position in WGS84 frame in 1e7 * meter
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  10);
}
INLINER float p87_alt_GET(Pack * src)//Altitude in meters in AMSL altitude, not WGS84 if absolute or relative, above terrain if GLOBAL_TERRAIN_ALT_I
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  14, 4)));
}
INLINER void p87_alt_SET(float  src, Pack * dst)//Altitude in meters in AMSL altitude, not WGS84 if absolute or relative, above terrain if GLOBAL_TERRAIN_ALT_I
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  14);
}
INLINER float p87_vx_GET(Pack * src)//X velocity in NED frame in meter /
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  18, 4)));
}
INLINER void p87_vx_SET(float  src, Pack * dst)//X velocity in NED frame in meter /
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  18);
}
INLINER float p87_vy_GET(Pack * src)//Y velocity in NED frame in meter /
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  22, 4)));
}
INLINER void p87_vy_SET(float  src, Pack * dst)//Y velocity in NED frame in meter /
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  22);
}
INLINER float p87_vz_GET(Pack * src)//Z velocity in NED frame in meter /
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  26, 4)));
}
INLINER void p87_vz_SET(float  src, Pack * dst)//Z velocity in NED frame in meter /
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  26);
}
INLINER float p87_afx_GET(Pack * src)//X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  30, 4)));
}
INLINER void p87_afx_SET(float  src, Pack * dst)//X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  30);
}
INLINER float p87_afy_GET(Pack * src)//Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  34, 4)));
}
INLINER void p87_afy_SET(float  src, Pack * dst)//Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  34);
}
INLINER float p87_afz_GET(Pack * src)//Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  38, 4)));
}
INLINER void p87_afz_SET(float  src, Pack * dst)//Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  38);
}
INLINER float p87_yaw_GET(Pack * src)//yaw setpoint in ra
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  42, 4)));
}
INLINER void p87_yaw_SET(float  src, Pack * dst)//yaw setpoint in ra
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  42);
}
INLINER float p87_yaw_rate_GET(Pack * src)//yaw rate setpoint in rad/
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  46, 4)));
}
INLINER void p87_yaw_rate_SET(float  src, Pack * dst)//yaw rate setpoint in rad/
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  46);
}
INLINER e_MAV_FRAME p87_coordinate_frame_GET(Pack * src)
{
    uint8_t * data = src->data;
    return  0 + (int)get_bits(data, 400, 4);
}
INLINER void p87_coordinate_frame_SET(e_MAV_FRAME  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 4, data, 400);
}
INLINER uint32_t p89_time_boot_ms_GET(Pack * src)//Timestamp (milliseconds since system boot
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 4)));
}
INLINER void p89_time_boot_ms_SET(uint32_t  src, Pack * dst)//Timestamp (milliseconds since system boot
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  0);
}
INLINER float p89_x_GET(Pack * src)//X Positio
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  4, 4)));
}
INLINER void p89_x_SET(float  src, Pack * dst)//X Positio
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  4);
}
INLINER float p89_y_GET(Pack * src)//Y Positio
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  8, 4)));
}
INLINER void p89_y_SET(float  src, Pack * dst)//Y Positio
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  8);
}
INLINER float p89_z_GET(Pack * src)//Z Positio
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  12, 4)));
}
INLINER void p89_z_SET(float  src, Pack * dst)//Z Positio
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  12);
}
INLINER float p89_roll_GET(Pack * src)//Rol
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  16, 4)));
}
INLINER void p89_roll_SET(float  src, Pack * dst)//Rol
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  16);
}
INLINER float p89_pitch_GET(Pack * src)//Pitc
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  20, 4)));
}
INLINER void p89_pitch_SET(float  src, Pack * dst)//Pitc
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  20);
}
INLINER float p89_yaw_GET(Pack * src)//Ya
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  24, 4)));
}
INLINER void p89_yaw_SET(float  src, Pack * dst)//Ya
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  24);
}
INLINER uint64_t p90_time_usec_GET(Pack * src)//Timestamp (microseconds since UNIX epoch or microseconds since system boot
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 8)));
}
INLINER void p90_time_usec_SET(uint64_t  src, Pack * dst)//Timestamp (microseconds since UNIX epoch or microseconds since system boot
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  0);
}
INLINER float p90_roll_GET(Pack * src)//Roll angle (rad
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  8, 4)));
}
INLINER void p90_roll_SET(float  src, Pack * dst)//Roll angle (rad
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  8);
}
INLINER float p90_pitch_GET(Pack * src)//Pitch angle (rad
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  12, 4)));
}
INLINER void p90_pitch_SET(float  src, Pack * dst)//Pitch angle (rad
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  12);
}
INLINER float p90_yaw_GET(Pack * src)//Yaw angle (rad
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  16, 4)));
}
INLINER void p90_yaw_SET(float  src, Pack * dst)//Yaw angle (rad
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  16);
}
INLINER float p90_rollspeed_GET(Pack * src)//Body frame roll / phi angular speed (rad/s
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  20, 4)));
}
INLINER void p90_rollspeed_SET(float  src, Pack * dst)//Body frame roll / phi angular speed (rad/s
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  20);
}
INLINER float p90_pitchspeed_GET(Pack * src)//Body frame pitch / theta angular speed (rad/s
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  24, 4)));
}
INLINER void p90_pitchspeed_SET(float  src, Pack * dst)//Body frame pitch / theta angular speed (rad/s
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  24);
}
INLINER float p90_yawspeed_GET(Pack * src)//Body frame yaw / psi angular speed (rad/s
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  28, 4)));
}
INLINER void p90_yawspeed_SET(float  src, Pack * dst)//Body frame yaw / psi angular speed (rad/s
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  28);
}
INLINER int32_t p90_lat_GET(Pack * src)//Latitude, expressed as * 1E
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  32, 4)));
}
INLINER void p90_lat_SET(int32_t  src, Pack * dst)//Latitude, expressed as * 1E
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  32);
}
INLINER int32_t p90_lon_GET(Pack * src)//Longitude, expressed as * 1E
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  36, 4)));
}
INLINER void p90_lon_SET(int32_t  src, Pack * dst)//Longitude, expressed as * 1E
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  36);
}
INLINER int32_t p90_alt_GET(Pack * src)//Altitude in meters, expressed as * 1000 (millimeters
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  40, 4)));
}
INLINER void p90_alt_SET(int32_t  src, Pack * dst)//Altitude in meters, expressed as * 1000 (millimeters
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  40);
}
INLINER int16_t p90_vx_GET(Pack * src)//Ground X Speed (Latitude), expressed as m/s * 10
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  44, 2)));
}
INLINER void p90_vx_SET(int16_t  src, Pack * dst)//Ground X Speed (Latitude), expressed as m/s * 10
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  44);
}
INLINER int16_t p90_vy_GET(Pack * src)//Ground Y Speed (Longitude), expressed as m/s * 10
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  46, 2)));
}
INLINER void p90_vy_SET(int16_t  src, Pack * dst)//Ground Y Speed (Longitude), expressed as m/s * 10
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  46);
}
INLINER int16_t p90_vz_GET(Pack * src)//Ground Z Speed (Altitude), expressed as m/s * 10
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  48, 2)));
}
INLINER void p90_vz_SET(int16_t  src, Pack * dst)//Ground Z Speed (Altitude), expressed as m/s * 10
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  48);
}
INLINER int16_t p90_xacc_GET(Pack * src)//X acceleration (mg
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  50, 2)));
}
INLINER void p90_xacc_SET(int16_t  src, Pack * dst)//X acceleration (mg
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  50);
}
INLINER int16_t p90_yacc_GET(Pack * src)//Y acceleration (mg
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  52, 2)));
}
INLINER void p90_yacc_SET(int16_t  src, Pack * dst)//Y acceleration (mg
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  52);
}
INLINER int16_t p90_zacc_GET(Pack * src)//Z acceleration (mg
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  54, 2)));
}
INLINER void p90_zacc_SET(int16_t  src, Pack * dst)//Z acceleration (mg
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  54);
}
INLINER uint64_t p91_time_usec_GET(Pack * src)//Timestamp (microseconds since UNIX epoch or microseconds since system boot
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 8)));
}
INLINER void p91_time_usec_SET(uint64_t  src, Pack * dst)//Timestamp (microseconds since UNIX epoch or microseconds since system boot
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  0);
}
INLINER float p91_roll_ailerons_GET(Pack * src)//Control output -1 ..
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  8, 4)));
}
INLINER void p91_roll_ailerons_SET(float  src, Pack * dst)//Control output -1 ..
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  8);
}
INLINER float p91_pitch_elevator_GET(Pack * src)//Control output -1 ..
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  12, 4)));
}
INLINER void p91_pitch_elevator_SET(float  src, Pack * dst)//Control output -1 ..
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  12);
}
INLINER float p91_yaw_rudder_GET(Pack * src)//Control output -1 ..
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  16, 4)));
}
INLINER void p91_yaw_rudder_SET(float  src, Pack * dst)//Control output -1 ..
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  16);
}
INLINER float p91_throttle_GET(Pack * src)//Throttle 0 ..
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  20, 4)));
}
INLINER void p91_throttle_SET(float  src, Pack * dst)//Throttle 0 ..
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  20);
}
INLINER float p91_aux1_GET(Pack * src)//Aux 1, -1 ..
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  24, 4)));
}
INLINER void p91_aux1_SET(float  src, Pack * dst)//Aux 1, -1 ..
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  24);
}
INLINER float p91_aux2_GET(Pack * src)//Aux 2, -1 ..
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  28, 4)));
}
INLINER void p91_aux2_SET(float  src, Pack * dst)//Aux 2, -1 ..
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  28);
}
INLINER float p91_aux3_GET(Pack * src)//Aux 3, -1 ..
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  32, 4)));
}
INLINER void p91_aux3_SET(float  src, Pack * dst)//Aux 3, -1 ..
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  32);
}
INLINER float p91_aux4_GET(Pack * src)//Aux 4, -1 ..
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  36, 4)));
}
INLINER void p91_aux4_SET(float  src, Pack * dst)//Aux 4, -1 ..
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  36);
}
INLINER uint8_t p91_nav_mode_GET(Pack * src)//Navigation mode (MAV_NAV_MODE
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  40, 1)));
}
INLINER void p91_nav_mode_SET(uint8_t  src, Pack * dst)//Navigation mode (MAV_NAV_MODE
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  40);
}
INLINER e_MAV_MODE p91_mode_GET(Pack * src)//System mode (MAV_MODE
{
    uint8_t * data = src->data;
    switch(get_bits(data, 328, 4))
    {
        case 0:
            return e_MAV_MODE_MAV_MODE_PREFLIGHT;
        case 1:
            return e_MAV_MODE_MAV_MODE_MANUAL_DISARMED;
        case 2:
            return e_MAV_MODE_MAV_MODE_TEST_DISARMED;
        case 3:
            return e_MAV_MODE_MAV_MODE_STABILIZE_DISARMED;
        case 4:
            return e_MAV_MODE_MAV_MODE_GUIDED_DISARMED;
        case 5:
            return e_MAV_MODE_MAV_MODE_AUTO_DISARMED;
        case 6:
            return e_MAV_MODE_MAV_MODE_MANUAL_ARMED;
        case 7:
            return e_MAV_MODE_MAV_MODE_TEST_ARMED;
        case 8:
            return e_MAV_MODE_MAV_MODE_STABILIZE_ARMED;
        case 9:
            return e_MAV_MODE_MAV_MODE_GUIDED_ARMED;
        case 10:
            return e_MAV_MODE_MAV_MODE_AUTO_ARMED;
        default: ;//assert(false);//("Unknown enum ID " + id);
    }
}
INLINER void p91_mode_SET(e_MAV_MODE  src, Pack * dst)//System mode (MAV_MODE
{
    uint8_t * data = dst->data;
    int32_t id;
    switch(src)
    {
        case e_MAV_MODE_MAV_MODE_PREFLIGHT:
            id = 0;
            break;
        case e_MAV_MODE_MAV_MODE_MANUAL_DISARMED:
            id = 1;
            break;
        case e_MAV_MODE_MAV_MODE_TEST_DISARMED:
            id = 2;
            break;
        case e_MAV_MODE_MAV_MODE_STABILIZE_DISARMED:
            id = 3;
            break;
        case e_MAV_MODE_MAV_MODE_GUIDED_DISARMED:
            id = 4;
            break;
        case e_MAV_MODE_MAV_MODE_AUTO_DISARMED:
            id = 5;
            break;
        case e_MAV_MODE_MAV_MODE_MANUAL_ARMED:
            id = 6;
            break;
        case e_MAV_MODE_MAV_MODE_TEST_ARMED:
            id = 7;
            break;
        case e_MAV_MODE_MAV_MODE_STABILIZE_ARMED:
            id = 8;
            break;
        case e_MAV_MODE_MAV_MODE_GUIDED_ARMED:
            id = 9;
            break;
        case e_MAV_MODE_MAV_MODE_AUTO_ARMED:
            id = 10;
            break;
        default: ;// assert(false);//("Unknown enum" + id);
    }
    set_bits(id, 4, data, 328);
}
INLINER uint16_t p92_chan1_raw_GET(Pack * src)//RC channel 1 value, in microsecond
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER void p92_chan1_raw_SET(uint16_t  src, Pack * dst)//RC channel 1 value, in microsecond
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER uint16_t p92_chan2_raw_GET(Pack * src)//RC channel 2 value, in microsecond
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 2)));
}
INLINER void p92_chan2_raw_SET(uint16_t  src, Pack * dst)//RC channel 2 value, in microsecond
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  2);
}
INLINER uint16_t p92_chan3_raw_GET(Pack * src)//RC channel 3 value, in microsecond
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  4, 2)));
}
INLINER void p92_chan3_raw_SET(uint16_t  src, Pack * dst)//RC channel 3 value, in microsecond
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  4);
}
INLINER uint16_t p92_chan4_raw_GET(Pack * src)//RC channel 4 value, in microsecond
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  6, 2)));
}
INLINER void p92_chan4_raw_SET(uint16_t  src, Pack * dst)//RC channel 4 value, in microsecond
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  6);
}
INLINER uint16_t p92_chan5_raw_GET(Pack * src)//RC channel 5 value, in microsecond
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  8, 2)));
}
INLINER void p92_chan5_raw_SET(uint16_t  src, Pack * dst)//RC channel 5 value, in microsecond
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  8);
}
INLINER uint16_t p92_chan6_raw_GET(Pack * src)//RC channel 6 value, in microsecond
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  10, 2)));
}
INLINER void p92_chan6_raw_SET(uint16_t  src, Pack * dst)//RC channel 6 value, in microsecond
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  10);
}
INLINER uint16_t p92_chan7_raw_GET(Pack * src)//RC channel 7 value, in microsecond
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  12, 2)));
}
INLINER void p92_chan7_raw_SET(uint16_t  src, Pack * dst)//RC channel 7 value, in microsecond
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  12);
}
INLINER uint16_t p92_chan8_raw_GET(Pack * src)//RC channel 8 value, in microsecond
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  14, 2)));
}
INLINER void p92_chan8_raw_SET(uint16_t  src, Pack * dst)//RC channel 8 value, in microsecond
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  14);
}
INLINER uint16_t p92_chan9_raw_GET(Pack * src)//RC channel 9 value, in microsecond
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  16, 2)));
}
INLINER void p92_chan9_raw_SET(uint16_t  src, Pack * dst)//RC channel 9 value, in microsecond
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  16);
}
INLINER uint16_t p92_chan10_raw_GET(Pack * src)//RC channel 10 value, in microsecond
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  18, 2)));
}
INLINER void p92_chan10_raw_SET(uint16_t  src, Pack * dst)//RC channel 10 value, in microsecond
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  18);
}
INLINER uint16_t p92_chan11_raw_GET(Pack * src)//RC channel 11 value, in microsecond
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  20, 2)));
}
INLINER void p92_chan11_raw_SET(uint16_t  src, Pack * dst)//RC channel 11 value, in microsecond
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  20);
}
INLINER uint16_t p92_chan12_raw_GET(Pack * src)//RC channel 12 value, in microsecond
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  22, 2)));
}
INLINER void p92_chan12_raw_SET(uint16_t  src, Pack * dst)//RC channel 12 value, in microsecond
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  22);
}
INLINER uint64_t p92_time_usec_GET(Pack * src)//Timestamp (microseconds since UNIX epoch or microseconds since system boot
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  24, 8)));
}
INLINER void p92_time_usec_SET(uint64_t  src, Pack * dst)//Timestamp (microseconds since UNIX epoch or microseconds since system boot
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  24);
}
INLINER uint8_t p92_rssi_GET(Pack * src)//Receive signal strength indicator, 0: 0%, 255: 100
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  32, 1)));
}
INLINER void p92_rssi_SET(uint8_t  src, Pack * dst)//Receive signal strength indicator, 0: 0%, 255: 100
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  32);
}
INLINER uint64_t p93_time_usec_GET(Pack * src)//Timestamp (microseconds since UNIX epoch or microseconds since system boot
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 8)));
}
INLINER void p93_time_usec_SET(uint64_t  src, Pack * dst)//Timestamp (microseconds since UNIX epoch or microseconds since system boot
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  0);
}
INLINER uint64_t p93_flags_GET(Pack * src)//Flags as bitfield, reserved for future use
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  8, 8)));
}
INLINER void p93_flags_SET(uint64_t  src, Pack * dst)//Flags as bitfield, reserved for future use
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  8);
}
INLINER float* p93_controls_GET(Pack * src, float*  dst, int32_t pos) //Control outputs -1 .. 1. Channel assignment depends on the simulated hardware
{
    uint8_t * data = src->data;
    for(int32_t BYTE = 16, dst_max = pos + 16; pos < dst_max ; pos++,  BYTE += 4)
        dst[pos] = (intBitsToFloat(get_bytes(data,  BYTE, 4)));
    return dst;
}

static const  uint32_t p93_controls_LEN = 16; //return array length

INLINER  float*  p93_controls_GET_(Pack * src) {return p93_controls_GET(src, malloc(16 * sizeof(float)), 0);}
INLINER void p93_controls_SET(float*  src, int32_t pos, Pack * dst) //Control outputs -1 .. 1. Channel assignment depends on the simulated hardware
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  16, src_max = pos + 16; pos < src_max; pos++, BYTE += 4)
        set_bytes(floatToIntBits(src[pos]), 4, data,  BYTE);
}
INLINER e_MAV_MODE p93_mode_GET(Pack * src)//System mode (MAV_MODE), includes arming state
{
    uint8_t * data = src->data;
    switch(get_bits(data, 640, 4))
    {
        case 0:
            return e_MAV_MODE_MAV_MODE_PREFLIGHT;
        case 1:
            return e_MAV_MODE_MAV_MODE_MANUAL_DISARMED;
        case 2:
            return e_MAV_MODE_MAV_MODE_TEST_DISARMED;
        case 3:
            return e_MAV_MODE_MAV_MODE_STABILIZE_DISARMED;
        case 4:
            return e_MAV_MODE_MAV_MODE_GUIDED_DISARMED;
        case 5:
            return e_MAV_MODE_MAV_MODE_AUTO_DISARMED;
        case 6:
            return e_MAV_MODE_MAV_MODE_MANUAL_ARMED;
        case 7:
            return e_MAV_MODE_MAV_MODE_TEST_ARMED;
        case 8:
            return e_MAV_MODE_MAV_MODE_STABILIZE_ARMED;
        case 9:
            return e_MAV_MODE_MAV_MODE_GUIDED_ARMED;
        case 10:
            return e_MAV_MODE_MAV_MODE_AUTO_ARMED;
        default: ;//assert(false);//("Unknown enum ID " + id);
    }
}
INLINER void p93_mode_SET(e_MAV_MODE  src, Pack * dst)//System mode (MAV_MODE), includes arming state
{
    uint8_t * data = dst->data;
    int32_t id;
    switch(src)
    {
        case e_MAV_MODE_MAV_MODE_PREFLIGHT:
            id = 0;
            break;
        case e_MAV_MODE_MAV_MODE_MANUAL_DISARMED:
            id = 1;
            break;
        case e_MAV_MODE_MAV_MODE_TEST_DISARMED:
            id = 2;
            break;
        case e_MAV_MODE_MAV_MODE_STABILIZE_DISARMED:
            id = 3;
            break;
        case e_MAV_MODE_MAV_MODE_GUIDED_DISARMED:
            id = 4;
            break;
        case e_MAV_MODE_MAV_MODE_AUTO_DISARMED:
            id = 5;
            break;
        case e_MAV_MODE_MAV_MODE_MANUAL_ARMED:
            id = 6;
            break;
        case e_MAV_MODE_MAV_MODE_TEST_ARMED:
            id = 7;
            break;
        case e_MAV_MODE_MAV_MODE_STABILIZE_ARMED:
            id = 8;
            break;
        case e_MAV_MODE_MAV_MODE_GUIDED_ARMED:
            id = 9;
            break;
        case e_MAV_MODE_MAV_MODE_AUTO_ARMED:
            id = 10;
            break;
        default: ;// assert(false);//("Unknown enum" + id);
    }
    set_bits(id, 4, data, 640);
}
INLINER uint64_t p100_time_usec_GET(Pack * src)//Timestamp (UNIX
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 8)));
}
INLINER void p100_time_usec_SET(uint64_t  src, Pack * dst)//Timestamp (UNIX
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  0);
}
INLINER uint8_t p100_sensor_id_GET(Pack * src)//Sensor I
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  8, 1)));
}
INLINER void p100_sensor_id_SET(uint8_t  src, Pack * dst)//Sensor I
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  8);
}
INLINER int16_t p100_flow_x_GET(Pack * src)//Flow in pixels * 10 in x-sensor direction (dezi-pixels
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  9, 2)));
}
INLINER void p100_flow_x_SET(int16_t  src, Pack * dst)//Flow in pixels * 10 in x-sensor direction (dezi-pixels
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  9);
}
INLINER int16_t p100_flow_y_GET(Pack * src)//Flow in pixels * 10 in y-sensor direction (dezi-pixels
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  11, 2)));
}
INLINER void p100_flow_y_SET(int16_t  src, Pack * dst)//Flow in pixels * 10 in y-sensor direction (dezi-pixels
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  11);
}
INLINER float p100_flow_comp_m_x_GET(Pack * src)//Flow in meters in x-sensor direction, angular-speed compensate
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  13, 4)));
}
INLINER void p100_flow_comp_m_x_SET(float  src, Pack * dst)//Flow in meters in x-sensor direction, angular-speed compensate
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  13);
}
INLINER float p100_flow_comp_m_y_GET(Pack * src)//Flow in meters in y-sensor direction, angular-speed compensate
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  17, 4)));
}
INLINER void p100_flow_comp_m_y_SET(float  src, Pack * dst)//Flow in meters in y-sensor direction, angular-speed compensate
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  17);
}
INLINER uint8_t p100_quality_GET(Pack * src)//Optical flow quality / confidence. 0: bad, 255: maximum qualit
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  21, 1)));
}
INLINER void p100_quality_SET(uint8_t  src, Pack * dst)//Optical flow quality / confidence. 0: bad, 255: maximum qualit
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  21);
}
INLINER float p100_ground_distance_GET(Pack * src)//Ground distance in meters. Positive value: distance known. Negative value: Unknown distanc
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  22, 4)));
}
INLINER void p100_ground_distance_SET(float  src, Pack * dst)//Ground distance in meters. Positive value: distance known. Negative value: Unknown distanc
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  22);
}
INLINER float  p100_flow_rate_x_TRY(Bounds_Inside * src)
{
    if(src->base.field_bit !=  208 && !try_visit_field(src, 208)) return 0;
    uint8_t * data = src->base.pack->data;
    return (intBitsToFloat(get_bytes(data,  src->BYTE, 4)));
}
INLINER void p100_flow_rate_x_SET(float  src, Bounds_Inside * dst)//Flow rate in radians/second about X axi
{
    if(dst->base.field_bit != 208)insert_field(dst, 208, 0);
    uint8_t * data = dst->base.pack->data;
    set_bytes(floatToIntBits(src), 4, data,  dst->BYTE);
}
INLINER float  p100_flow_rate_y_TRY(Bounds_Inside * src)
{
    if(src->base.field_bit !=  209 && !try_visit_field(src, 209)) return 0;
    uint8_t * data = src->base.pack->data;
    return (intBitsToFloat(get_bytes(data,  src->BYTE, 4)));
}
INLINER void p100_flow_rate_y_SET(float  src, Bounds_Inside * dst)//Flow rate in radians/second about Y axi
{
    if(dst->base.field_bit != 209)insert_field(dst, 209, 0);
    uint8_t * data = dst->base.pack->data;
    set_bytes(floatToIntBits(src), 4, data,  dst->BYTE);
}
INLINER uint64_t p101_usec_GET(Pack * src)//Timestamp (microseconds, synced to UNIX time or since system boot
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 8)));
}
INLINER void p101_usec_SET(uint64_t  src, Pack * dst)//Timestamp (microseconds, synced to UNIX time or since system boot
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  0);
}
INLINER float p101_x_GET(Pack * src)//Global X positio
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  8, 4)));
}
INLINER void p101_x_SET(float  src, Pack * dst)//Global X positio
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  8);
}
INLINER float p101_y_GET(Pack * src)//Global Y positio
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  12, 4)));
}
INLINER void p101_y_SET(float  src, Pack * dst)//Global Y positio
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  12);
}
INLINER float p101_z_GET(Pack * src)//Global Z positio
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  16, 4)));
}
INLINER void p101_z_SET(float  src, Pack * dst)//Global Z positio
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  16);
}
INLINER float p101_roll_GET(Pack * src)//Roll angle in ra
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  20, 4)));
}
INLINER void p101_roll_SET(float  src, Pack * dst)//Roll angle in ra
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  20);
}
INLINER float p101_pitch_GET(Pack * src)//Pitch angle in ra
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  24, 4)));
}
INLINER void p101_pitch_SET(float  src, Pack * dst)//Pitch angle in ra
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  24);
}
INLINER float p101_yaw_GET(Pack * src)//Yaw angle in ra
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  28, 4)));
}
INLINER void p101_yaw_SET(float  src, Pack * dst)//Yaw angle in ra
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  28);
}
INLINER uint64_t p102_usec_GET(Pack * src)//Timestamp (microseconds, synced to UNIX time or since system boot
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 8)));
}
INLINER void p102_usec_SET(uint64_t  src, Pack * dst)//Timestamp (microseconds, synced to UNIX time or since system boot
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  0);
}
INLINER float p102_x_GET(Pack * src)//Global X positio
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  8, 4)));
}
INLINER void p102_x_SET(float  src, Pack * dst)//Global X positio
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  8);
}
INLINER float p102_y_GET(Pack * src)//Global Y positio
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  12, 4)));
}
INLINER void p102_y_SET(float  src, Pack * dst)//Global Y positio
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  12);
}
INLINER float p102_z_GET(Pack * src)//Global Z positio
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  16, 4)));
}
INLINER void p102_z_SET(float  src, Pack * dst)//Global Z positio
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  16);
}
INLINER float p102_roll_GET(Pack * src)//Roll angle in ra
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  20, 4)));
}
INLINER void p102_roll_SET(float  src, Pack * dst)//Roll angle in ra
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  20);
}
INLINER float p102_pitch_GET(Pack * src)//Pitch angle in ra
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  24, 4)));
}
INLINER void p102_pitch_SET(float  src, Pack * dst)//Pitch angle in ra
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  24);
}
INLINER float p102_yaw_GET(Pack * src)//Yaw angle in ra
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  28, 4)));
}
INLINER void p102_yaw_SET(float  src, Pack * dst)//Yaw angle in ra
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  28);
}
INLINER uint64_t p103_usec_GET(Pack * src)//Timestamp (microseconds, synced to UNIX time or since system boot
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 8)));
}
INLINER void p103_usec_SET(uint64_t  src, Pack * dst)//Timestamp (microseconds, synced to UNIX time or since system boot
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  0);
}
INLINER float p103_x_GET(Pack * src)//Global X spee
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  8, 4)));
}
INLINER void p103_x_SET(float  src, Pack * dst)//Global X spee
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  8);
}
INLINER float p103_y_GET(Pack * src)//Global Y spee
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  12, 4)));
}
INLINER void p103_y_SET(float  src, Pack * dst)//Global Y spee
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  12);
}
INLINER float p103_z_GET(Pack * src)//Global Z spee
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  16, 4)));
}
INLINER void p103_z_SET(float  src, Pack * dst)//Global Z spee
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  16);
}
INLINER uint64_t p104_usec_GET(Pack * src)//Timestamp (microseconds, synced to UNIX time or since system boot
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 8)));
}
INLINER void p104_usec_SET(uint64_t  src, Pack * dst)//Timestamp (microseconds, synced to UNIX time or since system boot
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  0);
}
INLINER float p104_x_GET(Pack * src)//Global X positio
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  8, 4)));
}
INLINER void p104_x_SET(float  src, Pack * dst)//Global X positio
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  8);
}
INLINER float p104_y_GET(Pack * src)//Global Y positio
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  12, 4)));
}
INLINER void p104_y_SET(float  src, Pack * dst)//Global Y positio
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  12);
}
INLINER float p104_z_GET(Pack * src)//Global Z positio
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  16, 4)));
}
INLINER void p104_z_SET(float  src, Pack * dst)//Global Z positio
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  16);
}
INLINER float p104_roll_GET(Pack * src)//Roll angle in ra
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  20, 4)));
}
INLINER void p104_roll_SET(float  src, Pack * dst)//Roll angle in ra
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  20);
}
INLINER float p104_pitch_GET(Pack * src)//Pitch angle in ra
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  24, 4)));
}
INLINER void p104_pitch_SET(float  src, Pack * dst)//Pitch angle in ra
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  24);
}
INLINER float p104_yaw_GET(Pack * src)//Yaw angle in ra
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  28, 4)));
}
INLINER void p104_yaw_SET(float  src, Pack * dst)//Yaw angle in ra
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  28);
}
INLINER uint16_t p105_fields_updated_GET(Pack * src)//Bitmask for fields that have updated since last message, bit 0 = xacc, bit 12: temperatur
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER void p105_fields_updated_SET(uint16_t  src, Pack * dst)//Bitmask for fields that have updated since last message, bit 0 = xacc, bit 12: temperatur
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER uint64_t p105_time_usec_GET(Pack * src)//Timestamp (microseconds, synced to UNIX time or since system boot
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 8)));
}
INLINER void p105_time_usec_SET(uint64_t  src, Pack * dst)//Timestamp (microseconds, synced to UNIX time or since system boot
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  2);
}
INLINER float p105_xacc_GET(Pack * src)//X acceleration (m/s^2
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  10, 4)));
}
INLINER void p105_xacc_SET(float  src, Pack * dst)//X acceleration (m/s^2
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  10);
}
INLINER float p105_yacc_GET(Pack * src)//Y acceleration (m/s^2
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  14, 4)));
}
INLINER void p105_yacc_SET(float  src, Pack * dst)//Y acceleration (m/s^2
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  14);
}
INLINER float p105_zacc_GET(Pack * src)//Z acceleration (m/s^2
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  18, 4)));
}
INLINER void p105_zacc_SET(float  src, Pack * dst)//Z acceleration (m/s^2
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  18);
}
INLINER float p105_xgyro_GET(Pack * src)//Angular speed around X axis (rad / sec
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  22, 4)));
}
INLINER void p105_xgyro_SET(float  src, Pack * dst)//Angular speed around X axis (rad / sec
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  22);
}
INLINER float p105_ygyro_GET(Pack * src)//Angular speed around Y axis (rad / sec
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  26, 4)));
}
INLINER void p105_ygyro_SET(float  src, Pack * dst)//Angular speed around Y axis (rad / sec
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  26);
}
INLINER float p105_zgyro_GET(Pack * src)//Angular speed around Z axis (rad / sec
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  30, 4)));
}
INLINER void p105_zgyro_SET(float  src, Pack * dst)//Angular speed around Z axis (rad / sec
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  30);
}
INLINER float p105_xmag_GET(Pack * src)//X Magnetic field (Gauss
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  34, 4)));
}
INLINER void p105_xmag_SET(float  src, Pack * dst)//X Magnetic field (Gauss
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  34);
}
INLINER float p105_ymag_GET(Pack * src)//Y Magnetic field (Gauss
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  38, 4)));
}
INLINER void p105_ymag_SET(float  src, Pack * dst)//Y Magnetic field (Gauss
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  38);
}
INLINER float p105_zmag_GET(Pack * src)//Z Magnetic field (Gauss
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  42, 4)));
}
INLINER void p105_zmag_SET(float  src, Pack * dst)//Z Magnetic field (Gauss
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  42);
}
INLINER float p105_abs_pressure_GET(Pack * src)//Absolute pressure in milliba
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  46, 4)));
}
INLINER void p105_abs_pressure_SET(float  src, Pack * dst)//Absolute pressure in milliba
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  46);
}
INLINER float p105_diff_pressure_GET(Pack * src)//Differential pressure in milliba
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  50, 4)));
}
INLINER void p105_diff_pressure_SET(float  src, Pack * dst)//Differential pressure in milliba
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  50);
}
INLINER float p105_pressure_alt_GET(Pack * src)//Altitude calculated from pressur
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  54, 4)));
}
INLINER void p105_pressure_alt_SET(float  src, Pack * dst)//Altitude calculated from pressur
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  54);
}
INLINER float p105_temperature_GET(Pack * src)//Temperature in degrees celsiu
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  58, 4)));
}
INLINER void p105_temperature_SET(float  src, Pack * dst)//Temperature in degrees celsiu
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  58);
}
INLINER uint32_t p106_integration_time_us_GET(Pack * src)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 4)));
}
INLINER void p106_integration_time_us_SET(uint32_t  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  0);
}
INLINER uint32_t p106_time_delta_distance_us_GET(Pack * src)//Time in microseconds since the distance was sampled
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  4, 4)));
}
INLINER void p106_time_delta_distance_us_SET(uint32_t  src, Pack * dst)//Time in microseconds since the distance was sampled
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  4);
}
INLINER uint64_t p106_time_usec_GET(Pack * src)//Timestamp (microseconds, synced to UNIX time or since system boot
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  8, 8)));
}
INLINER void p106_time_usec_SET(uint64_t  src, Pack * dst)//Timestamp (microseconds, synced to UNIX time or since system boot
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  8);
}
INLINER uint8_t p106_sensor_id_GET(Pack * src)//Sensor I
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  16, 1)));
}
INLINER void p106_sensor_id_SET(uint8_t  src, Pack * dst)//Sensor I
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  16);
}
INLINER float p106_integrated_x_GET(Pack * src)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  17, 4)));
}
INLINER void p106_integrated_x_SET(float  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  17);
}
INLINER float p106_integrated_y_GET(Pack * src)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  21, 4)));
}
INLINER void p106_integrated_y_SET(float  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  21);
}
INLINER float p106_integrated_xgyro_GET(Pack * src)//RH rotation around X axis (rad
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  25, 4)));
}
INLINER void p106_integrated_xgyro_SET(float  src, Pack * dst)//RH rotation around X axis (rad
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  25);
}
INLINER float p106_integrated_ygyro_GET(Pack * src)//RH rotation around Y axis (rad
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  29, 4)));
}
INLINER void p106_integrated_ygyro_SET(float  src, Pack * dst)//RH rotation around Y axis (rad
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  29);
}
INLINER float p106_integrated_zgyro_GET(Pack * src)//RH rotation around Z axis (rad
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  33, 4)));
}
INLINER void p106_integrated_zgyro_SET(float  src, Pack * dst)//RH rotation around Z axis (rad
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  33);
}
INLINER int16_t p106_temperature_GET(Pack * src)//Temperature * 100 in centi-degrees Celsiu
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  37, 2)));
}
INLINER void p106_temperature_SET(int16_t  src, Pack * dst)//Temperature * 100 in centi-degrees Celsiu
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  37);
}
INLINER uint8_t p106_quality_GET(Pack * src)//Optical flow quality / confidence. 0: no valid flow, 255: maximum qualit
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  39, 1)));
}
INLINER void p106_quality_SET(uint8_t  src, Pack * dst)//Optical flow quality / confidence. 0: no valid flow, 255: maximum qualit
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  39);
}
INLINER float p106_distance_GET(Pack * src)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  40, 4)));
}
INLINER void p106_distance_SET(float  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  40);
}
INLINER uint32_t p107_fields_updated_GET(Pack * src)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 4)));
}
INLINER void p107_fields_updated_SET(uint32_t  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  0);
}
INLINER uint64_t p107_time_usec_GET(Pack * src)//Timestamp (microseconds, synced to UNIX time or since system boot
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  4, 8)));
}
INLINER void p107_time_usec_SET(uint64_t  src, Pack * dst)//Timestamp (microseconds, synced to UNIX time or since system boot
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  4);
}
INLINER float p107_xacc_GET(Pack * src)//X acceleration (m/s^2
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  12, 4)));
}
INLINER void p107_xacc_SET(float  src, Pack * dst)//X acceleration (m/s^2
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  12);
}
INLINER float p107_yacc_GET(Pack * src)//Y acceleration (m/s^2
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  16, 4)));
}
INLINER void p107_yacc_SET(float  src, Pack * dst)//Y acceleration (m/s^2
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  16);
}
INLINER float p107_zacc_GET(Pack * src)//Z acceleration (m/s^2
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  20, 4)));
}
INLINER void p107_zacc_SET(float  src, Pack * dst)//Z acceleration (m/s^2
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  20);
}
INLINER float p107_xgyro_GET(Pack * src)//Angular speed around X axis in body frame (rad / sec
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  24, 4)));
}
INLINER void p107_xgyro_SET(float  src, Pack * dst)//Angular speed around X axis in body frame (rad / sec
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  24);
}
INLINER float p107_ygyro_GET(Pack * src)//Angular speed around Y axis in body frame (rad / sec
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  28, 4)));
}
INLINER void p107_ygyro_SET(float  src, Pack * dst)//Angular speed around Y axis in body frame (rad / sec
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  28);
}
INLINER float p107_zgyro_GET(Pack * src)//Angular speed around Z axis in body frame (rad / sec
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  32, 4)));
}
INLINER void p107_zgyro_SET(float  src, Pack * dst)//Angular speed around Z axis in body frame (rad / sec
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  32);
}
INLINER float p107_xmag_GET(Pack * src)//X Magnetic field (Gauss
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  36, 4)));
}
INLINER void p107_xmag_SET(float  src, Pack * dst)//X Magnetic field (Gauss
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  36);
}
INLINER float p107_ymag_GET(Pack * src)//Y Magnetic field (Gauss
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  40, 4)));
}
INLINER void p107_ymag_SET(float  src, Pack * dst)//Y Magnetic field (Gauss
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  40);
}
INLINER float p107_zmag_GET(Pack * src)//Z Magnetic field (Gauss
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  44, 4)));
}
INLINER void p107_zmag_SET(float  src, Pack * dst)//Z Magnetic field (Gauss
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  44);
}
INLINER float p107_abs_pressure_GET(Pack * src)//Absolute pressure in milliba
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  48, 4)));
}
INLINER void p107_abs_pressure_SET(float  src, Pack * dst)//Absolute pressure in milliba
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  48);
}
INLINER float p107_diff_pressure_GET(Pack * src)//Differential pressure (airspeed) in milliba
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  52, 4)));
}
INLINER void p107_diff_pressure_SET(float  src, Pack * dst)//Differential pressure (airspeed) in milliba
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  52);
}
INLINER float p107_pressure_alt_GET(Pack * src)//Altitude calculated from pressur
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  56, 4)));
}
INLINER void p107_pressure_alt_SET(float  src, Pack * dst)//Altitude calculated from pressur
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  56);
}
INLINER float p107_temperature_GET(Pack * src)//Temperature in degrees celsiu
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  60, 4)));
}
INLINER void p107_temperature_SET(float  src, Pack * dst)//Temperature in degrees celsiu
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  60);
}
INLINER float p108_q1_GET(Pack * src)//True attitude quaternion component 1, w (1 in null-rotation
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  0, 4)));
}
INLINER void p108_q1_SET(float  src, Pack * dst)//True attitude quaternion component 1, w (1 in null-rotation
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  0);
}
INLINER float p108_q2_GET(Pack * src)//True attitude quaternion component 2, x (0 in null-rotation
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  4, 4)));
}
INLINER void p108_q2_SET(float  src, Pack * dst)//True attitude quaternion component 2, x (0 in null-rotation
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  4);
}
INLINER float p108_q3_GET(Pack * src)//True attitude quaternion component 3, y (0 in null-rotation
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  8, 4)));
}
INLINER void p108_q3_SET(float  src, Pack * dst)//True attitude quaternion component 3, y (0 in null-rotation
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  8);
}
INLINER float p108_q4_GET(Pack * src)//True attitude quaternion component 4, z (0 in null-rotation
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  12, 4)));
}
INLINER void p108_q4_SET(float  src, Pack * dst)//True attitude quaternion component 4, z (0 in null-rotation
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  12);
}
INLINER float p108_roll_GET(Pack * src)//Attitude roll expressed as Euler angles, not recommended except for human-readable output
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  16, 4)));
}
INLINER void p108_roll_SET(float  src, Pack * dst)//Attitude roll expressed as Euler angles, not recommended except for human-readable output
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  16);
}
INLINER float p108_pitch_GET(Pack * src)//Attitude pitch expressed as Euler angles, not recommended except for human-readable output
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  20, 4)));
}
INLINER void p108_pitch_SET(float  src, Pack * dst)//Attitude pitch expressed as Euler angles, not recommended except for human-readable output
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  20);
}
INLINER float p108_yaw_GET(Pack * src)//Attitude yaw expressed as Euler angles, not recommended except for human-readable output
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  24, 4)));
}
INLINER void p108_yaw_SET(float  src, Pack * dst)//Attitude yaw expressed as Euler angles, not recommended except for human-readable output
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  24);
}
INLINER float p108_xacc_GET(Pack * src)//X acceleration m/s/
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  28, 4)));
}
INLINER void p108_xacc_SET(float  src, Pack * dst)//X acceleration m/s/
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  28);
}
INLINER float p108_yacc_GET(Pack * src)//Y acceleration m/s/
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  32, 4)));
}
INLINER void p108_yacc_SET(float  src, Pack * dst)//Y acceleration m/s/
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  32);
}
INLINER float p108_zacc_GET(Pack * src)//Z acceleration m/s/
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  36, 4)));
}
INLINER void p108_zacc_SET(float  src, Pack * dst)//Z acceleration m/s/
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  36);
}
INLINER float p108_xgyro_GET(Pack * src)//Angular speed around X axis rad/
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  40, 4)));
}
INLINER void p108_xgyro_SET(float  src, Pack * dst)//Angular speed around X axis rad/
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  40);
}
INLINER float p108_ygyro_GET(Pack * src)//Angular speed around Y axis rad/
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  44, 4)));
}
INLINER void p108_ygyro_SET(float  src, Pack * dst)//Angular speed around Y axis rad/
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  44);
}
INLINER float p108_zgyro_GET(Pack * src)//Angular speed around Z axis rad/
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  48, 4)));
}
INLINER void p108_zgyro_SET(float  src, Pack * dst)//Angular speed around Z axis rad/
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  48);
}
INLINER float p108_lat_GET(Pack * src)//Latitude in degree
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  52, 4)));
}
INLINER void p108_lat_SET(float  src, Pack * dst)//Latitude in degree
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  52);
}
INLINER float p108_lon_GET(Pack * src)//Longitude in degree
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  56, 4)));
}
INLINER void p108_lon_SET(float  src, Pack * dst)//Longitude in degree
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  56);
}
INLINER float p108_alt_GET(Pack * src)//Altitude in meter
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  60, 4)));
}
INLINER void p108_alt_SET(float  src, Pack * dst)//Altitude in meter
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  60);
}
INLINER float p108_std_dev_horz_GET(Pack * src)//Horizontal position standard deviatio
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  64, 4)));
}
INLINER void p108_std_dev_horz_SET(float  src, Pack * dst)//Horizontal position standard deviatio
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  64);
}
INLINER float p108_std_dev_vert_GET(Pack * src)//Vertical position standard deviatio
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  68, 4)));
}
INLINER void p108_std_dev_vert_SET(float  src, Pack * dst)//Vertical position standard deviatio
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  68);
}
INLINER float p108_vn_GET(Pack * src)//True velocity in m/s in NORTH direction in earth-fixed NED fram
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  72, 4)));
}
INLINER void p108_vn_SET(float  src, Pack * dst)//True velocity in m/s in NORTH direction in earth-fixed NED fram
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  72);
}
INLINER float p108_ve_GET(Pack * src)//True velocity in m/s in EAST direction in earth-fixed NED fram
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  76, 4)));
}
INLINER void p108_ve_SET(float  src, Pack * dst)//True velocity in m/s in EAST direction in earth-fixed NED fram
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  76);
}
INLINER float p108_vd_GET(Pack * src)//True velocity in m/s in DOWN direction in earth-fixed NED fram
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  80, 4)));
}
INLINER void p108_vd_SET(float  src, Pack * dst)//True velocity in m/s in DOWN direction in earth-fixed NED fram
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  80);
}
INLINER uint16_t p109_rxerrors_GET(Pack * src)//Receive error
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER void p109_rxerrors_SET(uint16_t  src, Pack * dst)//Receive error
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER uint16_t p109_fixed__GET(Pack * src)//Count of error corrected packet
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 2)));
}
INLINER void p109_fixed__SET(uint16_t  src, Pack * dst)//Count of error corrected packet
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  2);
}
INLINER uint8_t p109_rssi_GET(Pack * src)//Local signal strengt
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  4, 1)));
}
INLINER void p109_rssi_SET(uint8_t  src, Pack * dst)//Local signal strengt
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  4);
}
INLINER uint8_t p109_remrssi_GET(Pack * src)//Remote signal strengt
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  5, 1)));
}
INLINER void p109_remrssi_SET(uint8_t  src, Pack * dst)//Remote signal strengt
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  5);
}
INLINER uint8_t p109_txbuf_GET(Pack * src)//Remaining free buffer space in percent
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  6, 1)));
}
INLINER void p109_txbuf_SET(uint8_t  src, Pack * dst)//Remaining free buffer space in percent
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  6);
}
INLINER uint8_t p109_noise_GET(Pack * src)//Background noise leve
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  7, 1)));
}
INLINER void p109_noise_SET(uint8_t  src, Pack * dst)//Background noise leve
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  7);
}
INLINER uint8_t p109_remnoise_GET(Pack * src)//Remote background noise leve
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  8, 1)));
}
INLINER void p109_remnoise_SET(uint8_t  src, Pack * dst)//Remote background noise leve
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  8);
}
INLINER uint8_t p110_target_network_GET(Pack * src)//Network ID (0 for broadcast
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 1)));
}
INLINER void p110_target_network_SET(uint8_t  src, Pack * dst)//Network ID (0 for broadcast
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  0);
}
INLINER uint8_t p110_target_system_GET(Pack * src)//System ID (0 for broadcast
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  1, 1)));
}
INLINER void p110_target_system_SET(uint8_t  src, Pack * dst)//System ID (0 for broadcast
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  1);
}
INLINER uint8_t p110_target_component_GET(Pack * src)//Component ID (0 for broadcast
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 1)));
}
INLINER void p110_target_component_SET(uint8_t  src, Pack * dst)//Component ID (0 for broadcast
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  2);
}
INLINER uint8_t* p110_payload_GET(Pack * src, uint8_t*  dst, int32_t pos)
{
    uint8_t * data = src->data;
    for(int32_t BYTE = 3, dst_max = pos + 251; pos < dst_max ; pos++,  BYTE += 1)
        dst[pos] = ((get_bytes(data,  BYTE, 1)));
    return dst;
}

static const  uint32_t p110_payload_LEN = 251; //return array length

INLINER  uint8_t*  p110_payload_GET_(Pack * src) {return p110_payload_GET(src, malloc(251 * sizeof(uint8_t)), 0);}
INLINER void p110_payload_SET(uint8_t*  src, int32_t pos, Pack * dst)
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  3, src_max = pos + 251; pos < src_max; pos++, BYTE += 1)
        set_bytes((src[pos]), 1, data,  BYTE);
}
INLINER int64_t p111_tc1_GET(Pack * src)//Time sync timestamp
{
    uint8_t * data = src->data;
    return ((int64_t)(get_bytes(data,  0, 8)));
}
INLINER void p111_tc1_SET(int64_t  src, Pack * dst)//Time sync timestamp
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  0);
}
INLINER int64_t p111_ts1_GET(Pack * src)//Time sync timestamp
{
    uint8_t * data = src->data;
    return ((int64_t)(get_bytes(data,  8, 8)));
}
INLINER void p111_ts1_SET(int64_t  src, Pack * dst)//Time sync timestamp
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  8);
}
INLINER uint32_t p112_seq_GET(Pack * src)//Image frame sequenc
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 4)));
}
INLINER void p112_seq_SET(uint32_t  src, Pack * dst)//Image frame sequenc
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  0);
}
INLINER uint64_t p112_time_usec_GET(Pack * src)//Timestamp for the image frame in microsecond
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  4, 8)));
}
INLINER void p112_time_usec_SET(uint64_t  src, Pack * dst)//Timestamp for the image frame in microsecond
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  4);
}
INLINER uint16_t p113_eph_GET(Pack * src)//GPS HDOP horizontal dilution of position in cm (m*100). If unknown, set to: 6553
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER void p113_eph_SET(uint16_t  src, Pack * dst)//GPS HDOP horizontal dilution of position in cm (m*100). If unknown, set to: 6553
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER uint16_t p113_epv_GET(Pack * src)//GPS VDOP vertical dilution of position in cm (m*100). If unknown, set to: 6553
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 2)));
}
INLINER void p113_epv_SET(uint16_t  src, Pack * dst)//GPS VDOP vertical dilution of position in cm (m*100). If unknown, set to: 6553
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  2);
}
INLINER uint16_t p113_vel_GET(Pack * src)//GPS ground speed in cm/s. If unknown, set to: 6553
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  4, 2)));
}
INLINER void p113_vel_SET(uint16_t  src, Pack * dst)//GPS ground speed in cm/s. If unknown, set to: 6553
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  4);
}
INLINER uint16_t p113_cog_GET(Pack * src)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  6, 2)));
}
INLINER void p113_cog_SET(uint16_t  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  6);
}
INLINER uint64_t p113_time_usec_GET(Pack * src)//Timestamp (microseconds since UNIX epoch or microseconds since system boot
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  8, 8)));
}
INLINER void p113_time_usec_SET(uint64_t  src, Pack * dst)//Timestamp (microseconds since UNIX epoch or microseconds since system boot
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  8);
}
INLINER uint8_t p113_fix_type_GET(Pack * src)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  16, 1)));
}
INLINER void p113_fix_type_SET(uint8_t  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  16);
}
INLINER int32_t p113_lat_GET(Pack * src)//Latitude (WGS84), in degrees * 1E
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  17, 4)));
}
INLINER void p113_lat_SET(int32_t  src, Pack * dst)//Latitude (WGS84), in degrees * 1E
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  17);
}
INLINER int32_t p113_lon_GET(Pack * src)//Longitude (WGS84), in degrees * 1E
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  21, 4)));
}
INLINER void p113_lon_SET(int32_t  src, Pack * dst)//Longitude (WGS84), in degrees * 1E
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  21);
}
INLINER int32_t p113_alt_GET(Pack * src)//Altitude (AMSL, not WGS84), in meters * 1000 (positive for up
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  25, 4)));
}
INLINER void p113_alt_SET(int32_t  src, Pack * dst)//Altitude (AMSL, not WGS84), in meters * 1000 (positive for up
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  25);
}
INLINER int16_t p113_vn_GET(Pack * src)//GPS velocity in cm/s in NORTH direction in earth-fixed NED fram
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  29, 2)));
}
INLINER void p113_vn_SET(int16_t  src, Pack * dst)//GPS velocity in cm/s in NORTH direction in earth-fixed NED fram
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  29);
}
INLINER int16_t p113_ve_GET(Pack * src)//GPS velocity in cm/s in EAST direction in earth-fixed NED fram
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  31, 2)));
}
INLINER void p113_ve_SET(int16_t  src, Pack * dst)//GPS velocity in cm/s in EAST direction in earth-fixed NED fram
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  31);
}
INLINER int16_t p113_vd_GET(Pack * src)//GPS velocity in cm/s in DOWN direction in earth-fixed NED fram
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  33, 2)));
}
INLINER void p113_vd_SET(int16_t  src, Pack * dst)//GPS velocity in cm/s in DOWN direction in earth-fixed NED fram
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  33);
}
INLINER uint8_t p113_satellites_visible_GET(Pack * src)//Number of satellites visible. If unknown, set to 25
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  35, 1)));
}
INLINER void p113_satellites_visible_SET(uint8_t  src, Pack * dst)//Number of satellites visible. If unknown, set to 25
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  35);
}
INLINER uint32_t p114_integration_time_us_GET(Pack * src)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 4)));
}
INLINER void p114_integration_time_us_SET(uint32_t  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  0);
}
INLINER uint32_t p114_time_delta_distance_us_GET(Pack * src)//Time in microseconds since the distance was sampled
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  4, 4)));
}
INLINER void p114_time_delta_distance_us_SET(uint32_t  src, Pack * dst)//Time in microseconds since the distance was sampled
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  4);
}
INLINER uint64_t p114_time_usec_GET(Pack * src)//Timestamp (microseconds, synced to UNIX time or since system boot
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  8, 8)));
}
INLINER void p114_time_usec_SET(uint64_t  src, Pack * dst)//Timestamp (microseconds, synced to UNIX time or since system boot
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  8);
}
INLINER uint8_t p114_sensor_id_GET(Pack * src)//Sensor I
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  16, 1)));
}
INLINER void p114_sensor_id_SET(uint8_t  src, Pack * dst)//Sensor I
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  16);
}
INLINER float p114_integrated_x_GET(Pack * src)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  17, 4)));
}
INLINER void p114_integrated_x_SET(float  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  17);
}
INLINER float p114_integrated_y_GET(Pack * src)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  21, 4)));
}
INLINER void p114_integrated_y_SET(float  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  21);
}
INLINER float p114_integrated_xgyro_GET(Pack * src)//RH rotation around X axis (rad
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  25, 4)));
}
INLINER void p114_integrated_xgyro_SET(float  src, Pack * dst)//RH rotation around X axis (rad
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  25);
}
INLINER float p114_integrated_ygyro_GET(Pack * src)//RH rotation around Y axis (rad
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  29, 4)));
}
INLINER void p114_integrated_ygyro_SET(float  src, Pack * dst)//RH rotation around Y axis (rad
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  29);
}
INLINER float p114_integrated_zgyro_GET(Pack * src)//RH rotation around Z axis (rad
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  33, 4)));
}
INLINER void p114_integrated_zgyro_SET(float  src, Pack * dst)//RH rotation around Z axis (rad
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  33);
}
INLINER int16_t p114_temperature_GET(Pack * src)//Temperature * 100 in centi-degrees Celsiu
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  37, 2)));
}
INLINER void p114_temperature_SET(int16_t  src, Pack * dst)//Temperature * 100 in centi-degrees Celsiu
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  37);
}
INLINER uint8_t p114_quality_GET(Pack * src)//Optical flow quality / confidence. 0: no valid flow, 255: maximum qualit
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  39, 1)));
}
INLINER void p114_quality_SET(uint8_t  src, Pack * dst)//Optical flow quality / confidence. 0: no valid flow, 255: maximum qualit
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  39);
}
INLINER float p114_distance_GET(Pack * src)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  40, 4)));
}
INLINER void p114_distance_SET(float  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  40);
}
INLINER uint16_t p115_ind_airspeed_GET(Pack * src)//Indicated airspeed, expressed as cm/
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER void p115_ind_airspeed_SET(uint16_t  src, Pack * dst)//Indicated airspeed, expressed as cm/
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER uint16_t p115_true_airspeed_GET(Pack * src)//True airspeed, expressed as cm/
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 2)));
}
INLINER void p115_true_airspeed_SET(uint16_t  src, Pack * dst)//True airspeed, expressed as cm/
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  2);
}
INLINER uint64_t p115_time_usec_GET(Pack * src)//Timestamp (microseconds since UNIX epoch or microseconds since system boot
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  4, 8)));
}
INLINER void p115_time_usec_SET(uint64_t  src, Pack * dst)//Timestamp (microseconds since UNIX epoch or microseconds since system boot
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  4);
}
INLINER float* p115_attitude_quaternion_GET(Pack * src, float*  dst, int32_t pos) //Vehicle attitude expressed as normalized quaternion in w, x, y, z order (with 1 0 0 0 being the null-rotatio
{
    uint8_t * data = src->data;
    for(int32_t BYTE = 12, dst_max = pos + 4; pos < dst_max ; pos++,  BYTE += 4)
        dst[pos] = (intBitsToFloat(get_bytes(data,  BYTE, 4)));
    return dst;
}

static const  uint32_t p115_attitude_quaternion_LEN = 4; //return array length

INLINER  float*  p115_attitude_quaternion_GET_(Pack * src) {return p115_attitude_quaternion_GET(src, malloc(4 * sizeof(float)), 0);}
INLINER void p115_attitude_quaternion_SET(float*  src, int32_t pos, Pack * dst) //Vehicle attitude expressed as normalized quaternion in w, x, y, z order (with 1 0 0 0 being the null-rotatio
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  12, src_max = pos + 4; pos < src_max; pos++, BYTE += 4)
        set_bytes(floatToIntBits(src[pos]), 4, data,  BYTE);
}
INLINER float p115_rollspeed_GET(Pack * src)//Body frame roll / phi angular speed (rad/s
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  28, 4)));
}
INLINER void p115_rollspeed_SET(float  src, Pack * dst)//Body frame roll / phi angular speed (rad/s
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  28);
}
INLINER float p115_pitchspeed_GET(Pack * src)//Body frame pitch / theta angular speed (rad/s
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  32, 4)));
}
INLINER void p115_pitchspeed_SET(float  src, Pack * dst)//Body frame pitch / theta angular speed (rad/s
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  32);
}
INLINER float p115_yawspeed_GET(Pack * src)//Body frame yaw / psi angular speed (rad/s
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  36, 4)));
}
INLINER void p115_yawspeed_SET(float  src, Pack * dst)//Body frame yaw / psi angular speed (rad/s
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  36);
}
INLINER int32_t p115_lat_GET(Pack * src)//Latitude, expressed as * 1E
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  40, 4)));
}
INLINER void p115_lat_SET(int32_t  src, Pack * dst)//Latitude, expressed as * 1E
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  40);
}
INLINER int32_t p115_lon_GET(Pack * src)//Longitude, expressed as * 1E
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  44, 4)));
}
INLINER void p115_lon_SET(int32_t  src, Pack * dst)//Longitude, expressed as * 1E
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  44);
}
INLINER int32_t p115_alt_GET(Pack * src)//Altitude in meters, expressed as * 1000 (millimeters
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  48, 4)));
}
INLINER void p115_alt_SET(int32_t  src, Pack * dst)//Altitude in meters, expressed as * 1000 (millimeters
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  48);
}
INLINER int16_t p115_vx_GET(Pack * src)//Ground X Speed (Latitude), expressed as cm/
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  52, 2)));
}
INLINER void p115_vx_SET(int16_t  src, Pack * dst)//Ground X Speed (Latitude), expressed as cm/
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  52);
}
INLINER int16_t p115_vy_GET(Pack * src)//Ground Y Speed (Longitude), expressed as cm/
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  54, 2)));
}
INLINER void p115_vy_SET(int16_t  src, Pack * dst)//Ground Y Speed (Longitude), expressed as cm/
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  54);
}
INLINER int16_t p115_vz_GET(Pack * src)//Ground Z Speed (Altitude), expressed as cm/
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  56, 2)));
}
INLINER void p115_vz_SET(int16_t  src, Pack * dst)//Ground Z Speed (Altitude), expressed as cm/
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  56);
}
INLINER int16_t p115_xacc_GET(Pack * src)//X acceleration (mg
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  58, 2)));
}
INLINER void p115_xacc_SET(int16_t  src, Pack * dst)//X acceleration (mg
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  58);
}
INLINER int16_t p115_yacc_GET(Pack * src)//Y acceleration (mg
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  60, 2)));
}
INLINER void p115_yacc_SET(int16_t  src, Pack * dst)//Y acceleration (mg
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  60);
}
INLINER int16_t p115_zacc_GET(Pack * src)//Z acceleration (mg
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  62, 2)));
}
INLINER void p115_zacc_SET(int16_t  src, Pack * dst)//Z acceleration (mg
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  62);
}
INLINER uint32_t p116_time_boot_ms_GET(Pack * src)//Timestamp (milliseconds since system boot
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 4)));
}
INLINER void p116_time_boot_ms_SET(uint32_t  src, Pack * dst)//Timestamp (milliseconds since system boot
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  0);
}
INLINER int16_t p116_xacc_GET(Pack * src)//X acceleration (mg
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  4, 2)));
}
INLINER void p116_xacc_SET(int16_t  src, Pack * dst)//X acceleration (mg
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  4);
}
INLINER int16_t p116_yacc_GET(Pack * src)//Y acceleration (mg
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  6, 2)));
}
INLINER void p116_yacc_SET(int16_t  src, Pack * dst)//Y acceleration (mg
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  6);
}
INLINER int16_t p116_zacc_GET(Pack * src)//Z acceleration (mg
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  8, 2)));
}
INLINER void p116_zacc_SET(int16_t  src, Pack * dst)//Z acceleration (mg
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  8);
}
INLINER int16_t p116_xgyro_GET(Pack * src)//Angular speed around X axis (millirad /sec
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  10, 2)));
}
INLINER void p116_xgyro_SET(int16_t  src, Pack * dst)//Angular speed around X axis (millirad /sec
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  10);
}
INLINER int16_t p116_ygyro_GET(Pack * src)//Angular speed around Y axis (millirad /sec
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  12, 2)));
}
INLINER void p116_ygyro_SET(int16_t  src, Pack * dst)//Angular speed around Y axis (millirad /sec
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  12);
}
INLINER int16_t p116_zgyro_GET(Pack * src)//Angular speed around Z axis (millirad /sec
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  14, 2)));
}
INLINER void p116_zgyro_SET(int16_t  src, Pack * dst)//Angular speed around Z axis (millirad /sec
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  14);
}
INLINER int16_t p116_xmag_GET(Pack * src)//X Magnetic field (milli tesla
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  16, 2)));
}
INLINER void p116_xmag_SET(int16_t  src, Pack * dst)//X Magnetic field (milli tesla
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  16);
}
INLINER int16_t p116_ymag_GET(Pack * src)//Y Magnetic field (milli tesla
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  18, 2)));
}
INLINER void p116_ymag_SET(int16_t  src, Pack * dst)//Y Magnetic field (milli tesla
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  18);
}
INLINER int16_t p116_zmag_GET(Pack * src)//Z Magnetic field (milli tesla
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  20, 2)));
}
INLINER void p116_zmag_SET(int16_t  src, Pack * dst)//Z Magnetic field (milli tesla
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  20);
}
INLINER uint16_t p117_start_GET(Pack * src)//First log id (0 for first available
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER void p117_start_SET(uint16_t  src, Pack * dst)//First log id (0 for first available
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER uint16_t p117_end_GET(Pack * src)//Last log id (0xffff for last available
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 2)));
}
INLINER void p117_end_SET(uint16_t  src, Pack * dst)//Last log id (0xffff for last available
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  2);
}
INLINER uint8_t p117_target_system_GET(Pack * src)//System I
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  4, 1)));
}
INLINER void p117_target_system_SET(uint8_t  src, Pack * dst)//System I
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  4);
}
INLINER uint8_t p117_target_component_GET(Pack * src)//Component I
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  5, 1)));
}
INLINER void p117_target_component_SET(uint8_t  src, Pack * dst)//Component I
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  5);
}
INLINER uint16_t p118_id_GET(Pack * src)//Log i
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER void p118_id_SET(uint16_t  src, Pack * dst)//Log i
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER uint16_t p118_num_logs_GET(Pack * src)//Total number of log
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 2)));
}
INLINER void p118_num_logs_SET(uint16_t  src, Pack * dst)//Total number of log
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  2);
}
INLINER uint16_t p118_last_log_num_GET(Pack * src)//High log numbe
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  4, 2)));
}
INLINER void p118_last_log_num_SET(uint16_t  src, Pack * dst)//High log numbe
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  4);
}
INLINER uint32_t p118_time_utc_GET(Pack * src)//UTC timestamp of log in seconds since 1970, or 0 if not availabl
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  6, 4)));
}
INLINER void p118_time_utc_SET(uint32_t  src, Pack * dst)//UTC timestamp of log in seconds since 1970, or 0 if not availabl
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  6);
}
INLINER uint32_t p118_size_GET(Pack * src)//Size of the log (may be approximate) in byte
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  10, 4)));
}
INLINER void p118_size_SET(uint32_t  src, Pack * dst)//Size of the log (may be approximate) in byte
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  10);
}
INLINER uint16_t p119_id_GET(Pack * src)//Log id (from LOG_ENTRY reply
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER void p119_id_SET(uint16_t  src, Pack * dst)//Log id (from LOG_ENTRY reply
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER uint32_t p119_ofs_GET(Pack * src)//Offset into the lo
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 4)));
}
INLINER void p119_ofs_SET(uint32_t  src, Pack * dst)//Offset into the lo
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  2);
}
INLINER uint32_t p119_count_GET(Pack * src)//Number of byte
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  6, 4)));
}
INLINER void p119_count_SET(uint32_t  src, Pack * dst)//Number of byte
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  6);
}
INLINER uint8_t p119_target_system_GET(Pack * src)//System I
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  10, 1)));
}
INLINER void p119_target_system_SET(uint8_t  src, Pack * dst)//System I
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  10);
}
INLINER uint8_t p119_target_component_GET(Pack * src)//Component I
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  11, 1)));
}
INLINER void p119_target_component_SET(uint8_t  src, Pack * dst)//Component I
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  11);
}
INLINER uint16_t p120_id_GET(Pack * src)//Log id (from LOG_ENTRY reply
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER void p120_id_SET(uint16_t  src, Pack * dst)//Log id (from LOG_ENTRY reply
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER uint32_t p120_ofs_GET(Pack * src)//Offset into the lo
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 4)));
}
INLINER void p120_ofs_SET(uint32_t  src, Pack * dst)//Offset into the lo
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  2);
}
INLINER uint8_t p120_count_GET(Pack * src)//Number of bytes (zero for end of log
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  6, 1)));
}
INLINER void p120_count_SET(uint8_t  src, Pack * dst)//Number of bytes (zero for end of log
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  6);
}
INLINER uint8_t* p120_data__GET(Pack * src, uint8_t*  dst, int32_t pos) //log dat
{
    uint8_t * data = src->data;
    for(int32_t BYTE = 7, dst_max = pos + 90; pos < dst_max ; pos++,  BYTE += 1)
        dst[pos] = ((get_bytes(data,  BYTE, 1)));
    return dst;
}

static const  uint32_t p120_data__LEN = 90; //return array length

INLINER  uint8_t*  p120_data__GET_(Pack * src) {return p120_data__GET(src, malloc(90 * sizeof(uint8_t)), 0);}
INLINER void p120_data__SET(uint8_t*  src, int32_t pos, Pack * dst) //log dat
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  7, src_max = pos + 90; pos < src_max; pos++, BYTE += 1)
        set_bytes((src[pos]), 1, data,  BYTE);
}
INLINER uint8_t p121_target_system_GET(Pack * src)//System I
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 1)));
}
INLINER void p121_target_system_SET(uint8_t  src, Pack * dst)//System I
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  0);
}
INLINER uint8_t p121_target_component_GET(Pack * src)//Component I
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  1, 1)));
}
INLINER void p121_target_component_SET(uint8_t  src, Pack * dst)//Component I
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  1);
}
INLINER uint8_t p122_target_system_GET(Pack * src)//System I
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 1)));
}
INLINER void p122_target_system_SET(uint8_t  src, Pack * dst)//System I
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  0);
}
INLINER uint8_t p122_target_component_GET(Pack * src)//Component I
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  1, 1)));
}
INLINER void p122_target_component_SET(uint8_t  src, Pack * dst)//Component I
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  1);
}
INLINER uint8_t p123_target_system_GET(Pack * src)//System I
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 1)));
}
INLINER void p123_target_system_SET(uint8_t  src, Pack * dst)//System I
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  0);
}
INLINER uint8_t p123_target_component_GET(Pack * src)//Component I
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  1, 1)));
}
INLINER void p123_target_component_SET(uint8_t  src, Pack * dst)//Component I
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  1);
}
INLINER uint8_t p123_len_GET(Pack * src)//data lengt
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 1)));
}
INLINER void p123_len_SET(uint8_t  src, Pack * dst)//data lengt
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  2);
}
INLINER uint8_t* p123_data__GET(Pack * src, uint8_t*  dst, int32_t pos) //raw data (110 is enough for 12 satellites of RTCMv2
{
    uint8_t * data = src->data;
    for(int32_t BYTE = 3, dst_max = pos + 110; pos < dst_max ; pos++,  BYTE += 1)
        dst[pos] = ((get_bytes(data,  BYTE, 1)));
    return dst;
}

static const  uint32_t p123_data__LEN = 110; //return array length

INLINER  uint8_t*  p123_data__GET_(Pack * src) {return p123_data__GET(src, malloc(110 * sizeof(uint8_t)), 0);}
INLINER void p123_data__SET(uint8_t*  src, int32_t pos, Pack * dst) //raw data (110 is enough for 12 satellites of RTCMv2
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  3, src_max = pos + 110; pos < src_max; pos++, BYTE += 1)
        set_bytes((src[pos]), 1, data,  BYTE);
}
INLINER uint16_t p124_eph_GET(Pack * src)//GPS HDOP horizontal dilution of position in cm (m*100). If unknown, set to: UINT16_MA
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER void p124_eph_SET(uint16_t  src, Pack * dst)//GPS HDOP horizontal dilution of position in cm (m*100). If unknown, set to: UINT16_MA
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER uint16_t p124_epv_GET(Pack * src)//GPS VDOP vertical dilution of position in cm (m*100). If unknown, set to: UINT16_MA
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 2)));
}
INLINER void p124_epv_SET(uint16_t  src, Pack * dst)//GPS VDOP vertical dilution of position in cm (m*100). If unknown, set to: UINT16_MA
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  2);
}
INLINER uint16_t p124_vel_GET(Pack * src)//GPS ground speed (m/s * 100). If unknown, set to: UINT16_MA
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  4, 2)));
}
INLINER void p124_vel_SET(uint16_t  src, Pack * dst)//GPS ground speed (m/s * 100). If unknown, set to: UINT16_MA
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  4);
}
INLINER uint16_t p124_cog_GET(Pack * src)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  6, 2)));
}
INLINER void p124_cog_SET(uint16_t  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  6);
}
INLINER uint32_t p124_dgps_age_GET(Pack * src)//Age of DGPS inf
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  8, 4)));
}
INLINER void p124_dgps_age_SET(uint32_t  src, Pack * dst)//Age of DGPS inf
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  8);
}
INLINER uint64_t p124_time_usec_GET(Pack * src)//Timestamp (microseconds since UNIX epoch or microseconds since system boot
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  12, 8)));
}
INLINER void p124_time_usec_SET(uint64_t  src, Pack * dst)//Timestamp (microseconds since UNIX epoch or microseconds since system boot
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  12);
}
INLINER int32_t p124_lat_GET(Pack * src)//Latitude (WGS84), in degrees * 1E
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  20, 4)));
}
INLINER void p124_lat_SET(int32_t  src, Pack * dst)//Latitude (WGS84), in degrees * 1E
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  20);
}
INLINER int32_t p124_lon_GET(Pack * src)//Longitude (WGS84), in degrees * 1E
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  24, 4)));
}
INLINER void p124_lon_SET(int32_t  src, Pack * dst)//Longitude (WGS84), in degrees * 1E
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  24);
}
INLINER int32_t p124_alt_GET(Pack * src)//Altitude (AMSL, not WGS84), in meters * 1000 (positive for up
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  28, 4)));
}
INLINER void p124_alt_SET(int32_t  src, Pack * dst)//Altitude (AMSL, not WGS84), in meters * 1000 (positive for up
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  28);
}
INLINER uint8_t p124_satellites_visible_GET(Pack * src)//Number of satellites visible. If unknown, set to 25
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  32, 1)));
}
INLINER void p124_satellites_visible_SET(uint8_t  src, Pack * dst)//Number of satellites visible. If unknown, set to 25
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  32);
}
INLINER uint8_t p124_dgps_numch_GET(Pack * src)//Number of DGPS satellite
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  33, 1)));
}
INLINER void p124_dgps_numch_SET(uint8_t  src, Pack * dst)//Number of DGPS satellite
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  33);
}
INLINER e_GPS_FIX_TYPE p124_fix_type_GET(Pack * src)//See the GPS_FIX_TYPE enum
{
    uint8_t * data = src->data;
    return  0 + (int)get_bits(data, 272, 4);
}
INLINER void p124_fix_type_SET(e_GPS_FIX_TYPE  src, Pack * dst)//See the GPS_FIX_TYPE enum
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 4, data, 272);
}
INLINER uint16_t p125_Vcc_GET(Pack * src)//5V rail voltage in millivolt
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER void p125_Vcc_SET(uint16_t  src, Pack * dst)//5V rail voltage in millivolt
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER uint16_t p125_Vservo_GET(Pack * src)//servo rail voltage in millivolt
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 2)));
}
INLINER void p125_Vservo_SET(uint16_t  src, Pack * dst)//servo rail voltage in millivolt
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  2);
}
INLINER e_MAV_POWER_STATUS p125_flags_GET(Pack * src)//power supply status flags (see MAV_POWER_STATUS enum
{
    uint8_t * data = src->data;
    switch(get_bits(data, 32, 3))
    {
        case 0:
            return e_MAV_POWER_STATUS_MAV_POWER_STATUS_BRICK_VALID;
        case 1:
            return e_MAV_POWER_STATUS_MAV_POWER_STATUS_SERVO_VALID;
        case 2:
            return e_MAV_POWER_STATUS_MAV_POWER_STATUS_USB_CONNECTED;
        case 3:
            return e_MAV_POWER_STATUS_MAV_POWER_STATUS_PERIPH_OVERCURRENT;
        case 4:
            return e_MAV_POWER_STATUS_MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT;
        case 5:
            return e_MAV_POWER_STATUS_MAV_POWER_STATUS_CHANGED;
        default: ;//assert(false);//("Unknown enum ID " + id);
    }
}
INLINER void p125_flags_SET(e_MAV_POWER_STATUS  src, Pack * dst)//power supply status flags (see MAV_POWER_STATUS enum
{
    uint8_t * data = dst->data;
    int32_t id;
    switch(src)
    {
        case e_MAV_POWER_STATUS_MAV_POWER_STATUS_BRICK_VALID:
            id = 0;
            break;
        case e_MAV_POWER_STATUS_MAV_POWER_STATUS_SERVO_VALID:
            id = 1;
            break;
        case e_MAV_POWER_STATUS_MAV_POWER_STATUS_USB_CONNECTED:
            id = 2;
            break;
        case e_MAV_POWER_STATUS_MAV_POWER_STATUS_PERIPH_OVERCURRENT:
            id = 3;
            break;
        case e_MAV_POWER_STATUS_MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT:
            id = 4;
            break;
        case e_MAV_POWER_STATUS_MAV_POWER_STATUS_CHANGED:
            id = 5;
            break;
        default: ;// assert(false);//("Unknown enum" + id);
    }
    set_bits(id, 3, data, 32);
}
INLINER uint16_t p126_timeout_GET(Pack * src)//Timeout for reply data in millisecond
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER void p126_timeout_SET(uint16_t  src, Pack * dst)//Timeout for reply data in millisecond
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER uint32_t p126_baudrate_GET(Pack * src)//Baudrate of transfer. Zero means no change
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 4)));
}
INLINER void p126_baudrate_SET(uint32_t  src, Pack * dst)//Baudrate of transfer. Zero means no change
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  2);
}
INLINER uint8_t p126_count_GET(Pack * src)//how many bytes in this transfe
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  6, 1)));
}
INLINER void p126_count_SET(uint8_t  src, Pack * dst)//how many bytes in this transfe
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  6);
}
INLINER uint8_t* p126_data__GET(Pack * src, uint8_t*  dst, int32_t pos) //serial dat
{
    uint8_t * data = src->data;
    for(int32_t BYTE = 7, dst_max = pos + 70; pos < dst_max ; pos++,  BYTE += 1)
        dst[pos] = ((get_bytes(data,  BYTE, 1)));
    return dst;
}

static const  uint32_t p126_data__LEN = 70; //return array length

INLINER  uint8_t*  p126_data__GET_(Pack * src) {return p126_data__GET(src, malloc(70 * sizeof(uint8_t)), 0);}
INLINER void p126_data__SET(uint8_t*  src, int32_t pos, Pack * dst) //serial dat
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  7, src_max = pos + 70; pos < src_max; pos++, BYTE += 1)
        set_bytes((src[pos]), 1, data,  BYTE);
}
INLINER e_SERIAL_CONTROL_DEV p126_device_GET(Pack * src)//See SERIAL_CONTROL_DEV enu
{
    uint8_t * data = src->data;
    switch(get_bits(data, 616, 3))
    {
        case 0:
            return e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_TELEM1;
        case 1:
            return e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_TELEM2;
        case 2:
            return e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_GPS1;
        case 3:
            return e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_GPS2;
        case 4:
            return e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_SHELL;
        default: ;//assert(false);//("Unknown enum ID " + id);
    }
}
INLINER void p126_device_SET(e_SERIAL_CONTROL_DEV  src, Pack * dst)//See SERIAL_CONTROL_DEV enu
{
    uint8_t * data = dst->data;
    int32_t id;
    switch(src)
    {
        case e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_TELEM1:
            id = 0;
            break;
        case e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_TELEM2:
            id = 1;
            break;
        case e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_GPS1:
            id = 2;
            break;
        case e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_GPS2:
            id = 3;
            break;
        case e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_SHELL:
            id = 4;
            break;
        default: ;// assert(false);//("Unknown enum" + id);
    }
    set_bits(id, 3, data, 616);
}
INLINER e_SERIAL_CONTROL_FLAG p126_flags_GET(Pack * src)//See SERIAL_CONTROL_FLAG enu
{
    uint8_t * data = src->data;
    switch(get_bits(data, 619, 3))
    {
        case 0:
            return e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_REPLY;
        case 1:
            return e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_RESPOND;
        case 2:
            return e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_EXCLUSIVE;
        case 3:
            return e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_BLOCKING;
        case 4:
            return e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_MULTI;
        default: ;//assert(false);//("Unknown enum ID " + id);
    }
}
INLINER void p126_flags_SET(e_SERIAL_CONTROL_FLAG  src, Pack * dst)//See SERIAL_CONTROL_FLAG enu
{
    uint8_t * data = dst->data;
    int32_t id;
    switch(src)
    {
        case e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_REPLY:
            id = 0;
            break;
        case e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_RESPOND:
            id = 1;
            break;
        case e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_EXCLUSIVE:
            id = 2;
            break;
        case e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_BLOCKING:
            id = 3;
            break;
        case e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_MULTI:
            id = 4;
            break;
        default: ;// assert(false);//("Unknown enum" + id);
    }
    set_bits(id, 3, data, 619);
}
INLINER uint16_t p127_wn_GET(Pack * src)//GPS Week Number of last baselin
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER void p127_wn_SET(uint16_t  src, Pack * dst)//GPS Week Number of last baselin
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER uint32_t p127_time_last_baseline_ms_GET(Pack * src)//Time since boot of last baseline message received in ms
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 4)));
}
INLINER void p127_time_last_baseline_ms_SET(uint32_t  src, Pack * dst)//Time since boot of last baseline message received in ms
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  2);
}
INLINER uint32_t p127_tow_GET(Pack * src)//GPS Time of Week of last baselin
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  6, 4)));
}
INLINER void p127_tow_SET(uint32_t  src, Pack * dst)//GPS Time of Week of last baselin
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  6);
}
INLINER uint32_t p127_accuracy_GET(Pack * src)//Current estimate of baseline accuracy
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  10, 4)));
}
INLINER void p127_accuracy_SET(uint32_t  src, Pack * dst)//Current estimate of baseline accuracy
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  10);
}
INLINER uint8_t p127_rtk_receiver_id_GET(Pack * src)//Identification of connected RTK receiver
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  14, 1)));
}
INLINER void p127_rtk_receiver_id_SET(uint8_t  src, Pack * dst)//Identification of connected RTK receiver
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  14);
}
INLINER uint8_t p127_rtk_health_GET(Pack * src)//GPS-specific health report for RTK data
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  15, 1)));
}
INLINER void p127_rtk_health_SET(uint8_t  src, Pack * dst)//GPS-specific health report for RTK data
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  15);
}
INLINER uint8_t p127_rtk_rate_GET(Pack * src)//Rate of baseline messages being received by GPS, in H
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  16, 1)));
}
INLINER void p127_rtk_rate_SET(uint8_t  src, Pack * dst)//Rate of baseline messages being received by GPS, in H
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  16);
}
INLINER uint8_t p127_nsats_GET(Pack * src)//Current number of sats used for RTK calculation
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  17, 1)));
}
INLINER void p127_nsats_SET(uint8_t  src, Pack * dst)//Current number of sats used for RTK calculation
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  17);
}
INLINER uint8_t p127_baseline_coords_type_GET(Pack * src)//Coordinate system of baseline. 0 == ECEF, 1 == NE
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  18, 1)));
}
INLINER void p127_baseline_coords_type_SET(uint8_t  src, Pack * dst)//Coordinate system of baseline. 0 == ECEF, 1 == NE
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  18);
}
INLINER int32_t p127_baseline_a_mm_GET(Pack * src)//Current baseline in ECEF x or NED north component in mm
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  19, 4)));
}
INLINER void p127_baseline_a_mm_SET(int32_t  src, Pack * dst)//Current baseline in ECEF x or NED north component in mm
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  19);
}
INLINER int32_t p127_baseline_b_mm_GET(Pack * src)//Current baseline in ECEF y or NED east component in mm
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  23, 4)));
}
INLINER void p127_baseline_b_mm_SET(int32_t  src, Pack * dst)//Current baseline in ECEF y or NED east component in mm
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  23);
}
INLINER int32_t p127_baseline_c_mm_GET(Pack * src)//Current baseline in ECEF z or NED down component in mm
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  27, 4)));
}
INLINER void p127_baseline_c_mm_SET(int32_t  src, Pack * dst)//Current baseline in ECEF z or NED down component in mm
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  27);
}
INLINER int32_t p127_iar_num_hypotheses_GET(Pack * src)//Current number of integer ambiguity hypotheses
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  31, 4)));
}
INLINER void p127_iar_num_hypotheses_SET(int32_t  src, Pack * dst)//Current number of integer ambiguity hypotheses
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  31);
}
INLINER uint16_t p128_wn_GET(Pack * src)//GPS Week Number of last baselin
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER void p128_wn_SET(uint16_t  src, Pack * dst)//GPS Week Number of last baselin
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER uint32_t p128_time_last_baseline_ms_GET(Pack * src)//Time since boot of last baseline message received in ms
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 4)));
}
INLINER void p128_time_last_baseline_ms_SET(uint32_t  src, Pack * dst)//Time since boot of last baseline message received in ms
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  2);
}
INLINER uint32_t p128_tow_GET(Pack * src)//GPS Time of Week of last baselin
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  6, 4)));
}
INLINER void p128_tow_SET(uint32_t  src, Pack * dst)//GPS Time of Week of last baselin
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  6);
}
INLINER uint32_t p128_accuracy_GET(Pack * src)//Current estimate of baseline accuracy
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  10, 4)));
}
INLINER void p128_accuracy_SET(uint32_t  src, Pack * dst)//Current estimate of baseline accuracy
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  10);
}
INLINER uint8_t p128_rtk_receiver_id_GET(Pack * src)//Identification of connected RTK receiver
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  14, 1)));
}
INLINER void p128_rtk_receiver_id_SET(uint8_t  src, Pack * dst)//Identification of connected RTK receiver
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  14);
}
INLINER uint8_t p128_rtk_health_GET(Pack * src)//GPS-specific health report for RTK data
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  15, 1)));
}
INLINER void p128_rtk_health_SET(uint8_t  src, Pack * dst)//GPS-specific health report for RTK data
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  15);
}
INLINER uint8_t p128_rtk_rate_GET(Pack * src)//Rate of baseline messages being received by GPS, in H
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  16, 1)));
}
INLINER void p128_rtk_rate_SET(uint8_t  src, Pack * dst)//Rate of baseline messages being received by GPS, in H
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  16);
}
INLINER uint8_t p128_nsats_GET(Pack * src)//Current number of sats used for RTK calculation
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  17, 1)));
}
INLINER void p128_nsats_SET(uint8_t  src, Pack * dst)//Current number of sats used for RTK calculation
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  17);
}
INLINER uint8_t p128_baseline_coords_type_GET(Pack * src)//Coordinate system of baseline. 0 == ECEF, 1 == NE
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  18, 1)));
}
INLINER void p128_baseline_coords_type_SET(uint8_t  src, Pack * dst)//Coordinate system of baseline. 0 == ECEF, 1 == NE
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  18);
}
INLINER int32_t p128_baseline_a_mm_GET(Pack * src)//Current baseline in ECEF x or NED north component in mm
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  19, 4)));
}
INLINER void p128_baseline_a_mm_SET(int32_t  src, Pack * dst)//Current baseline in ECEF x or NED north component in mm
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  19);
}
INLINER int32_t p128_baseline_b_mm_GET(Pack * src)//Current baseline in ECEF y or NED east component in mm
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  23, 4)));
}
INLINER void p128_baseline_b_mm_SET(int32_t  src, Pack * dst)//Current baseline in ECEF y or NED east component in mm
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  23);
}
INLINER int32_t p128_baseline_c_mm_GET(Pack * src)//Current baseline in ECEF z or NED down component in mm
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  27, 4)));
}
INLINER void p128_baseline_c_mm_SET(int32_t  src, Pack * dst)//Current baseline in ECEF z or NED down component in mm
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  27);
}
INLINER int32_t p128_iar_num_hypotheses_GET(Pack * src)//Current number of integer ambiguity hypotheses
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  31, 4)));
}
INLINER void p128_iar_num_hypotheses_SET(int32_t  src, Pack * dst)//Current number of integer ambiguity hypotheses
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  31);
}
INLINER uint32_t p129_time_boot_ms_GET(Pack * src)//Timestamp (milliseconds since system boot
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 4)));
}
INLINER void p129_time_boot_ms_SET(uint32_t  src, Pack * dst)//Timestamp (milliseconds since system boot
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  0);
}
INLINER int16_t p129_xacc_GET(Pack * src)//X acceleration (mg
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  4, 2)));
}
INLINER void p129_xacc_SET(int16_t  src, Pack * dst)//X acceleration (mg
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  4);
}
INLINER int16_t p129_yacc_GET(Pack * src)//Y acceleration (mg
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  6, 2)));
}
INLINER void p129_yacc_SET(int16_t  src, Pack * dst)//Y acceleration (mg
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  6);
}
INLINER int16_t p129_zacc_GET(Pack * src)//Z acceleration (mg
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  8, 2)));
}
INLINER void p129_zacc_SET(int16_t  src, Pack * dst)//Z acceleration (mg
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  8);
}
INLINER int16_t p129_xgyro_GET(Pack * src)//Angular speed around X axis (millirad /sec
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  10, 2)));
}
INLINER void p129_xgyro_SET(int16_t  src, Pack * dst)//Angular speed around X axis (millirad /sec
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  10);
}
INLINER int16_t p129_ygyro_GET(Pack * src)//Angular speed around Y axis (millirad /sec
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  12, 2)));
}
INLINER void p129_ygyro_SET(int16_t  src, Pack * dst)//Angular speed around Y axis (millirad /sec
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  12);
}
INLINER int16_t p129_zgyro_GET(Pack * src)//Angular speed around Z axis (millirad /sec
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  14, 2)));
}
INLINER void p129_zgyro_SET(int16_t  src, Pack * dst)//Angular speed around Z axis (millirad /sec
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  14);
}
INLINER int16_t p129_xmag_GET(Pack * src)//X Magnetic field (milli tesla
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  16, 2)));
}
INLINER void p129_xmag_SET(int16_t  src, Pack * dst)//X Magnetic field (milli tesla
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  16);
}
INLINER int16_t p129_ymag_GET(Pack * src)//Y Magnetic field (milli tesla
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  18, 2)));
}
INLINER void p129_ymag_SET(int16_t  src, Pack * dst)//Y Magnetic field (milli tesla
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  18);
}
INLINER int16_t p129_zmag_GET(Pack * src)//Z Magnetic field (milli tesla
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  20, 2)));
}
INLINER void p129_zmag_SET(int16_t  src, Pack * dst)//Z Magnetic field (milli tesla
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  20);
}
INLINER uint16_t p130_width_GET(Pack * src)//Width of a matrix or imag
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER void p130_width_SET(uint16_t  src, Pack * dst)//Width of a matrix or imag
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER uint16_t p130_height_GET(Pack * src)//Height of a matrix or imag
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 2)));
}
INLINER void p130_height_SET(uint16_t  src, Pack * dst)//Height of a matrix or imag
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  2);
}
INLINER uint16_t p130_packets_GET(Pack * src)//number of packets beeing sent (set on ACK only
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  4, 2)));
}
INLINER void p130_packets_SET(uint16_t  src, Pack * dst)//number of packets beeing sent (set on ACK only
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  4);
}
INLINER uint32_t p130_size_GET(Pack * src)//total data size in bytes (set on ACK only
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  6, 4)));
}
INLINER void p130_size_SET(uint32_t  src, Pack * dst)//total data size in bytes (set on ACK only
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  6);
}
INLINER uint8_t p130_type_GET(Pack * src)//type of requested/acknowledged data (as defined in ENUM DATA_TYPES in mavlink/include/mavlink_types.
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  10, 1)));
}
INLINER void p130_type_SET(uint8_t  src, Pack * dst)//type of requested/acknowledged data (as defined in ENUM DATA_TYPES in mavlink/include/mavlink_types.
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  10);
}
INLINER uint8_t p130_payload_GET(Pack * src)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  11, 1)));
}
INLINER void p130_payload_SET(uint8_t  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  11);
}
INLINER uint8_t p130_jpg_quality_GET(Pack * src)//JPEG quality out of [1,100
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  12, 1)));
}
INLINER void p130_jpg_quality_SET(uint8_t  src, Pack * dst)//JPEG quality out of [1,100
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  12);
}
INLINER uint16_t p131_seqnr_GET(Pack * src)//sequence number (starting with 0 on every transmission
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER void p131_seqnr_SET(uint16_t  src, Pack * dst)//sequence number (starting with 0 on every transmission
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER uint8_t* p131_data__GET(Pack * src, uint8_t*  dst, int32_t pos) //image data byte
{
    uint8_t * data = src->data;
    for(int32_t BYTE = 2, dst_max = pos + 253; pos < dst_max ; pos++,  BYTE += 1)
        dst[pos] = ((get_bytes(data,  BYTE, 1)));
    return dst;
}

static const  uint32_t p131_data__LEN = 253; //return array length

INLINER  uint8_t*  p131_data__GET_(Pack * src) {return p131_data__GET(src, malloc(253 * sizeof(uint8_t)), 0);}
INLINER void p131_data__SET(uint8_t*  src, int32_t pos, Pack * dst) //image data byte
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  2, src_max = pos + 253; pos < src_max; pos++, BYTE += 1)
        set_bytes((src[pos]), 1, data,  BYTE);
}
INLINER uint16_t p132_min_distance_GET(Pack * src)//Minimum distance the sensor can measure in centimeter
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER void p132_min_distance_SET(uint16_t  src, Pack * dst)//Minimum distance the sensor can measure in centimeter
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER uint16_t p132_max_distance_GET(Pack * src)//Maximum distance the sensor can measure in centimeter
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 2)));
}
INLINER void p132_max_distance_SET(uint16_t  src, Pack * dst)//Maximum distance the sensor can measure in centimeter
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  2);
}
INLINER uint16_t p132_current_distance_GET(Pack * src)//Current distance readin
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  4, 2)));
}
INLINER void p132_current_distance_SET(uint16_t  src, Pack * dst)//Current distance readin
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  4);
}
INLINER uint32_t p132_time_boot_ms_GET(Pack * src)//Time since system boo
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  6, 4)));
}
INLINER void p132_time_boot_ms_SET(uint32_t  src, Pack * dst)//Time since system boo
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  6);
}
INLINER uint8_t p132_id_GET(Pack * src)//Onboard ID of the senso
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  10, 1)));
}
INLINER void p132_id_SET(uint8_t  src, Pack * dst)//Onboard ID of the senso
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  10);
}
INLINER uint8_t p132_covariance_GET(Pack * src)//Measurement covariance in centimeters, 0 for unknown / invalid reading
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  11, 1)));
}
INLINER void p132_covariance_SET(uint8_t  src, Pack * dst)//Measurement covariance in centimeters, 0 for unknown / invalid reading
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  11);
}
INLINER e_MAV_DISTANCE_SENSOR p132_type_GET(Pack * src)//Type from MAV_DISTANCE_SENSOR enum
{
    uint8_t * data = src->data;
    return  0 + (int)get_bits(data, 96, 3);
}
INLINER void p132_type_SET(e_MAV_DISTANCE_SENSOR  src, Pack * dst)//Type from MAV_DISTANCE_SENSOR enum
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 3, data, 96);
}
INLINER e_MAV_SENSOR_ORIENTATION p132_orientation_GET(Pack * src)
{
    uint8_t * data = src->data;
    return  0 + (int)get_bits(data, 99, 6);
}
INLINER void p132_orientation_SET(e_MAV_SENSOR_ORIENTATION  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 6, data, 99);
}
INLINER uint16_t p133_grid_spacing_GET(Pack * src)//Grid spacing in meter
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER void p133_grid_spacing_SET(uint16_t  src, Pack * dst)//Grid spacing in meter
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER uint64_t p133_mask_GET(Pack * src)//Bitmask of requested 4x4 grids (row major 8x7 array of grids, 56 bits
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 8)));
}
INLINER void p133_mask_SET(uint64_t  src, Pack * dst)//Bitmask of requested 4x4 grids (row major 8x7 array of grids, 56 bits
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  2);
}
INLINER int32_t p133_lat_GET(Pack * src)//Latitude of SW corner of first grid (degrees *10^7
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  10, 4)));
}
INLINER void p133_lat_SET(int32_t  src, Pack * dst)//Latitude of SW corner of first grid (degrees *10^7
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  10);
}
INLINER int32_t p133_lon_GET(Pack * src)//Longitude of SW corner of first grid (in degrees *10^7
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  14, 4)));
}
INLINER void p133_lon_SET(int32_t  src, Pack * dst)//Longitude of SW corner of first grid (in degrees *10^7
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  14);
}
INLINER uint16_t p134_grid_spacing_GET(Pack * src)//Grid spacing in meter
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER void p134_grid_spacing_SET(uint16_t  src, Pack * dst)//Grid spacing in meter
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER int32_t p134_lat_GET(Pack * src)//Latitude of SW corner of first grid (degrees *10^7
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  2, 4)));
}
INLINER void p134_lat_SET(int32_t  src, Pack * dst)//Latitude of SW corner of first grid (degrees *10^7
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  2);
}
INLINER int32_t p134_lon_GET(Pack * src)//Longitude of SW corner of first grid (in degrees *10^7
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  6, 4)));
}
INLINER void p134_lon_SET(int32_t  src, Pack * dst)//Longitude of SW corner of first grid (in degrees *10^7
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  6);
}
INLINER uint8_t p134_gridbit_GET(Pack * src)//bit within the terrain request mas
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  10, 1)));
}
INLINER void p134_gridbit_SET(uint8_t  src, Pack * dst)//bit within the terrain request mas
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  10);
}
INLINER int16_t* p134_data__GET(Pack * src, int16_t*  dst, int32_t pos) //Terrain data in meters AMS
{
    uint8_t * data = src->data;
    for(int32_t BYTE = 11, dst_max = pos + 16; pos < dst_max ; pos++,  BYTE += 2)
        dst[pos] = ((int16_t)(get_bytes(data,  BYTE, 2)));
    return dst;
}

static const  uint32_t p134_data__LEN = 16; //return array length

INLINER  int16_t*  p134_data__GET_(Pack * src) {return p134_data__GET(src, malloc(16 * sizeof(int16_t)), 0);}
INLINER void p134_data__SET(int16_t*  src, int32_t pos, Pack * dst) //Terrain data in meters AMS
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  11, src_max = pos + 16; pos < src_max; pos++, BYTE += 2)
        set_bytes((uint16_t)(src[pos]), 2, data,  BYTE);
}
INLINER int32_t p135_lat_GET(Pack * src)//Latitude (degrees *10^7
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  0, 4)));
}
INLINER void p135_lat_SET(int32_t  src, Pack * dst)//Latitude (degrees *10^7
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  0);
}
INLINER int32_t p135_lon_GET(Pack * src)//Longitude (degrees *10^7
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  4, 4)));
}
INLINER void p135_lon_SET(int32_t  src, Pack * dst)//Longitude (degrees *10^7
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  4);
}
INLINER uint16_t p136_spacing_GET(Pack * src)//grid spacing (zero if terrain at this location unavailable
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER void p136_spacing_SET(uint16_t  src, Pack * dst)//grid spacing (zero if terrain at this location unavailable
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER uint16_t p136_pending_GET(Pack * src)//Number of 4x4 terrain blocks waiting to be received or read from dis
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 2)));
}
INLINER void p136_pending_SET(uint16_t  src, Pack * dst)//Number of 4x4 terrain blocks waiting to be received or read from dis
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  2);
}
INLINER uint16_t p136_loaded_GET(Pack * src)//Number of 4x4 terrain blocks in memor
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  4, 2)));
}
INLINER void p136_loaded_SET(uint16_t  src, Pack * dst)//Number of 4x4 terrain blocks in memor
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  4);
}
INLINER int32_t p136_lat_GET(Pack * src)//Latitude (degrees *10^7
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  6, 4)));
}
INLINER void p136_lat_SET(int32_t  src, Pack * dst)//Latitude (degrees *10^7
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  6);
}
INLINER int32_t p136_lon_GET(Pack * src)//Longitude (degrees *10^7
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  10, 4)));
}
INLINER void p136_lon_SET(int32_t  src, Pack * dst)//Longitude (degrees *10^7
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  10);
}
INLINER float p136_terrain_height_GET(Pack * src)//Terrain height in meters AMS
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  14, 4)));
}
INLINER void p136_terrain_height_SET(float  src, Pack * dst)//Terrain height in meters AMS
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  14);
}
INLINER float p136_current_height_GET(Pack * src)//Current vehicle height above lat/lon terrain height (meters
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  18, 4)));
}
INLINER void p136_current_height_SET(float  src, Pack * dst)//Current vehicle height above lat/lon terrain height (meters
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  18);
}
INLINER uint32_t p137_time_boot_ms_GET(Pack * src)//Timestamp (milliseconds since system boot
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 4)));
}
INLINER void p137_time_boot_ms_SET(uint32_t  src, Pack * dst)//Timestamp (milliseconds since system boot
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  0);
}
INLINER float p137_press_abs_GET(Pack * src)//Absolute pressure (hectopascal
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  4, 4)));
}
INLINER void p137_press_abs_SET(float  src, Pack * dst)//Absolute pressure (hectopascal
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  4);
}
INLINER float p137_press_diff_GET(Pack * src)//Differential pressure 1 (hectopascal
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  8, 4)));
}
INLINER void p137_press_diff_SET(float  src, Pack * dst)//Differential pressure 1 (hectopascal
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  8);
}
INLINER int16_t p137_temperature_GET(Pack * src)//Temperature measurement (0.01 degrees celsius
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  12, 2)));
}
INLINER void p137_temperature_SET(int16_t  src, Pack * dst)//Temperature measurement (0.01 degrees celsius
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  12);
}
INLINER uint64_t p138_time_usec_GET(Pack * src)//Timestamp (micros since boot or Unix epoch
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 8)));
}
INLINER void p138_time_usec_SET(uint64_t  src, Pack * dst)//Timestamp (micros since boot or Unix epoch
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  0);
}
INLINER float* p138_q_GET(Pack * src, float*  dst, int32_t pos) //Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0
{
    uint8_t * data = src->data;
    for(int32_t BYTE = 8, dst_max = pos + 4; pos < dst_max ; pos++,  BYTE += 4)
        dst[pos] = (intBitsToFloat(get_bytes(data,  BYTE, 4)));
    return dst;
}

static const  uint32_t p138_q_LEN = 4; //return array length

INLINER  float*  p138_q_GET_(Pack * src) {return p138_q_GET(src, malloc(4 * sizeof(float)), 0);}
INLINER void p138_q_SET(float*  src, int32_t pos, Pack * dst) //Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  8, src_max = pos + 4; pos < src_max; pos++, BYTE += 4)
        set_bytes(floatToIntBits(src[pos]), 4, data,  BYTE);
}
INLINER float p138_x_GET(Pack * src)//X position in meters (NED
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  24, 4)));
}
INLINER void p138_x_SET(float  src, Pack * dst)//X position in meters (NED
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  24);
}
INLINER float p138_y_GET(Pack * src)//Y position in meters (NED
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  28, 4)));
}
INLINER void p138_y_SET(float  src, Pack * dst)//Y position in meters (NED
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  28);
}
INLINER float p138_z_GET(Pack * src)//Z position in meters (NED
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  32, 4)));
}
INLINER void p138_z_SET(float  src, Pack * dst)//Z position in meters (NED
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  32);
}
INLINER uint64_t p139_time_usec_GET(Pack * src)//Timestamp (micros since boot or Unix epoch
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 8)));
}
INLINER void p139_time_usec_SET(uint64_t  src, Pack * dst)//Timestamp (micros since boot or Unix epoch
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  0);
}
INLINER uint8_t p139_group_mlx_GET(Pack * src)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  8, 1)));
}
INLINER void p139_group_mlx_SET(uint8_t  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  8);
}
INLINER uint8_t p139_target_system_GET(Pack * src)//System I
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  9, 1)));
}
INLINER void p139_target_system_SET(uint8_t  src, Pack * dst)//System I
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  9);
}
INLINER uint8_t p139_target_component_GET(Pack * src)//Component I
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  10, 1)));
}
INLINER void p139_target_component_SET(uint8_t  src, Pack * dst)//Component I
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  10);
}
INLINER float* p139_controls_GET(Pack * src, float*  dst, int32_t pos)
{
    uint8_t * data = src->data;
    for(int32_t BYTE = 11, dst_max = pos + 8; pos < dst_max ; pos++,  BYTE += 4)
        dst[pos] = (intBitsToFloat(get_bytes(data,  BYTE, 4)));
    return dst;
}

static const  uint32_t p139_controls_LEN = 8; //return array length

INLINER  float*  p139_controls_GET_(Pack * src) {return p139_controls_GET(src, malloc(8 * sizeof(float)), 0);}
INLINER void p139_controls_SET(float*  src, int32_t pos, Pack * dst)
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  11, src_max = pos + 8; pos < src_max; pos++, BYTE += 4)
        set_bytes(floatToIntBits(src[pos]), 4, data,  BYTE);
}
INLINER uint64_t p140_time_usec_GET(Pack * src)//Timestamp (micros since boot or Unix epoch
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 8)));
}
INLINER void p140_time_usec_SET(uint64_t  src, Pack * dst)//Timestamp (micros since boot or Unix epoch
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  0);
}
INLINER uint8_t p140_group_mlx_GET(Pack * src)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  8, 1)));
}
INLINER void p140_group_mlx_SET(uint8_t  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  8);
}
INLINER float* p140_controls_GET(Pack * src, float*  dst, int32_t pos)
{
    uint8_t * data = src->data;
    for(int32_t BYTE = 9, dst_max = pos + 8; pos < dst_max ; pos++,  BYTE += 4)
        dst[pos] = (intBitsToFloat(get_bytes(data,  BYTE, 4)));
    return dst;
}

static const  uint32_t p140_controls_LEN = 8; //return array length

INLINER  float*  p140_controls_GET_(Pack * src) {return p140_controls_GET(src, malloc(8 * sizeof(float)), 0);}
INLINER void p140_controls_SET(float*  src, int32_t pos, Pack * dst)
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  9, src_max = pos + 8; pos < src_max; pos++, BYTE += 4)
        set_bytes(floatToIntBits(src[pos]), 4, data,  BYTE);
}
INLINER uint64_t p141_time_usec_GET(Pack * src)//Timestamp (micros since boot or Unix epoch
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 8)));
}
INLINER void p141_time_usec_SET(uint64_t  src, Pack * dst)//Timestamp (micros since boot or Unix epoch
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  0);
}
INLINER float p141_altitude_monotonic_GET(Pack * src)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  8, 4)));
}
INLINER void p141_altitude_monotonic_SET(float  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  8);
}
INLINER float p141_altitude_amsl_GET(Pack * src)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  12, 4)));
}
INLINER void p141_altitude_amsl_SET(float  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  12);
}
INLINER float p141_altitude_local_GET(Pack * src)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  16, 4)));
}
INLINER void p141_altitude_local_SET(float  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  16);
}
INLINER float p141_altitude_relative_GET(Pack * src)//This is the altitude above the home position. It resets on each change of the current home positio
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  20, 4)));
}
INLINER void p141_altitude_relative_SET(float  src, Pack * dst)//This is the altitude above the home position. It resets on each change of the current home positio
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  20);
}
INLINER float p141_altitude_terrain_GET(Pack * src)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  24, 4)));
}
INLINER void p141_altitude_terrain_SET(float  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  24);
}
INLINER float p141_bottom_clearance_GET(Pack * src)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  28, 4)));
}
INLINER void p141_bottom_clearance_SET(float  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  28);
}
INLINER uint8_t p142_request_id_GET(Pack * src)//Request ID. This ID should be re-used when sending back URI content
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 1)));
}
INLINER void p142_request_id_SET(uint8_t  src, Pack * dst)//Request ID. This ID should be re-used when sending back URI content
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  0);
}
INLINER uint8_t p142_uri_type_GET(Pack * src)//The type of requested URI. 0 = a file via URL. 1 = a UAVCAN binar
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  1, 1)));
}
INLINER void p142_uri_type_SET(uint8_t  src, Pack * dst)//The type of requested URI. 0 = a file via URL. 1 = a UAVCAN binar
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  1);
}
INLINER uint8_t* p142_uri_GET(Pack * src, uint8_t*  dst, int32_t pos)
{
    uint8_t * data = src->data;
    for(int32_t BYTE = 2, dst_max = pos + 120; pos < dst_max ; pos++,  BYTE += 1)
        dst[pos] = ((get_bytes(data,  BYTE, 1)));
    return dst;
}

static const  uint32_t p142_uri_LEN = 120; //return array length

INLINER  uint8_t*  p142_uri_GET_(Pack * src) {return p142_uri_GET(src, malloc(120 * sizeof(uint8_t)), 0);}
INLINER void p142_uri_SET(uint8_t*  src, int32_t pos, Pack * dst)
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  2, src_max = pos + 120; pos < src_max; pos++, BYTE += 1)
        set_bytes((src[pos]), 1, data,  BYTE);
}
INLINER uint8_t p142_transfer_type_GET(Pack * src)//The way the autopilot wants to receive the URI. 0 = MAVLink FTP. 1 = binary stream
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  122, 1)));
}
INLINER void p142_transfer_type_SET(uint8_t  src, Pack * dst)//The way the autopilot wants to receive the URI. 0 = MAVLink FTP. 1 = binary stream
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  122);
}
INLINER uint8_t* p142_storage_GET(Pack * src, uint8_t*  dst, int32_t pos)
{
    uint8_t * data = src->data;
    for(int32_t BYTE = 123, dst_max = pos + 120; pos < dst_max ; pos++,  BYTE += 1)
        dst[pos] = ((get_bytes(data,  BYTE, 1)));
    return dst;
}

static const  uint32_t p142_storage_LEN = 120; //return array length

INLINER  uint8_t*  p142_storage_GET_(Pack * src) {return p142_storage_GET(src, malloc(120 * sizeof(uint8_t)), 0);}
INLINER void p142_storage_SET(uint8_t*  src, int32_t pos, Pack * dst)
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  123, src_max = pos + 120; pos < src_max; pos++, BYTE += 1)
        set_bytes((src[pos]), 1, data,  BYTE);
}
INLINER uint32_t p143_time_boot_ms_GET(Pack * src)//Timestamp (milliseconds since system boot
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 4)));
}
INLINER void p143_time_boot_ms_SET(uint32_t  src, Pack * dst)//Timestamp (milliseconds since system boot
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  0);
}
INLINER float p143_press_abs_GET(Pack * src)//Absolute pressure (hectopascal
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  4, 4)));
}
INLINER void p143_press_abs_SET(float  src, Pack * dst)//Absolute pressure (hectopascal
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  4);
}
INLINER float p143_press_diff_GET(Pack * src)//Differential pressure 1 (hectopascal
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  8, 4)));
}
INLINER void p143_press_diff_SET(float  src, Pack * dst)//Differential pressure 1 (hectopascal
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  8);
}
INLINER int16_t p143_temperature_GET(Pack * src)//Temperature measurement (0.01 degrees celsius
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  12, 2)));
}
INLINER void p143_temperature_SET(int16_t  src, Pack * dst)//Temperature measurement (0.01 degrees celsius
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  12);
}
INLINER uint64_t p144_timestamp_GET(Pack * src)//Timestamp in milliseconds since system boo
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 8)));
}
INLINER void p144_timestamp_SET(uint64_t  src, Pack * dst)//Timestamp in milliseconds since system boo
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  0);
}
INLINER uint64_t p144_custom_state_GET(Pack * src)//button states or switches of a tracker devic
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  8, 8)));
}
INLINER void p144_custom_state_SET(uint64_t  src, Pack * dst)//button states or switches of a tracker devic
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  8);
}
INLINER uint8_t p144_est_capabilities_GET(Pack * src)//bit positions for tracker reporting capabilities (POS = 0, VEL = 1, ACCEL = 2, ATT + RATES = 3
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  16, 1)));
}
INLINER void p144_est_capabilities_SET(uint8_t  src, Pack * dst)//bit positions for tracker reporting capabilities (POS = 0, VEL = 1, ACCEL = 2, ATT + RATES = 3
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  16);
}
INLINER int32_t p144_lat_GET(Pack * src)//Latitude (WGS84), in degrees * 1E
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  17, 4)));
}
INLINER void p144_lat_SET(int32_t  src, Pack * dst)//Latitude (WGS84), in degrees * 1E
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  17);
}
INLINER int32_t p144_lon_GET(Pack * src)//Longitude (WGS84), in degrees * 1E
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  21, 4)));
}
INLINER void p144_lon_SET(int32_t  src, Pack * dst)//Longitude (WGS84), in degrees * 1E
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  21);
}
INLINER float p144_alt_GET(Pack * src)//AMSL, in meter
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  25, 4)));
}
INLINER void p144_alt_SET(float  src, Pack * dst)//AMSL, in meter
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  25);
}
INLINER float* p144_vel_GET(Pack * src, float*  dst, int32_t pos) //target velocity (0,0,0) for unknow
{
    uint8_t * data = src->data;
    for(int32_t BYTE = 29, dst_max = pos + 3; pos < dst_max ; pos++,  BYTE += 4)
        dst[pos] = (intBitsToFloat(get_bytes(data,  BYTE, 4)));
    return dst;
}

static const  uint32_t p144_vel_LEN = 3; //return array length

INLINER  float*  p144_vel_GET_(Pack * src) {return p144_vel_GET(src, malloc(3 * sizeof(float)), 0);}
INLINER void p144_vel_SET(float*  src, int32_t pos, Pack * dst) //target velocity (0,0,0) for unknow
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  29, src_max = pos + 3; pos < src_max; pos++, BYTE += 4)
        set_bytes(floatToIntBits(src[pos]), 4, data,  BYTE);
}
INLINER float* p144_acc_GET(Pack * src, float*  dst, int32_t pos) //linear target acceleration (0,0,0) for unknow
{
    uint8_t * data = src->data;
    for(int32_t BYTE = 41, dst_max = pos + 3; pos < dst_max ; pos++,  BYTE += 4)
        dst[pos] = (intBitsToFloat(get_bytes(data,  BYTE, 4)));
    return dst;
}

static const  uint32_t p144_acc_LEN = 3; //return array length

INLINER  float*  p144_acc_GET_(Pack * src) {return p144_acc_GET(src, malloc(3 * sizeof(float)), 0);}
INLINER void p144_acc_SET(float*  src, int32_t pos, Pack * dst) //linear target acceleration (0,0,0) for unknow
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  41, src_max = pos + 3; pos < src_max; pos++, BYTE += 4)
        set_bytes(floatToIntBits(src[pos]), 4, data,  BYTE);
}
INLINER float* p144_attitude_q_GET(Pack * src, float*  dst, int32_t pos) //(1 0 0 0 for unknown
{
    uint8_t * data = src->data;
    for(int32_t BYTE = 53, dst_max = pos + 4; pos < dst_max ; pos++,  BYTE += 4)
        dst[pos] = (intBitsToFloat(get_bytes(data,  BYTE, 4)));
    return dst;
}

static const  uint32_t p144_attitude_q_LEN = 4; //return array length

INLINER  float*  p144_attitude_q_GET_(Pack * src) {return p144_attitude_q_GET(src, malloc(4 * sizeof(float)), 0);}
INLINER void p144_attitude_q_SET(float*  src, int32_t pos, Pack * dst) //(1 0 0 0 for unknown
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  53, src_max = pos + 4; pos < src_max; pos++, BYTE += 4)
        set_bytes(floatToIntBits(src[pos]), 4, data,  BYTE);
}
INLINER float* p144_rates_GET(Pack * src, float*  dst, int32_t pos) //(0 0 0 for unknown
{
    uint8_t * data = src->data;
    for(int32_t BYTE = 69, dst_max = pos + 3; pos < dst_max ; pos++,  BYTE += 4)
        dst[pos] = (intBitsToFloat(get_bytes(data,  BYTE, 4)));
    return dst;
}

static const  uint32_t p144_rates_LEN = 3; //return array length

INLINER  float*  p144_rates_GET_(Pack * src) {return p144_rates_GET(src, malloc(3 * sizeof(float)), 0);}
INLINER void p144_rates_SET(float*  src, int32_t pos, Pack * dst) //(0 0 0 for unknown
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  69, src_max = pos + 3; pos < src_max; pos++, BYTE += 4)
        set_bytes(floatToIntBits(src[pos]), 4, data,  BYTE);
}
INLINER float* p144_position_cov_GET(Pack * src, float*  dst, int32_t pos) //eph ep
{
    uint8_t * data = src->data;
    for(int32_t BYTE = 81, dst_max = pos + 3; pos < dst_max ; pos++,  BYTE += 4)
        dst[pos] = (intBitsToFloat(get_bytes(data,  BYTE, 4)));
    return dst;
}

static const  uint32_t p144_position_cov_LEN = 3; //return array length

INLINER  float*  p144_position_cov_GET_(Pack * src) {return p144_position_cov_GET(src, malloc(3 * sizeof(float)), 0);}
INLINER void p144_position_cov_SET(float*  src, int32_t pos, Pack * dst) //eph ep
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  81, src_max = pos + 3; pos < src_max; pos++, BYTE += 4)
        set_bytes(floatToIntBits(src[pos]), 4, data,  BYTE);
}
INLINER uint64_t p146_time_usec_GET(Pack * src)//Timestamp (micros since boot or Unix epoch
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 8)));
}
INLINER void p146_time_usec_SET(uint64_t  src, Pack * dst)//Timestamp (micros since boot or Unix epoch
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  0);
}
INLINER float p146_x_acc_GET(Pack * src)//X acceleration in body fram
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  8, 4)));
}
INLINER void p146_x_acc_SET(float  src, Pack * dst)//X acceleration in body fram
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  8);
}
INLINER float p146_y_acc_GET(Pack * src)//Y acceleration in body fram
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  12, 4)));
}
INLINER void p146_y_acc_SET(float  src, Pack * dst)//Y acceleration in body fram
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  12);
}
INLINER float p146_z_acc_GET(Pack * src)//Z acceleration in body fram
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  16, 4)));
}
INLINER void p146_z_acc_SET(float  src, Pack * dst)//Z acceleration in body fram
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  16);
}
INLINER float p146_x_vel_GET(Pack * src)//X velocity in body fram
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  20, 4)));
}
INLINER void p146_x_vel_SET(float  src, Pack * dst)//X velocity in body fram
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  20);
}
INLINER float p146_y_vel_GET(Pack * src)//Y velocity in body fram
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  24, 4)));
}
INLINER void p146_y_vel_SET(float  src, Pack * dst)//Y velocity in body fram
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  24);
}
INLINER float p146_z_vel_GET(Pack * src)//Z velocity in body fram
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  28, 4)));
}
INLINER void p146_z_vel_SET(float  src, Pack * dst)//Z velocity in body fram
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  28);
}
INLINER float p146_x_pos_GET(Pack * src)//X position in local fram
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  32, 4)));
}
INLINER void p146_x_pos_SET(float  src, Pack * dst)//X position in local fram
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  32);
}
INLINER float p146_y_pos_GET(Pack * src)//Y position in local fram
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  36, 4)));
}
INLINER void p146_y_pos_SET(float  src, Pack * dst)//Y position in local fram
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  36);
}
INLINER float p146_z_pos_GET(Pack * src)//Z position in local fram
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  40, 4)));
}
INLINER void p146_z_pos_SET(float  src, Pack * dst)//Z position in local fram
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  40);
}
INLINER float p146_airspeed_GET(Pack * src)//Airspeed, set to -1 if unknow
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  44, 4)));
}
INLINER void p146_airspeed_SET(float  src, Pack * dst)//Airspeed, set to -1 if unknow
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  44);
}
INLINER float* p146_vel_variance_GET(Pack * src, float*  dst, int32_t pos) //Variance of body velocity estimat
{
    uint8_t * data = src->data;
    for(int32_t BYTE = 48, dst_max = pos + 3; pos < dst_max ; pos++,  BYTE += 4)
        dst[pos] = (intBitsToFloat(get_bytes(data,  BYTE, 4)));
    return dst;
}

static const  uint32_t p146_vel_variance_LEN = 3; //return array length

INLINER  float*  p146_vel_variance_GET_(Pack * src) {return p146_vel_variance_GET(src, malloc(3 * sizeof(float)), 0);}
INLINER void p146_vel_variance_SET(float*  src, int32_t pos, Pack * dst) //Variance of body velocity estimat
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  48, src_max = pos + 3; pos < src_max; pos++, BYTE += 4)
        set_bytes(floatToIntBits(src[pos]), 4, data,  BYTE);
}
INLINER float* p146_pos_variance_GET(Pack * src, float*  dst, int32_t pos) //Variance in local positio
{
    uint8_t * data = src->data;
    for(int32_t BYTE = 60, dst_max = pos + 3; pos < dst_max ; pos++,  BYTE += 4)
        dst[pos] = (intBitsToFloat(get_bytes(data,  BYTE, 4)));
    return dst;
}

static const  uint32_t p146_pos_variance_LEN = 3; //return array length

INLINER  float*  p146_pos_variance_GET_(Pack * src) {return p146_pos_variance_GET(src, malloc(3 * sizeof(float)), 0);}
INLINER void p146_pos_variance_SET(float*  src, int32_t pos, Pack * dst) //Variance in local positio
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  60, src_max = pos + 3; pos < src_max; pos++, BYTE += 4)
        set_bytes(floatToIntBits(src[pos]), 4, data,  BYTE);
}
INLINER float* p146_q_GET(Pack * src, float*  dst, int32_t pos) //The attitude, represented as Quaternio
{
    uint8_t * data = src->data;
    for(int32_t BYTE = 72, dst_max = pos + 4; pos < dst_max ; pos++,  BYTE += 4)
        dst[pos] = (intBitsToFloat(get_bytes(data,  BYTE, 4)));
    return dst;
}

static const  uint32_t p146_q_LEN = 4; //return array length

INLINER  float*  p146_q_GET_(Pack * src) {return p146_q_GET(src, malloc(4 * sizeof(float)), 0);}
INLINER void p146_q_SET(float*  src, int32_t pos, Pack * dst) //The attitude, represented as Quaternio
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  72, src_max = pos + 4; pos < src_max; pos++, BYTE += 4)
        set_bytes(floatToIntBits(src[pos]), 4, data,  BYTE);
}
INLINER float p146_roll_rate_GET(Pack * src)//Angular rate in roll axi
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  88, 4)));
}
INLINER void p146_roll_rate_SET(float  src, Pack * dst)//Angular rate in roll axi
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  88);
}
INLINER float p146_pitch_rate_GET(Pack * src)//Angular rate in pitch axi
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  92, 4)));
}
INLINER void p146_pitch_rate_SET(float  src, Pack * dst)//Angular rate in pitch axi
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  92);
}
INLINER float p146_yaw_rate_GET(Pack * src)//Angular rate in yaw axi
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  96, 4)));
}
INLINER void p146_yaw_rate_SET(float  src, Pack * dst)//Angular rate in yaw axi
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  96);
}
INLINER uint16_t* p147_voltages_GET(Pack * src, uint16_t*  dst, int32_t pos)
{
    uint8_t * data = src->data;
    for(int32_t BYTE = 0, dst_max = pos + 10; pos < dst_max ; pos++,  BYTE += 2)
        dst[pos] = ((get_bytes(data,  BYTE, 2)));
    return dst;
}

static const  uint32_t p147_voltages_LEN = 10; //return array length

INLINER  uint16_t*  p147_voltages_GET_(Pack * src) {return p147_voltages_GET(src, malloc(10 * sizeof(uint16_t)), 0);}
INLINER void p147_voltages_SET(uint16_t*  src, int32_t pos, Pack * dst)
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  0, src_max = pos + 10; pos < src_max; pos++, BYTE += 2)
        set_bytes((src[pos]), 2, data,  BYTE);
}
INLINER uint8_t p147_id_GET(Pack * src)//Battery I
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  20, 1)));
}
INLINER void p147_id_SET(uint8_t  src, Pack * dst)//Battery I
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  20);
}
INLINER int16_t p147_temperature_GET(Pack * src)//Temperature of the battery in centi-degrees celsius. INT16_MAX for unknown temperature
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  21, 2)));
}
INLINER void p147_temperature_SET(int16_t  src, Pack * dst)//Temperature of the battery in centi-degrees celsius. INT16_MAX for unknown temperature
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  21);
}
INLINER int16_t p147_current_battery_GET(Pack * src)//Battery current, in 10*milliamperes (1 = 10 milliampere), -1: autopilot does not measure the curre
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  23, 2)));
}
INLINER void p147_current_battery_SET(int16_t  src, Pack * dst)//Battery current, in 10*milliamperes (1 = 10 milliampere), -1: autopilot does not measure the curre
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  23);
}
INLINER int32_t p147_current_consumed_GET(Pack * src)//Consumed charge, in milliampere hours (1 = 1 mAh), -1: autopilot does not provide mAh consumption estima
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  25, 4)));
}
INLINER void p147_current_consumed_SET(int32_t  src, Pack * dst)//Consumed charge, in milliampere hours (1 = 1 mAh), -1: autopilot does not provide mAh consumption estima
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  25);
}
INLINER int32_t p147_energy_consumed_GET(Pack * src)
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  29, 4)));
}
INLINER void p147_energy_consumed_SET(int32_t  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  29);
}
INLINER int8_t p147_battery_remaining_GET(Pack * src)//Remaining battery energy: (0%: 0, 100%: 100), -1: autopilot does not estimate the remaining batter
{
    uint8_t * data = src->data;
    return ((int8_t)(get_bytes(data,  33, 1)));
}
INLINER void p147_battery_remaining_SET(int8_t  src, Pack * dst)//Remaining battery energy: (0%: 0, 100%: 100), -1: autopilot does not estimate the remaining batter
{
    uint8_t * data = dst->data;
    set_bytes((uint8_t)(src), 1, data,  33);
}
INLINER e_MAV_BATTERY_FUNCTION p147_battery_function_GET(Pack * src)//Function of the batter
{
    uint8_t * data = src->data;
    return  0 + (int)get_bits(data, 272, 3);
}
INLINER void p147_battery_function_SET(e_MAV_BATTERY_FUNCTION  src, Pack * dst)//Function of the batter
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 3, data, 272);
}
INLINER e_MAV_BATTERY_TYPE p147_type_GET(Pack * src)//Type (chemistry) of the batter
{
    uint8_t * data = src->data;
    return  0 + (int)get_bits(data, 275, 3);
}
INLINER void p147_type_SET(e_MAV_BATTERY_TYPE  src, Pack * dst)//Type (chemistry) of the batter
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 3, data, 275);
}
INLINER uint16_t p148_vendor_id_GET(Pack * src)//ID of the board vendo
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER void p148_vendor_id_SET(uint16_t  src, Pack * dst)//ID of the board vendo
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER uint16_t p148_product_id_GET(Pack * src)//ID of the produc
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 2)));
}
INLINER void p148_product_id_SET(uint16_t  src, Pack * dst)//ID of the produc
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  2);
}
INLINER uint32_t p148_flight_sw_version_GET(Pack * src)//Firmware version numbe
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  4, 4)));
}
INLINER void p148_flight_sw_version_SET(uint32_t  src, Pack * dst)//Firmware version numbe
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  4);
}
INLINER uint32_t p148_middleware_sw_version_GET(Pack * src)//Middleware version numbe
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  8, 4)));
}
INLINER void p148_middleware_sw_version_SET(uint32_t  src, Pack * dst)//Middleware version numbe
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  8);
}
INLINER uint32_t p148_os_sw_version_GET(Pack * src)//Operating system version numbe
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  12, 4)));
}
INLINER void p148_os_sw_version_SET(uint32_t  src, Pack * dst)//Operating system version numbe
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  12);
}
INLINER uint32_t p148_board_version_GET(Pack * src)//HW / board version (last 8 bytes should be silicon ID, if any
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  16, 4)));
}
INLINER void p148_board_version_SET(uint32_t  src, Pack * dst)//HW / board version (last 8 bytes should be silicon ID, if any
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  16);
}
INLINER uint64_t p148_uid_GET(Pack * src)//UID if provided by hardware (see uid2
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  20, 8)));
}
INLINER void p148_uid_SET(uint64_t  src, Pack * dst)//UID if provided by hardware (see uid2
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  20);
}
INLINER uint8_t* p148_flight_custom_version_GET(Pack * src, uint8_t*  dst, int32_t pos)
{
    uint8_t * data = src->data;
    for(int32_t BYTE = 28, dst_max = pos + 8; pos < dst_max ; pos++,  BYTE += 1)
        dst[pos] = ((get_bytes(data,  BYTE, 1)));
    return dst;
}

static const  uint32_t p148_flight_custom_version_LEN = 8; //return array length

INLINER  uint8_t*  p148_flight_custom_version_GET_(Pack * src) {return p148_flight_custom_version_GET(src, malloc(8 * sizeof(uint8_t)), 0);}
INLINER void p148_flight_custom_version_SET(uint8_t*  src, int32_t pos, Pack * dst)
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  28, src_max = pos + 8; pos < src_max; pos++, BYTE += 1)
        set_bytes((src[pos]), 1, data,  BYTE);
}
INLINER uint8_t* p148_middleware_custom_version_GET(Pack * src, uint8_t*  dst, int32_t pos)
{
    uint8_t * data = src->data;
    for(int32_t BYTE = 36, dst_max = pos + 8; pos < dst_max ; pos++,  BYTE += 1)
        dst[pos] = ((get_bytes(data,  BYTE, 1)));
    return dst;
}

static const  uint32_t p148_middleware_custom_version_LEN = 8; //return array length

INLINER  uint8_t*  p148_middleware_custom_version_GET_(Pack * src) {return p148_middleware_custom_version_GET(src, malloc(8 * sizeof(uint8_t)), 0);}
INLINER void p148_middleware_custom_version_SET(uint8_t*  src, int32_t pos, Pack * dst)
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  36, src_max = pos + 8; pos < src_max; pos++, BYTE += 1)
        set_bytes((src[pos]), 1, data,  BYTE);
}
INLINER uint8_t* p148_os_custom_version_GET(Pack * src, uint8_t*  dst, int32_t pos)
{
    uint8_t * data = src->data;
    for(int32_t BYTE = 44, dst_max = pos + 8; pos < dst_max ; pos++,  BYTE += 1)
        dst[pos] = ((get_bytes(data,  BYTE, 1)));
    return dst;
}

static const  uint32_t p148_os_custom_version_LEN = 8; //return array length

INLINER  uint8_t*  p148_os_custom_version_GET_(Pack * src) {return p148_os_custom_version_GET(src, malloc(8 * sizeof(uint8_t)), 0);}
INLINER void p148_os_custom_version_SET(uint8_t*  src, int32_t pos, Pack * dst)
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  44, src_max = pos + 8; pos < src_max; pos++, BYTE += 1)
        set_bytes((src[pos]), 1, data,  BYTE);
}
INLINER e_MAV_PROTOCOL_CAPABILITY p148_capabilities_GET(Pack * src)//bitmask of capabilities (see MAV_PROTOCOL_CAPABILITY enum
{
    uint8_t * data = src->data;
    switch(get_bits(data, 416, 5))
    {
        case 0:
            return e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT;
        case 1:
            return e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT;
        case 2:
            return e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_MISSION_INT;
        case 3:
            return e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_COMMAND_INT;
        case 4:
            return e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_PARAM_UNION;
        case 5:
            return e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_FTP;
        case 6:
            return e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET;
        case 7:
            return e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED;
        case 8:
            return e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT;
        case 9:
            return e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_TERRAIN;
        case 10:
            return e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_SET_ACTUATOR_TARGET;
        case 11:
            return e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION;
        case 12:
            return e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION;
        case 13:
            return e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_MAVLINK2;
        case 14:
            return e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_MISSION_FENCE;
        case 15:
            return e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_MISSION_RALLY;
        case 16:
            return e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_FLIGHT_INFORMATION;
        default: ;//assert(false);//("Unknown enum ID " + id);
    }
}
INLINER void p148_capabilities_SET(e_MAV_PROTOCOL_CAPABILITY  src, Pack * dst)//bitmask of capabilities (see MAV_PROTOCOL_CAPABILITY enum
{
    uint8_t * data = dst->data;
    int32_t id;
    switch(src)
    {
        case e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT:
            id = 0;
            break;
        case e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT:
            id = 1;
            break;
        case e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_MISSION_INT:
            id = 2;
            break;
        case e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_COMMAND_INT:
            id = 3;
            break;
        case e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_PARAM_UNION:
            id = 4;
            break;
        case e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_FTP:
            id = 5;
            break;
        case e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET:
            id = 6;
            break;
        case e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED:
            id = 7;
            break;
        case e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT:
            id = 8;
            break;
        case e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_TERRAIN:
            id = 9;
            break;
        case e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_SET_ACTUATOR_TARGET:
            id = 10;
            break;
        case e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION:
            id = 11;
            break;
        case e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION:
            id = 12;
            break;
        case e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_MAVLINK2:
            id = 13;
            break;
        case e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_MISSION_FENCE:
            id = 14;
            break;
        case e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_MISSION_RALLY:
            id = 15;
            break;
        case e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_FLIGHT_INFORMATION:
            id = 16;
            break;
        default: ;// assert(false);//("Unknown enum" + id);
    }
    set_bits(id, 5, data, 416);
}
INLINER uint8_t* p148_uid2_GET(Bounds_Inside * src, uint8_t*  dst, int32_t pos)
{
    uint8_t * data = src->base.pack->data;
    for(int32_t BYTE = src->BYTE, dst_max = pos + 18; pos < dst_max ; pos++,  BYTE += 1)
        dst[pos] = ((get_bytes(data,  BYTE, 1)));
    return dst;
}
INLINER int32_t p148_uid2_LEN()
{
    return 18;
}
INLINER uint8_t*  p148_uid2_TRY(Bounds_Inside * src)
{
    if(src->base.field_bit !=  421 && !try_visit_field(src, 421)) return NULL;
    uint8_t * data = src->base.pack->data;
    return p148_uid2_GET(src, malloc(18 * sizeof(uint8_t)), 0);
}
INLINER void p148_uid2_SET(uint8_t*  src, int32_t pos, Bounds_Inside * dst)
{
    if(dst->base.field_bit != 421)insert_field(dst, 421, 0);
    uint8_t * data = dst->base.pack->data;
    for(int32_t BYTE =  dst->BYTE, src_max = pos + 18; pos < src_max; pos++, BYTE += 1)
        set_bytes((src[pos]), 1, data,  BYTE);
}
INLINER uint64_t p149_time_usec_GET(Pack * src)//Timestamp (micros since boot or Unix epoch
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 8)));
}
INLINER void p149_time_usec_SET(uint64_t  src, Pack * dst)//Timestamp (micros since boot or Unix epoch
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  0);
}
INLINER uint8_t p149_target_num_GET(Pack * src)//The ID of the target if multiple targets are presen
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  8, 1)));
}
INLINER void p149_target_num_SET(uint8_t  src, Pack * dst)//The ID of the target if multiple targets are presen
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  8);
}
INLINER float p149_angle_x_GET(Pack * src)//X-axis angular offset (in radians) of the target from the center of the imag
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  9, 4)));
}
INLINER void p149_angle_x_SET(float  src, Pack * dst)//X-axis angular offset (in radians) of the target from the center of the imag
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  9);
}
INLINER float p149_angle_y_GET(Pack * src)//Y-axis angular offset (in radians) of the target from the center of the imag
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  13, 4)));
}
INLINER void p149_angle_y_SET(float  src, Pack * dst)//Y-axis angular offset (in radians) of the target from the center of the imag
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  13);
}
INLINER float p149_distance_GET(Pack * src)//Distance to the target from the vehicle in meter
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  17, 4)));
}
INLINER void p149_distance_SET(float  src, Pack * dst)//Distance to the target from the vehicle in meter
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  17);
}
INLINER float p149_size_x_GET(Pack * src)//Size in radians of target along x-axi
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  21, 4)));
}
INLINER void p149_size_x_SET(float  src, Pack * dst)//Size in radians of target along x-axi
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  21);
}
INLINER float p149_size_y_GET(Pack * src)//Size in radians of target along y-axi
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  25, 4)));
}
INLINER void p149_size_y_SET(float  src, Pack * dst)//Size in radians of target along y-axi
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  25);
}
INLINER e_MAV_FRAME p149_frame_GET(Pack * src)//MAV_FRAME enum specifying the whether the following feilds are earth-frame, body-frame, etc
{
    uint8_t * data = src->data;
    return  0 + (int)get_bits(data, 232, 4);
}
INLINER void p149_frame_SET(e_MAV_FRAME  src, Pack * dst)//MAV_FRAME enum specifying the whether the following feilds are earth-frame, body-frame, etc
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 4, data, 232);
}
INLINER e_LANDING_TARGET_TYPE p149_type_GET(Pack * src)//LANDING_TARGET_TYPE enum specifying the type of landing targe
{
    uint8_t * data = src->data;
    return  0 + (int)get_bits(data, 236, 3);
}
INLINER void p149_type_SET(e_LANDING_TARGET_TYPE  src, Pack * dst)//LANDING_TARGET_TYPE enum specifying the type of landing targe
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 3, data, 236);
}
INLINER float  p149_x_TRY(Bounds_Inside * src)
{
    if(src->base.field_bit !=  239 && !try_visit_field(src, 239)) return 0;
    uint8_t * data = src->base.pack->data;
    return (intBitsToFloat(get_bytes(data,  src->BYTE, 4)));
}
INLINER void p149_x_SET(float  src, Bounds_Inside * dst)//X Position of the landing target on MAV_FRAM
{
    if(dst->base.field_bit != 239)insert_field(dst, 239, 0);
    uint8_t * data = dst->base.pack->data;
    set_bytes(floatToIntBits(src), 4, data,  dst->BYTE);
}
INLINER float  p149_y_TRY(Bounds_Inside * src)
{
    if(src->base.field_bit !=  240 && !try_visit_field(src, 240)) return 0;
    uint8_t * data = src->base.pack->data;
    return (intBitsToFloat(get_bytes(data,  src->BYTE, 4)));
}
INLINER void p149_y_SET(float  src, Bounds_Inside * dst)//Y Position of the landing target on MAV_FRAM
{
    if(dst->base.field_bit != 240)insert_field(dst, 240, 0);
    uint8_t * data = dst->base.pack->data;
    set_bytes(floatToIntBits(src), 4, data,  dst->BYTE);
}
INLINER float  p149_z_TRY(Bounds_Inside * src)
{
    if(src->base.field_bit !=  241 && !try_visit_field(src, 241)) return 0;
    uint8_t * data = src->base.pack->data;
    return (intBitsToFloat(get_bytes(data,  src->BYTE, 4)));
}
INLINER void p149_z_SET(float  src, Bounds_Inside * dst)//Z Position of the landing target on MAV_FRAM
{
    if(dst->base.field_bit != 241)insert_field(dst, 241, 0);
    uint8_t * data = dst->base.pack->data;
    set_bytes(floatToIntBits(src), 4, data,  dst->BYTE);
}
INLINER float* p149_q_GET(Bounds_Inside * src, float*  dst, int32_t pos) //Quaternion of landing target orientation (w, x, y, z order, zero-rotation is 1, 0, 0, 0
{
    uint8_t * data = src->base.pack->data;
    for(int32_t BYTE = src->BYTE, dst_max = pos + 4; pos < dst_max ; pos++,  BYTE += 4)
        dst[pos] = (intBitsToFloat(get_bytes(data,  BYTE, 4)));
    return dst;
}
INLINER int32_t p149_q_LEN()
{
    return 4;
}
INLINER float*  p149_q_TRY(Bounds_Inside * src)
{
    if(src->base.field_bit !=  242 && !try_visit_field(src, 242)) return NULL;
    uint8_t * data = src->base.pack->data;
    return p149_q_GET(src, malloc(4 * sizeof(float)), 0);
}
INLINER void p149_q_SET(float*  src, int32_t pos, Bounds_Inside * dst) //Quaternion of landing target orientation (w, x, y, z order, zero-rotation is 1, 0, 0, 0
{
    if(dst->base.field_bit != 242)insert_field(dst, 242, 0);
    uint8_t * data = dst->base.pack->data;
    for(int32_t BYTE =  dst->BYTE, src_max = pos + 4; pos < src_max; pos++, BYTE += 4)
        set_bytes(floatToIntBits(src[pos]), 4, data,  BYTE);
}
INLINER uint8_t  p149_position_valid_TRY(Bounds_Inside * src)
{
    if(src->base.field_bit !=  243 && !try_visit_field(src, 243)) return 0;
    uint8_t * data = src->base.pack->data;
    return ((get_bytes(data,  src->BYTE, 1)));
}
INLINER void p149_position_valid_SET(uint8_t  src, Bounds_Inside * dst)
{
    if(dst->base.field_bit != 243)insert_field(dst, 243, 0);
    uint8_t * data = dst->base.pack->data;
    set_bytes((src), 1, data,  dst->BYTE);
}
INLINER float p201_adc121_vspb_volt_GET(Pack * src)//Power board voltage sensor reading in volt
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  0, 4)));
}
INLINER void p201_adc121_vspb_volt_SET(float  src, Pack * dst)//Power board voltage sensor reading in volt
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  0);
}
INLINER float p201_adc121_cspb_amp_GET(Pack * src)//Power board current sensor reading in amp
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  4, 4)));
}
INLINER void p201_adc121_cspb_amp_SET(float  src, Pack * dst)//Power board current sensor reading in amp
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  4);
}
INLINER float p201_adc121_cs1_amp_GET(Pack * src)//Board current sensor 1 reading in amp
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  8, 4)));
}
INLINER void p201_adc121_cs1_amp_SET(float  src, Pack * dst)//Board current sensor 1 reading in amp
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  8);
}
INLINER float p201_adc121_cs2_amp_GET(Pack * src)//Board current sensor 2 reading in amp
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  12, 4)));
}
INLINER void p201_adc121_cs2_amp_SET(float  src, Pack * dst)//Board current sensor 2 reading in amp
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  12);
}
INLINER uint16_t p202_mppt1_pwm_GET(Pack * src)//MPPT1 pwm
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER void p202_mppt1_pwm_SET(uint16_t  src, Pack * dst)//MPPT1 pwm
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER uint16_t p202_mppt2_pwm_GET(Pack * src)//MPPT2 pwm
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 2)));
}
INLINER void p202_mppt2_pwm_SET(uint16_t  src, Pack * dst)//MPPT2 pwm
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  2);
}
INLINER uint16_t p202_mppt3_pwm_GET(Pack * src)//MPPT3 pwm
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  4, 2)));
}
INLINER void p202_mppt3_pwm_SET(uint16_t  src, Pack * dst)//MPPT3 pwm
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  4);
}
INLINER uint64_t p202_mppt_timestamp_GET(Pack * src)//MPPT last timestamp
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  6, 8)));
}
INLINER void p202_mppt_timestamp_SET(uint64_t  src, Pack * dst)//MPPT last timestamp
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  6);
}
INLINER float p202_mppt1_volt_GET(Pack * src)//MPPT1 voltage
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  14, 4)));
}
INLINER void p202_mppt1_volt_SET(float  src, Pack * dst)//MPPT1 voltage
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  14);
}
INLINER float p202_mppt1_amp_GET(Pack * src)//MPPT1 current
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  18, 4)));
}
INLINER void p202_mppt1_amp_SET(float  src, Pack * dst)//MPPT1 current
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  18);
}
INLINER uint8_t p202_mppt1_status_GET(Pack * src)//MPPT1 status
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  22, 1)));
}
INLINER void p202_mppt1_status_SET(uint8_t  src, Pack * dst)//MPPT1 status
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  22);
}
INLINER float p202_mppt2_volt_GET(Pack * src)//MPPT2 voltage
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  23, 4)));
}
INLINER void p202_mppt2_volt_SET(float  src, Pack * dst)//MPPT2 voltage
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  23);
}
INLINER float p202_mppt2_amp_GET(Pack * src)//MPPT2 current
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  27, 4)));
}
INLINER void p202_mppt2_amp_SET(float  src, Pack * dst)//MPPT2 current
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  27);
}
INLINER uint8_t p202_mppt2_status_GET(Pack * src)//MPPT2 status
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  31, 1)));
}
INLINER void p202_mppt2_status_SET(uint8_t  src, Pack * dst)//MPPT2 status
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  31);
}
INLINER float p202_mppt3_volt_GET(Pack * src)//MPPT3 voltage
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  32, 4)));
}
INLINER void p202_mppt3_volt_SET(float  src, Pack * dst)//MPPT3 voltage
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  32);
}
INLINER float p202_mppt3_amp_GET(Pack * src)//MPPT3 current
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  36, 4)));
}
INLINER void p202_mppt3_amp_SET(float  src, Pack * dst)//MPPT3 current
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  36);
}
INLINER uint8_t p202_mppt3_status_GET(Pack * src)//MPPT3 status
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  40, 1)));
}
INLINER void p202_mppt3_status_SET(uint8_t  src, Pack * dst)//MPPT3 status
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  40);
}
INLINER uint64_t p203_timestamp_GET(Pack * src)//Timestam
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 8)));
}
INLINER void p203_timestamp_SET(uint64_t  src, Pack * dst)//Timestam
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  0);
}
INLINER uint8_t p203_aslctrl_mode_GET(Pack * src)//ASLCTRL control-mode (manual, stabilized, auto, etc...
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  8, 1)));
}
INLINER void p203_aslctrl_mode_SET(uint8_t  src, Pack * dst)//ASLCTRL control-mode (manual, stabilized, auto, etc...
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  8);
}
INLINER float p203_h_GET(Pack * src)//See sourcecode for a description of these values...
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  9, 4)));
}
INLINER void p203_h_SET(float  src, Pack * dst)//See sourcecode for a description of these values...
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  9);
}
INLINER float p203_hRef_GET(Pack * src)//nul
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  13, 4)));
}
INLINER void p203_hRef_SET(float  src, Pack * dst)//nul
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  13);
}
INLINER float p203_hRef_t_GET(Pack * src)//nul
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  17, 4)));
}
INLINER void p203_hRef_t_SET(float  src, Pack * dst)//nul
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  17);
}
INLINER float p203_PitchAngle_GET(Pack * src)//Pitch angle [deg
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  21, 4)));
}
INLINER void p203_PitchAngle_SET(float  src, Pack * dst)//Pitch angle [deg
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  21);
}
INLINER float p203_PitchAngleRef_GET(Pack * src)//Pitch angle reference[deg]
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  25, 4)));
}
INLINER void p203_PitchAngleRef_SET(float  src, Pack * dst)//Pitch angle reference[deg]
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  25);
}
INLINER float p203_q_GET(Pack * src)//nul
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  29, 4)));
}
INLINER void p203_q_SET(float  src, Pack * dst)//nul
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  29);
}
INLINER float p203_qRef_GET(Pack * src)//nul
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  33, 4)));
}
INLINER void p203_qRef_SET(float  src, Pack * dst)//nul
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  33);
}
INLINER float p203_uElev_GET(Pack * src)//nul
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  37, 4)));
}
INLINER void p203_uElev_SET(float  src, Pack * dst)//nul
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  37);
}
INLINER float p203_uThrot_GET(Pack * src)//nul
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  41, 4)));
}
INLINER void p203_uThrot_SET(float  src, Pack * dst)//nul
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  41);
}
INLINER float p203_uThrot2_GET(Pack * src)//nul
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  45, 4)));
}
INLINER void p203_uThrot2_SET(float  src, Pack * dst)//nul
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  45);
}
INLINER float p203_nZ_GET(Pack * src)//nul
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  49, 4)));
}
INLINER void p203_nZ_SET(float  src, Pack * dst)//nul
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  49);
}
INLINER float p203_AirspeedRef_GET(Pack * src)//Airspeed reference [m/s
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  53, 4)));
}
INLINER void p203_AirspeedRef_SET(float  src, Pack * dst)//Airspeed reference [m/s
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  53);
}
INLINER uint8_t p203_SpoilersEngaged_GET(Pack * src)//nul
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  57, 1)));
}
INLINER void p203_SpoilersEngaged_SET(uint8_t  src, Pack * dst)//nul
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  57);
}
INLINER float p203_YawAngle_GET(Pack * src)//Yaw angle [deg
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  58, 4)));
}
INLINER void p203_YawAngle_SET(float  src, Pack * dst)//Yaw angle [deg
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  58);
}
INLINER float p203_YawAngleRef_GET(Pack * src)//Yaw angle reference[deg
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  62, 4)));
}
INLINER void p203_YawAngleRef_SET(float  src, Pack * dst)//Yaw angle reference[deg
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  62);
}
INLINER float p203_RollAngle_GET(Pack * src)//Roll angle [deg
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  66, 4)));
}
INLINER void p203_RollAngle_SET(float  src, Pack * dst)//Roll angle [deg
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  66);
}
INLINER float p203_RollAngleRef_GET(Pack * src)//Roll angle reference[deg
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  70, 4)));
}
INLINER void p203_RollAngleRef_SET(float  src, Pack * dst)//Roll angle reference[deg
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  70);
}
INLINER float p203_p_GET(Pack * src)//nul
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  74, 4)));
}
INLINER void p203_p_SET(float  src, Pack * dst)//nul
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  74);
}
INLINER float p203_pRef_GET(Pack * src)//nul
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  78, 4)));
}
INLINER void p203_pRef_SET(float  src, Pack * dst)//nul
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  78);
}
INLINER float p203_r_GET(Pack * src)//nul
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  82, 4)));
}
INLINER void p203_r_SET(float  src, Pack * dst)//nul
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  82);
}
INLINER float p203_rRef_GET(Pack * src)//nul
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  86, 4)));
}
INLINER void p203_rRef_SET(float  src, Pack * dst)//nul
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  86);
}
INLINER float p203_uAil_GET(Pack * src)//nul
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  90, 4)));
}
INLINER void p203_uAil_SET(float  src, Pack * dst)//nul
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  90);
}
INLINER float p203_uRud_GET(Pack * src)//nul
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  94, 4)));
}
INLINER void p203_uRud_SET(float  src, Pack * dst)//nul
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  94);
}
INLINER uint32_t p204_i32_1_GET(Pack * src)//Debug dat
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 4)));
}
INLINER void p204_i32_1_SET(uint32_t  src, Pack * dst)//Debug dat
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  0);
}
INLINER uint8_t p204_i8_1_GET(Pack * src)//Debug dat
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  4, 1)));
}
INLINER void p204_i8_1_SET(uint8_t  src, Pack * dst)//Debug dat
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  4);
}
INLINER uint8_t p204_i8_2_GET(Pack * src)//Debug dat
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  5, 1)));
}
INLINER void p204_i8_2_SET(uint8_t  src, Pack * dst)//Debug dat
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  5);
}
INLINER float p204_f_1_GET(Pack * src)//Debug data
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  6, 4)));
}
INLINER void p204_f_1_SET(float  src, Pack * dst)//Debug data
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  6);
}
INLINER float p204_f_2_GET(Pack * src)//Debug dat
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  10, 4)));
}
INLINER void p204_f_2_SET(float  src, Pack * dst)//Debug dat
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  10);
}
INLINER float p204_f_3_GET(Pack * src)//Debug dat
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  14, 4)));
}
INLINER void p204_f_3_SET(float  src, Pack * dst)//Debug dat
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  14);
}
INLINER float p204_f_4_GET(Pack * src)//Debug dat
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  18, 4)));
}
INLINER void p204_f_4_SET(float  src, Pack * dst)//Debug dat
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  18);
}
INLINER float p204_f_5_GET(Pack * src)//Debug dat
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  22, 4)));
}
INLINER void p204_f_5_SET(float  src, Pack * dst)//Debug dat
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  22);
}
INLINER float p204_f_6_GET(Pack * src)//Debug dat
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  26, 4)));
}
INLINER void p204_f_6_SET(float  src, Pack * dst)//Debug dat
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  26);
}
INLINER float p204_f_7_GET(Pack * src)//Debug dat
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  30, 4)));
}
INLINER void p204_f_7_SET(float  src, Pack * dst)//Debug dat
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  30);
}
INLINER float p204_f_8_GET(Pack * src)//Debug dat
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  34, 4)));
}
INLINER void p204_f_8_SET(float  src, Pack * dst)//Debug dat
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  34);
}
INLINER uint8_t p205_LED_status_GET(Pack * src)//Status of the position-indicator LED
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 1)));
}
INLINER void p205_LED_status_SET(uint8_t  src, Pack * dst)//Status of the position-indicator LED
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  0);
}
INLINER uint8_t p205_SATCOM_status_GET(Pack * src)//Status of the IRIDIUM satellite communication syste
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  1, 1)));
}
INLINER void p205_SATCOM_status_SET(uint8_t  src, Pack * dst)//Status of the IRIDIUM satellite communication syste
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  1);
}
INLINER uint8_t* p205_Servo_status_GET(Pack * src, uint8_t*  dst, int32_t pos) //Status vector for up to 8 servo
{
    uint8_t * data = src->data;
    for(int32_t BYTE = 2, dst_max = pos + 8; pos < dst_max ; pos++,  BYTE += 1)
        dst[pos] = ((get_bytes(data,  BYTE, 1)));
    return dst;
}

static const  uint32_t p205_Servo_status_LEN = 8; //return array length

INLINER  uint8_t*  p205_Servo_status_GET_(Pack * src) {return p205_Servo_status_GET(src, malloc(8 * sizeof(uint8_t)), 0);}
INLINER void p205_Servo_status_SET(uint8_t*  src, int32_t pos, Pack * dst) //Status vector for up to 8 servo
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  2, src_max = pos + 8; pos < src_max; pos++, BYTE += 1)
        set_bytes((src[pos]), 1, data,  BYTE);
}
INLINER float p205_Motor_rpm_GET(Pack * src)//Motor RPM
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  10, 4)));
}
INLINER void p205_Motor_rpm_SET(float  src, Pack * dst)//Motor RPM
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  10);
}
INLINER uint64_t p206_timestamp_GET(Pack * src)//Time since system start [us
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 8)));
}
INLINER void p206_timestamp_SET(uint64_t  src, Pack * dst)//Time since system start [us
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  0);
}
INLINER float p206_Windspeed_GET(Pack * src)//Magnitude of wind velocity (in lateral inertial plane) [m/s
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  8, 4)));
}
INLINER void p206_Windspeed_SET(float  src, Pack * dst)//Magnitude of wind velocity (in lateral inertial plane) [m/s
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  8);
}
INLINER float p206_WindDir_GET(Pack * src)//Wind heading angle from North [rad
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  12, 4)));
}
INLINER void p206_WindDir_SET(float  src, Pack * dst)//Wind heading angle from North [rad
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  12);
}
INLINER float p206_WindZ_GET(Pack * src)//Z (Down) component of inertial wind velocity [m/s
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  16, 4)));
}
INLINER void p206_WindZ_SET(float  src, Pack * dst)//Z (Down) component of inertial wind velocity [m/s
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  16);
}
INLINER float p206_Airspeed_GET(Pack * src)//Magnitude of air velocity [m/s
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  20, 4)));
}
INLINER void p206_Airspeed_SET(float  src, Pack * dst)//Magnitude of air velocity [m/s
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  20);
}
INLINER float p206_beta_GET(Pack * src)//Sideslip angle [rad
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  24, 4)));
}
INLINER void p206_beta_SET(float  src, Pack * dst)//Sideslip angle [rad
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  24);
}
INLINER float p206_alpha_GET(Pack * src)//Angle of attack [rad
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  28, 4)));
}
INLINER void p206_alpha_SET(float  src, Pack * dst)//Angle of attack [rad
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  28);
}
INLINER uint64_t p207_timestamp_GET(Pack * src)//Time since system start [us
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 8)));
}
INLINER void p207_timestamp_SET(uint64_t  src, Pack * dst)//Time since system start [us
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  0);
}
INLINER float p207_uElev_GET(Pack * src)//Elevator command [~
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  8, 4)));
}
INLINER void p207_uElev_SET(float  src, Pack * dst)//Elevator command [~
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  8);
}
INLINER float p207_uThrot_GET(Pack * src)//Throttle command [~
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  12, 4)));
}
INLINER void p207_uThrot_SET(float  src, Pack * dst)//Throttle command [~
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  12);
}
INLINER float p207_uThrot2_GET(Pack * src)//Throttle 2 command [~
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  16, 4)));
}
INLINER void p207_uThrot2_SET(float  src, Pack * dst)//Throttle 2 command [~
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  16);
}
INLINER float p207_uAilL_GET(Pack * src)//Left aileron command [~
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  20, 4)));
}
INLINER void p207_uAilL_SET(float  src, Pack * dst)//Left aileron command [~
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  20);
}
INLINER float p207_uAilR_GET(Pack * src)//Right aileron command [~
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  24, 4)));
}
INLINER void p207_uAilR_SET(float  src, Pack * dst)//Right aileron command [~
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  24);
}
INLINER float p207_uRud_GET(Pack * src)//Rudder command [~
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  28, 4)));
}
INLINER void p207_uRud_SET(float  src, Pack * dst)//Rudder command [~
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  28);
}
INLINER uint8_t p207_obctrl_status_GET(Pack * src)//Off-board computer statu
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  32, 1)));
}
INLINER void p207_obctrl_status_SET(uint8_t  src, Pack * dst)//Off-board computer statu
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  32);
}
INLINER float p208_TempAmbient_GET(Pack * src)//Ambient temperature [degrees Celsius
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  0, 4)));
}
INLINER void p208_TempAmbient_SET(float  src, Pack * dst)//Ambient temperature [degrees Celsius
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  0);
}
INLINER float p208_Humidity_GET(Pack * src)//Relative humidity [%
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  4, 4)));
}
INLINER void p208_Humidity_SET(float  src, Pack * dst)//Relative humidity [%
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  4);
}
INLINER uint16_t p209_voltage_GET(Pack * src)//Battery pack voltage in [mV
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER void p209_voltage_SET(uint16_t  src, Pack * dst)//Battery pack voltage in [mV
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER uint16_t p209_batterystatus_GET(Pack * src)//Battery monitor status report bits in He
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 2)));
}
INLINER void p209_batterystatus_SET(uint16_t  src, Pack * dst)//Battery monitor status report bits in He
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  2);
}
INLINER uint16_t p209_serialnumber_GET(Pack * src)//Battery monitor serial number in He
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  4, 2)));
}
INLINER void p209_serialnumber_SET(uint16_t  src, Pack * dst)//Battery monitor serial number in He
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  4);
}
INLINER uint16_t p209_hostfetcontrol_GET(Pack * src)//Battery monitor sensor host FET control in He
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  6, 2)));
}
INLINER void p209_hostfetcontrol_SET(uint16_t  src, Pack * dst)//Battery monitor sensor host FET control in He
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  6);
}
INLINER uint16_t p209_cellvoltage1_GET(Pack * src)//Battery pack cell 1 voltage in [mV
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  8, 2)));
}
INLINER void p209_cellvoltage1_SET(uint16_t  src, Pack * dst)//Battery pack cell 1 voltage in [mV
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  8);
}
INLINER uint16_t p209_cellvoltage2_GET(Pack * src)//Battery pack cell 2 voltage in [mV
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  10, 2)));
}
INLINER void p209_cellvoltage2_SET(uint16_t  src, Pack * dst)//Battery pack cell 2 voltage in [mV
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  10);
}
INLINER uint16_t p209_cellvoltage3_GET(Pack * src)//Battery pack cell 3 voltage in [mV
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  12, 2)));
}
INLINER void p209_cellvoltage3_SET(uint16_t  src, Pack * dst)//Battery pack cell 3 voltage in [mV
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  12);
}
INLINER uint16_t p209_cellvoltage4_GET(Pack * src)//Battery pack cell 4 voltage in [mV
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  14, 2)));
}
INLINER void p209_cellvoltage4_SET(uint16_t  src, Pack * dst)//Battery pack cell 4 voltage in [mV
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  14);
}
INLINER uint16_t p209_cellvoltage5_GET(Pack * src)//Battery pack cell 5 voltage in [mV
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  16, 2)));
}
INLINER void p209_cellvoltage5_SET(uint16_t  src, Pack * dst)//Battery pack cell 5 voltage in [mV
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  16);
}
INLINER uint16_t p209_cellvoltage6_GET(Pack * src)//Battery pack cell 6 voltage in [mV
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  18, 2)));
}
INLINER void p209_cellvoltage6_SET(uint16_t  src, Pack * dst)//Battery pack cell 6 voltage in [mV
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  18);
}
INLINER float p209_temperature_GET(Pack * src)//Battery pack temperature in [deg C
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  20, 4)));
}
INLINER void p209_temperature_SET(float  src, Pack * dst)//Battery pack temperature in [deg C
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  20);
}
INLINER int16_t p209_current_GET(Pack * src)//Battery pack current in [mA
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  24, 2)));
}
INLINER void p209_current_SET(int16_t  src, Pack * dst)//Battery pack current in [mA
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  24);
}
INLINER uint8_t p209_SoC_GET(Pack * src)//Battery pack state-of-charg
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  26, 1)));
}
INLINER void p209_SoC_SET(uint8_t  src, Pack * dst)//Battery pack state-of-charg
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  26);
}
INLINER uint64_t p210_timestamp_GET(Pack * src)//Timestamp [ms
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 8)));
}
INLINER void p210_timestamp_SET(uint64_t  src, Pack * dst)//Timestamp [ms
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  0);
}
INLINER uint64_t p210_timestampModeChanged_GET(Pack * src)//Timestamp since last mode change[ms
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  8, 8)));
}
INLINER void p210_timestampModeChanged_SET(uint64_t  src, Pack * dst)//Timestamp since last mode change[ms
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  8);
}
INLINER float p210_xW_GET(Pack * src)//Thermal core updraft strength [m/s
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  16, 4)));
}
INLINER void p210_xW_SET(float  src, Pack * dst)//Thermal core updraft strength [m/s
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  16);
}
INLINER float p210_xR_GET(Pack * src)//Thermal radius [m
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  20, 4)));
}
INLINER void p210_xR_SET(float  src, Pack * dst)//Thermal radius [m
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  20);
}
INLINER float p210_xLat_GET(Pack * src)//Thermal center latitude [deg
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  24, 4)));
}
INLINER void p210_xLat_SET(float  src, Pack * dst)//Thermal center latitude [deg
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  24);
}
INLINER float p210_xLon_GET(Pack * src)//Thermal center longitude [deg
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  28, 4)));
}
INLINER void p210_xLon_SET(float  src, Pack * dst)//Thermal center longitude [deg
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  28);
}
INLINER float p210_VarW_GET(Pack * src)//Variance
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  32, 4)));
}
INLINER void p210_VarW_SET(float  src, Pack * dst)//Variance
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  32);
}
INLINER float p210_VarR_GET(Pack * src)//Variance
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  36, 4)));
}
INLINER void p210_VarR_SET(float  src, Pack * dst)//Variance
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  36);
}
INLINER float p210_VarLat_GET(Pack * src)//Variance La
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  40, 4)));
}
INLINER void p210_VarLat_SET(float  src, Pack * dst)//Variance La
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  40);
}
INLINER float p210_VarLon_GET(Pack * src)//Variance Lon
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  44, 4)));
}
INLINER void p210_VarLon_SET(float  src, Pack * dst)//Variance Lon
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  44);
}
INLINER float p210_LoiterRadius_GET(Pack * src)//Suggested loiter radius [m
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  48, 4)));
}
INLINER void p210_LoiterRadius_SET(float  src, Pack * dst)//Suggested loiter radius [m
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  48);
}
INLINER float p210_LoiterDirection_GET(Pack * src)//Suggested loiter directio
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  52, 4)));
}
INLINER void p210_LoiterDirection_SET(float  src, Pack * dst)//Suggested loiter directio
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  52);
}
INLINER float p210_DistToSoarPoint_GET(Pack * src)//Distance to soar point [m
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  56, 4)));
}
INLINER void p210_DistToSoarPoint_SET(float  src, Pack * dst)//Distance to soar point [m
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  56);
}
INLINER float p210_vSinkExp_GET(Pack * src)//Expected sink rate at current airspeed, roll and throttle [m/s
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  60, 4)));
}
INLINER void p210_vSinkExp_SET(float  src, Pack * dst)//Expected sink rate at current airspeed, roll and throttle [m/s
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  60);
}
INLINER float p210_z1_LocalUpdraftSpeed_GET(Pack * src)//Measurement / updraft speed at current/local airplane position [m/s
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  64, 4)));
}
INLINER void p210_z1_LocalUpdraftSpeed_SET(float  src, Pack * dst)//Measurement / updraft speed at current/local airplane position [m/s
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  64);
}
INLINER float p210_z2_DeltaRoll_GET(Pack * src)//Measurement / roll angle tracking error [deg
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  68, 4)));
}
INLINER void p210_z2_DeltaRoll_SET(float  src, Pack * dst)//Measurement / roll angle tracking error [deg
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  68);
}
INLINER float p210_z1_exp_GET(Pack * src)//Expected measurement
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  72, 4)));
}
INLINER void p210_z1_exp_SET(float  src, Pack * dst)//Expected measurement
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  72);
}
INLINER float p210_z2_exp_GET(Pack * src)//Expected measurement
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  76, 4)));
}
INLINER void p210_z2_exp_SET(float  src, Pack * dst)//Expected measurement
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  76);
}
INLINER float p210_ThermalGSNorth_GET(Pack * src)//Thermal drift (from estimator prediction step only) [m/s
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  80, 4)));
}
INLINER void p210_ThermalGSNorth_SET(float  src, Pack * dst)//Thermal drift (from estimator prediction step only) [m/s
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  80);
}
INLINER float p210_ThermalGSEast_GET(Pack * src)//Thermal drift (from estimator prediction step only) [m/s
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  84, 4)));
}
INLINER void p210_ThermalGSEast_SET(float  src, Pack * dst)//Thermal drift (from estimator prediction step only) [m/s
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  84);
}
INLINER float p210_TSE_dot_GET(Pack * src)//Total specific energy change (filtered) [m/s
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  88, 4)));
}
INLINER void p210_TSE_dot_SET(float  src, Pack * dst)//Total specific energy change (filtered) [m/s
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  88);
}
INLINER float p210_DebugVar1_GET(Pack * src)//Debug variable
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  92, 4)));
}
INLINER void p210_DebugVar1_SET(float  src, Pack * dst)//Debug variable
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  92);
}
INLINER float p210_DebugVar2_GET(Pack * src)//Debug variable
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  96, 4)));
}
INLINER void p210_DebugVar2_SET(float  src, Pack * dst)//Debug variable
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  96);
}
INLINER uint8_t p210_ControlMode_GET(Pack * src)//Control Mode [-
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  100, 1)));
}
INLINER void p210_ControlMode_SET(uint8_t  src, Pack * dst)//Control Mode [-
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  100);
}
INLINER uint8_t p210_valid_GET(Pack * src)//Data valid [-
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  101, 1)));
}
INLINER void p210_valid_SET(uint8_t  src, Pack * dst)//Data valid [-
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  101);
}
INLINER uint16_t p211_free_space_GET(Pack * src)//Free space available in recordings directory in [Gb] * 1e
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER void p211_free_space_SET(uint16_t  src, Pack * dst)//Free space available in recordings directory in [Gb] * 1e
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER uint64_t p211_timestamp_GET(Pack * src)//Timestamp in linuxtime [ms] (since 1.1.1970
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 8)));
}
INLINER void p211_timestamp_SET(uint64_t  src, Pack * dst)//Timestamp in linuxtime [ms] (since 1.1.1970
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  2);
}
INLINER uint8_t p211_visensor_rate_1_GET(Pack * src)//Rate of ROS topic
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  10, 1)));
}
INLINER void p211_visensor_rate_1_SET(uint8_t  src, Pack * dst)//Rate of ROS topic
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  10);
}
INLINER uint8_t p211_visensor_rate_2_GET(Pack * src)//Rate of ROS topic
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  11, 1)));
}
INLINER void p211_visensor_rate_2_SET(uint8_t  src, Pack * dst)//Rate of ROS topic
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  11);
}
INLINER uint8_t p211_visensor_rate_3_GET(Pack * src)//Rate of ROS topic
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  12, 1)));
}
INLINER void p211_visensor_rate_3_SET(uint8_t  src, Pack * dst)//Rate of ROS topic
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  12);
}
INLINER uint8_t p211_visensor_rate_4_GET(Pack * src)//Rate of ROS topic
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  13, 1)));
}
INLINER void p211_visensor_rate_4_SET(uint8_t  src, Pack * dst)//Rate of ROS topic
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  13);
}
INLINER uint8_t p211_recording_nodes_count_GET(Pack * src)//Number of recording node
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  14, 1)));
}
INLINER void p211_recording_nodes_count_SET(uint8_t  src, Pack * dst)//Number of recording node
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  14);
}
INLINER uint8_t p211_cpu_temp_GET(Pack * src)//Temperature of sensorpod CPU in [deg C
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  15, 1)));
}
INLINER void p211_cpu_temp_SET(uint8_t  src, Pack * dst)//Temperature of sensorpod CPU in [deg C
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  15);
}
INLINER uint64_t p212_timestamp_GET(Pack * src)//Timestam
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 8)));
}
INLINER void p212_timestamp_SET(uint64_t  src, Pack * dst)//Timestam
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  0);
}
INLINER uint8_t p212_pwr_brd_status_GET(Pack * src)//Power board status registe
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  8, 1)));
}
INLINER void p212_pwr_brd_status_SET(uint8_t  src, Pack * dst)//Power board status registe
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  8);
}
INLINER uint8_t p212_pwr_brd_led_status_GET(Pack * src)//Power board leds statu
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  9, 1)));
}
INLINER void p212_pwr_brd_led_status_SET(uint8_t  src, Pack * dst)//Power board leds statu
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  9);
}
INLINER float p212_pwr_brd_system_volt_GET(Pack * src)//Power board system voltag
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  10, 4)));
}
INLINER void p212_pwr_brd_system_volt_SET(float  src, Pack * dst)//Power board system voltag
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  10);
}
INLINER float p212_pwr_brd_servo_volt_GET(Pack * src)//Power board servo voltag
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  14, 4)));
}
INLINER void p212_pwr_brd_servo_volt_SET(float  src, Pack * dst)//Power board servo voltag
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  14);
}
INLINER float p212_pwr_brd_mot_l_amp_GET(Pack * src)//Power board left motor current senso
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  18, 4)));
}
INLINER void p212_pwr_brd_mot_l_amp_SET(float  src, Pack * dst)//Power board left motor current senso
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  18);
}
INLINER float p212_pwr_brd_mot_r_amp_GET(Pack * src)//Power board right motor current senso
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  22, 4)));
}
INLINER void p212_pwr_brd_mot_r_amp_SET(float  src, Pack * dst)//Power board right motor current senso
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  22);
}
INLINER float p212_pwr_brd_servo_1_amp_GET(Pack * src)//Power board servo1 current senso
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  26, 4)));
}
INLINER void p212_pwr_brd_servo_1_amp_SET(float  src, Pack * dst)//Power board servo1 current senso
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  26);
}
INLINER float p212_pwr_brd_servo_2_amp_GET(Pack * src)//Power board servo1 current senso
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  30, 4)));
}
INLINER void p212_pwr_brd_servo_2_amp_SET(float  src, Pack * dst)//Power board servo1 current senso
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  30);
}
INLINER float p212_pwr_brd_servo_3_amp_GET(Pack * src)//Power board servo1 current senso
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  34, 4)));
}
INLINER void p212_pwr_brd_servo_3_amp_SET(float  src, Pack * dst)//Power board servo1 current senso
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  34);
}
INLINER float p212_pwr_brd_servo_4_amp_GET(Pack * src)//Power board servo1 current senso
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  38, 4)));
}
INLINER void p212_pwr_brd_servo_4_amp_SET(float  src, Pack * dst)//Power board servo1 current senso
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  38);
}
INLINER float p212_pwr_brd_aux_amp_GET(Pack * src)//Power board aux current senso
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  42, 4)));
}
INLINER void p212_pwr_brd_aux_amp_SET(float  src, Pack * dst)//Power board aux current senso
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  42);
}
INLINER uint64_t p230_time_usec_GET(Pack * src)//Timestamp (micros since boot or Unix epoch
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 8)));
}
INLINER void p230_time_usec_SET(uint64_t  src, Pack * dst)//Timestamp (micros since boot or Unix epoch
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  0);
}
INLINER float p230_vel_ratio_GET(Pack * src)//Velocity innovation test rati
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  8, 4)));
}
INLINER void p230_vel_ratio_SET(float  src, Pack * dst)//Velocity innovation test rati
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  8);
}
INLINER float p230_pos_horiz_ratio_GET(Pack * src)//Horizontal position innovation test rati
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  12, 4)));
}
INLINER void p230_pos_horiz_ratio_SET(float  src, Pack * dst)//Horizontal position innovation test rati
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  12);
}
INLINER float p230_pos_vert_ratio_GET(Pack * src)//Vertical position innovation test rati
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  16, 4)));
}
INLINER void p230_pos_vert_ratio_SET(float  src, Pack * dst)//Vertical position innovation test rati
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  16);
}
INLINER float p230_mag_ratio_GET(Pack * src)//Magnetometer innovation test rati
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  20, 4)));
}
INLINER void p230_mag_ratio_SET(float  src, Pack * dst)//Magnetometer innovation test rati
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  20);
}
INLINER float p230_hagl_ratio_GET(Pack * src)//Height above terrain innovation test rati
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  24, 4)));
}
INLINER void p230_hagl_ratio_SET(float  src, Pack * dst)//Height above terrain innovation test rati
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  24);
}
INLINER float p230_tas_ratio_GET(Pack * src)//True airspeed innovation test rati
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  28, 4)));
}
INLINER void p230_tas_ratio_SET(float  src, Pack * dst)//True airspeed innovation test rati
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  28);
}
INLINER float p230_pos_horiz_accuracy_GET(Pack * src)//Horizontal position 1-STD accuracy relative to the EKF local origin (m
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  32, 4)));
}
INLINER void p230_pos_horiz_accuracy_SET(float  src, Pack * dst)//Horizontal position 1-STD accuracy relative to the EKF local origin (m
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  32);
}
INLINER float p230_pos_vert_accuracy_GET(Pack * src)//Vertical position 1-STD accuracy relative to the EKF local origin (m
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  36, 4)));
}
INLINER void p230_pos_vert_accuracy_SET(float  src, Pack * dst)//Vertical position 1-STD accuracy relative to the EKF local origin (m
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  36);
}
INLINER e_ESTIMATOR_STATUS_FLAGS p230_flags_GET(Pack * src)//Integer bitmask indicating which EKF outputs are valid. See definition for ESTIMATOR_STATUS_FLAGS
{
    uint8_t * data = src->data;
    switch(get_bits(data, 320, 4))
    {
        case 0:
            return e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_ATTITUDE;
        case 1:
            return e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_VELOCITY_HORIZ;
        case 2:
            return e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_VELOCITY_VERT;
        case 3:
            return e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_POS_HORIZ_REL;
        case 4:
            return e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_POS_HORIZ_ABS;
        case 5:
            return e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_POS_VERT_ABS;
        case 6:
            return e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_POS_VERT_AGL;
        case 7:
            return e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_CONST_POS_MODE;
        case 8:
            return e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_PRED_POS_HORIZ_REL;
        case 9:
            return e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_PRED_POS_HORIZ_ABS;
        case 10:
            return e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_GPS_GLITCH;
        default: ;//assert(false);//("Unknown enum ID " + id);
    }
}
INLINER void p230_flags_SET(e_ESTIMATOR_STATUS_FLAGS  src, Pack * dst)//Integer bitmask indicating which EKF outputs are valid. See definition for ESTIMATOR_STATUS_FLAGS
{
    uint8_t * data = dst->data;
    int32_t id;
    switch(src)
    {
        case e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_ATTITUDE:
            id = 0;
            break;
        case e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_VELOCITY_HORIZ:
            id = 1;
            break;
        case e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_VELOCITY_VERT:
            id = 2;
            break;
        case e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_POS_HORIZ_REL:
            id = 3;
            break;
        case e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_POS_HORIZ_ABS:
            id = 4;
            break;
        case e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_POS_VERT_ABS:
            id = 5;
            break;
        case e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_POS_VERT_AGL:
            id = 6;
            break;
        case e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_CONST_POS_MODE:
            id = 7;
            break;
        case e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_PRED_POS_HORIZ_REL:
            id = 8;
            break;
        case e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_PRED_POS_HORIZ_ABS:
            id = 9;
            break;
        case e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_GPS_GLITCH:
            id = 10;
            break;
        default: ;// assert(false);//("Unknown enum" + id);
    }
    set_bits(id, 4, data, 320);
}
INLINER uint64_t p231_time_usec_GET(Pack * src)//Timestamp (micros since boot or Unix epoch
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 8)));
}
INLINER void p231_time_usec_SET(uint64_t  src, Pack * dst)//Timestamp (micros since boot or Unix epoch
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  0);
}
INLINER float p231_wind_x_GET(Pack * src)//Wind in X (NED) direction in m/
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  8, 4)));
}
INLINER void p231_wind_x_SET(float  src, Pack * dst)//Wind in X (NED) direction in m/
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  8);
}
INLINER float p231_wind_y_GET(Pack * src)//Wind in Y (NED) direction in m/
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  12, 4)));
}
INLINER void p231_wind_y_SET(float  src, Pack * dst)//Wind in Y (NED) direction in m/
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  12);
}
INLINER float p231_wind_z_GET(Pack * src)//Wind in Z (NED) direction in m/
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  16, 4)));
}
INLINER void p231_wind_z_SET(float  src, Pack * dst)//Wind in Z (NED) direction in m/
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  16);
}
INLINER float p231_var_horiz_GET(Pack * src)//Variability of the wind in XY. RMS of a 1 Hz lowpassed wind estimate
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  20, 4)));
}
INLINER void p231_var_horiz_SET(float  src, Pack * dst)//Variability of the wind in XY. RMS of a 1 Hz lowpassed wind estimate
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  20);
}
INLINER float p231_var_vert_GET(Pack * src)//Variability of the wind in Z. RMS of a 1 Hz lowpassed wind estimate
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  24, 4)));
}
INLINER void p231_var_vert_SET(float  src, Pack * dst)//Variability of the wind in Z. RMS of a 1 Hz lowpassed wind estimate
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  24);
}
INLINER float p231_wind_alt_GET(Pack * src)//AMSL altitude (m) this measurement was taken a
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  28, 4)));
}
INLINER void p231_wind_alt_SET(float  src, Pack * dst)//AMSL altitude (m) this measurement was taken a
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  28);
}
INLINER float p231_horiz_accuracy_GET(Pack * src)//Horizontal speed 1-STD accurac
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  32, 4)));
}
INLINER void p231_horiz_accuracy_SET(float  src, Pack * dst)//Horizontal speed 1-STD accurac
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  32);
}
INLINER float p231_vert_accuracy_GET(Pack * src)//Vertical speed 1-STD accurac
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  36, 4)));
}
INLINER void p231_vert_accuracy_SET(float  src, Pack * dst)//Vertical speed 1-STD accurac
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  36);
}
INLINER uint16_t p232_time_week_GET(Pack * src)//GPS week numbe
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER void p232_time_week_SET(uint16_t  src, Pack * dst)//GPS week numbe
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER uint32_t p232_time_week_ms_GET(Pack * src)//GPS time (milliseconds from start of GPS week
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 4)));
}
INLINER void p232_time_week_ms_SET(uint32_t  src, Pack * dst)//GPS time (milliseconds from start of GPS week
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  2);
}
INLINER uint64_t p232_time_usec_GET(Pack * src)//Timestamp (micros since boot or Unix epoch
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  6, 8)));
}
INLINER void p232_time_usec_SET(uint64_t  src, Pack * dst)//Timestamp (micros since boot or Unix epoch
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  6);
}
INLINER uint8_t p232_gps_id_GET(Pack * src)//ID of the GPS for multiple GPS input
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  14, 1)));
}
INLINER void p232_gps_id_SET(uint8_t  src, Pack * dst)//ID of the GPS for multiple GPS input
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  14);
}
INLINER uint8_t p232_fix_type_GET(Pack * src)//0-1: no fix, 2: 2D fix, 3: 3D fix. 4: 3D with DGPS. 5: 3D with RT
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  15, 1)));
}
INLINER void p232_fix_type_SET(uint8_t  src, Pack * dst)//0-1: no fix, 2: 2D fix, 3: 3D fix. 4: 3D with DGPS. 5: 3D with RT
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  15);
}
INLINER int32_t p232_lat_GET(Pack * src)//Latitude (WGS84), in degrees * 1E
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  16, 4)));
}
INLINER void p232_lat_SET(int32_t  src, Pack * dst)//Latitude (WGS84), in degrees * 1E
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  16);
}
INLINER int32_t p232_lon_GET(Pack * src)//Longitude (WGS84), in degrees * 1E
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  20, 4)));
}
INLINER void p232_lon_SET(int32_t  src, Pack * dst)//Longitude (WGS84), in degrees * 1E
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  20);
}
INLINER float p232_alt_GET(Pack * src)//Altitude (AMSL, not WGS84), in m (positive for up
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  24, 4)));
}
INLINER void p232_alt_SET(float  src, Pack * dst)//Altitude (AMSL, not WGS84), in m (positive for up
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  24);
}
INLINER float p232_hdop_GET(Pack * src)//GPS HDOP horizontal dilution of position in
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  28, 4)));
}
INLINER void p232_hdop_SET(float  src, Pack * dst)//GPS HDOP horizontal dilution of position in
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  28);
}
INLINER float p232_vdop_GET(Pack * src)//GPS VDOP vertical dilution of position in
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  32, 4)));
}
INLINER void p232_vdop_SET(float  src, Pack * dst)//GPS VDOP vertical dilution of position in
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  32);
}
INLINER float p232_vn_GET(Pack * src)//GPS velocity in m/s in NORTH direction in earth-fixed NED fram
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  36, 4)));
}
INLINER void p232_vn_SET(float  src, Pack * dst)//GPS velocity in m/s in NORTH direction in earth-fixed NED fram
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  36);
}
INLINER float p232_ve_GET(Pack * src)//GPS velocity in m/s in EAST direction in earth-fixed NED fram
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  40, 4)));
}
INLINER void p232_ve_SET(float  src, Pack * dst)//GPS velocity in m/s in EAST direction in earth-fixed NED fram
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  40);
}
INLINER float p232_vd_GET(Pack * src)//GPS velocity in m/s in DOWN direction in earth-fixed NED fram
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  44, 4)));
}
INLINER void p232_vd_SET(float  src, Pack * dst)//GPS velocity in m/s in DOWN direction in earth-fixed NED fram
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  44);
}
INLINER float p232_speed_accuracy_GET(Pack * src)//GPS speed accuracy in m/
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  48, 4)));
}
INLINER void p232_speed_accuracy_SET(float  src, Pack * dst)//GPS speed accuracy in m/
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  48);
}
INLINER float p232_horiz_accuracy_GET(Pack * src)//GPS horizontal accuracy in
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  52, 4)));
}
INLINER void p232_horiz_accuracy_SET(float  src, Pack * dst)//GPS horizontal accuracy in
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  52);
}
INLINER float p232_vert_accuracy_GET(Pack * src)//GPS vertical accuracy in
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  56, 4)));
}
INLINER void p232_vert_accuracy_SET(float  src, Pack * dst)//GPS vertical accuracy in
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  56);
}
INLINER uint8_t p232_satellites_visible_GET(Pack * src)//Number of satellites visible
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  60, 1)));
}
INLINER void p232_satellites_visible_SET(uint8_t  src, Pack * dst)//Number of satellites visible
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  60);
}
INLINER e_GPS_INPUT_IGNORE_FLAGS p232_ignore_flags_GET(Pack * src)//Flags indicating which fields to ignore (see GPS_INPUT_IGNORE_FLAGS enum).  All other fields must be provide
{
    uint8_t * data = src->data;
    switch(get_bits(data, 488, 4))
    {
        case 0:
            return e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_ALT;
        case 1:
            return e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_HDOP;
        case 2:
            return e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_VDOP;
        case 3:
            return e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_VEL_HORIZ;
        case 4:
            return e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_VEL_VERT;
        case 5:
            return e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY;
        case 6:
            return e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY;
        case 7:
            return e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY;
        default: ;//assert(false);//("Unknown enum ID " + id);
    }
}
INLINER void p232_ignore_flags_SET(e_GPS_INPUT_IGNORE_FLAGS  src, Pack * dst)//Flags indicating which fields to ignore (see GPS_INPUT_IGNORE_FLAGS enum).  All other fields must be provide
{
    uint8_t * data = dst->data;
    int32_t id;
    switch(src)
    {
        case e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_ALT:
            id = 0;
            break;
        case e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_HDOP:
            id = 1;
            break;
        case e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_VDOP:
            id = 2;
            break;
        case e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_VEL_HORIZ:
            id = 3;
            break;
        case e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_VEL_VERT:
            id = 4;
            break;
        case e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY:
            id = 5;
            break;
        case e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY:
            id = 6;
            break;
        case e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY:
            id = 7;
            break;
        default: ;// assert(false);//("Unknown enum" + id);
    }
    set_bits(id, 4, data, 488);
}
INLINER uint8_t p233_flags_GET(Pack * src)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 1)));
}
INLINER void p233_flags_SET(uint8_t  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  0);
}
INLINER uint8_t p233_len_GET(Pack * src)//data lengt
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  1, 1)));
}
INLINER void p233_len_SET(uint8_t  src, Pack * dst)//data lengt
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  1);
}
INLINER uint8_t* p233_data__GET(Pack * src, uint8_t*  dst, int32_t pos) //RTCM message (may be fragmented
{
    uint8_t * data = src->data;
    for(int32_t BYTE = 2, dst_max = pos + 180; pos < dst_max ; pos++,  BYTE += 1)
        dst[pos] = ((get_bytes(data,  BYTE, 1)));
    return dst;
}

static const  uint32_t p233_data__LEN = 180; //return array length

INLINER  uint8_t*  p233_data__GET_(Pack * src) {return p233_data__GET(src, malloc(180 * sizeof(uint8_t)), 0);}
INLINER void p233_data__SET(uint8_t*  src, int32_t pos, Pack * dst) //RTCM message (may be fragmented
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  2, src_max = pos + 180; pos < src_max; pos++, BYTE += 1)
        set_bytes((src[pos]), 1, data,  BYTE);
}
INLINER uint16_t p234_heading_GET(Pack * src)//heading (centidegrees
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER void p234_heading_SET(uint16_t  src, Pack * dst)//heading (centidegrees
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER uint16_t p234_wp_distance_GET(Pack * src)//distance to target (meters
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 2)));
}
INLINER void p234_wp_distance_SET(uint16_t  src, Pack * dst)//distance to target (meters
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  2);
}
INLINER uint32_t p234_custom_mode_GET(Pack * src)//A bitfield for use for autopilot-specific flags
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  4, 4)));
}
INLINER void p234_custom_mode_SET(uint32_t  src, Pack * dst)//A bitfield for use for autopilot-specific flags
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  4);
}
INLINER int16_t p234_roll_GET(Pack * src)//roll (centidegrees
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  8, 2)));
}
INLINER void p234_roll_SET(int16_t  src, Pack * dst)//roll (centidegrees
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  8);
}
INLINER int16_t p234_pitch_GET(Pack * src)//pitch (centidegrees
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  10, 2)));
}
INLINER void p234_pitch_SET(int16_t  src, Pack * dst)//pitch (centidegrees
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  10);
}
INLINER int8_t p234_throttle_GET(Pack * src)//throttle (percentage
{
    uint8_t * data = src->data;
    return ((int8_t)(get_bytes(data,  12, 1)));
}
INLINER void p234_throttle_SET(int8_t  src, Pack * dst)//throttle (percentage
{
    uint8_t * data = dst->data;
    set_bytes((uint8_t)(src), 1, data,  12);
}
INLINER int16_t p234_heading_sp_GET(Pack * src)//heading setpoint (centidegrees
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  13, 2)));
}
INLINER void p234_heading_sp_SET(int16_t  src, Pack * dst)//heading setpoint (centidegrees
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  13);
}
INLINER int32_t p234_latitude_GET(Pack * src)//Latitude, expressed as degrees * 1E
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  15, 4)));
}
INLINER void p234_latitude_SET(int32_t  src, Pack * dst)//Latitude, expressed as degrees * 1E
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  15);
}
INLINER int32_t p234_longitude_GET(Pack * src)//Longitude, expressed as degrees * 1E
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  19, 4)));
}
INLINER void p234_longitude_SET(int32_t  src, Pack * dst)//Longitude, expressed as degrees * 1E
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  19);
}
INLINER int16_t p234_altitude_amsl_GET(Pack * src)//Altitude above mean sea level (meters
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  23, 2)));
}
INLINER void p234_altitude_amsl_SET(int16_t  src, Pack * dst)//Altitude above mean sea level (meters
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  23);
}
INLINER int16_t p234_altitude_sp_GET(Pack * src)//Altitude setpoint relative to the home position (meters
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  25, 2)));
}
INLINER void p234_altitude_sp_SET(int16_t  src, Pack * dst)//Altitude setpoint relative to the home position (meters
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  25);
}
INLINER uint8_t p234_airspeed_GET(Pack * src)//airspeed (m/s
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  27, 1)));
}
INLINER void p234_airspeed_SET(uint8_t  src, Pack * dst)//airspeed (m/s
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  27);
}
INLINER uint8_t p234_airspeed_sp_GET(Pack * src)//airspeed setpoint (m/s
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  28, 1)));
}
INLINER void p234_airspeed_sp_SET(uint8_t  src, Pack * dst)//airspeed setpoint (m/s
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  28);
}
INLINER uint8_t p234_groundspeed_GET(Pack * src)//groundspeed (m/s
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  29, 1)));
}
INLINER void p234_groundspeed_SET(uint8_t  src, Pack * dst)//groundspeed (m/s
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  29);
}
INLINER int8_t p234_climb_rate_GET(Pack * src)//climb rate (m/s
{
    uint8_t * data = src->data;
    return ((int8_t)(get_bytes(data,  30, 1)));
}
INLINER void p234_climb_rate_SET(int8_t  src, Pack * dst)//climb rate (m/s
{
    uint8_t * data = dst->data;
    set_bytes((uint8_t)(src), 1, data,  30);
}
INLINER uint8_t p234_gps_nsat_GET(Pack * src)//Number of satellites visible. If unknown, set to 25
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  31, 1)));
}
INLINER void p234_gps_nsat_SET(uint8_t  src, Pack * dst)//Number of satellites visible. If unknown, set to 25
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  31);
}
INLINER uint8_t p234_battery_remaining_GET(Pack * src)//Remaining battery (percentage
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  32, 1)));
}
INLINER void p234_battery_remaining_SET(uint8_t  src, Pack * dst)//Remaining battery (percentage
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  32);
}
INLINER int8_t p234_temperature_GET(Pack * src)//Autopilot temperature (degrees C
{
    uint8_t * data = src->data;
    return ((int8_t)(get_bytes(data,  33, 1)));
}
INLINER void p234_temperature_SET(int8_t  src, Pack * dst)//Autopilot temperature (degrees C
{
    uint8_t * data = dst->data;
    set_bytes((uint8_t)(src), 1, data,  33);
}
INLINER int8_t p234_temperature_air_GET(Pack * src)//Air temperature (degrees C) from airspeed senso
{
    uint8_t * data = src->data;
    return ((int8_t)(get_bytes(data,  34, 1)));
}
INLINER void p234_temperature_air_SET(int8_t  src, Pack * dst)//Air temperature (degrees C) from airspeed senso
{
    uint8_t * data = dst->data;
    set_bytes((uint8_t)(src), 1, data,  34);
}
INLINER uint8_t p234_failsafe_GET(Pack * src)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  35, 1)));
}
INLINER void p234_failsafe_SET(uint8_t  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  35);
}
INLINER uint8_t p234_wp_num_GET(Pack * src)//current waypoint numbe
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  36, 1)));
}
INLINER void p234_wp_num_SET(uint8_t  src, Pack * dst)//current waypoint numbe
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  36);
}
INLINER e_MAV_MODE_FLAG p234_base_mode_GET(Pack * src)//System mode bitfield, see MAV_MODE_FLAG ENUM in mavlink/include/mavlink_types.
{
    uint8_t * data = src->data;
    switch(get_bits(data, 296, 4))
    {
        case 0:
            return e_MAV_MODE_FLAG_MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
        case 1:
            return e_MAV_MODE_FLAG_MAV_MODE_FLAG_TEST_ENABLED;
        case 2:
            return e_MAV_MODE_FLAG_MAV_MODE_FLAG_AUTO_ENABLED;
        case 3:
            return e_MAV_MODE_FLAG_MAV_MODE_FLAG_GUIDED_ENABLED;
        case 4:
            return e_MAV_MODE_FLAG_MAV_MODE_FLAG_STABILIZE_ENABLED;
        case 5:
            return e_MAV_MODE_FLAG_MAV_MODE_FLAG_HIL_ENABLED;
        case 6:
            return e_MAV_MODE_FLAG_MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
        case 7:
            return e_MAV_MODE_FLAG_MAV_MODE_FLAG_SAFETY_ARMED;
        default: ;//assert(false);//("Unknown enum ID " + id);
    }
}
INLINER void p234_base_mode_SET(e_MAV_MODE_FLAG  src, Pack * dst)//System mode bitfield, see MAV_MODE_FLAG ENUM in mavlink/include/mavlink_types.
{
    uint8_t * data = dst->data;
    int32_t id;
    switch(src)
    {
        case e_MAV_MODE_FLAG_MAV_MODE_FLAG_CUSTOM_MODE_ENABLED:
            id = 0;
            break;
        case e_MAV_MODE_FLAG_MAV_MODE_FLAG_TEST_ENABLED:
            id = 1;
            break;
        case e_MAV_MODE_FLAG_MAV_MODE_FLAG_AUTO_ENABLED:
            id = 2;
            break;
        case e_MAV_MODE_FLAG_MAV_MODE_FLAG_GUIDED_ENABLED:
            id = 3;
            break;
        case e_MAV_MODE_FLAG_MAV_MODE_FLAG_STABILIZE_ENABLED:
            id = 4;
            break;
        case e_MAV_MODE_FLAG_MAV_MODE_FLAG_HIL_ENABLED:
            id = 5;
            break;
        case e_MAV_MODE_FLAG_MAV_MODE_FLAG_MANUAL_INPUT_ENABLED:
            id = 6;
            break;
        case e_MAV_MODE_FLAG_MAV_MODE_FLAG_SAFETY_ARMED:
            id = 7;
            break;
        default: ;// assert(false);//("Unknown enum" + id);
    }
    set_bits(id, 4, data, 296);
}
INLINER e_MAV_LANDED_STATE p234_landed_state_GET(Pack * src)//The landed state. Is set to MAV_LANDED_STATE_UNDEFINED if landed state is unknown
{
    uint8_t * data = src->data;
    return  0 + (int)get_bits(data, 300, 3);
}
INLINER void p234_landed_state_SET(e_MAV_LANDED_STATE  src, Pack * dst)//The landed state. Is set to MAV_LANDED_STATE_UNDEFINED if landed state is unknown
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 3, data, 300);
}
INLINER e_GPS_FIX_TYPE p234_gps_fix_type_GET(Pack * src)//See the GPS_FIX_TYPE enum
{
    uint8_t * data = src->data;
    return  0 + (int)get_bits(data, 303, 4);
}
INLINER void p234_gps_fix_type_SET(e_GPS_FIX_TYPE  src, Pack * dst)//See the GPS_FIX_TYPE enum
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 4, data, 303);
}
INLINER uint32_t p241_clipping_0_GET(Pack * src)//first accelerometer clipping coun
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 4)));
}
INLINER void p241_clipping_0_SET(uint32_t  src, Pack * dst)//first accelerometer clipping coun
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  0);
}
INLINER uint32_t p241_clipping_1_GET(Pack * src)//second accelerometer clipping coun
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  4, 4)));
}
INLINER void p241_clipping_1_SET(uint32_t  src, Pack * dst)//second accelerometer clipping coun
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  4);
}
INLINER uint32_t p241_clipping_2_GET(Pack * src)//third accelerometer clipping coun
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  8, 4)));
}
INLINER void p241_clipping_2_SET(uint32_t  src, Pack * dst)//third accelerometer clipping coun
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  8);
}
INLINER uint64_t p241_time_usec_GET(Pack * src)//Timestamp (micros since boot or Unix epoch
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  12, 8)));
}
INLINER void p241_time_usec_SET(uint64_t  src, Pack * dst)//Timestamp (micros since boot or Unix epoch
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  12);
}
INLINER float p241_vibration_x_GET(Pack * src)//Vibration levels on X-axi
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  20, 4)));
}
INLINER void p241_vibration_x_SET(float  src, Pack * dst)//Vibration levels on X-axi
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  20);
}
INLINER float p241_vibration_y_GET(Pack * src)//Vibration levels on Y-axi
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  24, 4)));
}
INLINER void p241_vibration_y_SET(float  src, Pack * dst)//Vibration levels on Y-axi
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  24);
}
INLINER float p241_vibration_z_GET(Pack * src)//Vibration levels on Z-axi
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  28, 4)));
}
INLINER void p241_vibration_z_SET(float  src, Pack * dst)//Vibration levels on Z-axi
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  28);
}
INLINER int32_t p242_latitude_GET(Pack * src)//Latitude (WGS84), in degrees * 1E
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  0, 4)));
}
INLINER void p242_latitude_SET(int32_t  src, Pack * dst)//Latitude (WGS84), in degrees * 1E
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  0);
}
INLINER int32_t p242_longitude_GET(Pack * src)//Longitude (WGS84, in degrees * 1E
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  4, 4)));
}
INLINER void p242_longitude_SET(int32_t  src, Pack * dst)//Longitude (WGS84, in degrees * 1E
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  4);
}
INLINER int32_t p242_altitude_GET(Pack * src)//Altitude (AMSL), in meters * 1000 (positive for up
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  8, 4)));
}
INLINER void p242_altitude_SET(int32_t  src, Pack * dst)//Altitude (AMSL), in meters * 1000 (positive for up
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  8);
}
INLINER float p242_x_GET(Pack * src)//Local X position of this position in the local coordinate fram
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  12, 4)));
}
INLINER void p242_x_SET(float  src, Pack * dst)//Local X position of this position in the local coordinate fram
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  12);
}
INLINER float p242_y_GET(Pack * src)//Local Y position of this position in the local coordinate fram
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  16, 4)));
}
INLINER void p242_y_SET(float  src, Pack * dst)//Local Y position of this position in the local coordinate fram
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  16);
}
INLINER float p242_z_GET(Pack * src)//Local Z position of this position in the local coordinate fram
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  20, 4)));
}
INLINER void p242_z_SET(float  src, Pack * dst)//Local Z position of this position in the local coordinate fram
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  20);
}
INLINER float* p242_q_GET(Pack * src, float*  dst, int32_t pos)
{
    uint8_t * data = src->data;
    for(int32_t BYTE = 24, dst_max = pos + 4; pos < dst_max ; pos++,  BYTE += 4)
        dst[pos] = (intBitsToFloat(get_bytes(data,  BYTE, 4)));
    return dst;
}

static const  uint32_t p242_q_LEN = 4; //return array length

INLINER  float*  p242_q_GET_(Pack * src) {return p242_q_GET(src, malloc(4 * sizeof(float)), 0);}
INLINER void p242_q_SET(float*  src, int32_t pos, Pack * dst)
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  24, src_max = pos + 4; pos < src_max; pos++, BYTE += 4)
        set_bytes(floatToIntBits(src[pos]), 4, data,  BYTE);
}
INLINER float p242_approach_x_GET(Pack * src)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  40, 4)));
}
INLINER void p242_approach_x_SET(float  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  40);
}
INLINER float p242_approach_y_GET(Pack * src)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  44, 4)));
}
INLINER void p242_approach_y_SET(float  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  44);
}
INLINER float p242_approach_z_GET(Pack * src)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  48, 4)));
}
INLINER void p242_approach_z_SET(float  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  48);
}
INLINER uint64_t  p242_time_usec_TRY(Bounds_Inside * src)
{
    if(src->base.field_bit !=  416 && !try_visit_field(src, 416)) return 0;
    uint8_t * data = src->base.pack->data;
    return ((get_bytes(data,  src->BYTE, 8)));
}
INLINER void p242_time_usec_SET(uint64_t  src, Bounds_Inside * dst)//Timestamp (microseconds since UNIX epoch or microseconds since system boot
{
    if(dst->base.field_bit != 416)insert_field(dst, 416, 0);
    uint8_t * data = dst->base.pack->data;
    set_bytes((src), 8, data,  dst->BYTE);
}
INLINER uint8_t p243_target_system_GET(Pack * src)//System ID
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 1)));
}
INLINER void p243_target_system_SET(uint8_t  src, Pack * dst)//System ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  0);
}
INLINER int32_t p243_latitude_GET(Pack * src)//Latitude (WGS84), in degrees * 1E
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  1, 4)));
}
INLINER void p243_latitude_SET(int32_t  src, Pack * dst)//Latitude (WGS84), in degrees * 1E
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  1);
}
INLINER int32_t p243_longitude_GET(Pack * src)//Longitude (WGS84, in degrees * 1E
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  5, 4)));
}
INLINER void p243_longitude_SET(int32_t  src, Pack * dst)//Longitude (WGS84, in degrees * 1E
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  5);
}
INLINER int32_t p243_altitude_GET(Pack * src)//Altitude (AMSL), in meters * 1000 (positive for up
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  9, 4)));
}
INLINER void p243_altitude_SET(int32_t  src, Pack * dst)//Altitude (AMSL), in meters * 1000 (positive for up
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  9);
}
INLINER float p243_x_GET(Pack * src)//Local X position of this position in the local coordinate fram
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  13, 4)));
}
INLINER void p243_x_SET(float  src, Pack * dst)//Local X position of this position in the local coordinate fram
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  13);
}
INLINER float p243_y_GET(Pack * src)//Local Y position of this position in the local coordinate fram
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  17, 4)));
}
INLINER void p243_y_SET(float  src, Pack * dst)//Local Y position of this position in the local coordinate fram
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  17);
}
INLINER float p243_z_GET(Pack * src)//Local Z position of this position in the local coordinate fram
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  21, 4)));
}
INLINER void p243_z_SET(float  src, Pack * dst)//Local Z position of this position in the local coordinate fram
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  21);
}
INLINER float* p243_q_GET(Pack * src, float*  dst, int32_t pos)
{
    uint8_t * data = src->data;
    for(int32_t BYTE = 25, dst_max = pos + 4; pos < dst_max ; pos++,  BYTE += 4)
        dst[pos] = (intBitsToFloat(get_bytes(data,  BYTE, 4)));
    return dst;
}

static const  uint32_t p243_q_LEN = 4; //return array length

INLINER  float*  p243_q_GET_(Pack * src) {return p243_q_GET(src, malloc(4 * sizeof(float)), 0);}
INLINER void p243_q_SET(float*  src, int32_t pos, Pack * dst)
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  25, src_max = pos + 4; pos < src_max; pos++, BYTE += 4)
        set_bytes(floatToIntBits(src[pos]), 4, data,  BYTE);
}
INLINER float p243_approach_x_GET(Pack * src)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  41, 4)));
}
INLINER void p243_approach_x_SET(float  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  41);
}
INLINER float p243_approach_y_GET(Pack * src)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  45, 4)));
}
INLINER void p243_approach_y_SET(float  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  45);
}
INLINER float p243_approach_z_GET(Pack * src)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  49, 4)));
}
INLINER void p243_approach_z_SET(float  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  49);
}
INLINER uint64_t  p243_time_usec_TRY(Bounds_Inside * src)
{
    if(src->base.field_bit !=  424 && !try_visit_field(src, 424)) return 0;
    uint8_t * data = src->base.pack->data;
    return ((get_bytes(data,  src->BYTE, 8)));
}
INLINER void p243_time_usec_SET(uint64_t  src, Bounds_Inside * dst)//Timestamp (microseconds since UNIX epoch or microseconds since system boot
{
    if(dst->base.field_bit != 424)insert_field(dst, 424, 0);
    uint8_t * data = dst->base.pack->data;
    set_bytes((src), 8, data,  dst->BYTE);
}
INLINER uint16_t p244_message_id_GET(Pack * src)//The ID of the requested MAVLink message. v1.0 is limited to 254 messages
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER void p244_message_id_SET(uint16_t  src, Pack * dst)//The ID of the requested MAVLink message. v1.0 is limited to 254 messages
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER int32_t p244_interval_us_GET(Pack * src)//0 indicates the interval at which it is sent
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  2, 4)));
}
INLINER void p244_interval_us_SET(int32_t  src, Pack * dst)//0 indicates the interval at which it is sent
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  2);
}
INLINER e_MAV_VTOL_STATE p245_vtol_state_GET(Pack * src)//The VTOL state if applicable. Is set to MAV_VTOL_STATE_UNDEFINED if UAV is not in VTOL configuratio
{
    uint8_t * data = src->data;
    return  0 + (int)get_bits(data, 0, 3);
}
INLINER void p245_vtol_state_SET(e_MAV_VTOL_STATE  src, Pack * dst)//The VTOL state if applicable. Is set to MAV_VTOL_STATE_UNDEFINED if UAV is not in VTOL configuratio
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 3, data, 0);
}
INLINER e_MAV_LANDED_STATE p245_landed_state_GET(Pack * src)//The landed state. Is set to MAV_LANDED_STATE_UNDEFINED if landed state is unknown
{
    uint8_t * data = src->data;
    return  0 + (int)get_bits(data, 3, 3);
}
INLINER void p245_landed_state_SET(e_MAV_LANDED_STATE  src, Pack * dst)//The landed state. Is set to MAV_LANDED_STATE_UNDEFINED if landed state is unknown
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 3, data, 3);
}
INLINER uint16_t p246_heading_GET(Pack * src)//Course over ground in centidegree
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER void p246_heading_SET(uint16_t  src, Pack * dst)//Course over ground in centidegree
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER uint16_t p246_hor_velocity_GET(Pack * src)//The horizontal velocity in centimeters/secon
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 2)));
}
INLINER void p246_hor_velocity_SET(uint16_t  src, Pack * dst)//The horizontal velocity in centimeters/secon
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  2);
}
INLINER uint16_t p246_squawk_GET(Pack * src)//Squawk cod
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  4, 2)));
}
INLINER void p246_squawk_SET(uint16_t  src, Pack * dst)//Squawk cod
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  4);
}
INLINER uint32_t p246_ICAO_address_GET(Pack * src)//ICAO addres
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  6, 4)));
}
INLINER void p246_ICAO_address_SET(uint32_t  src, Pack * dst)//ICAO addres
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  6);
}
INLINER int32_t p246_lat_GET(Pack * src)//Latitude, expressed as degrees * 1E
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  10, 4)));
}
INLINER void p246_lat_SET(int32_t  src, Pack * dst)//Latitude, expressed as degrees * 1E
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  10);
}
INLINER int32_t p246_lon_GET(Pack * src)//Longitude, expressed as degrees * 1E
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  14, 4)));
}
INLINER void p246_lon_SET(int32_t  src, Pack * dst)//Longitude, expressed as degrees * 1E
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  14);
}
INLINER int32_t p246_altitude_GET(Pack * src)//Altitude(ASL) in millimeter
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  18, 4)));
}
INLINER void p246_altitude_SET(int32_t  src, Pack * dst)//Altitude(ASL) in millimeter
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  18);
}
INLINER int16_t p246_ver_velocity_GET(Pack * src)//The vertical velocity in centimeters/second, positive is u
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  22, 2)));
}
INLINER void p246_ver_velocity_SET(int16_t  src, Pack * dst)//The vertical velocity in centimeters/second, positive is u
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  22);
}
INLINER uint8_t p246_tslc_GET(Pack * src)//Time since last communication in second
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  24, 1)));
}
INLINER void p246_tslc_SET(uint8_t  src, Pack * dst)//Time since last communication in second
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  24);
}
INLINER e_ADSB_ALTITUDE_TYPE p246_altitude_type_GET(Pack * src)//Type from ADSB_ALTITUDE_TYPE enu
{
    uint8_t * data = src->data;
    return  0 + (int)get_bits(data, 200, 2);
}
INLINER void p246_altitude_type_SET(e_ADSB_ALTITUDE_TYPE  src, Pack * dst)//Type from ADSB_ALTITUDE_TYPE enu
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 2, data, 200);
}
INLINER e_ADSB_EMITTER_TYPE p246_emitter_type_GET(Pack * src)//Type from ADSB_EMITTER_TYPE enu
{
    uint8_t * data = src->data;
    return  0 + (int)get_bits(data, 202, 5);
}
INLINER void p246_emitter_type_SET(e_ADSB_EMITTER_TYPE  src, Pack * dst)//Type from ADSB_EMITTER_TYPE enu
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 5, data, 202);
}
INLINER e_ADSB_FLAGS p246_flags_GET(Pack * src)//Flags to indicate various statuses including valid data field
{
    uint8_t * data = src->data;
    switch(get_bits(data, 207, 3))
    {
        case 0:
            return e_ADSB_FLAGS_ADSB_FLAGS_VALID_COORDS;
        case 1:
            return e_ADSB_FLAGS_ADSB_FLAGS_VALID_ALTITUDE;
        case 2:
            return e_ADSB_FLAGS_ADSB_FLAGS_VALID_HEADING;
        case 3:
            return e_ADSB_FLAGS_ADSB_FLAGS_VALID_VELOCITY;
        case 4:
            return e_ADSB_FLAGS_ADSB_FLAGS_VALID_CALLSIGN;
        case 5:
            return e_ADSB_FLAGS_ADSB_FLAGS_VALID_SQUAWK;
        case 6:
            return e_ADSB_FLAGS_ADSB_FLAGS_SIMULATED;
        default: ;//assert(false);//("Unknown enum ID " + id);
    }
}
INLINER void p246_flags_SET(e_ADSB_FLAGS  src, Pack * dst)//Flags to indicate various statuses including valid data field
{
    uint8_t * data = dst->data;
    int32_t id;
    switch(src)
    {
        case e_ADSB_FLAGS_ADSB_FLAGS_VALID_COORDS:
            id = 0;
            break;
        case e_ADSB_FLAGS_ADSB_FLAGS_VALID_ALTITUDE:
            id = 1;
            break;
        case e_ADSB_FLAGS_ADSB_FLAGS_VALID_HEADING:
            id = 2;
            break;
        case e_ADSB_FLAGS_ADSB_FLAGS_VALID_VELOCITY:
            id = 3;
            break;
        case e_ADSB_FLAGS_ADSB_FLAGS_VALID_CALLSIGN:
            id = 4;
            break;
        case e_ADSB_FLAGS_ADSB_FLAGS_VALID_SQUAWK:
            id = 5;
            break;
        case e_ADSB_FLAGS_ADSB_FLAGS_SIMULATED:
            id = 6;
            break;
        default: ;// assert(false);//("Unknown enum" + id);
    }
    set_bits(id, 3, data, 207);
}
INLINER char16_t * p246_callsign_GET(Bounds_Inside * src, char16_t *  dst, int32_t pos) //The callsign, 8+nul
{
    uint8_t * data = src->base.pack->data;
    for(int32_t BYTE = src->BYTE, dst_max = pos + src->items; pos < dst_max ; pos++,  BYTE += 2)
        dst[pos] = ((get_bytes(data,  BYTE, 2)));
    return dst;
}
INLINER int32_t p246_callsign_LEN(Bounds_Inside * src)
{
    return (src->base.field_bit !=  210 && !try_visit_field(src, 210)  ||  !try_visit_item(src, 0)) ? 0 : src->items;
}
INLINER  char16_t* p246_callsign_TRY_(Bounds_Inside * src)
{
    if(src->base.field_bit !=  210 && !try_visit_field(src, 210)  ||  !try_visit_item(src, 0)) return NULL;
    char16_t * ret = p246_callsign_GET(src, malloc((src->items + 1) * 2), 0);
    ret[src->items] = 0;//the terminating null character.
    return ret;
}
INLINER void p246_callsign_SET(char16_t *  src, int32_t pos, int32_t items, Bounds_Inside * dst) //The callsign, 8+nul
{
    if(dst->base.field_bit != 210 && insert_field(dst, 210, items) ||
            ! try_visit_item(dst, 0)) insert_item(dst, 0, items);
    uint8_t * data = dst->base.pack->data;
    for(int32_t BYTE =  dst->BYTE, src_max = pos + dst->items; pos < src_max; pos++, BYTE += 2)
        set_bytes((uint16_t)(src[pos]), 2, data,  BYTE);
}
INLINER void p246_callsign_SET_(char16_t*  src, Bounds_Inside * dst) {p246_callsign_SET(src, 0, strlen16(src), dst);}
INLINER uint32_t p247_id_GET(Pack * src)//Unique identifier, domain based on src fiel
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 4)));
}
INLINER void p247_id_SET(uint32_t  src, Pack * dst)//Unique identifier, domain based on src fiel
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  0);
}
INLINER float p247_time_to_minimum_delta_GET(Pack * src)//Estimated time until collision occurs (seconds
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  4, 4)));
}
INLINER void p247_time_to_minimum_delta_SET(float  src, Pack * dst)//Estimated time until collision occurs (seconds
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  4);
}
INLINER float p247_altitude_minimum_delta_GET(Pack * src)//Closest vertical distance in meters between vehicle and objec
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  8, 4)));
}
INLINER void p247_altitude_minimum_delta_SET(float  src, Pack * dst)//Closest vertical distance in meters between vehicle and objec
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  8);
}
INLINER float p247_horizontal_minimum_delta_GET(Pack * src)//Closest horizontal distance in meteres between vehicle and objec
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  12, 4)));
}
INLINER void p247_horizontal_minimum_delta_SET(float  src, Pack * dst)//Closest horizontal distance in meteres between vehicle and objec
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  12);
}
INLINER e_MAV_COLLISION_SRC p247_src__GET(Pack * src)//Collision data sourc
{
    uint8_t * data = src->data;
    return  0 + (int)get_bits(data, 128, 2);
}
INLINER void p247_src__SET(e_MAV_COLLISION_SRC  src, Pack * dst)//Collision data sourc
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 2, data, 128);
}
INLINER e_MAV_COLLISION_ACTION p247_action_GET(Pack * src)//Action that is being taken to avoid this collisio
{
    uint8_t * data = src->data;
    return  0 + (int)get_bits(data, 130, 3);
}
INLINER void p247_action_SET(e_MAV_COLLISION_ACTION  src, Pack * dst)//Action that is being taken to avoid this collisio
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 3, data, 130);
}
INLINER e_MAV_COLLISION_THREAT_LEVEL p247_threat_level_GET(Pack * src)//How concerned the aircraft is about this collisio
{
    uint8_t * data = src->data;
    return  0 + (int)get_bits(data, 133, 2);
}
INLINER void p247_threat_level_SET(e_MAV_COLLISION_THREAT_LEVEL  src, Pack * dst)//How concerned the aircraft is about this collisio
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 2, data, 133);
}
INLINER uint16_t p248_message_type_GET(Pack * src)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER void p248_message_type_SET(uint16_t  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER uint8_t p248_target_network_GET(Pack * src)//Network ID (0 for broadcast
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 1)));
}
INLINER void p248_target_network_SET(uint8_t  src, Pack * dst)//Network ID (0 for broadcast
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  2);
}
INLINER uint8_t p248_target_system_GET(Pack * src)//System ID (0 for broadcast
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  3, 1)));
}
INLINER void p248_target_system_SET(uint8_t  src, Pack * dst)//System ID (0 for broadcast
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  3);
}
INLINER uint8_t p248_target_component_GET(Pack * src)//Component ID (0 for broadcast
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  4, 1)));
}
INLINER void p248_target_component_SET(uint8_t  src, Pack * dst)//Component ID (0 for broadcast
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  4);
}
INLINER uint8_t* p248_payload_GET(Pack * src, uint8_t*  dst, int32_t pos)
{
    uint8_t * data = src->data;
    for(int32_t BYTE = 5, dst_max = pos + 249; pos < dst_max ; pos++,  BYTE += 1)
        dst[pos] = ((get_bytes(data,  BYTE, 1)));
    return dst;
}

static const  uint32_t p248_payload_LEN = 249; //return array length

INLINER  uint8_t*  p248_payload_GET_(Pack * src) {return p248_payload_GET(src, malloc(249 * sizeof(uint8_t)), 0);}
INLINER void p248_payload_SET(uint8_t*  src, int32_t pos, Pack * dst)
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  5, src_max = pos + 249; pos < src_max; pos++, BYTE += 1)
        set_bytes((src[pos]), 1, data,  BYTE);
}
INLINER uint16_t p249_address_GET(Pack * src)//Starting address of the debug variable
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER void p249_address_SET(uint16_t  src, Pack * dst)//Starting address of the debug variable
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER uint8_t p249_ver_GET(Pack * src)//Version code of the type variable. 0=unknown, type ignored and assumed int16_t. 1=as belo
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 1)));
}
INLINER void p249_ver_SET(uint8_t  src, Pack * dst)//Version code of the type variable. 0=unknown, type ignored and assumed int16_t. 1=as belo
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  2);
}
INLINER uint8_t p249_type_GET(Pack * src)//Type code of the memory variables. for ver = 1: 0=16 x int16_t, 1=16 x uint16_t, 2=16 x Q15, 3=16 x 1Q
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  3, 1)));
}
INLINER void p249_type_SET(uint8_t  src, Pack * dst)//Type code of the memory variables. for ver = 1: 0=16 x int16_t, 1=16 x uint16_t, 2=16 x Q15, 3=16 x 1Q
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  3);
}
INLINER int8_t* p249_value_GET(Pack * src, int8_t*  dst, int32_t pos) //Memory contents at specified addres
{
    uint8_t * data = src->data;
    for(int32_t BYTE = 4, dst_max = pos + 32; pos < dst_max ; pos++,  BYTE += 1)
        dst[pos] = ((int8_t)(get_bytes(data,  BYTE, 1)));
    return dst;
}

static const  uint32_t p249_value_LEN = 32; //return array length

INLINER  int8_t*  p249_value_GET_(Pack * src) {return p249_value_GET(src, malloc(32 * sizeof(int8_t)), 0);}
INLINER void p249_value_SET(int8_t*  src, int32_t pos, Pack * dst) //Memory contents at specified addres
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  4, src_max = pos + 32; pos < src_max; pos++, BYTE += 1)
        set_bytes((uint8_t)(src[pos]), 1, data,  BYTE);
}
INLINER uint64_t p250_time_usec_GET(Pack * src)//Timestam
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 8)));
}
INLINER void p250_time_usec_SET(uint64_t  src, Pack * dst)//Timestam
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  0);
}
INLINER float p250_x_GET(Pack * src)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  8, 4)));
}
INLINER void p250_x_SET(float  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  8);
}
INLINER float p250_y_GET(Pack * src)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  12, 4)));
}
INLINER void p250_y_SET(float  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  12);
}
INLINER float p250_z_GET(Pack * src)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  16, 4)));
}
INLINER void p250_z_SET(float  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  16);
}
INLINER char16_t * p250_name_GET(Bounds_Inside * src, char16_t *  dst, int32_t pos) //Nam
{
    uint8_t * data = src->base.pack->data;
    for(int32_t BYTE = src->BYTE, dst_max = pos + src->items; pos < dst_max ; pos++,  BYTE += 2)
        dst[pos] = ((get_bytes(data,  BYTE, 2)));
    return dst;
}
INLINER int32_t p250_name_LEN(Bounds_Inside * src)
{
    return (src->base.field_bit !=  160 && !try_visit_field(src, 160)  ||  !try_visit_item(src, 0)) ? 0 : src->items;
}
INLINER  char16_t* p250_name_TRY_(Bounds_Inside * src)
{
    if(src->base.field_bit !=  160 && !try_visit_field(src, 160)  ||  !try_visit_item(src, 0)) return NULL;
    char16_t * ret = p250_name_GET(src, malloc((src->items + 1) * 2), 0);
    ret[src->items] = 0;//the terminating null character.
    return ret;
}
INLINER void p250_name_SET(char16_t *  src, int32_t pos, int32_t items, Bounds_Inside * dst) //Nam
{
    if(dst->base.field_bit != 160 && insert_field(dst, 160, items) ||
            ! try_visit_item(dst, 0)) insert_item(dst, 0, items);
    uint8_t * data = dst->base.pack->data;
    for(int32_t BYTE =  dst->BYTE, src_max = pos + dst->items; pos < src_max; pos++, BYTE += 2)
        set_bytes((uint16_t)(src[pos]), 2, data,  BYTE);
}
INLINER void p250_name_SET_(char16_t*  src, Bounds_Inside * dst) {p250_name_SET(src, 0, strlen16(src), dst);}
INLINER uint32_t p251_time_boot_ms_GET(Pack * src)//Timestamp (milliseconds since system boot
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 4)));
}
INLINER void p251_time_boot_ms_SET(uint32_t  src, Pack * dst)//Timestamp (milliseconds since system boot
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  0);
}
INLINER float p251_value_GET(Pack * src)//Floating point valu
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  4, 4)));
}
INLINER void p251_value_SET(float  src, Pack * dst)//Floating point valu
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  4);
}
INLINER char16_t * p251_name_GET(Bounds_Inside * src, char16_t *  dst, int32_t pos) //Name of the debug variabl
{
    uint8_t * data = src->base.pack->data;
    for(int32_t BYTE = src->BYTE, dst_max = pos + src->items; pos < dst_max ; pos++,  BYTE += 2)
        dst[pos] = ((get_bytes(data,  BYTE, 2)));
    return dst;
}
INLINER int32_t p251_name_LEN(Bounds_Inside * src)
{
    return (src->base.field_bit !=  64 && !try_visit_field(src, 64)  ||  !try_visit_item(src, 0)) ? 0 : src->items;
}
INLINER  char16_t* p251_name_TRY_(Bounds_Inside * src)
{
    if(src->base.field_bit !=  64 && !try_visit_field(src, 64)  ||  !try_visit_item(src, 0)) return NULL;
    char16_t * ret = p251_name_GET(src, malloc((src->items + 1) * 2), 0);
    ret[src->items] = 0;//the terminating null character.
    return ret;
}
INLINER void p251_name_SET(char16_t *  src, int32_t pos, int32_t items, Bounds_Inside * dst) //Name of the debug variabl
{
    if(dst->base.field_bit != 64 && insert_field(dst, 64, items) ||
            ! try_visit_item(dst, 0)) insert_item(dst, 0, items);
    uint8_t * data = dst->base.pack->data;
    for(int32_t BYTE =  dst->BYTE, src_max = pos + dst->items; pos < src_max; pos++, BYTE += 2)
        set_bytes((uint16_t)(src[pos]), 2, data,  BYTE);
}
INLINER void p251_name_SET_(char16_t*  src, Bounds_Inside * dst) {p251_name_SET(src, 0, strlen16(src), dst);}
INLINER uint32_t p252_time_boot_ms_GET(Pack * src)//Timestamp (milliseconds since system boot
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 4)));
}
INLINER void p252_time_boot_ms_SET(uint32_t  src, Pack * dst)//Timestamp (milliseconds since system boot
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  0);
}
INLINER int32_t p252_value_GET(Pack * src)//Signed integer valu
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  4, 4)));
}
INLINER void p252_value_SET(int32_t  src, Pack * dst)//Signed integer valu
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  4);
}
INLINER char16_t * p252_name_GET(Bounds_Inside * src, char16_t *  dst, int32_t pos) //Name of the debug variabl
{
    uint8_t * data = src->base.pack->data;
    for(int32_t BYTE = src->BYTE, dst_max = pos + src->items; pos < dst_max ; pos++,  BYTE += 2)
        dst[pos] = ((get_bytes(data,  BYTE, 2)));
    return dst;
}
INLINER int32_t p252_name_LEN(Bounds_Inside * src)
{
    return (src->base.field_bit !=  64 && !try_visit_field(src, 64)  ||  !try_visit_item(src, 0)) ? 0 : src->items;
}
INLINER  char16_t* p252_name_TRY_(Bounds_Inside * src)
{
    if(src->base.field_bit !=  64 && !try_visit_field(src, 64)  ||  !try_visit_item(src, 0)) return NULL;
    char16_t * ret = p252_name_GET(src, malloc((src->items + 1) * 2), 0);
    ret[src->items] = 0;//the terminating null character.
    return ret;
}
INLINER void p252_name_SET(char16_t *  src, int32_t pos, int32_t items, Bounds_Inside * dst) //Name of the debug variabl
{
    if(dst->base.field_bit != 64 && insert_field(dst, 64, items) ||
            ! try_visit_item(dst, 0)) insert_item(dst, 0, items);
    uint8_t * data = dst->base.pack->data;
    for(int32_t BYTE =  dst->BYTE, src_max = pos + dst->items; pos < src_max; pos++, BYTE += 2)
        set_bytes((uint16_t)(src[pos]), 2, data,  BYTE);
}
INLINER void p252_name_SET_(char16_t*  src, Bounds_Inside * dst) {p252_name_SET(src, 0, strlen16(src), dst);}
INLINER e_MAV_SEVERITY p253_severity_GET(Pack * src)//Severity of status. Relies on the definitions within RFC-5424. See enum MAV_SEVERITY
{
    uint8_t * data = src->data;
    return  0 + (int)get_bits(data, 0, 4);
}
INLINER void p253_severity_SET(e_MAV_SEVERITY  src, Pack * dst)//Severity of status. Relies on the definitions within RFC-5424. See enum MAV_SEVERITY
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 4, data, 0);
}
INLINER char16_t * p253_text_GET(Bounds_Inside * src, char16_t *  dst, int32_t pos) //Status text message, without null termination characte
{
    uint8_t * data = src->base.pack->data;
    for(int32_t BYTE = src->BYTE, dst_max = pos + src->items; pos < dst_max ; pos++,  BYTE += 2)
        dst[pos] = ((get_bytes(data,  BYTE, 2)));
    return dst;
}
INLINER int32_t p253_text_LEN(Bounds_Inside * src)
{
    return (src->base.field_bit !=  4 && !try_visit_field(src, 4)  ||  !try_visit_item(src, 0)) ? 0 : src->items;
}
INLINER  char16_t* p253_text_TRY_(Bounds_Inside * src)
{
    if(src->base.field_bit !=  4 && !try_visit_field(src, 4)  ||  !try_visit_item(src, 0)) return NULL;
    char16_t * ret = p253_text_GET(src, malloc((src->items + 1) * 2), 0);
    ret[src->items] = 0;//the terminating null character.
    return ret;
}
INLINER void p253_text_SET(char16_t *  src, int32_t pos, int32_t items, Bounds_Inside * dst) //Status text message, without null termination characte
{
    if(dst->base.field_bit != 4 && insert_field(dst, 4, items) ||
            ! try_visit_item(dst, 0)) insert_item(dst, 0, items);
    uint8_t * data = dst->base.pack->data;
    for(int32_t BYTE =  dst->BYTE, src_max = pos + dst->items; pos < src_max; pos++, BYTE += 2)
        set_bytes((uint16_t)(src[pos]), 2, data,  BYTE);
}
INLINER void p253_text_SET_(char16_t*  src, Bounds_Inside * dst) {p253_text_SET(src, 0, strlen16(src), dst);}
INLINER uint32_t p254_time_boot_ms_GET(Pack * src)//Timestamp (milliseconds since system boot
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 4)));
}
INLINER void p254_time_boot_ms_SET(uint32_t  src, Pack * dst)//Timestamp (milliseconds since system boot
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  0);
}
INLINER uint8_t p254_ind_GET(Pack * src)//index of debug variabl
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  4, 1)));
}
INLINER void p254_ind_SET(uint8_t  src, Pack * dst)//index of debug variabl
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  4);
}
INLINER float p254_value_GET(Pack * src)//DEBUG valu
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  5, 4)));
}
INLINER void p254_value_SET(float  src, Pack * dst)//DEBUG valu
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  5);
}
INLINER uint64_t p256_initial_timestamp_GET(Pack * src)//initial timestam
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 8)));
}
INLINER void p256_initial_timestamp_SET(uint64_t  src, Pack * dst)//initial timestam
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  0);
}
INLINER uint8_t p256_target_system_GET(Pack * src)//system id of the targe
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  8, 1)));
}
INLINER void p256_target_system_SET(uint8_t  src, Pack * dst)//system id of the targe
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  8);
}
INLINER uint8_t p256_target_component_GET(Pack * src)//component ID of the targe
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  9, 1)));
}
INLINER void p256_target_component_SET(uint8_t  src, Pack * dst)//component ID of the targe
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  9);
}
INLINER uint8_t* p256_secret_key_GET(Pack * src, uint8_t*  dst, int32_t pos) //signing ke
{
    uint8_t * data = src->data;
    for(int32_t BYTE = 10, dst_max = pos + 32; pos < dst_max ; pos++,  BYTE += 1)
        dst[pos] = ((get_bytes(data,  BYTE, 1)));
    return dst;
}

static const  uint32_t p256_secret_key_LEN = 32; //return array length

INLINER  uint8_t*  p256_secret_key_GET_(Pack * src) {return p256_secret_key_GET(src, malloc(32 * sizeof(uint8_t)), 0);}
INLINER void p256_secret_key_SET(uint8_t*  src, int32_t pos, Pack * dst) //signing ke
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  10, src_max = pos + 32; pos < src_max; pos++, BYTE += 1)
        set_bytes((src[pos]), 1, data,  BYTE);
}
INLINER uint32_t p257_time_boot_ms_GET(Pack * src)//Timestamp (milliseconds since system boot
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 4)));
}
INLINER void p257_time_boot_ms_SET(uint32_t  src, Pack * dst)//Timestamp (milliseconds since system boot
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  0);
}
INLINER uint32_t p257_last_change_ms_GET(Pack * src)//Time of last change of button stat
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  4, 4)));
}
INLINER void p257_last_change_ms_SET(uint32_t  src, Pack * dst)//Time of last change of button stat
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  4);
}
INLINER uint8_t p257_state_GET(Pack * src)//Bitmap state of button
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  8, 1)));
}
INLINER void p257_state_SET(uint8_t  src, Pack * dst)//Bitmap state of button
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  8);
}
INLINER uint8_t p258_target_system_GET(Pack * src)//System I
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 1)));
}
INLINER void p258_target_system_SET(uint8_t  src, Pack * dst)//System I
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  0);
}
INLINER uint8_t p258_target_component_GET(Pack * src)//Component I
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  1, 1)));
}
INLINER void p258_target_component_SET(uint8_t  src, Pack * dst)//Component I
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  1);
}
INLINER char16_t * p258_tune_GET(Bounds_Inside * src, char16_t *  dst, int32_t pos) //tune in board specific forma
{
    uint8_t * data = src->base.pack->data;
    for(int32_t BYTE = src->BYTE, dst_max = pos + src->items; pos < dst_max ; pos++,  BYTE += 2)
        dst[pos] = ((get_bytes(data,  BYTE, 2)));
    return dst;
}
INLINER int32_t p258_tune_LEN(Bounds_Inside * src)
{
    return (src->base.field_bit !=  16 && !try_visit_field(src, 16)  ||  !try_visit_item(src, 0)) ? 0 : src->items;
}
INLINER  char16_t* p258_tune_TRY_(Bounds_Inside * src)
{
    if(src->base.field_bit !=  16 && !try_visit_field(src, 16)  ||  !try_visit_item(src, 0)) return NULL;
    char16_t * ret = p258_tune_GET(src, malloc((src->items + 1) * 2), 0);
    ret[src->items] = 0;//the terminating null character.
    return ret;
}
INLINER void p258_tune_SET(char16_t *  src, int32_t pos, int32_t items, Bounds_Inside * dst) //tune in board specific forma
{
    if(dst->base.field_bit != 16 && insert_field(dst, 16, items) ||
            ! try_visit_item(dst, 0)) insert_item(dst, 0, items);
    uint8_t * data = dst->base.pack->data;
    for(int32_t BYTE =  dst->BYTE, src_max = pos + dst->items; pos < src_max; pos++, BYTE += 2)
        set_bytes((uint16_t)(src[pos]), 2, data,  BYTE);
}
INLINER void p258_tune_SET_(char16_t*  src, Bounds_Inside * dst) {p258_tune_SET(src, 0, strlen16(src), dst);}
INLINER uint16_t p259_resolution_h_GET(Pack * src)//Image resolution in pixels horizonta
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER void p259_resolution_h_SET(uint16_t  src, Pack * dst)//Image resolution in pixels horizonta
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER uint16_t p259_resolution_v_GET(Pack * src)//Image resolution in pixels vertica
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 2)));
}
INLINER void p259_resolution_v_SET(uint16_t  src, Pack * dst)//Image resolution in pixels vertica
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  2);
}
INLINER uint16_t p259_cam_definition_version_GET(Pack * src)//Camera definition version (iteration
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  4, 2)));
}
INLINER void p259_cam_definition_version_SET(uint16_t  src, Pack * dst)//Camera definition version (iteration
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  4);
}
INLINER uint32_t p259_time_boot_ms_GET(Pack * src)//Timestamp (milliseconds since system boot
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  6, 4)));
}
INLINER void p259_time_boot_ms_SET(uint32_t  src, Pack * dst)//Timestamp (milliseconds since system boot
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  6);
}
INLINER uint32_t p259_firmware_version_GET(Pack * src)//0xff = Major
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  10, 4)));
}
INLINER void p259_firmware_version_SET(uint32_t  src, Pack * dst)//0xff = Major
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  10);
}
INLINER uint8_t* p259_vendor_name_GET(Pack * src, uint8_t*  dst, int32_t pos) //Name of the camera vendo
{
    uint8_t * data = src->data;
    for(int32_t BYTE = 14, dst_max = pos + 32; pos < dst_max ; pos++,  BYTE += 1)
        dst[pos] = ((get_bytes(data,  BYTE, 1)));
    return dst;
}

static const  uint32_t p259_vendor_name_LEN = 32; //return array length

INLINER  uint8_t*  p259_vendor_name_GET_(Pack * src) {return p259_vendor_name_GET(src, malloc(32 * sizeof(uint8_t)), 0);}
INLINER void p259_vendor_name_SET(uint8_t*  src, int32_t pos, Pack * dst) //Name of the camera vendo
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  14, src_max = pos + 32; pos < src_max; pos++, BYTE += 1)
        set_bytes((src[pos]), 1, data,  BYTE);
}
INLINER uint8_t* p259_model_name_GET(Pack * src, uint8_t*  dst, int32_t pos) //Name of the camera mode
{
    uint8_t * data = src->data;
    for(int32_t BYTE = 46, dst_max = pos + 32; pos < dst_max ; pos++,  BYTE += 1)
        dst[pos] = ((get_bytes(data,  BYTE, 1)));
    return dst;
}

static const  uint32_t p259_model_name_LEN = 32; //return array length

INLINER  uint8_t*  p259_model_name_GET_(Pack * src) {return p259_model_name_GET(src, malloc(32 * sizeof(uint8_t)), 0);}
INLINER void p259_model_name_SET(uint8_t*  src, int32_t pos, Pack * dst) //Name of the camera mode
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  46, src_max = pos + 32; pos < src_max; pos++, BYTE += 1)
        set_bytes((src[pos]), 1, data,  BYTE);
}
INLINER float p259_focal_length_GET(Pack * src)//Focal length in m
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  78, 4)));
}
INLINER void p259_focal_length_SET(float  src, Pack * dst)//Focal length in m
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  78);
}
INLINER float p259_sensor_size_h_GET(Pack * src)//Image sensor size horizontal in m
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  82, 4)));
}
INLINER void p259_sensor_size_h_SET(float  src, Pack * dst)//Image sensor size horizontal in m
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  82);
}
INLINER float p259_sensor_size_v_GET(Pack * src)//Image sensor size vertical in m
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  86, 4)));
}
INLINER void p259_sensor_size_v_SET(float  src, Pack * dst)//Image sensor size vertical in m
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  86);
}
INLINER uint8_t p259_lens_id_GET(Pack * src)//Reserved for a lens I
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  90, 1)));
}
INLINER void p259_lens_id_SET(uint8_t  src, Pack * dst)//Reserved for a lens I
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  90);
}
INLINER e_CAMERA_CAP_FLAGS p259_flags_GET(Pack * src)//CAMERA_CAP_FLAGS enum flags (bitmap) describing camera capabilities
{
    uint8_t * data = src->data;
    switch(get_bits(data, 728, 3))
    {
        case 0:
            return e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_CAPTURE_VIDEO;
        case 1:
            return e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_CAPTURE_IMAGE;
        case 2:
            return e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_HAS_MODES;
        case 3:
            return e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE;
        case 4:
            return e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE;
        case 5:
            return e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE;
        default: ;//assert(false);//("Unknown enum ID " + id);
    }
}
INLINER void p259_flags_SET(e_CAMERA_CAP_FLAGS  src, Pack * dst)//CAMERA_CAP_FLAGS enum flags (bitmap) describing camera capabilities
{
    uint8_t * data = dst->data;
    int32_t id;
    switch(src)
    {
        case e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_CAPTURE_VIDEO:
            id = 0;
            break;
        case e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_CAPTURE_IMAGE:
            id = 1;
            break;
        case e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_HAS_MODES:
            id = 2;
            break;
        case e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE:
            id = 3;
            break;
        case e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE:
            id = 4;
            break;
        case e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE:
            id = 5;
            break;
        default: ;// assert(false);//("Unknown enum" + id);
    }
    set_bits(id, 3, data, 728);
}
INLINER char16_t * p259_cam_definition_uri_GET(Bounds_Inside * src, char16_t *  dst, int32_t pos) //Camera definition URI (if any, otherwise only basic functions will be available)
{
    uint8_t * data = src->base.pack->data;
    for(int32_t BYTE = src->BYTE, dst_max = pos + src->items; pos < dst_max ; pos++,  BYTE += 2)
        dst[pos] = ((get_bytes(data,  BYTE, 2)));
    return dst;
}
INLINER int32_t p259_cam_definition_uri_LEN(Bounds_Inside * src)
{
    return (src->base.field_bit !=  731 && !try_visit_field(src, 731)  ||  !try_visit_item(src, 0)) ? 0 : src->items;
}
INLINER  char16_t* p259_cam_definition_uri_TRY_(Bounds_Inside * src)
{
    if(src->base.field_bit !=  731 && !try_visit_field(src, 731)  ||  !try_visit_item(src, 0)) return NULL;
    char16_t * ret = p259_cam_definition_uri_GET(src, malloc((src->items + 1) * 2), 0);
    ret[src->items] = 0;//the terminating null character.
    return ret;
}
INLINER void p259_cam_definition_uri_SET(char16_t *  src, int32_t pos, int32_t items, Bounds_Inside * dst) //Camera definition URI (if any, otherwise only basic functions will be available)
{
    if(dst->base.field_bit != 731 && insert_field(dst, 731, items) ||
            ! try_visit_item(dst, 0)) insert_item(dst, 0, items);
    uint8_t * data = dst->base.pack->data;
    for(int32_t BYTE =  dst->BYTE, src_max = pos + dst->items; pos < src_max; pos++, BYTE += 2)
        set_bytes((uint16_t)(src[pos]), 2, data,  BYTE);
}
INLINER void p259_cam_definition_uri_SET_(char16_t*  src, Bounds_Inside * dst) {p259_cam_definition_uri_SET(src, 0, strlen16(src), dst);}
INLINER uint32_t p260_time_boot_ms_GET(Pack * src)//Timestamp (milliseconds since system boot
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 4)));
}
INLINER void p260_time_boot_ms_SET(uint32_t  src, Pack * dst)//Timestamp (milliseconds since system boot
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  0);
}
INLINER e_CAMERA_MODE p260_mode_id_GET(Pack * src)//Camera mode (CAMERA_MODE
{
    uint8_t * data = src->data;
    return  0 + (int)get_bits(data, 32, 2);
}
INLINER void p260_mode_id_SET(e_CAMERA_MODE  src, Pack * dst)//Camera mode (CAMERA_MODE
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 2, data, 32);
}
INLINER uint32_t p261_time_boot_ms_GET(Pack * src)//Timestamp (milliseconds since system boot
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 4)));
}
INLINER void p261_time_boot_ms_SET(uint32_t  src, Pack * dst)//Timestamp (milliseconds since system boot
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  0);
}
INLINER uint8_t p261_storage_id_GET(Pack * src)//Storage ID (1 for first, 2 for second, etc.
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  4, 1)));
}
INLINER void p261_storage_id_SET(uint8_t  src, Pack * dst)//Storage ID (1 for first, 2 for second, etc.
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  4);
}
INLINER uint8_t p261_storage_count_GET(Pack * src)//Number of storage device
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  5, 1)));
}
INLINER void p261_storage_count_SET(uint8_t  src, Pack * dst)//Number of storage device
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  5);
}
INLINER uint8_t p261_status_GET(Pack * src)//Status of storage (0 not available, 1 unformatted, 2 formatted
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  6, 1)));
}
INLINER void p261_status_SET(uint8_t  src, Pack * dst)//Status of storage (0 not available, 1 unformatted, 2 formatted
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  6);
}
INLINER float p261_total_capacity_GET(Pack * src)//Total capacity in Mi
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  7, 4)));
}
INLINER void p261_total_capacity_SET(float  src, Pack * dst)//Total capacity in Mi
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  7);
}
INLINER float p261_used_capacity_GET(Pack * src)//Used capacity in Mi
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  11, 4)));
}
INLINER void p261_used_capacity_SET(float  src, Pack * dst)//Used capacity in Mi
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  11);
}
INLINER float p261_available_capacity_GET(Pack * src)//Available capacity in Mi
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  15, 4)));
}
INLINER void p261_available_capacity_SET(float  src, Pack * dst)//Available capacity in Mi
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  15);
}
INLINER float p261_read_speed_GET(Pack * src)//Read speed in MiB/
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  19, 4)));
}
INLINER void p261_read_speed_SET(float  src, Pack * dst)//Read speed in MiB/
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  19);
}
INLINER float p261_write_speed_GET(Pack * src)//Write speed in MiB/
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  23, 4)));
}
INLINER void p261_write_speed_SET(float  src, Pack * dst)//Write speed in MiB/
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  23);
}
INLINER uint32_t p262_time_boot_ms_GET(Pack * src)//Timestamp (milliseconds since system boot
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 4)));
}
INLINER void p262_time_boot_ms_SET(uint32_t  src, Pack * dst)//Timestamp (milliseconds since system boot
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  0);
}
INLINER uint32_t p262_recording_time_ms_GET(Pack * src)//Time in milliseconds since recording starte
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  4, 4)));
}
INLINER void p262_recording_time_ms_SET(uint32_t  src, Pack * dst)//Time in milliseconds since recording starte
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  4);
}
INLINER uint8_t p262_image_status_GET(Pack * src)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  8, 1)));
}
INLINER void p262_image_status_SET(uint8_t  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  8);
}
INLINER uint8_t p262_video_status_GET(Pack * src)//Current status of video capturing (0: idle, 1: capture in progress
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  9, 1)));
}
INLINER void p262_video_status_SET(uint8_t  src, Pack * dst)//Current status of video capturing (0: idle, 1: capture in progress
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  9);
}
INLINER float p262_image_interval_GET(Pack * src)//Image capture interval in second
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  10, 4)));
}
INLINER void p262_image_interval_SET(float  src, Pack * dst)//Image capture interval in second
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  10);
}
INLINER float p262_available_capacity_GET(Pack * src)//Available storage capacity in Mi
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  14, 4)));
}
INLINER void p262_available_capacity_SET(float  src, Pack * dst)//Available storage capacity in Mi
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  14);
}
INLINER uint32_t p263_time_boot_ms_GET(Pack * src)//Timestamp (milliseconds since system boot
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 4)));
}
INLINER void p263_time_boot_ms_SET(uint32_t  src, Pack * dst)//Timestamp (milliseconds since system boot
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  0);
}
INLINER uint64_t p263_time_utc_GET(Pack * src)//Timestamp (microseconds since UNIX epoch) in UTC. 0 for unknown
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  4, 8)));
}
INLINER void p263_time_utc_SET(uint64_t  src, Pack * dst)//Timestamp (microseconds since UNIX epoch) in UTC. 0 for unknown
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  4);
}
INLINER uint8_t p263_camera_id_GET(Pack * src)//Camera ID (1 for first, 2 for second, etc.
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  12, 1)));
}
INLINER void p263_camera_id_SET(uint8_t  src, Pack * dst)//Camera ID (1 for first, 2 for second, etc.
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  12);
}
INLINER int32_t p263_lat_GET(Pack * src)//Latitude, expressed as degrees * 1E7 where image was take
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  13, 4)));
}
INLINER void p263_lat_SET(int32_t  src, Pack * dst)//Latitude, expressed as degrees * 1E7 where image was take
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  13);
}
INLINER int32_t p263_lon_GET(Pack * src)//Longitude, expressed as degrees * 1E7 where capture was take
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  17, 4)));
}
INLINER void p263_lon_SET(int32_t  src, Pack * dst)//Longitude, expressed as degrees * 1E7 where capture was take
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  17);
}
INLINER int32_t p263_alt_GET(Pack * src)//Altitude in meters, expressed as * 1E3 (AMSL, not WGS84) where image was take
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  21, 4)));
}
INLINER void p263_alt_SET(int32_t  src, Pack * dst)//Altitude in meters, expressed as * 1E3 (AMSL, not WGS84) where image was take
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  21);
}
INLINER int32_t p263_relative_alt_GET(Pack * src)//Altitude above ground in meters, expressed as * 1E3 where image was take
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  25, 4)));
}
INLINER void p263_relative_alt_SET(int32_t  src, Pack * dst)//Altitude above ground in meters, expressed as * 1E3 where image was take
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  25);
}
INLINER float* p263_q_GET(Pack * src, float*  dst, int32_t pos) //Quaternion of camera orientation (w, x, y, z order, zero-rotation is 0, 0, 0, 0
{
    uint8_t * data = src->data;
    for(int32_t BYTE = 29, dst_max = pos + 4; pos < dst_max ; pos++,  BYTE += 4)
        dst[pos] = (intBitsToFloat(get_bytes(data,  BYTE, 4)));
    return dst;
}

static const  uint32_t p263_q_LEN = 4; //return array length

INLINER  float*  p263_q_GET_(Pack * src) {return p263_q_GET(src, malloc(4 * sizeof(float)), 0);}
INLINER void p263_q_SET(float*  src, int32_t pos, Pack * dst) //Quaternion of camera orientation (w, x, y, z order, zero-rotation is 0, 0, 0, 0
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  29, src_max = pos + 4; pos < src_max; pos++, BYTE += 4)
        set_bytes(floatToIntBits(src[pos]), 4, data,  BYTE);
}
INLINER int32_t p263_image_index_GET(Pack * src)//Zero based index of this image (image count since armed -1
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  45, 4)));
}
INLINER void p263_image_index_SET(int32_t  src, Pack * dst)//Zero based index of this image (image count since armed -1
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  45);
}
INLINER int8_t p263_capture_result_GET(Pack * src)//Boolean indicating success (1) or failure (0) while capturing this image
{
    uint8_t * data = src->data;
    return ((int8_t)(get_bytes(data,  49, 1)));
}
INLINER void p263_capture_result_SET(int8_t  src, Pack * dst)//Boolean indicating success (1) or failure (0) while capturing this image
{
    uint8_t * data = dst->data;
    set_bytes((uint8_t)(src), 1, data,  49);
}
INLINER char16_t * p263_file_url_GET(Bounds_Inside * src, char16_t *  dst, int32_t pos) //foo.jpg if camera provides an HTTP interface
{
    uint8_t * data = src->base.pack->data;
    for(int32_t BYTE = src->BYTE, dst_max = pos + src->items; pos < dst_max ; pos++,  BYTE += 2)
        dst[pos] = ((get_bytes(data,  BYTE, 2)));
    return dst;
}
INLINER int32_t p263_file_url_LEN(Bounds_Inside * src)
{
    return (src->base.field_bit !=  402 && !try_visit_field(src, 402)  ||  !try_visit_item(src, 0)) ? 0 : src->items;
}
INLINER  char16_t* p263_file_url_TRY_(Bounds_Inside * src)
{
    if(src->base.field_bit !=  402 && !try_visit_field(src, 402)  ||  !try_visit_item(src, 0)) return NULL;
    char16_t * ret = p263_file_url_GET(src, malloc((src->items + 1) * 2), 0);
    ret[src->items] = 0;//the terminating null character.
    return ret;
}
INLINER void p263_file_url_SET(char16_t *  src, int32_t pos, int32_t items, Bounds_Inside * dst) //foo.jpg if camera provides an HTTP interface
{
    if(dst->base.field_bit != 402 && insert_field(dst, 402, items) ||
            ! try_visit_item(dst, 0)) insert_item(dst, 0, items);
    uint8_t * data = dst->base.pack->data;
    for(int32_t BYTE =  dst->BYTE, src_max = pos + dst->items; pos < src_max; pos++, BYTE += 2)
        set_bytes((uint16_t)(src[pos]), 2, data,  BYTE);
}
INLINER void p263_file_url_SET_(char16_t*  src, Bounds_Inside * dst) {p263_file_url_SET(src, 0, strlen16(src), dst);}
INLINER uint32_t p264_time_boot_ms_GET(Pack * src)//Timestamp (milliseconds since system boot
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 4)));
}
INLINER void p264_time_boot_ms_SET(uint32_t  src, Pack * dst)//Timestamp (milliseconds since system boot
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  0);
}
INLINER uint64_t p264_arming_time_utc_GET(Pack * src)//Timestamp at arming (microseconds since UNIX epoch) in UTC, 0 for unknow
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  4, 8)));
}
INLINER void p264_arming_time_utc_SET(uint64_t  src, Pack * dst)//Timestamp at arming (microseconds since UNIX epoch) in UTC, 0 for unknow
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  4);
}
INLINER uint64_t p264_takeoff_time_utc_GET(Pack * src)//Timestamp at takeoff (microseconds since UNIX epoch) in UTC, 0 for unknow
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  12, 8)));
}
INLINER void p264_takeoff_time_utc_SET(uint64_t  src, Pack * dst)//Timestamp at takeoff (microseconds since UNIX epoch) in UTC, 0 for unknow
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  12);
}
INLINER uint64_t p264_flight_uuid_GET(Pack * src)//Universally unique identifier (UUID) of flight, should correspond to name of logfile
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  20, 8)));
}
INLINER void p264_flight_uuid_SET(uint64_t  src, Pack * dst)//Universally unique identifier (UUID) of flight, should correspond to name of logfile
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  20);
}
INLINER uint32_t p265_time_boot_ms_GET(Pack * src)//Timestamp (milliseconds since system boot
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 4)));
}
INLINER void p265_time_boot_ms_SET(uint32_t  src, Pack * dst)//Timestamp (milliseconds since system boot
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  0);
}
INLINER float p265_roll_GET(Pack * src)//Roll in degree
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  4, 4)));
}
INLINER void p265_roll_SET(float  src, Pack * dst)//Roll in degree
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  4);
}
INLINER float p265_pitch_GET(Pack * src)//Pitch in degree
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  8, 4)));
}
INLINER void p265_pitch_SET(float  src, Pack * dst)//Pitch in degree
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  8);
}
INLINER float p265_yaw_GET(Pack * src)//Yaw in degree
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  12, 4)));
}
INLINER void p265_yaw_SET(float  src, Pack * dst)//Yaw in degree
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  12);
}
INLINER uint16_t p266_sequence_GET(Pack * src)//sequence number (can wrap
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER void p266_sequence_SET(uint16_t  src, Pack * dst)//sequence number (can wrap
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER uint8_t p266_target_system_GET(Pack * src)//system ID of the targe
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 1)));
}
INLINER void p266_target_system_SET(uint8_t  src, Pack * dst)//system ID of the targe
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  2);
}
INLINER uint8_t p266_target_component_GET(Pack * src)//component ID of the targe
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  3, 1)));
}
INLINER void p266_target_component_SET(uint8_t  src, Pack * dst)//component ID of the targe
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  3);
}
INLINER uint8_t p266_length_GET(Pack * src)//data lengt
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  4, 1)));
}
INLINER void p266_length_SET(uint8_t  src, Pack * dst)//data lengt
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  4);
}
INLINER uint8_t p266_first_message_offset_GET(Pack * src)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  5, 1)));
}
INLINER void p266_first_message_offset_SET(uint8_t  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  5);
}
INLINER uint8_t* p266_data__GET(Pack * src, uint8_t*  dst, int32_t pos) //logged dat
{
    uint8_t * data = src->data;
    for(int32_t BYTE = 6, dst_max = pos + 249; pos < dst_max ; pos++,  BYTE += 1)
        dst[pos] = ((get_bytes(data,  BYTE, 1)));
    return dst;
}

static const  uint32_t p266_data__LEN = 249; //return array length

INLINER  uint8_t*  p266_data__GET_(Pack * src) {return p266_data__GET(src, malloc(249 * sizeof(uint8_t)), 0);}
INLINER void p266_data__SET(uint8_t*  src, int32_t pos, Pack * dst) //logged dat
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  6, src_max = pos + 249; pos < src_max; pos++, BYTE += 1)
        set_bytes((src[pos]), 1, data,  BYTE);
}
INLINER uint16_t p267_sequence_GET(Pack * src)//sequence number (can wrap
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER void p267_sequence_SET(uint16_t  src, Pack * dst)//sequence number (can wrap
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER uint8_t p267_target_system_GET(Pack * src)//system ID of the targe
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 1)));
}
INLINER void p267_target_system_SET(uint8_t  src, Pack * dst)//system ID of the targe
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  2);
}
INLINER uint8_t p267_target_component_GET(Pack * src)//component ID of the targe
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  3, 1)));
}
INLINER void p267_target_component_SET(uint8_t  src, Pack * dst)//component ID of the targe
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  3);
}
INLINER uint8_t p267_length_GET(Pack * src)//data lengt
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  4, 1)));
}
INLINER void p267_length_SET(uint8_t  src, Pack * dst)//data lengt
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  4);
}
INLINER uint8_t p267_first_message_offset_GET(Pack * src)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  5, 1)));
}
INLINER void p267_first_message_offset_SET(uint8_t  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  5);
}
INLINER uint8_t* p267_data__GET(Pack * src, uint8_t*  dst, int32_t pos) //logged dat
{
    uint8_t * data = src->data;
    for(int32_t BYTE = 6, dst_max = pos + 249; pos < dst_max ; pos++,  BYTE += 1)
        dst[pos] = ((get_bytes(data,  BYTE, 1)));
    return dst;
}

static const  uint32_t p267_data__LEN = 249; //return array length

INLINER  uint8_t*  p267_data__GET_(Pack * src) {return p267_data__GET(src, malloc(249 * sizeof(uint8_t)), 0);}
INLINER void p267_data__SET(uint8_t*  src, int32_t pos, Pack * dst) //logged dat
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  6, src_max = pos + 249; pos < src_max; pos++, BYTE += 1)
        set_bytes((src[pos]), 1, data,  BYTE);
}
INLINER uint16_t p268_sequence_GET(Pack * src)//sequence number (must match the one in LOGGING_DATA_ACKED
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER void p268_sequence_SET(uint16_t  src, Pack * dst)//sequence number (must match the one in LOGGING_DATA_ACKED
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER uint8_t p268_target_system_GET(Pack * src)//system ID of the targe
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 1)));
}
INLINER void p268_target_system_SET(uint8_t  src, Pack * dst)//system ID of the targe
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  2);
}
INLINER uint8_t p268_target_component_GET(Pack * src)//component ID of the targe
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  3, 1)));
}
INLINER void p268_target_component_SET(uint8_t  src, Pack * dst)//component ID of the targe
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  3);
}
INLINER uint16_t p269_resolution_h_GET(Pack * src)//Resolution horizontal in pixel
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER void p269_resolution_h_SET(uint16_t  src, Pack * dst)//Resolution horizontal in pixel
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER uint16_t p269_resolution_v_GET(Pack * src)//Resolution vertical in pixel
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 2)));
}
INLINER void p269_resolution_v_SET(uint16_t  src, Pack * dst)//Resolution vertical in pixel
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  2);
}
INLINER uint16_t p269_rotation_GET(Pack * src)//Video image rotation clockwis
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  4, 2)));
}
INLINER void p269_rotation_SET(uint16_t  src, Pack * dst)//Video image rotation clockwis
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  4);
}
INLINER uint32_t p269_bitrate_GET(Pack * src)//Bit rate in bits per secon
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  6, 4)));
}
INLINER void p269_bitrate_SET(uint32_t  src, Pack * dst)//Bit rate in bits per secon
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  6);
}
INLINER uint8_t p269_camera_id_GET(Pack * src)//Camera ID (1 for first, 2 for second, etc.
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  10, 1)));
}
INLINER void p269_camera_id_SET(uint8_t  src, Pack * dst)//Camera ID (1 for first, 2 for second, etc.
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  10);
}
INLINER uint8_t p269_status_GET(Pack * src)//Current status of video streaming (0: not running, 1: in progress
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  11, 1)));
}
INLINER void p269_status_SET(uint8_t  src, Pack * dst)//Current status of video streaming (0: not running, 1: in progress
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  11);
}
INLINER float p269_framerate_GET(Pack * src)//Frames per secon
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  12, 4)));
}
INLINER void p269_framerate_SET(float  src, Pack * dst)//Frames per secon
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  12);
}
INLINER char16_t * p269_uri_GET(Bounds_Inside * src, char16_t *  dst, int32_t pos) //Video stream UR
{
    uint8_t * data = src->base.pack->data;
    for(int32_t BYTE = src->BYTE, dst_max = pos + src->items; pos < dst_max ; pos++,  BYTE += 2)
        dst[pos] = ((get_bytes(data,  BYTE, 2)));
    return dst;
}
INLINER int32_t p269_uri_LEN(Bounds_Inside * src)
{
    return (src->base.field_bit !=  130 && !try_visit_field(src, 130)  ||  !try_visit_item(src, 0)) ? 0 : src->items;
}
INLINER  char16_t* p269_uri_TRY_(Bounds_Inside * src)
{
    if(src->base.field_bit !=  130 && !try_visit_field(src, 130)  ||  !try_visit_item(src, 0)) return NULL;
    char16_t * ret = p269_uri_GET(src, malloc((src->items + 1) * 2), 0);
    ret[src->items] = 0;//the terminating null character.
    return ret;
}
INLINER void p269_uri_SET(char16_t *  src, int32_t pos, int32_t items, Bounds_Inside * dst) //Video stream UR
{
    if(dst->base.field_bit != 130 && insert_field(dst, 130, items) ||
            ! try_visit_item(dst, 0)) insert_item(dst, 0, items);
    uint8_t * data = dst->base.pack->data;
    for(int32_t BYTE =  dst->BYTE, src_max = pos + dst->items; pos < src_max; pos++, BYTE += 2)
        set_bytes((uint16_t)(src[pos]), 2, data,  BYTE);
}
INLINER void p269_uri_SET_(char16_t*  src, Bounds_Inside * dst) {p269_uri_SET(src, 0, strlen16(src), dst);}
INLINER uint16_t p270_resolution_h_GET(Pack * src)//Resolution horizontal in pixels (set to -1 for highest resolution possible
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER void p270_resolution_h_SET(uint16_t  src, Pack * dst)//Resolution horizontal in pixels (set to -1 for highest resolution possible
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER uint16_t p270_resolution_v_GET(Pack * src)//Resolution vertical in pixels (set to -1 for highest resolution possible
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 2)));
}
INLINER void p270_resolution_v_SET(uint16_t  src, Pack * dst)//Resolution vertical in pixels (set to -1 for highest resolution possible
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  2);
}
INLINER uint16_t p270_rotation_GET(Pack * src)//Video image rotation clockwise (0-359 degrees
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  4, 2)));
}
INLINER void p270_rotation_SET(uint16_t  src, Pack * dst)//Video image rotation clockwise (0-359 degrees
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  4);
}
INLINER uint32_t p270_bitrate_GET(Pack * src)//Bit rate in bits per second (set to -1 for auto
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  6, 4)));
}
INLINER void p270_bitrate_SET(uint32_t  src, Pack * dst)//Bit rate in bits per second (set to -1 for auto
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  6);
}
INLINER uint8_t p270_target_system_GET(Pack * src)//system ID of the targe
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  10, 1)));
}
INLINER void p270_target_system_SET(uint8_t  src, Pack * dst)//system ID of the targe
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  10);
}
INLINER uint8_t p270_target_component_GET(Pack * src)//component ID of the targe
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  11, 1)));
}
INLINER void p270_target_component_SET(uint8_t  src, Pack * dst)//component ID of the targe
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  11);
}
INLINER uint8_t p270_camera_id_GET(Pack * src)//Camera ID (1 for first, 2 for second, etc.
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  12, 1)));
}
INLINER void p270_camera_id_SET(uint8_t  src, Pack * dst)//Camera ID (1 for first, 2 for second, etc.
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  12);
}
INLINER float p270_framerate_GET(Pack * src)//Frames per second (set to -1 for highest framerate possible
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  13, 4)));
}
INLINER void p270_framerate_SET(float  src, Pack * dst)//Frames per second (set to -1 for highest framerate possible
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  13);
}
INLINER char16_t * p270_uri_GET(Bounds_Inside * src, char16_t *  dst, int32_t pos) //Video stream UR
{
    uint8_t * data = src->base.pack->data;
    for(int32_t BYTE = src->BYTE, dst_max = pos + src->items; pos < dst_max ; pos++,  BYTE += 2)
        dst[pos] = ((get_bytes(data,  BYTE, 2)));
    return dst;
}
INLINER int32_t p270_uri_LEN(Bounds_Inside * src)
{
    return (src->base.field_bit !=  138 && !try_visit_field(src, 138)  ||  !try_visit_item(src, 0)) ? 0 : src->items;
}
INLINER  char16_t* p270_uri_TRY_(Bounds_Inside * src)
{
    if(src->base.field_bit !=  138 && !try_visit_field(src, 138)  ||  !try_visit_item(src, 0)) return NULL;
    char16_t * ret = p270_uri_GET(src, malloc((src->items + 1) * 2), 0);
    ret[src->items] = 0;//the terminating null character.
    return ret;
}
INLINER void p270_uri_SET(char16_t *  src, int32_t pos, int32_t items, Bounds_Inside * dst) //Video stream UR
{
    if(dst->base.field_bit != 138 && insert_field(dst, 138, items) ||
            ! try_visit_item(dst, 0)) insert_item(dst, 0, items);
    uint8_t * data = dst->base.pack->data;
    for(int32_t BYTE =  dst->BYTE, src_max = pos + dst->items; pos < src_max; pos++, BYTE += 2)
        set_bytes((uint16_t)(src[pos]), 2, data,  BYTE);
}
INLINER void p270_uri_SET_(char16_t*  src, Bounds_Inside * dst) {p270_uri_SET(src, 0, strlen16(src), dst);}
INLINER char16_t * p299_ssid_GET(Bounds_Inside * src, char16_t *  dst, int32_t pos) //Name of Wi-Fi network (SSID). Leave it blank to leave it unchanged
{
    uint8_t * data = src->base.pack->data;
    for(int32_t BYTE = src->BYTE, dst_max = pos + src->items; pos < dst_max ; pos++,  BYTE += 2)
        dst[pos] = ((get_bytes(data,  BYTE, 2)));
    return dst;
}
INLINER int32_t p299_ssid_LEN(Bounds_Inside * src)
{
    return (src->base.field_bit !=  2 && !try_visit_field(src, 2)  ||  !try_visit_item(src, 0)) ? 0 : src->items;
}
INLINER  char16_t* p299_ssid_TRY_(Bounds_Inside * src)
{
    if(src->base.field_bit !=  2 && !try_visit_field(src, 2)  ||  !try_visit_item(src, 0)) return NULL;
    char16_t * ret = p299_ssid_GET(src, malloc((src->items + 1) * 2), 0);
    ret[src->items] = 0;//the terminating null character.
    return ret;
}
INLINER void p299_ssid_SET(char16_t *  src, int32_t pos, int32_t items, Bounds_Inside * dst) //Name of Wi-Fi network (SSID). Leave it blank to leave it unchanged
{
    if(dst->base.field_bit != 2 && insert_field(dst, 2, items) ||
            ! try_visit_item(dst, 0)) insert_item(dst, 0, items);
    uint8_t * data = dst->base.pack->data;
    for(int32_t BYTE =  dst->BYTE, src_max = pos + dst->items; pos < src_max; pos++, BYTE += 2)
        set_bytes((uint16_t)(src[pos]), 2, data,  BYTE);
}
INLINER void p299_ssid_SET_(char16_t*  src, Bounds_Inside * dst) {p299_ssid_SET(src, 0, strlen16(src), dst);}
INLINER char16_t * p299_password_GET(Bounds_Inside * src, char16_t *  dst, int32_t pos) //Password. Leave it blank for an open AP
{
    uint8_t * data = src->base.pack->data;
    for(int32_t BYTE = src->BYTE, dst_max = pos + src->items; pos < dst_max ; pos++,  BYTE += 2)
        dst[pos] = ((get_bytes(data,  BYTE, 2)));
    return dst;
}
INLINER int32_t p299_password_LEN(Bounds_Inside * src)
{
    return (src->base.field_bit !=  3 && !try_visit_field(src, 3)  ||  !try_visit_item(src, 0)) ? 0 : src->items;
}
INLINER  char16_t* p299_password_TRY_(Bounds_Inside * src)
{
    if(src->base.field_bit !=  3 && !try_visit_field(src, 3)  ||  !try_visit_item(src, 0)) return NULL;
    char16_t * ret = p299_password_GET(src, malloc((src->items + 1) * 2), 0);
    ret[src->items] = 0;//the terminating null character.
    return ret;
}
INLINER void p299_password_SET(char16_t *  src, int32_t pos, int32_t items, Bounds_Inside * dst) //Password. Leave it blank for an open AP
{
    if(dst->base.field_bit != 3 && insert_field(dst, 3, items) ||
            ! try_visit_item(dst, 0)) insert_item(dst, 0, items);
    uint8_t * data = dst->base.pack->data;
    for(int32_t BYTE =  dst->BYTE, src_max = pos + dst->items; pos < src_max; pos++, BYTE += 2)
        set_bytes((uint16_t)(src[pos]), 2, data,  BYTE);
}
INLINER void p299_password_SET_(char16_t*  src, Bounds_Inside * dst) {p299_password_SET(src, 0, strlen16(src), dst);}
INLINER uint16_t p300_version_GET(Pack * src)//Currently active MAVLink version number * 100: v1.0 is 100, v2.0 is 200, etc
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER void p300_version_SET(uint16_t  src, Pack * dst)//Currently active MAVLink version number * 100: v1.0 is 100, v2.0 is 200, etc
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER uint16_t p300_min_version_GET(Pack * src)//Minimum MAVLink version supporte
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 2)));
}
INLINER void p300_min_version_SET(uint16_t  src, Pack * dst)//Minimum MAVLink version supporte
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  2);
}
INLINER uint16_t p300_max_version_GET(Pack * src)//Maximum MAVLink version supported (set to the same value as version by default
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  4, 2)));
}
INLINER void p300_max_version_SET(uint16_t  src, Pack * dst)//Maximum MAVLink version supported (set to the same value as version by default
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  4);
}
INLINER uint8_t* p300_spec_version_hash_GET(Pack * src, uint8_t*  dst, int32_t pos) //The first 8 bytes (not characters printed in hex!) of the git hash
{
    uint8_t * data = src->data;
    for(int32_t BYTE = 6, dst_max = pos + 8; pos < dst_max ; pos++,  BYTE += 1)
        dst[pos] = ((get_bytes(data,  BYTE, 1)));
    return dst;
}

static const  uint32_t p300_spec_version_hash_LEN = 8; //return array length

INLINER  uint8_t*  p300_spec_version_hash_GET_(Pack * src) {return p300_spec_version_hash_GET(src, malloc(8 * sizeof(uint8_t)), 0);}
INLINER void p300_spec_version_hash_SET(uint8_t*  src, int32_t pos, Pack * dst) //The first 8 bytes (not characters printed in hex!) of the git hash
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  6, src_max = pos + 8; pos < src_max; pos++, BYTE += 1)
        set_bytes((src[pos]), 1, data,  BYTE);
}
INLINER uint8_t* p300_library_version_hash_GET(Pack * src, uint8_t*  dst, int32_t pos) //The first 8 bytes (not characters printed in hex!) of the git hash
{
    uint8_t * data = src->data;
    for(int32_t BYTE = 14, dst_max = pos + 8; pos < dst_max ; pos++,  BYTE += 1)
        dst[pos] = ((get_bytes(data,  BYTE, 1)));
    return dst;
}

static const  uint32_t p300_library_version_hash_LEN = 8; //return array length

INLINER  uint8_t*  p300_library_version_hash_GET_(Pack * src) {return p300_library_version_hash_GET(src, malloc(8 * sizeof(uint8_t)), 0);}
INLINER void p300_library_version_hash_SET(uint8_t*  src, int32_t pos, Pack * dst) //The first 8 bytes (not characters printed in hex!) of the git hash
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  14, src_max = pos + 8; pos < src_max; pos++, BYTE += 1)
        set_bytes((src[pos]), 1, data,  BYTE);
}
INLINER uint16_t p310_vendor_specific_status_code_GET(Pack * src)//Vendor-specific status information
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER void p310_vendor_specific_status_code_SET(uint16_t  src, Pack * dst)//Vendor-specific status information
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER uint32_t p310_uptime_sec_GET(Pack * src)//The number of seconds since the start-up of the node
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 4)));
}
INLINER void p310_uptime_sec_SET(uint32_t  src, Pack * dst)//The number of seconds since the start-up of the node
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  2);
}
INLINER uint64_t p310_time_usec_GET(Pack * src)//Timestamp (microseconds since UNIX epoch or microseconds since system boot
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  6, 8)));
}
INLINER void p310_time_usec_SET(uint64_t  src, Pack * dst)//Timestamp (microseconds since UNIX epoch or microseconds since system boot
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  6);
}
INLINER uint8_t p310_sub_mode_GET(Pack * src)//Not used currently
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  14, 1)));
}
INLINER void p310_sub_mode_SET(uint8_t  src, Pack * dst)//Not used currently
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  14);
}
INLINER e_UAVCAN_NODE_HEALTH p310_health_GET(Pack * src)//Generalized node health status
{
    uint8_t * data = src->data;
    return  0 + (int)get_bits(data, 120, 3);
}
INLINER void p310_health_SET(e_UAVCAN_NODE_HEALTH  src, Pack * dst)//Generalized node health status
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 3, data, 120);
}
INLINER e_UAVCAN_NODE_MODE p310_mode_GET(Pack * src)//Generalized operating mode
{
    uint8_t * data = src->data;
    switch(get_bits(data, 123, 3))
    {
        case 0:
            return e_UAVCAN_NODE_MODE_UAVCAN_NODE_MODE_OPERATIONAL;
        case 1:
            return e_UAVCAN_NODE_MODE_UAVCAN_NODE_MODE_INITIALIZATION;
        case 2:
            return e_UAVCAN_NODE_MODE_UAVCAN_NODE_MODE_MAINTENANCE;
        case 3:
            return e_UAVCAN_NODE_MODE_UAVCAN_NODE_MODE_SOFTWARE_UPDATE;
        case 4:
            return e_UAVCAN_NODE_MODE_UAVCAN_NODE_MODE_OFFLINE;
        default: ;//assert(false);//("Unknown enum ID " + id);
    }
}
INLINER void p310_mode_SET(e_UAVCAN_NODE_MODE  src, Pack * dst)//Generalized operating mode
{
    uint8_t * data = dst->data;
    int32_t id;
    switch(src)
    {
        case e_UAVCAN_NODE_MODE_UAVCAN_NODE_MODE_OPERATIONAL:
            id = 0;
            break;
        case e_UAVCAN_NODE_MODE_UAVCAN_NODE_MODE_INITIALIZATION:
            id = 1;
            break;
        case e_UAVCAN_NODE_MODE_UAVCAN_NODE_MODE_MAINTENANCE:
            id = 2;
            break;
        case e_UAVCAN_NODE_MODE_UAVCAN_NODE_MODE_SOFTWARE_UPDATE:
            id = 3;
            break;
        case e_UAVCAN_NODE_MODE_UAVCAN_NODE_MODE_OFFLINE:
            id = 4;
            break;
        default: ;// assert(false);//("Unknown enum" + id);
    }
    set_bits(id, 3, data, 123);
}
INLINER uint32_t p311_uptime_sec_GET(Pack * src)//The number of seconds since the start-up of the node
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 4)));
}
INLINER void p311_uptime_sec_SET(uint32_t  src, Pack * dst)//The number of seconds since the start-up of the node
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  0);
}
INLINER uint32_t p311_sw_vcs_commit_GET(Pack * src)//Version control system (VCS) revision identifier (e.g. git short commit hash). Zero if unknown
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  4, 4)));
}
INLINER void p311_sw_vcs_commit_SET(uint32_t  src, Pack * dst)//Version control system (VCS) revision identifier (e.g. git short commit hash). Zero if unknown
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  4);
}
INLINER uint64_t p311_time_usec_GET(Pack * src)//Timestamp (microseconds since UNIX epoch or microseconds since system boot
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  8, 8)));
}
INLINER void p311_time_usec_SET(uint64_t  src, Pack * dst)//Timestamp (microseconds since UNIX epoch or microseconds since system boot
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  8);
}
INLINER uint8_t p311_hw_version_major_GET(Pack * src)//Hardware major version number
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  16, 1)));
}
INLINER void p311_hw_version_major_SET(uint8_t  src, Pack * dst)//Hardware major version number
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  16);
}
INLINER uint8_t p311_hw_version_minor_GET(Pack * src)//Hardware minor version number
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  17, 1)));
}
INLINER void p311_hw_version_minor_SET(uint8_t  src, Pack * dst)//Hardware minor version number
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  17);
}
INLINER uint8_t* p311_hw_unique_id_GET(Pack * src, uint8_t*  dst, int32_t pos) //Hardware unique 128-bit ID
{
    uint8_t * data = src->data;
    for(int32_t BYTE = 18, dst_max = pos + 16; pos < dst_max ; pos++,  BYTE += 1)
        dst[pos] = ((get_bytes(data,  BYTE, 1)));
    return dst;
}

static const  uint32_t p311_hw_unique_id_LEN = 16; //return array length

INLINER  uint8_t*  p311_hw_unique_id_GET_(Pack * src) {return p311_hw_unique_id_GET(src, malloc(16 * sizeof(uint8_t)), 0);}
INLINER void p311_hw_unique_id_SET(uint8_t*  src, int32_t pos, Pack * dst) //Hardware unique 128-bit ID
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  18, src_max = pos + 16; pos < src_max; pos++, BYTE += 1)
        set_bytes((src[pos]), 1, data,  BYTE);
}
INLINER uint8_t p311_sw_version_major_GET(Pack * src)//Software major version number
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  34, 1)));
}
INLINER void p311_sw_version_major_SET(uint8_t  src, Pack * dst)//Software major version number
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  34);
}
INLINER uint8_t p311_sw_version_minor_GET(Pack * src)//Software minor version number
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  35, 1)));
}
INLINER void p311_sw_version_minor_SET(uint8_t  src, Pack * dst)//Software minor version number
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  35);
}
INLINER char16_t * p311_name_GET(Bounds_Inside * src, char16_t *  dst, int32_t pos) //Node name string. For example, "sapog.px4.io"
{
    uint8_t * data = src->base.pack->data;
    for(int32_t BYTE = src->BYTE, dst_max = pos + src->items; pos < dst_max ; pos++,  BYTE += 2)
        dst[pos] = ((get_bytes(data,  BYTE, 2)));
    return dst;
}
INLINER int32_t p311_name_LEN(Bounds_Inside * src)
{
    return (src->base.field_bit !=  288 && !try_visit_field(src, 288)  ||  !try_visit_item(src, 0)) ? 0 : src->items;
}
INLINER  char16_t* p311_name_TRY_(Bounds_Inside * src)
{
    if(src->base.field_bit !=  288 && !try_visit_field(src, 288)  ||  !try_visit_item(src, 0)) return NULL;
    char16_t * ret = p311_name_GET(src, malloc((src->items + 1) * 2), 0);
    ret[src->items] = 0;//the terminating null character.
    return ret;
}
INLINER void p311_name_SET(char16_t *  src, int32_t pos, int32_t items, Bounds_Inside * dst) //Node name string. For example, "sapog.px4.io"
{
    if(dst->base.field_bit != 288 && insert_field(dst, 288, items) ||
            ! try_visit_item(dst, 0)) insert_item(dst, 0, items);
    uint8_t * data = dst->base.pack->data;
    for(int32_t BYTE =  dst->BYTE, src_max = pos + dst->items; pos < src_max; pos++, BYTE += 2)
        set_bytes((uint16_t)(src[pos]), 2, data,  BYTE);
}
INLINER void p311_name_SET_(char16_t*  src, Bounds_Inside * dst) {p311_name_SET(src, 0, strlen16(src), dst);}
INLINER uint8_t p320_target_system_GET(Pack * src)//System I
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 1)));
}
INLINER void p320_target_system_SET(uint8_t  src, Pack * dst)//System I
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  0);
}
INLINER uint8_t p320_target_component_GET(Pack * src)//Component I
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  1, 1)));
}
INLINER void p320_target_component_SET(uint8_t  src, Pack * dst)//Component I
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  1);
}
INLINER int16_t p320_param_index_GET(Pack * src)//Parameter index. Set to -1 to use the Parameter ID field as identifier (else param_id will be ignore
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  2, 2)));
}
INLINER void p320_param_index_SET(int16_t  src, Pack * dst)//Parameter index. Set to -1 to use the Parameter ID field as identifier (else param_id will be ignore
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  2);
}
INLINER char16_t * p320_param_id_GET(Bounds_Inside * src, char16_t *  dst, int32_t pos)
{
    uint8_t * data = src->base.pack->data;
    for(int32_t BYTE = src->BYTE, dst_max = pos + src->items; pos < dst_max ; pos++,  BYTE += 2)
        dst[pos] = ((get_bytes(data,  BYTE, 2)));
    return dst;
}
INLINER int32_t p320_param_id_LEN(Bounds_Inside * src)
{
    return (src->base.field_bit !=  32 && !try_visit_field(src, 32)  ||  !try_visit_item(src, 0)) ? 0 : src->items;
}
INLINER  char16_t* p320_param_id_TRY_(Bounds_Inside * src)
{
    if(src->base.field_bit !=  32 && !try_visit_field(src, 32)  ||  !try_visit_item(src, 0)) return NULL;
    char16_t * ret = p320_param_id_GET(src, malloc((src->items + 1) * 2), 0);
    ret[src->items] = 0;//the terminating null character.
    return ret;
}
INLINER void p320_param_id_SET(char16_t *  src, int32_t pos, int32_t items, Bounds_Inside * dst)
{
    if(dst->base.field_bit != 32 && insert_field(dst, 32, items) ||
            ! try_visit_item(dst, 0)) insert_item(dst, 0, items);
    uint8_t * data = dst->base.pack->data;
    for(int32_t BYTE =  dst->BYTE, src_max = pos + dst->items; pos < src_max; pos++, BYTE += 2)
        set_bytes((uint16_t)(src[pos]), 2, data,  BYTE);
}
INLINER void p320_param_id_SET_(char16_t*  src, Bounds_Inside * dst) {p320_param_id_SET(src, 0, strlen16(src), dst);}
INLINER uint8_t p321_target_system_GET(Pack * src)//System I
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 1)));
}
INLINER void p321_target_system_SET(uint8_t  src, Pack * dst)//System I
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  0);
}
INLINER uint8_t p321_target_component_GET(Pack * src)//Component I
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  1, 1)));
}
INLINER void p321_target_component_SET(uint8_t  src, Pack * dst)//Component I
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  1);
}
INLINER uint16_t p322_param_count_GET(Pack * src)//Total number of parameter
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER void p322_param_count_SET(uint16_t  src, Pack * dst)//Total number of parameter
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER uint16_t p322_param_index_GET(Pack * src)//Index of this paramete
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 2)));
}
INLINER void p322_param_index_SET(uint16_t  src, Pack * dst)//Index of this paramete
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  2);
}
INLINER e_MAV_PARAM_EXT_TYPE p322_param_type_GET(Pack * src)//Parameter type: see the MAV_PARAM_EXT_TYPE enum for supported data types
{
    uint8_t * data = src->data;
    return  1 + (int)get_bits(data, 32, 4);
}
INLINER void p322_param_type_SET(e_MAV_PARAM_EXT_TYPE  src, Pack * dst)//Parameter type: see the MAV_PARAM_EXT_TYPE enum for supported data types
{
    uint8_t * data = dst->data;
    set_bits(- 1 +   src, 4, data, 32);
}
INLINER char16_t * p322_param_id_GET(Bounds_Inside * src, char16_t *  dst, int32_t pos)
{
    uint8_t * data = src->base.pack->data;
    for(int32_t BYTE = src->BYTE, dst_max = pos + src->items; pos < dst_max ; pos++,  BYTE += 2)
        dst[pos] = ((get_bytes(data,  BYTE, 2)));
    return dst;
}
INLINER int32_t p322_param_id_LEN(Bounds_Inside * src)
{
    return (src->base.field_bit !=  38 && !try_visit_field(src, 38)  ||  !try_visit_item(src, 0)) ? 0 : src->items;
}
INLINER  char16_t* p322_param_id_TRY_(Bounds_Inside * src)
{
    if(src->base.field_bit !=  38 && !try_visit_field(src, 38)  ||  !try_visit_item(src, 0)) return NULL;
    char16_t * ret = p322_param_id_GET(src, malloc((src->items + 1) * 2), 0);
    ret[src->items] = 0;//the terminating null character.
    return ret;
}
INLINER void p322_param_id_SET(char16_t *  src, int32_t pos, int32_t items, Bounds_Inside * dst)
{
    if(dst->base.field_bit != 38 && insert_field(dst, 38, items) ||
            ! try_visit_item(dst, 0)) insert_item(dst, 0, items);
    uint8_t * data = dst->base.pack->data;
    for(int32_t BYTE =  dst->BYTE, src_max = pos + dst->items; pos < src_max; pos++, BYTE += 2)
        set_bytes((uint16_t)(src[pos]), 2, data,  BYTE);
}
INLINER void p322_param_id_SET_(char16_t*  src, Bounds_Inside * dst) {p322_param_id_SET(src, 0, strlen16(src), dst);}
INLINER char16_t * p322_param_value_GET(Bounds_Inside * src, char16_t *  dst, int32_t pos) //Parameter valu
{
    uint8_t * data = src->base.pack->data;
    for(int32_t BYTE = src->BYTE, dst_max = pos + src->items; pos < dst_max ; pos++,  BYTE += 2)
        dst[pos] = ((get_bytes(data,  BYTE, 2)));
    return dst;
}
INLINER int32_t p322_param_value_LEN(Bounds_Inside * src)
{
    return (src->base.field_bit !=  39 && !try_visit_field(src, 39)  ||  !try_visit_item(src, 0)) ? 0 : src->items;
}
INLINER  char16_t* p322_param_value_TRY_(Bounds_Inside * src)
{
    if(src->base.field_bit !=  39 && !try_visit_field(src, 39)  ||  !try_visit_item(src, 0)) return NULL;
    char16_t * ret = p322_param_value_GET(src, malloc((src->items + 1) * 2), 0);
    ret[src->items] = 0;//the terminating null character.
    return ret;
}
INLINER void p322_param_value_SET(char16_t *  src, int32_t pos, int32_t items, Bounds_Inside * dst) //Parameter valu
{
    if(dst->base.field_bit != 39 && insert_field(dst, 39, items) ||
            ! try_visit_item(dst, 0)) insert_item(dst, 0, items);
    uint8_t * data = dst->base.pack->data;
    for(int32_t BYTE =  dst->BYTE, src_max = pos + dst->items; pos < src_max; pos++, BYTE += 2)
        set_bytes((uint16_t)(src[pos]), 2, data,  BYTE);
}
INLINER void p322_param_value_SET_(char16_t*  src, Bounds_Inside * dst) {p322_param_value_SET(src, 0, strlen16(src), dst);}
INLINER uint8_t p323_target_system_GET(Pack * src)//System I
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 1)));
}
INLINER void p323_target_system_SET(uint8_t  src, Pack * dst)//System I
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  0);
}
INLINER uint8_t p323_target_component_GET(Pack * src)//Component I
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  1, 1)));
}
INLINER void p323_target_component_SET(uint8_t  src, Pack * dst)//Component I
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  1);
}
INLINER e_MAV_PARAM_EXT_TYPE p323_param_type_GET(Pack * src)//Parameter type: see the MAV_PARAM_EXT_TYPE enum for supported data types
{
    uint8_t * data = src->data;
    return  1 + (int)get_bits(data, 16, 4);
}
INLINER void p323_param_type_SET(e_MAV_PARAM_EXT_TYPE  src, Pack * dst)//Parameter type: see the MAV_PARAM_EXT_TYPE enum for supported data types
{
    uint8_t * data = dst->data;
    set_bits(- 1 +   src, 4, data, 16);
}
INLINER char16_t * p323_param_id_GET(Bounds_Inside * src, char16_t *  dst, int32_t pos)
{
    uint8_t * data = src->base.pack->data;
    for(int32_t BYTE = src->BYTE, dst_max = pos + src->items; pos < dst_max ; pos++,  BYTE += 2)
        dst[pos] = ((get_bytes(data,  BYTE, 2)));
    return dst;
}
INLINER int32_t p323_param_id_LEN(Bounds_Inside * src)
{
    return (src->base.field_bit !=  22 && !try_visit_field(src, 22)  ||  !try_visit_item(src, 0)) ? 0 : src->items;
}
INLINER  char16_t* p323_param_id_TRY_(Bounds_Inside * src)
{
    if(src->base.field_bit !=  22 && !try_visit_field(src, 22)  ||  !try_visit_item(src, 0)) return NULL;
    char16_t * ret = p323_param_id_GET(src, malloc((src->items + 1) * 2), 0);
    ret[src->items] = 0;//the terminating null character.
    return ret;
}
INLINER void p323_param_id_SET(char16_t *  src, int32_t pos, int32_t items, Bounds_Inside * dst)
{
    if(dst->base.field_bit != 22 && insert_field(dst, 22, items) ||
            ! try_visit_item(dst, 0)) insert_item(dst, 0, items);
    uint8_t * data = dst->base.pack->data;
    for(int32_t BYTE =  dst->BYTE, src_max = pos + dst->items; pos < src_max; pos++, BYTE += 2)
        set_bytes((uint16_t)(src[pos]), 2, data,  BYTE);
}
INLINER void p323_param_id_SET_(char16_t*  src, Bounds_Inside * dst) {p323_param_id_SET(src, 0, strlen16(src), dst);}
INLINER char16_t * p323_param_value_GET(Bounds_Inside * src, char16_t *  dst, int32_t pos) //Parameter valu
{
    uint8_t * data = src->base.pack->data;
    for(int32_t BYTE = src->BYTE, dst_max = pos + src->items; pos < dst_max ; pos++,  BYTE += 2)
        dst[pos] = ((get_bytes(data,  BYTE, 2)));
    return dst;
}
INLINER int32_t p323_param_value_LEN(Bounds_Inside * src)
{
    return (src->base.field_bit !=  23 && !try_visit_field(src, 23)  ||  !try_visit_item(src, 0)) ? 0 : src->items;
}
INLINER  char16_t* p323_param_value_TRY_(Bounds_Inside * src)
{
    if(src->base.field_bit !=  23 && !try_visit_field(src, 23)  ||  !try_visit_item(src, 0)) return NULL;
    char16_t * ret = p323_param_value_GET(src, malloc((src->items + 1) * 2), 0);
    ret[src->items] = 0;//the terminating null character.
    return ret;
}
INLINER void p323_param_value_SET(char16_t *  src, int32_t pos, int32_t items, Bounds_Inside * dst) //Parameter valu
{
    if(dst->base.field_bit != 23 && insert_field(dst, 23, items) ||
            ! try_visit_item(dst, 0)) insert_item(dst, 0, items);
    uint8_t * data = dst->base.pack->data;
    for(int32_t BYTE =  dst->BYTE, src_max = pos + dst->items; pos < src_max; pos++, BYTE += 2)
        set_bytes((uint16_t)(src[pos]), 2, data,  BYTE);
}
INLINER void p323_param_value_SET_(char16_t*  src, Bounds_Inside * dst) {p323_param_value_SET(src, 0, strlen16(src), dst);}
INLINER e_MAV_PARAM_EXT_TYPE p324_param_type_GET(Pack * src)//Parameter type: see the MAV_PARAM_EXT_TYPE enum for supported data types
{
    uint8_t * data = src->data;
    return  1 + (int)get_bits(data, 0, 4);
}
INLINER void p324_param_type_SET(e_MAV_PARAM_EXT_TYPE  src, Pack * dst)//Parameter type: see the MAV_PARAM_EXT_TYPE enum for supported data types
{
    uint8_t * data = dst->data;
    set_bits(- 1 +   src, 4, data, 0);
}
INLINER e_PARAM_ACK p324_param_result_GET(Pack * src)//Result code: see the PARAM_ACK enum for possible codes
{
    uint8_t * data = src->data;
    return  0 + (int)get_bits(data, 4, 3);
}
INLINER void p324_param_result_SET(e_PARAM_ACK  src, Pack * dst)//Result code: see the PARAM_ACK enum for possible codes
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 3, data, 4);
}
INLINER char16_t * p324_param_id_GET(Bounds_Inside * src, char16_t *  dst, int32_t pos)
{
    uint8_t * data = src->base.pack->data;
    for(int32_t BYTE = src->BYTE, dst_max = pos + src->items; pos < dst_max ; pos++,  BYTE += 2)
        dst[pos] = ((get_bytes(data,  BYTE, 2)));
    return dst;
}
INLINER int32_t p324_param_id_LEN(Bounds_Inside * src)
{
    return (src->base.field_bit !=  9 && !try_visit_field(src, 9)  ||  !try_visit_item(src, 0)) ? 0 : src->items;
}
INLINER  char16_t* p324_param_id_TRY_(Bounds_Inside * src)
{
    if(src->base.field_bit !=  9 && !try_visit_field(src, 9)  ||  !try_visit_item(src, 0)) return NULL;
    char16_t * ret = p324_param_id_GET(src, malloc((src->items + 1) * 2), 0);
    ret[src->items] = 0;//the terminating null character.
    return ret;
}
INLINER void p324_param_id_SET(char16_t *  src, int32_t pos, int32_t items, Bounds_Inside * dst)
{
    if(dst->base.field_bit != 9 && insert_field(dst, 9, items) ||
            ! try_visit_item(dst, 0)) insert_item(dst, 0, items);
    uint8_t * data = dst->base.pack->data;
    for(int32_t BYTE =  dst->BYTE, src_max = pos + dst->items; pos < src_max; pos++, BYTE += 2)
        set_bytes((uint16_t)(src[pos]), 2, data,  BYTE);
}
INLINER void p324_param_id_SET_(char16_t*  src, Bounds_Inside * dst) {p324_param_id_SET(src, 0, strlen16(src), dst);}
INLINER char16_t * p324_param_value_GET(Bounds_Inside * src, char16_t *  dst, int32_t pos) //Parameter value (new value if PARAM_ACK_ACCEPTED, current value otherwise
{
    uint8_t * data = src->base.pack->data;
    for(int32_t BYTE = src->BYTE, dst_max = pos + src->items; pos < dst_max ; pos++,  BYTE += 2)
        dst[pos] = ((get_bytes(data,  BYTE, 2)));
    return dst;
}
INLINER int32_t p324_param_value_LEN(Bounds_Inside * src)
{
    return (src->base.field_bit !=  10 && !try_visit_field(src, 10)  ||  !try_visit_item(src, 0)) ? 0 : src->items;
}
INLINER  char16_t* p324_param_value_TRY_(Bounds_Inside * src)
{
    if(src->base.field_bit !=  10 && !try_visit_field(src, 10)  ||  !try_visit_item(src, 0)) return NULL;
    char16_t * ret = p324_param_value_GET(src, malloc((src->items + 1) * 2), 0);
    ret[src->items] = 0;//the terminating null character.
    return ret;
}
INLINER void p324_param_value_SET(char16_t *  src, int32_t pos, int32_t items, Bounds_Inside * dst) //Parameter value (new value if PARAM_ACK_ACCEPTED, current value otherwise
{
    if(dst->base.field_bit != 10 && insert_field(dst, 10, items) ||
            ! try_visit_item(dst, 0)) insert_item(dst, 0, items);
    uint8_t * data = dst->base.pack->data;
    for(int32_t BYTE =  dst->BYTE, src_max = pos + dst->items; pos < src_max; pos++, BYTE += 2)
        set_bytes((uint16_t)(src[pos]), 2, data,  BYTE);
}
INLINER void p324_param_value_SET_(char16_t*  src, Bounds_Inside * dst) {p324_param_value_SET(src, 0, strlen16(src), dst);}
INLINER uint16_t* p330_distances_GET(Pack * src, uint16_t*  dst, int32_t pos)
{
    uint8_t * data = src->data;
    for(int32_t BYTE = 0, dst_max = pos + 72; pos < dst_max ; pos++,  BYTE += 2)
        dst[pos] = ((get_bytes(data,  BYTE, 2)));
    return dst;
}

static const  uint32_t p330_distances_LEN = 72; //return array length

INLINER  uint16_t*  p330_distances_GET_(Pack * src) {return p330_distances_GET(src, malloc(72 * sizeof(uint16_t)), 0);}
INLINER void p330_distances_SET(uint16_t*  src, int32_t pos, Pack * dst)
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  0, src_max = pos + 72; pos < src_max; pos++, BYTE += 2)
        set_bytes((src[pos]), 2, data,  BYTE);
}
INLINER uint16_t p330_min_distance_GET(Pack * src)//Minimum distance the sensor can measure in centimeter
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  144, 2)));
}
INLINER void p330_min_distance_SET(uint16_t  src, Pack * dst)//Minimum distance the sensor can measure in centimeter
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  144);
}
INLINER uint16_t p330_max_distance_GET(Pack * src)//Maximum distance the sensor can measure in centimeter
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  146, 2)));
}
INLINER void p330_max_distance_SET(uint16_t  src, Pack * dst)//Maximum distance the sensor can measure in centimeter
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  146);
}
INLINER uint64_t p330_time_usec_GET(Pack * src)//Timestamp (microseconds since system boot or since UNIX epoch
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  148, 8)));
}
INLINER void p330_time_usec_SET(uint64_t  src, Pack * dst)//Timestamp (microseconds since system boot or since UNIX epoch
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  148);
}
INLINER uint8_t p330_increment_GET(Pack * src)//Angular width in degrees of each array element
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  156, 1)));
}
INLINER void p330_increment_SET(uint8_t  src, Pack * dst)//Angular width in degrees of each array element
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  156);
}
INLINER e_MAV_DISTANCE_SENSOR p330_sensor_type_GET(Pack * src)//Class id of the distance sensor type
{
    uint8_t * data = src->data;
    return  0 + (int)get_bits(data, 1256, 3);
}
INLINER void p330_sensor_type_SET(e_MAV_DISTANCE_SENSOR  src, Pack * dst)//Class id of the distance sensor type
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 3, data, 1256);
}

