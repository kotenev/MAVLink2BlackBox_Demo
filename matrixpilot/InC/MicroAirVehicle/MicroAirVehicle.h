
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
/**
*The heartbeat message shows that a system is present and responding. The type of the MAV and Autopilot
*	hardware allow the receiving system to treat further messages from this system appropriate (e.g. by laying
*	out the user interface based on the autopilot)*/
Pack * c_CommunicationChannel_new_HEARTBEAT_0();
/**
*The general system state. If the system is following the MAVLink standard, the system state is mainly
*	defined by three orthogonal states/modes: The system mode, which is either LOCKED (motors shut down and
*	locked), MANUAL (system under RC control), GUIDED (system with autonomous position control, position
*	setpoint controlled manually) or AUTO (system guided by path/waypoint planner). The NAV_MODE defined
*	the current flight state: LIFTOFF (often an open-loop maneuver), LANDING, WAYPOINTS or VECTOR. This represents
*	the internal navigation state machine. The system status shows whether the system is currently active
*	or not and if an emergency occured. During the CRITICAL and EMERGENCY states the MAV is still considered
*	to be active, but should start emergency procedures autonomously. After a failure occured it should first
*	move from active to critical to allow manual intervention and then move to emergency after a certain
*	timeout*/
Pack * c_CommunicationChannel_new_SYS_STATUS_1();
/**
*The system time is the time of the master clock, typically the computer clock of the main onboard computer*/
Pack * c_CommunicationChannel_new_SYSTEM_TIME_2();
/**
*Reports the current commanded vehicle position, velocity, and acceleration as specified by the autopilot.
*	This should match the commands sent in SET_POSITION_TARGET_LOCAL_NED if the vehicle is being controlled
*	this way*/
Pack * c_CommunicationChannel_new_POSITION_TARGET_LOCAL_NED_3();
/**
*A ping message either requesting or responding to a ping. This allows to measure the system latencies,
*	including serial port, radio modem and UDP connections*/
Pack * c_CommunicationChannel_new_PING_4();
/**
*Request to control this MAV*/
Pack * c_CommunicationChannel_new_CHANGE_OPERATOR_CONTROL_5();
/**
*Accept / deny control of this MAV*/
Pack * c_CommunicationChannel_new_CHANGE_OPERATOR_CONTROL_ACK_6();
/**
*Emit an encrypted signature / key identifying this system. PLEASE NOTE: This protocol has been kept simple,
*	so transmitting the key requires an encrypted channel for true safety*/
Pack * c_CommunicationChannel_new_AUTH_KEY_7();
/**
*THIS INTERFACE IS DEPRECATED. USE COMMAND_LONG with MAV_CMD_DO_SET_MODE INSTEAD. Set the system mode,
*	as defined by enum MAV_MODE. There is no target component id as the mode is by definition for the overall
*	aircraft, not only for one component*/
Pack * c_CommunicationChannel_new_SET_MODE_11();
/**
*value[float]. This allows to send a parameter to any other component (such as the GCS) without the need
*	of previous knowledge of possible parameter names. Thus the same GCS can store different parameters for
*	different autopilots. See also http:qgroundcontrol.org/parameter_interface for a full documentation
*	of QGroundControl and IMU code*/
Pack * c_CommunicationChannel_new_PARAM_REQUEST_READ_20();
/**
*Request all parameters of this component. After this request, all parameters are emitted.*/
Pack * c_CommunicationChannel_new_PARAM_REQUEST_LIST_21();
/**
*Emit the value of a onboard parameter. The inclusion of param_count and param_index in the message allows
*	the recipient to keep track of received parameters and allows him to re-request missing parameters after
*	a loss or timeout*/
Pack * c_CommunicationChannel_new_PARAM_VALUE_22();
/**
*Set a parameter value TEMPORARILY to RAM. It will be reset to default on system reboot. Send the ACTION
*	MAV_ACTION_STORAGE_WRITE to PERMANENTLY write the RAM contents to EEPROM. IMPORTANT: The receiving component
*	should acknowledge the new parameter value by sending a param_value message to all communication partners.
*	This will also ensure that multiple GCS all have an up-to-date list of all parameters. If the sending
*	GCS did not receive a PARAM_VALUE message within its timeout time, it should re-send the PARAM_SET message*/
Pack * c_CommunicationChannel_new_PARAM_SET_23();
/**
*The global position, as returned by the Global Positioning System (GPS). This is
*	NOT the global position estimate of the system, but rather a RAW sensor value. See message GLOBAL_POSITION for the global position estimate. Coordinate frame is right-handed, Z-axis up (GPS frame).*/
Pack * c_CommunicationChannel_new_GPS_RAW_INT_24();
/**
*The positioning status, as reported by GPS. This message is intended to display status information about
*	each satellite visible to the receiver. See message GLOBAL_POSITION for the global position estimate.
*	This message can contain information for up to 20 satellites*/
Pack * c_CommunicationChannel_new_GPS_STATUS_25();
/**
*The RAW IMU readings for the usual 9DOF sensor setup. This message should contain the scaled values to
*	the described unit*/
Pack * c_CommunicationChannel_new_SCALED_IMU_26();
/**
*The RAW IMU readings for the usual 9DOF sensor setup. This message should always contain the true raw
*	values without any scaling to allow data capture and system debugging*/
Pack * c_CommunicationChannel_new_RAW_IMU_27();
/**
*The RAW pressure readings for the typical setup of one absolute pressure and one differential pressure
*	sensor. The sensor values should be the raw, UNSCALED ADC values*/
Pack * c_CommunicationChannel_new_RAW_PRESSURE_28();
/**
*The pressure readings for the typical setup of one absolute and differential pressure sensor. The units
*	are as specified in each field*/
Pack * c_CommunicationChannel_new_SCALED_PRESSURE_29();
/**
*The attitude in the aeronautical frame (right-handed, Z-down, X-front, Y-right).*/
Pack * c_CommunicationChannel_new_ATTITUDE_30();
/**
*The attitude in the aeronautical frame (right-handed, Z-down, X-front, Y-right), expressed as quaternion.
*	Quaternion order is w, x, y, z and a zero rotation would be expressed as (1 0 0 0)*/
Pack * c_CommunicationChannel_new_ATTITUDE_QUATERNION_31();
/**
*The filtered local position (e.g. fused computer vision and accelerometers). Coordinate frame is right-handed,
*	Z-axis down (aeronautical frame, NED / north-east-down convention*/
Pack * c_CommunicationChannel_new_LOCAL_POSITION_NED_32();
/**
*nt.*/
Pack * c_CommunicationChannel_new_GLOBAL_POSITION_INT_33();
/**
*The scaled values of the RC channels received. (-100%) -10000, (0%) 0, (100%) 10000. Channels that are
*	inactive should be set to UINT16_MAX*/
Pack * c_CommunicationChannel_new_RC_CHANNELS_SCALED_34();
/**
*The RAW values of the RC channels received. The standard PPM modulation is as follows: 1000 microseconds:
*	0%, 2000 microseconds: 100%. Individual receivers/transmitters might violate this specification*/
Pack * c_CommunicationChannel_new_RC_CHANNELS_RAW_35();
/**
*The RAW values of the servo outputs (for RC input from the remote, use the RC_CHANNELS messages). The
*	standard PPM modulation is as follows: 1000 microseconds: 0%, 2000 microseconds: 100%*/
Pack * c_CommunicationChannel_new_SERVO_OUTPUT_RAW_36();
/**
*Request a partial list of mission items from the system/component. http:qgroundcontrol.org/mavlink/waypoint_protocol.
*	If start and end index are the same, just send one waypoint*/
Pack * c_CommunicationChannel_new_MISSION_REQUEST_PARTIAL_LIST_37();
/**
*This message is sent to the MAV to write a partial list. If start index == end index, only one item will
*	be transmitted / updated. If the start index is NOT 0 and above the current list size, this request should
*	be REJECTED*/
Pack * c_CommunicationChannel_new_MISSION_WRITE_PARTIAL_LIST_38();
/**
*Message encoding a mission item. This message is emitted to announce
*	the presence of a mission item and to set a mission item on the system. The mission item can be either in x, y, z meters (type: LOCAL) or x:lat, y:lon, z:altitude. Local frame is Z-down, right handed (NED), global frame is Z-up, right handed (ENU). See also http:qgroundcontrol.org/mavlink/waypoint_protocol.*/
Pack * c_CommunicationChannel_new_MISSION_ITEM_39();
/**
*Request the information of the mission item with the sequence number seq. The response of the system to
*	this message should be a MISSION_ITEM message. http:qgroundcontrol.org/mavlink/waypoint_protoco*/
Pack * c_CommunicationChannel_new_MISSION_REQUEST_40();
/**
*Set the mission item with sequence number seq as current item. This means that the MAV will continue to
*	this mission item on the shortest path (not following the mission items in-between)*/
Pack * c_CommunicationChannel_new_MISSION_SET_CURRENT_41();
/**
*Message that announces the sequence number of the current active mission item. The MAV will fly towards
*	this mission item*/
Pack * c_CommunicationChannel_new_MISSION_CURRENT_42();
/**
*Request the overall list of mission items from the system/component.*/
Pack * c_CommunicationChannel_new_MISSION_REQUEST_LIST_43();
/**
*This message is emitted as response to MISSION_REQUEST_LIST by the MAV and to initiate a write transaction.
*	The GCS can then request the individual mission item based on the knowledge of the total number of waypoints*/
Pack * c_CommunicationChannel_new_MISSION_COUNT_44();
/**
*Delete all mission items at once.*/
Pack * c_CommunicationChannel_new_MISSION_CLEAR_ALL_45();
/**
*A certain mission item has been reached. The system will either hold this position (or circle on the orbit)
*	or (if the autocontinue on the WP was set) continue to the next waypoint*/
Pack * c_CommunicationChannel_new_MISSION_ITEM_REACHED_46();
/**
*Ack message during waypoint handling. The type field states if this message is a positive ack (type=0)
*	or if an error happened (type=non-zero)*/
Pack * c_CommunicationChannel_new_MISSION_ACK_47();
/**
*As local waypoints exist, the global waypoint reference allows to transform between the local coordinate
*	frame and the global (GPS) coordinate frame. This can be necessary when e.g. in- and outdoor settings
*	are connected and the MAV should move from in- to outdoor*/
Pack * c_CommunicationChannel_new_SET_GPS_GLOBAL_ORIGIN_48();
/**
*Once the MAV sets a new GPS-Local correspondence, this message announces the origin (0,0,0) positio*/
Pack * c_CommunicationChannel_new_GPS_GLOBAL_ORIGIN_49();
/**
*Bind a RC channel to a parameter. The parameter should change accoding to the RC channel value.*/
Pack * c_CommunicationChannel_new_PARAM_MAP_RC_50();
/**
*Request the information of the mission item with the sequence number seq. The response of the system to
*	this message should be a MISSION_ITEM_INT message. http:qgroundcontrol.org/mavlink/waypoint_protoco*/
Pack * c_CommunicationChannel_new_MISSION_REQUEST_INT_51();
/**
*Set a safety zone (volume), which is defined by two corners of a cube. This message can be used to tell
*	the MAV which setpoints/waypoints to accept and which to reject. Safety areas are often enforced by national
*	or competition regulations*/
Pack * c_CommunicationChannel_new_SAFETY_SET_ALLOWED_AREA_54();
/**
*Read out the safety zone the MAV currently assumes.*/
Pack * c_CommunicationChannel_new_SAFETY_ALLOWED_AREA_55();
/**
*The attitude in the aeronautical frame (right-handed, Z-down, X-front, Y-right), expressed as quaternion.
*	Quaternion order is w, x, y, z and a zero rotation would be expressed as (1 0 0 0)*/
Pack * c_CommunicationChannel_new_ATTITUDE_QUATERNION_COV_61();
/**
*The state of the fixed wing navigation and position controller.*/
Pack * c_CommunicationChannel_new_NAV_CONTROLLER_OUTPUT_62();
/**
*The filtered global position (e.g. fused GPS and accelerometers). The position is in GPS-frame (right-handed,
*	Z-up). It  is designed as scaled integer message since the resolution of float is not sufficient. NOTE:
*	This message is intended for onboard networks / companion computers and higher-bandwidth links and optimized
*	for accuracy and completeness. Please use the GLOBAL_POSITION_INT message for a minimal subset*/
Pack * c_CommunicationChannel_new_GLOBAL_POSITION_INT_COV_63();
/**
*The filtered local position (e.g. fused computer vision and accelerometers). Coordinate frame is right-handed,
*	Z-axis down (aeronautical frame, NED / north-east-down convention*/
Pack * c_CommunicationChannel_new_LOCAL_POSITION_NED_COV_64();
/**
*The PPM values of the RC channels received. The standard PPM modulation is as follows: 1000 microseconds:
*	0%, 2000 microseconds: 100%. Individual receivers/transmitters might violate this specification*/
Pack * c_CommunicationChannel_new_RC_CHANNELS_65();
/**
*THIS INTERFACE IS DEPRECATED. USE SET_MESSAGE_INTERVAL INSTEAD.*/
Pack * c_CommunicationChannel_new_REQUEST_DATA_STREAM_66();
/**
*THIS INTERFACE IS DEPRECATED. USE MESSAGE_INTERVAL INSTEAD.*/
Pack * c_CommunicationChannel_new_DATA_STREAM_67();
/**
*This message provides an API for manually controlling the vehicle using standard joystick axes nomenclature,
*	along with a joystick-like input device. Unused axes can be disabled an buttons are also transmit as
*	boolean values of their*/
Pack * c_CommunicationChannel_new_MANUAL_CONTROL_69();
/**
*The RAW values of the RC channels sent to the MAV to override info received from the RC radio. A value
*	of UINT16_MAX means no change to that channel. A value of 0 means control of that channel should be released
*	back to the RC radio. The standard PPM modulation is as follows: 1000 microseconds: 0%, 2000 microseconds:
*	100%. Individual receivers/transmitters might violate this specification*/
Pack * c_CommunicationChannel_new_RC_CHANNELS_OVERRIDE_70();
/**
*Message encoding a mission item. This message is emitted to announce
*	the presence of a mission item and to set a mission item on the system. The mission item can be either in x, y, z meters (type: LOCAL) or x:lat, y:lon, z:altitude. Local frame is Z-down, right handed (NED), global frame is Z-up, right handed (ENU). See alsohttp:qgroundcontrol.org/mavlink/waypoint_protocol.*/
Pack * c_CommunicationChannel_new_MISSION_ITEM_INT_73();
/**
*Metrics typically displayed on a HUD for fixed wing aircraft*/
Pack * c_CommunicationChannel_new_VFR_HUD_74();
/**
*Message encoding a command with parameters as scaled integers. Scaling depends on the actual command value*/
Pack * c_CommunicationChannel_new_COMMAND_INT_75();
/**
*Send a command with up to seven parameters to the MAV*/
Pack * c_CommunicationChannel_new_COMMAND_LONG_76();
/**
*Report status of a command. Includes feedback whether the command was executed.*/
Pack * c_CommunicationChannel_new_COMMAND_ACK_77();
/**
*Setpoint in roll, pitch, yaw and thrust from the operator*/
Pack * c_CommunicationChannel_new_MANUAL_SETPOINT_81();
/**
*Sets a desired vehicle attitude. Used by an external controller to command the vehicle (manual controller
*	or other system)*/
Pack * c_CommunicationChannel_new_SET_ATTITUDE_TARGET_82();
/**
*Reports the current commanded attitude of the vehicle as specified by the autopilot. This should match
*	the commands sent in a SET_ATTITUDE_TARGET message if the vehicle is being controlled this way*/
Pack * c_CommunicationChannel_new_ATTITUDE_TARGET_83();
/**
*Sets a desired vehicle position in a local north-east-down coordinate frame. Used by an external controller
*	to command the vehicle (manual controller or other system)*/
Pack * c_CommunicationChannel_new_SET_POSITION_TARGET_LOCAL_NED_84();
/**
*Sets a desired vehicle position, velocity, and/or acceleration in a global coordinate system (WGS84).
*	Used by an external controller to command the vehicle (manual controller or other system)*/
Pack * c_CommunicationChannel_new_SET_POSITION_TARGET_GLOBAL_INT_86();
Pack * c_CommunicationChannel_new_POSITION_TARGET_GLOBAL_INT_87();
extern void c_CommunicationChannel_on_POSITION_TARGET_GLOBAL_INT_87(Bounds_Inside * bi, Pack * pack);
/**
*The offset in X, Y, Z and yaw between the LOCAL_POSITION_NED messages of MAV X and the global coordinate
*	frame in NED coordinates. Coordinate frame is right-handed, Z-axis down (aeronautical frame, NED / north-east-down
*	convention*/
Pack * c_CommunicationChannel_new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_89();
/**
*The offset in X, Y, Z and yaw between the LOCAL_POSITION_NED messages of MAV X and the global coordinate
*	frame in NED coordinates. Coordinate frame is right-handed, Z-axis down (aeronautical frame, NED / north-east-down
*	convention*/
extern void c_CommunicationChannel_on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_89(Bounds_Inside * bi, Pack * pack);
/**
*DEPRECATED PACKET! Suffers from missing airspeed fields and singularities due to Euler angles. Please
*	use HIL_STATE_QUATERNION instead. Sent from simulation to autopilot. This packet is useful for high throughput
*	applications such as hardware in the loop simulations*/
Pack * c_CommunicationChannel_new_HIL_STATE_90();
/**
*DEPRECATED PACKET! Suffers from missing airspeed fields and singularities due to Euler angles. Please
*	use HIL_STATE_QUATERNION instead. Sent from simulation to autopilot. This packet is useful for high throughput
*	applications such as hardware in the loop simulations*/
extern void c_CommunicationChannel_on_HIL_STATE_90(Bounds_Inside * bi, Pack * pack);
/**
*Sent from autopilot to simulation. Hardware in the loop control outputs*/
Pack * c_CommunicationChannel_new_HIL_CONTROLS_91();
/**
*Sent from autopilot to simulation. Hardware in the loop control outputs*/
extern void c_CommunicationChannel_on_HIL_CONTROLS_91(Bounds_Inside * bi, Pack * pack);
/**
*Sent from simulation to autopilot. The RAW values of the RC channels received. The standard PPM modulation
*	is as follows: 1000 microseconds: 0%, 2000 microseconds: 100%. Individual receivers/transmitters might
*	violate this specification*/
Pack * c_CommunicationChannel_new_HIL_RC_INPUTS_RAW_92();
/**
*Sent from simulation to autopilot. The RAW values of the RC channels received. The standard PPM modulation
*	is as follows: 1000 microseconds: 0%, 2000 microseconds: 100%. Individual receivers/transmitters might
*	violate this specification*/
extern void c_CommunicationChannel_on_HIL_RC_INPUTS_RAW_92(Bounds_Inside * bi, Pack * pack);
/**
*Sent from autopilot to simulation. Hardware in the loop control outputs (replacement for HIL_CONTROLS*/
Pack * c_CommunicationChannel_new_HIL_ACTUATOR_CONTROLS_93();
/**
*Sent from autopilot to simulation. Hardware in the loop control outputs (replacement for HIL_CONTROLS*/
extern void c_CommunicationChannel_on_HIL_ACTUATOR_CONTROLS_93(Bounds_Inside * bi, Pack * pack);
/**
*Optical flow from a flow sensor (e.g. optical mouse sensor)*/
Pack * c_CommunicationChannel_new_OPTICAL_FLOW_100();
/**
*Optical flow from a flow sensor (e.g. optical mouse sensor)*/
extern void c_CommunicationChannel_on_OPTICAL_FLOW_100(Bounds_Inside * bi, Pack * pack);
Pack * c_CommunicationChannel_new_GLOBAL_VISION_POSITION_ESTIMATE_101();
extern void c_CommunicationChannel_on_GLOBAL_VISION_POSITION_ESTIMATE_101(Bounds_Inside * bi, Pack * pack);
Pack * c_CommunicationChannel_new_VISION_POSITION_ESTIMATE_102();
extern void c_CommunicationChannel_on_VISION_POSITION_ESTIMATE_102(Bounds_Inside * bi, Pack * pack);
Pack * c_CommunicationChannel_new_VISION_SPEED_ESTIMATE_103();
extern void c_CommunicationChannel_on_VISION_SPEED_ESTIMATE_103(Bounds_Inside * bi, Pack * pack);
Pack * c_CommunicationChannel_new_VICON_POSITION_ESTIMATE_104();
extern void c_CommunicationChannel_on_VICON_POSITION_ESTIMATE_104(Bounds_Inside * bi, Pack * pack);
/**
*The IMU readings in SI units in NED body frame*/
Pack * c_CommunicationChannel_new_HIGHRES_IMU_105();
/**
*The IMU readings in SI units in NED body frame*/
extern void c_CommunicationChannel_on_HIGHRES_IMU_105(Bounds_Inside * bi, Pack * pack);
/**
*Optical flow from an angular rate flow sensor (e.g. PX4FLOW or mouse sensor)*/
Pack * c_CommunicationChannel_new_OPTICAL_FLOW_RAD_106();
/**
*Optical flow from an angular rate flow sensor (e.g. PX4FLOW or mouse sensor)*/
extern void c_CommunicationChannel_on_OPTICAL_FLOW_RAD_106(Bounds_Inside * bi, Pack * pack);
/**
*The IMU readings in SI units in NED body frame*/
Pack * c_CommunicationChannel_new_HIL_SENSOR_107();
/**
*The IMU readings in SI units in NED body frame*/
extern void c_CommunicationChannel_on_HIL_SENSOR_107(Bounds_Inside * bi, Pack * pack);
/**
*Status of simulation environment, if used*/
Pack * c_CommunicationChannel_new_SIM_STATE_108();
/**
*Status of simulation environment, if used*/
extern void c_CommunicationChannel_on_SIM_STATE_108(Bounds_Inside * bi, Pack * pack);
/**
*Status generated by radio and injected into MAVLink stream.*/
Pack * c_CommunicationChannel_new_RADIO_STATUS_109();
/**
*Status generated by radio and injected into MAVLink stream.*/
extern void c_CommunicationChannel_on_RADIO_STATUS_109(Bounds_Inside * bi, Pack * pack);
/**
*File transfer message*/
Pack * c_CommunicationChannel_new_FILE_TRANSFER_PROTOCOL_110();
/**
*File transfer message*/
extern void c_CommunicationChannel_on_FILE_TRANSFER_PROTOCOL_110(Bounds_Inside * bi, Pack * pack);
/**
*Time synchronization message.*/
Pack * c_CommunicationChannel_new_TIMESYNC_111();
/**
*Time synchronization message.*/
extern void c_CommunicationChannel_on_TIMESYNC_111(Bounds_Inside * bi, Pack * pack);
/**
*Camera-IMU triggering and synchronisation message.*/
Pack * c_CommunicationChannel_new_CAMERA_TRIGGER_112();
/**
*Camera-IMU triggering and synchronisation message.*/
extern void c_CommunicationChannel_on_CAMERA_TRIGGER_112(Bounds_Inside * bi, Pack * pack);
/**
*The global position, as returned by the Global Positioning System (GPS). This is
*	 NOT the global position estimate of the sytem, but rather a RAW sensor value. See message GLOBAL_POSITION for the global position estimate. Coordinate frame is right-handed, Z-axis up (GPS frame).*/
Pack * c_CommunicationChannel_new_HIL_GPS_113();
/**
*The global position, as returned by the Global Positioning System (GPS). This is
*	 NOT the global position estimate of the sytem, but rather a RAW sensor value. See message GLOBAL_POSITION for the global position estimate. Coordinate frame is right-handed, Z-axis up (GPS frame).*/
extern void c_CommunicationChannel_on_HIL_GPS_113(Bounds_Inside * bi, Pack * pack);
/**
*Simulated optical flow from a flow sensor (e.g. PX4FLOW or optical mouse sensor)*/
Pack * c_CommunicationChannel_new_HIL_OPTICAL_FLOW_114();
/**
*Simulated optical flow from a flow sensor (e.g. PX4FLOW or optical mouse sensor)*/
extern void c_CommunicationChannel_on_HIL_OPTICAL_FLOW_114(Bounds_Inside * bi, Pack * pack);
/**
*Sent from simulation to autopilot, avoids in contrast to HIL_STATE singularities. This packet is useful
*	for high throughput applications such as hardware in the loop simulations*/
Pack * c_CommunicationChannel_new_HIL_STATE_QUATERNION_115();
/**
*Sent from simulation to autopilot, avoids in contrast to HIL_STATE singularities. This packet is useful
*	for high throughput applications such as hardware in the loop simulations*/
extern void c_CommunicationChannel_on_HIL_STATE_QUATERNION_115(Bounds_Inside * bi, Pack * pack);
/**
*The RAW IMU readings for secondary 9DOF sensor setup. This message should contain the scaled values to
*	the described unit*/
Pack * c_CommunicationChannel_new_SCALED_IMU2_116();
/**
*The RAW IMU readings for secondary 9DOF sensor setup. This message should contain the scaled values to
*	the described unit*/
extern void c_CommunicationChannel_on_SCALED_IMU2_116(Bounds_Inside * bi, Pack * pack);
/**
*Request a list of available logs. On some systems calling this may stop on-board logging until LOG_REQUEST_END
*	is called*/
Pack * c_CommunicationChannel_new_LOG_REQUEST_LIST_117();
/**
*Request a list of available logs. On some systems calling this may stop on-board logging until LOG_REQUEST_END
*	is called*/
extern void c_CommunicationChannel_on_LOG_REQUEST_LIST_117(Bounds_Inside * bi, Pack * pack);
/**
*Reply to LOG_REQUEST_LIST*/
Pack * c_CommunicationChannel_new_LOG_ENTRY_118();
/**
*Reply to LOG_REQUEST_LIST*/
extern void c_CommunicationChannel_on_LOG_ENTRY_118(Bounds_Inside * bi, Pack * pack);
/**
*Request a chunk of a log*/
Pack * c_CommunicationChannel_new_LOG_REQUEST_DATA_119();
/**
*Request a chunk of a log*/
extern void c_CommunicationChannel_on_LOG_REQUEST_DATA_119(Bounds_Inside * bi, Pack * pack);
/**
*Reply to LOG_REQUEST_DATA*/
Pack * c_CommunicationChannel_new_LOG_DATA_120();
/**
*Reply to LOG_REQUEST_DATA*/
extern void c_CommunicationChannel_on_LOG_DATA_120(Bounds_Inside * bi, Pack * pack);
/**
*Erase all logs*/
Pack * c_CommunicationChannel_new_LOG_ERASE_121();
/**
*Erase all logs*/
extern void c_CommunicationChannel_on_LOG_ERASE_121(Bounds_Inside * bi, Pack * pack);
/**
*Stop log transfer and resume normal logging*/
Pack * c_CommunicationChannel_new_LOG_REQUEST_END_122();
/**
*Stop log transfer and resume normal logging*/
extern void c_CommunicationChannel_on_LOG_REQUEST_END_122(Bounds_Inside * bi, Pack * pack);
/**
*data for injecting into the onboard GPS (used for DGPS)*/
Pack * c_CommunicationChannel_new_GPS_INJECT_DATA_123();
/**
*data for injecting into the onboard GPS (used for DGPS)*/
extern void c_CommunicationChannel_on_GPS_INJECT_DATA_123(Bounds_Inside * bi, Pack * pack);
/**
*Second GPS data. Coordinate frame is right-handed, Z-axis up (GPS frame).*/
Pack * c_CommunicationChannel_new_GPS2_RAW_124();
/**
*Second GPS data. Coordinate frame is right-handed, Z-axis up (GPS frame).*/
extern void c_CommunicationChannel_on_GPS2_RAW_124(Bounds_Inside * bi, Pack * pack);
/**
*Power supply status*/
Pack * c_CommunicationChannel_new_POWER_STATUS_125();
/**
*Power supply status*/
extern void c_CommunicationChannel_on_POWER_STATUS_125(Bounds_Inside * bi, Pack * pack);
/**
*Control a serial port. This can be used for raw access to an onboard serial peripheral such as a GPS or
*	telemetry radio. It is designed to make it possible to update the devices firmware via MAVLink messages
*	or change the devices settings. A message with zero bytes can be used to change just the baudrate*/
Pack * c_CommunicationChannel_new_SERIAL_CONTROL_126();
/**
*Control a serial port. This can be used for raw access to an onboard serial peripheral such as a GPS or
*	telemetry radio. It is designed to make it possible to update the devices firmware via MAVLink messages
*	or change the devices settings. A message with zero bytes can be used to change just the baudrate*/
extern void c_CommunicationChannel_on_SERIAL_CONTROL_126(Bounds_Inside * bi, Pack * pack);
/**
*RTK GPS data. Gives information on the relative baseline calculation the GPS is reporting*/
Pack * c_CommunicationChannel_new_GPS_RTK_127();
/**
*RTK GPS data. Gives information on the relative baseline calculation the GPS is reporting*/
extern void c_CommunicationChannel_on_GPS_RTK_127(Bounds_Inside * bi, Pack * pack);
/**
*RTK GPS data. Gives information on the relative baseline calculation the GPS is reporting*/
Pack * c_CommunicationChannel_new_GPS2_RTK_128();
/**
*RTK GPS data. Gives information on the relative baseline calculation the GPS is reporting*/
extern void c_CommunicationChannel_on_GPS2_RTK_128(Bounds_Inside * bi, Pack * pack);
/**
*The RAW IMU readings for 3rd 9DOF sensor setup. This message should contain the scaled values to the described
*	unit*/
Pack * c_CommunicationChannel_new_SCALED_IMU3_129();
/**
*The RAW IMU readings for 3rd 9DOF sensor setup. This message should contain the scaled values to the described
*	unit*/
extern void c_CommunicationChannel_on_SCALED_IMU3_129(Bounds_Inside * bi, Pack * pack);
Pack * c_CommunicationChannel_new_DATA_TRANSMISSION_HANDSHAKE_130();
extern void c_CommunicationChannel_on_DATA_TRANSMISSION_HANDSHAKE_130(Bounds_Inside * bi, Pack * pack);
Pack * c_CommunicationChannel_new_ENCAPSULATED_DATA_131();
extern void c_CommunicationChannel_on_ENCAPSULATED_DATA_131(Bounds_Inside * bi, Pack * pack);
Pack * c_CommunicationChannel_new_DISTANCE_SENSOR_132();
extern void c_CommunicationChannel_on_DISTANCE_SENSOR_132(Bounds_Inside * bi, Pack * pack);
/**
*Request for terrain data and terrain status*/
Pack * c_CommunicationChannel_new_TERRAIN_REQUEST_133();
/**
*Request for terrain data and terrain status*/
extern void c_CommunicationChannel_on_TERRAIN_REQUEST_133(Bounds_Inside * bi, Pack * pack);
/**
*Terrain data sent from GCS. The lat/lon and grid_spacing must be the same as a lat/lon from a TERRAIN_REQUES*/
Pack * c_CommunicationChannel_new_TERRAIN_DATA_134();
/**
*Terrain data sent from GCS. The lat/lon and grid_spacing must be the same as a lat/lon from a TERRAIN_REQUES*/
extern void c_CommunicationChannel_on_TERRAIN_DATA_134(Bounds_Inside * bi, Pack * pack);
/**
*Request that the vehicle report terrain height at the given location. Used by GCS to check if vehicle
*	has all terrain data needed for a mission*/
Pack * c_CommunicationChannel_new_TERRAIN_CHECK_135();
/**
*Request that the vehicle report terrain height at the given location. Used by GCS to check if vehicle
*	has all terrain data needed for a mission*/
extern void c_CommunicationChannel_on_TERRAIN_CHECK_135(Bounds_Inside * bi, Pack * pack);
/**
*Response from a TERRAIN_CHECK request*/
Pack * c_CommunicationChannel_new_TERRAIN_REPORT_136();
/**
*Response from a TERRAIN_CHECK request*/
extern void c_CommunicationChannel_on_TERRAIN_REPORT_136(Bounds_Inside * bi, Pack * pack);
/**
*Barometer readings for 2nd barometer*/
Pack * c_CommunicationChannel_new_SCALED_PRESSURE2_137();
/**
*Barometer readings for 2nd barometer*/
extern void c_CommunicationChannel_on_SCALED_PRESSURE2_137(Bounds_Inside * bi, Pack * pack);
/**
*Motion capture attitude and position*/
Pack * c_CommunicationChannel_new_ATT_POS_MOCAP_138();
/**
*Motion capture attitude and position*/
extern void c_CommunicationChannel_on_ATT_POS_MOCAP_138(Bounds_Inside * bi, Pack * pack);
/**
*Set the vehicle attitude and body angular rates.*/
Pack * c_CommunicationChannel_new_SET_ACTUATOR_CONTROL_TARGET_139();
/**
*Set the vehicle attitude and body angular rates.*/
extern void c_CommunicationChannel_on_SET_ACTUATOR_CONTROL_TARGET_139(Bounds_Inside * bi, Pack * pack);
/**
*Set the vehicle attitude and body angular rates.*/
Pack * c_CommunicationChannel_new_ACTUATOR_CONTROL_TARGET_140();
/**
*Set the vehicle attitude and body angular rates.*/
extern void c_CommunicationChannel_on_ACTUATOR_CONTROL_TARGET_140(Bounds_Inside * bi, Pack * pack);
/**
*The current system altitude.*/
Pack * c_CommunicationChannel_new_ALTITUDE_141();
/**
*The current system altitude.*/
extern void c_CommunicationChannel_on_ALTITUDE_141(Bounds_Inside * bi, Pack * pack);
/**
*The autopilot is requesting a resource (file, binary, other type of data)*/
Pack * c_CommunicationChannel_new_RESOURCE_REQUEST_142();
/**
*The autopilot is requesting a resource (file, binary, other type of data)*/
extern void c_CommunicationChannel_on_RESOURCE_REQUEST_142(Bounds_Inside * bi, Pack * pack);
/**
*Barometer readings for 3rd barometer*/
Pack * c_CommunicationChannel_new_SCALED_PRESSURE3_143();
/**
*Barometer readings for 3rd barometer*/
extern void c_CommunicationChannel_on_SCALED_PRESSURE3_143(Bounds_Inside * bi, Pack * pack);
/**
*current motion information from a designated system*/
Pack * c_CommunicationChannel_new_FOLLOW_TARGET_144();
/**
*current motion information from a designated system*/
extern void c_CommunicationChannel_on_FOLLOW_TARGET_144(Bounds_Inside * bi, Pack * pack);
/**
*The smoothed, monotonic system state used to feed the control loops of the system.*/
Pack * c_CommunicationChannel_new_CONTROL_SYSTEM_STATE_146();
/**
*The smoothed, monotonic system state used to feed the control loops of the system.*/
extern void c_CommunicationChannel_on_CONTROL_SYSTEM_STATE_146(Bounds_Inside * bi, Pack * pack);
/**
*Battery information*/
Pack * c_CommunicationChannel_new_BATTERY_STATUS_147();
/**
*Battery information*/
extern void c_CommunicationChannel_on_BATTERY_STATUS_147(Bounds_Inside * bi, Pack * pack);
/**
*Version and capability of autopilot software*/
Pack * c_CommunicationChannel_new_AUTOPILOT_VERSION_148();
/**
*Version and capability of autopilot software*/
extern void c_CommunicationChannel_on_AUTOPILOT_VERSION_148(Bounds_Inside * bi, Pack * pack);
/**
*The location of a landing area captured from a downward facing camera*/
Pack * c_CommunicationChannel_new_LANDING_TARGET_149();
/**
*The location of a landing area captured from a downward facing camera*/
extern void c_CommunicationChannel_on_LANDING_TARGET_149(Bounds_Inside * bi, Pack * pack);
/**
*Depreciated but used as a compiler flag.  Do not remove*/
extern void c_CommunicationChannel_on_FLEXIFUNCTION_SET_150(Bounds_Inside * bi, Pack * pack);
/**
*Reqest reading of flexifunction data*/
extern void c_CommunicationChannel_on_FLEXIFUNCTION_READ_REQ_151(Bounds_Inside * bi, Pack * pack);
/**
*Flexifunction type and parameters for component at function index from buffer*/
extern void c_CommunicationChannel_on_FLEXIFUNCTION_BUFFER_FUNCTION_152(Bounds_Inside * bi, Pack * pack);
/**
*Flexifunction type and parameters for component at function index from buffer*/
extern void c_CommunicationChannel_on_FLEXIFUNCTION_BUFFER_FUNCTION_ACK_153(Bounds_Inside * bi, Pack * pack);
/**
*Acknowldge sucess or failure of a flexifunction command*/
extern void c_CommunicationChannel_on_FLEXIFUNCTION_DIRECTORY_155(Bounds_Inside * bi, Pack * pack);
/**
*Acknowldge sucess or failure of a flexifunction command*/
extern void c_CommunicationChannel_on_FLEXIFUNCTION_DIRECTORY_ACK_156(Bounds_Inside * bi, Pack * pack);
/**
*Acknowldge sucess or failure of a flexifunction command*/
extern void c_CommunicationChannel_on_FLEXIFUNCTION_COMMAND_157(Bounds_Inside * bi, Pack * pack);
/**
*Acknowldge sucess or failure of a flexifunction command*/
extern void c_CommunicationChannel_on_FLEXIFUNCTION_COMMAND_ACK_158(Bounds_Inside * bi, Pack * pack);
/**
*Backwards compatible MAVLink version of SERIAL_UDB_EXTRA - F2: Format Part A*/
extern void c_CommunicationChannel_on_SERIAL_UDB_EXTRA_F2_A_170(Bounds_Inside * bi, Pack * pack);
/**
*Backwards compatible version of SERIAL_UDB_EXTRA - F2: Part B*/
extern void c_CommunicationChannel_on_SERIAL_UDB_EXTRA_F2_B_171(Bounds_Inside * bi, Pack * pack);
/**
*Backwards compatible version of SERIAL_UDB_EXTRA F4: format*/
extern void c_CommunicationChannel_on_SERIAL_UDB_EXTRA_F4_172(Bounds_Inside * bi, Pack * pack);
/**
*Backwards compatible version of SERIAL_UDB_EXTRA F5: format*/
extern void c_CommunicationChannel_on_SERIAL_UDB_EXTRA_F5_173(Bounds_Inside * bi, Pack * pack);
/**
*Backwards compatible version of SERIAL_UDB_EXTRA F6: format*/
extern void c_CommunicationChannel_on_SERIAL_UDB_EXTRA_F6_174(Bounds_Inside * bi, Pack * pack);
/**
*Backwards compatible version of SERIAL_UDB_EXTRA F7: format*/
extern void c_CommunicationChannel_on_SERIAL_UDB_EXTRA_F7_175(Bounds_Inside * bi, Pack * pack);
/**
*Backwards compatible version of SERIAL_UDB_EXTRA F8: format*/
extern void c_CommunicationChannel_on_SERIAL_UDB_EXTRA_F8_176(Bounds_Inside * bi, Pack * pack);
/**
*Backwards compatible version of SERIAL_UDB_EXTRA F13: format*/
extern void c_CommunicationChannel_on_SERIAL_UDB_EXTRA_F13_177(Bounds_Inside * bi, Pack * pack);
/**
*Backwards compatible version of SERIAL_UDB_EXTRA F14: format*/
extern void c_CommunicationChannel_on_SERIAL_UDB_EXTRA_F14_178(Bounds_Inside * bi, Pack * pack);
/**
*Backwards compatible version of SERIAL_UDB_EXTRA F15 format*/
extern void c_CommunicationChannel_on_SERIAL_UDB_EXTRA_F15_179(Bounds_Inside * bi, Pack * pack);
/**
*Backwards compatible version of SERIAL_UDB_EXTRA F16 format*/
extern void c_CommunicationChannel_on_SERIAL_UDB_EXTRA_F16_180(Bounds_Inside * bi, Pack * pack);
/**
*The altitude measured by sensors and IMU*/
extern void c_CommunicationChannel_on_ALTITUDES_181(Bounds_Inside * bi, Pack * pack);
/**
*The airspeed measured by sensors and IMU*/
extern void c_CommunicationChannel_on_AIRSPEEDS_182(Bounds_Inside * bi, Pack * pack);
/**
*Backwards compatible version of SERIAL_UDB_EXTRA F17 format*/
extern void c_CommunicationChannel_on_SERIAL_UDB_EXTRA_F17_183(Bounds_Inside * bi, Pack * pack);
/**
*Backwards compatible version of SERIAL_UDB_EXTRA F18 format*/
extern void c_CommunicationChannel_on_SERIAL_UDB_EXTRA_F18_184(Bounds_Inside * bi, Pack * pack);
/**
*Backwards compatible version of SERIAL_UDB_EXTRA F19 format*/
extern void c_CommunicationChannel_on_SERIAL_UDB_EXTRA_F19_185(Bounds_Inside * bi, Pack * pack);
/**
*Backwards compatible version of SERIAL_UDB_EXTRA F20 format*/
extern void c_CommunicationChannel_on_SERIAL_UDB_EXTRA_F20_186(Bounds_Inside * bi, Pack * pack);
/**
*Backwards compatible version of SERIAL_UDB_EXTRA F21 format*/
extern void c_CommunicationChannel_on_SERIAL_UDB_EXTRA_F21_187(Bounds_Inside * bi, Pack * pack);
/**
*Backwards compatible version of SERIAL_UDB_EXTRA F22 format*/
extern void c_CommunicationChannel_on_SERIAL_UDB_EXTRA_F22_188(Bounds_Inside * bi, Pack * pack);
/**
*Estimator status message including flags, innovation test ratios and estimated accuracies. The flags message
*	is an integer bitmask containing information on which EKF outputs are valid. See the ESTIMATOR_STATUS_FLAGS
*	enum definition for further information. The innovaton test ratios show the magnitude of the sensor innovation
*	divided by the innovation check threshold. Under normal operation the innovaton test ratios should be
*	below 0.5 with occasional values up to 1.0. Values greater than 1.0 should be rare under normal operation
*	and indicate that a measurement has been rejected by the filter. The user should be notified if an innovation
*	test ratio greater than 1.0 is recorded. Notifications for values in the range between 0.5 and 1.0 should
*	be optional and controllable by the user*/
Pack * c_CommunicationChannel_new_ESTIMATOR_STATUS_230();
/**
*Estimator status message including flags, innovation test ratios and estimated accuracies. The flags message
*	is an integer bitmask containing information on which EKF outputs are valid. See the ESTIMATOR_STATUS_FLAGS
*	enum definition for further information. The innovaton test ratios show the magnitude of the sensor innovation
*	divided by the innovation check threshold. Under normal operation the innovaton test ratios should be
*	below 0.5 with occasional values up to 1.0. Values greater than 1.0 should be rare under normal operation
*	and indicate that a measurement has been rejected by the filter. The user should be notified if an innovation
*	test ratio greater than 1.0 is recorded. Notifications for values in the range between 0.5 and 1.0 should
*	be optional and controllable by the user*/
extern void c_CommunicationChannel_on_ESTIMATOR_STATUS_230(Bounds_Inside * bi, Pack * pack);
Pack * c_CommunicationChannel_new_WIND_COV_231();
extern void c_CommunicationChannel_on_WIND_COV_231(Bounds_Inside * bi, Pack * pack);
/**
*GPS sensor input message.  This is a raw sensor value sent by the GPS. This is NOT the global position
*	estimate of the sytem*/
Pack * c_CommunicationChannel_new_GPS_INPUT_232();
/**
*GPS sensor input message.  This is a raw sensor value sent by the GPS. This is NOT the global position
*	estimate of the sytem*/
extern void c_CommunicationChannel_on_GPS_INPUT_232(Bounds_Inside * bi, Pack * pack);
/**
*RTCM message for injecting into the onboard GPS (used for DGPS)*/
Pack * c_CommunicationChannel_new_GPS_RTCM_DATA_233();
/**
*RTCM message for injecting into the onboard GPS (used for DGPS)*/
extern void c_CommunicationChannel_on_GPS_RTCM_DATA_233(Bounds_Inside * bi, Pack * pack);
/**
*Message appropriate for high latency connections like Iridium*/
Pack * c_CommunicationChannel_new_HIGH_LATENCY_234();
/**
*Message appropriate for high latency connections like Iridium*/
extern void c_CommunicationChannel_on_HIGH_LATENCY_234(Bounds_Inside * bi, Pack * pack);
/**
*Vibration levels and accelerometer clipping*/
Pack * c_CommunicationChannel_new_VIBRATION_241();
/**
*Vibration levels and accelerometer clipping*/
extern void c_CommunicationChannel_on_VIBRATION_241(Bounds_Inside * bi, Pack * pack);
/**
*This message can be requested by sending the MAV_CMD_GET_HOME_POSITION command. The position the system
*	will return to and land on. The position is set automatically by the system during the takeoff in case
*	it was not explicitely set by the operator before or after. The position the system will return to and
*	land on. The global and local positions encode the position in the respective coordinate frames, while
*	the q parameter encodes the orientation of the surface. Under normal conditions it describes the heading
*	and terrain slope, which can be used by the aircraft to adjust the approach. The approach 3D vector describes
*	the point to which the system should fly in normal flight mode and then perform a landing sequence along
*	the vector*/
Pack * c_CommunicationChannel_new_HOME_POSITION_242();
/**
*This message can be requested by sending the MAV_CMD_GET_HOME_POSITION command. The position the system
*	will return to and land on. The position is set automatically by the system during the takeoff in case
*	it was not explicitely set by the operator before or after. The position the system will return to and
*	land on. The global and local positions encode the position in the respective coordinate frames, while
*	the q parameter encodes the orientation of the surface. Under normal conditions it describes the heading
*	and terrain slope, which can be used by the aircraft to adjust the approach. The approach 3D vector describes
*	the point to which the system should fly in normal flight mode and then perform a landing sequence along
*	the vector*/
extern void c_CommunicationChannel_on_HOME_POSITION_242(Bounds_Inside * bi, Pack * pack);
/**
*The position the system will return to and land on. The position is set automatically by the system during
*	the takeoff in case it was not explicitely set by the operator before or after. The global and local
*	positions encode the position in the respective coordinate frames, while the q parameter encodes the
*	orientation of the surface. Under normal conditions it describes the heading and terrain slope, which
*	can be used by the aircraft to adjust the approach. The approach 3D vector describes the point to which
*	the system should fly in normal flight mode and then perform a landing sequence along the vector*/
Pack * c_CommunicationChannel_new_SET_HOME_POSITION_243();
/**
*The position the system will return to and land on. The position is set automatically by the system during
*	the takeoff in case it was not explicitely set by the operator before or after. The global and local
*	positions encode the position in the respective coordinate frames, while the q parameter encodes the
*	orientation of the surface. Under normal conditions it describes the heading and terrain slope, which
*	can be used by the aircraft to adjust the approach. The approach 3D vector describes the point to which
*	the system should fly in normal flight mode and then perform a landing sequence along the vector*/
extern void c_CommunicationChannel_on_SET_HOME_POSITION_243(Bounds_Inside * bi, Pack * pack);
extern void c_CommunicationChannel_on_MESSAGE_INTERVAL_244(Bounds_Inside * bi, Pack * pack);
/**
*Provides state for additional features*/
extern void c_CommunicationChannel_on_EXTENDED_SYS_STATE_245(Bounds_Inside * bi, Pack * pack);
/**
*The location and information of an ADSB vehicle*/
extern void c_CommunicationChannel_on_ADSB_VEHICLE_246(Bounds_Inside * bi, Pack * pack);
/**
*Information about a potential collision*/
extern void c_CommunicationChannel_on_COLLISION_247(Bounds_Inside * bi, Pack * pack);
/**
*Message implementing parts of the V2 payload specs in V1 frames for transitional support.*/
extern void c_CommunicationChannel_on_V2_EXTENSION_248(Bounds_Inside * bi, Pack * pack);
/**
*Send raw controller memory. The use of this message is discouraged for normal packets, but a quite efficient
*	way for testing new messages and getting experimental debug output*/
extern void c_CommunicationChannel_on_MEMORY_VECT_249(Bounds_Inside * bi, Pack * pack);
extern void c_CommunicationChannel_on_DEBUG_VECT_250(Bounds_Inside * bi, Pack * pack);
/**
*Send a key-value pair as float. The use of this message is discouraged for normal packets, but a quite
*	efficient way for testing new messages and getting experimental debug output*/
extern void c_CommunicationChannel_on_NAMED_VALUE_FLOAT_251(Bounds_Inside * bi, Pack * pack);
/**
*Send a key-value pair as integer. The use of this message is discouraged for normal packets, but a quite
*	efficient way for testing new messages and getting experimental debug output*/
extern void c_CommunicationChannel_on_NAMED_VALUE_INT_252(Bounds_Inside * bi, Pack * pack);
/**
*Status text message. These messages are printed in yellow in the COMM console of QGroundControl. WARNING:
*	They consume quite some bandwidth, so use only for important status and error messages. If implemented
*	wisely, these messages are buffered on the MCU and sent only at a limited rate (e.g. 10 Hz)*/
extern void c_CommunicationChannel_on_STATUSTEXT_253(Bounds_Inside * bi, Pack * pack);
/**
*Send a debug value. The index is used to discriminate between values. These values show up in the plot
*	of QGroundControl as DEBUG N*/
extern void c_CommunicationChannel_on_DEBUG_254(Bounds_Inside * bi, Pack * pack);
/**
*Setup a MAVLink2 signing key. If called with secret_key of all zero and zero initial_timestamp will disable
*	signin*/
extern void c_CommunicationChannel_on_SETUP_SIGNING_256(Bounds_Inside * bi, Pack * pack);
/**
*Report button state change*/
extern void c_CommunicationChannel_on_BUTTON_CHANGE_257(Bounds_Inside * bi, Pack * pack);
/**
*Control vehicle tone generation (buzzer)*/
extern void c_CommunicationChannel_on_PLAY_TUNE_258(Bounds_Inside * bi, Pack * pack);
/**
*WIP: Information about a camera*/
extern void c_CommunicationChannel_on_CAMERA_INFORMATION_259(Bounds_Inside * bi, Pack * pack);
/**
*WIP: Settings of a camera, can be requested using MAV_CMD_REQUEST_CAMERA_SETTINGS.*/
extern void c_CommunicationChannel_on_CAMERA_SETTINGS_260(Bounds_Inside * bi, Pack * pack);
/**
*WIP: Information about a storage medium.*/
extern void c_CommunicationChannel_on_STORAGE_INFORMATION_261(Bounds_Inside * bi, Pack * pack);
/**
*WIP: Information about the status of a capture*/
extern void c_CommunicationChannel_on_CAMERA_CAPTURE_STATUS_262(Bounds_Inside * bi, Pack * pack);
/**
*Information about a captured image*/
extern void c_CommunicationChannel_on_CAMERA_IMAGE_CAPTURED_263(Bounds_Inside * bi, Pack * pack);
/**
*WIP: Information about flight since last arming*/
extern void c_CommunicationChannel_on_FLIGHT_INFORMATION_264(Bounds_Inside * bi, Pack * pack);
/**
*WIP: Orientation of a mount*/
extern void c_CommunicationChannel_on_MOUNT_ORIENTATION_265(Bounds_Inside * bi, Pack * pack);
/**
*A message containing logged data (see also MAV_CMD_LOGGING_START)*/
extern void c_CommunicationChannel_on_LOGGING_DATA_266(Bounds_Inside * bi, Pack * pack);
/**
*A message containing logged data which requires a LOGGING_ACK to be sent back*/
extern void c_CommunicationChannel_on_LOGGING_DATA_ACKED_267(Bounds_Inside * bi, Pack * pack);
/**
*An ack for a LOGGING_DATA_ACKED message*/
extern void c_CommunicationChannel_on_LOGGING_ACK_268(Bounds_Inside * bi, Pack * pack);
/**
*WIP: Information about video stream*/
extern void c_CommunicationChannel_on_VIDEO_STREAM_INFORMATION_269(Bounds_Inside * bi, Pack * pack);
/**
*WIP: Message that sets video stream settings*/
extern void c_CommunicationChannel_on_SET_VIDEO_STREAM_SETTINGS_270(Bounds_Inside * bi, Pack * pack);
/**
*Configure AP SSID and Password.*/
extern void c_CommunicationChannel_on_WIFI_CONFIG_AP_299(Bounds_Inside * bi, Pack * pack);
/**
*WIP: Version and capability of protocol version. This message is the response to REQUEST_PROTOCOL_VERSION
*	and is used as part of the handshaking to establish which MAVLink version should be used on the network.
*	Every node should respond to REQUEST_PROTOCOL_VERSION to enable the handshaking. Library implementers
*	should consider adding this into the default decoding state machine to allow the protocol core to respond
*	directly*/
extern void c_CommunicationChannel_on_PROTOCOL_VERSION_300(Bounds_Inside * bi, Pack * pack);
/**
*General status information of an UAVCAN node. Please refer to the definition of the UAVCAN message "uavcan.protocol.NodeStatus"
*	for the background information. The UAVCAN specification is available at http:uavcan.org*/
extern void c_CommunicationChannel_on_UAVCAN_NODE_STATUS_310(Bounds_Inside * bi, Pack * pack);
/**
*General information describing a particular UAVCAN node. Please refer to the definition of the UAVCAN
*	service "uavcan.protocol.GetNodeInfo" for the background information. This message should be emitted
*	by the system whenever a new node appears online, or an existing node reboots. Additionally, it can be
*	emitted upon request from the other end of the MAVLink channel (see MAV_CMD_UAVCAN_GET_NODE_INFO). It
*	is also not prohibited to emit this message unconditionally at a low frequency. The UAVCAN specification
*	is available at http:uavcan.org*/
extern void c_CommunicationChannel_on_UAVCAN_NODE_INFO_311(Bounds_Inside * bi, Pack * pack);
/**
*Request to read the value of a parameter with the either the param_id string id or param_index.*/
extern void c_CommunicationChannel_on_PARAM_EXT_REQUEST_READ_320(Bounds_Inside * bi, Pack * pack);
/**
*Request all parameters of this component. After this request, all parameters are emitted.*/
extern void c_CommunicationChannel_on_PARAM_EXT_REQUEST_LIST_321(Bounds_Inside * bi, Pack * pack);
/**
*Emit the value of a parameter. The inclusion of param_count and param_index in the message allows the
*	recipient to keep track of received parameters and allows them to re-request missing parameters after
*	a loss or timeout*/
extern void c_CommunicationChannel_on_PARAM_EXT_VALUE_322(Bounds_Inside * bi, Pack * pack);
/**
*Set a parameter value. In order to deal with message loss (and retransmission of PARAM_EXT_SET), when
*	setting a parameter value and the new value is the same as the current value, you will immediately get
*	a PARAM_ACK_ACCEPTED response. If the current state is PARAM_ACK_IN_PROGRESS, you will accordingly receive
*	a PARAM_ACK_IN_PROGRESS in response*/
extern void c_CommunicationChannel_on_PARAM_EXT_SET_323(Bounds_Inside * bi, Pack * pack);
/**
*Response from a PARAM_EXT_SET message.*/
extern void c_CommunicationChannel_on_PARAM_EXT_ACK_324(Bounds_Inside * bi, Pack * pack);
/**
*Obstacle distances in front of the sensor, starting from the left in increment degrees to the right*/
extern void c_CommunicationChannel_on_OBSTACLE_DISTANCE_330(Bounds_Inside * bi, Pack * pack);

extern Channel c_CommunicationChannel;
/*Push pack to the channel send out buffer*/
INLINER void c_CommunicationChannel_send(Pack * pack) {c_CommunicationChannel.process(pack, PROCESS_HOST_REQEST);}
/*Getting in the channel sending out buffer packs bytes*/
INLINER int32_t  c_CommunicationChannel_input_bytes(uint8_t* dst, int32_t bytes) { return input_bytes_adv(&c_CommunicationChannel, dst, bytes);}

/*Push received packs bytes to the channel*/
INLINER void  c_CommunicationChannel_output_bytes(uint8_t* src, int32_t bytes) {  output_bytes_adv(&c_CommunicationChannel, src, bytes);} //pass received from a network bytes to the channel
/*Dispatch received packs to handlers*/
INLINER void c_CommunicationChannel_process_received() { c_CommunicationChannel.process(NULL, PROCESS_RECEIVED_PACKS);}//dispatch received packs to its handlers


typedef  enum
{
    e_MAV_TYPE_MAV_TYPE_GENERIC = 0, //Generic micro air vehicle.
    e_MAV_TYPE_MAV_TYPE_FIXED_WING = 1, //Fixed wing aircraft.
    e_MAV_TYPE_MAV_TYPE_QUADROTOR = 2, //Quadrotor
    e_MAV_TYPE_MAV_TYPE_COAXIAL = 3, //Coaxial helicopter
    e_MAV_TYPE_MAV_TYPE_HELICOPTER = 4, //Normal helicopter with tail rotor.
    e_MAV_TYPE_MAV_TYPE_ANTENNA_TRACKER = 5, //Ground installation
    e_MAV_TYPE_MAV_TYPE_GCS = 6, //Operator control unit / ground control station
    e_MAV_TYPE_MAV_TYPE_AIRSHIP = 7, //Airship, controlled
    e_MAV_TYPE_MAV_TYPE_FREE_BALLOON = 8, //Free balloon, uncontrolled
    e_MAV_TYPE_MAV_TYPE_ROCKET = 9, //Rocket
    e_MAV_TYPE_MAV_TYPE_GROUND_ROVER = 10, //Ground rover
    e_MAV_TYPE_MAV_TYPE_SURFACE_BOAT = 11, //Surface vessel, boat, ship
    e_MAV_TYPE_MAV_TYPE_SUBMARINE = 12, //Submarine
    e_MAV_TYPE_MAV_TYPE_HEXAROTOR = 13, //Hexarotor
    e_MAV_TYPE_MAV_TYPE_OCTOROTOR = 14, //Octorotor
    e_MAV_TYPE_MAV_TYPE_TRICOPTER = 15, //Tricopter
    e_MAV_TYPE_MAV_TYPE_FLAPPING_WING = 16, //Flapping wing
    e_MAV_TYPE_MAV_TYPE_KITE = 17, //Kite
    e_MAV_TYPE_MAV_TYPE_ONBOARD_CONTROLLER = 18, //Onboard companion controller
    e_MAV_TYPE_MAV_TYPE_VTOL_DUOROTOR = 19, //Two-rotor VTOL using control surfaces in vertical operation in addition. Tailsitter.
    e_MAV_TYPE_MAV_TYPE_VTOL_QUADROTOR = 20, //Quad-rotor VTOL using a V-shaped quad config in vertical operation. Tailsitter.
    e_MAV_TYPE_MAV_TYPE_VTOL_TILTROTOR = 21, //Tiltrotor VTOL
    e_MAV_TYPE_MAV_TYPE_VTOL_RESERVED2 = 22, //VTOL reserved 2
    e_MAV_TYPE_MAV_TYPE_VTOL_RESERVED3 = 23, //VTOL reserved 3
    e_MAV_TYPE_MAV_TYPE_VTOL_RESERVED4 = 24, //VTOL reserved 4
    e_MAV_TYPE_MAV_TYPE_VTOL_RESERVED5 = 25, //VTOL reserved 5
    e_MAV_TYPE_MAV_TYPE_GIMBAL = 26, //Onboard gimbal
    e_MAV_TYPE_MAV_TYPE_ADSB = 27, //Onboard ADSB peripheral
    e_MAV_TYPE_MAV_TYPE_PARAFOIL = 28 //Steerable, nonrigid airfoil
} e_MAV_TYPE;

/**
*Micro air vehicle / autopilot classes. This identifies the individual model.*/
typedef  enum
{
    e_MAV_AUTOPILOT_MAV_AUTOPILOT_GENERIC = 0, //Generic autopilot, full support for everything
    e_MAV_AUTOPILOT_MAV_AUTOPILOT_RESERVED = 1, //Reserved for future use.
    e_MAV_AUTOPILOT_MAV_AUTOPILOT_SLUGS = 2, //SLUGS autopilot, http:slugsuav.soe.ucsc.edu
    e_MAV_AUTOPILOT_MAV_AUTOPILOT_ARDUPILOTMEGA = 3, //ArduPilotMega / ArduCopter, http:diydrones.com
    e_MAV_AUTOPILOT_MAV_AUTOPILOT_OPENPILOT = 4, //OpenPilot, http:openpilot.org
    e_MAV_AUTOPILOT_MAV_AUTOPILOT_GENERIC_WAYPOINTS_ONLY = 5, //Generic autopilot only supporting simple waypoints
    e_MAV_AUTOPILOT_MAV_AUTOPILOT_GENERIC_WAYPOINTS_AND_SIMPLE_NAVIGATION_ONLY = 6, //Generic autopilot supporting waypoints and other simple navigation commands
    e_MAV_AUTOPILOT_MAV_AUTOPILOT_GENERIC_MISSION_FULL = 7, //Generic autopilot supporting the full mission command set
    e_MAV_AUTOPILOT_MAV_AUTOPILOT_INVALID = 8, //No valid autopilot, e.g. a GCS or other MAVLink component
    e_MAV_AUTOPILOT_MAV_AUTOPILOT_PPZ = 9, //PPZ UAV - http:nongnu.org/paparazzi
    e_MAV_AUTOPILOT_MAV_AUTOPILOT_UDB = 10, //UAV Dev Board
    e_MAV_AUTOPILOT_MAV_AUTOPILOT_FP = 11, //FlexiPilot
    e_MAV_AUTOPILOT_MAV_AUTOPILOT_PX4 = 12, //PX4 Autopilot - http:pixhawk.ethz.ch/px4/
    e_MAV_AUTOPILOT_MAV_AUTOPILOT_SMACCMPILOT = 13, //SMACCMPilot - http:smaccmpilot.org
    e_MAV_AUTOPILOT_MAV_AUTOPILOT_AUTOQUAD = 14, //AutoQuad -- http:autoquad.org
    e_MAV_AUTOPILOT_MAV_AUTOPILOT_ARMAZILA = 15, //Armazila -- http:armazila.com
    e_MAV_AUTOPILOT_MAV_AUTOPILOT_AEROB = 16, //Aerob -- http:aerob.ru
    e_MAV_AUTOPILOT_MAV_AUTOPILOT_ASLUAV = 17, //ASLUAV autopilot -- http:www.asl.ethz.ch
    e_MAV_AUTOPILOT_MAV_AUTOPILOT_SMARTAP = 18 //SmartAP Autopilot - http:sky-drones.com
} e_MAV_AUTOPILOT;

/**
*These flags encode the MAV mode.*/
typedef  enum
{
    e_MAV_MODE_FLAG_MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1, //0b00000001 Reserved for future use.
    /**
    *0b00000010 system has a test mode enabled. This flag is intended for temporary system tests and should
    *	not be used for stable implementations*/
    e_MAV_MODE_FLAG_MAV_MODE_FLAG_TEST_ENABLED = 2,
    /**
    *0b00000100 autonomous mode enabled, system finds its own goal positions. Guided flag can be set or not,
    *	depends on the actual implementation*/
    e_MAV_MODE_FLAG_MAV_MODE_FLAG_AUTO_ENABLED = 4,
    e_MAV_MODE_FLAG_MAV_MODE_FLAG_GUIDED_ENABLED = 8, //0b00001000 guided mode enabled, system flies waypoints / mission items.
    /**
    *0b00010000 system stabilizes electronically its attitude (and optionally position). It needs however further
    *	control inputs to move around*/
    e_MAV_MODE_FLAG_MAV_MODE_FLAG_STABILIZE_ENABLED = 16,
    /**
    *0b00100000 hardware in the loop simulation. All motors / actuators are blocked, but internal software
    *	is full operational*/
    e_MAV_MODE_FLAG_MAV_MODE_FLAG_HIL_ENABLED = 32,
    e_MAV_MODE_FLAG_MAV_MODE_FLAG_MANUAL_INPUT_ENABLED = 64, //0b01000000 remote control input is enabled.
    /**
    *0b10000000 MAV safety set to armed. Motors are enabled / running / can start. Ready to fly. Additional
    *	note: this flag is to be ignore when sent in the command MAV_CMD_DO_SET_MODE and MAV_CMD_COMPONENT_ARM_DISARM
    *	shall be used instead. The flag can still be used to report the armed state*/
    e_MAV_MODE_FLAG_MAV_MODE_FLAG_SAFETY_ARMED = 128
} e_MAV_MODE_FLAG;

typedef  enum
{
    e_MAV_STATE_MAV_STATE_UNINIT = 0, //Uninitialized system, state is unknown.
    e_MAV_STATE_MAV_STATE_BOOT = 1, //System is booting up.
    e_MAV_STATE_MAV_STATE_CALIBRATING = 2, //System is calibrating and not flight-ready.
    e_MAV_STATE_MAV_STATE_STANDBY = 3, //System is grounded and on standby. It can be launched any time.
    e_MAV_STATE_MAV_STATE_ACTIVE = 4, //System is active and might be already airborne. Motors are engaged.
    e_MAV_STATE_MAV_STATE_CRITICAL = 5, //System is in a non-normal flight mode. It can however still navigate.
    /**
    *System is in a non-normal flight mode. It lost control over parts or over the whole airframe. It is in
    *	mayday and going down*/
    e_MAV_STATE_MAV_STATE_EMERGENCY = 6,
    e_MAV_STATE_MAV_STATE_POWEROFF = 7, //System just initialized its power-down sequence, will shut down now.
    e_MAV_STATE_MAV_STATE_FLIGHT_TERMINATION = 8 //System is terminating itself.
} e_MAV_STATE;

/**
*These encode the sensors whose status is sent as part of the SYS_STATUS message.*/
typedef  enum
{
    e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_GYRO = 1, //0x01 3D gyro
    e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_ACCEL = 2, //0x02 3D accelerometer
    e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_MAG = 4, //0x04 3D magnetometer
    e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE = 8, //0x08 absolute pressure
    e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE = 16, //0x10 differential pressure
    e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_GPS = 32, //0x20 GPS
    e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW = 64, //0x40 optical flow
    e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_VISION_POSITION = 128, //0x80 computer vision position
    e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_LASER_POSITION = 256, //0x100 laser based position
    e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH = 512, //0x200 external ground truth (Vicon or Leica)
    e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL = 1024, //0x400 3D angular rate control
    e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION = 2048, //0x800 attitude stabilization
    e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_YAW_POSITION = 4096, //0x1000 yaw position
    e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL = 8192, //0x2000 z/altitude control
    e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL = 16384, //0x4000 x/y position control
    e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS = 32768, //0x8000 motor outputs / control
    e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_RC_RECEIVER = 65536, //0x10000 rc receiver
    e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_GYRO2 = 131072, //0x20000 2nd 3D gyro
    e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_ACCEL2 = 262144, //0x40000 2nd 3D accelerometer
    e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_MAG2 = 524288, //0x80000 2nd 3D magnetometer
    e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_GEOFENCE = 1048576, //0x100000 geofence
    e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_AHRS = 2097152, //0x200000 AHRS subsystem health
    e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_TERRAIN = 4194304, //0x400000 Terrain subsystem health
    e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_REVERSE_MOTOR = 8388608, //0x800000 Motors are reversed
    e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_LOGGING = 16777216, //0x1000000 Logging
    e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_BATTERY = 33554432 //0x2000000 Battery
} e_MAV_SYS_STATUS_SENSOR;

typedef  enum
{
    /**
    *Global coordinate frame, WGS84 coordinate system. First value / x: latitude, second value / y: longitude,
    *	third value / z: positive altitude over mean sea level (MSL*/
    e_MAV_FRAME_MAV_FRAME_GLOBAL = 0,
    e_MAV_FRAME_MAV_FRAME_LOCAL_NED = 1, //Local coordinate frame, Z-up (x: north, y: east, z: down).
    e_MAV_FRAME_MAV_FRAME_MISSION = 2, //NOT a coordinate frame, indicates a mission command.
    /**
    *Global coordinate frame, WGS84 coordinate system, relative altitude over ground with respect to the home
    *	position. First value / x: latitude, second value / y: longitude, third value / z: positive altitude
    *	with 0 being at the altitude of the home location*/
    e_MAV_FRAME_MAV_FRAME_GLOBAL_RELATIVE_ALT = 3,
    e_MAV_FRAME_MAV_FRAME_LOCAL_ENU = 4, //Local coordinate frame, Z-down (x: east, y: north, z: up)
    /**
    *Global coordinate frame, WGS84 coordinate system. First value / x: latitude in degrees*1.0e-7, second
    *	value / y: longitude in degrees*1.0e-7, third value / z: positive altitude over mean sea level (MSL*/
    e_MAV_FRAME_MAV_FRAME_GLOBAL_INT = 5,
    /**
    *Global coordinate frame, WGS84 coordinate system, relative altitude over ground with respect to the home
    *	position. First value / x: latitude in degrees*10e-7, second value / y: longitude in degrees*10e-7, third
    *	value / z: positive altitude with 0 being at the altitude of the home location*/
    e_MAV_FRAME_MAV_FRAME_GLOBAL_RELATIVE_ALT_INT = 6,
    /**
    *Offset to the current local frame. Anything expressed in this frame should be added to the current local
    *	frame position*/
    e_MAV_FRAME_MAV_FRAME_LOCAL_OFFSET_NED = 7,
    /**
    *Setpoint in body NED frame. This makes sense if all position control is externalized - e.g. useful to
    *	command 2 m/s^2 acceleration to the right*/
    e_MAV_FRAME_MAV_FRAME_BODY_NED = 8,
    /**
    *Offset in body NED frame. This makes sense if adding setpoints to the current flight path, to avoid an
    *	obstacle - e.g. useful to command 2 m/s^2 acceleration to the east*/
    e_MAV_FRAME_MAV_FRAME_BODY_OFFSET_NED = 9,
    /**
    *Global coordinate frame with above terrain level altitude. WGS84 coordinate system, relative altitude
    *	over terrain with respect to the waypoint coordinate. First value / x: latitude in degrees, second value
    *	/ y: longitude in degrees, third value / z: positive altitude in meters with 0 being at ground level
    *	in terrain model*/
    e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT = 10,
    /**
    *Global coordinate frame with above terrain level altitude. WGS84 coordinate system, relative altitude
    *	over terrain with respect to the waypoint coordinate. First value / x: latitude in degrees*10e-7, second
    *	value / y: longitude in degrees*10e-7, third value / z: positive altitude in meters with 0 being at ground
    *	level in terrain model*/
    e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT_INT = 11
} e_MAV_FRAME;

/**
*These defines are predefined OR-combined mode flags. There is no need to use values from this enum, but it
*	 simplifies the use of the mode flags. Note that manual input is enabled in all modes as a safety override.*/
typedef  enum
{
    e_MAV_MODE_MAV_MODE_PREFLIGHT = 0, //System is not ready to fly, booting, calibrating, etc. No flag is set.
    e_MAV_MODE_MAV_MODE_MANUAL_DISARMED = 64, //System is allowed to be active, under manual (RC) control, no stabilization
    e_MAV_MODE_MAV_MODE_TEST_DISARMED = 66, //UNDEFINED mode. This solely depends on the autopilot - use with caution, intended for developers only
    e_MAV_MODE_MAV_MODE_STABILIZE_DISARMED = 80, //System is allowed to be active, under assisted RC control.
    e_MAV_MODE_MAV_MODE_GUIDED_DISARMED = 88, //System is allowed to be active, under autonomous control, manual setpoint
    /**
    *System is allowed to be active, under autonomous control and navigation (the trajectory is decided onboard
    *	and not pre-programmed by waypoints*/
    e_MAV_MODE_MAV_MODE_AUTO_DISARMED = 92,
    e_MAV_MODE_MAV_MODE_MANUAL_ARMED = 192, //System is allowed to be active, under manual (RC) control, no stabilization
    e_MAV_MODE_MAV_MODE_TEST_ARMED = 194, //UNDEFINED mode. This solely depends on the autopilot - use with caution, intended for developers only
    e_MAV_MODE_MAV_MODE_STABILIZE_ARMED = 208, //System is allowed to be active, under assisted RC control.
    e_MAV_MODE_MAV_MODE_GUIDED_ARMED = 216, //System is allowed to be active, under autonomous control, manual setpoint
    /**
    *System is allowed to be active, under autonomous control and navigation (the trajectory is decided onboard
    *	and not pre-programmed by waypoints*/
    e_MAV_MODE_MAV_MODE_AUTO_ARMED = 220
} e_MAV_MODE;

inline static e_MAV_MODE _en__a(UMAX id)
{
    switch(id)
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
    return -1;
}
inline static UMAX _id__a(e_MAV_MODE en)
{
    switch(en)
    {
        case e_MAV_MODE_MAV_MODE_PREFLIGHT:
            return 0;
        case e_MAV_MODE_MAV_MODE_MANUAL_DISARMED:
            return 1;
        case e_MAV_MODE_MAV_MODE_TEST_DISARMED:
            return 2;
        case e_MAV_MODE_MAV_MODE_STABILIZE_DISARMED:
            return 3;
        case e_MAV_MODE_MAV_MODE_GUIDED_DISARMED:
            return 4;
        case e_MAV_MODE_MAV_MODE_AUTO_DISARMED:
            return 5;
        case e_MAV_MODE_MAV_MODE_MANUAL_ARMED:
            return 6;
        case e_MAV_MODE_MAV_MODE_TEST_ARMED:
            return 7;
        case e_MAV_MODE_MAV_MODE_STABILIZE_ARMED:
            return 8;
        case e_MAV_MODE_MAV_MODE_GUIDED_ARMED:
            return 9;
        case e_MAV_MODE_MAV_MODE_AUTO_ARMED:
            return 10;
        default: ;// assert(false);//("Unknown enum" + id);
    }
}

/**
*Specifies the datatype of a MAVLink parameter.*/
typedef  enum
{
    e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_UINT8 = 1, //8-bit unsigned integer
    e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_INT8 = 2, //8-bit signed integer
    e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_UINT16 = 3, //16-bit unsigned integer
    e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_INT16 = 4, //16-bit signed integer
    e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_UINT32 = 5, //32-bit unsigned integer
    e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_INT32 = 6, //32-bit signed integer
    e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_UINT64 = 7, //64-bit unsigned integer
    e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_INT64 = 8, //64-bit signed integer
    e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_REAL32 = 9, //32-bit floating-point
    e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_REAL64 = 10 //64-bit floating-point
} e_MAV_PARAM_TYPE;

/**
*Type of GPS fix*/
typedef  enum
{
    e_GPS_FIX_TYPE_GPS_FIX_TYPE_NO_GPS = 0, //No GPS connected
    e_GPS_FIX_TYPE_GPS_FIX_TYPE_NO_FIX = 1, //No position information, GPS is connected
    e_GPS_FIX_TYPE_GPS_FIX_TYPE_2D_FIX = 2, //2D position
    e_GPS_FIX_TYPE_GPS_FIX_TYPE_3D_FIX = 3, //3D position
    e_GPS_FIX_TYPE_GPS_FIX_TYPE_DGPS = 4, //DGPS/SBAS aided 3D position
    e_GPS_FIX_TYPE_GPS_FIX_TYPE_RTK_FLOAT = 5, //RTK float, 3D position
    e_GPS_FIX_TYPE_GPS_FIX_TYPE_RTK_FIXED = 6, //RTK Fixed, 3D position
    e_GPS_FIX_TYPE_GPS_FIX_TYPE_STATIC = 7, //Static fixed, typically used for base stations
    e_GPS_FIX_TYPE_GPS_FIX_TYPE_PPP = 8 //PPP, 3D position.
} e_GPS_FIX_TYPE;

/**
*Type of mission items being requested/sent in mission protocol.*/
typedef  enum
{
    e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION = 0, //Items are mission commands for main mission.
    e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE = 1, //Specifies GeoFence area(s). Items are MAV_CMD_FENCE_ GeoFence items.
    /**
    *Specifies the rally points for the vehicle. Rally points are alternative RTL points. Items are MAV_CMD_RALLY_POINT
    *	rally point items*/
    e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY = 2,
    e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL = 255 //Only used in MISSION_CLEAR_ALL to clear all mission types.
} e_MAV_MISSION_TYPE;

inline static UMAX _id__j(e_MAV_MISSION_TYPE en)
{
    switch(en)
    {
        case e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION:
            return 0;
        case e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE:
            return 1;
        case e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY:
            return 2;
        case e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL:
            return 3;
        default: ;// assert(false);//("Unknown enum" + id);
    }
}

/**
*Commands to be executed by the MAV. They can be executed on user request, or as part of a mission script.
*	If the action is used in a mission, the parameter mapping to the waypoint/mission message is as follows:
*	Param 1, Param 2, Param 3, Param 4, X: Param 5, Y:Param 6, Z:Param 7. This command list is similar what
*	ARINC 424 is for commercial aircraft: A data format how to interpret waypoint/mission data*/
typedef  enum
{
    /**
    *Request storage of different parameter values and logs. This command will be only accepted if in pre-flight
    *	mode
    *	1	Storage action: Action defined by MAV_PREFLIGHT_STORAGE_ACTION_ADVANCED
    *	2	Storage area as defined by parameter database
    *	3	Storage flags as defined by parameter database
    *	4	Empty
    *	5	Empty
    *	6	Empty
    *	7	Empty*/
    e_MAV_CMD_MAV_CMD_PREFLIGHT_STORAGE_ADVANCED = 0,
    /**
    *Navigate to waypoint.
    *	1	Hold time in decimal seconds. (ignored by fixed wing, time to stay at waypoint for rotary wing)
    *	2	Acceptance radius in meters (if the sphere with this radius is hit, the waypoint counts as reached)
    *	3	0 to pass through the WP, if 	>	0 radius in meters to pass by WP. Positive value for clockwise orbit, negative value for counter-clockwise orbit. Allows trajectory control.
    *	4	Desired yaw angle at waypoint (rotary wing). NaN for unchanged.
    *	5	Latitude
    *	6	Longitude
    *	7	Altitude*/
    e_MAV_CMD_MAV_CMD_NAV_WAYPOINT = 16,
    /**
    *Loiter around this waypoint an unlimited amount of time
    *	1	Empty
    *	2	Empty
    *	3	Radius around waypoint, in meters. If positive loiter clockwise, else counter-clockwise
    *	4	Desired yaw angle.
    *	5	Latitude
    *	6	Longitude
    *	7	Altitude*/
    e_MAV_CMD_MAV_CMD_NAV_LOITER_UNLIM = 17,
    /**
    *Loiter around this waypoint for X turns
    *	1	Turns
    *	2	Empty
    *	3	Radius around waypoint, in meters. If positive loiter clockwise, else counter-clockwise
    *	4	Forward moving aircraft this sets exit xtrack location: 0 for center of loiter wp, 1 for exit location. Else, this is desired yaw angle
    *	5	Latitude
    *	6	Longitude
    *	7	Altitude*/
    e_MAV_CMD_MAV_CMD_NAV_LOITER_TURNS = 18,
    /**
    *Loiter around this waypoint for X seconds
    *	1	Seconds (decimal)
    *	2	Empty
    *	3	Radius around waypoint, in meters. If positive loiter clockwise, else counter-clockwise
    *	4	Forward moving aircraft this sets exit xtrack location: 0 for center of loiter wp, 1 for exit location. Else, this is desired yaw angle
    *	5	Latitude
    *	6	Longitude
    *	7	Altitude*/
    e_MAV_CMD_MAV_CMD_NAV_LOITER_TIME = 19,
    /**
    *Return to launch location
    *	1	Empty
    *	2	Empty
    *	3	Empty
    *	4	Empty
    *	5	Empty
    *	6	Empty
    *	7	Empty*/
    e_MAV_CMD_MAV_CMD_NAV_RETURN_TO_LAUNCH = 20,
    /**
    *Land at location
    *	1	Abort Alt
    *	2	Empty
    *	3	Empty
    *	4	Desired yaw angle. NaN for unchanged.
    *	5	Latitude
    *	6	Longitude
    *	7	Altitude (ground level)*/
    e_MAV_CMD_MAV_CMD_NAV_LAND = 21,
    /**
    *Takeoff from ground / hand
    *	1	Minimum pitch (if airspeed sensor present), desired pitch without sensor
    *	2	Empty
    *	3	Empty
    *	4	Yaw angle (if magnetometer present), ignored without magnetometer. NaN for unchanged.
    *	5	Latitude
    *	6	Longitude
    *	7	Altitude*/
    e_MAV_CMD_MAV_CMD_NAV_TAKEOFF = 22,
    /**
    *Land at local position (local frame only)
    *	1	Landing target number (if available)
    *	2	Maximum accepted offset from desired landing position [m] - computed magnitude from spherical coordinates: d = sqrt(x^2 + y^2 + z^2), which gives the maximum accepted distance between the desired landing position and the position where the vehicle is about to land
    *	3	Landing descend rate [ms^-1]
    *	4	Desired yaw angle [rad]
    *	5	Y-axis position [m]
    *	6	X-axis position [m]
    *	7	Z-axis / ground level position [m]*/
    e_MAV_CMD_MAV_CMD_NAV_LAND_LOCAL = 23,
    /**
    *Takeoff from local position (local frame only)
    *	1	Minimum pitch (if airspeed sensor present), desired pitch without sensor [rad]
    *	2	Empty
    *	3	Takeoff ascend rate [ms^-1]
    *	4	Yaw angle [rad] (if magnetometer or another yaw estimation source present), ignored without one of these
    *	5	Y-axis position [m]
    *	6	X-axis position [m]
    *	7	Z-axis position [m]*/
    e_MAV_CMD_MAV_CMD_NAV_TAKEOFF_LOCAL = 24,
    /**
    *Vehicle following, i.e. this waypoint represents the position of a moving vehicle
    *	1	Following logic to use (e.g. loitering or sinusoidal following) - depends on specific autopilot implementation
    *	2	Ground speed of vehicle to be followed
    *	3	Radius around waypoint, in meters. If positive loiter clockwise, else counter-clockwise
    *	4	Desired yaw angle.
    *	5	Latitude
    *	6	Longitude
    *	7	Altitude*/
    e_MAV_CMD_MAV_CMD_NAV_FOLLOW = 25,
    /**
    *Continue on the current course and climb/descend to specified altitude.  When the altitude is reached
    *	continue to the next command (i.e., don't proceed to the next command until the desired altitude is reached
    *	1	Climb or Descend (0 = Neutral, command completes when within 5m of this command's altitude, 1 = Climbing, command completes when at or above this command's altitude, 2 = Descending, command completes when at or below this command's altitude.
    *	2	Empty
    *	3	Empty
    *	4	Empty
    *	5	Empty
    *	6	Empty
    *	7	Desired altitude in meters*/
    e_MAV_CMD_MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT = 30,
    /**
    *Begin loiter at the specified Latitude and Longitude.  If Lat=Lon=0, then loiter at the current position.
    *	 Don't consider the navigation command complete (don't leave loiter) until the altitude has been reached.
    *	 Additionally, if the Heading Required parameter is non-zero the  aircraft will not leave the loiter
    *	until heading toward the next waypoint.
    *	1	Heading Required (0 = False)
    *	2	Radius in meters. If positive loiter clockwise, negative counter-clockwise, 0 means no change to standard loiter.
    *	3	Empty
    *	4	Forward moving aircraft this sets exit xtrack location: 0 for center of loiter wp, 1 for exit location
    *	5	Latitude
    *	6	Longitude
    *	7	Altitude*/
    e_MAV_CMD_MAV_CMD_NAV_LOITER_TO_ALT = 31,
    /**
    *Being following a target
    *	1	System ID (the system ID of the FOLLOW_TARGET beacon). Send 0 to disable follow-me and return to the default position hold mode
    *	2	RESERVED
    *	3	RESERVED
    *	4	altitude flag: 0: Keep current altitude, 1: keep altitude difference to target, 2: go to a fixed altitude above home
    *	5	altitude
    *	6	RESERVED
    *	7	TTL in seconds in which the MAV should go to the default position hold mode after a message rx timeout*/
    e_MAV_CMD_MAV_CMD_DO_FOLLOW = 32,
    /**
    *Reposition the MAV after a follow target command has been sent
    *	1	Camera q1 (where 0 is on the ray from the camera to the tracking device)
    *	2	Camera q2
    *	3	Camera q3
    *	4	Camera q4
    *	5	altitude offset from target (m)
    *	6	X offset from target (m)
    *	7	Y offset from target (m)*/
    e_MAV_CMD_MAV_CMD_DO_FOLLOW_REPOSITION = 33,
    /**
    *Sets the region of interest (ROI) for a sensor set or the vehicle itself. This can then be used by the
    *	vehicles control system to control the vehicle attitude and the attitude of various sensors such as cameras
    *	1	Region of intereset mode. (see MAV_ROI enum)
    *	2	Waypoint index/ target ID. (see MAV_ROI enum)
    *	3	ROI index (allows a vehicle to manage multiple ROI's)
    *	4	Empty
    *	5	x the location of the fixed ROI (see MAV_FRAME)
    *	6	y
    *	7	z*/
    e_MAV_CMD_MAV_CMD_NAV_ROI = 80,
    /**
    *Control autonomous path planning on the MAV.
    *	1	0: Disable local obstacle avoidance / local path planning (without resetting map), 1: Enable local path planning, 2: Enable and reset local path planning
    *	2	0: Disable full path planning (without resetting map), 1: Enable, 2: Enable and reset map/occupancy grid, 3: Enable and reset planned route, but not occupancy grid
    *	3	Empty
    *	4	Yaw angle at goal, in compass degrees, [0..360]
    *	5	Latitude/X of goal
    *	6	Longitude/Y of goal
    *	7	Altitude/Z of goal*/
    e_MAV_CMD_MAV_CMD_NAV_PATHPLANNING = 81,
    /**
    *Navigate to waypoint using a spline path.
    *	1	Hold time in decimal seconds. (ignored by fixed wing, time to stay at waypoint for rotary wing)
    *	2	Empty
    *	3	Empty
    *	4	Empty
    *	5	Latitude/X of goal
    *	6	Longitude/Y of goal
    *	7	Altitude/Z of goal*/
    e_MAV_CMD_MAV_CMD_NAV_SPLINE_WAYPOINT = 82,
    /**
    *Takeoff from ground using VTOL mode
    *	1	Empty
    *	2	Front transition heading, see VTOL_TRANSITION_HEADING enum.
    *	3	Empty
    *	4	Yaw angle in degrees. NaN for unchanged.
    *	5	Latitude
    *	6	Longitude
    *	7	Altitude*/
    e_MAV_CMD_MAV_CMD_NAV_VTOL_TAKEOFF = 84,
    /**
    *Land using VTOL mode
    *	1	Empty
    *	2	Empty
    *	3	Approach altitude (with the same reference as the Altitude field). NaN if unspecified.
    *	4	Yaw angle in degrees. NaN for unchanged.
    *	5	Latitude
    *	6	Longitude
    *	7	Altitude (ground level)*/
    e_MAV_CMD_MAV_CMD_NAV_VTOL_LAND = 85,
    /**
    *hand control over to an external controller
    *	1	On / Off (	>	0.5f on)
    *	2	Empty
    *	3	Empty
    *	4	Empty
    *	5	Empty
    *	6	Empty
    *	7	Empty*/
    e_MAV_CMD_MAV_CMD_NAV_GUIDED_ENABLE = 92,
    /**
    *Delay the next navigation command a number of seconds or until a specified time
    *	1	Delay in seconds (decimal, -1 to enable time-of-day fields)
    *	2	hour (24h format, UTC, -1 to ignore)
    *	3	minute (24h format, UTC, -1 to ignore)
    *	4	second (24h format, UTC)
    *	5	Empty
    *	6	Empty
    *	7	Empty*/
    e_MAV_CMD_MAV_CMD_NAV_DELAY = 93,
    /**
    *Descend and place payload.  Vehicle descends until it detects a hanging payload has reached the ground,
    *	the gripper is opened to release the payloa
    *	1	Maximum distance to descend (meters)
    *	2	Empty
    *	3	Empty
    *	4	Empty
    *	5	Latitude (deg * 1E7)
    *	6	Longitude (deg * 1E7)
    *	7	Altitude (meters)*/
    e_MAV_CMD_MAV_CMD_NAV_PAYLOAD_PLACE = 94,
    /**
    *NOP - This command is only used to mark the upper limit of the NAV/ACTION commands in the enumeratio
    *	1	Empty
    *	2	Empty
    *	3	Empty
    *	4	Empty
    *	5	Empty
    *	6	Empty
    *	7	Empty*/
    e_MAV_CMD_MAV_CMD_NAV_LAST = 95,
    /**
    *Delay mission state machine.
    *	1	Delay in seconds (decimal)
    *	2	Empty
    *	3	Empty
    *	4	Empty
    *	5	Empty
    *	6	Empty
    *	7	Empty*/
    e_MAV_CMD_MAV_CMD_CONDITION_DELAY = 112,
    /**
    *Ascend/descend at rate.  Delay mission state machine until desired altitude reached.
    *	1	Descent / Ascend rate (m/s)
    *	2	Empty
    *	3	Empty
    *	4	Empty
    *	5	Empty
    *	6	Empty
    *	7	Finish Altitude*/
    e_MAV_CMD_MAV_CMD_CONDITION_CHANGE_ALT = 113,
    /**
    *Delay mission state machine until within desired distance of next NAV point.
    *	1	Distance (meters)
    *	2	Empty
    *	3	Empty
    *	4	Empty
    *	5	Empty
    *	6	Empty
    *	7	Empty*/
    e_MAV_CMD_MAV_CMD_CONDITION_DISTANCE = 114,
    /**
    *Reach a certain target angle.
    *	1	target angle: [0-360], 0 is north
    *	2	speed during yaw change:[deg per second]
    *	3	direction: negative: counter clockwise, positive: clockwise [-1,1]
    *	4	relative offset or absolute angle: [ 1,0]
    *	5	Empty
    *	6	Empty
    *	7	Empty*/
    e_MAV_CMD_MAV_CMD_CONDITION_YAW = 115,
    /**
    *NOP - This command is only used to mark the upper limit of the CONDITION commands in the enumeratio
    *	1	Empty
    *	2	Empty
    *	3	Empty
    *	4	Empty
    *	5	Empty
    *	6	Empty
    *	7	Empty*/
    e_MAV_CMD_MAV_CMD_CONDITION_LAST = 159,
    /**
    *Set system mode.
    *	1	Mode, as defined by ENUM MAV_MODE
    *	2	Custom mode - this is system specific, please refer to the individual autopilot specifications for details.
    *	3	Custom sub mode - this is system specific, please refer to the individual autopilot specifications for details.
    *	4	Empty
    *	5	Empty
    *	6	Empty
    *	7	Empty*/
    e_MAV_CMD_MAV_CMD_DO_SET_MODE = 176,
    /**
    *Jump to the desired command in the mission list.  Repeat this action only the specified number of time
    *	1	Sequence number
    *	2	Repeat count
    *	3	Empty
    *	4	Empty
    *	5	Empty
    *	6	Empty
    *	7	Empty*/
    e_MAV_CMD_MAV_CMD_DO_JUMP = 177,
    /**
    *Change speed and/or throttle set points.
    *	1	Speed type (0=Airspeed, 1=Ground Speed)
    *	2	Speed  (m/s, -1 indicates no change)
    *	3	Throttle  ( Percent, -1 indicates no change)
    *	4	absolute or relative [0,1]
    *	5	Empty
    *	6	Empty
    *	7	Empty*/
    e_MAV_CMD_MAV_CMD_DO_CHANGE_SPEED = 178,
    /**
    *Changes the home location either to the current location or a specified location.
    *	1	Use current (1=use current location, 0=use specified location)
    *	2	Empty
    *	3	Empty
    *	4	Empty
    *	5	Latitude
    *	6	Longitude
    *	7	Altitude*/
    e_MAV_CMD_MAV_CMD_DO_SET_HOME = 179,
    /**
    *Set a system parameter.  Caution!  Use of this command requires knowledge of the numeric enumeration value
    *	of the parameter
    *	1	Parameter number
    *	2	Parameter value
    *	3	Empty
    *	4	Empty
    *	5	Empty
    *	6	Empty
    *	7	Empty*/
    e_MAV_CMD_MAV_CMD_DO_SET_PARAMETER = 180,
    /**
    *Set a relay to a condition.
    *	1	Relay number
    *	2	Setting (1=on, 0=off, others possible depending on system hardware)
    *	3	Empty
    *	4	Empty
    *	5	Empty
    *	6	Empty
    *	7	Empty*/
    e_MAV_CMD_MAV_CMD_DO_SET_RELAY = 181,
    /**
    *Cycle a relay on and off for a desired number of cyles with a desired period.
    *	1	Relay number
    *	2	Cycle count
    *	3	Cycle time (seconds, decimal)
    *	4	Empty
    *	5	Empty
    *	6	Empty
    *	7	Empty*/
    e_MAV_CMD_MAV_CMD_DO_REPEAT_RELAY = 182,
    /**
    *Set a servo to a desired PWM value.
    *	1	Servo number
    *	2	PWM (microseconds, 1000 to 2000 typical)
    *	3	Empty
    *	4	Empty
    *	5	Empty
    *	6	Empty
    *	7	Empty*/
    e_MAV_CMD_MAV_CMD_DO_SET_SERVO = 183,
    /**
    *Cycle a between its nominal setting and a desired PWM for a desired number of cycles with a desired period
    *	1	Servo number
    *	2	PWM (microseconds, 1000 to 2000 typical)
    *	3	Cycle count
    *	4	Cycle time (seconds)
    *	5	Empty
    *	6	Empty
    *	7	Empty*/
    e_MAV_CMD_MAV_CMD_DO_REPEAT_SERVO = 184,
    /**
    *Terminate flight immediately
    *	1	Flight termination activated if 	>	0.5
    *	2	Empty
    *	3	Empty
    *	4	Empty
    *	5	Empty
    *	6	Empty
    *	7	Empty*/
    e_MAV_CMD_MAV_CMD_DO_FLIGHTTERMINATION = 185,
    /**
    *Change altitude set point.
    *	1	Altitude in meters
    *	2	Mav frame of new altitude (see MAV_FRAME)
    *	3	Empty
    *	4	Empty
    *	5	Empty
    *	6	Empty
    *	7	Empty*/
    e_MAV_CMD_MAV_CMD_DO_CHANGE_ALTITUDE = 186,
    /**
    *Mission command to perform a landing. This is used as a marker in a mission to tell the autopilot where
    *	a sequence of mission items that represents a landing starts. It may also be sent via a COMMAND_LONG
    *	to trigger a landing, in which case the nearest (geographically) landing sequence in the mission will
    *	be used. The Latitude/Longitude is optional, and may be set to 0 if not needed. If specified then it
    *	will be used to help find the closest landing sequence
    *	1	Empty
    *	2	Empty
    *	3	Empty
    *	4	Empty
    *	5	Latitude
    *	6	Longitude
    *	7	Empty*/
    e_MAV_CMD_MAV_CMD_DO_LAND_START = 189,
    /**
    *Mission command to perform a landing from a rally point.
    *	1	Break altitude (meters)
    *	2	Landing speed (m/s)
    *	3	Empty
    *	4	Empty
    *	5	Empty
    *	6	Empty
    *	7	Empty*/
    e_MAV_CMD_MAV_CMD_DO_RALLY_LAND = 190,
    /**
    *Mission command to safely abort an autonmous landing.
    *	1	Altitude (meters)
    *	2	Empty
    *	3	Empty
    *	4	Empty
    *	5	Empty
    *	6	Empty
    *	7	Empty*/
    e_MAV_CMD_MAV_CMD_DO_GO_AROUND = 191,
    /**
    *Reposition the vehicle to a specific WGS84 global position.
    *	1	Ground speed, less than 0 (-1) for default
    *	2	Bitmask of option flags, see the MAV_DO_REPOSITION_FLAGS enum.
    *	3	Reserved
    *	4	Yaw heading, NaN for unchanged. For planes indicates loiter direction (0: clockwise, 1: counter clockwise)
    *	5	Latitude (deg * 1E7)
    *	6	Longitude (deg * 1E7)
    *	7	Altitude (meters)*/
    e_MAV_CMD_MAV_CMD_DO_REPOSITION = 192,
    /**
    *If in a GPS controlled position mode, hold the current position or continue.
    *	1	0: Pause current mission or reposition command, hold current position. 1: Continue mission. A VTOL capable vehicle should enter hover mode (multicopter and VTOL planes). A plane should loiter with the default loiter radius.
    *	2	Reserved
    *	3	Reserved
    *	4	Reserved
    *	5	Reserved
    *	6	Reserved
    *	7	Reserved*/
    e_MAV_CMD_MAV_CMD_DO_PAUSE_CONTINUE = 193,
    /**
    *Set moving direction to forward or reverse.
    *	1	Direction (0=Forward, 1=Reverse)
    *	2	Empty
    *	3	Empty
    *	4	Empty
    *	5	Empty
    *	6	Empty
    *	7	Empty*/
    e_MAV_CMD_MAV_CMD_DO_SET_REVERSE = 194,
    /**
    *Control onboard camera system.
    *	1	Camera ID (-1 for all)
    *	2	Transmission: 0: disabled, 1: enabled compressed, 2: enabled raw
    *	3	Transmission mode: 0: video stream, 	>	0: single images every n seconds (decimal)
    *	4	Recording: 0: disabled, 1: enabled compressed, 2: enabled raw
    *	5	Empty
    *	6	Empty
    *	7	Empty*/
    e_MAV_CMD_MAV_CMD_DO_CONTROL_VIDEO = 200,
    /**
    *Sets the region of interest (ROI) for a sensor set or the vehicle itself. This can then be used by the
    *	vehicles control system to control the vehicle attitude and the attitude of various sensors such as cameras
    *	1	Region of intereset mode. (see MAV_ROI enum)
    *	2	Waypoint index/ target ID. (see MAV_ROI enum)
    *	3	ROI index (allows a vehicle to manage multiple ROI's)
    *	4	Empty
    *	5	MAV_ROI_WPNEXT: pitch offset from next waypoint, MAV_ROI_LOCATION: latitude
    *	6	MAV_ROI_WPNEXT: roll offset from next waypoint, MAV_ROI_LOCATION: longitude
    *	7	MAV_ROI_WPNEXT: yaw offset from next waypoint, MAV_ROI_LOCATION: altitude*/
    e_MAV_CMD_MAV_CMD_DO_SET_ROI = 201,
    /**
    *Mission command to configure an on-board camera controller system.
    *	1	Modes: P, TV, AV, M, Etc
    *	2	Shutter speed: Divisor number for one second
    *	3	Aperture: F stop number
    *	4	ISO number e.g. 80, 100, 200, Etc
    *	5	Exposure type enumerator
    *	6	Command Identity
    *	7	Main engine cut-off time before camera trigger in seconds/10 (0 means no cut-off)*/
    e_MAV_CMD_MAV_CMD_DO_DIGICAM_CONFIGURE = 202,
    /**
    *Mission command to control an on-board camera controller system.
    *	1	Session control e.g. show/hide lens
    *	2	Zoom's absolute position
    *	3	Zooming step value to offset zoom from the current position
    *	4	Focus Locking, Unlocking or Re-locking
    *	5	Shooting Command
    *	6	Command Identity
    *	7	Test shot identifier. If set to 1, image will only be captured, but not counted towards internal frame count.*/
    e_MAV_CMD_MAV_CMD_DO_DIGICAM_CONTROL = 203,
    /**
    *Mission command to configure a camera or antenna mount
    *	1	Mount operation mode (see MAV_MOUNT_MODE enum)
    *	2	stabilize roll? (1 = yes, 0 = no)
    *	3	stabilize pitch? (1 = yes, 0 = no)
    *	4	stabilize yaw? (1 = yes, 0 = no)
    *	5	roll input (0 = angle, 1 = angular rate)
    *	6	pitch input (0 = angle, 1 = angular rate)
    *	7	yaw input (0 = angle, 1 = angular rate)*/
    e_MAV_CMD_MAV_CMD_DO_MOUNT_CONFIGURE = 204,
    /**
    *Mission command to control a camera or antenna mount
    *	1	pitch depending on mount mode (degrees or degrees/second depending on pitch input).
    *	2	roll depending on mount mode (degrees or degrees/second depending on roll input).
    *	3	yaw depending on mount mode (degrees or degrees/second depending on yaw input).
    *	4	alt in meters depending on mount mode.
    *	5	latitude in degrees * 1E7, set if appropriate mount mode.
    *	6	longitude in degrees * 1E7, set if appropriate mount mode.
    *	7	MAV_MOUNT_MODE enum value*/
    e_MAV_CMD_MAV_CMD_DO_MOUNT_CONTROL = 205,
    /**
    *Mission command to set camera trigger distance for this flight. The camera is trigerred each time this
    *	distance is exceeded. This command can also be used to set the shutter integration time for the camera
    *	1	Camera trigger distance (meters). 0 to stop triggering.
    *	2	Camera shutter integration time (milliseconds). -1 or 0 to ignore
    *	3	Trigger camera once immediately. (0 = no trigger, 1 = trigger)
    *	4	Empty
    *	5	Empty
    *	6	Empty
    *	7	Empty*/
    e_MAV_CMD_MAV_CMD_DO_SET_CAM_TRIGG_DIST = 206,
    /**
    *Mission command to enable the geofence
    *	1	enable? (0=disable, 1=enable, 2=disable_floor_only)
    *	2	Empty
    *	3	Empty
    *	4	Empty
    *	5	Empty
    *	6	Empty
    *	7	Empty*/
    e_MAV_CMD_MAV_CMD_DO_FENCE_ENABLE = 207,
    /**
    *Mission command to trigger a parachute
    *	1	action (0=disable, 1=enable, 2=release, for some systems see PARACHUTE_ACTION enum, not in general message set.)
    *	2	Empty
    *	3	Empty
    *	4	Empty
    *	5	Empty
    *	6	Empty
    *	7	Empty*/
    e_MAV_CMD_MAV_CMD_DO_PARACHUTE = 208,
    /**
    *Mission command to perform motor test
    *	1	motor sequence number (a number from 1 to max number of motors on the vehicle)
    *	2	throttle type (0=throttle percentage, 1=PWM, 2=pilot throttle channel pass-through. See MOTOR_TEST_THROTTLE_TYPE enum)
    *	3	throttle
    *	4	timeout (in seconds)
    *	5	Empty
    *	6	Empty
    *	7	Empty*/
    e_MAV_CMD_MAV_CMD_DO_MOTOR_TEST = 209,
    /**
    *Change to/from inverted flight
    *	1	inverted (0=normal, 1=inverted)
    *	2	Empty
    *	3	Empty
    *	4	Empty
    *	5	Empty
    *	6	Empty
    *	7	Empty*/
    e_MAV_CMD_MAV_CMD_DO_INVERTED_FLIGHT = 210,
    /**
    *Sets a desired vehicle turn angle and speed change
    *	1	yaw angle to adjust steering by in centidegress
    *	2	speed - normalized to 0 .. 1
    *	3	Empty
    *	4	Empty
    *	5	Empty
    *	6	Empty
    *	7	Empty*/
    e_MAV_CMD_MAV_CMD_NAV_SET_YAW_SPEED = 213,
    /**
    *Mission command to set camera trigger interval for this flight. If triggering is enabled, the camera is
    *	triggered each time this interval expires. This command can also be used to set the shutter integration
    *	time for the camera
    *	1	Camera trigger cycle time (milliseconds). -1 or 0 to ignore.
    *	2	Camera shutter integration time (milliseconds). Should be less than trigger cycle time. -1 or 0 to ignore.
    *	3	Empty
    *	4	Empty
    *	5	Empty
    *	6	Empty
    *	7	Empty*/
    e_MAV_CMD_MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL = 214,
    /**
    *Mission command to control a camera or antenna mount, using a quaternion as reference.
    *	1	q1 - quaternion param #1, w (1 in null-rotation)
    *	2	q2 - quaternion param #2, x (0 in null-rotation)
    *	3	q3 - quaternion param #3, y (0 in null-rotation)
    *	4	q4 - quaternion param #4, z (0 in null-rotation)
    *	5	Empty
    *	6	Empty
    *	7	Empty*/
    e_MAV_CMD_MAV_CMD_DO_MOUNT_CONTROL_QUAT = 220,
    /**
    *set id of master controller
    *	1	System ID
    *	2	Component ID
    *	3	Empty
    *	4	Empty
    *	5	Empty
    *	6	Empty
    *	7	Empty*/
    e_MAV_CMD_MAV_CMD_DO_GUIDED_MASTER = 221,
    /**
    *set limits for external control
    *	1	timeout - maximum time (in seconds) that external controller will be allowed to control vehicle. 0 means no timeout
    *	2	absolute altitude min (in meters, AMSL) - if vehicle moves below this alt, the command will be aborted and the mission will continue.  0 means no lower altitude limit
    *	3	absolute altitude max (in meters)- if vehicle moves above this alt, the command will be aborted and the mission will continue.  0 means no upper altitude limit
    *	4	horizontal move limit (in meters, AMSL) - if vehicle moves more than this distance from it's location at the moment the command was executed, the command will be aborted and the mission will continue. 0 means no horizontal altitude limit
    *	5	Empty
    *	6	Empty
    *	7	Empty*/
    e_MAV_CMD_MAV_CMD_DO_GUIDED_LIMITS = 222,
    /**
    *Control vehicle engine. This is interpreted by the vehicles engine controller to change the target engine
    *	state. It is intended for vehicles with internal combustion engine
    *	1	0: Stop engine, 1:Start Engine
    *	2	0: Warm start, 1:Cold start. Controls use of choke where applicable
    *	3	Height delay (meters). This is for commanding engine start only after the vehicle has gained the specified height. Used in VTOL vehicles during takeoff to start engine after the aircraft is off the ground. Zero for no delay.
    *	4	Empty
    *	5	Empty
    *	5	Empty
    *	6	Empty
    *	7	Empty*/
    e_MAV_CMD_MAV_CMD_DO_ENGINE_CONTROL = 223,
    /**
    *NOP - This command is only used to mark the upper limit of the DO commands in the enumeration
    *	1	Empty
    *	2	Empty
    *	3	Empty
    *	4	Empty
    *	5	Empty
    *	6	Empty
    *	7	Empty*/
    e_MAV_CMD_MAV_CMD_DO_LAST = 240,
    /**
    *Trigger calibration. This command will be only accepted if in pre-flight mode. Except for Temperature
    *	Calibration, only one sensor should be set in a single message and all others should be zero
    *	1	1: gyro calibration, 3: gyro temperature calibration
    *	2	1: magnetometer calibration
    *	3	1: ground pressure calibration
    *	4	1: radio RC calibration, 2: RC trim calibration
    *	5	1: accelerometer calibration, 2: board level calibration, 3: accelerometer temperature calibration
    *	6	1: APM: compass/motor interference calibration (PX4: airspeed calibration, deprecated), 2: airspeed calibration
    *	7	1: ESC calibration, 3: barometer temperature calibration*/
    e_MAV_CMD_MAV_CMD_PREFLIGHT_CALIBRATION = 241,
    /**
    *Set sensor offsets. This command will be only accepted if in pre-flight mode.
    *	1	Sensor to adjust the offsets for: 0: gyros, 1: accelerometer, 2: magnetometer, 3: barometer, 4: optical flow, 5: second magnetometer, 6: third magnetometer
    *	2	X axis offset (or generic dimension 1), in the sensor's raw units
    *	3	Y axis offset (or generic dimension 2), in the sensor's raw units
    *	4	Z axis offset (or generic dimension 3), in the sensor's raw units
    *	5	Generic dimension 4, in the sensor's raw units
    *	6	Generic dimension 5, in the sensor's raw units
    *	7	Generic dimension 6, in the sensor's raw units*/
    e_MAV_CMD_MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS = 242,
    /**
    *Trigger UAVCAN config. This command will be only accepted if in pre-flight mode.
    *	1	1: Trigger actuator ID assignment and direction mapping.
    *	2	Reserved
    *	3	Reserved
    *	4	Reserved
    *	5	Reserved
    *	6	Reserved
    *	7	Reserved*/
    e_MAV_CMD_MAV_CMD_PREFLIGHT_UAVCAN = 243,
    /**
    *Request storage of different parameter values and logs. This command will be only accepted if in pre-flight
    *	mode
    *	1	Parameter storage: 0: READ FROM FLASH/EEPROM, 1: WRITE CURRENT TO FLASH/EEPROM, 2: Reset to defaults
    *	2	Mission storage: 0: READ FROM FLASH/EEPROM, 1: WRITE CURRENT TO FLASH/EEPROM, 2: Reset to defaults
    *	3	Onboard logging: 0: Ignore, 1: Start default rate logging, -1: Stop logging, 	>	1: start logging with rate of param 3 in Hz (e.g. set to 1000 for 1000 Hz logging)
    *	4	Reserved
    *	5	Empty
    *	6	Empty
    *	7	Empty*/
    e_MAV_CMD_MAV_CMD_PREFLIGHT_STORAGE = 245,
    /**
    *Request the reboot or shutdown of system components.
    *	1	0: Do nothing for autopilot, 1: Reboot autopilot, 2: Shutdown autopilot, 3: Reboot autopilot and keep it in the bootloader until upgraded.
    *	2	0: Do nothing for onboard computer, 1: Reboot onboard computer, 2: Shutdown onboard computer, 3: Reboot onboard computer and keep it in the bootloader until upgraded.
    *	3	WIP: 0: Do nothing for camera, 1: Reboot onboard camera, 2: Shutdown onboard camera, 3: Reboot onboard camera and keep it in the bootloader until upgraded
    *	4	WIP: 0: Do nothing for mount (e.g. gimbal), 1: Reboot mount, 2: Shutdown mount, 3: Reboot mount and keep it in the bootloader until upgraded
    *	5	Reserved, send 0
    *	6	Reserved, send 0
    *	7	WIP: ID (e.g. camera ID -1 for all IDs)*/
    e_MAV_CMD_MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN = 246,
    /**
    *Hold / continue the current action
    *	1	MAV_GOTO_DO_HOLD: hold MAV_GOTO_DO_CONTINUE: continue with next item in mission plan
    *	2	MAV_GOTO_HOLD_AT_CURRENT_POSITION: Hold at current position MAV_GOTO_HOLD_AT_SPECIFIED_POSITION: hold at specified position
    *	3	MAV_FRAME coordinate frame of hold point
    *	4	Desired yaw angle in degrees
    *	5	Latitude / X position
    *	6	Longitude / Y position
    *	7	Altitude / Z position*/
    e_MAV_CMD_MAV_CMD_OVERRIDE_GOTO = 252,
    /**
    *start running a mission
    *	1	first_item: the first mission item to run
    *	2	last_item:  the last mission item to run (after this item is run, the mission ends)*/
    e_MAV_CMD_MAV_CMD_MISSION_START = 300,
    /**
    *Arms / Disarms a component
    *	1	1 to arm, 0 to disarm*/
    e_MAV_CMD_MAV_CMD_COMPONENT_ARM_DISARM = 400,
    /**
    *Request the home position from the vehicle.
    *	1	Reserved
    *	2	Reserved
    *	3	Reserved
    *	4	Reserved
    *	5	Reserved
    *	6	Reserved
    *	7	Reserved*/
    e_MAV_CMD_MAV_CMD_GET_HOME_POSITION = 410,
    /**
    *Starts receiver pairing
    *	1	0:Spektrum
    *	2	0:Spektrum DSM2, 1:Spektrum DSMX*/
    e_MAV_CMD_MAV_CMD_START_RX_PAIR = 500,
    /**
    *Request the interval between messages for a particular MAVLink message ID
    *	1	The MAVLink message ID*/
    e_MAV_CMD_MAV_CMD_GET_MESSAGE_INTERVAL = 510,
    /**
    *Request the interval between messages for a particular MAVLink message ID. This interface replaces REQUEST_DATA_STREA
    *	1	The MAVLink message ID
    *	2	The interval between two messages, in microseconds. Set to -1 to disable and 0 to request default rate.*/
    e_MAV_CMD_MAV_CMD_SET_MESSAGE_INTERVAL = 511,
    /**
    *Request MAVLink protocol version compatibility
    *	1	1: Request supported protocol versions by all nodes on the network
    *	2	Reserved (all remaining params)*/
    e_MAV_CMD_MAV_CMD_REQUEST_PROTOCOL_VERSION = 519,
    /**
    *Request autopilot capabilities
    *	1	1: Request autopilot version
    *	2	Reserved (all remaining params)*/
    e_MAV_CMD_MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES = 520,
    /**
    *WIP: Request camera information (CAMERA_INFORMATION).
    *	1	0: No action 1: Request camera capabilities
    *	2	Reserved (all remaining params)*/
    e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_INFORMATION = 521,
    /**
    *WIP: Request camera settings (CAMERA_SETTINGS).
    *	1	0: No Action 1: Request camera settings
    *	2	Reserved (all remaining params)*/
    e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_SETTINGS = 522,
    /**
    *WIP: Request storage information (STORAGE_INFORMATION). Use the command's target_component to target a
    *	specific component's storage
    *	1	Storage ID (0 for all, 1 for first, 2 for second, etc.)
    *	2	0: No Action 1: Request storage information
    *	3	Reserved (all remaining params)*/
    e_MAV_CMD_MAV_CMD_REQUEST_STORAGE_INFORMATION = 525,
    /**
    *WIP: Format a storage medium. Once format is complete, a STORAGE_INFORMATION message is sent. Use the
    *	command's target_component to target a specific component's storage
    *	1	Storage ID (1 for first, 2 for second, etc.)
    *	2	0: No action 1: Format storage
    *	3	Reserved (all remaining params)*/
    e_MAV_CMD_MAV_CMD_STORAGE_FORMAT = 526,
    /**
    *WIP: Request camera capture status (CAMERA_CAPTURE_STATUS)
    *	1	0: No Action 1: Request camera capture status
    *	2	Reserved (all remaining params)*/
    e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS = 527,
    /**
    *WIP: Request flight information (FLIGHT_INFORMATION)
    *	1	1: Request flight information
    *	2	Reserved (all remaining params)*/
    e_MAV_CMD_MAV_CMD_REQUEST_FLIGHT_INFORMATION = 528,
    /**
    *WIP: Reset all camera settings to Factory Default
    *	1	0: No Action 1: Reset all settings
    *	2	Reserved (all remaining params)*/
    e_MAV_CMD_MAV_CMD_RESET_CAMERA_SETTINGS = 529,
    /**
    *Set camera running mode. Use NAN for reserved values.
    *	1	Reserved (Set to 0)
    *	2	Camera mode (see CAMERA_MODE enum)
    *	3	Reserved (all remaining params)*/
    e_MAV_CMD_MAV_CMD_SET_CAMERA_MODE = 530,
    /**
    *Start image capture sequence. Sends CAMERA_IMAGE_CAPTURED after each capture. Use NAN for reserved values
    *	1	Reserved (Set to 0)
    *	2	Duration between two consecutive pictures (in seconds)
    *	3	Number of images to capture total - 0 for unlimited capture
    *	4	Reserved (all remaining params)*/
    e_MAV_CMD_MAV_CMD_IMAGE_START_CAPTURE = 2000,
    /**
    *Stop image capture sequence Use NAN for reserved values.
    *	1	Reserved (Set to 0)
    *	2	Reserved (all remaining params)*/
    e_MAV_CMD_MAV_CMD_IMAGE_STOP_CAPTURE = 2001,
    /**
    *WIP: Re-request a CAMERA_IMAGE_CAPTURE packet. Use NAN for reserved values.
    *	1	Sequence number for missing CAMERA_IMAGE_CAPTURE packet
    *	2	Reserved (all remaining params)*/
    e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE = 2002,
    /**
    *Enable or disable on-board camera triggering system.
    *	1	Trigger enable/disable (0 for disable, 1 for start), -1 to ignore
    *	2	1 to reset the trigger sequence, -1 or 0 to ignore
    *	3	1 to pause triggering, but without switching the camera off or retracting it. -1 to ignore*/
    e_MAV_CMD_MAV_CMD_DO_TRIGGER_CONTROL = 2003,
    /**
    *Starts video capture (recording). Use NAN for reserved values.
    *	1	Reserved (Set to 0)
    *	2	Frequency CAMERA_CAPTURE_STATUS messages should be sent while recording (0 for no messages, otherwise frequency in Hz)
    *	3	Reserved (all remaining params)*/
    e_MAV_CMD_MAV_CMD_VIDEO_START_CAPTURE = 2500,
    /**
    *Stop the current video capture (recording). Use NAN for reserved values.
    *	1	Reserved (Set to 0)
    *	2	Reserved (all remaining params)*/
    e_MAV_CMD_MAV_CMD_VIDEO_STOP_CAPTURE = 2501,
    /**
    *WIP: Start video streaming
    *	1	Camera ID (0 for all cameras, 1 for first, 2 for second, etc.)
    *	2	Reserved*/
    e_MAV_CMD_MAV_CMD_VIDEO_START_STREAMING = 2502,
    /**
    *WIP: Stop the current video streaming
    *	1	Camera ID (0 for all cameras, 1 for first, 2 for second, etc.)
    *	2	Reserved*/
    e_MAV_CMD_MAV_CMD_VIDEO_STOP_STREAMING = 2503,
    /**
    *WIP: Request video stream information (VIDEO_STREAM_INFORMATION)
    *	1	Camera ID (0 for all cameras, 1 for first, 2 for second, etc.)
    *	2	0: No Action 1: Request video stream information
    *	3	Reserved (all remaining params)*/
    e_MAV_CMD_MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION = 2504,
    /**
    *Request to start streaming logging data over MAVLink (see also LOGGING_DATA message)
    *	1	Format: 0: ULog
    *	2	Reserved (set to 0)
    *	3	Reserved (set to 0)
    *	4	Reserved (set to 0)
    *	5	Reserved (set to 0)
    *	6	Reserved (set to 0)
    *	7	Reserved (set to 0)*/
    e_MAV_CMD_MAV_CMD_LOGGING_START = 2510,
    /**
    *Request to stop streaming log data over MAVLink
    *	1	Reserved (set to 0)
    *	2	Reserved (set to 0)
    *	3	Reserved (set to 0)
    *	4	Reserved (set to 0)
    *	5	Reserved (set to 0)
    *	6	Reserved (set to 0)
    *	7	Reserved (set to 0)*/
    e_MAV_CMD_MAV_CMD_LOGGING_STOP = 2511,
    /**
    *1	Landing gear ID (default: 0, -1 for all)
    *	2	Landing gear position (Down: 0, Up: 1, NAN for no change)
    *	3	Reserved, set to NAN
    *	4	Reserved, set to NAN
    *	5	Reserved, set to NAN
    *	6	Reserved, set to NAN
    *	7	Reserved, set to NAN*/
    e_MAV_CMD_MAV_CMD_AIRFRAME_CONFIGURATION = 2520,
    /**
    *Create a panorama at the current position
    *	1	Viewing angle horizontal of the panorama (in degrees, +- 0.5 the total angle)
    *	2	Viewing angle vertical of panorama (in degrees)
    *	3	Speed of the horizontal rotation (in degrees per second)
    *	4	Speed of the vertical rotation (in degrees per second)*/
    e_MAV_CMD_MAV_CMD_PANORAMA_CREATE = 2800,
    /**
    *Request VTOL transition
    *	1	The target VTOL state, as defined by ENUM MAV_VTOL_STATE. Only MAV_VTOL_STATE_MC and MAV_VTOL_STATE_FW can be used.*/
    e_MAV_CMD_MAV_CMD_DO_VTOL_TRANSITION = 3000,
    /**
    *Request authorization to arm the vehicle to a external entity, the arm authorizer is resposible to request all data that is needs from the vehicle before authorize or deny the request. If approved the progress of command_ack message should be set with period of time that this authorization is valid in seconds or in case it was denied it should be set with one of the reasons in ARM_AUTH_DENIED_REASON.
    *
    *	1	Vehicle system id, this way ground station can request arm authorization on behalf of any vehicle*/
    e_MAV_CMD_MAV_CMD_ARM_AUTHORIZATION_REQUEST = 3001,
    /**
    *This command sets the submode to standard guided when vehicle is in guided mode. The vehicle holds position and altitude and the user can input the desired velocites along all three axes.*/
    e_MAV_CMD_MAV_CMD_SET_GUIDED_SUBMODE_STANDARD = 4000,
    /**
    *This command sets submode circle when vehicle is in guided mode. Vehicle flies along a circle facing the center of the circle. The user can input the velocity along the circle and change the radius. If no input is given the vehicle will hold position.
    *
    *	1	Radius of desired circle in CIRCLE_MODE
    *	2	User defined
    *	3	User defined
    *	4	User defined
    *	5	Unscaled target latitude of center of circle in CIRCLE_MODE
    *	6	Unscaled target longitude of center of circle in CIRCLE_MODE*/
    e_MAV_CMD_MAV_CMD_SET_GUIDED_SUBMODE_CIRCLE = 4001,
    /**
    *WIP: Delay mission state machine until gate has been reached.
    *	1	Geometry: 0: orthogonal to path between previous and next waypoint.
    *	2	Altitude: 0: ignore altitude
    *	3	Empty
    *	4	Empty
    *	5	Latitude
    *	6	Longitude
    *	7	Altitude*/
    e_MAV_CMD_MAV_CMD_CONDITION_GATE = 4501,
    /**
    *Fence return point. There can only be one fence return point.
    *
    *	1	Reserved
    *	2	Reserved
    *	3	Reserved
    *	4	Reserved
    *	5	Latitude
    *	6	Longitude
    *	7	Altitude*/
    e_MAV_CMD_MAV_CMD_NAV_FENCE_RETURN_POINT = 5000,
    /**
    *Fence vertex for an inclusion polygon (the polygon must not be self-intersecting). The vehicle must stay within this area. Minimum of 3 vertices required.
    *
    *	1	Polygon vertex count
    *	2	Reserved
    *	3	Reserved
    *	4	Reserved
    *	5	Latitude
    *	6	Longitude
    *	7	Reserved*/
    e_MAV_CMD_MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION = 5001,
    /**
    *Fence vertex for an exclusion polygon (the polygon must not be self-intersecting). The vehicle must stay outside this area. Minimum of 3 vertices required.
    *
    *	1	Polygon vertex count
    *	2	Reserved
    *	3	Reserved
    *	4	Reserved
    *	5	Latitude
    *	6	Longitude
    *	7	Reserved*/
    e_MAV_CMD_MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION = 5002,
    /**
    *Circular fence area. The vehicle must stay inside this area.
    *
    *	1	radius in meters
    *	2	Reserved
    *	3	Reserved
    *	4	Reserved
    *	5	Latitude
    *	6	Longitude
    *	7	Reserved*/
    e_MAV_CMD_MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION = 5003,
    /**
    *Circular fence area. The vehicle must stay outside this area.
    *
    *	1	radius in meters
    *	2	Reserved
    *	3	Reserved
    *	4	Reserved
    *	5	Latitude
    *	6	Longitude
    *	7	Reserved*/
    e_MAV_CMD_MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION = 5004,
    /**
    *Rally point. You can have multiple rally points defined.
    *
    *	1	Reserved
    *	2	Reserved
    *	3	Reserved
    *	4	Reserved
    *	5	Latitude
    *	6	Longitude
    *	7	Altitude*/
    e_MAV_CMD_MAV_CMD_NAV_RALLY_POINT = 5100,
    /**
    *Commands the vehicle to respond with a sequence of messages UAVCAN_NODE_INFO, one message per every UAVCAN
    *	node that is online. Note that some of the response messages can be lost, which the receiver can detect
    *	easily by checking whether every received UAVCAN_NODE_STATUS has a matching message UAVCAN_NODE_INFO
    *	received earlier; if not, this command should be sent again in order to request re-transmission of the
    *	node information messages
    *	1	Reserved (set to 0)
    *	2	Reserved (set to 0)
    *	3	Reserved (set to 0)
    *	4	Reserved (set to 0)
    *	5	Reserved (set to 0)
    *	6	Reserved (set to 0)
    *	7	Reserved (set to 0)*/
    e_MAV_CMD_MAV_CMD_UAVCAN_GET_NODE_INFO = 5200,
    /**
    *Deploy payload on a Lat / Lon / Alt position. This includes the navigation to reach the required release
    *	position and velocity
    *	1	Operation mode. 0: prepare single payload deploy (overwriting previous requests), but do not execute it. 1: execute payload deploy immediately (rejecting further deploy commands during execution, but allowing abort). 2: add payload deploy to existing deployment list.
    *	2	Desired approach vector in degrees compass heading (0..360). A negative value indicates the system can define the approach vector at will.
    *	3	Desired ground speed at release time. This can be overriden by the airframe in case it needs to meet minimum airspeed. A negative value indicates the system can define the ground speed at will.
    *	4	Minimum altitude clearance to the release position in meters. A negative value indicates the system can define the clearance at will.
    *	5	Latitude unscaled for MISSION_ITEM or in 1e7 degrees for MISSION_ITEM_INT
    *	6	Longitude unscaled for MISSION_ITEM or in 1e7 degrees for MISSION_ITEM_INT
    *	7	Altitude, in meters AMSL*/
    e_MAV_CMD_MAV_CMD_PAYLOAD_PREPARE_DEPLOY = 30001,
    /**
    *Control the payload deployment.
    *	1	Operation mode. 0: Abort deployment, continue normal mission. 1: switch to payload deploment mode. 100: delete first payload deployment request. 101: delete all payload deployment requests.
    *	2	Reserved
    *	3	Reserved
    *	4	Reserved
    *	5	Reserved
    *	6	Reserved
    *	7	Reserved*/
    e_MAV_CMD_MAV_CMD_PAYLOAD_CONTROL_DEPLOY = 30002,
    /**
    *User defined waypoint item. Ground Station will show the Vehicle as flying through this item.
    *	1	User defined
    *	2	User defined
    *	3	User defined
    *	4	User defined
    *	5	Latitude unscaled
    *	6	Longitude unscaled
    *	7	Altitude, in meters AMSL*/
    e_MAV_CMD_MAV_CMD_WAYPOINT_USER_1 = 31000,
    /**
    *User defined waypoint item. Ground Station will show the Vehicle as flying through this item.
    *	1	User defined
    *	2	User defined
    *	3	User defined
    *	4	User defined
    *	5	Latitude unscaled
    *	6	Longitude unscaled
    *	7	Altitude, in meters AMSL*/
    e_MAV_CMD_MAV_CMD_WAYPOINT_USER_2 = 31001,
    /**
    *User defined waypoint item. Ground Station will show the Vehicle as flying through this item.
    *	1	User defined
    *	2	User defined
    *	3	User defined
    *	4	User defined
    *	5	Latitude unscaled
    *	6	Longitude unscaled
    *	7	Altitude, in meters AMSL*/
    e_MAV_CMD_MAV_CMD_WAYPOINT_USER_3 = 31002,
    /**
    *User defined waypoint item. Ground Station will show the Vehicle as flying through this item.
    *	1	User defined
    *	2	User defined
    *	3	User defined
    *	4	User defined
    *	5	Latitude unscaled
    *	6	Longitude unscaled
    *	7	Altitude, in meters AMSL*/
    e_MAV_CMD_MAV_CMD_WAYPOINT_USER_4 = 31003,
    /**
    *User defined waypoint item. Ground Station will show the Vehicle as flying through this item.
    *	1	User defined
    *	2	User defined
    *	3	User defined
    *	4	User defined
    *	5	Latitude unscaled
    *	6	Longitude unscaled
    *	7	Altitude, in meters AMSL*/
    e_MAV_CMD_MAV_CMD_WAYPOINT_USER_5 = 31004,
    /**
    *User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example:
    *	ROI item
    *	1	User defined
    *	2	User defined
    *	3	User defined
    *	4	User defined
    *	5	Latitude unscaled
    *	6	Longitude unscaled
    *	7	Altitude, in meters AMSL*/
    e_MAV_CMD_MAV_CMD_SPATIAL_USER_1 = 31005,
    /**
    *User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example:
    *	ROI item
    *	1	User defined
    *	2	User defined
    *	3	User defined
    *	4	User defined
    *	5	Latitude unscaled
    *	6	Longitude unscaled
    *	7	Altitude, in meters AMSL*/
    e_MAV_CMD_MAV_CMD_SPATIAL_USER_2 = 31006,
    /**
    *User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example:
    *	ROI item
    *	1	User defined
    *	2	User defined
    *	3	User defined
    *	4	User defined
    *	5	Latitude unscaled
    *	6	Longitude unscaled
    *	7	Altitude, in meters AMSL*/
    e_MAV_CMD_MAV_CMD_SPATIAL_USER_3 = 31007,
    /**
    *User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example:
    *	ROI item
    *	1	User defined
    *	2	User defined
    *	3	User defined
    *	4	User defined
    *	5	Latitude unscaled
    *	6	Longitude unscaled
    *	7	Altitude, in meters AMSL*/
    e_MAV_CMD_MAV_CMD_SPATIAL_USER_4 = 31008,
    /**
    *User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example:
    *	ROI item
    *	1	User defined
    *	2	User defined
    *	3	User defined
    *	4	User defined
    *	5	Latitude unscaled
    *	6	Longitude unscaled
    *	7	Altitude, in meters AMSL*/
    e_MAV_CMD_MAV_CMD_SPATIAL_USER_5 = 31009,
    /**
    *User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER
    *	item
    *	1	User defined
    *	2	User defined
    *	3	User defined
    *	4	User defined
    *	5	User defined
    *	6	User defined
    *	7	User defined*/
    e_MAV_CMD_MAV_CMD_USER_1 = 31010,
    /**
    *User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER
    *	item
    *	1	User defined
    *	2	User defined
    *	3	User defined
    *	4	User defined
    *	5	User defined
    *	6	User defined
    *	7	User defined*/
    e_MAV_CMD_MAV_CMD_USER_2 = 31011,
    /**
    *User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER
    *	item
    *	1	User defined
    *	2	User defined
    *	3	User defined
    *	4	User defined
    *	5	User defined
    *	6	User defined
    *	7	User defined*/
    e_MAV_CMD_MAV_CMD_USER_3 = 31012,
    /**
    *User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER
    *	item
    *	1	User defined
    *	2	User defined
    *	3	User defined
    *	4	User defined
    *	5	User defined
    *	6	User defined
    *	7	User defined*/
    e_MAV_CMD_MAV_CMD_USER_4 = 31013,
    /**
    *User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER
    *	item
    *	1	User defined
    *	2	User defined
    *	3	User defined
    *	4	User defined
    *	5	User defined
    *	6	User defined
    *	7	User defined*/
    e_MAV_CMD_MAV_CMD_USER_5 = 31014
} e_MAV_CMD;

inline static UMAX _id__c(e_MAV_CMD en)
{
    switch(en)
    {
        case e_MAV_CMD_MAV_CMD_PREFLIGHT_STORAGE_ADVANCED:
            return 0;
        case e_MAV_CMD_MAV_CMD_NAV_WAYPOINT:
            return 1;
        case e_MAV_CMD_MAV_CMD_NAV_LOITER_UNLIM:
            return 2;
        case e_MAV_CMD_MAV_CMD_NAV_LOITER_TURNS:
            return 3;
        case e_MAV_CMD_MAV_CMD_NAV_LOITER_TIME:
            return 4;
        case e_MAV_CMD_MAV_CMD_NAV_RETURN_TO_LAUNCH:
            return 5;
        case e_MAV_CMD_MAV_CMD_NAV_LAND:
            return 6;
        case e_MAV_CMD_MAV_CMD_NAV_TAKEOFF:
            return 7;
        case e_MAV_CMD_MAV_CMD_NAV_LAND_LOCAL:
            return 8;
        case e_MAV_CMD_MAV_CMD_NAV_TAKEOFF_LOCAL:
            return 9;
        case e_MAV_CMD_MAV_CMD_NAV_FOLLOW:
            return 10;
        case e_MAV_CMD_MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT:
            return 11;
        case e_MAV_CMD_MAV_CMD_NAV_LOITER_TO_ALT:
            return 12;
        case e_MAV_CMD_MAV_CMD_DO_FOLLOW:
            return 13;
        case e_MAV_CMD_MAV_CMD_DO_FOLLOW_REPOSITION:
            return 14;
        case e_MAV_CMD_MAV_CMD_NAV_ROI:
            return 15;
        case e_MAV_CMD_MAV_CMD_NAV_PATHPLANNING:
            return 16;
        case e_MAV_CMD_MAV_CMD_NAV_SPLINE_WAYPOINT:
            return 17;
        case e_MAV_CMD_MAV_CMD_NAV_VTOL_TAKEOFF:
            return 18;
        case e_MAV_CMD_MAV_CMD_NAV_VTOL_LAND:
            return 19;
        case e_MAV_CMD_MAV_CMD_NAV_GUIDED_ENABLE:
            return 20;
        case e_MAV_CMD_MAV_CMD_NAV_DELAY:
            return 21;
        case e_MAV_CMD_MAV_CMD_NAV_PAYLOAD_PLACE:
            return 22;
        case e_MAV_CMD_MAV_CMD_NAV_LAST:
            return 23;
        case e_MAV_CMD_MAV_CMD_CONDITION_DELAY:
            return 24;
        case e_MAV_CMD_MAV_CMD_CONDITION_CHANGE_ALT:
            return 25;
        case e_MAV_CMD_MAV_CMD_CONDITION_DISTANCE:
            return 26;
        case e_MAV_CMD_MAV_CMD_CONDITION_YAW:
            return 27;
        case e_MAV_CMD_MAV_CMD_CONDITION_LAST:
            return 28;
        case e_MAV_CMD_MAV_CMD_DO_SET_MODE:
            return 29;
        case e_MAV_CMD_MAV_CMD_DO_JUMP:
            return 30;
        case e_MAV_CMD_MAV_CMD_DO_CHANGE_SPEED:
            return 31;
        case e_MAV_CMD_MAV_CMD_DO_SET_HOME:
            return 32;
        case e_MAV_CMD_MAV_CMD_DO_SET_PARAMETER:
            return 33;
        case e_MAV_CMD_MAV_CMD_DO_SET_RELAY:
            return 34;
        case e_MAV_CMD_MAV_CMD_DO_REPEAT_RELAY:
            return 35;
        case e_MAV_CMD_MAV_CMD_DO_SET_SERVO:
            return 36;
        case e_MAV_CMD_MAV_CMD_DO_REPEAT_SERVO:
            return 37;
        case e_MAV_CMD_MAV_CMD_DO_FLIGHTTERMINATION:
            return 38;
        case e_MAV_CMD_MAV_CMD_DO_CHANGE_ALTITUDE:
            return 39;
        case e_MAV_CMD_MAV_CMD_DO_LAND_START:
            return 40;
        case e_MAV_CMD_MAV_CMD_DO_RALLY_LAND:
            return 41;
        case e_MAV_CMD_MAV_CMD_DO_GO_AROUND:
            return 42;
        case e_MAV_CMD_MAV_CMD_DO_REPOSITION:
            return 43;
        case e_MAV_CMD_MAV_CMD_DO_PAUSE_CONTINUE:
            return 44;
        case e_MAV_CMD_MAV_CMD_DO_SET_REVERSE:
            return 45;
        case e_MAV_CMD_MAV_CMD_DO_CONTROL_VIDEO:
            return 46;
        case e_MAV_CMD_MAV_CMD_DO_SET_ROI:
            return 47;
        case e_MAV_CMD_MAV_CMD_DO_DIGICAM_CONFIGURE:
            return 48;
        case e_MAV_CMD_MAV_CMD_DO_DIGICAM_CONTROL:
            return 49;
        case e_MAV_CMD_MAV_CMD_DO_MOUNT_CONFIGURE:
            return 50;
        case e_MAV_CMD_MAV_CMD_DO_MOUNT_CONTROL:
            return 51;
        case e_MAV_CMD_MAV_CMD_DO_SET_CAM_TRIGG_DIST:
            return 52;
        case e_MAV_CMD_MAV_CMD_DO_FENCE_ENABLE:
            return 53;
        case e_MAV_CMD_MAV_CMD_DO_PARACHUTE:
            return 54;
        case e_MAV_CMD_MAV_CMD_DO_MOTOR_TEST:
            return 55;
        case e_MAV_CMD_MAV_CMD_DO_INVERTED_FLIGHT:
            return 56;
        case e_MAV_CMD_MAV_CMD_NAV_SET_YAW_SPEED:
            return 57;
        case e_MAV_CMD_MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL:
            return 58;
        case e_MAV_CMD_MAV_CMD_DO_MOUNT_CONTROL_QUAT:
            return 59;
        case e_MAV_CMD_MAV_CMD_DO_GUIDED_MASTER:
            return 60;
        case e_MAV_CMD_MAV_CMD_DO_GUIDED_LIMITS:
            return 61;
        case e_MAV_CMD_MAV_CMD_DO_ENGINE_CONTROL:
            return 62;
        case e_MAV_CMD_MAV_CMD_DO_LAST:
            return 63;
        case e_MAV_CMD_MAV_CMD_PREFLIGHT_CALIBRATION:
            return 64;
        case e_MAV_CMD_MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS:
            return 65;
        case e_MAV_CMD_MAV_CMD_PREFLIGHT_UAVCAN:
            return 66;
        case e_MAV_CMD_MAV_CMD_PREFLIGHT_STORAGE:
            return 67;
        case e_MAV_CMD_MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
            return 68;
        case e_MAV_CMD_MAV_CMD_OVERRIDE_GOTO:
            return 69;
        case e_MAV_CMD_MAV_CMD_MISSION_START:
            return 70;
        case e_MAV_CMD_MAV_CMD_COMPONENT_ARM_DISARM:
            return 71;
        case e_MAV_CMD_MAV_CMD_GET_HOME_POSITION:
            return 72;
        case e_MAV_CMD_MAV_CMD_START_RX_PAIR:
            return 73;
        case e_MAV_CMD_MAV_CMD_GET_MESSAGE_INTERVAL:
            return 74;
        case e_MAV_CMD_MAV_CMD_SET_MESSAGE_INTERVAL:
            return 75;
        case e_MAV_CMD_MAV_CMD_REQUEST_PROTOCOL_VERSION:
            return 76;
        case e_MAV_CMD_MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES:
            return 77;
        case e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_INFORMATION:
            return 78;
        case e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_SETTINGS:
            return 79;
        case e_MAV_CMD_MAV_CMD_REQUEST_STORAGE_INFORMATION:
            return 80;
        case e_MAV_CMD_MAV_CMD_STORAGE_FORMAT:
            return 81;
        case e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS:
            return 82;
        case e_MAV_CMD_MAV_CMD_REQUEST_FLIGHT_INFORMATION:
            return 83;
        case e_MAV_CMD_MAV_CMD_RESET_CAMERA_SETTINGS:
            return 84;
        case e_MAV_CMD_MAV_CMD_SET_CAMERA_MODE:
            return 85;
        case e_MAV_CMD_MAV_CMD_IMAGE_START_CAPTURE:
            return 86;
        case e_MAV_CMD_MAV_CMD_IMAGE_STOP_CAPTURE:
            return 87;
        case e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE:
            return 88;
        case e_MAV_CMD_MAV_CMD_DO_TRIGGER_CONTROL:
            return 89;
        case e_MAV_CMD_MAV_CMD_VIDEO_START_CAPTURE:
            return 90;
        case e_MAV_CMD_MAV_CMD_VIDEO_STOP_CAPTURE:
            return 91;
        case e_MAV_CMD_MAV_CMD_VIDEO_START_STREAMING:
            return 92;
        case e_MAV_CMD_MAV_CMD_VIDEO_STOP_STREAMING:
            return 93;
        case e_MAV_CMD_MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION:
            return 94;
        case e_MAV_CMD_MAV_CMD_LOGGING_START:
            return 95;
        case e_MAV_CMD_MAV_CMD_LOGGING_STOP:
            return 96;
        case e_MAV_CMD_MAV_CMD_AIRFRAME_CONFIGURATION:
            return 97;
        case e_MAV_CMD_MAV_CMD_PANORAMA_CREATE:
            return 98;
        case e_MAV_CMD_MAV_CMD_DO_VTOL_TRANSITION:
            return 99;
        case e_MAV_CMD_MAV_CMD_ARM_AUTHORIZATION_REQUEST:
            return 100;
        case e_MAV_CMD_MAV_CMD_SET_GUIDED_SUBMODE_STANDARD:
            return 101;
        case e_MAV_CMD_MAV_CMD_SET_GUIDED_SUBMODE_CIRCLE:
            return 102;
        case e_MAV_CMD_MAV_CMD_CONDITION_GATE:
            return 103;
        case e_MAV_CMD_MAV_CMD_NAV_FENCE_RETURN_POINT:
            return 104;
        case e_MAV_CMD_MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION:
            return 105;
        case e_MAV_CMD_MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION:
            return 106;
        case e_MAV_CMD_MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION:
            return 107;
        case e_MAV_CMD_MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION:
            return 108;
        case e_MAV_CMD_MAV_CMD_NAV_RALLY_POINT:
            return 109;
        case e_MAV_CMD_MAV_CMD_UAVCAN_GET_NODE_INFO:
            return 110;
        case e_MAV_CMD_MAV_CMD_PAYLOAD_PREPARE_DEPLOY:
            return 111;
        case e_MAV_CMD_MAV_CMD_PAYLOAD_CONTROL_DEPLOY:
            return 112;
        case e_MAV_CMD_MAV_CMD_WAYPOINT_USER_1:
            return 113;
        case e_MAV_CMD_MAV_CMD_WAYPOINT_USER_2:
            return 114;
        case e_MAV_CMD_MAV_CMD_WAYPOINT_USER_3:
            return 115;
        case e_MAV_CMD_MAV_CMD_WAYPOINT_USER_4:
            return 116;
        case e_MAV_CMD_MAV_CMD_WAYPOINT_USER_5:
            return 117;
        case e_MAV_CMD_MAV_CMD_SPATIAL_USER_1:
            return 118;
        case e_MAV_CMD_MAV_CMD_SPATIAL_USER_2:
            return 119;
        case e_MAV_CMD_MAV_CMD_SPATIAL_USER_3:
            return 120;
        case e_MAV_CMD_MAV_CMD_SPATIAL_USER_4:
            return 121;
        case e_MAV_CMD_MAV_CMD_SPATIAL_USER_5:
            return 122;
        case e_MAV_CMD_MAV_CMD_USER_1:
            return 123;
        case e_MAV_CMD_MAV_CMD_USER_2:
            return 124;
        case e_MAV_CMD_MAV_CMD_USER_3:
            return 125;
        case e_MAV_CMD_MAV_CMD_USER_4:
            return 126;
        case e_MAV_CMD_MAV_CMD_USER_5:
            return 127;
        default: ;// assert(false);//("Unknown enum" + id);
    }
}

/**
*result in a mavlink mission ack*/
typedef  enum
{
    e_MAV_MISSION_RESULT_MAV_MISSION_ACCEPTED = 0, //mission accepted OK
    e_MAV_MISSION_RESULT_MAV_MISSION_ERROR = 1, //generic error / not accepting mission commands at all right now
    e_MAV_MISSION_RESULT_MAV_MISSION_UNSUPPORTED_FRAME = 2, //coordinate frame is not supported
    e_MAV_MISSION_RESULT_MAV_MISSION_UNSUPPORTED = 3, //command is not supported
    e_MAV_MISSION_RESULT_MAV_MISSION_NO_SPACE = 4, //mission item exceeds storage space
    e_MAV_MISSION_RESULT_MAV_MISSION_INVALID = 5, //one of the parameters has an invalid value
    e_MAV_MISSION_RESULT_MAV_MISSION_INVALID_PARAM1 = 6, //param1 has an invalid value
    e_MAV_MISSION_RESULT_MAV_MISSION_INVALID_PARAM2 = 7, //param2 has an invalid value
    e_MAV_MISSION_RESULT_MAV_MISSION_INVALID_PARAM3 = 8, //param3 has an invalid value
    e_MAV_MISSION_RESULT_MAV_MISSION_INVALID_PARAM4 = 9, //param4 has an invalid value
    e_MAV_MISSION_RESULT_MAV_MISSION_INVALID_PARAM5_X = 10, //x/param5 has an invalid value
    e_MAV_MISSION_RESULT_MAV_MISSION_INVALID_PARAM6_Y = 11, //y/param6 has an invalid value
    e_MAV_MISSION_RESULT_MAV_MISSION_INVALID_PARAM7 = 12, //param7 has an invalid value
    e_MAV_MISSION_RESULT_MAV_MISSION_INVALID_SEQUENCE = 13, //received waypoint out of sequence
    e_MAV_MISSION_RESULT_MAV_MISSION_DENIED = 14 //not accepting any mission commands from this communication partner
} e_MAV_MISSION_RESULT;

/**
*Enumeration of estimator types*/
typedef  enum
{
    e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_NAIVE = 1, //This is a naive estimator without any real covariance feedback.
    e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_VISION = 2, //Computer vision based estimate. Might be up to scale.
    e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_VIO = 3, //Visual-inertial estimate.
    e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_GPS = 4, //Plain GPS estimate.
    e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_GPS_INS = 5 //Estimator integrating GPS and inertial sensing.
} e_MAV_ESTIMATOR_TYPE;

/**
*result from a mavlink command*/
typedef  enum
{
    e_MAV_RESULT_MAV_RESULT_ACCEPTED = 0, //Command ACCEPTED and EXECUTED
    e_MAV_RESULT_MAV_RESULT_TEMPORARILY_REJECTED = 1, //Command TEMPORARY REJECTED/DENIED
    e_MAV_RESULT_MAV_RESULT_DENIED = 2, //Command PERMANENTLY DENIED
    e_MAV_RESULT_MAV_RESULT_UNSUPPORTED = 3, //Command UNKNOWN/UNSUPPORTED
    e_MAV_RESULT_MAV_RESULT_FAILED = 4, //Command executed, but failed
    e_MAV_RESULT_MAV_RESULT_IN_PROGRESS = 5 //WIP: Command being executed
} e_MAV_RESULT;

/**
*Power supply status flags (bitmask)*/
typedef  enum
{
    e_MAV_POWER_STATUS_MAV_POWER_STATUS_BRICK_VALID = 1, //main brick power supply valid
    e_MAV_POWER_STATUS_MAV_POWER_STATUS_SERVO_VALID = 2, //main servo power supply valid for FMU
    e_MAV_POWER_STATUS_MAV_POWER_STATUS_USB_CONNECTED = 4, //USB power is connected
    e_MAV_POWER_STATUS_MAV_POWER_STATUS_PERIPH_OVERCURRENT = 8, //peripheral supply is in over-current state
    e_MAV_POWER_STATUS_MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT = 16, //hi-power peripheral supply is in over-current state
    e_MAV_POWER_STATUS_MAV_POWER_STATUS_CHANGED = 32 //Power status has changed since boot
} e_MAV_POWER_STATUS;

/**
*SERIAL_CONTROL device types*/
typedef  enum
{
    e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_TELEM1 = 0, //First telemetry port
    e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_TELEM2 = 1, //Second telemetry port
    e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_GPS1 = 2, //First GPS port
    e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_GPS2 = 3, //Second GPS port
    e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_SHELL = 10 //system shell
} e_SERIAL_CONTROL_DEV;

/**
*SERIAL_CONTROL flags (bitmask)*/
typedef  enum
{
    e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_REPLY = 1, //Set if this is a reply
    e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_RESPOND = 2, //Set if the sender wants the receiver to send a response as another SERIAL_CONTROL message
    /**
    *Set if access to the serial port should be removed from whatever driver is currently using it, giving
    *	exclusive access to the SERIAL_CONTROL protocol. The port can be handed back by sending a request without
    *	this flag se*/
    e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_EXCLUSIVE = 4,
    e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_BLOCKING = 8, //Block on writes to the serial port
    e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_MULTI = 16 //Send multiple replies until port is drained
} e_SERIAL_CONTROL_FLAG;

/**
*Enumeration of distance sensor types*/
typedef  enum
{
    e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_LASER = 0, //Laser rangefinder, e.g. LightWare SF02/F or PulsedLight units
    e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_ULTRASOUND = 1, //Ultrasound rangefinder, e.g. MaxBotix units
    e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_INFRARED = 2, //Infrared rangefinder, e.g. Sharp units
    e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_RADAR = 3, //Radar type, e.g. uLanding units
    e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_UNKNOWN = 4 //Broken or unknown type, e.g. analog units
} e_MAV_DISTANCE_SENSOR;

/**
*Enumeration of sensor orientation, according to its rotations*/
typedef  enum
{
    e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_NONE = 0, //Roll: 0, Pitch: 0, Yaw: 0
    e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_YAW_45 = 1, //Roll: 0, Pitch: 0, Yaw: 45
    e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_YAW_90 = 2, //Roll: 0, Pitch: 0, Yaw: 90
    e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_YAW_135 = 3, //Roll: 0, Pitch: 0, Yaw: 135
    e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_YAW_180 = 4, //Roll: 0, Pitch: 0, Yaw: 180
    e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_YAW_225 = 5, //Roll: 0, Pitch: 0, Yaw: 225
    e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_YAW_270 = 6, //Roll: 0, Pitch: 0, Yaw: 270
    e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_YAW_315 = 7, //Roll: 0, Pitch: 0, Yaw: 315
    e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_ROLL_180 = 8, //Roll: 180, Pitch: 0, Yaw: 0
    e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_ROLL_180_YAW_45 = 9, //Roll: 180, Pitch: 0, Yaw: 45
    e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_ROLL_180_YAW_90 = 10, //Roll: 180, Pitch: 0, Yaw: 90
    e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_ROLL_180_YAW_135 = 11, //Roll: 180, Pitch: 0, Yaw: 135
    e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_PITCH_180 = 12, //Roll: 0, Pitch: 180, Yaw: 0
    e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_ROLL_180_YAW_225 = 13, //Roll: 180, Pitch: 0, Yaw: 225
    e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_ROLL_180_YAW_270 = 14, //Roll: 180, Pitch: 0, Yaw: 270
    e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_ROLL_180_YAW_315 = 15, //Roll: 180, Pitch: 0, Yaw: 315
    e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_ROLL_90 = 16, //Roll: 90, Pitch: 0, Yaw: 0
    e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_ROLL_90_YAW_45 = 17, //Roll: 90, Pitch: 0, Yaw: 45
    e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_ROLL_90_YAW_90 = 18, //Roll: 90, Pitch: 0, Yaw: 90
    e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_ROLL_90_YAW_135 = 19, //Roll: 90, Pitch: 0, Yaw: 135
    e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_ROLL_270 = 20, //Roll: 270, Pitch: 0, Yaw: 0
    e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_ROLL_270_YAW_45 = 21, //Roll: 270, Pitch: 0, Yaw: 45
    e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_ROLL_270_YAW_90 = 22, //Roll: 270, Pitch: 0, Yaw: 90
    e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_ROLL_270_YAW_135 = 23, //Roll: 270, Pitch: 0, Yaw: 135
    e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_PITCH_90 = 24, //Roll: 0, Pitch: 90, Yaw: 0
    e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_PITCH_270 = 25, //Roll: 0, Pitch: 270, Yaw: 0
    e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_PITCH_180_YAW_90 = 26, //Roll: 0, Pitch: 180, Yaw: 90
    e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_PITCH_180_YAW_270 = 27, //Roll: 0, Pitch: 180, Yaw: 270
    e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_ROLL_90_PITCH_90 = 28, //Roll: 90, Pitch: 90, Yaw: 0
    e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_ROLL_180_PITCH_90 = 29, //Roll: 180, Pitch: 90, Yaw: 0
    e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_ROLL_270_PITCH_90 = 30, //Roll: 270, Pitch: 90, Yaw: 0
    e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_ROLL_90_PITCH_180 = 31, //Roll: 90, Pitch: 180, Yaw: 0
    e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_ROLL_270_PITCH_180 = 32, //Roll: 270, Pitch: 180, Yaw: 0
    e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_ROLL_90_PITCH_270 = 33, //Roll: 90, Pitch: 270, Yaw: 0
    e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_ROLL_180_PITCH_270 = 34, //Roll: 180, Pitch: 270, Yaw: 0
    e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_ROLL_270_PITCH_270 = 35, //Roll: 270, Pitch: 270, Yaw: 0
    e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_ROLL_90_PITCH_180_YAW_90 = 36, //Roll: 90, Pitch: 180, Yaw: 90
    e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_ROLL_90_YAW_270 = 37, //Roll: 90, Pitch: 0, Yaw: 270
    e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_ROLL_315_PITCH_315_YAW_315 = 38 //Roll: 315, Pitch: 315, Yaw: 315
} e_MAV_SENSOR_ORIENTATION;

/**
*Enumeration of battery functions*/
typedef  enum
{
    e_MAV_BATTERY_FUNCTION_MAV_BATTERY_FUNCTION_UNKNOWN = 0, //Battery function is unknown
    e_MAV_BATTERY_FUNCTION_MAV_BATTERY_FUNCTION_ALL = 1, //Battery supports all flight systems
    e_MAV_BATTERY_FUNCTION_MAV_BATTERY_FUNCTION_PROPULSION = 2, //Battery for the propulsion system
    e_MAV_BATTERY_FUNCTION_MAV_BATTERY_FUNCTION_AVIONICS = 3, //Avionics battery
    e_MAV_BATTERY_FUNCTION_MAV_BATTERY_TYPE_PAYLOAD = 4 //Payload battery
} e_MAV_BATTERY_FUNCTION;

/**
*Enumeration of battery types*/
typedef  enum
{
    e_MAV_BATTERY_TYPE_MAV_BATTERY_TYPE_UNKNOWN = 0, //Not specified.
    e_MAV_BATTERY_TYPE_MAV_BATTERY_TYPE_LIPO = 1, //Lithium polymer battery
    e_MAV_BATTERY_TYPE_MAV_BATTERY_TYPE_LIFE = 2, //Lithium-iron-phosphate battery
    e_MAV_BATTERY_TYPE_MAV_BATTERY_TYPE_LION = 3, //Lithium-ION battery
    e_MAV_BATTERY_TYPE_MAV_BATTERY_TYPE_NIMH = 4 //Nickel metal hydride battery
} e_MAV_BATTERY_TYPE;

/**
*Bitmask of (optional) autopilot capabilities (64 bit). If a bit is set, the autopilot supports this capability*/
typedef  enum
{
    e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT = 1, //Autopilot supports MISSION float message type.
    e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT = 2, //Autopilot supports the new param float message type.
    e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_MISSION_INT = 4, //Autopilot supports MISSION_INT scaled integer message type.
    e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_COMMAND_INT = 8, //Autopilot supports COMMAND_INT scaled integer message type.
    e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_PARAM_UNION = 16, //Autopilot supports the new param union message type.
    e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_FTP = 32, //Autopilot supports the new FILE_TRANSFER_PROTOCOL message type.
    e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET = 64, //Autopilot supports commanding attitude offboard.
    e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED = 128, //Autopilot supports commanding position and velocity targets in local NED frame.
    e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT = 256, //Autopilot supports commanding position and velocity targets in global scaled integers.
    e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_TERRAIN = 512, //Autopilot supports terrain protocol / data handling.
    e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_SET_ACTUATOR_TARGET = 1024, //Autopilot supports direct actuator control.
    e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION = 2048, //Autopilot supports the flight termination command.
    e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION = 4096, //Autopilot supports onboard compass calibration.
    e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_MAVLINK2 = 8192, //Autopilot supports mavlink version 2.
    e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_MISSION_FENCE = 16384, //Autopilot supports mission fence protocol.
    e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_MISSION_RALLY = 32768, //Autopilot supports mission rally point protocol.
    e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_FLIGHT_INFORMATION = 65536 //Autopilot supports the flight information protocol.
} e_MAV_PROTOCOL_CAPABILITY;

/**
*Type of landing target*/
typedef  enum
{
    e_LANDING_TARGET_TYPE_LANDING_TARGET_TYPE_LIGHT_BEACON = 0, //Landing target signaled by light beacon (ex: IR-LOCK)
    e_LANDING_TARGET_TYPE_LANDING_TARGET_TYPE_RADIO_BEACON = 1, //Landing target signaled by radio beacon (ex: ILS, NDB)
    e_LANDING_TARGET_TYPE_LANDING_TARGET_TYPE_VISION_FIDUCIAL = 2, //Landing target represented by a fiducial marker (ex: ARTag)
    e_LANDING_TARGET_TYPE_LANDING_TARGET_TYPE_VISION_OTHER = 3 //Landing target represented by a pre-defined visual shape/feature (ex: X-marker, H-marker, square)
} e_LANDING_TARGET_TYPE;

/**
*Flags in EKF_STATUS message*/
typedef  enum
{
    e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_ATTITUDE = 1, //True if the attitude estimate is good
    e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_VELOCITY_HORIZ = 2, //True if the horizontal velocity estimate is good
    e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_VELOCITY_VERT = 4, //True if the  vertical velocity estimate is good
    e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_POS_HORIZ_REL = 8, //True if the horizontal position (relative) estimate is good
    e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_POS_HORIZ_ABS = 16, //True if the horizontal position (absolute) estimate is good
    e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_POS_VERT_ABS = 32, //True if the vertical position (absolute) estimate is good
    e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_POS_VERT_AGL = 64, //True if the vertical position (above ground) estimate is good
    /**
    *True if the EKF is in a constant position mode and is not using external measurements (eg GPS or optical
    *	flow*/
    e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_CONST_POS_MODE = 128,
    e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_PRED_POS_HORIZ_REL = 256, //True if the EKF has sufficient data to enter a mode that will provide a (relative) position estimat
    e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_PRED_POS_HORIZ_ABS = 512, //True if the EKF has sufficient data to enter a mode that will provide a (absolute) position estimat
    e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_GPS_GLITCH = 1024 //True if the EKF has detected a GPS glitch
} e_ESTIMATOR_STATUS_FLAGS;

typedef  enum
{
    e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_ALT = 1, //ignore altitude field
    e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_HDOP = 2, //ignore hdop field
    e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_VDOP = 4, //ignore vdop field
    e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_VEL_HORIZ = 8, //ignore horizontal velocity field (vn and ve)
    e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_VEL_VERT = 16, //ignore vertical velocity field (vd)
    e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY = 32, //ignore speed accuracy field
    e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY = 64, //ignore horizontal accuracy field
    e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY = 128 //ignore vertical accuracy field
} e_GPS_INPUT_IGNORE_FLAGS;

/**
*Enumeration of landed detector states*/
typedef  enum
{
    e_MAV_LANDED_STATE_MAV_LANDED_STATE_UNDEFINED = 0, //MAV landed state is unknown
    e_MAV_LANDED_STATE_MAV_LANDED_STATE_ON_GROUND = 1, //MAV is landed (on ground)
    e_MAV_LANDED_STATE_MAV_LANDED_STATE_IN_AIR = 2, //MAV is in air
    e_MAV_LANDED_STATE_MAV_LANDED_STATE_TAKEOFF = 3, //MAV currently taking off
    e_MAV_LANDED_STATE_MAV_LANDED_STATE_LANDING = 4 //MAV currently landing
} e_MAV_LANDED_STATE;

/**
*Enumeration of VTOL states*/
typedef  enum
{
    e_MAV_VTOL_STATE_MAV_VTOL_STATE_UNDEFINED = 0, //MAV is not configured as VTOL
    e_MAV_VTOL_STATE_MAV_VTOL_STATE_TRANSITION_TO_FW = 1, //VTOL is in transition from multicopter to fixed-wing
    e_MAV_VTOL_STATE_MAV_VTOL_STATE_TRANSITION_TO_MC = 2, //VTOL is in transition from fixed-wing to multicopter
    e_MAV_VTOL_STATE_MAV_VTOL_STATE_MC = 3, //VTOL is in multicopter state
    e_MAV_VTOL_STATE_MAV_VTOL_STATE_FW = 4 //VTOL is in fixed-wing state
} e_MAV_VTOL_STATE;

/**
*Enumeration of the ADSB altimeter types*/
typedef  enum
{
    e_ADSB_ALTITUDE_TYPE_ADSB_ALTITUDE_TYPE_PRESSURE_QNH = 0, //Altitude reported from a Baro source using QNH reference
    e_ADSB_ALTITUDE_TYPE_ADSB_ALTITUDE_TYPE_GEOMETRIC = 1 //Altitude reported from a GNSS source
} e_ADSB_ALTITUDE_TYPE;

/**
*ADSB classification for the type of vehicle emitting the transponder signal*/
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

/**
*These flags indicate status such as data validity of each data source. Set = data valid*/
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

/**
*Source of information about this collision.*/
typedef  enum
{
    e_MAV_COLLISION_SRC_MAV_COLLISION_SRC_ADSB = 0, //ID field references ADSB_VEHICLE packets
    e_MAV_COLLISION_SRC_MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT = 1 //ID field references MAVLink SRC ID
} e_MAV_COLLISION_SRC;

/**
*Possible actions an aircraft can take to avoid a collision.*/
typedef  enum
{
    e_MAV_COLLISION_ACTION_MAV_COLLISION_ACTION_NONE = 0, //Ignore any potential collisions
    e_MAV_COLLISION_ACTION_MAV_COLLISION_ACTION_REPORT = 1, //Report potential collision
    e_MAV_COLLISION_ACTION_MAV_COLLISION_ACTION_ASCEND_OR_DESCEND = 2, //Ascend or Descend to avoid threat
    e_MAV_COLLISION_ACTION_MAV_COLLISION_ACTION_MOVE_HORIZONTALLY = 3, //Move horizontally to avoid threat
    e_MAV_COLLISION_ACTION_MAV_COLLISION_ACTION_MOVE_PERPENDICULAR = 4, //Aircraft to move perpendicular to the collision's velocity vector
    e_MAV_COLLISION_ACTION_MAV_COLLISION_ACTION_RTL = 5, //Aircraft to fly directly back to its launch point
    e_MAV_COLLISION_ACTION_MAV_COLLISION_ACTION_HOVER = 6 //Aircraft to stop in place
} e_MAV_COLLISION_ACTION;

/**
*Aircraft-rated danger from this threat.*/
typedef  enum
{
    e_MAV_COLLISION_THREAT_LEVEL_MAV_COLLISION_THREAT_LEVEL_NONE = 0, //Not a threat
    e_MAV_COLLISION_THREAT_LEVEL_MAV_COLLISION_THREAT_LEVEL_LOW = 1, //Craft is mildly concerned about this threat
    e_MAV_COLLISION_THREAT_LEVEL_MAV_COLLISION_THREAT_LEVEL_HIGH = 2 //Craft is panicing, and may take actions to avoid threat
} e_MAV_COLLISION_THREAT_LEVEL;

/**
*Indicates the severity level, generally used for status messages to indicate their relative urgency. Based
*	on RFC-5424 using expanded definitions at: http:www.kiwisyslog.com/kb/info:-syslog-message-levels/*/
typedef  enum
{
    e_MAV_SEVERITY_MAV_SEVERITY_EMERGENCY = 0, //System is unusable. This is a "panic" condition.
    e_MAV_SEVERITY_MAV_SEVERITY_ALERT = 1, //Action should be taken immediately. Indicates error in non-critical systems.
    e_MAV_SEVERITY_MAV_SEVERITY_CRITICAL = 2, //Action must be taken immediately. Indicates failure in a primary system.
    e_MAV_SEVERITY_MAV_SEVERITY_ERROR = 3, //Indicates an error in secondary/redundant systems.
    /**
    *Indicates about a possible future error if this is not resolved within a given timeframe. Example would
    *	be a low battery warning*/
    e_MAV_SEVERITY_MAV_SEVERITY_WARNING = 4,
    /**
    *An unusual event has occured, though not an error condition. This should be investigated for the root
    *	cause*/
    e_MAV_SEVERITY_MAV_SEVERITY_NOTICE = 5,
    e_MAV_SEVERITY_MAV_SEVERITY_INFO = 6, //Normal operational messages. Useful for logging. No action is required for these messages.
    e_MAV_SEVERITY_MAV_SEVERITY_DEBUG = 7 //Useful non-operational messages that can assist in debugging. These should not occur during normal operation
} e_MAV_SEVERITY;

/**
*Camera capability flags (Bitmap).*/
typedef  enum
{
    e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_CAPTURE_VIDEO = 1, //Camera is able to record video.
    e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_CAPTURE_IMAGE = 2, //Camera is able to capture images.
    e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_HAS_MODES = 4, //Camera has separate Video and Image/Photo modes (MAV_CMD_SET_CAMERA_MODE)
    e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE = 8, //Camera can capture images while in video mode
    e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE = 16, //Camera can capture videos while in Photo/Image mode
    e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE = 32 //Camera has image survey mode (MAV_CMD_SET_CAMERA_MODE)
} e_CAMERA_CAP_FLAGS;

/**
*Camera Modes.*/
typedef  enum
{
    e_CAMERA_MODE_CAMERA_MODE_IMAGE = 0, //Camera is in image/photo capture mode.
    e_CAMERA_MODE_CAMERA_MODE_VIDEO = 1, //Camera is in video capture mode.
    e_CAMERA_MODE_CAMERA_MODE_IMAGE_SURVEY = 2 //Camera is in image survey capture mode. It allows for camera controller to do specific settings for surveys
} e_CAMERA_MODE;

/**
*Generalized UAVCAN node health*/
typedef  enum
{
    e_UAVCAN_NODE_HEALTH_UAVCAN_NODE_HEALTH_OK = 0, //The node is functioning properly.
    e_UAVCAN_NODE_HEALTH_UAVCAN_NODE_HEALTH_WARNING = 1, //A critical parameter went out of range or the node has encountered a minor failure.
    e_UAVCAN_NODE_HEALTH_UAVCAN_NODE_HEALTH_ERROR = 2, //The node has encountered a major failure.
    e_UAVCAN_NODE_HEALTH_UAVCAN_NODE_HEALTH_CRITICAL = 3 //The node has suffered a fatal malfunction.
} e_UAVCAN_NODE_HEALTH;

/**
*Generalized UAVCAN node mode*/
typedef  enum
{
    e_UAVCAN_NODE_MODE_UAVCAN_NODE_MODE_OPERATIONAL = 0, //The node is performing its primary functions.
    e_UAVCAN_NODE_MODE_UAVCAN_NODE_MODE_INITIALIZATION = 1, //The node is initializing; this mode is entered immediately after startup.
    e_UAVCAN_NODE_MODE_UAVCAN_NODE_MODE_MAINTENANCE = 2, //The node is under maintenance.
    e_UAVCAN_NODE_MODE_UAVCAN_NODE_MODE_SOFTWARE_UPDATE = 3, //The node is in the process of updating its software.
    e_UAVCAN_NODE_MODE_UAVCAN_NODE_MODE_OFFLINE = 7 //The node is no longer available online.
} e_UAVCAN_NODE_MODE;

/**
*Specifies the datatype of a MAVLink extended parameter.*/
typedef  enum
{
    e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_UINT8 = 1, //8-bit unsigned integer
    e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_INT8 = 2, //8-bit signed integer
    e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_UINT16 = 3, //16-bit unsigned integer
    e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_INT16 = 4, //16-bit signed integer
    e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_UINT32 = 5, //32-bit unsigned integer
    e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_INT32 = 6, //32-bit signed integer
    e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_UINT64 = 7, //64-bit unsigned integer
    e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_INT64 = 8, //64-bit signed integer
    e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_REAL32 = 9, //32-bit floating-point
    e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_REAL64 = 10, //64-bit floating-point
    e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_CUSTOM = 11 //Custom Type
} e_MAV_PARAM_EXT_TYPE;

/**
*Result from a PARAM_EXT_SET message.*/
typedef  enum
{
    e_PARAM_ACK_PARAM_ACK_ACCEPTED = 0, //Parameter value ACCEPTED and SET
    e_PARAM_ACK_PARAM_ACK_VALUE_UNSUPPORTED = 1, //Parameter value UNKNOWN/UNSUPPORTED
    e_PARAM_ACK_PARAM_ACK_FAILED = 2, //Parameter failed to set
    /**
    *Parameter value received but not yet validated or set. A subsequent PARAM_EXT_ACK will follow once operation
    *	is completed with the actual result. These are for parameters that may take longer to set. Instead of
    *	waiting for an ACK and potentially timing out, you will immediately receive this response to let you
    *	know it was received*/
    e_PARAM_ACK_PARAM_ACK_IN_PROGRESS = 3
} e_PARAM_ACK;

INLINER void p0_custom_mode_SET(uint32_t  src, Pack * dst)//A bitfield for use for autopilot-specific flags.
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  0);
}
INLINER void p0_mavlink_version_SET(uint8_t  src, Pack * dst)//MAVLink version, not writable by user, gets added by protocol because of magic data type: uint8_t_mavlink_versio
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  4);
}
INLINER void p0_type_SET(e_MAV_TYPE  src, Pack * dst)//Type of the MAV (quadrotor, helicopter, etc., up to 15 types, defined in MAV_TYPE ENUM)
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 5, data, 40);
}
INLINER void p0_autopilot_SET(e_MAV_AUTOPILOT  src, Pack * dst)//Autopilot type / class. defined in MAV_AUTOPILOT ENUM
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 5, data, 45);
}
INLINER void p0_base_mode_SET(e_MAV_MODE_FLAG  src, Pack * dst)//System mode bitfield, see MAV_MODE_FLAG ENUM in mavlink/include/mavlink_types.h
{
    uint8_t * data = dst->data;
    set_bits(- 1 +   src, 8, data, 50);
}
INLINER void p0_system_status_SET(e_MAV_STATE  src, Pack * dst)//System status flag, see MAV_STATE ENUM
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 4, data, 58);
}
INLINER void p1_load_SET(uint16_t  src, Pack * dst)//Maximum usage in percent of the mainloop time, (0%: 0, 100%: 1000) should be always below 1000
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER void p1_voltage_battery_SET(uint16_t  src, Pack * dst)//Battery voltage, in millivolts (1 = 1 millivolt)
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  2);
}
/**
*Communication drops in percent, (0%: 0, 100%: 10'000), (UART, I2C, SPI, CAN), dropped packets on all links
*	(packets that were corrupted on reception on the MAV*/
INLINER void p1_drop_rate_comm_SET(uint16_t  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  4);
}
/**
*Communication errors (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted
*	on reception on the MAV*/
INLINER void p1_errors_comm_SET(uint16_t  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  6);
}
INLINER void p1_errors_count1_SET(uint16_t  src, Pack * dst)//Autopilot-specific errors
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  8);
}
INLINER void p1_errors_count2_SET(uint16_t  src, Pack * dst)//Autopilot-specific errors
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  10);
}
INLINER void p1_errors_count3_SET(uint16_t  src, Pack * dst)//Autopilot-specific errors
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  12);
}
INLINER void p1_errors_count4_SET(uint16_t  src, Pack * dst)//Autopilot-specific errors
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  14);
}
INLINER void p1_current_battery_SET(int16_t  src, Pack * dst)//Battery current, in 10*milliamperes (1 = 10 milliampere), -1: autopilot does not measure the curren
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  16);
}
INLINER void p1_battery_remaining_SET(int8_t  src, Pack * dst)//Remaining battery energy: (0%: 0, 100%: 100), -1: autopilot estimate the remaining battery
{
    uint8_t * data = dst->data;
    set_bytes((uint8_t)(src), 1, data,  18);
}
/**
*Bitmask showing which onboard controllers and sensors are present. Value of 0: not present. Value of 1:
*	present. Indices defined by ENUM MAV_SYS_STATUS_SENSO*/
INLINER void p1_onboard_control_sensors_present_SET(e_MAV_SYS_STATUS_SENSOR  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bits(- 1 +   src, 26, data, 152);
}
/**
*Bitmask showing which onboard controllers and sensors are enabled:  Value of 0: not enabled. Value of
*	1: enabled. Indices defined by ENUM MAV_SYS_STATUS_SENSO*/
INLINER void p1_onboard_control_sensors_enabled_SET(e_MAV_SYS_STATUS_SENSOR  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bits(- 1 +   src, 26, data, 178);
}
/**
*Bitmask showing which onboard controllers and sensors are operational or have an error:  Value of 0: not
*	enabled. Value of 1: enabled. Indices defined by ENUM MAV_SYS_STATUS_SENSO*/
INLINER void p1_onboard_control_sensors_health_SET(e_MAV_SYS_STATUS_SENSOR  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bits(- 1 +   src, 26, data, 204);
}
INLINER void p2_time_boot_ms_SET(uint32_t  src, Pack * dst)//Timestamp of the component clock since boot time in milliseconds.
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  0);
}
INLINER void p2_time_unix_usec_SET(uint64_t  src, Pack * dst)//Timestamp of the master clock in microseconds since UNIX epoch.
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  4);
}
/**
*Bitmask to indicate which dimensions should be ignored by the vehicle: a value of 0b0000000000000000 or
*	0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. If bit 10 is set
*	the floats afx afy afz should be interpreted as force instead of acceleration. Mapping: bit 1: x, bit
*	2: y, bit 3: z, bit 4: vx, bit 5: vy, bit 6: vz, bit 7: ax, bit 8: ay, bit 9: az, bit 10: is force setpoint,
*	bit 11: yaw, bit 12: yaw rat*/
INLINER void p3_type_mask_SET(uint16_t  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER void p3_time_boot_ms_SET(uint32_t  src, Pack * dst)//Timestamp in milliseconds since system boot
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  2);
}
INLINER void p3_x_SET(float  src, Pack * dst)//X Position in NED frame in meters
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  6);
}
INLINER void p3_y_SET(float  src, Pack * dst)//Y Position in NED frame in meters
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  10);
}
INLINER void p3_z_SET(float  src, Pack * dst)//Z Position in NED frame in meters (note, altitude is negative in NED)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  14);
}
INLINER void p3_vx_SET(float  src, Pack * dst)//X velocity in NED frame in meter / s
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  18);
}
INLINER void p3_vy_SET(float  src, Pack * dst)//Y velocity in NED frame in meter / s
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  22);
}
INLINER void p3_vz_SET(float  src, Pack * dst)//Z velocity in NED frame in meter / s
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  26);
}
INLINER void p3_afx_SET(float  src, Pack * dst)//X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  30);
}
INLINER void p3_afy_SET(float  src, Pack * dst)//Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  34);
}
INLINER void p3_afz_SET(float  src, Pack * dst)//Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  38);
}
INLINER void p3_yaw_SET(float  src, Pack * dst)//yaw setpoint in rad
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  42);
}
INLINER void p3_yaw_rate_SET(float  src, Pack * dst)//yaw rate setpoint in rad/s
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  46);
}
/**
*Valid options are: MAV_FRAME_LOCAL_NED = 1, MAV_FRAME_LOCAL_OFFSET_NED = 7, MAV_FRAME_BODY_NED = 8, MAV_FRAME_BODY_OFFSET_NED
*	=*/
INLINER void p3_coordinate_frame_SET(e_MAV_FRAME  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 4, data, 400);
}
INLINER void p4_seq_SET(uint32_t  src, Pack * dst)//PING sequence
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  0);
}
INLINER void p4_time_usec_SET(uint64_t  src, Pack * dst)//Unix timestamp in microseconds or since system boot if smaller than MAVLink epoch (1.1.2009)
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  4);
}
/**
*0: request ping from all receiving systems, if greater than 0: message is a ping response and number is
*	the system id of the requesting syste*/
INLINER void p4_target_system_SET(uint8_t  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  12);
}
/**
*0: request ping from all receiving components, if greater than 0: message is a ping response and number
*	is the system id of the requesting syste*/
INLINER void p4_target_component_SET(uint8_t  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  13);
}
INLINER void p5_target_system_SET(uint8_t  src, Pack * dst)//System the GCS requests control for
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  0);
}
INLINER void p5_control_request_SET(uint8_t  src, Pack * dst)//0: request control of this MAV, 1: Release control of this MAV
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  1);
}
/**
*0: key as plaintext, 1-255: future, different hashing/encryption variants. The GCS should in general use
*	the safest mode possible initially and then gradually move down the encryption level if it gets a NACK
*	message indicating an encryption mismatch*/
INLINER void p5_version_SET(uint8_t  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  2);
}
/**
*Password / Key, depending on version plaintext or encrypted. 25 or less characters, NULL terminated. The
*	characters may involve A-Z, a-z, 0-9, and "!?,.-*/
INLINER void p5_passkey_SET(char16_t *  src, int32_t pos, int32_t items, Bounds_Inside * dst)
{
    if(dst->base.field_bit != 24 && insert_field(dst, 24, items) ||
            ! try_visit_item(dst, 0)) insert_item(dst, 0, items);
    uint8_t * data = dst->base.pack->data;
    for(int32_t BYTE =  dst->BYTE, src_max = pos + dst->items; pos < src_max; pos++, BYTE += 2)
        set_bytes((uint16_t)(src[pos]), 2, data,  BYTE);
}/**
*Password / Key, depending on version plaintext or encrypted. 25 or less characters, NULL terminated. The
*	characters may involve A-Z, a-z, 0-9, and "!?,.-*/
INLINER void p5_passkey_SET_(char16_t*  src, Bounds_Inside * dst) {p5_passkey_SET(src, 0, strlen16(src), dst);}
INLINER void p6_gcs_system_id_SET(uint8_t  src, Pack * dst)//ID of the GCS this message
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  0);
}
INLINER void p6_control_request_SET(uint8_t  src, Pack * dst)//0: request control of this MAV, 1: Release control of this MAV
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  1);
}
/**
*0: ACK, 1: NACK: Wrong passkey, 2: NACK: Unsupported passkey encryption method, 3: NACK: Already under
*	contro*/
INLINER void p6_ack_SET(uint8_t  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  2);
}
INLINER void p7_key_SET(char16_t *  src, int32_t pos, int32_t items, Bounds_Inside * dst) //key
{
    if(dst->base.field_bit != 0 && insert_field(dst, 0, items) ||
            ! try_visit_item(dst, 0)) insert_item(dst, 0, items);
    uint8_t * data = dst->base.pack->data;
    for(int32_t BYTE =  dst->BYTE, src_max = pos + dst->items; pos < src_max; pos++, BYTE += 2)
        set_bytes((uint16_t)(src[pos]), 2, data,  BYTE);
}
INLINER void p7_key_SET_(char16_t*  src, Bounds_Inside * dst) {p7_key_SET(src, 0, strlen16(src), dst);}
INLINER void p11_custom_mode_SET(uint32_t  src, Pack * dst)//The new autopilot-specific mode. This field can be ignored by an autopilot.
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  0);
}
INLINER void p11_target_system_SET(uint8_t  src, Pack * dst)//The system setting the mode
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  4);
}
INLINER void p11_base_mode_SET(e_MAV_MODE  src, Pack * dst)//The new base mode
{
    uint8_t * data = dst->data;
    UMAX id = _id__a(src);
    set_bits(id, 4, data, 40);
}
INLINER void p20_target_system_SET(uint8_t  src, Pack * dst)//System ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  0);
}
INLINER void p20_target_component_SET(uint8_t  src, Pack * dst)//Component ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  1);
}
INLINER void p20_param_index_SET(int16_t  src, Pack * dst)//Parameter index. Send -1 to use the param ID field as identifier (else the param id will be ignored
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  2);
}
/**
*Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT
*	null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes
*	storage if the ID is stored as strin*/
INLINER void p20_param_id_SET(char16_t *  src, int32_t pos, int32_t items, Bounds_Inside * dst)
{
    if(dst->base.field_bit != 32 && insert_field(dst, 32, items) ||
            ! try_visit_item(dst, 0)) insert_item(dst, 0, items);
    uint8_t * data = dst->base.pack->data;
    for(int32_t BYTE =  dst->BYTE, src_max = pos + dst->items; pos < src_max; pos++, BYTE += 2)
        set_bytes((uint16_t)(src[pos]), 2, data,  BYTE);
}/**
*Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT
*	null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes
*	storage if the ID is stored as strin*/
INLINER void p20_param_id_SET_(char16_t*  src, Bounds_Inside * dst) {p20_param_id_SET(src, 0, strlen16(src), dst);}
INLINER void p21_target_system_SET(uint8_t  src, Pack * dst)//System ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  0);
}
INLINER void p21_target_component_SET(uint8_t  src, Pack * dst)//Component ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  1);
}
INLINER void p22_param_count_SET(uint16_t  src, Pack * dst)//Total number of onboard parameters
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER void p22_param_index_SET(uint16_t  src, Pack * dst)//Index of this onboard parameter
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  2);
}
INLINER void p22_param_value_SET(float  src, Pack * dst)//Onboard parameter value
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  4);
}
INLINER void p22_param_type_SET(e_MAV_PARAM_TYPE  src, Pack * dst)//Onboard parameter type: see the MAV_PARAM_TYPE enum for supported data types.
{
    uint8_t * data = dst->data;
    set_bits(- 1 +   src, 4, data, 64);
}
/**
*Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT
*	null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes
*	storage if the ID is stored as strin*/
INLINER void p22_param_id_SET(char16_t *  src, int32_t pos, int32_t items, Bounds_Inside * dst)
{
    if(dst->base.field_bit != 68 && insert_field(dst, 68, items) ||
            ! try_visit_item(dst, 0)) insert_item(dst, 0, items);
    uint8_t * data = dst->base.pack->data;
    for(int32_t BYTE =  dst->BYTE, src_max = pos + dst->items; pos < src_max; pos++, BYTE += 2)
        set_bytes((uint16_t)(src[pos]), 2, data,  BYTE);
}/**
*Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT
*	null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes
*	storage if the ID is stored as strin*/
INLINER void p22_param_id_SET_(char16_t*  src, Bounds_Inside * dst) {p22_param_id_SET(src, 0, strlen16(src), dst);}
INLINER void p23_target_system_SET(uint8_t  src, Pack * dst)//System ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  0);
}
INLINER void p23_target_component_SET(uint8_t  src, Pack * dst)//Component ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  1);
}
INLINER void p23_param_value_SET(float  src, Pack * dst)//Onboard parameter value
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  2);
}
INLINER void p23_param_type_SET(e_MAV_PARAM_TYPE  src, Pack * dst)//Onboard parameter type: see the MAV_PARAM_TYPE enum for supported data types.
{
    uint8_t * data = dst->data;
    set_bits(- 1 +   src, 4, data, 48);
}
/**
*Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT
*	null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes
*	storage if the ID is stored as strin*/
INLINER void p23_param_id_SET(char16_t *  src, int32_t pos, int32_t items, Bounds_Inside * dst)
{
    if(dst->base.field_bit != 52 && insert_field(dst, 52, items) ||
            ! try_visit_item(dst, 0)) insert_item(dst, 0, items);
    uint8_t * data = dst->base.pack->data;
    for(int32_t BYTE =  dst->BYTE, src_max = pos + dst->items; pos < src_max; pos++, BYTE += 2)
        set_bytes((uint16_t)(src[pos]), 2, data,  BYTE);
}/**
*Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT
*	null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes
*	storage if the ID is stored as strin*/
INLINER void p23_param_id_SET_(char16_t*  src, Bounds_Inside * dst) {p23_param_id_SET(src, 0, strlen16(src), dst);}
INLINER void p24_eph_SET(uint16_t  src, Pack * dst)//GPS HDOP horizontal dilution of position (unitless). If unknown, set to: UINT16_MAX
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER void p24_epv_SET(uint16_t  src, Pack * dst)//GPS VDOP vertical dilution of position (unitless). If unknown, set to: UINT16_MAX
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  2);
}
INLINER void p24_vel_SET(uint16_t  src, Pack * dst)//GPS ground speed (m/s * 100). If unknown, set to: UINT16_MAX
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  4);
}
/**
*Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If
*	unknown, set to: UINT16_MA*/
INLINER void p24_cog_SET(uint16_t  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  6);
}
INLINER void p24_time_usec_SET(uint64_t  src, Pack * dst)//Timestamp (microseconds since UNIX epoch or microseconds since system boot)
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  8);
}
INLINER void p24_lat_SET(int32_t  src, Pack * dst)//Latitude (WGS84, EGM96 ellipsoid), in degrees * 1E7
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  16);
}
INLINER void p24_lon_SET(int32_t  src, Pack * dst)//Longitude (WGS84, EGM96 ellipsoid), in degrees * 1E7
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  20);
}
/**
*Altitude (AMSL, NOT WGS84), in meters * 1000 (positive for up). Note that virtually all GPS modules provide
*	the AMSL altitude in addition to the WGS84 altitude*/
INLINER void p24_alt_SET(int32_t  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  24);
}
INLINER void p24_satellites_visible_SET(uint8_t  src, Pack * dst)//Number of satellites visible. If unknown, set to 255
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  28);
}
INLINER void p24_fix_type_SET(e_GPS_FIX_TYPE  src, Pack * dst)//See the GPS_FIX_TYPE enum.
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 4, data, 232);
}
INLINER void p24_alt_ellipsoid_SET(int32_t  src, Bounds_Inside * dst)//Altitude (above WGS84, EGM96 ellipsoid), in meters * 1000 (positive for up).
{
    if(dst->base.field_bit != 236)insert_field(dst, 236, 0);
    uint8_t * data = dst->base.pack->data;
    set_bytes((uint32_t)(src), 4, data,  dst->BYTE);
}
INLINER void p24_h_acc_SET(uint32_t  src, Bounds_Inside * dst)//Position uncertainty in meters * 1000 (positive for up).
{
    if(dst->base.field_bit != 237)insert_field(dst, 237, 0);
    uint8_t * data = dst->base.pack->data;
    set_bytes((src), 4, data,  dst->BYTE);
}
INLINER void p24_v_acc_SET(uint32_t  src, Bounds_Inside * dst)//Altitude uncertainty in meters * 1000 (positive for up).
{
    if(dst->base.field_bit != 238)insert_field(dst, 238, 0);
    uint8_t * data = dst->base.pack->data;
    set_bytes((src), 4, data,  dst->BYTE);
}
INLINER void p24_vel_acc_SET(uint32_t  src, Bounds_Inside * dst)//Speed uncertainty in meters * 1000 (positive for up).
{
    if(dst->base.field_bit != 239)insert_field(dst, 239, 0);
    uint8_t * data = dst->base.pack->data;
    set_bytes((src), 4, data,  dst->BYTE);
}
INLINER void p24_hdg_acc_SET(uint32_t  src, Bounds_Inside * dst)//Heading / track uncertainty in degrees * 1e5.
{
    if(dst->base.field_bit != 240)insert_field(dst, 240, 0);
    uint8_t * data = dst->base.pack->data;
    set_bytes((src), 4, data,  dst->BYTE);
}
INLINER void p25_satellites_visible_SET(uint8_t  src, Pack * dst)//Number of satellites visible
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  0);
}
INLINER void p25_satellite_prn_SET(uint8_t*  src, int32_t pos, Pack * dst) //Global satellite ID
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  1, src_max = pos + 20; pos < src_max; pos++, BYTE += 1)
        set_bytes((src[pos]), 1, data,  BYTE);
}
INLINER void p25_satellite_used_SET(uint8_t*  src, int32_t pos, Pack * dst) //0: Satellite not used, 1: used for localization
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  21, src_max = pos + 20; pos < src_max; pos++, BYTE += 1)
        set_bytes((src[pos]), 1, data,  BYTE);
}
INLINER void p25_satellite_elevation_SET(uint8_t*  src, int32_t pos, Pack * dst) //Elevation (0: right on top of receiver, 90: on the horizon) of satellite
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  41, src_max = pos + 20; pos < src_max; pos++, BYTE += 1)
        set_bytes((src[pos]), 1, data,  BYTE);
}
INLINER void p25_satellite_azimuth_SET(uint8_t*  src, int32_t pos, Pack * dst) //Direction of satellite, 0: 0 deg, 255: 360 deg.
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  61, src_max = pos + 20; pos < src_max; pos++, BYTE += 1)
        set_bytes((src[pos]), 1, data,  BYTE);
}
INLINER void p25_satellite_snr_SET(uint8_t*  src, int32_t pos, Pack * dst) //Signal to noise ratio of satellite
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  81, src_max = pos + 20; pos < src_max; pos++, BYTE += 1)
        set_bytes((src[pos]), 1, data,  BYTE);
}
INLINER void p26_time_boot_ms_SET(uint32_t  src, Pack * dst)//Timestamp (milliseconds since system boot)
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  0);
}
INLINER void p26_xacc_SET(int16_t  src, Pack * dst)//X acceleration (mg)
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  4);
}
INLINER void p26_yacc_SET(int16_t  src, Pack * dst)//Y acceleration (mg)
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  6);
}
INLINER void p26_zacc_SET(int16_t  src, Pack * dst)//Z acceleration (mg)
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  8);
}
INLINER void p26_xgyro_SET(int16_t  src, Pack * dst)//Angular speed around X axis (millirad /sec)
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  10);
}
INLINER void p26_ygyro_SET(int16_t  src, Pack * dst)//Angular speed around Y axis (millirad /sec)
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  12);
}
INLINER void p26_zgyro_SET(int16_t  src, Pack * dst)//Angular speed around Z axis (millirad /sec)
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  14);
}
INLINER void p26_xmag_SET(int16_t  src, Pack * dst)//X Magnetic field (milli tesla)
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  16);
}
INLINER void p26_ymag_SET(int16_t  src, Pack * dst)//Y Magnetic field (milli tesla)
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  18);
}
INLINER void p26_zmag_SET(int16_t  src, Pack * dst)//Z Magnetic field (milli tesla)
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  20);
}
INLINER void p27_time_usec_SET(uint64_t  src, Pack * dst)//Timestamp (microseconds since UNIX epoch or microseconds since system boot)
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  0);
}
INLINER void p27_xacc_SET(int16_t  src, Pack * dst)//X acceleration (raw)
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  8);
}
INLINER void p27_yacc_SET(int16_t  src, Pack * dst)//Y acceleration (raw)
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  10);
}
INLINER void p27_zacc_SET(int16_t  src, Pack * dst)//Z acceleration (raw)
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  12);
}
INLINER void p27_xgyro_SET(int16_t  src, Pack * dst)//Angular speed around X axis (raw)
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  14);
}
INLINER void p27_ygyro_SET(int16_t  src, Pack * dst)//Angular speed around Y axis (raw)
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  16);
}
INLINER void p27_zgyro_SET(int16_t  src, Pack * dst)//Angular speed around Z axis (raw)
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  18);
}
INLINER void p27_xmag_SET(int16_t  src, Pack * dst)//X Magnetic field (raw)
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  20);
}
INLINER void p27_ymag_SET(int16_t  src, Pack * dst)//Y Magnetic field (raw)
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  22);
}
INLINER void p27_zmag_SET(int16_t  src, Pack * dst)//Z Magnetic field (raw)
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  24);
}
INLINER void p28_time_usec_SET(uint64_t  src, Pack * dst)//Timestamp (microseconds since UNIX epoch or microseconds since system boot)
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  0);
}
INLINER void p28_press_abs_SET(int16_t  src, Pack * dst)//Absolute pressure (raw)
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  8);
}
INLINER void p28_press_diff1_SET(int16_t  src, Pack * dst)//Differential pressure 1 (raw, 0 if nonexistant)
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  10);
}
INLINER void p28_press_diff2_SET(int16_t  src, Pack * dst)//Differential pressure 2 (raw, 0 if nonexistant)
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  12);
}
INLINER void p28_temperature_SET(int16_t  src, Pack * dst)//Raw Temperature measurement (raw)
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  14);
}
INLINER void p29_time_boot_ms_SET(uint32_t  src, Pack * dst)//Timestamp (milliseconds since system boot)
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  0);
}
INLINER void p29_press_abs_SET(float  src, Pack * dst)//Absolute pressure (hectopascal)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  4);
}
INLINER void p29_press_diff_SET(float  src, Pack * dst)//Differential pressure 1 (hectopascal)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  8);
}
INLINER void p29_temperature_SET(int16_t  src, Pack * dst)//Temperature measurement (0.01 degrees celsius)
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  12);
}
INLINER void p30_time_boot_ms_SET(uint32_t  src, Pack * dst)//Timestamp (milliseconds since system boot)
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  0);
}
INLINER void p30_roll_SET(float  src, Pack * dst)//Roll angle (rad, -pi..+pi)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  4);
}
INLINER void p30_pitch_SET(float  src, Pack * dst)//Pitch angle (rad, -pi..+pi)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  8);
}
INLINER void p30_yaw_SET(float  src, Pack * dst)//Yaw angle (rad, -pi..+pi)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  12);
}
INLINER void p30_rollspeed_SET(float  src, Pack * dst)//Roll angular speed (rad/s)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  16);
}
INLINER void p30_pitchspeed_SET(float  src, Pack * dst)//Pitch angular speed (rad/s)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  20);
}
INLINER void p30_yawspeed_SET(float  src, Pack * dst)//Yaw angular speed (rad/s)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  24);
}
INLINER void p31_time_boot_ms_SET(uint32_t  src, Pack * dst)//Timestamp (milliseconds since system boot)
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  0);
}
INLINER void p31_q1_SET(float  src, Pack * dst)//Quaternion component 1, w (1 in null-rotation)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  4);
}
INLINER void p31_q2_SET(float  src, Pack * dst)//Quaternion component 2, x (0 in null-rotation)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  8);
}
INLINER void p31_q3_SET(float  src, Pack * dst)//Quaternion component 3, y (0 in null-rotation)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  12);
}
INLINER void p31_q4_SET(float  src, Pack * dst)//Quaternion component 4, z (0 in null-rotation)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  16);
}
INLINER void p31_rollspeed_SET(float  src, Pack * dst)//Roll angular speed (rad/s)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  20);
}
INLINER void p31_pitchspeed_SET(float  src, Pack * dst)//Pitch angular speed (rad/s)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  24);
}
INLINER void p31_yawspeed_SET(float  src, Pack * dst)//Yaw angular speed (rad/s)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  28);
}
INLINER void p32_time_boot_ms_SET(uint32_t  src, Pack * dst)//Timestamp (milliseconds since system boot)
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  0);
}
INLINER void p32_x_SET(float  src, Pack * dst)//X Position
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  4);
}
INLINER void p32_y_SET(float  src, Pack * dst)//Y Position
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  8);
}
INLINER void p32_z_SET(float  src, Pack * dst)//Z Position
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  12);
}
INLINER void p32_vx_SET(float  src, Pack * dst)//X Speed
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  16);
}
INLINER void p32_vy_SET(float  src, Pack * dst)//Y Speed
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  20);
}
INLINER void p32_vz_SET(float  src, Pack * dst)//Z Speed
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  24);
}
INLINER void p33_hdg_SET(uint16_t  src, Pack * dst)//Vehicle heading (yaw angle) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER void p33_time_boot_ms_SET(uint32_t  src, Pack * dst)//Timestamp (milliseconds since system boot)
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  2);
}
INLINER void p33_lat_SET(int32_t  src, Pack * dst)//Latitude, expressed as degrees * 1E7
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  6);
}
INLINER void p33_lon_SET(int32_t  src, Pack * dst)//Longitude, expressed as degrees * 1E7
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  10);
}
/**
*Altitude in meters, expressed as * 1000 (millimeters), AMSL (not WGS84 - note that virtually all GPS modules
*	provide the AMSL as well*/
INLINER void p33_alt_SET(int32_t  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  14);
}
INLINER void p33_relative_alt_SET(int32_t  src, Pack * dst)//Altitude above ground in meters, expressed as * 1000 (millimeters)
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  18);
}
INLINER void p33_vx_SET(int16_t  src, Pack * dst)//Ground X Speed (Latitude, positive north), expressed as m/s * 100
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  22);
}
INLINER void p33_vy_SET(int16_t  src, Pack * dst)//Ground Y Speed (Longitude, positive east), expressed as m/s * 100
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  24);
}
INLINER void p33_vz_SET(int16_t  src, Pack * dst)//Ground Z Speed (Altitude, positive down), expressed as m/s * 100
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  26);
}
INLINER void p34_time_boot_ms_SET(uint32_t  src, Pack * dst)//Timestamp (milliseconds since system boot)
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  0);
}
/**
*Servo output port (set of 8 outputs = 1 port). Most MAVs will just use one, but this allows for more than
*	8 servos*/
INLINER void p34_port_SET(uint8_t  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  4);
}
INLINER void p34_chan1_scaled_SET(int16_t  src, Pack * dst)//RC channel 1 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  5);
}
INLINER void p34_chan2_scaled_SET(int16_t  src, Pack * dst)//RC channel 2 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  7);
}
INLINER void p34_chan3_scaled_SET(int16_t  src, Pack * dst)//RC channel 3 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  9);
}
INLINER void p34_chan4_scaled_SET(int16_t  src, Pack * dst)//RC channel 4 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  11);
}
INLINER void p34_chan5_scaled_SET(int16_t  src, Pack * dst)//RC channel 5 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  13);
}
INLINER void p34_chan6_scaled_SET(int16_t  src, Pack * dst)//RC channel 6 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  15);
}
INLINER void p34_chan7_scaled_SET(int16_t  src, Pack * dst)//RC channel 7 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  17);
}
INLINER void p34_chan8_scaled_SET(int16_t  src, Pack * dst)//RC channel 8 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  19);
}
INLINER void p34_rssi_SET(uint8_t  src, Pack * dst)//Receive signal strength indicator, 0: 0%, 100: 100%, 255: invalid/unknown.
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  21);
}
INLINER void p35_chan1_raw_SET(uint16_t  src, Pack * dst)//RC channel 1 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER void p35_chan2_raw_SET(uint16_t  src, Pack * dst)//RC channel 2 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  2);
}
INLINER void p35_chan3_raw_SET(uint16_t  src, Pack * dst)//RC channel 3 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  4);
}
INLINER void p35_chan4_raw_SET(uint16_t  src, Pack * dst)//RC channel 4 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  6);
}
INLINER void p35_chan5_raw_SET(uint16_t  src, Pack * dst)//RC channel 5 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  8);
}
INLINER void p35_chan6_raw_SET(uint16_t  src, Pack * dst)//RC channel 6 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  10);
}
INLINER void p35_chan7_raw_SET(uint16_t  src, Pack * dst)//RC channel 7 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  12);
}
INLINER void p35_chan8_raw_SET(uint16_t  src, Pack * dst)//RC channel 8 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  14);
}
INLINER void p35_time_boot_ms_SET(uint32_t  src, Pack * dst)//Timestamp (milliseconds since system boot)
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  16);
}
/**
*Servo output port (set of 8 outputs = 1 port). Most MAVs will just use one, but this allows for more than
*	8 servos*/
INLINER void p35_port_SET(uint8_t  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  20);
}
INLINER void p35_rssi_SET(uint8_t  src, Pack * dst)//Receive signal strength indicator, 0: 0%, 100: 100%, 255: invalid/unknown.
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  21);
}
INLINER void p36_servo1_raw_SET(uint16_t  src, Pack * dst)//Servo output 1 value, in microseconds
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER void p36_servo2_raw_SET(uint16_t  src, Pack * dst)//Servo output 2 value, in microseconds
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  2);
}
INLINER void p36_servo3_raw_SET(uint16_t  src, Pack * dst)//Servo output 3 value, in microseconds
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  4);
}
INLINER void p36_servo4_raw_SET(uint16_t  src, Pack * dst)//Servo output 4 value, in microseconds
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  6);
}
INLINER void p36_servo5_raw_SET(uint16_t  src, Pack * dst)//Servo output 5 value, in microseconds
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  8);
}
INLINER void p36_servo6_raw_SET(uint16_t  src, Pack * dst)//Servo output 6 value, in microseconds
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  10);
}
INLINER void p36_servo7_raw_SET(uint16_t  src, Pack * dst)//Servo output 7 value, in microseconds
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  12);
}
INLINER void p36_servo8_raw_SET(uint16_t  src, Pack * dst)//Servo output 8 value, in microseconds
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  14);
}
INLINER void p36_time_usec_SET(uint32_t  src, Pack * dst)//Timestamp (microseconds since system boot)
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  16);
}
/**
*Servo output port (set of 8 outputs = 1 port). Most MAVs will just use one, but this allows to encode
*	more than 8 servos*/
INLINER void p36_port_SET(uint8_t  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  20);
}
INLINER void p36_servo9_raw_SET(uint16_t  src, Bounds_Inside * dst)//Servo output 9 value, in microseconds
{
    if(dst->base.field_bit != 168)insert_field(dst, 168, 0);
    uint8_t * data = dst->base.pack->data;
    set_bytes((src), 2, data,  dst->BYTE);
}
INLINER void p36_servo10_raw_SET(uint16_t  src, Bounds_Inside * dst)//Servo output 10 value, in microseconds
{
    if(dst->base.field_bit != 169)insert_field(dst, 169, 0);
    uint8_t * data = dst->base.pack->data;
    set_bytes((src), 2, data,  dst->BYTE);
}
INLINER void p36_servo11_raw_SET(uint16_t  src, Bounds_Inside * dst)//Servo output 11 value, in microseconds
{
    if(dst->base.field_bit != 170)insert_field(dst, 170, 0);
    uint8_t * data = dst->base.pack->data;
    set_bytes((src), 2, data,  dst->BYTE);
}
INLINER void p36_servo12_raw_SET(uint16_t  src, Bounds_Inside * dst)//Servo output 12 value, in microseconds
{
    if(dst->base.field_bit != 171)insert_field(dst, 171, 0);
    uint8_t * data = dst->base.pack->data;
    set_bytes((src), 2, data,  dst->BYTE);
}
INLINER void p36_servo13_raw_SET(uint16_t  src, Bounds_Inside * dst)//Servo output 13 value, in microseconds
{
    if(dst->base.field_bit != 172)insert_field(dst, 172, 0);
    uint8_t * data = dst->base.pack->data;
    set_bytes((src), 2, data,  dst->BYTE);
}
INLINER void p36_servo14_raw_SET(uint16_t  src, Bounds_Inside * dst)//Servo output 14 value, in microseconds
{
    if(dst->base.field_bit != 173)insert_field(dst, 173, 0);
    uint8_t * data = dst->base.pack->data;
    set_bytes((src), 2, data,  dst->BYTE);
}
INLINER void p36_servo15_raw_SET(uint16_t  src, Bounds_Inside * dst)//Servo output 15 value, in microseconds
{
    if(dst->base.field_bit != 174)insert_field(dst, 174, 0);
    uint8_t * data = dst->base.pack->data;
    set_bytes((src), 2, data,  dst->BYTE);
}
INLINER void p36_servo16_raw_SET(uint16_t  src, Bounds_Inside * dst)//Servo output 16 value, in microseconds
{
    if(dst->base.field_bit != 175)insert_field(dst, 175, 0);
    uint8_t * data = dst->base.pack->data;
    set_bytes((src), 2, data,  dst->BYTE);
}
INLINER void p37_target_system_SET(uint8_t  src, Pack * dst)//System ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  0);
}
INLINER void p37_target_component_SET(uint8_t  src, Pack * dst)//Component ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  1);
}
INLINER void p37_start_index_SET(int16_t  src, Pack * dst)//Start index, 0 by default
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  2);
}
INLINER void p37_end_index_SET(int16_t  src, Pack * dst)//End index, -1 by default (-1: send list to end). Else a valid index of the list
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  4);
}
INLINER void p37_mission_type_SET(e_MAV_MISSION_TYPE  src, Pack * dst)//Mission type, see MAV_MISSION_TYPE
{
    uint8_t * data = dst->data;
    UMAX id = _id__j(src);
    set_bits(id, 3, data, 48);
}
INLINER void p38_target_system_SET(uint8_t  src, Pack * dst)//System ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  0);
}
INLINER void p38_target_component_SET(uint8_t  src, Pack * dst)//Component ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  1);
}
INLINER void p38_start_index_SET(int16_t  src, Pack * dst)//Start index, 0 by default and smaller / equal to the largest index of the current onboard list.
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  2);
}
INLINER void p38_end_index_SET(int16_t  src, Pack * dst)//End index, equal or greater than start index.
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  4);
}
INLINER void p38_mission_type_SET(e_MAV_MISSION_TYPE  src, Pack * dst)//Mission type, see MAV_MISSION_TYPE
{
    uint8_t * data = dst->data;
    UMAX id = _id__j(src);
    set_bits(id, 3, data, 48);
}
INLINER void p39_seq_SET(uint16_t  src, Pack * dst)//Sequence
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER void p39_target_system_SET(uint8_t  src, Pack * dst)//System ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  2);
}
INLINER void p39_target_component_SET(uint8_t  src, Pack * dst)//Component ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  3);
}
INLINER void p39_current_SET(uint8_t  src, Pack * dst)//false:0, true:1
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  4);
}
INLINER void p39_autocontinue_SET(uint8_t  src, Pack * dst)//autocontinue to next wp
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  5);
}
INLINER void p39_param1_SET(float  src, Pack * dst)//PARAM1, see MAV_CMD enum
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  6);
}
INLINER void p39_param2_SET(float  src, Pack * dst)//PARAM2, see MAV_CMD enum
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  10);
}
INLINER void p39_param3_SET(float  src, Pack * dst)//PARAM3, see MAV_CMD enum
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  14);
}
INLINER void p39_param4_SET(float  src, Pack * dst)//PARAM4, see MAV_CMD enum
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  18);
}
INLINER void p39_x_SET(float  src, Pack * dst)//PARAM5 / local: x position, global: latitude
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  22);
}
INLINER void p39_y_SET(float  src, Pack * dst)//PARAM6 / y position: global: longitude
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  26);
}
INLINER void p39_z_SET(float  src, Pack * dst)//PARAM7 / z position: global: altitude (relative or absolute, depending on frame.
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  30);
}
INLINER void p39_frame_SET(e_MAV_FRAME  src, Pack * dst)//The coordinate system of the waypoint. see MAV_FRAME in mavlink_types.h
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 4, data, 272);
}
INLINER void p39_command_SET(e_MAV_CMD  src, Pack * dst)//The scheduled action for the waypoint. see MAV_CMD in common.xml MAVLink specs
{
    uint8_t * data = dst->data;
    UMAX id = _id__c(src);
    set_bits(id, 8, data, 276);
}
INLINER void p39_mission_type_SET(e_MAV_MISSION_TYPE  src, Pack * dst)//Mission type, see MAV_MISSION_TYPE
{
    uint8_t * data = dst->data;
    UMAX id = _id__j(src);
    set_bits(id, 3, data, 284);
}
INLINER void p40_seq_SET(uint16_t  src, Pack * dst)//Sequence
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER void p40_target_system_SET(uint8_t  src, Pack * dst)//System ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  2);
}
INLINER void p40_target_component_SET(uint8_t  src, Pack * dst)//Component ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  3);
}
INLINER void p40_mission_type_SET(e_MAV_MISSION_TYPE  src, Pack * dst)//Mission type, see MAV_MISSION_TYPE
{
    uint8_t * data = dst->data;
    UMAX id = _id__j(src);
    set_bits(id, 3, data, 32);
}
INLINER void p41_seq_SET(uint16_t  src, Pack * dst)//Sequence
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER void p41_target_system_SET(uint8_t  src, Pack * dst)//System ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  2);
}
INLINER void p41_target_component_SET(uint8_t  src, Pack * dst)//Component ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  3);
}
INLINER void p42_seq_SET(uint16_t  src, Pack * dst)//Sequence
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER void p43_target_system_SET(uint8_t  src, Pack * dst)//System ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  0);
}
INLINER void p43_target_component_SET(uint8_t  src, Pack * dst)//Component ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  1);
}
INLINER void p43_mission_type_SET(e_MAV_MISSION_TYPE  src, Pack * dst)//Mission type, see MAV_MISSION_TYPE
{
    uint8_t * data = dst->data;
    UMAX id = _id__j(src);
    set_bits(id, 3, data, 16);
}
INLINER void p44_count_SET(uint16_t  src, Pack * dst)//Number of mission items in the sequence
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER void p44_target_system_SET(uint8_t  src, Pack * dst)//System ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  2);
}
INLINER void p44_target_component_SET(uint8_t  src, Pack * dst)//Component ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  3);
}
INLINER void p44_mission_type_SET(e_MAV_MISSION_TYPE  src, Pack * dst)//Mission type, see MAV_MISSION_TYPE
{
    uint8_t * data = dst->data;
    UMAX id = _id__j(src);
    set_bits(id, 3, data, 32);
}
INLINER void p45_target_system_SET(uint8_t  src, Pack * dst)//System ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  0);
}
INLINER void p45_target_component_SET(uint8_t  src, Pack * dst)//Component ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  1);
}
INLINER void p45_mission_type_SET(e_MAV_MISSION_TYPE  src, Pack * dst)//Mission type, see MAV_MISSION_TYPE
{
    uint8_t * data = dst->data;
    UMAX id = _id__j(src);
    set_bits(id, 3, data, 16);
}
INLINER void p46_seq_SET(uint16_t  src, Pack * dst)//Sequence
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER void p47_target_system_SET(uint8_t  src, Pack * dst)//System ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  0);
}
INLINER void p47_target_component_SET(uint8_t  src, Pack * dst)//Component ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  1);
}
INLINER void p47_type_SET(e_MAV_MISSION_RESULT  src, Pack * dst)//See MAV_MISSION_RESULT enum
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 4, data, 16);
}
INLINER void p47_mission_type_SET(e_MAV_MISSION_TYPE  src, Pack * dst)//Mission type, see MAV_MISSION_TYPE
{
    uint8_t * data = dst->data;
    UMAX id = _id__j(src);
    set_bits(id, 3, data, 20);
}
INLINER void p48_target_system_SET(uint8_t  src, Pack * dst)//System ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  0);
}
INLINER void p48_latitude_SET(int32_t  src, Pack * dst)//Latitude (WGS84), in degrees * 1E7
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  1);
}
INLINER void p48_longitude_SET(int32_t  src, Pack * dst)//Longitude (WGS84, in degrees * 1E7
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  5);
}
INLINER void p48_altitude_SET(int32_t  src, Pack * dst)//Altitude (AMSL), in meters * 1000 (positive for up)
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  9);
}
INLINER void p48_time_usec_SET(uint64_t  src, Bounds_Inside * dst)//Timestamp (microseconds since UNIX epoch or microseconds since system boot)
{
    if(dst->base.field_bit != 104)insert_field(dst, 104, 0);
    uint8_t * data = dst->base.pack->data;
    set_bytes((src), 8, data,  dst->BYTE);
}
INLINER void p49_latitude_SET(int32_t  src, Pack * dst)//Latitude (WGS84), in degrees * 1E7
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  0);
}
INLINER void p49_longitude_SET(int32_t  src, Pack * dst)//Longitude (WGS84), in degrees * 1E7
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  4);
}
INLINER void p49_altitude_SET(int32_t  src, Pack * dst)//Altitude (AMSL), in meters * 1000 (positive for up)
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  8);
}
INLINER void p49_time_usec_SET(uint64_t  src, Bounds_Inside * dst)//Timestamp (microseconds since UNIX epoch or microseconds since system boot)
{
    if(dst->base.field_bit != 96)insert_field(dst, 96, 0);
    uint8_t * data = dst->base.pack->data;
    set_bytes((src), 8, data,  dst->BYTE);
}
INLINER void p50_target_system_SET(uint8_t  src, Pack * dst)//System ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  0);
}
INLINER void p50_target_component_SET(uint8_t  src, Pack * dst)//Component ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  1);
}
/**
*Parameter index. Send -1 to use the param ID field as identifier (else the param id will be ignored),
*	send -2 to disable any existing map for this rc_channel_index*/
INLINER void p50_param_index_SET(int16_t  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  2);
}
/**
*Index of parameter RC channel. Not equal to the RC channel id. Typically correpsonds to a potentiometer-knob
*	on the RC*/
INLINER void p50_parameter_rc_channel_index_SET(uint8_t  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  4);
}
INLINER void p50_param_value0_SET(float  src, Pack * dst)//Initial parameter value
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  5);
}
INLINER void p50_scale_SET(float  src, Pack * dst)//Scale, maps the RC range [-1, 1] to a parameter value
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  9);
}
/**
*Minimum param value. The protocol does not define if this overwrites an onboard minimum value. (Depends
*	on implementation*/
INLINER void p50_param_value_min_SET(float  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  13);
}
/**
*Maximum param value. The protocol does not define if this overwrites an onboard maximum value. (Depends
*	on implementation*/
INLINER void p50_param_value_max_SET(float  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  17);
}
/**
*Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT
*	null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes
*	storage if the ID is stored as strin*/
INLINER void p50_param_id_SET(char16_t *  src, int32_t pos, int32_t items, Bounds_Inside * dst)
{
    if(dst->base.field_bit != 168 && insert_field(dst, 168, items) ||
            ! try_visit_item(dst, 0)) insert_item(dst, 0, items);
    uint8_t * data = dst->base.pack->data;
    for(int32_t BYTE =  dst->BYTE, src_max = pos + dst->items; pos < src_max; pos++, BYTE += 2)
        set_bytes((uint16_t)(src[pos]), 2, data,  BYTE);
}/**
*Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT
*	null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes
*	storage if the ID is stored as strin*/
INLINER void p50_param_id_SET_(char16_t*  src, Bounds_Inside * dst) {p50_param_id_SET(src, 0, strlen16(src), dst);}
INLINER void p51_seq_SET(uint16_t  src, Pack * dst)//Sequence
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER void p51_target_system_SET(uint8_t  src, Pack * dst)//System ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  2);
}
INLINER void p51_target_component_SET(uint8_t  src, Pack * dst)//Component ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  3);
}
INLINER void p51_mission_type_SET(e_MAV_MISSION_TYPE  src, Pack * dst)//Mission type, see MAV_MISSION_TYPE
{
    uint8_t * data = dst->data;
    UMAX id = _id__j(src);
    set_bits(id, 3, data, 32);
}
INLINER void p54_target_system_SET(uint8_t  src, Pack * dst)//System ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  0);
}
INLINER void p54_target_component_SET(uint8_t  src, Pack * dst)//Component ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  1);
}
INLINER void p54_p1x_SET(float  src, Pack * dst)//x position 1 / Latitude 1
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  2);
}
INLINER void p54_p1y_SET(float  src, Pack * dst)//y position 1 / Longitude 1
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  6);
}
INLINER void p54_p1z_SET(float  src, Pack * dst)//z position 1 / Altitude 1
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  10);
}
INLINER void p54_p2x_SET(float  src, Pack * dst)//x position 2 / Latitude 2
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  14);
}
INLINER void p54_p2y_SET(float  src, Pack * dst)//y position 2 / Longitude 2
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  18);
}
INLINER void p54_p2z_SET(float  src, Pack * dst)//z position 2 / Altitude 2
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  22);
}
/**
*Coordinate frame, as defined by MAV_FRAME enum in mavlink_types.h. Can be either global, GPS, right-handed
*	with Z axis up or local, right handed, Z axis down*/
INLINER void p54_frame_SET(e_MAV_FRAME  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 4, data, 208);
}
INLINER void p55_p1x_SET(float  src, Pack * dst)//x position 1 / Latitude 1
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  0);
}
INLINER void p55_p1y_SET(float  src, Pack * dst)//y position 1 / Longitude 1
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  4);
}
INLINER void p55_p1z_SET(float  src, Pack * dst)//z position 1 / Altitude 1
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  8);
}
INLINER void p55_p2x_SET(float  src, Pack * dst)//x position 2 / Latitude 2
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  12);
}
INLINER void p55_p2y_SET(float  src, Pack * dst)//y position 2 / Longitude 2
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  16);
}
INLINER void p55_p2z_SET(float  src, Pack * dst)//z position 2 / Altitude 2
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  20);
}
/**
*Coordinate frame, as defined by MAV_FRAME enum in mavlink_types.h. Can be either global, GPS, right-handed
*	with Z axis up or local, right handed, Z axis down*/
INLINER void p55_frame_SET(e_MAV_FRAME  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 4, data, 192);
}
INLINER void p61_time_usec_SET(uint64_t  src, Pack * dst)//Timestamp (microseconds since system boot or since UNIX epoch)
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  0);
}
INLINER void p61_q_SET(float*  src, int32_t pos, Pack * dst) //Quaternion components, w, x, y, z (1 0 0 0 is the null-rotation)
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  8, src_max = pos + 4; pos < src_max; pos++, BYTE += 4)
        set_bytes(floatToIntBits(src[pos]), 4, data,  BYTE);
}
INLINER void p61_rollspeed_SET(float  src, Pack * dst)//Roll angular speed (rad/s)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  24);
}
INLINER void p61_pitchspeed_SET(float  src, Pack * dst)//Pitch angular speed (rad/s)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  28);
}
INLINER void p61_yawspeed_SET(float  src, Pack * dst)//Yaw angular speed (rad/s)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  32);
}
INLINER void p61_covariance_SET(float*  src, int32_t pos, Pack * dst) //Attitude covariance
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  36, src_max = pos + 9; pos < src_max; pos++, BYTE += 4)
        set_bytes(floatToIntBits(src[pos]), 4, data,  BYTE);
}
INLINER void p62_wp_dist_SET(uint16_t  src, Pack * dst)//Distance to active waypoint in meters
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER void p62_nav_roll_SET(float  src, Pack * dst)//Current desired roll in degrees
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  2);
}
INLINER void p62_nav_pitch_SET(float  src, Pack * dst)//Current desired pitch in degrees
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  6);
}
INLINER void p62_nav_bearing_SET(int16_t  src, Pack * dst)//Current desired heading in degrees
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  10);
}
INLINER void p62_target_bearing_SET(int16_t  src, Pack * dst)//Bearing to current waypoint/target in degrees
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  12);
}
INLINER void p62_alt_error_SET(float  src, Pack * dst)//Current altitude error in meters
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  14);
}
INLINER void p62_aspd_error_SET(float  src, Pack * dst)//Current airspeed error in meters/second
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  18);
}
INLINER void p62_xtrack_error_SET(float  src, Pack * dst)//Current crosstrack error on x-y plane in meters
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  22);
}
INLINER void p63_time_usec_SET(uint64_t  src, Pack * dst)//Timestamp (microseconds since system boot or since UNIX epoch)
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  0);
}
INLINER void p63_lat_SET(int32_t  src, Pack * dst)//Latitude, expressed as degrees * 1E7
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  8);
}
INLINER void p63_lon_SET(int32_t  src, Pack * dst)//Longitude, expressed as degrees * 1E7
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  12);
}
INLINER void p63_alt_SET(int32_t  src, Pack * dst)//Altitude in meters, expressed as * 1000 (millimeters), above MSL
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  16);
}
INLINER void p63_relative_alt_SET(int32_t  src, Pack * dst)//Altitude above ground in meters, expressed as * 1000 (millimeters)
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  20);
}
INLINER void p63_vx_SET(float  src, Pack * dst)//Ground X Speed (Latitude), expressed as m/s
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  24);
}
INLINER void p63_vy_SET(float  src, Pack * dst)//Ground Y Speed (Longitude), expressed as m/s
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  28);
}
INLINER void p63_vz_SET(float  src, Pack * dst)//Ground Z Speed (Altitude), expressed as m/s
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  32);
}
INLINER void p63_covariance_SET(float*  src, int32_t pos, Pack * dst) //Covariance matrix (first six entries are the first ROW, next six entries are the second row, etc.)
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  36, src_max = pos + 36; pos < src_max; pos++, BYTE += 4)
        set_bytes(floatToIntBits(src[pos]), 4, data,  BYTE);
}
INLINER void p63_estimator_type_SET(e_MAV_ESTIMATOR_TYPE  src, Pack * dst)//Class id of the estimator this estimate originated from.
{
    uint8_t * data = dst->data;
    set_bits(- 1 +   src, 3, data, 1440);
}
INLINER void p64_time_usec_SET(uint64_t  src, Pack * dst)//Timestamp (microseconds since system boot or since UNIX epoch)
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  0);
}
INLINER void p64_x_SET(float  src, Pack * dst)//X Position
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  8);
}
INLINER void p64_y_SET(float  src, Pack * dst)//Y Position
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  12);
}
INLINER void p64_z_SET(float  src, Pack * dst)//Z Position
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  16);
}
INLINER void p64_vx_SET(float  src, Pack * dst)//X Speed (m/s)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  20);
}
INLINER void p64_vy_SET(float  src, Pack * dst)//Y Speed (m/s)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  24);
}
INLINER void p64_vz_SET(float  src, Pack * dst)//Z Speed (m/s)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  28);
}
INLINER void p64_ax_SET(float  src, Pack * dst)//X Acceleration (m/s^2)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  32);
}
INLINER void p64_ay_SET(float  src, Pack * dst)//Y Acceleration (m/s^2)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  36);
}
INLINER void p64_az_SET(float  src, Pack * dst)//Z Acceleration (m/s^2)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  40);
}
/**
*Covariance matrix upper right triangular (first nine entries are the first ROW, next eight entries are
*	the second row, etc.*/
INLINER void p64_covariance_SET(float*  src, int32_t pos, Pack * dst)
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  44, src_max = pos + 45; pos < src_max; pos++, BYTE += 4)
        set_bytes(floatToIntBits(src[pos]), 4, data,  BYTE);
}
INLINER void p64_estimator_type_SET(e_MAV_ESTIMATOR_TYPE  src, Pack * dst)//Class id of the estimator this estimate originated from.
{
    uint8_t * data = dst->data;
    set_bits(- 1 +   src, 3, data, 1792);
}
INLINER void p65_chan1_raw_SET(uint16_t  src, Pack * dst)//RC channel 1 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER void p65_chan2_raw_SET(uint16_t  src, Pack * dst)//RC channel 2 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  2);
}
INLINER void p65_chan3_raw_SET(uint16_t  src, Pack * dst)//RC channel 3 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  4);
}
INLINER void p65_chan4_raw_SET(uint16_t  src, Pack * dst)//RC channel 4 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  6);
}
INLINER void p65_chan5_raw_SET(uint16_t  src, Pack * dst)//RC channel 5 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  8);
}
INLINER void p65_chan6_raw_SET(uint16_t  src, Pack * dst)//RC channel 6 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  10);
}
INLINER void p65_chan7_raw_SET(uint16_t  src, Pack * dst)//RC channel 7 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  12);
}
INLINER void p65_chan8_raw_SET(uint16_t  src, Pack * dst)//RC channel 8 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  14);
}
INLINER void p65_chan9_raw_SET(uint16_t  src, Pack * dst)//RC channel 9 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  16);
}
INLINER void p65_chan10_raw_SET(uint16_t  src, Pack * dst)//RC channel 10 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  18);
}
INLINER void p65_chan11_raw_SET(uint16_t  src, Pack * dst)//RC channel 11 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  20);
}
INLINER void p65_chan12_raw_SET(uint16_t  src, Pack * dst)//RC channel 12 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  22);
}
INLINER void p65_chan13_raw_SET(uint16_t  src, Pack * dst)//RC channel 13 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  24);
}
INLINER void p65_chan14_raw_SET(uint16_t  src, Pack * dst)//RC channel 14 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  26);
}
INLINER void p65_chan15_raw_SET(uint16_t  src, Pack * dst)//RC channel 15 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  28);
}
INLINER void p65_chan16_raw_SET(uint16_t  src, Pack * dst)//RC channel 16 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  30);
}
INLINER void p65_chan17_raw_SET(uint16_t  src, Pack * dst)//RC channel 17 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  32);
}
INLINER void p65_chan18_raw_SET(uint16_t  src, Pack * dst)//RC channel 18 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  34);
}
INLINER void p65_time_boot_ms_SET(uint32_t  src, Pack * dst)//Timestamp (milliseconds since system boot)
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  36);
}
/**
*Total number of RC channels being received. This can be larger than 18, indicating that more channels
*	are available but not given in this message. This value should be 0 when no RC channels are available*/
INLINER void p65_chancount_SET(uint8_t  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  40);
}
INLINER void p65_rssi_SET(uint8_t  src, Pack * dst)//Receive signal strength indicator, 0: 0%, 100: 100%, 255: invalid/unknown.
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  41);
}
INLINER void p66_req_message_rate_SET(uint16_t  src, Pack * dst)//The requested message rate
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER void p66_target_system_SET(uint8_t  src, Pack * dst)//The target requested to send the message stream.
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  2);
}
INLINER void p66_target_component_SET(uint8_t  src, Pack * dst)//The target requested to send the message stream.
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  3);
}
INLINER void p66_req_stream_id_SET(uint8_t  src, Pack * dst)//The ID of the requested data stream
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  4);
}
INLINER void p66_start_stop_SET(uint8_t  src, Pack * dst)//1 to start sending, 0 to stop sending.
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  5);
}
INLINER void p67_message_rate_SET(uint16_t  src, Pack * dst)//The message rate
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER void p67_stream_id_SET(uint8_t  src, Pack * dst)//The ID of the requested data stream
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  2);
}
INLINER void p67_on_off_SET(uint8_t  src, Pack * dst)//1 stream is enabled, 0 stream is stopped.
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  3);
}
/**
*A bitfield corresponding to the joystick buttons' current state, 1 for pressed, 0 for released. The lowest
*	bit corresponds to Button 1*/
INLINER void p69_buttons_SET(uint16_t  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER void p69_target_SET(uint8_t  src, Pack * dst)//The system to be controlled.
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  2);
}
/**
*X-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid.
*	Generally corresponds to forward(1000)-backward(-1000) movement on a joystick and the pitch of a vehicle*/
INLINER void p69_x_SET(int16_t  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  3);
}
/**
*Y-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid.
*	Generally corresponds to left(-1000)-right(1000) movement on a joystick and the roll of a vehicle*/
INLINER void p69_y_SET(int16_t  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  5);
}
/**
*Z-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid.
*	Generally corresponds to a separate slider movement with maximum being 1000 and minimum being -1000 on
*	a joystick and the thrust of a vehicle. Positive values are positive thrust, negative values are negative
*	thrust*/
INLINER void p69_z_SET(int16_t  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  7);
}
/**
*R-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid.
*	Generally corresponds to a twisting of the joystick, with counter-clockwise being 1000 and clockwise
*	being -1000, and the yaw of a vehicle*/
INLINER void p69_r_SET(int16_t  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  9);
}
INLINER void p70_chan1_raw_SET(uint16_t  src, Pack * dst)//RC channel 1 value, in microseconds. A value of UINT16_MAX means to ignore this field.
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER void p70_chan2_raw_SET(uint16_t  src, Pack * dst)//RC channel 2 value, in microseconds. A value of UINT16_MAX means to ignore this field.
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  2);
}
INLINER void p70_chan3_raw_SET(uint16_t  src, Pack * dst)//RC channel 3 value, in microseconds. A value of UINT16_MAX means to ignore this field.
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  4);
}
INLINER void p70_chan4_raw_SET(uint16_t  src, Pack * dst)//RC channel 4 value, in microseconds. A value of UINT16_MAX means to ignore this field.
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  6);
}
INLINER void p70_chan5_raw_SET(uint16_t  src, Pack * dst)//RC channel 5 value, in microseconds. A value of UINT16_MAX means to ignore this field.
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  8);
}
INLINER void p70_chan6_raw_SET(uint16_t  src, Pack * dst)//RC channel 6 value, in microseconds. A value of UINT16_MAX means to ignore this field.
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  10);
}
INLINER void p70_chan7_raw_SET(uint16_t  src, Pack * dst)//RC channel 7 value, in microseconds. A value of UINT16_MAX means to ignore this field.
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  12);
}
INLINER void p70_chan8_raw_SET(uint16_t  src, Pack * dst)//RC channel 8 value, in microseconds. A value of UINT16_MAX means to ignore this field.
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  14);
}
INLINER void p70_target_system_SET(uint8_t  src, Pack * dst)//System ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  16);
}
INLINER void p70_target_component_SET(uint8_t  src, Pack * dst)//Component ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  17);
}
/**
*Waypoint ID (sequence number). Starts at zero. Increases monotonically for each waypoint, no gaps in the
*	sequence (0,1,2,3,4)*/
INLINER void p73_seq_SET(uint16_t  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER void p73_target_system_SET(uint8_t  src, Pack * dst)//System ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  2);
}
INLINER void p73_target_component_SET(uint8_t  src, Pack * dst)//Component ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  3);
}
INLINER void p73_current_SET(uint8_t  src, Pack * dst)//false:0, true:1
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  4);
}
INLINER void p73_autocontinue_SET(uint8_t  src, Pack * dst)//autocontinue to next wp
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  5);
}
INLINER void p73_param1_SET(float  src, Pack * dst)//PARAM1, see MAV_CMD enum
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  6);
}
INLINER void p73_param2_SET(float  src, Pack * dst)//PARAM2, see MAV_CMD enum
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  10);
}
INLINER void p73_param3_SET(float  src, Pack * dst)//PARAM3, see MAV_CMD enum
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  14);
}
INLINER void p73_param4_SET(float  src, Pack * dst)//PARAM4, see MAV_CMD enum
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  18);
}
INLINER void p73_x_SET(int32_t  src, Pack * dst)//PARAM5 / local: x position in meters * 1e4, global: latitude in degrees * 10^7
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  22);
}
INLINER void p73_y_SET(int32_t  src, Pack * dst)//PARAM6 / y position: local: x position in meters * 1e4, global: longitude in degrees *10^7
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  26);
}
INLINER void p73_z_SET(float  src, Pack * dst)//PARAM7 / z position: global: altitude in meters (relative or absolute, depending on frame.
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  30);
}
INLINER void p73_frame_SET(e_MAV_FRAME  src, Pack * dst)//The coordinate system of the waypoint. see MAV_FRAME in mavlink_types.h
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 4, data, 272);
}
INLINER void p73_command_SET(e_MAV_CMD  src, Pack * dst)//The scheduled action for the waypoint. see MAV_CMD in common.xml MAVLink specs
{
    uint8_t * data = dst->data;
    UMAX id = _id__c(src);
    set_bits(id, 8, data, 276);
}
INLINER void p73_mission_type_SET(e_MAV_MISSION_TYPE  src, Pack * dst)//Mission type, see MAV_MISSION_TYPE
{
    uint8_t * data = dst->data;
    UMAX id = _id__j(src);
    set_bits(id, 3, data, 284);
}
INLINER void p74_throttle_SET(uint16_t  src, Pack * dst)//Current throttle setting in integer percent, 0 to 100
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER void p74_airspeed_SET(float  src, Pack * dst)//Current airspeed in m/s
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  2);
}
INLINER void p74_groundspeed_SET(float  src, Pack * dst)//Current ground speed in m/s
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  6);
}
INLINER void p74_heading_SET(int16_t  src, Pack * dst)//Current heading in degrees, in compass units (0..360, 0=north)
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  10);
}
INLINER void p74_alt_SET(float  src, Pack * dst)//Current altitude (MSL), in meters
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  12);
}
INLINER void p74_climb_SET(float  src, Pack * dst)//Current climb rate in meters/second
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  16);
}
INLINER void p75_target_system_SET(uint8_t  src, Pack * dst)//System ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  0);
}
INLINER void p75_target_component_SET(uint8_t  src, Pack * dst)//Component ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  1);
}
INLINER void p75_current_SET(uint8_t  src, Pack * dst)//false:0, true:1
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  2);
}
INLINER void p75_autocontinue_SET(uint8_t  src, Pack * dst)//autocontinue to next wp
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  3);
}
INLINER void p75_param1_SET(float  src, Pack * dst)//PARAM1, see MAV_CMD enum
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  4);
}
INLINER void p75_param2_SET(float  src, Pack * dst)//PARAM2, see MAV_CMD enum
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  8);
}
INLINER void p75_param3_SET(float  src, Pack * dst)//PARAM3, see MAV_CMD enum
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  12);
}
INLINER void p75_param4_SET(float  src, Pack * dst)//PARAM4, see MAV_CMD enum
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  16);
}
INLINER void p75_x_SET(int32_t  src, Pack * dst)//PARAM5 / local: x position in meters * 1e4, global: latitude in degrees * 10^7
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  20);
}
INLINER void p75_y_SET(int32_t  src, Pack * dst)//PARAM6 / local: y position in meters * 1e4, global: longitude in degrees * 10^7
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  24);
}
INLINER void p75_z_SET(float  src, Pack * dst)//PARAM7 / z position: global: altitude in meters (relative or absolute, depending on frame.
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  28);
}
INLINER void p75_frame_SET(e_MAV_FRAME  src, Pack * dst)//The coordinate system of the COMMAND. see MAV_FRAME in mavlink_types.h
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 4, data, 256);
}
INLINER void p75_command_SET(e_MAV_CMD  src, Pack * dst)//The scheduled action for the mission item. see MAV_CMD in common.xml MAVLink specs
{
    uint8_t * data = dst->data;
    UMAX id = _id__c(src);
    set_bits(id, 8, data, 260);
}
INLINER void p76_target_system_SET(uint8_t  src, Pack * dst)//System which should execute the command
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  0);
}
INLINER void p76_target_component_SET(uint8_t  src, Pack * dst)//Component which should execute the command, 0 for all components
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  1);
}
INLINER void p76_confirmation_SET(uint8_t  src, Pack * dst)//0: First transmission of this command. 1-255: Confirmation transmissions (e.g. for kill command)
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  2);
}
INLINER void p76_param1_SET(float  src, Pack * dst)//Parameter 1, as defined by MAV_CMD enum.
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  3);
}
INLINER void p76_param2_SET(float  src, Pack * dst)//Parameter 2, as defined by MAV_CMD enum.
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  7);
}
INLINER void p76_param3_SET(float  src, Pack * dst)//Parameter 3, as defined by MAV_CMD enum.
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  11);
}
INLINER void p76_param4_SET(float  src, Pack * dst)//Parameter 4, as defined by MAV_CMD enum.
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  15);
}
INLINER void p76_param5_SET(float  src, Pack * dst)//Parameter 5, as defined by MAV_CMD enum.
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  19);
}
INLINER void p76_param6_SET(float  src, Pack * dst)//Parameter 6, as defined by MAV_CMD enum.
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  23);
}
INLINER void p76_param7_SET(float  src, Pack * dst)//Parameter 7, as defined by MAV_CMD enum.
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  27);
}
INLINER void p76_command_SET(e_MAV_CMD  src, Pack * dst)//Command ID, as defined by MAV_CMD enum.
{
    uint8_t * data = dst->data;
    UMAX id = _id__c(src);
    set_bits(id, 8, data, 248);
}
INLINER void p77_command_SET(e_MAV_CMD  src, Pack * dst)//Command ID, as defined by MAV_CMD enum.
{
    uint8_t * data = dst->data;
    UMAX id = _id__c(src);
    set_bits(id, 8, data, 0);
}
INLINER void p77_result_SET(e_MAV_RESULT  src, Pack * dst)//See MAV_RESULT enum
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 3, data, 8);
}
/**
*WIP: Also used as result_param1, it can be set with a enum containing the errors reasons of why the command
*	was denied or the progress percentage or 255 if unknown the progress when result is MAV_RESULT_IN_PROGRESS*/
INLINER void p77_progress_SET(uint8_t  src, Bounds_Inside * dst)
{
    if(dst->base.field_bit != 11)insert_field(dst, 11, 0);
    uint8_t * data = dst->base.pack->data;
    set_bytes((src), 1, data,  dst->BYTE);
}/**
*WIP: Additional parameter of the result, example: which parameter of MAV_CMD_NAV_WAYPOINT caused it to
*	be denied*/
INLINER void p77_result_param2_SET(int32_t  src, Bounds_Inside * dst)
{
    if(dst->base.field_bit != 12)insert_field(dst, 12, 0);
    uint8_t * data = dst->base.pack->data;
    set_bytes((uint32_t)(src), 4, data,  dst->BYTE);
}
INLINER void p77_target_system_SET(uint8_t  src, Bounds_Inside * dst)//WIP: System which requested the command to be executed
{
    if(dst->base.field_bit != 13)insert_field(dst, 13, 0);
    uint8_t * data = dst->base.pack->data;
    set_bytes((src), 1, data,  dst->BYTE);
}
INLINER void p77_target_component_SET(uint8_t  src, Bounds_Inside * dst)//WIP: Component which requested the command to be executed
{
    if(dst->base.field_bit != 14)insert_field(dst, 14, 0);
    uint8_t * data = dst->base.pack->data;
    set_bytes((src), 1, data,  dst->BYTE);
}
INLINER void p81_time_boot_ms_SET(uint32_t  src, Pack * dst)//Timestamp in milliseconds since system boot
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  0);
}
INLINER void p81_roll_SET(float  src, Pack * dst)//Desired roll rate in radians per second
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  4);
}
INLINER void p81_pitch_SET(float  src, Pack * dst)//Desired pitch rate in radians per second
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  8);
}
INLINER void p81_yaw_SET(float  src, Pack * dst)//Desired yaw rate in radians per second
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  12);
}
INLINER void p81_thrust_SET(float  src, Pack * dst)//Collective thrust, normalized to 0 .. 1
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  16);
}
INLINER void p81_mode_switch_SET(uint8_t  src, Pack * dst)//Flight mode switch position, 0.. 255
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  20);
}
INLINER void p81_manual_override_switch_SET(uint8_t  src, Pack * dst)//Override mode switch position, 0.. 255
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  21);
}
INLINER void p82_time_boot_ms_SET(uint32_t  src, Pack * dst)//Timestamp in milliseconds since system boot
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  0);
}
INLINER void p82_target_system_SET(uint8_t  src, Pack * dst)//System ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  4);
}
INLINER void p82_target_component_SET(uint8_t  src, Pack * dst)//Component ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  5);
}
/**
*Mappings: If any of these bits are set, the corresponding input should be ignored: bit 1: body roll rate,
*	bit 2: body pitch rate, bit 3: body yaw rate. bit 4-bit 6: reserved, bit 7: throttle, bit 8: attitud*/
INLINER void p82_type_mask_SET(uint8_t  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  6);
}
INLINER void p82_q_SET(float*  src, int32_t pos, Pack * dst) //Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  7, src_max = pos + 4; pos < src_max; pos++, BYTE += 4)
        set_bytes(floatToIntBits(src[pos]), 4, data,  BYTE);
}
INLINER void p82_body_roll_rate_SET(float  src, Pack * dst)//Body roll rate in radians per second
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  23);
}
INLINER void p82_body_pitch_rate_SET(float  src, Pack * dst)//Body roll rate in radians per second
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  27);
}
INLINER void p82_body_yaw_rate_SET(float  src, Pack * dst)//Body roll rate in radians per second
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  31);
}
INLINER void p82_thrust_SET(float  src, Pack * dst)//Collective thrust, normalized to 0 .. 1 (-1 .. 1 for vehicles capable of reverse trust)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  35);
}
INLINER void p83_time_boot_ms_SET(uint32_t  src, Pack * dst)//Timestamp in milliseconds since system boot
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  0);
}
/**
*Mappings: If any of these bits are set, the corresponding input should be ignored: bit 1: body roll rate,
*	bit 2: body pitch rate, bit 3: body yaw rate. bit 4-bit 7: reserved, bit 8: attitud*/
INLINER void p83_type_mask_SET(uint8_t  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  4);
}
INLINER void p83_q_SET(float*  src, int32_t pos, Pack * dst) //Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  5, src_max = pos + 4; pos < src_max; pos++, BYTE += 4)
        set_bytes(floatToIntBits(src[pos]), 4, data,  BYTE);
}
INLINER void p83_body_roll_rate_SET(float  src, Pack * dst)//Body roll rate in radians per second
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  21);
}
INLINER void p83_body_pitch_rate_SET(float  src, Pack * dst)//Body pitch rate in radians per second
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  25);
}
INLINER void p83_body_yaw_rate_SET(float  src, Pack * dst)//Body yaw rate in radians per second
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  29);
}
INLINER void p83_thrust_SET(float  src, Pack * dst)//Collective thrust, normalized to 0 .. 1 (-1 .. 1 for vehicles capable of reverse trust)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  33);
}
/**
*Bitmask to indicate which dimensions should be ignored by the vehicle: a value of 0b0000000000000000 or
*	0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. If bit 10 is set
*	the floats afx afy afz should be interpreted as force instead of acceleration. Mapping: bit 1: x, bit
*	2: y, bit 3: z, bit 4: vx, bit 5: vy, bit 6: vz, bit 7: ax, bit 8: ay, bit 9: az, bit 10: is force setpoint,
*	bit 11: yaw, bit 12: yaw rat*/
INLINER void p84_type_mask_SET(uint16_t  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER void p84_time_boot_ms_SET(uint32_t  src, Pack * dst)//Timestamp in milliseconds since system boot
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  2);
}
INLINER void p84_target_system_SET(uint8_t  src, Pack * dst)//System ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  6);
}
INLINER void p84_target_component_SET(uint8_t  src, Pack * dst)//Component ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  7);
}
INLINER void p84_x_SET(float  src, Pack * dst)//X Position in NED frame in meters
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  8);
}
INLINER void p84_y_SET(float  src, Pack * dst)//Y Position in NED frame in meters
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  12);
}
INLINER void p84_z_SET(float  src, Pack * dst)//Z Position in NED frame in meters (note, altitude is negative in NED)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  16);
}
INLINER void p84_vx_SET(float  src, Pack * dst)//X velocity in NED frame in meter / s
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  20);
}
INLINER void p84_vy_SET(float  src, Pack * dst)//Y velocity in NED frame in meter / s
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  24);
}
INLINER void p84_vz_SET(float  src, Pack * dst)//Z velocity in NED frame in meter / s
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  28);
}
INLINER void p84_afx_SET(float  src, Pack * dst)//X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  32);
}
INLINER void p84_afy_SET(float  src, Pack * dst)//Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  36);
}
INLINER void p84_afz_SET(float  src, Pack * dst)//Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  40);
}
INLINER void p84_yaw_SET(float  src, Pack * dst)//yaw setpoint in rad
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  44);
}
INLINER void p84_yaw_rate_SET(float  src, Pack * dst)//yaw rate setpoint in rad/s
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  48);
}
/**
*Valid options are: MAV_FRAME_LOCAL_NED = 1, MAV_FRAME_LOCAL_OFFSET_NED = 7, MAV_FRAME_BODY_NED = 8, MAV_FRAME_BODY_OFFSET_NED
*	=*/
INLINER void p84_coordinate_frame_SET(e_MAV_FRAME  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 4, data, 416);
}
/**
*Bitmask to indicate which dimensions should be ignored by the vehicle: a value of 0b0000000000000000 or
*	0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. If bit 10 is set
*	the floats afx afy afz should be interpreted as force instead of acceleration. Mapping: bit 1: x, bit
*	2: y, bit 3: z, bit 4: vx, bit 5: vy, bit 6: vz, bit 7: ax, bit 8: ay, bit 9: az, bit 10: is force setpoint,
*	bit 11: yaw, bit 12: yaw rat*/
INLINER void p86_type_mask_SET(uint16_t  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
/**
*Timestamp in milliseconds since system boot. The rationale for the timestamp in the setpoint is to allow
*	the system to compensate for the transport delay of the setpoint. This allows the system to compensate
*	processing latency*/
INLINER void p86_time_boot_ms_SET(uint32_t  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  2);
}
INLINER void p86_target_system_SET(uint8_t  src, Pack * dst)//System ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  6);
}
INLINER void p86_target_component_SET(uint8_t  src, Pack * dst)//Component ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  7);
}
INLINER void p86_lat_int_SET(int32_t  src, Pack * dst)//X Position in WGS84 frame in 1e7 * meters
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  8);
}
INLINER void p86_lon_int_SET(int32_t  src, Pack * dst)//Y Position in WGS84 frame in 1e7 * meters
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  12);
}
INLINER void p86_alt_SET(float  src, Pack * dst)//Altitude in meters in AMSL altitude, not WGS84 if absolute or relative, above terrain if GLOBAL_TERRAIN_ALT_IN
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  16);
}
INLINER void p86_vx_SET(float  src, Pack * dst)//X velocity in NED frame in meter / s
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  20);
}
INLINER void p86_vy_SET(float  src, Pack * dst)//Y velocity in NED frame in meter / s
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  24);
}
INLINER void p86_vz_SET(float  src, Pack * dst)//Z velocity in NED frame in meter / s
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  28);
}
INLINER void p86_afx_SET(float  src, Pack * dst)//X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  32);
}
INLINER void p86_afy_SET(float  src, Pack * dst)//Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  36);
}
INLINER void p86_afz_SET(float  src, Pack * dst)//Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  40);
}
INLINER void p86_yaw_SET(float  src, Pack * dst)//yaw setpoint in rad
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  44);
}
INLINER void p86_yaw_rate_SET(float  src, Pack * dst)//yaw rate setpoint in rad/s
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  48);
}
/**
*Valid options are: MAV_FRAME_GLOBAL_INT = 5, MAV_FRAME_GLOBAL_RELATIVE_ALT_INT = 6, MAV_FRAME_GLOBAL_TERRAIN_ALT_INT
*	= 1*/
INLINER void p86_coordinate_frame_SET(e_MAV_FRAME  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 4, data, 416);
}
/**
*Bitmask to indicate which dimensions should be ignored by the vehicle: a value of 0b0000000000000000 or
*	0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. If bit 10 is set
*	the floats afx afy afz should be interpreted as force instead of acceleration. Mapping: bit 1: x, bit
*	2: y, bit 3: z, bit 4: vx, bit 5: vy, bit 6: vz, bit 7: ax, bit 8: ay, bit 9: az, bit 10: is force setpoint,
*	bit 11: yaw, bit 12: yaw rat*/
INLINER uint16_t p87_type_mask_GET(Pack * src)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
/**
*Bitmask to indicate which dimensions should be ignored by the vehicle: a value of 0b0000000000000000 or
*	0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. If bit 10 is set
*	the floats afx afy afz should be interpreted as force instead of acceleration. Mapping: bit 1: x, bit
*	2: y, bit 3: z, bit 4: vx, bit 5: vy, bit 6: vz, bit 7: ax, bit 8: ay, bit 9: az, bit 10: is force setpoint,
*	bit 11: yaw, bit 12: yaw rat*/
INLINER void p87_type_mask_SET(uint16_t  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
/**
*Timestamp in milliseconds since system boot. The rationale for the timestamp in the setpoint is to allow
*	the system to compensate for the transport delay of the setpoint. This allows the system to compensate
*	processing latency*/
INLINER uint32_t p87_time_boot_ms_GET(Pack * src)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 4)));
}
/**
*Timestamp in milliseconds since system boot. The rationale for the timestamp in the setpoint is to allow
*	the system to compensate for the transport delay of the setpoint. This allows the system to compensate
*	processing latency*/
INLINER void p87_time_boot_ms_SET(uint32_t  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  2);
}
INLINER int32_t p87_lat_int_GET(Pack * src)//X Position in WGS84 frame in 1e7 * meters
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  6, 4)));
}
INLINER void p87_lat_int_SET(int32_t  src, Pack * dst)//X Position in WGS84 frame in 1e7 * meters
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  6);
}
INLINER int32_t p87_lon_int_GET(Pack * src)//Y Position in WGS84 frame in 1e7 * meters
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  10, 4)));
}
INLINER void p87_lon_int_SET(int32_t  src, Pack * dst)//Y Position in WGS84 frame in 1e7 * meters
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  10);
}
INLINER float p87_alt_GET(Pack * src)//Altitude in meters in AMSL altitude, not WGS84 if absolute or relative, above terrain if GLOBAL_TERRAIN_ALT_IN
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  14, 4)));
}
INLINER void p87_alt_SET(float  src, Pack * dst)//Altitude in meters in AMSL altitude, not WGS84 if absolute or relative, above terrain if GLOBAL_TERRAIN_ALT_IN
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  14);
}
INLINER float p87_vx_GET(Pack * src)//X velocity in NED frame in meter / s
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  18, 4)));
}
INLINER void p87_vx_SET(float  src, Pack * dst)//X velocity in NED frame in meter / s
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  18);
}
INLINER float p87_vy_GET(Pack * src)//Y velocity in NED frame in meter / s
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  22, 4)));
}
INLINER void p87_vy_SET(float  src, Pack * dst)//Y velocity in NED frame in meter / s
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  22);
}
INLINER float p87_vz_GET(Pack * src)//Z velocity in NED frame in meter / s
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  26, 4)));
}
INLINER void p87_vz_SET(float  src, Pack * dst)//Z velocity in NED frame in meter / s
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  26);
}
INLINER float p87_afx_GET(Pack * src)//X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  30, 4)));
}
INLINER void p87_afx_SET(float  src, Pack * dst)//X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  30);
}
INLINER float p87_afy_GET(Pack * src)//Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  34, 4)));
}
INLINER void p87_afy_SET(float  src, Pack * dst)//Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  34);
}
INLINER float p87_afz_GET(Pack * src)//Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  38, 4)));
}
INLINER void p87_afz_SET(float  src, Pack * dst)//Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  38);
}
INLINER float p87_yaw_GET(Pack * src)//yaw setpoint in rad
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  42, 4)));
}
INLINER void p87_yaw_SET(float  src, Pack * dst)//yaw setpoint in rad
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  42);
}
INLINER float p87_yaw_rate_GET(Pack * src)//yaw rate setpoint in rad/s
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  46, 4)));
}
INLINER void p87_yaw_rate_SET(float  src, Pack * dst)//yaw rate setpoint in rad/s
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  46);
}
/**
*Valid options are: MAV_FRAME_GLOBAL_INT = 5, MAV_FRAME_GLOBAL_RELATIVE_ALT_INT = 6, MAV_FRAME_GLOBAL_TERRAIN_ALT_INT
*	= 1*/
INLINER e_MAV_FRAME p87_coordinate_frame_GET(Pack * src)
{
    uint8_t * data = src->data;
    return  0 + (int)get_bits(data, 400, 4);
}
/**
*Valid options are: MAV_FRAME_GLOBAL_INT = 5, MAV_FRAME_GLOBAL_RELATIVE_ALT_INT = 6, MAV_FRAME_GLOBAL_TERRAIN_ALT_INT
*	= 1*/
INLINER void p87_coordinate_frame_SET(e_MAV_FRAME  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 4, data, 400);
}
INLINER uint32_t p89_time_boot_ms_GET(Pack * src)//Timestamp (milliseconds since system boot)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 4)));
}
INLINER void p89_time_boot_ms_SET(uint32_t  src, Pack * dst)//Timestamp (milliseconds since system boot)
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  0);
}
INLINER float p89_x_GET(Pack * src)//X Position
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  4, 4)));
}
INLINER void p89_x_SET(float  src, Pack * dst)//X Position
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  4);
}
INLINER float p89_y_GET(Pack * src)//Y Position
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  8, 4)));
}
INLINER void p89_y_SET(float  src, Pack * dst)//Y Position
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  8);
}
INLINER float p89_z_GET(Pack * src)//Z Position
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  12, 4)));
}
INLINER void p89_z_SET(float  src, Pack * dst)//Z Position
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  12);
}
INLINER float p89_roll_GET(Pack * src)//Roll
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  16, 4)));
}
INLINER void p89_roll_SET(float  src, Pack * dst)//Roll
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  16);
}
INLINER float p89_pitch_GET(Pack * src)//Pitch
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  20, 4)));
}
INLINER void p89_pitch_SET(float  src, Pack * dst)//Pitch
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  20);
}
INLINER float p89_yaw_GET(Pack * src)//Yaw
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  24, 4)));
}
INLINER void p89_yaw_SET(float  src, Pack * dst)//Yaw
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  24);
}
INLINER uint64_t p90_time_usec_GET(Pack * src)//Timestamp (microseconds since UNIX epoch or microseconds since system boot)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 8)));
}
INLINER void p90_time_usec_SET(uint64_t  src, Pack * dst)//Timestamp (microseconds since UNIX epoch or microseconds since system boot)
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  0);
}
INLINER float p90_roll_GET(Pack * src)//Roll angle (rad)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  8, 4)));
}
INLINER void p90_roll_SET(float  src, Pack * dst)//Roll angle (rad)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  8);
}
INLINER float p90_pitch_GET(Pack * src)//Pitch angle (rad)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  12, 4)));
}
INLINER void p90_pitch_SET(float  src, Pack * dst)//Pitch angle (rad)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  12);
}
INLINER float p90_yaw_GET(Pack * src)//Yaw angle (rad)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  16, 4)));
}
INLINER void p90_yaw_SET(float  src, Pack * dst)//Yaw angle (rad)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  16);
}
INLINER float p90_rollspeed_GET(Pack * src)//Body frame roll / phi angular speed (rad/s)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  20, 4)));
}
INLINER void p90_rollspeed_SET(float  src, Pack * dst)//Body frame roll / phi angular speed (rad/s)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  20);
}
INLINER float p90_pitchspeed_GET(Pack * src)//Body frame pitch / theta angular speed (rad/s)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  24, 4)));
}
INLINER void p90_pitchspeed_SET(float  src, Pack * dst)//Body frame pitch / theta angular speed (rad/s)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  24);
}
INLINER float p90_yawspeed_GET(Pack * src)//Body frame yaw / psi angular speed (rad/s)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  28, 4)));
}
INLINER void p90_yawspeed_SET(float  src, Pack * dst)//Body frame yaw / psi angular speed (rad/s)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  28);
}
INLINER int32_t p90_lat_GET(Pack * src)//Latitude, expressed as * 1E7
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  32, 4)));
}
INLINER void p90_lat_SET(int32_t  src, Pack * dst)//Latitude, expressed as * 1E7
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  32);
}
INLINER int32_t p90_lon_GET(Pack * src)//Longitude, expressed as * 1E7
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  36, 4)));
}
INLINER void p90_lon_SET(int32_t  src, Pack * dst)//Longitude, expressed as * 1E7
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  36);
}
INLINER int32_t p90_alt_GET(Pack * src)//Altitude in meters, expressed as * 1000 (millimeters)
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  40, 4)));
}
INLINER void p90_alt_SET(int32_t  src, Pack * dst)//Altitude in meters, expressed as * 1000 (millimeters)
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  40);
}
INLINER int16_t p90_vx_GET(Pack * src)//Ground X Speed (Latitude), expressed as m/s * 100
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  44, 2)));
}
INLINER void p90_vx_SET(int16_t  src, Pack * dst)//Ground X Speed (Latitude), expressed as m/s * 100
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  44);
}
INLINER int16_t p90_vy_GET(Pack * src)//Ground Y Speed (Longitude), expressed as m/s * 100
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  46, 2)));
}
INLINER void p90_vy_SET(int16_t  src, Pack * dst)//Ground Y Speed (Longitude), expressed as m/s * 100
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  46);
}
INLINER int16_t p90_vz_GET(Pack * src)//Ground Z Speed (Altitude), expressed as m/s * 100
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  48, 2)));
}
INLINER void p90_vz_SET(int16_t  src, Pack * dst)//Ground Z Speed (Altitude), expressed as m/s * 100
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  48);
}
INLINER int16_t p90_xacc_GET(Pack * src)//X acceleration (mg)
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  50, 2)));
}
INLINER void p90_xacc_SET(int16_t  src, Pack * dst)//X acceleration (mg)
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  50);
}
INLINER int16_t p90_yacc_GET(Pack * src)//Y acceleration (mg)
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  52, 2)));
}
INLINER void p90_yacc_SET(int16_t  src, Pack * dst)//Y acceleration (mg)
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  52);
}
INLINER int16_t p90_zacc_GET(Pack * src)//Z acceleration (mg)
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  54, 2)));
}
INLINER void p90_zacc_SET(int16_t  src, Pack * dst)//Z acceleration (mg)
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  54);
}
INLINER uint64_t p91_time_usec_GET(Pack * src)//Timestamp (microseconds since UNIX epoch or microseconds since system boot)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 8)));
}
INLINER void p91_time_usec_SET(uint64_t  src, Pack * dst)//Timestamp (microseconds since UNIX epoch or microseconds since system boot)
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  0);
}
INLINER float p91_roll_ailerons_GET(Pack * src)//Control output -1 .. 1
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  8, 4)));
}
INLINER void p91_roll_ailerons_SET(float  src, Pack * dst)//Control output -1 .. 1
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  8);
}
INLINER float p91_pitch_elevator_GET(Pack * src)//Control output -1 .. 1
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  12, 4)));
}
INLINER void p91_pitch_elevator_SET(float  src, Pack * dst)//Control output -1 .. 1
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  12);
}
INLINER float p91_yaw_rudder_GET(Pack * src)//Control output -1 .. 1
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  16, 4)));
}
INLINER void p91_yaw_rudder_SET(float  src, Pack * dst)//Control output -1 .. 1
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  16);
}
INLINER float p91_throttle_GET(Pack * src)//Throttle 0 .. 1
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  20, 4)));
}
INLINER void p91_throttle_SET(float  src, Pack * dst)//Throttle 0 .. 1
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  20);
}
INLINER float p91_aux1_GET(Pack * src)//Aux 1, -1 .. 1
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  24, 4)));
}
INLINER void p91_aux1_SET(float  src, Pack * dst)//Aux 1, -1 .. 1
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  24);
}
INLINER float p91_aux2_GET(Pack * src)//Aux 2, -1 .. 1
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  28, 4)));
}
INLINER void p91_aux2_SET(float  src, Pack * dst)//Aux 2, -1 .. 1
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  28);
}
INLINER float p91_aux3_GET(Pack * src)//Aux 3, -1 .. 1
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  32, 4)));
}
INLINER void p91_aux3_SET(float  src, Pack * dst)//Aux 3, -1 .. 1
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  32);
}
INLINER float p91_aux4_GET(Pack * src)//Aux 4, -1 .. 1
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  36, 4)));
}
INLINER void p91_aux4_SET(float  src, Pack * dst)//Aux 4, -1 .. 1
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  36);
}
INLINER uint8_t p91_nav_mode_GET(Pack * src)//Navigation mode (MAV_NAV_MODE)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  40, 1)));
}
INLINER void p91_nav_mode_SET(uint8_t  src, Pack * dst)//Navigation mode (MAV_NAV_MODE)
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  40);
}
INLINER e_MAV_MODE p91_mode_GET(Pack * src)//System mode (MAV_MODE)
{
    uint8_t * data = src->data;
    return  _en__a(get_bits(data, 328, 4));
}
INLINER void p91_mode_SET(e_MAV_MODE  src, Pack * dst)//System mode (MAV_MODE)
{
    uint8_t * data = dst->data;
    UMAX id = _id__a(src);
    set_bits(id, 4, data, 328);
}
INLINER uint16_t p92_chan1_raw_GET(Pack * src)//RC channel 1 value, in microseconds
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER void p92_chan1_raw_SET(uint16_t  src, Pack * dst)//RC channel 1 value, in microseconds
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER uint16_t p92_chan2_raw_GET(Pack * src)//RC channel 2 value, in microseconds
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 2)));
}
INLINER void p92_chan2_raw_SET(uint16_t  src, Pack * dst)//RC channel 2 value, in microseconds
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  2);
}
INLINER uint16_t p92_chan3_raw_GET(Pack * src)//RC channel 3 value, in microseconds
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  4, 2)));
}
INLINER void p92_chan3_raw_SET(uint16_t  src, Pack * dst)//RC channel 3 value, in microseconds
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  4);
}
INLINER uint16_t p92_chan4_raw_GET(Pack * src)//RC channel 4 value, in microseconds
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  6, 2)));
}
INLINER void p92_chan4_raw_SET(uint16_t  src, Pack * dst)//RC channel 4 value, in microseconds
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  6);
}
INLINER uint16_t p92_chan5_raw_GET(Pack * src)//RC channel 5 value, in microseconds
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  8, 2)));
}
INLINER void p92_chan5_raw_SET(uint16_t  src, Pack * dst)//RC channel 5 value, in microseconds
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  8);
}
INLINER uint16_t p92_chan6_raw_GET(Pack * src)//RC channel 6 value, in microseconds
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  10, 2)));
}
INLINER void p92_chan6_raw_SET(uint16_t  src, Pack * dst)//RC channel 6 value, in microseconds
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  10);
}
INLINER uint16_t p92_chan7_raw_GET(Pack * src)//RC channel 7 value, in microseconds
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  12, 2)));
}
INLINER void p92_chan7_raw_SET(uint16_t  src, Pack * dst)//RC channel 7 value, in microseconds
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  12);
}
INLINER uint16_t p92_chan8_raw_GET(Pack * src)//RC channel 8 value, in microseconds
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  14, 2)));
}
INLINER void p92_chan8_raw_SET(uint16_t  src, Pack * dst)//RC channel 8 value, in microseconds
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  14);
}
INLINER uint16_t p92_chan9_raw_GET(Pack * src)//RC channel 9 value, in microseconds
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  16, 2)));
}
INLINER void p92_chan9_raw_SET(uint16_t  src, Pack * dst)//RC channel 9 value, in microseconds
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  16);
}
INLINER uint16_t p92_chan10_raw_GET(Pack * src)//RC channel 10 value, in microseconds
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  18, 2)));
}
INLINER void p92_chan10_raw_SET(uint16_t  src, Pack * dst)//RC channel 10 value, in microseconds
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  18);
}
INLINER uint16_t p92_chan11_raw_GET(Pack * src)//RC channel 11 value, in microseconds
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  20, 2)));
}
INLINER void p92_chan11_raw_SET(uint16_t  src, Pack * dst)//RC channel 11 value, in microseconds
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  20);
}
INLINER uint16_t p92_chan12_raw_GET(Pack * src)//RC channel 12 value, in microseconds
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  22, 2)));
}
INLINER void p92_chan12_raw_SET(uint16_t  src, Pack * dst)//RC channel 12 value, in microseconds
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  22);
}
INLINER uint64_t p92_time_usec_GET(Pack * src)//Timestamp (microseconds since UNIX epoch or microseconds since system boot)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  24, 8)));
}
INLINER void p92_time_usec_SET(uint64_t  src, Pack * dst)//Timestamp (microseconds since UNIX epoch or microseconds since system boot)
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  24);
}
INLINER uint8_t p92_rssi_GET(Pack * src)//Receive signal strength indicator, 0: 0%, 255: 100%
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  32, 1)));
}
INLINER void p92_rssi_SET(uint8_t  src, Pack * dst)//Receive signal strength indicator, 0: 0%, 255: 100%
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  32);
}
INLINER uint64_t p93_time_usec_GET(Pack * src)//Timestamp (microseconds since UNIX epoch or microseconds since system boot)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 8)));
}
INLINER void p93_time_usec_SET(uint64_t  src, Pack * dst)//Timestamp (microseconds since UNIX epoch or microseconds since system boot)
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  0);
}
INLINER uint64_t p93_flags_GET(Pack * src)//Flags as bitfield, reserved for future use.
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  8, 8)));
}
INLINER void p93_flags_SET(uint64_t  src, Pack * dst)//Flags as bitfield, reserved for future use.
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  8);
}
INLINER float* p93_controls_GET(Pack * src, float*  dst, int32_t pos) //Control outputs -1 .. 1. Channel assignment depends on the simulated hardware.
{
    uint8_t * data = src->data;
    for(int32_t BYTE = 16, dst_max = pos + 16; pos < dst_max ; pos++,  BYTE += 4)
        dst[pos] = (intBitsToFloat(get_bytes(data,  BYTE, 4)));
    return dst;
}

static const  uint32_t p93_controls_LEN = 16; //return array length

INLINER  float*  p93_controls_GET_(Pack * src) {return p93_controls_GET(src, malloc(16 * sizeof(float)), 0);}
INLINER void p93_controls_SET(float*  src, int32_t pos, Pack * dst) //Control outputs -1 .. 1. Channel assignment depends on the simulated hardware.
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  16, src_max = pos + 16; pos < src_max; pos++, BYTE += 4)
        set_bytes(floatToIntBits(src[pos]), 4, data,  BYTE);
}
INLINER e_MAV_MODE p93_mode_GET(Pack * src)//System mode (MAV_MODE), includes arming state.
{
    uint8_t * data = src->data;
    return  _en__a(get_bits(data, 640, 4));
}
INLINER void p93_mode_SET(e_MAV_MODE  src, Pack * dst)//System mode (MAV_MODE), includes arming state.
{
    uint8_t * data = dst->data;
    UMAX id = _id__a(src);
    set_bits(id, 4, data, 640);
}
INLINER uint64_t p100_time_usec_GET(Pack * src)//Timestamp (UNIX)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 8)));
}
INLINER void p100_time_usec_SET(uint64_t  src, Pack * dst)//Timestamp (UNIX)
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  0);
}
INLINER uint8_t p100_sensor_id_GET(Pack * src)//Sensor ID
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  8, 1)));
}
INLINER void p100_sensor_id_SET(uint8_t  src, Pack * dst)//Sensor ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  8);
}
INLINER int16_t p100_flow_x_GET(Pack * src)//Flow in pixels * 10 in x-sensor direction (dezi-pixels)
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  9, 2)));
}
INLINER void p100_flow_x_SET(int16_t  src, Pack * dst)//Flow in pixels * 10 in x-sensor direction (dezi-pixels)
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  9);
}
INLINER int16_t p100_flow_y_GET(Pack * src)//Flow in pixels * 10 in y-sensor direction (dezi-pixels)
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  11, 2)));
}
INLINER void p100_flow_y_SET(int16_t  src, Pack * dst)//Flow in pixels * 10 in y-sensor direction (dezi-pixels)
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  11);
}
INLINER float p100_flow_comp_m_x_GET(Pack * src)//Flow in meters in x-sensor direction, angular-speed compensated
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  13, 4)));
}
INLINER void p100_flow_comp_m_x_SET(float  src, Pack * dst)//Flow in meters in x-sensor direction, angular-speed compensated
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  13);
}
INLINER float p100_flow_comp_m_y_GET(Pack * src)//Flow in meters in y-sensor direction, angular-speed compensated
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  17, 4)));
}
INLINER void p100_flow_comp_m_y_SET(float  src, Pack * dst)//Flow in meters in y-sensor direction, angular-speed compensated
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  17);
}
INLINER uint8_t p100_quality_GET(Pack * src)//Optical flow quality / confidence. 0: bad, 255: maximum quality
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  21, 1)));
}
INLINER void p100_quality_SET(uint8_t  src, Pack * dst)//Optical flow quality / confidence. 0: bad, 255: maximum quality
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  21);
}
INLINER float p100_ground_distance_GET(Pack * src)//Ground distance in meters. Positive value: distance known. Negative value: Unknown distance
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  22, 4)));
}
INLINER void p100_ground_distance_SET(float  src, Pack * dst)//Ground distance in meters. Positive value: distance known. Negative value: Unknown distance
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
INLINER void p100_flow_rate_x_SET(float  src, Bounds_Inside * dst)//Flow rate in radians/second about X axis
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
INLINER void p100_flow_rate_y_SET(float  src, Bounds_Inside * dst)//Flow rate in radians/second about Y axis
{
    if(dst->base.field_bit != 209)insert_field(dst, 209, 0);
    uint8_t * data = dst->base.pack->data;
    set_bytes(floatToIntBits(src), 4, data,  dst->BYTE);
}
INLINER uint64_t p101_usec_GET(Pack * src)//Timestamp (microseconds, synced to UNIX time or since system boot)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 8)));
}
INLINER void p101_usec_SET(uint64_t  src, Pack * dst)//Timestamp (microseconds, synced to UNIX time or since system boot)
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  0);
}
INLINER float p101_x_GET(Pack * src)//Global X position
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  8, 4)));
}
INLINER void p101_x_SET(float  src, Pack * dst)//Global X position
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  8);
}
INLINER float p101_y_GET(Pack * src)//Global Y position
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  12, 4)));
}
INLINER void p101_y_SET(float  src, Pack * dst)//Global Y position
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  12);
}
INLINER float p101_z_GET(Pack * src)//Global Z position
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  16, 4)));
}
INLINER void p101_z_SET(float  src, Pack * dst)//Global Z position
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  16);
}
INLINER float p101_roll_GET(Pack * src)//Roll angle in rad
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  20, 4)));
}
INLINER void p101_roll_SET(float  src, Pack * dst)//Roll angle in rad
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  20);
}
INLINER float p101_pitch_GET(Pack * src)//Pitch angle in rad
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  24, 4)));
}
INLINER void p101_pitch_SET(float  src, Pack * dst)//Pitch angle in rad
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  24);
}
INLINER float p101_yaw_GET(Pack * src)//Yaw angle in rad
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  28, 4)));
}
INLINER void p101_yaw_SET(float  src, Pack * dst)//Yaw angle in rad
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  28);
}
INLINER uint64_t p102_usec_GET(Pack * src)//Timestamp (microseconds, synced to UNIX time or since system boot)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 8)));
}
INLINER void p102_usec_SET(uint64_t  src, Pack * dst)//Timestamp (microseconds, synced to UNIX time or since system boot)
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  0);
}
INLINER float p102_x_GET(Pack * src)//Global X position
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  8, 4)));
}
INLINER void p102_x_SET(float  src, Pack * dst)//Global X position
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  8);
}
INLINER float p102_y_GET(Pack * src)//Global Y position
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  12, 4)));
}
INLINER void p102_y_SET(float  src, Pack * dst)//Global Y position
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  12);
}
INLINER float p102_z_GET(Pack * src)//Global Z position
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  16, 4)));
}
INLINER void p102_z_SET(float  src, Pack * dst)//Global Z position
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  16);
}
INLINER float p102_roll_GET(Pack * src)//Roll angle in rad
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  20, 4)));
}
INLINER void p102_roll_SET(float  src, Pack * dst)//Roll angle in rad
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  20);
}
INLINER float p102_pitch_GET(Pack * src)//Pitch angle in rad
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  24, 4)));
}
INLINER void p102_pitch_SET(float  src, Pack * dst)//Pitch angle in rad
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  24);
}
INLINER float p102_yaw_GET(Pack * src)//Yaw angle in rad
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  28, 4)));
}
INLINER void p102_yaw_SET(float  src, Pack * dst)//Yaw angle in rad
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  28);
}
INLINER uint64_t p103_usec_GET(Pack * src)//Timestamp (microseconds, synced to UNIX time or since system boot)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 8)));
}
INLINER void p103_usec_SET(uint64_t  src, Pack * dst)//Timestamp (microseconds, synced to UNIX time or since system boot)
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  0);
}
INLINER float p103_x_GET(Pack * src)//Global X speed
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  8, 4)));
}
INLINER void p103_x_SET(float  src, Pack * dst)//Global X speed
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  8);
}
INLINER float p103_y_GET(Pack * src)//Global Y speed
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  12, 4)));
}
INLINER void p103_y_SET(float  src, Pack * dst)//Global Y speed
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  12);
}
INLINER float p103_z_GET(Pack * src)//Global Z speed
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  16, 4)));
}
INLINER void p103_z_SET(float  src, Pack * dst)//Global Z speed
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  16);
}
INLINER uint64_t p104_usec_GET(Pack * src)//Timestamp (microseconds, synced to UNIX time or since system boot)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 8)));
}
INLINER void p104_usec_SET(uint64_t  src, Pack * dst)//Timestamp (microseconds, synced to UNIX time or since system boot)
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  0);
}
INLINER float p104_x_GET(Pack * src)//Global X position
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  8, 4)));
}
INLINER void p104_x_SET(float  src, Pack * dst)//Global X position
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  8);
}
INLINER float p104_y_GET(Pack * src)//Global Y position
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  12, 4)));
}
INLINER void p104_y_SET(float  src, Pack * dst)//Global Y position
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  12);
}
INLINER float p104_z_GET(Pack * src)//Global Z position
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  16, 4)));
}
INLINER void p104_z_SET(float  src, Pack * dst)//Global Z position
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  16);
}
INLINER float p104_roll_GET(Pack * src)//Roll angle in rad
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  20, 4)));
}
INLINER void p104_roll_SET(float  src, Pack * dst)//Roll angle in rad
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  20);
}
INLINER float p104_pitch_GET(Pack * src)//Pitch angle in rad
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  24, 4)));
}
INLINER void p104_pitch_SET(float  src, Pack * dst)//Pitch angle in rad
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  24);
}
INLINER float p104_yaw_GET(Pack * src)//Yaw angle in rad
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  28, 4)));
}
INLINER void p104_yaw_SET(float  src, Pack * dst)//Yaw angle in rad
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  28);
}
INLINER uint16_t p105_fields_updated_GET(Pack * src)//Bitmask for fields that have updated since last message, bit 0 = xacc, bit 12: temperature
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER void p105_fields_updated_SET(uint16_t  src, Pack * dst)//Bitmask for fields that have updated since last message, bit 0 = xacc, bit 12: temperature
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER uint64_t p105_time_usec_GET(Pack * src)//Timestamp (microseconds, synced to UNIX time or since system boot)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 8)));
}
INLINER void p105_time_usec_SET(uint64_t  src, Pack * dst)//Timestamp (microseconds, synced to UNIX time or since system boot)
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  2);
}
INLINER float p105_xacc_GET(Pack * src)//X acceleration (m/s^2)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  10, 4)));
}
INLINER void p105_xacc_SET(float  src, Pack * dst)//X acceleration (m/s^2)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  10);
}
INLINER float p105_yacc_GET(Pack * src)//Y acceleration (m/s^2)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  14, 4)));
}
INLINER void p105_yacc_SET(float  src, Pack * dst)//Y acceleration (m/s^2)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  14);
}
INLINER float p105_zacc_GET(Pack * src)//Z acceleration (m/s^2)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  18, 4)));
}
INLINER void p105_zacc_SET(float  src, Pack * dst)//Z acceleration (m/s^2)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  18);
}
INLINER float p105_xgyro_GET(Pack * src)//Angular speed around X axis (rad / sec)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  22, 4)));
}
INLINER void p105_xgyro_SET(float  src, Pack * dst)//Angular speed around X axis (rad / sec)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  22);
}
INLINER float p105_ygyro_GET(Pack * src)//Angular speed around Y axis (rad / sec)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  26, 4)));
}
INLINER void p105_ygyro_SET(float  src, Pack * dst)//Angular speed around Y axis (rad / sec)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  26);
}
INLINER float p105_zgyro_GET(Pack * src)//Angular speed around Z axis (rad / sec)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  30, 4)));
}
INLINER void p105_zgyro_SET(float  src, Pack * dst)//Angular speed around Z axis (rad / sec)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  30);
}
INLINER float p105_xmag_GET(Pack * src)//X Magnetic field (Gauss)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  34, 4)));
}
INLINER void p105_xmag_SET(float  src, Pack * dst)//X Magnetic field (Gauss)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  34);
}
INLINER float p105_ymag_GET(Pack * src)//Y Magnetic field (Gauss)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  38, 4)));
}
INLINER void p105_ymag_SET(float  src, Pack * dst)//Y Magnetic field (Gauss)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  38);
}
INLINER float p105_zmag_GET(Pack * src)//Z Magnetic field (Gauss)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  42, 4)));
}
INLINER void p105_zmag_SET(float  src, Pack * dst)//Z Magnetic field (Gauss)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  42);
}
INLINER float p105_abs_pressure_GET(Pack * src)//Absolute pressure in millibar
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  46, 4)));
}
INLINER void p105_abs_pressure_SET(float  src, Pack * dst)//Absolute pressure in millibar
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  46);
}
INLINER float p105_diff_pressure_GET(Pack * src)//Differential pressure in millibar
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  50, 4)));
}
INLINER void p105_diff_pressure_SET(float  src, Pack * dst)//Differential pressure in millibar
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  50);
}
INLINER float p105_pressure_alt_GET(Pack * src)//Altitude calculated from pressure
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  54, 4)));
}
INLINER void p105_pressure_alt_SET(float  src, Pack * dst)//Altitude calculated from pressure
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  54);
}
INLINER float p105_temperature_GET(Pack * src)//Temperature in degrees celsius
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  58, 4)));
}
INLINER void p105_temperature_SET(float  src, Pack * dst)//Temperature in degrees celsius
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  58);
}
/**
*Integration time in microseconds. Divide integrated_x and integrated_y by the integration time to obtain
*	average flow. The integration time also indicates the*/
INLINER uint32_t p106_integration_time_us_GET(Pack * src)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 4)));
}
/**
*Integration time in microseconds. Divide integrated_x and integrated_y by the integration time to obtain
*	average flow. The integration time also indicates the*/
INLINER void p106_integration_time_us_SET(uint32_t  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  0);
}
INLINER uint32_t p106_time_delta_distance_us_GET(Pack * src)//Time in microseconds since the distance was sampled.
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  4, 4)));
}
INLINER void p106_time_delta_distance_us_SET(uint32_t  src, Pack * dst)//Time in microseconds since the distance was sampled.
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  4);
}
INLINER uint64_t p106_time_usec_GET(Pack * src)//Timestamp (microseconds, synced to UNIX time or since system boot)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  8, 8)));
}
INLINER void p106_time_usec_SET(uint64_t  src, Pack * dst)//Timestamp (microseconds, synced to UNIX time or since system boot)
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  8);
}
INLINER uint8_t p106_sensor_id_GET(Pack * src)//Sensor ID
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  16, 1)));
}
INLINER void p106_sensor_id_SET(uint8_t  src, Pack * dst)//Sensor ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  16);
}
/**
*Flow in radians around X axis (Sensor RH rotation about the X axis induces a positive flow. Sensor linear
*	motion along the positive Y axis induces a negative flow.*/
INLINER float p106_integrated_x_GET(Pack * src)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  17, 4)));
}
/**
*Flow in radians around X axis (Sensor RH rotation about the X axis induces a positive flow. Sensor linear
*	motion along the positive Y axis induces a negative flow.*/
INLINER void p106_integrated_x_SET(float  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  17);
}
/**
*Flow in radians around Y axis (Sensor RH rotation about the Y axis induces a positive flow. Sensor linear
*	motion along the positive X axis induces a positive flow.*/
INLINER float p106_integrated_y_GET(Pack * src)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  21, 4)));
}
/**
*Flow in radians around Y axis (Sensor RH rotation about the Y axis induces a positive flow. Sensor linear
*	motion along the positive X axis induces a positive flow.*/
INLINER void p106_integrated_y_SET(float  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  21);
}
INLINER float p106_integrated_xgyro_GET(Pack * src)//RH rotation around X axis (rad)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  25, 4)));
}
INLINER void p106_integrated_xgyro_SET(float  src, Pack * dst)//RH rotation around X axis (rad)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  25);
}
INLINER float p106_integrated_ygyro_GET(Pack * src)//RH rotation around Y axis (rad)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  29, 4)));
}
INLINER void p106_integrated_ygyro_SET(float  src, Pack * dst)//RH rotation around Y axis (rad)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  29);
}
INLINER float p106_integrated_zgyro_GET(Pack * src)//RH rotation around Z axis (rad)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  33, 4)));
}
INLINER void p106_integrated_zgyro_SET(float  src, Pack * dst)//RH rotation around Z axis (rad)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  33);
}
INLINER int16_t p106_temperature_GET(Pack * src)//Temperature * 100 in centi-degrees Celsius
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  37, 2)));
}
INLINER void p106_temperature_SET(int16_t  src, Pack * dst)//Temperature * 100 in centi-degrees Celsius
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  37);
}
INLINER uint8_t p106_quality_GET(Pack * src)//Optical flow quality / confidence. 0: no valid flow, 255: maximum quality
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  39, 1)));
}
INLINER void p106_quality_SET(uint8_t  src, Pack * dst)//Optical flow quality / confidence. 0: no valid flow, 255: maximum quality
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  39);
}
/**
*Distance to the center of the flow field in meters. Positive value (including zero): distance known. Negative
*	value: Unknown distance*/
INLINER float p106_distance_GET(Pack * src)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  40, 4)));
}
/**
*Distance to the center of the flow field in meters. Positive value (including zero): distance known. Negative
*	value: Unknown distance*/
INLINER void p106_distance_SET(float  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  40);
}
/**
*Bitmask for fields that have updated since last message, bit 0 = xacc, bit 12: temperature, bit 31: full
*	reset of attitude/position/velocities/etc was performed in sim*/
INLINER uint32_t p107_fields_updated_GET(Pack * src)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 4)));
}
/**
*Bitmask for fields that have updated since last message, bit 0 = xacc, bit 12: temperature, bit 31: full
*	reset of attitude/position/velocities/etc was performed in sim*/
INLINER void p107_fields_updated_SET(uint32_t  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  0);
}
INLINER uint64_t p107_time_usec_GET(Pack * src)//Timestamp (microseconds, synced to UNIX time or since system boot)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  4, 8)));
}
INLINER void p107_time_usec_SET(uint64_t  src, Pack * dst)//Timestamp (microseconds, synced to UNIX time or since system boot)
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  4);
}
INLINER float p107_xacc_GET(Pack * src)//X acceleration (m/s^2)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  12, 4)));
}
INLINER void p107_xacc_SET(float  src, Pack * dst)//X acceleration (m/s^2)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  12);
}
INLINER float p107_yacc_GET(Pack * src)//Y acceleration (m/s^2)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  16, 4)));
}
INLINER void p107_yacc_SET(float  src, Pack * dst)//Y acceleration (m/s^2)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  16);
}
INLINER float p107_zacc_GET(Pack * src)//Z acceleration (m/s^2)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  20, 4)));
}
INLINER void p107_zacc_SET(float  src, Pack * dst)//Z acceleration (m/s^2)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  20);
}
INLINER float p107_xgyro_GET(Pack * src)//Angular speed around X axis in body frame (rad / sec)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  24, 4)));
}
INLINER void p107_xgyro_SET(float  src, Pack * dst)//Angular speed around X axis in body frame (rad / sec)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  24);
}
INLINER float p107_ygyro_GET(Pack * src)//Angular speed around Y axis in body frame (rad / sec)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  28, 4)));
}
INLINER void p107_ygyro_SET(float  src, Pack * dst)//Angular speed around Y axis in body frame (rad / sec)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  28);
}
INLINER float p107_zgyro_GET(Pack * src)//Angular speed around Z axis in body frame (rad / sec)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  32, 4)));
}
INLINER void p107_zgyro_SET(float  src, Pack * dst)//Angular speed around Z axis in body frame (rad / sec)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  32);
}
INLINER float p107_xmag_GET(Pack * src)//X Magnetic field (Gauss)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  36, 4)));
}
INLINER void p107_xmag_SET(float  src, Pack * dst)//X Magnetic field (Gauss)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  36);
}
INLINER float p107_ymag_GET(Pack * src)//Y Magnetic field (Gauss)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  40, 4)));
}
INLINER void p107_ymag_SET(float  src, Pack * dst)//Y Magnetic field (Gauss)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  40);
}
INLINER float p107_zmag_GET(Pack * src)//Z Magnetic field (Gauss)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  44, 4)));
}
INLINER void p107_zmag_SET(float  src, Pack * dst)//Z Magnetic field (Gauss)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  44);
}
INLINER float p107_abs_pressure_GET(Pack * src)//Absolute pressure in millibar
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  48, 4)));
}
INLINER void p107_abs_pressure_SET(float  src, Pack * dst)//Absolute pressure in millibar
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  48);
}
INLINER float p107_diff_pressure_GET(Pack * src)//Differential pressure (airspeed) in millibar
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  52, 4)));
}
INLINER void p107_diff_pressure_SET(float  src, Pack * dst)//Differential pressure (airspeed) in millibar
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  52);
}
INLINER float p107_pressure_alt_GET(Pack * src)//Altitude calculated from pressure
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  56, 4)));
}
INLINER void p107_pressure_alt_SET(float  src, Pack * dst)//Altitude calculated from pressure
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  56);
}
INLINER float p107_temperature_GET(Pack * src)//Temperature in degrees celsius
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  60, 4)));
}
INLINER void p107_temperature_SET(float  src, Pack * dst)//Temperature in degrees celsius
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  60);
}
INLINER float p108_q1_GET(Pack * src)//True attitude quaternion component 1, w (1 in null-rotation)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  0, 4)));
}
INLINER void p108_q1_SET(float  src, Pack * dst)//True attitude quaternion component 1, w (1 in null-rotation)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  0);
}
INLINER float p108_q2_GET(Pack * src)//True attitude quaternion component 2, x (0 in null-rotation)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  4, 4)));
}
INLINER void p108_q2_SET(float  src, Pack * dst)//True attitude quaternion component 2, x (0 in null-rotation)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  4);
}
INLINER float p108_q3_GET(Pack * src)//True attitude quaternion component 3, y (0 in null-rotation)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  8, 4)));
}
INLINER void p108_q3_SET(float  src, Pack * dst)//True attitude quaternion component 3, y (0 in null-rotation)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  8);
}
INLINER float p108_q4_GET(Pack * src)//True attitude quaternion component 4, z (0 in null-rotation)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  12, 4)));
}
INLINER void p108_q4_SET(float  src, Pack * dst)//True attitude quaternion component 4, z (0 in null-rotation)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  12);
}
INLINER float p108_roll_GET(Pack * src)//Attitude roll expressed as Euler angles, not recommended except for human-readable outputs
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  16, 4)));
}
INLINER void p108_roll_SET(float  src, Pack * dst)//Attitude roll expressed as Euler angles, not recommended except for human-readable outputs
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  16);
}
INLINER float p108_pitch_GET(Pack * src)//Attitude pitch expressed as Euler angles, not recommended except for human-readable outputs
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  20, 4)));
}
INLINER void p108_pitch_SET(float  src, Pack * dst)//Attitude pitch expressed as Euler angles, not recommended except for human-readable outputs
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  20);
}
INLINER float p108_yaw_GET(Pack * src)//Attitude yaw expressed as Euler angles, not recommended except for human-readable outputs
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  24, 4)));
}
INLINER void p108_yaw_SET(float  src, Pack * dst)//Attitude yaw expressed as Euler angles, not recommended except for human-readable outputs
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  24);
}
INLINER float p108_xacc_GET(Pack * src)//X acceleration m/s/s
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  28, 4)));
}
INLINER void p108_xacc_SET(float  src, Pack * dst)//X acceleration m/s/s
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  28);
}
INLINER float p108_yacc_GET(Pack * src)//Y acceleration m/s/s
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  32, 4)));
}
INLINER void p108_yacc_SET(float  src, Pack * dst)//Y acceleration m/s/s
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  32);
}
INLINER float p108_zacc_GET(Pack * src)//Z acceleration m/s/s
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  36, 4)));
}
INLINER void p108_zacc_SET(float  src, Pack * dst)//Z acceleration m/s/s
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  36);
}
INLINER float p108_xgyro_GET(Pack * src)//Angular speed around X axis rad/s
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  40, 4)));
}
INLINER void p108_xgyro_SET(float  src, Pack * dst)//Angular speed around X axis rad/s
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  40);
}
INLINER float p108_ygyro_GET(Pack * src)//Angular speed around Y axis rad/s
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  44, 4)));
}
INLINER void p108_ygyro_SET(float  src, Pack * dst)//Angular speed around Y axis rad/s
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  44);
}
INLINER float p108_zgyro_GET(Pack * src)//Angular speed around Z axis rad/s
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  48, 4)));
}
INLINER void p108_zgyro_SET(float  src, Pack * dst)//Angular speed around Z axis rad/s
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  48);
}
INLINER float p108_lat_GET(Pack * src)//Latitude in degrees
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  52, 4)));
}
INLINER void p108_lat_SET(float  src, Pack * dst)//Latitude in degrees
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  52);
}
INLINER float p108_lon_GET(Pack * src)//Longitude in degrees
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  56, 4)));
}
INLINER void p108_lon_SET(float  src, Pack * dst)//Longitude in degrees
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  56);
}
INLINER float p108_alt_GET(Pack * src)//Altitude in meters
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  60, 4)));
}
INLINER void p108_alt_SET(float  src, Pack * dst)//Altitude in meters
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  60);
}
INLINER float p108_std_dev_horz_GET(Pack * src)//Horizontal position standard deviation
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  64, 4)));
}
INLINER void p108_std_dev_horz_SET(float  src, Pack * dst)//Horizontal position standard deviation
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  64);
}
INLINER float p108_std_dev_vert_GET(Pack * src)//Vertical position standard deviation
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  68, 4)));
}
INLINER void p108_std_dev_vert_SET(float  src, Pack * dst)//Vertical position standard deviation
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  68);
}
INLINER float p108_vn_GET(Pack * src)//True velocity in m/s in NORTH direction in earth-fixed NED frame
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  72, 4)));
}
INLINER void p108_vn_SET(float  src, Pack * dst)//True velocity in m/s in NORTH direction in earth-fixed NED frame
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  72);
}
INLINER float p108_ve_GET(Pack * src)//True velocity in m/s in EAST direction in earth-fixed NED frame
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  76, 4)));
}
INLINER void p108_ve_SET(float  src, Pack * dst)//True velocity in m/s in EAST direction in earth-fixed NED frame
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  76);
}
INLINER float p108_vd_GET(Pack * src)//True velocity in m/s in DOWN direction in earth-fixed NED frame
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  80, 4)));
}
INLINER void p108_vd_SET(float  src, Pack * dst)//True velocity in m/s in DOWN direction in earth-fixed NED frame
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  80);
}
INLINER uint16_t p109_rxerrors_GET(Pack * src)//Receive errors
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER void p109_rxerrors_SET(uint16_t  src, Pack * dst)//Receive errors
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER uint16_t p109_fixed__GET(Pack * src)//Count of error corrected packets
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 2)));
}
INLINER void p109_fixed__SET(uint16_t  src, Pack * dst)//Count of error corrected packets
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  2);
}
INLINER uint8_t p109_rssi_GET(Pack * src)//Local signal strength
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  4, 1)));
}
INLINER void p109_rssi_SET(uint8_t  src, Pack * dst)//Local signal strength
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  4);
}
INLINER uint8_t p109_remrssi_GET(Pack * src)//Remote signal strength
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  5, 1)));
}
INLINER void p109_remrssi_SET(uint8_t  src, Pack * dst)//Remote signal strength
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  5);
}
INLINER uint8_t p109_txbuf_GET(Pack * src)//Remaining free buffer space in percent.
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  6, 1)));
}
INLINER void p109_txbuf_SET(uint8_t  src, Pack * dst)//Remaining free buffer space in percent.
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  6);
}
INLINER uint8_t p109_noise_GET(Pack * src)//Background noise level
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  7, 1)));
}
INLINER void p109_noise_SET(uint8_t  src, Pack * dst)//Background noise level
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  7);
}
INLINER uint8_t p109_remnoise_GET(Pack * src)//Remote background noise level
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  8, 1)));
}
INLINER void p109_remnoise_SET(uint8_t  src, Pack * dst)//Remote background noise level
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  8);
}
INLINER uint8_t p110_target_network_GET(Pack * src)//Network ID (0 for broadcast)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 1)));
}
INLINER void p110_target_network_SET(uint8_t  src, Pack * dst)//Network ID (0 for broadcast)
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  0);
}
INLINER uint8_t p110_target_system_GET(Pack * src)//System ID (0 for broadcast)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  1, 1)));
}
INLINER void p110_target_system_SET(uint8_t  src, Pack * dst)//System ID (0 for broadcast)
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  1);
}
INLINER uint8_t p110_target_component_GET(Pack * src)//Component ID (0 for broadcast)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 1)));
}
INLINER void p110_target_component_SET(uint8_t  src, Pack * dst)//Component ID (0 for broadcast)
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  2);
}
/**
*Variable length payload. The length is defined by the remaining message length when subtracting the header
*	and other fields.  The entire content of this block is opaque unless you understand any the encoding
*	message_type.  The particular encoding used can be extension specific and might not always be documented
*	as part of the mavlink specification*/
INLINER uint8_t* p110_payload_GET(Pack * src, uint8_t*  dst, int32_t pos)
{
    uint8_t * data = src->data;
    for(int32_t BYTE = 3, dst_max = pos + 251; pos < dst_max ; pos++,  BYTE += 1)
        dst[pos] = ((get_bytes(data,  BYTE, 1)));
    return dst;
}

static const  uint32_t p110_payload_LEN = 251; //return array length
/**
*Variable length payload. The length is defined by the remaining message length when subtracting the header
*	and other fields.  The entire content of this block is opaque unless you understand any the encoding
*	message_type.  The particular encoding used can be extension specific and might not always be documented
*	as part of the mavlink specification*/

INLINER  uint8_t*  p110_payload_GET_(Pack * src) {return p110_payload_GET(src, malloc(251 * sizeof(uint8_t)), 0);}/**
*Variable length payload. The length is defined by the remaining message length when subtracting the header
*	and other fields.  The entire content of this block is opaque unless you understand any the encoding
*	message_type.  The particular encoding used can be extension specific and might not always be documented
*	as part of the mavlink specification*/
INLINER void p110_payload_SET(uint8_t*  src, int32_t pos, Pack * dst)
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  3, src_max = pos + 251; pos < src_max; pos++, BYTE += 1)
        set_bytes((src[pos]), 1, data,  BYTE);
}
INLINER int64_t p111_tc1_GET(Pack * src)//Time sync timestamp 1
{
    uint8_t * data = src->data;
    return ((int64_t)(get_bytes(data,  0, 8)));
}
INLINER void p111_tc1_SET(int64_t  src, Pack * dst)//Time sync timestamp 1
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  0);
}
INLINER int64_t p111_ts1_GET(Pack * src)//Time sync timestamp 2
{
    uint8_t * data = src->data;
    return ((int64_t)(get_bytes(data,  8, 8)));
}
INLINER void p111_ts1_SET(int64_t  src, Pack * dst)//Time sync timestamp 2
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  8);
}
INLINER uint32_t p112_seq_GET(Pack * src)//Image frame sequence
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 4)));
}
INLINER void p112_seq_SET(uint32_t  src, Pack * dst)//Image frame sequence
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  0);
}
INLINER uint64_t p112_time_usec_GET(Pack * src)//Timestamp for the image frame in microseconds
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  4, 8)));
}
INLINER void p112_time_usec_SET(uint64_t  src, Pack * dst)//Timestamp for the image frame in microseconds
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  4);
}
INLINER uint16_t p113_eph_GET(Pack * src)//GPS HDOP horizontal dilution of position in cm (m*100). If unknown, set to: 65535
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER void p113_eph_SET(uint16_t  src, Pack * dst)//GPS HDOP horizontal dilution of position in cm (m*100). If unknown, set to: 65535
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER uint16_t p113_epv_GET(Pack * src)//GPS VDOP vertical dilution of position in cm (m*100). If unknown, set to: 65535
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 2)));
}
INLINER void p113_epv_SET(uint16_t  src, Pack * dst)//GPS VDOP vertical dilution of position in cm (m*100). If unknown, set to: 65535
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  2);
}
INLINER uint16_t p113_vel_GET(Pack * src)//GPS ground speed in cm/s. If unknown, set to: 65535
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  4, 2)));
}
INLINER void p113_vel_SET(uint16_t  src, Pack * dst)//GPS ground speed in cm/s. If unknown, set to: 65535
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  4);
}
/**
*Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If
*	unknown, set to: 6553*/
INLINER uint16_t p113_cog_GET(Pack * src)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  6, 2)));
}
/**
*Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If
*	unknown, set to: 6553*/
INLINER void p113_cog_SET(uint16_t  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  6);
}
INLINER uint64_t p113_time_usec_GET(Pack * src)//Timestamp (microseconds since UNIX epoch or microseconds since system boot)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  8, 8)));
}
INLINER void p113_time_usec_SET(uint64_t  src, Pack * dst)//Timestamp (microseconds since UNIX epoch or microseconds since system boot)
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  8);
}
/**
*0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value of this field unless it is
*	at least two, so always correctly fill in the fix*/
INLINER uint8_t p113_fix_type_GET(Pack * src)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  16, 1)));
}
/**
*0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value of this field unless it is
*	at least two, so always correctly fill in the fix*/
INLINER void p113_fix_type_SET(uint8_t  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  16);
}
INLINER int32_t p113_lat_GET(Pack * src)//Latitude (WGS84), in degrees * 1E7
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  17, 4)));
}
INLINER void p113_lat_SET(int32_t  src, Pack * dst)//Latitude (WGS84), in degrees * 1E7
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  17);
}
INLINER int32_t p113_lon_GET(Pack * src)//Longitude (WGS84), in degrees * 1E7
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  21, 4)));
}
INLINER void p113_lon_SET(int32_t  src, Pack * dst)//Longitude (WGS84), in degrees * 1E7
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  21);
}
INLINER int32_t p113_alt_GET(Pack * src)//Altitude (AMSL, not WGS84), in meters * 1000 (positive for up)
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  25, 4)));
}
INLINER void p113_alt_SET(int32_t  src, Pack * dst)//Altitude (AMSL, not WGS84), in meters * 1000 (positive for up)
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  25);
}
INLINER int16_t p113_vn_GET(Pack * src)//GPS velocity in cm/s in NORTH direction in earth-fixed NED frame
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  29, 2)));
}
INLINER void p113_vn_SET(int16_t  src, Pack * dst)//GPS velocity in cm/s in NORTH direction in earth-fixed NED frame
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  29);
}
INLINER int16_t p113_ve_GET(Pack * src)//GPS velocity in cm/s in EAST direction in earth-fixed NED frame
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  31, 2)));
}
INLINER void p113_ve_SET(int16_t  src, Pack * dst)//GPS velocity in cm/s in EAST direction in earth-fixed NED frame
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  31);
}
INLINER int16_t p113_vd_GET(Pack * src)//GPS velocity in cm/s in DOWN direction in earth-fixed NED frame
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  33, 2)));
}
INLINER void p113_vd_SET(int16_t  src, Pack * dst)//GPS velocity in cm/s in DOWN direction in earth-fixed NED frame
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  33);
}
INLINER uint8_t p113_satellites_visible_GET(Pack * src)//Number of satellites visible. If unknown, set to 255
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  35, 1)));
}
INLINER void p113_satellites_visible_SET(uint8_t  src, Pack * dst)//Number of satellites visible. If unknown, set to 255
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  35);
}
/**
*Integration time in microseconds. Divide integrated_x and integrated_y by the integration time to obtain
*	average flow. The integration time also indicates the*/
INLINER uint32_t p114_integration_time_us_GET(Pack * src)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 4)));
}
/**
*Integration time in microseconds. Divide integrated_x and integrated_y by the integration time to obtain
*	average flow. The integration time also indicates the*/
INLINER void p114_integration_time_us_SET(uint32_t  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  0);
}
INLINER uint32_t p114_time_delta_distance_us_GET(Pack * src)//Time in microseconds since the distance was sampled.
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  4, 4)));
}
INLINER void p114_time_delta_distance_us_SET(uint32_t  src, Pack * dst)//Time in microseconds since the distance was sampled.
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  4);
}
INLINER uint64_t p114_time_usec_GET(Pack * src)//Timestamp (microseconds, synced to UNIX time or since system boot)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  8, 8)));
}
INLINER void p114_time_usec_SET(uint64_t  src, Pack * dst)//Timestamp (microseconds, synced to UNIX time or since system boot)
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  8);
}
INLINER uint8_t p114_sensor_id_GET(Pack * src)//Sensor ID
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  16, 1)));
}
INLINER void p114_sensor_id_SET(uint8_t  src, Pack * dst)//Sensor ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  16);
}
/**
*Flow in radians around X axis (Sensor RH rotation about the X axis induces a positive flow. Sensor linear
*	motion along the positive Y axis induces a negative flow.*/
INLINER float p114_integrated_x_GET(Pack * src)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  17, 4)));
}
/**
*Flow in radians around X axis (Sensor RH rotation about the X axis induces a positive flow. Sensor linear
*	motion along the positive Y axis induces a negative flow.*/
INLINER void p114_integrated_x_SET(float  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  17);
}
/**
*Flow in radians around Y axis (Sensor RH rotation about the Y axis induces a positive flow. Sensor linear
*	motion along the positive X axis induces a positive flow.*/
INLINER float p114_integrated_y_GET(Pack * src)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  21, 4)));
}
/**
*Flow in radians around Y axis (Sensor RH rotation about the Y axis induces a positive flow. Sensor linear
*	motion along the positive X axis induces a positive flow.*/
INLINER void p114_integrated_y_SET(float  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  21);
}
INLINER float p114_integrated_xgyro_GET(Pack * src)//RH rotation around X axis (rad)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  25, 4)));
}
INLINER void p114_integrated_xgyro_SET(float  src, Pack * dst)//RH rotation around X axis (rad)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  25);
}
INLINER float p114_integrated_ygyro_GET(Pack * src)//RH rotation around Y axis (rad)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  29, 4)));
}
INLINER void p114_integrated_ygyro_SET(float  src, Pack * dst)//RH rotation around Y axis (rad)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  29);
}
INLINER float p114_integrated_zgyro_GET(Pack * src)//RH rotation around Z axis (rad)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  33, 4)));
}
INLINER void p114_integrated_zgyro_SET(float  src, Pack * dst)//RH rotation around Z axis (rad)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  33);
}
INLINER int16_t p114_temperature_GET(Pack * src)//Temperature * 100 in centi-degrees Celsius
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  37, 2)));
}
INLINER void p114_temperature_SET(int16_t  src, Pack * dst)//Temperature * 100 in centi-degrees Celsius
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  37);
}
INLINER uint8_t p114_quality_GET(Pack * src)//Optical flow quality / confidence. 0: no valid flow, 255: maximum quality
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  39, 1)));
}
INLINER void p114_quality_SET(uint8_t  src, Pack * dst)//Optical flow quality / confidence. 0: no valid flow, 255: maximum quality
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  39);
}
/**
*Distance to the center of the flow field in meters. Positive value (including zero): distance known. Negative
*	value: Unknown distance*/
INLINER float p114_distance_GET(Pack * src)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  40, 4)));
}
/**
*Distance to the center of the flow field in meters. Positive value (including zero): distance known. Negative
*	value: Unknown distance*/
INLINER void p114_distance_SET(float  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  40);
}
INLINER uint16_t p115_ind_airspeed_GET(Pack * src)//Indicated airspeed, expressed as cm/s
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER void p115_ind_airspeed_SET(uint16_t  src, Pack * dst)//Indicated airspeed, expressed as cm/s
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER uint16_t p115_true_airspeed_GET(Pack * src)//True airspeed, expressed as cm/s
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 2)));
}
INLINER void p115_true_airspeed_SET(uint16_t  src, Pack * dst)//True airspeed, expressed as cm/s
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  2);
}
INLINER uint64_t p115_time_usec_GET(Pack * src)//Timestamp (microseconds since UNIX epoch or microseconds since system boot)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  4, 8)));
}
INLINER void p115_time_usec_SET(uint64_t  src, Pack * dst)//Timestamp (microseconds since UNIX epoch or microseconds since system boot)
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  4);
}
INLINER float* p115_attitude_quaternion_GET(Pack * src, float*  dst, int32_t pos) //Vehicle attitude expressed as normalized quaternion in w, x, y, z order (with 1 0 0 0 being the null-rotation
{
    uint8_t * data = src->data;
    for(int32_t BYTE = 12, dst_max = pos + 4; pos < dst_max ; pos++,  BYTE += 4)
        dst[pos] = (intBitsToFloat(get_bytes(data,  BYTE, 4)));
    return dst;
}

static const  uint32_t p115_attitude_quaternion_LEN = 4; //return array length

INLINER  float*  p115_attitude_quaternion_GET_(Pack * src) {return p115_attitude_quaternion_GET(src, malloc(4 * sizeof(float)), 0);}
INLINER void p115_attitude_quaternion_SET(float*  src, int32_t pos, Pack * dst) //Vehicle attitude expressed as normalized quaternion in w, x, y, z order (with 1 0 0 0 being the null-rotation
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  12, src_max = pos + 4; pos < src_max; pos++, BYTE += 4)
        set_bytes(floatToIntBits(src[pos]), 4, data,  BYTE);
}
INLINER float p115_rollspeed_GET(Pack * src)//Body frame roll / phi angular speed (rad/s)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  28, 4)));
}
INLINER void p115_rollspeed_SET(float  src, Pack * dst)//Body frame roll / phi angular speed (rad/s)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  28);
}
INLINER float p115_pitchspeed_GET(Pack * src)//Body frame pitch / theta angular speed (rad/s)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  32, 4)));
}
INLINER void p115_pitchspeed_SET(float  src, Pack * dst)//Body frame pitch / theta angular speed (rad/s)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  32);
}
INLINER float p115_yawspeed_GET(Pack * src)//Body frame yaw / psi angular speed (rad/s)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  36, 4)));
}
INLINER void p115_yawspeed_SET(float  src, Pack * dst)//Body frame yaw / psi angular speed (rad/s)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  36);
}
INLINER int32_t p115_lat_GET(Pack * src)//Latitude, expressed as * 1E7
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  40, 4)));
}
INLINER void p115_lat_SET(int32_t  src, Pack * dst)//Latitude, expressed as * 1E7
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  40);
}
INLINER int32_t p115_lon_GET(Pack * src)//Longitude, expressed as * 1E7
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  44, 4)));
}
INLINER void p115_lon_SET(int32_t  src, Pack * dst)//Longitude, expressed as * 1E7
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  44);
}
INLINER int32_t p115_alt_GET(Pack * src)//Altitude in meters, expressed as * 1000 (millimeters)
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  48, 4)));
}
INLINER void p115_alt_SET(int32_t  src, Pack * dst)//Altitude in meters, expressed as * 1000 (millimeters)
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  48);
}
INLINER int16_t p115_vx_GET(Pack * src)//Ground X Speed (Latitude), expressed as cm/s
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  52, 2)));
}
INLINER void p115_vx_SET(int16_t  src, Pack * dst)//Ground X Speed (Latitude), expressed as cm/s
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  52);
}
INLINER int16_t p115_vy_GET(Pack * src)//Ground Y Speed (Longitude), expressed as cm/s
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  54, 2)));
}
INLINER void p115_vy_SET(int16_t  src, Pack * dst)//Ground Y Speed (Longitude), expressed as cm/s
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  54);
}
INLINER int16_t p115_vz_GET(Pack * src)//Ground Z Speed (Altitude), expressed as cm/s
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  56, 2)));
}
INLINER void p115_vz_SET(int16_t  src, Pack * dst)//Ground Z Speed (Altitude), expressed as cm/s
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  56);
}
INLINER int16_t p115_xacc_GET(Pack * src)//X acceleration (mg)
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  58, 2)));
}
INLINER void p115_xacc_SET(int16_t  src, Pack * dst)//X acceleration (mg)
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  58);
}
INLINER int16_t p115_yacc_GET(Pack * src)//Y acceleration (mg)
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  60, 2)));
}
INLINER void p115_yacc_SET(int16_t  src, Pack * dst)//Y acceleration (mg)
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  60);
}
INLINER int16_t p115_zacc_GET(Pack * src)//Z acceleration (mg)
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  62, 2)));
}
INLINER void p115_zacc_SET(int16_t  src, Pack * dst)//Z acceleration (mg)
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  62);
}
INLINER uint32_t p116_time_boot_ms_GET(Pack * src)//Timestamp (milliseconds since system boot)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 4)));
}
INLINER void p116_time_boot_ms_SET(uint32_t  src, Pack * dst)//Timestamp (milliseconds since system boot)
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  0);
}
INLINER int16_t p116_xacc_GET(Pack * src)//X acceleration (mg)
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  4, 2)));
}
INLINER void p116_xacc_SET(int16_t  src, Pack * dst)//X acceleration (mg)
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  4);
}
INLINER int16_t p116_yacc_GET(Pack * src)//Y acceleration (mg)
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  6, 2)));
}
INLINER void p116_yacc_SET(int16_t  src, Pack * dst)//Y acceleration (mg)
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  6);
}
INLINER int16_t p116_zacc_GET(Pack * src)//Z acceleration (mg)
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  8, 2)));
}
INLINER void p116_zacc_SET(int16_t  src, Pack * dst)//Z acceleration (mg)
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  8);
}
INLINER int16_t p116_xgyro_GET(Pack * src)//Angular speed around X axis (millirad /sec)
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  10, 2)));
}
INLINER void p116_xgyro_SET(int16_t  src, Pack * dst)//Angular speed around X axis (millirad /sec)
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  10);
}
INLINER int16_t p116_ygyro_GET(Pack * src)//Angular speed around Y axis (millirad /sec)
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  12, 2)));
}
INLINER void p116_ygyro_SET(int16_t  src, Pack * dst)//Angular speed around Y axis (millirad /sec)
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  12);
}
INLINER int16_t p116_zgyro_GET(Pack * src)//Angular speed around Z axis (millirad /sec)
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  14, 2)));
}
INLINER void p116_zgyro_SET(int16_t  src, Pack * dst)//Angular speed around Z axis (millirad /sec)
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  14);
}
INLINER int16_t p116_xmag_GET(Pack * src)//X Magnetic field (milli tesla)
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  16, 2)));
}
INLINER void p116_xmag_SET(int16_t  src, Pack * dst)//X Magnetic field (milli tesla)
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  16);
}
INLINER int16_t p116_ymag_GET(Pack * src)//Y Magnetic field (milli tesla)
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  18, 2)));
}
INLINER void p116_ymag_SET(int16_t  src, Pack * dst)//Y Magnetic field (milli tesla)
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  18);
}
INLINER int16_t p116_zmag_GET(Pack * src)//Z Magnetic field (milli tesla)
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  20, 2)));
}
INLINER void p116_zmag_SET(int16_t  src, Pack * dst)//Z Magnetic field (milli tesla)
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  20);
}
INLINER uint16_t p117_start_GET(Pack * src)//First log id (0 for first available)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER void p117_start_SET(uint16_t  src, Pack * dst)//First log id (0 for first available)
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER uint16_t p117_end_GET(Pack * src)//Last log id (0xffff for last available)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 2)));
}
INLINER void p117_end_SET(uint16_t  src, Pack * dst)//Last log id (0xffff for last available)
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  2);
}
INLINER uint8_t p117_target_system_GET(Pack * src)//System ID
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  4, 1)));
}
INLINER void p117_target_system_SET(uint8_t  src, Pack * dst)//System ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  4);
}
INLINER uint8_t p117_target_component_GET(Pack * src)//Component ID
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  5, 1)));
}
INLINER void p117_target_component_SET(uint8_t  src, Pack * dst)//Component ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  5);
}
INLINER uint16_t p118_id_GET(Pack * src)//Log id
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER void p118_id_SET(uint16_t  src, Pack * dst)//Log id
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER uint16_t p118_num_logs_GET(Pack * src)//Total number of logs
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 2)));
}
INLINER void p118_num_logs_SET(uint16_t  src, Pack * dst)//Total number of logs
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  2);
}
INLINER uint16_t p118_last_log_num_GET(Pack * src)//High log number
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  4, 2)));
}
INLINER void p118_last_log_num_SET(uint16_t  src, Pack * dst)//High log number
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  4);
}
INLINER uint32_t p118_time_utc_GET(Pack * src)//UTC timestamp of log in seconds since 1970, or 0 if not available
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  6, 4)));
}
INLINER void p118_time_utc_SET(uint32_t  src, Pack * dst)//UTC timestamp of log in seconds since 1970, or 0 if not available
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  6);
}
INLINER uint32_t p118_size_GET(Pack * src)//Size of the log (may be approximate) in bytes
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  10, 4)));
}
INLINER void p118_size_SET(uint32_t  src, Pack * dst)//Size of the log (may be approximate) in bytes
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  10);
}
INLINER uint16_t p119_id_GET(Pack * src)//Log id (from LOG_ENTRY reply)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER void p119_id_SET(uint16_t  src, Pack * dst)//Log id (from LOG_ENTRY reply)
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER uint32_t p119_ofs_GET(Pack * src)//Offset into the log
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 4)));
}
INLINER void p119_ofs_SET(uint32_t  src, Pack * dst)//Offset into the log
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  2);
}
INLINER uint32_t p119_count_GET(Pack * src)//Number of bytes
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  6, 4)));
}
INLINER void p119_count_SET(uint32_t  src, Pack * dst)//Number of bytes
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  6);
}
INLINER uint8_t p119_target_system_GET(Pack * src)//System ID
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  10, 1)));
}
INLINER void p119_target_system_SET(uint8_t  src, Pack * dst)//System ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  10);
}
INLINER uint8_t p119_target_component_GET(Pack * src)//Component ID
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  11, 1)));
}
INLINER void p119_target_component_SET(uint8_t  src, Pack * dst)//Component ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  11);
}
INLINER uint16_t p120_id_GET(Pack * src)//Log id (from LOG_ENTRY reply)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER void p120_id_SET(uint16_t  src, Pack * dst)//Log id (from LOG_ENTRY reply)
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER uint32_t p120_ofs_GET(Pack * src)//Offset into the log
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 4)));
}
INLINER void p120_ofs_SET(uint32_t  src, Pack * dst)//Offset into the log
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  2);
}
INLINER uint8_t p120_count_GET(Pack * src)//Number of bytes (zero for end of log)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  6, 1)));
}
INLINER void p120_count_SET(uint8_t  src, Pack * dst)//Number of bytes (zero for end of log)
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  6);
}
INLINER uint8_t* p120_data__GET(Pack * src, uint8_t*  dst, int32_t pos) //log data
{
    uint8_t * data = src->data;
    for(int32_t BYTE = 7, dst_max = pos + 90; pos < dst_max ; pos++,  BYTE += 1)
        dst[pos] = ((get_bytes(data,  BYTE, 1)));
    return dst;
}

static const  uint32_t p120_data__LEN = 90; //return array length

INLINER  uint8_t*  p120_data__GET_(Pack * src) {return p120_data__GET(src, malloc(90 * sizeof(uint8_t)), 0);}
INLINER void p120_data__SET(uint8_t*  src, int32_t pos, Pack * dst) //log data
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  7, src_max = pos + 90; pos < src_max; pos++, BYTE += 1)
        set_bytes((src[pos]), 1, data,  BYTE);
}
INLINER uint8_t p121_target_system_GET(Pack * src)//System ID
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 1)));
}
INLINER void p121_target_system_SET(uint8_t  src, Pack * dst)//System ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  0);
}
INLINER uint8_t p121_target_component_GET(Pack * src)//Component ID
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  1, 1)));
}
INLINER void p121_target_component_SET(uint8_t  src, Pack * dst)//Component ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  1);
}
INLINER uint8_t p122_target_system_GET(Pack * src)//System ID
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 1)));
}
INLINER void p122_target_system_SET(uint8_t  src, Pack * dst)//System ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  0);
}
INLINER uint8_t p122_target_component_GET(Pack * src)//Component ID
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  1, 1)));
}
INLINER void p122_target_component_SET(uint8_t  src, Pack * dst)//Component ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  1);
}
INLINER uint8_t p123_target_system_GET(Pack * src)//System ID
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 1)));
}
INLINER void p123_target_system_SET(uint8_t  src, Pack * dst)//System ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  0);
}
INLINER uint8_t p123_target_component_GET(Pack * src)//Component ID
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  1, 1)));
}
INLINER void p123_target_component_SET(uint8_t  src, Pack * dst)//Component ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  1);
}
INLINER uint8_t p123_len_GET(Pack * src)//data length
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 1)));
}
INLINER void p123_len_SET(uint8_t  src, Pack * dst)//data length
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  2);
}
INLINER uint8_t* p123_data__GET(Pack * src, uint8_t*  dst, int32_t pos) //raw data (110 is enough for 12 satellites of RTCMv2)
{
    uint8_t * data = src->data;
    for(int32_t BYTE = 3, dst_max = pos + 110; pos < dst_max ; pos++,  BYTE += 1)
        dst[pos] = ((get_bytes(data,  BYTE, 1)));
    return dst;
}

static const  uint32_t p123_data__LEN = 110; //return array length

INLINER  uint8_t*  p123_data__GET_(Pack * src) {return p123_data__GET(src, malloc(110 * sizeof(uint8_t)), 0);}
INLINER void p123_data__SET(uint8_t*  src, int32_t pos, Pack * dst) //raw data (110 is enough for 12 satellites of RTCMv2)
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  3, src_max = pos + 110; pos < src_max; pos++, BYTE += 1)
        set_bytes((src[pos]), 1, data,  BYTE);
}
INLINER uint16_t p124_eph_GET(Pack * src)//GPS HDOP horizontal dilution of position in cm (m*100). If unknown, set to: UINT16_MAX
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER void p124_eph_SET(uint16_t  src, Pack * dst)//GPS HDOP horizontal dilution of position in cm (m*100). If unknown, set to: UINT16_MAX
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER uint16_t p124_epv_GET(Pack * src)//GPS VDOP vertical dilution of position in cm (m*100). If unknown, set to: UINT16_MAX
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 2)));
}
INLINER void p124_epv_SET(uint16_t  src, Pack * dst)//GPS VDOP vertical dilution of position in cm (m*100). If unknown, set to: UINT16_MAX
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  2);
}
INLINER uint16_t p124_vel_GET(Pack * src)//GPS ground speed (m/s * 100). If unknown, set to: UINT16_MAX
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  4, 2)));
}
INLINER void p124_vel_SET(uint16_t  src, Pack * dst)//GPS ground speed (m/s * 100). If unknown, set to: UINT16_MAX
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  4);
}
/**
*Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If
*	unknown, set to: UINT16_MA*/
INLINER uint16_t p124_cog_GET(Pack * src)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  6, 2)));
}
/**
*Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If
*	unknown, set to: UINT16_MA*/
INLINER void p124_cog_SET(uint16_t  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  6);
}
INLINER uint32_t p124_dgps_age_GET(Pack * src)//Age of DGPS info
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  8, 4)));
}
INLINER void p124_dgps_age_SET(uint32_t  src, Pack * dst)//Age of DGPS info
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  8);
}
INLINER uint64_t p124_time_usec_GET(Pack * src)//Timestamp (microseconds since UNIX epoch or microseconds since system boot)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  12, 8)));
}
INLINER void p124_time_usec_SET(uint64_t  src, Pack * dst)//Timestamp (microseconds since UNIX epoch or microseconds since system boot)
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  12);
}
INLINER int32_t p124_lat_GET(Pack * src)//Latitude (WGS84), in degrees * 1E7
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  20, 4)));
}
INLINER void p124_lat_SET(int32_t  src, Pack * dst)//Latitude (WGS84), in degrees * 1E7
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  20);
}
INLINER int32_t p124_lon_GET(Pack * src)//Longitude (WGS84), in degrees * 1E7
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  24, 4)));
}
INLINER void p124_lon_SET(int32_t  src, Pack * dst)//Longitude (WGS84), in degrees * 1E7
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  24);
}
INLINER int32_t p124_alt_GET(Pack * src)//Altitude (AMSL, not WGS84), in meters * 1000 (positive for up)
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  28, 4)));
}
INLINER void p124_alt_SET(int32_t  src, Pack * dst)//Altitude (AMSL, not WGS84), in meters * 1000 (positive for up)
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  28);
}
INLINER uint8_t p124_satellites_visible_GET(Pack * src)//Number of satellites visible. If unknown, set to 255
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  32, 1)));
}
INLINER void p124_satellites_visible_SET(uint8_t  src, Pack * dst)//Number of satellites visible. If unknown, set to 255
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  32);
}
INLINER uint8_t p124_dgps_numch_GET(Pack * src)//Number of DGPS satellites
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  33, 1)));
}
INLINER void p124_dgps_numch_SET(uint8_t  src, Pack * dst)//Number of DGPS satellites
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  33);
}
INLINER e_GPS_FIX_TYPE p124_fix_type_GET(Pack * src)//See the GPS_FIX_TYPE enum.
{
    uint8_t * data = src->data;
    return  0 + (int)get_bits(data, 272, 4);
}
INLINER void p124_fix_type_SET(e_GPS_FIX_TYPE  src, Pack * dst)//See the GPS_FIX_TYPE enum.
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 4, data, 272);
}
INLINER uint16_t p125_Vcc_GET(Pack * src)//5V rail voltage in millivolts
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER void p125_Vcc_SET(uint16_t  src, Pack * dst)//5V rail voltage in millivolts
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER uint16_t p125_Vservo_GET(Pack * src)//servo rail voltage in millivolts
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 2)));
}
INLINER void p125_Vservo_SET(uint16_t  src, Pack * dst)//servo rail voltage in millivolts
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  2);
}
INLINER e_MAV_POWER_STATUS p125_flags_GET(Pack * src)//power supply status flags (see MAV_POWER_STATUS enum)
{
    uint8_t * data = src->data;
    return  1 + (int)get_bits(data, 32, 6);
}
INLINER void p125_flags_SET(e_MAV_POWER_STATUS  src, Pack * dst)//power supply status flags (see MAV_POWER_STATUS enum)
{
    uint8_t * data = dst->data;
    set_bits(- 1 +   src, 6, data, 32);
}
INLINER uint16_t p126_timeout_GET(Pack * src)//Timeout for reply data in milliseconds
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER void p126_timeout_SET(uint16_t  src, Pack * dst)//Timeout for reply data in milliseconds
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER uint32_t p126_baudrate_GET(Pack * src)//Baudrate of transfer. Zero means no change.
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 4)));
}
INLINER void p126_baudrate_SET(uint32_t  src, Pack * dst)//Baudrate of transfer. Zero means no change.
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  2);
}
INLINER uint8_t p126_count_GET(Pack * src)//how many bytes in this transfer
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  6, 1)));
}
INLINER void p126_count_SET(uint8_t  src, Pack * dst)//how many bytes in this transfer
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  6);
}
INLINER uint8_t* p126_data__GET(Pack * src, uint8_t*  dst, int32_t pos) //serial data
{
    uint8_t * data = src->data;
    for(int32_t BYTE = 7, dst_max = pos + 70; pos < dst_max ; pos++,  BYTE += 1)
        dst[pos] = ((get_bytes(data,  BYTE, 1)));
    return dst;
}

static const  uint32_t p126_data__LEN = 70; //return array length

INLINER  uint8_t*  p126_data__GET_(Pack * src) {return p126_data__GET(src, malloc(70 * sizeof(uint8_t)), 0);}
INLINER void p126_data__SET(uint8_t*  src, int32_t pos, Pack * dst) //serial data
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  7, src_max = pos + 70; pos < src_max; pos++, BYTE += 1)
        set_bytes((src[pos]), 1, data,  BYTE);
}
INLINER e_SERIAL_CONTROL_DEV p126_device_GET(Pack * src)//See SERIAL_CONTROL_DEV enum
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
INLINER void p126_device_SET(e_SERIAL_CONTROL_DEV  src, Pack * dst)//See SERIAL_CONTROL_DEV enum
{
    uint8_t * data = dst->data;
    UMAX id;
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
INLINER e_SERIAL_CONTROL_FLAG p126_flags_GET(Pack * src)//See SERIAL_CONTROL_FLAG enum
{
    uint8_t * data = src->data;
    return  1 + (int)get_bits(data, 619, 5);
}
INLINER void p126_flags_SET(e_SERIAL_CONTROL_FLAG  src, Pack * dst)//See SERIAL_CONTROL_FLAG enum
{
    uint8_t * data = dst->data;
    set_bits(- 1 +   src, 5, data, 619);
}
INLINER uint16_t p127_wn_GET(Pack * src)//GPS Week Number of last baseline
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER void p127_wn_SET(uint16_t  src, Pack * dst)//GPS Week Number of last baseline
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER uint32_t p127_time_last_baseline_ms_GET(Pack * src)//Time since boot of last baseline message received in ms.
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 4)));
}
INLINER void p127_time_last_baseline_ms_SET(uint32_t  src, Pack * dst)//Time since boot of last baseline message received in ms.
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  2);
}
INLINER uint32_t p127_tow_GET(Pack * src)//GPS Time of Week of last baseline
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  6, 4)));
}
INLINER void p127_tow_SET(uint32_t  src, Pack * dst)//GPS Time of Week of last baseline
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  6);
}
INLINER uint32_t p127_accuracy_GET(Pack * src)//Current estimate of baseline accuracy.
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  10, 4)));
}
INLINER void p127_accuracy_SET(uint32_t  src, Pack * dst)//Current estimate of baseline accuracy.
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  10);
}
INLINER uint8_t p127_rtk_receiver_id_GET(Pack * src)//Identification of connected RTK receiver.
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  14, 1)));
}
INLINER void p127_rtk_receiver_id_SET(uint8_t  src, Pack * dst)//Identification of connected RTK receiver.
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  14);
}
INLINER uint8_t p127_rtk_health_GET(Pack * src)//GPS-specific health report for RTK data.
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  15, 1)));
}
INLINER void p127_rtk_health_SET(uint8_t  src, Pack * dst)//GPS-specific health report for RTK data.
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  15);
}
INLINER uint8_t p127_rtk_rate_GET(Pack * src)//Rate of baseline messages being received by GPS, in HZ
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  16, 1)));
}
INLINER void p127_rtk_rate_SET(uint8_t  src, Pack * dst)//Rate of baseline messages being received by GPS, in HZ
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  16);
}
INLINER uint8_t p127_nsats_GET(Pack * src)//Current number of sats used for RTK calculation.
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  17, 1)));
}
INLINER void p127_nsats_SET(uint8_t  src, Pack * dst)//Current number of sats used for RTK calculation.
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  17);
}
INLINER uint8_t p127_baseline_coords_type_GET(Pack * src)//Coordinate system of baseline. 0 == ECEF, 1 == NED
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  18, 1)));
}
INLINER void p127_baseline_coords_type_SET(uint8_t  src, Pack * dst)//Coordinate system of baseline. 0 == ECEF, 1 == NED
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  18);
}
INLINER int32_t p127_baseline_a_mm_GET(Pack * src)//Current baseline in ECEF x or NED north component in mm.
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  19, 4)));
}
INLINER void p127_baseline_a_mm_SET(int32_t  src, Pack * dst)//Current baseline in ECEF x or NED north component in mm.
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  19);
}
INLINER int32_t p127_baseline_b_mm_GET(Pack * src)//Current baseline in ECEF y or NED east component in mm.
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  23, 4)));
}
INLINER void p127_baseline_b_mm_SET(int32_t  src, Pack * dst)//Current baseline in ECEF y or NED east component in mm.
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  23);
}
INLINER int32_t p127_baseline_c_mm_GET(Pack * src)//Current baseline in ECEF z or NED down component in mm.
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  27, 4)));
}
INLINER void p127_baseline_c_mm_SET(int32_t  src, Pack * dst)//Current baseline in ECEF z or NED down component in mm.
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  27);
}
INLINER int32_t p127_iar_num_hypotheses_GET(Pack * src)//Current number of integer ambiguity hypotheses.
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  31, 4)));
}
INLINER void p127_iar_num_hypotheses_SET(int32_t  src, Pack * dst)//Current number of integer ambiguity hypotheses.
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  31);
}
INLINER uint16_t p128_wn_GET(Pack * src)//GPS Week Number of last baseline
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER void p128_wn_SET(uint16_t  src, Pack * dst)//GPS Week Number of last baseline
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER uint32_t p128_time_last_baseline_ms_GET(Pack * src)//Time since boot of last baseline message received in ms.
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 4)));
}
INLINER void p128_time_last_baseline_ms_SET(uint32_t  src, Pack * dst)//Time since boot of last baseline message received in ms.
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  2);
}
INLINER uint32_t p128_tow_GET(Pack * src)//GPS Time of Week of last baseline
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  6, 4)));
}
INLINER void p128_tow_SET(uint32_t  src, Pack * dst)//GPS Time of Week of last baseline
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  6);
}
INLINER uint32_t p128_accuracy_GET(Pack * src)//Current estimate of baseline accuracy.
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  10, 4)));
}
INLINER void p128_accuracy_SET(uint32_t  src, Pack * dst)//Current estimate of baseline accuracy.
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  10);
}
INLINER uint8_t p128_rtk_receiver_id_GET(Pack * src)//Identification of connected RTK receiver.
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  14, 1)));
}
INLINER void p128_rtk_receiver_id_SET(uint8_t  src, Pack * dst)//Identification of connected RTK receiver.
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  14);
}
INLINER uint8_t p128_rtk_health_GET(Pack * src)//GPS-specific health report for RTK data.
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  15, 1)));
}
INLINER void p128_rtk_health_SET(uint8_t  src, Pack * dst)//GPS-specific health report for RTK data.
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  15);
}
INLINER uint8_t p128_rtk_rate_GET(Pack * src)//Rate of baseline messages being received by GPS, in HZ
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  16, 1)));
}
INLINER void p128_rtk_rate_SET(uint8_t  src, Pack * dst)//Rate of baseline messages being received by GPS, in HZ
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  16);
}
INLINER uint8_t p128_nsats_GET(Pack * src)//Current number of sats used for RTK calculation.
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  17, 1)));
}
INLINER void p128_nsats_SET(uint8_t  src, Pack * dst)//Current number of sats used for RTK calculation.
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  17);
}
INLINER uint8_t p128_baseline_coords_type_GET(Pack * src)//Coordinate system of baseline. 0 == ECEF, 1 == NED
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  18, 1)));
}
INLINER void p128_baseline_coords_type_SET(uint8_t  src, Pack * dst)//Coordinate system of baseline. 0 == ECEF, 1 == NED
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  18);
}
INLINER int32_t p128_baseline_a_mm_GET(Pack * src)//Current baseline in ECEF x or NED north component in mm.
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  19, 4)));
}
INLINER void p128_baseline_a_mm_SET(int32_t  src, Pack * dst)//Current baseline in ECEF x or NED north component in mm.
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  19);
}
INLINER int32_t p128_baseline_b_mm_GET(Pack * src)//Current baseline in ECEF y or NED east component in mm.
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  23, 4)));
}
INLINER void p128_baseline_b_mm_SET(int32_t  src, Pack * dst)//Current baseline in ECEF y or NED east component in mm.
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  23);
}
INLINER int32_t p128_baseline_c_mm_GET(Pack * src)//Current baseline in ECEF z or NED down component in mm.
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  27, 4)));
}
INLINER void p128_baseline_c_mm_SET(int32_t  src, Pack * dst)//Current baseline in ECEF z or NED down component in mm.
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  27);
}
INLINER int32_t p128_iar_num_hypotheses_GET(Pack * src)//Current number of integer ambiguity hypotheses.
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  31, 4)));
}
INLINER void p128_iar_num_hypotheses_SET(int32_t  src, Pack * dst)//Current number of integer ambiguity hypotheses.
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  31);
}
INLINER uint32_t p129_time_boot_ms_GET(Pack * src)//Timestamp (milliseconds since system boot)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 4)));
}
INLINER void p129_time_boot_ms_SET(uint32_t  src, Pack * dst)//Timestamp (milliseconds since system boot)
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  0);
}
INLINER int16_t p129_xacc_GET(Pack * src)//X acceleration (mg)
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  4, 2)));
}
INLINER void p129_xacc_SET(int16_t  src, Pack * dst)//X acceleration (mg)
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  4);
}
INLINER int16_t p129_yacc_GET(Pack * src)//Y acceleration (mg)
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  6, 2)));
}
INLINER void p129_yacc_SET(int16_t  src, Pack * dst)//Y acceleration (mg)
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  6);
}
INLINER int16_t p129_zacc_GET(Pack * src)//Z acceleration (mg)
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  8, 2)));
}
INLINER void p129_zacc_SET(int16_t  src, Pack * dst)//Z acceleration (mg)
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  8);
}
INLINER int16_t p129_xgyro_GET(Pack * src)//Angular speed around X axis (millirad /sec)
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  10, 2)));
}
INLINER void p129_xgyro_SET(int16_t  src, Pack * dst)//Angular speed around X axis (millirad /sec)
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  10);
}
INLINER int16_t p129_ygyro_GET(Pack * src)//Angular speed around Y axis (millirad /sec)
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  12, 2)));
}
INLINER void p129_ygyro_SET(int16_t  src, Pack * dst)//Angular speed around Y axis (millirad /sec)
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  12);
}
INLINER int16_t p129_zgyro_GET(Pack * src)//Angular speed around Z axis (millirad /sec)
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  14, 2)));
}
INLINER void p129_zgyro_SET(int16_t  src, Pack * dst)//Angular speed around Z axis (millirad /sec)
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  14);
}
INLINER int16_t p129_xmag_GET(Pack * src)//X Magnetic field (milli tesla)
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  16, 2)));
}
INLINER void p129_xmag_SET(int16_t  src, Pack * dst)//X Magnetic field (milli tesla)
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  16);
}
INLINER int16_t p129_ymag_GET(Pack * src)//Y Magnetic field (milli tesla)
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  18, 2)));
}
INLINER void p129_ymag_SET(int16_t  src, Pack * dst)//Y Magnetic field (milli tesla)
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  18);
}
INLINER int16_t p129_zmag_GET(Pack * src)//Z Magnetic field (milli tesla)
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  20, 2)));
}
INLINER void p129_zmag_SET(int16_t  src, Pack * dst)//Z Magnetic field (milli tesla)
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  20);
}
INLINER uint16_t p130_width_GET(Pack * src)//Width of a matrix or image
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER void p130_width_SET(uint16_t  src, Pack * dst)//Width of a matrix or image
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER uint16_t p130_height_GET(Pack * src)//Height of a matrix or image
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 2)));
}
INLINER void p130_height_SET(uint16_t  src, Pack * dst)//Height of a matrix or image
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  2);
}
INLINER uint16_t p130_packets_GET(Pack * src)//number of packets beeing sent (set on ACK only)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  4, 2)));
}
INLINER void p130_packets_SET(uint16_t  src, Pack * dst)//number of packets beeing sent (set on ACK only)
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  4);
}
INLINER uint32_t p130_size_GET(Pack * src)//total data size in bytes (set on ACK only)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  6, 4)));
}
INLINER void p130_size_SET(uint32_t  src, Pack * dst)//total data size in bytes (set on ACK only)
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  6);
}
INLINER uint8_t p130_type_GET(Pack * src)//type of requested/acknowledged data (as defined in ENUM DATA_TYPES in mavlink/include/mavlink_types.h
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  10, 1)));
}
INLINER void p130_type_SET(uint8_t  src, Pack * dst)//type of requested/acknowledged data (as defined in ENUM DATA_TYPES in mavlink/include/mavlink_types.h
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  10);
}
/**
*payload size per packet (normally 253 byte, see DATA field size in message ENCAPSULATED_DATA) (set on
*	ACK only*/
INLINER uint8_t p130_payload_GET(Pack * src)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  11, 1)));
}
/**
*payload size per packet (normally 253 byte, see DATA field size in message ENCAPSULATED_DATA) (set on
*	ACK only*/
INLINER void p130_payload_SET(uint8_t  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  11);
}
INLINER uint8_t p130_jpg_quality_GET(Pack * src)//JPEG quality out of [1,100]
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  12, 1)));
}
INLINER void p130_jpg_quality_SET(uint8_t  src, Pack * dst)//JPEG quality out of [1,100]
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  12);
}
INLINER uint16_t p131_seqnr_GET(Pack * src)//sequence number (starting with 0 on every transmission)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER void p131_seqnr_SET(uint16_t  src, Pack * dst)//sequence number (starting with 0 on every transmission)
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER uint8_t* p131_data__GET(Pack * src, uint8_t*  dst, int32_t pos) //image data bytes
{
    uint8_t * data = src->data;
    for(int32_t BYTE = 2, dst_max = pos + 253; pos < dst_max ; pos++,  BYTE += 1)
        dst[pos] = ((get_bytes(data,  BYTE, 1)));
    return dst;
}

static const  uint32_t p131_data__LEN = 253; //return array length

INLINER  uint8_t*  p131_data__GET_(Pack * src) {return p131_data__GET(src, malloc(253 * sizeof(uint8_t)), 0);}
INLINER void p131_data__SET(uint8_t*  src, int32_t pos, Pack * dst) //image data bytes
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  2, src_max = pos + 253; pos < src_max; pos++, BYTE += 1)
        set_bytes((src[pos]), 1, data,  BYTE);
}
INLINER uint16_t p132_min_distance_GET(Pack * src)//Minimum distance the sensor can measure in centimeters
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER void p132_min_distance_SET(uint16_t  src, Pack * dst)//Minimum distance the sensor can measure in centimeters
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER uint16_t p132_max_distance_GET(Pack * src)//Maximum distance the sensor can measure in centimeters
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 2)));
}
INLINER void p132_max_distance_SET(uint16_t  src, Pack * dst)//Maximum distance the sensor can measure in centimeters
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  2);
}
INLINER uint16_t p132_current_distance_GET(Pack * src)//Current distance reading
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  4, 2)));
}
INLINER void p132_current_distance_SET(uint16_t  src, Pack * dst)//Current distance reading
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  4);
}
INLINER uint32_t p132_time_boot_ms_GET(Pack * src)//Time since system boot
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  6, 4)));
}
INLINER void p132_time_boot_ms_SET(uint32_t  src, Pack * dst)//Time since system boot
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  6);
}
INLINER uint8_t p132_id_GET(Pack * src)//Onboard ID of the sensor
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  10, 1)));
}
INLINER void p132_id_SET(uint8_t  src, Pack * dst)//Onboard ID of the sensor
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  10);
}
INLINER uint8_t p132_covariance_GET(Pack * src)//Measurement covariance in centimeters, 0 for unknown / invalid readings
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  11, 1)));
}
INLINER void p132_covariance_SET(uint8_t  src, Pack * dst)//Measurement covariance in centimeters, 0 for unknown / invalid readings
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  11);
}
INLINER e_MAV_DISTANCE_SENSOR p132_type_GET(Pack * src)//Type from MAV_DISTANCE_SENSOR enum.
{
    uint8_t * data = src->data;
    return  0 + (int)get_bits(data, 96, 3);
}
INLINER void p132_type_SET(e_MAV_DISTANCE_SENSOR  src, Pack * dst)//Type from MAV_DISTANCE_SENSOR enum.
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 3, data, 96);
}
/**
*Direction the sensor faces from MAV_SENSOR_ORIENTATION enum. downward-facing: ROTATION_PITCH_270, upward-facing:
*	ROTATION_PITCH_90, backward-facing: ROTATION_PITCH_180, forward-facing: ROTATION_NONE, left-facing: ROTATION_YAW_90,
*	right-facing: ROTATION_YAW_27*/
INLINER e_MAV_SENSOR_ORIENTATION p132_orientation_GET(Pack * src)
{
    uint8_t * data = src->data;
    return  0 + (int)get_bits(data, 99, 6);
}
/**
*Direction the sensor faces from MAV_SENSOR_ORIENTATION enum. downward-facing: ROTATION_PITCH_270, upward-facing:
*	ROTATION_PITCH_90, backward-facing: ROTATION_PITCH_180, forward-facing: ROTATION_NONE, left-facing: ROTATION_YAW_90,
*	right-facing: ROTATION_YAW_27*/
INLINER void p132_orientation_SET(e_MAV_SENSOR_ORIENTATION  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 6, data, 99);
}
INLINER uint16_t p133_grid_spacing_GET(Pack * src)//Grid spacing in meters
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER void p133_grid_spacing_SET(uint16_t  src, Pack * dst)//Grid spacing in meters
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER uint64_t p133_mask_GET(Pack * src)//Bitmask of requested 4x4 grids (row major 8x7 array of grids, 56 bits)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 8)));
}
INLINER void p133_mask_SET(uint64_t  src, Pack * dst)//Bitmask of requested 4x4 grids (row major 8x7 array of grids, 56 bits)
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  2);
}
INLINER int32_t p133_lat_GET(Pack * src)//Latitude of SW corner of first grid (degrees *10^7)
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  10, 4)));
}
INLINER void p133_lat_SET(int32_t  src, Pack * dst)//Latitude of SW corner of first grid (degrees *10^7)
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  10);
}
INLINER int32_t p133_lon_GET(Pack * src)//Longitude of SW corner of first grid (in degrees *10^7)
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  14, 4)));
}
INLINER void p133_lon_SET(int32_t  src, Pack * dst)//Longitude of SW corner of first grid (in degrees *10^7)
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  14);
}
INLINER uint16_t p134_grid_spacing_GET(Pack * src)//Grid spacing in meters
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER void p134_grid_spacing_SET(uint16_t  src, Pack * dst)//Grid spacing in meters
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER int32_t p134_lat_GET(Pack * src)//Latitude of SW corner of first grid (degrees *10^7)
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  2, 4)));
}
INLINER void p134_lat_SET(int32_t  src, Pack * dst)//Latitude of SW corner of first grid (degrees *10^7)
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  2);
}
INLINER int32_t p134_lon_GET(Pack * src)//Longitude of SW corner of first grid (in degrees *10^7)
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  6, 4)));
}
INLINER void p134_lon_SET(int32_t  src, Pack * dst)//Longitude of SW corner of first grid (in degrees *10^7)
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  6);
}
INLINER uint8_t p134_gridbit_GET(Pack * src)//bit within the terrain request mask
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  10, 1)));
}
INLINER void p134_gridbit_SET(uint8_t  src, Pack * dst)//bit within the terrain request mask
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  10);
}
INLINER int16_t* p134_data__GET(Pack * src, int16_t*  dst, int32_t pos) //Terrain data in meters AMSL
{
    uint8_t * data = src->data;
    for(int32_t BYTE = 11, dst_max = pos + 16; pos < dst_max ; pos++,  BYTE += 2)
        dst[pos] = ((int16_t)(get_bytes(data,  BYTE, 2)));
    return dst;
}

static const  uint32_t p134_data__LEN = 16; //return array length

INLINER  int16_t*  p134_data__GET_(Pack * src) {return p134_data__GET(src, malloc(16 * sizeof(int16_t)), 0);}
INLINER void p134_data__SET(int16_t*  src, int32_t pos, Pack * dst) //Terrain data in meters AMSL
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  11, src_max = pos + 16; pos < src_max; pos++, BYTE += 2)
        set_bytes((uint16_t)(src[pos]), 2, data,  BYTE);
}
INLINER int32_t p135_lat_GET(Pack * src)//Latitude (degrees *10^7)
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  0, 4)));
}
INLINER void p135_lat_SET(int32_t  src, Pack * dst)//Latitude (degrees *10^7)
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  0);
}
INLINER int32_t p135_lon_GET(Pack * src)//Longitude (degrees *10^7)
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  4, 4)));
}
INLINER void p135_lon_SET(int32_t  src, Pack * dst)//Longitude (degrees *10^7)
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  4);
}
INLINER uint16_t p136_spacing_GET(Pack * src)//grid spacing (zero if terrain at this location unavailable)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER void p136_spacing_SET(uint16_t  src, Pack * dst)//grid spacing (zero if terrain at this location unavailable)
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER uint16_t p136_pending_GET(Pack * src)//Number of 4x4 terrain blocks waiting to be received or read from disk
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 2)));
}
INLINER void p136_pending_SET(uint16_t  src, Pack * dst)//Number of 4x4 terrain blocks waiting to be received or read from disk
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  2);
}
INLINER uint16_t p136_loaded_GET(Pack * src)//Number of 4x4 terrain blocks in memory
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  4, 2)));
}
INLINER void p136_loaded_SET(uint16_t  src, Pack * dst)//Number of 4x4 terrain blocks in memory
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  4);
}
INLINER int32_t p136_lat_GET(Pack * src)//Latitude (degrees *10^7)
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  6, 4)));
}
INLINER void p136_lat_SET(int32_t  src, Pack * dst)//Latitude (degrees *10^7)
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  6);
}
INLINER int32_t p136_lon_GET(Pack * src)//Longitude (degrees *10^7)
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  10, 4)));
}
INLINER void p136_lon_SET(int32_t  src, Pack * dst)//Longitude (degrees *10^7)
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  10);
}
INLINER float p136_terrain_height_GET(Pack * src)//Terrain height in meters AMSL
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  14, 4)));
}
INLINER void p136_terrain_height_SET(float  src, Pack * dst)//Terrain height in meters AMSL
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  14);
}
INLINER float p136_current_height_GET(Pack * src)//Current vehicle height above lat/lon terrain height (meters)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  18, 4)));
}
INLINER void p136_current_height_SET(float  src, Pack * dst)//Current vehicle height above lat/lon terrain height (meters)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  18);
}
INLINER uint32_t p137_time_boot_ms_GET(Pack * src)//Timestamp (milliseconds since system boot)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 4)));
}
INLINER void p137_time_boot_ms_SET(uint32_t  src, Pack * dst)//Timestamp (milliseconds since system boot)
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  0);
}
INLINER float p137_press_abs_GET(Pack * src)//Absolute pressure (hectopascal)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  4, 4)));
}
INLINER void p137_press_abs_SET(float  src, Pack * dst)//Absolute pressure (hectopascal)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  4);
}
INLINER float p137_press_diff_GET(Pack * src)//Differential pressure 1 (hectopascal)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  8, 4)));
}
INLINER void p137_press_diff_SET(float  src, Pack * dst)//Differential pressure 1 (hectopascal)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  8);
}
INLINER int16_t p137_temperature_GET(Pack * src)//Temperature measurement (0.01 degrees celsius)
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  12, 2)));
}
INLINER void p137_temperature_SET(int16_t  src, Pack * dst)//Temperature measurement (0.01 degrees celsius)
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  12);
}
INLINER uint64_t p138_time_usec_GET(Pack * src)//Timestamp (micros since boot or Unix epoch)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 8)));
}
INLINER void p138_time_usec_SET(uint64_t  src, Pack * dst)//Timestamp (micros since boot or Unix epoch)
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  0);
}
INLINER float* p138_q_GET(Pack * src, float*  dst, int32_t pos) //Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
{
    uint8_t * data = src->data;
    for(int32_t BYTE = 8, dst_max = pos + 4; pos < dst_max ; pos++,  BYTE += 4)
        dst[pos] = (intBitsToFloat(get_bytes(data,  BYTE, 4)));
    return dst;
}

static const  uint32_t p138_q_LEN = 4; //return array length

INLINER  float*  p138_q_GET_(Pack * src) {return p138_q_GET(src, malloc(4 * sizeof(float)), 0);}
INLINER void p138_q_SET(float*  src, int32_t pos, Pack * dst) //Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  8, src_max = pos + 4; pos < src_max; pos++, BYTE += 4)
        set_bytes(floatToIntBits(src[pos]), 4, data,  BYTE);
}
INLINER float p138_x_GET(Pack * src)//X position in meters (NED)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  24, 4)));
}
INLINER void p138_x_SET(float  src, Pack * dst)//X position in meters (NED)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  24);
}
INLINER float p138_y_GET(Pack * src)//Y position in meters (NED)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  28, 4)));
}
INLINER void p138_y_SET(float  src, Pack * dst)//Y position in meters (NED)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  28);
}
INLINER float p138_z_GET(Pack * src)//Z position in meters (NED)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  32, 4)));
}
INLINER void p138_z_SET(float  src, Pack * dst)//Z position in meters (NED)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  32);
}
INLINER uint64_t p139_time_usec_GET(Pack * src)//Timestamp (micros since boot or Unix epoch)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 8)));
}
INLINER void p139_time_usec_SET(uint64_t  src, Pack * dst)//Timestamp (micros since boot or Unix epoch)
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  0);
}
/**
*Actuator group. The "_mlx" indicates this is a multi-instance message and a MAVLink parser should use
*	this field to difference between instances*/
INLINER uint8_t p139_group_mlx_GET(Pack * src)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  8, 1)));
}
/**
*Actuator group. The "_mlx" indicates this is a multi-instance message and a MAVLink parser should use
*	this field to difference between instances*/
INLINER void p139_group_mlx_SET(uint8_t  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  8);
}
INLINER uint8_t p139_target_system_GET(Pack * src)//System ID
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  9, 1)));
}
INLINER void p139_target_system_SET(uint8_t  src, Pack * dst)//System ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  9);
}
INLINER uint8_t p139_target_component_GET(Pack * src)//Component ID
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  10, 1)));
}
INLINER void p139_target_component_SET(uint8_t  src, Pack * dst)//Component ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  10);
}
/**
*Actuator controls. Normed to -1..+1 where 0 is neutral position. Throttle for single rotation direction
*	motors is 0..1, negative range for reverse direction. Standard mapping for attitude controls (group 0):
*	(index 0-7): roll, pitch, yaw, throttle, flaps, spoilers, airbrakes, landing gear. Load a pass-through
*	mixer to repurpose them as generic outputs*/
INLINER float* p139_controls_GET(Pack * src, float*  dst, int32_t pos)
{
    uint8_t * data = src->data;
    for(int32_t BYTE = 11, dst_max = pos + 8; pos < dst_max ; pos++,  BYTE += 4)
        dst[pos] = (intBitsToFloat(get_bytes(data,  BYTE, 4)));
    return dst;
}

static const  uint32_t p139_controls_LEN = 8; //return array length
/**
*Actuator controls. Normed to -1..+1 where 0 is neutral position. Throttle for single rotation direction
*	motors is 0..1, negative range for reverse direction. Standard mapping for attitude controls (group 0):
*	(index 0-7): roll, pitch, yaw, throttle, flaps, spoilers, airbrakes, landing gear. Load a pass-through
*	mixer to repurpose them as generic outputs*/

INLINER  float*  p139_controls_GET_(Pack * src) {return p139_controls_GET(src, malloc(8 * sizeof(float)), 0);}/**
*Actuator controls. Normed to -1..+1 where 0 is neutral position. Throttle for single rotation direction
*	motors is 0..1, negative range for reverse direction. Standard mapping for attitude controls (group 0):
*	(index 0-7): roll, pitch, yaw, throttle, flaps, spoilers, airbrakes, landing gear. Load a pass-through
*	mixer to repurpose them as generic outputs*/
INLINER void p139_controls_SET(float*  src, int32_t pos, Pack * dst)
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  11, src_max = pos + 8; pos < src_max; pos++, BYTE += 4)
        set_bytes(floatToIntBits(src[pos]), 4, data,  BYTE);
}
INLINER uint64_t p140_time_usec_GET(Pack * src)//Timestamp (micros since boot or Unix epoch)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 8)));
}
INLINER void p140_time_usec_SET(uint64_t  src, Pack * dst)//Timestamp (micros since boot or Unix epoch)
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  0);
}
/**
*Actuator group. The "_mlx" indicates this is a multi-instance message and a MAVLink parser should use
*	this field to difference between instances*/
INLINER uint8_t p140_group_mlx_GET(Pack * src)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  8, 1)));
}
/**
*Actuator group. The "_mlx" indicates this is a multi-instance message and a MAVLink parser should use
*	this field to difference between instances*/
INLINER void p140_group_mlx_SET(uint8_t  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  8);
}
/**
*Actuator controls. Normed to -1..+1 where 0 is neutral position. Throttle for single rotation direction
*	motors is 0..1, negative range for reverse direction. Standard mapping for attitude controls (group 0):
*	(index 0-7): roll, pitch, yaw, throttle, flaps, spoilers, airbrakes, landing gear. Load a pass-through
*	mixer to repurpose them as generic outputs*/
INLINER float* p140_controls_GET(Pack * src, float*  dst, int32_t pos)
{
    uint8_t * data = src->data;
    for(int32_t BYTE = 9, dst_max = pos + 8; pos < dst_max ; pos++,  BYTE += 4)
        dst[pos] = (intBitsToFloat(get_bytes(data,  BYTE, 4)));
    return dst;
}

static const  uint32_t p140_controls_LEN = 8; //return array length
/**
*Actuator controls. Normed to -1..+1 where 0 is neutral position. Throttle for single rotation direction
*	motors is 0..1, negative range for reverse direction. Standard mapping for attitude controls (group 0):
*	(index 0-7): roll, pitch, yaw, throttle, flaps, spoilers, airbrakes, landing gear. Load a pass-through
*	mixer to repurpose them as generic outputs*/

INLINER  float*  p140_controls_GET_(Pack * src) {return p140_controls_GET(src, malloc(8 * sizeof(float)), 0);}/**
*Actuator controls. Normed to -1..+1 where 0 is neutral position. Throttle for single rotation direction
*	motors is 0..1, negative range for reverse direction. Standard mapping for attitude controls (group 0):
*	(index 0-7): roll, pitch, yaw, throttle, flaps, spoilers, airbrakes, landing gear. Load a pass-through
*	mixer to repurpose them as generic outputs*/
INLINER void p140_controls_SET(float*  src, int32_t pos, Pack * dst)
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  9, src_max = pos + 8; pos < src_max; pos++, BYTE += 4)
        set_bytes(floatToIntBits(src[pos]), 4, data,  BYTE);
}
INLINER uint64_t p141_time_usec_GET(Pack * src)//Timestamp (micros since boot or Unix epoch)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 8)));
}
INLINER void p141_time_usec_SET(uint64_t  src, Pack * dst)//Timestamp (micros since boot or Unix epoch)
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  0);
}
/**
*This altitude measure is initialized on system boot and monotonic (it is never reset, but represents the
*	local altitude change). The only guarantee on this field is that it will never be reset and is consistent
*	within a flight. The recommended value for this field is the uncorrected barometric altitude at boot
*	time. This altitude will also drift and vary between flights*/
INLINER float p141_altitude_monotonic_GET(Pack * src)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  8, 4)));
}
/**
*This altitude measure is initialized on system boot and monotonic (it is never reset, but represents the
*	local altitude change). The only guarantee on this field is that it will never be reset and is consistent
*	within a flight. The recommended value for this field is the uncorrected barometric altitude at boot
*	time. This altitude will also drift and vary between flights*/
INLINER void p141_altitude_monotonic_SET(float  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  8);
}
/**
*This altitude measure is strictly above mean sea level and might be non-monotonic (it might reset on events
*	like GPS lock or when a new QNH value is set). It should be the altitude to which global altitude waypoints
*	are compared to. Note that it is *not* the GPS altitude, however, most GPS modules already output AMSL
*	by default and not the WGS84 altitude*/
INLINER float p141_altitude_amsl_GET(Pack * src)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  12, 4)));
}
/**
*This altitude measure is strictly above mean sea level and might be non-monotonic (it might reset on events
*	like GPS lock or when a new QNH value is set). It should be the altitude to which global altitude waypoints
*	are compared to. Note that it is *not* the GPS altitude, however, most GPS modules already output AMSL
*	by default and not the WGS84 altitude*/
INLINER void p141_altitude_amsl_SET(float  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  12);
}
/**
*This is the local altitude in the local coordinate frame. It is not the altitude above home, but in reference
*	to the coordinate origin (0, 0, 0). It is up-positive*/
INLINER float p141_altitude_local_GET(Pack * src)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  16, 4)));
}
/**
*This is the local altitude in the local coordinate frame. It is not the altitude above home, but in reference
*	to the coordinate origin (0, 0, 0). It is up-positive*/
INLINER void p141_altitude_local_SET(float  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  16);
}
INLINER float p141_altitude_relative_GET(Pack * src)//This is the altitude above the home position. It resets on each change of the current home position
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  20, 4)));
}
INLINER void p141_altitude_relative_SET(float  src, Pack * dst)//This is the altitude above the home position. It resets on each change of the current home position
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  20);
}
/**
*This is the altitude above terrain. It might be fed by a terrain database or an altimeter. Values smaller
*	than -1000 should be interpreted as unknown*/
INLINER float p141_altitude_terrain_GET(Pack * src)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  24, 4)));
}
/**
*This is the altitude above terrain. It might be fed by a terrain database or an altimeter. Values smaller
*	than -1000 should be interpreted as unknown*/
INLINER void p141_altitude_terrain_SET(float  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  24);
}
/**
*This is not the altitude, but the clear space below the system according to the fused clearance estimate.
*	It generally should max out at the maximum range of e.g. the laser altimeter. It is generally a moving
*	target. A negative value indicates no measurement available*/
INLINER float p141_bottom_clearance_GET(Pack * src)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  28, 4)));
}
/**
*This is not the altitude, but the clear space below the system according to the fused clearance estimate.
*	It generally should max out at the maximum range of e.g. the laser altimeter. It is generally a moving
*	target. A negative value indicates no measurement available*/
INLINER void p141_bottom_clearance_SET(float  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  28);
}
INLINER uint8_t p142_request_id_GET(Pack * src)//Request ID. This ID should be re-used when sending back URI contents
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 1)));
}
INLINER void p142_request_id_SET(uint8_t  src, Pack * dst)//Request ID. This ID should be re-used when sending back URI contents
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  0);
}
INLINER uint8_t p142_uri_type_GET(Pack * src)//The type of requested URI. 0 = a file via URL. 1 = a UAVCAN binary
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  1, 1)));
}
INLINER void p142_uri_type_SET(uint8_t  src, Pack * dst)//The type of requested URI. 0 = a file via URL. 1 = a UAVCAN binary
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  1);
}
/**
*The requested unique resource identifier (URI). It is not necessarily a straight domain name (depends
*	on the URI type enum*/
INLINER uint8_t* p142_uri_GET(Pack * src, uint8_t*  dst, int32_t pos)
{
    uint8_t * data = src->data;
    for(int32_t BYTE = 2, dst_max = pos + 120; pos < dst_max ; pos++,  BYTE += 1)
        dst[pos] = ((get_bytes(data,  BYTE, 1)));
    return dst;
}

static const  uint32_t p142_uri_LEN = 120; //return array length
/**
*The requested unique resource identifier (URI). It is not necessarily a straight domain name (depends
*	on the URI type enum*/

INLINER  uint8_t*  p142_uri_GET_(Pack * src) {return p142_uri_GET(src, malloc(120 * sizeof(uint8_t)), 0);}/**
*The requested unique resource identifier (URI). It is not necessarily a straight domain name (depends
*	on the URI type enum*/
INLINER void p142_uri_SET(uint8_t*  src, int32_t pos, Pack * dst)
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  2, src_max = pos + 120; pos < src_max; pos++, BYTE += 1)
        set_bytes((src[pos]), 1, data,  BYTE);
}
INLINER uint8_t p142_transfer_type_GET(Pack * src)//The way the autopilot wants to receive the URI. 0 = MAVLink FTP. 1 = binary stream.
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  122, 1)));
}
INLINER void p142_transfer_type_SET(uint8_t  src, Pack * dst)//The way the autopilot wants to receive the URI. 0 = MAVLink FTP. 1 = binary stream.
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  122);
}
/**
*The storage path the autopilot wants the URI to be stored in. Will only be valid if the transfer_type
*	has a storage associated (e.g. MAVLink FTP)*/
INLINER uint8_t* p142_storage_GET(Pack * src, uint8_t*  dst, int32_t pos)
{
    uint8_t * data = src->data;
    for(int32_t BYTE = 123, dst_max = pos + 120; pos < dst_max ; pos++,  BYTE += 1)
        dst[pos] = ((get_bytes(data,  BYTE, 1)));
    return dst;
}

static const  uint32_t p142_storage_LEN = 120; //return array length
/**
*The storage path the autopilot wants the URI to be stored in. Will only be valid if the transfer_type
*	has a storage associated (e.g. MAVLink FTP)*/

INLINER  uint8_t*  p142_storage_GET_(Pack * src) {return p142_storage_GET(src, malloc(120 * sizeof(uint8_t)), 0);}/**
*The storage path the autopilot wants the URI to be stored in. Will only be valid if the transfer_type
*	has a storage associated (e.g. MAVLink FTP)*/
INLINER void p142_storage_SET(uint8_t*  src, int32_t pos, Pack * dst)
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  123, src_max = pos + 120; pos < src_max; pos++, BYTE += 1)
        set_bytes((src[pos]), 1, data,  BYTE);
}
INLINER uint32_t p143_time_boot_ms_GET(Pack * src)//Timestamp (milliseconds since system boot)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 4)));
}
INLINER void p143_time_boot_ms_SET(uint32_t  src, Pack * dst)//Timestamp (milliseconds since system boot)
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  0);
}
INLINER float p143_press_abs_GET(Pack * src)//Absolute pressure (hectopascal)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  4, 4)));
}
INLINER void p143_press_abs_SET(float  src, Pack * dst)//Absolute pressure (hectopascal)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  4);
}
INLINER float p143_press_diff_GET(Pack * src)//Differential pressure 1 (hectopascal)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  8, 4)));
}
INLINER void p143_press_diff_SET(float  src, Pack * dst)//Differential pressure 1 (hectopascal)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  8);
}
INLINER int16_t p143_temperature_GET(Pack * src)//Temperature measurement (0.01 degrees celsius)
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  12, 2)));
}
INLINER void p143_temperature_SET(int16_t  src, Pack * dst)//Temperature measurement (0.01 degrees celsius)
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  12);
}
INLINER uint64_t p144_timestamp_GET(Pack * src)//Timestamp in milliseconds since system boot
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 8)));
}
INLINER void p144_timestamp_SET(uint64_t  src, Pack * dst)//Timestamp in milliseconds since system boot
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  0);
}
INLINER uint64_t p144_custom_state_GET(Pack * src)//button states or switches of a tracker device
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  8, 8)));
}
INLINER void p144_custom_state_SET(uint64_t  src, Pack * dst)//button states or switches of a tracker device
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  8);
}
INLINER uint8_t p144_est_capabilities_GET(Pack * src)//bit positions for tracker reporting capabilities (POS = 0, VEL = 1, ACCEL = 2, ATT + RATES = 3)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  16, 1)));
}
INLINER void p144_est_capabilities_SET(uint8_t  src, Pack * dst)//bit positions for tracker reporting capabilities (POS = 0, VEL = 1, ACCEL = 2, ATT + RATES = 3)
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  16);
}
INLINER int32_t p144_lat_GET(Pack * src)//Latitude (WGS84), in degrees * 1E7
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  17, 4)));
}
INLINER void p144_lat_SET(int32_t  src, Pack * dst)//Latitude (WGS84), in degrees * 1E7
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  17);
}
INLINER int32_t p144_lon_GET(Pack * src)//Longitude (WGS84), in degrees * 1E7
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  21, 4)));
}
INLINER void p144_lon_SET(int32_t  src, Pack * dst)//Longitude (WGS84), in degrees * 1E7
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  21);
}
INLINER float p144_alt_GET(Pack * src)//AMSL, in meters
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  25, 4)));
}
INLINER void p144_alt_SET(float  src, Pack * dst)//AMSL, in meters
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  25);
}
INLINER float* p144_vel_GET(Pack * src, float*  dst, int32_t pos) //target velocity (0,0,0) for unknown
{
    uint8_t * data = src->data;
    for(int32_t BYTE = 29, dst_max = pos + 3; pos < dst_max ; pos++,  BYTE += 4)
        dst[pos] = (intBitsToFloat(get_bytes(data,  BYTE, 4)));
    return dst;
}

static const  uint32_t p144_vel_LEN = 3; //return array length

INLINER  float*  p144_vel_GET_(Pack * src) {return p144_vel_GET(src, malloc(3 * sizeof(float)), 0);}
INLINER void p144_vel_SET(float*  src, int32_t pos, Pack * dst) //target velocity (0,0,0) for unknown
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  29, src_max = pos + 3; pos < src_max; pos++, BYTE += 4)
        set_bytes(floatToIntBits(src[pos]), 4, data,  BYTE);
}
INLINER float* p144_acc_GET(Pack * src, float*  dst, int32_t pos) //linear target acceleration (0,0,0) for unknown
{
    uint8_t * data = src->data;
    for(int32_t BYTE = 41, dst_max = pos + 3; pos < dst_max ; pos++,  BYTE += 4)
        dst[pos] = (intBitsToFloat(get_bytes(data,  BYTE, 4)));
    return dst;
}

static const  uint32_t p144_acc_LEN = 3; //return array length

INLINER  float*  p144_acc_GET_(Pack * src) {return p144_acc_GET(src, malloc(3 * sizeof(float)), 0);}
INLINER void p144_acc_SET(float*  src, int32_t pos, Pack * dst) //linear target acceleration (0,0,0) for unknown
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  41, src_max = pos + 3; pos < src_max; pos++, BYTE += 4)
        set_bytes(floatToIntBits(src[pos]), 4, data,  BYTE);
}
INLINER float* p144_attitude_q_GET(Pack * src, float*  dst, int32_t pos) //(1 0 0 0 for unknown)
{
    uint8_t * data = src->data;
    for(int32_t BYTE = 53, dst_max = pos + 4; pos < dst_max ; pos++,  BYTE += 4)
        dst[pos] = (intBitsToFloat(get_bytes(data,  BYTE, 4)));
    return dst;
}

static const  uint32_t p144_attitude_q_LEN = 4; //return array length

INLINER  float*  p144_attitude_q_GET_(Pack * src) {return p144_attitude_q_GET(src, malloc(4 * sizeof(float)), 0);}
INLINER void p144_attitude_q_SET(float*  src, int32_t pos, Pack * dst) //(1 0 0 0 for unknown)
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  53, src_max = pos + 4; pos < src_max; pos++, BYTE += 4)
        set_bytes(floatToIntBits(src[pos]), 4, data,  BYTE);
}
INLINER float* p144_rates_GET(Pack * src, float*  dst, int32_t pos) //(0 0 0 for unknown)
{
    uint8_t * data = src->data;
    for(int32_t BYTE = 69, dst_max = pos + 3; pos < dst_max ; pos++,  BYTE += 4)
        dst[pos] = (intBitsToFloat(get_bytes(data,  BYTE, 4)));
    return dst;
}

static const  uint32_t p144_rates_LEN = 3; //return array length

INLINER  float*  p144_rates_GET_(Pack * src) {return p144_rates_GET(src, malloc(3 * sizeof(float)), 0);}
INLINER void p144_rates_SET(float*  src, int32_t pos, Pack * dst) //(0 0 0 for unknown)
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  69, src_max = pos + 3; pos < src_max; pos++, BYTE += 4)
        set_bytes(floatToIntBits(src[pos]), 4, data,  BYTE);
}
INLINER float* p144_position_cov_GET(Pack * src, float*  dst, int32_t pos) //eph epv
{
    uint8_t * data = src->data;
    for(int32_t BYTE = 81, dst_max = pos + 3; pos < dst_max ; pos++,  BYTE += 4)
        dst[pos] = (intBitsToFloat(get_bytes(data,  BYTE, 4)));
    return dst;
}

static const  uint32_t p144_position_cov_LEN = 3; //return array length

INLINER  float*  p144_position_cov_GET_(Pack * src) {return p144_position_cov_GET(src, malloc(3 * sizeof(float)), 0);}
INLINER void p144_position_cov_SET(float*  src, int32_t pos, Pack * dst) //eph epv
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  81, src_max = pos + 3; pos < src_max; pos++, BYTE += 4)
        set_bytes(floatToIntBits(src[pos]), 4, data,  BYTE);
}
INLINER uint64_t p146_time_usec_GET(Pack * src)//Timestamp (micros since boot or Unix epoch)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 8)));
}
INLINER void p146_time_usec_SET(uint64_t  src, Pack * dst)//Timestamp (micros since boot or Unix epoch)
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  0);
}
INLINER float p146_x_acc_GET(Pack * src)//X acceleration in body frame
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  8, 4)));
}
INLINER void p146_x_acc_SET(float  src, Pack * dst)//X acceleration in body frame
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  8);
}
INLINER float p146_y_acc_GET(Pack * src)//Y acceleration in body frame
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  12, 4)));
}
INLINER void p146_y_acc_SET(float  src, Pack * dst)//Y acceleration in body frame
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  12);
}
INLINER float p146_z_acc_GET(Pack * src)//Z acceleration in body frame
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  16, 4)));
}
INLINER void p146_z_acc_SET(float  src, Pack * dst)//Z acceleration in body frame
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  16);
}
INLINER float p146_x_vel_GET(Pack * src)//X velocity in body frame
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  20, 4)));
}
INLINER void p146_x_vel_SET(float  src, Pack * dst)//X velocity in body frame
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  20);
}
INLINER float p146_y_vel_GET(Pack * src)//Y velocity in body frame
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  24, 4)));
}
INLINER void p146_y_vel_SET(float  src, Pack * dst)//Y velocity in body frame
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  24);
}
INLINER float p146_z_vel_GET(Pack * src)//Z velocity in body frame
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  28, 4)));
}
INLINER void p146_z_vel_SET(float  src, Pack * dst)//Z velocity in body frame
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  28);
}
INLINER float p146_x_pos_GET(Pack * src)//X position in local frame
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  32, 4)));
}
INLINER void p146_x_pos_SET(float  src, Pack * dst)//X position in local frame
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  32);
}
INLINER float p146_y_pos_GET(Pack * src)//Y position in local frame
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  36, 4)));
}
INLINER void p146_y_pos_SET(float  src, Pack * dst)//Y position in local frame
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  36);
}
INLINER float p146_z_pos_GET(Pack * src)//Z position in local frame
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  40, 4)));
}
INLINER void p146_z_pos_SET(float  src, Pack * dst)//Z position in local frame
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  40);
}
INLINER float p146_airspeed_GET(Pack * src)//Airspeed, set to -1 if unknown
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  44, 4)));
}
INLINER void p146_airspeed_SET(float  src, Pack * dst)//Airspeed, set to -1 if unknown
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  44);
}
INLINER float* p146_vel_variance_GET(Pack * src, float*  dst, int32_t pos) //Variance of body velocity estimate
{
    uint8_t * data = src->data;
    for(int32_t BYTE = 48, dst_max = pos + 3; pos < dst_max ; pos++,  BYTE += 4)
        dst[pos] = (intBitsToFloat(get_bytes(data,  BYTE, 4)));
    return dst;
}

static const  uint32_t p146_vel_variance_LEN = 3; //return array length

INLINER  float*  p146_vel_variance_GET_(Pack * src) {return p146_vel_variance_GET(src, malloc(3 * sizeof(float)), 0);}
INLINER void p146_vel_variance_SET(float*  src, int32_t pos, Pack * dst) //Variance of body velocity estimate
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  48, src_max = pos + 3; pos < src_max; pos++, BYTE += 4)
        set_bytes(floatToIntBits(src[pos]), 4, data,  BYTE);
}
INLINER float* p146_pos_variance_GET(Pack * src, float*  dst, int32_t pos) //Variance in local position
{
    uint8_t * data = src->data;
    for(int32_t BYTE = 60, dst_max = pos + 3; pos < dst_max ; pos++,  BYTE += 4)
        dst[pos] = (intBitsToFloat(get_bytes(data,  BYTE, 4)));
    return dst;
}

static const  uint32_t p146_pos_variance_LEN = 3; //return array length

INLINER  float*  p146_pos_variance_GET_(Pack * src) {return p146_pos_variance_GET(src, malloc(3 * sizeof(float)), 0);}
INLINER void p146_pos_variance_SET(float*  src, int32_t pos, Pack * dst) //Variance in local position
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  60, src_max = pos + 3; pos < src_max; pos++, BYTE += 4)
        set_bytes(floatToIntBits(src[pos]), 4, data,  BYTE);
}
INLINER float* p146_q_GET(Pack * src, float*  dst, int32_t pos) //The attitude, represented as Quaternion
{
    uint8_t * data = src->data;
    for(int32_t BYTE = 72, dst_max = pos + 4; pos < dst_max ; pos++,  BYTE += 4)
        dst[pos] = (intBitsToFloat(get_bytes(data,  BYTE, 4)));
    return dst;
}

static const  uint32_t p146_q_LEN = 4; //return array length

INLINER  float*  p146_q_GET_(Pack * src) {return p146_q_GET(src, malloc(4 * sizeof(float)), 0);}
INLINER void p146_q_SET(float*  src, int32_t pos, Pack * dst) //The attitude, represented as Quaternion
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  72, src_max = pos + 4; pos < src_max; pos++, BYTE += 4)
        set_bytes(floatToIntBits(src[pos]), 4, data,  BYTE);
}
INLINER float p146_roll_rate_GET(Pack * src)//Angular rate in roll axis
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  88, 4)));
}
INLINER void p146_roll_rate_SET(float  src, Pack * dst)//Angular rate in roll axis
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  88);
}
INLINER float p146_pitch_rate_GET(Pack * src)//Angular rate in pitch axis
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  92, 4)));
}
INLINER void p146_pitch_rate_SET(float  src, Pack * dst)//Angular rate in pitch axis
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  92);
}
INLINER float p146_yaw_rate_GET(Pack * src)//Angular rate in yaw axis
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  96, 4)));
}
INLINER void p146_yaw_rate_SET(float  src, Pack * dst)//Angular rate in yaw axis
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  96);
}
/**
*Battery voltage of cells, in millivolts (1 = 1 millivolt). Cells above the valid cell count for this battery
*	should have the UINT16_MAX value*/
INLINER uint16_t* p147_voltages_GET(Pack * src, uint16_t*  dst, int32_t pos)
{
    uint8_t * data = src->data;
    for(int32_t BYTE = 0, dst_max = pos + 10; pos < dst_max ; pos++,  BYTE += 2)
        dst[pos] = ((get_bytes(data,  BYTE, 2)));
    return dst;
}

static const  uint32_t p147_voltages_LEN = 10; //return array length
/**
*Battery voltage of cells, in millivolts (1 = 1 millivolt). Cells above the valid cell count for this battery
*	should have the UINT16_MAX value*/

INLINER  uint16_t*  p147_voltages_GET_(Pack * src) {return p147_voltages_GET(src, malloc(10 * sizeof(uint16_t)), 0);}/**
*Battery voltage of cells, in millivolts (1 = 1 millivolt). Cells above the valid cell count for this battery
*	should have the UINT16_MAX value*/
INLINER void p147_voltages_SET(uint16_t*  src, int32_t pos, Pack * dst)
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  0, src_max = pos + 10; pos < src_max; pos++, BYTE += 2)
        set_bytes((src[pos]), 2, data,  BYTE);
}
INLINER uint8_t p147_id_GET(Pack * src)//Battery ID
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  20, 1)));
}
INLINER void p147_id_SET(uint8_t  src, Pack * dst)//Battery ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  20);
}
INLINER int16_t p147_temperature_GET(Pack * src)//Temperature of the battery in centi-degrees celsius. INT16_MAX for unknown temperature.
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  21, 2)));
}
INLINER void p147_temperature_SET(int16_t  src, Pack * dst)//Temperature of the battery in centi-degrees celsius. INT16_MAX for unknown temperature.
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  21);
}
INLINER int16_t p147_current_battery_GET(Pack * src)//Battery current, in 10*milliamperes (1 = 10 milliampere), -1: autopilot does not measure the curren
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  23, 2)));
}
INLINER void p147_current_battery_SET(int16_t  src, Pack * dst)//Battery current, in 10*milliamperes (1 = 10 milliampere), -1: autopilot does not measure the curren
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  23);
}
INLINER int32_t p147_current_consumed_GET(Pack * src)//Consumed charge, in milliampere hours (1 = 1 mAh), -1: autopilot does not provide mAh consumption estimat
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  25, 4)));
}
INLINER void p147_current_consumed_SET(int32_t  src, Pack * dst)//Consumed charge, in milliampere hours (1 = 1 mAh), -1: autopilot does not provide mAh consumption estimat
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  25);
}
/**
*Consumed energy, in HectoJoules (intergrated U*I*dt)  (1 = 100 Joule), -1: autopilot does not provide
*	energy consumption estimat*/
INLINER int32_t p147_energy_consumed_GET(Pack * src)
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  29, 4)));
}
/**
*Consumed energy, in HectoJoules (intergrated U*I*dt)  (1 = 100 Joule), -1: autopilot does not provide
*	energy consumption estimat*/
INLINER void p147_energy_consumed_SET(int32_t  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  29);
}
INLINER int8_t p147_battery_remaining_GET(Pack * src)//Remaining battery energy: (0%: 0, 100%: 100), -1: autopilot does not estimate the remaining battery
{
    uint8_t * data = src->data;
    return ((int8_t)(get_bytes(data,  33, 1)));
}
INLINER void p147_battery_remaining_SET(int8_t  src, Pack * dst)//Remaining battery energy: (0%: 0, 100%: 100), -1: autopilot does not estimate the remaining battery
{
    uint8_t * data = dst->data;
    set_bytes((uint8_t)(src), 1, data,  33);
}
INLINER e_MAV_BATTERY_FUNCTION p147_battery_function_GET(Pack * src)//Function of the battery
{
    uint8_t * data = src->data;
    return  0 + (int)get_bits(data, 272, 3);
}
INLINER void p147_battery_function_SET(e_MAV_BATTERY_FUNCTION  src, Pack * dst)//Function of the battery
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 3, data, 272);
}
INLINER e_MAV_BATTERY_TYPE p147_type_GET(Pack * src)//Type (chemistry) of the battery
{
    uint8_t * data = src->data;
    return  0 + (int)get_bits(data, 275, 3);
}
INLINER void p147_type_SET(e_MAV_BATTERY_TYPE  src, Pack * dst)//Type (chemistry) of the battery
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 3, data, 275);
}
INLINER uint16_t p148_vendor_id_GET(Pack * src)//ID of the board vendor
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER void p148_vendor_id_SET(uint16_t  src, Pack * dst)//ID of the board vendor
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER uint16_t p148_product_id_GET(Pack * src)//ID of the product
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 2)));
}
INLINER void p148_product_id_SET(uint16_t  src, Pack * dst)//ID of the product
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  2);
}
INLINER uint32_t p148_flight_sw_version_GET(Pack * src)//Firmware version number
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  4, 4)));
}
INLINER void p148_flight_sw_version_SET(uint32_t  src, Pack * dst)//Firmware version number
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  4);
}
INLINER uint32_t p148_middleware_sw_version_GET(Pack * src)//Middleware version number
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  8, 4)));
}
INLINER void p148_middleware_sw_version_SET(uint32_t  src, Pack * dst)//Middleware version number
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  8);
}
INLINER uint32_t p148_os_sw_version_GET(Pack * src)//Operating system version number
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  12, 4)));
}
INLINER void p148_os_sw_version_SET(uint32_t  src, Pack * dst)//Operating system version number
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  12);
}
INLINER uint32_t p148_board_version_GET(Pack * src)//HW / board version (last 8 bytes should be silicon ID, if any)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  16, 4)));
}
INLINER void p148_board_version_SET(uint32_t  src, Pack * dst)//HW / board version (last 8 bytes should be silicon ID, if any)
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  16);
}
INLINER uint64_t p148_uid_GET(Pack * src)//UID if provided by hardware (see uid2)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  20, 8)));
}
INLINER void p148_uid_SET(uint64_t  src, Pack * dst)//UID if provided by hardware (see uid2)
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  20);
}
/**
*Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but
*	should allow to identify the commit using the main version number even for very large code bases*/
INLINER uint8_t* p148_flight_custom_version_GET(Pack * src, uint8_t*  dst, int32_t pos)
{
    uint8_t * data = src->data;
    for(int32_t BYTE = 28, dst_max = pos + 8; pos < dst_max ; pos++,  BYTE += 1)
        dst[pos] = ((get_bytes(data,  BYTE, 1)));
    return dst;
}

static const  uint32_t p148_flight_custom_version_LEN = 8; //return array length
/**
*Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but
*	should allow to identify the commit using the main version number even for very large code bases*/

INLINER  uint8_t*  p148_flight_custom_version_GET_(Pack * src) {return p148_flight_custom_version_GET(src, malloc(8 * sizeof(uint8_t)), 0);}/**
*Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but
*	should allow to identify the commit using the main version number even for very large code bases*/
INLINER void p148_flight_custom_version_SET(uint8_t*  src, int32_t pos, Pack * dst)
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  28, src_max = pos + 8; pos < src_max; pos++, BYTE += 1)
        set_bytes((src[pos]), 1, data,  BYTE);
}
/**
*Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but
*	should allow to identify the commit using the main version number even for very large code bases*/
INLINER uint8_t* p148_middleware_custom_version_GET(Pack * src, uint8_t*  dst, int32_t pos)
{
    uint8_t * data = src->data;
    for(int32_t BYTE = 36, dst_max = pos + 8; pos < dst_max ; pos++,  BYTE += 1)
        dst[pos] = ((get_bytes(data,  BYTE, 1)));
    return dst;
}

static const  uint32_t p148_middleware_custom_version_LEN = 8; //return array length
/**
*Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but
*	should allow to identify the commit using the main version number even for very large code bases*/

INLINER  uint8_t*  p148_middleware_custom_version_GET_(Pack * src) {return p148_middleware_custom_version_GET(src, malloc(8 * sizeof(uint8_t)), 0);}/**
*Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but
*	should allow to identify the commit using the main version number even for very large code bases*/
INLINER void p148_middleware_custom_version_SET(uint8_t*  src, int32_t pos, Pack * dst)
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  36, src_max = pos + 8; pos < src_max; pos++, BYTE += 1)
        set_bytes((src[pos]), 1, data,  BYTE);
}
/**
*Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but
*	should allow to identify the commit using the main version number even for very large code bases*/
INLINER uint8_t* p148_os_custom_version_GET(Pack * src, uint8_t*  dst, int32_t pos)
{
    uint8_t * data = src->data;
    for(int32_t BYTE = 44, dst_max = pos + 8; pos < dst_max ; pos++,  BYTE += 1)
        dst[pos] = ((get_bytes(data,  BYTE, 1)));
    return dst;
}

static const  uint32_t p148_os_custom_version_LEN = 8; //return array length
/**
*Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but
*	should allow to identify the commit using the main version number even for very large code bases*/

INLINER  uint8_t*  p148_os_custom_version_GET_(Pack * src) {return p148_os_custom_version_GET(src, malloc(8 * sizeof(uint8_t)), 0);}/**
*Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but
*	should allow to identify the commit using the main version number even for very large code bases*/
INLINER void p148_os_custom_version_SET(uint8_t*  src, int32_t pos, Pack * dst)
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  44, src_max = pos + 8; pos < src_max; pos++, BYTE += 1)
        set_bytes((src[pos]), 1, data,  BYTE);
}
INLINER e_MAV_PROTOCOL_CAPABILITY p148_capabilities_GET(Pack * src)//bitmask of capabilities (see MAV_PROTOCOL_CAPABILITY enum)
{
    uint8_t * data = src->data;
    return  1 + (int)get_bits(data, 416, 17);
}
INLINER void p148_capabilities_SET(e_MAV_PROTOCOL_CAPABILITY  src, Pack * dst)//bitmask of capabilities (see MAV_PROTOCOL_CAPABILITY enum)
{
    uint8_t * data = dst->data;
    set_bits(- 1 +   src, 17, data, 416);
}
/**
*UID if provided by hardware (supersedes the uid field. If this is non-zero, use this field, otherwise
*	use uid*/
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
    if(src->base.field_bit !=  433 && !try_visit_field(src, 433)) return NULL;
    uint8_t * data = src->base.pack->data;
    return p148_uid2_GET(src, malloc(18 * sizeof(uint8_t)), 0);
}
/**
*UID if provided by hardware (supersedes the uid field. If this is non-zero, use this field, otherwise
*	use uid*/
INLINER void p148_uid2_SET(uint8_t*  src, int32_t pos, Bounds_Inside * dst)
{
    if(dst->base.field_bit != 433)insert_field(dst, 433, 0);
    uint8_t * data = dst->base.pack->data;
    for(int32_t BYTE =  dst->BYTE, src_max = pos + 18; pos < src_max; pos++, BYTE += 1)
        set_bytes((src[pos]), 1, data,  BYTE);
}
INLINER uint64_t p149_time_usec_GET(Pack * src)//Timestamp (micros since boot or Unix epoch)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 8)));
}
INLINER void p149_time_usec_SET(uint64_t  src, Pack * dst)//Timestamp (micros since boot or Unix epoch)
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  0);
}
INLINER uint8_t p149_target_num_GET(Pack * src)//The ID of the target if multiple targets are present
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  8, 1)));
}
INLINER void p149_target_num_SET(uint8_t  src, Pack * dst)//The ID of the target if multiple targets are present
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  8);
}
INLINER float p149_angle_x_GET(Pack * src)//X-axis angular offset (in radians) of the target from the center of the image
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  9, 4)));
}
INLINER void p149_angle_x_SET(float  src, Pack * dst)//X-axis angular offset (in radians) of the target from the center of the image
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  9);
}
INLINER float p149_angle_y_GET(Pack * src)//Y-axis angular offset (in radians) of the target from the center of the image
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  13, 4)));
}
INLINER void p149_angle_y_SET(float  src, Pack * dst)//Y-axis angular offset (in radians) of the target from the center of the image
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  13);
}
INLINER float p149_distance_GET(Pack * src)//Distance to the target from the vehicle in meters
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  17, 4)));
}
INLINER void p149_distance_SET(float  src, Pack * dst)//Distance to the target from the vehicle in meters
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  17);
}
INLINER float p149_size_x_GET(Pack * src)//Size in radians of target along x-axis
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  21, 4)));
}
INLINER void p149_size_x_SET(float  src, Pack * dst)//Size in radians of target along x-axis
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  21);
}
INLINER float p149_size_y_GET(Pack * src)//Size in radians of target along y-axis
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  25, 4)));
}
INLINER void p149_size_y_SET(float  src, Pack * dst)//Size in radians of target along y-axis
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  25);
}
INLINER e_MAV_FRAME p149_frame_GET(Pack * src)//MAV_FRAME enum specifying the whether the following feilds are earth-frame, body-frame, etc.
{
    uint8_t * data = src->data;
    return  0 + (int)get_bits(data, 232, 4);
}
INLINER void p149_frame_SET(e_MAV_FRAME  src, Pack * dst)//MAV_FRAME enum specifying the whether the following feilds are earth-frame, body-frame, etc.
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 4, data, 232);
}
INLINER e_LANDING_TARGET_TYPE p149_type_GET(Pack * src)//LANDING_TARGET_TYPE enum specifying the type of landing target
{
    uint8_t * data = src->data;
    return  0 + (int)get_bits(data, 236, 2);
}
INLINER void p149_type_SET(e_LANDING_TARGET_TYPE  src, Pack * dst)//LANDING_TARGET_TYPE enum specifying the type of landing target
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 2, data, 236);
}
INLINER float  p149_x_TRY(Bounds_Inside * src)
{
    if(src->base.field_bit !=  238 && !try_visit_field(src, 238)) return 0;
    uint8_t * data = src->base.pack->data;
    return (intBitsToFloat(get_bytes(data,  src->BYTE, 4)));
}
INLINER void p149_x_SET(float  src, Bounds_Inside * dst)//X Position of the landing target on MAV_FRAME
{
    if(dst->base.field_bit != 238)insert_field(dst, 238, 0);
    uint8_t * data = dst->base.pack->data;
    set_bytes(floatToIntBits(src), 4, data,  dst->BYTE);
}
INLINER float  p149_y_TRY(Bounds_Inside * src)
{
    if(src->base.field_bit !=  239 && !try_visit_field(src, 239)) return 0;
    uint8_t * data = src->base.pack->data;
    return (intBitsToFloat(get_bytes(data,  src->BYTE, 4)));
}
INLINER void p149_y_SET(float  src, Bounds_Inside * dst)//Y Position of the landing target on MAV_FRAME
{
    if(dst->base.field_bit != 239)insert_field(dst, 239, 0);
    uint8_t * data = dst->base.pack->data;
    set_bytes(floatToIntBits(src), 4, data,  dst->BYTE);
}
INLINER float  p149_z_TRY(Bounds_Inside * src)
{
    if(src->base.field_bit !=  240 && !try_visit_field(src, 240)) return 0;
    uint8_t * data = src->base.pack->data;
    return (intBitsToFloat(get_bytes(data,  src->BYTE, 4)));
}
INLINER void p149_z_SET(float  src, Bounds_Inside * dst)//Z Position of the landing target on MAV_FRAME
{
    if(dst->base.field_bit != 240)insert_field(dst, 240, 0);
    uint8_t * data = dst->base.pack->data;
    set_bytes(floatToIntBits(src), 4, data,  dst->BYTE);
}
INLINER float* p149_q_GET(Bounds_Inside * src, float*  dst, int32_t pos) //Quaternion of landing target orientation (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
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
    if(src->base.field_bit !=  241 && !try_visit_field(src, 241)) return NULL;
    uint8_t * data = src->base.pack->data;
    return p149_q_GET(src, malloc(4 * sizeof(float)), 0);
}
INLINER void p149_q_SET(float*  src, int32_t pos, Bounds_Inside * dst) //Quaternion of landing target orientation (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
{
    if(dst->base.field_bit != 241)insert_field(dst, 241, 0);
    uint8_t * data = dst->base.pack->data;
    for(int32_t BYTE =  dst->BYTE, src_max = pos + 4; pos < src_max; pos++, BYTE += 4)
        set_bytes(floatToIntBits(src[pos]), 4, data,  BYTE);
}
INLINER uint8_t  p149_position_valid_TRY(Bounds_Inside * src)
{
    if(src->base.field_bit !=  242 && !try_visit_field(src, 242)) return 0;
    uint8_t * data = src->base.pack->data;
    return ((get_bytes(data,  src->BYTE, 1)));
}
/**
*Boolean indicating known position (1) or default unkown position (0), for validation of positioning of
*	the landing targe*/
INLINER void p149_position_valid_SET(uint8_t  src, Bounds_Inside * dst)
{
    if(dst->base.field_bit != 242)insert_field(dst, 242, 0);
    uint8_t * data = dst->base.pack->data;
    set_bytes((src), 1, data,  dst->BYTE);
}
INLINER uint8_t p150_target_system_GET(Pack * src)//System ID
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 1)));
}
INLINER uint8_t p150_target_component_GET(Pack * src)//Component ID
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  1, 1)));
}
INLINER uint8_t p151_target_system_GET(Pack * src)//System ID
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 1)));
}
INLINER uint8_t p151_target_component_GET(Pack * src)//Component ID
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  1, 1)));
}
INLINER int16_t p151_read_req_type_GET(Pack * src)//Type of flexifunction data requested
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  2, 2)));
}
INLINER int16_t p151_data_index_GET(Pack * src)//index into data where needed
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  4, 2)));
}
INLINER uint16_t p152_func_index_GET(Pack * src)//Function index
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER uint16_t p152_func_count_GET(Pack * src)//Total count of functions
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 2)));
}
INLINER uint16_t p152_data_address_GET(Pack * src)//Address in the flexifunction data, Set to 0xFFFF to use address in target memory
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  4, 2)));
}
INLINER uint16_t p152_data_size_GET(Pack * src)//Size of the
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  6, 2)));
}
INLINER uint8_t p152_target_system_GET(Pack * src)//System ID
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  8, 1)));
}
INLINER uint8_t p152_target_component_GET(Pack * src)//Component ID
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  9, 1)));
}
INLINER int8_t* p152_data__GET(Pack * src, int8_t*  dst, int32_t pos) //Settings data
{
    uint8_t * data = src->data;
    for(int32_t BYTE = 10, dst_max = pos + 48; pos < dst_max ; pos++,  BYTE += 1)
        dst[pos] = ((int8_t)(get_bytes(data,  BYTE, 1)));
    return dst;
}

static const  uint32_t p152_data__LEN = 48; //return array length

INLINER  int8_t*  p152_data__GET_(Pack * src) {return p152_data__GET(src, malloc(48 * sizeof(int8_t)), 0);}
INLINER uint16_t p153_func_index_GET(Pack * src)//Function index
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER uint16_t p153_result_GET(Pack * src)//result of acknowledge, 0=fail, 1=good
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 2)));
}
INLINER uint8_t p153_target_system_GET(Pack * src)//System ID
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  4, 1)));
}
INLINER uint8_t p153_target_component_GET(Pack * src)//Component ID
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  5, 1)));
}
INLINER uint8_t p155_target_system_GET(Pack * src)//System ID
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 1)));
}
INLINER uint8_t p155_target_component_GET(Pack * src)//Component ID
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  1, 1)));
}
INLINER uint8_t p155_directory_type_GET(Pack * src)//0=inputs, 1=outputs
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 1)));
}
INLINER uint8_t p155_start_index_GET(Pack * src)//index of first directory entry to write
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  3, 1)));
}
INLINER uint8_t p155_count_GET(Pack * src)//count of directory entries to write
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  4, 1)));
}
INLINER int8_t* p155_directory_data_GET(Pack * src, int8_t*  dst, int32_t pos) //Settings data
{
    uint8_t * data = src->data;
    for(int32_t BYTE = 5, dst_max = pos + 48; pos < dst_max ; pos++,  BYTE += 1)
        dst[pos] = ((int8_t)(get_bytes(data,  BYTE, 1)));
    return dst;
}

static const  uint32_t p155_directory_data_LEN = 48; //return array length

INLINER  int8_t*  p155_directory_data_GET_(Pack * src) {return p155_directory_data_GET(src, malloc(48 * sizeof(int8_t)), 0);}
INLINER uint16_t p156_result_GET(Pack * src)//result of acknowledge, 0=fail, 1=good
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER uint8_t p156_target_system_GET(Pack * src)//System ID
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 1)));
}
INLINER uint8_t p156_target_component_GET(Pack * src)//Component ID
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  3, 1)));
}
INLINER uint8_t p156_directory_type_GET(Pack * src)//0=inputs, 1=outputs
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  4, 1)));
}
INLINER uint8_t p156_start_index_GET(Pack * src)//index of first directory entry to write
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  5, 1)));
}
INLINER uint8_t p156_count_GET(Pack * src)//count of directory entries to write
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  6, 1)));
}
INLINER uint8_t p157_target_system_GET(Pack * src)//System ID
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 1)));
}
INLINER uint8_t p157_target_component_GET(Pack * src)//Component ID
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  1, 1)));
}
INLINER uint8_t p157_command_type_GET(Pack * src)//Flexifunction command type
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 1)));
}
INLINER uint16_t p158_command_type_GET(Pack * src)//Command acknowledged
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER uint16_t p158_result_GET(Pack * src)//result of acknowledge
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 2)));
}
INLINER uint16_t p170_sue_waypoint_index_GET(Pack * src)//Serial UDB Extra Waypoint Index
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER uint16_t p170_sue_cog_GET(Pack * src)//Serial UDB Extra GPS Course Over Ground
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 2)));
}
INLINER uint16_t p170_sue_cpu_load_GET(Pack * src)//Serial UDB Extra CPU Load
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  4, 2)));
}
INLINER uint16_t p170_sue_air_speed_3DIMU_GET(Pack * src)//Serial UDB Extra 3D IMU Air Speed
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  6, 2)));
}
INLINER uint32_t p170_sue_time_GET(Pack * src)//Serial UDB Extra Time
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  8, 4)));
}
INLINER uint8_t p170_sue_status_GET(Pack * src)//Serial UDB Extra Status
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  12, 1)));
}
INLINER int32_t p170_sue_latitude_GET(Pack * src)//Serial UDB Extra Latitude
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  13, 4)));
}
INLINER int32_t p170_sue_longitude_GET(Pack * src)//Serial UDB Extra Longitude
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  17, 4)));
}
INLINER int32_t p170_sue_altitude_GET(Pack * src)//Serial UDB Extra Altitude
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  21, 4)));
}
INLINER int16_t p170_sue_rmat0_GET(Pack * src)//Serial UDB Extra Rmat 0
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  25, 2)));
}
INLINER int16_t p170_sue_rmat1_GET(Pack * src)//Serial UDB Extra Rmat 1
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  27, 2)));
}
INLINER int16_t p170_sue_rmat2_GET(Pack * src)//Serial UDB Extra Rmat 2
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  29, 2)));
}
INLINER int16_t p170_sue_rmat3_GET(Pack * src)//Serial UDB Extra Rmat 3
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  31, 2)));
}
INLINER int16_t p170_sue_rmat4_GET(Pack * src)//Serial UDB Extra Rmat 4
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  33, 2)));
}
INLINER int16_t p170_sue_rmat5_GET(Pack * src)//Serial UDB Extra Rmat 5
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  35, 2)));
}
INLINER int16_t p170_sue_rmat6_GET(Pack * src)//Serial UDB Extra Rmat 6
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  37, 2)));
}
INLINER int16_t p170_sue_rmat7_GET(Pack * src)//Serial UDB Extra Rmat 7
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  39, 2)));
}
INLINER int16_t p170_sue_rmat8_GET(Pack * src)//Serial UDB Extra Rmat 8
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  41, 2)));
}
INLINER int16_t p170_sue_sog_GET(Pack * src)//Serial UDB Extra Speed Over Ground
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  43, 2)));
}
INLINER int16_t p170_sue_estimated_wind_0_GET(Pack * src)//Serial UDB Extra Estimated Wind 0
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  45, 2)));
}
INLINER int16_t p170_sue_estimated_wind_1_GET(Pack * src)//Serial UDB Extra Estimated Wind 1
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  47, 2)));
}
INLINER int16_t p170_sue_estimated_wind_2_GET(Pack * src)//Serial UDB Extra Estimated Wind 2
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  49, 2)));
}
INLINER int16_t p170_sue_magFieldEarth0_GET(Pack * src)//Serial UDB Extra Magnetic Field Earth 0
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  51, 2)));
}
INLINER int16_t p170_sue_magFieldEarth1_GET(Pack * src)//Serial UDB Extra Magnetic Field Earth 1
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  53, 2)));
}
INLINER int16_t p170_sue_magFieldEarth2_GET(Pack * src)//Serial UDB Extra Magnetic Field Earth 2
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  55, 2)));
}
INLINER int16_t p170_sue_svs_GET(Pack * src)//Serial UDB Extra Number of Sattelites in View
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  57, 2)));
}
INLINER int16_t p170_sue_hdop_GET(Pack * src)//Serial UDB Extra GPS Horizontal Dilution of Precision
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  59, 2)));
}
INLINER uint32_t p171_sue_time_GET(Pack * src)//Serial UDB Extra Time
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 4)));
}
INLINER uint32_t p171_sue_flags_GET(Pack * src)//Serial UDB Extra Status Flags
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  4, 4)));
}
INLINER int16_t p171_sue_pwm_input_1_GET(Pack * src)//Serial UDB Extra PWM Input Channel 1
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  8, 2)));
}
INLINER int16_t p171_sue_pwm_input_2_GET(Pack * src)//Serial UDB Extra PWM Input Channel 2
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  10, 2)));
}
INLINER int16_t p171_sue_pwm_input_3_GET(Pack * src)//Serial UDB Extra PWM Input Channel 3
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  12, 2)));
}
INLINER int16_t p171_sue_pwm_input_4_GET(Pack * src)//Serial UDB Extra PWM Input Channel 4
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  14, 2)));
}
INLINER int16_t p171_sue_pwm_input_5_GET(Pack * src)//Serial UDB Extra PWM Input Channel 5
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  16, 2)));
}
INLINER int16_t p171_sue_pwm_input_6_GET(Pack * src)//Serial UDB Extra PWM Input Channel 6
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  18, 2)));
}
INLINER int16_t p171_sue_pwm_input_7_GET(Pack * src)//Serial UDB Extra PWM Input Channel 7
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  20, 2)));
}
INLINER int16_t p171_sue_pwm_input_8_GET(Pack * src)//Serial UDB Extra PWM Input Channel 8
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  22, 2)));
}
INLINER int16_t p171_sue_pwm_input_9_GET(Pack * src)//Serial UDB Extra PWM Input Channel 9
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  24, 2)));
}
INLINER int16_t p171_sue_pwm_input_10_GET(Pack * src)//Serial UDB Extra PWM Input Channel 10
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  26, 2)));
}
INLINER int16_t p171_sue_pwm_input_11_GET(Pack * src)//Serial UDB Extra PWM Input Channel 11
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  28, 2)));
}
INLINER int16_t p171_sue_pwm_input_12_GET(Pack * src)//Serial UDB Extra PWM Input Channel 12
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  30, 2)));
}
INLINER int16_t p171_sue_pwm_output_1_GET(Pack * src)//Serial UDB Extra PWM Output Channel 1
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  32, 2)));
}
INLINER int16_t p171_sue_pwm_output_2_GET(Pack * src)//Serial UDB Extra PWM Output Channel 2
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  34, 2)));
}
INLINER int16_t p171_sue_pwm_output_3_GET(Pack * src)//Serial UDB Extra PWM Output Channel 3
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  36, 2)));
}
INLINER int16_t p171_sue_pwm_output_4_GET(Pack * src)//Serial UDB Extra PWM Output Channel 4
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  38, 2)));
}
INLINER int16_t p171_sue_pwm_output_5_GET(Pack * src)//Serial UDB Extra PWM Output Channel 5
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  40, 2)));
}
INLINER int16_t p171_sue_pwm_output_6_GET(Pack * src)//Serial UDB Extra PWM Output Channel 6
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  42, 2)));
}
INLINER int16_t p171_sue_pwm_output_7_GET(Pack * src)//Serial UDB Extra PWM Output Channel 7
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  44, 2)));
}
INLINER int16_t p171_sue_pwm_output_8_GET(Pack * src)//Serial UDB Extra PWM Output Channel 8
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  46, 2)));
}
INLINER int16_t p171_sue_pwm_output_9_GET(Pack * src)//Serial UDB Extra PWM Output Channel 9
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  48, 2)));
}
INLINER int16_t p171_sue_pwm_output_10_GET(Pack * src)//Serial UDB Extra PWM Output Channel 10
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  50, 2)));
}
INLINER int16_t p171_sue_pwm_output_11_GET(Pack * src)//Serial UDB Extra PWM Output Channel 11
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  52, 2)));
}
INLINER int16_t p171_sue_pwm_output_12_GET(Pack * src)//Serial UDB Extra PWM Output Channel 12
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  54, 2)));
}
INLINER int16_t p171_sue_imu_location_x_GET(Pack * src)//Serial UDB Extra IMU Location X
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  56, 2)));
}
INLINER int16_t p171_sue_imu_location_y_GET(Pack * src)//Serial UDB Extra IMU Location Y
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  58, 2)));
}
INLINER int16_t p171_sue_imu_location_z_GET(Pack * src)//Serial UDB Extra IMU Location Z
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  60, 2)));
}
INLINER int16_t p171_sue_location_error_earth_x_GET(Pack * src)//Serial UDB Location Error Earth X
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  62, 2)));
}
INLINER int16_t p171_sue_location_error_earth_y_GET(Pack * src)//Serial UDB Location Error Earth Y
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  64, 2)));
}
INLINER int16_t p171_sue_location_error_earth_z_GET(Pack * src)//Serial UDB Location Error Earth Z
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  66, 2)));
}
INLINER int16_t p171_sue_osc_fails_GET(Pack * src)//Serial UDB Extra Oscillator Failure Count
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  68, 2)));
}
INLINER int16_t p171_sue_imu_velocity_x_GET(Pack * src)//Serial UDB Extra IMU Velocity X
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  70, 2)));
}
INLINER int16_t p171_sue_imu_velocity_y_GET(Pack * src)//Serial UDB Extra IMU Velocity Y
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  72, 2)));
}
INLINER int16_t p171_sue_imu_velocity_z_GET(Pack * src)//Serial UDB Extra IMU Velocity Z
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  74, 2)));
}
INLINER int16_t p171_sue_waypoint_goal_x_GET(Pack * src)//Serial UDB Extra Current Waypoint Goal X
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  76, 2)));
}
INLINER int16_t p171_sue_waypoint_goal_y_GET(Pack * src)//Serial UDB Extra Current Waypoint Goal Y
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  78, 2)));
}
INLINER int16_t p171_sue_waypoint_goal_z_GET(Pack * src)//Serial UDB Extra Current Waypoint Goal Z
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  80, 2)));
}
INLINER int16_t p171_sue_aero_x_GET(Pack * src)//Aeroforce in UDB X Axis
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  82, 2)));
}
INLINER int16_t p171_sue_aero_y_GET(Pack * src)//Aeroforce in UDB Y Axis
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  84, 2)));
}
INLINER int16_t p171_sue_aero_z_GET(Pack * src)//Aeroforce in UDB Z axis
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  86, 2)));
}
INLINER int16_t p171_sue_barom_temp_GET(Pack * src)//SUE barometer temperature
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  88, 2)));
}
INLINER int32_t p171_sue_barom_press_GET(Pack * src)//SUE barometer pressure
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  90, 4)));
}
INLINER int32_t p171_sue_barom_alt_GET(Pack * src)//SUE barometer altitude
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  94, 4)));
}
INLINER int16_t p171_sue_bat_volt_GET(Pack * src)//SUE battery voltage
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  98, 2)));
}
INLINER int16_t p171_sue_bat_amp_GET(Pack * src)//SUE battery current
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  100, 2)));
}
INLINER int16_t p171_sue_bat_amp_hours_GET(Pack * src)//SUE battery milli amp hours used
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  102, 2)));
}
INLINER int16_t p171_sue_desired_height_GET(Pack * src)//Sue autopilot desired height
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  104, 2)));
}
INLINER int16_t p171_sue_memory_stack_free_GET(Pack * src)//Serial UDB Extra Stack Memory Free
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  106, 2)));
}
INLINER uint8_t p172_sue_ROLL_STABILIZATION_AILERONS_GET(Pack * src)//Serial UDB Extra Roll Stabilization with Ailerons Enabled
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 1)));
}
INLINER uint8_t p172_sue_ROLL_STABILIZATION_RUDDER_GET(Pack * src)//Serial UDB Extra Roll Stabilization with Rudder Enabled
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  1, 1)));
}
INLINER uint8_t p172_sue_PITCH_STABILIZATION_GET(Pack * src)//Serial UDB Extra Pitch Stabilization Enabled
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 1)));
}
INLINER uint8_t p172_sue_YAW_STABILIZATION_RUDDER_GET(Pack * src)//Serial UDB Extra Yaw Stabilization using Rudder Enabled
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  3, 1)));
}
INLINER uint8_t p172_sue_YAW_STABILIZATION_AILERON_GET(Pack * src)//Serial UDB Extra Yaw Stabilization using Ailerons Enabled
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  4, 1)));
}
INLINER uint8_t p172_sue_AILERON_NAVIGATION_GET(Pack * src)//Serial UDB Extra Navigation with Ailerons Enabled
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  5, 1)));
}
INLINER uint8_t p172_sue_RUDDER_NAVIGATION_GET(Pack * src)//Serial UDB Extra Navigation with Rudder Enabled
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  6, 1)));
}
INLINER uint8_t p172_sue_ALTITUDEHOLD_STABILIZED_GET(Pack * src)//Serial UDB Extra Type of Alitude Hold when in Stabilized Mode
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  7, 1)));
}
INLINER uint8_t p172_sue_ALTITUDEHOLD_WAYPOINT_GET(Pack * src)//Serial UDB Extra Type of Alitude Hold when in Waypoint Mode
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  8, 1)));
}
INLINER uint8_t p172_sue_RACING_MODE_GET(Pack * src)//Serial UDB Extra Firmware racing mode enabled
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  9, 1)));
}
INLINER float p173_sue_YAWKP_AILERON_GET(Pack * src)//Serial UDB YAWKP_AILERON Gain for Proporional control of navigation
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  0, 4)));
}
INLINER float p173_sue_YAWKD_AILERON_GET(Pack * src)//Serial UDB YAWKD_AILERON Gain for Rate control of navigation
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  4, 4)));
}
INLINER float p173_sue_ROLLKP_GET(Pack * src)//Serial UDB Extra ROLLKP Gain for Proportional control of roll stabilization
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  8, 4)));
}
INLINER float p173_sue_ROLLKD_GET(Pack * src)//Serial UDB Extra ROLLKD Gain for Rate control of roll stabilization
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  12, 4)));
}
INLINER float p174_sue_PITCHGAIN_GET(Pack * src)//Serial UDB Extra PITCHGAIN Proportional Control
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  0, 4)));
}
INLINER float p174_sue_PITCHKD_GET(Pack * src)//Serial UDB Extra Pitch Rate Control
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  4, 4)));
}
INLINER float p174_sue_RUDDER_ELEV_MIX_GET(Pack * src)//Serial UDB Extra Rudder to Elevator Mix
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  8, 4)));
}
INLINER float p174_sue_ROLL_ELEV_MIX_GET(Pack * src)//Serial UDB Extra Roll to Elevator Mix
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  12, 4)));
}
INLINER float p174_sue_ELEVATOR_BOOST_GET(Pack * src)//Gain For Boosting Manual Elevator control When Plane Stabilized
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  16, 4)));
}
INLINER float p175_sue_YAWKP_RUDDER_GET(Pack * src)//Serial UDB YAWKP_RUDDER Gain for Proporional control of navigation
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  0, 4)));
}
INLINER float p175_sue_YAWKD_RUDDER_GET(Pack * src)//Serial UDB YAWKD_RUDDER Gain for Rate control of navigation
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  4, 4)));
}
INLINER float p175_sue_ROLLKP_RUDDER_GET(Pack * src)//Serial UDB Extra ROLLKP_RUDDER Gain for Proportional control of roll stabilization
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  8, 4)));
}
INLINER float p175_sue_ROLLKD_RUDDER_GET(Pack * src)//Serial UDB Extra ROLLKD_RUDDER Gain for Rate control of roll stabilization
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  12, 4)));
}
INLINER float p175_sue_RUDDER_BOOST_GET(Pack * src)//SERIAL UDB EXTRA Rudder Boost Gain to Manual Control when stabilized
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  16, 4)));
}
INLINER float p175_sue_RTL_PITCH_DOWN_GET(Pack * src)//Serial UDB Extra Return To Landing - Angle to Pitch Plane Down
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  20, 4)));
}
INLINER float p176_sue_HEIGHT_TARGET_MAX_GET(Pack * src)//Serial UDB Extra HEIGHT_TARGET_MAX
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  0, 4)));
}
INLINER float p176_sue_HEIGHT_TARGET_MIN_GET(Pack * src)//Serial UDB Extra HEIGHT_TARGET_MIN
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  4, 4)));
}
INLINER float p176_sue_ALT_HOLD_THROTTLE_MIN_GET(Pack * src)//Serial UDB Extra ALT_HOLD_THROTTLE_MIN
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  8, 4)));
}
INLINER float p176_sue_ALT_HOLD_THROTTLE_MAX_GET(Pack * src)//Serial UDB Extra ALT_HOLD_THROTTLE_MAX
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  12, 4)));
}
INLINER float p176_sue_ALT_HOLD_PITCH_MIN_GET(Pack * src)//Serial UDB Extra ALT_HOLD_PITCH_MIN
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  16, 4)));
}
INLINER float p176_sue_ALT_HOLD_PITCH_MAX_GET(Pack * src)//Serial UDB Extra ALT_HOLD_PITCH_MAX
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  20, 4)));
}
INLINER float p176_sue_ALT_HOLD_PITCH_HIGH_GET(Pack * src)//Serial UDB Extra ALT_HOLD_PITCH_HIGH
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  24, 4)));
}
INLINER int16_t p177_sue_week_no_GET(Pack * src)//Serial UDB Extra GPS Week Number
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  0, 2)));
}
INLINER int32_t p177_sue_lat_origin_GET(Pack * src)//Serial UDB Extra MP Origin Latitude
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  2, 4)));
}
INLINER int32_t p177_sue_lon_origin_GET(Pack * src)//Serial UDB Extra MP Origin Longitude
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  6, 4)));
}
INLINER int32_t p177_sue_alt_origin_GET(Pack * src)//Serial UDB Extra MP Origin Altitude Above Sea Level
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  10, 4)));
}
INLINER uint32_t p178_sue_TRAP_SOURCE_GET(Pack * src)//Serial UDB Extra Type Program Address of Last Trap
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 4)));
}
INLINER uint8_t p178_sue_WIND_ESTIMATION_GET(Pack * src)//Serial UDB Extra Wind Estimation Enabled
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  4, 1)));
}
INLINER uint8_t p178_sue_GPS_TYPE_GET(Pack * src)//Serial UDB Extra Type of GPS Unit
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  5, 1)));
}
INLINER uint8_t p178_sue_DR_GET(Pack * src)//Serial UDB Extra Dead Reckoning Enabled
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  6, 1)));
}
INLINER uint8_t p178_sue_BOARD_TYPE_GET(Pack * src)//Serial UDB Extra Type of UDB Hardware
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  7, 1)));
}
INLINER uint8_t p178_sue_AIRFRAME_GET(Pack * src)//Serial UDB Extra Type of Airframe
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  8, 1)));
}
INLINER int16_t p178_sue_RCON_GET(Pack * src)//Serial UDB Extra Reboot Register of DSPIC
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  9, 2)));
}
INLINER int16_t p178_sue_TRAP_FLAGS_GET(Pack * src)//Serial UDB Extra  Last dspic Trap Flags
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  11, 2)));
}
INLINER int16_t p178_sue_osc_fail_count_GET(Pack * src)//Serial UDB Extra Number of Ocillator Failures
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  13, 2)));
}
INLINER uint8_t p178_sue_CLOCK_CONFIG_GET(Pack * src)//Serial UDB Extra UDB Internal Clock Configuration
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  15, 1)));
}
INLINER uint8_t p178_sue_FLIGHT_PLAN_TYPE_GET(Pack * src)//Serial UDB Extra Type of Flight Plan
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  16, 1)));
}
INLINER uint8_t* p179_sue_ID_VEHICLE_MODEL_NAME_GET(Pack * src, uint8_t*  dst, int32_t pos) //Serial UDB Extra Model Name Of Vehicle
{
    uint8_t * data = src->data;
    for(int32_t BYTE = 0, dst_max = pos + 40; pos < dst_max ; pos++,  BYTE += 1)
        dst[pos] = ((get_bytes(data,  BYTE, 1)));
    return dst;
}

static const  uint32_t p179_sue_ID_VEHICLE_MODEL_NAME_LEN = 40; //return array length

INLINER  uint8_t*  p179_sue_ID_VEHICLE_MODEL_NAME_GET_(Pack * src) {return p179_sue_ID_VEHICLE_MODEL_NAME_GET(src, malloc(40 * sizeof(uint8_t)), 0);}
INLINER uint8_t* p179_sue_ID_VEHICLE_REGISTRATION_GET(Pack * src, uint8_t*  dst, int32_t pos) //Serial UDB Extra Registraton Number of Vehicle
{
    uint8_t * data = src->data;
    for(int32_t BYTE = 40, dst_max = pos + 20; pos < dst_max ; pos++,  BYTE += 1)
        dst[pos] = ((get_bytes(data,  BYTE, 1)));
    return dst;
}

static const  uint32_t p179_sue_ID_VEHICLE_REGISTRATION_LEN = 20; //return array length

INLINER  uint8_t*  p179_sue_ID_VEHICLE_REGISTRATION_GET_(Pack * src) {return p179_sue_ID_VEHICLE_REGISTRATION_GET(src, malloc(20 * sizeof(uint8_t)), 0);}
INLINER uint8_t* p180_sue_ID_LEAD_PILOT_GET(Pack * src, uint8_t*  dst, int32_t pos) //Serial UDB Extra Name of Expected Lead Pilot
{
    uint8_t * data = src->data;
    for(int32_t BYTE = 0, dst_max = pos + 40; pos < dst_max ; pos++,  BYTE += 1)
        dst[pos] = ((get_bytes(data,  BYTE, 1)));
    return dst;
}

static const  uint32_t p180_sue_ID_LEAD_PILOT_LEN = 40; //return array length

INLINER  uint8_t*  p180_sue_ID_LEAD_PILOT_GET_(Pack * src) {return p180_sue_ID_LEAD_PILOT_GET(src, malloc(40 * sizeof(uint8_t)), 0);}
INLINER uint8_t* p180_sue_ID_DIY_DRONES_URL_GET(Pack * src, uint8_t*  dst, int32_t pos) //Serial UDB Extra URL of Lead Pilot or Team
{
    uint8_t * data = src->data;
    for(int32_t BYTE = 40, dst_max = pos + 70; pos < dst_max ; pos++,  BYTE += 1)
        dst[pos] = ((get_bytes(data,  BYTE, 1)));
    return dst;
}

static const  uint32_t p180_sue_ID_DIY_DRONES_URL_LEN = 70; //return array length

INLINER  uint8_t*  p180_sue_ID_DIY_DRONES_URL_GET_(Pack * src) {return p180_sue_ID_DIY_DRONES_URL_GET(src, malloc(70 * sizeof(uint8_t)), 0);}
INLINER uint32_t p181_time_boot_ms_GET(Pack * src)//Timestamp (milliseconds since system boot)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 4)));
}
INLINER int32_t p181_alt_gps_GET(Pack * src)//GPS altitude in meters, expressed as * 1000 (millimeters), above MSL
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  4, 4)));
}
INLINER int32_t p181_alt_imu_GET(Pack * src)//IMU altitude above ground in meters, expressed as * 1000 (millimeters)
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  8, 4)));
}
INLINER int32_t p181_alt_barometric_GET(Pack * src)//barometeric altitude above ground in meters, expressed as * 1000 (millimeters)
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  12, 4)));
}
INLINER int32_t p181_alt_optical_flow_GET(Pack * src)//Optical flow altitude above ground in meters, expressed as * 1000 (millimeters)
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  16, 4)));
}
INLINER int32_t p181_alt_range_finder_GET(Pack * src)//Rangefinder Altitude above ground in meters, expressed as * 1000 (millimeters)
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  20, 4)));
}
INLINER int32_t p181_alt_extra_GET(Pack * src)//Extra altitude above ground in meters, expressed as * 1000 (millimeters)
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  24, 4)));
}
INLINER uint32_t p182_time_boot_ms_GET(Pack * src)//Timestamp (milliseconds since system boot)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 4)));
}
INLINER int16_t p182_airspeed_imu_GET(Pack * src)//Airspeed estimate from IMU, cm/s
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  4, 2)));
}
INLINER int16_t p182_airspeed_pitot_GET(Pack * src)//Pitot measured forward airpseed, cm/s
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  6, 2)));
}
INLINER int16_t p182_airspeed_hot_wire_GET(Pack * src)//Hot wire anenometer measured airspeed, cm/s
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  8, 2)));
}
INLINER int16_t p182_airspeed_ultrasonic_GET(Pack * src)//Ultrasonic measured airspeed, cm/s
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  10, 2)));
}
INLINER int16_t p182_aoa_GET(Pack * src)//Angle of attack sensor, degrees * 10
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  12, 2)));
}
INLINER int16_t p182_aoy_GET(Pack * src)//Yaw angle sensor, degrees * 10
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  14, 2)));
}
INLINER float p183_sue_feed_forward_GET(Pack * src)//SUE Feed Forward Gain
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  0, 4)));
}
INLINER float p183_sue_turn_rate_nav_GET(Pack * src)//SUE Max Turn Rate when Navigating
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  4, 4)));
}
INLINER float p183_sue_turn_rate_fbw_GET(Pack * src)//SUE Max Turn Rate in Fly By Wire Mode
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  8, 4)));
}
INLINER float p184_angle_of_attack_normal_GET(Pack * src)//SUE Angle of Attack Normal
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  0, 4)));
}
INLINER float p184_angle_of_attack_inverted_GET(Pack * src)//SUE Angle of Attack Inverted
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  4, 4)));
}
INLINER float p184_elevator_trim_normal_GET(Pack * src)//SUE Elevator Trim Normal
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  8, 4)));
}
INLINER float p184_elevator_trim_inverted_GET(Pack * src)//SUE Elevator Trim Inverted
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  12, 4)));
}
INLINER float p184_reference_speed_GET(Pack * src)//SUE reference_speed
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  16, 4)));
}
INLINER uint8_t p185_sue_aileron_output_channel_GET(Pack * src)//SUE aileron output channel
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 1)));
}
INLINER uint8_t p185_sue_aileron_reversed_GET(Pack * src)//SUE aileron reversed
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  1, 1)));
}
INLINER uint8_t p185_sue_elevator_output_channel_GET(Pack * src)//SUE elevator output channel
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 1)));
}
INLINER uint8_t p185_sue_elevator_reversed_GET(Pack * src)//SUE elevator reversed
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  3, 1)));
}
INLINER uint8_t p185_sue_throttle_output_channel_GET(Pack * src)//SUE throttle output channel
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  4, 1)));
}
INLINER uint8_t p185_sue_throttle_reversed_GET(Pack * src)//SUE throttle reversed
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  5, 1)));
}
INLINER uint8_t p185_sue_rudder_output_channel_GET(Pack * src)//SUE rudder output channel
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  6, 1)));
}
INLINER uint8_t p185_sue_rudder_reversed_GET(Pack * src)//SUE rudder reversed
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  7, 1)));
}
INLINER uint8_t p186_sue_number_of_inputs_GET(Pack * src)//SUE Number of Input Channels
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 1)));
}
INLINER int16_t p186_sue_trim_value_input_1_GET(Pack * src)//SUE UDB PWM Trim Value on Input 1
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  1, 2)));
}
INLINER int16_t p186_sue_trim_value_input_2_GET(Pack * src)//SUE UDB PWM Trim Value on Input 2
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  3, 2)));
}
INLINER int16_t p186_sue_trim_value_input_3_GET(Pack * src)//SUE UDB PWM Trim Value on Input 3
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  5, 2)));
}
INLINER int16_t p186_sue_trim_value_input_4_GET(Pack * src)//SUE UDB PWM Trim Value on Input 4
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  7, 2)));
}
INLINER int16_t p186_sue_trim_value_input_5_GET(Pack * src)//SUE UDB PWM Trim Value on Input 5
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  9, 2)));
}
INLINER int16_t p186_sue_trim_value_input_6_GET(Pack * src)//SUE UDB PWM Trim Value on Input 6
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  11, 2)));
}
INLINER int16_t p186_sue_trim_value_input_7_GET(Pack * src)//SUE UDB PWM Trim Value on Input 7
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  13, 2)));
}
INLINER int16_t p186_sue_trim_value_input_8_GET(Pack * src)//SUE UDB PWM Trim Value on Input 8
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  15, 2)));
}
INLINER int16_t p186_sue_trim_value_input_9_GET(Pack * src)//SUE UDB PWM Trim Value on Input 9
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  17, 2)));
}
INLINER int16_t p186_sue_trim_value_input_10_GET(Pack * src)//SUE UDB PWM Trim Value on Input 10
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  19, 2)));
}
INLINER int16_t p186_sue_trim_value_input_11_GET(Pack * src)//SUE UDB PWM Trim Value on Input 11
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  21, 2)));
}
INLINER int16_t p186_sue_trim_value_input_12_GET(Pack * src)//SUE UDB PWM Trim Value on Input 12
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  23, 2)));
}
INLINER int16_t p187_sue_accel_x_offset_GET(Pack * src)//SUE X accelerometer offset
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  0, 2)));
}
INLINER int16_t p187_sue_accel_y_offset_GET(Pack * src)//SUE Y accelerometer offset
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  2, 2)));
}
INLINER int16_t p187_sue_accel_z_offset_GET(Pack * src)//SUE Z accelerometer offset
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  4, 2)));
}
INLINER int16_t p187_sue_gyro_x_offset_GET(Pack * src)//SUE X gyro offset
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  6, 2)));
}
INLINER int16_t p187_sue_gyro_y_offset_GET(Pack * src)//SUE Y gyro offset
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  8, 2)));
}
INLINER int16_t p187_sue_gyro_z_offset_GET(Pack * src)//SUE Z gyro offset
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  10, 2)));
}
INLINER int16_t p188_sue_accel_x_at_calibration_GET(Pack * src)//SUE X accelerometer at calibration time
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  0, 2)));
}
INLINER int16_t p188_sue_accel_y_at_calibration_GET(Pack * src)//SUE Y accelerometer at calibration time
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  2, 2)));
}
INLINER int16_t p188_sue_accel_z_at_calibration_GET(Pack * src)//SUE Z accelerometer at calibration time
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  4, 2)));
}
INLINER int16_t p188_sue_gyro_x_at_calibration_GET(Pack * src)//SUE X gyro at calibration time
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  6, 2)));
}
INLINER int16_t p188_sue_gyro_y_at_calibration_GET(Pack * src)//SUE Y gyro at calibration time
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  8, 2)));
}
INLINER int16_t p188_sue_gyro_z_at_calibration_GET(Pack * src)//SUE Z gyro at calibration time
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  10, 2)));
}
INLINER uint64_t p230_time_usec_GET(Pack * src)//Timestamp (micros since boot or Unix epoch)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 8)));
}
INLINER void p230_time_usec_SET(uint64_t  src, Pack * dst)//Timestamp (micros since boot or Unix epoch)
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  0);
}
INLINER float p230_vel_ratio_GET(Pack * src)//Velocity innovation test ratio
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  8, 4)));
}
INLINER void p230_vel_ratio_SET(float  src, Pack * dst)//Velocity innovation test ratio
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  8);
}
INLINER float p230_pos_horiz_ratio_GET(Pack * src)//Horizontal position innovation test ratio
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  12, 4)));
}
INLINER void p230_pos_horiz_ratio_SET(float  src, Pack * dst)//Horizontal position innovation test ratio
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  12);
}
INLINER float p230_pos_vert_ratio_GET(Pack * src)//Vertical position innovation test ratio
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  16, 4)));
}
INLINER void p230_pos_vert_ratio_SET(float  src, Pack * dst)//Vertical position innovation test ratio
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  16);
}
INLINER float p230_mag_ratio_GET(Pack * src)//Magnetometer innovation test ratio
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  20, 4)));
}
INLINER void p230_mag_ratio_SET(float  src, Pack * dst)//Magnetometer innovation test ratio
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  20);
}
INLINER float p230_hagl_ratio_GET(Pack * src)//Height above terrain innovation test ratio
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  24, 4)));
}
INLINER void p230_hagl_ratio_SET(float  src, Pack * dst)//Height above terrain innovation test ratio
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  24);
}
INLINER float p230_tas_ratio_GET(Pack * src)//True airspeed innovation test ratio
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  28, 4)));
}
INLINER void p230_tas_ratio_SET(float  src, Pack * dst)//True airspeed innovation test ratio
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  28);
}
INLINER float p230_pos_horiz_accuracy_GET(Pack * src)//Horizontal position 1-STD accuracy relative to the EKF local origin (m)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  32, 4)));
}
INLINER void p230_pos_horiz_accuracy_SET(float  src, Pack * dst)//Horizontal position 1-STD accuracy relative to the EKF local origin (m)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  32);
}
INLINER float p230_pos_vert_accuracy_GET(Pack * src)//Vertical position 1-STD accuracy relative to the EKF local origin (m)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  36, 4)));
}
INLINER void p230_pos_vert_accuracy_SET(float  src, Pack * dst)//Vertical position 1-STD accuracy relative to the EKF local origin (m)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  36);
}
INLINER e_ESTIMATOR_STATUS_FLAGS p230_flags_GET(Pack * src)//Integer bitmask indicating which EKF outputs are valid. See definition for ESTIMATOR_STATUS_FLAGS.
{
    uint8_t * data = src->data;
    return  1 + (int)get_bits(data, 320, 11);
}
INLINER void p230_flags_SET(e_ESTIMATOR_STATUS_FLAGS  src, Pack * dst)//Integer bitmask indicating which EKF outputs are valid. See definition for ESTIMATOR_STATUS_FLAGS.
{
    uint8_t * data = dst->data;
    set_bits(- 1 +   src, 11, data, 320);
}
INLINER uint64_t p231_time_usec_GET(Pack * src)//Timestamp (micros since boot or Unix epoch)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 8)));
}
INLINER void p231_time_usec_SET(uint64_t  src, Pack * dst)//Timestamp (micros since boot or Unix epoch)
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  0);
}
INLINER float p231_wind_x_GET(Pack * src)//Wind in X (NED) direction in m/s
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  8, 4)));
}
INLINER void p231_wind_x_SET(float  src, Pack * dst)//Wind in X (NED) direction in m/s
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  8);
}
INLINER float p231_wind_y_GET(Pack * src)//Wind in Y (NED) direction in m/s
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  12, 4)));
}
INLINER void p231_wind_y_SET(float  src, Pack * dst)//Wind in Y (NED) direction in m/s
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  12);
}
INLINER float p231_wind_z_GET(Pack * src)//Wind in Z (NED) direction in m/s
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  16, 4)));
}
INLINER void p231_wind_z_SET(float  src, Pack * dst)//Wind in Z (NED) direction in m/s
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  16);
}
INLINER float p231_var_horiz_GET(Pack * src)//Variability of the wind in XY. RMS of a 1 Hz lowpassed wind estimate.
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  20, 4)));
}
INLINER void p231_var_horiz_SET(float  src, Pack * dst)//Variability of the wind in XY. RMS of a 1 Hz lowpassed wind estimate.
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  20);
}
INLINER float p231_var_vert_GET(Pack * src)//Variability of the wind in Z. RMS of a 1 Hz lowpassed wind estimate.
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  24, 4)));
}
INLINER void p231_var_vert_SET(float  src, Pack * dst)//Variability of the wind in Z. RMS of a 1 Hz lowpassed wind estimate.
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  24);
}
INLINER float p231_wind_alt_GET(Pack * src)//AMSL altitude (m) this measurement was taken at
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  28, 4)));
}
INLINER void p231_wind_alt_SET(float  src, Pack * dst)//AMSL altitude (m) this measurement was taken at
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  28);
}
INLINER float p231_horiz_accuracy_GET(Pack * src)//Horizontal speed 1-STD accuracy
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  32, 4)));
}
INLINER void p231_horiz_accuracy_SET(float  src, Pack * dst)//Horizontal speed 1-STD accuracy
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  32);
}
INLINER float p231_vert_accuracy_GET(Pack * src)//Vertical speed 1-STD accuracy
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  36, 4)));
}
INLINER void p231_vert_accuracy_SET(float  src, Pack * dst)//Vertical speed 1-STD accuracy
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  36);
}
INLINER uint16_t p232_time_week_GET(Pack * src)//GPS week number
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER void p232_time_week_SET(uint16_t  src, Pack * dst)//GPS week number
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER uint32_t p232_time_week_ms_GET(Pack * src)//GPS time (milliseconds from start of GPS week)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 4)));
}
INLINER void p232_time_week_ms_SET(uint32_t  src, Pack * dst)//GPS time (milliseconds from start of GPS week)
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  2);
}
INLINER uint64_t p232_time_usec_GET(Pack * src)//Timestamp (micros since boot or Unix epoch)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  6, 8)));
}
INLINER void p232_time_usec_SET(uint64_t  src, Pack * dst)//Timestamp (micros since boot or Unix epoch)
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  6);
}
INLINER uint8_t p232_gps_id_GET(Pack * src)//ID of the GPS for multiple GPS inputs
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  14, 1)));
}
INLINER void p232_gps_id_SET(uint8_t  src, Pack * dst)//ID of the GPS for multiple GPS inputs
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  14);
}
INLINER uint8_t p232_fix_type_GET(Pack * src)//0-1: no fix, 2: 2D fix, 3: 3D fix. 4: 3D with DGPS. 5: 3D with RTK
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  15, 1)));
}
INLINER void p232_fix_type_SET(uint8_t  src, Pack * dst)//0-1: no fix, 2: 2D fix, 3: 3D fix. 4: 3D with DGPS. 5: 3D with RTK
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  15);
}
INLINER int32_t p232_lat_GET(Pack * src)//Latitude (WGS84), in degrees * 1E7
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  16, 4)));
}
INLINER void p232_lat_SET(int32_t  src, Pack * dst)//Latitude (WGS84), in degrees * 1E7
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  16);
}
INLINER int32_t p232_lon_GET(Pack * src)//Longitude (WGS84), in degrees * 1E7
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  20, 4)));
}
INLINER void p232_lon_SET(int32_t  src, Pack * dst)//Longitude (WGS84), in degrees * 1E7
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  20);
}
INLINER float p232_alt_GET(Pack * src)//Altitude (AMSL, not WGS84), in m (positive for up)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  24, 4)));
}
INLINER void p232_alt_SET(float  src, Pack * dst)//Altitude (AMSL, not WGS84), in m (positive for up)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  24);
}
INLINER float p232_hdop_GET(Pack * src)//GPS HDOP horizontal dilution of position in m
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  28, 4)));
}
INLINER void p232_hdop_SET(float  src, Pack * dst)//GPS HDOP horizontal dilution of position in m
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  28);
}
INLINER float p232_vdop_GET(Pack * src)//GPS VDOP vertical dilution of position in m
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  32, 4)));
}
INLINER void p232_vdop_SET(float  src, Pack * dst)//GPS VDOP vertical dilution of position in m
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  32);
}
INLINER float p232_vn_GET(Pack * src)//GPS velocity in m/s in NORTH direction in earth-fixed NED frame
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  36, 4)));
}
INLINER void p232_vn_SET(float  src, Pack * dst)//GPS velocity in m/s in NORTH direction in earth-fixed NED frame
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  36);
}
INLINER float p232_ve_GET(Pack * src)//GPS velocity in m/s in EAST direction in earth-fixed NED frame
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  40, 4)));
}
INLINER void p232_ve_SET(float  src, Pack * dst)//GPS velocity in m/s in EAST direction in earth-fixed NED frame
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  40);
}
INLINER float p232_vd_GET(Pack * src)//GPS velocity in m/s in DOWN direction in earth-fixed NED frame
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  44, 4)));
}
INLINER void p232_vd_SET(float  src, Pack * dst)//GPS velocity in m/s in DOWN direction in earth-fixed NED frame
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  44);
}
INLINER float p232_speed_accuracy_GET(Pack * src)//GPS speed accuracy in m/s
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  48, 4)));
}
INLINER void p232_speed_accuracy_SET(float  src, Pack * dst)//GPS speed accuracy in m/s
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  48);
}
INLINER float p232_horiz_accuracy_GET(Pack * src)//GPS horizontal accuracy in m
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  52, 4)));
}
INLINER void p232_horiz_accuracy_SET(float  src, Pack * dst)//GPS horizontal accuracy in m
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  52);
}
INLINER float p232_vert_accuracy_GET(Pack * src)//GPS vertical accuracy in m
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  56, 4)));
}
INLINER void p232_vert_accuracy_SET(float  src, Pack * dst)//GPS vertical accuracy in m
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  56);
}
INLINER uint8_t p232_satellites_visible_GET(Pack * src)//Number of satellites visible.
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  60, 1)));
}
INLINER void p232_satellites_visible_SET(uint8_t  src, Pack * dst)//Number of satellites visible.
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  60);
}
INLINER e_GPS_INPUT_IGNORE_FLAGS p232_ignore_flags_GET(Pack * src)//Flags indicating which fields to ignore (see GPS_INPUT_IGNORE_FLAGS enum).  All other fields must be provided
{
    uint8_t * data = src->data;
    return  1 + (int)get_bits(data, 488, 8);
}
INLINER void p232_ignore_flags_SET(e_GPS_INPUT_IGNORE_FLAGS  src, Pack * dst)//Flags indicating which fields to ignore (see GPS_INPUT_IGNORE_FLAGS enum).  All other fields must be provided
{
    uint8_t * data = dst->data;
    set_bits(- 1 +   src, 8, data, 488);
}
/**
*LSB: 1 means message is fragmented, next 2 bits are the fragment ID, the remaining 5 bits are used for
*	the sequence ID. Messages are only to be flushed to the GPS when the entire message has been reconstructed
*	on the autopilot. The fragment ID specifies which order the fragments should be assembled into a buffer,
*	while the sequence ID is used to detect a mismatch between different buffers. The buffer is considered
*	fully reconstructed when either all 4 fragments are present, or all the fragments before the first fragment
*	with a non full payload is received. This management is used to ensure that normal GPS operation doesn't
*	corrupt RTCM data, and to recover from a unreliable transport delivery order*/
INLINER uint8_t p233_flags_GET(Pack * src)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 1)));
}
/**
*LSB: 1 means message is fragmented, next 2 bits are the fragment ID, the remaining 5 bits are used for
*	the sequence ID. Messages are only to be flushed to the GPS when the entire message has been reconstructed
*	on the autopilot. The fragment ID specifies which order the fragments should be assembled into a buffer,
*	while the sequence ID is used to detect a mismatch between different buffers. The buffer is considered
*	fully reconstructed when either all 4 fragments are present, or all the fragments before the first fragment
*	with a non full payload is received. This management is used to ensure that normal GPS operation doesn't
*	corrupt RTCM data, and to recover from a unreliable transport delivery order*/
INLINER void p233_flags_SET(uint8_t  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  0);
}
INLINER uint8_t p233_len_GET(Pack * src)//data length
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  1, 1)));
}
INLINER void p233_len_SET(uint8_t  src, Pack * dst)//data length
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  1);
}
INLINER uint8_t* p233_data__GET(Pack * src, uint8_t*  dst, int32_t pos) //RTCM message (may be fragmented)
{
    uint8_t * data = src->data;
    for(int32_t BYTE = 2, dst_max = pos + 180; pos < dst_max ; pos++,  BYTE += 1)
        dst[pos] = ((get_bytes(data,  BYTE, 1)));
    return dst;
}

static const  uint32_t p233_data__LEN = 180; //return array length

INLINER  uint8_t*  p233_data__GET_(Pack * src) {return p233_data__GET(src, malloc(180 * sizeof(uint8_t)), 0);}
INLINER void p233_data__SET(uint8_t*  src, int32_t pos, Pack * dst) //RTCM message (may be fragmented)
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  2, src_max = pos + 180; pos < src_max; pos++, BYTE += 1)
        set_bytes((src[pos]), 1, data,  BYTE);
}
INLINER uint16_t p234_heading_GET(Pack * src)//heading (centidegrees)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER void p234_heading_SET(uint16_t  src, Pack * dst)//heading (centidegrees)
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER uint16_t p234_wp_distance_GET(Pack * src)//distance to target (meters)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 2)));
}
INLINER void p234_wp_distance_SET(uint16_t  src, Pack * dst)//distance to target (meters)
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  2);
}
INLINER uint32_t p234_custom_mode_GET(Pack * src)//A bitfield for use for autopilot-specific flags.
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  4, 4)));
}
INLINER void p234_custom_mode_SET(uint32_t  src, Pack * dst)//A bitfield for use for autopilot-specific flags.
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  4);
}
INLINER int16_t p234_roll_GET(Pack * src)//roll (centidegrees)
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  8, 2)));
}
INLINER void p234_roll_SET(int16_t  src, Pack * dst)//roll (centidegrees)
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  8);
}
INLINER int16_t p234_pitch_GET(Pack * src)//pitch (centidegrees)
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  10, 2)));
}
INLINER void p234_pitch_SET(int16_t  src, Pack * dst)//pitch (centidegrees)
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  10);
}
INLINER int8_t p234_throttle_GET(Pack * src)//throttle (percentage)
{
    uint8_t * data = src->data;
    return ((int8_t)(get_bytes(data,  12, 1)));
}
INLINER void p234_throttle_SET(int8_t  src, Pack * dst)//throttle (percentage)
{
    uint8_t * data = dst->data;
    set_bytes((uint8_t)(src), 1, data,  12);
}
INLINER int16_t p234_heading_sp_GET(Pack * src)//heading setpoint (centidegrees)
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  13, 2)));
}
INLINER void p234_heading_sp_SET(int16_t  src, Pack * dst)//heading setpoint (centidegrees)
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  13);
}
INLINER int32_t p234_latitude_GET(Pack * src)//Latitude, expressed as degrees * 1E7
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  15, 4)));
}
INLINER void p234_latitude_SET(int32_t  src, Pack * dst)//Latitude, expressed as degrees * 1E7
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  15);
}
INLINER int32_t p234_longitude_GET(Pack * src)//Longitude, expressed as degrees * 1E7
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  19, 4)));
}
INLINER void p234_longitude_SET(int32_t  src, Pack * dst)//Longitude, expressed as degrees * 1E7
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  19);
}
INLINER int16_t p234_altitude_amsl_GET(Pack * src)//Altitude above mean sea level (meters)
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  23, 2)));
}
INLINER void p234_altitude_amsl_SET(int16_t  src, Pack * dst)//Altitude above mean sea level (meters)
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  23);
}
INLINER int16_t p234_altitude_sp_GET(Pack * src)//Altitude setpoint relative to the home position (meters)
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  25, 2)));
}
INLINER void p234_altitude_sp_SET(int16_t  src, Pack * dst)//Altitude setpoint relative to the home position (meters)
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  25);
}
INLINER uint8_t p234_airspeed_GET(Pack * src)//airspeed (m/s)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  27, 1)));
}
INLINER void p234_airspeed_SET(uint8_t  src, Pack * dst)//airspeed (m/s)
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  27);
}
INLINER uint8_t p234_airspeed_sp_GET(Pack * src)//airspeed setpoint (m/s)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  28, 1)));
}
INLINER void p234_airspeed_sp_SET(uint8_t  src, Pack * dst)//airspeed setpoint (m/s)
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  28);
}
INLINER uint8_t p234_groundspeed_GET(Pack * src)//groundspeed (m/s)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  29, 1)));
}
INLINER void p234_groundspeed_SET(uint8_t  src, Pack * dst)//groundspeed (m/s)
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  29);
}
INLINER int8_t p234_climb_rate_GET(Pack * src)//climb rate (m/s)
{
    uint8_t * data = src->data;
    return ((int8_t)(get_bytes(data,  30, 1)));
}
INLINER void p234_climb_rate_SET(int8_t  src, Pack * dst)//climb rate (m/s)
{
    uint8_t * data = dst->data;
    set_bytes((uint8_t)(src), 1, data,  30);
}
INLINER uint8_t p234_gps_nsat_GET(Pack * src)//Number of satellites visible. If unknown, set to 255
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  31, 1)));
}
INLINER void p234_gps_nsat_SET(uint8_t  src, Pack * dst)//Number of satellites visible. If unknown, set to 255
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  31);
}
INLINER uint8_t p234_battery_remaining_GET(Pack * src)//Remaining battery (percentage)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  32, 1)));
}
INLINER void p234_battery_remaining_SET(uint8_t  src, Pack * dst)//Remaining battery (percentage)
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  32);
}
INLINER int8_t p234_temperature_GET(Pack * src)//Autopilot temperature (degrees C)
{
    uint8_t * data = src->data;
    return ((int8_t)(get_bytes(data,  33, 1)));
}
INLINER void p234_temperature_SET(int8_t  src, Pack * dst)//Autopilot temperature (degrees C)
{
    uint8_t * data = dst->data;
    set_bytes((uint8_t)(src), 1, data,  33);
}
INLINER int8_t p234_temperature_air_GET(Pack * src)//Air temperature (degrees C) from airspeed sensor
{
    uint8_t * data = src->data;
    return ((int8_t)(get_bytes(data,  34, 1)));
}
INLINER void p234_temperature_air_SET(int8_t  src, Pack * dst)//Air temperature (degrees C) from airspeed sensor
{
    uint8_t * data = dst->data;
    set_bytes((uint8_t)(src), 1, data,  34);
}
/**
*failsafe (each bit represents a failsafe where 0=ok, 1=failsafe active (bit0:RC, bit1:batt, bit2:GPS,
*	bit3:GCS, bit4:fence*/
INLINER uint8_t p234_failsafe_GET(Pack * src)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  35, 1)));
}
/**
*failsafe (each bit represents a failsafe where 0=ok, 1=failsafe active (bit0:RC, bit1:batt, bit2:GPS,
*	bit3:GCS, bit4:fence*/
INLINER void p234_failsafe_SET(uint8_t  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  35);
}
INLINER uint8_t p234_wp_num_GET(Pack * src)//current waypoint number
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  36, 1)));
}
INLINER void p234_wp_num_SET(uint8_t  src, Pack * dst)//current waypoint number
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  36);
}
INLINER e_MAV_MODE_FLAG p234_base_mode_GET(Pack * src)//System mode bitfield, see MAV_MODE_FLAG ENUM in mavlink/include/mavlink_types.h
{
    uint8_t * data = src->data;
    return  1 + (int)get_bits(data, 296, 8);
}
INLINER void p234_base_mode_SET(e_MAV_MODE_FLAG  src, Pack * dst)//System mode bitfield, see MAV_MODE_FLAG ENUM in mavlink/include/mavlink_types.h
{
    uint8_t * data = dst->data;
    set_bits(- 1 +   src, 8, data, 296);
}
INLINER e_MAV_LANDED_STATE p234_landed_state_GET(Pack * src)//The landed state. Is set to MAV_LANDED_STATE_UNDEFINED if landed state is unknown.
{
    uint8_t * data = src->data;
    return  0 + (int)get_bits(data, 304, 3);
}
INLINER void p234_landed_state_SET(e_MAV_LANDED_STATE  src, Pack * dst)//The landed state. Is set to MAV_LANDED_STATE_UNDEFINED if landed state is unknown.
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 3, data, 304);
}
INLINER e_GPS_FIX_TYPE p234_gps_fix_type_GET(Pack * src)//See the GPS_FIX_TYPE enum.
{
    uint8_t * data = src->data;
    return  0 + (int)get_bits(data, 307, 4);
}
INLINER void p234_gps_fix_type_SET(e_GPS_FIX_TYPE  src, Pack * dst)//See the GPS_FIX_TYPE enum.
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 4, data, 307);
}
INLINER uint32_t p241_clipping_0_GET(Pack * src)//first accelerometer clipping count
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 4)));
}
INLINER void p241_clipping_0_SET(uint32_t  src, Pack * dst)//first accelerometer clipping count
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  0);
}
INLINER uint32_t p241_clipping_1_GET(Pack * src)//second accelerometer clipping count
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  4, 4)));
}
INLINER void p241_clipping_1_SET(uint32_t  src, Pack * dst)//second accelerometer clipping count
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  4);
}
INLINER uint32_t p241_clipping_2_GET(Pack * src)//third accelerometer clipping count
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  8, 4)));
}
INLINER void p241_clipping_2_SET(uint32_t  src, Pack * dst)//third accelerometer clipping count
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  8);
}
INLINER uint64_t p241_time_usec_GET(Pack * src)//Timestamp (micros since boot or Unix epoch)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  12, 8)));
}
INLINER void p241_time_usec_SET(uint64_t  src, Pack * dst)//Timestamp (micros since boot or Unix epoch)
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  12);
}
INLINER float p241_vibration_x_GET(Pack * src)//Vibration levels on X-axis
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  20, 4)));
}
INLINER void p241_vibration_x_SET(float  src, Pack * dst)//Vibration levels on X-axis
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  20);
}
INLINER float p241_vibration_y_GET(Pack * src)//Vibration levels on Y-axis
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  24, 4)));
}
INLINER void p241_vibration_y_SET(float  src, Pack * dst)//Vibration levels on Y-axis
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  24);
}
INLINER float p241_vibration_z_GET(Pack * src)//Vibration levels on Z-axis
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  28, 4)));
}
INLINER void p241_vibration_z_SET(float  src, Pack * dst)//Vibration levels on Z-axis
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  28);
}
INLINER int32_t p242_latitude_GET(Pack * src)//Latitude (WGS84), in degrees * 1E7
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  0, 4)));
}
INLINER void p242_latitude_SET(int32_t  src, Pack * dst)//Latitude (WGS84), in degrees * 1E7
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  0);
}
INLINER int32_t p242_longitude_GET(Pack * src)//Longitude (WGS84, in degrees * 1E7
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  4, 4)));
}
INLINER void p242_longitude_SET(int32_t  src, Pack * dst)//Longitude (WGS84, in degrees * 1E7
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  4);
}
INLINER int32_t p242_altitude_GET(Pack * src)//Altitude (AMSL), in meters * 1000 (positive for up)
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  8, 4)));
}
INLINER void p242_altitude_SET(int32_t  src, Pack * dst)//Altitude (AMSL), in meters * 1000 (positive for up)
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  8);
}
INLINER float p242_x_GET(Pack * src)//Local X position of this position in the local coordinate frame
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  12, 4)));
}
INLINER void p242_x_SET(float  src, Pack * dst)//Local X position of this position in the local coordinate frame
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  12);
}
INLINER float p242_y_GET(Pack * src)//Local Y position of this position in the local coordinate frame
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  16, 4)));
}
INLINER void p242_y_SET(float  src, Pack * dst)//Local Y position of this position in the local coordinate frame
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  16);
}
INLINER float p242_z_GET(Pack * src)//Local Z position of this position in the local coordinate frame
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  20, 4)));
}
INLINER void p242_z_SET(float  src, Pack * dst)//Local Z position of this position in the local coordinate frame
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  20);
}
/**
*World to surface normal and heading transformation of the takeoff position. Used to indicate the heading
*	and slope of the groun*/
INLINER float* p242_q_GET(Pack * src, float*  dst, int32_t pos)
{
    uint8_t * data = src->data;
    for(int32_t BYTE = 24, dst_max = pos + 4; pos < dst_max ; pos++,  BYTE += 4)
        dst[pos] = (intBitsToFloat(get_bytes(data,  BYTE, 4)));
    return dst;
}

static const  uint32_t p242_q_LEN = 4; //return array length
/**
*World to surface normal and heading transformation of the takeoff position. Used to indicate the heading
*	and slope of the groun*/

INLINER  float*  p242_q_GET_(Pack * src) {return p242_q_GET(src, malloc(4 * sizeof(float)), 0);}/**
*World to surface normal and heading transformation of the takeoff position. Used to indicate the heading
*	and slope of the groun*/
INLINER void p242_q_SET(float*  src, int32_t pos, Pack * dst)
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  24, src_max = pos + 4; pos < src_max; pos++, BYTE += 4)
        set_bytes(floatToIntBits(src[pos]), 4, data,  BYTE);
}
/**
*Local X position of the end of the approach vector. Multicopters should set this position based on their
*	takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing
*	fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened
*	from the threshold / touchdown zone*/
INLINER float p242_approach_x_GET(Pack * src)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  40, 4)));
}
/**
*Local X position of the end of the approach vector. Multicopters should set this position based on their
*	takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing
*	fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened
*	from the threshold / touchdown zone*/
INLINER void p242_approach_x_SET(float  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  40);
}
/**
*Local Y position of the end of the approach vector. Multicopters should set this position based on their
*	takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing
*	fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened
*	from the threshold / touchdown zone*/
INLINER float p242_approach_y_GET(Pack * src)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  44, 4)));
}
/**
*Local Y position of the end of the approach vector. Multicopters should set this position based on their
*	takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing
*	fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened
*	from the threshold / touchdown zone*/
INLINER void p242_approach_y_SET(float  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  44);
}
/**
*Local Z position of the end of the approach vector. Multicopters should set this position based on their
*	takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing
*	fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened
*	from the threshold / touchdown zone*/
INLINER float p242_approach_z_GET(Pack * src)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  48, 4)));
}
/**
*Local Z position of the end of the approach vector. Multicopters should set this position based on their
*	takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing
*	fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened
*	from the threshold / touchdown zone*/
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
INLINER void p242_time_usec_SET(uint64_t  src, Bounds_Inside * dst)//Timestamp (microseconds since UNIX epoch or microseconds since system boot)
{
    if(dst->base.field_bit != 416)insert_field(dst, 416, 0);
    uint8_t * data = dst->base.pack->data;
    set_bytes((src), 8, data,  dst->BYTE);
}
INLINER uint8_t p243_target_system_GET(Pack * src)//System ID.
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 1)));
}
INLINER void p243_target_system_SET(uint8_t  src, Pack * dst)//System ID.
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  0);
}
INLINER int32_t p243_latitude_GET(Pack * src)//Latitude (WGS84), in degrees * 1E7
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  1, 4)));
}
INLINER void p243_latitude_SET(int32_t  src, Pack * dst)//Latitude (WGS84), in degrees * 1E7
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  1);
}
INLINER int32_t p243_longitude_GET(Pack * src)//Longitude (WGS84, in degrees * 1E7
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  5, 4)));
}
INLINER void p243_longitude_SET(int32_t  src, Pack * dst)//Longitude (WGS84, in degrees * 1E7
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  5);
}
INLINER int32_t p243_altitude_GET(Pack * src)//Altitude (AMSL), in meters * 1000 (positive for up)
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  9, 4)));
}
INLINER void p243_altitude_SET(int32_t  src, Pack * dst)//Altitude (AMSL), in meters * 1000 (positive for up)
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  9);
}
INLINER float p243_x_GET(Pack * src)//Local X position of this position in the local coordinate frame
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  13, 4)));
}
INLINER void p243_x_SET(float  src, Pack * dst)//Local X position of this position in the local coordinate frame
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  13);
}
INLINER float p243_y_GET(Pack * src)//Local Y position of this position in the local coordinate frame
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  17, 4)));
}
INLINER void p243_y_SET(float  src, Pack * dst)//Local Y position of this position in the local coordinate frame
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  17);
}
INLINER float p243_z_GET(Pack * src)//Local Z position of this position in the local coordinate frame
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  21, 4)));
}
INLINER void p243_z_SET(float  src, Pack * dst)//Local Z position of this position in the local coordinate frame
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  21);
}
/**
*World to surface normal and heading transformation of the takeoff position. Used to indicate the heading
*	and slope of the groun*/
INLINER float* p243_q_GET(Pack * src, float*  dst, int32_t pos)
{
    uint8_t * data = src->data;
    for(int32_t BYTE = 25, dst_max = pos + 4; pos < dst_max ; pos++,  BYTE += 4)
        dst[pos] = (intBitsToFloat(get_bytes(data,  BYTE, 4)));
    return dst;
}

static const  uint32_t p243_q_LEN = 4; //return array length
/**
*World to surface normal and heading transformation of the takeoff position. Used to indicate the heading
*	and slope of the groun*/

INLINER  float*  p243_q_GET_(Pack * src) {return p243_q_GET(src, malloc(4 * sizeof(float)), 0);}/**
*World to surface normal and heading transformation of the takeoff position. Used to indicate the heading
*	and slope of the groun*/
INLINER void p243_q_SET(float*  src, int32_t pos, Pack * dst)
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  25, src_max = pos + 4; pos < src_max; pos++, BYTE += 4)
        set_bytes(floatToIntBits(src[pos]), 4, data,  BYTE);
}
/**
*Local X position of the end of the approach vector. Multicopters should set this position based on their
*	takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing
*	fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened
*	from the threshold / touchdown zone*/
INLINER float p243_approach_x_GET(Pack * src)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  41, 4)));
}
/**
*Local X position of the end of the approach vector. Multicopters should set this position based on their
*	takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing
*	fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened
*	from the threshold / touchdown zone*/
INLINER void p243_approach_x_SET(float  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  41);
}
/**
*Local Y position of the end of the approach vector. Multicopters should set this position based on their
*	takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing
*	fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened
*	from the threshold / touchdown zone*/
INLINER float p243_approach_y_GET(Pack * src)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  45, 4)));
}
/**
*Local Y position of the end of the approach vector. Multicopters should set this position based on their
*	takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing
*	fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened
*	from the threshold / touchdown zone*/
INLINER void p243_approach_y_SET(float  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  45);
}
/**
*Local Z position of the end of the approach vector. Multicopters should set this position based on their
*	takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing
*	fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened
*	from the threshold / touchdown zone*/
INLINER float p243_approach_z_GET(Pack * src)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  49, 4)));
}
/**
*Local Z position of the end of the approach vector. Multicopters should set this position based on their
*	takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing
*	fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened
*	from the threshold / touchdown zone*/
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
INLINER void p243_time_usec_SET(uint64_t  src, Bounds_Inside * dst)//Timestamp (microseconds since UNIX epoch or microseconds since system boot)
{
    if(dst->base.field_bit != 424)insert_field(dst, 424, 0);
    uint8_t * data = dst->base.pack->data;
    set_bytes((src), 8, data,  dst->BYTE);
}
INLINER uint16_t p244_message_id_GET(Pack * src)//The ID of the requested MAVLink message. v1.0 is limited to 254 messages.
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER int32_t p244_interval_us_GET(Pack * src)//0 indicates the interval at which it is sent.
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  2, 4)));
}
INLINER e_MAV_VTOL_STATE p245_vtol_state_GET(Pack * src)//The VTOL state if applicable. Is set to MAV_VTOL_STATE_UNDEFINED if UAV is not in VTOL configuration
{
    uint8_t * data = src->data;
    return  0 + (int)get_bits(data, 0, 3);
}
INLINER e_MAV_LANDED_STATE p245_landed_state_GET(Pack * src)//The landed state. Is set to MAV_LANDED_STATE_UNDEFINED if landed state is unknown.
{
    uint8_t * data = src->data;
    return  0 + (int)get_bits(data, 3, 3);
}
INLINER uint16_t p246_heading_GET(Pack * src)//Course over ground in centidegrees
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER uint16_t p246_hor_velocity_GET(Pack * src)//The horizontal velocity in centimeters/second
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 2)));
}
INLINER uint16_t p246_squawk_GET(Pack * src)//Squawk code
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  4, 2)));
}
INLINER uint32_t p246_ICAO_address_GET(Pack * src)//ICAO address
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  6, 4)));
}
INLINER int32_t p246_lat_GET(Pack * src)//Latitude, expressed as degrees * 1E7
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  10, 4)));
}
INLINER int32_t p246_lon_GET(Pack * src)//Longitude, expressed as degrees * 1E7
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  14, 4)));
}
INLINER int32_t p246_altitude_GET(Pack * src)//Altitude(ASL) in millimeters
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  18, 4)));
}
INLINER int16_t p246_ver_velocity_GET(Pack * src)//The vertical velocity in centimeters/second, positive is up
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  22, 2)));
}
INLINER uint8_t p246_tslc_GET(Pack * src)//Time since last communication in seconds
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  24, 1)));
}
INLINER e_ADSB_ALTITUDE_TYPE p246_altitude_type_GET(Pack * src)//Type from ADSB_ALTITUDE_TYPE enum
{
    uint8_t * data = src->data;
    return  0 + (int)get_bits(data, 200, 1);
}
INLINER e_ADSB_EMITTER_TYPE p246_emitter_type_GET(Pack * src)//Type from ADSB_EMITTER_TYPE enum
{
    uint8_t * data = src->data;
    return  0 + (int)get_bits(data, 201, 5);
}
INLINER e_ADSB_FLAGS p246_flags_GET(Pack * src)//Flags to indicate various statuses including valid data fields
{
    uint8_t * data = src->data;
    return  1 + (int)get_bits(data, 206, 7);
}
INLINER char16_t * p246_callsign_GET(Bounds_Inside * src, char16_t *  dst, int32_t pos) //The callsign, 8+null
{
    uint8_t * data = src->base.pack->data;
    for(int32_t BYTE = src->BYTE, dst_max = pos + src->items; pos < dst_max ; pos++,  BYTE += 2)
        dst[pos] = ((get_bytes(data,  BYTE, 2)));
    return dst;
}
INLINER int32_t p246_callsign_LEN(Bounds_Inside * src)
{
    return (src->base.field_bit !=  213 && !try_visit_field(src, 213)  ||  !try_visit_item(src, 0)) ? 0 : src->items;
}
INLINER  char16_t* p246_callsign_TRY_(Bounds_Inside * src)
{
    if(src->base.field_bit !=  213 && !try_visit_field(src, 213)  ||  !try_visit_item(src, 0)) return NULL;
    char16_t * ret = p246_callsign_GET(src, malloc((src->items + 1) * 2), 0);
    ret[src->items] = 0;//the terminating null character.
    return ret;
}
INLINER uint32_t p247_id_GET(Pack * src)//Unique identifier, domain based on src field
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 4)));
}
INLINER float p247_time_to_minimum_delta_GET(Pack * src)//Estimated time until collision occurs (seconds)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  4, 4)));
}
INLINER float p247_altitude_minimum_delta_GET(Pack * src)//Closest vertical distance in meters between vehicle and object
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  8, 4)));
}
INLINER float p247_horizontal_minimum_delta_GET(Pack * src)//Closest horizontal distance in meteres between vehicle and object
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  12, 4)));
}
INLINER e_MAV_COLLISION_SRC p247_src__GET(Pack * src)//Collision data source
{
    uint8_t * data = src->data;
    return  0 + (int)get_bits(data, 128, 1);
}
INLINER e_MAV_COLLISION_ACTION p247_action_GET(Pack * src)//Action that is being taken to avoid this collision
{
    uint8_t * data = src->data;
    return  0 + (int)get_bits(data, 129, 3);
}
INLINER e_MAV_COLLISION_THREAT_LEVEL p247_threat_level_GET(Pack * src)//How concerned the aircraft is about this collision
{
    uint8_t * data = src->data;
    return  0 + (int)get_bits(data, 132, 2);
}
/**
*A code that identifies the software component that understands this message (analogous to usb device classes
*	or mime type strings).  If this code is less than 32768, it is considered a 'registered' protocol extension
*	and the corresponding entry should be added to https:github.com/mavlink/mavlink/extension-message-ids.xml.
*	 Software creators can register blocks of message IDs as needed (useful for GCS specific metadata, etc...).
*	Message_types greater than 32767 are considered local experiments and should not be checked in to any
*	widely distributed codebase*/
INLINER uint16_t p248_message_type_GET(Pack * src)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER uint8_t p248_target_network_GET(Pack * src)//Network ID (0 for broadcast)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 1)));
}
INLINER uint8_t p248_target_system_GET(Pack * src)//System ID (0 for broadcast)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  3, 1)));
}
INLINER uint8_t p248_target_component_GET(Pack * src)//Component ID (0 for broadcast)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  4, 1)));
}
/**
*Variable length payload. The length is defined by the remaining message length when subtracting the header
*	and other fields.  The entire content of this block is opaque unless you understand any the encoding
*	message_type.  The particular encoding used can be extension specific and might not always be documented
*	as part of the mavlink specification*/
INLINER uint8_t* p248_payload_GET(Pack * src, uint8_t*  dst, int32_t pos)
{
    uint8_t * data = src->data;
    for(int32_t BYTE = 5, dst_max = pos + 249; pos < dst_max ; pos++,  BYTE += 1)
        dst[pos] = ((get_bytes(data,  BYTE, 1)));
    return dst;
}

static const  uint32_t p248_payload_LEN = 249; //return array length
/**
*Variable length payload. The length is defined by the remaining message length when subtracting the header
*	and other fields.  The entire content of this block is opaque unless you understand any the encoding
*	message_type.  The particular encoding used can be extension specific and might not always be documented
*	as part of the mavlink specification*/

INLINER  uint8_t*  p248_payload_GET_(Pack * src) {return p248_payload_GET(src, malloc(249 * sizeof(uint8_t)), 0);}
INLINER uint16_t p249_address_GET(Pack * src)//Starting address of the debug variables
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER uint8_t p249_ver_GET(Pack * src)//Version code of the type variable. 0=unknown, type ignored and assumed int16_t. 1=as below
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 1)));
}
INLINER uint8_t p249_type_GET(Pack * src)//Type code of the memory variables. for ver = 1: 0=16 x int16_t, 1=16 x uint16_t, 2=16 x Q15, 3=16 x 1Q1
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  3, 1)));
}
INLINER int8_t* p249_value_GET(Pack * src, int8_t*  dst, int32_t pos) //Memory contents at specified address
{
    uint8_t * data = src->data;
    for(int32_t BYTE = 4, dst_max = pos + 32; pos < dst_max ; pos++,  BYTE += 1)
        dst[pos] = ((int8_t)(get_bytes(data,  BYTE, 1)));
    return dst;
}

static const  uint32_t p249_value_LEN = 32; //return array length

INLINER  int8_t*  p249_value_GET_(Pack * src) {return p249_value_GET(src, malloc(32 * sizeof(int8_t)), 0);}
INLINER uint64_t p250_time_usec_GET(Pack * src)//Timestamp
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 8)));
}
INLINER float p250_x_GET(Pack * src)//x
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  8, 4)));
}
INLINER float p250_y_GET(Pack * src)//y
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  12, 4)));
}
INLINER float p250_z_GET(Pack * src)//z
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  16, 4)));
}
INLINER char16_t * p250_name_GET(Bounds_Inside * src, char16_t *  dst, int32_t pos) //Name
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
INLINER uint32_t p251_time_boot_ms_GET(Pack * src)//Timestamp (milliseconds since system boot)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 4)));
}
INLINER float p251_value_GET(Pack * src)//Floating point value
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  4, 4)));
}
INLINER char16_t * p251_name_GET(Bounds_Inside * src, char16_t *  dst, int32_t pos) //Name of the debug variable
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
INLINER uint32_t p252_time_boot_ms_GET(Pack * src)//Timestamp (milliseconds since system boot)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 4)));
}
INLINER int32_t p252_value_GET(Pack * src)//Signed integer value
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  4, 4)));
}
INLINER char16_t * p252_name_GET(Bounds_Inside * src, char16_t *  dst, int32_t pos) //Name of the debug variable
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
INLINER e_MAV_SEVERITY p253_severity_GET(Pack * src)//Severity of status. Relies on the definitions within RFC-5424. See enum MAV_SEVERITY.
{
    uint8_t * data = src->data;
    return  0 + (int)get_bits(data, 0, 3);
}
INLINER char16_t * p253_text_GET(Bounds_Inside * src, char16_t *  dst, int32_t pos) //Status text message, without null termination character
{
    uint8_t * data = src->base.pack->data;
    for(int32_t BYTE = src->BYTE, dst_max = pos + src->items; pos < dst_max ; pos++,  BYTE += 2)
        dst[pos] = ((get_bytes(data,  BYTE, 2)));
    return dst;
}
INLINER int32_t p253_text_LEN(Bounds_Inside * src)
{
    return (src->base.field_bit !=  3 && !try_visit_field(src, 3)  ||  !try_visit_item(src, 0)) ? 0 : src->items;
}
INLINER  char16_t* p253_text_TRY_(Bounds_Inside * src)
{
    if(src->base.field_bit !=  3 && !try_visit_field(src, 3)  ||  !try_visit_item(src, 0)) return NULL;
    char16_t * ret = p253_text_GET(src, malloc((src->items + 1) * 2), 0);
    ret[src->items] = 0;//the terminating null character.
    return ret;
}
INLINER uint32_t p254_time_boot_ms_GET(Pack * src)//Timestamp (milliseconds since system boot)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 4)));
}
INLINER uint8_t p254_ind_GET(Pack * src)//index of debug variable
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  4, 1)));
}
INLINER float p254_value_GET(Pack * src)//DEBUG value
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  5, 4)));
}
INLINER uint64_t p256_initial_timestamp_GET(Pack * src)//initial timestamp
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 8)));
}
INLINER uint8_t p256_target_system_GET(Pack * src)//system id of the target
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  8, 1)));
}
INLINER uint8_t p256_target_component_GET(Pack * src)//component ID of the target
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  9, 1)));
}
INLINER uint8_t* p256_secret_key_GET(Pack * src, uint8_t*  dst, int32_t pos) //signing key
{
    uint8_t * data = src->data;
    for(int32_t BYTE = 10, dst_max = pos + 32; pos < dst_max ; pos++,  BYTE += 1)
        dst[pos] = ((get_bytes(data,  BYTE, 1)));
    return dst;
}

static const  uint32_t p256_secret_key_LEN = 32; //return array length

INLINER  uint8_t*  p256_secret_key_GET_(Pack * src) {return p256_secret_key_GET(src, malloc(32 * sizeof(uint8_t)), 0);}
INLINER uint32_t p257_time_boot_ms_GET(Pack * src)//Timestamp (milliseconds since system boot)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 4)));
}
INLINER uint32_t p257_last_change_ms_GET(Pack * src)//Time of last change of button state
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  4, 4)));
}
INLINER uint8_t p257_state_GET(Pack * src)//Bitmap state of buttons
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  8, 1)));
}
INLINER uint8_t p258_target_system_GET(Pack * src)//System ID
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 1)));
}
INLINER uint8_t p258_target_component_GET(Pack * src)//Component ID
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  1, 1)));
}
INLINER char16_t * p258_tune_GET(Bounds_Inside * src, char16_t *  dst, int32_t pos) //tune in board specific format
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
INLINER uint16_t p259_resolution_h_GET(Pack * src)//Image resolution in pixels horizontal
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER uint16_t p259_resolution_v_GET(Pack * src)//Image resolution in pixels vertical
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 2)));
}
INLINER uint16_t p259_cam_definition_version_GET(Pack * src)//Camera definition version (iteration)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  4, 2)));
}
INLINER uint32_t p259_time_boot_ms_GET(Pack * src)//Timestamp (milliseconds since system boot)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  6, 4)));
}
INLINER uint32_t p259_firmware_version_GET(Pack * src)//0xff = Major)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  10, 4)));
}
INLINER uint8_t* p259_vendor_name_GET(Pack * src, uint8_t*  dst, int32_t pos) //Name of the camera vendor
{
    uint8_t * data = src->data;
    for(int32_t BYTE = 14, dst_max = pos + 32; pos < dst_max ; pos++,  BYTE += 1)
        dst[pos] = ((get_bytes(data,  BYTE, 1)));
    return dst;
}

static const  uint32_t p259_vendor_name_LEN = 32; //return array length

INLINER  uint8_t*  p259_vendor_name_GET_(Pack * src) {return p259_vendor_name_GET(src, malloc(32 * sizeof(uint8_t)), 0);}
INLINER uint8_t* p259_model_name_GET(Pack * src, uint8_t*  dst, int32_t pos) //Name of the camera model
{
    uint8_t * data = src->data;
    for(int32_t BYTE = 46, dst_max = pos + 32; pos < dst_max ; pos++,  BYTE += 1)
        dst[pos] = ((get_bytes(data,  BYTE, 1)));
    return dst;
}

static const  uint32_t p259_model_name_LEN = 32; //return array length

INLINER  uint8_t*  p259_model_name_GET_(Pack * src) {return p259_model_name_GET(src, malloc(32 * sizeof(uint8_t)), 0);}
INLINER float p259_focal_length_GET(Pack * src)//Focal length in mm
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  78, 4)));
}
INLINER float p259_sensor_size_h_GET(Pack * src)//Image sensor size horizontal in mm
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  82, 4)));
}
INLINER float p259_sensor_size_v_GET(Pack * src)//Image sensor size vertical in mm
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  86, 4)));
}
INLINER uint8_t p259_lens_id_GET(Pack * src)//Reserved for a lens ID
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  90, 1)));
}
INLINER e_CAMERA_CAP_FLAGS p259_flags_GET(Pack * src)//CAMERA_CAP_FLAGS enum flags (bitmap) describing camera capabilities.
{
    uint8_t * data = src->data;
    return  1 + (int)get_bits(data, 728, 6);
}
INLINER char16_t * p259_cam_definition_uri_GET(Bounds_Inside * src, char16_t *  dst, int32_t pos) //Camera definition URI (if any, otherwise only basic functions will be available).
{
    uint8_t * data = src->base.pack->data;
    for(int32_t BYTE = src->BYTE, dst_max = pos + src->items; pos < dst_max ; pos++,  BYTE += 2)
        dst[pos] = ((get_bytes(data,  BYTE, 2)));
    return dst;
}
INLINER int32_t p259_cam_definition_uri_LEN(Bounds_Inside * src)
{
    return (src->base.field_bit !=  734 && !try_visit_field(src, 734)  ||  !try_visit_item(src, 0)) ? 0 : src->items;
}
INLINER  char16_t* p259_cam_definition_uri_TRY_(Bounds_Inside * src)
{
    if(src->base.field_bit !=  734 && !try_visit_field(src, 734)  ||  !try_visit_item(src, 0)) return NULL;
    char16_t * ret = p259_cam_definition_uri_GET(src, malloc((src->items + 1) * 2), 0);
    ret[src->items] = 0;//the terminating null character.
    return ret;
}
INLINER uint32_t p260_time_boot_ms_GET(Pack * src)//Timestamp (milliseconds since system boot)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 4)));
}
INLINER e_CAMERA_MODE p260_mode_id_GET(Pack * src)//Camera mode (CAMERA_MODE)
{
    uint8_t * data = src->data;
    return  0 + (int)get_bits(data, 32, 2);
}
INLINER uint32_t p261_time_boot_ms_GET(Pack * src)//Timestamp (milliseconds since system boot)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 4)));
}
INLINER uint8_t p261_storage_id_GET(Pack * src)//Storage ID (1 for first, 2 for second, etc.)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  4, 1)));
}
INLINER uint8_t p261_storage_count_GET(Pack * src)//Number of storage devices
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  5, 1)));
}
INLINER uint8_t p261_status_GET(Pack * src)//Status of storage (0 not available, 1 unformatted, 2 formatted)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  6, 1)));
}
INLINER float p261_total_capacity_GET(Pack * src)//Total capacity in MiB
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  7, 4)));
}
INLINER float p261_used_capacity_GET(Pack * src)//Used capacity in MiB
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  11, 4)));
}
INLINER float p261_available_capacity_GET(Pack * src)//Available capacity in MiB
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  15, 4)));
}
INLINER float p261_read_speed_GET(Pack * src)//Read speed in MiB/s
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  19, 4)));
}
INLINER float p261_write_speed_GET(Pack * src)//Write speed in MiB/s
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  23, 4)));
}
INLINER uint32_t p262_time_boot_ms_GET(Pack * src)//Timestamp (milliseconds since system boot)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 4)));
}
INLINER uint32_t p262_recording_time_ms_GET(Pack * src)//Time in milliseconds since recording started
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  4, 4)));
}
/**
*Current status of image capturing (0: idle, 1: capture in progress, 2: interval set but idle, 3: interval
*	set and capture in progress*/
INLINER uint8_t p262_image_status_GET(Pack * src)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  8, 1)));
}
INLINER uint8_t p262_video_status_GET(Pack * src)//Current status of video capturing (0: idle, 1: capture in progress)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  9, 1)));
}
INLINER float p262_image_interval_GET(Pack * src)//Image capture interval in seconds
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  10, 4)));
}
INLINER float p262_available_capacity_GET(Pack * src)//Available storage capacity in MiB
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  14, 4)));
}
INLINER uint32_t p263_time_boot_ms_GET(Pack * src)//Timestamp (milliseconds since system boot)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 4)));
}
INLINER uint64_t p263_time_utc_GET(Pack * src)//Timestamp (microseconds since UNIX epoch) in UTC. 0 for unknown.
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  4, 8)));
}
INLINER uint8_t p263_camera_id_GET(Pack * src)//Camera ID (1 for first, 2 for second, etc.)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  12, 1)));
}
INLINER int32_t p263_lat_GET(Pack * src)//Latitude, expressed as degrees * 1E7 where image was taken
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  13, 4)));
}
INLINER int32_t p263_lon_GET(Pack * src)//Longitude, expressed as degrees * 1E7 where capture was taken
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  17, 4)));
}
INLINER int32_t p263_alt_GET(Pack * src)//Altitude in meters, expressed as * 1E3 (AMSL, not WGS84) where image was taken
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  21, 4)));
}
INLINER int32_t p263_relative_alt_GET(Pack * src)//Altitude above ground in meters, expressed as * 1E3 where image was taken
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  25, 4)));
}
INLINER float* p263_q_GET(Pack * src, float*  dst, int32_t pos) //Quaternion of camera orientation (w, x, y, z order, zero-rotation is 0, 0, 0, 0)
{
    uint8_t * data = src->data;
    for(int32_t BYTE = 29, dst_max = pos + 4; pos < dst_max ; pos++,  BYTE += 4)
        dst[pos] = (intBitsToFloat(get_bytes(data,  BYTE, 4)));
    return dst;
}

static const  uint32_t p263_q_LEN = 4; //return array length

INLINER  float*  p263_q_GET_(Pack * src) {return p263_q_GET(src, malloc(4 * sizeof(float)), 0);}
INLINER int32_t p263_image_index_GET(Pack * src)//Zero based index of this image (image count since armed -1)
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  45, 4)));
}
INLINER int8_t p263_capture_result_GET(Pack * src)//Boolean indicating success (1) or failure (0) while capturing this image.
{
    uint8_t * data = src->data;
    return ((int8_t)(get_bytes(data,  49, 1)));
}
INLINER char16_t * p263_file_url_GET(Bounds_Inside * src, char16_t *  dst, int32_t pos) //URL of image taken. Either local storage or http:foo.jpg if camera provides an HTTP interface.
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
INLINER uint32_t p264_time_boot_ms_GET(Pack * src)//Timestamp (milliseconds since system boot)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 4)));
}
INLINER uint64_t p264_arming_time_utc_GET(Pack * src)//Timestamp at arming (microseconds since UNIX epoch) in UTC, 0 for unknown
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  4, 8)));
}
INLINER uint64_t p264_takeoff_time_utc_GET(Pack * src)//Timestamp at takeoff (microseconds since UNIX epoch) in UTC, 0 for unknown
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  12, 8)));
}
INLINER uint64_t p264_flight_uuid_GET(Pack * src)//Universally unique identifier (UUID) of flight, should correspond to name of logfiles
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  20, 8)));
}
INLINER uint32_t p265_time_boot_ms_GET(Pack * src)//Timestamp (milliseconds since system boot)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 4)));
}
INLINER float p265_roll_GET(Pack * src)//Roll in degrees
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  4, 4)));
}
INLINER float p265_pitch_GET(Pack * src)//Pitch in degrees
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  8, 4)));
}
INLINER float p265_yaw_GET(Pack * src)//Yaw in degrees
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  12, 4)));
}
INLINER uint16_t p266_sequence_GET(Pack * src)//sequence number (can wrap)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER uint8_t p266_target_system_GET(Pack * src)//system ID of the target
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 1)));
}
INLINER uint8_t p266_target_component_GET(Pack * src)//component ID of the target
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  3, 1)));
}
INLINER uint8_t p266_length_GET(Pack * src)//data length
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  4, 1)));
}
/**
*offset into data where first message starts. This can be used for recovery, when a previous message got
*	lost (set to 255 if no start exists)*/
INLINER uint8_t p266_first_message_offset_GET(Pack * src)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  5, 1)));
}
INLINER uint8_t* p266_data__GET(Pack * src, uint8_t*  dst, int32_t pos) //logged data
{
    uint8_t * data = src->data;
    for(int32_t BYTE = 6, dst_max = pos + 249; pos < dst_max ; pos++,  BYTE += 1)
        dst[pos] = ((get_bytes(data,  BYTE, 1)));
    return dst;
}

static const  uint32_t p266_data__LEN = 249; //return array length

INLINER  uint8_t*  p266_data__GET_(Pack * src) {return p266_data__GET(src, malloc(249 * sizeof(uint8_t)), 0);}
INLINER uint16_t p267_sequence_GET(Pack * src)//sequence number (can wrap)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER uint8_t p267_target_system_GET(Pack * src)//system ID of the target
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 1)));
}
INLINER uint8_t p267_target_component_GET(Pack * src)//component ID of the target
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  3, 1)));
}
INLINER uint8_t p267_length_GET(Pack * src)//data length
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  4, 1)));
}
/**
*offset into data where first message starts. This can be used for recovery, when a previous message got
*	lost (set to 255 if no start exists)*/
INLINER uint8_t p267_first_message_offset_GET(Pack * src)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  5, 1)));
}
INLINER uint8_t* p267_data__GET(Pack * src, uint8_t*  dst, int32_t pos) //logged data
{
    uint8_t * data = src->data;
    for(int32_t BYTE = 6, dst_max = pos + 249; pos < dst_max ; pos++,  BYTE += 1)
        dst[pos] = ((get_bytes(data,  BYTE, 1)));
    return dst;
}

static const  uint32_t p267_data__LEN = 249; //return array length

INLINER  uint8_t*  p267_data__GET_(Pack * src) {return p267_data__GET(src, malloc(249 * sizeof(uint8_t)), 0);}
INLINER uint16_t p268_sequence_GET(Pack * src)//sequence number (must match the one in LOGGING_DATA_ACKED)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER uint8_t p268_target_system_GET(Pack * src)//system ID of the target
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 1)));
}
INLINER uint8_t p268_target_component_GET(Pack * src)//component ID of the target
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  3, 1)));
}
INLINER uint16_t p269_resolution_h_GET(Pack * src)//Resolution horizontal in pixels
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER uint16_t p269_resolution_v_GET(Pack * src)//Resolution vertical in pixels
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 2)));
}
INLINER uint16_t p269_rotation_GET(Pack * src)//Video image rotation clockwise
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  4, 2)));
}
INLINER uint32_t p269_bitrate_GET(Pack * src)//Bit rate in bits per second
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  6, 4)));
}
INLINER uint8_t p269_camera_id_GET(Pack * src)//Camera ID (1 for first, 2 for second, etc.)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  10, 1)));
}
INLINER uint8_t p269_status_GET(Pack * src)//Current status of video streaming (0: not running, 1: in progress)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  11, 1)));
}
INLINER float p269_framerate_GET(Pack * src)//Frames per second
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  12, 4)));
}
INLINER char16_t * p269_uri_GET(Bounds_Inside * src, char16_t *  dst, int32_t pos) //Video stream URI
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
INLINER uint16_t p270_resolution_h_GET(Pack * src)//Resolution horizontal in pixels (set to -1 for highest resolution possible)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER uint16_t p270_resolution_v_GET(Pack * src)//Resolution vertical in pixels (set to -1 for highest resolution possible)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 2)));
}
INLINER uint16_t p270_rotation_GET(Pack * src)//Video image rotation clockwise (0-359 degrees)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  4, 2)));
}
INLINER uint32_t p270_bitrate_GET(Pack * src)//Bit rate in bits per second (set to -1 for auto)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  6, 4)));
}
INLINER uint8_t p270_target_system_GET(Pack * src)//system ID of the target
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  10, 1)));
}
INLINER uint8_t p270_target_component_GET(Pack * src)//component ID of the target
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  11, 1)));
}
INLINER uint8_t p270_camera_id_GET(Pack * src)//Camera ID (1 for first, 2 for second, etc.)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  12, 1)));
}
INLINER float p270_framerate_GET(Pack * src)//Frames per second (set to -1 for highest framerate possible)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  13, 4)));
}
INLINER char16_t * p270_uri_GET(Bounds_Inside * src, char16_t *  dst, int32_t pos) //Video stream URI
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
INLINER char16_t * p299_ssid_GET(Bounds_Inside * src, char16_t *  dst, int32_t pos) //Name of Wi-Fi network (SSID). Leave it blank to leave it unchanged.
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
INLINER char16_t * p299_password_GET(Bounds_Inside * src, char16_t *  dst, int32_t pos) //Password. Leave it blank for an open AP.
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
INLINER uint16_t p300_version_GET(Pack * src)//Currently active MAVLink version number * 100: v1.0 is 100, v2.0 is 200, etc.
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER uint16_t p300_min_version_GET(Pack * src)//Minimum MAVLink version supported
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 2)));
}
INLINER uint16_t p300_max_version_GET(Pack * src)//Maximum MAVLink version supported (set to the same value as version by default)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  4, 2)));
}
INLINER uint8_t* p300_spec_version_hash_GET(Pack * src, uint8_t*  dst, int32_t pos) //The first 8 bytes (not characters printed in hex!) of the git hash.
{
    uint8_t * data = src->data;
    for(int32_t BYTE = 6, dst_max = pos + 8; pos < dst_max ; pos++,  BYTE += 1)
        dst[pos] = ((get_bytes(data,  BYTE, 1)));
    return dst;
}

static const  uint32_t p300_spec_version_hash_LEN = 8; //return array length

INLINER  uint8_t*  p300_spec_version_hash_GET_(Pack * src) {return p300_spec_version_hash_GET(src, malloc(8 * sizeof(uint8_t)), 0);}
INLINER uint8_t* p300_library_version_hash_GET(Pack * src, uint8_t*  dst, int32_t pos) //The first 8 bytes (not characters printed in hex!) of the git hash.
{
    uint8_t * data = src->data;
    for(int32_t BYTE = 14, dst_max = pos + 8; pos < dst_max ; pos++,  BYTE += 1)
        dst[pos] = ((get_bytes(data,  BYTE, 1)));
    return dst;
}

static const  uint32_t p300_library_version_hash_LEN = 8; //return array length

INLINER  uint8_t*  p300_library_version_hash_GET_(Pack * src) {return p300_library_version_hash_GET(src, malloc(8 * sizeof(uint8_t)), 0);}
INLINER uint16_t p310_vendor_specific_status_code_GET(Pack * src)//Vendor-specific status information.
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER uint32_t p310_uptime_sec_GET(Pack * src)//The number of seconds since the start-up of the node.
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 4)));
}
INLINER uint64_t p310_time_usec_GET(Pack * src)//Timestamp (microseconds since UNIX epoch or microseconds since system boot)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  6, 8)));
}
INLINER uint8_t p310_sub_mode_GET(Pack * src)//Not used currently.
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  14, 1)));
}
INLINER e_UAVCAN_NODE_HEALTH p310_health_GET(Pack * src)//Generalized node health status.
{
    uint8_t * data = src->data;
    return  0 + (int)get_bits(data, 120, 2);
}
INLINER e_UAVCAN_NODE_MODE p310_mode_GET(Pack * src)//Generalized operating mode.
{
    uint8_t * data = src->data;
    switch(get_bits(data, 122, 3))
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
INLINER uint32_t p311_uptime_sec_GET(Pack * src)//The number of seconds since the start-up of the node.
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 4)));
}
INLINER uint32_t p311_sw_vcs_commit_GET(Pack * src)//Version control system (VCS) revision identifier (e.g. git short commit hash). Zero if unknown.
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  4, 4)));
}
INLINER uint64_t p311_time_usec_GET(Pack * src)//Timestamp (microseconds since UNIX epoch or microseconds since system boot)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  8, 8)));
}
INLINER uint8_t p311_hw_version_major_GET(Pack * src)//Hardware major version number.
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  16, 1)));
}
INLINER uint8_t p311_hw_version_minor_GET(Pack * src)//Hardware minor version number.
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  17, 1)));
}
INLINER uint8_t* p311_hw_unique_id_GET(Pack * src, uint8_t*  dst, int32_t pos) //Hardware unique 128-bit ID.
{
    uint8_t * data = src->data;
    for(int32_t BYTE = 18, dst_max = pos + 16; pos < dst_max ; pos++,  BYTE += 1)
        dst[pos] = ((get_bytes(data,  BYTE, 1)));
    return dst;
}

static const  uint32_t p311_hw_unique_id_LEN = 16; //return array length

INLINER  uint8_t*  p311_hw_unique_id_GET_(Pack * src) {return p311_hw_unique_id_GET(src, malloc(16 * sizeof(uint8_t)), 0);}
INLINER uint8_t p311_sw_version_major_GET(Pack * src)//Software major version number.
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  34, 1)));
}
INLINER uint8_t p311_sw_version_minor_GET(Pack * src)//Software minor version number.
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  35, 1)));
}
INLINER char16_t * p311_name_GET(Bounds_Inside * src, char16_t *  dst, int32_t pos) //Node name string. For example, "sapog.px4.io".
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
INLINER uint8_t p320_target_system_GET(Pack * src)//System ID
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 1)));
}
INLINER uint8_t p320_target_component_GET(Pack * src)//Component ID
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  1, 1)));
}
INLINER int16_t p320_param_index_GET(Pack * src)//Parameter index. Set to -1 to use the Parameter ID field as identifier (else param_id will be ignored
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  2, 2)));
}
/**
*Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination
*	(NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the
*	ID is stored as strin*/
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
INLINER uint8_t p321_target_system_GET(Pack * src)//System ID
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 1)));
}
INLINER uint8_t p321_target_component_GET(Pack * src)//Component ID
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  1, 1)));
}
INLINER uint16_t p322_param_count_GET(Pack * src)//Total number of parameters
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER uint16_t p322_param_index_GET(Pack * src)//Index of this parameter
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 2)));
}
INLINER e_MAV_PARAM_EXT_TYPE p322_param_type_GET(Pack * src)//Parameter type: see the MAV_PARAM_EXT_TYPE enum for supported data types.
{
    uint8_t * data = src->data;
    return  1 + (int)get_bits(data, 32, 4);
}
/**
*Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination
*	(NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the
*	ID is stored as strin*/
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
INLINER char16_t * p322_param_value_GET(Bounds_Inside * src, char16_t *  dst, int32_t pos) //Parameter value
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
INLINER uint8_t p323_target_system_GET(Pack * src)//System ID
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 1)));
}
INLINER uint8_t p323_target_component_GET(Pack * src)//Component ID
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  1, 1)));
}
INLINER e_MAV_PARAM_EXT_TYPE p323_param_type_GET(Pack * src)//Parameter type: see the MAV_PARAM_EXT_TYPE enum for supported data types.
{
    uint8_t * data = src->data;
    return  1 + (int)get_bits(data, 16, 4);
}
/**
*Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination
*	(NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the
*	ID is stored as strin*/
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
INLINER char16_t * p323_param_value_GET(Bounds_Inside * src, char16_t *  dst, int32_t pos) //Parameter value
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
INLINER e_MAV_PARAM_EXT_TYPE p324_param_type_GET(Pack * src)//Parameter type: see the MAV_PARAM_EXT_TYPE enum for supported data types.
{
    uint8_t * data = src->data;
    return  1 + (int)get_bits(data, 0, 4);
}
INLINER e_PARAM_ACK p324_param_result_GET(Pack * src)//Result code: see the PARAM_ACK enum for possible codes.
{
    uint8_t * data = src->data;
    return  0 + (int)get_bits(data, 4, 2);
}
/**
*Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination
*	(NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the
*	ID is stored as strin*/
INLINER char16_t * p324_param_id_GET(Bounds_Inside * src, char16_t *  dst, int32_t pos)
{
    uint8_t * data = src->base.pack->data;
    for(int32_t BYTE = src->BYTE, dst_max = pos + src->items; pos < dst_max ; pos++,  BYTE += 2)
        dst[pos] = ((get_bytes(data,  BYTE, 2)));
    return dst;
}
INLINER int32_t p324_param_id_LEN(Bounds_Inside * src)
{
    return (src->base.field_bit !=  8 && !try_visit_field(src, 8)  ||  !try_visit_item(src, 0)) ? 0 : src->items;
}
INLINER  char16_t* p324_param_id_TRY_(Bounds_Inside * src)
{
    if(src->base.field_bit !=  8 && !try_visit_field(src, 8)  ||  !try_visit_item(src, 0)) return NULL;
    char16_t * ret = p324_param_id_GET(src, malloc((src->items + 1) * 2), 0);
    ret[src->items] = 0;//the terminating null character.
    return ret;
}
INLINER char16_t * p324_param_value_GET(Bounds_Inside * src, char16_t *  dst, int32_t pos) //Parameter value (new value if PARAM_ACK_ACCEPTED, current value otherwise)
{
    uint8_t * data = src->base.pack->data;
    for(int32_t BYTE = src->BYTE, dst_max = pos + src->items; pos < dst_max ; pos++,  BYTE += 2)
        dst[pos] = ((get_bytes(data,  BYTE, 2)));
    return dst;
}
INLINER int32_t p324_param_value_LEN(Bounds_Inside * src)
{
    return (src->base.field_bit !=  9 && !try_visit_field(src, 9)  ||  !try_visit_item(src, 0)) ? 0 : src->items;
}
INLINER  char16_t* p324_param_value_TRY_(Bounds_Inside * src)
{
    if(src->base.field_bit !=  9 && !try_visit_field(src, 9)  ||  !try_visit_item(src, 0)) return NULL;
    char16_t * ret = p324_param_value_GET(src, malloc((src->items + 1) * 2), 0);
    ret[src->items] = 0;//the terminating null character.
    return ret;
}
/**
*Distance of obstacles in front of the sensor starting on the left side. A value of 0 means that the obstacle
*	is right in front of the sensor. A value of max_distance +1 means no obstace is present. A value of UINT16_MAX
*	for unknown/not used. In a array element, each unit corresponds to 1cm*/
INLINER uint16_t* p330_distances_GET(Pack * src, uint16_t*  dst, int32_t pos)
{
    uint8_t * data = src->data;
    for(int32_t BYTE = 0, dst_max = pos + 72; pos < dst_max ; pos++,  BYTE += 2)
        dst[pos] = ((get_bytes(data,  BYTE, 2)));
    return dst;
}

static const  uint32_t p330_distances_LEN = 72; //return array length
/**
*Distance of obstacles in front of the sensor starting on the left side. A value of 0 means that the obstacle
*	is right in front of the sensor. A value of max_distance +1 means no obstace is present. A value of UINT16_MAX
*	for unknown/not used. In a array element, each unit corresponds to 1cm*/

INLINER  uint16_t*  p330_distances_GET_(Pack * src) {return p330_distances_GET(src, malloc(72 * sizeof(uint16_t)), 0);}
INLINER uint16_t p330_min_distance_GET(Pack * src)//Minimum distance the sensor can measure in centimeters
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  144, 2)));
}
INLINER uint16_t p330_max_distance_GET(Pack * src)//Maximum distance the sensor can measure in centimeters
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  146, 2)));
}
INLINER uint64_t p330_time_usec_GET(Pack * src)//Timestamp (microseconds since system boot or since UNIX epoch)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  148, 8)));
}
INLINER uint8_t p330_increment_GET(Pack * src)//Angular width in degrees of each array element.
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  156, 1)));
}
INLINER e_MAV_DISTANCE_SENSOR p330_sensor_type_GET(Pack * src)//Class id of the distance sensor type.
{
    uint8_t * data = src->data;
    return  0 + (int)get_bits(data, 1256, 3);
}

