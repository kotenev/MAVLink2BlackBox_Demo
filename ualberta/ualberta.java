import org.unirail.BlackBox.*;

public class ualberta {}

class CommunicationChannel extends SimpleProtocol implements GroundControl.CommunicationInterface, MicroAirVehicle.CommunicationInterface {}

class GroundControl implements InJAVA, InCS {
	interface CommunicationInterface extends GroundControlHandledPacks, CommonPacks {}
	
	interface GroundControlHandledPacks {
		/**
		 * The heartbeat message shows that a system is present and responding. The type of the MAV and Autopilot
		 * hardware allow the receiving system to treat further messages from this system appropriate (e.g. by laying
		 * out the user interface based on the autopilot)
		 */
		@id(0) class HEARTBEAT {
			MAV_TYPE      type;//Type of the MAV (quadrotor, helicopter, etc., up to 15 types, defined in MAV_TYPE ENUM)
			MAV_AUTOPILOT autopilot;//Autopilot type / class. defined in MAV_AUTOPILOT ENUM
			MAV_MODE_FLAG base_mode;//System mode bitfield, see MAV_MODE_FLAG ENUM in mavlink/include/mavlink_types.h
			@A int custom_mode;//A bitfield for use for autopilot-specific flags.
			MAV_STATE system_status;//System status flag, see MAV_STATE ENUM
			@A byte mavlink_version;//MAVLink version, not writable by user, gets added by protocol because of magic data type: uint8_t_mavlink_versio
		}
		
		/**
		 * The general system state. If the system is following the MAVLink standard, the system state is mainly
		 * defined by three orthogonal states/modes: The system mode, which is either LOCKED (motors shut down and
		 * locked), MANUAL (system under RC control), GUIDED (system with autonomous position control, position
		 * setpoint controlled manually) or AUTO (system guided by path/waypoint planner). The NAV_MODE defined
		 * the current flight state: LIFTOFF (often an open-loop maneuver), LANDING, WAYPOINTS or VECTOR. This represents
		 * the internal navigation state machine. The system status shows whether the system is currently active
		 * or not and if an emergency occured. During the CRITICAL and EMERGENCY states the MAV is still considered
		 * to be active, but should start emergency procedures autonomously. After a failure occured it should first
		 * move from active to critical to allow manual intervention and then move to emergency after a certain
		 * timeout
		 */
		@id(1) class SYS_STATUS {
			/**
			 * Bitmask showing which onboard controllers and sensors are present. Value of 0: not present. Value of 1:
			 * present. Indices defined by ENUM MAV_SYS_STATUS_SENSO
			 */
			MAV_SYS_STATUS_SENSOR onboard_control_sensors_present;
			/**
			 * Bitmask showing which onboard controllers and sensors are enabled:  Value of 0: not enabled. Value of
			 * 1: enabled. Indices defined by ENUM MAV_SYS_STATUS_SENSO
			 */
			MAV_SYS_STATUS_SENSOR onboard_control_sensors_enabled;
			/**
			 * Bitmask showing which onboard controllers and sensors are operational or have an error:  Value of 0: not
			 * enabled. Value of 1: enabled. Indices defined by ENUM MAV_SYS_STATUS_SENSO
			 */
			MAV_SYS_STATUS_SENSOR onboard_control_sensors_health;
			@A short load;//Maximum usage in percent of the mainloop time, (0%: 0, 100%: 1000) should be always below 1000
			@A short voltage_battery;//Battery voltage, in millivolts (1 = 1 millivolt)
			short current_battery;//Battery current, in 10*milliamperes (1 = 10 milliampere), -1: autopilot does not measure the curren
			byte  battery_remaining;//Remaining battery energy: (0%: 0, 100%: 100), -1: autopilot estimate the remaining battery
			/**
			 * Communication drops in percent, (0%: 0, 100%: 10'000), (UART, I2C, SPI, CAN), dropped packets on all links
			 * (packets that were corrupted on reception on the MAV
			 */
			@A short drop_rate_comm;
			/**
			 * Communication errors (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted
			 * on reception on the MAV
			 */
			@A short errors_comm;
			@A short errors_count1;//Autopilot-specific errors
			@A short errors_count2;//Autopilot-specific errors
			@A short errors_count3;//Autopilot-specific errors
			@A short errors_count4;//Autopilot-specific errors
		}
		
		/**
		 * The system time is the time of the master clock, typically the computer clock of the main onboard computer
		 */
		@id(2) class SYSTEM_TIME {
			@A long time_unix_usec;//Timestamp of the master clock in microseconds since UNIX epoch.
			@A int  time_boot_ms;//Timestamp of the component clock since boot time in milliseconds.
		}
		
		/**
		 * A ping message either requesting or responding to a ping. This allows to measure the system latencies,
		 * including serial port, radio modem and UDP connections
		 */
		@id(4) class PING {
			@A long time_usec;//Unix timestamp in microseconds or since system boot if smaller than MAVLink epoch (1.1.2009)
			@A int  seq;//PING sequence
			/**
			 * 0: request ping from all receiving systems, if greater than 0: message is a ping response and number is
			 * the system id of the requesting syste
			 */
			@A byte target_system;
			/**
			 * 0: request ping from all receiving components, if greater than 0: message is a ping response and number
			 * is the system id of the requesting syste
			 */
			@A byte target_component;
		}
		
		/**
		 * Request to control this MAV
		 */
		@id(5) class CHANGE_OPERATOR_CONTROL {
			@A     byte   target_system;//System the GCS requests control for
			@A     byte   control_request;//0: request control of this MAV, 1: Release control of this MAV
			/**
			 * 0: key as plaintext, 1-255: future, different hashing/encryption variants. The GCS should in general use
			 * the safest mode possible initially and then gradually move down the encryption level if it gets a NACK
			 * message indicating an encryption mismatch
			 */
			@A     byte   version;
			/**
			 * Password / Key, depending on version plaintext or encrypted. 25 or less characters, NULL terminated. The
			 * characters may involve A-Z, a-z, 0-9, and "!?,.-
			 */
			@A(25) String passkey;
		}
		
		/**
		 * Accept / deny control of this MAV
		 */
		@id(6) class CHANGE_OPERATOR_CONTROL_ACK {
			@A byte gcs_system_id;//ID of the GCS this message
			@A byte control_request;//0: request control of this MAV, 1: Release control of this MAV
			/**
			 * 0: ACK, 1: NACK: Wrong passkey, 2: NACK: Unsupported passkey encryption method, 3: NACK: Already under
			 * contro
			 */
			@A byte ack;
		}
		
		/**
		 * Emit an encrypted signature / key identifying this system. PLEASE NOTE: This protocol has been kept simple,
		 * so transmitting the key requires an encrypted channel for true safety
		 */
		@id(7) class AUTH_KEY {
			@A(32) String key;//key
		}
		
		/**
		 * THIS INTERFACE IS DEPRECATED. USE COMMAND_LONG with MAV_CMD_DO_SET_MODE INSTEAD. Set the system mode,
		 * as defined by enum MAV_MODE. There is no target component id as the mode is by definition for the overall
		 * aircraft, not only for one component
		 */
		@id(11) class SET_MODE {
			@A byte target_system;//The system setting the mode
			MAV_MODE base_mode;//The new base mode
			@A int custom_mode;//The new autopilot-specific mode. This field can be ignored by an autopilot.
		}
		
		/**
		 * value[float]. This allows to send a parameter to any other component (such as the GCS) without the need
		 * of previous knowledge of possible parameter names. Thus the same GCS can store different parameters for
		 * different autopilots. See also http://qgroundcontrol.org/parameter_interface for a full documentation
		 * of QGroundControl and IMU code
		 */
		@id(20) class PARAM_REQUEST_READ {
			@A     byte   target_system;//System ID
			@A     byte   target_component;//Component ID
			/**
			 * Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT
			 * null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes
			 * storage if the ID is stored as strin
			 */
			@A(16) String param_id;
			short param_index;//Parameter index. Send -1 to use the param ID field as identifier (else the param id will be ignored
		}
		
		/**
		 * Request all parameters of this component. After this request, all parameters are emitted.
		 */
		@id(21) class PARAM_REQUEST_LIST {
			@A byte target_system;//System ID
			@A byte target_component;//Component ID
		}
		
		/**
		 * Emit the value of a onboard parameter. The inclusion of param_count and param_index in the message allows
		 * the recipient to keep track of received parameters and allows him to re-request missing parameters after
		 * a loss or timeout
		 */
		@id(22) class PARAM_VALUE {
			/**
			 * Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT
			 * null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes
			 * storage if the ID is stored as strin
			 */
			@A(16) String param_id;
			float          param_value;//Onboard parameter value
			MAV_PARAM_TYPE param_type;//Onboard parameter type: see the MAV_PARAM_TYPE enum for supported data types.
			@A short param_count;//Total number of onboard parameters
			@A short param_index;//Index of this onboard parameter
		}
		
		/**
		 * Set a parameter value TEMPORARILY to RAM. It will be reset to default on system reboot. Send the ACTION
		 * MAV_ACTION_STORAGE_WRITE to PERMANENTLY write the RAM contents to EEPROM. IMPORTANT: The receiving component
		 * should acknowledge the new parameter value by sending a param_value message to all communication partners.
		 * This will also ensure that multiple GCS all have an up-to-date list of all parameters. If the sending
		 * GCS did not receive a PARAM_VALUE message within its timeout time, it should re-send the PARAM_SET message
		 */
		@id(23) class PARAM_SET {
			@A     byte   target_system;//System ID
			@A     byte   target_component;//Component ID
			/**
			 * Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT
			 * null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes
			 * storage if the ID is stored as strin
			 */
			@A(16) String param_id;
			float          param_value;//Onboard parameter value
			MAV_PARAM_TYPE param_type;//Onboard parameter type: see the MAV_PARAM_TYPE enum for supported data types.
		}
		
		/**
		 * The global position, as returned by the Global Positioning System (GPS). This is
		 * NOT the global position estimate of the system, but rather a RAW sensor value. See message GLOBAL_POSITION for the global position estimate. Coordinate frame is right-handed, Z-axis up (GPS frame).
		 */
		@id(24) class GPS_RAW_INT {
			@A long time_usec;//Timestamp (microseconds since UNIX epoch or microseconds since system boot)
			GPS_FIX_TYPE fix_type;//See the GPS_FIX_TYPE enum.
			int          lat;//Latitude (WGS84, EGM96 ellipsoid), in degrees * 1E7
			int          lon;//Longitude (WGS84, EGM96 ellipsoid), in degrees * 1E7
			/**
			 * Altitude (AMSL, NOT WGS84), in meters * 1000 (positive for up). Note that virtually all GPS modules provide
			 * the AMSL altitude in addition to the WGS84 altitude
			 */
			int          alt;
			@A  short eph;//GPS HDOP horizontal dilution of position (unitless). If unknown, set to: UINT16_MAX
			@A  short epv;//GPS VDOP vertical dilution of position (unitless). If unknown, set to: UINT16_MAX
			@A  short vel;//GPS ground speed (m/s * 100). If unknown, set to: UINT16_MAX
			/**
			 * Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If
			 * unknown, set to: UINT16_MA
			 */
			@A  short cog;
			@A  byte  satellites_visible;//Number of satellites visible. If unknown, set to 255
			@I_ int   alt_ellipsoid;//Altitude (above WGS84, EGM96 ellipsoid), in meters * 1000 (positive for up).
			@A_ int   h_acc;//Position uncertainty in meters * 1000 (positive for up).
			@A_ int   v_acc;//Altitude uncertainty in meters * 1000 (positive for up).
			@A_ int   vel_acc;//Speed uncertainty in meters * 1000 (positive for up).
			@A_ int   hdg_acc;//Heading / track uncertainty in degrees * 1e5.
		}
		
		/**
		 * The positioning status, as reported by GPS. This message is intended to display status information about
		 * each satellite visible to the receiver. See message GLOBAL_POSITION for the global position estimate.
		 * This message can contain information for up to 20 satellites
		 */
		@id(25) class GPS_STATUS {
			@A        byte   satellites_visible;//Number of satellites visible
			@D(20) @A byte[] satellite_prn;//Global satellite ID
			@D(20) @A byte[] satellite_used;//0: Satellite not used, 1: used for localization
			@D(20) @A byte[] satellite_elevation;//Elevation (0: right on top of receiver, 90: on the horizon) of satellite
			@D(20) @A byte[] satellite_azimuth;//Direction of satellite, 0: 0 deg, 255: 360 deg.
			@D(20) @A byte[] satellite_snr;//Signal to noise ratio of satellite
		}
		
		/**
		 * The RAW IMU readings for the usual 9DOF sensor setup. This message should contain the scaled values to
		 * the described unit
		 */
		@id(26) class SCALED_IMU {
			@A int time_boot_ms;//Timestamp (milliseconds since system boot)
			short xacc;//X acceleration (mg)
			short yacc;//Y acceleration (mg)
			short zacc;//Z acceleration (mg)
			short xgyro;//Angular speed around X axis (millirad /sec)
			short ygyro;//Angular speed around Y axis (millirad /sec)
			short zgyro;//Angular speed around Z axis (millirad /sec)
			short xmag;//X Magnetic field (milli tesla)
			short ymag;//Y Magnetic field (milli tesla)
			short zmag;//Z Magnetic field (milli tesla)
		}
		
		/**
		 * The RAW IMU readings for the usual 9DOF sensor setup. This message should always contain the true raw
		 * values without any scaling to allow data capture and system debugging
		 */
		@id(27) class RAW_IMU {
			@A long time_usec;//Timestamp (microseconds since UNIX epoch or microseconds since system boot)
			short xacc;//X acceleration (raw)
			short yacc;//Y acceleration (raw)
			short zacc;//Z acceleration (raw)
			short xgyro;//Angular speed around X axis (raw)
			short ygyro;//Angular speed around Y axis (raw)
			short zgyro;//Angular speed around Z axis (raw)
			short xmag;//X Magnetic field (raw)
			short ymag;//Y Magnetic field (raw)
			short zmag;//Z Magnetic field (raw)
		}
		
		/**
		 * The RAW pressure readings for the typical setup of one absolute pressure and one differential pressure
		 * sensor. The sensor values should be the raw, UNSCALED ADC values
		 */
		@id(28) class RAW_PRESSURE {
			@A long time_usec;//Timestamp (microseconds since UNIX epoch or microseconds since system boot)
			short press_abs;//Absolute pressure (raw)
			short press_diff1;//Differential pressure 1 (raw, 0 if nonexistant)
			short press_diff2;//Differential pressure 2 (raw, 0 if nonexistant)
			short temperature;//Raw Temperature measurement (raw)
		}
		
		/**
		 * The pressure readings for the typical setup of one absolute and differential pressure sensor. The units
		 * are as specified in each field
		 */
		@id(29) class SCALED_PRESSURE {
			@A int time_boot_ms;//Timestamp (milliseconds since system boot)
			float press_abs;//Absolute pressure (hectopascal)
			float press_diff;//Differential pressure 1 (hectopascal)
			short temperature;//Temperature measurement (0.01 degrees celsius)
		}
		
		/**
		 * The attitude in the aeronautical frame (right-handed, Z-down, X-front, Y-right).
		 */
		@id(30) class ATTITUDE {
			@A int time_boot_ms;//Timestamp (milliseconds since system boot)
			float roll;//Roll angle (rad, -pi..+pi)
			float pitch;//Pitch angle (rad, -pi..+pi)
			float yaw;//Yaw angle (rad, -pi..+pi)
			float rollspeed;//Roll angular speed (rad/s)
			float pitchspeed;//Pitch angular speed (rad/s)
			float yawspeed;//Yaw angular speed (rad/s)
		}
		
		/**
		 * The attitude in the aeronautical frame (right-handed, Z-down, X-front, Y-right), expressed as quaternion.
		 * Quaternion order is w, x, y, z and a zero rotation would be expressed as (1 0 0 0)
		 */
		@id(31) class ATTITUDE_QUATERNION {
			@A int time_boot_ms;//Timestamp (milliseconds since system boot)
			float q1;//Quaternion component 1, w (1 in null-rotation)
			float q2;//Quaternion component 2, x (0 in null-rotation)
			float q3;//Quaternion component 3, y (0 in null-rotation)
			float q4;//Quaternion component 4, z (0 in null-rotation)
			float rollspeed;//Roll angular speed (rad/s)
			float pitchspeed;//Pitch angular speed (rad/s)
			float yawspeed;//Yaw angular speed (rad/s)
		}
		
		/**
		 * The filtered local position (e.g. fused computer vision and accelerometers). Coordinate frame is right-handed,
		 * Z-axis down (aeronautical frame, NED / north-east-down convention
		 */
		@id(32) class LOCAL_POSITION_NED {
			@A int time_boot_ms;//Timestamp (milliseconds since system boot)
			float x;//X Position
			float y;//Y Position
			float z;//Z Position
			float vx;//X Speed
			float vy;//Y Speed
			float vz;//Z Speed
		}
		
		/**
		 * nt.
		 */
		@id(33) class GLOBAL_POSITION_INT {
			@A int time_boot_ms;//Timestamp (milliseconds since system boot)
			int   lat;//Latitude, expressed as degrees * 1E7
			int   lon;//Longitude, expressed as degrees * 1E7
			/**
			 * Altitude in meters, expressed as * 1000 (millimeters), AMSL (not WGS84 - note that virtually all GPS modules
			 * provide the AMSL as well
			 */
			int   alt;
			int   relative_alt;//Altitude above ground in meters, expressed as * 1000 (millimeters)
			short vx;//Ground X Speed (Latitude, positive north), expressed as m/s * 100
			short vy;//Ground Y Speed (Longitude, positive east), expressed as m/s * 100
			short vz;//Ground Z Speed (Altitude, positive down), expressed as m/s * 100
			@A short hdg;//Vehicle heading (yaw angle) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
		}
		
		/**
		 * The scaled values of the RC channels received. (-100%) -10000, (0%) 0, (100%) 10000. Channels that are
		 * inactive should be set to UINT16_MAX
		 */
		@id(34) class RC_CHANNELS_SCALED {
			@A int  time_boot_ms;//Timestamp (milliseconds since system boot)
			/**
			 * Servo output port (set of 8 outputs = 1 port). Most MAVs will just use one, but this allows for more than
			 * 8 servos
			 */
			@A byte port;
			short chan1_scaled;//RC channel 1 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
			short chan2_scaled;//RC channel 2 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
			short chan3_scaled;//RC channel 3 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
			short chan4_scaled;//RC channel 4 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
			short chan5_scaled;//RC channel 5 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
			short chan6_scaled;//RC channel 6 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
			short chan7_scaled;//RC channel 7 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
			short chan8_scaled;//RC channel 8 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
			@A byte rssi;//Receive signal strength indicator, 0: 0%, 100: 100%, 255: invalid/unknown.
		}
		
		/**
		 * The RAW values of the RC channels received. The standard PPM modulation is as follows: 1000 microseconds:
		 * 0%, 2000 microseconds: 100%. Individual receivers/transmitters might violate this specification
		 */
		@id(35) class RC_CHANNELS_RAW {
			@A int   time_boot_ms;//Timestamp (milliseconds since system boot)
			/**
			 * Servo output port (set of 8 outputs = 1 port). Most MAVs will just use one, but this allows for more than
			 * 8 servos
			 */
			@A byte  port;
			@A short chan1_raw;//RC channel 1 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
			@A short chan2_raw;//RC channel 2 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
			@A short chan3_raw;//RC channel 3 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
			@A short chan4_raw;//RC channel 4 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
			@A short chan5_raw;//RC channel 5 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
			@A short chan6_raw;//RC channel 6 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
			@A short chan7_raw;//RC channel 7 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
			@A short chan8_raw;//RC channel 8 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
			@A byte  rssi;//Receive signal strength indicator, 0: 0%, 100: 100%, 255: invalid/unknown.
		}
		
		/**
		 * The RAW values of the servo outputs (for RC input from the remote, use the RC_CHANNELS messages). The
		 * standard PPM modulation is as follows: 1000 microseconds: 0%, 2000 microseconds: 100%
		 */
		@id(36) class SERVO_OUTPUT_RAW {
			@A  int   time_usec;//Timestamp (microseconds since system boot)
			/**
			 * Servo output port (set of 8 outputs = 1 port). Most MAVs will just use one, but this allows to encode
			 * more than 8 servos
			 */
			@A  byte  port;
			@A  short servo1_raw;//Servo output 1 value, in microseconds
			@A  short servo2_raw;//Servo output 2 value, in microseconds
			@A  short servo3_raw;//Servo output 3 value, in microseconds
			@A  short servo4_raw;//Servo output 4 value, in microseconds
			@A  short servo5_raw;//Servo output 5 value, in microseconds
			@A  short servo6_raw;//Servo output 6 value, in microseconds
			@A  short servo7_raw;//Servo output 7 value, in microseconds
			@A  short servo8_raw;//Servo output 8 value, in microseconds
			@A_ short servo9_raw;//Servo output 9 value, in microseconds
			@A_ short servo10_raw;//Servo output 10 value, in microseconds
			@A_ short servo11_raw;//Servo output 11 value, in microseconds
			@A_ short servo12_raw;//Servo output 12 value, in microseconds
			@A_ short servo13_raw;//Servo output 13 value, in microseconds
			@A_ short servo14_raw;//Servo output 14 value, in microseconds
			@A_ short servo15_raw;//Servo output 15 value, in microseconds
			@A_ short servo16_raw;//Servo output 16 value, in microseconds
		}
		
		/**
		 * Request a partial list of mission items from the system/component. http://qgroundcontrol.org/mavlink/waypoint_protocol.
		 * If start and end index are the same, just send one waypoint
		 */
		@id(37) class MISSION_REQUEST_PARTIAL_LIST {
			@A byte target_system;//System ID
			@A byte target_component;//Component ID
			short            start_index;//Start index, 0 by default
			short            end_index;//End index, -1 by default (-1: send list to end). Else a valid index of the list
			MAV_MISSION_TYPE mission_type;//Mission type, see MAV_MISSION_TYPE
		}
		
		/**
		 * This message is sent to the MAV to write a partial list. If start index == end index, only one item will
		 * be transmitted / updated. If the start index is NOT 0 and above the current list size, this request should
		 * be REJECTED
		 */
		@id(38) class MISSION_WRITE_PARTIAL_LIST {
			@A byte target_system;//System ID
			@A byte target_component;//Component ID
			short            start_index;//Start index, 0 by default and smaller / equal to the largest index of the current onboard list.
			short            end_index;//End index, equal or greater than start index.
			MAV_MISSION_TYPE mission_type;//Mission type, see MAV_MISSION_TYPE
		}
		
		/**
		 * Message encoding a mission item. This message is emitted to announce
		 * the presence of a mission item and to set a mission item on the system. The mission item can be either in x, y, z meters (type: LOCAL) or x:lat, y:lon, z:altitude. Local frame is Z-down, right handed (NED), global frame is Z-up, right handed (ENU). See also http://qgroundcontrol.org/mavlink/waypoint_protocol.
		 */
		@id(39) class MISSION_ITEM {
			@A byte  target_system;//System ID
			@A byte  target_component;//Component ID
			@A short seq;//Sequence
			MAV_FRAME frame;//The coordinate system of the waypoint. see MAV_FRAME in mavlink_types.h
			MAV_CMD   command;//The scheduled action for the waypoint. see MAV_CMD in common.xml MAVLink specs
			@A byte current;//false:0, true:1
			@A byte autocontinue;//autocontinue to next wp
			float            param1;//PARAM1, see MAV_CMD enum
			float            param2;//PARAM2, see MAV_CMD enum
			float            param3;//PARAM3, see MAV_CMD enum
			float            param4;//PARAM4, see MAV_CMD enum
			float            x;//PARAM5 / local: x position, global: latitude
			float            y;//PARAM6 / y position: global: longitude
			float            z;//PARAM7 / z position: global: altitude (relative or absolute, depending on frame.
			MAV_MISSION_TYPE mission_type;//Mission type, see MAV_MISSION_TYPE
		}
		
		/**
		 * Request the information of the mission item with the sequence number seq. The response of the system to
		 * this message should be a MISSION_ITEM message. http://qgroundcontrol.org/mavlink/waypoint_protoco
		 */
		@id(40) class MISSION_REQUEST {
			@A byte  target_system;//System ID
			@A byte  target_component;//Component ID
			@A short seq;//Sequence
			MAV_MISSION_TYPE mission_type;//Mission type, see MAV_MISSION_TYPE
		}
		
		/**
		 * Set the mission item with sequence number seq as current item. This means that the MAV will continue to
		 * this mission item on the shortest path (not following the mission items in-between)
		 */
		@id(41) class MISSION_SET_CURRENT {
			@A byte  target_system;//System ID
			@A byte  target_component;//Component ID
			@A short seq;//Sequence
		}
		
		/**
		 * Message that announces the sequence number of the current active mission item. The MAV will fly towards
		 * this mission item
		 */
		@id(42) class MISSION_CURRENT {
			@A short seq;//Sequence
		}
		
		/**
		 * Request the overall list of mission items from the system/component.
		 */
		@id(43) class MISSION_REQUEST_LIST {
			@A byte target_system;//System ID
			@A byte target_component;//Component ID
			MAV_MISSION_TYPE mission_type;//Mission type, see MAV_MISSION_TYPE
		}
		
		/**
		 * This message is emitted as response to MISSION_REQUEST_LIST by the MAV and to initiate a write transaction.
		 * The GCS can then request the individual mission item based on the knowledge of the total number of waypoints
		 */
		@id(44) class MISSION_COUNT {
			@A byte  target_system;//System ID
			@A byte  target_component;//Component ID
			@A short count;//Number of mission items in the sequence
			MAV_MISSION_TYPE mission_type;//Mission type, see MAV_MISSION_TYPE
		}
		
		/**
		 * Delete all mission items at once.
		 */
		@id(45) class MISSION_CLEAR_ALL {
			@A byte target_system;//System ID
			@A byte target_component;//Component ID
			MAV_MISSION_TYPE mission_type;//Mission type, see MAV_MISSION_TYPE
		}
		
		/**
		 * A certain mission item has been reached. The system will either hold this position (or circle on the orbit)
		 * or (if the autocontinue on the WP was set) continue to the next waypoint
		 */
		@id(46) class MISSION_ITEM_REACHED {
			@A short seq;//Sequence
		}
		
		/**
		 * Ack message during waypoint handling. The type field states if this message is a positive ack (type=0)
		 * or if an error happened (type=non-zero)
		 */
		@id(47) class MISSION_ACK {
			@A byte target_system;//System ID
			@A byte target_component;//Component ID
			MAV_MISSION_RESULT type;//See MAV_MISSION_RESULT enum
			MAV_MISSION_TYPE   mission_type;//Mission type, see MAV_MISSION_TYPE
		}
		
		/**
		 * As local waypoints exist, the global waypoint reference allows to transform between the local coordinate
		 * frame and the global (GPS) coordinate frame. This can be necessary when e.g. in- and outdoor settings
		 * are connected and the MAV should move from in- to outdoor
		 */
		@id(48) class SET_GPS_GLOBAL_ORIGIN {
			@A byte target_system;//System ID
			int latitude;//Latitude (WGS84), in degrees * 1E7
			int longitude;//Longitude (WGS84, in degrees * 1E7
			int altitude;//Altitude (AMSL), in meters * 1000 (positive for up)
			@A_ long time_usec;//Timestamp (microseconds since UNIX epoch or microseconds since system boot)
		}
		
		/**
		 * Once the MAV sets a new GPS-Local correspondence, this message announces the origin (0,0,0) positio
		 */
		@id(49) class GPS_GLOBAL_ORIGIN {
			int latitude;//Latitude (WGS84), in degrees * 1E7
			int longitude;//Longitude (WGS84), in degrees * 1E7
			int altitude;//Altitude (AMSL), in meters * 1000 (positive for up)
			@A_ long time_usec;//Timestamp (microseconds since UNIX epoch or microseconds since system boot)
		}
		
		/**
		 * Bind a RC channel to a parameter. The parameter should change accoding to the RC channel value.
		 */
		@id(50) class PARAM_MAP_RC {
			@A     byte   target_system;//System ID
			@A     byte   target_component;//Component ID
			/**
			 * Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT
			 * null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes
			 * storage if the ID is stored as strin
			 */
			@A(16) String param_id;
			/**
			 * Parameter index. Send -1 to use the param ID field as identifier (else the param id will be ignored),
			 * send -2 to disable any existing map for this rc_channel_index
			 */
			short param_index;
			/**
			 * Index of parameter RC channel. Not equal to the RC channel id. Typically correpsonds to a potentiometer-knob
			 * on the RC
			 */
			@A byte parameter_rc_channel_index;
			float param_value0;//Initial parameter value
			float scale;//Scale, maps the RC range [-1, 1] to a parameter value
			/**
			 * Minimum param value. The protocol does not define if this overwrites an onboard minimum value. (Depends
			 * on implementation
			 */
			float param_value_min;
			/**
			 * Maximum param value. The protocol does not define if this overwrites an onboard maximum value. (Depends
			 * on implementation
			 */
			float param_value_max;
		}
		
		/**
		 * Request the information of the mission item with the sequence number seq. The response of the system to
		 * this message should be a MISSION_ITEM_INT message. http://qgroundcontrol.org/mavlink/waypoint_protoco
		 */
		@id(51) class MISSION_REQUEST_INT {
			@A byte  target_system;//System ID
			@A byte  target_component;//Component ID
			@A short seq;//Sequence
			MAV_MISSION_TYPE mission_type;//Mission type, see MAV_MISSION_TYPE
		}
		
		/**
		 * Set a safety zone (volume), which is defined by two corners of a cube. This message can be used to tell
		 * the MAV which setpoints/waypoints to accept and which to reject. Safety areas are often enforced by national
		 * or competition regulations
		 */
		@id(54) class SAFETY_SET_ALLOWED_AREA {
			@A byte target_system;//System ID
			@A byte target_component;//Component ID
			/**
			 * Coordinate frame, as defined by MAV_FRAME enum in mavlink_types.h. Can be either global, GPS, right-handed
			 * with Z axis up or local, right handed, Z axis down
			 */
			MAV_FRAME frame;
			float     p1x;//x position 1 / Latitude 1
			float     p1y;//y position 1 / Longitude 1
			float     p1z;//z position 1 / Altitude 1
			float     p2x;//x position 2 / Latitude 2
			float     p2y;//y position 2 / Longitude 2
			float     p2z;//z position 2 / Altitude 2
		}
		
		/**
		 * Read out the safety zone the MAV currently assumes.
		 */
		@id(55) class SAFETY_ALLOWED_AREA {
			/**
			 * Coordinate frame, as defined by MAV_FRAME enum in mavlink_types.h. Can be either global, GPS, right-handed
			 * with Z axis up or local, right handed, Z axis down
			 */
			MAV_FRAME frame;
			float     p1x;//x position 1 / Latitude 1
			float     p1y;//y position 1 / Longitude 1
			float     p1z;//z position 1 / Altitude 1
			float     p2x;//x position 2 / Latitude 2
			float     p2y;//y position 2 / Longitude 2
			float     p2z;//z position 2 / Altitude 2
		}
		
		/**
		 * The attitude in the aeronautical frame (right-handed, Z-down, X-front, Y-right), expressed as quaternion.
		 * Quaternion order is w, x, y, z and a zero rotation would be expressed as (1 0 0 0)
		 */
		@id(61) class ATTITUDE_QUATERNION_COV {
			@A    long    time_usec;//Timestamp (microseconds since system boot or since UNIX epoch)
			@D(4) float[] q;//Quaternion components, w, x, y, z (1 0 0 0 is the null-rotation)
			float rollspeed;//Roll angular speed (rad/s)
			float pitchspeed;//Pitch angular speed (rad/s)
			float yawspeed;//Yaw angular speed (rad/s)
			@D(9) float[] covariance;//Attitude covariance
		}
		
		/**
		 * The state of the fixed wing navigation and position controller.
		 */
		@id(62) class NAV_CONTROLLER_OUTPUT {
			float nav_roll;//Current desired roll in degrees
			float nav_pitch;//Current desired pitch in degrees
			short nav_bearing;//Current desired heading in degrees
			short target_bearing;//Bearing to current waypoint/target in degrees
			@A short wp_dist;//Distance to active waypoint in meters
			float alt_error;//Current altitude error in meters
			float aspd_error;//Current airspeed error in meters/second
			float xtrack_error;//Current crosstrack error on x-y plane in meters
		}
		
		/**
		 * The filtered global position (e.g. fused GPS and accelerometers). The position is in GPS-frame (right-handed,
		 * Z-up). It  is designed as scaled integer message since the resolution of float is not sufficient. NOTE:
		 * This message is intended for onboard networks / companion computers and higher-bandwidth links and optimized
		 * for accuracy and completeness. Please use the GLOBAL_POSITION_INT message for a minimal subset
		 */
		@id(63) class GLOBAL_POSITION_INT_COV {
			@A long time_usec;//Timestamp (microseconds since system boot or since UNIX epoch)
			MAV_ESTIMATOR_TYPE estimator_type;//Class id of the estimator this estimate originated from.
			int                lat;//Latitude, expressed as degrees * 1E7
			int                lon;//Longitude, expressed as degrees * 1E7
			int                alt;//Altitude in meters, expressed as * 1000 (millimeters), above MSL
			int                relative_alt;//Altitude above ground in meters, expressed as * 1000 (millimeters)
			float              vx;//Ground X Speed (Latitude), expressed as m/s
			float              vy;//Ground Y Speed (Longitude), expressed as m/s
			float              vz;//Ground Z Speed (Altitude), expressed as m/s
			@D(36) float[] covariance;//Covariance matrix (first six entries are the first ROW, next six entries are the second row, etc.)
		}
		
		/**
		 * The filtered local position (e.g. fused computer vision and accelerometers). Coordinate frame is right-handed,
		 * Z-axis down (aeronautical frame, NED / north-east-down convention
		 */
		@id(64) class LOCAL_POSITION_NED_COV {
			@A long time_usec;//Timestamp (microseconds since system boot or since UNIX epoch)
			MAV_ESTIMATOR_TYPE estimator_type;//Class id of the estimator this estimate originated from.
			float              x;//X Position
			float              y;//Y Position
			float              z;//Z Position
			float              vx;//X Speed (m/s)
			float              vy;//Y Speed (m/s)
			float              vz;//Z Speed (m/s)
			float              ax;//X Acceleration (m/s^2)
			float              ay;//Y Acceleration (m/s^2)
			float              az;//Z Acceleration (m/s^2)
			/**
			 * Covariance matrix upper right triangular (first nine entries are the first ROW, next eight entries are
			 * the second row, etc.
			 */
			@D(45) float[] covariance;
		}
		
		/**
		 * The PPM values of the RC channels received. The standard PPM modulation is as follows: 1000 microseconds:
		 * 0%, 2000 microseconds: 100%. Individual receivers/transmitters might violate this specification
		 */
		@id(65) class RC_CHANNELS {
			@A int   time_boot_ms;//Timestamp (milliseconds since system boot)
			/**
			 * Total number of RC channels being received. This can be larger than 18, indicating that more channels
			 * are available but not given in this message. This value should be 0 when no RC channels are available
			 */
			@A byte  chancount;
			@A short chan1_raw;//RC channel 1 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
			@A short chan2_raw;//RC channel 2 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
			@A short chan3_raw;//RC channel 3 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
			@A short chan4_raw;//RC channel 4 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
			@A short chan5_raw;//RC channel 5 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
			@A short chan6_raw;//RC channel 6 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
			@A short chan7_raw;//RC channel 7 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
			@A short chan8_raw;//RC channel 8 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
			@A short chan9_raw;//RC channel 9 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
			@A short chan10_raw;//RC channel 10 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
			@A short chan11_raw;//RC channel 11 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
			@A short chan12_raw;//RC channel 12 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
			@A short chan13_raw;//RC channel 13 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
			@A short chan14_raw;//RC channel 14 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
			@A short chan15_raw;//RC channel 15 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
			@A short chan16_raw;//RC channel 16 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
			@A short chan17_raw;//RC channel 17 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
			@A short chan18_raw;//RC channel 18 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
			@A byte  rssi;//Receive signal strength indicator, 0: 0%, 100: 100%, 255: invalid/unknown.
		}
		
		/**
		 * THIS INTERFACE IS DEPRECATED. USE SET_MESSAGE_INTERVAL INSTEAD.
		 */
		@id(66) class REQUEST_DATA_STREAM {
			@A byte  target_system;//The target requested to send the message stream.
			@A byte  target_component;//The target requested to send the message stream.
			@A byte  req_stream_id;//The ID of the requested data stream
			@A short req_message_rate;//The requested message rate
			@A byte  start_stop;//1 to start sending, 0 to stop sending.
		}
		
		/**
		 * THIS INTERFACE IS DEPRECATED. USE MESSAGE_INTERVAL INSTEAD.
		 */
		@id(67) class DATA_STREAM {
			@A byte  stream_id;//The ID of the requested data stream
			@A short message_rate;//The message rate
			@A byte  on_off;//1 stream is enabled, 0 stream is stopped.
		}
		
		/**
		 * This message provides an API for manually controlling the vehicle using standard joystick axes nomenclature,
		 * along with a joystick-like input device. Unused axes can be disabled an buttons are also transmit as
		 * boolean values of their
		 */
		@id(69) class MANUAL_CONTROL {
			@A byte target;//The system to be controlled.
			/**
			 * X-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid.
			 * Generally corresponds to forward(1000)-backward(-1000) movement on a joystick and the pitch of a vehicle
			 */
			short x;
			/**
			 * Y-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid.
			 * Generally corresponds to left(-1000)-right(1000) movement on a joystick and the roll of a vehicle
			 */
			short y;
			/**
			 * Z-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid.
			 * Generally corresponds to a separate slider movement with maximum being 1000 and minimum being -1000 on
			 * a joystick and the thrust of a vehicle. Positive values are positive thrust, negative values are negative
			 * thrust
			 */
			short z;
			/**
			 * R-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid.
			 * Generally corresponds to a twisting of the joystick, with counter-clockwise being 1000 and clockwise
			 * being -1000, and the yaw of a vehicle
			 */
			short r;
			/**
			 * A bitfield corresponding to the joystick buttons' current state, 1 for pressed, 0 for released. The lowest
			 * bit corresponds to Button 1
			 */
			@A short buttons;
		}
		
		/**
		 * The RAW values of the RC channels sent to the MAV to override info received from the RC radio. A value
		 * of UINT16_MAX means no change to that channel. A value of 0 means control of that channel should be released
		 * back to the RC radio. The standard PPM modulation is as follows: 1000 microseconds: 0%, 2000 microseconds:
		 * 100%. Individual receivers/transmitters might violate this specification
		 */
		@id(70) class RC_CHANNELS_OVERRIDE {
			@A byte  target_system;//System ID
			@A byte  target_component;//Component ID
			@A short chan1_raw;//RC channel 1 value, in microseconds. A value of UINT16_MAX means to ignore this field.
			@A short chan2_raw;//RC channel 2 value, in microseconds. A value of UINT16_MAX means to ignore this field.
			@A short chan3_raw;//RC channel 3 value, in microseconds. A value of UINT16_MAX means to ignore this field.
			@A short chan4_raw;//RC channel 4 value, in microseconds. A value of UINT16_MAX means to ignore this field.
			@A short chan5_raw;//RC channel 5 value, in microseconds. A value of UINT16_MAX means to ignore this field.
			@A short chan6_raw;//RC channel 6 value, in microseconds. A value of UINT16_MAX means to ignore this field.
			@A short chan7_raw;//RC channel 7 value, in microseconds. A value of UINT16_MAX means to ignore this field.
			@A short chan8_raw;//RC channel 8 value, in microseconds. A value of UINT16_MAX means to ignore this field.
		}
		
		/**
		 * Message encoding a mission item. This message is emitted to announce
		 * the presence of a mission item and to set a mission item on the system. The mission item can be either in x, y, z meters (type: LOCAL) or x:lat, y:lon, z:altitude. Local frame is Z-down, right handed (NED), global frame is Z-up, right handed (ENU). See alsohttp://qgroundcontrol.org/mavlink/waypoint_protocol.
		 */
		@id(73) class MISSION_ITEM_INT {
			@A byte  target_system;//System ID
			@A byte  target_component;//Component ID
			/**
			 * Waypoint ID (sequence number). Starts at zero. Increases monotonically for each waypoint, no gaps in the
			 * sequence (0,1,2,3,4)
			 */
			@A short seq;
			MAV_FRAME frame;//The coordinate system of the waypoint. see MAV_FRAME in mavlink_types.h
			MAV_CMD   command;//The scheduled action for the waypoint. see MAV_CMD in common.xml MAVLink specs
			@A byte current;//false:0, true:1
			@A byte autocontinue;//autocontinue to next wp
			float            param1;//PARAM1, see MAV_CMD enum
			float            param2;//PARAM2, see MAV_CMD enum
			float            param3;//PARAM3, see MAV_CMD enum
			float            param4;//PARAM4, see MAV_CMD enum
			int              x;//PARAM5 / local: x position in meters * 1e4, global: latitude in degrees * 10^7
			int              y;//PARAM6 / y position: local: x position in meters * 1e4, global: longitude in degrees *10^7
			float            z;//PARAM7 / z position: global: altitude in meters (relative or absolute, depending on frame.
			MAV_MISSION_TYPE mission_type;//Mission type, see MAV_MISSION_TYPE
		}
		
		/**
		 * Metrics typically displayed on a HUD for fixed wing aircraft
		 */
		@id(74) class VFR_HUD {
			float airspeed;//Current airspeed in m/s
			float groundspeed;//Current ground speed in m/s
			short heading;//Current heading in degrees, in compass units (0..360, 0=north)
			@A short throttle;//Current throttle setting in integer percent, 0 to 100
			float alt;//Current altitude (MSL), in meters
			float climb;//Current climb rate in meters/second
		}
	}
	
	interface CommonPacks {
		/**
		 * Message encoding a command with parameters as scaled integers. Scaling depends on the actual command value
		 */
		@id(75) class COMMAND_INT {
			@A byte target_system;//System ID
			@A byte target_component;//Component ID
			MAV_FRAME frame;//The coordinate system of the COMMAND. see MAV_FRAME in mavlink_types.h
			MAV_CMD   command;//The scheduled action for the mission item. see MAV_CMD in common.xml MAVLink specs
			@A byte current;//false:0, true:1
			@A byte autocontinue;//autocontinue to next wp
			float param1;//PARAM1, see MAV_CMD enum
			float param2;//PARAM2, see MAV_CMD enum
			float param3;//PARAM3, see MAV_CMD enum
			float param4;//PARAM4, see MAV_CMD enum
			int   x;//PARAM5 / local: x position in meters * 1e4, global: latitude in degrees * 10^7
			int   y;//PARAM6 / local: y position in meters * 1e4, global: longitude in degrees * 10^7
			float z;//PARAM7 / z position: global: altitude in meters (relative or absolute, depending on frame.
		}
		
		/**
		 * Send a command with up to seven parameters to the MAV
		 */
		@id(76) class COMMAND_LONG {
			@A byte target_system;//System which should execute the command
			@A byte target_component;//Component which should execute the command, 0 for all components
			MAV_CMD command;//Command ID, as defined by MAV_CMD enum.
			@A byte confirmation;//0: First transmission of this command. 1-255: Confirmation transmissions (e.g. for kill command)
			float param1;//Parameter 1, as defined by MAV_CMD enum.
			float param2;//Parameter 2, as defined by MAV_CMD enum.
			float param3;//Parameter 3, as defined by MAV_CMD enum.
			float param4;//Parameter 4, as defined by MAV_CMD enum.
			float param5;//Parameter 5, as defined by MAV_CMD enum.
			float param6;//Parameter 6, as defined by MAV_CMD enum.
			float param7;//Parameter 7, as defined by MAV_CMD enum.
		}
		
		/**
		 * Report status of a command. Includes feedback whether the command was executed.
		 */
		@id(77) class COMMAND_ACK {
			MAV_CMD    command;//Command ID, as defined by MAV_CMD enum.
			MAV_RESULT result;//See MAV_RESULT enum
			/**
			 * WIP: Also used as result_param1, it can be set with a enum containing the errors reasons of why the command
			 * was denied or the progress percentage or 255 if unknown the progress when result is MAV_RESULT_IN_PROGRESS
			 */
			@A_ byte progress;
			/**
			 * WIP: Additional parameter of the result, example: which parameter of MAV_CMD_NAV_WAYPOINT caused it to
			 * be denied
			 */
			@I_ int  result_param2;
			@A_ byte target_system;//WIP: System which requested the command to be executed
			@A_ byte target_component;//WIP: Component which requested the command to be executed
		}
		
		/**
		 * Setpoint in roll, pitch, yaw and thrust from the operator
		 */
		@id(81) class MANUAL_SETPOINT {
			@A int time_boot_ms;//Timestamp in milliseconds since system boot
			float roll;//Desired roll rate in radians per second
			float pitch;//Desired pitch rate in radians per second
			float yaw;//Desired yaw rate in radians per second
			float thrust;//Collective thrust, normalized to 0 .. 1
			@A byte mode_switch;//Flight mode switch position, 0.. 255
			@A byte manual_override_switch;//Override mode switch position, 0.. 255
		}
		
		/**
		 * Sets a desired vehicle attitude. Used by an external controller to command the vehicle (manual controller
		 * or other system)
		 */
		@id(82) class SET_ATTITUDE_TARGET {
			@A    int     time_boot_ms;//Timestamp in milliseconds since system boot
			@A    byte    target_system;//System ID
			@A    byte    target_component;//Component ID
			/**
			 * Mappings: If any of these bits are set, the corresponding input should be ignored: bit 1: body roll rate,
			 * bit 2: body pitch rate, bit 3: body yaw rate. bit 4-bit 6: reserved, bit 7: throttle, bit 8: attitud
			 */
			@A    byte    type_mask;
			@D(4) float[] q;//Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
			float body_roll_rate;//Body roll rate in radians per second
			float body_pitch_rate;//Body roll rate in radians per second
			float body_yaw_rate;//Body roll rate in radians per second
			float thrust;//Collective thrust, normalized to 0 .. 1 (-1 .. 1 for vehicles capable of reverse trust)
		}
		
		/**
		 * Reports the current commanded attitude of the vehicle as specified by the autopilot. This should match
		 * the commands sent in a SET_ATTITUDE_TARGET message if the vehicle is being controlled this way
		 */
		@id(83) class ATTITUDE_TARGET {
			@A    int     time_boot_ms;//Timestamp in milliseconds since system boot
			/**
			 * Mappings: If any of these bits are set, the corresponding input should be ignored: bit 1: body roll rate,
			 * bit 2: body pitch rate, bit 3: body yaw rate. bit 4-bit 7: reserved, bit 8: attitud
			 */
			@A    byte    type_mask;
			@D(4) float[] q;//Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
			float body_roll_rate;//Body roll rate in radians per second
			float body_pitch_rate;//Body pitch rate in radians per second
			float body_yaw_rate;//Body yaw rate in radians per second
			float thrust;//Collective thrust, normalized to 0 .. 1 (-1 .. 1 for vehicles capable of reverse trust)
		}
		
		/**
		 * Sets a desired vehicle position in a local north-east-down coordinate frame. Used by an external controller
		 * to command the vehicle (manual controller or other system)
		 */
		@id(84) class SET_POSITION_TARGET_LOCAL_NED {
			@A int  time_boot_ms;//Timestamp in milliseconds since system boot
			@A byte target_system;//System ID
			@A byte target_component;//Component ID
			/**
			 * Valid options are: MAV_FRAME_LOCAL_NED = 1, MAV_FRAME_LOCAL_OFFSET_NED = 7, MAV_FRAME_BODY_NED = 8, MAV_FRAME_BODY_OFFSET_NED
			 * =
			 */
			MAV_FRAME coordinate_frame;
			/**
			 * Bitmask to indicate which dimensions should be ignored by the vehicle: a value of 0b0000000000000000 or
			 * 0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. If bit 10 is set
			 * the floats afx afy afz should be interpreted as force instead of acceleration. Mapping: bit 1: x, bit
			 * 2: y, bit 3: z, bit 4: vx, bit 5: vy, bit 6: vz, bit 7: ax, bit 8: ay, bit 9: az, bit 10: is force setpoint,
			 * bit 11: yaw, bit 12: yaw rat
			 */
			@A short type_mask;
			float x;//X Position in NED frame in meters
			float y;//Y Position in NED frame in meters
			float z;//Z Position in NED frame in meters (note, altitude is negative in NED)
			float vx;//X velocity in NED frame in meter / s
			float vy;//Y velocity in NED frame in meter / s
			float vz;//Z velocity in NED frame in meter / s
			float afx;//X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
			float afy;//Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
			float afz;//Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
			float yaw;//yaw setpoint in rad
			float yaw_rate;//yaw rate setpoint in rad/s
		}
		
		/**
		 * Reports the current commanded vehicle position, velocity, and acceleration as specified by the autopilot.
		 * This should match the commands sent in SET_POSITION_TARGET_LOCAL_NED if the vehicle is being controlled
		 * this way
		 */
		@id(85) class POSITION_TARGET_LOCAL_NED {
			@A int time_boot_ms;//Timestamp in milliseconds since system boot
			/**
			 * Valid options are: MAV_FRAME_LOCAL_NED = 1, MAV_FRAME_LOCAL_OFFSET_NED = 7, MAV_FRAME_BODY_NED = 8, MAV_FRAME_BODY_OFFSET_NED
			 * =
			 */
			MAV_FRAME coordinate_frame;
			/**
			 * Bitmask to indicate which dimensions should be ignored by the vehicle: a value of 0b0000000000000000 or
			 * 0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. If bit 10 is set
			 * the floats afx afy afz should be interpreted as force instead of acceleration. Mapping: bit 1: x, bit
			 * 2: y, bit 3: z, bit 4: vx, bit 5: vy, bit 6: vz, bit 7: ax, bit 8: ay, bit 9: az, bit 10: is force setpoint,
			 * bit 11: yaw, bit 12: yaw rat
			 */
			@A short type_mask;
			float x;//X Position in NED frame in meters
			float y;//Y Position in NED frame in meters
			float z;//Z Position in NED frame in meters (note, altitude is negative in NED)
			float vx;//X velocity in NED frame in meter / s
			float vy;//Y velocity in NED frame in meter / s
			float vz;//Z velocity in NED frame in meter / s
			float afx;//X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
			float afy;//Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
			float afz;//Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
			float yaw;//yaw setpoint in rad
			float yaw_rate;//yaw rate setpoint in rad/s
		}
		
		/**
		 * Sets a desired vehicle position, velocity, and/or acceleration in a global coordinate system (WGS84).
		 * Used by an external controller to command the vehicle (manual controller or other system)
		 */
		@id(86) class SET_POSITION_TARGET_GLOBAL_INT {
			/**
			 * Timestamp in milliseconds since system boot. The rationale for the timestamp in the setpoint is to allow
			 * the system to compensate for the transport delay of the setpoint. This allows the system to compensate
			 * processing latency
			 */
			@A int  time_boot_ms;
			@A byte target_system;//System ID
			@A byte target_component;//Component ID
			/**
			 * Valid options are: MAV_FRAME_GLOBAL_INT = 5, MAV_FRAME_GLOBAL_RELATIVE_ALT_INT = 6, MAV_FRAME_GLOBAL_TERRAIN_ALT_INT
			 * = 1
			 */
			MAV_FRAME coordinate_frame;
			/**
			 * Bitmask to indicate which dimensions should be ignored by the vehicle: a value of 0b0000000000000000 or
			 * 0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. If bit 10 is set
			 * the floats afx afy afz should be interpreted as force instead of acceleration. Mapping: bit 1: x, bit
			 * 2: y, bit 3: z, bit 4: vx, bit 5: vy, bit 6: vz, bit 7: ax, bit 8: ay, bit 9: az, bit 10: is force setpoint,
			 * bit 11: yaw, bit 12: yaw rat
			 */
			@A short type_mask;
			int   lat_int;//X Position in WGS84 frame in 1e7 * meters
			int   lon_int;//Y Position in WGS84 frame in 1e7 * meters
			float alt;//Altitude in meters in AMSL altitude, not WGS84 if absolute or relative, above terrain if GLOBAL_TERRAIN_ALT_IN
			float vx;//X velocity in NED frame in meter / s
			float vy;//Y velocity in NED frame in meter / s
			float vz;//Z velocity in NED frame in meter / s
			float afx;//X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
			float afy;//Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
			float afz;//Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
			float yaw;//yaw setpoint in rad
			float yaw_rate;//yaw rate setpoint in rad/s
		}
		
		/**
		 * Reports the current commanded vehicle position, velocity, and acceleration as specified by the autopilot.
		 * This should match the commands sent in SET_POSITION_TARGET_GLOBAL_INT if the vehicle is being controlled
		 * this way
		 */
		@id(87) class POSITION_TARGET_GLOBAL_INT {
			/**
			 * Timestamp in milliseconds since system boot. The rationale for the timestamp in the setpoint is to allow
			 * the system to compensate for the transport delay of the setpoint. This allows the system to compensate
			 * processing latency
			 */
			@A int time_boot_ms;
			/**
			 * Valid options are: MAV_FRAME_GLOBAL_INT = 5, MAV_FRAME_GLOBAL_RELATIVE_ALT_INT = 6, MAV_FRAME_GLOBAL_TERRAIN_ALT_INT
			 * = 1
			 */
			MAV_FRAME coordinate_frame;
			/**
			 * Bitmask to indicate which dimensions should be ignored by the vehicle: a value of 0b0000000000000000 or
			 * 0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. If bit 10 is set
			 * the floats afx afy afz should be interpreted as force instead of acceleration. Mapping: bit 1: x, bit
			 * 2: y, bit 3: z, bit 4: vx, bit 5: vy, bit 6: vz, bit 7: ax, bit 8: ay, bit 9: az, bit 10: is force setpoint,
			 * bit 11: yaw, bit 12: yaw rat
			 */
			@A short type_mask;
			int   lat_int;//X Position in WGS84 frame in 1e7 * meters
			int   lon_int;//Y Position in WGS84 frame in 1e7 * meters
			float alt;//Altitude in meters in AMSL altitude, not WGS84 if absolute or relative, above terrain if GLOBAL_TERRAIN_ALT_IN
			float vx;//X velocity in NED frame in meter / s
			float vy;//Y velocity in NED frame in meter / s
			float vz;//Z velocity in NED frame in meter / s
			float afx;//X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
			float afy;//Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
			float afz;//Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
			float yaw;//yaw setpoint in rad
			float yaw_rate;//yaw rate setpoint in rad/s
		}
		
		/**
		 * The offset in X, Y, Z and yaw between the LOCAL_POSITION_NED messages of MAV X and the global coordinate
		 * frame in NED coordinates. Coordinate frame is right-handed, Z-axis down (aeronautical frame, NED / north-east-down
		 * convention
		 */
		@id(89) class LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET {
			@A int time_boot_ms;//Timestamp (milliseconds since system boot)
			float x;//X Position
			float y;//Y Position
			float z;//Z Position
			float roll;//Roll
			float pitch;//Pitch
			float yaw;//Yaw
		}
		
		/**
		 * DEPRECATED PACKET! Suffers from missing airspeed fields and singularities due to Euler angles. Please
		 * use HIL_STATE_QUATERNION instead. Sent from simulation to autopilot. This packet is useful for high throughput
		 * applications such as hardware in the loop simulations
		 */
		@id(90) class HIL_STATE {
			@A long time_usec;//Timestamp (microseconds since UNIX epoch or microseconds since system boot)
			float roll;//Roll angle (rad)
			float pitch;//Pitch angle (rad)
			float yaw;//Yaw angle (rad)
			float rollspeed;//Body frame roll / phi angular speed (rad/s)
			float pitchspeed;//Body frame pitch / theta angular speed (rad/s)
			float yawspeed;//Body frame yaw / psi angular speed (rad/s)
			int   lat;//Latitude, expressed as * 1E7
			int   lon;//Longitude, expressed as * 1E7
			int   alt;//Altitude in meters, expressed as * 1000 (millimeters)
			short vx;//Ground X Speed (Latitude), expressed as m/s * 100
			short vy;//Ground Y Speed (Longitude), expressed as m/s * 100
			short vz;//Ground Z Speed (Altitude), expressed as m/s * 100
			short xacc;//X acceleration (mg)
			short yacc;//Y acceleration (mg)
			short zacc;//Z acceleration (mg)
		}
		
		/**
		 * Sent from autopilot to simulation. Hardware in the loop control outputs
		 */
		@id(91) class HIL_CONTROLS {
			@A long time_usec;//Timestamp (microseconds since UNIX epoch or microseconds since system boot)
			float    roll_ailerons;//Control output -1 .. 1
			float    pitch_elevator;//Control output -1 .. 1
			float    yaw_rudder;//Control output -1 .. 1
			float    throttle;//Throttle 0 .. 1
			float    aux1;//Aux 1, -1 .. 1
			float    aux2;//Aux 2, -1 .. 1
			float    aux3;//Aux 3, -1 .. 1
			float    aux4;//Aux 4, -1 .. 1
			MAV_MODE mode;//System mode (MAV_MODE)
			@A byte nav_mode;//Navigation mode (MAV_NAV_MODE)
		}
		
		/**
		 * Sent from simulation to autopilot. The RAW values of the RC channels received. The standard PPM modulation
		 * is as follows: 1000 microseconds: 0%, 2000 microseconds: 100%. Individual receivers/transmitters might
		 * violate this specification
		 */
		@id(92) class HIL_RC_INPUTS_RAW {
			@A long  time_usec;//Timestamp (microseconds since UNIX epoch or microseconds since system boot)
			@A short chan1_raw;//RC channel 1 value, in microseconds
			@A short chan2_raw;//RC channel 2 value, in microseconds
			@A short chan3_raw;//RC channel 3 value, in microseconds
			@A short chan4_raw;//RC channel 4 value, in microseconds
			@A short chan5_raw;//RC channel 5 value, in microseconds
			@A short chan6_raw;//RC channel 6 value, in microseconds
			@A short chan7_raw;//RC channel 7 value, in microseconds
			@A short chan8_raw;//RC channel 8 value, in microseconds
			@A short chan9_raw;//RC channel 9 value, in microseconds
			@A short chan10_raw;//RC channel 10 value, in microseconds
			@A short chan11_raw;//RC channel 11 value, in microseconds
			@A short chan12_raw;//RC channel 12 value, in microseconds
			@A byte  rssi;//Receive signal strength indicator, 0: 0%, 255: 100%
		}
		
		/**
		 * Sent from autopilot to simulation. Hardware in the loop control outputs (replacement for HIL_CONTROLS
		 */
		@id(93) class HIL_ACTUATOR_CONTROLS {
			@A     long    time_usec;//Timestamp (microseconds since UNIX epoch or microseconds since system boot)
			@D(16) float[] controls;//Control outputs -1 .. 1. Channel assignment depends on the simulated hardware.
			MAV_MODE mode;//System mode (MAV_MODE), includes arming state.
			@A long flags;//Flags as bitfield, reserved for future use.
		}
		
		/**
		 * Optical flow from a flow sensor (e.g. optical mouse sensor)
		 */
		@id(100) class OPTICAL_FLOW {
			@A long time_usec;//Timestamp (UNIX)
			@A byte sensor_id;//Sensor ID
			short flow_x;//Flow in pixels * 10 in x-sensor direction (dezi-pixels)
			short flow_y;//Flow in pixels * 10 in y-sensor direction (dezi-pixels)
			float flow_comp_m_x;//Flow in meters in x-sensor direction, angular-speed compensated
			float flow_comp_m_y;//Flow in meters in y-sensor direction, angular-speed compensated
			@A byte quality;//Optical flow quality / confidence. 0: bad, 255: maximum quality
			float ground_distance;//Ground distance in meters. Positive value: distance known. Negative value: Unknown distance
			@I_ float flow_rate_x;//Flow rate in radians/second about X axis
			@I_ float flow_rate_y;//Flow rate in radians/second about Y axis
		}
		
		@id(101) class GLOBAL_VISION_POSITION_ESTIMATE {
			@A long usec;//Timestamp (microseconds, synced to UNIX time or since system boot)
			float x;//Global X position
			float y;//Global Y position
			float z;//Global Z position
			float roll;//Roll angle in rad
			float pitch;//Pitch angle in rad
			float yaw;//Yaw angle in rad
		}
		
		@id(102) class VISION_POSITION_ESTIMATE {
			@A long usec;//Timestamp (microseconds, synced to UNIX time or since system boot)
			float x;//Global X position
			float y;//Global Y position
			float z;//Global Z position
			float roll;//Roll angle in rad
			float pitch;//Pitch angle in rad
			float yaw;//Yaw angle in rad
		}
		
		@id(103) class VISION_SPEED_ESTIMATE {
			@A long usec;//Timestamp (microseconds, synced to UNIX time or since system boot)
			float x;//Global X speed
			float y;//Global Y speed
			float z;//Global Z speed
		}
		
		@id(104) class VICON_POSITION_ESTIMATE {
			@A long usec;//Timestamp (microseconds, synced to UNIX time or since system boot)
			float x;//Global X position
			float y;//Global Y position
			float z;//Global Z position
			float roll;//Roll angle in rad
			float pitch;//Pitch angle in rad
			float yaw;//Yaw angle in rad
		}
		
		/**
		 * The IMU readings in SI units in NED body frame
		 */
		@id(105) class HIGHRES_IMU {
			@A long time_usec;//Timestamp (microseconds, synced to UNIX time or since system boot)
			float xacc;//X acceleration (m/s^2)
			float yacc;//Y acceleration (m/s^2)
			float zacc;//Z acceleration (m/s^2)
			float xgyro;//Angular speed around X axis (rad / sec)
			float ygyro;//Angular speed around Y axis (rad / sec)
			float zgyro;//Angular speed around Z axis (rad / sec)
			float xmag;//X Magnetic field (Gauss)
			float ymag;//Y Magnetic field (Gauss)
			float zmag;//Z Magnetic field (Gauss)
			float abs_pressure;//Absolute pressure in millibar
			float diff_pressure;//Differential pressure in millibar
			float pressure_alt;//Altitude calculated from pressure
			float temperature;//Temperature in degrees celsius
			@A short fields_updated;//Bitmask for fields that have updated since last message, bit 0 = xacc, bit 12: temperature
		}
		
		/**
		 * Optical flow from an angular rate flow sensor (e.g. PX4FLOW or mouse sensor)
		 */
		@id(106) class OPTICAL_FLOW_RAD {
			@A long time_usec;//Timestamp (microseconds, synced to UNIX time or since system boot)
			@A byte sensor_id;//Sensor ID
			/**
			 * Integration time in microseconds. Divide integrated_x and integrated_y by the integration time to obtain
			 * average flow. The integration time also indicates the
			 */
			@A int  integration_time_us;
			/**
			 * Flow in radians around X axis (Sensor RH rotation about the X axis induces a positive flow. Sensor linear
			 * motion along the positive Y axis induces a negative flow.
			 */
			float integrated_x;
			/**
			 * Flow in radians around Y axis (Sensor RH rotation about the Y axis induces a positive flow. Sensor linear
			 * motion along the positive X axis induces a positive flow.
			 */
			float integrated_y;
			float integrated_xgyro;//RH rotation around X axis (rad)
			float integrated_ygyro;//RH rotation around Y axis (rad)
			float integrated_zgyro;//RH rotation around Z axis (rad)
			short temperature;//Temperature * 100 in centi-degrees Celsius
			@A byte quality;//Optical flow quality / confidence. 0: no valid flow, 255: maximum quality
			@A int  time_delta_distance_us;//Time in microseconds since the distance was sampled.
			/**
			 * Distance to the center of the flow field in meters. Positive value (including zero): distance known. Negative
			 * value: Unknown distance
			 */
			float distance;
		}
		
		/**
		 * The IMU readings in SI units in NED body frame
		 */
		@id(107) class HIL_SENSOR {
			@A long time_usec;//Timestamp (microseconds, synced to UNIX time or since system boot)
			float xacc;//X acceleration (m/s^2)
			float yacc;//Y acceleration (m/s^2)
			float zacc;//Z acceleration (m/s^2)
			float xgyro;//Angular speed around X axis in body frame (rad / sec)
			float ygyro;//Angular speed around Y axis in body frame (rad / sec)
			float zgyro;//Angular speed around Z axis in body frame (rad / sec)
			float xmag;//X Magnetic field (Gauss)
			float ymag;//Y Magnetic field (Gauss)
			float zmag;//Z Magnetic field (Gauss)
			float abs_pressure;//Absolute pressure in millibar
			float diff_pressure;//Differential pressure (airspeed) in millibar
			float pressure_alt;//Altitude calculated from pressure
			float temperature;//Temperature in degrees celsius
			/**
			 * Bitmask for fields that have updated since last message, bit 0 = xacc, bit 12: temperature, bit 31: full
			 * reset of attitude/position/velocities/etc was performed in sim
			 */
			@A int fields_updated;
		}
		
		/**
		 * Status of simulation environment, if used
		 */
		@id(108) class SIM_STATE {
			float q1;//True attitude quaternion component 1, w (1 in null-rotation)
			float q2;//True attitude quaternion component 2, x (0 in null-rotation)
			float q3;//True attitude quaternion component 3, y (0 in null-rotation)
			float q4;//True attitude quaternion component 4, z (0 in null-rotation)
			float roll;//Attitude roll expressed as Euler angles, not recommended except for human-readable outputs
			float pitch;//Attitude pitch expressed as Euler angles, not recommended except for human-readable outputs
			float yaw;//Attitude yaw expressed as Euler angles, not recommended except for human-readable outputs
			float xacc;//X acceleration m/s/s
			float yacc;//Y acceleration m/s/s
			float zacc;//Z acceleration m/s/s
			float xgyro;//Angular speed around X axis rad/s
			float ygyro;//Angular speed around Y axis rad/s
			float zgyro;//Angular speed around Z axis rad/s
			float lat;//Latitude in degrees
			float lon;//Longitude in degrees
			float alt;//Altitude in meters
			float std_dev_horz;//Horizontal position standard deviation
			float std_dev_vert;//Vertical position standard deviation
			float vn;//True velocity in m/s in NORTH direction in earth-fixed NED frame
			float ve;//True velocity in m/s in EAST direction in earth-fixed NED frame
			float vd;//True velocity in m/s in DOWN direction in earth-fixed NED frame
		}
		
		/**
		 * Status generated by radio and injected into MAVLink stream.
		 */
		@id(109) class RADIO_STATUS {
			@A byte  rssi;//Local signal strength
			@A byte  remrssi;//Remote signal strength
			@A byte  txbuf;//Remaining free buffer space in percent.
			@A byte  noise;//Background noise level
			@A byte  remnoise;//Remote background noise level
			@A short rxerrors;//Receive errors
			@A short fixed;//Count of error corrected packets
		}
		
		/**
		 * File transfer message
		 */
		@id(110) class FILE_TRANSFER_PROTOCOL {
			@A         byte   target_network;//Network ID (0 for broadcast)
			@A         byte   target_system;//System ID (0 for broadcast)
			@A         byte   target_component;//Component ID (0 for broadcast)
			/**
			 * Variable length payload. The length is defined by the remaining message length when subtracting the header
			 * and other fields.  The entire content of this block is opaque unless you understand any the encoding
			 * message_type.  The particular encoding used can be extension specific and might not always be documented
			 * as part of the mavlink specification
			 */
			@D(251) @A byte[] payload;
		}
		
		/**
		 * Time synchronization message.
		 */
		@id(111) class TIMESYNC {
			long tc1;//Time sync timestamp 1
			long ts1;//Time sync timestamp 2
		}
		
		/**
		 * Camera-IMU triggering and synchronisation message.
		 */
		@id(112) class CAMERA_TRIGGER {
			@A long time_usec;//Timestamp for the image frame in microseconds
			@A int  seq;//Image frame sequence
		}
		
		/**
		 * The global position, as returned by the Global Positioning System (GPS). This is
		 * NOT the global position estimate of the sytem, but rather a RAW sensor value. See message GLOBAL_POSITION for the global position estimate. Coordinate frame is right-handed, Z-axis up (GPS frame).
		 */
		@id(113) class HIL_GPS {
			@A long time_usec;//Timestamp (microseconds since UNIX epoch or microseconds since system boot)
			/**
			 * 0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value of this field unless it is
			 * at least two, so always correctly fill in the fix
			 */
			@A byte fix_type;
			int lat;//Latitude (WGS84), in degrees * 1E7
			int lon;//Longitude (WGS84), in degrees * 1E7
			int alt;//Altitude (AMSL, not WGS84), in meters * 1000 (positive for up)
			@A short eph;//GPS HDOP horizontal dilution of position in cm (m*100). If unknown, set to: 65535
			@A short epv;//GPS VDOP vertical dilution of position in cm (m*100). If unknown, set to: 65535
			@A short vel;//GPS ground speed in cm/s. If unknown, set to: 65535
			short vn;//GPS velocity in cm/s in NORTH direction in earth-fixed NED frame
			short ve;//GPS velocity in cm/s in EAST direction in earth-fixed NED frame
			short vd;//GPS velocity in cm/s in DOWN direction in earth-fixed NED frame
			/**
			 * Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If
			 * unknown, set to: 6553
			 */
			@A short cog;
			@A byte  satellites_visible;//Number of satellites visible. If unknown, set to 255
		}
		
		/**
		 * Simulated optical flow from a flow sensor (e.g. PX4FLOW or optical mouse sensor)
		 */
		@id(114) class HIL_OPTICAL_FLOW {
			@A long time_usec;//Timestamp (microseconds, synced to UNIX time or since system boot)
			@A byte sensor_id;//Sensor ID
			/**
			 * Integration time in microseconds. Divide integrated_x and integrated_y by the integration time to obtain
			 * average flow. The integration time also indicates the
			 */
			@A int  integration_time_us;
			/**
			 * Flow in radians around X axis (Sensor RH rotation about the X axis induces a positive flow. Sensor linear
			 * motion along the positive Y axis induces a negative flow.
			 */
			float integrated_x;
			/**
			 * Flow in radians around Y axis (Sensor RH rotation about the Y axis induces a positive flow. Sensor linear
			 * motion along the positive X axis induces a positive flow.
			 */
			float integrated_y;
			float integrated_xgyro;//RH rotation around X axis (rad)
			float integrated_ygyro;//RH rotation around Y axis (rad)
			float integrated_zgyro;//RH rotation around Z axis (rad)
			short temperature;//Temperature * 100 in centi-degrees Celsius
			@A byte quality;//Optical flow quality / confidence. 0: no valid flow, 255: maximum quality
			@A int  time_delta_distance_us;//Time in microseconds since the distance was sampled.
			/**
			 * Distance to the center of the flow field in meters. Positive value (including zero): distance known. Negative
			 * value: Unknown distance
			 */
			float distance;
		}
		
		/**
		 * Sent from simulation to autopilot, avoids in contrast to HIL_STATE singularities. This packet is useful
		 * for high throughput applications such as hardware in the loop simulations
		 */
		@id(115) class HIL_STATE_QUATERNION {
			@A    long    time_usec;//Timestamp (microseconds since UNIX epoch or microseconds since system boot)
			@D(4) float[] attitude_quaternion;//Vehicle attitude expressed as normalized quaternion in w, x, y, z order (with 1 0 0 0 being the null-rotation
			float rollspeed;//Body frame roll / phi angular speed (rad/s)
			float pitchspeed;//Body frame pitch / theta angular speed (rad/s)
			float yawspeed;//Body frame yaw / psi angular speed (rad/s)
			int   lat;//Latitude, expressed as * 1E7
			int   lon;//Longitude, expressed as * 1E7
			int   alt;//Altitude in meters, expressed as * 1000 (millimeters)
			short vx;//Ground X Speed (Latitude), expressed as cm/s
			short vy;//Ground Y Speed (Longitude), expressed as cm/s
			short vz;//Ground Z Speed (Altitude), expressed as cm/s
			@A short ind_airspeed;//Indicated airspeed, expressed as cm/s
			@A short true_airspeed;//True airspeed, expressed as cm/s
			short xacc;//X acceleration (mg)
			short yacc;//Y acceleration (mg)
			short zacc;//Z acceleration (mg)
		}
		
		/**
		 * The RAW IMU readings for secondary 9DOF sensor setup. This message should contain the scaled values to
		 * the described unit
		 */
		@id(116) class SCALED_IMU2 {
			@A int time_boot_ms;//Timestamp (milliseconds since system boot)
			short xacc;//X acceleration (mg)
			short yacc;//Y acceleration (mg)
			short zacc;//Z acceleration (mg)
			short xgyro;//Angular speed around X axis (millirad /sec)
			short ygyro;//Angular speed around Y axis (millirad /sec)
			short zgyro;//Angular speed around Z axis (millirad /sec)
			short xmag;//X Magnetic field (milli tesla)
			short ymag;//Y Magnetic field (milli tesla)
			short zmag;//Z Magnetic field (milli tesla)
		}
		
		/**
		 * Request a list of available logs. On some systems calling this may stop on-board logging until LOG_REQUEST_END
		 * is called
		 */
		@id(117) class LOG_REQUEST_LIST {
			@A byte  target_system;//System ID
			@A byte  target_component;//Component ID
			@A short start;//First log id (0 for first available)
			@A short end;//Last log id (0xffff for last available)
		}
		
		/**
		 * Reply to LOG_REQUEST_LIST
		 */
		@id(118) class LOG_ENTRY {
			@A short id;//Log id
			@A short num_logs;//Total number of logs
			@A short last_log_num;//High log number
			@A int   time_utc;//UTC timestamp of log in seconds since 1970, or 0 if not available
			@A int   size;//Size of the log (may be approximate) in bytes
		}
		
		/**
		 * Request a chunk of a log
		 */
		@id(119) class LOG_REQUEST_DATA {
			@A byte  target_system;//System ID
			@A byte  target_component;//Component ID
			@A short id;//Log id (from LOG_ENTRY reply)
			@A int   ofs;//Offset into the log
			@A int   count;//Number of bytes
		}
		
		/**
		 * Reply to LOG_REQUEST_DATA
		 */
		@id(120) class LOG_DATA {
			@A        short  id;//Log id (from LOG_ENTRY reply)
			@A        int    ofs;//Offset into the log
			@A        byte   count;//Number of bytes (zero for end of log)
			@D(90) @A byte[] data;//log data
		}
		
		/**
		 * Erase all logs
		 */
		@id(121) class LOG_ERASE {
			@A byte target_system;//System ID
			@A byte target_component;//Component ID
		}
		
		/**
		 * Stop log transfer and resume normal logging
		 */
		@id(122) class LOG_REQUEST_END {
			@A byte target_system;//System ID
			@A byte target_component;//Component ID
		}
		
		/**
		 * data for injecting into the onboard GPS (used for DGPS)
		 */
		@id(123) class GPS_INJECT_DATA {
			@A         byte   target_system;//System ID
			@A         byte   target_component;//Component ID
			@A         byte   len;//data length
			@D(110) @A byte[] data;//raw data (110 is enough for 12 satellites of RTCMv2)
		}
		
		/**
		 * Second GPS data. Coordinate frame is right-handed, Z-axis up (GPS frame).
		 */
		@id(124) class GPS2_RAW {
			@A long time_usec;//Timestamp (microseconds since UNIX epoch or microseconds since system boot)
			GPS_FIX_TYPE fix_type;//See the GPS_FIX_TYPE enum.
			int          lat;//Latitude (WGS84), in degrees * 1E7
			int          lon;//Longitude (WGS84), in degrees * 1E7
			int          alt;//Altitude (AMSL, not WGS84), in meters * 1000 (positive for up)
			@A short eph;//GPS HDOP horizontal dilution of position in cm (m*100). If unknown, set to: UINT16_MAX
			@A short epv;//GPS VDOP vertical dilution of position in cm (m*100). If unknown, set to: UINT16_MAX
			@A short vel;//GPS ground speed (m/s * 100). If unknown, set to: UINT16_MAX
			/**
			 * Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If
			 * unknown, set to: UINT16_MA
			 */
			@A short cog;
			@A byte  satellites_visible;//Number of satellites visible. If unknown, set to 255
			@A byte  dgps_numch;//Number of DGPS satellites
			@A int   dgps_age;//Age of DGPS info
		}
		
		/**
		 * Power supply status
		 */
		@id(125) class POWER_STATUS {
			@A short Vcc;//5V rail voltage in millivolts
			@A short Vservo;//servo rail voltage in millivolts
			MAV_POWER_STATUS flags;//power supply status flags (see MAV_POWER_STATUS enum)
		}
		
		/**
		 * Control a serial port. This can be used for raw access to an onboard serial peripheral such as a GPS or
		 * telemetry radio. It is designed to make it possible to update the devices firmware via MAVLink messages
		 * or change the devices settings. A message with zero bytes can be used to change just the baudrate
		 */
		@id(126) class SERIAL_CONTROL {
			SERIAL_CONTROL_DEV  device;//See SERIAL_CONTROL_DEV enum
			SERIAL_CONTROL_FLAG flags;//See SERIAL_CONTROL_FLAG enum
			@A        short  timeout;//Timeout for reply data in milliseconds
			@A        int    baudrate;//Baudrate of transfer. Zero means no change.
			@A        byte   count;//how many bytes in this transfer
			@D(70) @A byte[] data;//serial data
		}
		
		/**
		 * RTK GPS data. Gives information on the relative baseline calculation the GPS is reporting
		 */
		@id(127) class GPS_RTK {
			@A int   time_last_baseline_ms;//Time since boot of last baseline message received in ms.
			@A byte  rtk_receiver_id;//Identification of connected RTK receiver.
			@A short wn;//GPS Week Number of last baseline
			@A int   tow;//GPS Time of Week of last baseline
			@A byte  rtk_health;//GPS-specific health report for RTK data.
			@A byte  rtk_rate;//Rate of baseline messages being received by GPS, in HZ
			@A byte  nsats;//Current number of sats used for RTK calculation.
			@A byte  baseline_coords_type;//Coordinate system of baseline. 0 == ECEF, 1 == NED
			int baseline_a_mm;//Current baseline in ECEF x or NED north component in mm.
			int baseline_b_mm;//Current baseline in ECEF y or NED east component in mm.
			int baseline_c_mm;//Current baseline in ECEF z or NED down component in mm.
			@A int accuracy;//Current estimate of baseline accuracy.
			int iar_num_hypotheses;//Current number of integer ambiguity hypotheses.
		}
		
		/**
		 * RTK GPS data. Gives information on the relative baseline calculation the GPS is reporting
		 */
		@id(128) class GPS2_RTK {
			@A int   time_last_baseline_ms;//Time since boot of last baseline message received in ms.
			@A byte  rtk_receiver_id;//Identification of connected RTK receiver.
			@A short wn;//GPS Week Number of last baseline
			@A int   tow;//GPS Time of Week of last baseline
			@A byte  rtk_health;//GPS-specific health report for RTK data.
			@A byte  rtk_rate;//Rate of baseline messages being received by GPS, in HZ
			@A byte  nsats;//Current number of sats used for RTK calculation.
			@A byte  baseline_coords_type;//Coordinate system of baseline. 0 == ECEF, 1 == NED
			int baseline_a_mm;//Current baseline in ECEF x or NED north component in mm.
			int baseline_b_mm;//Current baseline in ECEF y or NED east component in mm.
			int baseline_c_mm;//Current baseline in ECEF z or NED down component in mm.
			@A int accuracy;//Current estimate of baseline accuracy.
			int iar_num_hypotheses;//Current number of integer ambiguity hypotheses.
		}
		
		/**
		 * The RAW IMU readings for 3rd 9DOF sensor setup. This message should contain the scaled values to the described
		 * unit
		 */
		@id(129) class SCALED_IMU3 {
			@A int time_boot_ms;//Timestamp (milliseconds since system boot)
			short xacc;//X acceleration (mg)
			short yacc;//Y acceleration (mg)
			short zacc;//Z acceleration (mg)
			short xgyro;//Angular speed around X axis (millirad /sec)
			short ygyro;//Angular speed around Y axis (millirad /sec)
			short zgyro;//Angular speed around Z axis (millirad /sec)
			short xmag;//X Magnetic field (milli tesla)
			short ymag;//Y Magnetic field (milli tesla)
			short zmag;//Z Magnetic field (milli tesla)
		}
		
		@id(130) class DATA_TRANSMISSION_HANDSHAKE {
			@A byte  type;//type of requested/acknowledged data (as defined in ENUM DATA_TYPES in mavlink/include/mavlink_types.h
			@A int   size;//total data size in bytes (set on ACK only)
			@A short width;//Width of a matrix or image
			@A short height;//Height of a matrix or image
			@A short packets;//number of packets beeing sent (set on ACK only)
			/**
			 * payload size per packet (normally 253 byte, see DATA field size in message ENCAPSULATED_DATA) (set on
			 * ACK only
			 */
			@A byte  payload;
			@A byte  jpg_quality;//JPEG quality out of [1,100]
		}
		
		@id(131) class ENCAPSULATED_DATA {
			@A         short  seqnr;//sequence number (starting with 0 on every transmission)
			@D(253) @A byte[] data;//image data bytes
		}
		
		@id(132) class DISTANCE_SENSOR {
			@A int   time_boot_ms;//Time since system boot
			@A short min_distance;//Minimum distance the sensor can measure in centimeters
			@A short max_distance;//Maximum distance the sensor can measure in centimeters
			@A short current_distance;//Current distance reading
			MAV_DISTANCE_SENSOR type;//Type from MAV_DISTANCE_SENSOR enum.
			@A byte id;//Onboard ID of the sensor
			/**
			 * Direction the sensor faces from MAV_SENSOR_ORIENTATION enum. downward-facing: ROTATION_PITCH_270, upward-facing:
			 * ROTATION_PITCH_90, backward-facing: ROTATION_PITCH_180, forward-facing: ROTATION_NONE, left-facing: ROTATION_YAW_90,
			 * right-facing: ROTATION_YAW_27
			 */
			MAV_SENSOR_ORIENTATION orientation;
			@A byte covariance;//Measurement covariance in centimeters, 0 for unknown / invalid readings
		}
		
		/**
		 * Request for terrain data and terrain status
		 */
		@id(133) class TERRAIN_REQUEST {
			int lat;//Latitude of SW corner of first grid (degrees *10^7)
			int lon;//Longitude of SW corner of first grid (in degrees *10^7)
			@A short grid_spacing;//Grid spacing in meters
			@A long  mask;//Bitmask of requested 4x4 grids (row major 8x7 array of grids, 56 bits)
		}
		
		/**
		 * Terrain data sent from GCS. The lat/lon and grid_spacing must be the same as a lat/lon from a TERRAIN_REQUES
		 */
		@id(134) class TERRAIN_DATA {
			int lat;//Latitude of SW corner of first grid (degrees *10^7)
			int lon;//Longitude of SW corner of first grid (in degrees *10^7)
			@A     short   grid_spacing;//Grid spacing in meters
			@A     byte    gridbit;//bit within the terrain request mask
			@D(16) short[] data;//Terrain data in meters AMSL
		}
		
		/**
		 * Request that the vehicle report terrain height at the given location. Used by GCS to check if vehicle
		 * has all terrain data needed for a mission
		 */
		@id(135) class TERRAIN_CHECK {
			int lat;//Latitude (degrees *10^7)
			int lon;//Longitude (degrees *10^7)
		}
		
		/**
		 * Response from a TERRAIN_CHECK request
		 */
		@id(136) class TERRAIN_REPORT {
			int lat;//Latitude (degrees *10^7)
			int lon;//Longitude (degrees *10^7)
			@A short spacing;//grid spacing (zero if terrain at this location unavailable)
			float terrain_height;//Terrain height in meters AMSL
			float current_height;//Current vehicle height above lat/lon terrain height (meters)
			@A short pending;//Number of 4x4 terrain blocks waiting to be received or read from disk
			@A short loaded;//Number of 4x4 terrain blocks in memory
		}
		
		/**
		 * Barometer readings for 2nd barometer
		 */
		@id(137) class SCALED_PRESSURE2 {
			@A int time_boot_ms;//Timestamp (milliseconds since system boot)
			float press_abs;//Absolute pressure (hectopascal)
			float press_diff;//Differential pressure 1 (hectopascal)
			short temperature;//Temperature measurement (0.01 degrees celsius)
		}
		
		/**
		 * Motion capture attitude and position
		 */
		@id(138) class ATT_POS_MOCAP {
			@A    long    time_usec;//Timestamp (micros since boot or Unix epoch)
			@D(4) float[] q;//Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
			float x;//X position in meters (NED)
			float y;//Y position in meters (NED)
			float z;//Z position in meters (NED)
		}
		
		/**
		 * Set the vehicle attitude and body angular rates.
		 */
		@id(139) class SET_ACTUATOR_CONTROL_TARGET {
			@A    long    time_usec;//Timestamp (micros since boot or Unix epoch)
			/**
			 * Actuator group. The "_mlx" indicates this is a multi-instance message and a MAVLink parser should use
			 * this field to difference between instances
			 */
			@A    byte    group_mlx;
			@A    byte    target_system;//System ID
			@A    byte    target_component;//Component ID
			/**
			 * Actuator controls. Normed to -1..+1 where 0 is neutral position. Throttle for single rotation direction
			 * motors is 0..1, negative range for reverse direction. Standard mapping for attitude controls (group 0):
			 * (index 0-7): roll, pitch, yaw, throttle, flaps, spoilers, airbrakes, landing gear. Load a pass-through
			 * mixer to repurpose them as generic outputs
			 */
			@D(8) float[] controls;
		}
	}
}

class MicroAirVehicle implements InC {
	interface CommunicationInterface extends MicroAirVehicleHandledPacks, GroundControl.CommonPacks {}
	
	interface MicroAirVehicleHandledPacks {
		
		/**
		 * Set the vehicle attitude and body angular rates.
		 */
		@id(140) class ACTUATOR_CONTROL_TARGET {
			@A    long    time_usec;//Timestamp (micros since boot or Unix epoch)
			/**
			 * Actuator group. The "_mlx" indicates this is a multi-instance message and a MAVLink parser should use
			 * this field to difference between instances
			 */
			@A    byte    group_mlx;
			/**
			 * Actuator controls. Normed to -1..+1 where 0 is neutral position. Throttle for single rotation direction
			 * motors is 0..1, negative range for reverse direction. Standard mapping for attitude controls (group 0):
			 * (index 0-7): roll, pitch, yaw, throttle, flaps, spoilers, airbrakes, landing gear. Load a pass-through
			 * mixer to repurpose them as generic outputs
			 */
			@D(8) float[] controls;
		}
		
		/**
		 * The current system altitude.
		 */
		@id(141) class ALTITUDE {
			@A long time_usec;//Timestamp (micros since boot or Unix epoch)
			/**
			 * This altitude measure is initialized on system boot and monotonic (it is never reset, but represents the
			 * local altitude change). The only guarantee on this field is that it will never be reset and is consistent
			 * within a flight. The recommended value for this field is the uncorrected barometric altitude at boot
			 * time. This altitude will also drift and vary between flights
			 */
			float altitude_monotonic;
			/**
			 * This altitude measure is strictly above mean sea level and might be non-monotonic (it might reset on events
			 * like GPS lock or when a new QNH value is set). It should be the altitude to which global altitude waypoints
			 * are compared to. Note that it is *not* the GPS altitude, however, most GPS modules already output AMSL
			 * by default and not the WGS84 altitude
			 */
			float altitude_amsl;
			/**
			 * This is the local altitude in the local coordinate frame. It is not the altitude above home, but in reference
			 * to the coordinate origin (0, 0, 0). It is up-positive
			 */
			float altitude_local;
			float altitude_relative;//This is the altitude above the home position. It resets on each change of the current home position
			/**
			 * This is the altitude above terrain. It might be fed by a terrain database or an altimeter. Values smaller
			 * than -1000 should be interpreted as unknown
			 */
			float altitude_terrain;
			/**
			 * This is not the altitude, but the clear space below the system according to the fused clearance estimate.
			 * It generally should max out at the maximum range of e.g. the laser altimeter. It is generally a moving
			 * target. A negative value indicates no measurement available
			 */
			float bottom_clearance;
		}
		
		/**
		 * The autopilot is requesting a resource (file, binary, other type of data)
		 */
		@id(142) class RESOURCE_REQUEST {
			@A         byte   request_id;//Request ID. This ID should be re-used when sending back URI contents
			@A         byte   uri_type;//The type of requested URI. 0 = a file via URL. 1 = a UAVCAN binary
			/**
			 * The requested unique resource identifier (URI). It is not necessarily a straight domain name (depends
			 * on the URI type enum
			 */
			@D(120) @A byte[] uri;
			@A         byte   transfer_type;//The way the autopilot wants to receive the URI. 0 = MAVLink FTP. 1 = binary stream.
			/**
			 * The storage path the autopilot wants the URI to be stored in. Will only be valid if the transfer_type
			 * has a storage associated (e.g. MAVLink FTP)
			 */
			@D(120) @A byte[] storage;
		}
		
		/**
		 * Barometer readings for 3rd barometer
		 */
		@id(143) class SCALED_PRESSURE3 {
			@A int time_boot_ms;//Timestamp (milliseconds since system boot)
			float press_abs;//Absolute pressure (hectopascal)
			float press_diff;//Differential pressure 1 (hectopascal)
			short temperature;//Temperature measurement (0.01 degrees celsius)
		}
		
		/**
		 * current motion information from a designated system
		 */
		@id(144) class FOLLOW_TARGET {
			@A long timestamp;//Timestamp in milliseconds since system boot
			@A byte est_capabilities;//bit positions for tracker reporting capabilities (POS = 0, VEL = 1, ACCEL = 2, ATT + RATES = 3)
			int   lat;//Latitude (WGS84), in degrees * 1E7
			int   lon;//Longitude (WGS84), in degrees * 1E7
			float alt;//AMSL, in meters
			@D(3) float[] vel;//target velocity (0,0,0) for unknown
			@D(3) float[] acc;//linear target acceleration (0,0,0) for unknown
			@D(4) float[] attitude_q;//(1 0 0 0 for unknown)
			@D(3) float[] rates;//(0 0 0 for unknown)
			@D(3) float[] position_cov;//eph epv
			@A    long    custom_state;//button states or switches of a tracker device
		}
		
		/**
		 * The smoothed, monotonic system state used to feed the control loops of the system.
		 */
		@id(146) class CONTROL_SYSTEM_STATE {
			@A long time_usec;//Timestamp (micros since boot or Unix epoch)
			float x_acc;//X acceleration in body frame
			float y_acc;//Y acceleration in body frame
			float z_acc;//Z acceleration in body frame
			float x_vel;//X velocity in body frame
			float y_vel;//Y velocity in body frame
			float z_vel;//Z velocity in body frame
			float x_pos;//X position in local frame
			float y_pos;//Y position in local frame
			float z_pos;//Z position in local frame
			float airspeed;//Airspeed, set to -1 if unknown
			@D(3) float[] vel_variance;//Variance of body velocity estimate
			@D(3) float[] pos_variance;//Variance in local position
			@D(4) float[] q;//The attitude, represented as Quaternion
			float roll_rate;//Angular rate in roll axis
			float pitch_rate;//Angular rate in pitch axis
			float yaw_rate;//Angular rate in yaw axis
		}
		
		/**
		 * Battery information
		 */
		@id(147) class BATTERY_STATUS {
			@A byte id;//Battery ID
			MAV_BATTERY_FUNCTION battery_function;//Function of the battery
			MAV_BATTERY_TYPE     type;//Type (chemistry) of the battery
			short                temperature;//Temperature of the battery in centi-degrees celsius. INT16_MAX for unknown temperature.
			/**
			 * Battery voltage of cells, in millivolts (1 = 1 millivolt). Cells above the valid cell count for this battery
			 * should have the UINT16_MAX value
			 */
			@D(10) @A short[] voltages;
			short current_battery;//Battery current, in 10*milliamperes (1 = 10 milliampere), -1: autopilot does not measure the curren
			int   current_consumed;//Consumed charge, in milliampere hours (1 = 1 mAh), -1: autopilot does not provide mAh consumption estimat
			/**
			 * Consumed energy, in HectoJoules (intergrated U*I*dt)  (1 = 100 Joule), -1: autopilot does not provide
			 * energy consumption estimat
			 */
			int   energy_consumed;
			byte  battery_remaining;//Remaining battery energy: (0%: 0, 100%: 100), -1: autopilot does not estimate the remaining battery
		}
		
		/**
		 * Version and capability of autopilot software
		 */
		@id(148) class AUTOPILOT_VERSION {
			MAV_PROTOCOL_CAPABILITY capabilities;//bitmask of capabilities (see MAV_PROTOCOL_CAPABILITY enum)
			@A         int    flight_sw_version;//Firmware version number
			@A         int    middleware_sw_version;//Middleware version number
			@A         int    os_sw_version;//Operating system version number
			@A         int    board_version;//HW / board version (last 8 bytes should be silicon ID, if any)
			/**
			 * Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but
			 * should allow to identify the commit using the main version number even for very large code bases
			 */
			@D(8) @A   byte[] flight_custom_version;
			/**
			 * Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but
			 * should allow to identify the commit using the main version number even for very large code bases
			 */
			@D(8) @A   byte[] middleware_custom_version;
			/**
			 * Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but
			 * should allow to identify the commit using the main version number even for very large code bases
			 */
			@D(8) @A   byte[] os_custom_version;
			@A         short  vendor_id;//ID of the board vendor
			@A         short  product_id;//ID of the product
			@A         long   uid;//UID if provided by hardware (see uid2)
			/**
			 * UID if provided by hardware (supersedes the uid field. If this is non-zero, use this field, otherwise
			 * use uid
			 */
			@D_(18) @A byte[] uid2;
		}
		
		/**
		 * The location of a landing area captured from a downward facing camera
		 */
		@id(149) class LANDING_TARGET {
			@A long time_usec;//Timestamp (micros since boot or Unix epoch)
			@A byte target_num;//The ID of the target if multiple targets are present
			MAV_FRAME frame;//MAV_FRAME enum specifying the whether the following feilds are earth-frame, body-frame, etc.
			float     angle_x;//X-axis angular offset (in radians) of the target from the center of the image
			float     angle_y;//Y-axis angular offset (in radians) of the target from the center of the image
			float     distance;//Distance to the target from the vehicle in meters
			float     size_x;//Size in radians of target along x-axis
			float     size_y;//Size in radians of target along y-axis
			@I_    float   x;//X Position of the landing target on MAV_FRAME
			@I_    float   y;//Y Position of the landing target on MAV_FRAME
			@I_    float   z;//Z Position of the landing target on MAV_FRAME
			@D_(4) float[] q;//Quaternion of landing target orientation (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
			LANDING_TARGET_TYPE type;//LANDING_TARGET_TYPE enum specifying the type of landing target
			/**
			 * Boolean indicating known position (1) or default unkown position (0), for validation of positioning of
			 * the landing targe
			 */
			@A_ byte position_valid;
		}
		
		/**
		 * Estimator status message including flags, innovation test ratios and estimated accuracies. The flags message
		 * is an integer bitmask containing information on which EKF outputs are valid. See the ESTIMATOR_STATUS_FLAGS
		 * enum definition for further information. The innovaton test ratios show the magnitude of the sensor innovation
		 * divided by the innovation check threshold. Under normal operation the innovaton test ratios should be
		 * below 0.5 with occasional values up to 1.0. Values greater than 1.0 should be rare under normal operation
		 * and indicate that a measurement has been rejected by the filter. The user should be notified if an innovation
		 * test ratio greater than 1.0 is recorded. Notifications for values in the range between 0.5 and 1.0 should
		 * be optional and controllable by the user
		 */
		@id(230) class ESTIMATOR_STATUS {
			@A long time_usec;//Timestamp (micros since boot or Unix epoch)
			ESTIMATOR_STATUS_FLAGS flags;//Integer bitmask indicating which EKF outputs are valid. See definition for ESTIMATOR_STATUS_FLAGS.
			float                  vel_ratio;//Velocity innovation test ratio
			float                  pos_horiz_ratio;//Horizontal position innovation test ratio
			float                  pos_vert_ratio;//Vertical position innovation test ratio
			float                  mag_ratio;//Magnetometer innovation test ratio
			float                  hagl_ratio;//Height above terrain innovation test ratio
			float                  tas_ratio;//True airspeed innovation test ratio
			float                  pos_horiz_accuracy;//Horizontal position 1-STD accuracy relative to the EKF local origin (m)
			float                  pos_vert_accuracy;//Vertical position 1-STD accuracy relative to the EKF local origin (m)
		}
		
		@id(231) class WIND_COV {
			@A long time_usec;//Timestamp (micros since boot or Unix epoch)
			float wind_x;//Wind in X (NED) direction in m/s
			float wind_y;//Wind in Y (NED) direction in m/s
			float wind_z;//Wind in Z (NED) direction in m/s
			float var_horiz;//Variability of the wind in XY. RMS of a 1 Hz lowpassed wind estimate.
			float var_vert;//Variability of the wind in Z. RMS of a 1 Hz lowpassed wind estimate.
			float wind_alt;//AMSL altitude (m) this measurement was taken at
			float horiz_accuracy;//Horizontal speed 1-STD accuracy
			float vert_accuracy;//Vertical speed 1-STD accuracy
		}
		
		/**
		 * GPS sensor input message.  This is a raw sensor value sent by the GPS. This is NOT the global position
		 * estimate of the sytem
		 */
		@id(232) class GPS_INPUT {
			@A long time_usec;//Timestamp (micros since boot or Unix epoch)
			@A byte gps_id;//ID of the GPS for multiple GPS inputs
			GPS_INPUT_IGNORE_FLAGS ignore_flags;//Flags indicating which fields to ignore (see GPS_INPUT_IGNORE_FLAGS enum).  All other fields must be provided
			@A int   time_week_ms;//GPS time (milliseconds from start of GPS week)
			@A short time_week;//GPS week number
			@A byte  fix_type;//0-1: no fix, 2: 2D fix, 3: 3D fix. 4: 3D with DGPS. 5: 3D with RTK
			int   lat;//Latitude (WGS84), in degrees * 1E7
			int   lon;//Longitude (WGS84), in degrees * 1E7
			float alt;//Altitude (AMSL, not WGS84), in m (positive for up)
			float hdop;//GPS HDOP horizontal dilution of position in m
			float vdop;//GPS VDOP vertical dilution of position in m
			float vn;//GPS velocity in m/s in NORTH direction in earth-fixed NED frame
			float ve;//GPS velocity in m/s in EAST direction in earth-fixed NED frame
			float vd;//GPS velocity in m/s in DOWN direction in earth-fixed NED frame
			float speed_accuracy;//GPS speed accuracy in m/s
			float horiz_accuracy;//GPS horizontal accuracy in m
			float vert_accuracy;//GPS vertical accuracy in m
			@A byte satellites_visible;//Number of satellites visible.
		}
		
		/**
		 * RTCM message for injecting into the onboard GPS (used for DGPS)
		 */
		@id(233) class GPS_RTCM_DATA {
			/**
			 * LSB: 1 means message is fragmented, next 2 bits are the fragment ID, the remaining 5 bits are used for
			 * the sequence ID. Messages are only to be flushed to the GPS when the entire message has been reconstructed
			 * on the autopilot. The fragment ID specifies which order the fragments should be assembled into a buffer,
			 * while the sequence ID is used to detect a mismatch between different buffers. The buffer is considered
			 * fully reconstructed when either all 4 fragments are present, or all the fragments before the first fragment
			 * with a non full payload is received. This management is used to ensure that normal GPS operation doesn't
			 * corrupt RTCM data, and to recover from a unreliable transport delivery order
			 */
			@A         byte   flags;
			@A         byte   len;//data length
			@D(180) @A byte[] data;//RTCM message (may be fragmented)
		}
		
		/**
		 * Message appropriate for high latency connections like Iridium
		 */
		@id(234) class HIGH_LATENCY {
			MAV_MODE_FLAG base_mode;//System mode bitfield, see MAV_MODE_FLAG ENUM in mavlink/include/mavlink_types.h
			@A int custom_mode;//A bitfield for use for autopilot-specific flags.
			MAV_LANDED_STATE landed_state;//The landed state. Is set to MAV_LANDED_STATE_UNDEFINED if landed state is unknown.
			short            roll;//roll (centidegrees)
			short            pitch;//pitch (centidegrees)
			@A short heading;//heading (centidegrees)
			byte  throttle;//throttle (percentage)
			short heading_sp;//heading setpoint (centidegrees)
			int   latitude;//Latitude, expressed as degrees * 1E7
			int   longitude;//Longitude, expressed as degrees * 1E7
			short altitude_amsl;//Altitude above mean sea level (meters)
			short altitude_sp;//Altitude setpoint relative to the home position (meters)
			@A byte airspeed;//airspeed (m/s)
			@A byte airspeed_sp;//airspeed setpoint (m/s)
			@A byte groundspeed;//groundspeed (m/s)
			byte climb_rate;//climb rate (m/s)
			@A byte gps_nsat;//Number of satellites visible. If unknown, set to 255
			GPS_FIX_TYPE gps_fix_type;//See the GPS_FIX_TYPE enum.
			@A byte battery_remaining;//Remaining battery (percentage)
			byte temperature;//Autopilot temperature (degrees C)
			byte temperature_air;//Air temperature (degrees C) from airspeed sensor
			/**
			 * failsafe (each bit represents a failsafe where 0=ok, 1=failsafe active (bit0:RC, bit1:batt, bit2:GPS,
			 * bit3:GCS, bit4:fence
			 */
			@A byte  failsafe;
			@A byte  wp_num;//current waypoint number
			@A short wp_distance;//distance to target (meters)
		}
		
		/**
		 * Vibration levels and accelerometer clipping
		 */
		@id(241) class VIBRATION {
			@A long time_usec;//Timestamp (micros since boot or Unix epoch)
			float vibration_x;//Vibration levels on X-axis
			float vibration_y;//Vibration levels on Y-axis
			float vibration_z;//Vibration levels on Z-axis
			@A int clipping_0;//first accelerometer clipping count
			@A int clipping_1;//second accelerometer clipping count
			@A int clipping_2;//third accelerometer clipping count
		}
		
		/**
		 * This message can be requested by sending the MAV_CMD_GET_HOME_POSITION command. The position the system
		 * will return to and land on. The position is set automatically by the system during the takeoff in case
		 * it was not explicitely set by the operator before or after. The position the system will return to and
		 * land on. The global and local positions encode the position in the respective coordinate frames, while
		 * the q parameter encodes the orientation of the surface. Under normal conditions it describes the heading
		 * and terrain slope, which can be used by the aircraft to adjust the approach. The approach 3D vector describes
		 * the point to which the system should fly in normal flight mode and then perform a landing sequence along
		 * the vector
		 */
		@id(242) class HOME_POSITION {
			int   latitude;//Latitude (WGS84), in degrees * 1E7
			int   longitude;//Longitude (WGS84, in degrees * 1E7
			int   altitude;//Altitude (AMSL), in meters * 1000 (positive for up)
			float x;//Local X position of this position in the local coordinate frame
			float y;//Local Y position of this position in the local coordinate frame
			float z;//Local Z position of this position in the local coordinate frame
			/**
			 * World to surface normal and heading transformation of the takeoff position. Used to indicate the heading
			 * and slope of the groun
			 */
			@D(4) float[] q;
			/**
			 * Local X position of the end of the approach vector. Multicopters should set this position based on their
			 * takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing
			 * fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened
			 * from the threshold / touchdown zone
			 */
			float approach_x;
			/**
			 * Local Y position of the end of the approach vector. Multicopters should set this position based on their
			 * takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing
			 * fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened
			 * from the threshold / touchdown zone
			 */
			float approach_y;
			/**
			 * Local Z position of the end of the approach vector. Multicopters should set this position based on their
			 * takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing
			 * fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened
			 * from the threshold / touchdown zone
			 */
			float approach_z;
			@A_ long time_usec;//Timestamp (microseconds since UNIX epoch or microseconds since system boot)
		}
		
		/**
		 * The position the system will return to and land on. The position is set automatically by the system during
		 * the takeoff in case it was not explicitely set by the operator before or after. The global and local
		 * positions encode the position in the respective coordinate frames, while the q parameter encodes the
		 * orientation of the surface. Under normal conditions it describes the heading and terrain slope, which
		 * can be used by the aircraft to adjust the approach. The approach 3D vector describes the point to which
		 * the system should fly in normal flight mode and then perform a landing sequence along the vector
		 */
		@id(243) class SET_HOME_POSITION {
			@A byte target_system;//System ID.
			int   latitude;//Latitude (WGS84), in degrees * 1E7
			int   longitude;//Longitude (WGS84, in degrees * 1E7
			int   altitude;//Altitude (AMSL), in meters * 1000 (positive for up)
			float x;//Local X position of this position in the local coordinate frame
			float y;//Local Y position of this position in the local coordinate frame
			float z;//Local Z position of this position in the local coordinate frame
			/**
			 * World to surface normal and heading transformation of the takeoff position. Used to indicate the heading
			 * and slope of the groun
			 */
			@D(4) float[] q;
			/**
			 * Local X position of the end of the approach vector. Multicopters should set this position based on their
			 * takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing
			 * fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened
			 * from the threshold / touchdown zone
			 */
			float approach_x;
			/**
			 * Local Y position of the end of the approach vector. Multicopters should set this position based on their
			 * takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing
			 * fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened
			 * from the threshold / touchdown zone
			 */
			float approach_y;
			/**
			 * Local Z position of the end of the approach vector. Multicopters should set this position based on their
			 * takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing
			 * fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened
			 * from the threshold / touchdown zone
			 */
			float approach_z;
			@A_ long time_usec;//Timestamp (microseconds since UNIX epoch or microseconds since system boot)
		}
		
		/**
		 * This interface replaces DATA_STREAM
		 */
		@id(244) class MESSAGE_INTERVAL {
			@A short message_id;//The ID of the requested MAVLink message. v1.0 is limited to 254 messages.
			int interval_us;//0 indicates the interval at which it is sent.
		}
		
		/**
		 * Provides state for additional features
		 */
		@id(245) class EXTENDED_SYS_STATE {
			MAV_VTOL_STATE   vtol_state;//The VTOL state if applicable. Is set to MAV_VTOL_STATE_UNDEFINED if UAV is not in VTOL configuration
			MAV_LANDED_STATE landed_state;//The landed state. Is set to MAV_LANDED_STATE_UNDEFINED if landed state is unknown.
		}
		
		/**
		 * The location and information of an ADSB vehicle
		 */
		@id(246) class ADSB_VEHICLE {
			@A int ICAO_address;//ICAO address
			int                lat;//Latitude, expressed as degrees * 1E7
			int                lon;//Longitude, expressed as degrees * 1E7
			ADSB_ALTITUDE_TYPE altitude_type;//Type from ADSB_ALTITUDE_TYPE enum
			int                altitude;//Altitude(ASL) in millimeters
			@A short heading;//Course over ground in centidegrees
			@A short hor_velocity;//The horizontal velocity in centimeters/second
			short ver_velocity;//The vertical velocity in centimeters/second, positive is up
			@A(9) String callsign;//The callsign, 8+null
			ADSB_EMITTER_TYPE emitter_type;//Type from ADSB_EMITTER_TYPE enum
			@A byte tslc;//Time since last communication in seconds
			ADSB_FLAGS flags;//Flags to indicate various statuses including valid data fields
			@A short squawk;//Squawk code
		}
		
		/**
		 * Information about a potential collision
		 */
		@id(247) class COLLISION {
			MAV_COLLISION_SRC src;//Collision data source
			@A int id;//Unique identifier, domain based on src field
			MAV_COLLISION_ACTION       action;//Action that is being taken to avoid this collision
			MAV_COLLISION_THREAT_LEVEL threat_level;//How concerned the aircraft is about this collision
			float                      time_to_minimum_delta;//Estimated time until collision occurs (seconds)
			float                      altitude_minimum_delta;//Closest vertical distance in meters between vehicle and object
			float                      horizontal_minimum_delta;//Closest horizontal distance in meteres between vehicle and object
		}
		
		/**
		 * Message implementing parts of the V2 payload specs in V1 frames for transitional support.
		 */
		@id(248) class V2_EXTENSION {
			@A         byte   target_network;//Network ID (0 for broadcast)
			@A         byte   target_system;//System ID (0 for broadcast)
			@A         byte   target_component;//Component ID (0 for broadcast)
			/**
			 * A code that identifies the software component that understands this message (analogous to usb device classes
			 * or mime type strings).  If this code is less than 32768, it is considered a 'registered' protocol extension
			 * and the corresponding entry should be added to https://github.com/mavlink/mavlink/extension-message-ids.xml.
			 * Software creators can register blocks of message IDs as needed (useful for GCS specific metadata, etc...).
			 * Message_types greater than 32767 are considered local experiments and should not be checked in to any
			 * widely distributed codebase
			 */
			@A         short  message_type;
			/**
			 * Variable length payload. The length is defined by the remaining message length when subtracting the header
			 * and other fields.  The entire content of this block is opaque unless you understand any the encoding
			 * message_type.  The particular encoding used can be extension specific and might not always be documented
			 * as part of the mavlink specification
			 */
			@D(249) @A byte[] payload;
		}
		
		/**
		 * Send raw controller memory. The use of this message is discouraged for normal packets, but a quite efficient
		 * way for testing new messages and getting experimental debug output
		 */
		@id(249) class MEMORY_VECT {
			@A     short  address;//Starting address of the debug variables
			@A     byte   ver;//Version code of the type variable. 0=unknown, type ignored and assumed int16_t. 1=as below
			@A     byte   type;//Type code of the memory variables. for ver = 1: 0=16 x int16_t, 1=16 x uint16_t, 2=16 x Q15, 3=16 x 1Q1
			@D(32) byte[] value;//Memory contents at specified address
		}
		
		@id(250) class DEBUG_VECT {
			@A(10) String name;//Name
			@A     long   time_usec;//Timestamp
			float x;//x
			float y;//y
			float z;//z
		}
		
		/**
		 * Send a key-value pair as float. The use of this message is discouraged for normal packets, but a quite
		 * efficient way for testing new messages and getting experimental debug output
		 */
		@id(251) class NAMED_VALUE_FLOAT {
			@A     int    time_boot_ms;//Timestamp (milliseconds since system boot)
			@A(10) String name;//Name of the debug variable
			float value;//Floating point value
		}
		
		/**
		 * Send a key-value pair as integer. The use of this message is discouraged for normal packets, but a quite
		 * efficient way for testing new messages and getting experimental debug output
		 */
		@id(252) class NAMED_VALUE_INT {
			@A     int    time_boot_ms;//Timestamp (milliseconds since system boot)
			@A(10) String name;//Name of the debug variable
			int value;//Signed integer value
		}
		
		/**
		 * Status text message. These messages are printed in yellow in the COMM console of QGroundControl. WARNING:
		 * They consume quite some bandwidth, so use only for important status and error messages. If implemented
		 * wisely, these messages are buffered on the MCU and sent only at a limited rate (e.g. 10 Hz)
		 */
		@id(253) class STATUSTEXT {
			MAV_SEVERITY severity;//Severity of status. Relies on the definitions within RFC-5424. See enum MAV_SEVERITY.
			@A(50) String text;//Status text message, without null termination character
		}
		
		/**
		 * Send a debug value. The index is used to discriminate between values. These values show up in the plot
		 * of QGroundControl as DEBUG N
		 */
		@id(254) class DEBUG {
			@A int  time_boot_ms;//Timestamp (milliseconds since system boot)
			@A byte ind;//index of debug variable
			float value;//DEBUG value
		}
		
		/**
		 * Setup a MAVLink2 signing key. If called with secret_key of all zero and zero initial_timestamp will disable
		 * signin
		 */
		@id(256) class SETUP_SIGNING {
			@A        byte   target_system;//system id of the target
			@A        byte   target_component;//component ID of the target
			@D(32) @A byte[] secret_key;//signing key
			@A        long   initial_timestamp;//initial timestamp
		}
		
		/**
		 * Report button state change
		 */
		@id(257) class BUTTON_CHANGE {
			@A int  time_boot_ms;//Timestamp (milliseconds since system boot)
			@A int  last_change_ms;//Time of last change of button state
			@A byte state;//Bitmap state of buttons
		}
		
		/**
		 * Control vehicle tone generation (buzzer)
		 */
		@id(258) class PLAY_TUNE {
			@A     byte   target_system;//System ID
			@A     byte   target_component;//Component ID
			@A(30) String tune;//tune in board specific format
		}
		
		/**
		 * WIP: Information about a camera
		 */
		@id(259) class CAMERA_INFORMATION {
			@A        int    time_boot_ms;//Timestamp (milliseconds since system boot)
			@D(32) @A byte[] vendor_name;//Name of the camera vendor
			@D(32) @A byte[] model_name;//Name of the camera model
			@A        int    firmware_version;//0xff = Major)
			float focal_length;//Focal length in mm
			float sensor_size_h;//Image sensor size horizontal in mm
			float sensor_size_v;//Image sensor size vertical in mm
			@A short resolution_h;//Image resolution in pixels horizontal
			@A short resolution_v;//Image resolution in pixels vertical
			@A byte  lens_id;//Reserved for a lens ID
			CAMERA_CAP_FLAGS flags;//CAMERA_CAP_FLAGS enum flags (bitmap) describing camera capabilities.
			@A      short  cam_definition_version;//Camera definition version (iteration)
			@A(140) String cam_definition_uri;//Camera definition URI (if any, otherwise only basic functions will be available).
		}
		
		/**
		 * WIP: Settings of a camera, can be requested using MAV_CMD_REQUEST_CAMERA_SETTINGS.
		 */
		@id(260) class CAMERA_SETTINGS {
			@A int time_boot_ms;//Timestamp (milliseconds since system boot)
			CAMERA_MODE mode_id;//Camera mode (CAMERA_MODE)
		}
		
		/**
		 * WIP: Information about a storage medium.
		 */
		@id(261) class STORAGE_INFORMATION {
			@A int  time_boot_ms;//Timestamp (milliseconds since system boot)
			@A byte storage_id;//Storage ID (1 for first, 2 for second, etc.)
			@A byte storage_count;//Number of storage devices
			@A byte status;//Status of storage (0 not available, 1 unformatted, 2 formatted)
			float total_capacity;//Total capacity in MiB
			float used_capacity;//Used capacity in MiB
			float available_capacity;//Available capacity in MiB
			float read_speed;//Read speed in MiB/s
			float write_speed;//Write speed in MiB/s
		}
		
		/**
		 * WIP: Information about the status of a capture
		 */
		@id(262) class CAMERA_CAPTURE_STATUS {
			@A int  time_boot_ms;//Timestamp (milliseconds since system boot)
			/**
			 * Current status of image capturing (0: idle, 1: capture in progress, 2: interval set but idle, 3: interval
			 * set and capture in progress
			 */
			@A byte image_status;
			@A byte video_status;//Current status of video capturing (0: idle, 1: capture in progress)
			float image_interval;//Image capture interval in seconds
			@A int recording_time_ms;//Time in milliseconds since recording started
			float available_capacity;//Available storage capacity in MiB
		}
		
		/**
		 * Information about a captured image
		 */
		@id(263) class CAMERA_IMAGE_CAPTURED {
			@A int  time_boot_ms;//Timestamp (milliseconds since system boot)
			@A long time_utc;//Timestamp (microseconds since UNIX epoch) in UTC. 0 for unknown.
			@A byte camera_id;//Camera ID (1 for first, 2 for second, etc.)
			int lat;//Latitude, expressed as degrees * 1E7 where image was taken
			int lon;//Longitude, expressed as degrees * 1E7 where capture was taken
			int alt;//Altitude in meters, expressed as * 1E3 (AMSL, not WGS84) where image was taken
			int relative_alt;//Altitude above ground in meters, expressed as * 1E3 where image was taken
			@D(4) float[] q;//Quaternion of camera orientation (w, x, y, z order, zero-rotation is 0, 0, 0, 0)
			int  image_index;//Zero based index of this image (image count since armed -1)
			byte capture_result;//Boolean indicating success (1) or failure (0) while capturing this image.
			@A(205) String file_url;//URL of image taken. Either local storage or http://foo.jpg if camera provides an HTTP interface.
		}
		
		/**
		 * WIP: Information about flight since last arming
		 */
		@id(264) class FLIGHT_INFORMATION {
			@A int  time_boot_ms;//Timestamp (milliseconds since system boot)
			@A long arming_time_utc;//Timestamp at arming (microseconds since UNIX epoch) in UTC, 0 for unknown
			@A long takeoff_time_utc;//Timestamp at takeoff (microseconds since UNIX epoch) in UTC, 0 for unknown
			@A long flight_uuid;//Universally unique identifier (UUID) of flight, should correspond to name of logfiles
		}
		
		/**
		 * WIP: Orientation of a mount
		 */
		@id(265) class MOUNT_ORIENTATION {
			@A int time_boot_ms;//Timestamp (milliseconds since system boot)
			float roll;//Roll in degrees
			float pitch;//Pitch in degrees
			float yaw;//Yaw in degrees
		}
		
		/**
		 * A message containing logged data (see also MAV_CMD_LOGGING_START)
		 */
		@id(266) class LOGGING_DATA {
			@A         byte   target_system;//system ID of the target
			@A         byte   target_component;//component ID of the target
			@A         short  sequence;//sequence number (can wrap)
			@A         byte   length;//data length
			/**
			 * offset into data where first message starts. This can be used for recovery, when a previous message got
			 * lost (set to 255 if no start exists)
			 */
			@A         byte   first_message_offset;
			@D(249) @A byte[] data;//logged data
		}
		
		/**
		 * A message containing logged data which requires a LOGGING_ACK to be sent back
		 */
		@id(267) class LOGGING_DATA_ACKED {
			@A         byte   target_system;//system ID of the target
			@A         byte   target_component;//component ID of the target
			@A         short  sequence;//sequence number (can wrap)
			@A         byte   length;//data length
			/**
			 * offset into data where first message starts. This can be used for recovery, when a previous message got
			 * lost (set to 255 if no start exists)
			 */
			@A         byte   first_message_offset;
			@D(249) @A byte[] data;//logged data
		}
		
		/**
		 * An ack for a LOGGING_DATA_ACKED message
		 */
		@id(268) class LOGGING_ACK {
			@A byte  target_system;//system ID of the target
			@A byte  target_component;//component ID of the target
			@A short sequence;//sequence number (must match the one in LOGGING_DATA_ACKED)
		}
		
		/**
		 * WIP: Information about video stream
		 */
		@id(269) class VIDEO_STREAM_INFORMATION {
			@A byte camera_id;//Camera ID (1 for first, 2 for second, etc.)
			@A byte status;//Current status of video streaming (0: not running, 1: in progress)
			float framerate;//Frames per second
			@A      short  resolution_h;//Resolution horizontal in pixels
			@A      short  resolution_v;//Resolution vertical in pixels
			@A      int    bitrate;//Bit rate in bits per second
			@A      short  rotation;//Video image rotation clockwise
			@A(230) String uri;//Video stream URI
		}
		
		/**
		 * WIP: Message that sets video stream settings
		 */
		@id(270) class SET_VIDEO_STREAM_SETTINGS {
			@A byte target_system;//system ID of the target
			@A byte target_component;//component ID of the target
			@A byte camera_id;//Camera ID (1 for first, 2 for second, etc.)
			float framerate;//Frames per second (set to -1 for highest framerate possible)
			@A      short  resolution_h;//Resolution horizontal in pixels (set to -1 for highest resolution possible)
			@A      short  resolution_v;//Resolution vertical in pixels (set to -1 for highest resolution possible)
			@A      int    bitrate;//Bit rate in bits per second (set to -1 for auto)
			@A      short  rotation;//Video image rotation clockwise (0-359 degrees)
			@A(230) String uri;//Video stream URI
		}
		
		/**
		 * Configure AP SSID and Password.
		 */
		@id(299) class WIFI_CONFIG_AP {
			@A(32) String ssid;//Name of Wi-Fi network (SSID). Leave it blank to leave it unchanged.
			@A(64) String password;//Password. Leave it blank for an open AP.
		}
		
		/**
		 * WIP: Version and capability of protocol version. This message is the response to REQUEST_PROTOCOL_VERSION
		 * and is used as part of the handshaking to establish which MAVLink version should be used on the network.
		 * Every node should respond to REQUEST_PROTOCOL_VERSION to enable the handshaking. Library implementers
		 * should consider adding this into the default decoding state machine to allow the protocol core to respond
		 * directly
		 */
		@id(300) class PROTOCOL_VERSION {
			@A       short  version;//Currently active MAVLink version number * 100: v1.0 is 100, v2.0 is 200, etc.
			@A       short  min_version;//Minimum MAVLink version supported
			@A       short  max_version;//Maximum MAVLink version supported (set to the same value as version by default)
			@D(8) @A byte[] spec_version_hash;//The first 8 bytes (not characters printed in hex!) of the git hash.
			@D(8) @A byte[] library_version_hash;//The first 8 bytes (not characters printed in hex!) of the git hash.
		}
		
		/**
		 * General status information of an UAVCAN node. Please refer to the definition of the UAVCAN message "uavcan.protocol.NodeStatus"
		 * for the background information. The UAVCAN specification is available at http://uavcan.org
		 */
		@id(310) class UAVCAN_NODE_STATUS {
			@A long time_usec;//Timestamp (microseconds since UNIX epoch or microseconds since system boot)
			@A int  uptime_sec;//The number of seconds since the start-up of the node.
			UAVCAN_NODE_HEALTH health;//Generalized node health status.
			UAVCAN_NODE_MODE   mode;//Generalized operating mode.
			@A byte  sub_mode;//Not used currently.
			@A short vendor_specific_status_code;//Vendor-specific status information.
		}
		
		/**
		 * General information describing a particular UAVCAN node. Please refer to the definition of the UAVCAN
		 * service "uavcan.protocol.GetNodeInfo" for the background information. This message should be emitted
		 * by the system whenever a new node appears online, or an existing node reboots. Additionally, it can be
		 * emitted upon request from the other end of the MAVLink channel (see MAV_CMD_UAVCAN_GET_NODE_INFO). It
		 * is also not prohibited to emit this message unconditionally at a low frequency. The UAVCAN specification
		 * is available at http://uavcan.org
		 */
		@id(311) class UAVCAN_NODE_INFO {
			@A        long   time_usec;//Timestamp (microseconds since UNIX epoch or microseconds since system boot)
			@A        int    uptime_sec;//The number of seconds since the start-up of the node.
			@A(80)    String name;//Node name string. For example, "sapog.px4.io".
			@A        byte   hw_version_major;//Hardware major version number.
			@A        byte   hw_version_minor;//Hardware minor version number.
			@D(16) @A byte[] hw_unique_id;//Hardware unique 128-bit ID.
			@A        byte   sw_version_major;//Software major version number.
			@A        byte   sw_version_minor;//Software minor version number.
			@A        int    sw_vcs_commit;//Version control system (VCS) revision identifier (e.g. git short commit hash). Zero if unknown.
		}
		
		/**
		 * Request to read the value of a parameter with the either the param_id string id or param_index.
		 */
		@id(320) class PARAM_EXT_REQUEST_READ {
			@A     byte   target_system;//System ID
			@A     byte   target_component;//Component ID
			/**
			 * Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination
			 * (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the
			 * ID is stored as strin
			 */
			@A(16) String param_id;
			short param_index;//Parameter index. Set to -1 to use the Parameter ID field as identifier (else param_id will be ignored
		}
		
		/**
		 * Request all parameters of this component. After this request, all parameters are emitted.
		 */
		@id(321) class PARAM_EXT_REQUEST_LIST {
			@A byte target_system;//System ID
			@A byte target_component;//Component ID
		}
		
		/**
		 * Emit the value of a parameter. The inclusion of param_count and param_index in the message allows the
		 * recipient to keep track of received parameters and allows them to re-request missing parameters after
		 * a loss or timeout
		 */
		@id(322) class PARAM_EXT_VALUE {
			/**
			 * Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination
			 * (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the
			 * ID is stored as strin
			 */
			@A(16)  String param_id;
			@A(128) String param_value;//Parameter value
			MAV_PARAM_EXT_TYPE param_type;//Parameter type: see the MAV_PARAM_EXT_TYPE enum for supported data types.
			@A short param_count;//Total number of parameters
			@A short param_index;//Index of this parameter
		}
		
		/**
		 * Set a parameter value. In order to deal with message loss (and retransmission of PARAM_EXT_SET), when
		 * setting a parameter value and the new value is the same as the current value, you will immediately get
		 * a PARAM_ACK_ACCEPTED response. If the current state is PARAM_ACK_IN_PROGRESS, you will accordingly receive
		 * a PARAM_ACK_IN_PROGRESS in response
		 */
		@id(323) class PARAM_EXT_SET {
			@A      byte   target_system;//System ID
			@A      byte   target_component;//Component ID
			/**
			 * Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination
			 * (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the
			 * ID is stored as strin
			 */
			@A(16)  String param_id;
			@A(128) String param_value;//Parameter value
			MAV_PARAM_EXT_TYPE param_type;//Parameter type: see the MAV_PARAM_EXT_TYPE enum for supported data types.
		}
		
		/**
		 * Response from a PARAM_EXT_SET message.
		 */
		@id(324) class PARAM_EXT_ACK {
			/**
			 * Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination
			 * (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the
			 * ID is stored as strin
			 */
			@A(16)  String param_id;
			@A(128) String param_value;//Parameter value (new value if PARAM_ACK_ACCEPTED, current value otherwise)
			MAV_PARAM_EXT_TYPE param_type;//Parameter type: see the MAV_PARAM_EXT_TYPE enum for supported data types.
			PARAM_ACK          param_result;//Result code: see the PARAM_ACK enum for possible codes.
		}
		
		/**
		 * Obstacle distances in front of the sensor, starting from the left in increment degrees to the right
		 */
		@id(330) class OBSTACLE_DISTANCE {
			@A long time_usec;//Timestamp (microseconds since system boot or since UNIX epoch)
			MAV_DISTANCE_SENSOR sensor_type;//Class id of the distance sensor type.
			/**
			 * Distance of obstacles in front of the sensor starting on the left side. A value of 0 means that the obstacle
			 * is right in front of the sensor. A value of max_distance +1 means no obstace is present. A value of UINT16_MAX
			 * for unknown/not used. In a array element, each unit corresponds to 1cm
			 */
			@D(72) @A short[] distances;
			@A        byte    increment;//Angular width in degrees of each array element.
			@A        short   min_distance;//Minimum distance the sensor can measure in centimeters
			@A        short   max_distance;//Maximum distance the sensor can measure in centimeters
		}
		
		/**
		 * Accelerometer and Gyro biases from the navigation filter
		 */
		@id(220) class NAV_FILTER_BIAS {
			@A long usec;//Timestamp (microseconds)
			float accel_0;//b_f[0]
			float accel_1;//b_f[1]
			float accel_2;//b_f[2]
			float gyro_0;//b_f[0]
			float gyro_1;//b_f[1]
			float gyro_2;//b_f[2]
		}
		
		/**
		 * Complete set of calibration parameters for the radio
		 */
		@id(221) class RADIO_CALIBRATION {
			@D(3) @A short[] aileron;//Aileron setpoints: left, center, right
			@D(3) @A short[] elevator;//Elevator setpoints: nose down, center, nose up
			@D(3) @A short[] rudder;//Rudder setpoints: nose left, center, nose right
			@D(2) @A short[] gyro;//Tail gyro mode/gain setpoints: heading hold, rate mode
			@D(5) @A short[] pitch;//Pitch curve setpoints (every 25%)
			@D(5) @A short[] throttle;//Throttle curve setpoints (every 25%)
		}
		
		/**
		 * System status specific to ualberta uav
		 */
		@id(222) class UALBERTA_SYS_STATUS {
			@A byte mode;//System mode, see UALBERTA_AUTOPILOT_MODE ENUM
			@A byte nav_mode;//Navigation mode, see UALBERTA_NAV_MODE ENUM
			@A byte pilot;//Pilot mode, see UALBERTA_PILOT_MODE
		}
	}
}

/**
 * Commands to be executed by the MAV. They can be executed on user request, or as part of a mission script.
 * If the action is used in a mission, the parameter mapping to the waypoint/mission message is as follows:
 * Param 1, Param 2, Param 3, Param 4, X: Param 5, Y:Param 6, Z:Param 7. This command list is similar what
 * ARINC 424 is for commercial aircraft: A data format how to interpret waypoint/mission data
 */
enum MAV_CMD {
	;
	final int
			
			/**
			 Navigate to waypoint.
			 1	Hold time in decimal seconds. (ignored by fixed wing, time to stay at waypoint for rotary wing)
			 2	Acceptance radius in meters (if the sphere with this radius is hit, the waypoint counts as reached)
			 3	0 to pass through the WP, if 	>	0 radius in meters to pass by WP. Positive value for clockwise orbit, negative value for counter-clockwise orbit. Allows trajectory control.
			 4	Desired yaw angle at waypoint (rotary wing). NaN for unchanged.
			 5	Latitude
			 6	Longitude
			 7	Altitude*/
			MAV_CMD_NAV_WAYPOINT = 16,
	/**
	 * Loiter around this waypoint an unlimited amount of time
	 * 1	Empty
	 * 2	Empty
	 * 3	Radius around waypoint, in meters. If positive loiter clockwise, else counter-clockwise
	 * 4	Desired yaw angle.
	 * 5	Latitude
	 * 6	Longitude
	 * 7	Altitude
	 */
	MAV_CMD_NAV_LOITER_UNLIM = 17,
	/**
	 * Loiter around this waypoint for X turns
	 * 1	Turns
	 * 2	Empty
	 * 3	Radius around waypoint, in meters. If positive loiter clockwise, else counter-clockwise
	 * 4	Forward moving aircraft this sets exit xtrack location: 0 for center of loiter wp, 1 for exit location. Else, this is desired yaw angle
	 * 5	Latitude
	 * 6	Longitude
	 * 7	Altitude
	 */
	MAV_CMD_NAV_LOITER_TURNS = 18,
	/**
	 * Loiter around this waypoint for X seconds
	 * 1	Seconds (decimal)
	 * 2	Empty
	 * 3	Radius around waypoint, in meters. If positive loiter clockwise, else counter-clockwise
	 * 4	Forward moving aircraft this sets exit xtrack location: 0 for center of loiter wp, 1 for exit location. Else, this is desired yaw angle
	 * 5	Latitude
	 * 6	Longitude
	 * 7	Altitude
	 */
	MAV_CMD_NAV_LOITER_TIME = 19,
	/**
	 * Return to launch location
	 * 1	Empty
	 * 2	Empty
	 * 3	Empty
	 * 4	Empty
	 * 5	Empty
	 * 6	Empty
	 * 7	Empty
	 */
	MAV_CMD_NAV_RETURN_TO_LAUNCH = 20,
	/**
	 * Land at location
	 * 1	Abort Alt
	 * 2	Empty
	 * 3	Empty
	 * 4	Desired yaw angle. NaN for unchanged.
	 * 5	Latitude
	 * 6	Longitude
	 * 7	Altitude (ground level)
	 */
	MAV_CMD_NAV_LAND = 21,
	/**
	 * Takeoff from ground / hand
	 * 1	Minimum pitch (if airspeed sensor present), desired pitch without sensor
	 * 2	Empty
	 * 3	Empty
	 * 4	Yaw angle (if magnetometer present), ignored without magnetometer. NaN for unchanged.
	 * 5	Latitude
	 * 6	Longitude
	 * 7	Altitude
	 */
	MAV_CMD_NAV_TAKEOFF = 22,
	/**
	 * Land at local position (local frame only)
	 * 1	Landing target number (if available)
	 * 2	Maximum accepted offset from desired landing position [m] - computed magnitude from spherical coordinates: d = sqrt(x^2 + y^2 + z^2), which gives the maximum accepted distance between the desired landing position and the position where the vehicle is about to land
	 * 3	Landing descend rate [ms^-1]
	 * 4	Desired yaw angle [rad]
	 * 5	Y-axis position [m]
	 * 6	X-axis position [m]
	 * 7	Z-axis / ground level position [m]
	 */
	MAV_CMD_NAV_LAND_LOCAL = 23,
	/**
	 * Takeoff from local position (local frame only)
	 * 1	Minimum pitch (if airspeed sensor present), desired pitch without sensor [rad]
	 * 2	Empty
	 * 3	Takeoff ascend rate [ms^-1]
	 * 4	Yaw angle [rad] (if magnetometer or another yaw estimation source present), ignored without one of these
	 * 5	Y-axis position [m]
	 * 6	X-axis position [m]
	 * 7	Z-axis position [m]
	 */
	MAV_CMD_NAV_TAKEOFF_LOCAL = 24,
	/**
	 * Vehicle following, i.e. this waypoint represents the position of a moving vehicle
	 * 1	Following logic to use (e.g. loitering or sinusoidal following) - depends on specific autopilot implementation
	 * 2	Ground speed of vehicle to be followed
	 * 3	Radius around waypoint, in meters. If positive loiter clockwise, else counter-clockwise
	 * 4	Desired yaw angle.
	 * 5	Latitude
	 * 6	Longitude
	 * 7	Altitude
	 */
	MAV_CMD_NAV_FOLLOW = 25,
	/**
	 * Continue on the current course and climb/descend to specified altitude.  When the altitude is reached
	 * continue to the next command (i.e., don't proceed to the next command until the desired altitude is reached
	 * 1	Climb or Descend (0 = Neutral, command completes when within 5m of this command's altitude, 1 = Climbing, command completes when at or above this command's altitude, 2 = Descending, command completes when at or below this command's altitude.
	 * 2	Empty
	 * 3	Empty
	 * 4	Empty
	 * 5	Empty
	 * 6	Empty
	 * 7	Desired altitude in meters
	 */
	MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT = 30,
	/**
	 * Begin loiter at the specified Latitude and Longitude.  If Lat=Lon=0, then loiter at the current position.
	 * Don't consider the navigation command complete (don't leave loiter) until the altitude has been reached.
	 * Additionally, if the Heading Required parameter is non-zero the  aircraft will not leave the loiter
	 * until heading toward the next waypoint.
	 * 1	Heading Required (0 = False)
	 * 2	Radius in meters. If positive loiter clockwise, negative counter-clockwise, 0 means no change to standard loiter.
	 * 3	Empty
	 * 4	Forward moving aircraft this sets exit xtrack location: 0 for center of loiter wp, 1 for exit location
	 * 5	Latitude
	 * 6	Longitude
	 * 7	Altitude
	 */
	MAV_CMD_NAV_LOITER_TO_ALT = 31,
	/**
	 * Being following a target
	 * 1	System ID (the system ID of the FOLLOW_TARGET beacon). Send 0 to disable follow-me and return to the default position hold mode
	 * 2	RESERVED
	 * 3	RESERVED
	 * 4	altitude flag: 0: Keep current altitude, 1: keep altitude difference to target, 2: go to a fixed altitude above home
	 * 5	altitude
	 * 6	RESERVED
	 * 7	TTL in seconds in which the MAV should go to the default position hold mode after a message rx timeout
	 */
	MAV_CMD_DO_FOLLOW = 32,
	/**
	 * Reposition the MAV after a follow target command has been sent
	 * 1	Camera q1 (where 0 is on the ray from the camera to the tracking device)
	 * 2	Camera q2
	 * 3	Camera q3
	 * 4	Camera q4
	 * 5	altitude offset from target (m)
	 * 6	X offset from target (m)
	 * 7	Y offset from target (m)
	 */
	MAV_CMD_DO_FOLLOW_REPOSITION = 33,
	/**
	 * Sets the region of interest (ROI) for a sensor set or the vehicle itself. This can then be used by the
	 * vehicles control system to control the vehicle attitude and the attitude of various sensors such as cameras
	 * 1	Region of intereset mode. (see MAV_ROI enum)
	 * 2	Waypoint index/ target ID. (see MAV_ROI enum)
	 * 3	ROI index (allows a vehicle to manage multiple ROI's)
	 * 4	Empty
	 * 5	x the location of the fixed ROI (see MAV_FRAME)
	 * 6	y
	 * 7	z
	 */
	MAV_CMD_NAV_ROI = 80,
	/**
	 * Control autonomous path planning on the MAV.
	 * 1	0: Disable local obstacle avoidance / local path planning (without resetting map), 1: Enable local path planning, 2: Enable and reset local path planning
	 * 2	0: Disable full path planning (without resetting map), 1: Enable, 2: Enable and reset map/occupancy grid, 3: Enable and reset planned route, but not occupancy grid
	 * 3	Empty
	 * 4	Yaw angle at goal, in compass degrees, [0..360]
	 * 5	Latitude/X of goal
	 * 6	Longitude/Y of goal
	 * 7	Altitude/Z of goal
	 */
	MAV_CMD_NAV_PATHPLANNING = 81,
	/**
	 * Navigate to waypoint using a spline path.
	 * 1	Hold time in decimal seconds. (ignored by fixed wing, time to stay at waypoint for rotary wing)
	 * 2	Empty
	 * 3	Empty
	 * 4	Empty
	 * 5	Latitude/X of goal
	 * 6	Longitude/Y of goal
	 * 7	Altitude/Z of goal
	 */
	MAV_CMD_NAV_SPLINE_WAYPOINT = 82,
	/**
	 * Takeoff from ground using VTOL mode
	 * 1	Empty
	 * 2	Front transition heading, see VTOL_TRANSITION_HEADING enum.
	 * 3	Empty
	 * 4	Yaw angle in degrees. NaN for unchanged.
	 * 5	Latitude
	 * 6	Longitude
	 * 7	Altitude
	 */
	MAV_CMD_NAV_VTOL_TAKEOFF = 84,
	/**
	 * Land using VTOL mode
	 * 1	Empty
	 * 2	Empty
	 * 3	Approach altitude (with the same reference as the Altitude field). NaN if unspecified.
	 * 4	Yaw angle in degrees. NaN for unchanged.
	 * 5	Latitude
	 * 6	Longitude
	 * 7	Altitude (ground level)
	 */
	MAV_CMD_NAV_VTOL_LAND = 85,
	/**
	 * hand control over to an external controller
	 * 1	On / Off (	>	0.5f on)
	 * 2	Empty
	 * 3	Empty
	 * 4	Empty
	 * 5	Empty
	 * 6	Empty
	 * 7	Empty
	 */
	MAV_CMD_NAV_GUIDED_ENABLE = 92,
	/**
	 * Delay the next navigation command a number of seconds or until a specified time
	 * 1	Delay in seconds (decimal, -1 to enable time-of-day fields)
	 * 2	hour (24h format, UTC, -1 to ignore)
	 * 3	minute (24h format, UTC, -1 to ignore)
	 * 4	second (24h format, UTC)
	 * 5	Empty
	 * 6	Empty
	 * 7	Empty
	 */
	MAV_CMD_NAV_DELAY = 93,
	/**
	 * Descend and place payload.  Vehicle descends until it detects a hanging payload has reached the ground,
	 * the gripper is opened to release the payloa
	 * 1	Maximum distance to descend (meters)
	 * 2	Empty
	 * 3	Empty
	 * 4	Empty
	 * 5	Latitude (deg * 1E7)
	 * 6	Longitude (deg * 1E7)
	 * 7	Altitude (meters)
	 */
	MAV_CMD_NAV_PAYLOAD_PLACE = 94,
	/**
	 * NOP - This command is only used to mark the upper limit of the NAV/ACTION commands in the enumeratio
	 * 1	Empty
	 * 2	Empty
	 * 3	Empty
	 * 4	Empty
	 * 5	Empty
	 * 6	Empty
	 * 7	Empty
	 */
	MAV_CMD_NAV_LAST = 95,
	/**
	 * Delay mission state machine.
	 * 1	Delay in seconds (decimal)
	 * 2	Empty
	 * 3	Empty
	 * 4	Empty
	 * 5	Empty
	 * 6	Empty
	 * 7	Empty
	 */
	MAV_CMD_CONDITION_DELAY = 112,
	/**
	 * Ascend/descend at rate.  Delay mission state machine until desired altitude reached.
	 * 1	Descent / Ascend rate (m/s)
	 * 2	Empty
	 * 3	Empty
	 * 4	Empty
	 * 5	Empty
	 * 6	Empty
	 * 7	Finish Altitude
	 */
	MAV_CMD_CONDITION_CHANGE_ALT = 113,
	/**
	 * Delay mission state machine until within desired distance of next NAV point.
	 * 1	Distance (meters)
	 * 2	Empty
	 * 3	Empty
	 * 4	Empty
	 * 5	Empty
	 * 6	Empty
	 * 7	Empty
	 */
	MAV_CMD_CONDITION_DISTANCE = 114,
	/**
	 * Reach a certain target angle.
	 * 1	target angle: [0-360], 0 is north
	 * 2	speed during yaw change:[deg per second]
	 * 3	direction: negative: counter clockwise, positive: clockwise [-1,1]
	 * 4	relative offset or absolute angle: [ 1,0]
	 * 5	Empty
	 * 6	Empty
	 * 7	Empty
	 */
	MAV_CMD_CONDITION_YAW = 115,
	/**
	 * NOP - This command is only used to mark the upper limit of the CONDITION commands in the enumeratio
	 * 1	Empty
	 * 2	Empty
	 * 3	Empty
	 * 4	Empty
	 * 5	Empty
	 * 6	Empty
	 * 7	Empty
	 */
	MAV_CMD_CONDITION_LAST = 159,
	/**
	 * Set system mode.
	 * 1	Mode, as defined by ENUM MAV_MODE
	 * 2	Custom mode - this is system specific, please refer to the individual autopilot specifications for details.
	 * 3	Custom sub mode - this is system specific, please refer to the individual autopilot specifications for details.
	 * 4	Empty
	 * 5	Empty
	 * 6	Empty
	 * 7	Empty
	 */
	MAV_CMD_DO_SET_MODE = 176,
	/**
	 * Jump to the desired command in the mission list.  Repeat this action only the specified number of time
	 * 1	Sequence number
	 * 2	Repeat count
	 * 3	Empty
	 * 4	Empty
	 * 5	Empty
	 * 6	Empty
	 * 7	Empty
	 */
	MAV_CMD_DO_JUMP = 177,
	/**
	 * Change speed and/or throttle set points.
	 * 1	Speed type (0=Airspeed, 1=Ground Speed)
	 * 2	Speed  (m/s, -1 indicates no change)
	 * 3	Throttle  ( Percent, -1 indicates no change)
	 * 4	absolute or relative [0,1]
	 * 5	Empty
	 * 6	Empty
	 * 7	Empty
	 */
	MAV_CMD_DO_CHANGE_SPEED = 178,
	/**
	 * Changes the home location either to the current location or a specified location.
	 * 1	Use current (1=use current location, 0=use specified location)
	 * 2	Empty
	 * 3	Empty
	 * 4	Empty
	 * 5	Latitude
	 * 6	Longitude
	 * 7	Altitude
	 */
	MAV_CMD_DO_SET_HOME = 179,
	/**
	 * Set a system parameter.  Caution!  Use of this command requires knowledge of the numeric enumeration value
	 * of the parameter
	 * 1	Parameter number
	 * 2	Parameter value
	 * 3	Empty
	 * 4	Empty
	 * 5	Empty
	 * 6	Empty
	 * 7	Empty
	 */
	MAV_CMD_DO_SET_PARAMETER = 180,
	/**
	 * Set a relay to a condition.
	 * 1	Relay number
	 * 2	Setting (1=on, 0=off, others possible depending on system hardware)
	 * 3	Empty
	 * 4	Empty
	 * 5	Empty
	 * 6	Empty
	 * 7	Empty
	 */
	MAV_CMD_DO_SET_RELAY = 181,
	/**
	 * Cycle a relay on and off for a desired number of cyles with a desired period.
	 * 1	Relay number
	 * 2	Cycle count
	 * 3	Cycle time (seconds, decimal)
	 * 4	Empty
	 * 5	Empty
	 * 6	Empty
	 * 7	Empty
	 */
	MAV_CMD_DO_REPEAT_RELAY = 182,
	/**
	 * Set a servo to a desired PWM value.
	 * 1	Servo number
	 * 2	PWM (microseconds, 1000 to 2000 typical)
	 * 3	Empty
	 * 4	Empty
	 * 5	Empty
	 * 6	Empty
	 * 7	Empty
	 */
	MAV_CMD_DO_SET_SERVO = 183,
	/**
	 * Cycle a between its nominal setting and a desired PWM for a desired number of cycles with a desired period
	 * 1	Servo number
	 * 2	PWM (microseconds, 1000 to 2000 typical)
	 * 3	Cycle count
	 * 4	Cycle time (seconds)
	 * 5	Empty
	 * 6	Empty
	 * 7	Empty
	 */
	MAV_CMD_DO_REPEAT_SERVO = 184,
	/**
	 * Terminate flight immediately
	 * 1	Flight termination activated if 	>	0.5
	 * 2	Empty
	 * 3	Empty
	 * 4	Empty
	 * 5	Empty
	 * 6	Empty
	 * 7	Empty
	 */
	MAV_CMD_DO_FLIGHTTERMINATION = 185,
	/**
	 * Change altitude set point.
	 * 1	Altitude in meters
	 * 2	Mav frame of new altitude (see MAV_FRAME)
	 * 3	Empty
	 * 4	Empty
	 * 5	Empty
	 * 6	Empty
	 * 7	Empty
	 */
	MAV_CMD_DO_CHANGE_ALTITUDE = 186,
	/**
	 * Mission command to perform a landing. This is used as a marker in a mission to tell the autopilot where
	 * a sequence of mission items that represents a landing starts. It may also be sent via a COMMAND_LONG
	 * to trigger a landing, in which case the nearest (geographically) landing sequence in the mission will
	 * be used. The Latitude/Longitude is optional, and may be set to 0 if not needed. If specified then it
	 * will be used to help find the closest landing sequence
	 * 1	Empty
	 * 2	Empty
	 * 3	Empty
	 * 4	Empty
	 * 5	Latitude
	 * 6	Longitude
	 * 7	Empty
	 */
	MAV_CMD_DO_LAND_START = 189,
	/**
	 * Mission command to perform a landing from a rally point.
	 * 1	Break altitude (meters)
	 * 2	Landing speed (m/s)
	 * 3	Empty
	 * 4	Empty
	 * 5	Empty
	 * 6	Empty
	 * 7	Empty
	 */
	MAV_CMD_DO_RALLY_LAND = 190,
	/**
	 * Mission command to safely abort an autonmous landing.
	 * 1	Altitude (meters)
	 * 2	Empty
	 * 3	Empty
	 * 4	Empty
	 * 5	Empty
	 * 6	Empty
	 * 7	Empty
	 */
	MAV_CMD_DO_GO_AROUND = 191,
	/**
	 * Reposition the vehicle to a specific WGS84 global position.
	 * 1	Ground speed, less than 0 (-1) for default
	 * 2	Bitmask of option flags, see the MAV_DO_REPOSITION_FLAGS enum.
	 * 3	Reserved
	 * 4	Yaw heading, NaN for unchanged. For planes indicates loiter direction (0: clockwise, 1: counter clockwise)
	 * 5	Latitude (deg * 1E7)
	 * 6	Longitude (deg * 1E7)
	 * 7	Altitude (meters)
	 */
	MAV_CMD_DO_REPOSITION = 192,
	/**
	 * If in a GPS controlled position mode, hold the current position or continue.
	 * 1	0: Pause current mission or reposition command, hold current position. 1: Continue mission. A VTOL capable vehicle should enter hover mode (multicopter and VTOL planes). A plane should loiter with the default loiter radius.
	 * 2	Reserved
	 * 3	Reserved
	 * 4	Reserved
	 * 5	Reserved
	 * 6	Reserved
	 * 7	Reserved
	 */
	MAV_CMD_DO_PAUSE_CONTINUE = 193,
	/**
	 * Set moving direction to forward or reverse.
	 * 1	Direction (0=Forward, 1=Reverse)
	 * 2	Empty
	 * 3	Empty
	 * 4	Empty
	 * 5	Empty
	 * 6	Empty
	 * 7	Empty
	 */
	MAV_CMD_DO_SET_REVERSE = 194,
	/**
	 * Control onboard camera system.
	 * 1	Camera ID (-1 for all)
	 * 2	Transmission: 0: disabled, 1: enabled compressed, 2: enabled raw
	 * 3	Transmission mode: 0: video stream, 	>	0: single images every n seconds (decimal)
	 * 4	Recording: 0: disabled, 1: enabled compressed, 2: enabled raw
	 * 5	Empty
	 * 6	Empty
	 * 7	Empty
	 */
	MAV_CMD_DO_CONTROL_VIDEO = 200,
	/**
	 * Sets the region of interest (ROI) for a sensor set or the vehicle itself. This can then be used by the
	 * vehicles control system to control the vehicle attitude and the attitude of various sensors such as cameras
	 * 1	Region of intereset mode. (see MAV_ROI enum)
	 * 2	Waypoint index/ target ID. (see MAV_ROI enum)
	 * 3	ROI index (allows a vehicle to manage multiple ROI's)
	 * 4	Empty
	 * 5	MAV_ROI_WPNEXT: pitch offset from next waypoint, MAV_ROI_LOCATION: latitude
	 * 6	MAV_ROI_WPNEXT: roll offset from next waypoint, MAV_ROI_LOCATION: longitude
	 * 7	MAV_ROI_WPNEXT: yaw offset from next waypoint, MAV_ROI_LOCATION: altitude
	 */
	MAV_CMD_DO_SET_ROI = 201,
	/**
	 * Mission command to configure an on-board camera controller system.
	 * 1	Modes: P, TV, AV, M, Etc
	 * 2	Shutter speed: Divisor number for one second
	 * 3	Aperture: F stop number
	 * 4	ISO number e.g. 80, 100, 200, Etc
	 * 5	Exposure type enumerator
	 * 6	Command Identity
	 * 7	Main engine cut-off time before camera trigger in seconds/10 (0 means no cut-off)
	 */
	MAV_CMD_DO_DIGICAM_CONFIGURE = 202,
	/**
	 * Mission command to control an on-board camera controller system.
	 * 1	Session control e.g. show/hide lens
	 * 2	Zoom's absolute position
	 * 3	Zooming step value to offset zoom from the current position
	 * 4	Focus Locking, Unlocking or Re-locking
	 * 5	Shooting Command
	 * 6	Command Identity
	 * 7	Test shot identifier. If set to 1, image will only be captured, but not counted towards internal frame count.
	 */
	MAV_CMD_DO_DIGICAM_CONTROL = 203,
	/**
	 * Mission command to configure a camera or antenna mount
	 * 1	Mount operation mode (see MAV_MOUNT_MODE enum)
	 * 2	stabilize roll? (1 = yes, 0 = no)
	 * 3	stabilize pitch? (1 = yes, 0 = no)
	 * 4	stabilize yaw? (1 = yes, 0 = no)
	 * 5	roll input (0 = angle, 1 = angular rate)
	 * 6	pitch input (0 = angle, 1 = angular rate)
	 * 7	yaw input (0 = angle, 1 = angular rate)
	 */
	MAV_CMD_DO_MOUNT_CONFIGURE = 204,
	/**
	 * Mission command to control a camera or antenna mount
	 * 1	pitch depending on mount mode (degrees or degrees/second depending on pitch input).
	 * 2	roll depending on mount mode (degrees or degrees/second depending on roll input).
	 * 3	yaw depending on mount mode (degrees or degrees/second depending on yaw input).
	 * 4	alt in meters depending on mount mode.
	 * 5	latitude in degrees * 1E7, set if appropriate mount mode.
	 * 6	longitude in degrees * 1E7, set if appropriate mount mode.
	 * 7	MAV_MOUNT_MODE enum value
	 */
	MAV_CMD_DO_MOUNT_CONTROL = 205,
	/**
	 * Mission command to set camera trigger distance for this flight. The camera is trigerred each time this
	 * distance is exceeded. This command can also be used to set the shutter integration time for the camera
	 * 1	Camera trigger distance (meters). 0 to stop triggering.
	 * 2	Camera shutter integration time (milliseconds). -1 or 0 to ignore
	 * 3	Trigger camera once immediately. (0 = no trigger, 1 = trigger)
	 * 4	Empty
	 * 5	Empty
	 * 6	Empty
	 * 7	Empty
	 */
	MAV_CMD_DO_SET_CAM_TRIGG_DIST = 206,
	/**
	 * Mission command to enable the geofence
	 * 1	enable? (0=disable, 1=enable, 2=disable_floor_only)
	 * 2	Empty
	 * 3	Empty
	 * 4	Empty
	 * 5	Empty
	 * 6	Empty
	 * 7	Empty
	 */
	MAV_CMD_DO_FENCE_ENABLE = 207,
	/**
	 * Mission command to trigger a parachute
	 * 1	action (0=disable, 1=enable, 2=release, for some systems see PARACHUTE_ACTION enum, not in general message set.)
	 * 2	Empty
	 * 3	Empty
	 * 4	Empty
	 * 5	Empty
	 * 6	Empty
	 * 7	Empty
	 */
	MAV_CMD_DO_PARACHUTE = 208,
	/**
	 * Mission command to perform motor test
	 * 1	motor sequence number (a number from 1 to max number of motors on the vehicle)
	 * 2	throttle type (0=throttle percentage, 1=PWM, 2=pilot throttle channel pass-through. See MOTOR_TEST_THROTTLE_TYPE enum)
	 * 3	throttle
	 * 4	timeout (in seconds)
	 * 5	Empty
	 * 6	Empty
	 * 7	Empty
	 */
	MAV_CMD_DO_MOTOR_TEST = 209,
	/**
	 * Change to/from inverted flight
	 * 1	inverted (0=normal, 1=inverted)
	 * 2	Empty
	 * 3	Empty
	 * 4	Empty
	 * 5	Empty
	 * 6	Empty
	 * 7	Empty
	 */
	MAV_CMD_DO_INVERTED_FLIGHT = 210,
	/**
	 * Sets a desired vehicle turn angle and speed change
	 * 1	yaw angle to adjust steering by in centidegress
	 * 2	speed - normalized to 0 .. 1
	 * 3	Empty
	 * 4	Empty
	 * 5	Empty
	 * 6	Empty
	 * 7	Empty
	 */
	MAV_CMD_NAV_SET_YAW_SPEED = 213,
	/**
	 * Mission command to set camera trigger interval for this flight. If triggering is enabled, the camera is
	 * triggered each time this interval expires. This command can also be used to set the shutter integration
	 * time for the camera
	 * 1	Camera trigger cycle time (milliseconds). -1 or 0 to ignore.
	 * 2	Camera shutter integration time (milliseconds). Should be less than trigger cycle time. -1 or 0 to ignore.
	 * 3	Empty
	 * 4	Empty
	 * 5	Empty
	 * 6	Empty
	 * 7	Empty
	 */
	MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL = 214,
	/**
	 * Mission command to control a camera or antenna mount, using a quaternion as reference.
	 * 1	q1 - quaternion param #1, w (1 in null-rotation)
	 * 2	q2 - quaternion param #2, x (0 in null-rotation)
	 * 3	q3 - quaternion param #3, y (0 in null-rotation)
	 * 4	q4 - quaternion param #4, z (0 in null-rotation)
	 * 5	Empty
	 * 6	Empty
	 * 7	Empty
	 */
	MAV_CMD_DO_MOUNT_CONTROL_QUAT = 220,
	/**
	 * set id of master controller
	 * 1	System ID
	 * 2	Component ID
	 * 3	Empty
	 * 4	Empty
	 * 5	Empty
	 * 6	Empty
	 * 7	Empty
	 */
	MAV_CMD_DO_GUIDED_MASTER = 221,
	/**
	 * set limits for external control
	 * 1	timeout - maximum time (in seconds) that external controller will be allowed to control vehicle. 0 means no timeout
	 * 2	absolute altitude min (in meters, AMSL) - if vehicle moves below this alt, the command will be aborted and the mission will continue.  0 means no lower altitude limit
	 * 3	absolute altitude max (in meters)- if vehicle moves above this alt, the command will be aborted and the mission will continue.  0 means no upper altitude limit
	 * 4	horizontal move limit (in meters, AMSL) - if vehicle moves more than this distance from it's location at the moment the command was executed, the command will be aborted and the mission will continue. 0 means no horizontal altitude limit
	 * 5	Empty
	 * 6	Empty
	 * 7	Empty
	 */
	MAV_CMD_DO_GUIDED_LIMITS = 222,
	/**
	 * Control vehicle engine. This is interpreted by the vehicles engine controller to change the target engine
	 * state. It is intended for vehicles with internal combustion engine
	 * 1	0: Stop engine, 1:Start Engine
	 * 2	0: Warm start, 1:Cold start. Controls use of choke where applicable
	 * 3	Height delay (meters). This is for commanding engine start only after the vehicle has gained the specified height. Used in VTOL vehicles during takeoff to start engine after the aircraft is off the ground. Zero for no delay.
	 * 4	Empty
	 * 5	Empty
	 * 5	Empty
	 * 6	Empty
	 * 7	Empty
	 */
	MAV_CMD_DO_ENGINE_CONTROL = 223,
	/**
	 * NOP - This command is only used to mark the upper limit of the DO commands in the enumeration
	 * 1	Empty
	 * 2	Empty
	 * 3	Empty
	 * 4	Empty
	 * 5	Empty
	 * 6	Empty
	 * 7	Empty
	 */
	MAV_CMD_DO_LAST = 240,
	/**
	 * Trigger calibration. This command will be only accepted if in pre-flight mode. Except for Temperature
	 * Calibration, only one sensor should be set in a single message and all others should be zero
	 * 1	1: gyro calibration, 3: gyro temperature calibration
	 * 2	1: magnetometer calibration
	 * 3	1: ground pressure calibration
	 * 4	1: radio RC calibration, 2: RC trim calibration
	 * 5	1: accelerometer calibration, 2: board level calibration, 3: accelerometer temperature calibration
	 * 6	1: APM: compass/motor interference calibration (PX4: airspeed calibration, deprecated), 2: airspeed calibration
	 * 7	1: ESC calibration, 3: barometer temperature calibration
	 */
	MAV_CMD_PREFLIGHT_CALIBRATION = 241,
	/**
	 * Set sensor offsets. This command will be only accepted if in pre-flight mode.
	 * 1	Sensor to adjust the offsets for: 0: gyros, 1: accelerometer, 2: magnetometer, 3: barometer, 4: optical flow, 5: second magnetometer, 6: third magnetometer
	 * 2	X axis offset (or generic dimension 1), in the sensor's raw units
	 * 3	Y axis offset (or generic dimension 2), in the sensor's raw units
	 * 4	Z axis offset (or generic dimension 3), in the sensor's raw units
	 * 5	Generic dimension 4, in the sensor's raw units
	 * 6	Generic dimension 5, in the sensor's raw units
	 * 7	Generic dimension 6, in the sensor's raw units
	 */
	MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS = 242,
	/**
	 * Trigger UAVCAN config. This command will be only accepted if in pre-flight mode.
	 * 1	1: Trigger actuator ID assignment and direction mapping.
	 * 2	Reserved
	 * 3	Reserved
	 * 4	Reserved
	 * 5	Reserved
	 * 6	Reserved
	 * 7	Reserved
	 */
	MAV_CMD_PREFLIGHT_UAVCAN = 243,
	/**
	 * Request storage of different parameter values and logs. This command will be only accepted if in pre-flight
	 * mode
	 * 1	Parameter storage: 0: READ FROM FLASH/EEPROM, 1: WRITE CURRENT TO FLASH/EEPROM, 2: Reset to defaults
	 * 2	Mission storage: 0: READ FROM FLASH/EEPROM, 1: WRITE CURRENT TO FLASH/EEPROM, 2: Reset to defaults
	 * 3	Onboard logging: 0: Ignore, 1: Start default rate logging, -1: Stop logging, 	>	1: start logging with rate of param 3 in Hz (e.g. set to 1000 for 1000 Hz logging)
	 * 4	Reserved
	 * 5	Empty
	 * 6	Empty
	 * 7	Empty
	 */
	MAV_CMD_PREFLIGHT_STORAGE = 245,
	/**
	 * Request the reboot or shutdown of system components.
	 * 1	0: Do nothing for autopilot, 1: Reboot autopilot, 2: Shutdown autopilot, 3: Reboot autopilot and keep it in the bootloader until upgraded.
	 * 2	0: Do nothing for onboard computer, 1: Reboot onboard computer, 2: Shutdown onboard computer, 3: Reboot onboard computer and keep it in the bootloader until upgraded.
	 * 3	WIP: 0: Do nothing for camera, 1: Reboot onboard camera, 2: Shutdown onboard camera, 3: Reboot onboard camera and keep it in the bootloader until upgraded
	 * 4	WIP: 0: Do nothing for mount (e.g. gimbal), 1: Reboot mount, 2: Shutdown mount, 3: Reboot mount and keep it in the bootloader until upgraded
	 * 5	Reserved, send 0
	 * 6	Reserved, send 0
	 * 7	WIP: ID (e.g. camera ID -1 for all IDs)
	 */
	MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN = 246,
	/**
	 * Hold / continue the current action
	 * 1	MAV_GOTO_DO_HOLD: hold MAV_GOTO_DO_CONTINUE: continue with next item in mission plan
	 * 2	MAV_GOTO_HOLD_AT_CURRENT_POSITION: Hold at current position MAV_GOTO_HOLD_AT_SPECIFIED_POSITION: hold at specified position
	 * 3	MAV_FRAME coordinate frame of hold point
	 * 4	Desired yaw angle in degrees
	 * 5	Latitude / X position
	 * 6	Longitude / Y position
	 * 7	Altitude / Z position
	 */
	MAV_CMD_OVERRIDE_GOTO = 252,
	/**
	 * start running a mission
	 * 1	first_item: the first mission item to run
	 * 2	last_item:  the last mission item to run (after this item is run, the mission ends)
	 */
	MAV_CMD_MISSION_START = 300,
	/**
	 * Arms / Disarms a component
	 * 1	1 to arm, 0 to disarm
	 */
	MAV_CMD_COMPONENT_ARM_DISARM = 400,
	/**
	 * Request the home position from the vehicle.
	 * 1	Reserved
	 * 2	Reserved
	 * 3	Reserved
	 * 4	Reserved
	 * 5	Reserved
	 * 6	Reserved
	 * 7	Reserved
	 */
	MAV_CMD_GET_HOME_POSITION = 410,
	/**
	 * Starts receiver pairing
	 * 1	0:Spektrum
	 * 2	0:Spektrum DSM2, 1:Spektrum DSMX
	 */
	MAV_CMD_START_RX_PAIR = 500,
	/**
	 * Request the interval between messages for a particular MAVLink message ID
	 * 1	The MAVLink message ID
	 */
	MAV_CMD_GET_MESSAGE_INTERVAL = 510,
	/**
	 * Request the interval between messages for a particular MAVLink message ID. This interface replaces REQUEST_DATA_STREA
	 * 1	The MAVLink message ID
	 * 2	The interval between two messages, in microseconds. Set to -1 to disable and 0 to request default rate.
	 */
	MAV_CMD_SET_MESSAGE_INTERVAL = 511,
	/**
	 * Request MAVLink protocol version compatibility
	 * 1	1: Request supported protocol versions by all nodes on the network
	 * 2	Reserved (all remaining params)
	 */
	MAV_CMD_REQUEST_PROTOCOL_VERSION = 519,
	/**
	 * Request autopilot capabilities
	 * 1	1: Request autopilot version
	 * 2	Reserved (all remaining params)
	 */
	MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES = 520,
	/**
	 * WIP: Request camera information (CAMERA_INFORMATION).
	 * 1	0: No action 1: Request camera capabilities
	 * 2	Reserved (all remaining params)
	 */
	MAV_CMD_REQUEST_CAMERA_INFORMATION = 521,
	/**
	 * WIP: Request camera settings (CAMERA_SETTINGS).
	 * 1	0: No Action 1: Request camera settings
	 * 2	Reserved (all remaining params)
	 */
	MAV_CMD_REQUEST_CAMERA_SETTINGS = 522,
	/**
	 * WIP: Request storage information (STORAGE_INFORMATION). Use the command's target_component to target a
	 * specific component's storage
	 * 1	Storage ID (0 for all, 1 for first, 2 for second, etc.)
	 * 2	0: No Action 1: Request storage information
	 * 3	Reserved (all remaining params)
	 */
	MAV_CMD_REQUEST_STORAGE_INFORMATION = 525,
	/**
	 * WIP: Format a storage medium. Once format is complete, a STORAGE_INFORMATION message is sent. Use the
	 * command's target_component to target a specific component's storage
	 * 1	Storage ID (1 for first, 2 for second, etc.)
	 * 2	0: No action 1: Format storage
	 * 3	Reserved (all remaining params)
	 */
	MAV_CMD_STORAGE_FORMAT = 526,
	/**
	 * WIP: Request camera capture status (CAMERA_CAPTURE_STATUS)
	 * 1	0: No Action 1: Request camera capture status
	 * 2	Reserved (all remaining params)
	 */
	MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS = 527,
	/**
	 * WIP: Request flight information (FLIGHT_INFORMATION)
	 * 1	1: Request flight information
	 * 2	Reserved (all remaining params)
	 */
	MAV_CMD_REQUEST_FLIGHT_INFORMATION = 528,
	/**
	 * WIP: Reset all camera settings to Factory Default
	 * 1	0: No Action 1: Reset all settings
	 * 2	Reserved (all remaining params)
	 */
	MAV_CMD_RESET_CAMERA_SETTINGS = 529,
	/**
	 * Set camera running mode. Use NAN for reserved values.
	 * 1	Reserved (Set to 0)
	 * 2	Camera mode (see CAMERA_MODE enum)
	 * 3	Reserved (all remaining params)
	 */
	MAV_CMD_SET_CAMERA_MODE = 530,
	/**
	 * Start image capture sequence. Sends CAMERA_IMAGE_CAPTURED after each capture. Use NAN for reserved values
	 * 1	Reserved (Set to 0)
	 * 2	Duration between two consecutive pictures (in seconds)
	 * 3	Number of images to capture total - 0 for unlimited capture
	 * 4	Reserved (all remaining params)
	 */
	MAV_CMD_IMAGE_START_CAPTURE = 2000,
	/**
	 * Stop image capture sequence Use NAN for reserved values.
	 * 1	Reserved (Set to 0)
	 * 2	Reserved (all remaining params)
	 */
	MAV_CMD_IMAGE_STOP_CAPTURE = 2001,
	/**
	 * WIP: Re-request a CAMERA_IMAGE_CAPTURE packet. Use NAN for reserved values.
	 * 1	Sequence number for missing CAMERA_IMAGE_CAPTURE packet
	 * 2	Reserved (all remaining params)
	 */
	MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE = 2002,
	/**
	 * Enable or disable on-board camera triggering system.
	 * 1	Trigger enable/disable (0 for disable, 1 for start), -1 to ignore
	 * 2	1 to reset the trigger sequence, -1 or 0 to ignore
	 * 3	1 to pause triggering, but without switching the camera off or retracting it. -1 to ignore
	 */
	MAV_CMD_DO_TRIGGER_CONTROL = 2003,
	/**
	 * Starts video capture (recording). Use NAN for reserved values.
	 * 1	Reserved (Set to 0)
	 * 2	Frequency CAMERA_CAPTURE_STATUS messages should be sent while recording (0 for no messages, otherwise frequency in Hz)
	 * 3	Reserved (all remaining params)
	 */
	MAV_CMD_VIDEO_START_CAPTURE = 2500,
	/**
	 * Stop the current video capture (recording). Use NAN for reserved values.
	 * 1	Reserved (Set to 0)
	 * 2	Reserved (all remaining params)
	 */
	MAV_CMD_VIDEO_STOP_CAPTURE = 2501,
	/**
	 * WIP: Start video streaming
	 * 1	Camera ID (0 for all cameras, 1 for first, 2 for second, etc.)
	 * 2	Reserved
	 */
	MAV_CMD_VIDEO_START_STREAMING = 2502,
	/**
	 * WIP: Stop the current video streaming
	 * 1	Camera ID (0 for all cameras, 1 for first, 2 for second, etc.)
	 * 2	Reserved
	 */
	MAV_CMD_VIDEO_STOP_STREAMING = 2503,
	/**
	 * WIP: Request video stream information (VIDEO_STREAM_INFORMATION)
	 * 1	Camera ID (0 for all cameras, 1 for first, 2 for second, etc.)
	 * 2	0: No Action 1: Request video stream information
	 * 3	Reserved (all remaining params)
	 */
	MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION = 2504,
	/**
	 * Request to start streaming logging data over MAVLink (see also LOGGING_DATA message)
	 * 1	Format: 0: ULog
	 * 2	Reserved (set to 0)
	 * 3	Reserved (set to 0)
	 * 4	Reserved (set to 0)
	 * 5	Reserved (set to 0)
	 * 6	Reserved (set to 0)
	 * 7	Reserved (set to 0)
	 */
	MAV_CMD_LOGGING_START = 2510,
	/**
	 * Request to stop streaming log data over MAVLink
	 * 1	Reserved (set to 0)
	 * 2	Reserved (set to 0)
	 * 3	Reserved (set to 0)
	 * 4	Reserved (set to 0)
	 * 5	Reserved (set to 0)
	 * 6	Reserved (set to 0)
	 * 7	Reserved (set to 0)
	 */
	MAV_CMD_LOGGING_STOP = 2511,
	/**
	 * 1	Landing gear ID (default: 0, -1 for all)
	 * 2	Landing gear position (Down: 0, Up: 1, NAN for no change)
	 * 3	Reserved, set to NAN
	 * 4	Reserved, set to NAN
	 * 5	Reserved, set to NAN
	 * 6	Reserved, set to NAN
	 * 7	Reserved, set to NAN
	 */
	MAV_CMD_AIRFRAME_CONFIGURATION = 2520,
	/**
	 * Create a panorama at the current position
	 * 1	Viewing angle horizontal of the panorama (in degrees, +- 0.5 the total angle)
	 * 2	Viewing angle vertical of panorama (in degrees)
	 * 3	Speed of the horizontal rotation (in degrees per second)
	 * 4	Speed of the vertical rotation (in degrees per second)
	 */
	MAV_CMD_PANORAMA_CREATE = 2800,
	/**
	 * Request VTOL transition
	 * 1	The target VTOL state, as defined by ENUM MAV_VTOL_STATE. Only MAV_VTOL_STATE_MC and MAV_VTOL_STATE_FW can be used.
	 */
	MAV_CMD_DO_VTOL_TRANSITION = 3000,
	/**
	 * This command sets the submode to standard guided when vehicle is in guided mode. The vehicle holds position and altitude and the user can input the desired velocites along all three axes.
	 */
	MAV_CMD_SET_GUIDED_SUBMODE_STANDARD = 4000,
	/**
	 * This command sets submode circle when vehicle is in guided mode. Vehicle flies along a circle facing the center of the circle. The user can input the velocity along the circle and change the radius. If no input is given the vehicle will hold position.
	 * <p>
	 * 1	Radius of desired circle in CIRCLE_MODE
	 * 2	User defined
	 * 3	User defined
	 * 4	User defined
	 * 5	Unscaled target latitude of center of circle in CIRCLE_MODE
	 * 6	Unscaled target longitude of center of circle in CIRCLE_MODE
	 */
	MAV_CMD_SET_GUIDED_SUBMODE_CIRCLE = 4001,
	/**
	 * WIP: Delay mission state machine until gate has been reached.
	 * 1	Geometry: 0: orthogonal to path between previous and next waypoint.
	 * 2	Altitude: 0: ignore altitude
	 * 3	Empty
	 * 4	Empty
	 * 5	Latitude
	 * 6	Longitude
	 * 7	Altitude
	 */
	MAV_CMD_CONDITION_GATE = 4501,
	/**
	 * Request authorization to arm the vehicle to a external entity, the arm authorizer is resposible to request all data that is needs from the vehicle before authorize or deny the request. If approved the progress of command_ack message should be set with period of time that this authorization is valid in seconds or in case it was denied it should be set with one of the reasons in ARM_AUTH_DENIED_REASON.
	 * <p>
	 * 1	Vehicle system id, this way ground station can request arm authorization on behalf of any vehicle
	 */
	MAV_CMD_ARM_AUTHORIZATION_REQUEST = 3001,
	/**
	 * Fence return point. There can only be one fence return point.
	 * <p>
	 * 1	Reserved
	 * 2	Reserved
	 * 3	Reserved
	 * 4	Reserved
	 * 5	Latitude
	 * 6	Longitude
	 * 7	Altitude
	 */
	MAV_CMD_NAV_FENCE_RETURN_POINT = 5000,
	/**
	 * Fence vertex for an inclusion polygon (the polygon must not be self-intersecting). The vehicle must stay within this area. Minimum of 3 vertices required.
	 * <p>
	 * 1	Polygon vertex count
	 * 2	Reserved
	 * 3	Reserved
	 * 4	Reserved
	 * 5	Latitude
	 * 6	Longitude
	 * 7	Reserved
	 */
	MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION = 5001,
	/**
	 * Fence vertex for an exclusion polygon (the polygon must not be self-intersecting). The vehicle must stay outside this area. Minimum of 3 vertices required.
	 * <p>
	 * 1	Polygon vertex count
	 * 2	Reserved
	 * 3	Reserved
	 * 4	Reserved
	 * 5	Latitude
	 * 6	Longitude
	 * 7	Reserved
	 */
	MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION = 5002,
	/**
	 * Circular fence area. The vehicle must stay inside this area.
	 * <p>
	 * 1	radius in meters
	 * 2	Reserved
	 * 3	Reserved
	 * 4	Reserved
	 * 5	Latitude
	 * 6	Longitude
	 * 7	Reserved
	 */
	MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION = 5003,
	/**
	 * Circular fence area. The vehicle must stay outside this area.
	 * <p>
	 * 1	radius in meters
	 * 2	Reserved
	 * 3	Reserved
	 * 4	Reserved
	 * 5	Latitude
	 * 6	Longitude
	 * 7	Reserved
	 */
	MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION = 5004,
	/**
	 * Rally point. You can have multiple rally points defined.
	 * <p>
	 * 1	Reserved
	 * 2	Reserved
	 * 3	Reserved
	 * 4	Reserved
	 * 5	Latitude
	 * 6	Longitude
	 * 7	Altitude
	 */
	MAV_CMD_NAV_RALLY_POINT = 5100,
	/**
	 * Commands the vehicle to respond with a sequence of messages UAVCAN_NODE_INFO, one message per every UAVCAN
	 * node that is online. Note that some of the response messages can be lost, which the receiver can detect
	 * easily by checking whether every received UAVCAN_NODE_STATUS has a matching message UAVCAN_NODE_INFO
	 * received earlier; if not, this command should be sent again in order to request re-transmission of the
	 * node information messages
	 * 1	Reserved (set to 0)
	 * 2	Reserved (set to 0)
	 * 3	Reserved (set to 0)
	 * 4	Reserved (set to 0)
	 * 5	Reserved (set to 0)
	 * 6	Reserved (set to 0)
	 * 7	Reserved (set to 0)
	 */
	MAV_CMD_UAVCAN_GET_NODE_INFO = 5200,
	/**
	 * Deploy payload on a Lat / Lon / Alt position. This includes the navigation to reach the required release
	 * position and velocity
	 * 1	Operation mode. 0: prepare single payload deploy (overwriting previous requests), but do not execute it. 1: execute payload deploy immediately (rejecting further deploy commands during execution, but allowing abort). 2: add payload deploy to existing deployment list.
	 * 2	Desired approach vector in degrees compass heading (0..360). A negative value indicates the system can define the approach vector at will.
	 * 3	Desired ground speed at release time. This can be overriden by the airframe in case it needs to meet minimum airspeed. A negative value indicates the system can define the ground speed at will.
	 * 4	Minimum altitude clearance to the release position in meters. A negative value indicates the system can define the clearance at will.
	 * 5	Latitude unscaled for MISSION_ITEM or in 1e7 degrees for MISSION_ITEM_INT
	 * 6	Longitude unscaled for MISSION_ITEM or in 1e7 degrees for MISSION_ITEM_INT
	 * 7	Altitude, in meters AMSL
	 */
	MAV_CMD_PAYLOAD_PREPARE_DEPLOY = 30001,
	/**
	 * Control the payload deployment.
	 * 1	Operation mode. 0: Abort deployment, continue normal mission. 1: switch to payload deploment mode. 100: delete first payload deployment request. 101: delete all payload deployment requests.
	 * 2	Reserved
	 * 3	Reserved
	 * 4	Reserved
	 * 5	Reserved
	 * 6	Reserved
	 * 7	Reserved
	 */
	MAV_CMD_PAYLOAD_CONTROL_DEPLOY = 30002,
	/**
	 * User defined waypoint item. Ground Station will show the Vehicle as flying through this item.
	 * 1	User defined
	 * 2	User defined
	 * 3	User defined
	 * 4	User defined
	 * 5	Latitude unscaled
	 * 6	Longitude unscaled
	 * 7	Altitude, in meters AMSL
	 */
	MAV_CMD_WAYPOINT_USER_1 = 31000,
	/**
	 * User defined waypoint item. Ground Station will show the Vehicle as flying through this item.
	 * 1	User defined
	 * 2	User defined
	 * 3	User defined
	 * 4	User defined
	 * 5	Latitude unscaled
	 * 6	Longitude unscaled
	 * 7	Altitude, in meters AMSL
	 */
	MAV_CMD_WAYPOINT_USER_2 = 31001,
	/**
	 * User defined waypoint item. Ground Station will show the Vehicle as flying through this item.
	 * 1	User defined
	 * 2	User defined
	 * 3	User defined
	 * 4	User defined
	 * 5	Latitude unscaled
	 * 6	Longitude unscaled
	 * 7	Altitude, in meters AMSL
	 */
	MAV_CMD_WAYPOINT_USER_3 = 31002,
	/**
	 * User defined waypoint item. Ground Station will show the Vehicle as flying through this item.
	 * 1	User defined
	 * 2	User defined
	 * 3	User defined
	 * 4	User defined
	 * 5	Latitude unscaled
	 * 6	Longitude unscaled
	 * 7	Altitude, in meters AMSL
	 */
	MAV_CMD_WAYPOINT_USER_4 = 31003,
	/**
	 * User defined waypoint item. Ground Station will show the Vehicle as flying through this item.
	 * 1	User defined
	 * 2	User defined
	 * 3	User defined
	 * 4	User defined
	 * 5	Latitude unscaled
	 * 6	Longitude unscaled
	 * 7	Altitude, in meters AMSL
	 */
	MAV_CMD_WAYPOINT_USER_5 = 31004,
	/**
	 * User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example:
	 * ROI item
	 * 1	User defined
	 * 2	User defined
	 * 3	User defined
	 * 4	User defined
	 * 5	Latitude unscaled
	 * 6	Longitude unscaled
	 * 7	Altitude, in meters AMSL
	 */
	MAV_CMD_SPATIAL_USER_1 = 31005,
	/**
	 * User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example:
	 * ROI item
	 * 1	User defined
	 * 2	User defined
	 * 3	User defined
	 * 4	User defined
	 * 5	Latitude unscaled
	 * 6	Longitude unscaled
	 * 7	Altitude, in meters AMSL
	 */
	MAV_CMD_SPATIAL_USER_2 = 31006,
	/**
	 * User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example:
	 * ROI item
	 * 1	User defined
	 * 2	User defined
	 * 3	User defined
	 * 4	User defined
	 * 5	Latitude unscaled
	 * 6	Longitude unscaled
	 * 7	Altitude, in meters AMSL
	 */
	MAV_CMD_SPATIAL_USER_3 = 31007,
	/**
	 * User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example:
	 * ROI item
	 * 1	User defined
	 * 2	User defined
	 * 3	User defined
	 * 4	User defined
	 * 5	Latitude unscaled
	 * 6	Longitude unscaled
	 * 7	Altitude, in meters AMSL
	 */
	MAV_CMD_SPATIAL_USER_4 = 31008,
	/**
	 * User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example:
	 * ROI item
	 * 1	User defined
	 * 2	User defined
	 * 3	User defined
	 * 4	User defined
	 * 5	Latitude unscaled
	 * 6	Longitude unscaled
	 * 7	Altitude, in meters AMSL
	 */
	MAV_CMD_SPATIAL_USER_5 = 31009,
	/**
	 * User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER
	 * item
	 * 1	User defined
	 * 2	User defined
	 * 3	User defined
	 * 4	User defined
	 * 5	User defined
	 * 6	User defined
	 * 7	User defined
	 */
	MAV_CMD_USER_1 = 31010,
	/**
	 * User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER
	 * item
	 * 1	User defined
	 * 2	User defined
	 * 3	User defined
	 * 4	User defined
	 * 5	User defined
	 * 6	User defined
	 * 7	User defined
	 */
	MAV_CMD_USER_2 = 31011,
	/**
	 * User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER
	 * item
	 * 1	User defined
	 * 2	User defined
	 * 3	User defined
	 * 4	User defined
	 * 5	User defined
	 * 6	User defined
	 * 7	User defined
	 */
	MAV_CMD_USER_3 = 31012,
	/**
	 * User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER
	 * item
	 * 1	User defined
	 * 2	User defined
	 * 3	User defined
	 * 4	User defined
	 * 5	User defined
	 * 6	User defined
	 * 7	User defined
	 */
	MAV_CMD_USER_4 = 31013,
	/**
	 * User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER
	 * item
	 * 1	User defined
	 * 2	User defined
	 * 3	User defined
	 * 4	User defined
	 * 5	User defined
	 * 6	User defined
	 * 7	User defined
	 */
	MAV_CMD_USER_5 = 31014;
}

/**
 * Micro air vehicle / autopilot classes. This identifies the individual model.
 */
enum MAV_AUTOPILOT {
	;
	
	final int
			MAV_AUTOPILOT_GENERIC                                      = 0, //Generic autopilot, full support for everything
			MAV_AUTOPILOT_RESERVED                                     = 1, //Reserved for future use.
			MAV_AUTOPILOT_SLUGS                                        = 2, //SLUGS autopilot, http://slugsuav.soe.ucsc.edu
			MAV_AUTOPILOT_ARDUPILOTMEGA                                = 3, //ArduPilotMega / ArduCopter, http://diydrones.com
			MAV_AUTOPILOT_OPENPILOT                                    = 4, //OpenPilot, http://openpilot.org
			MAV_AUTOPILOT_GENERIC_WAYPOINTS_ONLY                       = 5, //Generic autopilot only supporting simple waypoints
			MAV_AUTOPILOT_GENERIC_WAYPOINTS_AND_SIMPLE_NAVIGATION_ONLY = 6, //Generic autopilot supporting waypoints and other simple navigation commands
			MAV_AUTOPILOT_GENERIC_MISSION_FULL                         = 7, //Generic autopilot supporting the full mission command set
			MAV_AUTOPILOT_INVALID                                      = 8, //No valid autopilot, e.g. a GCS or other MAVLink component
			MAV_AUTOPILOT_PPZ                                          = 9, //PPZ UAV - http://nongnu.org/paparazzi
			MAV_AUTOPILOT_UDB                                          = 10, //UAV Dev Board
			MAV_AUTOPILOT_FP                                           = 11, //FlexiPilot
			MAV_AUTOPILOT_PX4                                          = 12, //PX4 Autopilot - http://pixhawk.ethz.ch/px4/
			MAV_AUTOPILOT_SMACCMPILOT                                  = 13, //SMACCMPilot - http://smaccmpilot.org
			MAV_AUTOPILOT_AUTOQUAD                                     = 14, //AutoQuad -- http://autoquad.org
			MAV_AUTOPILOT_ARMAZILA                                     = 15, //Armazila -- http://armazila.com
			MAV_AUTOPILOT_AEROB                                        = 16, //Aerob -- http://aerob.ru
			MAV_AUTOPILOT_ASLUAV                                       = 17, //ASLUAV autopilot -- http://www.asl.ethz.ch
			MAV_AUTOPILOT_SMARTAP                                      = 18;  //SmartAP Autopilot - http://sky-drones.com
}

/**
 * Generic micro air vehicle.
 */
enum MAV_TYPE {
	;
	
	final int
			MAV_TYPE_GENERIC            = 0, //Generic micro air vehicle.
			MAV_TYPE_FIXED_WING         = 1, //Fixed wing aircraft.
			MAV_TYPE_QUADROTOR          = 2, //Quadrotor
			MAV_TYPE_COAXIAL            = 3, //Coaxial helicopter
			MAV_TYPE_HELICOPTER         = 4, //Normal helicopter with tail rotor.
			MAV_TYPE_ANTENNA_TRACKER    = 5, //Ground installation
			MAV_TYPE_GCS                = 6, //Operator control unit / ground control station
			MAV_TYPE_AIRSHIP            = 7, //Airship, controlled
			MAV_TYPE_FREE_BALLOON       = 8, //Free balloon, uncontrolled
			MAV_TYPE_ROCKET             = 9, //Rocket
			MAV_TYPE_GROUND_ROVER       = 10, //Ground rover
			MAV_TYPE_SURFACE_BOAT       = 11, //Surface vessel, boat, ship
			MAV_TYPE_SUBMARINE          = 12, //Submarine
			MAV_TYPE_HEXAROTOR          = 13, //Hexarotor
			MAV_TYPE_OCTOROTOR          = 14, //Octorotor
			MAV_TYPE_TRICOPTER          = 15, //Tricopter
			MAV_TYPE_FLAPPING_WING      = 16, //Flapping wing
			MAV_TYPE_KITE               = 17, //Kite
			MAV_TYPE_ONBOARD_CONTROLLER = 18, //Onboard companion controller
			MAV_TYPE_VTOL_DUOROTOR      = 19, //Two-rotor VTOL using control surfaces in vertical operation in addition. Tailsitter.
			MAV_TYPE_VTOL_QUADROTOR     = 20, //Quad-rotor VTOL using a V-shaped quad config in vertical operation. Tailsitter.
			MAV_TYPE_VTOL_TILTROTOR     = 21, //Tiltrotor VTOL
			MAV_TYPE_VTOL_RESERVED2     = 22, //VTOL reserved 2
			MAV_TYPE_VTOL_RESERVED3     = 23, //VTOL reserved 3
			MAV_TYPE_VTOL_RESERVED4     = 24, //VTOL reserved 4
			MAV_TYPE_VTOL_RESERVED5     = 25, //VTOL reserved 5
			MAV_TYPE_GIMBAL             = 26, //Onboard gimbal
			MAV_TYPE_ADSB               = 27, //Onboard ADSB peripheral
			MAV_TYPE_PARAFOIL           = 28;  //Steerable, nonrigid airfoil
}

/**
 * These values define the type of firmware release.  These values indicate the first version or release
 * of this type.  For example the first alpha release would be 64, the second would be 65
 */
enum FIRMWARE_VERSION_TYPE {
	;
	
	final int
			FIRMWARE_VERSION_TYPE_DEV      = 0, //development release
			FIRMWARE_VERSION_TYPE_ALPHA    = 64, //alpha release
			FIRMWARE_VERSION_TYPE_BETA     = 128, //beta release
			FIRMWARE_VERSION_TYPE_RC       = 192, //release candidate
			FIRMWARE_VERSION_TYPE_OFFICIAL = 255;  //official stable release
}

/**
 * These flags encode the MAV mode.
 */
enum MAV_MODE_FLAG {
	;
	
	final int
			/**
			 0b10000000 MAV safety set to armed. Motors are enabled / running / can start. Ready to fly. Additional
			 note: this flag is to be ignore when sent in the command MAV_CMD_DO_SET_MODE and MAV_CMD_COMPONENT_ARM_DISARM
			 shall be used instead. The flag can still be used to report the armed state*/
			MAV_MODE_FLAG_SAFETY_ARMED         = 128,
			MAV_MODE_FLAG_MANUAL_INPUT_ENABLED = 64, //0b01000000 remote control input is enabled.
	/**
	 * 0b00100000 hardware in the loop simulation. All motors / actuators are blocked, but internal software
	 * is full operational
	 */
	MAV_MODE_FLAG_HIL_ENABLED = 32,
	/**
	 * 0b00010000 system stabilizes electronically its attitude (and optionally position). It needs however further
	 * control inputs to move around
	 */
	MAV_MODE_FLAG_STABILIZE_ENABLED      = 16,
			MAV_MODE_FLAG_GUIDED_ENABLED = 8, //0b00001000 guided mode enabled, system flies waypoints / mission items.
	/**
	 * 0b00000100 autonomous mode enabled, system finds its own goal positions. Guided flag can be set or not,
	 * depends on the actual implementation
	 */
	MAV_MODE_FLAG_AUTO_ENABLED = 4,
	/**
	 * 0b00000010 system has a test mode enabled. This flag is intended for temporary system tests and should
	 * not be used for stable implementations
	 */
	MAV_MODE_FLAG_TEST_ENABLED                = 2,
			MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1;  //0b00000001 Reserved for future use.
}

/**
 * These values encode the bit positions of the decode position. These values can be used to read the value
 * of a flag bit by combining the base_mode variable with AND with the flag position value. The result will
 * be either 0 or 1, depending on if the flag is set or not
 */
enum MAV_MODE_FLAG_DECODE_POSITION {
	;
	
	final int
			MAV_MODE_FLAG_DECODE_POSITION_SAFETY      = 128, //First bit:  10000000
			MAV_MODE_FLAG_DECODE_POSITION_MANUAL      = 64, //Second bit: 01000000
			MAV_MODE_FLAG_DECODE_POSITION_HIL         = 32, //Third bit:  00100000
			MAV_MODE_FLAG_DECODE_POSITION_STABILIZE   = 16, //Fourth bit: 00010000
			MAV_MODE_FLAG_DECODE_POSITION_GUIDED      = 8, //Fifth bit:  00001000
			MAV_MODE_FLAG_DECODE_POSITION_AUTO        = 4, //Sixt bit:   00000100
			MAV_MODE_FLAG_DECODE_POSITION_TEST        = 2, //Seventh bit: 00000010
			MAV_MODE_FLAG_DECODE_POSITION_CUSTOM_MODE = 1;  //Eighth bit: 00000001
}

/**
 * Override command, pauses current mission execution and moves immediately to a position
 */
enum MAV_GOTO {
	;
	
	final int
			MAV_GOTO_DO_HOLD                    = 0, //Hold at the current position.
			MAV_GOTO_DO_CONTINUE                = 1, //Continue with the next item in mission execution.
			MAV_GOTO_HOLD_AT_CURRENT_POSITION   = 2, //Hold at the current position of the system
			MAV_GOTO_HOLD_AT_SPECIFIED_POSITION = 3;  //Hold at the position specified in the parameters of the DO_HOLD action
}

/**
 * These defines are predefined OR-combined mode flags. There is no need to use values from this enum, but it
 * simplifies the use of the mode flags. Note that manual input is enabled in all modes as a safety override.
 */
enum MAV_MODE {
	;
	
	final int
			MAV_MODE_PREFLIGHT          = 0, //System is not ready to fly, booting, calibrating, etc. No flag is set.
			MAV_MODE_STABILIZE_DISARMED = 80, //System is allowed to be active, under assisted RC control.
			MAV_MODE_STABILIZE_ARMED    = 208, //System is allowed to be active, under assisted RC control.
			MAV_MODE_MANUAL_DISARMED    = 64, //System is allowed to be active, under manual (RC) control, no stabilization
			MAV_MODE_MANUAL_ARMED       = 192, //System is allowed to be active, under manual (RC) control, no stabilization
			MAV_MODE_GUIDED_DISARMED    = 88, //System is allowed to be active, under autonomous control, manual setpoint
			MAV_MODE_GUIDED_ARMED       = 216, //System is allowed to be active, under autonomous control, manual setpoint
	/**
	 * System is allowed to be active, under autonomous control and navigation (the trajectory is decided onboard
	 * and not pre-programmed by waypoints
	 */
	MAV_MODE_AUTO_DISARMED = 92,
	/**
	 * System is allowed to be active, under autonomous control and navigation (the trajectory is decided onboard
	 * and not pre-programmed by waypoints
	 */
	MAV_MODE_AUTO_ARMED            = 220,
			MAV_MODE_TEST_DISARMED = 66, //UNDEFINED mode. This solely depends on the autopilot - use with caution, intended for developers only
			MAV_MODE_TEST_ARMED    = 194;  //UNDEFINED mode. This solely depends on the autopilot - use with caution, intended for developers only
}

/**
 * Uninitialized system, state is unknown.
 */
enum MAV_STATE {
	MAV_STATE_BOOT, //System is booting up.
	MAV_STATE_CALIBRATING, //System is calibrating and not flight-ready.
	MAV_STATE_STANDBY, //System is grounded and on standby. It can be launched any time.
	MAV_STATE_ACTIVE, //System is active and might be already airborne. Motors are engaged.
	MAV_STATE_CRITICAL, //System is in a non-normal flight mode. It can however still navigate.
	/**
	 * System is in a non-normal flight mode. It lost control over parts or over the whole airframe. It is in
	 * mayday and going down
	 */
	MAV_STATE_EMERGENCY,
	MAV_STATE_POWEROFF, //System just initialized its power-down sequence, will shut down now.
	MAV_STATE_FLIGHT_TERMINATION;  //System is terminating itself.
	final int
			MAV_STATE_UNINIT = 0;  //Uninitialized system, state is unknown.
}

/**
 */
enum MAV_COMPONENT {
	;
	
	final int
			MAV_COMP_ID_ALL            = 0,
			MAV_COMP_ID_AUTOPILOT1     = 1,
			MAV_COMP_ID_CAMERA         = 100,
			MAV_COMP_ID_CAMERA2        = 101,
			MAV_COMP_ID_CAMERA3        = 102,
			MAV_COMP_ID_CAMERA4        = 103,
			MAV_COMP_ID_CAMERA5        = 104,
			MAV_COMP_ID_CAMERA6        = 105,
			MAV_COMP_ID_SERVO1         = 140,
			MAV_COMP_ID_SERVO2         = 141,
			MAV_COMP_ID_SERVO3         = 142,
			MAV_COMP_ID_SERVO4         = 143,
			MAV_COMP_ID_SERVO5         = 144,
			MAV_COMP_ID_SERVO6         = 145,
			MAV_COMP_ID_SERVO7         = 146,
			MAV_COMP_ID_SERVO8         = 147,
			MAV_COMP_ID_SERVO9         = 148,
			MAV_COMP_ID_SERVO10        = 149,
			MAV_COMP_ID_SERVO11        = 150,
			MAV_COMP_ID_SERVO12        = 151,
			MAV_COMP_ID_SERVO13        = 152,
			MAV_COMP_ID_SERVO14        = 153,
			MAV_COMP_ID_GIMBAL         = 154,
			MAV_COMP_ID_LOG            = 155,
			MAV_COMP_ID_ADSB           = 156,
			MAV_COMP_ID_OSD            = 157, //On Screen Display (OSD) devices for video links
			MAV_COMP_ID_PERIPHERAL     = 158, //Generic autopilot peripheral component ID. Meant for devices that do not implement the parameter sub-protoco
			MAV_COMP_ID_QX1_GIMBAL     = 159,
			MAV_COMP_ID_MAPPER         = 180,
			MAV_COMP_ID_MISSIONPLANNER = 190,
			MAV_COMP_ID_PATHPLANNER    = 195,
			MAV_COMP_ID_IMU            = 200,
			MAV_COMP_ID_IMU_2          = 201,
			MAV_COMP_ID_IMU_3          = 202,
			MAV_COMP_ID_GPS            = 220,
			MAV_COMP_ID_GPS2           = 221,
			MAV_COMP_ID_UDP_BRIDGE     = 240,
			MAV_COMP_ID_UART_BRIDGE    = 241,
			MAV_COMP_ID_SYSTEM_CONTROL = 250;
}

/**
 * These encode the sensors whose status is sent as part of the SYS_STATUS message.
 */
enum MAV_SYS_STATUS_SENSOR {
	;
	
	final int
			MAV_SYS_STATUS_SENSOR_3D_GYRO                = 1, //0x01 3D gyro
			MAV_SYS_STATUS_SENSOR_3D_ACCEL               = 2, //0x02 3D accelerometer
			MAV_SYS_STATUS_SENSOR_3D_MAG                 = 4, //0x04 3D magnetometer
			MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE      = 8, //0x08 absolute pressure
			MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE  = 16, //0x10 differential pressure
			MAV_SYS_STATUS_SENSOR_GPS                    = 32, //0x20 GPS
			MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW           = 64, //0x40 optical flow
			MAV_SYS_STATUS_SENSOR_VISION_POSITION        = 128, //0x80 computer vision position
			MAV_SYS_STATUS_SENSOR_LASER_POSITION         = 256, //0x100 laser based position
			MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH  = 512, //0x200 external ground truth (Vicon or Leica)
			MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL   = 1024, //0x400 3D angular rate control
			MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION = 2048, //0x800 attitude stabilization
			MAV_SYS_STATUS_SENSOR_YAW_POSITION           = 4096, //0x1000 yaw position
			MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL     = 8192, //0x2000 z/altitude control
			MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL    = 16384, //0x4000 x/y position control
			MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS          = 32768, //0x8000 motor outputs / control
			MAV_SYS_STATUS_SENSOR_RC_RECEIVER            = 65536, //0x10000 rc receiver
			MAV_SYS_STATUS_SENSOR_3D_GYRO2               = 131072, //0x20000 2nd 3D gyro
			MAV_SYS_STATUS_SENSOR_3D_ACCEL2              = 262144, //0x40000 2nd 3D accelerometer
			MAV_SYS_STATUS_SENSOR_3D_MAG2                = 524288, //0x80000 2nd 3D magnetometer
			MAV_SYS_STATUS_GEOFENCE                      = 1048576, //0x100000 geofence
			MAV_SYS_STATUS_AHRS                          = 2097152, //0x200000 AHRS subsystem health
			MAV_SYS_STATUS_TERRAIN                       = 4194304, //0x400000 Terrain subsystem health
			MAV_SYS_STATUS_REVERSE_MOTOR                 = 8388608, //0x800000 Motors are reversed
			MAV_SYS_STATUS_LOGGING                       = 16777216, //0x1000000 Logging
			MAV_SYS_STATUS_SENSOR_BATTERY                = 33554432;  //0x2000000 Battery
}

/**
 * Global coordinate frame, WGS84 coordinate system. First value / x: latitude, second value / y: longitude,
 * third value / z: positive altitude over mean sea level (MSL
 */
enum MAV_FRAME {
	;
	
	final int
			/**
			 Global coordinate frame, WGS84 coordinate system. First value / x: latitude, second value / y: longitude,
			 third value / z: positive altitude over mean sea level (MSL*/
			MAV_FRAME_GLOBAL    = 0,
			MAV_FRAME_LOCAL_NED = 1, //Local coordinate frame, Z-up (x: north, y: east, z: down).
			MAV_FRAME_MISSION   = 2, //NOT a coordinate frame, indicates a mission command.
	/**
	 * Global coordinate frame, WGS84 coordinate system, relative altitude over ground with respect to the home
	 * position. First value / x: latitude, second value / y: longitude, third value / z: positive altitude
	 * with 0 being at the altitude of the home location
	 */
	MAV_FRAME_GLOBAL_RELATIVE_ALT = 3,
			MAV_FRAME_LOCAL_ENU   = 4, //Local coordinate frame, Z-down (x: east, y: north, z: up)
	/**
	 * Global coordinate frame, WGS84 coordinate system. First value / x: latitude in degrees*1.0e-7, second
	 * value / y: longitude in degrees*1.0e-7, third value / z: positive altitude over mean sea level (MSL
	 */
	MAV_FRAME_GLOBAL_INT = 5,
	/**
	 * Global coordinate frame, WGS84 coordinate system, relative altitude over ground with respect to the home
	 * position. First value / x: latitude in degrees*10e-7, second value / y: longitude in degrees*10e-7, third
	 * value / z: positive altitude with 0 being at the altitude of the home location
	 */
	MAV_FRAME_GLOBAL_RELATIVE_ALT_INT = 6,
	/**
	 * Offset to the current local frame. Anything expressed in this frame should be added to the current local
	 * frame position
	 */
	MAV_FRAME_LOCAL_OFFSET_NED = 7,
	/**
	 * Setpoint in body NED frame. This makes sense if all position control is externalized - e.g. useful to
	 * command 2 m/s^2 acceleration to the right
	 */
	MAV_FRAME_BODY_NED = 8,
	/**
	 * Offset in body NED frame. This makes sense if adding setpoints to the current flight path, to avoid an
	 * obstacle - e.g. useful to command 2 m/s^2 acceleration to the east
	 */
	MAV_FRAME_BODY_OFFSET_NED = 9,
	/**
	 * Global coordinate frame with above terrain level altitude. WGS84 coordinate system, relative altitude
	 * over terrain with respect to the waypoint coordinate. First value / x: latitude in degrees, second value
	 * / y: longitude in degrees, third value / z: positive altitude in meters with 0 being at ground level
	 * in terrain model
	 */
	MAV_FRAME_GLOBAL_TERRAIN_ALT = 10,
	/**
	 * Global coordinate frame with above terrain level altitude. WGS84 coordinate system, relative altitude
	 * over terrain with respect to the waypoint coordinate. First value / x: latitude in degrees*10e-7, second
	 * value / y: longitude in degrees*10e-7, third value / z: positive altitude in meters with 0 being at ground
	 * level in terrain model
	 */
	MAV_FRAME_GLOBAL_TERRAIN_ALT_INT = 11;
}

/**
 */
enum MAVLINK_DATA_STREAM_TYPE {
	MAVLINK_DATA_STREAM_IMG_JPEG,
	MAVLINK_DATA_STREAM_IMG_BMP,
	MAVLINK_DATA_STREAM_IMG_RAW8U,
	MAVLINK_DATA_STREAM_IMG_RAW32U,
	MAVLINK_DATA_STREAM_IMG_PGM,
	MAVLINK_DATA_STREAM_IMG_PNG;
}

/**
 * Disable fenced mode
 */
enum FENCE_ACTION {
	;
	
	final int
			FENCE_ACTION_NONE            = 0, //Disable fenced mode
			FENCE_ACTION_GUIDED          = 1, //Switched to guided mode to return point (fence point 0)
			FENCE_ACTION_REPORT          = 2, //Report fence breach, but don't take action
			FENCE_ACTION_GUIDED_THR_PASS = 3, //Switched to guided mode to return point (fence point 0) with manual throttle control
			FENCE_ACTION_RTL             = 4;  //Switch to RTL (return to launch) mode and head for the return point.
}

/**
 * No last fence breach
 */
enum FENCE_BREACH {
	;
	
	final int
			FENCE_BREACH_NONE     = 0, //No last fence breach
			FENCE_BREACH_MINALT   = 1, //Breached minimum altitude
			FENCE_BREACH_MAXALT   = 2, //Breached maximum altitude
			FENCE_BREACH_BOUNDARY = 3;  //Breached fence boundary
}

/**
 * Enumeration of possible mount operation modes
 */
enum MAV_MOUNT_MODE {
	;
	
	final int
			MAV_MOUNT_MODE_RETRACT           = 0, //Load and keep safe position (Roll,Pitch,Yaw) from permant memory and stop stabilization
			MAV_MOUNT_MODE_NEUTRAL           = 1, //Load and keep neutral position (Roll,Pitch,Yaw) from permanent memory.
			MAV_MOUNT_MODE_MAVLINK_TARGETING = 2, //Load neutral position and start MAVLink Roll,Pitch,Yaw control with stabilization
			MAV_MOUNT_MODE_RC_TARGETING      = 3, //Load neutral position and start RC Roll,Pitch,Yaw control with stabilization
			MAV_MOUNT_MODE_GPS_POINT         = 4;  //Load neutral position and start to point to Lat,Lon,Alt
}

/**
 * Generalized UAVCAN node health
 */
enum UAVCAN_NODE_HEALTH {
	;
	
	final int
			UAVCAN_NODE_HEALTH_OK       = 0, //The node is functioning properly.
			UAVCAN_NODE_HEALTH_WARNING  = 1, //A critical parameter went out of range or the node has encountered a minor failure.
			UAVCAN_NODE_HEALTH_ERROR    = 2, //The node has encountered a major failure.
			UAVCAN_NODE_HEALTH_CRITICAL = 3;  //The node has suffered a fatal malfunction.
}

/**
 * Generalized UAVCAN node mode
 */
enum UAVCAN_NODE_MODE {
	;
	
	final int
			UAVCAN_NODE_MODE_OPERATIONAL     = 0, //The node is performing its primary functions.
			UAVCAN_NODE_MODE_INITIALIZATION  = 1, //The node is initializing; this mode is entered immediately after startup.
			UAVCAN_NODE_MODE_MAINTENANCE     = 2, //The node is under maintenance.
			UAVCAN_NODE_MODE_SOFTWARE_UPDATE = 3, //The node is in the process of updating its software.
			UAVCAN_NODE_MODE_OFFLINE         = 7;  //The node is no longer available online.
}

/**
 * the recommended messages.
 */
enum MAV_DATA_STREAM {
	;
	
	final int
			MAV_DATA_STREAM_ALL             = 0, //Enable all data streams
			MAV_DATA_STREAM_RAW_SENSORS     = 1, //Enable IMU_RAW, GPS_RAW, GPS_STATUS packets.
			MAV_DATA_STREAM_EXTENDED_STATUS = 2, //Enable GPS_STATUS, CONTROL_STATUS, AUX_STATUS
			MAV_DATA_STREAM_RC_CHANNELS     = 3, //Enable RC_CHANNELS_SCALED, RC_CHANNELS_RAW, SERVO_OUTPUT_RAW
			MAV_DATA_STREAM_RAW_CONTROLLER  = 4, //Enable ATTITUDE_CONTROLLER_OUTPUT, POSITION_CONTROLLER_OUTPUT, NAV_CONTROLLER_OUTPUT.
			MAV_DATA_STREAM_POSITION        = 6, //Enable LOCAL_POSITION, GLOBAL_POSITION/GLOBAL_POSITION_INT messages.
			MAV_DATA_STREAM_EXTRA1          = 10, //Dependent on the autopilot
			MAV_DATA_STREAM_EXTRA2          = 11, //Dependent on the autopilot
			MAV_DATA_STREAM_EXTRA3          = 12;  //Dependent on the autopilot
}

/**
 * MAV_CMD_NAV_ROI).
 */
enum MAV_ROI {
	;
	
	final int
			MAV_ROI_NONE     = 0, //No region of interest.
			MAV_ROI_WPNEXT   = 1, //Point toward next waypoint, with optional pitch/roll/yaw offset.
			MAV_ROI_WPINDEX  = 2, //Point toward given waypoint.
			MAV_ROI_LOCATION = 3, //Point toward fixed location.
			MAV_ROI_TARGET   = 4;  //Point toward of given id.
}

/**
 * ACK / NACK / ERROR values as a result of MAV_CMDs and for mission item transmission.
 */
enum MAV_CMD_ACK {
	MAV_CMD_ACK_OK, //Command / mission item is ok.
	MAV_CMD_ACK_ERR_FAIL, //Generic error message if none of the other reasons fails or if no detailed error reporting is implemented
	MAV_CMD_ACK_ERR_ACCESS_DENIED, //The system is refusing to accept this command from this source / communication partner.
	MAV_CMD_ACK_ERR_NOT_SUPPORTED, //Command or mission item is not supported, other commands would be accepted.
	MAV_CMD_ACK_ERR_COORDINATE_FRAME_NOT_SUPPORTED, //The coordinate frame of this command / mission item is not supported.
	/**
	 * The coordinate frame of this command is ok, but he coordinate values exceed the safety limits of this
	 * system. This is a generic error, please use the more specific error messages below if possible
	 */
	MAV_CMD_ACK_ERR_COORDINATES_OUT_OF_RANGE,
	MAV_CMD_ACK_ERR_X_LAT_OUT_OF_RANGE, //The X or latitude value is out of range.
	MAV_CMD_ACK_ERR_Y_LON_OUT_OF_RANGE, //The Y or longitude value is out of range.
	MAV_CMD_ACK_ERR_Z_ALT_OUT_OF_RANGE;  //The Z or altitude value is out of range.
}

/**
 * Specifies the datatype of a MAVLink parameter.
 */
enum MAV_PARAM_TYPE {
	;
	
	final int
			MAV_PARAM_TYPE_UINT8  = 1, //8-bit unsigned integer
			MAV_PARAM_TYPE_INT8   = 2, //8-bit signed integer
			MAV_PARAM_TYPE_UINT16 = 3, //16-bit unsigned integer
			MAV_PARAM_TYPE_INT16  = 4, //16-bit signed integer
			MAV_PARAM_TYPE_UINT32 = 5, //32-bit unsigned integer
			MAV_PARAM_TYPE_INT32  = 6, //32-bit signed integer
			MAV_PARAM_TYPE_UINT64 = 7, //64-bit unsigned integer
			MAV_PARAM_TYPE_INT64  = 8, //64-bit signed integer
			MAV_PARAM_TYPE_REAL32 = 9, //32-bit floating-point
			MAV_PARAM_TYPE_REAL64 = 10;  //64-bit floating-point
}

/**
 * Specifies the datatype of a MAVLink extended parameter.
 */
enum MAV_PARAM_EXT_TYPE {
	;
	
	final int
			MAV_PARAM_EXT_TYPE_UINT8  = 1, //8-bit unsigned integer
			MAV_PARAM_EXT_TYPE_INT8   = 2, //8-bit signed integer
			MAV_PARAM_EXT_TYPE_UINT16 = 3, //16-bit unsigned integer
			MAV_PARAM_EXT_TYPE_INT16  = 4, //16-bit signed integer
			MAV_PARAM_EXT_TYPE_UINT32 = 5, //32-bit unsigned integer
			MAV_PARAM_EXT_TYPE_INT32  = 6, //32-bit signed integer
			MAV_PARAM_EXT_TYPE_UINT64 = 7, //64-bit unsigned integer
			MAV_PARAM_EXT_TYPE_INT64  = 8, //64-bit signed integer
			MAV_PARAM_EXT_TYPE_REAL32 = 9, //32-bit floating-point
			MAV_PARAM_EXT_TYPE_REAL64 = 10, //64-bit floating-point
			MAV_PARAM_EXT_TYPE_CUSTOM = 11;  //Custom Type
}

/**
 * result from a mavlink command
 */
enum MAV_RESULT {
	;
	
	final int
			MAV_RESULT_ACCEPTED             = 0, //Command ACCEPTED and EXECUTED
			MAV_RESULT_TEMPORARILY_REJECTED = 1, //Command TEMPORARY REJECTED/DENIED
			MAV_RESULT_DENIED               = 2, //Command PERMANENTLY DENIED
			MAV_RESULT_UNSUPPORTED          = 3, //Command UNKNOWN/UNSUPPORTED
			MAV_RESULT_FAILED               = 4, //Command executed, but failed
			MAV_RESULT_IN_PROGRESS          = 5;  //WIP: Command being executed
}

/**
 * result in a mavlink mission ack
 */
enum MAV_MISSION_RESULT {
	;
	
	final int
			MAV_MISSION_ACCEPTED          = 0, //mission accepted OK
			MAV_MISSION_ERROR             = 1, //generic error / not accepting mission commands at all right now
			MAV_MISSION_UNSUPPORTED_FRAME = 2, //coordinate frame is not supported
			MAV_MISSION_UNSUPPORTED       = 3, //command is not supported
			MAV_MISSION_NO_SPACE          = 4, //mission item exceeds storage space
			MAV_MISSION_INVALID           = 5, //one of the parameters has an invalid value
			MAV_MISSION_INVALID_PARAM1    = 6, //param1 has an invalid value
			MAV_MISSION_INVALID_PARAM2    = 7, //param2 has an invalid value
			MAV_MISSION_INVALID_PARAM3    = 8, //param3 has an invalid value
			MAV_MISSION_INVALID_PARAM4    = 9, //param4 has an invalid value
			MAV_MISSION_INVALID_PARAM5_X  = 10, //x/param5 has an invalid value
			MAV_MISSION_INVALID_PARAM6_Y  = 11, //y/param6 has an invalid value
			MAV_MISSION_INVALID_PARAM7    = 12, //param7 has an invalid value
			MAV_MISSION_INVALID_SEQUENCE  = 13, //received waypoint out of sequence
			MAV_MISSION_DENIED            = 14;  //not accepting any mission commands from this communication partner
}

/**
 * Indicates the severity level, generally used for status messages to indicate their relative urgency. Based
 * on RFC-5424 using expanded definitions at: http://www.kiwisyslog.com/kb/info:-syslog-message-levels/
 */
enum MAV_SEVERITY {
	;
	
	final int
			MAV_SEVERITY_EMERGENCY = 0, //System is unusable. This is a "panic" condition.
			MAV_SEVERITY_ALERT     = 1, //Action should be taken immediately. Indicates error in non-critical systems.
			MAV_SEVERITY_CRITICAL  = 2, //Action must be taken immediately. Indicates failure in a primary system.
			MAV_SEVERITY_ERROR     = 3, //Indicates an error in secondary/redundant systems.
	/**
	 * Indicates about a possible future error if this is not resolved within a given timeframe. Example would
	 * be a low battery warning
	 */
	MAV_SEVERITY_WARNING = 4,
	/**
	 * An unusual event has occured, though not an error condition. This should be investigated for the root
	 * cause
	 */
	MAV_SEVERITY_NOTICE        = 5,
			MAV_SEVERITY_INFO  = 6, //Normal operational messages. Useful for logging. No action is required for these messages.
			MAV_SEVERITY_DEBUG = 7;  //Useful non-operational messages that can assist in debugging. These should not occur during normal operation
}

/**
 * Power supply status flags (bitmask)
 */
enum MAV_POWER_STATUS {
	;
	
	final int
			MAV_POWER_STATUS_BRICK_VALID                = 1, //main brick power supply valid
			MAV_POWER_STATUS_SERVO_VALID                = 2, //main servo power supply valid for FMU
			MAV_POWER_STATUS_USB_CONNECTED              = 4, //USB power is connected
			MAV_POWER_STATUS_PERIPH_OVERCURRENT         = 8, //peripheral supply is in over-current state
			MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT = 16, //hi-power peripheral supply is in over-current state
			MAV_POWER_STATUS_CHANGED                    = 32;  //Power status has changed since boot
}

/**
 * SERIAL_CONTROL device types
 */
enum SERIAL_CONTROL_DEV {
	;
	
	final int
			SERIAL_CONTROL_DEV_TELEM1 = 0, //First telemetry port
			SERIAL_CONTROL_DEV_TELEM2 = 1, //Second telemetry port
			SERIAL_CONTROL_DEV_GPS1   = 2, //First GPS port
			SERIAL_CONTROL_DEV_GPS2   = 3, //Second GPS port
			SERIAL_CONTROL_DEV_SHELL  = 10;  //system shell
}

/**
 * SERIAL_CONTROL flags (bitmask)
 */
enum SERIAL_CONTROL_FLAG {
	;
	
	final int
			SERIAL_CONTROL_FLAG_REPLY   = 1, //Set if this is a reply
			SERIAL_CONTROL_FLAG_RESPOND = 2, //Set if the sender wants the receiver to send a response as another SERIAL_CONTROL message
	/**
	 * Set if access to the serial port should be removed from whatever driver is currently using it, giving
	 * exclusive access to the SERIAL_CONTROL protocol. The port can be handed back by sending a request without
	 * this flag se
	 */
	SERIAL_CONTROL_FLAG_EXCLUSIVE        = 4,
			SERIAL_CONTROL_FLAG_BLOCKING = 8, //Block on writes to the serial port
			SERIAL_CONTROL_FLAG_MULTI    = 16;  //Send multiple replies until port is drained
}

/**
 * Enumeration of distance sensor types
 */
enum MAV_DISTANCE_SENSOR {
	;
	
	final int
			MAV_DISTANCE_SENSOR_LASER      = 0, //Laser rangefinder, e.g. LightWare SF02/F or PulsedLight units
			MAV_DISTANCE_SENSOR_ULTRASOUND = 1, //Ultrasound rangefinder, e.g. MaxBotix units
			MAV_DISTANCE_SENSOR_INFRARED   = 2, //Infrared rangefinder, e.g. Sharp units
			MAV_DISTANCE_SENSOR_RADAR      = 3, //Radar type, e.g. uLanding units
			MAV_DISTANCE_SENSOR_UNKNOWN    = 4;  //Broken or unknown type, e.g. analog units
}

/**
 * Enumeration of sensor orientation, according to its rotations
 */
enum MAV_SENSOR_ORIENTATION {
	;
	
	final int
			MAV_SENSOR_ROTATION_NONE                       = 0, //Roll: 0, Pitch: 0, Yaw: 0
			MAV_SENSOR_ROTATION_YAW_45                     = 1, //Roll: 0, Pitch: 0, Yaw: 45
			MAV_SENSOR_ROTATION_YAW_90                     = 2, //Roll: 0, Pitch: 0, Yaw: 90
			MAV_SENSOR_ROTATION_YAW_135                    = 3, //Roll: 0, Pitch: 0, Yaw: 135
			MAV_SENSOR_ROTATION_YAW_180                    = 4, //Roll: 0, Pitch: 0, Yaw: 180
			MAV_SENSOR_ROTATION_YAW_225                    = 5, //Roll: 0, Pitch: 0, Yaw: 225
			MAV_SENSOR_ROTATION_YAW_270                    = 6, //Roll: 0, Pitch: 0, Yaw: 270
			MAV_SENSOR_ROTATION_YAW_315                    = 7, //Roll: 0, Pitch: 0, Yaw: 315
			MAV_SENSOR_ROTATION_ROLL_180                   = 8, //Roll: 180, Pitch: 0, Yaw: 0
			MAV_SENSOR_ROTATION_ROLL_180_YAW_45            = 9, //Roll: 180, Pitch: 0, Yaw: 45
			MAV_SENSOR_ROTATION_ROLL_180_YAW_90            = 10, //Roll: 180, Pitch: 0, Yaw: 90
			MAV_SENSOR_ROTATION_ROLL_180_YAW_135           = 11, //Roll: 180, Pitch: 0, Yaw: 135
			MAV_SENSOR_ROTATION_PITCH_180                  = 12, //Roll: 0, Pitch: 180, Yaw: 0
			MAV_SENSOR_ROTATION_ROLL_180_YAW_225           = 13, //Roll: 180, Pitch: 0, Yaw: 225
			MAV_SENSOR_ROTATION_ROLL_180_YAW_270           = 14, //Roll: 180, Pitch: 0, Yaw: 270
			MAV_SENSOR_ROTATION_ROLL_180_YAW_315           = 15, //Roll: 180, Pitch: 0, Yaw: 315
			MAV_SENSOR_ROTATION_ROLL_90                    = 16, //Roll: 90, Pitch: 0, Yaw: 0
			MAV_SENSOR_ROTATION_ROLL_90_YAW_45             = 17, //Roll: 90, Pitch: 0, Yaw: 45
			MAV_SENSOR_ROTATION_ROLL_90_YAW_90             = 18, //Roll: 90, Pitch: 0, Yaw: 90
			MAV_SENSOR_ROTATION_ROLL_90_YAW_135            = 19, //Roll: 90, Pitch: 0, Yaw: 135
			MAV_SENSOR_ROTATION_ROLL_270                   = 20, //Roll: 270, Pitch: 0, Yaw: 0
			MAV_SENSOR_ROTATION_ROLL_270_YAW_45            = 21, //Roll: 270, Pitch: 0, Yaw: 45
			MAV_SENSOR_ROTATION_ROLL_270_YAW_90            = 22, //Roll: 270, Pitch: 0, Yaw: 90
			MAV_SENSOR_ROTATION_ROLL_270_YAW_135           = 23, //Roll: 270, Pitch: 0, Yaw: 135
			MAV_SENSOR_ROTATION_PITCH_90                   = 24, //Roll: 0, Pitch: 90, Yaw: 0
			MAV_SENSOR_ROTATION_PITCH_270                  = 25, //Roll: 0, Pitch: 270, Yaw: 0
			MAV_SENSOR_ROTATION_PITCH_180_YAW_90           = 26, //Roll: 0, Pitch: 180, Yaw: 90
			MAV_SENSOR_ROTATION_PITCH_180_YAW_270          = 27, //Roll: 0, Pitch: 180, Yaw: 270
			MAV_SENSOR_ROTATION_ROLL_90_PITCH_90           = 28, //Roll: 90, Pitch: 90, Yaw: 0
			MAV_SENSOR_ROTATION_ROLL_180_PITCH_90          = 29, //Roll: 180, Pitch: 90, Yaw: 0
			MAV_SENSOR_ROTATION_ROLL_270_PITCH_90          = 30, //Roll: 270, Pitch: 90, Yaw: 0
			MAV_SENSOR_ROTATION_ROLL_90_PITCH_180          = 31, //Roll: 90, Pitch: 180, Yaw: 0
			MAV_SENSOR_ROTATION_ROLL_270_PITCH_180         = 32, //Roll: 270, Pitch: 180, Yaw: 0
			MAV_SENSOR_ROTATION_ROLL_90_PITCH_270          = 33, //Roll: 90, Pitch: 270, Yaw: 0
			MAV_SENSOR_ROTATION_ROLL_180_PITCH_270         = 34, //Roll: 180, Pitch: 270, Yaw: 0
			MAV_SENSOR_ROTATION_ROLL_270_PITCH_270         = 35, //Roll: 270, Pitch: 270, Yaw: 0
			MAV_SENSOR_ROTATION_ROLL_90_PITCH_180_YAW_90   = 36, //Roll: 90, Pitch: 180, Yaw: 90
			MAV_SENSOR_ROTATION_ROLL_90_YAW_270            = 37, //Roll: 90, Pitch: 0, Yaw: 270
			MAV_SENSOR_ROTATION_ROLL_315_PITCH_315_YAW_315 = 38;  //Roll: 315, Pitch: 315, Yaw: 315
}

/**
 * Bitmask of (optional) autopilot capabilities (64 bit). If a bit is set, the autopilot supports this capability
 */
enum MAV_PROTOCOL_CAPABILITY {
	;
	
	final int
			MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT                  = 1, //Autopilot supports MISSION float message type.
			MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT                    = 2, //Autopilot supports the new param float message type.
			MAV_PROTOCOL_CAPABILITY_MISSION_INT                    = 4, //Autopilot supports MISSION_INT scaled integer message type.
			MAV_PROTOCOL_CAPABILITY_COMMAND_INT                    = 8, //Autopilot supports COMMAND_INT scaled integer message type.
			MAV_PROTOCOL_CAPABILITY_PARAM_UNION                    = 16, //Autopilot supports the new param union message type.
			MAV_PROTOCOL_CAPABILITY_FTP                            = 32, //Autopilot supports the new FILE_TRANSFER_PROTOCOL message type.
			MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET            = 64, //Autopilot supports commanding attitude offboard.
			MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED  = 128, //Autopilot supports commanding position and velocity targets in local NED frame.
			MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT = 256, //Autopilot supports commanding position and velocity targets in global scaled integers.
			MAV_PROTOCOL_CAPABILITY_TERRAIN                        = 512, //Autopilot supports terrain protocol / data handling.
			MAV_PROTOCOL_CAPABILITY_SET_ACTUATOR_TARGET            = 1024, //Autopilot supports direct actuator control.
			MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION             = 2048, //Autopilot supports the flight termination command.
			MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION            = 4096, //Autopilot supports onboard compass calibration.
			MAV_PROTOCOL_CAPABILITY_MAVLINK2                       = 8192, //Autopilot supports mavlink version 2.
			MAV_PROTOCOL_CAPABILITY_MISSION_FENCE                  = 16384, //Autopilot supports mission fence protocol.
			MAV_PROTOCOL_CAPABILITY_MISSION_RALLY                  = 32768, //Autopilot supports mission rally point protocol.
			MAV_PROTOCOL_CAPABILITY_FLIGHT_INFORMATION             = 65536;  //Autopilot supports the flight information protocol.
}

/**
 * Type of mission items being requested/sent in mission protocol.
 */
enum MAV_MISSION_TYPE {
	;
	
	final int
			MAV_MISSION_TYPE_MISSION = 0, //Items are mission commands for main mission.
			MAV_MISSION_TYPE_FENCE   = 1, //Specifies GeoFence area(s). Items are MAV_CMD_FENCE_ GeoFence items.
	/**
	 * Specifies the rally points for the vehicle. Rally points are alternative RTL points. Items are MAV_CMD_RALLY_POINT
	 * rally point items
	 */
	MAV_MISSION_TYPE_RALLY       = 2,
			MAV_MISSION_TYPE_ALL = 255;  //Only used in MISSION_CLEAR_ALL to clear all mission types.
}

/**
 * Enumeration of estimator types
 */
enum MAV_ESTIMATOR_TYPE {
	;
	
	final int
			MAV_ESTIMATOR_TYPE_NAIVE   = 1, //This is a naive estimator without any real covariance feedback.
			MAV_ESTIMATOR_TYPE_VISION  = 2, //Computer vision based estimate. Might be up to scale.
			MAV_ESTIMATOR_TYPE_VIO     = 3, //Visual-inertial estimate.
			MAV_ESTIMATOR_TYPE_GPS     = 4, //Plain GPS estimate.
			MAV_ESTIMATOR_TYPE_GPS_INS = 5;  //Estimator integrating GPS and inertial sensing.
}

/**
 * Enumeration of battery types
 */
enum MAV_BATTERY_TYPE {
	;
	
	final int
			MAV_BATTERY_TYPE_UNKNOWN = 0, //Not specified.
			MAV_BATTERY_TYPE_LIPO    = 1, //Lithium polymer battery
			MAV_BATTERY_TYPE_LIFE    = 2, //Lithium-iron-phosphate battery
			MAV_BATTERY_TYPE_LION    = 3, //Lithium-ION battery
			MAV_BATTERY_TYPE_NIMH    = 4;  //Nickel metal hydride battery
}

/**
 * Enumeration of battery functions
 */
enum MAV_BATTERY_FUNCTION {
	;
	
	final int
			MAV_BATTERY_FUNCTION_UNKNOWN    = 0, //Battery function is unknown
			MAV_BATTERY_FUNCTION_ALL        = 1, //Battery supports all flight systems
			MAV_BATTERY_FUNCTION_PROPULSION = 2, //Battery for the propulsion system
			MAV_BATTERY_FUNCTION_AVIONICS   = 3, //Avionics battery
			MAV_BATTERY_TYPE_PAYLOAD        = 4;  //Payload battery
}

/**
 * Enumeration of VTOL states
 */
enum MAV_VTOL_STATE {
	;
	
	final int
			MAV_VTOL_STATE_UNDEFINED        = 0, //MAV is not configured as VTOL
			MAV_VTOL_STATE_TRANSITION_TO_FW = 1, //VTOL is in transition from multicopter to fixed-wing
			MAV_VTOL_STATE_TRANSITION_TO_MC = 2, //VTOL is in transition from fixed-wing to multicopter
			MAV_VTOL_STATE_MC               = 3, //VTOL is in multicopter state
			MAV_VTOL_STATE_FW               = 4;  //VTOL is in fixed-wing state
}

/**
 * Enumeration of landed detector states
 */
enum MAV_LANDED_STATE {
	;
	
	final int
			MAV_LANDED_STATE_UNDEFINED = 0, //MAV landed state is unknown
			MAV_LANDED_STATE_ON_GROUND = 1, //MAV is landed (on ground)
			MAV_LANDED_STATE_IN_AIR    = 2, //MAV is in air
			MAV_LANDED_STATE_TAKEOFF   = 3, //MAV currently taking off
			MAV_LANDED_STATE_LANDING   = 4;  //MAV currently landing
}

/**
 * Enumeration of the ADSB altimeter types
 */
enum ADSB_ALTITUDE_TYPE {
	;
	
	final int
			ADSB_ALTITUDE_TYPE_PRESSURE_QNH = 0, //Altitude reported from a Baro source using QNH reference
			ADSB_ALTITUDE_TYPE_GEOMETRIC    = 1;  //Altitude reported from a GNSS source
}

/**
 * ADSB classification for the type of vehicle emitting the transponder signal
 */
enum ADSB_EMITTER_TYPE {
	;
	
	final int
			ADSB_EMITTER_TYPE_NO_INFO           = 0, //ADSB classification for the type of vehicle emitting the transponder signal
			ADSB_EMITTER_TYPE_LIGHT             = 1,
			ADSB_EMITTER_TYPE_SMALL             = 2,
			ADSB_EMITTER_TYPE_LARGE             = 3,
			ADSB_EMITTER_TYPE_HIGH_VORTEX_LARGE = 4,
			ADSB_EMITTER_TYPE_HEAVY             = 5,
			ADSB_EMITTER_TYPE_HIGHLY_MANUV      = 6,
			ADSB_EMITTER_TYPE_ROTOCRAFT         = 7,
			ADSB_EMITTER_TYPE_UNASSIGNED        = 8,
			ADSB_EMITTER_TYPE_GLIDER            = 9,
			ADSB_EMITTER_TYPE_LIGHTER_AIR       = 10,
			ADSB_EMITTER_TYPE_PARACHUTE         = 11,
			ADSB_EMITTER_TYPE_ULTRA_LIGHT       = 12,
			ADSB_EMITTER_TYPE_UNASSIGNED2       = 13,
			ADSB_EMITTER_TYPE_UAV               = 14,
			ADSB_EMITTER_TYPE_SPACE             = 15,
			ADSB_EMITTER_TYPE_UNASSGINED3       = 16,
			ADSB_EMITTER_TYPE_EMERGENCY_SURFACE = 17,
			ADSB_EMITTER_TYPE_SERVICE_SURFACE   = 18,
			ADSB_EMITTER_TYPE_POINT_OBSTACLE    = 19;
}

/**
 * These flags indicate status such as data validity of each data source. Set = data valid
 */
enum ADSB_FLAGS {
	;
	
	final int
			ADSB_FLAGS_VALID_COORDS   = 1, //These flags indicate status such as data validity of each data source. Set = data valid
			ADSB_FLAGS_VALID_ALTITUDE = 2,
			ADSB_FLAGS_VALID_HEADING  = 4,
			ADSB_FLAGS_VALID_VELOCITY = 8,
			ADSB_FLAGS_VALID_CALLSIGN = 16,
			ADSB_FLAGS_VALID_SQUAWK   = 32,
			ADSB_FLAGS_SIMULATED      = 64;
}

/**
 * Bitmask of options for the MAV_CMD_DO_REPOSITION
 */
enum MAV_DO_REPOSITION_FLAGS {
	;
	
	final int
			MAV_DO_REPOSITION_FLAGS_CHANGE_MODE = 1;  //The aircraft should immediately transition into guided. This should not be set for follow me application
}

/**
 * Flags in EKF_STATUS message
 */
enum ESTIMATOR_STATUS_FLAGS {
	;
	
	final int
			ESTIMATOR_ATTITUDE       = 1, //True if the attitude estimate is good
			ESTIMATOR_VELOCITY_HORIZ = 2, //True if the horizontal velocity estimate is good
			ESTIMATOR_VELOCITY_VERT  = 4, //True if the  vertical velocity estimate is good
			ESTIMATOR_POS_HORIZ_REL  = 8, //True if the horizontal position (relative) estimate is good
			ESTIMATOR_POS_HORIZ_ABS  = 16, //True if the horizontal position (absolute) estimate is good
			ESTIMATOR_POS_VERT_ABS   = 32, //True if the vertical position (absolute) estimate is good
			ESTIMATOR_POS_VERT_AGL   = 64, //True if the vertical position (above ground) estimate is good
	/**
	 * True if the EKF is in a constant position mode and is not using external measurements (eg GPS or optical
	 * flow
	 */
	ESTIMATOR_CONST_POS_MODE             = 128,
			ESTIMATOR_PRED_POS_HORIZ_REL = 256, //True if the EKF has sufficient data to enter a mode that will provide a (relative) position estimat
			ESTIMATOR_PRED_POS_HORIZ_ABS = 512, //True if the EKF has sufficient data to enter a mode that will provide a (absolute) position estimat
			ESTIMATOR_GPS_GLITCH         = 1024;  //True if the EKF has detected a GPS glitch
}

/**
 * throttle as a percentage from 0 ~ 100
 */
enum MOTOR_TEST_THROTTLE_TYPE {
	;
	
	final int
			MOTOR_TEST_THROTTLE_PERCENT = 0, //throttle as a percentage from 0 ~ 100
			MOTOR_TEST_THROTTLE_PWM     = 1, //throttle as an absolute PWM value (normally in range of 1000~2000)
			MOTOR_TEST_THROTTLE_PILOT   = 2;  //throttle pass-through from pilot's transmitter
}

/**
 * ignore altitude field
 */
enum GPS_INPUT_IGNORE_FLAGS {
	;
	
	final int
			GPS_INPUT_IGNORE_FLAG_ALT                 = 1, //ignore altitude field
			GPS_INPUT_IGNORE_FLAG_HDOP                = 2, //ignore hdop field
			GPS_INPUT_IGNORE_FLAG_VDOP                = 4, //ignore vdop field
			GPS_INPUT_IGNORE_FLAG_VEL_HORIZ           = 8, //ignore horizontal velocity field (vn and ve)
			GPS_INPUT_IGNORE_FLAG_VEL_VERT            = 16, //ignore vertical velocity field (vd)
			GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY      = 32, //ignore speed accuracy field
			GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY = 64, //ignore horizontal accuracy field
			GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY   = 128;  //ignore vertical accuracy field
}

/**
 * Possible actions an aircraft can take to avoid a collision.
 */
enum MAV_COLLISION_ACTION {
	;
	
	final int
			MAV_COLLISION_ACTION_NONE               = 0, //Ignore any potential collisions
			MAV_COLLISION_ACTION_REPORT             = 1, //Report potential collision
			MAV_COLLISION_ACTION_ASCEND_OR_DESCEND  = 2, //Ascend or Descend to avoid threat
			MAV_COLLISION_ACTION_MOVE_HORIZONTALLY  = 3, //Move horizontally to avoid threat
			MAV_COLLISION_ACTION_MOVE_PERPENDICULAR = 4, //Aircraft to move perpendicular to the collision's velocity vector
			MAV_COLLISION_ACTION_RTL                = 5, //Aircraft to fly directly back to its launch point
			MAV_COLLISION_ACTION_HOVER              = 6;  //Aircraft to stop in place
}

/**
 * Aircraft-rated danger from this threat.
 */
enum MAV_COLLISION_THREAT_LEVEL {
	;
	
	final int
			MAV_COLLISION_THREAT_LEVEL_NONE = 0, //Not a threat
			MAV_COLLISION_THREAT_LEVEL_LOW  = 1, //Craft is mildly concerned about this threat
			MAV_COLLISION_THREAT_LEVEL_HIGH = 2;  //Craft is panicing, and may take actions to avoid threat
}

/**
 * Source of information about this collision.
 */
enum MAV_COLLISION_SRC {
	;
	
	final int
			MAV_COLLISION_SRC_ADSB                   = 0, //ID field references ADSB_VEHICLE packets
			MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT = 1;  //ID field references MAVLink SRC ID
}

/**
 * Type of GPS fix
 */
enum GPS_FIX_TYPE {
	;
	
	final int
			GPS_FIX_TYPE_NO_GPS    = 0, //No GPS connected
			GPS_FIX_TYPE_NO_FIX    = 1, //No position information, GPS is connected
			GPS_FIX_TYPE_2D_FIX    = 2, //2D position
			GPS_FIX_TYPE_3D_FIX    = 3, //3D position
			GPS_FIX_TYPE_DGPS      = 4, //DGPS/SBAS aided 3D position
			GPS_FIX_TYPE_RTK_FLOAT = 5, //RTK float, 3D position
			GPS_FIX_TYPE_RTK_FIXED = 6, //RTK Fixed, 3D position
			GPS_FIX_TYPE_STATIC    = 7, //Static fixed, typically used for base stations
			GPS_FIX_TYPE_PPP       = 8;  //PPP, 3D position.
}

/**
 * Type of landing target
 */
enum LANDING_TARGET_TYPE {
	;
	
	final int
			LANDING_TARGET_TYPE_LIGHT_BEACON    = 0, //Landing target signaled by light beacon (ex: IR-LOCK)
			LANDING_TARGET_TYPE_RADIO_BEACON    = 1, //Landing target signaled by radio beacon (ex: ILS, NDB)
			LANDING_TARGET_TYPE_VISION_FIDUCIAL = 2, //Landing target represented by a fiducial marker (ex: ARTag)
			LANDING_TARGET_TYPE_VISION_OTHER    = 3;  //Landing target represented by a pre-defined visual shape/feature (ex: X-marker, H-marker, square)
}

/**
 * Direction of VTOL transition
 */
enum VTOL_TRANSITION_HEADING {
	;
	
	final int
			VTOL_TRANSITION_HEADING_VEHICLE_DEFAULT = 0, //Respect the heading configuration of the vehicle.
			VTOL_TRANSITION_HEADING_NEXT_WAYPOINT   = 1, //Use the heading pointing towards the next waypoint.
			VTOL_TRANSITION_HEADING_TAKEOFF         = 2, //Use the heading on takeoff (while sitting on the ground).
			VTOL_TRANSITION_HEADING_SPECIFIED       = 3, //Use the specified heading in parameter 4.
	/**
	 * Use the current heading when reaching takeoff altitude (potentially facing the wind when weather-vaning
	 * is active)
	 */
	VTOL_TRANSITION_HEADING_ANY = 4;
}

/**
 * Camera capability flags (Bitmap).
 */
enum CAMERA_CAP_FLAGS {
	;
	
	final int
			CAMERA_CAP_FLAGS_CAPTURE_VIDEO                   = 1, //Camera is able to record video.
			CAMERA_CAP_FLAGS_CAPTURE_IMAGE                   = 2, //Camera is able to capture images.
			CAMERA_CAP_FLAGS_HAS_MODES                       = 4, //Camera has separate Video and Image/Photo modes (MAV_CMD_SET_CAMERA_MODE)
			CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE = 8, //Camera can capture images while in video mode
			CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE = 16, //Camera can capture videos while in Photo/Image mode
			CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE           = 32;  //Camera has image survey mode (MAV_CMD_SET_CAMERA_MODE)
}

/**
 * Result from a PARAM_EXT_SET message.
 */
enum PARAM_ACK {
	;
	
	final int
			PARAM_ACK_ACCEPTED          = 0, //Parameter value ACCEPTED and SET
			PARAM_ACK_VALUE_UNSUPPORTED = 1, //Parameter value UNKNOWN/UNSUPPORTED
			PARAM_ACK_FAILED            = 2, //Parameter failed to set
	/**
	 * Parameter value received but not yet validated or set. A subsequent PARAM_EXT_ACK will follow once operation
	 * is completed with the actual result. These are for parameters that may take longer to set. Instead of
	 * waiting for an ACK and potentially timing out, you will immediately receive this response to let you
	 * know it was received
	 */
	PARAM_ACK_IN_PROGRESS = 3;
}

/**
 * Camera Modes.
 */
enum CAMERA_MODE {
	;
	
	final int
			CAMERA_MODE_IMAGE        = 0, //Camera is in image/photo capture mode.
			CAMERA_MODE_VIDEO        = 1, //Camera is in video capture mode.
			CAMERA_MODE_IMAGE_SURVEY = 2;  //Camera is in image survey capture mode. It allows for camera controller to do specific settings for surveys
}

/**
 * Not a specific reason
 */
enum MAV_ARM_AUTH_DENIED_REASON {
	;
	
	final int
			MAV_ARM_AUTH_DENIED_REASON_GENERIC          = 0, //Not a specific reason
			MAV_ARM_AUTH_DENIED_REASON_NONE             = 1, //Authorizer will send the error as string to GCS
			MAV_ARM_AUTH_DENIED_REASON_INVALID_WAYPOINT = 2, //At least one waypoint have a invalid value
			MAV_ARM_AUTH_DENIED_REASON_TIMEOUT          = 3, //Timeout in the authorizer process(in case it depends on network)
	/**
	 * Airspace of the mission in use by another vehicle, second result parameter can have the waypoint id that
	 * caused it to be denied
	 */
	MAV_ARM_AUTH_DENIED_REASON_AIRSPACE_IN_USE     = 4,
			MAV_ARM_AUTH_DENIED_REASON_BAD_WEATHER = 5;  //Weather is not good to fly
}

/**
 * Available autopilot modes for ualberta uav
 */
enum UALBERTA_AUTOPILOT_MODE {
	MODE_MANUAL_DIRECT, //Raw input pulse widts sent to output
	MODE_MANUAL_SCALED, //Inputs are normalized using calibration, the converted back to raw pulse widths for output
	MODE_AUTO_PID_ATT, //dfsdfs
	MODE_AUTO_PID_VEL, //dfsfds
	MODE_AUTO_PID_POS;  //dfsdfsdfs
}

/**
 * Navigation filter mode
 */
enum UALBERTA_NAV_MODE {
	NAV_AHRS_INIT, //Navigation filter mode
	NAV_AHRS, //AHRS mode
	NAV_INS_GPS_INIT, //INS/GPS initialization mode
	NAV_INS_GPS;  //INS/GPS mode
}

/**
 * Mode currently commanded by pilot
 */
enum UALBERTA_PILOT_MODE {
	PILOT_MANUAL, //sdf
	PILOT_AUTO, //dfs
	PILOT_ROTO;  //Rotomotion mode
}
