
package org.noname;

import java.util.*;

import static org.unirail.BlackBox.BitUtils.*;

import org.unirail.BlackBox.Host.Pack.Meta.Field.Bounds;

import java.io.IOException;

public class Test extends GroundControl {


	public static class HEARTBEAT extends GroundControl.HEARTBEAT {
		public void custom_mode_SET(long src) //A bitfield for use for autopilot-specific flags.
		{ set_bytes((src) & -1L, 4, data, 0); }

		public void mavlink_version_SET(char src) //MAVLink version, not writable by user, gets added by protocol because of magic data type: char_mavlink_versio
		{ set_bytes((char) (src) & -1L, 1, data, 4); }

		public void type_SET(@MAV_TYPE int src) //Type of the MAV (quadrotor, helicopter, etc., up to 15 types, defined in MAV_TYPE ENUM)
		{ set_bits(-0 + src, 5, data, 40); }

		public void autopilot_SET(@MAV_AUTOPILOT int src) //Autopilot type / class. defined in MAV_AUTOPILOT ENUM
		{ set_bits(-0 + src, 5, data, 45); }

		public void base_mode_SET(@MAV_MODE_FLAG int src) //System mode bitfield, see MAV_MODE_FLAG ENUM in mavlink/include/mavlink_types.h
		{
			long id = 0;
			switch (src)
			{
				case MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED:
					id = 0;
					break;
				case MAV_MODE_FLAG.MAV_MODE_FLAG_TEST_ENABLED:
					id = 1;
					break;
				case MAV_MODE_FLAG.MAV_MODE_FLAG_AUTO_ENABLED:
					id = 2;
					break;
				case MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED:
					id = 3;
					break;
				case MAV_MODE_FLAG.MAV_MODE_FLAG_STABILIZE_ENABLED:
					id = 4;
					break;
				case MAV_MODE_FLAG.MAV_MODE_FLAG_HIL_ENABLED:
					id = 5;
					break;
				case MAV_MODE_FLAG.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED:
					id = 6;
					break;
				case MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED:
					id = 7;
					break;
				default:
					assert (false);//("Unknown enum" + id);
			}
			set_bits(id, 4, data, 50);
		}

		public void system_status_SET(@MAV_STATE int src) //System status flag, see MAV_STATE ENUM
		{ set_bits(-0 + src, 4, data, 54); }
	}

	public static class SYS_STATUS extends GroundControl.SYS_STATUS {
		public void load_SET(char src) //Maximum usage in percent of the mainloop time, (0%: 0, 100%: 1000) should be always below 1000
		{ set_bytes((char) (src) & -1L, 2, data, 0); }

		public void voltage_battery_SET(char src) //Battery voltage, in millivolts (1 = 1 millivolt)
		{ set_bytes((char) (src) & -1L, 2, data, 2); }

		/**
		 * Communication drops in percent, (0%: 0, 100%: 10'000), (UART, I2C, SPI, CAN), dropped packets on all links
		 * (packets that were corrupted on reception on the MAV
		 */
		public void drop_rate_comm_SET(char src) { set_bytes((char) (src) & -1L, 2, data, 4); }

		/**
		 * Communication errors (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted
		 * on reception on the MAV
		 */
		public void errors_comm_SET(char src) { set_bytes((char) (src) & -1L, 2, data, 6); }

		public void errors_count1_SET(char src) //Autopilot-specific errors
		{ set_bytes((char) (src) & -1L, 2, data, 8); }

		public void errors_count2_SET(char src) //Autopilot-specific errors
		{ set_bytes((char) (src) & -1L, 2, data, 10); }

		public void errors_count3_SET(char src) //Autopilot-specific errors
		{ set_bytes((char) (src) & -1L, 2, data, 12); }

		public void errors_count4_SET(char src) //Autopilot-specific errors
		{ set_bytes((char) (src) & -1L, 2, data, 14); }

		public void current_battery_SET(short src) //Battery current, in 10*milliamperes (1 = 10 milliampere), -1: autopilot does not measure the curren
		{ set_bytes((short) (src) & -1L, 2, data, 16); }

		public void battery_remaining_SET(byte src) //Remaining battery energy: (0%: 0, 100%: 100), -1: autopilot estimate the remaining battery
		{ set_bytes((byte) (src) & -1L, 1, data, 18); }

		/**
		 * Bitmask showing which onboard controllers and sensors are present. Value of 0: not present. Value of 1:
		 * present. Indices defined by ENUM MAV_SYS_STATUS_SENSO
		 */
		public void onboard_control_sensors_present_SET(@MAV_SYS_STATUS_SENSOR int src) {
			long id = 0;
			switch (src)
			{
				case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO:
					id = 0;
					break;
				case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL:
					id = 1;
					break;
				case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG:
					id = 2;
					break;
				case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE:
					id = 3;
					break;
				case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE:
					id = 4;
					break;
				case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_GPS:
					id = 5;
					break;
				case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW:
					id = 6;
					break;
				case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION:
					id = 7;
					break;
				case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION:
					id = 8;
					break;
				case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH:
					id = 9;
					break;
				case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL:
					id = 10;
					break;
				case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION:
					id = 11;
					break;
				case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION:
					id = 12;
					break;
				case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL:
					id = 13;
					break;
				case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL:
					id = 14;
					break;
				case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS:
					id = 15;
					break;
				case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER:
					id = 16;
					break;
				case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO2:
					id = 17;
					break;
				case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2:
					id = 18;
					break;
				case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG2:
					id = 19;
					break;
				case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE:
					id = 20;
					break;
				case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_AHRS:
					id = 21;
					break;
				case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN:
					id = 22;
					break;
				case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR:
					id = 23;
					break;
				case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_LOGGING:
					id = 24;
					break;
				case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_BATTERY:
					id = 25;
					break;
				default:
					assert (false);//("Unknown enum" + id);
			}
			set_bits(id, 5, data, 152);
		}

		/**
		 * Bitmask showing which onboard controllers and sensors are enabled:  Value of 0: not enabled. Value of
		 * 1: enabled. Indices defined by ENUM MAV_SYS_STATUS_SENSO
		 */
		public void onboard_control_sensors_enabled_SET(@MAV_SYS_STATUS_SENSOR int src) {
			long id = 0;
			switch (src)
			{
				case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO:
					id = 0;
					break;
				case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL:
					id = 1;
					break;
				case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG:
					id = 2;
					break;
				case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE:
					id = 3;
					break;
				case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE:
					id = 4;
					break;
				case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_GPS:
					id = 5;
					break;
				case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW:
					id = 6;
					break;
				case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION:
					id = 7;
					break;
				case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION:
					id = 8;
					break;
				case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH:
					id = 9;
					break;
				case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL:
					id = 10;
					break;
				case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION:
					id = 11;
					break;
				case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION:
					id = 12;
					break;
				case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL:
					id = 13;
					break;
				case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL:
					id = 14;
					break;
				case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS:
					id = 15;
					break;
				case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER:
					id = 16;
					break;
				case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO2:
					id = 17;
					break;
				case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2:
					id = 18;
					break;
				case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG2:
					id = 19;
					break;
				case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE:
					id = 20;
					break;
				case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_AHRS:
					id = 21;
					break;
				case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN:
					id = 22;
					break;
				case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR:
					id = 23;
					break;
				case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_LOGGING:
					id = 24;
					break;
				case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_BATTERY:
					id = 25;
					break;
				default:
					assert (false);//("Unknown enum" + id);
			}
			set_bits(id, 5, data, 157);
		}

		/**
		 * Bitmask showing which onboard controllers and sensors are operational or have an error:  Value of 0: not
		 * enabled. Value of 1: enabled. Indices defined by ENUM MAV_SYS_STATUS_SENSO
		 */
		public void onboard_control_sensors_health_SET(@MAV_SYS_STATUS_SENSOR int src) {
			long id = 0;
			switch (src)
			{
				case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO:
					id = 0;
					break;
				case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL:
					id = 1;
					break;
				case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG:
					id = 2;
					break;
				case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE:
					id = 3;
					break;
				case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE:
					id = 4;
					break;
				case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_GPS:
					id = 5;
					break;
				case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW:
					id = 6;
					break;
				case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION:
					id = 7;
					break;
				case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION:
					id = 8;
					break;
				case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH:
					id = 9;
					break;
				case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL:
					id = 10;
					break;
				case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION:
					id = 11;
					break;
				case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION:
					id = 12;
					break;
				case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL:
					id = 13;
					break;
				case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL:
					id = 14;
					break;
				case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS:
					id = 15;
					break;
				case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER:
					id = 16;
					break;
				case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO2:
					id = 17;
					break;
				case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2:
					id = 18;
					break;
				case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG2:
					id = 19;
					break;
				case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE:
					id = 20;
					break;
				case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_AHRS:
					id = 21;
					break;
				case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN:
					id = 22;
					break;
				case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR:
					id = 23;
					break;
				case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_LOGGING:
					id = 24;
					break;
				case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_BATTERY:
					id = 25;
					break;
				default:
					assert (false);//("Unknown enum" + id);
			}
			set_bits(id, 5, data, 162);
		}
	}

	public static class SYSTEM_TIME extends GroundControl.SYSTEM_TIME {
		public void time_boot_ms_SET(long src) //Timestamp of the component clock since boot time in milliseconds.
		{ set_bytes((src) & -1L, 4, data, 0); }

		public void time_unix_usec_SET(long src) //Timestamp of the master clock in microseconds since UNIX epoch.
		{ set_bytes((src) & -1L, 8, data, 4); }
	}

	public static class PING extends GroundControl.PING {
		public void seq_SET(long src) //PING sequence
		{ set_bytes((src) & -1L, 4, data, 0); }

		public void time_usec_SET(long src) //Unix timestamp in microseconds or since system boot if smaller than MAVLink epoch (1.1.2009)
		{ set_bytes((src) & -1L, 8, data, 4); }

		/**
		 * 0: request ping from all receiving systems, if greater than 0: message is a ping response and number is
		 * the system id of the requesting syste
		 */
		public void target_system_SET(char src) { set_bytes((char) (src) & -1L, 1, data, 12); }

		/**
		 * 0: request ping from all receiving components, if greater than 0: message is a ping response and number
		 * is the system id of the requesting syste
		 */
		public void target_component_SET(char src) { set_bytes((char) (src) & -1L, 1, data, 13); }
	}

	public static class CHANGE_OPERATOR_CONTROL extends GroundControl.CHANGE_OPERATOR_CONTROL {
		public void target_system_SET(char src) //System the GCS requests control for
		{ set_bytes((char) (src) & -1L, 1, data, 0); }

		public void control_request_SET(char src) //0: request control of this MAV, 1: Release control of this MAV
		{ set_bytes((char) (src) & -1L, 1, data, 1); }

		/**
		 * 0: key as plaintext, 1-255: future, different hashing/encryption variants. The GCS should in general use
		 * the safest mode possible initially and then gradually move down the encryption level if it gets a NACK
		 * message indicating an encryption mismatch
		 */
		public void version_SET(char src) { set_bytes((char) (src) & -1L, 1, data, 2); }

		/**
		 * Password / Key, depending on version plaintext or encrypted. 25 or less characters, NULL terminated. The
		 * characters may involve A-Z, a-z, 0-9, and "!?,.-
		 */
		public void passkey_SET(String src, Bounds.Inside ph) {passkey_SET(src.toCharArray(), 0, src.length(), ph);}

		/**
		 * Password / Key, depending on version plaintext or encrypted. 25 or less characters, NULL terminated. The
		 * characters may involve A-Z, a-z, 0-9, and "!?,.-
		 */
		public void passkey_SET(char[] src, int pos, int items, Bounds.Inside ph) {
			if (ph.field_bit != 24 && insert_field(ph, 24, items) || !try_visit_item(ph, 0))
				insert_item(ph, 0, items);
			for (int BYTE = ph.BYTE, src_max = pos + ph.items; pos < src_max; pos++, BYTE += 2)
				set_bytes((short) (src[pos]) & -1L, 2, data, BYTE);
		}
	}

	public static class CHANGE_OPERATOR_CONTROL_ACK extends GroundControl.CHANGE_OPERATOR_CONTROL_ACK {
		public void gcs_system_id_SET(char src) //ID of the GCS this message
		{ set_bytes((char) (src) & -1L, 1, data, 0); }

		public void control_request_SET(char src) //0: request control of this MAV, 1: Release control of this MAV
		{ set_bytes((char) (src) & -1L, 1, data, 1); }

		/**
		 * 0: ACK, 1: NACK: Wrong passkey, 2: NACK: Unsupported passkey encryption method, 3: NACK: Already under
		 * contro
		 */
		public void ack_SET(char src) { set_bytes((char) (src) & -1L, 1, data, 2); }
	}

	public static class AUTH_KEY extends GroundControl.AUTH_KEY {
		public void key_SET(String src, Bounds.Inside ph)//key
		{key_SET(src.toCharArray(), 0, src.length(), ph);}

		public void key_SET(char[] src, int pos, int items, Bounds.Inside ph) //key
		{
			if (ph.field_bit != 0 && insert_field(ph, 0, items) || !try_visit_item(ph, 0))
				insert_item(ph, 0, items);
			for (int BYTE = ph.BYTE, src_max = pos + ph.items; pos < src_max; pos++, BYTE += 2)
				set_bytes((short) (src[pos]) & -1L, 2, data, BYTE);
		}
	}

	public static class SET_MODE extends GroundControl.SET_MODE {
		public void custom_mode_SET(long src) //The new autopilot-specific mode. This field can be ignored by an autopilot.
		{ set_bytes((src) & -1L, 4, data, 0); }

		public void target_system_SET(char src) //The system setting the mode
		{ set_bytes((char) (src) & -1L, 1, data, 4); }

		public void base_mode_SET(@MAV_MODE int src) //The new base mode
		{
			long id = 0;
			switch (src)
			{
				case MAV_MODE.MAV_MODE_PREFLIGHT:
					id = 0;
					break;
				case MAV_MODE.MAV_MODE_MANUAL_DISARMED:
					id = 1;
					break;
				case MAV_MODE.MAV_MODE_TEST_DISARMED:
					id = 2;
					break;
				case MAV_MODE.MAV_MODE_STABILIZE_DISARMED:
					id = 3;
					break;
				case MAV_MODE.MAV_MODE_GUIDED_DISARMED:
					id = 4;
					break;
				case MAV_MODE.MAV_MODE_AUTO_DISARMED:
					id = 5;
					break;
				case MAV_MODE.MAV_MODE_MANUAL_ARMED:
					id = 6;
					break;
				case MAV_MODE.MAV_MODE_TEST_ARMED:
					id = 7;
					break;
				case MAV_MODE.MAV_MODE_STABILIZE_ARMED:
					id = 8;
					break;
				case MAV_MODE.MAV_MODE_GUIDED_ARMED:
					id = 9;
					break;
				case MAV_MODE.MAV_MODE_AUTO_ARMED:
					id = 10;
					break;
				default:
					assert (false);//("Unknown enum" + id);
			}
			set_bits(id, 4, data, 40);
		}
	}

	public static class PARAM_REQUEST_READ extends GroundControl.PARAM_REQUEST_READ {
		public void target_system_SET(char src) //System ID
		{ set_bytes((char) (src) & -1L, 1, data, 0); }

		public void target_component_SET(char src) //Component ID
		{ set_bytes((char) (src) & -1L, 1, data, 1); }

		public void param_index_SET(short src) //Parameter index. Send -1 to use the param ID field as identifier (else the param id will be ignored
		{ set_bytes((short) (src) & -1L, 2, data, 2); }

		/**
		 * Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT
		 * null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes
		 * storage if the ID is stored as strin
		 */
		public void param_id_SET(String src, Bounds.Inside ph) {param_id_SET(src.toCharArray(), 0, src.length(), ph);}

		/**
		 * Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT
		 * null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes
		 * storage if the ID is stored as strin
		 */
		public void param_id_SET(char[] src, int pos, int items, Bounds.Inside ph) {
			if (ph.field_bit != 32 && insert_field(ph, 32, items) || !try_visit_item(ph, 0))
				insert_item(ph, 0, items);
			for (int BYTE = ph.BYTE, src_max = pos + ph.items; pos < src_max; pos++, BYTE += 2)
				set_bytes((short) (src[pos]) & -1L, 2, data, BYTE);
		}
	}

	public static class PARAM_REQUEST_LIST extends GroundControl.PARAM_REQUEST_LIST {
		public void target_system_SET(char src) //System ID
		{ set_bytes((char) (src) & -1L, 1, data, 0); }

		public void target_component_SET(char src) //Component ID
		{ set_bytes((char) (src) & -1L, 1, data, 1); }
	}

	public static class PARAM_VALUE extends GroundControl.PARAM_VALUE {
		public void param_count_SET(char src) //Total number of onboard parameters
		{ set_bytes((char) (src) & -1L, 2, data, 0); }

		public void param_index_SET(char src) //Index of this onboard parameter
		{ set_bytes((char) (src) & -1L, 2, data, 2); }

		public void param_value_SET(float src) //Onboard parameter value
		{ set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 4); }

		public void param_type_SET(@MAV_PARAM_TYPE int src) //Onboard parameter type: see the MAV_PARAM_TYPE enum for supported data types.
		{ set_bits(-1 + src, 4, data, 64); }

		/**
		 * Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT
		 * null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes
		 * storage if the ID is stored as strin
		 */
		public void param_id_SET(String src, Bounds.Inside ph) {param_id_SET(src.toCharArray(), 0, src.length(), ph);}

		/**
		 * Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT
		 * null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes
		 * storage if the ID is stored as strin
		 */
		public void param_id_SET(char[] src, int pos, int items, Bounds.Inside ph) {
			if (ph.field_bit != 68 && insert_field(ph, 68, items) || !try_visit_item(ph, 0))
				insert_item(ph, 0, items);
			for (int BYTE = ph.BYTE, src_max = pos + ph.items; pos < src_max; pos++, BYTE += 2)
				set_bytes((short) (src[pos]) & -1L, 2, data, BYTE);
		}
	}

	public static class PARAM_SET extends GroundControl.PARAM_SET {
		public void target_system_SET(char src) //System ID
		{ set_bytes((char) (src) & -1L, 1, data, 0); }

		public void target_component_SET(char src) //Component ID
		{ set_bytes((char) (src) & -1L, 1, data, 1); }

		public void param_value_SET(float src) //Onboard parameter value
		{ set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 2); }

		public void param_type_SET(@MAV_PARAM_TYPE int src) //Onboard parameter type: see the MAV_PARAM_TYPE enum for supported data types.
		{ set_bits(-1 + src, 4, data, 48); }

		/**
		 * Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT
		 * null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes
		 * storage if the ID is stored as strin
		 */
		public void param_id_SET(String src, Bounds.Inside ph) {param_id_SET(src.toCharArray(), 0, src.length(), ph);}

		/**
		 * Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT
		 * null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes
		 * storage if the ID is stored as strin
		 */
		public void param_id_SET(char[] src, int pos, int items, Bounds.Inside ph) {
			if (ph.field_bit != 52 && insert_field(ph, 52, items) || !try_visit_item(ph, 0))
				insert_item(ph, 0, items);
			for (int BYTE = ph.BYTE, src_max = pos + ph.items; pos < src_max; pos++, BYTE += 2)
				set_bytes((short) (src[pos]) & -1L, 2, data, BYTE);
		}
	}

	public static class GPS_RAW_INT extends GroundControl.GPS_RAW_INT {
		public void eph_SET(char src) //GPS HDOP horizontal dilution of position (unitless). If unknown, set to: UINT16_MAX
		{ set_bytes((char) (src) & -1L, 2, data, 0); }

		public void epv_SET(char src) //GPS VDOP vertical dilution of position (unitless). If unknown, set to: UINT16_MAX
		{ set_bytes((char) (src) & -1L, 2, data, 2); }

		public void vel_SET(char src) //GPS ground speed (m/s * 100). If unknown, set to: UINT16_MAX
		{ set_bytes((char) (src) & -1L, 2, data, 4); }

		/**
		 * Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If
		 * unknown, set to: UINT16_MA
		 */
		public void cog_SET(char src) { set_bytes((char) (src) & -1L, 2, data, 6); }

		public void time_usec_SET(long src) //Timestamp (microseconds since UNIX epoch or microseconds since system boot)
		{ set_bytes((src) & -1L, 8, data, 8); }

		public void lat_SET(int src) //Latitude (WGS84, EGM96 ellipsoid), in degrees * 1E7
		{ set_bytes((int) (src) & -1L, 4, data, 16); }

		public void lon_SET(int src) //Longitude (WGS84, EGM96 ellipsoid), in degrees * 1E7
		{ set_bytes((int) (src) & -1L, 4, data, 20); }

		/**
		 * Altitude (AMSL, NOT WGS84), in meters * 1000 (positive for up). Note that virtually all GPS modules provide
		 * the AMSL altitude in addition to the WGS84 altitude
		 */
		public void alt_SET(int src) { set_bytes((int) (src) & -1L, 4, data, 24); }

		public void satellites_visible_SET(char src) //Number of satellites visible. If unknown, set to 255
		{ set_bytes((char) (src) & -1L, 1, data, 28); }

		public void fix_type_SET(@GPS_FIX_TYPE int src) //See the GPS_FIX_TYPE enum.
		{ set_bits(-0 + src, 4, data, 232); }

		public void alt_ellipsoid_SET(int src, Bounds.Inside ph)//Altitude (above WGS84, EGM96 ellipsoid), in meters * 1000 (positive for up).
		{
			if (ph.field_bit != 236) insert_field(ph, 236, 0);
			set_bytes((int) (src) & -1L, 4, data, ph.BYTE);
		}

		public void h_acc_SET(long src, Bounds.Inside ph) //Position uncertainty in meters * 1000 (positive for up).
		{
			if (ph.field_bit != 237) insert_field(ph, 237, 0);
			set_bytes((src) & -1L, 4, data, ph.BYTE);
		}

		public void v_acc_SET(long src, Bounds.Inside ph) //Altitude uncertainty in meters * 1000 (positive for up).
		{
			if (ph.field_bit != 238) insert_field(ph, 238, 0);
			set_bytes((src) & -1L, 4, data, ph.BYTE);
		}

		public void vel_acc_SET(long src, Bounds.Inside ph) //Speed uncertainty in meters * 1000 (positive for up).
		{
			if (ph.field_bit != 239) insert_field(ph, 239, 0);
			set_bytes((src) & -1L, 4, data, ph.BYTE);
		}

		public void hdg_acc_SET(long src, Bounds.Inside ph) //Heading / track uncertainty in degrees * 1e5.
		{
			if (ph.field_bit != 240) insert_field(ph, 240, 0);
			set_bytes((src) & -1L, 4, data, ph.BYTE);
		}
	}

	public static class GPS_STATUS extends GroundControl.GPS_STATUS {
		public void satellites_visible_SET(char src) //Number of satellites visible
		{ set_bytes((char) (src) & -1L, 1, data, 0); }

		public void satellite_prn_SET(char[] src, int pos)  //Global satellite ID
		{
			for (int BYTE = 1, src_max = pos + 20; pos < src_max; pos++, BYTE += 1)
				set_bytes((char) (src[pos]) & -1L, 1, data, BYTE);
		}

		public void satellite_used_SET(char[] src, int pos)  //0: Satellite not used, 1: used for localization
		{
			for (int BYTE = 21, src_max = pos + 20; pos < src_max; pos++, BYTE += 1)
				set_bytes((char) (src[pos]) & -1L, 1, data, BYTE);
		}

		public void satellite_elevation_SET(char[] src, int pos)  //Elevation (0: right on top of receiver, 90: on the horizon) of satellite
		{
			for (int BYTE = 41, src_max = pos + 20; pos < src_max; pos++, BYTE += 1)
				set_bytes((char) (src[pos]) & -1L, 1, data, BYTE);
		}

		public void satellite_azimuth_SET(char[] src, int pos)  //Direction of satellite, 0: 0 deg, 255: 360 deg.
		{
			for (int BYTE = 61, src_max = pos + 20; pos < src_max; pos++, BYTE += 1)
				set_bytes((char) (src[pos]) & -1L, 1, data, BYTE);
		}

		public void satellite_snr_SET(char[] src, int pos)  //Signal to noise ratio of satellite
		{
			for (int BYTE = 81, src_max = pos + 20; pos < src_max; pos++, BYTE += 1)
				set_bytes((char) (src[pos]) & -1L, 1, data, BYTE);
		}
	}

	public static class SCALED_IMU extends GroundControl.SCALED_IMU {
		public void time_boot_ms_SET(long src) //Timestamp (milliseconds since system boot)
		{ set_bytes((src) & -1L, 4, data, 0); }

		public void xacc_SET(short src) //X acceleration (mg)
		{ set_bytes((short) (src) & -1L, 2, data, 4); }

		public void yacc_SET(short src) //Y acceleration (mg)
		{ set_bytes((short) (src) & -1L, 2, data, 6); }

		public void zacc_SET(short src) //Z acceleration (mg)
		{ set_bytes((short) (src) & -1L, 2, data, 8); }

		public void xgyro_SET(short src) //Angular speed around X axis (millirad /sec)
		{ set_bytes((short) (src) & -1L, 2, data, 10); }

		public void ygyro_SET(short src) //Angular speed around Y axis (millirad /sec)
		{ set_bytes((short) (src) & -1L, 2, data, 12); }

		public void zgyro_SET(short src) //Angular speed around Z axis (millirad /sec)
		{ set_bytes((short) (src) & -1L, 2, data, 14); }

		public void xmag_SET(short src) //X Magnetic field (milli tesla)
		{ set_bytes((short) (src) & -1L, 2, data, 16); }

		public void ymag_SET(short src) //Y Magnetic field (milli tesla)
		{ set_bytes((short) (src) & -1L, 2, data, 18); }

		public void zmag_SET(short src) //Z Magnetic field (milli tesla)
		{ set_bytes((short) (src) & -1L, 2, data, 20); }
	}

	public static class RAW_IMU extends GroundControl.RAW_IMU {
		public void time_usec_SET(long src) //Timestamp (microseconds since UNIX epoch or microseconds since system boot)
		{ set_bytes((src) & -1L, 8, data, 0); }

		public void xacc_SET(short src) //X acceleration (raw)
		{ set_bytes((short) (src) & -1L, 2, data, 8); }

		public void yacc_SET(short src) //Y acceleration (raw)
		{ set_bytes((short) (src) & -1L, 2, data, 10); }

		public void zacc_SET(short src) //Z acceleration (raw)
		{ set_bytes((short) (src) & -1L, 2, data, 12); }

		public void xgyro_SET(short src) //Angular speed around X axis (raw)
		{ set_bytes((short) (src) & -1L, 2, data, 14); }

		public void ygyro_SET(short src) //Angular speed around Y axis (raw)
		{ set_bytes((short) (src) & -1L, 2, data, 16); }

		public void zgyro_SET(short src) //Angular speed around Z axis (raw)
		{ set_bytes((short) (src) & -1L, 2, data, 18); }

		public void xmag_SET(short src) //X Magnetic field (raw)
		{ set_bytes((short) (src) & -1L, 2, data, 20); }

		public void ymag_SET(short src) //Y Magnetic field (raw)
		{ set_bytes((short) (src) & -1L, 2, data, 22); }

		public void zmag_SET(short src) //Z Magnetic field (raw)
		{ set_bytes((short) (src) & -1L, 2, data, 24); }
	}

	public static class RAW_PRESSURE extends GroundControl.RAW_PRESSURE {
		public void time_usec_SET(long src) //Timestamp (microseconds since UNIX epoch or microseconds since system boot)
		{ set_bytes((src) & -1L, 8, data, 0); }

		public void press_abs_SET(short src) //Absolute pressure (raw)
		{ set_bytes((short) (src) & -1L, 2, data, 8); }

		public void press_diff1_SET(short src) //Differential pressure 1 (raw, 0 if nonexistant)
		{ set_bytes((short) (src) & -1L, 2, data, 10); }

		public void press_diff2_SET(short src) //Differential pressure 2 (raw, 0 if nonexistant)
		{ set_bytes((short) (src) & -1L, 2, data, 12); }

		public void temperature_SET(short src) //Raw Temperature measurement (raw)
		{ set_bytes((short) (src) & -1L, 2, data, 14); }
	}

	public static class SCALED_PRESSURE extends GroundControl.SCALED_PRESSURE {
		public void time_boot_ms_SET(long src) //Timestamp (milliseconds since system boot)
		{ set_bytes((src) & -1L, 4, data, 0); }

		public void press_abs_SET(float src) //Absolute pressure (hectopascal)
		{ set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 4); }

		public void press_diff_SET(float src) //Differential pressure 1 (hectopascal)
		{ set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 8); }

		public void temperature_SET(short src) //Temperature measurement (0.01 degrees celsius)
		{ set_bytes((short) (src) & -1L, 2, data, 12); }
	}

	public static class ATTITUDE extends GroundControl.ATTITUDE {
		public void time_boot_ms_SET(long src) //Timestamp (milliseconds since system boot)
		{ set_bytes((src) & -1L, 4, data, 0); }

		public void roll_SET(float src) //Roll angle (rad, -pi..+pi)
		{ set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 4); }

		public void pitch_SET(float src) //Pitch angle (rad, -pi..+pi)
		{ set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 8); }

		public void yaw_SET(float src) //Yaw angle (rad, -pi..+pi)
		{ set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 12); }

		public void rollspeed_SET(float src) //Roll angular speed (rad/s)
		{ set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 16); }

		public void pitchspeed_SET(float src) //Pitch angular speed (rad/s)
		{ set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 20); }

		public void yawspeed_SET(float src) //Yaw angular speed (rad/s)
		{ set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 24); }
	}

	public static class ATTITUDE_QUATERNION extends GroundControl.ATTITUDE_QUATERNION {
		public void time_boot_ms_SET(long src) //Timestamp (milliseconds since system boot)
		{ set_bytes((src) & -1L, 4, data, 0); }

		public void q1_SET(float src) //Quaternion component 1, w (1 in null-rotation)
		{ set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 4); }

		public void q2_SET(float src) //Quaternion component 2, x (0 in null-rotation)
		{ set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 8); }

		public void q3_SET(float src) //Quaternion component 3, y (0 in null-rotation)
		{ set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 12); }

		public void q4_SET(float src) //Quaternion component 4, z (0 in null-rotation)
		{ set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 16); }

		public void rollspeed_SET(float src) //Roll angular speed (rad/s)
		{ set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 20); }

		public void pitchspeed_SET(float src) //Pitch angular speed (rad/s)
		{ set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 24); }

		public void yawspeed_SET(float src) //Yaw angular speed (rad/s)
		{ set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 28); }
	}

	public static class LOCAL_POSITION_NED extends GroundControl.LOCAL_POSITION_NED {
		public void time_boot_ms_SET(long src) //Timestamp (milliseconds since system boot)
		{ set_bytes((src) & -1L, 4, data, 0); }

		public void x_SET(float src) //X Position
		{ set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 4); }

		public void y_SET(float src) //Y Position
		{ set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 8); }

		public void z_SET(float src) //Z Position
		{ set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 12); }

		public void vx_SET(float src) //X Speed
		{ set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 16); }

		public void vy_SET(float src) //Y Speed
		{ set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 20); }

		public void vz_SET(float src) //Z Speed
		{ set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 24); }
	}

	public static class GLOBAL_POSITION_INT extends GroundControl.GLOBAL_POSITION_INT {
		public void hdg_SET(char src) //Vehicle heading (yaw angle) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
		{ set_bytes((char) (src) & -1L, 2, data, 0); }

		public void time_boot_ms_SET(long src) //Timestamp (milliseconds since system boot)
		{ set_bytes((src) & -1L, 4, data, 2); }

		public void lat_SET(int src) //Latitude, expressed as degrees * 1E7
		{ set_bytes((int) (src) & -1L, 4, data, 6); }

		public void lon_SET(int src) //Longitude, expressed as degrees * 1E7
		{ set_bytes((int) (src) & -1L, 4, data, 10); }

		/**
		 * Altitude in meters, expressed as * 1000 (millimeters), AMSL (not WGS84 - note that virtually all GPS modules
		 * provide the AMSL as well
		 */
		public void alt_SET(int src) { set_bytes((int) (src) & -1L, 4, data, 14); }

		public void relative_alt_SET(int src) //Altitude above ground in meters, expressed as * 1000 (millimeters)
		{ set_bytes((int) (src) & -1L, 4, data, 18); }

		public void vx_SET(short src) //Ground X Speed (Latitude, positive north), expressed as m/s * 100
		{ set_bytes((short) (src) & -1L, 2, data, 22); }

		public void vy_SET(short src) //Ground Y Speed (Longitude, positive east), expressed as m/s * 100
		{ set_bytes((short) (src) & -1L, 2, data, 24); }

		public void vz_SET(short src) //Ground Z Speed (Altitude, positive down), expressed as m/s * 100
		{ set_bytes((short) (src) & -1L, 2, data, 26); }
	}

	public static class RC_CHANNELS_SCALED extends GroundControl.RC_CHANNELS_SCALED {
		public void time_boot_ms_SET(long src) //Timestamp (milliseconds since system boot)
		{ set_bytes((src) & -1L, 4, data, 0); }

		/**
		 * Servo output port (set of 8 outputs = 1 port). Most MAVs will just use one, but this allows for more than
		 * 8 servos
		 */
		public void port_SET(char src) { set_bytes((char) (src) & -1L, 1, data, 4); }

		public void chan1_scaled_SET(short src) //RC channel 1 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
		{ set_bytes((short) (src) & -1L, 2, data, 5); }

		public void chan2_scaled_SET(short src) //RC channel 2 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
		{ set_bytes((short) (src) & -1L, 2, data, 7); }

		public void chan3_scaled_SET(short src) //RC channel 3 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
		{ set_bytes((short) (src) & -1L, 2, data, 9); }

		public void chan4_scaled_SET(short src) //RC channel 4 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
		{ set_bytes((short) (src) & -1L, 2, data, 11); }

		public void chan5_scaled_SET(short src) //RC channel 5 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
		{ set_bytes((short) (src) & -1L, 2, data, 13); }

		public void chan6_scaled_SET(short src) //RC channel 6 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
		{ set_bytes((short) (src) & -1L, 2, data, 15); }

		public void chan7_scaled_SET(short src) //RC channel 7 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
		{ set_bytes((short) (src) & -1L, 2, data, 17); }

		public void chan8_scaled_SET(short src) //RC channel 8 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
		{ set_bytes((short) (src) & -1L, 2, data, 19); }

		public void rssi_SET(char src) //Receive signal strength indicator, 0: 0%, 100: 100%, 255: invalid/unknown.
		{ set_bytes((char) (src) & -1L, 1, data, 21); }
	}

	public static class RC_CHANNELS_RAW extends GroundControl.RC_CHANNELS_RAW {
		public void chan1_raw_SET(char src) //RC channel 1 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
		{ set_bytes((char) (src) & -1L, 2, data, 0); }

		public void chan2_raw_SET(char src) //RC channel 2 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
		{ set_bytes((char) (src) & -1L, 2, data, 2); }

		public void chan3_raw_SET(char src) //RC channel 3 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
		{ set_bytes((char) (src) & -1L, 2, data, 4); }

		public void chan4_raw_SET(char src) //RC channel 4 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
		{ set_bytes((char) (src) & -1L, 2, data, 6); }

		public void chan5_raw_SET(char src) //RC channel 5 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
		{ set_bytes((char) (src) & -1L, 2, data, 8); }

		public void chan6_raw_SET(char src) //RC channel 6 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
		{ set_bytes((char) (src) & -1L, 2, data, 10); }

		public void chan7_raw_SET(char src) //RC channel 7 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
		{ set_bytes((char) (src) & -1L, 2, data, 12); }

		public void chan8_raw_SET(char src) //RC channel 8 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
		{ set_bytes((char) (src) & -1L, 2, data, 14); }

		public void time_boot_ms_SET(long src) //Timestamp (milliseconds since system boot)
		{ set_bytes((src) & -1L, 4, data, 16); }

		/**
		 * Servo output port (set of 8 outputs = 1 port). Most MAVs will just use one, but this allows for more than
		 * 8 servos
		 */
		public void port_SET(char src) { set_bytes((char) (src) & -1L, 1, data, 20); }

		public void rssi_SET(char src) //Receive signal strength indicator, 0: 0%, 100: 100%, 255: invalid/unknown.
		{ set_bytes((char) (src) & -1L, 1, data, 21); }
	}

	public static class SERVO_OUTPUT_RAW extends GroundControl.SERVO_OUTPUT_RAW {
		public void servo1_raw_SET(char src) //Servo output 1 value, in microseconds
		{ set_bytes((char) (src) & -1L, 2, data, 0); }

		public void servo2_raw_SET(char src) //Servo output 2 value, in microseconds
		{ set_bytes((char) (src) & -1L, 2, data, 2); }

		public void servo3_raw_SET(char src) //Servo output 3 value, in microseconds
		{ set_bytes((char) (src) & -1L, 2, data, 4); }

		public void servo4_raw_SET(char src) //Servo output 4 value, in microseconds
		{ set_bytes((char) (src) & -1L, 2, data, 6); }

		public void servo5_raw_SET(char src) //Servo output 5 value, in microseconds
		{ set_bytes((char) (src) & -1L, 2, data, 8); }

		public void servo6_raw_SET(char src) //Servo output 6 value, in microseconds
		{ set_bytes((char) (src) & -1L, 2, data, 10); }

		public void servo7_raw_SET(char src) //Servo output 7 value, in microseconds
		{ set_bytes((char) (src) & -1L, 2, data, 12); }

		public void servo8_raw_SET(char src) //Servo output 8 value, in microseconds
		{ set_bytes((char) (src) & -1L, 2, data, 14); }

		public void time_usec_SET(long src) //Timestamp (microseconds since system boot)
		{ set_bytes((src) & -1L, 4, data, 16); }

		/**
		 * Servo output port (set of 8 outputs = 1 port). Most MAVs will just use one, but this allows to encode
		 * more than 8 servos
		 */
		public void port_SET(char src) { set_bytes((char) (src) & -1L, 1, data, 20); }

		public void servo9_raw_SET(char src, Bounds.Inside ph)//Servo output 9 value, in microseconds
		{
			if (ph.field_bit != 168) insert_field(ph, 168, 0);
			set_bytes((char) (src) & -1L, 2, data, ph.BYTE);
		}

		public void servo10_raw_SET(char src, Bounds.Inside ph) //Servo output 10 value, in microseconds
		{
			if (ph.field_bit != 169) insert_field(ph, 169, 0);
			set_bytes((char) (src) & -1L, 2, data, ph.BYTE);
		}

		public void servo11_raw_SET(char src, Bounds.Inside ph) //Servo output 11 value, in microseconds
		{
			if (ph.field_bit != 170) insert_field(ph, 170, 0);
			set_bytes((char) (src) & -1L, 2, data, ph.BYTE);
		}

		public void servo12_raw_SET(char src, Bounds.Inside ph) //Servo output 12 value, in microseconds
		{
			if (ph.field_bit != 171) insert_field(ph, 171, 0);
			set_bytes((char) (src) & -1L, 2, data, ph.BYTE);
		}

		public void servo13_raw_SET(char src, Bounds.Inside ph) //Servo output 13 value, in microseconds
		{
			if (ph.field_bit != 172) insert_field(ph, 172, 0);
			set_bytes((char) (src) & -1L, 2, data, ph.BYTE);
		}

		public void servo14_raw_SET(char src, Bounds.Inside ph) //Servo output 14 value, in microseconds
		{
			if (ph.field_bit != 173) insert_field(ph, 173, 0);
			set_bytes((char) (src) & -1L, 2, data, ph.BYTE);
		}

		public void servo15_raw_SET(char src, Bounds.Inside ph) //Servo output 15 value, in microseconds
		{
			if (ph.field_bit != 174) insert_field(ph, 174, 0);
			set_bytes((char) (src) & -1L, 2, data, ph.BYTE);
		}

		public void servo16_raw_SET(char src, Bounds.Inside ph) //Servo output 16 value, in microseconds
		{
			if (ph.field_bit != 175) insert_field(ph, 175, 0);
			set_bytes((char) (src) & -1L, 2, data, ph.BYTE);
		}
	}

	public static class MISSION_REQUEST_PARTIAL_LIST extends GroundControl.MISSION_REQUEST_PARTIAL_LIST {
		public void target_system_SET(char src) //System ID
		{ set_bytes((char) (src) & -1L, 1, data, 0); }

		public void target_component_SET(char src) //Component ID
		{ set_bytes((char) (src) & -1L, 1, data, 1); }

		public void start_index_SET(short src) //Start index, 0 by default
		{ set_bytes((short) (src) & -1L, 2, data, 2); }

		public void end_index_SET(short src) //End index, -1 by default (-1: send list to end). Else a valid index of the list
		{ set_bytes((short) (src) & -1L, 2, data, 4); }

		public void mission_type_SET(@MAV_MISSION_TYPE int src) //Mission type, see MAV_MISSION_TYPE
		{
			long id = 0;
			switch (src)
			{
				case MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION:
					id = 0;
					break;
				case MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE:
					id = 1;
					break;
				case MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY:
					id = 2;
					break;
				case MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL:
					id = 3;
					break;
				default:
					assert (false);//("Unknown enum" + id);
			}
			set_bits(id, 3, data, 48);
		}
	}

	public static class MISSION_WRITE_PARTIAL_LIST extends GroundControl.MISSION_WRITE_PARTIAL_LIST {
		public void target_system_SET(char src) //System ID
		{ set_bytes((char) (src) & -1L, 1, data, 0); }

		public void target_component_SET(char src) //Component ID
		{ set_bytes((char) (src) & -1L, 1, data, 1); }

		public void start_index_SET(short src) //Start index, 0 by default and smaller / equal to the largest index of the current onboard list.
		{ set_bytes((short) (src) & -1L, 2, data, 2); }

		public void end_index_SET(short src) //End index, equal or greater than start index.
		{ set_bytes((short) (src) & -1L, 2, data, 4); }

		public void mission_type_SET(@MAV_MISSION_TYPE int src) //Mission type, see MAV_MISSION_TYPE
		{
			long id = 0;
			switch (src)
			{
				case MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION:
					id = 0;
					break;
				case MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE:
					id = 1;
					break;
				case MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY:
					id = 2;
					break;
				case MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL:
					id = 3;
					break;
				default:
					assert (false);//("Unknown enum" + id);
			}
			set_bits(id, 3, data, 48);
		}
	}

	public static class MISSION_ITEM extends GroundControl.MISSION_ITEM {
		public void seq_SET(char src) //Sequence
		{ set_bytes((char) (src) & -1L, 2, data, 0); }

		public void target_system_SET(char src) //System ID
		{ set_bytes((char) (src) & -1L, 1, data, 2); }

		public void target_component_SET(char src) //Component ID
		{ set_bytes((char) (src) & -1L, 1, data, 3); }

		public void current_SET(char src) //false:0, true:1
		{ set_bytes((char) (src) & -1L, 1, data, 4); }

		public void autocontinue_SET(char src) //autocontinue to next wp
		{ set_bytes((char) (src) & -1L, 1, data, 5); }

		public void param1_SET(float src) //PARAM1, see MAV_CMD enum
		{ set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 6); }

		public void param2_SET(float src) //PARAM2, see MAV_CMD enum
		{ set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 10); }

		public void param3_SET(float src) //PARAM3, see MAV_CMD enum
		{ set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 14); }

		public void param4_SET(float src) //PARAM4, see MAV_CMD enum
		{ set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 18); }

		public void x_SET(float src) //PARAM5 / local: x position, global: latitude
		{ set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 22); }

		public void y_SET(float src) //PARAM6 / y position: global: longitude
		{ set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 26); }

		public void z_SET(float src) //PARAM7 / z position: global: altitude (relative or absolute, depending on frame.
		{ set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 30); }

		public void frame_SET(@MAV_FRAME int src) //The coordinate system of the waypoint. see MAV_FRAME in mavlink_types.h
		{ set_bits(-0 + src, 4, data, 272); }

		public void command_SET(@MAV_CMD int src) //The scheduled action for the waypoint. see MAV_CMD in common.xml MAVLink specs
		{
			long id = 0;
			switch (src)
			{
				case MAV_CMD.MAV_CMD_NAV_WAYPOINT:
					id = 0;
					break;
				case MAV_CMD.MAV_CMD_NAV_LOITER_UNLIM:
					id = 1;
					break;
				case MAV_CMD.MAV_CMD_NAV_LOITER_TURNS:
					id = 2;
					break;
				case MAV_CMD.MAV_CMD_NAV_LOITER_TIME:
					id = 3;
					break;
				case MAV_CMD.MAV_CMD_NAV_RETURN_TO_LAUNCH:
					id = 4;
					break;
				case MAV_CMD.MAV_CMD_NAV_LAND:
					id = 5;
					break;
				case MAV_CMD.MAV_CMD_NAV_TAKEOFF:
					id = 6;
					break;
				case MAV_CMD.MAV_CMD_NAV_LAND_LOCAL:
					id = 7;
					break;
				case MAV_CMD.MAV_CMD_NAV_TAKEOFF_LOCAL:
					id = 8;
					break;
				case MAV_CMD.MAV_CMD_NAV_FOLLOW:
					id = 9;
					break;
				case MAV_CMD.MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT:
					id = 10;
					break;
				case MAV_CMD.MAV_CMD_NAV_LOITER_TO_ALT:
					id = 11;
					break;
				case MAV_CMD.MAV_CMD_DO_FOLLOW:
					id = 12;
					break;
				case MAV_CMD.MAV_CMD_DO_FOLLOW_REPOSITION:
					id = 13;
					break;
				case MAV_CMD.MAV_CMD_NAV_ROI:
					id = 14;
					break;
				case MAV_CMD.MAV_CMD_NAV_PATHPLANNING:
					id = 15;
					break;
				case MAV_CMD.MAV_CMD_NAV_SPLINE_WAYPOINT:
					id = 16;
					break;
				case MAV_CMD.MAV_CMD_NAV_VTOL_TAKEOFF:
					id = 17;
					break;
				case MAV_CMD.MAV_CMD_NAV_VTOL_LAND:
					id = 18;
					break;
				case MAV_CMD.MAV_CMD_NAV_GUIDED_ENABLE:
					id = 19;
					break;
				case MAV_CMD.MAV_CMD_NAV_DELAY:
					id = 20;
					break;
				case MAV_CMD.MAV_CMD_NAV_PAYLOAD_PLACE:
					id = 21;
					break;
				case MAV_CMD.MAV_CMD_NAV_LAST:
					id = 22;
					break;
				case MAV_CMD.MAV_CMD_CONDITION_DELAY:
					id = 23;
					break;
				case MAV_CMD.MAV_CMD_CONDITION_CHANGE_ALT:
					id = 24;
					break;
				case MAV_CMD.MAV_CMD_CONDITION_DISTANCE:
					id = 25;
					break;
				case MAV_CMD.MAV_CMD_CONDITION_YAW:
					id = 26;
					break;
				case MAV_CMD.MAV_CMD_CONDITION_LAST:
					id = 27;
					break;
				case MAV_CMD.MAV_CMD_DO_SET_MODE:
					id = 28;
					break;
				case MAV_CMD.MAV_CMD_DO_JUMP:
					id = 29;
					break;
				case MAV_CMD.MAV_CMD_DO_CHANGE_SPEED:
					id = 30;
					break;
				case MAV_CMD.MAV_CMD_DO_SET_HOME:
					id = 31;
					break;
				case MAV_CMD.MAV_CMD_DO_SET_PARAMETER:
					id = 32;
					break;
				case MAV_CMD.MAV_CMD_DO_SET_RELAY:
					id = 33;
					break;
				case MAV_CMD.MAV_CMD_DO_REPEAT_RELAY:
					id = 34;
					break;
				case MAV_CMD.MAV_CMD_DO_SET_SERVO:
					id = 35;
					break;
				case MAV_CMD.MAV_CMD_DO_REPEAT_SERVO:
					id = 36;
					break;
				case MAV_CMD.MAV_CMD_DO_FLIGHTTERMINATION:
					id = 37;
					break;
				case MAV_CMD.MAV_CMD_DO_CHANGE_ALTITUDE:
					id = 38;
					break;
				case MAV_CMD.MAV_CMD_DO_LAND_START:
					id = 39;
					break;
				case MAV_CMD.MAV_CMD_DO_RALLY_LAND:
					id = 40;
					break;
				case MAV_CMD.MAV_CMD_DO_GO_AROUND:
					id = 41;
					break;
				case MAV_CMD.MAV_CMD_DO_REPOSITION:
					id = 42;
					break;
				case MAV_CMD.MAV_CMD_DO_PAUSE_CONTINUE:
					id = 43;
					break;
				case MAV_CMD.MAV_CMD_DO_SET_REVERSE:
					id = 44;
					break;
				case MAV_CMD.MAV_CMD_DO_CONTROL_VIDEO:
					id = 45;
					break;
				case MAV_CMD.MAV_CMD_DO_SET_ROI:
					id = 46;
					break;
				case MAV_CMD.MAV_CMD_DO_DIGICAM_CONFIGURE:
					id = 47;
					break;
				case MAV_CMD.MAV_CMD_DO_DIGICAM_CONTROL:
					id = 48;
					break;
				case MAV_CMD.MAV_CMD_DO_MOUNT_CONFIGURE:
					id = 49;
					break;
				case MAV_CMD.MAV_CMD_DO_MOUNT_CONTROL:
					id = 50;
					break;
				case MAV_CMD.MAV_CMD_DO_SET_CAM_TRIGG_DIST:
					id = 51;
					break;
				case MAV_CMD.MAV_CMD_DO_FENCE_ENABLE:
					id = 52;
					break;
				case MAV_CMD.MAV_CMD_DO_PARACHUTE:
					id = 53;
					break;
				case MAV_CMD.MAV_CMD_DO_MOTOR_TEST:
					id = 54;
					break;
				case MAV_CMD.MAV_CMD_DO_INVERTED_FLIGHT:
					id = 55;
					break;
				case MAV_CMD.MAV_CMD_NAV_SET_YAW_SPEED:
					id = 56;
					break;
				case MAV_CMD.MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL:
					id = 57;
					break;
				case MAV_CMD.MAV_CMD_DO_MOUNT_CONTROL_QUAT:
					id = 58;
					break;
				case MAV_CMD.MAV_CMD_DO_GUIDED_MASTER:
					id = 59;
					break;
				case MAV_CMD.MAV_CMD_DO_GUIDED_LIMITS:
					id = 60;
					break;
				case MAV_CMD.MAV_CMD_DO_ENGINE_CONTROL:
					id = 61;
					break;
				case MAV_CMD.MAV_CMD_DO_LAST:
					id = 62;
					break;
				case MAV_CMD.MAV_CMD_PREFLIGHT_CALIBRATION:
					id = 63;
					break;
				case MAV_CMD.MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS:
					id = 64;
					break;
				case MAV_CMD.MAV_CMD_PREFLIGHT_UAVCAN:
					id = 65;
					break;
				case MAV_CMD.MAV_CMD_PREFLIGHT_STORAGE:
					id = 66;
					break;
				case MAV_CMD.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
					id = 67;
					break;
				case MAV_CMD.MAV_CMD_OVERRIDE_GOTO:
					id = 68;
					break;
				case MAV_CMD.MAV_CMD_MISSION_START:
					id = 69;
					break;
				case MAV_CMD.MAV_CMD_COMPONENT_ARM_DISARM:
					id = 70;
					break;
				case MAV_CMD.MAV_CMD_GET_HOME_POSITION:
					id = 71;
					break;
				case MAV_CMD.MAV_CMD_START_RX_PAIR:
					id = 72;
					break;
				case MAV_CMD.MAV_CMD_GET_MESSAGE_INTERVAL:
					id = 73;
					break;
				case MAV_CMD.MAV_CMD_SET_MESSAGE_INTERVAL:
					id = 74;
					break;
				case MAV_CMD.MAV_CMD_REQUEST_PROTOCOL_VERSION:
					id = 75;
					break;
				case MAV_CMD.MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES:
					id = 76;
					break;
				case MAV_CMD.MAV_CMD_REQUEST_CAMERA_INFORMATION:
					id = 77;
					break;
				case MAV_CMD.MAV_CMD_REQUEST_CAMERA_SETTINGS:
					id = 78;
					break;
				case MAV_CMD.MAV_CMD_REQUEST_STORAGE_INFORMATION:
					id = 79;
					break;
				case MAV_CMD.MAV_CMD_STORAGE_FORMAT:
					id = 80;
					break;
				case MAV_CMD.MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS:
					id = 81;
					break;
				case MAV_CMD.MAV_CMD_REQUEST_FLIGHT_INFORMATION:
					id = 82;
					break;
				case MAV_CMD.MAV_CMD_RESET_CAMERA_SETTINGS:
					id = 83;
					break;
				case MAV_CMD.MAV_CMD_SET_CAMERA_MODE:
					id = 84;
					break;
				case MAV_CMD.MAV_CMD_IMAGE_START_CAPTURE:
					id = 85;
					break;
				case MAV_CMD.MAV_CMD_IMAGE_STOP_CAPTURE:
					id = 86;
					break;
				case MAV_CMD.MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE:
					id = 87;
					break;
				case MAV_CMD.MAV_CMD_DO_TRIGGER_CONTROL:
					id = 88;
					break;
				case MAV_CMD.MAV_CMD_VIDEO_START_CAPTURE:
					id = 89;
					break;
				case MAV_CMD.MAV_CMD_VIDEO_STOP_CAPTURE:
					id = 90;
					break;
				case MAV_CMD.MAV_CMD_VIDEO_START_STREAMING:
					id = 91;
					break;
				case MAV_CMD.MAV_CMD_VIDEO_STOP_STREAMING:
					id = 92;
					break;
				case MAV_CMD.MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION:
					id = 93;
					break;
				case MAV_CMD.MAV_CMD_LOGGING_START:
					id = 94;
					break;
				case MAV_CMD.MAV_CMD_LOGGING_STOP:
					id = 95;
					break;
				case MAV_CMD.MAV_CMD_AIRFRAME_CONFIGURATION:
					id = 96;
					break;
				case MAV_CMD.MAV_CMD_PANORAMA_CREATE:
					id = 97;
					break;
				case MAV_CMD.MAV_CMD_DO_VTOL_TRANSITION:
					id = 98;
					break;
				case MAV_CMD.MAV_CMD_ARM_AUTHORIZATION_REQUEST:
					id = 99;
					break;
				case MAV_CMD.MAV_CMD_SET_GUIDED_SUBMODE_STANDARD:
					id = 100;
					break;
				case MAV_CMD.MAV_CMD_SET_GUIDED_SUBMODE_CIRCLE:
					id = 101;
					break;
				case MAV_CMD.MAV_CMD_CONDITION_GATE:
					id = 102;
					break;
				case MAV_CMD.MAV_CMD_NAV_FENCE_RETURN_POINT:
					id = 103;
					break;
				case MAV_CMD.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION:
					id = 104;
					break;
				case MAV_CMD.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION:
					id = 105;
					break;
				case MAV_CMD.MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION:
					id = 106;
					break;
				case MAV_CMD.MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION:
					id = 107;
					break;
				case MAV_CMD.MAV_CMD_NAV_RALLY_POINT:
					id = 108;
					break;
				case MAV_CMD.MAV_CMD_UAVCAN_GET_NODE_INFO:
					id = 109;
					break;
				case MAV_CMD.MAV_CMD_PAYLOAD_PREPARE_DEPLOY:
					id = 110;
					break;
				case MAV_CMD.MAV_CMD_PAYLOAD_CONTROL_DEPLOY:
					id = 111;
					break;
				case MAV_CMD.MAV_CMD_WAYPOINT_USER_1:
					id = 112;
					break;
				case MAV_CMD.MAV_CMD_WAYPOINT_USER_2:
					id = 113;
					break;
				case MAV_CMD.MAV_CMD_WAYPOINT_USER_3:
					id = 114;
					break;
				case MAV_CMD.MAV_CMD_WAYPOINT_USER_4:
					id = 115;
					break;
				case MAV_CMD.MAV_CMD_WAYPOINT_USER_5:
					id = 116;
					break;
				case MAV_CMD.MAV_CMD_SPATIAL_USER_1:
					id = 117;
					break;
				case MAV_CMD.MAV_CMD_SPATIAL_USER_2:
					id = 118;
					break;
				case MAV_CMD.MAV_CMD_SPATIAL_USER_3:
					id = 119;
					break;
				case MAV_CMD.MAV_CMD_SPATIAL_USER_4:
					id = 120;
					break;
				case MAV_CMD.MAV_CMD_SPATIAL_USER_5:
					id = 121;
					break;
				case MAV_CMD.MAV_CMD_USER_1:
					id = 122;
					break;
				case MAV_CMD.MAV_CMD_USER_2:
					id = 123;
					break;
				case MAV_CMD.MAV_CMD_USER_3:
					id = 124;
					break;
				case MAV_CMD.MAV_CMD_USER_4:
					id = 125;
					break;
				case MAV_CMD.MAV_CMD_USER_5:
					id = 126;
					break;
				case MAV_CMD.MAV_CMD_RESET_MPPT:
					id = 127;
					break;
				case MAV_CMD.MAV_CMD_PAYLOAD_CONTROL:
					id = 128;
					break;
				default:
					assert (false);//("Unknown enum" + id);
			}
			set_bits(id, 8, data, 276);
		}

		public void mission_type_SET(@MAV_MISSION_TYPE int src) //Mission type, see MAV_MISSION_TYPE
		{
			long id = 0;
			switch (src)
			{
				case MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION:
					id = 0;
					break;
				case MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE:
					id = 1;
					break;
				case MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY:
					id = 2;
					break;
				case MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL:
					id = 3;
					break;
				default:
					assert (false);//("Unknown enum" + id);
			}
			set_bits(id, 3, data, 284);
		}
	}

	public static class MISSION_REQUEST extends GroundControl.MISSION_REQUEST {
		public void seq_SET(char src) //Sequence
		{ set_bytes((char) (src) & -1L, 2, data, 0); }

		public void target_system_SET(char src) //System ID
		{ set_bytes((char) (src) & -1L, 1, data, 2); }

		public void target_component_SET(char src) //Component ID
		{ set_bytes((char) (src) & -1L, 1, data, 3); }

		public void mission_type_SET(@MAV_MISSION_TYPE int src) //Mission type, see MAV_MISSION_TYPE
		{
			long id = 0;
			switch (src)
			{
				case MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION:
					id = 0;
					break;
				case MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE:
					id = 1;
					break;
				case MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY:
					id = 2;
					break;
				case MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL:
					id = 3;
					break;
				default:
					assert (false);//("Unknown enum" + id);
			}
			set_bits(id, 3, data, 32);
		}
	}

	public static class MISSION_SET_CURRENT extends GroundControl.MISSION_SET_CURRENT {
		public void seq_SET(char src) //Sequence
		{ set_bytes((char) (src) & -1L, 2, data, 0); }

		public void target_system_SET(char src) //System ID
		{ set_bytes((char) (src) & -1L, 1, data, 2); }

		public void target_component_SET(char src) //Component ID
		{ set_bytes((char) (src) & -1L, 1, data, 3); }
	}

	public static class MISSION_CURRENT extends GroundControl.MISSION_CURRENT {
		public void seq_SET(char src) //Sequence
		{ set_bytes((char) (src) & -1L, 2, data, 0); }
	}

	public static class MISSION_REQUEST_LIST extends GroundControl.MISSION_REQUEST_LIST {
		public void target_system_SET(char src) //System ID
		{ set_bytes((char) (src) & -1L, 1, data, 0); }

		public void target_component_SET(char src) //Component ID
		{ set_bytes((char) (src) & -1L, 1, data, 1); }

		public void mission_type_SET(@MAV_MISSION_TYPE int src) //Mission type, see MAV_MISSION_TYPE
		{
			long id = 0;
			switch (src)
			{
				case MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION:
					id = 0;
					break;
				case MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE:
					id = 1;
					break;
				case MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY:
					id = 2;
					break;
				case MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL:
					id = 3;
					break;
				default:
					assert (false);//("Unknown enum" + id);
			}
			set_bits(id, 3, data, 16);
		}
	}

	public static class MISSION_COUNT extends GroundControl.MISSION_COUNT {
		public void count_SET(char src) //Number of mission items in the sequence
		{ set_bytes((char) (src) & -1L, 2, data, 0); }

		public void target_system_SET(char src) //System ID
		{ set_bytes((char) (src) & -1L, 1, data, 2); }

		public void target_component_SET(char src) //Component ID
		{ set_bytes((char) (src) & -1L, 1, data, 3); }

		public void mission_type_SET(@MAV_MISSION_TYPE int src) //Mission type, see MAV_MISSION_TYPE
		{
			long id = 0;
			switch (src)
			{
				case MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION:
					id = 0;
					break;
				case MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE:
					id = 1;
					break;
				case MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY:
					id = 2;
					break;
				case MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL:
					id = 3;
					break;
				default:
					assert (false);//("Unknown enum" + id);
			}
			set_bits(id, 3, data, 32);
		}
	}

	public static class MISSION_CLEAR_ALL extends GroundControl.MISSION_CLEAR_ALL {
		public void target_system_SET(char src) //System ID
		{ set_bytes((char) (src) & -1L, 1, data, 0); }

		public void target_component_SET(char src) //Component ID
		{ set_bytes((char) (src) & -1L, 1, data, 1); }

		public void mission_type_SET(@MAV_MISSION_TYPE int src) //Mission type, see MAV_MISSION_TYPE
		{
			long id = 0;
			switch (src)
			{
				case MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION:
					id = 0;
					break;
				case MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE:
					id = 1;
					break;
				case MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY:
					id = 2;
					break;
				case MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL:
					id = 3;
					break;
				default:
					assert (false);//("Unknown enum" + id);
			}
			set_bits(id, 3, data, 16);
		}
	}

	public static class MISSION_ITEM_REACHED extends GroundControl.MISSION_ITEM_REACHED {
		public void seq_SET(char src) //Sequence
		{ set_bytes((char) (src) & -1L, 2, data, 0); }
	}

	public static class MISSION_ACK extends GroundControl.MISSION_ACK {
		public void target_system_SET(char src) //System ID
		{ set_bytes((char) (src) & -1L, 1, data, 0); }

		public void target_component_SET(char src) //Component ID
		{ set_bytes((char) (src) & -1L, 1, data, 1); }

		public void type_SET(@MAV_MISSION_RESULT int src) //See MAV_MISSION_RESULT enum
		{ set_bits(-0 + src, 4, data, 16); }

		public void mission_type_SET(@MAV_MISSION_TYPE int src) //Mission type, see MAV_MISSION_TYPE
		{
			long id = 0;
			switch (src)
			{
				case MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION:
					id = 0;
					break;
				case MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE:
					id = 1;
					break;
				case MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY:
					id = 2;
					break;
				case MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL:
					id = 3;
					break;
				default:
					assert (false);//("Unknown enum" + id);
			}
			set_bits(id, 3, data, 20);
		}
	}

	public static class SET_GPS_GLOBAL_ORIGIN extends GroundControl.SET_GPS_GLOBAL_ORIGIN {
		public void target_system_SET(char src) //System ID
		{ set_bytes((char) (src) & -1L, 1, data, 0); }

		public void latitude_SET(int src) //Latitude (WGS84), in degrees * 1E7
		{ set_bytes((int) (src) & -1L, 4, data, 1); }

		public void longitude_SET(int src) //Longitude (WGS84, in degrees * 1E7
		{ set_bytes((int) (src) & -1L, 4, data, 5); }

		public void altitude_SET(int src) //Altitude (AMSL), in meters * 1000 (positive for up)
		{ set_bytes((int) (src) & -1L, 4, data, 9); }

		public void time_usec_SET(long src, Bounds.Inside ph)//Timestamp (microseconds since UNIX epoch or microseconds since system boot)
		{
			if (ph.field_bit != 104) insert_field(ph, 104, 0);
			set_bytes((src) & -1L, 8, data, ph.BYTE);
		}
	}

	public static class GPS_GLOBAL_ORIGIN extends GroundControl.GPS_GLOBAL_ORIGIN {
		public void latitude_SET(int src) //Latitude (WGS84), in degrees * 1E7
		{ set_bytes((int) (src) & -1L, 4, data, 0); }

		public void longitude_SET(int src) //Longitude (WGS84), in degrees * 1E7
		{ set_bytes((int) (src) & -1L, 4, data, 4); }

		public void altitude_SET(int src) //Altitude (AMSL), in meters * 1000 (positive for up)
		{ set_bytes((int) (src) & -1L, 4, data, 8); }

		public void time_usec_SET(long src, Bounds.Inside ph)//Timestamp (microseconds since UNIX epoch or microseconds since system boot)
		{
			if (ph.field_bit != 96) insert_field(ph, 96, 0);
			set_bytes((src) & -1L, 8, data, ph.BYTE);
		}
	}

	public static class PARAM_MAP_RC extends GroundControl.PARAM_MAP_RC {
		public void target_system_SET(char src) //System ID
		{ set_bytes((char) (src) & -1L, 1, data, 0); }

		public void target_component_SET(char src) //Component ID
		{ set_bytes((char) (src) & -1L, 1, data, 1); }

		/**
		 * Parameter index. Send -1 to use the param ID field as identifier (else the param id will be ignored),
		 * send -2 to disable any existing map for this rc_channel_index
		 */
		public void param_index_SET(short src) { set_bytes((short) (src) & -1L, 2, data, 2); }

		/**
		 * Index of parameter RC channel. Not equal to the RC channel id. Typically correpsonds to a potentiometer-knob
		 * on the RC
		 */
		public void parameter_rc_channel_index_SET(char src) { set_bytes((char) (src) & -1L, 1, data, 4); }

		public void param_value0_SET(float src) //Initial parameter value
		{ set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 5); }

		public void scale_SET(float src) //Scale, maps the RC range [-1, 1] to a parameter value
		{ set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 9); }

		/**
		 * Minimum param value. The protocol does not define if this overwrites an onboard minimum value. (Depends
		 * on implementation
		 */
		public void param_value_min_SET(float src) { set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 13); }

		/**
		 * Maximum param value. The protocol does not define if this overwrites an onboard maximum value. (Depends
		 * on implementation
		 */
		public void param_value_max_SET(float src) { set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 17); }

		/**
		 * Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT
		 * null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes
		 * storage if the ID is stored as strin
		 */
		public void param_id_SET(String src, Bounds.Inside ph) {param_id_SET(src.toCharArray(), 0, src.length(), ph);}

		/**
		 * Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT
		 * null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes
		 * storage if the ID is stored as strin
		 */
		public void param_id_SET(char[] src, int pos, int items, Bounds.Inside ph) {
			if (ph.field_bit != 168 && insert_field(ph, 168, items) || !try_visit_item(ph, 0))
				insert_item(ph, 0, items);
			for (int BYTE = ph.BYTE, src_max = pos + ph.items; pos < src_max; pos++, BYTE += 2)
				set_bytes((short) (src[pos]) & -1L, 2, data, BYTE);
		}
	}

	public static class MISSION_REQUEST_INT extends GroundControl.MISSION_REQUEST_INT {
		public void seq_SET(char src) //Sequence
		{ set_bytes((char) (src) & -1L, 2, data, 0); }

		public void target_system_SET(char src) //System ID
		{ set_bytes((char) (src) & -1L, 1, data, 2); }

		public void target_component_SET(char src) //Component ID
		{ set_bytes((char) (src) & -1L, 1, data, 3); }

		public void mission_type_SET(@MAV_MISSION_TYPE int src) //Mission type, see MAV_MISSION_TYPE
		{
			long id = 0;
			switch (src)
			{
				case MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION:
					id = 0;
					break;
				case MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE:
					id = 1;
					break;
				case MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY:
					id = 2;
					break;
				case MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL:
					id = 3;
					break;
				default:
					assert (false);//("Unknown enum" + id);
			}
			set_bits(id, 3, data, 32);
		}
	}

	public static class SAFETY_SET_ALLOWED_AREA extends GroundControl.SAFETY_SET_ALLOWED_AREA {
		public void target_system_SET(char src) //System ID
		{ set_bytes((char) (src) & -1L, 1, data, 0); }

		public void target_component_SET(char src) //Component ID
		{ set_bytes((char) (src) & -1L, 1, data, 1); }

		public void p1x_SET(float src) //x position 1 / Latitude 1
		{ set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 2); }

		public void p1y_SET(float src) //y position 1 / Longitude 1
		{ set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 6); }

		public void p1z_SET(float src) //z position 1 / Altitude 1
		{ set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 10); }

		public void p2x_SET(float src) //x position 2 / Latitude 2
		{ set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 14); }

		public void p2y_SET(float src) //y position 2 / Longitude 2
		{ set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 18); }

		public void p2z_SET(float src) //z position 2 / Altitude 2
		{ set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 22); }

		/**
		 * Coordinate frame, as defined by MAV_FRAME enum in mavlink_types.h. Can be either global, GPS, right-handed
		 * with Z axis up or local, right handed, Z axis down
		 */
		public void frame_SET(@MAV_FRAME int src) { set_bits(-0 + src, 4, data, 208); }
	}

	public static class SAFETY_ALLOWED_AREA extends GroundControl.SAFETY_ALLOWED_AREA {
		public void p1x_SET(float src) //x position 1 / Latitude 1
		{ set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 0); }

		public void p1y_SET(float src) //y position 1 / Longitude 1
		{ set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 4); }

		public void p1z_SET(float src) //z position 1 / Altitude 1
		{ set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 8); }

		public void p2x_SET(float src) //x position 2 / Latitude 2
		{ set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 12); }

		public void p2y_SET(float src) //y position 2 / Longitude 2
		{ set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 16); }

		public void p2z_SET(float src) //z position 2 / Altitude 2
		{ set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 20); }

		/**
		 * Coordinate frame, as defined by MAV_FRAME enum in mavlink_types.h. Can be either global, GPS, right-handed
		 * with Z axis up or local, right handed, Z axis down
		 */
		public void frame_SET(@MAV_FRAME int src) { set_bits(-0 + src, 4, data, 192); }
	}

	public static class ATTITUDE_QUATERNION_COV extends GroundControl.ATTITUDE_QUATERNION_COV {
		public void time_usec_SET(long src) //Timestamp (microseconds since system boot or since UNIX epoch)
		{ set_bytes((src) & -1L, 8, data, 0); }

		public void q_SET(float[] src, int pos)  //Quaternion components, w, x, y, z (1 0 0 0 is the null-rotation)
		{
			for (int BYTE = 8, src_max = pos + 4; pos < src_max; pos++, BYTE += 4)
				set_bytes(Float.floatToIntBits(src[pos]) & -1L, 4, data, BYTE);
		}

		public void rollspeed_SET(float src) //Roll angular speed (rad/s)
		{ set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 24); }

		public void pitchspeed_SET(float src) //Pitch angular speed (rad/s)
		{ set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 28); }

		public void yawspeed_SET(float src) //Yaw angular speed (rad/s)
		{ set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 32); }

		public void covariance_SET(float[] src, int pos)  //Attitude covariance
		{
			for (int BYTE = 36, src_max = pos + 9; pos < src_max; pos++, BYTE += 4)
				set_bytes(Float.floatToIntBits(src[pos]) & -1L, 4, data, BYTE);
		}
	}

	public static class NAV_CONTROLLER_OUTPUT extends GroundControl.NAV_CONTROLLER_OUTPUT {
		public void wp_dist_SET(char src) //Distance to active waypoint in meters
		{ set_bytes((char) (src) & -1L, 2, data, 0); }

		public void nav_roll_SET(float src) //Current desired roll in degrees
		{ set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 2); }

		public void nav_pitch_SET(float src) //Current desired pitch in degrees
		{ set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 6); }

		public void nav_bearing_SET(short src) //Current desired heading in degrees
		{ set_bytes((short) (src) & -1L, 2, data, 10); }

		public void target_bearing_SET(short src) //Bearing to current waypoint/target in degrees
		{ set_bytes((short) (src) & -1L, 2, data, 12); }

		public void alt_error_SET(float src) //Current altitude error in meters
		{ set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 14); }

		public void aspd_error_SET(float src) //Current airspeed error in meters/second
		{ set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 18); }

		public void xtrack_error_SET(float src) //Current crosstrack error on x-y plane in meters
		{ set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 22); }
	}

	public static class GLOBAL_POSITION_INT_COV extends GroundControl.GLOBAL_POSITION_INT_COV {
		public void time_usec_SET(long src) //Timestamp (microseconds since system boot or since UNIX epoch)
		{ set_bytes((src) & -1L, 8, data, 0); }

		public void lat_SET(int src) //Latitude, expressed as degrees * 1E7
		{ set_bytes((int) (src) & -1L, 4, data, 8); }

		public void lon_SET(int src) //Longitude, expressed as degrees * 1E7
		{ set_bytes((int) (src) & -1L, 4, data, 12); }

		public void alt_SET(int src) //Altitude in meters, expressed as * 1000 (millimeters), above MSL
		{ set_bytes((int) (src) & -1L, 4, data, 16); }

		public void relative_alt_SET(int src) //Altitude above ground in meters, expressed as * 1000 (millimeters)
		{ set_bytes((int) (src) & -1L, 4, data, 20); }

		public void vx_SET(float src) //Ground X Speed (Latitude), expressed as m/s
		{ set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 24); }

		public void vy_SET(float src) //Ground Y Speed (Longitude), expressed as m/s
		{ set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 28); }

		public void vz_SET(float src) //Ground Z Speed (Altitude), expressed as m/s
		{ set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 32); }

		public void covariance_SET(float[] src, int pos)  //Covariance matrix (first six entries are the first ROW, next six entries are the second row, etc.)
		{
			for (int BYTE = 36, src_max = pos + 36; pos < src_max; pos++, BYTE += 4)
				set_bytes(Float.floatToIntBits(src[pos]) & -1L, 4, data, BYTE);
		}

		public void estimator_type_SET(@MAV_ESTIMATOR_TYPE int src) //Class id of the estimator this estimate originated from.
		{ set_bits(-1 + src, 3, data, 1440); }
	}

	public static class LOCAL_POSITION_NED_COV extends GroundControl.LOCAL_POSITION_NED_COV {
		public void time_usec_SET(long src) //Timestamp (microseconds since system boot or since UNIX epoch)
		{ set_bytes((src) & -1L, 8, data, 0); }

		public void x_SET(float src) //X Position
		{ set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 8); }

		public void y_SET(float src) //Y Position
		{ set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 12); }

		public void z_SET(float src) //Z Position
		{ set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 16); }

		public void vx_SET(float src) //X Speed (m/s)
		{ set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 20); }

		public void vy_SET(float src) //Y Speed (m/s)
		{ set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 24); }

		public void vz_SET(float src) //Z Speed (m/s)
		{ set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 28); }

		public void ax_SET(float src) //X Acceleration (m/s^2)
		{ set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 32); }

		public void ay_SET(float src) //Y Acceleration (m/s^2)
		{ set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 36); }

		public void az_SET(float src) //Z Acceleration (m/s^2)
		{ set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 40); }

		/**
		 * Covariance matrix upper right triangular (first nine entries are the first ROW, next eight entries are
		 * the second row, etc.
		 */
		public void covariance_SET(float[] src, int pos) {
			for (int BYTE = 44, src_max = pos + 45; pos < src_max; pos++, BYTE += 4)
				set_bytes(Float.floatToIntBits(src[pos]) & -1L, 4, data, BYTE);
		}

		public void estimator_type_SET(@MAV_ESTIMATOR_TYPE int src) //Class id of the estimator this estimate originated from.
		{ set_bits(-1 + src, 3, data, 1792); }
	}

	public static class RC_CHANNELS extends GroundControl.RC_CHANNELS {
		public void chan1_raw_SET(char src) //RC channel 1 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
		{ set_bytes((char) (src) & -1L, 2, data, 0); }

		public void chan2_raw_SET(char src) //RC channel 2 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
		{ set_bytes((char) (src) & -1L, 2, data, 2); }

		public void chan3_raw_SET(char src) //RC channel 3 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
		{ set_bytes((char) (src) & -1L, 2, data, 4); }

		public void chan4_raw_SET(char src) //RC channel 4 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
		{ set_bytes((char) (src) & -1L, 2, data, 6); }

		public void chan5_raw_SET(char src) //RC channel 5 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
		{ set_bytes((char) (src) & -1L, 2, data, 8); }

		public void chan6_raw_SET(char src) //RC channel 6 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
		{ set_bytes((char) (src) & -1L, 2, data, 10); }

		public void chan7_raw_SET(char src) //RC channel 7 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
		{ set_bytes((char) (src) & -1L, 2, data, 12); }

		public void chan8_raw_SET(char src) //RC channel 8 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
		{ set_bytes((char) (src) & -1L, 2, data, 14); }

		public void chan9_raw_SET(char src) //RC channel 9 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
		{ set_bytes((char) (src) & -1L, 2, data, 16); }

		public void chan10_raw_SET(char src) //RC channel 10 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
		{ set_bytes((char) (src) & -1L, 2, data, 18); }

		public void chan11_raw_SET(char src) //RC channel 11 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
		{ set_bytes((char) (src) & -1L, 2, data, 20); }

		public void chan12_raw_SET(char src) //RC channel 12 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
		{ set_bytes((char) (src) & -1L, 2, data, 22); }

		public void chan13_raw_SET(char src) //RC channel 13 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
		{ set_bytes((char) (src) & -1L, 2, data, 24); }

		public void chan14_raw_SET(char src) //RC channel 14 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
		{ set_bytes((char) (src) & -1L, 2, data, 26); }

		public void chan15_raw_SET(char src) //RC channel 15 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
		{ set_bytes((char) (src) & -1L, 2, data, 28); }

		public void chan16_raw_SET(char src) //RC channel 16 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
		{ set_bytes((char) (src) & -1L, 2, data, 30); }

		public void chan17_raw_SET(char src) //RC channel 17 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
		{ set_bytes((char) (src) & -1L, 2, data, 32); }

		public void chan18_raw_SET(char src) //RC channel 18 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
		{ set_bytes((char) (src) & -1L, 2, data, 34); }

		public void time_boot_ms_SET(long src) //Timestamp (milliseconds since system boot)
		{ set_bytes((src) & -1L, 4, data, 36); }

		/**
		 * Total number of RC channels being received. This can be larger than 18, indicating that more channels
		 * are available but not given in this message. This value should be 0 when no RC channels are available
		 */
		public void chancount_SET(char src) { set_bytes((char) (src) & -1L, 1, data, 40); }

		public void rssi_SET(char src) //Receive signal strength indicator, 0: 0%, 100: 100%, 255: invalid/unknown.
		{ set_bytes((char) (src) & -1L, 1, data, 41); }
	}

	public static class REQUEST_DATA_STREAM extends GroundControl.REQUEST_DATA_STREAM {
		public void req_message_rate_SET(char src) //The requested message rate
		{ set_bytes((char) (src) & -1L, 2, data, 0); }

		public void target_system_SET(char src) //The target requested to send the message stream.
		{ set_bytes((char) (src) & -1L, 1, data, 2); }

		public void target_component_SET(char src) //The target requested to send the message stream.
		{ set_bytes((char) (src) & -1L, 1, data, 3); }

		public void req_stream_id_SET(char src) //The ID of the requested data stream
		{ set_bytes((char) (src) & -1L, 1, data, 4); }

		public void start_stop_SET(char src) //1 to start sending, 0 to stop sending.
		{ set_bytes((char) (src) & -1L, 1, data, 5); }
	}

	public static class DATA_STREAM extends GroundControl.DATA_STREAM {
		public void message_rate_SET(char src) //The message rate
		{ set_bytes((char) (src) & -1L, 2, data, 0); }

		public void stream_id_SET(char src) //The ID of the requested data stream
		{ set_bytes((char) (src) & -1L, 1, data, 2); }

		public void on_off_SET(char src) //1 stream is enabled, 0 stream is stopped.
		{ set_bytes((char) (src) & -1L, 1, data, 3); }
	}

	public static class MANUAL_CONTROL extends GroundControl.MANUAL_CONTROL {
		/**
		 * A bitfield corresponding to the joystick buttons' current state, 1 for pressed, 0 for released. The lowest
		 * bit corresponds to Button 1
		 */
		public void buttons_SET(char src) { set_bytes((char) (src) & -1L, 2, data, 0); }

		public void target_SET(char src) //The system to be controlled.
		{ set_bytes((char) (src) & -1L, 1, data, 2); }

		/**
		 * X-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid.
		 * Generally corresponds to forward(1000)-backward(-1000) movement on a joystick and the pitch of a vehicle
		 */
		public void x_SET(short src) { set_bytes((short) (src) & -1L, 2, data, 3); }

		/**
		 * Y-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid.
		 * Generally corresponds to left(-1000)-right(1000) movement on a joystick and the roll of a vehicle
		 */
		public void y_SET(short src) { set_bytes((short) (src) & -1L, 2, data, 5); }

		/**
		 * Z-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid.
		 * Generally corresponds to a separate slider movement with maximum being 1000 and minimum being -1000 on
		 * a joystick and the thrust of a vehicle. Positive values are positive thrust, negative values are negative
		 * thrust
		 */
		public void z_SET(short src) { set_bytes((short) (src) & -1L, 2, data, 7); }

		/**
		 * R-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid.
		 * Generally corresponds to a twisting of the joystick, with counter-clockwise being 1000 and clockwise
		 * being -1000, and the yaw of a vehicle
		 */
		public void r_SET(short src) { set_bytes((short) (src) & -1L, 2, data, 9); }
	}

	public static class RC_CHANNELS_OVERRIDE extends GroundControl.RC_CHANNELS_OVERRIDE {
		public void chan1_raw_SET(char src) //RC channel 1 value, in microseconds. A value of UINT16_MAX means to ignore this field.
		{ set_bytes((char) (src) & -1L, 2, data, 0); }

		public void chan2_raw_SET(char src) //RC channel 2 value, in microseconds. A value of UINT16_MAX means to ignore this field.
		{ set_bytes((char) (src) & -1L, 2, data, 2); }

		public void chan3_raw_SET(char src) //RC channel 3 value, in microseconds. A value of UINT16_MAX means to ignore this field.
		{ set_bytes((char) (src) & -1L, 2, data, 4); }

		public void chan4_raw_SET(char src) //RC channel 4 value, in microseconds. A value of UINT16_MAX means to ignore this field.
		{ set_bytes((char) (src) & -1L, 2, data, 6); }

		public void chan5_raw_SET(char src) //RC channel 5 value, in microseconds. A value of UINT16_MAX means to ignore this field.
		{ set_bytes((char) (src) & -1L, 2, data, 8); }

		public void chan6_raw_SET(char src) //RC channel 6 value, in microseconds. A value of UINT16_MAX means to ignore this field.
		{ set_bytes((char) (src) & -1L, 2, data, 10); }

		public void chan7_raw_SET(char src) //RC channel 7 value, in microseconds. A value of UINT16_MAX means to ignore this field.
		{ set_bytes((char) (src) & -1L, 2, data, 12); }

		public void chan8_raw_SET(char src) //RC channel 8 value, in microseconds. A value of UINT16_MAX means to ignore this field.
		{ set_bytes((char) (src) & -1L, 2, data, 14); }

		public void target_system_SET(char src) //System ID
		{ set_bytes((char) (src) & -1L, 1, data, 16); }

		public void target_component_SET(char src) //Component ID
		{ set_bytes((char) (src) & -1L, 1, data, 17); }
	}

	public static class MISSION_ITEM_INT extends GroundControl.MISSION_ITEM_INT {
		/**
		 * Waypoint ID (sequence number). Starts at zero. Increases monotonically for each waypoint, no gaps in the
		 * sequence (0,1,2,3,4)
		 */
		public void seq_SET(char src) { set_bytes((char) (src) & -1L, 2, data, 0); }

		public void target_system_SET(char src) //System ID
		{ set_bytes((char) (src) & -1L, 1, data, 2); }

		public void target_component_SET(char src) //Component ID
		{ set_bytes((char) (src) & -1L, 1, data, 3); }

		public void current_SET(char src) //false:0, true:1
		{ set_bytes((char) (src) & -1L, 1, data, 4); }

		public void autocontinue_SET(char src) //autocontinue to next wp
		{ set_bytes((char) (src) & -1L, 1, data, 5); }

		public void param1_SET(float src) //PARAM1, see MAV_CMD enum
		{ set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 6); }

		public void param2_SET(float src) //PARAM2, see MAV_CMD enum
		{ set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 10); }

		public void param3_SET(float src) //PARAM3, see MAV_CMD enum
		{ set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 14); }

		public void param4_SET(float src) //PARAM4, see MAV_CMD enum
		{ set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 18); }

		public void x_SET(int src) //PARAM5 / local: x position in meters * 1e4, global: latitude in degrees * 10^7
		{ set_bytes((int) (src) & -1L, 4, data, 22); }

		public void y_SET(int src) //PARAM6 / y position: local: x position in meters * 1e4, global: longitude in degrees *10^7
		{ set_bytes((int) (src) & -1L, 4, data, 26); }

		public void z_SET(float src) //PARAM7 / z position: global: altitude in meters (relative or absolute, depending on frame.
		{ set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 30); }

		public void frame_SET(@MAV_FRAME int src) //The coordinate system of the waypoint. see MAV_FRAME in mavlink_types.h
		{ set_bits(-0 + src, 4, data, 272); }

		public void command_SET(@MAV_CMD int src) //The scheduled action for the waypoint. see MAV_CMD in common.xml MAVLink specs
		{
			long id = 0;
			switch (src)
			{
				case MAV_CMD.MAV_CMD_NAV_WAYPOINT:
					id = 0;
					break;
				case MAV_CMD.MAV_CMD_NAV_LOITER_UNLIM:
					id = 1;
					break;
				case MAV_CMD.MAV_CMD_NAV_LOITER_TURNS:
					id = 2;
					break;
				case MAV_CMD.MAV_CMD_NAV_LOITER_TIME:
					id = 3;
					break;
				case MAV_CMD.MAV_CMD_NAV_RETURN_TO_LAUNCH:
					id = 4;
					break;
				case MAV_CMD.MAV_CMD_NAV_LAND:
					id = 5;
					break;
				case MAV_CMD.MAV_CMD_NAV_TAKEOFF:
					id = 6;
					break;
				case MAV_CMD.MAV_CMD_NAV_LAND_LOCAL:
					id = 7;
					break;
				case MAV_CMD.MAV_CMD_NAV_TAKEOFF_LOCAL:
					id = 8;
					break;
				case MAV_CMD.MAV_CMD_NAV_FOLLOW:
					id = 9;
					break;
				case MAV_CMD.MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT:
					id = 10;
					break;
				case MAV_CMD.MAV_CMD_NAV_LOITER_TO_ALT:
					id = 11;
					break;
				case MAV_CMD.MAV_CMD_DO_FOLLOW:
					id = 12;
					break;
				case MAV_CMD.MAV_CMD_DO_FOLLOW_REPOSITION:
					id = 13;
					break;
				case MAV_CMD.MAV_CMD_NAV_ROI:
					id = 14;
					break;
				case MAV_CMD.MAV_CMD_NAV_PATHPLANNING:
					id = 15;
					break;
				case MAV_CMD.MAV_CMD_NAV_SPLINE_WAYPOINT:
					id = 16;
					break;
				case MAV_CMD.MAV_CMD_NAV_VTOL_TAKEOFF:
					id = 17;
					break;
				case MAV_CMD.MAV_CMD_NAV_VTOL_LAND:
					id = 18;
					break;
				case MAV_CMD.MAV_CMD_NAV_GUIDED_ENABLE:
					id = 19;
					break;
				case MAV_CMD.MAV_CMD_NAV_DELAY:
					id = 20;
					break;
				case MAV_CMD.MAV_CMD_NAV_PAYLOAD_PLACE:
					id = 21;
					break;
				case MAV_CMD.MAV_CMD_NAV_LAST:
					id = 22;
					break;
				case MAV_CMD.MAV_CMD_CONDITION_DELAY:
					id = 23;
					break;
				case MAV_CMD.MAV_CMD_CONDITION_CHANGE_ALT:
					id = 24;
					break;
				case MAV_CMD.MAV_CMD_CONDITION_DISTANCE:
					id = 25;
					break;
				case MAV_CMD.MAV_CMD_CONDITION_YAW:
					id = 26;
					break;
				case MAV_CMD.MAV_CMD_CONDITION_LAST:
					id = 27;
					break;
				case MAV_CMD.MAV_CMD_DO_SET_MODE:
					id = 28;
					break;
				case MAV_CMD.MAV_CMD_DO_JUMP:
					id = 29;
					break;
				case MAV_CMD.MAV_CMD_DO_CHANGE_SPEED:
					id = 30;
					break;
				case MAV_CMD.MAV_CMD_DO_SET_HOME:
					id = 31;
					break;
				case MAV_CMD.MAV_CMD_DO_SET_PARAMETER:
					id = 32;
					break;
				case MAV_CMD.MAV_CMD_DO_SET_RELAY:
					id = 33;
					break;
				case MAV_CMD.MAV_CMD_DO_REPEAT_RELAY:
					id = 34;
					break;
				case MAV_CMD.MAV_CMD_DO_SET_SERVO:
					id = 35;
					break;
				case MAV_CMD.MAV_CMD_DO_REPEAT_SERVO:
					id = 36;
					break;
				case MAV_CMD.MAV_CMD_DO_FLIGHTTERMINATION:
					id = 37;
					break;
				case MAV_CMD.MAV_CMD_DO_CHANGE_ALTITUDE:
					id = 38;
					break;
				case MAV_CMD.MAV_CMD_DO_LAND_START:
					id = 39;
					break;
				case MAV_CMD.MAV_CMD_DO_RALLY_LAND:
					id = 40;
					break;
				case MAV_CMD.MAV_CMD_DO_GO_AROUND:
					id = 41;
					break;
				case MAV_CMD.MAV_CMD_DO_REPOSITION:
					id = 42;
					break;
				case MAV_CMD.MAV_CMD_DO_PAUSE_CONTINUE:
					id = 43;
					break;
				case MAV_CMD.MAV_CMD_DO_SET_REVERSE:
					id = 44;
					break;
				case MAV_CMD.MAV_CMD_DO_CONTROL_VIDEO:
					id = 45;
					break;
				case MAV_CMD.MAV_CMD_DO_SET_ROI:
					id = 46;
					break;
				case MAV_CMD.MAV_CMD_DO_DIGICAM_CONFIGURE:
					id = 47;
					break;
				case MAV_CMD.MAV_CMD_DO_DIGICAM_CONTROL:
					id = 48;
					break;
				case MAV_CMD.MAV_CMD_DO_MOUNT_CONFIGURE:
					id = 49;
					break;
				case MAV_CMD.MAV_CMD_DO_MOUNT_CONTROL:
					id = 50;
					break;
				case MAV_CMD.MAV_CMD_DO_SET_CAM_TRIGG_DIST:
					id = 51;
					break;
				case MAV_CMD.MAV_CMD_DO_FENCE_ENABLE:
					id = 52;
					break;
				case MAV_CMD.MAV_CMD_DO_PARACHUTE:
					id = 53;
					break;
				case MAV_CMD.MAV_CMD_DO_MOTOR_TEST:
					id = 54;
					break;
				case MAV_CMD.MAV_CMD_DO_INVERTED_FLIGHT:
					id = 55;
					break;
				case MAV_CMD.MAV_CMD_NAV_SET_YAW_SPEED:
					id = 56;
					break;
				case MAV_CMD.MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL:
					id = 57;
					break;
				case MAV_CMD.MAV_CMD_DO_MOUNT_CONTROL_QUAT:
					id = 58;
					break;
				case MAV_CMD.MAV_CMD_DO_GUIDED_MASTER:
					id = 59;
					break;
				case MAV_CMD.MAV_CMD_DO_GUIDED_LIMITS:
					id = 60;
					break;
				case MAV_CMD.MAV_CMD_DO_ENGINE_CONTROL:
					id = 61;
					break;
				case MAV_CMD.MAV_CMD_DO_LAST:
					id = 62;
					break;
				case MAV_CMD.MAV_CMD_PREFLIGHT_CALIBRATION:
					id = 63;
					break;
				case MAV_CMD.MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS:
					id = 64;
					break;
				case MAV_CMD.MAV_CMD_PREFLIGHT_UAVCAN:
					id = 65;
					break;
				case MAV_CMD.MAV_CMD_PREFLIGHT_STORAGE:
					id = 66;
					break;
				case MAV_CMD.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
					id = 67;
					break;
				case MAV_CMD.MAV_CMD_OVERRIDE_GOTO:
					id = 68;
					break;
				case MAV_CMD.MAV_CMD_MISSION_START:
					id = 69;
					break;
				case MAV_CMD.MAV_CMD_COMPONENT_ARM_DISARM:
					id = 70;
					break;
				case MAV_CMD.MAV_CMD_GET_HOME_POSITION:
					id = 71;
					break;
				case MAV_CMD.MAV_CMD_START_RX_PAIR:
					id = 72;
					break;
				case MAV_CMD.MAV_CMD_GET_MESSAGE_INTERVAL:
					id = 73;
					break;
				case MAV_CMD.MAV_CMD_SET_MESSAGE_INTERVAL:
					id = 74;
					break;
				case MAV_CMD.MAV_CMD_REQUEST_PROTOCOL_VERSION:
					id = 75;
					break;
				case MAV_CMD.MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES:
					id = 76;
					break;
				case MAV_CMD.MAV_CMD_REQUEST_CAMERA_INFORMATION:
					id = 77;
					break;
				case MAV_CMD.MAV_CMD_REQUEST_CAMERA_SETTINGS:
					id = 78;
					break;
				case MAV_CMD.MAV_CMD_REQUEST_STORAGE_INFORMATION:
					id = 79;
					break;
				case MAV_CMD.MAV_CMD_STORAGE_FORMAT:
					id = 80;
					break;
				case MAV_CMD.MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS:
					id = 81;
					break;
				case MAV_CMD.MAV_CMD_REQUEST_FLIGHT_INFORMATION:
					id = 82;
					break;
				case MAV_CMD.MAV_CMD_RESET_CAMERA_SETTINGS:
					id = 83;
					break;
				case MAV_CMD.MAV_CMD_SET_CAMERA_MODE:
					id = 84;
					break;
				case MAV_CMD.MAV_CMD_IMAGE_START_CAPTURE:
					id = 85;
					break;
				case MAV_CMD.MAV_CMD_IMAGE_STOP_CAPTURE:
					id = 86;
					break;
				case MAV_CMD.MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE:
					id = 87;
					break;
				case MAV_CMD.MAV_CMD_DO_TRIGGER_CONTROL:
					id = 88;
					break;
				case MAV_CMD.MAV_CMD_VIDEO_START_CAPTURE:
					id = 89;
					break;
				case MAV_CMD.MAV_CMD_VIDEO_STOP_CAPTURE:
					id = 90;
					break;
				case MAV_CMD.MAV_CMD_VIDEO_START_STREAMING:
					id = 91;
					break;
				case MAV_CMD.MAV_CMD_VIDEO_STOP_STREAMING:
					id = 92;
					break;
				case MAV_CMD.MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION:
					id = 93;
					break;
				case MAV_CMD.MAV_CMD_LOGGING_START:
					id = 94;
					break;
				case MAV_CMD.MAV_CMD_LOGGING_STOP:
					id = 95;
					break;
				case MAV_CMD.MAV_CMD_AIRFRAME_CONFIGURATION:
					id = 96;
					break;
				case MAV_CMD.MAV_CMD_PANORAMA_CREATE:
					id = 97;
					break;
				case MAV_CMD.MAV_CMD_DO_VTOL_TRANSITION:
					id = 98;
					break;
				case MAV_CMD.MAV_CMD_ARM_AUTHORIZATION_REQUEST:
					id = 99;
					break;
				case MAV_CMD.MAV_CMD_SET_GUIDED_SUBMODE_STANDARD:
					id = 100;
					break;
				case MAV_CMD.MAV_CMD_SET_GUIDED_SUBMODE_CIRCLE:
					id = 101;
					break;
				case MAV_CMD.MAV_CMD_CONDITION_GATE:
					id = 102;
					break;
				case MAV_CMD.MAV_CMD_NAV_FENCE_RETURN_POINT:
					id = 103;
					break;
				case MAV_CMD.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION:
					id = 104;
					break;
				case MAV_CMD.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION:
					id = 105;
					break;
				case MAV_CMD.MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION:
					id = 106;
					break;
				case MAV_CMD.MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION:
					id = 107;
					break;
				case MAV_CMD.MAV_CMD_NAV_RALLY_POINT:
					id = 108;
					break;
				case MAV_CMD.MAV_CMD_UAVCAN_GET_NODE_INFO:
					id = 109;
					break;
				case MAV_CMD.MAV_CMD_PAYLOAD_PREPARE_DEPLOY:
					id = 110;
					break;
				case MAV_CMD.MAV_CMD_PAYLOAD_CONTROL_DEPLOY:
					id = 111;
					break;
				case MAV_CMD.MAV_CMD_WAYPOINT_USER_1:
					id = 112;
					break;
				case MAV_CMD.MAV_CMD_WAYPOINT_USER_2:
					id = 113;
					break;
				case MAV_CMD.MAV_CMD_WAYPOINT_USER_3:
					id = 114;
					break;
				case MAV_CMD.MAV_CMD_WAYPOINT_USER_4:
					id = 115;
					break;
				case MAV_CMD.MAV_CMD_WAYPOINT_USER_5:
					id = 116;
					break;
				case MAV_CMD.MAV_CMD_SPATIAL_USER_1:
					id = 117;
					break;
				case MAV_CMD.MAV_CMD_SPATIAL_USER_2:
					id = 118;
					break;
				case MAV_CMD.MAV_CMD_SPATIAL_USER_3:
					id = 119;
					break;
				case MAV_CMD.MAV_CMD_SPATIAL_USER_4:
					id = 120;
					break;
				case MAV_CMD.MAV_CMD_SPATIAL_USER_5:
					id = 121;
					break;
				case MAV_CMD.MAV_CMD_USER_1:
					id = 122;
					break;
				case MAV_CMD.MAV_CMD_USER_2:
					id = 123;
					break;
				case MAV_CMD.MAV_CMD_USER_3:
					id = 124;
					break;
				case MAV_CMD.MAV_CMD_USER_4:
					id = 125;
					break;
				case MAV_CMD.MAV_CMD_USER_5:
					id = 126;
					break;
				case MAV_CMD.MAV_CMD_RESET_MPPT:
					id = 127;
					break;
				case MAV_CMD.MAV_CMD_PAYLOAD_CONTROL:
					id = 128;
					break;
				default:
					assert (false);//("Unknown enum" + id);
			}
			set_bits(id, 8, data, 276);
		}

		public void mission_type_SET(@MAV_MISSION_TYPE int src) //Mission type, see MAV_MISSION_TYPE
		{
			long id = 0;
			switch (src)
			{
				case MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION:
					id = 0;
					break;
				case MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE:
					id = 1;
					break;
				case MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY:
					id = 2;
					break;
				case MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL:
					id = 3;
					break;
				default:
					assert (false);//("Unknown enum" + id);
			}
			set_bits(id, 3, data, 284);
		}
	}

	public static class VFR_HUD extends GroundControl.VFR_HUD {
		public void throttle_SET(char src) //Current throttle setting in integer percent, 0 to 100
		{ set_bytes((char) (src) & -1L, 2, data, 0); }

		public void airspeed_SET(float src) //Current airspeed in m/s
		{ set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 2); }

		public void groundspeed_SET(float src) //Current ground speed in m/s
		{ set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 6); }

		public void heading_SET(short src) //Current heading in degrees, in compass units (0..360, 0=north)
		{ set_bytes((short) (src) & -1L, 2, data, 10); }

		public void alt_SET(float src) //Current altitude (MSL), in meters
		{ set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 12); }

		public void climb_SET(float src) //Current climb rate in meters/second
		{ set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 16); }
	}

	public static class COMMAND_INT extends GroundControl.COMMAND_INT {
		public void target_system_SET(char src) //System ID
		{ set_bytes((char) (src) & -1L, 1, data, 0); }

		public void target_component_SET(char src) //Component ID
		{ set_bytes((char) (src) & -1L, 1, data, 1); }

		public void current_SET(char src) //false:0, true:1
		{ set_bytes((char) (src) & -1L, 1, data, 2); }

		public void autocontinue_SET(char src) //autocontinue to next wp
		{ set_bytes((char) (src) & -1L, 1, data, 3); }

		public void param1_SET(float src) //PARAM1, see MAV_CMD enum
		{ set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 4); }

		public void param2_SET(float src) //PARAM2, see MAV_CMD enum
		{ set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 8); }

		public void param3_SET(float src) //PARAM3, see MAV_CMD enum
		{ set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 12); }

		public void param4_SET(float src) //PARAM4, see MAV_CMD enum
		{ set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 16); }

		public void x_SET(int src) //PARAM5 / local: x position in meters * 1e4, global: latitude in degrees * 10^7
		{ set_bytes((int) (src) & -1L, 4, data, 20); }

		public void y_SET(int src) //PARAM6 / local: y position in meters * 1e4, global: longitude in degrees * 10^7
		{ set_bytes((int) (src) & -1L, 4, data, 24); }

		public void z_SET(float src) //PARAM7 / z position: global: altitude in meters (relative or absolute, depending on frame.
		{ set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 28); }

		public void frame_SET(@MAV_FRAME int src) //The coordinate system of the COMMAND. see MAV_FRAME in mavlink_types.h
		{ set_bits(-0 + src, 4, data, 256); }

		public void command_SET(@MAV_CMD int src) //The scheduled action for the mission item. see MAV_CMD in common.xml MAVLink specs
		{
			long id = 0;
			switch (src)
			{
				case MAV_CMD.MAV_CMD_NAV_WAYPOINT:
					id = 0;
					break;
				case MAV_CMD.MAV_CMD_NAV_LOITER_UNLIM:
					id = 1;
					break;
				case MAV_CMD.MAV_CMD_NAV_LOITER_TURNS:
					id = 2;
					break;
				case MAV_CMD.MAV_CMD_NAV_LOITER_TIME:
					id = 3;
					break;
				case MAV_CMD.MAV_CMD_NAV_RETURN_TO_LAUNCH:
					id = 4;
					break;
				case MAV_CMD.MAV_CMD_NAV_LAND:
					id = 5;
					break;
				case MAV_CMD.MAV_CMD_NAV_TAKEOFF:
					id = 6;
					break;
				case MAV_CMD.MAV_CMD_NAV_LAND_LOCAL:
					id = 7;
					break;
				case MAV_CMD.MAV_CMD_NAV_TAKEOFF_LOCAL:
					id = 8;
					break;
				case MAV_CMD.MAV_CMD_NAV_FOLLOW:
					id = 9;
					break;
				case MAV_CMD.MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT:
					id = 10;
					break;
				case MAV_CMD.MAV_CMD_NAV_LOITER_TO_ALT:
					id = 11;
					break;
				case MAV_CMD.MAV_CMD_DO_FOLLOW:
					id = 12;
					break;
				case MAV_CMD.MAV_CMD_DO_FOLLOW_REPOSITION:
					id = 13;
					break;
				case MAV_CMD.MAV_CMD_NAV_ROI:
					id = 14;
					break;
				case MAV_CMD.MAV_CMD_NAV_PATHPLANNING:
					id = 15;
					break;
				case MAV_CMD.MAV_CMD_NAV_SPLINE_WAYPOINT:
					id = 16;
					break;
				case MAV_CMD.MAV_CMD_NAV_VTOL_TAKEOFF:
					id = 17;
					break;
				case MAV_CMD.MAV_CMD_NAV_VTOL_LAND:
					id = 18;
					break;
				case MAV_CMD.MAV_CMD_NAV_GUIDED_ENABLE:
					id = 19;
					break;
				case MAV_CMD.MAV_CMD_NAV_DELAY:
					id = 20;
					break;
				case MAV_CMD.MAV_CMD_NAV_PAYLOAD_PLACE:
					id = 21;
					break;
				case MAV_CMD.MAV_CMD_NAV_LAST:
					id = 22;
					break;
				case MAV_CMD.MAV_CMD_CONDITION_DELAY:
					id = 23;
					break;
				case MAV_CMD.MAV_CMD_CONDITION_CHANGE_ALT:
					id = 24;
					break;
				case MAV_CMD.MAV_CMD_CONDITION_DISTANCE:
					id = 25;
					break;
				case MAV_CMD.MAV_CMD_CONDITION_YAW:
					id = 26;
					break;
				case MAV_CMD.MAV_CMD_CONDITION_LAST:
					id = 27;
					break;
				case MAV_CMD.MAV_CMD_DO_SET_MODE:
					id = 28;
					break;
				case MAV_CMD.MAV_CMD_DO_JUMP:
					id = 29;
					break;
				case MAV_CMD.MAV_CMD_DO_CHANGE_SPEED:
					id = 30;
					break;
				case MAV_CMD.MAV_CMD_DO_SET_HOME:
					id = 31;
					break;
				case MAV_CMD.MAV_CMD_DO_SET_PARAMETER:
					id = 32;
					break;
				case MAV_CMD.MAV_CMD_DO_SET_RELAY:
					id = 33;
					break;
				case MAV_CMD.MAV_CMD_DO_REPEAT_RELAY:
					id = 34;
					break;
				case MAV_CMD.MAV_CMD_DO_SET_SERVO:
					id = 35;
					break;
				case MAV_CMD.MAV_CMD_DO_REPEAT_SERVO:
					id = 36;
					break;
				case MAV_CMD.MAV_CMD_DO_FLIGHTTERMINATION:
					id = 37;
					break;
				case MAV_CMD.MAV_CMD_DO_CHANGE_ALTITUDE:
					id = 38;
					break;
				case MAV_CMD.MAV_CMD_DO_LAND_START:
					id = 39;
					break;
				case MAV_CMD.MAV_CMD_DO_RALLY_LAND:
					id = 40;
					break;
				case MAV_CMD.MAV_CMD_DO_GO_AROUND:
					id = 41;
					break;
				case MAV_CMD.MAV_CMD_DO_REPOSITION:
					id = 42;
					break;
				case MAV_CMD.MAV_CMD_DO_PAUSE_CONTINUE:
					id = 43;
					break;
				case MAV_CMD.MAV_CMD_DO_SET_REVERSE:
					id = 44;
					break;
				case MAV_CMD.MAV_CMD_DO_CONTROL_VIDEO:
					id = 45;
					break;
				case MAV_CMD.MAV_CMD_DO_SET_ROI:
					id = 46;
					break;
				case MAV_CMD.MAV_CMD_DO_DIGICAM_CONFIGURE:
					id = 47;
					break;
				case MAV_CMD.MAV_CMD_DO_DIGICAM_CONTROL:
					id = 48;
					break;
				case MAV_CMD.MAV_CMD_DO_MOUNT_CONFIGURE:
					id = 49;
					break;
				case MAV_CMD.MAV_CMD_DO_MOUNT_CONTROL:
					id = 50;
					break;
				case MAV_CMD.MAV_CMD_DO_SET_CAM_TRIGG_DIST:
					id = 51;
					break;
				case MAV_CMD.MAV_CMD_DO_FENCE_ENABLE:
					id = 52;
					break;
				case MAV_CMD.MAV_CMD_DO_PARACHUTE:
					id = 53;
					break;
				case MAV_CMD.MAV_CMD_DO_MOTOR_TEST:
					id = 54;
					break;
				case MAV_CMD.MAV_CMD_DO_INVERTED_FLIGHT:
					id = 55;
					break;
				case MAV_CMD.MAV_CMD_NAV_SET_YAW_SPEED:
					id = 56;
					break;
				case MAV_CMD.MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL:
					id = 57;
					break;
				case MAV_CMD.MAV_CMD_DO_MOUNT_CONTROL_QUAT:
					id = 58;
					break;
				case MAV_CMD.MAV_CMD_DO_GUIDED_MASTER:
					id = 59;
					break;
				case MAV_CMD.MAV_CMD_DO_GUIDED_LIMITS:
					id = 60;
					break;
				case MAV_CMD.MAV_CMD_DO_ENGINE_CONTROL:
					id = 61;
					break;
				case MAV_CMD.MAV_CMD_DO_LAST:
					id = 62;
					break;
				case MAV_CMD.MAV_CMD_PREFLIGHT_CALIBRATION:
					id = 63;
					break;
				case MAV_CMD.MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS:
					id = 64;
					break;
				case MAV_CMD.MAV_CMD_PREFLIGHT_UAVCAN:
					id = 65;
					break;
				case MAV_CMD.MAV_CMD_PREFLIGHT_STORAGE:
					id = 66;
					break;
				case MAV_CMD.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
					id = 67;
					break;
				case MAV_CMD.MAV_CMD_OVERRIDE_GOTO:
					id = 68;
					break;
				case MAV_CMD.MAV_CMD_MISSION_START:
					id = 69;
					break;
				case MAV_CMD.MAV_CMD_COMPONENT_ARM_DISARM:
					id = 70;
					break;
				case MAV_CMD.MAV_CMD_GET_HOME_POSITION:
					id = 71;
					break;
				case MAV_CMD.MAV_CMD_START_RX_PAIR:
					id = 72;
					break;
				case MAV_CMD.MAV_CMD_GET_MESSAGE_INTERVAL:
					id = 73;
					break;
				case MAV_CMD.MAV_CMD_SET_MESSAGE_INTERVAL:
					id = 74;
					break;
				case MAV_CMD.MAV_CMD_REQUEST_PROTOCOL_VERSION:
					id = 75;
					break;
				case MAV_CMD.MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES:
					id = 76;
					break;
				case MAV_CMD.MAV_CMD_REQUEST_CAMERA_INFORMATION:
					id = 77;
					break;
				case MAV_CMD.MAV_CMD_REQUEST_CAMERA_SETTINGS:
					id = 78;
					break;
				case MAV_CMD.MAV_CMD_REQUEST_STORAGE_INFORMATION:
					id = 79;
					break;
				case MAV_CMD.MAV_CMD_STORAGE_FORMAT:
					id = 80;
					break;
				case MAV_CMD.MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS:
					id = 81;
					break;
				case MAV_CMD.MAV_CMD_REQUEST_FLIGHT_INFORMATION:
					id = 82;
					break;
				case MAV_CMD.MAV_CMD_RESET_CAMERA_SETTINGS:
					id = 83;
					break;
				case MAV_CMD.MAV_CMD_SET_CAMERA_MODE:
					id = 84;
					break;
				case MAV_CMD.MAV_CMD_IMAGE_START_CAPTURE:
					id = 85;
					break;
				case MAV_CMD.MAV_CMD_IMAGE_STOP_CAPTURE:
					id = 86;
					break;
				case MAV_CMD.MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE:
					id = 87;
					break;
				case MAV_CMD.MAV_CMD_DO_TRIGGER_CONTROL:
					id = 88;
					break;
				case MAV_CMD.MAV_CMD_VIDEO_START_CAPTURE:
					id = 89;
					break;
				case MAV_CMD.MAV_CMD_VIDEO_STOP_CAPTURE:
					id = 90;
					break;
				case MAV_CMD.MAV_CMD_VIDEO_START_STREAMING:
					id = 91;
					break;
				case MAV_CMD.MAV_CMD_VIDEO_STOP_STREAMING:
					id = 92;
					break;
				case MAV_CMD.MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION:
					id = 93;
					break;
				case MAV_CMD.MAV_CMD_LOGGING_START:
					id = 94;
					break;
				case MAV_CMD.MAV_CMD_LOGGING_STOP:
					id = 95;
					break;
				case MAV_CMD.MAV_CMD_AIRFRAME_CONFIGURATION:
					id = 96;
					break;
				case MAV_CMD.MAV_CMD_PANORAMA_CREATE:
					id = 97;
					break;
				case MAV_CMD.MAV_CMD_DO_VTOL_TRANSITION:
					id = 98;
					break;
				case MAV_CMD.MAV_CMD_ARM_AUTHORIZATION_REQUEST:
					id = 99;
					break;
				case MAV_CMD.MAV_CMD_SET_GUIDED_SUBMODE_STANDARD:
					id = 100;
					break;
				case MAV_CMD.MAV_CMD_SET_GUIDED_SUBMODE_CIRCLE:
					id = 101;
					break;
				case MAV_CMD.MAV_CMD_CONDITION_GATE:
					id = 102;
					break;
				case MAV_CMD.MAV_CMD_NAV_FENCE_RETURN_POINT:
					id = 103;
					break;
				case MAV_CMD.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION:
					id = 104;
					break;
				case MAV_CMD.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION:
					id = 105;
					break;
				case MAV_CMD.MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION:
					id = 106;
					break;
				case MAV_CMD.MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION:
					id = 107;
					break;
				case MAV_CMD.MAV_CMD_NAV_RALLY_POINT:
					id = 108;
					break;
				case MAV_CMD.MAV_CMD_UAVCAN_GET_NODE_INFO:
					id = 109;
					break;
				case MAV_CMD.MAV_CMD_PAYLOAD_PREPARE_DEPLOY:
					id = 110;
					break;
				case MAV_CMD.MAV_CMD_PAYLOAD_CONTROL_DEPLOY:
					id = 111;
					break;
				case MAV_CMD.MAV_CMD_WAYPOINT_USER_1:
					id = 112;
					break;
				case MAV_CMD.MAV_CMD_WAYPOINT_USER_2:
					id = 113;
					break;
				case MAV_CMD.MAV_CMD_WAYPOINT_USER_3:
					id = 114;
					break;
				case MAV_CMD.MAV_CMD_WAYPOINT_USER_4:
					id = 115;
					break;
				case MAV_CMD.MAV_CMD_WAYPOINT_USER_5:
					id = 116;
					break;
				case MAV_CMD.MAV_CMD_SPATIAL_USER_1:
					id = 117;
					break;
				case MAV_CMD.MAV_CMD_SPATIAL_USER_2:
					id = 118;
					break;
				case MAV_CMD.MAV_CMD_SPATIAL_USER_3:
					id = 119;
					break;
				case MAV_CMD.MAV_CMD_SPATIAL_USER_4:
					id = 120;
					break;
				case MAV_CMD.MAV_CMD_SPATIAL_USER_5:
					id = 121;
					break;
				case MAV_CMD.MAV_CMD_USER_1:
					id = 122;
					break;
				case MAV_CMD.MAV_CMD_USER_2:
					id = 123;
					break;
				case MAV_CMD.MAV_CMD_USER_3:
					id = 124;
					break;
				case MAV_CMD.MAV_CMD_USER_4:
					id = 125;
					break;
				case MAV_CMD.MAV_CMD_USER_5:
					id = 126;
					break;
				case MAV_CMD.MAV_CMD_RESET_MPPT:
					id = 127;
					break;
				case MAV_CMD.MAV_CMD_PAYLOAD_CONTROL:
					id = 128;
					break;
				default:
					assert (false);//("Unknown enum" + id);
			}
			set_bits(id, 8, data, 260);
		}
	}

	public static class COMMAND_LONG extends GroundControl.COMMAND_LONG {
		public void target_system_SET(char src) //System which should execute the command
		{ set_bytes((char) (src) & -1L, 1, data, 0); }

		public void target_component_SET(char src) //Component which should execute the command, 0 for all components
		{ set_bytes((char) (src) & -1L, 1, data, 1); }

		public void confirmation_SET(char src) //0: First transmission of this command. 1-255: Confirmation transmissions (e.g. for kill command)
		{ set_bytes((char) (src) & -1L, 1, data, 2); }

		public void param1_SET(float src) //Parameter 1, as defined by MAV_CMD enum.
		{ set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 3); }

		public void param2_SET(float src) //Parameter 2, as defined by MAV_CMD enum.
		{ set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 7); }

		public void param3_SET(float src) //Parameter 3, as defined by MAV_CMD enum.
		{ set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 11); }

		public void param4_SET(float src) //Parameter 4, as defined by MAV_CMD enum.
		{ set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 15); }

		public void param5_SET(float src) //Parameter 5, as defined by MAV_CMD enum.
		{ set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 19); }

		public void param6_SET(float src) //Parameter 6, as defined by MAV_CMD enum.
		{ set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 23); }

		public void param7_SET(float src) //Parameter 7, as defined by MAV_CMD enum.
		{ set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 27); }

		public void command_SET(@MAV_CMD int src) //Command ID, as defined by MAV_CMD enum.
		{
			long id = 0;
			switch (src)
			{
				case MAV_CMD.MAV_CMD_NAV_WAYPOINT:
					id = 0;
					break;
				case MAV_CMD.MAV_CMD_NAV_LOITER_UNLIM:
					id = 1;
					break;
				case MAV_CMD.MAV_CMD_NAV_LOITER_TURNS:
					id = 2;
					break;
				case MAV_CMD.MAV_CMD_NAV_LOITER_TIME:
					id = 3;
					break;
				case MAV_CMD.MAV_CMD_NAV_RETURN_TO_LAUNCH:
					id = 4;
					break;
				case MAV_CMD.MAV_CMD_NAV_LAND:
					id = 5;
					break;
				case MAV_CMD.MAV_CMD_NAV_TAKEOFF:
					id = 6;
					break;
				case MAV_CMD.MAV_CMD_NAV_LAND_LOCAL:
					id = 7;
					break;
				case MAV_CMD.MAV_CMD_NAV_TAKEOFF_LOCAL:
					id = 8;
					break;
				case MAV_CMD.MAV_CMD_NAV_FOLLOW:
					id = 9;
					break;
				case MAV_CMD.MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT:
					id = 10;
					break;
				case MAV_CMD.MAV_CMD_NAV_LOITER_TO_ALT:
					id = 11;
					break;
				case MAV_CMD.MAV_CMD_DO_FOLLOW:
					id = 12;
					break;
				case MAV_CMD.MAV_CMD_DO_FOLLOW_REPOSITION:
					id = 13;
					break;
				case MAV_CMD.MAV_CMD_NAV_ROI:
					id = 14;
					break;
				case MAV_CMD.MAV_CMD_NAV_PATHPLANNING:
					id = 15;
					break;
				case MAV_CMD.MAV_CMD_NAV_SPLINE_WAYPOINT:
					id = 16;
					break;
				case MAV_CMD.MAV_CMD_NAV_VTOL_TAKEOFF:
					id = 17;
					break;
				case MAV_CMD.MAV_CMD_NAV_VTOL_LAND:
					id = 18;
					break;
				case MAV_CMD.MAV_CMD_NAV_GUIDED_ENABLE:
					id = 19;
					break;
				case MAV_CMD.MAV_CMD_NAV_DELAY:
					id = 20;
					break;
				case MAV_CMD.MAV_CMD_NAV_PAYLOAD_PLACE:
					id = 21;
					break;
				case MAV_CMD.MAV_CMD_NAV_LAST:
					id = 22;
					break;
				case MAV_CMD.MAV_CMD_CONDITION_DELAY:
					id = 23;
					break;
				case MAV_CMD.MAV_CMD_CONDITION_CHANGE_ALT:
					id = 24;
					break;
				case MAV_CMD.MAV_CMD_CONDITION_DISTANCE:
					id = 25;
					break;
				case MAV_CMD.MAV_CMD_CONDITION_YAW:
					id = 26;
					break;
				case MAV_CMD.MAV_CMD_CONDITION_LAST:
					id = 27;
					break;
				case MAV_CMD.MAV_CMD_DO_SET_MODE:
					id = 28;
					break;
				case MAV_CMD.MAV_CMD_DO_JUMP:
					id = 29;
					break;
				case MAV_CMD.MAV_CMD_DO_CHANGE_SPEED:
					id = 30;
					break;
				case MAV_CMD.MAV_CMD_DO_SET_HOME:
					id = 31;
					break;
				case MAV_CMD.MAV_CMD_DO_SET_PARAMETER:
					id = 32;
					break;
				case MAV_CMD.MAV_CMD_DO_SET_RELAY:
					id = 33;
					break;
				case MAV_CMD.MAV_CMD_DO_REPEAT_RELAY:
					id = 34;
					break;
				case MAV_CMD.MAV_CMD_DO_SET_SERVO:
					id = 35;
					break;
				case MAV_CMD.MAV_CMD_DO_REPEAT_SERVO:
					id = 36;
					break;
				case MAV_CMD.MAV_CMD_DO_FLIGHTTERMINATION:
					id = 37;
					break;
				case MAV_CMD.MAV_CMD_DO_CHANGE_ALTITUDE:
					id = 38;
					break;
				case MAV_CMD.MAV_CMD_DO_LAND_START:
					id = 39;
					break;
				case MAV_CMD.MAV_CMD_DO_RALLY_LAND:
					id = 40;
					break;
				case MAV_CMD.MAV_CMD_DO_GO_AROUND:
					id = 41;
					break;
				case MAV_CMD.MAV_CMD_DO_REPOSITION:
					id = 42;
					break;
				case MAV_CMD.MAV_CMD_DO_PAUSE_CONTINUE:
					id = 43;
					break;
				case MAV_CMD.MAV_CMD_DO_SET_REVERSE:
					id = 44;
					break;
				case MAV_CMD.MAV_CMD_DO_CONTROL_VIDEO:
					id = 45;
					break;
				case MAV_CMD.MAV_CMD_DO_SET_ROI:
					id = 46;
					break;
				case MAV_CMD.MAV_CMD_DO_DIGICAM_CONFIGURE:
					id = 47;
					break;
				case MAV_CMD.MAV_CMD_DO_DIGICAM_CONTROL:
					id = 48;
					break;
				case MAV_CMD.MAV_CMD_DO_MOUNT_CONFIGURE:
					id = 49;
					break;
				case MAV_CMD.MAV_CMD_DO_MOUNT_CONTROL:
					id = 50;
					break;
				case MAV_CMD.MAV_CMD_DO_SET_CAM_TRIGG_DIST:
					id = 51;
					break;
				case MAV_CMD.MAV_CMD_DO_FENCE_ENABLE:
					id = 52;
					break;
				case MAV_CMD.MAV_CMD_DO_PARACHUTE:
					id = 53;
					break;
				case MAV_CMD.MAV_CMD_DO_MOTOR_TEST:
					id = 54;
					break;
				case MAV_CMD.MAV_CMD_DO_INVERTED_FLIGHT:
					id = 55;
					break;
				case MAV_CMD.MAV_CMD_NAV_SET_YAW_SPEED:
					id = 56;
					break;
				case MAV_CMD.MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL:
					id = 57;
					break;
				case MAV_CMD.MAV_CMD_DO_MOUNT_CONTROL_QUAT:
					id = 58;
					break;
				case MAV_CMD.MAV_CMD_DO_GUIDED_MASTER:
					id = 59;
					break;
				case MAV_CMD.MAV_CMD_DO_GUIDED_LIMITS:
					id = 60;
					break;
				case MAV_CMD.MAV_CMD_DO_ENGINE_CONTROL:
					id = 61;
					break;
				case MAV_CMD.MAV_CMD_DO_LAST:
					id = 62;
					break;
				case MAV_CMD.MAV_CMD_PREFLIGHT_CALIBRATION:
					id = 63;
					break;
				case MAV_CMD.MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS:
					id = 64;
					break;
				case MAV_CMD.MAV_CMD_PREFLIGHT_UAVCAN:
					id = 65;
					break;
				case MAV_CMD.MAV_CMD_PREFLIGHT_STORAGE:
					id = 66;
					break;
				case MAV_CMD.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
					id = 67;
					break;
				case MAV_CMD.MAV_CMD_OVERRIDE_GOTO:
					id = 68;
					break;
				case MAV_CMD.MAV_CMD_MISSION_START:
					id = 69;
					break;
				case MAV_CMD.MAV_CMD_COMPONENT_ARM_DISARM:
					id = 70;
					break;
				case MAV_CMD.MAV_CMD_GET_HOME_POSITION:
					id = 71;
					break;
				case MAV_CMD.MAV_CMD_START_RX_PAIR:
					id = 72;
					break;
				case MAV_CMD.MAV_CMD_GET_MESSAGE_INTERVAL:
					id = 73;
					break;
				case MAV_CMD.MAV_CMD_SET_MESSAGE_INTERVAL:
					id = 74;
					break;
				case MAV_CMD.MAV_CMD_REQUEST_PROTOCOL_VERSION:
					id = 75;
					break;
				case MAV_CMD.MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES:
					id = 76;
					break;
				case MAV_CMD.MAV_CMD_REQUEST_CAMERA_INFORMATION:
					id = 77;
					break;
				case MAV_CMD.MAV_CMD_REQUEST_CAMERA_SETTINGS:
					id = 78;
					break;
				case MAV_CMD.MAV_CMD_REQUEST_STORAGE_INFORMATION:
					id = 79;
					break;
				case MAV_CMD.MAV_CMD_STORAGE_FORMAT:
					id = 80;
					break;
				case MAV_CMD.MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS:
					id = 81;
					break;
				case MAV_CMD.MAV_CMD_REQUEST_FLIGHT_INFORMATION:
					id = 82;
					break;
				case MAV_CMD.MAV_CMD_RESET_CAMERA_SETTINGS:
					id = 83;
					break;
				case MAV_CMD.MAV_CMD_SET_CAMERA_MODE:
					id = 84;
					break;
				case MAV_CMD.MAV_CMD_IMAGE_START_CAPTURE:
					id = 85;
					break;
				case MAV_CMD.MAV_CMD_IMAGE_STOP_CAPTURE:
					id = 86;
					break;
				case MAV_CMD.MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE:
					id = 87;
					break;
				case MAV_CMD.MAV_CMD_DO_TRIGGER_CONTROL:
					id = 88;
					break;
				case MAV_CMD.MAV_CMD_VIDEO_START_CAPTURE:
					id = 89;
					break;
				case MAV_CMD.MAV_CMD_VIDEO_STOP_CAPTURE:
					id = 90;
					break;
				case MAV_CMD.MAV_CMD_VIDEO_START_STREAMING:
					id = 91;
					break;
				case MAV_CMD.MAV_CMD_VIDEO_STOP_STREAMING:
					id = 92;
					break;
				case MAV_CMD.MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION:
					id = 93;
					break;
				case MAV_CMD.MAV_CMD_LOGGING_START:
					id = 94;
					break;
				case MAV_CMD.MAV_CMD_LOGGING_STOP:
					id = 95;
					break;
				case MAV_CMD.MAV_CMD_AIRFRAME_CONFIGURATION:
					id = 96;
					break;
				case MAV_CMD.MAV_CMD_PANORAMA_CREATE:
					id = 97;
					break;
				case MAV_CMD.MAV_CMD_DO_VTOL_TRANSITION:
					id = 98;
					break;
				case MAV_CMD.MAV_CMD_ARM_AUTHORIZATION_REQUEST:
					id = 99;
					break;
				case MAV_CMD.MAV_CMD_SET_GUIDED_SUBMODE_STANDARD:
					id = 100;
					break;
				case MAV_CMD.MAV_CMD_SET_GUIDED_SUBMODE_CIRCLE:
					id = 101;
					break;
				case MAV_CMD.MAV_CMD_CONDITION_GATE:
					id = 102;
					break;
				case MAV_CMD.MAV_CMD_NAV_FENCE_RETURN_POINT:
					id = 103;
					break;
				case MAV_CMD.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION:
					id = 104;
					break;
				case MAV_CMD.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION:
					id = 105;
					break;
				case MAV_CMD.MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION:
					id = 106;
					break;
				case MAV_CMD.MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION:
					id = 107;
					break;
				case MAV_CMD.MAV_CMD_NAV_RALLY_POINT:
					id = 108;
					break;
				case MAV_CMD.MAV_CMD_UAVCAN_GET_NODE_INFO:
					id = 109;
					break;
				case MAV_CMD.MAV_CMD_PAYLOAD_PREPARE_DEPLOY:
					id = 110;
					break;
				case MAV_CMD.MAV_CMD_PAYLOAD_CONTROL_DEPLOY:
					id = 111;
					break;
				case MAV_CMD.MAV_CMD_WAYPOINT_USER_1:
					id = 112;
					break;
				case MAV_CMD.MAV_CMD_WAYPOINT_USER_2:
					id = 113;
					break;
				case MAV_CMD.MAV_CMD_WAYPOINT_USER_3:
					id = 114;
					break;
				case MAV_CMD.MAV_CMD_WAYPOINT_USER_4:
					id = 115;
					break;
				case MAV_CMD.MAV_CMD_WAYPOINT_USER_5:
					id = 116;
					break;
				case MAV_CMD.MAV_CMD_SPATIAL_USER_1:
					id = 117;
					break;
				case MAV_CMD.MAV_CMD_SPATIAL_USER_2:
					id = 118;
					break;
				case MAV_CMD.MAV_CMD_SPATIAL_USER_3:
					id = 119;
					break;
				case MAV_CMD.MAV_CMD_SPATIAL_USER_4:
					id = 120;
					break;
				case MAV_CMD.MAV_CMD_SPATIAL_USER_5:
					id = 121;
					break;
				case MAV_CMD.MAV_CMD_USER_1:
					id = 122;
					break;
				case MAV_CMD.MAV_CMD_USER_2:
					id = 123;
					break;
				case MAV_CMD.MAV_CMD_USER_3:
					id = 124;
					break;
				case MAV_CMD.MAV_CMD_USER_4:
					id = 125;
					break;
				case MAV_CMD.MAV_CMD_USER_5:
					id = 126;
					break;
				case MAV_CMD.MAV_CMD_RESET_MPPT:
					id = 127;
					break;
				case MAV_CMD.MAV_CMD_PAYLOAD_CONTROL:
					id = 128;
					break;
				default:
					assert (false);//("Unknown enum" + id);
			}
			set_bits(id, 8, data, 248);
		}
	}

	public static class COMMAND_ACK extends GroundControl.COMMAND_ACK {
		public void command_SET(@MAV_CMD int src) //Command ID, as defined by MAV_CMD enum.
		{
			long id = 0;
			switch (src)
			{
				case MAV_CMD.MAV_CMD_NAV_WAYPOINT:
					id = 0;
					break;
				case MAV_CMD.MAV_CMD_NAV_LOITER_UNLIM:
					id = 1;
					break;
				case MAV_CMD.MAV_CMD_NAV_LOITER_TURNS:
					id = 2;
					break;
				case MAV_CMD.MAV_CMD_NAV_LOITER_TIME:
					id = 3;
					break;
				case MAV_CMD.MAV_CMD_NAV_RETURN_TO_LAUNCH:
					id = 4;
					break;
				case MAV_CMD.MAV_CMD_NAV_LAND:
					id = 5;
					break;
				case MAV_CMD.MAV_CMD_NAV_TAKEOFF:
					id = 6;
					break;
				case MAV_CMD.MAV_CMD_NAV_LAND_LOCAL:
					id = 7;
					break;
				case MAV_CMD.MAV_CMD_NAV_TAKEOFF_LOCAL:
					id = 8;
					break;
				case MAV_CMD.MAV_CMD_NAV_FOLLOW:
					id = 9;
					break;
				case MAV_CMD.MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT:
					id = 10;
					break;
				case MAV_CMD.MAV_CMD_NAV_LOITER_TO_ALT:
					id = 11;
					break;
				case MAV_CMD.MAV_CMD_DO_FOLLOW:
					id = 12;
					break;
				case MAV_CMD.MAV_CMD_DO_FOLLOW_REPOSITION:
					id = 13;
					break;
				case MAV_CMD.MAV_CMD_NAV_ROI:
					id = 14;
					break;
				case MAV_CMD.MAV_CMD_NAV_PATHPLANNING:
					id = 15;
					break;
				case MAV_CMD.MAV_CMD_NAV_SPLINE_WAYPOINT:
					id = 16;
					break;
				case MAV_CMD.MAV_CMD_NAV_VTOL_TAKEOFF:
					id = 17;
					break;
				case MAV_CMD.MAV_CMD_NAV_VTOL_LAND:
					id = 18;
					break;
				case MAV_CMD.MAV_CMD_NAV_GUIDED_ENABLE:
					id = 19;
					break;
				case MAV_CMD.MAV_CMD_NAV_DELAY:
					id = 20;
					break;
				case MAV_CMD.MAV_CMD_NAV_PAYLOAD_PLACE:
					id = 21;
					break;
				case MAV_CMD.MAV_CMD_NAV_LAST:
					id = 22;
					break;
				case MAV_CMD.MAV_CMD_CONDITION_DELAY:
					id = 23;
					break;
				case MAV_CMD.MAV_CMD_CONDITION_CHANGE_ALT:
					id = 24;
					break;
				case MAV_CMD.MAV_CMD_CONDITION_DISTANCE:
					id = 25;
					break;
				case MAV_CMD.MAV_CMD_CONDITION_YAW:
					id = 26;
					break;
				case MAV_CMD.MAV_CMD_CONDITION_LAST:
					id = 27;
					break;
				case MAV_CMD.MAV_CMD_DO_SET_MODE:
					id = 28;
					break;
				case MAV_CMD.MAV_CMD_DO_JUMP:
					id = 29;
					break;
				case MAV_CMD.MAV_CMD_DO_CHANGE_SPEED:
					id = 30;
					break;
				case MAV_CMD.MAV_CMD_DO_SET_HOME:
					id = 31;
					break;
				case MAV_CMD.MAV_CMD_DO_SET_PARAMETER:
					id = 32;
					break;
				case MAV_CMD.MAV_CMD_DO_SET_RELAY:
					id = 33;
					break;
				case MAV_CMD.MAV_CMD_DO_REPEAT_RELAY:
					id = 34;
					break;
				case MAV_CMD.MAV_CMD_DO_SET_SERVO:
					id = 35;
					break;
				case MAV_CMD.MAV_CMD_DO_REPEAT_SERVO:
					id = 36;
					break;
				case MAV_CMD.MAV_CMD_DO_FLIGHTTERMINATION:
					id = 37;
					break;
				case MAV_CMD.MAV_CMD_DO_CHANGE_ALTITUDE:
					id = 38;
					break;
				case MAV_CMD.MAV_CMD_DO_LAND_START:
					id = 39;
					break;
				case MAV_CMD.MAV_CMD_DO_RALLY_LAND:
					id = 40;
					break;
				case MAV_CMD.MAV_CMD_DO_GO_AROUND:
					id = 41;
					break;
				case MAV_CMD.MAV_CMD_DO_REPOSITION:
					id = 42;
					break;
				case MAV_CMD.MAV_CMD_DO_PAUSE_CONTINUE:
					id = 43;
					break;
				case MAV_CMD.MAV_CMD_DO_SET_REVERSE:
					id = 44;
					break;
				case MAV_CMD.MAV_CMD_DO_CONTROL_VIDEO:
					id = 45;
					break;
				case MAV_CMD.MAV_CMD_DO_SET_ROI:
					id = 46;
					break;
				case MAV_CMD.MAV_CMD_DO_DIGICAM_CONFIGURE:
					id = 47;
					break;
				case MAV_CMD.MAV_CMD_DO_DIGICAM_CONTROL:
					id = 48;
					break;
				case MAV_CMD.MAV_CMD_DO_MOUNT_CONFIGURE:
					id = 49;
					break;
				case MAV_CMD.MAV_CMD_DO_MOUNT_CONTROL:
					id = 50;
					break;
				case MAV_CMD.MAV_CMD_DO_SET_CAM_TRIGG_DIST:
					id = 51;
					break;
				case MAV_CMD.MAV_CMD_DO_FENCE_ENABLE:
					id = 52;
					break;
				case MAV_CMD.MAV_CMD_DO_PARACHUTE:
					id = 53;
					break;
				case MAV_CMD.MAV_CMD_DO_MOTOR_TEST:
					id = 54;
					break;
				case MAV_CMD.MAV_CMD_DO_INVERTED_FLIGHT:
					id = 55;
					break;
				case MAV_CMD.MAV_CMD_NAV_SET_YAW_SPEED:
					id = 56;
					break;
				case MAV_CMD.MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL:
					id = 57;
					break;
				case MAV_CMD.MAV_CMD_DO_MOUNT_CONTROL_QUAT:
					id = 58;
					break;
				case MAV_CMD.MAV_CMD_DO_GUIDED_MASTER:
					id = 59;
					break;
				case MAV_CMD.MAV_CMD_DO_GUIDED_LIMITS:
					id = 60;
					break;
				case MAV_CMD.MAV_CMD_DO_ENGINE_CONTROL:
					id = 61;
					break;
				case MAV_CMD.MAV_CMD_DO_LAST:
					id = 62;
					break;
				case MAV_CMD.MAV_CMD_PREFLIGHT_CALIBRATION:
					id = 63;
					break;
				case MAV_CMD.MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS:
					id = 64;
					break;
				case MAV_CMD.MAV_CMD_PREFLIGHT_UAVCAN:
					id = 65;
					break;
				case MAV_CMD.MAV_CMD_PREFLIGHT_STORAGE:
					id = 66;
					break;
				case MAV_CMD.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
					id = 67;
					break;
				case MAV_CMD.MAV_CMD_OVERRIDE_GOTO:
					id = 68;
					break;
				case MAV_CMD.MAV_CMD_MISSION_START:
					id = 69;
					break;
				case MAV_CMD.MAV_CMD_COMPONENT_ARM_DISARM:
					id = 70;
					break;
				case MAV_CMD.MAV_CMD_GET_HOME_POSITION:
					id = 71;
					break;
				case MAV_CMD.MAV_CMD_START_RX_PAIR:
					id = 72;
					break;
				case MAV_CMD.MAV_CMD_GET_MESSAGE_INTERVAL:
					id = 73;
					break;
				case MAV_CMD.MAV_CMD_SET_MESSAGE_INTERVAL:
					id = 74;
					break;
				case MAV_CMD.MAV_CMD_REQUEST_PROTOCOL_VERSION:
					id = 75;
					break;
				case MAV_CMD.MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES:
					id = 76;
					break;
				case MAV_CMD.MAV_CMD_REQUEST_CAMERA_INFORMATION:
					id = 77;
					break;
				case MAV_CMD.MAV_CMD_REQUEST_CAMERA_SETTINGS:
					id = 78;
					break;
				case MAV_CMD.MAV_CMD_REQUEST_STORAGE_INFORMATION:
					id = 79;
					break;
				case MAV_CMD.MAV_CMD_STORAGE_FORMAT:
					id = 80;
					break;
				case MAV_CMD.MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS:
					id = 81;
					break;
				case MAV_CMD.MAV_CMD_REQUEST_FLIGHT_INFORMATION:
					id = 82;
					break;
				case MAV_CMD.MAV_CMD_RESET_CAMERA_SETTINGS:
					id = 83;
					break;
				case MAV_CMD.MAV_CMD_SET_CAMERA_MODE:
					id = 84;
					break;
				case MAV_CMD.MAV_CMD_IMAGE_START_CAPTURE:
					id = 85;
					break;
				case MAV_CMD.MAV_CMD_IMAGE_STOP_CAPTURE:
					id = 86;
					break;
				case MAV_CMD.MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE:
					id = 87;
					break;
				case MAV_CMD.MAV_CMD_DO_TRIGGER_CONTROL:
					id = 88;
					break;
				case MAV_CMD.MAV_CMD_VIDEO_START_CAPTURE:
					id = 89;
					break;
				case MAV_CMD.MAV_CMD_VIDEO_STOP_CAPTURE:
					id = 90;
					break;
				case MAV_CMD.MAV_CMD_VIDEO_START_STREAMING:
					id = 91;
					break;
				case MAV_CMD.MAV_CMD_VIDEO_STOP_STREAMING:
					id = 92;
					break;
				case MAV_CMD.MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION:
					id = 93;
					break;
				case MAV_CMD.MAV_CMD_LOGGING_START:
					id = 94;
					break;
				case MAV_CMD.MAV_CMD_LOGGING_STOP:
					id = 95;
					break;
				case MAV_CMD.MAV_CMD_AIRFRAME_CONFIGURATION:
					id = 96;
					break;
				case MAV_CMD.MAV_CMD_PANORAMA_CREATE:
					id = 97;
					break;
				case MAV_CMD.MAV_CMD_DO_VTOL_TRANSITION:
					id = 98;
					break;
				case MAV_CMD.MAV_CMD_ARM_AUTHORIZATION_REQUEST:
					id = 99;
					break;
				case MAV_CMD.MAV_CMD_SET_GUIDED_SUBMODE_STANDARD:
					id = 100;
					break;
				case MAV_CMD.MAV_CMD_SET_GUIDED_SUBMODE_CIRCLE:
					id = 101;
					break;
				case MAV_CMD.MAV_CMD_CONDITION_GATE:
					id = 102;
					break;
				case MAV_CMD.MAV_CMD_NAV_FENCE_RETURN_POINT:
					id = 103;
					break;
				case MAV_CMD.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION:
					id = 104;
					break;
				case MAV_CMD.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION:
					id = 105;
					break;
				case MAV_CMD.MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION:
					id = 106;
					break;
				case MAV_CMD.MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION:
					id = 107;
					break;
				case MAV_CMD.MAV_CMD_NAV_RALLY_POINT:
					id = 108;
					break;
				case MAV_CMD.MAV_CMD_UAVCAN_GET_NODE_INFO:
					id = 109;
					break;
				case MAV_CMD.MAV_CMD_PAYLOAD_PREPARE_DEPLOY:
					id = 110;
					break;
				case MAV_CMD.MAV_CMD_PAYLOAD_CONTROL_DEPLOY:
					id = 111;
					break;
				case MAV_CMD.MAV_CMD_WAYPOINT_USER_1:
					id = 112;
					break;
				case MAV_CMD.MAV_CMD_WAYPOINT_USER_2:
					id = 113;
					break;
				case MAV_CMD.MAV_CMD_WAYPOINT_USER_3:
					id = 114;
					break;
				case MAV_CMD.MAV_CMD_WAYPOINT_USER_4:
					id = 115;
					break;
				case MAV_CMD.MAV_CMD_WAYPOINT_USER_5:
					id = 116;
					break;
				case MAV_CMD.MAV_CMD_SPATIAL_USER_1:
					id = 117;
					break;
				case MAV_CMD.MAV_CMD_SPATIAL_USER_2:
					id = 118;
					break;
				case MAV_CMD.MAV_CMD_SPATIAL_USER_3:
					id = 119;
					break;
				case MAV_CMD.MAV_CMD_SPATIAL_USER_4:
					id = 120;
					break;
				case MAV_CMD.MAV_CMD_SPATIAL_USER_5:
					id = 121;
					break;
				case MAV_CMD.MAV_CMD_USER_1:
					id = 122;
					break;
				case MAV_CMD.MAV_CMD_USER_2:
					id = 123;
					break;
				case MAV_CMD.MAV_CMD_USER_3:
					id = 124;
					break;
				case MAV_CMD.MAV_CMD_USER_4:
					id = 125;
					break;
				case MAV_CMD.MAV_CMD_USER_5:
					id = 126;
					break;
				case MAV_CMD.MAV_CMD_RESET_MPPT:
					id = 127;
					break;
				case MAV_CMD.MAV_CMD_PAYLOAD_CONTROL:
					id = 128;
					break;
				default:
					assert (false);//("Unknown enum" + id);
			}
			set_bits(id, 8, data, 0);
		}

		public void result_SET(@MAV_RESULT int src) //See MAV_RESULT enum
		{ set_bits(-0 + src, 3, data, 8); }

		/**
		 * WIP: Also used as result_param1, it can be set with a enum containing the errors reasons of why the command
		 * was denied or the progress percentage or 255 if unknown the progress when result is MAV_RESULT_IN_PROGRESS
		 */
		public void progress_SET(char src, Bounds.Inside ph) {
			if (ph.field_bit != 11) insert_field(ph, 11, 0);
			set_bytes((char) (src) & -1L, 1, data, ph.BYTE);
		}

		/**
		 * WIP: Additional parameter of the result, example: which parameter of MAV_CMD_NAV_WAYPOINT caused it to
		 * be denied
		 */
		public void result_param2_SET(int src, Bounds.Inside ph) {
			if (ph.field_bit != 12) insert_field(ph, 12, 0);
			set_bytes((int) (src) & -1L, 4, data, ph.BYTE);
		}

		public void target_system_SET(char src, Bounds.Inside ph) //WIP: System which requested the command to be executed
		{
			if (ph.field_bit != 13) insert_field(ph, 13, 0);
			set_bytes((char) (src) & -1L, 1, data, ph.BYTE);
		}

		public void target_component_SET(char src, Bounds.Inside ph) //WIP: Component which requested the command to be executed
		{
			if (ph.field_bit != 14) insert_field(ph, 14, 0);
			set_bytes((char) (src) & -1L, 1, data, ph.BYTE);
		}
	}

	public static class BATTERY_STATUS extends GroundControl.BATTERY_STATUS {
		/**
		 * Battery voltage of cells, in millivolts (1 = 1 millivolt). Cells above the valid cell count for this battery
		 * should have the UINT16_MAX value
		 */
		public char[] voltages_GET(char[] dst_ch, int pos) {
			for (int BYTE = 0, dst_max = pos + 10; pos < dst_max; pos++, BYTE += 2)
				dst_ch[pos] = (char) ((char) get_bytes(data, BYTE, 2));
			return dst_ch;
		}

		/**
		 * Battery voltage of cells, in millivolts (1 = 1 millivolt). Cells above the valid cell count for this battery
		 * should have the UINT16_MAX value
		 */
		public char[] voltages_GET() {return voltages_GET(new char[10], 0);}

		public char id_GET()//Battery ID
		{ return (char) ((char) get_bytes(data, 20, 1)); }

		public short temperature_GET()//Temperature of the battery in centi-degrees celsius. INT16_MAX for unknown temperature.
		{ return (short) ((short) get_bytes(data, 21, 2)); }

		public short current_battery_GET()//Battery current, in 10*milliamperes (1 = 10 milliampere), -1: autopilot does not measure the curren
		{ return (short) ((short) get_bytes(data, 23, 2)); }

		public int current_consumed_GET()//Consumed charge, in milliampere hours (1 = 1 mAh), -1: autopilot does not provide mAh consumption estimat
		{ return (int) ((int) get_bytes(data, 25, 4)); }

		/**
		 * Consumed energy, in HectoJoules (intergrated U*I*dt)  (1 = 100 Joule), -1: autopilot does not provide
		 * energy consumption estimat
		 */
		public int energy_consumed_GET() { return (int) ((int) get_bytes(data, 29, 4)); }

		public byte battery_remaining_GET()//Remaining battery energy: (0%: 0, 100%: 100), -1: autopilot does not estimate the remaining battery
		{ return (byte) ((byte) get_bytes(data, 33, 1)); }

		public @MAV_BATTERY_FUNCTION int battery_function_GET()//Function of the battery
		{ return 0 + (int) get_bits(data, 272, 3); }

		public @MAV_BATTERY_TYPE int type_GET()//Type (chemistry) of the battery
		{ return 0 + (int) get_bits(data, 275, 3); }
	}

	public static class AUTOPILOT_VERSION extends GroundControl.AUTOPILOT_VERSION {
		public char vendor_id_GET()//ID of the board vendor
		{ return (char) ((char) get_bytes(data, 0, 2)); }

		public char product_id_GET()//ID of the product
		{ return (char) ((char) get_bytes(data, 2, 2)); }

		public long flight_sw_version_GET()//Firmware version number
		{ return (get_bytes(data, 4, 4)); }

		public long middleware_sw_version_GET()//Middleware version number
		{ return (get_bytes(data, 8, 4)); }

		public long os_sw_version_GET()//Operating system version number
		{ return (get_bytes(data, 12, 4)); }

		public long board_version_GET()//HW / board version (last 8 bytes should be silicon ID, if any)
		{ return (get_bytes(data, 16, 4)); }

		public long uid_GET()//UID if provided by hardware (see uid2)
		{ return (get_bytes(data, 20, 8)); }

		/**
		 * Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but
		 * should allow to identify the commit using the main version number even for very large code bases
		 */
		public char[] flight_custom_version_GET(char[] dst_ch, int pos) {
			for (int BYTE = 28, dst_max = pos + 8; pos < dst_max; pos++, BYTE += 1)
				dst_ch[pos] = (char) ((char) get_bytes(data, BYTE, 1));
			return dst_ch;
		}

		/**
		 * Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but
		 * should allow to identify the commit using the main version number even for very large code bases
		 */
		public char[] flight_custom_version_GET() {return flight_custom_version_GET(new char[8], 0);}

		/**
		 * Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but
		 * should allow to identify the commit using the main version number even for very large code bases
		 */
		public char[] middleware_custom_version_GET(char[] dst_ch, int pos) {
			for (int BYTE = 36, dst_max = pos + 8; pos < dst_max; pos++, BYTE += 1)
				dst_ch[pos] = (char) ((char) get_bytes(data, BYTE, 1));
			return dst_ch;
		}

		/**
		 * Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but
		 * should allow to identify the commit using the main version number even for very large code bases
		 */
		public char[] middleware_custom_version_GET() {return middleware_custom_version_GET(new char[8], 0);}

		/**
		 * Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but
		 * should allow to identify the commit using the main version number even for very large code bases
		 */
		public char[] os_custom_version_GET(char[] dst_ch, int pos) {
			for (int BYTE = 44, dst_max = pos + 8; pos < dst_max; pos++, BYTE += 1)
				dst_ch[pos] = (char) ((char) get_bytes(data, BYTE, 1));
			return dst_ch;
		}

		/**
		 * Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but
		 * should allow to identify the commit using the main version number even for very large code bases
		 */
		public char[] os_custom_version_GET() {return os_custom_version_GET(new char[8], 0);}

		public @MAV_PROTOCOL_CAPABILITY int capabilities_GET()//bitmask of capabilities (see MAV_PROTOCOL_CAPABILITY enum)
		{
			switch ((int) get_bits(data, 416, 5))
			{
				case 0:
					return MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT;
				case 1:
					return MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT;
				case 2:
					return MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_INT;
				case 3:
					return MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_COMMAND_INT;
				case 4:
					return MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_PARAM_UNION;
				case 5:
					return MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FTP;
				case 6:
					return MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET;
				case 7:
					return MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED;
				case 8:
					return MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT;
				case 9:
					return MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_TERRAIN;
				case 10:
					return MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_ACTUATOR_TARGET;
				case 11:
					return MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION;
				case 12:
					return MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION;
				case 13:
					return MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MAVLINK2;
				case 14:
					return MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_FENCE;
				case 15:
					return MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_RALLY;
				case 16:
					return MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FLIGHT_INFORMATION;
			}
			assert (false);//("Unknown enum ID " + id);
			return Integer.MIN_VALUE;
		}

		/**
		 * UID if provided by hardware (supersedes the uid field. If this is non-zero, use this field, otherwise
		 * use uid
		 */
		public char[] uid2_TRY(Bounds.Inside ph) {
			if (ph.field_bit != 421 && !try_visit_field(ph, 421)) return null;
			return uid2_GET(ph, new char[ph.items], 0);
		}

		/**
		 * UID if provided by hardware (supersedes the uid field. If this is non-zero, use this field, otherwise
		 * use uid
		 */
		public char[] uid2_GET(Bounds.Inside ph, char[] dst_ch, int pos) {
			for (int BYTE = ph.BYTE, dst_max = pos + 18; pos < dst_max; pos++, BYTE += 1)
				dst_ch[pos] = (char) ((char) get_bytes(data, BYTE, 1));
			return dst_ch;
		}

		public int uid2_LEN() {
			return 18;
		}
	}

	public static class LANDING_TARGET extends GroundControl.LANDING_TARGET {
		public long time_usec_GET()//Timestamp (micros since boot or Unix epoch)
		{ return (get_bytes(data, 0, 8)); }

		public char target_num_GET()//The ID of the target if multiple targets are present
		{ return (char) ((char) get_bytes(data, 8, 1)); }

		public float angle_x_GET()//X-axis angular offset (in radians) of the target from the center of the image
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 9, 4))); }

		public float angle_y_GET()//Y-axis angular offset (in radians) of the target from the center of the image
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 13, 4))); }

		public float distance_GET()//Distance to the target from the vehicle in meters
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 17, 4))); }

		public float size_x_GET()//Size in radians of target along x-axis
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 21, 4))); }

		public float size_y_GET()//Size in radians of target along y-axis
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 25, 4))); }

		public @MAV_FRAME int frame_GET()//MAV_FRAME enum specifying the whether the following feilds are earth-frame, body-frame, etc.
		{ return 0 + (int) get_bits(data, 232, 4); }

		public @LANDING_TARGET_TYPE int type_GET()//LANDING_TARGET_TYPE enum specifying the type of landing target
		{ return 0 + (int) get_bits(data, 236, 3); }

		public float x_TRY(Bounds.Inside ph)//X Position of the landing target on MAV_FRAME
		{
			if (ph.field_bit != 239 && !try_visit_field(ph, 239)) return 0;
			return (float) (Float.intBitsToFloat((int) get_bytes(data, ph.BYTE, 4)));
		}

		public float y_TRY(Bounds.Inside ph)//Y Position of the landing target on MAV_FRAME
		{
			if (ph.field_bit != 240 && !try_visit_field(ph, 240)) return 0;
			return (float) (Float.intBitsToFloat((int) get_bytes(data, ph.BYTE, 4)));
		}

		public float z_TRY(Bounds.Inside ph)//Z Position of the landing target on MAV_FRAME
		{
			if (ph.field_bit != 241 && !try_visit_field(ph, 241)) return 0;
			return (float) (Float.intBitsToFloat((int) get_bytes(data, ph.BYTE, 4)));
		}

		public float[] q_TRY(Bounds.Inside ph)//Quaternion of landing target orientation (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
		{
			if (ph.field_bit != 242 && !try_visit_field(ph, 242)) return null;
			return q_GET(ph, new float[ph.items], 0);
		}

		public float[] q_GET(Bounds.Inside ph, float[] dst_ch, int pos) //Quaternion of landing target orientation (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
		{
			for (int BYTE = ph.BYTE, dst_max = pos + 4; pos < dst_max; pos++, BYTE += 4)
				dst_ch[pos] = (float) (Float.intBitsToFloat((int) get_bytes(data, BYTE, 4)));
			return dst_ch;
		}

		public int q_LEN() {
			return 4;
		}

		/**
		 * Boolean indicating known position (1) or default unkown position (0), for validation of positioning of
		 * the landing targe
		 */
		public char position_valid_TRY(Bounds.Inside ph) {
			if (ph.field_bit != 243 && !try_visit_field(ph, 243)) return 0;
			return (char) ((char) get_bytes(data, ph.BYTE, 1));
		}
	}

	public static class SENS_POWER extends GroundControl.SENS_POWER {
		public float adc121_vspb_volt_GET()//Power board voltage sensor reading in volts
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 0, 4))); }

		public float adc121_cspb_amp_GET()//Power board current sensor reading in amps
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 4, 4))); }

		public float adc121_cs1_amp_GET()//Board current sensor 1 reading in amps
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 8, 4))); }

		public float adc121_cs2_amp_GET()//Board current sensor 2 reading in amps
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 12, 4))); }
	}

	public static class SENS_MPPT extends GroundControl.SENS_MPPT {
		public char mppt1_pwm_GET()//MPPT1 pwm
		{ return (char) ((char) get_bytes(data, 0, 2)); }

		public char mppt2_pwm_GET()//MPPT2 pwm
		{ return (char) ((char) get_bytes(data, 2, 2)); }

		public char mppt3_pwm_GET()//MPPT3 pwm
		{ return (char) ((char) get_bytes(data, 4, 2)); }

		public long mppt_timestamp_GET()//MPPT last timestamp
		{ return (get_bytes(data, 6, 8)); }

		public float mppt1_volt_GET()//MPPT1 voltage
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 14, 4))); }

		public float mppt1_amp_GET()//MPPT1 current
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 18, 4))); }

		public char mppt1_status_GET()//MPPT1 status
		{ return (char) ((char) get_bytes(data, 22, 1)); }

		public float mppt2_volt_GET()//MPPT2 voltage
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 23, 4))); }

		public float mppt2_amp_GET()//MPPT2 current
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 27, 4))); }

		public char mppt2_status_GET()//MPPT2 status
		{ return (char) ((char) get_bytes(data, 31, 1)); }

		public float mppt3_volt_GET()//MPPT3 voltage
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 32, 4))); }

		public float mppt3_amp_GET()//MPPT3 current
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 36, 4))); }

		public char mppt3_status_GET()//MPPT3 status
		{ return (char) ((char) get_bytes(data, 40, 1)); }
	}

	public static class ASLCTRL_DATA extends GroundControl.ASLCTRL_DATA {
		public long timestamp_GET()//Timestamp
		{ return (get_bytes(data, 0, 8)); }

		public char aslctrl_mode_GET()//ASLCTRL control-mode (manual, stabilized, auto, etc...)
		{ return (char) ((char) get_bytes(data, 8, 1)); }

		public float h_GET()//See sourcecode for a description of these values...
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 9, 4))); }

		public float hRef_GET()//null
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 13, 4))); }

		public float hRef_t_GET()//null
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 17, 4))); }

		public float PitchAngle_GET()//Pitch angle [deg]
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 21, 4))); }

		public float PitchAngleRef_GET()//Pitch angle reference[deg]
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 25, 4))); }

		public float q_GET()//null
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 29, 4))); }

		public float qRef_GET()//null
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 33, 4))); }

		public float uElev_GET()//null
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 37, 4))); }

		public float uThrot_GET()//null
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 41, 4))); }

		public float uThrot2_GET()//null
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 45, 4))); }

		public float nZ_GET()//null
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 49, 4))); }

		public float AirspeedRef_GET()//Airspeed reference [m/s]
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 53, 4))); }

		public char SpoilersEngaged_GET()//null
		{ return (char) ((char) get_bytes(data, 57, 1)); }

		public float YawAngle_GET()//Yaw angle [deg]
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 58, 4))); }

		public float YawAngleRef_GET()//Yaw angle reference[deg]
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 62, 4))); }

		public float RollAngle_GET()//Roll angle [deg]
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 66, 4))); }

		public float RollAngleRef_GET()//Roll angle reference[deg]
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 70, 4))); }

		public float p_GET()//null
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 74, 4))); }

		public float pRef_GET()//null
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 78, 4))); }

		public float r_GET()//null
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 82, 4))); }

		public float rRef_GET()//null
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 86, 4))); }

		public float uAil_GET()//null
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 90, 4))); }

		public float uRud_GET()//null
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 94, 4))); }
	}

	public static class ASLCTRL_DEBUG extends GroundControl.ASLCTRL_DEBUG {
		public long i32_1_GET()//Debug data
		{ return (get_bytes(data, 0, 4)); }

		public char i8_1_GET()//Debug data
		{ return (char) ((char) get_bytes(data, 4, 1)); }

		public char i8_2_GET()//Debug data
		{ return (char) ((char) get_bytes(data, 5, 1)); }

		public float f_1_GET()//Debug data
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 6, 4))); }

		public float f_2_GET()//Debug data
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 10, 4))); }

		public float f_3_GET()//Debug data
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 14, 4))); }

		public float f_4_GET()//Debug data
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 18, 4))); }

		public float f_5_GET()//Debug data
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 22, 4))); }

		public float f_6_GET()//Debug data
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 26, 4))); }

		public float f_7_GET()//Debug data
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 30, 4))); }

		public float f_8_GET()//Debug data
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 34, 4))); }
	}

	public static class ASLUAV_STATUS extends GroundControl.ASLUAV_STATUS {
		public char LED_status_GET()//Status of the position-indicator LEDs
		{ return (char) ((char) get_bytes(data, 0, 1)); }

		public char SATCOM_status_GET()//Status of the IRIDIUM satellite communication system
		{ return (char) ((char) get_bytes(data, 1, 1)); }

		public char[] Servo_status_GET(char[] dst_ch, int pos)  //Status vector for up to 8 servos
		{
			for (int BYTE = 2, dst_max = pos + 8; pos < dst_max; pos++, BYTE += 1)
				dst_ch[pos] = (char) ((char) get_bytes(data, BYTE, 1));
			return dst_ch;
		}

		public char[] Servo_status_GET()//Status vector for up to 8 servos
		{return Servo_status_GET(new char[8], 0);}

		public float Motor_rpm_GET()//Motor RPM
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 10, 4))); }
	}

	public static class EKF_EXT extends GroundControl.EKF_EXT {
		public long timestamp_GET()//Time since system start [us]
		{ return (get_bytes(data, 0, 8)); }

		public float Windspeed_GET()//Magnitude of wind velocity (in lateral inertial plane) [m/s]
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 8, 4))); }

		public float WindDir_GET()//Wind heading angle from North [rad]
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 12, 4))); }

		public float WindZ_GET()//Z (Down) component of inertial wind velocity [m/s]
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 16, 4))); }

		public float Airspeed_GET()//Magnitude of air velocity [m/s]
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 20, 4))); }

		public float beta_GET()//Sideslip angle [rad]
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 24, 4))); }

		public float alpha_GET()//Angle of attack [rad]
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 28, 4))); }
	}

	public static class ASL_OBCTRL extends GroundControl.ASL_OBCTRL {
		public long timestamp_GET()//Time since system start [us]
		{ return (get_bytes(data, 0, 8)); }

		public float uElev_GET()//Elevator command [~]
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 8, 4))); }

		public float uThrot_GET()//Throttle command [~]
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 12, 4))); }

		public float uThrot2_GET()//Throttle 2 command [~]
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 16, 4))); }

		public float uAilL_GET()//Left aileron command [~]
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 20, 4))); }

		public float uAilR_GET()//Right aileron command [~]
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 24, 4))); }

		public float uRud_GET()//Rudder command [~]
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 28, 4))); }

		public char obctrl_status_GET()//Off-board computer status
		{ return (char) ((char) get_bytes(data, 32, 1)); }
	}

	public static class SENS_ATMOS extends GroundControl.SENS_ATMOS {
		public float TempAmbient_GET()//Ambient temperature [degrees Celsius]
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 0, 4))); }

		public float Humidity_GET()//Relative humidity [%]
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 4, 4))); }
	}

	public static class SENS_BATMON extends GroundControl.SENS_BATMON {
		public char voltage_GET()//Battery pack voltage in [mV]
		{ return (char) ((char) get_bytes(data, 0, 2)); }

		public char batterystatus_GET()//Battery monitor status report bits in Hex
		{ return (char) ((char) get_bytes(data, 2, 2)); }

		public char serialnumber_GET()//Battery monitor serial number in Hex
		{ return (char) ((char) get_bytes(data, 4, 2)); }

		public char hostfetcontrol_GET()//Battery monitor sensor host FET control in Hex
		{ return (char) ((char) get_bytes(data, 6, 2)); }

		public char cellvoltage1_GET()//Battery pack cell 1 voltage in [mV]
		{ return (char) ((char) get_bytes(data, 8, 2)); }

		public char cellvoltage2_GET()//Battery pack cell 2 voltage in [mV]
		{ return (char) ((char) get_bytes(data, 10, 2)); }

		public char cellvoltage3_GET()//Battery pack cell 3 voltage in [mV]
		{ return (char) ((char) get_bytes(data, 12, 2)); }

		public char cellvoltage4_GET()//Battery pack cell 4 voltage in [mV]
		{ return (char) ((char) get_bytes(data, 14, 2)); }

		public char cellvoltage5_GET()//Battery pack cell 5 voltage in [mV]
		{ return (char) ((char) get_bytes(data, 16, 2)); }

		public char cellvoltage6_GET()//Battery pack cell 6 voltage in [mV]
		{ return (char) ((char) get_bytes(data, 18, 2)); }

		public float temperature_GET()//Battery pack temperature in [deg C]
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 20, 4))); }

		public short current_GET()//Battery pack current in [mA]
		{ return (short) ((short) get_bytes(data, 24, 2)); }

		public char SoC_GET()//Battery pack state-of-charge
		{ return (char) ((char) get_bytes(data, 26, 1)); }
	}

	public static class FW_SOARING_DATA extends GroundControl.FW_SOARING_DATA {
		public long timestamp_GET()//Timestamp [ms]
		{ return (get_bytes(data, 0, 8)); }

		public long timestampModeChanged_GET()//Timestamp since last mode change[ms]
		{ return (get_bytes(data, 8, 8)); }

		public float xW_GET()//Thermal core updraft strength [m/s]
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 16, 4))); }

		public float xR_GET()//Thermal radius [m]
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 20, 4))); }

		public float xLat_GET()//Thermal center latitude [deg]
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 24, 4))); }

		public float xLon_GET()//Thermal center longitude [deg]
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 28, 4))); }

		public float VarW_GET()//Variance W
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 32, 4))); }

		public float VarR_GET()//Variance R
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 36, 4))); }

		public float VarLat_GET()//Variance Lat
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 40, 4))); }

		public float VarLon_GET()//Variance Lon
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 44, 4))); }

		public float LoiterRadius_GET()//Suggested loiter radius [m]
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 48, 4))); }

		public float LoiterDirection_GET()//Suggested loiter direction
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 52, 4))); }

		public float DistToSoarPoint_GET()//Distance to soar point [m]
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 56, 4))); }

		public float vSinkExp_GET()//Expected sink rate at current airspeed, roll and throttle [m/s]
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 60, 4))); }

		public float z1_LocalUpdraftSpeed_GET()//Measurement / updraft speed at current/local airplane position [m/s]
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 64, 4))); }

		public float z2_DeltaRoll_GET()//Measurement / roll angle tracking error [deg]
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 68, 4))); }

		public float z1_exp_GET()//Expected measurement 1
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 72, 4))); }

		public float z2_exp_GET()//Expected measurement 2
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 76, 4))); }

		public float ThermalGSNorth_GET()//Thermal drift (from estimator prediction step only) [m/s]
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 80, 4))); }

		public float ThermalGSEast_GET()//Thermal drift (from estimator prediction step only) [m/s]
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 84, 4))); }

		public float TSE_dot_GET()//Total specific energy change (filtered) [m/s]
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 88, 4))); }

		public float DebugVar1_GET()//Debug variable 1
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 92, 4))); }

		public float DebugVar2_GET()//Debug variable 2
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 96, 4))); }

		public char ControlMode_GET()//Control Mode [-]
		{ return (char) ((char) get_bytes(data, 100, 1)); }

		public char valid_GET()//Data valid [-]
		{ return (char) ((char) get_bytes(data, 101, 1)); }
	}

	public static class SENSORPOD_STATUS extends GroundControl.SENSORPOD_STATUS {
		public char free_space_GET()//Free space available in recordings directory in [Gb] * 1e2
		{ return (char) ((char) get_bytes(data, 0, 2)); }

		public long timestamp_GET()//Timestamp in linuxtime [ms] (since 1.1.1970)
		{ return (get_bytes(data, 2, 8)); }

		public char visensor_rate_1_GET()//Rate of ROS topic 1
		{ return (char) ((char) get_bytes(data, 10, 1)); }

		public char visensor_rate_2_GET()//Rate of ROS topic 2
		{ return (char) ((char) get_bytes(data, 11, 1)); }

		public char visensor_rate_3_GET()//Rate of ROS topic 3
		{ return (char) ((char) get_bytes(data, 12, 1)); }

		public char visensor_rate_4_GET()//Rate of ROS topic 4
		{ return (char) ((char) get_bytes(data, 13, 1)); }

		public char recording_nodes_count_GET()//Number of recording nodes
		{ return (char) ((char) get_bytes(data, 14, 1)); }

		public char cpu_temp_GET()//Temperature of sensorpod CPU in [deg C]
		{ return (char) ((char) get_bytes(data, 15, 1)); }
	}

	public static class SENS_POWER_BOARD extends GroundControl.SENS_POWER_BOARD {
		public long timestamp_GET()//Timestamp
		{ return (get_bytes(data, 0, 8)); }

		public char pwr_brd_status_GET()//Power board status register
		{ return (char) ((char) get_bytes(data, 8, 1)); }

		public char pwr_brd_led_status_GET()//Power board leds status
		{ return (char) ((char) get_bytes(data, 9, 1)); }

		public float pwr_brd_system_volt_GET()//Power board system voltage
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 10, 4))); }

		public float pwr_brd_servo_volt_GET()//Power board servo voltage
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 14, 4))); }

		public float pwr_brd_mot_l_amp_GET()//Power board left motor current sensor
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 18, 4))); }

		public float pwr_brd_mot_r_amp_GET()//Power board right motor current sensor
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 22, 4))); }

		public float pwr_brd_servo_1_amp_GET()//Power board servo1 current sensor
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 26, 4))); }

		public float pwr_brd_servo_2_amp_GET()//Power board servo1 current sensor
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 30, 4))); }

		public float pwr_brd_servo_3_amp_GET()//Power board servo1 current sensor
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 34, 4))); }

		public float pwr_brd_servo_4_amp_GET()//Power board servo1 current sensor
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 38, 4))); }

		public float pwr_brd_aux_amp_GET()//Power board aux current sensor
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 42, 4))); }
	}

	public static class ESTIMATOR_STATUS extends GroundControl.ESTIMATOR_STATUS {
		public long time_usec_GET()//Timestamp (micros since boot or Unix epoch)
		{ return (get_bytes(data, 0, 8)); }

		public float vel_ratio_GET()//Velocity innovation test ratio
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 8, 4))); }

		public float pos_horiz_ratio_GET()//Horizontal position innovation test ratio
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 12, 4))); }

		public float pos_vert_ratio_GET()//Vertical position innovation test ratio
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 16, 4))); }

		public float mag_ratio_GET()//Magnetometer innovation test ratio
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 20, 4))); }

		public float hagl_ratio_GET()//Height above terrain innovation test ratio
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 24, 4))); }

		public float tas_ratio_GET()//True airspeed innovation test ratio
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 28, 4))); }

		public float pos_horiz_accuracy_GET()//Horizontal position 1-STD accuracy relative to the EKF local origin (m)
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 32, 4))); }

		public float pos_vert_accuracy_GET()//Vertical position 1-STD accuracy relative to the EKF local origin (m)
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 36, 4))); }

		public @ESTIMATOR_STATUS_FLAGS int flags_GET()//Integer bitmask indicating which EKF outputs are valid. See definition for ESTIMATOR_STATUS_FLAGS.
		{
			switch ((int) get_bits(data, 320, 4))
			{
				case 0:
					return ESTIMATOR_STATUS_FLAGS.ESTIMATOR_ATTITUDE;
				case 1:
					return ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_HORIZ;
				case 2:
					return ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_VERT;
				case 3:
					return ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_HORIZ_REL;
				case 4:
					return ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_HORIZ_ABS;
				case 5:
					return ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_VERT_ABS;
				case 6:
					return ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_VERT_AGL;
				case 7:
					return ESTIMATOR_STATUS_FLAGS.ESTIMATOR_CONST_POS_MODE;
				case 8:
					return ESTIMATOR_STATUS_FLAGS.ESTIMATOR_PRED_POS_HORIZ_REL;
				case 9:
					return ESTIMATOR_STATUS_FLAGS.ESTIMATOR_PRED_POS_HORIZ_ABS;
				case 10:
					return ESTIMATOR_STATUS_FLAGS.ESTIMATOR_GPS_GLITCH;
			}
			assert (false);//("Unknown enum ID " + id);
			return Integer.MIN_VALUE;
		}
	}

	public static class WIND_COV extends GroundControl.WIND_COV {
		public long time_usec_GET()//Timestamp (micros since boot or Unix epoch)
		{ return (get_bytes(data, 0, 8)); }

		public float wind_x_GET()//Wind in X (NED) direction in m/s
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 8, 4))); }

		public float wind_y_GET()//Wind in Y (NED) direction in m/s
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 12, 4))); }

		public float wind_z_GET()//Wind in Z (NED) direction in m/s
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 16, 4))); }

		public float var_horiz_GET()//Variability of the wind in XY. RMS of a 1 Hz lowpassed wind estimate.
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 20, 4))); }

		public float var_vert_GET()//Variability of the wind in Z. RMS of a 1 Hz lowpassed wind estimate.
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 24, 4))); }

		public float wind_alt_GET()//AMSL altitude (m) this measurement was taken at
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 28, 4))); }

		public float horiz_accuracy_GET()//Horizontal speed 1-STD accuracy
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 32, 4))); }

		public float vert_accuracy_GET()//Vertical speed 1-STD accuracy
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 36, 4))); }
	}

	public static class GPS_INPUT extends GroundControl.GPS_INPUT {
		public char time_week_GET()//GPS week number
		{ return (char) ((char) get_bytes(data, 0, 2)); }

		public long time_week_ms_GET()//GPS time (milliseconds from start of GPS week)
		{ return (get_bytes(data, 2, 4)); }

		public long time_usec_GET()//Timestamp (micros since boot or Unix epoch)
		{ return (get_bytes(data, 6, 8)); }

		public char gps_id_GET()//ID of the GPS for multiple GPS inputs
		{ return (char) ((char) get_bytes(data, 14, 1)); }

		public char fix_type_GET()//0-1: no fix, 2: 2D fix, 3: 3D fix. 4: 3D with DGPS. 5: 3D with RTK
		{ return (char) ((char) get_bytes(data, 15, 1)); }

		public int lat_GET()//Latitude (WGS84), in degrees * 1E7
		{ return (int) ((int) get_bytes(data, 16, 4)); }

		public int lon_GET()//Longitude (WGS84), in degrees * 1E7
		{ return (int) ((int) get_bytes(data, 20, 4)); }

		public float alt_GET()//Altitude (AMSL, not WGS84), in m (positive for up)
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 24, 4))); }

		public float hdop_GET()//GPS HDOP horizontal dilution of position in m
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 28, 4))); }

		public float vdop_GET()//GPS VDOP vertical dilution of position in m
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 32, 4))); }

		public float vn_GET()//GPS velocity in m/s in NORTH direction in earth-fixed NED frame
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 36, 4))); }

		public float ve_GET()//GPS velocity in m/s in EAST direction in earth-fixed NED frame
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 40, 4))); }

		public float vd_GET()//GPS velocity in m/s in DOWN direction in earth-fixed NED frame
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 44, 4))); }

		public float speed_accuracy_GET()//GPS speed accuracy in m/s
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 48, 4))); }

		public float horiz_accuracy_GET()//GPS horizontal accuracy in m
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 52, 4))); }

		public float vert_accuracy_GET()//GPS vertical accuracy in m
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 56, 4))); }

		public char satellites_visible_GET()//Number of satellites visible.
		{ return (char) ((char) get_bytes(data, 60, 1)); }

		public @GPS_INPUT_IGNORE_FLAGS int ignore_flags_GET()//Flags indicating which fields to ignore (see GPS_INPUT_IGNORE_FLAGS enum).  All other fields must be provided
		{
			switch ((int) get_bits(data, 488, 4))
			{
				case 0:
					return GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_ALT;
				case 1:
					return GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_HDOP;
				case 2:
					return GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VDOP;
				case 3:
					return GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VEL_HORIZ;
				case 4:
					return GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VEL_VERT;
				case 5:
					return GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY;
				case 6:
					return GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY;
				case 7:
					return GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY;
			}
			assert (false);//("Unknown enum ID " + id);
			return Integer.MIN_VALUE;
		}
	}

	public static class GPS_RTCM_DATA extends GroundControl.GPS_RTCM_DATA {
		/**
		 * LSB: 1 means message is fragmented, next 2 bits are the fragment ID, the remaining 5 bits are used for
		 * the sequence ID. Messages are only to be flushed to the GPS when the entire message has been reconstructed
		 * on the autopilot. The fragment ID specifies which order the fragments should be assembled into a buffer,
		 * while the sequence ID is used to detect a mismatch between different buffers. The buffer is considered
		 * fully reconstructed when either all 4 fragments are present, or all the fragments before the first fragment
		 * with a non full payload is received. This management is used to ensure that normal GPS operation doesn't
		 * corrupt RTCM data, and to recover from a unreliable transport delivery order
		 */
		public char flags_GET() { return (char) ((char) get_bytes(data, 0, 1)); }

		public char len_GET()//data length
		{ return (char) ((char) get_bytes(data, 1, 1)); }

		public char[] data__GET(char[] dst_ch, int pos)  //RTCM message (may be fragmented)
		{
			for (int BYTE = 2, dst_max = pos + 180; pos < dst_max; pos++, BYTE += 1)
				dst_ch[pos] = (char) ((char) get_bytes(data, BYTE, 1));
			return dst_ch;
		}

		public char[] data__GET()//RTCM message (may be fragmented)
		{return data__GET(new char[180], 0);}
	}

	public static class HIGH_LATENCY extends GroundControl.HIGH_LATENCY {
		public char heading_GET()//heading (centidegrees)
		{ return (char) ((char) get_bytes(data, 0, 2)); }

		public char wp_distance_GET()//distance to target (meters)
		{ return (char) ((char) get_bytes(data, 2, 2)); }

		public long custom_mode_GET()//A bitfield for use for autopilot-specific flags.
		{ return (get_bytes(data, 4, 4)); }

		public short roll_GET()//roll (centidegrees)
		{ return (short) ((short) get_bytes(data, 8, 2)); }

		public short pitch_GET()//pitch (centidegrees)
		{ return (short) ((short) get_bytes(data, 10, 2)); }

		public byte throttle_GET()//throttle (percentage)
		{ return (byte) ((byte) get_bytes(data, 12, 1)); }

		public short heading_sp_GET()//heading setpoint (centidegrees)
		{ return (short) ((short) get_bytes(data, 13, 2)); }

		public int latitude_GET()//Latitude, expressed as degrees * 1E7
		{ return (int) ((int) get_bytes(data, 15, 4)); }

		public int longitude_GET()//Longitude, expressed as degrees * 1E7
		{ return (int) ((int) get_bytes(data, 19, 4)); }

		public short altitude_amsl_GET()//Altitude above mean sea level (meters)
		{ return (short) ((short) get_bytes(data, 23, 2)); }

		public short altitude_sp_GET()//Altitude setpoint relative to the home position (meters)
		{ return (short) ((short) get_bytes(data, 25, 2)); }

		public char airspeed_GET()//airspeed (m/s)
		{ return (char) ((char) get_bytes(data, 27, 1)); }

		public char airspeed_sp_GET()//airspeed setpoint (m/s)
		{ return (char) ((char) get_bytes(data, 28, 1)); }

		public char groundspeed_GET()//groundspeed (m/s)
		{ return (char) ((char) get_bytes(data, 29, 1)); }

		public byte climb_rate_GET()//climb rate (m/s)
		{ return (byte) ((byte) get_bytes(data, 30, 1)); }

		public char gps_nsat_GET()//Number of satellites visible. If unknown, set to 255
		{ return (char) ((char) get_bytes(data, 31, 1)); }

		public char battery_remaining_GET()//Remaining battery (percentage)
		{ return (char) ((char) get_bytes(data, 32, 1)); }

		public byte temperature_GET()//Autopilot temperature (degrees C)
		{ return (byte) ((byte) get_bytes(data, 33, 1)); }

		public byte temperature_air_GET()//Air temperature (degrees C) from airspeed sensor
		{ return (byte) ((byte) get_bytes(data, 34, 1)); }

		/**
		 * failsafe (each bit represents a failsafe where 0=ok, 1=failsafe active (bit0:RC, bit1:batt, bit2:GPS,
		 * bit3:GCS, bit4:fence
		 */
		public char failsafe_GET() { return (char) ((char) get_bytes(data, 35, 1)); }

		public char wp_num_GET()//current waypoint number
		{ return (char) ((char) get_bytes(data, 36, 1)); }

		public @MAV_MODE_FLAG int base_mode_GET()//System mode bitfield, see MAV_MODE_FLAG ENUM in mavlink/include/mavlink_types.h
		{
			switch ((int) get_bits(data, 296, 4))
			{
				case 0:
					return MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
				case 1:
					return MAV_MODE_FLAG.MAV_MODE_FLAG_TEST_ENABLED;
				case 2:
					return MAV_MODE_FLAG.MAV_MODE_FLAG_AUTO_ENABLED;
				case 3:
					return MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED;
				case 4:
					return MAV_MODE_FLAG.MAV_MODE_FLAG_STABILIZE_ENABLED;
				case 5:
					return MAV_MODE_FLAG.MAV_MODE_FLAG_HIL_ENABLED;
				case 6:
					return MAV_MODE_FLAG.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
				case 7:
					return MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED;
			}
			assert (false);//("Unknown enum ID " + id);
			return Integer.MIN_VALUE;
		}

		public @MAV_LANDED_STATE int landed_state_GET()//The landed state. Is set to MAV_LANDED_STATE_UNDEFINED if landed state is unknown.
		{ return 0 + (int) get_bits(data, 300, 3); }

		public @GPS_FIX_TYPE int gps_fix_type_GET()//See the GPS_FIX_TYPE enum.
		{ return 0 + (int) get_bits(data, 303, 4); }
	}

	public static class VIBRATION extends GroundControl.VIBRATION {
		public long clipping_0_GET()//first accelerometer clipping count
		{ return (get_bytes(data, 0, 4)); }

		public long clipping_1_GET()//second accelerometer clipping count
		{ return (get_bytes(data, 4, 4)); }

		public long clipping_2_GET()//third accelerometer clipping count
		{ return (get_bytes(data, 8, 4)); }

		public long time_usec_GET()//Timestamp (micros since boot or Unix epoch)
		{ return (get_bytes(data, 12, 8)); }

		public float vibration_x_GET()//Vibration levels on X-axis
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 20, 4))); }

		public float vibration_y_GET()//Vibration levels on Y-axis
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 24, 4))); }

		public float vibration_z_GET()//Vibration levels on Z-axis
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 28, 4))); }
	}

	public static class HOME_POSITION extends GroundControl.HOME_POSITION {
		public int latitude_GET()//Latitude (WGS84), in degrees * 1E7
		{ return (int) ((int) get_bytes(data, 0, 4)); }

		public int longitude_GET()//Longitude (WGS84, in degrees * 1E7
		{ return (int) ((int) get_bytes(data, 4, 4)); }

		public int altitude_GET()//Altitude (AMSL), in meters * 1000 (positive for up)
		{ return (int) ((int) get_bytes(data, 8, 4)); }

		public float x_GET()//Local X position of this position in the local coordinate frame
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 12, 4))); }

		public float y_GET()//Local Y position of this position in the local coordinate frame
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 16, 4))); }

		public float z_GET()//Local Z position of this position in the local coordinate frame
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 20, 4))); }

		/**
		 * World to surface normal and heading transformation of the takeoff position. Used to indicate the heading
		 * and slope of the groun
		 */
		public float[] q_GET(float[] dst_ch, int pos) {
			for (int BYTE = 24, dst_max = pos + 4; pos < dst_max; pos++, BYTE += 4)
				dst_ch[pos] = (float) (Float.intBitsToFloat((int) get_bytes(data, BYTE, 4)));
			return dst_ch;
		}

		/**
		 * World to surface normal and heading transformation of the takeoff position. Used to indicate the heading
		 * and slope of the groun
		 */
		public float[] q_GET() {return q_GET(new float[4], 0);}

		/**
		 * Local X position of the end of the approach vector. Multicopters should set this position based on their
		 * takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing
		 * fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened
		 * from the threshold / touchdown zone
		 */
		public float approach_x_GET() { return (float) (Float.intBitsToFloat((int) get_bytes(data, 40, 4))); }

		/**
		 * Local Y position of the end of the approach vector. Multicopters should set this position based on their
		 * takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing
		 * fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened
		 * from the threshold / touchdown zone
		 */
		public float approach_y_GET() { return (float) (Float.intBitsToFloat((int) get_bytes(data, 44, 4))); }

		/**
		 * Local Z position of the end of the approach vector. Multicopters should set this position based on their
		 * takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing
		 * fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened
		 * from the threshold / touchdown zone
		 */
		public float approach_z_GET() { return (float) (Float.intBitsToFloat((int) get_bytes(data, 48, 4))); }

		public long time_usec_TRY(Bounds.Inside ph)//Timestamp (microseconds since UNIX epoch or microseconds since system boot)
		{
			if (ph.field_bit != 416 && !try_visit_field(ph, 416)) return 0;
			return (get_bytes(data, ph.BYTE, 8));
		}
	}

	public static class SET_HOME_POSITION extends GroundControl.SET_HOME_POSITION {
		public char target_system_GET()//System ID.
		{ return (char) ((char) get_bytes(data, 0, 1)); }

		public int latitude_GET()//Latitude (WGS84), in degrees * 1E7
		{ return (int) ((int) get_bytes(data, 1, 4)); }

		public int longitude_GET()//Longitude (WGS84, in degrees * 1E7
		{ return (int) ((int) get_bytes(data, 5, 4)); }

		public int altitude_GET()//Altitude (AMSL), in meters * 1000 (positive for up)
		{ return (int) ((int) get_bytes(data, 9, 4)); }

		public float x_GET()//Local X position of this position in the local coordinate frame
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 13, 4))); }

		public float y_GET()//Local Y position of this position in the local coordinate frame
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 17, 4))); }

		public float z_GET()//Local Z position of this position in the local coordinate frame
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 21, 4))); }

		/**
		 * World to surface normal and heading transformation of the takeoff position. Used to indicate the heading
		 * and slope of the groun
		 */
		public float[] q_GET(float[] dst_ch, int pos) {
			for (int BYTE = 25, dst_max = pos + 4; pos < dst_max; pos++, BYTE += 4)
				dst_ch[pos] = (float) (Float.intBitsToFloat((int) get_bytes(data, BYTE, 4)));
			return dst_ch;
		}

		/**
		 * World to surface normal and heading transformation of the takeoff position. Used to indicate the heading
		 * and slope of the groun
		 */
		public float[] q_GET() {return q_GET(new float[4], 0);}

		/**
		 * Local X position of the end of the approach vector. Multicopters should set this position based on their
		 * takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing
		 * fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened
		 * from the threshold / touchdown zone
		 */
		public float approach_x_GET() { return (float) (Float.intBitsToFloat((int) get_bytes(data, 41, 4))); }

		/**
		 * Local Y position of the end of the approach vector. Multicopters should set this position based on their
		 * takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing
		 * fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened
		 * from the threshold / touchdown zone
		 */
		public float approach_y_GET() { return (float) (Float.intBitsToFloat((int) get_bytes(data, 45, 4))); }

		/**
		 * Local Z position of the end of the approach vector. Multicopters should set this position based on their
		 * takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing
		 * fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened
		 * from the threshold / touchdown zone
		 */
		public float approach_z_GET() { return (float) (Float.intBitsToFloat((int) get_bytes(data, 49, 4))); }

		public long time_usec_TRY(Bounds.Inside ph)//Timestamp (microseconds since UNIX epoch or microseconds since system boot)
		{
			if (ph.field_bit != 424 && !try_visit_field(ph, 424)) return 0;
			return (get_bytes(data, ph.BYTE, 8));
		}
	}

	public static class MESSAGE_INTERVAL extends GroundControl.MESSAGE_INTERVAL {
		public char message_id_GET()//The ID of the requested MAVLink message. v1.0 is limited to 254 messages.
		{ return (char) ((char) get_bytes(data, 0, 2)); }

		public int interval_us_GET()//0 indicates the interval at which it is sent.
		{ return (int) ((int) get_bytes(data, 2, 4)); }
	}

	public static class EXTENDED_SYS_STATE extends GroundControl.EXTENDED_SYS_STATE {
		public @MAV_VTOL_STATE int vtol_state_GET()//The VTOL state if applicable. Is set to MAV_VTOL_STATE_UNDEFINED if UAV is not in VTOL configuration
		{ return 0 + (int) get_bits(data, 0, 3); }

		public @MAV_LANDED_STATE int landed_state_GET()//The landed state. Is set to MAV_LANDED_STATE_UNDEFINED if landed state is unknown.
		{ return 0 + (int) get_bits(data, 3, 3); }
	}

	public static class ADSB_VEHICLE extends GroundControl.ADSB_VEHICLE {
		public char heading_GET()//Course over ground in centidegrees
		{ return (char) ((char) get_bytes(data, 0, 2)); }

		public char hor_velocity_GET()//The horizontal velocity in centimeters/second
		{ return (char) ((char) get_bytes(data, 2, 2)); }

		public char squawk_GET()//Squawk code
		{ return (char) ((char) get_bytes(data, 4, 2)); }

		public long ICAO_address_GET()//ICAO address
		{ return (get_bytes(data, 6, 4)); }

		public int lat_GET()//Latitude, expressed as degrees * 1E7
		{ return (int) ((int) get_bytes(data, 10, 4)); }

		public int lon_GET()//Longitude, expressed as degrees * 1E7
		{ return (int) ((int) get_bytes(data, 14, 4)); }

		public int altitude_GET()//Altitude(ASL) in millimeters
		{ return (int) ((int) get_bytes(data, 18, 4)); }

		public short ver_velocity_GET()//The vertical velocity in centimeters/second, positive is up
		{ return (short) ((short) get_bytes(data, 22, 2)); }

		public char tslc_GET()//Time since last communication in seconds
		{ return (char) ((char) get_bytes(data, 24, 1)); }

		public @ADSB_ALTITUDE_TYPE int altitude_type_GET()//Type from ADSB_ALTITUDE_TYPE enum
		{ return 0 + (int) get_bits(data, 200, 2); }

		public @ADSB_EMITTER_TYPE int emitter_type_GET()//Type from ADSB_EMITTER_TYPE enum
		{ return 0 + (int) get_bits(data, 202, 5); }

		public @ADSB_FLAGS int flags_GET()//Flags to indicate various statuses including valid data fields
		{
			switch ((int) get_bits(data, 207, 3))
			{
				case 0:
					return ADSB_FLAGS.ADSB_FLAGS_VALID_COORDS;
				case 1:
					return ADSB_FLAGS.ADSB_FLAGS_VALID_ALTITUDE;
				case 2:
					return ADSB_FLAGS.ADSB_FLAGS_VALID_HEADING;
				case 3:
					return ADSB_FLAGS.ADSB_FLAGS_VALID_VELOCITY;
				case 4:
					return ADSB_FLAGS.ADSB_FLAGS_VALID_CALLSIGN;
				case 5:
					return ADSB_FLAGS.ADSB_FLAGS_VALID_SQUAWK;
				case 6:
					return ADSB_FLAGS.ADSB_FLAGS_SIMULATED;
			}
			assert (false);//("Unknown enum ID " + id);
			return Integer.MIN_VALUE;
		}

		public String callsign_TRY(Bounds.Inside ph)//The callsign, 8+null
		{
			if (ph.field_bit != 210 && !try_visit_field(ph, 210) || !try_visit_item(ph, 0)) return null;
			return new String(callsign_GET(ph, new char[ph.items], 0));
		}

		public char[] callsign_GET(Bounds.Inside ph, char[] dst_ch, int pos) //The callsign, 8+null
		{
			for (int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max; pos++, BYTE += 2)
				dst_ch[pos] = (char) ((char) get_bytes(data, BYTE, 2));
			return dst_ch;
		}

		public int callsign_LEN(Bounds.Inside ph) {
			return (ph.field_bit != 210 && !try_visit_field(ph, 210) || !try_visit_item(ph, 0)) ? 0 : ph.items;
		}
	}

	public static class COLLISION extends GroundControl.COLLISION {
		public long id_GET()//Unique identifier, domain based on src field
		{ return (get_bytes(data, 0, 4)); }

		public float time_to_minimum_delta_GET()//Estimated time until collision occurs (seconds)
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 4, 4))); }

		public float altitude_minimum_delta_GET()//Closest vertical distance in meters between vehicle and object
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 8, 4))); }

		public float horizontal_minimum_delta_GET()//Closest horizontal distance in meteres between vehicle and object
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 12, 4))); }

		public @MAV_COLLISION_SRC int src__GET()//Collision data source
		{ return 0 + (int) get_bits(data, 128, 2); }

		public @MAV_COLLISION_ACTION int action_GET()//Action that is being taken to avoid this collision
		{ return 0 + (int) get_bits(data, 130, 3); }

		public @MAV_COLLISION_THREAT_LEVEL int threat_level_GET()//How concerned the aircraft is about this collision
		{ return 0 + (int) get_bits(data, 133, 2); }
	}

	public static class V2_EXTENSION extends GroundControl.V2_EXTENSION {
		/**
		 * A code that identifies the software component that understands this message (analogous to usb device classes
		 * or mime type strings).  If this code is less than 32768, it is considered a 'registered' protocol extension
		 * and the corresponding entry should be added to https:github.com/mavlink/mavlink/extension-message-ids.xml.
		 * Software creators can register blocks of message IDs as needed (useful for GCS specific metadata, etc...).
		 * Message_types greater than 32767 are considered local experiments and should not be checked in to any
		 * widely distributed codebase
		 */
		public char message_type_GET() { return (char) ((char) get_bytes(data, 0, 2)); }

		public char target_network_GET()//Network ID (0 for broadcast)
		{ return (char) ((char) get_bytes(data, 2, 1)); }

		public char target_system_GET()//System ID (0 for broadcast)
		{ return (char) ((char) get_bytes(data, 3, 1)); }

		public char target_component_GET()//Component ID (0 for broadcast)
		{ return (char) ((char) get_bytes(data, 4, 1)); }

		/**
		 * Variable length payload. The length is defined by the remaining message length when subtracting the header
		 * and other fields.  The entire content of this block is opaque unless you understand any the encoding
		 * message_type.  The particular encoding used can be extension specific and might not always be documented
		 * as part of the mavlink specification
		 */
		public char[] payload_GET(char[] dst_ch, int pos) {
			for (int BYTE = 5, dst_max = pos + 249; pos < dst_max; pos++, BYTE += 1)
				dst_ch[pos] = (char) ((char) get_bytes(data, BYTE, 1));
			return dst_ch;
		}

		/**
		 * Variable length payload. The length is defined by the remaining message length when subtracting the header
		 * and other fields.  The entire content of this block is opaque unless you understand any the encoding
		 * message_type.  The particular encoding used can be extension specific and might not always be documented
		 * as part of the mavlink specification
		 */
		public char[] payload_GET() {return payload_GET(new char[249], 0);}
	}

	public static class MEMORY_VECT extends GroundControl.MEMORY_VECT {
		public char address_GET()//Starting address of the debug variables
		{ return (char) ((char) get_bytes(data, 0, 2)); }

		public char ver_GET()//Version code of the type variable. 0=unknown, type ignored and assumed short. 1=as below
		{ return (char) ((char) get_bytes(data, 2, 1)); }

		public char type_GET()//Type code of the memory variables. for ver = 1: 0=16 x short, 1=16 x char, 2=16 x Q15, 3=16 x 1Q1
		{ return (char) ((char) get_bytes(data, 3, 1)); }

		public byte[] value_GET(byte[] dst_ch, int pos)  //Memory contents at specified address
		{
			for (int BYTE = 4, dst_max = pos + 32; pos < dst_max; pos++, BYTE += 1)
				dst_ch[pos] = (byte) ((byte) get_bytes(data, BYTE, 1));
			return dst_ch;
		}

		public byte[] value_GET()//Memory contents at specified address
		{return value_GET(new byte[32], 0);}
	}

	public static class DEBUG_VECT extends GroundControl.DEBUG_VECT {
		public long time_usec_GET()//Timestamp
		{ return (get_bytes(data, 0, 8)); }

		public float x_GET()//x
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 8, 4))); }

		public float y_GET()//y
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 12, 4))); }

		public float z_GET()//z
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 16, 4))); }

		public String name_TRY(Bounds.Inside ph)//Name
		{
			if (ph.field_bit != 160 && !try_visit_field(ph, 160) || !try_visit_item(ph, 0)) return null;
			return new String(name_GET(ph, new char[ph.items], 0));
		}

		public char[] name_GET(Bounds.Inside ph, char[] dst_ch, int pos) //Name
		{
			for (int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max; pos++, BYTE += 2)
				dst_ch[pos] = (char) ((char) get_bytes(data, BYTE, 2));
			return dst_ch;
		}

		public int name_LEN(Bounds.Inside ph) {
			return (ph.field_bit != 160 && !try_visit_field(ph, 160) || !try_visit_item(ph, 0)) ? 0 : ph.items;
		}
	}

	public static class NAMED_VALUE_FLOAT extends GroundControl.NAMED_VALUE_FLOAT {
		public long time_boot_ms_GET()//Timestamp (milliseconds since system boot)
		{ return (get_bytes(data, 0, 4)); }

		public float value_GET()//Floating point value
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 4, 4))); }

		public String name_TRY(Bounds.Inside ph)//Name of the debug variable
		{
			if (ph.field_bit != 64 && !try_visit_field(ph, 64) || !try_visit_item(ph, 0)) return null;
			return new String(name_GET(ph, new char[ph.items], 0));
		}

		public char[] name_GET(Bounds.Inside ph, char[] dst_ch, int pos) //Name of the debug variable
		{
			for (int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max; pos++, BYTE += 2)
				dst_ch[pos] = (char) ((char) get_bytes(data, BYTE, 2));
			return dst_ch;
		}

		public int name_LEN(Bounds.Inside ph) {
			return (ph.field_bit != 64 && !try_visit_field(ph, 64) || !try_visit_item(ph, 0)) ? 0 : ph.items;
		}
	}

	public static class NAMED_VALUE_INT extends GroundControl.NAMED_VALUE_INT {
		public long time_boot_ms_GET()//Timestamp (milliseconds since system boot)
		{ return (get_bytes(data, 0, 4)); }

		public int value_GET()//Signed integer value
		{ return (int) ((int) get_bytes(data, 4, 4)); }

		public String name_TRY(Bounds.Inside ph)//Name of the debug variable
		{
			if (ph.field_bit != 64 && !try_visit_field(ph, 64) || !try_visit_item(ph, 0)) return null;
			return new String(name_GET(ph, new char[ph.items], 0));
		}

		public char[] name_GET(Bounds.Inside ph, char[] dst_ch, int pos) //Name of the debug variable
		{
			for (int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max; pos++, BYTE += 2)
				dst_ch[pos] = (char) ((char) get_bytes(data, BYTE, 2));
			return dst_ch;
		}

		public int name_LEN(Bounds.Inside ph) {
			return (ph.field_bit != 64 && !try_visit_field(ph, 64) || !try_visit_item(ph, 0)) ? 0 : ph.items;
		}
	}

	public static class STATUSTEXT extends GroundControl.STATUSTEXT {
		public @MAV_SEVERITY int severity_GET()//Severity of status. Relies on the definitions within RFC-5424. See enum MAV_SEVERITY.
		{ return 0 + (int) get_bits(data, 0, 4); }

		public String text_TRY(Bounds.Inside ph)//Status text message, without null termination character
		{
			if (ph.field_bit != 4 && !try_visit_field(ph, 4) || !try_visit_item(ph, 0)) return null;
			return new String(text_GET(ph, new char[ph.items], 0));
		}

		public char[] text_GET(Bounds.Inside ph, char[] dst_ch, int pos) //Status text message, without null termination character
		{
			for (int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max; pos++, BYTE += 2)
				dst_ch[pos] = (char) ((char) get_bytes(data, BYTE, 2));
			return dst_ch;
		}

		public int text_LEN(Bounds.Inside ph) {
			return (ph.field_bit != 4 && !try_visit_field(ph, 4) || !try_visit_item(ph, 0)) ? 0 : ph.items;
		}
	}

	public static class DEBUG extends GroundControl.DEBUG {
		public long time_boot_ms_GET()//Timestamp (milliseconds since system boot)
		{ return (get_bytes(data, 0, 4)); }

		public char ind_GET()//index of debug variable
		{ return (char) ((char) get_bytes(data, 4, 1)); }

		public float value_GET()//DEBUG value
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 5, 4))); }
	}

	public static class SETUP_SIGNING extends GroundControl.SETUP_SIGNING {
		public long initial_timestamp_GET()//initial timestamp
		{ return (get_bytes(data, 0, 8)); }

		public char target_system_GET()//system id of the target
		{ return (char) ((char) get_bytes(data, 8, 1)); }

		public char target_component_GET()//component ID of the target
		{ return (char) ((char) get_bytes(data, 9, 1)); }

		public char[] secret_key_GET(char[] dst_ch, int pos)  //signing key
		{
			for (int BYTE = 10, dst_max = pos + 32; pos < dst_max; pos++, BYTE += 1)
				dst_ch[pos] = (char) ((char) get_bytes(data, BYTE, 1));
			return dst_ch;
		}

		public char[] secret_key_GET()//signing key
		{return secret_key_GET(new char[32], 0);}
	}

	public static class BUTTON_CHANGE extends GroundControl.BUTTON_CHANGE {
		public long time_boot_ms_GET()//Timestamp (milliseconds since system boot)
		{ return (get_bytes(data, 0, 4)); }

		public long last_change_ms_GET()//Time of last change of button state
		{ return (get_bytes(data, 4, 4)); }

		public char state_GET()//Bitmap state of buttons
		{ return (char) ((char) get_bytes(data, 8, 1)); }
	}

	public static class PLAY_TUNE extends GroundControl.PLAY_TUNE {
		public char target_system_GET()//System ID
		{ return (char) ((char) get_bytes(data, 0, 1)); }

		public char target_component_GET()//Component ID
		{ return (char) ((char) get_bytes(data, 1, 1)); }

		public String tune_TRY(Bounds.Inside ph)//tune in board specific format
		{
			if (ph.field_bit != 16 && !try_visit_field(ph, 16) || !try_visit_item(ph, 0)) return null;
			return new String(tune_GET(ph, new char[ph.items], 0));
		}

		public char[] tune_GET(Bounds.Inside ph, char[] dst_ch, int pos) //tune in board specific format
		{
			for (int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max; pos++, BYTE += 2)
				dst_ch[pos] = (char) ((char) get_bytes(data, BYTE, 2));
			return dst_ch;
		}

		public int tune_LEN(Bounds.Inside ph) {
			return (ph.field_bit != 16 && !try_visit_field(ph, 16) || !try_visit_item(ph, 0)) ? 0 : ph.items;
		}
	}

	public static class CAMERA_INFORMATION extends GroundControl.CAMERA_INFORMATION {
		public char resolution_h_GET()//Image resolution in pixels horizontal
		{ return (char) ((char) get_bytes(data, 0, 2)); }

		public char resolution_v_GET()//Image resolution in pixels vertical
		{ return (char) ((char) get_bytes(data, 2, 2)); }

		public char cam_definition_version_GET()//Camera definition version (iteration)
		{ return (char) ((char) get_bytes(data, 4, 2)); }

		public long time_boot_ms_GET()//Timestamp (milliseconds since system boot)
		{ return (get_bytes(data, 6, 4)); }

		public long firmware_version_GET()//0xff = Major)
		{ return (get_bytes(data, 10, 4)); }

		public char[] vendor_name_GET(char[] dst_ch, int pos)  //Name of the camera vendor
		{
			for (int BYTE = 14, dst_max = pos + 32; pos < dst_max; pos++, BYTE += 1)
				dst_ch[pos] = (char) ((char) get_bytes(data, BYTE, 1));
			return dst_ch;
		}

		public char[] vendor_name_GET()//Name of the camera vendor
		{return vendor_name_GET(new char[32], 0);}

		public char[] model_name_GET(char[] dst_ch, int pos)  //Name of the camera model
		{
			for (int BYTE = 46, dst_max = pos + 32; pos < dst_max; pos++, BYTE += 1)
				dst_ch[pos] = (char) ((char) get_bytes(data, BYTE, 1));
			return dst_ch;
		}

		public char[] model_name_GET()//Name of the camera model
		{return model_name_GET(new char[32], 0);}

		public float focal_length_GET()//Focal length in mm
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 78, 4))); }

		public float sensor_size_h_GET()//Image sensor size horizontal in mm
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 82, 4))); }

		public float sensor_size_v_GET()//Image sensor size vertical in mm
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 86, 4))); }

		public char lens_id_GET()//Reserved for a lens ID
		{ return (char) ((char) get_bytes(data, 90, 1)); }

		public @CAMERA_CAP_FLAGS int flags_GET()//CAMERA_CAP_FLAGS enum flags (bitmap) describing camera capabilities.
		{
			switch ((int) get_bits(data, 728, 3))
			{
				case 0:
					return CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_VIDEO;
				case 1:
					return CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_IMAGE;
				case 2:
					return CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_HAS_MODES;
				case 3:
					return CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE;
				case 4:
					return CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE;
				case 5:
					return CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE;
			}
			assert (false);//("Unknown enum ID " + id);
			return Integer.MIN_VALUE;
		}

		public String cam_definition_uri_TRY(Bounds.Inside ph)//Camera definition URI (if any, otherwise only basic functions will be available).
		{
			if (ph.field_bit != 731 && !try_visit_field(ph, 731) || !try_visit_item(ph, 0)) return null;
			return new String(cam_definition_uri_GET(ph, new char[ph.items], 0));
		}

		public char[] cam_definition_uri_GET(Bounds.Inside ph, char[] dst_ch, int pos) //Camera definition URI (if any, otherwise only basic functions will be available).
		{
			for (int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max; pos++, BYTE += 2)
				dst_ch[pos] = (char) ((char) get_bytes(data, BYTE, 2));
			return dst_ch;
		}

		public int cam_definition_uri_LEN(Bounds.Inside ph) {
			return (ph.field_bit != 731 && !try_visit_field(ph, 731) || !try_visit_item(ph, 0)) ? 0 : ph.items;
		}
	}

	public static class CAMERA_SETTINGS extends GroundControl.CAMERA_SETTINGS {
		public long time_boot_ms_GET()//Timestamp (milliseconds since system boot)
		{ return (get_bytes(data, 0, 4)); }

		public @CAMERA_MODE int mode_id_GET()//Camera mode (CAMERA_MODE)
		{ return 0 + (int) get_bits(data, 32, 2); }
	}

	public static class STORAGE_INFORMATION extends GroundControl.STORAGE_INFORMATION {
		public long time_boot_ms_GET()//Timestamp (milliseconds since system boot)
		{ return (get_bytes(data, 0, 4)); }

		public char storage_id_GET()//Storage ID (1 for first, 2 for second, etc.)
		{ return (char) ((char) get_bytes(data, 4, 1)); }

		public char storage_count_GET()//Number of storage devices
		{ return (char) ((char) get_bytes(data, 5, 1)); }

		public char status_GET()//Status of storage (0 not available, 1 unformatted, 2 formatted)
		{ return (char) ((char) get_bytes(data, 6, 1)); }

		public float total_capacity_GET()//Total capacity in MiB
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 7, 4))); }

		public float used_capacity_GET()//Used capacity in MiB
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 11, 4))); }

		public float available_capacity_GET()//Available capacity in MiB
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 15, 4))); }

		public float read_speed_GET()//Read speed in MiB/s
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 19, 4))); }

		public float write_speed_GET()//Write speed in MiB/s
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 23, 4))); }
	}

	public static class CAMERA_CAPTURE_STATUS extends GroundControl.CAMERA_CAPTURE_STATUS {
		public long time_boot_ms_GET()//Timestamp (milliseconds since system boot)
		{ return (get_bytes(data, 0, 4)); }

		public long recording_time_ms_GET()//Time in milliseconds since recording started
		{ return (get_bytes(data, 4, 4)); }

		/**
		 * Current status of image capturing (0: idle, 1: capture in progress, 2: interval set but idle, 3: interval
		 * set and capture in progress
		 */
		public char image_status_GET() { return (char) ((char) get_bytes(data, 8, 1)); }

		public char video_status_GET()//Current status of video capturing (0: idle, 1: capture in progress)
		{ return (char) ((char) get_bytes(data, 9, 1)); }

		public float image_interval_GET()//Image capture interval in seconds
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 10, 4))); }

		public float available_capacity_GET()//Available storage capacity in MiB
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 14, 4))); }
	}

	public static class CAMERA_IMAGE_CAPTURED extends GroundControl.CAMERA_IMAGE_CAPTURED {
		public long time_boot_ms_GET()//Timestamp (milliseconds since system boot)
		{ return (get_bytes(data, 0, 4)); }

		public long time_utc_GET()//Timestamp (microseconds since UNIX epoch) in UTC. 0 for unknown.
		{ return (get_bytes(data, 4, 8)); }

		public char camera_id_GET()//Camera ID (1 for first, 2 for second, etc.)
		{ return (char) ((char) get_bytes(data, 12, 1)); }

		public int lat_GET()//Latitude, expressed as degrees * 1E7 where image was taken
		{ return (int) ((int) get_bytes(data, 13, 4)); }

		public int lon_GET()//Longitude, expressed as degrees * 1E7 where capture was taken
		{ return (int) ((int) get_bytes(data, 17, 4)); }

		public int alt_GET()//Altitude in meters, expressed as * 1E3 (AMSL, not WGS84) where image was taken
		{ return (int) ((int) get_bytes(data, 21, 4)); }

		public int relative_alt_GET()//Altitude above ground in meters, expressed as * 1E3 where image was taken
		{ return (int) ((int) get_bytes(data, 25, 4)); }

		public float[] q_GET(float[] dst_ch, int pos)  //Quaternion of camera orientation (w, x, y, z order, zero-rotation is 0, 0, 0, 0)
		{
			for (int BYTE = 29, dst_max = pos + 4; pos < dst_max; pos++, BYTE += 4)
				dst_ch[pos] = (float) (Float.intBitsToFloat((int) get_bytes(data, BYTE, 4)));
			return dst_ch;
		}

		public float[] q_GET()//Quaternion of camera orientation (w, x, y, z order, zero-rotation is 0, 0, 0, 0)
		{return q_GET(new float[4], 0);}

		public int image_index_GET()//Zero based index of this image (image count since armed -1)
		{ return (int) ((int) get_bytes(data, 45, 4)); }

		public byte capture_result_GET()//Boolean indicating success (1) or failure (0) while capturing this image.
		{ return (byte) ((byte) get_bytes(data, 49, 1)); }

		public String file_url_TRY(Bounds.Inside ph)//URL of image taken. Either local storage or http:foo.jpg if camera provides an HTTP interface.
		{
			if (ph.field_bit != 402 && !try_visit_field(ph, 402) || !try_visit_item(ph, 0)) return null;
			return new String(file_url_GET(ph, new char[ph.items], 0));
		}

		public char[] file_url_GET(Bounds.Inside ph, char[] dst_ch, int pos) //URL of image taken. Either local storage or http:foo.jpg if camera provides an HTTP interface.
		{
			for (int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max; pos++, BYTE += 2)
				dst_ch[pos] = (char) ((char) get_bytes(data, BYTE, 2));
			return dst_ch;
		}

		public int file_url_LEN(Bounds.Inside ph) {
			return (ph.field_bit != 402 && !try_visit_field(ph, 402) || !try_visit_item(ph, 0)) ? 0 : ph.items;
		}
	}

	public static class FLIGHT_INFORMATION extends GroundControl.FLIGHT_INFORMATION {
		public long time_boot_ms_GET()//Timestamp (milliseconds since system boot)
		{ return (get_bytes(data, 0, 4)); }

		public long arming_time_utc_GET()//Timestamp at arming (microseconds since UNIX epoch) in UTC, 0 for unknown
		{ return (get_bytes(data, 4, 8)); }

		public long takeoff_time_utc_GET()//Timestamp at takeoff (microseconds since UNIX epoch) in UTC, 0 for unknown
		{ return (get_bytes(data, 12, 8)); }

		public long flight_uuid_GET()//Universally unique identifier (UUID) of flight, should correspond to name of logfiles
		{ return (get_bytes(data, 20, 8)); }
	}

	public static class MOUNT_ORIENTATION extends GroundControl.MOUNT_ORIENTATION {
		public long time_boot_ms_GET()//Timestamp (milliseconds since system boot)
		{ return (get_bytes(data, 0, 4)); }

		public float roll_GET()//Roll in degrees
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 4, 4))); }

		public float pitch_GET()//Pitch in degrees
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 8, 4))); }

		public float yaw_GET()//Yaw in degrees
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 12, 4))); }
	}

	public static class LOGGING_DATA extends GroundControl.LOGGING_DATA {
		public char sequence_GET()//sequence number (can wrap)
		{ return (char) ((char) get_bytes(data, 0, 2)); }

		public char target_system_GET()//system ID of the target
		{ return (char) ((char) get_bytes(data, 2, 1)); }

		public char target_component_GET()//component ID of the target
		{ return (char) ((char) get_bytes(data, 3, 1)); }

		public char length_GET()//data length
		{ return (char) ((char) get_bytes(data, 4, 1)); }

		/**
		 * offset into data where first message starts. This can be used for recovery, when a previous message got
		 * lost (set to 255 if no start exists)
		 */
		public char first_message_offset_GET() { return (char) ((char) get_bytes(data, 5, 1)); }

		public char[] data__GET(char[] dst_ch, int pos)  //logged data
		{
			for (int BYTE = 6, dst_max = pos + 249; pos < dst_max; pos++, BYTE += 1)
				dst_ch[pos] = (char) ((char) get_bytes(data, BYTE, 1));
			return dst_ch;
		}

		public char[] data__GET()//logged data
		{return data__GET(new char[249], 0);}
	}

	public static class LOGGING_DATA_ACKED extends GroundControl.LOGGING_DATA_ACKED {
		public char sequence_GET()//sequence number (can wrap)
		{ return (char) ((char) get_bytes(data, 0, 2)); }

		public char target_system_GET()//system ID of the target
		{ return (char) ((char) get_bytes(data, 2, 1)); }

		public char target_component_GET()//component ID of the target
		{ return (char) ((char) get_bytes(data, 3, 1)); }

		public char length_GET()//data length
		{ return (char) ((char) get_bytes(data, 4, 1)); }

		/**
		 * offset into data where first message starts. This can be used for recovery, when a previous message got
		 * lost (set to 255 if no start exists)
		 */
		public char first_message_offset_GET() { return (char) ((char) get_bytes(data, 5, 1)); }

		public char[] data__GET(char[] dst_ch, int pos)  //logged data
		{
			for (int BYTE = 6, dst_max = pos + 249; pos < dst_max; pos++, BYTE += 1)
				dst_ch[pos] = (char) ((char) get_bytes(data, BYTE, 1));
			return dst_ch;
		}

		public char[] data__GET()//logged data
		{return data__GET(new char[249], 0);}
	}

	public static class LOGGING_ACK extends GroundControl.LOGGING_ACK {
		public char sequence_GET()//sequence number (must match the one in LOGGING_DATA_ACKED)
		{ return (char) ((char) get_bytes(data, 0, 2)); }

		public char target_system_GET()//system ID of the target
		{ return (char) ((char) get_bytes(data, 2, 1)); }

		public char target_component_GET()//component ID of the target
		{ return (char) ((char) get_bytes(data, 3, 1)); }
	}

	public static class VIDEO_STREAM_INFORMATION extends GroundControl.VIDEO_STREAM_INFORMATION {
		public char resolution_h_GET()//Resolution horizontal in pixels
		{ return (char) ((char) get_bytes(data, 0, 2)); }

		public char resolution_v_GET()//Resolution vertical in pixels
		{ return (char) ((char) get_bytes(data, 2, 2)); }

		public char rotation_GET()//Video image rotation clockwise
		{ return (char) ((char) get_bytes(data, 4, 2)); }

		public long bitrate_GET()//Bit rate in bits per second
		{ return (get_bytes(data, 6, 4)); }

		public char camera_id_GET()//Camera ID (1 for first, 2 for second, etc.)
		{ return (char) ((char) get_bytes(data, 10, 1)); }

		public char status_GET()//Current status of video streaming (0: not running, 1: in progress)
		{ return (char) ((char) get_bytes(data, 11, 1)); }

		public float framerate_GET()//Frames per second
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 12, 4))); }

		public String uri_TRY(Bounds.Inside ph)//Video stream URI
		{
			if (ph.field_bit != 130 && !try_visit_field(ph, 130) || !try_visit_item(ph, 0)) return null;
			return new String(uri_GET(ph, new char[ph.items], 0));
		}

		public char[] uri_GET(Bounds.Inside ph, char[] dst_ch, int pos) //Video stream URI
		{
			for (int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max; pos++, BYTE += 2)
				dst_ch[pos] = (char) ((char) get_bytes(data, BYTE, 2));
			return dst_ch;
		}

		public int uri_LEN(Bounds.Inside ph) {
			return (ph.field_bit != 130 && !try_visit_field(ph, 130) || !try_visit_item(ph, 0)) ? 0 : ph.items;
		}
	}

	public static class SET_VIDEO_STREAM_SETTINGS extends GroundControl.SET_VIDEO_STREAM_SETTINGS {
		public char resolution_h_GET()//Resolution horizontal in pixels (set to -1 for highest resolution possible)
		{ return (char) ((char) get_bytes(data, 0, 2)); }

		public char resolution_v_GET()//Resolution vertical in pixels (set to -1 for highest resolution possible)
		{ return (char) ((char) get_bytes(data, 2, 2)); }

		public char rotation_GET()//Video image rotation clockwise (0-359 degrees)
		{ return (char) ((char) get_bytes(data, 4, 2)); }

		public long bitrate_GET()//Bit rate in bits per second (set to -1 for auto)
		{ return (get_bytes(data, 6, 4)); }

		public char target_system_GET()//system ID of the target
		{ return (char) ((char) get_bytes(data, 10, 1)); }

		public char target_component_GET()//component ID of the target
		{ return (char) ((char) get_bytes(data, 11, 1)); }

		public char camera_id_GET()//Camera ID (1 for first, 2 for second, etc.)
		{ return (char) ((char) get_bytes(data, 12, 1)); }

		public float framerate_GET()//Frames per second (set to -1 for highest framerate possible)
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 13, 4))); }

		public String uri_TRY(Bounds.Inside ph)//Video stream URI
		{
			if (ph.field_bit != 138 && !try_visit_field(ph, 138) || !try_visit_item(ph, 0)) return null;
			return new String(uri_GET(ph, new char[ph.items], 0));
		}

		public char[] uri_GET(Bounds.Inside ph, char[] dst_ch, int pos) //Video stream URI
		{
			for (int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max; pos++, BYTE += 2)
				dst_ch[pos] = (char) ((char) get_bytes(data, BYTE, 2));
			return dst_ch;
		}

		public int uri_LEN(Bounds.Inside ph) {
			return (ph.field_bit != 138 && !try_visit_field(ph, 138) || !try_visit_item(ph, 0)) ? 0 : ph.items;
		}
	}

	public static class WIFI_CONFIG_AP extends GroundControl.WIFI_CONFIG_AP {
		public String ssid_TRY(Bounds.Inside ph)//Name of Wi-Fi network (SSID). Leave it blank to leave it unchanged.
		{
			if (ph.field_bit != 2 && !try_visit_field(ph, 2) || !try_visit_item(ph, 0)) return null;
			return new String(ssid_GET(ph, new char[ph.items], 0));
		}

		public char[] ssid_GET(Bounds.Inside ph, char[] dst_ch, int pos) //Name of Wi-Fi network (SSID). Leave it blank to leave it unchanged.
		{
			for (int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max; pos++, BYTE += 2)
				dst_ch[pos] = (char) ((char) get_bytes(data, BYTE, 2));
			return dst_ch;
		}

		public int ssid_LEN(Bounds.Inside ph) {
			return (ph.field_bit != 2 && !try_visit_field(ph, 2) || !try_visit_item(ph, 0)) ? 0 : ph.items;
		}

		public String password_TRY(Bounds.Inside ph)//Password. Leave it blank for an open AP.
		{
			if (ph.field_bit != 3 && !try_visit_field(ph, 3) || !try_visit_item(ph, 0)) return null;
			return new String(password_GET(ph, new char[ph.items], 0));
		}

		public char[] password_GET(Bounds.Inside ph, char[] dst_ch, int pos) //Password. Leave it blank for an open AP.
		{
			for (int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max; pos++, BYTE += 2)
				dst_ch[pos] = (char) ((char) get_bytes(data, BYTE, 2));
			return dst_ch;
		}

		public int password_LEN(Bounds.Inside ph) {
			return (ph.field_bit != 3 && !try_visit_field(ph, 3) || !try_visit_item(ph, 0)) ? 0 : ph.items;
		}
	}

	public static class PROTOCOL_VERSION extends GroundControl.PROTOCOL_VERSION {
		public char version_GET()//Currently active MAVLink version number * 100: v1.0 is 100, v2.0 is 200, etc.
		{ return (char) ((char) get_bytes(data, 0, 2)); }

		public char min_version_GET()//Minimum MAVLink version supported
		{ return (char) ((char) get_bytes(data, 2, 2)); }

		public char max_version_GET()//Maximum MAVLink version supported (set to the same value as version by default)
		{ return (char) ((char) get_bytes(data, 4, 2)); }

		public char[] spec_version_hash_GET(char[] dst_ch, int pos)  //The first 8 bytes (not characters printed in hex!) of the git hash.
		{
			for (int BYTE = 6, dst_max = pos + 8; pos < dst_max; pos++, BYTE += 1)
				dst_ch[pos] = (char) ((char) get_bytes(data, BYTE, 1));
			return dst_ch;
		}

		public char[] spec_version_hash_GET()//The first 8 bytes (not characters printed in hex!) of the git hash.
		{return spec_version_hash_GET(new char[8], 0);}

		public char[] library_version_hash_GET(char[] dst_ch, int pos)  //The first 8 bytes (not characters printed in hex!) of the git hash.
		{
			for (int BYTE = 14, dst_max = pos + 8; pos < dst_max; pos++, BYTE += 1)
				dst_ch[pos] = (char) ((char) get_bytes(data, BYTE, 1));
			return dst_ch;
		}

		public char[] library_version_hash_GET()//The first 8 bytes (not characters printed in hex!) of the git hash.
		{return library_version_hash_GET(new char[8], 0);}
	}

	public static class UAVCAN_NODE_STATUS extends GroundControl.UAVCAN_NODE_STATUS {
		public char vendor_specific_status_code_GET()//Vendor-specific status information.
		{ return (char) ((char) get_bytes(data, 0, 2)); }

		public long uptime_sec_GET()//The number of seconds since the start-up of the node.
		{ return (get_bytes(data, 2, 4)); }

		public long time_usec_GET()//Timestamp (microseconds since UNIX epoch or microseconds since system boot)
		{ return (get_bytes(data, 6, 8)); }

		public char sub_mode_GET()//Not used currently.
		{ return (char) ((char) get_bytes(data, 14, 1)); }

		public @UAVCAN_NODE_HEALTH int health_GET()//Generalized node health status.
		{ return 0 + (int) get_bits(data, 120, 3); }

		public @UAVCAN_NODE_MODE int mode_GET()//Generalized operating mode.
		{
			switch ((int) get_bits(data, 123, 3))
			{
				case 0:
					return UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_OPERATIONAL;
				case 1:
					return UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_INITIALIZATION;
				case 2:
					return UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_MAINTENANCE;
				case 3:
					return UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_SOFTWARE_UPDATE;
				case 4:
					return UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_OFFLINE;
			}
			assert (false);//("Unknown enum ID " + id);
			return Integer.MIN_VALUE;
		}
	}

	public static class UAVCAN_NODE_INFO extends GroundControl.UAVCAN_NODE_INFO {
		public long uptime_sec_GET()//The number of seconds since the start-up of the node.
		{ return (get_bytes(data, 0, 4)); }

		public long sw_vcs_commit_GET()//Version control system (VCS) revision identifier (e.g. git short commit hash). Zero if unknown.
		{ return (get_bytes(data, 4, 4)); }

		public long time_usec_GET()//Timestamp (microseconds since UNIX epoch or microseconds since system boot)
		{ return (get_bytes(data, 8, 8)); }

		public char hw_version_major_GET()//Hardware major version number.
		{ return (char) ((char) get_bytes(data, 16, 1)); }

		public char hw_version_minor_GET()//Hardware minor version number.
		{ return (char) ((char) get_bytes(data, 17, 1)); }

		public char[] hw_unique_id_GET(char[] dst_ch, int pos)  //Hardware unique 128-bit ID.
		{
			for (int BYTE = 18, dst_max = pos + 16; pos < dst_max; pos++, BYTE += 1)
				dst_ch[pos] = (char) ((char) get_bytes(data, BYTE, 1));
			return dst_ch;
		}

		public char[] hw_unique_id_GET()//Hardware unique 128-bit ID.
		{return hw_unique_id_GET(new char[16], 0);}

		public char sw_version_major_GET()//Software major version number.
		{ return (char) ((char) get_bytes(data, 34, 1)); }

		public char sw_version_minor_GET()//Software minor version number.
		{ return (char) ((char) get_bytes(data, 35, 1)); }

		public String name_TRY(Bounds.Inside ph)//Node name string. For example, "sapog.px4.io".
		{
			if (ph.field_bit != 288 && !try_visit_field(ph, 288) || !try_visit_item(ph, 0)) return null;
			return new String(name_GET(ph, new char[ph.items], 0));
		}

		public char[] name_GET(Bounds.Inside ph, char[] dst_ch, int pos) //Node name string. For example, "sapog.px4.io".
		{
			for (int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max; pos++, BYTE += 2)
				dst_ch[pos] = (char) ((char) get_bytes(data, BYTE, 2));
			return dst_ch;
		}

		public int name_LEN(Bounds.Inside ph) {
			return (ph.field_bit != 288 && !try_visit_field(ph, 288) || !try_visit_item(ph, 0)) ? 0 : ph.items;
		}
	}

	public static class PARAM_EXT_REQUEST_READ extends GroundControl.PARAM_EXT_REQUEST_READ {
		public char target_system_GET()//System ID
		{ return (char) ((char) get_bytes(data, 0, 1)); }

		public char target_component_GET()//Component ID
		{ return (char) ((char) get_bytes(data, 1, 1)); }

		public short param_index_GET()//Parameter index. Set to -1 to use the Parameter ID field as identifier (else param_id will be ignored
		{ return (short) ((short) get_bytes(data, 2, 2)); }

		/**
		 * Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination
		 * (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the
		 * ID is stored as strin
		 */
		public String param_id_TRY(Bounds.Inside ph) {
			if (ph.field_bit != 32 && !try_visit_field(ph, 32) || !try_visit_item(ph, 0)) return null;
			return new String(param_id_GET(ph, new char[ph.items], 0));
		}

		/**
		 * Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination
		 * (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the
		 * ID is stored as strin
		 */
		public char[] param_id_GET(Bounds.Inside ph, char[] dst_ch, int pos) {
			for (int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max; pos++, BYTE += 2)
				dst_ch[pos] = (char) ((char) get_bytes(data, BYTE, 2));
			return dst_ch;
		}

		public int param_id_LEN(Bounds.Inside ph) {
			return (ph.field_bit != 32 && !try_visit_field(ph, 32) || !try_visit_item(ph, 0)) ? 0 : ph.items;
		}
	}

	public static class PARAM_EXT_REQUEST_LIST extends GroundControl.PARAM_EXT_REQUEST_LIST {
		public char target_system_GET()//System ID
		{ return (char) ((char) get_bytes(data, 0, 1)); }

		public char target_component_GET()//Component ID
		{ return (char) ((char) get_bytes(data, 1, 1)); }
	}

	public static class PARAM_EXT_VALUE extends GroundControl.PARAM_EXT_VALUE {
		public char param_count_GET()//Total number of parameters
		{ return (char) ((char) get_bytes(data, 0, 2)); }

		public char param_index_GET()//Index of this parameter
		{ return (char) ((char) get_bytes(data, 2, 2)); }

		public @MAV_PARAM_EXT_TYPE int param_type_GET()//Parameter type: see the MAV_PARAM_EXT_TYPE enum for supported data types.
		{ return 1 + (int) get_bits(data, 32, 4); }

		/**
		 * Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination
		 * (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the
		 * ID is stored as strin
		 */
		public String param_id_TRY(Bounds.Inside ph) {
			if (ph.field_bit != 38 && !try_visit_field(ph, 38) || !try_visit_item(ph, 0)) return null;
			return new String(param_id_GET(ph, new char[ph.items], 0));
		}

		/**
		 * Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination
		 * (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the
		 * ID is stored as strin
		 */
		public char[] param_id_GET(Bounds.Inside ph, char[] dst_ch, int pos) {
			for (int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max; pos++, BYTE += 2)
				dst_ch[pos] = (char) ((char) get_bytes(data, BYTE, 2));
			return dst_ch;
		}

		public int param_id_LEN(Bounds.Inside ph) {
			return (ph.field_bit != 38 && !try_visit_field(ph, 38) || !try_visit_item(ph, 0)) ? 0 : ph.items;
		}

		public String param_value_TRY(Bounds.Inside ph)//Parameter value
		{
			if (ph.field_bit != 39 && !try_visit_field(ph, 39) || !try_visit_item(ph, 0)) return null;
			return new String(param_value_GET(ph, new char[ph.items], 0));
		}

		public char[] param_value_GET(Bounds.Inside ph, char[] dst_ch, int pos) //Parameter value
		{
			for (int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max; pos++, BYTE += 2)
				dst_ch[pos] = (char) ((char) get_bytes(data, BYTE, 2));
			return dst_ch;
		}

		public int param_value_LEN(Bounds.Inside ph) {
			return (ph.field_bit != 39 && !try_visit_field(ph, 39) || !try_visit_item(ph, 0)) ? 0 : ph.items;
		}
	}

	public static class PARAM_EXT_SET extends GroundControl.PARAM_EXT_SET {
		public char target_system_GET()//System ID
		{ return (char) ((char) get_bytes(data, 0, 1)); }

		public char target_component_GET()//Component ID
		{ return (char) ((char) get_bytes(data, 1, 1)); }

		public @MAV_PARAM_EXT_TYPE int param_type_GET()//Parameter type: see the MAV_PARAM_EXT_TYPE enum for supported data types.
		{ return 1 + (int) get_bits(data, 16, 4); }

		/**
		 * Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination
		 * (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the
		 * ID is stored as strin
		 */
		public String param_id_TRY(Bounds.Inside ph) {
			if (ph.field_bit != 22 && !try_visit_field(ph, 22) || !try_visit_item(ph, 0)) return null;
			return new String(param_id_GET(ph, new char[ph.items], 0));
		}

		/**
		 * Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination
		 * (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the
		 * ID is stored as strin
		 */
		public char[] param_id_GET(Bounds.Inside ph, char[] dst_ch, int pos) {
			for (int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max; pos++, BYTE += 2)
				dst_ch[pos] = (char) ((char) get_bytes(data, BYTE, 2));
			return dst_ch;
		}

		public int param_id_LEN(Bounds.Inside ph) {
			return (ph.field_bit != 22 && !try_visit_field(ph, 22) || !try_visit_item(ph, 0)) ? 0 : ph.items;
		}

		public String param_value_TRY(Bounds.Inside ph)//Parameter value
		{
			if (ph.field_bit != 23 && !try_visit_field(ph, 23) || !try_visit_item(ph, 0)) return null;
			return new String(param_value_GET(ph, new char[ph.items], 0));
		}

		public char[] param_value_GET(Bounds.Inside ph, char[] dst_ch, int pos) //Parameter value
		{
			for (int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max; pos++, BYTE += 2)
				dst_ch[pos] = (char) ((char) get_bytes(data, BYTE, 2));
			return dst_ch;
		}

		public int param_value_LEN(Bounds.Inside ph) {
			return (ph.field_bit != 23 && !try_visit_field(ph, 23) || !try_visit_item(ph, 0)) ? 0 : ph.items;
		}
	}

	public static class PARAM_EXT_ACK extends GroundControl.PARAM_EXT_ACK {
		public @MAV_PARAM_EXT_TYPE int param_type_GET()//Parameter type: see the MAV_PARAM_EXT_TYPE enum for supported data types.
		{ return 1 + (int) get_bits(data, 0, 4); }

		public @PARAM_ACK int param_result_GET()//Result code: see the PARAM_ACK enum for possible codes.
		{ return 0 + (int) get_bits(data, 4, 3); }

		/**
		 * Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination
		 * (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the
		 * ID is stored as strin
		 */
		public String param_id_TRY(Bounds.Inside ph) {
			if (ph.field_bit != 9 && !try_visit_field(ph, 9) || !try_visit_item(ph, 0)) return null;
			return new String(param_id_GET(ph, new char[ph.items], 0));
		}

		/**
		 * Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination
		 * (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the
		 * ID is stored as strin
		 */
		public char[] param_id_GET(Bounds.Inside ph, char[] dst_ch, int pos) {
			for (int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max; pos++, BYTE += 2)
				dst_ch[pos] = (char) ((char) get_bytes(data, BYTE, 2));
			return dst_ch;
		}

		public int param_id_LEN(Bounds.Inside ph) {
			return (ph.field_bit != 9 && !try_visit_field(ph, 9) || !try_visit_item(ph, 0)) ? 0 : ph.items;
		}

		public String param_value_TRY(Bounds.Inside ph)//Parameter value (new value if PARAM_ACK_ACCEPTED, current value otherwise)
		{
			if (ph.field_bit != 10 && !try_visit_field(ph, 10) || !try_visit_item(ph, 0)) return null;
			return new String(param_value_GET(ph, new char[ph.items], 0));
		}

		public char[] param_value_GET(Bounds.Inside ph, char[] dst_ch, int pos) //Parameter value (new value if PARAM_ACK_ACCEPTED, current value otherwise)
		{
			for (int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max; pos++, BYTE += 2)
				dst_ch[pos] = (char) ((char) get_bytes(data, BYTE, 2));
			return dst_ch;
		}

		public int param_value_LEN(Bounds.Inside ph) {
			return (ph.field_bit != 10 && !try_visit_field(ph, 10) || !try_visit_item(ph, 0)) ? 0 : ph.items;
		}
	}

	public static class OBSTACLE_DISTANCE extends GroundControl.OBSTACLE_DISTANCE {
		/**
		 * Distance of obstacles in front of the sensor starting on the left side. A value of 0 means that the obstacle
		 * is right in front of the sensor. A value of max_distance +1 means no obstace is present. A value of UINT16_MAX
		 * for unknown/not used. In a array element, each unit corresponds to 1cm
		 */
		public char[] distances_GET(char[] dst_ch, int pos) {
			for (int BYTE = 0, dst_max = pos + 72; pos < dst_max; pos++, BYTE += 2)
				dst_ch[pos] = (char) ((char) get_bytes(data, BYTE, 2));
			return dst_ch;
		}

		/**
		 * Distance of obstacles in front of the sensor starting on the left side. A value of 0 means that the obstacle
		 * is right in front of the sensor. A value of max_distance +1 means no obstace is present. A value of UINT16_MAX
		 * for unknown/not used. In a array element, each unit corresponds to 1cm
		 */
		public char[] distances_GET() {return distances_GET(new char[72], 0);}

		public char min_distance_GET()//Minimum distance the sensor can measure in centimeters
		{ return (char) ((char) get_bytes(data, 144, 2)); }

		public char max_distance_GET()//Maximum distance the sensor can measure in centimeters
		{ return (char) ((char) get_bytes(data, 146, 2)); }

		public long time_usec_GET()//Timestamp (microseconds since system boot or since UNIX epoch)
		{ return (get_bytes(data, 148, 8)); }

		public char increment_GET()//Angular width in degrees of each array element.
		{ return (char) ((char) get_bytes(data, 156, 1)); }

		public @MAV_DISTANCE_SENSOR int sensor_type_GET()//Class id of the distance sensor type.
		{ return 0 + (int) get_bits(data, 1256, 3); }
	}

	@SuppressWarnings("unchecked")
	static class TestChannel extends Channel {
		static final TestChannel instance = new TestChannel(); //test channel

		public final java.io.InputStream  inputStream          = new InputStream();
		public final java.io.OutputStream outputStream         = new OutputStream();
		public final java.io.InputStream  inputStreamAdvanced  = new AdvancedInputStream();
		public final java.io.OutputStream outputStreamAdvanced = new AdvancedOutputStream();

		@Override protected void failure(String reason) {
			super.failure(reason);
			assert (false);
		}

		static final Collection<OnReceive.Handler<BATTERY_STATUS, Channel>>            on_BATTERY_STATUS            = new OnReceive<>();
		static final Collection<OnReceive.Handler<AUTOPILOT_VERSION, Channel>>         on_AUTOPILOT_VERSION         = new OnReceive<>();
		static final Collection<OnReceive.Handler<LANDING_TARGET, Channel>>            on_LANDING_TARGET            = new OnReceive<>();
		static final Collection<OnReceive.Handler<SENS_POWER, Channel>>                on_SENS_POWER                = new OnReceive<>();
		static final Collection<OnReceive.Handler<SENS_MPPT, Channel>>                 on_SENS_MPPT                 = new OnReceive<>();
		static final Collection<OnReceive.Handler<ASLCTRL_DATA, Channel>>              on_ASLCTRL_DATA              = new OnReceive<>();
		static final Collection<OnReceive.Handler<ASLCTRL_DEBUG, Channel>>             on_ASLCTRL_DEBUG             = new OnReceive<>();
		static final Collection<OnReceive.Handler<ASLUAV_STATUS, Channel>>             on_ASLUAV_STATUS             = new OnReceive<>();
		static final Collection<OnReceive.Handler<EKF_EXT, Channel>>                   on_EKF_EXT                   = new OnReceive<>();
		static final Collection<OnReceive.Handler<ASL_OBCTRL, Channel>>                on_ASL_OBCTRL                = new OnReceive<>();
		static final Collection<OnReceive.Handler<SENS_ATMOS, Channel>>                on_SENS_ATMOS                = new OnReceive<>();
		static final Collection<OnReceive.Handler<SENS_BATMON, Channel>>               on_SENS_BATMON               = new OnReceive<>();
		static final Collection<OnReceive.Handler<FW_SOARING_DATA, Channel>>           on_FW_SOARING_DATA           = new OnReceive<>();
		static final Collection<OnReceive.Handler<SENSORPOD_STATUS, Channel>>          on_SENSORPOD_STATUS          = new OnReceive<>();
		static final Collection<OnReceive.Handler<SENS_POWER_BOARD, Channel>>          on_SENS_POWER_BOARD          = new OnReceive<>();
		static final Collection<OnReceive.Handler<ESTIMATOR_STATUS, Channel>>          on_ESTIMATOR_STATUS          = new OnReceive<>();
		static final Collection<OnReceive.Handler<WIND_COV, Channel>>                  on_WIND_COV                  = new OnReceive<>();
		static final Collection<OnReceive.Handler<GPS_INPUT, Channel>>                 on_GPS_INPUT                 = new OnReceive<>();
		static final Collection<OnReceive.Handler<GPS_RTCM_DATA, Channel>>             on_GPS_RTCM_DATA             = new OnReceive<>();
		static final Collection<OnReceive.Handler<HIGH_LATENCY, Channel>>              on_HIGH_LATENCY              = new OnReceive<>();
		static final Collection<OnReceive.Handler<VIBRATION, Channel>>                 on_VIBRATION                 = new OnReceive<>();
		static final Collection<OnReceive.Handler<HOME_POSITION, Channel>>             on_HOME_POSITION             = new OnReceive<>();
		static final Collection<OnReceive.Handler<SET_HOME_POSITION, Channel>>         on_SET_HOME_POSITION         = new OnReceive<>();
		static final Collection<OnReceive.Handler<MESSAGE_INTERVAL, Channel>>          on_MESSAGE_INTERVAL          = new OnReceive<>();
		static final Collection<OnReceive.Handler<EXTENDED_SYS_STATE, Channel>>        on_EXTENDED_SYS_STATE        = new OnReceive<>();
		static final Collection<OnReceive.Handler<ADSB_VEHICLE, Channel>>              on_ADSB_VEHICLE              = new OnReceive<>();
		static final Collection<OnReceive.Handler<COLLISION, Channel>>                 on_COLLISION                 = new OnReceive<>();
		static final Collection<OnReceive.Handler<V2_EXTENSION, Channel>>              on_V2_EXTENSION              = new OnReceive<>();
		static final Collection<OnReceive.Handler<MEMORY_VECT, Channel>>               on_MEMORY_VECT               = new OnReceive<>();
		static final Collection<OnReceive.Handler<DEBUG_VECT, Channel>>                on_DEBUG_VECT                = new OnReceive<>();
		static final Collection<OnReceive.Handler<NAMED_VALUE_FLOAT, Channel>>         on_NAMED_VALUE_FLOAT         = new OnReceive<>();
		static final Collection<OnReceive.Handler<NAMED_VALUE_INT, Channel>>           on_NAMED_VALUE_INT           = new OnReceive<>();
		static final Collection<OnReceive.Handler<STATUSTEXT, Channel>>                on_STATUSTEXT                = new OnReceive<>();
		static final Collection<OnReceive.Handler<DEBUG, Channel>>                     on_DEBUG                     = new OnReceive<>();
		static final Collection<OnReceive.Handler<SETUP_SIGNING, Channel>>             on_SETUP_SIGNING             = new OnReceive<>();
		static final Collection<OnReceive.Handler<BUTTON_CHANGE, Channel>>             on_BUTTON_CHANGE             = new OnReceive<>();
		static final Collection<OnReceive.Handler<PLAY_TUNE, Channel>>                 on_PLAY_TUNE                 = new OnReceive<>();
		static final Collection<OnReceive.Handler<CAMERA_INFORMATION, Channel>>        on_CAMERA_INFORMATION        = new OnReceive<>();
		static final Collection<OnReceive.Handler<CAMERA_SETTINGS, Channel>>           on_CAMERA_SETTINGS           = new OnReceive<>();
		static final Collection<OnReceive.Handler<STORAGE_INFORMATION, Channel>>       on_STORAGE_INFORMATION       = new OnReceive<>();
		static final Collection<OnReceive.Handler<CAMERA_CAPTURE_STATUS, Channel>>     on_CAMERA_CAPTURE_STATUS     = new OnReceive<>();
		static final Collection<OnReceive.Handler<CAMERA_IMAGE_CAPTURED, Channel>>     on_CAMERA_IMAGE_CAPTURED     = new OnReceive<>();
		static final Collection<OnReceive.Handler<FLIGHT_INFORMATION, Channel>>        on_FLIGHT_INFORMATION        = new OnReceive<>();
		static final Collection<OnReceive.Handler<MOUNT_ORIENTATION, Channel>>         on_MOUNT_ORIENTATION         = new OnReceive<>();
		static final Collection<OnReceive.Handler<LOGGING_DATA, Channel>>              on_LOGGING_DATA              = new OnReceive<>();
		static final Collection<OnReceive.Handler<LOGGING_DATA_ACKED, Channel>>        on_LOGGING_DATA_ACKED        = new OnReceive<>();
		static final Collection<OnReceive.Handler<LOGGING_ACK, Channel>>               on_LOGGING_ACK               = new OnReceive<>();
		static final Collection<OnReceive.Handler<VIDEO_STREAM_INFORMATION, Channel>>  on_VIDEO_STREAM_INFORMATION  = new OnReceive<>();
		static final Collection<OnReceive.Handler<SET_VIDEO_STREAM_SETTINGS, Channel>> on_SET_VIDEO_STREAM_SETTINGS = new OnReceive<>();
		static final Collection<OnReceive.Handler<WIFI_CONFIG_AP, Channel>>            on_WIFI_CONFIG_AP            = new OnReceive<>();
		static final Collection<OnReceive.Handler<PROTOCOL_VERSION, Channel>>          on_PROTOCOL_VERSION          = new OnReceive<>();
		static final Collection<OnReceive.Handler<UAVCAN_NODE_STATUS, Channel>>        on_UAVCAN_NODE_STATUS        = new OnReceive<>();
		static final Collection<OnReceive.Handler<UAVCAN_NODE_INFO, Channel>>          on_UAVCAN_NODE_INFO          = new OnReceive<>();
		static final Collection<OnReceive.Handler<PARAM_EXT_REQUEST_READ, Channel>>    on_PARAM_EXT_REQUEST_READ    = new OnReceive<>();
		static final Collection<OnReceive.Handler<PARAM_EXT_REQUEST_LIST, Channel>>    on_PARAM_EXT_REQUEST_LIST    = new OnReceive<>();
		static final Collection<OnReceive.Handler<PARAM_EXT_VALUE, Channel>>           on_PARAM_EXT_VALUE           = new OnReceive<>();
		static final Collection<OnReceive.Handler<PARAM_EXT_SET, Channel>>             on_PARAM_EXT_SET             = new OnReceive<>();
		static final Collection<OnReceive.Handler<PARAM_EXT_ACK, Channel>>             on_PARAM_EXT_ACK             = new OnReceive<>();
		static final Collection<OnReceive.Handler<OBSTACLE_DISTANCE, Channel>>         on_OBSTACLE_DISTANCE         = new OnReceive<>();


		static Pack testing_pack; //one pack send/receive buffer

		void send(Pack pack) { testing_pack = pack; }

		final Bounds.Inside ph = new Bounds.Inside();

		@Override protected Pack process(Pack pack, int id) {
			switch (id)
			{
				case Channel.PROCESS_CHANNEL_REQEST:
					if (pack == null)
					{
						pack = testing_pack;
						testing_pack = null;
					}
					else testing_pack = pack;
					return pack;
				case Channel.PROCESS_RECEIVED:
					if (testing_pack == null) return null;
					ph.setPack(pack = testing_pack);
					testing_pack = null;
					id = pack.meta.id;
			}
			switch (id)
			{
				case 0:
					if (pack == null) return new HEARTBEAT();
					break;
				case 1:
					if (pack == null) return new SYS_STATUS();
					break;
				case 2:
					if (pack == null) return new SYSTEM_TIME();
					break;
				case 4:
					if (pack == null) return new PING();
					break;
				case 5:
					if (pack == null) return new CHANGE_OPERATOR_CONTROL();
					break;
				case 6:
					if (pack == null) return new CHANGE_OPERATOR_CONTROL_ACK();
					break;
				case 7:
					if (pack == null) return new AUTH_KEY();
					break;
				case 11:
					if (pack == null) return new SET_MODE();
					break;
				case 20:
					if (pack == null) return new PARAM_REQUEST_READ();
					break;
				case 21:
					if (pack == null) return new PARAM_REQUEST_LIST();
					break;
				case 22:
					if (pack == null) return new PARAM_VALUE();
					break;
				case 23:
					if (pack == null) return new PARAM_SET();
					break;
				case 24:
					if (pack == null) return new GPS_RAW_INT();
					break;
				case 25:
					if (pack == null) return new GPS_STATUS();
					break;
				case 26:
					if (pack == null) return new SCALED_IMU();
					break;
				case 27:
					if (pack == null) return new RAW_IMU();
					break;
				case 28:
					if (pack == null) return new RAW_PRESSURE();
					break;
				case 29:
					if (pack == null) return new SCALED_PRESSURE();
					break;
				case 30:
					if (pack == null) return new ATTITUDE();
					break;
				case 31:
					if (pack == null) return new ATTITUDE_QUATERNION();
					break;
				case 32:
					if (pack == null) return new LOCAL_POSITION_NED();
					break;
				case 33:
					if (pack == null) return new GLOBAL_POSITION_INT();
					break;
				case 34:
					if (pack == null) return new RC_CHANNELS_SCALED();
					break;
				case 35:
					if (pack == null) return new RC_CHANNELS_RAW();
					break;
				case 36:
					if (pack == null) return new SERVO_OUTPUT_RAW();
					break;
				case 37:
					if (pack == null) return new MISSION_REQUEST_PARTIAL_LIST();
					break;
				case 38:
					if (pack == null) return new MISSION_WRITE_PARTIAL_LIST();
					break;
				case 39:
					if (pack == null) return new MISSION_ITEM();
					break;
				case 40:
					if (pack == null) return new MISSION_REQUEST();
					break;
				case 41:
					if (pack == null) return new MISSION_SET_CURRENT();
					break;
				case 42:
					if (pack == null) return new MISSION_CURRENT();
					break;
				case 43:
					if (pack == null) return new MISSION_REQUEST_LIST();
					break;
				case 44:
					if (pack == null) return new MISSION_COUNT();
					break;
				case 45:
					if (pack == null) return new MISSION_CLEAR_ALL();
					break;
				case 46:
					if (pack == null) return new MISSION_ITEM_REACHED();
					break;
				case 47:
					if (pack == null) return new MISSION_ACK();
					break;
				case 48:
					if (pack == null) return new SET_GPS_GLOBAL_ORIGIN();
					break;
				case 49:
					if (pack == null) return new GPS_GLOBAL_ORIGIN();
					break;
				case 50:
					if (pack == null) return new PARAM_MAP_RC();
					break;
				case 51:
					if (pack == null) return new MISSION_REQUEST_INT();
					break;
				case 54:
					if (pack == null) return new SAFETY_SET_ALLOWED_AREA();
					break;
				case 55:
					if (pack == null) return new SAFETY_ALLOWED_AREA();
					break;
				case 61:
					if (pack == null) return new ATTITUDE_QUATERNION_COV();
					break;
				case 62:
					if (pack == null) return new NAV_CONTROLLER_OUTPUT();
					break;
				case 63:
					if (pack == null) return new GLOBAL_POSITION_INT_COV();
					break;
				case 64:
					if (pack == null) return new LOCAL_POSITION_NED_COV();
					break;
				case 65:
					if (pack == null) return new RC_CHANNELS();
					break;
				case 66:
					if (pack == null) return new REQUEST_DATA_STREAM();
					break;
				case 67:
					if (pack == null) return new DATA_STREAM();
					break;
				case 69:
					if (pack == null) return new MANUAL_CONTROL();
					break;
				case 70:
					if (pack == null) return new RC_CHANNELS_OVERRIDE();
					break;
				case 73:
					if (pack == null) return new MISSION_ITEM_INT();
					break;
				case 74:
					if (pack == null) return new VFR_HUD();
					break;
				case 75:
					if (pack == null) return new COMMAND_INT();
					break;
				case 76:
					if (pack == null) return new COMMAND_LONG();
					break;
				case 77:
					if (pack == null) return new COMMAND_ACK();
					break;
				case 147:
					if (pack == null) return new BATTERY_STATUS();
					((OnReceive) on_BATTERY_STATUS).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
					break;
				case 148:
					if (pack == null) return new AUTOPILOT_VERSION();
					((OnReceive) on_AUTOPILOT_VERSION).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
					break;
				case 149:
					if (pack == null) return new LANDING_TARGET();
					((OnReceive) on_LANDING_TARGET).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
					break;
				case 201:
					if (pack == null) return new SENS_POWER();
					((OnReceive) on_SENS_POWER).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
					break;
				case 202:
					if (pack == null) return new SENS_MPPT();
					((OnReceive) on_SENS_MPPT).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
					break;
				case 203:
					if (pack == null) return new ASLCTRL_DATA();
					((OnReceive) on_ASLCTRL_DATA).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
					break;
				case 204:
					if (pack == null) return new ASLCTRL_DEBUG();
					((OnReceive) on_ASLCTRL_DEBUG).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
					break;
				case 205:
					if (pack == null) return new ASLUAV_STATUS();
					((OnReceive) on_ASLUAV_STATUS).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
					break;
				case 206:
					if (pack == null) return new EKF_EXT();
					((OnReceive) on_EKF_EXT).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
					break;
				case 207:
					if (pack == null) return new ASL_OBCTRL();
					((OnReceive) on_ASL_OBCTRL).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
					break;
				case 208:
					if (pack == null) return new SENS_ATMOS();
					((OnReceive) on_SENS_ATMOS).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
					break;
				case 209:
					if (pack == null) return new SENS_BATMON();
					((OnReceive) on_SENS_BATMON).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
					break;
				case 210:
					if (pack == null) return new FW_SOARING_DATA();
					((OnReceive) on_FW_SOARING_DATA).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
					break;
				case 211:
					if (pack == null) return new SENSORPOD_STATUS();
					((OnReceive) on_SENSORPOD_STATUS).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
					break;
				case 212:
					if (pack == null) return new SENS_POWER_BOARD();
					((OnReceive) on_SENS_POWER_BOARD).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
					break;
				case 230:
					if (pack == null) return new ESTIMATOR_STATUS();
					((OnReceive) on_ESTIMATOR_STATUS).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
					break;
				case 231:
					if (pack == null) return new WIND_COV();
					((OnReceive) on_WIND_COV).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
					break;
				case 232:
					if (pack == null) return new GPS_INPUT();
					((OnReceive) on_GPS_INPUT).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
					break;
				case 233:
					if (pack == null) return new GPS_RTCM_DATA();
					((OnReceive) on_GPS_RTCM_DATA).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
					break;
				case 234:
					if (pack == null) return new HIGH_LATENCY();
					((OnReceive) on_HIGH_LATENCY).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
					break;
				case 241:
					if (pack == null) return new VIBRATION();
					((OnReceive) on_VIBRATION).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
					break;
				case 242:
					if (pack == null) return new HOME_POSITION();
					((OnReceive) on_HOME_POSITION).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
					break;
				case 243:
					if (pack == null) return new SET_HOME_POSITION();
					((OnReceive) on_SET_HOME_POSITION).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
					break;
				case 244:
					if (pack == null) return new MESSAGE_INTERVAL();
					((OnReceive) on_MESSAGE_INTERVAL).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
					break;
				case 245:
					if (pack == null) return new EXTENDED_SYS_STATE();
					((OnReceive) on_EXTENDED_SYS_STATE).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
					break;
				case 246:
					if (pack == null) return new ADSB_VEHICLE();
					((OnReceive) on_ADSB_VEHICLE).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
					break;
				case 247:
					if (pack == null) return new COLLISION();
					((OnReceive) on_COLLISION).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
					break;
				case 248:
					if (pack == null) return new V2_EXTENSION();
					((OnReceive) on_V2_EXTENSION).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
					break;
				case 249:
					if (pack == null) return new MEMORY_VECT();
					((OnReceive) on_MEMORY_VECT).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
					break;
				case 250:
					if (pack == null) return new DEBUG_VECT();
					((OnReceive) on_DEBUG_VECT).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
					break;
				case 251:
					if (pack == null) return new NAMED_VALUE_FLOAT();
					((OnReceive) on_NAMED_VALUE_FLOAT).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
					break;
				case 252:
					if (pack == null) return new NAMED_VALUE_INT();
					((OnReceive) on_NAMED_VALUE_INT).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
					break;
				case 253:
					if (pack == null) return new STATUSTEXT();
					((OnReceive) on_STATUSTEXT).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
					break;
				case 254:
					if (pack == null) return new DEBUG();
					((OnReceive) on_DEBUG).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
					break;
				case 256:
					if (pack == null) return new SETUP_SIGNING();
					((OnReceive) on_SETUP_SIGNING).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
					break;
				case 257:
					if (pack == null) return new BUTTON_CHANGE();
					((OnReceive) on_BUTTON_CHANGE).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
					break;
				case 258:
					if (pack == null) return new PLAY_TUNE();
					((OnReceive) on_PLAY_TUNE).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
					break;
				case 259:
					if (pack == null) return new CAMERA_INFORMATION();
					((OnReceive) on_CAMERA_INFORMATION).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
					break;
				case 260:
					if (pack == null) return new CAMERA_SETTINGS();
					((OnReceive) on_CAMERA_SETTINGS).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
					break;
				case 261:
					if (pack == null) return new STORAGE_INFORMATION();
					((OnReceive) on_STORAGE_INFORMATION).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
					break;
				case 262:
					if (pack == null) return new CAMERA_CAPTURE_STATUS();
					((OnReceive) on_CAMERA_CAPTURE_STATUS).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
					break;
				case 263:
					if (pack == null) return new CAMERA_IMAGE_CAPTURED();
					((OnReceive) on_CAMERA_IMAGE_CAPTURED).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
					break;
				case 264:
					if (pack == null) return new FLIGHT_INFORMATION();
					((OnReceive) on_FLIGHT_INFORMATION).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
					break;
				case 265:
					if (pack == null) return new MOUNT_ORIENTATION();
					((OnReceive) on_MOUNT_ORIENTATION).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
					break;
				case 266:
					if (pack == null) return new LOGGING_DATA();
					((OnReceive) on_LOGGING_DATA).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
					break;
				case 267:
					if (pack == null) return new LOGGING_DATA_ACKED();
					((OnReceive) on_LOGGING_DATA_ACKED).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
					break;
				case 268:
					if (pack == null) return new LOGGING_ACK();
					((OnReceive) on_LOGGING_ACK).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
					break;
				case 269:
					if (pack == null) return new VIDEO_STREAM_INFORMATION();
					((OnReceive) on_VIDEO_STREAM_INFORMATION).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
					break;
				case 270:
					if (pack == null) return new SET_VIDEO_STREAM_SETTINGS();
					((OnReceive) on_SET_VIDEO_STREAM_SETTINGS).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
					break;
				case 299:
					if (pack == null) return new WIFI_CONFIG_AP();
					((OnReceive) on_WIFI_CONFIG_AP).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
					break;
				case 300:
					if (pack == null) return new PROTOCOL_VERSION();
					((OnReceive) on_PROTOCOL_VERSION).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
					break;
				case 310:
					if (pack == null) return new UAVCAN_NODE_STATUS();
					((OnReceive) on_UAVCAN_NODE_STATUS).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
					break;
				case 311:
					if (pack == null) return new UAVCAN_NODE_INFO();
					((OnReceive) on_UAVCAN_NODE_INFO).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
					break;
				case 320:
					if (pack == null) return new PARAM_EXT_REQUEST_READ();
					((OnReceive) on_PARAM_EXT_REQUEST_READ).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
					break;
				case 321:
					if (pack == null) return new PARAM_EXT_REQUEST_LIST();
					((OnReceive) on_PARAM_EXT_REQUEST_LIST).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
					break;
				case 322:
					if (pack == null) return new PARAM_EXT_VALUE();
					((OnReceive) on_PARAM_EXT_VALUE).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
					break;
				case 323:
					if (pack == null) return new PARAM_EXT_SET();
					((OnReceive) on_PARAM_EXT_SET).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
					break;
				case 324:
					if (pack == null) return new PARAM_EXT_ACK();
					((OnReceive) on_PARAM_EXT_ACK).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
					break;
				case 330:
					if (pack == null) return new OBSTACLE_DISTANCE();
					((OnReceive) on_OBSTACLE_DISTANCE).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
					break;
			}
			return null;
		}

		static final byte[] buff = new byte[1024];

		static void transmission(java.io.InputStream src, java.io.OutputStream dst, Channel dst_ch) {
			try
			{
				if (src instanceof AdvancedInputStream && !(dst instanceof AdvancedOutputStream))
				{
					for (int bytes; 0 < (bytes = src.read(buff, 0, buff.length)); ) TestChannel.instance.outputStreamAdvanced.write(buff, 0, bytes);
					for (int bytes; 0 < (bytes = TestChannel.instance.inputStream.read(buff, 0, buff.length)); ) dst.write(buff, 0, bytes);
				}
				else
					if (!(src instanceof AdvancedInputStream) && dst instanceof AdvancedOutputStream)
					{
						for (int bytes; 0 < (bytes = src.read(buff, 0, buff.length)); ) TestChannel.instance.outputStream.write(buff, 0, bytes);
						for (int bytes; 0 < (bytes = TestChannel.instance.inputStreamAdvanced.read(buff, 0, buff.length)); ) dst.write(buff, 0, bytes);
					}
					else
						for (int bytes; 0 < (bytes = src.read(buff, 0, buff.length)); ) dst.write(buff, 0, bytes);
				processReceived(dst_ch);
			} catch (IOException e)
			{
				e.printStackTrace();
				assert (false);
			}
		}
	}

	public static void main(String[] args) {
		final Bounds.Inside PH = new Bounds.Inside();
		CommunicationChannel.instance.on_HEARTBEAT.add((src, ph, pack) ->
		{
			assert (pack.custom_mode_GET() == 4091110721L);
			assert (pack.autopilot_GET() == MAV_AUTOPILOT.MAV_AUTOPILOT_PPZ);
			assert (pack.type_GET() == MAV_TYPE.MAV_TYPE_HELICOPTER);
			assert (pack.base_mode_GET() == MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED);
			assert (pack.mavlink_version_GET() == (char) 57);
			assert (pack.system_status_GET() == MAV_STATE.MAV_STATE_ACTIVE);
		});
		HEARTBEAT p0 = new HEARTBEAT();
		PH.setPack(p0);
		p0.mavlink_version_SET((char) 57);
		p0.autopilot_SET(MAV_AUTOPILOT.MAV_AUTOPILOT_PPZ);
		p0.base_mode_SET(MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED);
		p0.type_SET(MAV_TYPE.MAV_TYPE_HELICOPTER);
		p0.custom_mode_SET(4091110721L);
		p0.system_status_SET(MAV_STATE.MAV_STATE_ACTIVE);
		TestChannel.instance.send(p0);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_SYS_STATUS.add((src, ph, pack) ->
		{
			assert (pack.load_GET() == (char) 61631);
			assert (pack.onboard_control_sensors_enabled_GET() == MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS);
			assert (pack.voltage_battery_GET() == (char) 9093);
			assert (pack.current_battery_GET() == (short) 25028);
			assert (pack.errors_count2_GET() == (char) 53937);
			assert (pack.onboard_control_sensors_present_GET() == MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS);
			assert (pack.errors_count3_GET() == (char) 42549);
			assert (pack.battery_remaining_GET() == (byte) 65);
			assert (pack.onboard_control_sensors_health_GET() == MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_BATTERY);
			assert (pack.errors_count4_GET() == (char) 18074);
			assert (pack.drop_rate_comm_GET() == (char) 10796);
			assert (pack.errors_comm_GET() == (char) 23868);
			assert (pack.errors_count1_GET() == (char) 35429);
		});
		SYS_STATUS p1 = new SYS_STATUS();
		PH.setPack(p1);
		p1.load_SET((char) 61631);
		p1.voltage_battery_SET((char) 9093);
		p1.current_battery_SET((short) 25028);
		p1.errors_count1_SET((char) 35429);
		p1.errors_comm_SET((char) 23868);
		p1.errors_count3_SET((char) 42549);
		p1.onboard_control_sensors_enabled_SET(MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS);
		p1.drop_rate_comm_SET((char) 10796);
		p1.onboard_control_sensors_health_SET(MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_BATTERY);
		p1.battery_remaining_SET((byte) 65);
		p1.onboard_control_sensors_present_SET(MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS);
		p1.errors_count4_SET((char) 18074);
		p1.errors_count2_SET((char) 53937);
		TestChannel.instance.send(p1);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_SYSTEM_TIME.add((src, ph, pack) ->
		{
			assert (pack.time_boot_ms_GET() == 2111247852L);
			assert (pack.time_unix_usec_GET() == 1214481502595512528L);
		});
		SYSTEM_TIME p2 = new SYSTEM_TIME();
		PH.setPack(p2);
		p2.time_unix_usec_SET(1214481502595512528L);
		p2.time_boot_ms_SET(2111247852L);
		TestChannel.instance.send(p2);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_POSITION_TARGET_LOCAL_NED.add((src, ph, pack) ->
		{
			assert (pack.time_boot_ms_GET() == 2452258785L);
			assert (pack.vy_GET() == 7.730983E36F);
			assert (pack.yaw_GET() == 1.3051943E38F);
			assert (pack.yaw_rate_GET() == -2.0628801E36F);
			assert (pack.afy_GET() == -2.6536957E38F);
			assert (pack.afz_GET() == -1.0386346E38F);
			assert (pack.afx_GET() == 1.6840512E38F);
			assert (pack.vx_GET() == -2.101489E38F);
			assert (pack.x_GET() == -3.262869E38F);
			assert (pack.type_mask_GET() == (char) 12067);
			assert (pack.y_GET() == 3.3618719E38F);
			assert (pack.vz_GET() == -2.3037528E38F);
			assert (pack.z_GET() == -2.747482E38F);
			assert (pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL);
		});
		GroundControl.POSITION_TARGET_LOCAL_NED p3 = CommunicationChannel.new_POSITION_TARGET_LOCAL_NED();
		PH.setPack(p3);
		p3.vy_SET(7.730983E36F);
		p3.vx_SET(-2.101489E38F);
		p3.type_mask_SET((char) 12067);
		p3.yaw_rate_SET(-2.0628801E36F);
		p3.yaw_SET(1.3051943E38F);
		p3.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL);
		p3.afy_SET(-2.6536957E38F);
		p3.time_boot_ms_SET(2452258785L);
		p3.afz_SET(-1.0386346E38F);
		p3.vz_SET(-2.3037528E38F);
		p3.z_SET(-2.747482E38F);
		p3.afx_SET(1.6840512E38F);
		p3.x_SET(-3.262869E38F);
		p3.y_SET(3.3618719E38F);
		CommunicationChannel.instance.send(p3);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_PING.add((src, ph, pack) ->
		{
			assert (pack.seq_GET() == 3199150153L);
			assert (pack.target_component_GET() == (char) 247);
			assert (pack.target_system_GET() == (char) 143);
			assert (pack.time_usec_GET() == 6121398152235908524L);
		});
		PING p4 = new PING();
		PH.setPack(p4);
		p4.target_component_SET((char) 247);
		p4.target_system_SET((char) 143);
		p4.time_usec_SET(6121398152235908524L);
		p4.seq_SET(3199150153L);
		TestChannel.instance.send(p4);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_CHANGE_OPERATOR_CONTROL.add((src, ph, pack) ->
		{
			assert (pack.version_GET() == (char) 116);
			assert (pack.target_system_GET() == (char) 86);
			assert (pack.passkey_LEN(ph) == 17);
			assert (pack.passkey_TRY(ph).equals("iehckkmAjpjpvfnwu"));
			assert (pack.control_request_GET() == (char) 226);
		});
		CHANGE_OPERATOR_CONTROL p5 = new CHANGE_OPERATOR_CONTROL();
		PH.setPack(p5);
		p5.control_request_SET((char) 226);
		p5.target_system_SET((char) 86);
		p5.version_SET((char) 116);
		p5.passkey_SET("iehckkmAjpjpvfnwu", PH);
		TestChannel.instance.send(p5);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_CHANGE_OPERATOR_CONTROL_ACK.add((src, ph, pack) ->
		{
			assert (pack.control_request_GET() == (char) 196);
			assert (pack.ack_GET() == (char) 92);
			assert (pack.gcs_system_id_GET() == (char) 43);
		});
		CHANGE_OPERATOR_CONTROL_ACK p6 = new CHANGE_OPERATOR_CONTROL_ACK();
		PH.setPack(p6);
		p6.control_request_SET((char) 196);
		p6.ack_SET((char) 92);
		p6.gcs_system_id_SET((char) 43);
		TestChannel.instance.send(p6);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_AUTH_KEY.add((src, ph, pack) ->
		{
			assert (pack.key_LEN(ph) == 16);
			assert (pack.key_TRY(ph).equals("tqhfpplwlfNpszVk"));
		});
		AUTH_KEY p7 = new AUTH_KEY();
		PH.setPack(p7);
		p7.key_SET("tqhfpplwlfNpszVk", PH);
		TestChannel.instance.send(p7);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_SET_MODE.add((src, ph, pack) ->
		{
			assert (pack.base_mode_GET() == MAV_MODE.MAV_MODE_TEST_ARMED);
			assert (pack.custom_mode_GET() == 2937162645L);
			assert (pack.target_system_GET() == (char) 94);
		});
		SET_MODE p11 = new SET_MODE();
		PH.setPack(p11);
		p11.target_system_SET((char) 94);
		p11.base_mode_SET(MAV_MODE.MAV_MODE_TEST_ARMED);
		p11.custom_mode_SET(2937162645L);
		TestChannel.instance.send(p11);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_PARAM_REQUEST_READ.add((src, ph, pack) ->
		{
			assert (pack.target_component_GET() == (char) 55);
			assert (pack.param_id_LEN(ph) == 11);
			assert (pack.param_id_TRY(ph).equals("ahenwegtabk"));
			assert (pack.target_system_GET() == (char) 112);
			assert (pack.param_index_GET() == (short) -27388);
		});
		PARAM_REQUEST_READ p20 = new PARAM_REQUEST_READ();
		PH.setPack(p20);
		p20.param_index_SET((short) -27388);
		p20.param_id_SET("ahenwegtabk", PH);
		p20.target_component_SET((char) 55);
		p20.target_system_SET((char) 112);
		TestChannel.instance.send(p20);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_PARAM_REQUEST_LIST.add((src, ph, pack) ->
		{
			assert (pack.target_system_GET() == (char) 26);
			assert (pack.target_component_GET() == (char) 208);
		});
		PARAM_REQUEST_LIST p21 = new PARAM_REQUEST_LIST();
		PH.setPack(p21);
		p21.target_component_SET((char) 208);
		p21.target_system_SET((char) 26);
		TestChannel.instance.send(p21);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_PARAM_VALUE.add((src, ph, pack) ->
		{
			assert (pack.param_index_GET() == (char) 2013);
			assert (pack.param_value_GET() == 1.6250645E38F);
			assert (pack.param_id_LEN(ph) == 1);
			assert (pack.param_id_TRY(ph).equals("X"));
			assert (pack.param_type_GET() == MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT64);
			assert (pack.param_count_GET() == (char) 15161);
		});
		PARAM_VALUE p22 = new PARAM_VALUE();
		PH.setPack(p22);
		p22.param_index_SET((char) 2013);
		p22.param_type_SET(MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT64);
		p22.param_id_SET("X", PH);
		p22.param_value_SET(1.6250645E38F);
		p22.param_count_SET((char) 15161);
		TestChannel.instance.send(p22);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_PARAM_SET.add((src, ph, pack) ->
		{
			assert (pack.target_system_GET() == (char) 23);
			assert (pack.param_type_GET() == MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT8);
			assert (pack.param_id_LEN(ph) == 7);
			assert (pack.param_id_TRY(ph).equals("pUdeydt"));
			assert (pack.target_component_GET() == (char) 86);
			assert (pack.param_value_GET() == -6.614184E36F);
		});
		PARAM_SET p23 = new PARAM_SET();
		PH.setPack(p23);
		p23.param_type_SET(MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT8);
		p23.param_id_SET("pUdeydt", PH);
		p23.param_value_SET(-6.614184E36F);
		p23.target_system_SET((char) 23);
		p23.target_component_SET((char) 86);
		TestChannel.instance.send(p23);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_GPS_RAW_INT.add((src, ph, pack) ->
		{
			assert (pack.fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_NO_FIX);
			assert (pack.h_acc_TRY(ph) == 750213067L);
			assert (pack.satellites_visible_GET() == (char) 150);
			assert (pack.cog_GET() == (char) 14407);
			assert (pack.epv_GET() == (char) 35605);
			assert (pack.lat_GET() == -915934782);
			assert (pack.v_acc_TRY(ph) == 2239608203L);
			assert (pack.vel_acc_TRY(ph) == 2938599798L);
			assert (pack.alt_GET() == -2124970866);
			assert (pack.alt_ellipsoid_TRY(ph) == -413762885);
			assert (pack.eph_GET() == (char) 43789);
			assert (pack.time_usec_GET() == 1867951601380352721L);
			assert (pack.vel_GET() == (char) 823);
			assert (pack.lon_GET() == -1426595364);
			assert (pack.hdg_acc_TRY(ph) == 900091622L);
		});
		GPS_RAW_INT p24 = new GPS_RAW_INT();
		PH.setPack(p24);
		p24.lat_SET(-915934782);
		p24.satellites_visible_SET((char) 150);
		p24.vel_acc_SET(2938599798L, PH);
		p24.alt_SET(-2124970866);
		p24.time_usec_SET(1867951601380352721L);
		p24.eph_SET((char) 43789);
		p24.alt_ellipsoid_SET(-413762885, PH);
		p24.h_acc_SET(750213067L, PH);
		p24.lon_SET(-1426595364);
		p24.hdg_acc_SET(900091622L, PH);
		p24.cog_SET((char) 14407);
		p24.fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_NO_FIX);
		p24.epv_SET((char) 35605);
		p24.v_acc_SET(2239608203L, PH);
		p24.vel_SET((char) 823);
		TestChannel.instance.send(p24);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_GPS_STATUS.add((src, ph, pack) ->
		{
			assert (Arrays.equals(pack.satellite_snr_GET(), new char[]{(char) 184, (char) 237, (char) 81, (char) 81, (char) 255, (char) 33, (char) 140, (char) 33, (char) 225, (char) 59, (char) 255, (char) 56, (char) 216, (char) 237, (char) 251, (char) 134, (char) 111, (char) 89, (char) 11, (char) 98}));
			assert (pack.satellites_visible_GET() == (char) 73);
			assert (Arrays.equals(pack.satellite_used_GET(), new char[]{(char) 115, (char) 58, (char) 183, (char) 167, (char) 2, (char) 202, (char) 55, (char) 218, (char) 234, (char) 236, (char) 128, (char) 27, (char) 215, (char) 32, (char) 1, (char) 93, (char) 21, (char) 189, (char) 31, (char) 135}));
			assert (Arrays.equals(pack.satellite_prn_GET(), new char[]{(char) 145, (char) 194, (char) 67, (char) 189, (char) 115, (char) 23, (char) 255, (char) 155, (char) 175, (char) 91, (char) 203, (char) 233, (char) 165, (char) 121, (char) 51, (char) 73, (char) 44, (char) 19, (char) 147, (char) 49}));
			assert (Arrays.equals(pack.satellite_elevation_GET(), new char[]{(char) 105, (char) 50, (char) 35, (char) 246, (char) 91, (char) 105, (char) 107, (char) 124, (char) 235, (char) 166, (char) 255, (char) 221, (char) 49, (char) 97, (char) 183, (char) 213, (char) 36, (char) 100, (char) 95, (char) 89}));
			assert (Arrays.equals(pack.satellite_azimuth_GET(), new char[]{(char) 185, (char) 39, (char) 137, (char) 249, (char) 75, (char) 159, (char) 21, (char) 11, (char) 255, (char) 74, (char) 147, (char) 44, (char) 3, (char) 127, (char) 46, (char) 190, (char) 43, (char) 252, (char) 169, (char) 166}));
		});
		GPS_STATUS p25 = new GPS_STATUS();
		PH.setPack(p25);
		p25.satellite_used_SET(new char[]{(char) 115, (char) 58, (char) 183, (char) 167, (char) 2, (char) 202, (char) 55, (char) 218, (char) 234, (char) 236, (char) 128, (char) 27, (char) 215, (char) 32, (char) 1, (char) 93, (char) 21, (char) 189, (char) 31, (char) 135}, 0);
		p25.satellite_prn_SET(new char[]{(char) 145, (char) 194, (char) 67, (char) 189, (char) 115, (char) 23, (char) 255, (char) 155, (char) 175, (char) 91, (char) 203, (char) 233, (char) 165, (char) 121, (char) 51, (char) 73, (char) 44, (char) 19, (char) 147, (char) 49}, 0);
		p25.satellite_snr_SET(new char[]{(char) 184, (char) 237, (char) 81, (char) 81, (char) 255, (char) 33, (char) 140, (char) 33, (char) 225, (char) 59, (char) 255, (char) 56, (char) 216, (char) 237, (char) 251, (char) 134, (char) 111, (char) 89, (char) 11, (char) 98}, 0);
		p25.satellite_azimuth_SET(new char[]{(char) 185, (char) 39, (char) 137, (char) 249, (char) 75, (char) 159, (char) 21, (char) 11, (char) 255, (char) 74, (char) 147, (char) 44, (char) 3, (char) 127, (char) 46, (char) 190, (char) 43, (char) 252, (char) 169, (char) 166}, 0);
		p25.satellites_visible_SET((char) 73);
		p25.satellite_elevation_SET(new char[]{(char) 105, (char) 50, (char) 35, (char) 246, (char) 91, (char) 105, (char) 107, (char) 124, (char) 235, (char) 166, (char) 255, (char) 221, (char) 49, (char) 97, (char) 183, (char) 213, (char) 36, (char) 100, (char) 95, (char) 89}, 0);
		TestChannel.instance.send(p25);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_SCALED_IMU.add((src, ph, pack) ->
		{
			assert (pack.xacc_GET() == (short) 22874);
			assert (pack.zacc_GET() == (short) -4641);
			assert (pack.xmag_GET() == (short) -29919);
			assert (pack.yacc_GET() == (short) 12852);
			assert (pack.ymag_GET() == (short) -27622);
			assert (pack.time_boot_ms_GET() == 3757705796L);
			assert (pack.zgyro_GET() == (short) -10340);
			assert (pack.xgyro_GET() == (short) -9723);
			assert (pack.ygyro_GET() == (short) -25737);
			assert (pack.zmag_GET() == (short) 1609);
		});
		SCALED_IMU p26 = new SCALED_IMU();
		PH.setPack(p26);
		p26.yacc_SET((short) 12852);
		p26.ygyro_SET((short) -25737);
		p26.xacc_SET((short) 22874);
		p26.zacc_SET((short) -4641);
		p26.xgyro_SET((short) -9723);
		p26.xmag_SET((short) -29919);
		p26.time_boot_ms_SET(3757705796L);
		p26.ymag_SET((short) -27622);
		p26.zmag_SET((short) 1609);
		p26.zgyro_SET((short) -10340);
		TestChannel.instance.send(p26);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_RAW_IMU.add((src, ph, pack) ->
		{
			assert (pack.ygyro_GET() == (short) 17792);
			assert (pack.zmag_GET() == (short) -18617);
			assert (pack.zgyro_GET() == (short) 6683);
			assert (pack.time_usec_GET() == 8763887409480725425L);
			assert (pack.ymag_GET() == (short) -14213);
			assert (pack.zacc_GET() == (short) -9892);
			assert (pack.yacc_GET() == (short) 4539);
			assert (pack.xacc_GET() == (short) 8879);
			assert (pack.xmag_GET() == (short) -14133);
			assert (pack.xgyro_GET() == (short) -30775);
		});
		RAW_IMU p27 = new RAW_IMU();
		PH.setPack(p27);
		p27.yacc_SET((short) 4539);
		p27.zmag_SET((short) -18617);
		p27.time_usec_SET(8763887409480725425L);
		p27.ygyro_SET((short) 17792);
		p27.ymag_SET((short) -14213);
		p27.xacc_SET((short) 8879);
		p27.xmag_SET((short) -14133);
		p27.zacc_SET((short) -9892);
		p27.zgyro_SET((short) 6683);
		p27.xgyro_SET((short) -30775);
		TestChannel.instance.send(p27);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_RAW_PRESSURE.add((src, ph, pack) ->
		{
			assert (pack.press_abs_GET() == (short) -13598);
			assert (pack.time_usec_GET() == 3592965837857647255L);
			assert (pack.temperature_GET() == (short) 32251);
			assert (pack.press_diff1_GET() == (short) 2923);
			assert (pack.press_diff2_GET() == (short) -21169);
		});
		RAW_PRESSURE p28 = new RAW_PRESSURE();
		PH.setPack(p28);
		p28.time_usec_SET(3592965837857647255L);
		p28.temperature_SET((short) 32251);
		p28.press_diff1_SET((short) 2923);
		p28.press_diff2_SET((short) -21169);
		p28.press_abs_SET((short) -13598);
		TestChannel.instance.send(p28);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_SCALED_PRESSURE.add((src, ph, pack) ->
		{
			assert (pack.press_abs_GET() == 1.1994023E38F);
			assert (pack.time_boot_ms_GET() == 1502104455L);
			assert (pack.press_diff_GET() == 2.7771691E38F);
			assert (pack.temperature_GET() == (short) 14338);
		});
		SCALED_PRESSURE p29 = new SCALED_PRESSURE();
		PH.setPack(p29);
		p29.time_boot_ms_SET(1502104455L);
		p29.press_abs_SET(1.1994023E38F);
		p29.press_diff_SET(2.7771691E38F);
		p29.temperature_SET((short) 14338);
		TestChannel.instance.send(p29);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_ATTITUDE.add((src, ph, pack) ->
		{
			assert (pack.rollspeed_GET() == -1.5659886E38F);
			assert (pack.yawspeed_GET() == 2.1223264E38F);
			assert (pack.pitchspeed_GET() == 2.7252162E38F);
			assert (pack.pitch_GET() == -1.128288E38F);
			assert (pack.time_boot_ms_GET() == 1553474581L);
			assert (pack.roll_GET() == 2.0033244E38F);
			assert (pack.yaw_GET() == 1.5521357E38F);
		});
		ATTITUDE p30 = new ATTITUDE();
		PH.setPack(p30);
		p30.pitchspeed_SET(2.7252162E38F);
		p30.roll_SET(2.0033244E38F);
		p30.pitch_SET(-1.128288E38F);
		p30.yawspeed_SET(2.1223264E38F);
		p30.rollspeed_SET(-1.5659886E38F);
		p30.time_boot_ms_SET(1553474581L);
		p30.yaw_SET(1.5521357E38F);
		TestChannel.instance.send(p30);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_ATTITUDE_QUATERNION.add((src, ph, pack) ->
		{
			assert (pack.rollspeed_GET() == -1.8297115E37F);
			assert (pack.q1_GET() == 2.9833646E38F);
			assert (pack.q3_GET() == 3.3638133E38F);
			assert (pack.yawspeed_GET() == -7.9167746E36F);
			assert (pack.pitchspeed_GET() == -9.105253E37F);
			assert (pack.time_boot_ms_GET() == 2382225989L);
			assert (pack.q2_GET() == -3.106945E38F);
			assert (pack.q4_GET() == -1.3443558E38F);
		});
		ATTITUDE_QUATERNION p31 = new ATTITUDE_QUATERNION();
		PH.setPack(p31);
		p31.rollspeed_SET(-1.8297115E37F);
		p31.time_boot_ms_SET(2382225989L);
		p31.yawspeed_SET(-7.9167746E36F);
		p31.pitchspeed_SET(-9.105253E37F);
		p31.q2_SET(-3.106945E38F);
		p31.q3_SET(3.3638133E38F);
		p31.q4_SET(-1.3443558E38F);
		p31.q1_SET(2.9833646E38F);
		TestChannel.instance.send(p31);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_LOCAL_POSITION_NED.add((src, ph, pack) ->
		{
			assert (pack.vy_GET() == -1.6933047E38F);
			assert (pack.vx_GET() == 1.8562433E38F);
			assert (pack.x_GET() == -5.8855755E37F);
			assert (pack.z_GET() == -1.1599558E38F);
			assert (pack.y_GET() == 3.3142108E38F);
			assert (pack.time_boot_ms_GET() == 2561532926L);
			assert (pack.vz_GET() == 1.1781149E38F);
		});
		LOCAL_POSITION_NED p32 = new LOCAL_POSITION_NED();
		PH.setPack(p32);
		p32.time_boot_ms_SET(2561532926L);
		p32.y_SET(3.3142108E38F);
		p32.x_SET(-5.8855755E37F);
		p32.vz_SET(1.1781149E38F);
		p32.vx_SET(1.8562433E38F);
		p32.vy_SET(-1.6933047E38F);
		p32.z_SET(-1.1599558E38F);
		TestChannel.instance.send(p32);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_GLOBAL_POSITION_INT.add((src, ph, pack) ->
		{
			assert (pack.lon_GET() == -669010761);
			assert (pack.time_boot_ms_GET() == 3815523016L);
			assert (pack.lat_GET() == -1169432319);
			assert (pack.relative_alt_GET() == -159357459);
			assert (pack.vx_GET() == (short) -27150);
			assert (pack.alt_GET() == -364752772);
			assert (pack.vz_GET() == (short) -18391);
			assert (pack.vy_GET() == (short) -22775);
			assert (pack.hdg_GET() == (char) 42523);
		});
		GLOBAL_POSITION_INT p33 = new GLOBAL_POSITION_INT();
		PH.setPack(p33);
		p33.time_boot_ms_SET(3815523016L);
		p33.alt_SET(-364752772);
		p33.lat_SET(-1169432319);
		p33.hdg_SET((char) 42523);
		p33.vz_SET((short) -18391);
		p33.vy_SET((short) -22775);
		p33.lon_SET(-669010761);
		p33.relative_alt_SET(-159357459);
		p33.vx_SET((short) -27150);
		TestChannel.instance.send(p33);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_RC_CHANNELS_SCALED.add((src, ph, pack) ->
		{
			assert (pack.chan2_scaled_GET() == (short) 28570);
			assert (pack.port_GET() == (char) 220);
			assert (pack.rssi_GET() == (char) 229);
			assert (pack.chan3_scaled_GET() == (short) 7486);
			assert (pack.chan4_scaled_GET() == (short) -6337);
			assert (pack.chan6_scaled_GET() == (short) 6715);
			assert (pack.chan1_scaled_GET() == (short) 8769);
			assert (pack.time_boot_ms_GET() == 3054828506L);
			assert (pack.chan7_scaled_GET() == (short) -11607);
			assert (pack.chan8_scaled_GET() == (short) -9307);
			assert (pack.chan5_scaled_GET() == (short) -10832);
		});
		RC_CHANNELS_SCALED p34 = new RC_CHANNELS_SCALED();
		PH.setPack(p34);
		p34.chan5_scaled_SET((short) -10832);
		p34.rssi_SET((char) 229);
		p34.chan2_scaled_SET((short) 28570);
		p34.chan8_scaled_SET((short) -9307);
		p34.chan4_scaled_SET((short) -6337);
		p34.chan3_scaled_SET((short) 7486);
		p34.chan7_scaled_SET((short) -11607);
		p34.time_boot_ms_SET(3054828506L);
		p34.port_SET((char) 220);
		p34.chan6_scaled_SET((short) 6715);
		p34.chan1_scaled_SET((short) 8769);
		TestChannel.instance.send(p34);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_RC_CHANNELS_RAW.add((src, ph, pack) ->
		{
			assert (pack.time_boot_ms_GET() == 3225621240L);
			assert (pack.chan6_raw_GET() == (char) 5936);
			assert (pack.chan3_raw_GET() == (char) 3837);
			assert (pack.rssi_GET() == (char) 80);
			assert (pack.chan5_raw_GET() == (char) 36647);
			assert (pack.chan7_raw_GET() == (char) 64028);
			assert (pack.chan2_raw_GET() == (char) 55730);
			assert (pack.chan1_raw_GET() == (char) 13517);
			assert (pack.chan8_raw_GET() == (char) 31114);
			assert (pack.port_GET() == (char) 86);
			assert (pack.chan4_raw_GET() == (char) 20935);
		});
		RC_CHANNELS_RAW p35 = new RC_CHANNELS_RAW();
		PH.setPack(p35);
		p35.chan2_raw_SET((char) 55730);
		p35.chan1_raw_SET((char) 13517);
		p35.chan4_raw_SET((char) 20935);
		p35.chan7_raw_SET((char) 64028);
		p35.chan8_raw_SET((char) 31114);
		p35.chan3_raw_SET((char) 3837);
		p35.chan5_raw_SET((char) 36647);
		p35.chan6_raw_SET((char) 5936);
		p35.rssi_SET((char) 80);
		p35.time_boot_ms_SET(3225621240L);
		p35.port_SET((char) 86);
		TestChannel.instance.send(p35);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_SERVO_OUTPUT_RAW.add((src, ph, pack) ->
		{
			assert (pack.servo2_raw_GET() == (char) 30542);
			assert (pack.servo1_raw_GET() == (char) 64463);
			assert (pack.servo11_raw_TRY(ph) == (char) 11993);
			assert (pack.port_GET() == (char) 255);
			assert (pack.servo10_raw_TRY(ph) == (char) 50457);
			assert (pack.servo14_raw_TRY(ph) == (char) 6224);
			assert (pack.servo13_raw_TRY(ph) == (char) 65471);
			assert (pack.servo12_raw_TRY(ph) == (char) 28795);
			assert (pack.servo5_raw_GET() == (char) 24054);
			assert (pack.servo15_raw_TRY(ph) == (char) 60151);
			assert (pack.servo9_raw_TRY(ph) == (char) 25501);
			assert (pack.servo3_raw_GET() == (char) 51883);
			assert (pack.servo8_raw_GET() == (char) 57167);
			assert (pack.servo7_raw_GET() == (char) 5203);
			assert (pack.time_usec_GET() == 3492730442L);
			assert (pack.servo4_raw_GET() == (char) 5722);
			assert (pack.servo6_raw_GET() == (char) 59702);
			assert (pack.servo16_raw_TRY(ph) == (char) 27752);
		});
		SERVO_OUTPUT_RAW p36 = new SERVO_OUTPUT_RAW();
		PH.setPack(p36);
		p36.servo11_raw_SET((char) 11993, PH);
		p36.servo13_raw_SET((char) 65471, PH);
		p36.time_usec_SET(3492730442L);
		p36.servo12_raw_SET((char) 28795, PH);
		p36.servo1_raw_SET((char) 64463);
		p36.servo2_raw_SET((char) 30542);
		p36.servo4_raw_SET((char) 5722);
		p36.servo6_raw_SET((char) 59702);
		p36.servo9_raw_SET((char) 25501, PH);
		p36.servo14_raw_SET((char) 6224, PH);
		p36.servo10_raw_SET((char) 50457, PH);
		p36.servo8_raw_SET((char) 57167);
		p36.servo7_raw_SET((char) 5203);
		p36.port_SET((char) 255);
		p36.servo16_raw_SET((char) 27752, PH);
		p36.servo5_raw_SET((char) 24054);
		p36.servo3_raw_SET((char) 51883);
		p36.servo15_raw_SET((char) 60151, PH);
		TestChannel.instance.send(p36);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_MISSION_REQUEST_PARTIAL_LIST.add((src, ph, pack) ->
		{
			assert (pack.end_index_GET() == (short) -14504);
			assert (pack.target_component_GET() == (char) 169);
			assert (pack.start_index_GET() == (short) 8347);
			assert (pack.target_system_GET() == (char) 190);
			assert (pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
		});
		MISSION_REQUEST_PARTIAL_LIST p37 = new MISSION_REQUEST_PARTIAL_LIST();
		PH.setPack(p37);
		p37.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
		p37.target_component_SET((char) 169);
		p37.start_index_SET((short) 8347);
		p37.end_index_SET((short) -14504);
		p37.target_system_SET((char) 190);
		TestChannel.instance.send(p37);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_MISSION_WRITE_PARTIAL_LIST.add((src, ph, pack) ->
		{
			assert (pack.start_index_GET() == (short) -20180);
			assert (pack.target_component_GET() == (char) 23);
			assert (pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
			assert (pack.end_index_GET() == (short) 16795);
			assert (pack.target_system_GET() == (char) 20);
		});
		MISSION_WRITE_PARTIAL_LIST p38 = new MISSION_WRITE_PARTIAL_LIST();
		PH.setPack(p38);
		p38.start_index_SET((short) -20180);
		p38.end_index_SET((short) 16795);
		p38.target_component_SET((char) 23);
		p38.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
		p38.target_system_SET((char) 20);
		TestChannel.instance.send(p38);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_MISSION_ITEM.add((src, ph, pack) ->
		{
			assert (pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
			assert (pack.param1_GET() == -2.6912776E38F);
			assert (pack.seq_GET() == (char) 54915);
			assert (pack.param2_GET() == 2.5684124E38F);
			assert (pack.param4_GET() == 8.76648E37F);
			assert (pack.command_GET() == MAV_CMD.MAV_CMD_DO_JUMP);
			assert (pack.frame_GET() == MAV_FRAME.MAV_FRAME_BODY_NED);
			assert (pack.x_GET() == 2.7497427E36F);
			assert (pack.target_component_GET() == (char) 197);
			assert (pack.z_GET() == 3.1702051E38F);
			assert (pack.current_GET() == (char) 14);
			assert (pack.target_system_GET() == (char) 169);
			assert (pack.y_GET() == -1.18783E38F);
			assert (pack.autocontinue_GET() == (char) 131);
			assert (pack.param3_GET() == 9.628298E37F);
		});
		MISSION_ITEM p39 = new MISSION_ITEM();
		PH.setPack(p39);
		p39.x_SET(2.7497427E36F);
		p39.frame_SET(MAV_FRAME.MAV_FRAME_BODY_NED);
		p39.target_component_SET((char) 197);
		p39.z_SET(3.1702051E38F);
		p39.target_system_SET((char) 169);
		p39.param1_SET(-2.6912776E38F);
		p39.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
		p39.seq_SET((char) 54915);
		p39.param3_SET(9.628298E37F);
		p39.current_SET((char) 14);
		p39.param4_SET(8.76648E37F);
		p39.command_SET(MAV_CMD.MAV_CMD_DO_JUMP);
		p39.autocontinue_SET((char) 131);
		p39.param2_SET(2.5684124E38F);
		p39.y_SET(-1.18783E38F);
		TestChannel.instance.send(p39);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_MISSION_REQUEST.add((src, ph, pack) ->
		{
			assert (pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
			assert (pack.target_component_GET() == (char) 155);
			assert (pack.seq_GET() == (char) 27530);
			assert (pack.target_system_GET() == (char) 48);
		});
		MISSION_REQUEST p40 = new MISSION_REQUEST();
		PH.setPack(p40);
		p40.seq_SET((char) 27530);
		p40.target_component_SET((char) 155);
		p40.target_system_SET((char) 48);
		p40.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
		TestChannel.instance.send(p40);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_MISSION_SET_CURRENT.add((src, ph, pack) ->
		{
			assert (pack.target_component_GET() == (char) 11);
			assert (pack.target_system_GET() == (char) 109);
			assert (pack.seq_GET() == (char) 8663);
		});
		MISSION_SET_CURRENT p41 = new MISSION_SET_CURRENT();
		PH.setPack(p41);
		p41.target_system_SET((char) 109);
		p41.seq_SET((char) 8663);
		p41.target_component_SET((char) 11);
		TestChannel.instance.send(p41);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_MISSION_CURRENT.add((src, ph, pack) ->
		{
			assert (pack.seq_GET() == (char) 36819);
		});
		MISSION_CURRENT p42 = new MISSION_CURRENT();
		PH.setPack(p42);
		p42.seq_SET((char) 36819);
		TestChannel.instance.send(p42);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_MISSION_REQUEST_LIST.add((src, ph, pack) ->
		{
			assert (pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
			assert (pack.target_component_GET() == (char) 234);
			assert (pack.target_system_GET() == (char) 57);
		});
		MISSION_REQUEST_LIST p43 = new MISSION_REQUEST_LIST();
		PH.setPack(p43);
		p43.target_component_SET((char) 234);
		p43.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
		p43.target_system_SET((char) 57);
		TestChannel.instance.send(p43);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_MISSION_COUNT.add((src, ph, pack) ->
		{
			assert (pack.target_system_GET() == (char) 227);
			assert (pack.count_GET() == (char) 15324);
			assert (pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
			assert (pack.target_component_GET() == (char) 68);
		});
		MISSION_COUNT p44 = new MISSION_COUNT();
		PH.setPack(p44);
		p44.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
		p44.target_system_SET((char) 227);
		p44.target_component_SET((char) 68);
		p44.count_SET((char) 15324);
		TestChannel.instance.send(p44);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_MISSION_CLEAR_ALL.add((src, ph, pack) ->
		{
			assert (pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
			assert (pack.target_system_GET() == (char) 100);
			assert (pack.target_component_GET() == (char) 109);
		});
		MISSION_CLEAR_ALL p45 = new MISSION_CLEAR_ALL();
		PH.setPack(p45);
		p45.target_component_SET((char) 109);
		p45.target_system_SET((char) 100);
		p45.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
		TestChannel.instance.send(p45);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_MISSION_ITEM_REACHED.add((src, ph, pack) ->
		{
			assert (pack.seq_GET() == (char) 41580);
		});
		MISSION_ITEM_REACHED p46 = new MISSION_ITEM_REACHED();
		PH.setPack(p46);
		p46.seq_SET((char) 41580);
		TestChannel.instance.send(p46);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_MISSION_ACK.add((src, ph, pack) ->
		{
			assert (pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
			assert (pack.target_component_GET() == (char) 52);
			assert (pack.type_GET() == MAV_MISSION_RESULT.MAV_MISSION_INVALID_PARAM4);
			assert (pack.target_system_GET() == (char) 23);
		});
		MISSION_ACK p47 = new MISSION_ACK();
		PH.setPack(p47);
		p47.target_system_SET((char) 23);
		p47.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
		p47.target_component_SET((char) 52);
		p47.type_SET(MAV_MISSION_RESULT.MAV_MISSION_INVALID_PARAM4);
		TestChannel.instance.send(p47);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_SET_GPS_GLOBAL_ORIGIN.add((src, ph, pack) ->
		{
			assert (pack.time_usec_TRY(ph) == 6529502558744463347L);
			assert (pack.latitude_GET() == 1975220630);
			assert (pack.longitude_GET() == 122329395);
			assert (pack.target_system_GET() == (char) 182);
			assert (pack.altitude_GET() == 691704916);
		});
		SET_GPS_GLOBAL_ORIGIN p48 = new SET_GPS_GLOBAL_ORIGIN();
		PH.setPack(p48);
		p48.altitude_SET(691704916);
		p48.time_usec_SET(6529502558744463347L, PH);
		p48.longitude_SET(122329395);
		p48.target_system_SET((char) 182);
		p48.latitude_SET(1975220630);
		TestChannel.instance.send(p48);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_GPS_GLOBAL_ORIGIN.add((src, ph, pack) ->
		{
			assert (pack.altitude_GET() == 443630834);
			assert (pack.longitude_GET() == 1019856867);
			assert (pack.latitude_GET() == 467420092);
			assert (pack.time_usec_TRY(ph) == 6025883589491645661L);
		});
		GPS_GLOBAL_ORIGIN p49 = new GPS_GLOBAL_ORIGIN();
		PH.setPack(p49);
		p49.altitude_SET(443630834);
		p49.longitude_SET(1019856867);
		p49.time_usec_SET(6025883589491645661L, PH);
		p49.latitude_SET(467420092);
		TestChannel.instance.send(p49);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_PARAM_MAP_RC.add((src, ph, pack) ->
		{
			assert (pack.target_system_GET() == (char) 75);
			assert (pack.scale_GET() == -1.2782586E38F);
			assert (pack.param_value_min_GET() == 2.218803E38F);
			assert (pack.param_id_LEN(ph) == 7);
			assert (pack.param_id_TRY(ph).equals("tnOmlep"));
			assert (pack.target_component_GET() == (char) 247);
			assert (pack.param_value0_GET() == -4.0453702E37F);
			assert (pack.param_value_max_GET() == -2.9891662E38F);
			assert (pack.parameter_rc_channel_index_GET() == (char) 130);
			assert (pack.param_index_GET() == (short) 30355);
		});
		PARAM_MAP_RC p50 = new PARAM_MAP_RC();
		PH.setPack(p50);
		p50.param_value_min_SET(2.218803E38F);
		p50.parameter_rc_channel_index_SET((char) 130);
		p50.param_value_max_SET(-2.9891662E38F);
		p50.param_value0_SET(-4.0453702E37F);
		p50.param_id_SET("tnOmlep", PH);
		p50.scale_SET(-1.2782586E38F);
		p50.target_system_SET((char) 75);
		p50.target_component_SET((char) 247);
		p50.param_index_SET((short) 30355);
		TestChannel.instance.send(p50);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_MISSION_REQUEST_INT.add((src, ph, pack) ->
		{
			assert (pack.target_component_GET() == (char) 46);
			assert (pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
			assert (pack.seq_GET() == (char) 59356);
			assert (pack.target_system_GET() == (char) 174);
		});
		MISSION_REQUEST_INT p51 = new MISSION_REQUEST_INT();
		PH.setPack(p51);
		p51.target_system_SET((char) 174);
		p51.target_component_SET((char) 46);
		p51.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
		p51.seq_SET((char) 59356);
		TestChannel.instance.send(p51);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_SAFETY_SET_ALLOWED_AREA.add((src, ph, pack) ->
		{
			assert (pack.p1x_GET() == 3.0420527E38F);
			assert (pack.p1z_GET() == -9.828544E37F);
			assert (pack.target_system_GET() == (char) 67);
			assert (pack.p2y_GET() == -1.2405459E38F);
			assert (pack.p1y_GET() == 1.4078001E38F);
			assert (pack.target_component_GET() == (char) 139);
			assert (pack.p2x_GET() == 9.356765E37F);
			assert (pack.frame_GET() == MAV_FRAME.MAV_FRAME_LOCAL_ENU);
			assert (pack.p2z_GET() == -2.1421952E38F);
		});
		SAFETY_SET_ALLOWED_AREA p54 = new SAFETY_SET_ALLOWED_AREA();
		PH.setPack(p54);
		p54.p2z_SET(-2.1421952E38F);
		p54.frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_ENU);
		p54.target_component_SET((char) 139);
		p54.p2x_SET(9.356765E37F);
		p54.p1x_SET(3.0420527E38F);
		p54.target_system_SET((char) 67);
		p54.p2y_SET(-1.2405459E38F);
		p54.p1y_SET(1.4078001E38F);
		p54.p1z_SET(-9.828544E37F);
		TestChannel.instance.send(p54);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_SAFETY_ALLOWED_AREA.add((src, ph, pack) ->
		{
			assert (pack.p2y_GET() == -1.8083975E38F);
			assert (pack.p1x_GET() == 1.0494807E38F);
			assert (pack.p2x_GET() == -2.5866362E38F);
			assert (pack.p1z_GET() == -3.1981784E38F);
			assert (pack.p2z_GET() == 2.1538432E38F);
			assert (pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL);
			assert (pack.p1y_GET() == 3.2657902E38F);
		});
		SAFETY_ALLOWED_AREA p55 = new SAFETY_ALLOWED_AREA();
		PH.setPack(p55);
		p55.p2y_SET(-1.8083975E38F);
		p55.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL);
		p55.p2x_SET(-2.5866362E38F);
		p55.p2z_SET(2.1538432E38F);
		p55.p1y_SET(3.2657902E38F);
		p55.p1z_SET(-3.1981784E38F);
		p55.p1x_SET(1.0494807E38F);
		TestChannel.instance.send(p55);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_ATTITUDE_QUATERNION_COV.add((src, ph, pack) ->
		{
			assert (Arrays.equals(pack.q_GET(), new float[]{2.7659081E38F, 1.8169486E38F, -8.856351E37F, -8.025052E37F}));
			assert (pack.pitchspeed_GET() == 4.803268E37F);
			assert (Arrays.equals(pack.covariance_GET(), new float[]{-5.0115055E37F, -3.226323E37F, -3.8255233E37F, 1.3404009E38F, 5.0670626E37F, 3.0584551E38F, -1.1305741E38F, -1.387767E38F, 6.156151E37F}));
			assert (pack.yawspeed_GET() == -1.9304941E38F);
			assert (pack.rollspeed_GET() == 1.1591986E38F);
			assert (pack.time_usec_GET() == 8558384662790266785L);
		});
		ATTITUDE_QUATERNION_COV p61 = new ATTITUDE_QUATERNION_COV();
		PH.setPack(p61);
		p61.pitchspeed_SET(4.803268E37F);
		p61.yawspeed_SET(-1.9304941E38F);
		p61.covariance_SET(new float[]{-5.0115055E37F, -3.226323E37F, -3.8255233E37F, 1.3404009E38F, 5.0670626E37F, 3.0584551E38F, -1.1305741E38F, -1.387767E38F, 6.156151E37F}, 0);
		p61.rollspeed_SET(1.1591986E38F);
		p61.q_SET(new float[]{2.7659081E38F, 1.8169486E38F, -8.856351E37F, -8.025052E37F}, 0);
		p61.time_usec_SET(8558384662790266785L);
		TestChannel.instance.send(p61);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_NAV_CONTROLLER_OUTPUT.add((src, ph, pack) ->
		{
			assert (pack.nav_pitch_GET() == 9.691923E37F);
			assert (pack.aspd_error_GET() == -1.5435797E38F);
			assert (pack.alt_error_GET() == 5.553428E37F);
			assert (pack.wp_dist_GET() == (char) 20299);
			assert (pack.target_bearing_GET() == (short) 15282);
			assert (pack.nav_roll_GET() == 2.1607549E38F);
			assert (pack.nav_bearing_GET() == (short) -15000);
			assert (pack.xtrack_error_GET() == -3.2676053E38F);
		});
		NAV_CONTROLLER_OUTPUT p62 = new NAV_CONTROLLER_OUTPUT();
		PH.setPack(p62);
		p62.wp_dist_SET((char) 20299);
		p62.nav_pitch_SET(9.691923E37F);
		p62.aspd_error_SET(-1.5435797E38F);
		p62.nav_bearing_SET((short) -15000);
		p62.nav_roll_SET(2.1607549E38F);
		p62.xtrack_error_SET(-3.2676053E38F);
		p62.alt_error_SET(5.553428E37F);
		p62.target_bearing_SET((short) 15282);
		TestChannel.instance.send(p62);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_GLOBAL_POSITION_INT_COV.add((src, ph, pack) ->
		{
			assert (pack.time_usec_GET() == 3936802521010936946L);
			assert (pack.vz_GET() == 1.7577892E38F);
			assert (pack.alt_GET() == -466558563);
			assert (pack.vx_GET() == -3.242519E38F);
			assert (pack.lon_GET() == 1129039483);
			assert (pack.relative_alt_GET() == -1623547113);
			assert (pack.vy_GET() == -6.0431825E37F);
			assert (pack.lat_GET() == -309271178);
			assert (Arrays.equals(pack.covariance_GET(), new float[]{-6.439901E36F, -2.5264175E38F, -3.136036E38F, 2.2123841E38F, 7.082173E37F, -1.6640641E38F, 2.3075123E38F, -2.4434339E38F, -2.6304389E38F, -3.2351003E38F, 2.7784139E38F, 1.5326399E38F, 3.2578294E38F, -2.0310831E38F, 3.3682132E38F, -1.7156363E38F, -3.190523E38F, 1.0767076E38F, -1.7384796E38F, -3.619876E36F, 2.0632336E38F, 1.9792433E38F, 9.316401E37F, -3.1009161E38F, -1.6225133E38F, 1.4034107E38F, 2.826332E37F, -2.9302656E38F, 2.6035598E38F, 4.2777904E37F, 3.2311534E38F, 3.3316292E38F, -1.8503851E36F, -5.897882E37F, 3.3048759E38F, 5.1677866E37F}));
			assert (pack.estimator_type_GET() == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS_INS);
		});
		GLOBAL_POSITION_INT_COV p63 = new GLOBAL_POSITION_INT_COV();
		PH.setPack(p63);
		p63.vy_SET(-6.0431825E37F);
		p63.lon_SET(1129039483);
		p63.estimator_type_SET(MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS_INS);
		p63.time_usec_SET(3936802521010936946L);
		p63.vx_SET(-3.242519E38F);
		p63.covariance_SET(new float[]{-6.439901E36F, -2.5264175E38F, -3.136036E38F, 2.2123841E38F, 7.082173E37F, -1.6640641E38F, 2.3075123E38F, -2.4434339E38F, -2.6304389E38F, -3.2351003E38F, 2.7784139E38F, 1.5326399E38F, 3.2578294E38F, -2.0310831E38F, 3.3682132E38F, -1.7156363E38F, -3.190523E38F, 1.0767076E38F, -1.7384796E38F, -3.619876E36F, 2.0632336E38F, 1.9792433E38F, 9.316401E37F, -3.1009161E38F, -1.6225133E38F, 1.4034107E38F, 2.826332E37F, -2.9302656E38F, 2.6035598E38F, 4.2777904E37F, 3.2311534E38F, 3.3316292E38F, -1.8503851E36F, -5.897882E37F, 3.3048759E38F, 5.1677866E37F}, 0);
		p63.relative_alt_SET(-1623547113);
		p63.lat_SET(-309271178);
		p63.alt_SET(-466558563);
		p63.vz_SET(1.7577892E38F);
		TestChannel.instance.send(p63);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_LOCAL_POSITION_NED_COV.add((src, ph, pack) ->
		{
			assert (pack.vx_GET() == -1.1950536E38F);
			assert (pack.ax_GET() == -5.0026E37F);
			assert (pack.estimator_type_GET() == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VIO);
			assert (pack.x_GET() == -1.1830794E38F);
			assert (pack.az_GET() == 3.3401539E37F);
			assert (pack.vy_GET() == 1.745446E38F);
			assert (pack.z_GET() == 1.8039461E38F);
			assert (pack.y_GET() == 1.7993887E38F);
			assert (Arrays.equals(pack.covariance_GET(), new float[]{6.9405706E37F, -2.7633662E37F, 1.6263178E38F, -2.6704875E38F, 2.5223681E38F, 8.532498E37F, 3.3935599E38F, 9.018424E37F, -2.9624264E38F, 1.7981756E38F, -9.35294E36F, -9.062331E37F, 1.1059056E38F, -2.7355379E38F, -1.5076269E37F, -2.7470055E38F, -1.3227733E38F, -1.0611062E38F, 3.0511608E38F, -2.2494957E38F, 1.6300788E38F, 1.8618755E38F, 1.8780958E38F, -1.6632126E38F, 2.8688347E38F, 3.120054E37F, -2.3196304E38F, -2.193693E38F, 1.3217648E38F, 2.4673561E37F, -6.644238E37F, -4.0379098E37F, 2.856866E38F, 2.0121517E37F, -4.9972217E37F, -3.0617626E38F, -3.072798E38F, 1.1697857E38F, 2.6703017E38F, -8.3355187E37F, 2.475067E38F, 1.6398648E38F, -1.7791168E38F, -3.0896391E38F, 2.256676E38F}));
			assert (pack.vz_GET() == -2.4054534E38F);
			assert (pack.ay_GET() == 9.203473E37F);
			assert (pack.time_usec_GET() == 5029617608366830760L);
		});
		LOCAL_POSITION_NED_COV p64 = new LOCAL_POSITION_NED_COV();
		PH.setPack(p64);
		p64.vy_SET(1.745446E38F);
		p64.vx_SET(-1.1950536E38F);
		p64.covariance_SET(new float[]{6.9405706E37F, -2.7633662E37F, 1.6263178E38F, -2.6704875E38F, 2.5223681E38F, 8.532498E37F, 3.3935599E38F, 9.018424E37F, -2.9624264E38F, 1.7981756E38F, -9.35294E36F, -9.062331E37F, 1.1059056E38F, -2.7355379E38F, -1.5076269E37F, -2.7470055E38F, -1.3227733E38F, -1.0611062E38F, 3.0511608E38F, -2.2494957E38F, 1.6300788E38F, 1.8618755E38F, 1.8780958E38F, -1.6632126E38F, 2.8688347E38F, 3.120054E37F, -2.3196304E38F, -2.193693E38F, 1.3217648E38F, 2.4673561E37F, -6.644238E37F, -4.0379098E37F, 2.856866E38F, 2.0121517E37F, -4.9972217E37F, -3.0617626E38F, -3.072798E38F, 1.1697857E38F, 2.6703017E38F, -8.3355187E37F, 2.475067E38F, 1.6398648E38F, -1.7791168E38F, -3.0896391E38F, 2.256676E38F}, 0);
		p64.x_SET(-1.1830794E38F);
		p64.ax_SET(-5.0026E37F);
		p64.time_usec_SET(5029617608366830760L);
		p64.az_SET(3.3401539E37F);
		p64.y_SET(1.7993887E38F);
		p64.z_SET(1.8039461E38F);
		p64.vz_SET(-2.4054534E38F);
		p64.estimator_type_SET(MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VIO);
		p64.ay_SET(9.203473E37F);
		TestChannel.instance.send(p64);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_RC_CHANNELS.add((src, ph, pack) ->
		{
			assert (pack.chan11_raw_GET() == (char) 29193);
			assert (pack.chan10_raw_GET() == (char) 33180);
			assert (pack.chan7_raw_GET() == (char) 58976);
			assert (pack.chan14_raw_GET() == (char) 52532);
			assert (pack.chan12_raw_GET() == (char) 3189);
			assert (pack.chan9_raw_GET() == (char) 6340);
			assert (pack.chan2_raw_GET() == (char) 40627);
			assert (pack.chan8_raw_GET() == (char) 40678);
			assert (pack.time_boot_ms_GET() == 2802030361L);
			assert (pack.rssi_GET() == (char) 223);
			assert (pack.chan6_raw_GET() == (char) 13059);
			assert (pack.chan3_raw_GET() == (char) 33601);
			assert (pack.chan13_raw_GET() == (char) 16516);
			assert (pack.chan1_raw_GET() == (char) 50760);
			assert (pack.chan5_raw_GET() == (char) 21107);
			assert (pack.chan17_raw_GET() == (char) 51732);
			assert (pack.chan16_raw_GET() == (char) 37159);
			assert (pack.chan4_raw_GET() == (char) 13375);
			assert (pack.chancount_GET() == (char) 164);
			assert (pack.chan18_raw_GET() == (char) 31183);
			assert (pack.chan15_raw_GET() == (char) 34758);
		});
		RC_CHANNELS p65 = new RC_CHANNELS();
		PH.setPack(p65);
		p65.chan16_raw_SET((char) 37159);
		p65.chan8_raw_SET((char) 40678);
		p65.chan5_raw_SET((char) 21107);
		p65.chan18_raw_SET((char) 31183);
		p65.chan15_raw_SET((char) 34758);
		p65.rssi_SET((char) 223);
		p65.chan17_raw_SET((char) 51732);
		p65.chan13_raw_SET((char) 16516);
		p65.chan14_raw_SET((char) 52532);
		p65.chan7_raw_SET((char) 58976);
		p65.chan9_raw_SET((char) 6340);
		p65.chan2_raw_SET((char) 40627);
		p65.chan3_raw_SET((char) 33601);
		p65.chan12_raw_SET((char) 3189);
		p65.chan10_raw_SET((char) 33180);
		p65.chan11_raw_SET((char) 29193);
		p65.time_boot_ms_SET(2802030361L);
		p65.chan4_raw_SET((char) 13375);
		p65.chan6_raw_SET((char) 13059);
		p65.chancount_SET((char) 164);
		p65.chan1_raw_SET((char) 50760);
		TestChannel.instance.send(p65);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_REQUEST_DATA_STREAM.add((src, ph, pack) ->
		{
			assert (pack.start_stop_GET() == (char) 84);
			assert (pack.req_stream_id_GET() == (char) 21);
			assert (pack.target_system_GET() == (char) 24);
			assert (pack.req_message_rate_GET() == (char) 46191);
			assert (pack.target_component_GET() == (char) 202);
		});
		REQUEST_DATA_STREAM p66 = new REQUEST_DATA_STREAM();
		PH.setPack(p66);
		p66.req_stream_id_SET((char) 21);
		p66.target_system_SET((char) 24);
		p66.req_message_rate_SET((char) 46191);
		p66.start_stop_SET((char) 84);
		p66.target_component_SET((char) 202);
		TestChannel.instance.send(p66);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_DATA_STREAM.add((src, ph, pack) ->
		{
			assert (pack.message_rate_GET() == (char) 48012);
			assert (pack.stream_id_GET() == (char) 87);
			assert (pack.on_off_GET() == (char) 227);
		});
		DATA_STREAM p67 = new DATA_STREAM();
		PH.setPack(p67);
		p67.message_rate_SET((char) 48012);
		p67.on_off_SET((char) 227);
		p67.stream_id_SET((char) 87);
		TestChannel.instance.send(p67);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_MANUAL_CONTROL.add((src, ph, pack) ->
		{
			assert (pack.r_GET() == (short) 6472);
			assert (pack.z_GET() == (short) 31467);
			assert (pack.y_GET() == (short) -6454);
			assert (pack.buttons_GET() == (char) 58976);
			assert (pack.target_GET() == (char) 140);
			assert (pack.x_GET() == (short) 1294);
		});
		MANUAL_CONTROL p69 = new MANUAL_CONTROL();
		PH.setPack(p69);
		p69.y_SET((short) -6454);
		p69.buttons_SET((char) 58976);
		p69.target_SET((char) 140);
		p69.z_SET((short) 31467);
		p69.r_SET((short) 6472);
		p69.x_SET((short) 1294);
		TestChannel.instance.send(p69);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_RC_CHANNELS_OVERRIDE.add((src, ph, pack) ->
		{
			assert (pack.chan4_raw_GET() == (char) 8122);
			assert (pack.chan7_raw_GET() == (char) 33185);
			assert (pack.chan1_raw_GET() == (char) 8381);
			assert (pack.chan3_raw_GET() == (char) 8454);
			assert (pack.chan8_raw_GET() == (char) 2407);
			assert (pack.chan2_raw_GET() == (char) 63925);
			assert (pack.chan5_raw_GET() == (char) 10549);
			assert (pack.chan6_raw_GET() == (char) 60030);
			assert (pack.target_component_GET() == (char) 194);
			assert (pack.target_system_GET() == (char) 162);
		});
		RC_CHANNELS_OVERRIDE p70 = new RC_CHANNELS_OVERRIDE();
		PH.setPack(p70);
		p70.chan4_raw_SET((char) 8122);
		p70.chan1_raw_SET((char) 8381);
		p70.chan2_raw_SET((char) 63925);
		p70.chan7_raw_SET((char) 33185);
		p70.chan3_raw_SET((char) 8454);
		p70.target_component_SET((char) 194);
		p70.chan5_raw_SET((char) 10549);
		p70.chan8_raw_SET((char) 2407);
		p70.target_system_SET((char) 162);
		p70.chan6_raw_SET((char) 60030);
		TestChannel.instance.send(p70);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_MISSION_ITEM_INT.add((src, ph, pack) ->
		{
			assert (pack.current_GET() == (char) 222);
			assert (pack.target_system_GET() == (char) 245);
			assert (pack.seq_GET() == (char) 36943);
			assert (pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT);
			assert (pack.param2_GET() == -1.0634852E38F);
			assert (pack.command_GET() == MAV_CMD.MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL);
			assert (pack.z_GET() == -1.4727453E38F);
			assert (pack.param3_GET() == -3.0094903E38F);
			assert (pack.x_GET() == -1242954453);
			assert (pack.param1_GET() == -3.0286252E38F);
			assert (pack.autocontinue_GET() == (char) 200);
			assert (pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
			assert (pack.y_GET() == 1942518067);
			assert (pack.target_component_GET() == (char) 216);
			assert (pack.param4_GET() == 2.1381033E38F);
		});
		MISSION_ITEM_INT p73 = new MISSION_ITEM_INT();
		PH.setPack(p73);
		p73.command_SET(MAV_CMD.MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL);
		p73.current_SET((char) 222);
		p73.y_SET(1942518067);
		p73.autocontinue_SET((char) 200);
		p73.z_SET(-1.4727453E38F);
		p73.param1_SET(-3.0286252E38F);
		p73.param2_SET(-1.0634852E38F);
		p73.param3_SET(-3.0094903E38F);
		p73.x_SET(-1242954453);
		p73.param4_SET(2.1381033E38F);
		p73.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
		p73.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT);
		p73.target_system_SET((char) 245);
		p73.target_component_SET((char) 216);
		p73.seq_SET((char) 36943);
		TestChannel.instance.send(p73);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_VFR_HUD.add((src, ph, pack) ->
		{
			assert (pack.heading_GET() == (short) 20278);
			assert (pack.throttle_GET() == (char) 24661);
			assert (pack.climb_GET() == -1.1287069E38F);
			assert (pack.alt_GET() == -2.2664654E38F);
			assert (pack.groundspeed_GET() == 7.729128E37F);
			assert (pack.airspeed_GET() == -3.3407323E38F);
		});
		VFR_HUD p74 = new VFR_HUD();
		PH.setPack(p74);
		p74.heading_SET((short) 20278);
		p74.groundspeed_SET(7.729128E37F);
		p74.alt_SET(-2.2664654E38F);
		p74.airspeed_SET(-3.3407323E38F);
		p74.climb_SET(-1.1287069E38F);
		p74.throttle_SET((char) 24661);
		TestChannel.instance.send(p74);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_COMMAND_INT.add((src, ph, pack) ->
		{
			assert (pack.command_GET() == MAV_CMD.MAV_CMD_MISSION_START);
			assert (pack.current_GET() == (char) 75);
			assert (pack.param2_GET() == -5.5930453E37F);
			assert (pack.z_GET() == 9.282085E37F);
			assert (pack.x_GET() == -1436061612);
			assert (pack.param1_GET() == 2.9176165E38F);
			assert (pack.param3_GET() == -2.3986233E38F);
			assert (pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT);
			assert (pack.target_component_GET() == (char) 188);
			assert (pack.y_GET() == 1538051758);
			assert (pack.param4_GET() == -1.8072308E38F);
			assert (pack.autocontinue_GET() == (char) 19);
			assert (pack.target_system_GET() == (char) 235);
		});
		COMMAND_INT p75 = new COMMAND_INT();
		PH.setPack(p75);
		p75.y_SET(1538051758);
		p75.x_SET(-1436061612);
		p75.autocontinue_SET((char) 19);
		p75.param4_SET(-1.8072308E38F);
		p75.target_system_SET((char) 235);
		p75.target_component_SET((char) 188);
		p75.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT);
		p75.param2_SET(-5.5930453E37F);
		p75.param1_SET(2.9176165E38F);
		p75.param3_SET(-2.3986233E38F);
		p75.command_SET(MAV_CMD.MAV_CMD_MISSION_START);
		p75.z_SET(9.282085E37F);
		p75.current_SET((char) 75);
		TestChannel.instance.send(p75);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_COMMAND_LONG.add((src, ph, pack) ->
		{
			assert (pack.target_component_GET() == (char) 24);
			assert (pack.confirmation_GET() == (char) 91);
			assert (pack.param7_GET() == -1.3121998E38F);
			assert (pack.command_GET() == MAV_CMD.MAV_CMD_NAV_LOITER_TIME);
			assert (pack.param6_GET() == -1.0319123E38F);
			assert (pack.param4_GET() == 1.6043183E38F);
			assert (pack.param2_GET() == -2.4287488E38F);
			assert (pack.param5_GET() == -1.5923678E38F);
			assert (pack.param1_GET() == 2.9994593E38F);
			assert (pack.param3_GET() == 2.542347E37F);
			assert (pack.target_system_GET() == (char) 241);
		});
		COMMAND_LONG p76 = new COMMAND_LONG();
		PH.setPack(p76);
		p76.target_component_SET((char) 24);
		p76.target_system_SET((char) 241);
		p76.param7_SET(-1.3121998E38F);
		p76.param3_SET(2.542347E37F);
		p76.param1_SET(2.9994593E38F);
		p76.command_SET(MAV_CMD.MAV_CMD_NAV_LOITER_TIME);
		p76.param5_SET(-1.5923678E38F);
		p76.param2_SET(-2.4287488E38F);
		p76.confirmation_SET((char) 91);
		p76.param4_SET(1.6043183E38F);
		p76.param6_SET(-1.0319123E38F);
		TestChannel.instance.send(p76);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_COMMAND_ACK.add((src, ph, pack) ->
		{
			assert (pack.command_GET() == MAV_CMD.MAV_CMD_VIDEO_STOP_CAPTURE);
			assert (pack.target_component_TRY(ph) == (char) 17);
			assert (pack.result_GET() == MAV_RESULT.MAV_RESULT_DENIED);
			assert (pack.progress_TRY(ph) == (char) 209);
			assert (pack.result_param2_TRY(ph) == -1152681626);
			assert (pack.target_system_TRY(ph) == (char) 162);
		});
		COMMAND_ACK p77 = new COMMAND_ACK();
		PH.setPack(p77);
		p77.result_param2_SET(-1152681626, PH);
		p77.result_SET(MAV_RESULT.MAV_RESULT_DENIED);
		p77.target_component_SET((char) 17, PH);
		p77.target_system_SET((char) 162, PH);
		p77.progress_SET((char) 209, PH);
		p77.command_SET(MAV_CMD.MAV_CMD_VIDEO_STOP_CAPTURE);
		TestChannel.instance.send(p77);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_MANUAL_SETPOINT.add((src, ph, pack) ->
		{
			assert (pack.mode_switch_GET() == (char) 124);
			assert (pack.roll_GET() == 3.0942332E37F);
			assert (pack.thrust_GET() == 1.9174982E38F);
			assert (pack.time_boot_ms_GET() == 229162335L);
			assert (pack.manual_override_switch_GET() == (char) 43);
			assert (pack.yaw_GET() == -7.8449663E37F);
			assert (pack.pitch_GET() == -7.1978475E36F);
		});
		GroundControl.MANUAL_SETPOINT p81 = CommunicationChannel.new_MANUAL_SETPOINT();
		PH.setPack(p81);
		p81.manual_override_switch_SET((char) 43);
		p81.pitch_SET(-7.1978475E36F);
		p81.thrust_SET(1.9174982E38F);
		p81.yaw_SET(-7.8449663E37F);
		p81.mode_switch_SET((char) 124);
		p81.time_boot_ms_SET(229162335L);
		p81.roll_SET(3.0942332E37F);
		CommunicationChannel.instance.send(p81);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_SET_ATTITUDE_TARGET.add((src, ph, pack) ->
		{
			assert (pack.type_mask_GET() == (char) 203);
			assert (pack.body_pitch_rate_GET() == 2.3172935E38F);
			assert (pack.target_component_GET() == (char) 120);
			assert (pack.target_system_GET() == (char) 191);
			assert (Arrays.equals(pack.q_GET(), new float[]{-1.2211642E38F, -2.0457143E38F, -2.0825508E38F, 1.073393E38F}));
			assert (pack.thrust_GET() == -7.8792365E37F);
			assert (pack.body_roll_rate_GET() == 2.5978582E38F);
			assert (pack.body_yaw_rate_GET() == -2.9244705E38F);
			assert (pack.time_boot_ms_GET() == 3259508760L);
		});
		GroundControl.SET_ATTITUDE_TARGET p82 = CommunicationChannel.new_SET_ATTITUDE_TARGET();
		PH.setPack(p82);
		p82.body_pitch_rate_SET(2.3172935E38F);
		p82.body_roll_rate_SET(2.5978582E38F);
		p82.time_boot_ms_SET(3259508760L);
		p82.body_yaw_rate_SET(-2.9244705E38F);
		p82.thrust_SET(-7.8792365E37F);
		p82.type_mask_SET((char) 203);
		p82.target_component_SET((char) 120);
		p82.q_SET(new float[]{-1.2211642E38F, -2.0457143E38F, -2.0825508E38F, 1.073393E38F}, 0);
		p82.target_system_SET((char) 191);
		CommunicationChannel.instance.send(p82);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_ATTITUDE_TARGET.add((src, ph, pack) ->
		{
			assert (pack.time_boot_ms_GET() == 946591622L);
			assert (pack.body_yaw_rate_GET() == 2.9340183E38F);
			assert (pack.thrust_GET() == 1.0298918E37F);
			assert (pack.body_pitch_rate_GET() == -1.5290588E38F);
			assert (pack.type_mask_GET() == (char) 44);
			assert (pack.body_roll_rate_GET() == 2.7031755E38F);
			assert (Arrays.equals(pack.q_GET(), new float[]{1.1033149E38F, -6.514439E37F, 2.5065588E36F, 5.6906083E37F}));
		});
		GroundControl.ATTITUDE_TARGET p83 = CommunicationChannel.new_ATTITUDE_TARGET();
		PH.setPack(p83);
		p83.type_mask_SET((char) 44);
		p83.body_yaw_rate_SET(2.9340183E38F);
		p83.thrust_SET(1.0298918E37F);
		p83.body_pitch_rate_SET(-1.5290588E38F);
		p83.time_boot_ms_SET(946591622L);
		p83.q_SET(new float[]{1.1033149E38F, -6.514439E37F, 2.5065588E36F, 5.6906083E37F}, 0);
		p83.body_roll_rate_SET(2.7031755E38F);
		CommunicationChannel.instance.send(p83);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_SET_POSITION_TARGET_LOCAL_NED.add((src, ph, pack) ->
		{
			assert (pack.type_mask_GET() == (char) 36186);
			assert (pack.yaw_rate_GET() == 2.3317285E38F);
			assert (pack.afx_GET() == 5.7540004E37F);
			assert (pack.afz_GET() == -6.1186924E37F);
			assert (pack.vy_GET() == 2.6923E38F);
			assert (pack.afy_GET() == 3.1096333E38F);
			assert (pack.target_component_GET() == (char) 0);
			assert (pack.y_GET() == 2.0695867E38F);
			assert (pack.yaw_GET() == -1.6894105E38F);
			assert (pack.vz_GET() == -1.7624465E38F);
			assert (pack.z_GET() == 1.8681955E38F);
			assert (pack.vx_GET() == -1.2614465E38F);
			assert (pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED);
			assert (pack.time_boot_ms_GET() == 3803048021L);
			assert (pack.target_system_GET() == (char) 144);
			assert (pack.x_GET() == 3.9580433E37F);
		});
		GroundControl.SET_POSITION_TARGET_LOCAL_NED p84 = CommunicationChannel.new_SET_POSITION_TARGET_LOCAL_NED();
		PH.setPack(p84);
		p84.type_mask_SET((char) 36186);
		p84.z_SET(1.8681955E38F);
		p84.afy_SET(3.1096333E38F);
		p84.target_component_SET((char) 0);
		p84.vx_SET(-1.2614465E38F);
		p84.vz_SET(-1.7624465E38F);
		p84.yaw_SET(-1.6894105E38F);
		p84.afx_SET(5.7540004E37F);
		p84.time_boot_ms_SET(3803048021L);
		p84.y_SET(2.0695867E38F);
		p84.afz_SET(-6.1186924E37F);
		p84.x_SET(3.9580433E37F);
		p84.yaw_rate_SET(2.3317285E38F);
		p84.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED);
		p84.vy_SET(2.6923E38F);
		p84.target_system_SET((char) 144);
		CommunicationChannel.instance.send(p84);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_SET_POSITION_TARGET_GLOBAL_INT.add((src, ph, pack) ->
		{
			assert (pack.afy_GET() == 2.4299838E38F);
			assert (pack.yaw_rate_GET() == -2.1602292E37F);
			assert (pack.lat_int_GET() == 1454022393);
			assert (pack.afz_GET() == 9.917173E37F);
			assert (pack.target_component_GET() == (char) 205);
			assert (pack.alt_GET() == 2.8582059E38F);
			assert (pack.time_boot_ms_GET() == 4081180783L);
			assert (pack.yaw_GET() == 1.7820774E38F);
			assert (pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_INT);
			assert (pack.target_system_GET() == (char) 12);
			assert (pack.vz_GET() == -2.0651558E38F);
			assert (pack.type_mask_GET() == (char) 7856);
			assert (pack.afx_GET() == 3.1518464E37F);
			assert (pack.vx_GET() == -2.2641562E38F);
			assert (pack.lon_int_GET() == 2004847842);
			assert (pack.vy_GET() == -8.0143457E37F);
		});
		GroundControl.SET_POSITION_TARGET_GLOBAL_INT p86 = CommunicationChannel.new_SET_POSITION_TARGET_GLOBAL_INT();
		PH.setPack(p86);
		p86.vz_SET(-2.0651558E38F);
		p86.yaw_rate_SET(-2.1602292E37F);
		p86.target_component_SET((char) 205);
		p86.lon_int_SET(2004847842);
		p86.afz_SET(9.917173E37F);
		p86.alt_SET(2.8582059E38F);
		p86.vy_SET(-8.0143457E37F);
		p86.type_mask_SET((char) 7856);
		p86.afx_SET(3.1518464E37F);
		p86.time_boot_ms_SET(4081180783L);
		p86.lat_int_SET(1454022393);
		p86.target_system_SET((char) 12);
		p86.afy_SET(2.4299838E38F);
		p86.yaw_SET(1.7820774E38F);
		p86.vx_SET(-2.2641562E38F);
		p86.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_INT);
		CommunicationChannel.instance.send(p86);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_POSITION_TARGET_GLOBAL_INT.add((src, ph, pack) ->
		{
			assert (pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL);
			assert (pack.vx_GET() == -7.9858905E37F);
			assert (pack.lat_int_GET() == -44941683);
			assert (pack.lon_int_GET() == -2017952548);
			assert (pack.alt_GET() == -2.8303403E38F);
			assert (pack.yaw_rate_GET() == 1.1333783E38F);
			assert (pack.afx_GET() == -1.0573698E38F);
			assert (pack.vz_GET() == 1.5702985E38F);
			assert (pack.afy_GET() == -1.8352788E37F);
			assert (pack.yaw_GET() == -2.4954637E38F);
			assert (pack.type_mask_GET() == (char) 53421);
			assert (pack.time_boot_ms_GET() == 1038466449L);
			assert (pack.vy_GET() == 2.8385443E38F);
			assert (pack.afz_GET() == -7.524879E37F);
		});
		GroundControl.POSITION_TARGET_GLOBAL_INT p87 = CommunicationChannel.new_POSITION_TARGET_GLOBAL_INT();
		PH.setPack(p87);
		p87.afx_SET(-1.0573698E38F);
		p87.vz_SET(1.5702985E38F);
		p87.yaw_rate_SET(1.1333783E38F);
		p87.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL);
		p87.afy_SET(-1.8352788E37F);
		p87.time_boot_ms_SET(1038466449L);
		p87.lon_int_SET(-2017952548);
		p87.alt_SET(-2.8303403E38F);
		p87.yaw_SET(-2.4954637E38F);
		p87.vy_SET(2.8385443E38F);
		p87.vx_SET(-7.9858905E37F);
		p87.lat_int_SET(-44941683);
		p87.afz_SET(-7.524879E37F);
		p87.type_mask_SET((char) 53421);
		CommunicationChannel.instance.send(p87);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET.add((src, ph, pack) ->
		{
			assert (pack.z_GET() == -2.2255209E38F);
			assert (pack.roll_GET() == 1.2609442E38F);
			assert (pack.yaw_GET() == 2.1264642E38F);
			assert (pack.y_GET() == -2.131994E38F);
			assert (pack.pitch_GET() == -6.3467493E37F);
			assert (pack.x_GET() == -1.0894003E38F);
			assert (pack.time_boot_ms_GET() == 2949065633L);
		});
		GroundControl.LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET p89 = CommunicationChannel.new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
		PH.setPack(p89);
		p89.roll_SET(1.2609442E38F);
		p89.y_SET(-2.131994E38F);
		p89.yaw_SET(2.1264642E38F);
		p89.pitch_SET(-6.3467493E37F);
		p89.time_boot_ms_SET(2949065633L);
		p89.x_SET(-1.0894003E38F);
		p89.z_SET(-2.2255209E38F);
		CommunicationChannel.instance.send(p89);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_HIL_STATE.add((src, ph, pack) ->
		{
			assert (pack.time_usec_GET() == 4780559911672496007L);
			assert (pack.roll_GET() == -2.8949905E38F);
			assert (pack.zacc_GET() == (short) -31222);
			assert (pack.vz_GET() == (short) -12465);
			assert (pack.lon_GET() == -926680151);
			assert (pack.yacc_GET() == (short) 15456);
			assert (pack.lat_GET() == -1314858818);
			assert (pack.vx_GET() == (short) -7234);
			assert (pack.yawspeed_GET() == -7.6597326E37F);
			assert (pack.alt_GET() == 688931134);
			assert (pack.pitchspeed_GET() == -1.952972E38F);
			assert (pack.rollspeed_GET() == 1.2019268E38F);
			assert (pack.vy_GET() == (short) 29083);
			assert (pack.pitch_GET() == 1.7134104E37F);
			assert (pack.xacc_GET() == (short) 543);
			assert (pack.yaw_GET() == 3.1045635E38F);
		});
		GroundControl.HIL_STATE p90 = CommunicationChannel.new_HIL_STATE();
		PH.setPack(p90);
		p90.yacc_SET((short) 15456);
		p90.vy_SET((short) 29083);
		p90.pitchspeed_SET(-1.952972E38F);
		p90.vx_SET((short) -7234);
		p90.xacc_SET((short) 543);
		p90.rollspeed_SET(1.2019268E38F);
		p90.alt_SET(688931134);
		p90.yawspeed_SET(-7.6597326E37F);
		p90.vz_SET((short) -12465);
		p90.pitch_SET(1.7134104E37F);
		p90.roll_SET(-2.8949905E38F);
		p90.lat_SET(-1314858818);
		p90.zacc_SET((short) -31222);
		p90.yaw_SET(3.1045635E38F);
		p90.time_usec_SET(4780559911672496007L);
		p90.lon_SET(-926680151);
		CommunicationChannel.instance.send(p90);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_HIL_CONTROLS.add((src, ph, pack) ->
		{
			assert (pack.aux3_GET() == -1.4152336E38F);
			assert (pack.yaw_rudder_GET() == -5.3671804E37F);
			assert (pack.roll_ailerons_GET() == 2.7200752E37F);
			assert (pack.pitch_elevator_GET() == -2.9266408E38F);
			assert (pack.aux1_GET() == -3.293661E38F);
			assert (pack.nav_mode_GET() == (char) 5);
			assert (pack.mode_GET() == MAV_MODE.MAV_MODE_GUIDED_DISARMED);
			assert (pack.throttle_GET() == 2.4571991E38F);
			assert (pack.aux4_GET() == 7.346599E37F);
			assert (pack.aux2_GET() == 3.215032E38F);
			assert (pack.time_usec_GET() == 7898765458321376194L);
		});
		GroundControl.HIL_CONTROLS p91 = CommunicationChannel.new_HIL_CONTROLS();
		PH.setPack(p91);
		p91.time_usec_SET(7898765458321376194L);
		p91.nav_mode_SET((char) 5);
		p91.aux4_SET(7.346599E37F);
		p91.roll_ailerons_SET(2.7200752E37F);
		p91.aux1_SET(-3.293661E38F);
		p91.aux3_SET(-1.4152336E38F);
		p91.aux2_SET(3.215032E38F);
		p91.pitch_elevator_SET(-2.9266408E38F);
		p91.yaw_rudder_SET(-5.3671804E37F);
		p91.throttle_SET(2.4571991E38F);
		p91.mode_SET(MAV_MODE.MAV_MODE_GUIDED_DISARMED);
		CommunicationChannel.instance.send(p91);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_HIL_RC_INPUTS_RAW.add((src, ph, pack) ->
		{
			assert (pack.chan7_raw_GET() == (char) 26193);
			assert (pack.chan3_raw_GET() == (char) 13945);
			assert (pack.rssi_GET() == (char) 75);
			assert (pack.time_usec_GET() == 3774394175889506460L);
			assert (pack.chan1_raw_GET() == (char) 55591);
			assert (pack.chan5_raw_GET() == (char) 37710);
			assert (pack.chan2_raw_GET() == (char) 5086);
			assert (pack.chan12_raw_GET() == (char) 5728);
			assert (pack.chan6_raw_GET() == (char) 3675);
			assert (pack.chan9_raw_GET() == (char) 50928);
			assert (pack.chan8_raw_GET() == (char) 4481);
			assert (pack.chan4_raw_GET() == (char) 65327);
			assert (pack.chan10_raw_GET() == (char) 47699);
			assert (pack.chan11_raw_GET() == (char) 543);
		});
		GroundControl.HIL_RC_INPUTS_RAW p92 = CommunicationChannel.new_HIL_RC_INPUTS_RAW();
		PH.setPack(p92);
		p92.chan11_raw_SET((char) 543);
		p92.chan6_raw_SET((char) 3675);
		p92.chan8_raw_SET((char) 4481);
		p92.chan4_raw_SET((char) 65327);
		p92.time_usec_SET(3774394175889506460L);
		p92.rssi_SET((char) 75);
		p92.chan2_raw_SET((char) 5086);
		p92.chan1_raw_SET((char) 55591);
		p92.chan10_raw_SET((char) 47699);
		p92.chan7_raw_SET((char) 26193);
		p92.chan12_raw_SET((char) 5728);
		p92.chan5_raw_SET((char) 37710);
		p92.chan9_raw_SET((char) 50928);
		p92.chan3_raw_SET((char) 13945);
		CommunicationChannel.instance.send(p92);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_HIL_ACTUATOR_CONTROLS.add((src, ph, pack) ->
		{
			assert (pack.mode_GET() == MAV_MODE.MAV_MODE_MANUAL_DISARMED);
			assert (pack.time_usec_GET() == 8523713098688646245L);
			assert (Arrays.equals(pack.controls_GET(), new float[]{-9.294511E37F, -1.5729241E38F, 8.398638E37F, -2.8051781E38F, -2.1603405E38F, -2.7521741E38F, 5.532633E37F, 3.3026324E37F, 1.5927054E38F, 7.8958234E37F, 7.3710997E37F, -1.336996E38F, 8.0556716E37F, 2.9253275E38F, -7.4438446E37F, -2.4191897E38F}));
			assert (pack.flags_GET() == 997162162665333322L);
		});
		GroundControl.HIL_ACTUATOR_CONTROLS p93 = CommunicationChannel.new_HIL_ACTUATOR_CONTROLS();
		PH.setPack(p93);
		p93.mode_SET(MAV_MODE.MAV_MODE_MANUAL_DISARMED);
		p93.flags_SET(997162162665333322L);
		p93.controls_SET(new float[]{-9.294511E37F, -1.5729241E38F, 8.398638E37F, -2.8051781E38F, -2.1603405E38F, -2.7521741E38F, 5.532633E37F, 3.3026324E37F, 1.5927054E38F, 7.8958234E37F, 7.3710997E37F, -1.336996E38F, 8.0556716E37F, 2.9253275E38F, -7.4438446E37F, -2.4191897E38F}, 0);
		p93.time_usec_SET(8523713098688646245L);
		CommunicationChannel.instance.send(p93);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_OPTICAL_FLOW.add((src, ph, pack) ->
		{
			assert (pack.time_usec_GET() == 949444714200247848L);
			assert (pack.flow_x_GET() == (short) 25605);
			assert (pack.ground_distance_GET() == 3.9557166E37F);
			assert (pack.flow_comp_m_y_GET() == 1.7401926E38F);
			assert (pack.sensor_id_GET() == (char) 182);
			assert (pack.flow_y_GET() == (short) 29254);
			assert (pack.flow_rate_x_TRY(ph) == 2.5840603E38F);
			assert (pack.flow_rate_y_TRY(ph) == -1.3104444E38F);
			assert (pack.quality_GET() == (char) 28);
			assert (pack.flow_comp_m_x_GET() == -3.2004299E37F);
		});
		GroundControl.OPTICAL_FLOW p100 = CommunicationChannel.new_OPTICAL_FLOW();
		PH.setPack(p100);
		p100.flow_rate_y_SET(-1.3104444E38F, PH);
		p100.flow_comp_m_y_SET(1.7401926E38F);
		p100.flow_x_SET((short) 25605);
		p100.time_usec_SET(949444714200247848L);
		p100.flow_y_SET((short) 29254);
		p100.flow_rate_x_SET(2.5840603E38F, PH);
		p100.ground_distance_SET(3.9557166E37F);
		p100.sensor_id_SET((char) 182);
		p100.quality_SET((char) 28);
		p100.flow_comp_m_x_SET(-3.2004299E37F);
		CommunicationChannel.instance.send(p100);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_GLOBAL_VISION_POSITION_ESTIMATE.add((src, ph, pack) ->
		{
			assert (pack.usec_GET() == 5205352187475053032L);
			assert (pack.yaw_GET() == -1.031869E38F);
			assert (pack.x_GET() == -2.2531062E38F);
			assert (pack.roll_GET() == -4.539851E37F);
			assert (pack.y_GET() == 2.4975923E38F);
			assert (pack.pitch_GET() == 2.1052301E38F);
			assert (pack.z_GET() == -3.845677E36F);
		});
		GroundControl.GLOBAL_VISION_POSITION_ESTIMATE p101 = CommunicationChannel.new_GLOBAL_VISION_POSITION_ESTIMATE();
		PH.setPack(p101);
		p101.yaw_SET(-1.031869E38F);
		p101.usec_SET(5205352187475053032L);
		p101.roll_SET(-4.539851E37F);
		p101.y_SET(2.4975923E38F);
		p101.z_SET(-3.845677E36F);
		p101.pitch_SET(2.1052301E38F);
		p101.x_SET(-2.2531062E38F);
		CommunicationChannel.instance.send(p101);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_VISION_POSITION_ESTIMATE.add((src, ph, pack) ->
		{
			assert (pack.roll_GET() == -2.951075E36F);
			assert (pack.z_GET() == -2.1119708E38F);
			assert (pack.x_GET() == 2.69159E38F);
			assert (pack.yaw_GET() == -2.6182642E38F);
			assert (pack.y_GET() == -1.3153538E38F);
			assert (pack.usec_GET() == 6496120578258713112L);
			assert (pack.pitch_GET() == 7.438606E37F);
		});
		GroundControl.VISION_POSITION_ESTIMATE p102 = CommunicationChannel.new_VISION_POSITION_ESTIMATE();
		PH.setPack(p102);
		p102.roll_SET(-2.951075E36F);
		p102.x_SET(2.69159E38F);
		p102.yaw_SET(-2.6182642E38F);
		p102.pitch_SET(7.438606E37F);
		p102.y_SET(-1.3153538E38F);
		p102.z_SET(-2.1119708E38F);
		p102.usec_SET(6496120578258713112L);
		CommunicationChannel.instance.send(p102);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_VISION_SPEED_ESTIMATE.add((src, ph, pack) ->
		{
			assert (pack.y_GET() == -2.9248398E37F);
			assert (pack.usec_GET() == 4786229125146402315L);
			assert (pack.x_GET() == -3.7359215E37F);
			assert (pack.z_GET() == -1.5690246E38F);
		});
		GroundControl.VISION_SPEED_ESTIMATE p103 = CommunicationChannel.new_VISION_SPEED_ESTIMATE();
		PH.setPack(p103);
		p103.x_SET(-3.7359215E37F);
		p103.z_SET(-1.5690246E38F);
		p103.y_SET(-2.9248398E37F);
		p103.usec_SET(4786229125146402315L);
		CommunicationChannel.instance.send(p103);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_VICON_POSITION_ESTIMATE.add((src, ph, pack) ->
		{
			assert (pack.x_GET() == -1.2650532E38F);
			assert (pack.roll_GET() == 2.7036746E38F);
			assert (pack.z_GET() == 1.929791E38F);
			assert (pack.y_GET() == -2.6073601E38F);
			assert (pack.usec_GET() == 7851762335550842040L);
			assert (pack.pitch_GET() == -3.0272614E38F);
			assert (pack.yaw_GET() == 9.805615E35F);
		});
		GroundControl.VICON_POSITION_ESTIMATE p104 = CommunicationChannel.new_VICON_POSITION_ESTIMATE();
		PH.setPack(p104);
		p104.y_SET(-2.6073601E38F);
		p104.z_SET(1.929791E38F);
		p104.usec_SET(7851762335550842040L);
		p104.yaw_SET(9.805615E35F);
		p104.pitch_SET(-3.0272614E38F);
		p104.x_SET(-1.2650532E38F);
		p104.roll_SET(2.7036746E38F);
		CommunicationChannel.instance.send(p104);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_HIGHRES_IMU.add((src, ph, pack) ->
		{
			assert (pack.ygyro_GET() == 7.7873485E37F);
			assert (pack.xgyro_GET() == -2.4675997E38F);
			assert (pack.abs_pressure_GET() == -1.4588652E38F);
			assert (pack.xacc_GET() == -1.1727743E38F);
			assert (pack.temperature_GET() == -2.31629E38F);
			assert (pack.diff_pressure_GET() == -1.7053316E38F);
			assert (pack.zgyro_GET() == 9.406954E37F);
			assert (pack.xmag_GET() == 1.5483422E38F);
			assert (pack.time_usec_GET() == 4692356997219041653L);
			assert (pack.pressure_alt_GET() == 2.4490342E38F);
			assert (pack.ymag_GET() == -2.3232679E38F);
			assert (pack.yacc_GET() == 2.8795369E38F);
			assert (pack.zacc_GET() == 3.334336E38F);
			assert (pack.zmag_GET() == 1.8196133E38F);
			assert (pack.fields_updated_GET() == (char) 6505);
		});
		GroundControl.HIGHRES_IMU p105 = CommunicationChannel.new_HIGHRES_IMU();
		PH.setPack(p105);
		p105.fields_updated_SET((char) 6505);
		p105.xmag_SET(1.5483422E38F);
		p105.ygyro_SET(7.7873485E37F);
		p105.yacc_SET(2.8795369E38F);
		p105.zacc_SET(3.334336E38F);
		p105.zmag_SET(1.8196133E38F);
		p105.time_usec_SET(4692356997219041653L);
		p105.temperature_SET(-2.31629E38F);
		p105.pressure_alt_SET(2.4490342E38F);
		p105.xgyro_SET(-2.4675997E38F);
		p105.zgyro_SET(9.406954E37F);
		p105.diff_pressure_SET(-1.7053316E38F);
		p105.abs_pressure_SET(-1.4588652E38F);
		p105.xacc_SET(-1.1727743E38F);
		p105.ymag_SET(-2.3232679E38F);
		CommunicationChannel.instance.send(p105);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_OPTICAL_FLOW_RAD.add((src, ph, pack) ->
		{
			assert (pack.integrated_y_GET() == -1.9067067E38F);
			assert (pack.time_usec_GET() == 80886617018836975L);
			assert (pack.sensor_id_GET() == (char) 72);
			assert (pack.temperature_GET() == (short) -13640);
			assert (pack.integrated_x_GET() == 1.9819871E38F);
			assert (pack.integrated_ygyro_GET() == -2.2727547E38F);
			assert (pack.integration_time_us_GET() == 356650585L);
			assert (pack.integrated_xgyro_GET() == -8.4721957E37F);
			assert (pack.quality_GET() == (char) 118);
			assert (pack.time_delta_distance_us_GET() == 343371207L);
			assert (pack.distance_GET() == 2.4526548E38F);
			assert (pack.integrated_zgyro_GET() == 3.3152446E38F);
		});
		GroundControl.OPTICAL_FLOW_RAD p106 = CommunicationChannel.new_OPTICAL_FLOW_RAD();
		PH.setPack(p106);
		p106.sensor_id_SET((char) 72);
		p106.integrated_y_SET(-1.9067067E38F);
		p106.quality_SET((char) 118);
		p106.temperature_SET((short) -13640);
		p106.integrated_ygyro_SET(-2.2727547E38F);
		p106.time_delta_distance_us_SET(343371207L);
		p106.integration_time_us_SET(356650585L);
		p106.integrated_x_SET(1.9819871E38F);
		p106.distance_SET(2.4526548E38F);
		p106.integrated_zgyro_SET(3.3152446E38F);
		p106.time_usec_SET(80886617018836975L);
		p106.integrated_xgyro_SET(-8.4721957E37F);
		CommunicationChannel.instance.send(p106);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_HIL_SENSOR.add((src, ph, pack) ->
		{
			assert (pack.abs_pressure_GET() == -2.301795E38F);
			assert (pack.time_usec_GET() == 6788395461116438738L);
			assert (pack.ymag_GET() == 3.3324068E38F);
			assert (pack.zgyro_GET() == -2.0420924E38F);
			assert (pack.xacc_GET() == 2.9247403E38F);
			assert (pack.diff_pressure_GET() == 2.181521E38F);
			assert (pack.zmag_GET() == 3.2193812E38F);
			assert (pack.ygyro_GET() == 4.491238E37F);
			assert (pack.pressure_alt_GET() == 1.3790087E38F);
			assert (pack.fields_updated_GET() == 1105172950L);
			assert (pack.xgyro_GET() == -3.0460603E38F);
			assert (pack.yacc_GET() == 2.205317E38F);
			assert (pack.zacc_GET() == -9.212427E37F);
			assert (pack.temperature_GET() == 2.46902E38F);
			assert (pack.xmag_GET() == -3.187713E38F);
		});
		GroundControl.HIL_SENSOR p107 = CommunicationChannel.new_HIL_SENSOR();
		PH.setPack(p107);
		p107.zgyro_SET(-2.0420924E38F);
		p107.xgyro_SET(-3.0460603E38F);
		p107.ymag_SET(3.3324068E38F);
		p107.ygyro_SET(4.491238E37F);
		p107.zmag_SET(3.2193812E38F);
		p107.pressure_alt_SET(1.3790087E38F);
		p107.xmag_SET(-3.187713E38F);
		p107.yacc_SET(2.205317E38F);
		p107.temperature_SET(2.46902E38F);
		p107.abs_pressure_SET(-2.301795E38F);
		p107.xacc_SET(2.9247403E38F);
		p107.fields_updated_SET(1105172950L);
		p107.time_usec_SET(6788395461116438738L);
		p107.zacc_SET(-9.212427E37F);
		p107.diff_pressure_SET(2.181521E38F);
		CommunicationChannel.instance.send(p107);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_SIM_STATE.add((src, ph, pack) ->
		{
			assert (pack.ygyro_GET() == 2.5624236E38F);
			assert (pack.lat_GET() == -1.977819E38F);
			assert (pack.q3_GET() == 2.2307292E38F);
			assert (pack.alt_GET() == 1.9482784E38F);
			assert (pack.vd_GET() == 1.1554916E38F);
			assert (pack.q4_GET() == -2.1596696E38F);
			assert (pack.xgyro_GET() == -4.4097665E37F);
			assert (pack.q1_GET() == -1.1852303E38F);
			assert (pack.q2_GET() == -3.3559048E38F);
			assert (pack.pitch_GET() == 2.7300367E37F);
			assert (pack.xacc_GET() == -2.346729E38F);
			assert (pack.zgyro_GET() == -2.0740311E38F);
			assert (pack.yaw_GET() == 8.669791E37F);
			assert (pack.zacc_GET() == -6.754433E37F);
			assert (pack.roll_GET() == -1.3460235E38F);
			assert (pack.lon_GET() == 2.819258E37F);
			assert (pack.std_dev_vert_GET() == -1.3443444E38F);
			assert (pack.ve_GET() == -2.0092873E38F);
			assert (pack.vn_GET() == -9.929312E37F);
			assert (pack.yacc_GET() == -5.8181157E37F);
			assert (pack.std_dev_horz_GET() == -1.1569507E38F);
		});
		GroundControl.SIM_STATE p108 = CommunicationChannel.new_SIM_STATE();
		PH.setPack(p108);
		p108.q4_SET(-2.1596696E38F);
		p108.lon_SET(2.819258E37F);
		p108.ygyro_SET(2.5624236E38F);
		p108.q2_SET(-3.3559048E38F);
		p108.lat_SET(-1.977819E38F);
		p108.ve_SET(-2.0092873E38F);
		p108.yacc_SET(-5.8181157E37F);
		p108.q1_SET(-1.1852303E38F);
		p108.zgyro_SET(-2.0740311E38F);
		p108.q3_SET(2.2307292E38F);
		p108.pitch_SET(2.7300367E37F);
		p108.std_dev_horz_SET(-1.1569507E38F);
		p108.yaw_SET(8.669791E37F);
		p108.vn_SET(-9.929312E37F);
		p108.zacc_SET(-6.754433E37F);
		p108.xgyro_SET(-4.4097665E37F);
		p108.xacc_SET(-2.346729E38F);
		p108.alt_SET(1.9482784E38F);
		p108.roll_SET(-1.3460235E38F);
		p108.vd_SET(1.1554916E38F);
		p108.std_dev_vert_SET(-1.3443444E38F);
		CommunicationChannel.instance.send(p108);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_RADIO_STATUS.add((src, ph, pack) ->
		{
			assert (pack.noise_GET() == (char) 155);
			assert (pack.remnoise_GET() == (char) 212);
			assert (pack.rxerrors_GET() == (char) 61339);
			assert (pack.rssi_GET() == (char) 201);
			assert (pack.txbuf_GET() == (char) 232);
			assert (pack.remrssi_GET() == (char) 249);
			assert (pack.fixed__GET() == (char) 4825);
		});
		GroundControl.RADIO_STATUS p109 = CommunicationChannel.new_RADIO_STATUS();
		PH.setPack(p109);
		p109.rxerrors_SET((char) 61339);
		p109.fixed__SET((char) 4825);
		p109.txbuf_SET((char) 232);
		p109.noise_SET((char) 155);
		p109.remrssi_SET((char) 249);
		p109.rssi_SET((char) 201);
		p109.remnoise_SET((char) 212);
		CommunicationChannel.instance.send(p109);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_FILE_TRANSFER_PROTOCOL.add((src, ph, pack) ->
		{
			assert (pack.target_network_GET() == (char) 175);
			assert (pack.target_component_GET() == (char) 87);
			assert (Arrays.equals(pack.payload_GET(), new char[]{(char) 6, (char) 4, (char) 42, (char) 63, (char) 213, (char) 167, (char) 226, (char) 146, (char) 237, (char) 172, (char) 150, (char) 12, (char) 40, (char) 19, (char) 96, (char) 154, (char) 163, (char) 104, (char) 233, (char) 106, (char) 94, (char) 137, (char) 190, (char) 14, (char) 143, (char) 140, (char) 174, (char) 110, (char) 163, (char) 214, (char) 130, (char) 7, (char) 225, (char) 95, (char) 157, (char) 21, (char) 30, (char) 46, (char) 95, (char) 4, (char) 46, (char) 185, (char) 130, (char) 161, (char) 78, (char) 38, (char) 130, (char) 201, (char) 174, (char) 146, (char) 47, (char) 240, (char) 132, (char) 46, (char) 252, (char) 243, (char) 133, (char) 87, (char) 51, (char) 183, (char) 90, (char) 203, (char) 10, (char) 205, (char) 126, (char) 112, (char) 200, (char) 250, (char) 124, (char) 143, (char) 46, (char) 166, (char) 166, (char) 199, (char) 116, (char) 90, (char) 87, (char) 141, (char) 45, (char) 61, (char) 240, (char) 101, (char) 60, (char) 104, (char) 49, (char) 87, (char) 34, (char) 119, (char) 57, (char) 211, (char) 130, (char) 250, (char) 159, (char) 75, (char) 121, (char) 183, (char) 181, (char) 210, (char) 182, (char) 163, (char) 13, (char) 243, (char) 223, (char) 6, (char) 202, (char) 118, (char) 100, (char) 215, (char) 180, (char) 138, (char) 237, (char) 213, (char) 215, (char) 14, (char) 233, (char) 153, (char) 239, (char) 79, (char) 204, (char) 20, (char) 99, (char) 92, (char) 126, (char) 236, (char) 148, (char) 189, (char) 130, (char) 23, (char) 68, (char) 46, (char) 222, (char) 52, (char) 162, (char) 173, (char) 1, (char) 47, (char) 43, (char) 65, (char) 110, (char) 126, (char) 151, (char) 215, (char) 7, (char) 44, (char) 125, (char) 153, (char) 151, (char) 50, (char) 112, (char) 139, (char) 94, (char) 216, (char) 168, (char) 22, (char) 78, (char) 15, (char) 64, (char) 77, (char) 134, (char) 14, (char) 135, (char) 140, (char) 225, (char) 73, (char) 12, (char) 3, (char) 201, (char) 156, (char) 153, (char) 47, (char) 155, (char) 186, (char) 38, (char) 251, (char) 111, (char) 111, (char) 22, (char) 3, (char) 15, (char) 219, (char) 134, (char) 180, (char) 86, (char) 98, (char) 39, (char) 113, (char) 129, (char) 144, (char) 162, (char) 194, (char) 98, (char) 157, (char) 84, (char) 162, (char) 205, (char) 149, (char) 228, (char) 218, (char) 172, (char) 89, (char) 103, (char) 140, (char) 146, (char) 70, (char) 158, (char) 61, (char) 187, (char) 190, (char) 255, (char) 145, (char) 190, (char) 211, (char) 244, (char) 147, (char) 92, (char) 238, (char) 246, (char) 155, (char) 141, (char) 165, (char) 162, (char) 174, (char) 225, (char) 153, (char) 35, (char) 169, (char) 222, (char) 251, (char) 142, (char) 175, (char) 172, (char) 107, (char) 227, (char) 112, (char) 145, (char) 9, (char) 2, (char) 146, (char) 133, (char) 106, (char) 121, (char) 243, (char) 22, (char) 255, (char) 235, (char) 101, (char) 239, (char) 34, (char) 82, (char) 127, (char) 212}));
			assert (pack.target_system_GET() == (char) 234);
		});
		GroundControl.FILE_TRANSFER_PROTOCOL p110 = CommunicationChannel.new_FILE_TRANSFER_PROTOCOL();
		PH.setPack(p110);
		p110.target_network_SET((char) 175);
		p110.target_system_SET((char) 234);
		p110.payload_SET(new char[]{(char) 6, (char) 4, (char) 42, (char) 63, (char) 213, (char) 167, (char) 226, (char) 146, (char) 237, (char) 172, (char) 150, (char) 12, (char) 40, (char) 19, (char) 96, (char) 154, (char) 163, (char) 104, (char) 233, (char) 106, (char) 94, (char) 137, (char) 190, (char) 14, (char) 143, (char) 140, (char) 174, (char) 110, (char) 163, (char) 214, (char) 130, (char) 7, (char) 225, (char) 95, (char) 157, (char) 21, (char) 30, (char) 46, (char) 95, (char) 4, (char) 46, (char) 185, (char) 130, (char) 161, (char) 78, (char) 38, (char) 130, (char) 201, (char) 174, (char) 146, (char) 47, (char) 240, (char) 132, (char) 46, (char) 252, (char) 243, (char) 133, (char) 87, (char) 51, (char) 183, (char) 90, (char) 203, (char) 10, (char) 205, (char) 126, (char) 112, (char) 200, (char) 250, (char) 124, (char) 143, (char) 46, (char) 166, (char) 166, (char) 199, (char) 116, (char) 90, (char) 87, (char) 141, (char) 45, (char) 61, (char) 240, (char) 101, (char) 60, (char) 104, (char) 49, (char) 87, (char) 34, (char) 119, (char) 57, (char) 211, (char) 130, (char) 250, (char) 159, (char) 75, (char) 121, (char) 183, (char) 181, (char) 210, (char) 182, (char) 163, (char) 13, (char) 243, (char) 223, (char) 6, (char) 202, (char) 118, (char) 100, (char) 215, (char) 180, (char) 138, (char) 237, (char) 213, (char) 215, (char) 14, (char) 233, (char) 153, (char) 239, (char) 79, (char) 204, (char) 20, (char) 99, (char) 92, (char) 126, (char) 236, (char) 148, (char) 189, (char) 130, (char) 23, (char) 68, (char) 46, (char) 222, (char) 52, (char) 162, (char) 173, (char) 1, (char) 47, (char) 43, (char) 65, (char) 110, (char) 126, (char) 151, (char) 215, (char) 7, (char) 44, (char) 125, (char) 153, (char) 151, (char) 50, (char) 112, (char) 139, (char) 94, (char) 216, (char) 168, (char) 22, (char) 78, (char) 15, (char) 64, (char) 77, (char) 134, (char) 14, (char) 135, (char) 140, (char) 225, (char) 73, (char) 12, (char) 3, (char) 201, (char) 156, (char) 153, (char) 47, (char) 155, (char) 186, (char) 38, (char) 251, (char) 111, (char) 111, (char) 22, (char) 3, (char) 15, (char) 219, (char) 134, (char) 180, (char) 86, (char) 98, (char) 39, (char) 113, (char) 129, (char) 144, (char) 162, (char) 194, (char) 98, (char) 157, (char) 84, (char) 162, (char) 205, (char) 149, (char) 228, (char) 218, (char) 172, (char) 89, (char) 103, (char) 140, (char) 146, (char) 70, (char) 158, (char) 61, (char) 187, (char) 190, (char) 255, (char) 145, (char) 190, (char) 211, (char) 244, (char) 147, (char) 92, (char) 238, (char) 246, (char) 155, (char) 141, (char) 165, (char) 162, (char) 174, (char) 225, (char) 153, (char) 35, (char) 169, (char) 222, (char) 251, (char) 142, (char) 175, (char) 172, (char) 107, (char) 227, (char) 112, (char) 145, (char) 9, (char) 2, (char) 146, (char) 133, (char) 106, (char) 121, (char) 243, (char) 22, (char) 255, (char) 235, (char) 101, (char) 239, (char) 34, (char) 82, (char) 127, (char) 212}, 0);
		p110.target_component_SET((char) 87);
		CommunicationChannel.instance.send(p110);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_TIMESYNC.add((src, ph, pack) ->
		{
			assert (pack.tc1_GET() == -9212352615667779709L);
			assert (pack.ts1_GET() == 4531130453899678323L);
		});
		GroundControl.TIMESYNC p111 = CommunicationChannel.new_TIMESYNC();
		PH.setPack(p111);
		p111.ts1_SET(4531130453899678323L);
		p111.tc1_SET(-9212352615667779709L);
		CommunicationChannel.instance.send(p111);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_CAMERA_TRIGGER.add((src, ph, pack) ->
		{
			assert (pack.time_usec_GET() == 5095193560739873846L);
			assert (pack.seq_GET() == 3441819567L);
		});
		GroundControl.CAMERA_TRIGGER p112 = CommunicationChannel.new_CAMERA_TRIGGER();
		PH.setPack(p112);
		p112.time_usec_SET(5095193560739873846L);
		p112.seq_SET(3441819567L);
		CommunicationChannel.instance.send(p112);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_HIL_GPS.add((src, ph, pack) ->
		{
			assert (pack.vn_GET() == (short) 28671);
			assert (pack.time_usec_GET() == 5745236936779727185L);
			assert (pack.fix_type_GET() == (char) 60);
			assert (pack.vd_GET() == (short) 27937);
			assert (pack.lon_GET() == -997753873);
			assert (pack.alt_GET() == 878902375);
			assert (pack.epv_GET() == (char) 4596);
			assert (pack.ve_GET() == (short) 20674);
			assert (pack.cog_GET() == (char) 30543);
			assert (pack.eph_GET() == (char) 11385);
			assert (pack.lat_GET() == 1045089364);
			assert (pack.vel_GET() == (char) 24683);
			assert (pack.satellites_visible_GET() == (char) 217);
		});
		GroundControl.HIL_GPS p113 = CommunicationChannel.new_HIL_GPS();
		PH.setPack(p113);
		p113.vd_SET((short) 27937);
		p113.lon_SET(-997753873);
		p113.time_usec_SET(5745236936779727185L);
		p113.fix_type_SET((char) 60);
		p113.eph_SET((char) 11385);
		p113.vel_SET((char) 24683);
		p113.vn_SET((short) 28671);
		p113.ve_SET((short) 20674);
		p113.satellites_visible_SET((char) 217);
		p113.alt_SET(878902375);
		p113.cog_SET((char) 30543);
		p113.epv_SET((char) 4596);
		p113.lat_SET(1045089364);
		CommunicationChannel.instance.send(p113);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_HIL_OPTICAL_FLOW.add((src, ph, pack) ->
		{
			assert (pack.integrated_ygyro_GET() == -1.0305214E38F);
			assert (pack.time_delta_distance_us_GET() == 2159404916L);
			assert (pack.temperature_GET() == (short) -27785);
			assert (pack.time_usec_GET() == 2322122361631898243L);
			assert (pack.integrated_xgyro_GET() == 9.135931E36F);
			assert (pack.distance_GET() == 6.300633E37F);
			assert (pack.integrated_zgyro_GET() == 1.3516993E38F);
			assert (pack.integrated_x_GET() == 1.3712727E38F);
			assert (pack.quality_GET() == (char) 229);
			assert (pack.integrated_y_GET() == 2.6842643E38F);
			assert (pack.sensor_id_GET() == (char) 42);
			assert (pack.integration_time_us_GET() == 4260495282L);
		});
		GroundControl.HIL_OPTICAL_FLOW p114 = CommunicationChannel.new_HIL_OPTICAL_FLOW();
		PH.setPack(p114);
		p114.integrated_zgyro_SET(1.3516993E38F);
		p114.integrated_ygyro_SET(-1.0305214E38F);
		p114.integrated_y_SET(2.6842643E38F);
		p114.quality_SET((char) 229);
		p114.integrated_x_SET(1.3712727E38F);
		p114.sensor_id_SET((char) 42);
		p114.temperature_SET((short) -27785);
		p114.distance_SET(6.300633E37F);
		p114.time_usec_SET(2322122361631898243L);
		p114.integration_time_us_SET(4260495282L);
		p114.integrated_xgyro_SET(9.135931E36F);
		p114.time_delta_distance_us_SET(2159404916L);
		CommunicationChannel.instance.send(p114);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_HIL_STATE_QUATERNION.add((src, ph, pack) ->
		{
			assert (pack.true_airspeed_GET() == (char) 30557);
			assert (pack.pitchspeed_GET() == 2.319781E38F);
			assert (pack.alt_GET() == 1227278021);
			assert (pack.zacc_GET() == (short) -7070);
			assert (pack.yawspeed_GET() == -3.0874862E38F);
			assert (pack.vx_GET() == (short) -28087);
			assert (pack.vz_GET() == (short) -16847);
			assert (pack.lon_GET() == 1658004926);
			assert (pack.vy_GET() == (short) 2366);
			assert (pack.rollspeed_GET() == -4.8564785E36F);
			assert (pack.lat_GET() == -214820003);
			assert (pack.ind_airspeed_GET() == (char) 6289);
			assert (pack.yacc_GET() == (short) -27055);
			assert (pack.time_usec_GET() == 8074621391923462211L);
			assert (pack.xacc_GET() == (short) -13851);
			assert (Arrays.equals(pack.attitude_quaternion_GET(), new float[]{4.2369906E36F, 3.896512E37F, 1.2832715E38F, 1.752331E37F}));
		});
		GroundControl.HIL_STATE_QUATERNION p115 = CommunicationChannel.new_HIL_STATE_QUATERNION();
		PH.setPack(p115);
		p115.xacc_SET((short) -13851);
		p115.time_usec_SET(8074621391923462211L);
		p115.zacc_SET((short) -7070);
		p115.vz_SET((short) -16847);
		p115.alt_SET(1227278021);
		p115.lat_SET(-214820003);
		p115.pitchspeed_SET(2.319781E38F);
		p115.lon_SET(1658004926);
		p115.vx_SET((short) -28087);
		p115.attitude_quaternion_SET(new float[]{4.2369906E36F, 3.896512E37F, 1.2832715E38F, 1.752331E37F}, 0);
		p115.true_airspeed_SET((char) 30557);
		p115.yacc_SET((short) -27055);
		p115.yawspeed_SET(-3.0874862E38F);
		p115.vy_SET((short) 2366);
		p115.ind_airspeed_SET((char) 6289);
		p115.rollspeed_SET(-4.8564785E36F);
		CommunicationChannel.instance.send(p115);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_SCALED_IMU2.add((src, ph, pack) ->
		{
			assert (pack.xgyro_GET() == (short) 1438);
			assert (pack.time_boot_ms_GET() == 1775375701L);
			assert (pack.ygyro_GET() == (short) -11855);
			assert (pack.ymag_GET() == (short) -20186);
			assert (pack.zacc_GET() == (short) -4171);
			assert (pack.xacc_GET() == (short) -28199);
			assert (pack.xmag_GET() == (short) 17939);
			assert (pack.yacc_GET() == (short) 27600);
			assert (pack.zgyro_GET() == (short) -27174);
			assert (pack.zmag_GET() == (short) 2567);
		});
		GroundControl.SCALED_IMU2 p116 = CommunicationChannel.new_SCALED_IMU2();
		PH.setPack(p116);
		p116.yacc_SET((short) 27600);
		p116.xmag_SET((short) 17939);
		p116.xgyro_SET((short) 1438);
		p116.time_boot_ms_SET(1775375701L);
		p116.ygyro_SET((short) -11855);
		p116.xacc_SET((short) -28199);
		p116.ymag_SET((short) -20186);
		p116.zacc_SET((short) -4171);
		p116.zmag_SET((short) 2567);
		p116.zgyro_SET((short) -27174);
		CommunicationChannel.instance.send(p116);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_LOG_REQUEST_LIST.add((src, ph, pack) ->
		{
			assert (pack.target_system_GET() == (char) 75);
			assert (pack.start_GET() == (char) 13562);
			assert (pack.end_GET() == (char) 52763);
			assert (pack.target_component_GET() == (char) 75);
		});
		GroundControl.LOG_REQUEST_LIST p117 = CommunicationChannel.new_LOG_REQUEST_LIST();
		PH.setPack(p117);
		p117.end_SET((char) 52763);
		p117.start_SET((char) 13562);
		p117.target_system_SET((char) 75);
		p117.target_component_SET((char) 75);
		CommunicationChannel.instance.send(p117);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_LOG_ENTRY.add((src, ph, pack) ->
		{
			assert (pack.time_utc_GET() == 688055873L);
			assert (pack.last_log_num_GET() == (char) 63715);
			assert (pack.num_logs_GET() == (char) 60169);
			assert (pack.id_GET() == (char) 14097);
			assert (pack.size_GET() == 1745350140L);
		});
		GroundControl.LOG_ENTRY p118 = CommunicationChannel.new_LOG_ENTRY();
		PH.setPack(p118);
		p118.time_utc_SET(688055873L);
		p118.num_logs_SET((char) 60169);
		p118.size_SET(1745350140L);
		p118.last_log_num_SET((char) 63715);
		p118.id_SET((char) 14097);
		CommunicationChannel.instance.send(p118);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_LOG_REQUEST_DATA.add((src, ph, pack) ->
		{
			assert (pack.ofs_GET() == 1292803663L);
			assert (pack.id_GET() == (char) 57027);
			assert (pack.target_system_GET() == (char) 223);
			assert (pack.target_component_GET() == (char) 159);
			assert (pack.count_GET() == 4121512847L);
		});
		GroundControl.LOG_REQUEST_DATA p119 = CommunicationChannel.new_LOG_REQUEST_DATA();
		PH.setPack(p119);
		p119.id_SET((char) 57027);
		p119.target_system_SET((char) 223);
		p119.ofs_SET(1292803663L);
		p119.count_SET(4121512847L);
		p119.target_component_SET((char) 159);
		CommunicationChannel.instance.send(p119);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_LOG_DATA.add((src, ph, pack) ->
		{
			assert (pack.id_GET() == (char) 52961);
			assert (Arrays.equals(pack.data__GET(), new char[]{(char) 174, (char) 63, (char) 60, (char) 199, (char) 240, (char) 16, (char) 218, (char) 119, (char) 101, (char) 136, (char) 59, (char) 214, (char) 224, (char) 60, (char) 64, (char) 35, (char) 119, (char) 121, (char) 142, (char) 62, (char) 205, (char) 185, (char) 243, (char) 208, (char) 35, (char) 37, (char) 233, (char) 85, (char) 96, (char) 73, (char) 17, (char) 66, (char) 15, (char) 114, (char) 29, (char) 182, (char) 234, (char) 236, (char) 10, (char) 223, (char) 29, (char) 231, (char) 111, (char) 211, (char) 62, (char) 162, (char) 199, (char) 57, (char) 129, (char) 250, (char) 19, (char) 32, (char) 243, (char) 245, (char) 31, (char) 163, (char) 145, (char) 96, (char) 87, (char) 245, (char) 115, (char) 211, (char) 165, (char) 179, (char) 154, (char) 222, (char) 125, (char) 126, (char) 98, (char) 125, (char) 35, (char) 75, (char) 133, (char) 158, (char) 112, (char) 58, (char) 243, (char) 149, (char) 47, (char) 158, (char) 110, (char) 122, (char) 7, (char) 89, (char) 6, (char) 59, (char) 156, (char) 26, (char) 207, (char) 139}));
			assert (pack.ofs_GET() == 1278467365L);
			assert (pack.count_GET() == (char) 102);
		});
		GroundControl.LOG_DATA p120 = CommunicationChannel.new_LOG_DATA();
		PH.setPack(p120);
		p120.id_SET((char) 52961);
		p120.data__SET(new char[]{(char) 174, (char) 63, (char) 60, (char) 199, (char) 240, (char) 16, (char) 218, (char) 119, (char) 101, (char) 136, (char) 59, (char) 214, (char) 224, (char) 60, (char) 64, (char) 35, (char) 119, (char) 121, (char) 142, (char) 62, (char) 205, (char) 185, (char) 243, (char) 208, (char) 35, (char) 37, (char) 233, (char) 85, (char) 96, (char) 73, (char) 17, (char) 66, (char) 15, (char) 114, (char) 29, (char) 182, (char) 234, (char) 236, (char) 10, (char) 223, (char) 29, (char) 231, (char) 111, (char) 211, (char) 62, (char) 162, (char) 199, (char) 57, (char) 129, (char) 250, (char) 19, (char) 32, (char) 243, (char) 245, (char) 31, (char) 163, (char) 145, (char) 96, (char) 87, (char) 245, (char) 115, (char) 211, (char) 165, (char) 179, (char) 154, (char) 222, (char) 125, (char) 126, (char) 98, (char) 125, (char) 35, (char) 75, (char) 133, (char) 158, (char) 112, (char) 58, (char) 243, (char) 149, (char) 47, (char) 158, (char) 110, (char) 122, (char) 7, (char) 89, (char) 6, (char) 59, (char) 156, (char) 26, (char) 207, (char) 139}, 0);
		p120.count_SET((char) 102);
		p120.ofs_SET(1278467365L);
		CommunicationChannel.instance.send(p120);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_LOG_ERASE.add((src, ph, pack) ->
		{
			assert (pack.target_component_GET() == (char) 164);
			assert (pack.target_system_GET() == (char) 86);
		});
		GroundControl.LOG_ERASE p121 = CommunicationChannel.new_LOG_ERASE();
		PH.setPack(p121);
		p121.target_system_SET((char) 86);
		p121.target_component_SET((char) 164);
		CommunicationChannel.instance.send(p121);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_LOG_REQUEST_END.add((src, ph, pack) ->
		{
			assert (pack.target_component_GET() == (char) 43);
			assert (pack.target_system_GET() == (char) 48);
		});
		GroundControl.LOG_REQUEST_END p122 = CommunicationChannel.new_LOG_REQUEST_END();
		PH.setPack(p122);
		p122.target_component_SET((char) 43);
		p122.target_system_SET((char) 48);
		CommunicationChannel.instance.send(p122);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_GPS_INJECT_DATA.add((src, ph, pack) ->
		{
			assert (pack.target_system_GET() == (char) 59);
			assert (pack.len_GET() == (char) 162);
			assert (pack.target_component_GET() == (char) 244);
			assert (Arrays.equals(pack.data__GET(), new char[]{(char) 219, (char) 29, (char) 104, (char) 14, (char) 203, (char) 158, (char) 65, (char) 179, (char) 55, (char) 205, (char) 121, (char) 245, (char) 30, (char) 30, (char) 24, (char) 217, (char) 226, (char) 47, (char) 204, (char) 95, (char) 185, (char) 222, (char) 87, (char) 10, (char) 227, (char) 195, (char) 166, (char) 159, (char) 41, (char) 242, (char) 177, (char) 221, (char) 175, (char) 228, (char) 222, (char) 54, (char) 19, (char) 37, (char) 144, (char) 135, (char) 8, (char) 86, (char) 169, (char) 135, (char) 9, (char) 96, (char) 239, (char) 2, (char) 49, (char) 91, (char) 128, (char) 201, (char) 39, (char) 75, (char) 249, (char) 173, (char) 87, (char) 61, (char) 173, (char) 241, (char) 192, (char) 73, (char) 212, (char) 74, (char) 251, (char) 118, (char) 67, (char) 90, (char) 188, (char) 243, (char) 190, (char) 79, (char) 62, (char) 58, (char) 163, (char) 32, (char) 40, (char) 103, (char) 130, (char) 32, (char) 124, (char) 100, (char) 224, (char) 57, (char) 152, (char) 13, (char) 210, (char) 253, (char) 121, (char) 195, (char) 147, (char) 116, (char) 126, (char) 90, (char) 53, (char) 252, (char) 185, (char) 71, (char) 210, (char) 109, (char) 89, (char) 231, (char) 180, (char) 81, (char) 216, (char) 172, (char) 191, (char) 129, (char) 239, (char) 145}));
		});
		GroundControl.GPS_INJECT_DATA p123 = CommunicationChannel.new_GPS_INJECT_DATA();
		PH.setPack(p123);
		p123.len_SET((char) 162);
		p123.target_component_SET((char) 244);
		p123.target_system_SET((char) 59);
		p123.data__SET(new char[]{(char) 219, (char) 29, (char) 104, (char) 14, (char) 203, (char) 158, (char) 65, (char) 179, (char) 55, (char) 205, (char) 121, (char) 245, (char) 30, (char) 30, (char) 24, (char) 217, (char) 226, (char) 47, (char) 204, (char) 95, (char) 185, (char) 222, (char) 87, (char) 10, (char) 227, (char) 195, (char) 166, (char) 159, (char) 41, (char) 242, (char) 177, (char) 221, (char) 175, (char) 228, (char) 222, (char) 54, (char) 19, (char) 37, (char) 144, (char) 135, (char) 8, (char) 86, (char) 169, (char) 135, (char) 9, (char) 96, (char) 239, (char) 2, (char) 49, (char) 91, (char) 128, (char) 201, (char) 39, (char) 75, (char) 249, (char) 173, (char) 87, (char) 61, (char) 173, (char) 241, (char) 192, (char) 73, (char) 212, (char) 74, (char) 251, (char) 118, (char) 67, (char) 90, (char) 188, (char) 243, (char) 190, (char) 79, (char) 62, (char) 58, (char) 163, (char) 32, (char) 40, (char) 103, (char) 130, (char) 32, (char) 124, (char) 100, (char) 224, (char) 57, (char) 152, (char) 13, (char) 210, (char) 253, (char) 121, (char) 195, (char) 147, (char) 116, (char) 126, (char) 90, (char) 53, (char) 252, (char) 185, (char) 71, (char) 210, (char) 109, (char) 89, (char) 231, (char) 180, (char) 81, (char) 216, (char) 172, (char) 191, (char) 129, (char) 239, (char) 145}, 0);
		CommunicationChannel.instance.send(p123);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_GPS2_RAW.add((src, ph, pack) ->
		{
			assert (pack.cog_GET() == (char) 3588);
			assert (pack.lat_GET() == 397650864);
			assert (pack.fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_RTK_FIXED);
			assert (pack.time_usec_GET() == 871665601009675432L);
			assert (pack.dgps_age_GET() == 2076905214L);
			assert (pack.lon_GET() == 799793825);
			assert (pack.alt_GET() == 189681660);
			assert (pack.satellites_visible_GET() == (char) 167);
			assert (pack.dgps_numch_GET() == (char) 114);
			assert (pack.eph_GET() == (char) 57632);
			assert (pack.epv_GET() == (char) 22084);
			assert (pack.vel_GET() == (char) 64481);
		});
		GroundControl.GPS2_RAW p124 = CommunicationChannel.new_GPS2_RAW();
		PH.setPack(p124);
		p124.epv_SET((char) 22084);
		p124.satellites_visible_SET((char) 167);
		p124.lat_SET(397650864);
		p124.dgps_age_SET(2076905214L);
		p124.cog_SET((char) 3588);
		p124.lon_SET(799793825);
		p124.eph_SET((char) 57632);
		p124.alt_SET(189681660);
		p124.dgps_numch_SET((char) 114);
		p124.time_usec_SET(871665601009675432L);
		p124.vel_SET((char) 64481);
		p124.fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_RTK_FIXED);
		CommunicationChannel.instance.send(p124);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_POWER_STATUS.add((src, ph, pack) ->
		{
			assert (pack.flags_GET() == MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_OVERCURRENT);
			assert (pack.Vcc_GET() == (char) 6179);
			assert (pack.Vservo_GET() == (char) 28764);
		});
		GroundControl.POWER_STATUS p125 = CommunicationChannel.new_POWER_STATUS();
		PH.setPack(p125);
		p125.Vcc_SET((char) 6179);
		p125.flags_SET(MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_OVERCURRENT);
		p125.Vservo_SET((char) 28764);
		CommunicationChannel.instance.send(p125);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_SERIAL_CONTROL.add((src, ph, pack) ->
		{
			assert (pack.count_GET() == (char) 35);
			assert (pack.timeout_GET() == (char) 2898);
			assert (pack.device_GET() == SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_GPS2);
			assert (pack.flags_GET() == SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_MULTI);
			assert (pack.baudrate_GET() == 361681615L);
			assert (Arrays.equals(pack.data__GET(), new char[]{(char) 242, (char) 41, (char) 24, (char) 129, (char) 221, (char) 196, (char) 199, (char) 0, (char) 105, (char) 202, (char) 222, (char) 116, (char) 0, (char) 129, (char) 43, (char) 83, (char) 1, (char) 13, (char) 105, (char) 205, (char) 90, (char) 226, (char) 169, (char) 70, (char) 90, (char) 25, (char) 21, (char) 184, (char) 123, (char) 146, (char) 183, (char) 203, (char) 107, (char) 199, (char) 68, (char) 21, (char) 196, (char) 50, (char) 187, (char) 85, (char) 153, (char) 243, (char) 123, (char) 206, (char) 216, (char) 26, (char) 30, (char) 226, (char) 154, (char) 144, (char) 195, (char) 224, (char) 201, (char) 205, (char) 48, (char) 102, (char) 228, (char) 198, (char) 4, (char) 232, (char) 128, (char) 201, (char) 96, (char) 18, (char) 51, (char) 99, (char) 115, (char) 89, (char) 7, (char) 173}));
		});
		GroundControl.SERIAL_CONTROL p126 = CommunicationChannel.new_SERIAL_CONTROL();
		PH.setPack(p126);
		p126.data__SET(new char[]{(char) 242, (char) 41, (char) 24, (char) 129, (char) 221, (char) 196, (char) 199, (char) 0, (char) 105, (char) 202, (char) 222, (char) 116, (char) 0, (char) 129, (char) 43, (char) 83, (char) 1, (char) 13, (char) 105, (char) 205, (char) 90, (char) 226, (char) 169, (char) 70, (char) 90, (char) 25, (char) 21, (char) 184, (char) 123, (char) 146, (char) 183, (char) 203, (char) 107, (char) 199, (char) 68, (char) 21, (char) 196, (char) 50, (char) 187, (char) 85, (char) 153, (char) 243, (char) 123, (char) 206, (char) 216, (char) 26, (char) 30, (char) 226, (char) 154, (char) 144, (char) 195, (char) 224, (char) 201, (char) 205, (char) 48, (char) 102, (char) 228, (char) 198, (char) 4, (char) 232, (char) 128, (char) 201, (char) 96, (char) 18, (char) 51, (char) 99, (char) 115, (char) 89, (char) 7, (char) 173}, 0);
		p126.timeout_SET((char) 2898);
		p126.count_SET((char) 35);
		p126.flags_SET(SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_MULTI);
		p126.device_SET(SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_GPS2);
		p126.baudrate_SET(361681615L);
		CommunicationChannel.instance.send(p126);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_GPS_RTK.add((src, ph, pack) ->
		{
			assert (pack.rtk_health_GET() == (char) 115);
			assert (pack.baseline_c_mm_GET() == -420019269);
			assert (pack.accuracy_GET() == 642689663L);
			assert (pack.tow_GET() == 2292040010L);
			assert (pack.iar_num_hypotheses_GET() == -1731110586);
			assert (pack.rtk_rate_GET() == (char) 175);
			assert (pack.rtk_receiver_id_GET() == (char) 16);
			assert (pack.time_last_baseline_ms_GET() == 2035417544L);
			assert (pack.baseline_coords_type_GET() == (char) 206);
			assert (pack.baseline_b_mm_GET() == -1781197357);
			assert (pack.nsats_GET() == (char) 6);
			assert (pack.wn_GET() == (char) 24408);
			assert (pack.baseline_a_mm_GET() == 1031212602);
		});
		GroundControl.GPS_RTK p127 = CommunicationChannel.new_GPS_RTK();
		PH.setPack(p127);
		p127.baseline_coords_type_SET((char) 206);
		p127.baseline_b_mm_SET(-1781197357);
		p127.rtk_health_SET((char) 115);
		p127.baseline_a_mm_SET(1031212602);
		p127.baseline_c_mm_SET(-420019269);
		p127.wn_SET((char) 24408);
		p127.rtk_rate_SET((char) 175);
		p127.time_last_baseline_ms_SET(2035417544L);
		p127.accuracy_SET(642689663L);
		p127.tow_SET(2292040010L);
		p127.rtk_receiver_id_SET((char) 16);
		p127.nsats_SET((char) 6);
		p127.iar_num_hypotheses_SET(-1731110586);
		CommunicationChannel.instance.send(p127);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_GPS2_RTK.add((src, ph, pack) ->
		{
			assert (pack.wn_GET() == (char) 59690);
			assert (pack.time_last_baseline_ms_GET() == 2383221079L);
			assert (pack.baseline_coords_type_GET() == (char) 144);
			assert (pack.rtk_rate_GET() == (char) 4);
			assert (pack.accuracy_GET() == 766547314L);
			assert (pack.rtk_health_GET() == (char) 227);
			assert (pack.baseline_b_mm_GET() == 157666280);
			assert (pack.iar_num_hypotheses_GET() == 953339902);
			assert (pack.baseline_a_mm_GET() == -706852035);
			assert (pack.baseline_c_mm_GET() == -353515058);
			assert (pack.nsats_GET() == (char) 243);
			assert (pack.tow_GET() == 1990537132L);
			assert (pack.rtk_receiver_id_GET() == (char) 36);
		});
		GroundControl.GPS2_RTK p128 = CommunicationChannel.new_GPS2_RTK();
		PH.setPack(p128);
		p128.rtk_rate_SET((char) 4);
		p128.wn_SET((char) 59690);
		p128.baseline_coords_type_SET((char) 144);
		p128.time_last_baseline_ms_SET(2383221079L);
		p128.baseline_b_mm_SET(157666280);
		p128.nsats_SET((char) 243);
		p128.baseline_c_mm_SET(-353515058);
		p128.tow_SET(1990537132L);
		p128.rtk_health_SET((char) 227);
		p128.rtk_receiver_id_SET((char) 36);
		p128.accuracy_SET(766547314L);
		p128.iar_num_hypotheses_SET(953339902);
		p128.baseline_a_mm_SET(-706852035);
		CommunicationChannel.instance.send(p128);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_SCALED_IMU3.add((src, ph, pack) ->
		{
			assert (pack.ygyro_GET() == (short) -6673);
			assert (pack.ymag_GET() == (short) -31416);
			assert (pack.xacc_GET() == (short) 15758);
			assert (pack.time_boot_ms_GET() == 3167391758L);
			assert (pack.xgyro_GET() == (short) -1163);
			assert (pack.xmag_GET() == (short) -28915);
			assert (pack.zgyro_GET() == (short) 11867);
			assert (pack.zmag_GET() == (short) 1306);
			assert (pack.yacc_GET() == (short) -5743);
			assert (pack.zacc_GET() == (short) -16227);
		});
		GroundControl.SCALED_IMU3 p129 = CommunicationChannel.new_SCALED_IMU3();
		PH.setPack(p129);
		p129.zacc_SET((short) -16227);
		p129.ygyro_SET((short) -6673);
		p129.zgyro_SET((short) 11867);
		p129.time_boot_ms_SET(3167391758L);
		p129.xmag_SET((short) -28915);
		p129.yacc_SET((short) -5743);
		p129.zmag_SET((short) 1306);
		p129.xacc_SET((short) 15758);
		p129.ymag_SET((short) -31416);
		p129.xgyro_SET((short) -1163);
		CommunicationChannel.instance.send(p129);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_DATA_TRANSMISSION_HANDSHAKE.add((src, ph, pack) ->
		{
			assert (pack.packets_GET() == (char) 37367);
			assert (pack.width_GET() == (char) 20845);
			assert (pack.size_GET() == 821185951L);
			assert (pack.type_GET() == (char) 195);
			assert (pack.payload_GET() == (char) 8);
			assert (pack.jpg_quality_GET() == (char) 197);
			assert (pack.height_GET() == (char) 873);
		});
		GroundControl.DATA_TRANSMISSION_HANDSHAKE p130 = CommunicationChannel.new_DATA_TRANSMISSION_HANDSHAKE();
		PH.setPack(p130);
		p130.width_SET((char) 20845);
		p130.height_SET((char) 873);
		p130.packets_SET((char) 37367);
		p130.jpg_quality_SET((char) 197);
		p130.type_SET((char) 195);
		p130.size_SET(821185951L);
		p130.payload_SET((char) 8);
		CommunicationChannel.instance.send(p130);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_ENCAPSULATED_DATA.add((src, ph, pack) ->
		{
			assert (pack.seqnr_GET() == (char) 32825);
			assert (Arrays.equals(pack.data__GET(), new char[]{(char) 0, (char) 179, (char) 101, (char) 83, (char) 220, (char) 157, (char) 237, (char) 215, (char) 204, (char) 15, (char) 49, (char) 87, (char) 252, (char) 211, (char) 8, (char) 242, (char) 71, (char) 69, (char) 216, (char) 0, (char) 33, (char) 236, (char) 83, (char) 234, (char) 103, (char) 9, (char) 158, (char) 244, (char) 95, (char) 162, (char) 31, (char) 11, (char) 152, (char) 42, (char) 199, (char) 194, (char) 155, (char) 111, (char) 33, (char) 42, (char) 34, (char) 246, (char) 24, (char) 89, (char) 214, (char) 172, (char) 3, (char) 203, (char) 7, (char) 156, (char) 155, (char) 144, (char) 252, (char) 30, (char) 226, (char) 142, (char) 160, (char) 122, (char) 5, (char) 63, (char) 130, (char) 185, (char) 5, (char) 59, (char) 224, (char) 202, (char) 195, (char) 200, (char) 131, (char) 125, (char) 67, (char) 94, (char) 16, (char) 58, (char) 0, (char) 226, (char) 2, (char) 153, (char) 59, (char) 166, (char) 92, (char) 0, (char) 113, (char) 214, (char) 36, (char) 103, (char) 87, (char) 157, (char) 52, (char) 15, (char) 25, (char) 5, (char) 252, (char) 214, (char) 184, (char) 181, (char) 182, (char) 115, (char) 19, (char) 244, (char) 132, (char) 230, (char) 56, (char) 90, (char) 93, (char) 184, (char) 94, (char) 16, (char) 145, (char) 93, (char) 53, (char) 167, (char) 134, (char) 239, (char) 101, (char) 71, (char) 203, (char) 61, (char) 114, (char) 151, (char) 197, (char) 33, (char) 38, (char) 77, (char) 154, (char) 135, (char) 172, (char) 156, (char) 122, (char) 130, (char) 22, (char) 30, (char) 218, (char) 83, (char) 38, (char) 15, (char) 215, (char) 91, (char) 151, (char) 195, (char) 178, (char) 34, (char) 4, (char) 45, (char) 99, (char) 104, (char) 109, (char) 165, (char) 155, (char) 188, (char) 211, (char) 153, (char) 183, (char) 195, (char) 204, (char) 126, (char) 5, (char) 127, (char) 248, (char) 63, (char) 123, (char) 127, (char) 159, (char) 31, (char) 168, (char) 53, (char) 91, (char) 85, (char) 21, (char) 113, (char) 112, (char) 236, (char) 232, (char) 230, (char) 57, (char) 36, (char) 161, (char) 123, (char) 208, (char) 96, (char) 250, (char) 254, (char) 42, (char) 164, (char) 192, (char) 61, (char) 86, (char) 162, (char) 28, (char) 107, (char) 78, (char) 246, (char) 174, (char) 246, (char) 56, (char) 114, (char) 105, (char) 148, (char) 175, (char) 188, (char) 138, (char) 145, (char) 222, (char) 173, (char) 33, (char) 85, (char) 132, (char) 156, (char) 232, (char) 51, (char) 186, (char) 116, (char) 220, (char) 158, (char) 213, (char) 201, (char) 89, (char) 219, (char) 186, (char) 241, (char) 9, (char) 57, (char) 177, (char) 64, (char) 61, (char) 19, (char) 79, (char) 51, (char) 46, (char) 108, (char) 98, (char) 18, (char) 112, (char) 181, (char) 248, (char) 234, (char) 170, (char) 94, (char) 43, (char) 20, (char) 204, (char) 5, (char) 53, (char) 47, (char) 64, (char) 157, (char) 222, (char) 11, (char) 53, (char) 234, (char) 190, (char) 239, (char) 74}));
		});
		GroundControl.ENCAPSULATED_DATA p131 = CommunicationChannel.new_ENCAPSULATED_DATA();
		PH.setPack(p131);
		p131.data__SET(new char[]{(char) 0, (char) 179, (char) 101, (char) 83, (char) 220, (char) 157, (char) 237, (char) 215, (char) 204, (char) 15, (char) 49, (char) 87, (char) 252, (char) 211, (char) 8, (char) 242, (char) 71, (char) 69, (char) 216, (char) 0, (char) 33, (char) 236, (char) 83, (char) 234, (char) 103, (char) 9, (char) 158, (char) 244, (char) 95, (char) 162, (char) 31, (char) 11, (char) 152, (char) 42, (char) 199, (char) 194, (char) 155, (char) 111, (char) 33, (char) 42, (char) 34, (char) 246, (char) 24, (char) 89, (char) 214, (char) 172, (char) 3, (char) 203, (char) 7, (char) 156, (char) 155, (char) 144, (char) 252, (char) 30, (char) 226, (char) 142, (char) 160, (char) 122, (char) 5, (char) 63, (char) 130, (char) 185, (char) 5, (char) 59, (char) 224, (char) 202, (char) 195, (char) 200, (char) 131, (char) 125, (char) 67, (char) 94, (char) 16, (char) 58, (char) 0, (char) 226, (char) 2, (char) 153, (char) 59, (char) 166, (char) 92, (char) 0, (char) 113, (char) 214, (char) 36, (char) 103, (char) 87, (char) 157, (char) 52, (char) 15, (char) 25, (char) 5, (char) 252, (char) 214, (char) 184, (char) 181, (char) 182, (char) 115, (char) 19, (char) 244, (char) 132, (char) 230, (char) 56, (char) 90, (char) 93, (char) 184, (char) 94, (char) 16, (char) 145, (char) 93, (char) 53, (char) 167, (char) 134, (char) 239, (char) 101, (char) 71, (char) 203, (char) 61, (char) 114, (char) 151, (char) 197, (char) 33, (char) 38, (char) 77, (char) 154, (char) 135, (char) 172, (char) 156, (char) 122, (char) 130, (char) 22, (char) 30, (char) 218, (char) 83, (char) 38, (char) 15, (char) 215, (char) 91, (char) 151, (char) 195, (char) 178, (char) 34, (char) 4, (char) 45, (char) 99, (char) 104, (char) 109, (char) 165, (char) 155, (char) 188, (char) 211, (char) 153, (char) 183, (char) 195, (char) 204, (char) 126, (char) 5, (char) 127, (char) 248, (char) 63, (char) 123, (char) 127, (char) 159, (char) 31, (char) 168, (char) 53, (char) 91, (char) 85, (char) 21, (char) 113, (char) 112, (char) 236, (char) 232, (char) 230, (char) 57, (char) 36, (char) 161, (char) 123, (char) 208, (char) 96, (char) 250, (char) 254, (char) 42, (char) 164, (char) 192, (char) 61, (char) 86, (char) 162, (char) 28, (char) 107, (char) 78, (char) 246, (char) 174, (char) 246, (char) 56, (char) 114, (char) 105, (char) 148, (char) 175, (char) 188, (char) 138, (char) 145, (char) 222, (char) 173, (char) 33, (char) 85, (char) 132, (char) 156, (char) 232, (char) 51, (char) 186, (char) 116, (char) 220, (char) 158, (char) 213, (char) 201, (char) 89, (char) 219, (char) 186, (char) 241, (char) 9, (char) 57, (char) 177, (char) 64, (char) 61, (char) 19, (char) 79, (char) 51, (char) 46, (char) 108, (char) 98, (char) 18, (char) 112, (char) 181, (char) 248, (char) 234, (char) 170, (char) 94, (char) 43, (char) 20, (char) 204, (char) 5, (char) 53, (char) 47, (char) 64, (char) 157, (char) 222, (char) 11, (char) 53, (char) 234, (char) 190, (char) 239, (char) 74}, 0);
		p131.seqnr_SET((char) 32825);
		CommunicationChannel.instance.send(p131);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_DISTANCE_SENSOR.add((src, ph, pack) ->
		{
			assert (pack.min_distance_GET() == (char) 61566);
			assert (pack.type_GET() == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_LASER);
			assert (pack.current_distance_GET() == (char) 57267);
			assert (pack.time_boot_ms_GET() == 4278500350L);
			assert (pack.orientation_GET() == MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_90_PITCH_90);
			assert (pack.covariance_GET() == (char) 198);
			assert (pack.id_GET() == (char) 134);
			assert (pack.max_distance_GET() == (char) 48819);
		});
		GroundControl.DISTANCE_SENSOR p132 = CommunicationChannel.new_DISTANCE_SENSOR();
		PH.setPack(p132);
		p132.time_boot_ms_SET(4278500350L);
		p132.min_distance_SET((char) 61566);
		p132.max_distance_SET((char) 48819);
		p132.current_distance_SET((char) 57267);
		p132.orientation_SET(MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_90_PITCH_90);
		p132.id_SET((char) 134);
		p132.covariance_SET((char) 198);
		p132.type_SET(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_LASER);
		CommunicationChannel.instance.send(p132);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_TERRAIN_REQUEST.add((src, ph, pack) ->
		{
			assert (pack.mask_GET() == 8555644136819218330L);
			assert (pack.lon_GET() == -577018173);
			assert (pack.lat_GET() == -175655412);
			assert (pack.grid_spacing_GET() == (char) 24059);
		});
		GroundControl.TERRAIN_REQUEST p133 = CommunicationChannel.new_TERRAIN_REQUEST();
		PH.setPack(p133);
		p133.grid_spacing_SET((char) 24059);
		p133.lat_SET(-175655412);
		p133.mask_SET(8555644136819218330L);
		p133.lon_SET(-577018173);
		CommunicationChannel.instance.send(p133);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_TERRAIN_DATA.add((src, ph, pack) ->
		{
			assert (pack.lat_GET() == -1881113138);
			assert (Arrays.equals(pack.data__GET(), new short[]{(short) -6591, (short) 6999, (short) 9746, (short) -32129, (short) 17242, (short) -22126, (short) -24884, (short) 6431, (short) -6499, (short) 23127, (short) 2416, (short) -16708, (short) -13896, (short) -21941, (short) -843, (short) -3855}));
			assert (pack.lon_GET() == 2146993817);
			assert (pack.gridbit_GET() == (char) 193);
			assert (pack.grid_spacing_GET() == (char) 13581);
		});
		GroundControl.TERRAIN_DATA p134 = CommunicationChannel.new_TERRAIN_DATA();
		PH.setPack(p134);
		p134.gridbit_SET((char) 193);
		p134.lat_SET(-1881113138);
		p134.lon_SET(2146993817);
		p134.data__SET(new short[]{(short) -6591, (short) 6999, (short) 9746, (short) -32129, (short) 17242, (short) -22126, (short) -24884, (short) 6431, (short) -6499, (short) 23127, (short) 2416, (short) -16708, (short) -13896, (short) -21941, (short) -843, (short) -3855}, 0);
		p134.grid_spacing_SET((char) 13581);
		CommunicationChannel.instance.send(p134);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_TERRAIN_CHECK.add((src, ph, pack) ->
		{
			assert (pack.lat_GET() == 1473261191);
			assert (pack.lon_GET() == -1285330204);
		});
		GroundControl.TERRAIN_CHECK p135 = CommunicationChannel.new_TERRAIN_CHECK();
		PH.setPack(p135);
		p135.lat_SET(1473261191);
		p135.lon_SET(-1285330204);
		CommunicationChannel.instance.send(p135);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_TERRAIN_REPORT.add((src, ph, pack) ->
		{
			assert (pack.pending_GET() == (char) 10868);
			assert (pack.lat_GET() == -166378943);
			assert (pack.loaded_GET() == (char) 13398);
			assert (pack.spacing_GET() == (char) 21068);
			assert (pack.terrain_height_GET() == -2.9571569E38F);
			assert (pack.current_height_GET() == -2.5638712E38F);
			assert (pack.lon_GET() == -207934094);
		});
		GroundControl.TERRAIN_REPORT p136 = CommunicationChannel.new_TERRAIN_REPORT();
		PH.setPack(p136);
		p136.terrain_height_SET(-2.9571569E38F);
		p136.lat_SET(-166378943);
		p136.pending_SET((char) 10868);
		p136.lon_SET(-207934094);
		p136.spacing_SET((char) 21068);
		p136.current_height_SET(-2.5638712E38F);
		p136.loaded_SET((char) 13398);
		CommunicationChannel.instance.send(p136);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_SCALED_PRESSURE2.add((src, ph, pack) ->
		{
			assert (pack.time_boot_ms_GET() == 1394608807L);
			assert (pack.temperature_GET() == (short) -26669);
			assert (pack.press_diff_GET() == 8.006883E37F);
			assert (pack.press_abs_GET() == -2.0838137E38F);
		});
		GroundControl.SCALED_PRESSURE2 p137 = CommunicationChannel.new_SCALED_PRESSURE2();
		PH.setPack(p137);
		p137.press_diff_SET(8.006883E37F);
		p137.temperature_SET((short) -26669);
		p137.time_boot_ms_SET(1394608807L);
		p137.press_abs_SET(-2.0838137E38F);
		CommunicationChannel.instance.send(p137);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_ATT_POS_MOCAP.add((src, ph, pack) ->
		{
			assert (pack.y_GET() == 6.75119E37F);
			assert (pack.x_GET() == 3.0345236E37F);
			assert (pack.time_usec_GET() == 614319354806255138L);
			assert (pack.z_GET() == -1.2870858E37F);
			assert (Arrays.equals(pack.q_GET(), new float[]{2.2475027E38F, 2.5903566E38F, 5.1307174E37F, -1.6672284E38F}));
		});
		GroundControl.ATT_POS_MOCAP p138 = CommunicationChannel.new_ATT_POS_MOCAP();
		PH.setPack(p138);
		p138.z_SET(-1.2870858E37F);
		p138.q_SET(new float[]{2.2475027E38F, 2.5903566E38F, 5.1307174E37F, -1.6672284E38F}, 0);
		p138.time_usec_SET(614319354806255138L);
		p138.x_SET(3.0345236E37F);
		p138.y_SET(6.75119E37F);
		CommunicationChannel.instance.send(p138);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_SET_ACTUATOR_CONTROL_TARGET.add((src, ph, pack) ->
		{
			assert (Arrays.equals(pack.controls_GET(), new float[]{3.2497915E38F, -3.8138822E37F, -2.7165888E38F, 1.1636853E38F, -7.981524E36F, 1.4986853E38F, -5.4712444E37F, 1.549257E38F}));
			assert (pack.time_usec_GET() == 7324793743557686811L);
			assert (pack.target_component_GET() == (char) 153);
			assert (pack.group_mlx_GET() == (char) 142);
			assert (pack.target_system_GET() == (char) 250);
		});
		GroundControl.SET_ACTUATOR_CONTROL_TARGET p139 = CommunicationChannel.new_SET_ACTUATOR_CONTROL_TARGET();
		PH.setPack(p139);
		p139.target_component_SET((char) 153);
		p139.group_mlx_SET((char) 142);
		p139.controls_SET(new float[]{3.2497915E38F, -3.8138822E37F, -2.7165888E38F, 1.1636853E38F, -7.981524E36F, 1.4986853E38F, -5.4712444E37F, 1.549257E38F}, 0);
		p139.target_system_SET((char) 250);
		p139.time_usec_SET(7324793743557686811L);
		CommunicationChannel.instance.send(p139);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_ACTUATOR_CONTROL_TARGET.add((src, ph, pack) ->
		{
			assert (pack.group_mlx_GET() == (char) 159);
			assert (pack.time_usec_GET() == 634370196172123888L);
			assert (Arrays.equals(pack.controls_GET(), new float[]{-2.6493046E38F, -3.3829767E38F, -2.1504283E38F, 3.3056894E38F, -2.4868751E38F, -2.78616E37F, 2.7286279E38F, -2.6262494E38F}));
		});
		GroundControl.ACTUATOR_CONTROL_TARGET p140 = CommunicationChannel.new_ACTUATOR_CONTROL_TARGET();
		PH.setPack(p140);
		p140.time_usec_SET(634370196172123888L);
		p140.controls_SET(new float[]{-2.6493046E38F, -3.3829767E38F, -2.1504283E38F, 3.3056894E38F, -2.4868751E38F, -2.78616E37F, 2.7286279E38F, -2.6262494E38F}, 0);
		p140.group_mlx_SET((char) 159);
		CommunicationChannel.instance.send(p140);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_ALTITUDE.add((src, ph, pack) ->
		{
			assert (pack.altitude_monotonic_GET() == 2.7258615E38F);
			assert (pack.altitude_amsl_GET() == 9.619757E37F);
			assert (pack.altitude_local_GET() == -1.1030036E38F);
			assert (pack.bottom_clearance_GET() == 2.4061627E38F);
			assert (pack.time_usec_GET() == 3657838980881137094L);
			assert (pack.altitude_terrain_GET() == 1.8778055E38F);
			assert (pack.altitude_relative_GET() == 2.631455E38F);
		});
		GroundControl.ALTITUDE p141 = CommunicationChannel.new_ALTITUDE();
		PH.setPack(p141);
		p141.altitude_relative_SET(2.631455E38F);
		p141.altitude_terrain_SET(1.8778055E38F);
		p141.bottom_clearance_SET(2.4061627E38F);
		p141.altitude_monotonic_SET(2.7258615E38F);
		p141.altitude_local_SET(-1.1030036E38F);
		p141.time_usec_SET(3657838980881137094L);
		p141.altitude_amsl_SET(9.619757E37F);
		CommunicationChannel.instance.send(p141);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_RESOURCE_REQUEST.add((src, ph, pack) ->
		{
			assert (pack.uri_type_GET() == (char) 165);
			assert (Arrays.equals(pack.storage_GET(), new char[]{(char) 228, (char) 45, (char) 246, (char) 209, (char) 54, (char) 141, (char) 127, (char) 171, (char) 89, (char) 212, (char) 76, (char) 139, (char) 174, (char) 213, (char) 235, (char) 193, (char) 245, (char) 209, (char) 38, (char) 255, (char) 118, (char) 35, (char) 178, (char) 152, (char) 205, (char) 75, (char) 154, (char) 95, (char) 118, (char) 149, (char) 76, (char) 195, (char) 100, (char) 118, (char) 6, (char) 54, (char) 60, (char) 70, (char) 189, (char) 218, (char) 112, (char) 244, (char) 15, (char) 44, (char) 131, (char) 108, (char) 1, (char) 105, (char) 139, (char) 123, (char) 231, (char) 168, (char) 229, (char) 87, (char) 234, (char) 207, (char) 141, (char) 15, (char) 63, (char) 127, (char) 40, (char) 231, (char) 119, (char) 143, (char) 9, (char) 161, (char) 98, (char) 169, (char) 174, (char) 67, (char) 87, (char) 40, (char) 56, (char) 243, (char) 168, (char) 87, (char) 241, (char) 137, (char) 148, (char) 105, (char) 3, (char) 161, (char) 124, (char) 82, (char) 95, (char) 248, (char) 135, (char) 119, (char) 64, (char) 164, (char) 52, (char) 227, (char) 229, (char) 198, (char) 50, (char) 14, (char) 88, (char) 175, (char) 103, (char) 28, (char) 69, (char) 9, (char) 68, (char) 234, (char) 231, (char) 73, (char) 156, (char) 26, (char) 156, (char) 240, (char) 0, (char) 32, (char) 168, (char) 11, (char) 78, (char) 88, (char) 225, (char) 98, (char) 121, (char) 11}));
			assert (pack.request_id_GET() == (char) 56);
			assert (pack.transfer_type_GET() == (char) 16);
			assert (Arrays.equals(pack.uri_GET(), new char[]{(char) 118, (char) 225, (char) 252, (char) 186, (char) 254, (char) 69, (char) 237, (char) 66, (char) 45, (char) 192, (char) 158, (char) 187, (char) 83, (char) 132, (char) 172, (char) 202, (char) 189, (char) 140, (char) 36, (char) 207, (char) 111, (char) 115, (char) 52, (char) 96, (char) 215, (char) 115, (char) 227, (char) 150, (char) 220, (char) 228, (char) 158, (char) 110, (char) 236, (char) 58, (char) 201, (char) 46, (char) 32, (char) 18, (char) 227, (char) 223, (char) 42, (char) 248, (char) 119, (char) 210, (char) 54, (char) 181, (char) 136, (char) 93, (char) 206, (char) 247, (char) 159, (char) 218, (char) 192, (char) 14, (char) 179, (char) 229, (char) 61, (char) 87, (char) 168, (char) 77, (char) 223, (char) 154, (char) 252, (char) 22, (char) 31, (char) 191, (char) 249, (char) 128, (char) 32, (char) 107, (char) 211, (char) 127, (char) 112, (char) 32, (char) 146, (char) 119, (char) 79, (char) 40, (char) 39, (char) 71, (char) 167, (char) 154, (char) 255, (char) 66, (char) 189, (char) 74, (char) 205, (char) 28, (char) 40, (char) 3, (char) 110, (char) 183, (char) 31, (char) 86, (char) 199, (char) 237, (char) 238, (char) 236, (char) 19, (char) 52, (char) 103, (char) 228, (char) 229, (char) 234, (char) 187, (char) 239, (char) 192, (char) 94, (char) 225, (char) 3, (char) 27, (char) 176, (char) 57, (char) 184, (char) 182, (char) 125, (char) 42, (char) 154, (char) 98, (char) 158}));
		});
		GroundControl.RESOURCE_REQUEST p142 = CommunicationChannel.new_RESOURCE_REQUEST();
		PH.setPack(p142);
		p142.uri_SET(new char[]{(char) 118, (char) 225, (char) 252, (char) 186, (char) 254, (char) 69, (char) 237, (char) 66, (char) 45, (char) 192, (char) 158, (char) 187, (char) 83, (char) 132, (char) 172, (char) 202, (char) 189, (char) 140, (char) 36, (char) 207, (char) 111, (char) 115, (char) 52, (char) 96, (char) 215, (char) 115, (char) 227, (char) 150, (char) 220, (char) 228, (char) 158, (char) 110, (char) 236, (char) 58, (char) 201, (char) 46, (char) 32, (char) 18, (char) 227, (char) 223, (char) 42, (char) 248, (char) 119, (char) 210, (char) 54, (char) 181, (char) 136, (char) 93, (char) 206, (char) 247, (char) 159, (char) 218, (char) 192, (char) 14, (char) 179, (char) 229, (char) 61, (char) 87, (char) 168, (char) 77, (char) 223, (char) 154, (char) 252, (char) 22, (char) 31, (char) 191, (char) 249, (char) 128, (char) 32, (char) 107, (char) 211, (char) 127, (char) 112, (char) 32, (char) 146, (char) 119, (char) 79, (char) 40, (char) 39, (char) 71, (char) 167, (char) 154, (char) 255, (char) 66, (char) 189, (char) 74, (char) 205, (char) 28, (char) 40, (char) 3, (char) 110, (char) 183, (char) 31, (char) 86, (char) 199, (char) 237, (char) 238, (char) 236, (char) 19, (char) 52, (char) 103, (char) 228, (char) 229, (char) 234, (char) 187, (char) 239, (char) 192, (char) 94, (char) 225, (char) 3, (char) 27, (char) 176, (char) 57, (char) 184, (char) 182, (char) 125, (char) 42, (char) 154, (char) 98, (char) 158}, 0);
		p142.storage_SET(new char[]{(char) 228, (char) 45, (char) 246, (char) 209, (char) 54, (char) 141, (char) 127, (char) 171, (char) 89, (char) 212, (char) 76, (char) 139, (char) 174, (char) 213, (char) 235, (char) 193, (char) 245, (char) 209, (char) 38, (char) 255, (char) 118, (char) 35, (char) 178, (char) 152, (char) 205, (char) 75, (char) 154, (char) 95, (char) 118, (char) 149, (char) 76, (char) 195, (char) 100, (char) 118, (char) 6, (char) 54, (char) 60, (char) 70, (char) 189, (char) 218, (char) 112, (char) 244, (char) 15, (char) 44, (char) 131, (char) 108, (char) 1, (char) 105, (char) 139, (char) 123, (char) 231, (char) 168, (char) 229, (char) 87, (char) 234, (char) 207, (char) 141, (char) 15, (char) 63, (char) 127, (char) 40, (char) 231, (char) 119, (char) 143, (char) 9, (char) 161, (char) 98, (char) 169, (char) 174, (char) 67, (char) 87, (char) 40, (char) 56, (char) 243, (char) 168, (char) 87, (char) 241, (char) 137, (char) 148, (char) 105, (char) 3, (char) 161, (char) 124, (char) 82, (char) 95, (char) 248, (char) 135, (char) 119, (char) 64, (char) 164, (char) 52, (char) 227, (char) 229, (char) 198, (char) 50, (char) 14, (char) 88, (char) 175, (char) 103, (char) 28, (char) 69, (char) 9, (char) 68, (char) 234, (char) 231, (char) 73, (char) 156, (char) 26, (char) 156, (char) 240, (char) 0, (char) 32, (char) 168, (char) 11, (char) 78, (char) 88, (char) 225, (char) 98, (char) 121, (char) 11}, 0);
		p142.request_id_SET((char) 56);
		p142.transfer_type_SET((char) 16);
		p142.uri_type_SET((char) 165);
		CommunicationChannel.instance.send(p142);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_SCALED_PRESSURE3.add((src, ph, pack) ->
		{
			assert (pack.press_diff_GET() == 9.448901E37F);
			assert (pack.press_abs_GET() == -3.2164322E38F);
			assert (pack.time_boot_ms_GET() == 3022837623L);
			assert (pack.temperature_GET() == (short) 7482);
		});
		GroundControl.SCALED_PRESSURE3 p143 = CommunicationChannel.new_SCALED_PRESSURE3();
		PH.setPack(p143);
		p143.press_diff_SET(9.448901E37F);
		p143.press_abs_SET(-3.2164322E38F);
		p143.temperature_SET((short) 7482);
		p143.time_boot_ms_SET(3022837623L);
		CommunicationChannel.instance.send(p143);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_FOLLOW_TARGET.add((src, ph, pack) ->
		{
			assert (pack.lat_GET() == -309841121);
			assert (Arrays.equals(pack.acc_GET(), new float[]{-2.888922E38F, -2.1889454E38F, 1.2074952E38F}));
			assert (Arrays.equals(pack.attitude_q_GET(), new float[]{7.783132E37F, -1.8186514E36F, -2.7423347E38F, 2.1695842E38F}));
			assert (pack.est_capabilities_GET() == (char) 136);
			assert (Arrays.equals(pack.position_cov_GET(), new float[]{2.6333222E38F, 2.2125492E38F, -1.1997049E38F}));
			assert (pack.custom_state_GET() == 2544059791198573455L);
			assert (pack.alt_GET() == -6.6563694E37F);
			assert (pack.lon_GET() == -1788308168);
			assert (Arrays.equals(pack.rates_GET(), new float[]{3.239032E36F, 1.510459E38F, 2.7008633E38F}));
			assert (pack.timestamp_GET() == 8867370787658305773L);
			assert (Arrays.equals(pack.vel_GET(), new float[]{-3.3975932E38F, 2.1236914E38F, 3.1429806E38F}));
		});
		GroundControl.FOLLOW_TARGET p144 = CommunicationChannel.new_FOLLOW_TARGET();
		PH.setPack(p144);
		p144.custom_state_SET(2544059791198573455L);
		p144.alt_SET(-6.6563694E37F);
		p144.vel_SET(new float[]{-3.3975932E38F, 2.1236914E38F, 3.1429806E38F}, 0);
		p144.lon_SET(-1788308168);
		p144.attitude_q_SET(new float[]{7.783132E37F, -1.8186514E36F, -2.7423347E38F, 2.1695842E38F}, 0);
		p144.acc_SET(new float[]{-2.888922E38F, -2.1889454E38F, 1.2074952E38F}, 0);
		p144.position_cov_SET(new float[]{2.6333222E38F, 2.2125492E38F, -1.1997049E38F}, 0);
		p144.est_capabilities_SET((char) 136);
		p144.rates_SET(new float[]{3.239032E36F, 1.510459E38F, 2.7008633E38F}, 0);
		p144.timestamp_SET(8867370787658305773L);
		p144.lat_SET(-309841121);
		CommunicationChannel.instance.send(p144);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_CONTROL_SYSTEM_STATE.add((src, ph, pack) ->
		{
			assert (pack.y_pos_GET() == -2.067338E38F);
			assert (Arrays.equals(pack.pos_variance_GET(), new float[]{-8.661036E36F, 2.4821361E38F, -2.9736966E38F}));
			assert (pack.airspeed_GET() == -1.1040195E38F);
			assert (pack.pitch_rate_GET() == 3.095722E36F);
			assert (pack.yaw_rate_GET() == 2.135257E38F);
			assert (pack.y_vel_GET() == -1.6918995E38F);
			assert (pack.z_acc_GET() == 6.375624E37F);
			assert (pack.x_vel_GET() == 2.0472041E37F);
			assert (pack.y_acc_GET() == 1.4551766E38F);
			assert (pack.z_vel_GET() == 1.1600382E38F);
			assert (Arrays.equals(pack.vel_variance_GET(), new float[]{-2.218714E38F, -1.4835027E37F, 2.4941798E38F}));
			assert (pack.time_usec_GET() == 6928978303225103530L);
			assert (Arrays.equals(pack.q_GET(), new float[]{-8.3616693E37F, 2.2257673E38F, 2.4053648E38F, -2.109166E38F}));
			assert (pack.z_pos_GET() == -1.0532025E38F);
			assert (pack.x_pos_GET() == -1.4465944E38F);
			assert (pack.roll_rate_GET() == 2.1640818E38F);
			assert (pack.x_acc_GET() == -3.348704E38F);
		});
		GroundControl.CONTROL_SYSTEM_STATE p146 = CommunicationChannel.new_CONTROL_SYSTEM_STATE();
		PH.setPack(p146);
		p146.y_pos_SET(-2.067338E38F);
		p146.x_vel_SET(2.0472041E37F);
		p146.z_pos_SET(-1.0532025E38F);
		p146.pitch_rate_SET(3.095722E36F);
		p146.q_SET(new float[]{-8.3616693E37F, 2.2257673E38F, 2.4053648E38F, -2.109166E38F}, 0);
		p146.time_usec_SET(6928978303225103530L);
		p146.y_vel_SET(-1.6918995E38F);
		p146.vel_variance_SET(new float[]{-2.218714E38F, -1.4835027E37F, 2.4941798E38F}, 0);
		p146.z_acc_SET(6.375624E37F);
		p146.yaw_rate_SET(2.135257E38F);
		p146.y_acc_SET(1.4551766E38F);
		p146.airspeed_SET(-1.1040195E38F);
		p146.roll_rate_SET(2.1640818E38F);
		p146.x_pos_SET(-1.4465944E38F);
		p146.z_vel_SET(1.1600382E38F);
		p146.pos_variance_SET(new float[]{-8.661036E36F, 2.4821361E38F, -2.9736966E38F}, 0);
		p146.x_acc_SET(-3.348704E38F);
		CommunicationChannel.instance.send(p146);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		TestChannel.instance.on_BATTERY_STATUS.add((src, ph, pack) ->
		{
			assert (pack.temperature_GET() == (short) -27432);
			assert (pack.battery_function_GET() == MAV_BATTERY_FUNCTION.MAV_BATTERY_TYPE_PAYLOAD);
			assert (pack.type_GET() == MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_LIFE);
			assert (pack.current_consumed_GET() == -322605872);
			assert (pack.energy_consumed_GET() == -263096952);
			assert (Arrays.equals(pack.voltages_GET(), new char[]{(char) 12518, (char) 49979, (char) 8221, (char) 23763, (char) 44374, (char) 10083, (char) 7645, (char) 35441, (char) 42008, (char) 59812}));
			assert (pack.battery_remaining_GET() == (byte) -24);
			assert (pack.current_battery_GET() == (short) 3312);
			assert (pack.id_GET() == (char) 18);
		});
		GroundControl.BATTERY_STATUS p147 = CommunicationChannel.new_BATTERY_STATUS();
		PH.setPack(p147);
		p147.energy_consumed_SET(-263096952);
		p147.voltages_SET(new char[]{(char) 12518, (char) 49979, (char) 8221, (char) 23763, (char) 44374, (char) 10083, (char) 7645, (char) 35441, (char) 42008, (char) 59812}, 0);
		p147.type_SET(MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_LIFE);
		p147.id_SET((char) 18);
		p147.temperature_SET((short) -27432);
		p147.battery_remaining_SET((byte) -24);
		p147.current_consumed_SET(-322605872);
		p147.current_battery_SET((short) 3312);
		p147.battery_function_SET(MAV_BATTERY_FUNCTION.MAV_BATTERY_TYPE_PAYLOAD);
		CommunicationChannel.instance.send(p147);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_AUTOPILOT_VERSION.add((src, ph, pack) ->
		{
			assert (pack.flight_sw_version_GET() == 2909425696L);
			assert (pack.os_sw_version_GET() == 1560923350L);
			assert (pack.board_version_GET() == 2533699808L);
			assert (Arrays.equals(pack.os_custom_version_GET(), new char[]{(char) 95, (char) 47, (char) 115, (char) 144, (char) 87, (char) 169, (char) 64, (char) 255}));
			assert (pack.uid_GET() == 6710170156458325886L);
			assert (Arrays.equals(pack.middleware_custom_version_GET(), new char[]{(char) 149, (char) 66, (char) 0, (char) 164, (char) 132, (char) 225, (char) 5, (char) 114}));
			assert (pack.product_id_GET() == (char) 10177);
			assert (Arrays.equals(pack.flight_custom_version_GET(), new char[]{(char) 171, (char) 40, (char) 75, (char) 128, (char) 160, (char) 123, (char) 169, (char) 110}));
			assert (pack.vendor_id_GET() == (char) 58515);
			assert (Arrays.equals(pack.uid2_TRY(ph), new char[]{(char) 120, (char) 50, (char) 6, (char) 224, (char) 10, (char) 44, (char) 253, (char) 178, (char) 84, (char) 223, (char) 105, (char) 160, (char) 63, (char) 111, (char) 162, (char) 216, (char) 43, (char) 53}));
			assert (pack.capabilities_GET() == MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FTP);
			assert (pack.middleware_sw_version_GET() == 3401979727L);
		});
		GroundControl.AUTOPILOT_VERSION p148 = CommunicationChannel.new_AUTOPILOT_VERSION();
		PH.setPack(p148);
		p148.uid_SET(6710170156458325886L);
		p148.flight_custom_version_SET(new char[]{(char) 171, (char) 40, (char) 75, (char) 128, (char) 160, (char) 123, (char) 169, (char) 110}, 0);
		p148.os_custom_version_SET(new char[]{(char) 95, (char) 47, (char) 115, (char) 144, (char) 87, (char) 169, (char) 64, (char) 255}, 0);
		p148.vendor_id_SET((char) 58515);
		p148.product_id_SET((char) 10177);
		p148.board_version_SET(2533699808L);
		p148.middleware_sw_version_SET(3401979727L);
		p148.flight_sw_version_SET(2909425696L);
		p148.os_sw_version_SET(1560923350L);
		p148.middleware_custom_version_SET(new char[]{(char) 149, (char) 66, (char) 0, (char) 164, (char) 132, (char) 225, (char) 5, (char) 114}, 0);
		p148.capabilities_SET(MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FTP);
		p148.uid2_SET(new char[]{(char) 120, (char) 50, (char) 6, (char) 224, (char) 10, (char) 44, (char) 253, (char) 178, (char) 84, (char) 223, (char) 105, (char) 160, (char) 63, (char) 111, (char) 162, (char) 216, (char) 43, (char) 53}, 0, PH);
		CommunicationChannel.instance.send(p148);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_LANDING_TARGET.add((src, ph, pack) ->
		{
			assert (pack.distance_GET() == 8.771835E37F);
			assert (pack.target_num_GET() == (char) 12);
			assert (pack.z_TRY(ph) == -3.0510476E38F);
			assert (pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_INT);
			assert (pack.type_GET() == LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_VISION_OTHER);
			assert (pack.size_y_GET() == -6.945209E37F);
			assert (pack.position_valid_TRY(ph) == (char) 255);
			assert (pack.y_TRY(ph) == -2.3709186E38F);
			assert (pack.angle_y_GET() == 2.5360115E38F);
			assert (pack.time_usec_GET() == 2939018901310388028L);
			assert (pack.angle_x_GET() == -1.2950698E38F);
			assert (pack.x_TRY(ph) == -1.6050073E38F);
			assert (pack.size_x_GET() == 2.0116615E38F);
			assert (Arrays.equals(pack.q_TRY(ph), new float[]{3.024216E38F, 2.3405213E38F, -2.1073653E38F, 1.5904514E38F}));
		});
		GroundControl.LANDING_TARGET p149 = CommunicationChannel.new_LANDING_TARGET();
		PH.setPack(p149);
		p149.q_SET(new float[]{3.024216E38F, 2.3405213E38F, -2.1073653E38F, 1.5904514E38F}, 0, PH);
		p149.position_valid_SET((char) 255, PH);
		p149.size_y_SET(-6.945209E37F);
		p149.target_num_SET((char) 12);
		p149.size_x_SET(2.0116615E38F);
		p149.z_SET(-3.0510476E38F, PH);
		p149.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_INT);
		p149.time_usec_SET(2939018901310388028L);
		p149.angle_y_SET(2.5360115E38F);
		p149.angle_x_SET(-1.2950698E38F);
		p149.distance_SET(8.771835E37F);
		p149.x_SET(-1.6050073E38F, PH);
		p149.y_SET(-2.3709186E38F, PH);
		p149.type_SET(LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_VISION_OTHER);
		CommunicationChannel.instance.send(p149);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_SENS_POWER.add((src, ph, pack) ->
		{
			assert (pack.adc121_vspb_volt_GET() == 2.910951E38F);
			assert (pack.adc121_cs1_amp_GET() == 1.8826559E38F);
			assert (pack.adc121_cspb_amp_GET() == -1.1483053E38F);
			assert (pack.adc121_cs2_amp_GET() == 1.747924E38F);
		});
		GroundControl.SENS_POWER p201 = CommunicationChannel.new_SENS_POWER();
		PH.setPack(p201);
		p201.adc121_vspb_volt_SET(2.910951E38F);
		p201.adc121_cs1_amp_SET(1.8826559E38F);
		p201.adc121_cs2_amp_SET(1.747924E38F);
		p201.adc121_cspb_amp_SET(-1.1483053E38F);
		CommunicationChannel.instance.send(p201);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_SENS_MPPT.add((src, ph, pack) ->
		{
			assert (pack.mppt3_pwm_GET() == (char) 36977);
			assert (pack.mppt3_status_GET() == (char) 110);
			assert (pack.mppt_timestamp_GET() == 3475570984862630455L);
			assert (pack.mppt3_volt_GET() == -1.8185947E38F);
			assert (pack.mppt1_volt_GET() == -1.2197457E38F);
			assert (pack.mppt2_amp_GET() == -8.714441E37F);
			assert (pack.mppt1_amp_GET() == 7.727526E37F);
			assert (pack.mppt2_status_GET() == (char) 138);
			assert (pack.mppt3_amp_GET() == 6.467236E37F);
			assert (pack.mppt1_status_GET() == (char) 149);
			assert (pack.mppt1_pwm_GET() == (char) 38897);
			assert (pack.mppt2_pwm_GET() == (char) 12756);
			assert (pack.mppt2_volt_GET() == 2.3061063E38F);
		});
		GroundControl.SENS_MPPT p202 = CommunicationChannel.new_SENS_MPPT();
		PH.setPack(p202);
		p202.mppt1_pwm_SET((char) 38897);
		p202.mppt3_pwm_SET((char) 36977);
		p202.mppt1_volt_SET(-1.2197457E38F);
		p202.mppt2_pwm_SET((char) 12756);
		p202.mppt2_volt_SET(2.3061063E38F);
		p202.mppt3_amp_SET(6.467236E37F);
		p202.mppt2_status_SET((char) 138);
		p202.mppt3_volt_SET(-1.8185947E38F);
		p202.mppt_timestamp_SET(3475570984862630455L);
		p202.mppt1_amp_SET(7.727526E37F);
		p202.mppt2_amp_SET(-8.714441E37F);
		p202.mppt1_status_SET((char) 149);
		p202.mppt3_status_SET((char) 110);
		CommunicationChannel.instance.send(p202);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_ASLCTRL_DATA.add((src, ph, pack) ->
		{
			assert (pack.hRef_t_GET() == -3.0934622E38F);
			assert (pack.YawAngle_GET() == 2.7117742E38F);
			assert (pack.q_GET() == -7.180148E37F);
			assert (pack.uThrot2_GET() == -1.2013345E38F);
			assert (pack.pRef_GET() == -2.2479185E38F);
			assert (pack.uAil_GET() == -2.5618002E38F);
			assert (pack.uThrot_GET() == 2.904646E38F);
			assert (pack.YawAngleRef_GET() == -2.2879408E38F);
			assert (pack.p_GET() == 1.3149447E36F);
			assert (pack.RollAngle_GET() == -4.7577594E37F);
			assert (pack.SpoilersEngaged_GET() == (char) 31);
			assert (pack.hRef_GET() == -1.8653917E38F);
			assert (pack.uRud_GET() == 1.5414449E38F);
			assert (pack.AirspeedRef_GET() == -2.7961007E38F);
			assert (pack.timestamp_GET() == 2973355354378873393L);
			assert (pack.PitchAngleRef_GET() == 1.6670813E37F);
			assert (pack.nZ_GET() == 2.0527E38F);
			assert (pack.rRef_GET() == -1.386265E38F);
			assert (pack.aslctrl_mode_GET() == (char) 107);
			assert (pack.RollAngleRef_GET() == -2.3896816E38F);
			assert (pack.PitchAngle_GET() == 2.8309948E38F);
			assert (pack.qRef_GET() == -1.0171679E38F);
			assert (pack.h_GET() == -4.5474597E37F);
			assert (pack.uElev_GET() == 1.6870934E37F);
			assert (pack.r_GET() == 1.0501888E38F);
		});
		GroundControl.ASLCTRL_DATA p203 = CommunicationChannel.new_ASLCTRL_DATA();
		PH.setPack(p203);
		p203.YawAngle_SET(2.7117742E38F);
		p203.PitchAngle_SET(2.8309948E38F);
		p203.uAil_SET(-2.5618002E38F);
		p203.rRef_SET(-1.386265E38F);
		p203.nZ_SET(2.0527E38F);
		p203.SpoilersEngaged_SET((char) 31);
		p203.aslctrl_mode_SET((char) 107);
		p203.uRud_SET(1.5414449E38F);
		p203.AirspeedRef_SET(-2.7961007E38F);
		p203.uThrot2_SET(-1.2013345E38F);
		p203.hRef_SET(-1.8653917E38F);
		p203.PitchAngleRef_SET(1.6670813E37F);
		p203.h_SET(-4.5474597E37F);
		p203.timestamp_SET(2973355354378873393L);
		p203.qRef_SET(-1.0171679E38F);
		p203.uThrot_SET(2.904646E38F);
		p203.RollAngle_SET(-4.7577594E37F);
		p203.q_SET(-7.180148E37F);
		p203.pRef_SET(-2.2479185E38F);
		p203.uElev_SET(1.6870934E37F);
		p203.hRef_t_SET(-3.0934622E38F);
		p203.p_SET(1.3149447E36F);
		p203.YawAngleRef_SET(-2.2879408E38F);
		p203.RollAngleRef_SET(-2.3896816E38F);
		p203.r_SET(1.0501888E38F);
		CommunicationChannel.instance.send(p203);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_ASLCTRL_DEBUG.add((src, ph, pack) ->
		{
			assert (pack.f_3_GET() == 2.551315E38F);
			assert (pack.f_8_GET() == 2.598335E38F);
			assert (pack.f_2_GET() == 2.189049E38F);
			assert (pack.f_6_GET() == 8.327577E37F);
			assert (pack.f_7_GET() == 1.7386948E38F);
			assert (pack.f_1_GET() == -1.1543453E38F);
			assert (pack.i8_1_GET() == (char) 167);
			assert (pack.i8_2_GET() == (char) 40);
			assert (pack.f_5_GET() == 2.5457355E38F);
			assert (pack.f_4_GET() == -2.2837817E38F);
			assert (pack.i32_1_GET() == 3333785990L);
		});
		GroundControl.ASLCTRL_DEBUG p204 = CommunicationChannel.new_ASLCTRL_DEBUG();
		PH.setPack(p204);
		p204.i32_1_SET(3333785990L);
		p204.f_7_SET(1.7386948E38F);
		p204.i8_2_SET((char) 40);
		p204.f_6_SET(8.327577E37F);
		p204.f_8_SET(2.598335E38F);
		p204.i8_1_SET((char) 167);
		p204.f_3_SET(2.551315E38F);
		p204.f_2_SET(2.189049E38F);
		p204.f_1_SET(-1.1543453E38F);
		p204.f_5_SET(2.5457355E38F);
		p204.f_4_SET(-2.2837817E38F);
		CommunicationChannel.instance.send(p204);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_ASLUAV_STATUS.add((src, ph, pack) ->
		{
			assert (pack.Motor_rpm_GET() == -5.2021825E37F);
			assert (Arrays.equals(pack.Servo_status_GET(), new char[]{(char) 104, (char) 228, (char) 89, (char) 82, (char) 196, (char) 192, (char) 43, (char) 254}));
			assert (pack.SATCOM_status_GET() == (char) 184);
			assert (pack.LED_status_GET() == (char) 86);
		});
		GroundControl.ASLUAV_STATUS p205 = CommunicationChannel.new_ASLUAV_STATUS();
		PH.setPack(p205);
		p205.LED_status_SET((char) 86);
		p205.Motor_rpm_SET(-5.2021825E37F);
		p205.Servo_status_SET(new char[]{(char) 104, (char) 228, (char) 89, (char) 82, (char) 196, (char) 192, (char) 43, (char) 254}, 0);
		p205.SATCOM_status_SET((char) 184);
		CommunicationChannel.instance.send(p205);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_EKF_EXT.add((src, ph, pack) ->
		{
			assert (pack.beta_GET() == -2.2105178E38F);
			assert (pack.alpha_GET() == -2.8475268E38F);
			assert (pack.timestamp_GET() == 2338941705051366233L);
			assert (pack.WindDir_GET() == 1.2452086E38F);
			assert (pack.Windspeed_GET() == 1.5170567E37F);
			assert (pack.Airspeed_GET() == -2.0723098E38F);
			assert (pack.WindZ_GET() == -1.90987E38F);
		});
		GroundControl.EKF_EXT p206 = CommunicationChannel.new_EKF_EXT();
		PH.setPack(p206);
		p206.WindZ_SET(-1.90987E38F);
		p206.beta_SET(-2.2105178E38F);
		p206.Windspeed_SET(1.5170567E37F);
		p206.WindDir_SET(1.2452086E38F);
		p206.alpha_SET(-2.8475268E38F);
		p206.timestamp_SET(2338941705051366233L);
		p206.Airspeed_SET(-2.0723098E38F);
		CommunicationChannel.instance.send(p206);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_ASL_OBCTRL.add((src, ph, pack) ->
		{
			assert (pack.obctrl_status_GET() == (char) 180);
			assert (pack.uThrot2_GET() == -1.5753918E38F);
			assert (pack.uAilL_GET() == 1.1981945E38F);
			assert (pack.uElev_GET() == -1.9985648E38F);
			assert (pack.uThrot_GET() == -1.4719837E38F);
			assert (pack.timestamp_GET() == 7710641004851129138L);
			assert (pack.uRud_GET() == 2.5316747E38F);
			assert (pack.uAilR_GET() == -1.9709695E38F);
		});
		GroundControl.ASL_OBCTRL p207 = CommunicationChannel.new_ASL_OBCTRL();
		PH.setPack(p207);
		p207.uAilL_SET(1.1981945E38F);
		p207.uAilR_SET(-1.9709695E38F);
		p207.uRud_SET(2.5316747E38F);
		p207.uThrot_SET(-1.4719837E38F);
		p207.timestamp_SET(7710641004851129138L);
		p207.uThrot2_SET(-1.5753918E38F);
		p207.uElev_SET(-1.9985648E38F);
		p207.obctrl_status_SET((char) 180);
		CommunicationChannel.instance.send(p207);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_SENS_ATMOS.add((src, ph, pack) ->
		{
			assert (pack.TempAmbient_GET() == 1.4871705E38F);
			assert (pack.Humidity_GET() == -9.657188E37F);
		});
		GroundControl.SENS_ATMOS p208 = CommunicationChannel.new_SENS_ATMOS();
		PH.setPack(p208);
		p208.TempAmbient_SET(1.4871705E38F);
		p208.Humidity_SET(-9.657188E37F);
		CommunicationChannel.instance.send(p208);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_SENS_BATMON.add((src, ph, pack) ->
		{
			assert (pack.cellvoltage5_GET() == (char) 64768);
			assert (pack.voltage_GET() == (char) 42263);
			assert (pack.hostfetcontrol_GET() == (char) 58648);
			assert (pack.current_GET() == (short) -27655);
			assert (pack.cellvoltage2_GET() == (char) 25241);
			assert (pack.cellvoltage3_GET() == (char) 52119);
			assert (pack.serialnumber_GET() == (char) 64442);
			assert (pack.cellvoltage1_GET() == (char) 24003);
			assert (pack.cellvoltage4_GET() == (char) 19521);
			assert (pack.temperature_GET() == -4.9276834E37F);
			assert (pack.cellvoltage6_GET() == (char) 31647);
			assert (pack.SoC_GET() == (char) 62);
			assert (pack.batterystatus_GET() == (char) 44062);
		});
		GroundControl.SENS_BATMON p209 = CommunicationChannel.new_SENS_BATMON();
		PH.setPack(p209);
		p209.serialnumber_SET((char) 64442);
		p209.temperature_SET(-4.9276834E37F);
		p209.cellvoltage2_SET((char) 25241);
		p209.cellvoltage5_SET((char) 64768);
		p209.batterystatus_SET((char) 44062);
		p209.cellvoltage6_SET((char) 31647);
		p209.hostfetcontrol_SET((char) 58648);
		p209.cellvoltage3_SET((char) 52119);
		p209.current_SET((short) -27655);
		p209.voltage_SET((char) 42263);
		p209.cellvoltage4_SET((char) 19521);
		p209.cellvoltage1_SET((char) 24003);
		p209.SoC_SET((char) 62);
		CommunicationChannel.instance.send(p209);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_FW_SOARING_DATA.add((src, ph, pack) ->
		{
			assert (pack.z2_DeltaRoll_GET() == 6.3403355E37F);
			assert (pack.ThermalGSEast_GET() == -5.056871E37F);
			assert (pack.ControlMode_GET() == (char) 64);
			assert (pack.VarLon_GET() == 2.9891716E38F);
			assert (pack.DebugVar2_GET() == -3.392475E35F);
			assert (pack.VarLat_GET() == 9.12998E37F);
			assert (pack.ThermalGSNorth_GET() == 7.658112E37F);
			assert (pack.vSinkExp_GET() == 2.347372E38F);
			assert (pack.DebugVar1_GET() == -3.2980808E38F);
			assert (pack.timestamp_GET() == 4359865251402112460L);
			assert (pack.xLon_GET() == -2.443218E38F);
			assert (pack.DistToSoarPoint_GET() == -1.6627767E38F);
			assert (pack.z2_exp_GET() == 9.587727E37F);
			assert (pack.xR_GET() == 1.1259236E38F);
			assert (pack.VarR_GET() == 2.8777192E38F);
			assert (pack.xLat_GET() == 3.0571169E38F);
			assert (pack.z1_exp_GET() == -3.3194958E38F);
			assert (pack.LoiterDirection_GET() == -1.4516458E38F);
			assert (pack.TSE_dot_GET() == 1.8644525E37F);
			assert (pack.xW_GET() == -2.8671557E38F);
			assert (pack.z1_LocalUpdraftSpeed_GET() == -7.380339E37F);
			assert (pack.LoiterRadius_GET() == 1.1075457E38F);
			assert (pack.valid_GET() == (char) 238);
			assert (pack.timestampModeChanged_GET() == 6657142614997888923L);
			assert (pack.VarW_GET() == 2.550959E38F);
		});
		GroundControl.FW_SOARING_DATA p210 = CommunicationChannel.new_FW_SOARING_DATA();
		PH.setPack(p210);
		p210.valid_SET((char) 238);
		p210.TSE_dot_SET(1.8644525E37F);
		p210.timestampModeChanged_SET(6657142614997888923L);
		p210.DistToSoarPoint_SET(-1.6627767E38F);
		p210.z1_exp_SET(-3.3194958E38F);
		p210.DebugVar1_SET(-3.2980808E38F);
		p210.z2_DeltaRoll_SET(6.3403355E37F);
		p210.z2_exp_SET(9.587727E37F);
		p210.ControlMode_SET((char) 64);
		p210.VarW_SET(2.550959E38F);
		p210.LoiterRadius_SET(1.1075457E38F);
		p210.ThermalGSNorth_SET(7.658112E37F);
		p210.xW_SET(-2.8671557E38F);
		p210.xLon_SET(-2.443218E38F);
		p210.VarLat_SET(9.12998E37F);
		p210.VarR_SET(2.8777192E38F);
		p210.VarLon_SET(2.9891716E38F);
		p210.timestamp_SET(4359865251402112460L);
		p210.LoiterDirection_SET(-1.4516458E38F);
		p210.vSinkExp_SET(2.347372E38F);
		p210.ThermalGSEast_SET(-5.056871E37F);
		p210.z1_LocalUpdraftSpeed_SET(-7.380339E37F);
		p210.DebugVar2_SET(-3.392475E35F);
		p210.xLat_SET(3.0571169E38F);
		p210.xR_SET(1.1259236E38F);
		CommunicationChannel.instance.send(p210);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_SENSORPOD_STATUS.add((src, ph, pack) ->
		{
			assert (pack.recording_nodes_count_GET() == (char) 206);
			assert (pack.cpu_temp_GET() == (char) 72);
			assert (pack.visensor_rate_4_GET() == (char) 18);
			assert (pack.visensor_rate_1_GET() == (char) 175);
			assert (pack.visensor_rate_3_GET() == (char) 236);
			assert (pack.visensor_rate_2_GET() == (char) 29);
			assert (pack.free_space_GET() == (char) 39783);
			assert (pack.timestamp_GET() == 7697999806838054840L);
		});
		GroundControl.SENSORPOD_STATUS p211 = CommunicationChannel.new_SENSORPOD_STATUS();
		PH.setPack(p211);
		p211.visensor_rate_1_SET((char) 175);
		p211.timestamp_SET(7697999806838054840L);
		p211.visensor_rate_3_SET((char) 236);
		p211.recording_nodes_count_SET((char) 206);
		p211.visensor_rate_4_SET((char) 18);
		p211.cpu_temp_SET((char) 72);
		p211.visensor_rate_2_SET((char) 29);
		p211.free_space_SET((char) 39783);
		CommunicationChannel.instance.send(p211);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_SENS_POWER_BOARD.add((src, ph, pack) ->
		{
			assert (pack.pwr_brd_aux_amp_GET() == -2.2902015E38F);
			assert (pack.pwr_brd_led_status_GET() == (char) 187);
			assert (pack.pwr_brd_servo_1_amp_GET() == -2.5036483E38F);
			assert (pack.pwr_brd_servo_3_amp_GET() == 1.805955E38F);
			assert (pack.pwr_brd_mot_r_amp_GET() == -1.7995961E38F);
			assert (pack.pwr_brd_status_GET() == (char) 150);
			assert (pack.timestamp_GET() == 5496752364630275665L);
			assert (pack.pwr_brd_servo_2_amp_GET() == -2.075939E38F);
			assert (pack.pwr_brd_servo_4_amp_GET() == -1.861421E38F);
			assert (pack.pwr_brd_servo_volt_GET() == 5.455313E36F);
			assert (pack.pwr_brd_mot_l_amp_GET() == 2.7859932E38F);
			assert (pack.pwr_brd_system_volt_GET() == 7.6274303E37F);
		});
		GroundControl.SENS_POWER_BOARD p212 = CommunicationChannel.new_SENS_POWER_BOARD();
		PH.setPack(p212);
		p212.pwr_brd_servo_4_amp_SET(-1.861421E38F);
		p212.pwr_brd_mot_r_amp_SET(-1.7995961E38F);
		p212.pwr_brd_servo_2_amp_SET(-2.075939E38F);
		p212.pwr_brd_status_SET((char) 150);
		p212.pwr_brd_system_volt_SET(7.6274303E37F);
		p212.pwr_brd_servo_volt_SET(5.455313E36F);
		p212.pwr_brd_aux_amp_SET(-2.2902015E38F);
		p212.timestamp_SET(5496752364630275665L);
		p212.pwr_brd_servo_1_amp_SET(-2.5036483E38F);
		p212.pwr_brd_led_status_SET((char) 187);
		p212.pwr_brd_mot_l_amp_SET(2.7859932E38F);
		p212.pwr_brd_servo_3_amp_SET(1.805955E38F);
		CommunicationChannel.instance.send(p212);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_ESTIMATOR_STATUS.add((src, ph, pack) ->
		{
			assert (pack.mag_ratio_GET() == -1.2020523E38F);
			assert (pack.vel_ratio_GET() == 1.6478398E38F);
			assert (pack.pos_horiz_accuracy_GET() == 2.4269493E38F);
			assert (pack.time_usec_GET() == 4359789014064543886L);
			assert (pack.pos_vert_ratio_GET() == -3.8681442E36F);
			assert (pack.hagl_ratio_GET() == -3.2813797E38F);
			assert (pack.flags_GET() == ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_HORIZ_REL);
			assert (pack.tas_ratio_GET() == -1.6883458E38F);
			assert (pack.pos_vert_accuracy_GET() == -2.4449104E38F);
			assert (pack.pos_horiz_ratio_GET() == -3.3737304E38F);
		});
		GroundControl.ESTIMATOR_STATUS p230 = CommunicationChannel.new_ESTIMATOR_STATUS();
		PH.setPack(p230);
		p230.time_usec_SET(4359789014064543886L);
		p230.mag_ratio_SET(-1.2020523E38F);
		p230.vel_ratio_SET(1.6478398E38F);
		p230.pos_vert_ratio_SET(-3.8681442E36F);
		p230.hagl_ratio_SET(-3.2813797E38F);
		p230.tas_ratio_SET(-1.6883458E38F);
		p230.pos_vert_accuracy_SET(-2.4449104E38F);
		p230.pos_horiz_accuracy_SET(2.4269493E38F);
		p230.flags_SET(ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_HORIZ_REL);
		p230.pos_horiz_ratio_SET(-3.3737304E38F);
		CommunicationChannel.instance.send(p230);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_WIND_COV.add((src, ph, pack) ->
		{
			assert (pack.time_usec_GET() == 8616598456226938740L);
			assert (pack.wind_alt_GET() == -1.8464457E38F);
			assert (pack.var_vert_GET() == -1.3777786E38F);
			assert (pack.horiz_accuracy_GET() == -2.3831955E38F);
			assert (pack.vert_accuracy_GET() == -3.2579562E38F);
			assert (pack.wind_y_GET() == 3.368218E38F);
			assert (pack.var_horiz_GET() == -2.573427E38F);
			assert (pack.wind_x_GET() == -7.7679353E36F);
			assert (pack.wind_z_GET() == 2.3103885E37F);
		});
		GroundControl.WIND_COV p231 = CommunicationChannel.new_WIND_COV();
		PH.setPack(p231);
		p231.wind_x_SET(-7.7679353E36F);
		p231.wind_y_SET(3.368218E38F);
		p231.horiz_accuracy_SET(-2.3831955E38F);
		p231.var_vert_SET(-1.3777786E38F);
		p231.vert_accuracy_SET(-3.2579562E38F);
		p231.wind_alt_SET(-1.8464457E38F);
		p231.wind_z_SET(2.3103885E37F);
		p231.var_horiz_SET(-2.573427E38F);
		p231.time_usec_SET(8616598456226938740L);
		CommunicationChannel.instance.send(p231);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_GPS_INPUT.add((src, ph, pack) ->
		{
			assert (pack.satellites_visible_GET() == (char) 169);
			assert (pack.vd_GET() == -1.5451439E38F);
			assert (pack.lat_GET() == 1090963211);
			assert (pack.lon_GET() == 1356399479);
			assert (pack.ignore_flags_GET() == GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VDOP);
			assert (pack.gps_id_GET() == (char) 48);
			assert (pack.time_week_ms_GET() == 2100128945L);
			assert (pack.speed_accuracy_GET() == -9.553213E37F);
			assert (pack.time_week_GET() == (char) 7123);
			assert (pack.ve_GET() == 1.4858796E38F);
			assert (pack.vert_accuracy_GET() == -2.4832943E38F);
			assert (pack.horiz_accuracy_GET() == -1.0268191E38F);
			assert (pack.vn_GET() == 1.1512293E38F);
			assert (pack.time_usec_GET() == 8895816344979747711L);
			assert (pack.vdop_GET() == 2.8919826E38F);
			assert (pack.fix_type_GET() == (char) 229);
			assert (pack.alt_GET() == -2.0856158E38F);
			assert (pack.hdop_GET() == -1.6754196E38F);
		});
		GroundControl.GPS_INPUT p232 = CommunicationChannel.new_GPS_INPUT();
		PH.setPack(p232);
		p232.alt_SET(-2.0856158E38F);
		p232.lat_SET(1090963211);
		p232.vert_accuracy_SET(-2.4832943E38F);
		p232.time_week_ms_SET(2100128945L);
		p232.speed_accuracy_SET(-9.553213E37F);
		p232.horiz_accuracy_SET(-1.0268191E38F);
		p232.fix_type_SET((char) 229);
		p232.hdop_SET(-1.6754196E38F);
		p232.time_usec_SET(8895816344979747711L);
		p232.vd_SET(-1.5451439E38F);
		p232.ve_SET(1.4858796E38F);
		p232.vdop_SET(2.8919826E38F);
		p232.satellites_visible_SET((char) 169);
		p232.ignore_flags_SET(GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VDOP);
		p232.time_week_SET((char) 7123);
		p232.gps_id_SET((char) 48);
		p232.lon_SET(1356399479);
		p232.vn_SET(1.1512293E38F);
		CommunicationChannel.instance.send(p232);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_GPS_RTCM_DATA.add((src, ph, pack) ->
		{
			assert (pack.len_GET() == (char) 106);
			assert (Arrays.equals(pack.data__GET(), new char[]{(char) 228, (char) 131, (char) 32, (char) 51, (char) 239, (char) 74, (char) 137, (char) 17, (char) 37, (char) 69, (char) 120, (char) 37, (char) 136, (char) 47, (char) 103, (char) 109, (char) 32, (char) 44, (char) 110, (char) 70, (char) 149, (char) 124, (char) 165, (char) 57, (char) 167, (char) 104, (char) 71, (char) 72, (char) 8, (char) 135, (char) 98, (char) 87, (char) 150, (char) 137, (char) 186, (char) 127, (char) 56, (char) 44, (char) 251, (char) 35, (char) 232, (char) 69, (char) 155, (char) 69, (char) 219, (char) 21, (char) 148, (char) 240, (char) 235, (char) 239, (char) 103, (char) 58, (char) 236, (char) 7, (char) 33, (char) 37, (char) 155, (char) 55, (char) 212, (char) 133, (char) 69, (char) 197, (char) 64, (char) 148, (char) 170, (char) 199, (char) 230, (char) 228, (char) 151, (char) 109, (char) 58, (char) 88, (char) 12, (char) 2, (char) 202, (char) 223, (char) 174, (char) 218, (char) 136, (char) 240, (char) 57, (char) 220, (char) 141, (char) 50, (char) 253, (char) 24, (char) 243, (char) 100, (char) 176, (char) 83, (char) 108, (char) 22, (char) 3, (char) 227, (char) 203, (char) 236, (char) 216, (char) 5, (char) 54, (char) 221, (char) 204, (char) 128, (char) 244, (char) 167, (char) 246, (char) 37, (char) 157, (char) 190, (char) 26, (char) 101, (char) 31, (char) 92, (char) 134, (char) 228, (char) 44, (char) 240, (char) 58, (char) 130, (char) 210, (char) 142, (char) 239, (char) 204, (char) 186, (char) 104, (char) 120, (char) 95, (char) 6, (char) 51, (char) 224, (char) 239, (char) 131, (char) 100, (char) 80, (char) 147, (char) 248, (char) 168, (char) 236, (char) 187, (char) 63, (char) 73, (char) 221, (char) 9, (char) 162, (char) 177, (char) 28, (char) 116, (char) 253, (char) 27, (char) 25, (char) 53, (char) 191, (char) 240, (char) 86, (char) 250, (char) 140, (char) 51, (char) 232, (char) 134, (char) 197, (char) 239, (char) 115, (char) 22, (char) 232, (char) 62, (char) 170, (char) 83, (char) 57, (char) 195, (char) 234, (char) 139, (char) 103, (char) 40, (char) 84, (char) 0, (char) 21, (char) 32, (char) 141, (char) 25, (char) 41, (char) 205}));
			assert (pack.flags_GET() == (char) 86);
		});
		GroundControl.GPS_RTCM_DATA p233 = CommunicationChannel.new_GPS_RTCM_DATA();
		PH.setPack(p233);
		p233.flags_SET((char) 86);
		p233.data__SET(new char[]{(char) 228, (char) 131, (char) 32, (char) 51, (char) 239, (char) 74, (char) 137, (char) 17, (char) 37, (char) 69, (char) 120, (char) 37, (char) 136, (char) 47, (char) 103, (char) 109, (char) 32, (char) 44, (char) 110, (char) 70, (char) 149, (char) 124, (char) 165, (char) 57, (char) 167, (char) 104, (char) 71, (char) 72, (char) 8, (char) 135, (char) 98, (char) 87, (char) 150, (char) 137, (char) 186, (char) 127, (char) 56, (char) 44, (char) 251, (char) 35, (char) 232, (char) 69, (char) 155, (char) 69, (char) 219, (char) 21, (char) 148, (char) 240, (char) 235, (char) 239, (char) 103, (char) 58, (char) 236, (char) 7, (char) 33, (char) 37, (char) 155, (char) 55, (char) 212, (char) 133, (char) 69, (char) 197, (char) 64, (char) 148, (char) 170, (char) 199, (char) 230, (char) 228, (char) 151, (char) 109, (char) 58, (char) 88, (char) 12, (char) 2, (char) 202, (char) 223, (char) 174, (char) 218, (char) 136, (char) 240, (char) 57, (char) 220, (char) 141, (char) 50, (char) 253, (char) 24, (char) 243, (char) 100, (char) 176, (char) 83, (char) 108, (char) 22, (char) 3, (char) 227, (char) 203, (char) 236, (char) 216, (char) 5, (char) 54, (char) 221, (char) 204, (char) 128, (char) 244, (char) 167, (char) 246, (char) 37, (char) 157, (char) 190, (char) 26, (char) 101, (char) 31, (char) 92, (char) 134, (char) 228, (char) 44, (char) 240, (char) 58, (char) 130, (char) 210, (char) 142, (char) 239, (char) 204, (char) 186, (char) 104, (char) 120, (char) 95, (char) 6, (char) 51, (char) 224, (char) 239, (char) 131, (char) 100, (char) 80, (char) 147, (char) 248, (char) 168, (char) 236, (char) 187, (char) 63, (char) 73, (char) 221, (char) 9, (char) 162, (char) 177, (char) 28, (char) 116, (char) 253, (char) 27, (char) 25, (char) 53, (char) 191, (char) 240, (char) 86, (char) 250, (char) 140, (char) 51, (char) 232, (char) 134, (char) 197, (char) 239, (char) 115, (char) 22, (char) 232, (char) 62, (char) 170, (char) 83, (char) 57, (char) 195, (char) 234, (char) 139, (char) 103, (char) 40, (char) 84, (char) 0, (char) 21, (char) 32, (char) 141, (char) 25, (char) 41, (char) 205}, 0);
		p233.len_SET((char) 106);
		CommunicationChannel.instance.send(p233);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_HIGH_LATENCY.add((src, ph, pack) ->
		{
			assert (pack.airspeed_GET() == (char) 165);
			assert (pack.groundspeed_GET() == (char) 194);
			assert (pack.altitude_sp_GET() == (short) 2120);
			assert (pack.longitude_GET() == -1870098575);
			assert (pack.roll_GET() == (short) -30702);
			assert (pack.heading_GET() == (char) 44998);
			assert (pack.climb_rate_GET() == (byte) -119);
			assert (pack.gps_fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_PPP);
			assert (pack.heading_sp_GET() == (short) 20423);
			assert (pack.throttle_GET() == (byte) 4);
			assert (pack.gps_nsat_GET() == (char) 18);
			assert (pack.temperature_GET() == (byte) -46);
			assert (pack.airspeed_sp_GET() == (char) 176);
			assert (pack.wp_num_GET() == (char) 13);
			assert (pack.wp_distance_GET() == (char) 1469);
			assert (pack.failsafe_GET() == (char) 51);
			assert (pack.altitude_amsl_GET() == (short) -6041);
			assert (pack.latitude_GET() == -617179032);
			assert (pack.temperature_air_GET() == (byte) 3);
			assert (pack.pitch_GET() == (short) 12191);
			assert (pack.battery_remaining_GET() == (char) 30);
			assert (pack.base_mode_GET() == MAV_MODE_FLAG.MAV_MODE_FLAG_HIL_ENABLED);
			assert (pack.landed_state_GET() == MAV_LANDED_STATE.MAV_LANDED_STATE_LANDING);
			assert (pack.custom_mode_GET() == 421879690L);
		});
		GroundControl.HIGH_LATENCY p234 = CommunicationChannel.new_HIGH_LATENCY();
		PH.setPack(p234);
		p234.base_mode_SET(MAV_MODE_FLAG.MAV_MODE_FLAG_HIL_ENABLED);
		p234.airspeed_SET((char) 165);
		p234.battery_remaining_SET((char) 30);
		p234.altitude_amsl_SET((short) -6041);
		p234.heading_SET((char) 44998);
		p234.latitude_SET(-617179032);
		p234.pitch_SET((short) 12191);
		p234.gps_fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_PPP);
		p234.gps_nsat_SET((char) 18);
		p234.airspeed_sp_SET((char) 176);
		p234.temperature_SET((byte) -46);
		p234.landed_state_SET(MAV_LANDED_STATE.MAV_LANDED_STATE_LANDING);
		p234.throttle_SET((byte) 4);
		p234.custom_mode_SET(421879690L);
		p234.heading_sp_SET((short) 20423);
		p234.wp_distance_SET((char) 1469);
		p234.climb_rate_SET((byte) -119);
		p234.failsafe_SET((char) 51);
		p234.wp_num_SET((char) 13);
		p234.groundspeed_SET((char) 194);
		p234.roll_SET((short) -30702);
		p234.temperature_air_SET((byte) 3);
		p234.altitude_sp_SET((short) 2120);
		p234.longitude_SET(-1870098575);
		CommunicationChannel.instance.send(p234);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_VIBRATION.add((src, ph, pack) ->
		{
			assert (pack.clipping_2_GET() == 3508981746L);
			assert (pack.time_usec_GET() == 4868815325823334697L);
			assert (pack.vibration_z_GET() == -1.2006265E38F);
			assert (pack.vibration_x_GET() == 9.865413E37F);
			assert (pack.clipping_0_GET() == 1086948003L);
			assert (pack.vibration_y_GET() == -1.5641512E37F);
			assert (pack.clipping_1_GET() == 479827114L);
		});
		GroundControl.VIBRATION p241 = CommunicationChannel.new_VIBRATION();
		PH.setPack(p241);
		p241.clipping_2_SET(3508981746L);
		p241.clipping_0_SET(1086948003L);
		p241.vibration_z_SET(-1.2006265E38F);
		p241.vibration_y_SET(-1.5641512E37F);
		p241.vibration_x_SET(9.865413E37F);
		p241.clipping_1_SET(479827114L);
		p241.time_usec_SET(4868815325823334697L);
		CommunicationChannel.instance.send(p241);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_HOME_POSITION.add((src, ph, pack) ->
		{
			assert (pack.y_GET() == 3.1823876E37F);
			assert (pack.approach_z_GET() == -2.0415919E38F);
			assert (pack.longitude_GET() == 683904612);
			assert (pack.approach_x_GET() == -1.8045534E37F);
			assert (pack.time_usec_TRY(ph) == 7366708450486028832L);
			assert (pack.altitude_GET() == -1600164186);
			assert (pack.x_GET() == -2.2058917E38F);
			assert (pack.z_GET() == -2.209651E38F);
			assert (pack.approach_y_GET() == -3.3612863E38F);
			assert (Arrays.equals(pack.q_GET(), new float[]{7.217578E37F, 2.8164923E38F, 1.0076199E38F, -2.5782784E38F}));
			assert (pack.latitude_GET() == 398436776);
		});
		GroundControl.HOME_POSITION p242 = CommunicationChannel.new_HOME_POSITION();
		PH.setPack(p242);
		p242.time_usec_SET(7366708450486028832L, PH);
		p242.altitude_SET(-1600164186);
		p242.approach_y_SET(-3.3612863E38F);
		p242.approach_z_SET(-2.0415919E38F);
		p242.longitude_SET(683904612);
		p242.q_SET(new float[]{7.217578E37F, 2.8164923E38F, 1.0076199E38F, -2.5782784E38F}, 0);
		p242.z_SET(-2.209651E38F);
		p242.latitude_SET(398436776);
		p242.x_SET(-2.2058917E38F);
		p242.approach_x_SET(-1.8045534E37F);
		p242.y_SET(3.1823876E37F);
		CommunicationChannel.instance.send(p242);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_SET_HOME_POSITION.add((src, ph, pack) ->
		{
			assert (pack.time_usec_TRY(ph) == 2346919243541565490L);
			assert (pack.longitude_GET() == -462482684);
			assert (pack.y_GET() == 1.1092583E37F);
			assert (pack.approach_x_GET() == 2.0415207E38F);
			assert (pack.target_system_GET() == (char) 74);
			assert (pack.x_GET() == -1.7220559E38F);
			assert (pack.altitude_GET() == -1040733500);
			assert (pack.z_GET() == 1.8569952E38F);
			assert (Arrays.equals(pack.q_GET(), new float[]{-1.5940642E38F, 1.7096202E37F, -1.8985652E38F, 2.1919888E38F}));
			assert (pack.latitude_GET() == -21305488);
			assert (pack.approach_y_GET() == -2.534226E38F);
			assert (pack.approach_z_GET() == 1.0190582E38F);
		});
		GroundControl.SET_HOME_POSITION p243 = CommunicationChannel.new_SET_HOME_POSITION();
		PH.setPack(p243);
		p243.y_SET(1.1092583E37F);
		p243.x_SET(-1.7220559E38F);
		p243.approach_z_SET(1.0190582E38F);
		p243.approach_x_SET(2.0415207E38F);
		p243.latitude_SET(-21305488);
		p243.q_SET(new float[]{-1.5940642E38F, 1.7096202E37F, -1.8985652E38F, 2.1919888E38F}, 0);
		p243.altitude_SET(-1040733500);
		p243.approach_y_SET(-2.534226E38F);
		p243.z_SET(1.8569952E38F);
		p243.target_system_SET((char) 74);
		p243.longitude_SET(-462482684);
		p243.time_usec_SET(2346919243541565490L, PH);
		CommunicationChannel.instance.send(p243);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_MESSAGE_INTERVAL.add((src, ph, pack) ->
		{
			assert (pack.interval_us_GET() == -412239536);
			assert (pack.message_id_GET() == (char) 24484);
		});
		GroundControl.MESSAGE_INTERVAL p244 = CommunicationChannel.new_MESSAGE_INTERVAL();
		PH.setPack(p244);
		p244.interval_us_SET(-412239536);
		p244.message_id_SET((char) 24484);
		CommunicationChannel.instance.send(p244);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_EXTENDED_SYS_STATE.add((src, ph, pack) ->
		{
			assert (pack.vtol_state_GET() == MAV_VTOL_STATE.MAV_VTOL_STATE_TRANSITION_TO_MC);
			assert (pack.landed_state_GET() == MAV_LANDED_STATE.MAV_LANDED_STATE_ON_GROUND);
		});
		GroundControl.EXTENDED_SYS_STATE p245 = CommunicationChannel.new_EXTENDED_SYS_STATE();
		PH.setPack(p245);
		p245.landed_state_SET(MAV_LANDED_STATE.MAV_LANDED_STATE_ON_GROUND);
		p245.vtol_state_SET(MAV_VTOL_STATE.MAV_VTOL_STATE_TRANSITION_TO_MC);
		CommunicationChannel.instance.send(p245);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_ADSB_VEHICLE.add((src, ph, pack) ->
		{
			assert (pack.heading_GET() == (char) 32731);
			assert (pack.emitter_type_GET() == ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_UAV);
			assert (pack.altitude_type_GET() == ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_GEOMETRIC);
			assert (pack.ver_velocity_GET() == (short) -18347);
			assert (pack.callsign_LEN(ph) == 2);
			assert (pack.callsign_TRY(ph).equals("yc"));
			assert (pack.tslc_GET() == (char) 49);
			assert (pack.ICAO_address_GET() == 1215561741L);
			assert (pack.squawk_GET() == (char) 53921);
			assert (pack.lon_GET() == -1235004528);
			assert (pack.hor_velocity_GET() == (char) 47796);
			assert (pack.lat_GET() == 1498135858);
			assert (pack.flags_GET() == ADSB_FLAGS.ADSB_FLAGS_VALID_VELOCITY);
			assert (pack.altitude_GET() == -1138741197);
		});
		GroundControl.ADSB_VEHICLE p246 = CommunicationChannel.new_ADSB_VEHICLE();
		PH.setPack(p246);
		p246.ICAO_address_SET(1215561741L);
		p246.tslc_SET((char) 49);
		p246.emitter_type_SET(ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_UAV);
		p246.altitude_SET(-1138741197);
		p246.lon_SET(-1235004528);
		p246.heading_SET((char) 32731);
		p246.ver_velocity_SET((short) -18347);
		p246.callsign_SET("yc", PH);
		p246.flags_SET(ADSB_FLAGS.ADSB_FLAGS_VALID_VELOCITY);
		p246.squawk_SET((char) 53921);
		p246.altitude_type_SET(ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_GEOMETRIC);
		p246.lat_SET(1498135858);
		p246.hor_velocity_SET((char) 47796);
		CommunicationChannel.instance.send(p246);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_COLLISION.add((src, ph, pack) ->
		{
			assert (pack.threat_level_GET() == MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_HIGH);
			assert (pack.action_GET() == MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_MOVE_PERPENDICULAR);
			assert (pack.id_GET() == 124537419L);
			assert (pack.time_to_minimum_delta_GET() == -2.1875778E37F);
			assert (pack.src__GET() == MAV_COLLISION_SRC.MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT);
			assert (pack.horizontal_minimum_delta_GET() == -1.1512732E38F);
			assert (pack.altitude_minimum_delta_GET() == 1.3726422E38F);
		});
		GroundControl.COLLISION p247 = CommunicationChannel.new_COLLISION();
		PH.setPack(p247);
		p247.altitude_minimum_delta_SET(1.3726422E38F);
		p247.src__SET(MAV_COLLISION_SRC.MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT);
		p247.time_to_minimum_delta_SET(-2.1875778E37F);
		p247.threat_level_SET(MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_HIGH);
		p247.horizontal_minimum_delta_SET(-1.1512732E38F);
		p247.action_SET(MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_MOVE_PERPENDICULAR);
		p247.id_SET(124537419L);
		CommunicationChannel.instance.send(p247);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_V2_EXTENSION.add((src, ph, pack) ->
		{
			assert (pack.message_type_GET() == (char) 20915);
			assert (pack.target_component_GET() == (char) 236);
			assert (pack.target_network_GET() == (char) 116);
			assert (pack.target_system_GET() == (char) 13);
			assert (Arrays.equals(pack.payload_GET(), new char[]{(char) 19, (char) 98, (char) 53, (char) 208, (char) 173, (char) 11, (char) 191, (char) 182, (char) 145, (char) 46, (char) 44, (char) 246, (char) 150, (char) 250, (char) 39, (char) 88, (char) 238, (char) 24, (char) 144, (char) 209, (char) 1, (char) 153, (char) 248, (char) 252, (char) 242, (char) 84, (char) 149, (char) 176, (char) 57, (char) 233, (char) 195, (char) 65, (char) 112, (char) 87, (char) 150, (char) 86, (char) 91, (char) 224, (char) 60, (char) 97, (char) 32, (char) 106, (char) 26, (char) 34, (char) 60, (char) 174, (char) 19, (char) 137, (char) 250, (char) 133, (char) 35, (char) 237, (char) 180, (char) 184, (char) 182, (char) 175, (char) 213, (char) 47, (char) 129, (char) 246, (char) 35, (char) 164, (char) 173, (char) 103, (char) 66, (char) 124, (char) 234, (char) 243, (char) 215, (char) 35, (char) 88, (char) 16, (char) 47, (char) 176, (char) 109, (char) 33, (char) 29, (char) 4, (char) 100, (char) 1, (char) 168, (char) 25, (char) 159, (char) 11, (char) 15, (char) 63, (char) 189, (char) 153, (char) 9, (char) 92, (char) 68, (char) 212, (char) 61, (char) 72, (char) 225, (char) 110, (char) 47, (char) 191, (char) 220, (char) 106, (char) 232, (char) 168, (char) 54, (char) 68, (char) 17, (char) 208, (char) 245, (char) 151, (char) 212, (char) 134, (char) 184, (char) 170, (char) 72, (char) 192, (char) 21, (char) 27, (char) 17, (char) 142, (char) 63, (char) 92, (char) 84, (char) 120, (char) 236, (char) 109, (char) 97, (char) 110, (char) 186, (char) 21, (char) 158, (char) 195, (char) 12, (char) 6, (char) 104, (char) 179, (char) 106, (char) 35, (char) 89, (char) 147, (char) 70, (char) 62, (char) 96, (char) 162, (char) 208, (char) 145, (char) 72, (char) 203, (char) 40, (char) 61, (char) 111, (char) 30, (char) 156, (char) 23, (char) 57, (char) 33, (char) 126, (char) 197, (char) 77, (char) 230, (char) 242, (char) 80, (char) 100, (char) 176, (char) 242, (char) 13, (char) 19, (char) 97, (char) 42, (char) 197, (char) 67, (char) 17, (char) 36, (char) 107, (char) 165, (char) 198, (char) 20, (char) 146, (char) 114, (char) 35, (char) 2, (char) 36, (char) 147, (char) 249, (char) 58, (char) 90, (char) 36, (char) 151, (char) 60, (char) 245, (char) 144, (char) 134, (char) 13, (char) 117, (char) 87, (char) 175, (char) 7, (char) 184, (char) 33, (char) 211, (char) 119, (char) 175, (char) 216, (char) 110, (char) 45, (char) 216, (char) 208, (char) 31, (char) 12, (char) 253, (char) 122, (char) 246, (char) 216, (char) 10, (char) 59, (char) 197, (char) 47, (char) 211, (char) 68, (char) 163, (char) 147, (char) 110, (char) 82, (char) 128, (char) 182, (char) 150, (char) 123, (char) 192, (char) 136, (char) 71, (char) 125, (char) 235, (char) 180, (char) 217, (char) 223, (char) 16, (char) 101, (char) 2, (char) 214, (char) 192, (char) 10, (char) 129, (char) 204, (char) 143, (char) 96, (char) 240, (char) 185, (char) 128, (char) 2, (char) 30, (char) 120}));
		});
		GroundControl.V2_EXTENSION p248 = CommunicationChannel.new_V2_EXTENSION();
		PH.setPack(p248);
		p248.payload_SET(new char[]{(char) 19, (char) 98, (char) 53, (char) 208, (char) 173, (char) 11, (char) 191, (char) 182, (char) 145, (char) 46, (char) 44, (char) 246, (char) 150, (char) 250, (char) 39, (char) 88, (char) 238, (char) 24, (char) 144, (char) 209, (char) 1, (char) 153, (char) 248, (char) 252, (char) 242, (char) 84, (char) 149, (char) 176, (char) 57, (char) 233, (char) 195, (char) 65, (char) 112, (char) 87, (char) 150, (char) 86, (char) 91, (char) 224, (char) 60, (char) 97, (char) 32, (char) 106, (char) 26, (char) 34, (char) 60, (char) 174, (char) 19, (char) 137, (char) 250, (char) 133, (char) 35, (char) 237, (char) 180, (char) 184, (char) 182, (char) 175, (char) 213, (char) 47, (char) 129, (char) 246, (char) 35, (char) 164, (char) 173, (char) 103, (char) 66, (char) 124, (char) 234, (char) 243, (char) 215, (char) 35, (char) 88, (char) 16, (char) 47, (char) 176, (char) 109, (char) 33, (char) 29, (char) 4, (char) 100, (char) 1, (char) 168, (char) 25, (char) 159, (char) 11, (char) 15, (char) 63, (char) 189, (char) 153, (char) 9, (char) 92, (char) 68, (char) 212, (char) 61, (char) 72, (char) 225, (char) 110, (char) 47, (char) 191, (char) 220, (char) 106, (char) 232, (char) 168, (char) 54, (char) 68, (char) 17, (char) 208, (char) 245, (char) 151, (char) 212, (char) 134, (char) 184, (char) 170, (char) 72, (char) 192, (char) 21, (char) 27, (char) 17, (char) 142, (char) 63, (char) 92, (char) 84, (char) 120, (char) 236, (char) 109, (char) 97, (char) 110, (char) 186, (char) 21, (char) 158, (char) 195, (char) 12, (char) 6, (char) 104, (char) 179, (char) 106, (char) 35, (char) 89, (char) 147, (char) 70, (char) 62, (char) 96, (char) 162, (char) 208, (char) 145, (char) 72, (char) 203, (char) 40, (char) 61, (char) 111, (char) 30, (char) 156, (char) 23, (char) 57, (char) 33, (char) 126, (char) 197, (char) 77, (char) 230, (char) 242, (char) 80, (char) 100, (char) 176, (char) 242, (char) 13, (char) 19, (char) 97, (char) 42, (char) 197, (char) 67, (char) 17, (char) 36, (char) 107, (char) 165, (char) 198, (char) 20, (char) 146, (char) 114, (char) 35, (char) 2, (char) 36, (char) 147, (char) 249, (char) 58, (char) 90, (char) 36, (char) 151, (char) 60, (char) 245, (char) 144, (char) 134, (char) 13, (char) 117, (char) 87, (char) 175, (char) 7, (char) 184, (char) 33, (char) 211, (char) 119, (char) 175, (char) 216, (char) 110, (char) 45, (char) 216, (char) 208, (char) 31, (char) 12, (char) 253, (char) 122, (char) 246, (char) 216, (char) 10, (char) 59, (char) 197, (char) 47, (char) 211, (char) 68, (char) 163, (char) 147, (char) 110, (char) 82, (char) 128, (char) 182, (char) 150, (char) 123, (char) 192, (char) 136, (char) 71, (char) 125, (char) 235, (char) 180, (char) 217, (char) 223, (char) 16, (char) 101, (char) 2, (char) 214, (char) 192, (char) 10, (char) 129, (char) 204, (char) 143, (char) 96, (char) 240, (char) 185, (char) 128, (char) 2, (char) 30, (char) 120}, 0);
		p248.target_component_SET((char) 236);
		p248.target_system_SET((char) 13);
		p248.message_type_SET((char) 20915);
		p248.target_network_SET((char) 116);
		CommunicationChannel.instance.send(p248);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_MEMORY_VECT.add((src, ph, pack) ->
		{
			assert (pack.address_GET() == (char) 28785);
			assert (Arrays.equals(pack.value_GET(), new byte[]{(byte) -35, (byte) 105, (byte) 114, (byte) 80, (byte) -28, (byte) 14, (byte) -54, (byte) -102, (byte) 50, (byte) -8, (byte) 59, (byte) -13, (byte) 26, (byte) -4, (byte) 4, (byte) -123, (byte) 93, (byte) -118, (byte) -106, (byte) -18, (byte) 107, (byte) -37, (byte) 2, (byte) 40, (byte) -72, (byte) -26, (byte) 11, (byte) 109, (byte) 109, (byte) 68, (byte) -10, (byte) 93}));
			assert (pack.type_GET() == (char) 215);
			assert (pack.ver_GET() == (char) 213);
		});
		GroundControl.MEMORY_VECT p249 = CommunicationChannel.new_MEMORY_VECT();
		PH.setPack(p249);
		p249.address_SET((char) 28785);
		p249.ver_SET((char) 213);
		p249.value_SET(new byte[]{(byte) -35, (byte) 105, (byte) 114, (byte) 80, (byte) -28, (byte) 14, (byte) -54, (byte) -102, (byte) 50, (byte) -8, (byte) 59, (byte) -13, (byte) 26, (byte) -4, (byte) 4, (byte) -123, (byte) 93, (byte) -118, (byte) -106, (byte) -18, (byte) 107, (byte) -37, (byte) 2, (byte) 40, (byte) -72, (byte) -26, (byte) 11, (byte) 109, (byte) 109, (byte) 68, (byte) -10, (byte) 93}, 0);
		p249.type_SET((char) 215);
		CommunicationChannel.instance.send(p249);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_DEBUG_VECT.add((src, ph, pack) ->
		{
			assert (pack.y_GET() == -3.31027E38F);
			assert (pack.time_usec_GET() == 2732840141229100959L);
			assert (pack.name_LEN(ph) == 3);
			assert (pack.name_TRY(ph).equals("zvi"));
			assert (pack.x_GET() == 2.8473797E38F);
			assert (pack.z_GET() == 2.1711954E37F);
		});
		GroundControl.DEBUG_VECT p250 = CommunicationChannel.new_DEBUG_VECT();
		PH.setPack(p250);
		p250.name_SET("zvi", PH);
		p250.z_SET(2.1711954E37F);
		p250.time_usec_SET(2732840141229100959L);
		p250.x_SET(2.8473797E38F);
		p250.y_SET(-3.31027E38F);
		CommunicationChannel.instance.send(p250);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_NAMED_VALUE_FLOAT.add((src, ph, pack) ->
		{
			assert (pack.value_GET() == -1.2373187E38F);
			assert (pack.time_boot_ms_GET() == 3776736016L);
			assert (pack.name_LEN(ph) == 10);
			assert (pack.name_TRY(ph).equals("hajoTuawfj"));
		});
		GroundControl.NAMED_VALUE_FLOAT p251 = CommunicationChannel.new_NAMED_VALUE_FLOAT();
		PH.setPack(p251);
		p251.value_SET(-1.2373187E38F);
		p251.name_SET("hajoTuawfj", PH);
		p251.time_boot_ms_SET(3776736016L);
		CommunicationChannel.instance.send(p251);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_NAMED_VALUE_INT.add((src, ph, pack) ->
		{
			assert (pack.time_boot_ms_GET() == 1616910490L);
			assert (pack.name_LEN(ph) == 6);
			assert (pack.name_TRY(ph).equals("eprhEl"));
			assert (pack.value_GET() == 662284135);
		});
		GroundControl.NAMED_VALUE_INT p252 = CommunicationChannel.new_NAMED_VALUE_INT();
		PH.setPack(p252);
		p252.time_boot_ms_SET(1616910490L);
		p252.value_SET(662284135);
		p252.name_SET("eprhEl", PH);
		CommunicationChannel.instance.send(p252);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_STATUSTEXT.add((src, ph, pack) ->
		{
			assert (pack.severity_GET() == MAV_SEVERITY.MAV_SEVERITY_INFO);
			assert (pack.text_LEN(ph) == 12);
			assert (pack.text_TRY(ph).equals("VxrpsAxkvtng"));
		});
		GroundControl.STATUSTEXT p253 = CommunicationChannel.new_STATUSTEXT();
		PH.setPack(p253);
		p253.text_SET("VxrpsAxkvtng", PH);
		p253.severity_SET(MAV_SEVERITY.MAV_SEVERITY_INFO);
		CommunicationChannel.instance.send(p253);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_DEBUG.add((src, ph, pack) ->
		{
			assert (pack.time_boot_ms_GET() == 299132448L);
			assert (pack.ind_GET() == (char) 131);
			assert (pack.value_GET() == 6.19918E36F);
		});
		GroundControl.DEBUG p254 = CommunicationChannel.new_DEBUG();
		PH.setPack(p254);
		p254.value_SET(6.19918E36F);
		p254.ind_SET((char) 131);
		p254.time_boot_ms_SET(299132448L);
		CommunicationChannel.instance.send(p254);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_SETUP_SIGNING.add((src, ph, pack) ->
		{
			assert (pack.initial_timestamp_GET() == 772082084864616042L);
			assert (pack.target_system_GET() == (char) 252);
			assert (Arrays.equals(pack.secret_key_GET(), new char[]{(char) 189, (char) 79, (char) 145, (char) 225, (char) 20, (char) 222, (char) 104, (char) 252, (char) 208, (char) 236, (char) 157, (char) 234, (char) 149, (char) 214, (char) 142, (char) 24, (char) 241, (char) 197, (char) 70, (char) 62, (char) 220, (char) 189, (char) 70, (char) 237, (char) 69, (char) 27, (char) 211, (char) 242, (char) 79, (char) 177, (char) 253, (char) 154}));
			assert (pack.target_component_GET() == (char) 66);
		});
		GroundControl.SETUP_SIGNING p256 = CommunicationChannel.new_SETUP_SIGNING();
		PH.setPack(p256);
		p256.target_system_SET((char) 252);
		p256.initial_timestamp_SET(772082084864616042L);
		p256.target_component_SET((char) 66);
		p256.secret_key_SET(new char[]{(char) 189, (char) 79, (char) 145, (char) 225, (char) 20, (char) 222, (char) 104, (char) 252, (char) 208, (char) 236, (char) 157, (char) 234, (char) 149, (char) 214, (char) 142, (char) 24, (char) 241, (char) 197, (char) 70, (char) 62, (char) 220, (char) 189, (char) 70, (char) 237, (char) 69, (char) 27, (char) 211, (char) 242, (char) 79, (char) 177, (char) 253, (char) 154}, 0);
		CommunicationChannel.instance.send(p256);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_BUTTON_CHANGE.add((src, ph, pack) ->
		{
			assert (pack.state_GET() == (char) 71);
			assert (pack.last_change_ms_GET() == 789795988L);
			assert (pack.time_boot_ms_GET() == 622077870L);
		});
		GroundControl.BUTTON_CHANGE p257 = CommunicationChannel.new_BUTTON_CHANGE();
		PH.setPack(p257);
		p257.last_change_ms_SET(789795988L);
		p257.time_boot_ms_SET(622077870L);
		p257.state_SET((char) 71);
		CommunicationChannel.instance.send(p257);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_PLAY_TUNE.add((src, ph, pack) ->
		{
			assert (pack.target_component_GET() == (char) 157);
			assert (pack.target_system_GET() == (char) 138);
			assert (pack.tune_LEN(ph) == 3);
			assert (pack.tune_TRY(ph).equals("xWN"));
		});
		GroundControl.PLAY_TUNE p258 = CommunicationChannel.new_PLAY_TUNE();
		PH.setPack(p258);
		p258.tune_SET("xWN", PH);
		p258.target_component_SET((char) 157);
		p258.target_system_SET((char) 138);
		CommunicationChannel.instance.send(p258);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_CAMERA_INFORMATION.add((src, ph, pack) ->
		{
			assert (Arrays.equals(pack.model_name_GET(), new char[]{(char) 60, (char) 222, (char) 171, (char) 42, (char) 223, (char) 123, (char) 224, (char) 65, (char) 153, (char) 27, (char) 98, (char) 241, (char) 56, (char) 49, (char) 93, (char) 42, (char) 163, (char) 124, (char) 94, (char) 91, (char) 123, (char) 255, (char) 106, (char) 181, (char) 151, (char) 126, (char) 193, (char) 102, (char) 98, (char) 154, (char) 22, (char) 26}));
			assert (pack.flags_GET() == CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE);
			assert (pack.lens_id_GET() == (char) 158);
			assert (pack.time_boot_ms_GET() == 4197653184L);
			assert (pack.sensor_size_h_GET() == 3.0005543E38F);
			assert (pack.cam_definition_uri_LEN(ph) == 38);
			assert (pack.cam_definition_uri_TRY(ph).equals("xdqmfjrrDwYbehgsbaoeanjwbvhpdgaikmKexc"));
			assert (pack.cam_definition_version_GET() == (char) 18369);
			assert (Arrays.equals(pack.vendor_name_GET(), new char[]{(char) 153, (char) 191, (char) 233, (char) 252, (char) 94, (char) 79, (char) 79, (char) 35, (char) 186, (char) 18, (char) 148, (char) 38, (char) 59, (char) 195, (char) 153, (char) 71, (char) 86, (char) 178, (char) 98, (char) 113, (char) 60, (char) 8, (char) 22, (char) 57, (char) 99, (char) 63, (char) 181, (char) 151, (char) 250, (char) 32, (char) 9, (char) 12}));
			assert (pack.resolution_h_GET() == (char) 21397);
			assert (pack.sensor_size_v_GET() == 3.1086863E38F);
			assert (pack.focal_length_GET() == 2.1310752E38F);
			assert (pack.firmware_version_GET() == 3907072471L);
			assert (pack.resolution_v_GET() == (char) 65124);
		});
		GroundControl.CAMERA_INFORMATION p259 = CommunicationChannel.new_CAMERA_INFORMATION();
		PH.setPack(p259);
		p259.focal_length_SET(2.1310752E38F);
		p259.model_name_SET(new char[]{(char) 60, (char) 222, (char) 171, (char) 42, (char) 223, (char) 123, (char) 224, (char) 65, (char) 153, (char) 27, (char) 98, (char) 241, (char) 56, (char) 49, (char) 93, (char) 42, (char) 163, (char) 124, (char) 94, (char) 91, (char) 123, (char) 255, (char) 106, (char) 181, (char) 151, (char) 126, (char) 193, (char) 102, (char) 98, (char) 154, (char) 22, (char) 26}, 0);
		p259.flags_SET(CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE);
		p259.cam_definition_version_SET((char) 18369);
		p259.cam_definition_uri_SET("xdqmfjrrDwYbehgsbaoeanjwbvhpdgaikmKexc", PH);
		p259.sensor_size_h_SET(3.0005543E38F);
		p259.resolution_h_SET((char) 21397);
		p259.sensor_size_v_SET(3.1086863E38F);
		p259.vendor_name_SET(new char[]{(char) 153, (char) 191, (char) 233, (char) 252, (char) 94, (char) 79, (char) 79, (char) 35, (char) 186, (char) 18, (char) 148, (char) 38, (char) 59, (char) 195, (char) 153, (char) 71, (char) 86, (char) 178, (char) 98, (char) 113, (char) 60, (char) 8, (char) 22, (char) 57, (char) 99, (char) 63, (char) 181, (char) 151, (char) 250, (char) 32, (char) 9, (char) 12}, 0);
		p259.time_boot_ms_SET(4197653184L);
		p259.firmware_version_SET(3907072471L);
		p259.lens_id_SET((char) 158);
		p259.resolution_v_SET((char) 65124);
		CommunicationChannel.instance.send(p259);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_CAMERA_SETTINGS.add((src, ph, pack) ->
		{
			assert (pack.time_boot_ms_GET() == 3018197943L);
			assert (pack.mode_id_GET() == CAMERA_MODE.CAMERA_MODE_IMAGE_SURVEY);
		});
		GroundControl.CAMERA_SETTINGS p260 = CommunicationChannel.new_CAMERA_SETTINGS();
		PH.setPack(p260);
		p260.mode_id_SET(CAMERA_MODE.CAMERA_MODE_IMAGE_SURVEY);
		p260.time_boot_ms_SET(3018197943L);
		CommunicationChannel.instance.send(p260);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_STORAGE_INFORMATION.add((src, ph, pack) ->
		{
			assert (pack.available_capacity_GET() == -3.2355834E38F);
			assert (pack.status_GET() == (char) 210);
			assert (pack.storage_id_GET() == (char) 98);
			assert (pack.read_speed_GET() == 2.5701297E38F);
			assert (pack.total_capacity_GET() == -2.8269345E38F);
			assert (pack.time_boot_ms_GET() == 1890208295L);
			assert (pack.storage_count_GET() == (char) 252);
			assert (pack.write_speed_GET() == -3.3241807E37F);
			assert (pack.used_capacity_GET() == -3.2261602E38F);
		});
		GroundControl.STORAGE_INFORMATION p261 = CommunicationChannel.new_STORAGE_INFORMATION();
		PH.setPack(p261);
		p261.write_speed_SET(-3.3241807E37F);
		p261.time_boot_ms_SET(1890208295L);
		p261.storage_id_SET((char) 98);
		p261.total_capacity_SET(-2.8269345E38F);
		p261.read_speed_SET(2.5701297E38F);
		p261.available_capacity_SET(-3.2355834E38F);
		p261.used_capacity_SET(-3.2261602E38F);
		p261.status_SET((char) 210);
		p261.storage_count_SET((char) 252);
		CommunicationChannel.instance.send(p261);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_CAMERA_CAPTURE_STATUS.add((src, ph, pack) ->
		{
			assert (pack.video_status_GET() == (char) 23);
			assert (pack.time_boot_ms_GET() == 2413399685L);
			assert (pack.available_capacity_GET() == 3.3619471E38F);
			assert (pack.image_interval_GET() == 1.4286755E38F);
			assert (pack.recording_time_ms_GET() == 1356844432L);
			assert (pack.image_status_GET() == (char) 213);
		});
		GroundControl.CAMERA_CAPTURE_STATUS p262 = CommunicationChannel.new_CAMERA_CAPTURE_STATUS();
		PH.setPack(p262);
		p262.video_status_SET((char) 23);
		p262.image_status_SET((char) 213);
		p262.recording_time_ms_SET(1356844432L);
		p262.image_interval_SET(1.4286755E38F);
		p262.available_capacity_SET(3.3619471E38F);
		p262.time_boot_ms_SET(2413399685L);
		CommunicationChannel.instance.send(p262);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_CAMERA_IMAGE_CAPTURED.add((src, ph, pack) ->
		{
			assert (pack.lon_GET() == 488232221);
			assert (pack.camera_id_GET() == (char) 0);
			assert (pack.time_boot_ms_GET() == 886798304L);
			assert (pack.time_utc_GET() == 5694736819847912609L);
			assert (pack.alt_GET() == 778166862);
			assert (pack.relative_alt_GET() == -360245474);
			assert (Arrays.equals(pack.q_GET(), new float[]{-8.984212E37F, 9.759853E37F, 1.8561332E38F, 1.3759035E37F}));
			assert (pack.image_index_GET() == 927539871);
			assert (pack.file_url_LEN(ph) == 131);
			assert (pack.file_url_TRY(ph).equals("dxfkvplIjvdwmPdEtyRnezunvketynkotflsOoePhmFMsHsutnatotbbhtpgjrwWulsfXatycvruhlfWyoikjjidaqmrnLcxwuvkydutbolagpmLkGwgtyYrhWmptlyjuiy"));
			assert (pack.lat_GET() == 187566281);
			assert (pack.capture_result_GET() == (byte) 18);
		});
		GroundControl.CAMERA_IMAGE_CAPTURED p263 = CommunicationChannel.new_CAMERA_IMAGE_CAPTURED();
		PH.setPack(p263);
		p263.camera_id_SET((char) 0);
		p263.time_utc_SET(5694736819847912609L);
		p263.lon_SET(488232221);
		p263.time_boot_ms_SET(886798304L);
		p263.capture_result_SET((byte) 18);
		p263.relative_alt_SET(-360245474);
		p263.alt_SET(778166862);
		p263.image_index_SET(927539871);
		p263.lat_SET(187566281);
		p263.q_SET(new float[]{-8.984212E37F, 9.759853E37F, 1.8561332E38F, 1.3759035E37F}, 0);
		p263.file_url_SET("dxfkvplIjvdwmPdEtyRnezunvketynkotflsOoePhmFMsHsutnatotbbhtpgjrwWulsfXatycvruhlfWyoikjjidaqmrnLcxwuvkydutbolagpmLkGwgtyYrhWmptlyjuiy", PH);
		CommunicationChannel.instance.send(p263);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_FLIGHT_INFORMATION.add((src, ph, pack) ->
		{
			assert (pack.time_boot_ms_GET() == 1289333730L);
			assert (pack.flight_uuid_GET() == 2342380508713287507L);
			assert (pack.arming_time_utc_GET() == 7643658172227126392L);
			assert (pack.takeoff_time_utc_GET() == 7384831348747272833L);
		});
		GroundControl.FLIGHT_INFORMATION p264 = CommunicationChannel.new_FLIGHT_INFORMATION();
		PH.setPack(p264);
		p264.arming_time_utc_SET(7643658172227126392L);
		p264.flight_uuid_SET(2342380508713287507L);
		p264.time_boot_ms_SET(1289333730L);
		p264.takeoff_time_utc_SET(7384831348747272833L);
		CommunicationChannel.instance.send(p264);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_MOUNT_ORIENTATION.add((src, ph, pack) ->
		{
			assert (pack.time_boot_ms_GET() == 2149680961L);
			assert (pack.yaw_GET() == 6.037344E37F);
			assert (pack.roll_GET() == 1.2106001E38F);
			assert (pack.pitch_GET() == 4.8335964E37F);
		});
		GroundControl.MOUNT_ORIENTATION p265 = CommunicationChannel.new_MOUNT_ORIENTATION();
		PH.setPack(p265);
		p265.roll_SET(1.2106001E38F);
		p265.pitch_SET(4.8335964E37F);
		p265.yaw_SET(6.037344E37F);
		p265.time_boot_ms_SET(2149680961L);
		CommunicationChannel.instance.send(p265);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_LOGGING_DATA.add((src, ph, pack) ->
		{
			assert (pack.sequence_GET() == (char) 46609);
			assert (Arrays.equals(pack.data__GET(), new char[]{(char) 1, (char) 23, (char) 226, (char) 138, (char) 105, (char) 222, (char) 250, (char) 46, (char) 179, (char) 215, (char) 246, (char) 6, (char) 153, (char) 185, (char) 211, (char) 109, (char) 248, (char) 200, (char) 203, (char) 21, (char) 69, (char) 78, (char) 64, (char) 141, (char) 6, (char) 214, (char) 84, (char) 241, (char) 241, (char) 24, (char) 221, (char) 140, (char) 11, (char) 206, (char) 248, (char) 43, (char) 28, (char) 150, (char) 14, (char) 193, (char) 103, (char) 205, (char) 227, (char) 154, (char) 83, (char) 137, (char) 154, (char) 23, (char) 49, (char) 89, (char) 193, (char) 6, (char) 58, (char) 153, (char) 59, (char) 142, (char) 64, (char) 159, (char) 113, (char) 161, (char) 44, (char) 171, (char) 163, (char) 190, (char) 118, (char) 171, (char) 74, (char) 181, (char) 200, (char) 110, (char) 75, (char) 2, (char) 75, (char) 167, (char) 70, (char) 41, (char) 113, (char) 174, (char) 198, (char) 190, (char) 187, (char) 191, (char) 217, (char) 251, (char) 103, (char) 52, (char) 51, (char) 37, (char) 209, (char) 244, (char) 67, (char) 13, (char) 30, (char) 39, (char) 138, (char) 250, (char) 38, (char) 63, (char) 61, (char) 28, (char) 77, (char) 67, (char) 74, (char) 233, (char) 192, (char) 243, (char) 38, (char) 201, (char) 230, (char) 39, (char) 246, (char) 19, (char) 76, (char) 135, (char) 157, (char) 204, (char) 133, (char) 100, (char) 27, (char) 204, (char) 21, (char) 83, (char) 43, (char) 252, (char) 184, (char) 167, (char) 200, (char) 124, (char) 69, (char) 61, (char) 113, (char) 136, (char) 185, (char) 69, (char) 155, (char) 178, (char) 42, (char) 154, (char) 51, (char) 234, (char) 46, (char) 122, (char) 67, (char) 245, (char) 147, (char) 254, (char) 146, (char) 57, (char) 126, (char) 187, (char) 115, (char) 210, (char) 68, (char) 20, (char) 129, (char) 105, (char) 59, (char) 132, (char) 145, (char) 207, (char) 98, (char) 200, (char) 116, (char) 195, (char) 199, (char) 72, (char) 250, (char) 74, (char) 86, (char) 208, (char) 174, (char) 174, (char) 199, (char) 231, (char) 159, (char) 219, (char) 130, (char) 16, (char) 134, (char) 98, (char) 182, (char) 200, (char) 245, (char) 20, (char) 221, (char) 234, (char) 232, (char) 134, (char) 102, (char) 148, (char) 239, (char) 153, (char) 166, (char) 185, (char) 80, (char) 47, (char) 60, (char) 45, (char) 162, (char) 209, (char) 15, (char) 228, (char) 38, (char) 7, (char) 45, (char) 78, (char) 30, (char) 228, (char) 36, (char) 130, (char) 121, (char) 190, (char) 86, (char) 82, (char) 108, (char) 159, (char) 76, (char) 57, (char) 33, (char) 24, (char) 73, (char) 247, (char) 206, (char) 238, (char) 236, (char) 211, (char) 157, (char) 120, (char) 182, (char) 217, (char) 122, (char) 48, (char) 111, (char) 142, (char) 227, (char) 72, (char) 121, (char) 207, (char) 214, (char) 91, (char) 109, (char) 62, (char) 150, (char) 150, (char) 213, (char) 219, (char) 243, (char) 132, (char) 176}));
			assert (pack.target_component_GET() == (char) 71);
			assert (pack.first_message_offset_GET() == (char) 4);
			assert (pack.target_system_GET() == (char) 225);
			assert (pack.length_GET() == (char) 121);
		});
		GroundControl.LOGGING_DATA p266 = CommunicationChannel.new_LOGGING_DATA();
		PH.setPack(p266);
		p266.target_system_SET((char) 225);
		p266.first_message_offset_SET((char) 4);
		p266.sequence_SET((char) 46609);
		p266.target_component_SET((char) 71);
		p266.data__SET(new char[]{(char) 1, (char) 23, (char) 226, (char) 138, (char) 105, (char) 222, (char) 250, (char) 46, (char) 179, (char) 215, (char) 246, (char) 6, (char) 153, (char) 185, (char) 211, (char) 109, (char) 248, (char) 200, (char) 203, (char) 21, (char) 69, (char) 78, (char) 64, (char) 141, (char) 6, (char) 214, (char) 84, (char) 241, (char) 241, (char) 24, (char) 221, (char) 140, (char) 11, (char) 206, (char) 248, (char) 43, (char) 28, (char) 150, (char) 14, (char) 193, (char) 103, (char) 205, (char) 227, (char) 154, (char) 83, (char) 137, (char) 154, (char) 23, (char) 49, (char) 89, (char) 193, (char) 6, (char) 58, (char) 153, (char) 59, (char) 142, (char) 64, (char) 159, (char) 113, (char) 161, (char) 44, (char) 171, (char) 163, (char) 190, (char) 118, (char) 171, (char) 74, (char) 181, (char) 200, (char) 110, (char) 75, (char) 2, (char) 75, (char) 167, (char) 70, (char) 41, (char) 113, (char) 174, (char) 198, (char) 190, (char) 187, (char) 191, (char) 217, (char) 251, (char) 103, (char) 52, (char) 51, (char) 37, (char) 209, (char) 244, (char) 67, (char) 13, (char) 30, (char) 39, (char) 138, (char) 250, (char) 38, (char) 63, (char) 61, (char) 28, (char) 77, (char) 67, (char) 74, (char) 233, (char) 192, (char) 243, (char) 38, (char) 201, (char) 230, (char) 39, (char) 246, (char) 19, (char) 76, (char) 135, (char) 157, (char) 204, (char) 133, (char) 100, (char) 27, (char) 204, (char) 21, (char) 83, (char) 43, (char) 252, (char) 184, (char) 167, (char) 200, (char) 124, (char) 69, (char) 61, (char) 113, (char) 136, (char) 185, (char) 69, (char) 155, (char) 178, (char) 42, (char) 154, (char) 51, (char) 234, (char) 46, (char) 122, (char) 67, (char) 245, (char) 147, (char) 254, (char) 146, (char) 57, (char) 126, (char) 187, (char) 115, (char) 210, (char) 68, (char) 20, (char) 129, (char) 105, (char) 59, (char) 132, (char) 145, (char) 207, (char) 98, (char) 200, (char) 116, (char) 195, (char) 199, (char) 72, (char) 250, (char) 74, (char) 86, (char) 208, (char) 174, (char) 174, (char) 199, (char) 231, (char) 159, (char) 219, (char) 130, (char) 16, (char) 134, (char) 98, (char) 182, (char) 200, (char) 245, (char) 20, (char) 221, (char) 234, (char) 232, (char) 134, (char) 102, (char) 148, (char) 239, (char) 153, (char) 166, (char) 185, (char) 80, (char) 47, (char) 60, (char) 45, (char) 162, (char) 209, (char) 15, (char) 228, (char) 38, (char) 7, (char) 45, (char) 78, (char) 30, (char) 228, (char) 36, (char) 130, (char) 121, (char) 190, (char) 86, (char) 82, (char) 108, (char) 159, (char) 76, (char) 57, (char) 33, (char) 24, (char) 73, (char) 247, (char) 206, (char) 238, (char) 236, (char) 211, (char) 157, (char) 120, (char) 182, (char) 217, (char) 122, (char) 48, (char) 111, (char) 142, (char) 227, (char) 72, (char) 121, (char) 207, (char) 214, (char) 91, (char) 109, (char) 62, (char) 150, (char) 150, (char) 213, (char) 219, (char) 243, (char) 132, (char) 176}, 0);
		p266.length_SET((char) 121);
		CommunicationChannel.instance.send(p266);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_LOGGING_DATA_ACKED.add((src, ph, pack) ->
		{
			assert (pack.target_system_GET() == (char) 117);
			assert (pack.sequence_GET() == (char) 16272);
			assert (pack.target_component_GET() == (char) 236);
			assert (Arrays.equals(pack.data__GET(), new char[]{(char) 173, (char) 54, (char) 159, (char) 232, (char) 243, (char) 233, (char) 60, (char) 227, (char) 60, (char) 9, (char) 150, (char) 228, (char) 40, (char) 181, (char) 109, (char) 70, (char) 211, (char) 96, (char) 24, (char) 82, (char) 54, (char) 144, (char) 85, (char) 11, (char) 241, (char) 143, (char) 93, (char) 177, (char) 172, (char) 144, (char) 220, (char) 74, (char) 10, (char) 43, (char) 40, (char) 148, (char) 113, (char) 138, (char) 100, (char) 23, (char) 205, (char) 21, (char) 101, (char) 111, (char) 185, (char) 62, (char) 255, (char) 60, (char) 54, (char) 175, (char) 76, (char) 77, (char) 178, (char) 231, (char) 109, (char) 82, (char) 101, (char) 187, (char) 77, (char) 5, (char) 250, (char) 252, (char) 178, (char) 23, (char) 213, (char) 45, (char) 2, (char) 59, (char) 70, (char) 75, (char) 201, (char) 93, (char) 109, (char) 35, (char) 106, (char) 170, (char) 235, (char) 188, (char) 30, (char) 119, (char) 22, (char) 78, (char) 133, (char) 112, (char) 38, (char) 92, (char) 210, (char) 112, (char) 217, (char) 105, (char) 232, (char) 225, (char) 230, (char) 57, (char) 42, (char) 153, (char) 80, (char) 122, (char) 182, (char) 158, (char) 72, (char) 16, (char) 6, (char) 177, (char) 89, (char) 220, (char) 156, (char) 145, (char) 60, (char) 110, (char) 206, (char) 253, (char) 246, (char) 74, (char) 116, (char) 187, (char) 198, (char) 238, (char) 31, (char) 248, (char) 118, (char) 215, (char) 107, (char) 98, (char) 152, (char) 16, (char) 253, (char) 233, (char) 166, (char) 141, (char) 139, (char) 16, (char) 51, (char) 152, (char) 181, (char) 37, (char) 131, (char) 113, (char) 127, (char) 88, (char) 10, (char) 55, (char) 226, (char) 106, (char) 211, (char) 98, (char) 83, (char) 103, (char) 96, (char) 92, (char) 78, (char) 153, (char) 189, (char) 110, (char) 138, (char) 16, (char) 109, (char) 102, (char) 115, (char) 32, (char) 24, (char) 194, (char) 228, (char) 192, (char) 50, (char) 249, (char) 152, (char) 186, (char) 193, (char) 182, (char) 56, (char) 223, (char) 212, (char) 149, (char) 158, (char) 248, (char) 176, (char) 142, (char) 25, (char) 89, (char) 27, (char) 88, (char) 35, (char) 111, (char) 57, (char) 15, (char) 242, (char) 189, (char) 170, (char) 125, (char) 189, (char) 100, (char) 110, (char) 79, (char) 58, (char) 51, (char) 233, (char) 103, (char) 111, (char) 214, (char) 75, (char) 87, (char) 235, (char) 56, (char) 25, (char) 219, (char) 216, (char) 42, (char) 145, (char) 141, (char) 234, (char) 54, (char) 86, (char) 23, (char) 159, (char) 142, (char) 20, (char) 31, (char) 7, (char) 244, (char) 22, (char) 75, (char) 102, (char) 110, (char) 137, (char) 60, (char) 230, (char) 177, (char) 118, (char) 194, (char) 214, (char) 0, (char) 217, (char) 66, (char) 196, (char) 52, (char) 207, (char) 211, (char) 81, (char) 202, (char) 7, (char) 150, (char) 92, (char) 59, (char) 139, (char) 136, (char) 93, (char) 142, (char) 12}));
			assert (pack.length_GET() == (char) 92);
			assert (pack.first_message_offset_GET() == (char) 159);
		});
		GroundControl.LOGGING_DATA_ACKED p267 = CommunicationChannel.new_LOGGING_DATA_ACKED();
		PH.setPack(p267);
		p267.length_SET((char) 92);
		p267.target_component_SET((char) 236);
		p267.sequence_SET((char) 16272);
		p267.target_system_SET((char) 117);
		p267.data__SET(new char[]{(char) 173, (char) 54, (char) 159, (char) 232, (char) 243, (char) 233, (char) 60, (char) 227, (char) 60, (char) 9, (char) 150, (char) 228, (char) 40, (char) 181, (char) 109, (char) 70, (char) 211, (char) 96, (char) 24, (char) 82, (char) 54, (char) 144, (char) 85, (char) 11, (char) 241, (char) 143, (char) 93, (char) 177, (char) 172, (char) 144, (char) 220, (char) 74, (char) 10, (char) 43, (char) 40, (char) 148, (char) 113, (char) 138, (char) 100, (char) 23, (char) 205, (char) 21, (char) 101, (char) 111, (char) 185, (char) 62, (char) 255, (char) 60, (char) 54, (char) 175, (char) 76, (char) 77, (char) 178, (char) 231, (char) 109, (char) 82, (char) 101, (char) 187, (char) 77, (char) 5, (char) 250, (char) 252, (char) 178, (char) 23, (char) 213, (char) 45, (char) 2, (char) 59, (char) 70, (char) 75, (char) 201, (char) 93, (char) 109, (char) 35, (char) 106, (char) 170, (char) 235, (char) 188, (char) 30, (char) 119, (char) 22, (char) 78, (char) 133, (char) 112, (char) 38, (char) 92, (char) 210, (char) 112, (char) 217, (char) 105, (char) 232, (char) 225, (char) 230, (char) 57, (char) 42, (char) 153, (char) 80, (char) 122, (char) 182, (char) 158, (char) 72, (char) 16, (char) 6, (char) 177, (char) 89, (char) 220, (char) 156, (char) 145, (char) 60, (char) 110, (char) 206, (char) 253, (char) 246, (char) 74, (char) 116, (char) 187, (char) 198, (char) 238, (char) 31, (char) 248, (char) 118, (char) 215, (char) 107, (char) 98, (char) 152, (char) 16, (char) 253, (char) 233, (char) 166, (char) 141, (char) 139, (char) 16, (char) 51, (char) 152, (char) 181, (char) 37, (char) 131, (char) 113, (char) 127, (char) 88, (char) 10, (char) 55, (char) 226, (char) 106, (char) 211, (char) 98, (char) 83, (char) 103, (char) 96, (char) 92, (char) 78, (char) 153, (char) 189, (char) 110, (char) 138, (char) 16, (char) 109, (char) 102, (char) 115, (char) 32, (char) 24, (char) 194, (char) 228, (char) 192, (char) 50, (char) 249, (char) 152, (char) 186, (char) 193, (char) 182, (char) 56, (char) 223, (char) 212, (char) 149, (char) 158, (char) 248, (char) 176, (char) 142, (char) 25, (char) 89, (char) 27, (char) 88, (char) 35, (char) 111, (char) 57, (char) 15, (char) 242, (char) 189, (char) 170, (char) 125, (char) 189, (char) 100, (char) 110, (char) 79, (char) 58, (char) 51, (char) 233, (char) 103, (char) 111, (char) 214, (char) 75, (char) 87, (char) 235, (char) 56, (char) 25, (char) 219, (char) 216, (char) 42, (char) 145, (char) 141, (char) 234, (char) 54, (char) 86, (char) 23, (char) 159, (char) 142, (char) 20, (char) 31, (char) 7, (char) 244, (char) 22, (char) 75, (char) 102, (char) 110, (char) 137, (char) 60, (char) 230, (char) 177, (char) 118, (char) 194, (char) 214, (char) 0, (char) 217, (char) 66, (char) 196, (char) 52, (char) 207, (char) 211, (char) 81, (char) 202, (char) 7, (char) 150, (char) 92, (char) 59, (char) 139, (char) 136, (char) 93, (char) 142, (char) 12}, 0);
		p267.first_message_offset_SET((char) 159);
		CommunicationChannel.instance.send(p267);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_LOGGING_ACK.add((src, ph, pack) ->
		{
			assert (pack.sequence_GET() == (char) 36503);
			assert (pack.target_component_GET() == (char) 151);
			assert (pack.target_system_GET() == (char) 191);
		});
		GroundControl.LOGGING_ACK p268 = CommunicationChannel.new_LOGGING_ACK();
		PH.setPack(p268);
		p268.target_component_SET((char) 151);
		p268.sequence_SET((char) 36503);
		p268.target_system_SET((char) 191);
		CommunicationChannel.instance.send(p268);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_VIDEO_STREAM_INFORMATION.add((src, ph, pack) ->
		{
			assert (pack.status_GET() == (char) 54);
			assert (pack.framerate_GET() == 9.577409E37F);
			assert (pack.rotation_GET() == (char) 5802);
			assert (pack.bitrate_GET() == 3209983081L);
			assert (pack.camera_id_GET() == (char) 115);
			assert (pack.uri_LEN(ph) == 137);
			assert (pack.uri_TRY(ph).equals("vyJkxtdqopHodYteuyofqZcueknadhdrrvqvdhnnvkhvnjvyxblRrVicujatipBIxySlnhdsPwnmwaowrldbxxeSskZeNkfobbflkpvtdqPfszagfSpuwdnqrgibjWKOpYqnxRuxd"));
			assert (pack.resolution_v_GET() == (char) 11698);
			assert (pack.resolution_h_GET() == (char) 63790);
		});
		GroundControl.VIDEO_STREAM_INFORMATION p269 = CommunicationChannel.new_VIDEO_STREAM_INFORMATION();
		PH.setPack(p269);
		p269.status_SET((char) 54);
		p269.framerate_SET(9.577409E37F);
		p269.uri_SET("vyJkxtdqopHodYteuyofqZcueknadhdrrvqvdhnnvkhvnjvyxblRrVicujatipBIxySlnhdsPwnmwaowrldbxxeSskZeNkfobbflkpvtdqPfszagfSpuwdnqrgibjWKOpYqnxRuxd", PH);
		p269.bitrate_SET(3209983081L);
		p269.resolution_h_SET((char) 63790);
		p269.camera_id_SET((char) 115);
		p269.resolution_v_SET((char) 11698);
		p269.rotation_SET((char) 5802);
		CommunicationChannel.instance.send(p269);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_SET_VIDEO_STREAM_SETTINGS.add((src, ph, pack) ->
		{
			assert (pack.camera_id_GET() == (char) 21);
			assert (pack.framerate_GET() == -6.8969E37F);
			assert (pack.rotation_GET() == (char) 60886);
			assert (pack.uri_LEN(ph) == 115);
			assert (pack.uri_TRY(ph).equals("xdLmxbwtznvWByjfcpvrqlkmqfvcnuvSeHRcbzcuQqenhdgrvafdxbyqzgjauOunSpDvyeujsZySioswmrozxmqpmlzuizundvnswctgdtzuvpgvmht"));
			assert (pack.target_component_GET() == (char) 75);
			assert (pack.target_system_GET() == (char) 81);
			assert (pack.resolution_h_GET() == (char) 53973);
			assert (pack.resolution_v_GET() == (char) 23550);
			assert (pack.bitrate_GET() == 3699249367L);
		});
		GroundControl.SET_VIDEO_STREAM_SETTINGS p270 = CommunicationChannel.new_SET_VIDEO_STREAM_SETTINGS();
		PH.setPack(p270);
		p270.target_component_SET((char) 75);
		p270.bitrate_SET(3699249367L);
		p270.camera_id_SET((char) 21);
		p270.resolution_v_SET((char) 23550);
		p270.resolution_h_SET((char) 53973);
		p270.uri_SET("xdLmxbwtznvWByjfcpvrqlkmqfvcnuvSeHRcbzcuQqenhdgrvafdxbyqzgjauOunSpDvyeujsZySioswmrozxmqpmlzuizundvnswctgdtzuvpgvmht", PH);
		p270.framerate_SET(-6.8969E37F);
		p270.target_system_SET((char) 81);
		p270.rotation_SET((char) 60886);
		CommunicationChannel.instance.send(p270);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_WIFI_CONFIG_AP.add((src, ph, pack) ->
		{
			assert (pack.password_LEN(ph) == 40);
			assert (pack.password_TRY(ph).equals("KrhtrindekXaariyccqtgxsbJplhotekuqqzuovw"));
			assert (pack.ssid_LEN(ph) == 22);
			assert (pack.ssid_TRY(ph).equals("gooahuyxZupwuditZrldsw"));
		});
		GroundControl.WIFI_CONFIG_AP p299 = CommunicationChannel.new_WIFI_CONFIG_AP();
		PH.setPack(p299);
		p299.password_SET("KrhtrindekXaariyccqtgxsbJplhotekuqqzuovw", PH);
		p299.ssid_SET("gooahuyxZupwuditZrldsw", PH);
		CommunicationChannel.instance.send(p299);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_PROTOCOL_VERSION.add((src, ph, pack) ->
		{
			assert (Arrays.equals(pack.library_version_hash_GET(), new char[]{(char) 183, (char) 229, (char) 43, (char) 253, (char) 174, (char) 175, (char) 9, (char) 21}));
			assert (Arrays.equals(pack.spec_version_hash_GET(), new char[]{(char) 123, (char) 116, (char) 154, (char) 95, (char) 234, (char) 248, (char) 182, (char) 65}));
			assert (pack.min_version_GET() == (char) 28799);
			assert (pack.version_GET() == (char) 33873);
			assert (pack.max_version_GET() == (char) 53458);
		});
		GroundControl.PROTOCOL_VERSION p300 = CommunicationChannel.new_PROTOCOL_VERSION();
		PH.setPack(p300);
		p300.library_version_hash_SET(new char[]{(char) 183, (char) 229, (char) 43, (char) 253, (char) 174, (char) 175, (char) 9, (char) 21}, 0);
		p300.version_SET((char) 33873);
		p300.min_version_SET((char) 28799);
		p300.max_version_SET((char) 53458);
		p300.spec_version_hash_SET(new char[]{(char) 123, (char) 116, (char) 154, (char) 95, (char) 234, (char) 248, (char) 182, (char) 65}, 0);
		CommunicationChannel.instance.send(p300);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_UAVCAN_NODE_STATUS.add((src, ph, pack) ->
		{
			assert (pack.uptime_sec_GET() == 1332880251L);
			assert (pack.vendor_specific_status_code_GET() == (char) 7539);
			assert (pack.health_GET() == UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_WARNING);
			assert (pack.mode_GET() == UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_OFFLINE);
			assert (pack.time_usec_GET() == 7713249812489218481L);
			assert (pack.sub_mode_GET() == (char) 142);
		});
		GroundControl.UAVCAN_NODE_STATUS p310 = CommunicationChannel.new_UAVCAN_NODE_STATUS();
		PH.setPack(p310);
		p310.sub_mode_SET((char) 142);
		p310.health_SET(UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_WARNING);
		p310.mode_SET(UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_OFFLINE);
		p310.vendor_specific_status_code_SET((char) 7539);
		p310.time_usec_SET(7713249812489218481L);
		p310.uptime_sec_SET(1332880251L);
		CommunicationChannel.instance.send(p310);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_UAVCAN_NODE_INFO.add((src, ph, pack) ->
		{
			assert (Arrays.equals(pack.hw_unique_id_GET(), new char[]{(char) 106, (char) 128, (char) 80, (char) 215, (char) 114, (char) 231, (char) 194, (char) 247, (char) 14, (char) 31, (char) 87, (char) 105, (char) 109, (char) 54, (char) 63, (char) 205}));
			assert (pack.hw_version_major_GET() == (char) 56);
			assert (pack.sw_version_major_GET() == (char) 184);
			assert (pack.uptime_sec_GET() == 1066633226L);
			assert (pack.sw_version_minor_GET() == (char) 143);
			assert (pack.time_usec_GET() == 2107994250417402807L);
			assert (pack.sw_vcs_commit_GET() == 2984517098L);
			assert (pack.hw_version_minor_GET() == (char) 50);
			assert (pack.name_LEN(ph) == 48);
			assert (pack.name_TRY(ph).equals("lsioxhgjorcywrdjvpsthuUoihhwsbExrliywdPitzjuxnjb"));
		});
		GroundControl.UAVCAN_NODE_INFO p311 = CommunicationChannel.new_UAVCAN_NODE_INFO();
		PH.setPack(p311);
		p311.sw_vcs_commit_SET(2984517098L);
		p311.hw_unique_id_SET(new char[]{(char) 106, (char) 128, (char) 80, (char) 215, (char) 114, (char) 231, (char) 194, (char) 247, (char) 14, (char) 31, (char) 87, (char) 105, (char) 109, (char) 54, (char) 63, (char) 205}, 0);
		p311.time_usec_SET(2107994250417402807L);
		p311.sw_version_minor_SET((char) 143);
		p311.hw_version_major_SET((char) 56);
		p311.uptime_sec_SET(1066633226L);
		p311.sw_version_major_SET((char) 184);
		p311.name_SET("lsioxhgjorcywrdjvpsthuUoihhwsbExrliywdPitzjuxnjb", PH);
		p311.hw_version_minor_SET((char) 50);
		CommunicationChannel.instance.send(p311);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_PARAM_EXT_REQUEST_READ.add((src, ph, pack) ->
		{
			assert (pack.param_index_GET() == (short) -1633);
			assert (pack.param_id_LEN(ph) == 15);
			assert (pack.param_id_TRY(ph).equals("qunrBxdkctvhjec"));
			assert (pack.target_component_GET() == (char) 172);
			assert (pack.target_system_GET() == (char) 251);
		});
		GroundControl.PARAM_EXT_REQUEST_READ p320 = CommunicationChannel.new_PARAM_EXT_REQUEST_READ();
		PH.setPack(p320);
		p320.param_index_SET((short) -1633);
		p320.param_id_SET("qunrBxdkctvhjec", PH);
		p320.target_component_SET((char) 172);
		p320.target_system_SET((char) 251);
		CommunicationChannel.instance.send(p320);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_PARAM_EXT_REQUEST_LIST.add((src, ph, pack) ->
		{
			assert (pack.target_component_GET() == (char) 27);
			assert (pack.target_system_GET() == (char) 216);
		});
		GroundControl.PARAM_EXT_REQUEST_LIST p321 = CommunicationChannel.new_PARAM_EXT_REQUEST_LIST();
		PH.setPack(p321);
		p321.target_system_SET((char) 216);
		p321.target_component_SET((char) 27);
		CommunicationChannel.instance.send(p321);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_PARAM_EXT_VALUE.add((src, ph, pack) ->
		{
			assert (pack.param_id_LEN(ph) == 4);
			assert (pack.param_id_TRY(ph).equals("zxvz"));
			assert (pack.param_value_LEN(ph) == 126);
			assert (pack.param_value_TRY(ph).equals("ykvxclhygktvodojfvjndgjnbxisoahzNueanigixvABzmfxovidINcqxghbltasdeayjrdthlaxzslfDlbVkyAsbnabhvibbtzsOwnargrctovgtureRugBKjUxdj"));
			assert (pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT16);
			assert (pack.param_index_GET() == (char) 25922);
			assert (pack.param_count_GET() == (char) 8748);
		});
		GroundControl.PARAM_EXT_VALUE p322 = CommunicationChannel.new_PARAM_EXT_VALUE();
		PH.setPack(p322);
		p322.param_id_SET("zxvz", PH);
		p322.param_count_SET((char) 8748);
		p322.param_index_SET((char) 25922);
		p322.param_value_SET("ykvxclhygktvodojfvjndgjnbxisoahzNueanigixvABzmfxovidINcqxghbltasdeayjrdthlaxzslfDlbVkyAsbnabhvibbtzsOwnargrctovgtureRugBKjUxdj", PH);
		p322.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT16);
		CommunicationChannel.instance.send(p322);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_PARAM_EXT_SET.add((src, ph, pack) ->
		{
			assert (pack.param_id_LEN(ph) == 7);
			assert (pack.param_id_TRY(ph).equals("occaagb"));
			assert (pack.target_system_GET() == (char) 59);
			assert (pack.param_value_LEN(ph) == 69);
			assert (pack.param_value_TRY(ph).equals("jdkedcqzJvpyqnvqfkphoRjBgryzwmupqcomeimyosbvbfckzushxhwejtblleuwdpikh"));
			assert (pack.target_component_GET() == (char) 23);
			assert (pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT16);
		});
		GroundControl.PARAM_EXT_SET p323 = CommunicationChannel.new_PARAM_EXT_SET();
		PH.setPack(p323);
		p323.param_id_SET("occaagb", PH);
		p323.target_system_SET((char) 59);
		p323.param_value_SET("jdkedcqzJvpyqnvqfkphoRjBgryzwmupqcomeimyosbvbfckzushxhwejtblleuwdpikh", PH);
		p323.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT16);
		p323.target_component_SET((char) 23);
		CommunicationChannel.instance.send(p323);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_PARAM_EXT_ACK.add((src, ph, pack) ->
		{
			assert (pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT64);
			assert (pack.param_id_LEN(ph) == 9);
			assert (pack.param_id_TRY(ph).equals("Sxigrvjny"));
			assert (pack.param_result_GET() == PARAM_ACK.PARAM_ACK_ACCEPTED);
			assert (pack.param_value_LEN(ph) == 107);
			assert (pack.param_value_TRY(ph).equals("JkbkszthwhSwCjzxghrgfbqHiEbmresqxutklGxcbiyzkhqdjdcorpikgkulxlpwZgioemtkqlhmuuycrzgatjtyzpKlukmwtArzadugnmy"));
		});
		GroundControl.PARAM_EXT_ACK p324 = CommunicationChannel.new_PARAM_EXT_ACK();
		PH.setPack(p324);
		p324.param_value_SET("JkbkszthwhSwCjzxghrgfbqHiEbmresqxutklGxcbiyzkhqdjdcorpikgkulxlpwZgioemtkqlhmuuycrzgatjtyzpKlukmwtArzadugnmy", PH);
		p324.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT64);
		p324.param_id_SET("Sxigrvjny", PH);
		p324.param_result_SET(PARAM_ACK.PARAM_ACK_ACCEPTED);
		CommunicationChannel.instance.send(p324);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_OBSTACLE_DISTANCE.add((src, ph, pack) ->
		{
			assert (pack.min_distance_GET() == (char) 61056);
			assert (pack.max_distance_GET() == (char) 60347);
			assert (pack.increment_GET() == (char) 221);
			assert (pack.sensor_type_GET() == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_UNKNOWN);
			assert (Arrays.equals(pack.distances_GET(), new char[]{(char) 16946, (char) 59980, (char) 26375, (char) 14970, (char) 43063, (char) 10755, (char) 1981, (char) 11628, (char) 11731, (char) 13654, (char) 42426, (char) 52788, (char) 28861, (char) 14674, (char) 14070, (char) 39638, (char) 8006, (char) 30845, (char) 41273, (char) 65127, (char) 64838, (char) 10198, (char) 48360, (char) 43425, (char) 6479, (char) 39194, (char) 59177, (char) 61620, (char) 1417, (char) 2554, (char) 41051, (char) 59565, (char) 11802, (char) 38520, (char) 56947, (char) 216, (char) 5474, (char) 36923, (char) 58665, (char) 31660, (char) 36855, (char) 2429, (char) 17193, (char) 44813, (char) 7797, (char) 56704, (char) 1997, (char) 42335, (char) 23361, (char) 58539, (char) 62017, (char) 46826, (char) 4783, (char) 35569, (char) 23876, (char) 46839, (char) 14579, (char) 56436, (char) 5683, (char) 49415, (char) 24931, (char) 52101, (char) 44325, (char) 56472, (char) 61538, (char) 46923, (char) 15294, (char) 12318, (char) 24675, (char) 6480, (char) 7688, (char) 51988}));
			assert (pack.time_usec_GET() == 7976790958210261253L);
		});
		GroundControl.OBSTACLE_DISTANCE p330 = CommunicationChannel.new_OBSTACLE_DISTANCE();
		PH.setPack(p330);
		p330.time_usec_SET(7976790958210261253L);
		p330.sensor_type_SET(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_UNKNOWN);
		p330.distances_SET(new char[]{(char) 16946, (char) 59980, (char) 26375, (char) 14970, (char) 43063, (char) 10755, (char) 1981, (char) 11628, (char) 11731, (char) 13654, (char) 42426, (char) 52788, (char) 28861, (char) 14674, (char) 14070, (char) 39638, (char) 8006, (char) 30845, (char) 41273, (char) 65127, (char) 64838, (char) 10198, (char) 48360, (char) 43425, (char) 6479, (char) 39194, (char) 59177, (char) 61620, (char) 1417, (char) 2554, (char) 41051, (char) 59565, (char) 11802, (char) 38520, (char) 56947, (char) 216, (char) 5474, (char) 36923, (char) 58665, (char) 31660, (char) 36855, (char) 2429, (char) 17193, (char) 44813, (char) 7797, (char) 56704, (char) 1997, (char) 42335, (char) 23361, (char) 58539, (char) 62017, (char) 46826, (char) 4783, (char) 35569, (char) 23876, (char) 46839, (char) 14579, (char) 56436, (char) 5683, (char) 49415, (char) 24931, (char) 52101, (char) 44325, (char) 56472, (char) 61538, (char) 46923, (char) 15294, (char) 12318, (char) 24675, (char) 6480, (char) 7688, (char) 51988}, 0);
		p330.increment_SET((char) 221);
		p330.min_distance_SET((char) 61056);
		p330.max_distance_SET((char) 60347);
		CommunicationChannel.instance.send(p330);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
	}

}