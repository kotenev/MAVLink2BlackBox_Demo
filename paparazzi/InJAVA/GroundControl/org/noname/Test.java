
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
				default:
					assert (false);//("Unknown enum" + id);
			}
			set_bits(id, 7, data, 276);
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
			set_bits(id, 3, data, 283);
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
				default:
					assert (false);//("Unknown enum" + id);
			}
			set_bits(id, 7, data, 276);
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
			set_bits(id, 3, data, 283);
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

	public static class ALTITUDE extends GroundControl.ALTITUDE {
		public long time_usec_GET()//Timestamp (micros since boot or Unix epoch)
		{ return (get_bytes(data, 0, 8)); }

		/**
		 * This altitude measure is initialized on system boot and monotonic (it is never reset, but represents the
		 * local altitude change). The only guarantee on this field is that it will never be reset and is consistent
		 * within a flight. The recommended value for this field is the uncorrected barometric altitude at boot
		 * time. This altitude will also drift and vary between flights
		 */
		public float altitude_monotonic_GET() { return (float) (Float.intBitsToFloat((int) get_bytes(data, 8, 4))); }

		/**
		 * This altitude measure is strictly above mean sea level and might be non-monotonic (it might reset on events
		 * like GPS lock or when a new QNH value is set). It should be the altitude to which global altitude waypoints
		 * are compared to. Note that it is *not* the GPS altitude, however, most GPS modules already output AMSL
		 * by default and not the WGS84 altitude
		 */
		public float altitude_amsl_GET() { return (float) (Float.intBitsToFloat((int) get_bytes(data, 12, 4))); }

		/**
		 * This is the local altitude in the local coordinate frame. It is not the altitude above home, but in reference
		 * to the coordinate origin (0, 0, 0). It is up-positive
		 */
		public float altitude_local_GET() { return (float) (Float.intBitsToFloat((int) get_bytes(data, 16, 4))); }

		public float altitude_relative_GET()//This is the altitude above the home position. It resets on each change of the current home position
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 20, 4))); }

		/**
		 * This is the altitude above terrain. It might be fed by a terrain database or an altimeter. Values smaller
		 * than -1000 should be interpreted as unknown
		 */
		public float altitude_terrain_GET() { return (float) (Float.intBitsToFloat((int) get_bytes(data, 24, 4))); }

		/**
		 * This is not the altitude, but the clear space below the system according to the fused clearance estimate.
		 * It generally should max out at the maximum range of e.g. the laser altimeter. It is generally a moving
		 * target. A negative value indicates no measurement available
		 */
		public float bottom_clearance_GET() { return (float) (Float.intBitsToFloat((int) get_bytes(data, 28, 4))); }
	}

	public static class RESOURCE_REQUEST extends GroundControl.RESOURCE_REQUEST {
		public char request_id_GET()//Request ID. This ID should be re-used when sending back URI contents
		{ return (char) ((char) get_bytes(data, 0, 1)); }

		public char uri_type_GET()//The type of requested URI. 0 = a file via URL. 1 = a UAVCAN binary
		{ return (char) ((char) get_bytes(data, 1, 1)); }

		/**
		 * The requested unique resource identifier (URI). It is not necessarily a straight domain name (depends
		 * on the URI type enum
		 */
		public char[] uri_GET(char[] dst_ch, int pos) {
			for (int BYTE = 2, dst_max = pos + 120; pos < dst_max; pos++, BYTE += 1)
				dst_ch[pos] = (char) ((char) get_bytes(data, BYTE, 1));
			return dst_ch;
		}

		/**
		 * The requested unique resource identifier (URI). It is not necessarily a straight domain name (depends
		 * on the URI type enum
		 */
		public char[] uri_GET() {return uri_GET(new char[120], 0);}

		public char transfer_type_GET()//The way the autopilot wants to receive the URI. 0 = MAVLink FTP. 1 = binary stream.
		{ return (char) ((char) get_bytes(data, 122, 1)); }

		/**
		 * The storage path the autopilot wants the URI to be stored in. Will only be valid if the transfer_type
		 * has a storage associated (e.g. MAVLink FTP)
		 */
		public char[] storage_GET(char[] dst_ch, int pos) {
			for (int BYTE = 123, dst_max = pos + 120; pos < dst_max; pos++, BYTE += 1)
				dst_ch[pos] = (char) ((char) get_bytes(data, BYTE, 1));
			return dst_ch;
		}

		/**
		 * The storage path the autopilot wants the URI to be stored in. Will only be valid if the transfer_type
		 * has a storage associated (e.g. MAVLink FTP)
		 */
		public char[] storage_GET() {return storage_GET(new char[120], 0);}
	}

	public static class SCALED_PRESSURE3 extends GroundControl.SCALED_PRESSURE3 {
		public long time_boot_ms_GET()//Timestamp (milliseconds since system boot)
		{ return (get_bytes(data, 0, 4)); }

		public float press_abs_GET()//Absolute pressure (hectopascal)
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 4, 4))); }

		public float press_diff_GET()//Differential pressure 1 (hectopascal)
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 8, 4))); }

		public short temperature_GET()//Temperature measurement (0.01 degrees celsius)
		{ return (short) ((short) get_bytes(data, 12, 2)); }
	}

	public static class FOLLOW_TARGET extends GroundControl.FOLLOW_TARGET {
		public long timestamp_GET()//Timestamp in milliseconds since system boot
		{ return (get_bytes(data, 0, 8)); }

		public long custom_state_GET()//button states or switches of a tracker device
		{ return (get_bytes(data, 8, 8)); }

		public char est_capabilities_GET()//bit positions for tracker reporting capabilities (POS = 0, VEL = 1, ACCEL = 2, ATT + RATES = 3)
		{ return (char) ((char) get_bytes(data, 16, 1)); }

		public int lat_GET()//Latitude (WGS84), in degrees * 1E7
		{ return (int) ((int) get_bytes(data, 17, 4)); }

		public int lon_GET()//Longitude (WGS84), in degrees * 1E7
		{ return (int) ((int) get_bytes(data, 21, 4)); }

		public float alt_GET()//AMSL, in meters
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 25, 4))); }

		public float[] vel_GET(float[] dst_ch, int pos)  //target velocity (0,0,0) for unknown
		{
			for (int BYTE = 29, dst_max = pos + 3; pos < dst_max; pos++, BYTE += 4)
				dst_ch[pos] = (float) (Float.intBitsToFloat((int) get_bytes(data, BYTE, 4)));
			return dst_ch;
		}

		public float[] vel_GET()//target velocity (0,0,0) for unknown
		{return vel_GET(new float[3], 0);}

		public float[] acc_GET(float[] dst_ch, int pos)  //linear target acceleration (0,0,0) for unknown
		{
			for (int BYTE = 41, dst_max = pos + 3; pos < dst_max; pos++, BYTE += 4)
				dst_ch[pos] = (float) (Float.intBitsToFloat((int) get_bytes(data, BYTE, 4)));
			return dst_ch;
		}

		public float[] acc_GET()//linear target acceleration (0,0,0) for unknown
		{return acc_GET(new float[3], 0);}

		public float[] attitude_q_GET(float[] dst_ch, int pos)  //(1 0 0 0 for unknown)
		{
			for (int BYTE = 53, dst_max = pos + 4; pos < dst_max; pos++, BYTE += 4)
				dst_ch[pos] = (float) (Float.intBitsToFloat((int) get_bytes(data, BYTE, 4)));
			return dst_ch;
		}

		public float[] attitude_q_GET()//(1 0 0 0 for unknown)
		{return attitude_q_GET(new float[4], 0);}

		public float[] rates_GET(float[] dst_ch, int pos)  //(0 0 0 for unknown)
		{
			for (int BYTE = 69, dst_max = pos + 3; pos < dst_max; pos++, BYTE += 4)
				dst_ch[pos] = (float) (Float.intBitsToFloat((int) get_bytes(data, BYTE, 4)));
			return dst_ch;
		}

		public float[] rates_GET()//(0 0 0 for unknown)
		{return rates_GET(new float[3], 0);}

		public float[] position_cov_GET(float[] dst_ch, int pos)  //eph epv
		{
			for (int BYTE = 81, dst_max = pos + 3; pos < dst_max; pos++, BYTE += 4)
				dst_ch[pos] = (float) (Float.intBitsToFloat((int) get_bytes(data, BYTE, 4)));
			return dst_ch;
		}

		public float[] position_cov_GET()//eph epv
		{return position_cov_GET(new float[3], 0);}
	}

	public static class CONTROL_SYSTEM_STATE extends GroundControl.CONTROL_SYSTEM_STATE {
		public long time_usec_GET()//Timestamp (micros since boot or Unix epoch)
		{ return (get_bytes(data, 0, 8)); }

		public float x_acc_GET()//X acceleration in body frame
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 8, 4))); }

		public float y_acc_GET()//Y acceleration in body frame
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 12, 4))); }

		public float z_acc_GET()//Z acceleration in body frame
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 16, 4))); }

		public float x_vel_GET()//X velocity in body frame
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 20, 4))); }

		public float y_vel_GET()//Y velocity in body frame
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 24, 4))); }

		public float z_vel_GET()//Z velocity in body frame
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 28, 4))); }

		public float x_pos_GET()//X position in local frame
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 32, 4))); }

		public float y_pos_GET()//Y position in local frame
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 36, 4))); }

		public float z_pos_GET()//Z position in local frame
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 40, 4))); }

		public float airspeed_GET()//Airspeed, set to -1 if unknown
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 44, 4))); }

		public float[] vel_variance_GET(float[] dst_ch, int pos)  //Variance of body velocity estimate
		{
			for (int BYTE = 48, dst_max = pos + 3; pos < dst_max; pos++, BYTE += 4)
				dst_ch[pos] = (float) (Float.intBitsToFloat((int) get_bytes(data, BYTE, 4)));
			return dst_ch;
		}

		public float[] vel_variance_GET()//Variance of body velocity estimate
		{return vel_variance_GET(new float[3], 0);}

		public float[] pos_variance_GET(float[] dst_ch, int pos)  //Variance in local position
		{
			for (int BYTE = 60, dst_max = pos + 3; pos < dst_max; pos++, BYTE += 4)
				dst_ch[pos] = (float) (Float.intBitsToFloat((int) get_bytes(data, BYTE, 4)));
			return dst_ch;
		}

		public float[] pos_variance_GET()//Variance in local position
		{return pos_variance_GET(new float[3], 0);}

		public float[] q_GET(float[] dst_ch, int pos)  //The attitude, represented as Quaternion
		{
			for (int BYTE = 72, dst_max = pos + 4; pos < dst_max; pos++, BYTE += 4)
				dst_ch[pos] = (float) (Float.intBitsToFloat((int) get_bytes(data, BYTE, 4)));
			return dst_ch;
		}

		public float[] q_GET()//The attitude, represented as Quaternion
		{return q_GET(new float[4], 0);}

		public float roll_rate_GET()//Angular rate in roll axis
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 88, 4))); }

		public float pitch_rate_GET()//Angular rate in pitch axis
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 92, 4))); }

		public float yaw_rate_GET()//Angular rate in yaw axis
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 96, 4))); }
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

	public static class SCRIPT_ITEM extends GroundControl.SCRIPT_ITEM {
		public char seq_GET()//Sequence
		{ return (char) ((char) get_bytes(data, 0, 2)); }

		public char target_system_GET()//System ID
		{ return (char) ((char) get_bytes(data, 2, 1)); }

		public char target_component_GET()//Component ID
		{ return (char) ((char) get_bytes(data, 3, 1)); }

		public String name_TRY(Bounds.Inside ph)//The name of the mission script, NULL terminated.
		{
			if (ph.field_bit != 32 && !try_visit_field(ph, 32) || !try_visit_item(ph, 0)) return null;
			return new String(name_GET(ph, new char[ph.items], 0));
		}

		public char[] name_GET(Bounds.Inside ph, char[] dst_ch, int pos) //The name of the mission script, NULL terminated.
		{
			for (int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max; pos++, BYTE += 2)
				dst_ch[pos] = (char) ((char) get_bytes(data, BYTE, 2));
			return dst_ch;
		}

		public int name_LEN(Bounds.Inside ph) {
			return (ph.field_bit != 32 && !try_visit_field(ph, 32) || !try_visit_item(ph, 0)) ? 0 : ph.items;
		}
	}

	public static class SCRIPT_REQUEST extends GroundControl.SCRIPT_REQUEST {
		public char seq_GET()//Sequence
		{ return (char) ((char) get_bytes(data, 0, 2)); }

		public char target_system_GET()//System ID
		{ return (char) ((char) get_bytes(data, 2, 1)); }

		public char target_component_GET()//Component ID
		{ return (char) ((char) get_bytes(data, 3, 1)); }
	}

	public static class SCRIPT_REQUEST_LIST extends GroundControl.SCRIPT_REQUEST_LIST {
		public char target_system_GET()//System ID
		{ return (char) ((char) get_bytes(data, 0, 1)); }

		public char target_component_GET()//Component ID
		{ return (char) ((char) get_bytes(data, 1, 1)); }
	}

	public static class SCRIPT_COUNT extends GroundControl.SCRIPT_COUNT {
		public char count_GET()//Number of script items in the sequence
		{ return (char) ((char) get_bytes(data, 0, 2)); }

		public char target_system_GET()//System ID
		{ return (char) ((char) get_bytes(data, 2, 1)); }

		public char target_component_GET()//Component ID
		{ return (char) ((char) get_bytes(data, 3, 1)); }
	}

	public static class SCRIPT_CURRENT extends GroundControl.SCRIPT_CURRENT {
		public char seq_GET()//Active Sequence
		{ return (char) ((char) get_bytes(data, 0, 2)); }
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

		static final Collection<OnReceive.Handler<ALTITUDE, Channel>>                  on_ALTITUDE                  = new OnReceive<>();
		static final Collection<OnReceive.Handler<RESOURCE_REQUEST, Channel>>          on_RESOURCE_REQUEST          = new OnReceive<>();
		static final Collection<OnReceive.Handler<SCALED_PRESSURE3, Channel>>          on_SCALED_PRESSURE3          = new OnReceive<>();
		static final Collection<OnReceive.Handler<FOLLOW_TARGET, Channel>>             on_FOLLOW_TARGET             = new OnReceive<>();
		static final Collection<OnReceive.Handler<CONTROL_SYSTEM_STATE, Channel>>      on_CONTROL_SYSTEM_STATE      = new OnReceive<>();
		static final Collection<OnReceive.Handler<BATTERY_STATUS, Channel>>            on_BATTERY_STATUS            = new OnReceive<>();
		static final Collection<OnReceive.Handler<AUTOPILOT_VERSION, Channel>>         on_AUTOPILOT_VERSION         = new OnReceive<>();
		static final Collection<OnReceive.Handler<LANDING_TARGET, Channel>>            on_LANDING_TARGET            = new OnReceive<>();
		static final Collection<OnReceive.Handler<SCRIPT_ITEM, Channel>>               on_SCRIPT_ITEM               = new OnReceive<>();
		static final Collection<OnReceive.Handler<SCRIPT_REQUEST, Channel>>            on_SCRIPT_REQUEST            = new OnReceive<>();
		static final Collection<OnReceive.Handler<SCRIPT_REQUEST_LIST, Channel>>       on_SCRIPT_REQUEST_LIST       = new OnReceive<>();
		static final Collection<OnReceive.Handler<SCRIPT_COUNT, Channel>>              on_SCRIPT_COUNT              = new OnReceive<>();
		static final Collection<OnReceive.Handler<SCRIPT_CURRENT, Channel>>            on_SCRIPT_CURRENT            = new OnReceive<>();
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
				case 141:
					if (pack == null) return new ALTITUDE();
					((OnReceive) on_ALTITUDE).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
					break;
				case 142:
					if (pack == null) return new RESOURCE_REQUEST();
					((OnReceive) on_RESOURCE_REQUEST).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
					break;
				case 143:
					if (pack == null) return new SCALED_PRESSURE3();
					((OnReceive) on_SCALED_PRESSURE3).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
					break;
				case 144:
					if (pack == null) return new FOLLOW_TARGET();
					((OnReceive) on_FOLLOW_TARGET).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
					break;
				case 146:
					if (pack == null) return new CONTROL_SYSTEM_STATE();
					((OnReceive) on_CONTROL_SYSTEM_STATE).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
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
				case 180:
					if (pack == null) return new SCRIPT_ITEM();
					((OnReceive) on_SCRIPT_ITEM).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
					break;
				case 181:
					if (pack == null) return new SCRIPT_REQUEST();
					((OnReceive) on_SCRIPT_REQUEST).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
					break;
				case 182:
					if (pack == null) return new SCRIPT_REQUEST_LIST();
					((OnReceive) on_SCRIPT_REQUEST_LIST).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
					break;
				case 183:
					if (pack == null) return new SCRIPT_COUNT();
					((OnReceive) on_SCRIPT_COUNT).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
					break;
				case 184:
					if (pack == null) return new SCRIPT_CURRENT();
					((OnReceive) on_SCRIPT_CURRENT).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
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
			assert (pack.mavlink_version_GET() == (char) 215);
			assert (pack.autopilot_GET() == MAV_AUTOPILOT.MAV_AUTOPILOT_SLUGS);
			assert (pack.type_GET() == MAV_TYPE.MAV_TYPE_HELICOPTER);
			assert (pack.base_mode_GET() == MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED);
			assert (pack.system_status_GET() == MAV_STATE.MAV_STATE_POWEROFF);
			assert (pack.custom_mode_GET() == 611766890L);
		});
		HEARTBEAT p0 = new HEARTBEAT();
		PH.setPack(p0);
		p0.base_mode_SET(MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED);
		p0.type_SET(MAV_TYPE.MAV_TYPE_HELICOPTER);
		p0.mavlink_version_SET((char) 215);
		p0.autopilot_SET(MAV_AUTOPILOT.MAV_AUTOPILOT_SLUGS);
		p0.custom_mode_SET(611766890L);
		p0.system_status_SET(MAV_STATE.MAV_STATE_POWEROFF);
		TestChannel.instance.send(p0);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_SYS_STATUS.add((src, ph, pack) ->
		{
			assert (pack.current_battery_GET() == (short) 24279);
			assert (pack.voltage_battery_GET() == (char) 8734);
			assert (pack.errors_count2_GET() == (char) 30578);
			assert (pack.errors_comm_GET() == (char) 48416);
			assert (pack.errors_count1_GET() == (char) 29973);
			assert (pack.load_GET() == (char) 33488);
			assert (pack.onboard_control_sensors_present_GET() == MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO);
			assert (pack.onboard_control_sensors_health_GET() == MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW);
			assert (pack.onboard_control_sensors_enabled_GET() == MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW);
			assert (pack.drop_rate_comm_GET() == (char) 1733);
			assert (pack.battery_remaining_GET() == (byte) 12);
			assert (pack.errors_count3_GET() == (char) 36349);
			assert (pack.errors_count4_GET() == (char) 25364);
		});
		SYS_STATUS p1 = new SYS_STATUS();
		PH.setPack(p1);
		p1.errors_comm_SET((char) 48416);
		p1.onboard_control_sensors_enabled_SET(MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW);
		p1.onboard_control_sensors_health_SET(MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW);
		p1.voltage_battery_SET((char) 8734);
		p1.errors_count2_SET((char) 30578);
		p1.errors_count1_SET((char) 29973);
		p1.onboard_control_sensors_present_SET(MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO);
		p1.errors_count3_SET((char) 36349);
		p1.errors_count4_SET((char) 25364);
		p1.load_SET((char) 33488);
		p1.drop_rate_comm_SET((char) 1733);
		p1.current_battery_SET((short) 24279);
		p1.battery_remaining_SET((byte) 12);
		TestChannel.instance.send(p1);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_SYSTEM_TIME.add((src, ph, pack) ->
		{
			assert (pack.time_boot_ms_GET() == 3652266517L);
			assert (pack.time_unix_usec_GET() == 7626983796322324689L);
		});
		SYSTEM_TIME p2 = new SYSTEM_TIME();
		PH.setPack(p2);
		p2.time_unix_usec_SET(7626983796322324689L);
		p2.time_boot_ms_SET(3652266517L);
		TestChannel.instance.send(p2);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_POSITION_TARGET_LOCAL_NED.add((src, ph, pack) ->
		{
			assert (pack.yaw_GET() == 3.2596086E38F);
			assert (pack.vz_GET() == -9.550424E37F);
			assert (pack.y_GET() == -2.5670225E38F);
			assert (pack.type_mask_GET() == (char) 56117);
			assert (pack.z_GET() == -2.2053676E38F);
			assert (pack.afy_GET() == -1.8130474E38F);
			assert (pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL);
			assert (pack.afz_GET() == 2.42859E38F);
			assert (pack.yaw_rate_GET() == -2.9359995E38F);
			assert (pack.x_GET() == -2.9721716E37F);
			assert (pack.vx_GET() == 5.003713E37F);
			assert (pack.afx_GET() == -9.73863E37F);
			assert (pack.time_boot_ms_GET() == 365456229L);
			assert (pack.vy_GET() == 2.9481247E38F);
		});
		GroundControl.POSITION_TARGET_LOCAL_NED p3 = CommunicationChannel.new_POSITION_TARGET_LOCAL_NED();
		PH.setPack(p3);
		p3.yaw_SET(3.2596086E38F);
		p3.vx_SET(5.003713E37F);
		p3.time_boot_ms_SET(365456229L);
		p3.afx_SET(-9.73863E37F);
		p3.afz_SET(2.42859E38F);
		p3.type_mask_SET((char) 56117);
		p3.vy_SET(2.9481247E38F);
		p3.yaw_rate_SET(-2.9359995E38F);
		p3.afy_SET(-1.8130474E38F);
		p3.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL);
		p3.x_SET(-2.9721716E37F);
		p3.z_SET(-2.2053676E38F);
		p3.vz_SET(-9.550424E37F);
		p3.y_SET(-2.5670225E38F);
		CommunicationChannel.instance.send(p3);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_PING.add((src, ph, pack) ->
		{
			assert (pack.seq_GET() == 3070759637L);
			assert (pack.target_system_GET() == (char) 195);
			assert (pack.time_usec_GET() == 150872694692045562L);
			assert (pack.target_component_GET() == (char) 85);
		});
		PING p4 = new PING();
		PH.setPack(p4);
		p4.time_usec_SET(150872694692045562L);
		p4.target_component_SET((char) 85);
		p4.seq_SET(3070759637L);
		p4.target_system_SET((char) 195);
		TestChannel.instance.send(p4);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_CHANGE_OPERATOR_CONTROL.add((src, ph, pack) ->
		{
			assert (pack.control_request_GET() == (char) 203);
			assert (pack.version_GET() == (char) 33);
			assert (pack.passkey_LEN(ph) == 9);
			assert (pack.passkey_TRY(ph).equals("txdhbthjg"));
			assert (pack.target_system_GET() == (char) 239);
		});
		CHANGE_OPERATOR_CONTROL p5 = new CHANGE_OPERATOR_CONTROL();
		PH.setPack(p5);
		p5.control_request_SET((char) 203);
		p5.passkey_SET("txdhbthjg", PH);
		p5.version_SET((char) 33);
		p5.target_system_SET((char) 239);
		TestChannel.instance.send(p5);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_CHANGE_OPERATOR_CONTROL_ACK.add((src, ph, pack) ->
		{
			assert (pack.control_request_GET() == (char) 50);
			assert (pack.ack_GET() == (char) 172);
			assert (pack.gcs_system_id_GET() == (char) 222);
		});
		CHANGE_OPERATOR_CONTROL_ACK p6 = new CHANGE_OPERATOR_CONTROL_ACK();
		PH.setPack(p6);
		p6.ack_SET((char) 172);
		p6.control_request_SET((char) 50);
		p6.gcs_system_id_SET((char) 222);
		TestChannel.instance.send(p6);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_AUTH_KEY.add((src, ph, pack) ->
		{
			assert (pack.key_LEN(ph) == 32);
			assert (pack.key_TRY(ph).equals("NIicqixrgvmuzgacgdhtuovgufkjenvb"));
		});
		AUTH_KEY p7 = new AUTH_KEY();
		PH.setPack(p7);
		p7.key_SET("NIicqixrgvmuzgacgdhtuovgufkjenvb", PH);
		TestChannel.instance.send(p7);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_SET_MODE.add((src, ph, pack) ->
		{
			assert (pack.base_mode_GET() == MAV_MODE.MAV_MODE_MANUAL_DISARMED);
			assert (pack.custom_mode_GET() == 3186922054L);
			assert (pack.target_system_GET() == (char) 148);
		});
		SET_MODE p11 = new SET_MODE();
		PH.setPack(p11);
		p11.custom_mode_SET(3186922054L);
		p11.base_mode_SET(MAV_MODE.MAV_MODE_MANUAL_DISARMED);
		p11.target_system_SET((char) 148);
		TestChannel.instance.send(p11);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_PARAM_REQUEST_READ.add((src, ph, pack) ->
		{
			assert (pack.param_id_LEN(ph) == 14);
			assert (pack.param_id_TRY(ph).equals("sDjmglrxlwpqpc"));
			assert (pack.target_component_GET() == (char) 231);
			assert (pack.param_index_GET() == (short) -28531);
			assert (pack.target_system_GET() == (char) 174);
		});
		PARAM_REQUEST_READ p20 = new PARAM_REQUEST_READ();
		PH.setPack(p20);
		p20.target_system_SET((char) 174);
		p20.param_id_SET("sDjmglrxlwpqpc", PH);
		p20.param_index_SET((short) -28531);
		p20.target_component_SET((char) 231);
		TestChannel.instance.send(p20);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_PARAM_REQUEST_LIST.add((src, ph, pack) ->
		{
			assert (pack.target_component_GET() == (char) 196);
			assert (pack.target_system_GET() == (char) 113);
		});
		PARAM_REQUEST_LIST p21 = new PARAM_REQUEST_LIST();
		PH.setPack(p21);
		p21.target_system_SET((char) 113);
		p21.target_component_SET((char) 196);
		TestChannel.instance.send(p21);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_PARAM_VALUE.add((src, ph, pack) ->
		{
			assert (pack.param_type_GET() == MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT16);
			assert (pack.param_count_GET() == (char) 52849);
			assert (pack.param_id_LEN(ph) == 10);
			assert (pack.param_id_TRY(ph).equals("bvaqOvztug"));
			assert (pack.param_index_GET() == (char) 41348);
			assert (pack.param_value_GET() == -8.3227367E37F);
		});
		PARAM_VALUE p22 = new PARAM_VALUE();
		PH.setPack(p22);
		p22.param_count_SET((char) 52849);
		p22.param_type_SET(MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT16);
		p22.param_index_SET((char) 41348);
		p22.param_id_SET("bvaqOvztug", PH);
		p22.param_value_SET(-8.3227367E37F);
		TestChannel.instance.send(p22);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_PARAM_SET.add((src, ph, pack) ->
		{
			assert (pack.param_type_GET() == MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT64);
			assert (pack.target_system_GET() == (char) 6);
			assert (pack.param_id_LEN(ph) == 1);
			assert (pack.param_id_TRY(ph).equals("s"));
			assert (pack.param_value_GET() == 2.7586752E38F);
			assert (pack.target_component_GET() == (char) 242);
		});
		PARAM_SET p23 = new PARAM_SET();
		PH.setPack(p23);
		p23.param_value_SET(2.7586752E38F);
		p23.target_component_SET((char) 242);
		p23.target_system_SET((char) 6);
		p23.param_type_SET(MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT64);
		p23.param_id_SET("s", PH);
		TestChannel.instance.send(p23);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_GPS_RAW_INT.add((src, ph, pack) ->
		{
			assert (pack.lon_GET() == 1566467591);
			assert (pack.v_acc_TRY(ph) == 3552871365L);
			assert (pack.hdg_acc_TRY(ph) == 1920384235L);
			assert (pack.time_usec_GET() == 7274151646139192023L);
			assert (pack.alt_ellipsoid_TRY(ph) == -693642483);
			assert (pack.cog_GET() == (char) 8790);
			assert (pack.eph_GET() == (char) 40424);
			assert (pack.vel_GET() == (char) 43997);
			assert (pack.h_acc_TRY(ph) == 313647582L);
			assert (pack.lat_GET() == 1749645436);
			assert (pack.epv_GET() == (char) 34234);
			assert (pack.vel_acc_TRY(ph) == 2328415496L);
			assert (pack.satellites_visible_GET() == (char) 121);
			assert (pack.alt_GET() == -544309726);
			assert (pack.fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_DGPS);
		});
		GPS_RAW_INT p24 = new GPS_RAW_INT();
		PH.setPack(p24);
		p24.lat_SET(1749645436);
		p24.h_acc_SET(313647582L, PH);
		p24.alt_SET(-544309726);
		p24.hdg_acc_SET(1920384235L, PH);
		p24.time_usec_SET(7274151646139192023L);
		p24.vel_SET((char) 43997);
		p24.eph_SET((char) 40424);
		p24.epv_SET((char) 34234);
		p24.fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_DGPS);
		p24.v_acc_SET(3552871365L, PH);
		p24.alt_ellipsoid_SET(-693642483, PH);
		p24.cog_SET((char) 8790);
		p24.vel_acc_SET(2328415496L, PH);
		p24.satellites_visible_SET((char) 121);
		p24.lon_SET(1566467591);
		TestChannel.instance.send(p24);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_GPS_STATUS.add((src, ph, pack) ->
		{
			assert (Arrays.equals(pack.satellite_used_GET(), new char[]{(char) 26, (char) 37, (char) 169, (char) 165, (char) 13, (char) 141, (char) 61, (char) 131, (char) 161, (char) 154, (char) 179, (char) 26, (char) 114, (char) 158, (char) 67, (char) 66, (char) 64, (char) 222, (char) 106, (char) 105}));
			assert (Arrays.equals(pack.satellite_prn_GET(), new char[]{(char) 77, (char) 155, (char) 242, (char) 162, (char) 190, (char) 45, (char) 37, (char) 214, (char) 61, (char) 41, (char) 247, (char) 148, (char) 214, (char) 253, (char) 69, (char) 213, (char) 230, (char) 169, (char) 180, (char) 118}));
			assert (Arrays.equals(pack.satellite_azimuth_GET(), new char[]{(char) 13, (char) 56, (char) 162, (char) 239, (char) 236, (char) 248, (char) 54, (char) 215, (char) 56, (char) 28, (char) 78, (char) 201, (char) 28, (char) 137, (char) 243, (char) 141, (char) 209, (char) 233, (char) 17, (char) 226}));
			assert (Arrays.equals(pack.satellite_snr_GET(), new char[]{(char) 142, (char) 184, (char) 124, (char) 119, (char) 73, (char) 213, (char) 11, (char) 48, (char) 205, (char) 232, (char) 45, (char) 249, (char) 1, (char) 243, (char) 130, (char) 147, (char) 210, (char) 135, (char) 26, (char) 111}));
			assert (pack.satellites_visible_GET() == (char) 90);
			assert (Arrays.equals(pack.satellite_elevation_GET(), new char[]{(char) 117, (char) 182, (char) 145, (char) 82, (char) 171, (char) 205, (char) 72, (char) 88, (char) 22, (char) 70, (char) 132, (char) 41, (char) 227, (char) 97, (char) 5, (char) 99, (char) 239, (char) 14, (char) 171, (char) 55}));
		});
		GPS_STATUS p25 = new GPS_STATUS();
		PH.setPack(p25);
		p25.satellite_snr_SET(new char[]{(char) 142, (char) 184, (char) 124, (char) 119, (char) 73, (char) 213, (char) 11, (char) 48, (char) 205, (char) 232, (char) 45, (char) 249, (char) 1, (char) 243, (char) 130, (char) 147, (char) 210, (char) 135, (char) 26, (char) 111}, 0);
		p25.satellite_used_SET(new char[]{(char) 26, (char) 37, (char) 169, (char) 165, (char) 13, (char) 141, (char) 61, (char) 131, (char) 161, (char) 154, (char) 179, (char) 26, (char) 114, (char) 158, (char) 67, (char) 66, (char) 64, (char) 222, (char) 106, (char) 105}, 0);
		p25.satellites_visible_SET((char) 90);
		p25.satellite_azimuth_SET(new char[]{(char) 13, (char) 56, (char) 162, (char) 239, (char) 236, (char) 248, (char) 54, (char) 215, (char) 56, (char) 28, (char) 78, (char) 201, (char) 28, (char) 137, (char) 243, (char) 141, (char) 209, (char) 233, (char) 17, (char) 226}, 0);
		p25.satellite_elevation_SET(new char[]{(char) 117, (char) 182, (char) 145, (char) 82, (char) 171, (char) 205, (char) 72, (char) 88, (char) 22, (char) 70, (char) 132, (char) 41, (char) 227, (char) 97, (char) 5, (char) 99, (char) 239, (char) 14, (char) 171, (char) 55}, 0);
		p25.satellite_prn_SET(new char[]{(char) 77, (char) 155, (char) 242, (char) 162, (char) 190, (char) 45, (char) 37, (char) 214, (char) 61, (char) 41, (char) 247, (char) 148, (char) 214, (char) 253, (char) 69, (char) 213, (char) 230, (char) 169, (char) 180, (char) 118}, 0);
		TestChannel.instance.send(p25);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_SCALED_IMU.add((src, ph, pack) ->
		{
			assert (pack.time_boot_ms_GET() == 2292019334L);
			assert (pack.xacc_GET() == (short) -21094);
			assert (pack.yacc_GET() == (short) 23214);
			assert (pack.zmag_GET() == (short) -19770);
			assert (pack.ymag_GET() == (short) -14780);
			assert (pack.zgyro_GET() == (short) 4438);
			assert (pack.xmag_GET() == (short) 19749);
			assert (pack.xgyro_GET() == (short) -11566);
			assert (pack.zacc_GET() == (short) 16922);
			assert (pack.ygyro_GET() == (short) -6459);
		});
		SCALED_IMU p26 = new SCALED_IMU();
		PH.setPack(p26);
		p26.zmag_SET((short) -19770);
		p26.zgyro_SET((short) 4438);
		p26.xacc_SET((short) -21094);
		p26.xgyro_SET((short) -11566);
		p26.ymag_SET((short) -14780);
		p26.ygyro_SET((short) -6459);
		p26.zacc_SET((short) 16922);
		p26.time_boot_ms_SET(2292019334L);
		p26.xmag_SET((short) 19749);
		p26.yacc_SET((short) 23214);
		TestChannel.instance.send(p26);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_RAW_IMU.add((src, ph, pack) ->
		{
			assert (pack.zacc_GET() == (short) 20240);
			assert (pack.xacc_GET() == (short) 31229);
			assert (pack.yacc_GET() == (short) -10978);
			assert (pack.zmag_GET() == (short) -14305);
			assert (pack.zgyro_GET() == (short) 31998);
			assert (pack.xmag_GET() == (short) -4815);
			assert (pack.time_usec_GET() == 7316828052504902682L);
			assert (pack.ymag_GET() == (short) -14020);
			assert (pack.xgyro_GET() == (short) 21637);
			assert (pack.ygyro_GET() == (short) -11526);
		});
		RAW_IMU p27 = new RAW_IMU();
		PH.setPack(p27);
		p27.zacc_SET((short) 20240);
		p27.zgyro_SET((short) 31998);
		p27.ymag_SET((short) -14020);
		p27.yacc_SET((short) -10978);
		p27.xmag_SET((short) -4815);
		p27.xgyro_SET((short) 21637);
		p27.time_usec_SET(7316828052504902682L);
		p27.xacc_SET((short) 31229);
		p27.ygyro_SET((short) -11526);
		p27.zmag_SET((short) -14305);
		TestChannel.instance.send(p27);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_RAW_PRESSURE.add((src, ph, pack) ->
		{
			assert (pack.temperature_GET() == (short) -27656);
			assert (pack.press_diff1_GET() == (short) -27288);
			assert (pack.press_diff2_GET() == (short) -13027);
			assert (pack.time_usec_GET() == 1282132102624312428L);
			assert (pack.press_abs_GET() == (short) 5981);
		});
		RAW_PRESSURE p28 = new RAW_PRESSURE();
		PH.setPack(p28);
		p28.press_diff2_SET((short) -13027);
		p28.temperature_SET((short) -27656);
		p28.press_abs_SET((short) 5981);
		p28.press_diff1_SET((short) -27288);
		p28.time_usec_SET(1282132102624312428L);
		TestChannel.instance.send(p28);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_SCALED_PRESSURE.add((src, ph, pack) ->
		{
			assert (pack.press_diff_GET() == -7.5057257E37F);
			assert (pack.temperature_GET() == (short) -20084);
			assert (pack.press_abs_GET() == 1.3121093E37F);
			assert (pack.time_boot_ms_GET() == 1986516424L);
		});
		SCALED_PRESSURE p29 = new SCALED_PRESSURE();
		PH.setPack(p29);
		p29.press_diff_SET(-7.5057257E37F);
		p29.time_boot_ms_SET(1986516424L);
		p29.press_abs_SET(1.3121093E37F);
		p29.temperature_SET((short) -20084);
		TestChannel.instance.send(p29);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_ATTITUDE.add((src, ph, pack) ->
		{
			assert (pack.pitch_GET() == 1.9449346E38F);
			assert (pack.roll_GET() == -8.823062E37F);
			assert (pack.yawspeed_GET() == 9.31343E37F);
			assert (pack.yaw_GET() == 4.0166838E37F);
			assert (pack.pitchspeed_GET() == 1.4098016E38F);
			assert (pack.time_boot_ms_GET() == 2475757687L);
			assert (pack.rollspeed_GET() == 1.0916373E38F);
		});
		ATTITUDE p30 = new ATTITUDE();
		PH.setPack(p30);
		p30.roll_SET(-8.823062E37F);
		p30.yawspeed_SET(9.31343E37F);
		p30.pitchspeed_SET(1.4098016E38F);
		p30.rollspeed_SET(1.0916373E38F);
		p30.yaw_SET(4.0166838E37F);
		p30.time_boot_ms_SET(2475757687L);
		p30.pitch_SET(1.9449346E38F);
		TestChannel.instance.send(p30);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_ATTITUDE_QUATERNION.add((src, ph, pack) ->
		{
			assert (pack.q1_GET() == 1.7339681E38F);
			assert (pack.yawspeed_GET() == 2.7056373E38F);
			assert (pack.time_boot_ms_GET() == 2488610430L);
			assert (pack.q2_GET() == 1.2062584E38F);
			assert (pack.rollspeed_GET() == -6.8360685E37F);
			assert (pack.q3_GET() == -2.2617323E38F);
			assert (pack.pitchspeed_GET() == -2.6086112E38F);
			assert (pack.q4_GET() == -7.5778824E37F);
		});
		ATTITUDE_QUATERNION p31 = new ATTITUDE_QUATERNION();
		PH.setPack(p31);
		p31.rollspeed_SET(-6.8360685E37F);
		p31.q4_SET(-7.5778824E37F);
		p31.time_boot_ms_SET(2488610430L);
		p31.pitchspeed_SET(-2.6086112E38F);
		p31.q1_SET(1.7339681E38F);
		p31.yawspeed_SET(2.7056373E38F);
		p31.q2_SET(1.2062584E38F);
		p31.q3_SET(-2.2617323E38F);
		TestChannel.instance.send(p31);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_LOCAL_POSITION_NED.add((src, ph, pack) ->
		{
			assert (pack.z_GET() == 2.490055E38F);
			assert (pack.vz_GET() == -2.0785681E38F);
			assert (pack.vy_GET() == 4.633149E37F);
			assert (pack.x_GET() == 1.1904624E38F);
			assert (pack.time_boot_ms_GET() == 2340827291L);
			assert (pack.y_GET() == 6.8180937E37F);
			assert (pack.vx_GET() == -1.9531508E38F);
		});
		LOCAL_POSITION_NED p32 = new LOCAL_POSITION_NED();
		PH.setPack(p32);
		p32.y_SET(6.8180937E37F);
		p32.vy_SET(4.633149E37F);
		p32.time_boot_ms_SET(2340827291L);
		p32.z_SET(2.490055E38F);
		p32.vz_SET(-2.0785681E38F);
		p32.x_SET(1.1904624E38F);
		p32.vx_SET(-1.9531508E38F);
		TestChannel.instance.send(p32);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_GLOBAL_POSITION_INT.add((src, ph, pack) ->
		{
			assert (pack.lon_GET() == 116448565);
			assert (pack.lat_GET() == -774533156);
			assert (pack.vx_GET() == (short) 22289);
			assert (pack.hdg_GET() == (char) 35576);
			assert (pack.vy_GET() == (short) -29438);
			assert (pack.relative_alt_GET() == 511307481);
			assert (pack.time_boot_ms_GET() == 1277078939L);
			assert (pack.vz_GET() == (short) 379);
			assert (pack.alt_GET() == 453920202);
		});
		GLOBAL_POSITION_INT p33 = new GLOBAL_POSITION_INT();
		PH.setPack(p33);
		p33.alt_SET(453920202);
		p33.vz_SET((short) 379);
		p33.time_boot_ms_SET(1277078939L);
		p33.vx_SET((short) 22289);
		p33.lat_SET(-774533156);
		p33.relative_alt_SET(511307481);
		p33.lon_SET(116448565);
		p33.vy_SET((short) -29438);
		p33.hdg_SET((char) 35576);
		TestChannel.instance.send(p33);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_RC_CHANNELS_SCALED.add((src, ph, pack) ->
		{
			assert (pack.chan8_scaled_GET() == (short) 2946);
			assert (pack.time_boot_ms_GET() == 3270524367L);
			assert (pack.chan6_scaled_GET() == (short) -17270);
			assert (pack.chan1_scaled_GET() == (short) -8810);
			assert (pack.chan2_scaled_GET() == (short) 18889);
			assert (pack.rssi_GET() == (char) 58);
			assert (pack.chan4_scaled_GET() == (short) 25904);
			assert (pack.port_GET() == (char) 7);
			assert (pack.chan5_scaled_GET() == (short) -18173);
			assert (pack.chan3_scaled_GET() == (short) -3506);
			assert (pack.chan7_scaled_GET() == (short) -24665);
		});
		RC_CHANNELS_SCALED p34 = new RC_CHANNELS_SCALED();
		PH.setPack(p34);
		p34.chan6_scaled_SET((short) -17270);
		p34.port_SET((char) 7);
		p34.chan1_scaled_SET((short) -8810);
		p34.rssi_SET((char) 58);
		p34.chan4_scaled_SET((short) 25904);
		p34.chan8_scaled_SET((short) 2946);
		p34.chan5_scaled_SET((short) -18173);
		p34.time_boot_ms_SET(3270524367L);
		p34.chan2_scaled_SET((short) 18889);
		p34.chan7_scaled_SET((short) -24665);
		p34.chan3_scaled_SET((short) -3506);
		TestChannel.instance.send(p34);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_RC_CHANNELS_RAW.add((src, ph, pack) ->
		{
			assert (pack.chan8_raw_GET() == (char) 30608);
			assert (pack.chan7_raw_GET() == (char) 50771);
			assert (pack.chan2_raw_GET() == (char) 32145);
			assert (pack.time_boot_ms_GET() == 2459072830L);
			assert (pack.chan5_raw_GET() == (char) 20667);
			assert (pack.chan4_raw_GET() == (char) 31693);
			assert (pack.chan3_raw_GET() == (char) 23571);
			assert (pack.rssi_GET() == (char) 68);
			assert (pack.chan1_raw_GET() == (char) 56326);
			assert (pack.port_GET() == (char) 116);
			assert (pack.chan6_raw_GET() == (char) 16);
		});
		RC_CHANNELS_RAW p35 = new RC_CHANNELS_RAW();
		PH.setPack(p35);
		p35.port_SET((char) 116);
		p35.chan3_raw_SET((char) 23571);
		p35.chan2_raw_SET((char) 32145);
		p35.chan8_raw_SET((char) 30608);
		p35.time_boot_ms_SET(2459072830L);
		p35.chan1_raw_SET((char) 56326);
		p35.chan4_raw_SET((char) 31693);
		p35.chan5_raw_SET((char) 20667);
		p35.chan7_raw_SET((char) 50771);
		p35.chan6_raw_SET((char) 16);
		p35.rssi_SET((char) 68);
		TestChannel.instance.send(p35);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_SERVO_OUTPUT_RAW.add((src, ph, pack) ->
		{
			assert (pack.servo6_raw_GET() == (char) 35664);
			assert (pack.servo4_raw_GET() == (char) 18424);
			assert (pack.port_GET() == (char) 140);
			assert (pack.servo14_raw_TRY(ph) == (char) 60082);
			assert (pack.servo15_raw_TRY(ph) == (char) 18453);
			assert (pack.servo2_raw_GET() == (char) 33930);
			assert (pack.servo13_raw_TRY(ph) == (char) 28456);
			assert (pack.servo3_raw_GET() == (char) 2401);
			assert (pack.servo8_raw_GET() == (char) 37464);
			assert (pack.servo1_raw_GET() == (char) 2280);
			assert (pack.servo7_raw_GET() == (char) 58145);
			assert (pack.servo10_raw_TRY(ph) == (char) 7916);
			assert (pack.servo16_raw_TRY(ph) == (char) 46400);
			assert (pack.servo9_raw_TRY(ph) == (char) 26784);
			assert (pack.time_usec_GET() == 1499220827L);
			assert (pack.servo12_raw_TRY(ph) == (char) 3419);
			assert (pack.servo5_raw_GET() == (char) 30993);
			assert (pack.servo11_raw_TRY(ph) == (char) 19766);
		});
		SERVO_OUTPUT_RAW p36 = new SERVO_OUTPUT_RAW();
		PH.setPack(p36);
		p36.servo2_raw_SET((char) 33930);
		p36.servo7_raw_SET((char) 58145);
		p36.servo14_raw_SET((char) 60082, PH);
		p36.servo1_raw_SET((char) 2280);
		p36.servo3_raw_SET((char) 2401);
		p36.servo12_raw_SET((char) 3419, PH);
		p36.servo13_raw_SET((char) 28456, PH);
		p36.servo15_raw_SET((char) 18453, PH);
		p36.servo4_raw_SET((char) 18424);
		p36.servo6_raw_SET((char) 35664);
		p36.servo10_raw_SET((char) 7916, PH);
		p36.time_usec_SET(1499220827L);
		p36.servo5_raw_SET((char) 30993);
		p36.servo16_raw_SET((char) 46400, PH);
		p36.servo11_raw_SET((char) 19766, PH);
		p36.servo9_raw_SET((char) 26784, PH);
		p36.servo8_raw_SET((char) 37464);
		p36.port_SET((char) 140);
		TestChannel.instance.send(p36);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_MISSION_REQUEST_PARTIAL_LIST.add((src, ph, pack) ->
		{
			assert (pack.start_index_GET() == (short) -945);
			assert (pack.target_system_GET() == (char) 119);
			assert (pack.target_component_GET() == (char) 93);
			assert (pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
			assert (pack.end_index_GET() == (short) 20719);
		});
		MISSION_REQUEST_PARTIAL_LIST p37 = new MISSION_REQUEST_PARTIAL_LIST();
		PH.setPack(p37);
		p37.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
		p37.target_system_SET((char) 119);
		p37.end_index_SET((short) 20719);
		p37.start_index_SET((short) -945);
		p37.target_component_SET((char) 93);
		TestChannel.instance.send(p37);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_MISSION_WRITE_PARTIAL_LIST.add((src, ph, pack) ->
		{
			assert (pack.end_index_GET() == (short) -4195);
			assert (pack.target_component_GET() == (char) 56);
			assert (pack.target_system_GET() == (char) 73);
			assert (pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
			assert (pack.start_index_GET() == (short) -25812);
		});
		MISSION_WRITE_PARTIAL_LIST p38 = new MISSION_WRITE_PARTIAL_LIST();
		PH.setPack(p38);
		p38.target_component_SET((char) 56);
		p38.target_system_SET((char) 73);
		p38.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
		p38.start_index_SET((short) -25812);
		p38.end_index_SET((short) -4195);
		TestChannel.instance.send(p38);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_MISSION_ITEM.add((src, ph, pack) ->
		{
			assert (pack.z_GET() == 6.372499E37F);
			assert (pack.param1_GET() == -2.979684E38F);
			assert (pack.command_GET() == MAV_CMD.MAV_CMD_REQUEST_STORAGE_INFORMATION);
			assert (pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_INT);
			assert (pack.param4_GET() == -1.4397154E37F);
			assert (pack.current_GET() == (char) 32);
			assert (pack.param2_GET() == -6.739436E37F);
			assert (pack.seq_GET() == (char) 35624);
			assert (pack.y_GET() == 3.8025976E37F);
			assert (pack.target_system_GET() == (char) 182);
			assert (pack.param3_GET() == -6.085091E37F);
			assert (pack.autocontinue_GET() == (char) 186);
			assert (pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
			assert (pack.target_component_GET() == (char) 232);
			assert (pack.x_GET() == 5.4346813E37F);
		});
		MISSION_ITEM p39 = new MISSION_ITEM();
		PH.setPack(p39);
		p39.autocontinue_SET((char) 186);
		p39.z_SET(6.372499E37F);
		p39.param2_SET(-6.739436E37F);
		p39.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_INT);
		p39.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
		p39.x_SET(5.4346813E37F);
		p39.param1_SET(-2.979684E38F);
		p39.command_SET(MAV_CMD.MAV_CMD_REQUEST_STORAGE_INFORMATION);
		p39.param3_SET(-6.085091E37F);
		p39.y_SET(3.8025976E37F);
		p39.seq_SET((char) 35624);
		p39.param4_SET(-1.4397154E37F);
		p39.target_system_SET((char) 182);
		p39.current_SET((char) 32);
		p39.target_component_SET((char) 232);
		TestChannel.instance.send(p39);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_MISSION_REQUEST.add((src, ph, pack) ->
		{
			assert (pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
			assert (pack.seq_GET() == (char) 38081);
			assert (pack.target_system_GET() == (char) 252);
			assert (pack.target_component_GET() == (char) 44);
		});
		MISSION_REQUEST p40 = new MISSION_REQUEST();
		PH.setPack(p40);
		p40.seq_SET((char) 38081);
		p40.target_component_SET((char) 44);
		p40.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
		p40.target_system_SET((char) 252);
		TestChannel.instance.send(p40);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_MISSION_SET_CURRENT.add((src, ph, pack) ->
		{
			assert (pack.target_system_GET() == (char) 43);
			assert (pack.seq_GET() == (char) 46935);
			assert (pack.target_component_GET() == (char) 97);
		});
		MISSION_SET_CURRENT p41 = new MISSION_SET_CURRENT();
		PH.setPack(p41);
		p41.target_system_SET((char) 43);
		p41.target_component_SET((char) 97);
		p41.seq_SET((char) 46935);
		TestChannel.instance.send(p41);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_MISSION_CURRENT.add((src, ph, pack) ->
		{
			assert (pack.seq_GET() == (char) 1685);
		});
		MISSION_CURRENT p42 = new MISSION_CURRENT();
		PH.setPack(p42);
		p42.seq_SET((char) 1685);
		TestChannel.instance.send(p42);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_MISSION_REQUEST_LIST.add((src, ph, pack) ->
		{
			assert (pack.target_system_GET() == (char) 23);
			assert (pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
			assert (pack.target_component_GET() == (char) 191);
		});
		MISSION_REQUEST_LIST p43 = new MISSION_REQUEST_LIST();
		PH.setPack(p43);
		p43.target_component_SET((char) 191);
		p43.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
		p43.target_system_SET((char) 23);
		TestChannel.instance.send(p43);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_MISSION_COUNT.add((src, ph, pack) ->
		{
			assert (pack.target_component_GET() == (char) 72);
			assert (pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
			assert (pack.target_system_GET() == (char) 215);
			assert (pack.count_GET() == (char) 37804);
		});
		MISSION_COUNT p44 = new MISSION_COUNT();
		PH.setPack(p44);
		p44.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
		p44.target_system_SET((char) 215);
		p44.count_SET((char) 37804);
		p44.target_component_SET((char) 72);
		TestChannel.instance.send(p44);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_MISSION_CLEAR_ALL.add((src, ph, pack) ->
		{
			assert (pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
			assert (pack.target_component_GET() == (char) 219);
			assert (pack.target_system_GET() == (char) 230);
		});
		MISSION_CLEAR_ALL p45 = new MISSION_CLEAR_ALL();
		PH.setPack(p45);
		p45.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
		p45.target_component_SET((char) 219);
		p45.target_system_SET((char) 230);
		TestChannel.instance.send(p45);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_MISSION_ITEM_REACHED.add((src, ph, pack) ->
		{
			assert (pack.seq_GET() == (char) 46454);
		});
		MISSION_ITEM_REACHED p46 = new MISSION_ITEM_REACHED();
		PH.setPack(p46);
		p46.seq_SET((char) 46454);
		TestChannel.instance.send(p46);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_MISSION_ACK.add((src, ph, pack) ->
		{
			assert (pack.type_GET() == MAV_MISSION_RESULT.MAV_MISSION_INVALID);
			assert (pack.target_system_GET() == (char) 149);
			assert (pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
			assert (pack.target_component_GET() == (char) 127);
		});
		MISSION_ACK p47 = new MISSION_ACK();
		PH.setPack(p47);
		p47.type_SET(MAV_MISSION_RESULT.MAV_MISSION_INVALID);
		p47.target_system_SET((char) 149);
		p47.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
		p47.target_component_SET((char) 127);
		TestChannel.instance.send(p47);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_SET_GPS_GLOBAL_ORIGIN.add((src, ph, pack) ->
		{
			assert (pack.longitude_GET() == 1057776735);
			assert (pack.target_system_GET() == (char) 82);
			assert (pack.time_usec_TRY(ph) == 3872114051311421405L);
			assert (pack.altitude_GET() == 322095261);
			assert (pack.latitude_GET() == -2035133216);
		});
		SET_GPS_GLOBAL_ORIGIN p48 = new SET_GPS_GLOBAL_ORIGIN();
		PH.setPack(p48);
		p48.altitude_SET(322095261);
		p48.target_system_SET((char) 82);
		p48.latitude_SET(-2035133216);
		p48.time_usec_SET(3872114051311421405L, PH);
		p48.longitude_SET(1057776735);
		TestChannel.instance.send(p48);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_GPS_GLOBAL_ORIGIN.add((src, ph, pack) ->
		{
			assert (pack.altitude_GET() == 1061177962);
			assert (pack.latitude_GET() == 1201295683);
			assert (pack.time_usec_TRY(ph) == 7282649856823111179L);
			assert (pack.longitude_GET() == 981649504);
		});
		GPS_GLOBAL_ORIGIN p49 = new GPS_GLOBAL_ORIGIN();
		PH.setPack(p49);
		p49.altitude_SET(1061177962);
		p49.longitude_SET(981649504);
		p49.latitude_SET(1201295683);
		p49.time_usec_SET(7282649856823111179L, PH);
		TestChannel.instance.send(p49);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_PARAM_MAP_RC.add((src, ph, pack) ->
		{
			assert (pack.parameter_rc_channel_index_GET() == (char) 227);
			assert (pack.param_id_LEN(ph) == 3);
			assert (pack.param_id_TRY(ph).equals("qGs"));
			assert (pack.target_system_GET() == (char) 159);
			assert (pack.param_value_max_GET() == -4.5645614E37F);
			assert (pack.param_index_GET() == (short) 9010);
			assert (pack.param_value_min_GET() == 1.3524231E38F);
			assert (pack.target_component_GET() == (char) 231);
			assert (pack.param_value0_GET() == -2.8146922E38F);
			assert (pack.scale_GET() == 3.2907873E38F);
		});
		PARAM_MAP_RC p50 = new PARAM_MAP_RC();
		PH.setPack(p50);
		p50.parameter_rc_channel_index_SET((char) 227);
		p50.param_value0_SET(-2.8146922E38F);
		p50.target_component_SET((char) 231);
		p50.param_id_SET("qGs", PH);
		p50.param_index_SET((short) 9010);
		p50.param_value_max_SET(-4.5645614E37F);
		p50.param_value_min_SET(1.3524231E38F);
		p50.target_system_SET((char) 159);
		p50.scale_SET(3.2907873E38F);
		TestChannel.instance.send(p50);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_MISSION_REQUEST_INT.add((src, ph, pack) ->
		{
			assert (pack.target_component_GET() == (char) 204);
			assert (pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
			assert (pack.target_system_GET() == (char) 143);
			assert (pack.seq_GET() == (char) 59583);
		});
		MISSION_REQUEST_INT p51 = new MISSION_REQUEST_INT();
		PH.setPack(p51);
		p51.seq_SET((char) 59583);
		p51.target_system_SET((char) 143);
		p51.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
		p51.target_component_SET((char) 204);
		TestChannel.instance.send(p51);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_SAFETY_SET_ALLOWED_AREA.add((src, ph, pack) ->
		{
			assert (pack.p1y_GET() == -8.704449E37F);
			assert (pack.p1z_GET() == -8.4770235E37F);
			assert (pack.target_system_GET() == (char) 149);
			assert (pack.p2x_GET() == -1.00227E38F);
			assert (pack.p2z_GET() == -2.6084179E38F);
			assert (pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_INT);
			assert (pack.p1x_GET() == -1.2998366E38F);
			assert (pack.p2y_GET() == 8.652691E37F);
			assert (pack.target_component_GET() == (char) 27);
		});
		SAFETY_SET_ALLOWED_AREA p54 = new SAFETY_SET_ALLOWED_AREA();
		PH.setPack(p54);
		p54.target_component_SET((char) 27);
		p54.p1z_SET(-8.4770235E37F);
		p54.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_INT);
		p54.p1x_SET(-1.2998366E38F);
		p54.p2z_SET(-2.6084179E38F);
		p54.target_system_SET((char) 149);
		p54.p2y_SET(8.652691E37F);
		p54.p2x_SET(-1.00227E38F);
		p54.p1y_SET(-8.704449E37F);
		TestChannel.instance.send(p54);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_SAFETY_ALLOWED_AREA.add((src, ph, pack) ->
		{
			assert (pack.p2z_GET() == -1.2695839E38F);
			assert (pack.p2x_GET() == -1.2743032E38F);
			assert (pack.frame_GET() == MAV_FRAME.MAV_FRAME_MISSION);
			assert (pack.p1z_GET() == 3.1565642E38F);
			assert (pack.p1y_GET() == -4.567352E36F);
			assert (pack.p1x_GET() == -1.0069324E37F);
			assert (pack.p2y_GET() == -1.7921314E38F);
		});
		SAFETY_ALLOWED_AREA p55 = new SAFETY_ALLOWED_AREA();
		PH.setPack(p55);
		p55.p2z_SET(-1.2695839E38F);
		p55.p2x_SET(-1.2743032E38F);
		p55.frame_SET(MAV_FRAME.MAV_FRAME_MISSION);
		p55.p1z_SET(3.1565642E38F);
		p55.p1y_SET(-4.567352E36F);
		p55.p2y_SET(-1.7921314E38F);
		p55.p1x_SET(-1.0069324E37F);
		TestChannel.instance.send(p55);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_ATTITUDE_QUATERNION_COV.add((src, ph, pack) ->
		{
			assert (Arrays.equals(pack.q_GET(), new float[]{1.2553353E38F, 3.3977713E38F, -2.7762244E38F, -1.0609694E38F}));
			assert (pack.time_usec_GET() == 1646349488154943454L);
			assert (pack.yawspeed_GET() == 2.9835289E38F);
			assert (Arrays.equals(pack.covariance_GET(), new float[]{-3.061947E38F, -2.7400846E37F, 1.0768766E38F, -3.2442536E38F, 2.0593905E38F, 3.1148818E38F, -2.4615233E38F, 1.5907485E38F, 7.6917793E37F}));
			assert (pack.pitchspeed_GET() == 9.085035E37F);
			assert (pack.rollspeed_GET() == -7.744893E36F);
		});
		ATTITUDE_QUATERNION_COV p61 = new ATTITUDE_QUATERNION_COV();
		PH.setPack(p61);
		p61.q_SET(new float[]{1.2553353E38F, 3.3977713E38F, -2.7762244E38F, -1.0609694E38F}, 0);
		p61.time_usec_SET(1646349488154943454L);
		p61.covariance_SET(new float[]{-3.061947E38F, -2.7400846E37F, 1.0768766E38F, -3.2442536E38F, 2.0593905E38F, 3.1148818E38F, -2.4615233E38F, 1.5907485E38F, 7.6917793E37F}, 0);
		p61.rollspeed_SET(-7.744893E36F);
		p61.pitchspeed_SET(9.085035E37F);
		p61.yawspeed_SET(2.9835289E38F);
		TestChannel.instance.send(p61);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_NAV_CONTROLLER_OUTPUT.add((src, ph, pack) ->
		{
			assert (pack.nav_pitch_GET() == 1.1166947E37F);
			assert (pack.nav_bearing_GET() == (short) 24478);
			assert (pack.nav_roll_GET() == -2.3136654E38F);
			assert (pack.target_bearing_GET() == (short) 12826);
			assert (pack.xtrack_error_GET() == -2.4742511E38F);
			assert (pack.alt_error_GET() == 2.0034226E38F);
			assert (pack.aspd_error_GET() == -4.762073E37F);
			assert (pack.wp_dist_GET() == (char) 23407);
		});
		NAV_CONTROLLER_OUTPUT p62 = new NAV_CONTROLLER_OUTPUT();
		PH.setPack(p62);
		p62.target_bearing_SET((short) 12826);
		p62.nav_roll_SET(-2.3136654E38F);
		p62.nav_pitch_SET(1.1166947E37F);
		p62.wp_dist_SET((char) 23407);
		p62.aspd_error_SET(-4.762073E37F);
		p62.nav_bearing_SET((short) 24478);
		p62.xtrack_error_SET(-2.4742511E38F);
		p62.alt_error_SET(2.0034226E38F);
		TestChannel.instance.send(p62);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_GLOBAL_POSITION_INT_COV.add((src, ph, pack) ->
		{
			assert (pack.estimator_type_GET() == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS);
			assert (pack.relative_alt_GET() == 654023992);
			assert (pack.vx_GET() == -2.0031346E38F);
			assert (pack.lat_GET() == 554435320);
			assert (Arrays.equals(pack.covariance_GET(), new float[]{-3.5489187E37F, -1.19128E38F, -1.1496105E38F, 3.9431337E37F, 1.921522E38F, -5.77052E36F, -1.0664636E38F, -2.955255E38F, -1.5468908E38F, 1.5729332E38F, -2.3922845E38F, 3.1218644E38F, -2.5338056E38F, -1.3442003E38F, 2.5157636E38F, -1.5769497E38F, -5.223933E37F, -1.5771898E38F, 1.669519E38F, -1.616175E38F, -2.8199108E37F, -2.9114377E38F, 3.9995358E37F, 3.0606225E38F, 2.5173296E38F, -4.160062E36F, -7.851665E37F, -2.3971859E37F, -3.3221987E38F, -8.394377E37F, 1.8806978E38F, -1.678182E37F, -1.0139488E38F, 3.385075E38F, -7.769343E37F, 2.677208E38F}));
			assert (pack.alt_GET() == 2021318718);
			assert (pack.lon_GET() == -1360175637);
			assert (pack.vy_GET() == 1.7190713E38F);
			assert (pack.vz_GET() == 2.236358E38F);
			assert (pack.time_usec_GET() == 2731219561264200836L);
		});
		GLOBAL_POSITION_INT_COV p63 = new GLOBAL_POSITION_INT_COV();
		PH.setPack(p63);
		p63.lat_SET(554435320);
		p63.relative_alt_SET(654023992);
		p63.covariance_SET(new float[]{-3.5489187E37F, -1.19128E38F, -1.1496105E38F, 3.9431337E37F, 1.921522E38F, -5.77052E36F, -1.0664636E38F, -2.955255E38F, -1.5468908E38F, 1.5729332E38F, -2.3922845E38F, 3.1218644E38F, -2.5338056E38F, -1.3442003E38F, 2.5157636E38F, -1.5769497E38F, -5.223933E37F, -1.5771898E38F, 1.669519E38F, -1.616175E38F, -2.8199108E37F, -2.9114377E38F, 3.9995358E37F, 3.0606225E38F, 2.5173296E38F, -4.160062E36F, -7.851665E37F, -2.3971859E37F, -3.3221987E38F, -8.394377E37F, 1.8806978E38F, -1.678182E37F, -1.0139488E38F, 3.385075E38F, -7.769343E37F, 2.677208E38F}, 0);
		p63.lon_SET(-1360175637);
		p63.time_usec_SET(2731219561264200836L);
		p63.vz_SET(2.236358E38F);
		p63.vx_SET(-2.0031346E38F);
		p63.estimator_type_SET(MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS);
		p63.vy_SET(1.7190713E38F);
		p63.alt_SET(2021318718);
		TestChannel.instance.send(p63);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_LOCAL_POSITION_NED_COV.add((src, ph, pack) ->
		{
			assert (pack.ay_GET() == 2.2081244E38F);
			assert (pack.ax_GET() == 7.887544E37F);
			assert (pack.z_GET() == -1.0446231E38F);
			assert (Arrays.equals(pack.covariance_GET(), new float[]{1.4874743E38F, 1.3654004E38F, -2.683265E38F, 4.7106317E37F, 3.364057E38F, 2.382094E38F, -1.4087518E38F, 3.3381187E38F, -2.8838796E38F, -1.8532933E37F, 3.224822E38F, 2.8490267E38F, 1.5150967E38F, -9.429314E37F, 1.7459502E38F, -1.9444957E38F, 8.452964E37F, 3.2509969E38F, -1.1210295E38F, -9.139715E37F, -1.4181778E38F, -2.4682717E38F, -1.4708293E38F, 8.3044313E37F, 2.3156577E38F, 1.1365606E38F, -5.178111E37F, -2.0091132E38F, 1.9796244E38F, 2.9699076E37F, -2.2772232E38F, -2.6030635E38F, -2.32691E38F, -1.7959429E38F, 1.9528145E38F, -8.931356E37F, -2.6825077E38F, -8.789368E37F, -1.4558238E38F, 1.4451576E37F, 1.361607E38F, 1.0215869E38F, 1.5474251E38F, -4.1572437E37F, 2.7639685E38F}));
			assert (pack.estimator_type_GET() == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VISION);
			assert (pack.vx_GET() == -5.7698917E37F);
			assert (pack.az_GET() == -2.3624912E38F);
			assert (pack.vz_GET() == 1.2657787E38F);
			assert (pack.time_usec_GET() == 5905495033638209132L);
			assert (pack.vy_GET() == 5.3704636E37F);
			assert (pack.y_GET() == 5.049577E37F);
			assert (pack.x_GET() == -3.2230879E38F);
		});
		LOCAL_POSITION_NED_COV p64 = new LOCAL_POSITION_NED_COV();
		PH.setPack(p64);
		p64.ay_SET(2.2081244E38F);
		p64.estimator_type_SET(MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VISION);
		p64.z_SET(-1.0446231E38F);
		p64.x_SET(-3.2230879E38F);
		p64.az_SET(-2.3624912E38F);
		p64.vx_SET(-5.7698917E37F);
		p64.vz_SET(1.2657787E38F);
		p64.covariance_SET(new float[]{1.4874743E38F, 1.3654004E38F, -2.683265E38F, 4.7106317E37F, 3.364057E38F, 2.382094E38F, -1.4087518E38F, 3.3381187E38F, -2.8838796E38F, -1.8532933E37F, 3.224822E38F, 2.8490267E38F, 1.5150967E38F, -9.429314E37F, 1.7459502E38F, -1.9444957E38F, 8.452964E37F, 3.2509969E38F, -1.1210295E38F, -9.139715E37F, -1.4181778E38F, -2.4682717E38F, -1.4708293E38F, 8.3044313E37F, 2.3156577E38F, 1.1365606E38F, -5.178111E37F, -2.0091132E38F, 1.9796244E38F, 2.9699076E37F, -2.2772232E38F, -2.6030635E38F, -2.32691E38F, -1.7959429E38F, 1.9528145E38F, -8.931356E37F, -2.6825077E38F, -8.789368E37F, -1.4558238E38F, 1.4451576E37F, 1.361607E38F, 1.0215869E38F, 1.5474251E38F, -4.1572437E37F, 2.7639685E38F}, 0);
		p64.y_SET(5.049577E37F);
		p64.time_usec_SET(5905495033638209132L);
		p64.ax_SET(7.887544E37F);
		p64.vy_SET(5.3704636E37F);
		TestChannel.instance.send(p64);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_RC_CHANNELS.add((src, ph, pack) ->
		{
			assert (pack.chan12_raw_GET() == (char) 7326);
			assert (pack.chan14_raw_GET() == (char) 34163);
			assert (pack.chan7_raw_GET() == (char) 5537);
			assert (pack.chan15_raw_GET() == (char) 46731);
			assert (pack.chan2_raw_GET() == (char) 25921);
			assert (pack.chan3_raw_GET() == (char) 37685);
			assert (pack.chan18_raw_GET() == (char) 31020);
			assert (pack.chan6_raw_GET() == (char) 19557);
			assert (pack.chan5_raw_GET() == (char) 32530);
			assert (pack.chan16_raw_GET() == (char) 46642);
			assert (pack.chan11_raw_GET() == (char) 41974);
			assert (pack.chan4_raw_GET() == (char) 18810);
			assert (pack.chan9_raw_GET() == (char) 1889);
			assert (pack.chan17_raw_GET() == (char) 59652);
			assert (pack.time_boot_ms_GET() == 616369157L);
			assert (pack.chan10_raw_GET() == (char) 19591);
			assert (pack.chan8_raw_GET() == (char) 25441);
			assert (pack.chan1_raw_GET() == (char) 29407);
			assert (pack.chancount_GET() == (char) 214);
			assert (pack.rssi_GET() == (char) 140);
			assert (pack.chan13_raw_GET() == (char) 7583);
		});
		RC_CHANNELS p65 = new RC_CHANNELS();
		PH.setPack(p65);
		p65.chan11_raw_SET((char) 41974);
		p65.chan2_raw_SET((char) 25921);
		p65.chan7_raw_SET((char) 5537);
		p65.chan10_raw_SET((char) 19591);
		p65.chan6_raw_SET((char) 19557);
		p65.time_boot_ms_SET(616369157L);
		p65.chan3_raw_SET((char) 37685);
		p65.chan8_raw_SET((char) 25441);
		p65.chan9_raw_SET((char) 1889);
		p65.rssi_SET((char) 140);
		p65.chancount_SET((char) 214);
		p65.chan18_raw_SET((char) 31020);
		p65.chan12_raw_SET((char) 7326);
		p65.chan4_raw_SET((char) 18810);
		p65.chan16_raw_SET((char) 46642);
		p65.chan1_raw_SET((char) 29407);
		p65.chan15_raw_SET((char) 46731);
		p65.chan13_raw_SET((char) 7583);
		p65.chan5_raw_SET((char) 32530);
		p65.chan17_raw_SET((char) 59652);
		p65.chan14_raw_SET((char) 34163);
		TestChannel.instance.send(p65);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_REQUEST_DATA_STREAM.add((src, ph, pack) ->
		{
			assert (pack.start_stop_GET() == (char) 222);
			assert (pack.target_component_GET() == (char) 153);
			assert (pack.req_stream_id_GET() == (char) 94);
			assert (pack.target_system_GET() == (char) 76);
			assert (pack.req_message_rate_GET() == (char) 40651);
		});
		REQUEST_DATA_STREAM p66 = new REQUEST_DATA_STREAM();
		PH.setPack(p66);
		p66.target_system_SET((char) 76);
		p66.req_stream_id_SET((char) 94);
		p66.req_message_rate_SET((char) 40651);
		p66.start_stop_SET((char) 222);
		p66.target_component_SET((char) 153);
		TestChannel.instance.send(p66);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_DATA_STREAM.add((src, ph, pack) ->
		{
			assert (pack.on_off_GET() == (char) 8);
			assert (pack.stream_id_GET() == (char) 70);
			assert (pack.message_rate_GET() == (char) 8435);
		});
		DATA_STREAM p67 = new DATA_STREAM();
		PH.setPack(p67);
		p67.stream_id_SET((char) 70);
		p67.message_rate_SET((char) 8435);
		p67.on_off_SET((char) 8);
		TestChannel.instance.send(p67);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_MANUAL_CONTROL.add((src, ph, pack) ->
		{
			assert (pack.buttons_GET() == (char) 43532);
			assert (pack.z_GET() == (short) -18328);
			assert (pack.r_GET() == (short) -11622);
			assert (pack.target_GET() == (char) 115);
			assert (pack.x_GET() == (short) -409);
			assert (pack.y_GET() == (short) 24490);
		});
		MANUAL_CONTROL p69 = new MANUAL_CONTROL();
		PH.setPack(p69);
		p69.y_SET((short) 24490);
		p69.target_SET((char) 115);
		p69.z_SET((short) -18328);
		p69.r_SET((short) -11622);
		p69.x_SET((short) -409);
		p69.buttons_SET((char) 43532);
		TestChannel.instance.send(p69);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_RC_CHANNELS_OVERRIDE.add((src, ph, pack) ->
		{
			assert (pack.chan4_raw_GET() == (char) 31285);
			assert (pack.target_component_GET() == (char) 207);
			assert (pack.chan5_raw_GET() == (char) 12443);
			assert (pack.target_system_GET() == (char) 150);
			assert (pack.chan2_raw_GET() == (char) 2152);
			assert (pack.chan7_raw_GET() == (char) 30441);
			assert (pack.chan3_raw_GET() == (char) 59639);
			assert (pack.chan8_raw_GET() == (char) 49994);
			assert (pack.chan6_raw_GET() == (char) 31345);
			assert (pack.chan1_raw_GET() == (char) 28180);
		});
		RC_CHANNELS_OVERRIDE p70 = new RC_CHANNELS_OVERRIDE();
		PH.setPack(p70);
		p70.target_component_SET((char) 207);
		p70.chan2_raw_SET((char) 2152);
		p70.target_system_SET((char) 150);
		p70.chan3_raw_SET((char) 59639);
		p70.chan7_raw_SET((char) 30441);
		p70.chan8_raw_SET((char) 49994);
		p70.chan1_raw_SET((char) 28180);
		p70.chan5_raw_SET((char) 12443);
		p70.chan6_raw_SET((char) 31345);
		p70.chan4_raw_SET((char) 31285);
		TestChannel.instance.send(p70);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_MISSION_ITEM_INT.add((src, ph, pack) ->
		{
			assert (pack.autocontinue_GET() == (char) 100);
			assert (pack.y_GET() == -210271048);
			assert (pack.target_component_GET() == (char) 107);
			assert (pack.param2_GET() == -2.7377503E38F);
			assert (pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
			assert (pack.current_GET() == (char) 105);
			assert (pack.command_GET() == MAV_CMD.MAV_CMD_NAV_LOITER_UNLIM);
			assert (pack.param3_GET() == -1.7718218E38F);
			assert (pack.seq_GET() == (char) 21042);
			assert (pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL);
			assert (pack.param1_GET() == 3.1513315E38F);
			assert (pack.x_GET() == 248729659);
			assert (pack.z_GET() == -2.0458978E38F);
			assert (pack.target_system_GET() == (char) 229);
			assert (pack.param4_GET() == -2.5528163E38F);
		});
		MISSION_ITEM_INT p73 = new MISSION_ITEM_INT();
		PH.setPack(p73);
		p73.y_SET(-210271048);
		p73.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
		p73.x_SET(248729659);
		p73.param3_SET(-1.7718218E38F);
		p73.command_SET(MAV_CMD.MAV_CMD_NAV_LOITER_UNLIM);
		p73.current_SET((char) 105);
		p73.param1_SET(3.1513315E38F);
		p73.param4_SET(-2.5528163E38F);
		p73.target_system_SET((char) 229);
		p73.param2_SET(-2.7377503E38F);
		p73.target_component_SET((char) 107);
		p73.seq_SET((char) 21042);
		p73.z_SET(-2.0458978E38F);
		p73.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL);
		p73.autocontinue_SET((char) 100);
		TestChannel.instance.send(p73);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_VFR_HUD.add((src, ph, pack) ->
		{
			assert (pack.climb_GET() == -2.1694637E38F);
			assert (pack.airspeed_GET() == 9.795998E37F);
			assert (pack.throttle_GET() == (char) 8473);
			assert (pack.alt_GET() == 2.8044094E38F);
			assert (pack.groundspeed_GET() == 3.26011E38F);
			assert (pack.heading_GET() == (short) -18437);
		});
		VFR_HUD p74 = new VFR_HUD();
		PH.setPack(p74);
		p74.groundspeed_SET(3.26011E38F);
		p74.throttle_SET((char) 8473);
		p74.airspeed_SET(9.795998E37F);
		p74.heading_SET((short) -18437);
		p74.alt_SET(2.8044094E38F);
		p74.climb_SET(-2.1694637E38F);
		TestChannel.instance.send(p74);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_COMMAND_INT.add((src, ph, pack) ->
		{
			assert (pack.param1_GET() == -1.5939261E38F);
			assert (pack.param3_GET() == 1.2597045E38F);
			assert (pack.target_system_GET() == (char) 182);
			assert (pack.x_GET() == 735009222);
			assert (pack.current_GET() == (char) 165);
			assert (pack.command_GET() == MAV_CMD.MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL);
			assert (pack.target_component_GET() == (char) 96);
			assert (pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT);
			assert (pack.y_GET() == 105047596);
			assert (pack.autocontinue_GET() == (char) 118);
			assert (pack.param2_GET() == -1.9718631E37F);
			assert (pack.param4_GET() == 2.9795125E38F);
			assert (pack.z_GET() == 2.7549893E38F);
		});
		GroundControl.COMMAND_INT p75 = CommunicationChannel.new_COMMAND_INT();
		PH.setPack(p75);
		p75.current_SET((char) 165);
		p75.y_SET(105047596);
		p75.target_system_SET((char) 182);
		p75.x_SET(735009222);
		p75.z_SET(2.7549893E38F);
		p75.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT);
		p75.param2_SET(-1.9718631E37F);
		p75.param4_SET(2.9795125E38F);
		p75.param1_SET(-1.5939261E38F);
		p75.param3_SET(1.2597045E38F);
		p75.autocontinue_SET((char) 118);
		p75.target_component_SET((char) 96);
		p75.command_SET(MAV_CMD.MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL);
		CommunicationChannel.instance.send(p75);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_COMMAND_LONG.add((src, ph, pack) ->
		{
			assert (pack.param5_GET() == -3.3819267E38F);
			assert (pack.param7_GET() == -3.3936627E38F);
			assert (pack.target_component_GET() == (char) 225);
			assert (pack.command_GET() == MAV_CMD.MAV_CMD_DO_MOUNT_CONFIGURE);
			assert (pack.param1_GET() == 3.1819235E38F);
			assert (pack.confirmation_GET() == (char) 35);
			assert (pack.param4_GET() == -2.16896E38F);
			assert (pack.param6_GET() == 2.4628516E38F);
			assert (pack.target_system_GET() == (char) 162);
			assert (pack.param2_GET() == -2.9198808E38F);
			assert (pack.param3_GET() == -6.069851E37F);
		});
		GroundControl.COMMAND_LONG p76 = CommunicationChannel.new_COMMAND_LONG();
		PH.setPack(p76);
		p76.target_system_SET((char) 162);
		p76.param4_SET(-2.16896E38F);
		p76.target_component_SET((char) 225);
		p76.confirmation_SET((char) 35);
		p76.command_SET(MAV_CMD.MAV_CMD_DO_MOUNT_CONFIGURE);
		p76.param3_SET(-6.069851E37F);
		p76.param2_SET(-2.9198808E38F);
		p76.param7_SET(-3.3936627E38F);
		p76.param6_SET(2.4628516E38F);
		p76.param5_SET(-3.3819267E38F);
		p76.param1_SET(3.1819235E38F);
		CommunicationChannel.instance.send(p76);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_COMMAND_ACK.add((src, ph, pack) ->
		{
			assert (pack.result_param2_TRY(ph) == 1179105043);
			assert (pack.target_system_TRY(ph) == (char) 35);
			assert (pack.target_component_TRY(ph) == (char) 7);
			assert (pack.progress_TRY(ph) == (char) 53);
			assert (pack.command_GET() == MAV_CMD.MAV_CMD_NAV_VTOL_LAND);
			assert (pack.result_GET() == MAV_RESULT.MAV_RESULT_IN_PROGRESS);
		});
		GroundControl.COMMAND_ACK p77 = CommunicationChannel.new_COMMAND_ACK();
		PH.setPack(p77);
		p77.target_system_SET((char) 35, PH);
		p77.command_SET(MAV_CMD.MAV_CMD_NAV_VTOL_LAND);
		p77.result_param2_SET(1179105043, PH);
		p77.progress_SET((char) 53, PH);
		p77.result_SET(MAV_RESULT.MAV_RESULT_IN_PROGRESS);
		p77.target_component_SET((char) 7, PH);
		CommunicationChannel.instance.send(p77);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_MANUAL_SETPOINT.add((src, ph, pack) ->
		{
			assert (pack.mode_switch_GET() == (char) 192);
			assert (pack.yaw_GET() == -7.2258376E37F);
			assert (pack.pitch_GET() == 3.2612423E38F);
			assert (pack.manual_override_switch_GET() == (char) 200);
			assert (pack.roll_GET() == -2.0447158E38F);
			assert (pack.time_boot_ms_GET() == 1633141639L);
			assert (pack.thrust_GET() == -3.0166687E38F);
		});
		GroundControl.MANUAL_SETPOINT p81 = CommunicationChannel.new_MANUAL_SETPOINT();
		PH.setPack(p81);
		p81.mode_switch_SET((char) 192);
		p81.time_boot_ms_SET(1633141639L);
		p81.pitch_SET(3.2612423E38F);
		p81.yaw_SET(-7.2258376E37F);
		p81.manual_override_switch_SET((char) 200);
		p81.roll_SET(-2.0447158E38F);
		p81.thrust_SET(-3.0166687E38F);
		CommunicationChannel.instance.send(p81);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_SET_ATTITUDE_TARGET.add((src, ph, pack) ->
		{
			assert (pack.body_yaw_rate_GET() == 2.6152382E38F);
			assert (pack.thrust_GET() == -1.2779428E38F);
			assert (pack.body_pitch_rate_GET() == 2.8331222E38F);
			assert (pack.type_mask_GET() == (char) 228);
			assert (Arrays.equals(pack.q_GET(), new float[]{1.5126963E38F, 9.188835E37F, 1.8350515E37F, 9.666715E37F}));
			assert (pack.body_roll_rate_GET() == 1.34997E38F);
			assert (pack.target_component_GET() == (char) 129);
			assert (pack.time_boot_ms_GET() == 53133685L);
			assert (pack.target_system_GET() == (char) 148);
		});
		GroundControl.SET_ATTITUDE_TARGET p82 = CommunicationChannel.new_SET_ATTITUDE_TARGET();
		PH.setPack(p82);
		p82.body_pitch_rate_SET(2.8331222E38F);
		p82.thrust_SET(-1.2779428E38F);
		p82.q_SET(new float[]{1.5126963E38F, 9.188835E37F, 1.8350515E37F, 9.666715E37F}, 0);
		p82.target_system_SET((char) 148);
		p82.target_component_SET((char) 129);
		p82.body_yaw_rate_SET(2.6152382E38F);
		p82.type_mask_SET((char) 228);
		p82.body_roll_rate_SET(1.34997E38F);
		p82.time_boot_ms_SET(53133685L);
		CommunicationChannel.instance.send(p82);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_ATTITUDE_TARGET.add((src, ph, pack) ->
		{
			assert (Arrays.equals(pack.q_GET(), new float[]{2.8365053E38F, -7.026353E37F, -5.0968066E36F, 2.7643752E38F}));
			assert (pack.time_boot_ms_GET() == 1311371825L);
			assert (pack.body_yaw_rate_GET() == -8.053309E37F);
			assert (pack.type_mask_GET() == (char) 174);
			assert (pack.body_roll_rate_GET() == 2.2965718E38F);
			assert (pack.body_pitch_rate_GET() == -2.9343438E38F);
			assert (pack.thrust_GET() == -2.7576057E38F);
		});
		GroundControl.ATTITUDE_TARGET p83 = CommunicationChannel.new_ATTITUDE_TARGET();
		PH.setPack(p83);
		p83.body_pitch_rate_SET(-2.9343438E38F);
		p83.body_yaw_rate_SET(-8.053309E37F);
		p83.time_boot_ms_SET(1311371825L);
		p83.type_mask_SET((char) 174);
		p83.q_SET(new float[]{2.8365053E38F, -7.026353E37F, -5.0968066E36F, 2.7643752E38F}, 0);
		p83.thrust_SET(-2.7576057E38F);
		p83.body_roll_rate_SET(2.2965718E38F);
		CommunicationChannel.instance.send(p83);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_SET_POSITION_TARGET_LOCAL_NED.add((src, ph, pack) ->
		{
			assert (pack.vz_GET() == -3.2937404E38F);
			assert (pack.z_GET() == -1.4899646E38F);
			assert (pack.afy_GET() == 2.372574E37F);
			assert (pack.y_GET() == -7.5530694E37F);
			assert (pack.time_boot_ms_GET() == 2489795089L);
			assert (pack.target_component_GET() == (char) 47);
			assert (pack.yaw_rate_GET() == -5.7230094E37F);
			assert (pack.afz_GET() == 7.4390037E37F);
			assert (pack.x_GET() == -2.89377E38F);
			assert (pack.vx_GET() == 7.2040827E37F);
			assert (pack.afx_GET() == -1.5238189E38F);
			assert (pack.target_system_GET() == (char) 18);
			assert (pack.vy_GET() == -1.820032E37F);
			assert (pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_LOCAL_ENU);
			assert (pack.yaw_GET() == -2.8281812E38F);
			assert (pack.type_mask_GET() == (char) 59665);
		});
		GroundControl.SET_POSITION_TARGET_LOCAL_NED p84 = CommunicationChannel.new_SET_POSITION_TARGET_LOCAL_NED();
		PH.setPack(p84);
		p84.yaw_SET(-2.8281812E38F);
		p84.target_component_SET((char) 47);
		p84.afx_SET(-1.5238189E38F);
		p84.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_ENU);
		p84.y_SET(-7.5530694E37F);
		p84.z_SET(-1.4899646E38F);
		p84.vy_SET(-1.820032E37F);
		p84.vz_SET(-3.2937404E38F);
		p84.time_boot_ms_SET(2489795089L);
		p84.target_system_SET((char) 18);
		p84.x_SET(-2.89377E38F);
		p84.yaw_rate_SET(-5.7230094E37F);
		p84.vx_SET(7.2040827E37F);
		p84.type_mask_SET((char) 59665);
		p84.afy_SET(2.372574E37F);
		p84.afz_SET(7.4390037E37F);
		CommunicationChannel.instance.send(p84);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_SET_POSITION_TARGET_GLOBAL_INT.add((src, ph, pack) ->
		{
			assert (pack.type_mask_GET() == (char) 23005);
			assert (pack.alt_GET() == -3.3771135E38F);
			assert (pack.afx_GET() == -1.8165452E37F);
			assert (pack.afy_GET() == 3.8780198E37F);
			assert (pack.yaw_rate_GET() == -3.3989582E38F);
			assert (pack.vx_GET() == 2.9451077E38F);
			assert (pack.lat_int_GET() == -47370671);
			assert (pack.vy_GET() == 1.5857509E38F);
			assert (pack.target_component_GET() == (char) 169);
			assert (pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT);
			assert (pack.yaw_GET() == -1.5318602E38F);
			assert (pack.afz_GET() == -2.8568212E38F);
			assert (pack.target_system_GET() == (char) 75);
			assert (pack.vz_GET() == 1.6065534E38F);
			assert (pack.lon_int_GET() == -2132237030);
			assert (pack.time_boot_ms_GET() == 73773270L);
		});
		GroundControl.SET_POSITION_TARGET_GLOBAL_INT p86 = CommunicationChannel.new_SET_POSITION_TARGET_GLOBAL_INT();
		PH.setPack(p86);
		p86.lon_int_SET(-2132237030);
		p86.afy_SET(3.8780198E37F);
		p86.time_boot_ms_SET(73773270L);
		p86.yaw_SET(-1.5318602E38F);
		p86.afz_SET(-2.8568212E38F);
		p86.target_system_SET((char) 75);
		p86.target_component_SET((char) 169);
		p86.type_mask_SET((char) 23005);
		p86.afx_SET(-1.8165452E37F);
		p86.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT);
		p86.vy_SET(1.5857509E38F);
		p86.yaw_rate_SET(-3.3989582E38F);
		p86.vz_SET(1.6065534E38F);
		p86.alt_SET(-3.3771135E38F);
		p86.vx_SET(2.9451077E38F);
		p86.lat_int_SET(-47370671);
		CommunicationChannel.instance.send(p86);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_POSITION_TARGET_GLOBAL_INT.add((src, ph, pack) ->
		{
			assert (pack.vy_GET() == 5.225848E37F);
			assert (pack.vz_GET() == -3.0628108E38F);
			assert (pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_LOCAL_ENU);
			assert (pack.type_mask_GET() == (char) 36470);
			assert (pack.yaw_GET() == -1.940157E38F);
			assert (pack.time_boot_ms_GET() == 3363454648L);
			assert (pack.afx_GET() == -2.7992278E37F);
			assert (pack.lon_int_GET() == -1031953721);
			assert (pack.afy_GET() == 1.9141727E37F);
			assert (pack.vx_GET() == 3.3269699E38F);
			assert (pack.lat_int_GET() == -1577025819);
			assert (pack.yaw_rate_GET() == 1.9328076E38F);
			assert (pack.afz_GET() == 2.0802421E37F);
			assert (pack.alt_GET() == 1.2991426E38F);
		});
		GroundControl.POSITION_TARGET_GLOBAL_INT p87 = CommunicationChannel.new_POSITION_TARGET_GLOBAL_INT();
		PH.setPack(p87);
		p87.type_mask_SET((char) 36470);
		p87.vz_SET(-3.0628108E38F);
		p87.vx_SET(3.3269699E38F);
		p87.lon_int_SET(-1031953721);
		p87.vy_SET(5.225848E37F);
		p87.afz_SET(2.0802421E37F);
		p87.lat_int_SET(-1577025819);
		p87.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_ENU);
		p87.alt_SET(1.2991426E38F);
		p87.time_boot_ms_SET(3363454648L);
		p87.afy_SET(1.9141727E37F);
		p87.yaw_rate_SET(1.9328076E38F);
		p87.afx_SET(-2.7992278E37F);
		p87.yaw_SET(-1.940157E38F);
		CommunicationChannel.instance.send(p87);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET.add((src, ph, pack) ->
		{
			assert (pack.y_GET() == -2.124608E38F);
			assert (pack.x_GET() == 1.755222E38F);
			assert (pack.pitch_GET() == -1.6423336E38F);
			assert (pack.yaw_GET() == -2.8123576E37F);
			assert (pack.z_GET() == 9.968323E37F);
			assert (pack.time_boot_ms_GET() == 2045441408L);
			assert (pack.roll_GET() == -6.718138E37F);
		});
		GroundControl.LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET p89 = CommunicationChannel.new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
		PH.setPack(p89);
		p89.time_boot_ms_SET(2045441408L);
		p89.x_SET(1.755222E38F);
		p89.pitch_SET(-1.6423336E38F);
		p89.yaw_SET(-2.8123576E37F);
		p89.z_SET(9.968323E37F);
		p89.roll_SET(-6.718138E37F);
		p89.y_SET(-2.124608E38F);
		CommunicationChannel.instance.send(p89);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_HIL_STATE.add((src, ph, pack) ->
		{
			assert (pack.roll_GET() == -2.9450432E38F);
			assert (pack.time_usec_GET() == 7422771712401797513L);
			assert (pack.lon_GET() == 1613122150);
			assert (pack.alt_GET() == -1042435695);
			assert (pack.yawspeed_GET() == -2.4723594E38F);
			assert (pack.yaw_GET() == 1.496248E38F);
			assert (pack.vz_GET() == (short) 26508);
			assert (pack.vy_GET() == (short) 24253);
			assert (pack.zacc_GET() == (short) -26985);
			assert (pack.yacc_GET() == (short) 17299);
			assert (pack.pitch_GET() == -2.2026285E38F);
			assert (pack.lat_GET() == -98112776);
			assert (pack.vx_GET() == (short) -14372);
			assert (pack.xacc_GET() == (short) -11019);
			assert (pack.rollspeed_GET() == 3.3935718E38F);
			assert (pack.pitchspeed_GET() == -2.9976515E38F);
		});
		GroundControl.HIL_STATE p90 = CommunicationChannel.new_HIL_STATE();
		PH.setPack(p90);
		p90.vz_SET((short) 26508);
		p90.lat_SET(-98112776);
		p90.lon_SET(1613122150);
		p90.vy_SET((short) 24253);
		p90.yawspeed_SET(-2.4723594E38F);
		p90.yaw_SET(1.496248E38F);
		p90.yacc_SET((short) 17299);
		p90.zacc_SET((short) -26985);
		p90.rollspeed_SET(3.3935718E38F);
		p90.alt_SET(-1042435695);
		p90.time_usec_SET(7422771712401797513L);
		p90.pitchspeed_SET(-2.9976515E38F);
		p90.roll_SET(-2.9450432E38F);
		p90.xacc_SET((short) -11019);
		p90.pitch_SET(-2.2026285E38F);
		p90.vx_SET((short) -14372);
		CommunicationChannel.instance.send(p90);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_HIL_CONTROLS.add((src, ph, pack) ->
		{
			assert (pack.aux2_GET() == 1.7089108E38F);
			assert (pack.aux4_GET() == -2.9608174E38F);
			assert (pack.aux3_GET() == 1.9077154E38F);
			assert (pack.yaw_rudder_GET() == -1.1394324E38F);
			assert (pack.mode_GET() == MAV_MODE.MAV_MODE_GUIDED_ARMED);
			assert (pack.aux1_GET() == 1.2521457E38F);
			assert (pack.roll_ailerons_GET() == 3.2116423E38F);
			assert (pack.pitch_elevator_GET() == -3.1645212E38F);
			assert (pack.throttle_GET() == -1.5376416E38F);
			assert (pack.time_usec_GET() == 4337676077612418673L);
			assert (pack.nav_mode_GET() == (char) 87);
		});
		GroundControl.HIL_CONTROLS p91 = CommunicationChannel.new_HIL_CONTROLS();
		PH.setPack(p91);
		p91.pitch_elevator_SET(-3.1645212E38F);
		p91.nav_mode_SET((char) 87);
		p91.aux4_SET(-2.9608174E38F);
		p91.yaw_rudder_SET(-1.1394324E38F);
		p91.time_usec_SET(4337676077612418673L);
		p91.roll_ailerons_SET(3.2116423E38F);
		p91.throttle_SET(-1.5376416E38F);
		p91.aux3_SET(1.9077154E38F);
		p91.mode_SET(MAV_MODE.MAV_MODE_GUIDED_ARMED);
		p91.aux2_SET(1.7089108E38F);
		p91.aux1_SET(1.2521457E38F);
		CommunicationChannel.instance.send(p91);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_HIL_RC_INPUTS_RAW.add((src, ph, pack) ->
		{
			assert (pack.chan9_raw_GET() == (char) 10274);
			assert (pack.chan11_raw_GET() == (char) 31062);
			assert (pack.chan8_raw_GET() == (char) 57004);
			assert (pack.rssi_GET() == (char) 241);
			assert (pack.time_usec_GET() == 4677648175047292875L);
			assert (pack.chan12_raw_GET() == (char) 65390);
			assert (pack.chan6_raw_GET() == (char) 54741);
			assert (pack.chan2_raw_GET() == (char) 54229);
			assert (pack.chan1_raw_GET() == (char) 9799);
			assert (pack.chan3_raw_GET() == (char) 38636);
			assert (pack.chan10_raw_GET() == (char) 4179);
			assert (pack.chan5_raw_GET() == (char) 40615);
			assert (pack.chan7_raw_GET() == (char) 3304);
			assert (pack.chan4_raw_GET() == (char) 7450);
		});
		GroundControl.HIL_RC_INPUTS_RAW p92 = CommunicationChannel.new_HIL_RC_INPUTS_RAW();
		PH.setPack(p92);
		p92.rssi_SET((char) 241);
		p92.chan7_raw_SET((char) 3304);
		p92.chan6_raw_SET((char) 54741);
		p92.chan12_raw_SET((char) 65390);
		p92.chan10_raw_SET((char) 4179);
		p92.chan5_raw_SET((char) 40615);
		p92.chan8_raw_SET((char) 57004);
		p92.time_usec_SET(4677648175047292875L);
		p92.chan4_raw_SET((char) 7450);
		p92.chan9_raw_SET((char) 10274);
		p92.chan11_raw_SET((char) 31062);
		p92.chan3_raw_SET((char) 38636);
		p92.chan2_raw_SET((char) 54229);
		p92.chan1_raw_SET((char) 9799);
		CommunicationChannel.instance.send(p92);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_HIL_ACTUATOR_CONTROLS.add((src, ph, pack) ->
		{
			assert (pack.time_usec_GET() == 5972066705617159793L);
			assert (pack.flags_GET() == 5922561385644630356L);
			assert (pack.mode_GET() == MAV_MODE.MAV_MODE_STABILIZE_ARMED);
			assert (Arrays.equals(pack.controls_GET(), new float[]{-1.2267382E38F, 2.9078177E38F, -3.2490197E38F, -2.1968024E38F, -2.3109992E38F, -1.9104973E38F, -1.0964955E38F, 1.0562615E38F, 2.1923863E38F, -1.8871837E38F, 1.2715303E38F, -1.482213E38F, 3.2392913E38F, -1.2988408E38F, -2.9474495E38F, 1.0564781E38F}));
		});
		GroundControl.HIL_ACTUATOR_CONTROLS p93 = CommunicationChannel.new_HIL_ACTUATOR_CONTROLS();
		PH.setPack(p93);
		p93.flags_SET(5922561385644630356L);
		p93.controls_SET(new float[]{-1.2267382E38F, 2.9078177E38F, -3.2490197E38F, -2.1968024E38F, -2.3109992E38F, -1.9104973E38F, -1.0964955E38F, 1.0562615E38F, 2.1923863E38F, -1.8871837E38F, 1.2715303E38F, -1.482213E38F, 3.2392913E38F, -1.2988408E38F, -2.9474495E38F, 1.0564781E38F}, 0);
		p93.mode_SET(MAV_MODE.MAV_MODE_STABILIZE_ARMED);
		p93.time_usec_SET(5972066705617159793L);
		CommunicationChannel.instance.send(p93);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_OPTICAL_FLOW.add((src, ph, pack) ->
		{
			assert (pack.ground_distance_GET() == -2.7301642E38F);
			assert (pack.flow_rate_y_TRY(ph) == -2.7021488E38F);
			assert (pack.time_usec_GET() == 5808455151178324439L);
			assert (pack.flow_y_GET() == (short) -11414);
			assert (pack.flow_x_GET() == (short) -21286);
			assert (pack.flow_comp_m_x_GET() == 5.99603E37F);
			assert (pack.flow_rate_x_TRY(ph) == 1.6183898E38F);
			assert (pack.quality_GET() == (char) 178);
			assert (pack.sensor_id_GET() == (char) 162);
			assert (pack.flow_comp_m_y_GET() == 2.8166004E38F);
		});
		GroundControl.OPTICAL_FLOW p100 = CommunicationChannel.new_OPTICAL_FLOW();
		PH.setPack(p100);
		p100.flow_x_SET((short) -21286);
		p100.sensor_id_SET((char) 162);
		p100.time_usec_SET(5808455151178324439L);
		p100.flow_rate_y_SET(-2.7021488E38F, PH);
		p100.flow_comp_m_x_SET(5.99603E37F);
		p100.flow_comp_m_y_SET(2.8166004E38F);
		p100.flow_y_SET((short) -11414);
		p100.flow_rate_x_SET(1.6183898E38F, PH);
		p100.quality_SET((char) 178);
		p100.ground_distance_SET(-2.7301642E38F);
		CommunicationChannel.instance.send(p100);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_GLOBAL_VISION_POSITION_ESTIMATE.add((src, ph, pack) ->
		{
			assert (pack.y_GET() == 2.1560224E38F);
			assert (pack.x_GET() == -7.361147E37F);
			assert (pack.z_GET() == -9.167896E37F);
			assert (pack.pitch_GET() == 2.9989108E38F);
			assert (pack.yaw_GET() == 3.3436105E37F);
			assert (pack.usec_GET() == 5583658289113302233L);
			assert (pack.roll_GET() == -6.7587996E37F);
		});
		GroundControl.GLOBAL_VISION_POSITION_ESTIMATE p101 = CommunicationChannel.new_GLOBAL_VISION_POSITION_ESTIMATE();
		PH.setPack(p101);
		p101.roll_SET(-6.7587996E37F);
		p101.z_SET(-9.167896E37F);
		p101.yaw_SET(3.3436105E37F);
		p101.pitch_SET(2.9989108E38F);
		p101.x_SET(-7.361147E37F);
		p101.usec_SET(5583658289113302233L);
		p101.y_SET(2.1560224E38F);
		CommunicationChannel.instance.send(p101);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_VISION_POSITION_ESTIMATE.add((src, ph, pack) ->
		{
			assert (pack.x_GET() == -1.1669869E38F);
			assert (pack.pitch_GET() == -7.339713E36F);
			assert (pack.z_GET() == 7.551729E37F);
			assert (pack.usec_GET() == 2564674435673428748L);
			assert (pack.yaw_GET() == -2.0927741E38F);
			assert (pack.y_GET() == 2.552804E38F);
			assert (pack.roll_GET() == 1.94154E38F);
		});
		GroundControl.VISION_POSITION_ESTIMATE p102 = CommunicationChannel.new_VISION_POSITION_ESTIMATE();
		PH.setPack(p102);
		p102.z_SET(7.551729E37F);
		p102.pitch_SET(-7.339713E36F);
		p102.yaw_SET(-2.0927741E38F);
		p102.y_SET(2.552804E38F);
		p102.x_SET(-1.1669869E38F);
		p102.roll_SET(1.94154E38F);
		p102.usec_SET(2564674435673428748L);
		CommunicationChannel.instance.send(p102);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_VISION_SPEED_ESTIMATE.add((src, ph, pack) ->
		{
			assert (pack.y_GET() == -3.090222E38F);
			assert (pack.z_GET() == -1.4348422E38F);
			assert (pack.x_GET() == -2.2513783E38F);
			assert (pack.usec_GET() == 6776099632520324699L);
		});
		GroundControl.VISION_SPEED_ESTIMATE p103 = CommunicationChannel.new_VISION_SPEED_ESTIMATE();
		PH.setPack(p103);
		p103.x_SET(-2.2513783E38F);
		p103.z_SET(-1.4348422E38F);
		p103.y_SET(-3.090222E38F);
		p103.usec_SET(6776099632520324699L);
		CommunicationChannel.instance.send(p103);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_VICON_POSITION_ESTIMATE.add((src, ph, pack) ->
		{
			assert (pack.x_GET() == -2.1977482E38F);
			assert (pack.yaw_GET() == -1.3135731E38F);
			assert (pack.roll_GET() == -2.6144436E38F);
			assert (pack.z_GET() == -1.9329426E38F);
			assert (pack.usec_GET() == 7782513144418588969L);
			assert (pack.pitch_GET() == -1.790066E38F);
			assert (pack.y_GET() == -1.0066447E38F);
		});
		GroundControl.VICON_POSITION_ESTIMATE p104 = CommunicationChannel.new_VICON_POSITION_ESTIMATE();
		PH.setPack(p104);
		p104.y_SET(-1.0066447E38F);
		p104.roll_SET(-2.6144436E38F);
		p104.pitch_SET(-1.790066E38F);
		p104.yaw_SET(-1.3135731E38F);
		p104.usec_SET(7782513144418588969L);
		p104.x_SET(-2.1977482E38F);
		p104.z_SET(-1.9329426E38F);
		CommunicationChannel.instance.send(p104);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_HIGHRES_IMU.add((src, ph, pack) ->
		{
			assert (pack.zmag_GET() == 3.373175E38F);
			assert (pack.pressure_alt_GET() == -2.2338379E38F);
			assert (pack.temperature_GET() == -3.7511739E37F);
			assert (pack.ymag_GET() == -2.9070863E38F);
			assert (pack.xacc_GET() == -7.491112E37F);
			assert (pack.ygyro_GET() == 2.977726E36F);
			assert (pack.fields_updated_GET() == (char) 3218);
			assert (pack.diff_pressure_GET() == -2.9370081E38F);
			assert (pack.zgyro_GET() == 1.1535564E38F);
			assert (pack.xmag_GET() == 2.0438554E38F);
			assert (pack.yacc_GET() == 5.7755315E36F);
			assert (pack.abs_pressure_GET() == -1.3950677E38F);
			assert (pack.xgyro_GET() == -2.4963833E38F);
			assert (pack.time_usec_GET() == 7629202319849231934L);
			assert (pack.zacc_GET() == -2.1540548E38F);
		});
		GroundControl.HIGHRES_IMU p105 = CommunicationChannel.new_HIGHRES_IMU();
		PH.setPack(p105);
		p105.diff_pressure_SET(-2.9370081E38F);
		p105.fields_updated_SET((char) 3218);
		p105.zmag_SET(3.373175E38F);
		p105.ymag_SET(-2.9070863E38F);
		p105.xgyro_SET(-2.4963833E38F);
		p105.xmag_SET(2.0438554E38F);
		p105.zgyro_SET(1.1535564E38F);
		p105.zacc_SET(-2.1540548E38F);
		p105.abs_pressure_SET(-1.3950677E38F);
		p105.time_usec_SET(7629202319849231934L);
		p105.temperature_SET(-3.7511739E37F);
		p105.xacc_SET(-7.491112E37F);
		p105.yacc_SET(5.7755315E36F);
		p105.ygyro_SET(2.977726E36F);
		p105.pressure_alt_SET(-2.2338379E38F);
		CommunicationChannel.instance.send(p105);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_OPTICAL_FLOW_RAD.add((src, ph, pack) ->
		{
			assert (pack.temperature_GET() == (short) 7309);
			assert (pack.integration_time_us_GET() == 114896941L);
			assert (pack.integrated_ygyro_GET() == -2.317437E38F);
			assert (pack.integrated_zgyro_GET() == -8.95649E36F);
			assert (pack.distance_GET() == 9.110888E37F);
			assert (pack.sensor_id_GET() == (char) 242);
			assert (pack.time_delta_distance_us_GET() == 2499904815L);
			assert (pack.quality_GET() == (char) 125);
			assert (pack.integrated_xgyro_GET() == 2.77442E38F);
			assert (pack.integrated_x_GET() == -2.0968927E38F);
			assert (pack.integrated_y_GET() == -1.8972466E38F);
			assert (pack.time_usec_GET() == 7254184252152601543L);
		});
		GroundControl.OPTICAL_FLOW_RAD p106 = CommunicationChannel.new_OPTICAL_FLOW_RAD();
		PH.setPack(p106);
		p106.time_delta_distance_us_SET(2499904815L);
		p106.time_usec_SET(7254184252152601543L);
		p106.distance_SET(9.110888E37F);
		p106.sensor_id_SET((char) 242);
		p106.integrated_xgyro_SET(2.77442E38F);
		p106.integrated_zgyro_SET(-8.95649E36F);
		p106.integration_time_us_SET(114896941L);
		p106.quality_SET((char) 125);
		p106.integrated_ygyro_SET(-2.317437E38F);
		p106.temperature_SET((short) 7309);
		p106.integrated_y_SET(-1.8972466E38F);
		p106.integrated_x_SET(-2.0968927E38F);
		CommunicationChannel.instance.send(p106);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_HIL_SENSOR.add((src, ph, pack) ->
		{
			assert (pack.temperature_GET() == -2.2628198E38F);
			assert (pack.time_usec_GET() == 4518145528576171566L);
			assert (pack.ymag_GET() == 2.8835978E38F);
			assert (pack.xgyro_GET() == -1.3438146E38F);
			assert (pack.zgyro_GET() == -2.3870455E38F);
			assert (pack.abs_pressure_GET() == -1.748184E38F);
			assert (pack.ygyro_GET() == -1.8922072E38F);
			assert (pack.zacc_GET() == -3.0642896E38F);
			assert (pack.xmag_GET() == 2.738059E38F);
			assert (pack.yacc_GET() == 2.8997665E37F);
			assert (pack.diff_pressure_GET() == 2.1219321E38F);
			assert (pack.fields_updated_GET() == 2286171879L);
			assert (pack.xacc_GET() == 2.6204946E38F);
			assert (pack.pressure_alt_GET() == -3.299128E37F);
			assert (pack.zmag_GET() == 1.3191079E38F);
		});
		GroundControl.HIL_SENSOR p107 = CommunicationChannel.new_HIL_SENSOR();
		PH.setPack(p107);
		p107.ymag_SET(2.8835978E38F);
		p107.xacc_SET(2.6204946E38F);
		p107.pressure_alt_SET(-3.299128E37F);
		p107.xmag_SET(2.738059E38F);
		p107.diff_pressure_SET(2.1219321E38F);
		p107.xgyro_SET(-1.3438146E38F);
		p107.ygyro_SET(-1.8922072E38F);
		p107.yacc_SET(2.8997665E37F);
		p107.time_usec_SET(4518145528576171566L);
		p107.temperature_SET(-2.2628198E38F);
		p107.zacc_SET(-3.0642896E38F);
		p107.abs_pressure_SET(-1.748184E38F);
		p107.fields_updated_SET(2286171879L);
		p107.zgyro_SET(-2.3870455E38F);
		p107.zmag_SET(1.3191079E38F);
		CommunicationChannel.instance.send(p107);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_SIM_STATE.add((src, ph, pack) ->
		{
			assert (pack.vd_GET() == -2.4829896E38F);
			assert (pack.q1_GET() == -2.2094367E38F);
			assert (pack.lat_GET() == -2.2664506E38F);
			assert (pack.std_dev_horz_GET() == 3.4477206E37F);
			assert (pack.zgyro_GET() == 2.637897E38F);
			assert (pack.xgyro_GET() == 2.4748243E38F);
			assert (pack.xacc_GET() == -8.917625E37F);
			assert (pack.ve_GET() == -7.9210157E37F);
			assert (pack.pitch_GET() == 6.3500437E37F);
			assert (pack.q3_GET() == -2.2192923E38F);
			assert (pack.std_dev_vert_GET() == 1.6368803E38F);
			assert (pack.roll_GET() == 8.3402404E37F);
			assert (pack.zacc_GET() == 2.3886922E38F);
			assert (pack.yaw_GET() == 2.7993263E38F);
			assert (pack.vn_GET() == -3.00274E37F);
			assert (pack.q4_GET() == 7.7579436E37F);
			assert (pack.ygyro_GET() == -2.5531686E38F);
			assert (pack.q2_GET() == 3.381885E37F);
			assert (pack.lon_GET() == -3.1613877E38F);
			assert (pack.yacc_GET() == 2.1758133E38F);
			assert (pack.alt_GET() == 2.408545E37F);
		});
		GroundControl.SIM_STATE p108 = CommunicationChannel.new_SIM_STATE();
		PH.setPack(p108);
		p108.ve_SET(-7.9210157E37F);
		p108.std_dev_vert_SET(1.6368803E38F);
		p108.zacc_SET(2.3886922E38F);
		p108.pitch_SET(6.3500437E37F);
		p108.q2_SET(3.381885E37F);
		p108.yacc_SET(2.1758133E38F);
		p108.ygyro_SET(-2.5531686E38F);
		p108.vd_SET(-2.4829896E38F);
		p108.vn_SET(-3.00274E37F);
		p108.alt_SET(2.408545E37F);
		p108.roll_SET(8.3402404E37F);
		p108.yaw_SET(2.7993263E38F);
		p108.xacc_SET(-8.917625E37F);
		p108.lat_SET(-2.2664506E38F);
		p108.xgyro_SET(2.4748243E38F);
		p108.std_dev_horz_SET(3.4477206E37F);
		p108.zgyro_SET(2.637897E38F);
		p108.q1_SET(-2.2094367E38F);
		p108.q3_SET(-2.2192923E38F);
		p108.q4_SET(7.7579436E37F);
		p108.lon_SET(-3.1613877E38F);
		CommunicationChannel.instance.send(p108);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_RADIO_STATUS.add((src, ph, pack) ->
		{
			assert (pack.rxerrors_GET() == (char) 22016);
			assert (pack.txbuf_GET() == (char) 64);
			assert (pack.rssi_GET() == (char) 170);
			assert (pack.noise_GET() == (char) 252);
			assert (pack.remrssi_GET() == (char) 75);
			assert (pack.remnoise_GET() == (char) 252);
			assert (pack.fixed__GET() == (char) 15357);
		});
		GroundControl.RADIO_STATUS p109 = CommunicationChannel.new_RADIO_STATUS();
		PH.setPack(p109);
		p109.fixed__SET((char) 15357);
		p109.rxerrors_SET((char) 22016);
		p109.rssi_SET((char) 170);
		p109.remnoise_SET((char) 252);
		p109.noise_SET((char) 252);
		p109.txbuf_SET((char) 64);
		p109.remrssi_SET((char) 75);
		CommunicationChannel.instance.send(p109);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_FILE_TRANSFER_PROTOCOL.add((src, ph, pack) ->
		{
			assert (pack.target_network_GET() == (char) 184);
			assert (pack.target_system_GET() == (char) 163);
			assert (Arrays.equals(pack.payload_GET(), new char[]{(char) 233, (char) 245, (char) 19, (char) 20, (char) 66, (char) 157, (char) 216, (char) 217, (char) 12, (char) 212, (char) 161, (char) 254, (char) 121, (char) 139, (char) 219, (char) 4, (char) 69, (char) 242, (char) 69, (char) 12, (char) 88, (char) 0, (char) 158, (char) 188, (char) 32, (char) 176, (char) 166, (char) 57, (char) 7, (char) 125, (char) 221, (char) 245, (char) 140, (char) 2, (char) 186, (char) 233, (char) 116, (char) 161, (char) 135, (char) 152, (char) 244, (char) 102, (char) 228, (char) 80, (char) 49, (char) 146, (char) 121, (char) 154, (char) 120, (char) 34, (char) 130, (char) 52, (char) 148, (char) 23, (char) 97, (char) 47, (char) 124, (char) 97, (char) 167, (char) 180, (char) 103, (char) 25, (char) 229, (char) 242, (char) 61, (char) 252, (char) 71, (char) 109, (char) 13, (char) 149, (char) 85, (char) 20, (char) 127, (char) 36, (char) 35, (char) 237, (char) 190, (char) 227, (char) 49, (char) 75, (char) 224, (char) 124, (char) 88, (char) 122, (char) 129, (char) 186, (char) 166, (char) 122, (char) 31, (char) 204, (char) 247, (char) 206, (char) 57, (char) 229, (char) 226, (char) 123, (char) 181, (char) 185, (char) 251, (char) 231, (char) 77, (char) 67, (char) 196, (char) 207, (char) 116, (char) 30, (char) 105, (char) 102, (char) 28, (char) 251, (char) 18, (char) 180, (char) 135, (char) 148, (char) 171, (char) 144, (char) 108, (char) 229, (char) 7, (char) 19, (char) 114, (char) 46, (char) 47, (char) 137, (char) 191, (char) 252, (char) 231, (char) 255, (char) 8, (char) 73, (char) 171, (char) 143, (char) 158, (char) 231, (char) 195, (char) 136, (char) 7, (char) 190, (char) 250, (char) 175, (char) 229, (char) 200, (char) 103, (char) 192, (char) 230, (char) 236, (char) 92, (char) 203, (char) 87, (char) 165, (char) 56, (char) 151, (char) 30, (char) 193, (char) 48, (char) 30, (char) 182, (char) 155, (char) 247, (char) 203, (char) 154, (char) 182, (char) 57, (char) 60, (char) 32, (char) 145, (char) 219, (char) 235, (char) 146, (char) 154, (char) 114, (char) 184, (char) 41, (char) 246, (char) 135, (char) 30, (char) 213, (char) 157, (char) 129, (char) 146, (char) 44, (char) 187, (char) 11, (char) 255, (char) 121, (char) 199, (char) 73, (char) 96, (char) 81, (char) 217, (char) 93, (char) 167, (char) 181, (char) 62, (char) 1, (char) 136, (char) 3, (char) 196, (char) 126, (char) 184, (char) 63, (char) 201, (char) 13, (char) 32, (char) 180, (char) 194, (char) 127, (char) 152, (char) 46, (char) 190, (char) 245, (char) 126, (char) 211, (char) 104, (char) 45, (char) 59, (char) 54, (char) 133, (char) 239, (char) 3, (char) 118, (char) 200, (char) 202, (char) 16, (char) 131, (char) 24, (char) 96, (char) 167, (char) 208, (char) 75, (char) 103, (char) 59, (char) 205, (char) 94, (char) 193, (char) 107, (char) 168, (char) 166, (char) 97, (char) 189, (char) 207, (char) 137, (char) 164, (char) 146, (char) 8, (char) 25, (char) 51, (char) 91, (char) 172, (char) 198, (char) 119}));
			assert (pack.target_component_GET() == (char) 205);
		});
		GroundControl.FILE_TRANSFER_PROTOCOL p110 = CommunicationChannel.new_FILE_TRANSFER_PROTOCOL();
		PH.setPack(p110);
		p110.payload_SET(new char[]{(char) 233, (char) 245, (char) 19, (char) 20, (char) 66, (char) 157, (char) 216, (char) 217, (char) 12, (char) 212, (char) 161, (char) 254, (char) 121, (char) 139, (char) 219, (char) 4, (char) 69, (char) 242, (char) 69, (char) 12, (char) 88, (char) 0, (char) 158, (char) 188, (char) 32, (char) 176, (char) 166, (char) 57, (char) 7, (char) 125, (char) 221, (char) 245, (char) 140, (char) 2, (char) 186, (char) 233, (char) 116, (char) 161, (char) 135, (char) 152, (char) 244, (char) 102, (char) 228, (char) 80, (char) 49, (char) 146, (char) 121, (char) 154, (char) 120, (char) 34, (char) 130, (char) 52, (char) 148, (char) 23, (char) 97, (char) 47, (char) 124, (char) 97, (char) 167, (char) 180, (char) 103, (char) 25, (char) 229, (char) 242, (char) 61, (char) 252, (char) 71, (char) 109, (char) 13, (char) 149, (char) 85, (char) 20, (char) 127, (char) 36, (char) 35, (char) 237, (char) 190, (char) 227, (char) 49, (char) 75, (char) 224, (char) 124, (char) 88, (char) 122, (char) 129, (char) 186, (char) 166, (char) 122, (char) 31, (char) 204, (char) 247, (char) 206, (char) 57, (char) 229, (char) 226, (char) 123, (char) 181, (char) 185, (char) 251, (char) 231, (char) 77, (char) 67, (char) 196, (char) 207, (char) 116, (char) 30, (char) 105, (char) 102, (char) 28, (char) 251, (char) 18, (char) 180, (char) 135, (char) 148, (char) 171, (char) 144, (char) 108, (char) 229, (char) 7, (char) 19, (char) 114, (char) 46, (char) 47, (char) 137, (char) 191, (char) 252, (char) 231, (char) 255, (char) 8, (char) 73, (char) 171, (char) 143, (char) 158, (char) 231, (char) 195, (char) 136, (char) 7, (char) 190, (char) 250, (char) 175, (char) 229, (char) 200, (char) 103, (char) 192, (char) 230, (char) 236, (char) 92, (char) 203, (char) 87, (char) 165, (char) 56, (char) 151, (char) 30, (char) 193, (char) 48, (char) 30, (char) 182, (char) 155, (char) 247, (char) 203, (char) 154, (char) 182, (char) 57, (char) 60, (char) 32, (char) 145, (char) 219, (char) 235, (char) 146, (char) 154, (char) 114, (char) 184, (char) 41, (char) 246, (char) 135, (char) 30, (char) 213, (char) 157, (char) 129, (char) 146, (char) 44, (char) 187, (char) 11, (char) 255, (char) 121, (char) 199, (char) 73, (char) 96, (char) 81, (char) 217, (char) 93, (char) 167, (char) 181, (char) 62, (char) 1, (char) 136, (char) 3, (char) 196, (char) 126, (char) 184, (char) 63, (char) 201, (char) 13, (char) 32, (char) 180, (char) 194, (char) 127, (char) 152, (char) 46, (char) 190, (char) 245, (char) 126, (char) 211, (char) 104, (char) 45, (char) 59, (char) 54, (char) 133, (char) 239, (char) 3, (char) 118, (char) 200, (char) 202, (char) 16, (char) 131, (char) 24, (char) 96, (char) 167, (char) 208, (char) 75, (char) 103, (char) 59, (char) 205, (char) 94, (char) 193, (char) 107, (char) 168, (char) 166, (char) 97, (char) 189, (char) 207, (char) 137, (char) 164, (char) 146, (char) 8, (char) 25, (char) 51, (char) 91, (char) 172, (char) 198, (char) 119}, 0);
		p110.target_component_SET((char) 205);
		p110.target_network_SET((char) 184);
		p110.target_system_SET((char) 163);
		CommunicationChannel.instance.send(p110);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_TIMESYNC.add((src, ph, pack) ->
		{
			assert (pack.tc1_GET() == -311025674795813319L);
			assert (pack.ts1_GET() == -5105396376471280187L);
		});
		GroundControl.TIMESYNC p111 = CommunicationChannel.new_TIMESYNC();
		PH.setPack(p111);
		p111.ts1_SET(-5105396376471280187L);
		p111.tc1_SET(-311025674795813319L);
		CommunicationChannel.instance.send(p111);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_CAMERA_TRIGGER.add((src, ph, pack) ->
		{
			assert (pack.time_usec_GET() == 3042793742130890387L);
			assert (pack.seq_GET() == 2984199263L);
		});
		GroundControl.CAMERA_TRIGGER p112 = CommunicationChannel.new_CAMERA_TRIGGER();
		PH.setPack(p112);
		p112.seq_SET(2984199263L);
		p112.time_usec_SET(3042793742130890387L);
		CommunicationChannel.instance.send(p112);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_HIL_GPS.add((src, ph, pack) ->
		{
			assert (pack.alt_GET() == 113373031);
			assert (pack.fix_type_GET() == (char) 40);
			assert (pack.eph_GET() == (char) 29772);
			assert (pack.epv_GET() == (char) 26934);
			assert (pack.lat_GET() == 904841779);
			assert (pack.ve_GET() == (short) 13992);
			assert (pack.time_usec_GET() == 561427991495330974L);
			assert (pack.vd_GET() == (short) 6651);
			assert (pack.vel_GET() == (char) 24245);
			assert (pack.lon_GET() == -1156625986);
			assert (pack.satellites_visible_GET() == (char) 62);
			assert (pack.vn_GET() == (short) -16796);
			assert (pack.cog_GET() == (char) 29343);
		});
		GroundControl.HIL_GPS p113 = CommunicationChannel.new_HIL_GPS();
		PH.setPack(p113);
		p113.vel_SET((char) 24245);
		p113.fix_type_SET((char) 40);
		p113.cog_SET((char) 29343);
		p113.lat_SET(904841779);
		p113.epv_SET((char) 26934);
		p113.vd_SET((short) 6651);
		p113.ve_SET((short) 13992);
		p113.lon_SET(-1156625986);
		p113.eph_SET((char) 29772);
		p113.time_usec_SET(561427991495330974L);
		p113.vn_SET((short) -16796);
		p113.alt_SET(113373031);
		p113.satellites_visible_SET((char) 62);
		CommunicationChannel.instance.send(p113);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_HIL_OPTICAL_FLOW.add((src, ph, pack) ->
		{
			assert (pack.temperature_GET() == (short) 22025);
			assert (pack.sensor_id_GET() == (char) 188);
			assert (pack.integrated_xgyro_GET() == 2.6112828E38F);
			assert (pack.quality_GET() == (char) 208);
			assert (pack.integrated_ygyro_GET() == 1.5271039E38F);
			assert (pack.integrated_x_GET() == -6.151146E37F);
			assert (pack.integration_time_us_GET() == 2228725876L);
			assert (pack.time_delta_distance_us_GET() == 529384958L);
			assert (pack.time_usec_GET() == 7907323141850739362L);
			assert (pack.integrated_zgyro_GET() == 2.5923315E38F);
			assert (pack.integrated_y_GET() == -1.3102699E38F);
			assert (pack.distance_GET() == -2.6725178E38F);
		});
		GroundControl.HIL_OPTICAL_FLOW p114 = CommunicationChannel.new_HIL_OPTICAL_FLOW();
		PH.setPack(p114);
		p114.quality_SET((char) 208);
		p114.integrated_ygyro_SET(1.5271039E38F);
		p114.distance_SET(-2.6725178E38F);
		p114.sensor_id_SET((char) 188);
		p114.integration_time_us_SET(2228725876L);
		p114.temperature_SET((short) 22025);
		p114.time_usec_SET(7907323141850739362L);
		p114.integrated_zgyro_SET(2.5923315E38F);
		p114.integrated_y_SET(-1.3102699E38F);
		p114.time_delta_distance_us_SET(529384958L);
		p114.integrated_xgyro_SET(2.6112828E38F);
		p114.integrated_x_SET(-6.151146E37F);
		CommunicationChannel.instance.send(p114);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_HIL_STATE_QUATERNION.add((src, ph, pack) ->
		{
			assert (Arrays.equals(pack.attitude_quaternion_GET(), new float[]{5.731427E37F, 2.6187767E38F, -1.0571274E38F, -2.728493E38F}));
			assert (pack.yacc_GET() == (short) -13349);
			assert (pack.lat_GET() == -1295348678);
			assert (pack.ind_airspeed_GET() == (char) 2960);
			assert (pack.vy_GET() == (short) 18099);
			assert (pack.xacc_GET() == (short) 12077);
			assert (pack.vx_GET() == (short) 11677);
			assert (pack.lon_GET() == 1270693671);
			assert (pack.zacc_GET() == (short) 14388);
			assert (pack.time_usec_GET() == 8315524456470984704L);
			assert (pack.true_airspeed_GET() == (char) 7986);
			assert (pack.alt_GET() == 2067006882);
			assert (pack.vz_GET() == (short) 9030);
			assert (pack.yawspeed_GET() == -1.2362411E38F);
			assert (pack.rollspeed_GET() == -1.6938111E38F);
			assert (pack.pitchspeed_GET() == 1.7635965E38F);
		});
		GroundControl.HIL_STATE_QUATERNION p115 = CommunicationChannel.new_HIL_STATE_QUATERNION();
		PH.setPack(p115);
		p115.vx_SET((short) 11677);
		p115.ind_airspeed_SET((char) 2960);
		p115.yawspeed_SET(-1.2362411E38F);
		p115.true_airspeed_SET((char) 7986);
		p115.vy_SET((short) 18099);
		p115.rollspeed_SET(-1.6938111E38F);
		p115.yacc_SET((short) -13349);
		p115.attitude_quaternion_SET(new float[]{5.731427E37F, 2.6187767E38F, -1.0571274E38F, -2.728493E38F}, 0);
		p115.time_usec_SET(8315524456470984704L);
		p115.xacc_SET((short) 12077);
		p115.vz_SET((short) 9030);
		p115.zacc_SET((short) 14388);
		p115.pitchspeed_SET(1.7635965E38F);
		p115.alt_SET(2067006882);
		p115.lon_SET(1270693671);
		p115.lat_SET(-1295348678);
		CommunicationChannel.instance.send(p115);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_SCALED_IMU2.add((src, ph, pack) ->
		{
			assert (pack.ymag_GET() == (short) 1220);
			assert (pack.ygyro_GET() == (short) 11510);
			assert (pack.xmag_GET() == (short) -17753);
			assert (pack.xacc_GET() == (short) -17873);
			assert (pack.xgyro_GET() == (short) -21605);
			assert (pack.zacc_GET() == (short) -23012);
			assert (pack.yacc_GET() == (short) -5970);
			assert (pack.zgyro_GET() == (short) -27293);
			assert (pack.zmag_GET() == (short) -14812);
			assert (pack.time_boot_ms_GET() == 1609643735L);
		});
		GroundControl.SCALED_IMU2 p116 = CommunicationChannel.new_SCALED_IMU2();
		PH.setPack(p116);
		p116.zacc_SET((short) -23012);
		p116.xmag_SET((short) -17753);
		p116.xacc_SET((short) -17873);
		p116.yacc_SET((short) -5970);
		p116.ymag_SET((short) 1220);
		p116.zgyro_SET((short) -27293);
		p116.zmag_SET((short) -14812);
		p116.xgyro_SET((short) -21605);
		p116.ygyro_SET((short) 11510);
		p116.time_boot_ms_SET(1609643735L);
		CommunicationChannel.instance.send(p116);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_LOG_REQUEST_LIST.add((src, ph, pack) ->
		{
			assert (pack.target_component_GET() == (char) 251);
			assert (pack.end_GET() == (char) 61113);
			assert (pack.start_GET() == (char) 45838);
			assert (pack.target_system_GET() == (char) 43);
		});
		GroundControl.LOG_REQUEST_LIST p117 = CommunicationChannel.new_LOG_REQUEST_LIST();
		PH.setPack(p117);
		p117.target_component_SET((char) 251);
		p117.start_SET((char) 45838);
		p117.target_system_SET((char) 43);
		p117.end_SET((char) 61113);
		CommunicationChannel.instance.send(p117);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_LOG_ENTRY.add((src, ph, pack) ->
		{
			assert (pack.size_GET() == 427370899L);
			assert (pack.time_utc_GET() == 4257933916L);
			assert (pack.last_log_num_GET() == (char) 15715);
			assert (pack.num_logs_GET() == (char) 11631);
			assert (pack.id_GET() == (char) 16272);
		});
		GroundControl.LOG_ENTRY p118 = CommunicationChannel.new_LOG_ENTRY();
		PH.setPack(p118);
		p118.last_log_num_SET((char) 15715);
		p118.size_SET(427370899L);
		p118.num_logs_SET((char) 11631);
		p118.id_SET((char) 16272);
		p118.time_utc_SET(4257933916L);
		CommunicationChannel.instance.send(p118);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_LOG_REQUEST_DATA.add((src, ph, pack) ->
		{
			assert (pack.count_GET() == 1856073772L);
			assert (pack.target_component_GET() == (char) 157);
			assert (pack.id_GET() == (char) 56714);
			assert (pack.target_system_GET() == (char) 73);
			assert (pack.ofs_GET() == 1314843699L);
		});
		GroundControl.LOG_REQUEST_DATA p119 = CommunicationChannel.new_LOG_REQUEST_DATA();
		PH.setPack(p119);
		p119.id_SET((char) 56714);
		p119.ofs_SET(1314843699L);
		p119.count_SET(1856073772L);
		p119.target_system_SET((char) 73);
		p119.target_component_SET((char) 157);
		CommunicationChannel.instance.send(p119);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_LOG_DATA.add((src, ph, pack) ->
		{
			assert (pack.ofs_GET() == 2503615144L);
			assert (Arrays.equals(pack.data__GET(), new char[]{(char) 130, (char) 41, (char) 40, (char) 126, (char) 133, (char) 241, (char) 22, (char) 127, (char) 25, (char) 34, (char) 104, (char) 46, (char) 228, (char) 181, (char) 224, (char) 92, (char) 241, (char) 255, (char) 44, (char) 148, (char) 29, (char) 172, (char) 134, (char) 109, (char) 106, (char) 207, (char) 99, (char) 124, (char) 189, (char) 249, (char) 44, (char) 157, (char) 73, (char) 9, (char) 40, (char) 224, (char) 245, (char) 108, (char) 196, (char) 122, (char) 65, (char) 156, (char) 198, (char) 104, (char) 123, (char) 83, (char) 90, (char) 228, (char) 141, (char) 216, (char) 107, (char) 66, (char) 107, (char) 26, (char) 67, (char) 86, (char) 161, (char) 131, (char) 108, (char) 86, (char) 49, (char) 76, (char) 174, (char) 14, (char) 3, (char) 198, (char) 52, (char) 29, (char) 216, (char) 43, (char) 80, (char) 168, (char) 66, (char) 178, (char) 22, (char) 183, (char) 24, (char) 161, (char) 35, (char) 175, (char) 216, (char) 196, (char) 196, (char) 171, (char) 0, (char) 119, (char) 123, (char) 188, (char) 122, (char) 12}));
			assert (pack.id_GET() == (char) 45263);
			assert (pack.count_GET() == (char) 76);
		});
		GroundControl.LOG_DATA p120 = CommunicationChannel.new_LOG_DATA();
		PH.setPack(p120);
		p120.data__SET(new char[]{(char) 130, (char) 41, (char) 40, (char) 126, (char) 133, (char) 241, (char) 22, (char) 127, (char) 25, (char) 34, (char) 104, (char) 46, (char) 228, (char) 181, (char) 224, (char) 92, (char) 241, (char) 255, (char) 44, (char) 148, (char) 29, (char) 172, (char) 134, (char) 109, (char) 106, (char) 207, (char) 99, (char) 124, (char) 189, (char) 249, (char) 44, (char) 157, (char) 73, (char) 9, (char) 40, (char) 224, (char) 245, (char) 108, (char) 196, (char) 122, (char) 65, (char) 156, (char) 198, (char) 104, (char) 123, (char) 83, (char) 90, (char) 228, (char) 141, (char) 216, (char) 107, (char) 66, (char) 107, (char) 26, (char) 67, (char) 86, (char) 161, (char) 131, (char) 108, (char) 86, (char) 49, (char) 76, (char) 174, (char) 14, (char) 3, (char) 198, (char) 52, (char) 29, (char) 216, (char) 43, (char) 80, (char) 168, (char) 66, (char) 178, (char) 22, (char) 183, (char) 24, (char) 161, (char) 35, (char) 175, (char) 216, (char) 196, (char) 196, (char) 171, (char) 0, (char) 119, (char) 123, (char) 188, (char) 122, (char) 12}, 0);
		p120.id_SET((char) 45263);
		p120.ofs_SET(2503615144L);
		p120.count_SET((char) 76);
		CommunicationChannel.instance.send(p120);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_LOG_ERASE.add((src, ph, pack) ->
		{
			assert (pack.target_system_GET() == (char) 127);
			assert (pack.target_component_GET() == (char) 227);
		});
		GroundControl.LOG_ERASE p121 = CommunicationChannel.new_LOG_ERASE();
		PH.setPack(p121);
		p121.target_component_SET((char) 227);
		p121.target_system_SET((char) 127);
		CommunicationChannel.instance.send(p121);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_LOG_REQUEST_END.add((src, ph, pack) ->
		{
			assert (pack.target_system_GET() == (char) 118);
			assert (pack.target_component_GET() == (char) 60);
		});
		GroundControl.LOG_REQUEST_END p122 = CommunicationChannel.new_LOG_REQUEST_END();
		PH.setPack(p122);
		p122.target_component_SET((char) 60);
		p122.target_system_SET((char) 118);
		CommunicationChannel.instance.send(p122);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_GPS_INJECT_DATA.add((src, ph, pack) ->
		{
			assert (pack.target_component_GET() == (char) 213);
			assert (pack.len_GET() == (char) 243);
			assert (pack.target_system_GET() == (char) 217);
			assert (Arrays.equals(pack.data__GET(), new char[]{(char) 136, (char) 7, (char) 28, (char) 87, (char) 103, (char) 46, (char) 72, (char) 220, (char) 243, (char) 190, (char) 247, (char) 114, (char) 130, (char) 26, (char) 143, (char) 244, (char) 76, (char) 224, (char) 14, (char) 106, (char) 116, (char) 163, (char) 103, (char) 125, (char) 91, (char) 141, (char) 53, (char) 146, (char) 46, (char) 6, (char) 63, (char) 102, (char) 254, (char) 113, (char) 240, (char) 136, (char) 148, (char) 175, (char) 77, (char) 15, (char) 171, (char) 138, (char) 208, (char) 245, (char) 234, (char) 163, (char) 206, (char) 135, (char) 124, (char) 163, (char) 129, (char) 3, (char) 102, (char) 213, (char) 15, (char) 173, (char) 230, (char) 203, (char) 246, (char) 177, (char) 212, (char) 115, (char) 141, (char) 27, (char) 102, (char) 108, (char) 176, (char) 18, (char) 3, (char) 5, (char) 82, (char) 33, (char) 8, (char) 19, (char) 241, (char) 122, (char) 0, (char) 50, (char) 121, (char) 188, (char) 12, (char) 163, (char) 102, (char) 209, (char) 184, (char) 127, (char) 138, (char) 162, (char) 212, (char) 142, (char) 161, (char) 239, (char) 179, (char) 83, (char) 174, (char) 235, (char) 93, (char) 86, (char) 228, (char) 217, (char) 141, (char) 226, (char) 12, (char) 34, (char) 88, (char) 166, (char) 67, (char) 238, (char) 52, (char) 37}));
		});
		GroundControl.GPS_INJECT_DATA p123 = CommunicationChannel.new_GPS_INJECT_DATA();
		PH.setPack(p123);
		p123.data__SET(new char[]{(char) 136, (char) 7, (char) 28, (char) 87, (char) 103, (char) 46, (char) 72, (char) 220, (char) 243, (char) 190, (char) 247, (char) 114, (char) 130, (char) 26, (char) 143, (char) 244, (char) 76, (char) 224, (char) 14, (char) 106, (char) 116, (char) 163, (char) 103, (char) 125, (char) 91, (char) 141, (char) 53, (char) 146, (char) 46, (char) 6, (char) 63, (char) 102, (char) 254, (char) 113, (char) 240, (char) 136, (char) 148, (char) 175, (char) 77, (char) 15, (char) 171, (char) 138, (char) 208, (char) 245, (char) 234, (char) 163, (char) 206, (char) 135, (char) 124, (char) 163, (char) 129, (char) 3, (char) 102, (char) 213, (char) 15, (char) 173, (char) 230, (char) 203, (char) 246, (char) 177, (char) 212, (char) 115, (char) 141, (char) 27, (char) 102, (char) 108, (char) 176, (char) 18, (char) 3, (char) 5, (char) 82, (char) 33, (char) 8, (char) 19, (char) 241, (char) 122, (char) 0, (char) 50, (char) 121, (char) 188, (char) 12, (char) 163, (char) 102, (char) 209, (char) 184, (char) 127, (char) 138, (char) 162, (char) 212, (char) 142, (char) 161, (char) 239, (char) 179, (char) 83, (char) 174, (char) 235, (char) 93, (char) 86, (char) 228, (char) 217, (char) 141, (char) 226, (char) 12, (char) 34, (char) 88, (char) 166, (char) 67, (char) 238, (char) 52, (char) 37}, 0);
		p123.target_system_SET((char) 217);
		p123.len_SET((char) 243);
		p123.target_component_SET((char) 213);
		CommunicationChannel.instance.send(p123);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_GPS2_RAW.add((src, ph, pack) ->
		{
			assert (pack.lat_GET() == -747794680);
			assert (pack.epv_GET() == (char) 16210);
			assert (pack.cog_GET() == (char) 13000);
			assert (pack.fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_STATIC);
			assert (pack.eph_GET() == (char) 26210);
			assert (pack.dgps_numch_GET() == (char) 127);
			assert (pack.vel_GET() == (char) 44433);
			assert (pack.satellites_visible_GET() == (char) 63);
			assert (pack.dgps_age_GET() == 1025409532L);
			assert (pack.alt_GET() == 1259027883);
			assert (pack.time_usec_GET() == 739468876498597548L);
			assert (pack.lon_GET() == 757525644);
		});
		GroundControl.GPS2_RAW p124 = CommunicationChannel.new_GPS2_RAW();
		PH.setPack(p124);
		p124.epv_SET((char) 16210);
		p124.lat_SET(-747794680);
		p124.fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_STATIC);
		p124.dgps_age_SET(1025409532L);
		p124.eph_SET((char) 26210);
		p124.vel_SET((char) 44433);
		p124.cog_SET((char) 13000);
		p124.alt_SET(1259027883);
		p124.dgps_numch_SET((char) 127);
		p124.satellites_visible_SET((char) 63);
		p124.time_usec_SET(739468876498597548L);
		p124.lon_SET(757525644);
		CommunicationChannel.instance.send(p124);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_POWER_STATUS.add((src, ph, pack) ->
		{
			assert (pack.flags_GET() == MAV_POWER_STATUS.MAV_POWER_STATUS_SERVO_VALID);
			assert (pack.Vservo_GET() == (char) 9436);
			assert (pack.Vcc_GET() == (char) 49914);
		});
		GroundControl.POWER_STATUS p125 = CommunicationChannel.new_POWER_STATUS();
		PH.setPack(p125);
		p125.Vservo_SET((char) 9436);
		p125.flags_SET(MAV_POWER_STATUS.MAV_POWER_STATUS_SERVO_VALID);
		p125.Vcc_SET((char) 49914);
		CommunicationChannel.instance.send(p125);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_SERIAL_CONTROL.add((src, ph, pack) ->
		{
			assert (pack.timeout_GET() == (char) 43199);
			assert (pack.device_GET() == SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_GPS1);
			assert (pack.count_GET() == (char) 188);
			assert (pack.flags_GET() == SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_REPLY);
			assert (pack.baudrate_GET() == 3283836025L);
			assert (Arrays.equals(pack.data__GET(), new char[]{(char) 243, (char) 76, (char) 154, (char) 74, (char) 223, (char) 18, (char) 7, (char) 53, (char) 184, (char) 22, (char) 126, (char) 116, (char) 47, (char) 137, (char) 11, (char) 196, (char) 75, (char) 127, (char) 244, (char) 60, (char) 226, (char) 47, (char) 104, (char) 65, (char) 17, (char) 194, (char) 146, (char) 98, (char) 116, (char) 73, (char) 123, (char) 224, (char) 58, (char) 241, (char) 61, (char) 130, (char) 130, (char) 155, (char) 182, (char) 241, (char) 11, (char) 49, (char) 144, (char) 155, (char) 34, (char) 211, (char) 174, (char) 32, (char) 55, (char) 88, (char) 99, (char) 28, (char) 153, (char) 113, (char) 95, (char) 18, (char) 181, (char) 118, (char) 181, (char) 251, (char) 6, (char) 91, (char) 180, (char) 109, (char) 179, (char) 37, (char) 252, (char) 221, (char) 92, (char) 135}));
		});
		GroundControl.SERIAL_CONTROL p126 = CommunicationChannel.new_SERIAL_CONTROL();
		PH.setPack(p126);
		p126.device_SET(SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_GPS1);
		p126.flags_SET(SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_REPLY);
		p126.baudrate_SET(3283836025L);
		p126.count_SET((char) 188);
		p126.timeout_SET((char) 43199);
		p126.data__SET(new char[]{(char) 243, (char) 76, (char) 154, (char) 74, (char) 223, (char) 18, (char) 7, (char) 53, (char) 184, (char) 22, (char) 126, (char) 116, (char) 47, (char) 137, (char) 11, (char) 196, (char) 75, (char) 127, (char) 244, (char) 60, (char) 226, (char) 47, (char) 104, (char) 65, (char) 17, (char) 194, (char) 146, (char) 98, (char) 116, (char) 73, (char) 123, (char) 224, (char) 58, (char) 241, (char) 61, (char) 130, (char) 130, (char) 155, (char) 182, (char) 241, (char) 11, (char) 49, (char) 144, (char) 155, (char) 34, (char) 211, (char) 174, (char) 32, (char) 55, (char) 88, (char) 99, (char) 28, (char) 153, (char) 113, (char) 95, (char) 18, (char) 181, (char) 118, (char) 181, (char) 251, (char) 6, (char) 91, (char) 180, (char) 109, (char) 179, (char) 37, (char) 252, (char) 221, (char) 92, (char) 135}, 0);
		CommunicationChannel.instance.send(p126);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_GPS_RTK.add((src, ph, pack) ->
		{
			assert (pack.rtk_health_GET() == (char) 17);
			assert (pack.baseline_coords_type_GET() == (char) 210);
			assert (pack.rtk_receiver_id_GET() == (char) 20);
			assert (pack.tow_GET() == 2606868289L);
			assert (pack.baseline_a_mm_GET() == -1142217332);
			assert (pack.baseline_b_mm_GET() == 1191385035);
			assert (pack.nsats_GET() == (char) 244);
			assert (pack.iar_num_hypotheses_GET() == 1620432048);
			assert (pack.baseline_c_mm_GET() == -690040945);
			assert (pack.time_last_baseline_ms_GET() == 3214942692L);
			assert (pack.wn_GET() == (char) 63304);
			assert (pack.rtk_rate_GET() == (char) 51);
			assert (pack.accuracy_GET() == 1721900507L);
		});
		GroundControl.GPS_RTK p127 = CommunicationChannel.new_GPS_RTK();
		PH.setPack(p127);
		p127.tow_SET(2606868289L);
		p127.rtk_rate_SET((char) 51);
		p127.iar_num_hypotheses_SET(1620432048);
		p127.nsats_SET((char) 244);
		p127.accuracy_SET(1721900507L);
		p127.baseline_c_mm_SET(-690040945);
		p127.rtk_health_SET((char) 17);
		p127.time_last_baseline_ms_SET(3214942692L);
		p127.wn_SET((char) 63304);
		p127.rtk_receiver_id_SET((char) 20);
		p127.baseline_b_mm_SET(1191385035);
		p127.baseline_a_mm_SET(-1142217332);
		p127.baseline_coords_type_SET((char) 210);
		CommunicationChannel.instance.send(p127);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_GPS2_RTK.add((src, ph, pack) ->
		{
			assert (pack.iar_num_hypotheses_GET() == -50654908);
			assert (pack.baseline_coords_type_GET() == (char) 242);
			assert (pack.rtk_health_GET() == (char) 164);
			assert (pack.rtk_rate_GET() == (char) 171);
			assert (pack.accuracy_GET() == 1503452129L);
			assert (pack.baseline_b_mm_GET() == 399066847);
			assert (pack.nsats_GET() == (char) 60);
			assert (pack.wn_GET() == (char) 9010);
			assert (pack.baseline_a_mm_GET() == -381128863);
			assert (pack.rtk_receiver_id_GET() == (char) 99);
			assert (pack.baseline_c_mm_GET() == 1903455202);
			assert (pack.time_last_baseline_ms_GET() == 1181904188L);
			assert (pack.tow_GET() == 4164866360L);
		});
		GroundControl.GPS2_RTK p128 = CommunicationChannel.new_GPS2_RTK();
		PH.setPack(p128);
		p128.iar_num_hypotheses_SET(-50654908);
		p128.baseline_a_mm_SET(-381128863);
		p128.rtk_rate_SET((char) 171);
		p128.tow_SET(4164866360L);
		p128.wn_SET((char) 9010);
		p128.baseline_coords_type_SET((char) 242);
		p128.accuracy_SET(1503452129L);
		p128.baseline_b_mm_SET(399066847);
		p128.nsats_SET((char) 60);
		p128.rtk_health_SET((char) 164);
		p128.time_last_baseline_ms_SET(1181904188L);
		p128.rtk_receiver_id_SET((char) 99);
		p128.baseline_c_mm_SET(1903455202);
		CommunicationChannel.instance.send(p128);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_SCALED_IMU3.add((src, ph, pack) ->
		{
			assert (pack.yacc_GET() == (short) -29417);
			assert (pack.ymag_GET() == (short) 9221);
			assert (pack.zmag_GET() == (short) 11823);
			assert (pack.time_boot_ms_GET() == 620597759L);
			assert (pack.xmag_GET() == (short) 27602);
			assert (pack.zacc_GET() == (short) -4551);
			assert (pack.xgyro_GET() == (short) 1627);
			assert (pack.zgyro_GET() == (short) -22549);
			assert (pack.xacc_GET() == (short) 26197);
			assert (pack.ygyro_GET() == (short) 23118);
		});
		GroundControl.SCALED_IMU3 p129 = CommunicationChannel.new_SCALED_IMU3();
		PH.setPack(p129);
		p129.zacc_SET((short) -4551);
		p129.zmag_SET((short) 11823);
		p129.xgyro_SET((short) 1627);
		p129.xacc_SET((short) 26197);
		p129.zgyro_SET((short) -22549);
		p129.time_boot_ms_SET(620597759L);
		p129.ygyro_SET((short) 23118);
		p129.xmag_SET((short) 27602);
		p129.ymag_SET((short) 9221);
		p129.yacc_SET((short) -29417);
		CommunicationChannel.instance.send(p129);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_DATA_TRANSMISSION_HANDSHAKE.add((src, ph, pack) ->
		{
			assert (pack.payload_GET() == (char) 19);
			assert (pack.height_GET() == (char) 9057);
			assert (pack.packets_GET() == (char) 32323);
			assert (pack.size_GET() == 1048643386L);
			assert (pack.jpg_quality_GET() == (char) 13);
			assert (pack.width_GET() == (char) 28241);
			assert (pack.type_GET() == (char) 59);
		});
		GroundControl.DATA_TRANSMISSION_HANDSHAKE p130 = CommunicationChannel.new_DATA_TRANSMISSION_HANDSHAKE();
		PH.setPack(p130);
		p130.width_SET((char) 28241);
		p130.packets_SET((char) 32323);
		p130.payload_SET((char) 19);
		p130.jpg_quality_SET((char) 13);
		p130.size_SET(1048643386L);
		p130.height_SET((char) 9057);
		p130.type_SET((char) 59);
		CommunicationChannel.instance.send(p130);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_ENCAPSULATED_DATA.add((src, ph, pack) ->
		{
			assert (pack.seqnr_GET() == (char) 6574);
			assert (Arrays.equals(pack.data__GET(), new char[]{(char) 197, (char) 211, (char) 179, (char) 0, (char) 33, (char) 78, (char) 217, (char) 30, (char) 231, (char) 79, (char) 116, (char) 85, (char) 119, (char) 143, (char) 251, (char) 141, (char) 74, (char) 154, (char) 67, (char) 139, (char) 41, (char) 192, (char) 158, (char) 186, (char) 8, (char) 234, (char) 148, (char) 169, (char) 199, (char) 201, (char) 168, (char) 249, (char) 23, (char) 220, (char) 109, (char) 24, (char) 207, (char) 131, (char) 163, (char) 235, (char) 116, (char) 186, (char) 160, (char) 131, (char) 152, (char) 85, (char) 90, (char) 117, (char) 223, (char) 96, (char) 245, (char) 5, (char) 126, (char) 195, (char) 186, (char) 124, (char) 122, (char) 151, (char) 214, (char) 28, (char) 196, (char) 60, (char) 30, (char) 27, (char) 169, (char) 143, (char) 154, (char) 224, (char) 62, (char) 114, (char) 168, (char) 129, (char) 123, (char) 72, (char) 207, (char) 8, (char) 78, (char) 244, (char) 137, (char) 24, (char) 204, (char) 116, (char) 255, (char) 90, (char) 231, (char) 149, (char) 176, (char) 231, (char) 255, (char) 164, (char) 230, (char) 45, (char) 92, (char) 53, (char) 253, (char) 224, (char) 216, (char) 190, (char) 72, (char) 37, (char) 233, (char) 229, (char) 179, (char) 10, (char) 53, (char) 148, (char) 194, (char) 188, (char) 2, (char) 2, (char) 228, (char) 24, (char) 20, (char) 193, (char) 159, (char) 147, (char) 165, (char) 228, (char) 26, (char) 242, (char) 236, (char) 238, (char) 229, (char) 230, (char) 97, (char) 73, (char) 39, (char) 126, (char) 121, (char) 182, (char) 205, (char) 48, (char) 20, (char) 176, (char) 238, (char) 48, (char) 73, (char) 176, (char) 80, (char) 240, (char) 249, (char) 173, (char) 41, (char) 127, (char) 60, (char) 180, (char) 251, (char) 223, (char) 64, (char) 101, (char) 27, (char) 81, (char) 81, (char) 246, (char) 178, (char) 171, (char) 38, (char) 56, (char) 13, (char) 56, (char) 165, (char) 101, (char) 175, (char) 97, (char) 7, (char) 246, (char) 235, (char) 64, (char) 242, (char) 139, (char) 131, (char) 248, (char) 123, (char) 143, (char) 224, (char) 222, (char) 101, (char) 205, (char) 12, (char) 20, (char) 86, (char) 125, (char) 191, (char) 61, (char) 173, (char) 170, (char) 72, (char) 79, (char) 134, (char) 54, (char) 204, (char) 54, (char) 218, (char) 85, (char) 144, (char) 87, (char) 102, (char) 226, (char) 46, (char) 80, (char) 63, (char) 182, (char) 150, (char) 235, (char) 233, (char) 206, (char) 229, (char) 37, (char) 237, (char) 11, (char) 167, (char) 192, (char) 129, (char) 164, (char) 21, (char) 229, (char) 115, (char) 193, (char) 217, (char) 162, (char) 171, (char) 226, (char) 112, (char) 113, (char) 157, (char) 154, (char) 239, (char) 248, (char) 19, (char) 217, (char) 199, (char) 230, (char) 155, (char) 51, (char) 243, (char) 35, (char) 223, (char) 81, (char) 209, (char) 3, (char) 12, (char) 41, (char) 149, (char) 0, (char) 58, (char) 19, (char) 253, (char) 99, (char) 239, (char) 193, (char) 73, (char) 166, (char) 37}));
		});
		GroundControl.ENCAPSULATED_DATA p131 = CommunicationChannel.new_ENCAPSULATED_DATA();
		PH.setPack(p131);
		p131.data__SET(new char[]{(char) 197, (char) 211, (char) 179, (char) 0, (char) 33, (char) 78, (char) 217, (char) 30, (char) 231, (char) 79, (char) 116, (char) 85, (char) 119, (char) 143, (char) 251, (char) 141, (char) 74, (char) 154, (char) 67, (char) 139, (char) 41, (char) 192, (char) 158, (char) 186, (char) 8, (char) 234, (char) 148, (char) 169, (char) 199, (char) 201, (char) 168, (char) 249, (char) 23, (char) 220, (char) 109, (char) 24, (char) 207, (char) 131, (char) 163, (char) 235, (char) 116, (char) 186, (char) 160, (char) 131, (char) 152, (char) 85, (char) 90, (char) 117, (char) 223, (char) 96, (char) 245, (char) 5, (char) 126, (char) 195, (char) 186, (char) 124, (char) 122, (char) 151, (char) 214, (char) 28, (char) 196, (char) 60, (char) 30, (char) 27, (char) 169, (char) 143, (char) 154, (char) 224, (char) 62, (char) 114, (char) 168, (char) 129, (char) 123, (char) 72, (char) 207, (char) 8, (char) 78, (char) 244, (char) 137, (char) 24, (char) 204, (char) 116, (char) 255, (char) 90, (char) 231, (char) 149, (char) 176, (char) 231, (char) 255, (char) 164, (char) 230, (char) 45, (char) 92, (char) 53, (char) 253, (char) 224, (char) 216, (char) 190, (char) 72, (char) 37, (char) 233, (char) 229, (char) 179, (char) 10, (char) 53, (char) 148, (char) 194, (char) 188, (char) 2, (char) 2, (char) 228, (char) 24, (char) 20, (char) 193, (char) 159, (char) 147, (char) 165, (char) 228, (char) 26, (char) 242, (char) 236, (char) 238, (char) 229, (char) 230, (char) 97, (char) 73, (char) 39, (char) 126, (char) 121, (char) 182, (char) 205, (char) 48, (char) 20, (char) 176, (char) 238, (char) 48, (char) 73, (char) 176, (char) 80, (char) 240, (char) 249, (char) 173, (char) 41, (char) 127, (char) 60, (char) 180, (char) 251, (char) 223, (char) 64, (char) 101, (char) 27, (char) 81, (char) 81, (char) 246, (char) 178, (char) 171, (char) 38, (char) 56, (char) 13, (char) 56, (char) 165, (char) 101, (char) 175, (char) 97, (char) 7, (char) 246, (char) 235, (char) 64, (char) 242, (char) 139, (char) 131, (char) 248, (char) 123, (char) 143, (char) 224, (char) 222, (char) 101, (char) 205, (char) 12, (char) 20, (char) 86, (char) 125, (char) 191, (char) 61, (char) 173, (char) 170, (char) 72, (char) 79, (char) 134, (char) 54, (char) 204, (char) 54, (char) 218, (char) 85, (char) 144, (char) 87, (char) 102, (char) 226, (char) 46, (char) 80, (char) 63, (char) 182, (char) 150, (char) 235, (char) 233, (char) 206, (char) 229, (char) 37, (char) 237, (char) 11, (char) 167, (char) 192, (char) 129, (char) 164, (char) 21, (char) 229, (char) 115, (char) 193, (char) 217, (char) 162, (char) 171, (char) 226, (char) 112, (char) 113, (char) 157, (char) 154, (char) 239, (char) 248, (char) 19, (char) 217, (char) 199, (char) 230, (char) 155, (char) 51, (char) 243, (char) 35, (char) 223, (char) 81, (char) 209, (char) 3, (char) 12, (char) 41, (char) 149, (char) 0, (char) 58, (char) 19, (char) 253, (char) 99, (char) 239, (char) 193, (char) 73, (char) 166, (char) 37}, 0);
		p131.seqnr_SET((char) 6574);
		CommunicationChannel.instance.send(p131);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_DISTANCE_SENSOR.add((src, ph, pack) ->
		{
			assert (pack.orientation_GET() == MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_180_YAW_45);
			assert (pack.current_distance_GET() == (char) 2738);
			assert (pack.covariance_GET() == (char) 78);
			assert (pack.min_distance_GET() == (char) 25356);
			assert (pack.type_GET() == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_RADAR);
			assert (pack.time_boot_ms_GET() == 282840154L);
			assert (pack.id_GET() == (char) 146);
			assert (pack.max_distance_GET() == (char) 7272);
		});
		GroundControl.DISTANCE_SENSOR p132 = CommunicationChannel.new_DISTANCE_SENSOR();
		PH.setPack(p132);
		p132.covariance_SET((char) 78);
		p132.orientation_SET(MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_180_YAW_45);
		p132.time_boot_ms_SET(282840154L);
		p132.min_distance_SET((char) 25356);
		p132.type_SET(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_RADAR);
		p132.max_distance_SET((char) 7272);
		p132.id_SET((char) 146);
		p132.current_distance_SET((char) 2738);
		CommunicationChannel.instance.send(p132);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_TERRAIN_REQUEST.add((src, ph, pack) ->
		{
			assert (pack.mask_GET() == 5312945787694547712L);
			assert (pack.grid_spacing_GET() == (char) 51081);
			assert (pack.lat_GET() == 989397790);
			assert (pack.lon_GET() == 265554351);
		});
		GroundControl.TERRAIN_REQUEST p133 = CommunicationChannel.new_TERRAIN_REQUEST();
		PH.setPack(p133);
		p133.mask_SET(5312945787694547712L);
		p133.lon_SET(265554351);
		p133.grid_spacing_SET((char) 51081);
		p133.lat_SET(989397790);
		CommunicationChannel.instance.send(p133);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_TERRAIN_DATA.add((src, ph, pack) ->
		{
			assert (pack.gridbit_GET() == (char) 242);
			assert (Arrays.equals(pack.data__GET(), new short[]{(short) 21697, (short) 16163, (short) -26537, (short) 11516, (short) -3699, (short) 20818, (short) -14184, (short) 9277, (short) 1177, (short) 10875, (short) 22265, (short) 14399, (short) -1966, (short) 10439, (short) 13409, (short) 20573}));
			assert (pack.grid_spacing_GET() == (char) 35707);
			assert (pack.lon_GET() == -1437554552);
			assert (pack.lat_GET() == -277371525);
		});
		GroundControl.TERRAIN_DATA p134 = CommunicationChannel.new_TERRAIN_DATA();
		PH.setPack(p134);
		p134.lon_SET(-1437554552);
		p134.data__SET(new short[]{(short) 21697, (short) 16163, (short) -26537, (short) 11516, (short) -3699, (short) 20818, (short) -14184, (short) 9277, (short) 1177, (short) 10875, (short) 22265, (short) 14399, (short) -1966, (short) 10439, (short) 13409, (short) 20573}, 0);
		p134.lat_SET(-277371525);
		p134.grid_spacing_SET((char) 35707);
		p134.gridbit_SET((char) 242);
		CommunicationChannel.instance.send(p134);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_TERRAIN_CHECK.add((src, ph, pack) ->
		{
			assert (pack.lat_GET() == 1560255445);
			assert (pack.lon_GET() == -1969406108);
		});
		GroundControl.TERRAIN_CHECK p135 = CommunicationChannel.new_TERRAIN_CHECK();
		PH.setPack(p135);
		p135.lon_SET(-1969406108);
		p135.lat_SET(1560255445);
		CommunicationChannel.instance.send(p135);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_TERRAIN_REPORT.add((src, ph, pack) ->
		{
			assert (pack.lat_GET() == 1951546248);
			assert (pack.pending_GET() == (char) 34126);
			assert (pack.spacing_GET() == (char) 34667);
			assert (pack.lon_GET() == -1929540978);
			assert (pack.terrain_height_GET() == 1.5654525E38F);
			assert (pack.loaded_GET() == (char) 32861);
			assert (pack.current_height_GET() == -1.3461797E38F);
		});
		GroundControl.TERRAIN_REPORT p136 = CommunicationChannel.new_TERRAIN_REPORT();
		PH.setPack(p136);
		p136.spacing_SET((char) 34667);
		p136.lat_SET(1951546248);
		p136.pending_SET((char) 34126);
		p136.loaded_SET((char) 32861);
		p136.terrain_height_SET(1.5654525E38F);
		p136.current_height_SET(-1.3461797E38F);
		p136.lon_SET(-1929540978);
		CommunicationChannel.instance.send(p136);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_SCALED_PRESSURE2.add((src, ph, pack) ->
		{
			assert (pack.press_abs_GET() == 2.7349472E38F);
			assert (pack.time_boot_ms_GET() == 3798165260L);
			assert (pack.press_diff_GET() == 2.7205398E36F);
			assert (pack.temperature_GET() == (short) 19124);
		});
		GroundControl.SCALED_PRESSURE2 p137 = CommunicationChannel.new_SCALED_PRESSURE2();
		PH.setPack(p137);
		p137.press_diff_SET(2.7205398E36F);
		p137.time_boot_ms_SET(3798165260L);
		p137.temperature_SET((short) 19124);
		p137.press_abs_SET(2.7349472E38F);
		CommunicationChannel.instance.send(p137);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_ATT_POS_MOCAP.add((src, ph, pack) ->
		{
			assert (pack.time_usec_GET() == 4488952823848261455L);
			assert (pack.x_GET() == 3.113159E38F);
			assert (pack.z_GET() == 1.710965E38F);
			assert (Arrays.equals(pack.q_GET(), new float[]{5.913969E37F, 1.4621484E38F, 4.1707852E37F, -2.7357172E38F}));
			assert (pack.y_GET() == -7.4990097E37F);
		});
		GroundControl.ATT_POS_MOCAP p138 = CommunicationChannel.new_ATT_POS_MOCAP();
		PH.setPack(p138);
		p138.time_usec_SET(4488952823848261455L);
		p138.y_SET(-7.4990097E37F);
		p138.x_SET(3.113159E38F);
		p138.z_SET(1.710965E38F);
		p138.q_SET(new float[]{5.913969E37F, 1.4621484E38F, 4.1707852E37F, -2.7357172E38F}, 0);
		CommunicationChannel.instance.send(p138);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_SET_ACTUATOR_CONTROL_TARGET.add((src, ph, pack) ->
		{
			assert (pack.target_system_GET() == (char) 169);
			assert (pack.time_usec_GET() == 1208146658525191517L);
			assert (pack.target_component_GET() == (char) 79);
			assert (pack.group_mlx_GET() == (char) 35);
			assert (Arrays.equals(pack.controls_GET(), new float[]{3.0294186E38F, 2.0137863E38F, -2.5708092E38F, -1.2015466E38F, -1.1921588E36F, 9.238427E37F, 1.0881435E38F, -1.5632678E38F}));
		});
		GroundControl.SET_ACTUATOR_CONTROL_TARGET p139 = CommunicationChannel.new_SET_ACTUATOR_CONTROL_TARGET();
		PH.setPack(p139);
		p139.time_usec_SET(1208146658525191517L);
		p139.target_system_SET((char) 169);
		p139.target_component_SET((char) 79);
		p139.group_mlx_SET((char) 35);
		p139.controls_SET(new float[]{3.0294186E38F, 2.0137863E38F, -2.5708092E38F, -1.2015466E38F, -1.1921588E36F, 9.238427E37F, 1.0881435E38F, -1.5632678E38F}, 0);
		CommunicationChannel.instance.send(p139);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_ACTUATOR_CONTROL_TARGET.add((src, ph, pack) ->
		{
			assert (pack.group_mlx_GET() == (char) 28);
			assert (pack.time_usec_GET() == 4179699143504839660L);
			assert (Arrays.equals(pack.controls_GET(), new float[]{2.7436271E38F, -8.671628E37F, -2.034874E38F, -1.199445E38F, -1.2531488E37F, -3.0878867E38F, 1.8312327E37F, 6.4079753E37F}));
		});
		GroundControl.ACTUATOR_CONTROL_TARGET p140 = CommunicationChannel.new_ACTUATOR_CONTROL_TARGET();
		PH.setPack(p140);
		p140.time_usec_SET(4179699143504839660L);
		p140.group_mlx_SET((char) 28);
		p140.controls_SET(new float[]{2.7436271E38F, -8.671628E37F, -2.034874E38F, -1.199445E38F, -1.2531488E37F, -3.0878867E38F, 1.8312327E37F, 6.4079753E37F}, 0);
		CommunicationChannel.instance.send(p140);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		TestChannel.instance.on_ALTITUDE.add((src, ph, pack) ->
		{
			assert (pack.altitude_local_GET() == 1.7977275E38F);
			assert (pack.altitude_relative_GET() == -1.0418283E38F);
			assert (pack.time_usec_GET() == 1455181842355416160L);
			assert (pack.altitude_monotonic_GET() == 8.596909E37F);
			assert (pack.altitude_terrain_GET() == -3.0855577E38F);
			assert (pack.bottom_clearance_GET() == -7.4282003E37F);
			assert (pack.altitude_amsl_GET() == 1.0396024E38F);
		});
		GroundControl.ALTITUDE p141 = CommunicationChannel.new_ALTITUDE();
		PH.setPack(p141);
		p141.altitude_amsl_SET(1.0396024E38F);
		p141.time_usec_SET(1455181842355416160L);
		p141.altitude_relative_SET(-1.0418283E38F);
		p141.altitude_local_SET(1.7977275E38F);
		p141.bottom_clearance_SET(-7.4282003E37F);
		p141.altitude_monotonic_SET(8.596909E37F);
		p141.altitude_terrain_SET(-3.0855577E38F);
		CommunicationChannel.instance.send(p141);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
		TestChannel.instance.on_RESOURCE_REQUEST.add((src, ph, pack) ->
		{
			assert (Arrays.equals(pack.storage_GET(), new char[]{(char) 112, (char) 40, (char) 255, (char) 157, (char) 3, (char) 198, (char) 243, (char) 57, (char) 125, (char) 247, (char) 106, (char) 42, (char) 121, (char) 152, (char) 60, (char) 119, (char) 99, (char) 122, (char) 254, (char) 186, (char) 91, (char) 175, (char) 139, (char) 51, (char) 76, (char) 133, (char) 92, (char) 225, (char) 134, (char) 232, (char) 231, (char) 41, (char) 32, (char) 239, (char) 243, (char) 178, (char) 61, (char) 145, (char) 248, (char) 211, (char) 118, (char) 31, (char) 179, (char) 251, (char) 62, (char) 251, (char) 205, (char) 82, (char) 96, (char) 127, (char) 9, (char) 102, (char) 34, (char) 117, (char) 109, (char) 39, (char) 74, (char) 14, (char) 141, (char) 234, (char) 242, (char) 144, (char) 150, (char) 164, (char) 1, (char) 7, (char) 164, (char) 6, (char) 208, (char) 0, (char) 106, (char) 155, (char) 127, (char) 155, (char) 163, (char) 107, (char) 238, (char) 100, (char) 22, (char) 233, (char) 43, (char) 71, (char) 29, (char) 184, (char) 48, (char) 57, (char) 183, (char) 166, (char) 78, (char) 65, (char) 86, (char) 252, (char) 114, (char) 64, (char) 59, (char) 119, (char) 109, (char) 220, (char) 31, (char) 233, (char) 208, (char) 98, (char) 34, (char) 176, (char) 109, (char) 164, (char) 250, (char) 150, (char) 197, (char) 204, (char) 116, (char) 80, (char) 195, (char) 244, (char) 63, (char) 192, (char) 140, (char) 191, (char) 204, (char) 150}));
			assert (pack.transfer_type_GET() == (char) 3);
			assert (pack.request_id_GET() == (char) 189);
			assert (pack.uri_type_GET() == (char) 191);
			assert (Arrays.equals(pack.uri_GET(), new char[]{(char) 117, (char) 214, (char) 92, (char) 27, (char) 182, (char) 123, (char) 41, (char) 63, (char) 143, (char) 133, (char) 156, (char) 29, (char) 248, (char) 11, (char) 177, (char) 174, (char) 74, (char) 220, (char) 217, (char) 94, (char) 81, (char) 238, (char) 129, (char) 52, (char) 37, (char) 130, (char) 230, (char) 176, (char) 221, (char) 114, (char) 10, (char) 244, (char) 174, (char) 211, (char) 54, (char) 221, (char) 149, (char) 233, (char) 245, (char) 31, (char) 158, (char) 27, (char) 176, (char) 224, (char) 199, (char) 21, (char) 170, (char) 67, (char) 33, (char) 149, (char) 228, (char) 50, (char) 201, (char) 179, (char) 47, (char) 99, (char) 160, (char) 195, (char) 144, (char) 46, (char) 80, (char) 54, (char) 66, (char) 245, (char) 91, (char) 186, (char) 21, (char) 232, (char) 107, (char) 155, (char) 62, (char) 138, (char) 113, (char) 147, (char) 223, (char) 252, (char) 101, (char) 165, (char) 174, (char) 33, (char) 80, (char) 187, (char) 106, (char) 88, (char) 61, (char) 94, (char) 156, (char) 172, (char) 100, (char) 253, (char) 245, (char) 55, (char) 29, (char) 238, (char) 104, (char) 22, (char) 162, (char) 84, (char) 243, (char) 92, (char) 149, (char) 119, (char) 162, (char) 183, (char) 47, (char) 143, (char) 245, (char) 76, (char) 242, (char) 206, (char) 177, (char) 16, (char) 205, (char) 9, (char) 147, (char) 226, (char) 106, (char) 234, (char) 237, (char) 48}));
		});
		GroundControl.RESOURCE_REQUEST p142 = CommunicationChannel.new_RESOURCE_REQUEST();
		PH.setPack(p142);
		p142.uri_type_SET((char) 191);
		p142.request_id_SET((char) 189);
		p142.uri_SET(new char[]{(char) 117, (char) 214, (char) 92, (char) 27, (char) 182, (char) 123, (char) 41, (char) 63, (char) 143, (char) 133, (char) 156, (char) 29, (char) 248, (char) 11, (char) 177, (char) 174, (char) 74, (char) 220, (char) 217, (char) 94, (char) 81, (char) 238, (char) 129, (char) 52, (char) 37, (char) 130, (char) 230, (char) 176, (char) 221, (char) 114, (char) 10, (char) 244, (char) 174, (char) 211, (char) 54, (char) 221, (char) 149, (char) 233, (char) 245, (char) 31, (char) 158, (char) 27, (char) 176, (char) 224, (char) 199, (char) 21, (char) 170, (char) 67, (char) 33, (char) 149, (char) 228, (char) 50, (char) 201, (char) 179, (char) 47, (char) 99, (char) 160, (char) 195, (char) 144, (char) 46, (char) 80, (char) 54, (char) 66, (char) 245, (char) 91, (char) 186, (char) 21, (char) 232, (char) 107, (char) 155, (char) 62, (char) 138, (char) 113, (char) 147, (char) 223, (char) 252, (char) 101, (char) 165, (char) 174, (char) 33, (char) 80, (char) 187, (char) 106, (char) 88, (char) 61, (char) 94, (char) 156, (char) 172, (char) 100, (char) 253, (char) 245, (char) 55, (char) 29, (char) 238, (char) 104, (char) 22, (char) 162, (char) 84, (char) 243, (char) 92, (char) 149, (char) 119, (char) 162, (char) 183, (char) 47, (char) 143, (char) 245, (char) 76, (char) 242, (char) 206, (char) 177, (char) 16, (char) 205, (char) 9, (char) 147, (char) 226, (char) 106, (char) 234, (char) 237, (char) 48}, 0);
		p142.transfer_type_SET((char) 3);
		p142.storage_SET(new char[]{(char) 112, (char) 40, (char) 255, (char) 157, (char) 3, (char) 198, (char) 243, (char) 57, (char) 125, (char) 247, (char) 106, (char) 42, (char) 121, (char) 152, (char) 60, (char) 119, (char) 99, (char) 122, (char) 254, (char) 186, (char) 91, (char) 175, (char) 139, (char) 51, (char) 76, (char) 133, (char) 92, (char) 225, (char) 134, (char) 232, (char) 231, (char) 41, (char) 32, (char) 239, (char) 243, (char) 178, (char) 61, (char) 145, (char) 248, (char) 211, (char) 118, (char) 31, (char) 179, (char) 251, (char) 62, (char) 251, (char) 205, (char) 82, (char) 96, (char) 127, (char) 9, (char) 102, (char) 34, (char) 117, (char) 109, (char) 39, (char) 74, (char) 14, (char) 141, (char) 234, (char) 242, (char) 144, (char) 150, (char) 164, (char) 1, (char) 7, (char) 164, (char) 6, (char) 208, (char) 0, (char) 106, (char) 155, (char) 127, (char) 155, (char) 163, (char) 107, (char) 238, (char) 100, (char) 22, (char) 233, (char) 43, (char) 71, (char) 29, (char) 184, (char) 48, (char) 57, (char) 183, (char) 166, (char) 78, (char) 65, (char) 86, (char) 252, (char) 114, (char) 64, (char) 59, (char) 119, (char) 109, (char) 220, (char) 31, (char) 233, (char) 208, (char) 98, (char) 34, (char) 176, (char) 109, (char) 164, (char) 250, (char) 150, (char) 197, (char) 204, (char) 116, (char) 80, (char) 195, (char) 244, (char) 63, (char) 192, (char) 140, (char) 191, (char) 204, (char) 150}, 0);
		CommunicationChannel.instance.send(p142);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
		TestChannel.instance.on_SCALED_PRESSURE3.add((src, ph, pack) ->
		{
			assert (pack.press_diff_GET() == -6.7753536E37F);
			assert (pack.temperature_GET() == (short) 6356);
			assert (pack.time_boot_ms_GET() == 1766958610L);
			assert (pack.press_abs_GET() == 1.3048022E38F);
		});
		GroundControl.SCALED_PRESSURE3 p143 = CommunicationChannel.new_SCALED_PRESSURE3();
		PH.setPack(p143);
		p143.press_abs_SET(1.3048022E38F);
		p143.press_diff_SET(-6.7753536E37F);
		p143.temperature_SET((short) 6356);
		p143.time_boot_ms_SET(1766958610L);
		CommunicationChannel.instance.send(p143);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
		TestChannel.instance.on_FOLLOW_TARGET.add((src, ph, pack) ->
		{
			assert (Arrays.equals(pack.vel_GET(), new float[]{1.874936E38F, -7.581637E37F, -3.370042E38F}));
			assert (pack.est_capabilities_GET() == (char) 153);
			assert (Arrays.equals(pack.attitude_q_GET(), new float[]{-3.0358084E38F, -1.2476731E38F, -2.3064876E38F, 2.2593406E38F}));
			assert (Arrays.equals(pack.acc_GET(), new float[]{-9.484501E37F, 1.2898255E38F, 7.531677E37F}));
			assert (pack.alt_GET() == 2.5184114E38F);
			assert (Arrays.equals(pack.rates_GET(), new float[]{-2.0071874E38F, 2.5648845E38F, 1.3802946E38F}));
			assert (pack.custom_state_GET() == 7355536587004738632L);
			assert (Arrays.equals(pack.position_cov_GET(), new float[]{-3.378001E37F, 1.8103255E38F, 1.934943E38F}));
			assert (pack.timestamp_GET() == 6013991363496676089L);
			assert (pack.lat_GET() == -576648612);
			assert (pack.lon_GET() == -334262017);
		});
		GroundControl.FOLLOW_TARGET p144 = CommunicationChannel.new_FOLLOW_TARGET();
		PH.setPack(p144);
		p144.acc_SET(new float[]{-9.484501E37F, 1.2898255E38F, 7.531677E37F}, 0);
		p144.lon_SET(-334262017);
		p144.custom_state_SET(7355536587004738632L);
		p144.attitude_q_SET(new float[]{-3.0358084E38F, -1.2476731E38F, -2.3064876E38F, 2.2593406E38F}, 0);
		p144.lat_SET(-576648612);
		p144.position_cov_SET(new float[]{-3.378001E37F, 1.8103255E38F, 1.934943E38F}, 0);
		p144.rates_SET(new float[]{-2.0071874E38F, 2.5648845E38F, 1.3802946E38F}, 0);
		p144.alt_SET(2.5184114E38F);
		p144.timestamp_SET(6013991363496676089L);
		p144.vel_SET(new float[]{1.874936E38F, -7.581637E37F, -3.370042E38F}, 0);
		p144.est_capabilities_SET((char) 153);
		CommunicationChannel.instance.send(p144);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
		TestChannel.instance.on_CONTROL_SYSTEM_STATE.add((src, ph, pack) ->
		{
			assert (pack.y_pos_GET() == -2.7028323E38F);
			assert (pack.time_usec_GET() == 8134866025007465496L);
			assert (pack.roll_rate_GET() == 1.6283578E38F);
			assert (pack.yaw_rate_GET() == -4.57281E37F);
			assert (pack.y_acc_GET() == -2.3383817E38F);
			assert (pack.y_vel_GET() == 1.5100039E38F);
			assert (pack.z_acc_GET() == -2.7240755E38F);
			assert (pack.z_pos_GET() == 3.380201E38F);
			assert (pack.x_acc_GET() == 3.0133688E38F);
			assert (pack.x_vel_GET() == -6.7121856E37F);
			assert (pack.x_pos_GET() == -1.484881E37F);
			assert (pack.pitch_rate_GET() == -2.2830391E38F);
			assert (Arrays.equals(pack.q_GET(), new float[]{2.1119805E38F, 9.665189E37F, 7.218416E37F, -8.786834E37F}));
			assert (Arrays.equals(pack.vel_variance_GET(), new float[]{-2.4032282E38F, 5.861301E37F, -3.200142E38F}));
			assert (pack.z_vel_GET() == 2.1090879E38F);
			assert (pack.airspeed_GET() == 1.2264087E38F);
			assert (Arrays.equals(pack.pos_variance_GET(), new float[]{-2.702983E38F, -9.832402E37F, 9.298685E37F}));
		});
		GroundControl.CONTROL_SYSTEM_STATE p146 = CommunicationChannel.new_CONTROL_SYSTEM_STATE();
		PH.setPack(p146);
		p146.airspeed_SET(1.2264087E38F);
		p146.y_pos_SET(-2.7028323E38F);
		p146.y_vel_SET(1.5100039E38F);
		p146.vel_variance_SET(new float[]{-2.4032282E38F, 5.861301E37F, -3.200142E38F}, 0);
		p146.pos_variance_SET(new float[]{-2.702983E38F, -9.832402E37F, 9.298685E37F}, 0);
		p146.time_usec_SET(8134866025007465496L);
		p146.z_acc_SET(-2.7240755E38F);
		p146.x_vel_SET(-6.7121856E37F);
		p146.yaw_rate_SET(-4.57281E37F);
		p146.y_acc_SET(-2.3383817E38F);
		p146.x_acc_SET(3.0133688E38F);
		p146.q_SET(new float[]{2.1119805E38F, 9.665189E37F, 7.218416E37F, -8.786834E37F}, 0);
		p146.x_pos_SET(-1.484881E37F);
		p146.pitch_rate_SET(-2.2830391E38F);
		p146.z_vel_SET(2.1090879E38F);
		p146.roll_rate_SET(1.6283578E38F);
		p146.z_pos_SET(3.380201E38F);
		CommunicationChannel.instance.send(p146);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
		TestChannel.instance.on_BATTERY_STATUS.add((src, ph, pack) ->
		{
			assert (pack.current_consumed_GET() == -474445250);
			assert (pack.energy_consumed_GET() == 2125428014);
			assert (pack.type_GET() == MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_LIPO);
			assert (pack.battery_function_GET() == MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_UNKNOWN);
			assert (Arrays.equals(pack.voltages_GET(), new char[]{(char) 57537, (char) 9104, (char) 58079, (char) 54278, (char) 41227, (char) 11775, (char) 31787, (char) 56262, (char) 14125, (char) 4380}));
			assert (pack.temperature_GET() == (short) -20903);
			assert (pack.id_GET() == (char) 41);
			assert (pack.current_battery_GET() == (short) -12563);
			assert (pack.battery_remaining_GET() == (byte) 78);
		});
		GroundControl.BATTERY_STATUS p147 = CommunicationChannel.new_BATTERY_STATUS();
		PH.setPack(p147);
		p147.battery_remaining_SET((byte) 78);
		p147.energy_consumed_SET(2125428014);
		p147.current_consumed_SET(-474445250);
		p147.temperature_SET((short) -20903);
		p147.current_battery_SET((short) -12563);
		p147.type_SET(MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_LIPO);
		p147.voltages_SET(new char[]{(char) 57537, (char) 9104, (char) 58079, (char) 54278, (char) 41227, (char) 11775, (char) 31787, (char) 56262, (char) 14125, (char) 4380}, 0);
		p147.battery_function_SET(MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_UNKNOWN);
		p147.id_SET((char) 41);
		CommunicationChannel.instance.send(p147);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
		TestChannel.instance.on_AUTOPILOT_VERSION.add((src, ph, pack) ->
		{
			assert (pack.capabilities_GET() == MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_ACTUATOR_TARGET);
			assert (Arrays.equals(pack.flight_custom_version_GET(), new char[]{(char) 230, (char) 195, (char) 169, (char) 72, (char) 26, (char) 244, (char) 27, (char) 250}));
			assert (Arrays.equals(pack.middleware_custom_version_GET(), new char[]{(char) 160, (char) 241, (char) 77, (char) 222, (char) 172, (char) 154, (char) 62, (char) 246}));
			assert (pack.os_sw_version_GET() == 2731575234L);
			assert (Arrays.equals(pack.uid2_TRY(ph), new char[]{(char) 61, (char) 34, (char) 232, (char) 120, (char) 157, (char) 238, (char) 8, (char) 129, (char) 229, (char) 242, (char) 201, (char) 88, (char) 171, (char) 101, (char) 76, (char) 11, (char) 160, (char) 64}));
			assert (pack.vendor_id_GET() == (char) 35062);
			assert (pack.uid_GET() == 2841041268553601404L);
			assert (pack.middleware_sw_version_GET() == 1814904101L);
			assert (pack.flight_sw_version_GET() == 2309887494L);
			assert (pack.product_id_GET() == (char) 41843);
			assert (Arrays.equals(pack.os_custom_version_GET(), new char[]{(char) 89, (char) 24, (char) 31, (char) 193, (char) 194, (char) 102, (char) 101, (char) 69}));
			assert (pack.board_version_GET() == 1603829051L);
		});
		GroundControl.AUTOPILOT_VERSION p148 = CommunicationChannel.new_AUTOPILOT_VERSION();
		PH.setPack(p148);
		p148.capabilities_SET(MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_ACTUATOR_TARGET);
		p148.os_custom_version_SET(new char[]{(char) 89, (char) 24, (char) 31, (char) 193, (char) 194, (char) 102, (char) 101, (char) 69}, 0);
		p148.vendor_id_SET((char) 35062);
		p148.uid_SET(2841041268553601404L);
		p148.flight_sw_version_SET(2309887494L);
		p148.flight_custom_version_SET(new char[]{(char) 230, (char) 195, (char) 169, (char) 72, (char) 26, (char) 244, (char) 27, (char) 250}, 0);
		p148.middleware_sw_version_SET(1814904101L);
		p148.board_version_SET(1603829051L);
		p148.middleware_custom_version_SET(new char[]{(char) 160, (char) 241, (char) 77, (char) 222, (char) 172, (char) 154, (char) 62, (char) 246}, 0);
		p148.os_sw_version_SET(2731575234L);
		p148.product_id_SET((char) 41843);
		p148.uid2_SET(new char[]{(char) 61, (char) 34, (char) 232, (char) 120, (char) 157, (char) 238, (char) 8, (char) 129, (char) 229, (char) 242, (char) 201, (char) 88, (char) 171, (char) 101, (char) 76, (char) 11, (char) 160, (char) 64}, 0, PH);
		CommunicationChannel.instance.send(p148);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
		TestChannel.instance.on_LANDING_TARGET.add((src, ph, pack) ->
		{
			assert (pack.angle_y_GET() == 1.1250555E38F);
			assert (pack.z_TRY(ph) == 3.063887E38F);
			assert (pack.type_GET() == LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_VISION_OTHER);
			assert (pack.target_num_GET() == (char) 63);
			assert (pack.angle_x_GET() == -3.0147052E38F);
			assert (pack.position_valid_TRY(ph) == (char) 113);
			assert (pack.size_y_GET() == -1.5614232E37F);
			assert (pack.x_TRY(ph) == 2.2738806E38F);
			assert (pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_INT);
			assert (pack.y_TRY(ph) == 2.4597389E38F);
			assert (pack.time_usec_GET() == 8421272872371680786L);
			assert (Arrays.equals(pack.q_TRY(ph), new float[]{-1.9226728E38F, -1.9852631E38F, 1.3964634E38F, -9.693198E37F}));
			assert (pack.size_x_GET() == -3.2120064E38F);
			assert (pack.distance_GET() == 2.462305E38F);
		});
		GroundControl.LANDING_TARGET p149 = CommunicationChannel.new_LANDING_TARGET();
		PH.setPack(p149);
		p149.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_INT);
		p149.z_SET(3.063887E38F, PH);
		p149.y_SET(2.4597389E38F, PH);
		p149.distance_SET(2.462305E38F);
		p149.angle_x_SET(-3.0147052E38F);
		p149.time_usec_SET(8421272872371680786L);
		p149.target_num_SET((char) 63);
		p149.q_SET(new float[]{-1.9226728E38F, -1.9852631E38F, 1.3964634E38F, -9.693198E37F}, 0, PH);
		p149.angle_y_SET(1.1250555E38F);
		p149.position_valid_SET((char) 113, PH);
		p149.type_SET(LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_VISION_OTHER);
		p149.size_x_SET(-3.2120064E38F);
		p149.x_SET(2.2738806E38F, PH);
		p149.size_y_SET(-1.5614232E37F);
		CommunicationChannel.instance.send(p149);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
		TestChannel.instance.on_SCRIPT_ITEM.add((src, ph, pack) ->
		{
			assert (pack.target_component_GET() == (char) 112);
			assert (pack.name_LEN(ph) == 7);
			assert (pack.name_TRY(ph).equals("Fowgabs"));
			assert (pack.target_system_GET() == (char) 219);
			assert (pack.seq_GET() == (char) 8715);
		});
		GroundControl.SCRIPT_ITEM p180 = CommunicationChannel.new_SCRIPT_ITEM();
		PH.setPack(p180);
		p180.target_system_SET((char) 219);
		p180.name_SET("Fowgabs", PH);
		p180.seq_SET((char) 8715);
		p180.target_component_SET((char) 112);
		CommunicationChannel.instance.send(p180);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
		TestChannel.instance.on_SCRIPT_REQUEST.add((src, ph, pack) ->
		{
			assert (pack.seq_GET() == (char) 51099);
			assert (pack.target_component_GET() == (char) 197);
			assert (pack.target_system_GET() == (char) 130);
		});
		GroundControl.SCRIPT_REQUEST p181 = CommunicationChannel.new_SCRIPT_REQUEST();
		PH.setPack(p181);
		p181.target_component_SET((char) 197);
		p181.seq_SET((char) 51099);
		p181.target_system_SET((char) 130);
		CommunicationChannel.instance.send(p181);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
		TestChannel.instance.on_SCRIPT_REQUEST_LIST.add((src, ph, pack) ->
		{
			assert (pack.target_component_GET() == (char) 186);
			assert (pack.target_system_GET() == (char) 24);
		});
		GroundControl.SCRIPT_REQUEST_LIST p182 = CommunicationChannel.new_SCRIPT_REQUEST_LIST();
		PH.setPack(p182);
		p182.target_system_SET((char) 24);
		p182.target_component_SET((char) 186);
		CommunicationChannel.instance.send(p182);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
		TestChannel.instance.on_SCRIPT_COUNT.add((src, ph, pack) ->
		{
			assert (pack.target_system_GET() == (char) 213);
			assert (pack.target_component_GET() == (char) 1);
			assert (pack.count_GET() == (char) 18541);
		});
		GroundControl.SCRIPT_COUNT p183 = CommunicationChannel.new_SCRIPT_COUNT();
		PH.setPack(p183);
		p183.target_component_SET((char) 1);
		p183.target_system_SET((char) 213);
		p183.count_SET((char) 18541);
		CommunicationChannel.instance.send(p183);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
		TestChannel.instance.on_SCRIPT_CURRENT.add((src, ph, pack) ->
		{
			assert (pack.seq_GET() == (char) 56831);
		});
		GroundControl.SCRIPT_CURRENT p184 = CommunicationChannel.new_SCRIPT_CURRENT();
		PH.setPack(p184);
		p184.seq_SET((char) 56831);
		CommunicationChannel.instance.send(p184);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
		TestChannel.instance.on_ESTIMATOR_STATUS.add((src, ph, pack) ->
		{
			assert (pack.hagl_ratio_GET() == -3.3537518E38F);
			assert (pack.flags_GET() == ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_HORIZ);
			assert (pack.pos_horiz_ratio_GET() == -2.183584E38F);
			assert (pack.pos_horiz_accuracy_GET() == 1.0423587E37F);
			assert (pack.time_usec_GET() == 8765658405078362286L);
			assert (pack.tas_ratio_GET() == -9.283282E37F);
			assert (pack.mag_ratio_GET() == -1.8630613E37F);
			assert (pack.pos_vert_accuracy_GET() == 1.0599505E38F);
			assert (pack.pos_vert_ratio_GET() == -5.0375147E37F);
			assert (pack.vel_ratio_GET() == 2.1458401E37F);
		});
		GroundControl.ESTIMATOR_STATUS p230 = CommunicationChannel.new_ESTIMATOR_STATUS();
		PH.setPack(p230);
		p230.flags_SET(ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_HORIZ);
		p230.mag_ratio_SET(-1.8630613E37F);
		p230.hagl_ratio_SET(-3.3537518E38F);
		p230.time_usec_SET(8765658405078362286L);
		p230.pos_horiz_ratio_SET(-2.183584E38F);
		p230.pos_horiz_accuracy_SET(1.0423587E37F);
		p230.tas_ratio_SET(-9.283282E37F);
		p230.vel_ratio_SET(2.1458401E37F);
		p230.pos_vert_accuracy_SET(1.0599505E38F);
		p230.pos_vert_ratio_SET(-5.0375147E37F);
		CommunicationChannel.instance.send(p230);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
		TestChannel.instance.on_WIND_COV.add((src, ph, pack) ->
		{
			assert (pack.horiz_accuracy_GET() == 1.4743841E38F);
			assert (pack.var_vert_GET() == 2.5967918E38F);
			assert (pack.wind_y_GET() == -6.4305944E36F);
			assert (pack.vert_accuracy_GET() == -5.0447834E37F);
			assert (pack.wind_alt_GET() == -1.8047602E38F);
			assert (pack.var_horiz_GET() == 6.2395547E37F);
			assert (pack.time_usec_GET() == 8724745105848360638L);
			assert (pack.wind_z_GET() == -1.3274519E38F);
			assert (pack.wind_x_GET() == 1.778478E37F);
		});
		GroundControl.WIND_COV p231 = CommunicationChannel.new_WIND_COV();
		PH.setPack(p231);
		p231.wind_x_SET(1.778478E37F);
		p231.wind_y_SET(-6.4305944E36F);
		p231.time_usec_SET(8724745105848360638L);
		p231.horiz_accuracy_SET(1.4743841E38F);
		p231.vert_accuracy_SET(-5.0447834E37F);
		p231.var_horiz_SET(6.2395547E37F);
		p231.wind_z_SET(-1.3274519E38F);
		p231.wind_alt_SET(-1.8047602E38F);
		p231.var_vert_SET(2.5967918E38F);
		CommunicationChannel.instance.send(p231);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
		TestChannel.instance.on_GPS_INPUT.add((src, ph, pack) ->
		{
			assert (pack.vdop_GET() == 1.0428447E38F);
			assert (pack.hdop_GET() == -2.8652155E38F);
			assert (pack.vd_GET() == 1.7316304E38F);
			assert (pack.speed_accuracy_GET() == 8.0766076E37F);
			assert (pack.ignore_flags_GET() == GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VDOP);
			assert (pack.time_usec_GET() == 3466278757155960423L);
			assert (pack.lon_GET() == -1346917905);
			assert (pack.ve_GET() == 2.223269E37F);
			assert (pack.vn_GET() == -9.14659E37F);
			assert (pack.lat_GET() == 1822089432);
			assert (pack.satellites_visible_GET() == (char) 140);
			assert (pack.gps_id_GET() == (char) 156);
			assert (pack.horiz_accuracy_GET() == -1.3460991E38F);
			assert (pack.alt_GET() == 2.1704117E38F);
			assert (pack.vert_accuracy_GET() == -3.0251419E38F);
			assert (pack.fix_type_GET() == (char) 238);
			assert (pack.time_week_GET() == (char) 53057);
			assert (pack.time_week_ms_GET() == 2074042405L);
		});
		GroundControl.GPS_INPUT p232 = CommunicationChannel.new_GPS_INPUT();
		PH.setPack(p232);
		p232.horiz_accuracy_SET(-1.3460991E38F);
		p232.vn_SET(-9.14659E37F);
		p232.ignore_flags_SET(GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VDOP);
		p232.lon_SET(-1346917905);
		p232.ve_SET(2.223269E37F);
		p232.vdop_SET(1.0428447E38F);
		p232.time_week_ms_SET(2074042405L);
		p232.fix_type_SET((char) 238);
		p232.satellites_visible_SET((char) 140);
		p232.time_week_SET((char) 53057);
		p232.gps_id_SET((char) 156);
		p232.alt_SET(2.1704117E38F);
		p232.hdop_SET(-2.8652155E38F);
		p232.vd_SET(1.7316304E38F);
		p232.speed_accuracy_SET(8.0766076E37F);
		p232.lat_SET(1822089432);
		p232.time_usec_SET(3466278757155960423L);
		p232.vert_accuracy_SET(-3.0251419E38F);
		CommunicationChannel.instance.send(p232);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
		TestChannel.instance.on_GPS_RTCM_DATA.add((src, ph, pack) ->
		{
			assert (pack.flags_GET() == (char) 64);
			assert (Arrays.equals(pack.data__GET(), new char[]{(char) 1, (char) 50, (char) 74, (char) 4, (char) 101, (char) 221, (char) 19, (char) 115, (char) 10, (char) 183, (char) 102, (char) 214, (char) 37, (char) 66, (char) 108, (char) 177, (char) 70, (char) 199, (char) 176, (char) 232, (char) 49, (char) 183, (char) 12, (char) 251, (char) 112, (char) 31, (char) 53, (char) 74, (char) 121, (char) 202, (char) 213, (char) 118, (char) 242, (char) 51, (char) 245, (char) 100, (char) 211, (char) 178, (char) 101, (char) 171, (char) 53, (char) 158, (char) 232, (char) 121, (char) 5, (char) 195, (char) 149, (char) 111, (char) 157, (char) 96, (char) 186, (char) 227, (char) 232, (char) 156, (char) 229, (char) 93, (char) 194, (char) 164, (char) 178, (char) 96, (char) 127, (char) 195, (char) 118, (char) 140, (char) 158, (char) 152, (char) 198, (char) 47, (char) 27, (char) 235, (char) 133, (char) 52, (char) 90, (char) 155, (char) 170, (char) 208, (char) 247, (char) 246, (char) 147, (char) 188, (char) 78, (char) 6, (char) 145, (char) 88, (char) 161, (char) 247, (char) 244, (char) 85, (char) 21, (char) 65, (char) 92, (char) 165, (char) 158, (char) 222, (char) 31, (char) 27, (char) 154, (char) 160, (char) 44, (char) 29, (char) 67, (char) 202, (char) 24, (char) 147, (char) 208, (char) 27, (char) 178, (char) 46, (char) 119, (char) 76, (char) 94, (char) 26, (char) 180, (char) 86, (char) 111, (char) 188, (char) 94, (char) 163, (char) 6, (char) 136, (char) 155, (char) 166, (char) 38, (char) 79, (char) 215, (char) 35, (char) 95, (char) 141, (char) 183, (char) 20, (char) 14, (char) 156, (char) 94, (char) 126, (char) 174, (char) 83, (char) 183, (char) 7, (char) 5, (char) 233, (char) 247, (char) 194, (char) 223, (char) 204, (char) 254, (char) 105, (char) 11, (char) 154, (char) 56, (char) 13, (char) 211, (char) 63, (char) 23, (char) 79, (char) 59, (char) 28, (char) 0, (char) 242, (char) 221, (char) 7, (char) 160, (char) 140, (char) 105, (char) 63, (char) 123, (char) 217, (char) 35, (char) 3, (char) 97, (char) 50, (char) 177, (char) 217, (char) 187, (char) 186, (char) 163, (char) 113, (char) 227, (char) 131, (char) 245, (char) 115}));
			assert (pack.len_GET() == (char) 195);
		});
		GroundControl.GPS_RTCM_DATA p233 = CommunicationChannel.new_GPS_RTCM_DATA();
		PH.setPack(p233);
		p233.data__SET(new char[]{(char) 1, (char) 50, (char) 74, (char) 4, (char) 101, (char) 221, (char) 19, (char) 115, (char) 10, (char) 183, (char) 102, (char) 214, (char) 37, (char) 66, (char) 108, (char) 177, (char) 70, (char) 199, (char) 176, (char) 232, (char) 49, (char) 183, (char) 12, (char) 251, (char) 112, (char) 31, (char) 53, (char) 74, (char) 121, (char) 202, (char) 213, (char) 118, (char) 242, (char) 51, (char) 245, (char) 100, (char) 211, (char) 178, (char) 101, (char) 171, (char) 53, (char) 158, (char) 232, (char) 121, (char) 5, (char) 195, (char) 149, (char) 111, (char) 157, (char) 96, (char) 186, (char) 227, (char) 232, (char) 156, (char) 229, (char) 93, (char) 194, (char) 164, (char) 178, (char) 96, (char) 127, (char) 195, (char) 118, (char) 140, (char) 158, (char) 152, (char) 198, (char) 47, (char) 27, (char) 235, (char) 133, (char) 52, (char) 90, (char) 155, (char) 170, (char) 208, (char) 247, (char) 246, (char) 147, (char) 188, (char) 78, (char) 6, (char) 145, (char) 88, (char) 161, (char) 247, (char) 244, (char) 85, (char) 21, (char) 65, (char) 92, (char) 165, (char) 158, (char) 222, (char) 31, (char) 27, (char) 154, (char) 160, (char) 44, (char) 29, (char) 67, (char) 202, (char) 24, (char) 147, (char) 208, (char) 27, (char) 178, (char) 46, (char) 119, (char) 76, (char) 94, (char) 26, (char) 180, (char) 86, (char) 111, (char) 188, (char) 94, (char) 163, (char) 6, (char) 136, (char) 155, (char) 166, (char) 38, (char) 79, (char) 215, (char) 35, (char) 95, (char) 141, (char) 183, (char) 20, (char) 14, (char) 156, (char) 94, (char) 126, (char) 174, (char) 83, (char) 183, (char) 7, (char) 5, (char) 233, (char) 247, (char) 194, (char) 223, (char) 204, (char) 254, (char) 105, (char) 11, (char) 154, (char) 56, (char) 13, (char) 211, (char) 63, (char) 23, (char) 79, (char) 59, (char) 28, (char) 0, (char) 242, (char) 221, (char) 7, (char) 160, (char) 140, (char) 105, (char) 63, (char) 123, (char) 217, (char) 35, (char) 3, (char) 97, (char) 50, (char) 177, (char) 217, (char) 187, (char) 186, (char) 163, (char) 113, (char) 227, (char) 131, (char) 245, (char) 115}, 0);
		p233.len_SET((char) 195);
		p233.flags_SET((char) 64);
		CommunicationChannel.instance.send(p233);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
		TestChannel.instance.on_HIGH_LATENCY.add((src, ph, pack) ->
		{
			assert (pack.base_mode_GET() == MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED);
			assert (pack.altitude_sp_GET() == (short) 3529);
			assert (pack.airspeed_sp_GET() == (char) 35);
			assert (pack.wp_num_GET() == (char) 34);
			assert (pack.temperature_air_GET() == (byte) -81);
			assert (pack.airspeed_GET() == (char) 18);
			assert (pack.throttle_GET() == (byte) 87);
			assert (pack.gps_fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_PPP);
			assert (pack.pitch_GET() == (short) 18806);
			assert (pack.battery_remaining_GET() == (char) 110);
			assert (pack.altitude_amsl_GET() == (short) -15937);
			assert (pack.latitude_GET() == 1211364316);
			assert (pack.climb_rate_GET() == (byte) 48);
			assert (pack.heading_GET() == (char) 39052);
			assert (pack.temperature_GET() == (byte) -94);
			assert (pack.gps_nsat_GET() == (char) 104);
			assert (pack.groundspeed_GET() == (char) 146);
			assert (pack.wp_distance_GET() == (char) 24826);
			assert (pack.heading_sp_GET() == (short) -6166);
			assert (pack.roll_GET() == (short) 24384);
			assert (pack.custom_mode_GET() == 1723670365L);
			assert (pack.longitude_GET() == -1017682615);
			assert (pack.landed_state_GET() == MAV_LANDED_STATE.MAV_LANDED_STATE_TAKEOFF);
			assert (pack.failsafe_GET() == (char) 108);
		});
		GroundControl.HIGH_LATENCY p234 = CommunicationChannel.new_HIGH_LATENCY();
		PH.setPack(p234);
		p234.custom_mode_SET(1723670365L);
		p234.wp_distance_SET((char) 24826);
		p234.heading_SET((char) 39052);
		p234.temperature_SET((byte) -94);
		p234.temperature_air_SET((byte) -81);
		p234.throttle_SET((byte) 87);
		p234.climb_rate_SET((byte) 48);
		p234.groundspeed_SET((char) 146);
		p234.gps_nsat_SET((char) 104);
		p234.airspeed_SET((char) 18);
		p234.roll_SET((short) 24384);
		p234.base_mode_SET(MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED);
		p234.pitch_SET((short) 18806);
		p234.failsafe_SET((char) 108);
		p234.heading_sp_SET((short) -6166);
		p234.airspeed_sp_SET((char) 35);
		p234.altitude_sp_SET((short) 3529);
		p234.gps_fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_PPP);
		p234.wp_num_SET((char) 34);
		p234.landed_state_SET(MAV_LANDED_STATE.MAV_LANDED_STATE_TAKEOFF);
		p234.latitude_SET(1211364316);
		p234.battery_remaining_SET((char) 110);
		p234.longitude_SET(-1017682615);
		p234.altitude_amsl_SET((short) -15937);
		CommunicationChannel.instance.send(p234);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
		TestChannel.instance.on_VIBRATION.add((src, ph, pack) ->
		{
			assert (pack.time_usec_GET() == 785304465993843027L);
			assert (pack.clipping_1_GET() == 3503757990L);
			assert (pack.clipping_0_GET() == 598074677L);
			assert (pack.vibration_z_GET() == 6.1934437E37F);
			assert (pack.vibration_x_GET() == 3.3264308E38F);
			assert (pack.vibration_y_GET() == -2.1268709E38F);
			assert (pack.clipping_2_GET() == 1922177581L);
		});
		GroundControl.VIBRATION p241 = CommunicationChannel.new_VIBRATION();
		PH.setPack(p241);
		p241.time_usec_SET(785304465993843027L);
		p241.clipping_2_SET(1922177581L);
		p241.clipping_1_SET(3503757990L);
		p241.vibration_z_SET(6.1934437E37F);
		p241.vibration_y_SET(-2.1268709E38F);
		p241.clipping_0_SET(598074677L);
		p241.vibration_x_SET(3.3264308E38F);
		CommunicationChannel.instance.send(p241);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
		TestChannel.instance.on_HOME_POSITION.add((src, ph, pack) ->
		{
			assert (pack.approach_x_GET() == -6.090909E37F);
			assert (pack.y_GET() == 2.5201768E38F);
			assert (pack.latitude_GET() == 857305171);
			assert (pack.x_GET() == -6.2781547E37F);
			assert (pack.altitude_GET() == 1479844569);
			assert (pack.z_GET() == 2.845638E38F);
			assert (pack.time_usec_TRY(ph) == 6382471993531817518L);
			assert (pack.longitude_GET() == 132782401);
			assert (pack.approach_y_GET() == -2.722585E38F);
			assert (pack.approach_z_GET() == 1.0449929E38F);
			assert (Arrays.equals(pack.q_GET(), new float[]{3.0952981E38F, 2.0037516E38F, -8.755772E37F, -2.8967883E38F}));
		});
		GroundControl.HOME_POSITION p242 = CommunicationChannel.new_HOME_POSITION();
		PH.setPack(p242);
		p242.z_SET(2.845638E38F);
		p242.time_usec_SET(6382471993531817518L, PH);
		p242.x_SET(-6.2781547E37F);
		p242.approach_y_SET(-2.722585E38F);
		p242.approach_x_SET(-6.090909E37F);
		p242.latitude_SET(857305171);
		p242.y_SET(2.5201768E38F);
		p242.altitude_SET(1479844569);
		p242.approach_z_SET(1.0449929E38F);
		p242.q_SET(new float[]{3.0952981E38F, 2.0037516E38F, -8.755772E37F, -2.8967883E38F}, 0);
		p242.longitude_SET(132782401);
		CommunicationChannel.instance.send(p242);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
		TestChannel.instance.on_SET_HOME_POSITION.add((src, ph, pack) ->
		{
			assert (pack.z_GET() == 2.6342337E38F);
			assert (pack.y_GET() == -2.3728851E38F);
			assert (pack.longitude_GET() == -2083819102);
			assert (pack.latitude_GET() == -1748089173);
			assert (pack.approach_x_GET() == 1.4010845E38F);
			assert (pack.target_system_GET() == (char) 214);
			assert (Arrays.equals(pack.q_GET(), new float[]{6.3527706E37F, -8.916927E37F, 1.140149E38F, 2.6423155E37F}));
			assert (pack.altitude_GET() == 1521022306);
			assert (pack.x_GET() == 5.0976553E36F);
			assert (pack.approach_y_GET() == 2.2369313E38F);
			assert (pack.approach_z_GET() == -2.3543136E38F);
			assert (pack.time_usec_TRY(ph) == 7933893855116843748L);
		});
		GroundControl.SET_HOME_POSITION p243 = CommunicationChannel.new_SET_HOME_POSITION();
		PH.setPack(p243);
		p243.longitude_SET(-2083819102);
		p243.approach_y_SET(2.2369313E38F);
		p243.q_SET(new float[]{6.3527706E37F, -8.916927E37F, 1.140149E38F, 2.6423155E37F}, 0);
		p243.altitude_SET(1521022306);
		p243.latitude_SET(-1748089173);
		p243.target_system_SET((char) 214);
		p243.z_SET(2.6342337E38F);
		p243.time_usec_SET(7933893855116843748L, PH);
		p243.approach_x_SET(1.4010845E38F);
		p243.y_SET(-2.3728851E38F);
		p243.approach_z_SET(-2.3543136E38F);
		p243.x_SET(5.0976553E36F);
		CommunicationChannel.instance.send(p243);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
		TestChannel.instance.on_MESSAGE_INTERVAL.add((src, ph, pack) ->
		{
			assert (pack.interval_us_GET() == 1388427307);
			assert (pack.message_id_GET() == (char) 43588);
		});
		GroundControl.MESSAGE_INTERVAL p244 = CommunicationChannel.new_MESSAGE_INTERVAL();
		PH.setPack(p244);
		p244.interval_us_SET(1388427307);
		p244.message_id_SET((char) 43588);
		CommunicationChannel.instance.send(p244);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
		TestChannel.instance.on_EXTENDED_SYS_STATE.add((src, ph, pack) ->
		{
			assert (pack.vtol_state_GET() == MAV_VTOL_STATE.MAV_VTOL_STATE_MC);
			assert (pack.landed_state_GET() == MAV_LANDED_STATE.MAV_LANDED_STATE_UNDEFINED);
		});
		GroundControl.EXTENDED_SYS_STATE p245 = CommunicationChannel.new_EXTENDED_SYS_STATE();
		PH.setPack(p245);
		p245.vtol_state_SET(MAV_VTOL_STATE.MAV_VTOL_STATE_MC);
		p245.landed_state_SET(MAV_LANDED_STATE.MAV_LANDED_STATE_UNDEFINED);
		CommunicationChannel.instance.send(p245);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
		TestChannel.instance.on_ADSB_VEHICLE.add((src, ph, pack) ->
		{
			assert (pack.lon_GET() == 2122592445);
			assert (pack.heading_GET() == (char) 56747);
			assert (pack.altitude_type_GET() == ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_GEOMETRIC);
			assert (pack.ICAO_address_GET() == 2490238991L);
			assert (pack.tslc_GET() == (char) 201);
			assert (pack.flags_GET() == ADSB_FLAGS.ADSB_FLAGS_VALID_HEADING);
			assert (pack.callsign_LEN(ph) == 4);
			assert (pack.callsign_TRY(ph).equals("drjz"));
			assert (pack.hor_velocity_GET() == (char) 50283);
			assert (pack.squawk_GET() == (char) 2988);
			assert (pack.emitter_type_GET() == ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_HIGHLY_MANUV);
			assert (pack.ver_velocity_GET() == (short) 18390);
			assert (pack.altitude_GET() == -916460071);
			assert (pack.lat_GET() == -626297889);
		});
		GroundControl.ADSB_VEHICLE p246 = CommunicationChannel.new_ADSB_VEHICLE();
		PH.setPack(p246);
		p246.flags_SET(ADSB_FLAGS.ADSB_FLAGS_VALID_HEADING);
		p246.lon_SET(2122592445);
		p246.ICAO_address_SET(2490238991L);
		p246.tslc_SET((char) 201);
		p246.emitter_type_SET(ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_HIGHLY_MANUV);
		p246.ver_velocity_SET((short) 18390);
		p246.altitude_type_SET(ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_GEOMETRIC);
		p246.lat_SET(-626297889);
		p246.hor_velocity_SET((char) 50283);
		p246.callsign_SET("drjz", PH);
		p246.altitude_SET(-916460071);
		p246.heading_SET((char) 56747);
		p246.squawk_SET((char) 2988);
		CommunicationChannel.instance.send(p246);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
		TestChannel.instance.on_COLLISION.add((src, ph, pack) ->
		{
			assert (pack.threat_level_GET() == MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_NONE);
			assert (pack.action_GET() == MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_NONE);
			assert (pack.id_GET() == 1859638819L);
			assert (pack.time_to_minimum_delta_GET() == -3.3781663E38F);
			assert (pack.horizontal_minimum_delta_GET() == 1.3445772E38F);
			assert (pack.src__GET() == MAV_COLLISION_SRC.MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT);
			assert (pack.altitude_minimum_delta_GET() == 2.1964166E38F);
		});
		GroundControl.COLLISION p247 = CommunicationChannel.new_COLLISION();
		PH.setPack(p247);
		p247.src__SET(MAV_COLLISION_SRC.MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT);
		p247.altitude_minimum_delta_SET(2.1964166E38F);
		p247.action_SET(MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_NONE);
		p247.threat_level_SET(MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_NONE);
		p247.id_SET(1859638819L);
		p247.horizontal_minimum_delta_SET(1.3445772E38F);
		p247.time_to_minimum_delta_SET(-3.3781663E38F);
		CommunicationChannel.instance.send(p247);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
		TestChannel.instance.on_V2_EXTENSION.add((src, ph, pack) ->
		{
			assert (pack.target_network_GET() == (char) 64);
			assert (pack.target_system_GET() == (char) 109);
			assert (pack.target_component_GET() == (char) 36);
			assert (Arrays.equals(pack.payload_GET(), new char[]{(char) 218, (char) 119, (char) 253, (char) 212, (char) 91, (char) 86, (char) 104, (char) 125, (char) 224, (char) 247, (char) 254, (char) 17, (char) 89, (char) 142, (char) 77, (char) 74, (char) 124, (char) 207, (char) 222, (char) 104, (char) 227, (char) 48, (char) 224, (char) 120, (char) 6, (char) 200, (char) 109, (char) 130, (char) 162, (char) 241, (char) 55, (char) 63, (char) 213, (char) 82, (char) 141, (char) 163, (char) 20, (char) 12, (char) 139, (char) 135, (char) 151, (char) 29, (char) 1, (char) 144, (char) 29, (char) 139, (char) 28, (char) 95, (char) 112, (char) 138, (char) 149, (char) 11, (char) 54, (char) 231, (char) 78, (char) 218, (char) 151, (char) 197, (char) 190, (char) 207, (char) 136, (char) 16, (char) 90, (char) 212, (char) 208, (char) 91, (char) 124, (char) 140, (char) 96, (char) 206, (char) 253, (char) 153, (char) 83, (char) 117, (char) 135, (char) 111, (char) 165, (char) 103, (char) 217, (char) 128, (char) 106, (char) 227, (char) 117, (char) 153, (char) 231, (char) 218, (char) 179, (char) 254, (char) 153, (char) 151, (char) 70, (char) 120, (char) 247, (char) 139, (char) 15, (char) 41, (char) 165, (char) 61, (char) 59, (char) 188, (char) 40, (char) 103, (char) 139, (char) 139, (char) 139, (char) 44, (char) 121, (char) 34, (char) 196, (char) 84, (char) 234, (char) 85, (char) 186, (char) 84, (char) 68, (char) 26, (char) 179, (char) 12, (char) 93, (char) 230, (char) 65, (char) 80, (char) 187, (char) 205, (char) 143, (char) 121, (char) 163, (char) 132, (char) 44, (char) 173, (char) 114, (char) 223, (char) 229, (char) 192, (char) 128, (char) 95, (char) 118, (char) 55, (char) 234, (char) 223, (char) 28, (char) 73, (char) 104, (char) 77, (char) 23, (char) 96, (char) 82, (char) 77, (char) 140, (char) 178, (char) 162, (char) 210, (char) 11, (char) 30, (char) 75, (char) 216, (char) 97, (char) 141, (char) 59, (char) 43, (char) 138, (char) 12, (char) 89, (char) 159, (char) 188, (char) 108, (char) 38, (char) 104, (char) 126, (char) 136, (char) 128, (char) 195, (char) 169, (char) 122, (char) 212, (char) 155, (char) 202, (char) 21, (char) 188, (char) 193, (char) 189, (char) 53, (char) 133, (char) 47, (char) 26, (char) 242, (char) 77, (char) 135, (char) 148, (char) 149, (char) 245, (char) 131, (char) 2, (char) 12, (char) 125, (char) 78, (char) 121, (char) 60, (char) 56, (char) 170, (char) 92, (char) 158, (char) 235, (char) 55, (char) 76, (char) 134, (char) 120, (char) 147, (char) 161, (char) 104, (char) 48, (char) 134, (char) 139, (char) 163, (char) 120, (char) 167, (char) 246, (char) 81, (char) 158, (char) 228, (char) 33, (char) 90, (char) 154, (char) 85, (char) 137, (char) 101, (char) 17, (char) 75, (char) 25, (char) 245, (char) 78, (char) 89, (char) 184, (char) 119, (char) 208, (char) 205, (char) 70, (char) 136, (char) 39, (char) 212, (char) 185, (char) 159, (char) 118, (char) 123, (char) 144, (char) 101, (char) 231, (char) 49, (char) 97}));
			assert (pack.message_type_GET() == (char) 62053);
		});
		GroundControl.V2_EXTENSION p248 = CommunicationChannel.new_V2_EXTENSION();
		PH.setPack(p248);
		p248.target_component_SET((char) 36);
		p248.target_network_SET((char) 64);
		p248.payload_SET(new char[]{(char) 218, (char) 119, (char) 253, (char) 212, (char) 91, (char) 86, (char) 104, (char) 125, (char) 224, (char) 247, (char) 254, (char) 17, (char) 89, (char) 142, (char) 77, (char) 74, (char) 124, (char) 207, (char) 222, (char) 104, (char) 227, (char) 48, (char) 224, (char) 120, (char) 6, (char) 200, (char) 109, (char) 130, (char) 162, (char) 241, (char) 55, (char) 63, (char) 213, (char) 82, (char) 141, (char) 163, (char) 20, (char) 12, (char) 139, (char) 135, (char) 151, (char) 29, (char) 1, (char) 144, (char) 29, (char) 139, (char) 28, (char) 95, (char) 112, (char) 138, (char) 149, (char) 11, (char) 54, (char) 231, (char) 78, (char) 218, (char) 151, (char) 197, (char) 190, (char) 207, (char) 136, (char) 16, (char) 90, (char) 212, (char) 208, (char) 91, (char) 124, (char) 140, (char) 96, (char) 206, (char) 253, (char) 153, (char) 83, (char) 117, (char) 135, (char) 111, (char) 165, (char) 103, (char) 217, (char) 128, (char) 106, (char) 227, (char) 117, (char) 153, (char) 231, (char) 218, (char) 179, (char) 254, (char) 153, (char) 151, (char) 70, (char) 120, (char) 247, (char) 139, (char) 15, (char) 41, (char) 165, (char) 61, (char) 59, (char) 188, (char) 40, (char) 103, (char) 139, (char) 139, (char) 139, (char) 44, (char) 121, (char) 34, (char) 196, (char) 84, (char) 234, (char) 85, (char) 186, (char) 84, (char) 68, (char) 26, (char) 179, (char) 12, (char) 93, (char) 230, (char) 65, (char) 80, (char) 187, (char) 205, (char) 143, (char) 121, (char) 163, (char) 132, (char) 44, (char) 173, (char) 114, (char) 223, (char) 229, (char) 192, (char) 128, (char) 95, (char) 118, (char) 55, (char) 234, (char) 223, (char) 28, (char) 73, (char) 104, (char) 77, (char) 23, (char) 96, (char) 82, (char) 77, (char) 140, (char) 178, (char) 162, (char) 210, (char) 11, (char) 30, (char) 75, (char) 216, (char) 97, (char) 141, (char) 59, (char) 43, (char) 138, (char) 12, (char) 89, (char) 159, (char) 188, (char) 108, (char) 38, (char) 104, (char) 126, (char) 136, (char) 128, (char) 195, (char) 169, (char) 122, (char) 212, (char) 155, (char) 202, (char) 21, (char) 188, (char) 193, (char) 189, (char) 53, (char) 133, (char) 47, (char) 26, (char) 242, (char) 77, (char) 135, (char) 148, (char) 149, (char) 245, (char) 131, (char) 2, (char) 12, (char) 125, (char) 78, (char) 121, (char) 60, (char) 56, (char) 170, (char) 92, (char) 158, (char) 235, (char) 55, (char) 76, (char) 134, (char) 120, (char) 147, (char) 161, (char) 104, (char) 48, (char) 134, (char) 139, (char) 163, (char) 120, (char) 167, (char) 246, (char) 81, (char) 158, (char) 228, (char) 33, (char) 90, (char) 154, (char) 85, (char) 137, (char) 101, (char) 17, (char) 75, (char) 25, (char) 245, (char) 78, (char) 89, (char) 184, (char) 119, (char) 208, (char) 205, (char) 70, (char) 136, (char) 39, (char) 212, (char) 185, (char) 159, (char) 118, (char) 123, (char) 144, (char) 101, (char) 231, (char) 49, (char) 97}, 0);
		p248.message_type_SET((char) 62053);
		p248.target_system_SET((char) 109);
		CommunicationChannel.instance.send(p248);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
		TestChannel.instance.on_MEMORY_VECT.add((src, ph, pack) ->
		{
			assert (Arrays.equals(pack.value_GET(), new byte[]{(byte) 42, (byte) -87, (byte) -11, (byte) 106, (byte) -54, (byte) 54, (byte) -10, (byte) 80, (byte) 20, (byte) 52, (byte) -72, (byte) -9, (byte) 7, (byte) 43, (byte) -13, (byte) 55, (byte) -105, (byte) -77, (byte) -24, (byte) -65, (byte) 18, (byte) -86, (byte) -44, (byte) -94, (byte) 0, (byte) -16, (byte) 11, (byte) 77, (byte) 96, (byte) -108, (byte) 20, (byte) -103}));
			assert (pack.type_GET() == (char) 209);
			assert (pack.address_GET() == (char) 50544);
			assert (pack.ver_GET() == (char) 220);
		});
		GroundControl.MEMORY_VECT p249 = CommunicationChannel.new_MEMORY_VECT();
		PH.setPack(p249);
		p249.type_SET((char) 209);
		p249.value_SET(new byte[]{(byte) 42, (byte) -87, (byte) -11, (byte) 106, (byte) -54, (byte) 54, (byte) -10, (byte) 80, (byte) 20, (byte) 52, (byte) -72, (byte) -9, (byte) 7, (byte) 43, (byte) -13, (byte) 55, (byte) -105, (byte) -77, (byte) -24, (byte) -65, (byte) 18, (byte) -86, (byte) -44, (byte) -94, (byte) 0, (byte) -16, (byte) 11, (byte) 77, (byte) 96, (byte) -108, (byte) 20, (byte) -103}, 0);
		p249.address_SET((char) 50544);
		p249.ver_SET((char) 220);
		CommunicationChannel.instance.send(p249);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
		TestChannel.instance.on_DEBUG_VECT.add((src, ph, pack) ->
		{
			assert (pack.y_GET() == 1.5441731E38F);
			assert (pack.time_usec_GET() == 9124195598056743699L);
			assert (pack.x_GET() == 1.8744782E38F);
			assert (pack.name_LEN(ph) == 1);
			assert (pack.name_TRY(ph).equals("t"));
			assert (pack.z_GET() == 1.2841764E38F);
		});
		GroundControl.DEBUG_VECT p250 = CommunicationChannel.new_DEBUG_VECT();
		PH.setPack(p250);
		p250.time_usec_SET(9124195598056743699L);
		p250.x_SET(1.8744782E38F);
		p250.y_SET(1.5441731E38F);
		p250.z_SET(1.2841764E38F);
		p250.name_SET("t", PH);
		CommunicationChannel.instance.send(p250);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
		TestChannel.instance.on_NAMED_VALUE_FLOAT.add((src, ph, pack) ->
		{
			assert (pack.time_boot_ms_GET() == 804715261L);
			assert (pack.value_GET() == 1.9843995E38F);
			assert (pack.name_LEN(ph) == 10);
			assert (pack.name_TRY(ph).equals("uafdNxucmh"));
		});
		GroundControl.NAMED_VALUE_FLOAT p251 = CommunicationChannel.new_NAMED_VALUE_FLOAT();
		PH.setPack(p251);
		p251.time_boot_ms_SET(804715261L);
		p251.name_SET("uafdNxucmh", PH);
		p251.value_SET(1.9843995E38F);
		CommunicationChannel.instance.send(p251);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
		TestChannel.instance.on_NAMED_VALUE_INT.add((src, ph, pack) ->
		{
			assert (pack.name_LEN(ph) == 5);
			assert (pack.name_TRY(ph).equals("Cdlki"));
			assert (pack.value_GET() == -491261654);
			assert (pack.time_boot_ms_GET() == 4032976372L);
		});
		GroundControl.NAMED_VALUE_INT p252 = CommunicationChannel.new_NAMED_VALUE_INT();
		PH.setPack(p252);
		p252.value_SET(-491261654);
		p252.name_SET("Cdlki", PH);
		p252.time_boot_ms_SET(4032976372L);
		CommunicationChannel.instance.send(p252);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
		TestChannel.instance.on_STATUSTEXT.add((src, ph, pack) ->
		{
			assert (pack.text_LEN(ph) == 19);
			assert (pack.text_TRY(ph).equals("qrogYbzzuuosxlUygrq"));
			assert (pack.severity_GET() == MAV_SEVERITY.MAV_SEVERITY_WARNING);
		});
		GroundControl.STATUSTEXT p253 = CommunicationChannel.new_STATUSTEXT();
		PH.setPack(p253);
		p253.text_SET("qrogYbzzuuosxlUygrq", PH);
		p253.severity_SET(MAV_SEVERITY.MAV_SEVERITY_WARNING);
		CommunicationChannel.instance.send(p253);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
		TestChannel.instance.on_DEBUG.add((src, ph, pack) ->
		{
			assert (pack.ind_GET() == (char) 154);
			assert (pack.time_boot_ms_GET() == 791113428L);
			assert (pack.value_GET() == -1.0790364E38F);
		});
		GroundControl.DEBUG p254 = CommunicationChannel.new_DEBUG();
		PH.setPack(p254);
		p254.time_boot_ms_SET(791113428L);
		p254.value_SET(-1.0790364E38F);
		p254.ind_SET((char) 154);
		CommunicationChannel.instance.send(p254);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
		TestChannel.instance.on_SETUP_SIGNING.add((src, ph, pack) ->
		{
			assert (pack.target_component_GET() == (char) 237);
			assert (Arrays.equals(pack.secret_key_GET(), new char[]{(char) 126, (char) 47, (char) 32, (char) 161, (char) 160, (char) 206, (char) 237, (char) 151, (char) 12, (char) 76, (char) 129, (char) 60, (char) 32, (char) 80, (char) 145, (char) 72, (char) 223, (char) 72, (char) 147, (char) 187, (char) 204, (char) 99, (char) 238, (char) 111, (char) 203, (char) 46, (char) 86, (char) 31, (char) 232, (char) 129, (char) 27, (char) 195}));
			assert (pack.target_system_GET() == (char) 147);
			assert (pack.initial_timestamp_GET() == 4315986876366698307L);
		});
		GroundControl.SETUP_SIGNING p256 = CommunicationChannel.new_SETUP_SIGNING();
		PH.setPack(p256);
		p256.target_system_SET((char) 147);
		p256.initial_timestamp_SET(4315986876366698307L);
		p256.target_component_SET((char) 237);
		p256.secret_key_SET(new char[]{(char) 126, (char) 47, (char) 32, (char) 161, (char) 160, (char) 206, (char) 237, (char) 151, (char) 12, (char) 76, (char) 129, (char) 60, (char) 32, (char) 80, (char) 145, (char) 72, (char) 223, (char) 72, (char) 147, (char) 187, (char) 204, (char) 99, (char) 238, (char) 111, (char) 203, (char) 46, (char) 86, (char) 31, (char) 232, (char) 129, (char) 27, (char) 195}, 0);
		CommunicationChannel.instance.send(p256);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
		TestChannel.instance.on_BUTTON_CHANGE.add((src, ph, pack) ->
		{
			assert (pack.time_boot_ms_GET() == 3486258363L);
			assert (pack.last_change_ms_GET() == 925741145L);
			assert (pack.state_GET() == (char) 160);
		});
		GroundControl.BUTTON_CHANGE p257 = CommunicationChannel.new_BUTTON_CHANGE();
		PH.setPack(p257);
		p257.time_boot_ms_SET(3486258363L);
		p257.last_change_ms_SET(925741145L);
		p257.state_SET((char) 160);
		CommunicationChannel.instance.send(p257);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
		TestChannel.instance.on_PLAY_TUNE.add((src, ph, pack) ->
		{
			assert (pack.target_component_GET() == (char) 164);
			assert (pack.tune_LEN(ph) == 28);
			assert (pack.tune_TRY(ph).equals("iuLTbmafpkvuObdbmKblvwsIrjop"));
			assert (pack.target_system_GET() == (char) 162);
		});
		GroundControl.PLAY_TUNE p258 = CommunicationChannel.new_PLAY_TUNE();
		PH.setPack(p258);
		p258.tune_SET("iuLTbmafpkvuObdbmKblvwsIrjop", PH);
		p258.target_component_SET((char) 164);
		p258.target_system_SET((char) 162);
		CommunicationChannel.instance.send(p258);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
		TestChannel.instance.on_CAMERA_INFORMATION.add((src, ph, pack) ->
		{
			assert (pack.lens_id_GET() == (char) 100);
			assert (pack.flags_GET() == CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE);
			assert (Arrays.equals(pack.model_name_GET(), new char[]{(char) 150, (char) 165, (char) 201, (char) 236, (char) 79, (char) 73, (char) 149, (char) 10, (char) 144, (char) 34, (char) 107, (char) 9, (char) 27, (char) 150, (char) 180, (char) 145, (char) 190, (char) 86, (char) 244, (char) 211, (char) 81, (char) 174, (char) 40, (char) 182, (char) 213, (char) 183, (char) 133, (char) 65, (char) 165, (char) 19, (char) 128, (char) 64}));
			assert (pack.sensor_size_h_GET() == 1.456356E38F);
			assert (pack.resolution_v_GET() == (char) 49754);
			assert (pack.resolution_h_GET() == (char) 13558);
			assert (pack.cam_definition_uri_LEN(ph) == 90);
			assert (pack.cam_definition_uri_TRY(ph).equals("zllTmixaylfjjcuLfsfuoSrrjcycrbfqNzeupfAezazuwqigxbQxbKozlygtofqyckwHcvukGkcZrjxzmvjtqmXqtb"));
			assert (pack.sensor_size_v_GET() == 2.2543807E38F);
			assert (pack.focal_length_GET() == -8.436742E37F);
			assert (pack.time_boot_ms_GET() == 3399713862L);
			assert (pack.cam_definition_version_GET() == (char) 26485);
			assert (pack.firmware_version_GET() == 2307330247L);
			assert (Arrays.equals(pack.vendor_name_GET(), new char[]{(char) 86, (char) 231, (char) 202, (char) 163, (char) 212, (char) 61, (char) 105, (char) 81, (char) 184, (char) 5, (char) 159, (char) 178, (char) 117, (char) 65, (char) 109, (char) 117, (char) 126, (char) 164, (char) 226, (char) 120, (char) 118, (char) 3, (char) 114, (char) 37, (char) 154, (char) 108, (char) 152, (char) 152, (char) 161, (char) 128, (char) 162, (char) 74}));
		});
		GroundControl.CAMERA_INFORMATION p259 = CommunicationChannel.new_CAMERA_INFORMATION();
		PH.setPack(p259);
		p259.resolution_h_SET((char) 13558);
		p259.flags_SET(CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE);
		p259.time_boot_ms_SET(3399713862L);
		p259.resolution_v_SET((char) 49754);
		p259.cam_definition_version_SET((char) 26485);
		p259.sensor_size_h_SET(1.456356E38F);
		p259.focal_length_SET(-8.436742E37F);
		p259.cam_definition_uri_SET("zllTmixaylfjjcuLfsfuoSrrjcycrbfqNzeupfAezazuwqigxbQxbKozlygtofqyckwHcvukGkcZrjxzmvjtqmXqtb", PH);
		p259.lens_id_SET((char) 100);
		p259.firmware_version_SET(2307330247L);
		p259.sensor_size_v_SET(2.2543807E38F);
		p259.model_name_SET(new char[]{(char) 150, (char) 165, (char) 201, (char) 236, (char) 79, (char) 73, (char) 149, (char) 10, (char) 144, (char) 34, (char) 107, (char) 9, (char) 27, (char) 150, (char) 180, (char) 145, (char) 190, (char) 86, (char) 244, (char) 211, (char) 81, (char) 174, (char) 40, (char) 182, (char) 213, (char) 183, (char) 133, (char) 65, (char) 165, (char) 19, (char) 128, (char) 64}, 0);
		p259.vendor_name_SET(new char[]{(char) 86, (char) 231, (char) 202, (char) 163, (char) 212, (char) 61, (char) 105, (char) 81, (char) 184, (char) 5, (char) 159, (char) 178, (char) 117, (char) 65, (char) 109, (char) 117, (char) 126, (char) 164, (char) 226, (char) 120, (char) 118, (char) 3, (char) 114, (char) 37, (char) 154, (char) 108, (char) 152, (char) 152, (char) 161, (char) 128, (char) 162, (char) 74}, 0);
		CommunicationChannel.instance.send(p259);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
		TestChannel.instance.on_CAMERA_SETTINGS.add((src, ph, pack) ->
		{
			assert (pack.mode_id_GET() == CAMERA_MODE.CAMERA_MODE_VIDEO);
			assert (pack.time_boot_ms_GET() == 3349200150L);
		});
		GroundControl.CAMERA_SETTINGS p260 = CommunicationChannel.new_CAMERA_SETTINGS();
		PH.setPack(p260);
		p260.time_boot_ms_SET(3349200150L);
		p260.mode_id_SET(CAMERA_MODE.CAMERA_MODE_VIDEO);
		CommunicationChannel.instance.send(p260);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
		TestChannel.instance.on_STORAGE_INFORMATION.add((src, ph, pack) ->
		{
			assert (pack.write_speed_GET() == -1.0277073E38F);
			assert (pack.storage_id_GET() == (char) 249);
			assert (pack.status_GET() == (char) 243);
			assert (pack.read_speed_GET() == -2.6396986E38F);
			assert (pack.total_capacity_GET() == 2.9745336E38F);
			assert (pack.used_capacity_GET() == -2.0769076E38F);
			assert (pack.time_boot_ms_GET() == 636516357L);
			assert (pack.available_capacity_GET() == 1.0296097E35F);
			assert (pack.storage_count_GET() == (char) 239);
		});
		GroundControl.STORAGE_INFORMATION p261 = CommunicationChannel.new_STORAGE_INFORMATION();
		PH.setPack(p261);
		p261.write_speed_SET(-1.0277073E38F);
		p261.status_SET((char) 243);
		p261.total_capacity_SET(2.9745336E38F);
		p261.time_boot_ms_SET(636516357L);
		p261.available_capacity_SET(1.0296097E35F);
		p261.read_speed_SET(-2.6396986E38F);
		p261.used_capacity_SET(-2.0769076E38F);
		p261.storage_id_SET((char) 249);
		p261.storage_count_SET((char) 239);
		CommunicationChannel.instance.send(p261);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
		TestChannel.instance.on_CAMERA_CAPTURE_STATUS.add((src, ph, pack) ->
		{
			assert (pack.time_boot_ms_GET() == 3292814726L);
			assert (pack.image_interval_GET() == 2.3630453E38F);
			assert (pack.recording_time_ms_GET() == 2637575406L);
			assert (pack.available_capacity_GET() == 2.5830955E38F);
			assert (pack.image_status_GET() == (char) 119);
			assert (pack.video_status_GET() == (char) 133);
		});
		GroundControl.CAMERA_CAPTURE_STATUS p262 = CommunicationChannel.new_CAMERA_CAPTURE_STATUS();
		PH.setPack(p262);
		p262.image_status_SET((char) 119);
		p262.video_status_SET((char) 133);
		p262.time_boot_ms_SET(3292814726L);
		p262.recording_time_ms_SET(2637575406L);
		p262.available_capacity_SET(2.5830955E38F);
		p262.image_interval_SET(2.3630453E38F);
		CommunicationChannel.instance.send(p262);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
		TestChannel.instance.on_CAMERA_IMAGE_CAPTURED.add((src, ph, pack) ->
		{
			assert (Arrays.equals(pack.q_GET(), new float[]{-2.6265578E38F, -7.8140604E37F, -1.5026565E38F, 1.4038852E38F}));
			assert (pack.time_boot_ms_GET() == 494106933L);
			assert (pack.file_url_LEN(ph) == 8);
			assert (pack.file_url_TRY(ph).equals("ybfenopm"));
			assert (pack.image_index_GET() == 159919923);
			assert (pack.camera_id_GET() == (char) 217);
			assert (pack.alt_GET() == -282144327);
			assert (pack.lon_GET() == -1089321260);
			assert (pack.relative_alt_GET() == 23623058);
			assert (pack.capture_result_GET() == (byte) -108);
			assert (pack.time_utc_GET() == 4245794070985110330L);
			assert (pack.lat_GET() == 634944162);
		});
		GroundControl.CAMERA_IMAGE_CAPTURED p263 = CommunicationChannel.new_CAMERA_IMAGE_CAPTURED();
		PH.setPack(p263);
		p263.time_utc_SET(4245794070985110330L);
		p263.image_index_SET(159919923);
		p263.alt_SET(-282144327);
		p263.time_boot_ms_SET(494106933L);
		p263.file_url_SET("ybfenopm", PH);
		p263.lon_SET(-1089321260);
		p263.q_SET(new float[]{-2.6265578E38F, -7.8140604E37F, -1.5026565E38F, 1.4038852E38F}, 0);
		p263.relative_alt_SET(23623058);
		p263.capture_result_SET((byte) -108);
		p263.camera_id_SET((char) 217);
		p263.lat_SET(634944162);
		CommunicationChannel.instance.send(p263);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
		TestChannel.instance.on_FLIGHT_INFORMATION.add((src, ph, pack) ->
		{
			assert (pack.flight_uuid_GET() == 8084824427380243930L);
			assert (pack.arming_time_utc_GET() == 5906150759147876969L);
			assert (pack.takeoff_time_utc_GET() == 4359738559789625613L);
			assert (pack.time_boot_ms_GET() == 1641522154L);
		});
		GroundControl.FLIGHT_INFORMATION p264 = CommunicationChannel.new_FLIGHT_INFORMATION();
		PH.setPack(p264);
		p264.takeoff_time_utc_SET(4359738559789625613L);
		p264.flight_uuid_SET(8084824427380243930L);
		p264.time_boot_ms_SET(1641522154L);
		p264.arming_time_utc_SET(5906150759147876969L);
		CommunicationChannel.instance.send(p264);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
		TestChannel.instance.on_MOUNT_ORIENTATION.add((src, ph, pack) ->
		{
			assert (pack.pitch_GET() == 8.4073605E36F);
			assert (pack.yaw_GET() == -8.516995E37F);
			assert (pack.roll_GET() == -3.3773262E38F);
			assert (pack.time_boot_ms_GET() == 3595179626L);
		});
		GroundControl.MOUNT_ORIENTATION p265 = CommunicationChannel.new_MOUNT_ORIENTATION();
		PH.setPack(p265);
		p265.pitch_SET(8.4073605E36F);
		p265.yaw_SET(-8.516995E37F);
		p265.roll_SET(-3.3773262E38F);
		p265.time_boot_ms_SET(3595179626L);
		CommunicationChannel.instance.send(p265);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
		TestChannel.instance.on_LOGGING_DATA.add((src, ph, pack) ->
		{
			assert (pack.sequence_GET() == (char) 55624);
			assert (pack.target_component_GET() == (char) 95);
			assert (Arrays.equals(pack.data__GET(), new char[]{(char) 184, (char) 23, (char) 139, (char) 188, (char) 67, (char) 51, (char) 209, (char) 84, (char) 183, (char) 205, (char) 124, (char) 228, (char) 72, (char) 65, (char) 192, (char) 175, (char) 5, (char) 141, (char) 77, (char) 206, (char) 154, (char) 169, (char) 100, (char) 106, (char) 236, (char) 55, (char) 86, (char) 194, (char) 230, (char) 56, (char) 216, (char) 165, (char) 187, (char) 227, (char) 164, (char) 157, (char) 255, (char) 46, (char) 58, (char) 81, (char) 104, (char) 191, (char) 203, (char) 173, (char) 100, (char) 237, (char) 21, (char) 4, (char) 186, (char) 209, (char) 157, (char) 29, (char) 88, (char) 55, (char) 148, (char) 20, (char) 33, (char) 179, (char) 73, (char) 6, (char) 100, (char) 112, (char) 208, (char) 247, (char) 27, (char) 194, (char) 219, (char) 167, (char) 81, (char) 112, (char) 222, (char) 0, (char) 59, (char) 128, (char) 158, (char) 171, (char) 150, (char) 170, (char) 55, (char) 227, (char) 71, (char) 1, (char) 205, (char) 126, (char) 149, (char) 141, (char) 229, (char) 107, (char) 189, (char) 14, (char) 193, (char) 89, (char) 184, (char) 167, (char) 252, (char) 227, (char) 237, (char) 147, (char) 169, (char) 26, (char) 251, (char) 54, (char) 32, (char) 74, (char) 200, (char) 181, (char) 138, (char) 207, (char) 191, (char) 84, (char) 243, (char) 213, (char) 228, (char) 171, (char) 139, (char) 43, (char) 109, (char) 57, (char) 147, (char) 127, (char) 41, (char) 238, (char) 236, (char) 171, (char) 62, (char) 84, (char) 40, (char) 35, (char) 55, (char) 22, (char) 175, (char) 170, (char) 118, (char) 2, (char) 142, (char) 242, (char) 102, (char) 252, (char) 27, (char) 202, (char) 37, (char) 8, (char) 145, (char) 43, (char) 148, (char) 208, (char) 22, (char) 59, (char) 227, (char) 95, (char) 39, (char) 111, (char) 4, (char) 192, (char) 156, (char) 112, (char) 155, (char) 224, (char) 87, (char) 50, (char) 249, (char) 109, (char) 138, (char) 76, (char) 71, (char) 112, (char) 66, (char) 206, (char) 29, (char) 170, (char) 104, (char) 239, (char) 67, (char) 5, (char) 67, (char) 199, (char) 221, (char) 178, (char) 39, (char) 62, (char) 133, (char) 73, (char) 73, (char) 246, (char) 10, (char) 63, (char) 19, (char) 143, (char) 31, (char) 211, (char) 101, (char) 231, (char) 136, (char) 119, (char) 40, (char) 90, (char) 214, (char) 183, (char) 111, (char) 38, (char) 212, (char) 116, (char) 230, (char) 166, (char) 207, (char) 11, (char) 158, (char) 205, (char) 208, (char) 122, (char) 41, (char) 242, (char) 127, (char) 224, (char) 26, (char) 17, (char) 30, (char) 12, (char) 230, (char) 189, (char) 15, (char) 182, (char) 176, (char) 188, (char) 218, (char) 165, (char) 69, (char) 116, (char) 96, (char) 56, (char) 175, (char) 223, (char) 140, (char) 252, (char) 194, (char) 160, (char) 144, (char) 0, (char) 253, (char) 152, (char) 125, (char) 164, (char) 169, (char) 47, (char) 187, (char) 243, (char) 52, (char) 50, (char) 186}));
			assert (pack.target_system_GET() == (char) 197);
			assert (pack.first_message_offset_GET() == (char) 71);
			assert (pack.length_GET() == (char) 127);
		});
		GroundControl.LOGGING_DATA p266 = CommunicationChannel.new_LOGGING_DATA();
		PH.setPack(p266);
		p266.first_message_offset_SET((char) 71);
		p266.target_system_SET((char) 197);
		p266.target_component_SET((char) 95);
		p266.sequence_SET((char) 55624);
		p266.data__SET(new char[]{(char) 184, (char) 23, (char) 139, (char) 188, (char) 67, (char) 51, (char) 209, (char) 84, (char) 183, (char) 205, (char) 124, (char) 228, (char) 72, (char) 65, (char) 192, (char) 175, (char) 5, (char) 141, (char) 77, (char) 206, (char) 154, (char) 169, (char) 100, (char) 106, (char) 236, (char) 55, (char) 86, (char) 194, (char) 230, (char) 56, (char) 216, (char) 165, (char) 187, (char) 227, (char) 164, (char) 157, (char) 255, (char) 46, (char) 58, (char) 81, (char) 104, (char) 191, (char) 203, (char) 173, (char) 100, (char) 237, (char) 21, (char) 4, (char) 186, (char) 209, (char) 157, (char) 29, (char) 88, (char) 55, (char) 148, (char) 20, (char) 33, (char) 179, (char) 73, (char) 6, (char) 100, (char) 112, (char) 208, (char) 247, (char) 27, (char) 194, (char) 219, (char) 167, (char) 81, (char) 112, (char) 222, (char) 0, (char) 59, (char) 128, (char) 158, (char) 171, (char) 150, (char) 170, (char) 55, (char) 227, (char) 71, (char) 1, (char) 205, (char) 126, (char) 149, (char) 141, (char) 229, (char) 107, (char) 189, (char) 14, (char) 193, (char) 89, (char) 184, (char) 167, (char) 252, (char) 227, (char) 237, (char) 147, (char) 169, (char) 26, (char) 251, (char) 54, (char) 32, (char) 74, (char) 200, (char) 181, (char) 138, (char) 207, (char) 191, (char) 84, (char) 243, (char) 213, (char) 228, (char) 171, (char) 139, (char) 43, (char) 109, (char) 57, (char) 147, (char) 127, (char) 41, (char) 238, (char) 236, (char) 171, (char) 62, (char) 84, (char) 40, (char) 35, (char) 55, (char) 22, (char) 175, (char) 170, (char) 118, (char) 2, (char) 142, (char) 242, (char) 102, (char) 252, (char) 27, (char) 202, (char) 37, (char) 8, (char) 145, (char) 43, (char) 148, (char) 208, (char) 22, (char) 59, (char) 227, (char) 95, (char) 39, (char) 111, (char) 4, (char) 192, (char) 156, (char) 112, (char) 155, (char) 224, (char) 87, (char) 50, (char) 249, (char) 109, (char) 138, (char) 76, (char) 71, (char) 112, (char) 66, (char) 206, (char) 29, (char) 170, (char) 104, (char) 239, (char) 67, (char) 5, (char) 67, (char) 199, (char) 221, (char) 178, (char) 39, (char) 62, (char) 133, (char) 73, (char) 73, (char) 246, (char) 10, (char) 63, (char) 19, (char) 143, (char) 31, (char) 211, (char) 101, (char) 231, (char) 136, (char) 119, (char) 40, (char) 90, (char) 214, (char) 183, (char) 111, (char) 38, (char) 212, (char) 116, (char) 230, (char) 166, (char) 207, (char) 11, (char) 158, (char) 205, (char) 208, (char) 122, (char) 41, (char) 242, (char) 127, (char) 224, (char) 26, (char) 17, (char) 30, (char) 12, (char) 230, (char) 189, (char) 15, (char) 182, (char) 176, (char) 188, (char) 218, (char) 165, (char) 69, (char) 116, (char) 96, (char) 56, (char) 175, (char) 223, (char) 140, (char) 252, (char) 194, (char) 160, (char) 144, (char) 0, (char) 253, (char) 152, (char) 125, (char) 164, (char) 169, (char) 47, (char) 187, (char) 243, (char) 52, (char) 50, (char) 186}, 0);
		p266.length_SET((char) 127);
		CommunicationChannel.instance.send(p266);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
		TestChannel.instance.on_LOGGING_DATA_ACKED.add((src, ph, pack) ->
		{
			assert (pack.first_message_offset_GET() == (char) 156);
			assert (Arrays.equals(pack.data__GET(), new char[]{(char) 250, (char) 46, (char) 230, (char) 88, (char) 49, (char) 159, (char) 229, (char) 87, (char) 14, (char) 146, (char) 199, (char) 85, (char) 45, (char) 51, (char) 137, (char) 202, (char) 188, (char) 46, (char) 68, (char) 195, (char) 105, (char) 153, (char) 99, (char) 70, (char) 243, (char) 24, (char) 108, (char) 69, (char) 140, (char) 189, (char) 60, (char) 102, (char) 7, (char) 1, (char) 38, (char) 119, (char) 108, (char) 15, (char) 34, (char) 1, (char) 157, (char) 75, (char) 206, (char) 104, (char) 48, (char) 24, (char) 151, (char) 189, (char) 30, (char) 254, (char) 34, (char) 145, (char) 72, (char) 67, (char) 203, (char) 5, (char) 63, (char) 61, (char) 153, (char) 218, (char) 223, (char) 130, (char) 64, (char) 203, (char) 115, (char) 235, (char) 188, (char) 101, (char) 0, (char) 151, (char) 123, (char) 121, (char) 200, (char) 3, (char) 86, (char) 144, (char) 230, (char) 219, (char) 61, (char) 11, (char) 10, (char) 188, (char) 209, (char) 153, (char) 209, (char) 225, (char) 58, (char) 145, (char) 248, (char) 230, (char) 41, (char) 34, (char) 144, (char) 38, (char) 52, (char) 13, (char) 236, (char) 58, (char) 148, (char) 89, (char) 59, (char) 35, (char) 191, (char) 243, (char) 213, (char) 185, (char) 59, (char) 210, (char) 48, (char) 109, (char) 169, (char) 203, (char) 70, (char) 117, (char) 109, (char) 121, (char) 55, (char) 106, (char) 61, (char) 117, (char) 238, (char) 168, (char) 105, (char) 180, (char) 23, (char) 193, (char) 217, (char) 57, (char) 42, (char) 107, (char) 39, (char) 168, (char) 46, (char) 1, (char) 85, (char) 18, (char) 13, (char) 190, (char) 166, (char) 78, (char) 247, (char) 200, (char) 171, (char) 217, (char) 115, (char) 134, (char) 226, (char) 76, (char) 201, (char) 151, (char) 91, (char) 239, (char) 135, (char) 132, (char) 8, (char) 118, (char) 82, (char) 167, (char) 203, (char) 88, (char) 115, (char) 253, (char) 179, (char) 52, (char) 143, (char) 205, (char) 55, (char) 79, (char) 113, (char) 205, (char) 5, (char) 180, (char) 13, (char) 192, (char) 58, (char) 138, (char) 181, (char) 111, (char) 61, (char) 28, (char) 200, (char) 79, (char) 218, (char) 147, (char) 0, (char) 53, (char) 23, (char) 236, (char) 173, (char) 10, (char) 6, (char) 193, (char) 222, (char) 241, (char) 86, (char) 254, (char) 20, (char) 37, (char) 175, (char) 16, (char) 249, (char) 144, (char) 207, (char) 155, (char) 248, (char) 9, (char) 234, (char) 77, (char) 39, (char) 152, (char) 65, (char) 72, (char) 118, (char) 180, (char) 55, (char) 133, (char) 171, (char) 4, (char) 210, (char) 54, (char) 251, (char) 36, (char) 90, (char) 233, (char) 17, (char) 86, (char) 255, (char) 165, (char) 229, (char) 130, (char) 135, (char) 206, (char) 63, (char) 100, (char) 88, (char) 254, (char) 222, (char) 51, (char) 40, (char) 161, (char) 196, (char) 43, (char) 129, (char) 125, (char) 234, (char) 51, (char) 244, (char) 208, (char) 25}));
			assert (pack.target_component_GET() == (char) 238);
			assert (pack.length_GET() == (char) 16);
			assert (pack.target_system_GET() == (char) 109);
			assert (pack.sequence_GET() == (char) 47535);
		});
		GroundControl.LOGGING_DATA_ACKED p267 = CommunicationChannel.new_LOGGING_DATA_ACKED();
		PH.setPack(p267);
		p267.data__SET(new char[]{(char) 250, (char) 46, (char) 230, (char) 88, (char) 49, (char) 159, (char) 229, (char) 87, (char) 14, (char) 146, (char) 199, (char) 85, (char) 45, (char) 51, (char) 137, (char) 202, (char) 188, (char) 46, (char) 68, (char) 195, (char) 105, (char) 153, (char) 99, (char) 70, (char) 243, (char) 24, (char) 108, (char) 69, (char) 140, (char) 189, (char) 60, (char) 102, (char) 7, (char) 1, (char) 38, (char) 119, (char) 108, (char) 15, (char) 34, (char) 1, (char) 157, (char) 75, (char) 206, (char) 104, (char) 48, (char) 24, (char) 151, (char) 189, (char) 30, (char) 254, (char) 34, (char) 145, (char) 72, (char) 67, (char) 203, (char) 5, (char) 63, (char) 61, (char) 153, (char) 218, (char) 223, (char) 130, (char) 64, (char) 203, (char) 115, (char) 235, (char) 188, (char) 101, (char) 0, (char) 151, (char) 123, (char) 121, (char) 200, (char) 3, (char) 86, (char) 144, (char) 230, (char) 219, (char) 61, (char) 11, (char) 10, (char) 188, (char) 209, (char) 153, (char) 209, (char) 225, (char) 58, (char) 145, (char) 248, (char) 230, (char) 41, (char) 34, (char) 144, (char) 38, (char) 52, (char) 13, (char) 236, (char) 58, (char) 148, (char) 89, (char) 59, (char) 35, (char) 191, (char) 243, (char) 213, (char) 185, (char) 59, (char) 210, (char) 48, (char) 109, (char) 169, (char) 203, (char) 70, (char) 117, (char) 109, (char) 121, (char) 55, (char) 106, (char) 61, (char) 117, (char) 238, (char) 168, (char) 105, (char) 180, (char) 23, (char) 193, (char) 217, (char) 57, (char) 42, (char) 107, (char) 39, (char) 168, (char) 46, (char) 1, (char) 85, (char) 18, (char) 13, (char) 190, (char) 166, (char) 78, (char) 247, (char) 200, (char) 171, (char) 217, (char) 115, (char) 134, (char) 226, (char) 76, (char) 201, (char) 151, (char) 91, (char) 239, (char) 135, (char) 132, (char) 8, (char) 118, (char) 82, (char) 167, (char) 203, (char) 88, (char) 115, (char) 253, (char) 179, (char) 52, (char) 143, (char) 205, (char) 55, (char) 79, (char) 113, (char) 205, (char) 5, (char) 180, (char) 13, (char) 192, (char) 58, (char) 138, (char) 181, (char) 111, (char) 61, (char) 28, (char) 200, (char) 79, (char) 218, (char) 147, (char) 0, (char) 53, (char) 23, (char) 236, (char) 173, (char) 10, (char) 6, (char) 193, (char) 222, (char) 241, (char) 86, (char) 254, (char) 20, (char) 37, (char) 175, (char) 16, (char) 249, (char) 144, (char) 207, (char) 155, (char) 248, (char) 9, (char) 234, (char) 77, (char) 39, (char) 152, (char) 65, (char) 72, (char) 118, (char) 180, (char) 55, (char) 133, (char) 171, (char) 4, (char) 210, (char) 54, (char) 251, (char) 36, (char) 90, (char) 233, (char) 17, (char) 86, (char) 255, (char) 165, (char) 229, (char) 130, (char) 135, (char) 206, (char) 63, (char) 100, (char) 88, (char) 254, (char) 222, (char) 51, (char) 40, (char) 161, (char) 196, (char) 43, (char) 129, (char) 125, (char) 234, (char) 51, (char) 244, (char) 208, (char) 25}, 0);
		p267.sequence_SET((char) 47535);
		p267.first_message_offset_SET((char) 156);
		p267.length_SET((char) 16);
		p267.target_system_SET((char) 109);
		p267.target_component_SET((char) 238);
		CommunicationChannel.instance.send(p267);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
		TestChannel.instance.on_LOGGING_ACK.add((src, ph, pack) ->
		{
			assert (pack.target_system_GET() == (char) 252);
			assert (pack.sequence_GET() == (char) 12379);
			assert (pack.target_component_GET() == (char) 205);
		});
		GroundControl.LOGGING_ACK p268 = CommunicationChannel.new_LOGGING_ACK();
		PH.setPack(p268);
		p268.sequence_SET((char) 12379);
		p268.target_system_SET((char) 252);
		p268.target_component_SET((char) 205);
		CommunicationChannel.instance.send(p268);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
		TestChannel.instance.on_VIDEO_STREAM_INFORMATION.add((src, ph, pack) ->
		{
			assert (pack.uri_LEN(ph) == 64);
			assert (pack.uri_TRY(ph).equals("tuevJqzkGrhylldxlMktidgbkePHnwksnpaigyirdkuxokcfzcljopjintdbpxlf"));
			assert (pack.status_GET() == (char) 69);
			assert (pack.framerate_GET() == 2.0039019E38F);
			assert (pack.camera_id_GET() == (char) 212);
			assert (pack.resolution_h_GET() == (char) 44190);
			assert (pack.rotation_GET() == (char) 31336);
			assert (pack.bitrate_GET() == 4235887167L);
			assert (pack.resolution_v_GET() == (char) 4025);
		});
		GroundControl.VIDEO_STREAM_INFORMATION p269 = CommunicationChannel.new_VIDEO_STREAM_INFORMATION();
		PH.setPack(p269);
		p269.bitrate_SET(4235887167L);
		p269.status_SET((char) 69);
		p269.rotation_SET((char) 31336);
		p269.resolution_v_SET((char) 4025);
		p269.camera_id_SET((char) 212);
		p269.resolution_h_SET((char) 44190);
		p269.framerate_SET(2.0039019E38F);
		p269.uri_SET("tuevJqzkGrhylldxlMktidgbkePHnwksnpaigyirdkuxokcfzcljopjintdbpxlf", PH);
		CommunicationChannel.instance.send(p269);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
		TestChannel.instance.on_SET_VIDEO_STREAM_SETTINGS.add((src, ph, pack) ->
		{
			assert (pack.resolution_h_GET() == (char) 719);
			assert (pack.bitrate_GET() == 2170308369L);
			assert (pack.target_system_GET() == (char) 195);
			assert (pack.camera_id_GET() == (char) 48);
			assert (pack.uri_LEN(ph) == 119);
			assert (pack.uri_TRY(ph).equals("zmldyhKimOZbeksudyunqtqdjuzogrrrjdghefihrpfzcsinxcdrbuzhkfemmsqqwQrvkbrtlqBXlmOpbeotrkxnJcxnqhxldkqyiundcrwegskzddyNsxM"));
			assert (pack.target_component_GET() == (char) 248);
			assert (pack.framerate_GET() == -1.3766112E38F);
			assert (pack.resolution_v_GET() == (char) 54274);
			assert (pack.rotation_GET() == (char) 15320);
		});
		GroundControl.SET_VIDEO_STREAM_SETTINGS p270 = CommunicationChannel.new_SET_VIDEO_STREAM_SETTINGS();
		PH.setPack(p270);
		p270.framerate_SET(-1.3766112E38F);
		p270.rotation_SET((char) 15320);
		p270.uri_SET("zmldyhKimOZbeksudyunqtqdjuzogrrrjdghefihrpfzcsinxcdrbuzhkfemmsqqwQrvkbrtlqBXlmOpbeotrkxnJcxnqhxldkqyiundcrwegskzddyNsxM", PH);
		p270.camera_id_SET((char) 48);
		p270.resolution_h_SET((char) 719);
		p270.bitrate_SET(2170308369L);
		p270.target_system_SET((char) 195);
		p270.target_component_SET((char) 248);
		p270.resolution_v_SET((char) 54274);
		CommunicationChannel.instance.send(p270);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
		TestChannel.instance.on_WIFI_CONFIG_AP.add((src, ph, pack) ->
		{
			assert (pack.password_LEN(ph) == 8);
			assert (pack.password_TRY(ph).equals("lzuecphw"));
			assert (pack.ssid_LEN(ph) == 6);
			assert (pack.ssid_TRY(ph).equals("wxpenx"));
		});
		GroundControl.WIFI_CONFIG_AP p299 = CommunicationChannel.new_WIFI_CONFIG_AP();
		PH.setPack(p299);
		p299.password_SET("lzuecphw", PH);
		p299.ssid_SET("wxpenx", PH);
		CommunicationChannel.instance.send(p299);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
		TestChannel.instance.on_PROTOCOL_VERSION.add((src, ph, pack) ->
		{
			assert (pack.min_version_GET() == (char) 11081);
			assert (Arrays.equals(pack.spec_version_hash_GET(), new char[]{(char) 174, (char) 173, (char) 235, (char) 97, (char) 188, (char) 182, (char) 99, (char) 102}));
			assert (pack.max_version_GET() == (char) 61535);
			assert (Arrays.equals(pack.library_version_hash_GET(), new char[]{(char) 207, (char) 68, (char) 172, (char) 59, (char) 18, (char) 57, (char) 48, (char) 170}));
			assert (pack.version_GET() == (char) 56465);
		});
		GroundControl.PROTOCOL_VERSION p300 = CommunicationChannel.new_PROTOCOL_VERSION();
		PH.setPack(p300);
		p300.min_version_SET((char) 11081);
		p300.version_SET((char) 56465);
		p300.library_version_hash_SET(new char[]{(char) 207, (char) 68, (char) 172, (char) 59, (char) 18, (char) 57, (char) 48, (char) 170}, 0);
		p300.max_version_SET((char) 61535);
		p300.spec_version_hash_SET(new char[]{(char) 174, (char) 173, (char) 235, (char) 97, (char) 188, (char) 182, (char) 99, (char) 102}, 0);
		CommunicationChannel.instance.send(p300);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
		TestChannel.instance.on_UAVCAN_NODE_STATUS.add((src, ph, pack) ->
		{
			assert (pack.vendor_specific_status_code_GET() == (char) 62154);
			assert (pack.time_usec_GET() == 2367004820116089153L);
			assert (pack.sub_mode_GET() == (char) 0);
			assert (pack.uptime_sec_GET() == 3166946102L);
			assert (pack.mode_GET() == UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_MAINTENANCE);
			assert (pack.health_GET() == UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_ERROR);
		});
		GroundControl.UAVCAN_NODE_STATUS p310 = CommunicationChannel.new_UAVCAN_NODE_STATUS();
		PH.setPack(p310);
		p310.mode_SET(UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_MAINTENANCE);
		p310.health_SET(UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_ERROR);
		p310.sub_mode_SET((char) 0);
		p310.vendor_specific_status_code_SET((char) 62154);
		p310.uptime_sec_SET(3166946102L);
		p310.time_usec_SET(2367004820116089153L);
		CommunicationChannel.instance.send(p310);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
		TestChannel.instance.on_UAVCAN_NODE_INFO.add((src, ph, pack) ->
		{
			assert (pack.sw_vcs_commit_GET() == 3252199045L);
			assert (pack.name_LEN(ph) == 65);
			assert (pack.name_TRY(ph).equals("uzplnbqwdmtuOomVvpSeqsPpNjafqaKovnpwslocurZfsypamplryDmecqndtAbqz"));
			assert (pack.time_usec_GET() == 7579166809202293913L);
			assert (pack.hw_version_major_GET() == (char) 192);
			assert (Arrays.equals(pack.hw_unique_id_GET(), new char[]{(char) 210, (char) 47, (char) 150, (char) 232, (char) 107, (char) 70, (char) 148, (char) 247, (char) 67, (char) 120, (char) 44, (char) 194, (char) 14, (char) 84, (char) 88, (char) 46}));
			assert (pack.sw_version_minor_GET() == (char) 125);
			assert (pack.sw_version_major_GET() == (char) 228);
			assert (pack.uptime_sec_GET() == 2590673543L);
			assert (pack.hw_version_minor_GET() == (char) 141);
		});
		GroundControl.UAVCAN_NODE_INFO p311 = CommunicationChannel.new_UAVCAN_NODE_INFO();
		PH.setPack(p311);
		p311.sw_version_minor_SET((char) 125);
		p311.sw_version_major_SET((char) 228);
		p311.name_SET("uzplnbqwdmtuOomVvpSeqsPpNjafqaKovnpwslocurZfsypamplryDmecqndtAbqz", PH);
		p311.hw_version_major_SET((char) 192);
		p311.uptime_sec_SET(2590673543L);
		p311.sw_vcs_commit_SET(3252199045L);
		p311.hw_unique_id_SET(new char[]{(char) 210, (char) 47, (char) 150, (char) 232, (char) 107, (char) 70, (char) 148, (char) 247, (char) 67, (char) 120, (char) 44, (char) 194, (char) 14, (char) 84, (char) 88, (char) 46}, 0);
		p311.hw_version_minor_SET((char) 141);
		p311.time_usec_SET(7579166809202293913L);
		CommunicationChannel.instance.send(p311);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
		TestChannel.instance.on_PARAM_EXT_REQUEST_READ.add((src, ph, pack) ->
		{
			assert (pack.param_index_GET() == (short) -13664);
			assert (pack.param_id_LEN(ph) == 6);
			assert (pack.param_id_TRY(ph).equals("knaait"));
			assert (pack.target_system_GET() == (char) 72);
			assert (pack.target_component_GET() == (char) 155);
		});
		GroundControl.PARAM_EXT_REQUEST_READ p320 = CommunicationChannel.new_PARAM_EXT_REQUEST_READ();
		PH.setPack(p320);
		p320.target_system_SET((char) 72);
		p320.target_component_SET((char) 155);
		p320.param_id_SET("knaait", PH);
		p320.param_index_SET((short) -13664);
		CommunicationChannel.instance.send(p320);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
		TestChannel.instance.on_PARAM_EXT_REQUEST_LIST.add((src, ph, pack) ->
		{
			assert (pack.target_component_GET() == (char) 127);
			assert (pack.target_system_GET() == (char) 146);
		});
		GroundControl.PARAM_EXT_REQUEST_LIST p321 = CommunicationChannel.new_PARAM_EXT_REQUEST_LIST();
		PH.setPack(p321);
		p321.target_component_SET((char) 127);
		p321.target_system_SET((char) 146);
		CommunicationChannel.instance.send(p321);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
		TestChannel.instance.on_PARAM_EXT_VALUE.add((src, ph, pack) ->
		{
			assert (pack.param_index_GET() == (char) 49519);
			assert (pack.param_count_GET() == (char) 52759);
			assert (pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_REAL64);
			assert (pack.param_id_LEN(ph) == 7);
			assert (pack.param_id_TRY(ph).equals("cfrrzdD"));
			assert (pack.param_value_LEN(ph) == 14);
			assert (pack.param_value_TRY(ph).equals("sxekFbgeernwgm"));
		});
		GroundControl.PARAM_EXT_VALUE p322 = CommunicationChannel.new_PARAM_EXT_VALUE();
		PH.setPack(p322);
		p322.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_REAL64);
		p322.param_value_SET("sxekFbgeernwgm", PH);
		p322.param_id_SET("cfrrzdD", PH);
		p322.param_index_SET((char) 49519);
		p322.param_count_SET((char) 52759);
		CommunicationChannel.instance.send(p322);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
		TestChannel.instance.on_PARAM_EXT_SET.add((src, ph, pack) ->
		{
			assert (pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT8);
			assert (pack.target_component_GET() == (char) 122);
			assert (pack.param_value_LEN(ph) == 43);
			assert (pack.param_value_TRY(ph).equals("owkpLSxouapmhjnueIezojQkeopjWyKxiMzAkFshvWp"));
			assert (pack.param_id_LEN(ph) == 13);
			assert (pack.param_id_TRY(ph).equals("mKyZYprdeomrp"));
			assert (pack.target_system_GET() == (char) 9);
		});
		GroundControl.PARAM_EXT_SET p323 = CommunicationChannel.new_PARAM_EXT_SET();
		PH.setPack(p323);
		p323.param_id_SET("mKyZYprdeomrp", PH);
		p323.target_system_SET((char) 9);
		p323.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT8);
		p323.param_value_SET("owkpLSxouapmhjnueIezojQkeopjWyKxiMzAkFshvWp", PH);
		p323.target_component_SET((char) 122);
		CommunicationChannel.instance.send(p323);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
		TestChannel.instance.on_PARAM_EXT_ACK.add((src, ph, pack) ->
		{
			assert (pack.param_id_LEN(ph) == 16);
			assert (pack.param_id_TRY(ph).equals("vpmipkgRqhybsbwq"));
			assert (pack.param_value_LEN(ph) == 34);
			assert (pack.param_value_TRY(ph).equals("gtWgliwuctyjwfjaywyLpyvhvannraffxg"));
			assert (pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT8);
			assert (pack.param_result_GET() == PARAM_ACK.PARAM_ACK_IN_PROGRESS);
		});
		GroundControl.PARAM_EXT_ACK p324 = CommunicationChannel.new_PARAM_EXT_ACK();
		PH.setPack(p324);
		p324.param_value_SET("gtWgliwuctyjwfjaywyLpyvhvannraffxg", PH);
		p324.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT8);
		p324.param_id_SET("vpmipkgRqhybsbwq", PH);
		p324.param_result_SET(PARAM_ACK.PARAM_ACK_IN_PROGRESS);
		CommunicationChannel.instance.send(p324);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
		TestChannel.instance.on_OBSTACLE_DISTANCE.add((src, ph, pack) ->
		{
			assert (pack.sensor_type_GET() == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_UNKNOWN);
			assert (Arrays.equals(pack.distances_GET(), new char[]{(char) 5943, (char) 6131, (char) 56026, (char) 38521, (char) 43911, (char) 20464, (char) 9162, (char) 22009, (char) 58572, (char) 41999, (char) 57532, (char) 58657, (char) 1184, (char) 43890, (char) 41120, (char) 42777, (char) 232, (char) 57258, (char) 6701, (char) 31155, (char) 45835, (char) 26832, (char) 20740, (char) 12281, (char) 52041, (char) 39208, (char) 30867, (char) 38442, (char) 53551, (char) 51963, (char) 25321, (char) 55763, (char) 50845, (char) 45397, (char) 12103, (char) 3623, (char) 28441, (char) 19491, (char) 16608, (char) 37453, (char) 16142, (char) 15254, (char) 44839, (char) 22550, (char) 21785, (char) 1425, (char) 26508, (char) 27386, (char) 32309, (char) 33264, (char) 52038, (char) 17675, (char) 37869, (char) 18153, (char) 22869, (char) 62629, (char) 61009, (char) 32708, (char) 18768, (char) 62425, (char) 54514, (char) 50779, (char) 38550, (char) 20552, (char) 6067, (char) 42405, (char) 19570, (char) 23542, (char) 16544, (char) 59777, (char) 58269, (char) 48845}));
			assert (pack.increment_GET() == (char) 222);
			assert (pack.min_distance_GET() == (char) 44077);
			assert (pack.time_usec_GET() == 556068523655154054L);
			assert (pack.max_distance_GET() == (char) 2404);
		});
		GroundControl.OBSTACLE_DISTANCE p330 = CommunicationChannel.new_OBSTACLE_DISTANCE();
		PH.setPack(p330);
		p330.distances_SET(new char[]{(char) 5943, (char) 6131, (char) 56026, (char) 38521, (char) 43911, (char) 20464, (char) 9162, (char) 22009, (char) 58572, (char) 41999, (char) 57532, (char) 58657, (char) 1184, (char) 43890, (char) 41120, (char) 42777, (char) 232, (char) 57258, (char) 6701, (char) 31155, (char) 45835, (char) 26832, (char) 20740, (char) 12281, (char) 52041, (char) 39208, (char) 30867, (char) 38442, (char) 53551, (char) 51963, (char) 25321, (char) 55763, (char) 50845, (char) 45397, (char) 12103, (char) 3623, (char) 28441, (char) 19491, (char) 16608, (char) 37453, (char) 16142, (char) 15254, (char) 44839, (char) 22550, (char) 21785, (char) 1425, (char) 26508, (char) 27386, (char) 32309, (char) 33264, (char) 52038, (char) 17675, (char) 37869, (char) 18153, (char) 22869, (char) 62629, (char) 61009, (char) 32708, (char) 18768, (char) 62425, (char) 54514, (char) 50779, (char) 38550, (char) 20552, (char) 6067, (char) 42405, (char) 19570, (char) 23542, (char) 16544, (char) 59777, (char) 58269, (char) 48845}, 0);
		p330.time_usec_SET(556068523655154054L);
		p330.min_distance_SET((char) 44077);
		p330.sensor_type_SET(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_UNKNOWN);
		p330.max_distance_SET((char) 2404);
		p330.increment_SET((char) 222);
		CommunicationChannel.instance.send(p330);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
	}

}