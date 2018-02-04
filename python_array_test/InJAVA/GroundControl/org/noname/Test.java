
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
				default:
					assert (false);//("Unknown enum" + id);
			}
			set_bits(id, 7, data, 260);
		}
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

	public static class ARRAY_TEST_0 extends GroundControl.ARRAY_TEST_0 {
		public char[] ar_u16_GET(char[] dst_ch, int pos)  //Value array
		{
			for (int BYTE = 0, dst_max = pos + 4; pos < dst_max; pos++, BYTE += 2)
				dst_ch[pos] = (char) ((char) get_bytes(data, BYTE, 2));
			return dst_ch;
		}

		public char[] ar_u16_GET()//Value array
		{return ar_u16_GET(new char[4], 0);}

		public long[] ar_u32_GET(long[] dst_ch, int pos)  //Value array
		{
			for (int BYTE = 8, dst_max = pos + 4; pos < dst_max; pos++, BYTE += 4)
				dst_ch[pos] = (get_bytes(data, BYTE, 4));
			return dst_ch;
		}

		public long[] ar_u32_GET()//Value array
		{return ar_u32_GET(new long[4], 0);}

		public char v1_GET()//Stub field
		{ return (char) ((char) get_bytes(data, 24, 1)); }

		public byte[] ar_i8_GET(byte[] dst_ch, int pos)  //Value array
		{
			for (int BYTE = 25, dst_max = pos + 4; pos < dst_max; pos++, BYTE += 1)
				dst_ch[pos] = (byte) ((byte) get_bytes(data, BYTE, 1));
			return dst_ch;
		}

		public byte[] ar_i8_GET()//Value array
		{return ar_i8_GET(new byte[4], 0);}

		public char[] ar_u8_GET(char[] dst_ch, int pos)  //Value array
		{
			for (int BYTE = 29, dst_max = pos + 4; pos < dst_max; pos++, BYTE += 1)
				dst_ch[pos] = (char) ((char) get_bytes(data, BYTE, 1));
			return dst_ch;
		}

		public char[] ar_u8_GET()//Value array
		{return ar_u8_GET(new char[4], 0);}
	}

	public static class ARRAY_TEST_1 extends GroundControl.ARRAY_TEST_1 {
		public long[] ar_u32_GET(long[] dst_ch, int pos)  //Value array
		{
			for (int BYTE = 0, dst_max = pos + 4; pos < dst_max; pos++, BYTE += 4)
				dst_ch[pos] = (get_bytes(data, BYTE, 4));
			return dst_ch;
		}

		public long[] ar_u32_GET()//Value array
		{return ar_u32_GET(new long[4], 0);}
	}

	public static class ARRAY_TEST_3 extends GroundControl.ARRAY_TEST_3 {
		public long[] ar_u32_GET(long[] dst_ch, int pos)  //Value array
		{
			for (int BYTE = 0, dst_max = pos + 4; pos < dst_max; pos++, BYTE += 4)
				dst_ch[pos] = (get_bytes(data, BYTE, 4));
			return dst_ch;
		}

		public long[] ar_u32_GET()//Value array
		{return ar_u32_GET(new long[4], 0);}

		public char v_GET()//Stub field
		{ return (char) ((char) get_bytes(data, 16, 1)); }
	}

	public static class ARRAY_TEST_4 extends GroundControl.ARRAY_TEST_4 {
		public long[] ar_u32_GET(long[] dst_ch, int pos)  //Value array
		{
			for (int BYTE = 0, dst_max = pos + 4; pos < dst_max; pos++, BYTE += 4)
				dst_ch[pos] = (get_bytes(data, BYTE, 4));
			return dst_ch;
		}

		public long[] ar_u32_GET()//Value array
		{return ar_u32_GET(new long[4], 0);}

		public char v_GET()//Stub field
		{ return (char) ((char) get_bytes(data, 16, 1)); }
	}

	public static class ARRAY_TEST_5 extends GroundControl.ARRAY_TEST_5 {
		public String c1_TRY(Bounds.Inside ph)//Value array
		{
			if (ph.field_bit != 0 && !try_visit_field(ph, 0) || !try_visit_item(ph, 0)) return null;
			return new String(c1_GET(ph, new char[ph.items], 0));
		}

		public char[] c1_GET(Bounds.Inside ph, char[] dst_ch, int pos) //Value array
		{
			for (int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max; pos++, BYTE += 2)
				dst_ch[pos] = (char) ((char) get_bytes(data, BYTE, 2));
			return dst_ch;
		}

		public int c1_LEN(Bounds.Inside ph) {
			return (ph.field_bit != 0 && !try_visit_field(ph, 0) || !try_visit_item(ph, 0)) ? 0 : ph.items;
		}

		public String c2_TRY(Bounds.Inside ph)//Value array
		{
			if (ph.field_bit != 1 && !try_visit_field(ph, 1) || !try_visit_item(ph, 0)) return null;
			return new String(c2_GET(ph, new char[ph.items], 0));
		}

		public char[] c2_GET(Bounds.Inside ph, char[] dst_ch, int pos) //Value array
		{
			for (int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max; pos++, BYTE += 2)
				dst_ch[pos] = (char) ((char) get_bytes(data, BYTE, 2));
			return dst_ch;
		}

		public int c2_LEN(Bounds.Inside ph) {
			return (ph.field_bit != 1 && !try_visit_field(ph, 1) || !try_visit_item(ph, 0)) ? 0 : ph.items;
		}
	}

	public static class ARRAY_TEST_6 extends GroundControl.ARRAY_TEST_6 {
		public char v2_GET()//Stub field
		{ return (char) ((char) get_bytes(data, 0, 2)); }

		public char[] ar_u16_GET(char[] dst_ch, int pos)  //Value array
		{
			for (int BYTE = 2, dst_max = pos + 2; pos < dst_max; pos++, BYTE += 2)
				dst_ch[pos] = (char) ((char) get_bytes(data, BYTE, 2));
			return dst_ch;
		}

		public char[] ar_u16_GET()//Value array
		{return ar_u16_GET(new char[2], 0);}

		public long v3_GET()//Stub field
		{ return (get_bytes(data, 6, 4)); }

		public long[] ar_u32_GET(long[] dst_ch, int pos)  //Value array
		{
			for (int BYTE = 10, dst_max = pos + 2; pos < dst_max; pos++, BYTE += 4)
				dst_ch[pos] = (get_bytes(data, BYTE, 4));
			return dst_ch;
		}

		public long[] ar_u32_GET()//Value array
		{return ar_u32_GET(new long[2], 0);}

		public char v1_GET()//Stub field
		{ return (char) ((char) get_bytes(data, 18, 1)); }

		public int[] ar_i32_GET(int[] dst_ch, int pos)  //Value array
		{
			for (int BYTE = 19, dst_max = pos + 2; pos < dst_max; pos++, BYTE += 4)
				dst_ch[pos] = (int) ((int) get_bytes(data, BYTE, 4));
			return dst_ch;
		}

		public int[] ar_i32_GET()//Value array
		{return ar_i32_GET(new int[2], 0);}

		public short[] ar_i16_GET(short[] dst_ch, int pos)  //Value array
		{
			for (int BYTE = 27, dst_max = pos + 2; pos < dst_max; pos++, BYTE += 2)
				dst_ch[pos] = (short) ((short) get_bytes(data, BYTE, 2));
			return dst_ch;
		}

		public short[] ar_i16_GET()//Value array
		{return ar_i16_GET(new short[2], 0);}

		public char[] ar_u8_GET(char[] dst_ch, int pos)  //Value array
		{
			for (int BYTE = 31, dst_max = pos + 2; pos < dst_max; pos++, BYTE += 1)
				dst_ch[pos] = (char) ((char) get_bytes(data, BYTE, 1));
			return dst_ch;
		}

		public char[] ar_u8_GET()//Value array
		{return ar_u8_GET(new char[2], 0);}

		public byte[] ar_i8_GET(byte[] dst_ch, int pos)  //Value array
		{
			for (int BYTE = 33, dst_max = pos + 2; pos < dst_max; pos++, BYTE += 1)
				dst_ch[pos] = (byte) ((byte) get_bytes(data, BYTE, 1));
			return dst_ch;
		}

		public byte[] ar_i8_GET()//Value array
		{return ar_i8_GET(new byte[2], 0);}

		public double[] ar_d_GET(double[] dst_ch, int pos)  //Value array
		{
			for (int BYTE = 35, dst_max = pos + 2; pos < dst_max; pos++, BYTE += 8)
				dst_ch[pos] = (double) (Double.longBitsToDouble(get_bytes(data, BYTE, 8)));
			return dst_ch;
		}

		public double[] ar_d_GET()//Value array
		{return ar_d_GET(new double[2], 0);}

		public float[] ar_f_GET(float[] dst_ch, int pos)  //Value array
		{
			for (int BYTE = 51, dst_max = pos + 2; pos < dst_max; pos++, BYTE += 4)
				dst_ch[pos] = (float) (Float.intBitsToFloat((int) get_bytes(data, BYTE, 4)));
			return dst_ch;
		}

		public float[] ar_f_GET()//Value array
		{return ar_f_GET(new float[2], 0);}

		public String ar_c_TRY(Bounds.Inside ph) //Value array
		{
			if (ph.field_bit != 472 && !try_visit_field(ph, 472) || !try_visit_item(ph, 0)) return null;
			return new String(ar_c_GET(ph, new char[ph.items], 0));
		}

		public char[] ar_c_GET(Bounds.Inside ph, char[] dst_ch, int pos) //Value array
		{
			for (int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max; pos++, BYTE += 2)
				dst_ch[pos] = (char) ((char) get_bytes(data, BYTE, 2));
			return dst_ch;
		}

		public int ar_c_LEN(Bounds.Inside ph) {
			return (ph.field_bit != 472 && !try_visit_field(ph, 472) || !try_visit_item(ph, 0)) ? 0 : ph.items;
		}
	}

	public static class ARRAY_TEST_7 extends GroundControl.ARRAY_TEST_7 {
		public char[] ar_u16_GET(char[] dst_ch, int pos)  //Value array
		{
			for (int BYTE = 0, dst_max = pos + 2; pos < dst_max; pos++, BYTE += 2)
				dst_ch[pos] = (char) ((char) get_bytes(data, BYTE, 2));
			return dst_ch;
		}

		public char[] ar_u16_GET()//Value array
		{return ar_u16_GET(new char[2], 0);}

		public long[] ar_u32_GET(long[] dst_ch, int pos)  //Value array
		{
			for (int BYTE = 4, dst_max = pos + 2; pos < dst_max; pos++, BYTE += 4)
				dst_ch[pos] = (get_bytes(data, BYTE, 4));
			return dst_ch;
		}

		public long[] ar_u32_GET()//Value array
		{return ar_u32_GET(new long[2], 0);}

		public double[] ar_d_GET(double[] dst_ch, int pos)  //Value array
		{
			for (int BYTE = 12, dst_max = pos + 2; pos < dst_max; pos++, BYTE += 8)
				dst_ch[pos] = (double) (Double.longBitsToDouble(get_bytes(data, BYTE, 8)));
			return dst_ch;
		}

		public double[] ar_d_GET()//Value array
		{return ar_d_GET(new double[2], 0);}

		public float[] ar_f_GET(float[] dst_ch, int pos)  //Value array
		{
			for (int BYTE = 28, dst_max = pos + 2; pos < dst_max; pos++, BYTE += 4)
				dst_ch[pos] = (float) (Float.intBitsToFloat((int) get_bytes(data, BYTE, 4)));
			return dst_ch;
		}

		public float[] ar_f_GET()//Value array
		{return ar_f_GET(new float[2], 0);}

		public int[] ar_i32_GET(int[] dst_ch, int pos)  //Value array
		{
			for (int BYTE = 36, dst_max = pos + 2; pos < dst_max; pos++, BYTE += 4)
				dst_ch[pos] = (int) ((int) get_bytes(data, BYTE, 4));
			return dst_ch;
		}

		public int[] ar_i32_GET()//Value array
		{return ar_i32_GET(new int[2], 0);}

		public short[] ar_i16_GET(short[] dst_ch, int pos)  //Value array
		{
			for (int BYTE = 44, dst_max = pos + 2; pos < dst_max; pos++, BYTE += 2)
				dst_ch[pos] = (short) ((short) get_bytes(data, BYTE, 2));
			return dst_ch;
		}

		public short[] ar_i16_GET()//Value array
		{return ar_i16_GET(new short[2], 0);}

		public char[] ar_u8_GET(char[] dst_ch, int pos)  //Value array
		{
			for (int BYTE = 48, dst_max = pos + 2; pos < dst_max; pos++, BYTE += 1)
				dst_ch[pos] = (char) ((char) get_bytes(data, BYTE, 1));
			return dst_ch;
		}

		public char[] ar_u8_GET()//Value array
		{return ar_u8_GET(new char[2], 0);}

		public byte[] ar_i8_GET(byte[] dst_ch, int pos)  //Value array
		{
			for (int BYTE = 50, dst_max = pos + 2; pos < dst_max; pos++, BYTE += 1)
				dst_ch[pos] = (byte) ((byte) get_bytes(data, BYTE, 1));
			return dst_ch;
		}

		public byte[] ar_i8_GET()//Value array
		{return ar_i8_GET(new byte[2], 0);}

		public String ar_c_TRY(Bounds.Inside ph) //Value array
		{
			if (ph.field_bit != 416 && !try_visit_field(ph, 416) || !try_visit_item(ph, 0)) return null;
			return new String(ar_c_GET(ph, new char[ph.items], 0));
		}

		public char[] ar_c_GET(Bounds.Inside ph, char[] dst_ch, int pos) //Value array
		{
			for (int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max; pos++, BYTE += 2)
				dst_ch[pos] = (char) ((char) get_bytes(data, BYTE, 2));
			return dst_ch;
		}

		public int ar_c_LEN(Bounds.Inside ph) {
			return (ph.field_bit != 416 && !try_visit_field(ph, 416) || !try_visit_item(ph, 0)) ? 0 : ph.items;
		}
	}

	public static class ARRAY_TEST_8 extends GroundControl.ARRAY_TEST_8 {
		public char[] ar_u16_GET(char[] dst_ch, int pos)  //Value array
		{
			for (int BYTE = 0, dst_max = pos + 2; pos < dst_max; pos++, BYTE += 2)
				dst_ch[pos] = (char) ((char) get_bytes(data, BYTE, 2));
			return dst_ch;
		}

		public char[] ar_u16_GET()//Value array
		{return ar_u16_GET(new char[2], 0);}

		public long v3_GET()//Stub field
		{ return (get_bytes(data, 4, 4)); }

		public double[] ar_d_GET(double[] dst_ch, int pos)  //Value array
		{
			for (int BYTE = 8, dst_max = pos + 2; pos < dst_max; pos++, BYTE += 8)
				dst_ch[pos] = (double) (Double.longBitsToDouble(get_bytes(data, BYTE, 8)));
			return dst_ch;
		}

		public double[] ar_d_GET()//Value array
		{return ar_d_GET(new double[2], 0);}
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

		static final Collection<OnReceive.Handler<SCALED_PRESSURE3, Channel>>          on_SCALED_PRESSURE3          = new OnReceive<>();
		static final Collection<OnReceive.Handler<FOLLOW_TARGET, Channel>>             on_FOLLOW_TARGET             = new OnReceive<>();
		static final Collection<OnReceive.Handler<CONTROL_SYSTEM_STATE, Channel>>      on_CONTROL_SYSTEM_STATE      = new OnReceive<>();
		static final Collection<OnReceive.Handler<BATTERY_STATUS, Channel>>            on_BATTERY_STATUS            = new OnReceive<>();
		static final Collection<OnReceive.Handler<AUTOPILOT_VERSION, Channel>>         on_AUTOPILOT_VERSION         = new OnReceive<>();
		static final Collection<OnReceive.Handler<LANDING_TARGET, Channel>>            on_LANDING_TARGET            = new OnReceive<>();
		static final Collection<OnReceive.Handler<ARRAY_TEST_0, Channel>>              on_ARRAY_TEST_0              = new OnReceive<>();
		static final Collection<OnReceive.Handler<ARRAY_TEST_1, Channel>>              on_ARRAY_TEST_1              = new OnReceive<>();
		static final Collection<OnReceive.Handler<ARRAY_TEST_3, Channel>>              on_ARRAY_TEST_3              = new OnReceive<>();
		static final Collection<OnReceive.Handler<ARRAY_TEST_4, Channel>>              on_ARRAY_TEST_4              = new OnReceive<>();
		static final Collection<OnReceive.Handler<ARRAY_TEST_5, Channel>>              on_ARRAY_TEST_5              = new OnReceive<>();
		static final Collection<OnReceive.Handler<ARRAY_TEST_6, Channel>>              on_ARRAY_TEST_6              = new OnReceive<>();
		static final Collection<OnReceive.Handler<ARRAY_TEST_7, Channel>>              on_ARRAY_TEST_7              = new OnReceive<>();
		static final Collection<OnReceive.Handler<ARRAY_TEST_8, Channel>>              on_ARRAY_TEST_8              = new OnReceive<>();
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
				case 150:
					if (pack == null) return new ARRAY_TEST_0();
					((OnReceive) on_ARRAY_TEST_0).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
					break;
				case 151:
					if (pack == null) return new ARRAY_TEST_1();
					((OnReceive) on_ARRAY_TEST_1).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
					break;
				case 153:
					if (pack == null) return new ARRAY_TEST_3();
					((OnReceive) on_ARRAY_TEST_3).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
					break;
				case 154:
					if (pack == null) return new ARRAY_TEST_4();
					((OnReceive) on_ARRAY_TEST_4).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
					break;
				case 155:
					if (pack == null) return new ARRAY_TEST_5();
					((OnReceive) on_ARRAY_TEST_5).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
					break;
				case 156:
					if (pack == null) return new ARRAY_TEST_6();
					((OnReceive) on_ARRAY_TEST_6).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
					break;
				case 157:
					if (pack == null) return new ARRAY_TEST_7();
					((OnReceive) on_ARRAY_TEST_7).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
					break;
				case 158:
					if (pack == null) return new ARRAY_TEST_8();
					((OnReceive) on_ARRAY_TEST_8).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
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
			assert (pack.base_mode_GET() == MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED);
			assert (pack.mavlink_version_GET() == (char) 152);
			assert (pack.type_GET() == MAV_TYPE.MAV_TYPE_GCS);
			assert (pack.custom_mode_GET() == 3022495839L);
			assert (pack.system_status_GET() == MAV_STATE.MAV_STATE_STANDBY);
			assert (pack.autopilot_GET() == MAV_AUTOPILOT.MAV_AUTOPILOT_INVALID);
		});
		HEARTBEAT p0 = new HEARTBEAT();
		PH.setPack(p0);
		p0.autopilot_SET(MAV_AUTOPILOT.MAV_AUTOPILOT_INVALID);
		p0.mavlink_version_SET((char) 152);
		p0.type_SET(MAV_TYPE.MAV_TYPE_GCS);
		p0.system_status_SET(MAV_STATE.MAV_STATE_STANDBY);
		p0.custom_mode_SET(3022495839L);
		p0.base_mode_SET(MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED);
		TestChannel.instance.send(p0);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_SYS_STATUS.add((src, ph, pack) ->
		{
			assert (pack.onboard_control_sensors_enabled_GET() == MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_GPS);
			assert (pack.load_GET() == (char) 14766);
			assert (pack.errors_count4_GET() == (char) 32744);
			assert (pack.voltage_battery_GET() == (char) 5067);
			assert (pack.current_battery_GET() == (short) 21694);
			assert (pack.onboard_control_sensors_present_GET() == MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW);
			assert (pack.errors_count2_GET() == (char) 46660);
			assert (pack.errors_count1_GET() == (char) 27290);
			assert (pack.drop_rate_comm_GET() == (char) 45563);
			assert (pack.onboard_control_sensors_health_GET() == MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR);
			assert (pack.battery_remaining_GET() == (byte) 94);
			assert (pack.errors_count3_GET() == (char) 4120);
			assert (pack.errors_comm_GET() == (char) 10277);
		});
		SYS_STATUS p1 = new SYS_STATUS();
		PH.setPack(p1);
		p1.errors_count1_SET((char) 27290);
		p1.onboard_control_sensors_health_SET(MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR);
		p1.battery_remaining_SET((byte) 94);
		p1.errors_count3_SET((char) 4120);
		p1.errors_count4_SET((char) 32744);
		p1.drop_rate_comm_SET((char) 45563);
		p1.voltage_battery_SET((char) 5067);
		p1.load_SET((char) 14766);
		p1.current_battery_SET((short) 21694);
		p1.errors_comm_SET((char) 10277);
		p1.onboard_control_sensors_enabled_SET(MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_GPS);
		p1.onboard_control_sensors_present_SET(MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW);
		p1.errors_count2_SET((char) 46660);
		TestChannel.instance.send(p1);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_SYSTEM_TIME.add((src, ph, pack) ->
		{
			assert (pack.time_unix_usec_GET() == 6193939409712659811L);
			assert (pack.time_boot_ms_GET() == 3734935379L);
		});
		SYSTEM_TIME p2 = new SYSTEM_TIME();
		PH.setPack(p2);
		p2.time_unix_usec_SET(6193939409712659811L);
		p2.time_boot_ms_SET(3734935379L);
		TestChannel.instance.send(p2);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_POSITION_TARGET_LOCAL_NED.add((src, ph, pack) ->
		{
			assert (pack.vy_GET() == -8.938663E37F);
			assert (pack.afx_GET() == 2.0150247E38F);
			assert (pack.time_boot_ms_GET() == 232665920L);
			assert (pack.vx_GET() == -1.8806408E38F);
			assert (pack.z_GET() == 1.8742099E38F);
			assert (pack.yaw_GET() == 2.3482745E38F);
			assert (pack.x_GET() == -3.0152738E37F);
			assert (pack.vz_GET() == 1.1516894E38F);
			assert (pack.y_GET() == 2.9005107E38F);
			assert (pack.yaw_rate_GET() == -4.849443E37F);
			assert (pack.type_mask_GET() == (char) 10232);
			assert (pack.afz_GET() == -4.5809004E37F);
			assert (pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT);
			assert (pack.afy_GET() == 9.134499E37F);
		});
		GroundControl.POSITION_TARGET_LOCAL_NED p3 = CommunicationChannel.new_POSITION_TARGET_LOCAL_NED();
		PH.setPack(p3);
		p3.type_mask_SET((char) 10232);
		p3.yaw_SET(2.3482745E38F);
		p3.z_SET(1.8742099E38F);
		p3.afz_SET(-4.5809004E37F);
		p3.x_SET(-3.0152738E37F);
		p3.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT);
		p3.vx_SET(-1.8806408E38F);
		p3.afy_SET(9.134499E37F);
		p3.yaw_rate_SET(-4.849443E37F);
		p3.vz_SET(1.1516894E38F);
		p3.afx_SET(2.0150247E38F);
		p3.vy_SET(-8.938663E37F);
		p3.y_SET(2.9005107E38F);
		p3.time_boot_ms_SET(232665920L);
		CommunicationChannel.instance.send(p3);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_PING.add((src, ph, pack) ->
		{
			assert (pack.seq_GET() == 303682085L);
			assert (pack.target_component_GET() == (char) 67);
			assert (pack.time_usec_GET() == 8072091355487508329L);
			assert (pack.target_system_GET() == (char) 123);
		});
		PING p4 = new PING();
		PH.setPack(p4);
		p4.seq_SET(303682085L);
		p4.target_component_SET((char) 67);
		p4.time_usec_SET(8072091355487508329L);
		p4.target_system_SET((char) 123);
		TestChannel.instance.send(p4);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_CHANGE_OPERATOR_CONTROL.add((src, ph, pack) ->
		{
			assert (pack.control_request_GET() == (char) 38);
			assert (pack.version_GET() == (char) 57);
			assert (pack.passkey_LEN(ph) == 18);
			assert (pack.passkey_TRY(ph).equals("uqekyiplsorqvztybs"));
			assert (pack.target_system_GET() == (char) 113);
		});
		CHANGE_OPERATOR_CONTROL p5 = new CHANGE_OPERATOR_CONTROL();
		PH.setPack(p5);
		p5.version_SET((char) 57);
		p5.passkey_SET("uqekyiplsorqvztybs", PH);
		p5.control_request_SET((char) 38);
		p5.target_system_SET((char) 113);
		TestChannel.instance.send(p5);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_CHANGE_OPERATOR_CONTROL_ACK.add((src, ph, pack) ->
		{
			assert (pack.gcs_system_id_GET() == (char) 224);
			assert (pack.ack_GET() == (char) 29);
			assert (pack.control_request_GET() == (char) 193);
		});
		CHANGE_OPERATOR_CONTROL_ACK p6 = new CHANGE_OPERATOR_CONTROL_ACK();
		PH.setPack(p6);
		p6.control_request_SET((char) 193);
		p6.ack_SET((char) 29);
		p6.gcs_system_id_SET((char) 224);
		TestChannel.instance.send(p6);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_AUTH_KEY.add((src, ph, pack) ->
		{
			assert (pack.key_LEN(ph) == 21);
			assert (pack.key_TRY(ph).equals("ubwztoaibzrdzqgvaqyww"));
		});
		AUTH_KEY p7 = new AUTH_KEY();
		PH.setPack(p7);
		p7.key_SET("ubwztoaibzrdzqgvaqyww", PH);
		TestChannel.instance.send(p7);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_SET_MODE.add((src, ph, pack) ->
		{
			assert (pack.base_mode_GET() == MAV_MODE.MAV_MODE_AUTO_ARMED);
			assert (pack.target_system_GET() == (char) 45);
			assert (pack.custom_mode_GET() == 1958814732L);
		});
		SET_MODE p11 = new SET_MODE();
		PH.setPack(p11);
		p11.target_system_SET((char) 45);
		p11.base_mode_SET(MAV_MODE.MAV_MODE_AUTO_ARMED);
		p11.custom_mode_SET(1958814732L);
		TestChannel.instance.send(p11);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_PARAM_REQUEST_READ.add((src, ph, pack) ->
		{
			assert (pack.target_component_GET() == (char) 45);
			assert (pack.param_id_LEN(ph) == 10);
			assert (pack.param_id_TRY(ph).equals("NdbCpztpvl"));
			assert (pack.target_system_GET() == (char) 87);
			assert (pack.param_index_GET() == (short) -4525);
		});
		PARAM_REQUEST_READ p20 = new PARAM_REQUEST_READ();
		PH.setPack(p20);
		p20.target_component_SET((char) 45);
		p20.target_system_SET((char) 87);
		p20.param_index_SET((short) -4525);
		p20.param_id_SET("NdbCpztpvl", PH);
		TestChannel.instance.send(p20);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_PARAM_REQUEST_LIST.add((src, ph, pack) ->
		{
			assert (pack.target_component_GET() == (char) 181);
			assert (pack.target_system_GET() == (char) 140);
		});
		PARAM_REQUEST_LIST p21 = new PARAM_REQUEST_LIST();
		PH.setPack(p21);
		p21.target_system_SET((char) 140);
		p21.target_component_SET((char) 181);
		TestChannel.instance.send(p21);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_PARAM_VALUE.add((src, ph, pack) ->
		{
			assert (pack.param_type_GET() == MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT8);
			assert (pack.param_count_GET() == (char) 1046);
			assert (pack.param_value_GET() == 1.743166E38F);
			assert (pack.param_id_LEN(ph) == 11);
			assert (pack.param_id_TRY(ph).equals("Ycxqagsvddn"));
			assert (pack.param_index_GET() == (char) 53830);
		});
		PARAM_VALUE p22 = new PARAM_VALUE();
		PH.setPack(p22);
		p22.param_count_SET((char) 1046);
		p22.param_index_SET((char) 53830);
		p22.param_id_SET("Ycxqagsvddn", PH);
		p22.param_type_SET(MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT8);
		p22.param_value_SET(1.743166E38F);
		TestChannel.instance.send(p22);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_PARAM_SET.add((src, ph, pack) ->
		{
			assert (pack.param_value_GET() == 1.7620719E38F);
			assert (pack.target_system_GET() == (char) 137);
			assert (pack.param_type_GET() == MAV_PARAM_TYPE.MAV_PARAM_TYPE_REAL64);
			assert (pack.target_component_GET() == (char) 42);
			assert (pack.param_id_LEN(ph) == 2);
			assert (pack.param_id_TRY(ph).equals("ns"));
		});
		PARAM_SET p23 = new PARAM_SET();
		PH.setPack(p23);
		p23.target_system_SET((char) 137);
		p23.param_value_SET(1.7620719E38F);
		p23.param_id_SET("ns", PH);
		p23.param_type_SET(MAV_PARAM_TYPE.MAV_PARAM_TYPE_REAL64);
		p23.target_component_SET((char) 42);
		TestChannel.instance.send(p23);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_GPS_RAW_INT.add((src, ph, pack) ->
		{
			assert (pack.lon_GET() == -1596860013);
			assert (pack.alt_ellipsoid_TRY(ph) == 760741244);
			assert (pack.hdg_acc_TRY(ph) == 2161113277L);
			assert (pack.alt_GET() == -2038920016);
			assert (pack.v_acc_TRY(ph) == 3668304672L);
			assert (pack.h_acc_TRY(ph) == 4222606935L);
			assert (pack.cog_GET() == (char) 5132);
			assert (pack.time_usec_GET() == 9092033343066871098L);
			assert (pack.fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_2D_FIX);
			assert (pack.eph_GET() == (char) 24442);
			assert (pack.lat_GET() == -749992918);
			assert (pack.satellites_visible_GET() == (char) 129);
			assert (pack.epv_GET() == (char) 4903);
			assert (pack.vel_GET() == (char) 39138);
			assert (pack.vel_acc_TRY(ph) == 49479289L);
		});
		GPS_RAW_INT p24 = new GPS_RAW_INT();
		PH.setPack(p24);
		p24.h_acc_SET(4222606935L, PH);
		p24.time_usec_SET(9092033343066871098L);
		p24.lat_SET(-749992918);
		p24.cog_SET((char) 5132);
		p24.vel_acc_SET(49479289L, PH);
		p24.eph_SET((char) 24442);
		p24.alt_SET(-2038920016);
		p24.satellites_visible_SET((char) 129);
		p24.v_acc_SET(3668304672L, PH);
		p24.fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_2D_FIX);
		p24.alt_ellipsoid_SET(760741244, PH);
		p24.epv_SET((char) 4903);
		p24.hdg_acc_SET(2161113277L, PH);
		p24.lon_SET(-1596860013);
		p24.vel_SET((char) 39138);
		TestChannel.instance.send(p24);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_GPS_STATUS.add((src, ph, pack) ->
		{
			assert (Arrays.equals(pack.satellite_prn_GET(), new char[]{(char) 38, (char) 172, (char) 45, (char) 36, (char) 111, (char) 75, (char) 103, (char) 171, (char) 97, (char) 140, (char) 194, (char) 68, (char) 2, (char) 51, (char) 68, (char) 79, (char) 21, (char) 165, (char) 149, (char) 173}));
			assert (Arrays.equals(pack.satellite_elevation_GET(), new char[]{(char) 84, (char) 84, (char) 243, (char) 71, (char) 142, (char) 125, (char) 122, (char) 79, (char) 2, (char) 45, (char) 60, (char) 31, (char) 9, (char) 14, (char) 161, (char) 108, (char) 1, (char) 123, (char) 252, (char) 1}));
			assert (Arrays.equals(pack.satellite_snr_GET(), new char[]{(char) 184, (char) 225, (char) 76, (char) 204, (char) 108, (char) 106, (char) 11, (char) 198, (char) 50, (char) 19, (char) 174, (char) 4, (char) 113, (char) 24, (char) 123, (char) 228, (char) 175, (char) 147, (char) 188, (char) 67}));
			assert (Arrays.equals(pack.satellite_azimuth_GET(), new char[]{(char) 6, (char) 8, (char) 163, (char) 110, (char) 145, (char) 229, (char) 45, (char) 137, (char) 182, (char) 243, (char) 217, (char) 105, (char) 99, (char) 194, (char) 240, (char) 83, (char) 228, (char) 31, (char) 105, (char) 29}));
			assert (Arrays.equals(pack.satellite_used_GET(), new char[]{(char) 115, (char) 107, (char) 16, (char) 168, (char) 93, (char) 225, (char) 47, (char) 167, (char) 69, (char) 6, (char) 3, (char) 83, (char) 6, (char) 115, (char) 246, (char) 48, (char) 54, (char) 31, (char) 78, (char) 122}));
			assert (pack.satellites_visible_GET() == (char) 116);
		});
		GPS_STATUS p25 = new GPS_STATUS();
		PH.setPack(p25);
		p25.satellite_prn_SET(new char[]{(char) 38, (char) 172, (char) 45, (char) 36, (char) 111, (char) 75, (char) 103, (char) 171, (char) 97, (char) 140, (char) 194, (char) 68, (char) 2, (char) 51, (char) 68, (char) 79, (char) 21, (char) 165, (char) 149, (char) 173}, 0);
		p25.satellites_visible_SET((char) 116);
		p25.satellite_azimuth_SET(new char[]{(char) 6, (char) 8, (char) 163, (char) 110, (char) 145, (char) 229, (char) 45, (char) 137, (char) 182, (char) 243, (char) 217, (char) 105, (char) 99, (char) 194, (char) 240, (char) 83, (char) 228, (char) 31, (char) 105, (char) 29}, 0);
		p25.satellite_snr_SET(new char[]{(char) 184, (char) 225, (char) 76, (char) 204, (char) 108, (char) 106, (char) 11, (char) 198, (char) 50, (char) 19, (char) 174, (char) 4, (char) 113, (char) 24, (char) 123, (char) 228, (char) 175, (char) 147, (char) 188, (char) 67}, 0);
		p25.satellite_used_SET(new char[]{(char) 115, (char) 107, (char) 16, (char) 168, (char) 93, (char) 225, (char) 47, (char) 167, (char) 69, (char) 6, (char) 3, (char) 83, (char) 6, (char) 115, (char) 246, (char) 48, (char) 54, (char) 31, (char) 78, (char) 122}, 0);
		p25.satellite_elevation_SET(new char[]{(char) 84, (char) 84, (char) 243, (char) 71, (char) 142, (char) 125, (char) 122, (char) 79, (char) 2, (char) 45, (char) 60, (char) 31, (char) 9, (char) 14, (char) 161, (char) 108, (char) 1, (char) 123, (char) 252, (char) 1}, 0);
		TestChannel.instance.send(p25);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_SCALED_IMU.add((src, ph, pack) ->
		{
			assert (pack.xacc_GET() == (short) -16548);
			assert (pack.zacc_GET() == (short) 21157);
			assert (pack.ymag_GET() == (short) -24443);
			assert (pack.yacc_GET() == (short) -11687);
			assert (pack.zgyro_GET() == (short) 26326);
			assert (pack.time_boot_ms_GET() == 520084120L);
			assert (pack.zmag_GET() == (short) 25447);
			assert (pack.xmag_GET() == (short) -6680);
			assert (pack.ygyro_GET() == (short) -8998);
			assert (pack.xgyro_GET() == (short) -15677);
		});
		SCALED_IMU p26 = new SCALED_IMU();
		PH.setPack(p26);
		p26.xgyro_SET((short) -15677);
		p26.yacc_SET((short) -11687);
		p26.ygyro_SET((short) -8998);
		p26.zacc_SET((short) 21157);
		p26.xacc_SET((short) -16548);
		p26.xmag_SET((short) -6680);
		p26.zgyro_SET((short) 26326);
		p26.ymag_SET((short) -24443);
		p26.zmag_SET((short) 25447);
		p26.time_boot_ms_SET(520084120L);
		TestChannel.instance.send(p26);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_RAW_IMU.add((src, ph, pack) ->
		{
			assert (pack.zgyro_GET() == (short) -28805);
			assert (pack.xacc_GET() == (short) 13681);
			assert (pack.zacc_GET() == (short) 5544);
			assert (pack.zmag_GET() == (short) -5881);
			assert (pack.ygyro_GET() == (short) 5972);
			assert (pack.xgyro_GET() == (short) 14124);
			assert (pack.xmag_GET() == (short) 2541);
			assert (pack.yacc_GET() == (short) 15597);
			assert (pack.ymag_GET() == (short) 32159);
			assert (pack.time_usec_GET() == 1464663818011033381L);
		});
		RAW_IMU p27 = new RAW_IMU();
		PH.setPack(p27);
		p27.time_usec_SET(1464663818011033381L);
		p27.ygyro_SET((short) 5972);
		p27.xmag_SET((short) 2541);
		p27.xgyro_SET((short) 14124);
		p27.zacc_SET((short) 5544);
		p27.ymag_SET((short) 32159);
		p27.zgyro_SET((short) -28805);
		p27.xacc_SET((short) 13681);
		p27.yacc_SET((short) 15597);
		p27.zmag_SET((short) -5881);
		TestChannel.instance.send(p27);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_RAW_PRESSURE.add((src, ph, pack) ->
		{
			assert (pack.time_usec_GET() == 119915329454059272L);
			assert (pack.press_diff2_GET() == (short) 3007);
			assert (pack.press_diff1_GET() == (short) -6875);
			assert (pack.temperature_GET() == (short) 28300);
			assert (pack.press_abs_GET() == (short) 25400);
		});
		RAW_PRESSURE p28 = new RAW_PRESSURE();
		PH.setPack(p28);
		p28.temperature_SET((short) 28300);
		p28.press_diff1_SET((short) -6875);
		p28.time_usec_SET(119915329454059272L);
		p28.press_diff2_SET((short) 3007);
		p28.press_abs_SET((short) 25400);
		TestChannel.instance.send(p28);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_SCALED_PRESSURE.add((src, ph, pack) ->
		{
			assert (pack.temperature_GET() == (short) 7624);
			assert (pack.press_diff_GET() == -1.0597492E38F);
			assert (pack.press_abs_GET() == -6.0185115E37F);
			assert (pack.time_boot_ms_GET() == 3620961100L);
		});
		SCALED_PRESSURE p29 = new SCALED_PRESSURE();
		PH.setPack(p29);
		p29.time_boot_ms_SET(3620961100L);
		p29.temperature_SET((short) 7624);
		p29.press_abs_SET(-6.0185115E37F);
		p29.press_diff_SET(-1.0597492E38F);
		TestChannel.instance.send(p29);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_ATTITUDE.add((src, ph, pack) ->
		{
			assert (pack.roll_GET() == -2.2702484E38F);
			assert (pack.pitchspeed_GET() == 1.7603509E38F);
			assert (pack.yaw_GET() == 1.3599889E38F);
			assert (pack.time_boot_ms_GET() == 2393186861L);
			assert (pack.pitch_GET() == 9.5508E37F);
			assert (pack.rollspeed_GET() == 8.816106E37F);
			assert (pack.yawspeed_GET() == 1.329614E38F);
		});
		ATTITUDE p30 = new ATTITUDE();
		PH.setPack(p30);
		p30.rollspeed_SET(8.816106E37F);
		p30.pitch_SET(9.5508E37F);
		p30.pitchspeed_SET(1.7603509E38F);
		p30.time_boot_ms_SET(2393186861L);
		p30.yawspeed_SET(1.329614E38F);
		p30.roll_SET(-2.2702484E38F);
		p30.yaw_SET(1.3599889E38F);
		TestChannel.instance.send(p30);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_ATTITUDE_QUATERNION.add((src, ph, pack) ->
		{
			assert (pack.q2_GET() == 1.6220889E38F);
			assert (pack.pitchspeed_GET() == 7.9631265E37F);
			assert (pack.rollspeed_GET() == 2.9169894E38F);
			assert (pack.time_boot_ms_GET() == 3562148798L);
			assert (pack.q3_GET() == 1.6007874E38F);
			assert (pack.q1_GET() == 9.085112E37F);
			assert (pack.yawspeed_GET() == -8.549302E36F);
			assert (pack.q4_GET() == -4.5070577E37F);
		});
		ATTITUDE_QUATERNION p31 = new ATTITUDE_QUATERNION();
		PH.setPack(p31);
		p31.q3_SET(1.6007874E38F);
		p31.pitchspeed_SET(7.9631265E37F);
		p31.rollspeed_SET(2.9169894E38F);
		p31.yawspeed_SET(-8.549302E36F);
		p31.q1_SET(9.085112E37F);
		p31.q4_SET(-4.5070577E37F);
		p31.time_boot_ms_SET(3562148798L);
		p31.q2_SET(1.6220889E38F);
		TestChannel.instance.send(p31);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_LOCAL_POSITION_NED.add((src, ph, pack) ->
		{
			assert (pack.vy_GET() == -2.1778223E38F);
			assert (pack.vx_GET() == -9.516168E37F);
			assert (pack.vz_GET() == 9.723058E37F);
			assert (pack.y_GET() == -2.1368194E38F);
			assert (pack.z_GET() == -1.281674E38F);
			assert (pack.x_GET() == -2.9501988E38F);
			assert (pack.time_boot_ms_GET() == 4238341268L);
		});
		LOCAL_POSITION_NED p32 = new LOCAL_POSITION_NED();
		PH.setPack(p32);
		p32.y_SET(-2.1368194E38F);
		p32.vy_SET(-2.1778223E38F);
		p32.x_SET(-2.9501988E38F);
		p32.time_boot_ms_SET(4238341268L);
		p32.vz_SET(9.723058E37F);
		p32.vx_SET(-9.516168E37F);
		p32.z_SET(-1.281674E38F);
		TestChannel.instance.send(p32);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_GLOBAL_POSITION_INT.add((src, ph, pack) ->
		{
			assert (pack.vy_GET() == (short) -16416);
			assert (pack.relative_alt_GET() == -55754335);
			assert (pack.alt_GET() == -777039463);
			assert (pack.vx_GET() == (short) -15082);
			assert (pack.vz_GET() == (short) 19616);
			assert (pack.lat_GET() == 1617909278);
			assert (pack.hdg_GET() == (char) 47751);
			assert (pack.time_boot_ms_GET() == 2226842189L);
			assert (pack.lon_GET() == 853361662);
		});
		GLOBAL_POSITION_INT p33 = new GLOBAL_POSITION_INT();
		PH.setPack(p33);
		p33.lon_SET(853361662);
		p33.vy_SET((short) -16416);
		p33.hdg_SET((char) 47751);
		p33.vz_SET((short) 19616);
		p33.vx_SET((short) -15082);
		p33.lat_SET(1617909278);
		p33.relative_alt_SET(-55754335);
		p33.alt_SET(-777039463);
		p33.time_boot_ms_SET(2226842189L);
		TestChannel.instance.send(p33);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_RC_CHANNELS_SCALED.add((src, ph, pack) ->
		{
			assert (pack.chan6_scaled_GET() == (short) 13864);
			assert (pack.chan7_scaled_GET() == (short) -7873);
			assert (pack.port_GET() == (char) 138);
			assert (pack.chan2_scaled_GET() == (short) -21092);
			assert (pack.time_boot_ms_GET() == 3775312146L);
			assert (pack.chan5_scaled_GET() == (short) 2066);
			assert (pack.chan3_scaled_GET() == (short) 28831);
			assert (pack.chan4_scaled_GET() == (short) 10889);
			assert (pack.chan8_scaled_GET() == (short) -14212);
			assert (pack.rssi_GET() == (char) 96);
			assert (pack.chan1_scaled_GET() == (short) 18507);
		});
		RC_CHANNELS_SCALED p34 = new RC_CHANNELS_SCALED();
		PH.setPack(p34);
		p34.time_boot_ms_SET(3775312146L);
		p34.chan1_scaled_SET((short) 18507);
		p34.chan4_scaled_SET((short) 10889);
		p34.chan2_scaled_SET((short) -21092);
		p34.rssi_SET((char) 96);
		p34.chan7_scaled_SET((short) -7873);
		p34.port_SET((char) 138);
		p34.chan6_scaled_SET((short) 13864);
		p34.chan8_scaled_SET((short) -14212);
		p34.chan3_scaled_SET((short) 28831);
		p34.chan5_scaled_SET((short) 2066);
		TestChannel.instance.send(p34);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_RC_CHANNELS_RAW.add((src, ph, pack) ->
		{
			assert (pack.time_boot_ms_GET() == 2444016501L);
			assert (pack.chan3_raw_GET() == (char) 9903);
			assert (pack.chan7_raw_GET() == (char) 8514);
			assert (pack.rssi_GET() == (char) 253);
			assert (pack.chan8_raw_GET() == (char) 49548);
			assert (pack.chan6_raw_GET() == (char) 48523);
			assert (pack.chan2_raw_GET() == (char) 48431);
			assert (pack.chan4_raw_GET() == (char) 51934);
			assert (pack.chan1_raw_GET() == (char) 5319);
			assert (pack.chan5_raw_GET() == (char) 58232);
			assert (pack.port_GET() == (char) 20);
		});
		RC_CHANNELS_RAW p35 = new RC_CHANNELS_RAW();
		PH.setPack(p35);
		p35.chan3_raw_SET((char) 9903);
		p35.chan1_raw_SET((char) 5319);
		p35.chan8_raw_SET((char) 49548);
		p35.chan7_raw_SET((char) 8514);
		p35.port_SET((char) 20);
		p35.chan2_raw_SET((char) 48431);
		p35.chan5_raw_SET((char) 58232);
		p35.rssi_SET((char) 253);
		p35.chan6_raw_SET((char) 48523);
		p35.time_boot_ms_SET(2444016501L);
		p35.chan4_raw_SET((char) 51934);
		TestChannel.instance.send(p35);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_SERVO_OUTPUT_RAW.add((src, ph, pack) ->
		{
			assert (pack.servo10_raw_TRY(ph) == (char) 50538);
			assert (pack.servo9_raw_TRY(ph) == (char) 52248);
			assert (pack.servo11_raw_TRY(ph) == (char) 50961);
			assert (pack.servo6_raw_GET() == (char) 22288);
			assert (pack.time_usec_GET() == 1562111325L);
			assert (pack.servo13_raw_TRY(ph) == (char) 47140);
			assert (pack.servo16_raw_TRY(ph) == (char) 4942);
			assert (pack.port_GET() == (char) 185);
			assert (pack.servo12_raw_TRY(ph) == (char) 1671);
			assert (pack.servo15_raw_TRY(ph) == (char) 48802);
			assert (pack.servo5_raw_GET() == (char) 15684);
			assert (pack.servo1_raw_GET() == (char) 64671);
			assert (pack.servo14_raw_TRY(ph) == (char) 64989);
			assert (pack.servo8_raw_GET() == (char) 23328);
			assert (pack.servo4_raw_GET() == (char) 13517);
			assert (pack.servo3_raw_GET() == (char) 43365);
			assert (pack.servo2_raw_GET() == (char) 19530);
			assert (pack.servo7_raw_GET() == (char) 36937);
		});
		SERVO_OUTPUT_RAW p36 = new SERVO_OUTPUT_RAW();
		PH.setPack(p36);
		p36.servo2_raw_SET((char) 19530);
		p36.servo16_raw_SET((char) 4942, PH);
		p36.servo4_raw_SET((char) 13517);
		p36.servo14_raw_SET((char) 64989, PH);
		p36.servo3_raw_SET((char) 43365);
		p36.servo9_raw_SET((char) 52248, PH);
		p36.servo7_raw_SET((char) 36937);
		p36.servo1_raw_SET((char) 64671);
		p36.servo8_raw_SET((char) 23328);
		p36.servo12_raw_SET((char) 1671, PH);
		p36.servo15_raw_SET((char) 48802, PH);
		p36.servo11_raw_SET((char) 50961, PH);
		p36.servo13_raw_SET((char) 47140, PH);
		p36.port_SET((char) 185);
		p36.servo5_raw_SET((char) 15684);
		p36.servo10_raw_SET((char) 50538, PH);
		p36.servo6_raw_SET((char) 22288);
		p36.time_usec_SET(1562111325L);
		TestChannel.instance.send(p36);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_MISSION_REQUEST_PARTIAL_LIST.add((src, ph, pack) ->
		{
			assert (pack.start_index_GET() == (short) -32542);
			assert (pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
			assert (pack.end_index_GET() == (short) -13113);
			assert (pack.target_component_GET() == (char) 71);
			assert (pack.target_system_GET() == (char) 167);
		});
		MISSION_REQUEST_PARTIAL_LIST p37 = new MISSION_REQUEST_PARTIAL_LIST();
		PH.setPack(p37);
		p37.end_index_SET((short) -13113);
		p37.start_index_SET((short) -32542);
		p37.target_system_SET((char) 167);
		p37.target_component_SET((char) 71);
		p37.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
		TestChannel.instance.send(p37);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_MISSION_WRITE_PARTIAL_LIST.add((src, ph, pack) ->
		{
			assert (pack.target_system_GET() == (char) 144);
			assert (pack.end_index_GET() == (short) -21868);
			assert (pack.start_index_GET() == (short) -11513);
			assert (pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
			assert (pack.target_component_GET() == (char) 243);
		});
		MISSION_WRITE_PARTIAL_LIST p38 = new MISSION_WRITE_PARTIAL_LIST();
		PH.setPack(p38);
		p38.start_index_SET((short) -11513);
		p38.target_component_SET((char) 243);
		p38.end_index_SET((short) -21868);
		p38.target_system_SET((char) 144);
		p38.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
		TestChannel.instance.send(p38);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_MISSION_ITEM.add((src, ph, pack) ->
		{
			assert (pack.x_GET() == 1.6845869E38F);
			assert (pack.autocontinue_GET() == (char) 173);
			assert (pack.z_GET() == -2.1176235E38F);
			assert (pack.target_system_GET() == (char) 198);
			assert (pack.seq_GET() == (char) 9480);
			assert (pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT);
			assert (pack.param4_GET() == 2.5805786E38F);
			assert (pack.param3_GET() == 8.825238E37F);
			assert (pack.param2_GET() == 1.828814E38F);
			assert (pack.param1_GET() == 3.3612109E38F);
			assert (pack.target_component_GET() == (char) 244);
			assert (pack.y_GET() == -2.0673546E38F);
			assert (pack.command_GET() == MAV_CMD.MAV_CMD_DO_DIGICAM_CONFIGURE);
			assert (pack.current_GET() == (char) 192);
			assert (pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
		});
		MISSION_ITEM p39 = new MISSION_ITEM();
		PH.setPack(p39);
		p39.x_SET(1.6845869E38F);
		p39.z_SET(-2.1176235E38F);
		p39.command_SET(MAV_CMD.MAV_CMD_DO_DIGICAM_CONFIGURE);
		p39.target_system_SET((char) 198);
		p39.autocontinue_SET((char) 173);
		p39.param2_SET(1.828814E38F);
		p39.target_component_SET((char) 244);
		p39.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT);
		p39.param4_SET(2.5805786E38F);
		p39.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
		p39.param3_SET(8.825238E37F);
		p39.y_SET(-2.0673546E38F);
		p39.seq_SET((char) 9480);
		p39.current_SET((char) 192);
		p39.param1_SET(3.3612109E38F);
		TestChannel.instance.send(p39);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_MISSION_REQUEST.add((src, ph, pack) ->
		{
			assert (pack.target_system_GET() == (char) 220);
			assert (pack.seq_GET() == (char) 39845);
			assert (pack.target_component_GET() == (char) 134);
			assert (pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
		});
		MISSION_REQUEST p40 = new MISSION_REQUEST();
		PH.setPack(p40);
		p40.target_component_SET((char) 134);
		p40.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
		p40.target_system_SET((char) 220);
		p40.seq_SET((char) 39845);
		TestChannel.instance.send(p40);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_MISSION_SET_CURRENT.add((src, ph, pack) ->
		{
			assert (pack.seq_GET() == (char) 34386);
			assert (pack.target_component_GET() == (char) 88);
			assert (pack.target_system_GET() == (char) 230);
		});
		MISSION_SET_CURRENT p41 = new MISSION_SET_CURRENT();
		PH.setPack(p41);
		p41.target_system_SET((char) 230);
		p41.target_component_SET((char) 88);
		p41.seq_SET((char) 34386);
		TestChannel.instance.send(p41);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_MISSION_CURRENT.add((src, ph, pack) ->
		{
			assert (pack.seq_GET() == (char) 9685);
		});
		MISSION_CURRENT p42 = new MISSION_CURRENT();
		PH.setPack(p42);
		p42.seq_SET((char) 9685);
		TestChannel.instance.send(p42);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_MISSION_REQUEST_LIST.add((src, ph, pack) ->
		{
			assert (pack.target_system_GET() == (char) 125);
			assert (pack.target_component_GET() == (char) 133);
			assert (pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
		});
		MISSION_REQUEST_LIST p43 = new MISSION_REQUEST_LIST();
		PH.setPack(p43);
		p43.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
		p43.target_system_SET((char) 125);
		p43.target_component_SET((char) 133);
		TestChannel.instance.send(p43);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_MISSION_COUNT.add((src, ph, pack) ->
		{
			assert (pack.count_GET() == (char) 35946);
			assert (pack.target_system_GET() == (char) 14);
			assert (pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
			assert (pack.target_component_GET() == (char) 118);
		});
		MISSION_COUNT p44 = new MISSION_COUNT();
		PH.setPack(p44);
		p44.count_SET((char) 35946);
		p44.target_system_SET((char) 14);
		p44.target_component_SET((char) 118);
		p44.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
		TestChannel.instance.send(p44);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_MISSION_CLEAR_ALL.add((src, ph, pack) ->
		{
			assert (pack.target_component_GET() == (char) 207);
			assert (pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
			assert (pack.target_system_GET() == (char) 172);
		});
		MISSION_CLEAR_ALL p45 = new MISSION_CLEAR_ALL();
		PH.setPack(p45);
		p45.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
		p45.target_component_SET((char) 207);
		p45.target_system_SET((char) 172);
		TestChannel.instance.send(p45);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_MISSION_ITEM_REACHED.add((src, ph, pack) ->
		{
			assert (pack.seq_GET() == (char) 17400);
		});
		MISSION_ITEM_REACHED p46 = new MISSION_ITEM_REACHED();
		PH.setPack(p46);
		p46.seq_SET((char) 17400);
		TestChannel.instance.send(p46);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_MISSION_ACK.add((src, ph, pack) ->
		{
			assert (pack.target_component_GET() == (char) 56);
			assert (pack.type_GET() == MAV_MISSION_RESULT.MAV_MISSION_INVALID_PARAM1);
			assert (pack.target_system_GET() == (char) 6);
			assert (pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
		});
		MISSION_ACK p47 = new MISSION_ACK();
		PH.setPack(p47);
		p47.target_system_SET((char) 6);
		p47.type_SET(MAV_MISSION_RESULT.MAV_MISSION_INVALID_PARAM1);
		p47.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
		p47.target_component_SET((char) 56);
		TestChannel.instance.send(p47);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_SET_GPS_GLOBAL_ORIGIN.add((src, ph, pack) ->
		{
			assert (pack.altitude_GET() == -1091909121);
			assert (pack.latitude_GET() == -1265854962);
			assert (pack.target_system_GET() == (char) 179);
			assert (pack.time_usec_TRY(ph) == 2614109933768030284L);
			assert (pack.longitude_GET() == 380366367);
		});
		SET_GPS_GLOBAL_ORIGIN p48 = new SET_GPS_GLOBAL_ORIGIN();
		PH.setPack(p48);
		p48.latitude_SET(-1265854962);
		p48.time_usec_SET(2614109933768030284L, PH);
		p48.longitude_SET(380366367);
		p48.altitude_SET(-1091909121);
		p48.target_system_SET((char) 179);
		TestChannel.instance.send(p48);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_GPS_GLOBAL_ORIGIN.add((src, ph, pack) ->
		{
			assert (pack.longitude_GET() == 493031143);
			assert (pack.time_usec_TRY(ph) == 3852722845659352901L);
			assert (pack.altitude_GET() == 1306884716);
			assert (pack.latitude_GET() == -1222855947);
		});
		GPS_GLOBAL_ORIGIN p49 = new GPS_GLOBAL_ORIGIN();
		PH.setPack(p49);
		p49.altitude_SET(1306884716);
		p49.latitude_SET(-1222855947);
		p49.time_usec_SET(3852722845659352901L, PH);
		p49.longitude_SET(493031143);
		TestChannel.instance.send(p49);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_PARAM_MAP_RC.add((src, ph, pack) ->
		{
			assert (pack.target_component_GET() == (char) 127);
			assert (pack.param_id_LEN(ph) == 4);
			assert (pack.param_id_TRY(ph).equals("obfr"));
			assert (pack.scale_GET() == -3.1435323E38F);
			assert (pack.param_value_max_GET() == 3.2800638E38F);
			assert (pack.parameter_rc_channel_index_GET() == (char) 138);
			assert (pack.param_value0_GET() == -5.4361224E37F);
			assert (pack.target_system_GET() == (char) 51);
			assert (pack.param_index_GET() == (short) -14654);
			assert (pack.param_value_min_GET() == 2.1890537E38F);
		});
		PARAM_MAP_RC p50 = new PARAM_MAP_RC();
		PH.setPack(p50);
		p50.param_id_SET("obfr", PH);
		p50.scale_SET(-3.1435323E38F);
		p50.parameter_rc_channel_index_SET((char) 138);
		p50.param_value0_SET(-5.4361224E37F);
		p50.target_component_SET((char) 127);
		p50.param_index_SET((short) -14654);
		p50.param_value_max_SET(3.2800638E38F);
		p50.target_system_SET((char) 51);
		p50.param_value_min_SET(2.1890537E38F);
		TestChannel.instance.send(p50);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_MISSION_REQUEST_INT.add((src, ph, pack) ->
		{
			assert (pack.target_component_GET() == (char) 62);
			assert (pack.target_system_GET() == (char) 188);
			assert (pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
			assert (pack.seq_GET() == (char) 54548);
		});
		MISSION_REQUEST_INT p51 = new MISSION_REQUEST_INT();
		PH.setPack(p51);
		p51.seq_SET((char) 54548);
		p51.target_component_SET((char) 62);
		p51.target_system_SET((char) 188);
		p51.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
		TestChannel.instance.send(p51);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_SAFETY_SET_ALLOWED_AREA.add((src, ph, pack) ->
		{
			assert (pack.p2y_GET() == -1.0977129E38F);
			assert (pack.frame_GET() == MAV_FRAME.MAV_FRAME_LOCAL_ENU);
			assert (pack.target_component_GET() == (char) 24);
			assert (pack.p1y_GET() == -2.3310168E37F);
			assert (pack.p1z_GET() == 1.3228884E38F);
			assert (pack.target_system_GET() == (char) 236);
			assert (pack.p2z_GET() == -2.7937155E38F);
			assert (pack.p2x_GET() == 1.118441E38F);
			assert (pack.p1x_GET() == -1.7175532E38F);
		});
		SAFETY_SET_ALLOWED_AREA p54 = new SAFETY_SET_ALLOWED_AREA();
		PH.setPack(p54);
		p54.p2z_SET(-2.7937155E38F);
		p54.p1y_SET(-2.3310168E37F);
		p54.p1x_SET(-1.7175532E38F);
		p54.p2y_SET(-1.0977129E38F);
		p54.frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_ENU);
		p54.p2x_SET(1.118441E38F);
		p54.target_component_SET((char) 24);
		p54.p1z_SET(1.3228884E38F);
		p54.target_system_SET((char) 236);
		TestChannel.instance.send(p54);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_SAFETY_ALLOWED_AREA.add((src, ph, pack) ->
		{
			assert (pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT);
			assert (pack.p2z_GET() == 3.137969E37F);
			assert (pack.p1y_GET() == 2.5329328E38F);
			assert (pack.p1x_GET() == -1.5817495E38F);
			assert (pack.p2x_GET() == -1.263136E38F);
			assert (pack.p2y_GET() == -9.878617E37F);
			assert (pack.p1z_GET() == 2.3517845E38F);
		});
		SAFETY_ALLOWED_AREA p55 = new SAFETY_ALLOWED_AREA();
		PH.setPack(p55);
		p55.p2z_SET(3.137969E37F);
		p55.p1y_SET(2.5329328E38F);
		p55.p1z_SET(2.3517845E38F);
		p55.p2y_SET(-9.878617E37F);
		p55.p2x_SET(-1.263136E38F);
		p55.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT);
		p55.p1x_SET(-1.5817495E38F);
		TestChannel.instance.send(p55);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_ATTITUDE_QUATERNION_COV.add((src, ph, pack) ->
		{
			assert (Arrays.equals(pack.q_GET(), new float[]{1.9442397E38F, -7.5810215E36F, -1.6573633E38F, -3.2653028E38F}));
			assert (pack.yawspeed_GET() == 2.6908314E38F);
			assert (pack.rollspeed_GET() == -1.4571903E38F);
			assert (pack.pitchspeed_GET() == -1.7194922E38F);
			assert (pack.time_usec_GET() == 7152561137271536701L);
			assert (Arrays.equals(pack.covariance_GET(), new float[]{-7.2181303E37F, 3.2093502E38F, 1.0041085E38F, 2.1213524E38F, 3.2799954E38F, 2.4951207E38F, -2.9874993E38F, 2.950087E38F, -2.5684719E38F}));
		});
		ATTITUDE_QUATERNION_COV p61 = new ATTITUDE_QUATERNION_COV();
		PH.setPack(p61);
		p61.q_SET(new float[]{1.9442397E38F, -7.5810215E36F, -1.6573633E38F, -3.2653028E38F}, 0);
		p61.yawspeed_SET(2.6908314E38F);
		p61.rollspeed_SET(-1.4571903E38F);
		p61.pitchspeed_SET(-1.7194922E38F);
		p61.time_usec_SET(7152561137271536701L);
		p61.covariance_SET(new float[]{-7.2181303E37F, 3.2093502E38F, 1.0041085E38F, 2.1213524E38F, 3.2799954E38F, 2.4951207E38F, -2.9874993E38F, 2.950087E38F, -2.5684719E38F}, 0);
		TestChannel.instance.send(p61);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_NAV_CONTROLLER_OUTPUT.add((src, ph, pack) ->
		{
			assert (pack.wp_dist_GET() == (char) 57042);
			assert (pack.aspd_error_GET() == -1.0773059E38F);
			assert (pack.target_bearing_GET() == (short) -32282);
			assert (pack.nav_pitch_GET() == -4.5038515E37F);
			assert (pack.xtrack_error_GET() == 3.0072719E38F);
			assert (pack.nav_bearing_GET() == (short) 23075);
			assert (pack.alt_error_GET() == -3.3636239E38F);
			assert (pack.nav_roll_GET() == 2.7440138E37F);
		});
		NAV_CONTROLLER_OUTPUT p62 = new NAV_CONTROLLER_OUTPUT();
		PH.setPack(p62);
		p62.aspd_error_SET(-1.0773059E38F);
		p62.target_bearing_SET((short) -32282);
		p62.alt_error_SET(-3.3636239E38F);
		p62.xtrack_error_SET(3.0072719E38F);
		p62.nav_bearing_SET((short) 23075);
		p62.nav_roll_SET(2.7440138E37F);
		p62.wp_dist_SET((char) 57042);
		p62.nav_pitch_SET(-4.5038515E37F);
		TestChannel.instance.send(p62);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_GLOBAL_POSITION_INT_COV.add((src, ph, pack) ->
		{
			assert (Arrays.equals(pack.covariance_GET(), new float[]{-2.2442744E38F, -2.5908426E38F, -2.56406E38F, 2.2454547E37F, -2.3689272E38F, -1.423686E38F, -1.7029711E38F, 1.0654069E38F, -2.6824302E38F, 2.1311116E37F, -1.6379579E38F, 1.6871751E38F, 2.7894211E38F, 2.174623E38F, 3.3445766E38F, -1.2454798E38F, 1.4509135E38F, -2.0615988E38F, 3.1635002E38F, -3.0128797E38F, 2.0293573E38F, -1.8953003E38F, 1.0737111E38F, 2.7227415E38F, 2.727566E38F, 1.4640898E38F, -3.2970487E38F, 9.482729E37F, 1.5495494E38F, -2.5215518E38F, -8.4175194E37F, -2.473683E38F, 3.348476E38F, -1.0474579E38F, 2.882306E38F, -3.052793E38F}));
			assert (pack.vz_GET() == 2.6796967E38F);
			assert (pack.time_usec_GET() == 4506010571094532007L);
			assert (pack.lon_GET() == 870798120);
			assert (pack.estimator_type_GET() == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VISION);
			assert (pack.vx_GET() == -9.552383E37F);
			assert (pack.relative_alt_GET() == 1586709832);
			assert (pack.lat_GET() == -1017041495);
			assert (pack.alt_GET() == -1041735941);
			assert (pack.vy_GET() == 9.990493E37F);
		});
		GLOBAL_POSITION_INT_COV p63 = new GLOBAL_POSITION_INT_COV();
		PH.setPack(p63);
		p63.vz_SET(2.6796967E38F);
		p63.covariance_SET(new float[]{-2.2442744E38F, -2.5908426E38F, -2.56406E38F, 2.2454547E37F, -2.3689272E38F, -1.423686E38F, -1.7029711E38F, 1.0654069E38F, -2.6824302E38F, 2.1311116E37F, -1.6379579E38F, 1.6871751E38F, 2.7894211E38F, 2.174623E38F, 3.3445766E38F, -1.2454798E38F, 1.4509135E38F, -2.0615988E38F, 3.1635002E38F, -3.0128797E38F, 2.0293573E38F, -1.8953003E38F, 1.0737111E38F, 2.7227415E38F, 2.727566E38F, 1.4640898E38F, -3.2970487E38F, 9.482729E37F, 1.5495494E38F, -2.5215518E38F, -8.4175194E37F, -2.473683E38F, 3.348476E38F, -1.0474579E38F, 2.882306E38F, -3.052793E38F}, 0);
		p63.estimator_type_SET(MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VISION);
		p63.lon_SET(870798120);
		p63.time_usec_SET(4506010571094532007L);
		p63.vy_SET(9.990493E37F);
		p63.lat_SET(-1017041495);
		p63.relative_alt_SET(1586709832);
		p63.vx_SET(-9.552383E37F);
		p63.alt_SET(-1041735941);
		TestChannel.instance.send(p63);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_LOCAL_POSITION_NED_COV.add((src, ph, pack) ->
		{
			assert (pack.estimator_type_GET() == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS_INS);
			assert (pack.vx_GET() == -2.1935795E38F);
			assert (pack.ax_GET() == 1.7581328E38F);
			assert (pack.vy_GET() == 3.2344018E38F);
			assert (pack.z_GET() == -1.6316824E38F);
			assert (pack.az_GET() == 2.187146E38F);
			assert (pack.x_GET() == 1.0776096E38F);
			assert (pack.ay_GET() == -1.5915361E38F);
			assert (Arrays.equals(pack.covariance_GET(), new float[]{-2.2837113E38F, -1.8936656E38F, 3.696733E37F, -2.7336301E38F, 9.313987E37F, 2.807367E38F, 1.9027467E37F, -3.3616715E38F, -4.657092E37F, -1.798432E38F, 3.1685685E38F, 2.4406838E38F, 5.39515E37F, -2.628938E38F, 3.2779033E38F, -2.9100554E38F, -2.533015E38F, 6.770142E37F, 6.209429E37F, 1.9673195E38F, -2.7624743E37F, -4.2413961E37F, -1.8432127E38F, -3.2439486E36F, -1.6936811E38F, -6.1833816E37F, 2.4540365E38F, -1.4606164E38F, -3.0519092E38F, -3.2247421E37F, 2.3991624E38F, -1.8900506E37F, -2.0875514E38F, -1.2642802E38F, 6.203947E37F, -3.1351135E38F, 4.405578E37F, -2.987426E38F, -7.317133E37F, 2.8064565E38F, -8.087757E37F, -1.6573369E38F, 3.1722979E38F, -3.144113E38F, -2.8506738E38F}));
			assert (pack.y_GET() == -5.1495294E37F);
			assert (pack.vz_GET() == -3.2056701E38F);
			assert (pack.time_usec_GET() == 2551759688256584236L);
		});
		LOCAL_POSITION_NED_COV p64 = new LOCAL_POSITION_NED_COV();
		PH.setPack(p64);
		p64.vx_SET(-2.1935795E38F);
		p64.time_usec_SET(2551759688256584236L);
		p64.vy_SET(3.2344018E38F);
		p64.y_SET(-5.1495294E37F);
		p64.ax_SET(1.7581328E38F);
		p64.covariance_SET(new float[]{-2.2837113E38F, -1.8936656E38F, 3.696733E37F, -2.7336301E38F, 9.313987E37F, 2.807367E38F, 1.9027467E37F, -3.3616715E38F, -4.657092E37F, -1.798432E38F, 3.1685685E38F, 2.4406838E38F, 5.39515E37F, -2.628938E38F, 3.2779033E38F, -2.9100554E38F, -2.533015E38F, 6.770142E37F, 6.209429E37F, 1.9673195E38F, -2.7624743E37F, -4.2413961E37F, -1.8432127E38F, -3.2439486E36F, -1.6936811E38F, -6.1833816E37F, 2.4540365E38F, -1.4606164E38F, -3.0519092E38F, -3.2247421E37F, 2.3991624E38F, -1.8900506E37F, -2.0875514E38F, -1.2642802E38F, 6.203947E37F, -3.1351135E38F, 4.405578E37F, -2.987426E38F, -7.317133E37F, 2.8064565E38F, -8.087757E37F, -1.6573369E38F, 3.1722979E38F, -3.144113E38F, -2.8506738E38F}, 0);
		p64.estimator_type_SET(MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS_INS);
		p64.az_SET(2.187146E38F);
		p64.vz_SET(-3.2056701E38F);
		p64.ay_SET(-1.5915361E38F);
		p64.z_SET(-1.6316824E38F);
		p64.x_SET(1.0776096E38F);
		TestChannel.instance.send(p64);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_RC_CHANNELS.add((src, ph, pack) ->
		{
			assert (pack.chan7_raw_GET() == (char) 31074);
			assert (pack.chan2_raw_GET() == (char) 61777);
			assert (pack.chan5_raw_GET() == (char) 12340);
			assert (pack.chan4_raw_GET() == (char) 51172);
			assert (pack.chan12_raw_GET() == (char) 14568);
			assert (pack.chan3_raw_GET() == (char) 26146);
			assert (pack.chan15_raw_GET() == (char) 47287);
			assert (pack.chan14_raw_GET() == (char) 42179);
			assert (pack.chan9_raw_GET() == (char) 63684);
			assert (pack.chan6_raw_GET() == (char) 2355);
			assert (pack.chan11_raw_GET() == (char) 8049);
			assert (pack.chan8_raw_GET() == (char) 37284);
			assert (pack.chan16_raw_GET() == (char) 27174);
			assert (pack.rssi_GET() == (char) 193);
			assert (pack.chancount_GET() == (char) 122);
			assert (pack.chan17_raw_GET() == (char) 24445);
			assert (pack.chan13_raw_GET() == (char) 46905);
			assert (pack.chan1_raw_GET() == (char) 44558);
			assert (pack.chan18_raw_GET() == (char) 24045);
			assert (pack.time_boot_ms_GET() == 2784267533L);
			assert (pack.chan10_raw_GET() == (char) 19686);
		});
		RC_CHANNELS p65 = new RC_CHANNELS();
		PH.setPack(p65);
		p65.time_boot_ms_SET(2784267533L);
		p65.chan11_raw_SET((char) 8049);
		p65.chan14_raw_SET((char) 42179);
		p65.chan3_raw_SET((char) 26146);
		p65.chan17_raw_SET((char) 24445);
		p65.chan6_raw_SET((char) 2355);
		p65.chan7_raw_SET((char) 31074);
		p65.chan5_raw_SET((char) 12340);
		p65.chan16_raw_SET((char) 27174);
		p65.chan12_raw_SET((char) 14568);
		p65.chan13_raw_SET((char) 46905);
		p65.chan15_raw_SET((char) 47287);
		p65.chancount_SET((char) 122);
		p65.chan18_raw_SET((char) 24045);
		p65.chan1_raw_SET((char) 44558);
		p65.chan2_raw_SET((char) 61777);
		p65.chan10_raw_SET((char) 19686);
		p65.chan8_raw_SET((char) 37284);
		p65.rssi_SET((char) 193);
		p65.chan4_raw_SET((char) 51172);
		p65.chan9_raw_SET((char) 63684);
		TestChannel.instance.send(p65);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_REQUEST_DATA_STREAM.add((src, ph, pack) ->
		{
			assert (pack.target_system_GET() == (char) 226);
			assert (pack.req_stream_id_GET() == (char) 61);
			assert (pack.start_stop_GET() == (char) 73);
			assert (pack.target_component_GET() == (char) 232);
			assert (pack.req_message_rate_GET() == (char) 32625);
		});
		REQUEST_DATA_STREAM p66 = new REQUEST_DATA_STREAM();
		PH.setPack(p66);
		p66.start_stop_SET((char) 73);
		p66.target_system_SET((char) 226);
		p66.req_message_rate_SET((char) 32625);
		p66.target_component_SET((char) 232);
		p66.req_stream_id_SET((char) 61);
		TestChannel.instance.send(p66);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_DATA_STREAM.add((src, ph, pack) ->
		{
			assert (pack.stream_id_GET() == (char) 172);
			assert (pack.message_rate_GET() == (char) 37069);
			assert (pack.on_off_GET() == (char) 116);
		});
		DATA_STREAM p67 = new DATA_STREAM();
		PH.setPack(p67);
		p67.message_rate_SET((char) 37069);
		p67.on_off_SET((char) 116);
		p67.stream_id_SET((char) 172);
		TestChannel.instance.send(p67);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_MANUAL_CONTROL.add((src, ph, pack) ->
		{
			assert (pack.y_GET() == (short) -3506);
			assert (pack.target_GET() == (char) 213);
			assert (pack.z_GET() == (short) -11634);
			assert (pack.x_GET() == (short) 19497);
			assert (pack.r_GET() == (short) -28755);
			assert (pack.buttons_GET() == (char) 50109);
		});
		MANUAL_CONTROL p69 = new MANUAL_CONTROL();
		PH.setPack(p69);
		p69.z_SET((short) -11634);
		p69.buttons_SET((char) 50109);
		p69.y_SET((short) -3506);
		p69.r_SET((short) -28755);
		p69.target_SET((char) 213);
		p69.x_SET((short) 19497);
		TestChannel.instance.send(p69);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_RC_CHANNELS_OVERRIDE.add((src, ph, pack) ->
		{
			assert (pack.target_system_GET() == (char) 223);
			assert (pack.target_component_GET() == (char) 132);
			assert (pack.chan2_raw_GET() == (char) 50306);
			assert (pack.chan3_raw_GET() == (char) 9210);
			assert (pack.chan8_raw_GET() == (char) 47471);
			assert (pack.chan6_raw_GET() == (char) 61259);
			assert (pack.chan7_raw_GET() == (char) 32119);
			assert (pack.chan4_raw_GET() == (char) 26329);
			assert (pack.chan5_raw_GET() == (char) 22288);
			assert (pack.chan1_raw_GET() == (char) 27315);
		});
		RC_CHANNELS_OVERRIDE p70 = new RC_CHANNELS_OVERRIDE();
		PH.setPack(p70);
		p70.chan7_raw_SET((char) 32119);
		p70.chan1_raw_SET((char) 27315);
		p70.chan3_raw_SET((char) 9210);
		p70.target_component_SET((char) 132);
		p70.chan6_raw_SET((char) 61259);
		p70.chan2_raw_SET((char) 50306);
		p70.chan8_raw_SET((char) 47471);
		p70.chan5_raw_SET((char) 22288);
		p70.target_system_SET((char) 223);
		p70.chan4_raw_SET((char) 26329);
		TestChannel.instance.send(p70);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_MISSION_ITEM_INT.add((src, ph, pack) ->
		{
			assert (pack.y_GET() == -1472844585);
			assert (pack.current_GET() == (char) 233);
			assert (pack.param2_GET() == -3.0952981E38F);
			assert (pack.command_GET() == MAV_CMD.MAV_CMD_DO_FOLLOW_REPOSITION);
			assert (pack.x_GET() == -270703787);
			assert (pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_INT);
			assert (pack.seq_GET() == (char) 64619);
			assert (pack.target_component_GET() == (char) 61);
			assert (pack.param3_GET() == 1.4475022E37F);
			assert (pack.param1_GET() == 2.1500482E38F);
			assert (pack.autocontinue_GET() == (char) 181);
			assert (pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
			assert (pack.target_system_GET() == (char) 182);
			assert (pack.z_GET() == 2.207091E38F);
			assert (pack.param4_GET() == -1.4935015E38F);
		});
		MISSION_ITEM_INT p73 = new MISSION_ITEM_INT();
		PH.setPack(p73);
		p73.param4_SET(-1.4935015E38F);
		p73.current_SET((char) 233);
		p73.command_SET(MAV_CMD.MAV_CMD_DO_FOLLOW_REPOSITION);
		p73.param3_SET(1.4475022E37F);
		p73.seq_SET((char) 64619);
		p73.autocontinue_SET((char) 181);
		p73.target_system_SET((char) 182);
		p73.target_component_SET((char) 61);
		p73.param2_SET(-3.0952981E38F);
		p73.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
		p73.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_INT);
		p73.z_SET(2.207091E38F);
		p73.param1_SET(2.1500482E38F);
		p73.x_SET(-270703787);
		p73.y_SET(-1472844585);
		TestChannel.instance.send(p73);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_VFR_HUD.add((src, ph, pack) ->
		{
			assert (pack.alt_GET() == -2.601349E38F);
			assert (pack.heading_GET() == (short) 30674);
			assert (pack.throttle_GET() == (char) 23206);
			assert (pack.airspeed_GET() == -1.5177489E38F);
			assert (pack.climb_GET() == -1.3891542E38F);
			assert (pack.groundspeed_GET() == -1.6465799E38F);
		});
		VFR_HUD p74 = new VFR_HUD();
		PH.setPack(p74);
		p74.heading_SET((short) 30674);
		p74.climb_SET(-1.3891542E38F);
		p74.throttle_SET((char) 23206);
		p74.airspeed_SET(-1.5177489E38F);
		p74.groundspeed_SET(-1.6465799E38F);
		p74.alt_SET(-2.601349E38F);
		TestChannel.instance.send(p74);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_COMMAND_INT.add((src, ph, pack) ->
		{
			assert (pack.frame_GET() == MAV_FRAME.MAV_FRAME_BODY_NED);
			assert (pack.target_component_GET() == (char) 205);
			assert (pack.target_system_GET() == (char) 54);
			assert (pack.param2_GET() == 1.0361247E36F);
			assert (pack.y_GET() == 899797459);
			assert (pack.autocontinue_GET() == (char) 1);
			assert (pack.param1_GET() == -3.013709E38F);
			assert (pack.command_GET() == MAV_CMD.MAV_CMD_SPATIAL_USER_5);
			assert (pack.param3_GET() == 3.0016542E38F);
			assert (pack.x_GET() == 1868911061);
			assert (pack.param4_GET() == -3.1416146E38F);
			assert (pack.current_GET() == (char) 186);
			assert (pack.z_GET() == 2.8038352E38F);
		});
		COMMAND_INT p75 = new COMMAND_INT();
		PH.setPack(p75);
		p75.target_component_SET((char) 205);
		p75.z_SET(2.8038352E38F);
		p75.param4_SET(-3.1416146E38F);
		p75.command_SET(MAV_CMD.MAV_CMD_SPATIAL_USER_5);
		p75.frame_SET(MAV_FRAME.MAV_FRAME_BODY_NED);
		p75.autocontinue_SET((char) 1);
		p75.param3_SET(3.0016542E38F);
		p75.param1_SET(-3.013709E38F);
		p75.param2_SET(1.0361247E36F);
		p75.y_SET(899797459);
		p75.x_SET(1868911061);
		p75.current_SET((char) 186);
		p75.target_system_SET((char) 54);
		TestChannel.instance.send(p75);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_COMMAND_LONG.add((src, ph, pack) ->
		{
			assert (pack.param1_GET() == -6.704939E37F);
			assert (pack.param2_GET() == 2.5709297E38F);
			assert (pack.command_GET() == MAV_CMD.MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION);
			assert (pack.param4_GET() == -1.220082E36F);
			assert (pack.param6_GET() == -2.0496924E38F);
			assert (pack.param3_GET() == -1.161028E38F);
			assert (pack.confirmation_GET() == (char) 72);
			assert (pack.param5_GET() == -3.4146245E36F);
			assert (pack.param7_GET() == -1.1715764E38F);
			assert (pack.target_component_GET() == (char) 77);
			assert (pack.target_system_GET() == (char) 235);
		});
		GroundControl.COMMAND_LONG p76 = CommunicationChannel.new_COMMAND_LONG();
		PH.setPack(p76);
		p76.param7_SET(-1.1715764E38F);
		p76.target_component_SET((char) 77);
		p76.command_SET(MAV_CMD.MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION);
		p76.target_system_SET((char) 235);
		p76.param6_SET(-2.0496924E38F);
		p76.param5_SET(-3.4146245E36F);
		p76.param2_SET(2.5709297E38F);
		p76.param3_SET(-1.161028E38F);
		p76.param4_SET(-1.220082E36F);
		p76.confirmation_SET((char) 72);
		p76.param1_SET(-6.704939E37F);
		CommunicationChannel.instance.send(p76);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_COMMAND_ACK.add((src, ph, pack) ->
		{
			assert (pack.result_param2_TRY(ph) == -1327980824);
			assert (pack.progress_TRY(ph) == (char) 159);
			assert (pack.target_system_TRY(ph) == (char) 191);
			assert (pack.result_GET() == MAV_RESULT.MAV_RESULT_FAILED);
			assert (pack.target_component_TRY(ph) == (char) 60);
			assert (pack.command_GET() == MAV_CMD.MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS);
		});
		GroundControl.COMMAND_ACK p77 = CommunicationChannel.new_COMMAND_ACK();
		PH.setPack(p77);
		p77.target_system_SET((char) 191, PH);
		p77.result_SET(MAV_RESULT.MAV_RESULT_FAILED);
		p77.progress_SET((char) 159, PH);
		p77.target_component_SET((char) 60, PH);
		p77.result_param2_SET(-1327980824, PH);
		p77.command_SET(MAV_CMD.MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS);
		CommunicationChannel.instance.send(p77);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_MANUAL_SETPOINT.add((src, ph, pack) ->
		{
			assert (pack.pitch_GET() == 9.207578E37F);
			assert (pack.manual_override_switch_GET() == (char) 44);
			assert (pack.yaw_GET() == 2.3495435E38F);
			assert (pack.time_boot_ms_GET() == 61539475L);
			assert (pack.mode_switch_GET() == (char) 35);
			assert (pack.roll_GET() == 2.9051294E38F);
			assert (pack.thrust_GET() == 2.0426613E38F);
		});
		GroundControl.MANUAL_SETPOINT p81 = CommunicationChannel.new_MANUAL_SETPOINT();
		PH.setPack(p81);
		p81.roll_SET(2.9051294E38F);
		p81.time_boot_ms_SET(61539475L);
		p81.thrust_SET(2.0426613E38F);
		p81.pitch_SET(9.207578E37F);
		p81.manual_override_switch_SET((char) 44);
		p81.mode_switch_SET((char) 35);
		p81.yaw_SET(2.3495435E38F);
		CommunicationChannel.instance.send(p81);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_SET_ATTITUDE_TARGET.add((src, ph, pack) ->
		{
			assert (pack.time_boot_ms_GET() == 2380490332L);
			assert (pack.body_pitch_rate_GET() == 9.803544E37F);
			assert (pack.target_system_GET() == (char) 51);
			assert (pack.target_component_GET() == (char) 185);
			assert (pack.body_yaw_rate_GET() == -3.3275386E38F);
			assert (pack.body_roll_rate_GET() == 1.6395553E38F);
			assert (pack.thrust_GET() == -1.244467E38F);
			assert (pack.type_mask_GET() == (char) 183);
			assert (Arrays.equals(pack.q_GET(), new float[]{2.8619017E38F, -9.032025E37F, -1.8729804E37F, 3.9482532E37F}));
		});
		GroundControl.SET_ATTITUDE_TARGET p82 = CommunicationChannel.new_SET_ATTITUDE_TARGET();
		PH.setPack(p82);
		p82.target_system_SET((char) 51);
		p82.body_roll_rate_SET(1.6395553E38F);
		p82.target_component_SET((char) 185);
		p82.type_mask_SET((char) 183);
		p82.body_pitch_rate_SET(9.803544E37F);
		p82.q_SET(new float[]{2.8619017E38F, -9.032025E37F, -1.8729804E37F, 3.9482532E37F}, 0);
		p82.time_boot_ms_SET(2380490332L);
		p82.thrust_SET(-1.244467E38F);
		p82.body_yaw_rate_SET(-3.3275386E38F);
		CommunicationChannel.instance.send(p82);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_ATTITUDE_TARGET.add((src, ph, pack) ->
		{
			assert (pack.thrust_GET() == -2.5472664E38F);
			assert (pack.body_pitch_rate_GET() == -7.0039536E37F);
			assert (pack.type_mask_GET() == (char) 104);
			assert (Arrays.equals(pack.q_GET(), new float[]{3.0942702E38F, 6.80665E37F, -4.0087123E37F, 1.8619854E38F}));
			assert (pack.time_boot_ms_GET() == 1949322462L);
			assert (pack.body_yaw_rate_GET() == 1.4446919E38F);
			assert (pack.body_roll_rate_GET() == -1.1918134E38F);
		});
		GroundControl.ATTITUDE_TARGET p83 = CommunicationChannel.new_ATTITUDE_TARGET();
		PH.setPack(p83);
		p83.type_mask_SET((char) 104);
		p83.thrust_SET(-2.5472664E38F);
		p83.q_SET(new float[]{3.0942702E38F, 6.80665E37F, -4.0087123E37F, 1.8619854E38F}, 0);
		p83.body_roll_rate_SET(-1.1918134E38F);
		p83.time_boot_ms_SET(1949322462L);
		p83.body_yaw_rate_SET(1.4446919E38F);
		p83.body_pitch_rate_SET(-7.0039536E37F);
		CommunicationChannel.instance.send(p83);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_SET_POSITION_TARGET_LOCAL_NED.add((src, ph, pack) ->
		{
			assert (pack.y_GET() == -4.2223555E37F);
			assert (pack.yaw_rate_GET() == -2.0240328E38F);
			assert (pack.type_mask_GET() == (char) 28164);
			assert (pack.time_boot_ms_GET() == 2072476089L);
			assert (pack.z_GET() == 8.463442E37F);
			assert (pack.afy_GET() == -2.1832687E38F);
			assert (pack.target_system_GET() == (char) 181);
			assert (pack.vy_GET() == -1.6953812E38F);
			assert (pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_INT);
			assert (pack.afx_GET() == -2.2180519E38F);
			assert (pack.afz_GET() == -3.1537313E38F);
			assert (pack.vz_GET() == -1.0867869E38F);
			assert (pack.yaw_GET() == -9.013003E37F);
			assert (pack.vx_GET() == 1.9997726E38F);
			assert (pack.x_GET() == 3.1153931E38F);
			assert (pack.target_component_GET() == (char) 230);
		});
		GroundControl.SET_POSITION_TARGET_LOCAL_NED p84 = CommunicationChannel.new_SET_POSITION_TARGET_LOCAL_NED();
		PH.setPack(p84);
		p84.yaw_SET(-9.013003E37F);
		p84.target_system_SET((char) 181);
		p84.y_SET(-4.2223555E37F);
		p84.time_boot_ms_SET(2072476089L);
		p84.afz_SET(-3.1537313E38F);
		p84.vz_SET(-1.0867869E38F);
		p84.type_mask_SET((char) 28164);
		p84.vy_SET(-1.6953812E38F);
		p84.afx_SET(-2.2180519E38F);
		p84.yaw_rate_SET(-2.0240328E38F);
		p84.vx_SET(1.9997726E38F);
		p84.z_SET(8.463442E37F);
		p84.afy_SET(-2.1832687E38F);
		p84.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_INT);
		p84.target_component_SET((char) 230);
		p84.x_SET(3.1153931E38F);
		CommunicationChannel.instance.send(p84);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_SET_POSITION_TARGET_GLOBAL_INT.add((src, ph, pack) ->
		{
			assert (pack.vz_GET() == -6.1903775E37F);
			assert (pack.vy_GET() == -1.8176637E38F);
			assert (pack.yaw_rate_GET() == 2.1068385E38F);
			assert (pack.target_system_GET() == (char) 101);
			assert (pack.time_boot_ms_GET() == 3470428416L);
			assert (pack.lon_int_GET() == 344742180);
			assert (pack.afz_GET() == -3.1123704E38F);
			assert (pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_LOCAL_NED);
			assert (pack.yaw_GET() == 3.1205126E38F);
			assert (pack.afy_GET() == 1.4142093E38F);
			assert (pack.vx_GET() == 2.0404305E38F);
			assert (pack.target_component_GET() == (char) 171);
			assert (pack.alt_GET() == 5.638005E37F);
			assert (pack.type_mask_GET() == (char) 59011);
			assert (pack.afx_GET() == 1.9653318E38F);
			assert (pack.lat_int_GET() == 1844991725);
		});
		GroundControl.SET_POSITION_TARGET_GLOBAL_INT p86 = CommunicationChannel.new_SET_POSITION_TARGET_GLOBAL_INT();
		PH.setPack(p86);
		p86.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_NED);
		p86.vz_SET(-6.1903775E37F);
		p86.time_boot_ms_SET(3470428416L);
		p86.alt_SET(5.638005E37F);
		p86.target_component_SET((char) 171);
		p86.yaw_SET(3.1205126E38F);
		p86.target_system_SET((char) 101);
		p86.vy_SET(-1.8176637E38F);
		p86.afz_SET(-3.1123704E38F);
		p86.lat_int_SET(1844991725);
		p86.afy_SET(1.4142093E38F);
		p86.afx_SET(1.9653318E38F);
		p86.vx_SET(2.0404305E38F);
		p86.yaw_rate_SET(2.1068385E38F);
		p86.lon_int_SET(344742180);
		p86.type_mask_SET((char) 59011);
		CommunicationChannel.instance.send(p86);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_POSITION_TARGET_GLOBAL_INT.add((src, ph, pack) ->
		{
			assert (pack.vx_GET() == 2.0868034E38F);
			assert (pack.afy_GET() == 1.7975884E38F);
			assert (pack.vy_GET() == -3.3724654E38F);
			assert (pack.afz_GET() == -3.7509796E37F);
			assert (pack.yaw_rate_GET() == 1.4591686E37F);
			assert (pack.lon_int_GET() == 1208030990);
			assert (pack.type_mask_GET() == (char) 34512);
			assert (pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_MISSION);
			assert (pack.afx_GET() == -4.7177908E36F);
			assert (pack.alt_GET() == 2.876343E38F);
			assert (pack.yaw_GET() == -2.2030515E37F);
			assert (pack.lat_int_GET() == 1305135521);
			assert (pack.time_boot_ms_GET() == 3934551183L);
			assert (pack.vz_GET() == -9.201817E37F);
		});
		GroundControl.POSITION_TARGET_GLOBAL_INT p87 = CommunicationChannel.new_POSITION_TARGET_GLOBAL_INT();
		PH.setPack(p87);
		p87.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_MISSION);
		p87.alt_SET(2.876343E38F);
		p87.vy_SET(-3.3724654E38F);
		p87.afz_SET(-3.7509796E37F);
		p87.time_boot_ms_SET(3934551183L);
		p87.afy_SET(1.7975884E38F);
		p87.vx_SET(2.0868034E38F);
		p87.lat_int_SET(1305135521);
		p87.yaw_rate_SET(1.4591686E37F);
		p87.lon_int_SET(1208030990);
		p87.vz_SET(-9.201817E37F);
		p87.yaw_SET(-2.2030515E37F);
		p87.type_mask_SET((char) 34512);
		p87.afx_SET(-4.7177908E36F);
		CommunicationChannel.instance.send(p87);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET.add((src, ph, pack) ->
		{
			assert (pack.pitch_GET() == -7.8037114E37F);
			assert (pack.y_GET() == -7.9104886E37F);
			assert (pack.yaw_GET() == -1.2304477E38F);
			assert (pack.time_boot_ms_GET() == 1941113648L);
			assert (pack.x_GET() == -2.7092788E38F);
			assert (pack.roll_GET() == -1.6344625E38F);
			assert (pack.z_GET() == 2.6939161E38F);
		});
		GroundControl.LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET p89 = CommunicationChannel.new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
		PH.setPack(p89);
		p89.roll_SET(-1.6344625E38F);
		p89.z_SET(2.6939161E38F);
		p89.y_SET(-7.9104886E37F);
		p89.yaw_SET(-1.2304477E38F);
		p89.pitch_SET(-7.8037114E37F);
		p89.time_boot_ms_SET(1941113648L);
		p89.x_SET(-2.7092788E38F);
		CommunicationChannel.instance.send(p89);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_HIL_STATE.add((src, ph, pack) ->
		{
			assert (pack.lon_GET() == 143976801);
			assert (pack.time_usec_GET() == 3402736959500549187L);
			assert (pack.rollspeed_GET() == 1.4529919E38F);
			assert (pack.roll_GET() == -6.798254E37F);
			assert (pack.zacc_GET() == (short) 29081);
			assert (pack.pitchspeed_GET() == 3.2182215E38F);
			assert (pack.vx_GET() == (short) 27450);
			assert (pack.yacc_GET() == (short) 5520);
			assert (pack.vy_GET() == (short) -22536);
			assert (pack.vz_GET() == (short) -5197);
			assert (pack.lat_GET() == -348353565);
			assert (pack.pitch_GET() == -3.7615146E37F);
			assert (pack.yawspeed_GET() == -1.7161026E38F);
			assert (pack.yaw_GET() == -2.2419756E38F);
			assert (pack.xacc_GET() == (short) 233);
			assert (pack.alt_GET() == 79712768);
		});
		GroundControl.HIL_STATE p90 = CommunicationChannel.new_HIL_STATE();
		PH.setPack(p90);
		p90.yawspeed_SET(-1.7161026E38F);
		p90.vx_SET((short) 27450);
		p90.alt_SET(79712768);
		p90.vz_SET((short) -5197);
		p90.lat_SET(-348353565);
		p90.vy_SET((short) -22536);
		p90.zacc_SET((short) 29081);
		p90.yacc_SET((short) 5520);
		p90.yaw_SET(-2.2419756E38F);
		p90.pitchspeed_SET(3.2182215E38F);
		p90.lon_SET(143976801);
		p90.time_usec_SET(3402736959500549187L);
		p90.pitch_SET(-3.7615146E37F);
		p90.xacc_SET((short) 233);
		p90.rollspeed_SET(1.4529919E38F);
		p90.roll_SET(-6.798254E37F);
		CommunicationChannel.instance.send(p90);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_HIL_CONTROLS.add((src, ph, pack) ->
		{
			assert (pack.nav_mode_GET() == (char) 59);
			assert (pack.aux2_GET() == 1.6459594E38F);
			assert (pack.time_usec_GET() == 3689565220546520651L);
			assert (pack.throttle_GET() == -3.0117368E38F);
			assert (pack.mode_GET() == MAV_MODE.MAV_MODE_MANUAL_DISARMED);
			assert (pack.roll_ailerons_GET() == 6.4242514E37F);
			assert (pack.yaw_rudder_GET() == 1.1631089E38F);
			assert (pack.aux1_GET() == -3.049357E38F);
			assert (pack.aux3_GET() == 2.676556E38F);
			assert (pack.aux4_GET() == 1.794466E38F);
			assert (pack.pitch_elevator_GET() == -8.3865137E37F);
		});
		GroundControl.HIL_CONTROLS p91 = CommunicationChannel.new_HIL_CONTROLS();
		PH.setPack(p91);
		p91.nav_mode_SET((char) 59);
		p91.aux2_SET(1.6459594E38F);
		p91.roll_ailerons_SET(6.4242514E37F);
		p91.aux4_SET(1.794466E38F);
		p91.pitch_elevator_SET(-8.3865137E37F);
		p91.mode_SET(MAV_MODE.MAV_MODE_MANUAL_DISARMED);
		p91.throttle_SET(-3.0117368E38F);
		p91.yaw_rudder_SET(1.1631089E38F);
		p91.aux3_SET(2.676556E38F);
		p91.time_usec_SET(3689565220546520651L);
		p91.aux1_SET(-3.049357E38F);
		CommunicationChannel.instance.send(p91);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_HIL_RC_INPUTS_RAW.add((src, ph, pack) ->
		{
			assert (pack.chan1_raw_GET() == (char) 20492);
			assert (pack.chan9_raw_GET() == (char) 27433);
			assert (pack.chan6_raw_GET() == (char) 47631);
			assert (pack.chan3_raw_GET() == (char) 34058);
			assert (pack.chan7_raw_GET() == (char) 34362);
			assert (pack.time_usec_GET() == 7225550309760244724L);
			assert (pack.chan11_raw_GET() == (char) 57538);
			assert (pack.chan10_raw_GET() == (char) 17427);
			assert (pack.chan12_raw_GET() == (char) 39814);
			assert (pack.rssi_GET() == (char) 208);
			assert (pack.chan2_raw_GET() == (char) 32350);
			assert (pack.chan4_raw_GET() == (char) 1437);
			assert (pack.chan8_raw_GET() == (char) 14572);
			assert (pack.chan5_raw_GET() == (char) 9569);
		});
		GroundControl.HIL_RC_INPUTS_RAW p92 = CommunicationChannel.new_HIL_RC_INPUTS_RAW();
		PH.setPack(p92);
		p92.chan7_raw_SET((char) 34362);
		p92.chan1_raw_SET((char) 20492);
		p92.chan2_raw_SET((char) 32350);
		p92.chan11_raw_SET((char) 57538);
		p92.chan8_raw_SET((char) 14572);
		p92.chan5_raw_SET((char) 9569);
		p92.chan3_raw_SET((char) 34058);
		p92.chan4_raw_SET((char) 1437);
		p92.time_usec_SET(7225550309760244724L);
		p92.chan10_raw_SET((char) 17427);
		p92.chan9_raw_SET((char) 27433);
		p92.rssi_SET((char) 208);
		p92.chan12_raw_SET((char) 39814);
		p92.chan6_raw_SET((char) 47631);
		CommunicationChannel.instance.send(p92);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_HIL_ACTUATOR_CONTROLS.add((src, ph, pack) ->
		{
			assert (pack.time_usec_GET() == 1243776571781381478L);
			assert (Arrays.equals(pack.controls_GET(), new float[]{7.4887316E37F, 3.3539279E38F, 2.1308079E38F, -2.5655114E38F, -2.6159828E38F, 1.0602591E38F, 1.4896626E38F, 2.243661E38F, 3.3485946E38F, 2.4730267E38F, -1.1595551E38F, -3.0615034E38F, 2.164982E38F, 2.5075429E38F, 1.4716538E38F, -2.8546579E38F}));
			assert (pack.mode_GET() == MAV_MODE.MAV_MODE_GUIDED_DISARMED);
			assert (pack.flags_GET() == 2114090016665618715L);
		});
		GroundControl.HIL_ACTUATOR_CONTROLS p93 = CommunicationChannel.new_HIL_ACTUATOR_CONTROLS();
		PH.setPack(p93);
		p93.mode_SET(MAV_MODE.MAV_MODE_GUIDED_DISARMED);
		p93.flags_SET(2114090016665618715L);
		p93.controls_SET(new float[]{7.4887316E37F, 3.3539279E38F, 2.1308079E38F, -2.5655114E38F, -2.6159828E38F, 1.0602591E38F, 1.4896626E38F, 2.243661E38F, 3.3485946E38F, 2.4730267E38F, -1.1595551E38F, -3.0615034E38F, 2.164982E38F, 2.5075429E38F, 1.4716538E38F, -2.8546579E38F}, 0);
		p93.time_usec_SET(1243776571781381478L);
		CommunicationChannel.instance.send(p93);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_OPTICAL_FLOW.add((src, ph, pack) ->
		{
			assert (pack.ground_distance_GET() == -1.8865685E38F);
			assert (pack.flow_rate_y_TRY(ph) == 2.0105827E38F);
			assert (pack.sensor_id_GET() == (char) 39);
			assert (pack.flow_comp_m_y_GET() == -2.3831136E38F);
			assert (pack.flow_x_GET() == (short) 24239);
			assert (pack.flow_comp_m_x_GET() == 1.88397E38F);
			assert (pack.flow_rate_x_TRY(ph) == -1.0143909E38F);
			assert (pack.time_usec_GET() == 4565658304522965716L);
			assert (pack.quality_GET() == (char) 104);
			assert (pack.flow_y_GET() == (short) -22203);
		});
		GroundControl.OPTICAL_FLOW p100 = CommunicationChannel.new_OPTICAL_FLOW();
		PH.setPack(p100);
		p100.ground_distance_SET(-1.8865685E38F);
		p100.flow_comp_m_y_SET(-2.3831136E38F);
		p100.flow_x_SET((short) 24239);
		p100.flow_rate_x_SET(-1.0143909E38F, PH);
		p100.flow_rate_y_SET(2.0105827E38F, PH);
		p100.time_usec_SET(4565658304522965716L);
		p100.flow_y_SET((short) -22203);
		p100.flow_comp_m_x_SET(1.88397E38F);
		p100.sensor_id_SET((char) 39);
		p100.quality_SET((char) 104);
		CommunicationChannel.instance.send(p100);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_GLOBAL_VISION_POSITION_ESTIMATE.add((src, ph, pack) ->
		{
			assert (pack.x_GET() == 2.9521355E38F);
			assert (pack.usec_GET() == 3637040078105643847L);
			assert (pack.y_GET() == -3.3014786E37F);
			assert (pack.pitch_GET() == 2.1045277E37F);
			assert (pack.z_GET() == -6.2867083E37F);
			assert (pack.roll_GET() == 2.7121409E38F);
			assert (pack.yaw_GET() == 3.14716E38F);
		});
		GroundControl.GLOBAL_VISION_POSITION_ESTIMATE p101 = CommunicationChannel.new_GLOBAL_VISION_POSITION_ESTIMATE();
		PH.setPack(p101);
		p101.roll_SET(2.7121409E38F);
		p101.yaw_SET(3.14716E38F);
		p101.y_SET(-3.3014786E37F);
		p101.x_SET(2.9521355E38F);
		p101.usec_SET(3637040078105643847L);
		p101.z_SET(-6.2867083E37F);
		p101.pitch_SET(2.1045277E37F);
		CommunicationChannel.instance.send(p101);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_VISION_POSITION_ESTIMATE.add((src, ph, pack) ->
		{
			assert (pack.usec_GET() == 3521736520911800528L);
			assert (pack.y_GET() == -1.7845113E38F);
			assert (pack.z_GET() == -2.3673343E38F);
			assert (pack.roll_GET() == -2.132008E38F);
			assert (pack.yaw_GET() == 4.2580287E37F);
			assert (pack.x_GET() == -3.7927722E36F);
			assert (pack.pitch_GET() == 2.621952E38F);
		});
		GroundControl.VISION_POSITION_ESTIMATE p102 = CommunicationChannel.new_VISION_POSITION_ESTIMATE();
		PH.setPack(p102);
		p102.roll_SET(-2.132008E38F);
		p102.x_SET(-3.7927722E36F);
		p102.yaw_SET(4.2580287E37F);
		p102.y_SET(-1.7845113E38F);
		p102.usec_SET(3521736520911800528L);
		p102.z_SET(-2.3673343E38F);
		p102.pitch_SET(2.621952E38F);
		CommunicationChannel.instance.send(p102);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_VISION_SPEED_ESTIMATE.add((src, ph, pack) ->
		{
			assert (pack.usec_GET() == 2858843553917594391L);
			assert (pack.y_GET() == 1.8615506E38F);
			assert (pack.x_GET() == 2.3462835E38F);
			assert (pack.z_GET() == 3.149385E38F);
		});
		GroundControl.VISION_SPEED_ESTIMATE p103 = CommunicationChannel.new_VISION_SPEED_ESTIMATE();
		PH.setPack(p103);
		p103.z_SET(3.149385E38F);
		p103.usec_SET(2858843553917594391L);
		p103.x_SET(2.3462835E38F);
		p103.y_SET(1.8615506E38F);
		CommunicationChannel.instance.send(p103);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_VICON_POSITION_ESTIMATE.add((src, ph, pack) ->
		{
			assert (pack.y_GET() == 1.7221747E38F);
			assert (pack.roll_GET() == 2.2003482E38F);
			assert (pack.x_GET() == 1.2327477E38F);
			assert (pack.pitch_GET() == -1.7416458E38F);
			assert (pack.yaw_GET() == 3.248261E38F);
			assert (pack.z_GET() == -2.3104133E38F);
			assert (pack.usec_GET() == 8537937009833449713L);
		});
		GroundControl.VICON_POSITION_ESTIMATE p104 = CommunicationChannel.new_VICON_POSITION_ESTIMATE();
		PH.setPack(p104);
		p104.z_SET(-2.3104133E38F);
		p104.usec_SET(8537937009833449713L);
		p104.yaw_SET(3.248261E38F);
		p104.roll_SET(2.2003482E38F);
		p104.y_SET(1.7221747E38F);
		p104.pitch_SET(-1.7416458E38F);
		p104.x_SET(1.2327477E38F);
		CommunicationChannel.instance.send(p104);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_HIGHRES_IMU.add((src, ph, pack) ->
		{
			assert (pack.temperature_GET() == -1.5665056E38F);
			assert (pack.diff_pressure_GET() == -2.7237341E38F);
			assert (pack.fields_updated_GET() == (char) 48230);
			assert (pack.ymag_GET() == -2.093308E37F);
			assert (pack.ygyro_GET() == 3.272011E38F);
			assert (pack.pressure_alt_GET() == -4.8509926E37F);
			assert (pack.zgyro_GET() == 2.0787659E38F);
			assert (pack.yacc_GET() == 2.3827992E38F);
			assert (pack.zacc_GET() == 2.398132E38F);
			assert (pack.xmag_GET() == -1.9938248E38F);
			assert (pack.xgyro_GET() == 8.579466E37F);
			assert (pack.abs_pressure_GET() == 2.3616909E38F);
			assert (pack.xacc_GET() == -4.8815014E37F);
			assert (pack.time_usec_GET() == 1586654108044742722L);
			assert (pack.zmag_GET() == -2.8197906E38F);
		});
		GroundControl.HIGHRES_IMU p105 = CommunicationChannel.new_HIGHRES_IMU();
		PH.setPack(p105);
		p105.temperature_SET(-1.5665056E38F);
		p105.xacc_SET(-4.8815014E37F);
		p105.xgyro_SET(8.579466E37F);
		p105.yacc_SET(2.3827992E38F);
		p105.xmag_SET(-1.9938248E38F);
		p105.time_usec_SET(1586654108044742722L);
		p105.ygyro_SET(3.272011E38F);
		p105.fields_updated_SET((char) 48230);
		p105.zmag_SET(-2.8197906E38F);
		p105.pressure_alt_SET(-4.8509926E37F);
		p105.abs_pressure_SET(2.3616909E38F);
		p105.diff_pressure_SET(-2.7237341E38F);
		p105.ymag_SET(-2.093308E37F);
		p105.zacc_SET(2.398132E38F);
		p105.zgyro_SET(2.0787659E38F);
		CommunicationChannel.instance.send(p105);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_OPTICAL_FLOW_RAD.add((src, ph, pack) ->
		{
			assert (pack.time_usec_GET() == 3840993380870737884L);
			assert (pack.integrated_zgyro_GET() == 2.1240549E38F);
			assert (pack.sensor_id_GET() == (char) 79);
			assert (pack.distance_GET() == 2.0074787E38F);
			assert (pack.temperature_GET() == (short) 15210);
			assert (pack.integrated_xgyro_GET() == 9.677314E37F);
			assert (pack.quality_GET() == (char) 249);
			assert (pack.integrated_y_GET() == -1.5897955E38F);
			assert (pack.integrated_x_GET() == -1.821567E38F);
			assert (pack.integrated_ygyro_GET() == -1.7176704E38F);
			assert (pack.integration_time_us_GET() == 1958050335L);
			assert (pack.time_delta_distance_us_GET() == 1293614538L);
		});
		GroundControl.OPTICAL_FLOW_RAD p106 = CommunicationChannel.new_OPTICAL_FLOW_RAD();
		PH.setPack(p106);
		p106.time_usec_SET(3840993380870737884L);
		p106.distance_SET(2.0074787E38F);
		p106.integrated_ygyro_SET(-1.7176704E38F);
		p106.integrated_xgyro_SET(9.677314E37F);
		p106.integrated_x_SET(-1.821567E38F);
		p106.temperature_SET((short) 15210);
		p106.integrated_zgyro_SET(2.1240549E38F);
		p106.sensor_id_SET((char) 79);
		p106.time_delta_distance_us_SET(1293614538L);
		p106.integrated_y_SET(-1.5897955E38F);
		p106.integration_time_us_SET(1958050335L);
		p106.quality_SET((char) 249);
		CommunicationChannel.instance.send(p106);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_HIL_SENSOR.add((src, ph, pack) ->
		{
			assert (pack.zmag_GET() == -2.1850187E38F);
			assert (pack.pressure_alt_GET() == -2.6332494E38F);
			assert (pack.zgyro_GET() == -3.3721964E38F);
			assert (pack.zacc_GET() == -1.3150255E38F);
			assert (pack.yacc_GET() == 1.380265E38F);
			assert (pack.xgyro_GET() == 4.991531E37F);
			assert (pack.diff_pressure_GET() == -1.3003026E38F);
			assert (pack.xmag_GET() == 1.754449E38F);
			assert (pack.temperature_GET() == 2.4930909E38F);
			assert (pack.ymag_GET() == 9.171916E37F);
			assert (pack.abs_pressure_GET() == 1.5648708E38F);
			assert (pack.fields_updated_GET() == 649828714L);
			assert (pack.xacc_GET() == -1.6657903E38F);
			assert (pack.ygyro_GET() == -2.8298551E38F);
			assert (pack.time_usec_GET() == 6931241781108759802L);
		});
		GroundControl.HIL_SENSOR p107 = CommunicationChannel.new_HIL_SENSOR();
		PH.setPack(p107);
		p107.ygyro_SET(-2.8298551E38F);
		p107.yacc_SET(1.380265E38F);
		p107.time_usec_SET(6931241781108759802L);
		p107.ymag_SET(9.171916E37F);
		p107.abs_pressure_SET(1.5648708E38F);
		p107.temperature_SET(2.4930909E38F);
		p107.fields_updated_SET(649828714L);
		p107.xgyro_SET(4.991531E37F);
		p107.zmag_SET(-2.1850187E38F);
		p107.zacc_SET(-1.3150255E38F);
		p107.pressure_alt_SET(-2.6332494E38F);
		p107.xacc_SET(-1.6657903E38F);
		p107.zgyro_SET(-3.3721964E38F);
		p107.diff_pressure_SET(-1.3003026E38F);
		p107.xmag_SET(1.754449E38F);
		CommunicationChannel.instance.send(p107);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_SIM_STATE.add((src, ph, pack) ->
		{
			assert (pack.lon_GET() == 1.2593746E38F);
			assert (pack.vd_GET() == 5.3119443E37F);
			assert (pack.zacc_GET() == 2.6075802E38F);
			assert (pack.xgyro_GET() == 1.5723529E38F);
			assert (pack.xacc_GET() == -2.6500522E38F);
			assert (pack.yaw_GET() == -1.9319172E38F);
			assert (pack.vn_GET() == 1.1374349E38F);
			assert (pack.q3_GET() == -1.5207701E37F);
			assert (pack.q4_GET() == 1.6537339E38F);
			assert (pack.lat_GET() == -2.11428E38F);
			assert (pack.q2_GET() == -1.4055312E38F);
			assert (pack.ve_GET() == -3.2143041E38F);
			assert (pack.ygyro_GET() == 6.3716404E36F);
			assert (pack.std_dev_vert_GET() == 2.8373763E38F);
			assert (pack.zgyro_GET() == 2.7003726E38F);
			assert (pack.std_dev_horz_GET() == -3.1606328E38F);
			assert (pack.roll_GET() == -1.7871594E38F);
			assert (pack.pitch_GET() == 3.2541435E38F);
			assert (pack.yacc_GET() == 3.1591435E38F);
			assert (pack.alt_GET() == -2.0585303E38F);
			assert (pack.q1_GET() == -5.2483914E37F);
		});
		GroundControl.SIM_STATE p108 = CommunicationChannel.new_SIM_STATE();
		PH.setPack(p108);
		p108.lon_SET(1.2593746E38F);
		p108.ve_SET(-3.2143041E38F);
		p108.xgyro_SET(1.5723529E38F);
		p108.alt_SET(-2.0585303E38F);
		p108.roll_SET(-1.7871594E38F);
		p108.pitch_SET(3.2541435E38F);
		p108.std_dev_horz_SET(-3.1606328E38F);
		p108.q4_SET(1.6537339E38F);
		p108.vn_SET(1.1374349E38F);
		p108.q1_SET(-5.2483914E37F);
		p108.xacc_SET(-2.6500522E38F);
		p108.vd_SET(5.3119443E37F);
		p108.yacc_SET(3.1591435E38F);
		p108.lat_SET(-2.11428E38F);
		p108.ygyro_SET(6.3716404E36F);
		p108.zacc_SET(2.6075802E38F);
		p108.zgyro_SET(2.7003726E38F);
		p108.q2_SET(-1.4055312E38F);
		p108.q3_SET(-1.5207701E37F);
		p108.yaw_SET(-1.9319172E38F);
		p108.std_dev_vert_SET(2.8373763E38F);
		CommunicationChannel.instance.send(p108);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_RADIO_STATUS.add((src, ph, pack) ->
		{
			assert (pack.remnoise_GET() == (char) 63);
			assert (pack.fixed__GET() == (char) 28500);
			assert (pack.rxerrors_GET() == (char) 44378);
			assert (pack.rssi_GET() == (char) 217);
			assert (pack.txbuf_GET() == (char) 171);
			assert (pack.remrssi_GET() == (char) 126);
			assert (pack.noise_GET() == (char) 232);
		});
		GroundControl.RADIO_STATUS p109 = CommunicationChannel.new_RADIO_STATUS();
		PH.setPack(p109);
		p109.remrssi_SET((char) 126);
		p109.rssi_SET((char) 217);
		p109.fixed__SET((char) 28500);
		p109.noise_SET((char) 232);
		p109.remnoise_SET((char) 63);
		p109.rxerrors_SET((char) 44378);
		p109.txbuf_SET((char) 171);
		CommunicationChannel.instance.send(p109);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_FILE_TRANSFER_PROTOCOL.add((src, ph, pack) ->
		{
			assert (pack.target_component_GET() == (char) 45);
			assert (pack.target_network_GET() == (char) 26);
			assert (pack.target_system_GET() == (char) 62);
			assert (Arrays.equals(pack.payload_GET(), new char[]{(char) 136, (char) 6, (char) 243, (char) 172, (char) 95, (char) 179, (char) 219, (char) 101, (char) 240, (char) 214, (char) 134, (char) 9, (char) 2, (char) 155, (char) 192, (char) 84, (char) 161, (char) 142, (char) 107, (char) 23, (char) 174, (char) 195, (char) 240, (char) 55, (char) 208, (char) 58, (char) 8, (char) 50, (char) 9, (char) 27, (char) 80, (char) 50, (char) 105, (char) 140, (char) 48, (char) 50, (char) 111, (char) 224, (char) 34, (char) 56, (char) 61, (char) 118, (char) 250, (char) 45, (char) 134, (char) 42, (char) 123, (char) 252, (char) 242, (char) 60, (char) 110, (char) 100, (char) 123, (char) 227, (char) 195, (char) 207, (char) 184, (char) 222, (char) 77, (char) 81, (char) 204, (char) 224, (char) 80, (char) 88, (char) 192, (char) 227, (char) 106, (char) 246, (char) 236, (char) 41, (char) 255, (char) 177, (char) 30, (char) 243, (char) 89, (char) 100, (char) 45, (char) 138, (char) 145, (char) 125, (char) 237, (char) 235, (char) 179, (char) 159, (char) 153, (char) 171, (char) 230, (char) 67, (char) 202, (char) 53, (char) 126, (char) 239, (char) 105, (char) 36, (char) 255, (char) 215, (char) 195, (char) 58, (char) 243, (char) 0, (char) 178, (char) 217, (char) 11, (char) 210, (char) 74, (char) 255, (char) 176, (char) 108, (char) 166, (char) 174, (char) 246, (char) 27, (char) 54, (char) 47, (char) 240, (char) 65, (char) 230, (char) 10, (char) 231, (char) 143, (char) 188, (char) 144, (char) 234, (char) 114, (char) 188, (char) 209, (char) 215, (char) 95, (char) 185, (char) 38, (char) 149, (char) 126, (char) 250, (char) 193, (char) 4, (char) 151, (char) 150, (char) 5, (char) 92, (char) 3, (char) 13, (char) 67, (char) 74, (char) 130, (char) 252, (char) 226, (char) 182, (char) 21, (char) 18, (char) 100, (char) 172, (char) 211, (char) 107, (char) 195, (char) 203, (char) 48, (char) 109, (char) 246, (char) 59, (char) 76, (char) 90, (char) 239, (char) 47, (char) 68, (char) 65, (char) 208, (char) 191, (char) 20, (char) 121, (char) 103, (char) 79, (char) 60, (char) 44, (char) 68, (char) 103, (char) 100, (char) 207, (char) 145, (char) 160, (char) 123, (char) 237, (char) 39, (char) 178, (char) 101, (char) 34, (char) 141, (char) 189, (char) 16, (char) 195, (char) 111, (char) 122, (char) 80, (char) 173, (char) 116, (char) 35, (char) 210, (char) 61, (char) 177, (char) 64, (char) 166, (char) 185, (char) 194, (char) 90, (char) 217, (char) 120, (char) 13, (char) 125, (char) 188, (char) 85, (char) 198, (char) 85, (char) 255, (char) 53, (char) 146, (char) 34, (char) 54, (char) 204, (char) 61, (char) 11, (char) 22, (char) 198, (char) 156, (char) 20, (char) 249, (char) 152, (char) 164, (char) 70, (char) 28, (char) 244, (char) 248, (char) 173, (char) 237, (char) 198, (char) 31, (char) 53, (char) 3, (char) 94, (char) 173, (char) 8, (char) 7, (char) 166, (char) 235, (char) 165, (char) 184, (char) 102, (char) 184, (char) 111, (char) 102, (char) 10, (char) 118, (char) 170}));
		});
		GroundControl.FILE_TRANSFER_PROTOCOL p110 = CommunicationChannel.new_FILE_TRANSFER_PROTOCOL();
		PH.setPack(p110);
		p110.payload_SET(new char[]{(char) 136, (char) 6, (char) 243, (char) 172, (char) 95, (char) 179, (char) 219, (char) 101, (char) 240, (char) 214, (char) 134, (char) 9, (char) 2, (char) 155, (char) 192, (char) 84, (char) 161, (char) 142, (char) 107, (char) 23, (char) 174, (char) 195, (char) 240, (char) 55, (char) 208, (char) 58, (char) 8, (char) 50, (char) 9, (char) 27, (char) 80, (char) 50, (char) 105, (char) 140, (char) 48, (char) 50, (char) 111, (char) 224, (char) 34, (char) 56, (char) 61, (char) 118, (char) 250, (char) 45, (char) 134, (char) 42, (char) 123, (char) 252, (char) 242, (char) 60, (char) 110, (char) 100, (char) 123, (char) 227, (char) 195, (char) 207, (char) 184, (char) 222, (char) 77, (char) 81, (char) 204, (char) 224, (char) 80, (char) 88, (char) 192, (char) 227, (char) 106, (char) 246, (char) 236, (char) 41, (char) 255, (char) 177, (char) 30, (char) 243, (char) 89, (char) 100, (char) 45, (char) 138, (char) 145, (char) 125, (char) 237, (char) 235, (char) 179, (char) 159, (char) 153, (char) 171, (char) 230, (char) 67, (char) 202, (char) 53, (char) 126, (char) 239, (char) 105, (char) 36, (char) 255, (char) 215, (char) 195, (char) 58, (char) 243, (char) 0, (char) 178, (char) 217, (char) 11, (char) 210, (char) 74, (char) 255, (char) 176, (char) 108, (char) 166, (char) 174, (char) 246, (char) 27, (char) 54, (char) 47, (char) 240, (char) 65, (char) 230, (char) 10, (char) 231, (char) 143, (char) 188, (char) 144, (char) 234, (char) 114, (char) 188, (char) 209, (char) 215, (char) 95, (char) 185, (char) 38, (char) 149, (char) 126, (char) 250, (char) 193, (char) 4, (char) 151, (char) 150, (char) 5, (char) 92, (char) 3, (char) 13, (char) 67, (char) 74, (char) 130, (char) 252, (char) 226, (char) 182, (char) 21, (char) 18, (char) 100, (char) 172, (char) 211, (char) 107, (char) 195, (char) 203, (char) 48, (char) 109, (char) 246, (char) 59, (char) 76, (char) 90, (char) 239, (char) 47, (char) 68, (char) 65, (char) 208, (char) 191, (char) 20, (char) 121, (char) 103, (char) 79, (char) 60, (char) 44, (char) 68, (char) 103, (char) 100, (char) 207, (char) 145, (char) 160, (char) 123, (char) 237, (char) 39, (char) 178, (char) 101, (char) 34, (char) 141, (char) 189, (char) 16, (char) 195, (char) 111, (char) 122, (char) 80, (char) 173, (char) 116, (char) 35, (char) 210, (char) 61, (char) 177, (char) 64, (char) 166, (char) 185, (char) 194, (char) 90, (char) 217, (char) 120, (char) 13, (char) 125, (char) 188, (char) 85, (char) 198, (char) 85, (char) 255, (char) 53, (char) 146, (char) 34, (char) 54, (char) 204, (char) 61, (char) 11, (char) 22, (char) 198, (char) 156, (char) 20, (char) 249, (char) 152, (char) 164, (char) 70, (char) 28, (char) 244, (char) 248, (char) 173, (char) 237, (char) 198, (char) 31, (char) 53, (char) 3, (char) 94, (char) 173, (char) 8, (char) 7, (char) 166, (char) 235, (char) 165, (char) 184, (char) 102, (char) 184, (char) 111, (char) 102, (char) 10, (char) 118, (char) 170}, 0);
		p110.target_component_SET((char) 45);
		p110.target_system_SET((char) 62);
		p110.target_network_SET((char) 26);
		CommunicationChannel.instance.send(p110);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_TIMESYNC.add((src, ph, pack) ->
		{
			assert (pack.ts1_GET() == -6575828305294666868L);
			assert (pack.tc1_GET() == -7674497442803704188L);
		});
		GroundControl.TIMESYNC p111 = CommunicationChannel.new_TIMESYNC();
		PH.setPack(p111);
		p111.tc1_SET(-7674497442803704188L);
		p111.ts1_SET(-6575828305294666868L);
		CommunicationChannel.instance.send(p111);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_CAMERA_TRIGGER.add((src, ph, pack) ->
		{
			assert (pack.time_usec_GET() == 5898010865576336183L);
			assert (pack.seq_GET() == 959422166L);
		});
		GroundControl.CAMERA_TRIGGER p112 = CommunicationChannel.new_CAMERA_TRIGGER();
		PH.setPack(p112);
		p112.seq_SET(959422166L);
		p112.time_usec_SET(5898010865576336183L);
		CommunicationChannel.instance.send(p112);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_HIL_GPS.add((src, ph, pack) ->
		{
			assert (pack.lon_GET() == 1011052203);
			assert (pack.vel_GET() == (char) 49264);
			assert (pack.eph_GET() == (char) 25969);
			assert (pack.alt_GET() == 317005473);
			assert (pack.time_usec_GET() == 1484081044703734567L);
			assert (pack.satellites_visible_GET() == (char) 42);
			assert (pack.epv_GET() == (char) 20525);
			assert (pack.vd_GET() == (short) -27080);
			assert (pack.lat_GET() == 293124107);
			assert (pack.cog_GET() == (char) 5153);
			assert (pack.vn_GET() == (short) 1756);
			assert (pack.fix_type_GET() == (char) 122);
			assert (pack.ve_GET() == (short) 26268);
		});
		GroundControl.HIL_GPS p113 = CommunicationChannel.new_HIL_GPS();
		PH.setPack(p113);
		p113.time_usec_SET(1484081044703734567L);
		p113.cog_SET((char) 5153);
		p113.lat_SET(293124107);
		p113.satellites_visible_SET((char) 42);
		p113.alt_SET(317005473);
		p113.eph_SET((char) 25969);
		p113.epv_SET((char) 20525);
		p113.vel_SET((char) 49264);
		p113.lon_SET(1011052203);
		p113.ve_SET((short) 26268);
		p113.vd_SET((short) -27080);
		p113.vn_SET((short) 1756);
		p113.fix_type_SET((char) 122);
		CommunicationChannel.instance.send(p113);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_HIL_OPTICAL_FLOW.add((src, ph, pack) ->
		{
			assert (pack.time_delta_distance_us_GET() == 3321623829L);
			assert (pack.sensor_id_GET() == (char) 52);
			assert (pack.integrated_x_GET() == -1.3459822E37F);
			assert (pack.integrated_zgyro_GET() == -1.3096168E38F);
			assert (pack.integration_time_us_GET() == 154337429L);
			assert (pack.integrated_y_GET() == 1.9554884E38F);
			assert (pack.integrated_xgyro_GET() == -1.8621242E38F);
			assert (pack.integrated_ygyro_GET() == -2.9854486E38F);
			assert (pack.distance_GET() == 3.2294162E38F);
			assert (pack.time_usec_GET() == 129320899702109914L);
			assert (pack.quality_GET() == (char) 211);
			assert (pack.temperature_GET() == (short) -29901);
		});
		GroundControl.HIL_OPTICAL_FLOW p114 = CommunicationChannel.new_HIL_OPTICAL_FLOW();
		PH.setPack(p114);
		p114.integrated_zgyro_SET(-1.3096168E38F);
		p114.quality_SET((char) 211);
		p114.integrated_y_SET(1.9554884E38F);
		p114.time_delta_distance_us_SET(3321623829L);
		p114.distance_SET(3.2294162E38F);
		p114.sensor_id_SET((char) 52);
		p114.integrated_ygyro_SET(-2.9854486E38F);
		p114.integrated_x_SET(-1.3459822E37F);
		p114.integration_time_us_SET(154337429L);
		p114.temperature_SET((short) -29901);
		p114.time_usec_SET(129320899702109914L);
		p114.integrated_xgyro_SET(-1.8621242E38F);
		CommunicationChannel.instance.send(p114);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_HIL_STATE_QUATERNION.add((src, ph, pack) ->
		{
			assert (pack.pitchspeed_GET() == 1.8226238E38F);
			assert (pack.vx_GET() == (short) 29614);
			assert (pack.vz_GET() == (short) -7501);
			assert (pack.true_airspeed_GET() == (char) 49234);
			assert (pack.vy_GET() == (short) -10411);
			assert (pack.zacc_GET() == (short) 15399);
			assert (pack.time_usec_GET() == 2072091423019461966L);
			assert (pack.yawspeed_GET() == -5.6663105E37F);
			assert (pack.yacc_GET() == (short) -24974);
			assert (pack.alt_GET() == 398080626);
			assert (pack.lon_GET() == -655023670);
			assert (pack.rollspeed_GET() == -6.7402417E37F);
			assert (pack.ind_airspeed_GET() == (char) 24116);
			assert (pack.lat_GET() == 779149023);
			assert (Arrays.equals(pack.attitude_quaternion_GET(), new float[]{-2.703342E37F, 2.2755123E38F, 1.2799799E38F, -2.1574768E38F}));
			assert (pack.xacc_GET() == (short) -13934);
		});
		GroundControl.HIL_STATE_QUATERNION p115 = CommunicationChannel.new_HIL_STATE_QUATERNION();
		PH.setPack(p115);
		p115.rollspeed_SET(-6.7402417E37F);
		p115.lat_SET(779149023);
		p115.ind_airspeed_SET((char) 24116);
		p115.pitchspeed_SET(1.8226238E38F);
		p115.vx_SET((short) 29614);
		p115.yacc_SET((short) -24974);
		p115.lon_SET(-655023670);
		p115.xacc_SET((short) -13934);
		p115.zacc_SET((short) 15399);
		p115.attitude_quaternion_SET(new float[]{-2.703342E37F, 2.2755123E38F, 1.2799799E38F, -2.1574768E38F}, 0);
		p115.vy_SET((short) -10411);
		p115.true_airspeed_SET((char) 49234);
		p115.vz_SET((short) -7501);
		p115.yawspeed_SET(-5.6663105E37F);
		p115.alt_SET(398080626);
		p115.time_usec_SET(2072091423019461966L);
		CommunicationChannel.instance.send(p115);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_SCALED_IMU2.add((src, ph, pack) ->
		{
			assert (pack.zmag_GET() == (short) 29828);
			assert (pack.xmag_GET() == (short) 971);
			assert (pack.time_boot_ms_GET() == 1555825357L);
			assert (pack.yacc_GET() == (short) 2113);
			assert (pack.ygyro_GET() == (short) 27573);
			assert (pack.zgyro_GET() == (short) -31914);
			assert (pack.xgyro_GET() == (short) -13083);
			assert (pack.ymag_GET() == (short) 27233);
			assert (pack.zacc_GET() == (short) -11146);
			assert (pack.xacc_GET() == (short) -10769);
		});
		GroundControl.SCALED_IMU2 p116 = CommunicationChannel.new_SCALED_IMU2();
		PH.setPack(p116);
		p116.ymag_SET((short) 27233);
		p116.zgyro_SET((short) -31914);
		p116.yacc_SET((short) 2113);
		p116.zmag_SET((short) 29828);
		p116.time_boot_ms_SET(1555825357L);
		p116.ygyro_SET((short) 27573);
		p116.zacc_SET((short) -11146);
		p116.xgyro_SET((short) -13083);
		p116.xmag_SET((short) 971);
		p116.xacc_SET((short) -10769);
		CommunicationChannel.instance.send(p116);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_LOG_REQUEST_LIST.add((src, ph, pack) ->
		{
			assert (pack.target_component_GET() == (char) 51);
			assert (pack.end_GET() == (char) 793);
			assert (pack.start_GET() == (char) 46178);
			assert (pack.target_system_GET() == (char) 231);
		});
		GroundControl.LOG_REQUEST_LIST p117 = CommunicationChannel.new_LOG_REQUEST_LIST();
		PH.setPack(p117);
		p117.start_SET((char) 46178);
		p117.end_SET((char) 793);
		p117.target_system_SET((char) 231);
		p117.target_component_SET((char) 51);
		CommunicationChannel.instance.send(p117);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_LOG_ENTRY.add((src, ph, pack) ->
		{
			assert (pack.num_logs_GET() == (char) 15371);
			assert (pack.time_utc_GET() == 3283251228L);
			assert (pack.last_log_num_GET() == (char) 2809);
			assert (pack.size_GET() == 327325023L);
			assert (pack.id_GET() == (char) 61912);
		});
		GroundControl.LOG_ENTRY p118 = CommunicationChannel.new_LOG_ENTRY();
		PH.setPack(p118);
		p118.id_SET((char) 61912);
		p118.last_log_num_SET((char) 2809);
		p118.size_SET(327325023L);
		p118.time_utc_SET(3283251228L);
		p118.num_logs_SET((char) 15371);
		CommunicationChannel.instance.send(p118);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_LOG_REQUEST_DATA.add((src, ph, pack) ->
		{
			assert (pack.target_component_GET() == (char) 118);
			assert (pack.id_GET() == (char) 33876);
			assert (pack.ofs_GET() == 712172317L);
			assert (pack.count_GET() == 3841840837L);
			assert (pack.target_system_GET() == (char) 189);
		});
		GroundControl.LOG_REQUEST_DATA p119 = CommunicationChannel.new_LOG_REQUEST_DATA();
		PH.setPack(p119);
		p119.count_SET(3841840837L);
		p119.ofs_SET(712172317L);
		p119.target_component_SET((char) 118);
		p119.id_SET((char) 33876);
		p119.target_system_SET((char) 189);
		CommunicationChannel.instance.send(p119);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_LOG_DATA.add((src, ph, pack) ->
		{
			assert (pack.ofs_GET() == 2850540695L);
			assert (Arrays.equals(pack.data__GET(), new char[]{(char) 197, (char) 161, (char) 217, (char) 153, (char) 239, (char) 129, (char) 226, (char) 127, (char) 189, (char) 99, (char) 69, (char) 101, (char) 15, (char) 192, (char) 121, (char) 169, (char) 234, (char) 202, (char) 207, (char) 33, (char) 103, (char) 118, (char) 153, (char) 9, (char) 216, (char) 232, (char) 61, (char) 250, (char) 165, (char) 153, (char) 238, (char) 125, (char) 13, (char) 122, (char) 109, (char) 203, (char) 212, (char) 91, (char) 176, (char) 231, (char) 130, (char) 57, (char) 120, (char) 65, (char) 45, (char) 55, (char) 104, (char) 226, (char) 83, (char) 233, (char) 173, (char) 80, (char) 197, (char) 124, (char) 238, (char) 132, (char) 57, (char) 218, (char) 192, (char) 184, (char) 100, (char) 79, (char) 64, (char) 43, (char) 245, (char) 217, (char) 52, (char) 87, (char) 53, (char) 32, (char) 153, (char) 175, (char) 1, (char) 91, (char) 107, (char) 31, (char) 166, (char) 128, (char) 178, (char) 254, (char) 15, (char) 228, (char) 130, (char) 117, (char) 235, (char) 170, (char) 95, (char) 58, (char) 35, (char) 173}));
			assert (pack.id_GET() == (char) 47194);
			assert (pack.count_GET() == (char) 63);
		});
		GroundControl.LOG_DATA p120 = CommunicationChannel.new_LOG_DATA();
		PH.setPack(p120);
		p120.ofs_SET(2850540695L);
		p120.data__SET(new char[]{(char) 197, (char) 161, (char) 217, (char) 153, (char) 239, (char) 129, (char) 226, (char) 127, (char) 189, (char) 99, (char) 69, (char) 101, (char) 15, (char) 192, (char) 121, (char) 169, (char) 234, (char) 202, (char) 207, (char) 33, (char) 103, (char) 118, (char) 153, (char) 9, (char) 216, (char) 232, (char) 61, (char) 250, (char) 165, (char) 153, (char) 238, (char) 125, (char) 13, (char) 122, (char) 109, (char) 203, (char) 212, (char) 91, (char) 176, (char) 231, (char) 130, (char) 57, (char) 120, (char) 65, (char) 45, (char) 55, (char) 104, (char) 226, (char) 83, (char) 233, (char) 173, (char) 80, (char) 197, (char) 124, (char) 238, (char) 132, (char) 57, (char) 218, (char) 192, (char) 184, (char) 100, (char) 79, (char) 64, (char) 43, (char) 245, (char) 217, (char) 52, (char) 87, (char) 53, (char) 32, (char) 153, (char) 175, (char) 1, (char) 91, (char) 107, (char) 31, (char) 166, (char) 128, (char) 178, (char) 254, (char) 15, (char) 228, (char) 130, (char) 117, (char) 235, (char) 170, (char) 95, (char) 58, (char) 35, (char) 173}, 0);
		p120.id_SET((char) 47194);
		p120.count_SET((char) 63);
		CommunicationChannel.instance.send(p120);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_LOG_ERASE.add((src, ph, pack) ->
		{
			assert (pack.target_component_GET() == (char) 78);
			assert (pack.target_system_GET() == (char) 110);
		});
		GroundControl.LOG_ERASE p121 = CommunicationChannel.new_LOG_ERASE();
		PH.setPack(p121);
		p121.target_component_SET((char) 78);
		p121.target_system_SET((char) 110);
		CommunicationChannel.instance.send(p121);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_LOG_REQUEST_END.add((src, ph, pack) ->
		{
			assert (pack.target_component_GET() == (char) 83);
			assert (pack.target_system_GET() == (char) 11);
		});
		GroundControl.LOG_REQUEST_END p122 = CommunicationChannel.new_LOG_REQUEST_END();
		PH.setPack(p122);
		p122.target_system_SET((char) 11);
		p122.target_component_SET((char) 83);
		CommunicationChannel.instance.send(p122);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_GPS_INJECT_DATA.add((src, ph, pack) ->
		{
			assert (pack.target_system_GET() == (char) 163);
			assert (pack.target_component_GET() == (char) 54);
			assert (Arrays.equals(pack.data__GET(), new char[]{(char) 148, (char) 249, (char) 215, (char) 228, (char) 104, (char) 32, (char) 7, (char) 72, (char) 74, (char) 0, (char) 43, (char) 209, (char) 132, (char) 113, (char) 129, (char) 113, (char) 166, (char) 104, (char) 212, (char) 105, (char) 209, (char) 202, (char) 222, (char) 230, (char) 158, (char) 161, (char) 137, (char) 206, (char) 9, (char) 37, (char) 55, (char) 7, (char) 24, (char) 248, (char) 198, (char) 243, (char) 60, (char) 38, (char) 204, (char) 7, (char) 189, (char) 93, (char) 59, (char) 208, (char) 180, (char) 36, (char) 173, (char) 96, (char) 210, (char) 46, (char) 107, (char) 235, (char) 132, (char) 143, (char) 224, (char) 217, (char) 152, (char) 52, (char) 129, (char) 87, (char) 148, (char) 215, (char) 250, (char) 115, (char) 10, (char) 73, (char) 107, (char) 13, (char) 195, (char) 4, (char) 123, (char) 144, (char) 99, (char) 36, (char) 11, (char) 25, (char) 93, (char) 167, (char) 104, (char) 56, (char) 195, (char) 55, (char) 234, (char) 112, (char) 153, (char) 85, (char) 66, (char) 62, (char) 18, (char) 189, (char) 57, (char) 68, (char) 80, (char) 163, (char) 231, (char) 37, (char) 225, (char) 132, (char) 20, (char) 203, (char) 79, (char) 85, (char) 36, (char) 23, (char) 28, (char) 22, (char) 64, (char) 136, (char) 15, (char) 4}));
			assert (pack.len_GET() == (char) 196);
		});
		GroundControl.GPS_INJECT_DATA p123 = CommunicationChannel.new_GPS_INJECT_DATA();
		PH.setPack(p123);
		p123.data__SET(new char[]{(char) 148, (char) 249, (char) 215, (char) 228, (char) 104, (char) 32, (char) 7, (char) 72, (char) 74, (char) 0, (char) 43, (char) 209, (char) 132, (char) 113, (char) 129, (char) 113, (char) 166, (char) 104, (char) 212, (char) 105, (char) 209, (char) 202, (char) 222, (char) 230, (char) 158, (char) 161, (char) 137, (char) 206, (char) 9, (char) 37, (char) 55, (char) 7, (char) 24, (char) 248, (char) 198, (char) 243, (char) 60, (char) 38, (char) 204, (char) 7, (char) 189, (char) 93, (char) 59, (char) 208, (char) 180, (char) 36, (char) 173, (char) 96, (char) 210, (char) 46, (char) 107, (char) 235, (char) 132, (char) 143, (char) 224, (char) 217, (char) 152, (char) 52, (char) 129, (char) 87, (char) 148, (char) 215, (char) 250, (char) 115, (char) 10, (char) 73, (char) 107, (char) 13, (char) 195, (char) 4, (char) 123, (char) 144, (char) 99, (char) 36, (char) 11, (char) 25, (char) 93, (char) 167, (char) 104, (char) 56, (char) 195, (char) 55, (char) 234, (char) 112, (char) 153, (char) 85, (char) 66, (char) 62, (char) 18, (char) 189, (char) 57, (char) 68, (char) 80, (char) 163, (char) 231, (char) 37, (char) 225, (char) 132, (char) 20, (char) 203, (char) 79, (char) 85, (char) 36, (char) 23, (char) 28, (char) 22, (char) 64, (char) 136, (char) 15, (char) 4}, 0);
		p123.target_system_SET((char) 163);
		p123.target_component_SET((char) 54);
		p123.len_SET((char) 196);
		CommunicationChannel.instance.send(p123);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_GPS2_RAW.add((src, ph, pack) ->
		{
			assert (pack.lon_GET() == -601758723);
			assert (pack.dgps_age_GET() == 4119575603L);
			assert (pack.epv_GET() == (char) 3005);
			assert (pack.dgps_numch_GET() == (char) 127);
			assert (pack.cog_GET() == (char) 40421);
			assert (pack.lat_GET() == -1710704782);
			assert (pack.vel_GET() == (char) 29846);
			assert (pack.fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_RTK_FLOAT);
			assert (pack.alt_GET() == 2014869286);
			assert (pack.satellites_visible_GET() == (char) 113);
			assert (pack.time_usec_GET() == 9187263135387621108L);
			assert (pack.eph_GET() == (char) 65482);
		});
		GroundControl.GPS2_RAW p124 = CommunicationChannel.new_GPS2_RAW();
		PH.setPack(p124);
		p124.cog_SET((char) 40421);
		p124.fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_RTK_FLOAT);
		p124.dgps_numch_SET((char) 127);
		p124.vel_SET((char) 29846);
		p124.lon_SET(-601758723);
		p124.epv_SET((char) 3005);
		p124.dgps_age_SET(4119575603L);
		p124.alt_SET(2014869286);
		p124.eph_SET((char) 65482);
		p124.satellites_visible_SET((char) 113);
		p124.time_usec_SET(9187263135387621108L);
		p124.lat_SET(-1710704782);
		CommunicationChannel.instance.send(p124);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_POWER_STATUS.add((src, ph, pack) ->
		{
			assert (pack.Vcc_GET() == (char) 47670);
			assert (pack.Vservo_GET() == (char) 4711);
			assert (pack.flags_GET() == MAV_POWER_STATUS.MAV_POWER_STATUS_SERVO_VALID);
		});
		GroundControl.POWER_STATUS p125 = CommunicationChannel.new_POWER_STATUS();
		PH.setPack(p125);
		p125.Vcc_SET((char) 47670);
		p125.flags_SET(MAV_POWER_STATUS.MAV_POWER_STATUS_SERVO_VALID);
		p125.Vservo_SET((char) 4711);
		CommunicationChannel.instance.send(p125);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_SERIAL_CONTROL.add((src, ph, pack) ->
		{
			assert (Arrays.equals(pack.data__GET(), new char[]{(char) 181, (char) 47, (char) 232, (char) 82, (char) 47, (char) 226, (char) 69, (char) 7, (char) 202, (char) 253, (char) 207, (char) 27, (char) 84, (char) 252, (char) 113, (char) 101, (char) 199, (char) 64, (char) 6, (char) 36, (char) 46, (char) 11, (char) 115, (char) 185, (char) 53, (char) 225, (char) 0, (char) 159, (char) 137, (char) 143, (char) 215, (char) 232, (char) 171, (char) 199, (char) 44, (char) 59, (char) 80, (char) 143, (char) 168, (char) 144, (char) 29, (char) 91, (char) 220, (char) 251, (char) 71, (char) 194, (char) 87, (char) 154, (char) 103, (char) 117, (char) 124, (char) 45, (char) 159, (char) 173, (char) 87, (char) 58, (char) 15, (char) 103, (char) 247, (char) 171, (char) 49, (char) 172, (char) 229, (char) 0, (char) 16, (char) 229, (char) 6, (char) 169, (char) 12, (char) 178}));
			assert (pack.device_GET() == SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_TELEM2);
			assert (pack.flags_GET() == SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_REPLY);
			assert (pack.timeout_GET() == (char) 18915);
			assert (pack.count_GET() == (char) 41);
			assert (pack.baudrate_GET() == 1626472128L);
		});
		GroundControl.SERIAL_CONTROL p126 = CommunicationChannel.new_SERIAL_CONTROL();
		PH.setPack(p126);
		p126.count_SET((char) 41);
		p126.device_SET(SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_TELEM2);
		p126.timeout_SET((char) 18915);
		p126.data__SET(new char[]{(char) 181, (char) 47, (char) 232, (char) 82, (char) 47, (char) 226, (char) 69, (char) 7, (char) 202, (char) 253, (char) 207, (char) 27, (char) 84, (char) 252, (char) 113, (char) 101, (char) 199, (char) 64, (char) 6, (char) 36, (char) 46, (char) 11, (char) 115, (char) 185, (char) 53, (char) 225, (char) 0, (char) 159, (char) 137, (char) 143, (char) 215, (char) 232, (char) 171, (char) 199, (char) 44, (char) 59, (char) 80, (char) 143, (char) 168, (char) 144, (char) 29, (char) 91, (char) 220, (char) 251, (char) 71, (char) 194, (char) 87, (char) 154, (char) 103, (char) 117, (char) 124, (char) 45, (char) 159, (char) 173, (char) 87, (char) 58, (char) 15, (char) 103, (char) 247, (char) 171, (char) 49, (char) 172, (char) 229, (char) 0, (char) 16, (char) 229, (char) 6, (char) 169, (char) 12, (char) 178}, 0);
		p126.flags_SET(SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_REPLY);
		p126.baudrate_SET(1626472128L);
		CommunicationChannel.instance.send(p126);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_GPS_RTK.add((src, ph, pack) ->
		{
			assert (pack.nsats_GET() == (char) 46);
			assert (pack.rtk_health_GET() == (char) 98);
			assert (pack.iar_num_hypotheses_GET() == -1084084081);
			assert (pack.baseline_b_mm_GET() == 734791392);
			assert (pack.rtk_receiver_id_GET() == (char) 154);
			assert (pack.baseline_coords_type_GET() == (char) 249);
			assert (pack.baseline_c_mm_GET() == 294744201);
			assert (pack.time_last_baseline_ms_GET() == 757738972L);
			assert (pack.tow_GET() == 2006592241L);
			assert (pack.baseline_a_mm_GET() == 1841209891);
			assert (pack.wn_GET() == (char) 53856);
			assert (pack.rtk_rate_GET() == (char) 79);
			assert (pack.accuracy_GET() == 3107246687L);
		});
		GroundControl.GPS_RTK p127 = CommunicationChannel.new_GPS_RTK();
		PH.setPack(p127);
		p127.baseline_a_mm_SET(1841209891);
		p127.accuracy_SET(3107246687L);
		p127.rtk_rate_SET((char) 79);
		p127.baseline_c_mm_SET(294744201);
		p127.baseline_b_mm_SET(734791392);
		p127.tow_SET(2006592241L);
		p127.nsats_SET((char) 46);
		p127.wn_SET((char) 53856);
		p127.iar_num_hypotheses_SET(-1084084081);
		p127.rtk_receiver_id_SET((char) 154);
		p127.baseline_coords_type_SET((char) 249);
		p127.rtk_health_SET((char) 98);
		p127.time_last_baseline_ms_SET(757738972L);
		CommunicationChannel.instance.send(p127);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_GPS2_RTK.add((src, ph, pack) ->
		{
			assert (pack.rtk_rate_GET() == (char) 114);
			assert (pack.iar_num_hypotheses_GET() == -838701023);
			assert (pack.tow_GET() == 161519031L);
			assert (pack.wn_GET() == (char) 45083);
			assert (pack.nsats_GET() == (char) 75);
			assert (pack.rtk_health_GET() == (char) 238);
			assert (pack.baseline_a_mm_GET() == -1779759873);
			assert (pack.accuracy_GET() == 754654266L);
			assert (pack.baseline_b_mm_GET() == 873078423);
			assert (pack.baseline_c_mm_GET() == -387363370);
			assert (pack.baseline_coords_type_GET() == (char) 110);
			assert (pack.rtk_receiver_id_GET() == (char) 97);
			assert (pack.time_last_baseline_ms_GET() == 3887115978L);
		});
		GroundControl.GPS2_RTK p128 = CommunicationChannel.new_GPS2_RTK();
		PH.setPack(p128);
		p128.rtk_receiver_id_SET((char) 97);
		p128.rtk_health_SET((char) 238);
		p128.baseline_b_mm_SET(873078423);
		p128.iar_num_hypotheses_SET(-838701023);
		p128.tow_SET(161519031L);
		p128.wn_SET((char) 45083);
		p128.nsats_SET((char) 75);
		p128.rtk_rate_SET((char) 114);
		p128.accuracy_SET(754654266L);
		p128.baseline_c_mm_SET(-387363370);
		p128.time_last_baseline_ms_SET(3887115978L);
		p128.baseline_coords_type_SET((char) 110);
		p128.baseline_a_mm_SET(-1779759873);
		CommunicationChannel.instance.send(p128);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_SCALED_IMU3.add((src, ph, pack) ->
		{
			assert (pack.zgyro_GET() == (short) -28666);
			assert (pack.xgyro_GET() == (short) -27695);
			assert (pack.zmag_GET() == (short) -18617);
			assert (pack.time_boot_ms_GET() == 4262329848L);
			assert (pack.yacc_GET() == (short) 24318);
			assert (pack.xacc_GET() == (short) -22552);
			assert (pack.zacc_GET() == (short) -4912);
			assert (pack.xmag_GET() == (short) -25991);
			assert (pack.ymag_GET() == (short) -7520);
			assert (pack.ygyro_GET() == (short) -30241);
		});
		GroundControl.SCALED_IMU3 p129 = CommunicationChannel.new_SCALED_IMU3();
		PH.setPack(p129);
		p129.time_boot_ms_SET(4262329848L);
		p129.ygyro_SET((short) -30241);
		p129.zacc_SET((short) -4912);
		p129.xmag_SET((short) -25991);
		p129.zgyro_SET((short) -28666);
		p129.xgyro_SET((short) -27695);
		p129.yacc_SET((short) 24318);
		p129.ymag_SET((short) -7520);
		p129.xacc_SET((short) -22552);
		p129.zmag_SET((short) -18617);
		CommunicationChannel.instance.send(p129);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_DATA_TRANSMISSION_HANDSHAKE.add((src, ph, pack) ->
		{
			assert (pack.type_GET() == (char) 240);
			assert (pack.jpg_quality_GET() == (char) 79);
			assert (pack.width_GET() == (char) 22917);
			assert (pack.height_GET() == (char) 12182);
			assert (pack.payload_GET() == (char) 88);
			assert (pack.size_GET() == 788832529L);
			assert (pack.packets_GET() == (char) 14341);
		});
		GroundControl.DATA_TRANSMISSION_HANDSHAKE p130 = CommunicationChannel.new_DATA_TRANSMISSION_HANDSHAKE();
		PH.setPack(p130);
		p130.size_SET(788832529L);
		p130.type_SET((char) 240);
		p130.packets_SET((char) 14341);
		p130.jpg_quality_SET((char) 79);
		p130.height_SET((char) 12182);
		p130.payload_SET((char) 88);
		p130.width_SET((char) 22917);
		CommunicationChannel.instance.send(p130);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_ENCAPSULATED_DATA.add((src, ph, pack) ->
		{
			assert (pack.seqnr_GET() == (char) 65087);
			assert (Arrays.equals(pack.data__GET(), new char[]{(char) 211, (char) 240, (char) 74, (char) 66, (char) 227, (char) 41, (char) 181, (char) 184, (char) 252, (char) 196, (char) 182, (char) 239, (char) 61, (char) 74, (char) 77, (char) 102, (char) 215, (char) 129, (char) 144, (char) 223, (char) 27, (char) 59, (char) 77, (char) 80, (char) 159, (char) 59, (char) 241, (char) 87, (char) 242, (char) 8, (char) 130, (char) 154, (char) 55, (char) 212, (char) 64, (char) 44, (char) 199, (char) 86, (char) 91, (char) 184, (char) 55, (char) 30, (char) 88, (char) 30, (char) 177, (char) 124, (char) 135, (char) 56, (char) 171, (char) 121, (char) 73, (char) 198, (char) 231, (char) 201, (char) 132, (char) 149, (char) 179, (char) 239, (char) 47, (char) 252, (char) 163, (char) 230, (char) 210, (char) 74, (char) 102, (char) 54, (char) 135, (char) 54, (char) 45, (char) 2, (char) 49, (char) 42, (char) 223, (char) 68, (char) 28, (char) 27, (char) 154, (char) 36, (char) 204, (char) 154, (char) 94, (char) 148, (char) 82, (char) 112, (char) 61, (char) 210, (char) 27, (char) 36, (char) 85, (char) 141, (char) 33, (char) 255, (char) 182, (char) 39, (char) 139, (char) 88, (char) 4, (char) 240, (char) 183, (char) 61, (char) 193, (char) 162, (char) 145, (char) 153, (char) 16, (char) 228, (char) 252, (char) 108, (char) 223, (char) 81, (char) 100, (char) 251, (char) 230, (char) 222, (char) 44, (char) 159, (char) 85, (char) 65, (char) 163, (char) 191, (char) 194, (char) 186, (char) 156, (char) 219, (char) 193, (char) 30, (char) 40, (char) 253, (char) 95, (char) 197, (char) 221, (char) 50, (char) 195, (char) 253, (char) 94, (char) 70, (char) 28, (char) 92, (char) 103, (char) 90, (char) 182, (char) 164, (char) 77, (char) 97, (char) 217, (char) 235, (char) 127, (char) 0, (char) 140, (char) 146, (char) 61, (char) 61, (char) 166, (char) 59, (char) 118, (char) 235, (char) 3, (char) 90, (char) 132, (char) 49, (char) 138, (char) 26, (char) 247, (char) 160, (char) 160, (char) 146, (char) 49, (char) 117, (char) 158, (char) 193, (char) 40, (char) 136, (char) 156, (char) 142, (char) 55, (char) 176, (char) 81, (char) 134, (char) 7, (char) 154, (char) 95, (char) 120, (char) 142, (char) 97, (char) 35, (char) 122, (char) 32, (char) 169, (char) 54, (char) 235, (char) 153, (char) 191, (char) 7, (char) 202, (char) 242, (char) 249, (char) 244, (char) 140, (char) 204, (char) 95, (char) 130, (char) 146, (char) 5, (char) 49, (char) 134, (char) 173, (char) 214, (char) 138, (char) 14, (char) 36, (char) 64, (char) 196, (char) 74, (char) 63, (char) 43, (char) 13, (char) 127, (char) 24, (char) 206, (char) 173, (char) 221, (char) 222, (char) 72, (char) 128, (char) 8, (char) 34, (char) 88, (char) 10, (char) 210, (char) 221, (char) 110, (char) 247, (char) 78, (char) 36, (char) 46, (char) 205, (char) 126, (char) 127, (char) 36, (char) 146, (char) 182, (char) 18, (char) 137, (char) 33, (char) 9, (char) 39, (char) 60, (char) 185, (char) 107, (char) 100, (char) 250, (char) 134, (char) 68}));
		});
		GroundControl.ENCAPSULATED_DATA p131 = CommunicationChannel.new_ENCAPSULATED_DATA();
		PH.setPack(p131);
		p131.seqnr_SET((char) 65087);
		p131.data__SET(new char[]{(char) 211, (char) 240, (char) 74, (char) 66, (char) 227, (char) 41, (char) 181, (char) 184, (char) 252, (char) 196, (char) 182, (char) 239, (char) 61, (char) 74, (char) 77, (char) 102, (char) 215, (char) 129, (char) 144, (char) 223, (char) 27, (char) 59, (char) 77, (char) 80, (char) 159, (char) 59, (char) 241, (char) 87, (char) 242, (char) 8, (char) 130, (char) 154, (char) 55, (char) 212, (char) 64, (char) 44, (char) 199, (char) 86, (char) 91, (char) 184, (char) 55, (char) 30, (char) 88, (char) 30, (char) 177, (char) 124, (char) 135, (char) 56, (char) 171, (char) 121, (char) 73, (char) 198, (char) 231, (char) 201, (char) 132, (char) 149, (char) 179, (char) 239, (char) 47, (char) 252, (char) 163, (char) 230, (char) 210, (char) 74, (char) 102, (char) 54, (char) 135, (char) 54, (char) 45, (char) 2, (char) 49, (char) 42, (char) 223, (char) 68, (char) 28, (char) 27, (char) 154, (char) 36, (char) 204, (char) 154, (char) 94, (char) 148, (char) 82, (char) 112, (char) 61, (char) 210, (char) 27, (char) 36, (char) 85, (char) 141, (char) 33, (char) 255, (char) 182, (char) 39, (char) 139, (char) 88, (char) 4, (char) 240, (char) 183, (char) 61, (char) 193, (char) 162, (char) 145, (char) 153, (char) 16, (char) 228, (char) 252, (char) 108, (char) 223, (char) 81, (char) 100, (char) 251, (char) 230, (char) 222, (char) 44, (char) 159, (char) 85, (char) 65, (char) 163, (char) 191, (char) 194, (char) 186, (char) 156, (char) 219, (char) 193, (char) 30, (char) 40, (char) 253, (char) 95, (char) 197, (char) 221, (char) 50, (char) 195, (char) 253, (char) 94, (char) 70, (char) 28, (char) 92, (char) 103, (char) 90, (char) 182, (char) 164, (char) 77, (char) 97, (char) 217, (char) 235, (char) 127, (char) 0, (char) 140, (char) 146, (char) 61, (char) 61, (char) 166, (char) 59, (char) 118, (char) 235, (char) 3, (char) 90, (char) 132, (char) 49, (char) 138, (char) 26, (char) 247, (char) 160, (char) 160, (char) 146, (char) 49, (char) 117, (char) 158, (char) 193, (char) 40, (char) 136, (char) 156, (char) 142, (char) 55, (char) 176, (char) 81, (char) 134, (char) 7, (char) 154, (char) 95, (char) 120, (char) 142, (char) 97, (char) 35, (char) 122, (char) 32, (char) 169, (char) 54, (char) 235, (char) 153, (char) 191, (char) 7, (char) 202, (char) 242, (char) 249, (char) 244, (char) 140, (char) 204, (char) 95, (char) 130, (char) 146, (char) 5, (char) 49, (char) 134, (char) 173, (char) 214, (char) 138, (char) 14, (char) 36, (char) 64, (char) 196, (char) 74, (char) 63, (char) 43, (char) 13, (char) 127, (char) 24, (char) 206, (char) 173, (char) 221, (char) 222, (char) 72, (char) 128, (char) 8, (char) 34, (char) 88, (char) 10, (char) 210, (char) 221, (char) 110, (char) 247, (char) 78, (char) 36, (char) 46, (char) 205, (char) 126, (char) 127, (char) 36, (char) 146, (char) 182, (char) 18, (char) 137, (char) 33, (char) 9, (char) 39, (char) 60, (char) 185, (char) 107, (char) 100, (char) 250, (char) 134, (char) 68}, 0);
		CommunicationChannel.instance.send(p131);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_DISTANCE_SENSOR.add((src, ph, pack) ->
		{
			assert (pack.max_distance_GET() == (char) 16107);
			assert (pack.time_boot_ms_GET() == 787555083L);
			assert (pack.current_distance_GET() == (char) 50518);
			assert (pack.covariance_GET() == (char) 70);
			assert (pack.id_GET() == (char) 226);
			assert (pack.min_distance_GET() == (char) 49108);
			assert (pack.type_GET() == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_ULTRASOUND);
			assert (pack.orientation_GET() == MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_90_YAW_90);
		});
		GroundControl.DISTANCE_SENSOR p132 = CommunicationChannel.new_DISTANCE_SENSOR();
		PH.setPack(p132);
		p132.time_boot_ms_SET(787555083L);
		p132.min_distance_SET((char) 49108);
		p132.covariance_SET((char) 70);
		p132.current_distance_SET((char) 50518);
		p132.type_SET(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_ULTRASOUND);
		p132.id_SET((char) 226);
		p132.max_distance_SET((char) 16107);
		p132.orientation_SET(MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_90_YAW_90);
		CommunicationChannel.instance.send(p132);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_TERRAIN_REQUEST.add((src, ph, pack) ->
		{
			assert (pack.grid_spacing_GET() == (char) 64405);
			assert (pack.mask_GET() == 7722538748529579926L);
			assert (pack.lat_GET() == -1003213242);
			assert (pack.lon_GET() == -2072541873);
		});
		GroundControl.TERRAIN_REQUEST p133 = CommunicationChannel.new_TERRAIN_REQUEST();
		PH.setPack(p133);
		p133.lon_SET(-2072541873);
		p133.grid_spacing_SET((char) 64405);
		p133.mask_SET(7722538748529579926L);
		p133.lat_SET(-1003213242);
		CommunicationChannel.instance.send(p133);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_TERRAIN_DATA.add((src, ph, pack) ->
		{
			assert (Arrays.equals(pack.data__GET(), new short[]{(short) -11114, (short) 29583, (short) -2701, (short) -8662, (short) -23269, (short) 14608, (short) 8196, (short) -12613, (short) 17491, (short) -30004, (short) 13465, (short) 25877, (short) 2234, (short) -6058, (short) -19238, (short) 15237}));
			assert (pack.lon_GET() == -1506956676);
			assert (pack.grid_spacing_GET() == (char) 32406);
			assert (pack.lat_GET() == 341430516);
			assert (pack.gridbit_GET() == (char) 224);
		});
		GroundControl.TERRAIN_DATA p134 = CommunicationChannel.new_TERRAIN_DATA();
		PH.setPack(p134);
		p134.lon_SET(-1506956676);
		p134.lat_SET(341430516);
		p134.data__SET(new short[]{(short) -11114, (short) 29583, (short) -2701, (short) -8662, (short) -23269, (short) 14608, (short) 8196, (short) -12613, (short) 17491, (short) -30004, (short) 13465, (short) 25877, (short) 2234, (short) -6058, (short) -19238, (short) 15237}, 0);
		p134.grid_spacing_SET((char) 32406);
		p134.gridbit_SET((char) 224);
		CommunicationChannel.instance.send(p134);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_TERRAIN_CHECK.add((src, ph, pack) ->
		{
			assert (pack.lat_GET() == -705214993);
			assert (pack.lon_GET() == -529295044);
		});
		GroundControl.TERRAIN_CHECK p135 = CommunicationChannel.new_TERRAIN_CHECK();
		PH.setPack(p135);
		p135.lat_SET(-705214993);
		p135.lon_SET(-529295044);
		CommunicationChannel.instance.send(p135);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_TERRAIN_REPORT.add((src, ph, pack) ->
		{
			assert (pack.loaded_GET() == (char) 13238);
			assert (pack.lat_GET() == -1320598685);
			assert (pack.current_height_GET() == -6.1286896E37F);
			assert (pack.terrain_height_GET() == -7.1239155E37F);
			assert (pack.pending_GET() == (char) 44475);
			assert (pack.lon_GET() == -568123831);
			assert (pack.spacing_GET() == (char) 15136);
		});
		GroundControl.TERRAIN_REPORT p136 = CommunicationChannel.new_TERRAIN_REPORT();
		PH.setPack(p136);
		p136.pending_SET((char) 44475);
		p136.loaded_SET((char) 13238);
		p136.spacing_SET((char) 15136);
		p136.current_height_SET(-6.1286896E37F);
		p136.lat_SET(-1320598685);
		p136.terrain_height_SET(-7.1239155E37F);
		p136.lon_SET(-568123831);
		CommunicationChannel.instance.send(p136);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_SCALED_PRESSURE2.add((src, ph, pack) ->
		{
			assert (pack.time_boot_ms_GET() == 2137594389L);
			assert (pack.temperature_GET() == (short) 26053);
			assert (pack.press_diff_GET() == -2.6287562E37F);
			assert (pack.press_abs_GET() == -1.3169132E38F);
		});
		GroundControl.SCALED_PRESSURE2 p137 = CommunicationChannel.new_SCALED_PRESSURE2();
		PH.setPack(p137);
		p137.temperature_SET((short) 26053);
		p137.press_diff_SET(-2.6287562E37F);
		p137.time_boot_ms_SET(2137594389L);
		p137.press_abs_SET(-1.3169132E38F);
		CommunicationChannel.instance.send(p137);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_ATT_POS_MOCAP.add((src, ph, pack) ->
		{
			assert (pack.z_GET() == -1.3446166E38F);
			assert (pack.time_usec_GET() == 8133064793497462550L);
			assert (pack.x_GET() == 1.3599973E37F);
			assert (pack.y_GET() == -2.976011E38F);
			assert (Arrays.equals(pack.q_GET(), new float[]{-1.6903319E38F, -1.7866746E38F, 2.0270597E38F, 8.2282805E37F}));
		});
		GroundControl.ATT_POS_MOCAP p138 = CommunicationChannel.new_ATT_POS_MOCAP();
		PH.setPack(p138);
		p138.time_usec_SET(8133064793497462550L);
		p138.z_SET(-1.3446166E38F);
		p138.q_SET(new float[]{-1.6903319E38F, -1.7866746E38F, 2.0270597E38F, 8.2282805E37F}, 0);
		p138.x_SET(1.3599973E37F);
		p138.y_SET(-2.976011E38F);
		CommunicationChannel.instance.send(p138);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_SET_ACTUATOR_CONTROL_TARGET.add((src, ph, pack) ->
		{
			assert (Arrays.equals(pack.controls_GET(), new float[]{2.0220396E38F, 1.4818981E38F, 7.496156E37F, 2.0514256E38F, 3.864737E37F, -2.2474352E38F, 1.3689591E38F, 3.3236077E38F}));
			assert (pack.target_system_GET() == (char) 80);
			assert (pack.target_component_GET() == (char) 241);
			assert (pack.group_mlx_GET() == (char) 189);
			assert (pack.time_usec_GET() == 5780146134535472685L);
		});
		GroundControl.SET_ACTUATOR_CONTROL_TARGET p139 = CommunicationChannel.new_SET_ACTUATOR_CONTROL_TARGET();
		PH.setPack(p139);
		p139.target_system_SET((char) 80);
		p139.target_component_SET((char) 241);
		p139.controls_SET(new float[]{2.0220396E38F, 1.4818981E38F, 7.496156E37F, 2.0514256E38F, 3.864737E37F, -2.2474352E38F, 1.3689591E38F, 3.3236077E38F}, 0);
		p139.group_mlx_SET((char) 189);
		p139.time_usec_SET(5780146134535472685L);
		CommunicationChannel.instance.send(p139);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_ACTUATOR_CONTROL_TARGET.add((src, ph, pack) ->
		{
			assert (pack.time_usec_GET() == 1736229034091992328L);
			assert (pack.group_mlx_GET() == (char) 40);
			assert (Arrays.equals(pack.controls_GET(), new float[]{1.0786671E38F, -2.4200505E38F, -2.989702E38F, 2.190858E38F, 1.132512E38F, 3.223792E38F, -3.3716573E38F, -5.4264035E37F}));
		});
		GroundControl.ACTUATOR_CONTROL_TARGET p140 = CommunicationChannel.new_ACTUATOR_CONTROL_TARGET();
		PH.setPack(p140);
		p140.time_usec_SET(1736229034091992328L);
		p140.group_mlx_SET((char) 40);
		p140.controls_SET(new float[]{1.0786671E38F, -2.4200505E38F, -2.989702E38F, 2.190858E38F, 1.132512E38F, 3.223792E38F, -3.3716573E38F, -5.4264035E37F}, 0);
		CommunicationChannel.instance.send(p140);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_ALTITUDE.add((src, ph, pack) ->
		{
			assert (pack.altitude_amsl_GET() == -5.9805717E37F);
			assert (pack.altitude_monotonic_GET() == 2.4634145E38F);
			assert (pack.altitude_terrain_GET() == 1.7097866E38F);
			assert (pack.bottom_clearance_GET() == -6.9497535E37F);
			assert (pack.altitude_local_GET() == -8.268643E37F);
			assert (pack.time_usec_GET() == 8638659715822464735L);
			assert (pack.altitude_relative_GET() == 2.7197456E37F);
		});
		GroundControl.ALTITUDE p141 = CommunicationChannel.new_ALTITUDE();
		PH.setPack(p141);
		p141.time_usec_SET(8638659715822464735L);
		p141.altitude_amsl_SET(-5.9805717E37F);
		p141.altitude_relative_SET(2.7197456E37F);
		p141.altitude_terrain_SET(1.7097866E38F);
		p141.altitude_monotonic_SET(2.4634145E38F);
		p141.altitude_local_SET(-8.268643E37F);
		p141.bottom_clearance_SET(-6.9497535E37F);
		CommunicationChannel.instance.send(p141);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_RESOURCE_REQUEST.add((src, ph, pack) ->
		{
			assert (pack.transfer_type_GET() == (char) 144);
			assert (pack.uri_type_GET() == (char) 124);
			assert (Arrays.equals(pack.storage_GET(), new char[]{(char) 9, (char) 168, (char) 208, (char) 144, (char) 200, (char) 241, (char) 195, (char) 202, (char) 99, (char) 192, (char) 42, (char) 92, (char) 180, (char) 83, (char) 41, (char) 204, (char) 184, (char) 158, (char) 198, (char) 26, (char) 177, (char) 15, (char) 79, (char) 184, (char) 206, (char) 109, (char) 103, (char) 163, (char) 47, (char) 233, (char) 125, (char) 254, (char) 172, (char) 142, (char) 136, (char) 7, (char) 145, (char) 134, (char) 47, (char) 143, (char) 158, (char) 18, (char) 54, (char) 115, (char) 8, (char) 126, (char) 238, (char) 101, (char) 68, (char) 115, (char) 187, (char) 143, (char) 210, (char) 159, (char) 182, (char) 212, (char) 82, (char) 245, (char) 113, (char) 116, (char) 53, (char) 51, (char) 40, (char) 155, (char) 134, (char) 54, (char) 132, (char) 201, (char) 213, (char) 225, (char) 242, (char) 163, (char) 186, (char) 28, (char) 229, (char) 213, (char) 231, (char) 102, (char) 200, (char) 114, (char) 125, (char) 106, (char) 9, (char) 163, (char) 237, (char) 38, (char) 22, (char) 242, (char) 92, (char) 216, (char) 246, (char) 158, (char) 20, (char) 115, (char) 104, (char) 246, (char) 21, (char) 154, (char) 142, (char) 42, (char) 198, (char) 62, (char) 145, (char) 61, (char) 141, (char) 118, (char) 214, (char) 68, (char) 56, (char) 96, (char) 195, (char) 253, (char) 84, (char) 75, (char) 115, (char) 146, (char) 80, (char) 83, (char) 218, (char) 17}));
			assert (pack.request_id_GET() == (char) 247);
			assert (Arrays.equals(pack.uri_GET(), new char[]{(char) 242, (char) 220, (char) 128, (char) 43, (char) 212, (char) 131, (char) 250, (char) 255, (char) 246, (char) 239, (char) 66, (char) 24, (char) 15, (char) 212, (char) 189, (char) 222, (char) 129, (char) 24, (char) 108, (char) 229, (char) 153, (char) 54, (char) 206, (char) 62, (char) 225, (char) 213, (char) 180, (char) 36, (char) 222, (char) 179, (char) 186, (char) 163, (char) 163, (char) 198, (char) 22, (char) 228, (char) 109, (char) 238, (char) 178, (char) 194, (char) 251, (char) 109, (char) 43, (char) 11, (char) 68, (char) 246, (char) 64, (char) 244, (char) 60, (char) 113, (char) 146, (char) 124, (char) 105, (char) 168, (char) 62, (char) 96, (char) 119, (char) 232, (char) 37, (char) 13, (char) 209, (char) 146, (char) 91, (char) 228, (char) 143, (char) 40, (char) 100, (char) 95, (char) 124, (char) 6, (char) 154, (char) 246, (char) 12, (char) 198, (char) 16, (char) 64, (char) 248, (char) 3, (char) 162, (char) 183, (char) 246, (char) 203, (char) 230, (char) 123, (char) 244, (char) 113, (char) 104, (char) 118, (char) 204, (char) 114, (char) 90, (char) 36, (char) 238, (char) 186, (char) 225, (char) 53, (char) 164, (char) 177, (char) 93, (char) 124, (char) 43, (char) 94, (char) 76, (char) 167, (char) 203, (char) 181, (char) 252, (char) 90, (char) 168, (char) 25, (char) 161, (char) 46, (char) 210, (char) 193, (char) 15, (char) 205, (char) 89, (char) 136, (char) 167, (char) 176}));
		});
		GroundControl.RESOURCE_REQUEST p142 = CommunicationChannel.new_RESOURCE_REQUEST();
		PH.setPack(p142);
		p142.uri_SET(new char[]{(char) 242, (char) 220, (char) 128, (char) 43, (char) 212, (char) 131, (char) 250, (char) 255, (char) 246, (char) 239, (char) 66, (char) 24, (char) 15, (char) 212, (char) 189, (char) 222, (char) 129, (char) 24, (char) 108, (char) 229, (char) 153, (char) 54, (char) 206, (char) 62, (char) 225, (char) 213, (char) 180, (char) 36, (char) 222, (char) 179, (char) 186, (char) 163, (char) 163, (char) 198, (char) 22, (char) 228, (char) 109, (char) 238, (char) 178, (char) 194, (char) 251, (char) 109, (char) 43, (char) 11, (char) 68, (char) 246, (char) 64, (char) 244, (char) 60, (char) 113, (char) 146, (char) 124, (char) 105, (char) 168, (char) 62, (char) 96, (char) 119, (char) 232, (char) 37, (char) 13, (char) 209, (char) 146, (char) 91, (char) 228, (char) 143, (char) 40, (char) 100, (char) 95, (char) 124, (char) 6, (char) 154, (char) 246, (char) 12, (char) 198, (char) 16, (char) 64, (char) 248, (char) 3, (char) 162, (char) 183, (char) 246, (char) 203, (char) 230, (char) 123, (char) 244, (char) 113, (char) 104, (char) 118, (char) 204, (char) 114, (char) 90, (char) 36, (char) 238, (char) 186, (char) 225, (char) 53, (char) 164, (char) 177, (char) 93, (char) 124, (char) 43, (char) 94, (char) 76, (char) 167, (char) 203, (char) 181, (char) 252, (char) 90, (char) 168, (char) 25, (char) 161, (char) 46, (char) 210, (char) 193, (char) 15, (char) 205, (char) 89, (char) 136, (char) 167, (char) 176}, 0);
		p142.uri_type_SET((char) 124);
		p142.transfer_type_SET((char) 144);
		p142.request_id_SET((char) 247);
		p142.storage_SET(new char[]{(char) 9, (char) 168, (char) 208, (char) 144, (char) 200, (char) 241, (char) 195, (char) 202, (char) 99, (char) 192, (char) 42, (char) 92, (char) 180, (char) 83, (char) 41, (char) 204, (char) 184, (char) 158, (char) 198, (char) 26, (char) 177, (char) 15, (char) 79, (char) 184, (char) 206, (char) 109, (char) 103, (char) 163, (char) 47, (char) 233, (char) 125, (char) 254, (char) 172, (char) 142, (char) 136, (char) 7, (char) 145, (char) 134, (char) 47, (char) 143, (char) 158, (char) 18, (char) 54, (char) 115, (char) 8, (char) 126, (char) 238, (char) 101, (char) 68, (char) 115, (char) 187, (char) 143, (char) 210, (char) 159, (char) 182, (char) 212, (char) 82, (char) 245, (char) 113, (char) 116, (char) 53, (char) 51, (char) 40, (char) 155, (char) 134, (char) 54, (char) 132, (char) 201, (char) 213, (char) 225, (char) 242, (char) 163, (char) 186, (char) 28, (char) 229, (char) 213, (char) 231, (char) 102, (char) 200, (char) 114, (char) 125, (char) 106, (char) 9, (char) 163, (char) 237, (char) 38, (char) 22, (char) 242, (char) 92, (char) 216, (char) 246, (char) 158, (char) 20, (char) 115, (char) 104, (char) 246, (char) 21, (char) 154, (char) 142, (char) 42, (char) 198, (char) 62, (char) 145, (char) 61, (char) 141, (char) 118, (char) 214, (char) 68, (char) 56, (char) 96, (char) 195, (char) 253, (char) 84, (char) 75, (char) 115, (char) 146, (char) 80, (char) 83, (char) 218, (char) 17}, 0);
		CommunicationChannel.instance.send(p142);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		TestChannel.instance.on_SCALED_PRESSURE3.add((src, ph, pack) ->
		{
			assert (pack.press_abs_GET() == -6.5067795E37F);
			assert (pack.press_diff_GET() == -5.3329696E37F);
			assert (pack.temperature_GET() == (short) 8595);
			assert (pack.time_boot_ms_GET() == 2033951674L);
		});
		GroundControl.SCALED_PRESSURE3 p143 = CommunicationChannel.new_SCALED_PRESSURE3();
		PH.setPack(p143);
		p143.temperature_SET((short) 8595);
		p143.time_boot_ms_SET(2033951674L);
		p143.press_abs_SET(-6.5067795E37F);
		p143.press_diff_SET(-5.3329696E37F);
		CommunicationChannel.instance.send(p143);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_FOLLOW_TARGET.add((src, ph, pack) ->
		{
			assert (pack.lat_GET() == -766961032);
			assert (pack.alt_GET() == -1.393705E38F);
			assert (pack.custom_state_GET() == 3123391118323558624L);
			assert (Arrays.equals(pack.vel_GET(), new float[]{6.8887754E37F, 2.3124056E38F, 1.2764539E38F}));
			assert (Arrays.equals(pack.rates_GET(), new float[]{1.3730354E38F, 1.859134E38F, 1.8745025E37F}));
			assert (pack.lon_GET() == 839837024);
			assert (Arrays.equals(pack.attitude_q_GET(), new float[]{-8.990888E37F, 2.1675614E38F, -3.24798E38F, 5.3668427E37F}));
			assert (Arrays.equals(pack.acc_GET(), new float[]{-6.995704E37F, -2.0084646E38F, 1.104803E37F}));
			assert (Arrays.equals(pack.position_cov_GET(), new float[]{-1.2239592E38F, -1.624528E38F, 2.304938E38F}));
			assert (pack.est_capabilities_GET() == (char) 55);
			assert (pack.timestamp_GET() == 6477876962260707315L);
		});
		GroundControl.FOLLOW_TARGET p144 = CommunicationChannel.new_FOLLOW_TARGET();
		PH.setPack(p144);
		p144.acc_SET(new float[]{-6.995704E37F, -2.0084646E38F, 1.104803E37F}, 0);
		p144.custom_state_SET(3123391118323558624L);
		p144.alt_SET(-1.393705E38F);
		p144.timestamp_SET(6477876962260707315L);
		p144.vel_SET(new float[]{6.8887754E37F, 2.3124056E38F, 1.2764539E38F}, 0);
		p144.rates_SET(new float[]{1.3730354E38F, 1.859134E38F, 1.8745025E37F}, 0);
		p144.position_cov_SET(new float[]{-1.2239592E38F, -1.624528E38F, 2.304938E38F}, 0);
		p144.lon_SET(839837024);
		p144.attitude_q_SET(new float[]{-8.990888E37F, 2.1675614E38F, -3.24798E38F, 5.3668427E37F}, 0);
		p144.lat_SET(-766961032);
		p144.est_capabilities_SET((char) 55);
		CommunicationChannel.instance.send(p144);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_CONTROL_SYSTEM_STATE.add((src, ph, pack) ->
		{
			assert (pack.y_vel_GET() == 3.1118526E38F);
			assert (pack.x_pos_GET() == 3.1308964E38F);
			assert (pack.x_acc_GET() == -1.252847E38F);
			assert (pack.airspeed_GET() == -2.7193728E38F);
			assert (pack.z_pos_GET() == -2.2919636E38F);
			assert (pack.y_pos_GET() == -1.5613363E38F);
			assert (Arrays.equals(pack.vel_variance_GET(), new float[]{-2.8290134E38F, 3.2714275E38F, 2.2167065E38F}));
			assert (pack.roll_rate_GET() == 2.2970306E38F);
			assert (pack.x_vel_GET() == -6.8167E37F);
			assert (pack.z_acc_GET() == 1.9631391E37F);
			assert (Arrays.equals(pack.pos_variance_GET(), new float[]{-3.3901133E38F, 1.3458401E38F, -2.2345177E36F}));
			assert (Arrays.equals(pack.q_GET(), new float[]{-3.3370164E38F, -9.723745E37F, 2.9968613E38F, 6.814194E37F}));
			assert (pack.yaw_rate_GET() == -1.9317083E38F);
			assert (pack.time_usec_GET() == 4551426101045614662L);
			assert (pack.y_acc_GET() == 1.6707358E38F);
			assert (pack.z_vel_GET() == -2.9548009E38F);
			assert (pack.pitch_rate_GET() == -2.494081E38F);
		});
		GroundControl.CONTROL_SYSTEM_STATE p146 = CommunicationChannel.new_CONTROL_SYSTEM_STATE();
		PH.setPack(p146);
		p146.vel_variance_SET(new float[]{-2.8290134E38F, 3.2714275E38F, 2.2167065E38F}, 0);
		p146.x_vel_SET(-6.8167E37F);
		p146.z_pos_SET(-2.2919636E38F);
		p146.pos_variance_SET(new float[]{-3.3901133E38F, 1.3458401E38F, -2.2345177E36F}, 0);
		p146.z_acc_SET(1.9631391E37F);
		p146.y_pos_SET(-1.5613363E38F);
		p146.roll_rate_SET(2.2970306E38F);
		p146.time_usec_SET(4551426101045614662L);
		p146.y_vel_SET(3.1118526E38F);
		p146.x_acc_SET(-1.252847E38F);
		p146.z_vel_SET(-2.9548009E38F);
		p146.x_pos_SET(3.1308964E38F);
		p146.pitch_rate_SET(-2.494081E38F);
		p146.y_acc_SET(1.6707358E38F);
		p146.airspeed_SET(-2.7193728E38F);
		p146.yaw_rate_SET(-1.9317083E38F);
		p146.q_SET(new float[]{-3.3370164E38F, -9.723745E37F, 2.9968613E38F, 6.814194E37F}, 0);
		CommunicationChannel.instance.send(p146);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_BATTERY_STATUS.add((src, ph, pack) ->
		{
			assert (pack.temperature_GET() == (short) 3014);
			assert (pack.energy_consumed_GET() == -458544736);
			assert (pack.id_GET() == (char) 88);
			assert (pack.type_GET() == MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_LIFE);
			assert (pack.current_consumed_GET() == -1497600230);
			assert (pack.battery_remaining_GET() == (byte) -106);
			assert (pack.battery_function_GET() == MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_PROPULSION);
			assert (pack.current_battery_GET() == (short) 30573);
			assert (Arrays.equals(pack.voltages_GET(), new char[]{(char) 17524, (char) 57535, (char) 61424, (char) 48283, (char) 11359, (char) 24501, (char) 48128, (char) 54439, (char) 57172, (char) 4526}));
		});
		GroundControl.BATTERY_STATUS p147 = CommunicationChannel.new_BATTERY_STATUS();
		PH.setPack(p147);
		p147.current_battery_SET((short) 30573);
		p147.energy_consumed_SET(-458544736);
		p147.battery_remaining_SET((byte) -106);
		p147.temperature_SET((short) 3014);
		p147.battery_function_SET(MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_PROPULSION);
		p147.type_SET(MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_LIFE);
		p147.voltages_SET(new char[]{(char) 17524, (char) 57535, (char) 61424, (char) 48283, (char) 11359, (char) 24501, (char) 48128, (char) 54439, (char) 57172, (char) 4526}, 0);
		p147.id_SET((char) 88);
		p147.current_consumed_SET(-1497600230);
		CommunicationChannel.instance.send(p147);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_AUTOPILOT_VERSION.add((src, ph, pack) ->
		{
			assert (Arrays.equals(pack.uid2_TRY(ph), new char[]{(char) 188, (char) 17, (char) 233, (char) 255, (char) 248, (char) 36, (char) 161, (char) 89, (char) 203, (char) 100, (char) 195, (char) 95, (char) 170, (char) 109, (char) 9, (char) 167, (char) 123, (char) 145}));
			assert (pack.capabilities_GET() == MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MAVLINK2);
			assert (pack.board_version_GET() == 2683551559L);
			assert (Arrays.equals(pack.os_custom_version_GET(), new char[]{(char) 7, (char) 24, (char) 202, (char) 69, (char) 145, (char) 47, (char) 91, (char) 154}));
			assert (pack.os_sw_version_GET() == 1764478203L);
			assert (pack.product_id_GET() == (char) 16705);
			assert (pack.vendor_id_GET() == (char) 6717);
			assert (Arrays.equals(pack.middleware_custom_version_GET(), new char[]{(char) 94, (char) 41, (char) 228, (char) 212, (char) 109, (char) 124, (char) 81, (char) 19}));
			assert (pack.middleware_sw_version_GET() == 1682724196L);
			assert (Arrays.equals(pack.flight_custom_version_GET(), new char[]{(char) 147, (char) 232, (char) 39, (char) 15, (char) 85, (char) 2, (char) 223, (char) 184}));
			assert (pack.uid_GET() == 6861902054682637138L);
			assert (pack.flight_sw_version_GET() == 1025398548L);
		});
		GroundControl.AUTOPILOT_VERSION p148 = CommunicationChannel.new_AUTOPILOT_VERSION();
		PH.setPack(p148);
		p148.middleware_custom_version_SET(new char[]{(char) 94, (char) 41, (char) 228, (char) 212, (char) 109, (char) 124, (char) 81, (char) 19}, 0);
		p148.flight_custom_version_SET(new char[]{(char) 147, (char) 232, (char) 39, (char) 15, (char) 85, (char) 2, (char) 223, (char) 184}, 0);
		p148.os_sw_version_SET(1764478203L);
		p148.uid_SET(6861902054682637138L);
		p148.os_custom_version_SET(new char[]{(char) 7, (char) 24, (char) 202, (char) 69, (char) 145, (char) 47, (char) 91, (char) 154}, 0);
		p148.uid2_SET(new char[]{(char) 188, (char) 17, (char) 233, (char) 255, (char) 248, (char) 36, (char) 161, (char) 89, (char) 203, (char) 100, (char) 195, (char) 95, (char) 170, (char) 109, (char) 9, (char) 167, (char) 123, (char) 145}, 0, PH);
		p148.product_id_SET((char) 16705);
		p148.capabilities_SET(MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MAVLINK2);
		p148.board_version_SET(2683551559L);
		p148.vendor_id_SET((char) 6717);
		p148.flight_sw_version_SET(1025398548L);
		p148.middleware_sw_version_SET(1682724196L);
		CommunicationChannel.instance.send(p148);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_LANDING_TARGET.add((src, ph, pack) ->
		{
			assert (pack.size_x_GET() == -3.4000862E38F);
			assert (pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT);
			assert (pack.angle_x_GET() == 6.9464464E37F);
			assert (pack.position_valid_TRY(ph) == (char) 133);
			assert (pack.y_TRY(ph) == 3.0518694E38F);
			assert (pack.angle_y_GET() == 2.7801114E37F);
			assert (pack.z_TRY(ph) == -1.3429385E38F);
			assert (pack.x_TRY(ph) == -2.5701995E38F);
			assert (pack.target_num_GET() == (char) 110);
			assert (pack.time_usec_GET() == 4382591523503511306L);
			assert (Arrays.equals(pack.q_TRY(ph), new float[]{-2.3768905E38F, 3.6456366E37F, 2.0321471E38F, 8.265765E37F}));
			assert (pack.size_y_GET() == 4.7062943E37F);
			assert (pack.distance_GET() == 2.891103E38F);
			assert (pack.type_GET() == LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_RADIO_BEACON);
		});
		GroundControl.LANDING_TARGET p149 = CommunicationChannel.new_LANDING_TARGET();
		PH.setPack(p149);
		p149.angle_y_SET(2.7801114E37F);
		p149.time_usec_SET(4382591523503511306L);
		p149.x_SET(-2.5701995E38F, PH);
		p149.y_SET(3.0518694E38F, PH);
		p149.angle_x_SET(6.9464464E37F);
		p149.q_SET(new float[]{-2.3768905E38F, 3.6456366E37F, 2.0321471E38F, 8.265765E37F}, 0, PH);
		p149.z_SET(-1.3429385E38F, PH);
		p149.size_y_SET(4.7062943E37F);
		p149.size_x_SET(-3.4000862E38F);
		p149.type_SET(LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_RADIO_BEACON);
		p149.target_num_SET((char) 110);
		p149.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT);
		p149.distance_SET(2.891103E38F);
		p149.position_valid_SET((char) 133, PH);
		CommunicationChannel.instance.send(p149);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_ARRAY_TEST_0.add((src, ph, pack) ->
		{
			assert (Arrays.equals(pack.ar_u32_GET(), new long[]{697750703L, 932360562L, 1189249361L, 1047869153L}));
			assert (Arrays.equals(pack.ar_u16_GET(), new char[]{(char) 28797, (char) 32470, (char) 14859, (char) 22745}));
			assert (Arrays.equals(pack.ar_u8_GET(), new char[]{(char) 175, (char) 145, (char) 158, (char) 50}));
			assert (pack.v1_GET() == (char) 238);
			assert (Arrays.equals(pack.ar_i8_GET(), new byte[]{(byte) 17, (byte) 77, (byte) 77, (byte) -69}));
		});
		GroundControl.ARRAY_TEST_0 p150 = CommunicationChannel.new_ARRAY_TEST_0();
		PH.setPack(p150);
		p150.ar_u32_SET(new long[]{697750703L, 932360562L, 1189249361L, 1047869153L}, 0);
		p150.ar_u8_SET(new char[]{(char) 175, (char) 145, (char) 158, (char) 50}, 0);
		p150.v1_SET((char) 238);
		p150.ar_i8_SET(new byte[]{(byte) 17, (byte) 77, (byte) 77, (byte) -69}, 0);
		p150.ar_u16_SET(new char[]{(char) 28797, (char) 32470, (char) 14859, (char) 22745}, 0);
		CommunicationChannel.instance.send(p150);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_ARRAY_TEST_1.add((src, ph, pack) ->
		{
			assert (Arrays.equals(pack.ar_u32_GET(), new long[]{1879197651L, 1754911010L, 2941599423L, 1952898884L}));
		});
		GroundControl.ARRAY_TEST_1 p151 = CommunicationChannel.new_ARRAY_TEST_1();
		PH.setPack(p151);
		p151.ar_u32_SET(new long[]{1879197651L, 1754911010L, 2941599423L, 1952898884L}, 0);
		CommunicationChannel.instance.send(p151);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_ARRAY_TEST_3.add((src, ph, pack) ->
		{
			assert (Arrays.equals(pack.ar_u32_GET(), new long[]{3086105250L, 757137702L, 3366615865L, 341011648L}));
			assert (pack.v_GET() == (char) 25);
		});
		GroundControl.ARRAY_TEST_3 p153 = CommunicationChannel.new_ARRAY_TEST_3();
		PH.setPack(p153);
		p153.v_SET((char) 25);
		p153.ar_u32_SET(new long[]{3086105250L, 757137702L, 3366615865L, 341011648L}, 0);
		CommunicationChannel.instance.send(p153);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_ARRAY_TEST_4.add((src, ph, pack) ->
		{
			assert (pack.v_GET() == (char) 111);
			assert (Arrays.equals(pack.ar_u32_GET(), new long[]{1085313024L, 3562812380L, 3439170901L, 9102950L}));
		});
		GroundControl.ARRAY_TEST_4 p154 = CommunicationChannel.new_ARRAY_TEST_4();
		PH.setPack(p154);
		p154.ar_u32_SET(new long[]{1085313024L, 3562812380L, 3439170901L, 9102950L}, 0);
		p154.v_SET((char) 111);
		CommunicationChannel.instance.send(p154);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_ARRAY_TEST_5.add((src, ph, pack) ->
		{
			assert (pack.c2_LEN(ph) == 3);
			assert (pack.c2_TRY(ph).equals("tZm"));
			assert (pack.c1_LEN(ph) == 4);
			assert (pack.c1_TRY(ph).equals("Alwb"));
		});
		GroundControl.ARRAY_TEST_5 p155 = CommunicationChannel.new_ARRAY_TEST_5();
		PH.setPack(p155);
		p155.c2_SET("tZm", PH);
		p155.c1_SET("Alwb", PH);
		CommunicationChannel.instance.send(p155);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_ARRAY_TEST_6.add((src, ph, pack) ->
		{
			assert (Arrays.equals(pack.ar_i8_GET(), new byte[]{(byte) 88, (byte) -120}));
			assert (Arrays.equals(pack.ar_i32_GET(), new int[]{-1602897685, -1374779819}));
			assert (pack.v1_GET() == (char) 150);
			assert (Arrays.equals(pack.ar_i16_GET(), new short[]{(short) 3529, (short) 24022}));
			assert (pack.ar_c_LEN(ph) == 26);
			assert (pack.ar_c_TRY(ph).equals("mwtiwmlptlBeaazhqyhtvzoxxu"));
			assert (Arrays.equals(pack.ar_u32_GET(), new long[]{1078752401L, 3180973734L}));
			assert (Arrays.equals(pack.ar_d_GET(), new double[]{8.970452888824314E307, -1.751158666317643E308}));
			assert (Arrays.equals(pack.ar_u8_GET(), new char[]{(char) 52, (char) 153}));
			assert (pack.v3_GET() == 3725269992L);
			assert (pack.v2_GET() == (char) 11651);
			assert (Arrays.equals(pack.ar_u16_GET(), new char[]{(char) 9347, (char) 41035}));
			assert (Arrays.equals(pack.ar_f_GET(), new float[]{-3.0196892E38F, 2.933472E38F}));
		});
		GroundControl.ARRAY_TEST_6 p156 = CommunicationChannel.new_ARRAY_TEST_6();
		PH.setPack(p156);
		p156.ar_i16_SET(new short[]{(short) 3529, (short) 24022}, 0);
		p156.ar_f_SET(new float[]{-3.0196892E38F, 2.933472E38F}, 0);
		p156.ar_u32_SET(new long[]{1078752401L, 3180973734L}, 0);
		p156.v3_SET(3725269992L);
		p156.v2_SET((char) 11651);
		p156.ar_i8_SET(new byte[]{(byte) 88, (byte) -120}, 0);
		p156.ar_c_SET("mwtiwmlptlBeaazhqyhtvzoxxu", PH);
		p156.ar_u8_SET(new char[]{(char) 52, (char) 153}, 0);
		p156.ar_i32_SET(new int[]{-1602897685, -1374779819}, 0);
		p156.ar_d_SET(new double[]{8.970452888824314E307, -1.751158666317643E308}, 0);
		p156.ar_u16_SET(new char[]{(char) 9347, (char) 41035}, 0);
		p156.v1_SET((char) 150);
		CommunicationChannel.instance.send(p156);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_ARRAY_TEST_7.add((src, ph, pack) ->
		{
			assert (pack.ar_c_LEN(ph) == 8);
			assert (pack.ar_c_TRY(ph).equals("xcvupezm"));
			assert (Arrays.equals(pack.ar_i8_GET(), new byte[]{(byte) 122, (byte) -64}));
			assert (Arrays.equals(pack.ar_i16_GET(), new short[]{(short) -2070, (short) -31017}));
			assert (Arrays.equals(pack.ar_u32_GET(), new long[]{1796737284L, 2380117858L}));
			assert (Arrays.equals(pack.ar_u16_GET(), new char[]{(char) 45823, (char) 38940}));
			assert (Arrays.equals(pack.ar_d_GET(), new double[]{1.7532981334093267E307, -8.139081728352248E307}));
			assert (Arrays.equals(pack.ar_i32_GET(), new int[]{-577781166, 1678578012}));
			assert (Arrays.equals(pack.ar_u8_GET(), new char[]{(char) 217, (char) 215}));
			assert (Arrays.equals(pack.ar_f_GET(), new float[]{-2.6012714E38F, 3.0359114E38F}));
		});
		GroundControl.ARRAY_TEST_7 p157 = CommunicationChannel.new_ARRAY_TEST_7();
		PH.setPack(p157);
		p157.ar_f_SET(new float[]{-2.6012714E38F, 3.0359114E38F}, 0);
		p157.ar_d_SET(new double[]{1.7532981334093267E307, -8.139081728352248E307}, 0);
		p157.ar_i32_SET(new int[]{-577781166, 1678578012}, 0);
		p157.ar_i8_SET(new byte[]{(byte) 122, (byte) -64}, 0);
		p157.ar_c_SET("xcvupezm", PH);
		p157.ar_i16_SET(new short[]{(short) -2070, (short) -31017}, 0);
		p157.ar_u16_SET(new char[]{(char) 45823, (char) 38940}, 0);
		p157.ar_u32_SET(new long[]{1796737284L, 2380117858L}, 0);
		p157.ar_u8_SET(new char[]{(char) 217, (char) 215}, 0);
		CommunicationChannel.instance.send(p157);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_ARRAY_TEST_8.add((src, ph, pack) ->
		{
			assert (pack.v3_GET() == 829695025L);
			assert (Arrays.equals(pack.ar_u16_GET(), new char[]{(char) 9983, (char) 53928}));
			assert (Arrays.equals(pack.ar_d_GET(), new double[]{1.785560398195192E308, -4.494830182600556E307}));
		});
		GroundControl.ARRAY_TEST_8 p158 = CommunicationChannel.new_ARRAY_TEST_8();
		PH.setPack(p158);
		p158.v3_SET(829695025L);
		p158.ar_d_SET(new double[]{1.785560398195192E308, -4.494830182600556E307}, 0);
		p158.ar_u16_SET(new char[]{(char) 9983, (char) 53928}, 0);
		CommunicationChannel.instance.send(p158);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_ESTIMATOR_STATUS.add((src, ph, pack) ->
		{
			assert (pack.pos_vert_ratio_GET() == 1.1806867E38F);
			assert (pack.flags_GET() == ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_HORIZ);
			assert (pack.hagl_ratio_GET() == 2.859588E38F);
			assert (pack.pos_vert_accuracy_GET() == -3.3492448E38F);
			assert (pack.time_usec_GET() == 1708637432516279977L);
			assert (pack.mag_ratio_GET() == -1.5320572E38F);
			assert (pack.vel_ratio_GET() == 2.4804874E38F);
			assert (pack.pos_horiz_accuracy_GET() == -3.8919566E37F);
			assert (pack.pos_horiz_ratio_GET() == -1.7639685E38F);
			assert (pack.tas_ratio_GET() == 1.7130385E38F);
		});
		GroundControl.ESTIMATOR_STATUS p230 = CommunicationChannel.new_ESTIMATOR_STATUS();
		PH.setPack(p230);
		p230.mag_ratio_SET(-1.5320572E38F);
		p230.vel_ratio_SET(2.4804874E38F);
		p230.tas_ratio_SET(1.7130385E38F);
		p230.pos_vert_accuracy_SET(-3.3492448E38F);
		p230.pos_horiz_ratio_SET(-1.7639685E38F);
		p230.hagl_ratio_SET(2.859588E38F);
		p230.pos_vert_ratio_SET(1.1806867E38F);
		p230.time_usec_SET(1708637432516279977L);
		p230.pos_horiz_accuracy_SET(-3.8919566E37F);
		p230.flags_SET(ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_HORIZ);
		CommunicationChannel.instance.send(p230);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_WIND_COV.add((src, ph, pack) ->
		{
			assert (pack.horiz_accuracy_GET() == -1.9357653E38F);
			assert (pack.wind_y_GET() == 3.0500097E38F);
			assert (pack.time_usec_GET() == 9067518619027752932L);
			assert (pack.wind_z_GET() == 3.1450411E38F);
			assert (pack.vert_accuracy_GET() == 2.1879043E37F);
			assert (pack.var_vert_GET() == 2.6938451E38F);
			assert (pack.var_horiz_GET() == -2.7473473E38F);
			assert (pack.wind_alt_GET() == 5.5345757E37F);
			assert (pack.wind_x_GET() == -2.537558E38F);
		});
		GroundControl.WIND_COV p231 = CommunicationChannel.new_WIND_COV();
		PH.setPack(p231);
		p231.time_usec_SET(9067518619027752932L);
		p231.wind_x_SET(-2.537558E38F);
		p231.wind_z_SET(3.1450411E38F);
		p231.vert_accuracy_SET(2.1879043E37F);
		p231.wind_y_SET(3.0500097E38F);
		p231.var_vert_SET(2.6938451E38F);
		p231.var_horiz_SET(-2.7473473E38F);
		p231.horiz_accuracy_SET(-1.9357653E38F);
		p231.wind_alt_SET(5.5345757E37F);
		CommunicationChannel.instance.send(p231);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_GPS_INPUT.add((src, ph, pack) ->
		{
			assert (pack.fix_type_GET() == (char) 148);
			assert (pack.time_usec_GET() == 1067842292605053957L);
			assert (pack.time_week_GET() == (char) 12265);
			assert (pack.gps_id_GET() == (char) 35);
			assert (pack.satellites_visible_GET() == (char) 49);
			assert (pack.hdop_GET() == -3.0070508E38F);
			assert (pack.ignore_flags_GET() == GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_ALT);
			assert (pack.vert_accuracy_GET() == -2.5989145E38F);
			assert (pack.vn_GET() == -2.5383748E38F);
			assert (pack.vd_GET() == -1.1400838E38F);
			assert (pack.lon_GET() == -1683975600);
			assert (pack.speed_accuracy_GET() == 1.427824E38F);
			assert (pack.ve_GET() == 1.7699145E38F);
			assert (pack.time_week_ms_GET() == 3405644499L);
			assert (pack.lat_GET() == -130661498);
			assert (pack.vdop_GET() == -2.2797838E38F);
			assert (pack.alt_GET() == -1.0105653E38F);
			assert (pack.horiz_accuracy_GET() == 2.6352243E38F);
		});
		GroundControl.GPS_INPUT p232 = CommunicationChannel.new_GPS_INPUT();
		PH.setPack(p232);
		p232.time_week_ms_SET(3405644499L);
		p232.alt_SET(-1.0105653E38F);
		p232.vd_SET(-1.1400838E38F);
		p232.satellites_visible_SET((char) 49);
		p232.gps_id_SET((char) 35);
		p232.vdop_SET(-2.2797838E38F);
		p232.speed_accuracy_SET(1.427824E38F);
		p232.time_week_SET((char) 12265);
		p232.ignore_flags_SET(GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_ALT);
		p232.vert_accuracy_SET(-2.5989145E38F);
		p232.horiz_accuracy_SET(2.6352243E38F);
		p232.time_usec_SET(1067842292605053957L);
		p232.lat_SET(-130661498);
		p232.lon_SET(-1683975600);
		p232.vn_SET(-2.5383748E38F);
		p232.ve_SET(1.7699145E38F);
		p232.fix_type_SET((char) 148);
		p232.hdop_SET(-3.0070508E38F);
		CommunicationChannel.instance.send(p232);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_GPS_RTCM_DATA.add((src, ph, pack) ->
		{
			assert (Arrays.equals(pack.data__GET(), new char[]{(char) 213, (char) 53, (char) 247, (char) 135, (char) 81, (char) 20, (char) 7, (char) 151, (char) 32, (char) 9, (char) 176, (char) 197, (char) 85, (char) 142, (char) 132, (char) 202, (char) 57, (char) 109, (char) 146, (char) 34, (char) 131, (char) 69, (char) 169, (char) 35, (char) 43, (char) 116, (char) 65, (char) 5, (char) 167, (char) 82, (char) 184, (char) 160, (char) 228, (char) 254, (char) 127, (char) 202, (char) 28, (char) 77, (char) 153, (char) 32, (char) 181, (char) 9, (char) 37, (char) 90, (char) 145, (char) 66, (char) 49, (char) 80, (char) 205, (char) 171, (char) 223, (char) 38, (char) 75, (char) 30, (char) 221, (char) 168, (char) 126, (char) 30, (char) 150, (char) 193, (char) 199, (char) 217, (char) 250, (char) 153, (char) 221, (char) 249, (char) 71, (char) 42, (char) 40, (char) 211, (char) 29, (char) 168, (char) 222, (char) 134, (char) 57, (char) 48, (char) 30, (char) 158, (char) 227, (char) 156, (char) 96, (char) 188, (char) 9, (char) 44, (char) 196, (char) 8, (char) 209, (char) 103, (char) 162, (char) 215, (char) 170, (char) 61, (char) 102, (char) 191, (char) 232, (char) 188, (char) 87, (char) 2, (char) 255, (char) 100, (char) 252, (char) 92, (char) 177, (char) 98, (char) 161, (char) 85, (char) 18, (char) 212, (char) 13, (char) 157, (char) 44, (char) 151, (char) 210, (char) 200, (char) 21, (char) 68, (char) 222, (char) 166, (char) 56, (char) 13, (char) 138, (char) 0, (char) 98, (char) 16, (char) 19, (char) 101, (char) 234, (char) 52, (char) 6, (char) 76, (char) 45, (char) 54, (char) 125, (char) 170, (char) 244, (char) 243, (char) 59, (char) 3, (char) 76, (char) 106, (char) 130, (char) 212, (char) 91, (char) 18, (char) 27, (char) 105, (char) 251, (char) 109, (char) 233, (char) 140, (char) 49, (char) 200, (char) 213, (char) 16, (char) 107, (char) 219, (char) 222, (char) 232, (char) 196, (char) 117, (char) 235, (char) 154, (char) 206, (char) 111, (char) 14, (char) 152, (char) 66, (char) 174, (char) 214, (char) 50, (char) 167, (char) 35, (char) 125, (char) 160, (char) 127, (char) 60, (char) 89, (char) 231, (char) 34, (char) 197}));
			assert (pack.len_GET() == (char) 226);
			assert (pack.flags_GET() == (char) 73);
		});
		GroundControl.GPS_RTCM_DATA p233 = CommunicationChannel.new_GPS_RTCM_DATA();
		PH.setPack(p233);
		p233.data__SET(new char[]{(char) 213, (char) 53, (char) 247, (char) 135, (char) 81, (char) 20, (char) 7, (char) 151, (char) 32, (char) 9, (char) 176, (char) 197, (char) 85, (char) 142, (char) 132, (char) 202, (char) 57, (char) 109, (char) 146, (char) 34, (char) 131, (char) 69, (char) 169, (char) 35, (char) 43, (char) 116, (char) 65, (char) 5, (char) 167, (char) 82, (char) 184, (char) 160, (char) 228, (char) 254, (char) 127, (char) 202, (char) 28, (char) 77, (char) 153, (char) 32, (char) 181, (char) 9, (char) 37, (char) 90, (char) 145, (char) 66, (char) 49, (char) 80, (char) 205, (char) 171, (char) 223, (char) 38, (char) 75, (char) 30, (char) 221, (char) 168, (char) 126, (char) 30, (char) 150, (char) 193, (char) 199, (char) 217, (char) 250, (char) 153, (char) 221, (char) 249, (char) 71, (char) 42, (char) 40, (char) 211, (char) 29, (char) 168, (char) 222, (char) 134, (char) 57, (char) 48, (char) 30, (char) 158, (char) 227, (char) 156, (char) 96, (char) 188, (char) 9, (char) 44, (char) 196, (char) 8, (char) 209, (char) 103, (char) 162, (char) 215, (char) 170, (char) 61, (char) 102, (char) 191, (char) 232, (char) 188, (char) 87, (char) 2, (char) 255, (char) 100, (char) 252, (char) 92, (char) 177, (char) 98, (char) 161, (char) 85, (char) 18, (char) 212, (char) 13, (char) 157, (char) 44, (char) 151, (char) 210, (char) 200, (char) 21, (char) 68, (char) 222, (char) 166, (char) 56, (char) 13, (char) 138, (char) 0, (char) 98, (char) 16, (char) 19, (char) 101, (char) 234, (char) 52, (char) 6, (char) 76, (char) 45, (char) 54, (char) 125, (char) 170, (char) 244, (char) 243, (char) 59, (char) 3, (char) 76, (char) 106, (char) 130, (char) 212, (char) 91, (char) 18, (char) 27, (char) 105, (char) 251, (char) 109, (char) 233, (char) 140, (char) 49, (char) 200, (char) 213, (char) 16, (char) 107, (char) 219, (char) 222, (char) 232, (char) 196, (char) 117, (char) 235, (char) 154, (char) 206, (char) 111, (char) 14, (char) 152, (char) 66, (char) 174, (char) 214, (char) 50, (char) 167, (char) 35, (char) 125, (char) 160, (char) 127, (char) 60, (char) 89, (char) 231, (char) 34, (char) 197}, 0);
		p233.flags_SET((char) 73);
		p233.len_SET((char) 226);
		CommunicationChannel.instance.send(p233);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_HIGH_LATENCY.add((src, ph, pack) ->
		{
			assert (pack.groundspeed_GET() == (char) 29);
			assert (pack.latitude_GET() == 357780401);
			assert (pack.longitude_GET() == -425772439);
			assert (pack.gps_nsat_GET() == (char) 65);
			assert (pack.battery_remaining_GET() == (char) 108);
			assert (pack.altitude_sp_GET() == (short) 29420);
			assert (pack.airspeed_sp_GET() == (char) 241);
			assert (pack.gps_fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_STATIC);
			assert (pack.wp_num_GET() == (char) 247);
			assert (pack.pitch_GET() == (short) 26972);
			assert (pack.throttle_GET() == (byte) 101);
			assert (pack.wp_distance_GET() == (char) 59813);
			assert (pack.airspeed_GET() == (char) 213);
			assert (pack.temperature_GET() == (byte) -95);
			assert (pack.failsafe_GET() == (char) 82);
			assert (pack.climb_rate_GET() == (byte) -25);
			assert (pack.roll_GET() == (short) -20120);
			assert (pack.temperature_air_GET() == (byte) -88);
			assert (pack.custom_mode_GET() == 4231648329L);
			assert (pack.base_mode_GET() == MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED);
			assert (pack.heading_sp_GET() == (short) 10095);
			assert (pack.heading_GET() == (char) 26879);
			assert (pack.landed_state_GET() == MAV_LANDED_STATE.MAV_LANDED_STATE_IN_AIR);
			assert (pack.altitude_amsl_GET() == (short) 22942);
		});
		GroundControl.HIGH_LATENCY p234 = CommunicationChannel.new_HIGH_LATENCY();
		PH.setPack(p234);
		p234.gps_nsat_SET((char) 65);
		p234.landed_state_SET(MAV_LANDED_STATE.MAV_LANDED_STATE_IN_AIR);
		p234.groundspeed_SET((char) 29);
		p234.airspeed_sp_SET((char) 241);
		p234.temperature_air_SET((byte) -88);
		p234.airspeed_SET((char) 213);
		p234.longitude_SET(-425772439);
		p234.temperature_SET((byte) -95);
		p234.failsafe_SET((char) 82);
		p234.gps_fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_STATIC);
		p234.heading_SET((char) 26879);
		p234.wp_num_SET((char) 247);
		p234.altitude_amsl_SET((short) 22942);
		p234.custom_mode_SET(4231648329L);
		p234.altitude_sp_SET((short) 29420);
		p234.base_mode_SET(MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED);
		p234.pitch_SET((short) 26972);
		p234.wp_distance_SET((char) 59813);
		p234.latitude_SET(357780401);
		p234.roll_SET((short) -20120);
		p234.battery_remaining_SET((char) 108);
		p234.climb_rate_SET((byte) -25);
		p234.throttle_SET((byte) 101);
		p234.heading_sp_SET((short) 10095);
		CommunicationChannel.instance.send(p234);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_VIBRATION.add((src, ph, pack) ->
		{
			assert (pack.clipping_2_GET() == 773804876L);
			assert (pack.vibration_z_GET() == 1.715158E38F);
			assert (pack.clipping_0_GET() == 3912045187L);
			assert (pack.time_usec_GET() == 1337169927913452648L);
			assert (pack.clipping_1_GET() == 3077412773L);
			assert (pack.vibration_y_GET() == 3.3427062E38F);
			assert (pack.vibration_x_GET() == 1.3676396E37F);
		});
		GroundControl.VIBRATION p241 = CommunicationChannel.new_VIBRATION();
		PH.setPack(p241);
		p241.clipping_1_SET(3077412773L);
		p241.time_usec_SET(1337169927913452648L);
		p241.clipping_2_SET(773804876L);
		p241.vibration_x_SET(1.3676396E37F);
		p241.vibration_y_SET(3.3427062E38F);
		p241.clipping_0_SET(3912045187L);
		p241.vibration_z_SET(1.715158E38F);
		CommunicationChannel.instance.send(p241);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_HOME_POSITION.add((src, ph, pack) ->
		{
			assert (pack.approach_y_GET() == -7.2938785E37F);
			assert (pack.approach_z_GET() == -3.0156905E38F);
			assert (pack.y_GET() == 2.1871453E38F);
			assert (pack.x_GET() == 6.4710206E37F);
			assert (pack.altitude_GET() == -450500242);
			assert (Arrays.equals(pack.q_GET(), new float[]{2.0425451E38F, -7.984717E37F, 1.3831028E38F, -6.550892E37F}));
			assert (pack.longitude_GET() == 593720215);
			assert (pack.z_GET() == 3.0956225E37F);
			assert (pack.approach_x_GET() == 6.114242E37F);
			assert (pack.latitude_GET() == 1315095746);
			assert (pack.time_usec_TRY(ph) == 4936350953437947334L);
		});
		GroundControl.HOME_POSITION p242 = CommunicationChannel.new_HOME_POSITION();
		PH.setPack(p242);
		p242.approach_y_SET(-7.2938785E37F);
		p242.latitude_SET(1315095746);
		p242.approach_z_SET(-3.0156905E38F);
		p242.q_SET(new float[]{2.0425451E38F, -7.984717E37F, 1.3831028E38F, -6.550892E37F}, 0);
		p242.time_usec_SET(4936350953437947334L, PH);
		p242.approach_x_SET(6.114242E37F);
		p242.y_SET(2.1871453E38F);
		p242.longitude_SET(593720215);
		p242.z_SET(3.0956225E37F);
		p242.x_SET(6.4710206E37F);
		p242.altitude_SET(-450500242);
		CommunicationChannel.instance.send(p242);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_SET_HOME_POSITION.add((src, ph, pack) ->
		{
			assert (pack.target_system_GET() == (char) 183);
			assert (pack.y_GET() == 1.2453057E38F);
			assert (Arrays.equals(pack.q_GET(), new float[]{2.8483713E38F, -1.5920269E38F, 1.95761E38F, -9.798855E37F}));
			assert (pack.approach_y_GET() == -6.0771915E37F);
			assert (pack.longitude_GET() == -430789797);
			assert (pack.altitude_GET() == -808876849);
			assert (pack.latitude_GET() == 1424660280);
			assert (pack.z_GET() == -6.3709006E37F);
			assert (pack.approach_x_GET() == 2.9066044E38F);
			assert (pack.approach_z_GET() == 1.5705272E37F);
			assert (pack.x_GET() == -9.688944E37F);
			assert (pack.time_usec_TRY(ph) == 8627399076335096520L);
		});
		GroundControl.SET_HOME_POSITION p243 = CommunicationChannel.new_SET_HOME_POSITION();
		PH.setPack(p243);
		p243.altitude_SET(-808876849);
		p243.x_SET(-9.688944E37F);
		p243.approach_x_SET(2.9066044E38F);
		p243.latitude_SET(1424660280);
		p243.y_SET(1.2453057E38F);
		p243.z_SET(-6.3709006E37F);
		p243.longitude_SET(-430789797);
		p243.q_SET(new float[]{2.8483713E38F, -1.5920269E38F, 1.95761E38F, -9.798855E37F}, 0);
		p243.target_system_SET((char) 183);
		p243.time_usec_SET(8627399076335096520L, PH);
		p243.approach_z_SET(1.5705272E37F);
		p243.approach_y_SET(-6.0771915E37F);
		CommunicationChannel.instance.send(p243);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_MESSAGE_INTERVAL.add((src, ph, pack) ->
		{
			assert (pack.interval_us_GET() == -1419355810);
			assert (pack.message_id_GET() == (char) 53056);
		});
		GroundControl.MESSAGE_INTERVAL p244 = CommunicationChannel.new_MESSAGE_INTERVAL();
		PH.setPack(p244);
		p244.interval_us_SET(-1419355810);
		p244.message_id_SET((char) 53056);
		CommunicationChannel.instance.send(p244);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_EXTENDED_SYS_STATE.add((src, ph, pack) ->
		{
			assert (pack.vtol_state_GET() == MAV_VTOL_STATE.MAV_VTOL_STATE_TRANSITION_TO_MC);
			assert (pack.landed_state_GET() == MAV_LANDED_STATE.MAV_LANDED_STATE_IN_AIR);
		});
		GroundControl.EXTENDED_SYS_STATE p245 = CommunicationChannel.new_EXTENDED_SYS_STATE();
		PH.setPack(p245);
		p245.landed_state_SET(MAV_LANDED_STATE.MAV_LANDED_STATE_IN_AIR);
		p245.vtol_state_SET(MAV_VTOL_STATE.MAV_VTOL_STATE_TRANSITION_TO_MC);
		CommunicationChannel.instance.send(p245);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_ADSB_VEHICLE.add((src, ph, pack) ->
		{
			assert (pack.emitter_type_GET() == ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_GLIDER);
			assert (pack.tslc_GET() == (char) 220);
			assert (pack.altitude_GET() == -1724464572);
			assert (pack.flags_GET() == ADSB_FLAGS.ADSB_FLAGS_VALID_ALTITUDE);
			assert (pack.hor_velocity_GET() == (char) 491);
			assert (pack.callsign_LEN(ph) == 5);
			assert (pack.callsign_TRY(ph).equals("siykr"));
			assert (pack.lat_GET() == 434158422);
			assert (pack.altitude_type_GET() == ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_GEOMETRIC);
			assert (pack.ICAO_address_GET() == 145297933L);
			assert (pack.lon_GET() == 269860608);
			assert (pack.squawk_GET() == (char) 23130);
			assert (pack.ver_velocity_GET() == (short) -12871);
			assert (pack.heading_GET() == (char) 56824);
		});
		GroundControl.ADSB_VEHICLE p246 = CommunicationChannel.new_ADSB_VEHICLE();
		PH.setPack(p246);
		p246.altitude_type_SET(ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_GEOMETRIC);
		p246.emitter_type_SET(ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_GLIDER);
		p246.tslc_SET((char) 220);
		p246.altitude_SET(-1724464572);
		p246.ICAO_address_SET(145297933L);
		p246.lat_SET(434158422);
		p246.hor_velocity_SET((char) 491);
		p246.ver_velocity_SET((short) -12871);
		p246.lon_SET(269860608);
		p246.callsign_SET("siykr", PH);
		p246.squawk_SET((char) 23130);
		p246.heading_SET((char) 56824);
		p246.flags_SET(ADSB_FLAGS.ADSB_FLAGS_VALID_ALTITUDE);
		CommunicationChannel.instance.send(p246);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_COLLISION.add((src, ph, pack) ->
		{
			assert (pack.id_GET() == 2086324240L);
			assert (pack.horizontal_minimum_delta_GET() == 1.3511767E38F);
			assert (pack.time_to_minimum_delta_GET() == 2.2162117E37F);
			assert (pack.action_GET() == MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_HOVER);
			assert (pack.threat_level_GET() == MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_HIGH);
			assert (pack.src__GET() == MAV_COLLISION_SRC.MAV_COLLISION_SRC_ADSB);
			assert (pack.altitude_minimum_delta_GET() == -1.1043582E38F);
		});
		GroundControl.COLLISION p247 = CommunicationChannel.new_COLLISION();
		PH.setPack(p247);
		p247.threat_level_SET(MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_HIGH);
		p247.horizontal_minimum_delta_SET(1.3511767E38F);
		p247.altitude_minimum_delta_SET(-1.1043582E38F);
		p247.src__SET(MAV_COLLISION_SRC.MAV_COLLISION_SRC_ADSB);
		p247.time_to_minimum_delta_SET(2.2162117E37F);
		p247.action_SET(MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_HOVER);
		p247.id_SET(2086324240L);
		CommunicationChannel.instance.send(p247);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_V2_EXTENSION.add((src, ph, pack) ->
		{
			assert (pack.message_type_GET() == (char) 22895);
			assert (pack.target_network_GET() == (char) 250);
			assert (Arrays.equals(pack.payload_GET(), new char[]{(char) 35, (char) 157, (char) 185, (char) 178, (char) 54, (char) 188, (char) 194, (char) 157, (char) 196, (char) 238, (char) 75, (char) 11, (char) 113, (char) 187, (char) 224, (char) 43, (char) 4, (char) 69, (char) 56, (char) 208, (char) 77, (char) 96, (char) 82, (char) 221, (char) 12, (char) 145, (char) 248, (char) 23, (char) 184, (char) 73, (char) 34, (char) 53, (char) 36, (char) 64, (char) 172, (char) 138, (char) 18, (char) 115, (char) 182, (char) 149, (char) 49, (char) 249, (char) 79, (char) 28, (char) 191, (char) 247, (char) 23, (char) 201, (char) 221, (char) 240, (char) 105, (char) 186, (char) 172, (char) 224, (char) 2, (char) 115, (char) 195, (char) 191, (char) 48, (char) 118, (char) 72, (char) 99, (char) 179, (char) 148, (char) 11, (char) 159, (char) 59, (char) 247, (char) 105, (char) 135, (char) 127, (char) 106, (char) 170, (char) 216, (char) 38, (char) 195, (char) 205, (char) 187, (char) 201, (char) 174, (char) 123, (char) 33, (char) 50, (char) 231, (char) 253, (char) 204, (char) 135, (char) 88, (char) 6, (char) 170, (char) 5, (char) 38, (char) 209, (char) 234, (char) 90, (char) 97, (char) 173, (char) 255, (char) 57, (char) 110, (char) 231, (char) 197, (char) 120, (char) 246, (char) 208, (char) 131, (char) 153, (char) 218, (char) 79, (char) 201, (char) 47, (char) 163, (char) 143, (char) 11, (char) 154, (char) 233, (char) 57, (char) 111, (char) 14, (char) 30, (char) 30, (char) 241, (char) 1, (char) 87, (char) 13, (char) 106, (char) 72, (char) 17, (char) 141, (char) 64, (char) 146, (char) 48, (char) 114, (char) 64, (char) 75, (char) 139, (char) 65, (char) 179, (char) 191, (char) 30, (char) 67, (char) 155, (char) 141, (char) 12, (char) 144, (char) 95, (char) 92, (char) 111, (char) 20, (char) 123, (char) 74, (char) 69, (char) 253, (char) 71, (char) 75, (char) 116, (char) 84, (char) 100, (char) 239, (char) 241, (char) 102, (char) 50, (char) 3, (char) 153, (char) 78, (char) 107, (char) 180, (char) 204, (char) 150, (char) 129, (char) 29, (char) 121, (char) 49, (char) 171, (char) 99, (char) 164, (char) 146, (char) 90, (char) 48, (char) 25, (char) 161, (char) 130, (char) 31, (char) 159, (char) 95, (char) 185, (char) 241, (char) 137, (char) 191, (char) 87, (char) 180, (char) 6, (char) 153, (char) 53, (char) 48, (char) 197, (char) 154, (char) 181, (char) 106, (char) 226, (char) 40, (char) 88, (char) 31, (char) 116, (char) 206, (char) 121, (char) 240, (char) 200, (char) 101, (char) 230, (char) 93, (char) 119, (char) 49, (char) 105, (char) 94, (char) 113, (char) 174, (char) 235, (char) 11, (char) 11, (char) 3, (char) 134, (char) 163, (char) 246, (char) 147, (char) 41, (char) 61, (char) 99, (char) 36, (char) 8, (char) 238, (char) 161, (char) 85, (char) 231, (char) 208, (char) 122, (char) 161, (char) 251, (char) 141, (char) 168, (char) 108, (char) 13, (char) 230, (char) 242, (char) 68, (char) 164, (char) 239, (char) 204, (char) 70}));
			assert (pack.target_component_GET() == (char) 224);
			assert (pack.target_system_GET() == (char) 186);
		});
		GroundControl.V2_EXTENSION p248 = CommunicationChannel.new_V2_EXTENSION();
		PH.setPack(p248);
		p248.target_component_SET((char) 224);
		p248.target_system_SET((char) 186);
		p248.target_network_SET((char) 250);
		p248.payload_SET(new char[]{(char) 35, (char) 157, (char) 185, (char) 178, (char) 54, (char) 188, (char) 194, (char) 157, (char) 196, (char) 238, (char) 75, (char) 11, (char) 113, (char) 187, (char) 224, (char) 43, (char) 4, (char) 69, (char) 56, (char) 208, (char) 77, (char) 96, (char) 82, (char) 221, (char) 12, (char) 145, (char) 248, (char) 23, (char) 184, (char) 73, (char) 34, (char) 53, (char) 36, (char) 64, (char) 172, (char) 138, (char) 18, (char) 115, (char) 182, (char) 149, (char) 49, (char) 249, (char) 79, (char) 28, (char) 191, (char) 247, (char) 23, (char) 201, (char) 221, (char) 240, (char) 105, (char) 186, (char) 172, (char) 224, (char) 2, (char) 115, (char) 195, (char) 191, (char) 48, (char) 118, (char) 72, (char) 99, (char) 179, (char) 148, (char) 11, (char) 159, (char) 59, (char) 247, (char) 105, (char) 135, (char) 127, (char) 106, (char) 170, (char) 216, (char) 38, (char) 195, (char) 205, (char) 187, (char) 201, (char) 174, (char) 123, (char) 33, (char) 50, (char) 231, (char) 253, (char) 204, (char) 135, (char) 88, (char) 6, (char) 170, (char) 5, (char) 38, (char) 209, (char) 234, (char) 90, (char) 97, (char) 173, (char) 255, (char) 57, (char) 110, (char) 231, (char) 197, (char) 120, (char) 246, (char) 208, (char) 131, (char) 153, (char) 218, (char) 79, (char) 201, (char) 47, (char) 163, (char) 143, (char) 11, (char) 154, (char) 233, (char) 57, (char) 111, (char) 14, (char) 30, (char) 30, (char) 241, (char) 1, (char) 87, (char) 13, (char) 106, (char) 72, (char) 17, (char) 141, (char) 64, (char) 146, (char) 48, (char) 114, (char) 64, (char) 75, (char) 139, (char) 65, (char) 179, (char) 191, (char) 30, (char) 67, (char) 155, (char) 141, (char) 12, (char) 144, (char) 95, (char) 92, (char) 111, (char) 20, (char) 123, (char) 74, (char) 69, (char) 253, (char) 71, (char) 75, (char) 116, (char) 84, (char) 100, (char) 239, (char) 241, (char) 102, (char) 50, (char) 3, (char) 153, (char) 78, (char) 107, (char) 180, (char) 204, (char) 150, (char) 129, (char) 29, (char) 121, (char) 49, (char) 171, (char) 99, (char) 164, (char) 146, (char) 90, (char) 48, (char) 25, (char) 161, (char) 130, (char) 31, (char) 159, (char) 95, (char) 185, (char) 241, (char) 137, (char) 191, (char) 87, (char) 180, (char) 6, (char) 153, (char) 53, (char) 48, (char) 197, (char) 154, (char) 181, (char) 106, (char) 226, (char) 40, (char) 88, (char) 31, (char) 116, (char) 206, (char) 121, (char) 240, (char) 200, (char) 101, (char) 230, (char) 93, (char) 119, (char) 49, (char) 105, (char) 94, (char) 113, (char) 174, (char) 235, (char) 11, (char) 11, (char) 3, (char) 134, (char) 163, (char) 246, (char) 147, (char) 41, (char) 61, (char) 99, (char) 36, (char) 8, (char) 238, (char) 161, (char) 85, (char) 231, (char) 208, (char) 122, (char) 161, (char) 251, (char) 141, (char) 168, (char) 108, (char) 13, (char) 230, (char) 242, (char) 68, (char) 164, (char) 239, (char) 204, (char) 70}, 0);
		p248.message_type_SET((char) 22895);
		CommunicationChannel.instance.send(p248);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_MEMORY_VECT.add((src, ph, pack) ->
		{
			assert (pack.ver_GET() == (char) 10);
			assert (pack.address_GET() == (char) 47786);
			assert (pack.type_GET() == (char) 169);
			assert (Arrays.equals(pack.value_GET(), new byte[]{(byte) -99, (byte) 105, (byte) 121, (byte) -84, (byte) 1, (byte) -88, (byte) -91, (byte) -117, (byte) 18, (byte) -104, (byte) -67, (byte) 80, (byte) -108, (byte) 87, (byte) 115, (byte) -35, (byte) 84, (byte) -73, (byte) 7, (byte) 35, (byte) 13, (byte) -50, (byte) -13, (byte) 105, (byte) 87, (byte) 14, (byte) -86, (byte) -43, (byte) 48, (byte) 104, (byte) 47, (byte) -22}));
		});
		GroundControl.MEMORY_VECT p249 = CommunicationChannel.new_MEMORY_VECT();
		PH.setPack(p249);
		p249.address_SET((char) 47786);
		p249.type_SET((char) 169);
		p249.ver_SET((char) 10);
		p249.value_SET(new byte[]{(byte) -99, (byte) 105, (byte) 121, (byte) -84, (byte) 1, (byte) -88, (byte) -91, (byte) -117, (byte) 18, (byte) -104, (byte) -67, (byte) 80, (byte) -108, (byte) 87, (byte) 115, (byte) -35, (byte) 84, (byte) -73, (byte) 7, (byte) 35, (byte) 13, (byte) -50, (byte) -13, (byte) 105, (byte) 87, (byte) 14, (byte) -86, (byte) -43, (byte) 48, (byte) 104, (byte) 47, (byte) -22}, 0);
		CommunicationChannel.instance.send(p249);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_DEBUG_VECT.add((src, ph, pack) ->
		{
			assert (pack.time_usec_GET() == 1209467249104461788L);
			assert (pack.x_GET() == 1.3086902E36F);
			assert (pack.name_LEN(ph) == 5);
			assert (pack.name_TRY(ph).equals("axrzx"));
			assert (pack.z_GET() == 1.3553858E38F);
			assert (pack.y_GET() == -2.8871562E38F);
		});
		GroundControl.DEBUG_VECT p250 = CommunicationChannel.new_DEBUG_VECT();
		PH.setPack(p250);
		p250.name_SET("axrzx", PH);
		p250.time_usec_SET(1209467249104461788L);
		p250.z_SET(1.3553858E38F);
		p250.y_SET(-2.8871562E38F);
		p250.x_SET(1.3086902E36F);
		CommunicationChannel.instance.send(p250);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_NAMED_VALUE_FLOAT.add((src, ph, pack) ->
		{
			assert (pack.time_boot_ms_GET() == 3950883865L);
			assert (pack.value_GET() == -8.52193E37F);
			assert (pack.name_LEN(ph) == 5);
			assert (pack.name_TRY(ph).equals("Jzlfq"));
		});
		GroundControl.NAMED_VALUE_FLOAT p251 = CommunicationChannel.new_NAMED_VALUE_FLOAT();
		PH.setPack(p251);
		p251.time_boot_ms_SET(3950883865L);
		p251.value_SET(-8.52193E37F);
		p251.name_SET("Jzlfq", PH);
		CommunicationChannel.instance.send(p251);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_NAMED_VALUE_INT.add((src, ph, pack) ->
		{
			assert (pack.time_boot_ms_GET() == 1211186238L);
			assert (pack.value_GET() == -344113911);
			assert (pack.name_LEN(ph) == 1);
			assert (pack.name_TRY(ph).equals("u"));
		});
		GroundControl.NAMED_VALUE_INT p252 = CommunicationChannel.new_NAMED_VALUE_INT();
		PH.setPack(p252);
		p252.name_SET("u", PH);
		p252.value_SET(-344113911);
		p252.time_boot_ms_SET(1211186238L);
		CommunicationChannel.instance.send(p252);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_STATUSTEXT.add((src, ph, pack) ->
		{
			assert (pack.severity_GET() == MAV_SEVERITY.MAV_SEVERITY_DEBUG);
			assert (pack.text_LEN(ph) == 9);
			assert (pack.text_TRY(ph).equals("jtecfqulj"));
		});
		GroundControl.STATUSTEXT p253 = CommunicationChannel.new_STATUSTEXT();
		PH.setPack(p253);
		p253.severity_SET(MAV_SEVERITY.MAV_SEVERITY_DEBUG);
		p253.text_SET("jtecfqulj", PH);
		CommunicationChannel.instance.send(p253);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_DEBUG.add((src, ph, pack) ->
		{
			assert (pack.value_GET() == 3.3004178E38F);
			assert (pack.time_boot_ms_GET() == 2121027354L);
			assert (pack.ind_GET() == (char) 244);
		});
		GroundControl.DEBUG p254 = CommunicationChannel.new_DEBUG();
		PH.setPack(p254);
		p254.time_boot_ms_SET(2121027354L);
		p254.ind_SET((char) 244);
		p254.value_SET(3.3004178E38F);
		CommunicationChannel.instance.send(p254);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_SETUP_SIGNING.add((src, ph, pack) ->
		{
			assert (pack.target_component_GET() == (char) 215);
			assert (pack.target_system_GET() == (char) 137);
			assert (Arrays.equals(pack.secret_key_GET(), new char[]{(char) 161, (char) 66, (char) 15, (char) 235, (char) 210, (char) 90, (char) 240, (char) 85, (char) 66, (char) 14, (char) 79, (char) 122, (char) 170, (char) 171, (char) 176, (char) 213, (char) 179, (char) 116, (char) 163, (char) 157, (char) 178, (char) 141, (char) 218, (char) 247, (char) 107, (char) 211, (char) 88, (char) 110, (char) 130, (char) 13, (char) 65, (char) 143}));
			assert (pack.initial_timestamp_GET() == 4461306874135918490L);
		});
		GroundControl.SETUP_SIGNING p256 = CommunicationChannel.new_SETUP_SIGNING();
		PH.setPack(p256);
		p256.target_component_SET((char) 215);
		p256.target_system_SET((char) 137);
		p256.initial_timestamp_SET(4461306874135918490L);
		p256.secret_key_SET(new char[]{(char) 161, (char) 66, (char) 15, (char) 235, (char) 210, (char) 90, (char) 240, (char) 85, (char) 66, (char) 14, (char) 79, (char) 122, (char) 170, (char) 171, (char) 176, (char) 213, (char) 179, (char) 116, (char) 163, (char) 157, (char) 178, (char) 141, (char) 218, (char) 247, (char) 107, (char) 211, (char) 88, (char) 110, (char) 130, (char) 13, (char) 65, (char) 143}, 0);
		CommunicationChannel.instance.send(p256);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_BUTTON_CHANGE.add((src, ph, pack) ->
		{
			assert (pack.last_change_ms_GET() == 562751029L);
			assert (pack.state_GET() == (char) 128);
			assert (pack.time_boot_ms_GET() == 2167830116L);
		});
		GroundControl.BUTTON_CHANGE p257 = CommunicationChannel.new_BUTTON_CHANGE();
		PH.setPack(p257);
		p257.time_boot_ms_SET(2167830116L);
		p257.state_SET((char) 128);
		p257.last_change_ms_SET(562751029L);
		CommunicationChannel.instance.send(p257);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_PLAY_TUNE.add((src, ph, pack) ->
		{
			assert (pack.target_component_GET() == (char) 114);
			assert (pack.target_system_GET() == (char) 199);
			assert (pack.tune_LEN(ph) == 1);
			assert (pack.tune_TRY(ph).equals("i"));
		});
		GroundControl.PLAY_TUNE p258 = CommunicationChannel.new_PLAY_TUNE();
		PH.setPack(p258);
		p258.target_component_SET((char) 114);
		p258.target_system_SET((char) 199);
		p258.tune_SET("i", PH);
		CommunicationChannel.instance.send(p258);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_CAMERA_INFORMATION.add((src, ph, pack) ->
		{
			assert (pack.lens_id_GET() == (char) 244);
			assert (pack.flags_GET() == CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE);
			assert (pack.firmware_version_GET() == 3584922782L);
			assert (pack.resolution_h_GET() == (char) 28154);
			assert (pack.cam_definition_version_GET() == (char) 22451);
			assert (Arrays.equals(pack.model_name_GET(), new char[]{(char) 1, (char) 133, (char) 128, (char) 123, (char) 48, (char) 137, (char) 164, (char) 14, (char) 218, (char) 185, (char) 210, (char) 86, (char) 172, (char) 48, (char) 40, (char) 47, (char) 175, (char) 18, (char) 201, (char) 241, (char) 232, (char) 249, (char) 211, (char) 206, (char) 63, (char) 20, (char) 89, (char) 229, (char) 116, (char) 156, (char) 233, (char) 236}));
			assert (pack.sensor_size_h_GET() == -1.2086124E38F);
			assert (pack.cam_definition_uri_LEN(ph) == 79);
			assert (pack.cam_definition_uri_TRY(ph).equals("jwetmzPuIPsqadnbewuZhlmawwdvhjqhaouobuywrupQkAkcyleSkqrvhbybBqararineiybkZywyfn"));
			assert (pack.time_boot_ms_GET() == 3381698840L);
			assert (Arrays.equals(pack.vendor_name_GET(), new char[]{(char) 129, (char) 167, (char) 57, (char) 75, (char) 76, (char) 193, (char) 231, (char) 124, (char) 119, (char) 156, (char) 100, (char) 176, (char) 62, (char) 54, (char) 86, (char) 204, (char) 223, (char) 178, (char) 231, (char) 165, (char) 157, (char) 82, (char) 69, (char) 241, (char) 246, (char) 169, (char) 80, (char) 85, (char) 119, (char) 103, (char) 72, (char) 220}));
			assert (pack.focal_length_GET() == -2.8501231E38F);
			assert (pack.sensor_size_v_GET() == -1.7566106E38F);
			assert (pack.resolution_v_GET() == (char) 15230);
		});
		GroundControl.CAMERA_INFORMATION p259 = CommunicationChannel.new_CAMERA_INFORMATION();
		PH.setPack(p259);
		p259.time_boot_ms_SET(3381698840L);
		p259.resolution_v_SET((char) 15230);
		p259.cam_definition_version_SET((char) 22451);
		p259.sensor_size_v_SET(-1.7566106E38F);
		p259.focal_length_SET(-2.8501231E38F);
		p259.resolution_h_SET((char) 28154);
		p259.firmware_version_SET(3584922782L);
		p259.lens_id_SET((char) 244);
		p259.sensor_size_h_SET(-1.2086124E38F);
		p259.vendor_name_SET(new char[]{(char) 129, (char) 167, (char) 57, (char) 75, (char) 76, (char) 193, (char) 231, (char) 124, (char) 119, (char) 156, (char) 100, (char) 176, (char) 62, (char) 54, (char) 86, (char) 204, (char) 223, (char) 178, (char) 231, (char) 165, (char) 157, (char) 82, (char) 69, (char) 241, (char) 246, (char) 169, (char) 80, (char) 85, (char) 119, (char) 103, (char) 72, (char) 220}, 0);
		p259.flags_SET(CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE);
		p259.model_name_SET(new char[]{(char) 1, (char) 133, (char) 128, (char) 123, (char) 48, (char) 137, (char) 164, (char) 14, (char) 218, (char) 185, (char) 210, (char) 86, (char) 172, (char) 48, (char) 40, (char) 47, (char) 175, (char) 18, (char) 201, (char) 241, (char) 232, (char) 249, (char) 211, (char) 206, (char) 63, (char) 20, (char) 89, (char) 229, (char) 116, (char) 156, (char) 233, (char) 236}, 0);
		p259.cam_definition_uri_SET("jwetmzPuIPsqadnbewuZhlmawwdvhjqhaouobuywrupQkAkcyleSkqrvhbybBqararineiybkZywyfn", PH);
		CommunicationChannel.instance.send(p259);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_CAMERA_SETTINGS.add((src, ph, pack) ->
		{
			assert (pack.mode_id_GET() == CAMERA_MODE.CAMERA_MODE_IMAGE);
			assert (pack.time_boot_ms_GET() == 1053954886L);
		});
		GroundControl.CAMERA_SETTINGS p260 = CommunicationChannel.new_CAMERA_SETTINGS();
		PH.setPack(p260);
		p260.time_boot_ms_SET(1053954886L);
		p260.mode_id_SET(CAMERA_MODE.CAMERA_MODE_IMAGE);
		CommunicationChannel.instance.send(p260);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_STORAGE_INFORMATION.add((src, ph, pack) ->
		{
			assert (pack.used_capacity_GET() == 3.0694097E38F);
			assert (pack.time_boot_ms_GET() == 2223826036L);
			assert (pack.storage_count_GET() == (char) 123);
			assert (pack.storage_id_GET() == (char) 250);
			assert (pack.status_GET() == (char) 120);
			assert (pack.total_capacity_GET() == -2.4587665E37F);
			assert (pack.available_capacity_GET() == -3.1441903E38F);
			assert (pack.write_speed_GET() == 3.2545735E38F);
			assert (pack.read_speed_GET() == 8.81293E37F);
		});
		GroundControl.STORAGE_INFORMATION p261 = CommunicationChannel.new_STORAGE_INFORMATION();
		PH.setPack(p261);
		p261.time_boot_ms_SET(2223826036L);
		p261.read_speed_SET(8.81293E37F);
		p261.available_capacity_SET(-3.1441903E38F);
		p261.total_capacity_SET(-2.4587665E37F);
		p261.status_SET((char) 120);
		p261.used_capacity_SET(3.0694097E38F);
		p261.write_speed_SET(3.2545735E38F);
		p261.storage_count_SET((char) 123);
		p261.storage_id_SET((char) 250);
		CommunicationChannel.instance.send(p261);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_CAMERA_CAPTURE_STATUS.add((src, ph, pack) ->
		{
			assert (pack.time_boot_ms_GET() == 398671156L);
			assert (pack.image_interval_GET() == 2.9205773E38F);
			assert (pack.available_capacity_GET() == 1.5118708E38F);
			assert (pack.video_status_GET() == (char) 254);
			assert (pack.image_status_GET() == (char) 248);
			assert (pack.recording_time_ms_GET() == 259171222L);
		});
		GroundControl.CAMERA_CAPTURE_STATUS p262 = CommunicationChannel.new_CAMERA_CAPTURE_STATUS();
		PH.setPack(p262);
		p262.image_interval_SET(2.9205773E38F);
		p262.time_boot_ms_SET(398671156L);
		p262.video_status_SET((char) 254);
		p262.image_status_SET((char) 248);
		p262.recording_time_ms_SET(259171222L);
		p262.available_capacity_SET(1.5118708E38F);
		CommunicationChannel.instance.send(p262);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_CAMERA_IMAGE_CAPTURED.add((src, ph, pack) ->
		{
			assert (pack.lon_GET() == -614617794);
			assert (pack.camera_id_GET() == (char) 243);
			assert (pack.file_url_LEN(ph) == 17);
			assert (pack.file_url_TRY(ph).equals("bdtepwyeMXmeoxhbi"));
			assert (pack.time_utc_GET() == 2916420087641239422L);
			assert (pack.relative_alt_GET() == 325121057);
			assert (pack.image_index_GET() == 1481246676);
			assert (pack.alt_GET() == -1538596574);
			assert (Arrays.equals(pack.q_GET(), new float[]{-1.831101E38F, 3.082134E38F, 3.0396629E38F, 1.0565351E38F}));
			assert (pack.time_boot_ms_GET() == 1686716240L);
			assert (pack.capture_result_GET() == (byte) -84);
			assert (pack.lat_GET() == -1527709587);
		});
		GroundControl.CAMERA_IMAGE_CAPTURED p263 = CommunicationChannel.new_CAMERA_IMAGE_CAPTURED();
		PH.setPack(p263);
		p263.lat_SET(-1527709587);
		p263.q_SET(new float[]{-1.831101E38F, 3.082134E38F, 3.0396629E38F, 1.0565351E38F}, 0);
		p263.time_utc_SET(2916420087641239422L);
		p263.image_index_SET(1481246676);
		p263.relative_alt_SET(325121057);
		p263.capture_result_SET((byte) -84);
		p263.file_url_SET("bdtepwyeMXmeoxhbi", PH);
		p263.time_boot_ms_SET(1686716240L);
		p263.alt_SET(-1538596574);
		p263.camera_id_SET((char) 243);
		p263.lon_SET(-614617794);
		CommunicationChannel.instance.send(p263);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_FLIGHT_INFORMATION.add((src, ph, pack) ->
		{
			assert (pack.takeoff_time_utc_GET() == 2944726447137101059L);
			assert (pack.time_boot_ms_GET() == 2602607126L);
			assert (pack.flight_uuid_GET() == 1789373994893040529L);
			assert (pack.arming_time_utc_GET() == 3101869595365862629L);
		});
		GroundControl.FLIGHT_INFORMATION p264 = CommunicationChannel.new_FLIGHT_INFORMATION();
		PH.setPack(p264);
		p264.arming_time_utc_SET(3101869595365862629L);
		p264.takeoff_time_utc_SET(2944726447137101059L);
		p264.time_boot_ms_SET(2602607126L);
		p264.flight_uuid_SET(1789373994893040529L);
		CommunicationChannel.instance.send(p264);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_MOUNT_ORIENTATION.add((src, ph, pack) ->
		{
			assert (pack.roll_GET() == -2.5578554E38F);
			assert (pack.pitch_GET() == 6.569799E37F);
			assert (pack.time_boot_ms_GET() == 1199250315L);
			assert (pack.yaw_GET() == 2.626967E38F);
		});
		GroundControl.MOUNT_ORIENTATION p265 = CommunicationChannel.new_MOUNT_ORIENTATION();
		PH.setPack(p265);
		p265.pitch_SET(6.569799E37F);
		p265.time_boot_ms_SET(1199250315L);
		p265.roll_SET(-2.5578554E38F);
		p265.yaw_SET(2.626967E38F);
		CommunicationChannel.instance.send(p265);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_LOGGING_DATA.add((src, ph, pack) ->
		{
			assert (pack.target_system_GET() == (char) 7);
			assert (pack.first_message_offset_GET() == (char) 232);
			assert (pack.sequence_GET() == (char) 16380);
			assert (Arrays.equals(pack.data__GET(), new char[]{(char) 207, (char) 232, (char) 234, (char) 148, (char) 149, (char) 207, (char) 211, (char) 249, (char) 99, (char) 226, (char) 48, (char) 243, (char) 221, (char) 202, (char) 136, (char) 141, (char) 148, (char) 123, (char) 98, (char) 227, (char) 225, (char) 206, (char) 226, (char) 191, (char) 67, (char) 16, (char) 211, (char) 118, (char) 208, (char) 252, (char) 116, (char) 129, (char) 104, (char) 89, (char) 104, (char) 137, (char) 130, (char) 109, (char) 32, (char) 142, (char) 29, (char) 112, (char) 62, (char) 209, (char) 186, (char) 183, (char) 242, (char) 219, (char) 41, (char) 164, (char) 221, (char) 252, (char) 113, (char) 16, (char) 68, (char) 72, (char) 229, (char) 227, (char) 97, (char) 247, (char) 177, (char) 245, (char) 229, (char) 234, (char) 119, (char) 202, (char) 171, (char) 174, (char) 216, (char) 89, (char) 85, (char) 51, (char) 67, (char) 113, (char) 146, (char) 168, (char) 119, (char) 231, (char) 216, (char) 216, (char) 132, (char) 14, (char) 86, (char) 172, (char) 205, (char) 91, (char) 150, (char) 218, (char) 84, (char) 238, (char) 80, (char) 19, (char) 240, (char) 204, (char) 134, (char) 146, (char) 19, (char) 23, (char) 45, (char) 33, (char) 233, (char) 181, (char) 124, (char) 211, (char) 130, (char) 251, (char) 227, (char) 126, (char) 206, (char) 7, (char) 135, (char) 85, (char) 87, (char) 198, (char) 92, (char) 79, (char) 177, (char) 126, (char) 67, (char) 77, (char) 227, (char) 225, (char) 210, (char) 76, (char) 203, (char) 249, (char) 176, (char) 144, (char) 63, (char) 76, (char) 207, (char) 181, (char) 161, (char) 126, (char) 84, (char) 164, (char) 244, (char) 66, (char) 99, (char) 215, (char) 215, (char) 215, (char) 78, (char) 81, (char) 209, (char) 249, (char) 78, (char) 31, (char) 30, (char) 26, (char) 244, (char) 239, (char) 85, (char) 180, (char) 31, (char) 201, (char) 220, (char) 92, (char) 51, (char) 42, (char) 110, (char) 5, (char) 156, (char) 78, (char) 26, (char) 61, (char) 213, (char) 127, (char) 173, (char) 63, (char) 156, (char) 77, (char) 204, (char) 36, (char) 51, (char) 195, (char) 228, (char) 191, (char) 117, (char) 190, (char) 212, (char) 3, (char) 97, (char) 136, (char) 51, (char) 220, (char) 190, (char) 47, (char) 200, (char) 114, (char) 17, (char) 5, (char) 132, (char) 161, (char) 25, (char) 210, (char) 87, (char) 222, (char) 221, (char) 242, (char) 167, (char) 17, (char) 157, (char) 50, (char) 251, (char) 24, (char) 123, (char) 95, (char) 146, (char) 120, (char) 189, (char) 248, (char) 30, (char) 214, (char) 48, (char) 138, (char) 226, (char) 248, (char) 199, (char) 219, (char) 52, (char) 40, (char) 177, (char) 241, (char) 229, (char) 25, (char) 174, (char) 133, (char) 45, (char) 150, (char) 224, (char) 197, (char) 197, (char) 113, (char) 25, (char) 54, (char) 193, (char) 48, (char) 165, (char) 217, (char) 10, (char) 97, (char) 62, (char) 87, (char) 125, (char) 37, (char) 40, (char) 53, (char) 199}));
			assert (pack.target_component_GET() == (char) 118);
			assert (pack.length_GET() == (char) 90);
		});
		GroundControl.LOGGING_DATA p266 = CommunicationChannel.new_LOGGING_DATA();
		PH.setPack(p266);
		p266.length_SET((char) 90);
		p266.target_component_SET((char) 118);
		p266.sequence_SET((char) 16380);
		p266.first_message_offset_SET((char) 232);
		p266.data__SET(new char[]{(char) 207, (char) 232, (char) 234, (char) 148, (char) 149, (char) 207, (char) 211, (char) 249, (char) 99, (char) 226, (char) 48, (char) 243, (char) 221, (char) 202, (char) 136, (char) 141, (char) 148, (char) 123, (char) 98, (char) 227, (char) 225, (char) 206, (char) 226, (char) 191, (char) 67, (char) 16, (char) 211, (char) 118, (char) 208, (char) 252, (char) 116, (char) 129, (char) 104, (char) 89, (char) 104, (char) 137, (char) 130, (char) 109, (char) 32, (char) 142, (char) 29, (char) 112, (char) 62, (char) 209, (char) 186, (char) 183, (char) 242, (char) 219, (char) 41, (char) 164, (char) 221, (char) 252, (char) 113, (char) 16, (char) 68, (char) 72, (char) 229, (char) 227, (char) 97, (char) 247, (char) 177, (char) 245, (char) 229, (char) 234, (char) 119, (char) 202, (char) 171, (char) 174, (char) 216, (char) 89, (char) 85, (char) 51, (char) 67, (char) 113, (char) 146, (char) 168, (char) 119, (char) 231, (char) 216, (char) 216, (char) 132, (char) 14, (char) 86, (char) 172, (char) 205, (char) 91, (char) 150, (char) 218, (char) 84, (char) 238, (char) 80, (char) 19, (char) 240, (char) 204, (char) 134, (char) 146, (char) 19, (char) 23, (char) 45, (char) 33, (char) 233, (char) 181, (char) 124, (char) 211, (char) 130, (char) 251, (char) 227, (char) 126, (char) 206, (char) 7, (char) 135, (char) 85, (char) 87, (char) 198, (char) 92, (char) 79, (char) 177, (char) 126, (char) 67, (char) 77, (char) 227, (char) 225, (char) 210, (char) 76, (char) 203, (char) 249, (char) 176, (char) 144, (char) 63, (char) 76, (char) 207, (char) 181, (char) 161, (char) 126, (char) 84, (char) 164, (char) 244, (char) 66, (char) 99, (char) 215, (char) 215, (char) 215, (char) 78, (char) 81, (char) 209, (char) 249, (char) 78, (char) 31, (char) 30, (char) 26, (char) 244, (char) 239, (char) 85, (char) 180, (char) 31, (char) 201, (char) 220, (char) 92, (char) 51, (char) 42, (char) 110, (char) 5, (char) 156, (char) 78, (char) 26, (char) 61, (char) 213, (char) 127, (char) 173, (char) 63, (char) 156, (char) 77, (char) 204, (char) 36, (char) 51, (char) 195, (char) 228, (char) 191, (char) 117, (char) 190, (char) 212, (char) 3, (char) 97, (char) 136, (char) 51, (char) 220, (char) 190, (char) 47, (char) 200, (char) 114, (char) 17, (char) 5, (char) 132, (char) 161, (char) 25, (char) 210, (char) 87, (char) 222, (char) 221, (char) 242, (char) 167, (char) 17, (char) 157, (char) 50, (char) 251, (char) 24, (char) 123, (char) 95, (char) 146, (char) 120, (char) 189, (char) 248, (char) 30, (char) 214, (char) 48, (char) 138, (char) 226, (char) 248, (char) 199, (char) 219, (char) 52, (char) 40, (char) 177, (char) 241, (char) 229, (char) 25, (char) 174, (char) 133, (char) 45, (char) 150, (char) 224, (char) 197, (char) 197, (char) 113, (char) 25, (char) 54, (char) 193, (char) 48, (char) 165, (char) 217, (char) 10, (char) 97, (char) 62, (char) 87, (char) 125, (char) 37, (char) 40, (char) 53, (char) 199}, 0);
		p266.target_system_SET((char) 7);
		CommunicationChannel.instance.send(p266);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_LOGGING_DATA_ACKED.add((src, ph, pack) ->
		{
			assert (Arrays.equals(pack.data__GET(), new char[]{(char) 34, (char) 175, (char) 23, (char) 180, (char) 247, (char) 67, (char) 28, (char) 251, (char) 149, (char) 2, (char) 25, (char) 231, (char) 61, (char) 162, (char) 79, (char) 9, (char) 220, (char) 117, (char) 192, (char) 234, (char) 152, (char) 49, (char) 93, (char) 183, (char) 62, (char) 175, (char) 85, (char) 23, (char) 240, (char) 182, (char) 43, (char) 201, (char) 35, (char) 63, (char) 180, (char) 168, (char) 45, (char) 191, (char) 31, (char) 2, (char) 177, (char) 201, (char) 254, (char) 207, (char) 144, (char) 126, (char) 196, (char) 104, (char) 206, (char) 44, (char) 64, (char) 168, (char) 21, (char) 198, (char) 16, (char) 149, (char) 243, (char) 220, (char) 139, (char) 167, (char) 225, (char) 122, (char) 216, (char) 243, (char) 125, (char) 112, (char) 90, (char) 35, (char) 242, (char) 176, (char) 133, (char) 148, (char) 7, (char) 71, (char) 4, (char) 125, (char) 193, (char) 52, (char) 212, (char) 232, (char) 212, (char) 170, (char) 91, (char) 213, (char) 107, (char) 254, (char) 244, (char) 251, (char) 75, (char) 163, (char) 3, (char) 208, (char) 173, (char) 241, (char) 204, (char) 78, (char) 112, (char) 86, (char) 49, (char) 125, (char) 130, (char) 121, (char) 152, (char) 42, (char) 43, (char) 232, (char) 92, (char) 239, (char) 255, (char) 223, (char) 57, (char) 152, (char) 59, (char) 176, (char) 43, (char) 129, (char) 114, (char) 153, (char) 36, (char) 49, (char) 47, (char) 106, (char) 241, (char) 100, (char) 191, (char) 15, (char) 77, (char) 21, (char) 170, (char) 131, (char) 2, (char) 116, (char) 224, (char) 209, (char) 73, (char) 112, (char) 161, (char) 18, (char) 67, (char) 99, (char) 27, (char) 187, (char) 87, (char) 77, (char) 73, (char) 222, (char) 153, (char) 89, (char) 47, (char) 91, (char) 74, (char) 177, (char) 115, (char) 51, (char) 126, (char) 167, (char) 67, (char) 244, (char) 183, (char) 24, (char) 44, (char) 151, (char) 248, (char) 212, (char) 31, (char) 226, (char) 62, (char) 71, (char) 141, (char) 52, (char) 219, (char) 68, (char) 178, (char) 36, (char) 25, (char) 240, (char) 135, (char) 103, (char) 18, (char) 239, (char) 4, (char) 32, (char) 98, (char) 70, (char) 2, (char) 22, (char) 222, (char) 85, (char) 43, (char) 156, (char) 81, (char) 82, (char) 48, (char) 153, (char) 208, (char) 110, (char) 81, (char) 51, (char) 61, (char) 224, (char) 208, (char) 250, (char) 145, (char) 35, (char) 94, (char) 224, (char) 129, (char) 184, (char) 121, (char) 23, (char) 228, (char) 105, (char) 231, (char) 101, (char) 75, (char) 195, (char) 175, (char) 79, (char) 221, (char) 139, (char) 30, (char) 249, (char) 3, (char) 98, (char) 11, (char) 61, (char) 167, (char) 109, (char) 235, (char) 78, (char) 106, (char) 138, (char) 161, (char) 201, (char) 243, (char) 78, (char) 152, (char) 168, (char) 175, (char) 1, (char) 33, (char) 76, (char) 240, (char) 18, (char) 134, (char) 193, (char) 205, (char) 17, (char) 87}));
			assert (pack.target_component_GET() == (char) 26);
			assert (pack.first_message_offset_GET() == (char) 15);
			assert (pack.sequence_GET() == (char) 49343);
			assert (pack.target_system_GET() == (char) 242);
			assert (pack.length_GET() == (char) 72);
		});
		GroundControl.LOGGING_DATA_ACKED p267 = CommunicationChannel.new_LOGGING_DATA_ACKED();
		PH.setPack(p267);
		p267.first_message_offset_SET((char) 15);
		p267.target_system_SET((char) 242);
		p267.length_SET((char) 72);
		p267.sequence_SET((char) 49343);
		p267.data__SET(new char[]{(char) 34, (char) 175, (char) 23, (char) 180, (char) 247, (char) 67, (char) 28, (char) 251, (char) 149, (char) 2, (char) 25, (char) 231, (char) 61, (char) 162, (char) 79, (char) 9, (char) 220, (char) 117, (char) 192, (char) 234, (char) 152, (char) 49, (char) 93, (char) 183, (char) 62, (char) 175, (char) 85, (char) 23, (char) 240, (char) 182, (char) 43, (char) 201, (char) 35, (char) 63, (char) 180, (char) 168, (char) 45, (char) 191, (char) 31, (char) 2, (char) 177, (char) 201, (char) 254, (char) 207, (char) 144, (char) 126, (char) 196, (char) 104, (char) 206, (char) 44, (char) 64, (char) 168, (char) 21, (char) 198, (char) 16, (char) 149, (char) 243, (char) 220, (char) 139, (char) 167, (char) 225, (char) 122, (char) 216, (char) 243, (char) 125, (char) 112, (char) 90, (char) 35, (char) 242, (char) 176, (char) 133, (char) 148, (char) 7, (char) 71, (char) 4, (char) 125, (char) 193, (char) 52, (char) 212, (char) 232, (char) 212, (char) 170, (char) 91, (char) 213, (char) 107, (char) 254, (char) 244, (char) 251, (char) 75, (char) 163, (char) 3, (char) 208, (char) 173, (char) 241, (char) 204, (char) 78, (char) 112, (char) 86, (char) 49, (char) 125, (char) 130, (char) 121, (char) 152, (char) 42, (char) 43, (char) 232, (char) 92, (char) 239, (char) 255, (char) 223, (char) 57, (char) 152, (char) 59, (char) 176, (char) 43, (char) 129, (char) 114, (char) 153, (char) 36, (char) 49, (char) 47, (char) 106, (char) 241, (char) 100, (char) 191, (char) 15, (char) 77, (char) 21, (char) 170, (char) 131, (char) 2, (char) 116, (char) 224, (char) 209, (char) 73, (char) 112, (char) 161, (char) 18, (char) 67, (char) 99, (char) 27, (char) 187, (char) 87, (char) 77, (char) 73, (char) 222, (char) 153, (char) 89, (char) 47, (char) 91, (char) 74, (char) 177, (char) 115, (char) 51, (char) 126, (char) 167, (char) 67, (char) 244, (char) 183, (char) 24, (char) 44, (char) 151, (char) 248, (char) 212, (char) 31, (char) 226, (char) 62, (char) 71, (char) 141, (char) 52, (char) 219, (char) 68, (char) 178, (char) 36, (char) 25, (char) 240, (char) 135, (char) 103, (char) 18, (char) 239, (char) 4, (char) 32, (char) 98, (char) 70, (char) 2, (char) 22, (char) 222, (char) 85, (char) 43, (char) 156, (char) 81, (char) 82, (char) 48, (char) 153, (char) 208, (char) 110, (char) 81, (char) 51, (char) 61, (char) 224, (char) 208, (char) 250, (char) 145, (char) 35, (char) 94, (char) 224, (char) 129, (char) 184, (char) 121, (char) 23, (char) 228, (char) 105, (char) 231, (char) 101, (char) 75, (char) 195, (char) 175, (char) 79, (char) 221, (char) 139, (char) 30, (char) 249, (char) 3, (char) 98, (char) 11, (char) 61, (char) 167, (char) 109, (char) 235, (char) 78, (char) 106, (char) 138, (char) 161, (char) 201, (char) 243, (char) 78, (char) 152, (char) 168, (char) 175, (char) 1, (char) 33, (char) 76, (char) 240, (char) 18, (char) 134, (char) 193, (char) 205, (char) 17, (char) 87}, 0);
		p267.target_component_SET((char) 26);
		CommunicationChannel.instance.send(p267);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_LOGGING_ACK.add((src, ph, pack) ->
		{
			assert (pack.target_system_GET() == (char) 111);
			assert (pack.sequence_GET() == (char) 51051);
			assert (pack.target_component_GET() == (char) 133);
		});
		GroundControl.LOGGING_ACK p268 = CommunicationChannel.new_LOGGING_ACK();
		PH.setPack(p268);
		p268.sequence_SET((char) 51051);
		p268.target_system_SET((char) 111);
		p268.target_component_SET((char) 133);
		CommunicationChannel.instance.send(p268);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_VIDEO_STREAM_INFORMATION.add((src, ph, pack) ->
		{
			assert (pack.camera_id_GET() == (char) 63);
			assert (pack.resolution_h_GET() == (char) 55618);
			assert (pack.status_GET() == (char) 58);
			assert (pack.uri_LEN(ph) == 107);
			assert (pack.uri_TRY(ph).equals("wskeeymahylryNTxlvqrcmwqchllnycvfhjgjjyqehuLwnhyjsalipmodgneaudrxbrmykKhndzkogvrhiiDjazcxnhvsyvsmyJmfnrwqqv"));
			assert (pack.rotation_GET() == (char) 58819);
			assert (pack.framerate_GET() == 1.2366582E38F);
			assert (pack.bitrate_GET() == 1737017558L);
			assert (pack.resolution_v_GET() == (char) 50730);
		});
		GroundControl.VIDEO_STREAM_INFORMATION p269 = CommunicationChannel.new_VIDEO_STREAM_INFORMATION();
		PH.setPack(p269);
		p269.resolution_v_SET((char) 50730);
		p269.resolution_h_SET((char) 55618);
		p269.framerate_SET(1.2366582E38F);
		p269.status_SET((char) 58);
		p269.bitrate_SET(1737017558L);
		p269.uri_SET("wskeeymahylryNTxlvqrcmwqchllnycvfhjgjjyqehuLwnhyjsalipmodgneaudrxbrmykKhndzkogvrhiiDjazcxnhvsyvsmyJmfnrwqqv", PH);
		p269.rotation_SET((char) 58819);
		p269.camera_id_SET((char) 63);
		CommunicationChannel.instance.send(p269);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_SET_VIDEO_STREAM_SETTINGS.add((src, ph, pack) ->
		{
			assert (pack.bitrate_GET() == 3724525233L);
			assert (pack.resolution_h_GET() == (char) 59760);
			assert (pack.rotation_GET() == (char) 33202);
			assert (pack.uri_LEN(ph) == 13);
			assert (pack.uri_TRY(ph).equals("brsblgggoqydn"));
			assert (pack.framerate_GET() == -1.5855247E38F);
			assert (pack.resolution_v_GET() == (char) 59157);
			assert (pack.target_system_GET() == (char) 248);
			assert (pack.target_component_GET() == (char) 31);
			assert (pack.camera_id_GET() == (char) 217);
		});
		GroundControl.SET_VIDEO_STREAM_SETTINGS p270 = CommunicationChannel.new_SET_VIDEO_STREAM_SETTINGS();
		PH.setPack(p270);
		p270.target_system_SET((char) 248);
		p270.bitrate_SET(3724525233L);
		p270.rotation_SET((char) 33202);
		p270.uri_SET("brsblgggoqydn", PH);
		p270.resolution_v_SET((char) 59157);
		p270.camera_id_SET((char) 217);
		p270.framerate_SET(-1.5855247E38F);
		p270.target_component_SET((char) 31);
		p270.resolution_h_SET((char) 59760);
		CommunicationChannel.instance.send(p270);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_WIFI_CONFIG_AP.add((src, ph, pack) ->
		{
			assert (pack.ssid_LEN(ph) == 28);
			assert (pack.ssid_TRY(ph).equals("LUtyNkfsxeysEdwtthjxpjnhcyue"));
			assert (pack.password_LEN(ph) == 61);
			assert (pack.password_TRY(ph).equals("tjywahHVqxJljkobGqdnwaqkvkfqiscbuzvggAdfjftctqbeyqwxwxuoPwcee"));
		});
		GroundControl.WIFI_CONFIG_AP p299 = CommunicationChannel.new_WIFI_CONFIG_AP();
		PH.setPack(p299);
		p299.password_SET("tjywahHVqxJljkobGqdnwaqkvkfqiscbuzvggAdfjftctqbeyqwxwxuoPwcee", PH);
		p299.ssid_SET("LUtyNkfsxeysEdwtthjxpjnhcyue", PH);
		CommunicationChannel.instance.send(p299);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_PROTOCOL_VERSION.add((src, ph, pack) ->
		{
			assert (pack.max_version_GET() == (char) 12553);
			assert (pack.min_version_GET() == (char) 48628);
			assert (Arrays.equals(pack.spec_version_hash_GET(), new char[]{(char) 22, (char) 225, (char) 116, (char) 35, (char) 111, (char) 178, (char) 152, (char) 125}));
			assert (pack.version_GET() == (char) 40094);
			assert (Arrays.equals(pack.library_version_hash_GET(), new char[]{(char) 12, (char) 177, (char) 34, (char) 55, (char) 39, (char) 73, (char) 159, (char) 177}));
		});
		GroundControl.PROTOCOL_VERSION p300 = CommunicationChannel.new_PROTOCOL_VERSION();
		PH.setPack(p300);
		p300.version_SET((char) 40094);
		p300.spec_version_hash_SET(new char[]{(char) 22, (char) 225, (char) 116, (char) 35, (char) 111, (char) 178, (char) 152, (char) 125}, 0);
		p300.min_version_SET((char) 48628);
		p300.max_version_SET((char) 12553);
		p300.library_version_hash_SET(new char[]{(char) 12, (char) 177, (char) 34, (char) 55, (char) 39, (char) 73, (char) 159, (char) 177}, 0);
		CommunicationChannel.instance.send(p300);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_UAVCAN_NODE_STATUS.add((src, ph, pack) ->
		{
			assert (pack.uptime_sec_GET() == 2030253570L);
			assert (pack.sub_mode_GET() == (char) 167);
			assert (pack.vendor_specific_status_code_GET() == (char) 13513);
			assert (pack.time_usec_GET() == 2633734481870273561L);
			assert (pack.health_GET() == UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_ERROR);
			assert (pack.mode_GET() == UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_SOFTWARE_UPDATE);
		});
		GroundControl.UAVCAN_NODE_STATUS p310 = CommunicationChannel.new_UAVCAN_NODE_STATUS();
		PH.setPack(p310);
		p310.health_SET(UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_ERROR);
		p310.sub_mode_SET((char) 167);
		p310.uptime_sec_SET(2030253570L);
		p310.mode_SET(UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_SOFTWARE_UPDATE);
		p310.time_usec_SET(2633734481870273561L);
		p310.vendor_specific_status_code_SET((char) 13513);
		CommunicationChannel.instance.send(p310);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_UAVCAN_NODE_INFO.add((src, ph, pack) ->
		{
			assert (pack.uptime_sec_GET() == 1987917500L);
			assert (Arrays.equals(pack.hw_unique_id_GET(), new char[]{(char) 1, (char) 10, (char) 77, (char) 69, (char) 240, (char) 126, (char) 36, (char) 118, (char) 210, (char) 85, (char) 173, (char) 84, (char) 184, (char) 206, (char) 32, (char) 245}));
			assert (pack.sw_vcs_commit_GET() == 2560281764L);
			assert (pack.hw_version_major_GET() == (char) 205);
			assert (pack.name_LEN(ph) == 50);
			assert (pack.name_TRY(ph).equals("pbacrxusaqojbfoskfbjgubwpbnpwtjsntnaqgjcavuufhrbgw"));
			assert (pack.hw_version_minor_GET() == (char) 54);
			assert (pack.time_usec_GET() == 4173083642746407909L);
			assert (pack.sw_version_major_GET() == (char) 35);
			assert (pack.sw_version_minor_GET() == (char) 59);
		});
		GroundControl.UAVCAN_NODE_INFO p311 = CommunicationChannel.new_UAVCAN_NODE_INFO();
		PH.setPack(p311);
		p311.name_SET("pbacrxusaqojbfoskfbjgubwpbnpwtjsntnaqgjcavuufhrbgw", PH);
		p311.sw_vcs_commit_SET(2560281764L);
		p311.sw_version_major_SET((char) 35);
		p311.hw_version_major_SET((char) 205);
		p311.uptime_sec_SET(1987917500L);
		p311.hw_version_minor_SET((char) 54);
		p311.sw_version_minor_SET((char) 59);
		p311.time_usec_SET(4173083642746407909L);
		p311.hw_unique_id_SET(new char[]{(char) 1, (char) 10, (char) 77, (char) 69, (char) 240, (char) 126, (char) 36, (char) 118, (char) 210, (char) 85, (char) 173, (char) 84, (char) 184, (char) 206, (char) 32, (char) 245}, 0);
		CommunicationChannel.instance.send(p311);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_PARAM_EXT_REQUEST_READ.add((src, ph, pack) ->
		{
			assert (pack.target_component_GET() == (char) 121);
			assert (pack.param_index_GET() == (short) 26926);
			assert (pack.param_id_LEN(ph) == 1);
			assert (pack.param_id_TRY(ph).equals("f"));
			assert (pack.target_system_GET() == (char) 83);
		});
		GroundControl.PARAM_EXT_REQUEST_READ p320 = CommunicationChannel.new_PARAM_EXT_REQUEST_READ();
		PH.setPack(p320);
		p320.param_index_SET((short) 26926);
		p320.target_system_SET((char) 83);
		p320.target_component_SET((char) 121);
		p320.param_id_SET("f", PH);
		CommunicationChannel.instance.send(p320);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_PARAM_EXT_REQUEST_LIST.add((src, ph, pack) ->
		{
			assert (pack.target_component_GET() == (char) 92);
			assert (pack.target_system_GET() == (char) 70);
		});
		GroundControl.PARAM_EXT_REQUEST_LIST p321 = CommunicationChannel.new_PARAM_EXT_REQUEST_LIST();
		PH.setPack(p321);
		p321.target_system_SET((char) 70);
		p321.target_component_SET((char) 92);
		CommunicationChannel.instance.send(p321);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_PARAM_EXT_VALUE.add((src, ph, pack) ->
		{
			assert (pack.param_value_LEN(ph) == 6);
			assert (pack.param_value_TRY(ph).equals("sdlvxK"));
			assert (pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_REAL64);
			assert (pack.param_id_LEN(ph) == 2);
			assert (pack.param_id_TRY(ph).equals("cX"));
			assert (pack.param_index_GET() == (char) 3411);
			assert (pack.param_count_GET() == (char) 12994);
		});
		GroundControl.PARAM_EXT_VALUE p322 = CommunicationChannel.new_PARAM_EXT_VALUE();
		PH.setPack(p322);
		p322.param_id_SET("cX", PH);
		p322.param_count_SET((char) 12994);
		p322.param_value_SET("sdlvxK", PH);
		p322.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_REAL64);
		p322.param_index_SET((char) 3411);
		CommunicationChannel.instance.send(p322);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_PARAM_EXT_SET.add((src, ph, pack) ->
		{
			assert (pack.param_value_LEN(ph) == 82);
			assert (pack.param_value_TRY(ph).equals("afcgxfYkSbkGmMndgsdNtlYenctblignoeppcdfopjmhoqbmxlsmalmdvoFvzyoGhfdhmwbJjwkyUogqje"));
			assert (pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT64);
			assert (pack.target_system_GET() == (char) 191);
			assert (pack.target_component_GET() == (char) 221);
			assert (pack.param_id_LEN(ph) == 3);
			assert (pack.param_id_TRY(ph).equals("kzu"));
		});
		GroundControl.PARAM_EXT_SET p323 = CommunicationChannel.new_PARAM_EXT_SET();
		PH.setPack(p323);
		p323.param_id_SET("kzu", PH);
		p323.target_component_SET((char) 221);
		p323.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT64);
		p323.target_system_SET((char) 191);
		p323.param_value_SET("afcgxfYkSbkGmMndgsdNtlYenctblignoeppcdfopjmhoqbmxlsmalmdvoFvzyoGhfdhmwbJjwkyUogqje", PH);
		CommunicationChannel.instance.send(p323);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_PARAM_EXT_ACK.add((src, ph, pack) ->
		{
			assert (pack.param_result_GET() == PARAM_ACK.PARAM_ACK_VALUE_UNSUPPORTED);
			assert (pack.param_id_LEN(ph) == 7);
			assert (pack.param_id_TRY(ph).equals("xvmsrsb"));
			assert (pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT32);
			assert (pack.param_value_LEN(ph) == 123);
			assert (pack.param_value_TRY(ph).equals("KzemwydooEpvBouptlbbemsdybheypthsypvlgpSzidmedtbymKdtvjLtudlbvuimrdZKsgsvgMqhpycotytuJoLbowvsEfsqnJgwbvIgppommghwjdwbfqagsq"));
		});
		GroundControl.PARAM_EXT_ACK p324 = CommunicationChannel.new_PARAM_EXT_ACK();
		PH.setPack(p324);
		p324.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT32);
		p324.param_value_SET("KzemwydooEpvBouptlbbemsdybheypthsypvlgpSzidmedtbymKdtvjLtudlbvuimrdZKsgsvgMqhpycotytuJoLbowvsEfsqnJgwbvIgppommghwjdwbfqagsq", PH);
		p324.param_id_SET("xvmsrsb", PH);
		p324.param_result_SET(PARAM_ACK.PARAM_ACK_VALUE_UNSUPPORTED);
		CommunicationChannel.instance.send(p324);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_OBSTACLE_DISTANCE.add((src, ph, pack) ->
		{
			assert (pack.increment_GET() == (char) 160);
			assert (pack.max_distance_GET() == (char) 3873);
			assert (pack.sensor_type_GET() == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_UNKNOWN);
			assert (pack.time_usec_GET() == 5239418855877042331L);
			assert (pack.min_distance_GET() == (char) 35508);
			assert (Arrays.equals(pack.distances_GET(), new char[]{(char) 29128, (char) 44816, (char) 56056, (char) 62370, (char) 49107, (char) 65053, (char) 13347, (char) 60952, (char) 59997, (char) 12030, (char) 10705, (char) 44696, (char) 31120, (char) 33808, (char) 56953, (char) 28514, (char) 58034, (char) 19147, (char) 35976, (char) 4346, (char) 61765, (char) 63209, (char) 31283, (char) 62443, (char) 46227, (char) 8478, (char) 34545, (char) 2305, (char) 36079, (char) 5354, (char) 64093, (char) 55094, (char) 19698, (char) 32936, (char) 53140, (char) 36934, (char) 13765, (char) 50811, (char) 18677, (char) 42168, (char) 3152, (char) 31498, (char) 37801, (char) 29285, (char) 60927, (char) 54936, (char) 40955, (char) 42072, (char) 27218, (char) 56033, (char) 62349, (char) 45881, (char) 14716, (char) 62504, (char) 60057, (char) 23, (char) 18897, (char) 33105, (char) 20692, (char) 42583, (char) 10661, (char) 4026, (char) 7261, (char) 21790, (char) 53595, (char) 5355, (char) 64424, (char) 42456, (char) 31444, (char) 42813, (char) 21962, (char) 56798}));
		});
		GroundControl.OBSTACLE_DISTANCE p330 = CommunicationChannel.new_OBSTACLE_DISTANCE();
		PH.setPack(p330);
		p330.distances_SET(new char[]{(char) 29128, (char) 44816, (char) 56056, (char) 62370, (char) 49107, (char) 65053, (char) 13347, (char) 60952, (char) 59997, (char) 12030, (char) 10705, (char) 44696, (char) 31120, (char) 33808, (char) 56953, (char) 28514, (char) 58034, (char) 19147, (char) 35976, (char) 4346, (char) 61765, (char) 63209, (char) 31283, (char) 62443, (char) 46227, (char) 8478, (char) 34545, (char) 2305, (char) 36079, (char) 5354, (char) 64093, (char) 55094, (char) 19698, (char) 32936, (char) 53140, (char) 36934, (char) 13765, (char) 50811, (char) 18677, (char) 42168, (char) 3152, (char) 31498, (char) 37801, (char) 29285, (char) 60927, (char) 54936, (char) 40955, (char) 42072, (char) 27218, (char) 56033, (char) 62349, (char) 45881, (char) 14716, (char) 62504, (char) 60057, (char) 23, (char) 18897, (char) 33105, (char) 20692, (char) 42583, (char) 10661, (char) 4026, (char) 7261, (char) 21790, (char) 53595, (char) 5355, (char) 64424, (char) 42456, (char) 31444, (char) 42813, (char) 21962, (char) 56798}, 0);
		p330.sensor_type_SET(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_UNKNOWN);
		p330.max_distance_SET((char) 3873);
		p330.increment_SET((char) 160);
		p330.min_distance_SET((char) 35508);
		p330.time_usec_SET(5239418855877042331L);
		CommunicationChannel.instance.send(p330);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
	}

}