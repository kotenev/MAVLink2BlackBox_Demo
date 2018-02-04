
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

	public static class ATT_POS_MOCAP extends GroundControl.ATT_POS_MOCAP {
		public long time_usec_GET()//Timestamp (micros since boot or Unix epoch)
		{ return (get_bytes(data, 0, 8)); }

		public float[] q_GET(float[] dst_ch, int pos)  //Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
		{
			for (int BYTE = 8, dst_max = pos + 4; pos < dst_max; pos++, BYTE += 4)
				dst_ch[pos] = (float) (Float.intBitsToFloat((int) get_bytes(data, BYTE, 4)));
			return dst_ch;
		}

		public float[] q_GET()//Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
		{return q_GET(new float[4], 0);}

		public float x_GET()//X position in meters (NED)
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 24, 4))); }

		public float y_GET()//Y position in meters (NED)
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 28, 4))); }

		public float z_GET()//Z position in meters (NED)
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 32, 4))); }
	}

	public static class SET_ACTUATOR_CONTROL_TARGET extends GroundControl.SET_ACTUATOR_CONTROL_TARGET {
		public long time_usec_GET()//Timestamp (micros since boot or Unix epoch)
		{ return (get_bytes(data, 0, 8)); }

		/**
		 * Actuator group. The "_mlx" indicates this is a multi-instance message and a MAVLink parser should use
		 * this field to difference between instances
		 */
		public char group_mlx_GET() { return (char) ((char) get_bytes(data, 8, 1)); }

		public char target_system_GET()//System ID
		{ return (char) ((char) get_bytes(data, 9, 1)); }

		public char target_component_GET()//Component ID
		{ return (char) ((char) get_bytes(data, 10, 1)); }

		/**
		 * Actuator controls. Normed to -1..+1 where 0 is neutral position. Throttle for single rotation direction
		 * motors is 0..1, negative range for reverse direction. Standard mapping for attitude controls (group 0):
		 * (index 0-7): roll, pitch, yaw, throttle, flaps, spoilers, airbrakes, landing gear. Load a pass-through
		 * mixer to repurpose them as generic outputs
		 */
		public float[] controls_GET(float[] dst_ch, int pos) {
			for (int BYTE = 11, dst_max = pos + 8; pos < dst_max; pos++, BYTE += 4)
				dst_ch[pos] = (float) (Float.intBitsToFloat((int) get_bytes(data, BYTE, 4)));
			return dst_ch;
		}

		/**
		 * Actuator controls. Normed to -1..+1 where 0 is neutral position. Throttle for single rotation direction
		 * motors is 0..1, negative range for reverse direction. Standard mapping for attitude controls (group 0):
		 * (index 0-7): roll, pitch, yaw, throttle, flaps, spoilers, airbrakes, landing gear. Load a pass-through
		 * mixer to repurpose them as generic outputs
		 */
		public float[] controls_GET() {return controls_GET(new float[8], 0);}
	}

	public static class ACTUATOR_CONTROL_TARGET extends GroundControl.ACTUATOR_CONTROL_TARGET {
		public long time_usec_GET()//Timestamp (micros since boot or Unix epoch)
		{ return (get_bytes(data, 0, 8)); }

		/**
		 * Actuator group. The "_mlx" indicates this is a multi-instance message and a MAVLink parser should use
		 * this field to difference between instances
		 */
		public char group_mlx_GET() { return (char) ((char) get_bytes(data, 8, 1)); }

		/**
		 * Actuator controls. Normed to -1..+1 where 0 is neutral position. Throttle for single rotation direction
		 * motors is 0..1, negative range for reverse direction. Standard mapping for attitude controls (group 0):
		 * (index 0-7): roll, pitch, yaw, throttle, flaps, spoilers, airbrakes, landing gear. Load a pass-through
		 * mixer to repurpose them as generic outputs
		 */
		public float[] controls_GET(float[] dst_ch, int pos) {
			for (int BYTE = 9, dst_max = pos + 8; pos < dst_max; pos++, BYTE += 4)
				dst_ch[pos] = (float) (Float.intBitsToFloat((int) get_bytes(data, BYTE, 4)));
			return dst_ch;
		}

		/**
		 * Actuator controls. Normed to -1..+1 where 0 is neutral position. Throttle for single rotation direction
		 * motors is 0..1, negative range for reverse direction. Standard mapping for attitude controls (group 0):
		 * (index 0-7): roll, pitch, yaw, throttle, flaps, spoilers, airbrakes, landing gear. Load a pass-through
		 * mixer to repurpose them as generic outputs
		 */
		public float[] controls_GET() {return controls_GET(new float[8], 0);}
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

		static final Collection<OnReceive.Handler<ATT_POS_MOCAP, Channel>>               on_ATT_POS_MOCAP               = new OnReceive<>();
		static final Collection<OnReceive.Handler<SET_ACTUATOR_CONTROL_TARGET, Channel>> on_SET_ACTUATOR_CONTROL_TARGET = new OnReceive<>();
		static final Collection<OnReceive.Handler<ACTUATOR_CONTROL_TARGET, Channel>>     on_ACTUATOR_CONTROL_TARGET     = new OnReceive<>();
		static final Collection<OnReceive.Handler<ALTITUDE, Channel>>                    on_ALTITUDE                    = new OnReceive<>();
		static final Collection<OnReceive.Handler<RESOURCE_REQUEST, Channel>>            on_RESOURCE_REQUEST            = new OnReceive<>();
		static final Collection<OnReceive.Handler<SCALED_PRESSURE3, Channel>>            on_SCALED_PRESSURE3            = new OnReceive<>();
		static final Collection<OnReceive.Handler<FOLLOW_TARGET, Channel>>               on_FOLLOW_TARGET               = new OnReceive<>();
		static final Collection<OnReceive.Handler<CONTROL_SYSTEM_STATE, Channel>>        on_CONTROL_SYSTEM_STATE        = new OnReceive<>();
		static final Collection<OnReceive.Handler<BATTERY_STATUS, Channel>>              on_BATTERY_STATUS              = new OnReceive<>();
		static final Collection<OnReceive.Handler<AUTOPILOT_VERSION, Channel>>           on_AUTOPILOT_VERSION           = new OnReceive<>();
		static final Collection<OnReceive.Handler<LANDING_TARGET, Channel>>              on_LANDING_TARGET              = new OnReceive<>();
		static final Collection<OnReceive.Handler<ESTIMATOR_STATUS, Channel>>            on_ESTIMATOR_STATUS            = new OnReceive<>();
		static final Collection<OnReceive.Handler<WIND_COV, Channel>>                    on_WIND_COV                    = new OnReceive<>();
		static final Collection<OnReceive.Handler<GPS_INPUT, Channel>>                   on_GPS_INPUT                   = new OnReceive<>();
		static final Collection<OnReceive.Handler<GPS_RTCM_DATA, Channel>>               on_GPS_RTCM_DATA               = new OnReceive<>();
		static final Collection<OnReceive.Handler<HIGH_LATENCY, Channel>>                on_HIGH_LATENCY                = new OnReceive<>();
		static final Collection<OnReceive.Handler<VIBRATION, Channel>>                   on_VIBRATION                   = new OnReceive<>();
		static final Collection<OnReceive.Handler<HOME_POSITION, Channel>>               on_HOME_POSITION               = new OnReceive<>();
		static final Collection<OnReceive.Handler<SET_HOME_POSITION, Channel>>           on_SET_HOME_POSITION           = new OnReceive<>();
		static final Collection<OnReceive.Handler<MESSAGE_INTERVAL, Channel>>            on_MESSAGE_INTERVAL            = new OnReceive<>();
		static final Collection<OnReceive.Handler<EXTENDED_SYS_STATE, Channel>>          on_EXTENDED_SYS_STATE          = new OnReceive<>();
		static final Collection<OnReceive.Handler<ADSB_VEHICLE, Channel>>                on_ADSB_VEHICLE                = new OnReceive<>();
		static final Collection<OnReceive.Handler<COLLISION, Channel>>                   on_COLLISION                   = new OnReceive<>();
		static final Collection<OnReceive.Handler<V2_EXTENSION, Channel>>                on_V2_EXTENSION                = new OnReceive<>();
		static final Collection<OnReceive.Handler<MEMORY_VECT, Channel>>                 on_MEMORY_VECT                 = new OnReceive<>();
		static final Collection<OnReceive.Handler<DEBUG_VECT, Channel>>                  on_DEBUG_VECT                  = new OnReceive<>();
		static final Collection<OnReceive.Handler<NAMED_VALUE_FLOAT, Channel>>           on_NAMED_VALUE_FLOAT           = new OnReceive<>();
		static final Collection<OnReceive.Handler<NAMED_VALUE_INT, Channel>>             on_NAMED_VALUE_INT             = new OnReceive<>();
		static final Collection<OnReceive.Handler<STATUSTEXT, Channel>>                  on_STATUSTEXT                  = new OnReceive<>();
		static final Collection<OnReceive.Handler<DEBUG, Channel>>                       on_DEBUG                       = new OnReceive<>();
		static final Collection<OnReceive.Handler<SETUP_SIGNING, Channel>>               on_SETUP_SIGNING               = new OnReceive<>();
		static final Collection<OnReceive.Handler<BUTTON_CHANGE, Channel>>               on_BUTTON_CHANGE               = new OnReceive<>();
		static final Collection<OnReceive.Handler<PLAY_TUNE, Channel>>                   on_PLAY_TUNE                   = new OnReceive<>();
		static final Collection<OnReceive.Handler<CAMERA_INFORMATION, Channel>>          on_CAMERA_INFORMATION          = new OnReceive<>();
		static final Collection<OnReceive.Handler<CAMERA_SETTINGS, Channel>>             on_CAMERA_SETTINGS             = new OnReceive<>();
		static final Collection<OnReceive.Handler<STORAGE_INFORMATION, Channel>>         on_STORAGE_INFORMATION         = new OnReceive<>();
		static final Collection<OnReceive.Handler<CAMERA_CAPTURE_STATUS, Channel>>       on_CAMERA_CAPTURE_STATUS       = new OnReceive<>();
		static final Collection<OnReceive.Handler<CAMERA_IMAGE_CAPTURED, Channel>>       on_CAMERA_IMAGE_CAPTURED       = new OnReceive<>();
		static final Collection<OnReceive.Handler<FLIGHT_INFORMATION, Channel>>          on_FLIGHT_INFORMATION          = new OnReceive<>();
		static final Collection<OnReceive.Handler<MOUNT_ORIENTATION, Channel>>           on_MOUNT_ORIENTATION           = new OnReceive<>();
		static final Collection<OnReceive.Handler<LOGGING_DATA, Channel>>                on_LOGGING_DATA                = new OnReceive<>();
		static final Collection<OnReceive.Handler<LOGGING_DATA_ACKED, Channel>>          on_LOGGING_DATA_ACKED          = new OnReceive<>();
		static final Collection<OnReceive.Handler<LOGGING_ACK, Channel>>                 on_LOGGING_ACK                 = new OnReceive<>();
		static final Collection<OnReceive.Handler<VIDEO_STREAM_INFORMATION, Channel>>    on_VIDEO_STREAM_INFORMATION    = new OnReceive<>();
		static final Collection<OnReceive.Handler<SET_VIDEO_STREAM_SETTINGS, Channel>>   on_SET_VIDEO_STREAM_SETTINGS   = new OnReceive<>();
		static final Collection<OnReceive.Handler<WIFI_CONFIG_AP, Channel>>              on_WIFI_CONFIG_AP              = new OnReceive<>();
		static final Collection<OnReceive.Handler<PROTOCOL_VERSION, Channel>>            on_PROTOCOL_VERSION            = new OnReceive<>();
		static final Collection<OnReceive.Handler<UAVCAN_NODE_STATUS, Channel>>          on_UAVCAN_NODE_STATUS          = new OnReceive<>();
		static final Collection<OnReceive.Handler<UAVCAN_NODE_INFO, Channel>>            on_UAVCAN_NODE_INFO            = new OnReceive<>();
		static final Collection<OnReceive.Handler<PARAM_EXT_REQUEST_READ, Channel>>      on_PARAM_EXT_REQUEST_READ      = new OnReceive<>();
		static final Collection<OnReceive.Handler<PARAM_EXT_REQUEST_LIST, Channel>>      on_PARAM_EXT_REQUEST_LIST      = new OnReceive<>();
		static final Collection<OnReceive.Handler<PARAM_EXT_VALUE, Channel>>             on_PARAM_EXT_VALUE             = new OnReceive<>();
		static final Collection<OnReceive.Handler<PARAM_EXT_SET, Channel>>               on_PARAM_EXT_SET               = new OnReceive<>();
		static final Collection<OnReceive.Handler<PARAM_EXT_ACK, Channel>>               on_PARAM_EXT_ACK               = new OnReceive<>();
		static final Collection<OnReceive.Handler<OBSTACLE_DISTANCE, Channel>>           on_OBSTACLE_DISTANCE           = new OnReceive<>();


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
				case 138:
					if (pack == null) return new ATT_POS_MOCAP();
					((OnReceive) on_ATT_POS_MOCAP).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
					break;
				case 139:
					if (pack == null) return new SET_ACTUATOR_CONTROL_TARGET();
					((OnReceive) on_SET_ACTUATOR_CONTROL_TARGET).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
					break;
				case 140:
					if (pack == null) return new ACTUATOR_CONTROL_TARGET();
					((OnReceive) on_ACTUATOR_CONTROL_TARGET).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
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
			assert (pack.type_GET() == MAV_TYPE.MAV_TYPE_FIXED_WING);
			assert (pack.autopilot_GET() == MAV_AUTOPILOT.MAV_AUTOPILOT_PX4);
			assert (pack.mavlink_version_GET() == (char) 28);
			assert (pack.system_status_GET() == MAV_STATE.MAV_STATE_ACTIVE);
			assert (pack.custom_mode_GET() == 926812424L);
			assert (pack.base_mode_GET() == MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED);
		});
		HEARTBEAT p0 = new HEARTBEAT();
		PH.setPack(p0);
		p0.base_mode_SET(MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED);
		p0.system_status_SET(MAV_STATE.MAV_STATE_ACTIVE);
		p0.autopilot_SET(MAV_AUTOPILOT.MAV_AUTOPILOT_PX4);
		p0.type_SET(MAV_TYPE.MAV_TYPE_FIXED_WING);
		p0.custom_mode_SET(926812424L);
		p0.mavlink_version_SET((char) 28);
		TestChannel.instance.send(p0);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_SYS_STATUS.add((src, ph, pack) ->
		{
			assert (pack.drop_rate_comm_GET() == (char) 45306);
			assert (pack.errors_count3_GET() == (char) 3079);
			assert (pack.onboard_control_sensors_enabled_GET() == MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2);
			assert (pack.errors_comm_GET() == (char) 35477);
			assert (pack.onboard_control_sensors_health_GET() == MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_AHRS);
			assert (pack.errors_count2_GET() == (char) 45283);
			assert (pack.voltage_battery_GET() == (char) 4979);
			assert (pack.errors_count4_GET() == (char) 22276);
			assert (pack.errors_count1_GET() == (char) 967);
			assert (pack.onboard_control_sensors_present_GET() == MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO);
			assert (pack.current_battery_GET() == (short) -9781);
			assert (pack.battery_remaining_GET() == (byte) -98);
			assert (pack.load_GET() == (char) 25303);
		});
		SYS_STATUS p1 = new SYS_STATUS();
		PH.setPack(p1);
		p1.errors_count1_SET((char) 967);
		p1.errors_count2_SET((char) 45283);
		p1.onboard_control_sensors_health_SET(MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_AHRS);
		p1.current_battery_SET((short) -9781);
		p1.voltage_battery_SET((char) 4979);
		p1.errors_count3_SET((char) 3079);
		p1.drop_rate_comm_SET((char) 45306);
		p1.errors_comm_SET((char) 35477);
		p1.onboard_control_sensors_present_SET(MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO);
		p1.onboard_control_sensors_enabled_SET(MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2);
		p1.battery_remaining_SET((byte) -98);
		p1.load_SET((char) 25303);
		p1.errors_count4_SET((char) 22276);
		TestChannel.instance.send(p1);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_SYSTEM_TIME.add((src, ph, pack) ->
		{
			assert (pack.time_unix_usec_GET() == 1270743578334733105L);
			assert (pack.time_boot_ms_GET() == 696312629L);
		});
		SYSTEM_TIME p2 = new SYSTEM_TIME();
		PH.setPack(p2);
		p2.time_boot_ms_SET(696312629L);
		p2.time_unix_usec_SET(1270743578334733105L);
		TestChannel.instance.send(p2);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_POSITION_TARGET_LOCAL_NED.add((src, ph, pack) ->
		{
			assert (pack.yaw_GET() == -1.4472783E38F);
			assert (pack.type_mask_GET() == (char) 9294);
			assert (pack.vy_GET() == -1.9384434E38F);
			assert (pack.time_boot_ms_GET() == 835591173L);
			assert (pack.afy_GET() == 9.599131E37F);
			assert (pack.y_GET() == 2.1821926E38F);
			assert (pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT);
			assert (pack.afz_GET() == 2.677615E38F);
			assert (pack.vz_GET() == 1.8411538E38F);
			assert (pack.yaw_rate_GET() == 1.2377668E38F);
			assert (pack.x_GET() == -1.3867666E38F);
			assert (pack.vx_GET() == -7.6907104E37F);
			assert (pack.z_GET() == -1.7822776E38F);
			assert (pack.afx_GET() == -1.3548644E38F);
		});
		GroundControl.POSITION_TARGET_LOCAL_NED p3 = CommunicationChannel.new_POSITION_TARGET_LOCAL_NED();
		PH.setPack(p3);
		p3.afy_SET(9.599131E37F);
		p3.x_SET(-1.3867666E38F);
		p3.vx_SET(-7.6907104E37F);
		p3.time_boot_ms_SET(835591173L);
		p3.vy_SET(-1.9384434E38F);
		p3.z_SET(-1.7822776E38F);
		p3.type_mask_SET((char) 9294);
		p3.afx_SET(-1.3548644E38F);
		p3.yaw_rate_SET(1.2377668E38F);
		p3.afz_SET(2.677615E38F);
		p3.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT);
		p3.vz_SET(1.8411538E38F);
		p3.y_SET(2.1821926E38F);
		p3.yaw_SET(-1.4472783E38F);
		CommunicationChannel.instance.send(p3);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_PING.add((src, ph, pack) ->
		{
			assert (pack.seq_GET() == 2694995896L);
			assert (pack.time_usec_GET() == 2995936643017331050L);
			assert (pack.target_system_GET() == (char) 201);
			assert (pack.target_component_GET() == (char) 169);
		});
		PING p4 = new PING();
		PH.setPack(p4);
		p4.target_component_SET((char) 169);
		p4.target_system_SET((char) 201);
		p4.seq_SET(2694995896L);
		p4.time_usec_SET(2995936643017331050L);
		TestChannel.instance.send(p4);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_CHANGE_OPERATOR_CONTROL.add((src, ph, pack) ->
		{
			assert (pack.passkey_LEN(ph) == 12);
			assert (pack.passkey_TRY(ph).equals("iSegtfYjcofq"));
			assert (pack.target_system_GET() == (char) 116);
			assert (pack.control_request_GET() == (char) 203);
			assert (pack.version_GET() == (char) 15);
		});
		CHANGE_OPERATOR_CONTROL p5 = new CHANGE_OPERATOR_CONTROL();
		PH.setPack(p5);
		p5.control_request_SET((char) 203);
		p5.passkey_SET("iSegtfYjcofq", PH);
		p5.target_system_SET((char) 116);
		p5.version_SET((char) 15);
		TestChannel.instance.send(p5);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_CHANGE_OPERATOR_CONTROL_ACK.add((src, ph, pack) ->
		{
			assert (pack.ack_GET() == (char) 231);
			assert (pack.gcs_system_id_GET() == (char) 185);
			assert (pack.control_request_GET() == (char) 254);
		});
		CHANGE_OPERATOR_CONTROL_ACK p6 = new CHANGE_OPERATOR_CONTROL_ACK();
		PH.setPack(p6);
		p6.ack_SET((char) 231);
		p6.gcs_system_id_SET((char) 185);
		p6.control_request_SET((char) 254);
		TestChannel.instance.send(p6);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_AUTH_KEY.add((src, ph, pack) ->
		{
			assert (pack.key_LEN(ph) == 11);
			assert (pack.key_TRY(ph).equals("bqmmFwujNas"));
		});
		AUTH_KEY p7 = new AUTH_KEY();
		PH.setPack(p7);
		p7.key_SET("bqmmFwujNas", PH);
		TestChannel.instance.send(p7);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_SET_MODE.add((src, ph, pack) ->
		{
			assert (pack.base_mode_GET() == MAV_MODE.MAV_MODE_TEST_ARMED);
			assert (pack.custom_mode_GET() == 2647805738L);
			assert (pack.target_system_GET() == (char) 183);
		});
		SET_MODE p11 = new SET_MODE();
		PH.setPack(p11);
		p11.base_mode_SET(MAV_MODE.MAV_MODE_TEST_ARMED);
		p11.custom_mode_SET(2647805738L);
		p11.target_system_SET((char) 183);
		TestChannel.instance.send(p11);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_PARAM_REQUEST_READ.add((src, ph, pack) ->
		{
			assert (pack.param_index_GET() == (short) -29231);
			assert (pack.param_id_LEN(ph) == 14);
			assert (pack.param_id_TRY(ph).equals("haqszkjiawiiaP"));
			assert (pack.target_system_GET() == (char) 100);
			assert (pack.target_component_GET() == (char) 21);
		});
		PARAM_REQUEST_READ p20 = new PARAM_REQUEST_READ();
		PH.setPack(p20);
		p20.target_system_SET((char) 100);
		p20.target_component_SET((char) 21);
		p20.param_index_SET((short) -29231);
		p20.param_id_SET("haqszkjiawiiaP", PH);
		TestChannel.instance.send(p20);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_PARAM_REQUEST_LIST.add((src, ph, pack) ->
		{
			assert (pack.target_system_GET() == (char) 136);
			assert (pack.target_component_GET() == (char) 72);
		});
		PARAM_REQUEST_LIST p21 = new PARAM_REQUEST_LIST();
		PH.setPack(p21);
		p21.target_system_SET((char) 136);
		p21.target_component_SET((char) 72);
		TestChannel.instance.send(p21);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_PARAM_VALUE.add((src, ph, pack) ->
		{
			assert (pack.param_id_LEN(ph) == 1);
			assert (pack.param_id_TRY(ph).equals("s"));
			assert (pack.param_index_GET() == (char) 17496);
			assert (pack.param_type_GET() == MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT8);
			assert (pack.param_value_GET() == -2.77281E38F);
			assert (pack.param_count_GET() == (char) 17320);
		});
		PARAM_VALUE p22 = new PARAM_VALUE();
		PH.setPack(p22);
		p22.param_id_SET("s", PH);
		p22.param_value_SET(-2.77281E38F);
		p22.param_count_SET((char) 17320);
		p22.param_index_SET((char) 17496);
		p22.param_type_SET(MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT8);
		TestChannel.instance.send(p22);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_PARAM_SET.add((src, ph, pack) ->
		{
			assert (pack.target_component_GET() == (char) 242);
			assert (pack.target_system_GET() == (char) 158);
			assert (pack.param_type_GET() == MAV_PARAM_TYPE.MAV_PARAM_TYPE_REAL32);
			assert (pack.param_id_LEN(ph) == 10);
			assert (pack.param_id_TRY(ph).equals("ifnhyerAry"));
			assert (pack.param_value_GET() == -3.0025093E38F);
		});
		PARAM_SET p23 = new PARAM_SET();
		PH.setPack(p23);
		p23.target_system_SET((char) 158);
		p23.target_component_SET((char) 242);
		p23.param_id_SET("ifnhyerAry", PH);
		p23.param_type_SET(MAV_PARAM_TYPE.MAV_PARAM_TYPE_REAL32);
		p23.param_value_SET(-3.0025093E38F);
		TestChannel.instance.send(p23);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_GPS_RAW_INT.add((src, ph, pack) ->
		{
			assert (pack.eph_GET() == (char) 2042);
			assert (pack.alt_GET() == -378113636);
			assert (pack.hdg_acc_TRY(ph) == 1474487256L);
			assert (pack.alt_ellipsoid_TRY(ph) == -830452395);
			assert (pack.epv_GET() == (char) 58416);
			assert (pack.v_acc_TRY(ph) == 2744151840L);
			assert (pack.lat_GET() == 1266558070);
			assert (pack.vel_GET() == (char) 54271);
			assert (pack.fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_NO_FIX);
			assert (pack.h_acc_TRY(ph) == 3830248429L);
			assert (pack.vel_acc_TRY(ph) == 1690137869L);
			assert (pack.satellites_visible_GET() == (char) 66);
			assert (pack.lon_GET() == 1638119448);
			assert (pack.cog_GET() == (char) 16721);
			assert (pack.time_usec_GET() == 1535517759042072301L);
		});
		GPS_RAW_INT p24 = new GPS_RAW_INT();
		PH.setPack(p24);
		p24.vel_SET((char) 54271);
		p24.alt_SET(-378113636);
		p24.vel_acc_SET(1690137869L, PH);
		p24.alt_ellipsoid_SET(-830452395, PH);
		p24.time_usec_SET(1535517759042072301L);
		p24.v_acc_SET(2744151840L, PH);
		p24.eph_SET((char) 2042);
		p24.hdg_acc_SET(1474487256L, PH);
		p24.cog_SET((char) 16721);
		p24.h_acc_SET(3830248429L, PH);
		p24.fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_NO_FIX);
		p24.lat_SET(1266558070);
		p24.lon_SET(1638119448);
		p24.epv_SET((char) 58416);
		p24.satellites_visible_SET((char) 66);
		TestChannel.instance.send(p24);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_GPS_STATUS.add((src, ph, pack) ->
		{
			assert (Arrays.equals(pack.satellite_used_GET(), new char[]{(char) 188, (char) 170, (char) 7, (char) 174, (char) 252, (char) 151, (char) 52, (char) 166, (char) 250, (char) 50, (char) 199, (char) 156, (char) 144, (char) 180, (char) 3, (char) 79, (char) 97, (char) 221, (char) 6, (char) 221}));
			assert (Arrays.equals(pack.satellite_prn_GET(), new char[]{(char) 240, (char) 45, (char) 2, (char) 153, (char) 27, (char) 202, (char) 114, (char) 12, (char) 58, (char) 218, (char) 19, (char) 228, (char) 41, (char) 192, (char) 169, (char) 91, (char) 220, (char) 213, (char) 111, (char) 129}));
			assert (Arrays.equals(pack.satellite_snr_GET(), new char[]{(char) 135, (char) 72, (char) 16, (char) 212, (char) 116, (char) 156, (char) 92, (char) 187, (char) 104, (char) 100, (char) 88, (char) 243, (char) 145, (char) 47, (char) 28, (char) 181, (char) 88, (char) 107, (char) 110, (char) 20}));
			assert (Arrays.equals(pack.satellite_azimuth_GET(), new char[]{(char) 152, (char) 247, (char) 91, (char) 27, (char) 223, (char) 168, (char) 236, (char) 240, (char) 118, (char) 94, (char) 118, (char) 18, (char) 207, (char) 32, (char) 141, (char) 147, (char) 21, (char) 11, (char) 222, (char) 219}));
			assert (pack.satellites_visible_GET() == (char) 203);
			assert (Arrays.equals(pack.satellite_elevation_GET(), new char[]{(char) 147, (char) 71, (char) 119, (char) 68, (char) 116, (char) 219, (char) 45, (char) 43, (char) 130, (char) 216, (char) 142, (char) 224, (char) 154, (char) 69, (char) 22, (char) 210, (char) 70, (char) 98, (char) 248, (char) 95}));
		});
		GPS_STATUS p25 = new GPS_STATUS();
		PH.setPack(p25);
		p25.satellite_prn_SET(new char[]{(char) 240, (char) 45, (char) 2, (char) 153, (char) 27, (char) 202, (char) 114, (char) 12, (char) 58, (char) 218, (char) 19, (char) 228, (char) 41, (char) 192, (char) 169, (char) 91, (char) 220, (char) 213, (char) 111, (char) 129}, 0);
		p25.satellite_snr_SET(new char[]{(char) 135, (char) 72, (char) 16, (char) 212, (char) 116, (char) 156, (char) 92, (char) 187, (char) 104, (char) 100, (char) 88, (char) 243, (char) 145, (char) 47, (char) 28, (char) 181, (char) 88, (char) 107, (char) 110, (char) 20}, 0);
		p25.satellites_visible_SET((char) 203);
		p25.satellite_azimuth_SET(new char[]{(char) 152, (char) 247, (char) 91, (char) 27, (char) 223, (char) 168, (char) 236, (char) 240, (char) 118, (char) 94, (char) 118, (char) 18, (char) 207, (char) 32, (char) 141, (char) 147, (char) 21, (char) 11, (char) 222, (char) 219}, 0);
		p25.satellite_elevation_SET(new char[]{(char) 147, (char) 71, (char) 119, (char) 68, (char) 116, (char) 219, (char) 45, (char) 43, (char) 130, (char) 216, (char) 142, (char) 224, (char) 154, (char) 69, (char) 22, (char) 210, (char) 70, (char) 98, (char) 248, (char) 95}, 0);
		p25.satellite_used_SET(new char[]{(char) 188, (char) 170, (char) 7, (char) 174, (char) 252, (char) 151, (char) 52, (char) 166, (char) 250, (char) 50, (char) 199, (char) 156, (char) 144, (char) 180, (char) 3, (char) 79, (char) 97, (char) 221, (char) 6, (char) 221}, 0);
		TestChannel.instance.send(p25);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_SCALED_IMU.add((src, ph, pack) ->
		{
			assert (pack.ymag_GET() == (short) -31506);
			assert (pack.xmag_GET() == (short) 9721);
			assert (pack.yacc_GET() == (short) -28989);
			assert (pack.zacc_GET() == (short) -18553);
			assert (pack.zgyro_GET() == (short) -5172);
			assert (pack.xacc_GET() == (short) -7524);
			assert (pack.ygyro_GET() == (short) -780);
			assert (pack.xgyro_GET() == (short) -25516);
			assert (pack.zmag_GET() == (short) 6100);
			assert (pack.time_boot_ms_GET() == 2414488397L);
		});
		SCALED_IMU p26 = new SCALED_IMU();
		PH.setPack(p26);
		p26.xmag_SET((short) 9721);
		p26.time_boot_ms_SET(2414488397L);
		p26.xacc_SET((short) -7524);
		p26.yacc_SET((short) -28989);
		p26.zmag_SET((short) 6100);
		p26.ygyro_SET((short) -780);
		p26.zacc_SET((short) -18553);
		p26.ymag_SET((short) -31506);
		p26.xgyro_SET((short) -25516);
		p26.zgyro_SET((short) -5172);
		TestChannel.instance.send(p26);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_RAW_IMU.add((src, ph, pack) ->
		{
			assert (pack.ymag_GET() == (short) 10443);
			assert (pack.zacc_GET() == (short) -23630);
			assert (pack.xacc_GET() == (short) -31595);
			assert (pack.xmag_GET() == (short) -8951);
			assert (pack.ygyro_GET() == (short) 16081);
			assert (pack.zmag_GET() == (short) 294);
			assert (pack.xgyro_GET() == (short) 17426);
			assert (pack.time_usec_GET() == 5792765620884980946L);
			assert (pack.yacc_GET() == (short) -5563);
			assert (pack.zgyro_GET() == (short) 2068);
		});
		RAW_IMU p27 = new RAW_IMU();
		PH.setPack(p27);
		p27.yacc_SET((short) -5563);
		p27.time_usec_SET(5792765620884980946L);
		p27.xmag_SET((short) -8951);
		p27.zmag_SET((short) 294);
		p27.xgyro_SET((short) 17426);
		p27.zgyro_SET((short) 2068);
		p27.xacc_SET((short) -31595);
		p27.zacc_SET((short) -23630);
		p27.ymag_SET((short) 10443);
		p27.ygyro_SET((short) 16081);
		TestChannel.instance.send(p27);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_RAW_PRESSURE.add((src, ph, pack) ->
		{
			assert (pack.press_abs_GET() == (short) 8324);
			assert (pack.temperature_GET() == (short) -11592);
			assert (pack.press_diff1_GET() == (short) 7574);
			assert (pack.time_usec_GET() == 5151469335258398644L);
			assert (pack.press_diff2_GET() == (short) 19245);
		});
		RAW_PRESSURE p28 = new RAW_PRESSURE();
		PH.setPack(p28);
		p28.temperature_SET((short) -11592);
		p28.press_diff1_SET((short) 7574);
		p28.time_usec_SET(5151469335258398644L);
		p28.press_diff2_SET((short) 19245);
		p28.press_abs_SET((short) 8324);
		TestChannel.instance.send(p28);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_SCALED_PRESSURE.add((src, ph, pack) ->
		{
			assert (pack.temperature_GET() == (short) -22005);
			assert (pack.time_boot_ms_GET() == 3395344862L);
			assert (pack.press_abs_GET() == 2.8043705E38F);
			assert (pack.press_diff_GET() == -3.356514E38F);
		});
		SCALED_PRESSURE p29 = new SCALED_PRESSURE();
		PH.setPack(p29);
		p29.temperature_SET((short) -22005);
		p29.press_abs_SET(2.8043705E38F);
		p29.press_diff_SET(-3.356514E38F);
		p29.time_boot_ms_SET(3395344862L);
		TestChannel.instance.send(p29);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_ATTITUDE.add((src, ph, pack) ->
		{
			assert (pack.roll_GET() == 2.2485753E38F);
			assert (pack.yawspeed_GET() == -1.6669728E37F);
			assert (pack.yaw_GET() == 3.3344884E38F);
			assert (pack.time_boot_ms_GET() == 2703498718L);
			assert (pack.pitch_GET() == -2.6822231E38F);
			assert (pack.pitchspeed_GET() == -2.7041665E38F);
			assert (pack.rollspeed_GET() == 5.9141916E36F);
		});
		ATTITUDE p30 = new ATTITUDE();
		PH.setPack(p30);
		p30.pitch_SET(-2.6822231E38F);
		p30.yawspeed_SET(-1.6669728E37F);
		p30.roll_SET(2.2485753E38F);
		p30.time_boot_ms_SET(2703498718L);
		p30.rollspeed_SET(5.9141916E36F);
		p30.pitchspeed_SET(-2.7041665E38F);
		p30.yaw_SET(3.3344884E38F);
		TestChannel.instance.send(p30);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_ATTITUDE_QUATERNION.add((src, ph, pack) ->
		{
			assert (pack.q3_GET() == 1.9689707E38F);
			assert (pack.rollspeed_GET() == -1.6431101E38F);
			assert (pack.q4_GET() == -1.2009319E38F);
			assert (pack.q1_GET() == 7.3915647E37F);
			assert (pack.pitchspeed_GET() == 2.8793178E36F);
			assert (pack.time_boot_ms_GET() == 1644514212L);
			assert (pack.q2_GET() == 1.4165141E38F);
			assert (pack.yawspeed_GET() == -2.8872832E38F);
		});
		ATTITUDE_QUATERNION p31 = new ATTITUDE_QUATERNION();
		PH.setPack(p31);
		p31.q2_SET(1.4165141E38F);
		p31.yawspeed_SET(-2.8872832E38F);
		p31.q3_SET(1.9689707E38F);
		p31.pitchspeed_SET(2.8793178E36F);
		p31.q1_SET(7.3915647E37F);
		p31.q4_SET(-1.2009319E38F);
		p31.rollspeed_SET(-1.6431101E38F);
		p31.time_boot_ms_SET(1644514212L);
		TestChannel.instance.send(p31);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_LOCAL_POSITION_NED.add((src, ph, pack) ->
		{
			assert (pack.vx_GET() == 4.906493E37F);
			assert (pack.vy_GET() == -4.8681E37F);
			assert (pack.y_GET() == 8.684119E37F);
			assert (pack.x_GET() == -3.2507141E38F);
			assert (pack.vz_GET() == -2.8068103E38F);
			assert (pack.time_boot_ms_GET() == 1694119866L);
			assert (pack.z_GET() == -2.939094E38F);
		});
		LOCAL_POSITION_NED p32 = new LOCAL_POSITION_NED();
		PH.setPack(p32);
		p32.vz_SET(-2.8068103E38F);
		p32.z_SET(-2.939094E38F);
		p32.x_SET(-3.2507141E38F);
		p32.y_SET(8.684119E37F);
		p32.vy_SET(-4.8681E37F);
		p32.vx_SET(4.906493E37F);
		p32.time_boot_ms_SET(1694119866L);
		TestChannel.instance.send(p32);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_GLOBAL_POSITION_INT.add((src, ph, pack) ->
		{
			assert (pack.lat_GET() == -5906361);
			assert (pack.vz_GET() == (short) 10221);
			assert (pack.hdg_GET() == (char) 23048);
			assert (pack.time_boot_ms_GET() == 3823055920L);
			assert (pack.relative_alt_GET() == -1905265008);
			assert (pack.vy_GET() == (short) 357);
			assert (pack.lon_GET() == 1235506535);
			assert (pack.vx_GET() == (short) 27256);
			assert (pack.alt_GET() == 1683673470);
		});
		GLOBAL_POSITION_INT p33 = new GLOBAL_POSITION_INT();
		PH.setPack(p33);
		p33.relative_alt_SET(-1905265008);
		p33.vz_SET((short) 10221);
		p33.vx_SET((short) 27256);
		p33.lon_SET(1235506535);
		p33.hdg_SET((char) 23048);
		p33.vy_SET((short) 357);
		p33.alt_SET(1683673470);
		p33.time_boot_ms_SET(3823055920L);
		p33.lat_SET(-5906361);
		TestChannel.instance.send(p33);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_RC_CHANNELS_SCALED.add((src, ph, pack) ->
		{
			assert (pack.port_GET() == (char) 226);
			assert (pack.chan2_scaled_GET() == (short) -4701);
			assert (pack.chan7_scaled_GET() == (short) -8692);
			assert (pack.chan3_scaled_GET() == (short) -24801);
			assert (pack.chan8_scaled_GET() == (short) -1793);
			assert (pack.chan5_scaled_GET() == (short) 17337);
			assert (pack.rssi_GET() == (char) 20);
			assert (pack.chan6_scaled_GET() == (short) -20823);
			assert (pack.chan1_scaled_GET() == (short) -25098);
			assert (pack.time_boot_ms_GET() == 3902887698L);
			assert (pack.chan4_scaled_GET() == (short) 24965);
		});
		RC_CHANNELS_SCALED p34 = new RC_CHANNELS_SCALED();
		PH.setPack(p34);
		p34.chan7_scaled_SET((short) -8692);
		p34.chan3_scaled_SET((short) -24801);
		p34.chan1_scaled_SET((short) -25098);
		p34.chan4_scaled_SET((short) 24965);
		p34.chan5_scaled_SET((short) 17337);
		p34.chan8_scaled_SET((short) -1793);
		p34.rssi_SET((char) 20);
		p34.port_SET((char) 226);
		p34.chan6_scaled_SET((short) -20823);
		p34.time_boot_ms_SET(3902887698L);
		p34.chan2_scaled_SET((short) -4701);
		TestChannel.instance.send(p34);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_RC_CHANNELS_RAW.add((src, ph, pack) ->
		{
			assert (pack.chan3_raw_GET() == (char) 39226);
			assert (pack.chan4_raw_GET() == (char) 38411);
			assert (pack.chan8_raw_GET() == (char) 52991);
			assert (pack.chan1_raw_GET() == (char) 51171);
			assert (pack.port_GET() == (char) 127);
			assert (pack.time_boot_ms_GET() == 4036820043L);
			assert (pack.chan6_raw_GET() == (char) 8840);
			assert (pack.chan5_raw_GET() == (char) 41564);
			assert (pack.chan2_raw_GET() == (char) 59191);
			assert (pack.chan7_raw_GET() == (char) 56840);
			assert (pack.rssi_GET() == (char) 252);
		});
		RC_CHANNELS_RAW p35 = new RC_CHANNELS_RAW();
		PH.setPack(p35);
		p35.chan1_raw_SET((char) 51171);
		p35.chan6_raw_SET((char) 8840);
		p35.chan3_raw_SET((char) 39226);
		p35.time_boot_ms_SET(4036820043L);
		p35.chan4_raw_SET((char) 38411);
		p35.chan2_raw_SET((char) 59191);
		p35.port_SET((char) 127);
		p35.rssi_SET((char) 252);
		p35.chan7_raw_SET((char) 56840);
		p35.chan5_raw_SET((char) 41564);
		p35.chan8_raw_SET((char) 52991);
		TestChannel.instance.send(p35);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_SERVO_OUTPUT_RAW.add((src, ph, pack) ->
		{
			assert (pack.servo3_raw_GET() == (char) 52249);
			assert (pack.servo6_raw_GET() == (char) 19236);
			assert (pack.servo5_raw_GET() == (char) 31908);
			assert (pack.servo12_raw_TRY(ph) == (char) 26021);
			assert (pack.port_GET() == (char) 139);
			assert (pack.servo2_raw_GET() == (char) 56441);
			assert (pack.time_usec_GET() == 4154687610L);
			assert (pack.servo10_raw_TRY(ph) == (char) 10057);
			assert (pack.servo7_raw_GET() == (char) 38798);
			assert (pack.servo9_raw_TRY(ph) == (char) 44999);
			assert (pack.servo15_raw_TRY(ph) == (char) 15404);
			assert (pack.servo14_raw_TRY(ph) == (char) 3010);
			assert (pack.servo8_raw_GET() == (char) 44183);
			assert (pack.servo11_raw_TRY(ph) == (char) 17022);
			assert (pack.servo4_raw_GET() == (char) 36049);
			assert (pack.servo1_raw_GET() == (char) 56077);
			assert (pack.servo13_raw_TRY(ph) == (char) 6981);
			assert (pack.servo16_raw_TRY(ph) == (char) 38359);
		});
		SERVO_OUTPUT_RAW p36 = new SERVO_OUTPUT_RAW();
		PH.setPack(p36);
		p36.servo12_raw_SET((char) 26021, PH);
		p36.servo16_raw_SET((char) 38359, PH);
		p36.servo11_raw_SET((char) 17022, PH);
		p36.servo2_raw_SET((char) 56441);
		p36.servo4_raw_SET((char) 36049);
		p36.servo8_raw_SET((char) 44183);
		p36.port_SET((char) 139);
		p36.servo1_raw_SET((char) 56077);
		p36.servo14_raw_SET((char) 3010, PH);
		p36.servo9_raw_SET((char) 44999, PH);
		p36.servo15_raw_SET((char) 15404, PH);
		p36.servo5_raw_SET((char) 31908);
		p36.servo6_raw_SET((char) 19236);
		p36.servo7_raw_SET((char) 38798);
		p36.servo10_raw_SET((char) 10057, PH);
		p36.servo13_raw_SET((char) 6981, PH);
		p36.servo3_raw_SET((char) 52249);
		p36.time_usec_SET(4154687610L);
		TestChannel.instance.send(p36);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_MISSION_REQUEST_PARTIAL_LIST.add((src, ph, pack) ->
		{
			assert (pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
			assert (pack.end_index_GET() == (short) -32503);
			assert (pack.start_index_GET() == (short) 16645);
			assert (pack.target_system_GET() == (char) 61);
			assert (pack.target_component_GET() == (char) 223);
		});
		MISSION_REQUEST_PARTIAL_LIST p37 = new MISSION_REQUEST_PARTIAL_LIST();
		PH.setPack(p37);
		p37.start_index_SET((short) 16645);
		p37.target_system_SET((char) 61);
		p37.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
		p37.end_index_SET((short) -32503);
		p37.target_component_SET((char) 223);
		TestChannel.instance.send(p37);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_MISSION_WRITE_PARTIAL_LIST.add((src, ph, pack) ->
		{
			assert (pack.target_system_GET() == (char) 242);
			assert (pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
			assert (pack.start_index_GET() == (short) -22826);
			assert (pack.end_index_GET() == (short) -6431);
			assert (pack.target_component_GET() == (char) 186);
		});
		MISSION_WRITE_PARTIAL_LIST p38 = new MISSION_WRITE_PARTIAL_LIST();
		PH.setPack(p38);
		p38.start_index_SET((short) -22826);
		p38.target_system_SET((char) 242);
		p38.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
		p38.target_component_SET((char) 186);
		p38.end_index_SET((short) -6431);
		TestChannel.instance.send(p38);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_MISSION_ITEM.add((src, ph, pack) ->
		{
			assert (pack.x_GET() == 8.3118493E36F);
			assert (pack.seq_GET() == (char) 16687);
			assert (pack.command_GET() == MAV_CMD.MAV_CMD_DO_MOUNT_CONTROL);
			assert (pack.frame_GET() == MAV_FRAME.MAV_FRAME_LOCAL_NED);
			assert (pack.z_GET() == 1.4934657E38F);
			assert (pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
			assert (pack.param1_GET() == 2.241217E38F);
			assert (pack.param2_GET() == 1.3901554E38F);
			assert (pack.y_GET() == -1.3328628E38F);
			assert (pack.current_GET() == (char) 36);
			assert (pack.param4_GET() == 3.3578551E38F);
			assert (pack.target_component_GET() == (char) 255);
			assert (pack.autocontinue_GET() == (char) 53);
			assert (pack.param3_GET() == 3.344983E38F);
			assert (pack.target_system_GET() == (char) 95);
		});
		MISSION_ITEM p39 = new MISSION_ITEM();
		PH.setPack(p39);
		p39.param4_SET(3.3578551E38F);
		p39.param3_SET(3.344983E38F);
		p39.command_SET(MAV_CMD.MAV_CMD_DO_MOUNT_CONTROL);
		p39.target_system_SET((char) 95);
		p39.y_SET(-1.3328628E38F);
		p39.param1_SET(2.241217E38F);
		p39.seq_SET((char) 16687);
		p39.target_component_SET((char) 255);
		p39.z_SET(1.4934657E38F);
		p39.current_SET((char) 36);
		p39.param2_SET(1.3901554E38F);
		p39.x_SET(8.3118493E36F);
		p39.autocontinue_SET((char) 53);
		p39.frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_NED);
		p39.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
		TestChannel.instance.send(p39);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_MISSION_REQUEST.add((src, ph, pack) ->
		{
			assert (pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
			assert (pack.target_component_GET() == (char) 241);
			assert (pack.target_system_GET() == (char) 90);
			assert (pack.seq_GET() == (char) 30221);
		});
		MISSION_REQUEST p40 = new MISSION_REQUEST();
		PH.setPack(p40);
		p40.seq_SET((char) 30221);
		p40.target_system_SET((char) 90);
		p40.target_component_SET((char) 241);
		p40.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
		TestChannel.instance.send(p40);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_MISSION_SET_CURRENT.add((src, ph, pack) ->
		{
			assert (pack.seq_GET() == (char) 36768);
			assert (pack.target_system_GET() == (char) 144);
			assert (pack.target_component_GET() == (char) 23);
		});
		MISSION_SET_CURRENT p41 = new MISSION_SET_CURRENT();
		PH.setPack(p41);
		p41.seq_SET((char) 36768);
		p41.target_component_SET((char) 23);
		p41.target_system_SET((char) 144);
		TestChannel.instance.send(p41);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_MISSION_CURRENT.add((src, ph, pack) ->
		{
			assert (pack.seq_GET() == (char) 40465);
		});
		MISSION_CURRENT p42 = new MISSION_CURRENT();
		PH.setPack(p42);
		p42.seq_SET((char) 40465);
		TestChannel.instance.send(p42);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_MISSION_REQUEST_LIST.add((src, ph, pack) ->
		{
			assert (pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
			assert (pack.target_component_GET() == (char) 130);
			assert (pack.target_system_GET() == (char) 168);
		});
		MISSION_REQUEST_LIST p43 = new MISSION_REQUEST_LIST();
		PH.setPack(p43);
		p43.target_component_SET((char) 130);
		p43.target_system_SET((char) 168);
		p43.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
		TestChannel.instance.send(p43);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_MISSION_COUNT.add((src, ph, pack) ->
		{
			assert (pack.target_component_GET() == (char) 228);
			assert (pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
			assert (pack.count_GET() == (char) 26363);
			assert (pack.target_system_GET() == (char) 10);
		});
		MISSION_COUNT p44 = new MISSION_COUNT();
		PH.setPack(p44);
		p44.count_SET((char) 26363);
		p44.target_component_SET((char) 228);
		p44.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
		p44.target_system_SET((char) 10);
		TestChannel.instance.send(p44);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_MISSION_CLEAR_ALL.add((src, ph, pack) ->
		{
			assert (pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
			assert (pack.target_system_GET() == (char) 159);
			assert (pack.target_component_GET() == (char) 123);
		});
		MISSION_CLEAR_ALL p45 = new MISSION_CLEAR_ALL();
		PH.setPack(p45);
		p45.target_component_SET((char) 123);
		p45.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
		p45.target_system_SET((char) 159);
		TestChannel.instance.send(p45);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_MISSION_ITEM_REACHED.add((src, ph, pack) ->
		{
			assert (pack.seq_GET() == (char) 57275);
		});
		MISSION_ITEM_REACHED p46 = new MISSION_ITEM_REACHED();
		PH.setPack(p46);
		p46.seq_SET((char) 57275);
		TestChannel.instance.send(p46);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_MISSION_ACK.add((src, ph, pack) ->
		{
			assert (pack.type_GET() == MAV_MISSION_RESULT.MAV_MISSION_NO_SPACE);
			assert (pack.target_system_GET() == (char) 52);
			assert (pack.target_component_GET() == (char) 166);
			assert (pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
		});
		MISSION_ACK p47 = new MISSION_ACK();
		PH.setPack(p47);
		p47.target_system_SET((char) 52);
		p47.type_SET(MAV_MISSION_RESULT.MAV_MISSION_NO_SPACE);
		p47.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
		p47.target_component_SET((char) 166);
		TestChannel.instance.send(p47);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_SET_GPS_GLOBAL_ORIGIN.add((src, ph, pack) ->
		{
			assert (pack.target_system_GET() == (char) 10);
			assert (pack.altitude_GET() == 1663750409);
			assert (pack.latitude_GET() == -1488798837);
			assert (pack.longitude_GET() == -2127578944);
			assert (pack.time_usec_TRY(ph) == 8182668434479207605L);
		});
		SET_GPS_GLOBAL_ORIGIN p48 = new SET_GPS_GLOBAL_ORIGIN();
		PH.setPack(p48);
		p48.longitude_SET(-2127578944);
		p48.latitude_SET(-1488798837);
		p48.time_usec_SET(8182668434479207605L, PH);
		p48.altitude_SET(1663750409);
		p48.target_system_SET((char) 10);
		TestChannel.instance.send(p48);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_GPS_GLOBAL_ORIGIN.add((src, ph, pack) ->
		{
			assert (pack.longitude_GET() == -1541163594);
			assert (pack.altitude_GET() == -1311838115);
			assert (pack.time_usec_TRY(ph) == 4193297278666874479L);
			assert (pack.latitude_GET() == 287423764);
		});
		GPS_GLOBAL_ORIGIN p49 = new GPS_GLOBAL_ORIGIN();
		PH.setPack(p49);
		p49.time_usec_SET(4193297278666874479L, PH);
		p49.latitude_SET(287423764);
		p49.longitude_SET(-1541163594);
		p49.altitude_SET(-1311838115);
		TestChannel.instance.send(p49);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_PARAM_MAP_RC.add((src, ph, pack) ->
		{
			assert (pack.param_value_min_GET() == -7.5679197E37F);
			assert (pack.param_value_max_GET() == 2.6767696E38F);
			assert (pack.scale_GET() == 4.1781236E37F);
			assert (pack.parameter_rc_channel_index_GET() == (char) 185);
			assert (pack.param_value0_GET() == 2.0562023E38F);
			assert (pack.param_id_LEN(ph) == 1);
			assert (pack.param_id_TRY(ph).equals("j"));
			assert (pack.target_system_GET() == (char) 53);
			assert (pack.param_index_GET() == (short) -26841);
			assert (pack.target_component_GET() == (char) 127);
		});
		PARAM_MAP_RC p50 = new PARAM_MAP_RC();
		PH.setPack(p50);
		p50.param_value_min_SET(-7.5679197E37F);
		p50.scale_SET(4.1781236E37F);
		p50.param_id_SET("j", PH);
		p50.param_value0_SET(2.0562023E38F);
		p50.param_index_SET((short) -26841);
		p50.target_system_SET((char) 53);
		p50.parameter_rc_channel_index_SET((char) 185);
		p50.target_component_SET((char) 127);
		p50.param_value_max_SET(2.6767696E38F);
		TestChannel.instance.send(p50);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_MISSION_REQUEST_INT.add((src, ph, pack) ->
		{
			assert (pack.seq_GET() == (char) 46779);
			assert (pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
			assert (pack.target_component_GET() == (char) 158);
			assert (pack.target_system_GET() == (char) 81);
		});
		MISSION_REQUEST_INT p51 = new MISSION_REQUEST_INT();
		PH.setPack(p51);
		p51.target_system_SET((char) 81);
		p51.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
		p51.target_component_SET((char) 158);
		p51.seq_SET((char) 46779);
		TestChannel.instance.send(p51);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_SAFETY_SET_ALLOWED_AREA.add((src, ph, pack) ->
		{
			assert (pack.p2y_GET() == 1.4196102E38F);
			assert (pack.target_system_GET() == (char) 168);
			assert (pack.p2x_GET() == 2.4206987E38F);
			assert (pack.target_component_GET() == (char) 205);
			assert (pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT);
			assert (pack.p1z_GET() == -1.1044949E38F);
			assert (pack.p1x_GET() == -1.4330172E38F);
			assert (pack.p2z_GET() == -1.3751317E38F);
			assert (pack.p1y_GET() == -8.3769754E37F);
		});
		SAFETY_SET_ALLOWED_AREA p54 = new SAFETY_SET_ALLOWED_AREA();
		PH.setPack(p54);
		p54.target_system_SET((char) 168);
		p54.p1x_SET(-1.4330172E38F);
		p54.p2y_SET(1.4196102E38F);
		p54.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT);
		p54.p2z_SET(-1.3751317E38F);
		p54.target_component_SET((char) 205);
		p54.p2x_SET(2.4206987E38F);
		p54.p1z_SET(-1.1044949E38F);
		p54.p1y_SET(-8.3769754E37F);
		TestChannel.instance.send(p54);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_SAFETY_ALLOWED_AREA.add((src, ph, pack) ->
		{
			assert (pack.frame_GET() == MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED);
			assert (pack.p1x_GET() == -3.3539362E38F);
			assert (pack.p2z_GET() == -2.269202E38F);
			assert (pack.p1y_GET() == -1.9493724E38F);
			assert (pack.p2x_GET() == -4.959215E37F);
			assert (pack.p1z_GET() == -1.977215E38F);
			assert (pack.p2y_GET() == 2.1028165E38F);
		});
		SAFETY_ALLOWED_AREA p55 = new SAFETY_ALLOWED_AREA();
		PH.setPack(p55);
		p55.p2x_SET(-4.959215E37F);
		p55.p1z_SET(-1.977215E38F);
		p55.p2y_SET(2.1028165E38F);
		p55.p1y_SET(-1.9493724E38F);
		p55.p2z_SET(-2.269202E38F);
		p55.frame_SET(MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED);
		p55.p1x_SET(-3.3539362E38F);
		TestChannel.instance.send(p55);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_ATTITUDE_QUATERNION_COV.add((src, ph, pack) ->
		{
			assert (pack.pitchspeed_GET() == 2.8282082E38F);
			assert (Arrays.equals(pack.q_GET(), new float[]{-2.0196546E38F, -2.6564766E38F, -2.0154375E38F, 2.521616E38F}));
			assert (Arrays.equals(pack.covariance_GET(), new float[]{-5.752468E37F, 3.3230793E38F, 7.2263325E37F, 3.2533157E37F, -3.088973E38F, -3.2272975E38F, 2.3942281E38F, -2.44236E38F, -1.6380113E38F}));
			assert (pack.rollspeed_GET() == 1.8831763E38F);
			assert (pack.yawspeed_GET() == 2.572085E38F);
			assert (pack.time_usec_GET() == 984393987827404717L);
		});
		ATTITUDE_QUATERNION_COV p61 = new ATTITUDE_QUATERNION_COV();
		PH.setPack(p61);
		p61.pitchspeed_SET(2.8282082E38F);
		p61.time_usec_SET(984393987827404717L);
		p61.q_SET(new float[]{-2.0196546E38F, -2.6564766E38F, -2.0154375E38F, 2.521616E38F}, 0);
		p61.rollspeed_SET(1.8831763E38F);
		p61.yawspeed_SET(2.572085E38F);
		p61.covariance_SET(new float[]{-5.752468E37F, 3.3230793E38F, 7.2263325E37F, 3.2533157E37F, -3.088973E38F, -3.2272975E38F, 2.3942281E38F, -2.44236E38F, -1.6380113E38F}, 0);
		TestChannel.instance.send(p61);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_NAV_CONTROLLER_OUTPUT.add((src, ph, pack) ->
		{
			assert (pack.nav_roll_GET() == -1.0392509E38F);
			assert (pack.nav_pitch_GET() == 3.3186336E38F);
			assert (pack.aspd_error_GET() == -2.9856208E38F);
			assert (pack.xtrack_error_GET() == 2.5809843E38F);
			assert (pack.alt_error_GET() == -2.4504023E38F);
			assert (pack.target_bearing_GET() == (short) -5426);
			assert (pack.nav_bearing_GET() == (short) -28960);
			assert (pack.wp_dist_GET() == (char) 5471);
		});
		NAV_CONTROLLER_OUTPUT p62 = new NAV_CONTROLLER_OUTPUT();
		PH.setPack(p62);
		p62.target_bearing_SET((short) -5426);
		p62.xtrack_error_SET(2.5809843E38F);
		p62.wp_dist_SET((char) 5471);
		p62.nav_bearing_SET((short) -28960);
		p62.alt_error_SET(-2.4504023E38F);
		p62.aspd_error_SET(-2.9856208E38F);
		p62.nav_pitch_SET(3.3186336E38F);
		p62.nav_roll_SET(-1.0392509E38F);
		TestChannel.instance.send(p62);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_GLOBAL_POSITION_INT_COV.add((src, ph, pack) ->
		{
			assert (Arrays.equals(pack.covariance_GET(), new float[]{3.3530233E38F, 3.3302986E38F, 2.9037823E38F, 1.2568523E38F, 1.205427E38F, -1.7702424E36F, -1.1538359E38F, 1.8656647E38F, 7.42424E37F, 2.0878246E38F, -8.206723E35F, -1.5707346E38F, -9.69828E37F, 1.3611589E38F, 2.9894779E38F, -2.374111E38F, 1.4928915E38F, -1.9732267E38F, 8.518842E37F, 1.9648315E38F, 2.9533797E38F, 2.101007E38F, -1.784521E38F, 1.2745396E38F, -1.0323577E38F, -1.4674357E38F, -2.3942253E38F, -1.2051564E38F, -8.855059E37F, 2.4484676E37F, -2.1465815E38F, 3.2520006E38F, -5.730641E37F, 2.4346337E38F, 2.3691051E38F, 2.1001644E38F}));
			assert (pack.alt_GET() == -1143685715);
			assert (pack.vx_GET() == 1.7243249E38F);
			assert (pack.relative_alt_GET() == -1162336555);
			assert (pack.lat_GET() == 650458475);
			assert (pack.vz_GET() == -6.704797E37F);
			assert (pack.time_usec_GET() == 3393994944400447436L);
			assert (pack.lon_GET() == 1295074459);
			assert (pack.vy_GET() == 6.258073E36F);
			assert (pack.estimator_type_GET() == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VISION);
		});
		GLOBAL_POSITION_INT_COV p63 = new GLOBAL_POSITION_INT_COV();
		PH.setPack(p63);
		p63.alt_SET(-1143685715);
		p63.time_usec_SET(3393994944400447436L);
		p63.relative_alt_SET(-1162336555);
		p63.vz_SET(-6.704797E37F);
		p63.lon_SET(1295074459);
		p63.vx_SET(1.7243249E38F);
		p63.lat_SET(650458475);
		p63.covariance_SET(new float[]{3.3530233E38F, 3.3302986E38F, 2.9037823E38F, 1.2568523E38F, 1.205427E38F, -1.7702424E36F, -1.1538359E38F, 1.8656647E38F, 7.42424E37F, 2.0878246E38F, -8.206723E35F, -1.5707346E38F, -9.69828E37F, 1.3611589E38F, 2.9894779E38F, -2.374111E38F, 1.4928915E38F, -1.9732267E38F, 8.518842E37F, 1.9648315E38F, 2.9533797E38F, 2.101007E38F, -1.784521E38F, 1.2745396E38F, -1.0323577E38F, -1.4674357E38F, -2.3942253E38F, -1.2051564E38F, -8.855059E37F, 2.4484676E37F, -2.1465815E38F, 3.2520006E38F, -5.730641E37F, 2.4346337E38F, 2.3691051E38F, 2.1001644E38F}, 0);
		p63.estimator_type_SET(MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VISION);
		p63.vy_SET(6.258073E36F);
		TestChannel.instance.send(p63);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_LOCAL_POSITION_NED_COV.add((src, ph, pack) ->
		{
			assert (pack.ax_GET() == 2.3043732E38F);
			assert (pack.z_GET() == 2.4929945E38F);
			assert (pack.vz_GET() == -1.7303419E38F);
			assert (pack.az_GET() == 1.4158971E38F);
			assert (pack.estimator_type_GET() == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS);
			assert (pack.ay_GET() == -1.8007066E38F);
			assert (pack.y_GET() == 2.5699052E38F);
			assert (pack.x_GET() == -6.0794505E37F);
			assert (pack.vy_GET() == 7.122175E37F);
			assert (pack.time_usec_GET() == 2932162305324405814L);
			assert (pack.vx_GET() == 2.2872857E38F);
			assert (Arrays.equals(pack.covariance_GET(), new float[]{-2.4136368E38F, -7.150273E37F, -2.7924598E38F, 2.2807136E37F, 3.349047E38F, -1.9011183E38F, -2.8094744E38F, 2.1118968E38F, 1.7332546E38F, -1.6498523E37F, 1.1373651E38F, 2.4861813E38F, 7.268807E37F, -1.0897728E38F, -1.3180677E38F, 1.7700366E38F, -2.3094715E36F, 3.314908E38F, -9.95028E37F, -1.7207671E38F, -2.0078616E38F, -1.8634133E38F, 3.1136933E38F, -8.3967097E37F, -2.007342E38F, 1.4775109E38F, 1.4375947E38F, 3.3895823E38F, 4.951109E37F, -3.09551E38F, 1.7404283E38F, 3.2122252E38F, 1.9414935E38F, -1.2318761E38F, -3.4413745E37F, 1.396648E38F, -1.6044613E38F, -2.4162465E38F, 7.985383E37F, -2.4177863E38F, -5.0853553E37F, -1.9208233E38F, 1.0618595E38F, -2.2622807E38F, -2.7897136E38F}));
		});
		LOCAL_POSITION_NED_COV p64 = new LOCAL_POSITION_NED_COV();
		PH.setPack(p64);
		p64.z_SET(2.4929945E38F);
		p64.estimator_type_SET(MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS);
		p64.time_usec_SET(2932162305324405814L);
		p64.ay_SET(-1.8007066E38F);
		p64.y_SET(2.5699052E38F);
		p64.covariance_SET(new float[]{-2.4136368E38F, -7.150273E37F, -2.7924598E38F, 2.2807136E37F, 3.349047E38F, -1.9011183E38F, -2.8094744E38F, 2.1118968E38F, 1.7332546E38F, -1.6498523E37F, 1.1373651E38F, 2.4861813E38F, 7.268807E37F, -1.0897728E38F, -1.3180677E38F, 1.7700366E38F, -2.3094715E36F, 3.314908E38F, -9.95028E37F, -1.7207671E38F, -2.0078616E38F, -1.8634133E38F, 3.1136933E38F, -8.3967097E37F, -2.007342E38F, 1.4775109E38F, 1.4375947E38F, 3.3895823E38F, 4.951109E37F, -3.09551E38F, 1.7404283E38F, 3.2122252E38F, 1.9414935E38F, -1.2318761E38F, -3.4413745E37F, 1.396648E38F, -1.6044613E38F, -2.4162465E38F, 7.985383E37F, -2.4177863E38F, -5.0853553E37F, -1.9208233E38F, 1.0618595E38F, -2.2622807E38F, -2.7897136E38F}, 0);
		p64.az_SET(1.4158971E38F);
		p64.x_SET(-6.0794505E37F);
		p64.ax_SET(2.3043732E38F);
		p64.vx_SET(2.2872857E38F);
		p64.vy_SET(7.122175E37F);
		p64.vz_SET(-1.7303419E38F);
		TestChannel.instance.send(p64);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_RC_CHANNELS.add((src, ph, pack) ->
		{
			assert (pack.chan15_raw_GET() == (char) 25116);
			assert (pack.chan16_raw_GET() == (char) 25580);
			assert (pack.chan3_raw_GET() == (char) 60894);
			assert (pack.chancount_GET() == (char) 153);
			assert (pack.chan1_raw_GET() == (char) 57701);
			assert (pack.chan11_raw_GET() == (char) 10925);
			assert (pack.chan6_raw_GET() == (char) 10417);
			assert (pack.chan7_raw_GET() == (char) 54257);
			assert (pack.chan4_raw_GET() == (char) 62362);
			assert (pack.chan8_raw_GET() == (char) 18759);
			assert (pack.chan10_raw_GET() == (char) 50683);
			assert (pack.chan5_raw_GET() == (char) 19834);
			assert (pack.chan14_raw_GET() == (char) 25205);
			assert (pack.rssi_GET() == (char) 124);
			assert (pack.chan9_raw_GET() == (char) 58192);
			assert (pack.chan13_raw_GET() == (char) 12428);
			assert (pack.chan2_raw_GET() == (char) 58894);
			assert (pack.chan17_raw_GET() == (char) 30179);
			assert (pack.chan12_raw_GET() == (char) 2275);
			assert (pack.chan18_raw_GET() == (char) 5183);
			assert (pack.time_boot_ms_GET() == 190631853L);
		});
		RC_CHANNELS p65 = new RC_CHANNELS();
		PH.setPack(p65);
		p65.chan7_raw_SET((char) 54257);
		p65.chan2_raw_SET((char) 58894);
		p65.chan8_raw_SET((char) 18759);
		p65.rssi_SET((char) 124);
		p65.chan15_raw_SET((char) 25116);
		p65.chan6_raw_SET((char) 10417);
		p65.chan18_raw_SET((char) 5183);
		p65.chan16_raw_SET((char) 25580);
		p65.chan11_raw_SET((char) 10925);
		p65.chancount_SET((char) 153);
		p65.chan1_raw_SET((char) 57701);
		p65.chan17_raw_SET((char) 30179);
		p65.chan13_raw_SET((char) 12428);
		p65.chan14_raw_SET((char) 25205);
		p65.chan9_raw_SET((char) 58192);
		p65.time_boot_ms_SET(190631853L);
		p65.chan10_raw_SET((char) 50683);
		p65.chan5_raw_SET((char) 19834);
		p65.chan3_raw_SET((char) 60894);
		p65.chan12_raw_SET((char) 2275);
		p65.chan4_raw_SET((char) 62362);
		TestChannel.instance.send(p65);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_REQUEST_DATA_STREAM.add((src, ph, pack) ->
		{
			assert (pack.req_message_rate_GET() == (char) 64734);
			assert (pack.req_stream_id_GET() == (char) 142);
			assert (pack.target_system_GET() == (char) 204);
			assert (pack.start_stop_GET() == (char) 53);
			assert (pack.target_component_GET() == (char) 83);
		});
		REQUEST_DATA_STREAM p66 = new REQUEST_DATA_STREAM();
		PH.setPack(p66);
		p66.req_stream_id_SET((char) 142);
		p66.start_stop_SET((char) 53);
		p66.target_system_SET((char) 204);
		p66.target_component_SET((char) 83);
		p66.req_message_rate_SET((char) 64734);
		TestChannel.instance.send(p66);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_DATA_STREAM.add((src, ph, pack) ->
		{
			assert (pack.stream_id_GET() == (char) 73);
			assert (pack.message_rate_GET() == (char) 63529);
			assert (pack.on_off_GET() == (char) 186);
		});
		DATA_STREAM p67 = new DATA_STREAM();
		PH.setPack(p67);
		p67.stream_id_SET((char) 73);
		p67.message_rate_SET((char) 63529);
		p67.on_off_SET((char) 186);
		TestChannel.instance.send(p67);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_MANUAL_CONTROL.add((src, ph, pack) ->
		{
			assert (pack.buttons_GET() == (char) 39035);
			assert (pack.target_GET() == (char) 178);
			assert (pack.r_GET() == (short) 25578);
			assert (pack.z_GET() == (short) -21022);
			assert (pack.x_GET() == (short) -23857);
			assert (pack.y_GET() == (short) 10320);
		});
		MANUAL_CONTROL p69 = new MANUAL_CONTROL();
		PH.setPack(p69);
		p69.x_SET((short) -23857);
		p69.z_SET((short) -21022);
		p69.target_SET((char) 178);
		p69.y_SET((short) 10320);
		p69.r_SET((short) 25578);
		p69.buttons_SET((char) 39035);
		TestChannel.instance.send(p69);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_RC_CHANNELS_OVERRIDE.add((src, ph, pack) ->
		{
			assert (pack.chan1_raw_GET() == (char) 1424);
			assert (pack.target_component_GET() == (char) 188);
			assert (pack.chan5_raw_GET() == (char) 59536);
			assert (pack.chan7_raw_GET() == (char) 6903);
			assert (pack.target_system_GET() == (char) 190);
			assert (pack.chan3_raw_GET() == (char) 45118);
			assert (pack.chan2_raw_GET() == (char) 40315);
			assert (pack.chan6_raw_GET() == (char) 46616);
			assert (pack.chan4_raw_GET() == (char) 61589);
			assert (pack.chan8_raw_GET() == (char) 38435);
		});
		RC_CHANNELS_OVERRIDE p70 = new RC_CHANNELS_OVERRIDE();
		PH.setPack(p70);
		p70.chan3_raw_SET((char) 45118);
		p70.chan6_raw_SET((char) 46616);
		p70.chan2_raw_SET((char) 40315);
		p70.chan1_raw_SET((char) 1424);
		p70.chan7_raw_SET((char) 6903);
		p70.chan5_raw_SET((char) 59536);
		p70.chan8_raw_SET((char) 38435);
		p70.target_system_SET((char) 190);
		p70.chan4_raw_SET((char) 61589);
		p70.target_component_SET((char) 188);
		TestChannel.instance.send(p70);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_MISSION_ITEM_INT.add((src, ph, pack) ->
		{
			assert (pack.target_system_GET() == (char) 53);
			assert (pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
			assert (pack.y_GET() == 1881587804);
			assert (pack.frame_GET() == MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED);
			assert (pack.x_GET() == -1338002733);
			assert (pack.param1_GET() == -3.1128256E38F);
			assert (pack.param4_GET() == -1.9518572E38F);
			assert (pack.param3_GET() == 3.103976E38F);
			assert (pack.current_GET() == (char) 124);
			assert (pack.target_component_GET() == (char) 65);
			assert (pack.z_GET() == -2.8469642E38F);
			assert (pack.command_GET() == MAV_CMD.MAV_CMD_STORAGE_FORMAT);
			assert (pack.autocontinue_GET() == (char) 165);
			assert (pack.seq_GET() == (char) 56199);
			assert (pack.param2_GET() == 2.3942109E38F);
		});
		MISSION_ITEM_INT p73 = new MISSION_ITEM_INT();
		PH.setPack(p73);
		p73.current_SET((char) 124);
		p73.seq_SET((char) 56199);
		p73.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
		p73.param2_SET(2.3942109E38F);
		p73.param4_SET(-1.9518572E38F);
		p73.target_system_SET((char) 53);
		p73.autocontinue_SET((char) 165);
		p73.x_SET(-1338002733);
		p73.param3_SET(3.103976E38F);
		p73.y_SET(1881587804);
		p73.param1_SET(-3.1128256E38F);
		p73.command_SET(MAV_CMD.MAV_CMD_STORAGE_FORMAT);
		p73.frame_SET(MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED);
		p73.target_component_SET((char) 65);
		p73.z_SET(-2.8469642E38F);
		TestChannel.instance.send(p73);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_VFR_HUD.add((src, ph, pack) ->
		{
			assert (pack.alt_GET() == 2.6036764E38F);
			assert (pack.airspeed_GET() == -2.066389E38F);
			assert (pack.groundspeed_GET() == 5.2273023E37F);
			assert (pack.throttle_GET() == (char) 16562);
			assert (pack.heading_GET() == (short) 14913);
			assert (pack.climb_GET() == 1.4837915E38F);
		});
		GroundControl.VFR_HUD p74 = CommunicationChannel.new_VFR_HUD();
		PH.setPack(p74);
		p74.climb_SET(1.4837915E38F);
		p74.alt_SET(2.6036764E38F);
		p74.throttle_SET((char) 16562);
		p74.groundspeed_SET(5.2273023E37F);
		p74.heading_SET((short) 14913);
		p74.airspeed_SET(-2.066389E38F);
		CommunicationChannel.instance.send(p74);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_COMMAND_INT.add((src, ph, pack) ->
		{
			assert (pack.param4_GET() == -2.9850752E38F);
			assert (pack.z_GET() == 1.2542754E37F);
			assert (pack.param2_GET() == -2.886268E37F);
			assert (pack.x_GET() == 348472208);
			assert (pack.target_component_GET() == (char) 68);
			assert (pack.param1_GET() == 5.796195E37F);
			assert (pack.autocontinue_GET() == (char) 90);
			assert (pack.current_GET() == (char) 139);
			assert (pack.param3_GET() == -2.9784966E38F);
			assert (pack.frame_GET() == MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED);
			assert (pack.target_system_GET() == (char) 163);
			assert (pack.y_GET() == -152957941);
			assert (pack.command_GET() == MAV_CMD.MAV_CMD_WAYPOINT_USER_3);
		});
		GroundControl.COMMAND_INT p75 = CommunicationChannel.new_COMMAND_INT();
		PH.setPack(p75);
		p75.param4_SET(-2.9850752E38F);
		p75.autocontinue_SET((char) 90);
		p75.target_component_SET((char) 68);
		p75.param1_SET(5.796195E37F);
		p75.x_SET(348472208);
		p75.param2_SET(-2.886268E37F);
		p75.command_SET(MAV_CMD.MAV_CMD_WAYPOINT_USER_3);
		p75.z_SET(1.2542754E37F);
		p75.param3_SET(-2.9784966E38F);
		p75.target_system_SET((char) 163);
		p75.y_SET(-152957941);
		p75.frame_SET(MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED);
		p75.current_SET((char) 139);
		CommunicationChannel.instance.send(p75);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_COMMAND_LONG.add((src, ph, pack) ->
		{
			assert (pack.param6_GET() == 6.86507E37F);
			assert (pack.command_GET() == MAV_CMD.MAV_CMD_DO_SET_CAM_TRIGG_DIST);
			assert (pack.param1_GET() == -1.0159776E38F);
			assert (pack.target_system_GET() == (char) 226);
			assert (pack.param4_GET() == 6.31711E37F);
			assert (pack.param5_GET() == -2.0437703E37F);
			assert (pack.confirmation_GET() == (char) 8);
			assert (pack.param2_GET() == -2.6103066E38F);
			assert (pack.param7_GET() == 1.7507137E38F);
			assert (pack.param3_GET() == 3.3669086E38F);
			assert (pack.target_component_GET() == (char) 136);
		});
		GroundControl.COMMAND_LONG p76 = CommunicationChannel.new_COMMAND_LONG();
		PH.setPack(p76);
		p76.param4_SET(6.31711E37F);
		p76.command_SET(MAV_CMD.MAV_CMD_DO_SET_CAM_TRIGG_DIST);
		p76.param6_SET(6.86507E37F);
		p76.confirmation_SET((char) 8);
		p76.target_component_SET((char) 136);
		p76.param7_SET(1.7507137E38F);
		p76.param3_SET(3.3669086E38F);
		p76.target_system_SET((char) 226);
		p76.param2_SET(-2.6103066E38F);
		p76.param1_SET(-1.0159776E38F);
		p76.param5_SET(-2.0437703E37F);
		CommunicationChannel.instance.send(p76);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_COMMAND_ACK.add((src, ph, pack) ->
		{
			assert (pack.result_GET() == MAV_RESULT.MAV_RESULT_TEMPORARILY_REJECTED);
			assert (pack.target_component_TRY(ph) == (char) 243);
			assert (pack.command_GET() == MAV_CMD.MAV_CMD_DO_FOLLOW_REPOSITION);
			assert (pack.progress_TRY(ph) == (char) 27);
			assert (pack.result_param2_TRY(ph) == 2135451893);
			assert (pack.target_system_TRY(ph) == (char) 111);
		});
		GroundControl.COMMAND_ACK p77 = CommunicationChannel.new_COMMAND_ACK();
		PH.setPack(p77);
		p77.result_param2_SET(2135451893, PH);
		p77.progress_SET((char) 27, PH);
		p77.target_system_SET((char) 111, PH);
		p77.result_SET(MAV_RESULT.MAV_RESULT_TEMPORARILY_REJECTED);
		p77.target_component_SET((char) 243, PH);
		p77.command_SET(MAV_CMD.MAV_CMD_DO_FOLLOW_REPOSITION);
		CommunicationChannel.instance.send(p77);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_MANUAL_SETPOINT.add((src, ph, pack) ->
		{
			assert (pack.pitch_GET() == 2.0189565E38F);
			assert (pack.thrust_GET() == -1.0575491E38F);
			assert (pack.mode_switch_GET() == (char) 179);
			assert (pack.manual_override_switch_GET() == (char) 179);
			assert (pack.time_boot_ms_GET() == 2111547633L);
			assert (pack.yaw_GET() == 3.318902E38F);
			assert (pack.roll_GET() == -3.1211217E38F);
		});
		GroundControl.MANUAL_SETPOINT p81 = CommunicationChannel.new_MANUAL_SETPOINT();
		PH.setPack(p81);
		p81.time_boot_ms_SET(2111547633L);
		p81.manual_override_switch_SET((char) 179);
		p81.pitch_SET(2.0189565E38F);
		p81.roll_SET(-3.1211217E38F);
		p81.yaw_SET(3.318902E38F);
		p81.thrust_SET(-1.0575491E38F);
		p81.mode_switch_SET((char) 179);
		CommunicationChannel.instance.send(p81);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_SET_ATTITUDE_TARGET.add((src, ph, pack) ->
		{
			assert (pack.time_boot_ms_GET() == 4013271682L);
			assert (pack.body_pitch_rate_GET() == 1.9775796E38F);
			assert (pack.target_component_GET() == (char) 155);
			assert (pack.body_roll_rate_GET() == 1.0605331E38F);
			assert (pack.target_system_GET() == (char) 54);
			assert (pack.type_mask_GET() == (char) 148);
			assert (pack.body_yaw_rate_GET() == 1.7933333E38F);
			assert (Arrays.equals(pack.q_GET(), new float[]{-4.485249E37F, -1.7181298E38F, 2.355547E38F, -1.1112753E38F}));
			assert (pack.thrust_GET() == -3.0687434E38F);
		});
		GroundControl.SET_ATTITUDE_TARGET p82 = CommunicationChannel.new_SET_ATTITUDE_TARGET();
		PH.setPack(p82);
		p82.body_roll_rate_SET(1.0605331E38F);
		p82.q_SET(new float[]{-4.485249E37F, -1.7181298E38F, 2.355547E38F, -1.1112753E38F}, 0);
		p82.target_system_SET((char) 54);
		p82.time_boot_ms_SET(4013271682L);
		p82.body_yaw_rate_SET(1.7933333E38F);
		p82.thrust_SET(-3.0687434E38F);
		p82.type_mask_SET((char) 148);
		p82.body_pitch_rate_SET(1.9775796E38F);
		p82.target_component_SET((char) 155);
		CommunicationChannel.instance.send(p82);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_ATTITUDE_TARGET.add((src, ph, pack) ->
		{
			assert (pack.body_yaw_rate_GET() == -2.4194152E38F);
			assert (pack.thrust_GET() == -1.7644684E38F);
			assert (pack.time_boot_ms_GET() == 2036650231L);
			assert (pack.type_mask_GET() == (char) 201);
			assert (pack.body_pitch_rate_GET() == 1.009558E38F);
			assert (pack.body_roll_rate_GET() == 6.5617337E37F);
			assert (Arrays.equals(pack.q_GET(), new float[]{2.9508087E38F, -2.332801E38F, 3.8836687E37F, -1.3876747E38F}));
		});
		GroundControl.ATTITUDE_TARGET p83 = CommunicationChannel.new_ATTITUDE_TARGET();
		PH.setPack(p83);
		p83.body_yaw_rate_SET(-2.4194152E38F);
		p83.time_boot_ms_SET(2036650231L);
		p83.q_SET(new float[]{2.9508087E38F, -2.332801E38F, 3.8836687E37F, -1.3876747E38F}, 0);
		p83.type_mask_SET((char) 201);
		p83.body_roll_rate_SET(6.5617337E37F);
		p83.thrust_SET(-1.7644684E38F);
		p83.body_pitch_rate_SET(1.009558E38F);
		CommunicationChannel.instance.send(p83);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_SET_POSITION_TARGET_LOCAL_NED.add((src, ph, pack) ->
		{
			assert (pack.yaw_GET() == -3.1396008E38F);
			assert (pack.afz_GET() == 3.2841833E38F);
			assert (pack.afx_GET() == -9.163358E37F);
			assert (pack.y_GET() == 1.528789E38F);
			assert (pack.afy_GET() == 3.0720389E38F);
			assert (pack.target_component_GET() == (char) 201);
			assert (pack.target_system_GET() == (char) 90);
			assert (pack.vx_GET() == -3.618844E35F);
			assert (pack.x_GET() == -9.199411E37F);
			assert (pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT);
			assert (pack.yaw_rate_GET() == -2.613236E37F);
			assert (pack.vz_GET() == 2.8419877E38F);
			assert (pack.time_boot_ms_GET() == 4156432778L);
			assert (pack.z_GET() == -1.7244524E37F);
			assert (pack.type_mask_GET() == (char) 27707);
			assert (pack.vy_GET() == 3.3352948E38F);
		});
		GroundControl.SET_POSITION_TARGET_LOCAL_NED p84 = CommunicationChannel.new_SET_POSITION_TARGET_LOCAL_NED();
		PH.setPack(p84);
		p84.x_SET(-9.199411E37F);
		p84.afy_SET(3.0720389E38F);
		p84.type_mask_SET((char) 27707);
		p84.z_SET(-1.7244524E37F);
		p84.target_system_SET((char) 90);
		p84.y_SET(1.528789E38F);
		p84.target_component_SET((char) 201);
		p84.vz_SET(2.8419877E38F);
		p84.yaw_rate_SET(-2.613236E37F);
		p84.vy_SET(3.3352948E38F);
		p84.afx_SET(-9.163358E37F);
		p84.vx_SET(-3.618844E35F);
		p84.yaw_SET(-3.1396008E38F);
		p84.afz_SET(3.2841833E38F);
		p84.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT);
		p84.time_boot_ms_SET(4156432778L);
		CommunicationChannel.instance.send(p84);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_SET_POSITION_TARGET_GLOBAL_INT.add((src, ph, pack) ->
		{
			assert (pack.alt_GET() == -7.078643E37F);
			assert (pack.type_mask_GET() == (char) 46676);
			assert (pack.afx_GET() == 1.3810933E38F);
			assert (pack.time_boot_ms_GET() == 1075065583L);
			assert (pack.afy_GET() == 1.0417124E38F);
			assert (pack.yaw_GET() == 2.5546034E38F);
			assert (pack.target_component_GET() == (char) 182);
			assert (pack.vz_GET() == -1.4098941E38F);
			assert (pack.target_system_GET() == (char) 68);
			assert (pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED);
			assert (pack.yaw_rate_GET() == 6.5267486E37F);
			assert (pack.vx_GET() == -1.3410552E37F);
			assert (pack.vy_GET() == -3.7777542E37F);
			assert (pack.afz_GET() == -1.5384314E38F);
			assert (pack.lat_int_GET() == -2143368874);
			assert (pack.lon_int_GET() == 1383455988);
		});
		GroundControl.SET_POSITION_TARGET_GLOBAL_INT p86 = CommunicationChannel.new_SET_POSITION_TARGET_GLOBAL_INT();
		PH.setPack(p86);
		p86.vx_SET(-1.3410552E37F);
		p86.yaw_rate_SET(6.5267486E37F);
		p86.target_component_SET((char) 182);
		p86.afx_SET(1.3810933E38F);
		p86.lon_int_SET(1383455988);
		p86.type_mask_SET((char) 46676);
		p86.target_system_SET((char) 68);
		p86.lat_int_SET(-2143368874);
		p86.vz_SET(-1.4098941E38F);
		p86.alt_SET(-7.078643E37F);
		p86.afz_SET(-1.5384314E38F);
		p86.yaw_SET(2.5546034E38F);
		p86.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED);
		p86.time_boot_ms_SET(1075065583L);
		p86.afy_SET(1.0417124E38F);
		p86.vy_SET(-3.7777542E37F);
		CommunicationChannel.instance.send(p86);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_POSITION_TARGET_GLOBAL_INT.add((src, ph, pack) ->
		{
			assert (pack.vy_GET() == 2.6268298E38F);
			assert (pack.time_boot_ms_GET() == 3991217284L);
			assert (pack.yaw_rate_GET() == -3.2405171E38F);
			assert (pack.vz_GET() == -1.8372036E38F);
			assert (pack.type_mask_GET() == (char) 50831);
			assert (pack.afy_GET() == 2.7514827E38F);
			assert (pack.afz_GET() == -8.138673E37F);
			assert (pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_LOCAL_NED);
			assert (pack.yaw_GET() == 1.951532E38F);
			assert (pack.vx_GET() == -2.70087E38F);
			assert (pack.lon_int_GET() == 1745230481);
			assert (pack.lat_int_GET() == -386800077);
			assert (pack.afx_GET() == 1.4367392E37F);
			assert (pack.alt_GET() == -1.3111044E38F);
		});
		GroundControl.POSITION_TARGET_GLOBAL_INT p87 = CommunicationChannel.new_POSITION_TARGET_GLOBAL_INT();
		PH.setPack(p87);
		p87.vx_SET(-2.70087E38F);
		p87.yaw_rate_SET(-3.2405171E38F);
		p87.lat_int_SET(-386800077);
		p87.type_mask_SET((char) 50831);
		p87.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_NED);
		p87.yaw_SET(1.951532E38F);
		p87.afz_SET(-8.138673E37F);
		p87.vz_SET(-1.8372036E38F);
		p87.vy_SET(2.6268298E38F);
		p87.afx_SET(1.4367392E37F);
		p87.time_boot_ms_SET(3991217284L);
		p87.lon_int_SET(1745230481);
		p87.alt_SET(-1.3111044E38F);
		p87.afy_SET(2.7514827E38F);
		CommunicationChannel.instance.send(p87);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET.add((src, ph, pack) ->
		{
			assert (pack.time_boot_ms_GET() == 1238331112L);
			assert (pack.z_GET() == -2.7454977E38F);
			assert (pack.roll_GET() == -1.4743692E38F);
			assert (pack.yaw_GET() == -2.7765866E38F);
			assert (pack.y_GET() == 2.0484266E38F);
			assert (pack.pitch_GET() == 2.6856916E38F);
			assert (pack.x_GET() == -2.768052E37F);
		});
		GroundControl.LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET p89 = CommunicationChannel.new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
		PH.setPack(p89);
		p89.time_boot_ms_SET(1238331112L);
		p89.y_SET(2.0484266E38F);
		p89.x_SET(-2.768052E37F);
		p89.yaw_SET(-2.7765866E38F);
		p89.z_SET(-2.7454977E38F);
		p89.roll_SET(-1.4743692E38F);
		p89.pitch_SET(2.6856916E38F);
		CommunicationChannel.instance.send(p89);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_HIL_STATE.add((src, ph, pack) ->
		{
			assert (pack.vx_GET() == (short) -16852);
			assert (pack.time_usec_GET() == 2584686517189733966L);
			assert (pack.vy_GET() == (short) -10110);
			assert (pack.yacc_GET() == (short) -7909);
			assert (pack.vz_GET() == (short) -16127);
			assert (pack.pitch_GET() == 2.8935628E38F);
			assert (pack.xacc_GET() == (short) -28490);
			assert (pack.yawspeed_GET() == -3.2908362E38F);
			assert (pack.lon_GET() == -613177426);
			assert (pack.rollspeed_GET() == 4.2396648E37F);
			assert (pack.zacc_GET() == (short) -12765);
			assert (pack.pitchspeed_GET() == 2.629001E38F);
			assert (pack.alt_GET() == 1514135442);
			assert (pack.roll_GET() == 1.1653119E37F);
			assert (pack.yaw_GET() == 2.7755167E38F);
			assert (pack.lat_GET() == -625422029);
		});
		GroundControl.HIL_STATE p90 = CommunicationChannel.new_HIL_STATE();
		PH.setPack(p90);
		p90.zacc_SET((short) -12765);
		p90.rollspeed_SET(4.2396648E37F);
		p90.lon_SET(-613177426);
		p90.pitch_SET(2.8935628E38F);
		p90.alt_SET(1514135442);
		p90.yaw_SET(2.7755167E38F);
		p90.roll_SET(1.1653119E37F);
		p90.pitchspeed_SET(2.629001E38F);
		p90.vy_SET((short) -10110);
		p90.vx_SET((short) -16852);
		p90.lat_SET(-625422029);
		p90.yacc_SET((short) -7909);
		p90.yawspeed_SET(-3.2908362E38F);
		p90.time_usec_SET(2584686517189733966L);
		p90.vz_SET((short) -16127);
		p90.xacc_SET((short) -28490);
		CommunicationChannel.instance.send(p90);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_HIL_CONTROLS.add((src, ph, pack) ->
		{
			assert (pack.throttle_GET() == -2.3137341E38F);
			assert (pack.roll_ailerons_GET() == -2.7051262E38F);
			assert (pack.aux3_GET() == -1.5398835E38F);
			assert (pack.aux4_GET() == 1.2857834E38F);
			assert (pack.aux2_GET() == -2.6757652E38F);
			assert (pack.pitch_elevator_GET() == 2.0727521E38F);
			assert (pack.nav_mode_GET() == (char) 210);
			assert (pack.aux1_GET() == -8.751097E37F);
			assert (pack.mode_GET() == MAV_MODE.MAV_MODE_AUTO_DISARMED);
			assert (pack.time_usec_GET() == 8822047435742283421L);
			assert (pack.yaw_rudder_GET() == -9.458866E37F);
		});
		GroundControl.HIL_CONTROLS p91 = CommunicationChannel.new_HIL_CONTROLS();
		PH.setPack(p91);
		p91.time_usec_SET(8822047435742283421L);
		p91.nav_mode_SET((char) 210);
		p91.aux2_SET(-2.6757652E38F);
		p91.throttle_SET(-2.3137341E38F);
		p91.aux3_SET(-1.5398835E38F);
		p91.pitch_elevator_SET(2.0727521E38F);
		p91.yaw_rudder_SET(-9.458866E37F);
		p91.aux4_SET(1.2857834E38F);
		p91.roll_ailerons_SET(-2.7051262E38F);
		p91.aux1_SET(-8.751097E37F);
		p91.mode_SET(MAV_MODE.MAV_MODE_AUTO_DISARMED);
		CommunicationChannel.instance.send(p91);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_HIL_RC_INPUTS_RAW.add((src, ph, pack) ->
		{
			assert (pack.chan7_raw_GET() == (char) 15224);
			assert (pack.chan2_raw_GET() == (char) 32127);
			assert (pack.chan9_raw_GET() == (char) 16283);
			assert (pack.rssi_GET() == (char) 22);
			assert (pack.chan1_raw_GET() == (char) 34184);
			assert (pack.chan10_raw_GET() == (char) 7276);
			assert (pack.time_usec_GET() == 6509186386881183942L);
			assert (pack.chan8_raw_GET() == (char) 54641);
			assert (pack.chan12_raw_GET() == (char) 12403);
			assert (pack.chan6_raw_GET() == (char) 4802);
			assert (pack.chan11_raw_GET() == (char) 47816);
			assert (pack.chan3_raw_GET() == (char) 55334);
			assert (pack.chan5_raw_GET() == (char) 41694);
			assert (pack.chan4_raw_GET() == (char) 63290);
		});
		GroundControl.HIL_RC_INPUTS_RAW p92 = CommunicationChannel.new_HIL_RC_INPUTS_RAW();
		PH.setPack(p92);
		p92.chan9_raw_SET((char) 16283);
		p92.chan5_raw_SET((char) 41694);
		p92.chan2_raw_SET((char) 32127);
		p92.time_usec_SET(6509186386881183942L);
		p92.chan11_raw_SET((char) 47816);
		p92.chan8_raw_SET((char) 54641);
		p92.chan12_raw_SET((char) 12403);
		p92.chan3_raw_SET((char) 55334);
		p92.chan10_raw_SET((char) 7276);
		p92.chan6_raw_SET((char) 4802);
		p92.chan7_raw_SET((char) 15224);
		p92.rssi_SET((char) 22);
		p92.chan4_raw_SET((char) 63290);
		p92.chan1_raw_SET((char) 34184);
		CommunicationChannel.instance.send(p92);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_HIL_ACTUATOR_CONTROLS.add((src, ph, pack) ->
		{
			assert (pack.flags_GET() == 442773187808513773L);
			assert (pack.time_usec_GET() == 7699741440109934344L);
			assert (Arrays.equals(pack.controls_GET(), new float[]{2.0529741E38F, 4.504816E37F, -3.0073413E36F, 2.588583E38F, 2.1579488E38F, -2.9636691E38F, -1.6900515E38F, -1.2217549E38F, -3.1597473E38F, 2.291401E37F, 3.0183506E37F, 5.415301E37F, -2.2479956E38F, -3.2466071E38F, -2.9594452E37F, 1.9724149E37F}));
			assert (pack.mode_GET() == MAV_MODE.MAV_MODE_MANUAL_ARMED);
		});
		GroundControl.HIL_ACTUATOR_CONTROLS p93 = CommunicationChannel.new_HIL_ACTUATOR_CONTROLS();
		PH.setPack(p93);
		p93.controls_SET(new float[]{2.0529741E38F, 4.504816E37F, -3.0073413E36F, 2.588583E38F, 2.1579488E38F, -2.9636691E38F, -1.6900515E38F, -1.2217549E38F, -3.1597473E38F, 2.291401E37F, 3.0183506E37F, 5.415301E37F, -2.2479956E38F, -3.2466071E38F, -2.9594452E37F, 1.9724149E37F}, 0);
		p93.time_usec_SET(7699741440109934344L);
		p93.flags_SET(442773187808513773L);
		p93.mode_SET(MAV_MODE.MAV_MODE_MANUAL_ARMED);
		CommunicationChannel.instance.send(p93);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_OPTICAL_FLOW.add((src, ph, pack) ->
		{
			assert (pack.flow_comp_m_y_GET() == -7.676789E37F);
			assert (pack.flow_x_GET() == (short) 10336);
			assert (pack.ground_distance_GET() == -4.4328773E37F);
			assert (pack.quality_GET() == (char) 17);
			assert (pack.flow_y_GET() == (short) -4548);
			assert (pack.time_usec_GET() == 3824142067777582122L);
			assert (pack.flow_rate_y_TRY(ph) == 1.342143E38F);
			assert (pack.flow_comp_m_x_GET() == 2.575609E38F);
			assert (pack.flow_rate_x_TRY(ph) == 8.326011E37F);
			assert (pack.sensor_id_GET() == (char) 223);
		});
		GroundControl.OPTICAL_FLOW p100 = CommunicationChannel.new_OPTICAL_FLOW();
		PH.setPack(p100);
		p100.flow_rate_x_SET(8.326011E37F, PH);
		p100.flow_y_SET((short) -4548);
		p100.flow_x_SET((short) 10336);
		p100.flow_comp_m_y_SET(-7.676789E37F);
		p100.flow_rate_y_SET(1.342143E38F, PH);
		p100.ground_distance_SET(-4.4328773E37F);
		p100.flow_comp_m_x_SET(2.575609E38F);
		p100.quality_SET((char) 17);
		p100.sensor_id_SET((char) 223);
		p100.time_usec_SET(3824142067777582122L);
		CommunicationChannel.instance.send(p100);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_GLOBAL_VISION_POSITION_ESTIMATE.add((src, ph, pack) ->
		{
			assert (pack.usec_GET() == 1894892874656080049L);
			assert (pack.roll_GET() == -1.4724607E38F);
			assert (pack.y_GET() == -3.1227972E38F);
			assert (pack.yaw_GET() == -2.694942E38F);
			assert (pack.pitch_GET() == 8.532043E37F);
			assert (pack.z_GET() == -3.0299827E38F);
			assert (pack.x_GET() == -3.1797727E38F);
		});
		GroundControl.GLOBAL_VISION_POSITION_ESTIMATE p101 = CommunicationChannel.new_GLOBAL_VISION_POSITION_ESTIMATE();
		PH.setPack(p101);
		p101.usec_SET(1894892874656080049L);
		p101.yaw_SET(-2.694942E38F);
		p101.pitch_SET(8.532043E37F);
		p101.z_SET(-3.0299827E38F);
		p101.roll_SET(-1.4724607E38F);
		p101.x_SET(-3.1797727E38F);
		p101.y_SET(-3.1227972E38F);
		CommunicationChannel.instance.send(p101);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_VISION_POSITION_ESTIMATE.add((src, ph, pack) ->
		{
			assert (pack.yaw_GET() == -2.8254516E38F);
			assert (pack.roll_GET() == 1.4132678E38F);
			assert (pack.y_GET() == -3.1956179E37F);
			assert (pack.z_GET() == 1.5733148E37F);
			assert (pack.x_GET() == 7.725394E36F);
			assert (pack.pitch_GET() == 2.6125948E38F);
			assert (pack.usec_GET() == 5766813804302523897L);
		});
		GroundControl.VISION_POSITION_ESTIMATE p102 = CommunicationChannel.new_VISION_POSITION_ESTIMATE();
		PH.setPack(p102);
		p102.roll_SET(1.4132678E38F);
		p102.y_SET(-3.1956179E37F);
		p102.z_SET(1.5733148E37F);
		p102.x_SET(7.725394E36F);
		p102.usec_SET(5766813804302523897L);
		p102.yaw_SET(-2.8254516E38F);
		p102.pitch_SET(2.6125948E38F);
		CommunicationChannel.instance.send(p102);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_VISION_SPEED_ESTIMATE.add((src, ph, pack) ->
		{
			assert (pack.z_GET() == -1.3777956E38F);
			assert (pack.x_GET() == 2.8448613E38F);
			assert (pack.usec_GET() == 477377851516315370L);
			assert (pack.y_GET() == -2.6757208E38F);
		});
		GroundControl.VISION_SPEED_ESTIMATE p103 = CommunicationChannel.new_VISION_SPEED_ESTIMATE();
		PH.setPack(p103);
		p103.z_SET(-1.3777956E38F);
		p103.usec_SET(477377851516315370L);
		p103.y_SET(-2.6757208E38F);
		p103.x_SET(2.8448613E38F);
		CommunicationChannel.instance.send(p103);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_VICON_POSITION_ESTIMATE.add((src, ph, pack) ->
		{
			assert (pack.z_GET() == 2.9949513E38F);
			assert (pack.roll_GET() == -1.2439693E38F);
			assert (pack.usec_GET() == 7613499391384487701L);
			assert (pack.pitch_GET() == 6.274516E37F);
			assert (pack.x_GET() == -6.51957E37F);
			assert (pack.yaw_GET() == 1.7537265E38F);
			assert (pack.y_GET() == -1.1897097E37F);
		});
		GroundControl.VICON_POSITION_ESTIMATE p104 = CommunicationChannel.new_VICON_POSITION_ESTIMATE();
		PH.setPack(p104);
		p104.yaw_SET(1.7537265E38F);
		p104.roll_SET(-1.2439693E38F);
		p104.x_SET(-6.51957E37F);
		p104.pitch_SET(6.274516E37F);
		p104.usec_SET(7613499391384487701L);
		p104.y_SET(-1.1897097E37F);
		p104.z_SET(2.9949513E38F);
		CommunicationChannel.instance.send(p104);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_HIGHRES_IMU.add((src, ph, pack) ->
		{
			assert (pack.temperature_GET() == 1.8094915E38F);
			assert (pack.time_usec_GET() == 2086513698080414733L);
			assert (pack.xmag_GET() == -1.8879398E38F);
			assert (pack.zacc_GET() == 6.0392046E37F);
			assert (pack.ygyro_GET() == -1.8028225E37F);
			assert (pack.pressure_alt_GET() == 2.829304E38F);
			assert (pack.xgyro_GET() == -8.4425773E37F);
			assert (pack.zmag_GET() == 7.415254E37F);
			assert (pack.yacc_GET() == 3.5209457E37F);
			assert (pack.diff_pressure_GET() == -1.0587553E38F);
			assert (pack.abs_pressure_GET() == 2.7718387E38F);
			assert (pack.fields_updated_GET() == (char) 17860);
			assert (pack.zgyro_GET() == -8.371901E37F);
			assert (pack.xacc_GET() == 1.1336202E38F);
			assert (pack.ymag_GET() == 3.0510825E38F);
		});
		GroundControl.HIGHRES_IMU p105 = CommunicationChannel.new_HIGHRES_IMU();
		PH.setPack(p105);
		p105.fields_updated_SET((char) 17860);
		p105.diff_pressure_SET(-1.0587553E38F);
		p105.temperature_SET(1.8094915E38F);
		p105.xmag_SET(-1.8879398E38F);
		p105.pressure_alt_SET(2.829304E38F);
		p105.zacc_SET(6.0392046E37F);
		p105.xgyro_SET(-8.4425773E37F);
		p105.zmag_SET(7.415254E37F);
		p105.yacc_SET(3.5209457E37F);
		p105.ygyro_SET(-1.8028225E37F);
		p105.zgyro_SET(-8.371901E37F);
		p105.xacc_SET(1.1336202E38F);
		p105.abs_pressure_SET(2.7718387E38F);
		p105.ymag_SET(3.0510825E38F);
		p105.time_usec_SET(2086513698080414733L);
		CommunicationChannel.instance.send(p105);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_OPTICAL_FLOW_RAD.add((src, ph, pack) ->
		{
			assert (pack.time_usec_GET() == 5166729700416409740L);
			assert (pack.integrated_zgyro_GET() == -1.9507222E38F);
			assert (pack.sensor_id_GET() == (char) 62);
			assert (pack.temperature_GET() == (short) 25196);
			assert (pack.integrated_xgyro_GET() == -1.9038183E38F);
			assert (pack.time_delta_distance_us_GET() == 3851905L);
			assert (pack.integrated_ygyro_GET() == -1.9133353E37F);
			assert (pack.quality_GET() == (char) 16);
			assert (pack.integrated_x_GET() == -1.1780156E38F);
			assert (pack.distance_GET() == 2.6853519E38F);
			assert (pack.integrated_y_GET() == -1.8971001E37F);
			assert (pack.integration_time_us_GET() == 2648504397L);
		});
		GroundControl.OPTICAL_FLOW_RAD p106 = CommunicationChannel.new_OPTICAL_FLOW_RAD();
		PH.setPack(p106);
		p106.time_delta_distance_us_SET(3851905L);
		p106.temperature_SET((short) 25196);
		p106.distance_SET(2.6853519E38F);
		p106.sensor_id_SET((char) 62);
		p106.integrated_xgyro_SET(-1.9038183E38F);
		p106.time_usec_SET(5166729700416409740L);
		p106.quality_SET((char) 16);
		p106.integrated_y_SET(-1.8971001E37F);
		p106.integration_time_us_SET(2648504397L);
		p106.integrated_zgyro_SET(-1.9507222E38F);
		p106.integrated_x_SET(-1.1780156E38F);
		p106.integrated_ygyro_SET(-1.9133353E37F);
		CommunicationChannel.instance.send(p106);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_HIL_SENSOR.add((src, ph, pack) ->
		{
			assert (pack.zacc_GET() == 7.505482E37F);
			assert (pack.time_usec_GET() == 3295232708137492939L);
			assert (pack.diff_pressure_GET() == 3.3617354E38F);
			assert (pack.xacc_GET() == -1.7846318E38F);
			assert (pack.pressure_alt_GET() == -1.3611187E38F);
			assert (pack.temperature_GET() == 2.1937708E37F);
			assert (pack.yacc_GET() == 6.441317E37F);
			assert (pack.abs_pressure_GET() == 1.8016688E38F);
			assert (pack.xmag_GET() == 2.9130706E38F);
			assert (pack.zmag_GET() == 2.4775758E38F);
			assert (pack.zgyro_GET() == -5.45188E37F);
			assert (pack.ygyro_GET() == -2.5115872E38F);
			assert (pack.xgyro_GET() == 2.6278957E38F);
			assert (pack.fields_updated_GET() == 2035502809L);
			assert (pack.ymag_GET() == 2.9275182E38F);
		});
		GroundControl.HIL_SENSOR p107 = CommunicationChannel.new_HIL_SENSOR();
		PH.setPack(p107);
		p107.ymag_SET(2.9275182E38F);
		p107.pressure_alt_SET(-1.3611187E38F);
		p107.time_usec_SET(3295232708137492939L);
		p107.xmag_SET(2.9130706E38F);
		p107.xacc_SET(-1.7846318E38F);
		p107.temperature_SET(2.1937708E37F);
		p107.abs_pressure_SET(1.8016688E38F);
		p107.zmag_SET(2.4775758E38F);
		p107.diff_pressure_SET(3.3617354E38F);
		p107.xgyro_SET(2.6278957E38F);
		p107.ygyro_SET(-2.5115872E38F);
		p107.zacc_SET(7.505482E37F);
		p107.yacc_SET(6.441317E37F);
		p107.fields_updated_SET(2035502809L);
		p107.zgyro_SET(-5.45188E37F);
		CommunicationChannel.instance.send(p107);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_SIM_STATE.add((src, ph, pack) ->
		{
			assert (pack.q2_GET() == -1.6276361E38F);
			assert (pack.pitch_GET() == -2.4504812E38F);
			assert (pack.q3_GET() == -1.160761E38F);
			assert (pack.zgyro_GET() == 2.3818826E38F);
			assert (pack.std_dev_vert_GET() == 2.0140285E38F);
			assert (pack.ygyro_GET() == 1.2480434E36F);
			assert (pack.vn_GET() == -2.5133746E37F);
			assert (pack.zacc_GET() == 2.6450735E38F);
			assert (pack.yaw_GET() == 1.6252564E38F);
			assert (pack.q4_GET() == -1.2689767E37F);
			assert (pack.vd_GET() == 1.6859672E38F);
			assert (pack.lon_GET() == -3.40135E37F);
			assert (pack.xacc_GET() == 1.471187E38F);
			assert (pack.alt_GET() == -3.2562707E38F);
			assert (pack.xgyro_GET() == 1.5249699E38F);
			assert (pack.q1_GET() == -4.923927E37F);
			assert (pack.ve_GET() == -3.3166492E38F);
			assert (pack.lat_GET() == -1.4441188E38F);
			assert (pack.roll_GET() == -1.7505499E37F);
			assert (pack.yacc_GET() == -3.3654523E38F);
			assert (pack.std_dev_horz_GET() == 2.0368772E38F);
		});
		GroundControl.SIM_STATE p108 = CommunicationChannel.new_SIM_STATE();
		PH.setPack(p108);
		p108.vn_SET(-2.5133746E37F);
		p108.q3_SET(-1.160761E38F);
		p108.zgyro_SET(2.3818826E38F);
		p108.xgyro_SET(1.5249699E38F);
		p108.ygyro_SET(1.2480434E36F);
		p108.q2_SET(-1.6276361E38F);
		p108.ve_SET(-3.3166492E38F);
		p108.q1_SET(-4.923927E37F);
		p108.vd_SET(1.6859672E38F);
		p108.xacc_SET(1.471187E38F);
		p108.q4_SET(-1.2689767E37F);
		p108.roll_SET(-1.7505499E37F);
		p108.yacc_SET(-3.3654523E38F);
		p108.alt_SET(-3.2562707E38F);
		p108.yaw_SET(1.6252564E38F);
		p108.lat_SET(-1.4441188E38F);
		p108.std_dev_vert_SET(2.0140285E38F);
		p108.zacc_SET(2.6450735E38F);
		p108.std_dev_horz_SET(2.0368772E38F);
		p108.lon_SET(-3.40135E37F);
		p108.pitch_SET(-2.4504812E38F);
		CommunicationChannel.instance.send(p108);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_RADIO_STATUS.add((src, ph, pack) ->
		{
			assert (pack.rssi_GET() == (char) 20);
			assert (pack.noise_GET() == (char) 30);
			assert (pack.txbuf_GET() == (char) 129);
			assert (pack.rxerrors_GET() == (char) 62151);
			assert (pack.fixed__GET() == (char) 51817);
			assert (pack.remnoise_GET() == (char) 34);
			assert (pack.remrssi_GET() == (char) 192);
		});
		GroundControl.RADIO_STATUS p109 = CommunicationChannel.new_RADIO_STATUS();
		PH.setPack(p109);
		p109.rssi_SET((char) 20);
		p109.fixed__SET((char) 51817);
		p109.rxerrors_SET((char) 62151);
		p109.txbuf_SET((char) 129);
		p109.noise_SET((char) 30);
		p109.remnoise_SET((char) 34);
		p109.remrssi_SET((char) 192);
		CommunicationChannel.instance.send(p109);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_FILE_TRANSFER_PROTOCOL.add((src, ph, pack) ->
		{
			assert (pack.target_system_GET() == (char) 122);
			assert (pack.target_component_GET() == (char) 187);
			assert (pack.target_network_GET() == (char) 144);
			assert (Arrays.equals(pack.payload_GET(), new char[]{(char) 111, (char) 36, (char) 90, (char) 25, (char) 92, (char) 36, (char) 111, (char) 62, (char) 120, (char) 64, (char) 15, (char) 76, (char) 254, (char) 200, (char) 147, (char) 55, (char) 162, (char) 245, (char) 0, (char) 128, (char) 82, (char) 143, (char) 94, (char) 110, (char) 10, (char) 23, (char) 123, (char) 148, (char) 14, (char) 158, (char) 156, (char) 22, (char) 137, (char) 3, (char) 12, (char) 72, (char) 242, (char) 61, (char) 51, (char) 113, (char) 210, (char) 68, (char) 164, (char) 118, (char) 4, (char) 136, (char) 125, (char) 0, (char) 214, (char) 17, (char) 99, (char) 116, (char) 106, (char) 97, (char) 211, (char) 192, (char) 197, (char) 250, (char) 85, (char) 94, (char) 112, (char) 133, (char) 114, (char) 249, (char) 25, (char) 252, (char) 197, (char) 29, (char) 242, (char) 122, (char) 189, (char) 108, (char) 194, (char) 150, (char) 234, (char) 147, (char) 206, (char) 28, (char) 253, (char) 225, (char) 111, (char) 46, (char) 160, (char) 190, (char) 155, (char) 200, (char) 230, (char) 238, (char) 181, (char) 248, (char) 141, (char) 248, (char) 51, (char) 140, (char) 83, (char) 213, (char) 235, (char) 19, (char) 229, (char) 87, (char) 56, (char) 124, (char) 147, (char) 67, (char) 205, (char) 67, (char) 72, (char) 213, (char) 203, (char) 80, (char) 11, (char) 56, (char) 132, (char) 18, (char) 157, (char) 242, (char) 82, (char) 101, (char) 190, (char) 50, (char) 203, (char) 206, (char) 77, (char) 167, (char) 149, (char) 57, (char) 192, (char) 164, (char) 230, (char) 229, (char) 234, (char) 170, (char) 92, (char) 93, (char) 90, (char) 134, (char) 69, (char) 217, (char) 53, (char) 36, (char) 125, (char) 59, (char) 196, (char) 117, (char) 36, (char) 75, (char) 66, (char) 69, (char) 108, (char) 156, (char) 217, (char) 111, (char) 143, (char) 97, (char) 156, (char) 223, (char) 127, (char) 49, (char) 91, (char) 111, (char) 159, (char) 34, (char) 45, (char) 232, (char) 48, (char) 34, (char) 29, (char) 98, (char) 33, (char) 19, (char) 39, (char) 249, (char) 159, (char) 199, (char) 16, (char) 239, (char) 17, (char) 249, (char) 9, (char) 175, (char) 142, (char) 130, (char) 1, (char) 218, (char) 199, (char) 208, (char) 212, (char) 193, (char) 98, (char) 249, (char) 239, (char) 54, (char) 153, (char) 231, (char) 196, (char) 44, (char) 205, (char) 25, (char) 96, (char) 158, (char) 208, (char) 97, (char) 29, (char) 169, (char) 137, (char) 142, (char) 90, (char) 166, (char) 40, (char) 131, (char) 254, (char) 18, (char) 43, (char) 118, (char) 19, (char) 118, (char) 37, (char) 119, (char) 146, (char) 222, (char) 84, (char) 158, (char) 32, (char) 136, (char) 72, (char) 194, (char) 4, (char) 181, (char) 36, (char) 148, (char) 95, (char) 14, (char) 189, (char) 241, (char) 127, (char) 246, (char) 231, (char) 140, (char) 207, (char) 103, (char) 37, (char) 1, (char) 12, (char) 68, (char) 177, (char) 14, (char) 111, (char) 200, (char) 155, (char) 12, (char) 11}));
		});
		GroundControl.FILE_TRANSFER_PROTOCOL p110 = CommunicationChannel.new_FILE_TRANSFER_PROTOCOL();
		PH.setPack(p110);
		p110.target_network_SET((char) 144);
		p110.target_system_SET((char) 122);
		p110.payload_SET(new char[]{(char) 111, (char) 36, (char) 90, (char) 25, (char) 92, (char) 36, (char) 111, (char) 62, (char) 120, (char) 64, (char) 15, (char) 76, (char) 254, (char) 200, (char) 147, (char) 55, (char) 162, (char) 245, (char) 0, (char) 128, (char) 82, (char) 143, (char) 94, (char) 110, (char) 10, (char) 23, (char) 123, (char) 148, (char) 14, (char) 158, (char) 156, (char) 22, (char) 137, (char) 3, (char) 12, (char) 72, (char) 242, (char) 61, (char) 51, (char) 113, (char) 210, (char) 68, (char) 164, (char) 118, (char) 4, (char) 136, (char) 125, (char) 0, (char) 214, (char) 17, (char) 99, (char) 116, (char) 106, (char) 97, (char) 211, (char) 192, (char) 197, (char) 250, (char) 85, (char) 94, (char) 112, (char) 133, (char) 114, (char) 249, (char) 25, (char) 252, (char) 197, (char) 29, (char) 242, (char) 122, (char) 189, (char) 108, (char) 194, (char) 150, (char) 234, (char) 147, (char) 206, (char) 28, (char) 253, (char) 225, (char) 111, (char) 46, (char) 160, (char) 190, (char) 155, (char) 200, (char) 230, (char) 238, (char) 181, (char) 248, (char) 141, (char) 248, (char) 51, (char) 140, (char) 83, (char) 213, (char) 235, (char) 19, (char) 229, (char) 87, (char) 56, (char) 124, (char) 147, (char) 67, (char) 205, (char) 67, (char) 72, (char) 213, (char) 203, (char) 80, (char) 11, (char) 56, (char) 132, (char) 18, (char) 157, (char) 242, (char) 82, (char) 101, (char) 190, (char) 50, (char) 203, (char) 206, (char) 77, (char) 167, (char) 149, (char) 57, (char) 192, (char) 164, (char) 230, (char) 229, (char) 234, (char) 170, (char) 92, (char) 93, (char) 90, (char) 134, (char) 69, (char) 217, (char) 53, (char) 36, (char) 125, (char) 59, (char) 196, (char) 117, (char) 36, (char) 75, (char) 66, (char) 69, (char) 108, (char) 156, (char) 217, (char) 111, (char) 143, (char) 97, (char) 156, (char) 223, (char) 127, (char) 49, (char) 91, (char) 111, (char) 159, (char) 34, (char) 45, (char) 232, (char) 48, (char) 34, (char) 29, (char) 98, (char) 33, (char) 19, (char) 39, (char) 249, (char) 159, (char) 199, (char) 16, (char) 239, (char) 17, (char) 249, (char) 9, (char) 175, (char) 142, (char) 130, (char) 1, (char) 218, (char) 199, (char) 208, (char) 212, (char) 193, (char) 98, (char) 249, (char) 239, (char) 54, (char) 153, (char) 231, (char) 196, (char) 44, (char) 205, (char) 25, (char) 96, (char) 158, (char) 208, (char) 97, (char) 29, (char) 169, (char) 137, (char) 142, (char) 90, (char) 166, (char) 40, (char) 131, (char) 254, (char) 18, (char) 43, (char) 118, (char) 19, (char) 118, (char) 37, (char) 119, (char) 146, (char) 222, (char) 84, (char) 158, (char) 32, (char) 136, (char) 72, (char) 194, (char) 4, (char) 181, (char) 36, (char) 148, (char) 95, (char) 14, (char) 189, (char) 241, (char) 127, (char) 246, (char) 231, (char) 140, (char) 207, (char) 103, (char) 37, (char) 1, (char) 12, (char) 68, (char) 177, (char) 14, (char) 111, (char) 200, (char) 155, (char) 12, (char) 11}, 0);
		p110.target_component_SET((char) 187);
		CommunicationChannel.instance.send(p110);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_TIMESYNC.add((src, ph, pack) ->
		{
			assert (pack.ts1_GET() == 1713617468201529021L);
			assert (pack.tc1_GET() == 3329095106177548760L);
		});
		GroundControl.TIMESYNC p111 = CommunicationChannel.new_TIMESYNC();
		PH.setPack(p111);
		p111.tc1_SET(3329095106177548760L);
		p111.ts1_SET(1713617468201529021L);
		CommunicationChannel.instance.send(p111);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_CAMERA_TRIGGER.add((src, ph, pack) ->
		{
			assert (pack.seq_GET() == 1385228609L);
			assert (pack.time_usec_GET() == 1185137537989747323L);
		});
		GroundControl.CAMERA_TRIGGER p112 = CommunicationChannel.new_CAMERA_TRIGGER();
		PH.setPack(p112);
		p112.time_usec_SET(1185137537989747323L);
		p112.seq_SET(1385228609L);
		CommunicationChannel.instance.send(p112);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_HIL_GPS.add((src, ph, pack) ->
		{
			assert (pack.cog_GET() == (char) 27468);
			assert (pack.satellites_visible_GET() == (char) 48);
			assert (pack.vd_GET() == (short) -4403);
			assert (pack.fix_type_GET() == (char) 9);
			assert (pack.ve_GET() == (short) 15537);
			assert (pack.lat_GET() == 1148705301);
			assert (pack.vn_GET() == (short) 13878);
			assert (pack.alt_GET() == 839170552);
			assert (pack.epv_GET() == (char) 9827);
			assert (pack.eph_GET() == (char) 1411);
			assert (pack.time_usec_GET() == 5614799139178991043L);
			assert (pack.vel_GET() == (char) 51858);
			assert (pack.lon_GET() == 349384541);
		});
		GroundControl.HIL_GPS p113 = CommunicationChannel.new_HIL_GPS();
		PH.setPack(p113);
		p113.vn_SET((short) 13878);
		p113.eph_SET((char) 1411);
		p113.lon_SET(349384541);
		p113.alt_SET(839170552);
		p113.cog_SET((char) 27468);
		p113.ve_SET((short) 15537);
		p113.vel_SET((char) 51858);
		p113.satellites_visible_SET((char) 48);
		p113.time_usec_SET(5614799139178991043L);
		p113.epv_SET((char) 9827);
		p113.lat_SET(1148705301);
		p113.vd_SET((short) -4403);
		p113.fix_type_SET((char) 9);
		CommunicationChannel.instance.send(p113);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_HIL_OPTICAL_FLOW.add((src, ph, pack) ->
		{
			assert (pack.distance_GET() == 1.3933351E38F);
			assert (pack.integrated_zgyro_GET() == -1.0202611E38F);
			assert (pack.time_delta_distance_us_GET() == 271853220L);
			assert (pack.integrated_ygyro_GET() == -1.7079963E38F);
			assert (pack.integrated_xgyro_GET() == -2.04769E38F);
			assert (pack.temperature_GET() == (short) 17689);
			assert (pack.integrated_y_GET() == 1.3083528E38F);
			assert (pack.quality_GET() == (char) 209);
			assert (pack.integration_time_us_GET() == 993781805L);
			assert (pack.integrated_x_GET() == 1.1567479E38F);
			assert (pack.time_usec_GET() == 8856907928633991465L);
			assert (pack.sensor_id_GET() == (char) 157);
		});
		GroundControl.HIL_OPTICAL_FLOW p114 = CommunicationChannel.new_HIL_OPTICAL_FLOW();
		PH.setPack(p114);
		p114.integrated_xgyro_SET(-2.04769E38F);
		p114.time_delta_distance_us_SET(271853220L);
		p114.integration_time_us_SET(993781805L);
		p114.integrated_x_SET(1.1567479E38F);
		p114.temperature_SET((short) 17689);
		p114.sensor_id_SET((char) 157);
		p114.distance_SET(1.3933351E38F);
		p114.time_usec_SET(8856907928633991465L);
		p114.integrated_zgyro_SET(-1.0202611E38F);
		p114.integrated_ygyro_SET(-1.7079963E38F);
		p114.quality_SET((char) 209);
		p114.integrated_y_SET(1.3083528E38F);
		CommunicationChannel.instance.send(p114);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_HIL_STATE_QUATERNION.add((src, ph, pack) ->
		{
			assert (pack.rollspeed_GET() == 9.3917685E36F);
			assert (pack.vy_GET() == (short) -30817);
			assert (Arrays.equals(pack.attitude_quaternion_GET(), new float[]{-3.211068E38F, 3.5769937E36F, -1.2050871E35F, -2.7555298E38F}));
			assert (pack.vz_GET() == (short) 13028);
			assert (pack.alt_GET() == 1375334309);
			assert (pack.pitchspeed_GET() == -9.057744E37F);
			assert (pack.zacc_GET() == (short) -31050);
			assert (pack.lon_GET() == 2076172783);
			assert (pack.yacc_GET() == (short) -29559);
			assert (pack.lat_GET() == 636246121);
			assert (pack.yawspeed_GET() == 1.5451968E36F);
			assert (pack.xacc_GET() == (short) 7424);
			assert (pack.vx_GET() == (short) 17323);
			assert (pack.true_airspeed_GET() == (char) 37079);
			assert (pack.time_usec_GET() == 8963337943627048686L);
			assert (pack.ind_airspeed_GET() == (char) 33636);
		});
		GroundControl.HIL_STATE_QUATERNION p115 = CommunicationChannel.new_HIL_STATE_QUATERNION();
		PH.setPack(p115);
		p115.alt_SET(1375334309);
		p115.pitchspeed_SET(-9.057744E37F);
		p115.vy_SET((short) -30817);
		p115.xacc_SET((short) 7424);
		p115.true_airspeed_SET((char) 37079);
		p115.yawspeed_SET(1.5451968E36F);
		p115.vz_SET((short) 13028);
		p115.vx_SET((short) 17323);
		p115.attitude_quaternion_SET(new float[]{-3.211068E38F, 3.5769937E36F, -1.2050871E35F, -2.7555298E38F}, 0);
		p115.time_usec_SET(8963337943627048686L);
		p115.yacc_SET((short) -29559);
		p115.zacc_SET((short) -31050);
		p115.lon_SET(2076172783);
		p115.lat_SET(636246121);
		p115.rollspeed_SET(9.3917685E36F);
		p115.ind_airspeed_SET((char) 33636);
		CommunicationChannel.instance.send(p115);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_SCALED_IMU2.add((src, ph, pack) ->
		{
			assert (pack.ymag_GET() == (short) -9732);
			assert (pack.zmag_GET() == (short) 26052);
			assert (pack.time_boot_ms_GET() == 3472751134L);
			assert (pack.zacc_GET() == (short) 29799);
			assert (pack.xgyro_GET() == (short) -4050);
			assert (pack.yacc_GET() == (short) 5735);
			assert (pack.ygyro_GET() == (short) -5920);
			assert (pack.zgyro_GET() == (short) -32206);
			assert (pack.xacc_GET() == (short) 21209);
			assert (pack.xmag_GET() == (short) -28504);
		});
		GroundControl.SCALED_IMU2 p116 = CommunicationChannel.new_SCALED_IMU2();
		PH.setPack(p116);
		p116.xgyro_SET((short) -4050);
		p116.time_boot_ms_SET(3472751134L);
		p116.xmag_SET((short) -28504);
		p116.zmag_SET((short) 26052);
		p116.yacc_SET((short) 5735);
		p116.xacc_SET((short) 21209);
		p116.zgyro_SET((short) -32206);
		p116.ymag_SET((short) -9732);
		p116.zacc_SET((short) 29799);
		p116.ygyro_SET((short) -5920);
		CommunicationChannel.instance.send(p116);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_LOG_REQUEST_LIST.add((src, ph, pack) ->
		{
			assert (pack.end_GET() == (char) 27829);
			assert (pack.target_system_GET() == (char) 95);
			assert (pack.start_GET() == (char) 56566);
			assert (pack.target_component_GET() == (char) 83);
		});
		GroundControl.LOG_REQUEST_LIST p117 = CommunicationChannel.new_LOG_REQUEST_LIST();
		PH.setPack(p117);
		p117.start_SET((char) 56566);
		p117.end_SET((char) 27829);
		p117.target_system_SET((char) 95);
		p117.target_component_SET((char) 83);
		CommunicationChannel.instance.send(p117);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_LOG_ENTRY.add((src, ph, pack) ->
		{
			assert (pack.last_log_num_GET() == (char) 57906);
			assert (pack.size_GET() == 2463327077L);
			assert (pack.id_GET() == (char) 62909);
			assert (pack.time_utc_GET() == 304204271L);
			assert (pack.num_logs_GET() == (char) 25873);
		});
		GroundControl.LOG_ENTRY p118 = CommunicationChannel.new_LOG_ENTRY();
		PH.setPack(p118);
		p118.num_logs_SET((char) 25873);
		p118.id_SET((char) 62909);
		p118.time_utc_SET(304204271L);
		p118.size_SET(2463327077L);
		p118.last_log_num_SET((char) 57906);
		CommunicationChannel.instance.send(p118);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_LOG_REQUEST_DATA.add((src, ph, pack) ->
		{
			assert (pack.target_component_GET() == (char) 239);
			assert (pack.id_GET() == (char) 8547);
			assert (pack.ofs_GET() == 4122173189L);
			assert (pack.count_GET() == 3687242879L);
			assert (pack.target_system_GET() == (char) 77);
		});
		GroundControl.LOG_REQUEST_DATA p119 = CommunicationChannel.new_LOG_REQUEST_DATA();
		PH.setPack(p119);
		p119.ofs_SET(4122173189L);
		p119.id_SET((char) 8547);
		p119.target_component_SET((char) 239);
		p119.count_SET(3687242879L);
		p119.target_system_SET((char) 77);
		CommunicationChannel.instance.send(p119);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_LOG_DATA.add((src, ph, pack) ->
		{
			assert (Arrays.equals(pack.data__GET(), new char[]{(char) 157, (char) 17, (char) 40, (char) 178, (char) 173, (char) 69, (char) 155, (char) 41, (char) 118, (char) 110, (char) 136, (char) 114, (char) 189, (char) 221, (char) 243, (char) 21, (char) 105, (char) 235, (char) 196, (char) 16, (char) 211, (char) 205, (char) 145, (char) 248, (char) 7, (char) 58, (char) 243, (char) 210, (char) 73, (char) 83, (char) 81, (char) 251, (char) 115, (char) 33, (char) 173, (char) 62, (char) 92, (char) 145, (char) 57, (char) 235, (char) 151, (char) 85, (char) 129, (char) 125, (char) 165, (char) 129, (char) 165, (char) 202, (char) 75, (char) 79, (char) 222, (char) 28, (char) 177, (char) 165, (char) 76, (char) 4, (char) 159, (char) 174, (char) 121, (char) 230, (char) 210, (char) 255, (char) 223, (char) 216, (char) 212, (char) 124, (char) 9, (char) 216, (char) 22, (char) 1, (char) 200, (char) 10, (char) 170, (char) 93, (char) 200, (char) 90, (char) 31, (char) 8, (char) 12, (char) 149, (char) 55, (char) 196, (char) 105, (char) 104, (char) 141, (char) 104, (char) 53, (char) 165, (char) 216, (char) 87}));
			assert (pack.id_GET() == (char) 5719);
			assert (pack.count_GET() == (char) 72);
			assert (pack.ofs_GET() == 2397014782L);
		});
		GroundControl.LOG_DATA p120 = CommunicationChannel.new_LOG_DATA();
		PH.setPack(p120);
		p120.count_SET((char) 72);
		p120.id_SET((char) 5719);
		p120.data__SET(new char[]{(char) 157, (char) 17, (char) 40, (char) 178, (char) 173, (char) 69, (char) 155, (char) 41, (char) 118, (char) 110, (char) 136, (char) 114, (char) 189, (char) 221, (char) 243, (char) 21, (char) 105, (char) 235, (char) 196, (char) 16, (char) 211, (char) 205, (char) 145, (char) 248, (char) 7, (char) 58, (char) 243, (char) 210, (char) 73, (char) 83, (char) 81, (char) 251, (char) 115, (char) 33, (char) 173, (char) 62, (char) 92, (char) 145, (char) 57, (char) 235, (char) 151, (char) 85, (char) 129, (char) 125, (char) 165, (char) 129, (char) 165, (char) 202, (char) 75, (char) 79, (char) 222, (char) 28, (char) 177, (char) 165, (char) 76, (char) 4, (char) 159, (char) 174, (char) 121, (char) 230, (char) 210, (char) 255, (char) 223, (char) 216, (char) 212, (char) 124, (char) 9, (char) 216, (char) 22, (char) 1, (char) 200, (char) 10, (char) 170, (char) 93, (char) 200, (char) 90, (char) 31, (char) 8, (char) 12, (char) 149, (char) 55, (char) 196, (char) 105, (char) 104, (char) 141, (char) 104, (char) 53, (char) 165, (char) 216, (char) 87}, 0);
		p120.ofs_SET(2397014782L);
		CommunicationChannel.instance.send(p120);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_LOG_ERASE.add((src, ph, pack) ->
		{
			assert (pack.target_system_GET() == (char) 154);
			assert (pack.target_component_GET() == (char) 37);
		});
		GroundControl.LOG_ERASE p121 = CommunicationChannel.new_LOG_ERASE();
		PH.setPack(p121);
		p121.target_component_SET((char) 37);
		p121.target_system_SET((char) 154);
		CommunicationChannel.instance.send(p121);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_LOG_REQUEST_END.add((src, ph, pack) ->
		{
			assert (pack.target_system_GET() == (char) 89);
			assert (pack.target_component_GET() == (char) 67);
		});
		GroundControl.LOG_REQUEST_END p122 = CommunicationChannel.new_LOG_REQUEST_END();
		PH.setPack(p122);
		p122.target_component_SET((char) 67);
		p122.target_system_SET((char) 89);
		CommunicationChannel.instance.send(p122);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_GPS_INJECT_DATA.add((src, ph, pack) ->
		{
			assert (pack.target_component_GET() == (char) 164);
			assert (Arrays.equals(pack.data__GET(), new char[]{(char) 61, (char) 242, (char) 0, (char) 9, (char) 63, (char) 175, (char) 44, (char) 223, (char) 178, (char) 178, (char) 244, (char) 95, (char) 211, (char) 1, (char) 230, (char) 231, (char) 161, (char) 214, (char) 43, (char) 232, (char) 207, (char) 37, (char) 99, (char) 157, (char) 134, (char) 55, (char) 160, (char) 187, (char) 118, (char) 23, (char) 134, (char) 158, (char) 79, (char) 123, (char) 82, (char) 28, (char) 99, (char) 101, (char) 116, (char) 247, (char) 197, (char) 238, (char) 180, (char) 222, (char) 248, (char) 169, (char) 44, (char) 143, (char) 224, (char) 108, (char) 122, (char) 191, (char) 225, (char) 148, (char) 134, (char) 248, (char) 104, (char) 96, (char) 28, (char) 202, (char) 58, (char) 190, (char) 81, (char) 217, (char) 179, (char) 206, (char) 100, (char) 66, (char) 178, (char) 118, (char) 106, (char) 117, (char) 193, (char) 100, (char) 43, (char) 136, (char) 226, (char) 169, (char) 209, (char) 229, (char) 142, (char) 155, (char) 181, (char) 200, (char) 98, (char) 134, (char) 164, (char) 166, (char) 83, (char) 28, (char) 200, (char) 83, (char) 128, (char) 30, (char) 55, (char) 204, (char) 199, (char) 87, (char) 7, (char) 64, (char) 171, (char) 78, (char) 240, (char) 218, (char) 120, (char) 178, (char) 159, (char) 123, (char) 89, (char) 26}));
			assert (pack.target_system_GET() == (char) 139);
			assert (pack.len_GET() == (char) 218);
		});
		GroundControl.GPS_INJECT_DATA p123 = CommunicationChannel.new_GPS_INJECT_DATA();
		PH.setPack(p123);
		p123.target_component_SET((char) 164);
		p123.data__SET(new char[]{(char) 61, (char) 242, (char) 0, (char) 9, (char) 63, (char) 175, (char) 44, (char) 223, (char) 178, (char) 178, (char) 244, (char) 95, (char) 211, (char) 1, (char) 230, (char) 231, (char) 161, (char) 214, (char) 43, (char) 232, (char) 207, (char) 37, (char) 99, (char) 157, (char) 134, (char) 55, (char) 160, (char) 187, (char) 118, (char) 23, (char) 134, (char) 158, (char) 79, (char) 123, (char) 82, (char) 28, (char) 99, (char) 101, (char) 116, (char) 247, (char) 197, (char) 238, (char) 180, (char) 222, (char) 248, (char) 169, (char) 44, (char) 143, (char) 224, (char) 108, (char) 122, (char) 191, (char) 225, (char) 148, (char) 134, (char) 248, (char) 104, (char) 96, (char) 28, (char) 202, (char) 58, (char) 190, (char) 81, (char) 217, (char) 179, (char) 206, (char) 100, (char) 66, (char) 178, (char) 118, (char) 106, (char) 117, (char) 193, (char) 100, (char) 43, (char) 136, (char) 226, (char) 169, (char) 209, (char) 229, (char) 142, (char) 155, (char) 181, (char) 200, (char) 98, (char) 134, (char) 164, (char) 166, (char) 83, (char) 28, (char) 200, (char) 83, (char) 128, (char) 30, (char) 55, (char) 204, (char) 199, (char) 87, (char) 7, (char) 64, (char) 171, (char) 78, (char) 240, (char) 218, (char) 120, (char) 178, (char) 159, (char) 123, (char) 89, (char) 26}, 0);
		p123.target_system_SET((char) 139);
		p123.len_SET((char) 218);
		CommunicationChannel.instance.send(p123);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_GPS2_RAW.add((src, ph, pack) ->
		{
			assert (pack.satellites_visible_GET() == (char) 55);
			assert (pack.lat_GET() == 1196660345);
			assert (pack.time_usec_GET() == 3815463908391695083L);
			assert (pack.lon_GET() == -1956044547);
			assert (pack.dgps_age_GET() == 2070972809L);
			assert (pack.dgps_numch_GET() == (char) 9);
			assert (pack.eph_GET() == (char) 24241);
			assert (pack.fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_NO_FIX);
			assert (pack.cog_GET() == (char) 32184);
			assert (pack.alt_GET() == -582886707);
			assert (pack.epv_GET() == (char) 8856);
			assert (pack.vel_GET() == (char) 38954);
		});
		GroundControl.GPS2_RAW p124 = CommunicationChannel.new_GPS2_RAW();
		PH.setPack(p124);
		p124.lon_SET(-1956044547);
		p124.eph_SET((char) 24241);
		p124.alt_SET(-582886707);
		p124.lat_SET(1196660345);
		p124.fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_NO_FIX);
		p124.cog_SET((char) 32184);
		p124.satellites_visible_SET((char) 55);
		p124.time_usec_SET(3815463908391695083L);
		p124.dgps_age_SET(2070972809L);
		p124.epv_SET((char) 8856);
		p124.dgps_numch_SET((char) 9);
		p124.vel_SET((char) 38954);
		CommunicationChannel.instance.send(p124);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_POWER_STATUS.add((src, ph, pack) ->
		{
			assert (pack.flags_GET() == MAV_POWER_STATUS.MAV_POWER_STATUS_USB_CONNECTED);
			assert (pack.Vservo_GET() == (char) 29623);
			assert (pack.Vcc_GET() == (char) 44563);
		});
		GroundControl.POWER_STATUS p125 = CommunicationChannel.new_POWER_STATUS();
		PH.setPack(p125);
		p125.Vservo_SET((char) 29623);
		p125.flags_SET(MAV_POWER_STATUS.MAV_POWER_STATUS_USB_CONNECTED);
		p125.Vcc_SET((char) 44563);
		CommunicationChannel.instance.send(p125);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_SERIAL_CONTROL.add((src, ph, pack) ->
		{
			assert (Arrays.equals(pack.data__GET(), new char[]{(char) 173, (char) 183, (char) 240, (char) 99, (char) 148, (char) 51, (char) 128, (char) 115, (char) 222, (char) 219, (char) 220, (char) 115, (char) 118, (char) 62, (char) 176, (char) 190, (char) 9, (char) 137, (char) 55, (char) 10, (char) 25, (char) 1, (char) 64, (char) 214, (char) 249, (char) 191, (char) 204, (char) 39, (char) 178, (char) 108, (char) 132, (char) 225, (char) 2, (char) 27, (char) 211, (char) 228, (char) 138, (char) 48, (char) 5, (char) 117, (char) 13, (char) 189, (char) 221, (char) 112, (char) 16, (char) 184, (char) 31, (char) 192, (char) 230, (char) 118, (char) 6, (char) 108, (char) 236, (char) 145, (char) 108, (char) 242, (char) 182, (char) 114, (char) 226, (char) 119, (char) 3, (char) 27, (char) 203, (char) 104, (char) 34, (char) 251, (char) 80, (char) 195, (char) 175, (char) 117}));
			assert (pack.count_GET() == (char) 195);
			assert (pack.timeout_GET() == (char) 15496);
			assert (pack.flags_GET() == SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_BLOCKING);
			assert (pack.device_GET() == SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_SHELL);
			assert (pack.baudrate_GET() == 4174080554L);
		});
		GroundControl.SERIAL_CONTROL p126 = CommunicationChannel.new_SERIAL_CONTROL();
		PH.setPack(p126);
		p126.flags_SET(SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_BLOCKING);
		p126.timeout_SET((char) 15496);
		p126.device_SET(SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_SHELL);
		p126.data__SET(new char[]{(char) 173, (char) 183, (char) 240, (char) 99, (char) 148, (char) 51, (char) 128, (char) 115, (char) 222, (char) 219, (char) 220, (char) 115, (char) 118, (char) 62, (char) 176, (char) 190, (char) 9, (char) 137, (char) 55, (char) 10, (char) 25, (char) 1, (char) 64, (char) 214, (char) 249, (char) 191, (char) 204, (char) 39, (char) 178, (char) 108, (char) 132, (char) 225, (char) 2, (char) 27, (char) 211, (char) 228, (char) 138, (char) 48, (char) 5, (char) 117, (char) 13, (char) 189, (char) 221, (char) 112, (char) 16, (char) 184, (char) 31, (char) 192, (char) 230, (char) 118, (char) 6, (char) 108, (char) 236, (char) 145, (char) 108, (char) 242, (char) 182, (char) 114, (char) 226, (char) 119, (char) 3, (char) 27, (char) 203, (char) 104, (char) 34, (char) 251, (char) 80, (char) 195, (char) 175, (char) 117}, 0);
		p126.baudrate_SET(4174080554L);
		p126.count_SET((char) 195);
		CommunicationChannel.instance.send(p126);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_GPS_RTK.add((src, ph, pack) ->
		{
			assert (pack.rtk_receiver_id_GET() == (char) 242);
			assert (pack.iar_num_hypotheses_GET() == -1294457689);
			assert (pack.time_last_baseline_ms_GET() == 2181002245L);
			assert (pack.accuracy_GET() == 3804380215L);
			assert (pack.baseline_b_mm_GET() == 1456121065);
			assert (pack.baseline_a_mm_GET() == -221998903);
			assert (pack.tow_GET() == 1953534400L);
			assert (pack.wn_GET() == (char) 61178);
			assert (pack.baseline_c_mm_GET() == 287480254);
			assert (pack.nsats_GET() == (char) 51);
			assert (pack.baseline_coords_type_GET() == (char) 113);
			assert (pack.rtk_rate_GET() == (char) 193);
			assert (pack.rtk_health_GET() == (char) 140);
		});
		GroundControl.GPS_RTK p127 = CommunicationChannel.new_GPS_RTK();
		PH.setPack(p127);
		p127.rtk_rate_SET((char) 193);
		p127.rtk_receiver_id_SET((char) 242);
		p127.nsats_SET((char) 51);
		p127.baseline_b_mm_SET(1456121065);
		p127.rtk_health_SET((char) 140);
		p127.baseline_a_mm_SET(-221998903);
		p127.wn_SET((char) 61178);
		p127.iar_num_hypotheses_SET(-1294457689);
		p127.time_last_baseline_ms_SET(2181002245L);
		p127.baseline_coords_type_SET((char) 113);
		p127.accuracy_SET(3804380215L);
		p127.baseline_c_mm_SET(287480254);
		p127.tow_SET(1953534400L);
		CommunicationChannel.instance.send(p127);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_GPS2_RTK.add((src, ph, pack) ->
		{
			assert (pack.rtk_receiver_id_GET() == (char) 224);
			assert (pack.baseline_b_mm_GET() == 231424426);
			assert (pack.wn_GET() == (char) 54936);
			assert (pack.rtk_rate_GET() == (char) 189);
			assert (pack.tow_GET() == 369252195L);
			assert (pack.nsats_GET() == (char) 186);
			assert (pack.iar_num_hypotheses_GET() == -1203331375);
			assert (pack.rtk_health_GET() == (char) 117);
			assert (pack.accuracy_GET() == 2848428060L);
			assert (pack.time_last_baseline_ms_GET() == 49478047L);
			assert (pack.baseline_a_mm_GET() == 925314580);
			assert (pack.baseline_coords_type_GET() == (char) 131);
			assert (pack.baseline_c_mm_GET() == 1053031705);
		});
		GroundControl.GPS2_RTK p128 = CommunicationChannel.new_GPS2_RTK();
		PH.setPack(p128);
		p128.tow_SET(369252195L);
		p128.rtk_rate_SET((char) 189);
		p128.nsats_SET((char) 186);
		p128.baseline_b_mm_SET(231424426);
		p128.baseline_a_mm_SET(925314580);
		p128.rtk_receiver_id_SET((char) 224);
		p128.time_last_baseline_ms_SET(49478047L);
		p128.rtk_health_SET((char) 117);
		p128.baseline_c_mm_SET(1053031705);
		p128.accuracy_SET(2848428060L);
		p128.baseline_coords_type_SET((char) 131);
		p128.iar_num_hypotheses_SET(-1203331375);
		p128.wn_SET((char) 54936);
		CommunicationChannel.instance.send(p128);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_SCALED_IMU3.add((src, ph, pack) ->
		{
			assert (pack.zmag_GET() == (short) 16749);
			assert (pack.xmag_GET() == (short) 30387);
			assert (pack.zacc_GET() == (short) -29495);
			assert (pack.time_boot_ms_GET() == 3962715855L);
			assert (pack.ymag_GET() == (short) 7869);
			assert (pack.yacc_GET() == (short) -14796);
			assert (pack.ygyro_GET() == (short) -16132);
			assert (pack.zgyro_GET() == (short) 15790);
			assert (pack.xgyro_GET() == (short) -28524);
			assert (pack.xacc_GET() == (short) -27716);
		});
		GroundControl.SCALED_IMU3 p129 = CommunicationChannel.new_SCALED_IMU3();
		PH.setPack(p129);
		p129.zacc_SET((short) -29495);
		p129.xgyro_SET((short) -28524);
		p129.time_boot_ms_SET(3962715855L);
		p129.zmag_SET((short) 16749);
		p129.yacc_SET((short) -14796);
		p129.ymag_SET((short) 7869);
		p129.xmag_SET((short) 30387);
		p129.zgyro_SET((short) 15790);
		p129.xacc_SET((short) -27716);
		p129.ygyro_SET((short) -16132);
		CommunicationChannel.instance.send(p129);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_DATA_TRANSMISSION_HANDSHAKE.add((src, ph, pack) ->
		{
			assert (pack.height_GET() == (char) 13427);
			assert (pack.type_GET() == (char) 127);
			assert (pack.payload_GET() == (char) 60);
			assert (pack.packets_GET() == (char) 25912);
			assert (pack.width_GET() == (char) 59486);
			assert (pack.jpg_quality_GET() == (char) 244);
			assert (pack.size_GET() == 3915560555L);
		});
		GroundControl.DATA_TRANSMISSION_HANDSHAKE p130 = CommunicationChannel.new_DATA_TRANSMISSION_HANDSHAKE();
		PH.setPack(p130);
		p130.packets_SET((char) 25912);
		p130.width_SET((char) 59486);
		p130.type_SET((char) 127);
		p130.payload_SET((char) 60);
		p130.size_SET(3915560555L);
		p130.height_SET((char) 13427);
		p130.jpg_quality_SET((char) 244);
		CommunicationChannel.instance.send(p130);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_ENCAPSULATED_DATA.add((src, ph, pack) ->
		{
			assert (Arrays.equals(pack.data__GET(), new char[]{(char) 75, (char) 122, (char) 135, (char) 156, (char) 63, (char) 210, (char) 95, (char) 71, (char) 142, (char) 180, (char) 131, (char) 244, (char) 124, (char) 255, (char) 134, (char) 251, (char) 183, (char) 148, (char) 40, (char) 179, (char) 49, (char) 124, (char) 222, (char) 162, (char) 73, (char) 35, (char) 141, (char) 215, (char) 185, (char) 39, (char) 88, (char) 107, (char) 112, (char) 109, (char) 211, (char) 62, (char) 85, (char) 51, (char) 90, (char) 161, (char) 20, (char) 16, (char) 91, (char) 168, (char) 73, (char) 16, (char) 130, (char) 207, (char) 59, (char) 223, (char) 18, (char) 19, (char) 20, (char) 61, (char) 180, (char) 222, (char) 155, (char) 148, (char) 166, (char) 188, (char) 122, (char) 48, (char) 72, (char) 192, (char) 91, (char) 73, (char) 119, (char) 234, (char) 128, (char) 22, (char) 72, (char) 123, (char) 72, (char) 48, (char) 179, (char) 5, (char) 166, (char) 251, (char) 39, (char) 6, (char) 242, (char) 54, (char) 14, (char) 70, (char) 164, (char) 27, (char) 238, (char) 99, (char) 17, (char) 124, (char) 99, (char) 213, (char) 143, (char) 156, (char) 23, (char) 95, (char) 210, (char) 248, (char) 172, (char) 162, (char) 163, (char) 113, (char) 103, (char) 234, (char) 102, (char) 42, (char) 168, (char) 102, (char) 30, (char) 97, (char) 9, (char) 168, (char) 63, (char) 102, (char) 165, (char) 65, (char) 7, (char) 241, (char) 237, (char) 208, (char) 18, (char) 183, (char) 16, (char) 239, (char) 24, (char) 171, (char) 64, (char) 60, (char) 16, (char) 185, (char) 173, (char) 164, (char) 26, (char) 138, (char) 215, (char) 149, (char) 133, (char) 136, (char) 216, (char) 29, (char) 20, (char) 193, (char) 103, (char) 85, (char) 246, (char) 233, (char) 102, (char) 59, (char) 92, (char) 134, (char) 57, (char) 249, (char) 118, (char) 114, (char) 220, (char) 10, (char) 179, (char) 66, (char) 34, (char) 140, (char) 92, (char) 132, (char) 173, (char) 140, (char) 130, (char) 135, (char) 134, (char) 11, (char) 252, (char) 53, (char) 207, (char) 177, (char) 144, (char) 240, (char) 215, (char) 108, (char) 185, (char) 117, (char) 245, (char) 204, (char) 74, (char) 167, (char) 252, (char) 211, (char) 85, (char) 95, (char) 30, (char) 29, (char) 21, (char) 204, (char) 164, (char) 93, (char) 110, (char) 248, (char) 0, (char) 31, (char) 66, (char) 161, (char) 125, (char) 29, (char) 52, (char) 31, (char) 100, (char) 108, (char) 104, (char) 150, (char) 237, (char) 175, (char) 53, (char) 42, (char) 106, (char) 99, (char) 82, (char) 41, (char) 150, (char) 247, (char) 66, (char) 9, (char) 143, (char) 253, (char) 53, (char) 210, (char) 202, (char) 0, (char) 210, (char) 59, (char) 234, (char) 229, (char) 249, (char) 107, (char) 40, (char) 7, (char) 201, (char) 90, (char) 234, (char) 109, (char) 196, (char) 44, (char) 199, (char) 180, (char) 177, (char) 239, (char) 19, (char) 151, (char) 136, (char) 177, (char) 245, (char) 212, (char) 206, (char) 201, (char) 126, (char) 174, (char) 113}));
			assert (pack.seqnr_GET() == (char) 53948);
		});
		GroundControl.ENCAPSULATED_DATA p131 = CommunicationChannel.new_ENCAPSULATED_DATA();
		PH.setPack(p131);
		p131.data__SET(new char[]{(char) 75, (char) 122, (char) 135, (char) 156, (char) 63, (char) 210, (char) 95, (char) 71, (char) 142, (char) 180, (char) 131, (char) 244, (char) 124, (char) 255, (char) 134, (char) 251, (char) 183, (char) 148, (char) 40, (char) 179, (char) 49, (char) 124, (char) 222, (char) 162, (char) 73, (char) 35, (char) 141, (char) 215, (char) 185, (char) 39, (char) 88, (char) 107, (char) 112, (char) 109, (char) 211, (char) 62, (char) 85, (char) 51, (char) 90, (char) 161, (char) 20, (char) 16, (char) 91, (char) 168, (char) 73, (char) 16, (char) 130, (char) 207, (char) 59, (char) 223, (char) 18, (char) 19, (char) 20, (char) 61, (char) 180, (char) 222, (char) 155, (char) 148, (char) 166, (char) 188, (char) 122, (char) 48, (char) 72, (char) 192, (char) 91, (char) 73, (char) 119, (char) 234, (char) 128, (char) 22, (char) 72, (char) 123, (char) 72, (char) 48, (char) 179, (char) 5, (char) 166, (char) 251, (char) 39, (char) 6, (char) 242, (char) 54, (char) 14, (char) 70, (char) 164, (char) 27, (char) 238, (char) 99, (char) 17, (char) 124, (char) 99, (char) 213, (char) 143, (char) 156, (char) 23, (char) 95, (char) 210, (char) 248, (char) 172, (char) 162, (char) 163, (char) 113, (char) 103, (char) 234, (char) 102, (char) 42, (char) 168, (char) 102, (char) 30, (char) 97, (char) 9, (char) 168, (char) 63, (char) 102, (char) 165, (char) 65, (char) 7, (char) 241, (char) 237, (char) 208, (char) 18, (char) 183, (char) 16, (char) 239, (char) 24, (char) 171, (char) 64, (char) 60, (char) 16, (char) 185, (char) 173, (char) 164, (char) 26, (char) 138, (char) 215, (char) 149, (char) 133, (char) 136, (char) 216, (char) 29, (char) 20, (char) 193, (char) 103, (char) 85, (char) 246, (char) 233, (char) 102, (char) 59, (char) 92, (char) 134, (char) 57, (char) 249, (char) 118, (char) 114, (char) 220, (char) 10, (char) 179, (char) 66, (char) 34, (char) 140, (char) 92, (char) 132, (char) 173, (char) 140, (char) 130, (char) 135, (char) 134, (char) 11, (char) 252, (char) 53, (char) 207, (char) 177, (char) 144, (char) 240, (char) 215, (char) 108, (char) 185, (char) 117, (char) 245, (char) 204, (char) 74, (char) 167, (char) 252, (char) 211, (char) 85, (char) 95, (char) 30, (char) 29, (char) 21, (char) 204, (char) 164, (char) 93, (char) 110, (char) 248, (char) 0, (char) 31, (char) 66, (char) 161, (char) 125, (char) 29, (char) 52, (char) 31, (char) 100, (char) 108, (char) 104, (char) 150, (char) 237, (char) 175, (char) 53, (char) 42, (char) 106, (char) 99, (char) 82, (char) 41, (char) 150, (char) 247, (char) 66, (char) 9, (char) 143, (char) 253, (char) 53, (char) 210, (char) 202, (char) 0, (char) 210, (char) 59, (char) 234, (char) 229, (char) 249, (char) 107, (char) 40, (char) 7, (char) 201, (char) 90, (char) 234, (char) 109, (char) 196, (char) 44, (char) 199, (char) 180, (char) 177, (char) 239, (char) 19, (char) 151, (char) 136, (char) 177, (char) 245, (char) 212, (char) 206, (char) 201, (char) 126, (char) 174, (char) 113}, 0);
		p131.seqnr_SET((char) 53948);
		CommunicationChannel.instance.send(p131);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_DISTANCE_SENSOR.add((src, ph, pack) ->
		{
			assert (pack.orientation_GET() == MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_270_PITCH_90);
			assert (pack.type_GET() == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_INFRARED);
			assert (pack.id_GET() == (char) 173);
			assert (pack.time_boot_ms_GET() == 2705490014L);
			assert (pack.max_distance_GET() == (char) 7324);
			assert (pack.min_distance_GET() == (char) 29755);
			assert (pack.covariance_GET() == (char) 187);
			assert (pack.current_distance_GET() == (char) 11528);
		});
		GroundControl.DISTANCE_SENSOR p132 = CommunicationChannel.new_DISTANCE_SENSOR();
		PH.setPack(p132);
		p132.covariance_SET((char) 187);
		p132.id_SET((char) 173);
		p132.max_distance_SET((char) 7324);
		p132.time_boot_ms_SET(2705490014L);
		p132.min_distance_SET((char) 29755);
		p132.type_SET(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_INFRARED);
		p132.orientation_SET(MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_270_PITCH_90);
		p132.current_distance_SET((char) 11528);
		CommunicationChannel.instance.send(p132);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_TERRAIN_REQUEST.add((src, ph, pack) ->
		{
			assert (pack.grid_spacing_GET() == (char) 11252);
			assert (pack.mask_GET() == 2197638274430280491L);
			assert (pack.lat_GET() == 1818702505);
			assert (pack.lon_GET() == -25509397);
		});
		GroundControl.TERRAIN_REQUEST p133 = CommunicationChannel.new_TERRAIN_REQUEST();
		PH.setPack(p133);
		p133.lon_SET(-25509397);
		p133.mask_SET(2197638274430280491L);
		p133.lat_SET(1818702505);
		p133.grid_spacing_SET((char) 11252);
		CommunicationChannel.instance.send(p133);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_TERRAIN_DATA.add((src, ph, pack) ->
		{
			assert (pack.grid_spacing_GET() == (char) 25722);
			assert (pack.lat_GET() == 238374459);
			assert (pack.gridbit_GET() == (char) 192);
			assert (pack.lon_GET() == 1122245403);
			assert (Arrays.equals(pack.data__GET(), new short[]{(short) -467, (short) -17094, (short) 4878, (short) -10156, (short) 9994, (short) 17974, (short) 10261, (short) -21438, (short) -4180, (short) -12108, (short) 11869, (short) 28708, (short) -18247, (short) 27797, (short) 1570, (short) -4485}));
		});
		GroundControl.TERRAIN_DATA p134 = CommunicationChannel.new_TERRAIN_DATA();
		PH.setPack(p134);
		p134.grid_spacing_SET((char) 25722);
		p134.gridbit_SET((char) 192);
		p134.lat_SET(238374459);
		p134.lon_SET(1122245403);
		p134.data__SET(new short[]{(short) -467, (short) -17094, (short) 4878, (short) -10156, (short) 9994, (short) 17974, (short) 10261, (short) -21438, (short) -4180, (short) -12108, (short) 11869, (short) 28708, (short) -18247, (short) 27797, (short) 1570, (short) -4485}, 0);
		CommunicationChannel.instance.send(p134);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_TERRAIN_CHECK.add((src, ph, pack) ->
		{
			assert (pack.lon_GET() == 143223524);
			assert (pack.lat_GET() == 941611560);
		});
		GroundControl.TERRAIN_CHECK p135 = CommunicationChannel.new_TERRAIN_CHECK();
		PH.setPack(p135);
		p135.lon_SET(143223524);
		p135.lat_SET(941611560);
		CommunicationChannel.instance.send(p135);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_TERRAIN_REPORT.add((src, ph, pack) ->
		{
			assert (pack.spacing_GET() == (char) 28480);
			assert (pack.pending_GET() == (char) 62738);
			assert (pack.loaded_GET() == (char) 42206);
			assert (pack.lon_GET() == 694424794);
			assert (pack.lat_GET() == 768893290);
			assert (pack.current_height_GET() == 9.092987E37F);
			assert (pack.terrain_height_GET() == -2.6271028E38F);
		});
		GroundControl.TERRAIN_REPORT p136 = CommunicationChannel.new_TERRAIN_REPORT();
		PH.setPack(p136);
		p136.lon_SET(694424794);
		p136.current_height_SET(9.092987E37F);
		p136.terrain_height_SET(-2.6271028E38F);
		p136.spacing_SET((char) 28480);
		p136.pending_SET((char) 62738);
		p136.lat_SET(768893290);
		p136.loaded_SET((char) 42206);
		CommunicationChannel.instance.send(p136);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_SCALED_PRESSURE2.add((src, ph, pack) ->
		{
			assert (pack.press_abs_GET() == 1.4179704E38F);
			assert (pack.press_diff_GET() == 4.2447336E37F);
			assert (pack.time_boot_ms_GET() == 2103290311L);
			assert (pack.temperature_GET() == (short) -9781);
		});
		GroundControl.SCALED_PRESSURE2 p137 = CommunicationChannel.new_SCALED_PRESSURE2();
		PH.setPack(p137);
		p137.press_abs_SET(1.4179704E38F);
		p137.time_boot_ms_SET(2103290311L);
		p137.temperature_SET((short) -9781);
		p137.press_diff_SET(4.2447336E37F);
		CommunicationChannel.instance.send(p137);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		TestChannel.instance.on_ATT_POS_MOCAP.add((src, ph, pack) ->
		{
			assert (pack.y_GET() == -2.7049378E38F);
			assert (Arrays.equals(pack.q_GET(), new float[]{-1.0270433E38F, -6.077185E37F, -3.099441E38F, -1.2144908E37F}));
			assert (pack.time_usec_GET() == 5706515032800076909L);
			assert (pack.z_GET() == 1.7372981E38F);
			assert (pack.x_GET() == 3.2328725E38F);
		});
		GroundControl.ATT_POS_MOCAP p138 = CommunicationChannel.new_ATT_POS_MOCAP();
		PH.setPack(p138);
		p138.y_SET(-2.7049378E38F);
		p138.time_usec_SET(5706515032800076909L);
		p138.z_SET(1.7372981E38F);
		p138.q_SET(new float[]{-1.0270433E38F, -6.077185E37F, -3.099441E38F, -1.2144908E37F}, 0);
		p138.x_SET(3.2328725E38F);
		CommunicationChannel.instance.send(p138);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_SET_ACTUATOR_CONTROL_TARGET.add((src, ph, pack) ->
		{
			assert (pack.target_system_GET() == (char) 112);
			assert (Arrays.equals(pack.controls_GET(), new float[]{4.93181E37F, 8.164912E37F, -1.0408357E38F, -1.0321576E38F, -2.9809207E38F, -5.7756256E37F, 1.2843676E37F, 7.614204E37F}));
			assert (pack.group_mlx_GET() == (char) 197);
			assert (pack.target_component_GET() == (char) 44);
			assert (pack.time_usec_GET() == 8294033749228409625L);
		});
		GroundControl.SET_ACTUATOR_CONTROL_TARGET p139 = CommunicationChannel.new_SET_ACTUATOR_CONTROL_TARGET();
		PH.setPack(p139);
		p139.controls_SET(new float[]{4.93181E37F, 8.164912E37F, -1.0408357E38F, -1.0321576E38F, -2.9809207E38F, -5.7756256E37F, 1.2843676E37F, 7.614204E37F}, 0);
		p139.target_system_SET((char) 112);
		p139.group_mlx_SET((char) 197);
		p139.target_component_SET((char) 44);
		p139.time_usec_SET(8294033749228409625L);
		CommunicationChannel.instance.send(p139);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_ACTUATOR_CONTROL_TARGET.add((src, ph, pack) ->
		{
			assert (pack.time_usec_GET() == 1170258352748219647L);
			assert (Arrays.equals(pack.controls_GET(), new float[]{-1.6588796E38F, 2.1925873E38F, -2.101093E37F, -1.0311746E38F, 2.040275E38F, -3.0996844E38F, -8.479977E37F, 2.0630235E38F}));
			assert (pack.group_mlx_GET() == (char) 154);
		});
		GroundControl.ACTUATOR_CONTROL_TARGET p140 = CommunicationChannel.new_ACTUATOR_CONTROL_TARGET();
		PH.setPack(p140);
		p140.time_usec_SET(1170258352748219647L);
		p140.controls_SET(new float[]{-1.6588796E38F, 2.1925873E38F, -2.101093E37F, -1.0311746E38F, 2.040275E38F, -3.0996844E38F, -8.479977E37F, 2.0630235E38F}, 0);
		p140.group_mlx_SET((char) 154);
		CommunicationChannel.instance.send(p140);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_ALTITUDE.add((src, ph, pack) ->
		{
			assert (pack.altitude_local_GET() == -4.520599E37F);
			assert (pack.time_usec_GET() == 7597672113613948949L);
			assert (pack.altitude_amsl_GET() == 2.7085422E38F);
			assert (pack.bottom_clearance_GET() == -5.081965E37F);
			assert (pack.altitude_relative_GET() == 1.0113681E38F);
			assert (pack.altitude_terrain_GET() == 2.7467376E38F);
			assert (pack.altitude_monotonic_GET() == -2.549433E38F);
		});
		GroundControl.ALTITUDE p141 = CommunicationChannel.new_ALTITUDE();
		PH.setPack(p141);
		p141.bottom_clearance_SET(-5.081965E37F);
		p141.altitude_terrain_SET(2.7467376E38F);
		p141.altitude_amsl_SET(2.7085422E38F);
		p141.altitude_monotonic_SET(-2.549433E38F);
		p141.altitude_relative_SET(1.0113681E38F);
		p141.altitude_local_SET(-4.520599E37F);
		p141.time_usec_SET(7597672113613948949L);
		CommunicationChannel.instance.send(p141);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_RESOURCE_REQUEST.add((src, ph, pack) ->
		{
			assert (pack.uri_type_GET() == (char) 229);
			assert (pack.transfer_type_GET() == (char) 132);
			assert (pack.request_id_GET() == (char) 79);
			assert (Arrays.equals(pack.uri_GET(), new char[]{(char) 55, (char) 86, (char) 141, (char) 59, (char) 97, (char) 20, (char) 190, (char) 238, (char) 127, (char) 28, (char) 8, (char) 227, (char) 244, (char) 227, (char) 231, (char) 169, (char) 146, (char) 18, (char) 154, (char) 163, (char) 184, (char) 49, (char) 147, (char) 176, (char) 58, (char) 218, (char) 218, (char) 2, (char) 75, (char) 125, (char) 44, (char) 245, (char) 201, (char) 15, (char) 56, (char) 105, (char) 47, (char) 28, (char) 132, (char) 75, (char) 183, (char) 138, (char) 119, (char) 169, (char) 60, (char) 26, (char) 162, (char) 236, (char) 131, (char) 144, (char) 85, (char) 81, (char) 255, (char) 90, (char) 180, (char) 233, (char) 238, (char) 145, (char) 180, (char) 101, (char) 253, (char) 179, (char) 112, (char) 87, (char) 224, (char) 181, (char) 82, (char) 215, (char) 108, (char) 220, (char) 201, (char) 180, (char) 125, (char) 28, (char) 26, (char) 174, (char) 165, (char) 11, (char) 232, (char) 80, (char) 63, (char) 53, (char) 214, (char) 75, (char) 136, (char) 96, (char) 118, (char) 162, (char) 44, (char) 223, (char) 31, (char) 187, (char) 82, (char) 143, (char) 230, (char) 222, (char) 47, (char) 42, (char) 128, (char) 150, (char) 229, (char) 9, (char) 33, (char) 154, (char) 132, (char) 76, (char) 222, (char) 104, (char) 73, (char) 111, (char) 19, (char) 126, (char) 160, (char) 63, (char) 178, (char) 21, (char) 157, (char) 90, (char) 86, (char) 65}));
			assert (Arrays.equals(pack.storage_GET(), new char[]{(char) 153, (char) 135, (char) 95, (char) 237, (char) 76, (char) 166, (char) 26, (char) 136, (char) 235, (char) 78, (char) 78, (char) 249, (char) 146, (char) 97, (char) 152, (char) 198, (char) 190, (char) 151, (char) 164, (char) 220, (char) 153, (char) 195, (char) 182, (char) 251, (char) 205, (char) 162, (char) 218, (char) 44, (char) 203, (char) 58, (char) 94, (char) 220, (char) 137, (char) 147, (char) 118, (char) 174, (char) 238, (char) 130, (char) 189, (char) 218, (char) 149, (char) 116, (char) 97, (char) 200, (char) 131, (char) 124, (char) 132, (char) 46, (char) 218, (char) 29, (char) 63, (char) 125, (char) 143, (char) 113, (char) 83, (char) 145, (char) 23, (char) 48, (char) 248, (char) 85, (char) 55, (char) 202, (char) 200, (char) 9, (char) 69, (char) 4, (char) 249, (char) 168, (char) 147, (char) 130, (char) 226, (char) 8, (char) 10, (char) 248, (char) 255, (char) 50, (char) 29, (char) 107, (char) 18, (char) 117, (char) 38, (char) 162, (char) 97, (char) 27, (char) 141, (char) 16, (char) 18, (char) 48, (char) 180, (char) 182, (char) 192, (char) 111, (char) 97, (char) 248, (char) 32, (char) 130, (char) 228, (char) 77, (char) 150, (char) 206, (char) 188, (char) 220, (char) 96, (char) 17, (char) 56, (char) 75, (char) 14, (char) 18, (char) 233, (char) 88, (char) 66, (char) 74, (char) 103, (char) 192, (char) 26, (char) 251, (char) 51, (char) 250, (char) 1, (char) 219}));
		});
		GroundControl.RESOURCE_REQUEST p142 = CommunicationChannel.new_RESOURCE_REQUEST();
		PH.setPack(p142);
		p142.transfer_type_SET((char) 132);
		p142.storage_SET(new char[]{(char) 153, (char) 135, (char) 95, (char) 237, (char) 76, (char) 166, (char) 26, (char) 136, (char) 235, (char) 78, (char) 78, (char) 249, (char) 146, (char) 97, (char) 152, (char) 198, (char) 190, (char) 151, (char) 164, (char) 220, (char) 153, (char) 195, (char) 182, (char) 251, (char) 205, (char) 162, (char) 218, (char) 44, (char) 203, (char) 58, (char) 94, (char) 220, (char) 137, (char) 147, (char) 118, (char) 174, (char) 238, (char) 130, (char) 189, (char) 218, (char) 149, (char) 116, (char) 97, (char) 200, (char) 131, (char) 124, (char) 132, (char) 46, (char) 218, (char) 29, (char) 63, (char) 125, (char) 143, (char) 113, (char) 83, (char) 145, (char) 23, (char) 48, (char) 248, (char) 85, (char) 55, (char) 202, (char) 200, (char) 9, (char) 69, (char) 4, (char) 249, (char) 168, (char) 147, (char) 130, (char) 226, (char) 8, (char) 10, (char) 248, (char) 255, (char) 50, (char) 29, (char) 107, (char) 18, (char) 117, (char) 38, (char) 162, (char) 97, (char) 27, (char) 141, (char) 16, (char) 18, (char) 48, (char) 180, (char) 182, (char) 192, (char) 111, (char) 97, (char) 248, (char) 32, (char) 130, (char) 228, (char) 77, (char) 150, (char) 206, (char) 188, (char) 220, (char) 96, (char) 17, (char) 56, (char) 75, (char) 14, (char) 18, (char) 233, (char) 88, (char) 66, (char) 74, (char) 103, (char) 192, (char) 26, (char) 251, (char) 51, (char) 250, (char) 1, (char) 219}, 0);
		p142.request_id_SET((char) 79);
		p142.uri_SET(new char[]{(char) 55, (char) 86, (char) 141, (char) 59, (char) 97, (char) 20, (char) 190, (char) 238, (char) 127, (char) 28, (char) 8, (char) 227, (char) 244, (char) 227, (char) 231, (char) 169, (char) 146, (char) 18, (char) 154, (char) 163, (char) 184, (char) 49, (char) 147, (char) 176, (char) 58, (char) 218, (char) 218, (char) 2, (char) 75, (char) 125, (char) 44, (char) 245, (char) 201, (char) 15, (char) 56, (char) 105, (char) 47, (char) 28, (char) 132, (char) 75, (char) 183, (char) 138, (char) 119, (char) 169, (char) 60, (char) 26, (char) 162, (char) 236, (char) 131, (char) 144, (char) 85, (char) 81, (char) 255, (char) 90, (char) 180, (char) 233, (char) 238, (char) 145, (char) 180, (char) 101, (char) 253, (char) 179, (char) 112, (char) 87, (char) 224, (char) 181, (char) 82, (char) 215, (char) 108, (char) 220, (char) 201, (char) 180, (char) 125, (char) 28, (char) 26, (char) 174, (char) 165, (char) 11, (char) 232, (char) 80, (char) 63, (char) 53, (char) 214, (char) 75, (char) 136, (char) 96, (char) 118, (char) 162, (char) 44, (char) 223, (char) 31, (char) 187, (char) 82, (char) 143, (char) 230, (char) 222, (char) 47, (char) 42, (char) 128, (char) 150, (char) 229, (char) 9, (char) 33, (char) 154, (char) 132, (char) 76, (char) 222, (char) 104, (char) 73, (char) 111, (char) 19, (char) 126, (char) 160, (char) 63, (char) 178, (char) 21, (char) 157, (char) 90, (char) 86, (char) 65}, 0);
		p142.uri_type_SET((char) 229);
		CommunicationChannel.instance.send(p142);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_SCALED_PRESSURE3.add((src, ph, pack) ->
		{
			assert (pack.press_abs_GET() == -1.9243188E38F);
			assert (pack.press_diff_GET() == 1.9181687E38F);
			assert (pack.time_boot_ms_GET() == 1895419432L);
			assert (pack.temperature_GET() == (short) 9301);
		});
		GroundControl.SCALED_PRESSURE3 p143 = CommunicationChannel.new_SCALED_PRESSURE3();
		PH.setPack(p143);
		p143.press_abs_SET(-1.9243188E38F);
		p143.temperature_SET((short) 9301);
		p143.time_boot_ms_SET(1895419432L);
		p143.press_diff_SET(1.9181687E38F);
		CommunicationChannel.instance.send(p143);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_FOLLOW_TARGET.add((src, ph, pack) ->
		{
			assert (pack.lon_GET() == 928785325);
			assert (pack.alt_GET() == -3.0907396E37F);
			assert (Arrays.equals(pack.position_cov_GET(), new float[]{1.3345714E38F, 7.709156E37F, -4.839481E37F}));
			assert (pack.timestamp_GET() == 4261342103922725824L);
			assert (Arrays.equals(pack.attitude_q_GET(), new float[]{1.200407E37F, -3.1068873E38F, 2.5552242E38F, 1.1269392E38F}));
			assert (pack.custom_state_GET() == 5221402164083455683L);
			assert (Arrays.equals(pack.acc_GET(), new float[]{-3.1429533E38F, 1.5383491E36F, -3.1700704E38F}));
			assert (Arrays.equals(pack.vel_GET(), new float[]{2.8367814E38F, -2.634721E38F, -3.1181035E38F}));
			assert (pack.est_capabilities_GET() == (char) 169);
			assert (Arrays.equals(pack.rates_GET(), new float[]{2.9855105E38F, 1.7671015E38F, 2.814324E38F}));
			assert (pack.lat_GET() == 432367096);
		});
		GroundControl.FOLLOW_TARGET p144 = CommunicationChannel.new_FOLLOW_TARGET();
		PH.setPack(p144);
		p144.vel_SET(new float[]{2.8367814E38F, -2.634721E38F, -3.1181035E38F}, 0);
		p144.acc_SET(new float[]{-3.1429533E38F, 1.5383491E36F, -3.1700704E38F}, 0);
		p144.position_cov_SET(new float[]{1.3345714E38F, 7.709156E37F, -4.839481E37F}, 0);
		p144.lon_SET(928785325);
		p144.alt_SET(-3.0907396E37F);
		p144.timestamp_SET(4261342103922725824L);
		p144.est_capabilities_SET((char) 169);
		p144.custom_state_SET(5221402164083455683L);
		p144.attitude_q_SET(new float[]{1.200407E37F, -3.1068873E38F, 2.5552242E38F, 1.1269392E38F}, 0);
		p144.rates_SET(new float[]{2.9855105E38F, 1.7671015E38F, 2.814324E38F}, 0);
		p144.lat_SET(432367096);
		CommunicationChannel.instance.send(p144);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_CONTROL_SYSTEM_STATE.add((src, ph, pack) ->
		{
			assert (Arrays.equals(pack.vel_variance_GET(), new float[]{-7.9204696E37F, -2.0793855E38F, 2.7502211E38F}));
			assert (pack.roll_rate_GET() == 1.3214503E38F);
			assert (pack.airspeed_GET() == -1.5942704E38F);
			assert (pack.z_pos_GET() == -1.7800242E38F);
			assert (pack.y_acc_GET() == 3.0254003E38F);
			assert (pack.z_vel_GET() == 3.1748693E38F);
			assert (pack.yaw_rate_GET() == 1.5319661E38F);
			assert (pack.x_acc_GET() == -1.4568909E38F);
			assert (pack.z_acc_GET() == 2.0512546E38F);
			assert (Arrays.equals(pack.q_GET(), new float[]{2.6350876E38F, -3.0937155E38F, -3.31861E38F, 3.3454664E38F}));
			assert (pack.x_pos_GET() == 1.9936623E38F);
			assert (pack.time_usec_GET() == 2648385598001828273L);
			assert (pack.y_pos_GET() == -2.2230273E38F);
			assert (pack.pitch_rate_GET() == -2.260959E38F);
			assert (Arrays.equals(pack.pos_variance_GET(), new float[]{1.00183504E36F, -8.491867E37F, -1.840621E38F}));
			assert (pack.y_vel_GET() == 2.8614294E38F);
			assert (pack.x_vel_GET() == -1.271928E38F);
		});
		GroundControl.CONTROL_SYSTEM_STATE p146 = CommunicationChannel.new_CONTROL_SYSTEM_STATE();
		PH.setPack(p146);
		p146.x_vel_SET(-1.271928E38F);
		p146.pos_variance_SET(new float[]{1.00183504E36F, -8.491867E37F, -1.840621E38F}, 0);
		p146.time_usec_SET(2648385598001828273L);
		p146.roll_rate_SET(1.3214503E38F);
		p146.z_acc_SET(2.0512546E38F);
		p146.vel_variance_SET(new float[]{-7.9204696E37F, -2.0793855E38F, 2.7502211E38F}, 0);
		p146.pitch_rate_SET(-2.260959E38F);
		p146.z_vel_SET(3.1748693E38F);
		p146.yaw_rate_SET(1.5319661E38F);
		p146.x_pos_SET(1.9936623E38F);
		p146.x_acc_SET(-1.4568909E38F);
		p146.y_acc_SET(3.0254003E38F);
		p146.airspeed_SET(-1.5942704E38F);
		p146.z_pos_SET(-1.7800242E38F);
		p146.y_pos_SET(-2.2230273E38F);
		p146.q_SET(new float[]{2.6350876E38F, -3.0937155E38F, -3.31861E38F, 3.3454664E38F}, 0);
		p146.y_vel_SET(2.8614294E38F);
		CommunicationChannel.instance.send(p146);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_BATTERY_STATUS.add((src, ph, pack) ->
		{
			assert (pack.temperature_GET() == (short) 18611);
			assert (pack.id_GET() == (char) 52);
			assert (pack.battery_function_GET() == MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_PROPULSION);
			assert (pack.current_battery_GET() == (short) 3619);
			assert (Arrays.equals(pack.voltages_GET(), new char[]{(char) 16055, (char) 2750, (char) 18476, (char) 43684, (char) 50557, (char) 42072, (char) 14437, (char) 29554, (char) 34017, (char) 56170}));
			assert (pack.current_consumed_GET() == -708012859);
			assert (pack.type_GET() == MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_UNKNOWN);
			assert (pack.battery_remaining_GET() == (byte) 35);
			assert (pack.energy_consumed_GET() == -982312749);
		});
		GroundControl.BATTERY_STATUS p147 = CommunicationChannel.new_BATTERY_STATUS();
		PH.setPack(p147);
		p147.type_SET(MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_UNKNOWN);
		p147.current_consumed_SET(-708012859);
		p147.battery_remaining_SET((byte) 35);
		p147.temperature_SET((short) 18611);
		p147.id_SET((char) 52);
		p147.battery_function_SET(MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_PROPULSION);
		p147.current_battery_SET((short) 3619);
		p147.energy_consumed_SET(-982312749);
		p147.voltages_SET(new char[]{(char) 16055, (char) 2750, (char) 18476, (char) 43684, (char) 50557, (char) 42072, (char) 14437, (char) 29554, (char) 34017, (char) 56170}, 0);
		CommunicationChannel.instance.send(p147);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_AUTOPILOT_VERSION.add((src, ph, pack) ->
		{
			assert (pack.uid_GET() == 3084707451939090738L);
			assert (pack.capabilities_GET() == MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_RALLY);
			assert (Arrays.equals(pack.os_custom_version_GET(), new char[]{(char) 130, (char) 174, (char) 193, (char) 112, (char) 176, (char) 40, (char) 123, (char) 195}));
			assert (pack.flight_sw_version_GET() == 1002355964L);
			assert (pack.board_version_GET() == 1104233078L);
			assert (Arrays.equals(pack.middleware_custom_version_GET(), new char[]{(char) 232, (char) 175, (char) 168, (char) 230, (char) 48, (char) 82, (char) 88, (char) 179}));
			assert (pack.middleware_sw_version_GET() == 3048938252L);
			assert (pack.os_sw_version_GET() == 382281840L);
			assert (Arrays.equals(pack.flight_custom_version_GET(), new char[]{(char) 183, (char) 187, (char) 250, (char) 165, (char) 192, (char) 153, (char) 15, (char) 89}));
			assert (pack.product_id_GET() == (char) 44334);
			assert (Arrays.equals(pack.uid2_TRY(ph), new char[]{(char) 240, (char) 48, (char) 92, (char) 116, (char) 112, (char) 120, (char) 52, (char) 71, (char) 254, (char) 33, (char) 22, (char) 68, (char) 244, (char) 252, (char) 137, (char) 141, (char) 114, (char) 255}));
			assert (pack.vendor_id_GET() == (char) 1154);
		});
		GroundControl.AUTOPILOT_VERSION p148 = CommunicationChannel.new_AUTOPILOT_VERSION();
		PH.setPack(p148);
		p148.flight_custom_version_SET(new char[]{(char) 183, (char) 187, (char) 250, (char) 165, (char) 192, (char) 153, (char) 15, (char) 89}, 0);
		p148.vendor_id_SET((char) 1154);
		p148.uid_SET(3084707451939090738L);
		p148.uid2_SET(new char[]{(char) 240, (char) 48, (char) 92, (char) 116, (char) 112, (char) 120, (char) 52, (char) 71, (char) 254, (char) 33, (char) 22, (char) 68, (char) 244, (char) 252, (char) 137, (char) 141, (char) 114, (char) 255}, 0, PH);
		p148.capabilities_SET(MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_RALLY);
		p148.os_custom_version_SET(new char[]{(char) 130, (char) 174, (char) 193, (char) 112, (char) 176, (char) 40, (char) 123, (char) 195}, 0);
		p148.os_sw_version_SET(382281840L);
		p148.middleware_custom_version_SET(new char[]{(char) 232, (char) 175, (char) 168, (char) 230, (char) 48, (char) 82, (char) 88, (char) 179}, 0);
		p148.board_version_SET(1104233078L);
		p148.flight_sw_version_SET(1002355964L);
		p148.product_id_SET((char) 44334);
		p148.middleware_sw_version_SET(3048938252L);
		CommunicationChannel.instance.send(p148);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_LANDING_TARGET.add((src, ph, pack) ->
		{
			assert (pack.x_TRY(ph) == -2.4581345E38F);
			assert (pack.type_GET() == LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_RADIO_BEACON);
			assert (Arrays.equals(pack.q_TRY(ph), new float[]{-2.5518441E38F, -1.0227084E38F, -1.7498199E38F, 1.6778072E38F}));
			assert (pack.position_valid_TRY(ph) == (char) 177);
			assert (pack.z_TRY(ph) == 2.9103438E38F);
			assert (pack.distance_GET() == -3.2541928E38F);
			assert (pack.size_y_GET() == 6.3697643E37F);
			assert (pack.angle_x_GET() == 2.3582226E38F);
			assert (pack.angle_y_GET() == -1.2010541E37F);
			assert (pack.y_TRY(ph) == 2.8718006E38F);
			assert (pack.time_usec_GET() == 685102411810748712L);
			assert (pack.frame_GET() == MAV_FRAME.MAV_FRAME_LOCAL_NED);
			assert (pack.target_num_GET() == (char) 104);
			assert (pack.size_x_GET() == -2.310523E37F);
		});
		GroundControl.LANDING_TARGET p149 = CommunicationChannel.new_LANDING_TARGET();
		PH.setPack(p149);
		p149.target_num_SET((char) 104);
		p149.x_SET(-2.4581345E38F, PH);
		p149.angle_y_SET(-1.2010541E37F);
		p149.type_SET(LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_RADIO_BEACON);
		p149.size_x_SET(-2.310523E37F);
		p149.q_SET(new float[]{-2.5518441E38F, -1.0227084E38F, -1.7498199E38F, 1.6778072E38F}, 0, PH);
		p149.distance_SET(-3.2541928E38F);
		p149.time_usec_SET(685102411810748712L);
		p149.size_y_SET(6.3697643E37F);
		p149.z_SET(2.9103438E38F, PH);
		p149.angle_x_SET(2.3582226E38F);
		p149.y_SET(2.8718006E38F, PH);
		p149.frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_NED);
		p149.position_valid_SET((char) 177, PH);
		CommunicationChannel.instance.send(p149);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_ESTIMATOR_STATUS.add((src, ph, pack) ->
		{
			assert (pack.pos_horiz_accuracy_GET() == 2.2719783E38F);
			assert (pack.pos_vert_accuracy_GET() == -1.0425248E38F);
			assert (pack.pos_vert_ratio_GET() == 1.3541283E38F);
			assert (pack.time_usec_GET() == 7008960734383220425L);
			assert (pack.mag_ratio_GET() == 1.5065085E38F);
			assert (pack.tas_ratio_GET() == 3.4230085E37F);
			assert (pack.vel_ratio_GET() == 9.735612E37F);
			assert (pack.pos_horiz_ratio_GET() == 1.0842421E38F);
			assert (pack.flags_GET() == ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_HORIZ_ABS);
			assert (pack.hagl_ratio_GET() == 2.2503413E38F);
		});
		GroundControl.ESTIMATOR_STATUS p230 = CommunicationChannel.new_ESTIMATOR_STATUS();
		PH.setPack(p230);
		p230.hagl_ratio_SET(2.2503413E38F);
		p230.flags_SET(ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_HORIZ_ABS);
		p230.vel_ratio_SET(9.735612E37F);
		p230.pos_horiz_ratio_SET(1.0842421E38F);
		p230.pos_vert_accuracy_SET(-1.0425248E38F);
		p230.time_usec_SET(7008960734383220425L);
		p230.pos_horiz_accuracy_SET(2.2719783E38F);
		p230.mag_ratio_SET(1.5065085E38F);
		p230.tas_ratio_SET(3.4230085E37F);
		p230.pos_vert_ratio_SET(1.3541283E38F);
		CommunicationChannel.instance.send(p230);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_WIND_COV.add((src, ph, pack) ->
		{
			assert (pack.vert_accuracy_GET() == 2.5174628E38F);
			assert (pack.var_vert_GET() == -1.2607517E38F);
			assert (pack.wind_y_GET() == -2.966681E37F);
			assert (pack.var_horiz_GET() == 1.4345515E38F);
			assert (pack.time_usec_GET() == 7167897036128786771L);
			assert (pack.wind_alt_GET() == 2.2343854E36F);
			assert (pack.wind_x_GET() == -1.9141027E38F);
			assert (pack.horiz_accuracy_GET() == -1.9440327E38F);
			assert (pack.wind_z_GET() == 2.0015302E38F);
		});
		GroundControl.WIND_COV p231 = CommunicationChannel.new_WIND_COV();
		PH.setPack(p231);
		p231.wind_y_SET(-2.966681E37F);
		p231.wind_z_SET(2.0015302E38F);
		p231.vert_accuracy_SET(2.5174628E38F);
		p231.var_vert_SET(-1.2607517E38F);
		p231.horiz_accuracy_SET(-1.9440327E38F);
		p231.var_horiz_SET(1.4345515E38F);
		p231.time_usec_SET(7167897036128786771L);
		p231.wind_x_SET(-1.9141027E38F);
		p231.wind_alt_SET(2.2343854E36F);
		CommunicationChannel.instance.send(p231);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_GPS_INPUT.add((src, ph, pack) ->
		{
			assert (pack.satellites_visible_GET() == (char) 72);
			assert (pack.vd_GET() == 2.0869397E38F);
			assert (pack.ve_GET() == 2.7861003E38F);
			assert (pack.gps_id_GET() == (char) 27);
			assert (pack.time_week_ms_GET() == 2391796478L);
			assert (pack.time_week_GET() == (char) 51219);
			assert (pack.lon_GET() == 674357046);
			assert (pack.fix_type_GET() == (char) 132);
			assert (pack.alt_GET() == 3.2749565E38F);
			assert (pack.vdop_GET() == 8.752333E37F);
			assert (pack.hdop_GET() == -2.1163662E38F);
			assert (pack.lat_GET() == -1294624665);
			assert (pack.vn_GET() == 1.7148412E38F);
			assert (pack.speed_accuracy_GET() == 8.719456E37F);
			assert (pack.vert_accuracy_GET() == -1.0016476E38F);
			assert (pack.time_usec_GET() == 1039084330218034204L);
			assert (pack.horiz_accuracy_GET() == 1.8174122E38F);
			assert (pack.ignore_flags_GET() == GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_ALT);
		});
		GroundControl.GPS_INPUT p232 = CommunicationChannel.new_GPS_INPUT();
		PH.setPack(p232);
		p232.speed_accuracy_SET(8.719456E37F);
		p232.time_usec_SET(1039084330218034204L);
		p232.hdop_SET(-2.1163662E38F);
		p232.vdop_SET(8.752333E37F);
		p232.time_week_ms_SET(2391796478L);
		p232.lat_SET(-1294624665);
		p232.ignore_flags_SET(GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_ALT);
		p232.horiz_accuracy_SET(1.8174122E38F);
		p232.gps_id_SET((char) 27);
		p232.satellites_visible_SET((char) 72);
		p232.vert_accuracy_SET(-1.0016476E38F);
		p232.fix_type_SET((char) 132);
		p232.ve_SET(2.7861003E38F);
		p232.alt_SET(3.2749565E38F);
		p232.time_week_SET((char) 51219);
		p232.lon_SET(674357046);
		p232.vd_SET(2.0869397E38F);
		p232.vn_SET(1.7148412E38F);
		CommunicationChannel.instance.send(p232);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_GPS_RTCM_DATA.add((src, ph, pack) ->
		{
			assert (pack.flags_GET() == (char) 90);
			assert (Arrays.equals(pack.data__GET(), new char[]{(char) 102, (char) 36, (char) 117, (char) 221, (char) 76, (char) 153, (char) 148, (char) 252, (char) 114, (char) 53, (char) 41, (char) 243, (char) 39, (char) 113, (char) 23, (char) 14, (char) 142, (char) 55, (char) 64, (char) 175, (char) 177, (char) 88, (char) 190, (char) 31, (char) 103, (char) 59, (char) 169, (char) 192, (char) 167, (char) 214, (char) 184, (char) 141, (char) 42, (char) 145, (char) 87, (char) 56, (char) 183, (char) 191, (char) 18, (char) 215, (char) 104, (char) 177, (char) 86, (char) 188, (char) 78, (char) 143, (char) 162, (char) 37, (char) 147, (char) 123, (char) 209, (char) 149, (char) 56, (char) 101, (char) 30, (char) 166, (char) 215, (char) 189, (char) 13, (char) 24, (char) 217, (char) 47, (char) 254, (char) 202, (char) 106, (char) 207, (char) 106, (char) 229, (char) 182, (char) 219, (char) 224, (char) 220, (char) 32, (char) 96, (char) 215, (char) 160, (char) 40, (char) 254, (char) 169, (char) 66, (char) 44, (char) 127, (char) 60, (char) 254, (char) 88, (char) 76, (char) 113, (char) 202, (char) 83, (char) 140, (char) 96, (char) 200, (char) 150, (char) 54, (char) 198, (char) 28, (char) 3, (char) 191, (char) 111, (char) 95, (char) 234, (char) 231, (char) 166, (char) 227, (char) 83, (char) 235, (char) 35, (char) 114, (char) 95, (char) 21, (char) 81, (char) 145, (char) 200, (char) 111, (char) 234, (char) 69, (char) 234, (char) 96, (char) 178, (char) 163, (char) 141, (char) 68, (char) 221, (char) 37, (char) 240, (char) 218, (char) 9, (char) 35, (char) 52, (char) 154, (char) 171, (char) 92, (char) 27, (char) 144, (char) 7, (char) 172, (char) 77, (char) 10, (char) 195, (char) 73, (char) 22, (char) 236, (char) 110, (char) 236, (char) 112, (char) 66, (char) 176, (char) 247, (char) 149, (char) 120, (char) 208, (char) 142, (char) 50, (char) 206, (char) 122, (char) 94, (char) 52, (char) 90, (char) 167, (char) 48, (char) 1, (char) 8, (char) 227, (char) 98, (char) 215, (char) 246, (char) 209, (char) 216, (char) 108, (char) 159, (char) 95, (char) 122, (char) 152, (char) 96, (char) 72, (char) 144, (char) 197, (char) 130, (char) 4, (char) 81}));
			assert (pack.len_GET() == (char) 86);
		});
		GroundControl.GPS_RTCM_DATA p233 = CommunicationChannel.new_GPS_RTCM_DATA();
		PH.setPack(p233);
		p233.data__SET(new char[]{(char) 102, (char) 36, (char) 117, (char) 221, (char) 76, (char) 153, (char) 148, (char) 252, (char) 114, (char) 53, (char) 41, (char) 243, (char) 39, (char) 113, (char) 23, (char) 14, (char) 142, (char) 55, (char) 64, (char) 175, (char) 177, (char) 88, (char) 190, (char) 31, (char) 103, (char) 59, (char) 169, (char) 192, (char) 167, (char) 214, (char) 184, (char) 141, (char) 42, (char) 145, (char) 87, (char) 56, (char) 183, (char) 191, (char) 18, (char) 215, (char) 104, (char) 177, (char) 86, (char) 188, (char) 78, (char) 143, (char) 162, (char) 37, (char) 147, (char) 123, (char) 209, (char) 149, (char) 56, (char) 101, (char) 30, (char) 166, (char) 215, (char) 189, (char) 13, (char) 24, (char) 217, (char) 47, (char) 254, (char) 202, (char) 106, (char) 207, (char) 106, (char) 229, (char) 182, (char) 219, (char) 224, (char) 220, (char) 32, (char) 96, (char) 215, (char) 160, (char) 40, (char) 254, (char) 169, (char) 66, (char) 44, (char) 127, (char) 60, (char) 254, (char) 88, (char) 76, (char) 113, (char) 202, (char) 83, (char) 140, (char) 96, (char) 200, (char) 150, (char) 54, (char) 198, (char) 28, (char) 3, (char) 191, (char) 111, (char) 95, (char) 234, (char) 231, (char) 166, (char) 227, (char) 83, (char) 235, (char) 35, (char) 114, (char) 95, (char) 21, (char) 81, (char) 145, (char) 200, (char) 111, (char) 234, (char) 69, (char) 234, (char) 96, (char) 178, (char) 163, (char) 141, (char) 68, (char) 221, (char) 37, (char) 240, (char) 218, (char) 9, (char) 35, (char) 52, (char) 154, (char) 171, (char) 92, (char) 27, (char) 144, (char) 7, (char) 172, (char) 77, (char) 10, (char) 195, (char) 73, (char) 22, (char) 236, (char) 110, (char) 236, (char) 112, (char) 66, (char) 176, (char) 247, (char) 149, (char) 120, (char) 208, (char) 142, (char) 50, (char) 206, (char) 122, (char) 94, (char) 52, (char) 90, (char) 167, (char) 48, (char) 1, (char) 8, (char) 227, (char) 98, (char) 215, (char) 246, (char) 209, (char) 216, (char) 108, (char) 159, (char) 95, (char) 122, (char) 152, (char) 96, (char) 72, (char) 144, (char) 197, (char) 130, (char) 4, (char) 81}, 0);
		p233.len_SET((char) 86);
		p233.flags_SET((char) 90);
		CommunicationChannel.instance.send(p233);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_HIGH_LATENCY.add((src, ph, pack) ->
		{
			assert (pack.wp_distance_GET() == (char) 36681);
			assert (pack.longitude_GET() == -386214537);
			assert (pack.base_mode_GET() == MAV_MODE_FLAG.MAV_MODE_FLAG_TEST_ENABLED);
			assert (pack.climb_rate_GET() == (byte) -70);
			assert (pack.battery_remaining_GET() == (char) 240);
			assert (pack.altitude_amsl_GET() == (short) -23138);
			assert (pack.groundspeed_GET() == (char) 125);
			assert (pack.gps_nsat_GET() == (char) 220);
			assert (pack.throttle_GET() == (byte) -7);
			assert (pack.wp_num_GET() == (char) 80);
			assert (pack.temperature_air_GET() == (byte) -4);
			assert (pack.landed_state_GET() == MAV_LANDED_STATE.MAV_LANDED_STATE_ON_GROUND);
			assert (pack.custom_mode_GET() == 163936760L);
			assert (pack.latitude_GET() == 1990681928);
			assert (pack.heading_sp_GET() == (short) 21675);
			assert (pack.airspeed_GET() == (char) 97);
			assert (pack.pitch_GET() == (short) -17551);
			assert (pack.gps_fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_STATIC);
			assert (pack.roll_GET() == (short) -32620);
			assert (pack.heading_GET() == (char) 18892);
			assert (pack.altitude_sp_GET() == (short) -26785);
			assert (pack.failsafe_GET() == (char) 212);
			assert (pack.airspeed_sp_GET() == (char) 125);
			assert (pack.temperature_GET() == (byte) -55);
		});
		GroundControl.HIGH_LATENCY p234 = CommunicationChannel.new_HIGH_LATENCY();
		PH.setPack(p234);
		p234.groundspeed_SET((char) 125);
		p234.altitude_sp_SET((short) -26785);
		p234.temperature_air_SET((byte) -4);
		p234.altitude_amsl_SET((short) -23138);
		p234.custom_mode_SET(163936760L);
		p234.gps_fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_STATIC);
		p234.gps_nsat_SET((char) 220);
		p234.heading_SET((char) 18892);
		p234.longitude_SET(-386214537);
		p234.airspeed_sp_SET((char) 125);
		p234.landed_state_SET(MAV_LANDED_STATE.MAV_LANDED_STATE_ON_GROUND);
		p234.wp_distance_SET((char) 36681);
		p234.battery_remaining_SET((char) 240);
		p234.climb_rate_SET((byte) -70);
		p234.roll_SET((short) -32620);
		p234.airspeed_SET((char) 97);
		p234.heading_sp_SET((short) 21675);
		p234.temperature_SET((byte) -55);
		p234.pitch_SET((short) -17551);
		p234.latitude_SET(1990681928);
		p234.throttle_SET((byte) -7);
		p234.wp_num_SET((char) 80);
		p234.failsafe_SET((char) 212);
		p234.base_mode_SET(MAV_MODE_FLAG.MAV_MODE_FLAG_TEST_ENABLED);
		CommunicationChannel.instance.send(p234);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_VIBRATION.add((src, ph, pack) ->
		{
			assert (pack.clipping_0_GET() == 2808484929L);
			assert (pack.time_usec_GET() == 1134986323670420350L);
			assert (pack.vibration_y_GET() == 1.9253343E38F);
			assert (pack.vibration_x_GET() == 1.9940614E38F);
			assert (pack.clipping_2_GET() == 1246205287L);
			assert (pack.clipping_1_GET() == 3797777753L);
			assert (pack.vibration_z_GET() == 2.9016291E38F);
		});
		GroundControl.VIBRATION p241 = CommunicationChannel.new_VIBRATION();
		PH.setPack(p241);
		p241.time_usec_SET(1134986323670420350L);
		p241.clipping_2_SET(1246205287L);
		p241.clipping_1_SET(3797777753L);
		p241.vibration_y_SET(1.9253343E38F);
		p241.vibration_x_SET(1.9940614E38F);
		p241.vibration_z_SET(2.9016291E38F);
		p241.clipping_0_SET(2808484929L);
		CommunicationChannel.instance.send(p241);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_HOME_POSITION.add((src, ph, pack) ->
		{
			assert (pack.x_GET() == -1.5060175E38F);
			assert (pack.approach_z_GET() == 8.919155E37F);
			assert (pack.altitude_GET() == 1021178275);
			assert (pack.longitude_GET() == -543709333);
			assert (pack.latitude_GET() == 1550116491);
			assert (Arrays.equals(pack.q_GET(), new float[]{-1.460359E38F, 1.3169139E38F, -2.9379744E38F, 2.2643274E38F}));
			assert (pack.y_GET() == 1.3643475E38F);
			assert (pack.approach_x_GET() == -3.2898022E38F);
			assert (pack.approach_y_GET() == 6.5194535E37F);
			assert (pack.time_usec_TRY(ph) == 5506499825807210191L);
			assert (pack.z_GET() == -3.1976886E38F);
		});
		GroundControl.HOME_POSITION p242 = CommunicationChannel.new_HOME_POSITION();
		PH.setPack(p242);
		p242.q_SET(new float[]{-1.460359E38F, 1.3169139E38F, -2.9379744E38F, 2.2643274E38F}, 0);
		p242.time_usec_SET(5506499825807210191L, PH);
		p242.altitude_SET(1021178275);
		p242.longitude_SET(-543709333);
		p242.approach_x_SET(-3.2898022E38F);
		p242.x_SET(-1.5060175E38F);
		p242.approach_y_SET(6.5194535E37F);
		p242.z_SET(-3.1976886E38F);
		p242.approach_z_SET(8.919155E37F);
		p242.y_SET(1.3643475E38F);
		p242.latitude_SET(1550116491);
		CommunicationChannel.instance.send(p242);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_SET_HOME_POSITION.add((src, ph, pack) ->
		{
			assert (Arrays.equals(pack.q_GET(), new float[]{4.8384454E37F, -1.6594371E38F, -4.2677516E37F, 2.846237E38F}));
			assert (pack.approach_z_GET() == -2.5742773E38F);
			assert (pack.latitude_GET() == 1074307819);
			assert (pack.altitude_GET() == 1868537782);
			assert (pack.approach_y_GET() == -1.716779E38F);
			assert (pack.longitude_GET() == -2137901719);
			assert (pack.target_system_GET() == (char) 57);
			assert (pack.y_GET() == -9.275526E37F);
			assert (pack.z_GET() == 1.8942156E38F);
			assert (pack.x_GET() == -1.4083738E38F);
			assert (pack.approach_x_GET() == 1.3376955E38F);
			assert (pack.time_usec_TRY(ph) == 4753015088821272248L);
		});
		GroundControl.SET_HOME_POSITION p243 = CommunicationChannel.new_SET_HOME_POSITION();
		PH.setPack(p243);
		p243.target_system_SET((char) 57);
		p243.x_SET(-1.4083738E38F);
		p243.altitude_SET(1868537782);
		p243.approach_z_SET(-2.5742773E38F);
		p243.approach_x_SET(1.3376955E38F);
		p243.y_SET(-9.275526E37F);
		p243.time_usec_SET(4753015088821272248L, PH);
		p243.z_SET(1.8942156E38F);
		p243.approach_y_SET(-1.716779E38F);
		p243.longitude_SET(-2137901719);
		p243.q_SET(new float[]{4.8384454E37F, -1.6594371E38F, -4.2677516E37F, 2.846237E38F}, 0);
		p243.latitude_SET(1074307819);
		CommunicationChannel.instance.send(p243);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_MESSAGE_INTERVAL.add((src, ph, pack) ->
		{
			assert (pack.interval_us_GET() == -1009024031);
			assert (pack.message_id_GET() == (char) 43473);
		});
		GroundControl.MESSAGE_INTERVAL p244 = CommunicationChannel.new_MESSAGE_INTERVAL();
		PH.setPack(p244);
		p244.interval_us_SET(-1009024031);
		p244.message_id_SET((char) 43473);
		CommunicationChannel.instance.send(p244);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_EXTENDED_SYS_STATE.add((src, ph, pack) ->
		{
			assert (pack.landed_state_GET() == MAV_LANDED_STATE.MAV_LANDED_STATE_IN_AIR);
			assert (pack.vtol_state_GET() == MAV_VTOL_STATE.MAV_VTOL_STATE_FW);
		});
		GroundControl.EXTENDED_SYS_STATE p245 = CommunicationChannel.new_EXTENDED_SYS_STATE();
		PH.setPack(p245);
		p245.vtol_state_SET(MAV_VTOL_STATE.MAV_VTOL_STATE_FW);
		p245.landed_state_SET(MAV_LANDED_STATE.MAV_LANDED_STATE_IN_AIR);
		CommunicationChannel.instance.send(p245);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_ADSB_VEHICLE.add((src, ph, pack) ->
		{
			assert (pack.altitude_type_GET() == ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_PRESSURE_QNH);
			assert (pack.callsign_LEN(ph) == 8);
			assert (pack.callsign_TRY(ph).equals("rmrqPwgo"));
			assert (pack.lat_GET() == 417739636);
			assert (pack.squawk_GET() == (char) 11340);
			assert (pack.ICAO_address_GET() == 2158010493L);
			assert (pack.flags_GET() == ADSB_FLAGS.ADSB_FLAGS_VALID_VELOCITY);
			assert (pack.tslc_GET() == (char) 177);
			assert (pack.heading_GET() == (char) 61986);
			assert (pack.emitter_type_GET() == ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_SMALL);
			assert (pack.ver_velocity_GET() == (short) 6714);
			assert (pack.lon_GET() == 694403282);
			assert (pack.altitude_GET() == 1799359705);
			assert (pack.hor_velocity_GET() == (char) 17470);
		});
		GroundControl.ADSB_VEHICLE p246 = CommunicationChannel.new_ADSB_VEHICLE();
		PH.setPack(p246);
		p246.callsign_SET("rmrqPwgo", PH);
		p246.emitter_type_SET(ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_SMALL);
		p246.lat_SET(417739636);
		p246.lon_SET(694403282);
		p246.altitude_SET(1799359705);
		p246.flags_SET(ADSB_FLAGS.ADSB_FLAGS_VALID_VELOCITY);
		p246.hor_velocity_SET((char) 17470);
		p246.altitude_type_SET(ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_PRESSURE_QNH);
		p246.ICAO_address_SET(2158010493L);
		p246.ver_velocity_SET((short) 6714);
		p246.squawk_SET((char) 11340);
		p246.tslc_SET((char) 177);
		p246.heading_SET((char) 61986);
		CommunicationChannel.instance.send(p246);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_COLLISION.add((src, ph, pack) ->
		{
			assert (pack.altitude_minimum_delta_GET() == -6.2780497E37F);
			assert (pack.threat_level_GET() == MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_LOW);
			assert (pack.action_GET() == MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_HOVER);
			assert (pack.horizontal_minimum_delta_GET() == -1.663481E38F);
			assert (pack.time_to_minimum_delta_GET() == 1.1275603E38F);
			assert (pack.src__GET() == MAV_COLLISION_SRC.MAV_COLLISION_SRC_ADSB);
			assert (pack.id_GET() == 364457191L);
		});
		GroundControl.COLLISION p247 = CommunicationChannel.new_COLLISION();
		PH.setPack(p247);
		p247.src__SET(MAV_COLLISION_SRC.MAV_COLLISION_SRC_ADSB);
		p247.horizontal_minimum_delta_SET(-1.663481E38F);
		p247.threat_level_SET(MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_LOW);
		p247.id_SET(364457191L);
		p247.action_SET(MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_HOVER);
		p247.altitude_minimum_delta_SET(-6.2780497E37F);
		p247.time_to_minimum_delta_SET(1.1275603E38F);
		CommunicationChannel.instance.send(p247);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_V2_EXTENSION.add((src, ph, pack) ->
		{
			assert (pack.target_network_GET() == (char) 120);
			assert (pack.target_system_GET() == (char) 12);
			assert (pack.target_component_GET() == (char) 219);
			assert (Arrays.equals(pack.payload_GET(), new char[]{(char) 110, (char) 230, (char) 253, (char) 14, (char) 236, (char) 89, (char) 132, (char) 7, (char) 216, (char) 165, (char) 181, (char) 185, (char) 126, (char) 178, (char) 154, (char) 123, (char) 248, (char) 95, (char) 92, (char) 89, (char) 230, (char) 101, (char) 52, (char) 251, (char) 77, (char) 114, (char) 37, (char) 141, (char) 249, (char) 219, (char) 23, (char) 81, (char) 175, (char) 43, (char) 101, (char) 34, (char) 43, (char) 104, (char) 108, (char) 194, (char) 20, (char) 109, (char) 82, (char) 139, (char) 209, (char) 64, (char) 141, (char) 80, (char) 130, (char) 159, (char) 199, (char) 143, (char) 169, (char) 179, (char) 223, (char) 29, (char) 72, (char) 167, (char) 254, (char) 208, (char) 139, (char) 233, (char) 73, (char) 157, (char) 194, (char) 157, (char) 58, (char) 167, (char) 9, (char) 144, (char) 135, (char) 56, (char) 240, (char) 65, (char) 75, (char) 162, (char) 228, (char) 250, (char) 117, (char) 110, (char) 83, (char) 152, (char) 186, (char) 37, (char) 113, (char) 104, (char) 140, (char) 46, (char) 190, (char) 163, (char) 74, (char) 15, (char) 18, (char) 130, (char) 204, (char) 17, (char) 20, (char) 239, (char) 173, (char) 144, (char) 214, (char) 28, (char) 95, (char) 122, (char) 148, (char) 79, (char) 144, (char) 46, (char) 86, (char) 59, (char) 251, (char) 184, (char) 127, (char) 3, (char) 97, (char) 68, (char) 77, (char) 19, (char) 216, (char) 41, (char) 185, (char) 161, (char) 131, (char) 129, (char) 221, (char) 47, (char) 199, (char) 82, (char) 166, (char) 239, (char) 88, (char) 100, (char) 191, (char) 0, (char) 78, (char) 225, (char) 12, (char) 161, (char) 112, (char) 74, (char) 239, (char) 140, (char) 151, (char) 1, (char) 32, (char) 241, (char) 116, (char) 110, (char) 176, (char) 82, (char) 229, (char) 230, (char) 66, (char) 60, (char) 131, (char) 1, (char) 190, (char) 253, (char) 165, (char) 191, (char) 20, (char) 151, (char) 159, (char) 205, (char) 107, (char) 213, (char) 125, (char) 228, (char) 211, (char) 187, (char) 106, (char) 134, (char) 115, (char) 172, (char) 89, (char) 153, (char) 250, (char) 177, (char) 112, (char) 8, (char) 158, (char) 127, (char) 49, (char) 173, (char) 227, (char) 14, (char) 184, (char) 69, (char) 117, (char) 232, (char) 247, (char) 208, (char) 206, (char) 204, (char) 120, (char) 43, (char) 165, (char) 117, (char) 19, (char) 229, (char) 98, (char) 210, (char) 208, (char) 29, (char) 20, (char) 128, (char) 42, (char) 96, (char) 152, (char) 189, (char) 47, (char) 20, (char) 177, (char) 137, (char) 152, (char) 10, (char) 20, (char) 66, (char) 220, (char) 96, (char) 243, (char) 163, (char) 116, (char) 232, (char) 96, (char) 226, (char) 156, (char) 140, (char) 91, (char) 136, (char) 44, (char) 90, (char) 27, (char) 216, (char) 14, (char) 13, (char) 205, (char) 199, (char) 84, (char) 63, (char) 198, (char) 240, (char) 84, (char) 139, (char) 214, (char) 95, (char) 79, (char) 75, (char) 63}));
			assert (pack.message_type_GET() == (char) 31966);
		});
		GroundControl.V2_EXTENSION p248 = CommunicationChannel.new_V2_EXTENSION();
		PH.setPack(p248);
		p248.payload_SET(new char[]{(char) 110, (char) 230, (char) 253, (char) 14, (char) 236, (char) 89, (char) 132, (char) 7, (char) 216, (char) 165, (char) 181, (char) 185, (char) 126, (char) 178, (char) 154, (char) 123, (char) 248, (char) 95, (char) 92, (char) 89, (char) 230, (char) 101, (char) 52, (char) 251, (char) 77, (char) 114, (char) 37, (char) 141, (char) 249, (char) 219, (char) 23, (char) 81, (char) 175, (char) 43, (char) 101, (char) 34, (char) 43, (char) 104, (char) 108, (char) 194, (char) 20, (char) 109, (char) 82, (char) 139, (char) 209, (char) 64, (char) 141, (char) 80, (char) 130, (char) 159, (char) 199, (char) 143, (char) 169, (char) 179, (char) 223, (char) 29, (char) 72, (char) 167, (char) 254, (char) 208, (char) 139, (char) 233, (char) 73, (char) 157, (char) 194, (char) 157, (char) 58, (char) 167, (char) 9, (char) 144, (char) 135, (char) 56, (char) 240, (char) 65, (char) 75, (char) 162, (char) 228, (char) 250, (char) 117, (char) 110, (char) 83, (char) 152, (char) 186, (char) 37, (char) 113, (char) 104, (char) 140, (char) 46, (char) 190, (char) 163, (char) 74, (char) 15, (char) 18, (char) 130, (char) 204, (char) 17, (char) 20, (char) 239, (char) 173, (char) 144, (char) 214, (char) 28, (char) 95, (char) 122, (char) 148, (char) 79, (char) 144, (char) 46, (char) 86, (char) 59, (char) 251, (char) 184, (char) 127, (char) 3, (char) 97, (char) 68, (char) 77, (char) 19, (char) 216, (char) 41, (char) 185, (char) 161, (char) 131, (char) 129, (char) 221, (char) 47, (char) 199, (char) 82, (char) 166, (char) 239, (char) 88, (char) 100, (char) 191, (char) 0, (char) 78, (char) 225, (char) 12, (char) 161, (char) 112, (char) 74, (char) 239, (char) 140, (char) 151, (char) 1, (char) 32, (char) 241, (char) 116, (char) 110, (char) 176, (char) 82, (char) 229, (char) 230, (char) 66, (char) 60, (char) 131, (char) 1, (char) 190, (char) 253, (char) 165, (char) 191, (char) 20, (char) 151, (char) 159, (char) 205, (char) 107, (char) 213, (char) 125, (char) 228, (char) 211, (char) 187, (char) 106, (char) 134, (char) 115, (char) 172, (char) 89, (char) 153, (char) 250, (char) 177, (char) 112, (char) 8, (char) 158, (char) 127, (char) 49, (char) 173, (char) 227, (char) 14, (char) 184, (char) 69, (char) 117, (char) 232, (char) 247, (char) 208, (char) 206, (char) 204, (char) 120, (char) 43, (char) 165, (char) 117, (char) 19, (char) 229, (char) 98, (char) 210, (char) 208, (char) 29, (char) 20, (char) 128, (char) 42, (char) 96, (char) 152, (char) 189, (char) 47, (char) 20, (char) 177, (char) 137, (char) 152, (char) 10, (char) 20, (char) 66, (char) 220, (char) 96, (char) 243, (char) 163, (char) 116, (char) 232, (char) 96, (char) 226, (char) 156, (char) 140, (char) 91, (char) 136, (char) 44, (char) 90, (char) 27, (char) 216, (char) 14, (char) 13, (char) 205, (char) 199, (char) 84, (char) 63, (char) 198, (char) 240, (char) 84, (char) 139, (char) 214, (char) 95, (char) 79, (char) 75, (char) 63}, 0);
		p248.target_network_SET((char) 120);
		p248.target_component_SET((char) 219);
		p248.message_type_SET((char) 31966);
		p248.target_system_SET((char) 12);
		CommunicationChannel.instance.send(p248);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_MEMORY_VECT.add((src, ph, pack) ->
		{
			assert (pack.address_GET() == (char) 16315);
			assert (pack.ver_GET() == (char) 184);
			assert (pack.type_GET() == (char) 114);
			assert (Arrays.equals(pack.value_GET(), new byte[]{(byte) 59, (byte) 18, (byte) -3, (byte) 80, (byte) 0, (byte) 52, (byte) -111, (byte) -16, (byte) -15, (byte) 10, (byte) -96, (byte) 51, (byte) -109, (byte) 4, (byte) 22, (byte) 9, (byte) 33, (byte) -59, (byte) 6, (byte) -46, (byte) 35, (byte) 65, (byte) 93, (byte) 112, (byte) -57, (byte) -104, (byte) 103, (byte) 122, (byte) -95, (byte) -75, (byte) 119, (byte) -30}));
		});
		GroundControl.MEMORY_VECT p249 = CommunicationChannel.new_MEMORY_VECT();
		PH.setPack(p249);
		p249.value_SET(new byte[]{(byte) 59, (byte) 18, (byte) -3, (byte) 80, (byte) 0, (byte) 52, (byte) -111, (byte) -16, (byte) -15, (byte) 10, (byte) -96, (byte) 51, (byte) -109, (byte) 4, (byte) 22, (byte) 9, (byte) 33, (byte) -59, (byte) 6, (byte) -46, (byte) 35, (byte) 65, (byte) 93, (byte) 112, (byte) -57, (byte) -104, (byte) 103, (byte) 122, (byte) -95, (byte) -75, (byte) 119, (byte) -30}, 0);
		p249.type_SET((char) 114);
		p249.ver_SET((char) 184);
		p249.address_SET((char) 16315);
		CommunicationChannel.instance.send(p249);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_DEBUG_VECT.add((src, ph, pack) ->
		{
			assert (pack.x_GET() == 5.5800454E35F);
			assert (pack.time_usec_GET() == 4032496653148853272L);
			assert (pack.y_GET() == 2.5387778E38F);
			assert (pack.name_LEN(ph) == 9);
			assert (pack.name_TRY(ph).equals("hubsbkNVn"));
			assert (pack.z_GET() == 6.6148944E37F);
		});
		GroundControl.DEBUG_VECT p250 = CommunicationChannel.new_DEBUG_VECT();
		PH.setPack(p250);
		p250.y_SET(2.5387778E38F);
		p250.time_usec_SET(4032496653148853272L);
		p250.x_SET(5.5800454E35F);
		p250.z_SET(6.6148944E37F);
		p250.name_SET("hubsbkNVn", PH);
		CommunicationChannel.instance.send(p250);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_NAMED_VALUE_FLOAT.add((src, ph, pack) ->
		{
			assert (pack.time_boot_ms_GET() == 853496995L);
			assert (pack.value_GET() == 5.320639E36F);
			assert (pack.name_LEN(ph) == 6);
			assert (pack.name_TRY(ph).equals("vhegdl"));
		});
		GroundControl.NAMED_VALUE_FLOAT p251 = CommunicationChannel.new_NAMED_VALUE_FLOAT();
		PH.setPack(p251);
		p251.value_SET(5.320639E36F);
		p251.name_SET("vhegdl", PH);
		p251.time_boot_ms_SET(853496995L);
		CommunicationChannel.instance.send(p251);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_NAMED_VALUE_INT.add((src, ph, pack) ->
		{
			assert (pack.name_LEN(ph) == 10);
			assert (pack.name_TRY(ph).equals("lmcksjnnjz"));
			assert (pack.time_boot_ms_GET() == 3370784089L);
			assert (pack.value_GET() == 1056283846);
		});
		GroundControl.NAMED_VALUE_INT p252 = CommunicationChannel.new_NAMED_VALUE_INT();
		PH.setPack(p252);
		p252.value_SET(1056283846);
		p252.time_boot_ms_SET(3370784089L);
		p252.name_SET("lmcksjnnjz", PH);
		CommunicationChannel.instance.send(p252);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_STATUSTEXT.add((src, ph, pack) ->
		{
			assert (pack.severity_GET() == MAV_SEVERITY.MAV_SEVERITY_EMERGENCY);
			assert (pack.text_LEN(ph) == 9);
			assert (pack.text_TRY(ph).equals("pbxFFbglb"));
		});
		GroundControl.STATUSTEXT p253 = CommunicationChannel.new_STATUSTEXT();
		PH.setPack(p253);
		p253.severity_SET(MAV_SEVERITY.MAV_SEVERITY_EMERGENCY);
		p253.text_SET("pbxFFbglb", PH);
		CommunicationChannel.instance.send(p253);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_DEBUG.add((src, ph, pack) ->
		{
			assert (pack.ind_GET() == (char) 110);
			assert (pack.value_GET() == 2.4850534E38F);
			assert (pack.time_boot_ms_GET() == 1912130799L);
		});
		GroundControl.DEBUG p254 = CommunicationChannel.new_DEBUG();
		PH.setPack(p254);
		p254.ind_SET((char) 110);
		p254.time_boot_ms_SET(1912130799L);
		p254.value_SET(2.4850534E38F);
		CommunicationChannel.instance.send(p254);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_SETUP_SIGNING.add((src, ph, pack) ->
		{
			assert (pack.target_component_GET() == (char) 245);
			assert (pack.initial_timestamp_GET() == 5236016738074641645L);
			assert (pack.target_system_GET() == (char) 161);
			assert (Arrays.equals(pack.secret_key_GET(), new char[]{(char) 23, (char) 27, (char) 38, (char) 141, (char) 225, (char) 125, (char) 115, (char) 50, (char) 141, (char) 120, (char) 28, (char) 28, (char) 191, (char) 152, (char) 92, (char) 114, (char) 209, (char) 180, (char) 119, (char) 155, (char) 64, (char) 40, (char) 160, (char) 116, (char) 252, (char) 2, (char) 28, (char) 143, (char) 84, (char) 215, (char) 138, (char) 78}));
		});
		GroundControl.SETUP_SIGNING p256 = CommunicationChannel.new_SETUP_SIGNING();
		PH.setPack(p256);
		p256.secret_key_SET(new char[]{(char) 23, (char) 27, (char) 38, (char) 141, (char) 225, (char) 125, (char) 115, (char) 50, (char) 141, (char) 120, (char) 28, (char) 28, (char) 191, (char) 152, (char) 92, (char) 114, (char) 209, (char) 180, (char) 119, (char) 155, (char) 64, (char) 40, (char) 160, (char) 116, (char) 252, (char) 2, (char) 28, (char) 143, (char) 84, (char) 215, (char) 138, (char) 78}, 0);
		p256.initial_timestamp_SET(5236016738074641645L);
		p256.target_system_SET((char) 161);
		p256.target_component_SET((char) 245);
		CommunicationChannel.instance.send(p256);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_BUTTON_CHANGE.add((src, ph, pack) ->
		{
			assert (pack.time_boot_ms_GET() == 3438678411L);
			assert (pack.state_GET() == (char) 171);
			assert (pack.last_change_ms_GET() == 4249874775L);
		});
		GroundControl.BUTTON_CHANGE p257 = CommunicationChannel.new_BUTTON_CHANGE();
		PH.setPack(p257);
		p257.last_change_ms_SET(4249874775L);
		p257.time_boot_ms_SET(3438678411L);
		p257.state_SET((char) 171);
		CommunicationChannel.instance.send(p257);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_PLAY_TUNE.add((src, ph, pack) ->
		{
			assert (pack.tune_LEN(ph) == 1);
			assert (pack.tune_TRY(ph).equals("f"));
			assert (pack.target_component_GET() == (char) 252);
			assert (pack.target_system_GET() == (char) 153);
		});
		GroundControl.PLAY_TUNE p258 = CommunicationChannel.new_PLAY_TUNE();
		PH.setPack(p258);
		p258.tune_SET("f", PH);
		p258.target_component_SET((char) 252);
		p258.target_system_SET((char) 153);
		CommunicationChannel.instance.send(p258);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_CAMERA_INFORMATION.add((src, ph, pack) ->
		{
			assert (pack.resolution_h_GET() == (char) 19361);
			assert (pack.sensor_size_h_GET() == 1.3025551E37F);
			assert (pack.sensor_size_v_GET() == -2.083406E37F);
			assert (pack.focal_length_GET() == 5.7812296E37F);
			assert (pack.time_boot_ms_GET() == 1517385831L);
			assert (pack.firmware_version_GET() == 2596440766L);
			assert (pack.flags_GET() == CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE);
			assert (pack.resolution_v_GET() == (char) 31365);
			assert (Arrays.equals(pack.model_name_GET(), new char[]{(char) 98, (char) 131, (char) 200, (char) 183, (char) 179, (char) 75, (char) 169, (char) 43, (char) 81, (char) 219, (char) 158, (char) 134, (char) 171, (char) 178, (char) 14, (char) 46, (char) 163, (char) 205, (char) 63, (char) 89, (char) 63, (char) 88, (char) 15, (char) 228, (char) 176, (char) 168, (char) 164, (char) 120, (char) 155, (char) 149, (char) 229, (char) 217}));
			assert (pack.lens_id_GET() == (char) 185);
			assert (pack.cam_definition_uri_LEN(ph) == 43);
			assert (pack.cam_definition_uri_TRY(ph).equals("WrfnxlyVqpeixvlVciunchwxqsnyOoedbbbpyjHwljt"));
			assert (pack.cam_definition_version_GET() == (char) 35619);
			assert (Arrays.equals(pack.vendor_name_GET(), new char[]{(char) 3, (char) 135, (char) 93, (char) 239, (char) 55, (char) 125, (char) 126, (char) 209, (char) 154, (char) 146, (char) 226, (char) 45, (char) 225, (char) 200, (char) 68, (char) 144, (char) 70, (char) 150, (char) 151, (char) 76, (char) 132, (char) 134, (char) 179, (char) 166, (char) 147, (char) 66, (char) 133, (char) 214, (char) 149, (char) 246, (char) 51, (char) 50}));
		});
		GroundControl.CAMERA_INFORMATION p259 = CommunicationChannel.new_CAMERA_INFORMATION();
		PH.setPack(p259);
		p259.resolution_v_SET((char) 31365);
		p259.resolution_h_SET((char) 19361);
		p259.lens_id_SET((char) 185);
		p259.cam_definition_version_SET((char) 35619);
		p259.firmware_version_SET(2596440766L);
		p259.cam_definition_uri_SET("WrfnxlyVqpeixvlVciunchwxqsnyOoedbbbpyjHwljt", PH);
		p259.sensor_size_h_SET(1.3025551E37F);
		p259.flags_SET(CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE);
		p259.vendor_name_SET(new char[]{(char) 3, (char) 135, (char) 93, (char) 239, (char) 55, (char) 125, (char) 126, (char) 209, (char) 154, (char) 146, (char) 226, (char) 45, (char) 225, (char) 200, (char) 68, (char) 144, (char) 70, (char) 150, (char) 151, (char) 76, (char) 132, (char) 134, (char) 179, (char) 166, (char) 147, (char) 66, (char) 133, (char) 214, (char) 149, (char) 246, (char) 51, (char) 50}, 0);
		p259.model_name_SET(new char[]{(char) 98, (char) 131, (char) 200, (char) 183, (char) 179, (char) 75, (char) 169, (char) 43, (char) 81, (char) 219, (char) 158, (char) 134, (char) 171, (char) 178, (char) 14, (char) 46, (char) 163, (char) 205, (char) 63, (char) 89, (char) 63, (char) 88, (char) 15, (char) 228, (char) 176, (char) 168, (char) 164, (char) 120, (char) 155, (char) 149, (char) 229, (char) 217}, 0);
		p259.focal_length_SET(5.7812296E37F);
		p259.time_boot_ms_SET(1517385831L);
		p259.sensor_size_v_SET(-2.083406E37F);
		CommunicationChannel.instance.send(p259);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_CAMERA_SETTINGS.add((src, ph, pack) ->
		{
			assert (pack.time_boot_ms_GET() == 2548893502L);
			assert (pack.mode_id_GET() == CAMERA_MODE.CAMERA_MODE_IMAGE);
		});
		GroundControl.CAMERA_SETTINGS p260 = CommunicationChannel.new_CAMERA_SETTINGS();
		PH.setPack(p260);
		p260.time_boot_ms_SET(2548893502L);
		p260.mode_id_SET(CAMERA_MODE.CAMERA_MODE_IMAGE);
		CommunicationChannel.instance.send(p260);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_STORAGE_INFORMATION.add((src, ph, pack) ->
		{
			assert (pack.storage_count_GET() == (char) 112);
			assert (pack.status_GET() == (char) 81);
			assert (pack.storage_id_GET() == (char) 132);
			assert (pack.used_capacity_GET() == 1.8515623E38F);
			assert (pack.write_speed_GET() == 2.7005179E38F);
			assert (pack.available_capacity_GET() == -7.2018745E37F);
			assert (pack.total_capacity_GET() == -1.2135605E38F);
			assert (pack.read_speed_GET() == 1.3734448E38F);
			assert (pack.time_boot_ms_GET() == 3219320399L);
		});
		GroundControl.STORAGE_INFORMATION p261 = CommunicationChannel.new_STORAGE_INFORMATION();
		PH.setPack(p261);
		p261.status_SET((char) 81);
		p261.write_speed_SET(2.7005179E38F);
		p261.total_capacity_SET(-1.2135605E38F);
		p261.available_capacity_SET(-7.2018745E37F);
		p261.time_boot_ms_SET(3219320399L);
		p261.storage_count_SET((char) 112);
		p261.storage_id_SET((char) 132);
		p261.used_capacity_SET(1.8515623E38F);
		p261.read_speed_SET(1.3734448E38F);
		CommunicationChannel.instance.send(p261);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_CAMERA_CAPTURE_STATUS.add((src, ph, pack) ->
		{
			assert (pack.image_status_GET() == (char) 140);
			assert (pack.image_interval_GET() == 7.150301E37F);
			assert (pack.available_capacity_GET() == 7.2641963E36F);
			assert (pack.time_boot_ms_GET() == 1498803603L);
			assert (pack.recording_time_ms_GET() == 1324171286L);
			assert (pack.video_status_GET() == (char) 38);
		});
		GroundControl.CAMERA_CAPTURE_STATUS p262 = CommunicationChannel.new_CAMERA_CAPTURE_STATUS();
		PH.setPack(p262);
		p262.recording_time_ms_SET(1324171286L);
		p262.available_capacity_SET(7.2641963E36F);
		p262.image_status_SET((char) 140);
		p262.image_interval_SET(7.150301E37F);
		p262.time_boot_ms_SET(1498803603L);
		p262.video_status_SET((char) 38);
		CommunicationChannel.instance.send(p262);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_CAMERA_IMAGE_CAPTURED.add((src, ph, pack) ->
		{
			assert (pack.time_boot_ms_GET() == 1975525315L);
			assert (pack.relative_alt_GET() == 1620053713);
			assert (Arrays.equals(pack.q_GET(), new float[]{8.2706266E37F, -1.3197994E38F, 7.8200407E37F, 7.031144E37F}));
			assert (pack.time_utc_GET() == 5765149284014352378L);
			assert (pack.file_url_LEN(ph) == 125);
			assert (pack.file_url_TRY(ph).equals("qxnfufcfXcbPsgyzuhorlgxubfuvEkrOVvfcmctrlhjikQRmsgjxEgaozysqjvMqjfbfmbxygttrspxefCbjkhomyitRywamwhqyzonpioiSlbvxzhuwdfdnUjvnu"));
			assert (pack.camera_id_GET() == (char) 29);
			assert (pack.image_index_GET() == -367339620);
			assert (pack.alt_GET() == 2064364721);
			assert (pack.lat_GET() == 480553868);
			assert (pack.lon_GET() == -1022460047);
			assert (pack.capture_result_GET() == (byte) 6);
		});
		GroundControl.CAMERA_IMAGE_CAPTURED p263 = CommunicationChannel.new_CAMERA_IMAGE_CAPTURED();
		PH.setPack(p263);
		p263.q_SET(new float[]{8.2706266E37F, -1.3197994E38F, 7.8200407E37F, 7.031144E37F}, 0);
		p263.time_utc_SET(5765149284014352378L);
		p263.alt_SET(2064364721);
		p263.relative_alt_SET(1620053713);
		p263.lat_SET(480553868);
		p263.file_url_SET("qxnfufcfXcbPsgyzuhorlgxubfuvEkrOVvfcmctrlhjikQRmsgjxEgaozysqjvMqjfbfmbxygttrspxefCbjkhomyitRywamwhqyzonpioiSlbvxzhuwdfdnUjvnu", PH);
		p263.capture_result_SET((byte) 6);
		p263.image_index_SET(-367339620);
		p263.lon_SET(-1022460047);
		p263.camera_id_SET((char) 29);
		p263.time_boot_ms_SET(1975525315L);
		CommunicationChannel.instance.send(p263);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_FLIGHT_INFORMATION.add((src, ph, pack) ->
		{
			assert (pack.flight_uuid_GET() == 2739593138303355328L);
			assert (pack.arming_time_utc_GET() == 514221168500439578L);
			assert (pack.takeoff_time_utc_GET() == 7936833764386839600L);
			assert (pack.time_boot_ms_GET() == 2595147003L);
		});
		GroundControl.FLIGHT_INFORMATION p264 = CommunicationChannel.new_FLIGHT_INFORMATION();
		PH.setPack(p264);
		p264.flight_uuid_SET(2739593138303355328L);
		p264.arming_time_utc_SET(514221168500439578L);
		p264.time_boot_ms_SET(2595147003L);
		p264.takeoff_time_utc_SET(7936833764386839600L);
		CommunicationChannel.instance.send(p264);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_MOUNT_ORIENTATION.add((src, ph, pack) ->
		{
			assert (pack.roll_GET() == 2.44347E38F);
			assert (pack.time_boot_ms_GET() == 3360978810L);
			assert (pack.pitch_GET() == 2.3218844E37F);
			assert (pack.yaw_GET() == -2.570165E38F);
		});
		GroundControl.MOUNT_ORIENTATION p265 = CommunicationChannel.new_MOUNT_ORIENTATION();
		PH.setPack(p265);
		p265.time_boot_ms_SET(3360978810L);
		p265.roll_SET(2.44347E38F);
		p265.yaw_SET(-2.570165E38F);
		p265.pitch_SET(2.3218844E37F);
		CommunicationChannel.instance.send(p265);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_LOGGING_DATA.add((src, ph, pack) ->
		{
			assert (pack.target_system_GET() == (char) 44);
			assert (pack.first_message_offset_GET() == (char) 225);
			assert (pack.length_GET() == (char) 66);
			assert (Arrays.equals(pack.data__GET(), new char[]{(char) 10, (char) 112, (char) 213, (char) 253, (char) 45, (char) 161, (char) 46, (char) 112, (char) 162, (char) 31, (char) 155, (char) 114, (char) 94, (char) 194, (char) 142, (char) 140, (char) 58, (char) 212, (char) 9, (char) 205, (char) 205, (char) 251, (char) 211, (char) 43, (char) 141, (char) 143, (char) 168, (char) 56, (char) 181, (char) 3, (char) 234, (char) 65, (char) 53, (char) 104, (char) 122, (char) 36, (char) 212, (char) 99, (char) 83, (char) 84, (char) 173, (char) 145, (char) 143, (char) 168, (char) 195, (char) 173, (char) 10, (char) 221, (char) 20, (char) 130, (char) 195, (char) 85, (char) 248, (char) 60, (char) 253, (char) 111, (char) 33, (char) 177, (char) 100, (char) 226, (char) 52, (char) 203, (char) 34, (char) 30, (char) 145, (char) 82, (char) 44, (char) 221, (char) 228, (char) 86, (char) 111, (char) 51, (char) 47, (char) 62, (char) 101, (char) 153, (char) 217, (char) 95, (char) 217, (char) 57, (char) 33, (char) 253, (char) 200, (char) 204, (char) 141, (char) 245, (char) 91, (char) 142, (char) 242, (char) 110, (char) 147, (char) 81, (char) 224, (char) 220, (char) 95, (char) 183, (char) 166, (char) 165, (char) 19, (char) 48, (char) 82, (char) 239, (char) 189, (char) 37, (char) 81, (char) 179, (char) 88, (char) 190, (char) 190, (char) 51, (char) 60, (char) 67, (char) 222, (char) 124, (char) 47, (char) 113, (char) 160, (char) 145, (char) 195, (char) 124, (char) 115, (char) 103, (char) 206, (char) 161, (char) 126, (char) 181, (char) 115, (char) 148, (char) 138, (char) 176, (char) 51, (char) 123, (char) 160, (char) 229, (char) 159, (char) 217, (char) 108, (char) 30, (char) 240, (char) 227, (char) 106, (char) 121, (char) 113, (char) 215, (char) 98, (char) 119, (char) 95, (char) 92, (char) 112, (char) 196, (char) 102, (char) 96, (char) 234, (char) 251, (char) 79, (char) 40, (char) 182, (char) 172, (char) 214, (char) 154, (char) 1, (char) 191, (char) 130, (char) 140, (char) 47, (char) 193, (char) 229, (char) 204, (char) 154, (char) 191, (char) 50, (char) 17, (char) 206, (char) 199, (char) 83, (char) 151, (char) 18, (char) 53, (char) 124, (char) 229, (char) 236, (char) 158, (char) 119, (char) 142, (char) 231, (char) 236, (char) 87, (char) 254, (char) 121, (char) 212, (char) 252, (char) 253, (char) 156, (char) 133, (char) 225, (char) 118, (char) 63, (char) 162, (char) 129, (char) 130, (char) 118, (char) 19, (char) 110, (char) 245, (char) 151, (char) 197, (char) 178, (char) 150, (char) 121, (char) 94, (char) 32, (char) 109, (char) 19, (char) 153, (char) 71, (char) 112, (char) 215, (char) 145, (char) 188, (char) 97, (char) 242, (char) 244, (char) 137, (char) 229, (char) 229, (char) 133, (char) 61, (char) 166, (char) 81, (char) 107, (char) 110, (char) 235, (char) 125, (char) 229, (char) 253, (char) 130, (char) 99, (char) 31, (char) 114, (char) 117, (char) 155, (char) 246, (char) 67, (char) 80, (char) 109, (char) 102, (char) 30, (char) 175, (char) 83}));
			assert (pack.target_component_GET() == (char) 160);
			assert (pack.sequence_GET() == (char) 27918);
		});
		GroundControl.LOGGING_DATA p266 = CommunicationChannel.new_LOGGING_DATA();
		PH.setPack(p266);
		p266.target_system_SET((char) 44);
		p266.data__SET(new char[]{(char) 10, (char) 112, (char) 213, (char) 253, (char) 45, (char) 161, (char) 46, (char) 112, (char) 162, (char) 31, (char) 155, (char) 114, (char) 94, (char) 194, (char) 142, (char) 140, (char) 58, (char) 212, (char) 9, (char) 205, (char) 205, (char) 251, (char) 211, (char) 43, (char) 141, (char) 143, (char) 168, (char) 56, (char) 181, (char) 3, (char) 234, (char) 65, (char) 53, (char) 104, (char) 122, (char) 36, (char) 212, (char) 99, (char) 83, (char) 84, (char) 173, (char) 145, (char) 143, (char) 168, (char) 195, (char) 173, (char) 10, (char) 221, (char) 20, (char) 130, (char) 195, (char) 85, (char) 248, (char) 60, (char) 253, (char) 111, (char) 33, (char) 177, (char) 100, (char) 226, (char) 52, (char) 203, (char) 34, (char) 30, (char) 145, (char) 82, (char) 44, (char) 221, (char) 228, (char) 86, (char) 111, (char) 51, (char) 47, (char) 62, (char) 101, (char) 153, (char) 217, (char) 95, (char) 217, (char) 57, (char) 33, (char) 253, (char) 200, (char) 204, (char) 141, (char) 245, (char) 91, (char) 142, (char) 242, (char) 110, (char) 147, (char) 81, (char) 224, (char) 220, (char) 95, (char) 183, (char) 166, (char) 165, (char) 19, (char) 48, (char) 82, (char) 239, (char) 189, (char) 37, (char) 81, (char) 179, (char) 88, (char) 190, (char) 190, (char) 51, (char) 60, (char) 67, (char) 222, (char) 124, (char) 47, (char) 113, (char) 160, (char) 145, (char) 195, (char) 124, (char) 115, (char) 103, (char) 206, (char) 161, (char) 126, (char) 181, (char) 115, (char) 148, (char) 138, (char) 176, (char) 51, (char) 123, (char) 160, (char) 229, (char) 159, (char) 217, (char) 108, (char) 30, (char) 240, (char) 227, (char) 106, (char) 121, (char) 113, (char) 215, (char) 98, (char) 119, (char) 95, (char) 92, (char) 112, (char) 196, (char) 102, (char) 96, (char) 234, (char) 251, (char) 79, (char) 40, (char) 182, (char) 172, (char) 214, (char) 154, (char) 1, (char) 191, (char) 130, (char) 140, (char) 47, (char) 193, (char) 229, (char) 204, (char) 154, (char) 191, (char) 50, (char) 17, (char) 206, (char) 199, (char) 83, (char) 151, (char) 18, (char) 53, (char) 124, (char) 229, (char) 236, (char) 158, (char) 119, (char) 142, (char) 231, (char) 236, (char) 87, (char) 254, (char) 121, (char) 212, (char) 252, (char) 253, (char) 156, (char) 133, (char) 225, (char) 118, (char) 63, (char) 162, (char) 129, (char) 130, (char) 118, (char) 19, (char) 110, (char) 245, (char) 151, (char) 197, (char) 178, (char) 150, (char) 121, (char) 94, (char) 32, (char) 109, (char) 19, (char) 153, (char) 71, (char) 112, (char) 215, (char) 145, (char) 188, (char) 97, (char) 242, (char) 244, (char) 137, (char) 229, (char) 229, (char) 133, (char) 61, (char) 166, (char) 81, (char) 107, (char) 110, (char) 235, (char) 125, (char) 229, (char) 253, (char) 130, (char) 99, (char) 31, (char) 114, (char) 117, (char) 155, (char) 246, (char) 67, (char) 80, (char) 109, (char) 102, (char) 30, (char) 175, (char) 83}, 0);
		p266.sequence_SET((char) 27918);
		p266.length_SET((char) 66);
		p266.first_message_offset_SET((char) 225);
		p266.target_component_SET((char) 160);
		CommunicationChannel.instance.send(p266);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_LOGGING_DATA_ACKED.add((src, ph, pack) ->
		{
			assert (pack.target_system_GET() == (char) 210);
			assert (pack.sequence_GET() == (char) 8885);
			assert (pack.length_GET() == (char) 141);
			assert (pack.first_message_offset_GET() == (char) 110);
			assert (pack.target_component_GET() == (char) 25);
			assert (Arrays.equals(pack.data__GET(), new char[]{(char) 250, (char) 69, (char) 19, (char) 229, (char) 246, (char) 95, (char) 154, (char) 3, (char) 196, (char) 91, (char) 152, (char) 236, (char) 145, (char) 226, (char) 13, (char) 106, (char) 90, (char) 233, (char) 170, (char) 253, (char) 116, (char) 48, (char) 101, (char) 249, (char) 159, (char) 79, (char) 166, (char) 230, (char) 107, (char) 36, (char) 5, (char) 168, (char) 213, (char) 127, (char) 179, (char) 121, (char) 91, (char) 242, (char) 140, (char) 104, (char) 106, (char) 14, (char) 234, (char) 225, (char) 74, (char) 112, (char) 41, (char) 233, (char) 80, (char) 250, (char) 222, (char) 232, (char) 207, (char) 240, (char) 137, (char) 41, (char) 220, (char) 123, (char) 246, (char) 156, (char) 232, (char) 160, (char) 228, (char) 98, (char) 106, (char) 190, (char) 143, (char) 62, (char) 88, (char) 226, (char) 56, (char) 205, (char) 129, (char) 235, (char) 241, (char) 174, (char) 153, (char) 143, (char) 63, (char) 51, (char) 0, (char) 84, (char) 221, (char) 117, (char) 88, (char) 58, (char) 40, (char) 74, (char) 201, (char) 194, (char) 220, (char) 35, (char) 183, (char) 218, (char) 240, (char) 114, (char) 247, (char) 245, (char) 195, (char) 15, (char) 105, (char) 43, (char) 137, (char) 40, (char) 255, (char) 207, (char) 208, (char) 197, (char) 22, (char) 13, (char) 22, (char) 201, (char) 30, (char) 143, (char) 147, (char) 179, (char) 149, (char) 135, (char) 19, (char) 12, (char) 243, (char) 83, (char) 112, (char) 145, (char) 190, (char) 193, (char) 213, (char) 47, (char) 200, (char) 206, (char) 117, (char) 182, (char) 114, (char) 236, (char) 0, (char) 184, (char) 18, (char) 55, (char) 66, (char) 107, (char) 24, (char) 196, (char) 135, (char) 237, (char) 122, (char) 96, (char) 200, (char) 221, (char) 247, (char) 202, (char) 227, (char) 101, (char) 54, (char) 117, (char) 139, (char) 107, (char) 66, (char) 95, (char) 128, (char) 161, (char) 213, (char) 112, (char) 6, (char) 129, (char) 12, (char) 101, (char) 32, (char) 13, (char) 124, (char) 199, (char) 82, (char) 199, (char) 149, (char) 46, (char) 117, (char) 112, (char) 102, (char) 248, (char) 122, (char) 100, (char) 11, (char) 196, (char) 122, (char) 238, (char) 89, (char) 210, (char) 69, (char) 13, (char) 18, (char) 199, (char) 16, (char) 117, (char) 239, (char) 118, (char) 188, (char) 190, (char) 69, (char) 80, (char) 226, (char) 41, (char) 213, (char) 65, (char) 179, (char) 89, (char) 18, (char) 76, (char) 22, (char) 215, (char) 253, (char) 252, (char) 4, (char) 29, (char) 82, (char) 222, (char) 250, (char) 109, (char) 92, (char) 76, (char) 126, (char) 176, (char) 167, (char) 131, (char) 222, (char) 117, (char) 91, (char) 203, (char) 170, (char) 136, (char) 118, (char) 147, (char) 250, (char) 199, (char) 249, (char) 195, (char) 192, (char) 62, (char) 53, (char) 93, (char) 170, (char) 1, (char) 169, (char) 73, (char) 160, (char) 153, (char) 32, (char) 106, (char) 179, (char) 229, (char) 197}));
		});
		GroundControl.LOGGING_DATA_ACKED p267 = CommunicationChannel.new_LOGGING_DATA_ACKED();
		PH.setPack(p267);
		p267.sequence_SET((char) 8885);
		p267.length_SET((char) 141);
		p267.first_message_offset_SET((char) 110);
		p267.target_component_SET((char) 25);
		p267.data__SET(new char[]{(char) 250, (char) 69, (char) 19, (char) 229, (char) 246, (char) 95, (char) 154, (char) 3, (char) 196, (char) 91, (char) 152, (char) 236, (char) 145, (char) 226, (char) 13, (char) 106, (char) 90, (char) 233, (char) 170, (char) 253, (char) 116, (char) 48, (char) 101, (char) 249, (char) 159, (char) 79, (char) 166, (char) 230, (char) 107, (char) 36, (char) 5, (char) 168, (char) 213, (char) 127, (char) 179, (char) 121, (char) 91, (char) 242, (char) 140, (char) 104, (char) 106, (char) 14, (char) 234, (char) 225, (char) 74, (char) 112, (char) 41, (char) 233, (char) 80, (char) 250, (char) 222, (char) 232, (char) 207, (char) 240, (char) 137, (char) 41, (char) 220, (char) 123, (char) 246, (char) 156, (char) 232, (char) 160, (char) 228, (char) 98, (char) 106, (char) 190, (char) 143, (char) 62, (char) 88, (char) 226, (char) 56, (char) 205, (char) 129, (char) 235, (char) 241, (char) 174, (char) 153, (char) 143, (char) 63, (char) 51, (char) 0, (char) 84, (char) 221, (char) 117, (char) 88, (char) 58, (char) 40, (char) 74, (char) 201, (char) 194, (char) 220, (char) 35, (char) 183, (char) 218, (char) 240, (char) 114, (char) 247, (char) 245, (char) 195, (char) 15, (char) 105, (char) 43, (char) 137, (char) 40, (char) 255, (char) 207, (char) 208, (char) 197, (char) 22, (char) 13, (char) 22, (char) 201, (char) 30, (char) 143, (char) 147, (char) 179, (char) 149, (char) 135, (char) 19, (char) 12, (char) 243, (char) 83, (char) 112, (char) 145, (char) 190, (char) 193, (char) 213, (char) 47, (char) 200, (char) 206, (char) 117, (char) 182, (char) 114, (char) 236, (char) 0, (char) 184, (char) 18, (char) 55, (char) 66, (char) 107, (char) 24, (char) 196, (char) 135, (char) 237, (char) 122, (char) 96, (char) 200, (char) 221, (char) 247, (char) 202, (char) 227, (char) 101, (char) 54, (char) 117, (char) 139, (char) 107, (char) 66, (char) 95, (char) 128, (char) 161, (char) 213, (char) 112, (char) 6, (char) 129, (char) 12, (char) 101, (char) 32, (char) 13, (char) 124, (char) 199, (char) 82, (char) 199, (char) 149, (char) 46, (char) 117, (char) 112, (char) 102, (char) 248, (char) 122, (char) 100, (char) 11, (char) 196, (char) 122, (char) 238, (char) 89, (char) 210, (char) 69, (char) 13, (char) 18, (char) 199, (char) 16, (char) 117, (char) 239, (char) 118, (char) 188, (char) 190, (char) 69, (char) 80, (char) 226, (char) 41, (char) 213, (char) 65, (char) 179, (char) 89, (char) 18, (char) 76, (char) 22, (char) 215, (char) 253, (char) 252, (char) 4, (char) 29, (char) 82, (char) 222, (char) 250, (char) 109, (char) 92, (char) 76, (char) 126, (char) 176, (char) 167, (char) 131, (char) 222, (char) 117, (char) 91, (char) 203, (char) 170, (char) 136, (char) 118, (char) 147, (char) 250, (char) 199, (char) 249, (char) 195, (char) 192, (char) 62, (char) 53, (char) 93, (char) 170, (char) 1, (char) 169, (char) 73, (char) 160, (char) 153, (char) 32, (char) 106, (char) 179, (char) 229, (char) 197}, 0);
		p267.target_system_SET((char) 210);
		CommunicationChannel.instance.send(p267);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_LOGGING_ACK.add((src, ph, pack) ->
		{
			assert (pack.target_system_GET() == (char) 154);
			assert (pack.sequence_GET() == (char) 2818);
			assert (pack.target_component_GET() == (char) 45);
		});
		GroundControl.LOGGING_ACK p268 = CommunicationChannel.new_LOGGING_ACK();
		PH.setPack(p268);
		p268.target_component_SET((char) 45);
		p268.target_system_SET((char) 154);
		p268.sequence_SET((char) 2818);
		CommunicationChannel.instance.send(p268);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_VIDEO_STREAM_INFORMATION.add((src, ph, pack) ->
		{
			assert (pack.framerate_GET() == 2.1222872E38F);
			assert (pack.bitrate_GET() == 2499430980L);
			assert (pack.uri_LEN(ph) == 98);
			assert (pack.uri_TRY(ph).equals("ghflambugivqnbikpuqynvocrwgzXwzmUpearyxslVhfvesipbowgshiuFmrTkafsTjasocxyxhpFpsinYxWdsafnesfphyfJU"));
			assert (pack.rotation_GET() == (char) 51594);
			assert (pack.resolution_v_GET() == (char) 43379);
			assert (pack.status_GET() == (char) 57);
			assert (pack.resolution_h_GET() == (char) 10049);
			assert (pack.camera_id_GET() == (char) 67);
		});
		GroundControl.VIDEO_STREAM_INFORMATION p269 = CommunicationChannel.new_VIDEO_STREAM_INFORMATION();
		PH.setPack(p269);
		p269.resolution_h_SET((char) 10049);
		p269.camera_id_SET((char) 67);
		p269.bitrate_SET(2499430980L);
		p269.rotation_SET((char) 51594);
		p269.status_SET((char) 57);
		p269.uri_SET("ghflambugivqnbikpuqynvocrwgzXwzmUpearyxslVhfvesipbowgshiuFmrTkafsTjasocxyxhpFpsinYxWdsafnesfphyfJU", PH);
		p269.framerate_SET(2.1222872E38F);
		p269.resolution_v_SET((char) 43379);
		CommunicationChannel.instance.send(p269);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_SET_VIDEO_STREAM_SETTINGS.add((src, ph, pack) ->
		{
			assert (pack.target_system_GET() == (char) 21);
			assert (pack.camera_id_GET() == (char) 115);
			assert (pack.rotation_GET() == (char) 6186);
			assert (pack.uri_LEN(ph) == 14);
			assert (pack.uri_TRY(ph).equals("uzveoInzjTyezb"));
			assert (pack.bitrate_GET() == 2660491949L);
			assert (pack.target_component_GET() == (char) 27);
			assert (pack.resolution_h_GET() == (char) 10915);
			assert (pack.resolution_v_GET() == (char) 2967);
			assert (pack.framerate_GET() == 3.2881879E38F);
		});
		GroundControl.SET_VIDEO_STREAM_SETTINGS p270 = CommunicationChannel.new_SET_VIDEO_STREAM_SETTINGS();
		PH.setPack(p270);
		p270.resolution_v_SET((char) 2967);
		p270.target_component_SET((char) 27);
		p270.camera_id_SET((char) 115);
		p270.framerate_SET(3.2881879E38F);
		p270.resolution_h_SET((char) 10915);
		p270.rotation_SET((char) 6186);
		p270.target_system_SET((char) 21);
		p270.bitrate_SET(2660491949L);
		p270.uri_SET("uzveoInzjTyezb", PH);
		CommunicationChannel.instance.send(p270);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_WIFI_CONFIG_AP.add((src, ph, pack) ->
		{
			assert (pack.ssid_LEN(ph) == 5);
			assert (pack.ssid_TRY(ph).equals("jseas"));
			assert (pack.password_LEN(ph) == 48);
			assert (pack.password_TRY(ph).equals("sibrkcGotcgwIbCtcputiJWawrlHhcuzsOgyqjxtmnqbrgiR"));
		});
		GroundControl.WIFI_CONFIG_AP p299 = CommunicationChannel.new_WIFI_CONFIG_AP();
		PH.setPack(p299);
		p299.ssid_SET("jseas", PH);
		p299.password_SET("sibrkcGotcgwIbCtcputiJWawrlHhcuzsOgyqjxtmnqbrgiR", PH);
		CommunicationChannel.instance.send(p299);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_PROTOCOL_VERSION.add((src, ph, pack) ->
		{
			assert (Arrays.equals(pack.spec_version_hash_GET(), new char[]{(char) 65, (char) 36, (char) 216, (char) 232, (char) 134, (char) 72, (char) 85, (char) 207}));
			assert (pack.max_version_GET() == (char) 44080);
			assert (pack.version_GET() == (char) 45294);
			assert (Arrays.equals(pack.library_version_hash_GET(), new char[]{(char) 126, (char) 197, (char) 43, (char) 23, (char) 190, (char) 30, (char) 29, (char) 84}));
			assert (pack.min_version_GET() == (char) 52102);
		});
		GroundControl.PROTOCOL_VERSION p300 = CommunicationChannel.new_PROTOCOL_VERSION();
		PH.setPack(p300);
		p300.max_version_SET((char) 44080);
		p300.min_version_SET((char) 52102);
		p300.spec_version_hash_SET(new char[]{(char) 65, (char) 36, (char) 216, (char) 232, (char) 134, (char) 72, (char) 85, (char) 207}, 0);
		p300.library_version_hash_SET(new char[]{(char) 126, (char) 197, (char) 43, (char) 23, (char) 190, (char) 30, (char) 29, (char) 84}, 0);
		p300.version_SET((char) 45294);
		CommunicationChannel.instance.send(p300);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_UAVCAN_NODE_STATUS.add((src, ph, pack) ->
		{
			assert (pack.mode_GET() == UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_INITIALIZATION);
			assert (pack.vendor_specific_status_code_GET() == (char) 59576);
			assert (pack.health_GET() == UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_OK);
			assert (pack.uptime_sec_GET() == 2700950450L);
			assert (pack.time_usec_GET() == 7507049814751134644L);
			assert (pack.sub_mode_GET() == (char) 221);
		});
		GroundControl.UAVCAN_NODE_STATUS p310 = CommunicationChannel.new_UAVCAN_NODE_STATUS();
		PH.setPack(p310);
		p310.vendor_specific_status_code_SET((char) 59576);
		p310.mode_SET(UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_INITIALIZATION);
		p310.health_SET(UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_OK);
		p310.time_usec_SET(7507049814751134644L);
		p310.uptime_sec_SET(2700950450L);
		p310.sub_mode_SET((char) 221);
		CommunicationChannel.instance.send(p310);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_UAVCAN_NODE_INFO.add((src, ph, pack) ->
		{
			assert (pack.uptime_sec_GET() == 483444684L);
			assert (pack.name_LEN(ph) == 79);
			assert (pack.name_TRY(ph).equals("hdSpuzrizwofhwmIvigcdtMisawZucpHgsquaarxisfUbzokoobjmjjxGhizsZZcylnvqstsZwrtMpn"));
			assert (pack.hw_version_minor_GET() == (char) 105);
			assert (pack.hw_version_major_GET() == (char) 4);
			assert (pack.sw_vcs_commit_GET() == 445721638L);
			assert (Arrays.equals(pack.hw_unique_id_GET(), new char[]{(char) 29, (char) 96, (char) 204, (char) 71, (char) 180, (char) 183, (char) 229, (char) 8, (char) 240, (char) 231, (char) 42, (char) 1, (char) 99, (char) 56, (char) 145, (char) 171}));
			assert (pack.sw_version_major_GET() == (char) 234);
			assert (pack.sw_version_minor_GET() == (char) 69);
			assert (pack.time_usec_GET() == 7898078084762483480L);
		});
		GroundControl.UAVCAN_NODE_INFO p311 = CommunicationChannel.new_UAVCAN_NODE_INFO();
		PH.setPack(p311);
		p311.time_usec_SET(7898078084762483480L);
		p311.name_SET("hdSpuzrizwofhwmIvigcdtMisawZucpHgsquaarxisfUbzokoobjmjjxGhizsZZcylnvqstsZwrtMpn", PH);
		p311.sw_vcs_commit_SET(445721638L);
		p311.uptime_sec_SET(483444684L);
		p311.sw_version_minor_SET((char) 69);
		p311.hw_version_major_SET((char) 4);
		p311.hw_version_minor_SET((char) 105);
		p311.hw_unique_id_SET(new char[]{(char) 29, (char) 96, (char) 204, (char) 71, (char) 180, (char) 183, (char) 229, (char) 8, (char) 240, (char) 231, (char) 42, (char) 1, (char) 99, (char) 56, (char) 145, (char) 171}, 0);
		p311.sw_version_major_SET((char) 234);
		CommunicationChannel.instance.send(p311);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_PARAM_EXT_REQUEST_READ.add((src, ph, pack) ->
		{
			assert (pack.param_id_LEN(ph) == 1);
			assert (pack.param_id_TRY(ph).equals("h"));
			assert (pack.target_system_GET() == (char) 253);
			assert (pack.target_component_GET() == (char) 193);
			assert (pack.param_index_GET() == (short) -11849);
		});
		GroundControl.PARAM_EXT_REQUEST_READ p320 = CommunicationChannel.new_PARAM_EXT_REQUEST_READ();
		PH.setPack(p320);
		p320.target_component_SET((char) 193);
		p320.param_index_SET((short) -11849);
		p320.param_id_SET("h", PH);
		p320.target_system_SET((char) 253);
		CommunicationChannel.instance.send(p320);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_PARAM_EXT_REQUEST_LIST.add((src, ph, pack) ->
		{
			assert (pack.target_component_GET() == (char) 130);
			assert (pack.target_system_GET() == (char) 70);
		});
		GroundControl.PARAM_EXT_REQUEST_LIST p321 = CommunicationChannel.new_PARAM_EXT_REQUEST_LIST();
		PH.setPack(p321);
		p321.target_component_SET((char) 130);
		p321.target_system_SET((char) 70);
		CommunicationChannel.instance.send(p321);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_PARAM_EXT_VALUE.add((src, ph, pack) ->
		{
			assert (pack.param_count_GET() == (char) 11028);
			assert (pack.param_id_LEN(ph) == 4);
			assert (pack.param_id_TRY(ph).equals("rkXx"));
			assert (pack.param_index_GET() == (char) 46283);
			assert (pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT32);
			assert (pack.param_value_LEN(ph) == 54);
			assert (pack.param_value_TRY(ph).equals("dknlwyeybxpthiqdnvhmxUtwlplheoxqxshwdpcLqqvfqfsyTyobwf"));
		});
		GroundControl.PARAM_EXT_VALUE p322 = CommunicationChannel.new_PARAM_EXT_VALUE();
		PH.setPack(p322);
		p322.param_id_SET("rkXx", PH);
		p322.param_index_SET((char) 46283);
		p322.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT32);
		p322.param_value_SET("dknlwyeybxpthiqdnvhmxUtwlplheoxqxshwdpcLqqvfqfsyTyobwf", PH);
		p322.param_count_SET((char) 11028);
		CommunicationChannel.instance.send(p322);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_PARAM_EXT_SET.add((src, ph, pack) ->
		{
			assert (pack.target_system_GET() == (char) 128);
			assert (pack.param_id_LEN(ph) == 8);
			assert (pack.param_id_TRY(ph).equals("yPamrbkp"));
			assert (pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT32);
			assert (pack.target_component_GET() == (char) 87);
			assert (pack.param_value_LEN(ph) == 103);
			assert (pack.param_value_TRY(ph).equals("sovgpxwvmfboqUfzskhdoeyelFMynnjprapthpqNjphgtfmaowzsppbokhkcgbgtpxqotmrrkvcmBlMcrlcwtrlzrwjptlvzfswKlbz"));
		});
		GroundControl.PARAM_EXT_SET p323 = CommunicationChannel.new_PARAM_EXT_SET();
		PH.setPack(p323);
		p323.param_value_SET("sovgpxwvmfboqUfzskhdoeyelFMynnjprapthpqNjphgtfmaowzsppbokhkcgbgtpxqotmrrkvcmBlMcrlcwtrlzrwjptlvzfswKlbz", PH);
		p323.target_component_SET((char) 87);
		p323.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT32);
		p323.param_id_SET("yPamrbkp", PH);
		p323.target_system_SET((char) 128);
		CommunicationChannel.instance.send(p323);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_PARAM_EXT_ACK.add((src, ph, pack) ->
		{
			assert (pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT8);
			assert (pack.param_result_GET() == PARAM_ACK.PARAM_ACK_FAILED);
			assert (pack.param_value_LEN(ph) == 34);
			assert (pack.param_value_TRY(ph).equals("vcImtprzsxlfvLlfkclxvczdfubNxcjiwh"));
			assert (pack.param_id_LEN(ph) == 10);
			assert (pack.param_id_TRY(ph).equals("zirdJozrQl"));
		});
		GroundControl.PARAM_EXT_ACK p324 = CommunicationChannel.new_PARAM_EXT_ACK();
		PH.setPack(p324);
		p324.param_id_SET("zirdJozrQl", PH);
		p324.param_value_SET("vcImtprzsxlfvLlfkclxvczdfubNxcjiwh", PH);
		p324.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT8);
		p324.param_result_SET(PARAM_ACK.PARAM_ACK_FAILED);
		CommunicationChannel.instance.send(p324);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_OBSTACLE_DISTANCE.add((src, ph, pack) ->
		{
			assert (pack.sensor_type_GET() == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_ULTRASOUND);
			assert (pack.increment_GET() == (char) 222);
			assert (pack.time_usec_GET() == 3288992859079273561L);
			assert (Arrays.equals(pack.distances_GET(), new char[]{(char) 29121, (char) 65345, (char) 21588, (char) 38629, (char) 53395, (char) 48584, (char) 36223, (char) 21998, (char) 39849, (char) 21719, (char) 11213, (char) 26212, (char) 60943, (char) 51998, (char) 29059, (char) 13564, (char) 5756, (char) 39492, (char) 31827, (char) 34385, (char) 12939, (char) 39655, (char) 52808, (char) 32426, (char) 26406, (char) 39785, (char) 416, (char) 4532, (char) 46213, (char) 11869, (char) 31456, (char) 43732, (char) 51513, (char) 26061, (char) 31061, (char) 33008, (char) 55710, (char) 14512, (char) 36098, (char) 31882, (char) 410, (char) 38244, (char) 22746, (char) 2929, (char) 34537, (char) 30306, (char) 57089, (char) 10850, (char) 57199, (char) 61640, (char) 47602, (char) 34894, (char) 54237, (char) 51188, (char) 36786, (char) 18312, (char) 63581, (char) 3112, (char) 29962, (char) 55872, (char) 62726, (char) 8975, (char) 55120, (char) 20051, (char) 23943, (char) 40938, (char) 57287, (char) 11355, (char) 7976, (char) 17586, (char) 9602, (char) 1029}));
			assert (pack.max_distance_GET() == (char) 43686);
			assert (pack.min_distance_GET() == (char) 16535);
		});
		GroundControl.OBSTACLE_DISTANCE p330 = CommunicationChannel.new_OBSTACLE_DISTANCE();
		PH.setPack(p330);
		p330.increment_SET((char) 222);
		p330.min_distance_SET((char) 16535);
		p330.time_usec_SET(3288992859079273561L);
		p330.sensor_type_SET(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_ULTRASOUND);
		p330.distances_SET(new char[]{(char) 29121, (char) 65345, (char) 21588, (char) 38629, (char) 53395, (char) 48584, (char) 36223, (char) 21998, (char) 39849, (char) 21719, (char) 11213, (char) 26212, (char) 60943, (char) 51998, (char) 29059, (char) 13564, (char) 5756, (char) 39492, (char) 31827, (char) 34385, (char) 12939, (char) 39655, (char) 52808, (char) 32426, (char) 26406, (char) 39785, (char) 416, (char) 4532, (char) 46213, (char) 11869, (char) 31456, (char) 43732, (char) 51513, (char) 26061, (char) 31061, (char) 33008, (char) 55710, (char) 14512, (char) 36098, (char) 31882, (char) 410, (char) 38244, (char) 22746, (char) 2929, (char) 34537, (char) 30306, (char) 57089, (char) 10850, (char) 57199, (char) 61640, (char) 47602, (char) 34894, (char) 54237, (char) 51188, (char) 36786, (char) 18312, (char) 63581, (char) 3112, (char) 29962, (char) 55872, (char) 62726, (char) 8975, (char) 55120, (char) 20051, (char) 23943, (char) 40938, (char) 57287, (char) 11355, (char) 7976, (char) 17586, (char) 9602, (char) 1029}, 0);
		p330.max_distance_SET((char) 43686);
		CommunicationChannel.instance.send(p330);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
	}

}