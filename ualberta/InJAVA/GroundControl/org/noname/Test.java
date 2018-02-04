
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

	public static class NAV_FILTER_BIAS extends GroundControl.NAV_FILTER_BIAS {
		public long usec_GET()//Timestamp (microseconds)
		{ return (get_bytes(data, 0, 8)); }

		public float accel_0_GET()//b_f[0]
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 8, 4))); }

		public float accel_1_GET()//b_f[1]
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 12, 4))); }

		public float accel_2_GET()//b_f[2]
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 16, 4))); }

		public float gyro_0_GET()//b_f[0]
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 20, 4))); }

		public float gyro_1_GET()//b_f[1]
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 24, 4))); }

		public float gyro_2_GET()//b_f[2]
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 28, 4))); }
	}

	public static class RADIO_CALIBRATION extends GroundControl.RADIO_CALIBRATION {
		public char[] aileron_GET(char[] dst_ch, int pos)  //Aileron setpoints: left, center, right
		{
			for (int BYTE = 0, dst_max = pos + 3; pos < dst_max; pos++, BYTE += 2)
				dst_ch[pos] = (char) ((char) get_bytes(data, BYTE, 2));
			return dst_ch;
		}

		public char[] aileron_GET()//Aileron setpoints: left, center, right
		{return aileron_GET(new char[3], 0);}

		public char[] elevator_GET(char[] dst_ch, int pos)  //Elevator setpoints: nose down, center, nose up
		{
			for (int BYTE = 6, dst_max = pos + 3; pos < dst_max; pos++, BYTE += 2)
				dst_ch[pos] = (char) ((char) get_bytes(data, BYTE, 2));
			return dst_ch;
		}

		public char[] elevator_GET()//Elevator setpoints: nose down, center, nose up
		{return elevator_GET(new char[3], 0);}

		public char[] rudder_GET(char[] dst_ch, int pos)  //Rudder setpoints: nose left, center, nose right
		{
			for (int BYTE = 12, dst_max = pos + 3; pos < dst_max; pos++, BYTE += 2)
				dst_ch[pos] = (char) ((char) get_bytes(data, BYTE, 2));
			return dst_ch;
		}

		public char[] rudder_GET()//Rudder setpoints: nose left, center, nose right
		{return rudder_GET(new char[3], 0);}

		public char[] gyro_GET(char[] dst_ch, int pos)  //Tail gyro mode/gain setpoints: heading hold, rate mode
		{
			for (int BYTE = 18, dst_max = pos + 2; pos < dst_max; pos++, BYTE += 2)
				dst_ch[pos] = (char) ((char) get_bytes(data, BYTE, 2));
			return dst_ch;
		}

		public char[] gyro_GET()//Tail gyro mode/gain setpoints: heading hold, rate mode
		{return gyro_GET(new char[2], 0);}

		public char[] pitch_GET(char[] dst_ch, int pos)  //Pitch curve setpoints (every 25%)
		{
			for (int BYTE = 22, dst_max = pos + 5; pos < dst_max; pos++, BYTE += 2)
				dst_ch[pos] = (char) ((char) get_bytes(data, BYTE, 2));
			return dst_ch;
		}

		public char[] pitch_GET()//Pitch curve setpoints (every 25%)
		{return pitch_GET(new char[5], 0);}

		public char[] throttle_GET(char[] dst_ch, int pos)  //Throttle curve setpoints (every 25%)
		{
			for (int BYTE = 32, dst_max = pos + 5; pos < dst_max; pos++, BYTE += 2)
				dst_ch[pos] = (char) ((char) get_bytes(data, BYTE, 2));
			return dst_ch;
		}

		public char[] throttle_GET()//Throttle curve setpoints (every 25%)
		{return throttle_GET(new char[5], 0);}
	}

	public static class UALBERTA_SYS_STATUS extends GroundControl.UALBERTA_SYS_STATUS {
		public char mode_GET()//System mode, see UALBERTA_AUTOPILOT_MODE ENUM
		{ return (char) ((char) get_bytes(data, 0, 1)); }

		public char nav_mode_GET()//Navigation mode, see UALBERTA_NAV_MODE ENUM
		{ return (char) ((char) get_bytes(data, 1, 1)); }

		public char pilot_GET()//Pilot mode, see UALBERTA_PILOT_MODE
		{ return (char) ((char) get_bytes(data, 2, 1)); }
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

		static final Collection<OnReceive.Handler<ACTUATOR_CONTROL_TARGET, Channel>>   on_ACTUATOR_CONTROL_TARGET   = new OnReceive<>();
		static final Collection<OnReceive.Handler<ALTITUDE, Channel>>                  on_ALTITUDE                  = new OnReceive<>();
		static final Collection<OnReceive.Handler<RESOURCE_REQUEST, Channel>>          on_RESOURCE_REQUEST          = new OnReceive<>();
		static final Collection<OnReceive.Handler<SCALED_PRESSURE3, Channel>>          on_SCALED_PRESSURE3          = new OnReceive<>();
		static final Collection<OnReceive.Handler<FOLLOW_TARGET, Channel>>             on_FOLLOW_TARGET             = new OnReceive<>();
		static final Collection<OnReceive.Handler<CONTROL_SYSTEM_STATE, Channel>>      on_CONTROL_SYSTEM_STATE      = new OnReceive<>();
		static final Collection<OnReceive.Handler<BATTERY_STATUS, Channel>>            on_BATTERY_STATUS            = new OnReceive<>();
		static final Collection<OnReceive.Handler<AUTOPILOT_VERSION, Channel>>         on_AUTOPILOT_VERSION         = new OnReceive<>();
		static final Collection<OnReceive.Handler<LANDING_TARGET, Channel>>            on_LANDING_TARGET            = new OnReceive<>();
		static final Collection<OnReceive.Handler<NAV_FILTER_BIAS, Channel>>           on_NAV_FILTER_BIAS           = new OnReceive<>();
		static final Collection<OnReceive.Handler<RADIO_CALIBRATION, Channel>>         on_RADIO_CALIBRATION         = new OnReceive<>();
		static final Collection<OnReceive.Handler<UALBERTA_SYS_STATUS, Channel>>       on_UALBERTA_SYS_STATUS       = new OnReceive<>();
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
				case 220:
					if (pack == null) return new NAV_FILTER_BIAS();
					((OnReceive) on_NAV_FILTER_BIAS).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
					break;
				case 221:
					if (pack == null) return new RADIO_CALIBRATION();
					((OnReceive) on_RADIO_CALIBRATION).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
					break;
				case 222:
					if (pack == null) return new UALBERTA_SYS_STATUS();
					((OnReceive) on_UALBERTA_SYS_STATUS).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
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
			assert (pack.autopilot_GET() == MAV_AUTOPILOT.MAV_AUTOPILOT_AEROB);
			assert (pack.system_status_GET() == MAV_STATE.MAV_STATE_CALIBRATING);
			assert (pack.custom_mode_GET() == 2887298199L);
			assert (pack.type_GET() == MAV_TYPE.MAV_TYPE_ROCKET);
			assert (pack.base_mode_GET() == MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED);
			assert (pack.mavlink_version_GET() == (char) 0);
		});
		HEARTBEAT p0 = new HEARTBEAT();
		PH.setPack(p0);
		p0.autopilot_SET(MAV_AUTOPILOT.MAV_AUTOPILOT_AEROB);
		p0.base_mode_SET(MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED);
		p0.system_status_SET(MAV_STATE.MAV_STATE_CALIBRATING);
		p0.type_SET(MAV_TYPE.MAV_TYPE_ROCKET);
		p0.custom_mode_SET(2887298199L);
		p0.mavlink_version_SET((char) 0);
		TestChannel.instance.send(p0);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_SYS_STATUS.add((src, ph, pack) ->
		{
			assert (pack.onboard_control_sensors_health_GET() == MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG2);
			assert (pack.onboard_control_sensors_present_GET() == MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN);
			assert (pack.errors_count3_GET() == (char) 61549);
			assert (pack.drop_rate_comm_GET() == (char) 23510);
			assert (pack.load_GET() == (char) 35317);
			assert (pack.battery_remaining_GET() == (byte) -83);
			assert (pack.current_battery_GET() == (short) 13993);
			assert (pack.errors_count1_GET() == (char) 38132);
			assert (pack.voltage_battery_GET() == (char) 37980);
			assert (pack.errors_comm_GET() == (char) 28818);
			assert (pack.errors_count2_GET() == (char) 62024);
			assert (pack.onboard_control_sensors_enabled_GET() == MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_BATTERY);
			assert (pack.errors_count4_GET() == (char) 59836);
		});
		SYS_STATUS p1 = new SYS_STATUS();
		PH.setPack(p1);
		p1.current_battery_SET((short) 13993);
		p1.onboard_control_sensors_present_SET(MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN);
		p1.onboard_control_sensors_health_SET(MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG2);
		p1.errors_count1_SET((char) 38132);
		p1.drop_rate_comm_SET((char) 23510);
		p1.battery_remaining_SET((byte) -83);
		p1.errors_count4_SET((char) 59836);
		p1.voltage_battery_SET((char) 37980);
		p1.load_SET((char) 35317);
		p1.onboard_control_sensors_enabled_SET(MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_BATTERY);
		p1.errors_count2_SET((char) 62024);
		p1.errors_count3_SET((char) 61549);
		p1.errors_comm_SET((char) 28818);
		TestChannel.instance.send(p1);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_SYSTEM_TIME.add((src, ph, pack) ->
		{
			assert (pack.time_unix_usec_GET() == 2676367146303931689L);
			assert (pack.time_boot_ms_GET() == 2064591736L);
		});
		SYSTEM_TIME p2 = new SYSTEM_TIME();
		PH.setPack(p2);
		p2.time_boot_ms_SET(2064591736L);
		p2.time_unix_usec_SET(2676367146303931689L);
		TestChannel.instance.send(p2);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_POSITION_TARGET_LOCAL_NED.add((src, ph, pack) ->
		{
			assert (pack.y_GET() == -2.9698003E38F);
			assert (pack.x_GET() == 5.334586E37F);
			assert (pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT);
			assert (pack.vz_GET() == 3.0588936E38F);
			assert (pack.afx_GET() == -2.3125604E38F);
			assert (pack.afy_GET() == 3.1510612E38F);
			assert (pack.type_mask_GET() == (char) 65085);
			assert (pack.vx_GET() == 3.1126516E38F);
			assert (pack.yaw_rate_GET() == -3.1362994E38F);
			assert (pack.time_boot_ms_GET() == 2827799764L);
			assert (pack.yaw_GET() == -2.9453322E38F);
			assert (pack.afz_GET() == -9.11564E37F);
			assert (pack.z_GET() == 3.3371062E38F);
			assert (pack.vy_GET() == 8.32967E37F);
		});
		GroundControl.POSITION_TARGET_LOCAL_NED p3 = CommunicationChannel.new_POSITION_TARGET_LOCAL_NED();
		PH.setPack(p3);
		p3.y_SET(-2.9698003E38F);
		p3.yaw_SET(-2.9453322E38F);
		p3.afy_SET(3.1510612E38F);
		p3.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT);
		p3.afz_SET(-9.11564E37F);
		p3.z_SET(3.3371062E38F);
		p3.afx_SET(-2.3125604E38F);
		p3.x_SET(5.334586E37F);
		p3.yaw_rate_SET(-3.1362994E38F);
		p3.vx_SET(3.1126516E38F);
		p3.time_boot_ms_SET(2827799764L);
		p3.type_mask_SET((char) 65085);
		p3.vz_SET(3.0588936E38F);
		p3.vy_SET(8.32967E37F);
		CommunicationChannel.instance.send(p3);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_PING.add((src, ph, pack) ->
		{
			assert (pack.target_system_GET() == (char) 212);
			assert (pack.seq_GET() == 2663315598L);
			assert (pack.target_component_GET() == (char) 13);
			assert (pack.time_usec_GET() == 3313939293352897631L);
		});
		PING p4 = new PING();
		PH.setPack(p4);
		p4.time_usec_SET(3313939293352897631L);
		p4.target_system_SET((char) 212);
		p4.target_component_SET((char) 13);
		p4.seq_SET(2663315598L);
		TestChannel.instance.send(p4);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_CHANGE_OPERATOR_CONTROL.add((src, ph, pack) ->
		{
			assert (pack.target_system_GET() == (char) 111);
			assert (pack.passkey_LEN(ph) == 19);
			assert (pack.passkey_TRY(ph).equals("vfkokCcpkifoaqyirjz"));
			assert (pack.version_GET() == (char) 178);
			assert (pack.control_request_GET() == (char) 165);
		});
		CHANGE_OPERATOR_CONTROL p5 = new CHANGE_OPERATOR_CONTROL();
		PH.setPack(p5);
		p5.version_SET((char) 178);
		p5.control_request_SET((char) 165);
		p5.passkey_SET("vfkokCcpkifoaqyirjz", PH);
		p5.target_system_SET((char) 111);
		TestChannel.instance.send(p5);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_CHANGE_OPERATOR_CONTROL_ACK.add((src, ph, pack) ->
		{
			assert (pack.control_request_GET() == (char) 182);
			assert (pack.ack_GET() == (char) 230);
			assert (pack.gcs_system_id_GET() == (char) 151);
		});
		CHANGE_OPERATOR_CONTROL_ACK p6 = new CHANGE_OPERATOR_CONTROL_ACK();
		PH.setPack(p6);
		p6.gcs_system_id_SET((char) 151);
		p6.ack_SET((char) 230);
		p6.control_request_SET((char) 182);
		TestChannel.instance.send(p6);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_AUTH_KEY.add((src, ph, pack) ->
		{
			assert (pack.key_LEN(ph) == 7);
			assert (pack.key_TRY(ph).equals("dovpvbA"));
		});
		AUTH_KEY p7 = new AUTH_KEY();
		PH.setPack(p7);
		p7.key_SET("dovpvbA", PH);
		TestChannel.instance.send(p7);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_SET_MODE.add((src, ph, pack) ->
		{
			assert (pack.target_system_GET() == (char) 9);
			assert (pack.custom_mode_GET() == 1326711784L);
			assert (pack.base_mode_GET() == MAV_MODE.MAV_MODE_GUIDED_ARMED);
		});
		SET_MODE p11 = new SET_MODE();
		PH.setPack(p11);
		p11.base_mode_SET(MAV_MODE.MAV_MODE_GUIDED_ARMED);
		p11.target_system_SET((char) 9);
		p11.custom_mode_SET(1326711784L);
		TestChannel.instance.send(p11);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_PARAM_REQUEST_READ.add((src, ph, pack) ->
		{
			assert (pack.param_id_LEN(ph) == 9);
			assert (pack.param_id_TRY(ph).equals("nbsIlejij"));
			assert (pack.target_system_GET() == (char) 239);
			assert (pack.param_index_GET() == (short) -17891);
			assert (pack.target_component_GET() == (char) 40);
		});
		PARAM_REQUEST_READ p20 = new PARAM_REQUEST_READ();
		PH.setPack(p20);
		p20.param_index_SET((short) -17891);
		p20.param_id_SET("nbsIlejij", PH);
		p20.target_component_SET((char) 40);
		p20.target_system_SET((char) 239);
		TestChannel.instance.send(p20);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_PARAM_REQUEST_LIST.add((src, ph, pack) ->
		{
			assert (pack.target_system_GET() == (char) 134);
			assert (pack.target_component_GET() == (char) 210);
		});
		PARAM_REQUEST_LIST p21 = new PARAM_REQUEST_LIST();
		PH.setPack(p21);
		p21.target_component_SET((char) 210);
		p21.target_system_SET((char) 134);
		TestChannel.instance.send(p21);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_PARAM_VALUE.add((src, ph, pack) ->
		{
			assert (pack.param_count_GET() == (char) 36002);
			assert (pack.param_index_GET() == (char) 20236);
			assert (pack.param_value_GET() == 7.306939E37F);
			assert (pack.param_id_LEN(ph) == 6);
			assert (pack.param_id_TRY(ph).equals("gmqpvg"));
			assert (pack.param_type_GET() == MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT16);
		});
		PARAM_VALUE p22 = new PARAM_VALUE();
		PH.setPack(p22);
		p22.param_type_SET(MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT16);
		p22.param_id_SET("gmqpvg", PH);
		p22.param_count_SET((char) 36002);
		p22.param_value_SET(7.306939E37F);
		p22.param_index_SET((char) 20236);
		TestChannel.instance.send(p22);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_PARAM_SET.add((src, ph, pack) ->
		{
			assert (pack.param_id_LEN(ph) == 6);
			assert (pack.param_id_TRY(ph).equals("ltbjsh"));
			assert (pack.target_system_GET() == (char) 33);
			assert (pack.param_type_GET() == MAV_PARAM_TYPE.MAV_PARAM_TYPE_REAL32);
			assert (pack.target_component_GET() == (char) 144);
			assert (pack.param_value_GET() == -3.303752E38F);
		});
		PARAM_SET p23 = new PARAM_SET();
		PH.setPack(p23);
		p23.param_value_SET(-3.303752E38F);
		p23.target_system_SET((char) 33);
		p23.param_id_SET("ltbjsh", PH);
		p23.param_type_SET(MAV_PARAM_TYPE.MAV_PARAM_TYPE_REAL32);
		p23.target_component_SET((char) 144);
		TestChannel.instance.send(p23);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_GPS_RAW_INT.add((src, ph, pack) ->
		{
			assert (pack.h_acc_TRY(ph) == 272557298L);
			assert (pack.lon_GET() == -1705385591);
			assert (pack.alt_ellipsoid_TRY(ph) == 1492092979);
			assert (pack.eph_GET() == (char) 52874);
			assert (pack.time_usec_GET() == 8498132268426611712L);
			assert (pack.vel_acc_TRY(ph) == 3420898156L);
			assert (pack.alt_GET() == 1690492964);
			assert (pack.hdg_acc_TRY(ph) == 1456796806L);
			assert (pack.vel_GET() == (char) 15883);
			assert (pack.cog_GET() == (char) 52440);
			assert (pack.fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_NO_GPS);
			assert (pack.v_acc_TRY(ph) == 13175682L);
			assert (pack.epv_GET() == (char) 58366);
			assert (pack.satellites_visible_GET() == (char) 181);
			assert (pack.lat_GET() == 1475105035);
		});
		GPS_RAW_INT p24 = new GPS_RAW_INT();
		PH.setPack(p24);
		p24.alt_ellipsoid_SET(1492092979, PH);
		p24.epv_SET((char) 58366);
		p24.h_acc_SET(272557298L, PH);
		p24.lon_SET(-1705385591);
		p24.satellites_visible_SET((char) 181);
		p24.cog_SET((char) 52440);
		p24.time_usec_SET(8498132268426611712L);
		p24.alt_SET(1690492964);
		p24.eph_SET((char) 52874);
		p24.hdg_acc_SET(1456796806L, PH);
		p24.v_acc_SET(13175682L, PH);
		p24.vel_SET((char) 15883);
		p24.fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_NO_GPS);
		p24.lat_SET(1475105035);
		p24.vel_acc_SET(3420898156L, PH);
		TestChannel.instance.send(p24);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_GPS_STATUS.add((src, ph, pack) ->
		{
			assert (Arrays.equals(pack.satellite_prn_GET(), new char[]{(char) 90, (char) 103, (char) 103, (char) 165, (char) 172, (char) 32, (char) 198, (char) 114, (char) 82, (char) 9, (char) 18, (char) 231, (char) 225, (char) 166, (char) 84, (char) 127, (char) 233, (char) 112, (char) 184, (char) 11}));
			assert (Arrays.equals(pack.satellite_azimuth_GET(), new char[]{(char) 51, (char) 114, (char) 72, (char) 204, (char) 73, (char) 42, (char) 103, (char) 86, (char) 119, (char) 40, (char) 202, (char) 194, (char) 74, (char) 181, (char) 120, (char) 161, (char) 16, (char) 46, (char) 211, (char) 163}));
			assert (Arrays.equals(pack.satellite_elevation_GET(), new char[]{(char) 236, (char) 125, (char) 57, (char) 43, (char) 215, (char) 53, (char) 201, (char) 64, (char) 67, (char) 32, (char) 248, (char) 104, (char) 94, (char) 134, (char) 202, (char) 168, (char) 132, (char) 192, (char) 104, (char) 49}));
			assert (Arrays.equals(pack.satellite_snr_GET(), new char[]{(char) 255, (char) 180, (char) 72, (char) 221, (char) 161, (char) 219, (char) 122, (char) 90, (char) 19, (char) 109, (char) 127, (char) 162, (char) 127, (char) 95, (char) 239, (char) 193, (char) 174, (char) 204, (char) 58, (char) 17}));
			assert (Arrays.equals(pack.satellite_used_GET(), new char[]{(char) 33, (char) 232, (char) 7, (char) 254, (char) 35, (char) 139, (char) 102, (char) 186, (char) 234, (char) 74, (char) 59, (char) 247, (char) 118, (char) 1, (char) 6, (char) 17, (char) 53, (char) 245, (char) 145, (char) 117}));
			assert (pack.satellites_visible_GET() == (char) 173);
		});
		GPS_STATUS p25 = new GPS_STATUS();
		PH.setPack(p25);
		p25.satellite_elevation_SET(new char[]{(char) 236, (char) 125, (char) 57, (char) 43, (char) 215, (char) 53, (char) 201, (char) 64, (char) 67, (char) 32, (char) 248, (char) 104, (char) 94, (char) 134, (char) 202, (char) 168, (char) 132, (char) 192, (char) 104, (char) 49}, 0);
		p25.satellite_used_SET(new char[]{(char) 33, (char) 232, (char) 7, (char) 254, (char) 35, (char) 139, (char) 102, (char) 186, (char) 234, (char) 74, (char) 59, (char) 247, (char) 118, (char) 1, (char) 6, (char) 17, (char) 53, (char) 245, (char) 145, (char) 117}, 0);
		p25.satellites_visible_SET((char) 173);
		p25.satellite_azimuth_SET(new char[]{(char) 51, (char) 114, (char) 72, (char) 204, (char) 73, (char) 42, (char) 103, (char) 86, (char) 119, (char) 40, (char) 202, (char) 194, (char) 74, (char) 181, (char) 120, (char) 161, (char) 16, (char) 46, (char) 211, (char) 163}, 0);
		p25.satellite_prn_SET(new char[]{(char) 90, (char) 103, (char) 103, (char) 165, (char) 172, (char) 32, (char) 198, (char) 114, (char) 82, (char) 9, (char) 18, (char) 231, (char) 225, (char) 166, (char) 84, (char) 127, (char) 233, (char) 112, (char) 184, (char) 11}, 0);
		p25.satellite_snr_SET(new char[]{(char) 255, (char) 180, (char) 72, (char) 221, (char) 161, (char) 219, (char) 122, (char) 90, (char) 19, (char) 109, (char) 127, (char) 162, (char) 127, (char) 95, (char) 239, (char) 193, (char) 174, (char) 204, (char) 58, (char) 17}, 0);
		TestChannel.instance.send(p25);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_SCALED_IMU.add((src, ph, pack) ->
		{
			assert (pack.zacc_GET() == (short) 2367);
			assert (pack.yacc_GET() == (short) 27928);
			assert (pack.ygyro_GET() == (short) -624);
			assert (pack.xgyro_GET() == (short) -26656);
			assert (pack.ymag_GET() == (short) 19978);
			assert (pack.zmag_GET() == (short) 1126);
			assert (pack.zgyro_GET() == (short) -11651);
			assert (pack.time_boot_ms_GET() == 2616718539L);
			assert (pack.xmag_GET() == (short) -22343);
			assert (pack.xacc_GET() == (short) -6060);
		});
		SCALED_IMU p26 = new SCALED_IMU();
		PH.setPack(p26);
		p26.zacc_SET((short) 2367);
		p26.yacc_SET((short) 27928);
		p26.xmag_SET((short) -22343);
		p26.xgyro_SET((short) -26656);
		p26.time_boot_ms_SET(2616718539L);
		p26.zgyro_SET((short) -11651);
		p26.zmag_SET((short) 1126);
		p26.ygyro_SET((short) -624);
		p26.xacc_SET((short) -6060);
		p26.ymag_SET((short) 19978);
		TestChannel.instance.send(p26);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_RAW_IMU.add((src, ph, pack) ->
		{
			assert (pack.zmag_GET() == (short) 15651);
			assert (pack.zgyro_GET() == (short) -30180);
			assert (pack.zacc_GET() == (short) -14761);
			assert (pack.ygyro_GET() == (short) 15867);
			assert (pack.xmag_GET() == (short) 4412);
			assert (pack.ymag_GET() == (short) -7878);
			assert (pack.yacc_GET() == (short) 10141);
			assert (pack.time_usec_GET() == 4826181737102063617L);
			assert (pack.xacc_GET() == (short) 10743);
			assert (pack.xgyro_GET() == (short) -6180);
		});
		RAW_IMU p27 = new RAW_IMU();
		PH.setPack(p27);
		p27.xmag_SET((short) 4412);
		p27.zmag_SET((short) 15651);
		p27.time_usec_SET(4826181737102063617L);
		p27.zacc_SET((short) -14761);
		p27.xgyro_SET((short) -6180);
		p27.xacc_SET((short) 10743);
		p27.ygyro_SET((short) 15867);
		p27.ymag_SET((short) -7878);
		p27.zgyro_SET((short) -30180);
		p27.yacc_SET((short) 10141);
		TestChannel.instance.send(p27);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_RAW_PRESSURE.add((src, ph, pack) ->
		{
			assert (pack.press_abs_GET() == (short) -3384);
			assert (pack.temperature_GET() == (short) 11733);
			assert (pack.time_usec_GET() == 7786839076594591333L);
			assert (pack.press_diff2_GET() == (short) -2908);
			assert (pack.press_diff1_GET() == (short) 8327);
		});
		RAW_PRESSURE p28 = new RAW_PRESSURE();
		PH.setPack(p28);
		p28.temperature_SET((short) 11733);
		p28.press_diff1_SET((short) 8327);
		p28.press_diff2_SET((short) -2908);
		p28.press_abs_SET((short) -3384);
		p28.time_usec_SET(7786839076594591333L);
		TestChannel.instance.send(p28);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_SCALED_PRESSURE.add((src, ph, pack) ->
		{
			assert (pack.time_boot_ms_GET() == 2863322315L);
			assert (pack.temperature_GET() == (short) -22862);
			assert (pack.press_abs_GET() == 2.7336733E38F);
			assert (pack.press_diff_GET() == 3.2814225E38F);
		});
		SCALED_PRESSURE p29 = new SCALED_PRESSURE();
		PH.setPack(p29);
		p29.press_diff_SET(3.2814225E38F);
		p29.press_abs_SET(2.7336733E38F);
		p29.time_boot_ms_SET(2863322315L);
		p29.temperature_SET((short) -22862);
		TestChannel.instance.send(p29);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_ATTITUDE.add((src, ph, pack) ->
		{
			assert (pack.pitch_GET() == -3.1774547E38F);
			assert (pack.yawspeed_GET() == -2.1702665E38F);
			assert (pack.yaw_GET() == 6.0815005E37F);
			assert (pack.rollspeed_GET() == 3.0860203E38F);
			assert (pack.roll_GET() == 8.717698E37F);
			assert (pack.time_boot_ms_GET() == 231623042L);
			assert (pack.pitchspeed_GET() == 3.14534E38F);
		});
		ATTITUDE p30 = new ATTITUDE();
		PH.setPack(p30);
		p30.time_boot_ms_SET(231623042L);
		p30.rollspeed_SET(3.0860203E38F);
		p30.pitchspeed_SET(3.14534E38F);
		p30.yaw_SET(6.0815005E37F);
		p30.yawspeed_SET(-2.1702665E38F);
		p30.pitch_SET(-3.1774547E38F);
		p30.roll_SET(8.717698E37F);
		TestChannel.instance.send(p30);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_ATTITUDE_QUATERNION.add((src, ph, pack) ->
		{
			assert (pack.pitchspeed_GET() == -6.1612895E37F);
			assert (pack.q4_GET() == -1.9292291E38F);
			assert (pack.time_boot_ms_GET() == 2755595301L);
			assert (pack.yawspeed_GET() == -2.9678765E38F);
			assert (pack.q1_GET() == -1.876699E38F);
			assert (pack.q3_GET() == -2.8786015E38F);
			assert (pack.q2_GET() == -2.9068595E38F);
			assert (pack.rollspeed_GET() == -2.84536E38F);
		});
		ATTITUDE_QUATERNION p31 = new ATTITUDE_QUATERNION();
		PH.setPack(p31);
		p31.q2_SET(-2.9068595E38F);
		p31.pitchspeed_SET(-6.1612895E37F);
		p31.q1_SET(-1.876699E38F);
		p31.time_boot_ms_SET(2755595301L);
		p31.q4_SET(-1.9292291E38F);
		p31.q3_SET(-2.8786015E38F);
		p31.rollspeed_SET(-2.84536E38F);
		p31.yawspeed_SET(-2.9678765E38F);
		TestChannel.instance.send(p31);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_LOCAL_POSITION_NED.add((src, ph, pack) ->
		{
			assert (pack.vx_GET() == 5.489069E37F);
			assert (pack.vz_GET() == -5.321178E37F);
			assert (pack.vy_GET() == -1.6954754E38F);
			assert (pack.time_boot_ms_GET() == 2910119318L);
			assert (pack.x_GET() == -2.7936662E38F);
			assert (pack.z_GET() == 3.3049156E38F);
			assert (pack.y_GET() == 2.2555033E38F);
		});
		LOCAL_POSITION_NED p32 = new LOCAL_POSITION_NED();
		PH.setPack(p32);
		p32.y_SET(2.2555033E38F);
		p32.vz_SET(-5.321178E37F);
		p32.z_SET(3.3049156E38F);
		p32.vx_SET(5.489069E37F);
		p32.time_boot_ms_SET(2910119318L);
		p32.vy_SET(-1.6954754E38F);
		p32.x_SET(-2.7936662E38F);
		TestChannel.instance.send(p32);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_GLOBAL_POSITION_INT.add((src, ph, pack) ->
		{
			assert (pack.vx_GET() == (short) 540);
			assert (pack.lat_GET() == 1640209639);
			assert (pack.time_boot_ms_GET() == 1448246760L);
			assert (pack.lon_GET() == 874509409);
			assert (pack.vz_GET() == (short) -19197);
			assert (pack.relative_alt_GET() == 1551916322);
			assert (pack.alt_GET() == -608117406);
			assert (pack.vy_GET() == (short) 833);
			assert (pack.hdg_GET() == (char) 63412);
		});
		GLOBAL_POSITION_INT p33 = new GLOBAL_POSITION_INT();
		PH.setPack(p33);
		p33.time_boot_ms_SET(1448246760L);
		p33.lat_SET(1640209639);
		p33.lon_SET(874509409);
		p33.vx_SET((short) 540);
		p33.vy_SET((short) 833);
		p33.vz_SET((short) -19197);
		p33.relative_alt_SET(1551916322);
		p33.alt_SET(-608117406);
		p33.hdg_SET((char) 63412);
		TestChannel.instance.send(p33);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_RC_CHANNELS_SCALED.add((src, ph, pack) ->
		{
			assert (pack.chan4_scaled_GET() == (short) 7831);
			assert (pack.chan6_scaled_GET() == (short) 18688);
			assert (pack.chan5_scaled_GET() == (short) -217);
			assert (pack.chan2_scaled_GET() == (short) 15558);
			assert (pack.chan8_scaled_GET() == (short) 9059);
			assert (pack.chan3_scaled_GET() == (short) -2753);
			assert (pack.chan7_scaled_GET() == (short) 21166);
			assert (pack.time_boot_ms_GET() == 2501988292L);
			assert (pack.chan1_scaled_GET() == (short) -5410);
			assert (pack.port_GET() == (char) 75);
			assert (pack.rssi_GET() == (char) 145);
		});
		RC_CHANNELS_SCALED p34 = new RC_CHANNELS_SCALED();
		PH.setPack(p34);
		p34.chan6_scaled_SET((short) 18688);
		p34.chan8_scaled_SET((short) 9059);
		p34.time_boot_ms_SET(2501988292L);
		p34.port_SET((char) 75);
		p34.chan1_scaled_SET((short) -5410);
		p34.chan5_scaled_SET((short) -217);
		p34.chan2_scaled_SET((short) 15558);
		p34.rssi_SET((char) 145);
		p34.chan7_scaled_SET((short) 21166);
		p34.chan4_scaled_SET((short) 7831);
		p34.chan3_scaled_SET((short) -2753);
		TestChannel.instance.send(p34);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_RC_CHANNELS_RAW.add((src, ph, pack) ->
		{
			assert (pack.port_GET() == (char) 124);
			assert (pack.chan5_raw_GET() == (char) 1924);
			assert (pack.chan6_raw_GET() == (char) 38494);
			assert (pack.chan1_raw_GET() == (char) 17090);
			assert (pack.chan3_raw_GET() == (char) 45746);
			assert (pack.chan8_raw_GET() == (char) 55445);
			assert (pack.rssi_GET() == (char) 41);
			assert (pack.chan7_raw_GET() == (char) 45319);
			assert (pack.chan2_raw_GET() == (char) 25715);
			assert (pack.time_boot_ms_GET() == 1236930332L);
			assert (pack.chan4_raw_GET() == (char) 31118);
		});
		RC_CHANNELS_RAW p35 = new RC_CHANNELS_RAW();
		PH.setPack(p35);
		p35.rssi_SET((char) 41);
		p35.port_SET((char) 124);
		p35.chan5_raw_SET((char) 1924);
		p35.chan8_raw_SET((char) 55445);
		p35.chan1_raw_SET((char) 17090);
		p35.time_boot_ms_SET(1236930332L);
		p35.chan3_raw_SET((char) 45746);
		p35.chan7_raw_SET((char) 45319);
		p35.chan4_raw_SET((char) 31118);
		p35.chan2_raw_SET((char) 25715);
		p35.chan6_raw_SET((char) 38494);
		TestChannel.instance.send(p35);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_SERVO_OUTPUT_RAW.add((src, ph, pack) ->
		{
			assert (pack.servo15_raw_TRY(ph) == (char) 17652);
			assert (pack.servo2_raw_GET() == (char) 64128);
			assert (pack.servo3_raw_GET() == (char) 50035);
			assert (pack.time_usec_GET() == 709101480L);
			assert (pack.servo14_raw_TRY(ph) == (char) 22418);
			assert (pack.servo13_raw_TRY(ph) == (char) 11334);
			assert (pack.servo8_raw_GET() == (char) 6664);
			assert (pack.servo11_raw_TRY(ph) == (char) 54638);
			assert (pack.servo9_raw_TRY(ph) == (char) 1161);
			assert (pack.servo1_raw_GET() == (char) 56726);
			assert (pack.servo6_raw_GET() == (char) 4762);
			assert (pack.servo10_raw_TRY(ph) == (char) 26462);
			assert (pack.servo16_raw_TRY(ph) == (char) 19343);
			assert (pack.servo4_raw_GET() == (char) 57426);
			assert (pack.servo12_raw_TRY(ph) == (char) 61738);
			assert (pack.port_GET() == (char) 46);
			assert (pack.servo5_raw_GET() == (char) 6473);
			assert (pack.servo7_raw_GET() == (char) 49462);
		});
		SERVO_OUTPUT_RAW p36 = new SERVO_OUTPUT_RAW();
		PH.setPack(p36);
		p36.servo12_raw_SET((char) 61738, PH);
		p36.time_usec_SET(709101480L);
		p36.servo6_raw_SET((char) 4762);
		p36.servo9_raw_SET((char) 1161, PH);
		p36.servo1_raw_SET((char) 56726);
		p36.port_SET((char) 46);
		p36.servo8_raw_SET((char) 6664);
		p36.servo14_raw_SET((char) 22418, PH);
		p36.servo5_raw_SET((char) 6473);
		p36.servo16_raw_SET((char) 19343, PH);
		p36.servo7_raw_SET((char) 49462);
		p36.servo13_raw_SET((char) 11334, PH);
		p36.servo2_raw_SET((char) 64128);
		p36.servo15_raw_SET((char) 17652, PH);
		p36.servo4_raw_SET((char) 57426);
		p36.servo11_raw_SET((char) 54638, PH);
		p36.servo10_raw_SET((char) 26462, PH);
		p36.servo3_raw_SET((char) 50035);
		TestChannel.instance.send(p36);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_MISSION_REQUEST_PARTIAL_LIST.add((src, ph, pack) ->
		{
			assert (pack.end_index_GET() == (short) -19590);
			assert (pack.target_system_GET() == (char) 203);
			assert (pack.start_index_GET() == (short) 27225);
			assert (pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
			assert (pack.target_component_GET() == (char) 97);
		});
		MISSION_REQUEST_PARTIAL_LIST p37 = new MISSION_REQUEST_PARTIAL_LIST();
		PH.setPack(p37);
		p37.end_index_SET((short) -19590);
		p37.target_component_SET((char) 97);
		p37.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
		p37.start_index_SET((short) 27225);
		p37.target_system_SET((char) 203);
		TestChannel.instance.send(p37);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_MISSION_WRITE_PARTIAL_LIST.add((src, ph, pack) ->
		{
			assert (pack.end_index_GET() == (short) -11002);
			assert (pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
			assert (pack.start_index_GET() == (short) 20423);
			assert (pack.target_component_GET() == (char) 44);
			assert (pack.target_system_GET() == (char) 251);
		});
		MISSION_WRITE_PARTIAL_LIST p38 = new MISSION_WRITE_PARTIAL_LIST();
		PH.setPack(p38);
		p38.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
		p38.target_system_SET((char) 251);
		p38.target_component_SET((char) 44);
		p38.start_index_SET((short) 20423);
		p38.end_index_SET((short) -11002);
		TestChannel.instance.send(p38);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_MISSION_ITEM.add((src, ph, pack) ->
		{
			assert (pack.command_GET() == MAV_CMD.MAV_CMD_IMAGE_STOP_CAPTURE);
			assert (pack.autocontinue_GET() == (char) 5);
			assert (pack.frame_GET() == MAV_FRAME.MAV_FRAME_LOCAL_ENU);
			assert (pack.x_GET() == -3.1546791E38F);
			assert (pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
			assert (pack.param3_GET() == -1.4779615E38F);
			assert (pack.y_GET() == -2.4760321E38F);
			assert (pack.param1_GET() == -3.315328E38F);
			assert (pack.param2_GET() == 1.6158976E38F);
			assert (pack.z_GET() == 2.2169574E38F);
			assert (pack.seq_GET() == (char) 34738);
			assert (pack.target_system_GET() == (char) 124);
			assert (pack.current_GET() == (char) 148);
			assert (pack.target_component_GET() == (char) 198);
			assert (pack.param4_GET() == 3.3329944E38F);
		});
		MISSION_ITEM p39 = new MISSION_ITEM();
		PH.setPack(p39);
		p39.target_system_SET((char) 124);
		p39.param2_SET(1.6158976E38F);
		p39.param3_SET(-1.4779615E38F);
		p39.x_SET(-3.1546791E38F);
		p39.target_component_SET((char) 198);
		p39.seq_SET((char) 34738);
		p39.param4_SET(3.3329944E38F);
		p39.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
		p39.command_SET(MAV_CMD.MAV_CMD_IMAGE_STOP_CAPTURE);
		p39.z_SET(2.2169574E38F);
		p39.current_SET((char) 148);
		p39.autocontinue_SET((char) 5);
		p39.y_SET(-2.4760321E38F);
		p39.param1_SET(-3.315328E38F);
		p39.frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_ENU);
		TestChannel.instance.send(p39);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_MISSION_REQUEST.add((src, ph, pack) ->
		{
			assert (pack.target_component_GET() == (char) 227);
			assert (pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
			assert (pack.target_system_GET() == (char) 39);
			assert (pack.seq_GET() == (char) 38723);
		});
		MISSION_REQUEST p40 = new MISSION_REQUEST();
		PH.setPack(p40);
		p40.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
		p40.target_component_SET((char) 227);
		p40.target_system_SET((char) 39);
		p40.seq_SET((char) 38723);
		TestChannel.instance.send(p40);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_MISSION_SET_CURRENT.add((src, ph, pack) ->
		{
			assert (pack.target_component_GET() == (char) 62);
			assert (pack.target_system_GET() == (char) 109);
			assert (pack.seq_GET() == (char) 24950);
		});
		MISSION_SET_CURRENT p41 = new MISSION_SET_CURRENT();
		PH.setPack(p41);
		p41.seq_SET((char) 24950);
		p41.target_component_SET((char) 62);
		p41.target_system_SET((char) 109);
		TestChannel.instance.send(p41);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_MISSION_CURRENT.add((src, ph, pack) ->
		{
			assert (pack.seq_GET() == (char) 11471);
		});
		MISSION_CURRENT p42 = new MISSION_CURRENT();
		PH.setPack(p42);
		p42.seq_SET((char) 11471);
		TestChannel.instance.send(p42);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_MISSION_REQUEST_LIST.add((src, ph, pack) ->
		{
			assert (pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
			assert (pack.target_system_GET() == (char) 14);
			assert (pack.target_component_GET() == (char) 36);
		});
		MISSION_REQUEST_LIST p43 = new MISSION_REQUEST_LIST();
		PH.setPack(p43);
		p43.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
		p43.target_component_SET((char) 36);
		p43.target_system_SET((char) 14);
		TestChannel.instance.send(p43);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_MISSION_COUNT.add((src, ph, pack) ->
		{
			assert (pack.count_GET() == (char) 26768);
			assert (pack.target_system_GET() == (char) 198);
			assert (pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
			assert (pack.target_component_GET() == (char) 207);
		});
		MISSION_COUNT p44 = new MISSION_COUNT();
		PH.setPack(p44);
		p44.target_system_SET((char) 198);
		p44.count_SET((char) 26768);
		p44.target_component_SET((char) 207);
		p44.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
		TestChannel.instance.send(p44);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_MISSION_CLEAR_ALL.add((src, ph, pack) ->
		{
			assert (pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
			assert (pack.target_system_GET() == (char) 213);
			assert (pack.target_component_GET() == (char) 151);
		});
		MISSION_CLEAR_ALL p45 = new MISSION_CLEAR_ALL();
		PH.setPack(p45);
		p45.target_system_SET((char) 213);
		p45.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
		p45.target_component_SET((char) 151);
		TestChannel.instance.send(p45);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_MISSION_ITEM_REACHED.add((src, ph, pack) ->
		{
			assert (pack.seq_GET() == (char) 21616);
		});
		MISSION_ITEM_REACHED p46 = new MISSION_ITEM_REACHED();
		PH.setPack(p46);
		p46.seq_SET((char) 21616);
		TestChannel.instance.send(p46);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_MISSION_ACK.add((src, ph, pack) ->
		{
			assert (pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
			assert (pack.target_system_GET() == (char) 24);
			assert (pack.type_GET() == MAV_MISSION_RESULT.MAV_MISSION_INVALID_PARAM6_Y);
			assert (pack.target_component_GET() == (char) 149);
		});
		MISSION_ACK p47 = new MISSION_ACK();
		PH.setPack(p47);
		p47.target_system_SET((char) 24);
		p47.target_component_SET((char) 149);
		p47.type_SET(MAV_MISSION_RESULT.MAV_MISSION_INVALID_PARAM6_Y);
		p47.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
		TestChannel.instance.send(p47);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_SET_GPS_GLOBAL_ORIGIN.add((src, ph, pack) ->
		{
			assert (pack.latitude_GET() == 502635584);
			assert (pack.target_system_GET() == (char) 63);
			assert (pack.longitude_GET() == -1027399803);
			assert (pack.altitude_GET() == -735709405);
			assert (pack.time_usec_TRY(ph) == 615934460773637760L);
		});
		SET_GPS_GLOBAL_ORIGIN p48 = new SET_GPS_GLOBAL_ORIGIN();
		PH.setPack(p48);
		p48.longitude_SET(-1027399803);
		p48.latitude_SET(502635584);
		p48.time_usec_SET(615934460773637760L, PH);
		p48.target_system_SET((char) 63);
		p48.altitude_SET(-735709405);
		TestChannel.instance.send(p48);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_GPS_GLOBAL_ORIGIN.add((src, ph, pack) ->
		{
			assert (pack.longitude_GET() == 964464409);
			assert (pack.altitude_GET() == -1784880437);
			assert (pack.latitude_GET() == -1530316131);
			assert (pack.time_usec_TRY(ph) == 7141347079168619605L);
		});
		GPS_GLOBAL_ORIGIN p49 = new GPS_GLOBAL_ORIGIN();
		PH.setPack(p49);
		p49.time_usec_SET(7141347079168619605L, PH);
		p49.latitude_SET(-1530316131);
		p49.altitude_SET(-1784880437);
		p49.longitude_SET(964464409);
		TestChannel.instance.send(p49);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_PARAM_MAP_RC.add((src, ph, pack) ->
		{
			assert (pack.target_component_GET() == (char) 1);
			assert (pack.target_system_GET() == (char) 221);
			assert (pack.param_value_max_GET() == -1.760084E38F);
			assert (pack.param_index_GET() == (short) 24074);
			assert (pack.param_value0_GET() == -4.6900217E37F);
			assert (pack.param_id_LEN(ph) == 4);
			assert (pack.param_id_TRY(ph).equals("hroF"));
			assert (pack.scale_GET() == 3.2849023E38F);
			assert (pack.parameter_rc_channel_index_GET() == (char) 149);
			assert (pack.param_value_min_GET() == 3.5545757E37F);
		});
		PARAM_MAP_RC p50 = new PARAM_MAP_RC();
		PH.setPack(p50);
		p50.param_index_SET((short) 24074);
		p50.target_system_SET((char) 221);
		p50.target_component_SET((char) 1);
		p50.param_value0_SET(-4.6900217E37F);
		p50.scale_SET(3.2849023E38F);
		p50.param_id_SET("hroF", PH);
		p50.parameter_rc_channel_index_SET((char) 149);
		p50.param_value_max_SET(-1.760084E38F);
		p50.param_value_min_SET(3.5545757E37F);
		TestChannel.instance.send(p50);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_MISSION_REQUEST_INT.add((src, ph, pack) ->
		{
			assert (pack.seq_GET() == (char) 34576);
			assert (pack.target_system_GET() == (char) 14);
			assert (pack.target_component_GET() == (char) 100);
			assert (pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
		});
		MISSION_REQUEST_INT p51 = new MISSION_REQUEST_INT();
		PH.setPack(p51);
		p51.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
		p51.seq_SET((char) 34576);
		p51.target_system_SET((char) 14);
		p51.target_component_SET((char) 100);
		TestChannel.instance.send(p51);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_SAFETY_SET_ALLOWED_AREA.add((src, ph, pack) ->
		{
			assert (pack.p2z_GET() == 2.7969153E38F);
			assert (pack.target_component_GET() == (char) 117);
			assert (pack.p1x_GET() == 1.2662585E38F);
			assert (pack.p2x_GET() == -3.2820614E38F);
			assert (pack.p2y_GET() == 4.540437E37F);
			assert (pack.frame_GET() == MAV_FRAME.MAV_FRAME_LOCAL_NED);
			assert (pack.p1y_GET() == -2.5084828E38F);
			assert (pack.target_system_GET() == (char) 109);
			assert (pack.p1z_GET() == 3.1893339E38F);
		});
		SAFETY_SET_ALLOWED_AREA p54 = new SAFETY_SET_ALLOWED_AREA();
		PH.setPack(p54);
		p54.target_system_SET((char) 109);
		p54.p1y_SET(-2.5084828E38F);
		p54.p1x_SET(1.2662585E38F);
		p54.p2x_SET(-3.2820614E38F);
		p54.p1z_SET(3.1893339E38F);
		p54.frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_NED);
		p54.p2z_SET(2.7969153E38F);
		p54.target_component_SET((char) 117);
		p54.p2y_SET(4.540437E37F);
		TestChannel.instance.send(p54);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_SAFETY_ALLOWED_AREA.add((src, ph, pack) ->
		{
			assert (pack.p1z_GET() == -3.3447065E38F);
			assert (pack.p1y_GET() == 1.085816E38F);
			assert (pack.p2z_GET() == -9.603517E36F);
			assert (pack.frame_GET() == MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED);
			assert (pack.p2y_GET() == 3.1751164E37F);
			assert (pack.p1x_GET() == -2.2412527E38F);
			assert (pack.p2x_GET() == -1.5124753E38F);
		});
		SAFETY_ALLOWED_AREA p55 = new SAFETY_ALLOWED_AREA();
		PH.setPack(p55);
		p55.p2y_SET(3.1751164E37F);
		p55.p1z_SET(-3.3447065E38F);
		p55.frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED);
		p55.p1x_SET(-2.2412527E38F);
		p55.p2x_SET(-1.5124753E38F);
		p55.p2z_SET(-9.603517E36F);
		p55.p1y_SET(1.085816E38F);
		TestChannel.instance.send(p55);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_ATTITUDE_QUATERNION_COV.add((src, ph, pack) ->
		{
			assert (Arrays.equals(pack.covariance_GET(), new float[]{-3.3855854E38F, 3.196936E38F, -1.5000504E38F, 1.6081346E38F, 2.3101346E38F, -2.0806241E38F, -1.6103599E38F, 3.2998387E38F, -6.340819E36F}));
			assert (pack.yawspeed_GET() == -1.1164574E38F);
			assert (pack.pitchspeed_GET() == 1.7005717E38F);
			assert (pack.time_usec_GET() == 8147136186375936915L);
			assert (pack.rollspeed_GET() == 1.2340101E38F);
			assert (Arrays.equals(pack.q_GET(), new float[]{-1.5318698E38F, -1.9193569E38F, 7.0643475E37F, 1.791141E38F}));
		});
		ATTITUDE_QUATERNION_COV p61 = new ATTITUDE_QUATERNION_COV();
		PH.setPack(p61);
		p61.covariance_SET(new float[]{-3.3855854E38F, 3.196936E38F, -1.5000504E38F, 1.6081346E38F, 2.3101346E38F, -2.0806241E38F, -1.6103599E38F, 3.2998387E38F, -6.340819E36F}, 0);
		p61.yawspeed_SET(-1.1164574E38F);
		p61.rollspeed_SET(1.2340101E38F);
		p61.time_usec_SET(8147136186375936915L);
		p61.pitchspeed_SET(1.7005717E38F);
		p61.q_SET(new float[]{-1.5318698E38F, -1.9193569E38F, 7.0643475E37F, 1.791141E38F}, 0);
		TestChannel.instance.send(p61);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_NAV_CONTROLLER_OUTPUT.add((src, ph, pack) ->
		{
			assert (pack.wp_dist_GET() == (char) 62333);
			assert (pack.nav_pitch_GET() == 7.722244E37F);
			assert (pack.xtrack_error_GET() == -1.2167146E38F);
			assert (pack.target_bearing_GET() == (short) -6228);
			assert (pack.alt_error_GET() == 2.1089864E38F);
			assert (pack.aspd_error_GET() == -8.84399E37F);
			assert (pack.nav_bearing_GET() == (short) 12277);
			assert (pack.nav_roll_GET() == -1.8943731E37F);
		});
		NAV_CONTROLLER_OUTPUT p62 = new NAV_CONTROLLER_OUTPUT();
		PH.setPack(p62);
		p62.alt_error_SET(2.1089864E38F);
		p62.nav_pitch_SET(7.722244E37F);
		p62.target_bearing_SET((short) -6228);
		p62.nav_roll_SET(-1.8943731E37F);
		p62.wp_dist_SET((char) 62333);
		p62.aspd_error_SET(-8.84399E37F);
		p62.nav_bearing_SET((short) 12277);
		p62.xtrack_error_SET(-1.2167146E38F);
		TestChannel.instance.send(p62);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_GLOBAL_POSITION_INT_COV.add((src, ph, pack) ->
		{
			assert (pack.lat_GET() == 2017923229);
			assert (pack.vy_GET() == 2.2836E38F);
			assert (pack.estimator_type_GET() == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VIO);
			assert (pack.lon_GET() == 1300092430);
			assert (Arrays.equals(pack.covariance_GET(), new float[]{1.6530657E37F, -3.1752528E38F, -3.305496E36F, 1.7354356E38F, -1.2059865E37F, 6.347651E37F, 7.119721E37F, 3.2630854E38F, 2.566296E37F, 3.92851E36F, 2.8281057E37F, -3.0825013E38F, 2.7083787E38F, -2.125245E36F, 1.855678E38F, 2.3103395E38F, 1.98054E38F, 3.357633E38F, 1.4804681E38F, 2.3416734E38F, 2.0809164E38F, -9.588924E37F, 1.2631408E37F, 1.6815874E38F, 2.294398E38F, -9.561231E37F, -2.274695E38F, 1.6008366E38F, 8.573793E37F, 1.0370437E38F, 3.3115725E38F, 8.194635E37F, 2.7304044E38F, 2.6735327E38F, 7.050176E37F, 9.996832E37F}));
			assert (pack.time_usec_GET() == 7405523066554464000L);
			assert (pack.alt_GET() == -1495288532);
			assert (pack.vz_GET() == 2.9192121E38F);
			assert (pack.vx_GET() == 2.903143E38F);
			assert (pack.relative_alt_GET() == -432479014);
		});
		GLOBAL_POSITION_INT_COV p63 = new GLOBAL_POSITION_INT_COV();
		PH.setPack(p63);
		p63.relative_alt_SET(-432479014);
		p63.time_usec_SET(7405523066554464000L);
		p63.covariance_SET(new float[]{1.6530657E37F, -3.1752528E38F, -3.305496E36F, 1.7354356E38F, -1.2059865E37F, 6.347651E37F, 7.119721E37F, 3.2630854E38F, 2.566296E37F, 3.92851E36F, 2.8281057E37F, -3.0825013E38F, 2.7083787E38F, -2.125245E36F, 1.855678E38F, 2.3103395E38F, 1.98054E38F, 3.357633E38F, 1.4804681E38F, 2.3416734E38F, 2.0809164E38F, -9.588924E37F, 1.2631408E37F, 1.6815874E38F, 2.294398E38F, -9.561231E37F, -2.274695E38F, 1.6008366E38F, 8.573793E37F, 1.0370437E38F, 3.3115725E38F, 8.194635E37F, 2.7304044E38F, 2.6735327E38F, 7.050176E37F, 9.996832E37F}, 0);
		p63.alt_SET(-1495288532);
		p63.lat_SET(2017923229);
		p63.estimator_type_SET(MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VIO);
		p63.vx_SET(2.903143E38F);
		p63.lon_SET(1300092430);
		p63.vz_SET(2.9192121E38F);
		p63.vy_SET(2.2836E38F);
		TestChannel.instance.send(p63);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_LOCAL_POSITION_NED_COV.add((src, ph, pack) ->
		{
			assert (pack.vz_GET() == 2.9558196E38F);
			assert (pack.ay_GET() == -3.6865206E37F);
			assert (pack.time_usec_GET() == 7394138598617245487L);
			assert (pack.z_GET() == -1.0512503E38F);
			assert (pack.vx_GET() == 2.4824266E38F);
			assert (pack.ax_GET() == 1.1941195E37F);
			assert (pack.az_GET() == -9.551424E37F);
			assert (Arrays.equals(pack.covariance_GET(), new float[]{-1.829615E38F, -3.0201897E38F, 3.3124933E38F, 1.0048464E38F, 1.3880901E38F, -1.3015938E38F, -2.7353373E38F, -9.337175E37F, -4.7016106E37F, 3.1721908E38F, 2.2409274E38F, 3.2163103E38F, 1.1912522E38F, -2.7248464E38F, -4.883607E37F, 2.9931519E38F, -1.291117E38F, 3.3465874E38F, -3.1144013E38F, -3.1550811E38F, -3.2449338E38F, 2.9166967E38F, 3.3220192E38F, -2.1101638E38F, -1.5052044E38F, -1.0556968E38F, 6.696174E37F, -2.038186E38F, -3.2293965E38F, 3.112046E38F, 7.1105265E37F, 1.878644E38F, 1.0189968E38F, 1.3570788E38F, -2.3756314E38F, -2.6492048E38F, 7.664939E37F, -3.2224926E38F, -1.4594161E37F, 1.1102633E38F, 1.2121591E38F, 1.1697588E38F, -2.9945075E38F, 2.226493E38F, -1.7209235E38F}));
			assert (pack.vy_GET() == -3.215318E38F);
			assert (pack.y_GET() == 3.1716253E38F);
			assert (pack.x_GET() == -5.8781503E36F);
			assert (pack.estimator_type_GET() == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VISION);
		});
		LOCAL_POSITION_NED_COV p64 = new LOCAL_POSITION_NED_COV();
		PH.setPack(p64);
		p64.y_SET(3.1716253E38F);
		p64.ax_SET(1.1941195E37F);
		p64.z_SET(-1.0512503E38F);
		p64.x_SET(-5.8781503E36F);
		p64.vz_SET(2.9558196E38F);
		p64.az_SET(-9.551424E37F);
		p64.vx_SET(2.4824266E38F);
		p64.vy_SET(-3.215318E38F);
		p64.time_usec_SET(7394138598617245487L);
		p64.estimator_type_SET(MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VISION);
		p64.covariance_SET(new float[]{-1.829615E38F, -3.0201897E38F, 3.3124933E38F, 1.0048464E38F, 1.3880901E38F, -1.3015938E38F, -2.7353373E38F, -9.337175E37F, -4.7016106E37F, 3.1721908E38F, 2.2409274E38F, 3.2163103E38F, 1.1912522E38F, -2.7248464E38F, -4.883607E37F, 2.9931519E38F, -1.291117E38F, 3.3465874E38F, -3.1144013E38F, -3.1550811E38F, -3.2449338E38F, 2.9166967E38F, 3.3220192E38F, -2.1101638E38F, -1.5052044E38F, -1.0556968E38F, 6.696174E37F, -2.038186E38F, -3.2293965E38F, 3.112046E38F, 7.1105265E37F, 1.878644E38F, 1.0189968E38F, 1.3570788E38F, -2.3756314E38F, -2.6492048E38F, 7.664939E37F, -3.2224926E38F, -1.4594161E37F, 1.1102633E38F, 1.2121591E38F, 1.1697588E38F, -2.9945075E38F, 2.226493E38F, -1.7209235E38F}, 0);
		p64.ay_SET(-3.6865206E37F);
		TestChannel.instance.send(p64);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_RC_CHANNELS.add((src, ph, pack) ->
		{
			assert (pack.chan6_raw_GET() == (char) 61216);
			assert (pack.chan5_raw_GET() == (char) 38654);
			assert (pack.chan15_raw_GET() == (char) 24530);
			assert (pack.chan3_raw_GET() == (char) 2407);
			assert (pack.chan8_raw_GET() == (char) 40993);
			assert (pack.chancount_GET() == (char) 237);
			assert (pack.chan9_raw_GET() == (char) 31545);
			assert (pack.chan2_raw_GET() == (char) 4284);
			assert (pack.chan10_raw_GET() == (char) 25011);
			assert (pack.rssi_GET() == (char) 42);
			assert (pack.chan4_raw_GET() == (char) 16978);
			assert (pack.time_boot_ms_GET() == 3988615132L);
			assert (pack.chan17_raw_GET() == (char) 1834);
			assert (pack.chan16_raw_GET() == (char) 47017);
			assert (pack.chan12_raw_GET() == (char) 29980);
			assert (pack.chan18_raw_GET() == (char) 17117);
			assert (pack.chan7_raw_GET() == (char) 26427);
			assert (pack.chan1_raw_GET() == (char) 2065);
			assert (pack.chan14_raw_GET() == (char) 21912);
			assert (pack.chan11_raw_GET() == (char) 47972);
			assert (pack.chan13_raw_GET() == (char) 41169);
		});
		RC_CHANNELS p65 = new RC_CHANNELS();
		PH.setPack(p65);
		p65.chan6_raw_SET((char) 61216);
		p65.chan7_raw_SET((char) 26427);
		p65.chan8_raw_SET((char) 40993);
		p65.rssi_SET((char) 42);
		p65.chancount_SET((char) 237);
		p65.chan5_raw_SET((char) 38654);
		p65.chan15_raw_SET((char) 24530);
		p65.chan11_raw_SET((char) 47972);
		p65.chan16_raw_SET((char) 47017);
		p65.chan3_raw_SET((char) 2407);
		p65.chan18_raw_SET((char) 17117);
		p65.chan9_raw_SET((char) 31545);
		p65.chan13_raw_SET((char) 41169);
		p65.chan1_raw_SET((char) 2065);
		p65.time_boot_ms_SET(3988615132L);
		p65.chan14_raw_SET((char) 21912);
		p65.chan4_raw_SET((char) 16978);
		p65.chan10_raw_SET((char) 25011);
		p65.chan12_raw_SET((char) 29980);
		p65.chan17_raw_SET((char) 1834);
		p65.chan2_raw_SET((char) 4284);
		TestChannel.instance.send(p65);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_REQUEST_DATA_STREAM.add((src, ph, pack) ->
		{
			assert (pack.target_system_GET() == (char) 59);
			assert (pack.req_stream_id_GET() == (char) 195);
			assert (pack.req_message_rate_GET() == (char) 16360);
			assert (pack.target_component_GET() == (char) 240);
			assert (pack.start_stop_GET() == (char) 223);
		});
		REQUEST_DATA_STREAM p66 = new REQUEST_DATA_STREAM();
		PH.setPack(p66);
		p66.target_component_SET((char) 240);
		p66.req_message_rate_SET((char) 16360);
		p66.start_stop_SET((char) 223);
		p66.target_system_SET((char) 59);
		p66.req_stream_id_SET((char) 195);
		TestChannel.instance.send(p66);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_DATA_STREAM.add((src, ph, pack) ->
		{
			assert (pack.stream_id_GET() == (char) 63);
			assert (pack.on_off_GET() == (char) 85);
			assert (pack.message_rate_GET() == (char) 20524);
		});
		DATA_STREAM p67 = new DATA_STREAM();
		PH.setPack(p67);
		p67.message_rate_SET((char) 20524);
		p67.stream_id_SET((char) 63);
		p67.on_off_SET((char) 85);
		TestChannel.instance.send(p67);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_MANUAL_CONTROL.add((src, ph, pack) ->
		{
			assert (pack.buttons_GET() == (char) 9506);
			assert (pack.target_GET() == (char) 245);
			assert (pack.y_GET() == (short) -32521);
			assert (pack.x_GET() == (short) -15077);
			assert (pack.r_GET() == (short) 17078);
			assert (pack.z_GET() == (short) -30613);
		});
		MANUAL_CONTROL p69 = new MANUAL_CONTROL();
		PH.setPack(p69);
		p69.target_SET((char) 245);
		p69.y_SET((short) -32521);
		p69.buttons_SET((char) 9506);
		p69.x_SET((short) -15077);
		p69.z_SET((short) -30613);
		p69.r_SET((short) 17078);
		TestChannel.instance.send(p69);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_RC_CHANNELS_OVERRIDE.add((src, ph, pack) ->
		{
			assert (pack.chan1_raw_GET() == (char) 16800);
			assert (pack.chan5_raw_GET() == (char) 30722);
			assert (pack.chan8_raw_GET() == (char) 34881);
			assert (pack.target_component_GET() == (char) 85);
			assert (pack.chan4_raw_GET() == (char) 19035);
			assert (pack.chan6_raw_GET() == (char) 57721);
			assert (pack.target_system_GET() == (char) 30);
			assert (pack.chan7_raw_GET() == (char) 19001);
			assert (pack.chan2_raw_GET() == (char) 12209);
			assert (pack.chan3_raw_GET() == (char) 28018);
		});
		RC_CHANNELS_OVERRIDE p70 = new RC_CHANNELS_OVERRIDE();
		PH.setPack(p70);
		p70.chan1_raw_SET((char) 16800);
		p70.target_component_SET((char) 85);
		p70.chan7_raw_SET((char) 19001);
		p70.chan4_raw_SET((char) 19035);
		p70.chan8_raw_SET((char) 34881);
		p70.chan3_raw_SET((char) 28018);
		p70.chan2_raw_SET((char) 12209);
		p70.chan5_raw_SET((char) 30722);
		p70.chan6_raw_SET((char) 57721);
		p70.target_system_SET((char) 30);
		TestChannel.instance.send(p70);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_MISSION_ITEM_INT.add((src, ph, pack) ->
		{
			assert (pack.param1_GET() == 7.9060006E37F);
			assert (pack.z_GET() == 2.9417808E38F);
			assert (pack.target_system_GET() == (char) 235);
			assert (pack.param4_GET() == 1.5679716E38F);
			assert (pack.target_component_GET() == (char) 182);
			assert (pack.command_GET() == MAV_CMD.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN);
			assert (pack.seq_GET() == (char) 30960);
			assert (pack.x_GET() == -830130574);
			assert (pack.current_GET() == (char) 94);
			assert (pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT);
			assert (pack.param2_GET() == -2.1621838E38F);
			assert (pack.param3_GET() == -3.3178206E36F);
			assert (pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
			assert (pack.y_GET() == 211800316);
			assert (pack.autocontinue_GET() == (char) 88);
		});
		MISSION_ITEM_INT p73 = new MISSION_ITEM_INT();
		PH.setPack(p73);
		p73.param2_SET(-2.1621838E38F);
		p73.y_SET(211800316);
		p73.target_system_SET((char) 235);
		p73.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
		p73.param4_SET(1.5679716E38F);
		p73.param1_SET(7.9060006E37F);
		p73.autocontinue_SET((char) 88);
		p73.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT);
		p73.param3_SET(-3.3178206E36F);
		p73.z_SET(2.9417808E38F);
		p73.command_SET(MAV_CMD.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN);
		p73.current_SET((char) 94);
		p73.target_component_SET((char) 182);
		p73.x_SET(-830130574);
		p73.seq_SET((char) 30960);
		TestChannel.instance.send(p73);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_VFR_HUD.add((src, ph, pack) ->
		{
			assert (pack.heading_GET() == (short) 1045);
			assert (pack.throttle_GET() == (char) 58320);
			assert (pack.climb_GET() == -1.8550357E38F);
			assert (pack.airspeed_GET() == 2.5019679E38F);
			assert (pack.alt_GET() == -2.4268647E38F);
			assert (pack.groundspeed_GET() == -3.061356E38F);
		});
		VFR_HUD p74 = new VFR_HUD();
		PH.setPack(p74);
		p74.alt_SET(-2.4268647E38F);
		p74.groundspeed_SET(-3.061356E38F);
		p74.throttle_SET((char) 58320);
		p74.heading_SET((short) 1045);
		p74.airspeed_SET(2.5019679E38F);
		p74.climb_SET(-1.8550357E38F);
		TestChannel.instance.send(p74);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_COMMAND_INT.add((src, ph, pack) ->
		{
			assert (pack.target_system_GET() == (char) 251);
			assert (pack.target_component_GET() == (char) 0);
			assert (pack.x_GET() == -101058507);
			assert (pack.current_GET() == (char) 69);
			assert (pack.param4_GET() == -9.556497E37F);
			assert (pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT);
			assert (pack.z_GET() == 1.9749585E38F);
			assert (pack.param1_GET() == 1.8161233E38F);
			assert (pack.param2_GET() == -1.5670245E38F);
			assert (pack.param3_GET() == 3.3179615E38F);
			assert (pack.command_GET() == MAV_CMD.MAV_CMD_NAV_VTOL_LAND);
			assert (pack.y_GET() == -1511310680);
			assert (pack.autocontinue_GET() == (char) 103);
		});
		GroundControl.COMMAND_INT p75 = CommunicationChannel.new_COMMAND_INT();
		PH.setPack(p75);
		p75.param1_SET(1.8161233E38F);
		p75.current_SET((char) 69);
		p75.y_SET(-1511310680);
		p75.autocontinue_SET((char) 103);
		p75.target_system_SET((char) 251);
		p75.param4_SET(-9.556497E37F);
		p75.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT);
		p75.command_SET(MAV_CMD.MAV_CMD_NAV_VTOL_LAND);
		p75.param3_SET(3.3179615E38F);
		p75.z_SET(1.9749585E38F);
		p75.target_component_SET((char) 0);
		p75.param2_SET(-1.5670245E38F);
		p75.x_SET(-101058507);
		CommunicationChannel.instance.send(p75);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_COMMAND_LONG.add((src, ph, pack) ->
		{
			assert (pack.param7_GET() == 1.3563261E38F);
			assert (pack.param5_GET() == 3.220866E38F);
			assert (pack.param1_GET() == -8.2786833E37F);
			assert (pack.command_GET() == MAV_CMD.MAV_CMD_VIDEO_STOP_STREAMING);
			assert (pack.param2_GET() == 3.2688052E38F);
			assert (pack.param6_GET() == 6.9235146E36F);
			assert (pack.target_component_GET() == (char) 230);
			assert (pack.target_system_GET() == (char) 139);
			assert (pack.confirmation_GET() == (char) 23);
			assert (pack.param4_GET() == 1.0527334E38F);
			assert (pack.param3_GET() == 8.074158E36F);
		});
		GroundControl.COMMAND_LONG p76 = CommunicationChannel.new_COMMAND_LONG();
		PH.setPack(p76);
		p76.param5_SET(3.220866E38F);
		p76.confirmation_SET((char) 23);
		p76.param4_SET(1.0527334E38F);
		p76.param1_SET(-8.2786833E37F);
		p76.command_SET(MAV_CMD.MAV_CMD_VIDEO_STOP_STREAMING);
		p76.param7_SET(1.3563261E38F);
		p76.target_system_SET((char) 139);
		p76.param3_SET(8.074158E36F);
		p76.target_component_SET((char) 230);
		p76.param2_SET(3.2688052E38F);
		p76.param6_SET(6.9235146E36F);
		CommunicationChannel.instance.send(p76);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_COMMAND_ACK.add((src, ph, pack) ->
		{
			assert (pack.result_param2_TRY(ph) == -1948366802);
			assert (pack.target_system_TRY(ph) == (char) 186);
			assert (pack.result_GET() == MAV_RESULT.MAV_RESULT_ACCEPTED);
			assert (pack.progress_TRY(ph) == (char) 117);
			assert (pack.target_component_TRY(ph) == (char) 138);
			assert (pack.command_GET() == MAV_CMD.MAV_CMD_DO_GUIDED_LIMITS);
		});
		GroundControl.COMMAND_ACK p77 = CommunicationChannel.new_COMMAND_ACK();
		PH.setPack(p77);
		p77.command_SET(MAV_CMD.MAV_CMD_DO_GUIDED_LIMITS);
		p77.target_system_SET((char) 186, PH);
		p77.result_param2_SET(-1948366802, PH);
		p77.result_SET(MAV_RESULT.MAV_RESULT_ACCEPTED);
		p77.progress_SET((char) 117, PH);
		p77.target_component_SET((char) 138, PH);
		CommunicationChannel.instance.send(p77);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_MANUAL_SETPOINT.add((src, ph, pack) ->
		{
			assert (pack.time_boot_ms_GET() == 1375369377L);
			assert (pack.manual_override_switch_GET() == (char) 20);
			assert (pack.mode_switch_GET() == (char) 203);
			assert (pack.yaw_GET() == 2.0916213E38F);
			assert (pack.roll_GET() == 3.0681323E38F);
			assert (pack.pitch_GET() == -2.0208622E37F);
			assert (pack.thrust_GET() == 3.0628774E37F);
		});
		GroundControl.MANUAL_SETPOINT p81 = CommunicationChannel.new_MANUAL_SETPOINT();
		PH.setPack(p81);
		p81.thrust_SET(3.0628774E37F);
		p81.roll_SET(3.0681323E38F);
		p81.yaw_SET(2.0916213E38F);
		p81.pitch_SET(-2.0208622E37F);
		p81.time_boot_ms_SET(1375369377L);
		p81.mode_switch_SET((char) 203);
		p81.manual_override_switch_SET((char) 20);
		CommunicationChannel.instance.send(p81);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_SET_ATTITUDE_TARGET.add((src, ph, pack) ->
		{
			assert (pack.type_mask_GET() == (char) 119);
			assert (pack.body_yaw_rate_GET() == 3.2902269E38F);
			assert (pack.thrust_GET() == -1.0559459E38F);
			assert (pack.target_system_GET() == (char) 160);
			assert (pack.body_roll_rate_GET() == 2.8858904E38F);
			assert (pack.time_boot_ms_GET() == 2115515770L);
			assert (pack.body_pitch_rate_GET() == -4.0831198E37F);
			assert (pack.target_component_GET() == (char) 235);
			assert (Arrays.equals(pack.q_GET(), new float[]{-2.0634017E38F, 1.9310543E38F, 2.1094592E38F, 2.0484376E38F}));
		});
		GroundControl.SET_ATTITUDE_TARGET p82 = CommunicationChannel.new_SET_ATTITUDE_TARGET();
		PH.setPack(p82);
		p82.q_SET(new float[]{-2.0634017E38F, 1.9310543E38F, 2.1094592E38F, 2.0484376E38F}, 0);
		p82.target_system_SET((char) 160);
		p82.thrust_SET(-1.0559459E38F);
		p82.time_boot_ms_SET(2115515770L);
		p82.body_pitch_rate_SET(-4.0831198E37F);
		p82.body_roll_rate_SET(2.8858904E38F);
		p82.body_yaw_rate_SET(3.2902269E38F);
		p82.target_component_SET((char) 235);
		p82.type_mask_SET((char) 119);
		CommunicationChannel.instance.send(p82);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_ATTITUDE_TARGET.add((src, ph, pack) ->
		{
			assert (pack.body_yaw_rate_GET() == -2.59207E38F);
			assert (pack.time_boot_ms_GET() == 3236319975L);
			assert (pack.body_roll_rate_GET() == 1.3958084E38F);
			assert (Arrays.equals(pack.q_GET(), new float[]{-1.724984E38F, 2.247136E38F, 1.7896088E37F, 3.0377832E37F}));
			assert (pack.body_pitch_rate_GET() == 2.7629856E38F);
			assert (pack.type_mask_GET() == (char) 203);
			assert (pack.thrust_GET() == 7.756502E36F);
		});
		GroundControl.ATTITUDE_TARGET p83 = CommunicationChannel.new_ATTITUDE_TARGET();
		PH.setPack(p83);
		p83.type_mask_SET((char) 203);
		p83.body_roll_rate_SET(1.3958084E38F);
		p83.time_boot_ms_SET(3236319975L);
		p83.thrust_SET(7.756502E36F);
		p83.body_yaw_rate_SET(-2.59207E38F);
		p83.body_pitch_rate_SET(2.7629856E38F);
		p83.q_SET(new float[]{-1.724984E38F, 2.247136E38F, 1.7896088E37F, 3.0377832E37F}, 0);
		CommunicationChannel.instance.send(p83);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_SET_POSITION_TARGET_LOCAL_NED.add((src, ph, pack) ->
		{
			assert (pack.target_component_GET() == (char) 136);
			assert (pack.vx_GET() == -2.5057193E38F);
			assert (pack.afz_GET() == -8.759459E37F);
			assert (pack.target_system_GET() == (char) 198);
			assert (pack.z_GET() == -1.6865734E38F);
			assert (pack.type_mask_GET() == (char) 4074);
			assert (pack.time_boot_ms_GET() == 4144365834L);
			assert (pack.afx_GET() == -2.16407E37F);
			assert (pack.yaw_GET() == 2.6396603E38F);
			assert (pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED);
			assert (pack.y_GET() == 2.5530327E38F);
			assert (pack.vz_GET() == -1.9367164E38F);
			assert (pack.yaw_rate_GET() == 1.1284449E38F);
			assert (pack.x_GET() == -1.931684E38F);
			assert (pack.vy_GET() == -4.4936297E37F);
			assert (pack.afy_GET() == -2.0260099E38F);
		});
		GroundControl.SET_POSITION_TARGET_LOCAL_NED p84 = CommunicationChannel.new_SET_POSITION_TARGET_LOCAL_NED();
		PH.setPack(p84);
		p84.y_SET(2.5530327E38F);
		p84.afy_SET(-2.0260099E38F);
		p84.vy_SET(-4.4936297E37F);
		p84.yaw_SET(2.6396603E38F);
		p84.z_SET(-1.6865734E38F);
		p84.afx_SET(-2.16407E37F);
		p84.vz_SET(-1.9367164E38F);
		p84.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED);
		p84.vx_SET(-2.5057193E38F);
		p84.target_system_SET((char) 198);
		p84.afz_SET(-8.759459E37F);
		p84.target_component_SET((char) 136);
		p84.x_SET(-1.931684E38F);
		p84.type_mask_SET((char) 4074);
		p84.time_boot_ms_SET(4144365834L);
		p84.yaw_rate_SET(1.1284449E38F);
		CommunicationChannel.instance.send(p84);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_SET_POSITION_TARGET_GLOBAL_INT.add((src, ph, pack) ->
		{
			assert (pack.vx_GET() == 1.3915182E38F);
			assert (pack.type_mask_GET() == (char) 17474);
			assert (pack.time_boot_ms_GET() == 3091956694L);
			assert (pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_INT);
			assert (pack.alt_GET() == -1.957411E38F);
			assert (pack.lon_int_GET() == 69949328);
			assert (pack.afx_GET() == -1.4545349E37F);
			assert (pack.yaw_GET() == 4.6798035E37F);
			assert (pack.vy_GET() == -2.2544788E37F);
			assert (pack.target_system_GET() == (char) 69);
			assert (pack.target_component_GET() == (char) 7);
			assert (pack.afz_GET() == 2.4295009E38F);
			assert (pack.vz_GET() == -2.0956424E38F);
			assert (pack.afy_GET() == 3.14402E38F);
			assert (pack.yaw_rate_GET() == 1.4993675E38F);
			assert (pack.lat_int_GET() == -1223031241);
		});
		GroundControl.SET_POSITION_TARGET_GLOBAL_INT p86 = CommunicationChannel.new_SET_POSITION_TARGET_GLOBAL_INT();
		PH.setPack(p86);
		p86.lat_int_SET(-1223031241);
		p86.vz_SET(-2.0956424E38F);
		p86.alt_SET(-1.957411E38F);
		p86.target_component_SET((char) 7);
		p86.type_mask_SET((char) 17474);
		p86.yaw_rate_SET(1.4993675E38F);
		p86.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_INT);
		p86.vy_SET(-2.2544788E37F);
		p86.afz_SET(2.4295009E38F);
		p86.yaw_SET(4.6798035E37F);
		p86.target_system_SET((char) 69);
		p86.afx_SET(-1.4545349E37F);
		p86.vx_SET(1.3915182E38F);
		p86.afy_SET(3.14402E38F);
		p86.lon_int_SET(69949328);
		p86.time_boot_ms_SET(3091956694L);
		CommunicationChannel.instance.send(p86);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_POSITION_TARGET_GLOBAL_INT.add((src, ph, pack) ->
		{
			assert (pack.vy_GET() == 2.2418973E38F);
			assert (pack.alt_GET() == -2.5449583E38F);
			assert (pack.vx_GET() == 9.867593E37F);
			assert (pack.type_mask_GET() == (char) 34888);
			assert (pack.lat_int_GET() == 335345474);
			assert (pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED);
			assert (pack.yaw_rate_GET() == 6.1070807E37F);
			assert (pack.time_boot_ms_GET() == 2345995352L);
			assert (pack.vz_GET() == 9.823376E36F);
			assert (pack.lon_int_GET() == 792520808);
			assert (pack.afy_GET() == 1.7804972E38F);
			assert (pack.afz_GET() == 1.8508492E38F);
			assert (pack.yaw_GET() == 2.1936884E37F);
			assert (pack.afx_GET() == 4.914306E37F);
		});
		GroundControl.POSITION_TARGET_GLOBAL_INT p87 = CommunicationChannel.new_POSITION_TARGET_GLOBAL_INT();
		PH.setPack(p87);
		p87.afy_SET(1.7804972E38F);
		p87.lat_int_SET(335345474);
		p87.alt_SET(-2.5449583E38F);
		p87.vz_SET(9.823376E36F);
		p87.yaw_SET(2.1936884E37F);
		p87.vy_SET(2.2418973E38F);
		p87.lon_int_SET(792520808);
		p87.time_boot_ms_SET(2345995352L);
		p87.afx_SET(4.914306E37F);
		p87.afz_SET(1.8508492E38F);
		p87.vx_SET(9.867593E37F);
		p87.yaw_rate_SET(6.1070807E37F);
		p87.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED);
		p87.type_mask_SET((char) 34888);
		CommunicationChannel.instance.send(p87);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET.add((src, ph, pack) ->
		{
			assert (pack.roll_GET() == -2.551323E38F);
			assert (pack.x_GET() == 3.4543897E37F);
			assert (pack.y_GET() == 1.0857372E38F);
			assert (pack.yaw_GET() == -9.134523E37F);
			assert (pack.z_GET() == -1.7091808E38F);
			assert (pack.pitch_GET() == 1.5144127E38F);
			assert (pack.time_boot_ms_GET() == 2880695320L);
		});
		GroundControl.LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET p89 = CommunicationChannel.new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
		PH.setPack(p89);
		p89.roll_SET(-2.551323E38F);
		p89.z_SET(-1.7091808E38F);
		p89.yaw_SET(-9.134523E37F);
		p89.x_SET(3.4543897E37F);
		p89.time_boot_ms_SET(2880695320L);
		p89.pitch_SET(1.5144127E38F);
		p89.y_SET(1.0857372E38F);
		CommunicationChannel.instance.send(p89);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_HIL_STATE.add((src, ph, pack) ->
		{
			assert (pack.vx_GET() == (short) 6464);
			assert (pack.zacc_GET() == (short) -16637);
			assert (pack.alt_GET() == -1891500172);
			assert (pack.rollspeed_GET() == 2.2975183E38F);
			assert (pack.xacc_GET() == (short) 14555);
			assert (pack.pitch_GET() == -1.3219509E37F);
			assert (pack.vz_GET() == (short) 24326);
			assert (pack.pitchspeed_GET() == 2.2109656E38F);
			assert (pack.time_usec_GET() == 7748613709667325920L);
			assert (pack.yacc_GET() == (short) 7898);
			assert (pack.yaw_GET() == 2.6154719E38F);
			assert (pack.lat_GET() == 428032107);
			assert (pack.roll_GET() == -1.3840552E38F);
			assert (pack.yawspeed_GET() == -2.4640004E38F);
			assert (pack.vy_GET() == (short) 23541);
			assert (pack.lon_GET() == -375329942);
		});
		GroundControl.HIL_STATE p90 = CommunicationChannel.new_HIL_STATE();
		PH.setPack(p90);
		p90.yawspeed_SET(-2.4640004E38F);
		p90.pitchspeed_SET(2.2109656E38F);
		p90.lon_SET(-375329942);
		p90.roll_SET(-1.3840552E38F);
		p90.yaw_SET(2.6154719E38F);
		p90.lat_SET(428032107);
		p90.vz_SET((short) 24326);
		p90.time_usec_SET(7748613709667325920L);
		p90.vy_SET((short) 23541);
		p90.yacc_SET((short) 7898);
		p90.vx_SET((short) 6464);
		p90.xacc_SET((short) 14555);
		p90.pitch_SET(-1.3219509E37F);
		p90.rollspeed_SET(2.2975183E38F);
		p90.zacc_SET((short) -16637);
		p90.alt_SET(-1891500172);
		CommunicationChannel.instance.send(p90);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_HIL_CONTROLS.add((src, ph, pack) ->
		{
			assert (pack.roll_ailerons_GET() == 3.2079472E38F);
			assert (pack.throttle_GET() == -5.3762046E37F);
			assert (pack.aux2_GET() == -4.1791994E37F);
			assert (pack.time_usec_GET() == 655066891448406980L);
			assert (pack.aux4_GET() == -2.2716812E38F);
			assert (pack.mode_GET() == MAV_MODE.MAV_MODE_STABILIZE_ARMED);
			assert (pack.pitch_elevator_GET() == 3.3034575E38F);
			assert (pack.yaw_rudder_GET() == -2.3004782E38F);
			assert (pack.nav_mode_GET() == (char) 59);
			assert (pack.aux1_GET() == -1.8708138E38F);
			assert (pack.aux3_GET() == -1.2638743E38F);
		});
		GroundControl.HIL_CONTROLS p91 = CommunicationChannel.new_HIL_CONTROLS();
		PH.setPack(p91);
		p91.aux3_SET(-1.2638743E38F);
		p91.aux4_SET(-2.2716812E38F);
		p91.time_usec_SET(655066891448406980L);
		p91.mode_SET(MAV_MODE.MAV_MODE_STABILIZE_ARMED);
		p91.nav_mode_SET((char) 59);
		p91.roll_ailerons_SET(3.2079472E38F);
		p91.aux2_SET(-4.1791994E37F);
		p91.pitch_elevator_SET(3.3034575E38F);
		p91.yaw_rudder_SET(-2.3004782E38F);
		p91.throttle_SET(-5.3762046E37F);
		p91.aux1_SET(-1.8708138E38F);
		CommunicationChannel.instance.send(p91);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_HIL_RC_INPUTS_RAW.add((src, ph, pack) ->
		{
			assert (pack.chan9_raw_GET() == (char) 26673);
			assert (pack.rssi_GET() == (char) 25);
			assert (pack.chan10_raw_GET() == (char) 15064);
			assert (pack.chan8_raw_GET() == (char) 41050);
			assert (pack.chan7_raw_GET() == (char) 21081);
			assert (pack.chan4_raw_GET() == (char) 7537);
			assert (pack.chan6_raw_GET() == (char) 3958);
			assert (pack.chan2_raw_GET() == (char) 61867);
			assert (pack.chan11_raw_GET() == (char) 25467);
			assert (pack.time_usec_GET() == 9185392942740196033L);
			assert (pack.chan1_raw_GET() == (char) 41101);
			assert (pack.chan3_raw_GET() == (char) 9258);
			assert (pack.chan5_raw_GET() == (char) 49228);
			assert (pack.chan12_raw_GET() == (char) 20117);
		});
		GroundControl.HIL_RC_INPUTS_RAW p92 = CommunicationChannel.new_HIL_RC_INPUTS_RAW();
		PH.setPack(p92);
		p92.chan9_raw_SET((char) 26673);
		p92.time_usec_SET(9185392942740196033L);
		p92.chan5_raw_SET((char) 49228);
		p92.chan11_raw_SET((char) 25467);
		p92.chan7_raw_SET((char) 21081);
		p92.rssi_SET((char) 25);
		p92.chan6_raw_SET((char) 3958);
		p92.chan2_raw_SET((char) 61867);
		p92.chan3_raw_SET((char) 9258);
		p92.chan10_raw_SET((char) 15064);
		p92.chan8_raw_SET((char) 41050);
		p92.chan12_raw_SET((char) 20117);
		p92.chan4_raw_SET((char) 7537);
		p92.chan1_raw_SET((char) 41101);
		CommunicationChannel.instance.send(p92);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_HIL_ACTUATOR_CONTROLS.add((src, ph, pack) ->
		{
			assert (pack.time_usec_GET() == 3358857989578970814L);
			assert (pack.mode_GET() == MAV_MODE.MAV_MODE_AUTO_ARMED);
			assert (pack.flags_GET() == 3734469709664235767L);
			assert (Arrays.equals(pack.controls_GET(), new float[]{-1.1665819E38F, -2.5629924E38F, 6.601214E37F, -1.0479751E38F, -1.8586025E38F, -9.306765E37F, -2.5410229E38F, 3.3255627E38F, -1.4536719E38F, 1.5192356E38F, 1.585327E38F, -2.7332105E38F, -2.161221E38F, -1.843401E36F, 1.8615285E38F, -2.582952E38F}));
		});
		GroundControl.HIL_ACTUATOR_CONTROLS p93 = CommunicationChannel.new_HIL_ACTUATOR_CONTROLS();
		PH.setPack(p93);
		p93.time_usec_SET(3358857989578970814L);
		p93.controls_SET(new float[]{-1.1665819E38F, -2.5629924E38F, 6.601214E37F, -1.0479751E38F, -1.8586025E38F, -9.306765E37F, -2.5410229E38F, 3.3255627E38F, -1.4536719E38F, 1.5192356E38F, 1.585327E38F, -2.7332105E38F, -2.161221E38F, -1.843401E36F, 1.8615285E38F, -2.582952E38F}, 0);
		p93.mode_SET(MAV_MODE.MAV_MODE_AUTO_ARMED);
		p93.flags_SET(3734469709664235767L);
		CommunicationChannel.instance.send(p93);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_OPTICAL_FLOW.add((src, ph, pack) ->
		{
			assert (pack.quality_GET() == (char) 129);
			assert (pack.flow_comp_m_y_GET() == -2.0845818E38F);
			assert (pack.flow_rate_x_TRY(ph) == -1.2609741E38F);
			assert (pack.flow_comp_m_x_GET() == 1.0972997E38F);
			assert (pack.ground_distance_GET() == 3.2759467E38F);
			assert (pack.flow_y_GET() == (short) -5247);
			assert (pack.time_usec_GET() == 8984197436842824662L);
			assert (pack.flow_x_GET() == (short) -13196);
			assert (pack.sensor_id_GET() == (char) 229);
			assert (pack.flow_rate_y_TRY(ph) == -3.0795274E37F);
		});
		GroundControl.OPTICAL_FLOW p100 = CommunicationChannel.new_OPTICAL_FLOW();
		PH.setPack(p100);
		p100.flow_x_SET((short) -13196);
		p100.flow_y_SET((short) -5247);
		p100.flow_rate_x_SET(-1.2609741E38F, PH);
		p100.ground_distance_SET(3.2759467E38F);
		p100.sensor_id_SET((char) 229);
		p100.flow_comp_m_y_SET(-2.0845818E38F);
		p100.quality_SET((char) 129);
		p100.flow_rate_y_SET(-3.0795274E37F, PH);
		p100.flow_comp_m_x_SET(1.0972997E38F);
		p100.time_usec_SET(8984197436842824662L);
		CommunicationChannel.instance.send(p100);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_GLOBAL_VISION_POSITION_ESTIMATE.add((src, ph, pack) ->
		{
			assert (pack.y_GET() == 1.2494991E38F);
			assert (pack.z_GET() == 8.824434E37F);
			assert (pack.x_GET() == -1.3568704E38F);
			assert (pack.yaw_GET() == -9.149153E37F);
			assert (pack.pitch_GET() == 2.4247905E38F);
			assert (pack.usec_GET() == 5331375614182354895L);
			assert (pack.roll_GET() == 6.893477E37F);
		});
		GroundControl.GLOBAL_VISION_POSITION_ESTIMATE p101 = CommunicationChannel.new_GLOBAL_VISION_POSITION_ESTIMATE();
		PH.setPack(p101);
		p101.roll_SET(6.893477E37F);
		p101.pitch_SET(2.4247905E38F);
		p101.y_SET(1.2494991E38F);
		p101.z_SET(8.824434E37F);
		p101.yaw_SET(-9.149153E37F);
		p101.x_SET(-1.3568704E38F);
		p101.usec_SET(5331375614182354895L);
		CommunicationChannel.instance.send(p101);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_VISION_POSITION_ESTIMATE.add((src, ph, pack) ->
		{
			assert (pack.yaw_GET() == 1.8670429E38F);
			assert (pack.roll_GET() == 2.5611266E38F);
			assert (pack.x_GET() == 3.2401823E38F);
			assert (pack.pitch_GET() == 6.3763074E37F);
			assert (pack.y_GET() == 3.098843E38F);
			assert (pack.usec_GET() == 866679238565267518L);
			assert (pack.z_GET() == -8.2202984E37F);
		});
		GroundControl.VISION_POSITION_ESTIMATE p102 = CommunicationChannel.new_VISION_POSITION_ESTIMATE();
		PH.setPack(p102);
		p102.yaw_SET(1.8670429E38F);
		p102.usec_SET(866679238565267518L);
		p102.z_SET(-8.2202984E37F);
		p102.x_SET(3.2401823E38F);
		p102.y_SET(3.098843E38F);
		p102.pitch_SET(6.3763074E37F);
		p102.roll_SET(2.5611266E38F);
		CommunicationChannel.instance.send(p102);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_VISION_SPEED_ESTIMATE.add((src, ph, pack) ->
		{
			assert (pack.y_GET() == -6.0816476E37F);
			assert (pack.usec_GET() == 5836558372646860018L);
			assert (pack.x_GET() == -1.5543297E38F);
			assert (pack.z_GET() == 1.5138228E38F);
		});
		GroundControl.VISION_SPEED_ESTIMATE p103 = CommunicationChannel.new_VISION_SPEED_ESTIMATE();
		PH.setPack(p103);
		p103.z_SET(1.5138228E38F);
		p103.y_SET(-6.0816476E37F);
		p103.x_SET(-1.5543297E38F);
		p103.usec_SET(5836558372646860018L);
		CommunicationChannel.instance.send(p103);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_VICON_POSITION_ESTIMATE.add((src, ph, pack) ->
		{
			assert (pack.z_GET() == 2.582777E38F);
			assert (pack.pitch_GET() == -2.3671255E38F);
			assert (pack.usec_GET() == 8396005348144972646L);
			assert (pack.x_GET() == -2.0133983E38F);
			assert (pack.roll_GET() == 8.892919E37F);
			assert (pack.y_GET() == -2.7255295E38F);
			assert (pack.yaw_GET() == 2.1153344E37F);
		});
		GroundControl.VICON_POSITION_ESTIMATE p104 = CommunicationChannel.new_VICON_POSITION_ESTIMATE();
		PH.setPack(p104);
		p104.roll_SET(8.892919E37F);
		p104.z_SET(2.582777E38F);
		p104.y_SET(-2.7255295E38F);
		p104.usec_SET(8396005348144972646L);
		p104.yaw_SET(2.1153344E37F);
		p104.pitch_SET(-2.3671255E38F);
		p104.x_SET(-2.0133983E38F);
		CommunicationChannel.instance.send(p104);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_HIGHRES_IMU.add((src, ph, pack) ->
		{
			assert (pack.pressure_alt_GET() == -3.0434096E38F);
			assert (pack.xmag_GET() == -2.6582296E38F);
			assert (pack.time_usec_GET() == 7593205028592425069L);
			assert (pack.zacc_GET() == 3.12101E37F);
			assert (pack.diff_pressure_GET() == 1.0880629E38F);
			assert (pack.ygyro_GET() == -8.2083277E37F);
			assert (pack.yacc_GET() == -5.496125E37F);
			assert (pack.zmag_GET() == -1.2149571E38F);
			assert (pack.xacc_GET() == 1.8323784E38F);
			assert (pack.abs_pressure_GET() == 2.7539198E38F);
			assert (pack.xgyro_GET() == 3.2033182E38F);
			assert (pack.fields_updated_GET() == (char) 59955);
			assert (pack.temperature_GET() == -2.8303817E38F);
			assert (pack.ymag_GET() == 8.390603E37F);
			assert (pack.zgyro_GET() == 8.928675E37F);
		});
		GroundControl.HIGHRES_IMU p105 = CommunicationChannel.new_HIGHRES_IMU();
		PH.setPack(p105);
		p105.fields_updated_SET((char) 59955);
		p105.ygyro_SET(-8.2083277E37F);
		p105.diff_pressure_SET(1.0880629E38F);
		p105.abs_pressure_SET(2.7539198E38F);
		p105.xmag_SET(-2.6582296E38F);
		p105.temperature_SET(-2.8303817E38F);
		p105.time_usec_SET(7593205028592425069L);
		p105.xgyro_SET(3.2033182E38F);
		p105.zmag_SET(-1.2149571E38F);
		p105.zacc_SET(3.12101E37F);
		p105.xacc_SET(1.8323784E38F);
		p105.zgyro_SET(8.928675E37F);
		p105.ymag_SET(8.390603E37F);
		p105.yacc_SET(-5.496125E37F);
		p105.pressure_alt_SET(-3.0434096E38F);
		CommunicationChannel.instance.send(p105);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_OPTICAL_FLOW_RAD.add((src, ph, pack) ->
		{
			assert (pack.integrated_zgyro_GET() == -2.4460653E38F);
			assert (pack.quality_GET() == (char) 192);
			assert (pack.distance_GET() == -1.9727083E38F);
			assert (pack.time_usec_GET() == 5136003217090946389L);
			assert (pack.sensor_id_GET() == (char) 129);
			assert (pack.temperature_GET() == (short) -22686);
			assert (pack.integrated_x_GET() == -1.2912398E37F);
			assert (pack.time_delta_distance_us_GET() == 969319599L);
			assert (pack.integrated_xgyro_GET() == -1.0732576E37F);
			assert (pack.integrated_ygyro_GET() == -3.2841194E38F);
			assert (pack.integrated_y_GET() == -2.0242555E38F);
			assert (pack.integration_time_us_GET() == 1666712886L);
		});
		GroundControl.OPTICAL_FLOW_RAD p106 = CommunicationChannel.new_OPTICAL_FLOW_RAD();
		PH.setPack(p106);
		p106.integrated_ygyro_SET(-3.2841194E38F);
		p106.quality_SET((char) 192);
		p106.integrated_y_SET(-2.0242555E38F);
		p106.distance_SET(-1.9727083E38F);
		p106.temperature_SET((short) -22686);
		p106.integration_time_us_SET(1666712886L);
		p106.integrated_xgyro_SET(-1.0732576E37F);
		p106.time_delta_distance_us_SET(969319599L);
		p106.integrated_zgyro_SET(-2.4460653E38F);
		p106.sensor_id_SET((char) 129);
		p106.time_usec_SET(5136003217090946389L);
		p106.integrated_x_SET(-1.2912398E37F);
		CommunicationChannel.instance.send(p106);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_HIL_SENSOR.add((src, ph, pack) ->
		{
			assert (pack.yacc_GET() == -1.7858964E38F);
			assert (pack.xmag_GET() == -1.8375595E38F);
			assert (pack.pressure_alt_GET() == 3.0846655E38F);
			assert (pack.zacc_GET() == -3.091646E38F);
			assert (pack.ymag_GET() == 3.1728613E38F);
			assert (pack.xacc_GET() == 1.645229E38F);
			assert (pack.zmag_GET() == -3.0634265E38F);
			assert (pack.temperature_GET() == -2.645082E38F);
			assert (pack.ygyro_GET() == -2.6892623E38F);
			assert (pack.abs_pressure_GET() == 9.245217E37F);
			assert (pack.fields_updated_GET() == 455900941L);
			assert (pack.zgyro_GET() == -3.2996826E37F);
			assert (pack.diff_pressure_GET() == 2.953094E38F);
			assert (pack.time_usec_GET() == 8881958738076397815L);
			assert (pack.xgyro_GET() == 1.7113604E38F);
		});
		GroundControl.HIL_SENSOR p107 = CommunicationChannel.new_HIL_SENSOR();
		PH.setPack(p107);
		p107.abs_pressure_SET(9.245217E37F);
		p107.zgyro_SET(-3.2996826E37F);
		p107.zmag_SET(-3.0634265E38F);
		p107.ymag_SET(3.1728613E38F);
		p107.temperature_SET(-2.645082E38F);
		p107.time_usec_SET(8881958738076397815L);
		p107.yacc_SET(-1.7858964E38F);
		p107.xmag_SET(-1.8375595E38F);
		p107.zacc_SET(-3.091646E38F);
		p107.pressure_alt_SET(3.0846655E38F);
		p107.xgyro_SET(1.7113604E38F);
		p107.fields_updated_SET(455900941L);
		p107.ygyro_SET(-2.6892623E38F);
		p107.diff_pressure_SET(2.953094E38F);
		p107.xacc_SET(1.645229E38F);
		CommunicationChannel.instance.send(p107);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_SIM_STATE.add((src, ph, pack) ->
		{
			assert (pack.ygyro_GET() == -1.6186556E38F);
			assert (pack.q3_GET() == 2.6494707E38F);
			assert (pack.yacc_GET() == -1.9102677E38F);
			assert (pack.lon_GET() == 1.2489321E38F);
			assert (pack.zacc_GET() == -1.2943665E38F);
			assert (pack.q1_GET() == 2.4750407E38F);
			assert (pack.std_dev_horz_GET() == -2.0966604E38F);
			assert (pack.zgyro_GET() == 1.0294303E38F);
			assert (pack.lat_GET() == -5.9329253E37F);
			assert (pack.ve_GET() == 2.1919543E38F);
			assert (pack.xgyro_GET() == 1.979753E38F);
			assert (pack.xacc_GET() == 1.6334297E38F);
			assert (pack.vd_GET() == -2.6556081E38F);
			assert (pack.q4_GET() == -3.905656E37F);
			assert (pack.roll_GET() == -3.0109302E38F);
			assert (pack.vn_GET() == 1.2114653E38F);
			assert (pack.std_dev_vert_GET() == 3.3748074E38F);
			assert (pack.pitch_GET() == 2.3423331E38F);
			assert (pack.q2_GET() == -9.743204E37F);
			assert (pack.alt_GET() == -2.6900026E38F);
			assert (pack.yaw_GET() == -2.039966E38F);
		});
		GroundControl.SIM_STATE p108 = CommunicationChannel.new_SIM_STATE();
		PH.setPack(p108);
		p108.q2_SET(-9.743204E37F);
		p108.vd_SET(-2.6556081E38F);
		p108.std_dev_horz_SET(-2.0966604E38F);
		p108.lon_SET(1.2489321E38F);
		p108.xgyro_SET(1.979753E38F);
		p108.yaw_SET(-2.039966E38F);
		p108.zgyro_SET(1.0294303E38F);
		p108.alt_SET(-2.6900026E38F);
		p108.q4_SET(-3.905656E37F);
		p108.pitch_SET(2.3423331E38F);
		p108.q1_SET(2.4750407E38F);
		p108.xacc_SET(1.6334297E38F);
		p108.zacc_SET(-1.2943665E38F);
		p108.q3_SET(2.6494707E38F);
		p108.std_dev_vert_SET(3.3748074E38F);
		p108.yacc_SET(-1.9102677E38F);
		p108.ygyro_SET(-1.6186556E38F);
		p108.roll_SET(-3.0109302E38F);
		p108.lat_SET(-5.9329253E37F);
		p108.ve_SET(2.1919543E38F);
		p108.vn_SET(1.2114653E38F);
		CommunicationChannel.instance.send(p108);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_RADIO_STATUS.add((src, ph, pack) ->
		{
			assert (pack.rssi_GET() == (char) 158);
			assert (pack.remrssi_GET() == (char) 239);
			assert (pack.noise_GET() == (char) 230);
			assert (pack.remnoise_GET() == (char) 95);
			assert (pack.txbuf_GET() == (char) 127);
			assert (pack.fixed__GET() == (char) 24201);
			assert (pack.rxerrors_GET() == (char) 28858);
		});
		GroundControl.RADIO_STATUS p109 = CommunicationChannel.new_RADIO_STATUS();
		PH.setPack(p109);
		p109.noise_SET((char) 230);
		p109.remrssi_SET((char) 239);
		p109.fixed__SET((char) 24201);
		p109.txbuf_SET((char) 127);
		p109.rssi_SET((char) 158);
		p109.rxerrors_SET((char) 28858);
		p109.remnoise_SET((char) 95);
		CommunicationChannel.instance.send(p109);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_FILE_TRANSFER_PROTOCOL.add((src, ph, pack) ->
		{
			assert (pack.target_component_GET() == (char) 125);
			assert (pack.target_network_GET() == (char) 43);
			assert (pack.target_system_GET() == (char) 222);
			assert (Arrays.equals(pack.payload_GET(), new char[]{(char) 79, (char) 19, (char) 66, (char) 56, (char) 206, (char) 81, (char) 97, (char) 195, (char) 69, (char) 7, (char) 142, (char) 11, (char) 129, (char) 78, (char) 28, (char) 170, (char) 0, (char) 136, (char) 9, (char) 83, (char) 153, (char) 63, (char) 224, (char) 210, (char) 163, (char) 99, (char) 7, (char) 106, (char) 71, (char) 28, (char) 72, (char) 244, (char) 45, (char) 224, (char) 85, (char) 103, (char) 66, (char) 97, (char) 172, (char) 123, (char) 138, (char) 149, (char) 153, (char) 241, (char) 128, (char) 106, (char) 251, (char) 144, (char) 97, (char) 176, (char) 91, (char) 59, (char) 52, (char) 102, (char) 86, (char) 125, (char) 235, (char) 23, (char) 184, (char) 44, (char) 9, (char) 96, (char) 194, (char) 221, (char) 100, (char) 84, (char) 25, (char) 161, (char) 200, (char) 49, (char) 206, (char) 218, (char) 105, (char) 137, (char) 117, (char) 65, (char) 185, (char) 179, (char) 164, (char) 229, (char) 78, (char) 224, (char) 161, (char) 199, (char) 97, (char) 119, (char) 43, (char) 251, (char) 150, (char) 204, (char) 109, (char) 192, (char) 102, (char) 215, (char) 119, (char) 88, (char) 190, (char) 173, (char) 144, (char) 92, (char) 160, (char) 249, (char) 21, (char) 245, (char) 146, (char) 56, (char) 132, (char) 140, (char) 33, (char) 23, (char) 156, (char) 16, (char) 169, (char) 214, (char) 231, (char) 29, (char) 38, (char) 243, (char) 158, (char) 113, (char) 192, (char) 88, (char) 45, (char) 13, (char) 226, (char) 14, (char) 15, (char) 103, (char) 22, (char) 29, (char) 215, (char) 25, (char) 201, (char) 74, (char) 139, (char) 21, (char) 240, (char) 12, (char) 188, (char) 95, (char) 77, (char) 251, (char) 40, (char) 153, (char) 49, (char) 103, (char) 74, (char) 17, (char) 133, (char) 242, (char) 96, (char) 32, (char) 145, (char) 148, (char) 18, (char) 161, (char) 103, (char) 154, (char) 30, (char) 194, (char) 132, (char) 230, (char) 232, (char) 226, (char) 138, (char) 80, (char) 253, (char) 38, (char) 185, (char) 11, (char) 215, (char) 210, (char) 132, (char) 167, (char) 23, (char) 189, (char) 57, (char) 124, (char) 56, (char) 134, (char) 142, (char) 70, (char) 44, (char) 217, (char) 156, (char) 217, (char) 37, (char) 254, (char) 214, (char) 117, (char) 167, (char) 152, (char) 183, (char) 158, (char) 56, (char) 19, (char) 70, (char) 161, (char) 255, (char) 108, (char) 170, (char) 194, (char) 236, (char) 19, (char) 247, (char) 57, (char) 89, (char) 52, (char) 159, (char) 99, (char) 173, (char) 210, (char) 11, (char) 66, (char) 118, (char) 150, (char) 58, (char) 228, (char) 106, (char) 122, (char) 164, (char) 93, (char) 101, (char) 87, (char) 167, (char) 13, (char) 135, (char) 45, (char) 125, (char) 200, (char) 38, (char) 69, (char) 244, (char) 72, (char) 146, (char) 130, (char) 171, (char) 123, (char) 67, (char) 164, (char) 185, (char) 124, (char) 110, (char) 187, (char) 195, (char) 189, (char) 107, (char) 45, (char) 182, (char) 138, (char) 251}));
		});
		GroundControl.FILE_TRANSFER_PROTOCOL p110 = CommunicationChannel.new_FILE_TRANSFER_PROTOCOL();
		PH.setPack(p110);
		p110.payload_SET(new char[]{(char) 79, (char) 19, (char) 66, (char) 56, (char) 206, (char) 81, (char) 97, (char) 195, (char) 69, (char) 7, (char) 142, (char) 11, (char) 129, (char) 78, (char) 28, (char) 170, (char) 0, (char) 136, (char) 9, (char) 83, (char) 153, (char) 63, (char) 224, (char) 210, (char) 163, (char) 99, (char) 7, (char) 106, (char) 71, (char) 28, (char) 72, (char) 244, (char) 45, (char) 224, (char) 85, (char) 103, (char) 66, (char) 97, (char) 172, (char) 123, (char) 138, (char) 149, (char) 153, (char) 241, (char) 128, (char) 106, (char) 251, (char) 144, (char) 97, (char) 176, (char) 91, (char) 59, (char) 52, (char) 102, (char) 86, (char) 125, (char) 235, (char) 23, (char) 184, (char) 44, (char) 9, (char) 96, (char) 194, (char) 221, (char) 100, (char) 84, (char) 25, (char) 161, (char) 200, (char) 49, (char) 206, (char) 218, (char) 105, (char) 137, (char) 117, (char) 65, (char) 185, (char) 179, (char) 164, (char) 229, (char) 78, (char) 224, (char) 161, (char) 199, (char) 97, (char) 119, (char) 43, (char) 251, (char) 150, (char) 204, (char) 109, (char) 192, (char) 102, (char) 215, (char) 119, (char) 88, (char) 190, (char) 173, (char) 144, (char) 92, (char) 160, (char) 249, (char) 21, (char) 245, (char) 146, (char) 56, (char) 132, (char) 140, (char) 33, (char) 23, (char) 156, (char) 16, (char) 169, (char) 214, (char) 231, (char) 29, (char) 38, (char) 243, (char) 158, (char) 113, (char) 192, (char) 88, (char) 45, (char) 13, (char) 226, (char) 14, (char) 15, (char) 103, (char) 22, (char) 29, (char) 215, (char) 25, (char) 201, (char) 74, (char) 139, (char) 21, (char) 240, (char) 12, (char) 188, (char) 95, (char) 77, (char) 251, (char) 40, (char) 153, (char) 49, (char) 103, (char) 74, (char) 17, (char) 133, (char) 242, (char) 96, (char) 32, (char) 145, (char) 148, (char) 18, (char) 161, (char) 103, (char) 154, (char) 30, (char) 194, (char) 132, (char) 230, (char) 232, (char) 226, (char) 138, (char) 80, (char) 253, (char) 38, (char) 185, (char) 11, (char) 215, (char) 210, (char) 132, (char) 167, (char) 23, (char) 189, (char) 57, (char) 124, (char) 56, (char) 134, (char) 142, (char) 70, (char) 44, (char) 217, (char) 156, (char) 217, (char) 37, (char) 254, (char) 214, (char) 117, (char) 167, (char) 152, (char) 183, (char) 158, (char) 56, (char) 19, (char) 70, (char) 161, (char) 255, (char) 108, (char) 170, (char) 194, (char) 236, (char) 19, (char) 247, (char) 57, (char) 89, (char) 52, (char) 159, (char) 99, (char) 173, (char) 210, (char) 11, (char) 66, (char) 118, (char) 150, (char) 58, (char) 228, (char) 106, (char) 122, (char) 164, (char) 93, (char) 101, (char) 87, (char) 167, (char) 13, (char) 135, (char) 45, (char) 125, (char) 200, (char) 38, (char) 69, (char) 244, (char) 72, (char) 146, (char) 130, (char) 171, (char) 123, (char) 67, (char) 164, (char) 185, (char) 124, (char) 110, (char) 187, (char) 195, (char) 189, (char) 107, (char) 45, (char) 182, (char) 138, (char) 251}, 0);
		p110.target_component_SET((char) 125);
		p110.target_system_SET((char) 222);
		p110.target_network_SET((char) 43);
		CommunicationChannel.instance.send(p110);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_TIMESYNC.add((src, ph, pack) ->
		{
			assert (pack.ts1_GET() == 4925556516287861001L);
			assert (pack.tc1_GET() == 6281258150830537460L);
		});
		GroundControl.TIMESYNC p111 = CommunicationChannel.new_TIMESYNC();
		PH.setPack(p111);
		p111.ts1_SET(4925556516287861001L);
		p111.tc1_SET(6281258150830537460L);
		CommunicationChannel.instance.send(p111);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_CAMERA_TRIGGER.add((src, ph, pack) ->
		{
			assert (pack.seq_GET() == 2639935985L);
			assert (pack.time_usec_GET() == 3419091475969420119L);
		});
		GroundControl.CAMERA_TRIGGER p112 = CommunicationChannel.new_CAMERA_TRIGGER();
		PH.setPack(p112);
		p112.time_usec_SET(3419091475969420119L);
		p112.seq_SET(2639935985L);
		CommunicationChannel.instance.send(p112);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_HIL_GPS.add((src, ph, pack) ->
		{
			assert (pack.vd_GET() == (short) 26338);
			assert (pack.alt_GET() == -1653329254);
			assert (pack.eph_GET() == (char) 55900);
			assert (pack.ve_GET() == (short) -24508);
			assert (pack.vel_GET() == (char) 27658);
			assert (pack.cog_GET() == (char) 33882);
			assert (pack.epv_GET() == (char) 33464);
			assert (pack.lat_GET() == -2095507266);
			assert (pack.time_usec_GET() == 7715923354308675311L);
			assert (pack.vn_GET() == (short) -32570);
			assert (pack.lon_GET() == 1306441387);
			assert (pack.fix_type_GET() == (char) 228);
			assert (pack.satellites_visible_GET() == (char) 46);
		});
		GroundControl.HIL_GPS p113 = CommunicationChannel.new_HIL_GPS();
		PH.setPack(p113);
		p113.vn_SET((short) -32570);
		p113.vel_SET((char) 27658);
		p113.vd_SET((short) 26338);
		p113.epv_SET((char) 33464);
		p113.ve_SET((short) -24508);
		p113.time_usec_SET(7715923354308675311L);
		p113.alt_SET(-1653329254);
		p113.fix_type_SET((char) 228);
		p113.eph_SET((char) 55900);
		p113.lon_SET(1306441387);
		p113.cog_SET((char) 33882);
		p113.lat_SET(-2095507266);
		p113.satellites_visible_SET((char) 46);
		CommunicationChannel.instance.send(p113);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_HIL_OPTICAL_FLOW.add((src, ph, pack) ->
		{
			assert (pack.integrated_zgyro_GET() == 1.183198E38F);
			assert (pack.integrated_xgyro_GET() == -3.2212756E38F);
			assert (pack.quality_GET() == (char) 65);
			assert (pack.integrated_y_GET() == -1.2323088E38F);
			assert (pack.integrated_ygyro_GET() == 2.4903148E38F);
			assert (pack.sensor_id_GET() == (char) 62);
			assert (pack.time_usec_GET() == 7130862691938083770L);
			assert (pack.integration_time_us_GET() == 2216367584L);
			assert (pack.distance_GET() == -1.2947346E38F);
			assert (pack.temperature_GET() == (short) -19823);
			assert (pack.time_delta_distance_us_GET() == 3913708659L);
			assert (pack.integrated_x_GET() == 1.5098882E38F);
		});
		GroundControl.HIL_OPTICAL_FLOW p114 = CommunicationChannel.new_HIL_OPTICAL_FLOW();
		PH.setPack(p114);
		p114.integrated_zgyro_SET(1.183198E38F);
		p114.sensor_id_SET((char) 62);
		p114.integrated_ygyro_SET(2.4903148E38F);
		p114.temperature_SET((short) -19823);
		p114.integrated_x_SET(1.5098882E38F);
		p114.integration_time_us_SET(2216367584L);
		p114.integrated_y_SET(-1.2323088E38F);
		p114.integrated_xgyro_SET(-3.2212756E38F);
		p114.quality_SET((char) 65);
		p114.distance_SET(-1.2947346E38F);
		p114.time_usec_SET(7130862691938083770L);
		p114.time_delta_distance_us_SET(3913708659L);
		CommunicationChannel.instance.send(p114);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_HIL_STATE_QUATERNION.add((src, ph, pack) ->
		{
			assert (pack.lat_GET() == 61356217);
			assert (pack.vy_GET() == (short) 12365);
			assert (pack.yacc_GET() == (short) -573);
			assert (pack.time_usec_GET() == 8924525744158221631L);
			assert (pack.true_airspeed_GET() == (char) 19623);
			assert (Arrays.equals(pack.attitude_quaternion_GET(), new float[]{3.0365513E38F, 1.046539E38F, 5.6686303E37F, -2.6811216E38F}));
			assert (pack.rollspeed_GET() == 9.111899E37F);
			assert (pack.vx_GET() == (short) -31107);
			assert (pack.vz_GET() == (short) -25122);
			assert (pack.alt_GET() == 217143765);
			assert (pack.yawspeed_GET() == -2.8724182E38F);
			assert (pack.xacc_GET() == (short) 737);
			assert (pack.lon_GET() == 2046029027);
			assert (pack.pitchspeed_GET() == -2.0081089E37F);
			assert (pack.zacc_GET() == (short) 15717);
			assert (pack.ind_airspeed_GET() == (char) 20281);
		});
		GroundControl.HIL_STATE_QUATERNION p115 = CommunicationChannel.new_HIL_STATE_QUATERNION();
		PH.setPack(p115);
		p115.vx_SET((short) -31107);
		p115.zacc_SET((short) 15717);
		p115.vz_SET((short) -25122);
		p115.true_airspeed_SET((char) 19623);
		p115.yawspeed_SET(-2.8724182E38F);
		p115.xacc_SET((short) 737);
		p115.ind_airspeed_SET((char) 20281);
		p115.yacc_SET((short) -573);
		p115.pitchspeed_SET(-2.0081089E37F);
		p115.lat_SET(61356217);
		p115.rollspeed_SET(9.111899E37F);
		p115.vy_SET((short) 12365);
		p115.alt_SET(217143765);
		p115.time_usec_SET(8924525744158221631L);
		p115.attitude_quaternion_SET(new float[]{3.0365513E38F, 1.046539E38F, 5.6686303E37F, -2.6811216E38F}, 0);
		p115.lon_SET(2046029027);
		CommunicationChannel.instance.send(p115);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_SCALED_IMU2.add((src, ph, pack) ->
		{
			assert (pack.yacc_GET() == (short) 23036);
			assert (pack.ymag_GET() == (short) -9801);
			assert (pack.zacc_GET() == (short) -26968);
			assert (pack.xmag_GET() == (short) -32423);
			assert (pack.time_boot_ms_GET() == 3431151896L);
			assert (pack.zmag_GET() == (short) -10896);
			assert (pack.xacc_GET() == (short) -28791);
			assert (pack.zgyro_GET() == (short) 21143);
			assert (pack.ygyro_GET() == (short) 1807);
			assert (pack.xgyro_GET() == (short) 24427);
		});
		GroundControl.SCALED_IMU2 p116 = CommunicationChannel.new_SCALED_IMU2();
		PH.setPack(p116);
		p116.zmag_SET((short) -10896);
		p116.ygyro_SET((short) 1807);
		p116.zgyro_SET((short) 21143);
		p116.xmag_SET((short) -32423);
		p116.ymag_SET((short) -9801);
		p116.zacc_SET((short) -26968);
		p116.xacc_SET((short) -28791);
		p116.time_boot_ms_SET(3431151896L);
		p116.yacc_SET((short) 23036);
		p116.xgyro_SET((short) 24427);
		CommunicationChannel.instance.send(p116);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_LOG_REQUEST_LIST.add((src, ph, pack) ->
		{
			assert (pack.end_GET() == (char) 10229);
			assert (pack.target_component_GET() == (char) 229);
			assert (pack.start_GET() == (char) 33222);
			assert (pack.target_system_GET() == (char) 59);
		});
		GroundControl.LOG_REQUEST_LIST p117 = CommunicationChannel.new_LOG_REQUEST_LIST();
		PH.setPack(p117);
		p117.target_system_SET((char) 59);
		p117.end_SET((char) 10229);
		p117.start_SET((char) 33222);
		p117.target_component_SET((char) 229);
		CommunicationChannel.instance.send(p117);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_LOG_ENTRY.add((src, ph, pack) ->
		{
			assert (pack.last_log_num_GET() == (char) 3818);
			assert (pack.size_GET() == 1879743077L);
			assert (pack.time_utc_GET() == 1790829373L);
			assert (pack.id_GET() == (char) 53549);
			assert (pack.num_logs_GET() == (char) 15211);
		});
		GroundControl.LOG_ENTRY p118 = CommunicationChannel.new_LOG_ENTRY();
		PH.setPack(p118);
		p118.time_utc_SET(1790829373L);
		p118.size_SET(1879743077L);
		p118.num_logs_SET((char) 15211);
		p118.id_SET((char) 53549);
		p118.last_log_num_SET((char) 3818);
		CommunicationChannel.instance.send(p118);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_LOG_REQUEST_DATA.add((src, ph, pack) ->
		{
			assert (pack.target_component_GET() == (char) 189);
			assert (pack.target_system_GET() == (char) 75);
			assert (pack.id_GET() == (char) 10933);
			assert (pack.ofs_GET() == 4033934711L);
			assert (pack.count_GET() == 2442172226L);
		});
		GroundControl.LOG_REQUEST_DATA p119 = CommunicationChannel.new_LOG_REQUEST_DATA();
		PH.setPack(p119);
		p119.target_system_SET((char) 75);
		p119.count_SET(2442172226L);
		p119.id_SET((char) 10933);
		p119.target_component_SET((char) 189);
		p119.ofs_SET(4033934711L);
		CommunicationChannel.instance.send(p119);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_LOG_DATA.add((src, ph, pack) ->
		{
			assert (pack.count_GET() == (char) 83);
			assert (pack.ofs_GET() == 1791143065L);
			assert (pack.id_GET() == (char) 35876);
			assert (Arrays.equals(pack.data__GET(), new char[]{(char) 238, (char) 59, (char) 3, (char) 72, (char) 196, (char) 151, (char) 115, (char) 153, (char) 69, (char) 46, (char) 74, (char) 214, (char) 90, (char) 139, (char) 164, (char) 25, (char) 64, (char) 9, (char) 132, (char) 24, (char) 67, (char) 12, (char) 5, (char) 245, (char) 47, (char) 137, (char) 84, (char) 52, (char) 192, (char) 152, (char) 115, (char) 209, (char) 144, (char) 233, (char) 203, (char) 250, (char) 105, (char) 94, (char) 162, (char) 213, (char) 4, (char) 98, (char) 85, (char) 64, (char) 24, (char) 190, (char) 229, (char) 92, (char) 146, (char) 67, (char) 68, (char) 21, (char) 42, (char) 147, (char) 33, (char) 201, (char) 192, (char) 65, (char) 137, (char) 152, (char) 2, (char) 106, (char) 213, (char) 244, (char) 96, (char) 80, (char) 16, (char) 86, (char) 203, (char) 150, (char) 150, (char) 156, (char) 166, (char) 76, (char) 149, (char) 72, (char) 70, (char) 103, (char) 233, (char) 82, (char) 42, (char) 95, (char) 58, (char) 104, (char) 174, (char) 85, (char) 7, (char) 43, (char) 132, (char) 105}));
		});
		GroundControl.LOG_DATA p120 = CommunicationChannel.new_LOG_DATA();
		PH.setPack(p120);
		p120.id_SET((char) 35876);
		p120.data__SET(new char[]{(char) 238, (char) 59, (char) 3, (char) 72, (char) 196, (char) 151, (char) 115, (char) 153, (char) 69, (char) 46, (char) 74, (char) 214, (char) 90, (char) 139, (char) 164, (char) 25, (char) 64, (char) 9, (char) 132, (char) 24, (char) 67, (char) 12, (char) 5, (char) 245, (char) 47, (char) 137, (char) 84, (char) 52, (char) 192, (char) 152, (char) 115, (char) 209, (char) 144, (char) 233, (char) 203, (char) 250, (char) 105, (char) 94, (char) 162, (char) 213, (char) 4, (char) 98, (char) 85, (char) 64, (char) 24, (char) 190, (char) 229, (char) 92, (char) 146, (char) 67, (char) 68, (char) 21, (char) 42, (char) 147, (char) 33, (char) 201, (char) 192, (char) 65, (char) 137, (char) 152, (char) 2, (char) 106, (char) 213, (char) 244, (char) 96, (char) 80, (char) 16, (char) 86, (char) 203, (char) 150, (char) 150, (char) 156, (char) 166, (char) 76, (char) 149, (char) 72, (char) 70, (char) 103, (char) 233, (char) 82, (char) 42, (char) 95, (char) 58, (char) 104, (char) 174, (char) 85, (char) 7, (char) 43, (char) 132, (char) 105}, 0);
		p120.ofs_SET(1791143065L);
		p120.count_SET((char) 83);
		CommunicationChannel.instance.send(p120);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_LOG_ERASE.add((src, ph, pack) ->
		{
			assert (pack.target_component_GET() == (char) 160);
			assert (pack.target_system_GET() == (char) 106);
		});
		GroundControl.LOG_ERASE p121 = CommunicationChannel.new_LOG_ERASE();
		PH.setPack(p121);
		p121.target_component_SET((char) 160);
		p121.target_system_SET((char) 106);
		CommunicationChannel.instance.send(p121);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_LOG_REQUEST_END.add((src, ph, pack) ->
		{
			assert (pack.target_system_GET() == (char) 228);
			assert (pack.target_component_GET() == (char) 49);
		});
		GroundControl.LOG_REQUEST_END p122 = CommunicationChannel.new_LOG_REQUEST_END();
		PH.setPack(p122);
		p122.target_component_SET((char) 49);
		p122.target_system_SET((char) 228);
		CommunicationChannel.instance.send(p122);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_GPS_INJECT_DATA.add((src, ph, pack) ->
		{
			assert (pack.target_component_GET() == (char) 187);
			assert (pack.target_system_GET() == (char) 184);
			assert (pack.len_GET() == (char) 216);
			assert (Arrays.equals(pack.data__GET(), new char[]{(char) 24, (char) 50, (char) 37, (char) 164, (char) 148, (char) 172, (char) 35, (char) 245, (char) 202, (char) 76, (char) 168, (char) 53, (char) 246, (char) 64, (char) 165, (char) 66, (char) 172, (char) 205, (char) 158, (char) 19, (char) 202, (char) 125, (char) 117, (char) 45, (char) 49, (char) 203, (char) 191, (char) 5, (char) 162, (char) 64, (char) 232, (char) 132, (char) 221, (char) 245, (char) 198, (char) 63, (char) 205, (char) 76, (char) 31, (char) 78, (char) 99, (char) 118, (char) 51, (char) 218, (char) 249, (char) 95, (char) 69, (char) 127, (char) 39, (char) 200, (char) 238, (char) 49, (char) 11, (char) 97, (char) 252, (char) 23, (char) 123, (char) 211, (char) 134, (char) 45, (char) 110, (char) 11, (char) 216, (char) 183, (char) 164, (char) 222, (char) 104, (char) 167, (char) 208, (char) 248, (char) 152, (char) 57, (char) 110, (char) 16, (char) 102, (char) 3, (char) 8, (char) 209, (char) 136, (char) 14, (char) 7, (char) 40, (char) 148, (char) 87, (char) 26, (char) 19, (char) 158, (char) 33, (char) 37, (char) 23, (char) 12, (char) 217, (char) 164, (char) 14, (char) 65, (char) 42, (char) 136, (char) 137, (char) 30, (char) 218, (char) 144, (char) 49, (char) 201, (char) 121, (char) 130, (char) 175, (char) 231, (char) 8, (char) 230, (char) 31}));
		});
		GroundControl.GPS_INJECT_DATA p123 = CommunicationChannel.new_GPS_INJECT_DATA();
		PH.setPack(p123);
		p123.len_SET((char) 216);
		p123.target_component_SET((char) 187);
		p123.data__SET(new char[]{(char) 24, (char) 50, (char) 37, (char) 164, (char) 148, (char) 172, (char) 35, (char) 245, (char) 202, (char) 76, (char) 168, (char) 53, (char) 246, (char) 64, (char) 165, (char) 66, (char) 172, (char) 205, (char) 158, (char) 19, (char) 202, (char) 125, (char) 117, (char) 45, (char) 49, (char) 203, (char) 191, (char) 5, (char) 162, (char) 64, (char) 232, (char) 132, (char) 221, (char) 245, (char) 198, (char) 63, (char) 205, (char) 76, (char) 31, (char) 78, (char) 99, (char) 118, (char) 51, (char) 218, (char) 249, (char) 95, (char) 69, (char) 127, (char) 39, (char) 200, (char) 238, (char) 49, (char) 11, (char) 97, (char) 252, (char) 23, (char) 123, (char) 211, (char) 134, (char) 45, (char) 110, (char) 11, (char) 216, (char) 183, (char) 164, (char) 222, (char) 104, (char) 167, (char) 208, (char) 248, (char) 152, (char) 57, (char) 110, (char) 16, (char) 102, (char) 3, (char) 8, (char) 209, (char) 136, (char) 14, (char) 7, (char) 40, (char) 148, (char) 87, (char) 26, (char) 19, (char) 158, (char) 33, (char) 37, (char) 23, (char) 12, (char) 217, (char) 164, (char) 14, (char) 65, (char) 42, (char) 136, (char) 137, (char) 30, (char) 218, (char) 144, (char) 49, (char) 201, (char) 121, (char) 130, (char) 175, (char) 231, (char) 8, (char) 230, (char) 31}, 0);
		p123.target_system_SET((char) 184);
		CommunicationChannel.instance.send(p123);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_GPS2_RAW.add((src, ph, pack) ->
		{
			assert (pack.fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_2D_FIX);
			assert (pack.dgps_numch_GET() == (char) 98);
			assert (pack.lat_GET() == -1759669637);
			assert (pack.eph_GET() == (char) 62380);
			assert (pack.satellites_visible_GET() == (char) 148);
			assert (pack.alt_GET() == 1172855011);
			assert (pack.vel_GET() == (char) 31867);
			assert (pack.time_usec_GET() == 2294117257577434118L);
			assert (pack.cog_GET() == (char) 1062);
			assert (pack.epv_GET() == (char) 65318);
			assert (pack.dgps_age_GET() == 3178176656L);
			assert (pack.lon_GET() == -36539921);
		});
		GroundControl.GPS2_RAW p124 = CommunicationChannel.new_GPS2_RAW();
		PH.setPack(p124);
		p124.eph_SET((char) 62380);
		p124.dgps_age_SET(3178176656L);
		p124.time_usec_SET(2294117257577434118L);
		p124.epv_SET((char) 65318);
		p124.dgps_numch_SET((char) 98);
		p124.alt_SET(1172855011);
		p124.cog_SET((char) 1062);
		p124.lat_SET(-1759669637);
		p124.lon_SET(-36539921);
		p124.satellites_visible_SET((char) 148);
		p124.vel_SET((char) 31867);
		p124.fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_2D_FIX);
		CommunicationChannel.instance.send(p124);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_POWER_STATUS.add((src, ph, pack) ->
		{
			assert (pack.Vservo_GET() == (char) 14322);
			assert (pack.Vcc_GET() == (char) 8935);
			assert (pack.flags_GET() == MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT);
		});
		GroundControl.POWER_STATUS p125 = CommunicationChannel.new_POWER_STATUS();
		PH.setPack(p125);
		p125.Vservo_SET((char) 14322);
		p125.Vcc_SET((char) 8935);
		p125.flags_SET(MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT);
		CommunicationChannel.instance.send(p125);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_SERIAL_CONTROL.add((src, ph, pack) ->
		{
			assert (pack.timeout_GET() == (char) 14228);
			assert (Arrays.equals(pack.data__GET(), new char[]{(char) 34, (char) 240, (char) 1, (char) 145, (char) 94, (char) 209, (char) 186, (char) 124, (char) 72, (char) 95, (char) 126, (char) 252, (char) 127, (char) 63, (char) 85, (char) 95, (char) 57, (char) 124, (char) 111, (char) 210, (char) 237, (char) 123, (char) 215, (char) 38, (char) 77, (char) 188, (char) 245, (char) 236, (char) 90, (char) 254, (char) 159, (char) 113, (char) 111, (char) 121, (char) 177, (char) 213, (char) 124, (char) 111, (char) 128, (char) 207, (char) 110, (char) 163, (char) 213, (char) 144, (char) 14, (char) 100, (char) 211, (char) 197, (char) 163, (char) 66, (char) 161, (char) 150, (char) 101, (char) 73, (char) 74, (char) 98, (char) 183, (char) 192, (char) 169, (char) 173, (char) 44, (char) 53, (char) 136, (char) 63, (char) 185, (char) 191, (char) 34, (char) 18, (char) 132, (char) 171}));
			assert (pack.device_GET() == SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_TELEM2);
			assert (pack.count_GET() == (char) 10);
			assert (pack.flags_GET() == SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_EXCLUSIVE);
			assert (pack.baudrate_GET() == 206723420L);
		});
		GroundControl.SERIAL_CONTROL p126 = CommunicationChannel.new_SERIAL_CONTROL();
		PH.setPack(p126);
		p126.flags_SET(SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_EXCLUSIVE);
		p126.device_SET(SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_TELEM2);
		p126.count_SET((char) 10);
		p126.data__SET(new char[]{(char) 34, (char) 240, (char) 1, (char) 145, (char) 94, (char) 209, (char) 186, (char) 124, (char) 72, (char) 95, (char) 126, (char) 252, (char) 127, (char) 63, (char) 85, (char) 95, (char) 57, (char) 124, (char) 111, (char) 210, (char) 237, (char) 123, (char) 215, (char) 38, (char) 77, (char) 188, (char) 245, (char) 236, (char) 90, (char) 254, (char) 159, (char) 113, (char) 111, (char) 121, (char) 177, (char) 213, (char) 124, (char) 111, (char) 128, (char) 207, (char) 110, (char) 163, (char) 213, (char) 144, (char) 14, (char) 100, (char) 211, (char) 197, (char) 163, (char) 66, (char) 161, (char) 150, (char) 101, (char) 73, (char) 74, (char) 98, (char) 183, (char) 192, (char) 169, (char) 173, (char) 44, (char) 53, (char) 136, (char) 63, (char) 185, (char) 191, (char) 34, (char) 18, (char) 132, (char) 171}, 0);
		p126.timeout_SET((char) 14228);
		p126.baudrate_SET(206723420L);
		CommunicationChannel.instance.send(p126);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_GPS_RTK.add((src, ph, pack) ->
		{
			assert (pack.baseline_b_mm_GET() == -697190762);
			assert (pack.time_last_baseline_ms_GET() == 933814800L);
			assert (pack.baseline_coords_type_GET() == (char) 73);
			assert (pack.accuracy_GET() == 3015185970L);
			assert (pack.baseline_c_mm_GET() == -1020883954);
			assert (pack.nsats_GET() == (char) 137);
			assert (pack.rtk_health_GET() == (char) 231);
			assert (pack.tow_GET() == 4153111689L);
			assert (pack.rtk_rate_GET() == (char) 77);
			assert (pack.baseline_a_mm_GET() == 1813153594);
			assert (pack.wn_GET() == (char) 64052);
			assert (pack.rtk_receiver_id_GET() == (char) 98);
			assert (pack.iar_num_hypotheses_GET() == -738674636);
		});
		GroundControl.GPS_RTK p127 = CommunicationChannel.new_GPS_RTK();
		PH.setPack(p127);
		p127.time_last_baseline_ms_SET(933814800L);
		p127.baseline_a_mm_SET(1813153594);
		p127.baseline_c_mm_SET(-1020883954);
		p127.nsats_SET((char) 137);
		p127.baseline_b_mm_SET(-697190762);
		p127.accuracy_SET(3015185970L);
		p127.rtk_health_SET((char) 231);
		p127.wn_SET((char) 64052);
		p127.baseline_coords_type_SET((char) 73);
		p127.rtk_rate_SET((char) 77);
		p127.tow_SET(4153111689L);
		p127.iar_num_hypotheses_SET(-738674636);
		p127.rtk_receiver_id_SET((char) 98);
		CommunicationChannel.instance.send(p127);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_GPS2_RTK.add((src, ph, pack) ->
		{
			assert (pack.rtk_health_GET() == (char) 255);
			assert (pack.baseline_coords_type_GET() == (char) 93);
			assert (pack.time_last_baseline_ms_GET() == 2600364118L);
			assert (pack.baseline_c_mm_GET() == -1109481785);
			assert (pack.accuracy_GET() == 4174359093L);
			assert (pack.iar_num_hypotheses_GET() == 413206928);
			assert (pack.rtk_rate_GET() == (char) 198);
			assert (pack.wn_GET() == (char) 35034);
			assert (pack.tow_GET() == 2621411952L);
			assert (pack.baseline_b_mm_GET() == 765859850);
			assert (pack.rtk_receiver_id_GET() == (char) 193);
			assert (pack.baseline_a_mm_GET() == -871463544);
			assert (pack.nsats_GET() == (char) 132);
		});
		GroundControl.GPS2_RTK p128 = CommunicationChannel.new_GPS2_RTK();
		PH.setPack(p128);
		p128.rtk_receiver_id_SET((char) 193);
		p128.wn_SET((char) 35034);
		p128.baseline_b_mm_SET(765859850);
		p128.rtk_rate_SET((char) 198);
		p128.rtk_health_SET((char) 255);
		p128.baseline_a_mm_SET(-871463544);
		p128.iar_num_hypotheses_SET(413206928);
		p128.tow_SET(2621411952L);
		p128.accuracy_SET(4174359093L);
		p128.baseline_c_mm_SET(-1109481785);
		p128.baseline_coords_type_SET((char) 93);
		p128.time_last_baseline_ms_SET(2600364118L);
		p128.nsats_SET((char) 132);
		CommunicationChannel.instance.send(p128);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_SCALED_IMU3.add((src, ph, pack) ->
		{
			assert (pack.zgyro_GET() == (short) -7311);
			assert (pack.xgyro_GET() == (short) 29637);
			assert (pack.xacc_GET() == (short) -5812);
			assert (pack.yacc_GET() == (short) -15588);
			assert (pack.time_boot_ms_GET() == 2336202933L);
			assert (pack.zmag_GET() == (short) 29104);
			assert (pack.xmag_GET() == (short) 18479);
			assert (pack.ymag_GET() == (short) -779);
			assert (pack.ygyro_GET() == (short) -5011);
			assert (pack.zacc_GET() == (short) 23054);
		});
		GroundControl.SCALED_IMU3 p129 = CommunicationChannel.new_SCALED_IMU3();
		PH.setPack(p129);
		p129.yacc_SET((short) -15588);
		p129.zmag_SET((short) 29104);
		p129.ymag_SET((short) -779);
		p129.ygyro_SET((short) -5011);
		p129.zgyro_SET((short) -7311);
		p129.xacc_SET((short) -5812);
		p129.xmag_SET((short) 18479);
		p129.xgyro_SET((short) 29637);
		p129.zacc_SET((short) 23054);
		p129.time_boot_ms_SET(2336202933L);
		CommunicationChannel.instance.send(p129);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_DATA_TRANSMISSION_HANDSHAKE.add((src, ph, pack) ->
		{
			assert (pack.width_GET() == (char) 65126);
			assert (pack.size_GET() == 2653257334L);
			assert (pack.jpg_quality_GET() == (char) 62);
			assert (pack.height_GET() == (char) 57905);
			assert (pack.type_GET() == (char) 231);
			assert (pack.payload_GET() == (char) 64);
			assert (pack.packets_GET() == (char) 20704);
		});
		GroundControl.DATA_TRANSMISSION_HANDSHAKE p130 = CommunicationChannel.new_DATA_TRANSMISSION_HANDSHAKE();
		PH.setPack(p130);
		p130.size_SET(2653257334L);
		p130.width_SET((char) 65126);
		p130.packets_SET((char) 20704);
		p130.jpg_quality_SET((char) 62);
		p130.height_SET((char) 57905);
		p130.type_SET((char) 231);
		p130.payload_SET((char) 64);
		CommunicationChannel.instance.send(p130);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_ENCAPSULATED_DATA.add((src, ph, pack) ->
		{
			assert (Arrays.equals(pack.data__GET(), new char[]{(char) 112, (char) 215, (char) 66, (char) 237, (char) 179, (char) 87, (char) 35, (char) 142, (char) 233, (char) 109, (char) 87, (char) 125, (char) 18, (char) 143, (char) 140, (char) 107, (char) 149, (char) 60, (char) 74, (char) 59, (char) 206, (char) 174, (char) 153, (char) 60, (char) 207, (char) 63, (char) 113, (char) 231, (char) 123, (char) 241, (char) 162, (char) 154, (char) 155, (char) 43, (char) 249, (char) 193, (char) 87, (char) 75, (char) 38, (char) 133, (char) 203, (char) 238, (char) 16, (char) 232, (char) 57, (char) 221, (char) 197, (char) 0, (char) 101, (char) 136, (char) 72, (char) 35, (char) 231, (char) 181, (char) 179, (char) 67, (char) 233, (char) 32, (char) 116, (char) 150, (char) 156, (char) 159, (char) 71, (char) 111, (char) 128, (char) 6, (char) 233, (char) 40, (char) 142, (char) 147, (char) 232, (char) 131, (char) 43, (char) 114, (char) 187, (char) 17, (char) 36, (char) 133, (char) 254, (char) 183, (char) 0, (char) 57, (char) 200, (char) 186, (char) 253, (char) 101, (char) 123, (char) 166, (char) 203, (char) 59, (char) 76, (char) 138, (char) 245, (char) 252, (char) 84, (char) 108, (char) 104, (char) 75, (char) 174, (char) 222, (char) 156, (char) 79, (char) 19, (char) 99, (char) 169, (char) 117, (char) 138, (char) 175, (char) 90, (char) 237, (char) 139, (char) 235, (char) 182, (char) 82, (char) 2, (char) 60, (char) 178, (char) 0, (char) 24, (char) 51, (char) 193, (char) 104, (char) 173, (char) 0, (char) 237, (char) 161, (char) 107, (char) 133, (char) 131, (char) 39, (char) 28, (char) 209, (char) 56, (char) 160, (char) 133, (char) 235, (char) 105, (char) 109, (char) 121, (char) 190, (char) 238, (char) 250, (char) 62, (char) 206, (char) 124, (char) 240, (char) 248, (char) 221, (char) 24, (char) 184, (char) 115, (char) 218, (char) 194, (char) 39, (char) 45, (char) 180, (char) 94, (char) 182, (char) 145, (char) 92, (char) 11, (char) 145, (char) 198, (char) 53, (char) 121, (char) 252, (char) 254, (char) 209, (char) 161, (char) 21, (char) 19, (char) 91, (char) 84, (char) 184, (char) 50, (char) 21, (char) 164, (char) 178, (char) 72, (char) 155, (char) 121, (char) 10, (char) 215, (char) 167, (char) 188, (char) 219, (char) 255, (char) 153, (char) 227, (char) 11, (char) 116, (char) 91, (char) 66, (char) 237, (char) 123, (char) 93, (char) 168, (char) 4, (char) 43, (char) 79, (char) 223, (char) 38, (char) 178, (char) 23, (char) 198, (char) 226, (char) 181, (char) 62, (char) 39, (char) 12, (char) 7, (char) 78, (char) 30, (char) 131, (char) 13, (char) 76, (char) 201, (char) 232, (char) 123, (char) 119, (char) 184, (char) 83, (char) 236, (char) 191, (char) 98, (char) 227, (char) 75, (char) 190, (char) 254, (char) 255, (char) 16, (char) 215, (char) 168, (char) 240, (char) 12, (char) 207, (char) 31, (char) 9, (char) 120, (char) 27, (char) 171, (char) 215, (char) 249, (char) 250, (char) 53, (char) 47, (char) 220, (char) 232, (char) 73, (char) 51, (char) 183, (char) 164, (char) 77}));
			assert (pack.seqnr_GET() == (char) 18446);
		});
		GroundControl.ENCAPSULATED_DATA p131 = CommunicationChannel.new_ENCAPSULATED_DATA();
		PH.setPack(p131);
		p131.data__SET(new char[]{(char) 112, (char) 215, (char) 66, (char) 237, (char) 179, (char) 87, (char) 35, (char) 142, (char) 233, (char) 109, (char) 87, (char) 125, (char) 18, (char) 143, (char) 140, (char) 107, (char) 149, (char) 60, (char) 74, (char) 59, (char) 206, (char) 174, (char) 153, (char) 60, (char) 207, (char) 63, (char) 113, (char) 231, (char) 123, (char) 241, (char) 162, (char) 154, (char) 155, (char) 43, (char) 249, (char) 193, (char) 87, (char) 75, (char) 38, (char) 133, (char) 203, (char) 238, (char) 16, (char) 232, (char) 57, (char) 221, (char) 197, (char) 0, (char) 101, (char) 136, (char) 72, (char) 35, (char) 231, (char) 181, (char) 179, (char) 67, (char) 233, (char) 32, (char) 116, (char) 150, (char) 156, (char) 159, (char) 71, (char) 111, (char) 128, (char) 6, (char) 233, (char) 40, (char) 142, (char) 147, (char) 232, (char) 131, (char) 43, (char) 114, (char) 187, (char) 17, (char) 36, (char) 133, (char) 254, (char) 183, (char) 0, (char) 57, (char) 200, (char) 186, (char) 253, (char) 101, (char) 123, (char) 166, (char) 203, (char) 59, (char) 76, (char) 138, (char) 245, (char) 252, (char) 84, (char) 108, (char) 104, (char) 75, (char) 174, (char) 222, (char) 156, (char) 79, (char) 19, (char) 99, (char) 169, (char) 117, (char) 138, (char) 175, (char) 90, (char) 237, (char) 139, (char) 235, (char) 182, (char) 82, (char) 2, (char) 60, (char) 178, (char) 0, (char) 24, (char) 51, (char) 193, (char) 104, (char) 173, (char) 0, (char) 237, (char) 161, (char) 107, (char) 133, (char) 131, (char) 39, (char) 28, (char) 209, (char) 56, (char) 160, (char) 133, (char) 235, (char) 105, (char) 109, (char) 121, (char) 190, (char) 238, (char) 250, (char) 62, (char) 206, (char) 124, (char) 240, (char) 248, (char) 221, (char) 24, (char) 184, (char) 115, (char) 218, (char) 194, (char) 39, (char) 45, (char) 180, (char) 94, (char) 182, (char) 145, (char) 92, (char) 11, (char) 145, (char) 198, (char) 53, (char) 121, (char) 252, (char) 254, (char) 209, (char) 161, (char) 21, (char) 19, (char) 91, (char) 84, (char) 184, (char) 50, (char) 21, (char) 164, (char) 178, (char) 72, (char) 155, (char) 121, (char) 10, (char) 215, (char) 167, (char) 188, (char) 219, (char) 255, (char) 153, (char) 227, (char) 11, (char) 116, (char) 91, (char) 66, (char) 237, (char) 123, (char) 93, (char) 168, (char) 4, (char) 43, (char) 79, (char) 223, (char) 38, (char) 178, (char) 23, (char) 198, (char) 226, (char) 181, (char) 62, (char) 39, (char) 12, (char) 7, (char) 78, (char) 30, (char) 131, (char) 13, (char) 76, (char) 201, (char) 232, (char) 123, (char) 119, (char) 184, (char) 83, (char) 236, (char) 191, (char) 98, (char) 227, (char) 75, (char) 190, (char) 254, (char) 255, (char) 16, (char) 215, (char) 168, (char) 240, (char) 12, (char) 207, (char) 31, (char) 9, (char) 120, (char) 27, (char) 171, (char) 215, (char) 249, (char) 250, (char) 53, (char) 47, (char) 220, (char) 232, (char) 73, (char) 51, (char) 183, (char) 164, (char) 77}, 0);
		p131.seqnr_SET((char) 18446);
		CommunicationChannel.instance.send(p131);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_DISTANCE_SENSOR.add((src, ph, pack) ->
		{
			assert (pack.orientation_GET() == MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_180_YAW_45);
			assert (pack.current_distance_GET() == (char) 25268);
			assert (pack.id_GET() == (char) 19);
			assert (pack.covariance_GET() == (char) 196);
			assert (pack.type_GET() == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_INFRARED);
			assert (pack.time_boot_ms_GET() == 2231821427L);
			assert (pack.max_distance_GET() == (char) 13276);
			assert (pack.min_distance_GET() == (char) 48123);
		});
		GroundControl.DISTANCE_SENSOR p132 = CommunicationChannel.new_DISTANCE_SENSOR();
		PH.setPack(p132);
		p132.type_SET(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_INFRARED);
		p132.id_SET((char) 19);
		p132.time_boot_ms_SET(2231821427L);
		p132.max_distance_SET((char) 13276);
		p132.covariance_SET((char) 196);
		p132.min_distance_SET((char) 48123);
		p132.orientation_SET(MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_180_YAW_45);
		p132.current_distance_SET((char) 25268);
		CommunicationChannel.instance.send(p132);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_TERRAIN_REQUEST.add((src, ph, pack) ->
		{
			assert (pack.grid_spacing_GET() == (char) 59918);
			assert (pack.lat_GET() == -684971529);
			assert (pack.mask_GET() == 6808445269263615966L);
			assert (pack.lon_GET() == 1975813567);
		});
		GroundControl.TERRAIN_REQUEST p133 = CommunicationChannel.new_TERRAIN_REQUEST();
		PH.setPack(p133);
		p133.lat_SET(-684971529);
		p133.grid_spacing_SET((char) 59918);
		p133.lon_SET(1975813567);
		p133.mask_SET(6808445269263615966L);
		CommunicationChannel.instance.send(p133);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_TERRAIN_DATA.add((src, ph, pack) ->
		{
			assert (Arrays.equals(pack.data__GET(), new short[]{(short) 8541, (short) -5462, (short) -23851, (short) 6504, (short) 12582, (short) -14126, (short) 30393, (short) -2928, (short) -7565, (short) 1312, (short) -2678, (short) 3633, (short) -14453, (short) -21607, (short) -14585, (short) 31536}));
			assert (pack.gridbit_GET() == (char) 140);
			assert (pack.grid_spacing_GET() == (char) 52109);
			assert (pack.lat_GET() == -2063732317);
			assert (pack.lon_GET() == 59106371);
		});
		GroundControl.TERRAIN_DATA p134 = CommunicationChannel.new_TERRAIN_DATA();
		PH.setPack(p134);
		p134.lat_SET(-2063732317);
		p134.grid_spacing_SET((char) 52109);
		p134.data__SET(new short[]{(short) 8541, (short) -5462, (short) -23851, (short) 6504, (short) 12582, (short) -14126, (short) 30393, (short) -2928, (short) -7565, (short) 1312, (short) -2678, (short) 3633, (short) -14453, (short) -21607, (short) -14585, (short) 31536}, 0);
		p134.lon_SET(59106371);
		p134.gridbit_SET((char) 140);
		CommunicationChannel.instance.send(p134);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_TERRAIN_CHECK.add((src, ph, pack) ->
		{
			assert (pack.lon_GET() == -2136815835);
			assert (pack.lat_GET() == -2075625768);
		});
		GroundControl.TERRAIN_CHECK p135 = CommunicationChannel.new_TERRAIN_CHECK();
		PH.setPack(p135);
		p135.lon_SET(-2136815835);
		p135.lat_SET(-2075625768);
		CommunicationChannel.instance.send(p135);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_TERRAIN_REPORT.add((src, ph, pack) ->
		{
			assert (pack.loaded_GET() == (char) 13076);
			assert (pack.lon_GET() == 502107025);
			assert (pack.terrain_height_GET() == 3.159792E38F);
			assert (pack.pending_GET() == (char) 10441);
			assert (pack.spacing_GET() == (char) 42380);
			assert (pack.current_height_GET() == 3.3320738E38F);
			assert (pack.lat_GET() == 876541801);
		});
		GroundControl.TERRAIN_REPORT p136 = CommunicationChannel.new_TERRAIN_REPORT();
		PH.setPack(p136);
		p136.pending_SET((char) 10441);
		p136.lat_SET(876541801);
		p136.loaded_SET((char) 13076);
		p136.lon_SET(502107025);
		p136.spacing_SET((char) 42380);
		p136.terrain_height_SET(3.159792E38F);
		p136.current_height_SET(3.3320738E38F);
		CommunicationChannel.instance.send(p136);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_SCALED_PRESSURE2.add((src, ph, pack) ->
		{
			assert (pack.press_diff_GET() == 2.5868917E38F);
			assert (pack.time_boot_ms_GET() == 2602238825L);
			assert (pack.press_abs_GET() == -1.9593793E37F);
			assert (pack.temperature_GET() == (short) -31284);
		});
		GroundControl.SCALED_PRESSURE2 p137 = CommunicationChannel.new_SCALED_PRESSURE2();
		PH.setPack(p137);
		p137.time_boot_ms_SET(2602238825L);
		p137.press_abs_SET(-1.9593793E37F);
		p137.temperature_SET((short) -31284);
		p137.press_diff_SET(2.5868917E38F);
		CommunicationChannel.instance.send(p137);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_ATT_POS_MOCAP.add((src, ph, pack) ->
		{
			assert (Arrays.equals(pack.q_GET(), new float[]{2.2480932E38F, 3.2484094E38F, 2.8652917E38F, -5.679172E37F}));
			assert (pack.z_GET() == 2.375463E38F);
			assert (pack.y_GET() == -3.2307086E38F);
			assert (pack.x_GET() == -1.1842561E38F);
			assert (pack.time_usec_GET() == 6811604252614318557L);
		});
		GroundControl.ATT_POS_MOCAP p138 = CommunicationChannel.new_ATT_POS_MOCAP();
		PH.setPack(p138);
		p138.z_SET(2.375463E38F);
		p138.y_SET(-3.2307086E38F);
		p138.time_usec_SET(6811604252614318557L);
		p138.x_SET(-1.1842561E38F);
		p138.q_SET(new float[]{2.2480932E38F, 3.2484094E38F, 2.8652917E38F, -5.679172E37F}, 0);
		CommunicationChannel.instance.send(p138);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_SET_ACTUATOR_CONTROL_TARGET.add((src, ph, pack) ->
		{
			assert (Arrays.equals(pack.controls_GET(), new float[]{9.15176E37F, -1.0718949E37F, 1.898249E38F, 1.5528723E38F, -1.338388E38F, -8.7712725E36F, -1.1526145E38F, -2.6623894E38F}));
			assert (pack.time_usec_GET() == 6821959288378779589L);
			assert (pack.target_system_GET() == (char) 118);
			assert (pack.target_component_GET() == (char) 94);
			assert (pack.group_mlx_GET() == (char) 54);
		});
		GroundControl.SET_ACTUATOR_CONTROL_TARGET p139 = CommunicationChannel.new_SET_ACTUATOR_CONTROL_TARGET();
		PH.setPack(p139);
		p139.controls_SET(new float[]{9.15176E37F, -1.0718949E37F, 1.898249E38F, 1.5528723E38F, -1.338388E38F, -8.7712725E36F, -1.1526145E38F, -2.6623894E38F}, 0);
		p139.group_mlx_SET((char) 54);
		p139.target_system_SET((char) 118);
		p139.target_component_SET((char) 94);
		p139.time_usec_SET(6821959288378779589L);
		CommunicationChannel.instance.send(p139);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		TestChannel.instance.on_ACTUATOR_CONTROL_TARGET.add((src, ph, pack) ->
		{
			assert (Arrays.equals(pack.controls_GET(), new float[]{5.3463956E36F, 3.2930532E38F, 2.9573996E38F, 3.2549937E38F, -1.2355512E37F, 1.922475E38F, -3.3627076E37F, -1.29405E37F}));
			assert (pack.time_usec_GET() == 6357937975196160938L);
			assert (pack.group_mlx_GET() == (char) 188);
		});
		GroundControl.ACTUATOR_CONTROL_TARGET p140 = CommunicationChannel.new_ACTUATOR_CONTROL_TARGET();
		PH.setPack(p140);
		p140.time_usec_SET(6357937975196160938L);
		p140.controls_SET(new float[]{5.3463956E36F, 3.2930532E38F, 2.9573996E38F, 3.2549937E38F, -1.2355512E37F, 1.922475E38F, -3.3627076E37F, -1.29405E37F}, 0);
		p140.group_mlx_SET((char) 188);
		CommunicationChannel.instance.send(p140);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
		TestChannel.instance.on_ALTITUDE.add((src, ph, pack) ->
		{
			assert (pack.altitude_terrain_GET() == -1.3868277E38F);
			assert (pack.altitude_local_GET() == 1.4046597E38F);
			assert (pack.altitude_monotonic_GET() == 8.748734E37F);
			assert (pack.time_usec_GET() == 6659688272607592945L);
			assert (pack.altitude_amsl_GET() == 2.5088213E38F);
			assert (pack.bottom_clearance_GET() == -1.4281826E37F);
			assert (pack.altitude_relative_GET() == -1.6820357E37F);
		});
		GroundControl.ALTITUDE p141 = CommunicationChannel.new_ALTITUDE();
		PH.setPack(p141);
		p141.altitude_terrain_SET(-1.3868277E38F);
		p141.bottom_clearance_SET(-1.4281826E37F);
		p141.altitude_monotonic_SET(8.748734E37F);
		p141.altitude_amsl_SET(2.5088213E38F);
		p141.altitude_local_SET(1.4046597E38F);
		p141.altitude_relative_SET(-1.6820357E37F);
		p141.time_usec_SET(6659688272607592945L);
		CommunicationChannel.instance.send(p141);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
		TestChannel.instance.on_RESOURCE_REQUEST.add((src, ph, pack) ->
		{
			assert (pack.request_id_GET() == (char) 119);
			assert (Arrays.equals(pack.storage_GET(), new char[]{(char) 150, (char) 52, (char) 93, (char) 157, (char) 68, (char) 130, (char) 150, (char) 190, (char) 137, (char) 226, (char) 131, (char) 3, (char) 247, (char) 127, (char) 221, (char) 16, (char) 28, (char) 199, (char) 230, (char) 100, (char) 228, (char) 199, (char) 170, (char) 73, (char) 96, (char) 119, (char) 3, (char) 230, (char) 86, (char) 167, (char) 252, (char) 5, (char) 184, (char) 74, (char) 235, (char) 251, (char) 58, (char) 8, (char) 199, (char) 171, (char) 11, (char) 124, (char) 40, (char) 239, (char) 32, (char) 97, (char) 10, (char) 111, (char) 12, (char) 131, (char) 196, (char) 82, (char) 125, (char) 12, (char) 85, (char) 149, (char) 131, (char) 121, (char) 177, (char) 156, (char) 41, (char) 114, (char) 80, (char) 127, (char) 146, (char) 83, (char) 81, (char) 41, (char) 251, (char) 171, (char) 43, (char) 235, (char) 77, (char) 251, (char) 18, (char) 83, (char) 170, (char) 152, (char) 0, (char) 149, (char) 52, (char) 104, (char) 144, (char) 194, (char) 213, (char) 37, (char) 95, (char) 200, (char) 101, (char) 230, (char) 50, (char) 137, (char) 109, (char) 2, (char) 28, (char) 209, (char) 131, (char) 82, (char) 112, (char) 155, (char) 228, (char) 127, (char) 39, (char) 112, (char) 186, (char) 60, (char) 23, (char) 99, (char) 204, (char) 128, (char) 225, (char) 91, (char) 217, (char) 11, (char) 58, (char) 38, (char) 148, (char) 45, (char) 115, (char) 1}));
			assert (pack.uri_type_GET() == (char) 105);
			assert (pack.transfer_type_GET() == (char) 103);
			assert (Arrays.equals(pack.uri_GET(), new char[]{(char) 32, (char) 61, (char) 25, (char) 70, (char) 30, (char) 26, (char) 134, (char) 49, (char) 142, (char) 250, (char) 104, (char) 211, (char) 111, (char) 80, (char) 61, (char) 133, (char) 59, (char) 160, (char) 10, (char) 98, (char) 36, (char) 131, (char) 112, (char) 202, (char) 52, (char) 244, (char) 97, (char) 193, (char) 240, (char) 134, (char) 68, (char) 124, (char) 207, (char) 90, (char) 107, (char) 116, (char) 95, (char) 139, (char) 26, (char) 163, (char) 118, (char) 16, (char) 4, (char) 27, (char) 88, (char) 177, (char) 24, (char) 68, (char) 23, (char) 103, (char) 215, (char) 219, (char) 97, (char) 46, (char) 115, (char) 125, (char) 84, (char) 16, (char) 29, (char) 120, (char) 115, (char) 154, (char) 245, (char) 166, (char) 231, (char) 92, (char) 149, (char) 137, (char) 155, (char) 166, (char) 234, (char) 94, (char) 233, (char) 208, (char) 19, (char) 232, (char) 46, (char) 154, (char) 136, (char) 201, (char) 204, (char) 77, (char) 26, (char) 19, (char) 87, (char) 193, (char) 177, (char) 77, (char) 52, (char) 246, (char) 162, (char) 60, (char) 143, (char) 106, (char) 113, (char) 48, (char) 99, (char) 194, (char) 168, (char) 167, (char) 241, (char) 23, (char) 182, (char) 150, (char) 182, (char) 11, (char) 249, (char) 249, (char) 195, (char) 126, (char) 213, (char) 180, (char) 34, (char) 160, (char) 32, (char) 219, (char) 191, (char) 68, (char) 67, (char) 220}));
		});
		GroundControl.RESOURCE_REQUEST p142 = CommunicationChannel.new_RESOURCE_REQUEST();
		PH.setPack(p142);
		p142.uri_type_SET((char) 105);
		p142.storage_SET(new char[]{(char) 150, (char) 52, (char) 93, (char) 157, (char) 68, (char) 130, (char) 150, (char) 190, (char) 137, (char) 226, (char) 131, (char) 3, (char) 247, (char) 127, (char) 221, (char) 16, (char) 28, (char) 199, (char) 230, (char) 100, (char) 228, (char) 199, (char) 170, (char) 73, (char) 96, (char) 119, (char) 3, (char) 230, (char) 86, (char) 167, (char) 252, (char) 5, (char) 184, (char) 74, (char) 235, (char) 251, (char) 58, (char) 8, (char) 199, (char) 171, (char) 11, (char) 124, (char) 40, (char) 239, (char) 32, (char) 97, (char) 10, (char) 111, (char) 12, (char) 131, (char) 196, (char) 82, (char) 125, (char) 12, (char) 85, (char) 149, (char) 131, (char) 121, (char) 177, (char) 156, (char) 41, (char) 114, (char) 80, (char) 127, (char) 146, (char) 83, (char) 81, (char) 41, (char) 251, (char) 171, (char) 43, (char) 235, (char) 77, (char) 251, (char) 18, (char) 83, (char) 170, (char) 152, (char) 0, (char) 149, (char) 52, (char) 104, (char) 144, (char) 194, (char) 213, (char) 37, (char) 95, (char) 200, (char) 101, (char) 230, (char) 50, (char) 137, (char) 109, (char) 2, (char) 28, (char) 209, (char) 131, (char) 82, (char) 112, (char) 155, (char) 228, (char) 127, (char) 39, (char) 112, (char) 186, (char) 60, (char) 23, (char) 99, (char) 204, (char) 128, (char) 225, (char) 91, (char) 217, (char) 11, (char) 58, (char) 38, (char) 148, (char) 45, (char) 115, (char) 1}, 0);
		p142.request_id_SET((char) 119);
		p142.transfer_type_SET((char) 103);
		p142.uri_SET(new char[]{(char) 32, (char) 61, (char) 25, (char) 70, (char) 30, (char) 26, (char) 134, (char) 49, (char) 142, (char) 250, (char) 104, (char) 211, (char) 111, (char) 80, (char) 61, (char) 133, (char) 59, (char) 160, (char) 10, (char) 98, (char) 36, (char) 131, (char) 112, (char) 202, (char) 52, (char) 244, (char) 97, (char) 193, (char) 240, (char) 134, (char) 68, (char) 124, (char) 207, (char) 90, (char) 107, (char) 116, (char) 95, (char) 139, (char) 26, (char) 163, (char) 118, (char) 16, (char) 4, (char) 27, (char) 88, (char) 177, (char) 24, (char) 68, (char) 23, (char) 103, (char) 215, (char) 219, (char) 97, (char) 46, (char) 115, (char) 125, (char) 84, (char) 16, (char) 29, (char) 120, (char) 115, (char) 154, (char) 245, (char) 166, (char) 231, (char) 92, (char) 149, (char) 137, (char) 155, (char) 166, (char) 234, (char) 94, (char) 233, (char) 208, (char) 19, (char) 232, (char) 46, (char) 154, (char) 136, (char) 201, (char) 204, (char) 77, (char) 26, (char) 19, (char) 87, (char) 193, (char) 177, (char) 77, (char) 52, (char) 246, (char) 162, (char) 60, (char) 143, (char) 106, (char) 113, (char) 48, (char) 99, (char) 194, (char) 168, (char) 167, (char) 241, (char) 23, (char) 182, (char) 150, (char) 182, (char) 11, (char) 249, (char) 249, (char) 195, (char) 126, (char) 213, (char) 180, (char) 34, (char) 160, (char) 32, (char) 219, (char) 191, (char) 68, (char) 67, (char) 220}, 0);
		CommunicationChannel.instance.send(p142);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
		TestChannel.instance.on_SCALED_PRESSURE3.add((src, ph, pack) ->
		{
			assert (pack.press_diff_GET() == -3.0945824E38F);
			assert (pack.time_boot_ms_GET() == 1598205222L);
			assert (pack.press_abs_GET() == -1.31484404E36F);
			assert (pack.temperature_GET() == (short) 28817);
		});
		GroundControl.SCALED_PRESSURE3 p143 = CommunicationChannel.new_SCALED_PRESSURE3();
		PH.setPack(p143);
		p143.time_boot_ms_SET(1598205222L);
		p143.temperature_SET((short) 28817);
		p143.press_abs_SET(-1.31484404E36F);
		p143.press_diff_SET(-3.0945824E38F);
		CommunicationChannel.instance.send(p143);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
		TestChannel.instance.on_FOLLOW_TARGET.add((src, ph, pack) ->
		{
			assert (Arrays.equals(pack.attitude_q_GET(), new float[]{-2.1099489E38F, -1.1077221E38F, -9.682411E37F, 7.1038475E37F}));
			assert (pack.est_capabilities_GET() == (char) 212);
			assert (pack.custom_state_GET() == 2868320636401544392L);
			assert (Arrays.equals(pack.position_cov_GET(), new float[]{-4.608119E37F, -2.9243708E38F, -1.5716282E38F}));
			assert (pack.lon_GET() == 2044278635);
			assert (pack.alt_GET() == 2.485912E38F);
			assert (Arrays.equals(pack.acc_GET(), new float[]{2.5248318E38F, -2.0559007E38F, 1.5219854E38F}));
			assert (Arrays.equals(pack.vel_GET(), new float[]{-2.1092818E38F, 1.0076679E38F, -1.762473E38F}));
			assert (pack.lat_GET() == 1609864667);
			assert (pack.timestamp_GET() == 5520459272231626216L);
			assert (Arrays.equals(pack.rates_GET(), new float[]{-2.4035889E38F, 2.998836E38F, -3.2709231E38F}));
		});
		GroundControl.FOLLOW_TARGET p144 = CommunicationChannel.new_FOLLOW_TARGET();
		PH.setPack(p144);
		p144.attitude_q_SET(new float[]{-2.1099489E38F, -1.1077221E38F, -9.682411E37F, 7.1038475E37F}, 0);
		p144.acc_SET(new float[]{2.5248318E38F, -2.0559007E38F, 1.5219854E38F}, 0);
		p144.timestamp_SET(5520459272231626216L);
		p144.rates_SET(new float[]{-2.4035889E38F, 2.998836E38F, -3.2709231E38F}, 0);
		p144.custom_state_SET(2868320636401544392L);
		p144.alt_SET(2.485912E38F);
		p144.vel_SET(new float[]{-2.1092818E38F, 1.0076679E38F, -1.762473E38F}, 0);
		p144.position_cov_SET(new float[]{-4.608119E37F, -2.9243708E38F, -1.5716282E38F}, 0);
		p144.lon_SET(2044278635);
		p144.est_capabilities_SET((char) 212);
		p144.lat_SET(1609864667);
		CommunicationChannel.instance.send(p144);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
		TestChannel.instance.on_CONTROL_SYSTEM_STATE.add((src, ph, pack) ->
		{
			assert (pack.y_acc_GET() == 3.0819265E38F);
			assert (pack.x_acc_GET() == -2.3512213E38F);
			assert (pack.z_pos_GET() == -1.1549962E38F);
			assert (pack.yaw_rate_GET() == 1.5257816E38F);
			assert (pack.time_usec_GET() == 2922292348710111933L);
			assert (pack.x_vel_GET() == 1.4445512E38F);
			assert (pack.x_pos_GET() == 1.2039661E38F);
			assert (Arrays.equals(pack.q_GET(), new float[]{1.5613172E38F, -2.3162627E36F, -2.3179555E38F, 3.3216484E38F}));
			assert (pack.pitch_rate_GET() == 2.7613087E38F);
			assert (pack.y_pos_GET() == -2.0482135E38F);
			assert (pack.z_acc_GET() == -3.1511618E38F);
			assert (pack.roll_rate_GET() == 1.7333467E38F);
			assert (Arrays.equals(pack.pos_variance_GET(), new float[]{-1.7781743E38F, 1.4558106E38F, -2.5479552E38F}));
			assert (pack.airspeed_GET() == 1.0153062E38F);
			assert (pack.y_vel_GET() == 7.6590663E37F);
			assert (pack.z_vel_GET() == -1.9837296E38F);
			assert (Arrays.equals(pack.vel_variance_GET(), new float[]{-1.7256706E38F, -7.2346463E37F, 4.701829E37F}));
		});
		GroundControl.CONTROL_SYSTEM_STATE p146 = CommunicationChannel.new_CONTROL_SYSTEM_STATE();
		PH.setPack(p146);
		p146.time_usec_SET(2922292348710111933L);
		p146.x_pos_SET(1.2039661E38F);
		p146.pos_variance_SET(new float[]{-1.7781743E38F, 1.4558106E38F, -2.5479552E38F}, 0);
		p146.airspeed_SET(1.0153062E38F);
		p146.vel_variance_SET(new float[]{-1.7256706E38F, -7.2346463E37F, 4.701829E37F}, 0);
		p146.x_vel_SET(1.4445512E38F);
		p146.y_acc_SET(3.0819265E38F);
		p146.roll_rate_SET(1.7333467E38F);
		p146.z_pos_SET(-1.1549962E38F);
		p146.yaw_rate_SET(1.5257816E38F);
		p146.pitch_rate_SET(2.7613087E38F);
		p146.x_acc_SET(-2.3512213E38F);
		p146.z_vel_SET(-1.9837296E38F);
		p146.y_vel_SET(7.6590663E37F);
		p146.q_SET(new float[]{1.5613172E38F, -2.3162627E36F, -2.3179555E38F, 3.3216484E38F}, 0);
		p146.y_pos_SET(-2.0482135E38F);
		p146.z_acc_SET(-3.1511618E38F);
		CommunicationChannel.instance.send(p146);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
		TestChannel.instance.on_BATTERY_STATUS.add((src, ph, pack) ->
		{
			assert (pack.battery_remaining_GET() == (byte) 34);
			assert (pack.temperature_GET() == (short) -32683);
			assert (pack.battery_function_GET() == MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_ALL);
			assert (pack.id_GET() == (char) 139);
			assert (pack.energy_consumed_GET() == 856254069);
			assert (Arrays.equals(pack.voltages_GET(), new char[]{(char) 13902, (char) 20875, (char) 28273, (char) 32858, (char) 7391, (char) 64470, (char) 34692, (char) 59384, (char) 64459, (char) 13701}));
			assert (pack.type_GET() == MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_NIMH);
			assert (pack.current_battery_GET() == (short) -1913);
			assert (pack.current_consumed_GET() == 505472048);
		});
		GroundControl.BATTERY_STATUS p147 = CommunicationChannel.new_BATTERY_STATUS();
		PH.setPack(p147);
		p147.type_SET(MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_NIMH);
		p147.battery_function_SET(MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_ALL);
		p147.current_battery_SET((short) -1913);
		p147.energy_consumed_SET(856254069);
		p147.battery_remaining_SET((byte) 34);
		p147.temperature_SET((short) -32683);
		p147.id_SET((char) 139);
		p147.current_consumed_SET(505472048);
		p147.voltages_SET(new char[]{(char) 13902, (char) 20875, (char) 28273, (char) 32858, (char) 7391, (char) 64470, (char) 34692, (char) 59384, (char) 64459, (char) 13701}, 0);
		CommunicationChannel.instance.send(p147);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
		TestChannel.instance.on_AUTOPILOT_VERSION.add((src, ph, pack) ->
		{
			assert (pack.os_sw_version_GET() == 68798528L);
			assert (pack.uid_GET() == 1379827544715416488L);
			assert (pack.flight_sw_version_GET() == 2585680732L);
			assert (pack.capabilities_GET() == MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT);
			assert (pack.product_id_GET() == (char) 48010);
			assert (pack.vendor_id_GET() == (char) 31411);
			assert (Arrays.equals(pack.flight_custom_version_GET(), new char[]{(char) 57, (char) 199, (char) 19, (char) 93, (char) 145, (char) 93, (char) 157, (char) 129}));
			assert (Arrays.equals(pack.uid2_TRY(ph), new char[]{(char) 9, (char) 192, (char) 171, (char) 244, (char) 155, (char) 68, (char) 199, (char) 98, (char) 174, (char) 62, (char) 226, (char) 214, (char) 251, (char) 250, (char) 146, (char) 228, (char) 30, (char) 34}));
			assert (pack.middleware_sw_version_GET() == 2647581044L);
			assert (pack.board_version_GET() == 2029261981L);
			assert (Arrays.equals(pack.os_custom_version_GET(), new char[]{(char) 255, (char) 190, (char) 194, (char) 113, (char) 128, (char) 128, (char) 194, (char) 92}));
			assert (Arrays.equals(pack.middleware_custom_version_GET(), new char[]{(char) 22, (char) 168, (char) 198, (char) 250, (char) 167, (char) 171, (char) 88, (char) 193}));
		});
		GroundControl.AUTOPILOT_VERSION p148 = CommunicationChannel.new_AUTOPILOT_VERSION();
		PH.setPack(p148);
		p148.board_version_SET(2029261981L);
		p148.os_custom_version_SET(new char[]{(char) 255, (char) 190, (char) 194, (char) 113, (char) 128, (char) 128, (char) 194, (char) 92}, 0);
		p148.product_id_SET((char) 48010);
		p148.uid2_SET(new char[]{(char) 9, (char) 192, (char) 171, (char) 244, (char) 155, (char) 68, (char) 199, (char) 98, (char) 174, (char) 62, (char) 226, (char) 214, (char) 251, (char) 250, (char) 146, (char) 228, (char) 30, (char) 34}, 0, PH);
		p148.flight_custom_version_SET(new char[]{(char) 57, (char) 199, (char) 19, (char) 93, (char) 145, (char) 93, (char) 157, (char) 129}, 0);
		p148.flight_sw_version_SET(2585680732L);
		p148.os_sw_version_SET(68798528L);
		p148.middleware_custom_version_SET(new char[]{(char) 22, (char) 168, (char) 198, (char) 250, (char) 167, (char) 171, (char) 88, (char) 193}, 0);
		p148.uid_SET(1379827544715416488L);
		p148.vendor_id_SET((char) 31411);
		p148.capabilities_SET(MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT);
		p148.middleware_sw_version_SET(2647581044L);
		CommunicationChannel.instance.send(p148);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
		TestChannel.instance.on_LANDING_TARGET.add((src, ph, pack) ->
		{
			assert (pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT);
			assert (pack.target_num_GET() == (char) 8);
			assert (pack.distance_GET() == 8.725867E37F);
			assert (pack.time_usec_GET() == 2642237181777001286L);
			assert (pack.angle_y_GET() == 1.83545E38F);
			assert (pack.y_TRY(ph) == -2.8904753E37F);
			assert (pack.type_GET() == LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_VISION_OTHER);
			assert (pack.size_x_GET() == -1.1487771E38F);
			assert (pack.z_TRY(ph) == 1.7078091E38F);
			assert (pack.angle_x_GET() == -2.751757E37F);
			assert (pack.position_valid_TRY(ph) == (char) 151);
			assert (Arrays.equals(pack.q_TRY(ph), new float[]{-7.202243E37F, -6.719915E37F, -7.4728723E37F, -1.2185564E38F}));
			assert (pack.x_TRY(ph) == 1.7767535E38F);
			assert (pack.size_y_GET() == -7.2278425E37F);
		});
		GroundControl.LANDING_TARGET p149 = CommunicationChannel.new_LANDING_TARGET();
		PH.setPack(p149);
		p149.target_num_SET((char) 8);
		p149.position_valid_SET((char) 151, PH);
		p149.x_SET(1.7767535E38F, PH);
		p149.angle_y_SET(1.83545E38F);
		p149.size_y_SET(-7.2278425E37F);
		p149.y_SET(-2.8904753E37F, PH);
		p149.z_SET(1.7078091E38F, PH);
		p149.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT);
		p149.angle_x_SET(-2.751757E37F);
		p149.size_x_SET(-1.1487771E38F);
		p149.distance_SET(8.725867E37F);
		p149.type_SET(LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_VISION_OTHER);
		p149.time_usec_SET(2642237181777001286L);
		p149.q_SET(new float[]{-7.202243E37F, -6.719915E37F, -7.4728723E37F, -1.2185564E38F}, 0, PH);
		CommunicationChannel.instance.send(p149);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
		TestChannel.instance.on_NAV_FILTER_BIAS.add((src, ph, pack) ->
		{
			assert (pack.accel_1_GET() == -1.9581944E38F);
			assert (pack.usec_GET() == 958752830576954600L);
			assert (pack.gyro_0_GET() == -1.043945E38F);
			assert (pack.accel_0_GET() == -2.279202E38F);
			assert (pack.accel_2_GET() == -2.0271504E38F);
			assert (pack.gyro_1_GET() == -2.8409963E38F);
			assert (pack.gyro_2_GET() == -2.5723841E38F);
		});
		GroundControl.NAV_FILTER_BIAS p220 = CommunicationChannel.new_NAV_FILTER_BIAS();
		PH.setPack(p220);
		p220.gyro_0_SET(-1.043945E38F);
		p220.accel_2_SET(-2.0271504E38F);
		p220.accel_1_SET(-1.9581944E38F);
		p220.accel_0_SET(-2.279202E38F);
		p220.gyro_2_SET(-2.5723841E38F);
		p220.gyro_1_SET(-2.8409963E38F);
		p220.usec_SET(958752830576954600L);
		CommunicationChannel.instance.send(p220);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
		TestChannel.instance.on_RADIO_CALIBRATION.add((src, ph, pack) ->
		{
			assert (Arrays.equals(pack.throttle_GET(), new char[]{(char) 6621, (char) 62782, (char) 21126, (char) 2330, (char) 13603}));
			assert (Arrays.equals(pack.rudder_GET(), new char[]{(char) 59557, (char) 3907, (char) 23166}));
			assert (Arrays.equals(pack.aileron_GET(), new char[]{(char) 38350, (char) 43201, (char) 14948}));
			assert (Arrays.equals(pack.elevator_GET(), new char[]{(char) 5377, (char) 6714, (char) 42720}));
			assert (Arrays.equals(pack.gyro_GET(), new char[]{(char) 20640, (char) 53342}));
			assert (Arrays.equals(pack.pitch_GET(), new char[]{(char) 24241, (char) 34466, (char) 64233, (char) 4233, (char) 1461}));
		});
		GroundControl.RADIO_CALIBRATION p221 = CommunicationChannel.new_RADIO_CALIBRATION();
		PH.setPack(p221);
		p221.rudder_SET(new char[]{(char) 59557, (char) 3907, (char) 23166}, 0);
		p221.aileron_SET(new char[]{(char) 38350, (char) 43201, (char) 14948}, 0);
		p221.elevator_SET(new char[]{(char) 5377, (char) 6714, (char) 42720}, 0);
		p221.gyro_SET(new char[]{(char) 20640, (char) 53342}, 0);
		p221.throttle_SET(new char[]{(char) 6621, (char) 62782, (char) 21126, (char) 2330, (char) 13603}, 0);
		p221.pitch_SET(new char[]{(char) 24241, (char) 34466, (char) 64233, (char) 4233, (char) 1461}, 0);
		CommunicationChannel.instance.send(p221);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
		TestChannel.instance.on_UALBERTA_SYS_STATUS.add((src, ph, pack) ->
		{
			assert (pack.mode_GET() == (char) 232);
			assert (pack.nav_mode_GET() == (char) 109);
			assert (pack.pilot_GET() == (char) 10);
		});
		GroundControl.UALBERTA_SYS_STATUS p222 = CommunicationChannel.new_UALBERTA_SYS_STATUS();
		PH.setPack(p222);
		p222.nav_mode_SET((char) 109);
		p222.mode_SET((char) 232);
		p222.pilot_SET((char) 10);
		CommunicationChannel.instance.send(p222);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
		TestChannel.instance.on_ESTIMATOR_STATUS.add((src, ph, pack) ->
		{
			assert (pack.pos_horiz_ratio_GET() == 2.9536782E38F);
			assert (pack.mag_ratio_GET() == 1.0471882E38F);
			assert (pack.pos_vert_accuracy_GET() == 1.4054112E38F);
			assert (pack.time_usec_GET() == 6216775132081817795L);
			assert (pack.pos_horiz_accuracy_GET() == -1.3728837E38F);
			assert (pack.tas_ratio_GET() == 2.9454831E38F);
			assert (pack.pos_vert_ratio_GET() == -1.5028044E38F);
			assert (pack.vel_ratio_GET() == 2.2408888E38F);
			assert (pack.flags_GET() == ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_VERT_AGL);
			assert (pack.hagl_ratio_GET() == -1.6352066E38F);
		});
		GroundControl.ESTIMATOR_STATUS p230 = CommunicationChannel.new_ESTIMATOR_STATUS();
		PH.setPack(p230);
		p230.tas_ratio_SET(2.9454831E38F);
		p230.flags_SET(ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_VERT_AGL);
		p230.time_usec_SET(6216775132081817795L);
		p230.pos_horiz_accuracy_SET(-1.3728837E38F);
		p230.pos_horiz_ratio_SET(2.9536782E38F);
		p230.pos_vert_ratio_SET(-1.5028044E38F);
		p230.vel_ratio_SET(2.2408888E38F);
		p230.mag_ratio_SET(1.0471882E38F);
		p230.hagl_ratio_SET(-1.6352066E38F);
		p230.pos_vert_accuracy_SET(1.4054112E38F);
		CommunicationChannel.instance.send(p230);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
		TestChannel.instance.on_WIND_COV.add((src, ph, pack) ->
		{
			assert (pack.vert_accuracy_GET() == -1.4178167E38F);
			assert (pack.horiz_accuracy_GET() == -1.9837107E37F);
			assert (pack.wind_y_GET() == -1.0693548E38F);
			assert (pack.time_usec_GET() == 3704129181151677624L);
			assert (pack.wind_alt_GET() == -5.9473724E37F);
			assert (pack.var_vert_GET() == 4.0645378E37F);
			assert (pack.var_horiz_GET() == -5.144736E37F);
			assert (pack.wind_z_GET() == -3.0082825E38F);
			assert (pack.wind_x_GET() == 2.6764333E38F);
		});
		GroundControl.WIND_COV p231 = CommunicationChannel.new_WIND_COV();
		PH.setPack(p231);
		p231.var_horiz_SET(-5.144736E37F);
		p231.horiz_accuracy_SET(-1.9837107E37F);
		p231.wind_alt_SET(-5.9473724E37F);
		p231.wind_y_SET(-1.0693548E38F);
		p231.time_usec_SET(3704129181151677624L);
		p231.var_vert_SET(4.0645378E37F);
		p231.wind_x_SET(2.6764333E38F);
		p231.vert_accuracy_SET(-1.4178167E38F);
		p231.wind_z_SET(-3.0082825E38F);
		CommunicationChannel.instance.send(p231);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
		TestChannel.instance.on_GPS_INPUT.add((src, ph, pack) ->
		{
			assert (pack.time_week_GET() == (char) 10498);
			assert (pack.vn_GET() == -1.8116816E38F);
			assert (pack.vdop_GET() == -1.9059372E38F);
			assert (pack.satellites_visible_GET() == (char) 176);
			assert (pack.fix_type_GET() == (char) 220);
			assert (pack.alt_GET() == 3.2675274E38F);
			assert (pack.time_usec_GET() == 5234209271230957068L);
			assert (pack.hdop_GET() == -8.877974E37F);
			assert (pack.gps_id_GET() == (char) 185);
			assert (pack.vd_GET() == -1.2312472E38F);
			assert (pack.ignore_flags_GET() == GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_ALT);
			assert (pack.vert_accuracy_GET() == -1.7775895E38F);
			assert (pack.horiz_accuracy_GET() == -2.2098162E38F);
			assert (pack.speed_accuracy_GET() == -2.7163037E38F);
			assert (pack.lon_GET() == -1044748101);
			assert (pack.lat_GET() == -1594680243);
			assert (pack.time_week_ms_GET() == 1137388473L);
			assert (pack.ve_GET() == 9.81344E37F);
		});
		GroundControl.GPS_INPUT p232 = CommunicationChannel.new_GPS_INPUT();
		PH.setPack(p232);
		p232.vdop_SET(-1.9059372E38F);
		p232.ignore_flags_SET(GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_ALT);
		p232.alt_SET(3.2675274E38F);
		p232.hdop_SET(-8.877974E37F);
		p232.vd_SET(-1.2312472E38F);
		p232.time_week_ms_SET(1137388473L);
		p232.lat_SET(-1594680243);
		p232.time_usec_SET(5234209271230957068L);
		p232.speed_accuracy_SET(-2.7163037E38F);
		p232.horiz_accuracy_SET(-2.2098162E38F);
		p232.ve_SET(9.81344E37F);
		p232.satellites_visible_SET((char) 176);
		p232.fix_type_SET((char) 220);
		p232.vert_accuracy_SET(-1.7775895E38F);
		p232.lon_SET(-1044748101);
		p232.gps_id_SET((char) 185);
		p232.time_week_SET((char) 10498);
		p232.vn_SET(-1.8116816E38F);
		CommunicationChannel.instance.send(p232);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
		TestChannel.instance.on_GPS_RTCM_DATA.add((src, ph, pack) ->
		{
			assert (Arrays.equals(pack.data__GET(), new char[]{(char) 187, (char) 250, (char) 69, (char) 71, (char) 93, (char) 147, (char) 143, (char) 232, (char) 248, (char) 170, (char) 156, (char) 160, (char) 104, (char) 95, (char) 166, (char) 139, (char) 205, (char) 140, (char) 153, (char) 19, (char) 205, (char) 11, (char) 227, (char) 194, (char) 154, (char) 102, (char) 190, (char) 5, (char) 242, (char) 132, (char) 246, (char) 103, (char) 245, (char) 94, (char) 97, (char) 211, (char) 55, (char) 63, (char) 69, (char) 64, (char) 62, (char) 10, (char) 55, (char) 163, (char) 122, (char) 226, (char) 139, (char) 1, (char) 190, (char) 189, (char) 109, (char) 132, (char) 197, (char) 187, (char) 152, (char) 138, (char) 210, (char) 200, (char) 193, (char) 154, (char) 183, (char) 211, (char) 196, (char) 203, (char) 112, (char) 229, (char) 125, (char) 149, (char) 67, (char) 177, (char) 79, (char) 31, (char) 71, (char) 182, (char) 191, (char) 216, (char) 227, (char) 146, (char) 45, (char) 219, (char) 232, (char) 82, (char) 35, (char) 15, (char) 7, (char) 191, (char) 122, (char) 27, (char) 81, (char) 214, (char) 184, (char) 214, (char) 244, (char) 48, (char) 73, (char) 88, (char) 39, (char) 229, (char) 8, (char) 201, (char) 147, (char) 89, (char) 93, (char) 204, (char) 228, (char) 76, (char) 125, (char) 220, (char) 211, (char) 246, (char) 176, (char) 104, (char) 88, (char) 84, (char) 255, (char) 150, (char) 239, (char) 77, (char) 11, (char) 225, (char) 170, (char) 4, (char) 16, (char) 131, (char) 12, (char) 34, (char) 252, (char) 10, (char) 64, (char) 163, (char) 247, (char) 174, (char) 160, (char) 56, (char) 192, (char) 226, (char) 254, (char) 192, (char) 0, (char) 188, (char) 147, (char) 226, (char) 205, (char) 58, (char) 176, (char) 103, (char) 36, (char) 168, (char) 25, (char) 51, (char) 29, (char) 192, (char) 124, (char) 114, (char) 187, (char) 227, (char) 68, (char) 157, (char) 106, (char) 80, (char) 50, (char) 129, (char) 4, (char) 39, (char) 254, (char) 99, (char) 194, (char) 154, (char) 247, (char) 199, (char) 33, (char) 121, (char) 133, (char) 106, (char) 117, (char) 130, (char) 214, (char) 10, (char) 141, (char) 230}));
			assert (pack.len_GET() == (char) 136);
			assert (pack.flags_GET() == (char) 135);
		});
		GroundControl.GPS_RTCM_DATA p233 = CommunicationChannel.new_GPS_RTCM_DATA();
		PH.setPack(p233);
		p233.data__SET(new char[]{(char) 187, (char) 250, (char) 69, (char) 71, (char) 93, (char) 147, (char) 143, (char) 232, (char) 248, (char) 170, (char) 156, (char) 160, (char) 104, (char) 95, (char) 166, (char) 139, (char) 205, (char) 140, (char) 153, (char) 19, (char) 205, (char) 11, (char) 227, (char) 194, (char) 154, (char) 102, (char) 190, (char) 5, (char) 242, (char) 132, (char) 246, (char) 103, (char) 245, (char) 94, (char) 97, (char) 211, (char) 55, (char) 63, (char) 69, (char) 64, (char) 62, (char) 10, (char) 55, (char) 163, (char) 122, (char) 226, (char) 139, (char) 1, (char) 190, (char) 189, (char) 109, (char) 132, (char) 197, (char) 187, (char) 152, (char) 138, (char) 210, (char) 200, (char) 193, (char) 154, (char) 183, (char) 211, (char) 196, (char) 203, (char) 112, (char) 229, (char) 125, (char) 149, (char) 67, (char) 177, (char) 79, (char) 31, (char) 71, (char) 182, (char) 191, (char) 216, (char) 227, (char) 146, (char) 45, (char) 219, (char) 232, (char) 82, (char) 35, (char) 15, (char) 7, (char) 191, (char) 122, (char) 27, (char) 81, (char) 214, (char) 184, (char) 214, (char) 244, (char) 48, (char) 73, (char) 88, (char) 39, (char) 229, (char) 8, (char) 201, (char) 147, (char) 89, (char) 93, (char) 204, (char) 228, (char) 76, (char) 125, (char) 220, (char) 211, (char) 246, (char) 176, (char) 104, (char) 88, (char) 84, (char) 255, (char) 150, (char) 239, (char) 77, (char) 11, (char) 225, (char) 170, (char) 4, (char) 16, (char) 131, (char) 12, (char) 34, (char) 252, (char) 10, (char) 64, (char) 163, (char) 247, (char) 174, (char) 160, (char) 56, (char) 192, (char) 226, (char) 254, (char) 192, (char) 0, (char) 188, (char) 147, (char) 226, (char) 205, (char) 58, (char) 176, (char) 103, (char) 36, (char) 168, (char) 25, (char) 51, (char) 29, (char) 192, (char) 124, (char) 114, (char) 187, (char) 227, (char) 68, (char) 157, (char) 106, (char) 80, (char) 50, (char) 129, (char) 4, (char) 39, (char) 254, (char) 99, (char) 194, (char) 154, (char) 247, (char) 199, (char) 33, (char) 121, (char) 133, (char) 106, (char) 117, (char) 130, (char) 214, (char) 10, (char) 141, (char) 230}, 0);
		p233.len_SET((char) 136);
		p233.flags_SET((char) 135);
		CommunicationChannel.instance.send(p233);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
		TestChannel.instance.on_HIGH_LATENCY.add((src, ph, pack) ->
		{
			assert (pack.temperature_GET() == (byte) -77);
			assert (pack.gps_fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_NO_FIX);
			assert (pack.custom_mode_GET() == 3932219918L);
			assert (pack.landed_state_GET() == MAV_LANDED_STATE.MAV_LANDED_STATE_ON_GROUND);
			assert (pack.airspeed_sp_GET() == (char) 140);
			assert (pack.throttle_GET() == (byte) -13);
			assert (pack.groundspeed_GET() == (char) 44);
			assert (pack.base_mode_GET() == MAV_MODE_FLAG.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED);
			assert (pack.climb_rate_GET() == (byte) -77);
			assert (pack.gps_nsat_GET() == (char) 13);
			assert (pack.roll_GET() == (short) 13148);
			assert (pack.altitude_sp_GET() == (short) 11364);
			assert (pack.wp_distance_GET() == (char) 47500);
			assert (pack.heading_sp_GET() == (short) 15486);
			assert (pack.heading_GET() == (char) 10090);
			assert (pack.pitch_GET() == (short) 6722);
			assert (pack.failsafe_GET() == (char) 115);
			assert (pack.airspeed_GET() == (char) 214);
			assert (pack.wp_num_GET() == (char) 222);
			assert (pack.longitude_GET() == 1829878557);
			assert (pack.temperature_air_GET() == (byte) -41);
			assert (pack.altitude_amsl_GET() == (short) -9438);
			assert (pack.battery_remaining_GET() == (char) 69);
			assert (pack.latitude_GET() == 866874344);
		});
		GroundControl.HIGH_LATENCY p234 = CommunicationChannel.new_HIGH_LATENCY();
		PH.setPack(p234);
		p234.latitude_SET(866874344);
		p234.altitude_sp_SET((short) 11364);
		p234.climb_rate_SET((byte) -77);
		p234.base_mode_SET(MAV_MODE_FLAG.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED);
		p234.airspeed_sp_SET((char) 140);
		p234.heading_SET((char) 10090);
		p234.heading_sp_SET((short) 15486);
		p234.temperature_air_SET((byte) -41);
		p234.battery_remaining_SET((char) 69);
		p234.gps_nsat_SET((char) 13);
		p234.pitch_SET((short) 6722);
		p234.longitude_SET(1829878557);
		p234.groundspeed_SET((char) 44);
		p234.airspeed_SET((char) 214);
		p234.custom_mode_SET(3932219918L);
		p234.failsafe_SET((char) 115);
		p234.throttle_SET((byte) -13);
		p234.landed_state_SET(MAV_LANDED_STATE.MAV_LANDED_STATE_ON_GROUND);
		p234.gps_fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_NO_FIX);
		p234.wp_num_SET((char) 222);
		p234.temperature_SET((byte) -77);
		p234.roll_SET((short) 13148);
		p234.wp_distance_SET((char) 47500);
		p234.altitude_amsl_SET((short) -9438);
		CommunicationChannel.instance.send(p234);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
		TestChannel.instance.on_VIBRATION.add((src, ph, pack) ->
		{
			assert (pack.clipping_0_GET() == 1870916517L);
			assert (pack.clipping_1_GET() == 1272447351L);
			assert (pack.vibration_z_GET() == 2.0738216E37F);
			assert (pack.vibration_x_GET() == -2.4915952E38F);
			assert (pack.vibration_y_GET() == -7.621614E37F);
			assert (pack.time_usec_GET() == 6195341963455561980L);
			assert (pack.clipping_2_GET() == 1349806220L);
		});
		GroundControl.VIBRATION p241 = CommunicationChannel.new_VIBRATION();
		PH.setPack(p241);
		p241.time_usec_SET(6195341963455561980L);
		p241.vibration_y_SET(-7.621614E37F);
		p241.clipping_1_SET(1272447351L);
		p241.vibration_x_SET(-2.4915952E38F);
		p241.vibration_z_SET(2.0738216E37F);
		p241.clipping_2_SET(1349806220L);
		p241.clipping_0_SET(1870916517L);
		CommunicationChannel.instance.send(p241);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
		TestChannel.instance.on_HOME_POSITION.add((src, ph, pack) ->
		{
			assert (pack.approach_z_GET() == 1.8467185E38F);
			assert (pack.y_GET() == -1.1213124E37F);
			assert (pack.x_GET() == -2.9834775E38F);
			assert (Arrays.equals(pack.q_GET(), new float[]{3.5961333E37F, -3.1915124E37F, -2.7176545E38F, -2.5051514E38F}));
			assert (pack.approach_x_GET() == -1.3713505E38F);
			assert (pack.altitude_GET() == -234491731);
			assert (pack.latitude_GET() == -780459722);
			assert (pack.time_usec_TRY(ph) == 1169290686396661409L);
			assert (pack.z_GET() == 1.3136544E38F);
			assert (pack.approach_y_GET() == 6.0759066E37F);
			assert (pack.longitude_GET() == 383915531);
		});
		GroundControl.HOME_POSITION p242 = CommunicationChannel.new_HOME_POSITION();
		PH.setPack(p242);
		p242.time_usec_SET(1169290686396661409L, PH);
		p242.y_SET(-1.1213124E37F);
		p242.approach_z_SET(1.8467185E38F);
		p242.longitude_SET(383915531);
		p242.latitude_SET(-780459722);
		p242.x_SET(-2.9834775E38F);
		p242.z_SET(1.3136544E38F);
		p242.altitude_SET(-234491731);
		p242.q_SET(new float[]{3.5961333E37F, -3.1915124E37F, -2.7176545E38F, -2.5051514E38F}, 0);
		p242.approach_x_SET(-1.3713505E38F);
		p242.approach_y_SET(6.0759066E37F);
		CommunicationChannel.instance.send(p242);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
		TestChannel.instance.on_SET_HOME_POSITION.add((src, ph, pack) ->
		{
			assert (pack.latitude_GET() == 397562312);
			assert (Arrays.equals(pack.q_GET(), new float[]{6.266611E37F, -3.3907708E38F, -2.7995385E36F, 2.6808496E38F}));
			assert (pack.x_GET() == -3.0242287E37F);
			assert (pack.target_system_GET() == (char) 42);
			assert (pack.approach_y_GET() == 5.914296E37F);
			assert (pack.altitude_GET() == -1573848293);
			assert (pack.time_usec_TRY(ph) == 2467179962926097557L);
			assert (pack.longitude_GET() == -1843904922);
			assert (pack.approach_x_GET() == -1.9539704E38F);
			assert (pack.z_GET() == -2.867634E38F);
			assert (pack.approach_z_GET() == -1.9848064E38F);
			assert (pack.y_GET() == -1.4583509E38F);
		});
		GroundControl.SET_HOME_POSITION p243 = CommunicationChannel.new_SET_HOME_POSITION();
		PH.setPack(p243);
		p243.approach_y_SET(5.914296E37F);
		p243.altitude_SET(-1573848293);
		p243.approach_z_SET(-1.9848064E38F);
		p243.time_usec_SET(2467179962926097557L, PH);
		p243.x_SET(-3.0242287E37F);
		p243.latitude_SET(397562312);
		p243.q_SET(new float[]{6.266611E37F, -3.3907708E38F, -2.7995385E36F, 2.6808496E38F}, 0);
		p243.target_system_SET((char) 42);
		p243.y_SET(-1.4583509E38F);
		p243.z_SET(-2.867634E38F);
		p243.longitude_SET(-1843904922);
		p243.approach_x_SET(-1.9539704E38F);
		CommunicationChannel.instance.send(p243);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
		TestChannel.instance.on_MESSAGE_INTERVAL.add((src, ph, pack) ->
		{
			assert (pack.message_id_GET() == (char) 35970);
			assert (pack.interval_us_GET() == 403188991);
		});
		GroundControl.MESSAGE_INTERVAL p244 = CommunicationChannel.new_MESSAGE_INTERVAL();
		PH.setPack(p244);
		p244.message_id_SET((char) 35970);
		p244.interval_us_SET(403188991);
		CommunicationChannel.instance.send(p244);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
		TestChannel.instance.on_EXTENDED_SYS_STATE.add((src, ph, pack) ->
		{
			assert (pack.landed_state_GET() == MAV_LANDED_STATE.MAV_LANDED_STATE_TAKEOFF);
			assert (pack.vtol_state_GET() == MAV_VTOL_STATE.MAV_VTOL_STATE_MC);
		});
		GroundControl.EXTENDED_SYS_STATE p245 = CommunicationChannel.new_EXTENDED_SYS_STATE();
		PH.setPack(p245);
		p245.vtol_state_SET(MAV_VTOL_STATE.MAV_VTOL_STATE_MC);
		p245.landed_state_SET(MAV_LANDED_STATE.MAV_LANDED_STATE_TAKEOFF);
		CommunicationChannel.instance.send(p245);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
		TestChannel.instance.on_ADSB_VEHICLE.add((src, ph, pack) ->
		{
			assert (pack.ICAO_address_GET() == 3740652210L);
			assert (pack.lat_GET() == -382995482);
			assert (pack.altitude_GET() == -1118987715);
			assert (pack.callsign_LEN(ph) == 5);
			assert (pack.callsign_TRY(ph).equals("capcv"));
			assert (pack.lon_GET() == -538695922);
			assert (pack.flags_GET() == ADSB_FLAGS.ADSB_FLAGS_SIMULATED);
			assert (pack.hor_velocity_GET() == (char) 57321);
			assert (pack.heading_GET() == (char) 34173);
			assert (pack.tslc_GET() == (char) 38);
			assert (pack.squawk_GET() == (char) 19240);
			assert (pack.emitter_type_GET() == ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_ROTOCRAFT);
			assert (pack.altitude_type_GET() == ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_PRESSURE_QNH);
			assert (pack.ver_velocity_GET() == (short) -23309);
		});
		GroundControl.ADSB_VEHICLE p246 = CommunicationChannel.new_ADSB_VEHICLE();
		PH.setPack(p246);
		p246.altitude_type_SET(ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_PRESSURE_QNH);
		p246.emitter_type_SET(ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_ROTOCRAFT);
		p246.tslc_SET((char) 38);
		p246.heading_SET((char) 34173);
		p246.callsign_SET("capcv", PH);
		p246.altitude_SET(-1118987715);
		p246.ver_velocity_SET((short) -23309);
		p246.ICAO_address_SET(3740652210L);
		p246.hor_velocity_SET((char) 57321);
		p246.flags_SET(ADSB_FLAGS.ADSB_FLAGS_SIMULATED);
		p246.lat_SET(-382995482);
		p246.squawk_SET((char) 19240);
		p246.lon_SET(-538695922);
		CommunicationChannel.instance.send(p246);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
		TestChannel.instance.on_COLLISION.add((src, ph, pack) ->
		{
			assert (pack.id_GET() == 2916411125L);
			assert (pack.threat_level_GET() == MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_LOW);
			assert (pack.altitude_minimum_delta_GET() == -8.542476E36F);
			assert (pack.action_GET() == MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_ASCEND_OR_DESCEND);
			assert (pack.horizontal_minimum_delta_GET() == 2.444447E38F);
			assert (pack.time_to_minimum_delta_GET() == 1.0013027E38F);
			assert (pack.src__GET() == MAV_COLLISION_SRC.MAV_COLLISION_SRC_ADSB);
		});
		GroundControl.COLLISION p247 = CommunicationChannel.new_COLLISION();
		PH.setPack(p247);
		p247.id_SET(2916411125L);
		p247.horizontal_minimum_delta_SET(2.444447E38F);
		p247.src__SET(MAV_COLLISION_SRC.MAV_COLLISION_SRC_ADSB);
		p247.altitude_minimum_delta_SET(-8.542476E36F);
		p247.threat_level_SET(MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_LOW);
		p247.action_SET(MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_ASCEND_OR_DESCEND);
		p247.time_to_minimum_delta_SET(1.0013027E38F);
		CommunicationChannel.instance.send(p247);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
		TestChannel.instance.on_V2_EXTENSION.add((src, ph, pack) ->
		{
			assert (pack.target_system_GET() == (char) 240);
			assert (Arrays.equals(pack.payload_GET(), new char[]{(char) 57, (char) 12, (char) 217, (char) 42, (char) 248, (char) 49, (char) 44, (char) 252, (char) 92, (char) 153, (char) 13, (char) 127, (char) 168, (char) 119, (char) 215, (char) 194, (char) 128, (char) 68, (char) 240, (char) 69, (char) 127, (char) 166, (char) 205, (char) 150, (char) 230, (char) 170, (char) 188, (char) 61, (char) 188, (char) 112, (char) 252, (char) 78, (char) 216, (char) 8, (char) 84, (char) 151, (char) 3, (char) 45, (char) 120, (char) 183, (char) 231, (char) 142, (char) 136, (char) 7, (char) 59, (char) 253, (char) 187, (char) 30, (char) 123, (char) 96, (char) 132, (char) 233, (char) 12, (char) 215, (char) 125, (char) 205, (char) 134, (char) 199, (char) 232, (char) 76, (char) 164, (char) 34, (char) 66, (char) 92, (char) 153, (char) 193, (char) 68, (char) 224, (char) 168, (char) 60, (char) 231, (char) 99, (char) 232, (char) 124, (char) 136, (char) 213, (char) 59, (char) 62, (char) 0, (char) 63, (char) 46, (char) 49, (char) 173, (char) 174, (char) 221, (char) 134, (char) 19, (char) 70, (char) 42, (char) 34, (char) 157, (char) 192, (char) 183, (char) 99, (char) 255, (char) 160, (char) 217, (char) 17, (char) 179, (char) 85, (char) 238, (char) 189, (char) 15, (char) 255, (char) 66, (char) 77, (char) 125, (char) 189, (char) 87, (char) 200, (char) 230, (char) 121, (char) 72, (char) 70, (char) 131, (char) 199, (char) 133, (char) 61, (char) 185, (char) 69, (char) 12, (char) 36, (char) 129, (char) 10, (char) 58, (char) 196, (char) 28, (char) 131, (char) 209, (char) 95, (char) 225, (char) 218, (char) 72, (char) 230, (char) 172, (char) 102, (char) 49, (char) 48, (char) 180, (char) 224, (char) 157, (char) 130, (char) 77, (char) 202, (char) 217, (char) 43, (char) 136, (char) 150, (char) 229, (char) 160, (char) 103, (char) 252, (char) 151, (char) 23, (char) 252, (char) 70, (char) 136, (char) 24, (char) 30, (char) 207, (char) 226, (char) 225, (char) 74, (char) 220, (char) 124, (char) 82, (char) 89, (char) 177, (char) 88, (char) 147, (char) 8, (char) 204, (char) 167, (char) 87, (char) 103, (char) 7, (char) 98, (char) 222, (char) 35, (char) 166, (char) 133, (char) 184, (char) 162, (char) 220, (char) 232, (char) 239, (char) 21, (char) 52, (char) 241, (char) 108, (char) 68, (char) 198, (char) 74, (char) 32, (char) 254, (char) 9, (char) 95, (char) 97, (char) 254, (char) 6, (char) 38, (char) 152, (char) 45, (char) 61, (char) 64, (char) 132, (char) 178, (char) 54, (char) 188, (char) 102, (char) 135, (char) 17, (char) 162, (char) 191, (char) 208, (char) 126, (char) 251, (char) 59, (char) 107, (char) 79, (char) 37, (char) 254, (char) 239, (char) 243, (char) 31, (char) 28, (char) 209, (char) 7, (char) 213, (char) 92, (char) 98, (char) 104, (char) 186, (char) 165, (char) 214, (char) 131, (char) 163, (char) 197, (char) 170, (char) 166, (char) 35, (char) 49, (char) 34, (char) 184, (char) 35, (char) 177, (char) 14, (char) 15, (char) 102}));
			assert (pack.target_component_GET() == (char) 252);
			assert (pack.target_network_GET() == (char) 23);
			assert (pack.message_type_GET() == (char) 27128);
		});
		GroundControl.V2_EXTENSION p248 = CommunicationChannel.new_V2_EXTENSION();
		PH.setPack(p248);
		p248.message_type_SET((char) 27128);
		p248.target_system_SET((char) 240);
		p248.target_component_SET((char) 252);
		p248.payload_SET(new char[]{(char) 57, (char) 12, (char) 217, (char) 42, (char) 248, (char) 49, (char) 44, (char) 252, (char) 92, (char) 153, (char) 13, (char) 127, (char) 168, (char) 119, (char) 215, (char) 194, (char) 128, (char) 68, (char) 240, (char) 69, (char) 127, (char) 166, (char) 205, (char) 150, (char) 230, (char) 170, (char) 188, (char) 61, (char) 188, (char) 112, (char) 252, (char) 78, (char) 216, (char) 8, (char) 84, (char) 151, (char) 3, (char) 45, (char) 120, (char) 183, (char) 231, (char) 142, (char) 136, (char) 7, (char) 59, (char) 253, (char) 187, (char) 30, (char) 123, (char) 96, (char) 132, (char) 233, (char) 12, (char) 215, (char) 125, (char) 205, (char) 134, (char) 199, (char) 232, (char) 76, (char) 164, (char) 34, (char) 66, (char) 92, (char) 153, (char) 193, (char) 68, (char) 224, (char) 168, (char) 60, (char) 231, (char) 99, (char) 232, (char) 124, (char) 136, (char) 213, (char) 59, (char) 62, (char) 0, (char) 63, (char) 46, (char) 49, (char) 173, (char) 174, (char) 221, (char) 134, (char) 19, (char) 70, (char) 42, (char) 34, (char) 157, (char) 192, (char) 183, (char) 99, (char) 255, (char) 160, (char) 217, (char) 17, (char) 179, (char) 85, (char) 238, (char) 189, (char) 15, (char) 255, (char) 66, (char) 77, (char) 125, (char) 189, (char) 87, (char) 200, (char) 230, (char) 121, (char) 72, (char) 70, (char) 131, (char) 199, (char) 133, (char) 61, (char) 185, (char) 69, (char) 12, (char) 36, (char) 129, (char) 10, (char) 58, (char) 196, (char) 28, (char) 131, (char) 209, (char) 95, (char) 225, (char) 218, (char) 72, (char) 230, (char) 172, (char) 102, (char) 49, (char) 48, (char) 180, (char) 224, (char) 157, (char) 130, (char) 77, (char) 202, (char) 217, (char) 43, (char) 136, (char) 150, (char) 229, (char) 160, (char) 103, (char) 252, (char) 151, (char) 23, (char) 252, (char) 70, (char) 136, (char) 24, (char) 30, (char) 207, (char) 226, (char) 225, (char) 74, (char) 220, (char) 124, (char) 82, (char) 89, (char) 177, (char) 88, (char) 147, (char) 8, (char) 204, (char) 167, (char) 87, (char) 103, (char) 7, (char) 98, (char) 222, (char) 35, (char) 166, (char) 133, (char) 184, (char) 162, (char) 220, (char) 232, (char) 239, (char) 21, (char) 52, (char) 241, (char) 108, (char) 68, (char) 198, (char) 74, (char) 32, (char) 254, (char) 9, (char) 95, (char) 97, (char) 254, (char) 6, (char) 38, (char) 152, (char) 45, (char) 61, (char) 64, (char) 132, (char) 178, (char) 54, (char) 188, (char) 102, (char) 135, (char) 17, (char) 162, (char) 191, (char) 208, (char) 126, (char) 251, (char) 59, (char) 107, (char) 79, (char) 37, (char) 254, (char) 239, (char) 243, (char) 31, (char) 28, (char) 209, (char) 7, (char) 213, (char) 92, (char) 98, (char) 104, (char) 186, (char) 165, (char) 214, (char) 131, (char) 163, (char) 197, (char) 170, (char) 166, (char) 35, (char) 49, (char) 34, (char) 184, (char) 35, (char) 177, (char) 14, (char) 15, (char) 102}, 0);
		p248.target_network_SET((char) 23);
		CommunicationChannel.instance.send(p248);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
		TestChannel.instance.on_MEMORY_VECT.add((src, ph, pack) ->
		{
			assert (Arrays.equals(pack.value_GET(), new byte[]{(byte) -99, (byte) 11, (byte) 103, (byte) 62, (byte) 31, (byte) -116, (byte) 106, (byte) 108, (byte) -50, (byte) -95, (byte) -56, (byte) 8, (byte) 86, (byte) -32, (byte) -38, (byte) -32, (byte) 19, (byte) -26, (byte) -14, (byte) -124, (byte) -2, (byte) 110, (byte) -85, (byte) -45, (byte) -17, (byte) 31, (byte) 71, (byte) -69, (byte) -41, (byte) 87, (byte) 125, (byte) 70}));
			assert (pack.ver_GET() == (char) 178);
			assert (pack.address_GET() == (char) 9328);
			assert (pack.type_GET() == (char) 57);
		});
		GroundControl.MEMORY_VECT p249 = CommunicationChannel.new_MEMORY_VECT();
		PH.setPack(p249);
		p249.value_SET(new byte[]{(byte) -99, (byte) 11, (byte) 103, (byte) 62, (byte) 31, (byte) -116, (byte) 106, (byte) 108, (byte) -50, (byte) -95, (byte) -56, (byte) 8, (byte) 86, (byte) -32, (byte) -38, (byte) -32, (byte) 19, (byte) -26, (byte) -14, (byte) -124, (byte) -2, (byte) 110, (byte) -85, (byte) -45, (byte) -17, (byte) 31, (byte) 71, (byte) -69, (byte) -41, (byte) 87, (byte) 125, (byte) 70}, 0);
		p249.type_SET((char) 57);
		p249.ver_SET((char) 178);
		p249.address_SET((char) 9328);
		CommunicationChannel.instance.send(p249);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
		TestChannel.instance.on_DEBUG_VECT.add((src, ph, pack) ->
		{
			assert (pack.z_GET() == 3.9275844E37F);
			assert (pack.y_GET() == -4.477145E37F);
			assert (pack.time_usec_GET() == 8992481302037469858L);
			assert (pack.name_LEN(ph) == 1);
			assert (pack.name_TRY(ph).equals("u"));
			assert (pack.x_GET() == 2.607199E38F);
		});
		GroundControl.DEBUG_VECT p250 = CommunicationChannel.new_DEBUG_VECT();
		PH.setPack(p250);
		p250.name_SET("u", PH);
		p250.z_SET(3.9275844E37F);
		p250.x_SET(2.607199E38F);
		p250.time_usec_SET(8992481302037469858L);
		p250.y_SET(-4.477145E37F);
		CommunicationChannel.instance.send(p250);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
		TestChannel.instance.on_NAMED_VALUE_FLOAT.add((src, ph, pack) ->
		{
			assert (pack.value_GET() == -6.7022355E37F);
			assert (pack.time_boot_ms_GET() == 2899847027L);
			assert (pack.name_LEN(ph) == 8);
			assert (pack.name_TRY(ph).equals("pVCwzclr"));
		});
		GroundControl.NAMED_VALUE_FLOAT p251 = CommunicationChannel.new_NAMED_VALUE_FLOAT();
		PH.setPack(p251);
		p251.name_SET("pVCwzclr", PH);
		p251.time_boot_ms_SET(2899847027L);
		p251.value_SET(-6.7022355E37F);
		CommunicationChannel.instance.send(p251);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
		TestChannel.instance.on_NAMED_VALUE_INT.add((src, ph, pack) ->
		{
			assert (pack.value_GET() == 846423399);
			assert (pack.name_LEN(ph) == 5);
			assert (pack.name_TRY(ph).equals("puoco"));
			assert (pack.time_boot_ms_GET() == 681453886L);
		});
		GroundControl.NAMED_VALUE_INT p252 = CommunicationChannel.new_NAMED_VALUE_INT();
		PH.setPack(p252);
		p252.value_SET(846423399);
		p252.name_SET("puoco", PH);
		p252.time_boot_ms_SET(681453886L);
		CommunicationChannel.instance.send(p252);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
		TestChannel.instance.on_STATUSTEXT.add((src, ph, pack) ->
		{
			assert (pack.text_LEN(ph) == 41);
			assert (pack.text_TRY(ph).equals("kjujefgkznhglLxpLvbMkFoyqhtwgdTllmxgPtpih"));
			assert (pack.severity_GET() == MAV_SEVERITY.MAV_SEVERITY_EMERGENCY);
		});
		GroundControl.STATUSTEXT p253 = CommunicationChannel.new_STATUSTEXT();
		PH.setPack(p253);
		p253.text_SET("kjujefgkznhglLxpLvbMkFoyqhtwgdTllmxgPtpih", PH);
		p253.severity_SET(MAV_SEVERITY.MAV_SEVERITY_EMERGENCY);
		CommunicationChannel.instance.send(p253);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
		TestChannel.instance.on_DEBUG.add((src, ph, pack) ->
		{
			assert (pack.value_GET() == -1.717441E38F);
			assert (pack.ind_GET() == (char) 32);
			assert (pack.time_boot_ms_GET() == 1010442512L);
		});
		GroundControl.DEBUG p254 = CommunicationChannel.new_DEBUG();
		PH.setPack(p254);
		p254.ind_SET((char) 32);
		p254.time_boot_ms_SET(1010442512L);
		p254.value_SET(-1.717441E38F);
		CommunicationChannel.instance.send(p254);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
		TestChannel.instance.on_SETUP_SIGNING.add((src, ph, pack) ->
		{
			assert (pack.target_system_GET() == (char) 128);
			assert (Arrays.equals(pack.secret_key_GET(), new char[]{(char) 57, (char) 174, (char) 147, (char) 43, (char) 166, (char) 134, (char) 114, (char) 64, (char) 95, (char) 140, (char) 122, (char) 226, (char) 136, (char) 84, (char) 250, (char) 213, (char) 241, (char) 106, (char) 110, (char) 79, (char) 29, (char) 117, (char) 115, (char) 25, (char) 1, (char) 211, (char) 196, (char) 5, (char) 152, (char) 85, (char) 125, (char) 251}));
			assert (pack.target_component_GET() == (char) 166);
			assert (pack.initial_timestamp_GET() == 1311949306821051420L);
		});
		GroundControl.SETUP_SIGNING p256 = CommunicationChannel.new_SETUP_SIGNING();
		PH.setPack(p256);
		p256.initial_timestamp_SET(1311949306821051420L);
		p256.target_system_SET((char) 128);
		p256.secret_key_SET(new char[]{(char) 57, (char) 174, (char) 147, (char) 43, (char) 166, (char) 134, (char) 114, (char) 64, (char) 95, (char) 140, (char) 122, (char) 226, (char) 136, (char) 84, (char) 250, (char) 213, (char) 241, (char) 106, (char) 110, (char) 79, (char) 29, (char) 117, (char) 115, (char) 25, (char) 1, (char) 211, (char) 196, (char) 5, (char) 152, (char) 85, (char) 125, (char) 251}, 0);
		p256.target_component_SET((char) 166);
		CommunicationChannel.instance.send(p256);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
		TestChannel.instance.on_BUTTON_CHANGE.add((src, ph, pack) ->
		{
			assert (pack.time_boot_ms_GET() == 1345692246L);
			assert (pack.last_change_ms_GET() == 148370965L);
			assert (pack.state_GET() == (char) 207);
		});
		GroundControl.BUTTON_CHANGE p257 = CommunicationChannel.new_BUTTON_CHANGE();
		PH.setPack(p257);
		p257.last_change_ms_SET(148370965L);
		p257.state_SET((char) 207);
		p257.time_boot_ms_SET(1345692246L);
		CommunicationChannel.instance.send(p257);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
		TestChannel.instance.on_PLAY_TUNE.add((src, ph, pack) ->
		{
			assert (pack.target_system_GET() == (char) 201);
			assert (pack.target_component_GET() == (char) 158);
			assert (pack.tune_LEN(ph) == 14);
			assert (pack.tune_TRY(ph).equals("hxulteslgerwJq"));
		});
		GroundControl.PLAY_TUNE p258 = CommunicationChannel.new_PLAY_TUNE();
		PH.setPack(p258);
		p258.tune_SET("hxulteslgerwJq", PH);
		p258.target_component_SET((char) 158);
		p258.target_system_SET((char) 201);
		CommunicationChannel.instance.send(p258);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
		TestChannel.instance.on_CAMERA_INFORMATION.add((src, ph, pack) ->
		{
			assert (Arrays.equals(pack.model_name_GET(), new char[]{(char) 190, (char) 28, (char) 87, (char) 4, (char) 162, (char) 32, (char) 115, (char) 132, (char) 23, (char) 26, (char) 65, (char) 132, (char) 70, (char) 94, (char) 157, (char) 208, (char) 77, (char) 232, (char) 229, (char) 239, (char) 156, (char) 144, (char) 90, (char) 18, (char) 92, (char) 94, (char) 97, (char) 60, (char) 100, (char) 228, (char) 78, (char) 58}));
			assert (pack.firmware_version_GET() == 3012060642L);
			assert (pack.sensor_size_h_GET() == -5.6569816E37F);
			assert (pack.focal_length_GET() == 2.8580598E38F);
			assert (pack.cam_definition_version_GET() == (char) 58043);
			assert (pack.resolution_h_GET() == (char) 60294);
			assert (Arrays.equals(pack.vendor_name_GET(), new char[]{(char) 182, (char) 171, (char) 121, (char) 229, (char) 204, (char) 49, (char) 131, (char) 71, (char) 173, (char) 34, (char) 68, (char) 251, (char) 165, (char) 220, (char) 15, (char) 69, (char) 246, (char) 140, (char) 239, (char) 235, (char) 255, (char) 97, (char) 181, (char) 27, (char) 184, (char) 18, (char) 248, (char) 188, (char) 174, (char) 201, (char) 159, (char) 162}));
			assert (pack.resolution_v_GET() == (char) 57071);
			assert (pack.cam_definition_uri_LEN(ph) == 126);
			assert (pack.cam_definition_uri_TRY(ph).equals("fqbpkkyfsiirgzjirhuirwwufHicwxgydottuiqikmujWLnpkpqfhPoichatRyvbcwdvrxqhJnoQspgtiargnrrzrphpyRhUfrfkduzotlhymTcflduXklnuxvoimh"));
			assert (pack.lens_id_GET() == (char) 190);
			assert (pack.sensor_size_v_GET() == 2.2078456E38F);
			assert (pack.flags_GET() == CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_VIDEO);
			assert (pack.time_boot_ms_GET() == 55180475L);
		});
		GroundControl.CAMERA_INFORMATION p259 = CommunicationChannel.new_CAMERA_INFORMATION();
		PH.setPack(p259);
		p259.flags_SET(CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_VIDEO);
		p259.model_name_SET(new char[]{(char) 190, (char) 28, (char) 87, (char) 4, (char) 162, (char) 32, (char) 115, (char) 132, (char) 23, (char) 26, (char) 65, (char) 132, (char) 70, (char) 94, (char) 157, (char) 208, (char) 77, (char) 232, (char) 229, (char) 239, (char) 156, (char) 144, (char) 90, (char) 18, (char) 92, (char) 94, (char) 97, (char) 60, (char) 100, (char) 228, (char) 78, (char) 58}, 0);
		p259.focal_length_SET(2.8580598E38F);
		p259.cam_definition_uri_SET("fqbpkkyfsiirgzjirhuirwwufHicwxgydottuiqikmujWLnpkpqfhPoichatRyvbcwdvrxqhJnoQspgtiargnrrzrphpyRhUfrfkduzotlhymTcflduXklnuxvoimh", PH);
		p259.resolution_v_SET((char) 57071);
		p259.cam_definition_version_SET((char) 58043);
		p259.time_boot_ms_SET(55180475L);
		p259.firmware_version_SET(3012060642L);
		p259.vendor_name_SET(new char[]{(char) 182, (char) 171, (char) 121, (char) 229, (char) 204, (char) 49, (char) 131, (char) 71, (char) 173, (char) 34, (char) 68, (char) 251, (char) 165, (char) 220, (char) 15, (char) 69, (char) 246, (char) 140, (char) 239, (char) 235, (char) 255, (char) 97, (char) 181, (char) 27, (char) 184, (char) 18, (char) 248, (char) 188, (char) 174, (char) 201, (char) 159, (char) 162}, 0);
		p259.resolution_h_SET((char) 60294);
		p259.sensor_size_v_SET(2.2078456E38F);
		p259.sensor_size_h_SET(-5.6569816E37F);
		p259.lens_id_SET((char) 190);
		CommunicationChannel.instance.send(p259);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
		TestChannel.instance.on_CAMERA_SETTINGS.add((src, ph, pack) ->
		{
			assert (pack.time_boot_ms_GET() == 1288154163L);
			assert (pack.mode_id_GET() == CAMERA_MODE.CAMERA_MODE_VIDEO);
		});
		GroundControl.CAMERA_SETTINGS p260 = CommunicationChannel.new_CAMERA_SETTINGS();
		PH.setPack(p260);
		p260.mode_id_SET(CAMERA_MODE.CAMERA_MODE_VIDEO);
		p260.time_boot_ms_SET(1288154163L);
		CommunicationChannel.instance.send(p260);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
		TestChannel.instance.on_STORAGE_INFORMATION.add((src, ph, pack) ->
		{
			assert (pack.read_speed_GET() == 5.894336E37F);
			assert (pack.status_GET() == (char) 41);
			assert (pack.storage_id_GET() == (char) 46);
			assert (pack.available_capacity_GET() == 3.0448568E38F);
			assert (pack.used_capacity_GET() == -2.8945416E38F);
			assert (pack.storage_count_GET() == (char) 98);
			assert (pack.write_speed_GET() == -3.0089046E38F);
			assert (pack.time_boot_ms_GET() == 3497983921L);
			assert (pack.total_capacity_GET() == -1.1099445E38F);
		});
		GroundControl.STORAGE_INFORMATION p261 = CommunicationChannel.new_STORAGE_INFORMATION();
		PH.setPack(p261);
		p261.status_SET((char) 41);
		p261.time_boot_ms_SET(3497983921L);
		p261.read_speed_SET(5.894336E37F);
		p261.write_speed_SET(-3.0089046E38F);
		p261.storage_id_SET((char) 46);
		p261.used_capacity_SET(-2.8945416E38F);
		p261.total_capacity_SET(-1.1099445E38F);
		p261.storage_count_SET((char) 98);
		p261.available_capacity_SET(3.0448568E38F);
		CommunicationChannel.instance.send(p261);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
		TestChannel.instance.on_CAMERA_CAPTURE_STATUS.add((src, ph, pack) ->
		{
			assert (pack.available_capacity_GET() == 1.8292766E38F);
			assert (pack.image_interval_GET() == 1.2197938E38F);
			assert (pack.time_boot_ms_GET() == 3242089888L);
			assert (pack.image_status_GET() == (char) 229);
			assert (pack.video_status_GET() == (char) 245);
			assert (pack.recording_time_ms_GET() == 3145862664L);
		});
		GroundControl.CAMERA_CAPTURE_STATUS p262 = CommunicationChannel.new_CAMERA_CAPTURE_STATUS();
		PH.setPack(p262);
		p262.image_status_SET((char) 229);
		p262.recording_time_ms_SET(3145862664L);
		p262.time_boot_ms_SET(3242089888L);
		p262.video_status_SET((char) 245);
		p262.image_interval_SET(1.2197938E38F);
		p262.available_capacity_SET(1.8292766E38F);
		CommunicationChannel.instance.send(p262);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
		TestChannel.instance.on_CAMERA_IMAGE_CAPTURED.add((src, ph, pack) ->
		{
			assert (Arrays.equals(pack.q_GET(), new float[]{-7.18425E37F, 1.908397E38F, 2.21989E38F, 8.81966E37F}));
			assert (pack.file_url_LEN(ph) == 4);
			assert (pack.file_url_TRY(ph).equals("zoij"));
			assert (pack.alt_GET() == 1373821398);
			assert (pack.time_utc_GET() == 3100695829257729660L);
			assert (pack.camera_id_GET() == (char) 32);
			assert (pack.time_boot_ms_GET() == 3882059764L);
			assert (pack.lon_GET() == 879372507);
			assert (pack.image_index_GET() == -1132924796);
			assert (pack.lat_GET() == -1918588711);
			assert (pack.capture_result_GET() == (byte) -98);
			assert (pack.relative_alt_GET() == -1278780203);
		});
		GroundControl.CAMERA_IMAGE_CAPTURED p263 = CommunicationChannel.new_CAMERA_IMAGE_CAPTURED();
		PH.setPack(p263);
		p263.image_index_SET(-1132924796);
		p263.time_utc_SET(3100695829257729660L);
		p263.camera_id_SET((char) 32);
		p263.file_url_SET("zoij", PH);
		p263.lat_SET(-1918588711);
		p263.lon_SET(879372507);
		p263.q_SET(new float[]{-7.18425E37F, 1.908397E38F, 2.21989E38F, 8.81966E37F}, 0);
		p263.relative_alt_SET(-1278780203);
		p263.time_boot_ms_SET(3882059764L);
		p263.capture_result_SET((byte) -98);
		p263.alt_SET(1373821398);
		CommunicationChannel.instance.send(p263);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
		TestChannel.instance.on_FLIGHT_INFORMATION.add((src, ph, pack) ->
		{
			assert (pack.time_boot_ms_GET() == 388036136L);
			assert (pack.arming_time_utc_GET() == 9127657815985006530L);
			assert (pack.flight_uuid_GET() == 2175773317558674366L);
			assert (pack.takeoff_time_utc_GET() == 9090859319870034040L);
		});
		GroundControl.FLIGHT_INFORMATION p264 = CommunicationChannel.new_FLIGHT_INFORMATION();
		PH.setPack(p264);
		p264.time_boot_ms_SET(388036136L);
		p264.flight_uuid_SET(2175773317558674366L);
		p264.takeoff_time_utc_SET(9090859319870034040L);
		p264.arming_time_utc_SET(9127657815985006530L);
		CommunicationChannel.instance.send(p264);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
		TestChannel.instance.on_MOUNT_ORIENTATION.add((src, ph, pack) ->
		{
			assert (pack.time_boot_ms_GET() == 1595587599L);
			assert (pack.pitch_GET() == -1.3650406E38F);
			assert (pack.yaw_GET() == -2.5156461E38F);
			assert (pack.roll_GET() == -2.4893514E38F);
		});
		GroundControl.MOUNT_ORIENTATION p265 = CommunicationChannel.new_MOUNT_ORIENTATION();
		PH.setPack(p265);
		p265.pitch_SET(-1.3650406E38F);
		p265.roll_SET(-2.4893514E38F);
		p265.yaw_SET(-2.5156461E38F);
		p265.time_boot_ms_SET(1595587599L);
		CommunicationChannel.instance.send(p265);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
		TestChannel.instance.on_LOGGING_DATA.add((src, ph, pack) ->
		{
			assert (pack.target_component_GET() == (char) 232);
			assert (pack.length_GET() == (char) 225);
			assert (pack.target_system_GET() == (char) 149);
			assert (Arrays.equals(pack.data__GET(), new char[]{(char) 150, (char) 3, (char) 126, (char) 141, (char) 110, (char) 201, (char) 236, (char) 231, (char) 199, (char) 56, (char) 58, (char) 116, (char) 178, (char) 6, (char) 179, (char) 220, (char) 108, (char) 8, (char) 41, (char) 189, (char) 31, (char) 52, (char) 103, (char) 253, (char) 116, (char) 66, (char) 167, (char) 51, (char) 225, (char) 71, (char) 68, (char) 103, (char) 57, (char) 71, (char) 109, (char) 247, (char) 99, (char) 189, (char) 187, (char) 18, (char) 50, (char) 144, (char) 187, (char) 65, (char) 68, (char) 22, (char) 247, (char) 103, (char) 37, (char) 126, (char) 110, (char) 225, (char) 0, (char) 12, (char) 243, (char) 148, (char) 42, (char) 64, (char) 195, (char) 142, (char) 232, (char) 85, (char) 12, (char) 81, (char) 240, (char) 129, (char) 163, (char) 123, (char) 37, (char) 95, (char) 101, (char) 184, (char) 10, (char) 142, (char) 231, (char) 95, (char) 235, (char) 225, (char) 29, (char) 183, (char) 43, (char) 251, (char) 166, (char) 194, (char) 97, (char) 126, (char) 100, (char) 32, (char) 23, (char) 57, (char) 251, (char) 187, (char) 61, (char) 34, (char) 244, (char) 203, (char) 67, (char) 85, (char) 129, (char) 165, (char) 236, (char) 10, (char) 113, (char) 137, (char) 173, (char) 220, (char) 131, (char) 175, (char) 58, (char) 13, (char) 99, (char) 159, (char) 146, (char) 45, (char) 156, (char) 5, (char) 202, (char) 177, (char) 223, (char) 3, (char) 218, (char) 173, (char) 205, (char) 130, (char) 206, (char) 102, (char) 19, (char) 18, (char) 45, (char) 36, (char) 115, (char) 28, (char) 124, (char) 6, (char) 217, (char) 168, (char) 40, (char) 69, (char) 100, (char) 249, (char) 58, (char) 150, (char) 231, (char) 210, (char) 143, (char) 73, (char) 148, (char) 66, (char) 152, (char) 212, (char) 103, (char) 238, (char) 88, (char) 23, (char) 214, (char) 13, (char) 111, (char) 240, (char) 93, (char) 36, (char) 203, (char) 236, (char) 36, (char) 116, (char) 141, (char) 13, (char) 5, (char) 135, (char) 227, (char) 219, (char) 160, (char) 52, (char) 97, (char) 83, (char) 237, (char) 35, (char) 145, (char) 5, (char) 248, (char) 123, (char) 118, (char) 134, (char) 43, (char) 172, (char) 120, (char) 14, (char) 207, (char) 96, (char) 1, (char) 55, (char) 201, (char) 192, (char) 207, (char) 48, (char) 128, (char) 239, (char) 124, (char) 6, (char) 235, (char) 62, (char) 175, (char) 105, (char) 192, (char) 212, (char) 225, (char) 50, (char) 30, (char) 252, (char) 95, (char) 35, (char) 143, (char) 1, (char) 35, (char) 159, (char) 142, (char) 132, (char) 228, (char) 35, (char) 208, (char) 172, (char) 2, (char) 235, (char) 79, (char) 51, (char) 23, (char) 157, (char) 104, (char) 201, (char) 154, (char) 137, (char) 52, (char) 1, (char) 59, (char) 127, (char) 76, (char) 20, (char) 18, (char) 74, (char) 100, (char) 99, (char) 190, (char) 212, (char) 180, (char) 139, (char) 176, (char) 74, (char) 27, (char) 80, (char) 53}));
			assert (pack.sequence_GET() == (char) 54605);
			assert (pack.first_message_offset_GET() == (char) 106);
		});
		GroundControl.LOGGING_DATA p266 = CommunicationChannel.new_LOGGING_DATA();
		PH.setPack(p266);
		p266.sequence_SET((char) 54605);
		p266.target_component_SET((char) 232);
		p266.target_system_SET((char) 149);
		p266.data__SET(new char[]{(char) 150, (char) 3, (char) 126, (char) 141, (char) 110, (char) 201, (char) 236, (char) 231, (char) 199, (char) 56, (char) 58, (char) 116, (char) 178, (char) 6, (char) 179, (char) 220, (char) 108, (char) 8, (char) 41, (char) 189, (char) 31, (char) 52, (char) 103, (char) 253, (char) 116, (char) 66, (char) 167, (char) 51, (char) 225, (char) 71, (char) 68, (char) 103, (char) 57, (char) 71, (char) 109, (char) 247, (char) 99, (char) 189, (char) 187, (char) 18, (char) 50, (char) 144, (char) 187, (char) 65, (char) 68, (char) 22, (char) 247, (char) 103, (char) 37, (char) 126, (char) 110, (char) 225, (char) 0, (char) 12, (char) 243, (char) 148, (char) 42, (char) 64, (char) 195, (char) 142, (char) 232, (char) 85, (char) 12, (char) 81, (char) 240, (char) 129, (char) 163, (char) 123, (char) 37, (char) 95, (char) 101, (char) 184, (char) 10, (char) 142, (char) 231, (char) 95, (char) 235, (char) 225, (char) 29, (char) 183, (char) 43, (char) 251, (char) 166, (char) 194, (char) 97, (char) 126, (char) 100, (char) 32, (char) 23, (char) 57, (char) 251, (char) 187, (char) 61, (char) 34, (char) 244, (char) 203, (char) 67, (char) 85, (char) 129, (char) 165, (char) 236, (char) 10, (char) 113, (char) 137, (char) 173, (char) 220, (char) 131, (char) 175, (char) 58, (char) 13, (char) 99, (char) 159, (char) 146, (char) 45, (char) 156, (char) 5, (char) 202, (char) 177, (char) 223, (char) 3, (char) 218, (char) 173, (char) 205, (char) 130, (char) 206, (char) 102, (char) 19, (char) 18, (char) 45, (char) 36, (char) 115, (char) 28, (char) 124, (char) 6, (char) 217, (char) 168, (char) 40, (char) 69, (char) 100, (char) 249, (char) 58, (char) 150, (char) 231, (char) 210, (char) 143, (char) 73, (char) 148, (char) 66, (char) 152, (char) 212, (char) 103, (char) 238, (char) 88, (char) 23, (char) 214, (char) 13, (char) 111, (char) 240, (char) 93, (char) 36, (char) 203, (char) 236, (char) 36, (char) 116, (char) 141, (char) 13, (char) 5, (char) 135, (char) 227, (char) 219, (char) 160, (char) 52, (char) 97, (char) 83, (char) 237, (char) 35, (char) 145, (char) 5, (char) 248, (char) 123, (char) 118, (char) 134, (char) 43, (char) 172, (char) 120, (char) 14, (char) 207, (char) 96, (char) 1, (char) 55, (char) 201, (char) 192, (char) 207, (char) 48, (char) 128, (char) 239, (char) 124, (char) 6, (char) 235, (char) 62, (char) 175, (char) 105, (char) 192, (char) 212, (char) 225, (char) 50, (char) 30, (char) 252, (char) 95, (char) 35, (char) 143, (char) 1, (char) 35, (char) 159, (char) 142, (char) 132, (char) 228, (char) 35, (char) 208, (char) 172, (char) 2, (char) 235, (char) 79, (char) 51, (char) 23, (char) 157, (char) 104, (char) 201, (char) 154, (char) 137, (char) 52, (char) 1, (char) 59, (char) 127, (char) 76, (char) 20, (char) 18, (char) 74, (char) 100, (char) 99, (char) 190, (char) 212, (char) 180, (char) 139, (char) 176, (char) 74, (char) 27, (char) 80, (char) 53}, 0);
		p266.length_SET((char) 225);
		p266.first_message_offset_SET((char) 106);
		CommunicationChannel.instance.send(p266);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
		TestChannel.instance.on_LOGGING_DATA_ACKED.add((src, ph, pack) ->
		{
			assert (pack.target_system_GET() == (char) 34);
			assert (pack.length_GET() == (char) 239);
			assert (pack.target_component_GET() == (char) 64);
			assert (pack.sequence_GET() == (char) 27415);
			assert (pack.first_message_offset_GET() == (char) 164);
			assert (Arrays.equals(pack.data__GET(), new char[]{(char) 47, (char) 7, (char) 77, (char) 181, (char) 245, (char) 167, (char) 4, (char) 226, (char) 126, (char) 210, (char) 192, (char) 82, (char) 205, (char) 169, (char) 125, (char) 138, (char) 220, (char) 44, (char) 159, (char) 162, (char) 151, (char) 61, (char) 25, (char) 109, (char) 34, (char) 244, (char) 33, (char) 184, (char) 217, (char) 112, (char) 121, (char) 71, (char) 206, (char) 121, (char) 164, (char) 133, (char) 137, (char) 22, (char) 154, (char) 253, (char) 137, (char) 1, (char) 1, (char) 50, (char) 157, (char) 95, (char) 44, (char) 12, (char) 216, (char) 179, (char) 194, (char) 119, (char) 146, (char) 227, (char) 42, (char) 157, (char) 12, (char) 238, (char) 141, (char) 193, (char) 67, (char) 75, (char) 146, (char) 106, (char) 145, (char) 203, (char) 42, (char) 99, (char) 141, (char) 155, (char) 57, (char) 105, (char) 45, (char) 218, (char) 63, (char) 24, (char) 40, (char) 229, (char) 58, (char) 171, (char) 246, (char) 102, (char) 44, (char) 213, (char) 228, (char) 189, (char) 92, (char) 120, (char) 210, (char) 138, (char) 85, (char) 143, (char) 146, (char) 31, (char) 241, (char) 144, (char) 252, (char) 187, (char) 131, (char) 138, (char) 195, (char) 158, (char) 125, (char) 254, (char) 204, (char) 179, (char) 183, (char) 192, (char) 34, (char) 121, (char) 51, (char) 152, (char) 245, (char) 139, (char) 168, (char) 56, (char) 242, (char) 237, (char) 40, (char) 216, (char) 98, (char) 255, (char) 68, (char) 65, (char) 135, (char) 251, (char) 18, (char) 230, (char) 44, (char) 18, (char) 97, (char) 125, (char) 116, (char) 197, (char) 144, (char) 10, (char) 7, (char) 8, (char) 157, (char) 187, (char) 30, (char) 175, (char) 48, (char) 138, (char) 47, (char) 190, (char) 222, (char) 134, (char) 8, (char) 130, (char) 75, (char) 214, (char) 19, (char) 199, (char) 157, (char) 186, (char) 107, (char) 82, (char) 227, (char) 242, (char) 34, (char) 239, (char) 183, (char) 194, (char) 229, (char) 231, (char) 209, (char) 73, (char) 117, (char) 253, (char) 41, (char) 167, (char) 213, (char) 195, (char) 195, (char) 199, (char) 181, (char) 66, (char) 44, (char) 24, (char) 136, (char) 166, (char) 12, (char) 126, (char) 30, (char) 72, (char) 65, (char) 188, (char) 44, (char) 1, (char) 130, (char) 85, (char) 186, (char) 119, (char) 104, (char) 35, (char) 95, (char) 245, (char) 32, (char) 198, (char) 145, (char) 158, (char) 18, (char) 177, (char) 139, (char) 125, (char) 158, (char) 99, (char) 194, (char) 82, (char) 193, (char) 143, (char) 31, (char) 2, (char) 18, (char) 240, (char) 237, (char) 54, (char) 130, (char) 199, (char) 55, (char) 168, (char) 79, (char) 44, (char) 54, (char) 75, (char) 41, (char) 120, (char) 219, (char) 244, (char) 129, (char) 223, (char) 170, (char) 227, (char) 112, (char) 1, (char) 220, (char) 211, (char) 199, (char) 75, (char) 188, (char) 180, (char) 231, (char) 120, (char) 54, (char) 199, (char) 140, (char) 44, (char) 118}));
		});
		GroundControl.LOGGING_DATA_ACKED p267 = CommunicationChannel.new_LOGGING_DATA_ACKED();
		PH.setPack(p267);
		p267.first_message_offset_SET((char) 164);
		p267.length_SET((char) 239);
		p267.target_system_SET((char) 34);
		p267.sequence_SET((char) 27415);
		p267.data__SET(new char[]{(char) 47, (char) 7, (char) 77, (char) 181, (char) 245, (char) 167, (char) 4, (char) 226, (char) 126, (char) 210, (char) 192, (char) 82, (char) 205, (char) 169, (char) 125, (char) 138, (char) 220, (char) 44, (char) 159, (char) 162, (char) 151, (char) 61, (char) 25, (char) 109, (char) 34, (char) 244, (char) 33, (char) 184, (char) 217, (char) 112, (char) 121, (char) 71, (char) 206, (char) 121, (char) 164, (char) 133, (char) 137, (char) 22, (char) 154, (char) 253, (char) 137, (char) 1, (char) 1, (char) 50, (char) 157, (char) 95, (char) 44, (char) 12, (char) 216, (char) 179, (char) 194, (char) 119, (char) 146, (char) 227, (char) 42, (char) 157, (char) 12, (char) 238, (char) 141, (char) 193, (char) 67, (char) 75, (char) 146, (char) 106, (char) 145, (char) 203, (char) 42, (char) 99, (char) 141, (char) 155, (char) 57, (char) 105, (char) 45, (char) 218, (char) 63, (char) 24, (char) 40, (char) 229, (char) 58, (char) 171, (char) 246, (char) 102, (char) 44, (char) 213, (char) 228, (char) 189, (char) 92, (char) 120, (char) 210, (char) 138, (char) 85, (char) 143, (char) 146, (char) 31, (char) 241, (char) 144, (char) 252, (char) 187, (char) 131, (char) 138, (char) 195, (char) 158, (char) 125, (char) 254, (char) 204, (char) 179, (char) 183, (char) 192, (char) 34, (char) 121, (char) 51, (char) 152, (char) 245, (char) 139, (char) 168, (char) 56, (char) 242, (char) 237, (char) 40, (char) 216, (char) 98, (char) 255, (char) 68, (char) 65, (char) 135, (char) 251, (char) 18, (char) 230, (char) 44, (char) 18, (char) 97, (char) 125, (char) 116, (char) 197, (char) 144, (char) 10, (char) 7, (char) 8, (char) 157, (char) 187, (char) 30, (char) 175, (char) 48, (char) 138, (char) 47, (char) 190, (char) 222, (char) 134, (char) 8, (char) 130, (char) 75, (char) 214, (char) 19, (char) 199, (char) 157, (char) 186, (char) 107, (char) 82, (char) 227, (char) 242, (char) 34, (char) 239, (char) 183, (char) 194, (char) 229, (char) 231, (char) 209, (char) 73, (char) 117, (char) 253, (char) 41, (char) 167, (char) 213, (char) 195, (char) 195, (char) 199, (char) 181, (char) 66, (char) 44, (char) 24, (char) 136, (char) 166, (char) 12, (char) 126, (char) 30, (char) 72, (char) 65, (char) 188, (char) 44, (char) 1, (char) 130, (char) 85, (char) 186, (char) 119, (char) 104, (char) 35, (char) 95, (char) 245, (char) 32, (char) 198, (char) 145, (char) 158, (char) 18, (char) 177, (char) 139, (char) 125, (char) 158, (char) 99, (char) 194, (char) 82, (char) 193, (char) 143, (char) 31, (char) 2, (char) 18, (char) 240, (char) 237, (char) 54, (char) 130, (char) 199, (char) 55, (char) 168, (char) 79, (char) 44, (char) 54, (char) 75, (char) 41, (char) 120, (char) 219, (char) 244, (char) 129, (char) 223, (char) 170, (char) 227, (char) 112, (char) 1, (char) 220, (char) 211, (char) 199, (char) 75, (char) 188, (char) 180, (char) 231, (char) 120, (char) 54, (char) 199, (char) 140, (char) 44, (char) 118}, 0);
		p267.target_component_SET((char) 64);
		CommunicationChannel.instance.send(p267);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
		TestChannel.instance.on_LOGGING_ACK.add((src, ph, pack) ->
		{
			assert (pack.sequence_GET() == (char) 51607);
			assert (pack.target_component_GET() == (char) 174);
			assert (pack.target_system_GET() == (char) 172);
		});
		GroundControl.LOGGING_ACK p268 = CommunicationChannel.new_LOGGING_ACK();
		PH.setPack(p268);
		p268.target_component_SET((char) 174);
		p268.sequence_SET((char) 51607);
		p268.target_system_SET((char) 172);
		CommunicationChannel.instance.send(p268);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
		TestChannel.instance.on_VIDEO_STREAM_INFORMATION.add((src, ph, pack) ->
		{
			assert (pack.rotation_GET() == (char) 14742);
			assert (pack.bitrate_GET() == 719041123L);
			assert (pack.resolution_v_GET() == (char) 31246);
			assert (pack.status_GET() == (char) 37);
			assert (pack.uri_LEN(ph) == 104);
			assert (pack.uri_TRY(ph).equals("deqymqulqxuhgoifwiPzmfyFqntasarlVcdpkqJesgddlkqcttohktJvzfvhjiTkwevyjuYvltwpRymsGtNtuxdkjmnarvgdxlnmiTgv"));
			assert (pack.camera_id_GET() == (char) 158);
			assert (pack.resolution_h_GET() == (char) 6174);
			assert (pack.framerate_GET() == -2.095497E38F);
		});
		GroundControl.VIDEO_STREAM_INFORMATION p269 = CommunicationChannel.new_VIDEO_STREAM_INFORMATION();
		PH.setPack(p269);
		p269.camera_id_SET((char) 158);
		p269.bitrate_SET(719041123L);
		p269.resolution_h_SET((char) 6174);
		p269.resolution_v_SET((char) 31246);
		p269.status_SET((char) 37);
		p269.framerate_SET(-2.095497E38F);
		p269.rotation_SET((char) 14742);
		p269.uri_SET("deqymqulqxuhgoifwiPzmfyFqntasarlVcdpkqJesgddlkqcttohktJvzfvhjiTkwevyjuYvltwpRymsGtNtuxdkjmnarvgdxlnmiTgv", PH);
		CommunicationChannel.instance.send(p269);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
		TestChannel.instance.on_SET_VIDEO_STREAM_SETTINGS.add((src, ph, pack) ->
		{
			assert (pack.camera_id_GET() == (char) 148);
			assert (pack.resolution_v_GET() == (char) 4926);
			assert (pack.rotation_GET() == (char) 52259);
			assert (pack.bitrate_GET() == 294801888L);
			assert (pack.target_component_GET() == (char) 45);
			assert (pack.resolution_h_GET() == (char) 6305);
			assert (pack.uri_LEN(ph) == 10);
			assert (pack.uri_TRY(ph).equals("smJmbxkkfa"));
			assert (pack.framerate_GET() == -2.0812772E38F);
			assert (pack.target_system_GET() == (char) 201);
		});
		GroundControl.SET_VIDEO_STREAM_SETTINGS p270 = CommunicationChannel.new_SET_VIDEO_STREAM_SETTINGS();
		PH.setPack(p270);
		p270.bitrate_SET(294801888L);
		p270.target_system_SET((char) 201);
		p270.rotation_SET((char) 52259);
		p270.camera_id_SET((char) 148);
		p270.uri_SET("smJmbxkkfa", PH);
		p270.resolution_h_SET((char) 6305);
		p270.resolution_v_SET((char) 4926);
		p270.target_component_SET((char) 45);
		p270.framerate_SET(-2.0812772E38F);
		CommunicationChannel.instance.send(p270);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
		TestChannel.instance.on_WIFI_CONFIG_AP.add((src, ph, pack) ->
		{
			assert (pack.password_LEN(ph) == 39);
			assert (pack.password_TRY(ph).equals("TDliwqmraqigFthckhkcHnbeetmovdsnsxDxlcM"));
			assert (pack.ssid_LEN(ph) == 1);
			assert (pack.ssid_TRY(ph).equals("f"));
		});
		GroundControl.WIFI_CONFIG_AP p299 = CommunicationChannel.new_WIFI_CONFIG_AP();
		PH.setPack(p299);
		p299.ssid_SET("f", PH);
		p299.password_SET("TDliwqmraqigFthckhkcHnbeetmovdsnsxDxlcM", PH);
		CommunicationChannel.instance.send(p299);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
		TestChannel.instance.on_PROTOCOL_VERSION.add((src, ph, pack) ->
		{
			assert (pack.min_version_GET() == (char) 36118);
			assert (pack.max_version_GET() == (char) 758);
			assert (Arrays.equals(pack.spec_version_hash_GET(), new char[]{(char) 229, (char) 255, (char) 30, (char) 41, (char) 55, (char) 6, (char) 197, (char) 158}));
			assert (pack.version_GET() == (char) 38145);
			assert (Arrays.equals(pack.library_version_hash_GET(), new char[]{(char) 236, (char) 178, (char) 86, (char) 29, (char) 80, (char) 163, (char) 118, (char) 35}));
		});
		GroundControl.PROTOCOL_VERSION p300 = CommunicationChannel.new_PROTOCOL_VERSION();
		PH.setPack(p300);
		p300.library_version_hash_SET(new char[]{(char) 236, (char) 178, (char) 86, (char) 29, (char) 80, (char) 163, (char) 118, (char) 35}, 0);
		p300.max_version_SET((char) 758);
		p300.spec_version_hash_SET(new char[]{(char) 229, (char) 255, (char) 30, (char) 41, (char) 55, (char) 6, (char) 197, (char) 158}, 0);
		p300.min_version_SET((char) 36118);
		p300.version_SET((char) 38145);
		CommunicationChannel.instance.send(p300);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
		TestChannel.instance.on_UAVCAN_NODE_STATUS.add((src, ph, pack) ->
		{
			assert (pack.health_GET() == UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_CRITICAL);
			assert (pack.mode_GET() == UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_INITIALIZATION);
			assert (pack.uptime_sec_GET() == 3566005324L);
			assert (pack.vendor_specific_status_code_GET() == (char) 50496);
			assert (pack.time_usec_GET() == 402710309634279831L);
			assert (pack.sub_mode_GET() == (char) 189);
		});
		GroundControl.UAVCAN_NODE_STATUS p310 = CommunicationChannel.new_UAVCAN_NODE_STATUS();
		PH.setPack(p310);
		p310.time_usec_SET(402710309634279831L);
		p310.mode_SET(UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_INITIALIZATION);
		p310.sub_mode_SET((char) 189);
		p310.uptime_sec_SET(3566005324L);
		p310.vendor_specific_status_code_SET((char) 50496);
		p310.health_SET(UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_CRITICAL);
		CommunicationChannel.instance.send(p310);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
		TestChannel.instance.on_UAVCAN_NODE_INFO.add((src, ph, pack) ->
		{
			assert (Arrays.equals(pack.hw_unique_id_GET(), new char[]{(char) 66, (char) 23, (char) 251, (char) 38, (char) 176, (char) 235, (char) 153, (char) 131, (char) 46, (char) 178, (char) 193, (char) 110, (char) 168, (char) 178, (char) 73, (char) 70}));
			assert (pack.time_usec_GET() == 1146673911249588004L);
			assert (pack.sw_version_major_GET() == (char) 85);
			assert (pack.sw_vcs_commit_GET() == 2526888681L);
			assert (pack.name_LEN(ph) == 68);
			assert (pack.name_TRY(ph).equals("giadqgcsduqkxcdNjqzyFxtrsiacmjIHxaesdfzuqxgofjcitmbuuuhweiwswxiwnkbt"));
			assert (pack.hw_version_minor_GET() == (char) 113);
			assert (pack.sw_version_minor_GET() == (char) 132);
			assert (pack.hw_version_major_GET() == (char) 114);
			assert (pack.uptime_sec_GET() == 3774169061L);
		});
		GroundControl.UAVCAN_NODE_INFO p311 = CommunicationChannel.new_UAVCAN_NODE_INFO();
		PH.setPack(p311);
		p311.hw_unique_id_SET(new char[]{(char) 66, (char) 23, (char) 251, (char) 38, (char) 176, (char) 235, (char) 153, (char) 131, (char) 46, (char) 178, (char) 193, (char) 110, (char) 168, (char) 178, (char) 73, (char) 70}, 0);
		p311.hw_version_minor_SET((char) 113);
		p311.sw_version_minor_SET((char) 132);
		p311.name_SET("giadqgcsduqkxcdNjqzyFxtrsiacmjIHxaesdfzuqxgofjcitmbuuuhweiwswxiwnkbt", PH);
		p311.sw_vcs_commit_SET(2526888681L);
		p311.hw_version_major_SET((char) 114);
		p311.uptime_sec_SET(3774169061L);
		p311.time_usec_SET(1146673911249588004L);
		p311.sw_version_major_SET((char) 85);
		CommunicationChannel.instance.send(p311);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
		TestChannel.instance.on_PARAM_EXT_REQUEST_READ.add((src, ph, pack) ->
		{
			assert (pack.param_id_LEN(ph) == 8);
			assert (pack.param_id_TRY(ph).equals("jQfmeqsa"));
			assert (pack.param_index_GET() == (short) -505);
			assert (pack.target_component_GET() == (char) 202);
			assert (pack.target_system_GET() == (char) 102);
		});
		GroundControl.PARAM_EXT_REQUEST_READ p320 = CommunicationChannel.new_PARAM_EXT_REQUEST_READ();
		PH.setPack(p320);
		p320.target_component_SET((char) 202);
		p320.param_index_SET((short) -505);
		p320.param_id_SET("jQfmeqsa", PH);
		p320.target_system_SET((char) 102);
		CommunicationChannel.instance.send(p320);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
		TestChannel.instance.on_PARAM_EXT_REQUEST_LIST.add((src, ph, pack) ->
		{
			assert (pack.target_system_GET() == (char) 108);
			assert (pack.target_component_GET() == (char) 122);
		});
		GroundControl.PARAM_EXT_REQUEST_LIST p321 = CommunicationChannel.new_PARAM_EXT_REQUEST_LIST();
		PH.setPack(p321);
		p321.target_component_SET((char) 122);
		p321.target_system_SET((char) 108);
		CommunicationChannel.instance.send(p321);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
		TestChannel.instance.on_PARAM_EXT_VALUE.add((src, ph, pack) ->
		{
			assert (pack.param_value_LEN(ph) == 13);
			assert (pack.param_value_TRY(ph).equals("vctkqlNdqsrbo"));
			assert (pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT16);
			assert (pack.param_index_GET() == (char) 50915);
			assert (pack.param_id_LEN(ph) == 1);
			assert (pack.param_id_TRY(ph).equals("x"));
			assert (pack.param_count_GET() == (char) 42194);
		});
		GroundControl.PARAM_EXT_VALUE p322 = CommunicationChannel.new_PARAM_EXT_VALUE();
		PH.setPack(p322);
		p322.param_value_SET("vctkqlNdqsrbo", PH);
		p322.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT16);
		p322.param_count_SET((char) 42194);
		p322.param_id_SET("x", PH);
		p322.param_index_SET((char) 50915);
		CommunicationChannel.instance.send(p322);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
		TestChannel.instance.on_PARAM_EXT_SET.add((src, ph, pack) ->
		{
			assert (pack.param_id_LEN(ph) == 6);
			assert (pack.param_id_TRY(ph).equals("ywgkkb"));
			assert (pack.param_value_LEN(ph) == 105);
			assert (pack.param_value_TRY(ph).equals("nquvbdtxnzeovgdTyxmqtioqdccxjAknpDgokAcdkhpmbuffcWfldrvpvbfgnMzexbbkzmfpfcPfBhywnmozbqxYmvrsfynZryxQfevzr"));
			assert (pack.target_system_GET() == (char) 184);
			assert (pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_REAL64);
			assert (pack.target_component_GET() == (char) 144);
		});
		GroundControl.PARAM_EXT_SET p323 = CommunicationChannel.new_PARAM_EXT_SET();
		PH.setPack(p323);
		p323.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_REAL64);
		p323.target_component_SET((char) 144);
		p323.target_system_SET((char) 184);
		p323.param_id_SET("ywgkkb", PH);
		p323.param_value_SET("nquvbdtxnzeovgdTyxmqtioqdccxjAknpDgokAcdkhpmbuffcWfldrvpvbfgnMzexbbkzmfpfcPfBhywnmozbqxYmvrsfynZryxQfevzr", PH);
		CommunicationChannel.instance.send(p323);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
		TestChannel.instance.on_PARAM_EXT_ACK.add((src, ph, pack) ->
		{
			assert (pack.param_value_LEN(ph) == 115);
			assert (pack.param_value_TRY(ph).equals("uwynkheDNbjddcTdxxyMtjOdbkcdpvepdLjujtciejHrqbpjxvzpbcxhwimwwbdmxnlotlqurVjvzcazryvAxjpkXwngueyiwffuswHfbfrZixqehxp"));
			assert (pack.param_result_GET() == PARAM_ACK.PARAM_ACK_ACCEPTED);
			assert (pack.param_id_LEN(ph) == 7);
			assert (pack.param_id_TRY(ph).equals("jpaqejg"));
			assert (pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_REAL32);
		});
		GroundControl.PARAM_EXT_ACK p324 = CommunicationChannel.new_PARAM_EXT_ACK();
		PH.setPack(p324);
		p324.param_id_SET("jpaqejg", PH);
		p324.param_value_SET("uwynkheDNbjddcTdxxyMtjOdbkcdpvepdLjujtciejHrqbpjxvzpbcxhwimwwbdmxnlotlqurVjvzcazryvAxjpkXwngueyiwffuswHfbfrZixqehxp", PH);
		p324.param_result_SET(PARAM_ACK.PARAM_ACK_ACCEPTED);
		p324.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_REAL32);
		CommunicationChannel.instance.send(p324);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
		TestChannel.instance.on_OBSTACLE_DISTANCE.add((src, ph, pack) ->
		{
			assert (pack.min_distance_GET() == (char) 57088);
			assert (pack.sensor_type_GET() == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_UNKNOWN);
			assert (pack.time_usec_GET() == 1225770957243067071L);
			assert (pack.max_distance_GET() == (char) 41049);
			assert (pack.increment_GET() == (char) 57);
			assert (Arrays.equals(pack.distances_GET(), new char[]{(char) 14463, (char) 35708, (char) 22066, (char) 11653, (char) 5544, (char) 8537, (char) 54785, (char) 8347, (char) 34734, (char) 61621, (char) 31401, (char) 58117, (char) 30339, (char) 31655, (char) 47363, (char) 67, (char) 47538, (char) 41680, (char) 36206, (char) 22412, (char) 8025, (char) 25418, (char) 38565, (char) 23637, (char) 49800, (char) 17479, (char) 43225, (char) 7272, (char) 4485, (char) 36158, (char) 13103, (char) 47514, (char) 22129, (char) 60119, (char) 676, (char) 32565, (char) 52295, (char) 55387, (char) 11969, (char) 52931, (char) 48238, (char) 53270, (char) 1101, (char) 28301, (char) 45797, (char) 18425, (char) 28183, (char) 52559, (char) 13594, (char) 62893, (char) 27903, (char) 18984, (char) 49198, (char) 57231, (char) 40961, (char) 54024, (char) 19638, (char) 50252, (char) 23415, (char) 1957, (char) 36110, (char) 32864, (char) 16807, (char) 25163, (char) 34754, (char) 61933, (char) 25491, (char) 44146, (char) 6199, (char) 28537, (char) 20740, (char) 60015}));
		});
		GroundControl.OBSTACLE_DISTANCE p330 = CommunicationChannel.new_OBSTACLE_DISTANCE();
		PH.setPack(p330);
		p330.distances_SET(new char[]{(char) 14463, (char) 35708, (char) 22066, (char) 11653, (char) 5544, (char) 8537, (char) 54785, (char) 8347, (char) 34734, (char) 61621, (char) 31401, (char) 58117, (char) 30339, (char) 31655, (char) 47363, (char) 67, (char) 47538, (char) 41680, (char) 36206, (char) 22412, (char) 8025, (char) 25418, (char) 38565, (char) 23637, (char) 49800, (char) 17479, (char) 43225, (char) 7272, (char) 4485, (char) 36158, (char) 13103, (char) 47514, (char) 22129, (char) 60119, (char) 676, (char) 32565, (char) 52295, (char) 55387, (char) 11969, (char) 52931, (char) 48238, (char) 53270, (char) 1101, (char) 28301, (char) 45797, (char) 18425, (char) 28183, (char) 52559, (char) 13594, (char) 62893, (char) 27903, (char) 18984, (char) 49198, (char) 57231, (char) 40961, (char) 54024, (char) 19638, (char) 50252, (char) 23415, (char) 1957, (char) 36110, (char) 32864, (char) 16807, (char) 25163, (char) 34754, (char) 61933, (char) 25491, (char) 44146, (char) 6199, (char) 28537, (char) 20740, (char) 60015}, 0);
		p330.sensor_type_SET(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_UNKNOWN);
		p330.min_distance_SET((char) 57088);
		p330.max_distance_SET((char) 41049);
		p330.time_usec_SET(1225770957243067071L);
		p330.increment_SET((char) 57);
		CommunicationChannel.instance.send(p330);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
	}

}