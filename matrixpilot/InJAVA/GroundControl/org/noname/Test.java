
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

	public static class POSITION_TARGET_LOCAL_NED extends GroundControl.POSITION_TARGET_LOCAL_NED {
		/**
		 * Bitmask to indicate which dimensions should be ignored by the vehicle: a value of 0b0000000000000000 or
		 * 0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. If bit 10 is set
		 * the floats afx afy afz should be interpreted as force instead of acceleration. Mapping: bit 1: x, bit
		 * 2: y, bit 3: z, bit 4: vx, bit 5: vy, bit 6: vz, bit 7: ax, bit 8: ay, bit 9: az, bit 10: is force setpoint,
		 * bit 11: yaw, bit 12: yaw rat
		 */
		public void type_mask_SET(char src) { set_bytes((char) (src) & -1L, 2, data, 0); }

		public void time_boot_ms_SET(long src) //Timestamp in milliseconds since system boot
		{ set_bytes((src) & -1L, 4, data, 2); }

		public void x_SET(float src) //X Position in NED frame in meters
		{ set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 6); }

		public void y_SET(float src) //Y Position in NED frame in meters
		{ set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 10); }

		public void z_SET(float src) //Z Position in NED frame in meters (note, altitude is negative in NED)
		{ set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 14); }

		public void vx_SET(float src) //X velocity in NED frame in meter / s
		{ set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 18); }

		public void vy_SET(float src) //Y velocity in NED frame in meter / s
		{ set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 22); }

		public void vz_SET(float src) //Z velocity in NED frame in meter / s
		{ set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 26); }

		public void afx_SET(float src) //X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
		{ set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 30); }

		public void afy_SET(float src) //Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
		{ set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 34); }

		public void afz_SET(float src) //Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
		{ set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 38); }

		public void yaw_SET(float src) //yaw setpoint in rad
		{ set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 42); }

		public void yaw_rate_SET(float src) //yaw rate setpoint in rad/s
		{ set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 46); }

		/**
		 * Valid options are: MAV_FRAME_LOCAL_NED = 1, MAV_FRAME_LOCAL_OFFSET_NED = 7, MAV_FRAME_BODY_NED = 8, MAV_FRAME_BODY_OFFSET_NED
		 * =
		 */
		public void coordinate_frame_SET(@MAV_FRAME int src) { set_bits(-0 + src, 4, data, 400); }
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
				case MAV_CMD.MAV_CMD_PREFLIGHT_STORAGE_ADVANCED:
					id = 0;
					break;
				case MAV_CMD.MAV_CMD_NAV_WAYPOINT:
					id = 1;
					break;
				case MAV_CMD.MAV_CMD_NAV_LOITER_UNLIM:
					id = 2;
					break;
				case MAV_CMD.MAV_CMD_NAV_LOITER_TURNS:
					id = 3;
					break;
				case MAV_CMD.MAV_CMD_NAV_LOITER_TIME:
					id = 4;
					break;
				case MAV_CMD.MAV_CMD_NAV_RETURN_TO_LAUNCH:
					id = 5;
					break;
				case MAV_CMD.MAV_CMD_NAV_LAND:
					id = 6;
					break;
				case MAV_CMD.MAV_CMD_NAV_TAKEOFF:
					id = 7;
					break;
				case MAV_CMD.MAV_CMD_NAV_LAND_LOCAL:
					id = 8;
					break;
				case MAV_CMD.MAV_CMD_NAV_TAKEOFF_LOCAL:
					id = 9;
					break;
				case MAV_CMD.MAV_CMD_NAV_FOLLOW:
					id = 10;
					break;
				case MAV_CMD.MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT:
					id = 11;
					break;
				case MAV_CMD.MAV_CMD_NAV_LOITER_TO_ALT:
					id = 12;
					break;
				case MAV_CMD.MAV_CMD_DO_FOLLOW:
					id = 13;
					break;
				case MAV_CMD.MAV_CMD_DO_FOLLOW_REPOSITION:
					id = 14;
					break;
				case MAV_CMD.MAV_CMD_NAV_ROI:
					id = 15;
					break;
				case MAV_CMD.MAV_CMD_NAV_PATHPLANNING:
					id = 16;
					break;
				case MAV_CMD.MAV_CMD_NAV_SPLINE_WAYPOINT:
					id = 17;
					break;
				case MAV_CMD.MAV_CMD_NAV_VTOL_TAKEOFF:
					id = 18;
					break;
				case MAV_CMD.MAV_CMD_NAV_VTOL_LAND:
					id = 19;
					break;
				case MAV_CMD.MAV_CMD_NAV_GUIDED_ENABLE:
					id = 20;
					break;
				case MAV_CMD.MAV_CMD_NAV_DELAY:
					id = 21;
					break;
				case MAV_CMD.MAV_CMD_NAV_PAYLOAD_PLACE:
					id = 22;
					break;
				case MAV_CMD.MAV_CMD_NAV_LAST:
					id = 23;
					break;
				case MAV_CMD.MAV_CMD_CONDITION_DELAY:
					id = 24;
					break;
				case MAV_CMD.MAV_CMD_CONDITION_CHANGE_ALT:
					id = 25;
					break;
				case MAV_CMD.MAV_CMD_CONDITION_DISTANCE:
					id = 26;
					break;
				case MAV_CMD.MAV_CMD_CONDITION_YAW:
					id = 27;
					break;
				case MAV_CMD.MAV_CMD_CONDITION_LAST:
					id = 28;
					break;
				case MAV_CMD.MAV_CMD_DO_SET_MODE:
					id = 29;
					break;
				case MAV_CMD.MAV_CMD_DO_JUMP:
					id = 30;
					break;
				case MAV_CMD.MAV_CMD_DO_CHANGE_SPEED:
					id = 31;
					break;
				case MAV_CMD.MAV_CMD_DO_SET_HOME:
					id = 32;
					break;
				case MAV_CMD.MAV_CMD_DO_SET_PARAMETER:
					id = 33;
					break;
				case MAV_CMD.MAV_CMD_DO_SET_RELAY:
					id = 34;
					break;
				case MAV_CMD.MAV_CMD_DO_REPEAT_RELAY:
					id = 35;
					break;
				case MAV_CMD.MAV_CMD_DO_SET_SERVO:
					id = 36;
					break;
				case MAV_CMD.MAV_CMD_DO_REPEAT_SERVO:
					id = 37;
					break;
				case MAV_CMD.MAV_CMD_DO_FLIGHTTERMINATION:
					id = 38;
					break;
				case MAV_CMD.MAV_CMD_DO_CHANGE_ALTITUDE:
					id = 39;
					break;
				case MAV_CMD.MAV_CMD_DO_LAND_START:
					id = 40;
					break;
				case MAV_CMD.MAV_CMD_DO_RALLY_LAND:
					id = 41;
					break;
				case MAV_CMD.MAV_CMD_DO_GO_AROUND:
					id = 42;
					break;
				case MAV_CMD.MAV_CMD_DO_REPOSITION:
					id = 43;
					break;
				case MAV_CMD.MAV_CMD_DO_PAUSE_CONTINUE:
					id = 44;
					break;
				case MAV_CMD.MAV_CMD_DO_SET_REVERSE:
					id = 45;
					break;
				case MAV_CMD.MAV_CMD_DO_CONTROL_VIDEO:
					id = 46;
					break;
				case MAV_CMD.MAV_CMD_DO_SET_ROI:
					id = 47;
					break;
				case MAV_CMD.MAV_CMD_DO_DIGICAM_CONFIGURE:
					id = 48;
					break;
				case MAV_CMD.MAV_CMD_DO_DIGICAM_CONTROL:
					id = 49;
					break;
				case MAV_CMD.MAV_CMD_DO_MOUNT_CONFIGURE:
					id = 50;
					break;
				case MAV_CMD.MAV_CMD_DO_MOUNT_CONTROL:
					id = 51;
					break;
				case MAV_CMD.MAV_CMD_DO_SET_CAM_TRIGG_DIST:
					id = 52;
					break;
				case MAV_CMD.MAV_CMD_DO_FENCE_ENABLE:
					id = 53;
					break;
				case MAV_CMD.MAV_CMD_DO_PARACHUTE:
					id = 54;
					break;
				case MAV_CMD.MAV_CMD_DO_MOTOR_TEST:
					id = 55;
					break;
				case MAV_CMD.MAV_CMD_DO_INVERTED_FLIGHT:
					id = 56;
					break;
				case MAV_CMD.MAV_CMD_NAV_SET_YAW_SPEED:
					id = 57;
					break;
				case MAV_CMD.MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL:
					id = 58;
					break;
				case MAV_CMD.MAV_CMD_DO_MOUNT_CONTROL_QUAT:
					id = 59;
					break;
				case MAV_CMD.MAV_CMD_DO_GUIDED_MASTER:
					id = 60;
					break;
				case MAV_CMD.MAV_CMD_DO_GUIDED_LIMITS:
					id = 61;
					break;
				case MAV_CMD.MAV_CMD_DO_ENGINE_CONTROL:
					id = 62;
					break;
				case MAV_CMD.MAV_CMD_DO_LAST:
					id = 63;
					break;
				case MAV_CMD.MAV_CMD_PREFLIGHT_CALIBRATION:
					id = 64;
					break;
				case MAV_CMD.MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS:
					id = 65;
					break;
				case MAV_CMD.MAV_CMD_PREFLIGHT_UAVCAN:
					id = 66;
					break;
				case MAV_CMD.MAV_CMD_PREFLIGHT_STORAGE:
					id = 67;
					break;
				case MAV_CMD.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
					id = 68;
					break;
				case MAV_CMD.MAV_CMD_OVERRIDE_GOTO:
					id = 69;
					break;
				case MAV_CMD.MAV_CMD_MISSION_START:
					id = 70;
					break;
				case MAV_CMD.MAV_CMD_COMPONENT_ARM_DISARM:
					id = 71;
					break;
				case MAV_CMD.MAV_CMD_GET_HOME_POSITION:
					id = 72;
					break;
				case MAV_CMD.MAV_CMD_START_RX_PAIR:
					id = 73;
					break;
				case MAV_CMD.MAV_CMD_GET_MESSAGE_INTERVAL:
					id = 74;
					break;
				case MAV_CMD.MAV_CMD_SET_MESSAGE_INTERVAL:
					id = 75;
					break;
				case MAV_CMD.MAV_CMD_REQUEST_PROTOCOL_VERSION:
					id = 76;
					break;
				case MAV_CMD.MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES:
					id = 77;
					break;
				case MAV_CMD.MAV_CMD_REQUEST_CAMERA_INFORMATION:
					id = 78;
					break;
				case MAV_CMD.MAV_CMD_REQUEST_CAMERA_SETTINGS:
					id = 79;
					break;
				case MAV_CMD.MAV_CMD_REQUEST_STORAGE_INFORMATION:
					id = 80;
					break;
				case MAV_CMD.MAV_CMD_STORAGE_FORMAT:
					id = 81;
					break;
				case MAV_CMD.MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS:
					id = 82;
					break;
				case MAV_CMD.MAV_CMD_REQUEST_FLIGHT_INFORMATION:
					id = 83;
					break;
				case MAV_CMD.MAV_CMD_RESET_CAMERA_SETTINGS:
					id = 84;
					break;
				case MAV_CMD.MAV_CMD_SET_CAMERA_MODE:
					id = 85;
					break;
				case MAV_CMD.MAV_CMD_IMAGE_START_CAPTURE:
					id = 86;
					break;
				case MAV_CMD.MAV_CMD_IMAGE_STOP_CAPTURE:
					id = 87;
					break;
				case MAV_CMD.MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE:
					id = 88;
					break;
				case MAV_CMD.MAV_CMD_DO_TRIGGER_CONTROL:
					id = 89;
					break;
				case MAV_CMD.MAV_CMD_VIDEO_START_CAPTURE:
					id = 90;
					break;
				case MAV_CMD.MAV_CMD_VIDEO_STOP_CAPTURE:
					id = 91;
					break;
				case MAV_CMD.MAV_CMD_VIDEO_START_STREAMING:
					id = 92;
					break;
				case MAV_CMD.MAV_CMD_VIDEO_STOP_STREAMING:
					id = 93;
					break;
				case MAV_CMD.MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION:
					id = 94;
					break;
				case MAV_CMD.MAV_CMD_LOGGING_START:
					id = 95;
					break;
				case MAV_CMD.MAV_CMD_LOGGING_STOP:
					id = 96;
					break;
				case MAV_CMD.MAV_CMD_AIRFRAME_CONFIGURATION:
					id = 97;
					break;
				case MAV_CMD.MAV_CMD_PANORAMA_CREATE:
					id = 98;
					break;
				case MAV_CMD.MAV_CMD_DO_VTOL_TRANSITION:
					id = 99;
					break;
				case MAV_CMD.MAV_CMD_ARM_AUTHORIZATION_REQUEST:
					id = 100;
					break;
				case MAV_CMD.MAV_CMD_SET_GUIDED_SUBMODE_STANDARD:
					id = 101;
					break;
				case MAV_CMD.MAV_CMD_SET_GUIDED_SUBMODE_CIRCLE:
					id = 102;
					break;
				case MAV_CMD.MAV_CMD_CONDITION_GATE:
					id = 103;
					break;
				case MAV_CMD.MAV_CMD_NAV_FENCE_RETURN_POINT:
					id = 104;
					break;
				case MAV_CMD.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION:
					id = 105;
					break;
				case MAV_CMD.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION:
					id = 106;
					break;
				case MAV_CMD.MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION:
					id = 107;
					break;
				case MAV_CMD.MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION:
					id = 108;
					break;
				case MAV_CMD.MAV_CMD_NAV_RALLY_POINT:
					id = 109;
					break;
				case MAV_CMD.MAV_CMD_UAVCAN_GET_NODE_INFO:
					id = 110;
					break;
				case MAV_CMD.MAV_CMD_PAYLOAD_PREPARE_DEPLOY:
					id = 111;
					break;
				case MAV_CMD.MAV_CMD_PAYLOAD_CONTROL_DEPLOY:
					id = 112;
					break;
				case MAV_CMD.MAV_CMD_WAYPOINT_USER_1:
					id = 113;
					break;
				case MAV_CMD.MAV_CMD_WAYPOINT_USER_2:
					id = 114;
					break;
				case MAV_CMD.MAV_CMD_WAYPOINT_USER_3:
					id = 115;
					break;
				case MAV_CMD.MAV_CMD_WAYPOINT_USER_4:
					id = 116;
					break;
				case MAV_CMD.MAV_CMD_WAYPOINT_USER_5:
					id = 117;
					break;
				case MAV_CMD.MAV_CMD_SPATIAL_USER_1:
					id = 118;
					break;
				case MAV_CMD.MAV_CMD_SPATIAL_USER_2:
					id = 119;
					break;
				case MAV_CMD.MAV_CMD_SPATIAL_USER_3:
					id = 120;
					break;
				case MAV_CMD.MAV_CMD_SPATIAL_USER_4:
					id = 121;
					break;
				case MAV_CMD.MAV_CMD_SPATIAL_USER_5:
					id = 122;
					break;
				case MAV_CMD.MAV_CMD_USER_1:
					id = 123;
					break;
				case MAV_CMD.MAV_CMD_USER_2:
					id = 124;
					break;
				case MAV_CMD.MAV_CMD_USER_3:
					id = 125;
					break;
				case MAV_CMD.MAV_CMD_USER_4:
					id = 126;
					break;
				case MAV_CMD.MAV_CMD_USER_5:
					id = 127;
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
				case MAV_CMD.MAV_CMD_PREFLIGHT_STORAGE_ADVANCED:
					id = 0;
					break;
				case MAV_CMD.MAV_CMD_NAV_WAYPOINT:
					id = 1;
					break;
				case MAV_CMD.MAV_CMD_NAV_LOITER_UNLIM:
					id = 2;
					break;
				case MAV_CMD.MAV_CMD_NAV_LOITER_TURNS:
					id = 3;
					break;
				case MAV_CMD.MAV_CMD_NAV_LOITER_TIME:
					id = 4;
					break;
				case MAV_CMD.MAV_CMD_NAV_RETURN_TO_LAUNCH:
					id = 5;
					break;
				case MAV_CMD.MAV_CMD_NAV_LAND:
					id = 6;
					break;
				case MAV_CMD.MAV_CMD_NAV_TAKEOFF:
					id = 7;
					break;
				case MAV_CMD.MAV_CMD_NAV_LAND_LOCAL:
					id = 8;
					break;
				case MAV_CMD.MAV_CMD_NAV_TAKEOFF_LOCAL:
					id = 9;
					break;
				case MAV_CMD.MAV_CMD_NAV_FOLLOW:
					id = 10;
					break;
				case MAV_CMD.MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT:
					id = 11;
					break;
				case MAV_CMD.MAV_CMD_NAV_LOITER_TO_ALT:
					id = 12;
					break;
				case MAV_CMD.MAV_CMD_DO_FOLLOW:
					id = 13;
					break;
				case MAV_CMD.MAV_CMD_DO_FOLLOW_REPOSITION:
					id = 14;
					break;
				case MAV_CMD.MAV_CMD_NAV_ROI:
					id = 15;
					break;
				case MAV_CMD.MAV_CMD_NAV_PATHPLANNING:
					id = 16;
					break;
				case MAV_CMD.MAV_CMD_NAV_SPLINE_WAYPOINT:
					id = 17;
					break;
				case MAV_CMD.MAV_CMD_NAV_VTOL_TAKEOFF:
					id = 18;
					break;
				case MAV_CMD.MAV_CMD_NAV_VTOL_LAND:
					id = 19;
					break;
				case MAV_CMD.MAV_CMD_NAV_GUIDED_ENABLE:
					id = 20;
					break;
				case MAV_CMD.MAV_CMD_NAV_DELAY:
					id = 21;
					break;
				case MAV_CMD.MAV_CMD_NAV_PAYLOAD_PLACE:
					id = 22;
					break;
				case MAV_CMD.MAV_CMD_NAV_LAST:
					id = 23;
					break;
				case MAV_CMD.MAV_CMD_CONDITION_DELAY:
					id = 24;
					break;
				case MAV_CMD.MAV_CMD_CONDITION_CHANGE_ALT:
					id = 25;
					break;
				case MAV_CMD.MAV_CMD_CONDITION_DISTANCE:
					id = 26;
					break;
				case MAV_CMD.MAV_CMD_CONDITION_YAW:
					id = 27;
					break;
				case MAV_CMD.MAV_CMD_CONDITION_LAST:
					id = 28;
					break;
				case MAV_CMD.MAV_CMD_DO_SET_MODE:
					id = 29;
					break;
				case MAV_CMD.MAV_CMD_DO_JUMP:
					id = 30;
					break;
				case MAV_CMD.MAV_CMD_DO_CHANGE_SPEED:
					id = 31;
					break;
				case MAV_CMD.MAV_CMD_DO_SET_HOME:
					id = 32;
					break;
				case MAV_CMD.MAV_CMD_DO_SET_PARAMETER:
					id = 33;
					break;
				case MAV_CMD.MAV_CMD_DO_SET_RELAY:
					id = 34;
					break;
				case MAV_CMD.MAV_CMD_DO_REPEAT_RELAY:
					id = 35;
					break;
				case MAV_CMD.MAV_CMD_DO_SET_SERVO:
					id = 36;
					break;
				case MAV_CMD.MAV_CMD_DO_REPEAT_SERVO:
					id = 37;
					break;
				case MAV_CMD.MAV_CMD_DO_FLIGHTTERMINATION:
					id = 38;
					break;
				case MAV_CMD.MAV_CMD_DO_CHANGE_ALTITUDE:
					id = 39;
					break;
				case MAV_CMD.MAV_CMD_DO_LAND_START:
					id = 40;
					break;
				case MAV_CMD.MAV_CMD_DO_RALLY_LAND:
					id = 41;
					break;
				case MAV_CMD.MAV_CMD_DO_GO_AROUND:
					id = 42;
					break;
				case MAV_CMD.MAV_CMD_DO_REPOSITION:
					id = 43;
					break;
				case MAV_CMD.MAV_CMD_DO_PAUSE_CONTINUE:
					id = 44;
					break;
				case MAV_CMD.MAV_CMD_DO_SET_REVERSE:
					id = 45;
					break;
				case MAV_CMD.MAV_CMD_DO_CONTROL_VIDEO:
					id = 46;
					break;
				case MAV_CMD.MAV_CMD_DO_SET_ROI:
					id = 47;
					break;
				case MAV_CMD.MAV_CMD_DO_DIGICAM_CONFIGURE:
					id = 48;
					break;
				case MAV_CMD.MAV_CMD_DO_DIGICAM_CONTROL:
					id = 49;
					break;
				case MAV_CMD.MAV_CMD_DO_MOUNT_CONFIGURE:
					id = 50;
					break;
				case MAV_CMD.MAV_CMD_DO_MOUNT_CONTROL:
					id = 51;
					break;
				case MAV_CMD.MAV_CMD_DO_SET_CAM_TRIGG_DIST:
					id = 52;
					break;
				case MAV_CMD.MAV_CMD_DO_FENCE_ENABLE:
					id = 53;
					break;
				case MAV_CMD.MAV_CMD_DO_PARACHUTE:
					id = 54;
					break;
				case MAV_CMD.MAV_CMD_DO_MOTOR_TEST:
					id = 55;
					break;
				case MAV_CMD.MAV_CMD_DO_INVERTED_FLIGHT:
					id = 56;
					break;
				case MAV_CMD.MAV_CMD_NAV_SET_YAW_SPEED:
					id = 57;
					break;
				case MAV_CMD.MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL:
					id = 58;
					break;
				case MAV_CMD.MAV_CMD_DO_MOUNT_CONTROL_QUAT:
					id = 59;
					break;
				case MAV_CMD.MAV_CMD_DO_GUIDED_MASTER:
					id = 60;
					break;
				case MAV_CMD.MAV_CMD_DO_GUIDED_LIMITS:
					id = 61;
					break;
				case MAV_CMD.MAV_CMD_DO_ENGINE_CONTROL:
					id = 62;
					break;
				case MAV_CMD.MAV_CMD_DO_LAST:
					id = 63;
					break;
				case MAV_CMD.MAV_CMD_PREFLIGHT_CALIBRATION:
					id = 64;
					break;
				case MAV_CMD.MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS:
					id = 65;
					break;
				case MAV_CMD.MAV_CMD_PREFLIGHT_UAVCAN:
					id = 66;
					break;
				case MAV_CMD.MAV_CMD_PREFLIGHT_STORAGE:
					id = 67;
					break;
				case MAV_CMD.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
					id = 68;
					break;
				case MAV_CMD.MAV_CMD_OVERRIDE_GOTO:
					id = 69;
					break;
				case MAV_CMD.MAV_CMD_MISSION_START:
					id = 70;
					break;
				case MAV_CMD.MAV_CMD_COMPONENT_ARM_DISARM:
					id = 71;
					break;
				case MAV_CMD.MAV_CMD_GET_HOME_POSITION:
					id = 72;
					break;
				case MAV_CMD.MAV_CMD_START_RX_PAIR:
					id = 73;
					break;
				case MAV_CMD.MAV_CMD_GET_MESSAGE_INTERVAL:
					id = 74;
					break;
				case MAV_CMD.MAV_CMD_SET_MESSAGE_INTERVAL:
					id = 75;
					break;
				case MAV_CMD.MAV_CMD_REQUEST_PROTOCOL_VERSION:
					id = 76;
					break;
				case MAV_CMD.MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES:
					id = 77;
					break;
				case MAV_CMD.MAV_CMD_REQUEST_CAMERA_INFORMATION:
					id = 78;
					break;
				case MAV_CMD.MAV_CMD_REQUEST_CAMERA_SETTINGS:
					id = 79;
					break;
				case MAV_CMD.MAV_CMD_REQUEST_STORAGE_INFORMATION:
					id = 80;
					break;
				case MAV_CMD.MAV_CMD_STORAGE_FORMAT:
					id = 81;
					break;
				case MAV_CMD.MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS:
					id = 82;
					break;
				case MAV_CMD.MAV_CMD_REQUEST_FLIGHT_INFORMATION:
					id = 83;
					break;
				case MAV_CMD.MAV_CMD_RESET_CAMERA_SETTINGS:
					id = 84;
					break;
				case MAV_CMD.MAV_CMD_SET_CAMERA_MODE:
					id = 85;
					break;
				case MAV_CMD.MAV_CMD_IMAGE_START_CAPTURE:
					id = 86;
					break;
				case MAV_CMD.MAV_CMD_IMAGE_STOP_CAPTURE:
					id = 87;
					break;
				case MAV_CMD.MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE:
					id = 88;
					break;
				case MAV_CMD.MAV_CMD_DO_TRIGGER_CONTROL:
					id = 89;
					break;
				case MAV_CMD.MAV_CMD_VIDEO_START_CAPTURE:
					id = 90;
					break;
				case MAV_CMD.MAV_CMD_VIDEO_STOP_CAPTURE:
					id = 91;
					break;
				case MAV_CMD.MAV_CMD_VIDEO_START_STREAMING:
					id = 92;
					break;
				case MAV_CMD.MAV_CMD_VIDEO_STOP_STREAMING:
					id = 93;
					break;
				case MAV_CMD.MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION:
					id = 94;
					break;
				case MAV_CMD.MAV_CMD_LOGGING_START:
					id = 95;
					break;
				case MAV_CMD.MAV_CMD_LOGGING_STOP:
					id = 96;
					break;
				case MAV_CMD.MAV_CMD_AIRFRAME_CONFIGURATION:
					id = 97;
					break;
				case MAV_CMD.MAV_CMD_PANORAMA_CREATE:
					id = 98;
					break;
				case MAV_CMD.MAV_CMD_DO_VTOL_TRANSITION:
					id = 99;
					break;
				case MAV_CMD.MAV_CMD_ARM_AUTHORIZATION_REQUEST:
					id = 100;
					break;
				case MAV_CMD.MAV_CMD_SET_GUIDED_SUBMODE_STANDARD:
					id = 101;
					break;
				case MAV_CMD.MAV_CMD_SET_GUIDED_SUBMODE_CIRCLE:
					id = 102;
					break;
				case MAV_CMD.MAV_CMD_CONDITION_GATE:
					id = 103;
					break;
				case MAV_CMD.MAV_CMD_NAV_FENCE_RETURN_POINT:
					id = 104;
					break;
				case MAV_CMD.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION:
					id = 105;
					break;
				case MAV_CMD.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION:
					id = 106;
					break;
				case MAV_CMD.MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION:
					id = 107;
					break;
				case MAV_CMD.MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION:
					id = 108;
					break;
				case MAV_CMD.MAV_CMD_NAV_RALLY_POINT:
					id = 109;
					break;
				case MAV_CMD.MAV_CMD_UAVCAN_GET_NODE_INFO:
					id = 110;
					break;
				case MAV_CMD.MAV_CMD_PAYLOAD_PREPARE_DEPLOY:
					id = 111;
					break;
				case MAV_CMD.MAV_CMD_PAYLOAD_CONTROL_DEPLOY:
					id = 112;
					break;
				case MAV_CMD.MAV_CMD_WAYPOINT_USER_1:
					id = 113;
					break;
				case MAV_CMD.MAV_CMD_WAYPOINT_USER_2:
					id = 114;
					break;
				case MAV_CMD.MAV_CMD_WAYPOINT_USER_3:
					id = 115;
					break;
				case MAV_CMD.MAV_CMD_WAYPOINT_USER_4:
					id = 116;
					break;
				case MAV_CMD.MAV_CMD_WAYPOINT_USER_5:
					id = 117;
					break;
				case MAV_CMD.MAV_CMD_SPATIAL_USER_1:
					id = 118;
					break;
				case MAV_CMD.MAV_CMD_SPATIAL_USER_2:
					id = 119;
					break;
				case MAV_CMD.MAV_CMD_SPATIAL_USER_3:
					id = 120;
					break;
				case MAV_CMD.MAV_CMD_SPATIAL_USER_4:
					id = 121;
					break;
				case MAV_CMD.MAV_CMD_SPATIAL_USER_5:
					id = 122;
					break;
				case MAV_CMD.MAV_CMD_USER_1:
					id = 123;
					break;
				case MAV_CMD.MAV_CMD_USER_2:
					id = 124;
					break;
				case MAV_CMD.MAV_CMD_USER_3:
					id = 125;
					break;
				case MAV_CMD.MAV_CMD_USER_4:
					id = 126;
					break;
				case MAV_CMD.MAV_CMD_USER_5:
					id = 127;
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
				case MAV_CMD.MAV_CMD_PREFLIGHT_STORAGE_ADVANCED:
					id = 0;
					break;
				case MAV_CMD.MAV_CMD_NAV_WAYPOINT:
					id = 1;
					break;
				case MAV_CMD.MAV_CMD_NAV_LOITER_UNLIM:
					id = 2;
					break;
				case MAV_CMD.MAV_CMD_NAV_LOITER_TURNS:
					id = 3;
					break;
				case MAV_CMD.MAV_CMD_NAV_LOITER_TIME:
					id = 4;
					break;
				case MAV_CMD.MAV_CMD_NAV_RETURN_TO_LAUNCH:
					id = 5;
					break;
				case MAV_CMD.MAV_CMD_NAV_LAND:
					id = 6;
					break;
				case MAV_CMD.MAV_CMD_NAV_TAKEOFF:
					id = 7;
					break;
				case MAV_CMD.MAV_CMD_NAV_LAND_LOCAL:
					id = 8;
					break;
				case MAV_CMD.MAV_CMD_NAV_TAKEOFF_LOCAL:
					id = 9;
					break;
				case MAV_CMD.MAV_CMD_NAV_FOLLOW:
					id = 10;
					break;
				case MAV_CMD.MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT:
					id = 11;
					break;
				case MAV_CMD.MAV_CMD_NAV_LOITER_TO_ALT:
					id = 12;
					break;
				case MAV_CMD.MAV_CMD_DO_FOLLOW:
					id = 13;
					break;
				case MAV_CMD.MAV_CMD_DO_FOLLOW_REPOSITION:
					id = 14;
					break;
				case MAV_CMD.MAV_CMD_NAV_ROI:
					id = 15;
					break;
				case MAV_CMD.MAV_CMD_NAV_PATHPLANNING:
					id = 16;
					break;
				case MAV_CMD.MAV_CMD_NAV_SPLINE_WAYPOINT:
					id = 17;
					break;
				case MAV_CMD.MAV_CMD_NAV_VTOL_TAKEOFF:
					id = 18;
					break;
				case MAV_CMD.MAV_CMD_NAV_VTOL_LAND:
					id = 19;
					break;
				case MAV_CMD.MAV_CMD_NAV_GUIDED_ENABLE:
					id = 20;
					break;
				case MAV_CMD.MAV_CMD_NAV_DELAY:
					id = 21;
					break;
				case MAV_CMD.MAV_CMD_NAV_PAYLOAD_PLACE:
					id = 22;
					break;
				case MAV_CMD.MAV_CMD_NAV_LAST:
					id = 23;
					break;
				case MAV_CMD.MAV_CMD_CONDITION_DELAY:
					id = 24;
					break;
				case MAV_CMD.MAV_CMD_CONDITION_CHANGE_ALT:
					id = 25;
					break;
				case MAV_CMD.MAV_CMD_CONDITION_DISTANCE:
					id = 26;
					break;
				case MAV_CMD.MAV_CMD_CONDITION_YAW:
					id = 27;
					break;
				case MAV_CMD.MAV_CMD_CONDITION_LAST:
					id = 28;
					break;
				case MAV_CMD.MAV_CMD_DO_SET_MODE:
					id = 29;
					break;
				case MAV_CMD.MAV_CMD_DO_JUMP:
					id = 30;
					break;
				case MAV_CMD.MAV_CMD_DO_CHANGE_SPEED:
					id = 31;
					break;
				case MAV_CMD.MAV_CMD_DO_SET_HOME:
					id = 32;
					break;
				case MAV_CMD.MAV_CMD_DO_SET_PARAMETER:
					id = 33;
					break;
				case MAV_CMD.MAV_CMD_DO_SET_RELAY:
					id = 34;
					break;
				case MAV_CMD.MAV_CMD_DO_REPEAT_RELAY:
					id = 35;
					break;
				case MAV_CMD.MAV_CMD_DO_SET_SERVO:
					id = 36;
					break;
				case MAV_CMD.MAV_CMD_DO_REPEAT_SERVO:
					id = 37;
					break;
				case MAV_CMD.MAV_CMD_DO_FLIGHTTERMINATION:
					id = 38;
					break;
				case MAV_CMD.MAV_CMD_DO_CHANGE_ALTITUDE:
					id = 39;
					break;
				case MAV_CMD.MAV_CMD_DO_LAND_START:
					id = 40;
					break;
				case MAV_CMD.MAV_CMD_DO_RALLY_LAND:
					id = 41;
					break;
				case MAV_CMD.MAV_CMD_DO_GO_AROUND:
					id = 42;
					break;
				case MAV_CMD.MAV_CMD_DO_REPOSITION:
					id = 43;
					break;
				case MAV_CMD.MAV_CMD_DO_PAUSE_CONTINUE:
					id = 44;
					break;
				case MAV_CMD.MAV_CMD_DO_SET_REVERSE:
					id = 45;
					break;
				case MAV_CMD.MAV_CMD_DO_CONTROL_VIDEO:
					id = 46;
					break;
				case MAV_CMD.MAV_CMD_DO_SET_ROI:
					id = 47;
					break;
				case MAV_CMD.MAV_CMD_DO_DIGICAM_CONFIGURE:
					id = 48;
					break;
				case MAV_CMD.MAV_CMD_DO_DIGICAM_CONTROL:
					id = 49;
					break;
				case MAV_CMD.MAV_CMD_DO_MOUNT_CONFIGURE:
					id = 50;
					break;
				case MAV_CMD.MAV_CMD_DO_MOUNT_CONTROL:
					id = 51;
					break;
				case MAV_CMD.MAV_CMD_DO_SET_CAM_TRIGG_DIST:
					id = 52;
					break;
				case MAV_CMD.MAV_CMD_DO_FENCE_ENABLE:
					id = 53;
					break;
				case MAV_CMD.MAV_CMD_DO_PARACHUTE:
					id = 54;
					break;
				case MAV_CMD.MAV_CMD_DO_MOTOR_TEST:
					id = 55;
					break;
				case MAV_CMD.MAV_CMD_DO_INVERTED_FLIGHT:
					id = 56;
					break;
				case MAV_CMD.MAV_CMD_NAV_SET_YAW_SPEED:
					id = 57;
					break;
				case MAV_CMD.MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL:
					id = 58;
					break;
				case MAV_CMD.MAV_CMD_DO_MOUNT_CONTROL_QUAT:
					id = 59;
					break;
				case MAV_CMD.MAV_CMD_DO_GUIDED_MASTER:
					id = 60;
					break;
				case MAV_CMD.MAV_CMD_DO_GUIDED_LIMITS:
					id = 61;
					break;
				case MAV_CMD.MAV_CMD_DO_ENGINE_CONTROL:
					id = 62;
					break;
				case MAV_CMD.MAV_CMD_DO_LAST:
					id = 63;
					break;
				case MAV_CMD.MAV_CMD_PREFLIGHT_CALIBRATION:
					id = 64;
					break;
				case MAV_CMD.MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS:
					id = 65;
					break;
				case MAV_CMD.MAV_CMD_PREFLIGHT_UAVCAN:
					id = 66;
					break;
				case MAV_CMD.MAV_CMD_PREFLIGHT_STORAGE:
					id = 67;
					break;
				case MAV_CMD.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
					id = 68;
					break;
				case MAV_CMD.MAV_CMD_OVERRIDE_GOTO:
					id = 69;
					break;
				case MAV_CMD.MAV_CMD_MISSION_START:
					id = 70;
					break;
				case MAV_CMD.MAV_CMD_COMPONENT_ARM_DISARM:
					id = 71;
					break;
				case MAV_CMD.MAV_CMD_GET_HOME_POSITION:
					id = 72;
					break;
				case MAV_CMD.MAV_CMD_START_RX_PAIR:
					id = 73;
					break;
				case MAV_CMD.MAV_CMD_GET_MESSAGE_INTERVAL:
					id = 74;
					break;
				case MAV_CMD.MAV_CMD_SET_MESSAGE_INTERVAL:
					id = 75;
					break;
				case MAV_CMD.MAV_CMD_REQUEST_PROTOCOL_VERSION:
					id = 76;
					break;
				case MAV_CMD.MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES:
					id = 77;
					break;
				case MAV_CMD.MAV_CMD_REQUEST_CAMERA_INFORMATION:
					id = 78;
					break;
				case MAV_CMD.MAV_CMD_REQUEST_CAMERA_SETTINGS:
					id = 79;
					break;
				case MAV_CMD.MAV_CMD_REQUEST_STORAGE_INFORMATION:
					id = 80;
					break;
				case MAV_CMD.MAV_CMD_STORAGE_FORMAT:
					id = 81;
					break;
				case MAV_CMD.MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS:
					id = 82;
					break;
				case MAV_CMD.MAV_CMD_REQUEST_FLIGHT_INFORMATION:
					id = 83;
					break;
				case MAV_CMD.MAV_CMD_RESET_CAMERA_SETTINGS:
					id = 84;
					break;
				case MAV_CMD.MAV_CMD_SET_CAMERA_MODE:
					id = 85;
					break;
				case MAV_CMD.MAV_CMD_IMAGE_START_CAPTURE:
					id = 86;
					break;
				case MAV_CMD.MAV_CMD_IMAGE_STOP_CAPTURE:
					id = 87;
					break;
				case MAV_CMD.MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE:
					id = 88;
					break;
				case MAV_CMD.MAV_CMD_DO_TRIGGER_CONTROL:
					id = 89;
					break;
				case MAV_CMD.MAV_CMD_VIDEO_START_CAPTURE:
					id = 90;
					break;
				case MAV_CMD.MAV_CMD_VIDEO_STOP_CAPTURE:
					id = 91;
					break;
				case MAV_CMD.MAV_CMD_VIDEO_START_STREAMING:
					id = 92;
					break;
				case MAV_CMD.MAV_CMD_VIDEO_STOP_STREAMING:
					id = 93;
					break;
				case MAV_CMD.MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION:
					id = 94;
					break;
				case MAV_CMD.MAV_CMD_LOGGING_START:
					id = 95;
					break;
				case MAV_CMD.MAV_CMD_LOGGING_STOP:
					id = 96;
					break;
				case MAV_CMD.MAV_CMD_AIRFRAME_CONFIGURATION:
					id = 97;
					break;
				case MAV_CMD.MAV_CMD_PANORAMA_CREATE:
					id = 98;
					break;
				case MAV_CMD.MAV_CMD_DO_VTOL_TRANSITION:
					id = 99;
					break;
				case MAV_CMD.MAV_CMD_ARM_AUTHORIZATION_REQUEST:
					id = 100;
					break;
				case MAV_CMD.MAV_CMD_SET_GUIDED_SUBMODE_STANDARD:
					id = 101;
					break;
				case MAV_CMD.MAV_CMD_SET_GUIDED_SUBMODE_CIRCLE:
					id = 102;
					break;
				case MAV_CMD.MAV_CMD_CONDITION_GATE:
					id = 103;
					break;
				case MAV_CMD.MAV_CMD_NAV_FENCE_RETURN_POINT:
					id = 104;
					break;
				case MAV_CMD.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION:
					id = 105;
					break;
				case MAV_CMD.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION:
					id = 106;
					break;
				case MAV_CMD.MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION:
					id = 107;
					break;
				case MAV_CMD.MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION:
					id = 108;
					break;
				case MAV_CMD.MAV_CMD_NAV_RALLY_POINT:
					id = 109;
					break;
				case MAV_CMD.MAV_CMD_UAVCAN_GET_NODE_INFO:
					id = 110;
					break;
				case MAV_CMD.MAV_CMD_PAYLOAD_PREPARE_DEPLOY:
					id = 111;
					break;
				case MAV_CMD.MAV_CMD_PAYLOAD_CONTROL_DEPLOY:
					id = 112;
					break;
				case MAV_CMD.MAV_CMD_WAYPOINT_USER_1:
					id = 113;
					break;
				case MAV_CMD.MAV_CMD_WAYPOINT_USER_2:
					id = 114;
					break;
				case MAV_CMD.MAV_CMD_WAYPOINT_USER_3:
					id = 115;
					break;
				case MAV_CMD.MAV_CMD_WAYPOINT_USER_4:
					id = 116;
					break;
				case MAV_CMD.MAV_CMD_WAYPOINT_USER_5:
					id = 117;
					break;
				case MAV_CMD.MAV_CMD_SPATIAL_USER_1:
					id = 118;
					break;
				case MAV_CMD.MAV_CMD_SPATIAL_USER_2:
					id = 119;
					break;
				case MAV_CMD.MAV_CMD_SPATIAL_USER_3:
					id = 120;
					break;
				case MAV_CMD.MAV_CMD_SPATIAL_USER_4:
					id = 121;
					break;
				case MAV_CMD.MAV_CMD_SPATIAL_USER_5:
					id = 122;
					break;
				case MAV_CMD.MAV_CMD_USER_1:
					id = 123;
					break;
				case MAV_CMD.MAV_CMD_USER_2:
					id = 124;
					break;
				case MAV_CMD.MAV_CMD_USER_3:
					id = 125;
					break;
				case MAV_CMD.MAV_CMD_USER_4:
					id = 126;
					break;
				case MAV_CMD.MAV_CMD_USER_5:
					id = 127;
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
				case MAV_CMD.MAV_CMD_PREFLIGHT_STORAGE_ADVANCED:
					id = 0;
					break;
				case MAV_CMD.MAV_CMD_NAV_WAYPOINT:
					id = 1;
					break;
				case MAV_CMD.MAV_CMD_NAV_LOITER_UNLIM:
					id = 2;
					break;
				case MAV_CMD.MAV_CMD_NAV_LOITER_TURNS:
					id = 3;
					break;
				case MAV_CMD.MAV_CMD_NAV_LOITER_TIME:
					id = 4;
					break;
				case MAV_CMD.MAV_CMD_NAV_RETURN_TO_LAUNCH:
					id = 5;
					break;
				case MAV_CMD.MAV_CMD_NAV_LAND:
					id = 6;
					break;
				case MAV_CMD.MAV_CMD_NAV_TAKEOFF:
					id = 7;
					break;
				case MAV_CMD.MAV_CMD_NAV_LAND_LOCAL:
					id = 8;
					break;
				case MAV_CMD.MAV_CMD_NAV_TAKEOFF_LOCAL:
					id = 9;
					break;
				case MAV_CMD.MAV_CMD_NAV_FOLLOW:
					id = 10;
					break;
				case MAV_CMD.MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT:
					id = 11;
					break;
				case MAV_CMD.MAV_CMD_NAV_LOITER_TO_ALT:
					id = 12;
					break;
				case MAV_CMD.MAV_CMD_DO_FOLLOW:
					id = 13;
					break;
				case MAV_CMD.MAV_CMD_DO_FOLLOW_REPOSITION:
					id = 14;
					break;
				case MAV_CMD.MAV_CMD_NAV_ROI:
					id = 15;
					break;
				case MAV_CMD.MAV_CMD_NAV_PATHPLANNING:
					id = 16;
					break;
				case MAV_CMD.MAV_CMD_NAV_SPLINE_WAYPOINT:
					id = 17;
					break;
				case MAV_CMD.MAV_CMD_NAV_VTOL_TAKEOFF:
					id = 18;
					break;
				case MAV_CMD.MAV_CMD_NAV_VTOL_LAND:
					id = 19;
					break;
				case MAV_CMD.MAV_CMD_NAV_GUIDED_ENABLE:
					id = 20;
					break;
				case MAV_CMD.MAV_CMD_NAV_DELAY:
					id = 21;
					break;
				case MAV_CMD.MAV_CMD_NAV_PAYLOAD_PLACE:
					id = 22;
					break;
				case MAV_CMD.MAV_CMD_NAV_LAST:
					id = 23;
					break;
				case MAV_CMD.MAV_CMD_CONDITION_DELAY:
					id = 24;
					break;
				case MAV_CMD.MAV_CMD_CONDITION_CHANGE_ALT:
					id = 25;
					break;
				case MAV_CMD.MAV_CMD_CONDITION_DISTANCE:
					id = 26;
					break;
				case MAV_CMD.MAV_CMD_CONDITION_YAW:
					id = 27;
					break;
				case MAV_CMD.MAV_CMD_CONDITION_LAST:
					id = 28;
					break;
				case MAV_CMD.MAV_CMD_DO_SET_MODE:
					id = 29;
					break;
				case MAV_CMD.MAV_CMD_DO_JUMP:
					id = 30;
					break;
				case MAV_CMD.MAV_CMD_DO_CHANGE_SPEED:
					id = 31;
					break;
				case MAV_CMD.MAV_CMD_DO_SET_HOME:
					id = 32;
					break;
				case MAV_CMD.MAV_CMD_DO_SET_PARAMETER:
					id = 33;
					break;
				case MAV_CMD.MAV_CMD_DO_SET_RELAY:
					id = 34;
					break;
				case MAV_CMD.MAV_CMD_DO_REPEAT_RELAY:
					id = 35;
					break;
				case MAV_CMD.MAV_CMD_DO_SET_SERVO:
					id = 36;
					break;
				case MAV_CMD.MAV_CMD_DO_REPEAT_SERVO:
					id = 37;
					break;
				case MAV_CMD.MAV_CMD_DO_FLIGHTTERMINATION:
					id = 38;
					break;
				case MAV_CMD.MAV_CMD_DO_CHANGE_ALTITUDE:
					id = 39;
					break;
				case MAV_CMD.MAV_CMD_DO_LAND_START:
					id = 40;
					break;
				case MAV_CMD.MAV_CMD_DO_RALLY_LAND:
					id = 41;
					break;
				case MAV_CMD.MAV_CMD_DO_GO_AROUND:
					id = 42;
					break;
				case MAV_CMD.MAV_CMD_DO_REPOSITION:
					id = 43;
					break;
				case MAV_CMD.MAV_CMD_DO_PAUSE_CONTINUE:
					id = 44;
					break;
				case MAV_CMD.MAV_CMD_DO_SET_REVERSE:
					id = 45;
					break;
				case MAV_CMD.MAV_CMD_DO_CONTROL_VIDEO:
					id = 46;
					break;
				case MAV_CMD.MAV_CMD_DO_SET_ROI:
					id = 47;
					break;
				case MAV_CMD.MAV_CMD_DO_DIGICAM_CONFIGURE:
					id = 48;
					break;
				case MAV_CMD.MAV_CMD_DO_DIGICAM_CONTROL:
					id = 49;
					break;
				case MAV_CMD.MAV_CMD_DO_MOUNT_CONFIGURE:
					id = 50;
					break;
				case MAV_CMD.MAV_CMD_DO_MOUNT_CONTROL:
					id = 51;
					break;
				case MAV_CMD.MAV_CMD_DO_SET_CAM_TRIGG_DIST:
					id = 52;
					break;
				case MAV_CMD.MAV_CMD_DO_FENCE_ENABLE:
					id = 53;
					break;
				case MAV_CMD.MAV_CMD_DO_PARACHUTE:
					id = 54;
					break;
				case MAV_CMD.MAV_CMD_DO_MOTOR_TEST:
					id = 55;
					break;
				case MAV_CMD.MAV_CMD_DO_INVERTED_FLIGHT:
					id = 56;
					break;
				case MAV_CMD.MAV_CMD_NAV_SET_YAW_SPEED:
					id = 57;
					break;
				case MAV_CMD.MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL:
					id = 58;
					break;
				case MAV_CMD.MAV_CMD_DO_MOUNT_CONTROL_QUAT:
					id = 59;
					break;
				case MAV_CMD.MAV_CMD_DO_GUIDED_MASTER:
					id = 60;
					break;
				case MAV_CMD.MAV_CMD_DO_GUIDED_LIMITS:
					id = 61;
					break;
				case MAV_CMD.MAV_CMD_DO_ENGINE_CONTROL:
					id = 62;
					break;
				case MAV_CMD.MAV_CMD_DO_LAST:
					id = 63;
					break;
				case MAV_CMD.MAV_CMD_PREFLIGHT_CALIBRATION:
					id = 64;
					break;
				case MAV_CMD.MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS:
					id = 65;
					break;
				case MAV_CMD.MAV_CMD_PREFLIGHT_UAVCAN:
					id = 66;
					break;
				case MAV_CMD.MAV_CMD_PREFLIGHT_STORAGE:
					id = 67;
					break;
				case MAV_CMD.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
					id = 68;
					break;
				case MAV_CMD.MAV_CMD_OVERRIDE_GOTO:
					id = 69;
					break;
				case MAV_CMD.MAV_CMD_MISSION_START:
					id = 70;
					break;
				case MAV_CMD.MAV_CMD_COMPONENT_ARM_DISARM:
					id = 71;
					break;
				case MAV_CMD.MAV_CMD_GET_HOME_POSITION:
					id = 72;
					break;
				case MAV_CMD.MAV_CMD_START_RX_PAIR:
					id = 73;
					break;
				case MAV_CMD.MAV_CMD_GET_MESSAGE_INTERVAL:
					id = 74;
					break;
				case MAV_CMD.MAV_CMD_SET_MESSAGE_INTERVAL:
					id = 75;
					break;
				case MAV_CMD.MAV_CMD_REQUEST_PROTOCOL_VERSION:
					id = 76;
					break;
				case MAV_CMD.MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES:
					id = 77;
					break;
				case MAV_CMD.MAV_CMD_REQUEST_CAMERA_INFORMATION:
					id = 78;
					break;
				case MAV_CMD.MAV_CMD_REQUEST_CAMERA_SETTINGS:
					id = 79;
					break;
				case MAV_CMD.MAV_CMD_REQUEST_STORAGE_INFORMATION:
					id = 80;
					break;
				case MAV_CMD.MAV_CMD_STORAGE_FORMAT:
					id = 81;
					break;
				case MAV_CMD.MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS:
					id = 82;
					break;
				case MAV_CMD.MAV_CMD_REQUEST_FLIGHT_INFORMATION:
					id = 83;
					break;
				case MAV_CMD.MAV_CMD_RESET_CAMERA_SETTINGS:
					id = 84;
					break;
				case MAV_CMD.MAV_CMD_SET_CAMERA_MODE:
					id = 85;
					break;
				case MAV_CMD.MAV_CMD_IMAGE_START_CAPTURE:
					id = 86;
					break;
				case MAV_CMD.MAV_CMD_IMAGE_STOP_CAPTURE:
					id = 87;
					break;
				case MAV_CMD.MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE:
					id = 88;
					break;
				case MAV_CMD.MAV_CMD_DO_TRIGGER_CONTROL:
					id = 89;
					break;
				case MAV_CMD.MAV_CMD_VIDEO_START_CAPTURE:
					id = 90;
					break;
				case MAV_CMD.MAV_CMD_VIDEO_STOP_CAPTURE:
					id = 91;
					break;
				case MAV_CMD.MAV_CMD_VIDEO_START_STREAMING:
					id = 92;
					break;
				case MAV_CMD.MAV_CMD_VIDEO_STOP_STREAMING:
					id = 93;
					break;
				case MAV_CMD.MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION:
					id = 94;
					break;
				case MAV_CMD.MAV_CMD_LOGGING_START:
					id = 95;
					break;
				case MAV_CMD.MAV_CMD_LOGGING_STOP:
					id = 96;
					break;
				case MAV_CMD.MAV_CMD_AIRFRAME_CONFIGURATION:
					id = 97;
					break;
				case MAV_CMD.MAV_CMD_PANORAMA_CREATE:
					id = 98;
					break;
				case MAV_CMD.MAV_CMD_DO_VTOL_TRANSITION:
					id = 99;
					break;
				case MAV_CMD.MAV_CMD_ARM_AUTHORIZATION_REQUEST:
					id = 100;
					break;
				case MAV_CMD.MAV_CMD_SET_GUIDED_SUBMODE_STANDARD:
					id = 101;
					break;
				case MAV_CMD.MAV_CMD_SET_GUIDED_SUBMODE_CIRCLE:
					id = 102;
					break;
				case MAV_CMD.MAV_CMD_CONDITION_GATE:
					id = 103;
					break;
				case MAV_CMD.MAV_CMD_NAV_FENCE_RETURN_POINT:
					id = 104;
					break;
				case MAV_CMD.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION:
					id = 105;
					break;
				case MAV_CMD.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION:
					id = 106;
					break;
				case MAV_CMD.MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION:
					id = 107;
					break;
				case MAV_CMD.MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION:
					id = 108;
					break;
				case MAV_CMD.MAV_CMD_NAV_RALLY_POINT:
					id = 109;
					break;
				case MAV_CMD.MAV_CMD_UAVCAN_GET_NODE_INFO:
					id = 110;
					break;
				case MAV_CMD.MAV_CMD_PAYLOAD_PREPARE_DEPLOY:
					id = 111;
					break;
				case MAV_CMD.MAV_CMD_PAYLOAD_CONTROL_DEPLOY:
					id = 112;
					break;
				case MAV_CMD.MAV_CMD_WAYPOINT_USER_1:
					id = 113;
					break;
				case MAV_CMD.MAV_CMD_WAYPOINT_USER_2:
					id = 114;
					break;
				case MAV_CMD.MAV_CMD_WAYPOINT_USER_3:
					id = 115;
					break;
				case MAV_CMD.MAV_CMD_WAYPOINT_USER_4:
					id = 116;
					break;
				case MAV_CMD.MAV_CMD_WAYPOINT_USER_5:
					id = 117;
					break;
				case MAV_CMD.MAV_CMD_SPATIAL_USER_1:
					id = 118;
					break;
				case MAV_CMD.MAV_CMD_SPATIAL_USER_2:
					id = 119;
					break;
				case MAV_CMD.MAV_CMD_SPATIAL_USER_3:
					id = 120;
					break;
				case MAV_CMD.MAV_CMD_SPATIAL_USER_4:
					id = 121;
					break;
				case MAV_CMD.MAV_CMD_SPATIAL_USER_5:
					id = 122;
					break;
				case MAV_CMD.MAV_CMD_USER_1:
					id = 123;
					break;
				case MAV_CMD.MAV_CMD_USER_2:
					id = 124;
					break;
				case MAV_CMD.MAV_CMD_USER_3:
					id = 125;
					break;
				case MAV_CMD.MAV_CMD_USER_4:
					id = 126;
					break;
				case MAV_CMD.MAV_CMD_USER_5:
					id = 127;
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
				case MAV_CMD.MAV_CMD_PREFLIGHT_STORAGE_ADVANCED:
					id = 0;
					break;
				case MAV_CMD.MAV_CMD_NAV_WAYPOINT:
					id = 1;
					break;
				case MAV_CMD.MAV_CMD_NAV_LOITER_UNLIM:
					id = 2;
					break;
				case MAV_CMD.MAV_CMD_NAV_LOITER_TURNS:
					id = 3;
					break;
				case MAV_CMD.MAV_CMD_NAV_LOITER_TIME:
					id = 4;
					break;
				case MAV_CMD.MAV_CMD_NAV_RETURN_TO_LAUNCH:
					id = 5;
					break;
				case MAV_CMD.MAV_CMD_NAV_LAND:
					id = 6;
					break;
				case MAV_CMD.MAV_CMD_NAV_TAKEOFF:
					id = 7;
					break;
				case MAV_CMD.MAV_CMD_NAV_LAND_LOCAL:
					id = 8;
					break;
				case MAV_CMD.MAV_CMD_NAV_TAKEOFF_LOCAL:
					id = 9;
					break;
				case MAV_CMD.MAV_CMD_NAV_FOLLOW:
					id = 10;
					break;
				case MAV_CMD.MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT:
					id = 11;
					break;
				case MAV_CMD.MAV_CMD_NAV_LOITER_TO_ALT:
					id = 12;
					break;
				case MAV_CMD.MAV_CMD_DO_FOLLOW:
					id = 13;
					break;
				case MAV_CMD.MAV_CMD_DO_FOLLOW_REPOSITION:
					id = 14;
					break;
				case MAV_CMD.MAV_CMD_NAV_ROI:
					id = 15;
					break;
				case MAV_CMD.MAV_CMD_NAV_PATHPLANNING:
					id = 16;
					break;
				case MAV_CMD.MAV_CMD_NAV_SPLINE_WAYPOINT:
					id = 17;
					break;
				case MAV_CMD.MAV_CMD_NAV_VTOL_TAKEOFF:
					id = 18;
					break;
				case MAV_CMD.MAV_CMD_NAV_VTOL_LAND:
					id = 19;
					break;
				case MAV_CMD.MAV_CMD_NAV_GUIDED_ENABLE:
					id = 20;
					break;
				case MAV_CMD.MAV_CMD_NAV_DELAY:
					id = 21;
					break;
				case MAV_CMD.MAV_CMD_NAV_PAYLOAD_PLACE:
					id = 22;
					break;
				case MAV_CMD.MAV_CMD_NAV_LAST:
					id = 23;
					break;
				case MAV_CMD.MAV_CMD_CONDITION_DELAY:
					id = 24;
					break;
				case MAV_CMD.MAV_CMD_CONDITION_CHANGE_ALT:
					id = 25;
					break;
				case MAV_CMD.MAV_CMD_CONDITION_DISTANCE:
					id = 26;
					break;
				case MAV_CMD.MAV_CMD_CONDITION_YAW:
					id = 27;
					break;
				case MAV_CMD.MAV_CMD_CONDITION_LAST:
					id = 28;
					break;
				case MAV_CMD.MAV_CMD_DO_SET_MODE:
					id = 29;
					break;
				case MAV_CMD.MAV_CMD_DO_JUMP:
					id = 30;
					break;
				case MAV_CMD.MAV_CMD_DO_CHANGE_SPEED:
					id = 31;
					break;
				case MAV_CMD.MAV_CMD_DO_SET_HOME:
					id = 32;
					break;
				case MAV_CMD.MAV_CMD_DO_SET_PARAMETER:
					id = 33;
					break;
				case MAV_CMD.MAV_CMD_DO_SET_RELAY:
					id = 34;
					break;
				case MAV_CMD.MAV_CMD_DO_REPEAT_RELAY:
					id = 35;
					break;
				case MAV_CMD.MAV_CMD_DO_SET_SERVO:
					id = 36;
					break;
				case MAV_CMD.MAV_CMD_DO_REPEAT_SERVO:
					id = 37;
					break;
				case MAV_CMD.MAV_CMD_DO_FLIGHTTERMINATION:
					id = 38;
					break;
				case MAV_CMD.MAV_CMD_DO_CHANGE_ALTITUDE:
					id = 39;
					break;
				case MAV_CMD.MAV_CMD_DO_LAND_START:
					id = 40;
					break;
				case MAV_CMD.MAV_CMD_DO_RALLY_LAND:
					id = 41;
					break;
				case MAV_CMD.MAV_CMD_DO_GO_AROUND:
					id = 42;
					break;
				case MAV_CMD.MAV_CMD_DO_REPOSITION:
					id = 43;
					break;
				case MAV_CMD.MAV_CMD_DO_PAUSE_CONTINUE:
					id = 44;
					break;
				case MAV_CMD.MAV_CMD_DO_SET_REVERSE:
					id = 45;
					break;
				case MAV_CMD.MAV_CMD_DO_CONTROL_VIDEO:
					id = 46;
					break;
				case MAV_CMD.MAV_CMD_DO_SET_ROI:
					id = 47;
					break;
				case MAV_CMD.MAV_CMD_DO_DIGICAM_CONFIGURE:
					id = 48;
					break;
				case MAV_CMD.MAV_CMD_DO_DIGICAM_CONTROL:
					id = 49;
					break;
				case MAV_CMD.MAV_CMD_DO_MOUNT_CONFIGURE:
					id = 50;
					break;
				case MAV_CMD.MAV_CMD_DO_MOUNT_CONTROL:
					id = 51;
					break;
				case MAV_CMD.MAV_CMD_DO_SET_CAM_TRIGG_DIST:
					id = 52;
					break;
				case MAV_CMD.MAV_CMD_DO_FENCE_ENABLE:
					id = 53;
					break;
				case MAV_CMD.MAV_CMD_DO_PARACHUTE:
					id = 54;
					break;
				case MAV_CMD.MAV_CMD_DO_MOTOR_TEST:
					id = 55;
					break;
				case MAV_CMD.MAV_CMD_DO_INVERTED_FLIGHT:
					id = 56;
					break;
				case MAV_CMD.MAV_CMD_NAV_SET_YAW_SPEED:
					id = 57;
					break;
				case MAV_CMD.MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL:
					id = 58;
					break;
				case MAV_CMD.MAV_CMD_DO_MOUNT_CONTROL_QUAT:
					id = 59;
					break;
				case MAV_CMD.MAV_CMD_DO_GUIDED_MASTER:
					id = 60;
					break;
				case MAV_CMD.MAV_CMD_DO_GUIDED_LIMITS:
					id = 61;
					break;
				case MAV_CMD.MAV_CMD_DO_ENGINE_CONTROL:
					id = 62;
					break;
				case MAV_CMD.MAV_CMD_DO_LAST:
					id = 63;
					break;
				case MAV_CMD.MAV_CMD_PREFLIGHT_CALIBRATION:
					id = 64;
					break;
				case MAV_CMD.MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS:
					id = 65;
					break;
				case MAV_CMD.MAV_CMD_PREFLIGHT_UAVCAN:
					id = 66;
					break;
				case MAV_CMD.MAV_CMD_PREFLIGHT_STORAGE:
					id = 67;
					break;
				case MAV_CMD.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
					id = 68;
					break;
				case MAV_CMD.MAV_CMD_OVERRIDE_GOTO:
					id = 69;
					break;
				case MAV_CMD.MAV_CMD_MISSION_START:
					id = 70;
					break;
				case MAV_CMD.MAV_CMD_COMPONENT_ARM_DISARM:
					id = 71;
					break;
				case MAV_CMD.MAV_CMD_GET_HOME_POSITION:
					id = 72;
					break;
				case MAV_CMD.MAV_CMD_START_RX_PAIR:
					id = 73;
					break;
				case MAV_CMD.MAV_CMD_GET_MESSAGE_INTERVAL:
					id = 74;
					break;
				case MAV_CMD.MAV_CMD_SET_MESSAGE_INTERVAL:
					id = 75;
					break;
				case MAV_CMD.MAV_CMD_REQUEST_PROTOCOL_VERSION:
					id = 76;
					break;
				case MAV_CMD.MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES:
					id = 77;
					break;
				case MAV_CMD.MAV_CMD_REQUEST_CAMERA_INFORMATION:
					id = 78;
					break;
				case MAV_CMD.MAV_CMD_REQUEST_CAMERA_SETTINGS:
					id = 79;
					break;
				case MAV_CMD.MAV_CMD_REQUEST_STORAGE_INFORMATION:
					id = 80;
					break;
				case MAV_CMD.MAV_CMD_STORAGE_FORMAT:
					id = 81;
					break;
				case MAV_CMD.MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS:
					id = 82;
					break;
				case MAV_CMD.MAV_CMD_REQUEST_FLIGHT_INFORMATION:
					id = 83;
					break;
				case MAV_CMD.MAV_CMD_RESET_CAMERA_SETTINGS:
					id = 84;
					break;
				case MAV_CMD.MAV_CMD_SET_CAMERA_MODE:
					id = 85;
					break;
				case MAV_CMD.MAV_CMD_IMAGE_START_CAPTURE:
					id = 86;
					break;
				case MAV_CMD.MAV_CMD_IMAGE_STOP_CAPTURE:
					id = 87;
					break;
				case MAV_CMD.MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE:
					id = 88;
					break;
				case MAV_CMD.MAV_CMD_DO_TRIGGER_CONTROL:
					id = 89;
					break;
				case MAV_CMD.MAV_CMD_VIDEO_START_CAPTURE:
					id = 90;
					break;
				case MAV_CMD.MAV_CMD_VIDEO_STOP_CAPTURE:
					id = 91;
					break;
				case MAV_CMD.MAV_CMD_VIDEO_START_STREAMING:
					id = 92;
					break;
				case MAV_CMD.MAV_CMD_VIDEO_STOP_STREAMING:
					id = 93;
					break;
				case MAV_CMD.MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION:
					id = 94;
					break;
				case MAV_CMD.MAV_CMD_LOGGING_START:
					id = 95;
					break;
				case MAV_CMD.MAV_CMD_LOGGING_STOP:
					id = 96;
					break;
				case MAV_CMD.MAV_CMD_AIRFRAME_CONFIGURATION:
					id = 97;
					break;
				case MAV_CMD.MAV_CMD_PANORAMA_CREATE:
					id = 98;
					break;
				case MAV_CMD.MAV_CMD_DO_VTOL_TRANSITION:
					id = 99;
					break;
				case MAV_CMD.MAV_CMD_ARM_AUTHORIZATION_REQUEST:
					id = 100;
					break;
				case MAV_CMD.MAV_CMD_SET_GUIDED_SUBMODE_STANDARD:
					id = 101;
					break;
				case MAV_CMD.MAV_CMD_SET_GUIDED_SUBMODE_CIRCLE:
					id = 102;
					break;
				case MAV_CMD.MAV_CMD_CONDITION_GATE:
					id = 103;
					break;
				case MAV_CMD.MAV_CMD_NAV_FENCE_RETURN_POINT:
					id = 104;
					break;
				case MAV_CMD.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION:
					id = 105;
					break;
				case MAV_CMD.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION:
					id = 106;
					break;
				case MAV_CMD.MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION:
					id = 107;
					break;
				case MAV_CMD.MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION:
					id = 108;
					break;
				case MAV_CMD.MAV_CMD_NAV_RALLY_POINT:
					id = 109;
					break;
				case MAV_CMD.MAV_CMD_UAVCAN_GET_NODE_INFO:
					id = 110;
					break;
				case MAV_CMD.MAV_CMD_PAYLOAD_PREPARE_DEPLOY:
					id = 111;
					break;
				case MAV_CMD.MAV_CMD_PAYLOAD_CONTROL_DEPLOY:
					id = 112;
					break;
				case MAV_CMD.MAV_CMD_WAYPOINT_USER_1:
					id = 113;
					break;
				case MAV_CMD.MAV_CMD_WAYPOINT_USER_2:
					id = 114;
					break;
				case MAV_CMD.MAV_CMD_WAYPOINT_USER_3:
					id = 115;
					break;
				case MAV_CMD.MAV_CMD_WAYPOINT_USER_4:
					id = 116;
					break;
				case MAV_CMD.MAV_CMD_WAYPOINT_USER_5:
					id = 117;
					break;
				case MAV_CMD.MAV_CMD_SPATIAL_USER_1:
					id = 118;
					break;
				case MAV_CMD.MAV_CMD_SPATIAL_USER_2:
					id = 119;
					break;
				case MAV_CMD.MAV_CMD_SPATIAL_USER_3:
					id = 120;
					break;
				case MAV_CMD.MAV_CMD_SPATIAL_USER_4:
					id = 121;
					break;
				case MAV_CMD.MAV_CMD_SPATIAL_USER_5:
					id = 122;
					break;
				case MAV_CMD.MAV_CMD_USER_1:
					id = 123;
					break;
				case MAV_CMD.MAV_CMD_USER_2:
					id = 124;
					break;
				case MAV_CMD.MAV_CMD_USER_3:
					id = 125;
					break;
				case MAV_CMD.MAV_CMD_USER_4:
					id = 126;
					break;
				case MAV_CMD.MAV_CMD_USER_5:
					id = 127;
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

	public static class MANUAL_SETPOINT extends GroundControl.MANUAL_SETPOINT {
		public void time_boot_ms_SET(long src) //Timestamp in milliseconds since system boot
		{ set_bytes((src) & -1L, 4, data, 0); }

		public void roll_SET(float src) //Desired roll rate in radians per second
		{ set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 4); }

		public void pitch_SET(float src) //Desired pitch rate in radians per second
		{ set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 8); }

		public void yaw_SET(float src) //Desired yaw rate in radians per second
		{ set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 12); }

		public void thrust_SET(float src) //Collective thrust, normalized to 0 .. 1
		{ set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 16); }

		public void mode_switch_SET(char src) //Flight mode switch position, 0.. 255
		{ set_bytes((char) (src) & -1L, 1, data, 20); }

		public void manual_override_switch_SET(char src) //Override mode switch position, 0.. 255
		{ set_bytes((char) (src) & -1L, 1, data, 21); }
	}

	public static class SET_ATTITUDE_TARGET extends GroundControl.SET_ATTITUDE_TARGET {
		public void time_boot_ms_SET(long src) //Timestamp in milliseconds since system boot
		{ set_bytes((src) & -1L, 4, data, 0); }

		public void target_system_SET(char src) //System ID
		{ set_bytes((char) (src) & -1L, 1, data, 4); }

		public void target_component_SET(char src) //Component ID
		{ set_bytes((char) (src) & -1L, 1, data, 5); }

		/**
		 * Mappings: If any of these bits are set, the corresponding input should be ignored: bit 1: body roll rate,
		 * bit 2: body pitch rate, bit 3: body yaw rate. bit 4-bit 6: reserved, bit 7: throttle, bit 8: attitud
		 */
		public void type_mask_SET(char src) { set_bytes((char) (src) & -1L, 1, data, 6); }

		public void q_SET(float[] src, int pos)  //Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
		{
			for (int BYTE = 7, src_max = pos + 4; pos < src_max; pos++, BYTE += 4)
				set_bytes(Float.floatToIntBits(src[pos]) & -1L, 4, data, BYTE);
		}

		public void body_roll_rate_SET(float src) //Body roll rate in radians per second
		{ set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 23); }

		public void body_pitch_rate_SET(float src) //Body roll rate in radians per second
		{ set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 27); }

		public void body_yaw_rate_SET(float src) //Body roll rate in radians per second
		{ set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 31); }

		public void thrust_SET(float src) //Collective thrust, normalized to 0 .. 1 (-1 .. 1 for vehicles capable of reverse trust)
		{ set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 35); }
	}

	public static class ATTITUDE_TARGET extends GroundControl.ATTITUDE_TARGET {
		public void time_boot_ms_SET(long src) //Timestamp in milliseconds since system boot
		{ set_bytes((src) & -1L, 4, data, 0); }

		/**
		 * Mappings: If any of these bits are set, the corresponding input should be ignored: bit 1: body roll rate,
		 * bit 2: body pitch rate, bit 3: body yaw rate. bit 4-bit 7: reserved, bit 8: attitud
		 */
		public void type_mask_SET(char src) { set_bytes((char) (src) & -1L, 1, data, 4); }

		public void q_SET(float[] src, int pos)  //Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
		{
			for (int BYTE = 5, src_max = pos + 4; pos < src_max; pos++, BYTE += 4)
				set_bytes(Float.floatToIntBits(src[pos]) & -1L, 4, data, BYTE);
		}

		public void body_roll_rate_SET(float src) //Body roll rate in radians per second
		{ set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 21); }

		public void body_pitch_rate_SET(float src) //Body pitch rate in radians per second
		{ set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 25); }

		public void body_yaw_rate_SET(float src) //Body yaw rate in radians per second
		{ set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 29); }

		public void thrust_SET(float src) //Collective thrust, normalized to 0 .. 1 (-1 .. 1 for vehicles capable of reverse trust)
		{ set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 33); }
	}

	public static class SET_POSITION_TARGET_LOCAL_NED extends GroundControl.SET_POSITION_TARGET_LOCAL_NED {
		/**
		 * Bitmask to indicate which dimensions should be ignored by the vehicle: a value of 0b0000000000000000 or
		 * 0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. If bit 10 is set
		 * the floats afx afy afz should be interpreted as force instead of acceleration. Mapping: bit 1: x, bit
		 * 2: y, bit 3: z, bit 4: vx, bit 5: vy, bit 6: vz, bit 7: ax, bit 8: ay, bit 9: az, bit 10: is force setpoint,
		 * bit 11: yaw, bit 12: yaw rat
		 */
		public void type_mask_SET(char src) { set_bytes((char) (src) & -1L, 2, data, 0); }

		public void time_boot_ms_SET(long src) //Timestamp in milliseconds since system boot
		{ set_bytes((src) & -1L, 4, data, 2); }

		public void target_system_SET(char src) //System ID
		{ set_bytes((char) (src) & -1L, 1, data, 6); }

		public void target_component_SET(char src) //Component ID
		{ set_bytes((char) (src) & -1L, 1, data, 7); }

		public void x_SET(float src) //X Position in NED frame in meters
		{ set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 8); }

		public void y_SET(float src) //Y Position in NED frame in meters
		{ set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 12); }

		public void z_SET(float src) //Z Position in NED frame in meters (note, altitude is negative in NED)
		{ set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 16); }

		public void vx_SET(float src) //X velocity in NED frame in meter / s
		{ set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 20); }

		public void vy_SET(float src) //Y velocity in NED frame in meter / s
		{ set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 24); }

		public void vz_SET(float src) //Z velocity in NED frame in meter / s
		{ set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 28); }

		public void afx_SET(float src) //X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
		{ set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 32); }

		public void afy_SET(float src) //Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
		{ set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 36); }

		public void afz_SET(float src) //Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
		{ set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 40); }

		public void yaw_SET(float src) //yaw setpoint in rad
		{ set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 44); }

		public void yaw_rate_SET(float src) //yaw rate setpoint in rad/s
		{ set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 48); }

		/**
		 * Valid options are: MAV_FRAME_LOCAL_NED = 1, MAV_FRAME_LOCAL_OFFSET_NED = 7, MAV_FRAME_BODY_NED = 8, MAV_FRAME_BODY_OFFSET_NED
		 * =
		 */
		public void coordinate_frame_SET(@MAV_FRAME int src) { set_bits(-0 + src, 4, data, 416); }
	}

	public static class FLEXIFUNCTION_SET extends GroundControl.FLEXIFUNCTION_SET {
		public char target_system_GET()//System ID
		{ return (char) ((char) get_bytes(data, 0, 1)); }

		public char target_component_GET()//Component ID
		{ return (char) ((char) get_bytes(data, 1, 1)); }
	}

	public static class FLEXIFUNCTION_READ_REQ extends GroundControl.FLEXIFUNCTION_READ_REQ {
		public char target_system_GET()//System ID
		{ return (char) ((char) get_bytes(data, 0, 1)); }

		public char target_component_GET()//Component ID
		{ return (char) ((char) get_bytes(data, 1, 1)); }

		public short read_req_type_GET()//Type of flexifunction data requested
		{ return (short) ((short) get_bytes(data, 2, 2)); }

		public short data_index_GET()//index into data where needed
		{ return (short) ((short) get_bytes(data, 4, 2)); }
	}

	public static class FLEXIFUNCTION_BUFFER_FUNCTION extends GroundControl.FLEXIFUNCTION_BUFFER_FUNCTION {
		public char func_index_GET()//Function index
		{ return (char) ((char) get_bytes(data, 0, 2)); }

		public char func_count_GET()//Total count of functions
		{ return (char) ((char) get_bytes(data, 2, 2)); }

		public char data_address_GET()//Address in the flexifunction data, Set to 0xFFFF to use address in target memory
		{ return (char) ((char) get_bytes(data, 4, 2)); }

		public char data_size_GET()//Size of the
		{ return (char) ((char) get_bytes(data, 6, 2)); }

		public char target_system_GET()//System ID
		{ return (char) ((char) get_bytes(data, 8, 1)); }

		public char target_component_GET()//Component ID
		{ return (char) ((char) get_bytes(data, 9, 1)); }

		public byte[] data__GET(byte[] dst_ch, int pos)  //Settings data
		{
			for (int BYTE = 10, dst_max = pos + 48; pos < dst_max; pos++, BYTE += 1)
				dst_ch[pos] = (byte) ((byte) get_bytes(data, BYTE, 1));
			return dst_ch;
		}

		public byte[] data__GET()//Settings data
		{return data__GET(new byte[48], 0);}
	}

	public static class FLEXIFUNCTION_BUFFER_FUNCTION_ACK extends GroundControl.FLEXIFUNCTION_BUFFER_FUNCTION_ACK {
		public char func_index_GET()//Function index
		{ return (char) ((char) get_bytes(data, 0, 2)); }

		public char result_GET()//result of acknowledge, 0=fail, 1=good
		{ return (char) ((char) get_bytes(data, 2, 2)); }

		public char target_system_GET()//System ID
		{ return (char) ((char) get_bytes(data, 4, 1)); }

		public char target_component_GET()//Component ID
		{ return (char) ((char) get_bytes(data, 5, 1)); }
	}

	public static class FLEXIFUNCTION_DIRECTORY extends GroundControl.FLEXIFUNCTION_DIRECTORY {
		public char target_system_GET()//System ID
		{ return (char) ((char) get_bytes(data, 0, 1)); }

		public char target_component_GET()//Component ID
		{ return (char) ((char) get_bytes(data, 1, 1)); }

		public char directory_type_GET()//0=inputs, 1=outputs
		{ return (char) ((char) get_bytes(data, 2, 1)); }

		public char start_index_GET()//index of first directory entry to write
		{ return (char) ((char) get_bytes(data, 3, 1)); }

		public char count_GET()//count of directory entries to write
		{ return (char) ((char) get_bytes(data, 4, 1)); }

		public byte[] directory_data_GET(byte[] dst_ch, int pos)  //Settings data
		{
			for (int BYTE = 5, dst_max = pos + 48; pos < dst_max; pos++, BYTE += 1)
				dst_ch[pos] = (byte) ((byte) get_bytes(data, BYTE, 1));
			return dst_ch;
		}

		public byte[] directory_data_GET()//Settings data
		{return directory_data_GET(new byte[48], 0);}
	}

	public static class FLEXIFUNCTION_DIRECTORY_ACK extends GroundControl.FLEXIFUNCTION_DIRECTORY_ACK {
		public char result_GET()//result of acknowledge, 0=fail, 1=good
		{ return (char) ((char) get_bytes(data, 0, 2)); }

		public char target_system_GET()//System ID
		{ return (char) ((char) get_bytes(data, 2, 1)); }

		public char target_component_GET()//Component ID
		{ return (char) ((char) get_bytes(data, 3, 1)); }

		public char directory_type_GET()//0=inputs, 1=outputs
		{ return (char) ((char) get_bytes(data, 4, 1)); }

		public char start_index_GET()//index of first directory entry to write
		{ return (char) ((char) get_bytes(data, 5, 1)); }

		public char count_GET()//count of directory entries to write
		{ return (char) ((char) get_bytes(data, 6, 1)); }
	}

	public static class FLEXIFUNCTION_COMMAND extends GroundControl.FLEXIFUNCTION_COMMAND {
		public char target_system_GET()//System ID
		{ return (char) ((char) get_bytes(data, 0, 1)); }

		public char target_component_GET()//Component ID
		{ return (char) ((char) get_bytes(data, 1, 1)); }

		public char command_type_GET()//Flexifunction command type
		{ return (char) ((char) get_bytes(data, 2, 1)); }
	}

	public static class FLEXIFUNCTION_COMMAND_ACK extends GroundControl.FLEXIFUNCTION_COMMAND_ACK {
		public char command_type_GET()//Command acknowledged
		{ return (char) ((char) get_bytes(data, 0, 2)); }

		public char result_GET()//result of acknowledge
		{ return (char) ((char) get_bytes(data, 2, 2)); }
	}

	public static class SERIAL_UDB_EXTRA_F2_A extends GroundControl.SERIAL_UDB_EXTRA_F2_A {
		public char sue_waypoint_index_GET()//Serial UDB Extra Waypoint Index
		{ return (char) ((char) get_bytes(data, 0, 2)); }

		public char sue_cog_GET()//Serial UDB Extra GPS Course Over Ground
		{ return (char) ((char) get_bytes(data, 2, 2)); }

		public char sue_cpu_load_GET()//Serial UDB Extra CPU Load
		{ return (char) ((char) get_bytes(data, 4, 2)); }

		public char sue_air_speed_3DIMU_GET()//Serial UDB Extra 3D IMU Air Speed
		{ return (char) ((char) get_bytes(data, 6, 2)); }

		public long sue_time_GET()//Serial UDB Extra Time
		{ return (get_bytes(data, 8, 4)); }

		public char sue_status_GET()//Serial UDB Extra Status
		{ return (char) ((char) get_bytes(data, 12, 1)); }

		public int sue_latitude_GET()//Serial UDB Extra Latitude
		{ return (int) ((int) get_bytes(data, 13, 4)); }

		public int sue_longitude_GET()//Serial UDB Extra Longitude
		{ return (int) ((int) get_bytes(data, 17, 4)); }

		public int sue_altitude_GET()//Serial UDB Extra Altitude
		{ return (int) ((int) get_bytes(data, 21, 4)); }

		public short sue_rmat0_GET()//Serial UDB Extra Rmat 0
		{ return (short) ((short) get_bytes(data, 25, 2)); }

		public short sue_rmat1_GET()//Serial UDB Extra Rmat 1
		{ return (short) ((short) get_bytes(data, 27, 2)); }

		public short sue_rmat2_GET()//Serial UDB Extra Rmat 2
		{ return (short) ((short) get_bytes(data, 29, 2)); }

		public short sue_rmat3_GET()//Serial UDB Extra Rmat 3
		{ return (short) ((short) get_bytes(data, 31, 2)); }

		public short sue_rmat4_GET()//Serial UDB Extra Rmat 4
		{ return (short) ((short) get_bytes(data, 33, 2)); }

		public short sue_rmat5_GET()//Serial UDB Extra Rmat 5
		{ return (short) ((short) get_bytes(data, 35, 2)); }

		public short sue_rmat6_GET()//Serial UDB Extra Rmat 6
		{ return (short) ((short) get_bytes(data, 37, 2)); }

		public short sue_rmat7_GET()//Serial UDB Extra Rmat 7
		{ return (short) ((short) get_bytes(data, 39, 2)); }

		public short sue_rmat8_GET()//Serial UDB Extra Rmat 8
		{ return (short) ((short) get_bytes(data, 41, 2)); }

		public short sue_sog_GET()//Serial UDB Extra Speed Over Ground
		{ return (short) ((short) get_bytes(data, 43, 2)); }

		public short sue_estimated_wind_0_GET()//Serial UDB Extra Estimated Wind 0
		{ return (short) ((short) get_bytes(data, 45, 2)); }

		public short sue_estimated_wind_1_GET()//Serial UDB Extra Estimated Wind 1
		{ return (short) ((short) get_bytes(data, 47, 2)); }

		public short sue_estimated_wind_2_GET()//Serial UDB Extra Estimated Wind 2
		{ return (short) ((short) get_bytes(data, 49, 2)); }

		public short sue_magFieldEarth0_GET()//Serial UDB Extra Magnetic Field Earth 0
		{ return (short) ((short) get_bytes(data, 51, 2)); }

		public short sue_magFieldEarth1_GET()//Serial UDB Extra Magnetic Field Earth 1
		{ return (short) ((short) get_bytes(data, 53, 2)); }

		public short sue_magFieldEarth2_GET()//Serial UDB Extra Magnetic Field Earth 2
		{ return (short) ((short) get_bytes(data, 55, 2)); }

		public short sue_svs_GET()//Serial UDB Extra Number of Sattelites in View
		{ return (short) ((short) get_bytes(data, 57, 2)); }

		public short sue_hdop_GET()//Serial UDB Extra GPS Horizontal Dilution of Precision
		{ return (short) ((short) get_bytes(data, 59, 2)); }
	}

	public static class SERIAL_UDB_EXTRA_F2_B extends GroundControl.SERIAL_UDB_EXTRA_F2_B {
		public long sue_time_GET()//Serial UDB Extra Time
		{ return (get_bytes(data, 0, 4)); }

		public long sue_flags_GET()//Serial UDB Extra Status Flags
		{ return (get_bytes(data, 4, 4)); }

		public short sue_pwm_input_1_GET()//Serial UDB Extra PWM Input Channel 1
		{ return (short) ((short) get_bytes(data, 8, 2)); }

		public short sue_pwm_input_2_GET()//Serial UDB Extra PWM Input Channel 2
		{ return (short) ((short) get_bytes(data, 10, 2)); }

		public short sue_pwm_input_3_GET()//Serial UDB Extra PWM Input Channel 3
		{ return (short) ((short) get_bytes(data, 12, 2)); }

		public short sue_pwm_input_4_GET()//Serial UDB Extra PWM Input Channel 4
		{ return (short) ((short) get_bytes(data, 14, 2)); }

		public short sue_pwm_input_5_GET()//Serial UDB Extra PWM Input Channel 5
		{ return (short) ((short) get_bytes(data, 16, 2)); }

		public short sue_pwm_input_6_GET()//Serial UDB Extra PWM Input Channel 6
		{ return (short) ((short) get_bytes(data, 18, 2)); }

		public short sue_pwm_input_7_GET()//Serial UDB Extra PWM Input Channel 7
		{ return (short) ((short) get_bytes(data, 20, 2)); }

		public short sue_pwm_input_8_GET()//Serial UDB Extra PWM Input Channel 8
		{ return (short) ((short) get_bytes(data, 22, 2)); }

		public short sue_pwm_input_9_GET()//Serial UDB Extra PWM Input Channel 9
		{ return (short) ((short) get_bytes(data, 24, 2)); }

		public short sue_pwm_input_10_GET()//Serial UDB Extra PWM Input Channel 10
		{ return (short) ((short) get_bytes(data, 26, 2)); }

		public short sue_pwm_input_11_GET()//Serial UDB Extra PWM Input Channel 11
		{ return (short) ((short) get_bytes(data, 28, 2)); }

		public short sue_pwm_input_12_GET()//Serial UDB Extra PWM Input Channel 12
		{ return (short) ((short) get_bytes(data, 30, 2)); }

		public short sue_pwm_output_1_GET()//Serial UDB Extra PWM Output Channel 1
		{ return (short) ((short) get_bytes(data, 32, 2)); }

		public short sue_pwm_output_2_GET()//Serial UDB Extra PWM Output Channel 2
		{ return (short) ((short) get_bytes(data, 34, 2)); }

		public short sue_pwm_output_3_GET()//Serial UDB Extra PWM Output Channel 3
		{ return (short) ((short) get_bytes(data, 36, 2)); }

		public short sue_pwm_output_4_GET()//Serial UDB Extra PWM Output Channel 4
		{ return (short) ((short) get_bytes(data, 38, 2)); }

		public short sue_pwm_output_5_GET()//Serial UDB Extra PWM Output Channel 5
		{ return (short) ((short) get_bytes(data, 40, 2)); }

		public short sue_pwm_output_6_GET()//Serial UDB Extra PWM Output Channel 6
		{ return (short) ((short) get_bytes(data, 42, 2)); }

		public short sue_pwm_output_7_GET()//Serial UDB Extra PWM Output Channel 7
		{ return (short) ((short) get_bytes(data, 44, 2)); }

		public short sue_pwm_output_8_GET()//Serial UDB Extra PWM Output Channel 8
		{ return (short) ((short) get_bytes(data, 46, 2)); }

		public short sue_pwm_output_9_GET()//Serial UDB Extra PWM Output Channel 9
		{ return (short) ((short) get_bytes(data, 48, 2)); }

		public short sue_pwm_output_10_GET()//Serial UDB Extra PWM Output Channel 10
		{ return (short) ((short) get_bytes(data, 50, 2)); }

		public short sue_pwm_output_11_GET()//Serial UDB Extra PWM Output Channel 11
		{ return (short) ((short) get_bytes(data, 52, 2)); }

		public short sue_pwm_output_12_GET()//Serial UDB Extra PWM Output Channel 12
		{ return (short) ((short) get_bytes(data, 54, 2)); }

		public short sue_imu_location_x_GET()//Serial UDB Extra IMU Location X
		{ return (short) ((short) get_bytes(data, 56, 2)); }

		public short sue_imu_location_y_GET()//Serial UDB Extra IMU Location Y
		{ return (short) ((short) get_bytes(data, 58, 2)); }

		public short sue_imu_location_z_GET()//Serial UDB Extra IMU Location Z
		{ return (short) ((short) get_bytes(data, 60, 2)); }

		public short sue_location_error_earth_x_GET()//Serial UDB Location Error Earth X
		{ return (short) ((short) get_bytes(data, 62, 2)); }

		public short sue_location_error_earth_y_GET()//Serial UDB Location Error Earth Y
		{ return (short) ((short) get_bytes(data, 64, 2)); }

		public short sue_location_error_earth_z_GET()//Serial UDB Location Error Earth Z
		{ return (short) ((short) get_bytes(data, 66, 2)); }

		public short sue_osc_fails_GET()//Serial UDB Extra Oscillator Failure Count
		{ return (short) ((short) get_bytes(data, 68, 2)); }

		public short sue_imu_velocity_x_GET()//Serial UDB Extra IMU Velocity X
		{ return (short) ((short) get_bytes(data, 70, 2)); }

		public short sue_imu_velocity_y_GET()//Serial UDB Extra IMU Velocity Y
		{ return (short) ((short) get_bytes(data, 72, 2)); }

		public short sue_imu_velocity_z_GET()//Serial UDB Extra IMU Velocity Z
		{ return (short) ((short) get_bytes(data, 74, 2)); }

		public short sue_waypoint_goal_x_GET()//Serial UDB Extra Current Waypoint Goal X
		{ return (short) ((short) get_bytes(data, 76, 2)); }

		public short sue_waypoint_goal_y_GET()//Serial UDB Extra Current Waypoint Goal Y
		{ return (short) ((short) get_bytes(data, 78, 2)); }

		public short sue_waypoint_goal_z_GET()//Serial UDB Extra Current Waypoint Goal Z
		{ return (short) ((short) get_bytes(data, 80, 2)); }

		public short sue_aero_x_GET()//Aeroforce in UDB X Axis
		{ return (short) ((short) get_bytes(data, 82, 2)); }

		public short sue_aero_y_GET()//Aeroforce in UDB Y Axis
		{ return (short) ((short) get_bytes(data, 84, 2)); }

		public short sue_aero_z_GET()//Aeroforce in UDB Z axis
		{ return (short) ((short) get_bytes(data, 86, 2)); }

		public short sue_barom_temp_GET()//SUE barometer temperature
		{ return (short) ((short) get_bytes(data, 88, 2)); }

		public int sue_barom_press_GET()//SUE barometer pressure
		{ return (int) ((int) get_bytes(data, 90, 4)); }

		public int sue_barom_alt_GET()//SUE barometer altitude
		{ return (int) ((int) get_bytes(data, 94, 4)); }

		public short sue_bat_volt_GET()//SUE battery voltage
		{ return (short) ((short) get_bytes(data, 98, 2)); }

		public short sue_bat_amp_GET()//SUE battery current
		{ return (short) ((short) get_bytes(data, 100, 2)); }

		public short sue_bat_amp_hours_GET()//SUE battery milli amp hours used
		{ return (short) ((short) get_bytes(data, 102, 2)); }

		public short sue_desired_height_GET()//Sue autopilot desired height
		{ return (short) ((short) get_bytes(data, 104, 2)); }

		public short sue_memory_stack_free_GET()//Serial UDB Extra Stack Memory Free
		{ return (short) ((short) get_bytes(data, 106, 2)); }
	}

	public static class SERIAL_UDB_EXTRA_F4 extends GroundControl.SERIAL_UDB_EXTRA_F4 {
		public char sue_ROLL_STABILIZATION_AILERONS_GET()//Serial UDB Extra Roll Stabilization with Ailerons Enabled
		{ return (char) ((char) get_bytes(data, 0, 1)); }

		public char sue_ROLL_STABILIZATION_RUDDER_GET()//Serial UDB Extra Roll Stabilization with Rudder Enabled
		{ return (char) ((char) get_bytes(data, 1, 1)); }

		public char sue_PITCH_STABILIZATION_GET()//Serial UDB Extra Pitch Stabilization Enabled
		{ return (char) ((char) get_bytes(data, 2, 1)); }

		public char sue_YAW_STABILIZATION_RUDDER_GET()//Serial UDB Extra Yaw Stabilization using Rudder Enabled
		{ return (char) ((char) get_bytes(data, 3, 1)); }

		public char sue_YAW_STABILIZATION_AILERON_GET()//Serial UDB Extra Yaw Stabilization using Ailerons Enabled
		{ return (char) ((char) get_bytes(data, 4, 1)); }

		public char sue_AILERON_NAVIGATION_GET()//Serial UDB Extra Navigation with Ailerons Enabled
		{ return (char) ((char) get_bytes(data, 5, 1)); }

		public char sue_RUDDER_NAVIGATION_GET()//Serial UDB Extra Navigation with Rudder Enabled
		{ return (char) ((char) get_bytes(data, 6, 1)); }

		public char sue_ALTITUDEHOLD_STABILIZED_GET()//Serial UDB Extra Type of Alitude Hold when in Stabilized Mode
		{ return (char) ((char) get_bytes(data, 7, 1)); }

		public char sue_ALTITUDEHOLD_WAYPOINT_GET()//Serial UDB Extra Type of Alitude Hold when in Waypoint Mode
		{ return (char) ((char) get_bytes(data, 8, 1)); }

		public char sue_RACING_MODE_GET()//Serial UDB Extra Firmware racing mode enabled
		{ return (char) ((char) get_bytes(data, 9, 1)); }
	}

	public static class SERIAL_UDB_EXTRA_F5 extends GroundControl.SERIAL_UDB_EXTRA_F5 {
		public float sue_YAWKP_AILERON_GET()//Serial UDB YAWKP_AILERON Gain for Proporional control of navigation
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 0, 4))); }

		public float sue_YAWKD_AILERON_GET()//Serial UDB YAWKD_AILERON Gain for Rate control of navigation
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 4, 4))); }

		public float sue_ROLLKP_GET()//Serial UDB Extra ROLLKP Gain for Proportional control of roll stabilization
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 8, 4))); }

		public float sue_ROLLKD_GET()//Serial UDB Extra ROLLKD Gain for Rate control of roll stabilization
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 12, 4))); }
	}

	public static class SERIAL_UDB_EXTRA_F6 extends GroundControl.SERIAL_UDB_EXTRA_F6 {
		public float sue_PITCHGAIN_GET()//Serial UDB Extra PITCHGAIN Proportional Control
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 0, 4))); }

		public float sue_PITCHKD_GET()//Serial UDB Extra Pitch Rate Control
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 4, 4))); }

		public float sue_RUDDER_ELEV_MIX_GET()//Serial UDB Extra Rudder to Elevator Mix
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 8, 4))); }

		public float sue_ROLL_ELEV_MIX_GET()//Serial UDB Extra Roll to Elevator Mix
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 12, 4))); }

		public float sue_ELEVATOR_BOOST_GET()//Gain For Boosting Manual Elevator control When Plane Stabilized
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 16, 4))); }
	}

	public static class SERIAL_UDB_EXTRA_F7 extends GroundControl.SERIAL_UDB_EXTRA_F7 {
		public float sue_YAWKP_RUDDER_GET()//Serial UDB YAWKP_RUDDER Gain for Proporional control of navigation
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 0, 4))); }

		public float sue_YAWKD_RUDDER_GET()//Serial UDB YAWKD_RUDDER Gain for Rate control of navigation
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 4, 4))); }

		public float sue_ROLLKP_RUDDER_GET()//Serial UDB Extra ROLLKP_RUDDER Gain for Proportional control of roll stabilization
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 8, 4))); }

		public float sue_ROLLKD_RUDDER_GET()//Serial UDB Extra ROLLKD_RUDDER Gain for Rate control of roll stabilization
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 12, 4))); }

		public float sue_RUDDER_BOOST_GET()//SERIAL UDB EXTRA Rudder Boost Gain to Manual Control when stabilized
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 16, 4))); }

		public float sue_RTL_PITCH_DOWN_GET()//Serial UDB Extra Return To Landing - Angle to Pitch Plane Down
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 20, 4))); }
	}

	public static class SERIAL_UDB_EXTRA_F8 extends GroundControl.SERIAL_UDB_EXTRA_F8 {
		public float sue_HEIGHT_TARGET_MAX_GET()//Serial UDB Extra HEIGHT_TARGET_MAX
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 0, 4))); }

		public float sue_HEIGHT_TARGET_MIN_GET()//Serial UDB Extra HEIGHT_TARGET_MIN
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 4, 4))); }

		public float sue_ALT_HOLD_THROTTLE_MIN_GET()//Serial UDB Extra ALT_HOLD_THROTTLE_MIN
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 8, 4))); }

		public float sue_ALT_HOLD_THROTTLE_MAX_GET()//Serial UDB Extra ALT_HOLD_THROTTLE_MAX
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 12, 4))); }

		public float sue_ALT_HOLD_PITCH_MIN_GET()//Serial UDB Extra ALT_HOLD_PITCH_MIN
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 16, 4))); }

		public float sue_ALT_HOLD_PITCH_MAX_GET()//Serial UDB Extra ALT_HOLD_PITCH_MAX
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 20, 4))); }

		public float sue_ALT_HOLD_PITCH_HIGH_GET()//Serial UDB Extra ALT_HOLD_PITCH_HIGH
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 24, 4))); }
	}

	public static class SERIAL_UDB_EXTRA_F13 extends GroundControl.SERIAL_UDB_EXTRA_F13 {
		public short sue_week_no_GET()//Serial UDB Extra GPS Week Number
		{ return (short) ((short) get_bytes(data, 0, 2)); }

		public int sue_lat_origin_GET()//Serial UDB Extra MP Origin Latitude
		{ return (int) ((int) get_bytes(data, 2, 4)); }

		public int sue_lon_origin_GET()//Serial UDB Extra MP Origin Longitude
		{ return (int) ((int) get_bytes(data, 6, 4)); }

		public int sue_alt_origin_GET()//Serial UDB Extra MP Origin Altitude Above Sea Level
		{ return (int) ((int) get_bytes(data, 10, 4)); }
	}

	public static class SERIAL_UDB_EXTRA_F14 extends GroundControl.SERIAL_UDB_EXTRA_F14 {
		public long sue_TRAP_SOURCE_GET()//Serial UDB Extra Type Program Address of Last Trap
		{ return (get_bytes(data, 0, 4)); }

		public char sue_WIND_ESTIMATION_GET()//Serial UDB Extra Wind Estimation Enabled
		{ return (char) ((char) get_bytes(data, 4, 1)); }

		public char sue_GPS_TYPE_GET()//Serial UDB Extra Type of GPS Unit
		{ return (char) ((char) get_bytes(data, 5, 1)); }

		public char sue_DR_GET()//Serial UDB Extra Dead Reckoning Enabled
		{ return (char) ((char) get_bytes(data, 6, 1)); }

		public char sue_BOARD_TYPE_GET()//Serial UDB Extra Type of UDB Hardware
		{ return (char) ((char) get_bytes(data, 7, 1)); }

		public char sue_AIRFRAME_GET()//Serial UDB Extra Type of Airframe
		{ return (char) ((char) get_bytes(data, 8, 1)); }

		public short sue_RCON_GET()//Serial UDB Extra Reboot Register of DSPIC
		{ return (short) ((short) get_bytes(data, 9, 2)); }

		public short sue_TRAP_FLAGS_GET()//Serial UDB Extra  Last dspic Trap Flags
		{ return (short) ((short) get_bytes(data, 11, 2)); }

		public short sue_osc_fail_count_GET()//Serial UDB Extra Number of Ocillator Failures
		{ return (short) ((short) get_bytes(data, 13, 2)); }

		public char sue_CLOCK_CONFIG_GET()//Serial UDB Extra UDB Internal Clock Configuration
		{ return (char) ((char) get_bytes(data, 15, 1)); }

		public char sue_FLIGHT_PLAN_TYPE_GET()//Serial UDB Extra Type of Flight Plan
		{ return (char) ((char) get_bytes(data, 16, 1)); }
	}

	public static class SERIAL_UDB_EXTRA_F15 extends GroundControl.SERIAL_UDB_EXTRA_F15 {
		public char[] sue_ID_VEHICLE_MODEL_NAME_GET(char[] dst_ch, int pos)  //Serial UDB Extra Model Name Of Vehicle
		{
			for (int BYTE = 0, dst_max = pos + 40; pos < dst_max; pos++, BYTE += 1)
				dst_ch[pos] = (char) ((char) get_bytes(data, BYTE, 1));
			return dst_ch;
		}

		public char[] sue_ID_VEHICLE_MODEL_NAME_GET()//Serial UDB Extra Model Name Of Vehicle
		{return sue_ID_VEHICLE_MODEL_NAME_GET(new char[40], 0);}

		public char[] sue_ID_VEHICLE_REGISTRATION_GET(char[] dst_ch, int pos)  //Serial UDB Extra Registraton Number of Vehicle
		{
			for (int BYTE = 40, dst_max = pos + 20; pos < dst_max; pos++, BYTE += 1)
				dst_ch[pos] = (char) ((char) get_bytes(data, BYTE, 1));
			return dst_ch;
		}

		public char[] sue_ID_VEHICLE_REGISTRATION_GET()//Serial UDB Extra Registraton Number of Vehicle
		{return sue_ID_VEHICLE_REGISTRATION_GET(new char[20], 0);}
	}

	public static class SERIAL_UDB_EXTRA_F16 extends GroundControl.SERIAL_UDB_EXTRA_F16 {
		public char[] sue_ID_LEAD_PILOT_GET(char[] dst_ch, int pos)  //Serial UDB Extra Name of Expected Lead Pilot
		{
			for (int BYTE = 0, dst_max = pos + 40; pos < dst_max; pos++, BYTE += 1)
				dst_ch[pos] = (char) ((char) get_bytes(data, BYTE, 1));
			return dst_ch;
		}

		public char[] sue_ID_LEAD_PILOT_GET()//Serial UDB Extra Name of Expected Lead Pilot
		{return sue_ID_LEAD_PILOT_GET(new char[40], 0);}

		public char[] sue_ID_DIY_DRONES_URL_GET(char[] dst_ch, int pos)  //Serial UDB Extra URL of Lead Pilot or Team
		{
			for (int BYTE = 40, dst_max = pos + 70; pos < dst_max; pos++, BYTE += 1)
				dst_ch[pos] = (char) ((char) get_bytes(data, BYTE, 1));
			return dst_ch;
		}

		public char[] sue_ID_DIY_DRONES_URL_GET()//Serial UDB Extra URL of Lead Pilot or Team
		{return sue_ID_DIY_DRONES_URL_GET(new char[70], 0);}
	}

	public static class ALTITUDES extends GroundControl.ALTITUDES {
		public long time_boot_ms_GET()//Timestamp (milliseconds since system boot)
		{ return (get_bytes(data, 0, 4)); }

		public int alt_gps_GET()//GPS altitude in meters, expressed as * 1000 (millimeters), above MSL
		{ return (int) ((int) get_bytes(data, 4, 4)); }

		public int alt_imu_GET()//IMU altitude above ground in meters, expressed as * 1000 (millimeters)
		{ return (int) ((int) get_bytes(data, 8, 4)); }

		public int alt_barometric_GET()//barometeric altitude above ground in meters, expressed as * 1000 (millimeters)
		{ return (int) ((int) get_bytes(data, 12, 4)); }

		public int alt_optical_flow_GET()//Optical flow altitude above ground in meters, expressed as * 1000 (millimeters)
		{ return (int) ((int) get_bytes(data, 16, 4)); }

		public int alt_range_finder_GET()//Rangefinder Altitude above ground in meters, expressed as * 1000 (millimeters)
		{ return (int) ((int) get_bytes(data, 20, 4)); }

		public int alt_extra_GET()//Extra altitude above ground in meters, expressed as * 1000 (millimeters)
		{ return (int) ((int) get_bytes(data, 24, 4)); }
	}

	public static class AIRSPEEDS extends GroundControl.AIRSPEEDS {
		public long time_boot_ms_GET()//Timestamp (milliseconds since system boot)
		{ return (get_bytes(data, 0, 4)); }

		public short airspeed_imu_GET()//Airspeed estimate from IMU, cm/s
		{ return (short) ((short) get_bytes(data, 4, 2)); }

		public short airspeed_pitot_GET()//Pitot measured forward airpseed, cm/s
		{ return (short) ((short) get_bytes(data, 6, 2)); }

		public short airspeed_hot_wire_GET()//Hot wire anenometer measured airspeed, cm/s
		{ return (short) ((short) get_bytes(data, 8, 2)); }

		public short airspeed_ultrasonic_GET()//Ultrasonic measured airspeed, cm/s
		{ return (short) ((short) get_bytes(data, 10, 2)); }

		public short aoa_GET()//Angle of attack sensor, degrees * 10
		{ return (short) ((short) get_bytes(data, 12, 2)); }

		public short aoy_GET()//Yaw angle sensor, degrees * 10
		{ return (short) ((short) get_bytes(data, 14, 2)); }
	}

	public static class SERIAL_UDB_EXTRA_F17 extends GroundControl.SERIAL_UDB_EXTRA_F17 {
		public float sue_feed_forward_GET()//SUE Feed Forward Gain
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 0, 4))); }

		public float sue_turn_rate_nav_GET()//SUE Max Turn Rate when Navigating
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 4, 4))); }

		public float sue_turn_rate_fbw_GET()//SUE Max Turn Rate in Fly By Wire Mode
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 8, 4))); }
	}

	public static class SERIAL_UDB_EXTRA_F18 extends GroundControl.SERIAL_UDB_EXTRA_F18 {
		public float angle_of_attack_normal_GET()//SUE Angle of Attack Normal
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 0, 4))); }

		public float angle_of_attack_inverted_GET()//SUE Angle of Attack Inverted
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 4, 4))); }

		public float elevator_trim_normal_GET()//SUE Elevator Trim Normal
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 8, 4))); }

		public float elevator_trim_inverted_GET()//SUE Elevator Trim Inverted
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 12, 4))); }

		public float reference_speed_GET()//SUE reference_speed
		{ return (float) (Float.intBitsToFloat((int) get_bytes(data, 16, 4))); }
	}

	public static class SERIAL_UDB_EXTRA_F19 extends GroundControl.SERIAL_UDB_EXTRA_F19 {
		public char sue_aileron_output_channel_GET()//SUE aileron output channel
		{ return (char) ((char) get_bytes(data, 0, 1)); }

		public char sue_aileron_reversed_GET()//SUE aileron reversed
		{ return (char) ((char) get_bytes(data, 1, 1)); }

		public char sue_elevator_output_channel_GET()//SUE elevator output channel
		{ return (char) ((char) get_bytes(data, 2, 1)); }

		public char sue_elevator_reversed_GET()//SUE elevator reversed
		{ return (char) ((char) get_bytes(data, 3, 1)); }

		public char sue_throttle_output_channel_GET()//SUE throttle output channel
		{ return (char) ((char) get_bytes(data, 4, 1)); }

		public char sue_throttle_reversed_GET()//SUE throttle reversed
		{ return (char) ((char) get_bytes(data, 5, 1)); }

		public char sue_rudder_output_channel_GET()//SUE rudder output channel
		{ return (char) ((char) get_bytes(data, 6, 1)); }

		public char sue_rudder_reversed_GET()//SUE rudder reversed
		{ return (char) ((char) get_bytes(data, 7, 1)); }
	}

	public static class SERIAL_UDB_EXTRA_F20 extends GroundControl.SERIAL_UDB_EXTRA_F20 {
		public char sue_number_of_inputs_GET()//SUE Number of Input Channels
		{ return (char) ((char) get_bytes(data, 0, 1)); }

		public short sue_trim_value_input_1_GET()//SUE UDB PWM Trim Value on Input 1
		{ return (short) ((short) get_bytes(data, 1, 2)); }

		public short sue_trim_value_input_2_GET()//SUE UDB PWM Trim Value on Input 2
		{ return (short) ((short) get_bytes(data, 3, 2)); }

		public short sue_trim_value_input_3_GET()//SUE UDB PWM Trim Value on Input 3
		{ return (short) ((short) get_bytes(data, 5, 2)); }

		public short sue_trim_value_input_4_GET()//SUE UDB PWM Trim Value on Input 4
		{ return (short) ((short) get_bytes(data, 7, 2)); }

		public short sue_trim_value_input_5_GET()//SUE UDB PWM Trim Value on Input 5
		{ return (short) ((short) get_bytes(data, 9, 2)); }

		public short sue_trim_value_input_6_GET()//SUE UDB PWM Trim Value on Input 6
		{ return (short) ((short) get_bytes(data, 11, 2)); }

		public short sue_trim_value_input_7_GET()//SUE UDB PWM Trim Value on Input 7
		{ return (short) ((short) get_bytes(data, 13, 2)); }

		public short sue_trim_value_input_8_GET()//SUE UDB PWM Trim Value on Input 8
		{ return (short) ((short) get_bytes(data, 15, 2)); }

		public short sue_trim_value_input_9_GET()//SUE UDB PWM Trim Value on Input 9
		{ return (short) ((short) get_bytes(data, 17, 2)); }

		public short sue_trim_value_input_10_GET()//SUE UDB PWM Trim Value on Input 10
		{ return (short) ((short) get_bytes(data, 19, 2)); }

		public short sue_trim_value_input_11_GET()//SUE UDB PWM Trim Value on Input 11
		{ return (short) ((short) get_bytes(data, 21, 2)); }

		public short sue_trim_value_input_12_GET()//SUE UDB PWM Trim Value on Input 12
		{ return (short) ((short) get_bytes(data, 23, 2)); }
	}

	public static class SERIAL_UDB_EXTRA_F21 extends GroundControl.SERIAL_UDB_EXTRA_F21 {
		public short sue_accel_x_offset_GET()//SUE X accelerometer offset
		{ return (short) ((short) get_bytes(data, 0, 2)); }

		public short sue_accel_y_offset_GET()//SUE Y accelerometer offset
		{ return (short) ((short) get_bytes(data, 2, 2)); }

		public short sue_accel_z_offset_GET()//SUE Z accelerometer offset
		{ return (short) ((short) get_bytes(data, 4, 2)); }

		public short sue_gyro_x_offset_GET()//SUE X gyro offset
		{ return (short) ((short) get_bytes(data, 6, 2)); }

		public short sue_gyro_y_offset_GET()//SUE Y gyro offset
		{ return (short) ((short) get_bytes(data, 8, 2)); }

		public short sue_gyro_z_offset_GET()//SUE Z gyro offset
		{ return (short) ((short) get_bytes(data, 10, 2)); }
	}

	public static class SERIAL_UDB_EXTRA_F22 extends GroundControl.SERIAL_UDB_EXTRA_F22 {
		public short sue_accel_x_at_calibration_GET()//SUE X accelerometer at calibration time
		{ return (short) ((short) get_bytes(data, 0, 2)); }

		public short sue_accel_y_at_calibration_GET()//SUE Y accelerometer at calibration time
		{ return (short) ((short) get_bytes(data, 2, 2)); }

		public short sue_accel_z_at_calibration_GET()//SUE Z accelerometer at calibration time
		{ return (short) ((short) get_bytes(data, 4, 2)); }

		public short sue_gyro_x_at_calibration_GET()//SUE X gyro at calibration time
		{ return (short) ((short) get_bytes(data, 6, 2)); }

		public short sue_gyro_y_at_calibration_GET()//SUE Y gyro at calibration time
		{ return (short) ((short) get_bytes(data, 8, 2)); }

		public short sue_gyro_z_at_calibration_GET()//SUE Z gyro at calibration time
		{ return (short) ((short) get_bytes(data, 10, 2)); }
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

		static final Collection<OnReceive.Handler<FLEXIFUNCTION_SET, Channel>>                 on_FLEXIFUNCTION_SET                 = new OnReceive<>();
		static final Collection<OnReceive.Handler<FLEXIFUNCTION_READ_REQ, Channel>>            on_FLEXIFUNCTION_READ_REQ            = new OnReceive<>();
		static final Collection<OnReceive.Handler<FLEXIFUNCTION_BUFFER_FUNCTION, Channel>>     on_FLEXIFUNCTION_BUFFER_FUNCTION     = new OnReceive<>();
		static final Collection<OnReceive.Handler<FLEXIFUNCTION_BUFFER_FUNCTION_ACK, Channel>> on_FLEXIFUNCTION_BUFFER_FUNCTION_ACK = new OnReceive<>();
		static final Collection<OnReceive.Handler<FLEXIFUNCTION_DIRECTORY, Channel>>           on_FLEXIFUNCTION_DIRECTORY           = new OnReceive<>();
		static final Collection<OnReceive.Handler<FLEXIFUNCTION_DIRECTORY_ACK, Channel>>       on_FLEXIFUNCTION_DIRECTORY_ACK       = new OnReceive<>();
		static final Collection<OnReceive.Handler<FLEXIFUNCTION_COMMAND, Channel>>             on_FLEXIFUNCTION_COMMAND             = new OnReceive<>();
		static final Collection<OnReceive.Handler<FLEXIFUNCTION_COMMAND_ACK, Channel>>         on_FLEXIFUNCTION_COMMAND_ACK         = new OnReceive<>();
		static final Collection<OnReceive.Handler<SERIAL_UDB_EXTRA_F2_A, Channel>>             on_SERIAL_UDB_EXTRA_F2_A             = new OnReceive<>();
		static final Collection<OnReceive.Handler<SERIAL_UDB_EXTRA_F2_B, Channel>>             on_SERIAL_UDB_EXTRA_F2_B             = new OnReceive<>();
		static final Collection<OnReceive.Handler<SERIAL_UDB_EXTRA_F4, Channel>>               on_SERIAL_UDB_EXTRA_F4               = new OnReceive<>();
		static final Collection<OnReceive.Handler<SERIAL_UDB_EXTRA_F5, Channel>>               on_SERIAL_UDB_EXTRA_F5               = new OnReceive<>();
		static final Collection<OnReceive.Handler<SERIAL_UDB_EXTRA_F6, Channel>>               on_SERIAL_UDB_EXTRA_F6               = new OnReceive<>();
		static final Collection<OnReceive.Handler<SERIAL_UDB_EXTRA_F7, Channel>>               on_SERIAL_UDB_EXTRA_F7               = new OnReceive<>();
		static final Collection<OnReceive.Handler<SERIAL_UDB_EXTRA_F8, Channel>>               on_SERIAL_UDB_EXTRA_F8               = new OnReceive<>();
		static final Collection<OnReceive.Handler<SERIAL_UDB_EXTRA_F13, Channel>>              on_SERIAL_UDB_EXTRA_F13              = new OnReceive<>();
		static final Collection<OnReceive.Handler<SERIAL_UDB_EXTRA_F14, Channel>>              on_SERIAL_UDB_EXTRA_F14              = new OnReceive<>();
		static final Collection<OnReceive.Handler<SERIAL_UDB_EXTRA_F15, Channel>>              on_SERIAL_UDB_EXTRA_F15              = new OnReceive<>();
		static final Collection<OnReceive.Handler<SERIAL_UDB_EXTRA_F16, Channel>>              on_SERIAL_UDB_EXTRA_F16              = new OnReceive<>();
		static final Collection<OnReceive.Handler<ALTITUDES, Channel>>                         on_ALTITUDES                         = new OnReceive<>();
		static final Collection<OnReceive.Handler<AIRSPEEDS, Channel>>                         on_AIRSPEEDS                         = new OnReceive<>();
		static final Collection<OnReceive.Handler<SERIAL_UDB_EXTRA_F17, Channel>>              on_SERIAL_UDB_EXTRA_F17              = new OnReceive<>();
		static final Collection<OnReceive.Handler<SERIAL_UDB_EXTRA_F18, Channel>>              on_SERIAL_UDB_EXTRA_F18              = new OnReceive<>();
		static final Collection<OnReceive.Handler<SERIAL_UDB_EXTRA_F19, Channel>>              on_SERIAL_UDB_EXTRA_F19              = new OnReceive<>();
		static final Collection<OnReceive.Handler<SERIAL_UDB_EXTRA_F20, Channel>>              on_SERIAL_UDB_EXTRA_F20              = new OnReceive<>();
		static final Collection<OnReceive.Handler<SERIAL_UDB_EXTRA_F21, Channel>>              on_SERIAL_UDB_EXTRA_F21              = new OnReceive<>();
		static final Collection<OnReceive.Handler<SERIAL_UDB_EXTRA_F22, Channel>>              on_SERIAL_UDB_EXTRA_F22              = new OnReceive<>();
		static final Collection<OnReceive.Handler<SET_HOME_POSITION, Channel>>                 on_SET_HOME_POSITION                 = new OnReceive<>();
		static final Collection<OnReceive.Handler<MESSAGE_INTERVAL, Channel>>                  on_MESSAGE_INTERVAL                  = new OnReceive<>();
		static final Collection<OnReceive.Handler<EXTENDED_SYS_STATE, Channel>>                on_EXTENDED_SYS_STATE                = new OnReceive<>();
		static final Collection<OnReceive.Handler<ADSB_VEHICLE, Channel>>                      on_ADSB_VEHICLE                      = new OnReceive<>();
		static final Collection<OnReceive.Handler<COLLISION, Channel>>                         on_COLLISION                         = new OnReceive<>();
		static final Collection<OnReceive.Handler<V2_EXTENSION, Channel>>                      on_V2_EXTENSION                      = new OnReceive<>();
		static final Collection<OnReceive.Handler<MEMORY_VECT, Channel>>                       on_MEMORY_VECT                       = new OnReceive<>();
		static final Collection<OnReceive.Handler<DEBUG_VECT, Channel>>                        on_DEBUG_VECT                        = new OnReceive<>();
		static final Collection<OnReceive.Handler<NAMED_VALUE_FLOAT, Channel>>                 on_NAMED_VALUE_FLOAT                 = new OnReceive<>();
		static final Collection<OnReceive.Handler<NAMED_VALUE_INT, Channel>>                   on_NAMED_VALUE_INT                   = new OnReceive<>();
		static final Collection<OnReceive.Handler<STATUSTEXT, Channel>>                        on_STATUSTEXT                        = new OnReceive<>();
		static final Collection<OnReceive.Handler<DEBUG, Channel>>                             on_DEBUG                             = new OnReceive<>();
		static final Collection<OnReceive.Handler<SETUP_SIGNING, Channel>>                     on_SETUP_SIGNING                     = new OnReceive<>();
		static final Collection<OnReceive.Handler<BUTTON_CHANGE, Channel>>                     on_BUTTON_CHANGE                     = new OnReceive<>();
		static final Collection<OnReceive.Handler<PLAY_TUNE, Channel>>                         on_PLAY_TUNE                         = new OnReceive<>();
		static final Collection<OnReceive.Handler<CAMERA_INFORMATION, Channel>>                on_CAMERA_INFORMATION                = new OnReceive<>();
		static final Collection<OnReceive.Handler<CAMERA_SETTINGS, Channel>>                   on_CAMERA_SETTINGS                   = new OnReceive<>();
		static final Collection<OnReceive.Handler<STORAGE_INFORMATION, Channel>>               on_STORAGE_INFORMATION               = new OnReceive<>();
		static final Collection<OnReceive.Handler<CAMERA_CAPTURE_STATUS, Channel>>             on_CAMERA_CAPTURE_STATUS             = new OnReceive<>();
		static final Collection<OnReceive.Handler<CAMERA_IMAGE_CAPTURED, Channel>>             on_CAMERA_IMAGE_CAPTURED             = new OnReceive<>();
		static final Collection<OnReceive.Handler<FLIGHT_INFORMATION, Channel>>                on_FLIGHT_INFORMATION                = new OnReceive<>();
		static final Collection<OnReceive.Handler<MOUNT_ORIENTATION, Channel>>                 on_MOUNT_ORIENTATION                 = new OnReceive<>();
		static final Collection<OnReceive.Handler<LOGGING_DATA, Channel>>                      on_LOGGING_DATA                      = new OnReceive<>();
		static final Collection<OnReceive.Handler<LOGGING_DATA_ACKED, Channel>>                on_LOGGING_DATA_ACKED                = new OnReceive<>();
		static final Collection<OnReceive.Handler<LOGGING_ACK, Channel>>                       on_LOGGING_ACK                       = new OnReceive<>();
		static final Collection<OnReceive.Handler<VIDEO_STREAM_INFORMATION, Channel>>          on_VIDEO_STREAM_INFORMATION          = new OnReceive<>();
		static final Collection<OnReceive.Handler<SET_VIDEO_STREAM_SETTINGS, Channel>>         on_SET_VIDEO_STREAM_SETTINGS         = new OnReceive<>();
		static final Collection<OnReceive.Handler<WIFI_CONFIG_AP, Channel>>                    on_WIFI_CONFIG_AP                    = new OnReceive<>();
		static final Collection<OnReceive.Handler<PROTOCOL_VERSION, Channel>>                  on_PROTOCOL_VERSION                  = new OnReceive<>();
		static final Collection<OnReceive.Handler<UAVCAN_NODE_STATUS, Channel>>                on_UAVCAN_NODE_STATUS                = new OnReceive<>();
		static final Collection<OnReceive.Handler<UAVCAN_NODE_INFO, Channel>>                  on_UAVCAN_NODE_INFO                  = new OnReceive<>();
		static final Collection<OnReceive.Handler<PARAM_EXT_REQUEST_READ, Channel>>            on_PARAM_EXT_REQUEST_READ            = new OnReceive<>();
		static final Collection<OnReceive.Handler<PARAM_EXT_REQUEST_LIST, Channel>>            on_PARAM_EXT_REQUEST_LIST            = new OnReceive<>();
		static final Collection<OnReceive.Handler<PARAM_EXT_VALUE, Channel>>                   on_PARAM_EXT_VALUE                   = new OnReceive<>();
		static final Collection<OnReceive.Handler<PARAM_EXT_SET, Channel>>                     on_PARAM_EXT_SET                     = new OnReceive<>();
		static final Collection<OnReceive.Handler<PARAM_EXT_ACK, Channel>>                     on_PARAM_EXT_ACK                     = new OnReceive<>();
		static final Collection<OnReceive.Handler<OBSTACLE_DISTANCE, Channel>>                 on_OBSTACLE_DISTANCE                 = new OnReceive<>();


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
				case 3:
					if (pack == null) return new POSITION_TARGET_LOCAL_NED();
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
				case 81:
					if (pack == null) return new MANUAL_SETPOINT();
					break;
				case 82:
					if (pack == null) return new SET_ATTITUDE_TARGET();
					break;
				case 83:
					if (pack == null) return new ATTITUDE_TARGET();
					break;
				case 84:
					if (pack == null) return new SET_POSITION_TARGET_LOCAL_NED();
					break;
				case 150:
					if (pack == null) return new FLEXIFUNCTION_SET();
					((OnReceive) on_FLEXIFUNCTION_SET).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
					break;
				case 151:
					if (pack == null) return new FLEXIFUNCTION_READ_REQ();
					((OnReceive) on_FLEXIFUNCTION_READ_REQ).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
					break;
				case 152:
					if (pack == null) return new FLEXIFUNCTION_BUFFER_FUNCTION();
					((OnReceive) on_FLEXIFUNCTION_BUFFER_FUNCTION).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
					break;
				case 153:
					if (pack == null) return new FLEXIFUNCTION_BUFFER_FUNCTION_ACK();
					((OnReceive) on_FLEXIFUNCTION_BUFFER_FUNCTION_ACK).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
					break;
				case 155:
					if (pack == null) return new FLEXIFUNCTION_DIRECTORY();
					((OnReceive) on_FLEXIFUNCTION_DIRECTORY).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
					break;
				case 156:
					if (pack == null) return new FLEXIFUNCTION_DIRECTORY_ACK();
					((OnReceive) on_FLEXIFUNCTION_DIRECTORY_ACK).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
					break;
				case 157:
					if (pack == null) return new FLEXIFUNCTION_COMMAND();
					((OnReceive) on_FLEXIFUNCTION_COMMAND).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
					break;
				case 158:
					if (pack == null) return new FLEXIFUNCTION_COMMAND_ACK();
					((OnReceive) on_FLEXIFUNCTION_COMMAND_ACK).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
					break;
				case 170:
					if (pack == null) return new SERIAL_UDB_EXTRA_F2_A();
					((OnReceive) on_SERIAL_UDB_EXTRA_F2_A).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
					break;
				case 171:
					if (pack == null) return new SERIAL_UDB_EXTRA_F2_B();
					((OnReceive) on_SERIAL_UDB_EXTRA_F2_B).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
					break;
				case 172:
					if (pack == null) return new SERIAL_UDB_EXTRA_F4();
					((OnReceive) on_SERIAL_UDB_EXTRA_F4).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
					break;
				case 173:
					if (pack == null) return new SERIAL_UDB_EXTRA_F5();
					((OnReceive) on_SERIAL_UDB_EXTRA_F5).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
					break;
				case 174:
					if (pack == null) return new SERIAL_UDB_EXTRA_F6();
					((OnReceive) on_SERIAL_UDB_EXTRA_F6).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
					break;
				case 175:
					if (pack == null) return new SERIAL_UDB_EXTRA_F7();
					((OnReceive) on_SERIAL_UDB_EXTRA_F7).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
					break;
				case 176:
					if (pack == null) return new SERIAL_UDB_EXTRA_F8();
					((OnReceive) on_SERIAL_UDB_EXTRA_F8).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
					break;
				case 177:
					if (pack == null) return new SERIAL_UDB_EXTRA_F13();
					((OnReceive) on_SERIAL_UDB_EXTRA_F13).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
					break;
				case 178:
					if (pack == null) return new SERIAL_UDB_EXTRA_F14();
					((OnReceive) on_SERIAL_UDB_EXTRA_F14).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
					break;
				case 179:
					if (pack == null) return new SERIAL_UDB_EXTRA_F15();
					((OnReceive) on_SERIAL_UDB_EXTRA_F15).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
					break;
				case 180:
					if (pack == null) return new SERIAL_UDB_EXTRA_F16();
					((OnReceive) on_SERIAL_UDB_EXTRA_F16).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
					break;
				case 181:
					if (pack == null) return new ALTITUDES();
					((OnReceive) on_ALTITUDES).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
					break;
				case 182:
					if (pack == null) return new AIRSPEEDS();
					((OnReceive) on_AIRSPEEDS).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
					break;
				case 183:
					if (pack == null) return new SERIAL_UDB_EXTRA_F17();
					((OnReceive) on_SERIAL_UDB_EXTRA_F17).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
					break;
				case 184:
					if (pack == null) return new SERIAL_UDB_EXTRA_F18();
					((OnReceive) on_SERIAL_UDB_EXTRA_F18).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
					break;
				case 185:
					if (pack == null) return new SERIAL_UDB_EXTRA_F19();
					((OnReceive) on_SERIAL_UDB_EXTRA_F19).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
					break;
				case 186:
					if (pack == null) return new SERIAL_UDB_EXTRA_F20();
					((OnReceive) on_SERIAL_UDB_EXTRA_F20).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
					break;
				case 187:
					if (pack == null) return new SERIAL_UDB_EXTRA_F21();
					((OnReceive) on_SERIAL_UDB_EXTRA_F21).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
					break;
				case 188:
					if (pack == null) return new SERIAL_UDB_EXTRA_F22();
					((OnReceive) on_SERIAL_UDB_EXTRA_F22).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
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
			assert (pack.custom_mode_GET() == 2700416332L);
			assert (pack.autopilot_GET() == MAV_AUTOPILOT.MAV_AUTOPILOT_AEROB);
			assert (pack.system_status_GET() == MAV_STATE.MAV_STATE_UNINIT);
			assert (pack.type_GET() == MAV_TYPE.MAV_TYPE_TRICOPTER);
			assert (pack.base_mode_GET() == MAV_MODE_FLAG.MAV_MODE_FLAG_HIL_ENABLED);
			assert (pack.mavlink_version_GET() == (char) 221);
		});
		HEARTBEAT p0 = new HEARTBEAT();
		PH.setPack(p0);
		p0.custom_mode_SET(2700416332L);
		p0.type_SET(MAV_TYPE.MAV_TYPE_TRICOPTER);
		p0.mavlink_version_SET((char) 221);
		p0.system_status_SET(MAV_STATE.MAV_STATE_UNINIT);
		p0.autopilot_SET(MAV_AUTOPILOT.MAV_AUTOPILOT_AEROB);
		p0.base_mode_SET(MAV_MODE_FLAG.MAV_MODE_FLAG_HIL_ENABLED);
		TestChannel.instance.send(p0);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_SYS_STATUS.add((src, ph, pack) ->
		{
			assert (pack.errors_comm_GET() == (char) 830);
			assert (pack.current_battery_GET() == (short) -10748);
			assert (pack.errors_count2_GET() == (char) 49101);
			assert (pack.voltage_battery_GET() == (char) 5065);
			assert (pack.drop_rate_comm_GET() == (char) 36408);
			assert (pack.battery_remaining_GET() == (byte) -52);
			assert (pack.onboard_control_sensors_health_GET() == MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL);
			assert (pack.onboard_control_sensors_enabled_GET() == MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH);
			assert (pack.errors_count1_GET() == (char) 56207);
			assert (pack.errors_count4_GET() == (char) 55879);
			assert (pack.onboard_control_sensors_present_GET() == MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL);
			assert (pack.errors_count3_GET() == (char) 34932);
			assert (pack.load_GET() == (char) 58202);
		});
		SYS_STATUS p1 = new SYS_STATUS();
		PH.setPack(p1);
		p1.voltage_battery_SET((char) 5065);
		p1.errors_count1_SET((char) 56207);
		p1.battery_remaining_SET((byte) -52);
		p1.onboard_control_sensors_present_SET(MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL);
		p1.load_SET((char) 58202);
		p1.errors_count2_SET((char) 49101);
		p1.onboard_control_sensors_enabled_SET(MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH);
		p1.errors_count3_SET((char) 34932);
		p1.drop_rate_comm_SET((char) 36408);
		p1.current_battery_SET((short) -10748);
		p1.errors_count4_SET((char) 55879);
		p1.onboard_control_sensors_health_SET(MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL);
		p1.errors_comm_SET((char) 830);
		TestChannel.instance.send(p1);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_SYSTEM_TIME.add((src, ph, pack) ->
		{
			assert (pack.time_unix_usec_GET() == 3781579353468286471L);
			assert (pack.time_boot_ms_GET() == 1953213730L);
		});
		SYSTEM_TIME p2 = new SYSTEM_TIME();
		PH.setPack(p2);
		p2.time_boot_ms_SET(1953213730L);
		p2.time_unix_usec_SET(3781579353468286471L);
		TestChannel.instance.send(p2);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_POSITION_TARGET_LOCAL_NED.add((src, ph, pack) ->
		{
			assert (pack.afz_GET() == -1.5215769E38F);
			assert (pack.type_mask_GET() == (char) 61716);
			assert (pack.vz_GET() == 4.749751E37F);
			assert (pack.vx_GET() == 2.593788E38F);
			assert (pack.afx_GET() == -4.1081174E37F);
			assert (pack.vy_GET() == -8.519343E37F);
			assert (pack.x_GET() == -2.672864E38F);
			assert (pack.y_GET() == -2.2996234E37F);
			assert (pack.yaw_GET() == 2.4655979E38F);
			assert (pack.afy_GET() == -1.2718672E38F);
			assert (pack.yaw_rate_GET() == -4.1096576E36F);
			assert (pack.z_GET() == -7.5971355E37F);
			assert (pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_BODY_NED);
			assert (pack.time_boot_ms_GET() == 3506340390L);
		});
		POSITION_TARGET_LOCAL_NED p3 = new POSITION_TARGET_LOCAL_NED();
		PH.setPack(p3);
		p3.afz_SET(-1.5215769E38F);
		p3.y_SET(-2.2996234E37F);
		p3.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_BODY_NED);
		p3.vx_SET(2.593788E38F);
		p3.yaw_SET(2.4655979E38F);
		p3.vz_SET(4.749751E37F);
		p3.afy_SET(-1.2718672E38F);
		p3.time_boot_ms_SET(3506340390L);
		p3.afx_SET(-4.1081174E37F);
		p3.vy_SET(-8.519343E37F);
		p3.x_SET(-2.672864E38F);
		p3.z_SET(-7.5971355E37F);
		p3.type_mask_SET((char) 61716);
		p3.yaw_rate_SET(-4.1096576E36F);
		TestChannel.instance.send(p3);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_PING.add((src, ph, pack) ->
		{
			assert (pack.time_usec_GET() == 5837025936870502767L);
			assert (pack.seq_GET() == 1711592623L);
			assert (pack.target_component_GET() == (char) 48);
			assert (pack.target_system_GET() == (char) 1);
		});
		PING p4 = new PING();
		PH.setPack(p4);
		p4.target_component_SET((char) 48);
		p4.time_usec_SET(5837025936870502767L);
		p4.seq_SET(1711592623L);
		p4.target_system_SET((char) 1);
		TestChannel.instance.send(p4);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_CHANGE_OPERATOR_CONTROL.add((src, ph, pack) ->
		{
			assert (pack.passkey_LEN(ph) == 19);
			assert (pack.passkey_TRY(ph).equals("oEgQpuonhFybthkzMar"));
			assert (pack.target_system_GET() == (char) 165);
			assert (pack.control_request_GET() == (char) 48);
			assert (pack.version_GET() == (char) 50);
		});
		CHANGE_OPERATOR_CONTROL p5 = new CHANGE_OPERATOR_CONTROL();
		PH.setPack(p5);
		p5.control_request_SET((char) 48);
		p5.version_SET((char) 50);
		p5.passkey_SET("oEgQpuonhFybthkzMar", PH);
		p5.target_system_SET((char) 165);
		TestChannel.instance.send(p5);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_CHANGE_OPERATOR_CONTROL_ACK.add((src, ph, pack) ->
		{
			assert (pack.ack_GET() == (char) 232);
			assert (pack.control_request_GET() == (char) 143);
			assert (pack.gcs_system_id_GET() == (char) 132);
		});
		CHANGE_OPERATOR_CONTROL_ACK p6 = new CHANGE_OPERATOR_CONTROL_ACK();
		PH.setPack(p6);
		p6.ack_SET((char) 232);
		p6.gcs_system_id_SET((char) 132);
		p6.control_request_SET((char) 143);
		TestChannel.instance.send(p6);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_AUTH_KEY.add((src, ph, pack) ->
		{
			assert (pack.key_LEN(ph) == 28);
			assert (pack.key_TRY(ph).equals("brhOgAsIkHejoydnuedfgcieybsz"));
		});
		AUTH_KEY p7 = new AUTH_KEY();
		PH.setPack(p7);
		p7.key_SET("brhOgAsIkHejoydnuedfgcieybsz", PH);
		TestChannel.instance.send(p7);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_SET_MODE.add((src, ph, pack) ->
		{
			assert (pack.base_mode_GET() == MAV_MODE.MAV_MODE_STABILIZE_DISARMED);
			assert (pack.custom_mode_GET() == 2192356958L);
			assert (pack.target_system_GET() == (char) 213);
		});
		SET_MODE p11 = new SET_MODE();
		PH.setPack(p11);
		p11.base_mode_SET(MAV_MODE.MAV_MODE_STABILIZE_DISARMED);
		p11.custom_mode_SET(2192356958L);
		p11.target_system_SET((char) 213);
		TestChannel.instance.send(p11);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_PARAM_REQUEST_READ.add((src, ph, pack) ->
		{
			assert (pack.param_id_LEN(ph) == 11);
			assert (pack.param_id_TRY(ph).equals("xitowGswtXf"));
			assert (pack.target_system_GET() == (char) 114);
			assert (pack.param_index_GET() == (short) 32477);
			assert (pack.target_component_GET() == (char) 191);
		});
		PARAM_REQUEST_READ p20 = new PARAM_REQUEST_READ();
		PH.setPack(p20);
		p20.param_id_SET("xitowGswtXf", PH);
		p20.target_system_SET((char) 114);
		p20.target_component_SET((char) 191);
		p20.param_index_SET((short) 32477);
		TestChannel.instance.send(p20);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_PARAM_REQUEST_LIST.add((src, ph, pack) ->
		{
			assert (pack.target_component_GET() == (char) 45);
			assert (pack.target_system_GET() == (char) 67);
		});
		PARAM_REQUEST_LIST p21 = new PARAM_REQUEST_LIST();
		PH.setPack(p21);
		p21.target_component_SET((char) 45);
		p21.target_system_SET((char) 67);
		TestChannel.instance.send(p21);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_PARAM_VALUE.add((src, ph, pack) ->
		{
			assert (pack.param_type_GET() == MAV_PARAM_TYPE.MAV_PARAM_TYPE_REAL32);
			assert (pack.param_id_LEN(ph) == 16);
			assert (pack.param_id_TRY(ph).equals("Qokohppzuanrfybu"));
			assert (pack.param_index_GET() == (char) 40836);
			assert (pack.param_value_GET() == -2.7128335E38F);
			assert (pack.param_count_GET() == (char) 53357);
		});
		PARAM_VALUE p22 = new PARAM_VALUE();
		PH.setPack(p22);
		p22.param_index_SET((char) 40836);
		p22.param_type_SET(MAV_PARAM_TYPE.MAV_PARAM_TYPE_REAL32);
		p22.param_id_SET("Qokohppzuanrfybu", PH);
		p22.param_value_SET(-2.7128335E38F);
		p22.param_count_SET((char) 53357);
		TestChannel.instance.send(p22);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_PARAM_SET.add((src, ph, pack) ->
		{
			assert (pack.target_system_GET() == (char) 62);
			assert (pack.param_id_LEN(ph) == 1);
			assert (pack.param_id_TRY(ph).equals("s"));
			assert (pack.target_component_GET() == (char) 49);
			assert (pack.param_value_GET() == 4.4871074E37F);
			assert (pack.param_type_GET() == MAV_PARAM_TYPE.MAV_PARAM_TYPE_REAL32);
		});
		PARAM_SET p23 = new PARAM_SET();
		PH.setPack(p23);
		p23.param_type_SET(MAV_PARAM_TYPE.MAV_PARAM_TYPE_REAL32);
		p23.target_system_SET((char) 62);
		p23.param_value_SET(4.4871074E37F);
		p23.target_component_SET((char) 49);
		p23.param_id_SET("s", PH);
		TestChannel.instance.send(p23);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_GPS_RAW_INT.add((src, ph, pack) ->
		{
			assert (pack.satellites_visible_GET() == (char) 54);
			assert (pack.vel_GET() == (char) 47213);
			assert (pack.alt_GET() == 1968453553);
			assert (pack.vel_acc_TRY(ph) == 1693904570L);
			assert (pack.lon_GET() == -1564041181);
			assert (pack.eph_GET() == (char) 10124);
			assert (pack.lat_GET() == -44469732);
			assert (pack.time_usec_GET() == 9184836161516075084L);
			assert (pack.h_acc_TRY(ph) == 1621237748L);
			assert (pack.hdg_acc_TRY(ph) == 4069658375L);
			assert (pack.cog_GET() == (char) 3954);
			assert (pack.v_acc_TRY(ph) == 2706186593L);
			assert (pack.epv_GET() == (char) 38890);
			assert (pack.alt_ellipsoid_TRY(ph) == -662835236);
			assert (pack.fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_STATIC);
		});
		GPS_RAW_INT p24 = new GPS_RAW_INT();
		PH.setPack(p24);
		p24.satellites_visible_SET((char) 54);
		p24.lon_SET(-1564041181);
		p24.fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_STATIC);
		p24.alt_SET(1968453553);
		p24.time_usec_SET(9184836161516075084L);
		p24.vel_acc_SET(1693904570L, PH);
		p24.h_acc_SET(1621237748L, PH);
		p24.alt_ellipsoid_SET(-662835236, PH);
		p24.epv_SET((char) 38890);
		p24.cog_SET((char) 3954);
		p24.hdg_acc_SET(4069658375L, PH);
		p24.lat_SET(-44469732);
		p24.vel_SET((char) 47213);
		p24.v_acc_SET(2706186593L, PH);
		p24.eph_SET((char) 10124);
		TestChannel.instance.send(p24);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_GPS_STATUS.add((src, ph, pack) ->
		{
			assert (Arrays.equals(pack.satellite_prn_GET(), new char[]{(char) 4, (char) 56, (char) 80, (char) 0, (char) 191, (char) 123, (char) 124, (char) 255, (char) 89, (char) 104, (char) 122, (char) 27, (char) 129, (char) 189, (char) 93, (char) 98, (char) 141, (char) 118, (char) 76, (char) 76}));
			assert (Arrays.equals(pack.satellite_used_GET(), new char[]{(char) 61, (char) 28, (char) 55, (char) 220, (char) 7, (char) 66, (char) 214, (char) 53, (char) 30, (char) 118, (char) 147, (char) 3, (char) 190, (char) 69, (char) 46, (char) 177, (char) 88, (char) 13, (char) 37, (char) 172}));
			assert (Arrays.equals(pack.satellite_snr_GET(), new char[]{(char) 9, (char) 131, (char) 55, (char) 236, (char) 76, (char) 61, (char) 110, (char) 40, (char) 75, (char) 34, (char) 106, (char) 93, (char) 236, (char) 177, (char) 6, (char) 199, (char) 93, (char) 254, (char) 52, (char) 237}));
			assert (Arrays.equals(pack.satellite_elevation_GET(), new char[]{(char) 90, (char) 204, (char) 222, (char) 240, (char) 74, (char) 142, (char) 224, (char) 146, (char) 87, (char) 210, (char) 182, (char) 79, (char) 17, (char) 252, (char) 166, (char) 99, (char) 190, (char) 175, (char) 15, (char) 158}));
			assert (pack.satellites_visible_GET() == (char) 77);
			assert (Arrays.equals(pack.satellite_azimuth_GET(), new char[]{(char) 161, (char) 189, (char) 180, (char) 202, (char) 18, (char) 200, (char) 245, (char) 43, (char) 118, (char) 175, (char) 119, (char) 129, (char) 154, (char) 167, (char) 123, (char) 237, (char) 180, (char) 88, (char) 105, (char) 228}));
		});
		GPS_STATUS p25 = new GPS_STATUS();
		PH.setPack(p25);
		p25.satellites_visible_SET((char) 77);
		p25.satellite_azimuth_SET(new char[]{(char) 161, (char) 189, (char) 180, (char) 202, (char) 18, (char) 200, (char) 245, (char) 43, (char) 118, (char) 175, (char) 119, (char) 129, (char) 154, (char) 167, (char) 123, (char) 237, (char) 180, (char) 88, (char) 105, (char) 228}, 0);
		p25.satellite_snr_SET(new char[]{(char) 9, (char) 131, (char) 55, (char) 236, (char) 76, (char) 61, (char) 110, (char) 40, (char) 75, (char) 34, (char) 106, (char) 93, (char) 236, (char) 177, (char) 6, (char) 199, (char) 93, (char) 254, (char) 52, (char) 237}, 0);
		p25.satellite_used_SET(new char[]{(char) 61, (char) 28, (char) 55, (char) 220, (char) 7, (char) 66, (char) 214, (char) 53, (char) 30, (char) 118, (char) 147, (char) 3, (char) 190, (char) 69, (char) 46, (char) 177, (char) 88, (char) 13, (char) 37, (char) 172}, 0);
		p25.satellite_elevation_SET(new char[]{(char) 90, (char) 204, (char) 222, (char) 240, (char) 74, (char) 142, (char) 224, (char) 146, (char) 87, (char) 210, (char) 182, (char) 79, (char) 17, (char) 252, (char) 166, (char) 99, (char) 190, (char) 175, (char) 15, (char) 158}, 0);
		p25.satellite_prn_SET(new char[]{(char) 4, (char) 56, (char) 80, (char) 0, (char) 191, (char) 123, (char) 124, (char) 255, (char) 89, (char) 104, (char) 122, (char) 27, (char) 129, (char) 189, (char) 93, (char) 98, (char) 141, (char) 118, (char) 76, (char) 76}, 0);
		TestChannel.instance.send(p25);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_SCALED_IMU.add((src, ph, pack) ->
		{
			assert (pack.ymag_GET() == (short) 20070);
			assert (pack.xgyro_GET() == (short) 31164);
			assert (pack.zacc_GET() == (short) 12912);
			assert (pack.zmag_GET() == (short) -7446);
			assert (pack.zgyro_GET() == (short) 23121);
			assert (pack.xacc_GET() == (short) -10766);
			assert (pack.ygyro_GET() == (short) -27074);
			assert (pack.xmag_GET() == (short) 25706);
			assert (pack.yacc_GET() == (short) 4371);
			assert (pack.time_boot_ms_GET() == 584894454L);
		});
		SCALED_IMU p26 = new SCALED_IMU();
		PH.setPack(p26);
		p26.zgyro_SET((short) 23121);
		p26.time_boot_ms_SET(584894454L);
		p26.zmag_SET((short) -7446);
		p26.xacc_SET((short) -10766);
		p26.xmag_SET((short) 25706);
		p26.ygyro_SET((short) -27074);
		p26.ymag_SET((short) 20070);
		p26.xgyro_SET((short) 31164);
		p26.yacc_SET((short) 4371);
		p26.zacc_SET((short) 12912);
		TestChannel.instance.send(p26);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_RAW_IMU.add((src, ph, pack) ->
		{
			assert (pack.time_usec_GET() == 3811539208820406690L);
			assert (pack.xmag_GET() == (short) 30492);
			assert (pack.yacc_GET() == (short) -29066);
			assert (pack.ymag_GET() == (short) -8638);
			assert (pack.ygyro_GET() == (short) 14033);
			assert (pack.zmag_GET() == (short) 27289);
			assert (pack.zacc_GET() == (short) 5595);
			assert (pack.zgyro_GET() == (short) -23027);
			assert (pack.xacc_GET() == (short) -23089);
			assert (pack.xgyro_GET() == (short) -22563);
		});
		RAW_IMU p27 = new RAW_IMU();
		PH.setPack(p27);
		p27.zmag_SET((short) 27289);
		p27.xacc_SET((short) -23089);
		p27.xmag_SET((short) 30492);
		p27.xgyro_SET((short) -22563);
		p27.ygyro_SET((short) 14033);
		p27.zacc_SET((short) 5595);
		p27.ymag_SET((short) -8638);
		p27.time_usec_SET(3811539208820406690L);
		p27.yacc_SET((short) -29066);
		p27.zgyro_SET((short) -23027);
		TestChannel.instance.send(p27);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_RAW_PRESSURE.add((src, ph, pack) ->
		{
			assert (pack.press_abs_GET() == (short) -16999);
			assert (pack.press_diff1_GET() == (short) 19691);
			assert (pack.press_diff2_GET() == (short) 30449);
			assert (pack.time_usec_GET() == 4553215534808041098L);
			assert (pack.temperature_GET() == (short) -16832);
		});
		RAW_PRESSURE p28 = new RAW_PRESSURE();
		PH.setPack(p28);
		p28.temperature_SET((short) -16832);
		p28.time_usec_SET(4553215534808041098L);
		p28.press_diff1_SET((short) 19691);
		p28.press_abs_SET((short) -16999);
		p28.press_diff2_SET((short) 30449);
		TestChannel.instance.send(p28);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_SCALED_PRESSURE.add((src, ph, pack) ->
		{
			assert (pack.press_abs_GET() == 2.2792463E38F);
			assert (pack.press_diff_GET() == -3.1686156E38F);
			assert (pack.temperature_GET() == (short) -1975);
			assert (pack.time_boot_ms_GET() == 2351391803L);
		});
		SCALED_PRESSURE p29 = new SCALED_PRESSURE();
		PH.setPack(p29);
		p29.press_abs_SET(2.2792463E38F);
		p29.temperature_SET((short) -1975);
		p29.press_diff_SET(-3.1686156E38F);
		p29.time_boot_ms_SET(2351391803L);
		TestChannel.instance.send(p29);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_ATTITUDE.add((src, ph, pack) ->
		{
			assert (pack.pitchspeed_GET() == 2.4451834E36F);
			assert (pack.yawspeed_GET() == 2.497311E38F);
			assert (pack.roll_GET() == -1.661017E38F);
			assert (pack.rollspeed_GET() == 9.141945E37F);
			assert (pack.pitch_GET() == -2.8928935E38F);
			assert (pack.yaw_GET() == 1.0233416E38F);
			assert (pack.time_boot_ms_GET() == 1908549004L);
		});
		ATTITUDE p30 = new ATTITUDE();
		PH.setPack(p30);
		p30.yawspeed_SET(2.497311E38F);
		p30.pitchspeed_SET(2.4451834E36F);
		p30.rollspeed_SET(9.141945E37F);
		p30.roll_SET(-1.661017E38F);
		p30.yaw_SET(1.0233416E38F);
		p30.pitch_SET(-2.8928935E38F);
		p30.time_boot_ms_SET(1908549004L);
		TestChannel.instance.send(p30);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_ATTITUDE_QUATERNION.add((src, ph, pack) ->
		{
			assert (pack.q3_GET() == 1.7693403E38F);
			assert (pack.pitchspeed_GET() == -8.2581007E37F);
			assert (pack.q1_GET() == -3.1309982E38F);
			assert (pack.q4_GET() == -3.2564372E38F);
			assert (pack.rollspeed_GET() == -2.2734445E38F);
			assert (pack.yawspeed_GET() == 1.7835964E38F);
			assert (pack.q2_GET() == 1.4968113E38F);
			assert (pack.time_boot_ms_GET() == 2016881227L);
		});
		ATTITUDE_QUATERNION p31 = new ATTITUDE_QUATERNION();
		PH.setPack(p31);
		p31.q2_SET(1.4968113E38F);
		p31.pitchspeed_SET(-8.2581007E37F);
		p31.q4_SET(-3.2564372E38F);
		p31.yawspeed_SET(1.7835964E38F);
		p31.q1_SET(-3.1309982E38F);
		p31.rollspeed_SET(-2.2734445E38F);
		p31.time_boot_ms_SET(2016881227L);
		p31.q3_SET(1.7693403E38F);
		TestChannel.instance.send(p31);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_LOCAL_POSITION_NED.add((src, ph, pack) ->
		{
			assert (pack.z_GET() == -1.6741469E38F);
			assert (pack.vz_GET() == -1.9699716E38F);
			assert (pack.x_GET() == 2.3759354E38F);
			assert (pack.time_boot_ms_GET() == 2089328333L);
			assert (pack.vy_GET() == 3.0156811E38F);
			assert (pack.vx_GET() == -2.2545626E38F);
			assert (pack.y_GET() == -1.3568173E38F);
		});
		LOCAL_POSITION_NED p32 = new LOCAL_POSITION_NED();
		PH.setPack(p32);
		p32.y_SET(-1.3568173E38F);
		p32.vx_SET(-2.2545626E38F);
		p32.z_SET(-1.6741469E38F);
		p32.time_boot_ms_SET(2089328333L);
		p32.vy_SET(3.0156811E38F);
		p32.x_SET(2.3759354E38F);
		p32.vz_SET(-1.9699716E38F);
		TestChannel.instance.send(p32);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_GLOBAL_POSITION_INT.add((src, ph, pack) ->
		{
			assert (pack.alt_GET() == 1001304618);
			assert (pack.vx_GET() == (short) -8531);
			assert (pack.lon_GET() == 1931033571);
			assert (pack.relative_alt_GET() == -907410739);
			assert (pack.lat_GET() == -1226204889);
			assert (pack.vz_GET() == (short) -10416);
			assert (pack.hdg_GET() == (char) 58961);
			assert (pack.vy_GET() == (short) -1476);
			assert (pack.time_boot_ms_GET() == 3722075165L);
		});
		GLOBAL_POSITION_INT p33 = new GLOBAL_POSITION_INT();
		PH.setPack(p33);
		p33.vy_SET((short) -1476);
		p33.lat_SET(-1226204889);
		p33.vz_SET((short) -10416);
		p33.hdg_SET((char) 58961);
		p33.vx_SET((short) -8531);
		p33.lon_SET(1931033571);
		p33.relative_alt_SET(-907410739);
		p33.alt_SET(1001304618);
		p33.time_boot_ms_SET(3722075165L);
		TestChannel.instance.send(p33);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_RC_CHANNELS_SCALED.add((src, ph, pack) ->
		{
			assert (pack.chan4_scaled_GET() == (short) -1109);
			assert (pack.chan2_scaled_GET() == (short) -5593);
			assert (pack.chan8_scaled_GET() == (short) 4853);
			assert (pack.chan6_scaled_GET() == (short) 5036);
			assert (pack.chan1_scaled_GET() == (short) 20301);
			assert (pack.rssi_GET() == (char) 232);
			assert (pack.port_GET() == (char) 86);
			assert (pack.chan3_scaled_GET() == (short) -25654);
			assert (pack.chan7_scaled_GET() == (short) -3742);
			assert (pack.chan5_scaled_GET() == (short) -20948);
			assert (pack.time_boot_ms_GET() == 997014525L);
		});
		RC_CHANNELS_SCALED p34 = new RC_CHANNELS_SCALED();
		PH.setPack(p34);
		p34.chan1_scaled_SET((short) 20301);
		p34.chan7_scaled_SET((short) -3742);
		p34.time_boot_ms_SET(997014525L);
		p34.chan5_scaled_SET((short) -20948);
		p34.chan8_scaled_SET((short) 4853);
		p34.chan6_scaled_SET((short) 5036);
		p34.chan2_scaled_SET((short) -5593);
		p34.rssi_SET((char) 232);
		p34.chan3_scaled_SET((short) -25654);
		p34.chan4_scaled_SET((short) -1109);
		p34.port_SET((char) 86);
		TestChannel.instance.send(p34);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_RC_CHANNELS_RAW.add((src, ph, pack) ->
		{
			assert (pack.chan5_raw_GET() == (char) 48887);
			assert (pack.port_GET() == (char) 133);
			assert (pack.chan1_raw_GET() == (char) 55996);
			assert (pack.chan4_raw_GET() == (char) 22116);
			assert (pack.chan6_raw_GET() == (char) 14778);
			assert (pack.chan3_raw_GET() == (char) 56886);
			assert (pack.chan2_raw_GET() == (char) 57883);
			assert (pack.chan8_raw_GET() == (char) 6685);
			assert (pack.rssi_GET() == (char) 238);
			assert (pack.chan7_raw_GET() == (char) 31467);
			assert (pack.time_boot_ms_GET() == 2999381145L);
		});
		RC_CHANNELS_RAW p35 = new RC_CHANNELS_RAW();
		PH.setPack(p35);
		p35.chan7_raw_SET((char) 31467);
		p35.chan8_raw_SET((char) 6685);
		p35.time_boot_ms_SET(2999381145L);
		p35.chan5_raw_SET((char) 48887);
		p35.chan3_raw_SET((char) 56886);
		p35.chan4_raw_SET((char) 22116);
		p35.port_SET((char) 133);
		p35.chan1_raw_SET((char) 55996);
		p35.chan6_raw_SET((char) 14778);
		p35.chan2_raw_SET((char) 57883);
		p35.rssi_SET((char) 238);
		TestChannel.instance.send(p35);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_SERVO_OUTPUT_RAW.add((src, ph, pack) ->
		{
			assert (pack.servo9_raw_TRY(ph) == (char) 3623);
			assert (pack.time_usec_GET() == 320370836L);
			assert (pack.servo16_raw_TRY(ph) == (char) 4746);
			assert (pack.servo1_raw_GET() == (char) 34246);
			assert (pack.servo10_raw_TRY(ph) == (char) 4883);
			assert (pack.servo15_raw_TRY(ph) == (char) 1802);
			assert (pack.servo14_raw_TRY(ph) == (char) 12073);
			assert (pack.servo2_raw_GET() == (char) 12188);
			assert (pack.servo12_raw_TRY(ph) == (char) 32153);
			assert (pack.servo7_raw_GET() == (char) 47800);
			assert (pack.servo13_raw_TRY(ph) == (char) 231);
			assert (pack.servo5_raw_GET() == (char) 31752);
			assert (pack.servo3_raw_GET() == (char) 61958);
			assert (pack.servo4_raw_GET() == (char) 15333);
			assert (pack.port_GET() == (char) 242);
			assert (pack.servo11_raw_TRY(ph) == (char) 51202);
			assert (pack.servo8_raw_GET() == (char) 16030);
			assert (pack.servo6_raw_GET() == (char) 56484);
		});
		SERVO_OUTPUT_RAW p36 = new SERVO_OUTPUT_RAW();
		PH.setPack(p36);
		p36.servo8_raw_SET((char) 16030);
		p36.servo3_raw_SET((char) 61958);
		p36.servo6_raw_SET((char) 56484);
		p36.servo7_raw_SET((char) 47800);
		p36.servo4_raw_SET((char) 15333);
		p36.servo1_raw_SET((char) 34246);
		p36.servo12_raw_SET((char) 32153, PH);
		p36.servo16_raw_SET((char) 4746, PH);
		p36.time_usec_SET(320370836L);
		p36.servo11_raw_SET((char) 51202, PH);
		p36.servo14_raw_SET((char) 12073, PH);
		p36.servo15_raw_SET((char) 1802, PH);
		p36.servo9_raw_SET((char) 3623, PH);
		p36.servo5_raw_SET((char) 31752);
		p36.port_SET((char) 242);
		p36.servo10_raw_SET((char) 4883, PH);
		p36.servo2_raw_SET((char) 12188);
		p36.servo13_raw_SET((char) 231, PH);
		TestChannel.instance.send(p36);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_MISSION_REQUEST_PARTIAL_LIST.add((src, ph, pack) ->
		{
			assert (pack.target_system_GET() == (char) 165);
			assert (pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
			assert (pack.target_component_GET() == (char) 10);
			assert (pack.start_index_GET() == (short) 494);
			assert (pack.end_index_GET() == (short) -1502);
		});
		MISSION_REQUEST_PARTIAL_LIST p37 = new MISSION_REQUEST_PARTIAL_LIST();
		PH.setPack(p37);
		p37.target_system_SET((char) 165);
		p37.end_index_SET((short) -1502);
		p37.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
		p37.target_component_SET((char) 10);
		p37.start_index_SET((short) 494);
		TestChannel.instance.send(p37);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_MISSION_WRITE_PARTIAL_LIST.add((src, ph, pack) ->
		{
			assert (pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
			assert (pack.start_index_GET() == (short) 1527);
			assert (pack.target_system_GET() == (char) 121);
			assert (pack.end_index_GET() == (short) -14923);
			assert (pack.target_component_GET() == (char) 238);
		});
		MISSION_WRITE_PARTIAL_LIST p38 = new MISSION_WRITE_PARTIAL_LIST();
		PH.setPack(p38);
		p38.end_index_SET((short) -14923);
		p38.start_index_SET((short) 1527);
		p38.target_component_SET((char) 238);
		p38.target_system_SET((char) 121);
		p38.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
		TestChannel.instance.send(p38);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_MISSION_ITEM.add((src, ph, pack) ->
		{
			assert (pack.z_GET() == 3.1458445E38F);
			assert (pack.y_GET() == 2.11656E38F);
			assert (pack.param4_GET() == -2.9199175E37F);
			assert (pack.param3_GET() == -1.1195572E38F);
			assert (pack.param1_GET() == -5.917265E36F);
			assert (pack.frame_GET() == MAV_FRAME.MAV_FRAME_LOCAL_ENU);
			assert (pack.x_GET() == 3.3584324E38F);
			assert (pack.target_system_GET() == (char) 221);
			assert (pack.param2_GET() == -2.4719742E38F);
			assert (pack.current_GET() == (char) 52);
			assert (pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
			assert (pack.target_component_GET() == (char) 195);
			assert (pack.autocontinue_GET() == (char) 75);
			assert (pack.command_GET() == MAV_CMD.MAV_CMD_USER_4);
			assert (pack.seq_GET() == (char) 50396);
		});
		MISSION_ITEM p39 = new MISSION_ITEM();
		PH.setPack(p39);
		p39.param2_SET(-2.4719742E38F);
		p39.autocontinue_SET((char) 75);
		p39.current_SET((char) 52);
		p39.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
		p39.x_SET(3.3584324E38F);
		p39.param1_SET(-5.917265E36F);
		p39.target_component_SET((char) 195);
		p39.target_system_SET((char) 221);
		p39.frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_ENU);
		p39.y_SET(2.11656E38F);
		p39.z_SET(3.1458445E38F);
		p39.param3_SET(-1.1195572E38F);
		p39.seq_SET((char) 50396);
		p39.command_SET(MAV_CMD.MAV_CMD_USER_4);
		p39.param4_SET(-2.9199175E37F);
		TestChannel.instance.send(p39);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_MISSION_REQUEST.add((src, ph, pack) ->
		{
			assert (pack.target_system_GET() == (char) 237);
			assert (pack.target_component_GET() == (char) 130);
			assert (pack.seq_GET() == (char) 27432);
			assert (pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
		});
		MISSION_REQUEST p40 = new MISSION_REQUEST();
		PH.setPack(p40);
		p40.target_system_SET((char) 237);
		p40.seq_SET((char) 27432);
		p40.target_component_SET((char) 130);
		p40.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
		TestChannel.instance.send(p40);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_MISSION_SET_CURRENT.add((src, ph, pack) ->
		{
			assert (pack.target_system_GET() == (char) 32);
			assert (pack.target_component_GET() == (char) 103);
			assert (pack.seq_GET() == (char) 11255);
		});
		MISSION_SET_CURRENT p41 = new MISSION_SET_CURRENT();
		PH.setPack(p41);
		p41.seq_SET((char) 11255);
		p41.target_component_SET((char) 103);
		p41.target_system_SET((char) 32);
		TestChannel.instance.send(p41);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_MISSION_CURRENT.add((src, ph, pack) ->
		{
			assert (pack.seq_GET() == (char) 19501);
		});
		MISSION_CURRENT p42 = new MISSION_CURRENT();
		PH.setPack(p42);
		p42.seq_SET((char) 19501);
		TestChannel.instance.send(p42);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_MISSION_REQUEST_LIST.add((src, ph, pack) ->
		{
			assert (pack.target_component_GET() == (char) 85);
			assert (pack.target_system_GET() == (char) 53);
			assert (pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
		});
		MISSION_REQUEST_LIST p43 = new MISSION_REQUEST_LIST();
		PH.setPack(p43);
		p43.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
		p43.target_component_SET((char) 85);
		p43.target_system_SET((char) 53);
		TestChannel.instance.send(p43);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_MISSION_COUNT.add((src, ph, pack) ->
		{
			assert (pack.target_system_GET() == (char) 219);
			assert (pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
			assert (pack.count_GET() == (char) 9721);
			assert (pack.target_component_GET() == (char) 170);
		});
		MISSION_COUNT p44 = new MISSION_COUNT();
		PH.setPack(p44);
		p44.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
		p44.target_component_SET((char) 170);
		p44.count_SET((char) 9721);
		p44.target_system_SET((char) 219);
		TestChannel.instance.send(p44);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_MISSION_CLEAR_ALL.add((src, ph, pack) ->
		{
			assert (pack.target_component_GET() == (char) 1);
			assert (pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
			assert (pack.target_system_GET() == (char) 9);
		});
		MISSION_CLEAR_ALL p45 = new MISSION_CLEAR_ALL();
		PH.setPack(p45);
		p45.target_component_SET((char) 1);
		p45.target_system_SET((char) 9);
		p45.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
		TestChannel.instance.send(p45);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_MISSION_ITEM_REACHED.add((src, ph, pack) ->
		{
			assert (pack.seq_GET() == (char) 57012);
		});
		MISSION_ITEM_REACHED p46 = new MISSION_ITEM_REACHED();
		PH.setPack(p46);
		p46.seq_SET((char) 57012);
		TestChannel.instance.send(p46);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_MISSION_ACK.add((src, ph, pack) ->
		{
			assert (pack.target_component_GET() == (char) 123);
			assert (pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
			assert (pack.type_GET() == MAV_MISSION_RESULT.MAV_MISSION_UNSUPPORTED);
			assert (pack.target_system_GET() == (char) 78);
		});
		MISSION_ACK p47 = new MISSION_ACK();
		PH.setPack(p47);
		p47.type_SET(MAV_MISSION_RESULT.MAV_MISSION_UNSUPPORTED);
		p47.target_system_SET((char) 78);
		p47.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
		p47.target_component_SET((char) 123);
		TestChannel.instance.send(p47);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_SET_GPS_GLOBAL_ORIGIN.add((src, ph, pack) ->
		{
			assert (pack.time_usec_TRY(ph) == 5093765519147236982L);
			assert (pack.longitude_GET() == -1263816140);
			assert (pack.altitude_GET() == 2056966931);
			assert (pack.latitude_GET() == 2051752669);
			assert (pack.target_system_GET() == (char) 23);
		});
		SET_GPS_GLOBAL_ORIGIN p48 = new SET_GPS_GLOBAL_ORIGIN();
		PH.setPack(p48);
		p48.altitude_SET(2056966931);
		p48.latitude_SET(2051752669);
		p48.target_system_SET((char) 23);
		p48.longitude_SET(-1263816140);
		p48.time_usec_SET(5093765519147236982L, PH);
		TestChannel.instance.send(p48);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_GPS_GLOBAL_ORIGIN.add((src, ph, pack) ->
		{
			assert (pack.altitude_GET() == 264653272);
			assert (pack.latitude_GET() == 602625127);
			assert (pack.longitude_GET() == 683561902);
			assert (pack.time_usec_TRY(ph) == 7919294891390108296L);
		});
		GPS_GLOBAL_ORIGIN p49 = new GPS_GLOBAL_ORIGIN();
		PH.setPack(p49);
		p49.latitude_SET(602625127);
		p49.altitude_SET(264653272);
		p49.time_usec_SET(7919294891390108296L, PH);
		p49.longitude_SET(683561902);
		TestChannel.instance.send(p49);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_PARAM_MAP_RC.add((src, ph, pack) ->
		{
			assert (pack.param_value0_GET() == -2.6313688E38F);
			assert (pack.param_index_GET() == (short) -29802);
			assert (pack.parameter_rc_channel_index_GET() == (char) 47);
			assert (pack.param_id_LEN(ph) == 5);
			assert (pack.param_id_TRY(ph).equals("ujzzh"));
			assert (pack.scale_GET() == 1.8061624E38F);
			assert (pack.target_system_GET() == (char) 48);
			assert (pack.param_value_max_GET() == 2.1131733E38F);
			assert (pack.target_component_GET() == (char) 58);
			assert (pack.param_value_min_GET() == 9.950135E37F);
		});
		PARAM_MAP_RC p50 = new PARAM_MAP_RC();
		PH.setPack(p50);
		p50.param_value0_SET(-2.6313688E38F);
		p50.target_system_SET((char) 48);
		p50.scale_SET(1.8061624E38F);
		p50.param_id_SET("ujzzh", PH);
		p50.target_component_SET((char) 58);
		p50.param_index_SET((short) -29802);
		p50.param_value_max_SET(2.1131733E38F);
		p50.param_value_min_SET(9.950135E37F);
		p50.parameter_rc_channel_index_SET((char) 47);
		TestChannel.instance.send(p50);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_MISSION_REQUEST_INT.add((src, ph, pack) ->
		{
			assert (pack.target_component_GET() == (char) 223);
			assert (pack.target_system_GET() == (char) 124);
			assert (pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
			assert (pack.seq_GET() == (char) 35528);
		});
		MISSION_REQUEST_INT p51 = new MISSION_REQUEST_INT();
		PH.setPack(p51);
		p51.target_component_SET((char) 223);
		p51.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
		p51.target_system_SET((char) 124);
		p51.seq_SET((char) 35528);
		TestChannel.instance.send(p51);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_SAFETY_SET_ALLOWED_AREA.add((src, ph, pack) ->
		{
			assert (pack.p1x_GET() == -8.991313E37F);
			assert (pack.p2y_GET() == -3.3333943E38F);
			assert (pack.frame_GET() == MAV_FRAME.MAV_FRAME_LOCAL_NED);
			assert (pack.target_component_GET() == (char) 214);
			assert (pack.p1z_GET() == -1.4522641E38F);
			assert (pack.p2z_GET() == 8.337049E37F);
			assert (pack.p1y_GET() == -1.9671962E38F);
			assert (pack.target_system_GET() == (char) 173);
			assert (pack.p2x_GET() == -2.922622E38F);
		});
		SAFETY_SET_ALLOWED_AREA p54 = new SAFETY_SET_ALLOWED_AREA();
		PH.setPack(p54);
		p54.p1z_SET(-1.4522641E38F);
		p54.p2x_SET(-2.922622E38F);
		p54.p2y_SET(-3.3333943E38F);
		p54.p2z_SET(8.337049E37F);
		p54.p1y_SET(-1.9671962E38F);
		p54.target_system_SET((char) 173);
		p54.frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_NED);
		p54.p1x_SET(-8.991313E37F);
		p54.target_component_SET((char) 214);
		TestChannel.instance.send(p54);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_SAFETY_ALLOWED_AREA.add((src, ph, pack) ->
		{
			assert (pack.p1y_GET() == 1.5343002E38F);
			assert (pack.p2z_GET() == -1.3827368E38F);
			assert (pack.p1z_GET() == -8.989093E37F);
			assert (pack.p2x_GET() == 3.335159E38F);
			assert (pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT);
			assert (pack.p2y_GET() == -2.3948193E38F);
			assert (pack.p1x_GET() == -1.7334442E38F);
		});
		SAFETY_ALLOWED_AREA p55 = new SAFETY_ALLOWED_AREA();
		PH.setPack(p55);
		p55.p2y_SET(-2.3948193E38F);
		p55.p2z_SET(-1.3827368E38F);
		p55.p1x_SET(-1.7334442E38F);
		p55.p1z_SET(-8.989093E37F);
		p55.p2x_SET(3.335159E38F);
		p55.p1y_SET(1.5343002E38F);
		p55.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT);
		TestChannel.instance.send(p55);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_ATTITUDE_QUATERNION_COV.add((src, ph, pack) ->
		{
			assert (pack.rollspeed_GET() == -2.3631735E38F);
			assert (Arrays.equals(pack.covariance_GET(), new float[]{-4.0094683E37F, -3.184275E38F, -1.055893E38F, 2.5546908E38F, 1.8826307E38F, -6.1896737E37F, -2.9833248E38F, 3.059374E36F, 1.3248233E38F}));
			assert (Arrays.equals(pack.q_GET(), new float[]{-3.2942897E38F, -2.0123428E38F, -4.881054E37F, 2.6809277E38F}));
			assert (pack.yawspeed_GET() == 1.5440072E38F);
			assert (pack.pitchspeed_GET() == -3.16922E38F);
			assert (pack.time_usec_GET() == 1750772709552740273L);
		});
		ATTITUDE_QUATERNION_COV p61 = new ATTITUDE_QUATERNION_COV();
		PH.setPack(p61);
		p61.yawspeed_SET(1.5440072E38F);
		p61.covariance_SET(new float[]{-4.0094683E37F, -3.184275E38F, -1.055893E38F, 2.5546908E38F, 1.8826307E38F, -6.1896737E37F, -2.9833248E38F, 3.059374E36F, 1.3248233E38F}, 0);
		p61.rollspeed_SET(-2.3631735E38F);
		p61.pitchspeed_SET(-3.16922E38F);
		p61.q_SET(new float[]{-3.2942897E38F, -2.0123428E38F, -4.881054E37F, 2.6809277E38F}, 0);
		p61.time_usec_SET(1750772709552740273L);
		TestChannel.instance.send(p61);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_NAV_CONTROLLER_OUTPUT.add((src, ph, pack) ->
		{
			assert (pack.nav_pitch_GET() == 2.385783E38F);
			assert (pack.xtrack_error_GET() == 3.126433E38F);
			assert (pack.alt_error_GET() == 2.7352338E38F);
			assert (pack.wp_dist_GET() == (char) 25796);
			assert (pack.target_bearing_GET() == (short) -24818);
			assert (pack.nav_roll_GET() == 2.7571084E38F);
			assert (pack.aspd_error_GET() == -3.2676903E38F);
			assert (pack.nav_bearing_GET() == (short) 32183);
		});
		NAV_CONTROLLER_OUTPUT p62 = new NAV_CONTROLLER_OUTPUT();
		PH.setPack(p62);
		p62.nav_bearing_SET((short) 32183);
		p62.target_bearing_SET((short) -24818);
		p62.wp_dist_SET((char) 25796);
		p62.xtrack_error_SET(3.126433E38F);
		p62.nav_roll_SET(2.7571084E38F);
		p62.alt_error_SET(2.7352338E38F);
		p62.aspd_error_SET(-3.2676903E38F);
		p62.nav_pitch_SET(2.385783E38F);
		TestChannel.instance.send(p62);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_GLOBAL_POSITION_INT_COV.add((src, ph, pack) ->
		{
			assert (pack.alt_GET() == 432844551);
			assert (pack.time_usec_GET() == 1365031749527923092L);
			assert (pack.relative_alt_GET() == 1346600307);
			assert (pack.vx_GET() == -1.9047663E38F);
			assert (pack.lon_GET() == -1327125836);
			assert (Arrays.equals(pack.covariance_GET(), new float[]{2.1177671E38F, -2.331997E38F, 1.1673259E38F, 1.6036974E37F, 3.369524E38F, 2.7998147E38F, -2.5639694E38F, 1.5387783E38F, 1.2800984E38F, 1.78781E38F, -1.8863215E38F, -2.555034E38F, -8.901167E37F, -1.9833708E38F, -2.3722813E38F, 1.1386206E38F, -3.1981744E38F, 1.9265784E38F, 2.6038712E37F, 2.371988E38F, -6.1355805E37F, 1.156976E38F, -1.1690159E38F, 3.690694E37F, 3.0474122E38F, -1.8723676E38F, -2.2844382E38F, 1.437288E38F, -1.5992275E38F, -1.1986952E38F, 2.1726134E38F, 2.998915E38F, -3.091352E38F, 2.9117372E38F, 6.5212906E37F, -1.8722885E38F}));
			assert (pack.estimator_type_GET() == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VIO);
			assert (pack.lat_GET() == 387134611);
			assert (pack.vy_GET() == -7.2289383E37F);
			assert (pack.vz_GET() == 3.164894E38F);
		});
		GLOBAL_POSITION_INT_COV p63 = new GLOBAL_POSITION_INT_COV();
		PH.setPack(p63);
		p63.vy_SET(-7.2289383E37F);
		p63.lat_SET(387134611);
		p63.alt_SET(432844551);
		p63.vx_SET(-1.9047663E38F);
		p63.relative_alt_SET(1346600307);
		p63.covariance_SET(new float[]{2.1177671E38F, -2.331997E38F, 1.1673259E38F, 1.6036974E37F, 3.369524E38F, 2.7998147E38F, -2.5639694E38F, 1.5387783E38F, 1.2800984E38F, 1.78781E38F, -1.8863215E38F, -2.555034E38F, -8.901167E37F, -1.9833708E38F, -2.3722813E38F, 1.1386206E38F, -3.1981744E38F, 1.9265784E38F, 2.6038712E37F, 2.371988E38F, -6.1355805E37F, 1.156976E38F, -1.1690159E38F, 3.690694E37F, 3.0474122E38F, -1.8723676E38F, -2.2844382E38F, 1.437288E38F, -1.5992275E38F, -1.1986952E38F, 2.1726134E38F, 2.998915E38F, -3.091352E38F, 2.9117372E38F, 6.5212906E37F, -1.8722885E38F}, 0);
		p63.vz_SET(3.164894E38F);
		p63.estimator_type_SET(MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VIO);
		p63.lon_SET(-1327125836);
		p63.time_usec_SET(1365031749527923092L);
		TestChannel.instance.send(p63);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_LOCAL_POSITION_NED_COV.add((src, ph, pack) ->
		{
			assert (pack.vy_GET() == 2.669417E38F);
			assert (pack.ay_GET() == -3.0646648E36F);
			assert (pack.ax_GET() == 2.803764E38F);
			assert (pack.z_GET() == -1.7886775E38F);
			assert (pack.vx_GET() == 1.795115E38F);
			assert (pack.y_GET() == -2.4748722E38F);
			assert (pack.time_usec_GET() == 7320575952785574887L);
			assert (pack.estimator_type_GET() == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VISION);
			assert (pack.az_GET() == 1.4679533E38F);
			assert (Arrays.equals(pack.covariance_GET(), new float[]{1.2886002E38F, 3.1478294E38F, 1.4159829E38F, -1.1385727E38F, -3.2861787E38F, 1.7143017E38F, -3.6117455E37F, -1.7844744E38F, -1.4616511E38F, 8.504163E37F, -3.2066439E38F, -2.3530345E38F, 2.5136546E38F, -5.8209836E37F, -3.351002E38F, -3.0107657E38F, -3.499508E37F, 2.564795E37F, 1.225512E38F, 1.4712702E38F, 3.0912324E37F, -2.630505E38F, -1.1733857E38F, -1.7812815E38F, 8.89083E37F, -2.6676066E38F, 5.4459214E36F, -2.1925832E38F, -2.664038E38F, 1.2700486E37F, -2.102242E38F, -2.25358E38F, 2.936297E38F, 2.8062198E38F, -2.8068167E38F, -4.513353E37F, -2.5620776E38F, -3.8268422E37F, -2.4098995E38F, 5.667182E37F, 5.951361E37F, -7.0067054E37F, 1.6636794E38F, 1.4338248E37F, 3.1763034E37F}));
			assert (pack.x_GET() == 5.846645E37F);
			assert (pack.vz_GET() == -1.9617838E38F);
		});
		LOCAL_POSITION_NED_COV p64 = new LOCAL_POSITION_NED_COV();
		PH.setPack(p64);
		p64.vx_SET(1.795115E38F);
		p64.estimator_type_SET(MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VISION);
		p64.x_SET(5.846645E37F);
		p64.ax_SET(2.803764E38F);
		p64.time_usec_SET(7320575952785574887L);
		p64.az_SET(1.4679533E38F);
		p64.covariance_SET(new float[]{1.2886002E38F, 3.1478294E38F, 1.4159829E38F, -1.1385727E38F, -3.2861787E38F, 1.7143017E38F, -3.6117455E37F, -1.7844744E38F, -1.4616511E38F, 8.504163E37F, -3.2066439E38F, -2.3530345E38F, 2.5136546E38F, -5.8209836E37F, -3.351002E38F, -3.0107657E38F, -3.499508E37F, 2.564795E37F, 1.225512E38F, 1.4712702E38F, 3.0912324E37F, -2.630505E38F, -1.1733857E38F, -1.7812815E38F, 8.89083E37F, -2.6676066E38F, 5.4459214E36F, -2.1925832E38F, -2.664038E38F, 1.2700486E37F, -2.102242E38F, -2.25358E38F, 2.936297E38F, 2.8062198E38F, -2.8068167E38F, -4.513353E37F, -2.5620776E38F, -3.8268422E37F, -2.4098995E38F, 5.667182E37F, 5.951361E37F, -7.0067054E37F, 1.6636794E38F, 1.4338248E37F, 3.1763034E37F}, 0);
		p64.y_SET(-2.4748722E38F);
		p64.z_SET(-1.7886775E38F);
		p64.vz_SET(-1.9617838E38F);
		p64.vy_SET(2.669417E38F);
		p64.ay_SET(-3.0646648E36F);
		TestChannel.instance.send(p64);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_RC_CHANNELS.add((src, ph, pack) ->
		{
			assert (pack.chan10_raw_GET() == (char) 11689);
			assert (pack.chan17_raw_GET() == (char) 13128);
			assert (pack.time_boot_ms_GET() == 3921110657L);
			assert (pack.chan6_raw_GET() == (char) 31133);
			assert (pack.chan2_raw_GET() == (char) 8888);
			assert (pack.chan11_raw_GET() == (char) 3757);
			assert (pack.chan14_raw_GET() == (char) 53709);
			assert (pack.chan18_raw_GET() == (char) 35520);
			assert (pack.chan8_raw_GET() == (char) 64325);
			assert (pack.chan7_raw_GET() == (char) 4783);
			assert (pack.chan3_raw_GET() == (char) 18769);
			assert (pack.chan16_raw_GET() == (char) 18673);
			assert (pack.chan13_raw_GET() == (char) 49660);
			assert (pack.chancount_GET() == (char) 28);
			assert (pack.chan4_raw_GET() == (char) 8225);
			assert (pack.chan1_raw_GET() == (char) 37213);
			assert (pack.chan5_raw_GET() == (char) 59097);
			assert (pack.rssi_GET() == (char) 63);
			assert (pack.chan12_raw_GET() == (char) 45604);
			assert (pack.chan9_raw_GET() == (char) 14231);
			assert (pack.chan15_raw_GET() == (char) 16044);
		});
		RC_CHANNELS p65 = new RC_CHANNELS();
		PH.setPack(p65);
		p65.chan3_raw_SET((char) 18769);
		p65.chan15_raw_SET((char) 16044);
		p65.rssi_SET((char) 63);
		p65.chan11_raw_SET((char) 3757);
		p65.chan10_raw_SET((char) 11689);
		p65.chan6_raw_SET((char) 31133);
		p65.chan8_raw_SET((char) 64325);
		p65.chan5_raw_SET((char) 59097);
		p65.chan12_raw_SET((char) 45604);
		p65.chan1_raw_SET((char) 37213);
		p65.chancount_SET((char) 28);
		p65.chan14_raw_SET((char) 53709);
		p65.chan9_raw_SET((char) 14231);
		p65.chan2_raw_SET((char) 8888);
		p65.chan4_raw_SET((char) 8225);
		p65.time_boot_ms_SET(3921110657L);
		p65.chan17_raw_SET((char) 13128);
		p65.chan16_raw_SET((char) 18673);
		p65.chan18_raw_SET((char) 35520);
		p65.chan7_raw_SET((char) 4783);
		p65.chan13_raw_SET((char) 49660);
		TestChannel.instance.send(p65);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_REQUEST_DATA_STREAM.add((src, ph, pack) ->
		{
			assert (pack.target_system_GET() == (char) 108);
			assert (pack.target_component_GET() == (char) 12);
			assert (pack.req_message_rate_GET() == (char) 5832);
			assert (pack.start_stop_GET() == (char) 204);
			assert (pack.req_stream_id_GET() == (char) 90);
		});
		REQUEST_DATA_STREAM p66 = new REQUEST_DATA_STREAM();
		PH.setPack(p66);
		p66.target_system_SET((char) 108);
		p66.target_component_SET((char) 12);
		p66.start_stop_SET((char) 204);
		p66.req_message_rate_SET((char) 5832);
		p66.req_stream_id_SET((char) 90);
		TestChannel.instance.send(p66);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_DATA_STREAM.add((src, ph, pack) ->
		{
			assert (pack.on_off_GET() == (char) 143);
			assert (pack.message_rate_GET() == (char) 17377);
			assert (pack.stream_id_GET() == (char) 80);
		});
		DATA_STREAM p67 = new DATA_STREAM();
		PH.setPack(p67);
		p67.stream_id_SET((char) 80);
		p67.message_rate_SET((char) 17377);
		p67.on_off_SET((char) 143);
		TestChannel.instance.send(p67);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_MANUAL_CONTROL.add((src, ph, pack) ->
		{
			assert (pack.buttons_GET() == (char) 42307);
			assert (pack.x_GET() == (short) 13169);
			assert (pack.y_GET() == (short) 9243);
			assert (pack.target_GET() == (char) 222);
			assert (pack.z_GET() == (short) 12879);
			assert (pack.r_GET() == (short) 25534);
		});
		MANUAL_CONTROL p69 = new MANUAL_CONTROL();
		PH.setPack(p69);
		p69.z_SET((short) 12879);
		p69.x_SET((short) 13169);
		p69.target_SET((char) 222);
		p69.buttons_SET((char) 42307);
		p69.y_SET((short) 9243);
		p69.r_SET((short) 25534);
		TestChannel.instance.send(p69);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_RC_CHANNELS_OVERRIDE.add((src, ph, pack) ->
		{
			assert (pack.chan5_raw_GET() == (char) 14878);
			assert (pack.target_system_GET() == (char) 184);
			assert (pack.chan6_raw_GET() == (char) 64077);
			assert (pack.chan3_raw_GET() == (char) 19545);
			assert (pack.chan8_raw_GET() == (char) 58453);
			assert (pack.chan2_raw_GET() == (char) 11948);
			assert (pack.chan7_raw_GET() == (char) 52177);
			assert (pack.target_component_GET() == (char) 21);
			assert (pack.chan4_raw_GET() == (char) 35684);
			assert (pack.chan1_raw_GET() == (char) 58933);
		});
		RC_CHANNELS_OVERRIDE p70 = new RC_CHANNELS_OVERRIDE();
		PH.setPack(p70);
		p70.chan5_raw_SET((char) 14878);
		p70.chan2_raw_SET((char) 11948);
		p70.target_component_SET((char) 21);
		p70.chan1_raw_SET((char) 58933);
		p70.chan6_raw_SET((char) 64077);
		p70.target_system_SET((char) 184);
		p70.chan8_raw_SET((char) 58453);
		p70.chan7_raw_SET((char) 52177);
		p70.chan4_raw_SET((char) 35684);
		p70.chan3_raw_SET((char) 19545);
		TestChannel.instance.send(p70);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_MISSION_ITEM_INT.add((src, ph, pack) ->
		{
			assert (pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT);
			assert (pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
			assert (pack.autocontinue_GET() == (char) 14);
			assert (pack.param2_GET() == 3.1132426E38F);
			assert (pack.param4_GET() == -2.841691E38F);
			assert (pack.target_component_GET() == (char) 13);
			assert (pack.param1_GET() == 3.9003644E37F);
			assert (pack.param3_GET() == -1.3701924E38F);
			assert (pack.y_GET() == -166215055);
			assert (pack.command_GET() == MAV_CMD.MAV_CMD_UAVCAN_GET_NODE_INFO);
			assert (pack.x_GET() == 323783090);
			assert (pack.seq_GET() == (char) 1800);
			assert (pack.current_GET() == (char) 29);
			assert (pack.target_system_GET() == (char) 5);
			assert (pack.z_GET() == -2.5277121E38F);
		});
		MISSION_ITEM_INT p73 = new MISSION_ITEM_INT();
		PH.setPack(p73);
		p73.y_SET(-166215055);
		p73.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT);
		p73.target_component_SET((char) 13);
		p73.x_SET(323783090);
		p73.param4_SET(-2.841691E38F);
		p73.z_SET(-2.5277121E38F);
		p73.autocontinue_SET((char) 14);
		p73.param2_SET(3.1132426E38F);
		p73.target_system_SET((char) 5);
		p73.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
		p73.seq_SET((char) 1800);
		p73.param3_SET(-1.3701924E38F);
		p73.param1_SET(3.9003644E37F);
		p73.current_SET((char) 29);
		p73.command_SET(MAV_CMD.MAV_CMD_UAVCAN_GET_NODE_INFO);
		TestChannel.instance.send(p73);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_VFR_HUD.add((src, ph, pack) ->
		{
			assert (pack.climb_GET() == -7.0712263E37F);
			assert (pack.alt_GET() == 2.3657445E38F);
			assert (pack.groundspeed_GET() == -2.2721317E38F);
			assert (pack.heading_GET() == (short) 28907);
			assert (pack.throttle_GET() == (char) 40380);
			assert (pack.airspeed_GET() == -2.5496741E38F);
		});
		VFR_HUD p74 = new VFR_HUD();
		PH.setPack(p74);
		p74.groundspeed_SET(-2.2721317E38F);
		p74.climb_SET(-7.0712263E37F);
		p74.alt_SET(2.3657445E38F);
		p74.throttle_SET((char) 40380);
		p74.heading_SET((short) 28907);
		p74.airspeed_SET(-2.5496741E38F);
		TestChannel.instance.send(p74);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_COMMAND_INT.add((src, ph, pack) ->
		{
			assert (pack.param4_GET() == -3.1135085E38F);
			assert (pack.param1_GET() == 6.854899E37F);
			assert (pack.target_component_GET() == (char) 137);
			assert (pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT);
			assert (pack.target_system_GET() == (char) 7);
			assert (pack.x_GET() == 767932531);
			assert (pack.param2_GET() == -1.627536E37F);
			assert (pack.param3_GET() == -1.5142907E37F);
			assert (pack.y_GET() == -747835617);
			assert (pack.z_GET() == 2.4464403E38F);
			assert (pack.autocontinue_GET() == (char) 187);
			assert (pack.command_GET() == MAV_CMD.MAV_CMD_DO_INVERTED_FLIGHT);
			assert (pack.current_GET() == (char) 172);
		});
		COMMAND_INT p75 = new COMMAND_INT();
		PH.setPack(p75);
		p75.target_system_SET((char) 7);
		p75.command_SET(MAV_CMD.MAV_CMD_DO_INVERTED_FLIGHT);
		p75.z_SET(2.4464403E38F);
		p75.autocontinue_SET((char) 187);
		p75.y_SET(-747835617);
		p75.param4_SET(-3.1135085E38F);
		p75.param1_SET(6.854899E37F);
		p75.param3_SET(-1.5142907E37F);
		p75.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT);
		p75.param2_SET(-1.627536E37F);
		p75.target_component_SET((char) 137);
		p75.current_SET((char) 172);
		p75.x_SET(767932531);
		TestChannel.instance.send(p75);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_COMMAND_LONG.add((src, ph, pack) ->
		{
			assert (pack.command_GET() == MAV_CMD.MAV_CMD_SET_MESSAGE_INTERVAL);
			assert (pack.param7_GET() == -2.9584582E38F);
			assert (pack.param2_GET() == 2.1937448E38F);
			assert (pack.param4_GET() == -1.0467652E38F);
			assert (pack.param6_GET() == -2.6705267E38F);
			assert (pack.target_component_GET() == (char) 182);
			assert (pack.param1_GET() == -2.5594186E38F);
			assert (pack.param5_GET() == -9.036226E37F);
			assert (pack.param3_GET() == -1.5200256E38F);
			assert (pack.confirmation_GET() == (char) 106);
			assert (pack.target_system_GET() == (char) 104);
		});
		COMMAND_LONG p76 = new COMMAND_LONG();
		PH.setPack(p76);
		p76.param5_SET(-9.036226E37F);
		p76.target_component_SET((char) 182);
		p76.param4_SET(-1.0467652E38F);
		p76.param2_SET(2.1937448E38F);
		p76.param6_SET(-2.6705267E38F);
		p76.command_SET(MAV_CMD.MAV_CMD_SET_MESSAGE_INTERVAL);
		p76.param3_SET(-1.5200256E38F);
		p76.param1_SET(-2.5594186E38F);
		p76.target_system_SET((char) 104);
		p76.param7_SET(-2.9584582E38F);
		p76.confirmation_SET((char) 106);
		TestChannel.instance.send(p76);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_COMMAND_ACK.add((src, ph, pack) ->
		{
			assert (pack.result_GET() == MAV_RESULT.MAV_RESULT_ACCEPTED);
			assert (pack.target_system_TRY(ph) == (char) 20);
			assert (pack.result_param2_TRY(ph) == 37078907);
			assert (pack.progress_TRY(ph) == (char) 133);
			assert (pack.target_component_TRY(ph) == (char) 220);
			assert (pack.command_GET() == MAV_CMD.MAV_CMD_PREFLIGHT_STORAGE);
		});
		COMMAND_ACK p77 = new COMMAND_ACK();
		PH.setPack(p77);
		p77.target_system_SET((char) 20, PH);
		p77.progress_SET((char) 133, PH);
		p77.result_SET(MAV_RESULT.MAV_RESULT_ACCEPTED);
		p77.target_component_SET((char) 220, PH);
		p77.command_SET(MAV_CMD.MAV_CMD_PREFLIGHT_STORAGE);
		p77.result_param2_SET(37078907, PH);
		TestChannel.instance.send(p77);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_MANUAL_SETPOINT.add((src, ph, pack) ->
		{
			assert (pack.thrust_GET() == -2.6162956E38F);
			assert (pack.mode_switch_GET() == (char) 140);
			assert (pack.roll_GET() == 1.2588597E38F);
			assert (pack.manual_override_switch_GET() == (char) 249);
			assert (pack.yaw_GET() == 1.2313655E38F);
			assert (pack.time_boot_ms_GET() == 2848318608L);
			assert (pack.pitch_GET() == 4.73684E37F);
		});
		MANUAL_SETPOINT p81 = new MANUAL_SETPOINT();
		PH.setPack(p81);
		p81.pitch_SET(4.73684E37F);
		p81.roll_SET(1.2588597E38F);
		p81.manual_override_switch_SET((char) 249);
		p81.thrust_SET(-2.6162956E38F);
		p81.yaw_SET(1.2313655E38F);
		p81.time_boot_ms_SET(2848318608L);
		p81.mode_switch_SET((char) 140);
		TestChannel.instance.send(p81);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_SET_ATTITUDE_TARGET.add((src, ph, pack) ->
		{
			assert (pack.body_roll_rate_GET() == -2.8133E38F);
			assert (Arrays.equals(pack.q_GET(), new float[]{1.5120565E38F, 1.5522928E38F, -2.748707E37F, 1.6458711E38F}));
			assert (pack.type_mask_GET() == (char) 83);
			assert (pack.target_component_GET() == (char) 121);
			assert (pack.target_system_GET() == (char) 51);
			assert (pack.thrust_GET() == -3.149569E38F);
			assert (pack.body_pitch_rate_GET() == 2.57017E38F);
			assert (pack.body_yaw_rate_GET() == -2.6378412E38F);
			assert (pack.time_boot_ms_GET() == 2137447055L);
		});
		SET_ATTITUDE_TARGET p82 = new SET_ATTITUDE_TARGET();
		PH.setPack(p82);
		p82.body_pitch_rate_SET(2.57017E38F);
		p82.target_system_SET((char) 51);
		p82.thrust_SET(-3.149569E38F);
		p82.target_component_SET((char) 121);
		p82.body_roll_rate_SET(-2.8133E38F);
		p82.type_mask_SET((char) 83);
		p82.body_yaw_rate_SET(-2.6378412E38F);
		p82.q_SET(new float[]{1.5120565E38F, 1.5522928E38F, -2.748707E37F, 1.6458711E38F}, 0);
		p82.time_boot_ms_SET(2137447055L);
		TestChannel.instance.send(p82);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_ATTITUDE_TARGET.add((src, ph, pack) ->
		{
			assert (pack.thrust_GET() == -1.4362806E38F);
			assert (pack.type_mask_GET() == (char) 66);
			assert (pack.body_roll_rate_GET() == -1.8891373E38F);
			assert (pack.body_yaw_rate_GET() == -8.715733E37F);
			assert (Arrays.equals(pack.q_GET(), new float[]{3.558506E37F, -1.6068428E38F, 6.72603E37F, 3.2245792E38F}));
			assert (pack.time_boot_ms_GET() == 826283027L);
			assert (pack.body_pitch_rate_GET() == 1.9725505E38F);
		});
		ATTITUDE_TARGET p83 = new ATTITUDE_TARGET();
		PH.setPack(p83);
		p83.time_boot_ms_SET(826283027L);
		p83.body_pitch_rate_SET(1.9725505E38F);
		p83.body_roll_rate_SET(-1.8891373E38F);
		p83.thrust_SET(-1.4362806E38F);
		p83.body_yaw_rate_SET(-8.715733E37F);
		p83.q_SET(new float[]{3.558506E37F, -1.6068428E38F, 6.72603E37F, 3.2245792E38F}, 0);
		p83.type_mask_SET((char) 66);
		TestChannel.instance.send(p83);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_SET_POSITION_TARGET_LOCAL_NED.add((src, ph, pack) ->
		{
			assert (pack.yaw_rate_GET() == 3.635434E37F);
			assert (pack.afy_GET() == 3.2454119E38F);
			assert (pack.target_component_GET() == (char) 34);
			assert (pack.y_GET() == -2.7392047E38F);
			assert (pack.z_GET() == -1.4361683E38F);
			assert (pack.vy_GET() == 2.6088284E38F);
			assert (pack.vx_GET() == 1.6501628E37F);
			assert (pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT);
			assert (pack.target_system_GET() == (char) 176);
			assert (pack.yaw_GET() == -9.04933E37F);
			assert (pack.vz_GET() == -1.4732233E38F);
			assert (pack.time_boot_ms_GET() == 1408442919L);
			assert (pack.type_mask_GET() == (char) 57863);
			assert (pack.x_GET() == -6.643494E37F);
			assert (pack.afx_GET() == -7.938655E37F);
			assert (pack.afz_GET() == -1.9741174E38F);
		});
		SET_POSITION_TARGET_LOCAL_NED p84 = new SET_POSITION_TARGET_LOCAL_NED();
		PH.setPack(p84);
		p84.vy_SET(2.6088284E38F);
		p84.afz_SET(-1.9741174E38F);
		p84.afx_SET(-7.938655E37F);
		p84.vz_SET(-1.4732233E38F);
		p84.type_mask_SET((char) 57863);
		p84.target_component_SET((char) 34);
		p84.vx_SET(1.6501628E37F);
		p84.x_SET(-6.643494E37F);
		p84.yaw_SET(-9.04933E37F);
		p84.z_SET(-1.4361683E38F);
		p84.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT);
		p84.time_boot_ms_SET(1408442919L);
		p84.y_SET(-2.7392047E38F);
		p84.yaw_rate_SET(3.635434E37F);
		p84.afy_SET(3.2454119E38F);
		p84.target_system_SET((char) 176);
		TestChannel.instance.send(p84);//put test pack to the  channel send buffer
		TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_SET_POSITION_TARGET_GLOBAL_INT.add((src, ph, pack) ->
		{
			assert (pack.alt_GET() == 4.5703373E37F);
			assert (pack.target_system_GET() == (char) 195);
			assert (pack.vz_GET() == 1.6480352E38F);
			assert (pack.lon_int_GET() == 1948378496);
			assert (pack.afz_GET() == -2.9553162E38F);
			assert (pack.afx_GET() == 1.929305E38F);
			assert (pack.yaw_GET() == 2.6416212E38F);
			assert (pack.type_mask_GET() == (char) 58508);
			assert (pack.vx_GET() == 7.981477E37F);
			assert (pack.time_boot_ms_GET() == 892346285L);
			assert (pack.vy_GET() == 8.553265E37F);
			assert (pack.yaw_rate_GET() == 8.903115E37F);
			assert (pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL);
			assert (pack.target_component_GET() == (char) 141);
			assert (pack.lat_int_GET() == -203075240);
			assert (pack.afy_GET() == 9.907087E37F);
		});
		GroundControl.SET_POSITION_TARGET_GLOBAL_INT p86 = CommunicationChannel.new_SET_POSITION_TARGET_GLOBAL_INT();
		PH.setPack(p86);
		p86.afy_SET(9.907087E37F);
		p86.yaw_rate_SET(8.903115E37F);
		p86.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL);
		p86.target_system_SET((char) 195);
		p86.vx_SET(7.981477E37F);
		p86.afz_SET(-2.9553162E38F);
		p86.vy_SET(8.553265E37F);
		p86.afx_SET(1.929305E38F);
		p86.vz_SET(1.6480352E38F);
		p86.target_component_SET((char) 141);
		p86.lon_int_SET(1948378496);
		p86.yaw_SET(2.6416212E38F);
		p86.lat_int_SET(-203075240);
		p86.type_mask_SET((char) 58508);
		p86.time_boot_ms_SET(892346285L);
		p86.alt_SET(4.5703373E37F);
		CommunicationChannel.instance.send(p86);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_POSITION_TARGET_GLOBAL_INT.add((src, ph, pack) ->
		{
			assert (pack.alt_GET() == -1.7674548E38F);
			assert (pack.afx_GET() == -2.4520944E38F);
			assert (pack.lat_int_GET() == -2134659239);
			assert (pack.time_boot_ms_GET() == 3329302574L);
			assert (pack.vz_GET() == 6.5445484E37F);
			assert (pack.afz_GET() == 2.1039956E38F);
			assert (pack.vx_GET() == -2.096689E38F);
			assert (pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT);
			assert (pack.vy_GET() == -1.8330303E38F);
			assert (pack.yaw_rate_GET() == 1.8406986E38F);
			assert (pack.type_mask_GET() == (char) 44984);
			assert (pack.afy_GET() == -2.5803645E38F);
			assert (pack.yaw_GET() == 2.0888004E38F);
			assert (pack.lon_int_GET() == 836385464);
		});
		GroundControl.POSITION_TARGET_GLOBAL_INT p87 = CommunicationChannel.new_POSITION_TARGET_GLOBAL_INT();
		PH.setPack(p87);
		p87.yaw_SET(2.0888004E38F);
		p87.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT);
		p87.lon_int_SET(836385464);
		p87.vy_SET(-1.8330303E38F);
		p87.yaw_rate_SET(1.8406986E38F);
		p87.vz_SET(6.5445484E37F);
		p87.type_mask_SET((char) 44984);
		p87.alt_SET(-1.7674548E38F);
		p87.afx_SET(-2.4520944E38F);
		p87.afy_SET(-2.5803645E38F);
		p87.afz_SET(2.1039956E38F);
		p87.lat_int_SET(-2134659239);
		p87.time_boot_ms_SET(3329302574L);
		p87.vx_SET(-2.096689E38F);
		CommunicationChannel.instance.send(p87);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET.add((src, ph, pack) ->
		{
			assert (pack.y_GET() == 2.767853E38F);
			assert (pack.x_GET() == -1.4351312E38F);
			assert (pack.pitch_GET() == -2.1253347E38F);
			assert (pack.z_GET() == -8.859877E37F);
			assert (pack.yaw_GET() == 1.5358007E38F);
			assert (pack.time_boot_ms_GET() == 1307993176L);
			assert (pack.roll_GET() == 2.7778934E38F);
		});
		GroundControl.LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET p89 = CommunicationChannel.new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
		PH.setPack(p89);
		p89.roll_SET(2.7778934E38F);
		p89.time_boot_ms_SET(1307993176L);
		p89.yaw_SET(1.5358007E38F);
		p89.y_SET(2.767853E38F);
		p89.pitch_SET(-2.1253347E38F);
		p89.z_SET(-8.859877E37F);
		p89.x_SET(-1.4351312E38F);
		CommunicationChannel.instance.send(p89);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_HIL_STATE.add((src, ph, pack) ->
		{
			assert (pack.zacc_GET() == (short) -11035);
			assert (pack.vx_GET() == (short) 18434);
			assert (pack.time_usec_GET() == 997829297029935435L);
			assert (pack.xacc_GET() == (short) -12922);
			assert (pack.pitchspeed_GET() == -1.729414E38F);
			assert (pack.rollspeed_GET() == 2.8850598E38F);
			assert (pack.alt_GET() == -2144131033);
			assert (pack.vz_GET() == (short) -27788);
			assert (pack.yaw_GET() == 3.1736282E38F);
			assert (pack.roll_GET() == 1.8425882E38F);
			assert (pack.vy_GET() == (short) 18575);
			assert (pack.pitch_GET() == 2.88563E38F);
			assert (pack.yawspeed_GET() == 3.0356512E38F);
			assert (pack.lon_GET() == -533955913);
			assert (pack.yacc_GET() == (short) 4102);
			assert (pack.lat_GET() == -1990124065);
		});
		GroundControl.HIL_STATE p90 = CommunicationChannel.new_HIL_STATE();
		PH.setPack(p90);
		p90.lat_SET(-1990124065);
		p90.vz_SET((short) -27788);
		p90.vx_SET((short) 18434);
		p90.rollspeed_SET(2.8850598E38F);
		p90.vy_SET((short) 18575);
		p90.lon_SET(-533955913);
		p90.roll_SET(1.8425882E38F);
		p90.yacc_SET((short) 4102);
		p90.yaw_SET(3.1736282E38F);
		p90.yawspeed_SET(3.0356512E38F);
		p90.zacc_SET((short) -11035);
		p90.alt_SET(-2144131033);
		p90.pitchspeed_SET(-1.729414E38F);
		p90.xacc_SET((short) -12922);
		p90.time_usec_SET(997829297029935435L);
		p90.pitch_SET(2.88563E38F);
		CommunicationChannel.instance.send(p90);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_HIL_CONTROLS.add((src, ph, pack) ->
		{
			assert (pack.yaw_rudder_GET() == -1.6993267E38F);
			assert (pack.pitch_elevator_GET() == -2.9712387E38F);
			assert (pack.mode_GET() == MAV_MODE.MAV_MODE_GUIDED_ARMED);
			assert (pack.nav_mode_GET() == (char) 104);
			assert (pack.aux1_GET() == 3.1586048E38F);
			assert (pack.time_usec_GET() == 3233069378166820140L);
			assert (pack.aux2_GET() == 1.6976272E38F);
			assert (pack.aux4_GET() == -1.7239649E38F);
			assert (pack.aux3_GET() == -1.8097791E38F);
			assert (pack.throttle_GET() == 2.1088214E38F);
			assert (pack.roll_ailerons_GET() == 1.5036144E38F);
		});
		GroundControl.HIL_CONTROLS p91 = CommunicationChannel.new_HIL_CONTROLS();
		PH.setPack(p91);
		p91.time_usec_SET(3233069378166820140L);
		p91.throttle_SET(2.1088214E38F);
		p91.aux1_SET(3.1586048E38F);
		p91.aux2_SET(1.6976272E38F);
		p91.aux3_SET(-1.8097791E38F);
		p91.aux4_SET(-1.7239649E38F);
		p91.roll_ailerons_SET(1.5036144E38F);
		p91.nav_mode_SET((char) 104);
		p91.pitch_elevator_SET(-2.9712387E38F);
		p91.yaw_rudder_SET(-1.6993267E38F);
		p91.mode_SET(MAV_MODE.MAV_MODE_GUIDED_ARMED);
		CommunicationChannel.instance.send(p91);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_HIL_RC_INPUTS_RAW.add((src, ph, pack) ->
		{
			assert (pack.chan4_raw_GET() == (char) 49829);
			assert (pack.chan5_raw_GET() == (char) 19910);
			assert (pack.chan3_raw_GET() == (char) 62258);
			assert (pack.chan8_raw_GET() == (char) 53756);
			assert (pack.chan1_raw_GET() == (char) 18971);
			assert (pack.chan10_raw_GET() == (char) 53856);
			assert (pack.chan7_raw_GET() == (char) 34182);
			assert (pack.chan9_raw_GET() == (char) 46587);
			assert (pack.rssi_GET() == (char) 140);
			assert (pack.chan12_raw_GET() == (char) 35571);
			assert (pack.chan11_raw_GET() == (char) 70);
			assert (pack.time_usec_GET() == 5249089684187995051L);
			assert (pack.chan6_raw_GET() == (char) 43413);
			assert (pack.chan2_raw_GET() == (char) 19959);
		});
		GroundControl.HIL_RC_INPUTS_RAW p92 = CommunicationChannel.new_HIL_RC_INPUTS_RAW();
		PH.setPack(p92);
		p92.chan2_raw_SET((char) 19959);
		p92.chan1_raw_SET((char) 18971);
		p92.chan12_raw_SET((char) 35571);
		p92.chan7_raw_SET((char) 34182);
		p92.chan10_raw_SET((char) 53856);
		p92.chan5_raw_SET((char) 19910);
		p92.time_usec_SET(5249089684187995051L);
		p92.chan8_raw_SET((char) 53756);
		p92.chan11_raw_SET((char) 70);
		p92.chan4_raw_SET((char) 49829);
		p92.chan3_raw_SET((char) 62258);
		p92.chan6_raw_SET((char) 43413);
		p92.chan9_raw_SET((char) 46587);
		p92.rssi_SET((char) 140);
		CommunicationChannel.instance.send(p92);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_HIL_ACTUATOR_CONTROLS.add((src, ph, pack) ->
		{
			assert (Arrays.equals(pack.controls_GET(), new float[]{2.2168185E38F, 6.890904E37F, 5.985676E37F, 4.0643145E37F, -3.2859546E38F, 2.3239682E38F, -1.4119649E38F, -1.7344955E38F, -3.0833542E38F, -3.8610406E37F, 2.1677036E38F, -9.101393E37F, -8.659984E37F, -2.7810214E38F, 1.6326593E37F, -1.872385E38F}));
			assert (pack.time_usec_GET() == 460835044655503592L);
			assert (pack.mode_GET() == MAV_MODE.MAV_MODE_MANUAL_ARMED);
			assert (pack.flags_GET() == 3976449725018593021L);
		});
		GroundControl.HIL_ACTUATOR_CONTROLS p93 = CommunicationChannel.new_HIL_ACTUATOR_CONTROLS();
		PH.setPack(p93);
		p93.time_usec_SET(460835044655503592L);
		p93.controls_SET(new float[]{2.2168185E38F, 6.890904E37F, 5.985676E37F, 4.0643145E37F, -3.2859546E38F, 2.3239682E38F, -1.4119649E38F, -1.7344955E38F, -3.0833542E38F, -3.8610406E37F, 2.1677036E38F, -9.101393E37F, -8.659984E37F, -2.7810214E38F, 1.6326593E37F, -1.872385E38F}, 0);
		p93.flags_SET(3976449725018593021L);
		p93.mode_SET(MAV_MODE.MAV_MODE_MANUAL_ARMED);
		CommunicationChannel.instance.send(p93);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_OPTICAL_FLOW.add((src, ph, pack) ->
		{
			assert (pack.flow_rate_y_TRY(ph) == 1.342391E38F);
			assert (pack.quality_GET() == (char) 34);
			assert (pack.time_usec_GET() == 6534741769014316928L);
			assert (pack.flow_comp_m_y_GET() == -1.8603393E38F);
			assert (pack.flow_comp_m_x_GET() == -2.1097767E38F);
			assert (pack.ground_distance_GET() == -5.248888E37F);
			assert (pack.sensor_id_GET() == (char) 248);
			assert (pack.flow_x_GET() == (short) 31549);
			assert (pack.flow_rate_x_TRY(ph) == -2.793358E38F);
			assert (pack.flow_y_GET() == (short) 11731);
		});
		GroundControl.OPTICAL_FLOW p100 = CommunicationChannel.new_OPTICAL_FLOW();
		PH.setPack(p100);
		p100.flow_rate_x_SET(-2.793358E38F, PH);
		p100.flow_comp_m_x_SET(-2.1097767E38F);
		p100.time_usec_SET(6534741769014316928L);
		p100.flow_y_SET((short) 11731);
		p100.quality_SET((char) 34);
		p100.sensor_id_SET((char) 248);
		p100.ground_distance_SET(-5.248888E37F);
		p100.flow_rate_y_SET(1.342391E38F, PH);
		p100.flow_comp_m_y_SET(-1.8603393E38F);
		p100.flow_x_SET((short) 31549);
		CommunicationChannel.instance.send(p100);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_GLOBAL_VISION_POSITION_ESTIMATE.add((src, ph, pack) ->
		{
			assert (pack.yaw_GET() == -1.6337741E38F);
			assert (pack.x_GET() == -3.2712251E38F);
			assert (pack.z_GET() == -3.3719845E38F);
			assert (pack.y_GET() == -1.7554667E38F);
			assert (pack.roll_GET() == 1.1074788E38F);
			assert (pack.usec_GET() == 5618102178592779022L);
			assert (pack.pitch_GET() == 8.626033E37F);
		});
		GroundControl.GLOBAL_VISION_POSITION_ESTIMATE p101 = CommunicationChannel.new_GLOBAL_VISION_POSITION_ESTIMATE();
		PH.setPack(p101);
		p101.yaw_SET(-1.6337741E38F);
		p101.pitch_SET(8.626033E37F);
		p101.usec_SET(5618102178592779022L);
		p101.roll_SET(1.1074788E38F);
		p101.x_SET(-3.2712251E38F);
		p101.y_SET(-1.7554667E38F);
		p101.z_SET(-3.3719845E38F);
		CommunicationChannel.instance.send(p101);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_VISION_POSITION_ESTIMATE.add((src, ph, pack) ->
		{
			assert (pack.x_GET() == -1.2082398E38F);
			assert (pack.pitch_GET() == -2.866581E37F);
			assert (pack.yaw_GET() == 2.6653763E38F);
			assert (pack.y_GET() == 3.0409881E38F);
			assert (pack.usec_GET() == 1283681443106539252L);
			assert (pack.roll_GET() == 1.5948964E38F);
			assert (pack.z_GET() == -2.6571776E38F);
		});
		GroundControl.VISION_POSITION_ESTIMATE p102 = CommunicationChannel.new_VISION_POSITION_ESTIMATE();
		PH.setPack(p102);
		p102.yaw_SET(2.6653763E38F);
		p102.y_SET(3.0409881E38F);
		p102.pitch_SET(-2.866581E37F);
		p102.x_SET(-1.2082398E38F);
		p102.usec_SET(1283681443106539252L);
		p102.z_SET(-2.6571776E38F);
		p102.roll_SET(1.5948964E38F);
		CommunicationChannel.instance.send(p102);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_VISION_SPEED_ESTIMATE.add((src, ph, pack) ->
		{
			assert (pack.y_GET() == 1.4147275E38F);
			assert (pack.x_GET() == -1.0031695E38F);
			assert (pack.z_GET() == -4.83133E37F);
			assert (pack.usec_GET() == 4512394692943349071L);
		});
		GroundControl.VISION_SPEED_ESTIMATE p103 = CommunicationChannel.new_VISION_SPEED_ESTIMATE();
		PH.setPack(p103);
		p103.z_SET(-4.83133E37F);
		p103.y_SET(1.4147275E38F);
		p103.x_SET(-1.0031695E38F);
		p103.usec_SET(4512394692943349071L);
		CommunicationChannel.instance.send(p103);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_VICON_POSITION_ESTIMATE.add((src, ph, pack) ->
		{
			assert (pack.pitch_GET() == -2.8547398E38F);
			assert (pack.y_GET() == -1.1479962E38F);
			assert (pack.roll_GET() == -3.4796253E37F);
			assert (pack.x_GET() == 4.9943426E37F);
			assert (pack.yaw_GET() == 1.9735675E38F);
			assert (pack.usec_GET() == 6562822843036038434L);
			assert (pack.z_GET() == 1.1474114E38F);
		});
		GroundControl.VICON_POSITION_ESTIMATE p104 = CommunicationChannel.new_VICON_POSITION_ESTIMATE();
		PH.setPack(p104);
		p104.x_SET(4.9943426E37F);
		p104.yaw_SET(1.9735675E38F);
		p104.roll_SET(-3.4796253E37F);
		p104.pitch_SET(-2.8547398E38F);
		p104.usec_SET(6562822843036038434L);
		p104.z_SET(1.1474114E38F);
		p104.y_SET(-1.1479962E38F);
		CommunicationChannel.instance.send(p104);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_HIGHRES_IMU.add((src, ph, pack) ->
		{
			assert (pack.zmag_GET() == -3.3114812E38F);
			assert (pack.temperature_GET() == -3.2013161E38F);
			assert (pack.ygyro_GET() == -1.0047144E38F);
			assert (pack.fields_updated_GET() == (char) 51612);
			assert (pack.yacc_GET() == -3.3232986E38F);
			assert (pack.pressure_alt_GET() == 1.8089936E38F);
			assert (pack.xacc_GET() == 2.7629491E38F);
			assert (pack.ymag_GET() == 9.775448E37F);
			assert (pack.zacc_GET() == 1.4010097E38F);
			assert (pack.abs_pressure_GET() == 1.2791411E38F);
			assert (pack.xmag_GET() == 1.107118E38F);
			assert (pack.zgyro_GET() == -2.387026E38F);
			assert (pack.xgyro_GET() == 2.773231E38F);
			assert (pack.diff_pressure_GET() == 3.13838E38F);
			assert (pack.time_usec_GET() == 3577652430168497319L);
		});
		GroundControl.HIGHRES_IMU p105 = CommunicationChannel.new_HIGHRES_IMU();
		PH.setPack(p105);
		p105.ymag_SET(9.775448E37F);
		p105.ygyro_SET(-1.0047144E38F);
		p105.temperature_SET(-3.2013161E38F);
		p105.time_usec_SET(3577652430168497319L);
		p105.diff_pressure_SET(3.13838E38F);
		p105.yacc_SET(-3.3232986E38F);
		p105.xgyro_SET(2.773231E38F);
		p105.zacc_SET(1.4010097E38F);
		p105.abs_pressure_SET(1.2791411E38F);
		p105.zgyro_SET(-2.387026E38F);
		p105.pressure_alt_SET(1.8089936E38F);
		p105.zmag_SET(-3.3114812E38F);
		p105.xacc_SET(2.7629491E38F);
		p105.xmag_SET(1.107118E38F);
		p105.fields_updated_SET((char) 51612);
		CommunicationChannel.instance.send(p105);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_OPTICAL_FLOW_RAD.add((src, ph, pack) ->
		{
			assert (pack.sensor_id_GET() == (char) 158);
			assert (pack.distance_GET() == 3.6234847E37F);
			assert (pack.integration_time_us_GET() == 1082497131L);
			assert (pack.time_usec_GET() == 4084901300359457661L);
			assert (pack.integrated_ygyro_GET() == 2.923402E38F);
			assert (pack.integrated_x_GET() == 2.424913E38F);
			assert (pack.integrated_xgyro_GET() == 4.518562E37F);
			assert (pack.integrated_y_GET() == 5.4443697E37F);
			assert (pack.temperature_GET() == (short) -6386);
			assert (pack.time_delta_distance_us_GET() == 2469239423L);
			assert (pack.quality_GET() == (char) 80);
			assert (pack.integrated_zgyro_GET() == -2.3875643E38F);
		});
		GroundControl.OPTICAL_FLOW_RAD p106 = CommunicationChannel.new_OPTICAL_FLOW_RAD();
		PH.setPack(p106);
		p106.integration_time_us_SET(1082497131L);
		p106.time_delta_distance_us_SET(2469239423L);
		p106.integrated_xgyro_SET(4.518562E37F);
		p106.temperature_SET((short) -6386);
		p106.distance_SET(3.6234847E37F);
		p106.integrated_zgyro_SET(-2.3875643E38F);
		p106.time_usec_SET(4084901300359457661L);
		p106.integrated_ygyro_SET(2.923402E38F);
		p106.integrated_y_SET(5.4443697E37F);
		p106.quality_SET((char) 80);
		p106.integrated_x_SET(2.424913E38F);
		p106.sensor_id_SET((char) 158);
		CommunicationChannel.instance.send(p106);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_HIL_SENSOR.add((src, ph, pack) ->
		{
			assert (pack.ygyro_GET() == -2.2399051E38F);
			assert (pack.pressure_alt_GET() == -1.6512871E38F);
			assert (pack.zmag_GET() == 1.0263966E38F);
			assert (pack.diff_pressure_GET() == 2.6375494E37F);
			assert (pack.zgyro_GET() == 2.1641578E38F);
			assert (pack.xmag_GET() == 9.075009E37F);
			assert (pack.yacc_GET() == 8.3392065E37F);
			assert (pack.ymag_GET() == 2.0018296E38F);
			assert (pack.xacc_GET() == 5.385573E37F);
			assert (pack.zacc_GET() == 7.7755857E37F);
			assert (pack.temperature_GET() == -3.1876606E38F);
			assert (pack.time_usec_GET() == 6796812206414570554L);
			assert (pack.abs_pressure_GET() == -1.4200357E38F);
			assert (pack.xgyro_GET() == 2.4490164E38F);
			assert (pack.fields_updated_GET() == 3373669324L);
		});
		GroundControl.HIL_SENSOR p107 = CommunicationChannel.new_HIL_SENSOR();
		PH.setPack(p107);
		p107.abs_pressure_SET(-1.4200357E38F);
		p107.fields_updated_SET(3373669324L);
		p107.diff_pressure_SET(2.6375494E37F);
		p107.yacc_SET(8.3392065E37F);
		p107.time_usec_SET(6796812206414570554L);
		p107.zacc_SET(7.7755857E37F);
		p107.ygyro_SET(-2.2399051E38F);
		p107.xmag_SET(9.075009E37F);
		p107.xacc_SET(5.385573E37F);
		p107.zmag_SET(1.0263966E38F);
		p107.zgyro_SET(2.1641578E38F);
		p107.temperature_SET(-3.1876606E38F);
		p107.xgyro_SET(2.4490164E38F);
		p107.pressure_alt_SET(-1.6512871E38F);
		p107.ymag_SET(2.0018296E38F);
		CommunicationChannel.instance.send(p107);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_SIM_STATE.add((src, ph, pack) ->
		{
			assert (pack.alt_GET() == 2.323707E38F);
			assert (pack.yaw_GET() == 1.0030912E38F);
			assert (pack.ve_GET() == -8.670161E37F);
			assert (pack.lat_GET() == -3.9592432E37F);
			assert (pack.xgyro_GET() == -3.3455952E38F);
			assert (pack.vd_GET() == -3.2508601E38F);
			assert (pack.std_dev_vert_GET() == -2.4995311E38F);
			assert (pack.zacc_GET() == 1.9891632E38F);
			assert (pack.q3_GET() == -9.939602E37F);
			assert (pack.vn_GET() == -2.4518819E38F);
			assert (pack.roll_GET() == -1.7096735E38F);
			assert (pack.q2_GET() == 5.0975816E37F);
			assert (pack.xacc_GET() == -2.3110562E38F);
			assert (pack.zgyro_GET() == 1.5474482E38F);
			assert (pack.ygyro_GET() == -1.7709E38F);
			assert (pack.yacc_GET() == -3.3292952E37F);
			assert (pack.std_dev_horz_GET() == 9.741095E37F);
			assert (pack.q1_GET() == -1.2147137E38F);
			assert (pack.q4_GET() == -3.2309343E38F);
			assert (pack.pitch_GET() == -1.392112E38F);
			assert (pack.lon_GET() == 3.2945603E37F);
		});
		GroundControl.SIM_STATE p108 = CommunicationChannel.new_SIM_STATE();
		PH.setPack(p108);
		p108.lat_SET(-3.9592432E37F);
		p108.q1_SET(-1.2147137E38F);
		p108.zgyro_SET(1.5474482E38F);
		p108.vd_SET(-3.2508601E38F);
		p108.roll_SET(-1.7096735E38F);
		p108.yacc_SET(-3.3292952E37F);
		p108.alt_SET(2.323707E38F);
		p108.std_dev_horz_SET(9.741095E37F);
		p108.ve_SET(-8.670161E37F);
		p108.lon_SET(3.2945603E37F);
		p108.q2_SET(5.0975816E37F);
		p108.xacc_SET(-2.3110562E38F);
		p108.xgyro_SET(-3.3455952E38F);
		p108.std_dev_vert_SET(-2.4995311E38F);
		p108.zacc_SET(1.9891632E38F);
		p108.q4_SET(-3.2309343E38F);
		p108.q3_SET(-9.939602E37F);
		p108.pitch_SET(-1.392112E38F);
		p108.vn_SET(-2.4518819E38F);
		p108.ygyro_SET(-1.7709E38F);
		p108.yaw_SET(1.0030912E38F);
		CommunicationChannel.instance.send(p108);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_RADIO_STATUS.add((src, ph, pack) ->
		{
			assert (pack.remrssi_GET() == (char) 196);
			assert (pack.remnoise_GET() == (char) 143);
			assert (pack.rssi_GET() == (char) 34);
			assert (pack.rxerrors_GET() == (char) 57917);
			assert (pack.fixed__GET() == (char) 19729);
			assert (pack.noise_GET() == (char) 66);
			assert (pack.txbuf_GET() == (char) 129);
		});
		GroundControl.RADIO_STATUS p109 = CommunicationChannel.new_RADIO_STATUS();
		PH.setPack(p109);
		p109.rssi_SET((char) 34);
		p109.noise_SET((char) 66);
		p109.fixed__SET((char) 19729);
		p109.remnoise_SET((char) 143);
		p109.rxerrors_SET((char) 57917);
		p109.txbuf_SET((char) 129);
		p109.remrssi_SET((char) 196);
		CommunicationChannel.instance.send(p109);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_FILE_TRANSFER_PROTOCOL.add((src, ph, pack) ->
		{
			assert (pack.target_network_GET() == (char) 69);
			assert (pack.target_component_GET() == (char) 155);
			assert (pack.target_system_GET() == (char) 169);
			assert (Arrays.equals(pack.payload_GET(), new char[]{(char) 52, (char) 4, (char) 235, (char) 167, (char) 44, (char) 42, (char) 43, (char) 73, (char) 123, (char) 26, (char) 226, (char) 60, (char) 205, (char) 100, (char) 137, (char) 93, (char) 198, (char) 74, (char) 110, (char) 236, (char) 205, (char) 121, (char) 106, (char) 4, (char) 72, (char) 156, (char) 37, (char) 82, (char) 20, (char) 89, (char) 81, (char) 114, (char) 132, (char) 227, (char) 171, (char) 231, (char) 165, (char) 142, (char) 72, (char) 235, (char) 236, (char) 233, (char) 77, (char) 144, (char) 127, (char) 21, (char) 169, (char) 108, (char) 88, (char) 60, (char) 186, (char) 122, (char) 156, (char) 212, (char) 214, (char) 122, (char) 77, (char) 155, (char) 116, (char) 106, (char) 114, (char) 202, (char) 111, (char) 89, (char) 101, (char) 242, (char) 27, (char) 228, (char) 221, (char) 239, (char) 45, (char) 75, (char) 122, (char) 78, (char) 185, (char) 58, (char) 64, (char) 146, (char) 226, (char) 77, (char) 246, (char) 81, (char) 168, (char) 150, (char) 209, (char) 253, (char) 246, (char) 115, (char) 235, (char) 203, (char) 57, (char) 4, (char) 167, (char) 153, (char) 111, (char) 32, (char) 177, (char) 23, (char) 75, (char) 78, (char) 247, (char) 127, (char) 82, (char) 77, (char) 117, (char) 93, (char) 79, (char) 84, (char) 46, (char) 203, (char) 207, (char) 156, (char) 190, (char) 244, (char) 31, (char) 164, (char) 147, (char) 10, (char) 10, (char) 198, (char) 235, (char) 6, (char) 9, (char) 76, (char) 58, (char) 148, (char) 100, (char) 205, (char) 145, (char) 148, (char) 150, (char) 217, (char) 105, (char) 18, (char) 194, (char) 174, (char) 101, (char) 41, (char) 75, (char) 254, (char) 128, (char) 187, (char) 53, (char) 86, (char) 232, (char) 106, (char) 91, (char) 159, (char) 232, (char) 237, (char) 148, (char) 251, (char) 247, (char) 106, (char) 39, (char) 114, (char) 165, (char) 112, (char) 229, (char) 20, (char) 63, (char) 220, (char) 126, (char) 121, (char) 136, (char) 203, (char) 8, (char) 25, (char) 108, (char) 247, (char) 241, (char) 75, (char) 250, (char) 159, (char) 86, (char) 188, (char) 66, (char) 12, (char) 85, (char) 18, (char) 0, (char) 237, (char) 141, (char) 55, (char) 12, (char) 144, (char) 90, (char) 113, (char) 189, (char) 183, (char) 67, (char) 219, (char) 113, (char) 6, (char) 215, (char) 8, (char) 26, (char) 58, (char) 189, (char) 143, (char) 90, (char) 148, (char) 250, (char) 98, (char) 98, (char) 123, (char) 93, (char) 48, (char) 211, (char) 116, (char) 44, (char) 161, (char) 18, (char) 177, (char) 205, (char) 109, (char) 101, (char) 141, (char) 74, (char) 103, (char) 43, (char) 22, (char) 65, (char) 61, (char) 167, (char) 235, (char) 112, (char) 92, (char) 120, (char) 11, (char) 105, (char) 222, (char) 93, (char) 106, (char) 9, (char) 172, (char) 209, (char) 211, (char) 44, (char) 1, (char) 139, (char) 212, (char) 188, (char) 86, (char) 184, (char) 46, (char) 205, (char) 208, (char) 169, (char) 206, (char) 117}));
		});
		GroundControl.FILE_TRANSFER_PROTOCOL p110 = CommunicationChannel.new_FILE_TRANSFER_PROTOCOL();
		PH.setPack(p110);
		p110.target_network_SET((char) 69);
		p110.target_system_SET((char) 169);
		p110.payload_SET(new char[]{(char) 52, (char) 4, (char) 235, (char) 167, (char) 44, (char) 42, (char) 43, (char) 73, (char) 123, (char) 26, (char) 226, (char) 60, (char) 205, (char) 100, (char) 137, (char) 93, (char) 198, (char) 74, (char) 110, (char) 236, (char) 205, (char) 121, (char) 106, (char) 4, (char) 72, (char) 156, (char) 37, (char) 82, (char) 20, (char) 89, (char) 81, (char) 114, (char) 132, (char) 227, (char) 171, (char) 231, (char) 165, (char) 142, (char) 72, (char) 235, (char) 236, (char) 233, (char) 77, (char) 144, (char) 127, (char) 21, (char) 169, (char) 108, (char) 88, (char) 60, (char) 186, (char) 122, (char) 156, (char) 212, (char) 214, (char) 122, (char) 77, (char) 155, (char) 116, (char) 106, (char) 114, (char) 202, (char) 111, (char) 89, (char) 101, (char) 242, (char) 27, (char) 228, (char) 221, (char) 239, (char) 45, (char) 75, (char) 122, (char) 78, (char) 185, (char) 58, (char) 64, (char) 146, (char) 226, (char) 77, (char) 246, (char) 81, (char) 168, (char) 150, (char) 209, (char) 253, (char) 246, (char) 115, (char) 235, (char) 203, (char) 57, (char) 4, (char) 167, (char) 153, (char) 111, (char) 32, (char) 177, (char) 23, (char) 75, (char) 78, (char) 247, (char) 127, (char) 82, (char) 77, (char) 117, (char) 93, (char) 79, (char) 84, (char) 46, (char) 203, (char) 207, (char) 156, (char) 190, (char) 244, (char) 31, (char) 164, (char) 147, (char) 10, (char) 10, (char) 198, (char) 235, (char) 6, (char) 9, (char) 76, (char) 58, (char) 148, (char) 100, (char) 205, (char) 145, (char) 148, (char) 150, (char) 217, (char) 105, (char) 18, (char) 194, (char) 174, (char) 101, (char) 41, (char) 75, (char) 254, (char) 128, (char) 187, (char) 53, (char) 86, (char) 232, (char) 106, (char) 91, (char) 159, (char) 232, (char) 237, (char) 148, (char) 251, (char) 247, (char) 106, (char) 39, (char) 114, (char) 165, (char) 112, (char) 229, (char) 20, (char) 63, (char) 220, (char) 126, (char) 121, (char) 136, (char) 203, (char) 8, (char) 25, (char) 108, (char) 247, (char) 241, (char) 75, (char) 250, (char) 159, (char) 86, (char) 188, (char) 66, (char) 12, (char) 85, (char) 18, (char) 0, (char) 237, (char) 141, (char) 55, (char) 12, (char) 144, (char) 90, (char) 113, (char) 189, (char) 183, (char) 67, (char) 219, (char) 113, (char) 6, (char) 215, (char) 8, (char) 26, (char) 58, (char) 189, (char) 143, (char) 90, (char) 148, (char) 250, (char) 98, (char) 98, (char) 123, (char) 93, (char) 48, (char) 211, (char) 116, (char) 44, (char) 161, (char) 18, (char) 177, (char) 205, (char) 109, (char) 101, (char) 141, (char) 74, (char) 103, (char) 43, (char) 22, (char) 65, (char) 61, (char) 167, (char) 235, (char) 112, (char) 92, (char) 120, (char) 11, (char) 105, (char) 222, (char) 93, (char) 106, (char) 9, (char) 172, (char) 209, (char) 211, (char) 44, (char) 1, (char) 139, (char) 212, (char) 188, (char) 86, (char) 184, (char) 46, (char) 205, (char) 208, (char) 169, (char) 206, (char) 117}, 0);
		p110.target_component_SET((char) 155);
		CommunicationChannel.instance.send(p110);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_TIMESYNC.add((src, ph, pack) ->
		{
			assert (pack.tc1_GET() == 690454779956882744L);
			assert (pack.ts1_GET() == -3614385600617683376L);
		});
		GroundControl.TIMESYNC p111 = CommunicationChannel.new_TIMESYNC();
		PH.setPack(p111);
		p111.ts1_SET(-3614385600617683376L);
		p111.tc1_SET(690454779956882744L);
		CommunicationChannel.instance.send(p111);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_CAMERA_TRIGGER.add((src, ph, pack) ->
		{
			assert (pack.seq_GET() == 3748460152L);
			assert (pack.time_usec_GET() == 618037767142587335L);
		});
		GroundControl.CAMERA_TRIGGER p112 = CommunicationChannel.new_CAMERA_TRIGGER();
		PH.setPack(p112);
		p112.time_usec_SET(618037767142587335L);
		p112.seq_SET(3748460152L);
		CommunicationChannel.instance.send(p112);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_HIL_GPS.add((src, ph, pack) ->
		{
			assert (pack.alt_GET() == 350994315);
			assert (pack.satellites_visible_GET() == (char) 122);
			assert (pack.fix_type_GET() == (char) 11);
			assert (pack.vd_GET() == (short) -7582);
			assert (pack.eph_GET() == (char) 24960);
			assert (pack.lat_GET() == 2102770663);
			assert (pack.lon_GET() == 1968093345);
			assert (pack.cog_GET() == (char) 22314);
			assert (pack.ve_GET() == (short) -8478);
			assert (pack.vn_GET() == (short) -12858);
			assert (pack.vel_GET() == (char) 53690);
			assert (pack.epv_GET() == (char) 47674);
			assert (pack.time_usec_GET() == 5974333765670091547L);
		});
		GroundControl.HIL_GPS p113 = CommunicationChannel.new_HIL_GPS();
		PH.setPack(p113);
		p113.vd_SET((short) -7582);
		p113.vn_SET((short) -12858);
		p113.eph_SET((char) 24960);
		p113.cog_SET((char) 22314);
		p113.ve_SET((short) -8478);
		p113.vel_SET((char) 53690);
		p113.epv_SET((char) 47674);
		p113.lon_SET(1968093345);
		p113.satellites_visible_SET((char) 122);
		p113.fix_type_SET((char) 11);
		p113.lat_SET(2102770663);
		p113.alt_SET(350994315);
		p113.time_usec_SET(5974333765670091547L);
		CommunicationChannel.instance.send(p113);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_HIL_OPTICAL_FLOW.add((src, ph, pack) ->
		{
			assert (pack.integrated_zgyro_GET() == 1.175754E38F);
			assert (pack.integrated_y_GET() == 1.5271856E38F);
			assert (pack.time_delta_distance_us_GET() == 1941746495L);
			assert (pack.sensor_id_GET() == (char) 180);
			assert (pack.integrated_ygyro_GET() == 4.6492724E37F);
			assert (pack.temperature_GET() == (short) 24385);
			assert (pack.distance_GET() == 5.459879E37F);
			assert (pack.integrated_xgyro_GET() == -5.0151234E37F);
			assert (pack.integration_time_us_GET() == 3565217535L);
			assert (pack.quality_GET() == (char) 35);
			assert (pack.integrated_x_GET() == -1.5519701E38F);
			assert (pack.time_usec_GET() == 1744578449213265932L);
		});
		GroundControl.HIL_OPTICAL_FLOW p114 = CommunicationChannel.new_HIL_OPTICAL_FLOW();
		PH.setPack(p114);
		p114.integrated_x_SET(-1.5519701E38F);
		p114.integrated_zgyro_SET(1.175754E38F);
		p114.integrated_ygyro_SET(4.6492724E37F);
		p114.temperature_SET((short) 24385);
		p114.time_delta_distance_us_SET(1941746495L);
		p114.integrated_xgyro_SET(-5.0151234E37F);
		p114.quality_SET((char) 35);
		p114.sensor_id_SET((char) 180);
		p114.integration_time_us_SET(3565217535L);
		p114.integrated_y_SET(1.5271856E38F);
		p114.time_usec_SET(1744578449213265932L);
		p114.distance_SET(5.459879E37F);
		CommunicationChannel.instance.send(p114);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_HIL_STATE_QUATERNION.add((src, ph, pack) ->
		{
			assert (pack.xacc_GET() == (short) -30402);
			assert (pack.alt_GET() == -1570558352);
			assert (pack.lon_GET() == 1635730394);
			assert (Arrays.equals(pack.attitude_quaternion_GET(), new float[]{-1.1342597E38F, 1.4346061E38F, 3.2940842E38F, 1.5204292E38F}));
			assert (pack.lat_GET() == 1020006218);
			assert (pack.ind_airspeed_GET() == (char) 19945);
			assert (pack.zacc_GET() == (short) 19964);
			assert (pack.pitchspeed_GET() == 7.614998E37F);
			assert (pack.rollspeed_GET() == -2.8001554E38F);
			assert (pack.yawspeed_GET() == 2.7337743E38F);
			assert (pack.vy_GET() == (short) 4851);
			assert (pack.vz_GET() == (short) 32690);
			assert (pack.yacc_GET() == (short) 25477);
			assert (pack.time_usec_GET() == 4307076305721383007L);
			assert (pack.true_airspeed_GET() == (char) 36875);
			assert (pack.vx_GET() == (short) 31658);
		});
		GroundControl.HIL_STATE_QUATERNION p115 = CommunicationChannel.new_HIL_STATE_QUATERNION();
		PH.setPack(p115);
		p115.yacc_SET((short) 25477);
		p115.xacc_SET((short) -30402);
		p115.lon_SET(1635730394);
		p115.true_airspeed_SET((char) 36875);
		p115.alt_SET(-1570558352);
		p115.vz_SET((short) 32690);
		p115.ind_airspeed_SET((char) 19945);
		p115.vx_SET((short) 31658);
		p115.yawspeed_SET(2.7337743E38F);
		p115.zacc_SET((short) 19964);
		p115.lat_SET(1020006218);
		p115.vy_SET((short) 4851);
		p115.rollspeed_SET(-2.8001554E38F);
		p115.pitchspeed_SET(7.614998E37F);
		p115.time_usec_SET(4307076305721383007L);
		p115.attitude_quaternion_SET(new float[]{-1.1342597E38F, 1.4346061E38F, 3.2940842E38F, 1.5204292E38F}, 0);
		CommunicationChannel.instance.send(p115);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_SCALED_IMU2.add((src, ph, pack) ->
		{
			assert (pack.xacc_GET() == (short) 16191);
			assert (pack.xmag_GET() == (short) 18888);
			assert (pack.xgyro_GET() == (short) 13758);
			assert (pack.ymag_GET() == (short) -17177);
			assert (pack.ygyro_GET() == (short) 5533);
			assert (pack.zgyro_GET() == (short) 16198);
			assert (pack.zacc_GET() == (short) 32310);
			assert (pack.time_boot_ms_GET() == 4042451475L);
			assert (pack.zmag_GET() == (short) 30031);
			assert (pack.yacc_GET() == (short) -31811);
		});
		GroundControl.SCALED_IMU2 p116 = CommunicationChannel.new_SCALED_IMU2();
		PH.setPack(p116);
		p116.zgyro_SET((short) 16198);
		p116.zacc_SET((short) 32310);
		p116.xmag_SET((short) 18888);
		p116.zmag_SET((short) 30031);
		p116.time_boot_ms_SET(4042451475L);
		p116.ymag_SET((short) -17177);
		p116.xgyro_SET((short) 13758);
		p116.xacc_SET((short) 16191);
		p116.yacc_SET((short) -31811);
		p116.ygyro_SET((short) 5533);
		CommunicationChannel.instance.send(p116);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_LOG_REQUEST_LIST.add((src, ph, pack) ->
		{
			assert (pack.end_GET() == (char) 61961);
			assert (pack.target_component_GET() == (char) 117);
			assert (pack.target_system_GET() == (char) 23);
			assert (pack.start_GET() == (char) 5993);
		});
		GroundControl.LOG_REQUEST_LIST p117 = CommunicationChannel.new_LOG_REQUEST_LIST();
		PH.setPack(p117);
		p117.target_system_SET((char) 23);
		p117.start_SET((char) 5993);
		p117.end_SET((char) 61961);
		p117.target_component_SET((char) 117);
		CommunicationChannel.instance.send(p117);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_LOG_ENTRY.add((src, ph, pack) ->
		{
			assert (pack.time_utc_GET() == 2486737466L);
			assert (pack.num_logs_GET() == (char) 50185);
			assert (pack.id_GET() == (char) 35677);
			assert (pack.last_log_num_GET() == (char) 41904);
			assert (pack.size_GET() == 1488001553L);
		});
		GroundControl.LOG_ENTRY p118 = CommunicationChannel.new_LOG_ENTRY();
		PH.setPack(p118);
		p118.last_log_num_SET((char) 41904);
		p118.size_SET(1488001553L);
		p118.id_SET((char) 35677);
		p118.time_utc_SET(2486737466L);
		p118.num_logs_SET((char) 50185);
		CommunicationChannel.instance.send(p118);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_LOG_REQUEST_DATA.add((src, ph, pack) ->
		{
			assert (pack.id_GET() == (char) 4893);
			assert (pack.target_system_GET() == (char) 236);
			assert (pack.ofs_GET() == 4088447720L);
			assert (pack.target_component_GET() == (char) 109);
			assert (pack.count_GET() == 2264685300L);
		});
		GroundControl.LOG_REQUEST_DATA p119 = CommunicationChannel.new_LOG_REQUEST_DATA();
		PH.setPack(p119);
		p119.count_SET(2264685300L);
		p119.target_component_SET((char) 109);
		p119.target_system_SET((char) 236);
		p119.id_SET((char) 4893);
		p119.ofs_SET(4088447720L);
		CommunicationChannel.instance.send(p119);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_LOG_DATA.add((src, ph, pack) ->
		{
			assert (pack.id_GET() == (char) 62442);
			assert (Arrays.equals(pack.data__GET(), new char[]{(char) 154, (char) 38, (char) 65, (char) 95, (char) 102, (char) 93, (char) 25, (char) 237, (char) 167, (char) 10, (char) 48, (char) 101, (char) 217, (char) 22, (char) 73, (char) 145, (char) 26, (char) 20, (char) 144, (char) 107, (char) 199, (char) 30, (char) 186, (char) 220, (char) 30, (char) 82, (char) 107, (char) 16, (char) 127, (char) 172, (char) 198, (char) 80, (char) 211, (char) 153, (char) 147, (char) 17, (char) 149, (char) 165, (char) 230, (char) 47, (char) 141, (char) 77, (char) 136, (char) 53, (char) 224, (char) 99, (char) 39, (char) 9, (char) 251, (char) 251, (char) 95, (char) 57, (char) 242, (char) 34, (char) 58, (char) 117, (char) 120, (char) 31, (char) 17, (char) 243, (char) 206, (char) 162, (char) 8, (char) 101, (char) 185, (char) 151, (char) 245, (char) 245, (char) 46, (char) 222, (char) 197, (char) 226, (char) 105, (char) 221, (char) 76, (char) 237, (char) 125, (char) 197, (char) 71, (char) 79, (char) 137, (char) 151, (char) 224, (char) 58, (char) 39, (char) 175, (char) 89, (char) 56, (char) 195, (char) 235}));
			assert (pack.count_GET() == (char) 90);
			assert (pack.ofs_GET() == 1521313181L);
		});
		GroundControl.LOG_DATA p120 = CommunicationChannel.new_LOG_DATA();
		PH.setPack(p120);
		p120.id_SET((char) 62442);
		p120.count_SET((char) 90);
		p120.data__SET(new char[]{(char) 154, (char) 38, (char) 65, (char) 95, (char) 102, (char) 93, (char) 25, (char) 237, (char) 167, (char) 10, (char) 48, (char) 101, (char) 217, (char) 22, (char) 73, (char) 145, (char) 26, (char) 20, (char) 144, (char) 107, (char) 199, (char) 30, (char) 186, (char) 220, (char) 30, (char) 82, (char) 107, (char) 16, (char) 127, (char) 172, (char) 198, (char) 80, (char) 211, (char) 153, (char) 147, (char) 17, (char) 149, (char) 165, (char) 230, (char) 47, (char) 141, (char) 77, (char) 136, (char) 53, (char) 224, (char) 99, (char) 39, (char) 9, (char) 251, (char) 251, (char) 95, (char) 57, (char) 242, (char) 34, (char) 58, (char) 117, (char) 120, (char) 31, (char) 17, (char) 243, (char) 206, (char) 162, (char) 8, (char) 101, (char) 185, (char) 151, (char) 245, (char) 245, (char) 46, (char) 222, (char) 197, (char) 226, (char) 105, (char) 221, (char) 76, (char) 237, (char) 125, (char) 197, (char) 71, (char) 79, (char) 137, (char) 151, (char) 224, (char) 58, (char) 39, (char) 175, (char) 89, (char) 56, (char) 195, (char) 235}, 0);
		p120.ofs_SET(1521313181L);
		CommunicationChannel.instance.send(p120);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_LOG_ERASE.add((src, ph, pack) ->
		{
			assert (pack.target_component_GET() == (char) 226);
			assert (pack.target_system_GET() == (char) 214);
		});
		GroundControl.LOG_ERASE p121 = CommunicationChannel.new_LOG_ERASE();
		PH.setPack(p121);
		p121.target_component_SET((char) 226);
		p121.target_system_SET((char) 214);
		CommunicationChannel.instance.send(p121);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_LOG_REQUEST_END.add((src, ph, pack) ->
		{
			assert (pack.target_component_GET() == (char) 105);
			assert (pack.target_system_GET() == (char) 41);
		});
		GroundControl.LOG_REQUEST_END p122 = CommunicationChannel.new_LOG_REQUEST_END();
		PH.setPack(p122);
		p122.target_system_SET((char) 41);
		p122.target_component_SET((char) 105);
		CommunicationChannel.instance.send(p122);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_GPS_INJECT_DATA.add((src, ph, pack) ->
		{
			assert (pack.target_system_GET() == (char) 60);
			assert (pack.len_GET() == (char) 38);
			assert (pack.target_component_GET() == (char) 136);
			assert (Arrays.equals(pack.data__GET(), new char[]{(char) 246, (char) 61, (char) 179, (char) 148, (char) 129, (char) 34, (char) 149, (char) 89, (char) 187, (char) 157, (char) 24, (char) 236, (char) 79, (char) 29, (char) 128, (char) 108, (char) 176, (char) 207, (char) 236, (char) 147, (char) 224, (char) 78, (char) 47, (char) 237, (char) 205, (char) 54, (char) 255, (char) 197, (char) 244, (char) 122, (char) 62, (char) 146, (char) 168, (char) 114, (char) 167, (char) 87, (char) 191, (char) 152, (char) 202, (char) 83, (char) 184, (char) 34, (char) 27, (char) 20, (char) 205, (char) 36, (char) 36, (char) 82, (char) 93, (char) 209, (char) 172, (char) 205, (char) 52, (char) 22, (char) 254, (char) 30, (char) 167, (char) 181, (char) 125, (char) 169, (char) 85, (char) 90, (char) 234, (char) 248, (char) 2, (char) 138, (char) 91, (char) 131, (char) 48, (char) 152, (char) 180, (char) 209, (char) 71, (char) 37, (char) 43, (char) 187, (char) 104, (char) 114, (char) 138, (char) 238, (char) 97, (char) 46, (char) 202, (char) 72, (char) 159, (char) 147, (char) 175, (char) 176, (char) 1, (char) 203, (char) 58, (char) 211, (char) 217, (char) 93, (char) 35, (char) 238, (char) 241, (char) 115, (char) 245, (char) 236, (char) 119, (char) 161, (char) 155, (char) 124, (char) 24, (char) 80, (char) 53, (char) 119, (char) 8, (char) 75}));
		});
		GroundControl.GPS_INJECT_DATA p123 = CommunicationChannel.new_GPS_INJECT_DATA();
		PH.setPack(p123);
		p123.len_SET((char) 38);
		p123.target_system_SET((char) 60);
		p123.target_component_SET((char) 136);
		p123.data__SET(new char[]{(char) 246, (char) 61, (char) 179, (char) 148, (char) 129, (char) 34, (char) 149, (char) 89, (char) 187, (char) 157, (char) 24, (char) 236, (char) 79, (char) 29, (char) 128, (char) 108, (char) 176, (char) 207, (char) 236, (char) 147, (char) 224, (char) 78, (char) 47, (char) 237, (char) 205, (char) 54, (char) 255, (char) 197, (char) 244, (char) 122, (char) 62, (char) 146, (char) 168, (char) 114, (char) 167, (char) 87, (char) 191, (char) 152, (char) 202, (char) 83, (char) 184, (char) 34, (char) 27, (char) 20, (char) 205, (char) 36, (char) 36, (char) 82, (char) 93, (char) 209, (char) 172, (char) 205, (char) 52, (char) 22, (char) 254, (char) 30, (char) 167, (char) 181, (char) 125, (char) 169, (char) 85, (char) 90, (char) 234, (char) 248, (char) 2, (char) 138, (char) 91, (char) 131, (char) 48, (char) 152, (char) 180, (char) 209, (char) 71, (char) 37, (char) 43, (char) 187, (char) 104, (char) 114, (char) 138, (char) 238, (char) 97, (char) 46, (char) 202, (char) 72, (char) 159, (char) 147, (char) 175, (char) 176, (char) 1, (char) 203, (char) 58, (char) 211, (char) 217, (char) 93, (char) 35, (char) 238, (char) 241, (char) 115, (char) 245, (char) 236, (char) 119, (char) 161, (char) 155, (char) 124, (char) 24, (char) 80, (char) 53, (char) 119, (char) 8, (char) 75}, 0);
		CommunicationChannel.instance.send(p123);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_GPS2_RAW.add((src, ph, pack) ->
		{
			assert (pack.satellites_visible_GET() == (char) 51);
			assert (pack.dgps_numch_GET() == (char) 189);
			assert (pack.alt_GET() == 1738700013);
			assert (pack.vel_GET() == (char) 30414);
			assert (pack.time_usec_GET() == 9156028708769239703L);
			assert (pack.cog_GET() == (char) 51837);
			assert (pack.dgps_age_GET() == 3645998351L);
			assert (pack.fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_PPP);
			assert (pack.lon_GET() == -1913876208);
			assert (pack.eph_GET() == (char) 13523);
			assert (pack.epv_GET() == (char) 1710);
			assert (pack.lat_GET() == 147158012);
		});
		GroundControl.GPS2_RAW p124 = CommunicationChannel.new_GPS2_RAW();
		PH.setPack(p124);
		p124.lat_SET(147158012);
		p124.cog_SET((char) 51837);
		p124.lon_SET(-1913876208);
		p124.epv_SET((char) 1710);
		p124.satellites_visible_SET((char) 51);
		p124.alt_SET(1738700013);
		p124.time_usec_SET(9156028708769239703L);
		p124.eph_SET((char) 13523);
		p124.dgps_age_SET(3645998351L);
		p124.fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_PPP);
		p124.dgps_numch_SET((char) 189);
		p124.vel_SET((char) 30414);
		CommunicationChannel.instance.send(p124);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_POWER_STATUS.add((src, ph, pack) ->
		{
			assert (pack.flags_GET() == MAV_POWER_STATUS.MAV_POWER_STATUS_CHANGED);
			assert (pack.Vservo_GET() == (char) 60779);
			assert (pack.Vcc_GET() == (char) 42496);
		});
		GroundControl.POWER_STATUS p125 = CommunicationChannel.new_POWER_STATUS();
		PH.setPack(p125);
		p125.Vservo_SET((char) 60779);
		p125.flags_SET(MAV_POWER_STATUS.MAV_POWER_STATUS_CHANGED);
		p125.Vcc_SET((char) 42496);
		CommunicationChannel.instance.send(p125);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_SERIAL_CONTROL.add((src, ph, pack) ->
		{
			assert (pack.count_GET() == (char) 70);
			assert (Arrays.equals(pack.data__GET(), new char[]{(char) 230, (char) 60, (char) 140, (char) 212, (char) 240, (char) 132, (char) 82, (char) 113, (char) 168, (char) 72, (char) 31, (char) 76, (char) 123, (char) 123, (char) 124, (char) 215, (char) 150, (char) 112, (char) 154, (char) 117, (char) 41, (char) 44, (char) 160, (char) 64, (char) 160, (char) 189, (char) 110, (char) 46, (char) 84, (char) 117, (char) 217, (char) 175, (char) 98, (char) 197, (char) 69, (char) 216, (char) 198, (char) 136, (char) 23, (char) 137, (char) 177, (char) 230, (char) 148, (char) 162, (char) 64, (char) 9, (char) 50, (char) 208, (char) 30, (char) 182, (char) 34, (char) 203, (char) 119, (char) 73, (char) 156, (char) 195, (char) 33, (char) 173, (char) 148, (char) 7, (char) 41, (char) 232, (char) 112, (char) 2, (char) 111, (char) 195, (char) 17, (char) 162, (char) 56, (char) 243}));
			assert (pack.timeout_GET() == (char) 1138);
			assert (pack.flags_GET() == SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_REPLY);
			assert (pack.device_GET() == SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_TELEM2);
			assert (pack.baudrate_GET() == 4087871693L);
		});
		GroundControl.SERIAL_CONTROL p126 = CommunicationChannel.new_SERIAL_CONTROL();
		PH.setPack(p126);
		p126.count_SET((char) 70);
		p126.baudrate_SET(4087871693L);
		p126.timeout_SET((char) 1138);
		p126.device_SET(SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_TELEM2);
		p126.data__SET(new char[]{(char) 230, (char) 60, (char) 140, (char) 212, (char) 240, (char) 132, (char) 82, (char) 113, (char) 168, (char) 72, (char) 31, (char) 76, (char) 123, (char) 123, (char) 124, (char) 215, (char) 150, (char) 112, (char) 154, (char) 117, (char) 41, (char) 44, (char) 160, (char) 64, (char) 160, (char) 189, (char) 110, (char) 46, (char) 84, (char) 117, (char) 217, (char) 175, (char) 98, (char) 197, (char) 69, (char) 216, (char) 198, (char) 136, (char) 23, (char) 137, (char) 177, (char) 230, (char) 148, (char) 162, (char) 64, (char) 9, (char) 50, (char) 208, (char) 30, (char) 182, (char) 34, (char) 203, (char) 119, (char) 73, (char) 156, (char) 195, (char) 33, (char) 173, (char) 148, (char) 7, (char) 41, (char) 232, (char) 112, (char) 2, (char) 111, (char) 195, (char) 17, (char) 162, (char) 56, (char) 243}, 0);
		p126.flags_SET(SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_REPLY);
		CommunicationChannel.instance.send(p126);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_GPS_RTK.add((src, ph, pack) ->
		{
			assert (pack.wn_GET() == (char) 15459);
			assert (pack.time_last_baseline_ms_GET() == 2210394182L);
			assert (pack.baseline_a_mm_GET() == -687969219);
			assert (pack.rtk_receiver_id_GET() == (char) 35);
			assert (pack.accuracy_GET() == 970214158L);
			assert (pack.baseline_coords_type_GET() == (char) 187);
			assert (pack.rtk_rate_GET() == (char) 108);
			assert (pack.baseline_b_mm_GET() == 1677720771);
			assert (pack.nsats_GET() == (char) 20);
			assert (pack.rtk_health_GET() == (char) 231);
			assert (pack.tow_GET() == 932779551L);
			assert (pack.baseline_c_mm_GET() == 1054336057);
			assert (pack.iar_num_hypotheses_GET() == 2113625815);
		});
		GroundControl.GPS_RTK p127 = CommunicationChannel.new_GPS_RTK();
		PH.setPack(p127);
		p127.accuracy_SET(970214158L);
		p127.baseline_c_mm_SET(1054336057);
		p127.iar_num_hypotheses_SET(2113625815);
		p127.rtk_receiver_id_SET((char) 35);
		p127.wn_SET((char) 15459);
		p127.tow_SET(932779551L);
		p127.time_last_baseline_ms_SET(2210394182L);
		p127.baseline_coords_type_SET((char) 187);
		p127.rtk_health_SET((char) 231);
		p127.baseline_a_mm_SET(-687969219);
		p127.rtk_rate_SET((char) 108);
		p127.baseline_b_mm_SET(1677720771);
		p127.nsats_SET((char) 20);
		CommunicationChannel.instance.send(p127);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_GPS2_RTK.add((src, ph, pack) ->
		{
			assert (pack.baseline_coords_type_GET() == (char) 176);
			assert (pack.wn_GET() == (char) 4569);
			assert (pack.tow_GET() == 708368789L);
			assert (pack.nsats_GET() == (char) 38);
			assert (pack.time_last_baseline_ms_GET() == 3288726181L);
			assert (pack.accuracy_GET() == 1731534044L);
			assert (pack.baseline_c_mm_GET() == -1302618740);
			assert (pack.rtk_health_GET() == (char) 5);
			assert (pack.baseline_b_mm_GET() == 1869536383);
			assert (pack.baseline_a_mm_GET() == -1060721447);
			assert (pack.rtk_rate_GET() == (char) 224);
			assert (pack.rtk_receiver_id_GET() == (char) 0);
			assert (pack.iar_num_hypotheses_GET() == -449349123);
		});
		GroundControl.GPS2_RTK p128 = CommunicationChannel.new_GPS2_RTK();
		PH.setPack(p128);
		p128.rtk_receiver_id_SET((char) 0);
		p128.baseline_coords_type_SET((char) 176);
		p128.accuracy_SET(1731534044L);
		p128.iar_num_hypotheses_SET(-449349123);
		p128.baseline_a_mm_SET(-1060721447);
		p128.baseline_c_mm_SET(-1302618740);
		p128.wn_SET((char) 4569);
		p128.rtk_rate_SET((char) 224);
		p128.nsats_SET((char) 38);
		p128.time_last_baseline_ms_SET(3288726181L);
		p128.baseline_b_mm_SET(1869536383);
		p128.rtk_health_SET((char) 5);
		p128.tow_SET(708368789L);
		CommunicationChannel.instance.send(p128);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_SCALED_IMU3.add((src, ph, pack) ->
		{
			assert (pack.xmag_GET() == (short) -13778);
			assert (pack.zgyro_GET() == (short) -14993);
			assert (pack.ygyro_GET() == (short) -25899);
			assert (pack.xacc_GET() == (short) -14861);
			assert (pack.yacc_GET() == (short) 19135);
			assert (pack.zmag_GET() == (short) -20356);
			assert (pack.zacc_GET() == (short) -5245);
			assert (pack.time_boot_ms_GET() == 1319664134L);
			assert (pack.ymag_GET() == (short) -13531);
			assert (pack.xgyro_GET() == (short) -30781);
		});
		GroundControl.SCALED_IMU3 p129 = CommunicationChannel.new_SCALED_IMU3();
		PH.setPack(p129);
		p129.ygyro_SET((short) -25899);
		p129.zgyro_SET((short) -14993);
		p129.ymag_SET((short) -13531);
		p129.zmag_SET((short) -20356);
		p129.zacc_SET((short) -5245);
		p129.xacc_SET((short) -14861);
		p129.yacc_SET((short) 19135);
		p129.time_boot_ms_SET(1319664134L);
		p129.xmag_SET((short) -13778);
		p129.xgyro_SET((short) -30781);
		CommunicationChannel.instance.send(p129);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_DATA_TRANSMISSION_HANDSHAKE.add((src, ph, pack) ->
		{
			assert (pack.size_GET() == 3223114937L);
			assert (pack.width_GET() == (char) 12412);
			assert (pack.payload_GET() == (char) 140);
			assert (pack.jpg_quality_GET() == (char) 134);
			assert (pack.packets_GET() == (char) 29945);
			assert (pack.type_GET() == (char) 62);
			assert (pack.height_GET() == (char) 29482);
		});
		GroundControl.DATA_TRANSMISSION_HANDSHAKE p130 = CommunicationChannel.new_DATA_TRANSMISSION_HANDSHAKE();
		PH.setPack(p130);
		p130.payload_SET((char) 140);
		p130.jpg_quality_SET((char) 134);
		p130.size_SET(3223114937L);
		p130.packets_SET((char) 29945);
		p130.type_SET((char) 62);
		p130.width_SET((char) 12412);
		p130.height_SET((char) 29482);
		CommunicationChannel.instance.send(p130);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_ENCAPSULATED_DATA.add((src, ph, pack) ->
		{
			assert (pack.seqnr_GET() == (char) 35142);
			assert (Arrays.equals(pack.data__GET(), new char[]{(char) 38, (char) 178, (char) 255, (char) 80, (char) 154, (char) 66, (char) 190, (char) 123, (char) 198, (char) 133, (char) 175, (char) 13, (char) 74, (char) 69, (char) 167, (char) 244, (char) 225, (char) 111, (char) 21, (char) 171, (char) 13, (char) 252, (char) 251, (char) 111, (char) 138, (char) 66, (char) 96, (char) 118, (char) 151, (char) 128, (char) 22, (char) 69, (char) 172, (char) 39, (char) 135, (char) 69, (char) 96, (char) 182, (char) 40, (char) 108, (char) 200, (char) 239, (char) 149, (char) 74, (char) 153, (char) 96, (char) 67, (char) 1, (char) 119, (char) 85, (char) 9, (char) 80, (char) 93, (char) 60, (char) 166, (char) 53, (char) 197, (char) 87, (char) 37, (char) 92, (char) 239, (char) 204, (char) 196, (char) 120, (char) 116, (char) 230, (char) 95, (char) 56, (char) 209, (char) 194, (char) 199, (char) 192, (char) 174, (char) 231, (char) 185, (char) 143, (char) 119, (char) 68, (char) 140, (char) 197, (char) 236, (char) 208, (char) 201, (char) 43, (char) 18, (char) 161, (char) 222, (char) 22, (char) 212, (char) 60, (char) 35, (char) 109, (char) 47, (char) 173, (char) 216, (char) 168, (char) 89, (char) 29, (char) 37, (char) 218, (char) 168, (char) 5, (char) 154, (char) 156, (char) 135, (char) 176, (char) 17, (char) 20, (char) 224, (char) 189, (char) 218, (char) 242, (char) 195, (char) 25, (char) 148, (char) 231, (char) 141, (char) 55, (char) 104, (char) 35, (char) 91, (char) 172, (char) 54, (char) 9, (char) 199, (char) 71, (char) 71, (char) 235, (char) 33, (char) 156, (char) 186, (char) 3, (char) 153, (char) 214, (char) 39, (char) 166, (char) 168, (char) 83, (char) 85, (char) 43, (char) 215, (char) 29, (char) 232, (char) 93, (char) 149, (char) 92, (char) 238, (char) 43, (char) 169, (char) 199, (char) 97, (char) 88, (char) 107, (char) 122, (char) 117, (char) 205, (char) 208, (char) 2, (char) 62, (char) 0, (char) 158, (char) 114, (char) 203, (char) 241, (char) 214, (char) 245, (char) 209, (char) 22, (char) 160, (char) 37, (char) 145, (char) 103, (char) 122, (char) 139, (char) 221, (char) 99, (char) 47, (char) 44, (char) 4, (char) 111, (char) 3, (char) 222, (char) 199, (char) 213, (char) 108, (char) 36, (char) 186, (char) 58, (char) 160, (char) 193, (char) 73, (char) 178, (char) 16, (char) 201, (char) 109, (char) 31, (char) 79, (char) 102, (char) 208, (char) 19, (char) 252, (char) 128, (char) 106, (char) 78, (char) 5, (char) 29, (char) 112, (char) 160, (char) 77, (char) 220, (char) 100, (char) 10, (char) 172, (char) 20, (char) 133, (char) 180, (char) 137, (char) 216, (char) 155, (char) 67, (char) 175, (char) 213, (char) 125, (char) 226, (char) 85, (char) 164, (char) 178, (char) 185, (char) 52, (char) 36, (char) 130, (char) 44, (char) 146, (char) 35, (char) 137, (char) 208, (char) 226, (char) 226, (char) 196, (char) 201, (char) 228, (char) 243, (char) 104, (char) 179, (char) 145, (char) 182, (char) 199, (char) 68, (char) 144, (char) 118, (char) 58, (char) 214, (char) 46}));
		});
		GroundControl.ENCAPSULATED_DATA p131 = CommunicationChannel.new_ENCAPSULATED_DATA();
		PH.setPack(p131);
		p131.seqnr_SET((char) 35142);
		p131.data__SET(new char[]{(char) 38, (char) 178, (char) 255, (char) 80, (char) 154, (char) 66, (char) 190, (char) 123, (char) 198, (char) 133, (char) 175, (char) 13, (char) 74, (char) 69, (char) 167, (char) 244, (char) 225, (char) 111, (char) 21, (char) 171, (char) 13, (char) 252, (char) 251, (char) 111, (char) 138, (char) 66, (char) 96, (char) 118, (char) 151, (char) 128, (char) 22, (char) 69, (char) 172, (char) 39, (char) 135, (char) 69, (char) 96, (char) 182, (char) 40, (char) 108, (char) 200, (char) 239, (char) 149, (char) 74, (char) 153, (char) 96, (char) 67, (char) 1, (char) 119, (char) 85, (char) 9, (char) 80, (char) 93, (char) 60, (char) 166, (char) 53, (char) 197, (char) 87, (char) 37, (char) 92, (char) 239, (char) 204, (char) 196, (char) 120, (char) 116, (char) 230, (char) 95, (char) 56, (char) 209, (char) 194, (char) 199, (char) 192, (char) 174, (char) 231, (char) 185, (char) 143, (char) 119, (char) 68, (char) 140, (char) 197, (char) 236, (char) 208, (char) 201, (char) 43, (char) 18, (char) 161, (char) 222, (char) 22, (char) 212, (char) 60, (char) 35, (char) 109, (char) 47, (char) 173, (char) 216, (char) 168, (char) 89, (char) 29, (char) 37, (char) 218, (char) 168, (char) 5, (char) 154, (char) 156, (char) 135, (char) 176, (char) 17, (char) 20, (char) 224, (char) 189, (char) 218, (char) 242, (char) 195, (char) 25, (char) 148, (char) 231, (char) 141, (char) 55, (char) 104, (char) 35, (char) 91, (char) 172, (char) 54, (char) 9, (char) 199, (char) 71, (char) 71, (char) 235, (char) 33, (char) 156, (char) 186, (char) 3, (char) 153, (char) 214, (char) 39, (char) 166, (char) 168, (char) 83, (char) 85, (char) 43, (char) 215, (char) 29, (char) 232, (char) 93, (char) 149, (char) 92, (char) 238, (char) 43, (char) 169, (char) 199, (char) 97, (char) 88, (char) 107, (char) 122, (char) 117, (char) 205, (char) 208, (char) 2, (char) 62, (char) 0, (char) 158, (char) 114, (char) 203, (char) 241, (char) 214, (char) 245, (char) 209, (char) 22, (char) 160, (char) 37, (char) 145, (char) 103, (char) 122, (char) 139, (char) 221, (char) 99, (char) 47, (char) 44, (char) 4, (char) 111, (char) 3, (char) 222, (char) 199, (char) 213, (char) 108, (char) 36, (char) 186, (char) 58, (char) 160, (char) 193, (char) 73, (char) 178, (char) 16, (char) 201, (char) 109, (char) 31, (char) 79, (char) 102, (char) 208, (char) 19, (char) 252, (char) 128, (char) 106, (char) 78, (char) 5, (char) 29, (char) 112, (char) 160, (char) 77, (char) 220, (char) 100, (char) 10, (char) 172, (char) 20, (char) 133, (char) 180, (char) 137, (char) 216, (char) 155, (char) 67, (char) 175, (char) 213, (char) 125, (char) 226, (char) 85, (char) 164, (char) 178, (char) 185, (char) 52, (char) 36, (char) 130, (char) 44, (char) 146, (char) 35, (char) 137, (char) 208, (char) 226, (char) 226, (char) 196, (char) 201, (char) 228, (char) 243, (char) 104, (char) 179, (char) 145, (char) 182, (char) 199, (char) 68, (char) 144, (char) 118, (char) 58, (char) 214, (char) 46}, 0);
		CommunicationChannel.instance.send(p131);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_DISTANCE_SENSOR.add((src, ph, pack) ->
		{
			assert (pack.covariance_GET() == (char) 236);
			assert (pack.id_GET() == (char) 153);
			assert (pack.max_distance_GET() == (char) 52875);
			assert (pack.current_distance_GET() == (char) 37769);
			assert (pack.type_GET() == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_LASER);
			assert (pack.orientation_GET() == MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_90);
			assert (pack.time_boot_ms_GET() == 3280479802L);
			assert (pack.min_distance_GET() == (char) 62102);
		});
		GroundControl.DISTANCE_SENSOR p132 = CommunicationChannel.new_DISTANCE_SENSOR();
		PH.setPack(p132);
		p132.min_distance_SET((char) 62102);
		p132.time_boot_ms_SET(3280479802L);
		p132.covariance_SET((char) 236);
		p132.max_distance_SET((char) 52875);
		p132.orientation_SET(MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_90);
		p132.current_distance_SET((char) 37769);
		p132.type_SET(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_LASER);
		p132.id_SET((char) 153);
		CommunicationChannel.instance.send(p132);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_TERRAIN_REQUEST.add((src, ph, pack) ->
		{
			assert (pack.lat_GET() == -200466172);
			assert (pack.lon_GET() == 2146699233);
			assert (pack.grid_spacing_GET() == (char) 43620);
			assert (pack.mask_GET() == 2349281928019934128L);
		});
		GroundControl.TERRAIN_REQUEST p133 = CommunicationChannel.new_TERRAIN_REQUEST();
		PH.setPack(p133);
		p133.grid_spacing_SET((char) 43620);
		p133.lat_SET(-200466172);
		p133.lon_SET(2146699233);
		p133.mask_SET(2349281928019934128L);
		CommunicationChannel.instance.send(p133);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_TERRAIN_DATA.add((src, ph, pack) ->
		{
			assert (pack.lat_GET() == 1128839864);
			assert (pack.lon_GET() == 1310221955);
			assert (pack.grid_spacing_GET() == (char) 58225);
			assert (Arrays.equals(pack.data__GET(), new short[]{(short) 26742, (short) -5693, (short) 25312, (short) -11930, (short) 13927, (short) -11706, (short) 8597, (short) 16065, (short) -20668, (short) 17089, (short) -6549, (short) 7309, (short) -6624, (short) 2028, (short) -6738, (short) 32321}));
			assert (pack.gridbit_GET() == (char) 56);
		});
		GroundControl.TERRAIN_DATA p134 = CommunicationChannel.new_TERRAIN_DATA();
		PH.setPack(p134);
		p134.grid_spacing_SET((char) 58225);
		p134.lon_SET(1310221955);
		p134.lat_SET(1128839864);
		p134.data__SET(new short[]{(short) 26742, (short) -5693, (short) 25312, (short) -11930, (short) 13927, (short) -11706, (short) 8597, (short) 16065, (short) -20668, (short) 17089, (short) -6549, (short) 7309, (short) -6624, (short) 2028, (short) -6738, (short) 32321}, 0);
		p134.gridbit_SET((char) 56);
		CommunicationChannel.instance.send(p134);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_TERRAIN_CHECK.add((src, ph, pack) ->
		{
			assert (pack.lon_GET() == -1463429244);
			assert (pack.lat_GET() == 507456236);
		});
		GroundControl.TERRAIN_CHECK p135 = CommunicationChannel.new_TERRAIN_CHECK();
		PH.setPack(p135);
		p135.lat_SET(507456236);
		p135.lon_SET(-1463429244);
		CommunicationChannel.instance.send(p135);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_TERRAIN_REPORT.add((src, ph, pack) ->
		{
			assert (pack.terrain_height_GET() == 3.141145E38F);
			assert (pack.loaded_GET() == (char) 49787);
			assert (pack.pending_GET() == (char) 17763);
			assert (pack.lon_GET() == -1740641324);
			assert (pack.spacing_GET() == (char) 8426);
			assert (pack.lat_GET() == 1877048434);
			assert (pack.current_height_GET() == 2.5349294E38F);
		});
		GroundControl.TERRAIN_REPORT p136 = CommunicationChannel.new_TERRAIN_REPORT();
		PH.setPack(p136);
		p136.current_height_SET(2.5349294E38F);
		p136.pending_SET((char) 17763);
		p136.lat_SET(1877048434);
		p136.spacing_SET((char) 8426);
		p136.terrain_height_SET(3.141145E38F);
		p136.loaded_SET((char) 49787);
		p136.lon_SET(-1740641324);
		CommunicationChannel.instance.send(p136);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_SCALED_PRESSURE2.add((src, ph, pack) ->
		{
			assert (pack.press_abs_GET() == 1.1847747E38F);
			assert (pack.temperature_GET() == (short) -4573);
			assert (pack.press_diff_GET() == 5.316397E37F);
			assert (pack.time_boot_ms_GET() == 1209466891L);
		});
		GroundControl.SCALED_PRESSURE2 p137 = CommunicationChannel.new_SCALED_PRESSURE2();
		PH.setPack(p137);
		p137.time_boot_ms_SET(1209466891L);
		p137.temperature_SET((short) -4573);
		p137.press_diff_SET(5.316397E37F);
		p137.press_abs_SET(1.1847747E38F);
		CommunicationChannel.instance.send(p137);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_ATT_POS_MOCAP.add((src, ph, pack) ->
		{
			assert (pack.z_GET() == 1.5664881E38F);
			assert (Arrays.equals(pack.q_GET(), new float[]{-1.9270989E38F, 3.0924507E38F, 1.0030909E38F, -9.198304E37F}));
			assert (pack.y_GET() == 3.302794E38F);
			assert (pack.time_usec_GET() == 2775326177375823785L);
			assert (pack.x_GET() == -3.0727222E38F);
		});
		GroundControl.ATT_POS_MOCAP p138 = CommunicationChannel.new_ATT_POS_MOCAP();
		PH.setPack(p138);
		p138.q_SET(new float[]{-1.9270989E38F, 3.0924507E38F, 1.0030909E38F, -9.198304E37F}, 0);
		p138.time_usec_SET(2775326177375823785L);
		p138.y_SET(3.302794E38F);
		p138.z_SET(1.5664881E38F);
		p138.x_SET(-3.0727222E38F);
		CommunicationChannel.instance.send(p138);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_SET_ACTUATOR_CONTROL_TARGET.add((src, ph, pack) ->
		{
			assert (pack.target_system_GET() == (char) 131);
			assert (pack.target_component_GET() == (char) 165);
			assert (Arrays.equals(pack.controls_GET(), new float[]{-1.7929092E38F, 1.8961327E38F, 2.1023026E38F, 1.4581971E37F, 3.1453571E38F, -2.278684E38F, -2.2315407E38F, 1.0779242E38F}));
			assert (pack.time_usec_GET() == 5677858249340124461L);
			assert (pack.group_mlx_GET() == (char) 106);
		});
		GroundControl.SET_ACTUATOR_CONTROL_TARGET p139 = CommunicationChannel.new_SET_ACTUATOR_CONTROL_TARGET();
		PH.setPack(p139);
		p139.group_mlx_SET((char) 106);
		p139.time_usec_SET(5677858249340124461L);
		p139.target_component_SET((char) 165);
		p139.target_system_SET((char) 131);
		p139.controls_SET(new float[]{-1.7929092E38F, 1.8961327E38F, 2.1023026E38F, 1.4581971E37F, 3.1453571E38F, -2.278684E38F, -2.2315407E38F, 1.0779242E38F}, 0);
		CommunicationChannel.instance.send(p139);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_ACTUATOR_CONTROL_TARGET.add((src, ph, pack) ->
		{
			assert (pack.time_usec_GET() == 8935533102707561129L);
			assert (Arrays.equals(pack.controls_GET(), new float[]{-2.5139456E38F, 4.653536E37F, 2.9182323E38F, -3.3141737E38F, -2.5656277E38F, -1.7422452E38F, 1.1381028E38F, -9.832011E37F}));
			assert (pack.group_mlx_GET() == (char) 211);
		});
		GroundControl.ACTUATOR_CONTROL_TARGET p140 = CommunicationChannel.new_ACTUATOR_CONTROL_TARGET();
		PH.setPack(p140);
		p140.time_usec_SET(8935533102707561129L);
		p140.group_mlx_SET((char) 211);
		p140.controls_SET(new float[]{-2.5139456E38F, 4.653536E37F, 2.9182323E38F, -3.3141737E38F, -2.5656277E38F, -1.7422452E38F, 1.1381028E38F, -9.832011E37F}, 0);
		CommunicationChannel.instance.send(p140);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_ALTITUDE.add((src, ph, pack) ->
		{
			assert (pack.altitude_terrain_GET() == -1.7307935E38F);
			assert (pack.altitude_local_GET() == 3.0576692E38F);
			assert (pack.altitude_monotonic_GET() == 1.0066845E38F);
			assert (pack.altitude_relative_GET() == 1.6295223E38F);
			assert (pack.altitude_amsl_GET() == -3.1044404E38F);
			assert (pack.time_usec_GET() == 1841271749102899361L);
			assert (pack.bottom_clearance_GET() == 2.9950943E37F);
		});
		GroundControl.ALTITUDE p141 = CommunicationChannel.new_ALTITUDE();
		PH.setPack(p141);
		p141.altitude_local_SET(3.0576692E38F);
		p141.time_usec_SET(1841271749102899361L);
		p141.altitude_relative_SET(1.6295223E38F);
		p141.bottom_clearance_SET(2.9950943E37F);
		p141.altitude_terrain_SET(-1.7307935E38F);
		p141.altitude_amsl_SET(-3.1044404E38F);
		p141.altitude_monotonic_SET(1.0066845E38F);
		CommunicationChannel.instance.send(p141);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_RESOURCE_REQUEST.add((src, ph, pack) ->
		{
			assert (pack.transfer_type_GET() == (char) 17);
			assert (pack.uri_type_GET() == (char) 134);
			assert (pack.request_id_GET() == (char) 117);
			assert (Arrays.equals(pack.storage_GET(), new char[]{(char) 103, (char) 156, (char) 32, (char) 17, (char) 198, (char) 130, (char) 171, (char) 38, (char) 160, (char) 98, (char) 189, (char) 94, (char) 70, (char) 161, (char) 249, (char) 109, (char) 239, (char) 68, (char) 46, (char) 142, (char) 52, (char) 248, (char) 212, (char) 198, (char) 153, (char) 22, (char) 63, (char) 208, (char) 70, (char) 3, (char) 66, (char) 201, (char) 215, (char) 249, (char) 88, (char) 131, (char) 137, (char) 115, (char) 162, (char) 137, (char) 86, (char) 24, (char) 18, (char) 117, (char) 246, (char) 76, (char) 239, (char) 156, (char) 24, (char) 163, (char) 210, (char) 127, (char) 177, (char) 100, (char) 35, (char) 197, (char) 233, (char) 33, (char) 99, (char) 171, (char) 250, (char) 41, (char) 11, (char) 45, (char) 135, (char) 213, (char) 245, (char) 35, (char) 194, (char) 127, (char) 1, (char) 207, (char) 249, (char) 185, (char) 26, (char) 47, (char) 100, (char) 225, (char) 77, (char) 254, (char) 197, (char) 31, (char) 18, (char) 69, (char) 254, (char) 200, (char) 88, (char) 31, (char) 71, (char) 212, (char) 148, (char) 203, (char) 128, (char) 74, (char) 173, (char) 69, (char) 78, (char) 211, (char) 133, (char) 82, (char) 108, (char) 213, (char) 87, (char) 249, (char) 25, (char) 75, (char) 69, (char) 233, (char) 17, (char) 226, (char) 201, (char) 88, (char) 83, (char) 63, (char) 155, (char) 226, (char) 86, (char) 184, (char) 110, (char) 164}));
			assert (Arrays.equals(pack.uri_GET(), new char[]{(char) 255, (char) 155, (char) 121, (char) 12, (char) 130, (char) 15, (char) 148, (char) 28, (char) 41, (char) 117, (char) 199, (char) 24, (char) 178, (char) 1, (char) 243, (char) 221, (char) 41, (char) 22, (char) 53, (char) 30, (char) 32, (char) 153, (char) 43, (char) 23, (char) 224, (char) 238, (char) 203, (char) 110, (char) 170, (char) 84, (char) 145, (char) 155, (char) 149, (char) 245, (char) 110, (char) 21, (char) 41, (char) 23, (char) 99, (char) 114, (char) 153, (char) 213, (char) 164, (char) 5, (char) 88, (char) 250, (char) 186, (char) 199, (char) 166, (char) 226, (char) 151, (char) 170, (char) 95, (char) 238, (char) 146, (char) 53, (char) 45, (char) 26, (char) 228, (char) 246, (char) 231, (char) 207, (char) 240, (char) 82, (char) 75, (char) 89, (char) 79, (char) 137, (char) 91, (char) 81, (char) 5, (char) 190, (char) 95, (char) 31, (char) 210, (char) 54, (char) 164, (char) 228, (char) 38, (char) 114, (char) 204, (char) 146, (char) 60, (char) 159, (char) 228, (char) 237, (char) 240, (char) 24, (char) 103, (char) 170, (char) 34, (char) 99, (char) 37, (char) 219, (char) 112, (char) 124, (char) 195, (char) 17, (char) 18, (char) 223, (char) 114, (char) 83, (char) 2, (char) 96, (char) 15, (char) 0, (char) 40, (char) 50, (char) 5, (char) 76, (char) 121, (char) 189, (char) 153, (char) 19, (char) 234, (char) 189, (char) 163, (char) 160, (char) 195, (char) 165}));
		});
		GroundControl.RESOURCE_REQUEST p142 = CommunicationChannel.new_RESOURCE_REQUEST();
		PH.setPack(p142);
		p142.storage_SET(new char[]{(char) 103, (char) 156, (char) 32, (char) 17, (char) 198, (char) 130, (char) 171, (char) 38, (char) 160, (char) 98, (char) 189, (char) 94, (char) 70, (char) 161, (char) 249, (char) 109, (char) 239, (char) 68, (char) 46, (char) 142, (char) 52, (char) 248, (char) 212, (char) 198, (char) 153, (char) 22, (char) 63, (char) 208, (char) 70, (char) 3, (char) 66, (char) 201, (char) 215, (char) 249, (char) 88, (char) 131, (char) 137, (char) 115, (char) 162, (char) 137, (char) 86, (char) 24, (char) 18, (char) 117, (char) 246, (char) 76, (char) 239, (char) 156, (char) 24, (char) 163, (char) 210, (char) 127, (char) 177, (char) 100, (char) 35, (char) 197, (char) 233, (char) 33, (char) 99, (char) 171, (char) 250, (char) 41, (char) 11, (char) 45, (char) 135, (char) 213, (char) 245, (char) 35, (char) 194, (char) 127, (char) 1, (char) 207, (char) 249, (char) 185, (char) 26, (char) 47, (char) 100, (char) 225, (char) 77, (char) 254, (char) 197, (char) 31, (char) 18, (char) 69, (char) 254, (char) 200, (char) 88, (char) 31, (char) 71, (char) 212, (char) 148, (char) 203, (char) 128, (char) 74, (char) 173, (char) 69, (char) 78, (char) 211, (char) 133, (char) 82, (char) 108, (char) 213, (char) 87, (char) 249, (char) 25, (char) 75, (char) 69, (char) 233, (char) 17, (char) 226, (char) 201, (char) 88, (char) 83, (char) 63, (char) 155, (char) 226, (char) 86, (char) 184, (char) 110, (char) 164}, 0);
		p142.transfer_type_SET((char) 17);
		p142.request_id_SET((char) 117);
		p142.uri_SET(new char[]{(char) 255, (char) 155, (char) 121, (char) 12, (char) 130, (char) 15, (char) 148, (char) 28, (char) 41, (char) 117, (char) 199, (char) 24, (char) 178, (char) 1, (char) 243, (char) 221, (char) 41, (char) 22, (char) 53, (char) 30, (char) 32, (char) 153, (char) 43, (char) 23, (char) 224, (char) 238, (char) 203, (char) 110, (char) 170, (char) 84, (char) 145, (char) 155, (char) 149, (char) 245, (char) 110, (char) 21, (char) 41, (char) 23, (char) 99, (char) 114, (char) 153, (char) 213, (char) 164, (char) 5, (char) 88, (char) 250, (char) 186, (char) 199, (char) 166, (char) 226, (char) 151, (char) 170, (char) 95, (char) 238, (char) 146, (char) 53, (char) 45, (char) 26, (char) 228, (char) 246, (char) 231, (char) 207, (char) 240, (char) 82, (char) 75, (char) 89, (char) 79, (char) 137, (char) 91, (char) 81, (char) 5, (char) 190, (char) 95, (char) 31, (char) 210, (char) 54, (char) 164, (char) 228, (char) 38, (char) 114, (char) 204, (char) 146, (char) 60, (char) 159, (char) 228, (char) 237, (char) 240, (char) 24, (char) 103, (char) 170, (char) 34, (char) 99, (char) 37, (char) 219, (char) 112, (char) 124, (char) 195, (char) 17, (char) 18, (char) 223, (char) 114, (char) 83, (char) 2, (char) 96, (char) 15, (char) 0, (char) 40, (char) 50, (char) 5, (char) 76, (char) 121, (char) 189, (char) 153, (char) 19, (char) 234, (char) 189, (char) 163, (char) 160, (char) 195, (char) 165}, 0);
		p142.uri_type_SET((char) 134);
		CommunicationChannel.instance.send(p142);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_SCALED_PRESSURE3.add((src, ph, pack) ->
		{
			assert (pack.press_abs_GET() == -2.8412989E38F);
			assert (pack.temperature_GET() == (short) -26093);
			assert (pack.time_boot_ms_GET() == 3778106748L);
			assert (pack.press_diff_GET() == 2.2126902E37F);
		});
		GroundControl.SCALED_PRESSURE3 p143 = CommunicationChannel.new_SCALED_PRESSURE3();
		PH.setPack(p143);
		p143.temperature_SET((short) -26093);
		p143.press_diff_SET(2.2126902E37F);
		p143.press_abs_SET(-2.8412989E38F);
		p143.time_boot_ms_SET(3778106748L);
		CommunicationChannel.instance.send(p143);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_FOLLOW_TARGET.add((src, ph, pack) ->
		{
			assert (pack.lon_GET() == -1584643600);
			assert (Arrays.equals(pack.vel_GET(), new float[]{2.4034824E38F, -2.2781823E38F, 1.3469923E37F}));
			assert (Arrays.equals(pack.rates_GET(), new float[]{1.5511073E38F, -3.278031E38F, -1.5609824E38F}));
			assert (Arrays.equals(pack.attitude_q_GET(), new float[]{3.2564731E38F, 1.2713348E38F, -1.5722934E37F, -6.2081697E37F}));
			assert (pack.custom_state_GET() == 1311519807252032123L);
			assert (pack.lat_GET() == -809483207);
			assert (pack.est_capabilities_GET() == (char) 33);
			assert (pack.alt_GET() == -2.9996978E38F);
			assert (Arrays.equals(pack.acc_GET(), new float[]{1.0184192E38F, 1.6013543E37F, -2.7534365E38F}));
			assert (Arrays.equals(pack.position_cov_GET(), new float[]{-1.0286061E38F, -1.8198422E38F, 4.5425657E36F}));
			assert (pack.timestamp_GET() == 3770884613347662719L);
		});
		GroundControl.FOLLOW_TARGET p144 = CommunicationChannel.new_FOLLOW_TARGET();
		PH.setPack(p144);
		p144.rates_SET(new float[]{1.5511073E38F, -3.278031E38F, -1.5609824E38F}, 0);
		p144.custom_state_SET(1311519807252032123L);
		p144.est_capabilities_SET((char) 33);
		p144.lat_SET(-809483207);
		p144.attitude_q_SET(new float[]{3.2564731E38F, 1.2713348E38F, -1.5722934E37F, -6.2081697E37F}, 0);
		p144.alt_SET(-2.9996978E38F);
		p144.timestamp_SET(3770884613347662719L);
		p144.acc_SET(new float[]{1.0184192E38F, 1.6013543E37F, -2.7534365E38F}, 0);
		p144.position_cov_SET(new float[]{-1.0286061E38F, -1.8198422E38F, 4.5425657E36F}, 0);
		p144.lon_SET(-1584643600);
		p144.vel_SET(new float[]{2.4034824E38F, -2.2781823E38F, 1.3469923E37F}, 0);
		CommunicationChannel.instance.send(p144);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_CONTROL_SYSTEM_STATE.add((src, ph, pack) ->
		{
			assert (pack.roll_rate_GET() == -2.0953283E38F);
			assert (Arrays.equals(pack.q_GET(), new float[]{1.2875779E38F, 2.5197523E38F, 3.1673972E38F, -6.0121585E37F}));
			assert (pack.y_acc_GET() == -1.06097545E37F);
			assert (Arrays.equals(pack.vel_variance_GET(), new float[]{-6.953182E35F, -1.0171868E38F, -7.712531E37F}));
			assert (pack.x_pos_GET() == -2.7957547E38F);
			assert (pack.pitch_rate_GET() == 2.2719187E38F);
			assert (pack.yaw_rate_GET() == -3.6409144E37F);
			assert (pack.x_vel_GET() == 6.18584E37F);
			assert (pack.z_acc_GET() == -1.7970274E38F);
			assert (pack.z_pos_GET() == 2.1228919E38F);
			assert (Arrays.equals(pack.pos_variance_GET(), new float[]{8.909881E37F, -1.81365E38F, 3.2253368E38F}));
			assert (pack.z_vel_GET() == -7.294233E37F);
			assert (pack.time_usec_GET() == 8004669048033373399L);
			assert (pack.airspeed_GET() == 5.064352E37F);
			assert (pack.y_vel_GET() == 1.807825E38F);
			assert (pack.x_acc_GET() == 1.6631932E38F);
			assert (pack.y_pos_GET() == 3.0772523E38F);
		});
		GroundControl.CONTROL_SYSTEM_STATE p146 = CommunicationChannel.new_CONTROL_SYSTEM_STATE();
		PH.setPack(p146);
		p146.pitch_rate_SET(2.2719187E38F);
		p146.y_pos_SET(3.0772523E38F);
		p146.pos_variance_SET(new float[]{8.909881E37F, -1.81365E38F, 3.2253368E38F}, 0);
		p146.z_vel_SET(-7.294233E37F);
		p146.roll_rate_SET(-2.0953283E38F);
		p146.x_acc_SET(1.6631932E38F);
		p146.yaw_rate_SET(-3.6409144E37F);
		p146.x_pos_SET(-2.7957547E38F);
		p146.y_acc_SET(-1.06097545E37F);
		p146.vel_variance_SET(new float[]{-6.953182E35F, -1.0171868E38F, -7.712531E37F}, 0);
		p146.z_acc_SET(-1.7970274E38F);
		p146.x_vel_SET(6.18584E37F);
		p146.y_vel_SET(1.807825E38F);
		p146.time_usec_SET(8004669048033373399L);
		p146.z_pos_SET(2.1228919E38F);
		p146.airspeed_SET(5.064352E37F);
		p146.q_SET(new float[]{1.2875779E38F, 2.5197523E38F, 3.1673972E38F, -6.0121585E37F}, 0);
		CommunicationChannel.instance.send(p146);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_BATTERY_STATUS.add((src, ph, pack) ->
		{
			assert (pack.battery_function_GET() == MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_AVIONICS);
			assert (pack.type_GET() == MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_LIFE);
			assert (pack.energy_consumed_GET() == 37211393);
			assert (pack.battery_remaining_GET() == (byte) 47);
			assert (pack.current_battery_GET() == (short) -20931);
			assert (pack.current_consumed_GET() == -1860326279);
			assert (Arrays.equals(pack.voltages_GET(), new char[]{(char) 2019, (char) 63312, (char) 39163, (char) 31265, (char) 12746, (char) 30568, (char) 35726, (char) 58086, (char) 28646, (char) 30956}));
			assert (pack.temperature_GET() == (short) -1143);
			assert (pack.id_GET() == (char) 89);
		});
		GroundControl.BATTERY_STATUS p147 = CommunicationChannel.new_BATTERY_STATUS();
		PH.setPack(p147);
		p147.type_SET(MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_LIFE);
		p147.id_SET((char) 89);
		p147.current_battery_SET((short) -20931);
		p147.voltages_SET(new char[]{(char) 2019, (char) 63312, (char) 39163, (char) 31265, (char) 12746, (char) 30568, (char) 35726, (char) 58086, (char) 28646, (char) 30956}, 0);
		p147.temperature_SET((short) -1143);
		p147.battery_remaining_SET((byte) 47);
		p147.current_consumed_SET(-1860326279);
		p147.energy_consumed_SET(37211393);
		p147.battery_function_SET(MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_AVIONICS);
		CommunicationChannel.instance.send(p147);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_AUTOPILOT_VERSION.add((src, ph, pack) ->
		{
			assert (pack.middleware_sw_version_GET() == 3009902499L);
			assert (pack.uid_GET() == 8865933288357205124L);
			assert (pack.product_id_GET() == (char) 44741);
			assert (Arrays.equals(pack.flight_custom_version_GET(), new char[]{(char) 254, (char) 66, (char) 143, (char) 159, (char) 193, (char) 78, (char) 16, (char) 177}));
			assert (Arrays.equals(pack.os_custom_version_GET(), new char[]{(char) 182, (char) 169, (char) 203, (char) 211, (char) 143, (char) 92, (char) 77, (char) 183}));
			assert (Arrays.equals(pack.uid2_TRY(ph), new char[]{(char) 9, (char) 247, (char) 196, (char) 174, (char) 14, (char) 190, (char) 34, (char) 209, (char) 76, (char) 11, (char) 247, (char) 112, (char) 6, (char) 10, (char) 183, (char) 166, (char) 47, (char) 202}));
			assert (pack.vendor_id_GET() == (char) 23381);
			assert (pack.flight_sw_version_GET() == 3199056108L);
			assert (pack.board_version_GET() == 3584570930L);
			assert (pack.capabilities_GET() == MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_COMMAND_INT);
			assert (Arrays.equals(pack.middleware_custom_version_GET(), new char[]{(char) 188, (char) 60, (char) 177, (char) 100, (char) 253, (char) 190, (char) 216, (char) 13}));
			assert (pack.os_sw_version_GET() == 489410736L);
		});
		GroundControl.AUTOPILOT_VERSION p148 = CommunicationChannel.new_AUTOPILOT_VERSION();
		PH.setPack(p148);
		p148.vendor_id_SET((char) 23381);
		p148.uid2_SET(new char[]{(char) 9, (char) 247, (char) 196, (char) 174, (char) 14, (char) 190, (char) 34, (char) 209, (char) 76, (char) 11, (char) 247, (char) 112, (char) 6, (char) 10, (char) 183, (char) 166, (char) 47, (char) 202}, 0, PH);
		p148.middleware_custom_version_SET(new char[]{(char) 188, (char) 60, (char) 177, (char) 100, (char) 253, (char) 190, (char) 216, (char) 13}, 0);
		p148.os_custom_version_SET(new char[]{(char) 182, (char) 169, (char) 203, (char) 211, (char) 143, (char) 92, (char) 77, (char) 183}, 0);
		p148.uid_SET(8865933288357205124L);
		p148.capabilities_SET(MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_COMMAND_INT);
		p148.os_sw_version_SET(489410736L);
		p148.board_version_SET(3584570930L);
		p148.product_id_SET((char) 44741);
		p148.flight_sw_version_SET(3199056108L);
		p148.flight_custom_version_SET(new char[]{(char) 254, (char) 66, (char) 143, (char) 159, (char) 193, (char) 78, (char) 16, (char) 177}, 0);
		p148.middleware_sw_version_SET(3009902499L);
		CommunicationChannel.instance.send(p148);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_LANDING_TARGET.add((src, ph, pack) ->
		{
			assert (pack.angle_y_GET() == 2.4018446E38F);
			assert (pack.size_y_GET() == -3.154345E38F);
			assert (pack.angle_x_GET() == 1.2659279E38F);
			assert (Arrays.equals(pack.q_TRY(ph), new float[]{1.2456489E38F, -8.798334E37F, 2.2942012E38F, 1.0134556E38F}));
			assert (pack.size_x_GET() == 2.2837969E38F);
			assert (pack.type_GET() == LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_VISION_FIDUCIAL);
			assert (pack.frame_GET() == MAV_FRAME.MAV_FRAME_MISSION);
			assert (pack.position_valid_TRY(ph) == (char) 166);
			assert (pack.target_num_GET() == (char) 203);
			assert (pack.distance_GET() == -3.1972712E38F);
			assert (pack.time_usec_GET() == 8035556356847609653L);
			assert (pack.z_TRY(ph) == -2.7938474E38F);
			assert (pack.y_TRY(ph) == 1.0708956E38F);
			assert (pack.x_TRY(ph) == 9.94418E36F);
		});
		GroundControl.LANDING_TARGET p149 = CommunicationChannel.new_LANDING_TARGET();
		PH.setPack(p149);
		p149.angle_x_SET(1.2659279E38F);
		p149.size_y_SET(-3.154345E38F);
		p149.target_num_SET((char) 203);
		p149.time_usec_SET(8035556356847609653L);
		p149.x_SET(9.94418E36F, PH);
		p149.y_SET(1.0708956E38F, PH);
		p149.angle_y_SET(2.4018446E38F);
		p149.z_SET(-2.7938474E38F, PH);
		p149.type_SET(LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_VISION_FIDUCIAL);
		p149.frame_SET(MAV_FRAME.MAV_FRAME_MISSION);
		p149.distance_SET(-3.1972712E38F);
		p149.size_x_SET(2.2837969E38F);
		p149.position_valid_SET((char) 166, PH);
		p149.q_SET(new float[]{1.2456489E38F, -8.798334E37F, 2.2942012E38F, 1.0134556E38F}, 0, PH);
		CommunicationChannel.instance.send(p149);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		TestChannel.instance.on_FLEXIFUNCTION_SET.add((src, ph, pack) ->
		{
			assert (pack.target_system_GET() == (char) 112);
			assert (pack.target_component_GET() == (char) 76);
		});
		GroundControl.FLEXIFUNCTION_SET p150 = CommunicationChannel.new_FLEXIFUNCTION_SET();
		PH.setPack(p150);
		p150.target_system_SET((char) 112);
		p150.target_component_SET((char) 76);
		CommunicationChannel.instance.send(p150);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_FLEXIFUNCTION_READ_REQ.add((src, ph, pack) ->
		{
			assert (pack.target_system_GET() == (char) 150);
			assert (pack.read_req_type_GET() == (short) -29246);
			assert (pack.target_component_GET() == (char) 56);
			assert (pack.data_index_GET() == (short) 27109);
		});
		GroundControl.FLEXIFUNCTION_READ_REQ p151 = CommunicationChannel.new_FLEXIFUNCTION_READ_REQ();
		PH.setPack(p151);
		p151.target_system_SET((char) 150);
		p151.read_req_type_SET((short) -29246);
		p151.target_component_SET((char) 56);
		p151.data_index_SET((short) 27109);
		CommunicationChannel.instance.send(p151);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_FLEXIFUNCTION_BUFFER_FUNCTION.add((src, ph, pack) ->
		{
			assert (pack.data_address_GET() == (char) 24374);
			assert (Arrays.equals(pack.data__GET(), new byte[]{(byte) 11, (byte) -72, (byte) -74, (byte) -104, (byte) -21, (byte) -33, (byte) -53, (byte) -98, (byte) -79, (byte) 40, (byte) -53, (byte) -77, (byte) 98, (byte) 118, (byte) -50, (byte) -24, (byte) 63, (byte) -1, (byte) -53, (byte) -93, (byte) -124, (byte) 105, (byte) 92, (byte) 106, (byte) 21, (byte) -76, (byte) -5, (byte) 114, (byte) -8, (byte) 98, (byte) 32, (byte) 65, (byte) -16, (byte) 114, (byte) 6, (byte) -1, (byte) -88, (byte) 40, (byte) -13, (byte) -93, (byte) 11, (byte) -36, (byte) 64, (byte) -55, (byte) -38, (byte) -85, (byte) 93, (byte) -50}));
			assert (pack.func_index_GET() == (char) 35547);
			assert (pack.target_system_GET() == (char) 202);
			assert (pack.func_count_GET() == (char) 9682);
			assert (pack.data_size_GET() == (char) 48373);
			assert (pack.target_component_GET() == (char) 247);
		});
		GroundControl.FLEXIFUNCTION_BUFFER_FUNCTION p152 = CommunicationChannel.new_FLEXIFUNCTION_BUFFER_FUNCTION();
		PH.setPack(p152);
		p152.func_index_SET((char) 35547);
		p152.target_system_SET((char) 202);
		p152.data_address_SET((char) 24374);
		p152.func_count_SET((char) 9682);
		p152.data__SET(new byte[]{(byte) 11, (byte) -72, (byte) -74, (byte) -104, (byte) -21, (byte) -33, (byte) -53, (byte) -98, (byte) -79, (byte) 40, (byte) -53, (byte) -77, (byte) 98, (byte) 118, (byte) -50, (byte) -24, (byte) 63, (byte) -1, (byte) -53, (byte) -93, (byte) -124, (byte) 105, (byte) 92, (byte) 106, (byte) 21, (byte) -76, (byte) -5, (byte) 114, (byte) -8, (byte) 98, (byte) 32, (byte) 65, (byte) -16, (byte) 114, (byte) 6, (byte) -1, (byte) -88, (byte) 40, (byte) -13, (byte) -93, (byte) 11, (byte) -36, (byte) 64, (byte) -55, (byte) -38, (byte) -85, (byte) 93, (byte) -50}, 0);
		p152.data_size_SET((char) 48373);
		p152.target_component_SET((char) 247);
		CommunicationChannel.instance.send(p152);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_FLEXIFUNCTION_BUFFER_FUNCTION_ACK.add((src, ph, pack) ->
		{
			assert (pack.target_component_GET() == (char) 232);
			assert (pack.result_GET() == (char) 57905);
			assert (pack.target_system_GET() == (char) 221);
			assert (pack.func_index_GET() == (char) 14264);
		});
		GroundControl.FLEXIFUNCTION_BUFFER_FUNCTION_ACK p153 = CommunicationChannel.new_FLEXIFUNCTION_BUFFER_FUNCTION_ACK();
		PH.setPack(p153);
		p153.result_SET((char) 57905);
		p153.func_index_SET((char) 14264);
		p153.target_component_SET((char) 232);
		p153.target_system_SET((char) 221);
		CommunicationChannel.instance.send(p153);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_FLEXIFUNCTION_DIRECTORY.add((src, ph, pack) ->
		{
			assert (pack.target_system_GET() == (char) 22);
			assert (pack.directory_type_GET() == (char) 247);
			assert (pack.count_GET() == (char) 88);
			assert (Arrays.equals(pack.directory_data_GET(), new byte[]{(byte) -100, (byte) -81, (byte) -45, (byte) -46, (byte) 125, (byte) 75, (byte) -23, (byte) -50, (byte) -30, (byte) -56, (byte) -31, (byte) -76, (byte) 113, (byte) -50, (byte) -91, (byte) 103, (byte) -44, (byte) -91, (byte) 69, (byte) -103, (byte) -93, (byte) 50, (byte) 63, (byte) -106, (byte) 64, (byte) 20, (byte) 90, (byte) 107, (byte) 34, (byte) -43, (byte) 32, (byte) -79, (byte) -125, (byte) -40, (byte) -126, (byte) -76, (byte) -116, (byte) 73, (byte) -127, (byte) 97, (byte) 127, (byte) 115, (byte) -62, (byte) 35, (byte) 28, (byte) -63, (byte) -26, (byte) 83}));
			assert (pack.start_index_GET() == (char) 208);
			assert (pack.target_component_GET() == (char) 232);
		});
		GroundControl.FLEXIFUNCTION_DIRECTORY p155 = CommunicationChannel.new_FLEXIFUNCTION_DIRECTORY();
		PH.setPack(p155);
		p155.directory_data_SET(new byte[]{(byte) -100, (byte) -81, (byte) -45, (byte) -46, (byte) 125, (byte) 75, (byte) -23, (byte) -50, (byte) -30, (byte) -56, (byte) -31, (byte) -76, (byte) 113, (byte) -50, (byte) -91, (byte) 103, (byte) -44, (byte) -91, (byte) 69, (byte) -103, (byte) -93, (byte) 50, (byte) 63, (byte) -106, (byte) 64, (byte) 20, (byte) 90, (byte) 107, (byte) 34, (byte) -43, (byte) 32, (byte) -79, (byte) -125, (byte) -40, (byte) -126, (byte) -76, (byte) -116, (byte) 73, (byte) -127, (byte) 97, (byte) 127, (byte) 115, (byte) -62, (byte) 35, (byte) 28, (byte) -63, (byte) -26, (byte) 83}, 0);
		p155.start_index_SET((char) 208);
		p155.target_component_SET((char) 232);
		p155.target_system_SET((char) 22);
		p155.directory_type_SET((char) 247);
		p155.count_SET((char) 88);
		CommunicationChannel.instance.send(p155);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_FLEXIFUNCTION_DIRECTORY_ACK.add((src, ph, pack) ->
		{
			assert (pack.directory_type_GET() == (char) 100);
			assert (pack.result_GET() == (char) 11886);
			assert (pack.target_component_GET() == (char) 119);
			assert (pack.start_index_GET() == (char) 8);
			assert (pack.target_system_GET() == (char) 43);
			assert (pack.count_GET() == (char) 73);
		});
		GroundControl.FLEXIFUNCTION_DIRECTORY_ACK p156 = CommunicationChannel.new_FLEXIFUNCTION_DIRECTORY_ACK();
		PH.setPack(p156);
		p156.start_index_SET((char) 8);
		p156.target_system_SET((char) 43);
		p156.count_SET((char) 73);
		p156.target_component_SET((char) 119);
		p156.directory_type_SET((char) 100);
		p156.result_SET((char) 11886);
		CommunicationChannel.instance.send(p156);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_FLEXIFUNCTION_COMMAND.add((src, ph, pack) ->
		{
			assert (pack.target_component_GET() == (char) 132);
			assert (pack.target_system_GET() == (char) 197);
			assert (pack.command_type_GET() == (char) 74);
		});
		GroundControl.FLEXIFUNCTION_COMMAND p157 = CommunicationChannel.new_FLEXIFUNCTION_COMMAND();
		PH.setPack(p157);
		p157.target_system_SET((char) 197);
		p157.target_component_SET((char) 132);
		p157.command_type_SET((char) 74);
		CommunicationChannel.instance.send(p157);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_FLEXIFUNCTION_COMMAND_ACK.add((src, ph, pack) ->
		{
			assert (pack.command_type_GET() == (char) 38539);
			assert (pack.result_GET() == (char) 63675);
		});
		GroundControl.FLEXIFUNCTION_COMMAND_ACK p158 = CommunicationChannel.new_FLEXIFUNCTION_COMMAND_ACK();
		PH.setPack(p158);
		p158.result_SET((char) 63675);
		p158.command_type_SET((char) 38539);
		CommunicationChannel.instance.send(p158);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_SERIAL_UDB_EXTRA_F2_A.add((src, ph, pack) ->
		{
			assert (pack.sue_rmat2_GET() == (short) -10562);
			assert (pack.sue_svs_GET() == (short) -25594);
			assert (pack.sue_rmat3_GET() == (short) 17406);
			assert (pack.sue_altitude_GET() == -1883904830);
			assert (pack.sue_magFieldEarth0_GET() == (short) 26190);
			assert (pack.sue_rmat1_GET() == (short) 11257);
			assert (pack.sue_rmat6_GET() == (short) -26325);
			assert (pack.sue_rmat0_GET() == (short) 3194);
			assert (pack.sue_waypoint_index_GET() == (char) 44433);
			assert (pack.sue_magFieldEarth2_GET() == (short) 31275);
			assert (pack.sue_estimated_wind_0_GET() == (short) 16260);
			assert (pack.sue_cpu_load_GET() == (char) 55665);
			assert (pack.sue_sog_GET() == (short) 30118);
			assert (pack.sue_hdop_GET() == (short) -6564);
			assert (pack.sue_rmat7_GET() == (short) -119);
			assert (pack.sue_estimated_wind_1_GET() == (short) 31871);
			assert (pack.sue_magFieldEarth1_GET() == (short) 28960);
			assert (pack.sue_air_speed_3DIMU_GET() == (char) 23916);
			assert (pack.sue_rmat8_GET() == (short) -7796);
			assert (pack.sue_latitude_GET() == -1347048885);
			assert (pack.sue_status_GET() == (char) 0);
			assert (pack.sue_longitude_GET() == -500040419);
			assert (pack.sue_rmat5_GET() == (short) -10159);
			assert (pack.sue_rmat4_GET() == (short) -29156);
			assert (pack.sue_cog_GET() == (char) 10433);
			assert (pack.sue_time_GET() == 4063256658L);
			assert (pack.sue_estimated_wind_2_GET() == (short) -24288);
		});
		GroundControl.SERIAL_UDB_EXTRA_F2_A p170 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F2_A();
		PH.setPack(p170);
		p170.sue_sog_SET((short) 30118);
		p170.sue_magFieldEarth2_SET((short) 31275);
		p170.sue_time_SET(4063256658L);
		p170.sue_air_speed_3DIMU_SET((char) 23916);
		p170.sue_rmat7_SET((short) -119);
		p170.sue_magFieldEarth1_SET((short) 28960);
		p170.sue_status_SET((char) 0);
		p170.sue_rmat5_SET((short) -10159);
		p170.sue_longitude_SET(-500040419);
		p170.sue_estimated_wind_0_SET((short) 16260);
		p170.sue_estimated_wind_1_SET((short) 31871);
		p170.sue_cpu_load_SET((char) 55665);
		p170.sue_latitude_SET(-1347048885);
		p170.sue_rmat6_SET((short) -26325);
		p170.sue_rmat1_SET((short) 11257);
		p170.sue_estimated_wind_2_SET((short) -24288);
		p170.sue_rmat3_SET((short) 17406);
		p170.sue_rmat0_SET((short) 3194);
		p170.sue_rmat4_SET((short) -29156);
		p170.sue_rmat8_SET((short) -7796);
		p170.sue_hdop_SET((short) -6564);
		p170.sue_cog_SET((char) 10433);
		p170.sue_magFieldEarth0_SET((short) 26190);
		p170.sue_rmat2_SET((short) -10562);
		p170.sue_altitude_SET(-1883904830);
		p170.sue_svs_SET((short) -25594);
		p170.sue_waypoint_index_SET((char) 44433);
		CommunicationChannel.instance.send(p170);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_SERIAL_UDB_EXTRA_F2_B.add((src, ph, pack) ->
		{
			assert (pack.sue_pwm_output_6_GET() == (short) -22265);
			assert (pack.sue_desired_height_GET() == (short) -15476);
			assert (pack.sue_waypoint_goal_y_GET() == (short) 5508);
			assert (pack.sue_imu_location_y_GET() == (short) -3514);
			assert (pack.sue_pwm_output_8_GET() == (short) -9862);
			assert (pack.sue_pwm_input_3_GET() == (short) 13708);
			assert (pack.sue_pwm_output_4_GET() == (short) 15020);
			assert (pack.sue_pwm_input_10_GET() == (short) -19154);
			assert (pack.sue_imu_location_z_GET() == (short) 19950);
			assert (pack.sue_bat_amp_GET() == (short) 6037);
			assert (pack.sue_imu_location_x_GET() == (short) 6395);
			assert (pack.sue_pwm_output_1_GET() == (short) 3874);
			assert (pack.sue_pwm_output_12_GET() == (short) -27727);
			assert (pack.sue_pwm_output_5_GET() == (short) -25737);
			assert (pack.sue_imu_velocity_x_GET() == (short) -624);
			assert (pack.sue_aero_z_GET() == (short) -8826);
			assert (pack.sue_pwm_input_9_GET() == (short) 15150);
			assert (pack.sue_location_error_earth_x_GET() == (short) 32167);
			assert (pack.sue_pwm_output_10_GET() == (short) 24759);
			assert (pack.sue_pwm_input_12_GET() == (short) -8918);
			assert (pack.sue_pwm_output_3_GET() == (short) 22015);
			assert (pack.sue_aero_x_GET() == (short) -623);
			assert (pack.sue_imu_velocity_y_GET() == (short) -24923);
			assert (pack.sue_pwm_input_2_GET() == (short) 12564);
			assert (pack.sue_pwm_output_7_GET() == (short) -14456);
			assert (pack.sue_pwm_input_6_GET() == (short) -10020);
			assert (pack.sue_barom_temp_GET() == (short) -20012);
			assert (pack.sue_pwm_input_5_GET() == (short) -28271);
			assert (pack.sue_location_error_earth_z_GET() == (short) -9536);
			assert (pack.sue_osc_fails_GET() == (short) 4330);
			assert (pack.sue_pwm_output_9_GET() == (short) 21783);
			assert (pack.sue_bat_amp_hours_GET() == (short) 25342);
			assert (pack.sue_pwm_input_1_GET() == (short) -18420);
			assert (pack.sue_flags_GET() == 3057671917L);
			assert (pack.sue_waypoint_goal_z_GET() == (short) -22982);
			assert (pack.sue_pwm_output_2_GET() == (short) -214);
			assert (pack.sue_time_GET() == 2788135766L);
			assert (pack.sue_pwm_output_11_GET() == (short) -24781);
			assert (pack.sue_aero_y_GET() == (short) 1095);
			assert (pack.sue_barom_press_GET() == 1985110494);
			assert (pack.sue_pwm_input_8_GET() == (short) 23999);
			assert (pack.sue_pwm_input_7_GET() == (short) -5522);
			assert (pack.sue_imu_velocity_z_GET() == (short) -10980);
			assert (pack.sue_location_error_earth_y_GET() == (short) -3829);
			assert (pack.sue_memory_stack_free_GET() == (short) 13541);
			assert (pack.sue_bat_volt_GET() == (short) 6873);
			assert (pack.sue_pwm_input_4_GET() == (short) 671);
			assert (pack.sue_waypoint_goal_x_GET() == (short) -24490);
			assert (pack.sue_pwm_input_11_GET() == (short) -29082);
			assert (pack.sue_barom_alt_GET() == -963105838);
		});
		GroundControl.SERIAL_UDB_EXTRA_F2_B p171 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F2_B();
		PH.setPack(p171);
		p171.sue_pwm_input_1_SET((short) -18420);
		p171.sue_barom_press_SET(1985110494);
		p171.sue_pwm_input_8_SET((short) 23999);
		p171.sue_aero_y_SET((short) 1095);
		p171.sue_pwm_input_4_SET((short) 671);
		p171.sue_waypoint_goal_z_SET((short) -22982);
		p171.sue_imu_location_y_SET((short) -3514);
		p171.sue_imu_velocity_z_SET((short) -10980);
		p171.sue_pwm_output_11_SET((short) -24781);
		p171.sue_pwm_output_3_SET((short) 22015);
		p171.sue_aero_x_SET((short) -623);
		p171.sue_desired_height_SET((short) -15476);
		p171.sue_waypoint_goal_y_SET((short) 5508);
		p171.sue_flags_SET(3057671917L);
		p171.sue_imu_velocity_x_SET((short) -624);
		p171.sue_time_SET(2788135766L);
		p171.sue_bat_volt_SET((short) 6873);
		p171.sue_pwm_output_7_SET((short) -14456);
		p171.sue_pwm_output_5_SET((short) -25737);
		p171.sue_location_error_earth_z_SET((short) -9536);
		p171.sue_location_error_earth_y_SET((short) -3829);
		p171.sue_pwm_output_8_SET((short) -9862);
		p171.sue_pwm_input_10_SET((short) -19154);
		p171.sue_pwm_input_7_SET((short) -5522);
		p171.sue_waypoint_goal_x_SET((short) -24490);
		p171.sue_pwm_input_3_SET((short) 13708);
		p171.sue_barom_temp_SET((short) -20012);
		p171.sue_pwm_input_2_SET((short) 12564);
		p171.sue_pwm_output_2_SET((short) -214);
		p171.sue_memory_stack_free_SET((short) 13541);
		p171.sue_barom_alt_SET(-963105838);
		p171.sue_pwm_output_12_SET((short) -27727);
		p171.sue_bat_amp_SET((short) 6037);
		p171.sue_pwm_input_11_SET((short) -29082);
		p171.sue_imu_location_x_SET((short) 6395);
		p171.sue_pwm_output_9_SET((short) 21783);
		p171.sue_osc_fails_SET((short) 4330);
		p171.sue_pwm_input_5_SET((short) -28271);
		p171.sue_pwm_input_6_SET((short) -10020);
		p171.sue_imu_location_z_SET((short) 19950);
		p171.sue_pwm_output_6_SET((short) -22265);
		p171.sue_imu_velocity_y_SET((short) -24923);
		p171.sue_location_error_earth_x_SET((short) 32167);
		p171.sue_aero_z_SET((short) -8826);
		p171.sue_pwm_input_12_SET((short) -8918);
		p171.sue_pwm_input_9_SET((short) 15150);
		p171.sue_bat_amp_hours_SET((short) 25342);
		p171.sue_pwm_output_1_SET((short) 3874);
		p171.sue_pwm_output_10_SET((short) 24759);
		p171.sue_pwm_output_4_SET((short) 15020);
		CommunicationChannel.instance.send(p171);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_SERIAL_UDB_EXTRA_F4.add((src, ph, pack) ->
		{
			assert (pack.sue_YAW_STABILIZATION_RUDDER_GET() == (char) 44);
			assert (pack.sue_ROLL_STABILIZATION_AILERONS_GET() == (char) 39);
			assert (pack.sue_ROLL_STABILIZATION_RUDDER_GET() == (char) 61);
			assert (pack.sue_RACING_MODE_GET() == (char) 176);
			assert (pack.sue_ALTITUDEHOLD_STABILIZED_GET() == (char) 101);
			assert (pack.sue_YAW_STABILIZATION_AILERON_GET() == (char) 210);
			assert (pack.sue_PITCH_STABILIZATION_GET() == (char) 77);
			assert (pack.sue_AILERON_NAVIGATION_GET() == (char) 177);
			assert (pack.sue_ALTITUDEHOLD_WAYPOINT_GET() == (char) 27);
			assert (pack.sue_RUDDER_NAVIGATION_GET() == (char) 209);
		});
		GroundControl.SERIAL_UDB_EXTRA_F4 p172 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F4();
		PH.setPack(p172);
		p172.sue_ROLL_STABILIZATION_RUDDER_SET((char) 61);
		p172.sue_RUDDER_NAVIGATION_SET((char) 209);
		p172.sue_ROLL_STABILIZATION_AILERONS_SET((char) 39);
		p172.sue_RACING_MODE_SET((char) 176);
		p172.sue_YAW_STABILIZATION_AILERON_SET((char) 210);
		p172.sue_ALTITUDEHOLD_WAYPOINT_SET((char) 27);
		p172.sue_ALTITUDEHOLD_STABILIZED_SET((char) 101);
		p172.sue_AILERON_NAVIGATION_SET((char) 177);
		p172.sue_YAW_STABILIZATION_RUDDER_SET((char) 44);
		p172.sue_PITCH_STABILIZATION_SET((char) 77);
		CommunicationChannel.instance.send(p172);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_SERIAL_UDB_EXTRA_F5.add((src, ph, pack) ->
		{
			assert (pack.sue_ROLLKP_GET() == -3.4899552E37F);
			assert (pack.sue_YAWKP_AILERON_GET() == -2.617855E38F);
			assert (pack.sue_ROLLKD_GET() == -3.0942325E38F);
			assert (pack.sue_YAWKD_AILERON_GET() == -9.97232E37F);
		});
		GroundControl.SERIAL_UDB_EXTRA_F5 p173 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F5();
		PH.setPack(p173);
		p173.sue_YAWKD_AILERON_SET(-9.97232E37F);
		p173.sue_ROLLKD_SET(-3.0942325E38F);
		p173.sue_YAWKP_AILERON_SET(-2.617855E38F);
		p173.sue_ROLLKP_SET(-3.4899552E37F);
		CommunicationChannel.instance.send(p173);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_SERIAL_UDB_EXTRA_F6.add((src, ph, pack) ->
		{
			assert (pack.sue_ELEVATOR_BOOST_GET() == -2.0707983E38F);
			assert (pack.sue_PITCHKD_GET() == 4.8158905E36F);
			assert (pack.sue_RUDDER_ELEV_MIX_GET() == -3.1151179E38F);
			assert (pack.sue_PITCHGAIN_GET() == -1.2163011E38F);
			assert (pack.sue_ROLL_ELEV_MIX_GET() == -3.396328E38F);
		});
		GroundControl.SERIAL_UDB_EXTRA_F6 p174 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F6();
		PH.setPack(p174);
		p174.sue_PITCHKD_SET(4.8158905E36F);
		p174.sue_PITCHGAIN_SET(-1.2163011E38F);
		p174.sue_RUDDER_ELEV_MIX_SET(-3.1151179E38F);
		p174.sue_ELEVATOR_BOOST_SET(-2.0707983E38F);
		p174.sue_ROLL_ELEV_MIX_SET(-3.396328E38F);
		CommunicationChannel.instance.send(p174);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_SERIAL_UDB_EXTRA_F7.add((src, ph, pack) ->
		{
			assert (pack.sue_ROLLKP_RUDDER_GET() == -4.2776793E37F);
			assert (pack.sue_YAWKD_RUDDER_GET() == -2.704202E38F);
			assert (pack.sue_YAWKP_RUDDER_GET() == -3.0228228E38F);
			assert (pack.sue_ROLLKD_RUDDER_GET() == -2.7703293E38F);
			assert (pack.sue_RTL_PITCH_DOWN_GET() == -2.0160847E38F);
			assert (pack.sue_RUDDER_BOOST_GET() == -4.186699E37F);
		});
		GroundControl.SERIAL_UDB_EXTRA_F7 p175 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F7();
		PH.setPack(p175);
		p175.sue_RUDDER_BOOST_SET(-4.186699E37F);
		p175.sue_RTL_PITCH_DOWN_SET(-2.0160847E38F);
		p175.sue_ROLLKP_RUDDER_SET(-4.2776793E37F);
		p175.sue_ROLLKD_RUDDER_SET(-2.7703293E38F);
		p175.sue_YAWKP_RUDDER_SET(-3.0228228E38F);
		p175.sue_YAWKD_RUDDER_SET(-2.704202E38F);
		CommunicationChannel.instance.send(p175);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_SERIAL_UDB_EXTRA_F8.add((src, ph, pack) ->
		{
			assert (pack.sue_HEIGHT_TARGET_MAX_GET() == -1.9774128E38F);
			assert (pack.sue_ALT_HOLD_THROTTLE_MIN_GET() == -9.988388E37F);
			assert (pack.sue_ALT_HOLD_PITCH_MAX_GET() == -2.3253304E38F);
			assert (pack.sue_HEIGHT_TARGET_MIN_GET() == 2.8777167E38F);
			assert (pack.sue_ALT_HOLD_PITCH_HIGH_GET() == -6.9599286E37F);
			assert (pack.sue_ALT_HOLD_PITCH_MIN_GET() == 1.9990747E38F);
			assert (pack.sue_ALT_HOLD_THROTTLE_MAX_GET() == 1.5175654E38F);
		});
		GroundControl.SERIAL_UDB_EXTRA_F8 p176 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F8();
		PH.setPack(p176);
		p176.sue_HEIGHT_TARGET_MAX_SET(-1.9774128E38F);
		p176.sue_ALT_HOLD_PITCH_MAX_SET(-2.3253304E38F);
		p176.sue_ALT_HOLD_THROTTLE_MAX_SET(1.5175654E38F);
		p176.sue_ALT_HOLD_THROTTLE_MIN_SET(-9.988388E37F);
		p176.sue_HEIGHT_TARGET_MIN_SET(2.8777167E38F);
		p176.sue_ALT_HOLD_PITCH_HIGH_SET(-6.9599286E37F);
		p176.sue_ALT_HOLD_PITCH_MIN_SET(1.9990747E38F);
		CommunicationChannel.instance.send(p176);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_SERIAL_UDB_EXTRA_F13.add((src, ph, pack) ->
		{
			assert (pack.sue_lon_origin_GET() == -1807227433);
			assert (pack.sue_week_no_GET() == (short) 24193);
			assert (pack.sue_alt_origin_GET() == 11080776);
			assert (pack.sue_lat_origin_GET() == 1535862741);
		});
		GroundControl.SERIAL_UDB_EXTRA_F13 p177 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F13();
		PH.setPack(p177);
		p177.sue_lat_origin_SET(1535862741);
		p177.sue_alt_origin_SET(11080776);
		p177.sue_lon_origin_SET(-1807227433);
		p177.sue_week_no_SET((short) 24193);
		CommunicationChannel.instance.send(p177);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_SERIAL_UDB_EXTRA_F14.add((src, ph, pack) ->
		{
			assert (pack.sue_TRAP_SOURCE_GET() == 3321813976L);
			assert (pack.sue_RCON_GET() == (short) 5495);
			assert (pack.sue_GPS_TYPE_GET() == (char) 124);
			assert (pack.sue_DR_GET() == (char) 249);
			assert (pack.sue_TRAP_FLAGS_GET() == (short) -2069);
			assert (pack.sue_osc_fail_count_GET() == (short) 705);
			assert (pack.sue_AIRFRAME_GET() == (char) 46);
			assert (pack.sue_WIND_ESTIMATION_GET() == (char) 197);
			assert (pack.sue_CLOCK_CONFIG_GET() == (char) 183);
			assert (pack.sue_BOARD_TYPE_GET() == (char) 102);
			assert (pack.sue_FLIGHT_PLAN_TYPE_GET() == (char) 63);
		});
		GroundControl.SERIAL_UDB_EXTRA_F14 p178 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F14();
		PH.setPack(p178);
		p178.sue_BOARD_TYPE_SET((char) 102);
		p178.sue_TRAP_FLAGS_SET((short) -2069);
		p178.sue_AIRFRAME_SET((char) 46);
		p178.sue_GPS_TYPE_SET((char) 124);
		p178.sue_WIND_ESTIMATION_SET((char) 197);
		p178.sue_FLIGHT_PLAN_TYPE_SET((char) 63);
		p178.sue_osc_fail_count_SET((short) 705);
		p178.sue_RCON_SET((short) 5495);
		p178.sue_CLOCK_CONFIG_SET((char) 183);
		p178.sue_TRAP_SOURCE_SET(3321813976L);
		p178.sue_DR_SET((char) 249);
		CommunicationChannel.instance.send(p178);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_SERIAL_UDB_EXTRA_F15.add((src, ph, pack) ->
		{
			assert (Arrays.equals(pack.sue_ID_VEHICLE_MODEL_NAME_GET(), new char[]{(char) 248, (char) 58, (char) 115, (char) 19, (char) 104, (char) 249, (char) 194, (char) 193, (char) 123, (char) 50, (char) 170, (char) 93, (char) 220, (char) 171, (char) 50, (char) 55, (char) 237, (char) 222, (char) 79, (char) 62, (char) 129, (char) 65, (char) 154, (char) 34, (char) 204, (char) 134, (char) 252, (char) 57, (char) 244, (char) 250, (char) 48, (char) 153, (char) 89, (char) 145, (char) 131, (char) 71, (char) 139, (char) 189, (char) 45, (char) 148}));
			assert (Arrays.equals(pack.sue_ID_VEHICLE_REGISTRATION_GET(), new char[]{(char) 217, (char) 57, (char) 95, (char) 65, (char) 138, (char) 213, (char) 167, (char) 88, (char) 231, (char) 207, (char) 147, (char) 161, (char) 3, (char) 20, (char) 158, (char) 62, (char) 250, (char) 11, (char) 251, (char) 148}));
		});
		GroundControl.SERIAL_UDB_EXTRA_F15 p179 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F15();
		PH.setPack(p179);
		p179.sue_ID_VEHICLE_REGISTRATION_SET(new char[]{(char) 217, (char) 57, (char) 95, (char) 65, (char) 138, (char) 213, (char) 167, (char) 88, (char) 231, (char) 207, (char) 147, (char) 161, (char) 3, (char) 20, (char) 158, (char) 62, (char) 250, (char) 11, (char) 251, (char) 148}, 0);
		p179.sue_ID_VEHICLE_MODEL_NAME_SET(new char[]{(char) 248, (char) 58, (char) 115, (char) 19, (char) 104, (char) 249, (char) 194, (char) 193, (char) 123, (char) 50, (char) 170, (char) 93, (char) 220, (char) 171, (char) 50, (char) 55, (char) 237, (char) 222, (char) 79, (char) 62, (char) 129, (char) 65, (char) 154, (char) 34, (char) 204, (char) 134, (char) 252, (char) 57, (char) 244, (char) 250, (char) 48, (char) 153, (char) 89, (char) 145, (char) 131, (char) 71, (char) 139, (char) 189, (char) 45, (char) 148}, 0);
		CommunicationChannel.instance.send(p179);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_SERIAL_UDB_EXTRA_F16.add((src, ph, pack) ->
		{
			assert (Arrays.equals(pack.sue_ID_DIY_DRONES_URL_GET(), new char[]{(char) 12, (char) 138, (char) 179, (char) 46, (char) 35, (char) 75, (char) 92, (char) 35, (char) 201, (char) 103, (char) 211, (char) 81, (char) 172, (char) 145, (char) 77, (char) 164, (char) 4, (char) 197, (char) 68, (char) 232, (char) 198, (char) 191, (char) 149, (char) 26, (char) 207, (char) 239, (char) 241, (char) 143, (char) 8, (char) 239, (char) 185, (char) 170, (char) 2, (char) 9, (char) 239, (char) 212, (char) 173, (char) 84, (char) 173, (char) 78, (char) 194, (char) 255, (char) 133, (char) 7, (char) 161, (char) 179, (char) 164, (char) 147, (char) 58, (char) 210, (char) 197, (char) 98, (char) 33, (char) 22, (char) 248, (char) 209, (char) 74, (char) 43, (char) 121, (char) 71, (char) 193, (char) 196, (char) 62, (char) 19, (char) 120, (char) 121, (char) 124, (char) 109, (char) 116, (char) 175}));
			assert (Arrays.equals(pack.sue_ID_LEAD_PILOT_GET(), new char[]{(char) 50, (char) 162, (char) 238, (char) 7, (char) 128, (char) 45, (char) 149, (char) 197, (char) 240, (char) 207, (char) 154, (char) 146, (char) 80, (char) 97, (char) 128, (char) 183, (char) 74, (char) 25, (char) 15, (char) 248, (char) 70, (char) 119, (char) 236, (char) 130, (char) 110, (char) 220, (char) 32, (char) 2, (char) 231, (char) 77, (char) 114, (char) 49, (char) 186, (char) 152, (char) 92, (char) 198, (char) 246, (char) 32, (char) 126, (char) 173}));
		});
		GroundControl.SERIAL_UDB_EXTRA_F16 p180 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F16();
		PH.setPack(p180);
		p180.sue_ID_LEAD_PILOT_SET(new char[]{(char) 50, (char) 162, (char) 238, (char) 7, (char) 128, (char) 45, (char) 149, (char) 197, (char) 240, (char) 207, (char) 154, (char) 146, (char) 80, (char) 97, (char) 128, (char) 183, (char) 74, (char) 25, (char) 15, (char) 248, (char) 70, (char) 119, (char) 236, (char) 130, (char) 110, (char) 220, (char) 32, (char) 2, (char) 231, (char) 77, (char) 114, (char) 49, (char) 186, (char) 152, (char) 92, (char) 198, (char) 246, (char) 32, (char) 126, (char) 173}, 0);
		p180.sue_ID_DIY_DRONES_URL_SET(new char[]{(char) 12, (char) 138, (char) 179, (char) 46, (char) 35, (char) 75, (char) 92, (char) 35, (char) 201, (char) 103, (char) 211, (char) 81, (char) 172, (char) 145, (char) 77, (char) 164, (char) 4, (char) 197, (char) 68, (char) 232, (char) 198, (char) 191, (char) 149, (char) 26, (char) 207, (char) 239, (char) 241, (char) 143, (char) 8, (char) 239, (char) 185, (char) 170, (char) 2, (char) 9, (char) 239, (char) 212, (char) 173, (char) 84, (char) 173, (char) 78, (char) 194, (char) 255, (char) 133, (char) 7, (char) 161, (char) 179, (char) 164, (char) 147, (char) 58, (char) 210, (char) 197, (char) 98, (char) 33, (char) 22, (char) 248, (char) 209, (char) 74, (char) 43, (char) 121, (char) 71, (char) 193, (char) 196, (char) 62, (char) 19, (char) 120, (char) 121, (char) 124, (char) 109, (char) 116, (char) 175}, 0);
		CommunicationChannel.instance.send(p180);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_ALTITUDES.add((src, ph, pack) ->
		{
			assert (pack.alt_extra_GET() == 677351528);
			assert (pack.alt_imu_GET() == -2036712060);
			assert (pack.alt_barometric_GET() == 982267180);
			assert (pack.alt_optical_flow_GET() == 2046347398);
			assert (pack.time_boot_ms_GET() == 3721358817L);
			assert (pack.alt_gps_GET() == -882149795);
			assert (pack.alt_range_finder_GET() == -1854790588);
		});
		GroundControl.ALTITUDES p181 = CommunicationChannel.new_ALTITUDES();
		PH.setPack(p181);
		p181.alt_extra_SET(677351528);
		p181.alt_imu_SET(-2036712060);
		p181.alt_range_finder_SET(-1854790588);
		p181.alt_barometric_SET(982267180);
		p181.alt_gps_SET(-882149795);
		p181.alt_optical_flow_SET(2046347398);
		p181.time_boot_ms_SET(3721358817L);
		CommunicationChannel.instance.send(p181);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_AIRSPEEDS.add((src, ph, pack) ->
		{
			assert (pack.airspeed_hot_wire_GET() == (short) -7776);
			assert (pack.airspeed_ultrasonic_GET() == (short) 553);
			assert (pack.aoa_GET() == (short) -7727);
			assert (pack.aoy_GET() == (short) -5062);
			assert (pack.airspeed_pitot_GET() == (short) 9103);
			assert (pack.airspeed_imu_GET() == (short) 21760);
			assert (pack.time_boot_ms_GET() == 1182120051L);
		});
		GroundControl.AIRSPEEDS p182 = CommunicationChannel.new_AIRSPEEDS();
		PH.setPack(p182);
		p182.aoy_SET((short) -5062);
		p182.time_boot_ms_SET(1182120051L);
		p182.airspeed_hot_wire_SET((short) -7776);
		p182.airspeed_imu_SET((short) 21760);
		p182.airspeed_pitot_SET((short) 9103);
		p182.aoa_SET((short) -7727);
		p182.airspeed_ultrasonic_SET((short) 553);
		CommunicationChannel.instance.send(p182);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_SERIAL_UDB_EXTRA_F17.add((src, ph, pack) ->
		{
			assert (pack.sue_turn_rate_fbw_GET() == -1.1712661E38F);
			assert (pack.sue_turn_rate_nav_GET() == 6.3739637E37F);
			assert (pack.sue_feed_forward_GET() == -3.2968905E38F);
		});
		GroundControl.SERIAL_UDB_EXTRA_F17 p183 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F17();
		PH.setPack(p183);
		p183.sue_turn_rate_fbw_SET(-1.1712661E38F);
		p183.sue_turn_rate_nav_SET(6.3739637E37F);
		p183.sue_feed_forward_SET(-3.2968905E38F);
		CommunicationChannel.instance.send(p183);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_SERIAL_UDB_EXTRA_F18.add((src, ph, pack) ->
		{
			assert (pack.elevator_trim_inverted_GET() == 2.020033E36F);
			assert (pack.angle_of_attack_normal_GET() == -3.0492788E38F);
			assert (pack.reference_speed_GET() == -2.588768E37F);
			assert (pack.angle_of_attack_inverted_GET() == 9.444315E37F);
			assert (pack.elevator_trim_normal_GET() == -1.7682584E38F);
		});
		GroundControl.SERIAL_UDB_EXTRA_F18 p184 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F18();
		PH.setPack(p184);
		p184.elevator_trim_normal_SET(-1.7682584E38F);
		p184.reference_speed_SET(-2.588768E37F);
		p184.angle_of_attack_normal_SET(-3.0492788E38F);
		p184.angle_of_attack_inverted_SET(9.444315E37F);
		p184.elevator_trim_inverted_SET(2.020033E36F);
		CommunicationChannel.instance.send(p184);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_SERIAL_UDB_EXTRA_F19.add((src, ph, pack) ->
		{
			assert (pack.sue_rudder_reversed_GET() == (char) 240);
			assert (pack.sue_elevator_output_channel_GET() == (char) 131);
			assert (pack.sue_aileron_reversed_GET() == (char) 144);
			assert (pack.sue_aileron_output_channel_GET() == (char) 144);
			assert (pack.sue_throttle_reversed_GET() == (char) 231);
			assert (pack.sue_rudder_output_channel_GET() == (char) 173);
			assert (pack.sue_elevator_reversed_GET() == (char) 251);
			assert (pack.sue_throttle_output_channel_GET() == (char) 62);
		});
		GroundControl.SERIAL_UDB_EXTRA_F19 p185 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F19();
		PH.setPack(p185);
		p185.sue_rudder_output_channel_SET((char) 173);
		p185.sue_throttle_reversed_SET((char) 231);
		p185.sue_throttle_output_channel_SET((char) 62);
		p185.sue_elevator_reversed_SET((char) 251);
		p185.sue_elevator_output_channel_SET((char) 131);
		p185.sue_rudder_reversed_SET((char) 240);
		p185.sue_aileron_reversed_SET((char) 144);
		p185.sue_aileron_output_channel_SET((char) 144);
		CommunicationChannel.instance.send(p185);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_SERIAL_UDB_EXTRA_F20.add((src, ph, pack) ->
		{
			assert (pack.sue_trim_value_input_6_GET() == (short) -30832);
			assert (pack.sue_trim_value_input_11_GET() == (short) 20716);
			assert (pack.sue_trim_value_input_1_GET() == (short) 2558);
			assert (pack.sue_trim_value_input_9_GET() == (short) 25455);
			assert (pack.sue_trim_value_input_2_GET() == (short) 25679);
			assert (pack.sue_number_of_inputs_GET() == (char) 2);
			assert (pack.sue_trim_value_input_8_GET() == (short) -11076);
			assert (pack.sue_trim_value_input_12_GET() == (short) -23875);
			assert (pack.sue_trim_value_input_4_GET() == (short) -30412);
			assert (pack.sue_trim_value_input_7_GET() == (short) -32645);
			assert (pack.sue_trim_value_input_10_GET() == (short) -10123);
			assert (pack.sue_trim_value_input_3_GET() == (short) 31412);
			assert (pack.sue_trim_value_input_5_GET() == (short) -3904);
		});
		GroundControl.SERIAL_UDB_EXTRA_F20 p186 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F20();
		PH.setPack(p186);
		p186.sue_trim_value_input_4_SET((short) -30412);
		p186.sue_trim_value_input_8_SET((short) -11076);
		p186.sue_trim_value_input_5_SET((short) -3904);
		p186.sue_trim_value_input_3_SET((short) 31412);
		p186.sue_trim_value_input_7_SET((short) -32645);
		p186.sue_trim_value_input_11_SET((short) 20716);
		p186.sue_number_of_inputs_SET((char) 2);
		p186.sue_trim_value_input_1_SET((short) 2558);
		p186.sue_trim_value_input_2_SET((short) 25679);
		p186.sue_trim_value_input_6_SET((short) -30832);
		p186.sue_trim_value_input_9_SET((short) 25455);
		p186.sue_trim_value_input_10_SET((short) -10123);
		p186.sue_trim_value_input_12_SET((short) -23875);
		CommunicationChannel.instance.send(p186);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_SERIAL_UDB_EXTRA_F21.add((src, ph, pack) ->
		{
			assert (pack.sue_gyro_y_offset_GET() == (short) -24257);
			assert (pack.sue_gyro_x_offset_GET() == (short) -3278);
			assert (pack.sue_accel_y_offset_GET() == (short) 12359);
			assert (pack.sue_gyro_z_offset_GET() == (short) 31009);
			assert (pack.sue_accel_z_offset_GET() == (short) -18147);
			assert (pack.sue_accel_x_offset_GET() == (short) -19989);
		});
		GroundControl.SERIAL_UDB_EXTRA_F21 p187 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F21();
		PH.setPack(p187);
		p187.sue_gyro_z_offset_SET((short) 31009);
		p187.sue_accel_x_offset_SET((short) -19989);
		p187.sue_accel_z_offset_SET((short) -18147);
		p187.sue_gyro_y_offset_SET((short) -24257);
		p187.sue_accel_y_offset_SET((short) 12359);
		p187.sue_gyro_x_offset_SET((short) -3278);
		CommunicationChannel.instance.send(p187);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_SERIAL_UDB_EXTRA_F22.add((src, ph, pack) ->
		{
			assert (pack.sue_gyro_x_at_calibration_GET() == (short) -7772);
			assert (pack.sue_accel_y_at_calibration_GET() == (short) -12215);
			assert (pack.sue_accel_x_at_calibration_GET() == (short) -14256);
			assert (pack.sue_accel_z_at_calibration_GET() == (short) -5120);
			assert (pack.sue_gyro_z_at_calibration_GET() == (short) 10973);
			assert (pack.sue_gyro_y_at_calibration_GET() == (short) 19088);
		});
		GroundControl.SERIAL_UDB_EXTRA_F22 p188 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F22();
		PH.setPack(p188);
		p188.sue_gyro_z_at_calibration_SET((short) 10973);
		p188.sue_gyro_x_at_calibration_SET((short) -7772);
		p188.sue_accel_z_at_calibration_SET((short) -5120);
		p188.sue_accel_y_at_calibration_SET((short) -12215);
		p188.sue_accel_x_at_calibration_SET((short) -14256);
		p188.sue_gyro_y_at_calibration_SET((short) 19088);
		CommunicationChannel.instance.send(p188);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		CommunicationChannel.instance.on_ESTIMATOR_STATUS.add((src, ph, pack) ->
		{
			assert (pack.time_usec_GET() == 4427615705455116406L);
			assert (pack.mag_ratio_GET() == -1.4913384E38F);
			assert (pack.pos_horiz_ratio_GET() == 3.8741775E37F);
			assert (pack.tas_ratio_GET() == 3.0017147E37F);
			assert (pack.vel_ratio_GET() == 3.0563717E38F);
			assert (pack.hagl_ratio_GET() == 2.7141127E38F);
			assert (pack.pos_vert_accuracy_GET() == -2.2440722E38F);
			assert (pack.pos_horiz_accuracy_GET() == 2.7695971E38F);
			assert (pack.pos_vert_ratio_GET() == -1.5166267E37F);
			assert (pack.flags_GET() == ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_HORIZ);
		});
		GroundControl.ESTIMATOR_STATUS p230 = CommunicationChannel.new_ESTIMATOR_STATUS();
		PH.setPack(p230);
		p230.pos_vert_accuracy_SET(-2.2440722E38F);
		p230.vel_ratio_SET(3.0563717E38F);
		p230.tas_ratio_SET(3.0017147E37F);
		p230.mag_ratio_SET(-1.4913384E38F);
		p230.hagl_ratio_SET(2.7141127E38F);
		p230.time_usec_SET(4427615705455116406L);
		p230.flags_SET(ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_HORIZ);
		p230.pos_vert_ratio_SET(-1.5166267E37F);
		p230.pos_horiz_ratio_SET(3.8741775E37F);
		p230.pos_horiz_accuracy_SET(2.7695971E38F);
		CommunicationChannel.instance.send(p230);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_WIND_COV.add((src, ph, pack) ->
		{
			assert (pack.wind_z_GET() == -9.981618E37F);
			assert (pack.time_usec_GET() == 3836255331499745624L);
			assert (pack.var_vert_GET() == -1.9145672E38F);
			assert (pack.wind_alt_GET() == 2.074608E37F);
			assert (pack.vert_accuracy_GET() == 2.3953587E38F);
			assert (pack.wind_x_GET() == -3.3217222E38F);
			assert (pack.wind_y_GET() == 3.3904394E38F);
			assert (pack.horiz_accuracy_GET() == 8.93155E37F);
			assert (pack.var_horiz_GET() == 2.063645E38F);
		});
		GroundControl.WIND_COV p231 = CommunicationChannel.new_WIND_COV();
		PH.setPack(p231);
		p231.wind_z_SET(-9.981618E37F);
		p231.wind_y_SET(3.3904394E38F);
		p231.var_horiz_SET(2.063645E38F);
		p231.vert_accuracy_SET(2.3953587E38F);
		p231.horiz_accuracy_SET(8.93155E37F);
		p231.time_usec_SET(3836255331499745624L);
		p231.var_vert_SET(-1.9145672E38F);
		p231.wind_alt_SET(2.074608E37F);
		p231.wind_x_SET(-3.3217222E38F);
		CommunicationChannel.instance.send(p231);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_GPS_INPUT.add((src, ph, pack) ->
		{
			assert (pack.speed_accuracy_GET() == 1.8462536E38F);
			assert (pack.ignore_flags_GET() == GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VEL_HORIZ);
			assert (pack.time_usec_GET() == 7276757642912726959L);
			assert (pack.horiz_accuracy_GET() == -1.5727629E38F);
			assert (pack.time_week_ms_GET() == 3355573303L);
			assert (pack.alt_GET() == -3.2335698E38F);
			assert (pack.vd_GET() == -1.939672E38F);
			assert (pack.lat_GET() == 1863308767);
			assert (pack.fix_type_GET() == (char) 175);
			assert (pack.satellites_visible_GET() == (char) 65);
			assert (pack.vdop_GET() == -2.2420036E38F);
			assert (pack.vert_accuracy_GET() == -3.2014887E38F);
			assert (pack.lon_GET() == 1540304627);
			assert (pack.hdop_GET() == -2.8294846E38F);
			assert (pack.time_week_GET() == (char) 4837);
			assert (pack.vn_GET() == -3.5563136E37F);
			assert (pack.ve_GET() == -2.7503775E38F);
			assert (pack.gps_id_GET() == (char) 129);
		});
		GroundControl.GPS_INPUT p232 = CommunicationChannel.new_GPS_INPUT();
		PH.setPack(p232);
		p232.alt_SET(-3.2335698E38F);
		p232.vdop_SET(-2.2420036E38F);
		p232.time_week_ms_SET(3355573303L);
		p232.ve_SET(-2.7503775E38F);
		p232.horiz_accuracy_SET(-1.5727629E38F);
		p232.ignore_flags_SET(GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VEL_HORIZ);
		p232.satellites_visible_SET((char) 65);
		p232.lat_SET(1863308767);
		p232.vd_SET(-1.939672E38F);
		p232.vert_accuracy_SET(-3.2014887E38F);
		p232.speed_accuracy_SET(1.8462536E38F);
		p232.hdop_SET(-2.8294846E38F);
		p232.lon_SET(1540304627);
		p232.gps_id_SET((char) 129);
		p232.vn_SET(-3.5563136E37F);
		p232.fix_type_SET((char) 175);
		p232.time_usec_SET(7276757642912726959L);
		p232.time_week_SET((char) 4837);
		CommunicationChannel.instance.send(p232);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_GPS_RTCM_DATA.add((src, ph, pack) ->
		{
			assert (pack.len_GET() == (char) 70);
			assert (Arrays.equals(pack.data__GET(), new char[]{(char) 67, (char) 247, (char) 20, (char) 157, (char) 126, (char) 20, (char) 186, (char) 6, (char) 160, (char) 188, (char) 171, (char) 117, (char) 152, (char) 65, (char) 91, (char) 31, (char) 75, (char) 70, (char) 33, (char) 207, (char) 85, (char) 246, (char) 21, (char) 235, (char) 55, (char) 48, (char) 127, (char) 39, (char) 251, (char) 199, (char) 93, (char) 23, (char) 13, (char) 213, (char) 129, (char) 98, (char) 209, (char) 36, (char) 19, (char) 156, (char) 203, (char) 66, (char) 13, (char) 221, (char) 227, (char) 135, (char) 17, (char) 238, (char) 60, (char) 197, (char) 177, (char) 140, (char) 84, (char) 57, (char) 98, (char) 118, (char) 13, (char) 240, (char) 151, (char) 184, (char) 82, (char) 201, (char) 68, (char) 10, (char) 38, (char) 241, (char) 80, (char) 160, (char) 118, (char) 151, (char) 172, (char) 60, (char) 117, (char) 29, (char) 252, (char) 163, (char) 97, (char) 93, (char) 7, (char) 175, (char) 83, (char) 133, (char) 96, (char) 130, (char) 137, (char) 169, (char) 58, (char) 132, (char) 182, (char) 34, (char) 88, (char) 145, (char) 54, (char) 221, (char) 164, (char) 202, (char) 149, (char) 208, (char) 199, (char) 126, (char) 123, (char) 164, (char) 145, (char) 60, (char) 255, (char) 90, (char) 82, (char) 106, (char) 27, (char) 235, (char) 111, (char) 140, (char) 155, (char) 101, (char) 29, (char) 215, (char) 207, (char) 186, (char) 205, (char) 96, (char) 158, (char) 68, (char) 242, (char) 8, (char) 251, (char) 67, (char) 168, (char) 149, (char) 249, (char) 79, (char) 4, (char) 229, (char) 103, (char) 97, (char) 138, (char) 140, (char) 241, (char) 79, (char) 143, (char) 32, (char) 16, (char) 248, (char) 188, (char) 147, (char) 156, (char) 7, (char) 78, (char) 183, (char) 193, (char) 73, (char) 116, (char) 87, (char) 249, (char) 96, (char) 123, (char) 52, (char) 28, (char) 178, (char) 213, (char) 32, (char) 43, (char) 139, (char) 18, (char) 242, (char) 171, (char) 205, (char) 132, (char) 231, (char) 36, (char) 252, (char) 179, (char) 151, (char) 198, (char) 232, (char) 197, (char) 216, (char) 126, (char) 25, (char) 107, (char) 114}));
			assert (pack.flags_GET() == (char) 97);
		});
		GroundControl.GPS_RTCM_DATA p233 = CommunicationChannel.new_GPS_RTCM_DATA();
		PH.setPack(p233);
		p233.len_SET((char) 70);
		p233.data__SET(new char[]{(char) 67, (char) 247, (char) 20, (char) 157, (char) 126, (char) 20, (char) 186, (char) 6, (char) 160, (char) 188, (char) 171, (char) 117, (char) 152, (char) 65, (char) 91, (char) 31, (char) 75, (char) 70, (char) 33, (char) 207, (char) 85, (char) 246, (char) 21, (char) 235, (char) 55, (char) 48, (char) 127, (char) 39, (char) 251, (char) 199, (char) 93, (char) 23, (char) 13, (char) 213, (char) 129, (char) 98, (char) 209, (char) 36, (char) 19, (char) 156, (char) 203, (char) 66, (char) 13, (char) 221, (char) 227, (char) 135, (char) 17, (char) 238, (char) 60, (char) 197, (char) 177, (char) 140, (char) 84, (char) 57, (char) 98, (char) 118, (char) 13, (char) 240, (char) 151, (char) 184, (char) 82, (char) 201, (char) 68, (char) 10, (char) 38, (char) 241, (char) 80, (char) 160, (char) 118, (char) 151, (char) 172, (char) 60, (char) 117, (char) 29, (char) 252, (char) 163, (char) 97, (char) 93, (char) 7, (char) 175, (char) 83, (char) 133, (char) 96, (char) 130, (char) 137, (char) 169, (char) 58, (char) 132, (char) 182, (char) 34, (char) 88, (char) 145, (char) 54, (char) 221, (char) 164, (char) 202, (char) 149, (char) 208, (char) 199, (char) 126, (char) 123, (char) 164, (char) 145, (char) 60, (char) 255, (char) 90, (char) 82, (char) 106, (char) 27, (char) 235, (char) 111, (char) 140, (char) 155, (char) 101, (char) 29, (char) 215, (char) 207, (char) 186, (char) 205, (char) 96, (char) 158, (char) 68, (char) 242, (char) 8, (char) 251, (char) 67, (char) 168, (char) 149, (char) 249, (char) 79, (char) 4, (char) 229, (char) 103, (char) 97, (char) 138, (char) 140, (char) 241, (char) 79, (char) 143, (char) 32, (char) 16, (char) 248, (char) 188, (char) 147, (char) 156, (char) 7, (char) 78, (char) 183, (char) 193, (char) 73, (char) 116, (char) 87, (char) 249, (char) 96, (char) 123, (char) 52, (char) 28, (char) 178, (char) 213, (char) 32, (char) 43, (char) 139, (char) 18, (char) 242, (char) 171, (char) 205, (char) 132, (char) 231, (char) 36, (char) 252, (char) 179, (char) 151, (char) 198, (char) 232, (char) 197, (char) 216, (char) 126, (char) 25, (char) 107, (char) 114}, 0);
		p233.flags_SET((char) 97);
		CommunicationChannel.instance.send(p233);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_HIGH_LATENCY.add((src, ph, pack) ->
		{
			assert (pack.heading_sp_GET() == (short) 15992);
			assert (pack.airspeed_GET() == (char) 37);
			assert (pack.latitude_GET() == -2089515618);
			assert (pack.gps_nsat_GET() == (char) 8);
			assert (pack.altitude_amsl_GET() == (short) 11532);
			assert (pack.base_mode_GET() == MAV_MODE_FLAG.MAV_MODE_FLAG_TEST_ENABLED);
			assert (pack.airspeed_sp_GET() == (char) 54);
			assert (pack.pitch_GET() == (short) -407);
			assert (pack.failsafe_GET() == (char) 59);
			assert (pack.temperature_air_GET() == (byte) -123);
			assert (pack.custom_mode_GET() == 1279682468L);
			assert (pack.throttle_GET() == (byte) -52);
			assert (pack.battery_remaining_GET() == (char) 254);
			assert (pack.wp_distance_GET() == (char) 62431);
			assert (pack.temperature_GET() == (byte) -84);
			assert (pack.wp_num_GET() == (char) 56);
			assert (pack.heading_GET() == (char) 10761);
			assert (pack.groundspeed_GET() == (char) 82);
			assert (pack.climb_rate_GET() == (byte) -30);
			assert (pack.longitude_GET() == 2110072792);
			assert (pack.landed_state_GET() == MAV_LANDED_STATE.MAV_LANDED_STATE_UNDEFINED);
			assert (pack.roll_GET() == (short) -571);
			assert (pack.gps_fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_NO_GPS);
			assert (pack.altitude_sp_GET() == (short) -12139);
		});
		GroundControl.HIGH_LATENCY p234 = CommunicationChannel.new_HIGH_LATENCY();
		PH.setPack(p234);
		p234.gps_nsat_SET((char) 8);
		p234.battery_remaining_SET((char) 254);
		p234.airspeed_SET((char) 37);
		p234.wp_distance_SET((char) 62431);
		p234.altitude_sp_SET((short) -12139);
		p234.throttle_SET((byte) -52);
		p234.custom_mode_SET(1279682468L);
		p234.gps_fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_NO_GPS);
		p234.altitude_amsl_SET((short) 11532);
		p234.temperature_SET((byte) -84);
		p234.airspeed_sp_SET((char) 54);
		p234.climb_rate_SET((byte) -30);
		p234.wp_num_SET((char) 56);
		p234.pitch_SET((short) -407);
		p234.groundspeed_SET((char) 82);
		p234.longitude_SET(2110072792);
		p234.base_mode_SET(MAV_MODE_FLAG.MAV_MODE_FLAG_TEST_ENABLED);
		p234.failsafe_SET((char) 59);
		p234.temperature_air_SET((byte) -123);
		p234.roll_SET((short) -571);
		p234.latitude_SET(-2089515618);
		p234.landed_state_SET(MAV_LANDED_STATE.MAV_LANDED_STATE_UNDEFINED);
		p234.heading_sp_SET((short) 15992);
		p234.heading_SET((char) 10761);
		CommunicationChannel.instance.send(p234);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_VIBRATION.add((src, ph, pack) ->
		{
			assert (pack.vibration_x_GET() == 1.4361294E38F);
			assert (pack.time_usec_GET() == 1229430541923843990L);
			assert (pack.clipping_1_GET() == 3567881928L);
			assert (pack.clipping_0_GET() == 4156982788L);
			assert (pack.vibration_z_GET() == 8.395188E36F);
			assert (pack.vibration_y_GET() == 2.2709954E38F);
			assert (pack.clipping_2_GET() == 4099523599L);
		});
		GroundControl.VIBRATION p241 = CommunicationChannel.new_VIBRATION();
		PH.setPack(p241);
		p241.clipping_2_SET(4099523599L);
		p241.clipping_1_SET(3567881928L);
		p241.vibration_z_SET(8.395188E36F);
		p241.time_usec_SET(1229430541923843990L);
		p241.clipping_0_SET(4156982788L);
		p241.vibration_x_SET(1.4361294E38F);
		p241.vibration_y_SET(2.2709954E38F);
		CommunicationChannel.instance.send(p241);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		CommunicationChannel.instance.on_HOME_POSITION.add((src, ph, pack) ->
		{
			assert (pack.z_GET() == -5.01537E37F);
			assert (pack.latitude_GET() == -662478974);
			assert (pack.altitude_GET() == -155082980);
			assert (pack.y_GET() == -1.5773269E38F);
			assert (pack.x_GET() == 2.9872543E38F);
			assert (Arrays.equals(pack.q_GET(), new float[]{2.345752E38F, 3.1060991E38F, -3.0458506E38F, -2.8582633E37F}));
			assert (pack.time_usec_TRY(ph) == 7950934764494855170L);
			assert (pack.approach_z_GET() == 1.747531E38F);
			assert (pack.approach_x_GET() == 2.738586E36F);
			assert (pack.approach_y_GET() == 4.7164837E37F);
			assert (pack.longitude_GET() == 1669104504);
		});
		GroundControl.HOME_POSITION p242 = CommunicationChannel.new_HOME_POSITION();
		PH.setPack(p242);
		p242.approach_z_SET(1.747531E38F);
		p242.q_SET(new float[]{2.345752E38F, 3.1060991E38F, -3.0458506E38F, -2.8582633E37F}, 0);
		p242.y_SET(-1.5773269E38F);
		p242.altitude_SET(-155082980);
		p242.time_usec_SET(7950934764494855170L, PH);
		p242.longitude_SET(1669104504);
		p242.approach_y_SET(4.7164837E37F);
		p242.latitude_SET(-662478974);
		p242.x_SET(2.9872543E38F);
		p242.approach_x_SET(2.738586E36F);
		p242.z_SET(-5.01537E37F);
		CommunicationChannel.instance.send(p242);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
		TestChannel.instance.on_SET_HOME_POSITION.add((src, ph, pack) ->
		{
			assert (pack.altitude_GET() == 1757915270);
			assert (pack.target_system_GET() == (char) 124);
			assert (pack.approach_x_GET() == 7.0440083E37F);
			assert (pack.latitude_GET() == -1778864659);
			assert (pack.approach_y_GET() == 1.0947603E38F);
			assert (Arrays.equals(pack.q_GET(), new float[]{-1.3831859E38F, -3.2029083E38F, -1.3791798E38F, 4.996724E37F}));
			assert (pack.time_usec_TRY(ph) == 5881373832110786775L);
			assert (pack.longitude_GET() == 1866952069);
			assert (pack.y_GET() == 3.0311173E38F);
			assert (pack.x_GET() == -1.1687262E38F);
			assert (pack.z_GET() == 3.3079286E38F);
			assert (pack.approach_z_GET() == -4.2270694E37F);
		});
		GroundControl.SET_HOME_POSITION p243 = CommunicationChannel.new_SET_HOME_POSITION();
		PH.setPack(p243);
		p243.time_usec_SET(5881373832110786775L, PH);
		p243.approach_z_SET(-4.2270694E37F);
		p243.target_system_SET((char) 124);
		p243.approach_y_SET(1.0947603E38F);
		p243.z_SET(3.3079286E38F);
		p243.longitude_SET(1866952069);
		p243.x_SET(-1.1687262E38F);
		p243.altitude_SET(1757915270);
		p243.latitude_SET(-1778864659);
		p243.q_SET(new float[]{-1.3831859E38F, -3.2029083E38F, -1.3791798E38F, 4.996724E37F}, 0);
		p243.approach_x_SET(7.0440083E37F);
		p243.y_SET(3.0311173E38F);
		CommunicationChannel.instance.send(p243);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_MESSAGE_INTERVAL.add((src, ph, pack) ->
		{
			assert (pack.interval_us_GET() == 1235466307);
			assert (pack.message_id_GET() == (char) 39770);
		});
		GroundControl.MESSAGE_INTERVAL p244 = CommunicationChannel.new_MESSAGE_INTERVAL();
		PH.setPack(p244);
		p244.interval_us_SET(1235466307);
		p244.message_id_SET((char) 39770);
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
			assert (pack.squawk_GET() == (char) 49096);
			assert (pack.heading_GET() == (char) 9252);
			assert (pack.ver_velocity_GET() == (short) 1429);
			assert (pack.altitude_type_GET() == ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_PRESSURE_QNH);
			assert (pack.altitude_GET() == -1870465419);
			assert (pack.ICAO_address_GET() == 2776400305L);
			assert (pack.emitter_type_GET() == ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_ULTRA_LIGHT);
			assert (pack.lat_GET() == -329285953);
			assert (pack.hor_velocity_GET() == (char) 33085);
			assert (pack.lon_GET() == -792009887);
			assert (pack.flags_GET() == ADSB_FLAGS.ADSB_FLAGS_SIMULATED);
			assert (pack.tslc_GET() == (char) 53);
			assert (pack.callsign_LEN(ph) == 9);
			assert (pack.callsign_TRY(ph).equals("Atpgjqauc"));
		});
		GroundControl.ADSB_VEHICLE p246 = CommunicationChannel.new_ADSB_VEHICLE();
		PH.setPack(p246);
		p246.squawk_SET((char) 49096);
		p246.flags_SET(ADSB_FLAGS.ADSB_FLAGS_SIMULATED);
		p246.ver_velocity_SET((short) 1429);
		p246.tslc_SET((char) 53);
		p246.callsign_SET("Atpgjqauc", PH);
		p246.altitude_type_SET(ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_PRESSURE_QNH);
		p246.lat_SET(-329285953);
		p246.ICAO_address_SET(2776400305L);
		p246.lon_SET(-792009887);
		p246.emitter_type_SET(ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_ULTRA_LIGHT);
		p246.hor_velocity_SET((char) 33085);
		p246.altitude_SET(-1870465419);
		p246.heading_SET((char) 9252);
		CommunicationChannel.instance.send(p246);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_COLLISION.add((src, ph, pack) ->
		{
			assert (pack.id_GET() == 2218362519L);
			assert (pack.action_GET() == MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_ASCEND_OR_DESCEND);
			assert (pack.time_to_minimum_delta_GET() == 1.0768734E38F);
			assert (pack.altitude_minimum_delta_GET() == 1.3893169E38F);
			assert (pack.threat_level_GET() == MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_NONE);
			assert (pack.horizontal_minimum_delta_GET() == 3.877535E37F);
			assert (pack.src__GET() == MAV_COLLISION_SRC.MAV_COLLISION_SRC_ADSB);
		});
		GroundControl.COLLISION p247 = CommunicationChannel.new_COLLISION();
		PH.setPack(p247);
		p247.action_SET(MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_ASCEND_OR_DESCEND);
		p247.altitude_minimum_delta_SET(1.3893169E38F);
		p247.src__SET(MAV_COLLISION_SRC.MAV_COLLISION_SRC_ADSB);
		p247.horizontal_minimum_delta_SET(3.877535E37F);
		p247.threat_level_SET(MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_NONE);
		p247.time_to_minimum_delta_SET(1.0768734E38F);
		p247.id_SET(2218362519L);
		CommunicationChannel.instance.send(p247);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_V2_EXTENSION.add((src, ph, pack) ->
		{
			assert (pack.message_type_GET() == (char) 1745);
			assert (pack.target_component_GET() == (char) 35);
			assert (pack.target_system_GET() == (char) 101);
			assert (pack.target_network_GET() == (char) 159);
			assert (Arrays.equals(pack.payload_GET(), new char[]{(char) 151, (char) 93, (char) 126, (char) 13, (char) 143, (char) 55, (char) 31, (char) 154, (char) 183, (char) 74, (char) 189, (char) 229, (char) 250, (char) 80, (char) 14, (char) 132, (char) 180, (char) 132, (char) 151, (char) 63, (char) 66, (char) 238, (char) 83, (char) 4, (char) 183, (char) 47, (char) 188, (char) 243, (char) 27, (char) 207, (char) 94, (char) 221, (char) 205, (char) 137, (char) 122, (char) 251, (char) 218, (char) 229, (char) 127, (char) 204, (char) 72, (char) 146, (char) 105, (char) 229, (char) 126, (char) 47, (char) 182, (char) 191, (char) 143, (char) 68, (char) 59, (char) 152, (char) 48, (char) 82, (char) 54, (char) 163, (char) 157, (char) 240, (char) 10, (char) 237, (char) 3, (char) 111, (char) 24, (char) 83, (char) 240, (char) 17, (char) 122, (char) 114, (char) 115, (char) 126, (char) 85, (char) 253, (char) 241, (char) 255, (char) 242, (char) 95, (char) 251, (char) 101, (char) 128, (char) 80, (char) 11, (char) 240, (char) 242, (char) 201, (char) 190, (char) 179, (char) 213, (char) 120, (char) 147, (char) 51, (char) 135, (char) 181, (char) 46, (char) 84, (char) 96, (char) 115, (char) 204, (char) 227, (char) 175, (char) 17, (char) 215, (char) 43, (char) 221, (char) 233, (char) 75, (char) 190, (char) 32, (char) 171, (char) 253, (char) 78, (char) 197, (char) 97, (char) 248, (char) 221, (char) 84, (char) 106, (char) 178, (char) 10, (char) 86, (char) 5, (char) 129, (char) 86, (char) 2, (char) 55, (char) 157, (char) 208, (char) 161, (char) 30, (char) 150, (char) 21, (char) 158, (char) 15, (char) 163, (char) 9, (char) 98, (char) 86, (char) 177, (char) 140, (char) 35, (char) 202, (char) 249, (char) 178, (char) 29, (char) 249, (char) 229, (char) 147, (char) 93, (char) 235, (char) 110, (char) 229, (char) 11, (char) 210, (char) 82, (char) 119, (char) 120, (char) 41, (char) 246, (char) 118, (char) 74, (char) 255, (char) 101, (char) 56, (char) 82, (char) 253, (char) 145, (char) 33, (char) 56, (char) 107, (char) 195, (char) 17, (char) 16, (char) 135, (char) 21, (char) 94, (char) 110, (char) 205, (char) 96, (char) 110, (char) 228, (char) 146, (char) 53, (char) 180, (char) 236, (char) 143, (char) 177, (char) 27, (char) 17, (char) 173, (char) 190, (char) 209, (char) 218, (char) 163, (char) 13, (char) 38, (char) 114, (char) 216, (char) 202, (char) 221, (char) 119, (char) 123, (char) 177, (char) 154, (char) 113, (char) 248, (char) 116, (char) 68, (char) 151, (char) 113, (char) 139, (char) 218, (char) 7, (char) 238, (char) 147, (char) 122, (char) 67, (char) 125, (char) 30, (char) 44, (char) 91, (char) 195, (char) 96, (char) 186, (char) 219, (char) 238, (char) 42, (char) 204, (char) 233, (char) 73, (char) 10, (char) 135, (char) 131, (char) 44, (char) 69, (char) 51, (char) 200, (char) 233, (char) 47, (char) 159, (char) 150, (char) 208, (char) 219, (char) 150, (char) 221, (char) 38, (char) 128, (char) 198, (char) 255, (char) 8, (char) 179}));
		});
		GroundControl.V2_EXTENSION p248 = CommunicationChannel.new_V2_EXTENSION();
		PH.setPack(p248);
		p248.target_component_SET((char) 35);
		p248.payload_SET(new char[]{(char) 151, (char) 93, (char) 126, (char) 13, (char) 143, (char) 55, (char) 31, (char) 154, (char) 183, (char) 74, (char) 189, (char) 229, (char) 250, (char) 80, (char) 14, (char) 132, (char) 180, (char) 132, (char) 151, (char) 63, (char) 66, (char) 238, (char) 83, (char) 4, (char) 183, (char) 47, (char) 188, (char) 243, (char) 27, (char) 207, (char) 94, (char) 221, (char) 205, (char) 137, (char) 122, (char) 251, (char) 218, (char) 229, (char) 127, (char) 204, (char) 72, (char) 146, (char) 105, (char) 229, (char) 126, (char) 47, (char) 182, (char) 191, (char) 143, (char) 68, (char) 59, (char) 152, (char) 48, (char) 82, (char) 54, (char) 163, (char) 157, (char) 240, (char) 10, (char) 237, (char) 3, (char) 111, (char) 24, (char) 83, (char) 240, (char) 17, (char) 122, (char) 114, (char) 115, (char) 126, (char) 85, (char) 253, (char) 241, (char) 255, (char) 242, (char) 95, (char) 251, (char) 101, (char) 128, (char) 80, (char) 11, (char) 240, (char) 242, (char) 201, (char) 190, (char) 179, (char) 213, (char) 120, (char) 147, (char) 51, (char) 135, (char) 181, (char) 46, (char) 84, (char) 96, (char) 115, (char) 204, (char) 227, (char) 175, (char) 17, (char) 215, (char) 43, (char) 221, (char) 233, (char) 75, (char) 190, (char) 32, (char) 171, (char) 253, (char) 78, (char) 197, (char) 97, (char) 248, (char) 221, (char) 84, (char) 106, (char) 178, (char) 10, (char) 86, (char) 5, (char) 129, (char) 86, (char) 2, (char) 55, (char) 157, (char) 208, (char) 161, (char) 30, (char) 150, (char) 21, (char) 158, (char) 15, (char) 163, (char) 9, (char) 98, (char) 86, (char) 177, (char) 140, (char) 35, (char) 202, (char) 249, (char) 178, (char) 29, (char) 249, (char) 229, (char) 147, (char) 93, (char) 235, (char) 110, (char) 229, (char) 11, (char) 210, (char) 82, (char) 119, (char) 120, (char) 41, (char) 246, (char) 118, (char) 74, (char) 255, (char) 101, (char) 56, (char) 82, (char) 253, (char) 145, (char) 33, (char) 56, (char) 107, (char) 195, (char) 17, (char) 16, (char) 135, (char) 21, (char) 94, (char) 110, (char) 205, (char) 96, (char) 110, (char) 228, (char) 146, (char) 53, (char) 180, (char) 236, (char) 143, (char) 177, (char) 27, (char) 17, (char) 173, (char) 190, (char) 209, (char) 218, (char) 163, (char) 13, (char) 38, (char) 114, (char) 216, (char) 202, (char) 221, (char) 119, (char) 123, (char) 177, (char) 154, (char) 113, (char) 248, (char) 116, (char) 68, (char) 151, (char) 113, (char) 139, (char) 218, (char) 7, (char) 238, (char) 147, (char) 122, (char) 67, (char) 125, (char) 30, (char) 44, (char) 91, (char) 195, (char) 96, (char) 186, (char) 219, (char) 238, (char) 42, (char) 204, (char) 233, (char) 73, (char) 10, (char) 135, (char) 131, (char) 44, (char) 69, (char) 51, (char) 200, (char) 233, (char) 47, (char) 159, (char) 150, (char) 208, (char) 219, (char) 150, (char) 221, (char) 38, (char) 128, (char) 198, (char) 255, (char) 8, (char) 179}, 0);
		p248.message_type_SET((char) 1745);
		p248.target_system_SET((char) 101);
		p248.target_network_SET((char) 159);
		CommunicationChannel.instance.send(p248);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_MEMORY_VECT.add((src, ph, pack) ->
		{
			assert (pack.ver_GET() == (char) 204);
			assert (pack.type_GET() == (char) 23);
			assert (pack.address_GET() == (char) 13959);
			assert (Arrays.equals(pack.value_GET(), new byte[]{(byte) -104, (byte) 51, (byte) -96, (byte) 8, (byte) 23, (byte) -65, (byte) -32, (byte) 88, (byte) -65, (byte) 44, (byte) -53, (byte) 18, (byte) 95, (byte) -126, (byte) 63, (byte) 90, (byte) -90, (byte) 13, (byte) 41, (byte) -14, (byte) -22, (byte) 84, (byte) -94, (byte) 47, (byte) 33, (byte) -23, (byte) -89, (byte) 101, (byte) -62, (byte) 97, (byte) 103, (byte) -81}));
		});
		GroundControl.MEMORY_VECT p249 = CommunicationChannel.new_MEMORY_VECT();
		PH.setPack(p249);
		p249.address_SET((char) 13959);
		p249.ver_SET((char) 204);
		p249.type_SET((char) 23);
		p249.value_SET(new byte[]{(byte) -104, (byte) 51, (byte) -96, (byte) 8, (byte) 23, (byte) -65, (byte) -32, (byte) 88, (byte) -65, (byte) 44, (byte) -53, (byte) 18, (byte) 95, (byte) -126, (byte) 63, (byte) 90, (byte) -90, (byte) 13, (byte) 41, (byte) -14, (byte) -22, (byte) 84, (byte) -94, (byte) 47, (byte) 33, (byte) -23, (byte) -89, (byte) 101, (byte) -62, (byte) 97, (byte) 103, (byte) -81}, 0);
		CommunicationChannel.instance.send(p249);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_DEBUG_VECT.add((src, ph, pack) ->
		{
			assert (pack.name_LEN(ph) == 4);
			assert (pack.name_TRY(ph).equals("Pezv"));
			assert (pack.y_GET() == -2.8717121E38F);
			assert (pack.time_usec_GET() == 8678816376780444186L);
			assert (pack.z_GET() == -8.0189625E37F);
			assert (pack.x_GET() == -2.4266272E38F);
		});
		GroundControl.DEBUG_VECT p250 = CommunicationChannel.new_DEBUG_VECT();
		PH.setPack(p250);
		p250.name_SET("Pezv", PH);
		p250.x_SET(-2.4266272E38F);
		p250.z_SET(-8.0189625E37F);
		p250.y_SET(-2.8717121E38F);
		p250.time_usec_SET(8678816376780444186L);
		CommunicationChannel.instance.send(p250);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_NAMED_VALUE_FLOAT.add((src, ph, pack) ->
		{
			assert (pack.time_boot_ms_GET() == 61921095L);
			assert (pack.name_LEN(ph) == 2);
			assert (pack.name_TRY(ph).equals("fd"));
			assert (pack.value_GET() == 1.6353194E38F);
		});
		GroundControl.NAMED_VALUE_FLOAT p251 = CommunicationChannel.new_NAMED_VALUE_FLOAT();
		PH.setPack(p251);
		p251.time_boot_ms_SET(61921095L);
		p251.name_SET("fd", PH);
		p251.value_SET(1.6353194E38F);
		CommunicationChannel.instance.send(p251);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_NAMED_VALUE_INT.add((src, ph, pack) ->
		{
			assert (pack.name_LEN(ph) == 2);
			assert (pack.name_TRY(ph).equals("In"));
			assert (pack.value_GET() == 601733616);
			assert (pack.time_boot_ms_GET() == 75341095L);
		});
		GroundControl.NAMED_VALUE_INT p252 = CommunicationChannel.new_NAMED_VALUE_INT();
		PH.setPack(p252);
		p252.name_SET("In", PH);
		p252.time_boot_ms_SET(75341095L);
		p252.value_SET(601733616);
		CommunicationChannel.instance.send(p252);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_STATUSTEXT.add((src, ph, pack) ->
		{
			assert (pack.severity_GET() == MAV_SEVERITY.MAV_SEVERITY_INFO);
			assert (pack.text_LEN(ph) == 38);
			assert (pack.text_TRY(ph).equals("yObxhrwfqrijwBsyzqoumFdfxwkhqPnhSvhana"));
		});
		GroundControl.STATUSTEXT p253 = CommunicationChannel.new_STATUSTEXT();
		PH.setPack(p253);
		p253.text_SET("yObxhrwfqrijwBsyzqoumFdfxwkhqPnhSvhana", PH);
		p253.severity_SET(MAV_SEVERITY.MAV_SEVERITY_INFO);
		CommunicationChannel.instance.send(p253);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_DEBUG.add((src, ph, pack) ->
		{
			assert (pack.time_boot_ms_GET() == 1861386547L);
			assert (pack.value_GET() == -2.516883E38F);
			assert (pack.ind_GET() == (char) 16);
		});
		GroundControl.DEBUG p254 = CommunicationChannel.new_DEBUG();
		PH.setPack(p254);
		p254.time_boot_ms_SET(1861386547L);
		p254.value_SET(-2.516883E38F);
		p254.ind_SET((char) 16);
		CommunicationChannel.instance.send(p254);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_SETUP_SIGNING.add((src, ph, pack) ->
		{
			assert (pack.target_system_GET() == (char) 66);
			assert (pack.initial_timestamp_GET() == 1222805724751834098L);
			assert (pack.target_component_GET() == (char) 213);
			assert (Arrays.equals(pack.secret_key_GET(), new char[]{(char) 35, (char) 64, (char) 196, (char) 184, (char) 233, (char) 48, (char) 146, (char) 99, (char) 254, (char) 67, (char) 107, (char) 18, (char) 206, (char) 129, (char) 207, (char) 169, (char) 69, (char) 4, (char) 192, (char) 245, (char) 222, (char) 103, (char) 82, (char) 63, (char) 109, (char) 126, (char) 176, (char) 56, (char) 169, (char) 180, (char) 105, (char) 47}));
		});
		GroundControl.SETUP_SIGNING p256 = CommunicationChannel.new_SETUP_SIGNING();
		PH.setPack(p256);
		p256.initial_timestamp_SET(1222805724751834098L);
		p256.secret_key_SET(new char[]{(char) 35, (char) 64, (char) 196, (char) 184, (char) 233, (char) 48, (char) 146, (char) 99, (char) 254, (char) 67, (char) 107, (char) 18, (char) 206, (char) 129, (char) 207, (char) 169, (char) 69, (char) 4, (char) 192, (char) 245, (char) 222, (char) 103, (char) 82, (char) 63, (char) 109, (char) 126, (char) 176, (char) 56, (char) 169, (char) 180, (char) 105, (char) 47}, 0);
		p256.target_component_SET((char) 213);
		p256.target_system_SET((char) 66);
		CommunicationChannel.instance.send(p256);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_BUTTON_CHANGE.add((src, ph, pack) ->
		{
			assert (pack.time_boot_ms_GET() == 1689003068L);
			assert (pack.state_GET() == (char) 29);
			assert (pack.last_change_ms_GET() == 1189153624L);
		});
		GroundControl.BUTTON_CHANGE p257 = CommunicationChannel.new_BUTTON_CHANGE();
		PH.setPack(p257);
		p257.time_boot_ms_SET(1689003068L);
		p257.state_SET((char) 29);
		p257.last_change_ms_SET(1189153624L);
		CommunicationChannel.instance.send(p257);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_PLAY_TUNE.add((src, ph, pack) ->
		{
			assert (pack.target_system_GET() == (char) 142);
			assert (pack.target_component_GET() == (char) 234);
			assert (pack.tune_LEN(ph) == 25);
			assert (pack.tune_TRY(ph).equals("ddidtjhOebdXcvkulrnzqJwdb"));
		});
		GroundControl.PLAY_TUNE p258 = CommunicationChannel.new_PLAY_TUNE();
		PH.setPack(p258);
		p258.tune_SET("ddidtjhOebdXcvkulrnzqJwdb", PH);
		p258.target_system_SET((char) 142);
		p258.target_component_SET((char) 234);
		CommunicationChannel.instance.send(p258);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_CAMERA_INFORMATION.add((src, ph, pack) ->
		{
			assert (pack.cam_definition_version_GET() == (char) 29278);
			assert (pack.sensor_size_h_GET() == 1.784218E38F);
			assert (Arrays.equals(pack.model_name_GET(), new char[]{(char) 43, (char) 161, (char) 203, (char) 229, (char) 125, (char) 232, (char) 247, (char) 169, (char) 54, (char) 188, (char) 222, (char) 76, (char) 5, (char) 244, (char) 204, (char) 83, (char) 28, (char) 119, (char) 92, (char) 218, (char) 85, (char) 17, (char) 158, (char) 222, (char) 236, (char) 91, (char) 183, (char) 255, (char) 218, (char) 98, (char) 201, (char) 230}));
			assert (Arrays.equals(pack.vendor_name_GET(), new char[]{(char) 152, (char) 159, (char) 75, (char) 37, (char) 12, (char) 46, (char) 51, (char) 137, (char) 34, (char) 254, (char) 48, (char) 244, (char) 203, (char) 177, (char) 215, (char) 46, (char) 15, (char) 28, (char) 103, (char) 149, (char) 191, (char) 14, (char) 83, (char) 124, (char) 175, (char) 15, (char) 237, (char) 224, (char) 158, (char) 212, (char) 166, (char) 193}));
			assert (pack.lens_id_GET() == (char) 12);
			assert (pack.time_boot_ms_GET() == 2861781214L);
			assert (pack.resolution_h_GET() == (char) 34827);
			assert (pack.resolution_v_GET() == (char) 16476);
			assert (pack.firmware_version_GET() == 4075399819L);
			assert (pack.cam_definition_uri_LEN(ph) == 140);
			assert (pack.cam_definition_uri_TRY(ph).equals("xcqaMzicrgeVnNfdOvracMslydidjxscPltxosvZEtgbltifimyzvydfkvibadmniNhgoryxsgzqhsgJmqxFfYgwfpiaskntJNduJcanhdvyikmiauvnluvhwyqliqjkWwvfydSxykmW"));
			assert (pack.flags_GET() == CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE);
			assert (pack.focal_length_GET() == -2.4031844E38F);
			assert (pack.sensor_size_v_GET() == 5.080246E37F);
		});
		GroundControl.CAMERA_INFORMATION p259 = CommunicationChannel.new_CAMERA_INFORMATION();
		PH.setPack(p259);
		p259.firmware_version_SET(4075399819L);
		p259.sensor_size_v_SET(5.080246E37F);
		p259.sensor_size_h_SET(1.784218E38F);
		p259.vendor_name_SET(new char[]{(char) 152, (char) 159, (char) 75, (char) 37, (char) 12, (char) 46, (char) 51, (char) 137, (char) 34, (char) 254, (char) 48, (char) 244, (char) 203, (char) 177, (char) 215, (char) 46, (char) 15, (char) 28, (char) 103, (char) 149, (char) 191, (char) 14, (char) 83, (char) 124, (char) 175, (char) 15, (char) 237, (char) 224, (char) 158, (char) 212, (char) 166, (char) 193}, 0);
		p259.flags_SET(CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE);
		p259.resolution_h_SET((char) 34827);
		p259.resolution_v_SET((char) 16476);
		p259.time_boot_ms_SET(2861781214L);
		p259.model_name_SET(new char[]{(char) 43, (char) 161, (char) 203, (char) 229, (char) 125, (char) 232, (char) 247, (char) 169, (char) 54, (char) 188, (char) 222, (char) 76, (char) 5, (char) 244, (char) 204, (char) 83, (char) 28, (char) 119, (char) 92, (char) 218, (char) 85, (char) 17, (char) 158, (char) 222, (char) 236, (char) 91, (char) 183, (char) 255, (char) 218, (char) 98, (char) 201, (char) 230}, 0);
		p259.lens_id_SET((char) 12);
		p259.focal_length_SET(-2.4031844E38F);
		p259.cam_definition_uri_SET("xcqaMzicrgeVnNfdOvracMslydidjxscPltxosvZEtgbltifimyzvydfkvibadmniNhgoryxsgzqhsgJmqxFfYgwfpiaskntJNduJcanhdvyikmiauvnluvhwyqliqjkWwvfydSxykmW", PH);
		p259.cam_definition_version_SET((char) 29278);
		CommunicationChannel.instance.send(p259);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_CAMERA_SETTINGS.add((src, ph, pack) ->
		{
			assert (pack.mode_id_GET() == CAMERA_MODE.CAMERA_MODE_VIDEO);
			assert (pack.time_boot_ms_GET() == 2364129631L);
		});
		GroundControl.CAMERA_SETTINGS p260 = CommunicationChannel.new_CAMERA_SETTINGS();
		PH.setPack(p260);
		p260.mode_id_SET(CAMERA_MODE.CAMERA_MODE_VIDEO);
		p260.time_boot_ms_SET(2364129631L);
		CommunicationChannel.instance.send(p260);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_STORAGE_INFORMATION.add((src, ph, pack) ->
		{
			assert (pack.storage_id_GET() == (char) 72);
			assert (pack.status_GET() == (char) 216);
			assert (pack.storage_count_GET() == (char) 33);
			assert (pack.total_capacity_GET() == 1.2300584E38F);
			assert (pack.time_boot_ms_GET() == 1148671178L);
			assert (pack.available_capacity_GET() == 1.3250428E38F);
			assert (pack.used_capacity_GET() == 8.79856E37F);
			assert (pack.read_speed_GET() == 2.7980542E37F);
			assert (pack.write_speed_GET() == -2.774478E38F);
		});
		GroundControl.STORAGE_INFORMATION p261 = CommunicationChannel.new_STORAGE_INFORMATION();
		PH.setPack(p261);
		p261.read_speed_SET(2.7980542E37F);
		p261.status_SET((char) 216);
		p261.used_capacity_SET(8.79856E37F);
		p261.storage_id_SET((char) 72);
		p261.available_capacity_SET(1.3250428E38F);
		p261.time_boot_ms_SET(1148671178L);
		p261.write_speed_SET(-2.774478E38F);
		p261.storage_count_SET((char) 33);
		p261.total_capacity_SET(1.2300584E38F);
		CommunicationChannel.instance.send(p261);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_CAMERA_CAPTURE_STATUS.add((src, ph, pack) ->
		{
			assert (pack.time_boot_ms_GET() == 4175992858L);
			assert (pack.image_status_GET() == (char) 240);
			assert (pack.recording_time_ms_GET() == 2865479817L);
			assert (pack.video_status_GET() == (char) 212);
			assert (pack.available_capacity_GET() == 1.8978326E38F);
			assert (pack.image_interval_GET() == -2.5021084E38F);
		});
		GroundControl.CAMERA_CAPTURE_STATUS p262 = CommunicationChannel.new_CAMERA_CAPTURE_STATUS();
		PH.setPack(p262);
		p262.time_boot_ms_SET(4175992858L);
		p262.video_status_SET((char) 212);
		p262.available_capacity_SET(1.8978326E38F);
		p262.image_interval_SET(-2.5021084E38F);
		p262.image_status_SET((char) 240);
		p262.recording_time_ms_SET(2865479817L);
		CommunicationChannel.instance.send(p262);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_CAMERA_IMAGE_CAPTURED.add((src, ph, pack) ->
		{
			assert (pack.lon_GET() == -282243252);
			assert (pack.file_url_LEN(ph) == 92);
			assert (pack.file_url_TRY(ph).equals("HnvfhhveykzslqdejdgdyryraYwfaIqcuvdoNjttiijdrcftzTeylbwnldeiowunxnfkCezqyfdFvnSjjypbwnfpgMdl"));
			assert (pack.lat_GET() == 1619113158);
			assert (pack.time_boot_ms_GET() == 118614721L);
			assert (Arrays.equals(pack.q_GET(), new float[]{-1.0712552E38F, -3.0296778E38F, 3.1768952E37F, 7.432861E37F}));
			assert (pack.alt_GET() == -714676899);
			assert (pack.time_utc_GET() == 3708793786947821717L);
			assert (pack.relative_alt_GET() == 2088959397);
			assert (pack.capture_result_GET() == (byte) -37);
			assert (pack.camera_id_GET() == (char) 167);
			assert (pack.image_index_GET() == 1854666078);
		});
		GroundControl.CAMERA_IMAGE_CAPTURED p263 = CommunicationChannel.new_CAMERA_IMAGE_CAPTURED();
		PH.setPack(p263);
		p263.alt_SET(-714676899);
		p263.time_utc_SET(3708793786947821717L);
		p263.image_index_SET(1854666078);
		p263.q_SET(new float[]{-1.0712552E38F, -3.0296778E38F, 3.1768952E37F, 7.432861E37F}, 0);
		p263.relative_alt_SET(2088959397);
		p263.camera_id_SET((char) 167);
		p263.capture_result_SET((byte) -37);
		p263.lat_SET(1619113158);
		p263.lon_SET(-282243252);
		p263.file_url_SET("HnvfhhveykzslqdejdgdyryraYwfaIqcuvdoNjttiijdrcftzTeylbwnldeiowunxnfkCezqyfdFvnSjjypbwnfpgMdl", PH);
		p263.time_boot_ms_SET(118614721L);
		CommunicationChannel.instance.send(p263);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_FLIGHT_INFORMATION.add((src, ph, pack) ->
		{
			assert (pack.takeoff_time_utc_GET() == 5005146052453146057L);
			assert (pack.flight_uuid_GET() == 4459416448509350275L);
			assert (pack.time_boot_ms_GET() == 3935401911L);
			assert (pack.arming_time_utc_GET() == 1642241941592344290L);
		});
		GroundControl.FLIGHT_INFORMATION p264 = CommunicationChannel.new_FLIGHT_INFORMATION();
		PH.setPack(p264);
		p264.takeoff_time_utc_SET(5005146052453146057L);
		p264.flight_uuid_SET(4459416448509350275L);
		p264.arming_time_utc_SET(1642241941592344290L);
		p264.time_boot_ms_SET(3935401911L);
		CommunicationChannel.instance.send(p264);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_MOUNT_ORIENTATION.add((src, ph, pack) ->
		{
			assert (pack.pitch_GET() == -2.6335317E38F);
			assert (pack.time_boot_ms_GET() == 3966945020L);
			assert (pack.yaw_GET() == 3.1664736E38F);
			assert (pack.roll_GET() == 6.1783044E37F);
		});
		GroundControl.MOUNT_ORIENTATION p265 = CommunicationChannel.new_MOUNT_ORIENTATION();
		PH.setPack(p265);
		p265.roll_SET(6.1783044E37F);
		p265.yaw_SET(3.1664736E38F);
		p265.time_boot_ms_SET(3966945020L);
		p265.pitch_SET(-2.6335317E38F);
		CommunicationChannel.instance.send(p265);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_LOGGING_DATA.add((src, ph, pack) ->
		{
			assert (pack.target_system_GET() == (char) 20);
			assert (pack.length_GET() == (char) 118);
			assert (pack.first_message_offset_GET() == (char) 220);
			assert (Arrays.equals(pack.data__GET(), new char[]{(char) 152, (char) 125, (char) 173, (char) 88, (char) 160, (char) 66, (char) 31, (char) 203, (char) 51, (char) 84, (char) 234, (char) 197, (char) 144, (char) 69, (char) 96, (char) 43, (char) 205, (char) 106, (char) 206, (char) 159, (char) 232, (char) 107, (char) 36, (char) 152, (char) 96, (char) 201, (char) 172, (char) 165, (char) 171, (char) 107, (char) 57, (char) 140, (char) 193, (char) 35, (char) 53, (char) 4, (char) 228, (char) 202, (char) 178, (char) 48, (char) 75, (char) 212, (char) 56, (char) 29, (char) 82, (char) 110, (char) 31, (char) 138, (char) 127, (char) 17, (char) 136, (char) 20, (char) 24, (char) 116, (char) 168, (char) 1, (char) 42, (char) 25, (char) 247, (char) 235, (char) 58, (char) 100, (char) 202, (char) 211, (char) 209, (char) 131, (char) 227, (char) 13, (char) 219, (char) 84, (char) 88, (char) 84, (char) 75, (char) 147, (char) 78, (char) 237, (char) 221, (char) 114, (char) 36, (char) 86, (char) 87, (char) 96, (char) 37, (char) 186, (char) 167, (char) 219, (char) 247, (char) 177, (char) 198, (char) 215, (char) 239, (char) 215, (char) 198, (char) 229, (char) 70, (char) 92, (char) 29, (char) 98, (char) 236, (char) 139, (char) 23, (char) 19, (char) 248, (char) 193, (char) 54, (char) 73, (char) 137, (char) 145, (char) 65, (char) 130, (char) 183, (char) 52, (char) 109, (char) 36, (char) 4, (char) 24, (char) 158, (char) 251, (char) 217, (char) 46, (char) 126, (char) 134, (char) 149, (char) 0, (char) 42, (char) 143, (char) 242, (char) 179, (char) 247, (char) 91, (char) 110, (char) 125, (char) 243, (char) 114, (char) 130, (char) 47, (char) 20, (char) 176, (char) 61, (char) 125, (char) 229, (char) 74, (char) 130, (char) 45, (char) 24, (char) 200, (char) 98, (char) 187, (char) 213, (char) 209, (char) 243, (char) 20, (char) 23, (char) 47, (char) 245, (char) 152, (char) 100, (char) 35, (char) 176, (char) 7, (char) 139, (char) 5, (char) 179, (char) 19, (char) 20, (char) 157, (char) 251, (char) 223, (char) 177, (char) 218, (char) 243, (char) 81, (char) 12, (char) 157, (char) 55, (char) 111, (char) 15, (char) 35, (char) 251, (char) 33, (char) 109, (char) 81, (char) 113, (char) 208, (char) 136, (char) 104, (char) 158, (char) 49, (char) 79, (char) 83, (char) 11, (char) 127, (char) 3, (char) 206, (char) 119, (char) 106, (char) 5, (char) 166, (char) 36, (char) 13, (char) 136, (char) 165, (char) 207, (char) 176, (char) 238, (char) 133, (char) 42, (char) 179, (char) 166, (char) 156, (char) 44, (char) 49, (char) 24, (char) 241, (char) 58, (char) 192, (char) 75, (char) 126, (char) 85, (char) 146, (char) 181, (char) 91, (char) 133, (char) 158, (char) 234, (char) 53, (char) 216, (char) 223, (char) 194, (char) 236, (char) 39, (char) 216, (char) 68, (char) 161, (char) 149, (char) 124, (char) 220, (char) 159, (char) 33, (char) 32, (char) 86, (char) 199, (char) 103, (char) 206, (char) 121, (char) 130, (char) 44, (char) 207, (char) 174}));
			assert (pack.target_component_GET() == (char) 10);
			assert (pack.sequence_GET() == (char) 53374);
		});
		GroundControl.LOGGING_DATA p266 = CommunicationChannel.new_LOGGING_DATA();
		PH.setPack(p266);
		p266.sequence_SET((char) 53374);
		p266.target_system_SET((char) 20);
		p266.length_SET((char) 118);
		p266.target_component_SET((char) 10);
		p266.first_message_offset_SET((char) 220);
		p266.data__SET(new char[]{(char) 152, (char) 125, (char) 173, (char) 88, (char) 160, (char) 66, (char) 31, (char) 203, (char) 51, (char) 84, (char) 234, (char) 197, (char) 144, (char) 69, (char) 96, (char) 43, (char) 205, (char) 106, (char) 206, (char) 159, (char) 232, (char) 107, (char) 36, (char) 152, (char) 96, (char) 201, (char) 172, (char) 165, (char) 171, (char) 107, (char) 57, (char) 140, (char) 193, (char) 35, (char) 53, (char) 4, (char) 228, (char) 202, (char) 178, (char) 48, (char) 75, (char) 212, (char) 56, (char) 29, (char) 82, (char) 110, (char) 31, (char) 138, (char) 127, (char) 17, (char) 136, (char) 20, (char) 24, (char) 116, (char) 168, (char) 1, (char) 42, (char) 25, (char) 247, (char) 235, (char) 58, (char) 100, (char) 202, (char) 211, (char) 209, (char) 131, (char) 227, (char) 13, (char) 219, (char) 84, (char) 88, (char) 84, (char) 75, (char) 147, (char) 78, (char) 237, (char) 221, (char) 114, (char) 36, (char) 86, (char) 87, (char) 96, (char) 37, (char) 186, (char) 167, (char) 219, (char) 247, (char) 177, (char) 198, (char) 215, (char) 239, (char) 215, (char) 198, (char) 229, (char) 70, (char) 92, (char) 29, (char) 98, (char) 236, (char) 139, (char) 23, (char) 19, (char) 248, (char) 193, (char) 54, (char) 73, (char) 137, (char) 145, (char) 65, (char) 130, (char) 183, (char) 52, (char) 109, (char) 36, (char) 4, (char) 24, (char) 158, (char) 251, (char) 217, (char) 46, (char) 126, (char) 134, (char) 149, (char) 0, (char) 42, (char) 143, (char) 242, (char) 179, (char) 247, (char) 91, (char) 110, (char) 125, (char) 243, (char) 114, (char) 130, (char) 47, (char) 20, (char) 176, (char) 61, (char) 125, (char) 229, (char) 74, (char) 130, (char) 45, (char) 24, (char) 200, (char) 98, (char) 187, (char) 213, (char) 209, (char) 243, (char) 20, (char) 23, (char) 47, (char) 245, (char) 152, (char) 100, (char) 35, (char) 176, (char) 7, (char) 139, (char) 5, (char) 179, (char) 19, (char) 20, (char) 157, (char) 251, (char) 223, (char) 177, (char) 218, (char) 243, (char) 81, (char) 12, (char) 157, (char) 55, (char) 111, (char) 15, (char) 35, (char) 251, (char) 33, (char) 109, (char) 81, (char) 113, (char) 208, (char) 136, (char) 104, (char) 158, (char) 49, (char) 79, (char) 83, (char) 11, (char) 127, (char) 3, (char) 206, (char) 119, (char) 106, (char) 5, (char) 166, (char) 36, (char) 13, (char) 136, (char) 165, (char) 207, (char) 176, (char) 238, (char) 133, (char) 42, (char) 179, (char) 166, (char) 156, (char) 44, (char) 49, (char) 24, (char) 241, (char) 58, (char) 192, (char) 75, (char) 126, (char) 85, (char) 146, (char) 181, (char) 91, (char) 133, (char) 158, (char) 234, (char) 53, (char) 216, (char) 223, (char) 194, (char) 236, (char) 39, (char) 216, (char) 68, (char) 161, (char) 149, (char) 124, (char) 220, (char) 159, (char) 33, (char) 32, (char) 86, (char) 199, (char) 103, (char) 206, (char) 121, (char) 130, (char) 44, (char) 207, (char) 174}, 0);
		CommunicationChannel.instance.send(p266);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_LOGGING_DATA_ACKED.add((src, ph, pack) ->
		{
			assert (pack.sequence_GET() == (char) 55014);
			assert (Arrays.equals(pack.data__GET(), new char[]{(char) 168, (char) 138, (char) 209, (char) 193, (char) 194, (char) 134, (char) 218, (char) 125, (char) 92, (char) 237, (char) 57, (char) 136, (char) 235, (char) 145, (char) 123, (char) 184, (char) 141, (char) 231, (char) 148, (char) 32, (char) 184, (char) 223, (char) 190, (char) 28, (char) 37, (char) 75, (char) 76, (char) 14, (char) 139, (char) 230, (char) 164, (char) 4, (char) 105, (char) 69, (char) 122, (char) 195, (char) 9, (char) 241, (char) 157, (char) 91, (char) 11, (char) 251, (char) 200, (char) 4, (char) 104, (char) 237, (char) 131, (char) 4, (char) 30, (char) 169, (char) 226, (char) 216, (char) 213, (char) 83, (char) 253, (char) 196, (char) 255, (char) 204, (char) 23, (char) 138, (char) 173, (char) 17, (char) 17, (char) 62, (char) 137, (char) 9, (char) 6, (char) 149, (char) 100, (char) 172, (char) 226, (char) 4, (char) 13, (char) 87, (char) 53, (char) 95, (char) 87, (char) 11, (char) 166, (char) 171, (char) 152, (char) 112, (char) 168, (char) 46, (char) 147, (char) 179, (char) 114, (char) 20, (char) 217, (char) 78, (char) 95, (char) 148, (char) 17, (char) 25, (char) 238, (char) 175, (char) 131, (char) 136, (char) 174, (char) 162, (char) 250, (char) 216, (char) 7, (char) 195, (char) 14, (char) 171, (char) 247, (char) 241, (char) 91, (char) 135, (char) 209, (char) 209, (char) 0, (char) 207, (char) 7, (char) 248, (char) 213, (char) 160, (char) 197, (char) 152, (char) 226, (char) 212, (char) 75, (char) 238, (char) 24, (char) 67, (char) 123, (char) 216, (char) 210, (char) 139, (char) 195, (char) 22, (char) 215, (char) 116, (char) 165, (char) 176, (char) 159, (char) 2, (char) 109, (char) 19, (char) 12, (char) 82, (char) 186, (char) 133, (char) 69, (char) 19, (char) 40, (char) 116, (char) 190, (char) 143, (char) 139, (char) 70, (char) 19, (char) 15, (char) 239, (char) 84, (char) 96, (char) 179, (char) 87, (char) 82, (char) 209, (char) 44, (char) 28, (char) 192, (char) 200, (char) 162, (char) 190, (char) 188, (char) 95, (char) 81, (char) 70, (char) 20, (char) 206, (char) 4, (char) 235, (char) 188, (char) 187, (char) 189, (char) 162, (char) 1, (char) 92, (char) 245, (char) 12, (char) 79, (char) 23, (char) 128, (char) 96, (char) 185, (char) 236, (char) 85, (char) 160, (char) 196, (char) 231, (char) 184, (char) 230, (char) 214, (char) 44, (char) 251, (char) 114, (char) 66, (char) 164, (char) 140, (char) 163, (char) 242, (char) 235, (char) 39, (char) 155, (char) 119, (char) 188, (char) 233, (char) 30, (char) 37, (char) 240, (char) 72, (char) 15, (char) 49, (char) 63, (char) 162, (char) 8, (char) 109, (char) 41, (char) 202, (char) 189, (char) 55, (char) 182, (char) 85, (char) 195, (char) 33, (char) 211, (char) 45, (char) 54, (char) 71, (char) 56, (char) 96, (char) 221, (char) 158, (char) 1, (char) 234, (char) 80, (char) 128, (char) 205, (char) 22, (char) 20, (char) 66, (char) 196, (char) 14, (char) 50, (char) 4, (char) 14}));
			assert (pack.length_GET() == (char) 153);
			assert (pack.first_message_offset_GET() == (char) 83);
			assert (pack.target_system_GET() == (char) 105);
			assert (pack.target_component_GET() == (char) 192);
		});
		GroundControl.LOGGING_DATA_ACKED p267 = CommunicationChannel.new_LOGGING_DATA_ACKED();
		PH.setPack(p267);
		p267.data__SET(new char[]{(char) 168, (char) 138, (char) 209, (char) 193, (char) 194, (char) 134, (char) 218, (char) 125, (char) 92, (char) 237, (char) 57, (char) 136, (char) 235, (char) 145, (char) 123, (char) 184, (char) 141, (char) 231, (char) 148, (char) 32, (char) 184, (char) 223, (char) 190, (char) 28, (char) 37, (char) 75, (char) 76, (char) 14, (char) 139, (char) 230, (char) 164, (char) 4, (char) 105, (char) 69, (char) 122, (char) 195, (char) 9, (char) 241, (char) 157, (char) 91, (char) 11, (char) 251, (char) 200, (char) 4, (char) 104, (char) 237, (char) 131, (char) 4, (char) 30, (char) 169, (char) 226, (char) 216, (char) 213, (char) 83, (char) 253, (char) 196, (char) 255, (char) 204, (char) 23, (char) 138, (char) 173, (char) 17, (char) 17, (char) 62, (char) 137, (char) 9, (char) 6, (char) 149, (char) 100, (char) 172, (char) 226, (char) 4, (char) 13, (char) 87, (char) 53, (char) 95, (char) 87, (char) 11, (char) 166, (char) 171, (char) 152, (char) 112, (char) 168, (char) 46, (char) 147, (char) 179, (char) 114, (char) 20, (char) 217, (char) 78, (char) 95, (char) 148, (char) 17, (char) 25, (char) 238, (char) 175, (char) 131, (char) 136, (char) 174, (char) 162, (char) 250, (char) 216, (char) 7, (char) 195, (char) 14, (char) 171, (char) 247, (char) 241, (char) 91, (char) 135, (char) 209, (char) 209, (char) 0, (char) 207, (char) 7, (char) 248, (char) 213, (char) 160, (char) 197, (char) 152, (char) 226, (char) 212, (char) 75, (char) 238, (char) 24, (char) 67, (char) 123, (char) 216, (char) 210, (char) 139, (char) 195, (char) 22, (char) 215, (char) 116, (char) 165, (char) 176, (char) 159, (char) 2, (char) 109, (char) 19, (char) 12, (char) 82, (char) 186, (char) 133, (char) 69, (char) 19, (char) 40, (char) 116, (char) 190, (char) 143, (char) 139, (char) 70, (char) 19, (char) 15, (char) 239, (char) 84, (char) 96, (char) 179, (char) 87, (char) 82, (char) 209, (char) 44, (char) 28, (char) 192, (char) 200, (char) 162, (char) 190, (char) 188, (char) 95, (char) 81, (char) 70, (char) 20, (char) 206, (char) 4, (char) 235, (char) 188, (char) 187, (char) 189, (char) 162, (char) 1, (char) 92, (char) 245, (char) 12, (char) 79, (char) 23, (char) 128, (char) 96, (char) 185, (char) 236, (char) 85, (char) 160, (char) 196, (char) 231, (char) 184, (char) 230, (char) 214, (char) 44, (char) 251, (char) 114, (char) 66, (char) 164, (char) 140, (char) 163, (char) 242, (char) 235, (char) 39, (char) 155, (char) 119, (char) 188, (char) 233, (char) 30, (char) 37, (char) 240, (char) 72, (char) 15, (char) 49, (char) 63, (char) 162, (char) 8, (char) 109, (char) 41, (char) 202, (char) 189, (char) 55, (char) 182, (char) 85, (char) 195, (char) 33, (char) 211, (char) 45, (char) 54, (char) 71, (char) 56, (char) 96, (char) 221, (char) 158, (char) 1, (char) 234, (char) 80, (char) 128, (char) 205, (char) 22, (char) 20, (char) 66, (char) 196, (char) 14, (char) 50, (char) 4, (char) 14}, 0);
		p267.first_message_offset_SET((char) 83);
		p267.target_system_SET((char) 105);
		p267.length_SET((char) 153);
		p267.sequence_SET((char) 55014);
		p267.target_component_SET((char) 192);
		CommunicationChannel.instance.send(p267);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_LOGGING_ACK.add((src, ph, pack) ->
		{
			assert (pack.target_system_GET() == (char) 81);
			assert (pack.target_component_GET() == (char) 224);
			assert (pack.sequence_GET() == (char) 4744);
		});
		GroundControl.LOGGING_ACK p268 = CommunicationChannel.new_LOGGING_ACK();
		PH.setPack(p268);
		p268.target_system_SET((char) 81);
		p268.target_component_SET((char) 224);
		p268.sequence_SET((char) 4744);
		CommunicationChannel.instance.send(p268);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_VIDEO_STREAM_INFORMATION.add((src, ph, pack) ->
		{
			assert (pack.uri_LEN(ph) == 195);
			assert (pack.uri_TRY(ph).equals("xjthpxerrckzhubucyaosizefwoxqrtryayTbiAyaishibhhxyuiysfwfqscxajjhnsxsezfwqcwqjglpgyxbvrXqGThHmKcvzvWyaloyxlskEuketoloofyoreqrkhznewvxoitqtxlxssfbxyccokriweqisgpZdkjuvryFqclnzaffVuvuzktugggbcFWuor"));
			assert (pack.framerate_GET() == 9.246153E37F);
			assert (pack.resolution_h_GET() == (char) 64782);
			assert (pack.camera_id_GET() == (char) 196);
			assert (pack.status_GET() == (char) 107);
			assert (pack.bitrate_GET() == 3393826735L);
			assert (pack.rotation_GET() == (char) 31195);
			assert (pack.resolution_v_GET() == (char) 33352);
		});
		GroundControl.VIDEO_STREAM_INFORMATION p269 = CommunicationChannel.new_VIDEO_STREAM_INFORMATION();
		PH.setPack(p269);
		p269.framerate_SET(9.246153E37F);
		p269.resolution_v_SET((char) 33352);
		p269.rotation_SET((char) 31195);
		p269.camera_id_SET((char) 196);
		p269.status_SET((char) 107);
		p269.resolution_h_SET((char) 64782);
		p269.bitrate_SET(3393826735L);
		p269.uri_SET("xjthpxerrckzhubucyaosizefwoxqrtryayTbiAyaishibhhxyuiysfwfqscxajjhnsxsezfwqcwqjglpgyxbvrXqGThHmKcvzvWyaloyxlskEuketoloofyoreqrkhznewvxoitqtxlxssfbxyccokriweqisgpZdkjuvryFqclnzaffVuvuzktugggbcFWuor", PH);
		CommunicationChannel.instance.send(p269);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_SET_VIDEO_STREAM_SETTINGS.add((src, ph, pack) ->
		{
			assert (pack.resolution_h_GET() == (char) 60520);
			assert (pack.framerate_GET() == 9.093555E37F);
			assert (pack.camera_id_GET() == (char) 125);
			assert (pack.target_component_GET() == (char) 182);
			assert (pack.target_system_GET() == (char) 16);
			assert (pack.bitrate_GET() == 3914060879L);
			assert (pack.resolution_v_GET() == (char) 64004);
			assert (pack.uri_LEN(ph) == 175);
			assert (pack.uri_TRY(ph).equals("anhymGkeuEkpminKnmqaopAbcwhstzoDlyafzabwwmvkejbczGoztezzhtofwhmfxugbdfbFtJbxygYamplkbmvjngwmzxkqKTjnhgoiyeghvnytyznjskphoRngwqnuqbzsqvjorcicxfmbLInhvijrydidxleoxwrbqncfyzozDSr"));
			assert (pack.rotation_GET() == (char) 12968);
		});
		GroundControl.SET_VIDEO_STREAM_SETTINGS p270 = CommunicationChannel.new_SET_VIDEO_STREAM_SETTINGS();
		PH.setPack(p270);
		p270.resolution_v_SET((char) 64004);
		p270.rotation_SET((char) 12968);
		p270.target_system_SET((char) 16);
		p270.resolution_h_SET((char) 60520);
		p270.target_component_SET((char) 182);
		p270.framerate_SET(9.093555E37F);
		p270.camera_id_SET((char) 125);
		p270.bitrate_SET(3914060879L);
		p270.uri_SET("anhymGkeuEkpminKnmqaopAbcwhstzoDlyafzabwwmvkejbczGoztezzhtofwhmfxugbdfbFtJbxygYamplkbmvjngwmzxkqKTjnhgoiyeghvnytyznjskphoRngwqnuqbzsqvjorcicxfmbLInhvijrydidxleoxwrbqncfyzozDSr", PH);
		CommunicationChannel.instance.send(p270);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_WIFI_CONFIG_AP.add((src, ph, pack) ->
		{
			assert (pack.ssid_LEN(ph) == 15);
			assert (pack.ssid_TRY(ph).equals("XfJyyotfdmjrmri"));
			assert (pack.password_LEN(ph) == 53);
			assert (pack.password_TRY(ph).equals("SmzEbljzllgfroylsXdbojGlvkjmkaiayqvtixracxrxzxotqvljk"));
		});
		GroundControl.WIFI_CONFIG_AP p299 = CommunicationChannel.new_WIFI_CONFIG_AP();
		PH.setPack(p299);
		p299.password_SET("SmzEbljzllgfroylsXdbojGlvkjmkaiayqvtixracxrxzxotqvljk", PH);
		p299.ssid_SET("XfJyyotfdmjrmri", PH);
		CommunicationChannel.instance.send(p299);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_PROTOCOL_VERSION.add((src, ph, pack) ->
		{
			assert (Arrays.equals(pack.spec_version_hash_GET(), new char[]{(char) 60, (char) 185, (char) 103, (char) 4, (char) 223, (char) 249, (char) 46, (char) 20}));
			assert (pack.min_version_GET() == (char) 38053);
			assert (pack.max_version_GET() == (char) 14800);
			assert (pack.version_GET() == (char) 13191);
			assert (Arrays.equals(pack.library_version_hash_GET(), new char[]{(char) 228, (char) 247, (char) 19, (char) 175, (char) 161, (char) 189, (char) 73, (char) 106}));
		});
		GroundControl.PROTOCOL_VERSION p300 = CommunicationChannel.new_PROTOCOL_VERSION();
		PH.setPack(p300);
		p300.version_SET((char) 13191);
		p300.library_version_hash_SET(new char[]{(char) 228, (char) 247, (char) 19, (char) 175, (char) 161, (char) 189, (char) 73, (char) 106}, 0);
		p300.min_version_SET((char) 38053);
		p300.spec_version_hash_SET(new char[]{(char) 60, (char) 185, (char) 103, (char) 4, (char) 223, (char) 249, (char) 46, (char) 20}, 0);
		p300.max_version_SET((char) 14800);
		CommunicationChannel.instance.send(p300);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_UAVCAN_NODE_STATUS.add((src, ph, pack) ->
		{
			assert (pack.vendor_specific_status_code_GET() == (char) 43056);
			assert (pack.uptime_sec_GET() == 424461229L);
			assert (pack.time_usec_GET() == 1935777876715546336L);
			assert (pack.sub_mode_GET() == (char) 194);
			assert (pack.mode_GET() == UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_MAINTENANCE);
			assert (pack.health_GET() == UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_WARNING);
		});
		GroundControl.UAVCAN_NODE_STATUS p310 = CommunicationChannel.new_UAVCAN_NODE_STATUS();
		PH.setPack(p310);
		p310.time_usec_SET(1935777876715546336L);
		p310.health_SET(UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_WARNING);
		p310.vendor_specific_status_code_SET((char) 43056);
		p310.sub_mode_SET((char) 194);
		p310.mode_SET(UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_MAINTENANCE);
		p310.uptime_sec_SET(424461229L);
		CommunicationChannel.instance.send(p310);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_UAVCAN_NODE_INFO.add((src, ph, pack) ->
		{
			assert (pack.sw_vcs_commit_GET() == 3241292740L);
			assert (pack.uptime_sec_GET() == 1478530686L);
			assert (pack.name_LEN(ph) == 10);
			assert (pack.name_TRY(ph).equals("ovcUgnkhCn"));
			assert (pack.time_usec_GET() == 6444994035598778648L);
			assert (pack.hw_version_minor_GET() == (char) 121);
			assert (pack.hw_version_major_GET() == (char) 209);
			assert (Arrays.equals(pack.hw_unique_id_GET(), new char[]{(char) 159, (char) 166, (char) 48, (char) 150, (char) 125, (char) 49, (char) 52, (char) 216, (char) 2, (char) 34, (char) 69, (char) 115, (char) 218, (char) 28, (char) 163, (char) 30}));
			assert (pack.sw_version_major_GET() == (char) 100);
			assert (pack.sw_version_minor_GET() == (char) 107);
		});
		GroundControl.UAVCAN_NODE_INFO p311 = CommunicationChannel.new_UAVCAN_NODE_INFO();
		PH.setPack(p311);
		p311.uptime_sec_SET(1478530686L);
		p311.hw_unique_id_SET(new char[]{(char) 159, (char) 166, (char) 48, (char) 150, (char) 125, (char) 49, (char) 52, (char) 216, (char) 2, (char) 34, (char) 69, (char) 115, (char) 218, (char) 28, (char) 163, (char) 30}, 0);
		p311.name_SET("ovcUgnkhCn", PH);
		p311.hw_version_minor_SET((char) 121);
		p311.sw_version_major_SET((char) 100);
		p311.sw_vcs_commit_SET(3241292740L);
		p311.hw_version_major_SET((char) 209);
		p311.sw_version_minor_SET((char) 107);
		p311.time_usec_SET(6444994035598778648L);
		CommunicationChannel.instance.send(p311);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_PARAM_EXT_REQUEST_READ.add((src, ph, pack) ->
		{
			assert (pack.target_component_GET() == (char) 9);
			assert (pack.param_index_GET() == (short) -27249);
			assert (pack.target_system_GET() == (char) 109);
			assert (pack.param_id_LEN(ph) == 9);
			assert (pack.param_id_TRY(ph).equals("aklgcycpK"));
		});
		GroundControl.PARAM_EXT_REQUEST_READ p320 = CommunicationChannel.new_PARAM_EXT_REQUEST_READ();
		PH.setPack(p320);
		p320.param_id_SET("aklgcycpK", PH);
		p320.target_component_SET((char) 9);
		p320.target_system_SET((char) 109);
		p320.param_index_SET((short) -27249);
		CommunicationChannel.instance.send(p320);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_PARAM_EXT_REQUEST_LIST.add((src, ph, pack) ->
		{
			assert (pack.target_system_GET() == (char) 84);
			assert (pack.target_component_GET() == (char) 31);
		});
		GroundControl.PARAM_EXT_REQUEST_LIST p321 = CommunicationChannel.new_PARAM_EXT_REQUEST_LIST();
		PH.setPack(p321);
		p321.target_system_SET((char) 84);
		p321.target_component_SET((char) 31);
		CommunicationChannel.instance.send(p321);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_PARAM_EXT_VALUE.add((src, ph, pack) ->
		{
			assert (pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT8);
			assert (pack.param_count_GET() == (char) 63952);
			assert (pack.param_index_GET() == (char) 57631);
			assert (pack.param_value_LEN(ph) == 87);
			assert (pack.param_value_TRY(ph).equals("ujswjpxdgxxbErzlZbwBmdjilqsuaygrpecRnvpbegMszpuDpvpweybzvcrpqCeysukiauZsfktjawdbqpsVvas"));
			assert (pack.param_id_LEN(ph) == 7);
			assert (pack.param_id_TRY(ph).equals("eaziQrv"));
		});
		GroundControl.PARAM_EXT_VALUE p322 = CommunicationChannel.new_PARAM_EXT_VALUE();
		PH.setPack(p322);
		p322.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT8);
		p322.param_id_SET("eaziQrv", PH);
		p322.param_index_SET((char) 57631);
		p322.param_count_SET((char) 63952);
		p322.param_value_SET("ujswjpxdgxxbErzlZbwBmdjilqsuaygrpecRnvpbegMszpuDpvpweybzvcrpqCeysukiauZsfktjawdbqpsVvas", PH);
		CommunicationChannel.instance.send(p322);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_PARAM_EXT_SET.add((src, ph, pack) ->
		{
			assert (pack.param_id_LEN(ph) == 1);
			assert (pack.param_id_TRY(ph).equals("S"));
			assert (pack.target_system_GET() == (char) 152);
			assert (pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT64);
			assert (pack.target_component_GET() == (char) 77);
			assert (pack.param_value_LEN(ph) == 51);
			assert (pack.param_value_TRY(ph).equals("drdiocaVeslvfybcewuqwajufeDHepyjiyrntKkjnxqBncegifi"));
		});
		GroundControl.PARAM_EXT_SET p323 = CommunicationChannel.new_PARAM_EXT_SET();
		PH.setPack(p323);
		p323.target_component_SET((char) 77);
		p323.param_id_SET("S", PH);
		p323.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT64);
		p323.param_value_SET("drdiocaVeslvfybcewuqwajufeDHepyjiyrntKkjnxqBncegifi", PH);
		p323.target_system_SET((char) 152);
		CommunicationChannel.instance.send(p323);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_PARAM_EXT_ACK.add((src, ph, pack) ->
		{
			assert (pack.param_value_LEN(ph) == 10);
			assert (pack.param_value_TRY(ph).equals("vnxkfauuds"));
			assert (pack.param_result_GET() == PARAM_ACK.PARAM_ACK_FAILED);
			assert (pack.param_id_LEN(ph) == 16);
			assert (pack.param_id_TRY(ph).equals("dwmdbWesJOGapqez"));
			assert (pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT16);
		});
		GroundControl.PARAM_EXT_ACK p324 = CommunicationChannel.new_PARAM_EXT_ACK();
		PH.setPack(p324);
		p324.param_result_SET(PARAM_ACK.PARAM_ACK_FAILED);
		p324.param_id_SET("dwmdbWesJOGapqez", PH);
		p324.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT16);
		p324.param_value_SET("vnxkfauuds", PH);
		CommunicationChannel.instance.send(p324);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
		TestChannel.instance.on_OBSTACLE_DISTANCE.add((src, ph, pack) ->
		{
			assert (pack.increment_GET() == (char) 12);
			assert (pack.min_distance_GET() == (char) 6622);
			assert (pack.max_distance_GET() == (char) 3202);
			assert (Arrays.equals(pack.distances_GET(), new char[]{(char) 37085, (char) 54420, (char) 37322, (char) 30548, (char) 31399, (char) 10017, (char) 55286, (char) 18728, (char) 44820, (char) 32542, (char) 51258, (char) 22717, (char) 2866, (char) 63806, (char) 17008, (char) 24335, (char) 61045, (char) 49596, (char) 28687, (char) 1003, (char) 19416, (char) 23192, (char) 34622, (char) 50263, (char) 26885, (char) 20050, (char) 58726, (char) 64521, (char) 25907, (char) 2009, (char) 61792, (char) 65299, (char) 38211, (char) 48667, (char) 764, (char) 64999, (char) 7760, (char) 47745, (char) 15191, (char) 7339, (char) 48987, (char) 38764, (char) 51939, (char) 50675, (char) 41807, (char) 44684, (char) 18341, (char) 10605, (char) 65129, (char) 24301, (char) 22545, (char) 40297, (char) 31123, (char) 49557, (char) 28479, (char) 22578, (char) 23206, (char) 14216, (char) 40645, (char) 59333, (char) 57776, (char) 52411, (char) 29221, (char) 14251, (char) 28802, (char) 4208, (char) 40518, (char) 16415, (char) 56593, (char) 45963, (char) 43759, (char) 14079}));
			assert (pack.sensor_type_GET() == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_UNKNOWN);
			assert (pack.time_usec_GET() == 7237327356406448229L);
		});
		GroundControl.OBSTACLE_DISTANCE p330 = CommunicationChannel.new_OBSTACLE_DISTANCE();
		PH.setPack(p330);
		p330.distances_SET(new char[]{(char) 37085, (char) 54420, (char) 37322, (char) 30548, (char) 31399, (char) 10017, (char) 55286, (char) 18728, (char) 44820, (char) 32542, (char) 51258, (char) 22717, (char) 2866, (char) 63806, (char) 17008, (char) 24335, (char) 61045, (char) 49596, (char) 28687, (char) 1003, (char) 19416, (char) 23192, (char) 34622, (char) 50263, (char) 26885, (char) 20050, (char) 58726, (char) 64521, (char) 25907, (char) 2009, (char) 61792, (char) 65299, (char) 38211, (char) 48667, (char) 764, (char) 64999, (char) 7760, (char) 47745, (char) 15191, (char) 7339, (char) 48987, (char) 38764, (char) 51939, (char) 50675, (char) 41807, (char) 44684, (char) 18341, (char) 10605, (char) 65129, (char) 24301, (char) 22545, (char) 40297, (char) 31123, (char) 49557, (char) 28479, (char) 22578, (char) 23206, (char) 14216, (char) 40645, (char) 59333, (char) 57776, (char) 52411, (char) 29221, (char) 14251, (char) 28802, (char) 4208, (char) 40518, (char) 16415, (char) 56593, (char) 45963, (char) 43759, (char) 14079}, 0);
		p330.sensor_type_SET(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_UNKNOWN);
		p330.time_usec_SET(7237327356406448229L);
		p330.max_distance_SET((char) 3202);
		p330.increment_SET((char) 12);
		p330.min_distance_SET((char) 6622);
		CommunicationChannel.instance.send(p330);//put test pack to the  channel send buffer
		TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
	}

}