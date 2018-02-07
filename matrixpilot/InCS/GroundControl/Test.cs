
using System;
using System.Diagnostics;
using System.Linq;
using org.unirail.BlackBox;
using Inside = org.unirail.BlackBox.Host.Pack.Meta.Field.Bounds.Inside;
namespace org.noname
{
    public class Test : GroundControl
    {


        public class HEARTBEAT : GroundControl.HEARTBEAT
        {
            public uint custom_mode //A bitfield for use for autopilot-specific flags.
            {
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  0);}
            }

            public byte mavlink_version //MAVLink version, not writable by user, gets added by protocol because of magic data type: byte_mavlink_versio
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  4);}
            }

            public MAV_TYPE type //Type of the MAV (quadrotor, helicopter, etc., up to 15 types, defined in MAV_TYPE ENUM)
            {
                set {  BitUtils.set_bits((ulong)(- 0 +   value), 5, data, 40);}
            }

            public MAV_AUTOPILOT autopilot //Autopilot type / class. defined in MAV_AUTOPILOT ENUM
            {
                set {  BitUtils.set_bits((ulong)(- 0 +   value), 5, data, 45);}
            }

            public MAV_MODE_FLAG base_mode //System mode bitfield, see MAV_MODE_FLAG ENUM in mavlink/include/mavlink_types.h
            {
                set {  BitUtils.set_bits((ulong)(- 1 +   value), 8, data, 50);}
            }

            public MAV_STATE system_status //System status flag, see MAV_STATE ENUM
            {
                set {  BitUtils.set_bits((ulong)(- 0 +   value), 4, data, 58);}
            }
        }
        public class SYS_STATUS : GroundControl.SYS_STATUS
        {
            public ushort load //Maximum usage in percent of the mainloop time, (0%: 0, 100%: 1000) should be always below 1000
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public ushort voltage_battery //Battery voltage, in millivolts (1 = 1 millivolt)
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  2);}
            }

            /**
            *Communication drops in percent, (0%: 0, 100%: 10'000), (UART, I2C, SPI, CAN), dropped packets on all links
            *	(packets that were corrupted on reception on the MAV*/
            public ushort drop_rate_comm
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  4);}
            }

            /**
            *Communication errors (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted
            *	on reception on the MAV*/
            public ushort errors_comm
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  6);}
            }

            public ushort errors_count1 //Autopilot-specific errors
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  8);}
            }

            public ushort errors_count2 //Autopilot-specific errors
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  10);}
            }

            public ushort errors_count3 //Autopilot-specific errors
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  12);}
            }

            public ushort errors_count4 //Autopilot-specific errors
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  14);}
            }

            public short current_battery //Battery current, in 10*milliamperes (1 = 10 milliampere), -1: autopilot does not measure the curren
            {
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  16);}
            }

            public sbyte battery_remaining //Remaining battery energy: (0%: 0, 100%: 100), -1: autopilot estimate the remaining battery
            {
                set {  BitUtils.set_bytes((byte)(value), 1, data,  18);}
            }

            /**
            *Bitmask showing which onboard controllers and sensors are present. Value of 0: not present. Value of 1:
            *	present. Indices defined by ENUM MAV_SYS_STATUS_SENSO*/
            public MAV_SYS_STATUS_SENSOR onboard_control_sensors_present
            {
                set {  BitUtils.set_bits((ulong)(- 1 +   value), 26, data, 152);}
            }

            /**
            *Bitmask showing which onboard controllers and sensors are enabled:  Value of 0: not enabled. Value of
            *	1: enabled. Indices defined by ENUM MAV_SYS_STATUS_SENSO*/
            public MAV_SYS_STATUS_SENSOR onboard_control_sensors_enabled
            {
                set {  BitUtils.set_bits((ulong)(- 1 +   value), 26, data, 178);}
            }

            /**
            *Bitmask showing which onboard controllers and sensors are operational or have an error:  Value of 0: not
            *	enabled. Value of 1: enabled. Indices defined by ENUM MAV_SYS_STATUS_SENSO*/
            public MAV_SYS_STATUS_SENSOR onboard_control_sensors_health
            {
                set {  BitUtils.set_bits((ulong)(- 1 +   value), 26, data, 204);}
            }
        }
        public class SYSTEM_TIME : GroundControl.SYSTEM_TIME
        {
            public uint time_boot_ms //Timestamp of the component clock since boot time in milliseconds.
            {
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  0);}
            }

            public ulong time_unix_usec //Timestamp of the master clock in microseconds since UNIX epoch.
            {
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  4);}
            }
        }
        public class POSITION_TARGET_LOCAL_NED : GroundControl.POSITION_TARGET_LOCAL_NED
        {
            /**
            *Bitmask to indicate which dimensions should be ignored by the vehicle: a value of 0b0000000000000000 or
            *	0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. If bit 10 is set
            *	the floats afx afy afz should be interpreted as force instead of acceleration. Mapping: bit 1: x, bit
            *	2: y, bit 3: z, bit 4: vx, bit 5: vy, bit 6: vz, bit 7: ax, bit 8: ay, bit 9: az, bit 10: is force setpoint,
            *	bit 11: yaw, bit 12: yaw rat*/
            public ushort type_mask
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public uint time_boot_ms //Timestamp in milliseconds since system boot
            {
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  2);}
            }

            public float x //X Position in NED frame in meters
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 6);}
            }

            public float y //Y Position in NED frame in meters
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 10);}
            }

            public float z //Z Position in NED frame in meters (note, altitude is negative in NED)
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 14);}
            }

            public float vx //X velocity in NED frame in meter / s
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 18);}
            }

            public float vy //Y velocity in NED frame in meter / s
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 22);}
            }

            public float vz //Z velocity in NED frame in meter / s
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 26);}
            }

            public float afx //X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 30);}
            }

            public float afy //Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 34);}
            }

            public float afz //Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 38);}
            }

            public float yaw //yaw setpoint in rad
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 42);}
            }

            public float yaw_rate //yaw rate setpoint in rad/s
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 46);}
            }

            /**
            *Valid options are: MAV_FRAME_LOCAL_NED = 1, MAV_FRAME_LOCAL_OFFSET_NED = 7, MAV_FRAME_BODY_NED = 8, MAV_FRAME_BODY_OFFSET_NED
            *	=*/
            public MAV_FRAME coordinate_frame
            {
                set {  BitUtils.set_bits((ulong)(- 0 +   value), 4, data, 400);}
            }
        }
        public class PING : GroundControl.PING
        {
            public uint seq //PING sequence
            {
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  0);}
            }

            public ulong time_usec //Unix timestamp in microseconds or since system boot if smaller than MAVLink epoch (1.1.2009)
            {
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  4);}
            }

            /**
            *0: request ping from all receiving systems, if greater than 0: message is a ping response and number is
            *	the system id of the requesting syste*/
            public byte target_system
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  12);}
            }

            /**
            *0: request ping from all receiving components, if greater than 0: message is a ping response and number
            *	is the system id of the requesting syste*/
            public byte target_component
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  13);}
            }
        }
        public class CHANGE_OPERATOR_CONTROL : GroundControl.CHANGE_OPERATOR_CONTROL
        {
            public byte target_system //System the GCS requests control for
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  0);}
            }

            public byte control_request //0: request control of this MAV, 1: Release control of this MAV
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  1);}
            }

            /**
            *0: key as plaintext, 1-255: future, different hashing/encryption variants. The GCS should in general use
            *	the safest mode possible initially and then gradually move down the encryption level if it gets a NACK
            *	message indicating an encryption mismatch*/
            public byte version
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  2);}
            }
            /**
            *Password / Key, depending on version plaintext or encrypted. 25 or less characters, NULL terminated. The
            *	characters may involve A-Z, a-z, 0-9, and "!?,.-*/
            public void passkey_SET(string src, Inside ph)
            {passkey_SET(src.ToCharArray(), 0, src.Length, ph);}/**
*Password / Key, depending on version plaintext or encrypted. 25 or less characters, NULL terminated. The
*	characters may involve A-Z, a-z, 0-9, and "!?,.-*/
            public void passkey_SET(char[] src, int pos, int items, Inside ph)
            {
                if(ph.field_bit != 24 && insert_field(ph, 24, items) ||
                        ! try_visit_item(ph, 0)) insert_item(ph, 0, items);
                for(int BYTE =  ph.BYTE, src_max = pos + ph.items; pos < src_max; pos++, BYTE += 2)
                    BitUtils.set_bytes((ushort)(src[pos]), 2, data,  BYTE);
            }
        }
        public class CHANGE_OPERATOR_CONTROL_ACK : GroundControl.CHANGE_OPERATOR_CONTROL_ACK
        {
            public byte gcs_system_id //ID of the GCS this message
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  0);}
            }

            public byte control_request //0: request control of this MAV, 1: Release control of this MAV
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  1);}
            }

            /**
            *0: ACK, 1: NACK: Wrong passkey, 2: NACK: Unsupported passkey encryption method, 3: NACK: Already under
            *	contro*/
            public byte ack
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  2);}
            }
        }
        public class AUTH_KEY : GroundControl.AUTH_KEY
        {
            public void key_SET(string src, Inside ph)//key
            {key_SET(src.ToCharArray(), 0, src.Length, ph);} public void key_SET(char[] src, int pos, int items, Inside ph) //key
            {
                if(ph.field_bit != 0 && insert_field(ph, 0, items) ||
                        ! try_visit_item(ph, 0)) insert_item(ph, 0, items);
                for(int BYTE =  ph.BYTE, src_max = pos + ph.items; pos < src_max; pos++, BYTE += 2)
                    BitUtils.set_bytes((ushort)(src[pos]), 2, data,  BYTE);
            }
        }
        public class SET_MODE : GroundControl.SET_MODE
        {
            public uint custom_mode //The new autopilot-specific mode. This field can be ignored by an autopilot.
            {
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  0);}
            }

            public byte target_system //The system setting the mode
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  4);}
            }

            public MAV_MODE base_mode //The new base mode
            {
                set
                {
                    ulong id = id__O(value);
                    BitUtils.set_bits(id, 4, data, 40);
                }
            }
        }
        public class PARAM_REQUEST_READ : GroundControl.PARAM_REQUEST_READ
        {
            public byte target_system //System ID
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  0);}
            }

            public byte target_component //Component ID
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  1);}
            }

            public short param_index //Parameter index. Send -1 to use the param ID field as identifier (else the param id will be ignored
            {
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  2);}
            }
            /**
            *Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT
            *	null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes
            *	storage if the ID is stored as strin*/
            public void param_id_SET(string src, Inside ph)
            {param_id_SET(src.ToCharArray(), 0, src.Length, ph);}/**
*Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT
*	null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes
*	storage if the ID is stored as strin*/
            public void param_id_SET(char[] src, int pos, int items, Inside ph)
            {
                if(ph.field_bit != 32 && insert_field(ph, 32, items) ||
                        ! try_visit_item(ph, 0)) insert_item(ph, 0, items);
                for(int BYTE =  ph.BYTE, src_max = pos + ph.items; pos < src_max; pos++, BYTE += 2)
                    BitUtils.set_bytes((ushort)(src[pos]), 2, data,  BYTE);
            }
        }
        public class PARAM_REQUEST_LIST : GroundControl.PARAM_REQUEST_LIST
        {
            public byte target_system //System ID
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  0);}
            }

            public byte target_component //Component ID
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  1);}
            }
        }
        public class PARAM_VALUE : GroundControl.PARAM_VALUE
        {
            public ushort param_count //Total number of onboard parameters
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public ushort param_index //Index of this onboard parameter
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  2);}
            }

            public float param_value //Onboard parameter value
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 4);}
            }

            public MAV_PARAM_TYPE param_type //Onboard parameter type: see the MAV_PARAM_TYPE enum for supported data types.
            {
                set {  BitUtils.set_bits((ulong)(- 1 +   value), 4, data, 64);}
            }
            /**
            *Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT
            *	null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes
            *	storage if the ID is stored as strin*/
            public void param_id_SET(string src, Inside ph)
            {param_id_SET(src.ToCharArray(), 0, src.Length, ph);}/**
*Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT
*	null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes
*	storage if the ID is stored as strin*/
            public void param_id_SET(char[] src, int pos, int items, Inside ph)
            {
                if(ph.field_bit != 68 && insert_field(ph, 68, items) ||
                        ! try_visit_item(ph, 0)) insert_item(ph, 0, items);
                for(int BYTE =  ph.BYTE, src_max = pos + ph.items; pos < src_max; pos++, BYTE += 2)
                    BitUtils.set_bytes((ushort)(src[pos]), 2, data,  BYTE);
            }
        }
        public class PARAM_SET : GroundControl.PARAM_SET
        {
            public byte target_system //System ID
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  0);}
            }

            public byte target_component //Component ID
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  1);}
            }

            public float param_value //Onboard parameter value
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 2);}
            }

            public MAV_PARAM_TYPE param_type //Onboard parameter type: see the MAV_PARAM_TYPE enum for supported data types.
            {
                set {  BitUtils.set_bits((ulong)(- 1 +   value), 4, data, 48);}
            }
            /**
            *Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT
            *	null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes
            *	storage if the ID is stored as strin*/
            public void param_id_SET(string src, Inside ph)
            {param_id_SET(src.ToCharArray(), 0, src.Length, ph);}/**
*Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT
*	null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes
*	storage if the ID is stored as strin*/
            public void param_id_SET(char[] src, int pos, int items, Inside ph)
            {
                if(ph.field_bit != 52 && insert_field(ph, 52, items) ||
                        ! try_visit_item(ph, 0)) insert_item(ph, 0, items);
                for(int BYTE =  ph.BYTE, src_max = pos + ph.items; pos < src_max; pos++, BYTE += 2)
                    BitUtils.set_bytes((ushort)(src[pos]), 2, data,  BYTE);
            }
        }
        public class GPS_RAW_INT : GroundControl.GPS_RAW_INT
        {
            public ushort eph //GPS HDOP horizontal dilution of position (unitless). If unknown, set to: UINT16_MAX
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public ushort epv //GPS VDOP vertical dilution of position (unitless). If unknown, set to: UINT16_MAX
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  2);}
            }

            public ushort vel //GPS ground speed (m/s * 100). If unknown, set to: UINT16_MAX
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  4);}
            }

            /**
            *Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If
            *	unknown, set to: UINT16_MA*/
            public ushort cog
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  6);}
            }

            public ulong time_usec //Timestamp (microseconds since UNIX epoch or microseconds since system boot)
            {
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  8);}
            }

            public int lat //Latitude (WGS84, EGM96 ellipsoid), in degrees * 1E7
            {
                set {  BitUtils.set_bytes((uint)(value), 4, data,  16);}
            }

            public int lon //Longitude (WGS84, EGM96 ellipsoid), in degrees * 1E7
            {
                set {  BitUtils.set_bytes((uint)(value), 4, data,  20);}
            }

            /**
            *Altitude (AMSL, NOT WGS84), in meters * 1000 (positive for up). Note that virtually all GPS modules provide
            *	the AMSL altitude in addition to the WGS84 altitude*/
            public int alt
            {
                set {  BitUtils.set_bytes((uint)(value), 4, data,  24);}
            }

            public byte satellites_visible //Number of satellites visible. If unknown, set to 255
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  28);}
            }

            public GPS_FIX_TYPE fix_type //See the GPS_FIX_TYPE enum.
            {
                set {  BitUtils.set_bits((ulong)(- 0 +   value), 4, data, 232);}
            }
            public void alt_ellipsoid_SET(int src, Inside ph)//Altitude (above WGS84, EGM96 ellipsoid), in meters * 1000 (positive for up).
            {
                if(ph.field_bit != 236)insert_field(ph, 236, 0);
                BitUtils.set_bytes((uint)(src), 4, data,  ph.BYTE);
            } public void h_acc_SET(uint src, Inside ph) //Position uncertainty in meters * 1000 (positive for up).
            {
                if(ph.field_bit != 237)insert_field(ph, 237, 0);
                BitUtils.set_bytes((ulong)(src), 4, data,  ph.BYTE);
            } public void v_acc_SET(uint src, Inside ph) //Altitude uncertainty in meters * 1000 (positive for up).
            {
                if(ph.field_bit != 238)insert_field(ph, 238, 0);
                BitUtils.set_bytes((ulong)(src), 4, data,  ph.BYTE);
            } public void vel_acc_SET(uint src, Inside ph) //Speed uncertainty in meters * 1000 (positive for up).
            {
                if(ph.field_bit != 239)insert_field(ph, 239, 0);
                BitUtils.set_bytes((ulong)(src), 4, data,  ph.BYTE);
            } public void hdg_acc_SET(uint src, Inside ph) //Heading / track uncertainty in degrees * 1e5.
            {
                if(ph.field_bit != 240)insert_field(ph, 240, 0);
                BitUtils.set_bytes((ulong)(src), 4, data,  ph.BYTE);
            }
        }
        public class GPS_STATUS : GroundControl.GPS_STATUS
        {
            public byte satellites_visible //Number of satellites visible
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  0);}
            }

            public byte[] satellite_prn //Global satellite ID
            {
                set {satellite_prn_SET(value, 0)  ;}
            }
            public void satellite_prn_SET(byte[] src, int pos)  //Global satellite ID
            {
                for(int BYTE =  1, src_max = pos + 20; pos < src_max; pos++, BYTE += 1)
                    BitUtils.set_bytes((ulong)(src[pos]), 1, data,  BYTE);
            }
            public byte[] satellite_used //0: Satellite not used, 1: used for localization
            {
                set {satellite_used_SET(value, 0)  ;}
            }
            public void satellite_used_SET(byte[] src, int pos)  //0: Satellite not used, 1: used for localization
            {
                for(int BYTE =  21, src_max = pos + 20; pos < src_max; pos++, BYTE += 1)
                    BitUtils.set_bytes((ulong)(src[pos]), 1, data,  BYTE);
            }
            public byte[] satellite_elevation //Elevation (0: right on top of receiver, 90: on the horizon) of satellite
            {
                set {satellite_elevation_SET(value, 0)  ;}
            }
            public void satellite_elevation_SET(byte[] src, int pos)  //Elevation (0: right on top of receiver, 90: on the horizon) of satellite
            {
                for(int BYTE =  41, src_max = pos + 20; pos < src_max; pos++, BYTE += 1)
                    BitUtils.set_bytes((ulong)(src[pos]), 1, data,  BYTE);
            }
            public byte[] satellite_azimuth //Direction of satellite, 0: 0 deg, 255: 360 deg.
            {
                set {satellite_azimuth_SET(value, 0)  ;}
            }
            public void satellite_azimuth_SET(byte[] src, int pos)  //Direction of satellite, 0: 0 deg, 255: 360 deg.
            {
                for(int BYTE =  61, src_max = pos + 20; pos < src_max; pos++, BYTE += 1)
                    BitUtils.set_bytes((ulong)(src[pos]), 1, data,  BYTE);
            }
            public byte[] satellite_snr //Signal to noise ratio of satellite
            {
                set {satellite_snr_SET(value, 0)  ;}
            }
            public void satellite_snr_SET(byte[] src, int pos)  //Signal to noise ratio of satellite
            {
                for(int BYTE =  81, src_max = pos + 20; pos < src_max; pos++, BYTE += 1)
                    BitUtils.set_bytes((ulong)(src[pos]), 1, data,  BYTE);
            }
        }
        public class SCALED_IMU : GroundControl.SCALED_IMU
        {
            public uint time_boot_ms //Timestamp (milliseconds since system boot)
            {
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  0);}
            }

            public short xacc //X acceleration (mg)
            {
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  4);}
            }

            public short yacc //Y acceleration (mg)
            {
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  6);}
            }

            public short zacc //Z acceleration (mg)
            {
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  8);}
            }

            public short xgyro //Angular speed around X axis (millirad /sec)
            {
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  10);}
            }

            public short ygyro //Angular speed around Y axis (millirad /sec)
            {
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  12);}
            }

            public short zgyro //Angular speed around Z axis (millirad /sec)
            {
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  14);}
            }

            public short xmag //X Magnetic field (milli tesla)
            {
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  16);}
            }

            public short ymag //Y Magnetic field (milli tesla)
            {
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  18);}
            }

            public short zmag //Z Magnetic field (milli tesla)
            {
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  20);}
            }
        }
        public class RAW_IMU : GroundControl.RAW_IMU
        {
            public ulong time_usec //Timestamp (microseconds since UNIX epoch or microseconds since system boot)
            {
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  0);}
            }

            public short xacc //X acceleration (raw)
            {
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  8);}
            }

            public short yacc //Y acceleration (raw)
            {
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  10);}
            }

            public short zacc //Z acceleration (raw)
            {
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  12);}
            }

            public short xgyro //Angular speed around X axis (raw)
            {
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  14);}
            }

            public short ygyro //Angular speed around Y axis (raw)
            {
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  16);}
            }

            public short zgyro //Angular speed around Z axis (raw)
            {
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  18);}
            }

            public short xmag //X Magnetic field (raw)
            {
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  20);}
            }

            public short ymag //Y Magnetic field (raw)
            {
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  22);}
            }

            public short zmag //Z Magnetic field (raw)
            {
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  24);}
            }
        }
        public class RAW_PRESSURE : GroundControl.RAW_PRESSURE
        {
            public ulong time_usec //Timestamp (microseconds since UNIX epoch or microseconds since system boot)
            {
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  0);}
            }

            public short press_abs //Absolute pressure (raw)
            {
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  8);}
            }

            public short press_diff1 //Differential pressure 1 (raw, 0 if nonexistant)
            {
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  10);}
            }

            public short press_diff2 //Differential pressure 2 (raw, 0 if nonexistant)
            {
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  12);}
            }

            public short temperature //Raw Temperature measurement (raw)
            {
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  14);}
            }
        }
        public class SCALED_PRESSURE : GroundControl.SCALED_PRESSURE
        {
            public uint time_boot_ms //Timestamp (milliseconds since system boot)
            {
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  0);}
            }

            public float press_abs //Absolute pressure (hectopascal)
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 4);}
            }

            public float press_diff //Differential pressure 1 (hectopascal)
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 8);}
            }

            public short temperature //Temperature measurement (0.01 degrees celsius)
            {
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  12);}
            }
        }
        public class ATTITUDE : GroundControl.ATTITUDE
        {
            public uint time_boot_ms //Timestamp (milliseconds since system boot)
            {
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  0);}
            }

            public float roll //Roll angle (rad, -pi..+pi)
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 4);}
            }

            public float pitch //Pitch angle (rad, -pi..+pi)
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 8);}
            }

            public float yaw //Yaw angle (rad, -pi..+pi)
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 12);}
            }

            public float rollspeed //Roll angular speed (rad/s)
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 16);}
            }

            public float pitchspeed //Pitch angular speed (rad/s)
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 20);}
            }

            public float yawspeed //Yaw angular speed (rad/s)
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 24);}
            }
        }
        public class ATTITUDE_QUATERNION : GroundControl.ATTITUDE_QUATERNION
        {
            public uint time_boot_ms //Timestamp (milliseconds since system boot)
            {
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  0);}
            }

            public float q1 //Quaternion component 1, w (1 in null-rotation)
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 4);}
            }

            public float q2 //Quaternion component 2, x (0 in null-rotation)
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 8);}
            }

            public float q3 //Quaternion component 3, y (0 in null-rotation)
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 12);}
            }

            public float q4 //Quaternion component 4, z (0 in null-rotation)
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 16);}
            }

            public float rollspeed //Roll angular speed (rad/s)
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 20);}
            }

            public float pitchspeed //Pitch angular speed (rad/s)
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 24);}
            }

            public float yawspeed //Yaw angular speed (rad/s)
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 28);}
            }
        }
        public class LOCAL_POSITION_NED : GroundControl.LOCAL_POSITION_NED
        {
            public uint time_boot_ms //Timestamp (milliseconds since system boot)
            {
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  0);}
            }

            public float x //X Position
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 4);}
            }

            public float y //Y Position
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 8);}
            }

            public float z //Z Position
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 12);}
            }

            public float vx //X Speed
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 16);}
            }

            public float vy //Y Speed
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 20);}
            }

            public float vz //Z Speed
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 24);}
            }
        }
        public class GLOBAL_POSITION_INT : GroundControl.GLOBAL_POSITION_INT
        {
            public ushort hdg //Vehicle heading (yaw angle) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public uint time_boot_ms //Timestamp (milliseconds since system boot)
            {
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  2);}
            }

            public int lat //Latitude, expressed as degrees * 1E7
            {
                set {  BitUtils.set_bytes((uint)(value), 4, data,  6);}
            }

            public int lon //Longitude, expressed as degrees * 1E7
            {
                set {  BitUtils.set_bytes((uint)(value), 4, data,  10);}
            }

            /**
            *Altitude in meters, expressed as * 1000 (millimeters), AMSL (not WGS84 - note that virtually all GPS modules
            *	provide the AMSL as well*/
            public int alt
            {
                set {  BitUtils.set_bytes((uint)(value), 4, data,  14);}
            }

            public int relative_alt //Altitude above ground in meters, expressed as * 1000 (millimeters)
            {
                set {  BitUtils.set_bytes((uint)(value), 4, data,  18);}
            }

            public short vx //Ground X Speed (Latitude, positive north), expressed as m/s * 100
            {
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  22);}
            }

            public short vy //Ground Y Speed (Longitude, positive east), expressed as m/s * 100
            {
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  24);}
            }

            public short vz //Ground Z Speed (Altitude, positive down), expressed as m/s * 100
            {
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  26);}
            }
        }
        public class RC_CHANNELS_SCALED : GroundControl.RC_CHANNELS_SCALED
        {
            public uint time_boot_ms //Timestamp (milliseconds since system boot)
            {
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  0);}
            }

            /**
            *Servo output port (set of 8 outputs = 1 port). Most MAVs will just use one, but this allows for more than
            *	8 servos*/
            public byte port
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  4);}
            }

            public short chan1_scaled //RC channel 1 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
            {
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  5);}
            }

            public short chan2_scaled //RC channel 2 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
            {
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  7);}
            }

            public short chan3_scaled //RC channel 3 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
            {
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  9);}
            }

            public short chan4_scaled //RC channel 4 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
            {
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  11);}
            }

            public short chan5_scaled //RC channel 5 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
            {
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  13);}
            }

            public short chan6_scaled //RC channel 6 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
            {
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  15);}
            }

            public short chan7_scaled //RC channel 7 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
            {
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  17);}
            }

            public short chan8_scaled //RC channel 8 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
            {
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  19);}
            }

            public byte rssi //Receive signal strength indicator, 0: 0%, 100: 100%, 255: invalid/unknown.
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  21);}
            }
        }
        public class RC_CHANNELS_RAW : GroundControl.RC_CHANNELS_RAW
        {
            public ushort chan1_raw //RC channel 1 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public ushort chan2_raw //RC channel 2 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  2);}
            }

            public ushort chan3_raw //RC channel 3 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  4);}
            }

            public ushort chan4_raw //RC channel 4 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  6);}
            }

            public ushort chan5_raw //RC channel 5 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  8);}
            }

            public ushort chan6_raw //RC channel 6 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  10);}
            }

            public ushort chan7_raw //RC channel 7 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  12);}
            }

            public ushort chan8_raw //RC channel 8 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  14);}
            }

            public uint time_boot_ms //Timestamp (milliseconds since system boot)
            {
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  16);}
            }

            /**
            *Servo output port (set of 8 outputs = 1 port). Most MAVs will just use one, but this allows for more than
            *	8 servos*/
            public byte port
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  20);}
            }

            public byte rssi //Receive signal strength indicator, 0: 0%, 100: 100%, 255: invalid/unknown.
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  21);}
            }
        }
        public class SERVO_OUTPUT_RAW : GroundControl.SERVO_OUTPUT_RAW
        {
            public ushort servo1_raw //Servo output 1 value, in microseconds
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public ushort servo2_raw //Servo output 2 value, in microseconds
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  2);}
            }

            public ushort servo3_raw //Servo output 3 value, in microseconds
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  4);}
            }

            public ushort servo4_raw //Servo output 4 value, in microseconds
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  6);}
            }

            public ushort servo5_raw //Servo output 5 value, in microseconds
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  8);}
            }

            public ushort servo6_raw //Servo output 6 value, in microseconds
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  10);}
            }

            public ushort servo7_raw //Servo output 7 value, in microseconds
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  12);}
            }

            public ushort servo8_raw //Servo output 8 value, in microseconds
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  14);}
            }

            public uint time_usec //Timestamp (microseconds since system boot)
            {
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  16);}
            }

            /**
            *Servo output port (set of 8 outputs = 1 port). Most MAVs will just use one, but this allows to encode
            *	more than 8 servos*/
            public byte port
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  20);}
            }
            public void servo9_raw_SET(ushort src, Inside ph)//Servo output 9 value, in microseconds
            {
                if(ph.field_bit != 168)insert_field(ph, 168, 0);
                BitUtils.set_bytes((ulong)(src), 2, data,  ph.BYTE);
            } public void servo10_raw_SET(ushort src, Inside ph) //Servo output 10 value, in microseconds
            {
                if(ph.field_bit != 169)insert_field(ph, 169, 0);
                BitUtils.set_bytes((ulong)(src), 2, data,  ph.BYTE);
            } public void servo11_raw_SET(ushort src, Inside ph) //Servo output 11 value, in microseconds
            {
                if(ph.field_bit != 170)insert_field(ph, 170, 0);
                BitUtils.set_bytes((ulong)(src), 2, data,  ph.BYTE);
            } public void servo12_raw_SET(ushort src, Inside ph) //Servo output 12 value, in microseconds
            {
                if(ph.field_bit != 171)insert_field(ph, 171, 0);
                BitUtils.set_bytes((ulong)(src), 2, data,  ph.BYTE);
            } public void servo13_raw_SET(ushort src, Inside ph) //Servo output 13 value, in microseconds
            {
                if(ph.field_bit != 172)insert_field(ph, 172, 0);
                BitUtils.set_bytes((ulong)(src), 2, data,  ph.BYTE);
            } public void servo14_raw_SET(ushort src, Inside ph) //Servo output 14 value, in microseconds
            {
                if(ph.field_bit != 173)insert_field(ph, 173, 0);
                BitUtils.set_bytes((ulong)(src), 2, data,  ph.BYTE);
            } public void servo15_raw_SET(ushort src, Inside ph) //Servo output 15 value, in microseconds
            {
                if(ph.field_bit != 174)insert_field(ph, 174, 0);
                BitUtils.set_bytes((ulong)(src), 2, data,  ph.BYTE);
            } public void servo16_raw_SET(ushort src, Inside ph) //Servo output 16 value, in microseconds
            {
                if(ph.field_bit != 175)insert_field(ph, 175, 0);
                BitUtils.set_bytes((ulong)(src), 2, data,  ph.BYTE);
            }
        }
        public class MISSION_REQUEST_PARTIAL_LIST : GroundControl.MISSION_REQUEST_PARTIAL_LIST
        {
            public byte target_system //System ID
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  0);}
            }

            public byte target_component //Component ID
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  1);}
            }

            public short start_index //Start index, 0 by default
            {
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  2);}
            }

            public short end_index //End index, -1 by default (-1: send list to end). Else a valid index of the list
            {
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  4);}
            }

            public MAV_MISSION_TYPE mission_type //Mission type, see MAV_MISSION_TYPE
            {
                set
                {
                    ulong id = id__h(value);
                    BitUtils.set_bits(id, 3, data, 48);
                }
            }
        }
        public class MISSION_WRITE_PARTIAL_LIST : GroundControl.MISSION_WRITE_PARTIAL_LIST
        {
            public byte target_system //System ID
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  0);}
            }

            public byte target_component //Component ID
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  1);}
            }

            public short start_index //Start index, 0 by default and smaller / equal to the largest index of the current onboard list.
            {
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  2);}
            }

            public short end_index //End index, equal or greater than start index.
            {
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  4);}
            }

            public MAV_MISSION_TYPE mission_type //Mission type, see MAV_MISSION_TYPE
            {
                set
                {
                    ulong id = id__h(value);
                    BitUtils.set_bits(id, 3, data, 48);
                }
            }
        }
        public class MISSION_ITEM : GroundControl.MISSION_ITEM
        {
            public ushort seq //Sequence
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public byte target_system //System ID
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  2);}
            }

            public byte target_component //Component ID
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  3);}
            }

            public byte current //false:0, true:1
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  4);}
            }

            public byte autocontinue //autocontinue to next wp
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  5);}
            }

            public float param1 //PARAM1, see MAV_CMD enum
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 6);}
            }

            public float param2 //PARAM2, see MAV_CMD enum
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 10);}
            }

            public float param3 //PARAM3, see MAV_CMD enum
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 14);}
            }

            public float param4 //PARAM4, see MAV_CMD enum
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 18);}
            }

            public float x //PARAM5 / local: x position, global: latitude
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 22);}
            }

            public float y //PARAM6 / y position: global: longitude
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 26);}
            }

            public float z //PARAM7 / z position: global: altitude (relative or absolute, depending on frame.
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 30);}
            }

            public MAV_FRAME frame //The coordinate system of the waypoint. see MAV_FRAME in mavlink_types.h
            {
                set {  BitUtils.set_bits((ulong)(- 0 +   value), 4, data, 272);}
            }

            public MAV_CMD command //The scheduled action for the waypoint. see MAV_CMD in common.xml MAVLink specs
            {
                set
                {
                    ulong id = id__W(value);
                    BitUtils.set_bits(id, 8, data, 276);
                }
            }

            public MAV_MISSION_TYPE mission_type //Mission type, see MAV_MISSION_TYPE
            {
                set
                {
                    ulong id = id__h(value);
                    BitUtils.set_bits(id, 3, data, 284);
                }
            }
        }
        public class MISSION_REQUEST : GroundControl.MISSION_REQUEST
        {
            public ushort seq //Sequence
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public byte target_system //System ID
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  2);}
            }

            public byte target_component //Component ID
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  3);}
            }

            public MAV_MISSION_TYPE mission_type //Mission type, see MAV_MISSION_TYPE
            {
                set
                {
                    ulong id = id__h(value);
                    BitUtils.set_bits(id, 3, data, 32);
                }
            }
        }
        public class MISSION_SET_CURRENT : GroundControl.MISSION_SET_CURRENT
        {
            public ushort seq //Sequence
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public byte target_system //System ID
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  2);}
            }

            public byte target_component //Component ID
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  3);}
            }
        }
        public class MISSION_CURRENT : GroundControl.MISSION_CURRENT
        {
            public ushort seq //Sequence
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }
        }
        public class MISSION_REQUEST_LIST : GroundControl.MISSION_REQUEST_LIST
        {
            public byte target_system //System ID
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  0);}
            }

            public byte target_component //Component ID
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  1);}
            }

            public MAV_MISSION_TYPE mission_type //Mission type, see MAV_MISSION_TYPE
            {
                set
                {
                    ulong id = id__h(value);
                    BitUtils.set_bits(id, 3, data, 16);
                }
            }
        }
        public class MISSION_COUNT : GroundControl.MISSION_COUNT
        {
            public ushort count //Number of mission items in the sequence
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public byte target_system //System ID
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  2);}
            }

            public byte target_component //Component ID
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  3);}
            }

            public MAV_MISSION_TYPE mission_type //Mission type, see MAV_MISSION_TYPE
            {
                set
                {
                    ulong id = id__h(value);
                    BitUtils.set_bits(id, 3, data, 32);
                }
            }
        }
        public class MISSION_CLEAR_ALL : GroundControl.MISSION_CLEAR_ALL
        {
            public byte target_system //System ID
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  0);}
            }

            public byte target_component //Component ID
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  1);}
            }

            public MAV_MISSION_TYPE mission_type //Mission type, see MAV_MISSION_TYPE
            {
                set
                {
                    ulong id = id__h(value);
                    BitUtils.set_bits(id, 3, data, 16);
                }
            }
        }
        public class MISSION_ITEM_REACHED : GroundControl.MISSION_ITEM_REACHED
        {
            public ushort seq //Sequence
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }
        }
        public class MISSION_ACK : GroundControl.MISSION_ACK
        {
            public byte target_system //System ID
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  0);}
            }

            public byte target_component //Component ID
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  1);}
            }

            public MAV_MISSION_RESULT type //See MAV_MISSION_RESULT enum
            {
                set {  BitUtils.set_bits((ulong)(- 0 +   value), 4, data, 16);}
            }

            public MAV_MISSION_TYPE mission_type //Mission type, see MAV_MISSION_TYPE
            {
                set
                {
                    ulong id = id__h(value);
                    BitUtils.set_bits(id, 3, data, 20);
                }
            }
        }
        public class SET_GPS_GLOBAL_ORIGIN : GroundControl.SET_GPS_GLOBAL_ORIGIN
        {
            public byte target_system //System ID
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  0);}
            }

            public int latitude //Latitude (WGS84), in degrees * 1E7
            {
                set {  BitUtils.set_bytes((uint)(value), 4, data,  1);}
            }

            public int longitude //Longitude (WGS84, in degrees * 1E7
            {
                set {  BitUtils.set_bytes((uint)(value), 4, data,  5);}
            }

            public int altitude //Altitude (AMSL), in meters * 1000 (positive for up)
            {
                set {  BitUtils.set_bytes((uint)(value), 4, data,  9);}
            }
            public void time_usec_SET(ulong src, Inside ph)//Timestamp (microseconds since UNIX epoch or microseconds since system boot)
            {
                if(ph.field_bit != 104)insert_field(ph, 104, 0);
                BitUtils.set_bytes((ulong)(src), 8, data,  ph.BYTE);
            }
        }
        public class GPS_GLOBAL_ORIGIN : GroundControl.GPS_GLOBAL_ORIGIN
        {
            public int latitude //Latitude (WGS84), in degrees * 1E7
            {
                set {  BitUtils.set_bytes((uint)(value), 4, data,  0);}
            }

            public int longitude //Longitude (WGS84), in degrees * 1E7
            {
                set {  BitUtils.set_bytes((uint)(value), 4, data,  4);}
            }

            public int altitude //Altitude (AMSL), in meters * 1000 (positive for up)
            {
                set {  BitUtils.set_bytes((uint)(value), 4, data,  8);}
            }
            public void time_usec_SET(ulong src, Inside ph)//Timestamp (microseconds since UNIX epoch or microseconds since system boot)
            {
                if(ph.field_bit != 96)insert_field(ph, 96, 0);
                BitUtils.set_bytes((ulong)(src), 8, data,  ph.BYTE);
            }
        }
        public class PARAM_MAP_RC : GroundControl.PARAM_MAP_RC
        {
            public byte target_system //System ID
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  0);}
            }

            public byte target_component //Component ID
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  1);}
            }

            /**
            *Parameter index. Send -1 to use the param ID field as identifier (else the param id will be ignored),
            *	send -2 to disable any existing map for this rc_channel_index*/
            public short param_index
            {
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  2);}
            }

            /**
            *Index of parameter RC channel. Not equal to the RC channel id. Typically correpsonds to a potentiometer-knob
            *	on the RC*/
            public byte parameter_rc_channel_index
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  4);}
            }

            public float param_value0 //Initial parameter value
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 5);}
            }

            public float scale //Scale, maps the RC range [-1, 1] to a parameter value
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 9);}
            }

            /**
            *Minimum param value. The protocol does not define if this overwrites an onboard minimum value. (Depends
            *	on implementation*/
            public float param_value_min
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 13);}
            }

            /**
            *Maximum param value. The protocol does not define if this overwrites an onboard maximum value. (Depends
            *	on implementation*/
            public float param_value_max
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 17);}
            }
            /**
            *Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT
            *	null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes
            *	storage if the ID is stored as strin*/
            public void param_id_SET(string src, Inside ph)
            {param_id_SET(src.ToCharArray(), 0, src.Length, ph);}/**
*Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT
*	null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes
*	storage if the ID is stored as strin*/
            public void param_id_SET(char[] src, int pos, int items, Inside ph)
            {
                if(ph.field_bit != 168 && insert_field(ph, 168, items) ||
                        ! try_visit_item(ph, 0)) insert_item(ph, 0, items);
                for(int BYTE =  ph.BYTE, src_max = pos + ph.items; pos < src_max; pos++, BYTE += 2)
                    BitUtils.set_bytes((ushort)(src[pos]), 2, data,  BYTE);
            }
        }
        public class MISSION_REQUEST_INT : GroundControl.MISSION_REQUEST_INT
        {
            public ushort seq //Sequence
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public byte target_system //System ID
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  2);}
            }

            public byte target_component //Component ID
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  3);}
            }

            public MAV_MISSION_TYPE mission_type //Mission type, see MAV_MISSION_TYPE
            {
                set
                {
                    ulong id = id__h(value);
                    BitUtils.set_bits(id, 3, data, 32);
                }
            }
        }
        public class SAFETY_SET_ALLOWED_AREA : GroundControl.SAFETY_SET_ALLOWED_AREA
        {
            public byte target_system //System ID
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  0);}
            }

            public byte target_component //Component ID
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  1);}
            }

            public float p1x //x position 1 / Latitude 1
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 2);}
            }

            public float p1y //y position 1 / Longitude 1
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 6);}
            }

            public float p1z //z position 1 / Altitude 1
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 10);}
            }

            public float p2x //x position 2 / Latitude 2
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 14);}
            }

            public float p2y //y position 2 / Longitude 2
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 18);}
            }

            public float p2z //z position 2 / Altitude 2
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 22);}
            }

            /**
            *Coordinate frame, as defined by MAV_FRAME enum in mavlink_types.h. Can be either global, GPS, right-handed
            *	with Z axis up or local, right handed, Z axis down*/
            public MAV_FRAME frame
            {
                set {  BitUtils.set_bits((ulong)(- 0 +   value), 4, data, 208);}
            }
        }
        public class SAFETY_ALLOWED_AREA : GroundControl.SAFETY_ALLOWED_AREA
        {
            public float p1x //x position 1 / Latitude 1
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 0);}
            }

            public float p1y //y position 1 / Longitude 1
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 4);}
            }

            public float p1z //z position 1 / Altitude 1
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 8);}
            }

            public float p2x //x position 2 / Latitude 2
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 12);}
            }

            public float p2y //y position 2 / Longitude 2
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 16);}
            }

            public float p2z //z position 2 / Altitude 2
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 20);}
            }

            /**
            *Coordinate frame, as defined by MAV_FRAME enum in mavlink_types.h. Can be either global, GPS, right-handed
            *	with Z axis up or local, right handed, Z axis down*/
            public MAV_FRAME frame
            {
                set {  BitUtils.set_bits((ulong)(- 0 +   value), 4, data, 192);}
            }
        }
        public class ATTITUDE_QUATERNION_COV : GroundControl.ATTITUDE_QUATERNION_COV
        {
            public ulong time_usec //Timestamp (microseconds since system boot or since UNIX epoch)
            {
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  0);}
            }

            public float[] q //Quaternion components, w, x, y, z (1 0 0 0 is the null-rotation)
            {
                set {q_SET(value, 0)  ;}
            }
            public void q_SET(float[] src, int pos)  //Quaternion components, w, x, y, z (1 0 0 0 is the null-rotation)
            {
                for(int BYTE =  8, src_max = pos + 4; pos < src_max; pos++, BYTE += 4)
                    BitUtils.set_bytes(BitUtils.FloatToInt32Bits(src[pos]), 4, data, BYTE);
            }
            public float rollspeed //Roll angular speed (rad/s)
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 24);}
            }

            public float pitchspeed //Pitch angular speed (rad/s)
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 28);}
            }

            public float yawspeed //Yaw angular speed (rad/s)
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 32);}
            }

            public float[] covariance //Attitude covariance
            {
                set {covariance_SET(value, 0)  ;}
            }
            public void covariance_SET(float[] src, int pos)  //Attitude covariance
            {
                for(int BYTE =  36, src_max = pos + 9; pos < src_max; pos++, BYTE += 4)
                    BitUtils.set_bytes(BitUtils.FloatToInt32Bits(src[pos]), 4, data, BYTE);
            }
        }
        public class NAV_CONTROLLER_OUTPUT : GroundControl.NAV_CONTROLLER_OUTPUT
        {
            public ushort wp_dist //Distance to active waypoint in meters
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public float nav_roll //Current desired roll in degrees
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 2);}
            }

            public float nav_pitch //Current desired pitch in degrees
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 6);}
            }

            public short nav_bearing //Current desired heading in degrees
            {
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  10);}
            }

            public short target_bearing //Bearing to current waypoint/target in degrees
            {
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  12);}
            }

            public float alt_error //Current altitude error in meters
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 14);}
            }

            public float aspd_error //Current airspeed error in meters/second
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 18);}
            }

            public float xtrack_error //Current crosstrack error on x-y plane in meters
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 22);}
            }
        }
        public class GLOBAL_POSITION_INT_COV : GroundControl.GLOBAL_POSITION_INT_COV
        {
            public ulong time_usec //Timestamp (microseconds since system boot or since UNIX epoch)
            {
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  0);}
            }

            public int lat //Latitude, expressed as degrees * 1E7
            {
                set {  BitUtils.set_bytes((uint)(value), 4, data,  8);}
            }

            public int lon //Longitude, expressed as degrees * 1E7
            {
                set {  BitUtils.set_bytes((uint)(value), 4, data,  12);}
            }

            public int alt //Altitude in meters, expressed as * 1000 (millimeters), above MSL
            {
                set {  BitUtils.set_bytes((uint)(value), 4, data,  16);}
            }

            public int relative_alt //Altitude above ground in meters, expressed as * 1000 (millimeters)
            {
                set {  BitUtils.set_bytes((uint)(value), 4, data,  20);}
            }

            public float vx //Ground X Speed (Latitude), expressed as m/s
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 24);}
            }

            public float vy //Ground Y Speed (Longitude), expressed as m/s
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 28);}
            }

            public float vz //Ground Z Speed (Altitude), expressed as m/s
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 32);}
            }

            public float[] covariance //Covariance matrix (first six entries are the first ROW, next six entries are the second row, etc.)
            {
                set {covariance_SET(value, 0)  ;}
            }
            public void covariance_SET(float[] src, int pos)  //Covariance matrix (first six entries are the first ROW, next six entries are the second row, etc.)
            {
                for(int BYTE =  36, src_max = pos + 36; pos < src_max; pos++, BYTE += 4)
                    BitUtils.set_bytes(BitUtils.FloatToInt32Bits(src[pos]), 4, data, BYTE);
            }
            public MAV_ESTIMATOR_TYPE estimator_type //Class id of the estimator this estimate originated from.
            {
                set {  BitUtils.set_bits((ulong)(- 1 +   value), 3, data, 1440);}
            }
        }
        public class LOCAL_POSITION_NED_COV : GroundControl.LOCAL_POSITION_NED_COV
        {
            public ulong time_usec //Timestamp (microseconds since system boot or since UNIX epoch)
            {
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  0);}
            }

            public float x //X Position
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 8);}
            }

            public float y //Y Position
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 12);}
            }

            public float z //Z Position
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 16);}
            }

            public float vx //X Speed (m/s)
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 20);}
            }

            public float vy //Y Speed (m/s)
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 24);}
            }

            public float vz //Z Speed (m/s)
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 28);}
            }

            public float ax //X Acceleration (m/s^2)
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 32);}
            }

            public float ay //Y Acceleration (m/s^2)
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 36);}
            }

            public float az //Z Acceleration (m/s^2)
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 40);}
            }

            /**
            *Covariance matrix upper right triangular (first nine entries are the first ROW, next eight entries are
            *	the second row, etc.*/
            public float[] covariance
            {
                set {covariance_SET(value, 0)  ;}
            }
            /**
            *Covariance matrix upper right triangular (first nine entries are the first ROW, next eight entries are
            *	the second row, etc.*/
            public void covariance_SET(float[] src, int pos)
            {
                for(int BYTE =  44, src_max = pos + 45; pos < src_max; pos++, BYTE += 4)
                    BitUtils.set_bytes(BitUtils.FloatToInt32Bits(src[pos]), 4, data, BYTE);
            }
            public MAV_ESTIMATOR_TYPE estimator_type //Class id of the estimator this estimate originated from.
            {
                set {  BitUtils.set_bits((ulong)(- 1 +   value), 3, data, 1792);}
            }
        }
        public class RC_CHANNELS : GroundControl.RC_CHANNELS
        {
            public ushort chan1_raw //RC channel 1 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public ushort chan2_raw //RC channel 2 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  2);}
            }

            public ushort chan3_raw //RC channel 3 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  4);}
            }

            public ushort chan4_raw //RC channel 4 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  6);}
            }

            public ushort chan5_raw //RC channel 5 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  8);}
            }

            public ushort chan6_raw //RC channel 6 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  10);}
            }

            public ushort chan7_raw //RC channel 7 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  12);}
            }

            public ushort chan8_raw //RC channel 8 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  14);}
            }

            public ushort chan9_raw //RC channel 9 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  16);}
            }

            public ushort chan10_raw //RC channel 10 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  18);}
            }

            public ushort chan11_raw //RC channel 11 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  20);}
            }

            public ushort chan12_raw //RC channel 12 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  22);}
            }

            public ushort chan13_raw //RC channel 13 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  24);}
            }

            public ushort chan14_raw //RC channel 14 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  26);}
            }

            public ushort chan15_raw //RC channel 15 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  28);}
            }

            public ushort chan16_raw //RC channel 16 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  30);}
            }

            public ushort chan17_raw //RC channel 17 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  32);}
            }

            public ushort chan18_raw //RC channel 18 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  34);}
            }

            public uint time_boot_ms //Timestamp (milliseconds since system boot)
            {
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  36);}
            }

            /**
            *Total number of RC channels being received. This can be larger than 18, indicating that more channels
            *	are available but not given in this message. This value should be 0 when no RC channels are available*/
            public byte chancount
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  40);}
            }

            public byte rssi //Receive signal strength indicator, 0: 0%, 100: 100%, 255: invalid/unknown.
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  41);}
            }
        }
        public class REQUEST_DATA_STREAM : GroundControl.REQUEST_DATA_STREAM
        {
            public ushort req_message_rate //The requested message rate
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public byte target_system //The target requested to send the message stream.
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  2);}
            }

            public byte target_component //The target requested to send the message stream.
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  3);}
            }

            public byte req_stream_id //The ID of the requested data stream
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  4);}
            }

            public byte start_stop //1 to start sending, 0 to stop sending.
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  5);}
            }
        }
        public class DATA_STREAM : GroundControl.DATA_STREAM
        {
            public ushort message_rate //The message rate
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public byte stream_id //The ID of the requested data stream
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  2);}
            }

            public byte on_off //1 stream is enabled, 0 stream is stopped.
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  3);}
            }
        }
        public class MANUAL_CONTROL : GroundControl.MANUAL_CONTROL
        {
            /**
            *A bitfield corresponding to the joystick buttons' current state, 1 for pressed, 0 for released. The lowest
            *	bit corresponds to Button 1*/
            public ushort buttons
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public byte target //The system to be controlled.
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  2);}
            }

            /**
            *X-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid.
            *	Generally corresponds to forward(1000)-backward(-1000) movement on a joystick and the pitch of a vehicle*/
            public short x
            {
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  3);}
            }

            /**
            *Y-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid.
            *	Generally corresponds to left(-1000)-right(1000) movement on a joystick and the roll of a vehicle*/
            public short y
            {
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  5);}
            }

            /**
            *Z-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid.
            *	Generally corresponds to a separate slider movement with maximum being 1000 and minimum being -1000 on
            *	a joystick and the thrust of a vehicle. Positive values are positive thrust, negative values are negative
            *	thrust*/
            public short z
            {
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  7);}
            }

            /**
            *R-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid.
            *	Generally corresponds to a twisting of the joystick, with counter-clockwise being 1000 and clockwise
            *	being -1000, and the yaw of a vehicle*/
            public short r
            {
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  9);}
            }
        }
        public class RC_CHANNELS_OVERRIDE : GroundControl.RC_CHANNELS_OVERRIDE
        {
            public ushort chan1_raw //RC channel 1 value, in microseconds. A value of UINT16_MAX means to ignore this field.
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public ushort chan2_raw //RC channel 2 value, in microseconds. A value of UINT16_MAX means to ignore this field.
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  2);}
            }

            public ushort chan3_raw //RC channel 3 value, in microseconds. A value of UINT16_MAX means to ignore this field.
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  4);}
            }

            public ushort chan4_raw //RC channel 4 value, in microseconds. A value of UINT16_MAX means to ignore this field.
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  6);}
            }

            public ushort chan5_raw //RC channel 5 value, in microseconds. A value of UINT16_MAX means to ignore this field.
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  8);}
            }

            public ushort chan6_raw //RC channel 6 value, in microseconds. A value of UINT16_MAX means to ignore this field.
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  10);}
            }

            public ushort chan7_raw //RC channel 7 value, in microseconds. A value of UINT16_MAX means to ignore this field.
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  12);}
            }

            public ushort chan8_raw //RC channel 8 value, in microseconds. A value of UINT16_MAX means to ignore this field.
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  14);}
            }

            public byte target_system //System ID
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  16);}
            }

            public byte target_component //Component ID
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  17);}
            }
        }
        public class MISSION_ITEM_INT : GroundControl.MISSION_ITEM_INT
        {
            /**
            *Waypoint ID (sequence number). Starts at zero. Increases monotonically for each waypoint, no gaps in the
            *	sequence (0,1,2,3,4)*/
            public ushort seq
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public byte target_system //System ID
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  2);}
            }

            public byte target_component //Component ID
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  3);}
            }

            public byte current //false:0, true:1
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  4);}
            }

            public byte autocontinue //autocontinue to next wp
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  5);}
            }

            public float param1 //PARAM1, see MAV_CMD enum
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 6);}
            }

            public float param2 //PARAM2, see MAV_CMD enum
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 10);}
            }

            public float param3 //PARAM3, see MAV_CMD enum
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 14);}
            }

            public float param4 //PARAM4, see MAV_CMD enum
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 18);}
            }

            public int x //PARAM5 / local: x position in meters * 1e4, global: latitude in degrees * 10^7
            {
                set {  BitUtils.set_bytes((uint)(value), 4, data,  22);}
            }

            public int y //PARAM6 / y position: local: x position in meters * 1e4, global: longitude in degrees *10^7
            {
                set {  BitUtils.set_bytes((uint)(value), 4, data,  26);}
            }

            public float z //PARAM7 / z position: global: altitude in meters (relative or absolute, depending on frame.
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 30);}
            }

            public MAV_FRAME frame //The coordinate system of the waypoint. see MAV_FRAME in mavlink_types.h
            {
                set {  BitUtils.set_bits((ulong)(- 0 +   value), 4, data, 272);}
            }

            public MAV_CMD command //The scheduled action for the waypoint. see MAV_CMD in common.xml MAVLink specs
            {
                set
                {
                    ulong id = id__W(value);
                    BitUtils.set_bits(id, 8, data, 276);
                }
            }

            public MAV_MISSION_TYPE mission_type //Mission type, see MAV_MISSION_TYPE
            {
                set
                {
                    ulong id = id__h(value);
                    BitUtils.set_bits(id, 3, data, 284);
                }
            }
        }
        public class VFR_HUD : GroundControl.VFR_HUD
        {
            public ushort throttle //Current throttle setting in integer percent, 0 to 100
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public float airspeed //Current airspeed in m/s
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 2);}
            }

            public float groundspeed //Current ground speed in m/s
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 6);}
            }

            public short heading //Current heading in degrees, in compass units (0..360, 0=north)
            {
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  10);}
            }

            public float alt //Current altitude (MSL), in meters
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 12);}
            }

            public float climb //Current climb rate in meters/second
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 16);}
            }
        }
        public class COMMAND_INT : GroundControl.COMMAND_INT
        {
            public byte target_system //System ID
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  0);}
            }

            public byte target_component //Component ID
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  1);}
            }

            public byte current //false:0, true:1
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  2);}
            }

            public byte autocontinue //autocontinue to next wp
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  3);}
            }

            public float param1 //PARAM1, see MAV_CMD enum
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 4);}
            }

            public float param2 //PARAM2, see MAV_CMD enum
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 8);}
            }

            public float param3 //PARAM3, see MAV_CMD enum
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 12);}
            }

            public float param4 //PARAM4, see MAV_CMD enum
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 16);}
            }

            public int x //PARAM5 / local: x position in meters * 1e4, global: latitude in degrees * 10^7
            {
                set {  BitUtils.set_bytes((uint)(value), 4, data,  20);}
            }

            public int y //PARAM6 / local: y position in meters * 1e4, global: longitude in degrees * 10^7
            {
                set {  BitUtils.set_bytes((uint)(value), 4, data,  24);}
            }

            public float z //PARAM7 / z position: global: altitude in meters (relative or absolute, depending on frame.
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 28);}
            }

            public MAV_FRAME frame //The coordinate system of the COMMAND. see MAV_FRAME in mavlink_types.h
            {
                set {  BitUtils.set_bits((ulong)(- 0 +   value), 4, data, 256);}
            }

            public MAV_CMD command //The scheduled action for the mission item. see MAV_CMD in common.xml MAVLink specs
            {
                set
                {
                    ulong id = id__W(value);
                    BitUtils.set_bits(id, 8, data, 260);
                }
            }
        }
        public class COMMAND_LONG : GroundControl.COMMAND_LONG
        {
            public byte target_system //System which should execute the command
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  0);}
            }

            public byte target_component //Component which should execute the command, 0 for all components
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  1);}
            }

            public byte confirmation //0: First transmission of this command. 1-255: Confirmation transmissions (e.g. for kill command)
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  2);}
            }

            public float param1 //Parameter 1, as defined by MAV_CMD enum.
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 3);}
            }

            public float param2 //Parameter 2, as defined by MAV_CMD enum.
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 7);}
            }

            public float param3 //Parameter 3, as defined by MAV_CMD enum.
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 11);}
            }

            public float param4 //Parameter 4, as defined by MAV_CMD enum.
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 15);}
            }

            public float param5 //Parameter 5, as defined by MAV_CMD enum.
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 19);}
            }

            public float param6 //Parameter 6, as defined by MAV_CMD enum.
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 23);}
            }

            public float param7 //Parameter 7, as defined by MAV_CMD enum.
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 27);}
            }

            public MAV_CMD command //Command ID, as defined by MAV_CMD enum.
            {
                set
                {
                    ulong id = id__W(value);
                    BitUtils.set_bits(id, 8, data, 248);
                }
            }
        }
        public class COMMAND_ACK : GroundControl.COMMAND_ACK
        {
            public MAV_CMD command //Command ID, as defined by MAV_CMD enum.
            {
                set
                {
                    ulong id = id__W(value);
                    BitUtils.set_bits(id, 8, data, 0);
                }
            }

            public MAV_RESULT result //See MAV_RESULT enum
            {
                set {  BitUtils.set_bits((ulong)(- 0 +   value), 3, data, 8);}
            }
            /**
            *WIP: Also used as result_param1, it can be set with a enum containing the errors reasons of why the command
            *	was denied or the progress percentage or 255 if unknown the progress when result is MAV_RESULT_IN_PROGRESS*/
            public void progress_SET(byte src, Inside ph)
            {
                if(ph.field_bit != 11)insert_field(ph, 11, 0);
                BitUtils.set_bytes((ulong)(src), 1, data,  ph.BYTE);
            }/**
*WIP: Additional parameter of the result, example: which parameter of MAV_CMD_NAV_WAYPOINT caused it to
*	be denied*/
            public void result_param2_SET(int src, Inside ph)
            {
                if(ph.field_bit != 12)insert_field(ph, 12, 0);
                BitUtils.set_bytes((uint)(src), 4, data,  ph.BYTE);
            } public void target_system_SET(byte src, Inside ph) //WIP: System which requested the command to be executed
            {
                if(ph.field_bit != 13)insert_field(ph, 13, 0);
                BitUtils.set_bytes((ulong)(src), 1, data,  ph.BYTE);
            } public void target_component_SET(byte src, Inside ph) //WIP: Component which requested the command to be executed
            {
                if(ph.field_bit != 14)insert_field(ph, 14, 0);
                BitUtils.set_bytes((ulong)(src), 1, data,  ph.BYTE);
            }
        }
        public class MANUAL_SETPOINT : GroundControl.MANUAL_SETPOINT
        {
            public uint time_boot_ms //Timestamp in milliseconds since system boot
            {
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  0);}
            }

            public float roll //Desired roll rate in radians per second
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 4);}
            }

            public float pitch //Desired pitch rate in radians per second
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 8);}
            }

            public float yaw //Desired yaw rate in radians per second
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 12);}
            }

            public float thrust //Collective thrust, normalized to 0 .. 1
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 16);}
            }

            public byte mode_switch //Flight mode switch position, 0.. 255
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  20);}
            }

            public byte manual_override_switch //Override mode switch position, 0.. 255
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  21);}
            }
        }
        public class SET_ATTITUDE_TARGET : GroundControl.SET_ATTITUDE_TARGET
        {
            public uint time_boot_ms //Timestamp in milliseconds since system boot
            {
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  0);}
            }

            public byte target_system //System ID
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  4);}
            }

            public byte target_component //Component ID
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  5);}
            }

            /**
            *Mappings: If any of these bits are set, the corresponding input should be ignored: bit 1: body roll rate,
            *	bit 2: body pitch rate, bit 3: body yaw rate. bit 4-bit 6: reserved, bit 7: throttle, bit 8: attitud*/
            public byte type_mask
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  6);}
            }

            public float[] q //Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
            {
                set {q_SET(value, 0)  ;}
            }
            public void q_SET(float[] src, int pos)  //Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
            {
                for(int BYTE =  7, src_max = pos + 4; pos < src_max; pos++, BYTE += 4)
                    BitUtils.set_bytes(BitUtils.FloatToInt32Bits(src[pos]), 4, data, BYTE);
            }
            public float body_roll_rate //Body roll rate in radians per second
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 23);}
            }

            public float body_pitch_rate //Body roll rate in radians per second
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 27);}
            }

            public float body_yaw_rate //Body roll rate in radians per second
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 31);}
            }

            public float thrust //Collective thrust, normalized to 0 .. 1 (-1 .. 1 for vehicles capable of reverse trust)
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 35);}
            }
        }
        public class ATTITUDE_TARGET : GroundControl.ATTITUDE_TARGET
        {
            public uint time_boot_ms //Timestamp in milliseconds since system boot
            {
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  0);}
            }

            /**
            *Mappings: If any of these bits are set, the corresponding input should be ignored: bit 1: body roll rate,
            *	bit 2: body pitch rate, bit 3: body yaw rate. bit 4-bit 7: reserved, bit 8: attitud*/
            public byte type_mask
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  4);}
            }

            public float[] q //Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
            {
                set {q_SET(value, 0)  ;}
            }
            public void q_SET(float[] src, int pos)  //Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
            {
                for(int BYTE =  5, src_max = pos + 4; pos < src_max; pos++, BYTE += 4)
                    BitUtils.set_bytes(BitUtils.FloatToInt32Bits(src[pos]), 4, data, BYTE);
            }
            public float body_roll_rate //Body roll rate in radians per second
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 21);}
            }

            public float body_pitch_rate //Body pitch rate in radians per second
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 25);}
            }

            public float body_yaw_rate //Body yaw rate in radians per second
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 29);}
            }

            public float thrust //Collective thrust, normalized to 0 .. 1 (-1 .. 1 for vehicles capable of reverse trust)
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 33);}
            }
        }
        public class SET_POSITION_TARGET_LOCAL_NED : GroundControl.SET_POSITION_TARGET_LOCAL_NED
        {
            /**
            *Bitmask to indicate which dimensions should be ignored by the vehicle: a value of 0b0000000000000000 or
            *	0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. If bit 10 is set
            *	the floats afx afy afz should be interpreted as force instead of acceleration. Mapping: bit 1: x, bit
            *	2: y, bit 3: z, bit 4: vx, bit 5: vy, bit 6: vz, bit 7: ax, bit 8: ay, bit 9: az, bit 10: is force setpoint,
            *	bit 11: yaw, bit 12: yaw rat*/
            public ushort type_mask
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public uint time_boot_ms //Timestamp in milliseconds since system boot
            {
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  2);}
            }

            public byte target_system //System ID
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  6);}
            }

            public byte target_component //Component ID
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  7);}
            }

            public float x //X Position in NED frame in meters
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 8);}
            }

            public float y //Y Position in NED frame in meters
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 12);}
            }

            public float z //Z Position in NED frame in meters (note, altitude is negative in NED)
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 16);}
            }

            public float vx //X velocity in NED frame in meter / s
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 20);}
            }

            public float vy //Y velocity in NED frame in meter / s
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 24);}
            }

            public float vz //Z velocity in NED frame in meter / s
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 28);}
            }

            public float afx //X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 32);}
            }

            public float afy //Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 36);}
            }

            public float afz //Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 40);}
            }

            public float yaw //yaw setpoint in rad
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 44);}
            }

            public float yaw_rate //yaw rate setpoint in rad/s
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 48);}
            }

            /**
            *Valid options are: MAV_FRAME_LOCAL_NED = 1, MAV_FRAME_LOCAL_OFFSET_NED = 7, MAV_FRAME_BODY_NED = 8, MAV_FRAME_BODY_OFFSET_NED
            *	=*/
            public MAV_FRAME coordinate_frame
            {
                set {  BitUtils.set_bits((ulong)(- 0 +   value), 4, data, 416);}
            }
        }
        public class SET_POSITION_TARGET_GLOBAL_INT : GroundControl.SET_POSITION_TARGET_GLOBAL_INT
        {
            /**
            *Bitmask to indicate which dimensions should be ignored by the vehicle: a value of 0b0000000000000000 or
            *	0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. If bit 10 is set
            *	the floats afx afy afz should be interpreted as force instead of acceleration. Mapping: bit 1: x, bit
            *	2: y, bit 3: z, bit 4: vx, bit 5: vy, bit 6: vz, bit 7: ax, bit 8: ay, bit 9: az, bit 10: is force setpoint,
            *	bit 11: yaw, bit 12: yaw rat*/
            public ushort type_mask
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            /**
            *Timestamp in milliseconds since system boot. The rationale for the timestamp in the setpoint is to allow
            *	the system to compensate for the transport delay of the setpoint. This allows the system to compensate
            *	processing latency*/
            public uint time_boot_ms
            {
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  2);}
            }

            public byte target_system //System ID
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  6);}
            }

            public byte target_component //Component ID
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  7);}
            }

            public int lat_int //X Position in WGS84 frame in 1e7 * meters
            {
                set {  BitUtils.set_bytes((uint)(value), 4, data,  8);}
            }

            public int lon_int //Y Position in WGS84 frame in 1e7 * meters
            {
                set {  BitUtils.set_bytes((uint)(value), 4, data,  12);}
            }

            public float alt //Altitude in meters in AMSL altitude, not WGS84 if absolute or relative, above terrain if GLOBAL_TERRAIN_ALT_IN
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 16);}
            }

            public float vx //X velocity in NED frame in meter / s
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 20);}
            }

            public float vy //Y velocity in NED frame in meter / s
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 24);}
            }

            public float vz //Z velocity in NED frame in meter / s
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 28);}
            }

            public float afx //X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 32);}
            }

            public float afy //Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 36);}
            }

            public float afz //Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 40);}
            }

            public float yaw //yaw setpoint in rad
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 44);}
            }

            public float yaw_rate //yaw rate setpoint in rad/s
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 48);}
            }

            /**
            *Valid options are: MAV_FRAME_GLOBAL_INT = 5, MAV_FRAME_GLOBAL_RELATIVE_ALT_INT = 6, MAV_FRAME_GLOBAL_TERRAIN_ALT_INT
            *	= 1*/
            public MAV_FRAME coordinate_frame
            {
                set {  BitUtils.set_bits((ulong)(- 0 +   value), 4, data, 416);}
            }
        }
        new class FLEXIFUNCTION_SET : GroundControl.FLEXIFUNCTION_SET
        {
            public byte target_system //System ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  0, 1));}
            }

            public byte target_component //Component ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  1, 1));}
            }
        }
        new class FLEXIFUNCTION_READ_REQ : GroundControl.FLEXIFUNCTION_READ_REQ
        {
            public byte target_system //System ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  0, 1));}
            }

            public byte target_component //Component ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  1, 1));}
            }

            public short read_req_type //Type of flexifunction data requested
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  2, 2));}
            }

            public short data_index //index into data where needed
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  4, 2));}
            }
        }
        new class FLEXIFUNCTION_BUFFER_FUNCTION : GroundControl.FLEXIFUNCTION_BUFFER_FUNCTION
        {
            public ushort func_index //Function index
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
            }

            public ushort func_count //Total count of functions
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  2, 2));}
            }

            public ushort data_address //Address in the flexifunction data, Set to 0xFFFF to use address in target memory
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  4, 2));}
            }

            public ushort data_size //Size of the
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  6, 2));}
            }

            public byte target_system //System ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  8, 1));}
            }

            public byte target_component //Component ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  9, 1));}
            }

            public sbyte[] data_ //Settings data
            {
                get {return data__GET(new sbyte[48], 0);}
            }
            public sbyte[]data__GET(sbyte[] dst_ch, int pos)  //Settings data
            {
                for(int BYTE = 10, dst_max = pos + 48; pos < dst_max ; pos++,  BYTE += 1)
                    dst_ch[pos] = (sbyte)((sbyte) BitUtils.get_bytes(data,  BYTE, 1));
                return dst_ch;
            }
        }
        new class FLEXIFUNCTION_BUFFER_FUNCTION_ACK : GroundControl.FLEXIFUNCTION_BUFFER_FUNCTION_ACK
        {
            public ushort func_index //Function index
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
            }

            public ushort result //result of acknowledge, 0=fail, 1=good
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  2, 2));}
            }

            public byte target_system //System ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  4, 1));}
            }

            public byte target_component //Component ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  5, 1));}
            }
        }
        new class FLEXIFUNCTION_DIRECTORY : GroundControl.FLEXIFUNCTION_DIRECTORY
        {
            public byte target_system //System ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  0, 1));}
            }

            public byte target_component //Component ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  1, 1));}
            }

            public byte directory_type //0=inputs, 1=outputs
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  2, 1));}
            }

            public byte start_index //index of first directory entry to write
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  3, 1));}
            }

            public byte count //count of directory entries to write
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  4, 1));}
            }

            public sbyte[] directory_data //Settings data
            {
                get {return directory_data_GET(new sbyte[48], 0);}
            }
            public sbyte[]directory_data_GET(sbyte[] dst_ch, int pos)  //Settings data
            {
                for(int BYTE = 5, dst_max = pos + 48; pos < dst_max ; pos++,  BYTE += 1)
                    dst_ch[pos] = (sbyte)((sbyte) BitUtils.get_bytes(data,  BYTE, 1));
                return dst_ch;
            }
        }
        new class FLEXIFUNCTION_DIRECTORY_ACK : GroundControl.FLEXIFUNCTION_DIRECTORY_ACK
        {
            public ushort result //result of acknowledge, 0=fail, 1=good
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
            }

            public byte target_system //System ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  2, 1));}
            }

            public byte target_component //Component ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  3, 1));}
            }

            public byte directory_type //0=inputs, 1=outputs
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  4, 1));}
            }

            public byte start_index //index of first directory entry to write
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  5, 1));}
            }

            public byte count //count of directory entries to write
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  6, 1));}
            }
        }
        new class FLEXIFUNCTION_COMMAND : GroundControl.FLEXIFUNCTION_COMMAND
        {
            public byte target_system //System ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  0, 1));}
            }

            public byte target_component //Component ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  1, 1));}
            }

            public byte command_type //Flexifunction command type
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  2, 1));}
            }
        }
        new class FLEXIFUNCTION_COMMAND_ACK : GroundControl.FLEXIFUNCTION_COMMAND_ACK
        {
            public ushort command_type //Command acknowledged
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
            }

            public ushort result //result of acknowledge
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  2, 2));}
            }
        }
        new class SERIAL_UDB_EXTRA_F2_A : GroundControl.SERIAL_UDB_EXTRA_F2_A
        {
            public ushort sue_waypoint_index //Serial UDB Extra Waypoint Index
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
            }

            public ushort sue_cog //Serial UDB Extra GPS Course Over Ground
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  2, 2));}
            }

            public ushort sue_cpu_load //Serial UDB Extra CPU Load
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  4, 2));}
            }

            public ushort sue_air_speed_3DIMU //Serial UDB Extra 3D IMU Air Speed
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  6, 2));}
            }

            public uint sue_time //Serial UDB Extra Time
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  8, 4));}
            }

            public byte sue_status //Serial UDB Extra Status
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  12, 1));}
            }

            public int sue_latitude //Serial UDB Extra Latitude
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  13, 4));}
            }

            public int sue_longitude //Serial UDB Extra Longitude
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  17, 4));}
            }

            public int sue_altitude //Serial UDB Extra Altitude
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  21, 4));}
            }

            public short sue_rmat0 //Serial UDB Extra Rmat 0
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  25, 2));}
            }

            public short sue_rmat1 //Serial UDB Extra Rmat 1
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  27, 2));}
            }

            public short sue_rmat2 //Serial UDB Extra Rmat 2
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  29, 2));}
            }

            public short sue_rmat3 //Serial UDB Extra Rmat 3
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  31, 2));}
            }

            public short sue_rmat4 //Serial UDB Extra Rmat 4
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  33, 2));}
            }

            public short sue_rmat5 //Serial UDB Extra Rmat 5
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  35, 2));}
            }

            public short sue_rmat6 //Serial UDB Extra Rmat 6
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  37, 2));}
            }

            public short sue_rmat7 //Serial UDB Extra Rmat 7
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  39, 2));}
            }

            public short sue_rmat8 //Serial UDB Extra Rmat 8
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  41, 2));}
            }

            public short sue_sog //Serial UDB Extra Speed Over Ground
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  43, 2));}
            }

            public short sue_estimated_wind_0 //Serial UDB Extra Estimated Wind 0
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  45, 2));}
            }

            public short sue_estimated_wind_1 //Serial UDB Extra Estimated Wind 1
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  47, 2));}
            }

            public short sue_estimated_wind_2 //Serial UDB Extra Estimated Wind 2
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  49, 2));}
            }

            public short sue_magFieldEarth0 //Serial UDB Extra Magnetic Field Earth 0
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  51, 2));}
            }

            public short sue_magFieldEarth1 //Serial UDB Extra Magnetic Field Earth 1
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  53, 2));}
            }

            public short sue_magFieldEarth2 //Serial UDB Extra Magnetic Field Earth 2
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  55, 2));}
            }

            public short sue_svs //Serial UDB Extra Number of Sattelites in View
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  57, 2));}
            }

            public short sue_hdop //Serial UDB Extra GPS Horizontal Dilution of Precision
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  59, 2));}
            }
        }
        new class SERIAL_UDB_EXTRA_F2_B : GroundControl.SERIAL_UDB_EXTRA_F2_B
        {
            public uint sue_time //Serial UDB Extra Time
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  0, 4));}
            }

            public uint sue_flags //Serial UDB Extra Status Flags
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  4, 4));}
            }

            public short sue_pwm_input_1 //Serial UDB Extra PWM Input Channel 1
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  8, 2));}
            }

            public short sue_pwm_input_2 //Serial UDB Extra PWM Input Channel 2
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  10, 2));}
            }

            public short sue_pwm_input_3 //Serial UDB Extra PWM Input Channel 3
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  12, 2));}
            }

            public short sue_pwm_input_4 //Serial UDB Extra PWM Input Channel 4
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  14, 2));}
            }

            public short sue_pwm_input_5 //Serial UDB Extra PWM Input Channel 5
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  16, 2));}
            }

            public short sue_pwm_input_6 //Serial UDB Extra PWM Input Channel 6
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  18, 2));}
            }

            public short sue_pwm_input_7 //Serial UDB Extra PWM Input Channel 7
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  20, 2));}
            }

            public short sue_pwm_input_8 //Serial UDB Extra PWM Input Channel 8
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  22, 2));}
            }

            public short sue_pwm_input_9 //Serial UDB Extra PWM Input Channel 9
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  24, 2));}
            }

            public short sue_pwm_input_10 //Serial UDB Extra PWM Input Channel 10
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  26, 2));}
            }

            public short sue_pwm_input_11 //Serial UDB Extra PWM Input Channel 11
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  28, 2));}
            }

            public short sue_pwm_input_12 //Serial UDB Extra PWM Input Channel 12
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  30, 2));}
            }

            public short sue_pwm_output_1 //Serial UDB Extra PWM Output Channel 1
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  32, 2));}
            }

            public short sue_pwm_output_2 //Serial UDB Extra PWM Output Channel 2
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  34, 2));}
            }

            public short sue_pwm_output_3 //Serial UDB Extra PWM Output Channel 3
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  36, 2));}
            }

            public short sue_pwm_output_4 //Serial UDB Extra PWM Output Channel 4
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  38, 2));}
            }

            public short sue_pwm_output_5 //Serial UDB Extra PWM Output Channel 5
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  40, 2));}
            }

            public short sue_pwm_output_6 //Serial UDB Extra PWM Output Channel 6
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  42, 2));}
            }

            public short sue_pwm_output_7 //Serial UDB Extra PWM Output Channel 7
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  44, 2));}
            }

            public short sue_pwm_output_8 //Serial UDB Extra PWM Output Channel 8
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  46, 2));}
            }

            public short sue_pwm_output_9 //Serial UDB Extra PWM Output Channel 9
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  48, 2));}
            }

            public short sue_pwm_output_10 //Serial UDB Extra PWM Output Channel 10
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  50, 2));}
            }

            public short sue_pwm_output_11 //Serial UDB Extra PWM Output Channel 11
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  52, 2));}
            }

            public short sue_pwm_output_12 //Serial UDB Extra PWM Output Channel 12
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  54, 2));}
            }

            public short sue_imu_location_x //Serial UDB Extra IMU Location X
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  56, 2));}
            }

            public short sue_imu_location_y //Serial UDB Extra IMU Location Y
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  58, 2));}
            }

            public short sue_imu_location_z //Serial UDB Extra IMU Location Z
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  60, 2));}
            }

            public short sue_location_error_earth_x //Serial UDB Location Error Earth X
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  62, 2));}
            }

            public short sue_location_error_earth_y //Serial UDB Location Error Earth Y
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  64, 2));}
            }

            public short sue_location_error_earth_z //Serial UDB Location Error Earth Z
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  66, 2));}
            }

            public short sue_osc_fails //Serial UDB Extra Oscillator Failure Count
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  68, 2));}
            }

            public short sue_imu_velocity_x //Serial UDB Extra IMU Velocity X
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  70, 2));}
            }

            public short sue_imu_velocity_y //Serial UDB Extra IMU Velocity Y
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  72, 2));}
            }

            public short sue_imu_velocity_z //Serial UDB Extra IMU Velocity Z
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  74, 2));}
            }

            public short sue_waypoint_goal_x //Serial UDB Extra Current Waypoint Goal X
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  76, 2));}
            }

            public short sue_waypoint_goal_y //Serial UDB Extra Current Waypoint Goal Y
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  78, 2));}
            }

            public short sue_waypoint_goal_z //Serial UDB Extra Current Waypoint Goal Z
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  80, 2));}
            }

            public short sue_aero_x //Aeroforce in UDB X Axis
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  82, 2));}
            }

            public short sue_aero_y //Aeroforce in UDB Y Axis
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  84, 2));}
            }

            public short sue_aero_z //Aeroforce in UDB Z axis
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  86, 2));}
            }

            public short sue_barom_temp //SUE barometer temperature
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  88, 2));}
            }

            public int sue_barom_press //SUE barometer pressure
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  90, 4));}
            }

            public int sue_barom_alt //SUE barometer altitude
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  94, 4));}
            }

            public short sue_bat_volt //SUE battery voltage
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  98, 2));}
            }

            public short sue_bat_amp //SUE battery current
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  100, 2));}
            }

            public short sue_bat_amp_hours //SUE battery milli amp hours used
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  102, 2));}
            }

            public short sue_desired_height //Sue autopilot desired height
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  104, 2));}
            }

            public short sue_memory_stack_free //Serial UDB Extra Stack Memory Free
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  106, 2));}
            }
        }
        new class SERIAL_UDB_EXTRA_F4 : GroundControl.SERIAL_UDB_EXTRA_F4
        {
            public byte sue_ROLL_STABILIZATION_AILERONS //Serial UDB Extra Roll Stabilization with Ailerons Enabled
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  0, 1));}
            }

            public byte sue_ROLL_STABILIZATION_RUDDER //Serial UDB Extra Roll Stabilization with Rudder Enabled
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  1, 1));}
            }

            public byte sue_PITCH_STABILIZATION //Serial UDB Extra Pitch Stabilization Enabled
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  2, 1));}
            }

            public byte sue_YAW_STABILIZATION_RUDDER //Serial UDB Extra Yaw Stabilization using Rudder Enabled
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  3, 1));}
            }

            public byte sue_YAW_STABILIZATION_AILERON //Serial UDB Extra Yaw Stabilization using Ailerons Enabled
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  4, 1));}
            }

            public byte sue_AILERON_NAVIGATION //Serial UDB Extra Navigation with Ailerons Enabled
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  5, 1));}
            }

            public byte sue_RUDDER_NAVIGATION //Serial UDB Extra Navigation with Rudder Enabled
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  6, 1));}
            }

            public byte sue_ALTITUDEHOLD_STABILIZED //Serial UDB Extra Type of Alitude Hold when in Stabilized Mode
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  7, 1));}
            }

            public byte sue_ALTITUDEHOLD_WAYPOINT //Serial UDB Extra Type of Alitude Hold when in Waypoint Mode
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  8, 1));}
            }

            public byte sue_RACING_MODE //Serial UDB Extra Firmware racing mode enabled
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  9, 1));}
            }
        }
        new class SERIAL_UDB_EXTRA_F5 : GroundControl.SERIAL_UDB_EXTRA_F5
        {
            public float sue_YAWKP_AILERON //Serial UDB YAWKP_AILERON Gain for Proporional control of navigation
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  0, 4)));}
            }

            public float sue_YAWKD_AILERON //Serial UDB YAWKD_AILERON Gain for Rate control of navigation
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  4, 4)));}
            }

            public float sue_ROLLKP //Serial UDB Extra ROLLKP Gain for Proportional control of roll stabilization
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  8, 4)));}
            }

            public float sue_ROLLKD //Serial UDB Extra ROLLKD Gain for Rate control of roll stabilization
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  12, 4)));}
            }
        }
        new class SERIAL_UDB_EXTRA_F6 : GroundControl.SERIAL_UDB_EXTRA_F6
        {
            public float sue_PITCHGAIN //Serial UDB Extra PITCHGAIN Proportional Control
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  0, 4)));}
            }

            public float sue_PITCHKD //Serial UDB Extra Pitch Rate Control
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  4, 4)));}
            }

            public float sue_RUDDER_ELEV_MIX //Serial UDB Extra Rudder to Elevator Mix
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  8, 4)));}
            }

            public float sue_ROLL_ELEV_MIX //Serial UDB Extra Roll to Elevator Mix
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  12, 4)));}
            }

            public float sue_ELEVATOR_BOOST //Gain For Boosting Manual Elevator control When Plane Stabilized
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  16, 4)));}
            }
        }
        new class SERIAL_UDB_EXTRA_F7 : GroundControl.SERIAL_UDB_EXTRA_F7
        {
            public float sue_YAWKP_RUDDER //Serial UDB YAWKP_RUDDER Gain for Proporional control of navigation
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  0, 4)));}
            }

            public float sue_YAWKD_RUDDER //Serial UDB YAWKD_RUDDER Gain for Rate control of navigation
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  4, 4)));}
            }

            public float sue_ROLLKP_RUDDER //Serial UDB Extra ROLLKP_RUDDER Gain for Proportional control of roll stabilization
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  8, 4)));}
            }

            public float sue_ROLLKD_RUDDER //Serial UDB Extra ROLLKD_RUDDER Gain for Rate control of roll stabilization
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  12, 4)));}
            }

            public float sue_RUDDER_BOOST //SERIAL UDB EXTRA Rudder Boost Gain to Manual Control when stabilized
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  16, 4)));}
            }

            public float sue_RTL_PITCH_DOWN //Serial UDB Extra Return To Landing - Angle to Pitch Plane Down
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  20, 4)));}
            }
        }
        new class SERIAL_UDB_EXTRA_F8 : GroundControl.SERIAL_UDB_EXTRA_F8
        {
            public float sue_HEIGHT_TARGET_MAX //Serial UDB Extra HEIGHT_TARGET_MAX
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  0, 4)));}
            }

            public float sue_HEIGHT_TARGET_MIN //Serial UDB Extra HEIGHT_TARGET_MIN
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  4, 4)));}
            }

            public float sue_ALT_HOLD_THROTTLE_MIN //Serial UDB Extra ALT_HOLD_THROTTLE_MIN
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  8, 4)));}
            }

            public float sue_ALT_HOLD_THROTTLE_MAX //Serial UDB Extra ALT_HOLD_THROTTLE_MAX
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  12, 4)));}
            }

            public float sue_ALT_HOLD_PITCH_MIN //Serial UDB Extra ALT_HOLD_PITCH_MIN
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  16, 4)));}
            }

            public float sue_ALT_HOLD_PITCH_MAX //Serial UDB Extra ALT_HOLD_PITCH_MAX
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  20, 4)));}
            }

            public float sue_ALT_HOLD_PITCH_HIGH //Serial UDB Extra ALT_HOLD_PITCH_HIGH
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  24, 4)));}
            }
        }
        new class SERIAL_UDB_EXTRA_F13 : GroundControl.SERIAL_UDB_EXTRA_F13
        {
            public short sue_week_no //Serial UDB Extra GPS Week Number
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  0, 2));}
            }

            public int sue_lat_origin //Serial UDB Extra MP Origin Latitude
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  2, 4));}
            }

            public int sue_lon_origin //Serial UDB Extra MP Origin Longitude
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  6, 4));}
            }

            public int sue_alt_origin //Serial UDB Extra MP Origin Altitude Above Sea Level
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  10, 4));}
            }
        }
        new class SERIAL_UDB_EXTRA_F14 : GroundControl.SERIAL_UDB_EXTRA_F14
        {
            public uint sue_TRAP_SOURCE //Serial UDB Extra Type Program Address of Last Trap
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  0, 4));}
            }

            public byte sue_WIND_ESTIMATION //Serial UDB Extra Wind Estimation Enabled
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  4, 1));}
            }

            public byte sue_GPS_TYPE //Serial UDB Extra Type of GPS Unit
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  5, 1));}
            }

            public byte sue_DR //Serial UDB Extra Dead Reckoning Enabled
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  6, 1));}
            }

            public byte sue_BOARD_TYPE //Serial UDB Extra Type of UDB Hardware
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  7, 1));}
            }

            public byte sue_AIRFRAME //Serial UDB Extra Type of Airframe
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  8, 1));}
            }

            public short sue_RCON //Serial UDB Extra Reboot Register of DSPIC
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  9, 2));}
            }

            public short sue_TRAP_FLAGS //Serial UDB Extra  Last dspic Trap Flags
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  11, 2));}
            }

            public short sue_osc_fail_count //Serial UDB Extra Number of Ocillator Failures
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  13, 2));}
            }

            public byte sue_CLOCK_CONFIG //Serial UDB Extra UDB Internal Clock Configuration
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  15, 1));}
            }

            public byte sue_FLIGHT_PLAN_TYPE //Serial UDB Extra Type of Flight Plan
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  16, 1));}
            }
        }
        new class SERIAL_UDB_EXTRA_F15 : GroundControl.SERIAL_UDB_EXTRA_F15
        {
            public byte[] sue_ID_VEHICLE_MODEL_NAME //Serial UDB Extra Model Name Of Vehicle
            {
                get {return sue_ID_VEHICLE_MODEL_NAME_GET(new byte[40], 0);}
            }
            public byte[]sue_ID_VEHICLE_MODEL_NAME_GET(byte[] dst_ch, int pos)  //Serial UDB Extra Model Name Of Vehicle
            {
                for(int BYTE = 0, dst_max = pos + 40; pos < dst_max ; pos++,  BYTE += 1)
                    dst_ch[pos] = (byte)((byte) BitUtils.get_bytes(data,  BYTE, 1));
                return dst_ch;
            }
            public byte[] sue_ID_VEHICLE_REGISTRATION //Serial UDB Extra Registraton Number of Vehicle
            {
                get {return sue_ID_VEHICLE_REGISTRATION_GET(new byte[20], 0);}
            }
            public byte[]sue_ID_VEHICLE_REGISTRATION_GET(byte[] dst_ch, int pos)  //Serial UDB Extra Registraton Number of Vehicle
            {
                for(int BYTE = 40, dst_max = pos + 20; pos < dst_max ; pos++,  BYTE += 1)
                    dst_ch[pos] = (byte)((byte) BitUtils.get_bytes(data,  BYTE, 1));
                return dst_ch;
            }
        }
        new class SERIAL_UDB_EXTRA_F16 : GroundControl.SERIAL_UDB_EXTRA_F16
        {
            public byte[] sue_ID_LEAD_PILOT //Serial UDB Extra Name of Expected Lead Pilot
            {
                get {return sue_ID_LEAD_PILOT_GET(new byte[40], 0);}
            }
            public byte[]sue_ID_LEAD_PILOT_GET(byte[] dst_ch, int pos)  //Serial UDB Extra Name of Expected Lead Pilot
            {
                for(int BYTE = 0, dst_max = pos + 40; pos < dst_max ; pos++,  BYTE += 1)
                    dst_ch[pos] = (byte)((byte) BitUtils.get_bytes(data,  BYTE, 1));
                return dst_ch;
            }
            public byte[] sue_ID_DIY_DRONES_URL //Serial UDB Extra URL of Lead Pilot or Team
            {
                get {return sue_ID_DIY_DRONES_URL_GET(new byte[70], 0);}
            }
            public byte[]sue_ID_DIY_DRONES_URL_GET(byte[] dst_ch, int pos)  //Serial UDB Extra URL of Lead Pilot or Team
            {
                for(int BYTE = 40, dst_max = pos + 70; pos < dst_max ; pos++,  BYTE += 1)
                    dst_ch[pos] = (byte)((byte) BitUtils.get_bytes(data,  BYTE, 1));
                return dst_ch;
            }
        }
        new class ALTITUDES : GroundControl.ALTITUDES
        {
            public uint time_boot_ms //Timestamp (milliseconds since system boot)
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  0, 4));}
            }

            public int alt_gps //GPS altitude in meters, expressed as * 1000 (millimeters), above MSL
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  4, 4));}
            }

            public int alt_imu //IMU altitude above ground in meters, expressed as * 1000 (millimeters)
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  8, 4));}
            }

            public int alt_barometric //barometeric altitude above ground in meters, expressed as * 1000 (millimeters)
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  12, 4));}
            }

            public int alt_optical_flow //Optical flow altitude above ground in meters, expressed as * 1000 (millimeters)
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  16, 4));}
            }

            public int alt_range_finder //Rangefinder Altitude above ground in meters, expressed as * 1000 (millimeters)
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  20, 4));}
            }

            public int alt_extra //Extra altitude above ground in meters, expressed as * 1000 (millimeters)
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  24, 4));}
            }
        }
        new class AIRSPEEDS : GroundControl.AIRSPEEDS
        {
            public uint time_boot_ms //Timestamp (milliseconds since system boot)
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  0, 4));}
            }

            public short airspeed_imu //Airspeed estimate from IMU, cm/s
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  4, 2));}
            }

            public short airspeed_pitot //Pitot measured forward airpseed, cm/s
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  6, 2));}
            }

            public short airspeed_hot_wire //Hot wire anenometer measured airspeed, cm/s
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  8, 2));}
            }

            public short airspeed_ultrasonic //Ultrasonic measured airspeed, cm/s
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  10, 2));}
            }

            public short aoa //Angle of attack sensor, degrees * 10
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  12, 2));}
            }

            public short aoy //Yaw angle sensor, degrees * 10
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  14, 2));}
            }
        }
        new class SERIAL_UDB_EXTRA_F17 : GroundControl.SERIAL_UDB_EXTRA_F17
        {
            public float sue_feed_forward //SUE Feed Forward Gain
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  0, 4)));}
            }

            public float sue_turn_rate_nav //SUE Max Turn Rate when Navigating
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  4, 4)));}
            }

            public float sue_turn_rate_fbw //SUE Max Turn Rate in Fly By Wire Mode
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  8, 4)));}
            }
        }
        new class SERIAL_UDB_EXTRA_F18 : GroundControl.SERIAL_UDB_EXTRA_F18
        {
            public float angle_of_attack_normal //SUE Angle of Attack Normal
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  0, 4)));}
            }

            public float angle_of_attack_inverted //SUE Angle of Attack Inverted
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  4, 4)));}
            }

            public float elevator_trim_normal //SUE Elevator Trim Normal
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  8, 4)));}
            }

            public float elevator_trim_inverted //SUE Elevator Trim Inverted
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  12, 4)));}
            }

            public float reference_speed //SUE reference_speed
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  16, 4)));}
            }
        }
        new class SERIAL_UDB_EXTRA_F19 : GroundControl.SERIAL_UDB_EXTRA_F19
        {
            public byte sue_aileron_output_channel //SUE aileron output channel
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  0, 1));}
            }

            public byte sue_aileron_reversed //SUE aileron reversed
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  1, 1));}
            }

            public byte sue_elevator_output_channel //SUE elevator output channel
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  2, 1));}
            }

            public byte sue_elevator_reversed //SUE elevator reversed
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  3, 1));}
            }

            public byte sue_throttle_output_channel //SUE throttle output channel
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  4, 1));}
            }

            public byte sue_throttle_reversed //SUE throttle reversed
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  5, 1));}
            }

            public byte sue_rudder_output_channel //SUE rudder output channel
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  6, 1));}
            }

            public byte sue_rudder_reversed //SUE rudder reversed
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  7, 1));}
            }
        }
        new class SERIAL_UDB_EXTRA_F20 : GroundControl.SERIAL_UDB_EXTRA_F20
        {
            public byte sue_number_of_inputs //SUE Number of Input Channels
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  0, 1));}
            }

            public short sue_trim_value_input_1 //SUE UDB PWM Trim Value on Input 1
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  1, 2));}
            }

            public short sue_trim_value_input_2 //SUE UDB PWM Trim Value on Input 2
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  3, 2));}
            }

            public short sue_trim_value_input_3 //SUE UDB PWM Trim Value on Input 3
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  5, 2));}
            }

            public short sue_trim_value_input_4 //SUE UDB PWM Trim Value on Input 4
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  7, 2));}
            }

            public short sue_trim_value_input_5 //SUE UDB PWM Trim Value on Input 5
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  9, 2));}
            }

            public short sue_trim_value_input_6 //SUE UDB PWM Trim Value on Input 6
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  11, 2));}
            }

            public short sue_trim_value_input_7 //SUE UDB PWM Trim Value on Input 7
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  13, 2));}
            }

            public short sue_trim_value_input_8 //SUE UDB PWM Trim Value on Input 8
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  15, 2));}
            }

            public short sue_trim_value_input_9 //SUE UDB PWM Trim Value on Input 9
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  17, 2));}
            }

            public short sue_trim_value_input_10 //SUE UDB PWM Trim Value on Input 10
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  19, 2));}
            }

            public short sue_trim_value_input_11 //SUE UDB PWM Trim Value on Input 11
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  21, 2));}
            }

            public short sue_trim_value_input_12 //SUE UDB PWM Trim Value on Input 12
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  23, 2));}
            }
        }
        new class SERIAL_UDB_EXTRA_F21 : GroundControl.SERIAL_UDB_EXTRA_F21
        {
            public short sue_accel_x_offset //SUE X accelerometer offset
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  0, 2));}
            }

            public short sue_accel_y_offset //SUE Y accelerometer offset
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  2, 2));}
            }

            public short sue_accel_z_offset //SUE Z accelerometer offset
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  4, 2));}
            }

            public short sue_gyro_x_offset //SUE X gyro offset
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  6, 2));}
            }

            public short sue_gyro_y_offset //SUE Y gyro offset
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  8, 2));}
            }

            public short sue_gyro_z_offset //SUE Z gyro offset
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  10, 2));}
            }
        }
        new class SERIAL_UDB_EXTRA_F22 : GroundControl.SERIAL_UDB_EXTRA_F22
        {
            public short sue_accel_x_at_calibration //SUE X accelerometer at calibration time
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  0, 2));}
            }

            public short sue_accel_y_at_calibration //SUE Y accelerometer at calibration time
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  2, 2));}
            }

            public short sue_accel_z_at_calibration //SUE Z accelerometer at calibration time
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  4, 2));}
            }

            public short sue_gyro_x_at_calibration //SUE X gyro at calibration time
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  6, 2));}
            }

            public short sue_gyro_y_at_calibration //SUE Y gyro at calibration time
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  8, 2));}
            }

            public short sue_gyro_z_at_calibration //SUE Z gyro at calibration time
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  10, 2));}
            }
        }
        new class MESSAGE_INTERVAL : GroundControl.MESSAGE_INTERVAL
        {
            public ushort message_id //The ID of the requested MAVLink message. v1.0 is limited to 254 messages.
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
            }

            public int interval_us //0 indicates the interval at which it is sent.
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  2, 4));}
            }
        }
        new class EXTENDED_SYS_STATE : GroundControl.EXTENDED_SYS_STATE
        {
            public MAV_VTOL_STATE vtol_state //The VTOL state if applicable. Is set to MAV_VTOL_STATE_UNDEFINED if UAV is not in VTOL configuration
            {
                get {  return (MAV_VTOL_STATE)(0 +  BitUtils.get_bits(data, 0, 3));}
            }

            public MAV_LANDED_STATE landed_state //The landed state. Is set to MAV_LANDED_STATE_UNDEFINED if landed state is unknown.
            {
                get {  return (MAV_LANDED_STATE)(0 +  BitUtils.get_bits(data, 3, 3));}
            }
        }
        new class ADSB_VEHICLE : GroundControl.ADSB_VEHICLE
        {
            public ushort heading //Course over ground in centidegrees
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
            }

            public ushort hor_velocity //The horizontal velocity in centimeters/second
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  2, 2));}
            }

            public ushort squawk //Squawk code
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  4, 2));}
            }

            public uint ICAO_address //ICAO address
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  6, 4));}
            }

            public int lat //Latitude, expressed as degrees * 1E7
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  10, 4));}
            }

            public int lon //Longitude, expressed as degrees * 1E7
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  14, 4));}
            }

            public int altitude //Altitude(ASL) in millimeters
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  18, 4));}
            }

            public short ver_velocity //The vertical velocity in centimeters/second, positive is up
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  22, 2));}
            }

            public byte tslc //Time since last communication in seconds
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  24, 1));}
            }

            public ADSB_ALTITUDE_TYPE altitude_type //Type from ADSB_ALTITUDE_TYPE enum
            {
                get {  return (ADSB_ALTITUDE_TYPE)(0 +  BitUtils.get_bits(data, 200, 1));}
            }

            public ADSB_EMITTER_TYPE emitter_type //Type from ADSB_EMITTER_TYPE enum
            {
                get {  return (ADSB_EMITTER_TYPE)(0 +  BitUtils.get_bits(data, 201, 5));}
            }

            public ADSB_FLAGS flags //Flags to indicate various statuses including valid data fields
            {
                get {  return (ADSB_FLAGS)(1 +  BitUtils.get_bits(data, 206, 7));}
            }
            public string callsign_TRY(Inside ph)//The callsign, 8+null
            {
                if(ph.field_bit !=  213 && !try_visit_field(ph, 213)  ||  !try_visit_item(ph, 0)) return null;
                return new string(callsign_GET(ph, new char[ph.items], 0));
            }
            public char[]callsign_GET(Inside ph, char[] dst_ch, int pos) //The callsign, 8+null
            {
                for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (char)((char) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }
            public int callsign_LEN(Inside ph)
            {
                return (ph.field_bit !=  213 && !try_visit_field(ph, 213)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
            }
        }
        new class COLLISION : GroundControl.COLLISION
        {
            public uint id //Unique identifier, domain based on src field
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  0, 4));}
            }

            public float time_to_minimum_delta //Estimated time until collision occurs (seconds)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  4, 4)));}
            }

            public float altitude_minimum_delta //Closest vertical distance in meters between vehicle and object
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  8, 4)));}
            }

            public float horizontal_minimum_delta //Closest horizontal distance in meteres between vehicle and object
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  12, 4)));}
            }

            public MAV_COLLISION_SRC src_ //Collision data source
            {
                get {  return (MAV_COLLISION_SRC)(0 +  BitUtils.get_bits(data, 128, 1));}
            }

            public MAV_COLLISION_ACTION action //Action that is being taken to avoid this collision
            {
                get {  return (MAV_COLLISION_ACTION)(0 +  BitUtils.get_bits(data, 129, 3));}
            }

            public MAV_COLLISION_THREAT_LEVEL threat_level //How concerned the aircraft is about this collision
            {
                get {  return (MAV_COLLISION_THREAT_LEVEL)(0 +  BitUtils.get_bits(data, 132, 2));}
            }
        }
        new class V2_EXTENSION : GroundControl.V2_EXTENSION
        {
            /**
            *A code that identifies the software component that understands this message (analogous to usb device classes
            *	or mime type strings).  If this code is less than 32768, it is considered a 'registered' protocol extension
            *	and the corresponding entry should be added to https:github.com/mavlink/mavlink/extension-message-ids.xml.
            *	 Software creators can register blocks of message IDs as needed (useful for GCS specific metadata, etc...).
            *	Message_types greater than 32767 are considered local experiments and should not be checked in to any
            *	widely distributed codebase*/
            public ushort message_type
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
            }

            public byte target_network //Network ID (0 for broadcast)
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  2, 1));}
            }

            public byte target_system //System ID (0 for broadcast)
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  3, 1));}
            }

            public byte target_component //Component ID (0 for broadcast)
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  4, 1));}
            }

            /**
            *Variable length payload. The length is defined by the remaining message length when subtracting the header
            *	and other fields.  The entire content of this block is opaque unless you understand any the encoding
            *	message_type.  The particular encoding used can be extension specific and might not always be documented
            *	as part of the mavlink specification*/
            public byte[] payload
            {
                get {return payload_GET(new byte[249], 0);}
            }
            /**
            *Variable length payload. The length is defined by the remaining message length when subtracting the header
            *	and other fields.  The entire content of this block is opaque unless you understand any the encoding
            *	message_type.  The particular encoding used can be extension specific and might not always be documented
            *	as part of the mavlink specification*/
            public byte[]payload_GET(byte[] dst_ch, int pos)
            {
                for(int BYTE = 5, dst_max = pos + 249; pos < dst_max ; pos++,  BYTE += 1)
                    dst_ch[pos] = (byte)((byte) BitUtils.get_bytes(data,  BYTE, 1));
                return dst_ch;
            }
        }
        new class MEMORY_VECT : GroundControl.MEMORY_VECT
        {
            public ushort address //Starting address of the debug variables
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
            }

            public byte ver //Version code of the type variable. 0=unknown, type ignored and assumed short. 1=as below
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  2, 1));}
            }

            public byte type //Type code of the memory variables. for ver = 1: 0=16 x short, 1=16 x ushort, 2=16 x Q15, 3=16 x 1Q1
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  3, 1));}
            }

            public sbyte[] value //Memory contents at specified address
            {
                get {return value_GET(new sbyte[32], 0);}
            }
            public sbyte[]value_GET(sbyte[] dst_ch, int pos)  //Memory contents at specified address
            {
                for(int BYTE = 4, dst_max = pos + 32; pos < dst_max ; pos++,  BYTE += 1)
                    dst_ch[pos] = (sbyte)((sbyte) BitUtils.get_bytes(data,  BYTE, 1));
                return dst_ch;
            }
        }
        new class DEBUG_VECT : GroundControl.DEBUG_VECT
        {
            public ulong time_usec //Timestamp
            {
                get {  return (BitUtils.get_bytes(data,  0, 8));}
            }

            public float x //x
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  8, 4)));}
            }

            public float y //y
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  12, 4)));}
            }

            public float z //z
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  16, 4)));}
            }
            public string name_TRY(Inside ph)//Name
            {
                if(ph.field_bit !=  160 && !try_visit_field(ph, 160)  ||  !try_visit_item(ph, 0)) return null;
                return new string(name_GET(ph, new char[ph.items], 0));
            }
            public char[]name_GET(Inside ph, char[] dst_ch, int pos) //Name
            {
                for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (char)((char) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }
            public int name_LEN(Inside ph)
            {
                return (ph.field_bit !=  160 && !try_visit_field(ph, 160)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
            }
        }
        new class NAMED_VALUE_FLOAT : GroundControl.NAMED_VALUE_FLOAT
        {
            public uint time_boot_ms //Timestamp (milliseconds since system boot)
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  0, 4));}
            }

            public float value //Floating point value
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  4, 4)));}
            }
            public string name_TRY(Inside ph)//Name of the debug variable
            {
                if(ph.field_bit !=  64 && !try_visit_field(ph, 64)  ||  !try_visit_item(ph, 0)) return null;
                return new string(name_GET(ph, new char[ph.items], 0));
            }
            public char[]name_GET(Inside ph, char[] dst_ch, int pos) //Name of the debug variable
            {
                for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (char)((char) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }
            public int name_LEN(Inside ph)
            {
                return (ph.field_bit !=  64 && !try_visit_field(ph, 64)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
            }
        }
        new class NAMED_VALUE_INT : GroundControl.NAMED_VALUE_INT
        {
            public uint time_boot_ms //Timestamp (milliseconds since system boot)
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  0, 4));}
            }

            public int value //Signed integer value
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  4, 4));}
            }
            public string name_TRY(Inside ph)//Name of the debug variable
            {
                if(ph.field_bit !=  64 && !try_visit_field(ph, 64)  ||  !try_visit_item(ph, 0)) return null;
                return new string(name_GET(ph, new char[ph.items], 0));
            }
            public char[]name_GET(Inside ph, char[] dst_ch, int pos) //Name of the debug variable
            {
                for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (char)((char) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }
            public int name_LEN(Inside ph)
            {
                return (ph.field_bit !=  64 && !try_visit_field(ph, 64)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
            }
        }
        new class STATUSTEXT : GroundControl.STATUSTEXT
        {
            public MAV_SEVERITY severity //Severity of status. Relies on the definitions within RFC-5424. See enum MAV_SEVERITY.
            {
                get {  return (MAV_SEVERITY)(0 +  BitUtils.get_bits(data, 0, 3));}
            }
            public string text_TRY(Inside ph)//Status text message, without null termination character
            {
                if(ph.field_bit !=  3 && !try_visit_field(ph, 3)  ||  !try_visit_item(ph, 0)) return null;
                return new string(text_GET(ph, new char[ph.items], 0));
            }
            public char[]text_GET(Inside ph, char[] dst_ch, int pos) //Status text message, without null termination character
            {
                for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (char)((char) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }
            public int text_LEN(Inside ph)
            {
                return (ph.field_bit !=  3 && !try_visit_field(ph, 3)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
            }
        }
        new class DEBUG : GroundControl.DEBUG
        {
            public uint time_boot_ms //Timestamp (milliseconds since system boot)
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  0, 4));}
            }

            public byte ind //index of debug variable
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  4, 1));}
            }

            public float value //DEBUG value
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  5, 4)));}
            }
        }
        new class SETUP_SIGNING : GroundControl.SETUP_SIGNING
        {
            public ulong initial_timestamp //initial timestamp
            {
                get {  return (BitUtils.get_bytes(data,  0, 8));}
            }

            public byte target_system //system id of the target
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  8, 1));}
            }

            public byte target_component //component ID of the target
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  9, 1));}
            }

            public byte[] secret_key //signing key
            {
                get {return secret_key_GET(new byte[32], 0);}
            }
            public byte[]secret_key_GET(byte[] dst_ch, int pos)  //signing key
            {
                for(int BYTE = 10, dst_max = pos + 32; pos < dst_max ; pos++,  BYTE += 1)
                    dst_ch[pos] = (byte)((byte) BitUtils.get_bytes(data,  BYTE, 1));
                return dst_ch;
            }
        }
        new class BUTTON_CHANGE : GroundControl.BUTTON_CHANGE
        {
            public uint time_boot_ms //Timestamp (milliseconds since system boot)
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  0, 4));}
            }

            public uint last_change_ms //Time of last change of button state
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  4, 4));}
            }

            public byte state //Bitmap state of buttons
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  8, 1));}
            }
        }
        new class PLAY_TUNE : GroundControl.PLAY_TUNE
        {
            public byte target_system //System ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  0, 1));}
            }

            public byte target_component //Component ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  1, 1));}
            }
            public string tune_TRY(Inside ph)//tune in board specific format
            {
                if(ph.field_bit !=  16 && !try_visit_field(ph, 16)  ||  !try_visit_item(ph, 0)) return null;
                return new string(tune_GET(ph, new char[ph.items], 0));
            }
            public char[]tune_GET(Inside ph, char[] dst_ch, int pos) //tune in board specific format
            {
                for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (char)((char) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }
            public int tune_LEN(Inside ph)
            {
                return (ph.field_bit !=  16 && !try_visit_field(ph, 16)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
            }
        }
        new class CAMERA_INFORMATION : GroundControl.CAMERA_INFORMATION
        {
            public ushort resolution_h //Image resolution in pixels horizontal
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
            }

            public ushort resolution_v //Image resolution in pixels vertical
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  2, 2));}
            }

            public ushort cam_definition_version //Camera definition version (iteration)
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  4, 2));}
            }

            public uint time_boot_ms //Timestamp (milliseconds since system boot)
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  6, 4));}
            }

            public uint firmware_version //0xff = Major)
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  10, 4));}
            }

            public byte[] vendor_name //Name of the camera vendor
            {
                get {return vendor_name_GET(new byte[32], 0);}
            }
            public byte[]vendor_name_GET(byte[] dst_ch, int pos)  //Name of the camera vendor
            {
                for(int BYTE = 14, dst_max = pos + 32; pos < dst_max ; pos++,  BYTE += 1)
                    dst_ch[pos] = (byte)((byte) BitUtils.get_bytes(data,  BYTE, 1));
                return dst_ch;
            }
            public byte[] model_name //Name of the camera model
            {
                get {return model_name_GET(new byte[32], 0);}
            }
            public byte[]model_name_GET(byte[] dst_ch, int pos)  //Name of the camera model
            {
                for(int BYTE = 46, dst_max = pos + 32; pos < dst_max ; pos++,  BYTE += 1)
                    dst_ch[pos] = (byte)((byte) BitUtils.get_bytes(data,  BYTE, 1));
                return dst_ch;
            }
            public float focal_length //Focal length in mm
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  78, 4)));}
            }

            public float sensor_size_h //Image sensor size horizontal in mm
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  82, 4)));}
            }

            public float sensor_size_v //Image sensor size vertical in mm
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  86, 4)));}
            }

            public byte lens_id //Reserved for a lens ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  90, 1));}
            }

            public CAMERA_CAP_FLAGS flags //CAMERA_CAP_FLAGS enum flags (bitmap) describing camera capabilities.
            {
                get {  return (CAMERA_CAP_FLAGS)(1 +  BitUtils.get_bits(data, 728, 6));}
            }
            public string cam_definition_uri_TRY(Inside ph)//Camera definition URI (if any, otherwise only basic functions will be available).
            {
                if(ph.field_bit !=  734 && !try_visit_field(ph, 734)  ||  !try_visit_item(ph, 0)) return null;
                return new string(cam_definition_uri_GET(ph, new char[ph.items], 0));
            }
            public char[]cam_definition_uri_GET(Inside ph, char[] dst_ch, int pos) //Camera definition URI (if any, otherwise only basic functions will be available).
            {
                for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (char)((char) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }
            public int cam_definition_uri_LEN(Inside ph)
            {
                return (ph.field_bit !=  734 && !try_visit_field(ph, 734)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
            }
        }
        new class CAMERA_SETTINGS : GroundControl.CAMERA_SETTINGS
        {
            public uint time_boot_ms //Timestamp (milliseconds since system boot)
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  0, 4));}
            }

            public CAMERA_MODE mode_id //Camera mode (CAMERA_MODE)
            {
                get {  return (CAMERA_MODE)(0 +  BitUtils.get_bits(data, 32, 2));}
            }
        }
        new class STORAGE_INFORMATION : GroundControl.STORAGE_INFORMATION
        {
            public uint time_boot_ms //Timestamp (milliseconds since system boot)
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  0, 4));}
            }

            public byte storage_id //Storage ID (1 for first, 2 for second, etc.)
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  4, 1));}
            }

            public byte storage_count //Number of storage devices
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  5, 1));}
            }

            public byte status //Status of storage (0 not available, 1 unformatted, 2 formatted)
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  6, 1));}
            }

            public float total_capacity //Total capacity in MiB
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  7, 4)));}
            }

            public float used_capacity //Used capacity in MiB
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  11, 4)));}
            }

            public float available_capacity //Available capacity in MiB
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  15, 4)));}
            }

            public float read_speed //Read speed in MiB/s
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  19, 4)));}
            }

            public float write_speed //Write speed in MiB/s
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  23, 4)));}
            }
        }
        new class CAMERA_CAPTURE_STATUS : GroundControl.CAMERA_CAPTURE_STATUS
        {
            public uint time_boot_ms //Timestamp (milliseconds since system boot)
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  0, 4));}
            }

            public uint recording_time_ms //Time in milliseconds since recording started
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  4, 4));}
            }

            /**
            *Current status of image capturing (0: idle, 1: capture in progress, 2: interval set but idle, 3: interval
            *	set and capture in progress*/
            public byte image_status
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  8, 1));}
            }

            public byte video_status //Current status of video capturing (0: idle, 1: capture in progress)
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  9, 1));}
            }

            public float image_interval //Image capture interval in seconds
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  10, 4)));}
            }

            public float available_capacity //Available storage capacity in MiB
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  14, 4)));}
            }
        }
        new class CAMERA_IMAGE_CAPTURED : GroundControl.CAMERA_IMAGE_CAPTURED
        {
            public uint time_boot_ms //Timestamp (milliseconds since system boot)
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  0, 4));}
            }

            public ulong time_utc //Timestamp (microseconds since UNIX epoch) in UTC. 0 for unknown.
            {
                get {  return (BitUtils.get_bytes(data,  4, 8));}
            }

            public byte camera_id //Camera ID (1 for first, 2 for second, etc.)
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  12, 1));}
            }

            public int lat //Latitude, expressed as degrees * 1E7 where image was taken
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  13, 4));}
            }

            public int lon //Longitude, expressed as degrees * 1E7 where capture was taken
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  17, 4));}
            }

            public int alt //Altitude in meters, expressed as * 1E3 (AMSL, not WGS84) where image was taken
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  21, 4));}
            }

            public int relative_alt //Altitude above ground in meters, expressed as * 1E3 where image was taken
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  25, 4));}
            }

            public float[] q //Quaternion of camera orientation (w, x, y, z order, zero-rotation is 0, 0, 0, 0)
            {
                get {return q_GET(new float[4], 0);}
            }
            public float[]q_GET(float[] dst_ch, int pos)  //Quaternion of camera orientation (w, x, y, z order, zero-rotation is 0, 0, 0, 0)
            {
                for(int BYTE = 29, dst_max = pos + 4; pos < dst_max ; pos++,  BYTE += 4)
                    dst_ch[pos] = (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  BYTE, 4)));
                return dst_ch;
            }
            public int image_index //Zero based index of this image (image count since armed -1)
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  45, 4));}
            }

            public sbyte capture_result //Boolean indicating success (1) or failure (0) while capturing this image.
            {
                get {  return (sbyte)((sbyte) BitUtils.get_bytes(data,  49, 1));}
            }
            public string file_url_TRY(Inside ph)//URL of image taken. Either local storage or http:foo.jpg if camera provides an HTTP interface.
            {
                if(ph.field_bit !=  402 && !try_visit_field(ph, 402)  ||  !try_visit_item(ph, 0)) return null;
                return new string(file_url_GET(ph, new char[ph.items], 0));
            }
            public char[]file_url_GET(Inside ph, char[] dst_ch, int pos) //URL of image taken. Either local storage or http:foo.jpg if camera provides an HTTP interface.
            {
                for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (char)((char) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }
            public int file_url_LEN(Inside ph)
            {
                return (ph.field_bit !=  402 && !try_visit_field(ph, 402)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
            }
        }
        new class FLIGHT_INFORMATION : GroundControl.FLIGHT_INFORMATION
        {
            public uint time_boot_ms //Timestamp (milliseconds since system boot)
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  0, 4));}
            }

            public ulong arming_time_utc //Timestamp at arming (microseconds since UNIX epoch) in UTC, 0 for unknown
            {
                get {  return (BitUtils.get_bytes(data,  4, 8));}
            }

            public ulong takeoff_time_utc //Timestamp at takeoff (microseconds since UNIX epoch) in UTC, 0 for unknown
            {
                get {  return (BitUtils.get_bytes(data,  12, 8));}
            }

            public ulong flight_uuid //Universally unique identifier (UUID) of flight, should correspond to name of logfiles
            {
                get {  return (BitUtils.get_bytes(data,  20, 8));}
            }
        }
        new class MOUNT_ORIENTATION : GroundControl.MOUNT_ORIENTATION
        {
            public uint time_boot_ms //Timestamp (milliseconds since system boot)
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  0, 4));}
            }

            public float roll //Roll in degrees
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  4, 4)));}
            }

            public float pitch //Pitch in degrees
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  8, 4)));}
            }

            public float yaw //Yaw in degrees
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  12, 4)));}
            }
        }
        new class LOGGING_DATA : GroundControl.LOGGING_DATA
        {
            public ushort sequence //sequence number (can wrap)
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
            }

            public byte target_system //system ID of the target
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  2, 1));}
            }

            public byte target_component //component ID of the target
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  3, 1));}
            }

            public byte length //data length
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  4, 1));}
            }

            /**
            *offset into data where first message starts. This can be used for recovery, when a previous message got
            *	lost (set to 255 if no start exists)*/
            public byte first_message_offset
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  5, 1));}
            }

            public byte[] data_ //logged data
            {
                get {return data__GET(new byte[249], 0);}
            }
            public byte[]data__GET(byte[] dst_ch, int pos)  //logged data
            {
                for(int BYTE = 6, dst_max = pos + 249; pos < dst_max ; pos++,  BYTE += 1)
                    dst_ch[pos] = (byte)((byte) BitUtils.get_bytes(data,  BYTE, 1));
                return dst_ch;
            }
        }
        new class LOGGING_DATA_ACKED : GroundControl.LOGGING_DATA_ACKED
        {
            public ushort sequence //sequence number (can wrap)
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
            }

            public byte target_system //system ID of the target
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  2, 1));}
            }

            public byte target_component //component ID of the target
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  3, 1));}
            }

            public byte length //data length
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  4, 1));}
            }

            /**
            *offset into data where first message starts. This can be used for recovery, when a previous message got
            *	lost (set to 255 if no start exists)*/
            public byte first_message_offset
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  5, 1));}
            }

            public byte[] data_ //logged data
            {
                get {return data__GET(new byte[249], 0);}
            }
            public byte[]data__GET(byte[] dst_ch, int pos)  //logged data
            {
                for(int BYTE = 6, dst_max = pos + 249; pos < dst_max ; pos++,  BYTE += 1)
                    dst_ch[pos] = (byte)((byte) BitUtils.get_bytes(data,  BYTE, 1));
                return dst_ch;
            }
        }
        new class LOGGING_ACK : GroundControl.LOGGING_ACK
        {
            public ushort sequence //sequence number (must match the one in LOGGING_DATA_ACKED)
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
            }

            public byte target_system //system ID of the target
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  2, 1));}
            }

            public byte target_component //component ID of the target
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  3, 1));}
            }
        }
        new class VIDEO_STREAM_INFORMATION : GroundControl.VIDEO_STREAM_INFORMATION
        {
            public ushort resolution_h //Resolution horizontal in pixels
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
            }

            public ushort resolution_v //Resolution vertical in pixels
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  2, 2));}
            }

            public ushort rotation //Video image rotation clockwise
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  4, 2));}
            }

            public uint bitrate //Bit rate in bits per second
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  6, 4));}
            }

            public byte camera_id //Camera ID (1 for first, 2 for second, etc.)
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  10, 1));}
            }

            public byte status //Current status of video streaming (0: not running, 1: in progress)
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  11, 1));}
            }

            public float framerate //Frames per second
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  12, 4)));}
            }
            public string uri_TRY(Inside ph)//Video stream URI
            {
                if(ph.field_bit !=  130 && !try_visit_field(ph, 130)  ||  !try_visit_item(ph, 0)) return null;
                return new string(uri_GET(ph, new char[ph.items], 0));
            }
            public char[]uri_GET(Inside ph, char[] dst_ch, int pos) //Video stream URI
            {
                for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (char)((char) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }
            public int uri_LEN(Inside ph)
            {
                return (ph.field_bit !=  130 && !try_visit_field(ph, 130)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
            }
        }
        new class SET_VIDEO_STREAM_SETTINGS : GroundControl.SET_VIDEO_STREAM_SETTINGS
        {
            public ushort resolution_h //Resolution horizontal in pixels (set to -1 for highest resolution possible)
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
            }

            public ushort resolution_v //Resolution vertical in pixels (set to -1 for highest resolution possible)
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  2, 2));}
            }

            public ushort rotation //Video image rotation clockwise (0-359 degrees)
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  4, 2));}
            }

            public uint bitrate //Bit rate in bits per second (set to -1 for auto)
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  6, 4));}
            }

            public byte target_system //system ID of the target
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  10, 1));}
            }

            public byte target_component //component ID of the target
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  11, 1));}
            }

            public byte camera_id //Camera ID (1 for first, 2 for second, etc.)
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  12, 1));}
            }

            public float framerate //Frames per second (set to -1 for highest framerate possible)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  13, 4)));}
            }
            public string uri_TRY(Inside ph)//Video stream URI
            {
                if(ph.field_bit !=  138 && !try_visit_field(ph, 138)  ||  !try_visit_item(ph, 0)) return null;
                return new string(uri_GET(ph, new char[ph.items], 0));
            }
            public char[]uri_GET(Inside ph, char[] dst_ch, int pos) //Video stream URI
            {
                for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (char)((char) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }
            public int uri_LEN(Inside ph)
            {
                return (ph.field_bit !=  138 && !try_visit_field(ph, 138)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
            }
        }
        new class WIFI_CONFIG_AP : GroundControl.WIFI_CONFIG_AP
        {
            public string ssid_TRY(Inside ph)//Name of Wi-Fi network (SSID). Leave it blank to leave it unchanged.
            {
                if(ph.field_bit !=  2 && !try_visit_field(ph, 2)  ||  !try_visit_item(ph, 0)) return null;
                return new string(ssid_GET(ph, new char[ph.items], 0));
            }
            public char[]ssid_GET(Inside ph, char[] dst_ch, int pos) //Name of Wi-Fi network (SSID). Leave it blank to leave it unchanged.
            {
                for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (char)((char) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }
            public int ssid_LEN(Inside ph)
            {
                return (ph.field_bit !=  2 && !try_visit_field(ph, 2)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
            } public string password_TRY(Inside ph)//Password. Leave it blank for an open AP.
            {
                if(ph.field_bit !=  3 && !try_visit_field(ph, 3)  ||  !try_visit_item(ph, 0)) return null;
                return new string(password_GET(ph, new char[ph.items], 0));
            }
            public char[]password_GET(Inside ph, char[] dst_ch, int pos) //Password. Leave it blank for an open AP.
            {
                for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (char)((char) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }
            public int password_LEN(Inside ph)
            {
                return (ph.field_bit !=  3 && !try_visit_field(ph, 3)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
            }
        }
        new class PROTOCOL_VERSION : GroundControl.PROTOCOL_VERSION
        {
            public ushort version //Currently active MAVLink version number * 100: v1.0 is 100, v2.0 is 200, etc.
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
            }

            public ushort min_version //Minimum MAVLink version supported
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  2, 2));}
            }

            public ushort max_version //Maximum MAVLink version supported (set to the same value as version by default)
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  4, 2));}
            }

            public byte[] spec_version_hash //The first 8 bytes (not characters printed in hex!) of the git hash.
            {
                get {return spec_version_hash_GET(new byte[8], 0);}
            }
            public byte[]spec_version_hash_GET(byte[] dst_ch, int pos)  //The first 8 bytes (not characters printed in hex!) of the git hash.
            {
                for(int BYTE = 6, dst_max = pos + 8; pos < dst_max ; pos++,  BYTE += 1)
                    dst_ch[pos] = (byte)((byte) BitUtils.get_bytes(data,  BYTE, 1));
                return dst_ch;
            }
            public byte[] library_version_hash //The first 8 bytes (not characters printed in hex!) of the git hash.
            {
                get {return library_version_hash_GET(new byte[8], 0);}
            }
            public byte[]library_version_hash_GET(byte[] dst_ch, int pos)  //The first 8 bytes (not characters printed in hex!) of the git hash.
            {
                for(int BYTE = 14, dst_max = pos + 8; pos < dst_max ; pos++,  BYTE += 1)
                    dst_ch[pos] = (byte)((byte) BitUtils.get_bytes(data,  BYTE, 1));
                return dst_ch;
            }
        }
        new class UAVCAN_NODE_STATUS : GroundControl.UAVCAN_NODE_STATUS
        {
            public ushort vendor_specific_status_code //Vendor-specific status information.
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
            }

            public uint uptime_sec //The number of seconds since the start-up of the node.
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  2, 4));}
            }

            public ulong time_usec //Timestamp (microseconds since UNIX epoch or microseconds since system boot)
            {
                get {  return (BitUtils.get_bytes(data,  6, 8));}
            }

            public byte sub_mode //Not used currently.
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  14, 1));}
            }

            public UAVCAN_NODE_HEALTH health //Generalized node health status.
            {
                get {  return (UAVCAN_NODE_HEALTH)(0 +  BitUtils.get_bits(data, 120, 2));}
            }

            public UAVCAN_NODE_MODE mode //Generalized operating mode.
            {
                get
                {
                    switch((int)BitUtils.get_bits(data, 122, 3))
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
                    throw  new ArgumentException("Unknown enum ID ");
                }
            }
        }
        new class UAVCAN_NODE_INFO : GroundControl.UAVCAN_NODE_INFO
        {
            public uint uptime_sec //The number of seconds since the start-up of the node.
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  0, 4));}
            }

            public uint sw_vcs_commit //Version control system (VCS) revision identifier (e.g. git short commit hash). Zero if unknown.
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  4, 4));}
            }

            public ulong time_usec //Timestamp (microseconds since UNIX epoch or microseconds since system boot)
            {
                get {  return (BitUtils.get_bytes(data,  8, 8));}
            }

            public byte hw_version_major //Hardware major version number.
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  16, 1));}
            }

            public byte hw_version_minor //Hardware minor version number.
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  17, 1));}
            }

            public byte[] hw_unique_id //Hardware unique 128-bit ID.
            {
                get {return hw_unique_id_GET(new byte[16], 0);}
            }
            public byte[]hw_unique_id_GET(byte[] dst_ch, int pos)  //Hardware unique 128-bit ID.
            {
                for(int BYTE = 18, dst_max = pos + 16; pos < dst_max ; pos++,  BYTE += 1)
                    dst_ch[pos] = (byte)((byte) BitUtils.get_bytes(data,  BYTE, 1));
                return dst_ch;
            }
            public byte sw_version_major //Software major version number.
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  34, 1));}
            }

            public byte sw_version_minor //Software minor version number.
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  35, 1));}
            }
            public string name_TRY(Inside ph)//Node name string. For example, "sapog.px4.io".
            {
                if(ph.field_bit !=  288 && !try_visit_field(ph, 288)  ||  !try_visit_item(ph, 0)) return null;
                return new string(name_GET(ph, new char[ph.items], 0));
            }
            public char[]name_GET(Inside ph, char[] dst_ch, int pos) //Node name string. For example, "sapog.px4.io".
            {
                for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (char)((char) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }
            public int name_LEN(Inside ph)
            {
                return (ph.field_bit !=  288 && !try_visit_field(ph, 288)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
            }
        }
        new class PARAM_EXT_REQUEST_READ : GroundControl.PARAM_EXT_REQUEST_READ
        {
            public byte target_system //System ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  0, 1));}
            }

            public byte target_component //Component ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  1, 1));}
            }

            public short param_index //Parameter index. Set to -1 to use the Parameter ID field as identifier (else param_id will be ignored
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  2, 2));}
            }
            /**
            *Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination
            *	(NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the
            *	ID is stored as strin*/
            public string param_id_TRY(Inside ph)
            {
                if(ph.field_bit !=  32 && !try_visit_field(ph, 32)  ||  !try_visit_item(ph, 0)) return null;
                return new string(param_id_GET(ph, new char[ph.items], 0));
            }
            /**
            *Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination
            *	(NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the
            *	ID is stored as strin*/
            public char[]param_id_GET(Inside ph, char[] dst_ch, int pos)
            {
                for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (char)((char) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }
            public int param_id_LEN(Inside ph)
            {
                return (ph.field_bit !=  32 && !try_visit_field(ph, 32)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
            }
        }
        new class PARAM_EXT_REQUEST_LIST : GroundControl.PARAM_EXT_REQUEST_LIST
        {
            public byte target_system //System ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  0, 1));}
            }

            public byte target_component //Component ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  1, 1));}
            }
        }
        new class PARAM_EXT_VALUE : GroundControl.PARAM_EXT_VALUE
        {
            public ushort param_count //Total number of parameters
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
            }

            public ushort param_index //Index of this parameter
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  2, 2));}
            }

            public MAV_PARAM_EXT_TYPE param_type //Parameter type: see the MAV_PARAM_EXT_TYPE enum for supported data types.
            {
                get {  return (MAV_PARAM_EXT_TYPE)(1 +  BitUtils.get_bits(data, 32, 4));}
            }
            /**
            *Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination
            *	(NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the
            *	ID is stored as strin*/
            public string param_id_TRY(Inside ph)
            {
                if(ph.field_bit !=  38 && !try_visit_field(ph, 38)  ||  !try_visit_item(ph, 0)) return null;
                return new string(param_id_GET(ph, new char[ph.items], 0));
            }
            /**
            *Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination
            *	(NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the
            *	ID is stored as strin*/
            public char[]param_id_GET(Inside ph, char[] dst_ch, int pos)
            {
                for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (char)((char) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }
            public int param_id_LEN(Inside ph)
            {
                return (ph.field_bit !=  38 && !try_visit_field(ph, 38)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
            } public string param_value_TRY(Inside ph)//Parameter value
            {
                if(ph.field_bit !=  39 && !try_visit_field(ph, 39)  ||  !try_visit_item(ph, 0)) return null;
                return new string(param_value_GET(ph, new char[ph.items], 0));
            }
            public char[]param_value_GET(Inside ph, char[] dst_ch, int pos) //Parameter value
            {
                for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (char)((char) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }
            public int param_value_LEN(Inside ph)
            {
                return (ph.field_bit !=  39 && !try_visit_field(ph, 39)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
            }
        }
        new class PARAM_EXT_SET : GroundControl.PARAM_EXT_SET
        {
            public byte target_system //System ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  0, 1));}
            }

            public byte target_component //Component ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  1, 1));}
            }

            public MAV_PARAM_EXT_TYPE param_type //Parameter type: see the MAV_PARAM_EXT_TYPE enum for supported data types.
            {
                get {  return (MAV_PARAM_EXT_TYPE)(1 +  BitUtils.get_bits(data, 16, 4));}
            }
            /**
            *Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination
            *	(NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the
            *	ID is stored as strin*/
            public string param_id_TRY(Inside ph)
            {
                if(ph.field_bit !=  22 && !try_visit_field(ph, 22)  ||  !try_visit_item(ph, 0)) return null;
                return new string(param_id_GET(ph, new char[ph.items], 0));
            }
            /**
            *Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination
            *	(NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the
            *	ID is stored as strin*/
            public char[]param_id_GET(Inside ph, char[] dst_ch, int pos)
            {
                for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (char)((char) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }
            public int param_id_LEN(Inside ph)
            {
                return (ph.field_bit !=  22 && !try_visit_field(ph, 22)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
            } public string param_value_TRY(Inside ph)//Parameter value
            {
                if(ph.field_bit !=  23 && !try_visit_field(ph, 23)  ||  !try_visit_item(ph, 0)) return null;
                return new string(param_value_GET(ph, new char[ph.items], 0));
            }
            public char[]param_value_GET(Inside ph, char[] dst_ch, int pos) //Parameter value
            {
                for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (char)((char) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }
            public int param_value_LEN(Inside ph)
            {
                return (ph.field_bit !=  23 && !try_visit_field(ph, 23)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
            }
        }
        new class PARAM_EXT_ACK : GroundControl.PARAM_EXT_ACK
        {
            public MAV_PARAM_EXT_TYPE param_type //Parameter type: see the MAV_PARAM_EXT_TYPE enum for supported data types.
            {
                get {  return (MAV_PARAM_EXT_TYPE)(1 +  BitUtils.get_bits(data, 0, 4));}
            }

            public PARAM_ACK param_result //Result code: see the PARAM_ACK enum for possible codes.
            {
                get {  return (PARAM_ACK)(0 +  BitUtils.get_bits(data, 4, 2));}
            }
            /**
            *Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination
            *	(NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the
            *	ID is stored as strin*/
            public string param_id_TRY(Inside ph)
            {
                if(ph.field_bit !=  8 && !try_visit_field(ph, 8)  ||  !try_visit_item(ph, 0)) return null;
                return new string(param_id_GET(ph, new char[ph.items], 0));
            }
            /**
            *Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination
            *	(NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the
            *	ID is stored as strin*/
            public char[]param_id_GET(Inside ph, char[] dst_ch, int pos)
            {
                for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (char)((char) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }
            public int param_id_LEN(Inside ph)
            {
                return (ph.field_bit !=  8 && !try_visit_field(ph, 8)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
            } public string param_value_TRY(Inside ph)//Parameter value (new value if PARAM_ACK_ACCEPTED, current value otherwise)
            {
                if(ph.field_bit !=  9 && !try_visit_field(ph, 9)  ||  !try_visit_item(ph, 0)) return null;
                return new string(param_value_GET(ph, new char[ph.items], 0));
            }
            public char[]param_value_GET(Inside ph, char[] dst_ch, int pos) //Parameter value (new value if PARAM_ACK_ACCEPTED, current value otherwise)
            {
                for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (char)((char) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }
            public int param_value_LEN(Inside ph)
            {
                return (ph.field_bit !=  9 && !try_visit_field(ph, 9)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
            }
        }
        new class OBSTACLE_DISTANCE : GroundControl.OBSTACLE_DISTANCE
        {
            /**
            *Distance of obstacles in front of the sensor starting on the left side. A value of 0 means that the obstacle
            *	is right in front of the sensor. A value of max_distance +1 means no obstace is present. A value of UINT16_MAX
            *	for unknown/not used. In a array element, each unit corresponds to 1cm*/
            public ushort[] distances
            {
                get {return distances_GET(new ushort[72], 0);}
            }
            /**
            *Distance of obstacles in front of the sensor starting on the left side. A value of 0 means that the obstacle
            *	is right in front of the sensor. A value of max_distance +1 means no obstace is present. A value of UINT16_MAX
            *	for unknown/not used. In a array element, each unit corresponds to 1cm*/
            public ushort[]distances_GET(ushort[] dst_ch, int pos)
            {
                for(int BYTE = 0, dst_max = pos + 72; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (ushort)((ushort) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }
            public ushort min_distance //Minimum distance the sensor can measure in centimeters
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  144, 2));}
            }

            public ushort max_distance //Maximum distance the sensor can measure in centimeters
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  146, 2));}
            }

            public ulong time_usec //Timestamp (microseconds since system boot or since UNIX epoch)
            {
                get {  return (BitUtils.get_bytes(data,  148, 8));}
            }

            public byte increment //Angular width in degrees of each array element.
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  156, 1));}
            }

            public MAV_DISTANCE_SENSOR sensor_type //Class id of the distance sensor type.
            {
                get {  return (MAV_DISTANCE_SENSOR)(0 +  BitUtils.get_bits(data, 1256, 3));}
            }
        }



        class TestChannelAdvanced : Channel.Advanced
        {
            public override bool CanRead { get { return true; } }
            public override bool CanWrite { get { return true; } }

            public override void failure(string reason)
            {
                base.failure(reason);
                Debug.Assert(false);
            }

            public void OnFLEXIFUNCTION_SETReceive_direct(Channel src, Inside ph, FLEXIFUNCTION_SET pack) {OnFLEXIFUNCTION_SETReceive(this, ph,  pack);}
            public event FLEXIFUNCTION_SETReceiveHandler OnFLEXIFUNCTION_SETReceive;
            public delegate void FLEXIFUNCTION_SETReceiveHandler(Channel src, Inside ph, FLEXIFUNCTION_SET pack);
            public void OnFLEXIFUNCTION_READ_REQReceive_direct(Channel src, Inside ph, FLEXIFUNCTION_READ_REQ pack) {OnFLEXIFUNCTION_READ_REQReceive(this, ph,  pack);}
            public event FLEXIFUNCTION_READ_REQReceiveHandler OnFLEXIFUNCTION_READ_REQReceive;
            public delegate void FLEXIFUNCTION_READ_REQReceiveHandler(Channel src, Inside ph, FLEXIFUNCTION_READ_REQ pack);
            public void OnFLEXIFUNCTION_BUFFER_FUNCTIONReceive_direct(Channel src, Inside ph, FLEXIFUNCTION_BUFFER_FUNCTION pack) {OnFLEXIFUNCTION_BUFFER_FUNCTIONReceive(this, ph,  pack);}
            public event FLEXIFUNCTION_BUFFER_FUNCTIONReceiveHandler OnFLEXIFUNCTION_BUFFER_FUNCTIONReceive;
            public delegate void FLEXIFUNCTION_BUFFER_FUNCTIONReceiveHandler(Channel src, Inside ph, FLEXIFUNCTION_BUFFER_FUNCTION pack);
            public void OnFLEXIFUNCTION_BUFFER_FUNCTION_ACKReceive_direct(Channel src, Inside ph, FLEXIFUNCTION_BUFFER_FUNCTION_ACK pack) {OnFLEXIFUNCTION_BUFFER_FUNCTION_ACKReceive(this, ph,  pack);}
            public event FLEXIFUNCTION_BUFFER_FUNCTION_ACKReceiveHandler OnFLEXIFUNCTION_BUFFER_FUNCTION_ACKReceive;
            public delegate void FLEXIFUNCTION_BUFFER_FUNCTION_ACKReceiveHandler(Channel src, Inside ph, FLEXIFUNCTION_BUFFER_FUNCTION_ACK pack);
            public void OnFLEXIFUNCTION_DIRECTORYReceive_direct(Channel src, Inside ph, FLEXIFUNCTION_DIRECTORY pack) {OnFLEXIFUNCTION_DIRECTORYReceive(this, ph,  pack);}
            public event FLEXIFUNCTION_DIRECTORYReceiveHandler OnFLEXIFUNCTION_DIRECTORYReceive;
            public delegate void FLEXIFUNCTION_DIRECTORYReceiveHandler(Channel src, Inside ph, FLEXIFUNCTION_DIRECTORY pack);
            public void OnFLEXIFUNCTION_DIRECTORY_ACKReceive_direct(Channel src, Inside ph, FLEXIFUNCTION_DIRECTORY_ACK pack) {OnFLEXIFUNCTION_DIRECTORY_ACKReceive(this, ph,  pack);}
            public event FLEXIFUNCTION_DIRECTORY_ACKReceiveHandler OnFLEXIFUNCTION_DIRECTORY_ACKReceive;
            public delegate void FLEXIFUNCTION_DIRECTORY_ACKReceiveHandler(Channel src, Inside ph, FLEXIFUNCTION_DIRECTORY_ACK pack);
            public void OnFLEXIFUNCTION_COMMANDReceive_direct(Channel src, Inside ph, FLEXIFUNCTION_COMMAND pack) {OnFLEXIFUNCTION_COMMANDReceive(this, ph,  pack);}
            public event FLEXIFUNCTION_COMMANDReceiveHandler OnFLEXIFUNCTION_COMMANDReceive;
            public delegate void FLEXIFUNCTION_COMMANDReceiveHandler(Channel src, Inside ph, FLEXIFUNCTION_COMMAND pack);
            public void OnFLEXIFUNCTION_COMMAND_ACKReceive_direct(Channel src, Inside ph, FLEXIFUNCTION_COMMAND_ACK pack) {OnFLEXIFUNCTION_COMMAND_ACKReceive(this, ph,  pack);}
            public event FLEXIFUNCTION_COMMAND_ACKReceiveHandler OnFLEXIFUNCTION_COMMAND_ACKReceive;
            public delegate void FLEXIFUNCTION_COMMAND_ACKReceiveHandler(Channel src, Inside ph, FLEXIFUNCTION_COMMAND_ACK pack);
            public void OnSERIAL_UDB_EXTRA_F2_AReceive_direct(Channel src, Inside ph, SERIAL_UDB_EXTRA_F2_A pack) {OnSERIAL_UDB_EXTRA_F2_AReceive(this, ph,  pack);}
            public event SERIAL_UDB_EXTRA_F2_AReceiveHandler OnSERIAL_UDB_EXTRA_F2_AReceive;
            public delegate void SERIAL_UDB_EXTRA_F2_AReceiveHandler(Channel src, Inside ph, SERIAL_UDB_EXTRA_F2_A pack);
            public void OnSERIAL_UDB_EXTRA_F2_BReceive_direct(Channel src, Inside ph, SERIAL_UDB_EXTRA_F2_B pack) {OnSERIAL_UDB_EXTRA_F2_BReceive(this, ph,  pack);}
            public event SERIAL_UDB_EXTRA_F2_BReceiveHandler OnSERIAL_UDB_EXTRA_F2_BReceive;
            public delegate void SERIAL_UDB_EXTRA_F2_BReceiveHandler(Channel src, Inside ph, SERIAL_UDB_EXTRA_F2_B pack);
            public void OnSERIAL_UDB_EXTRA_F4Receive_direct(Channel src, Inside ph, SERIAL_UDB_EXTRA_F4 pack) {OnSERIAL_UDB_EXTRA_F4Receive(this, ph,  pack);}
            public event SERIAL_UDB_EXTRA_F4ReceiveHandler OnSERIAL_UDB_EXTRA_F4Receive;
            public delegate void SERIAL_UDB_EXTRA_F4ReceiveHandler(Channel src, Inside ph, SERIAL_UDB_EXTRA_F4 pack);
            public void OnSERIAL_UDB_EXTRA_F5Receive_direct(Channel src, Inside ph, SERIAL_UDB_EXTRA_F5 pack) {OnSERIAL_UDB_EXTRA_F5Receive(this, ph,  pack);}
            public event SERIAL_UDB_EXTRA_F5ReceiveHandler OnSERIAL_UDB_EXTRA_F5Receive;
            public delegate void SERIAL_UDB_EXTRA_F5ReceiveHandler(Channel src, Inside ph, SERIAL_UDB_EXTRA_F5 pack);
            public void OnSERIAL_UDB_EXTRA_F6Receive_direct(Channel src, Inside ph, SERIAL_UDB_EXTRA_F6 pack) {OnSERIAL_UDB_EXTRA_F6Receive(this, ph,  pack);}
            public event SERIAL_UDB_EXTRA_F6ReceiveHandler OnSERIAL_UDB_EXTRA_F6Receive;
            public delegate void SERIAL_UDB_EXTRA_F6ReceiveHandler(Channel src, Inside ph, SERIAL_UDB_EXTRA_F6 pack);
            public void OnSERIAL_UDB_EXTRA_F7Receive_direct(Channel src, Inside ph, SERIAL_UDB_EXTRA_F7 pack) {OnSERIAL_UDB_EXTRA_F7Receive(this, ph,  pack);}
            public event SERIAL_UDB_EXTRA_F7ReceiveHandler OnSERIAL_UDB_EXTRA_F7Receive;
            public delegate void SERIAL_UDB_EXTRA_F7ReceiveHandler(Channel src, Inside ph, SERIAL_UDB_EXTRA_F7 pack);
            public void OnSERIAL_UDB_EXTRA_F8Receive_direct(Channel src, Inside ph, SERIAL_UDB_EXTRA_F8 pack) {OnSERIAL_UDB_EXTRA_F8Receive(this, ph,  pack);}
            public event SERIAL_UDB_EXTRA_F8ReceiveHandler OnSERIAL_UDB_EXTRA_F8Receive;
            public delegate void SERIAL_UDB_EXTRA_F8ReceiveHandler(Channel src, Inside ph, SERIAL_UDB_EXTRA_F8 pack);
            public void OnSERIAL_UDB_EXTRA_F13Receive_direct(Channel src, Inside ph, SERIAL_UDB_EXTRA_F13 pack) {OnSERIAL_UDB_EXTRA_F13Receive(this, ph,  pack);}
            public event SERIAL_UDB_EXTRA_F13ReceiveHandler OnSERIAL_UDB_EXTRA_F13Receive;
            public delegate void SERIAL_UDB_EXTRA_F13ReceiveHandler(Channel src, Inside ph, SERIAL_UDB_EXTRA_F13 pack);
            public void OnSERIAL_UDB_EXTRA_F14Receive_direct(Channel src, Inside ph, SERIAL_UDB_EXTRA_F14 pack) {OnSERIAL_UDB_EXTRA_F14Receive(this, ph,  pack);}
            public event SERIAL_UDB_EXTRA_F14ReceiveHandler OnSERIAL_UDB_EXTRA_F14Receive;
            public delegate void SERIAL_UDB_EXTRA_F14ReceiveHandler(Channel src, Inside ph, SERIAL_UDB_EXTRA_F14 pack);
            public void OnSERIAL_UDB_EXTRA_F15Receive_direct(Channel src, Inside ph, SERIAL_UDB_EXTRA_F15 pack) {OnSERIAL_UDB_EXTRA_F15Receive(this, ph,  pack);}
            public event SERIAL_UDB_EXTRA_F15ReceiveHandler OnSERIAL_UDB_EXTRA_F15Receive;
            public delegate void SERIAL_UDB_EXTRA_F15ReceiveHandler(Channel src, Inside ph, SERIAL_UDB_EXTRA_F15 pack);
            public void OnSERIAL_UDB_EXTRA_F16Receive_direct(Channel src, Inside ph, SERIAL_UDB_EXTRA_F16 pack) {OnSERIAL_UDB_EXTRA_F16Receive(this, ph,  pack);}
            public event SERIAL_UDB_EXTRA_F16ReceiveHandler OnSERIAL_UDB_EXTRA_F16Receive;
            public delegate void SERIAL_UDB_EXTRA_F16ReceiveHandler(Channel src, Inside ph, SERIAL_UDB_EXTRA_F16 pack);
            public void OnALTITUDESReceive_direct(Channel src, Inside ph, ALTITUDES pack) {OnALTITUDESReceive(this, ph,  pack);}
            public event ALTITUDESReceiveHandler OnALTITUDESReceive;
            public delegate void ALTITUDESReceiveHandler(Channel src, Inside ph, ALTITUDES pack);
            public void OnAIRSPEEDSReceive_direct(Channel src, Inside ph, AIRSPEEDS pack) {OnAIRSPEEDSReceive(this, ph,  pack);}
            public event AIRSPEEDSReceiveHandler OnAIRSPEEDSReceive;
            public delegate void AIRSPEEDSReceiveHandler(Channel src, Inside ph, AIRSPEEDS pack);
            public void OnSERIAL_UDB_EXTRA_F17Receive_direct(Channel src, Inside ph, SERIAL_UDB_EXTRA_F17 pack) {OnSERIAL_UDB_EXTRA_F17Receive(this, ph,  pack);}
            public event SERIAL_UDB_EXTRA_F17ReceiveHandler OnSERIAL_UDB_EXTRA_F17Receive;
            public delegate void SERIAL_UDB_EXTRA_F17ReceiveHandler(Channel src, Inside ph, SERIAL_UDB_EXTRA_F17 pack);
            public void OnSERIAL_UDB_EXTRA_F18Receive_direct(Channel src, Inside ph, SERIAL_UDB_EXTRA_F18 pack) {OnSERIAL_UDB_EXTRA_F18Receive(this, ph,  pack);}
            public event SERIAL_UDB_EXTRA_F18ReceiveHandler OnSERIAL_UDB_EXTRA_F18Receive;
            public delegate void SERIAL_UDB_EXTRA_F18ReceiveHandler(Channel src, Inside ph, SERIAL_UDB_EXTRA_F18 pack);
            public void OnSERIAL_UDB_EXTRA_F19Receive_direct(Channel src, Inside ph, SERIAL_UDB_EXTRA_F19 pack) {OnSERIAL_UDB_EXTRA_F19Receive(this, ph,  pack);}
            public event SERIAL_UDB_EXTRA_F19ReceiveHandler OnSERIAL_UDB_EXTRA_F19Receive;
            public delegate void SERIAL_UDB_EXTRA_F19ReceiveHandler(Channel src, Inside ph, SERIAL_UDB_EXTRA_F19 pack);
            public void OnSERIAL_UDB_EXTRA_F20Receive_direct(Channel src, Inside ph, SERIAL_UDB_EXTRA_F20 pack) {OnSERIAL_UDB_EXTRA_F20Receive(this, ph,  pack);}
            public event SERIAL_UDB_EXTRA_F20ReceiveHandler OnSERIAL_UDB_EXTRA_F20Receive;
            public delegate void SERIAL_UDB_EXTRA_F20ReceiveHandler(Channel src, Inside ph, SERIAL_UDB_EXTRA_F20 pack);
            public void OnSERIAL_UDB_EXTRA_F21Receive_direct(Channel src, Inside ph, SERIAL_UDB_EXTRA_F21 pack) {OnSERIAL_UDB_EXTRA_F21Receive(this, ph,  pack);}
            public event SERIAL_UDB_EXTRA_F21ReceiveHandler OnSERIAL_UDB_EXTRA_F21Receive;
            public delegate void SERIAL_UDB_EXTRA_F21ReceiveHandler(Channel src, Inside ph, SERIAL_UDB_EXTRA_F21 pack);
            public void OnSERIAL_UDB_EXTRA_F22Receive_direct(Channel src, Inside ph, SERIAL_UDB_EXTRA_F22 pack) {OnSERIAL_UDB_EXTRA_F22Receive(this, ph,  pack);}
            public event SERIAL_UDB_EXTRA_F22ReceiveHandler OnSERIAL_UDB_EXTRA_F22Receive;
            public delegate void SERIAL_UDB_EXTRA_F22ReceiveHandler(Channel src, Inside ph, SERIAL_UDB_EXTRA_F22 pack);
            public void OnMESSAGE_INTERVALReceive_direct(Channel src, Inside ph, MESSAGE_INTERVAL pack) {OnMESSAGE_INTERVALReceive(this, ph,  pack);}
            public event MESSAGE_INTERVALReceiveHandler OnMESSAGE_INTERVALReceive;
            public delegate void MESSAGE_INTERVALReceiveHandler(Channel src, Inside ph, MESSAGE_INTERVAL pack);
            public void OnEXTENDED_SYS_STATEReceive_direct(Channel src, Inside ph, EXTENDED_SYS_STATE pack) {OnEXTENDED_SYS_STATEReceive(this, ph,  pack);}
            public event EXTENDED_SYS_STATEReceiveHandler OnEXTENDED_SYS_STATEReceive;
            public delegate void EXTENDED_SYS_STATEReceiveHandler(Channel src, Inside ph, EXTENDED_SYS_STATE pack);
            public void OnADSB_VEHICLEReceive_direct(Channel src, Inside ph, ADSB_VEHICLE pack) {OnADSB_VEHICLEReceive(this, ph,  pack);}
            public event ADSB_VEHICLEReceiveHandler OnADSB_VEHICLEReceive;
            public delegate void ADSB_VEHICLEReceiveHandler(Channel src, Inside ph, ADSB_VEHICLE pack);
            public void OnCOLLISIONReceive_direct(Channel src, Inside ph, COLLISION pack) {OnCOLLISIONReceive(this, ph,  pack);}
            public event COLLISIONReceiveHandler OnCOLLISIONReceive;
            public delegate void COLLISIONReceiveHandler(Channel src, Inside ph, COLLISION pack);
            public void OnV2_EXTENSIONReceive_direct(Channel src, Inside ph, V2_EXTENSION pack) {OnV2_EXTENSIONReceive(this, ph,  pack);}
            public event V2_EXTENSIONReceiveHandler OnV2_EXTENSIONReceive;
            public delegate void V2_EXTENSIONReceiveHandler(Channel src, Inside ph, V2_EXTENSION pack);
            public void OnMEMORY_VECTReceive_direct(Channel src, Inside ph, MEMORY_VECT pack) {OnMEMORY_VECTReceive(this, ph,  pack);}
            public event MEMORY_VECTReceiveHandler OnMEMORY_VECTReceive;
            public delegate void MEMORY_VECTReceiveHandler(Channel src, Inside ph, MEMORY_VECT pack);
            public void OnDEBUG_VECTReceive_direct(Channel src, Inside ph, DEBUG_VECT pack) {OnDEBUG_VECTReceive(this, ph,  pack);}
            public event DEBUG_VECTReceiveHandler OnDEBUG_VECTReceive;
            public delegate void DEBUG_VECTReceiveHandler(Channel src, Inside ph, DEBUG_VECT pack);
            public void OnNAMED_VALUE_FLOATReceive_direct(Channel src, Inside ph, NAMED_VALUE_FLOAT pack) {OnNAMED_VALUE_FLOATReceive(this, ph,  pack);}
            public event NAMED_VALUE_FLOATReceiveHandler OnNAMED_VALUE_FLOATReceive;
            public delegate void NAMED_VALUE_FLOATReceiveHandler(Channel src, Inside ph, NAMED_VALUE_FLOAT pack);
            public void OnNAMED_VALUE_INTReceive_direct(Channel src, Inside ph, NAMED_VALUE_INT pack) {OnNAMED_VALUE_INTReceive(this, ph,  pack);}
            public event NAMED_VALUE_INTReceiveHandler OnNAMED_VALUE_INTReceive;
            public delegate void NAMED_VALUE_INTReceiveHandler(Channel src, Inside ph, NAMED_VALUE_INT pack);
            public void OnSTATUSTEXTReceive_direct(Channel src, Inside ph, STATUSTEXT pack) {OnSTATUSTEXTReceive(this, ph,  pack);}
            public event STATUSTEXTReceiveHandler OnSTATUSTEXTReceive;
            public delegate void STATUSTEXTReceiveHandler(Channel src, Inside ph, STATUSTEXT pack);
            public void OnDEBUGReceive_direct(Channel src, Inside ph, DEBUG pack) {OnDEBUGReceive(this, ph,  pack);}
            public event DEBUGReceiveHandler OnDEBUGReceive;
            public delegate void DEBUGReceiveHandler(Channel src, Inside ph, DEBUG pack);
            public void OnSETUP_SIGNINGReceive_direct(Channel src, Inside ph, SETUP_SIGNING pack) {OnSETUP_SIGNINGReceive(this, ph,  pack);}
            public event SETUP_SIGNINGReceiveHandler OnSETUP_SIGNINGReceive;
            public delegate void SETUP_SIGNINGReceiveHandler(Channel src, Inside ph, SETUP_SIGNING pack);
            public void OnBUTTON_CHANGEReceive_direct(Channel src, Inside ph, BUTTON_CHANGE pack) {OnBUTTON_CHANGEReceive(this, ph,  pack);}
            public event BUTTON_CHANGEReceiveHandler OnBUTTON_CHANGEReceive;
            public delegate void BUTTON_CHANGEReceiveHandler(Channel src, Inside ph, BUTTON_CHANGE pack);
            public void OnPLAY_TUNEReceive_direct(Channel src, Inside ph, PLAY_TUNE pack) {OnPLAY_TUNEReceive(this, ph,  pack);}
            public event PLAY_TUNEReceiveHandler OnPLAY_TUNEReceive;
            public delegate void PLAY_TUNEReceiveHandler(Channel src, Inside ph, PLAY_TUNE pack);
            public void OnCAMERA_INFORMATIONReceive_direct(Channel src, Inside ph, CAMERA_INFORMATION pack) {OnCAMERA_INFORMATIONReceive(this, ph,  pack);}
            public event CAMERA_INFORMATIONReceiveHandler OnCAMERA_INFORMATIONReceive;
            public delegate void CAMERA_INFORMATIONReceiveHandler(Channel src, Inside ph, CAMERA_INFORMATION pack);
            public void OnCAMERA_SETTINGSReceive_direct(Channel src, Inside ph, CAMERA_SETTINGS pack) {OnCAMERA_SETTINGSReceive(this, ph,  pack);}
            public event CAMERA_SETTINGSReceiveHandler OnCAMERA_SETTINGSReceive;
            public delegate void CAMERA_SETTINGSReceiveHandler(Channel src, Inside ph, CAMERA_SETTINGS pack);
            public void OnSTORAGE_INFORMATIONReceive_direct(Channel src, Inside ph, STORAGE_INFORMATION pack) {OnSTORAGE_INFORMATIONReceive(this, ph,  pack);}
            public event STORAGE_INFORMATIONReceiveHandler OnSTORAGE_INFORMATIONReceive;
            public delegate void STORAGE_INFORMATIONReceiveHandler(Channel src, Inside ph, STORAGE_INFORMATION pack);
            public void OnCAMERA_CAPTURE_STATUSReceive_direct(Channel src, Inside ph, CAMERA_CAPTURE_STATUS pack) {OnCAMERA_CAPTURE_STATUSReceive(this, ph,  pack);}
            public event CAMERA_CAPTURE_STATUSReceiveHandler OnCAMERA_CAPTURE_STATUSReceive;
            public delegate void CAMERA_CAPTURE_STATUSReceiveHandler(Channel src, Inside ph, CAMERA_CAPTURE_STATUS pack);
            public void OnCAMERA_IMAGE_CAPTUREDReceive_direct(Channel src, Inside ph, CAMERA_IMAGE_CAPTURED pack) {OnCAMERA_IMAGE_CAPTUREDReceive(this, ph,  pack);}
            public event CAMERA_IMAGE_CAPTUREDReceiveHandler OnCAMERA_IMAGE_CAPTUREDReceive;
            public delegate void CAMERA_IMAGE_CAPTUREDReceiveHandler(Channel src, Inside ph, CAMERA_IMAGE_CAPTURED pack);
            public void OnFLIGHT_INFORMATIONReceive_direct(Channel src, Inside ph, FLIGHT_INFORMATION pack) {OnFLIGHT_INFORMATIONReceive(this, ph,  pack);}
            public event FLIGHT_INFORMATIONReceiveHandler OnFLIGHT_INFORMATIONReceive;
            public delegate void FLIGHT_INFORMATIONReceiveHandler(Channel src, Inside ph, FLIGHT_INFORMATION pack);
            public void OnMOUNT_ORIENTATIONReceive_direct(Channel src, Inside ph, MOUNT_ORIENTATION pack) {OnMOUNT_ORIENTATIONReceive(this, ph,  pack);}
            public event MOUNT_ORIENTATIONReceiveHandler OnMOUNT_ORIENTATIONReceive;
            public delegate void MOUNT_ORIENTATIONReceiveHandler(Channel src, Inside ph, MOUNT_ORIENTATION pack);
            public void OnLOGGING_DATAReceive_direct(Channel src, Inside ph, LOGGING_DATA pack) {OnLOGGING_DATAReceive(this, ph,  pack);}
            public event LOGGING_DATAReceiveHandler OnLOGGING_DATAReceive;
            public delegate void LOGGING_DATAReceiveHandler(Channel src, Inside ph, LOGGING_DATA pack);
            public void OnLOGGING_DATA_ACKEDReceive_direct(Channel src, Inside ph, LOGGING_DATA_ACKED pack) {OnLOGGING_DATA_ACKEDReceive(this, ph,  pack);}
            public event LOGGING_DATA_ACKEDReceiveHandler OnLOGGING_DATA_ACKEDReceive;
            public delegate void LOGGING_DATA_ACKEDReceiveHandler(Channel src, Inside ph, LOGGING_DATA_ACKED pack);
            public void OnLOGGING_ACKReceive_direct(Channel src, Inside ph, LOGGING_ACK pack) {OnLOGGING_ACKReceive(this, ph,  pack);}
            public event LOGGING_ACKReceiveHandler OnLOGGING_ACKReceive;
            public delegate void LOGGING_ACKReceiveHandler(Channel src, Inside ph, LOGGING_ACK pack);
            public void OnVIDEO_STREAM_INFORMATIONReceive_direct(Channel src, Inside ph, VIDEO_STREAM_INFORMATION pack) {OnVIDEO_STREAM_INFORMATIONReceive(this, ph,  pack);}
            public event VIDEO_STREAM_INFORMATIONReceiveHandler OnVIDEO_STREAM_INFORMATIONReceive;
            public delegate void VIDEO_STREAM_INFORMATIONReceiveHandler(Channel src, Inside ph, VIDEO_STREAM_INFORMATION pack);
            public void OnSET_VIDEO_STREAM_SETTINGSReceive_direct(Channel src, Inside ph, SET_VIDEO_STREAM_SETTINGS pack) {OnSET_VIDEO_STREAM_SETTINGSReceive(this, ph,  pack);}
            public event SET_VIDEO_STREAM_SETTINGSReceiveHandler OnSET_VIDEO_STREAM_SETTINGSReceive;
            public delegate void SET_VIDEO_STREAM_SETTINGSReceiveHandler(Channel src, Inside ph, SET_VIDEO_STREAM_SETTINGS pack);
            public void OnWIFI_CONFIG_APReceive_direct(Channel src, Inside ph, WIFI_CONFIG_AP pack) {OnWIFI_CONFIG_APReceive(this, ph,  pack);}
            public event WIFI_CONFIG_APReceiveHandler OnWIFI_CONFIG_APReceive;
            public delegate void WIFI_CONFIG_APReceiveHandler(Channel src, Inside ph, WIFI_CONFIG_AP pack);
            public void OnPROTOCOL_VERSIONReceive_direct(Channel src, Inside ph, PROTOCOL_VERSION pack) {OnPROTOCOL_VERSIONReceive(this, ph,  pack);}
            public event PROTOCOL_VERSIONReceiveHandler OnPROTOCOL_VERSIONReceive;
            public delegate void PROTOCOL_VERSIONReceiveHandler(Channel src, Inside ph, PROTOCOL_VERSION pack);
            public void OnUAVCAN_NODE_STATUSReceive_direct(Channel src, Inside ph, UAVCAN_NODE_STATUS pack) {OnUAVCAN_NODE_STATUSReceive(this, ph,  pack);}
            public event UAVCAN_NODE_STATUSReceiveHandler OnUAVCAN_NODE_STATUSReceive;
            public delegate void UAVCAN_NODE_STATUSReceiveHandler(Channel src, Inside ph, UAVCAN_NODE_STATUS pack);
            public void OnUAVCAN_NODE_INFOReceive_direct(Channel src, Inside ph, UAVCAN_NODE_INFO pack) {OnUAVCAN_NODE_INFOReceive(this, ph,  pack);}
            public event UAVCAN_NODE_INFOReceiveHandler OnUAVCAN_NODE_INFOReceive;
            public delegate void UAVCAN_NODE_INFOReceiveHandler(Channel src, Inside ph, UAVCAN_NODE_INFO pack);
            public void OnPARAM_EXT_REQUEST_READReceive_direct(Channel src, Inside ph, PARAM_EXT_REQUEST_READ pack) {OnPARAM_EXT_REQUEST_READReceive(this, ph,  pack);}
            public event PARAM_EXT_REQUEST_READReceiveHandler OnPARAM_EXT_REQUEST_READReceive;
            public delegate void PARAM_EXT_REQUEST_READReceiveHandler(Channel src, Inside ph, PARAM_EXT_REQUEST_READ pack);
            public void OnPARAM_EXT_REQUEST_LISTReceive_direct(Channel src, Inside ph, PARAM_EXT_REQUEST_LIST pack) {OnPARAM_EXT_REQUEST_LISTReceive(this, ph,  pack);}
            public event PARAM_EXT_REQUEST_LISTReceiveHandler OnPARAM_EXT_REQUEST_LISTReceive;
            public delegate void PARAM_EXT_REQUEST_LISTReceiveHandler(Channel src, Inside ph, PARAM_EXT_REQUEST_LIST pack);
            public void OnPARAM_EXT_VALUEReceive_direct(Channel src, Inside ph, PARAM_EXT_VALUE pack) {OnPARAM_EXT_VALUEReceive(this, ph,  pack);}
            public event PARAM_EXT_VALUEReceiveHandler OnPARAM_EXT_VALUEReceive;
            public delegate void PARAM_EXT_VALUEReceiveHandler(Channel src, Inside ph, PARAM_EXT_VALUE pack);
            public void OnPARAM_EXT_SETReceive_direct(Channel src, Inside ph, PARAM_EXT_SET pack) {OnPARAM_EXT_SETReceive(this, ph,  pack);}
            public event PARAM_EXT_SETReceiveHandler OnPARAM_EXT_SETReceive;
            public delegate void PARAM_EXT_SETReceiveHandler(Channel src, Inside ph, PARAM_EXT_SET pack);
            public void OnPARAM_EXT_ACKReceive_direct(Channel src, Inside ph, PARAM_EXT_ACK pack) {OnPARAM_EXT_ACKReceive(this, ph,  pack);}
            public event PARAM_EXT_ACKReceiveHandler OnPARAM_EXT_ACKReceive;
            public delegate void PARAM_EXT_ACKReceiveHandler(Channel src, Inside ph, PARAM_EXT_ACK pack);
            public void OnOBSTACLE_DISTANCEReceive_direct(Channel src, Inside ph, OBSTACLE_DISTANCE pack) {OnOBSTACLE_DISTANCEReceive(this, ph,  pack);}
            public event OBSTACLE_DISTANCEReceiveHandler OnOBSTACLE_DISTANCEReceive;
            public delegate void OBSTACLE_DISTANCEReceiveHandler(Channel src, Inside ph, OBSTACLE_DISTANCE pack);

            static Pack testing_pack;
            readonly  byte[]                        buf = new byte[4024];

            internal void send(Pack pack)
            {
                testing_pack = pack;
            }

            readonly Inside ph = new Inside();

            protected internal override Pack process(Pack pack, int id)
            {
                switch(id)
                {
                    case Channel.PROCESS_CHANNEL_REQEST:
                        if(pack == null)
                        {
                            pack = testing_pack;
                            testing_pack = null;
                        }
                        else testing_pack = pack;
                        return pack;
                    case Channel.PROCESS_RECEIVED:
                        if(testing_pack == null) return null;
                        ph.setPack(pack = testing_pack);
                        testing_pack = null;
                        id = pack.meta.id;
                        break;
                }
                switch(id)
                {
                    case 0:
                        if(pack == null) return new HEARTBEAT();
                        break;
                    case 1:
                        if(pack == null) return new SYS_STATUS();
                        break;
                    case 2:
                        if(pack == null) return new SYSTEM_TIME();
                        break;
                    case 3:
                        if(pack == null) return new POSITION_TARGET_LOCAL_NED();
                        break;
                    case 4:
                        if(pack == null) return new PING();
                        break;
                    case 5:
                        if(pack == null) return new CHANGE_OPERATOR_CONTROL();
                        break;
                    case 6:
                        if(pack == null) return new CHANGE_OPERATOR_CONTROL_ACK();
                        break;
                    case 7:
                        if(pack == null) return new AUTH_KEY();
                        break;
                    case 11:
                        if(pack == null) return new SET_MODE();
                        break;
                    case 20:
                        if(pack == null) return new PARAM_REQUEST_READ();
                        break;
                    case 21:
                        if(pack == null) return new PARAM_REQUEST_LIST();
                        break;
                    case 22:
                        if(pack == null) return new PARAM_VALUE();
                        break;
                    case 23:
                        if(pack == null) return new PARAM_SET();
                        break;
                    case 24:
                        if(pack == null) return new GPS_RAW_INT();
                        break;
                    case 25:
                        if(pack == null) return new GPS_STATUS();
                        break;
                    case 26:
                        if(pack == null) return new SCALED_IMU();
                        break;
                    case 27:
                        if(pack == null) return new RAW_IMU();
                        break;
                    case 28:
                        if(pack == null) return new RAW_PRESSURE();
                        break;
                    case 29:
                        if(pack == null) return new SCALED_PRESSURE();
                        break;
                    case 30:
                        if(pack == null) return new ATTITUDE();
                        break;
                    case 31:
                        if(pack == null) return new ATTITUDE_QUATERNION();
                        break;
                    case 32:
                        if(pack == null) return new LOCAL_POSITION_NED();
                        break;
                    case 33:
                        if(pack == null) return new GLOBAL_POSITION_INT();
                        break;
                    case 34:
                        if(pack == null) return new RC_CHANNELS_SCALED();
                        break;
                    case 35:
                        if(pack == null) return new RC_CHANNELS_RAW();
                        break;
                    case 36:
                        if(pack == null) return new SERVO_OUTPUT_RAW();
                        break;
                    case 37:
                        if(pack == null) return new MISSION_REQUEST_PARTIAL_LIST();
                        break;
                    case 38:
                        if(pack == null) return new MISSION_WRITE_PARTIAL_LIST();
                        break;
                    case 39:
                        if(pack == null) return new MISSION_ITEM();
                        break;
                    case 40:
                        if(pack == null) return new MISSION_REQUEST();
                        break;
                    case 41:
                        if(pack == null) return new MISSION_SET_CURRENT();
                        break;
                    case 42:
                        if(pack == null) return new MISSION_CURRENT();
                        break;
                    case 43:
                        if(pack == null) return new MISSION_REQUEST_LIST();
                        break;
                    case 44:
                        if(pack == null) return new MISSION_COUNT();
                        break;
                    case 45:
                        if(pack == null) return new MISSION_CLEAR_ALL();
                        break;
                    case 46:
                        if(pack == null) return new MISSION_ITEM_REACHED();
                        break;
                    case 47:
                        if(pack == null) return new MISSION_ACK();
                        break;
                    case 48:
                        if(pack == null) return new SET_GPS_GLOBAL_ORIGIN();
                        break;
                    case 49:
                        if(pack == null) return new GPS_GLOBAL_ORIGIN();
                        break;
                    case 50:
                        if(pack == null) return new PARAM_MAP_RC();
                        break;
                    case 51:
                        if(pack == null) return new MISSION_REQUEST_INT();
                        break;
                    case 54:
                        if(pack == null) return new SAFETY_SET_ALLOWED_AREA();
                        break;
                    case 55:
                        if(pack == null) return new SAFETY_ALLOWED_AREA();
                        break;
                    case 61:
                        if(pack == null) return new ATTITUDE_QUATERNION_COV();
                        break;
                    case 62:
                        if(pack == null) return new NAV_CONTROLLER_OUTPUT();
                        break;
                    case 63:
                        if(pack == null) return new GLOBAL_POSITION_INT_COV();
                        break;
                    case 64:
                        if(pack == null) return new LOCAL_POSITION_NED_COV();
                        break;
                    case 65:
                        if(pack == null) return new RC_CHANNELS();
                        break;
                    case 66:
                        if(pack == null) return new REQUEST_DATA_STREAM();
                        break;
                    case 67:
                        if(pack == null) return new DATA_STREAM();
                        break;
                    case 69:
                        if(pack == null) return new MANUAL_CONTROL();
                        break;
                    case 70:
                        if(pack == null) return new RC_CHANNELS_OVERRIDE();
                        break;
                    case 73:
                        if(pack == null) return new MISSION_ITEM_INT();
                        break;
                    case 74:
                        if(pack == null) return new VFR_HUD();
                        break;
                    case 75:
                        if(pack == null) return new COMMAND_INT();
                        break;
                    case 76:
                        if(pack == null) return new COMMAND_LONG();
                        break;
                    case 77:
                        if(pack == null) return new COMMAND_ACK();
                        break;
                    case 81:
                        if(pack == null) return new MANUAL_SETPOINT();
                        break;
                    case 82:
                        if(pack == null) return new SET_ATTITUDE_TARGET();
                        break;
                    case 83:
                        if(pack == null) return new ATTITUDE_TARGET();
                        break;
                    case 84:
                        if(pack == null) return new SET_POSITION_TARGET_LOCAL_NED();
                        break;
                    case 86:
                        if(pack == null) return new SET_POSITION_TARGET_GLOBAL_INT();
                        break;
                    case 150:
                        if(pack == null) return new FLEXIFUNCTION_SET();
                        OnFLEXIFUNCTION_SETReceive(this, ph, (FLEXIFUNCTION_SET) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 151:
                        if(pack == null) return new FLEXIFUNCTION_READ_REQ();
                        OnFLEXIFUNCTION_READ_REQReceive(this, ph, (FLEXIFUNCTION_READ_REQ) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 152:
                        if(pack == null) return new FLEXIFUNCTION_BUFFER_FUNCTION();
                        OnFLEXIFUNCTION_BUFFER_FUNCTIONReceive(this, ph, (FLEXIFUNCTION_BUFFER_FUNCTION) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 153:
                        if(pack == null) return new FLEXIFUNCTION_BUFFER_FUNCTION_ACK();
                        OnFLEXIFUNCTION_BUFFER_FUNCTION_ACKReceive(this, ph, (FLEXIFUNCTION_BUFFER_FUNCTION_ACK) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 155:
                        if(pack == null) return new FLEXIFUNCTION_DIRECTORY();
                        OnFLEXIFUNCTION_DIRECTORYReceive(this, ph, (FLEXIFUNCTION_DIRECTORY) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 156:
                        if(pack == null) return new FLEXIFUNCTION_DIRECTORY_ACK();
                        OnFLEXIFUNCTION_DIRECTORY_ACKReceive(this, ph, (FLEXIFUNCTION_DIRECTORY_ACK) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 157:
                        if(pack == null) return new FLEXIFUNCTION_COMMAND();
                        OnFLEXIFUNCTION_COMMANDReceive(this, ph, (FLEXIFUNCTION_COMMAND) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 158:
                        if(pack == null) return new FLEXIFUNCTION_COMMAND_ACK();
                        OnFLEXIFUNCTION_COMMAND_ACKReceive(this, ph, (FLEXIFUNCTION_COMMAND_ACK) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 170:
                        if(pack == null) return new SERIAL_UDB_EXTRA_F2_A();
                        OnSERIAL_UDB_EXTRA_F2_AReceive(this, ph, (SERIAL_UDB_EXTRA_F2_A) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 171:
                        if(pack == null) return new SERIAL_UDB_EXTRA_F2_B();
                        OnSERIAL_UDB_EXTRA_F2_BReceive(this, ph, (SERIAL_UDB_EXTRA_F2_B) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 172:
                        if(pack == null) return new SERIAL_UDB_EXTRA_F4();
                        OnSERIAL_UDB_EXTRA_F4Receive(this, ph, (SERIAL_UDB_EXTRA_F4) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 173:
                        if(pack == null) return new SERIAL_UDB_EXTRA_F5();
                        OnSERIAL_UDB_EXTRA_F5Receive(this, ph, (SERIAL_UDB_EXTRA_F5) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 174:
                        if(pack == null) return new SERIAL_UDB_EXTRA_F6();
                        OnSERIAL_UDB_EXTRA_F6Receive(this, ph, (SERIAL_UDB_EXTRA_F6) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 175:
                        if(pack == null) return new SERIAL_UDB_EXTRA_F7();
                        OnSERIAL_UDB_EXTRA_F7Receive(this, ph, (SERIAL_UDB_EXTRA_F7) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 176:
                        if(pack == null) return new SERIAL_UDB_EXTRA_F8();
                        OnSERIAL_UDB_EXTRA_F8Receive(this, ph, (SERIAL_UDB_EXTRA_F8) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 177:
                        if(pack == null) return new SERIAL_UDB_EXTRA_F13();
                        OnSERIAL_UDB_EXTRA_F13Receive(this, ph, (SERIAL_UDB_EXTRA_F13) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 178:
                        if(pack == null) return new SERIAL_UDB_EXTRA_F14();
                        OnSERIAL_UDB_EXTRA_F14Receive(this, ph, (SERIAL_UDB_EXTRA_F14) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 179:
                        if(pack == null) return new SERIAL_UDB_EXTRA_F15();
                        OnSERIAL_UDB_EXTRA_F15Receive(this, ph, (SERIAL_UDB_EXTRA_F15) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 180:
                        if(pack == null) return new SERIAL_UDB_EXTRA_F16();
                        OnSERIAL_UDB_EXTRA_F16Receive(this, ph, (SERIAL_UDB_EXTRA_F16) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 181:
                        if(pack == null) return new ALTITUDES();
                        OnALTITUDESReceive(this, ph, (ALTITUDES) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 182:
                        if(pack == null) return new AIRSPEEDS();
                        OnAIRSPEEDSReceive(this, ph, (AIRSPEEDS) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 183:
                        if(pack == null) return new SERIAL_UDB_EXTRA_F17();
                        OnSERIAL_UDB_EXTRA_F17Receive(this, ph, (SERIAL_UDB_EXTRA_F17) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 184:
                        if(pack == null) return new SERIAL_UDB_EXTRA_F18();
                        OnSERIAL_UDB_EXTRA_F18Receive(this, ph, (SERIAL_UDB_EXTRA_F18) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 185:
                        if(pack == null) return new SERIAL_UDB_EXTRA_F19();
                        OnSERIAL_UDB_EXTRA_F19Receive(this, ph, (SERIAL_UDB_EXTRA_F19) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 186:
                        if(pack == null) return new SERIAL_UDB_EXTRA_F20();
                        OnSERIAL_UDB_EXTRA_F20Receive(this, ph, (SERIAL_UDB_EXTRA_F20) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 187:
                        if(pack == null) return new SERIAL_UDB_EXTRA_F21();
                        OnSERIAL_UDB_EXTRA_F21Receive(this, ph, (SERIAL_UDB_EXTRA_F21) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 188:
                        if(pack == null) return new SERIAL_UDB_EXTRA_F22();
                        OnSERIAL_UDB_EXTRA_F22Receive(this, ph, (SERIAL_UDB_EXTRA_F22) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 244:
                        if(pack == null) return new MESSAGE_INTERVAL();
                        OnMESSAGE_INTERVALReceive(this, ph, (MESSAGE_INTERVAL) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 245:
                        if(pack == null) return new EXTENDED_SYS_STATE();
                        OnEXTENDED_SYS_STATEReceive(this, ph, (EXTENDED_SYS_STATE) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 246:
                        if(pack == null) return new ADSB_VEHICLE();
                        OnADSB_VEHICLEReceive(this, ph, (ADSB_VEHICLE) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 247:
                        if(pack == null) return new COLLISION();
                        OnCOLLISIONReceive(this, ph, (COLLISION) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 248:
                        if(pack == null) return new V2_EXTENSION();
                        OnV2_EXTENSIONReceive(this, ph, (V2_EXTENSION) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 249:
                        if(pack == null) return new MEMORY_VECT();
                        OnMEMORY_VECTReceive(this, ph, (MEMORY_VECT) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 250:
                        if(pack == null) return new DEBUG_VECT();
                        OnDEBUG_VECTReceive(this, ph, (DEBUG_VECT) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 251:
                        if(pack == null) return new NAMED_VALUE_FLOAT();
                        OnNAMED_VALUE_FLOATReceive(this, ph, (NAMED_VALUE_FLOAT) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 252:
                        if(pack == null) return new NAMED_VALUE_INT();
                        OnNAMED_VALUE_INTReceive(this, ph, (NAMED_VALUE_INT) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 253:
                        if(pack == null) return new STATUSTEXT();
                        OnSTATUSTEXTReceive(this, ph, (STATUSTEXT) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 254:
                        if(pack == null) return new DEBUG();
                        OnDEBUGReceive(this, ph, (DEBUG) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 256:
                        if(pack == null) return new SETUP_SIGNING();
                        OnSETUP_SIGNINGReceive(this, ph, (SETUP_SIGNING) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 257:
                        if(pack == null) return new BUTTON_CHANGE();
                        OnBUTTON_CHANGEReceive(this, ph, (BUTTON_CHANGE) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 258:
                        if(pack == null) return new PLAY_TUNE();
                        OnPLAY_TUNEReceive(this, ph, (PLAY_TUNE) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 259:
                        if(pack == null) return new CAMERA_INFORMATION();
                        OnCAMERA_INFORMATIONReceive(this, ph, (CAMERA_INFORMATION) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 260:
                        if(pack == null) return new CAMERA_SETTINGS();
                        OnCAMERA_SETTINGSReceive(this, ph, (CAMERA_SETTINGS) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 261:
                        if(pack == null) return new STORAGE_INFORMATION();
                        OnSTORAGE_INFORMATIONReceive(this, ph, (STORAGE_INFORMATION) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 262:
                        if(pack == null) return new CAMERA_CAPTURE_STATUS();
                        OnCAMERA_CAPTURE_STATUSReceive(this, ph, (CAMERA_CAPTURE_STATUS) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 263:
                        if(pack == null) return new CAMERA_IMAGE_CAPTURED();
                        OnCAMERA_IMAGE_CAPTUREDReceive(this, ph, (CAMERA_IMAGE_CAPTURED) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 264:
                        if(pack == null) return new FLIGHT_INFORMATION();
                        OnFLIGHT_INFORMATIONReceive(this, ph, (FLIGHT_INFORMATION) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 265:
                        if(pack == null) return new MOUNT_ORIENTATION();
                        OnMOUNT_ORIENTATIONReceive(this, ph, (MOUNT_ORIENTATION) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 266:
                        if(pack == null) return new LOGGING_DATA();
                        OnLOGGING_DATAReceive(this, ph, (LOGGING_DATA) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 267:
                        if(pack == null) return new LOGGING_DATA_ACKED();
                        OnLOGGING_DATA_ACKEDReceive(this, ph, (LOGGING_DATA_ACKED) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 268:
                        if(pack == null) return new LOGGING_ACK();
                        OnLOGGING_ACKReceive(this, ph, (LOGGING_ACK) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 269:
                        if(pack == null) return new VIDEO_STREAM_INFORMATION();
                        OnVIDEO_STREAM_INFORMATIONReceive(this, ph, (VIDEO_STREAM_INFORMATION) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 270:
                        if(pack == null) return new SET_VIDEO_STREAM_SETTINGS();
                        OnSET_VIDEO_STREAM_SETTINGSReceive(this, ph, (SET_VIDEO_STREAM_SETTINGS) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 299:
                        if(pack == null) return new WIFI_CONFIG_AP();
                        OnWIFI_CONFIG_APReceive(this, ph, (WIFI_CONFIG_AP) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 300:
                        if(pack == null) return new PROTOCOL_VERSION();
                        OnPROTOCOL_VERSIONReceive(this, ph, (PROTOCOL_VERSION) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 310:
                        if(pack == null) return new UAVCAN_NODE_STATUS();
                        OnUAVCAN_NODE_STATUSReceive(this, ph, (UAVCAN_NODE_STATUS) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 311:
                        if(pack == null) return new UAVCAN_NODE_INFO();
                        OnUAVCAN_NODE_INFOReceive(this, ph, (UAVCAN_NODE_INFO) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 320:
                        if(pack == null) return new PARAM_EXT_REQUEST_READ();
                        OnPARAM_EXT_REQUEST_READReceive(this, ph, (PARAM_EXT_REQUEST_READ) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 321:
                        if(pack == null) return new PARAM_EXT_REQUEST_LIST();
                        OnPARAM_EXT_REQUEST_LISTReceive(this, ph, (PARAM_EXT_REQUEST_LIST) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 322:
                        if(pack == null) return new PARAM_EXT_VALUE();
                        OnPARAM_EXT_VALUEReceive(this, ph, (PARAM_EXT_VALUE) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 323:
                        if(pack == null) return new PARAM_EXT_SET();
                        OnPARAM_EXT_SETReceive(this, ph, (PARAM_EXT_SET) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 324:
                        if(pack == null) return new PARAM_EXT_ACK();
                        OnPARAM_EXT_ACKReceive(this, ph, (PARAM_EXT_ACK) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 330:
                        if(pack == null) return new OBSTACLE_DISTANCE();
                        OnOBSTACLE_DISTANCEReceive(this, ph, (OBSTACLE_DISTANCE) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                }
                return null;
            }
            static readonly byte[] buff = new byte[1024];
            internal static void transmission(Channel src, Channel dst)
            {
                if(src is Channel.Advanced && !(dst is Channel.Advanced))
                {
                    for(int bytes; 0 < (bytes = src.Read(buff, 0, buff.Length));) ADV_TEST_CH.Write(buff, 0, bytes);
                    for(int bytes; 0 < (bytes = SMP_TEST_CH.Read(buff, 0, buff.Length));) dst.Write(buff, 0, bytes);
                }
                else if(!(src is Channel.Advanced) && dst is Channel.Advanced)
                {
                    for(int bytes; 0 < (bytes = src.Read(buff, 0, buff.Length));) SMP_TEST_CH.Write(buff, 0, bytes);
                    for(int bytes; 0 < (bytes = ADV_TEST_CH.Read(buff, 0, buff.Length));) dst.Write(buff, 0, bytes);
                }
                else
                    for(int bytes; 0 < (bytes = src.Read(buff, 0, buff.Length));) dst.Write(buff, 0, bytes);
                dst.process(null, Channel.PROCESS_RECEIVED);
            }
        }
        static readonly TestChannelAdvanced ADV_TEST_CH = new TestChannelAdvanced();

        public class TestChannelSimple : Channel
        {
            public override bool CanRead { get { return true; } }
            public override bool CanWrite { get { return true; } }

            public override void failure(String reason) { ADV_TEST_CH.failure(reason);}
            public void send(Pack pack) {ADV_TEST_CH.send(pack); }
            protected internal override Pack process(Pack pack, int id) { return ADV_TEST_CH.process(pack, id); }
        }
        static readonly TestChannelSimple SMP_TEST_CH = new TestChannelSimple();  //test channel with SimpleProtocol



        public static void Main(string[] args)
        {
            Inside PH = new Inside();
            CommunicationChannel.instance.OnHEARTBEATReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.base_mode == (MAV_MODE_FLAG.MAV_MODE_FLAG_AUTO_ENABLED |
                                                MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED |
                                                MAV_MODE_FLAG.MAV_MODE_FLAG_STABILIZE_ENABLED |
                                                MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED |
                                                MAV_MODE_FLAG.MAV_MODE_FLAG_HIL_ENABLED));
                Debug.Assert(pack.mavlink_version == (byte)(byte)232);
                Debug.Assert(pack.type == MAV_TYPE.MAV_TYPE_GCS);
                Debug.Assert(pack.system_status == MAV_STATE.MAV_STATE_POWEROFF);
                Debug.Assert(pack.custom_mode == (uint)2479149506U);
                Debug.Assert(pack.autopilot == MAV_AUTOPILOT.MAV_AUTOPILOT_ARMAZILA);
            };
            HEARTBEAT p0 = new HEARTBEAT();
            PH.setPack(p0);
            p0.autopilot = MAV_AUTOPILOT.MAV_AUTOPILOT_ARMAZILA;
            p0.type = MAV_TYPE.MAV_TYPE_GCS;
            p0.custom_mode = (uint)2479149506U;
            p0.mavlink_version = (byte)(byte)232;
            p0.system_status = MAV_STATE.MAV_STATE_POWEROFF;
            p0.base_mode = (MAV_MODE_FLAG.MAV_MODE_FLAG_AUTO_ENABLED |
                            MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED |
                            MAV_MODE_FLAG.MAV_MODE_FLAG_STABILIZE_ENABLED |
                            MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED |
                            MAV_MODE_FLAG.MAV_MODE_FLAG_HIL_ENABLED);
            ADV_TEST_CH.send(p0);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSYS_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.drop_rate_comm == (ushort)(ushort)42725);
                Debug.Assert(pack.onboard_control_sensors_health == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_BATTERY |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE));
                Debug.Assert(pack.errors_count3 == (ushort)(ushort)15087);
                Debug.Assert(pack.errors_count4 == (ushort)(ushort)49345);
                Debug.Assert(pack.onboard_control_sensors_present == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL));
                Debug.Assert(pack.errors_comm == (ushort)(ushort)38252);
                Debug.Assert(pack.load == (ushort)(ushort)54025);
                Debug.Assert(pack.current_battery == (short)(short)4992);
                Debug.Assert(pack.onboard_control_sensors_enabled == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR));
                Debug.Assert(pack.battery_remaining == (sbyte)(sbyte)73);
                Debug.Assert(pack.errors_count2 == (ushort)(ushort)65335);
                Debug.Assert(pack.voltage_battery == (ushort)(ushort)12552);
                Debug.Assert(pack.errors_count1 == (ushort)(ushort)7530);
            };
            SYS_STATUS p1 = new SYS_STATUS();
            PH.setPack(p1);
            p1.current_battery = (short)(short)4992;
            p1.errors_count4 = (ushort)(ushort)49345;
            p1.drop_rate_comm = (ushort)(ushort)42725;
            p1.errors_comm = (ushort)(ushort)38252;
            p1.onboard_control_sensors_enabled = (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR);
            p1.onboard_control_sensors_present = (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL);
            p1.errors_count1 = (ushort)(ushort)7530;
            p1.voltage_battery = (ushort)(ushort)12552;
            p1.errors_count2 = (ushort)(ushort)65335;
            p1.errors_count3 = (ushort)(ushort)15087;
            p1.load = (ushort)(ushort)54025;
            p1.battery_remaining = (sbyte)(sbyte)73;
            p1.onboard_control_sensors_health = (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_BATTERY |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE);
            ADV_TEST_CH.send(p1);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSYSTEM_TIMEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)2046638572U);
                Debug.Assert(pack.time_unix_usec == (ulong)8327080738610668934L);
            };
            SYSTEM_TIME p2 = new SYSTEM_TIME();
            PH.setPack(p2);
            p2.time_boot_ms = (uint)2046638572U;
            p2.time_unix_usec = (ulong)8327080738610668934L;
            ADV_TEST_CH.send(p2);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPOSITION_TARGET_LOCAL_NEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vz == (float) -1.8231594E38F);
                Debug.Assert(pack.y == (float)2.302281E38F);
                Debug.Assert(pack.afx == (float) -1.9405589E38F);
                Debug.Assert(pack.z == (float)1.6718597E38F);
                Debug.Assert(pack.afy == (float) -1.4139325E38F);
                Debug.Assert(pack.afz == (float)1.4336567E38F);
                Debug.Assert(pack.x == (float)2.961851E37F);
                Debug.Assert(pack.vy == (float) -1.3380031E38F);
                Debug.Assert(pack.time_boot_ms == (uint)3712268894U);
                Debug.Assert(pack.yaw == (float)6.6107563E37F);
                Debug.Assert(pack.type_mask == (ushort)(ushort)10395);
                Debug.Assert(pack.vx == (float)1.452954E38F);
                Debug.Assert(pack.yaw_rate == (float)6.8181597E37F);
                Debug.Assert(pack.coordinate_frame == MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT);
            };
            POSITION_TARGET_LOCAL_NED p3 = new POSITION_TARGET_LOCAL_NED();
            PH.setPack(p3);
            p3.vx = (float)1.452954E38F;
            p3.afy = (float) -1.4139325E38F;
            p3.x = (float)2.961851E37F;
            p3.coordinate_frame = MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT;
            p3.yaw_rate = (float)6.8181597E37F;
            p3.yaw = (float)6.6107563E37F;
            p3.vy = (float) -1.3380031E38F;
            p3.type_mask = (ushort)(ushort)10395;
            p3.vz = (float) -1.8231594E38F;
            p3.afz = (float)1.4336567E38F;
            p3.afx = (float) -1.9405589E38F;
            p3.time_boot_ms = (uint)3712268894U;
            p3.y = (float)2.302281E38F;
            p3.z = (float)1.6718597E38F;
            ADV_TEST_CH.send(p3);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPINGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (uint)667608551U);
                Debug.Assert(pack.time_usec == (ulong)3051734752481118286L);
                Debug.Assert(pack.target_system == (byte)(byte)112);
                Debug.Assert(pack.target_component == (byte)(byte)157);
            };
            PING p4 = new PING();
            PH.setPack(p4);
            p4.time_usec = (ulong)3051734752481118286L;
            p4.target_component = (byte)(byte)157;
            p4.target_system = (byte)(byte)112;
            p4.seq = (uint)667608551U;
            ADV_TEST_CH.send(p4);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCHANGE_OPERATOR_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.control_request == (byte)(byte)195);
                Debug.Assert(pack.version == (byte)(byte)131);
                Debug.Assert(pack.target_system == (byte)(byte)186);
                Debug.Assert(pack.passkey_LEN(ph) == 21);
                Debug.Assert(pack.passkey_TRY(ph).Equals("hxyjyfkbpwzgbhmwqgeck"));
            };
            CHANGE_OPERATOR_CONTROL p5 = new CHANGE_OPERATOR_CONTROL();
            PH.setPack(p5);
            p5.control_request = (byte)(byte)195;
            p5.version = (byte)(byte)131;
            p5.target_system = (byte)(byte)186;
            p5.passkey_SET("hxyjyfkbpwzgbhmwqgeck", PH) ;
            ADV_TEST_CH.send(p5);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCHANGE_OPERATOR_CONTROL_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.gcs_system_id == (byte)(byte)119);
                Debug.Assert(pack.ack == (byte)(byte)151);
                Debug.Assert(pack.control_request == (byte)(byte)163);
            };
            CHANGE_OPERATOR_CONTROL_ACK p6 = new CHANGE_OPERATOR_CONTROL_ACK();
            PH.setPack(p6);
            p6.gcs_system_id = (byte)(byte)119;
            p6.control_request = (byte)(byte)163;
            p6.ack = (byte)(byte)151;
            ADV_TEST_CH.send(p6);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnAUTH_KEYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.key_LEN(ph) == 1);
                Debug.Assert(pack.key_TRY(ph).Equals("m"));
            };
            AUTH_KEY p7 = new AUTH_KEY();
            PH.setPack(p7);
            p7.key_SET("m", PH) ;
            ADV_TEST_CH.send(p7);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_MODEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.custom_mode == (uint)3317468949U);
                Debug.Assert(pack.base_mode == MAV_MODE.MAV_MODE_TEST_DISARMED);
                Debug.Assert(pack.target_system == (byte)(byte)171);
            };
            SET_MODE p11 = new SET_MODE();
            PH.setPack(p11);
            p11.custom_mode = (uint)3317468949U;
            p11.base_mode = MAV_MODE.MAV_MODE_TEST_DISARMED;
            p11.target_system = (byte)(byte)171;
            ADV_TEST_CH.send(p11);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_REQUEST_READReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)157);
                Debug.Assert(pack.target_component == (byte)(byte)0);
                Debug.Assert(pack.param_id_LEN(ph) == 5);
                Debug.Assert(pack.param_id_TRY(ph).Equals("ryuuc"));
                Debug.Assert(pack.param_index == (short)(short)7168);
            };
            PARAM_REQUEST_READ p20 = new PARAM_REQUEST_READ();
            PH.setPack(p20);
            p20.target_system = (byte)(byte)157;
            p20.param_id_SET("ryuuc", PH) ;
            p20.param_index = (short)(short)7168;
            p20.target_component = (byte)(byte)0;
            ADV_TEST_CH.send(p20);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)105);
                Debug.Assert(pack.target_system == (byte)(byte)215);
            };
            PARAM_REQUEST_LIST p21 = new PARAM_REQUEST_LIST();
            PH.setPack(p21);
            p21.target_component = (byte)(byte)105;
            p21.target_system = (byte)(byte)215;
            ADV_TEST_CH.send(p21);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_VALUEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_value == (float)9.126643E37F);
                Debug.Assert(pack.param_count == (ushort)(ushort)1187);
                Debug.Assert(pack.param_index == (ushort)(ushort)58764);
                Debug.Assert(pack.param_id_LEN(ph) == 4);
                Debug.Assert(pack.param_id_TRY(ph).Equals("uexu"));
                Debug.Assert(pack.param_type == MAV_PARAM_TYPE.MAV_PARAM_TYPE_REAL32);
            };
            PARAM_VALUE p22 = new PARAM_VALUE();
            PH.setPack(p22);
            p22.param_id_SET("uexu", PH) ;
            p22.param_type = MAV_PARAM_TYPE.MAV_PARAM_TYPE_REAL32;
            p22.param_count = (ushort)(ushort)1187;
            p22.param_value = (float)9.126643E37F;
            p22.param_index = (ushort)(ushort)58764;
            ADV_TEST_CH.send(p22);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_SETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)30);
                Debug.Assert(pack.param_id_LEN(ph) == 13);
                Debug.Assert(pack.param_id_TRY(ph).Equals("gpAknsjmlpymh"));
                Debug.Assert(pack.param_type == MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT64);
                Debug.Assert(pack.param_value == (float)2.0149101E37F);
                Debug.Assert(pack.target_system == (byte)(byte)131);
            };
            PARAM_SET p23 = new PARAM_SET();
            PH.setPack(p23);
            p23.param_id_SET("gpAknsjmlpymh", PH) ;
            p23.target_system = (byte)(byte)131;
            p23.target_component = (byte)(byte)30;
            p23.param_type = MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT64;
            p23.param_value = (float)2.0149101E37F;
            ADV_TEST_CH.send(p23);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_RAW_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)8764851472085498180L);
                Debug.Assert(pack.fix_type == GPS_FIX_TYPE.GPS_FIX_TYPE_STATIC);
                Debug.Assert(pack.epv == (ushort)(ushort)35669);
                Debug.Assert(pack.vel == (ushort)(ushort)3101);
                Debug.Assert(pack.vel_acc_TRY(ph) == (uint)714902521U);
                Debug.Assert(pack.lon == (int)1338533751);
                Debug.Assert(pack.alt == (int) -741783196);
                Debug.Assert(pack.lat == (int) -631458872);
                Debug.Assert(pack.hdg_acc_TRY(ph) == (uint)22533002U);
                Debug.Assert(pack.h_acc_TRY(ph) == (uint)3524857430U);
                Debug.Assert(pack.alt_ellipsoid_TRY(ph) == (int)919884693);
                Debug.Assert(pack.satellites_visible == (byte)(byte)189);
                Debug.Assert(pack.cog == (ushort)(ushort)64013);
                Debug.Assert(pack.v_acc_TRY(ph) == (uint)4190085460U);
                Debug.Assert(pack.eph == (ushort)(ushort)41470);
            };
            GPS_RAW_INT p24 = new GPS_RAW_INT();
            PH.setPack(p24);
            p24.alt_ellipsoid_SET((int)919884693, PH) ;
            p24.lon = (int)1338533751;
            p24.vel_acc_SET((uint)714902521U, PH) ;
            p24.cog = (ushort)(ushort)64013;
            p24.fix_type = GPS_FIX_TYPE.GPS_FIX_TYPE_STATIC;
            p24.time_usec = (ulong)8764851472085498180L;
            p24.epv = (ushort)(ushort)35669;
            p24.eph = (ushort)(ushort)41470;
            p24.vel = (ushort)(ushort)3101;
            p24.v_acc_SET((uint)4190085460U, PH) ;
            p24.alt = (int) -741783196;
            p24.lat = (int) -631458872;
            p24.satellites_visible = (byte)(byte)189;
            p24.h_acc_SET((uint)3524857430U, PH) ;
            p24.hdg_acc_SET((uint)22533002U, PH) ;
            ADV_TEST_CH.send(p24);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.satellite_azimuth.SequenceEqual(new byte[] {(byte)239, (byte)75, (byte)143, (byte)196, (byte)7, (byte)10, (byte)103, (byte)197, (byte)46, (byte)104, (byte)53, (byte)120, (byte)169, (byte)16, (byte)55, (byte)203, (byte)47, (byte)8, (byte)190, (byte)68}));
                Debug.Assert(pack.satellite_snr.SequenceEqual(new byte[] {(byte)18, (byte)173, (byte)45, (byte)210, (byte)161, (byte)255, (byte)228, (byte)244, (byte)68, (byte)18, (byte)27, (byte)56, (byte)83, (byte)117, (byte)25, (byte)36, (byte)240, (byte)178, (byte)239, (byte)36}));
                Debug.Assert(pack.satellite_prn.SequenceEqual(new byte[] {(byte)7, (byte)205, (byte)224, (byte)19, (byte)177, (byte)11, (byte)182, (byte)163, (byte)234, (byte)102, (byte)248, (byte)38, (byte)36, (byte)44, (byte)107, (byte)190, (byte)146, (byte)86, (byte)214, (byte)89}));
                Debug.Assert(pack.satellites_visible == (byte)(byte)150);
                Debug.Assert(pack.satellite_used.SequenceEqual(new byte[] {(byte)24, (byte)238, (byte)253, (byte)137, (byte)41, (byte)58, (byte)99, (byte)51, (byte)41, (byte)35, (byte)206, (byte)229, (byte)215, (byte)105, (byte)43, (byte)116, (byte)89, (byte)254, (byte)160, (byte)164}));
                Debug.Assert(pack.satellite_elevation.SequenceEqual(new byte[] {(byte)60, (byte)29, (byte)178, (byte)203, (byte)202, (byte)176, (byte)188, (byte)198, (byte)91, (byte)174, (byte)209, (byte)5, (byte)1, (byte)188, (byte)201, (byte)43, (byte)197, (byte)56, (byte)252, (byte)243}));
            };
            GPS_STATUS p25 = new GPS_STATUS();
            PH.setPack(p25);
            p25.satellite_prn_SET(new byte[] {(byte)7, (byte)205, (byte)224, (byte)19, (byte)177, (byte)11, (byte)182, (byte)163, (byte)234, (byte)102, (byte)248, (byte)38, (byte)36, (byte)44, (byte)107, (byte)190, (byte)146, (byte)86, (byte)214, (byte)89}, 0) ;
            p25.satellite_elevation_SET(new byte[] {(byte)60, (byte)29, (byte)178, (byte)203, (byte)202, (byte)176, (byte)188, (byte)198, (byte)91, (byte)174, (byte)209, (byte)5, (byte)1, (byte)188, (byte)201, (byte)43, (byte)197, (byte)56, (byte)252, (byte)243}, 0) ;
            p25.satellites_visible = (byte)(byte)150;
            p25.satellite_azimuth_SET(new byte[] {(byte)239, (byte)75, (byte)143, (byte)196, (byte)7, (byte)10, (byte)103, (byte)197, (byte)46, (byte)104, (byte)53, (byte)120, (byte)169, (byte)16, (byte)55, (byte)203, (byte)47, (byte)8, (byte)190, (byte)68}, 0) ;
            p25.satellite_snr_SET(new byte[] {(byte)18, (byte)173, (byte)45, (byte)210, (byte)161, (byte)255, (byte)228, (byte)244, (byte)68, (byte)18, (byte)27, (byte)56, (byte)83, (byte)117, (byte)25, (byte)36, (byte)240, (byte)178, (byte)239, (byte)36}, 0) ;
            p25.satellite_used_SET(new byte[] {(byte)24, (byte)238, (byte)253, (byte)137, (byte)41, (byte)58, (byte)99, (byte)51, (byte)41, (byte)35, (byte)206, (byte)229, (byte)215, (byte)105, (byte)43, (byte)116, (byte)89, (byte)254, (byte)160, (byte)164}, 0) ;
            ADV_TEST_CH.send(p25);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_IMUReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.zmag == (short)(short)20860);
                Debug.Assert(pack.zacc == (short)(short) -6374);
                Debug.Assert(pack.xacc == (short)(short)7185);
                Debug.Assert(pack.zgyro == (short)(short)7681);
                Debug.Assert(pack.time_boot_ms == (uint)850717558U);
                Debug.Assert(pack.ymag == (short)(short)18836);
                Debug.Assert(pack.xmag == (short)(short)25318);
                Debug.Assert(pack.ygyro == (short)(short) -27786);
                Debug.Assert(pack.xgyro == (short)(short)22861);
                Debug.Assert(pack.yacc == (short)(short)14608);
            };
            SCALED_IMU p26 = new SCALED_IMU();
            PH.setPack(p26);
            p26.zacc = (short)(short) -6374;
            p26.time_boot_ms = (uint)850717558U;
            p26.xmag = (short)(short)25318;
            p26.zgyro = (short)(short)7681;
            p26.zmag = (short)(short)20860;
            p26.xacc = (short)(short)7185;
            p26.xgyro = (short)(short)22861;
            p26.ymag = (short)(short)18836;
            p26.yacc = (short)(short)14608;
            p26.ygyro = (short)(short) -27786;
            ADV_TEST_CH.send(p26);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRAW_IMUReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.xmag == (short)(short) -31364);
                Debug.Assert(pack.ygyro == (short)(short)26794);
                Debug.Assert(pack.xacc == (short)(short) -17074);
                Debug.Assert(pack.zacc == (short)(short)12436);
                Debug.Assert(pack.zmag == (short)(short) -567);
                Debug.Assert(pack.yacc == (short)(short)29574);
                Debug.Assert(pack.xgyro == (short)(short)6614);
                Debug.Assert(pack.time_usec == (ulong)7245659085066461747L);
                Debug.Assert(pack.ymag == (short)(short)5632);
                Debug.Assert(pack.zgyro == (short)(short)4350);
            };
            RAW_IMU p27 = new RAW_IMU();
            PH.setPack(p27);
            p27.zmag = (short)(short) -567;
            p27.zacc = (short)(short)12436;
            p27.yacc = (short)(short)29574;
            p27.zgyro = (short)(short)4350;
            p27.ymag = (short)(short)5632;
            p27.xacc = (short)(short) -17074;
            p27.xgyro = (short)(short)6614;
            p27.ygyro = (short)(short)26794;
            p27.time_usec = (ulong)7245659085066461747L;
            p27.xmag = (short)(short) -31364;
            ADV_TEST_CH.send(p27);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRAW_PRESSUREReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.temperature == (short)(short) -1164);
                Debug.Assert(pack.press_abs == (short)(short)25625);
                Debug.Assert(pack.time_usec == (ulong)2321874297000889846L);
                Debug.Assert(pack.press_diff1 == (short)(short)25003);
                Debug.Assert(pack.press_diff2 == (short)(short)19775);
            };
            RAW_PRESSURE p28 = new RAW_PRESSURE();
            PH.setPack(p28);
            p28.time_usec = (ulong)2321874297000889846L;
            p28.temperature = (short)(short) -1164;
            p28.press_diff2 = (short)(short)19775;
            p28.press_abs = (short)(short)25625;
            p28.press_diff1 = (short)(short)25003;
            ADV_TEST_CH.send(p28);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_PRESSUREReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)1344558292U);
                Debug.Assert(pack.temperature == (short)(short)6461);
                Debug.Assert(pack.press_abs == (float)2.342512E38F);
                Debug.Assert(pack.press_diff == (float) -2.264419E38F);
            };
            SCALED_PRESSURE p29 = new SCALED_PRESSURE();
            PH.setPack(p29);
            p29.press_diff = (float) -2.264419E38F;
            p29.press_abs = (float)2.342512E38F;
            p29.time_boot_ms = (uint)1344558292U;
            p29.temperature = (short)(short)6461;
            ADV_TEST_CH.send(p29);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATTITUDEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.yaw == (float) -1.6793135E38F);
                Debug.Assert(pack.yawspeed == (float)4.43158E37F);
                Debug.Assert(pack.roll == (float) -3.0890966E38F);
                Debug.Assert(pack.rollspeed == (float) -4.559436E37F);
                Debug.Assert(pack.pitchspeed == (float)6.7888714E37F);
                Debug.Assert(pack.pitch == (float) -1.819341E38F);
                Debug.Assert(pack.time_boot_ms == (uint)237518540U);
            };
            ATTITUDE p30 = new ATTITUDE();
            PH.setPack(p30);
            p30.yawspeed = (float)4.43158E37F;
            p30.pitch = (float) -1.819341E38F;
            p30.yaw = (float) -1.6793135E38F;
            p30.pitchspeed = (float)6.7888714E37F;
            p30.roll = (float) -3.0890966E38F;
            p30.time_boot_ms = (uint)237518540U;
            p30.rollspeed = (float) -4.559436E37F;
            ADV_TEST_CH.send(p30);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATTITUDE_QUATERNIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.yawspeed == (float)7.3115506E37F);
                Debug.Assert(pack.time_boot_ms == (uint)4182940550U);
                Debug.Assert(pack.q4 == (float) -2.0318318E38F);
                Debug.Assert(pack.q1 == (float) -3.1658436E38F);
                Debug.Assert(pack.pitchspeed == (float) -1.2232031E37F);
                Debug.Assert(pack.q2 == (float) -1.0329125E38F);
                Debug.Assert(pack.q3 == (float) -2.8069632E38F);
                Debug.Assert(pack.rollspeed == (float)1.0109579E38F);
            };
            ATTITUDE_QUATERNION p31 = new ATTITUDE_QUATERNION();
            PH.setPack(p31);
            p31.yawspeed = (float)7.3115506E37F;
            p31.q1 = (float) -3.1658436E38F;
            p31.pitchspeed = (float) -1.2232031E37F;
            p31.q2 = (float) -1.0329125E38F;
            p31.q4 = (float) -2.0318318E38F;
            p31.q3 = (float) -2.8069632E38F;
            p31.rollspeed = (float)1.0109579E38F;
            p31.time_boot_ms = (uint)4182940550U;
            ADV_TEST_CH.send(p31);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOCAL_POSITION_NEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.x == (float) -1.5157111E38F);
                Debug.Assert(pack.z == (float)2.7602392E38F);
                Debug.Assert(pack.vx == (float) -8.0343E37F);
                Debug.Assert(pack.y == (float) -2.9899637E38F);
                Debug.Assert(pack.vy == (float)1.7643356E38F);
                Debug.Assert(pack.vz == (float)1.9604659E38F);
                Debug.Assert(pack.time_boot_ms == (uint)207089443U);
            };
            LOCAL_POSITION_NED p32 = new LOCAL_POSITION_NED();
            PH.setPack(p32);
            p32.time_boot_ms = (uint)207089443U;
            p32.x = (float) -1.5157111E38F;
            p32.y = (float) -2.9899637E38F;
            p32.vx = (float) -8.0343E37F;
            p32.z = (float)2.7602392E38F;
            p32.vy = (float)1.7643356E38F;
            p32.vz = (float)1.9604659E38F;
            ADV_TEST_CH.send(p32);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGLOBAL_POSITION_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vx == (short)(short) -30060);
                Debug.Assert(pack.vz == (short)(short) -6426);
                Debug.Assert(pack.time_boot_ms == (uint)2441917082U);
                Debug.Assert(pack.vy == (short)(short) -26075);
                Debug.Assert(pack.hdg == (ushort)(ushort)26926);
                Debug.Assert(pack.lon == (int) -1041659882);
                Debug.Assert(pack.lat == (int)350570811);
                Debug.Assert(pack.relative_alt == (int)1202635559);
                Debug.Assert(pack.alt == (int)1518911862);
            };
            GLOBAL_POSITION_INT p33 = new GLOBAL_POSITION_INT();
            PH.setPack(p33);
            p33.vz = (short)(short) -6426;
            p33.alt = (int)1518911862;
            p33.lat = (int)350570811;
            p33.time_boot_ms = (uint)2441917082U;
            p33.relative_alt = (int)1202635559;
            p33.hdg = (ushort)(ushort)26926;
            p33.lon = (int) -1041659882;
            p33.vy = (short)(short) -26075;
            p33.vx = (short)(short) -30060;
            ADV_TEST_CH.send(p33);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRC_CHANNELS_SCALEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)2050353161U);
                Debug.Assert(pack.port == (byte)(byte)163);
                Debug.Assert(pack.rssi == (byte)(byte)15);
                Debug.Assert(pack.chan1_scaled == (short)(short)14470);
                Debug.Assert(pack.chan4_scaled == (short)(short)28340);
                Debug.Assert(pack.chan6_scaled == (short)(short) -15307);
                Debug.Assert(pack.chan3_scaled == (short)(short) -20837);
                Debug.Assert(pack.chan8_scaled == (short)(short) -6806);
                Debug.Assert(pack.chan7_scaled == (short)(short) -10617);
                Debug.Assert(pack.chan2_scaled == (short)(short)9156);
                Debug.Assert(pack.chan5_scaled == (short)(short) -11794);
            };
            RC_CHANNELS_SCALED p34 = new RC_CHANNELS_SCALED();
            PH.setPack(p34);
            p34.chan1_scaled = (short)(short)14470;
            p34.chan6_scaled = (short)(short) -15307;
            p34.chan3_scaled = (short)(short) -20837;
            p34.port = (byte)(byte)163;
            p34.chan5_scaled = (short)(short) -11794;
            p34.time_boot_ms = (uint)2050353161U;
            p34.chan7_scaled = (short)(short) -10617;
            p34.rssi = (byte)(byte)15;
            p34.chan8_scaled = (short)(short) -6806;
            p34.chan2_scaled = (short)(short)9156;
            p34.chan4_scaled = (short)(short)28340;
            ADV_TEST_CH.send(p34);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRC_CHANNELS_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.rssi == (byte)(byte)208);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)28773);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)1067);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)39782);
                Debug.Assert(pack.time_boot_ms == (uint)3185074216U);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)63110);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)55811);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)8180);
                Debug.Assert(pack.port == (byte)(byte)254);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)55138);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)20371);
            };
            RC_CHANNELS_RAW p35 = new RC_CHANNELS_RAW();
            PH.setPack(p35);
            p35.chan1_raw = (ushort)(ushort)55138;
            p35.time_boot_ms = (uint)3185074216U;
            p35.chan7_raw = (ushort)(ushort)1067;
            p35.rssi = (byte)(byte)208;
            p35.port = (byte)(byte)254;
            p35.chan5_raw = (ushort)(ushort)55811;
            p35.chan2_raw = (ushort)(ushort)63110;
            p35.chan4_raw = (ushort)(ushort)39782;
            p35.chan3_raw = (ushort)(ushort)28773;
            p35.chan8_raw = (ushort)(ushort)8180;
            p35.chan6_raw = (ushort)(ushort)20371;
            ADV_TEST_CH.send(p35);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSERVO_OUTPUT_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.servo12_raw_TRY(ph) == (ushort)(ushort)15138);
                Debug.Assert(pack.servo9_raw_TRY(ph) == (ushort)(ushort)13873);
                Debug.Assert(pack.servo3_raw == (ushort)(ushort)27937);
                Debug.Assert(pack.servo14_raw_TRY(ph) == (ushort)(ushort)22753);
                Debug.Assert(pack.servo13_raw_TRY(ph) == (ushort)(ushort)9594);
                Debug.Assert(pack.servo16_raw_TRY(ph) == (ushort)(ushort)14101);
                Debug.Assert(pack.servo15_raw_TRY(ph) == (ushort)(ushort)64920);
                Debug.Assert(pack.servo4_raw == (ushort)(ushort)18327);
                Debug.Assert(pack.time_usec == (uint)917166669U);
                Debug.Assert(pack.servo5_raw == (ushort)(ushort)64419);
                Debug.Assert(pack.servo8_raw == (ushort)(ushort)6551);
                Debug.Assert(pack.servo10_raw_TRY(ph) == (ushort)(ushort)11762);
                Debug.Assert(pack.servo2_raw == (ushort)(ushort)61854);
                Debug.Assert(pack.servo7_raw == (ushort)(ushort)15421);
                Debug.Assert(pack.servo11_raw_TRY(ph) == (ushort)(ushort)41777);
                Debug.Assert(pack.servo1_raw == (ushort)(ushort)41022);
                Debug.Assert(pack.servo6_raw == (ushort)(ushort)64928);
                Debug.Assert(pack.port == (byte)(byte)41);
            };
            SERVO_OUTPUT_RAW p36 = new SERVO_OUTPUT_RAW();
            PH.setPack(p36);
            p36.servo15_raw_SET((ushort)(ushort)64920, PH) ;
            p36.time_usec = (uint)917166669U;
            p36.servo13_raw_SET((ushort)(ushort)9594, PH) ;
            p36.servo9_raw_SET((ushort)(ushort)13873, PH) ;
            p36.servo5_raw = (ushort)(ushort)64419;
            p36.servo10_raw_SET((ushort)(ushort)11762, PH) ;
            p36.servo12_raw_SET((ushort)(ushort)15138, PH) ;
            p36.servo2_raw = (ushort)(ushort)61854;
            p36.servo1_raw = (ushort)(ushort)41022;
            p36.servo4_raw = (ushort)(ushort)18327;
            p36.servo8_raw = (ushort)(ushort)6551;
            p36.servo16_raw_SET((ushort)(ushort)14101, PH) ;
            p36.port = (byte)(byte)41;
            p36.servo14_raw_SET((ushort)(ushort)22753, PH) ;
            p36.servo11_raw_SET((ushort)(ushort)41777, PH) ;
            p36.servo3_raw = (ushort)(ushort)27937;
            p36.servo6_raw = (ushort)(ushort)64928;
            p36.servo7_raw = (ushort)(ushort)15421;
            ADV_TEST_CH.send(p36);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_REQUEST_PARTIAL_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.start_index == (short)(short) -5038);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
                Debug.Assert(pack.end_index == (short)(short) -15464);
                Debug.Assert(pack.target_component == (byte)(byte)43);
                Debug.Assert(pack.target_system == (byte)(byte)9);
            };
            MISSION_REQUEST_PARTIAL_LIST p37 = new MISSION_REQUEST_PARTIAL_LIST();
            PH.setPack(p37);
            p37.end_index = (short)(short) -15464;
            p37.start_index = (short)(short) -5038;
            p37.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY;
            p37.target_system = (byte)(byte)9;
            p37.target_component = (byte)(byte)43;
            ADV_TEST_CH.send(p37);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_WRITE_PARTIAL_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
                Debug.Assert(pack.end_index == (short)(short) -25361);
                Debug.Assert(pack.target_system == (byte)(byte)194);
                Debug.Assert(pack.target_component == (byte)(byte)108);
                Debug.Assert(pack.start_index == (short)(short) -12652);
            };
            MISSION_WRITE_PARTIAL_LIST p38 = new MISSION_WRITE_PARTIAL_LIST();
            PH.setPack(p38);
            p38.end_index = (short)(short) -25361;
            p38.target_system = (byte)(byte)194;
            p38.target_component = (byte)(byte)108;
            p38.start_index = (short)(short) -12652;
            p38.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE;
            ADV_TEST_CH.send(p38);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_ITEMReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.frame == MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT);
                Debug.Assert(pack.param3 == (float)2.4405067E38F);
                Debug.Assert(pack.command == MAV_CMD.MAV_CMD_WAYPOINT_USER_2);
                Debug.Assert(pack.param4 == (float)9.416315E37F);
                Debug.Assert(pack.autocontinue == (byte)(byte)118);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
                Debug.Assert(pack.z == (float) -3.0652493E38F);
                Debug.Assert(pack.y == (float) -2.5841788E38F);
                Debug.Assert(pack.seq == (ushort)(ushort)61371);
                Debug.Assert(pack.current == (byte)(byte)170);
                Debug.Assert(pack.x == (float) -5.056246E37F);
                Debug.Assert(pack.target_component == (byte)(byte)37);
                Debug.Assert(pack.param2 == (float)2.5242187E38F);
                Debug.Assert(pack.param1 == (float)2.6868796E38F);
                Debug.Assert(pack.target_system == (byte)(byte)64);
            };
            MISSION_ITEM p39 = new MISSION_ITEM();
            PH.setPack(p39);
            p39.y = (float) -2.5841788E38F;
            p39.command = MAV_CMD.MAV_CMD_WAYPOINT_USER_2;
            p39.x = (float) -5.056246E37F;
            p39.z = (float) -3.0652493E38F;
            p39.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION;
            p39.current = (byte)(byte)170;
            p39.param2 = (float)2.5242187E38F;
            p39.frame = MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;
            p39.target_system = (byte)(byte)64;
            p39.target_component = (byte)(byte)37;
            p39.param1 = (float)2.6868796E38F;
            p39.autocontinue = (byte)(byte)118;
            p39.param4 = (float)9.416315E37F;
            p39.param3 = (float)2.4405067E38F;
            p39.seq = (ushort)(ushort)61371;
            ADV_TEST_CH.send(p39);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)188);
                Debug.Assert(pack.seq == (ushort)(ushort)40979);
                Debug.Assert(pack.target_component == (byte)(byte)228);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
            };
            MISSION_REQUEST p40 = new MISSION_REQUEST();
            PH.setPack(p40);
            p40.target_system = (byte)(byte)188;
            p40.seq = (ushort)(ushort)40979;
            p40.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE;
            p40.target_component = (byte)(byte)228;
            ADV_TEST_CH.send(p40);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_SET_CURRENTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)254);
                Debug.Assert(pack.target_component == (byte)(byte)77);
                Debug.Assert(pack.seq == (ushort)(ushort)39846);
            };
            MISSION_SET_CURRENT p41 = new MISSION_SET_CURRENT();
            PH.setPack(p41);
            p41.seq = (ushort)(ushort)39846;
            p41.target_component = (byte)(byte)77;
            p41.target_system = (byte)(byte)254;
            ADV_TEST_CH.send(p41);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_CURRENTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (ushort)(ushort)19919);
            };
            MISSION_CURRENT p42 = new MISSION_CURRENT();
            PH.setPack(p42);
            p42.seq = (ushort)(ushort)19919;
            ADV_TEST_CH.send(p42);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)43);
                Debug.Assert(pack.target_component == (byte)(byte)86);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
            };
            MISSION_REQUEST_LIST p43 = new MISSION_REQUEST_LIST();
            PH.setPack(p43);
            p43.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL;
            p43.target_system = (byte)(byte)43;
            p43.target_component = (byte)(byte)86;
            ADV_TEST_CH.send(p43);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_COUNTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.count == (ushort)(ushort)11423);
                Debug.Assert(pack.target_system == (byte)(byte)135);
                Debug.Assert(pack.target_component == (byte)(byte)161);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
            };
            MISSION_COUNT p44 = new MISSION_COUNT();
            PH.setPack(p44);
            p44.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY;
            p44.count = (ushort)(ushort)11423;
            p44.target_component = (byte)(byte)161;
            p44.target_system = (byte)(byte)135;
            ADV_TEST_CH.send(p44);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_CLEAR_ALLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
                Debug.Assert(pack.target_component == (byte)(byte)41);
                Debug.Assert(pack.target_system == (byte)(byte)250);
            };
            MISSION_CLEAR_ALL p45 = new MISSION_CLEAR_ALL();
            PH.setPack(p45);
            p45.target_component = (byte)(byte)41;
            p45.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE;
            p45.target_system = (byte)(byte)250;
            ADV_TEST_CH.send(p45);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_ITEM_REACHEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (ushort)(ushort)39647);
            };
            MISSION_ITEM_REACHED p46 = new MISSION_ITEM_REACHED();
            PH.setPack(p46);
            p46.seq = (ushort)(ushort)39647;
            ADV_TEST_CH.send(p46);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)44);
                Debug.Assert(pack.type == MAV_MISSION_RESULT.MAV_MISSION_INVALID_PARAM2);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
                Debug.Assert(pack.target_component == (byte)(byte)253);
            };
            MISSION_ACK p47 = new MISSION_ACK();
            PH.setPack(p47);
            p47.target_component = (byte)(byte)253;
            p47.type = MAV_MISSION_RESULT.MAV_MISSION_INVALID_PARAM2;
            p47.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL;
            p47.target_system = (byte)(byte)44;
            ADV_TEST_CH.send(p47);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_GPS_GLOBAL_ORIGINReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)87095709183248624L);
                Debug.Assert(pack.latitude == (int)1450234698);
                Debug.Assert(pack.altitude == (int) -819758621);
                Debug.Assert(pack.target_system == (byte)(byte)221);
                Debug.Assert(pack.longitude == (int) -2034225112);
            };
            SET_GPS_GLOBAL_ORIGIN p48 = new SET_GPS_GLOBAL_ORIGIN();
            PH.setPack(p48);
            p48.time_usec_SET((ulong)87095709183248624L, PH) ;
            p48.altitude = (int) -819758621;
            p48.target_system = (byte)(byte)221;
            p48.latitude = (int)1450234698;
            p48.longitude = (int) -2034225112;
            ADV_TEST_CH.send(p48);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_GLOBAL_ORIGINReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.latitude == (int) -795177606);
                Debug.Assert(pack.longitude == (int)1612361903);
                Debug.Assert(pack.altitude == (int)2136445586);
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)5188566925037213500L);
            };
            GPS_GLOBAL_ORIGIN p49 = new GPS_GLOBAL_ORIGIN();
            PH.setPack(p49);
            p49.time_usec_SET((ulong)5188566925037213500L, PH) ;
            p49.longitude = (int)1612361903;
            p49.latitude = (int) -795177606;
            p49.altitude = (int)2136445586;
            ADV_TEST_CH.send(p49);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_MAP_RCReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)130);
                Debug.Assert(pack.target_component == (byte)(byte)180);
                Debug.Assert(pack.param_value0 == (float) -8.4996196E37F);
                Debug.Assert(pack.param_value_max == (float) -1.4374475E38F);
                Debug.Assert(pack.param_index == (short)(short)25299);
                Debug.Assert(pack.scale == (float) -9.2119E37F);
                Debug.Assert(pack.param_value_min == (float)1.3299907E38F);
                Debug.Assert(pack.param_id_LEN(ph) == 7);
                Debug.Assert(pack.param_id_TRY(ph).Equals("aoRqpsu"));
                Debug.Assert(pack.parameter_rc_channel_index == (byte)(byte)40);
            };
            PARAM_MAP_RC p50 = new PARAM_MAP_RC();
            PH.setPack(p50);
            p50.scale = (float) -9.2119E37F;
            p50.param_id_SET("aoRqpsu", PH) ;
            p50.param_value0 = (float) -8.4996196E37F;
            p50.param_value_min = (float)1.3299907E38F;
            p50.parameter_rc_channel_index = (byte)(byte)40;
            p50.param_value_max = (float) -1.4374475E38F;
            p50.param_index = (short)(short)25299;
            p50.target_system = (byte)(byte)130;
            p50.target_component = (byte)(byte)180;
            ADV_TEST_CH.send(p50);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_REQUEST_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)114);
                Debug.Assert(pack.target_system == (byte)(byte)19);
                Debug.Assert(pack.seq == (ushort)(ushort)21216);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
            };
            MISSION_REQUEST_INT p51 = new MISSION_REQUEST_INT();
            PH.setPack(p51);
            p51.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE;
            p51.target_component = (byte)(byte)114;
            p51.seq = (ushort)(ushort)21216;
            p51.target_system = (byte)(byte)19;
            ADV_TEST_CH.send(p51);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSAFETY_SET_ALLOWED_AREAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.p1y == (float)1.0347334E38F);
                Debug.Assert(pack.p2x == (float)2.4612065E38F);
                Debug.Assert(pack.p1x == (float) -3.0741087E37F);
                Debug.Assert(pack.p1z == (float) -1.2862974E38F);
                Debug.Assert(pack.target_system == (byte)(byte)233);
                Debug.Assert(pack.frame == MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED);
                Debug.Assert(pack.p2y == (float)7.318586E37F);
                Debug.Assert(pack.target_component == (byte)(byte)47);
                Debug.Assert(pack.p2z == (float) -2.9216497E38F);
            };
            SAFETY_SET_ALLOWED_AREA p54 = new SAFETY_SET_ALLOWED_AREA();
            PH.setPack(p54);
            p54.p1x = (float) -3.0741087E37F;
            p54.frame = MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED;
            p54.p2x = (float)2.4612065E38F;
            p54.p1z = (float) -1.2862974E38F;
            p54.p1y = (float)1.0347334E38F;
            p54.target_component = (byte)(byte)47;
            p54.p2y = (float)7.318586E37F;
            p54.target_system = (byte)(byte)233;
            p54.p2z = (float) -2.9216497E38F;
            ADV_TEST_CH.send(p54);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSAFETY_ALLOWED_AREAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.p1z == (float)1.2066294E38F);
                Debug.Assert(pack.p2y == (float) -2.622978E38F);
                Debug.Assert(pack.frame == MAV_FRAME.MAV_FRAME_LOCAL_NED);
                Debug.Assert(pack.p2x == (float)8.756221E37F);
                Debug.Assert(pack.p1y == (float)3.341641E38F);
                Debug.Assert(pack.p2z == (float)2.2925522E38F);
                Debug.Assert(pack.p1x == (float) -2.4000118E38F);
            };
            SAFETY_ALLOWED_AREA p55 = new SAFETY_ALLOWED_AREA();
            PH.setPack(p55);
            p55.p1y = (float)3.341641E38F;
            p55.p2x = (float)8.756221E37F;
            p55.p2y = (float) -2.622978E38F;
            p55.p1x = (float) -2.4000118E38F;
            p55.p2z = (float)2.2925522E38F;
            p55.p1z = (float)1.2066294E38F;
            p55.frame = MAV_FRAME.MAV_FRAME_LOCAL_NED;
            ADV_TEST_CH.send(p55);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATTITUDE_QUATERNION_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.q.SequenceEqual(new float[] {-1.112912E38F, 1.2321756E38F, -1.7471773E38F, 1.5484153E38F}));
                Debug.Assert(pack.rollspeed == (float) -1.5401266E37F);
                Debug.Assert(pack.yawspeed == (float)4.8721826E37F);
                Debug.Assert(pack.covariance.SequenceEqual(new float[] {2.872723E38F, -1.7946034E38F, -2.701923E38F, 1.6439307E38F, -2.8489829E38F, -3.0847987E38F, -1.0054807E38F, 1.8417201E37F, 3.0165073E38F}));
                Debug.Assert(pack.pitchspeed == (float) -2.8487362E38F);
                Debug.Assert(pack.time_usec == (ulong)4058701953654332505L);
            };
            ATTITUDE_QUATERNION_COV p61 = new ATTITUDE_QUATERNION_COV();
            PH.setPack(p61);
            p61.q_SET(new float[] {-1.112912E38F, 1.2321756E38F, -1.7471773E38F, 1.5484153E38F}, 0) ;
            p61.covariance_SET(new float[] {2.872723E38F, -1.7946034E38F, -2.701923E38F, 1.6439307E38F, -2.8489829E38F, -3.0847987E38F, -1.0054807E38F, 1.8417201E37F, 3.0165073E38F}, 0) ;
            p61.time_usec = (ulong)4058701953654332505L;
            p61.pitchspeed = (float) -2.8487362E38F;
            p61.rollspeed = (float) -1.5401266E37F;
            p61.yawspeed = (float)4.8721826E37F;
            ADV_TEST_CH.send(p61);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnNAV_CONTROLLER_OUTPUTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.nav_roll == (float) -2.82239E38F);
                Debug.Assert(pack.aspd_error == (float)3.3888426E38F);
                Debug.Assert(pack.nav_bearing == (short)(short) -9599);
                Debug.Assert(pack.xtrack_error == (float) -3.201828E37F);
                Debug.Assert(pack.target_bearing == (short)(short) -21577);
                Debug.Assert(pack.alt_error == (float)1.72233E38F);
                Debug.Assert(pack.wp_dist == (ushort)(ushort)58488);
                Debug.Assert(pack.nav_pitch == (float) -3.2078077E38F);
            };
            NAV_CONTROLLER_OUTPUT p62 = new NAV_CONTROLLER_OUTPUT();
            PH.setPack(p62);
            p62.nav_bearing = (short)(short) -9599;
            p62.xtrack_error = (float) -3.201828E37F;
            p62.nav_roll = (float) -2.82239E38F;
            p62.wp_dist = (ushort)(ushort)58488;
            p62.target_bearing = (short)(short) -21577;
            p62.nav_pitch = (float) -3.2078077E38F;
            p62.aspd_error = (float)3.3888426E38F;
            p62.alt_error = (float)1.72233E38F;
            ADV_TEST_CH.send(p62);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGLOBAL_POSITION_INT_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vx == (float) -7.8742754E37F);
                Debug.Assert(pack.lon == (int)1816836576);
                Debug.Assert(pack.estimator_type == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS_INS);
                Debug.Assert(pack.vz == (float) -1.0768426E38F);
                Debug.Assert(pack.relative_alt == (int)815458453);
                Debug.Assert(pack.covariance.SequenceEqual(new float[] {-1.8723114E38F, 2.972821E38F, -3.2643555E38F, 3.788888E37F, 3.1522532E38F, 1.7900052E36F, 2.1864421E38F, -6.47868E37F, 2.499016E38F, 1.4704778E38F, 3.0787661E38F, 2.4611866E38F, -3.1125558E38F, 2.363617E38F, -5.032238E37F, -1.2992313E38F, -6.779751E37F, -2.4687936E38F, 5.08702E37F, 1.8605529E38F, -4.0677815E37F, -4.629509E37F, -1.97528E38F, 5.196025E37F, 3.2460946E38F, 6.6246214E37F, -2.7554513E38F, -3.0262834E38F, -3.6846473E37F, 3.2335394E37F, 9.104208E37F, 9.343608E37F, -3.286369E38F, -1.5977508E38F, -2.6278644E38F, 8.773972E37F}));
                Debug.Assert(pack.lat == (int) -1064320285);
                Debug.Assert(pack.alt == (int)484951641);
                Debug.Assert(pack.time_usec == (ulong)1721691252679568386L);
                Debug.Assert(pack.vy == (float)2.8439064E38F);
            };
            GLOBAL_POSITION_INT_COV p63 = new GLOBAL_POSITION_INT_COV();
            PH.setPack(p63);
            p63.lon = (int)1816836576;
            p63.estimator_type = MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS_INS;
            p63.vx = (float) -7.8742754E37F;
            p63.time_usec = (ulong)1721691252679568386L;
            p63.vy = (float)2.8439064E38F;
            p63.lat = (int) -1064320285;
            p63.alt = (int)484951641;
            p63.vz = (float) -1.0768426E38F;
            p63.covariance_SET(new float[] {-1.8723114E38F, 2.972821E38F, -3.2643555E38F, 3.788888E37F, 3.1522532E38F, 1.7900052E36F, 2.1864421E38F, -6.47868E37F, 2.499016E38F, 1.4704778E38F, 3.0787661E38F, 2.4611866E38F, -3.1125558E38F, 2.363617E38F, -5.032238E37F, -1.2992313E38F, -6.779751E37F, -2.4687936E38F, 5.08702E37F, 1.8605529E38F, -4.0677815E37F, -4.629509E37F, -1.97528E38F, 5.196025E37F, 3.2460946E38F, 6.6246214E37F, -2.7554513E38F, -3.0262834E38F, -3.6846473E37F, 3.2335394E37F, 9.104208E37F, 9.343608E37F, -3.286369E38F, -1.5977508E38F, -2.6278644E38F, 8.773972E37F}, 0) ;
            p63.relative_alt = (int)815458453;
            ADV_TEST_CH.send(p63);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOCAL_POSITION_NED_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vz == (float)3.1954468E38F);
                Debug.Assert(pack.vx == (float) -1.4871632E38F);
                Debug.Assert(pack.y == (float)1.448968E38F);
                Debug.Assert(pack.covariance.SequenceEqual(new float[] {-3.135781E37F, -1.3580129E38F, 1.2288571E38F, 2.8975919E38F, -2.9922813E38F, 3.3327664E38F, 2.6438334E38F, -3.0710635E38F, 1.3918775E38F, -1.655711E38F, 5.27199E37F, -9.944889E37F, -1.8469103E38F, 1.3577063E38F, -6.031943E36F, -1.4373185E38F, -2.601565E38F, 2.9110245E38F, 1.6624737E38F, -4.0352947E36F, 2.8278843E38F, -2.4351428E38F, 2.8255762E38F, 2.6887559E38F, 2.023826E38F, 3.1235708E38F, 3.2065278E37F, 2.6779099E38F, -2.172256E38F, 7.473335E37F, 3.3771155E38F, 1.7097653E38F, 2.8590638E38F, -4.3325595E37F, 2.7980225E38F, -2.4216913E38F, 1.1221799E38F, 8.676564E37F, 1.6296428E38F, -2.0767441E38F, -3.3617386E38F, -1.4687592E37F, 3.374433E38F, -5.3174206E37F, -1.5384394E38F}));
                Debug.Assert(pack.vy == (float)5.5702063E37F);
                Debug.Assert(pack.z == (float)1.3926186E38F);
                Debug.Assert(pack.az == (float) -5.185827E37F);
                Debug.Assert(pack.estimator_type == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS_INS);
                Debug.Assert(pack.x == (float) -9.39206E37F);
                Debug.Assert(pack.ay == (float) -2.2260263E38F);
                Debug.Assert(pack.time_usec == (ulong)1650504021740510155L);
                Debug.Assert(pack.ax == (float)6.973455E37F);
            };
            LOCAL_POSITION_NED_COV p64 = new LOCAL_POSITION_NED_COV();
            PH.setPack(p64);
            p64.time_usec = (ulong)1650504021740510155L;
            p64.vy = (float)5.5702063E37F;
            p64.x = (float) -9.39206E37F;
            p64.covariance_SET(new float[] {-3.135781E37F, -1.3580129E38F, 1.2288571E38F, 2.8975919E38F, -2.9922813E38F, 3.3327664E38F, 2.6438334E38F, -3.0710635E38F, 1.3918775E38F, -1.655711E38F, 5.27199E37F, -9.944889E37F, -1.8469103E38F, 1.3577063E38F, -6.031943E36F, -1.4373185E38F, -2.601565E38F, 2.9110245E38F, 1.6624737E38F, -4.0352947E36F, 2.8278843E38F, -2.4351428E38F, 2.8255762E38F, 2.6887559E38F, 2.023826E38F, 3.1235708E38F, 3.2065278E37F, 2.6779099E38F, -2.172256E38F, 7.473335E37F, 3.3771155E38F, 1.7097653E38F, 2.8590638E38F, -4.3325595E37F, 2.7980225E38F, -2.4216913E38F, 1.1221799E38F, 8.676564E37F, 1.6296428E38F, -2.0767441E38F, -3.3617386E38F, -1.4687592E37F, 3.374433E38F, -5.3174206E37F, -1.5384394E38F}, 0) ;
            p64.estimator_type = MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS_INS;
            p64.ay = (float) -2.2260263E38F;
            p64.vz = (float)3.1954468E38F;
            p64.y = (float)1.448968E38F;
            p64.az = (float) -5.185827E37F;
            p64.vx = (float) -1.4871632E38F;
            p64.ax = (float)6.973455E37F;
            p64.z = (float)1.3926186E38F;
            ADV_TEST_CH.send(p64);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRC_CHANNELSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)1034068826U);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)59328);
                Debug.Assert(pack.chan12_raw == (ushort)(ushort)3873);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)52152);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)31096);
                Debug.Assert(pack.chan17_raw == (ushort)(ushort)46160);
                Debug.Assert(pack.chan15_raw == (ushort)(ushort)20096);
                Debug.Assert(pack.chan14_raw == (ushort)(ushort)38429);
                Debug.Assert(pack.chan16_raw == (ushort)(ushort)3294);
                Debug.Assert(pack.rssi == (byte)(byte)141);
                Debug.Assert(pack.chan13_raw == (ushort)(ushort)31497);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)14416);
                Debug.Assert(pack.chancount == (byte)(byte)136);
                Debug.Assert(pack.chan18_raw == (ushort)(ushort)27314);
                Debug.Assert(pack.chan9_raw == (ushort)(ushort)25685);
                Debug.Assert(pack.chan11_raw == (ushort)(ushort)58521);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)21797);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)53753);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)21456);
                Debug.Assert(pack.chan10_raw == (ushort)(ushort)35348);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)13092);
            };
            RC_CHANNELS p65 = new RC_CHANNELS();
            PH.setPack(p65);
            p65.chan17_raw = (ushort)(ushort)46160;
            p65.chan10_raw = (ushort)(ushort)35348;
            p65.chan18_raw = (ushort)(ushort)27314;
            p65.chan1_raw = (ushort)(ushort)21797;
            p65.chan15_raw = (ushort)(ushort)20096;
            p65.chan2_raw = (ushort)(ushort)59328;
            p65.rssi = (byte)(byte)141;
            p65.chancount = (byte)(byte)136;
            p65.chan3_raw = (ushort)(ushort)53753;
            p65.chan5_raw = (ushort)(ushort)21456;
            p65.chan7_raw = (ushort)(ushort)52152;
            p65.chan4_raw = (ushort)(ushort)14416;
            p65.time_boot_ms = (uint)1034068826U;
            p65.chan6_raw = (ushort)(ushort)31096;
            p65.chan12_raw = (ushort)(ushort)3873;
            p65.chan16_raw = (ushort)(ushort)3294;
            p65.chan8_raw = (ushort)(ushort)13092;
            p65.chan11_raw = (ushort)(ushort)58521;
            p65.chan9_raw = (ushort)(ushort)25685;
            p65.chan13_raw = (ushort)(ushort)31497;
            p65.chan14_raw = (ushort)(ushort)38429;
            ADV_TEST_CH.send(p65);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnREQUEST_DATA_STREAMReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.start_stop == (byte)(byte)137);
                Debug.Assert(pack.req_stream_id == (byte)(byte)201);
                Debug.Assert(pack.target_component == (byte)(byte)203);
                Debug.Assert(pack.target_system == (byte)(byte)213);
                Debug.Assert(pack.req_message_rate == (ushort)(ushort)14401);
            };
            REQUEST_DATA_STREAM p66 = new REQUEST_DATA_STREAM();
            PH.setPack(p66);
            p66.req_message_rate = (ushort)(ushort)14401;
            p66.target_system = (byte)(byte)213;
            p66.req_stream_id = (byte)(byte)201;
            p66.target_component = (byte)(byte)203;
            p66.start_stop = (byte)(byte)137;
            ADV_TEST_CH.send(p66);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnDATA_STREAMReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.message_rate == (ushort)(ushort)21665);
                Debug.Assert(pack.on_off == (byte)(byte)74);
                Debug.Assert(pack.stream_id == (byte)(byte)29);
            };
            DATA_STREAM p67 = new DATA_STREAM();
            PH.setPack(p67);
            p67.on_off = (byte)(byte)74;
            p67.stream_id = (byte)(byte)29;
            p67.message_rate = (ushort)(ushort)21665;
            ADV_TEST_CH.send(p67);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMANUAL_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.x == (short)(short)29178);
                Debug.Assert(pack.target == (byte)(byte)128);
                Debug.Assert(pack.y == (short)(short) -365);
                Debug.Assert(pack.buttons == (ushort)(ushort)63626);
                Debug.Assert(pack.z == (short)(short) -8605);
                Debug.Assert(pack.r == (short)(short)13352);
            };
            MANUAL_CONTROL p69 = new MANUAL_CONTROL();
            PH.setPack(p69);
            p69.buttons = (ushort)(ushort)63626;
            p69.target = (byte)(byte)128;
            p69.x = (short)(short)29178;
            p69.z = (short)(short) -8605;
            p69.r = (short)(short)13352;
            p69.y = (short)(short) -365;
            ADV_TEST_CH.send(p69);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRC_CHANNELS_OVERRIDEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)129);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)5663);
                Debug.Assert(pack.target_system == (byte)(byte)210);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)34323);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)57107);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)38131);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)33913);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)13447);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)5972);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)34122);
            };
            RC_CHANNELS_OVERRIDE p70 = new RC_CHANNELS_OVERRIDE();
            PH.setPack(p70);
            p70.target_system = (byte)(byte)210;
            p70.chan4_raw = (ushort)(ushort)13447;
            p70.chan2_raw = (ushort)(ushort)34122;
            p70.chan5_raw = (ushort)(ushort)5972;
            p70.target_component = (byte)(byte)129;
            p70.chan6_raw = (ushort)(ushort)57107;
            p70.chan3_raw = (ushort)(ushort)33913;
            p70.chan7_raw = (ushort)(ushort)38131;
            p70.chan8_raw = (ushort)(ushort)5663;
            p70.chan1_raw = (ushort)(ushort)34323;
            ADV_TEST_CH.send(p70);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_ITEM_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.autocontinue == (byte)(byte)134);
                Debug.Assert(pack.frame == MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT);
                Debug.Assert(pack.x == (int) -717109753);
                Debug.Assert(pack.param1 == (float) -1.3630487E38F);
                Debug.Assert(pack.param4 == (float) -1.2466553E38F);
                Debug.Assert(pack.param2 == (float) -6.7424667E37F);
                Debug.Assert(pack.param3 == (float)3.0470343E38F);
                Debug.Assert(pack.command == MAV_CMD.MAV_CMD_WAYPOINT_USER_4);
                Debug.Assert(pack.target_component == (byte)(byte)161);
                Debug.Assert(pack.seq == (ushort)(ushort)46491);
                Debug.Assert(pack.target_system == (byte)(byte)111);
                Debug.Assert(pack.current == (byte)(byte)133);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
                Debug.Assert(pack.y == (int) -482962015);
                Debug.Assert(pack.z == (float)3.3418917E38F);
            };
            MISSION_ITEM_INT p73 = new MISSION_ITEM_INT();
            PH.setPack(p73);
            p73.param3 = (float)3.0470343E38F;
            p73.param1 = (float) -1.3630487E38F;
            p73.current = (byte)(byte)133;
            p73.y = (int) -482962015;
            p73.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE;
            p73.frame = MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT;
            p73.target_component = (byte)(byte)161;
            p73.param2 = (float) -6.7424667E37F;
            p73.param4 = (float) -1.2466553E38F;
            p73.autocontinue = (byte)(byte)134;
            p73.seq = (ushort)(ushort)46491;
            p73.z = (float)3.3418917E38F;
            p73.target_system = (byte)(byte)111;
            p73.command = MAV_CMD.MAV_CMD_WAYPOINT_USER_4;
            p73.x = (int) -717109753;
            ADV_TEST_CH.send(p73);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnVFR_HUDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.heading == (short)(short) -18693);
                Debug.Assert(pack.climb == (float)2.3760557E38F);
                Debug.Assert(pack.alt == (float)1.514005E38F);
                Debug.Assert(pack.airspeed == (float)1.0283593E38F);
                Debug.Assert(pack.groundspeed == (float)2.6070383E37F);
                Debug.Assert(pack.throttle == (ushort)(ushort)56137);
            };
            VFR_HUD p74 = new VFR_HUD();
            PH.setPack(p74);
            p74.alt = (float)1.514005E38F;
            p74.climb = (float)2.3760557E38F;
            p74.heading = (short)(short) -18693;
            p74.airspeed = (float)1.0283593E38F;
            p74.throttle = (ushort)(ushort)56137;
            p74.groundspeed = (float)2.6070383E37F;
            ADV_TEST_CH.send(p74);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCOMMAND_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.autocontinue == (byte)(byte)21);
                Debug.Assert(pack.command == MAV_CMD.MAV_CMD_SPATIAL_USER_2);
                Debug.Assert(pack.y == (int)1259137291);
                Debug.Assert(pack.param2 == (float)2.0153235E38F);
                Debug.Assert(pack.target_component == (byte)(byte)198);
                Debug.Assert(pack.current == (byte)(byte)195);
                Debug.Assert(pack.frame == MAV_FRAME.MAV_FRAME_MISSION);
                Debug.Assert(pack.z == (float)1.0195571E38F);
                Debug.Assert(pack.param3 == (float) -1.6049961E38F);
                Debug.Assert(pack.x == (int)212558523);
                Debug.Assert(pack.target_system == (byte)(byte)66);
                Debug.Assert(pack.param4 == (float)3.273763E38F);
                Debug.Assert(pack.param1 == (float)2.2662978E38F);
            };
            COMMAND_INT p75 = new COMMAND_INT();
            PH.setPack(p75);
            p75.target_component = (byte)(byte)198;
            p75.command = MAV_CMD.MAV_CMD_SPATIAL_USER_2;
            p75.param4 = (float)3.273763E38F;
            p75.param1 = (float)2.2662978E38F;
            p75.z = (float)1.0195571E38F;
            p75.frame = MAV_FRAME.MAV_FRAME_MISSION;
            p75.current = (byte)(byte)195;
            p75.target_system = (byte)(byte)66;
            p75.autocontinue = (byte)(byte)21;
            p75.y = (int)1259137291;
            p75.param2 = (float)2.0153235E38F;
            p75.param3 = (float) -1.6049961E38F;
            p75.x = (int)212558523;
            ADV_TEST_CH.send(p75);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCOMMAND_LONGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param7 == (float)1.891453E38F);
                Debug.Assert(pack.param3 == (float)3.8122693E37F);
                Debug.Assert(pack.target_component == (byte)(byte)175);
                Debug.Assert(pack.param5 == (float)2.6328529E38F);
                Debug.Assert(pack.target_system == (byte)(byte)124);
                Debug.Assert(pack.command == MAV_CMD.MAV_CMD_SET_MESSAGE_INTERVAL);
                Debug.Assert(pack.param6 == (float)8.788215E35F);
                Debug.Assert(pack.param1 == (float)2.9539204E38F);
                Debug.Assert(pack.param2 == (float) -1.8764953E38F);
                Debug.Assert(pack.confirmation == (byte)(byte)101);
                Debug.Assert(pack.param4 == (float) -9.725696E36F);
            };
            COMMAND_LONG p76 = new COMMAND_LONG();
            PH.setPack(p76);
            p76.param3 = (float)3.8122693E37F;
            p76.param4 = (float) -9.725696E36F;
            p76.target_system = (byte)(byte)124;
            p76.param6 = (float)8.788215E35F;
            p76.confirmation = (byte)(byte)101;
            p76.param1 = (float)2.9539204E38F;
            p76.target_component = (byte)(byte)175;
            p76.param2 = (float) -1.8764953E38F;
            p76.command = MAV_CMD.MAV_CMD_SET_MESSAGE_INTERVAL;
            p76.param7 = (float)1.891453E38F;
            p76.param5 = (float)2.6328529E38F;
            ADV_TEST_CH.send(p76);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCOMMAND_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.progress_TRY(ph) == (byte)(byte)99);
                Debug.Assert(pack.target_component_TRY(ph) == (byte)(byte)151);
                Debug.Assert(pack.result_param2_TRY(ph) == (int) -1393497926);
                Debug.Assert(pack.result == MAV_RESULT.MAV_RESULT_IN_PROGRESS);
                Debug.Assert(pack.target_system_TRY(ph) == (byte)(byte)101);
                Debug.Assert(pack.command == MAV_CMD.MAV_CMD_NAV_PAYLOAD_PLACE);
            };
            COMMAND_ACK p77 = new COMMAND_ACK();
            PH.setPack(p77);
            p77.progress_SET((byte)(byte)99, PH) ;
            p77.target_component_SET((byte)(byte)151, PH) ;
            p77.target_system_SET((byte)(byte)101, PH) ;
            p77.result = MAV_RESULT.MAV_RESULT_IN_PROGRESS;
            p77.command = MAV_CMD.MAV_CMD_NAV_PAYLOAD_PLACE;
            p77.result_param2_SET((int) -1393497926, PH) ;
            ADV_TEST_CH.send(p77);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMANUAL_SETPOINTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.roll == (float)2.1236602E38F);
                Debug.Assert(pack.mode_switch == (byte)(byte)209);
                Debug.Assert(pack.manual_override_switch == (byte)(byte)180);
                Debug.Assert(pack.yaw == (float)1.6898062E38F);
                Debug.Assert(pack.thrust == (float) -3.3631046E38F);
                Debug.Assert(pack.time_boot_ms == (uint)2240056140U);
                Debug.Assert(pack.pitch == (float) -6.5873783E37F);
            };
            MANUAL_SETPOINT p81 = new MANUAL_SETPOINT();
            PH.setPack(p81);
            p81.manual_override_switch = (byte)(byte)180;
            p81.yaw = (float)1.6898062E38F;
            p81.roll = (float)2.1236602E38F;
            p81.pitch = (float) -6.5873783E37F;
            p81.thrust = (float) -3.3631046E38F;
            p81.time_boot_ms = (uint)2240056140U;
            p81.mode_switch = (byte)(byte)209;
            ADV_TEST_CH.send(p81);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_ATTITUDE_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.body_yaw_rate == (float)1.2993564E38F);
                Debug.Assert(pack.target_system == (byte)(byte)43);
                Debug.Assert(pack.thrust == (float)1.0971585E38F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-3.3505026E38F, -9.918084E37F, -2.0157963E38F, -8.520458E37F}));
                Debug.Assert(pack.time_boot_ms == (uint)4207244853U);
                Debug.Assert(pack.body_roll_rate == (float)3.0049388E38F);
                Debug.Assert(pack.body_pitch_rate == (float) -1.0714001E38F);
                Debug.Assert(pack.type_mask == (byte)(byte)41);
                Debug.Assert(pack.target_component == (byte)(byte)88);
            };
            SET_ATTITUDE_TARGET p82 = new SET_ATTITUDE_TARGET();
            PH.setPack(p82);
            p82.target_system = (byte)(byte)43;
            p82.type_mask = (byte)(byte)41;
            p82.body_yaw_rate = (float)1.2993564E38F;
            p82.time_boot_ms = (uint)4207244853U;
            p82.thrust = (float)1.0971585E38F;
            p82.body_pitch_rate = (float) -1.0714001E38F;
            p82.body_roll_rate = (float)3.0049388E38F;
            p82.q_SET(new float[] {-3.3505026E38F, -9.918084E37F, -2.0157963E38F, -8.520458E37F}, 0) ;
            p82.target_component = (byte)(byte)88;
            ADV_TEST_CH.send(p82);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATTITUDE_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.type_mask == (byte)(byte)148);
                Debug.Assert(pack.time_boot_ms == (uint)1830936483U);
                Debug.Assert(pack.body_pitch_rate == (float)2.1555687E38F);
                Debug.Assert(pack.thrust == (float) -2.0898157E38F);
                Debug.Assert(pack.body_roll_rate == (float)1.3350352E38F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {1.3146246E38F, -1.2848659E38F, -2.468572E38F, -3.0388804E38F}));
                Debug.Assert(pack.body_yaw_rate == (float) -1.8060311E38F);
            };
            ATTITUDE_TARGET p83 = new ATTITUDE_TARGET();
            PH.setPack(p83);
            p83.body_roll_rate = (float)1.3350352E38F;
            p83.thrust = (float) -2.0898157E38F;
            p83.body_yaw_rate = (float) -1.8060311E38F;
            p83.time_boot_ms = (uint)1830936483U;
            p83.body_pitch_rate = (float)2.1555687E38F;
            p83.type_mask = (byte)(byte)148;
            p83.q_SET(new float[] {1.3146246E38F, -1.2848659E38F, -2.468572E38F, -3.0388804E38F}, 0) ;
            ADV_TEST_CH.send(p83);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_POSITION_TARGET_LOCAL_NEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vz == (float)7.198566E37F);
                Debug.Assert(pack.vx == (float) -4.3304897E37F);
                Debug.Assert(pack.time_boot_ms == (uint)2119091725U);
                Debug.Assert(pack.vy == (float)1.7156266E38F);
                Debug.Assert(pack.target_component == (byte)(byte)152);
                Debug.Assert(pack.afz == (float)1.974898E38F);
                Debug.Assert(pack.y == (float)2.477482E37F);
                Debug.Assert(pack.type_mask == (ushort)(ushort)54164);
                Debug.Assert(pack.yaw_rate == (float)2.096056E38F);
                Debug.Assert(pack.x == (float) -7.6844954E37F);
                Debug.Assert(pack.yaw == (float)1.6905538E38F);
                Debug.Assert(pack.afy == (float)6.4280666E37F);
                Debug.Assert(pack.afx == (float)1.3861634E38F);
                Debug.Assert(pack.target_system == (byte)(byte)248);
                Debug.Assert(pack.z == (float) -1.8226224E38F);
                Debug.Assert(pack.coordinate_frame == MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT);
            };
            SET_POSITION_TARGET_LOCAL_NED p84 = new SET_POSITION_TARGET_LOCAL_NED();
            PH.setPack(p84);
            p84.afx = (float)1.3861634E38F;
            p84.time_boot_ms = (uint)2119091725U;
            p84.afy = (float)6.4280666E37F;
            p84.x = (float) -7.6844954E37F;
            p84.vz = (float)7.198566E37F;
            p84.z = (float) -1.8226224E38F;
            p84.vx = (float) -4.3304897E37F;
            p84.yaw_rate = (float)2.096056E38F;
            p84.y = (float)2.477482E37F;
            p84.vy = (float)1.7156266E38F;
            p84.type_mask = (ushort)(ushort)54164;
            p84.target_component = (byte)(byte)152;
            p84.afz = (float)1.974898E38F;
            p84.target_system = (byte)(byte)248;
            p84.coordinate_frame = MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT;
            p84.yaw = (float)1.6905538E38F;
            ADV_TEST_CH.send(p84);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_POSITION_TARGET_GLOBAL_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lat_int == (int)99912697);
                Debug.Assert(pack.coordinate_frame == MAV_FRAME.MAV_FRAME_LOCAL_NED);
                Debug.Assert(pack.afy == (float) -7.4161885E37F);
                Debug.Assert(pack.vy == (float) -1.3986302E37F);
                Debug.Assert(pack.alt == (float)2.3871378E38F);
                Debug.Assert(pack.vz == (float)1.2885307E38F);
                Debug.Assert(pack.yaw_rate == (float)2.8400525E38F);
                Debug.Assert(pack.afz == (float) -1.1336224E38F);
                Debug.Assert(pack.lon_int == (int)1911673439);
                Debug.Assert(pack.target_component == (byte)(byte)216);
                Debug.Assert(pack.afx == (float)8.743184E37F);
                Debug.Assert(pack.target_system == (byte)(byte)64);
                Debug.Assert(pack.time_boot_ms == (uint)2545327297U);
                Debug.Assert(pack.yaw == (float)3.4655625E37F);
                Debug.Assert(pack.type_mask == (ushort)(ushort)1272);
                Debug.Assert(pack.vx == (float)8.0211535E37F);
            };
            SET_POSITION_TARGET_GLOBAL_INT p86 = new SET_POSITION_TARGET_GLOBAL_INT();
            PH.setPack(p86);
            p86.afy = (float) -7.4161885E37F;
            p86.type_mask = (ushort)(ushort)1272;
            p86.vz = (float)1.2885307E38F;
            p86.lat_int = (int)99912697;
            p86.afx = (float)8.743184E37F;
            p86.lon_int = (int)1911673439;
            p86.vx = (float)8.0211535E37F;
            p86.yaw = (float)3.4655625E37F;
            p86.afz = (float) -1.1336224E38F;
            p86.coordinate_frame = MAV_FRAME.MAV_FRAME_LOCAL_NED;
            p86.vy = (float) -1.3986302E37F;
            p86.alt = (float)2.3871378E38F;
            p86.time_boot_ms = (uint)2545327297U;
            p86.target_system = (byte)(byte)64;
            p86.target_component = (byte)(byte)216;
            p86.yaw_rate = (float)2.8400525E38F;
            ADV_TEST_CH.send(p86);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPOSITION_TARGET_GLOBAL_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.yaw_rate == (float)4.6904286E36F);
                Debug.Assert(pack.lon_int == (int)2015715759);
                Debug.Assert(pack.vx == (float) -2.9619488E38F);
                Debug.Assert(pack.yaw == (float)2.6563517E38F);
                Debug.Assert(pack.time_boot_ms == (uint)2988237977U);
                Debug.Assert(pack.afx == (float) -2.6435219E38F);
                Debug.Assert(pack.alt == (float)2.0518345E38F);
                Debug.Assert(pack.type_mask == (ushort)(ushort)43673);
                Debug.Assert(pack.vz == (float) -1.3560616E38F);
                Debug.Assert(pack.coordinate_frame == MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT);
                Debug.Assert(pack.vy == (float)1.7849183E37F);
                Debug.Assert(pack.lat_int == (int)1674927350);
                Debug.Assert(pack.afy == (float) -1.910792E38F);
                Debug.Assert(pack.afz == (float)2.672451E38F);
            };
            GroundControl.POSITION_TARGET_GLOBAL_INT p87 = CommunicationChannel.new_POSITION_TARGET_GLOBAL_INT();
            PH.setPack(p87);
            p87.vy = (float)1.7849183E37F;
            p87.alt = (float)2.0518345E38F;
            p87.coordinate_frame = MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT;
            p87.vz = (float) -1.3560616E38F;
            p87.afx = (float) -2.6435219E38F;
            p87.vx = (float) -2.9619488E38F;
            p87.yaw_rate = (float)4.6904286E36F;
            p87.type_mask = (ushort)(ushort)43673;
            p87.yaw = (float)2.6563517E38F;
            p87.lon_int = (int)2015715759;
            p87.afz = (float)2.672451E38F;
            p87.afy = (float) -1.910792E38F;
            p87.lat_int = (int)1674927350;
            p87.time_boot_ms = (uint)2988237977U;
            CommunicationChannel.instance.send(p87);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.x == (float) -2.9739014E38F);
                Debug.Assert(pack.yaw == (float) -1.4673464E38F);
                Debug.Assert(pack.roll == (float) -2.681323E38F);
                Debug.Assert(pack.y == (float) -1.0763277E38F);
                Debug.Assert(pack.time_boot_ms == (uint)4122332151U);
                Debug.Assert(pack.z == (float) -2.7188556E38F);
                Debug.Assert(pack.pitch == (float)6.3216863E37F);
            };
            GroundControl.LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET p89 = CommunicationChannel.new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
            PH.setPack(p89);
            p89.z = (float) -2.7188556E38F;
            p89.yaw = (float) -1.4673464E38F;
            p89.pitch = (float)6.3216863E37F;
            p89.roll = (float) -2.681323E38F;
            p89.time_boot_ms = (uint)4122332151U;
            p89.x = (float) -2.9739014E38F;
            p89.y = (float) -1.0763277E38F;
            CommunicationChannel.instance.send(p89);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)5187443801947039138L);
                Debug.Assert(pack.alt == (int) -809404032);
                Debug.Assert(pack.vz == (short)(short) -19238);
                Debug.Assert(pack.zacc == (short)(short) -27827);
                Debug.Assert(pack.pitch == (float)8.945211E37F);
                Debug.Assert(pack.vy == (short)(short)30211);
                Debug.Assert(pack.lat == (int)599413754);
                Debug.Assert(pack.lon == (int)1625791847);
                Debug.Assert(pack.yawspeed == (float) -2.4897668E38F);
                Debug.Assert(pack.xacc == (short)(short)9401);
                Debug.Assert(pack.rollspeed == (float)2.7185826E38F);
                Debug.Assert(pack.pitchspeed == (float)8.147785E37F);
                Debug.Assert(pack.yacc == (short)(short) -10775);
                Debug.Assert(pack.vx == (short)(short)2582);
                Debug.Assert(pack.roll == (float)1.7705779E38F);
                Debug.Assert(pack.yaw == (float)8.90666E37F);
            };
            GroundControl.HIL_STATE p90 = CommunicationChannel.new_HIL_STATE();
            PH.setPack(p90);
            p90.vz = (short)(short) -19238;
            p90.time_usec = (ulong)5187443801947039138L;
            p90.yaw = (float)8.90666E37F;
            p90.lat = (int)599413754;
            p90.xacc = (short)(short)9401;
            p90.lon = (int)1625791847;
            p90.pitch = (float)8.945211E37F;
            p90.yawspeed = (float) -2.4897668E38F;
            p90.vx = (short)(short)2582;
            p90.pitchspeed = (float)8.147785E37F;
            p90.zacc = (short)(short) -27827;
            p90.yacc = (short)(short) -10775;
            p90.roll = (float)1.7705779E38F;
            p90.vy = (short)(short)30211;
            p90.alt = (int) -809404032;
            p90.rollspeed = (float)2.7185826E38F;
            CommunicationChannel.instance.send(p90);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_CONTROLSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.aux3 == (float)3.158667E38F);
                Debug.Assert(pack.roll_ailerons == (float) -6.8355544E37F);
                Debug.Assert(pack.throttle == (float)3.0577185E38F);
                Debug.Assert(pack.pitch_elevator == (float)5.0723244E37F);
                Debug.Assert(pack.yaw_rudder == (float) -1.9871823E38F);
                Debug.Assert(pack.aux1 == (float) -1.8904953E37F);
                Debug.Assert(pack.aux2 == (float) -1.1771415E37F);
                Debug.Assert(pack.mode == MAV_MODE.MAV_MODE_TEST_ARMED);
                Debug.Assert(pack.nav_mode == (byte)(byte)24);
                Debug.Assert(pack.aux4 == (float) -4.832007E37F);
                Debug.Assert(pack.time_usec == (ulong)7359417118969223848L);
            };
            GroundControl.HIL_CONTROLS p91 = CommunicationChannel.new_HIL_CONTROLS();
            PH.setPack(p91);
            p91.mode = MAV_MODE.MAV_MODE_TEST_ARMED;
            p91.aux3 = (float)3.158667E38F;
            p91.aux1 = (float) -1.8904953E37F;
            p91.pitch_elevator = (float)5.0723244E37F;
            p91.roll_ailerons = (float) -6.8355544E37F;
            p91.aux2 = (float) -1.1771415E37F;
            p91.nav_mode = (byte)(byte)24;
            p91.yaw_rudder = (float) -1.9871823E38F;
            p91.aux4 = (float) -4.832007E37F;
            p91.time_usec = (ulong)7359417118969223848L;
            p91.throttle = (float)3.0577185E38F;
            CommunicationChannel.instance.send(p91);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_RC_INPUTS_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)7420);
                Debug.Assert(pack.chan10_raw == (ushort)(ushort)40377);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)8540);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)2515);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)11986);
                Debug.Assert(pack.time_usec == (ulong)4048775756530220329L);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)3489);
                Debug.Assert(pack.rssi == (byte)(byte)129);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)63304);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)31497);
                Debug.Assert(pack.chan12_raw == (ushort)(ushort)21061);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)53803);
                Debug.Assert(pack.chan11_raw == (ushort)(ushort)5047);
                Debug.Assert(pack.chan9_raw == (ushort)(ushort)36369);
            };
            GroundControl.HIL_RC_INPUTS_RAW p92 = CommunicationChannel.new_HIL_RC_INPUTS_RAW();
            PH.setPack(p92);
            p92.chan11_raw = (ushort)(ushort)5047;
            p92.chan7_raw = (ushort)(ushort)3489;
            p92.chan8_raw = (ushort)(ushort)53803;
            p92.chan6_raw = (ushort)(ushort)7420;
            p92.chan9_raw = (ushort)(ushort)36369;
            p92.chan12_raw = (ushort)(ushort)21061;
            p92.chan10_raw = (ushort)(ushort)40377;
            p92.chan4_raw = (ushort)(ushort)31497;
            p92.rssi = (byte)(byte)129;
            p92.time_usec = (ulong)4048775756530220329L;
            p92.chan5_raw = (ushort)(ushort)11986;
            p92.chan2_raw = (ushort)(ushort)8540;
            p92.chan3_raw = (ushort)(ushort)2515;
            p92.chan1_raw = (ushort)(ushort)63304;
            CommunicationChannel.instance.send(p92);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_ACTUATOR_CONTROLSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)3244920156064736643L);
                Debug.Assert(pack.controls.SequenceEqual(new float[] {8.717278E37F, -1.5952847E38F, 2.5544875E38F, 9.54547E37F, -3.0206722E38F, -8.455404E37F, 1.45719E38F, -1.7930277E38F, 1.4799845E38F, 2.4478157E38F, -1.7392996E38F, -1.4068496E38F, -2.5677936E37F, -1.9217526E38F, 8.9442073E36F, 9.908767E37F}));
                Debug.Assert(pack.mode == MAV_MODE.MAV_MODE_STABILIZE_ARMED);
                Debug.Assert(pack.flags == (ulong)818996083551491306L);
            };
            GroundControl.HIL_ACTUATOR_CONTROLS p93 = CommunicationChannel.new_HIL_ACTUATOR_CONTROLS();
            PH.setPack(p93);
            p93.mode = MAV_MODE.MAV_MODE_STABILIZE_ARMED;
            p93.time_usec = (ulong)3244920156064736643L;
            p93.controls_SET(new float[] {8.717278E37F, -1.5952847E38F, 2.5544875E38F, 9.54547E37F, -3.0206722E38F, -8.455404E37F, 1.45719E38F, -1.7930277E38F, 1.4799845E38F, 2.4478157E38F, -1.7392996E38F, -1.4068496E38F, -2.5677936E37F, -1.9217526E38F, 8.9442073E36F, 9.908767E37F}, 0) ;
            p93.flags = (ulong)818996083551491306L;
            CommunicationChannel.instance.send(p93);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnOPTICAL_FLOWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.flow_x == (short)(short)11558);
                Debug.Assert(pack.flow_comp_m_x == (float)4.6957875E37F);
                Debug.Assert(pack.ground_distance == (float)2.8046715E38F);
                Debug.Assert(pack.flow_rate_x_TRY(ph) == (float)1.017256E37F);
                Debug.Assert(pack.quality == (byte)(byte)203);
                Debug.Assert(pack.flow_y == (short)(short)21694);
                Debug.Assert(pack.time_usec == (ulong)8939872380095705532L);
                Debug.Assert(pack.flow_comp_m_y == (float) -2.9837065E38F);
                Debug.Assert(pack.sensor_id == (byte)(byte)158);
                Debug.Assert(pack.flow_rate_y_TRY(ph) == (float) -8.051307E37F);
            };
            GroundControl.OPTICAL_FLOW p100 = CommunicationChannel.new_OPTICAL_FLOW();
            PH.setPack(p100);
            p100.sensor_id = (byte)(byte)158;
            p100.flow_comp_m_x = (float)4.6957875E37F;
            p100.time_usec = (ulong)8939872380095705532L;
            p100.flow_rate_y_SET((float) -8.051307E37F, PH) ;
            p100.flow_comp_m_y = (float) -2.9837065E38F;
            p100.ground_distance = (float)2.8046715E38F;
            p100.flow_x = (short)(short)11558;
            p100.flow_y = (short)(short)21694;
            p100.flow_rate_x_SET((float)1.017256E37F, PH) ;
            p100.quality = (byte)(byte)203;
            CommunicationChannel.instance.send(p100);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGLOBAL_VISION_POSITION_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.y == (float)1.0976471E38F);
                Debug.Assert(pack.pitch == (float) -2.7834585E38F);
                Debug.Assert(pack.roll == (float)2.887162E38F);
                Debug.Assert(pack.yaw == (float) -8.626123E37F);
                Debug.Assert(pack.usec == (ulong)3590850985263491527L);
                Debug.Assert(pack.x == (float) -3.302853E38F);
                Debug.Assert(pack.z == (float)2.0109328E38F);
            };
            GroundControl.GLOBAL_VISION_POSITION_ESTIMATE p101 = CommunicationChannel.new_GLOBAL_VISION_POSITION_ESTIMATE();
            PH.setPack(p101);
            p101.roll = (float)2.887162E38F;
            p101.usec = (ulong)3590850985263491527L;
            p101.x = (float) -3.302853E38F;
            p101.pitch = (float) -2.7834585E38F;
            p101.y = (float)1.0976471E38F;
            p101.yaw = (float) -8.626123E37F;
            p101.z = (float)2.0109328E38F;
            CommunicationChannel.instance.send(p101);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnVISION_POSITION_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.y == (float) -1.3182775E38F);
                Debug.Assert(pack.pitch == (float)3.1586602E38F);
                Debug.Assert(pack.x == (float)2.1705444E38F);
                Debug.Assert(pack.z == (float) -4.3871197E37F);
                Debug.Assert(pack.roll == (float) -3.0751384E38F);
                Debug.Assert(pack.yaw == (float)8.773862E37F);
                Debug.Assert(pack.usec == (ulong)736232482462312814L);
            };
            GroundControl.VISION_POSITION_ESTIMATE p102 = CommunicationChannel.new_VISION_POSITION_ESTIMATE();
            PH.setPack(p102);
            p102.roll = (float) -3.0751384E38F;
            p102.usec = (ulong)736232482462312814L;
            p102.y = (float) -1.3182775E38F;
            p102.z = (float) -4.3871197E37F;
            p102.yaw = (float)8.773862E37F;
            p102.x = (float)2.1705444E38F;
            p102.pitch = (float)3.1586602E38F;
            CommunicationChannel.instance.send(p102);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnVISION_SPEED_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.usec == (ulong)3155169686305117324L);
                Debug.Assert(pack.x == (float)2.3588043E38F);
                Debug.Assert(pack.z == (float)1.9513183E38F);
                Debug.Assert(pack.y == (float)3.1842391E38F);
            };
            GroundControl.VISION_SPEED_ESTIMATE p103 = CommunicationChannel.new_VISION_SPEED_ESTIMATE();
            PH.setPack(p103);
            p103.y = (float)3.1842391E38F;
            p103.z = (float)1.9513183E38F;
            p103.x = (float)2.3588043E38F;
            p103.usec = (ulong)3155169686305117324L;
            CommunicationChannel.instance.send(p103);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnVICON_POSITION_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.roll == (float) -3.087069E38F);
                Debug.Assert(pack.x == (float) -2.3462272E38F);
                Debug.Assert(pack.usec == (ulong)7316748215541404690L);
                Debug.Assert(pack.yaw == (float)4.271312E37F);
                Debug.Assert(pack.y == (float) -1.3009382E38F);
                Debug.Assert(pack.z == (float)1.3508589E36F);
                Debug.Assert(pack.pitch == (float)1.0558758E38F);
            };
            GroundControl.VICON_POSITION_ESTIMATE p104 = CommunicationChannel.new_VICON_POSITION_ESTIMATE();
            PH.setPack(p104);
            p104.yaw = (float)4.271312E37F;
            p104.roll = (float) -3.087069E38F;
            p104.y = (float) -1.3009382E38F;
            p104.x = (float) -2.3462272E38F;
            p104.pitch = (float)1.0558758E38F;
            p104.usec = (ulong)7316748215541404690L;
            p104.z = (float)1.3508589E36F;
            CommunicationChannel.instance.send(p104);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIGHRES_IMUReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.pressure_alt == (float)1.1602457E37F);
                Debug.Assert(pack.ymag == (float) -1.0351501E38F);
                Debug.Assert(pack.ygyro == (float)9.932318E37F);
                Debug.Assert(pack.zmag == (float) -1.1263044E38F);
                Debug.Assert(pack.xmag == (float)2.9446201E38F);
                Debug.Assert(pack.fields_updated == (ushort)(ushort)28134);
                Debug.Assert(pack.abs_pressure == (float)2.4529781E38F);
                Debug.Assert(pack.diff_pressure == (float) -1.9429453E38F);
                Debug.Assert(pack.zgyro == (float)2.9537817E38F);
                Debug.Assert(pack.zacc == (float) -1.1312036E38F);
                Debug.Assert(pack.xacc == (float) -1.6057624E37F);
                Debug.Assert(pack.temperature == (float)2.949455E37F);
                Debug.Assert(pack.time_usec == (ulong)5820761905625001063L);
                Debug.Assert(pack.yacc == (float) -2.3628324E38F);
                Debug.Assert(pack.xgyro == (float)3.2092763E38F);
            };
            GroundControl.HIGHRES_IMU p105 = CommunicationChannel.new_HIGHRES_IMU();
            PH.setPack(p105);
            p105.yacc = (float) -2.3628324E38F;
            p105.xacc = (float) -1.6057624E37F;
            p105.xgyro = (float)3.2092763E38F;
            p105.fields_updated = (ushort)(ushort)28134;
            p105.ymag = (float) -1.0351501E38F;
            p105.time_usec = (ulong)5820761905625001063L;
            p105.ygyro = (float)9.932318E37F;
            p105.zgyro = (float)2.9537817E38F;
            p105.temperature = (float)2.949455E37F;
            p105.zmag = (float) -1.1263044E38F;
            p105.diff_pressure = (float) -1.9429453E38F;
            p105.abs_pressure = (float)2.4529781E38F;
            p105.xmag = (float)2.9446201E38F;
            p105.zacc = (float) -1.1312036E38F;
            p105.pressure_alt = (float)1.1602457E37F;
            CommunicationChannel.instance.send(p105);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnOPTICAL_FLOW_RADReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.quality == (byte)(byte)61);
                Debug.Assert(pack.integration_time_us == (uint)2007712914U);
                Debug.Assert(pack.integrated_xgyro == (float)3.2925034E38F);
                Debug.Assert(pack.time_delta_distance_us == (uint)1998296250U);
                Debug.Assert(pack.time_usec == (ulong)2830042861324995457L);
                Debug.Assert(pack.integrated_ygyro == (float)2.7313124E38F);
                Debug.Assert(pack.sensor_id == (byte)(byte)217);
                Debug.Assert(pack.integrated_y == (float) -1.2535684E38F);
                Debug.Assert(pack.temperature == (short)(short) -28174);
                Debug.Assert(pack.integrated_x == (float) -8.3181534E36F);
                Debug.Assert(pack.distance == (float) -1.743373E38F);
                Debug.Assert(pack.integrated_zgyro == (float)2.7966376E38F);
            };
            GroundControl.OPTICAL_FLOW_RAD p106 = CommunicationChannel.new_OPTICAL_FLOW_RAD();
            PH.setPack(p106);
            p106.integrated_ygyro = (float)2.7313124E38F;
            p106.distance = (float) -1.743373E38F;
            p106.integrated_x = (float) -8.3181534E36F;
            p106.integrated_xgyro = (float)3.2925034E38F;
            p106.integrated_zgyro = (float)2.7966376E38F;
            p106.integration_time_us = (uint)2007712914U;
            p106.sensor_id = (byte)(byte)217;
            p106.temperature = (short)(short) -28174;
            p106.time_delta_distance_us = (uint)1998296250U;
            p106.time_usec = (ulong)2830042861324995457L;
            p106.integrated_y = (float) -1.2535684E38F;
            p106.quality = (byte)(byte)61;
            CommunicationChannel.instance.send(p106);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_SENSORReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.yacc == (float)2.6307922E38F);
                Debug.Assert(pack.zacc == (float)1.1975139E38F);
                Debug.Assert(pack.xgyro == (float)1.8189703E38F);
                Debug.Assert(pack.temperature == (float) -3.0412631E38F);
                Debug.Assert(pack.xmag == (float)3.265775E38F);
                Debug.Assert(pack.pressure_alt == (float)2.6043078E38F);
                Debug.Assert(pack.fields_updated == (uint)2862864218U);
                Debug.Assert(pack.ygyro == (float)2.002503E38F);
                Debug.Assert(pack.diff_pressure == (float)2.6760244E38F);
                Debug.Assert(pack.xacc == (float)9.283197E37F);
                Debug.Assert(pack.abs_pressure == (float) -6.176247E37F);
                Debug.Assert(pack.zmag == (float)5.3555905E37F);
                Debug.Assert(pack.ymag == (float)2.3481879E38F);
                Debug.Assert(pack.time_usec == (ulong)3776276628881521114L);
                Debug.Assert(pack.zgyro == (float)5.450582E37F);
            };
            GroundControl.HIL_SENSOR p107 = CommunicationChannel.new_HIL_SENSOR();
            PH.setPack(p107);
            p107.abs_pressure = (float) -6.176247E37F;
            p107.ygyro = (float)2.002503E38F;
            p107.fields_updated = (uint)2862864218U;
            p107.zgyro = (float)5.450582E37F;
            p107.xmag = (float)3.265775E38F;
            p107.xacc = (float)9.283197E37F;
            p107.pressure_alt = (float)2.6043078E38F;
            p107.ymag = (float)2.3481879E38F;
            p107.zacc = (float)1.1975139E38F;
            p107.temperature = (float) -3.0412631E38F;
            p107.xgyro = (float)1.8189703E38F;
            p107.yacc = (float)2.6307922E38F;
            p107.time_usec = (ulong)3776276628881521114L;
            p107.diff_pressure = (float)2.6760244E38F;
            p107.zmag = (float)5.3555905E37F;
            CommunicationChannel.instance.send(p107);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSIM_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.std_dev_vert == (float)8.320773E37F);
                Debug.Assert(pack.xacc == (float)2.8979192E38F);
                Debug.Assert(pack.q1 == (float) -1.8841547E38F);
                Debug.Assert(pack.yacc == (float)1.3741782E38F);
                Debug.Assert(pack.yaw == (float)2.3877994E38F);
                Debug.Assert(pack.lat == (float)1.6126442E38F);
                Debug.Assert(pack.q4 == (float) -7.0444064E37F);
                Debug.Assert(pack.q3 == (float)4.3561697E37F);
                Debug.Assert(pack.q2 == (float) -2.946476E38F);
                Debug.Assert(pack.roll == (float) -2.954673E38F);
                Debug.Assert(pack.lon == (float)1.060268E38F);
                Debug.Assert(pack.vn == (float)2.0942791E37F);
                Debug.Assert(pack.zacc == (float) -3.1738884E38F);
                Debug.Assert(pack.std_dev_horz == (float) -1.2413833E38F);
                Debug.Assert(pack.ygyro == (float)8.289908E37F);
                Debug.Assert(pack.ve == (float) -1.8990572E38F);
                Debug.Assert(pack.xgyro == (float)2.5094614E38F);
                Debug.Assert(pack.alt == (float) -3.0993893E38F);
                Debug.Assert(pack.vd == (float) -2.849121E38F);
                Debug.Assert(pack.pitch == (float)3.6530587E37F);
                Debug.Assert(pack.zgyro == (float)2.9454034E38F);
            };
            GroundControl.SIM_STATE p108 = CommunicationChannel.new_SIM_STATE();
            PH.setPack(p108);
            p108.q3 = (float)4.3561697E37F;
            p108.std_dev_vert = (float)8.320773E37F;
            p108.yacc = (float)1.3741782E38F;
            p108.xgyro = (float)2.5094614E38F;
            p108.pitch = (float)3.6530587E37F;
            p108.yaw = (float)2.3877994E38F;
            p108.vd = (float) -2.849121E38F;
            p108.zgyro = (float)2.9454034E38F;
            p108.std_dev_horz = (float) -1.2413833E38F;
            p108.ve = (float) -1.8990572E38F;
            p108.roll = (float) -2.954673E38F;
            p108.vn = (float)2.0942791E37F;
            p108.q2 = (float) -2.946476E38F;
            p108.lon = (float)1.060268E38F;
            p108.lat = (float)1.6126442E38F;
            p108.q4 = (float) -7.0444064E37F;
            p108.xacc = (float)2.8979192E38F;
            p108.q1 = (float) -1.8841547E38F;
            p108.ygyro = (float)8.289908E37F;
            p108.zacc = (float) -3.1738884E38F;
            p108.alt = (float) -3.0993893E38F;
            CommunicationChannel.instance.send(p108);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRADIO_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.remnoise == (byte)(byte)185);
                Debug.Assert(pack.remrssi == (byte)(byte)109);
                Debug.Assert(pack.noise == (byte)(byte)106);
                Debug.Assert(pack.fixed_ == (ushort)(ushort)47945);
                Debug.Assert(pack.rxerrors == (ushort)(ushort)50224);
                Debug.Assert(pack.txbuf == (byte)(byte)52);
                Debug.Assert(pack.rssi == (byte)(byte)188);
            };
            GroundControl.RADIO_STATUS p109 = CommunicationChannel.new_RADIO_STATUS();
            PH.setPack(p109);
            p109.remnoise = (byte)(byte)185;
            p109.rssi = (byte)(byte)188;
            p109.txbuf = (byte)(byte)52;
            p109.remrssi = (byte)(byte)109;
            p109.noise = (byte)(byte)106;
            p109.rxerrors = (ushort)(ushort)50224;
            p109.fixed_ = (ushort)(ushort)47945;
            CommunicationChannel.instance.send(p109);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnFILE_TRANSFER_PROTOCOLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_network == (byte)(byte)94);
                Debug.Assert(pack.payload.SequenceEqual(new byte[] {(byte)132, (byte)15, (byte)193, (byte)52, (byte)38, (byte)238, (byte)64, (byte)128, (byte)15, (byte)57, (byte)217, (byte)235, (byte)80, (byte)194, (byte)238, (byte)175, (byte)218, (byte)36, (byte)116, (byte)124, (byte)72, (byte)211, (byte)59, (byte)65, (byte)219, (byte)141, (byte)41, (byte)83, (byte)54, (byte)41, (byte)128, (byte)223, (byte)114, (byte)113, (byte)221, (byte)73, (byte)133, (byte)180, (byte)19, (byte)255, (byte)54, (byte)255, (byte)217, (byte)25, (byte)168, (byte)56, (byte)6, (byte)146, (byte)117, (byte)104, (byte)153, (byte)26, (byte)19, (byte)142, (byte)114, (byte)46, (byte)218, (byte)132, (byte)75, (byte)104, (byte)126, (byte)174, (byte)88, (byte)137, (byte)25, (byte)104, (byte)107, (byte)111, (byte)35, (byte)86, (byte)169, (byte)183, (byte)215, (byte)172, (byte)8, (byte)35, (byte)103, (byte)13, (byte)186, (byte)228, (byte)86, (byte)203, (byte)106, (byte)103, (byte)37, (byte)134, (byte)68, (byte)216, (byte)254, (byte)25, (byte)173, (byte)182, (byte)31, (byte)89, (byte)186, (byte)93, (byte)208, (byte)127, (byte)93, (byte)10, (byte)86, (byte)127, (byte)241, (byte)154, (byte)63, (byte)126, (byte)155, (byte)50, (byte)102, (byte)119, (byte)111, (byte)204, (byte)215, (byte)77, (byte)115, (byte)48, (byte)183, (byte)196, (byte)36, (byte)122, (byte)232, (byte)192, (byte)193, (byte)198, (byte)201, (byte)20, (byte)79, (byte)76, (byte)132, (byte)25, (byte)60, (byte)14, (byte)103, (byte)157, (byte)114, (byte)140, (byte)150, (byte)83, (byte)87, (byte)52, (byte)51, (byte)13, (byte)38, (byte)89, (byte)136, (byte)76, (byte)114, (byte)165, (byte)254, (byte)202, (byte)227, (byte)61, (byte)107, (byte)219, (byte)185, (byte)213, (byte)111, (byte)147, (byte)20, (byte)134, (byte)138, (byte)83, (byte)174, (byte)240, (byte)220, (byte)114, (byte)40, (byte)138, (byte)78, (byte)82, (byte)171, (byte)6, (byte)213, (byte)172, (byte)83, (byte)138, (byte)189, (byte)73, (byte)247, (byte)28, (byte)16, (byte)182, (byte)249, (byte)8, (byte)38, (byte)22, (byte)168, (byte)46, (byte)244, (byte)14, (byte)212, (byte)44, (byte)21, (byte)153, (byte)18, (byte)215, (byte)202, (byte)120, (byte)82, (byte)155, (byte)26, (byte)21, (byte)14, (byte)48, (byte)140, (byte)241, (byte)79, (byte)91, (byte)180, (byte)92, (byte)176, (byte)149, (byte)219, (byte)107, (byte)91, (byte)108, (byte)174, (byte)141, (byte)165, (byte)249, (byte)162, (byte)172, (byte)208, (byte)26, (byte)162, (byte)193, (byte)187, (byte)222, (byte)160, (byte)200, (byte)171, (byte)135, (byte)180, (byte)189, (byte)194, (byte)170, (byte)168, (byte)44, (byte)84, (byte)198, (byte)239, (byte)140, (byte)40, (byte)183, (byte)184, (byte)98, (byte)183, (byte)95, (byte)1, (byte)234, (byte)214}));
                Debug.Assert(pack.target_component == (byte)(byte)129);
                Debug.Assert(pack.target_system == (byte)(byte)132);
            };
            GroundControl.FILE_TRANSFER_PROTOCOL p110 = CommunicationChannel.new_FILE_TRANSFER_PROTOCOL();
            PH.setPack(p110);
            p110.target_component = (byte)(byte)129;
            p110.target_network = (byte)(byte)94;
            p110.payload_SET(new byte[] {(byte)132, (byte)15, (byte)193, (byte)52, (byte)38, (byte)238, (byte)64, (byte)128, (byte)15, (byte)57, (byte)217, (byte)235, (byte)80, (byte)194, (byte)238, (byte)175, (byte)218, (byte)36, (byte)116, (byte)124, (byte)72, (byte)211, (byte)59, (byte)65, (byte)219, (byte)141, (byte)41, (byte)83, (byte)54, (byte)41, (byte)128, (byte)223, (byte)114, (byte)113, (byte)221, (byte)73, (byte)133, (byte)180, (byte)19, (byte)255, (byte)54, (byte)255, (byte)217, (byte)25, (byte)168, (byte)56, (byte)6, (byte)146, (byte)117, (byte)104, (byte)153, (byte)26, (byte)19, (byte)142, (byte)114, (byte)46, (byte)218, (byte)132, (byte)75, (byte)104, (byte)126, (byte)174, (byte)88, (byte)137, (byte)25, (byte)104, (byte)107, (byte)111, (byte)35, (byte)86, (byte)169, (byte)183, (byte)215, (byte)172, (byte)8, (byte)35, (byte)103, (byte)13, (byte)186, (byte)228, (byte)86, (byte)203, (byte)106, (byte)103, (byte)37, (byte)134, (byte)68, (byte)216, (byte)254, (byte)25, (byte)173, (byte)182, (byte)31, (byte)89, (byte)186, (byte)93, (byte)208, (byte)127, (byte)93, (byte)10, (byte)86, (byte)127, (byte)241, (byte)154, (byte)63, (byte)126, (byte)155, (byte)50, (byte)102, (byte)119, (byte)111, (byte)204, (byte)215, (byte)77, (byte)115, (byte)48, (byte)183, (byte)196, (byte)36, (byte)122, (byte)232, (byte)192, (byte)193, (byte)198, (byte)201, (byte)20, (byte)79, (byte)76, (byte)132, (byte)25, (byte)60, (byte)14, (byte)103, (byte)157, (byte)114, (byte)140, (byte)150, (byte)83, (byte)87, (byte)52, (byte)51, (byte)13, (byte)38, (byte)89, (byte)136, (byte)76, (byte)114, (byte)165, (byte)254, (byte)202, (byte)227, (byte)61, (byte)107, (byte)219, (byte)185, (byte)213, (byte)111, (byte)147, (byte)20, (byte)134, (byte)138, (byte)83, (byte)174, (byte)240, (byte)220, (byte)114, (byte)40, (byte)138, (byte)78, (byte)82, (byte)171, (byte)6, (byte)213, (byte)172, (byte)83, (byte)138, (byte)189, (byte)73, (byte)247, (byte)28, (byte)16, (byte)182, (byte)249, (byte)8, (byte)38, (byte)22, (byte)168, (byte)46, (byte)244, (byte)14, (byte)212, (byte)44, (byte)21, (byte)153, (byte)18, (byte)215, (byte)202, (byte)120, (byte)82, (byte)155, (byte)26, (byte)21, (byte)14, (byte)48, (byte)140, (byte)241, (byte)79, (byte)91, (byte)180, (byte)92, (byte)176, (byte)149, (byte)219, (byte)107, (byte)91, (byte)108, (byte)174, (byte)141, (byte)165, (byte)249, (byte)162, (byte)172, (byte)208, (byte)26, (byte)162, (byte)193, (byte)187, (byte)222, (byte)160, (byte)200, (byte)171, (byte)135, (byte)180, (byte)189, (byte)194, (byte)170, (byte)168, (byte)44, (byte)84, (byte)198, (byte)239, (byte)140, (byte)40, (byte)183, (byte)184, (byte)98, (byte)183, (byte)95, (byte)1, (byte)234, (byte)214}, 0) ;
            p110.target_system = (byte)(byte)132;
            CommunicationChannel.instance.send(p110);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTIMESYNCReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.tc1 == (long)1679269248402983467L);
                Debug.Assert(pack.ts1 == (long)3028105839773357798L);
            };
            GroundControl.TIMESYNC p111 = CommunicationChannel.new_TIMESYNC();
            PH.setPack(p111);
            p111.ts1 = (long)3028105839773357798L;
            p111.tc1 = (long)1679269248402983467L;
            CommunicationChannel.instance.send(p111);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCAMERA_TRIGGERReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)1132452343660687161L);
                Debug.Assert(pack.seq == (uint)4291773180U);
            };
            GroundControl.CAMERA_TRIGGER p112 = CommunicationChannel.new_CAMERA_TRIGGER();
            PH.setPack(p112);
            p112.time_usec = (ulong)1132452343660687161L;
            p112.seq = (uint)4291773180U;
            CommunicationChannel.instance.send(p112);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_GPSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vel == (ushort)(ushort)47490);
                Debug.Assert(pack.vn == (short)(short)31408);
                Debug.Assert(pack.alt == (int) -775356501);
                Debug.Assert(pack.eph == (ushort)(ushort)49138);
                Debug.Assert(pack.epv == (ushort)(ushort)9357);
                Debug.Assert(pack.time_usec == (ulong)2476267457675349084L);
                Debug.Assert(pack.lon == (int) -2010537703);
                Debug.Assert(pack.satellites_visible == (byte)(byte)187);
                Debug.Assert(pack.cog == (ushort)(ushort)51143);
                Debug.Assert(pack.ve == (short)(short)7700);
                Debug.Assert(pack.vd == (short)(short)16182);
                Debug.Assert(pack.lat == (int)575207018);
                Debug.Assert(pack.fix_type == (byte)(byte)22);
            };
            GroundControl.HIL_GPS p113 = CommunicationChannel.new_HIL_GPS();
            PH.setPack(p113);
            p113.cog = (ushort)(ushort)51143;
            p113.alt = (int) -775356501;
            p113.vd = (short)(short)16182;
            p113.eph = (ushort)(ushort)49138;
            p113.fix_type = (byte)(byte)22;
            p113.lon = (int) -2010537703;
            p113.satellites_visible = (byte)(byte)187;
            p113.ve = (short)(short)7700;
            p113.vn = (short)(short)31408;
            p113.vel = (ushort)(ushort)47490;
            p113.epv = (ushort)(ushort)9357;
            p113.time_usec = (ulong)2476267457675349084L;
            p113.lat = (int)575207018;
            CommunicationChannel.instance.send(p113);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_OPTICAL_FLOWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_delta_distance_us == (uint)2293060620U);
                Debug.Assert(pack.integrated_x == (float)3.3184829E37F);
                Debug.Assert(pack.integrated_ygyro == (float)1.9383428E38F);
                Debug.Assert(pack.integrated_y == (float) -2.3299704E38F);
                Debug.Assert(pack.integrated_zgyro == (float)2.5799262E38F);
                Debug.Assert(pack.distance == (float) -2.3771392E38F);
                Debug.Assert(pack.temperature == (short)(short) -5057);
                Debug.Assert(pack.time_usec == (ulong)3171361898795788059L);
                Debug.Assert(pack.sensor_id == (byte)(byte)89);
                Debug.Assert(pack.quality == (byte)(byte)199);
                Debug.Assert(pack.integration_time_us == (uint)384223514U);
                Debug.Assert(pack.integrated_xgyro == (float)2.282244E38F);
            };
            GroundControl.HIL_OPTICAL_FLOW p114 = CommunicationChannel.new_HIL_OPTICAL_FLOW();
            PH.setPack(p114);
            p114.time_delta_distance_us = (uint)2293060620U;
            p114.temperature = (short)(short) -5057;
            p114.integrated_x = (float)3.3184829E37F;
            p114.integrated_y = (float) -2.3299704E38F;
            p114.sensor_id = (byte)(byte)89;
            p114.distance = (float) -2.3771392E38F;
            p114.quality = (byte)(byte)199;
            p114.integrated_ygyro = (float)1.9383428E38F;
            p114.integrated_xgyro = (float)2.282244E38F;
            p114.integration_time_us = (uint)384223514U;
            p114.time_usec = (ulong)3171361898795788059L;
            p114.integrated_zgyro = (float)2.5799262E38F;
            CommunicationChannel.instance.send(p114);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_STATE_QUATERNIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lat == (int)1957428584);
                Debug.Assert(pack.ind_airspeed == (ushort)(ushort)41188);
                Debug.Assert(pack.yawspeed == (float) -3.095855E38F);
                Debug.Assert(pack.time_usec == (ulong)8461437642468349065L);
                Debug.Assert(pack.lon == (int) -456891843);
                Debug.Assert(pack.rollspeed == (float)1.2314941E38F);
                Debug.Assert(pack.xacc == (short)(short) -5038);
                Debug.Assert(pack.vx == (short)(short) -23557);
                Debug.Assert(pack.true_airspeed == (ushort)(ushort)52760);
                Debug.Assert(pack.zacc == (short)(short) -733);
                Debug.Assert(pack.vy == (short)(short) -17950);
                Debug.Assert(pack.alt == (int) -1554746500);
                Debug.Assert(pack.pitchspeed == (float) -2.7011154E38F);
                Debug.Assert(pack.attitude_quaternion.SequenceEqual(new float[] {-2.1113638E38F, 2.7036198E38F, 3.47646E37F, 2.1609624E38F}));
                Debug.Assert(pack.vz == (short)(short)13079);
                Debug.Assert(pack.yacc == (short)(short)28027);
            };
            GroundControl.HIL_STATE_QUATERNION p115 = CommunicationChannel.new_HIL_STATE_QUATERNION();
            PH.setPack(p115);
            p115.rollspeed = (float)1.2314941E38F;
            p115.true_airspeed = (ushort)(ushort)52760;
            p115.yacc = (short)(short)28027;
            p115.vz = (short)(short)13079;
            p115.time_usec = (ulong)8461437642468349065L;
            p115.zacc = (short)(short) -733;
            p115.alt = (int) -1554746500;
            p115.yawspeed = (float) -3.095855E38F;
            p115.attitude_quaternion_SET(new float[] {-2.1113638E38F, 2.7036198E38F, 3.47646E37F, 2.1609624E38F}, 0) ;
            p115.pitchspeed = (float) -2.7011154E38F;
            p115.lat = (int)1957428584;
            p115.vy = (short)(short) -17950;
            p115.vx = (short)(short) -23557;
            p115.xacc = (short)(short) -5038;
            p115.lon = (int) -456891843;
            p115.ind_airspeed = (ushort)(ushort)41188;
            CommunicationChannel.instance.send(p115);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_IMU2Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.xacc == (short)(short)17951);
                Debug.Assert(pack.time_boot_ms == (uint)2883321040U);
                Debug.Assert(pack.ygyro == (short)(short)24230);
                Debug.Assert(pack.ymag == (short)(short)16158);
                Debug.Assert(pack.zacc == (short)(short)31367);
                Debug.Assert(pack.xmag == (short)(short) -12141);
                Debug.Assert(pack.yacc == (short)(short) -30208);
                Debug.Assert(pack.zgyro == (short)(short)17952);
                Debug.Assert(pack.zmag == (short)(short)32188);
                Debug.Assert(pack.xgyro == (short)(short)26269);
            };
            GroundControl.SCALED_IMU2 p116 = CommunicationChannel.new_SCALED_IMU2();
            PH.setPack(p116);
            p116.ymag = (short)(short)16158;
            p116.xgyro = (short)(short)26269;
            p116.time_boot_ms = (uint)2883321040U;
            p116.yacc = (short)(short) -30208;
            p116.xacc = (short)(short)17951;
            p116.xmag = (short)(short) -12141;
            p116.ygyro = (short)(short)24230;
            p116.zgyro = (short)(short)17952;
            p116.zacc = (short)(short)31367;
            p116.zmag = (short)(short)32188;
            CommunicationChannel.instance.send(p116);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.end == (ushort)(ushort)18893);
                Debug.Assert(pack.target_component == (byte)(byte)23);
                Debug.Assert(pack.target_system == (byte)(byte)132);
                Debug.Assert(pack.start == (ushort)(ushort)39730);
            };
            GroundControl.LOG_REQUEST_LIST p117 = CommunicationChannel.new_LOG_REQUEST_LIST();
            PH.setPack(p117);
            p117.start = (ushort)(ushort)39730;
            p117.target_system = (byte)(byte)132;
            p117.target_component = (byte)(byte)23;
            p117.end = (ushort)(ushort)18893;
            CommunicationChannel.instance.send(p117);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_ENTRYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_utc == (uint)1076166893U);
                Debug.Assert(pack.id == (ushort)(ushort)37390);
                Debug.Assert(pack.size == (uint)1469631716U);
                Debug.Assert(pack.num_logs == (ushort)(ushort)59238);
                Debug.Assert(pack.last_log_num == (ushort)(ushort)38898);
            };
            GroundControl.LOG_ENTRY p118 = CommunicationChannel.new_LOG_ENTRY();
            PH.setPack(p118);
            p118.id = (ushort)(ushort)37390;
            p118.last_log_num = (ushort)(ushort)38898;
            p118.num_logs = (ushort)(ushort)59238;
            p118.size = (uint)1469631716U;
            p118.time_utc = (uint)1076166893U;
            CommunicationChannel.instance.send(p118);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_REQUEST_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)125);
                Debug.Assert(pack.ofs == (uint)1024541909U);
                Debug.Assert(pack.id == (ushort)(ushort)27582);
                Debug.Assert(pack.target_component == (byte)(byte)178);
                Debug.Assert(pack.count == (uint)2750476580U);
            };
            GroundControl.LOG_REQUEST_DATA p119 = CommunicationChannel.new_LOG_REQUEST_DATA();
            PH.setPack(p119);
            p119.count = (uint)2750476580U;
            p119.target_component = (byte)(byte)178;
            p119.id = (ushort)(ushort)27582;
            p119.target_system = (byte)(byte)125;
            p119.ofs = (uint)1024541909U;
            CommunicationChannel.instance.send(p119);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.count == (byte)(byte)218);
                Debug.Assert(pack.ofs == (uint)2695240705U);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)189, (byte)15, (byte)236, (byte)199, (byte)81, (byte)228, (byte)192, (byte)117, (byte)141, (byte)129, (byte)249, (byte)60, (byte)104, (byte)248, (byte)21, (byte)81, (byte)27, (byte)235, (byte)120, (byte)250, (byte)204, (byte)125, (byte)126, (byte)64, (byte)224, (byte)33, (byte)33, (byte)55, (byte)253, (byte)206, (byte)22, (byte)115, (byte)149, (byte)80, (byte)10, (byte)32, (byte)107, (byte)13, (byte)118, (byte)176, (byte)81, (byte)50, (byte)135, (byte)193, (byte)68, (byte)251, (byte)109, (byte)186, (byte)188, (byte)119, (byte)141, (byte)247, (byte)44, (byte)222, (byte)38, (byte)151, (byte)42, (byte)151, (byte)43, (byte)185, (byte)35, (byte)234, (byte)222, (byte)140, (byte)112, (byte)31, (byte)245, (byte)213, (byte)160, (byte)223, (byte)154, (byte)226, (byte)118, (byte)114, (byte)122, (byte)182, (byte)227, (byte)138, (byte)110, (byte)171, (byte)128, (byte)206, (byte)190, (byte)134, (byte)27, (byte)141, (byte)206, (byte)202, (byte)236, (byte)57}));
                Debug.Assert(pack.id == (ushort)(ushort)64194);
            };
            GroundControl.LOG_DATA p120 = CommunicationChannel.new_LOG_DATA();
            PH.setPack(p120);
            p120.data__SET(new byte[] {(byte)189, (byte)15, (byte)236, (byte)199, (byte)81, (byte)228, (byte)192, (byte)117, (byte)141, (byte)129, (byte)249, (byte)60, (byte)104, (byte)248, (byte)21, (byte)81, (byte)27, (byte)235, (byte)120, (byte)250, (byte)204, (byte)125, (byte)126, (byte)64, (byte)224, (byte)33, (byte)33, (byte)55, (byte)253, (byte)206, (byte)22, (byte)115, (byte)149, (byte)80, (byte)10, (byte)32, (byte)107, (byte)13, (byte)118, (byte)176, (byte)81, (byte)50, (byte)135, (byte)193, (byte)68, (byte)251, (byte)109, (byte)186, (byte)188, (byte)119, (byte)141, (byte)247, (byte)44, (byte)222, (byte)38, (byte)151, (byte)42, (byte)151, (byte)43, (byte)185, (byte)35, (byte)234, (byte)222, (byte)140, (byte)112, (byte)31, (byte)245, (byte)213, (byte)160, (byte)223, (byte)154, (byte)226, (byte)118, (byte)114, (byte)122, (byte)182, (byte)227, (byte)138, (byte)110, (byte)171, (byte)128, (byte)206, (byte)190, (byte)134, (byte)27, (byte)141, (byte)206, (byte)202, (byte)236, (byte)57}, 0) ;
            p120.count = (byte)(byte)218;
            p120.ofs = (uint)2695240705U;
            p120.id = (ushort)(ushort)64194;
            CommunicationChannel.instance.send(p120);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_ERASEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)231);
                Debug.Assert(pack.target_system == (byte)(byte)21);
            };
            GroundControl.LOG_ERASE p121 = CommunicationChannel.new_LOG_ERASE();
            PH.setPack(p121);
            p121.target_system = (byte)(byte)21;
            p121.target_component = (byte)(byte)231;
            CommunicationChannel.instance.send(p121);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_REQUEST_ENDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)16);
                Debug.Assert(pack.target_system == (byte)(byte)158);
            };
            GroundControl.LOG_REQUEST_END p122 = CommunicationChannel.new_LOG_REQUEST_END();
            PH.setPack(p122);
            p122.target_system = (byte)(byte)158;
            p122.target_component = (byte)(byte)16;
            CommunicationChannel.instance.send(p122);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_INJECT_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.len == (byte)(byte)216);
                Debug.Assert(pack.target_system == (byte)(byte)0);
                Debug.Assert(pack.target_component == (byte)(byte)208);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)185, (byte)78, (byte)173, (byte)66, (byte)48, (byte)43, (byte)248, (byte)2, (byte)58, (byte)74, (byte)171, (byte)100, (byte)18, (byte)245, (byte)37, (byte)21, (byte)100, (byte)31, (byte)244, (byte)54, (byte)120, (byte)170, (byte)181, (byte)168, (byte)190, (byte)45, (byte)155, (byte)12, (byte)188, (byte)112, (byte)174, (byte)202, (byte)3, (byte)46, (byte)126, (byte)246, (byte)112, (byte)10, (byte)132, (byte)224, (byte)248, (byte)19, (byte)213, (byte)116, (byte)106, (byte)139, (byte)165, (byte)157, (byte)46, (byte)219, (byte)38, (byte)204, (byte)7, (byte)189, (byte)217, (byte)252, (byte)118, (byte)68, (byte)95, (byte)212, (byte)103, (byte)23, (byte)7, (byte)113, (byte)171, (byte)17, (byte)149, (byte)109, (byte)140, (byte)10, (byte)219, (byte)23, (byte)213, (byte)192, (byte)93, (byte)69, (byte)0, (byte)79, (byte)69, (byte)44, (byte)30, (byte)147, (byte)225, (byte)177, (byte)66, (byte)106, (byte)167, (byte)81, (byte)137, (byte)17, (byte)179, (byte)231, (byte)51, (byte)7, (byte)207, (byte)176, (byte)205, (byte)128, (byte)161, (byte)126, (byte)55, (byte)251, (byte)30, (byte)54, (byte)151, (byte)204, (byte)95, (byte)125, (byte)117, (byte)49}));
            };
            GroundControl.GPS_INJECT_DATA p123 = CommunicationChannel.new_GPS_INJECT_DATA();
            PH.setPack(p123);
            p123.target_system = (byte)(byte)0;
            p123.data__SET(new byte[] {(byte)185, (byte)78, (byte)173, (byte)66, (byte)48, (byte)43, (byte)248, (byte)2, (byte)58, (byte)74, (byte)171, (byte)100, (byte)18, (byte)245, (byte)37, (byte)21, (byte)100, (byte)31, (byte)244, (byte)54, (byte)120, (byte)170, (byte)181, (byte)168, (byte)190, (byte)45, (byte)155, (byte)12, (byte)188, (byte)112, (byte)174, (byte)202, (byte)3, (byte)46, (byte)126, (byte)246, (byte)112, (byte)10, (byte)132, (byte)224, (byte)248, (byte)19, (byte)213, (byte)116, (byte)106, (byte)139, (byte)165, (byte)157, (byte)46, (byte)219, (byte)38, (byte)204, (byte)7, (byte)189, (byte)217, (byte)252, (byte)118, (byte)68, (byte)95, (byte)212, (byte)103, (byte)23, (byte)7, (byte)113, (byte)171, (byte)17, (byte)149, (byte)109, (byte)140, (byte)10, (byte)219, (byte)23, (byte)213, (byte)192, (byte)93, (byte)69, (byte)0, (byte)79, (byte)69, (byte)44, (byte)30, (byte)147, (byte)225, (byte)177, (byte)66, (byte)106, (byte)167, (byte)81, (byte)137, (byte)17, (byte)179, (byte)231, (byte)51, (byte)7, (byte)207, (byte)176, (byte)205, (byte)128, (byte)161, (byte)126, (byte)55, (byte)251, (byte)30, (byte)54, (byte)151, (byte)204, (byte)95, (byte)125, (byte)117, (byte)49}, 0) ;
            p123.len = (byte)(byte)216;
            p123.target_component = (byte)(byte)208;
            CommunicationChannel.instance.send(p123);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS2_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lon == (int)128745708);
                Debug.Assert(pack.eph == (ushort)(ushort)44164);
                Debug.Assert(pack.epv == (ushort)(ushort)1509);
                Debug.Assert(pack.dgps_age == (uint)3855540375U);
                Debug.Assert(pack.satellites_visible == (byte)(byte)133);
                Debug.Assert(pack.cog == (ushort)(ushort)24514);
                Debug.Assert(pack.fix_type == GPS_FIX_TYPE.GPS_FIX_TYPE_NO_FIX);
                Debug.Assert(pack.time_usec == (ulong)4953617366673139905L);
                Debug.Assert(pack.lat == (int)371656181);
                Debug.Assert(pack.dgps_numch == (byte)(byte)2);
                Debug.Assert(pack.vel == (ushort)(ushort)13750);
                Debug.Assert(pack.alt == (int) -211163740);
            };
            GroundControl.GPS2_RAW p124 = CommunicationChannel.new_GPS2_RAW();
            PH.setPack(p124);
            p124.satellites_visible = (byte)(byte)133;
            p124.alt = (int) -211163740;
            p124.vel = (ushort)(ushort)13750;
            p124.lat = (int)371656181;
            p124.cog = (ushort)(ushort)24514;
            p124.epv = (ushort)(ushort)1509;
            p124.fix_type = GPS_FIX_TYPE.GPS_FIX_TYPE_NO_FIX;
            p124.lon = (int)128745708;
            p124.dgps_age = (uint)3855540375U;
            p124.time_usec = (ulong)4953617366673139905L;
            p124.dgps_numch = (byte)(byte)2;
            p124.eph = (ushort)(ushort)44164;
            CommunicationChannel.instance.send(p124);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPOWER_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.Vcc == (ushort)(ushort)5917);
                Debug.Assert(pack.Vservo == (ushort)(ushort)34917);
                Debug.Assert(pack.flags == (MAV_POWER_STATUS.MAV_POWER_STATUS_BRICK_VALID));
            };
            GroundControl.POWER_STATUS p125 = CommunicationChannel.new_POWER_STATUS();
            PH.setPack(p125);
            p125.Vservo = (ushort)(ushort)34917;
            p125.Vcc = (ushort)(ushort)5917;
            p125.flags = (MAV_POWER_STATUS.MAV_POWER_STATUS_BRICK_VALID);
            CommunicationChannel.instance.send(p125);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSERIAL_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.baudrate == (uint)36736155U);
                Debug.Assert(pack.device == SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_TELEM1);
                Debug.Assert(pack.timeout == (ushort)(ushort)40619);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)70, (byte)172, (byte)108, (byte)108, (byte)249, (byte)189, (byte)227, (byte)18, (byte)36, (byte)250, (byte)4, (byte)167, (byte)208, (byte)83, (byte)74, (byte)219, (byte)128, (byte)130, (byte)247, (byte)65, (byte)172, (byte)188, (byte)238, (byte)38, (byte)233, (byte)144, (byte)187, (byte)77, (byte)136, (byte)95, (byte)116, (byte)131, (byte)217, (byte)181, (byte)161, (byte)47, (byte)197, (byte)244, (byte)191, (byte)28, (byte)14, (byte)88, (byte)250, (byte)202, (byte)222, (byte)125, (byte)180, (byte)127, (byte)169, (byte)222, (byte)227, (byte)114, (byte)13, (byte)136, (byte)67, (byte)207, (byte)62, (byte)173, (byte)97, (byte)52, (byte)143, (byte)140, (byte)222, (byte)146, (byte)101, (byte)204, (byte)127, (byte)190, (byte)11, (byte)164}));
                Debug.Assert(pack.flags == (SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_BLOCKING |
                                            SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_REPLY));
                Debug.Assert(pack.count == (byte)(byte)233);
            };
            GroundControl.SERIAL_CONTROL p126 = CommunicationChannel.new_SERIAL_CONTROL();
            PH.setPack(p126);
            p126.data__SET(new byte[] {(byte)70, (byte)172, (byte)108, (byte)108, (byte)249, (byte)189, (byte)227, (byte)18, (byte)36, (byte)250, (byte)4, (byte)167, (byte)208, (byte)83, (byte)74, (byte)219, (byte)128, (byte)130, (byte)247, (byte)65, (byte)172, (byte)188, (byte)238, (byte)38, (byte)233, (byte)144, (byte)187, (byte)77, (byte)136, (byte)95, (byte)116, (byte)131, (byte)217, (byte)181, (byte)161, (byte)47, (byte)197, (byte)244, (byte)191, (byte)28, (byte)14, (byte)88, (byte)250, (byte)202, (byte)222, (byte)125, (byte)180, (byte)127, (byte)169, (byte)222, (byte)227, (byte)114, (byte)13, (byte)136, (byte)67, (byte)207, (byte)62, (byte)173, (byte)97, (byte)52, (byte)143, (byte)140, (byte)222, (byte)146, (byte)101, (byte)204, (byte)127, (byte)190, (byte)11, (byte)164}, 0) ;
            p126.baudrate = (uint)36736155U;
            p126.flags = (SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_BLOCKING |
                          SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_REPLY);
            p126.device = SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_TELEM1;
            p126.timeout = (ushort)(ushort)40619;
            p126.count = (byte)(byte)233;
            CommunicationChannel.instance.send(p126);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_RTKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_last_baseline_ms == (uint)966278041U);
                Debug.Assert(pack.nsats == (byte)(byte)211);
                Debug.Assert(pack.baseline_c_mm == (int)1230252806);
                Debug.Assert(pack.tow == (uint)3440694137U);
                Debug.Assert(pack.rtk_rate == (byte)(byte)219);
                Debug.Assert(pack.rtk_health == (byte)(byte)218);
                Debug.Assert(pack.baseline_coords_type == (byte)(byte)24);
                Debug.Assert(pack.baseline_a_mm == (int)576160033);
                Debug.Assert(pack.rtk_receiver_id == (byte)(byte)222);
                Debug.Assert(pack.baseline_b_mm == (int)1543839802);
                Debug.Assert(pack.accuracy == (uint)3593609660U);
                Debug.Assert(pack.iar_num_hypotheses == (int)1139998452);
                Debug.Assert(pack.wn == (ushort)(ushort)57329);
            };
            GroundControl.GPS_RTK p127 = CommunicationChannel.new_GPS_RTK();
            PH.setPack(p127);
            p127.baseline_a_mm = (int)576160033;
            p127.accuracy = (uint)3593609660U;
            p127.nsats = (byte)(byte)211;
            p127.baseline_coords_type = (byte)(byte)24;
            p127.rtk_rate = (byte)(byte)219;
            p127.wn = (ushort)(ushort)57329;
            p127.iar_num_hypotheses = (int)1139998452;
            p127.rtk_receiver_id = (byte)(byte)222;
            p127.tow = (uint)3440694137U;
            p127.rtk_health = (byte)(byte)218;
            p127.baseline_c_mm = (int)1230252806;
            p127.baseline_b_mm = (int)1543839802;
            p127.time_last_baseline_ms = (uint)966278041U;
            CommunicationChannel.instance.send(p127);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS2_RTKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.iar_num_hypotheses == (int) -811790728);
                Debug.Assert(pack.nsats == (byte)(byte)121);
                Debug.Assert(pack.rtk_health == (byte)(byte)243);
                Debug.Assert(pack.baseline_b_mm == (int)1643460194);
                Debug.Assert(pack.baseline_coords_type == (byte)(byte)58);
                Debug.Assert(pack.time_last_baseline_ms == (uint)544378434U);
                Debug.Assert(pack.baseline_c_mm == (int)732439771);
                Debug.Assert(pack.wn == (ushort)(ushort)41128);
                Debug.Assert(pack.accuracy == (uint)2980380037U);
                Debug.Assert(pack.rtk_receiver_id == (byte)(byte)189);
                Debug.Assert(pack.rtk_rate == (byte)(byte)35);
                Debug.Assert(pack.baseline_a_mm == (int)2000223381);
                Debug.Assert(pack.tow == (uint)2193417029U);
            };
            GroundControl.GPS2_RTK p128 = CommunicationChannel.new_GPS2_RTK();
            PH.setPack(p128);
            p128.accuracy = (uint)2980380037U;
            p128.iar_num_hypotheses = (int) -811790728;
            p128.baseline_c_mm = (int)732439771;
            p128.baseline_b_mm = (int)1643460194;
            p128.baseline_coords_type = (byte)(byte)58;
            p128.rtk_rate = (byte)(byte)35;
            p128.rtk_health = (byte)(byte)243;
            p128.time_last_baseline_ms = (uint)544378434U;
            p128.rtk_receiver_id = (byte)(byte)189;
            p128.tow = (uint)2193417029U;
            p128.baseline_a_mm = (int)2000223381;
            p128.wn = (ushort)(ushort)41128;
            p128.nsats = (byte)(byte)121;
            CommunicationChannel.instance.send(p128);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_IMU3Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.xgyro == (short)(short) -15877);
                Debug.Assert(pack.zgyro == (short)(short)28298);
                Debug.Assert(pack.time_boot_ms == (uint)1492098101U);
                Debug.Assert(pack.ygyro == (short)(short)5665);
                Debug.Assert(pack.yacc == (short)(short)28025);
                Debug.Assert(pack.ymag == (short)(short)22486);
                Debug.Assert(pack.zmag == (short)(short) -7389);
                Debug.Assert(pack.zacc == (short)(short) -6021);
                Debug.Assert(pack.xmag == (short)(short)21877);
                Debug.Assert(pack.xacc == (short)(short)21677);
            };
            GroundControl.SCALED_IMU3 p129 = CommunicationChannel.new_SCALED_IMU3();
            PH.setPack(p129);
            p129.zacc = (short)(short) -6021;
            p129.ymag = (short)(short)22486;
            p129.ygyro = (short)(short)5665;
            p129.zmag = (short)(short) -7389;
            p129.xmag = (short)(short)21877;
            p129.yacc = (short)(short)28025;
            p129.zgyro = (short)(short)28298;
            p129.xgyro = (short)(short) -15877;
            p129.time_boot_ms = (uint)1492098101U;
            p129.xacc = (short)(short)21677;
            CommunicationChannel.instance.send(p129);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnDATA_TRANSMISSION_HANDSHAKEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.height == (ushort)(ushort)27645);
                Debug.Assert(pack.packets == (ushort)(ushort)34922);
                Debug.Assert(pack.type == (byte)(byte)148);
                Debug.Assert(pack.size == (uint)3663393881U);
                Debug.Assert(pack.width == (ushort)(ushort)18351);
                Debug.Assert(pack.jpg_quality == (byte)(byte)54);
                Debug.Assert(pack.payload == (byte)(byte)222);
            };
            GroundControl.DATA_TRANSMISSION_HANDSHAKE p130 = CommunicationChannel.new_DATA_TRANSMISSION_HANDSHAKE();
            PH.setPack(p130);
            p130.height = (ushort)(ushort)27645;
            p130.jpg_quality = (byte)(byte)54;
            p130.type = (byte)(byte)148;
            p130.payload = (byte)(byte)222;
            p130.packets = (ushort)(ushort)34922;
            p130.width = (ushort)(ushort)18351;
            p130.size = (uint)3663393881U;
            CommunicationChannel.instance.send(p130);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnENCAPSULATED_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seqnr == (ushort)(ushort)26899);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)22, (byte)133, (byte)129, (byte)25, (byte)78, (byte)53, (byte)5, (byte)141, (byte)241, (byte)245, (byte)75, (byte)91, (byte)42, (byte)66, (byte)121, (byte)81, (byte)70, (byte)184, (byte)239, (byte)243, (byte)121, (byte)1, (byte)120, (byte)115, (byte)25, (byte)181, (byte)68, (byte)87, (byte)234, (byte)200, (byte)52, (byte)107, (byte)241, (byte)97, (byte)8, (byte)19, (byte)240, (byte)3, (byte)131, (byte)101, (byte)186, (byte)122, (byte)34, (byte)97, (byte)168, (byte)76, (byte)140, (byte)84, (byte)96, (byte)225, (byte)197, (byte)99, (byte)130, (byte)144, (byte)129, (byte)254, (byte)169, (byte)173, (byte)212, (byte)205, (byte)223, (byte)39, (byte)82, (byte)132, (byte)150, (byte)211, (byte)231, (byte)214, (byte)87, (byte)59, (byte)191, (byte)83, (byte)74, (byte)227, (byte)87, (byte)222, (byte)45, (byte)186, (byte)96, (byte)240, (byte)29, (byte)170, (byte)157, (byte)253, (byte)177, (byte)94, (byte)177, (byte)127, (byte)155, (byte)194, (byte)150, (byte)101, (byte)177, (byte)229, (byte)43, (byte)74, (byte)7, (byte)160, (byte)174, (byte)2, (byte)142, (byte)213, (byte)54, (byte)86, (byte)0, (byte)203, (byte)120, (byte)163, (byte)206, (byte)13, (byte)4, (byte)53, (byte)105, (byte)160, (byte)180, (byte)11, (byte)188, (byte)119, (byte)174, (byte)188, (byte)203, (byte)33, (byte)0, (byte)72, (byte)9, (byte)235, (byte)30, (byte)129, (byte)70, (byte)4, (byte)114, (byte)92, (byte)242, (byte)6, (byte)195, (byte)72, (byte)224, (byte)40, (byte)37, (byte)114, (byte)28, (byte)126, (byte)154, (byte)200, (byte)216, (byte)81, (byte)179, (byte)101, (byte)45, (byte)235, (byte)190, (byte)86, (byte)68, (byte)109, (byte)211, (byte)242, (byte)67, (byte)250, (byte)239, (byte)249, (byte)50, (byte)54, (byte)42, (byte)29, (byte)14, (byte)39, (byte)33, (byte)215, (byte)139, (byte)213, (byte)125, (byte)152, (byte)149, (byte)146, (byte)147, (byte)131, (byte)16, (byte)187, (byte)133, (byte)144, (byte)48, (byte)253, (byte)135, (byte)255, (byte)151, (byte)192, (byte)152, (byte)16, (byte)32, (byte)185, (byte)124, (byte)52, (byte)49, (byte)235, (byte)48, (byte)81, (byte)62, (byte)15, (byte)171, (byte)236, (byte)200, (byte)80, (byte)200, (byte)31, (byte)1, (byte)244, (byte)203, (byte)153, (byte)142, (byte)196, (byte)229, (byte)147, (byte)15, (byte)165, (byte)38, (byte)192, (byte)210, (byte)244, (byte)145, (byte)27, (byte)116, (byte)157, (byte)2, (byte)243, (byte)72, (byte)92, (byte)109, (byte)13, (byte)151, (byte)146, (byte)105, (byte)95, (byte)228, (byte)141, (byte)165, (byte)48, (byte)36, (byte)106, (byte)111, (byte)65, (byte)224, (byte)247, (byte)41, (byte)71, (byte)50, (byte)243, (byte)176, (byte)22, (byte)250, (byte)54, (byte)26, (byte)213, (byte)35}));
            };
            GroundControl.ENCAPSULATED_DATA p131 = CommunicationChannel.new_ENCAPSULATED_DATA();
            PH.setPack(p131);
            p131.data__SET(new byte[] {(byte)22, (byte)133, (byte)129, (byte)25, (byte)78, (byte)53, (byte)5, (byte)141, (byte)241, (byte)245, (byte)75, (byte)91, (byte)42, (byte)66, (byte)121, (byte)81, (byte)70, (byte)184, (byte)239, (byte)243, (byte)121, (byte)1, (byte)120, (byte)115, (byte)25, (byte)181, (byte)68, (byte)87, (byte)234, (byte)200, (byte)52, (byte)107, (byte)241, (byte)97, (byte)8, (byte)19, (byte)240, (byte)3, (byte)131, (byte)101, (byte)186, (byte)122, (byte)34, (byte)97, (byte)168, (byte)76, (byte)140, (byte)84, (byte)96, (byte)225, (byte)197, (byte)99, (byte)130, (byte)144, (byte)129, (byte)254, (byte)169, (byte)173, (byte)212, (byte)205, (byte)223, (byte)39, (byte)82, (byte)132, (byte)150, (byte)211, (byte)231, (byte)214, (byte)87, (byte)59, (byte)191, (byte)83, (byte)74, (byte)227, (byte)87, (byte)222, (byte)45, (byte)186, (byte)96, (byte)240, (byte)29, (byte)170, (byte)157, (byte)253, (byte)177, (byte)94, (byte)177, (byte)127, (byte)155, (byte)194, (byte)150, (byte)101, (byte)177, (byte)229, (byte)43, (byte)74, (byte)7, (byte)160, (byte)174, (byte)2, (byte)142, (byte)213, (byte)54, (byte)86, (byte)0, (byte)203, (byte)120, (byte)163, (byte)206, (byte)13, (byte)4, (byte)53, (byte)105, (byte)160, (byte)180, (byte)11, (byte)188, (byte)119, (byte)174, (byte)188, (byte)203, (byte)33, (byte)0, (byte)72, (byte)9, (byte)235, (byte)30, (byte)129, (byte)70, (byte)4, (byte)114, (byte)92, (byte)242, (byte)6, (byte)195, (byte)72, (byte)224, (byte)40, (byte)37, (byte)114, (byte)28, (byte)126, (byte)154, (byte)200, (byte)216, (byte)81, (byte)179, (byte)101, (byte)45, (byte)235, (byte)190, (byte)86, (byte)68, (byte)109, (byte)211, (byte)242, (byte)67, (byte)250, (byte)239, (byte)249, (byte)50, (byte)54, (byte)42, (byte)29, (byte)14, (byte)39, (byte)33, (byte)215, (byte)139, (byte)213, (byte)125, (byte)152, (byte)149, (byte)146, (byte)147, (byte)131, (byte)16, (byte)187, (byte)133, (byte)144, (byte)48, (byte)253, (byte)135, (byte)255, (byte)151, (byte)192, (byte)152, (byte)16, (byte)32, (byte)185, (byte)124, (byte)52, (byte)49, (byte)235, (byte)48, (byte)81, (byte)62, (byte)15, (byte)171, (byte)236, (byte)200, (byte)80, (byte)200, (byte)31, (byte)1, (byte)244, (byte)203, (byte)153, (byte)142, (byte)196, (byte)229, (byte)147, (byte)15, (byte)165, (byte)38, (byte)192, (byte)210, (byte)244, (byte)145, (byte)27, (byte)116, (byte)157, (byte)2, (byte)243, (byte)72, (byte)92, (byte)109, (byte)13, (byte)151, (byte)146, (byte)105, (byte)95, (byte)228, (byte)141, (byte)165, (byte)48, (byte)36, (byte)106, (byte)111, (byte)65, (byte)224, (byte)247, (byte)41, (byte)71, (byte)50, (byte)243, (byte)176, (byte)22, (byte)250, (byte)54, (byte)26, (byte)213, (byte)35}, 0) ;
            p131.seqnr = (ushort)(ushort)26899;
            CommunicationChannel.instance.send(p131);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnDISTANCE_SENSORReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.current_distance == (ushort)(ushort)25284);
                Debug.Assert(pack.covariance == (byte)(byte)136);
                Debug.Assert(pack.min_distance == (ushort)(ushort)37953);
                Debug.Assert(pack.id == (byte)(byte)105);
                Debug.Assert(pack.max_distance == (ushort)(ushort)12641);
                Debug.Assert(pack.orientation == MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_YAW_135);
                Debug.Assert(pack.time_boot_ms == (uint)1968536129U);
                Debug.Assert(pack.type == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_ULTRASOUND);
            };
            GroundControl.DISTANCE_SENSOR p132 = CommunicationChannel.new_DISTANCE_SENSOR();
            PH.setPack(p132);
            p132.time_boot_ms = (uint)1968536129U;
            p132.covariance = (byte)(byte)136;
            p132.min_distance = (ushort)(ushort)37953;
            p132.max_distance = (ushort)(ushort)12641;
            p132.type = MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_ULTRASOUND;
            p132.current_distance = (ushort)(ushort)25284;
            p132.id = (byte)(byte)105;
            p132.orientation = MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_YAW_135;
            CommunicationChannel.instance.send(p132);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTERRAIN_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mask == (ulong)6655821904553460652L);
                Debug.Assert(pack.lat == (int) -1094597273);
                Debug.Assert(pack.grid_spacing == (ushort)(ushort)15867);
                Debug.Assert(pack.lon == (int) -723146622);
            };
            GroundControl.TERRAIN_REQUEST p133 = CommunicationChannel.new_TERRAIN_REQUEST();
            PH.setPack(p133);
            p133.lon = (int) -723146622;
            p133.grid_spacing = (ushort)(ushort)15867;
            p133.lat = (int) -1094597273;
            p133.mask = (ulong)6655821904553460652L;
            CommunicationChannel.instance.send(p133);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTERRAIN_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lon == (int) -1838329817);
                Debug.Assert(pack.grid_spacing == (ushort)(ushort)6620);
                Debug.Assert(pack.lat == (int)306071266);
                Debug.Assert(pack.gridbit == (byte)(byte)168);
                Debug.Assert(pack.data_.SequenceEqual(new short[] {(short)20774, (short)14898, (short) -18944, (short)21319, (short)21960, (short)27828, (short)2664, (short) -8414, (short) -17635, (short)19478, (short) -28753, (short) -20322, (short) -28051, (short) -27748, (short) -22497, (short) -27246}));
            };
            GroundControl.TERRAIN_DATA p134 = CommunicationChannel.new_TERRAIN_DATA();
            PH.setPack(p134);
            p134.lat = (int)306071266;
            p134.lon = (int) -1838329817;
            p134.gridbit = (byte)(byte)168;
            p134.data__SET(new short[] {(short)20774, (short)14898, (short) -18944, (short)21319, (short)21960, (short)27828, (short)2664, (short) -8414, (short) -17635, (short)19478, (short) -28753, (short) -20322, (short) -28051, (short) -27748, (short) -22497, (short) -27246}, 0) ;
            p134.grid_spacing = (ushort)(ushort)6620;
            CommunicationChannel.instance.send(p134);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTERRAIN_CHECKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lat == (int) -1730033571);
                Debug.Assert(pack.lon == (int) -176729776);
            };
            GroundControl.TERRAIN_CHECK p135 = CommunicationChannel.new_TERRAIN_CHECK();
            PH.setPack(p135);
            p135.lon = (int) -176729776;
            p135.lat = (int) -1730033571;
            CommunicationChannel.instance.send(p135);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTERRAIN_REPORTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lon == (int) -1133200238);
                Debug.Assert(pack.terrain_height == (float) -4.9406353E35F);
                Debug.Assert(pack.pending == (ushort)(ushort)28727);
                Debug.Assert(pack.lat == (int) -624241799);
                Debug.Assert(pack.loaded == (ushort)(ushort)27175);
                Debug.Assert(pack.current_height == (float)1.3441326E38F);
                Debug.Assert(pack.spacing == (ushort)(ushort)28303);
            };
            GroundControl.TERRAIN_REPORT p136 = CommunicationChannel.new_TERRAIN_REPORT();
            PH.setPack(p136);
            p136.spacing = (ushort)(ushort)28303;
            p136.current_height = (float)1.3441326E38F;
            p136.loaded = (ushort)(ushort)27175;
            p136.terrain_height = (float) -4.9406353E35F;
            p136.pending = (ushort)(ushort)28727;
            p136.lon = (int) -1133200238;
            p136.lat = (int) -624241799;
            CommunicationChannel.instance.send(p136);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_PRESSURE2Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.press_diff == (float) -3.3448819E38F);
                Debug.Assert(pack.press_abs == (float)1.4504381E38F);
                Debug.Assert(pack.time_boot_ms == (uint)894029067U);
                Debug.Assert(pack.temperature == (short)(short) -6810);
            };
            GroundControl.SCALED_PRESSURE2 p137 = CommunicationChannel.new_SCALED_PRESSURE2();
            PH.setPack(p137);
            p137.time_boot_ms = (uint)894029067U;
            p137.press_diff = (float) -3.3448819E38F;
            p137.temperature = (short)(short) -6810;
            p137.press_abs = (float)1.4504381E38F;
            CommunicationChannel.instance.send(p137);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATT_POS_MOCAPReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)5144540343465170846L);
                Debug.Assert(pack.z == (float) -9.393277E37F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-1.8684752E38F, 2.3992042E37F, -6.8349327E37F, -3.3574917E38F}));
                Debug.Assert(pack.y == (float)2.6509602E38F);
                Debug.Assert(pack.x == (float) -6.3223967E37F);
            };
            GroundControl.ATT_POS_MOCAP p138 = CommunicationChannel.new_ATT_POS_MOCAP();
            PH.setPack(p138);
            p138.time_usec = (ulong)5144540343465170846L;
            p138.q_SET(new float[] {-1.8684752E38F, 2.3992042E37F, -6.8349327E37F, -3.3574917E38F}, 0) ;
            p138.x = (float) -6.3223967E37F;
            p138.y = (float)2.6509602E38F;
            p138.z = (float) -9.393277E37F;
            CommunicationChannel.instance.send(p138);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_ACTUATOR_CONTROL_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)175);
                Debug.Assert(pack.time_usec == (ulong)4427314325068555540L);
                Debug.Assert(pack.target_component == (byte)(byte)162);
                Debug.Assert(pack.group_mlx == (byte)(byte)192);
                Debug.Assert(pack.controls.SequenceEqual(new float[] {-2.7596352E38F, -3.2204933E38F, -2.9751895E38F, -6.8236866E37F, 2.858747E38F, -2.2170304E38F, 1.01228416E37F, 4.63085E36F}));
            };
            GroundControl.SET_ACTUATOR_CONTROL_TARGET p139 = CommunicationChannel.new_SET_ACTUATOR_CONTROL_TARGET();
            PH.setPack(p139);
            p139.controls_SET(new float[] {-2.7596352E38F, -3.2204933E38F, -2.9751895E38F, -6.8236866E37F, 2.858747E38F, -2.2170304E38F, 1.01228416E37F, 4.63085E36F}, 0) ;
            p139.target_system = (byte)(byte)175;
            p139.time_usec = (ulong)4427314325068555540L;
            p139.group_mlx = (byte)(byte)192;
            p139.target_component = (byte)(byte)162;
            CommunicationChannel.instance.send(p139);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnACTUATOR_CONTROL_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)4749320348597289752L);
                Debug.Assert(pack.controls.SequenceEqual(new float[] {1.4355925E38F, -3.1765876E38F, 2.4065448E38F, 1.6339218E38F, 9.469436E37F, -2.955248E38F, -1.55822E37F, 2.929442E38F}));
                Debug.Assert(pack.group_mlx == (byte)(byte)202);
            };
            GroundControl.ACTUATOR_CONTROL_TARGET p140 = CommunicationChannel.new_ACTUATOR_CONTROL_TARGET();
            PH.setPack(p140);
            p140.controls_SET(new float[] {1.4355925E38F, -3.1765876E38F, 2.4065448E38F, 1.6339218E38F, 9.469436E37F, -2.955248E38F, -1.55822E37F, 2.929442E38F}, 0) ;
            p140.time_usec = (ulong)4749320348597289752L;
            p140.group_mlx = (byte)(byte)202;
            CommunicationChannel.instance.send(p140);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnALTITUDEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)8209759499997120124L);
                Debug.Assert(pack.altitude_relative == (float)9.662935E37F);
                Debug.Assert(pack.altitude_amsl == (float)9.698215E37F);
                Debug.Assert(pack.altitude_monotonic == (float)2.9371152E38F);
                Debug.Assert(pack.altitude_local == (float) -1.1247193E37F);
                Debug.Assert(pack.bottom_clearance == (float)2.6403913E38F);
                Debug.Assert(pack.altitude_terrain == (float) -1.3167649E38F);
            };
            GroundControl.ALTITUDE p141 = CommunicationChannel.new_ALTITUDE();
            PH.setPack(p141);
            p141.altitude_relative = (float)9.662935E37F;
            p141.altitude_monotonic = (float)2.9371152E38F;
            p141.time_usec = (ulong)8209759499997120124L;
            p141.altitude_terrain = (float) -1.3167649E38F;
            p141.bottom_clearance = (float)2.6403913E38F;
            p141.altitude_local = (float) -1.1247193E37F;
            p141.altitude_amsl = (float)9.698215E37F;
            CommunicationChannel.instance.send(p141);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRESOURCE_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.transfer_type == (byte)(byte)122);
                Debug.Assert(pack.uri_type == (byte)(byte)73);
                Debug.Assert(pack.uri.SequenceEqual(new byte[] {(byte)26, (byte)162, (byte)65, (byte)204, (byte)174, (byte)165, (byte)63, (byte)231, (byte)170, (byte)239, (byte)35, (byte)27, (byte)77, (byte)230, (byte)164, (byte)151, (byte)16, (byte)162, (byte)209, (byte)14, (byte)168, (byte)222, (byte)159, (byte)190, (byte)4, (byte)197, (byte)195, (byte)195, (byte)92, (byte)202, (byte)92, (byte)246, (byte)37, (byte)117, (byte)90, (byte)205, (byte)123, (byte)94, (byte)42, (byte)101, (byte)229, (byte)22, (byte)225, (byte)40, (byte)61, (byte)247, (byte)38, (byte)174, (byte)139, (byte)207, (byte)80, (byte)202, (byte)162, (byte)203, (byte)128, (byte)94, (byte)46, (byte)254, (byte)181, (byte)90, (byte)181, (byte)76, (byte)16, (byte)34, (byte)190, (byte)120, (byte)28, (byte)26, (byte)131, (byte)17, (byte)71, (byte)10, (byte)236, (byte)222, (byte)55, (byte)32, (byte)129, (byte)122, (byte)175, (byte)51, (byte)96, (byte)217, (byte)219, (byte)78, (byte)237, (byte)229, (byte)108, (byte)160, (byte)167, (byte)89, (byte)94, (byte)77, (byte)29, (byte)212, (byte)183, (byte)226, (byte)162, (byte)6, (byte)54, (byte)150, (byte)185, (byte)21, (byte)81, (byte)97, (byte)249, (byte)227, (byte)63, (byte)50, (byte)189, (byte)101, (byte)232, (byte)14, (byte)31, (byte)13, (byte)109, (byte)238, (byte)127, (byte)182, (byte)250, (byte)122}));
                Debug.Assert(pack.storage.SequenceEqual(new byte[] {(byte)132, (byte)165, (byte)170, (byte)185, (byte)197, (byte)145, (byte)224, (byte)219, (byte)96, (byte)3, (byte)111, (byte)206, (byte)227, (byte)109, (byte)12, (byte)90, (byte)253, (byte)245, (byte)92, (byte)202, (byte)171, (byte)210, (byte)52, (byte)177, (byte)24, (byte)48, (byte)149, (byte)129, (byte)230, (byte)80, (byte)150, (byte)246, (byte)1, (byte)71, (byte)154, (byte)174, (byte)107, (byte)248, (byte)71, (byte)227, (byte)180, (byte)180, (byte)240, (byte)170, (byte)113, (byte)21, (byte)62, (byte)142, (byte)17, (byte)233, (byte)108, (byte)43, (byte)89, (byte)212, (byte)11, (byte)1, (byte)168, (byte)48, (byte)181, (byte)231, (byte)148, (byte)207, (byte)199, (byte)95, (byte)100, (byte)34, (byte)173, (byte)85, (byte)99, (byte)102, (byte)145, (byte)244, (byte)175, (byte)206, (byte)73, (byte)152, (byte)105, (byte)86, (byte)157, (byte)237, (byte)68, (byte)254, (byte)200, (byte)17, (byte)217, (byte)79, (byte)105, (byte)48, (byte)88, (byte)230, (byte)41, (byte)206, (byte)129, (byte)217, (byte)156, (byte)43, (byte)85, (byte)25, (byte)170, (byte)247, (byte)36, (byte)196, (byte)38, (byte)127, (byte)124, (byte)61, (byte)48, (byte)172, (byte)176, (byte)5, (byte)254, (byte)180, (byte)33, (byte)249, (byte)105, (byte)194, (byte)21, (byte)25, (byte)108, (byte)107}));
                Debug.Assert(pack.request_id == (byte)(byte)224);
            };
            GroundControl.RESOURCE_REQUEST p142 = CommunicationChannel.new_RESOURCE_REQUEST();
            PH.setPack(p142);
            p142.request_id = (byte)(byte)224;
            p142.uri_SET(new byte[] {(byte)26, (byte)162, (byte)65, (byte)204, (byte)174, (byte)165, (byte)63, (byte)231, (byte)170, (byte)239, (byte)35, (byte)27, (byte)77, (byte)230, (byte)164, (byte)151, (byte)16, (byte)162, (byte)209, (byte)14, (byte)168, (byte)222, (byte)159, (byte)190, (byte)4, (byte)197, (byte)195, (byte)195, (byte)92, (byte)202, (byte)92, (byte)246, (byte)37, (byte)117, (byte)90, (byte)205, (byte)123, (byte)94, (byte)42, (byte)101, (byte)229, (byte)22, (byte)225, (byte)40, (byte)61, (byte)247, (byte)38, (byte)174, (byte)139, (byte)207, (byte)80, (byte)202, (byte)162, (byte)203, (byte)128, (byte)94, (byte)46, (byte)254, (byte)181, (byte)90, (byte)181, (byte)76, (byte)16, (byte)34, (byte)190, (byte)120, (byte)28, (byte)26, (byte)131, (byte)17, (byte)71, (byte)10, (byte)236, (byte)222, (byte)55, (byte)32, (byte)129, (byte)122, (byte)175, (byte)51, (byte)96, (byte)217, (byte)219, (byte)78, (byte)237, (byte)229, (byte)108, (byte)160, (byte)167, (byte)89, (byte)94, (byte)77, (byte)29, (byte)212, (byte)183, (byte)226, (byte)162, (byte)6, (byte)54, (byte)150, (byte)185, (byte)21, (byte)81, (byte)97, (byte)249, (byte)227, (byte)63, (byte)50, (byte)189, (byte)101, (byte)232, (byte)14, (byte)31, (byte)13, (byte)109, (byte)238, (byte)127, (byte)182, (byte)250, (byte)122}, 0) ;
            p142.uri_type = (byte)(byte)73;
            p142.storage_SET(new byte[] {(byte)132, (byte)165, (byte)170, (byte)185, (byte)197, (byte)145, (byte)224, (byte)219, (byte)96, (byte)3, (byte)111, (byte)206, (byte)227, (byte)109, (byte)12, (byte)90, (byte)253, (byte)245, (byte)92, (byte)202, (byte)171, (byte)210, (byte)52, (byte)177, (byte)24, (byte)48, (byte)149, (byte)129, (byte)230, (byte)80, (byte)150, (byte)246, (byte)1, (byte)71, (byte)154, (byte)174, (byte)107, (byte)248, (byte)71, (byte)227, (byte)180, (byte)180, (byte)240, (byte)170, (byte)113, (byte)21, (byte)62, (byte)142, (byte)17, (byte)233, (byte)108, (byte)43, (byte)89, (byte)212, (byte)11, (byte)1, (byte)168, (byte)48, (byte)181, (byte)231, (byte)148, (byte)207, (byte)199, (byte)95, (byte)100, (byte)34, (byte)173, (byte)85, (byte)99, (byte)102, (byte)145, (byte)244, (byte)175, (byte)206, (byte)73, (byte)152, (byte)105, (byte)86, (byte)157, (byte)237, (byte)68, (byte)254, (byte)200, (byte)17, (byte)217, (byte)79, (byte)105, (byte)48, (byte)88, (byte)230, (byte)41, (byte)206, (byte)129, (byte)217, (byte)156, (byte)43, (byte)85, (byte)25, (byte)170, (byte)247, (byte)36, (byte)196, (byte)38, (byte)127, (byte)124, (byte)61, (byte)48, (byte)172, (byte)176, (byte)5, (byte)254, (byte)180, (byte)33, (byte)249, (byte)105, (byte)194, (byte)21, (byte)25, (byte)108, (byte)107}, 0) ;
            p142.transfer_type = (byte)(byte)122;
            CommunicationChannel.instance.send(p142);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_PRESSURE3Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.press_abs == (float) -2.60563E38F);
                Debug.Assert(pack.press_diff == (float)2.8283133E38F);
                Debug.Assert(pack.temperature == (short)(short)27279);
                Debug.Assert(pack.time_boot_ms == (uint)2839682091U);
            };
            GroundControl.SCALED_PRESSURE3 p143 = CommunicationChannel.new_SCALED_PRESSURE3();
            PH.setPack(p143);
            p143.time_boot_ms = (uint)2839682091U;
            p143.temperature = (short)(short)27279;
            p143.press_abs = (float) -2.60563E38F;
            p143.press_diff = (float)2.8283133E38F;
            CommunicationChannel.instance.send(p143);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnFOLLOW_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.est_capabilities == (byte)(byte)233);
                Debug.Assert(pack.position_cov.SequenceEqual(new float[] {2.0642393E37F, -1.7849752E38F, 4.832787E37F}));
                Debug.Assert(pack.acc.SequenceEqual(new float[] {2.6616125E38F, 8.784549E37F, 2.17512E38F}));
                Debug.Assert(pack.rates.SequenceEqual(new float[] {3.9278207E37F, 7.6621467E37F, 1.5399218E38F}));
                Debug.Assert(pack.lon == (int) -671129190);
                Debug.Assert(pack.attitude_q.SequenceEqual(new float[] {1.2992083E38F, -1.2581155E38F, -2.5081145E38F, -1.7598763E38F}));
                Debug.Assert(pack.vel.SequenceEqual(new float[] {2.7594425E38F, 1.7494483E38F, 5.621538E37F}));
                Debug.Assert(pack.timestamp == (ulong)8564561888460463021L);
                Debug.Assert(pack.lat == (int)1965630413);
                Debug.Assert(pack.alt == (float) -6.4937785E37F);
                Debug.Assert(pack.custom_state == (ulong)8715470927838078993L);
            };
            GroundControl.FOLLOW_TARGET p144 = CommunicationChannel.new_FOLLOW_TARGET();
            PH.setPack(p144);
            p144.custom_state = (ulong)8715470927838078993L;
            p144.est_capabilities = (byte)(byte)233;
            p144.vel_SET(new float[] {2.7594425E38F, 1.7494483E38F, 5.621538E37F}, 0) ;
            p144.lon = (int) -671129190;
            p144.lat = (int)1965630413;
            p144.alt = (float) -6.4937785E37F;
            p144.rates_SET(new float[] {3.9278207E37F, 7.6621467E37F, 1.5399218E38F}, 0) ;
            p144.attitude_q_SET(new float[] {1.2992083E38F, -1.2581155E38F, -2.5081145E38F, -1.7598763E38F}, 0) ;
            p144.timestamp = (ulong)8564561888460463021L;
            p144.position_cov_SET(new float[] {2.0642393E37F, -1.7849752E38F, 4.832787E37F}, 0) ;
            p144.acc_SET(new float[] {2.6616125E38F, 8.784549E37F, 2.17512E38F}, 0) ;
            CommunicationChannel.instance.send(p144);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCONTROL_SYSTEM_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vel_variance.SequenceEqual(new float[] {-2.6029658E38F, 4.2664287E37F, 7.5959297E37F}));
                Debug.Assert(pack.time_usec == (ulong)2231921740737346458L);
                Debug.Assert(pack.yaw_rate == (float)1.145591E37F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {9.6566796E36F, 2.759512E38F, 4.7869935E37F, 5.128379E37F}));
                Debug.Assert(pack.pitch_rate == (float) -2.5066876E38F);
                Debug.Assert(pack.x_vel == (float)2.560209E38F);
                Debug.Assert(pack.y_pos == (float) -4.454204E37F);
                Debug.Assert(pack.x_acc == (float)1.2753176E37F);
                Debug.Assert(pack.pos_variance.SequenceEqual(new float[] {1.6203726E38F, 7.988222E37F, -2.4628534E38F}));
                Debug.Assert(pack.z_acc == (float) -1.7560178E38F);
                Debug.Assert(pack.z_vel == (float)2.9631775E38F);
                Debug.Assert(pack.airspeed == (float)1.3301422E38F);
                Debug.Assert(pack.z_pos == (float) -1.4163252E38F);
                Debug.Assert(pack.x_pos == (float)1.6550398E38F);
                Debug.Assert(pack.y_acc == (float)6.578951E37F);
                Debug.Assert(pack.roll_rate == (float) -1.6995642E38F);
                Debug.Assert(pack.y_vel == (float)2.0083476E38F);
            };
            GroundControl.CONTROL_SYSTEM_STATE p146 = CommunicationChannel.new_CONTROL_SYSTEM_STATE();
            PH.setPack(p146);
            p146.yaw_rate = (float)1.145591E37F;
            p146.pos_variance_SET(new float[] {1.6203726E38F, 7.988222E37F, -2.4628534E38F}, 0) ;
            p146.y_vel = (float)2.0083476E38F;
            p146.z_vel = (float)2.9631775E38F;
            p146.x_vel = (float)2.560209E38F;
            p146.z_acc = (float) -1.7560178E38F;
            p146.y_pos = (float) -4.454204E37F;
            p146.x_acc = (float)1.2753176E37F;
            p146.airspeed = (float)1.3301422E38F;
            p146.x_pos = (float)1.6550398E38F;
            p146.time_usec = (ulong)2231921740737346458L;
            p146.q_SET(new float[] {9.6566796E36F, 2.759512E38F, 4.7869935E37F, 5.128379E37F}, 0) ;
            p146.y_acc = (float)6.578951E37F;
            p146.pitch_rate = (float) -2.5066876E38F;
            p146.roll_rate = (float) -1.6995642E38F;
            p146.z_pos = (float) -1.4163252E38F;
            p146.vel_variance_SET(new float[] {-2.6029658E38F, 4.2664287E37F, 7.5959297E37F}, 0) ;
            CommunicationChannel.instance.send(p146);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnBATTERY_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.type == MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_UNKNOWN);
                Debug.Assert(pack.energy_consumed == (int)1334509310);
                Debug.Assert(pack.temperature == (short)(short) -1297);
                Debug.Assert(pack.battery_remaining == (sbyte)(sbyte) - 89);
                Debug.Assert(pack.current_battery == (short)(short) -17851);
                Debug.Assert(pack.id == (byte)(byte)63);
                Debug.Assert(pack.voltages.SequenceEqual(new ushort[] {(ushort)19895, (ushort)6415, (ushort)58615, (ushort)15593, (ushort)30752, (ushort)32011, (ushort)8610, (ushort)20040, (ushort)56343, (ushort)32203}));
                Debug.Assert(pack.current_consumed == (int)661319174);
                Debug.Assert(pack.battery_function == MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_ALL);
            };
            GroundControl.BATTERY_STATUS p147 = CommunicationChannel.new_BATTERY_STATUS();
            PH.setPack(p147);
            p147.current_consumed = (int)661319174;
            p147.battery_function = MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_ALL;
            p147.battery_remaining = (sbyte)(sbyte) - 89;
            p147.energy_consumed = (int)1334509310;
            p147.voltages_SET(new ushort[] {(ushort)19895, (ushort)6415, (ushort)58615, (ushort)15593, (ushort)30752, (ushort)32011, (ushort)8610, (ushort)20040, (ushort)56343, (ushort)32203}, 0) ;
            p147.temperature = (short)(short) -1297;
            p147.current_battery = (short)(short) -17851;
            p147.type = MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_UNKNOWN;
            p147.id = (byte)(byte)63;
            CommunicationChannel.instance.send(p147);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnAUTOPILOT_VERSIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.flight_custom_version.SequenceEqual(new byte[] {(byte)89, (byte)18, (byte)169, (byte)57, (byte)52, (byte)201, (byte)249, (byte)144}));
                Debug.Assert(pack.uid2_TRY(ph).SequenceEqual(new byte[] {(byte)16, (byte)111, (byte)33, (byte)254, (byte)98, (byte)134, (byte)188, (byte)252, (byte)203, (byte)91, (byte)197, (byte)4, (byte)217, (byte)219, (byte)139, (byte)183, (byte)190, (byte)95}));
                Debug.Assert(pack.capabilities == (MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_PARAM_UNION |
                                                   MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_TERRAIN |
                                                   MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FTP |
                                                   MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FLIGHT_INFORMATION));
                Debug.Assert(pack.middleware_custom_version.SequenceEqual(new byte[] {(byte)236, (byte)216, (byte)59, (byte)123, (byte)41, (byte)43, (byte)138, (byte)29}));
                Debug.Assert(pack.os_custom_version.SequenceEqual(new byte[] {(byte)126, (byte)83, (byte)74, (byte)40, (byte)46, (byte)230, (byte)188, (byte)34}));
                Debug.Assert(pack.product_id == (ushort)(ushort)32467);
                Debug.Assert(pack.board_version == (uint)4008327102U);
                Debug.Assert(pack.middleware_sw_version == (uint)2135091058U);
                Debug.Assert(pack.os_sw_version == (uint)1603267613U);
                Debug.Assert(pack.uid == (ulong)6861160119850191312L);
                Debug.Assert(pack.flight_sw_version == (uint)887339696U);
                Debug.Assert(pack.vendor_id == (ushort)(ushort)13461);
            };
            GroundControl.AUTOPILOT_VERSION p148 = CommunicationChannel.new_AUTOPILOT_VERSION();
            PH.setPack(p148);
            p148.flight_custom_version_SET(new byte[] {(byte)89, (byte)18, (byte)169, (byte)57, (byte)52, (byte)201, (byte)249, (byte)144}, 0) ;
            p148.board_version = (uint)4008327102U;
            p148.flight_sw_version = (uint)887339696U;
            p148.middleware_custom_version_SET(new byte[] {(byte)236, (byte)216, (byte)59, (byte)123, (byte)41, (byte)43, (byte)138, (byte)29}, 0) ;
            p148.uid2_SET(new byte[] {(byte)16, (byte)111, (byte)33, (byte)254, (byte)98, (byte)134, (byte)188, (byte)252, (byte)203, (byte)91, (byte)197, (byte)4, (byte)217, (byte)219, (byte)139, (byte)183, (byte)190, (byte)95}, 0, PH) ;
            p148.os_sw_version = (uint)1603267613U;
            p148.product_id = (ushort)(ushort)32467;
            p148.capabilities = (MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_PARAM_UNION |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_TERRAIN |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FTP |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FLIGHT_INFORMATION);
            p148.uid = (ulong)6861160119850191312L;
            p148.middleware_sw_version = (uint)2135091058U;
            p148.vendor_id = (ushort)(ushort)13461;
            p148.os_custom_version_SET(new byte[] {(byte)126, (byte)83, (byte)74, (byte)40, (byte)46, (byte)230, (byte)188, (byte)34}, 0) ;
            CommunicationChannel.instance.send(p148);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLANDING_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.x_TRY(ph) == (float)6.504098E37F);
                Debug.Assert(pack.time_usec == (ulong)3466801147625564620L);
                Debug.Assert(pack.distance == (float) -3.044995E38F);
                Debug.Assert(pack.size_x == (float)2.899972E38F);
                Debug.Assert(pack.position_valid_TRY(ph) == (byte)(byte)107);
                Debug.Assert(pack.frame == MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED);
                Debug.Assert(pack.y_TRY(ph) == (float) -2.0263663E38F);
                Debug.Assert(pack.angle_x == (float)1.0266039E38F);
                Debug.Assert(pack.size_y == (float) -1.5150558E38F);
                Debug.Assert(pack.angle_y == (float)2.1805346E36F);
                Debug.Assert(pack.target_num == (byte)(byte)253);
                Debug.Assert(pack.q_TRY(ph).SequenceEqual(new float[] {-2.902542E38F, -1.8165881E38F, 2.2829438E38F, -2.8603374E38F}));
                Debug.Assert(pack.z_TRY(ph) == (float) -2.111521E38F);
                Debug.Assert(pack.type == LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_VISION_OTHER);
            };
            GroundControl.LANDING_TARGET p149 = CommunicationChannel.new_LANDING_TARGET();
            PH.setPack(p149);
            p149.position_valid_SET((byte)(byte)107, PH) ;
            p149.q_SET(new float[] {-2.902542E38F, -1.8165881E38F, 2.2829438E38F, -2.8603374E38F}, 0, PH) ;
            p149.angle_x = (float)1.0266039E38F;
            p149.time_usec = (ulong)3466801147625564620L;
            p149.size_y = (float) -1.5150558E38F;
            p149.type = LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_VISION_OTHER;
            p149.target_num = (byte)(byte)253;
            p149.z_SET((float) -2.111521E38F, PH) ;
            p149.distance = (float) -3.044995E38F;
            p149.angle_y = (float)2.1805346E36F;
            p149.size_x = (float)2.899972E38F;
            p149.x_SET((float)6.504098E37F, PH) ;
            p149.y_SET((float) -2.0263663E38F, PH) ;
            p149.frame = MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED;
            CommunicationChannel.instance.send(p149);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            ADV_TEST_CH.OnFLEXIFUNCTION_SETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)155);
                Debug.Assert(pack.target_system == (byte)(byte)130);
            };
            GroundControl.FLEXIFUNCTION_SET p150 = CommunicationChannel.new_FLEXIFUNCTION_SET();
            PH.setPack(p150);
            p150.target_component = (byte)(byte)155;
            p150.target_system = (byte)(byte)130;
            CommunicationChannel.instance.send(p150);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnFLEXIFUNCTION_READ_REQReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.read_req_type == (short)(short)17732);
                Debug.Assert(pack.target_system == (byte)(byte)152);
                Debug.Assert(pack.target_component == (byte)(byte)251);
                Debug.Assert(pack.data_index == (short)(short)25526);
            };
            GroundControl.FLEXIFUNCTION_READ_REQ p151 = CommunicationChannel.new_FLEXIFUNCTION_READ_REQ();
            PH.setPack(p151);
            p151.target_component = (byte)(byte)251;
            p151.target_system = (byte)(byte)152;
            p151.data_index = (short)(short)25526;
            p151.read_req_type = (short)(short)17732;
            CommunicationChannel.instance.send(p151);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnFLEXIFUNCTION_BUFFER_FUNCTIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.data_address == (ushort)(ushort)21741);
                Debug.Assert(pack.data_size == (ushort)(ushort)3787);
                Debug.Assert(pack.target_system == (byte)(byte)227);
                Debug.Assert(pack.func_index == (ushort)(ushort)28764);
                Debug.Assert(pack.func_count == (ushort)(ushort)17203);
                Debug.Assert(pack.data_.SequenceEqual(new sbyte[] {(sbyte)125, (sbyte) - 113, (sbyte) - 63, (sbyte) - 71, (sbyte) - 6, (sbyte) - 2, (sbyte)111, (sbyte) - 34, (sbyte) - 59, (sbyte) - 16, (sbyte)78, (sbyte) - 102, (sbyte)62, (sbyte) - 117, (sbyte) - 121, (sbyte)82, (sbyte) - 99, (sbyte) - 34, (sbyte) - 68, (sbyte) - 111, (sbyte) - 1, (sbyte) - 108, (sbyte)109, (sbyte)112, (sbyte)64, (sbyte)23, (sbyte)35, (sbyte)87, (sbyte) - 120, (sbyte)68, (sbyte)25, (sbyte)25, (sbyte)85, (sbyte)4, (sbyte)83, (sbyte) - 101, (sbyte) - 104, (sbyte)11, (sbyte) - 121, (sbyte) - 122, (sbyte)76, (sbyte) - 2, (sbyte)71, (sbyte) - 6, (sbyte)13, (sbyte) - 38, (sbyte) - 112, (sbyte)110}));
                Debug.Assert(pack.target_component == (byte)(byte)15);
            };
            GroundControl.FLEXIFUNCTION_BUFFER_FUNCTION p152 = CommunicationChannel.new_FLEXIFUNCTION_BUFFER_FUNCTION();
            PH.setPack(p152);
            p152.data_size = (ushort)(ushort)3787;
            p152.data_address = (ushort)(ushort)21741;
            p152.target_system = (byte)(byte)227;
            p152.func_count = (ushort)(ushort)17203;
            p152.target_component = (byte)(byte)15;
            p152.data__SET(new sbyte[] {(sbyte)125, (sbyte) - 113, (sbyte) - 63, (sbyte) - 71, (sbyte) - 6, (sbyte) - 2, (sbyte)111, (sbyte) - 34, (sbyte) - 59, (sbyte) - 16, (sbyte)78, (sbyte) - 102, (sbyte)62, (sbyte) - 117, (sbyte) - 121, (sbyte)82, (sbyte) - 99, (sbyte) - 34, (sbyte) - 68, (sbyte) - 111, (sbyte) - 1, (sbyte) - 108, (sbyte)109, (sbyte)112, (sbyte)64, (sbyte)23, (sbyte)35, (sbyte)87, (sbyte) - 120, (sbyte)68, (sbyte)25, (sbyte)25, (sbyte)85, (sbyte)4, (sbyte)83, (sbyte) - 101, (sbyte) - 104, (sbyte)11, (sbyte) - 121, (sbyte) - 122, (sbyte)76, (sbyte) - 2, (sbyte)71, (sbyte) - 6, (sbyte)13, (sbyte) - 38, (sbyte) - 112, (sbyte)110}, 0) ;
            p152.func_index = (ushort)(ushort)28764;
            CommunicationChannel.instance.send(p152);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnFLEXIFUNCTION_BUFFER_FUNCTION_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.func_index == (ushort)(ushort)38951);
                Debug.Assert(pack.result == (ushort)(ushort)23871);
                Debug.Assert(pack.target_system == (byte)(byte)138);
                Debug.Assert(pack.target_component == (byte)(byte)121);
            };
            GroundControl.FLEXIFUNCTION_BUFFER_FUNCTION_ACK p153 = CommunicationChannel.new_FLEXIFUNCTION_BUFFER_FUNCTION_ACK();
            PH.setPack(p153);
            p153.func_index = (ushort)(ushort)38951;
            p153.target_component = (byte)(byte)121;
            p153.target_system = (byte)(byte)138;
            p153.result = (ushort)(ushort)23871;
            CommunicationChannel.instance.send(p153);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnFLEXIFUNCTION_DIRECTORYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.directory_type == (byte)(byte)153);
                Debug.Assert(pack.target_system == (byte)(byte)160);
                Debug.Assert(pack.count == (byte)(byte)233);
                Debug.Assert(pack.directory_data.SequenceEqual(new sbyte[] {(sbyte)16, (sbyte)96, (sbyte) - 43, (sbyte) - 2, (sbyte)1, (sbyte)45, (sbyte) - 36, (sbyte) - 82, (sbyte)117, (sbyte)46, (sbyte)123, (sbyte) - 55, (sbyte)61, (sbyte)127, (sbyte) - 30, (sbyte)34, (sbyte)43, (sbyte) - 58, (sbyte) - 48, (sbyte) - 27, (sbyte)90, (sbyte)44, (sbyte) - 63, (sbyte)103, (sbyte) - 39, (sbyte) - 19, (sbyte)120, (sbyte)61, (sbyte)39, (sbyte)18, (sbyte) - 116, (sbyte) - 52, (sbyte)124, (sbyte)26, (sbyte)100, (sbyte) - 40, (sbyte) - 25, (sbyte) - 52, (sbyte) - 22, (sbyte) - 16, (sbyte) - 24, (sbyte) - 115, (sbyte)7, (sbyte) - 23, (sbyte)101, (sbyte) - 102, (sbyte)47, (sbyte) - 99}));
                Debug.Assert(pack.target_component == (byte)(byte)85);
                Debug.Assert(pack.start_index == (byte)(byte)143);
            };
            GroundControl.FLEXIFUNCTION_DIRECTORY p155 = CommunicationChannel.new_FLEXIFUNCTION_DIRECTORY();
            PH.setPack(p155);
            p155.start_index = (byte)(byte)143;
            p155.count = (byte)(byte)233;
            p155.target_system = (byte)(byte)160;
            p155.target_component = (byte)(byte)85;
            p155.directory_data_SET(new sbyte[] {(sbyte)16, (sbyte)96, (sbyte) - 43, (sbyte) - 2, (sbyte)1, (sbyte)45, (sbyte) - 36, (sbyte) - 82, (sbyte)117, (sbyte)46, (sbyte)123, (sbyte) - 55, (sbyte)61, (sbyte)127, (sbyte) - 30, (sbyte)34, (sbyte)43, (sbyte) - 58, (sbyte) - 48, (sbyte) - 27, (sbyte)90, (sbyte)44, (sbyte) - 63, (sbyte)103, (sbyte) - 39, (sbyte) - 19, (sbyte)120, (sbyte)61, (sbyte)39, (sbyte)18, (sbyte) - 116, (sbyte) - 52, (sbyte)124, (sbyte)26, (sbyte)100, (sbyte) - 40, (sbyte) - 25, (sbyte) - 52, (sbyte) - 22, (sbyte) - 16, (sbyte) - 24, (sbyte) - 115, (sbyte)7, (sbyte) - 23, (sbyte)101, (sbyte) - 102, (sbyte)47, (sbyte) - 99}, 0) ;
            p155.directory_type = (byte)(byte)153;
            CommunicationChannel.instance.send(p155);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnFLEXIFUNCTION_DIRECTORY_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.start_index == (byte)(byte)74);
                Debug.Assert(pack.target_system == (byte)(byte)109);
                Debug.Assert(pack.result == (ushort)(ushort)41306);
                Debug.Assert(pack.directory_type == (byte)(byte)206);
                Debug.Assert(pack.target_component == (byte)(byte)134);
                Debug.Assert(pack.count == (byte)(byte)251);
            };
            GroundControl.FLEXIFUNCTION_DIRECTORY_ACK p156 = CommunicationChannel.new_FLEXIFUNCTION_DIRECTORY_ACK();
            PH.setPack(p156);
            p156.start_index = (byte)(byte)74;
            p156.target_system = (byte)(byte)109;
            p156.count = (byte)(byte)251;
            p156.result = (ushort)(ushort)41306;
            p156.target_component = (byte)(byte)134;
            p156.directory_type = (byte)(byte)206;
            CommunicationChannel.instance.send(p156);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnFLEXIFUNCTION_COMMANDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)83);
                Debug.Assert(pack.command_type == (byte)(byte)175);
                Debug.Assert(pack.target_component == (byte)(byte)212);
            };
            GroundControl.FLEXIFUNCTION_COMMAND p157 = CommunicationChannel.new_FLEXIFUNCTION_COMMAND();
            PH.setPack(p157);
            p157.target_component = (byte)(byte)212;
            p157.target_system = (byte)(byte)83;
            p157.command_type = (byte)(byte)175;
            CommunicationChannel.instance.send(p157);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnFLEXIFUNCTION_COMMAND_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.command_type == (ushort)(ushort)18908);
                Debug.Assert(pack.result == (ushort)(ushort)29789);
            };
            GroundControl.FLEXIFUNCTION_COMMAND_ACK p158 = CommunicationChannel.new_FLEXIFUNCTION_COMMAND_ACK();
            PH.setPack(p158);
            p158.command_type = (ushort)(ushort)18908;
            p158.result = (ushort)(ushort)29789;
            CommunicationChannel.instance.send(p158);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSERIAL_UDB_EXTRA_F2_AReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sue_rmat2 == (short)(short)18885);
                Debug.Assert(pack.sue_hdop == (short)(short)7);
                Debug.Assert(pack.sue_rmat0 == (short)(short) -13756);
                Debug.Assert(pack.sue_waypoint_index == (ushort)(ushort)47666);
                Debug.Assert(pack.sue_estimated_wind_2 == (short)(short)31402);
                Debug.Assert(pack.sue_rmat8 == (short)(short)6677);
                Debug.Assert(pack.sue_latitude == (int) -299734028);
                Debug.Assert(pack.sue_rmat1 == (short)(short) -29342);
                Debug.Assert(pack.sue_sog == (short)(short)3076);
                Debug.Assert(pack.sue_altitude == (int) -1874672877);
                Debug.Assert(pack.sue_rmat4 == (short)(short)18158);
                Debug.Assert(pack.sue_rmat3 == (short)(short)27878);
                Debug.Assert(pack.sue_estimated_wind_1 == (short)(short)8828);
                Debug.Assert(pack.sue_air_speed_3DIMU == (ushort)(ushort)44491);
                Debug.Assert(pack.sue_rmat5 == (short)(short) -32344);
                Debug.Assert(pack.sue_magFieldEarth1 == (short)(short) -31092);
                Debug.Assert(pack.sue_cpu_load == (ushort)(ushort)14613);
                Debug.Assert(pack.sue_status == (byte)(byte)91);
                Debug.Assert(pack.sue_svs == (short)(short) -7525);
                Debug.Assert(pack.sue_rmat7 == (short)(short) -14732);
                Debug.Assert(pack.sue_cog == (ushort)(ushort)61016);
                Debug.Assert(pack.sue_magFieldEarth2 == (short)(short)15318);
                Debug.Assert(pack.sue_estimated_wind_0 == (short)(short)31842);
                Debug.Assert(pack.sue_longitude == (int)603338026);
                Debug.Assert(pack.sue_time == (uint)1764364568U);
                Debug.Assert(pack.sue_magFieldEarth0 == (short)(short)12017);
                Debug.Assert(pack.sue_rmat6 == (short)(short)18165);
            };
            GroundControl.SERIAL_UDB_EXTRA_F2_A p170 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F2_A();
            PH.setPack(p170);
            p170.sue_rmat7 = (short)(short) -14732;
            p170.sue_hdop = (short)(short)7;
            p170.sue_waypoint_index = (ushort)(ushort)47666;
            p170.sue_rmat3 = (short)(short)27878;
            p170.sue_rmat8 = (short)(short)6677;
            p170.sue_estimated_wind_1 = (short)(short)8828;
            p170.sue_svs = (short)(short) -7525;
            p170.sue_status = (byte)(byte)91;
            p170.sue_estimated_wind_0 = (short)(short)31842;
            p170.sue_rmat0 = (short)(short) -13756;
            p170.sue_rmat5 = (short)(short) -32344;
            p170.sue_estimated_wind_2 = (short)(short)31402;
            p170.sue_sog = (short)(short)3076;
            p170.sue_magFieldEarth1 = (short)(short) -31092;
            p170.sue_latitude = (int) -299734028;
            p170.sue_rmat1 = (short)(short) -29342;
            p170.sue_time = (uint)1764364568U;
            p170.sue_rmat4 = (short)(short)18158;
            p170.sue_cog = (ushort)(ushort)61016;
            p170.sue_air_speed_3DIMU = (ushort)(ushort)44491;
            p170.sue_magFieldEarth0 = (short)(short)12017;
            p170.sue_altitude = (int) -1874672877;
            p170.sue_longitude = (int)603338026;
            p170.sue_cpu_load = (ushort)(ushort)14613;
            p170.sue_rmat2 = (short)(short)18885;
            p170.sue_magFieldEarth2 = (short)(short)15318;
            p170.sue_rmat6 = (short)(short)18165;
            CommunicationChannel.instance.send(p170);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSERIAL_UDB_EXTRA_F2_BReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sue_osc_fails == (short)(short)14087);
                Debug.Assert(pack.sue_pwm_input_8 == (short)(short)8668);
                Debug.Assert(pack.sue_waypoint_goal_y == (short)(short)27611);
                Debug.Assert(pack.sue_location_error_earth_x == (short)(short) -10126);
                Debug.Assert(pack.sue_pwm_input_1 == (short)(short) -17340);
                Debug.Assert(pack.sue_pwm_output_4 == (short)(short) -5430);
                Debug.Assert(pack.sue_pwm_input_12 == (short)(short)7796);
                Debug.Assert(pack.sue_imu_location_z == (short)(short) -11506);
                Debug.Assert(pack.sue_pwm_input_2 == (short)(short)14758);
                Debug.Assert(pack.sue_flags == (uint)728525715U);
                Debug.Assert(pack.sue_pwm_input_6 == (short)(short)14290);
                Debug.Assert(pack.sue_aero_x == (short)(short)24443);
                Debug.Assert(pack.sue_pwm_input_7 == (short)(short)23238);
                Debug.Assert(pack.sue_time == (uint)491427696U);
                Debug.Assert(pack.sue_pwm_output_6 == (short)(short) -26044);
                Debug.Assert(pack.sue_pwm_input_9 == (short)(short)21448);
                Debug.Assert(pack.sue_pwm_output_8 == (short)(short) -3427);
                Debug.Assert(pack.sue_pwm_input_3 == (short)(short) -14970);
                Debug.Assert(pack.sue_imu_velocity_y == (short)(short)14314);
                Debug.Assert(pack.sue_pwm_output_2 == (short)(short) -32465);
                Debug.Assert(pack.sue_barom_press == (int) -379017807);
                Debug.Assert(pack.sue_bat_amp == (short)(short)23457);
                Debug.Assert(pack.sue_imu_location_x == (short)(short) -25117);
                Debug.Assert(pack.sue_imu_location_y == (short)(short) -22885);
                Debug.Assert(pack.sue_waypoint_goal_x == (short)(short)21613);
                Debug.Assert(pack.sue_aero_y == (short)(short)30930);
                Debug.Assert(pack.sue_barom_alt == (int) -1275358099);
                Debug.Assert(pack.sue_pwm_input_11 == (short)(short) -522);
                Debug.Assert(pack.sue_location_error_earth_y == (short)(short) -299);
                Debug.Assert(pack.sue_pwm_input_10 == (short)(short)14119);
                Debug.Assert(pack.sue_pwm_output_11 == (short)(short) -28201);
                Debug.Assert(pack.sue_pwm_output_1 == (short)(short)30793);
                Debug.Assert(pack.sue_bat_amp_hours == (short)(short) -3874);
                Debug.Assert(pack.sue_memory_stack_free == (short)(short) -2891);
                Debug.Assert(pack.sue_barom_temp == (short)(short) -26750);
                Debug.Assert(pack.sue_pwm_output_3 == (short)(short)24360);
                Debug.Assert(pack.sue_desired_height == (short)(short)5295);
                Debug.Assert(pack.sue_imu_velocity_z == (short)(short)31079);
                Debug.Assert(pack.sue_pwm_output_10 == (short)(short)24208);
                Debug.Assert(pack.sue_pwm_output_12 == (short)(short) -6641);
                Debug.Assert(pack.sue_imu_velocity_x == (short)(short) -6573);
                Debug.Assert(pack.sue_bat_volt == (short)(short) -29803);
                Debug.Assert(pack.sue_pwm_output_7 == (short)(short)8831);
                Debug.Assert(pack.sue_pwm_input_5 == (short)(short)11540);
                Debug.Assert(pack.sue_pwm_output_5 == (short)(short) -23102);
                Debug.Assert(pack.sue_pwm_output_9 == (short)(short)2616);
                Debug.Assert(pack.sue_pwm_input_4 == (short)(short) -22799);
                Debug.Assert(pack.sue_aero_z == (short)(short) -16260);
                Debug.Assert(pack.sue_location_error_earth_z == (short)(short)11699);
                Debug.Assert(pack.sue_waypoint_goal_z == (short)(short) -12899);
            };
            GroundControl.SERIAL_UDB_EXTRA_F2_B p171 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F2_B();
            PH.setPack(p171);
            p171.sue_imu_velocity_x = (short)(short) -6573;
            p171.sue_barom_press = (int) -379017807;
            p171.sue_barom_alt = (int) -1275358099;
            p171.sue_location_error_earth_y = (short)(short) -299;
            p171.sue_pwm_output_6 = (short)(short) -26044;
            p171.sue_imu_velocity_y = (short)(short)14314;
            p171.sue_pwm_output_9 = (short)(short)2616;
            p171.sue_pwm_output_8 = (short)(short) -3427;
            p171.sue_pwm_input_6 = (short)(short)14290;
            p171.sue_pwm_output_1 = (short)(short)30793;
            p171.sue_pwm_output_11 = (short)(short) -28201;
            p171.sue_pwm_output_3 = (short)(short)24360;
            p171.sue_osc_fails = (short)(short)14087;
            p171.sue_memory_stack_free = (short)(short) -2891;
            p171.sue_waypoint_goal_x = (short)(short)21613;
            p171.sue_bat_amp_hours = (short)(short) -3874;
            p171.sue_waypoint_goal_z = (short)(short) -12899;
            p171.sue_pwm_input_7 = (short)(short)23238;
            p171.sue_pwm_input_8 = (short)(short)8668;
            p171.sue_pwm_output_12 = (short)(short) -6641;
            p171.sue_time = (uint)491427696U;
            p171.sue_imu_location_y = (short)(short) -22885;
            p171.sue_pwm_input_9 = (short)(short)21448;
            p171.sue_flags = (uint)728525715U;
            p171.sue_pwm_output_7 = (short)(short)8831;
            p171.sue_bat_amp = (short)(short)23457;
            p171.sue_imu_velocity_z = (short)(short)31079;
            p171.sue_barom_temp = (short)(short) -26750;
            p171.sue_pwm_output_10 = (short)(short)24208;
            p171.sue_pwm_output_4 = (short)(short) -5430;
            p171.sue_pwm_input_1 = (short)(short) -17340;
            p171.sue_aero_z = (short)(short) -16260;
            p171.sue_pwm_output_2 = (short)(short) -32465;
            p171.sue_aero_x = (short)(short)24443;
            p171.sue_bat_volt = (short)(short) -29803;
            p171.sue_pwm_input_2 = (short)(short)14758;
            p171.sue_imu_location_z = (short)(short) -11506;
            p171.sue_pwm_input_12 = (short)(short)7796;
            p171.sue_waypoint_goal_y = (short)(short)27611;
            p171.sue_pwm_output_5 = (short)(short) -23102;
            p171.sue_pwm_input_3 = (short)(short) -14970;
            p171.sue_location_error_earth_x = (short)(short) -10126;
            p171.sue_imu_location_x = (short)(short) -25117;
            p171.sue_pwm_input_4 = (short)(short) -22799;
            p171.sue_desired_height = (short)(short)5295;
            p171.sue_location_error_earth_z = (short)(short)11699;
            p171.sue_pwm_input_11 = (short)(short) -522;
            p171.sue_pwm_input_5 = (short)(short)11540;
            p171.sue_aero_y = (short)(short)30930;
            p171.sue_pwm_input_10 = (short)(short)14119;
            CommunicationChannel.instance.send(p171);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSERIAL_UDB_EXTRA_F4Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sue_ALTITUDEHOLD_STABILIZED == (byte)(byte)162);
                Debug.Assert(pack.sue_RUDDER_NAVIGATION == (byte)(byte)196);
                Debug.Assert(pack.sue_RACING_MODE == (byte)(byte)169);
                Debug.Assert(pack.sue_AILERON_NAVIGATION == (byte)(byte)157);
                Debug.Assert(pack.sue_ROLL_STABILIZATION_RUDDER == (byte)(byte)36);
                Debug.Assert(pack.sue_ALTITUDEHOLD_WAYPOINT == (byte)(byte)251);
                Debug.Assert(pack.sue_ROLL_STABILIZATION_AILERONS == (byte)(byte)47);
                Debug.Assert(pack.sue_YAW_STABILIZATION_RUDDER == (byte)(byte)139);
                Debug.Assert(pack.sue_PITCH_STABILIZATION == (byte)(byte)140);
                Debug.Assert(pack.sue_YAW_STABILIZATION_AILERON == (byte)(byte)213);
            };
            GroundControl.SERIAL_UDB_EXTRA_F4 p172 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F4();
            PH.setPack(p172);
            p172.sue_YAW_STABILIZATION_RUDDER = (byte)(byte)139;
            p172.sue_ROLL_STABILIZATION_AILERONS = (byte)(byte)47;
            p172.sue_RACING_MODE = (byte)(byte)169;
            p172.sue_ALTITUDEHOLD_WAYPOINT = (byte)(byte)251;
            p172.sue_ALTITUDEHOLD_STABILIZED = (byte)(byte)162;
            p172.sue_ROLL_STABILIZATION_RUDDER = (byte)(byte)36;
            p172.sue_AILERON_NAVIGATION = (byte)(byte)157;
            p172.sue_RUDDER_NAVIGATION = (byte)(byte)196;
            p172.sue_PITCH_STABILIZATION = (byte)(byte)140;
            p172.sue_YAW_STABILIZATION_AILERON = (byte)(byte)213;
            CommunicationChannel.instance.send(p172);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSERIAL_UDB_EXTRA_F5Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sue_ROLLKD == (float)4.6722184E37F);
                Debug.Assert(pack.sue_ROLLKP == (float)1.2946877E38F);
                Debug.Assert(pack.sue_YAWKP_AILERON == (float)1.2448048E38F);
                Debug.Assert(pack.sue_YAWKD_AILERON == (float)2.1003407E38F);
            };
            GroundControl.SERIAL_UDB_EXTRA_F5 p173 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F5();
            PH.setPack(p173);
            p173.sue_YAWKP_AILERON = (float)1.2448048E38F;
            p173.sue_YAWKD_AILERON = (float)2.1003407E38F;
            p173.sue_ROLLKD = (float)4.6722184E37F;
            p173.sue_ROLLKP = (float)1.2946877E38F;
            CommunicationChannel.instance.send(p173);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSERIAL_UDB_EXTRA_F6Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sue_RUDDER_ELEV_MIX == (float)1.9275467E38F);
                Debug.Assert(pack.sue_ROLL_ELEV_MIX == (float) -3.5098598E37F);
                Debug.Assert(pack.sue_PITCHKD == (float) -2.5789386E38F);
                Debug.Assert(pack.sue_PITCHGAIN == (float) -4.8084583E37F);
                Debug.Assert(pack.sue_ELEVATOR_BOOST == (float) -1.7751506E38F);
            };
            GroundControl.SERIAL_UDB_EXTRA_F6 p174 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F6();
            PH.setPack(p174);
            p174.sue_ELEVATOR_BOOST = (float) -1.7751506E38F;
            p174.sue_RUDDER_ELEV_MIX = (float)1.9275467E38F;
            p174.sue_PITCHKD = (float) -2.5789386E38F;
            p174.sue_PITCHGAIN = (float) -4.8084583E37F;
            p174.sue_ROLL_ELEV_MIX = (float) -3.5098598E37F;
            CommunicationChannel.instance.send(p174);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSERIAL_UDB_EXTRA_F7Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sue_RUDDER_BOOST == (float) -1.6524139E38F);
                Debug.Assert(pack.sue_ROLLKD_RUDDER == (float)1.2525741E38F);
                Debug.Assert(pack.sue_YAWKP_RUDDER == (float)1.2197721E38F);
                Debug.Assert(pack.sue_RTL_PITCH_DOWN == (float)1.6322083E38F);
                Debug.Assert(pack.sue_ROLLKP_RUDDER == (float)2.894389E38F);
                Debug.Assert(pack.sue_YAWKD_RUDDER == (float) -2.6487107E38F);
            };
            GroundControl.SERIAL_UDB_EXTRA_F7 p175 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F7();
            PH.setPack(p175);
            p175.sue_RUDDER_BOOST = (float) -1.6524139E38F;
            p175.sue_YAWKP_RUDDER = (float)1.2197721E38F;
            p175.sue_RTL_PITCH_DOWN = (float)1.6322083E38F;
            p175.sue_ROLLKP_RUDDER = (float)2.894389E38F;
            p175.sue_YAWKD_RUDDER = (float) -2.6487107E38F;
            p175.sue_ROLLKD_RUDDER = (float)1.2525741E38F;
            CommunicationChannel.instance.send(p175);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSERIAL_UDB_EXTRA_F8Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sue_HEIGHT_TARGET_MAX == (float) -1.1503066E38F);
                Debug.Assert(pack.sue_ALT_HOLD_PITCH_HIGH == (float)3.1616603E38F);
                Debug.Assert(pack.sue_ALT_HOLD_PITCH_MAX == (float)3.3579922E38F);
                Debug.Assert(pack.sue_ALT_HOLD_THROTTLE_MAX == (float) -1.0075665E37F);
                Debug.Assert(pack.sue_HEIGHT_TARGET_MIN == (float) -2.1702693E38F);
                Debug.Assert(pack.sue_ALT_HOLD_THROTTLE_MIN == (float)1.1949536E38F);
                Debug.Assert(pack.sue_ALT_HOLD_PITCH_MIN == (float)1.949013E38F);
            };
            GroundControl.SERIAL_UDB_EXTRA_F8 p176 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F8();
            PH.setPack(p176);
            p176.sue_HEIGHT_TARGET_MAX = (float) -1.1503066E38F;
            p176.sue_HEIGHT_TARGET_MIN = (float) -2.1702693E38F;
            p176.sue_ALT_HOLD_PITCH_MIN = (float)1.949013E38F;
            p176.sue_ALT_HOLD_THROTTLE_MAX = (float) -1.0075665E37F;
            p176.sue_ALT_HOLD_THROTTLE_MIN = (float)1.1949536E38F;
            p176.sue_ALT_HOLD_PITCH_MAX = (float)3.3579922E38F;
            p176.sue_ALT_HOLD_PITCH_HIGH = (float)3.1616603E38F;
            CommunicationChannel.instance.send(p176);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSERIAL_UDB_EXTRA_F13Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sue_lon_origin == (int)10462237);
                Debug.Assert(pack.sue_week_no == (short)(short) -22996);
                Debug.Assert(pack.sue_alt_origin == (int)1112845082);
                Debug.Assert(pack.sue_lat_origin == (int) -1983435863);
            };
            GroundControl.SERIAL_UDB_EXTRA_F13 p177 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F13();
            PH.setPack(p177);
            p177.sue_week_no = (short)(short) -22996;
            p177.sue_lon_origin = (int)10462237;
            p177.sue_lat_origin = (int) -1983435863;
            p177.sue_alt_origin = (int)1112845082;
            CommunicationChannel.instance.send(p177);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSERIAL_UDB_EXTRA_F14Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sue_WIND_ESTIMATION == (byte)(byte)211);
                Debug.Assert(pack.sue_DR == (byte)(byte)215);
                Debug.Assert(pack.sue_TRAP_SOURCE == (uint)161707810U);
                Debug.Assert(pack.sue_FLIGHT_PLAN_TYPE == (byte)(byte)151);
                Debug.Assert(pack.sue_TRAP_FLAGS == (short)(short)25757);
                Debug.Assert(pack.sue_GPS_TYPE == (byte)(byte)58);
                Debug.Assert(pack.sue_osc_fail_count == (short)(short) -14336);
                Debug.Assert(pack.sue_AIRFRAME == (byte)(byte)90);
                Debug.Assert(pack.sue_BOARD_TYPE == (byte)(byte)90);
                Debug.Assert(pack.sue_CLOCK_CONFIG == (byte)(byte)117);
                Debug.Assert(pack.sue_RCON == (short)(short) -29295);
            };
            GroundControl.SERIAL_UDB_EXTRA_F14 p178 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F14();
            PH.setPack(p178);
            p178.sue_WIND_ESTIMATION = (byte)(byte)211;
            p178.sue_RCON = (short)(short) -29295;
            p178.sue_DR = (byte)(byte)215;
            p178.sue_osc_fail_count = (short)(short) -14336;
            p178.sue_TRAP_FLAGS = (short)(short)25757;
            p178.sue_BOARD_TYPE = (byte)(byte)90;
            p178.sue_AIRFRAME = (byte)(byte)90;
            p178.sue_GPS_TYPE = (byte)(byte)58;
            p178.sue_CLOCK_CONFIG = (byte)(byte)117;
            p178.sue_TRAP_SOURCE = (uint)161707810U;
            p178.sue_FLIGHT_PLAN_TYPE = (byte)(byte)151;
            CommunicationChannel.instance.send(p178);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSERIAL_UDB_EXTRA_F15Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sue_ID_VEHICLE_MODEL_NAME.SequenceEqual(new byte[] {(byte)188, (byte)155, (byte)156, (byte)140, (byte)209, (byte)89, (byte)142, (byte)104, (byte)219, (byte)58, (byte)32, (byte)115, (byte)101, (byte)11, (byte)96, (byte)26, (byte)230, (byte)85, (byte)46, (byte)206, (byte)66, (byte)29, (byte)213, (byte)200, (byte)75, (byte)48, (byte)77, (byte)230, (byte)4, (byte)244, (byte)57, (byte)174, (byte)235, (byte)253, (byte)173, (byte)82, (byte)121, (byte)145, (byte)252, (byte)224}));
                Debug.Assert(pack.sue_ID_VEHICLE_REGISTRATION.SequenceEqual(new byte[] {(byte)255, (byte)230, (byte)164, (byte)113, (byte)182, (byte)242, (byte)80, (byte)128, (byte)255, (byte)147, (byte)158, (byte)69, (byte)16, (byte)250, (byte)94, (byte)130, (byte)96, (byte)59, (byte)4, (byte)88}));
            };
            GroundControl.SERIAL_UDB_EXTRA_F15 p179 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F15();
            PH.setPack(p179);
            p179.sue_ID_VEHICLE_MODEL_NAME_SET(new byte[] {(byte)188, (byte)155, (byte)156, (byte)140, (byte)209, (byte)89, (byte)142, (byte)104, (byte)219, (byte)58, (byte)32, (byte)115, (byte)101, (byte)11, (byte)96, (byte)26, (byte)230, (byte)85, (byte)46, (byte)206, (byte)66, (byte)29, (byte)213, (byte)200, (byte)75, (byte)48, (byte)77, (byte)230, (byte)4, (byte)244, (byte)57, (byte)174, (byte)235, (byte)253, (byte)173, (byte)82, (byte)121, (byte)145, (byte)252, (byte)224}, 0) ;
            p179.sue_ID_VEHICLE_REGISTRATION_SET(new byte[] {(byte)255, (byte)230, (byte)164, (byte)113, (byte)182, (byte)242, (byte)80, (byte)128, (byte)255, (byte)147, (byte)158, (byte)69, (byte)16, (byte)250, (byte)94, (byte)130, (byte)96, (byte)59, (byte)4, (byte)88}, 0) ;
            CommunicationChannel.instance.send(p179);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSERIAL_UDB_EXTRA_F16Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sue_ID_DIY_DRONES_URL.SequenceEqual(new byte[] {(byte)101, (byte)222, (byte)253, (byte)199, (byte)58, (byte)101, (byte)208, (byte)67, (byte)23, (byte)17, (byte)119, (byte)55, (byte)231, (byte)95, (byte)10, (byte)107, (byte)186, (byte)132, (byte)109, (byte)248, (byte)233, (byte)255, (byte)218, (byte)55, (byte)117, (byte)250, (byte)48, (byte)194, (byte)235, (byte)118, (byte)212, (byte)128, (byte)165, (byte)92, (byte)108, (byte)122, (byte)148, (byte)64, (byte)102, (byte)249, (byte)15, (byte)124, (byte)206, (byte)110, (byte)28, (byte)127, (byte)238, (byte)207, (byte)234, (byte)32, (byte)22, (byte)8, (byte)21, (byte)65, (byte)14, (byte)79, (byte)34, (byte)8, (byte)73, (byte)102, (byte)213, (byte)179, (byte)249, (byte)2, (byte)135, (byte)174, (byte)1, (byte)98, (byte)224, (byte)0}));
                Debug.Assert(pack.sue_ID_LEAD_PILOT.SequenceEqual(new byte[] {(byte)119, (byte)18, (byte)149, (byte)157, (byte)56, (byte)209, (byte)160, (byte)5, (byte)2, (byte)110, (byte)50, (byte)31, (byte)219, (byte)180, (byte)84, (byte)146, (byte)249, (byte)155, (byte)148, (byte)216, (byte)144, (byte)47, (byte)7, (byte)68, (byte)82, (byte)87, (byte)88, (byte)14, (byte)143, (byte)123, (byte)144, (byte)43, (byte)252, (byte)211, (byte)207, (byte)163, (byte)176, (byte)219, (byte)252, (byte)145}));
            };
            GroundControl.SERIAL_UDB_EXTRA_F16 p180 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F16();
            PH.setPack(p180);
            p180.sue_ID_DIY_DRONES_URL_SET(new byte[] {(byte)101, (byte)222, (byte)253, (byte)199, (byte)58, (byte)101, (byte)208, (byte)67, (byte)23, (byte)17, (byte)119, (byte)55, (byte)231, (byte)95, (byte)10, (byte)107, (byte)186, (byte)132, (byte)109, (byte)248, (byte)233, (byte)255, (byte)218, (byte)55, (byte)117, (byte)250, (byte)48, (byte)194, (byte)235, (byte)118, (byte)212, (byte)128, (byte)165, (byte)92, (byte)108, (byte)122, (byte)148, (byte)64, (byte)102, (byte)249, (byte)15, (byte)124, (byte)206, (byte)110, (byte)28, (byte)127, (byte)238, (byte)207, (byte)234, (byte)32, (byte)22, (byte)8, (byte)21, (byte)65, (byte)14, (byte)79, (byte)34, (byte)8, (byte)73, (byte)102, (byte)213, (byte)179, (byte)249, (byte)2, (byte)135, (byte)174, (byte)1, (byte)98, (byte)224, (byte)0}, 0) ;
            p180.sue_ID_LEAD_PILOT_SET(new byte[] {(byte)119, (byte)18, (byte)149, (byte)157, (byte)56, (byte)209, (byte)160, (byte)5, (byte)2, (byte)110, (byte)50, (byte)31, (byte)219, (byte)180, (byte)84, (byte)146, (byte)249, (byte)155, (byte)148, (byte)216, (byte)144, (byte)47, (byte)7, (byte)68, (byte)82, (byte)87, (byte)88, (byte)14, (byte)143, (byte)123, (byte)144, (byte)43, (byte)252, (byte)211, (byte)207, (byte)163, (byte)176, (byte)219, (byte)252, (byte)145}, 0) ;
            CommunicationChannel.instance.send(p180);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnALTITUDESReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.alt_gps == (int) -1484383580);
                Debug.Assert(pack.time_boot_ms == (uint)2998788292U);
                Debug.Assert(pack.alt_extra == (int)912923204);
                Debug.Assert(pack.alt_range_finder == (int)1308210407);
                Debug.Assert(pack.alt_barometric == (int) -176217713);
                Debug.Assert(pack.alt_optical_flow == (int) -1426787762);
                Debug.Assert(pack.alt_imu == (int) -2104818860);
            };
            GroundControl.ALTITUDES p181 = CommunicationChannel.new_ALTITUDES();
            PH.setPack(p181);
            p181.alt_extra = (int)912923204;
            p181.alt_optical_flow = (int) -1426787762;
            p181.alt_imu = (int) -2104818860;
            p181.alt_range_finder = (int)1308210407;
            p181.alt_barometric = (int) -176217713;
            p181.time_boot_ms = (uint)2998788292U;
            p181.alt_gps = (int) -1484383580;
            CommunicationChannel.instance.send(p181);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnAIRSPEEDSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)2624488044U);
                Debug.Assert(pack.airspeed_hot_wire == (short)(short)25289);
                Debug.Assert(pack.airspeed_imu == (short)(short)11373);
                Debug.Assert(pack.airspeed_pitot == (short)(short) -12245);
                Debug.Assert(pack.aoa == (short)(short)4639);
                Debug.Assert(pack.airspeed_ultrasonic == (short)(short)16817);
                Debug.Assert(pack.aoy == (short)(short)12275);
            };
            GroundControl.AIRSPEEDS p182 = CommunicationChannel.new_AIRSPEEDS();
            PH.setPack(p182);
            p182.airspeed_hot_wire = (short)(short)25289;
            p182.airspeed_imu = (short)(short)11373;
            p182.airspeed_ultrasonic = (short)(short)16817;
            p182.aoy = (short)(short)12275;
            p182.aoa = (short)(short)4639;
            p182.airspeed_pitot = (short)(short) -12245;
            p182.time_boot_ms = (uint)2624488044U;
            CommunicationChannel.instance.send(p182);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSERIAL_UDB_EXTRA_F17Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sue_turn_rate_fbw == (float)2.6751482E38F);
                Debug.Assert(pack.sue_feed_forward == (float) -3.4694192E37F);
                Debug.Assert(pack.sue_turn_rate_nav == (float) -2.5374426E38F);
            };
            GroundControl.SERIAL_UDB_EXTRA_F17 p183 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F17();
            PH.setPack(p183);
            p183.sue_feed_forward = (float) -3.4694192E37F;
            p183.sue_turn_rate_fbw = (float)2.6751482E38F;
            p183.sue_turn_rate_nav = (float) -2.5374426E38F;
            CommunicationChannel.instance.send(p183);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSERIAL_UDB_EXTRA_F18Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.elevator_trim_normal == (float)3.346666E38F);
                Debug.Assert(pack.angle_of_attack_normal == (float)1.040225E38F);
                Debug.Assert(pack.angle_of_attack_inverted == (float) -2.4101421E38F);
                Debug.Assert(pack.elevator_trim_inverted == (float)2.3462085E38F);
                Debug.Assert(pack.reference_speed == (float)2.5099425E38F);
            };
            GroundControl.SERIAL_UDB_EXTRA_F18 p184 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F18();
            PH.setPack(p184);
            p184.reference_speed = (float)2.5099425E38F;
            p184.elevator_trim_inverted = (float)2.3462085E38F;
            p184.elevator_trim_normal = (float)3.346666E38F;
            p184.angle_of_attack_normal = (float)1.040225E38F;
            p184.angle_of_attack_inverted = (float) -2.4101421E38F;
            CommunicationChannel.instance.send(p184);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSERIAL_UDB_EXTRA_F19Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sue_throttle_output_channel == (byte)(byte)181);
                Debug.Assert(pack.sue_throttle_reversed == (byte)(byte)81);
                Debug.Assert(pack.sue_elevator_output_channel == (byte)(byte)113);
                Debug.Assert(pack.sue_aileron_reversed == (byte)(byte)122);
                Debug.Assert(pack.sue_aileron_output_channel == (byte)(byte)239);
                Debug.Assert(pack.sue_elevator_reversed == (byte)(byte)160);
                Debug.Assert(pack.sue_rudder_output_channel == (byte)(byte)154);
                Debug.Assert(pack.sue_rudder_reversed == (byte)(byte)154);
            };
            GroundControl.SERIAL_UDB_EXTRA_F19 p185 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F19();
            PH.setPack(p185);
            p185.sue_elevator_output_channel = (byte)(byte)113;
            p185.sue_throttle_output_channel = (byte)(byte)181;
            p185.sue_aileron_reversed = (byte)(byte)122;
            p185.sue_elevator_reversed = (byte)(byte)160;
            p185.sue_rudder_output_channel = (byte)(byte)154;
            p185.sue_throttle_reversed = (byte)(byte)81;
            p185.sue_aileron_output_channel = (byte)(byte)239;
            p185.sue_rudder_reversed = (byte)(byte)154;
            CommunicationChannel.instance.send(p185);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSERIAL_UDB_EXTRA_F20Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sue_trim_value_input_8 == (short)(short) -20825);
                Debug.Assert(pack.sue_trim_value_input_12 == (short)(short)18376);
                Debug.Assert(pack.sue_trim_value_input_5 == (short)(short) -4655);
                Debug.Assert(pack.sue_trim_value_input_10 == (short)(short)16753);
                Debug.Assert(pack.sue_trim_value_input_2 == (short)(short)22585);
                Debug.Assert(pack.sue_trim_value_input_11 == (short)(short) -18350);
                Debug.Assert(pack.sue_trim_value_input_6 == (short)(short)12863);
                Debug.Assert(pack.sue_trim_value_input_3 == (short)(short)7175);
                Debug.Assert(pack.sue_trim_value_input_1 == (short)(short) -6650);
                Debug.Assert(pack.sue_trim_value_input_4 == (short)(short) -7265);
                Debug.Assert(pack.sue_trim_value_input_9 == (short)(short) -30524);
                Debug.Assert(pack.sue_number_of_inputs == (byte)(byte)59);
                Debug.Assert(pack.sue_trim_value_input_7 == (short)(short)21581);
            };
            GroundControl.SERIAL_UDB_EXTRA_F20 p186 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F20();
            PH.setPack(p186);
            p186.sue_trim_value_input_6 = (short)(short)12863;
            p186.sue_trim_value_input_10 = (short)(short)16753;
            p186.sue_trim_value_input_5 = (short)(short) -4655;
            p186.sue_trim_value_input_3 = (short)(short)7175;
            p186.sue_trim_value_input_7 = (short)(short)21581;
            p186.sue_trim_value_input_9 = (short)(short) -30524;
            p186.sue_trim_value_input_8 = (short)(short) -20825;
            p186.sue_trim_value_input_1 = (short)(short) -6650;
            p186.sue_trim_value_input_2 = (short)(short)22585;
            p186.sue_number_of_inputs = (byte)(byte)59;
            p186.sue_trim_value_input_11 = (short)(short) -18350;
            p186.sue_trim_value_input_4 = (short)(short) -7265;
            p186.sue_trim_value_input_12 = (short)(short)18376;
            CommunicationChannel.instance.send(p186);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSERIAL_UDB_EXTRA_F21Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sue_accel_z_offset == (short)(short)1566);
                Debug.Assert(pack.sue_accel_x_offset == (short)(short)5082);
                Debug.Assert(pack.sue_gyro_z_offset == (short)(short)27865);
                Debug.Assert(pack.sue_gyro_x_offset == (short)(short)2087);
                Debug.Assert(pack.sue_gyro_y_offset == (short)(short)13401);
                Debug.Assert(pack.sue_accel_y_offset == (short)(short)12499);
            };
            GroundControl.SERIAL_UDB_EXTRA_F21 p187 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F21();
            PH.setPack(p187);
            p187.sue_accel_y_offset = (short)(short)12499;
            p187.sue_gyro_x_offset = (short)(short)2087;
            p187.sue_gyro_y_offset = (short)(short)13401;
            p187.sue_accel_x_offset = (short)(short)5082;
            p187.sue_accel_z_offset = (short)(short)1566;
            p187.sue_gyro_z_offset = (short)(short)27865;
            CommunicationChannel.instance.send(p187);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSERIAL_UDB_EXTRA_F22Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sue_gyro_z_at_calibration == (short)(short) -5664);
                Debug.Assert(pack.sue_accel_y_at_calibration == (short)(short) -14415);
                Debug.Assert(pack.sue_accel_z_at_calibration == (short)(short)858);
                Debug.Assert(pack.sue_gyro_y_at_calibration == (short)(short) -14603);
                Debug.Assert(pack.sue_accel_x_at_calibration == (short)(short) -25999);
                Debug.Assert(pack.sue_gyro_x_at_calibration == (short)(short) -392);
            };
            GroundControl.SERIAL_UDB_EXTRA_F22 p188 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F22();
            PH.setPack(p188);
            p188.sue_accel_x_at_calibration = (short)(short) -25999;
            p188.sue_gyro_x_at_calibration = (short)(short) -392;
            p188.sue_gyro_z_at_calibration = (short)(short) -5664;
            p188.sue_accel_y_at_calibration = (short)(short) -14415;
            p188.sue_accel_z_at_calibration = (short)(short)858;
            p188.sue_gyro_y_at_calibration = (short)(short) -14603;
            CommunicationChannel.instance.send(p188);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            CommunicationChannel.instance.OnESTIMATOR_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.flags == (ESTIMATOR_STATUS_FLAGS.ESTIMATOR_ATTITUDE |
                                            ESTIMATOR_STATUS_FLAGS.ESTIMATOR_PRED_POS_HORIZ_ABS |
                                            ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_HORIZ));
                Debug.Assert(pack.hagl_ratio == (float)1.7726319E38F);
                Debug.Assert(pack.vel_ratio == (float) -7.507757E37F);
                Debug.Assert(pack.pos_horiz_ratio == (float) -1.2557532E38F);
                Debug.Assert(pack.pos_vert_accuracy == (float) -1.8973813E38F);
                Debug.Assert(pack.pos_horiz_accuracy == (float)3.159564E38F);
                Debug.Assert(pack.mag_ratio == (float)7.0429313E37F);
                Debug.Assert(pack.pos_vert_ratio == (float) -2.9400391E38F);
                Debug.Assert(pack.time_usec == (ulong)6618765841559576424L);
                Debug.Assert(pack.tas_ratio == (float)1.1785072E38F);
            };
            GroundControl.ESTIMATOR_STATUS p230 = CommunicationChannel.new_ESTIMATOR_STATUS();
            PH.setPack(p230);
            p230.pos_vert_accuracy = (float) -1.8973813E38F;
            p230.pos_vert_ratio = (float) -2.9400391E38F;
            p230.mag_ratio = (float)7.0429313E37F;
            p230.flags = (ESTIMATOR_STATUS_FLAGS.ESTIMATOR_ATTITUDE |
                          ESTIMATOR_STATUS_FLAGS.ESTIMATOR_PRED_POS_HORIZ_ABS |
                          ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_HORIZ);
            p230.vel_ratio = (float) -7.507757E37F;
            p230.tas_ratio = (float)1.1785072E38F;
            p230.time_usec = (ulong)6618765841559576424L;
            p230.pos_horiz_accuracy = (float)3.159564E38F;
            p230.pos_horiz_ratio = (float) -1.2557532E38F;
            p230.hagl_ratio = (float)1.7726319E38F;
            CommunicationChannel.instance.send(p230);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnWIND_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.var_horiz == (float)1.657212E38F);
                Debug.Assert(pack.horiz_accuracy == (float) -2.0875064E38F);
                Debug.Assert(pack.var_vert == (float) -2.992813E38F);
                Debug.Assert(pack.wind_y == (float) -2.5268773E38F);
                Debug.Assert(pack.wind_x == (float) -1.7217352E38F);
                Debug.Assert(pack.wind_alt == (float) -2.7139454E38F);
                Debug.Assert(pack.time_usec == (ulong)1679434199837725641L);
                Debug.Assert(pack.wind_z == (float) -4.9761675E37F);
                Debug.Assert(pack.vert_accuracy == (float) -3.3413984E37F);
            };
            GroundControl.WIND_COV p231 = CommunicationChannel.new_WIND_COV();
            PH.setPack(p231);
            p231.wind_x = (float) -1.7217352E38F;
            p231.wind_y = (float) -2.5268773E38F;
            p231.wind_alt = (float) -2.7139454E38F;
            p231.vert_accuracy = (float) -3.3413984E37F;
            p231.wind_z = (float) -4.9761675E37F;
            p231.horiz_accuracy = (float) -2.0875064E38F;
            p231.time_usec = (ulong)1679434199837725641L;
            p231.var_horiz = (float)1.657212E38F;
            p231.var_vert = (float) -2.992813E38F;
            CommunicationChannel.instance.send(p231);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_INPUTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.hdop == (float) -1.0336497E38F);
                Debug.Assert(pack.gps_id == (byte)(byte)235);
                Debug.Assert(pack.ignore_flags == (GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VEL_VERT |
                                                   GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY));
                Debug.Assert(pack.alt == (float)9.131016E37F);
                Debug.Assert(pack.fix_type == (byte)(byte)246);
                Debug.Assert(pack.vdop == (float)2.1775889E38F);
                Debug.Assert(pack.lat == (int)537837411);
                Debug.Assert(pack.lon == (int)1505188921);
                Debug.Assert(pack.vert_accuracy == (float)2.2796574E37F);
                Debug.Assert(pack.vd == (float)5.391593E37F);
                Debug.Assert(pack.time_week == (ushort)(ushort)23585);
                Debug.Assert(pack.time_week_ms == (uint)1887634349U);
                Debug.Assert(pack.speed_accuracy == (float)1.1812621E38F);
                Debug.Assert(pack.time_usec == (ulong)9113758805676674498L);
                Debug.Assert(pack.satellites_visible == (byte)(byte)178);
                Debug.Assert(pack.ve == (float) -2.8966015E38F);
                Debug.Assert(pack.horiz_accuracy == (float) -1.559314E38F);
                Debug.Assert(pack.vn == (float) -2.6313122E38F);
            };
            GroundControl.GPS_INPUT p232 = CommunicationChannel.new_GPS_INPUT();
            PH.setPack(p232);
            p232.speed_accuracy = (float)1.1812621E38F;
            p232.gps_id = (byte)(byte)235;
            p232.lat = (int)537837411;
            p232.vn = (float) -2.6313122E38F;
            p232.time_week = (ushort)(ushort)23585;
            p232.lon = (int)1505188921;
            p232.vdop = (float)2.1775889E38F;
            p232.fix_type = (byte)(byte)246;
            p232.horiz_accuracy = (float) -1.559314E38F;
            p232.hdop = (float) -1.0336497E38F;
            p232.ve = (float) -2.8966015E38F;
            p232.vd = (float)5.391593E37F;
            p232.time_usec = (ulong)9113758805676674498L;
            p232.alt = (float)9.131016E37F;
            p232.time_week_ms = (uint)1887634349U;
            p232.vert_accuracy = (float)2.2796574E37F;
            p232.satellites_visible = (byte)(byte)178;
            p232.ignore_flags = (GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VEL_VERT |
                                 GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY);
            CommunicationChannel.instance.send(p232);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_RTCM_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.flags == (byte)(byte)196);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)25, (byte)136, (byte)246, (byte)93, (byte)89, (byte)21, (byte)89, (byte)150, (byte)35, (byte)231, (byte)20, (byte)198, (byte)105, (byte)237, (byte)252, (byte)242, (byte)173, (byte)81, (byte)252, (byte)199, (byte)99, (byte)183, (byte)75, (byte)219, (byte)76, (byte)146, (byte)186, (byte)78, (byte)129, (byte)9, (byte)232, (byte)46, (byte)27, (byte)112, (byte)34, (byte)179, (byte)49, (byte)134, (byte)65, (byte)107, (byte)53, (byte)184, (byte)175, (byte)82, (byte)47, (byte)177, (byte)238, (byte)177, (byte)132, (byte)243, (byte)84, (byte)43, (byte)85, (byte)103, (byte)128, (byte)5, (byte)132, (byte)53, (byte)36, (byte)20, (byte)218, (byte)193, (byte)236, (byte)86, (byte)32, (byte)229, (byte)88, (byte)197, (byte)250, (byte)73, (byte)208, (byte)237, (byte)43, (byte)223, (byte)7, (byte)83, (byte)183, (byte)11, (byte)131, (byte)212, (byte)139, (byte)238, (byte)46, (byte)181, (byte)75, (byte)227, (byte)84, (byte)71, (byte)163, (byte)107, (byte)56, (byte)196, (byte)148, (byte)230, (byte)219, (byte)60, (byte)94, (byte)188, (byte)9, (byte)239, (byte)23, (byte)134, (byte)69, (byte)172, (byte)139, (byte)203, (byte)228, (byte)100, (byte)163, (byte)207, (byte)136, (byte)89, (byte)204, (byte)155, (byte)161, (byte)235, (byte)0, (byte)174, (byte)75, (byte)241, (byte)203, (byte)55, (byte)116, (byte)227, (byte)128, (byte)125, (byte)119, (byte)172, (byte)73, (byte)149, (byte)202, (byte)33, (byte)95, (byte)54, (byte)225, (byte)182, (byte)87, (byte)6, (byte)109, (byte)221, (byte)183, (byte)250, (byte)115, (byte)49, (byte)163, (byte)164, (byte)74, (byte)7, (byte)194, (byte)34, (byte)174, (byte)222, (byte)210, (byte)26, (byte)172, (byte)29, (byte)58, (byte)181, (byte)93, (byte)177, (byte)127, (byte)197, (byte)155, (byte)75, (byte)66, (byte)206, (byte)159, (byte)123, (byte)213, (byte)9, (byte)191, (byte)58, (byte)67, (byte)42, (byte)248, (byte)202, (byte)33, (byte)182, (byte)22, (byte)22}));
                Debug.Assert(pack.len == (byte)(byte)184);
            };
            GroundControl.GPS_RTCM_DATA p233 = CommunicationChannel.new_GPS_RTCM_DATA();
            PH.setPack(p233);
            p233.flags = (byte)(byte)196;
            p233.data__SET(new byte[] {(byte)25, (byte)136, (byte)246, (byte)93, (byte)89, (byte)21, (byte)89, (byte)150, (byte)35, (byte)231, (byte)20, (byte)198, (byte)105, (byte)237, (byte)252, (byte)242, (byte)173, (byte)81, (byte)252, (byte)199, (byte)99, (byte)183, (byte)75, (byte)219, (byte)76, (byte)146, (byte)186, (byte)78, (byte)129, (byte)9, (byte)232, (byte)46, (byte)27, (byte)112, (byte)34, (byte)179, (byte)49, (byte)134, (byte)65, (byte)107, (byte)53, (byte)184, (byte)175, (byte)82, (byte)47, (byte)177, (byte)238, (byte)177, (byte)132, (byte)243, (byte)84, (byte)43, (byte)85, (byte)103, (byte)128, (byte)5, (byte)132, (byte)53, (byte)36, (byte)20, (byte)218, (byte)193, (byte)236, (byte)86, (byte)32, (byte)229, (byte)88, (byte)197, (byte)250, (byte)73, (byte)208, (byte)237, (byte)43, (byte)223, (byte)7, (byte)83, (byte)183, (byte)11, (byte)131, (byte)212, (byte)139, (byte)238, (byte)46, (byte)181, (byte)75, (byte)227, (byte)84, (byte)71, (byte)163, (byte)107, (byte)56, (byte)196, (byte)148, (byte)230, (byte)219, (byte)60, (byte)94, (byte)188, (byte)9, (byte)239, (byte)23, (byte)134, (byte)69, (byte)172, (byte)139, (byte)203, (byte)228, (byte)100, (byte)163, (byte)207, (byte)136, (byte)89, (byte)204, (byte)155, (byte)161, (byte)235, (byte)0, (byte)174, (byte)75, (byte)241, (byte)203, (byte)55, (byte)116, (byte)227, (byte)128, (byte)125, (byte)119, (byte)172, (byte)73, (byte)149, (byte)202, (byte)33, (byte)95, (byte)54, (byte)225, (byte)182, (byte)87, (byte)6, (byte)109, (byte)221, (byte)183, (byte)250, (byte)115, (byte)49, (byte)163, (byte)164, (byte)74, (byte)7, (byte)194, (byte)34, (byte)174, (byte)222, (byte)210, (byte)26, (byte)172, (byte)29, (byte)58, (byte)181, (byte)93, (byte)177, (byte)127, (byte)197, (byte)155, (byte)75, (byte)66, (byte)206, (byte)159, (byte)123, (byte)213, (byte)9, (byte)191, (byte)58, (byte)67, (byte)42, (byte)248, (byte)202, (byte)33, (byte)182, (byte)22, (byte)22}, 0) ;
            p233.len = (byte)(byte)184;
            CommunicationChannel.instance.send(p233);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIGH_LATENCYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.gps_nsat == (byte)(byte)183);
                Debug.Assert(pack.latitude == (int)231068340);
                Debug.Assert(pack.roll == (short)(short) -14918);
                Debug.Assert(pack.base_mode == (MAV_MODE_FLAG.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED |
                                                MAV_MODE_FLAG.MAV_MODE_FLAG_TEST_ENABLED));
                Debug.Assert(pack.heading_sp == (short)(short) -17701);
                Debug.Assert(pack.temperature_air == (sbyte)(sbyte) - 122);
                Debug.Assert(pack.gps_fix_type == GPS_FIX_TYPE.GPS_FIX_TYPE_2D_FIX);
                Debug.Assert(pack.battery_remaining == (byte)(byte)69);
                Debug.Assert(pack.temperature == (sbyte)(sbyte) - 89);
                Debug.Assert(pack.heading == (ushort)(ushort)44484);
                Debug.Assert(pack.airspeed_sp == (byte)(byte)226);
                Debug.Assert(pack.pitch == (short)(short) -13582);
                Debug.Assert(pack.wp_distance == (ushort)(ushort)55437);
                Debug.Assert(pack.climb_rate == (sbyte)(sbyte) - 65);
                Debug.Assert(pack.longitude == (int)743128437);
                Debug.Assert(pack.throttle == (sbyte)(sbyte) - 35);
                Debug.Assert(pack.altitude_sp == (short)(short)7700);
                Debug.Assert(pack.airspeed == (byte)(byte)37);
                Debug.Assert(pack.custom_mode == (uint)111335515U);
                Debug.Assert(pack.groundspeed == (byte)(byte)228);
                Debug.Assert(pack.wp_num == (byte)(byte)146);
                Debug.Assert(pack.altitude_amsl == (short)(short) -21685);
                Debug.Assert(pack.failsafe == (byte)(byte)99);
                Debug.Assert(pack.landed_state == MAV_LANDED_STATE.MAV_LANDED_STATE_IN_AIR);
            };
            GroundControl.HIGH_LATENCY p234 = CommunicationChannel.new_HIGH_LATENCY();
            PH.setPack(p234);
            p234.heading_sp = (short)(short) -17701;
            p234.temperature = (sbyte)(sbyte) - 89;
            p234.airspeed_sp = (byte)(byte)226;
            p234.battery_remaining = (byte)(byte)69;
            p234.failsafe = (byte)(byte)99;
            p234.landed_state = MAV_LANDED_STATE.MAV_LANDED_STATE_IN_AIR;
            p234.altitude_sp = (short)(short)7700;
            p234.wp_distance = (ushort)(ushort)55437;
            p234.altitude_amsl = (short)(short) -21685;
            p234.temperature_air = (sbyte)(sbyte) - 122;
            p234.latitude = (int)231068340;
            p234.gps_nsat = (byte)(byte)183;
            p234.airspeed = (byte)(byte)37;
            p234.gps_fix_type = GPS_FIX_TYPE.GPS_FIX_TYPE_2D_FIX;
            p234.wp_num = (byte)(byte)146;
            p234.climb_rate = (sbyte)(sbyte) - 65;
            p234.custom_mode = (uint)111335515U;
            p234.throttle = (sbyte)(sbyte) - 35;
            p234.groundspeed = (byte)(byte)228;
            p234.base_mode = (MAV_MODE_FLAG.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED |
                              MAV_MODE_FLAG.MAV_MODE_FLAG_TEST_ENABLED);
            p234.longitude = (int)743128437;
            p234.heading = (ushort)(ushort)44484;
            p234.pitch = (short)(short) -13582;
            p234.roll = (short)(short) -14918;
            CommunicationChannel.instance.send(p234);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnVIBRATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vibration_y == (float) -2.492725E38F);
                Debug.Assert(pack.vibration_x == (float) -2.7260796E38F);
                Debug.Assert(pack.time_usec == (ulong)6120336543570188554L);
                Debug.Assert(pack.clipping_0 == (uint)3244255858U);
                Debug.Assert(pack.clipping_2 == (uint)58988532U);
                Debug.Assert(pack.vibration_z == (float) -1.6341575E38F);
                Debug.Assert(pack.clipping_1 == (uint)2848942716U);
            };
            GroundControl.VIBRATION p241 = CommunicationChannel.new_VIBRATION();
            PH.setPack(p241);
            p241.vibration_x = (float) -2.7260796E38F;
            p241.clipping_2 = (uint)58988532U;
            p241.clipping_1 = (uint)2848942716U;
            p241.vibration_y = (float) -2.492725E38F;
            p241.clipping_0 = (uint)3244255858U;
            p241.time_usec = (ulong)6120336543570188554L;
            p241.vibration_z = (float) -1.6341575E38F;
            CommunicationChannel.instance.send(p241);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHOME_POSITIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.x == (float) -1.6287429E38F);
                Debug.Assert(pack.approach_x == (float) -3.390409E38F);
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)4374508797307235584L);
                Debug.Assert(pack.y == (float) -1.1176476E38F);
                Debug.Assert(pack.longitude == (int)2048244774);
                Debug.Assert(pack.approach_y == (float)1.4912303E38F);
                Debug.Assert(pack.approach_z == (float) -2.9880508E38F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {1.7509845E38F, -2.512622E38F, -1.3237863E38F, 7.755635E37F}));
                Debug.Assert(pack.z == (float) -8.677045E37F);
                Debug.Assert(pack.latitude == (int)1036499450);
                Debug.Assert(pack.altitude == (int)557929370);
            };
            GroundControl.HOME_POSITION p242 = CommunicationChannel.new_HOME_POSITION();
            PH.setPack(p242);
            p242.longitude = (int)2048244774;
            p242.latitude = (int)1036499450;
            p242.approach_z = (float) -2.9880508E38F;
            p242.y = (float) -1.1176476E38F;
            p242.x = (float) -1.6287429E38F;
            p242.z = (float) -8.677045E37F;
            p242.q_SET(new float[] {1.7509845E38F, -2.512622E38F, -1.3237863E38F, 7.755635E37F}, 0) ;
            p242.approach_x = (float) -3.390409E38F;
            p242.altitude = (int)557929370;
            p242.time_usec_SET((ulong)4374508797307235584L, PH) ;
            p242.approach_y = (float)1.4912303E38F;
            CommunicationChannel.instance.send(p242);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_HOME_POSITIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.approach_y == (float) -1.8452813E38F);
                Debug.Assert(pack.latitude == (int)1830750929);
                Debug.Assert(pack.target_system == (byte)(byte)37);
                Debug.Assert(pack.longitude == (int) -804862884);
                Debug.Assert(pack.y == (float)2.7681907E38F);
                Debug.Assert(pack.approach_x == (float)7.975605E36F);
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)604748023639638556L);
                Debug.Assert(pack.altitude == (int) -1126185384);
                Debug.Assert(pack.x == (float)5.554492E37F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-2.0376406E38F, 1.2722178E38F, 2.7827156E38F, -3.120998E38F}));
                Debug.Assert(pack.z == (float) -1.4456144E37F);
                Debug.Assert(pack.approach_z == (float)3.3154907E38F);
            };
            GroundControl.SET_HOME_POSITION p243 = CommunicationChannel.new_SET_HOME_POSITION();
            PH.setPack(p243);
            p243.latitude = (int)1830750929;
            p243.approach_y = (float) -1.8452813E38F;
            p243.y = (float)2.7681907E38F;
            p243.approach_x = (float)7.975605E36F;
            p243.altitude = (int) -1126185384;
            p243.approach_z = (float)3.3154907E38F;
            p243.longitude = (int) -804862884;
            p243.z = (float) -1.4456144E37F;
            p243.x = (float)5.554492E37F;
            p243.target_system = (byte)(byte)37;
            p243.time_usec_SET((ulong)604748023639638556L, PH) ;
            p243.q_SET(new float[] {-2.0376406E38F, 1.2722178E38F, 2.7827156E38F, -3.120998E38F}, 0) ;
            CommunicationChannel.instance.send(p243);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            ADV_TEST_CH.OnMESSAGE_INTERVALReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.message_id == (ushort)(ushort)60065);
                Debug.Assert(pack.interval_us == (int) -460474855);
            };
            GroundControl.MESSAGE_INTERVAL p244 = CommunicationChannel.new_MESSAGE_INTERVAL();
            PH.setPack(p244);
            p244.interval_us = (int) -460474855;
            p244.message_id = (ushort)(ushort)60065;
            CommunicationChannel.instance.send(p244);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnEXTENDED_SYS_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vtol_state == MAV_VTOL_STATE.MAV_VTOL_STATE_MC);
                Debug.Assert(pack.landed_state == MAV_LANDED_STATE.MAV_LANDED_STATE_TAKEOFF);
            };
            GroundControl.EXTENDED_SYS_STATE p245 = CommunicationChannel.new_EXTENDED_SYS_STATE();
            PH.setPack(p245);
            p245.landed_state = MAV_LANDED_STATE.MAV_LANDED_STATE_TAKEOFF;
            p245.vtol_state = MAV_VTOL_STATE.MAV_VTOL_STATE_MC;
            CommunicationChannel.instance.send(p245);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnADSB_VEHICLEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.hor_velocity == (ushort)(ushort)7391);
                Debug.Assert(pack.heading == (ushort)(ushort)1698);
                Debug.Assert(pack.callsign_LEN(ph) == 5);
                Debug.Assert(pack.callsign_TRY(ph).Equals("Fpfrk"));
                Debug.Assert(pack.lat == (int)81668522);
                Debug.Assert(pack.emitter_type == ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_HIGH_VORTEX_LARGE);
                Debug.Assert(pack.flags == (ADSB_FLAGS.ADSB_FLAGS_VALID_VELOCITY));
                Debug.Assert(pack.tslc == (byte)(byte)244);
                Debug.Assert(pack.ICAO_address == (uint)3359110071U);
                Debug.Assert(pack.lon == (int)677996049);
                Debug.Assert(pack.altitude_type == ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_PRESSURE_QNH);
                Debug.Assert(pack.ver_velocity == (short)(short)181);
                Debug.Assert(pack.squawk == (ushort)(ushort)40957);
                Debug.Assert(pack.altitude == (int) -1862505941);
            };
            GroundControl.ADSB_VEHICLE p246 = CommunicationChannel.new_ADSB_VEHICLE();
            PH.setPack(p246);
            p246.altitude = (int) -1862505941;
            p246.lon = (int)677996049;
            p246.lat = (int)81668522;
            p246.altitude_type = ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_PRESSURE_QNH;
            p246.tslc = (byte)(byte)244;
            p246.flags = (ADSB_FLAGS.ADSB_FLAGS_VALID_VELOCITY);
            p246.callsign_SET("Fpfrk", PH) ;
            p246.ICAO_address = (uint)3359110071U;
            p246.ver_velocity = (short)(short)181;
            p246.heading = (ushort)(ushort)1698;
            p246.hor_velocity = (ushort)(ushort)7391;
            p246.squawk = (ushort)(ushort)40957;
            p246.emitter_type = ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_HIGH_VORTEX_LARGE;
            CommunicationChannel.instance.send(p246);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnCOLLISIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.action == MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_MOVE_HORIZONTALLY);
                Debug.Assert(pack.horizontal_minimum_delta == (float) -1.8380074E38F);
                Debug.Assert(pack.threat_level == MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_HIGH);
                Debug.Assert(pack.altitude_minimum_delta == (float) -9.263663E37F);
                Debug.Assert(pack.id == (uint)397222659U);
                Debug.Assert(pack.time_to_minimum_delta == (float)5.5892966E37F);
                Debug.Assert(pack.src_ == MAV_COLLISION_SRC.MAV_COLLISION_SRC_ADSB);
            };
            GroundControl.COLLISION p247 = CommunicationChannel.new_COLLISION();
            PH.setPack(p247);
            p247.id = (uint)397222659U;
            p247.time_to_minimum_delta = (float)5.5892966E37F;
            p247.threat_level = MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_HIGH;
            p247.src_ = MAV_COLLISION_SRC.MAV_COLLISION_SRC_ADSB;
            p247.action = MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_MOVE_HORIZONTALLY;
            p247.altitude_minimum_delta = (float) -9.263663E37F;
            p247.horizontal_minimum_delta = (float) -1.8380074E38F;
            CommunicationChannel.instance.send(p247);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnV2_EXTENSIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)171);
                Debug.Assert(pack.payload.SequenceEqual(new byte[] {(byte)151, (byte)171, (byte)161, (byte)229, (byte)191, (byte)136, (byte)221, (byte)38, (byte)174, (byte)92, (byte)75, (byte)164, (byte)155, (byte)67, (byte)48, (byte)168, (byte)196, (byte)188, (byte)199, (byte)56, (byte)70, (byte)157, (byte)62, (byte)92, (byte)47, (byte)131, (byte)24, (byte)157, (byte)207, (byte)172, (byte)208, (byte)19, (byte)208, (byte)51, (byte)165, (byte)68, (byte)143, (byte)202, (byte)247, (byte)219, (byte)64, (byte)229, (byte)36, (byte)128, (byte)16, (byte)37, (byte)215, (byte)173, (byte)185, (byte)246, (byte)151, (byte)122, (byte)104, (byte)190, (byte)225, (byte)94, (byte)120, (byte)3, (byte)39, (byte)127, (byte)231, (byte)47, (byte)183, (byte)197, (byte)103, (byte)55, (byte)165, (byte)80, (byte)184, (byte)225, (byte)42, (byte)213, (byte)40, (byte)202, (byte)28, (byte)76, (byte)153, (byte)89, (byte)135, (byte)136, (byte)23, (byte)78, (byte)213, (byte)18, (byte)130, (byte)128, (byte)92, (byte)142, (byte)154, (byte)28, (byte)111, (byte)83, (byte)225, (byte)116, (byte)7, (byte)187, (byte)67, (byte)47, (byte)99, (byte)255, (byte)208, (byte)66, (byte)37, (byte)232, (byte)174, (byte)7, (byte)165, (byte)133, (byte)194, (byte)196, (byte)246, (byte)170, (byte)241, (byte)56, (byte)238, (byte)232, (byte)223, (byte)111, (byte)107, (byte)7, (byte)79, (byte)75, (byte)95, (byte)147, (byte)42, (byte)122, (byte)203, (byte)252, (byte)55, (byte)45, (byte)142, (byte)228, (byte)177, (byte)72, (byte)139, (byte)58, (byte)131, (byte)6, (byte)201, (byte)31, (byte)26, (byte)100, (byte)229, (byte)33, (byte)171, (byte)204, (byte)217, (byte)70, (byte)125, (byte)93, (byte)248, (byte)148, (byte)76, (byte)211, (byte)97, (byte)185, (byte)110, (byte)105, (byte)251, (byte)109, (byte)84, (byte)32, (byte)66, (byte)64, (byte)74, (byte)179, (byte)234, (byte)155, (byte)235, (byte)119, (byte)85, (byte)194, (byte)101, (byte)53, (byte)33, (byte)213, (byte)34, (byte)198, (byte)15, (byte)248, (byte)49, (byte)109, (byte)109, (byte)196, (byte)75, (byte)21, (byte)3, (byte)126, (byte)112, (byte)101, (byte)218, (byte)139, (byte)149, (byte)78, (byte)255, (byte)65, (byte)17, (byte)198, (byte)120, (byte)33, (byte)73, (byte)165, (byte)211, (byte)163, (byte)170, (byte)225, (byte)120, (byte)167, (byte)174, (byte)180, (byte)158, (byte)22, (byte)50, (byte)104, (byte)36, (byte)185, (byte)148, (byte)184, (byte)1, (byte)196, (byte)35, (byte)234, (byte)51, (byte)131, (byte)92, (byte)171, (byte)129, (byte)174, (byte)164, (byte)56, (byte)104, (byte)165, (byte)202, (byte)37, (byte)209, (byte)17, (byte)100, (byte)83, (byte)43, (byte)140, (byte)57, (byte)253, (byte)37, (byte)241, (byte)239, (byte)133, (byte)234, (byte)171, (byte)5}));
                Debug.Assert(pack.target_component == (byte)(byte)247);
                Debug.Assert(pack.message_type == (ushort)(ushort)6497);
                Debug.Assert(pack.target_network == (byte)(byte)132);
            };
            GroundControl.V2_EXTENSION p248 = CommunicationChannel.new_V2_EXTENSION();
            PH.setPack(p248);
            p248.payload_SET(new byte[] {(byte)151, (byte)171, (byte)161, (byte)229, (byte)191, (byte)136, (byte)221, (byte)38, (byte)174, (byte)92, (byte)75, (byte)164, (byte)155, (byte)67, (byte)48, (byte)168, (byte)196, (byte)188, (byte)199, (byte)56, (byte)70, (byte)157, (byte)62, (byte)92, (byte)47, (byte)131, (byte)24, (byte)157, (byte)207, (byte)172, (byte)208, (byte)19, (byte)208, (byte)51, (byte)165, (byte)68, (byte)143, (byte)202, (byte)247, (byte)219, (byte)64, (byte)229, (byte)36, (byte)128, (byte)16, (byte)37, (byte)215, (byte)173, (byte)185, (byte)246, (byte)151, (byte)122, (byte)104, (byte)190, (byte)225, (byte)94, (byte)120, (byte)3, (byte)39, (byte)127, (byte)231, (byte)47, (byte)183, (byte)197, (byte)103, (byte)55, (byte)165, (byte)80, (byte)184, (byte)225, (byte)42, (byte)213, (byte)40, (byte)202, (byte)28, (byte)76, (byte)153, (byte)89, (byte)135, (byte)136, (byte)23, (byte)78, (byte)213, (byte)18, (byte)130, (byte)128, (byte)92, (byte)142, (byte)154, (byte)28, (byte)111, (byte)83, (byte)225, (byte)116, (byte)7, (byte)187, (byte)67, (byte)47, (byte)99, (byte)255, (byte)208, (byte)66, (byte)37, (byte)232, (byte)174, (byte)7, (byte)165, (byte)133, (byte)194, (byte)196, (byte)246, (byte)170, (byte)241, (byte)56, (byte)238, (byte)232, (byte)223, (byte)111, (byte)107, (byte)7, (byte)79, (byte)75, (byte)95, (byte)147, (byte)42, (byte)122, (byte)203, (byte)252, (byte)55, (byte)45, (byte)142, (byte)228, (byte)177, (byte)72, (byte)139, (byte)58, (byte)131, (byte)6, (byte)201, (byte)31, (byte)26, (byte)100, (byte)229, (byte)33, (byte)171, (byte)204, (byte)217, (byte)70, (byte)125, (byte)93, (byte)248, (byte)148, (byte)76, (byte)211, (byte)97, (byte)185, (byte)110, (byte)105, (byte)251, (byte)109, (byte)84, (byte)32, (byte)66, (byte)64, (byte)74, (byte)179, (byte)234, (byte)155, (byte)235, (byte)119, (byte)85, (byte)194, (byte)101, (byte)53, (byte)33, (byte)213, (byte)34, (byte)198, (byte)15, (byte)248, (byte)49, (byte)109, (byte)109, (byte)196, (byte)75, (byte)21, (byte)3, (byte)126, (byte)112, (byte)101, (byte)218, (byte)139, (byte)149, (byte)78, (byte)255, (byte)65, (byte)17, (byte)198, (byte)120, (byte)33, (byte)73, (byte)165, (byte)211, (byte)163, (byte)170, (byte)225, (byte)120, (byte)167, (byte)174, (byte)180, (byte)158, (byte)22, (byte)50, (byte)104, (byte)36, (byte)185, (byte)148, (byte)184, (byte)1, (byte)196, (byte)35, (byte)234, (byte)51, (byte)131, (byte)92, (byte)171, (byte)129, (byte)174, (byte)164, (byte)56, (byte)104, (byte)165, (byte)202, (byte)37, (byte)209, (byte)17, (byte)100, (byte)83, (byte)43, (byte)140, (byte)57, (byte)253, (byte)37, (byte)241, (byte)239, (byte)133, (byte)234, (byte)171, (byte)5}, 0) ;
            p248.target_system = (byte)(byte)171;
            p248.target_component = (byte)(byte)247;
            p248.message_type = (ushort)(ushort)6497;
            p248.target_network = (byte)(byte)132;
            CommunicationChannel.instance.send(p248);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnMEMORY_VECTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.type == (byte)(byte)90);
                Debug.Assert(pack.ver == (byte)(byte)87);
                Debug.Assert(pack.value.SequenceEqual(new sbyte[] {(sbyte)30, (sbyte)81, (sbyte)127, (sbyte)89, (sbyte)33, (sbyte)88, (sbyte) - 106, (sbyte)96, (sbyte)72, (sbyte) - 19, (sbyte)50, (sbyte)38, (sbyte) - 5, (sbyte)101, (sbyte)62, (sbyte) - 99, (sbyte)70, (sbyte) - 109, (sbyte) - 28, (sbyte) - 17, (sbyte) - 46, (sbyte)85, (sbyte)17, (sbyte)106, (sbyte) - 119, (sbyte) - 113, (sbyte)93, (sbyte) - 49, (sbyte) - 52, (sbyte)20, (sbyte)63, (sbyte) - 61}));
                Debug.Assert(pack.address == (ushort)(ushort)51563);
            };
            GroundControl.MEMORY_VECT p249 = CommunicationChannel.new_MEMORY_VECT();
            PH.setPack(p249);
            p249.type = (byte)(byte)90;
            p249.value_SET(new sbyte[] {(sbyte)30, (sbyte)81, (sbyte)127, (sbyte)89, (sbyte)33, (sbyte)88, (sbyte) - 106, (sbyte)96, (sbyte)72, (sbyte) - 19, (sbyte)50, (sbyte)38, (sbyte) - 5, (sbyte)101, (sbyte)62, (sbyte) - 99, (sbyte)70, (sbyte) - 109, (sbyte) - 28, (sbyte) - 17, (sbyte) - 46, (sbyte)85, (sbyte)17, (sbyte)106, (sbyte) - 119, (sbyte) - 113, (sbyte)93, (sbyte) - 49, (sbyte) - 52, (sbyte)20, (sbyte)63, (sbyte) - 61}, 0) ;
            p249.ver = (byte)(byte)87;
            p249.address = (ushort)(ushort)51563;
            CommunicationChannel.instance.send(p249);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnDEBUG_VECTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.x == (float) -2.3049374E38F);
                Debug.Assert(pack.z == (float)2.0678949E38F);
                Debug.Assert(pack.name_LEN(ph) == 5);
                Debug.Assert(pack.name_TRY(ph).Equals("jsGxh"));
                Debug.Assert(pack.time_usec == (ulong)2255157103107422563L);
                Debug.Assert(pack.y == (float) -1.2846514E38F);
            };
            GroundControl.DEBUG_VECT p250 = CommunicationChannel.new_DEBUG_VECT();
            PH.setPack(p250);
            p250.x = (float) -2.3049374E38F;
            p250.name_SET("jsGxh", PH) ;
            p250.time_usec = (ulong)2255157103107422563L;
            p250.z = (float)2.0678949E38F;
            p250.y = (float) -1.2846514E38F;
            CommunicationChannel.instance.send(p250);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnNAMED_VALUE_FLOATReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.value == (float)9.027665E37F);
                Debug.Assert(pack.name_LEN(ph) == 3);
                Debug.Assert(pack.name_TRY(ph).Equals("myk"));
                Debug.Assert(pack.time_boot_ms == (uint)3683729087U);
            };
            GroundControl.NAMED_VALUE_FLOAT p251 = CommunicationChannel.new_NAMED_VALUE_FLOAT();
            PH.setPack(p251);
            p251.name_SET("myk", PH) ;
            p251.value = (float)9.027665E37F;
            p251.time_boot_ms = (uint)3683729087U;
            CommunicationChannel.instance.send(p251);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnNAMED_VALUE_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.name_LEN(ph) == 4);
                Debug.Assert(pack.name_TRY(ph).Equals("xmTt"));
                Debug.Assert(pack.time_boot_ms == (uint)792494582U);
                Debug.Assert(pack.value == (int) -209041804);
            };
            GroundControl.NAMED_VALUE_INT p252 = CommunicationChannel.new_NAMED_VALUE_INT();
            PH.setPack(p252);
            p252.time_boot_ms = (uint)792494582U;
            p252.name_SET("xmTt", PH) ;
            p252.value = (int) -209041804;
            CommunicationChannel.instance.send(p252);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSTATUSTEXTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.severity == MAV_SEVERITY.MAV_SEVERITY_WARNING);
                Debug.Assert(pack.text_LEN(ph) == 34);
                Debug.Assert(pack.text_TRY(ph).Equals("wcqiJzyysicudanstpwsongibiwzydtmno"));
            };
            GroundControl.STATUSTEXT p253 = CommunicationChannel.new_STATUSTEXT();
            PH.setPack(p253);
            p253.severity = MAV_SEVERITY.MAV_SEVERITY_WARNING;
            p253.text_SET("wcqiJzyysicudanstpwsongibiwzydtmno", PH) ;
            CommunicationChannel.instance.send(p253);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnDEBUGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)668863916U);
                Debug.Assert(pack.value == (float)3.3036747E38F);
                Debug.Assert(pack.ind == (byte)(byte)37);
            };
            GroundControl.DEBUG p254 = CommunicationChannel.new_DEBUG();
            PH.setPack(p254);
            p254.value = (float)3.3036747E38F;
            p254.time_boot_ms = (uint)668863916U;
            p254.ind = (byte)(byte)37;
            CommunicationChannel.instance.send(p254);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSETUP_SIGNINGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)126);
                Debug.Assert(pack.initial_timestamp == (ulong)5349584593773387485L);
                Debug.Assert(pack.secret_key.SequenceEqual(new byte[] {(byte)4, (byte)31, (byte)73, (byte)199, (byte)251, (byte)6, (byte)97, (byte)210, (byte)154, (byte)138, (byte)183, (byte)229, (byte)182, (byte)6, (byte)132, (byte)32, (byte)172, (byte)87, (byte)117, (byte)106, (byte)130, (byte)27, (byte)78, (byte)161, (byte)221, (byte)52, (byte)167, (byte)47, (byte)205, (byte)71, (byte)95, (byte)12}));
                Debug.Assert(pack.target_component == (byte)(byte)140);
            };
            GroundControl.SETUP_SIGNING p256 = CommunicationChannel.new_SETUP_SIGNING();
            PH.setPack(p256);
            p256.target_component = (byte)(byte)140;
            p256.target_system = (byte)(byte)126;
            p256.initial_timestamp = (ulong)5349584593773387485L;
            p256.secret_key_SET(new byte[] {(byte)4, (byte)31, (byte)73, (byte)199, (byte)251, (byte)6, (byte)97, (byte)210, (byte)154, (byte)138, (byte)183, (byte)229, (byte)182, (byte)6, (byte)132, (byte)32, (byte)172, (byte)87, (byte)117, (byte)106, (byte)130, (byte)27, (byte)78, (byte)161, (byte)221, (byte)52, (byte)167, (byte)47, (byte)205, (byte)71, (byte)95, (byte)12}, 0) ;
            CommunicationChannel.instance.send(p256);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnBUTTON_CHANGEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)3238331139U);
                Debug.Assert(pack.last_change_ms == (uint)3571727335U);
                Debug.Assert(pack.state == (byte)(byte)227);
            };
            GroundControl.BUTTON_CHANGE p257 = CommunicationChannel.new_BUTTON_CHANGE();
            PH.setPack(p257);
            p257.time_boot_ms = (uint)3238331139U;
            p257.last_change_ms = (uint)3571727335U;
            p257.state = (byte)(byte)227;
            CommunicationChannel.instance.send(p257);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnPLAY_TUNEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.tune_LEN(ph) == 13);
                Debug.Assert(pack.tune_TRY(ph).Equals("rolxkcdwrqasd"));
                Debug.Assert(pack.target_system == (byte)(byte)184);
                Debug.Assert(pack.target_component == (byte)(byte)16);
            };
            GroundControl.PLAY_TUNE p258 = CommunicationChannel.new_PLAY_TUNE();
            PH.setPack(p258);
            p258.tune_SET("rolxkcdwrqasd", PH) ;
            p258.target_component = (byte)(byte)16;
            p258.target_system = (byte)(byte)184;
            CommunicationChannel.instance.send(p258);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnCAMERA_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.resolution_h == (ushort)(ushort)22715);
                Debug.Assert(pack.sensor_size_h == (float)2.6465478E38F);
                Debug.Assert(pack.cam_definition_uri_LEN(ph) == 119);
                Debug.Assert(pack.cam_definition_uri_TRY(ph).Equals("aysmccwxzLgywqkiryhqGmdmnioeexuatseehrccbnmqtwscofQswcjChprbefoBxzkpSlvztwzfemvyPnfaekejxgmlksksSqBhnbvbMmdbzLissagufyg"));
                Debug.Assert(pack.flags == (CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE));
                Debug.Assert(pack.firmware_version == (uint)415241558U);
                Debug.Assert(pack.focal_length == (float) -1.1795555E38F);
                Debug.Assert(pack.lens_id == (byte)(byte)192);
                Debug.Assert(pack.time_boot_ms == (uint)2252226174U);
                Debug.Assert(pack.sensor_size_v == (float)1.669508E38F);
                Debug.Assert(pack.cam_definition_version == (ushort)(ushort)21908);
                Debug.Assert(pack.resolution_v == (ushort)(ushort)61339);
                Debug.Assert(pack.model_name.SequenceEqual(new byte[] {(byte)182, (byte)27, (byte)28, (byte)235, (byte)31, (byte)252, (byte)173, (byte)97, (byte)75, (byte)53, (byte)239, (byte)239, (byte)206, (byte)19, (byte)166, (byte)215, (byte)219, (byte)195, (byte)249, (byte)181, (byte)65, (byte)48, (byte)176, (byte)135, (byte)53, (byte)123, (byte)121, (byte)138, (byte)127, (byte)208, (byte)101, (byte)20}));
                Debug.Assert(pack.vendor_name.SequenceEqual(new byte[] {(byte)89, (byte)213, (byte)244, (byte)55, (byte)223, (byte)3, (byte)177, (byte)232, (byte)90, (byte)47, (byte)154, (byte)89, (byte)227, (byte)60, (byte)204, (byte)198, (byte)223, (byte)83, (byte)55, (byte)129, (byte)76, (byte)124, (byte)96, (byte)104, (byte)89, (byte)216, (byte)253, (byte)133, (byte)72, (byte)20, (byte)153, (byte)234}));
            };
            GroundControl.CAMERA_INFORMATION p259 = CommunicationChannel.new_CAMERA_INFORMATION();
            PH.setPack(p259);
            p259.sensor_size_h = (float)2.6465478E38F;
            p259.firmware_version = (uint)415241558U;
            p259.cam_definition_uri_SET("aysmccwxzLgywqkiryhqGmdmnioeexuatseehrccbnmqtwscofQswcjChprbefoBxzkpSlvztwzfemvyPnfaekejxgmlksksSqBhnbvbMmdbzLissagufyg", PH) ;
            p259.sensor_size_v = (float)1.669508E38F;
            p259.focal_length = (float) -1.1795555E38F;
            p259.lens_id = (byte)(byte)192;
            p259.flags = (CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE);
            p259.resolution_h = (ushort)(ushort)22715;
            p259.vendor_name_SET(new byte[] {(byte)89, (byte)213, (byte)244, (byte)55, (byte)223, (byte)3, (byte)177, (byte)232, (byte)90, (byte)47, (byte)154, (byte)89, (byte)227, (byte)60, (byte)204, (byte)198, (byte)223, (byte)83, (byte)55, (byte)129, (byte)76, (byte)124, (byte)96, (byte)104, (byte)89, (byte)216, (byte)253, (byte)133, (byte)72, (byte)20, (byte)153, (byte)234}, 0) ;
            p259.resolution_v = (ushort)(ushort)61339;
            p259.time_boot_ms = (uint)2252226174U;
            p259.model_name_SET(new byte[] {(byte)182, (byte)27, (byte)28, (byte)235, (byte)31, (byte)252, (byte)173, (byte)97, (byte)75, (byte)53, (byte)239, (byte)239, (byte)206, (byte)19, (byte)166, (byte)215, (byte)219, (byte)195, (byte)249, (byte)181, (byte)65, (byte)48, (byte)176, (byte)135, (byte)53, (byte)123, (byte)121, (byte)138, (byte)127, (byte)208, (byte)101, (byte)20}, 0) ;
            p259.cam_definition_version = (ushort)(ushort)21908;
            CommunicationChannel.instance.send(p259);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnCAMERA_SETTINGSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mode_id == CAMERA_MODE.CAMERA_MODE_IMAGE);
                Debug.Assert(pack.time_boot_ms == (uint)1490733092U);
            };
            GroundControl.CAMERA_SETTINGS p260 = CommunicationChannel.new_CAMERA_SETTINGS();
            PH.setPack(p260);
            p260.mode_id = CAMERA_MODE.CAMERA_MODE_IMAGE;
            p260.time_boot_ms = (uint)1490733092U;
            CommunicationChannel.instance.send(p260);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSTORAGE_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.available_capacity == (float)8.711342E37F);
                Debug.Assert(pack.time_boot_ms == (uint)2460468395U);
                Debug.Assert(pack.status == (byte)(byte)58);
                Debug.Assert(pack.total_capacity == (float) -1.4730586E38F);
                Debug.Assert(pack.storage_count == (byte)(byte)88);
                Debug.Assert(pack.write_speed == (float) -2.060616E38F);
                Debug.Assert(pack.read_speed == (float) -2.1367537E38F);
                Debug.Assert(pack.storage_id == (byte)(byte)113);
                Debug.Assert(pack.used_capacity == (float)2.8341998E38F);
            };
            GroundControl.STORAGE_INFORMATION p261 = CommunicationChannel.new_STORAGE_INFORMATION();
            PH.setPack(p261);
            p261.status = (byte)(byte)58;
            p261.total_capacity = (float) -1.4730586E38F;
            p261.used_capacity = (float)2.8341998E38F;
            p261.time_boot_ms = (uint)2460468395U;
            p261.read_speed = (float) -2.1367537E38F;
            p261.available_capacity = (float)8.711342E37F;
            p261.storage_count = (byte)(byte)88;
            p261.write_speed = (float) -2.060616E38F;
            p261.storage_id = (byte)(byte)113;
            CommunicationChannel.instance.send(p261);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnCAMERA_CAPTURE_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)3517995984U);
                Debug.Assert(pack.video_status == (byte)(byte)80);
                Debug.Assert(pack.recording_time_ms == (uint)3948252730U);
                Debug.Assert(pack.image_status == (byte)(byte)117);
                Debug.Assert(pack.available_capacity == (float) -3.0102718E38F);
                Debug.Assert(pack.image_interval == (float) -6.2120574E37F);
            };
            GroundControl.CAMERA_CAPTURE_STATUS p262 = CommunicationChannel.new_CAMERA_CAPTURE_STATUS();
            PH.setPack(p262);
            p262.image_interval = (float) -6.2120574E37F;
            p262.recording_time_ms = (uint)3948252730U;
            p262.image_status = (byte)(byte)117;
            p262.time_boot_ms = (uint)3517995984U;
            p262.available_capacity = (float) -3.0102718E38F;
            p262.video_status = (byte)(byte)80;
            CommunicationChannel.instance.send(p262);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnCAMERA_IMAGE_CAPTUREDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.camera_id == (byte)(byte)38);
                Debug.Assert(pack.relative_alt == (int)730168900);
                Debug.Assert(pack.alt == (int)677383126);
                Debug.Assert(pack.capture_result == (sbyte)(sbyte)41);
                Debug.Assert(pack.time_boot_ms == (uint)1301965348U);
                Debug.Assert(pack.lat == (int)617297870);
                Debug.Assert(pack.time_utc == (ulong)4160445601512124418L);
                Debug.Assert(pack.image_index == (int) -1917713217);
                Debug.Assert(pack.file_url_LEN(ph) == 18);
                Debug.Assert(pack.file_url_TRY(ph).Equals("ktnmzjeqlyqvETniTv"));
                Debug.Assert(pack.q.SequenceEqual(new float[] {-2.8017492E38F, 2.3916764E38F, -2.402805E38F, 1.3782947E38F}));
                Debug.Assert(pack.lon == (int)565244058);
            };
            GroundControl.CAMERA_IMAGE_CAPTURED p263 = CommunicationChannel.new_CAMERA_IMAGE_CAPTURED();
            PH.setPack(p263);
            p263.alt = (int)677383126;
            p263.file_url_SET("ktnmzjeqlyqvETniTv", PH) ;
            p263.camera_id = (byte)(byte)38;
            p263.q_SET(new float[] {-2.8017492E38F, 2.3916764E38F, -2.402805E38F, 1.3782947E38F}, 0) ;
            p263.time_utc = (ulong)4160445601512124418L;
            p263.relative_alt = (int)730168900;
            p263.capture_result = (sbyte)(sbyte)41;
            p263.lon = (int)565244058;
            p263.image_index = (int) -1917713217;
            p263.time_boot_ms = (uint)1301965348U;
            p263.lat = (int)617297870;
            CommunicationChannel.instance.send(p263);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnFLIGHT_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.takeoff_time_utc == (ulong)6980663758515064103L);
                Debug.Assert(pack.flight_uuid == (ulong)926594885087576529L);
                Debug.Assert(pack.arming_time_utc == (ulong)8128323470129955397L);
                Debug.Assert(pack.time_boot_ms == (uint)2221561306U);
            };
            GroundControl.FLIGHT_INFORMATION p264 = CommunicationChannel.new_FLIGHT_INFORMATION();
            PH.setPack(p264);
            p264.arming_time_utc = (ulong)8128323470129955397L;
            p264.flight_uuid = (ulong)926594885087576529L;
            p264.takeoff_time_utc = (ulong)6980663758515064103L;
            p264.time_boot_ms = (uint)2221561306U;
            CommunicationChannel.instance.send(p264);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnMOUNT_ORIENTATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)3678642981U);
                Debug.Assert(pack.roll == (float)2.2231687E38F);
                Debug.Assert(pack.yaw == (float)4.2065484E37F);
                Debug.Assert(pack.pitch == (float)7.356892E36F);
            };
            GroundControl.MOUNT_ORIENTATION p265 = CommunicationChannel.new_MOUNT_ORIENTATION();
            PH.setPack(p265);
            p265.roll = (float)2.2231687E38F;
            p265.pitch = (float)7.356892E36F;
            p265.time_boot_ms = (uint)3678642981U;
            p265.yaw = (float)4.2065484E37F;
            CommunicationChannel.instance.send(p265);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnLOGGING_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)156);
                Debug.Assert(pack.length == (byte)(byte)144);
                Debug.Assert(pack.sequence == (ushort)(ushort)62925);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)140, (byte)148, (byte)112, (byte)129, (byte)98, (byte)159, (byte)85, (byte)226, (byte)34, (byte)110, (byte)111, (byte)172, (byte)157, (byte)197, (byte)142, (byte)230, (byte)242, (byte)226, (byte)77, (byte)246, (byte)251, (byte)49, (byte)161, (byte)116, (byte)10, (byte)97, (byte)176, (byte)219, (byte)116, (byte)86, (byte)181, (byte)241, (byte)4, (byte)63, (byte)105, (byte)230, (byte)205, (byte)84, (byte)54, (byte)8, (byte)176, (byte)41, (byte)201, (byte)183, (byte)150, (byte)14, (byte)226, (byte)146, (byte)202, (byte)9, (byte)45, (byte)69, (byte)38, (byte)74, (byte)224, (byte)134, (byte)10, (byte)55, (byte)125, (byte)226, (byte)15, (byte)19, (byte)92, (byte)62, (byte)16, (byte)47, (byte)117, (byte)62, (byte)100, (byte)185, (byte)170, (byte)35, (byte)2, (byte)76, (byte)103, (byte)240, (byte)47, (byte)74, (byte)49, (byte)205, (byte)207, (byte)113, (byte)192, (byte)113, (byte)217, (byte)145, (byte)138, (byte)71, (byte)209, (byte)55, (byte)0, (byte)246, (byte)205, (byte)83, (byte)149, (byte)96, (byte)82, (byte)109, (byte)128, (byte)187, (byte)23, (byte)51, (byte)174, (byte)74, (byte)131, (byte)114, (byte)16, (byte)158, (byte)115, (byte)234, (byte)16, (byte)175, (byte)23, (byte)162, (byte)77, (byte)204, (byte)166, (byte)52, (byte)30, (byte)180, (byte)37, (byte)206, (byte)160, (byte)52, (byte)139, (byte)193, (byte)140, (byte)27, (byte)220, (byte)247, (byte)30, (byte)157, (byte)253, (byte)35, (byte)250, (byte)81, (byte)235, (byte)214, (byte)225, (byte)166, (byte)192, (byte)251, (byte)23, (byte)105, (byte)157, (byte)211, (byte)33, (byte)135, (byte)112, (byte)33, (byte)208, (byte)11, (byte)155, (byte)175, (byte)230, (byte)110, (byte)170, (byte)185, (byte)114, (byte)6, (byte)250, (byte)124, (byte)125, (byte)237, (byte)52, (byte)245, (byte)43, (byte)129, (byte)158, (byte)16, (byte)30, (byte)155, (byte)112, (byte)241, (byte)126, (byte)143, (byte)97, (byte)206, (byte)186, (byte)28, (byte)217, (byte)160, (byte)255, (byte)223, (byte)79, (byte)151, (byte)141, (byte)164, (byte)255, (byte)203, (byte)154, (byte)238, (byte)9, (byte)120, (byte)123, (byte)14, (byte)148, (byte)214, (byte)81, (byte)118, (byte)75, (byte)31, (byte)237, (byte)54, (byte)98, (byte)95, (byte)29, (byte)1, (byte)48, (byte)68, (byte)240, (byte)72, (byte)170, (byte)226, (byte)171, (byte)79, (byte)34, (byte)13, (byte)254, (byte)186, (byte)163, (byte)187, (byte)152, (byte)187, (byte)99, (byte)160, (byte)116, (byte)121, (byte)126, (byte)85, (byte)16, (byte)12, (byte)50, (byte)220, (byte)30, (byte)84, (byte)59, (byte)81, (byte)39, (byte)19, (byte)23, (byte)159, (byte)218, (byte)215, (byte)11, (byte)19, (byte)21, (byte)206, (byte)180}));
                Debug.Assert(pack.target_component == (byte)(byte)31);
                Debug.Assert(pack.first_message_offset == (byte)(byte)63);
            };
            GroundControl.LOGGING_DATA p266 = CommunicationChannel.new_LOGGING_DATA();
            PH.setPack(p266);
            p266.target_component = (byte)(byte)31;
            p266.length = (byte)(byte)144;
            p266.first_message_offset = (byte)(byte)63;
            p266.sequence = (ushort)(ushort)62925;
            p266.data__SET(new byte[] {(byte)140, (byte)148, (byte)112, (byte)129, (byte)98, (byte)159, (byte)85, (byte)226, (byte)34, (byte)110, (byte)111, (byte)172, (byte)157, (byte)197, (byte)142, (byte)230, (byte)242, (byte)226, (byte)77, (byte)246, (byte)251, (byte)49, (byte)161, (byte)116, (byte)10, (byte)97, (byte)176, (byte)219, (byte)116, (byte)86, (byte)181, (byte)241, (byte)4, (byte)63, (byte)105, (byte)230, (byte)205, (byte)84, (byte)54, (byte)8, (byte)176, (byte)41, (byte)201, (byte)183, (byte)150, (byte)14, (byte)226, (byte)146, (byte)202, (byte)9, (byte)45, (byte)69, (byte)38, (byte)74, (byte)224, (byte)134, (byte)10, (byte)55, (byte)125, (byte)226, (byte)15, (byte)19, (byte)92, (byte)62, (byte)16, (byte)47, (byte)117, (byte)62, (byte)100, (byte)185, (byte)170, (byte)35, (byte)2, (byte)76, (byte)103, (byte)240, (byte)47, (byte)74, (byte)49, (byte)205, (byte)207, (byte)113, (byte)192, (byte)113, (byte)217, (byte)145, (byte)138, (byte)71, (byte)209, (byte)55, (byte)0, (byte)246, (byte)205, (byte)83, (byte)149, (byte)96, (byte)82, (byte)109, (byte)128, (byte)187, (byte)23, (byte)51, (byte)174, (byte)74, (byte)131, (byte)114, (byte)16, (byte)158, (byte)115, (byte)234, (byte)16, (byte)175, (byte)23, (byte)162, (byte)77, (byte)204, (byte)166, (byte)52, (byte)30, (byte)180, (byte)37, (byte)206, (byte)160, (byte)52, (byte)139, (byte)193, (byte)140, (byte)27, (byte)220, (byte)247, (byte)30, (byte)157, (byte)253, (byte)35, (byte)250, (byte)81, (byte)235, (byte)214, (byte)225, (byte)166, (byte)192, (byte)251, (byte)23, (byte)105, (byte)157, (byte)211, (byte)33, (byte)135, (byte)112, (byte)33, (byte)208, (byte)11, (byte)155, (byte)175, (byte)230, (byte)110, (byte)170, (byte)185, (byte)114, (byte)6, (byte)250, (byte)124, (byte)125, (byte)237, (byte)52, (byte)245, (byte)43, (byte)129, (byte)158, (byte)16, (byte)30, (byte)155, (byte)112, (byte)241, (byte)126, (byte)143, (byte)97, (byte)206, (byte)186, (byte)28, (byte)217, (byte)160, (byte)255, (byte)223, (byte)79, (byte)151, (byte)141, (byte)164, (byte)255, (byte)203, (byte)154, (byte)238, (byte)9, (byte)120, (byte)123, (byte)14, (byte)148, (byte)214, (byte)81, (byte)118, (byte)75, (byte)31, (byte)237, (byte)54, (byte)98, (byte)95, (byte)29, (byte)1, (byte)48, (byte)68, (byte)240, (byte)72, (byte)170, (byte)226, (byte)171, (byte)79, (byte)34, (byte)13, (byte)254, (byte)186, (byte)163, (byte)187, (byte)152, (byte)187, (byte)99, (byte)160, (byte)116, (byte)121, (byte)126, (byte)85, (byte)16, (byte)12, (byte)50, (byte)220, (byte)30, (byte)84, (byte)59, (byte)81, (byte)39, (byte)19, (byte)23, (byte)159, (byte)218, (byte)215, (byte)11, (byte)19, (byte)21, (byte)206, (byte)180}, 0) ;
            p266.target_system = (byte)(byte)156;
            CommunicationChannel.instance.send(p266);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnLOGGING_DATA_ACKEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.length == (byte)(byte)235);
                Debug.Assert(pack.target_component == (byte)(byte)57);
                Debug.Assert(pack.target_system == (byte)(byte)174);
                Debug.Assert(pack.sequence == (ushort)(ushort)53160);
                Debug.Assert(pack.first_message_offset == (byte)(byte)226);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)217, (byte)161, (byte)187, (byte)136, (byte)182, (byte)8, (byte)136, (byte)101, (byte)12, (byte)14, (byte)199, (byte)116, (byte)156, (byte)101, (byte)211, (byte)200, (byte)13, (byte)59, (byte)209, (byte)54, (byte)117, (byte)145, (byte)211, (byte)11, (byte)4, (byte)163, (byte)96, (byte)61, (byte)8, (byte)30, (byte)240, (byte)38, (byte)113, (byte)10, (byte)156, (byte)86, (byte)132, (byte)182, (byte)232, (byte)8, (byte)191, (byte)91, (byte)230, (byte)57, (byte)134, (byte)246, (byte)249, (byte)116, (byte)58, (byte)167, (byte)202, (byte)66, (byte)57, (byte)17, (byte)236, (byte)172, (byte)6, (byte)218, (byte)107, (byte)119, (byte)99, (byte)115, (byte)88, (byte)59, (byte)9, (byte)42, (byte)204, (byte)233, (byte)173, (byte)130, (byte)97, (byte)158, (byte)62, (byte)109, (byte)38, (byte)247, (byte)113, (byte)154, (byte)172, (byte)176, (byte)195, (byte)102, (byte)66, (byte)92, (byte)70, (byte)2, (byte)12, (byte)255, (byte)241, (byte)140, (byte)88, (byte)89, (byte)47, (byte)216, (byte)183, (byte)62, (byte)211, (byte)164, (byte)172, (byte)183, (byte)8, (byte)170, (byte)184, (byte)201, (byte)196, (byte)78, (byte)4, (byte)169, (byte)8, (byte)201, (byte)142, (byte)134, (byte)125, (byte)195, (byte)64, (byte)39, (byte)215, (byte)244, (byte)14, (byte)251, (byte)60, (byte)138, (byte)76, (byte)56, (byte)78, (byte)231, (byte)139, (byte)243, (byte)72, (byte)209, (byte)130, (byte)60, (byte)74, (byte)80, (byte)222, (byte)124, (byte)123, (byte)78, (byte)92, (byte)180, (byte)212, (byte)23, (byte)255, (byte)149, (byte)78, (byte)47, (byte)185, (byte)118, (byte)169, (byte)214, (byte)217, (byte)243, (byte)1, (byte)36, (byte)178, (byte)121, (byte)99, (byte)119, (byte)46, (byte)51, (byte)217, (byte)98, (byte)187, (byte)238, (byte)160, (byte)146, (byte)15, (byte)237, (byte)49, (byte)243, (byte)66, (byte)237, (byte)8, (byte)204, (byte)100, (byte)164, (byte)128, (byte)41, (byte)102, (byte)67, (byte)40, (byte)128, (byte)90, (byte)40, (byte)75, (byte)90, (byte)112, (byte)192, (byte)112, (byte)86, (byte)99, (byte)127, (byte)184, (byte)148, (byte)153, (byte)97, (byte)137, (byte)164, (byte)180, (byte)255, (byte)122, (byte)243, (byte)152, (byte)63, (byte)102, (byte)149, (byte)75, (byte)162, (byte)8, (byte)100, (byte)224, (byte)88, (byte)86, (byte)113, (byte)246, (byte)119, (byte)201, (byte)175, (byte)46, (byte)108, (byte)6, (byte)70, (byte)183, (byte)249, (byte)146, (byte)156, (byte)87, (byte)86, (byte)127, (byte)112, (byte)139, (byte)143, (byte)192, (byte)37, (byte)191, (byte)80, (byte)248, (byte)187, (byte)109, (byte)30, (byte)71, (byte)118, (byte)84, (byte)195, (byte)83, (byte)135, (byte)250, (byte)91, (byte)239}));
            };
            GroundControl.LOGGING_DATA_ACKED p267 = CommunicationChannel.new_LOGGING_DATA_ACKED();
            PH.setPack(p267);
            p267.first_message_offset = (byte)(byte)226;
            p267.target_system = (byte)(byte)174;
            p267.length = (byte)(byte)235;
            p267.sequence = (ushort)(ushort)53160;
            p267.data__SET(new byte[] {(byte)217, (byte)161, (byte)187, (byte)136, (byte)182, (byte)8, (byte)136, (byte)101, (byte)12, (byte)14, (byte)199, (byte)116, (byte)156, (byte)101, (byte)211, (byte)200, (byte)13, (byte)59, (byte)209, (byte)54, (byte)117, (byte)145, (byte)211, (byte)11, (byte)4, (byte)163, (byte)96, (byte)61, (byte)8, (byte)30, (byte)240, (byte)38, (byte)113, (byte)10, (byte)156, (byte)86, (byte)132, (byte)182, (byte)232, (byte)8, (byte)191, (byte)91, (byte)230, (byte)57, (byte)134, (byte)246, (byte)249, (byte)116, (byte)58, (byte)167, (byte)202, (byte)66, (byte)57, (byte)17, (byte)236, (byte)172, (byte)6, (byte)218, (byte)107, (byte)119, (byte)99, (byte)115, (byte)88, (byte)59, (byte)9, (byte)42, (byte)204, (byte)233, (byte)173, (byte)130, (byte)97, (byte)158, (byte)62, (byte)109, (byte)38, (byte)247, (byte)113, (byte)154, (byte)172, (byte)176, (byte)195, (byte)102, (byte)66, (byte)92, (byte)70, (byte)2, (byte)12, (byte)255, (byte)241, (byte)140, (byte)88, (byte)89, (byte)47, (byte)216, (byte)183, (byte)62, (byte)211, (byte)164, (byte)172, (byte)183, (byte)8, (byte)170, (byte)184, (byte)201, (byte)196, (byte)78, (byte)4, (byte)169, (byte)8, (byte)201, (byte)142, (byte)134, (byte)125, (byte)195, (byte)64, (byte)39, (byte)215, (byte)244, (byte)14, (byte)251, (byte)60, (byte)138, (byte)76, (byte)56, (byte)78, (byte)231, (byte)139, (byte)243, (byte)72, (byte)209, (byte)130, (byte)60, (byte)74, (byte)80, (byte)222, (byte)124, (byte)123, (byte)78, (byte)92, (byte)180, (byte)212, (byte)23, (byte)255, (byte)149, (byte)78, (byte)47, (byte)185, (byte)118, (byte)169, (byte)214, (byte)217, (byte)243, (byte)1, (byte)36, (byte)178, (byte)121, (byte)99, (byte)119, (byte)46, (byte)51, (byte)217, (byte)98, (byte)187, (byte)238, (byte)160, (byte)146, (byte)15, (byte)237, (byte)49, (byte)243, (byte)66, (byte)237, (byte)8, (byte)204, (byte)100, (byte)164, (byte)128, (byte)41, (byte)102, (byte)67, (byte)40, (byte)128, (byte)90, (byte)40, (byte)75, (byte)90, (byte)112, (byte)192, (byte)112, (byte)86, (byte)99, (byte)127, (byte)184, (byte)148, (byte)153, (byte)97, (byte)137, (byte)164, (byte)180, (byte)255, (byte)122, (byte)243, (byte)152, (byte)63, (byte)102, (byte)149, (byte)75, (byte)162, (byte)8, (byte)100, (byte)224, (byte)88, (byte)86, (byte)113, (byte)246, (byte)119, (byte)201, (byte)175, (byte)46, (byte)108, (byte)6, (byte)70, (byte)183, (byte)249, (byte)146, (byte)156, (byte)87, (byte)86, (byte)127, (byte)112, (byte)139, (byte)143, (byte)192, (byte)37, (byte)191, (byte)80, (byte)248, (byte)187, (byte)109, (byte)30, (byte)71, (byte)118, (byte)84, (byte)195, (byte)83, (byte)135, (byte)250, (byte)91, (byte)239}, 0) ;
            p267.target_component = (byte)(byte)57;
            CommunicationChannel.instance.send(p267);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnLOGGING_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)121);
                Debug.Assert(pack.sequence == (ushort)(ushort)4168);
                Debug.Assert(pack.target_system == (byte)(byte)172);
            };
            GroundControl.LOGGING_ACK p268 = CommunicationChannel.new_LOGGING_ACK();
            PH.setPack(p268);
            p268.target_component = (byte)(byte)121;
            p268.target_system = (byte)(byte)172;
            p268.sequence = (ushort)(ushort)4168;
            CommunicationChannel.instance.send(p268);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnVIDEO_STREAM_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.rotation == (ushort)(ushort)30178);
                Debug.Assert(pack.framerate == (float) -7.425056E37F);
                Debug.Assert(pack.resolution_v == (ushort)(ushort)6009);
                Debug.Assert(pack.status == (byte)(byte)84);
                Debug.Assert(pack.resolution_h == (ushort)(ushort)64899);
                Debug.Assert(pack.bitrate == (uint)2866408934U);
                Debug.Assert(pack.camera_id == (byte)(byte)118);
                Debug.Assert(pack.uri_LEN(ph) == 216);
                Debug.Assert(pack.uri_TRY(ph).Equals("xfzsicwjwubroAhIrnvxJvujsgmwaicrptnpmhuUTjdkmkzfognGjncltmGxwtWpaprfugvxmcqtsasltkkXKgvlxhggshcpbpqquxcveuuzahdimuonjhbtlzgkigoulegwazlgztxtsiIafcnmabpsnLvePhjhaqpyWqMmdiqrKeCihvnypzbsiozOvPpcmzhwJzxfpwxuhktgjubdtzAq"));
            };
            GroundControl.VIDEO_STREAM_INFORMATION p269 = CommunicationChannel.new_VIDEO_STREAM_INFORMATION();
            PH.setPack(p269);
            p269.uri_SET("xfzsicwjwubroAhIrnvxJvujsgmwaicrptnpmhuUTjdkmkzfognGjncltmGxwtWpaprfugvxmcqtsasltkkXKgvlxhggshcpbpqquxcveuuzahdimuonjhbtlzgkigoulegwazlgztxtsiIafcnmabpsnLvePhjhaqpyWqMmdiqrKeCihvnypzbsiozOvPpcmzhwJzxfpwxuhktgjubdtzAq", PH) ;
            p269.bitrate = (uint)2866408934U;
            p269.rotation = (ushort)(ushort)30178;
            p269.resolution_h = (ushort)(ushort)64899;
            p269.camera_id = (byte)(byte)118;
            p269.status = (byte)(byte)84;
            p269.resolution_v = (ushort)(ushort)6009;
            p269.framerate = (float) -7.425056E37F;
            CommunicationChannel.instance.send(p269);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSET_VIDEO_STREAM_SETTINGSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.bitrate == (uint)3949046976U);
                Debug.Assert(pack.target_component == (byte)(byte)124);
                Debug.Assert(pack.target_system == (byte)(byte)126);
                Debug.Assert(pack.resolution_h == (ushort)(ushort)21204);
                Debug.Assert(pack.rotation == (ushort)(ushort)870);
                Debug.Assert(pack.uri_LEN(ph) == 133);
                Debug.Assert(pack.uri_TRY(ph).Equals("qvvvlnwmbtdqcndbvnnvXqxxolreniwspvsekitwIfYhpuxeuhtwovbuWqoaioqxohvwfzxehzzrdueavnuvUdgoGdddpfwCyhehbVwwwcjospBzetQzrstxkdheuefdqrPQo"));
                Debug.Assert(pack.resolution_v == (ushort)(ushort)49021);
                Debug.Assert(pack.camera_id == (byte)(byte)139);
                Debug.Assert(pack.framerate == (float)1.5182662E38F);
            };
            GroundControl.SET_VIDEO_STREAM_SETTINGS p270 = CommunicationChannel.new_SET_VIDEO_STREAM_SETTINGS();
            PH.setPack(p270);
            p270.target_system = (byte)(byte)126;
            p270.target_component = (byte)(byte)124;
            p270.rotation = (ushort)(ushort)870;
            p270.uri_SET("qvvvlnwmbtdqcndbvnnvXqxxolreniwspvsekitwIfYhpuxeuhtwovbuWqoaioqxohvwfzxehzzrdueavnuvUdgoGdddpfwCyhehbVwwwcjospBzetQzrstxkdheuefdqrPQo", PH) ;
            p270.camera_id = (byte)(byte)139;
            p270.resolution_h = (ushort)(ushort)21204;
            p270.resolution_v = (ushort)(ushort)49021;
            p270.framerate = (float)1.5182662E38F;
            p270.bitrate = (uint)3949046976U;
            CommunicationChannel.instance.send(p270);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnWIFI_CONFIG_APReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.password_LEN(ph) == 42);
                Debug.Assert(pack.password_TRY(ph).Equals("rqgddywghkeyqoezkpdptmsdziuaemvjwukaaeygtv"));
                Debug.Assert(pack.ssid_LEN(ph) == 29);
                Debug.Assert(pack.ssid_TRY(ph).Equals("hqYnpDrjryvzcvfCawvVkhflcqary"));
            };
            GroundControl.WIFI_CONFIG_AP p299 = CommunicationChannel.new_WIFI_CONFIG_AP();
            PH.setPack(p299);
            p299.password_SET("rqgddywghkeyqoezkpdptmsdziuaemvjwukaaeygtv", PH) ;
            p299.ssid_SET("hqYnpDrjryvzcvfCawvVkhflcqary", PH) ;
            CommunicationChannel.instance.send(p299);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnPROTOCOL_VERSIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.min_version == (ushort)(ushort)15671);
                Debug.Assert(pack.spec_version_hash.SequenceEqual(new byte[] {(byte)217, (byte)71, (byte)121, (byte)228, (byte)118, (byte)172, (byte)181, (byte)236}));
                Debug.Assert(pack.max_version == (ushort)(ushort)57578);
                Debug.Assert(pack.version == (ushort)(ushort)50076);
                Debug.Assert(pack.library_version_hash.SequenceEqual(new byte[] {(byte)194, (byte)93, (byte)192, (byte)183, (byte)230, (byte)62, (byte)30, (byte)223}));
            };
            GroundControl.PROTOCOL_VERSION p300 = CommunicationChannel.new_PROTOCOL_VERSION();
            PH.setPack(p300);
            p300.min_version = (ushort)(ushort)15671;
            p300.spec_version_hash_SET(new byte[] {(byte)217, (byte)71, (byte)121, (byte)228, (byte)118, (byte)172, (byte)181, (byte)236}, 0) ;
            p300.library_version_hash_SET(new byte[] {(byte)194, (byte)93, (byte)192, (byte)183, (byte)230, (byte)62, (byte)30, (byte)223}, 0) ;
            p300.max_version = (ushort)(ushort)57578;
            p300.version = (ushort)(ushort)50076;
            CommunicationChannel.instance.send(p300);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnUAVCAN_NODE_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.uptime_sec == (uint)2430515162U);
                Debug.Assert(pack.sub_mode == (byte)(byte)138);
                Debug.Assert(pack.vendor_specific_status_code == (ushort)(ushort)291);
                Debug.Assert(pack.time_usec == (ulong)5295530768605368560L);
                Debug.Assert(pack.health == UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_ERROR);
                Debug.Assert(pack.mode == UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_OPERATIONAL);
            };
            GroundControl.UAVCAN_NODE_STATUS p310 = CommunicationChannel.new_UAVCAN_NODE_STATUS();
            PH.setPack(p310);
            p310.mode = UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_OPERATIONAL;
            p310.vendor_specific_status_code = (ushort)(ushort)291;
            p310.sub_mode = (byte)(byte)138;
            p310.uptime_sec = (uint)2430515162U;
            p310.time_usec = (ulong)5295530768605368560L;
            p310.health = UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_ERROR;
            CommunicationChannel.instance.send(p310);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnUAVCAN_NODE_INFOReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sw_vcs_commit == (uint)501762142U);
                Debug.Assert(pack.hw_version_major == (byte)(byte)27);
                Debug.Assert(pack.name_LEN(ph) == 14);
                Debug.Assert(pack.name_TRY(ph).Equals("utFLjanqormVec"));
                Debug.Assert(pack.hw_version_minor == (byte)(byte)218);
                Debug.Assert(pack.hw_unique_id.SequenceEqual(new byte[] {(byte)39, (byte)48, (byte)25, (byte)222, (byte)141, (byte)217, (byte)25, (byte)19, (byte)139, (byte)249, (byte)151, (byte)211, (byte)86, (byte)151, (byte)102, (byte)54}));
                Debug.Assert(pack.time_usec == (ulong)3647047582578442162L);
                Debug.Assert(pack.sw_version_minor == (byte)(byte)89);
                Debug.Assert(pack.uptime_sec == (uint)138083486U);
                Debug.Assert(pack.sw_version_major == (byte)(byte)89);
            };
            GroundControl.UAVCAN_NODE_INFO p311 = CommunicationChannel.new_UAVCAN_NODE_INFO();
            PH.setPack(p311);
            p311.hw_version_major = (byte)(byte)27;
            p311.uptime_sec = (uint)138083486U;
            p311.hw_unique_id_SET(new byte[] {(byte)39, (byte)48, (byte)25, (byte)222, (byte)141, (byte)217, (byte)25, (byte)19, (byte)139, (byte)249, (byte)151, (byte)211, (byte)86, (byte)151, (byte)102, (byte)54}, 0) ;
            p311.time_usec = (ulong)3647047582578442162L;
            p311.name_SET("utFLjanqormVec", PH) ;
            p311.hw_version_minor = (byte)(byte)218;
            p311.sw_version_major = (byte)(byte)89;
            p311.sw_version_minor = (byte)(byte)89;
            p311.sw_vcs_commit = (uint)501762142U;
            CommunicationChannel.instance.send(p311);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_REQUEST_READReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)5);
                Debug.Assert(pack.target_component == (byte)(byte)26);
                Debug.Assert(pack.param_index == (short)(short) -30945);
                Debug.Assert(pack.param_id_LEN(ph) == 8);
                Debug.Assert(pack.param_id_TRY(ph).Equals("vbWEhcXs"));
            };
            GroundControl.PARAM_EXT_REQUEST_READ p320 = CommunicationChannel.new_PARAM_EXT_REQUEST_READ();
            PH.setPack(p320);
            p320.param_id_SET("vbWEhcXs", PH) ;
            p320.param_index = (short)(short) -30945;
            p320.target_system = (byte)(byte)5;
            p320.target_component = (byte)(byte)26;
            CommunicationChannel.instance.send(p320);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)156);
                Debug.Assert(pack.target_system == (byte)(byte)126);
            };
            GroundControl.PARAM_EXT_REQUEST_LIST p321 = CommunicationChannel.new_PARAM_EXT_REQUEST_LIST();
            PH.setPack(p321);
            p321.target_component = (byte)(byte)156;
            p321.target_system = (byte)(byte)126;
            CommunicationChannel.instance.send(p321);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_VALUEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_type == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT32);
                Debug.Assert(pack.param_count == (ushort)(ushort)28411);
                Debug.Assert(pack.param_index == (ushort)(ushort)57373);
                Debug.Assert(pack.param_value_LEN(ph) == 95);
                Debug.Assert(pack.param_value_TRY(ph).Equals("jttellglsoavjedodlvumxvrftfeumuviwwsxhgtsiqcPxFcfktygffneoaxirmkimmhvzaofmbDzwCchduutdtrvsugibo"));
                Debug.Assert(pack.param_id_LEN(ph) == 3);
                Debug.Assert(pack.param_id_TRY(ph).Equals("efd"));
            };
            GroundControl.PARAM_EXT_VALUE p322 = CommunicationChannel.new_PARAM_EXT_VALUE();
            PH.setPack(p322);
            p322.param_value_SET("jttellglsoavjedodlvumxvrftfeumuviwwsxhgtsiqcPxFcfktygffneoaxirmkimmhvzaofmbDzwCchduutdtrvsugibo", PH) ;
            p322.param_count = (ushort)(ushort)28411;
            p322.param_index = (ushort)(ushort)57373;
            p322.param_type = MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT32;
            p322.param_id_SET("efd", PH) ;
            CommunicationChannel.instance.send(p322);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_SETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_id_LEN(ph) == 12);
                Debug.Assert(pack.param_id_TRY(ph).Equals("zchbnqostNwk"));
                Debug.Assert(pack.target_system == (byte)(byte)28);
                Debug.Assert(pack.target_component == (byte)(byte)158);
                Debug.Assert(pack.param_value_LEN(ph) == 6);
                Debug.Assert(pack.param_value_TRY(ph).Equals("kvmtnA"));
                Debug.Assert(pack.param_type == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT16);
            };
            GroundControl.PARAM_EXT_SET p323 = CommunicationChannel.new_PARAM_EXT_SET();
            PH.setPack(p323);
            p323.target_system = (byte)(byte)28;
            p323.param_type = MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT16;
            p323.param_value_SET("kvmtnA", PH) ;
            p323.param_id_SET("zchbnqostNwk", PH) ;
            p323.target_component = (byte)(byte)158;
            CommunicationChannel.instance.send(p323);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_result == PARAM_ACK.PARAM_ACK_FAILED);
                Debug.Assert(pack.param_id_LEN(ph) == 15);
                Debug.Assert(pack.param_id_TRY(ph).Equals("hrdbnczOcgkWkxs"));
                Debug.Assert(pack.param_value_LEN(ph) == 20);
                Debug.Assert(pack.param_value_TRY(ph).Equals("wcRwjsshextAizontYSg"));
                Debug.Assert(pack.param_type == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT8);
            };
            GroundControl.PARAM_EXT_ACK p324 = CommunicationChannel.new_PARAM_EXT_ACK();
            PH.setPack(p324);
            p324.param_id_SET("hrdbnczOcgkWkxs", PH) ;
            p324.param_type = MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT8;
            p324.param_value_SET("wcRwjsshextAizontYSg", PH) ;
            p324.param_result = PARAM_ACK.PARAM_ACK_FAILED;
            CommunicationChannel.instance.send(p324);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnOBSTACLE_DISTANCEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.distances.SequenceEqual(new ushort[] {(ushort)28765, (ushort)64639, (ushort)15851, (ushort)33113, (ushort)17366, (ushort)51335, (ushort)28785, (ushort)50432, (ushort)20897, (ushort)39545, (ushort)42089, (ushort)24442, (ushort)11985, (ushort)49563, (ushort)7609, (ushort)54255, (ushort)60762, (ushort)14359, (ushort)29999, (ushort)20220, (ushort)32089, (ushort)64504, (ushort)52, (ushort)51781, (ushort)16305, (ushort)5444, (ushort)24273, (ushort)41241, (ushort)12212, (ushort)51627, (ushort)58544, (ushort)9981, (ushort)20964, (ushort)38891, (ushort)50255, (ushort)60834, (ushort)59500, (ushort)29878, (ushort)62670, (ushort)30340, (ushort)11040, (ushort)61252, (ushort)25670, (ushort)7968, (ushort)56198, (ushort)54091, (ushort)14090, (ushort)31178, (ushort)37361, (ushort)38613, (ushort)45461, (ushort)1671, (ushort)9039, (ushort)53997, (ushort)50090, (ushort)45635, (ushort)18893, (ushort)21958, (ushort)20611, (ushort)40488, (ushort)13339, (ushort)24645, (ushort)19581, (ushort)4852, (ushort)57355, (ushort)49832, (ushort)48123, (ushort)45627, (ushort)10099, (ushort)59346, (ushort)46514, (ushort)11924}));
                Debug.Assert(pack.increment == (byte)(byte)190);
                Debug.Assert(pack.sensor_type == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_LASER);
                Debug.Assert(pack.max_distance == (ushort)(ushort)41703);
                Debug.Assert(pack.time_usec == (ulong)660375838823890899L);
                Debug.Assert(pack.min_distance == (ushort)(ushort)50456);
            };
            GroundControl.OBSTACLE_DISTANCE p330 = CommunicationChannel.new_OBSTACLE_DISTANCE();
            PH.setPack(p330);
            p330.time_usec = (ulong)660375838823890899L;
            p330.min_distance = (ushort)(ushort)50456;
            p330.distances_SET(new ushort[] {(ushort)28765, (ushort)64639, (ushort)15851, (ushort)33113, (ushort)17366, (ushort)51335, (ushort)28785, (ushort)50432, (ushort)20897, (ushort)39545, (ushort)42089, (ushort)24442, (ushort)11985, (ushort)49563, (ushort)7609, (ushort)54255, (ushort)60762, (ushort)14359, (ushort)29999, (ushort)20220, (ushort)32089, (ushort)64504, (ushort)52, (ushort)51781, (ushort)16305, (ushort)5444, (ushort)24273, (ushort)41241, (ushort)12212, (ushort)51627, (ushort)58544, (ushort)9981, (ushort)20964, (ushort)38891, (ushort)50255, (ushort)60834, (ushort)59500, (ushort)29878, (ushort)62670, (ushort)30340, (ushort)11040, (ushort)61252, (ushort)25670, (ushort)7968, (ushort)56198, (ushort)54091, (ushort)14090, (ushort)31178, (ushort)37361, (ushort)38613, (ushort)45461, (ushort)1671, (ushort)9039, (ushort)53997, (ushort)50090, (ushort)45635, (ushort)18893, (ushort)21958, (ushort)20611, (ushort)40488, (ushort)13339, (ushort)24645, (ushort)19581, (ushort)4852, (ushort)57355, (ushort)49832, (ushort)48123, (ushort)45627, (ushort)10099, (ushort)59346, (ushort)46514, (ushort)11924}, 0) ;
            p330.max_distance = (ushort)(ushort)41703;
            p330.increment = (byte)(byte)190;
            p330.sensor_type = MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_LASER;
            CommunicationChannel.instance.send(p330);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
        }
    }
}