
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
                    ulong id = id__v(value);
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
                    ulong id = id__R(value);
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
                    ulong id = id__R(value);
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
                    ulong id = id__I(value);
                    BitUtils.set_bits(id, 8, data, 276);
                }
            }

            public MAV_MISSION_TYPE mission_type //Mission type, see MAV_MISSION_TYPE
            {
                set
                {
                    ulong id = id__R(value);
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
                    ulong id = id__R(value);
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
                    ulong id = id__R(value);
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
                    ulong id = id__R(value);
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
                    ulong id = id__R(value);
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
                    ulong id = id__R(value);
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
                    ulong id = id__R(value);
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
                    ulong id = id__I(value);
                    BitUtils.set_bits(id, 8, data, 276);
                }
            }

            public MAV_MISSION_TYPE mission_type //Mission type, see MAV_MISSION_TYPE
            {
                set
                {
                    ulong id = id__R(value);
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
                    ulong id = id__I(value);
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
                    ulong id = id__I(value);
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
                    ulong id = id__I(value);
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
        public class POSITION_TARGET_GLOBAL_INT : GroundControl.POSITION_TARGET_GLOBAL_INT
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

            public int lat_int //X Position in WGS84 frame in 1e7 * meters
            {
                set {  BitUtils.set_bytes((uint)(value), 4, data,  6);}
            }

            public int lon_int //Y Position in WGS84 frame in 1e7 * meters
            {
                set {  BitUtils.set_bytes((uint)(value), 4, data,  10);}
            }

            public float alt //Altitude in meters in AMSL altitude, not WGS84 if absolute or relative, above terrain if GLOBAL_TERRAIN_ALT_IN
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
            *Valid options are: MAV_FRAME_GLOBAL_INT = 5, MAV_FRAME_GLOBAL_RELATIVE_ALT_INT = 6, MAV_FRAME_GLOBAL_TERRAIN_ALT_INT
            *	= 1*/
            public MAV_FRAME coordinate_frame
            {
                set {  BitUtils.set_bits((ulong)(- 0 +   value), 4, data, 400);}
            }
        }
        public class LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET : GroundControl.LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET
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

            public float roll //Roll
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 16);}
            }

            public float pitch //Pitch
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 20);}
            }

            public float yaw //Yaw
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 24);}
            }
        }
        public class HIL_STATE : GroundControl.HIL_STATE
        {
            public ulong time_usec //Timestamp (microseconds since UNIX epoch or microseconds since system boot)
            {
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  0);}
            }

            public float roll //Roll angle (rad)
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 8);}
            }

            public float pitch //Pitch angle (rad)
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 12);}
            }

            public float yaw //Yaw angle (rad)
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 16);}
            }

            public float rollspeed //Body frame roll / phi angular speed (rad/s)
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 20);}
            }

            public float pitchspeed //Body frame pitch / theta angular speed (rad/s)
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 24);}
            }

            public float yawspeed //Body frame yaw / psi angular speed (rad/s)
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 28);}
            }

            public int lat //Latitude, expressed as * 1E7
            {
                set {  BitUtils.set_bytes((uint)(value), 4, data,  32);}
            }

            public int lon //Longitude, expressed as * 1E7
            {
                set {  BitUtils.set_bytes((uint)(value), 4, data,  36);}
            }

            public int alt //Altitude in meters, expressed as * 1000 (millimeters)
            {
                set {  BitUtils.set_bytes((uint)(value), 4, data,  40);}
            }

            public short vx //Ground X Speed (Latitude), expressed as m/s * 100
            {
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  44);}
            }

            public short vy //Ground Y Speed (Longitude), expressed as m/s * 100
            {
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  46);}
            }

            public short vz //Ground Z Speed (Altitude), expressed as m/s * 100
            {
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  48);}
            }

            public short xacc //X acceleration (mg)
            {
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  50);}
            }

            public short yacc //Y acceleration (mg)
            {
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  52);}
            }

            public short zacc //Z acceleration (mg)
            {
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  54);}
            }
        }
        public class HIL_CONTROLS : GroundControl.HIL_CONTROLS
        {
            public ulong time_usec //Timestamp (microseconds since UNIX epoch or microseconds since system boot)
            {
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  0);}
            }

            public float roll_ailerons //Control output -1 .. 1
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 8);}
            }

            public float pitch_elevator //Control output -1 .. 1
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 12);}
            }

            public float yaw_rudder //Control output -1 .. 1
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 16);}
            }

            public float throttle //Throttle 0 .. 1
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 20);}
            }

            public float aux1 //Aux 1, -1 .. 1
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 24);}
            }

            public float aux2 //Aux 2, -1 .. 1
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 28);}
            }

            public float aux3 //Aux 3, -1 .. 1
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 32);}
            }

            public float aux4 //Aux 4, -1 .. 1
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 36);}
            }

            public byte nav_mode //Navigation mode (MAV_NAV_MODE)
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  40);}
            }

            public MAV_MODE mode //System mode (MAV_MODE)
            {
                set
                {
                    ulong id = id__v(value);
                    BitUtils.set_bits(id, 4, data, 328);
                }
            }
        }
        public class HIL_RC_INPUTS_RAW : GroundControl.HIL_RC_INPUTS_RAW
        {
            public ushort chan1_raw //RC channel 1 value, in microseconds
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public ushort chan2_raw //RC channel 2 value, in microseconds
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  2);}
            }

            public ushort chan3_raw //RC channel 3 value, in microseconds
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  4);}
            }

            public ushort chan4_raw //RC channel 4 value, in microseconds
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  6);}
            }

            public ushort chan5_raw //RC channel 5 value, in microseconds
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  8);}
            }

            public ushort chan6_raw //RC channel 6 value, in microseconds
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  10);}
            }

            public ushort chan7_raw //RC channel 7 value, in microseconds
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  12);}
            }

            public ushort chan8_raw //RC channel 8 value, in microseconds
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  14);}
            }

            public ushort chan9_raw //RC channel 9 value, in microseconds
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  16);}
            }

            public ushort chan10_raw //RC channel 10 value, in microseconds
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  18);}
            }

            public ushort chan11_raw //RC channel 11 value, in microseconds
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  20);}
            }

            public ushort chan12_raw //RC channel 12 value, in microseconds
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  22);}
            }

            public ulong time_usec //Timestamp (microseconds since UNIX epoch or microseconds since system boot)
            {
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  24);}
            }

            public byte rssi //Receive signal strength indicator, 0: 0%, 255: 100%
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  32);}
            }
        }
        public class HIL_ACTUATOR_CONTROLS : GroundControl.HIL_ACTUATOR_CONTROLS
        {
            public ulong time_usec //Timestamp (microseconds since UNIX epoch or microseconds since system boot)
            {
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  0);}
            }

            public ulong flags //Flags as bitfield, reserved for future use.
            {
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  8);}
            }

            public float[] controls //Control outputs -1 .. 1. Channel assignment depends on the simulated hardware.
            {
                set {controls_SET(value, 0)  ;}
            }
            public void controls_SET(float[] src, int pos)  //Control outputs -1 .. 1. Channel assignment depends on the simulated hardware.
            {
                for(int BYTE =  16, src_max = pos + 16; pos < src_max; pos++, BYTE += 4)
                    BitUtils.set_bytes(BitUtils.FloatToInt32Bits(src[pos]), 4, data, BYTE);
            }
            public MAV_MODE mode //System mode (MAV_MODE), includes arming state.
            {
                set
                {
                    ulong id = id__v(value);
                    BitUtils.set_bits(id, 4, data, 640);
                }
            }
        }
        public class OPTICAL_FLOW : GroundControl.OPTICAL_FLOW
        {
            public ulong time_usec //Timestamp (UNIX)
            {
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  0);}
            }

            public byte sensor_id //Sensor ID
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  8);}
            }

            public short flow_x //Flow in pixels * 10 in x-sensor direction (dezi-pixels)
            {
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  9);}
            }

            public short flow_y //Flow in pixels * 10 in y-sensor direction (dezi-pixels)
            {
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  11);}
            }

            public float flow_comp_m_x //Flow in meters in x-sensor direction, angular-speed compensated
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 13);}
            }

            public float flow_comp_m_y //Flow in meters in y-sensor direction, angular-speed compensated
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 17);}
            }

            public byte quality //Optical flow quality / confidence. 0: bad, 255: maximum quality
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  21);}
            }

            public float ground_distance //Ground distance in meters. Positive value: distance known. Negative value: Unknown distance
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 22);}
            }
            public void flow_rate_x_SET(float src, Inside ph)//Flow rate in radians/second about X axis
            {
                if(ph.field_bit != 208)insert_field(ph, 208, 0);
                BitUtils.set_bytes(BitUtils.FloatToInt32Bits(src), 4, data, ph.BYTE);
            } public void flow_rate_y_SET(float src, Inside ph) //Flow rate in radians/second about Y axis
            {
                if(ph.field_bit != 209)insert_field(ph, 209, 0);
                BitUtils.set_bytes(BitUtils.FloatToInt32Bits(src), 4, data, ph.BYTE);
            }
        }
        public class GLOBAL_VISION_POSITION_ESTIMATE : GroundControl.GLOBAL_VISION_POSITION_ESTIMATE
        {
            public ulong usec //Timestamp (microseconds, synced to UNIX time or since system boot)
            {
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  0);}
            }

            public float x //Global X position
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 8);}
            }

            public float y //Global Y position
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 12);}
            }

            public float z //Global Z position
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 16);}
            }

            public float roll //Roll angle in rad
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 20);}
            }

            public float pitch //Pitch angle in rad
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 24);}
            }

            public float yaw //Yaw angle in rad
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 28);}
            }
        }
        public class VISION_POSITION_ESTIMATE : GroundControl.VISION_POSITION_ESTIMATE
        {
            public ulong usec //Timestamp (microseconds, synced to UNIX time or since system boot)
            {
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  0);}
            }

            public float x //Global X position
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 8);}
            }

            public float y //Global Y position
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 12);}
            }

            public float z //Global Z position
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 16);}
            }

            public float roll //Roll angle in rad
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 20);}
            }

            public float pitch //Pitch angle in rad
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 24);}
            }

            public float yaw //Yaw angle in rad
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 28);}
            }
        }
        new class SENSOR_OFFSETS : GroundControl.SENSOR_OFFSETS
        {
            public short mag_ofs_x //magnetometer X offset
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  0, 2));}
            }

            public short mag_ofs_y //magnetometer Y offset
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  2, 2));}
            }

            public short mag_ofs_z //magnetometer Z offset
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  4, 2));}
            }

            public float mag_declination //magnetic declination (radians)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  6, 4)));}
            }

            public int raw_press //raw pressure from barometer
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  10, 4));}
            }

            public int raw_temp //raw temperature from barometer
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  14, 4));}
            }

            public float gyro_cal_x //gyro X calibration
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  18, 4)));}
            }

            public float gyro_cal_y //gyro Y calibration
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  22, 4)));}
            }

            public float gyro_cal_z //gyro Z calibration
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  26, 4)));}
            }

            public float accel_cal_x //accel X calibration
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  30, 4)));}
            }

            public float accel_cal_y //accel Y calibration
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  34, 4)));}
            }

            public float accel_cal_z //accel Z calibration
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  38, 4)));}
            }
        }
        new class SET_MAG_OFFSETS : GroundControl.SET_MAG_OFFSETS
        {
            public byte target_system //System ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  0, 1));}
            }

            public byte target_component //Component ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  1, 1));}
            }

            public short mag_ofs_x //magnetometer X offset
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  2, 2));}
            }

            public short mag_ofs_y //magnetometer Y offset
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  4, 2));}
            }

            public short mag_ofs_z //magnetometer Z offset
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  6, 2));}
            }
        }
        new class MEMINFO : GroundControl.MEMINFO
        {
            public ushort brkval //heap top
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
            }

            public ushort freemem //free memory
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  2, 2));}
            }
            public uint freemem32_TRY(Inside ph)//free memory (32 bit)
            {
                if(ph.field_bit !=  32 && !try_visit_field(ph, 32)) return 0;
                return (uint)((uint) BitUtils.get_bytes(data,  ph.BYTE, 4));
            }
        }
        new class AP_ADC : GroundControl.AP_ADC
        {
            public ushort adc1 //ADC output 1
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
            }

            public ushort adc2 //ADC output 2
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  2, 2));}
            }

            public ushort adc3 //ADC output 3
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  4, 2));}
            }

            public ushort adc4 //ADC output 4
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  6, 2));}
            }

            public ushort adc5 //ADC output 5
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  8, 2));}
            }

            public ushort adc6 //ADC output 6
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  10, 2));}
            }
        }
        new class DIGICAM_CONFIGURE : GroundControl.DIGICAM_CONFIGURE
        {
            public ushort shutter_speed //Divisor number e.g. 1000 means 1/1000 (0 means ignore)
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

            public byte mode //Mode enumeration from 1 to N P, TV, AV, M, Etc (0 means ignore)
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  4, 1));}
            }

            public byte aperture //F stop number x 10 e.g. 28 means 2.8 (0 means ignore)
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  5, 1));}
            }

            public byte iso //ISO enumeration from 1 to N e.g. 80, 100, 200, Etc (0 means ignore)
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  6, 1));}
            }

            public byte exposure_type //Exposure type enumeration from 1 to N (0 means ignore)
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  7, 1));}
            }

            /**
            *Command Identity (incremental loop: 0 to 255)A command sent multiple times will be executed or pooled
            *	just onc*/
            public byte command_id
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  8, 1));}
            }

            public byte engine_cut_off //Main engine cut-off time before camera trigger in seconds/10 (0 means no cut-off)
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  9, 1));}
            }

            public byte extra_param //Extra parameters enumeration (0 means ignore)
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  10, 1));}
            }

            public float extra_value //Correspondent value to given extra_param
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  11, 4)));}
            }
        }
        new class DIGICAM_CONTROL : GroundControl.DIGICAM_CONTROL
        {
            public byte target_system //System ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  0, 1));}
            }

            public byte target_component //Component ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  1, 1));}
            }

            public byte session //0: stop, 1: start or keep it up Session control e.g. show/hide lens
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  2, 1));}
            }

            public byte zoom_pos //1 to N Zoom's absolute position (0 means ignore)
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  3, 1));}
            }

            public sbyte zoom_step //-100 to 100 Zooming step value to offset zoom from the current position
            {
                get {  return (sbyte)((sbyte) BitUtils.get_bytes(data,  4, 1));}
            }

            public byte focus_lock //0: unlock focus or keep unlocked, 1: lock focus or keep locked, 3: re-lock focus
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  5, 1));}
            }

            public byte shot //0: ignore, 1: shot or start filming
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  6, 1));}
            }

            /**
            *Command Identity (incremental loop: 0 to 255)A command sent multiple times will be executed or pooled
            *	just onc*/
            public byte command_id
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  7, 1));}
            }

            public byte extra_param //Extra parameters enumeration (0 means ignore)
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  8, 1));}
            }

            public float extra_value //Correspondent value to given extra_param
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  9, 4)));}
            }
        }
        new class MOUNT_CONFIGURE : GroundControl.MOUNT_CONFIGURE
        {
            public byte target_system //System ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  0, 1));}
            }

            public byte target_component //Component ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  1, 1));}
            }

            public byte stab_roll //(1 = yes, 0 = no)
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  2, 1));}
            }

            public byte stab_pitch //(1 = yes, 0 = no)
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  3, 1));}
            }

            public byte stab_yaw //(1 = yes, 0 = no)
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  4, 1));}
            }

            public MAV_MOUNT_MODE mount_mode //mount operating mode (see MAV_MOUNT_MODE enum)
            {
                get {  return (MAV_MOUNT_MODE)(0 +  BitUtils.get_bits(data, 40, 3));}
            }
        }
        new class MOUNT_CONTROL : GroundControl.MOUNT_CONTROL
        {
            public byte target_system //System ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  0, 1));}
            }

            public byte target_component //Component ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  1, 1));}
            }

            public int input_a //pitch(deg*100) or lat, depending on mount mode
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  2, 4));}
            }

            public int input_b //roll(deg*100) or lon depending on mount mode
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  6, 4));}
            }

            public int input_c //yaw(deg*100) or alt (in cm) depending on mount mode
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  10, 4));}
            }

            public byte save_position //if "1" it will save current trimmed position on EEPROM (just valid for NEUTRAL and LANDING)
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  14, 1));}
            }
        }
        new class MOUNT_STATUS : GroundControl.MOUNT_STATUS
        {
            public byte target_system //System ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  0, 1));}
            }

            public byte target_component //Component ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  1, 1));}
            }

            public int pointing_a //pitch(deg*100)
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  2, 4));}
            }

            public int pointing_b //roll(deg*100)
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  6, 4));}
            }

            public int pointing_c //yaw(deg*100)
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  10, 4));}
            }
        }
        new class FENCE_POINT : GroundControl.FENCE_POINT
        {
            public byte target_system //System ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  0, 1));}
            }

            public byte target_component //Component ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  1, 1));}
            }

            public byte idx //point index (first point is 1, 0 is for return point)
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  2, 1));}
            }

            public byte count //total number of points (for sanity checking)
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  3, 1));}
            }

            public float lat //Latitude of point
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  4, 4)));}
            }

            public float lng //Longitude of point
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  8, 4)));}
            }
        }
        new class FENCE_FETCH_POINT : GroundControl.FENCE_FETCH_POINT
        {
            public byte target_system //System ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  0, 1));}
            }

            public byte target_component //Component ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  1, 1));}
            }

            public byte idx //point index (first point is 1, 0 is for return point)
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  2, 1));}
            }
        }
        new class FENCE_STATUS : GroundControl.FENCE_STATUS
        {
            public ushort breach_count //number of fence breaches
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
            }

            public uint breach_time //time of last breach in milliseconds since boot
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  2, 4));}
            }

            public byte breach_status //0 if currently inside fence, 1 if outside
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  6, 1));}
            }

            public FENCE_BREACH breach_type //last breach type (see FENCE_BREACH_* enum)
            {
                get {  return (FENCE_BREACH)(0 +  BitUtils.get_bits(data, 56, 2));}
            }
        }
        new class AHRS : GroundControl.AHRS
        {
            public float omegaIx //X gyro drift estimate rad/s
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  0, 4)));}
            }

            public float omegaIy //Y gyro drift estimate rad/s
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  4, 4)));}
            }

            public float omegaIz //Z gyro drift estimate rad/s
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  8, 4)));}
            }

            public float accel_weight //average accel_weight
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  12, 4)));}
            }

            public float renorm_val //average renormalisation value
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  16, 4)));}
            }

            public float error_rp //average error_roll_pitch value
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  20, 4)));}
            }

            public float error_yaw //average error_yaw value
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  24, 4)));}
            }
        }
        new class SIMSTATE : GroundControl.SIMSTATE
        {
            public float roll //Roll angle (rad)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  0, 4)));}
            }

            public float pitch //Pitch angle (rad)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  4, 4)));}
            }

            public float yaw //Yaw angle (rad)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  8, 4)));}
            }

            public float xacc //X acceleration m/s/s
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  12, 4)));}
            }

            public float yacc //Y acceleration m/s/s
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  16, 4)));}
            }

            public float zacc //Z acceleration m/s/s
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  20, 4)));}
            }

            public float xgyro //Angular speed around X axis rad/s
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  24, 4)));}
            }

            public float ygyro //Angular speed around Y axis rad/s
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  28, 4)));}
            }

            public float zgyro //Angular speed around Z axis rad/s
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  32, 4)));}
            }

            public int lat //Latitude in degrees * 1E7
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  36, 4));}
            }

            public int lng //Longitude in degrees * 1E7
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  40, 4));}
            }
        }
        new class HWSTATUS : GroundControl.HWSTATUS
        {
            public ushort Vcc //board voltage (mV)
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
            }

            public byte I2Cerr //I2C error count
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  2, 1));}
            }
        }
        new class RADIO : GroundControl.RADIO
        {
            public ushort rxerrors //receive errors
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
            }

            public ushort fixed_ //count of error corrected packets
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  2, 2));}
            }

            public byte rssi //local signal strength
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  4, 1));}
            }

            public byte remrssi //remote signal strength
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  5, 1));}
            }

            public byte txbuf //how full the tx buffer is as a percentage
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  6, 1));}
            }

            public byte noise //background noise level
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  7, 1));}
            }

            public byte remnoise //remote background noise level
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  8, 1));}
            }
        }
        new class LIMITS_STATUS : GroundControl.LIMITS_STATUS
        {
            public ushort breach_count //number of fence breaches
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
            }

            public uint last_trigger //time of last breach in milliseconds since boot
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  2, 4));}
            }

            public uint last_action //time of last recovery action in milliseconds since boot
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  6, 4));}
            }

            public uint last_recovery //time of last successful recovery in milliseconds since boot
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  10, 4));}
            }

            public uint last_clear //time of last all-clear in milliseconds since boot
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  14, 4));}
            }

            public LIMITS_STATE limits_state //state of AP_Limits, (see enum LimitState, LIMITS_STATE)
            {
                get {  return (LIMITS_STATE)(0 +  BitUtils.get_bits(data, 144, 3));}
            }

            public LIMIT_MODULE mods_enabled //AP_Limit_Module bitfield of enabled modules, (see enum moduleid or LIMIT_MODULE)
            {
                get {  return (LIMIT_MODULE)(1 +  BitUtils.get_bits(data, 147, 3));}
            }

            public LIMIT_MODULE mods_required //AP_Limit_Module bitfield of required modules, (see enum moduleid or LIMIT_MODULE)
            {
                get {  return (LIMIT_MODULE)(1 +  BitUtils.get_bits(data, 150, 3));}
            }

            public LIMIT_MODULE mods_triggered //AP_Limit_Module bitfield of triggered modules, (see enum moduleid or LIMIT_MODULE)
            {
                get {  return (LIMIT_MODULE)(1 +  BitUtils.get_bits(data, 153, 3));}
            }
        }
        new class WIND : GroundControl.WIND
        {
            public float direction //wind direction that wind is coming from (degrees)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  0, 4)));}
            }

            public float speed //wind speed in ground plane (m/s)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  4, 4)));}
            }

            public float speed_z //vertical wind speed (m/s)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  8, 4)));}
            }
        }
        new class DATA16 : GroundControl.DATA16
        {
            public byte type //data type
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  0, 1));}
            }

            public byte len //data length
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  1, 1));}
            }

            public byte[] data_ //raw data
            {
                get {return data__GET(new byte[16], 0);}
            }
            public byte[]data__GET(byte[] dst_ch, int pos)  //raw data
            {
                for(int BYTE = 2, dst_max = pos + 16; pos < dst_max ; pos++,  BYTE += 1)
                    dst_ch[pos] = (byte)((byte) BitUtils.get_bytes(data,  BYTE, 1));
                return dst_ch;
            }
        }
        new class DATA32 : GroundControl.DATA32
        {
            public byte type //data type
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  0, 1));}
            }

            public byte len //data length
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  1, 1));}
            }

            public byte[] data_ //raw data
            {
                get {return data__GET(new byte[32], 0);}
            }
            public byte[]data__GET(byte[] dst_ch, int pos)  //raw data
            {
                for(int BYTE = 2, dst_max = pos + 32; pos < dst_max ; pos++,  BYTE += 1)
                    dst_ch[pos] = (byte)((byte) BitUtils.get_bytes(data,  BYTE, 1));
                return dst_ch;
            }
        }
        new class DATA64 : GroundControl.DATA64
        {
            public byte type //data type
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  0, 1));}
            }

            public byte len //data length
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  1, 1));}
            }

            public byte[] data_ //raw data
            {
                get {return data__GET(new byte[64], 0);}
            }
            public byte[]data__GET(byte[] dst_ch, int pos)  //raw data
            {
                for(int BYTE = 2, dst_max = pos + 64; pos < dst_max ; pos++,  BYTE += 1)
                    dst_ch[pos] = (byte)((byte) BitUtils.get_bytes(data,  BYTE, 1));
                return dst_ch;
            }
        }
        new class DATA96 : GroundControl.DATA96
        {
            public byte type //data type
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  0, 1));}
            }

            public byte len //data length
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  1, 1));}
            }

            public byte[] data_ //raw data
            {
                get {return data__GET(new byte[96], 0);}
            }
            public byte[]data__GET(byte[] dst_ch, int pos)  //raw data
            {
                for(int BYTE = 2, dst_max = pos + 96; pos < dst_max ; pos++,  BYTE += 1)
                    dst_ch[pos] = (byte)((byte) BitUtils.get_bytes(data,  BYTE, 1));
                return dst_ch;
            }
        }
        new class RANGEFINDER : GroundControl.RANGEFINDER
        {
            public float distance //distance in meters
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  0, 4)));}
            }

            public float voltage //raw voltage if available, zero otherwise
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  4, 4)));}
            }
        }
        new class AIRSPEED_AUTOCAL : GroundControl.AIRSPEED_AUTOCAL
        {
            public float vx //GPS velocity north m/s
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  0, 4)));}
            }

            public float vy //GPS velocity east m/s
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  4, 4)));}
            }

            public float vz //GPS velocity down m/s
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  8, 4)));}
            }

            public float diff_pressure //Differential pressure pascals
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  12, 4)));}
            }

            public float EAS2TAS //Estimated to true airspeed ratio
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  16, 4)));}
            }

            public float ratio //Airspeed ratio
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  20, 4)));}
            }

            public float state_x //EKF state x
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  24, 4)));}
            }

            public float state_y //EKF state y
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  28, 4)));}
            }

            public float state_z //EKF state z
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  32, 4)));}
            }

            public float Pax //EKF Pax
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  36, 4)));}
            }

            public float Pby //EKF Pby
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  40, 4)));}
            }

            public float Pcz //EKF Pcz
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  44, 4)));}
            }
        }
        new class RALLY_POINT : GroundControl.RALLY_POINT
        {
            public ushort land_dir //Heading to aim for when landing. In centi-degrees.
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

            public byte idx //point index (first point is 0)
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  4, 1));}
            }

            public byte count //total number of points (for sanity checking)
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  5, 1));}
            }

            public int lat //Latitude of point in degrees * 1E7
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  6, 4));}
            }

            public int lng //Longitude of point in degrees * 1E7
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  10, 4));}
            }

            public short alt //Transit / loiter altitude in meters relative to home
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  14, 2));}
            }

            public short break_alt //Break altitude in meters relative to home
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  16, 2));}
            }

            public RALLY_FLAGS flags //See RALLY_FLAGS enum for definition of the bitmask.
            {
                get {  return (RALLY_FLAGS)(1 +  BitUtils.get_bits(data, 144, 1));}
            }
        }
        new class RALLY_FETCH_POINT : GroundControl.RALLY_FETCH_POINT
        {
            public byte target_system //System ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  0, 1));}
            }

            public byte target_component //Component ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  1, 1));}
            }

            public byte idx //point index (first point is 0)
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  2, 1));}
            }
        }
        new class COMPASSMOT_STATUS : GroundControl.COMPASSMOT_STATUS
        {
            public ushort throttle //throttle (percent*10)
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
            }

            public ushort interference //interference (percent)
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  2, 2));}
            }

            public float current //current (Ampere)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  4, 4)));}
            }

            public float CompensationX //Motor Compensation X
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  8, 4)));}
            }

            public float CompensationY //Motor Compensation Y
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  12, 4)));}
            }

            public float CompensationZ //Motor Compensation Z
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  16, 4)));}
            }
        }
        new class AHRS2 : GroundControl.AHRS2
        {
            public float roll //Roll angle (rad)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  0, 4)));}
            }

            public float pitch //Pitch angle (rad)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  4, 4)));}
            }

            public float yaw //Yaw angle (rad)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  8, 4)));}
            }

            public float altitude //Altitude (MSL)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  12, 4)));}
            }

            public int lat //Latitude in degrees * 1E7
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  16, 4));}
            }

            public int lng //Longitude in degrees * 1E7
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  20, 4));}
            }
        }
        new class CAMERA_STATUS : GroundControl.CAMERA_STATUS
        {
            public ushort img_idx //Image index
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
            }

            public ulong time_usec //Image timestamp (microseconds since UNIX epoch, according to camera clock)
            {
                get {  return (BitUtils.get_bytes(data,  2, 8));}
            }

            public byte target_system //System ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  10, 1));}
            }

            public byte cam_idx //Camera ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  11, 1));}
            }

            public float p1 //Parameter 1 (meaning depends on event, see CAMERA_STATUS_TYPES enum)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  12, 4)));}
            }

            public float p2 //Parameter 2 (meaning depends on event, see CAMERA_STATUS_TYPES enum)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  16, 4)));}
            }

            public float p3 //Parameter 3 (meaning depends on event, see CAMERA_STATUS_TYPES enum)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  20, 4)));}
            }

            public float p4 //Parameter 4 (meaning depends on event, see CAMERA_STATUS_TYPES enum)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  24, 4)));}
            }

            public CAMERA_STATUS_TYPES event_id //See CAMERA_STATUS_TYPES enum for definition of the bitmask
            {
                get {  return (CAMERA_STATUS_TYPES)(0 +  BitUtils.get_bits(data, 224, 3));}
            }
        }
        new class CAMERA_FEEDBACK : GroundControl.CAMERA_FEEDBACK
        {
            public ushort img_idx //Image index
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
            }

            /**
            *Image timestamp (microseconds since UNIX epoch), as passed in by CAMERA_STATUS message (or autopilot if
            *	no CCB*/
            public ulong time_usec
            {
                get {  return (BitUtils.get_bytes(data,  2, 8));}
            }

            public byte target_system //System ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  10, 1));}
            }

            public byte cam_idx //Camera ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  11, 1));}
            }

            public int lat //Latitude in (deg * 1E7)
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  12, 4));}
            }

            public int lng //Longitude in (deg * 1E7)
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  16, 4));}
            }

            public float alt_msl //Altitude Absolute (meters AMSL)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  20, 4)));}
            }

            public float alt_rel //Altitude Relative (meters above HOME location)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  24, 4)));}
            }

            public float roll //Camera Roll angle (earth frame, degrees, +-180)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  28, 4)));}
            }

            public float pitch //Camera Pitch angle (earth frame, degrees, +-180)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  32, 4)));}
            }

            public float yaw //Camera Yaw (earth frame, degrees, 0-360, true)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  36, 4)));}
            }

            public float foc_len //Focal Length (mm)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  40, 4)));}
            }

            public CAMERA_FEEDBACK_FLAGS flags //See CAMERA_FEEDBACK_FLAGS enum for definition of the bitmask
            {
                get {  return (CAMERA_FEEDBACK_FLAGS)(0 +  BitUtils.get_bits(data, 352, 3));}
            }
        }
        new class BATTERY2 : GroundControl.BATTERY2
        {
            public ushort voltage //voltage in millivolts
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
            }

            public short current_battery //Battery current, in 10*milliamperes (1 = 10 milliampere), -1: autopilot does not measure the curren
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  2, 2));}
            }
        }
        new class AHRS3 : GroundControl.AHRS3
        {
            public float roll //Roll angle (rad)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  0, 4)));}
            }

            public float pitch //Pitch angle (rad)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  4, 4)));}
            }

            public float yaw //Yaw angle (rad)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  8, 4)));}
            }

            public float altitude //Altitude (MSL)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  12, 4)));}
            }

            public int lat //Latitude in degrees * 1E7
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  16, 4));}
            }

            public int lng //Longitude in degrees * 1E7
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  20, 4));}
            }

            public float v1 //test variable1
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  24, 4)));}
            }

            public float v2 //test variable2
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  28, 4)));}
            }

            public float v3 //test variable3
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  32, 4)));}
            }

            public float v4 //test variable4
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  36, 4)));}
            }
        }
        new class AUTOPILOT_VERSION_REQUEST : GroundControl.AUTOPILOT_VERSION_REQUEST
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
        new class REMOTE_LOG_DATA_BLOCK : GroundControl.REMOTE_LOG_DATA_BLOCK
        {
            public byte target_system //System ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  0, 1));}
            }

            public byte target_component //Component ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  1, 1));}
            }

            public byte[] data_ //log data block
            {
                get {return data__GET(new byte[200], 0);}
            }
            public byte[]data__GET(byte[] dst_ch, int pos)  //log data block
            {
                for(int BYTE = 2, dst_max = pos + 200; pos < dst_max ; pos++,  BYTE += 1)
                    dst_ch[pos] = (byte)((byte) BitUtils.get_bytes(data,  BYTE, 1));
                return dst_ch;
            }
            public MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS seqno //log data block sequence number
            {
                get {  return (MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS)(2147483645 +  BitUtils.get_bits(data, 1616, 1));}
            }
        }
        new class REMOTE_LOG_BLOCK_STATUS : GroundControl.REMOTE_LOG_BLOCK_STATUS
        {
            public uint seqno //log data block sequence number
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  0, 4));}
            }

            public byte target_system //System ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  4, 1));}
            }

            public byte target_component //Component ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  5, 1));}
            }

            public MAV_REMOTE_LOG_DATA_BLOCK_STATUSES status //log data block status
            {
                get {  return (MAV_REMOTE_LOG_DATA_BLOCK_STATUSES)(0 +  BitUtils.get_bits(data, 48, 1));}
            }
        }
        new class LED_CONTROL : GroundControl.LED_CONTROL
        {
            public byte target_system //System ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  0, 1));}
            }

            public byte target_component //Component ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  1, 1));}
            }

            public byte instance //Instance (LED instance to control or 255 for all LEDs)
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  2, 1));}
            }

            public byte pattern //Pattern (see LED_PATTERN_ENUM)
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  3, 1));}
            }

            public byte custom_len //Custom Byte Length
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  4, 1));}
            }

            public byte[] custom_bytes //Custom Bytes
            {
                get {return custom_bytes_GET(new byte[24], 0);}
            }
            public byte[]custom_bytes_GET(byte[] dst_ch, int pos)  //Custom Bytes
            {
                for(int BYTE = 5, dst_max = pos + 24; pos < dst_max ; pos++,  BYTE += 1)
                    dst_ch[pos] = (byte)((byte) BitUtils.get_bytes(data,  BYTE, 1));
                return dst_ch;
            }
        }
        new class MAG_CAL_PROGRESS : GroundControl.MAG_CAL_PROGRESS
        {
            public byte compass_id //Compass being calibrated
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  0, 1));}
            }

            public byte cal_mask //Bitmask of compasses being calibrated
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  1, 1));}
            }

            public byte attempt //Attempt number
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  2, 1));}
            }

            public byte completion_pct //Completion percentage
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  3, 1));}
            }

            public byte[] completion_mask //Bitmask of sphere sections (see http:en.wikipedia.org/wiki/Geodesic_grid)
            {
                get {return completion_mask_GET(new byte[10], 0);}
            }
            public byte[]completion_mask_GET(byte[] dst_ch, int pos)  //Bitmask of sphere sections (see http:en.wikipedia.org/wiki/Geodesic_grid)
            {
                for(int BYTE = 4, dst_max = pos + 10; pos < dst_max ; pos++,  BYTE += 1)
                    dst_ch[pos] = (byte)((byte) BitUtils.get_bytes(data,  BYTE, 1));
                return dst_ch;
            }
            public float direction_x //Body frame direction vector for display
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  14, 4)));}
            }

            public float direction_y //Body frame direction vector for display
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  18, 4)));}
            }

            public float direction_z //Body frame direction vector for display
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  22, 4)));}
            }

            public MAG_CAL_STATUS cal_status //Status (see MAG_CAL_STATUS enum)
            {
                get {  return (MAG_CAL_STATUS)(0 +  BitUtils.get_bits(data, 208, 3));}
            }
        }
        new class MAG_CAL_REPORT : GroundControl.MAG_CAL_REPORT
        {
            public byte compass_id //Compass being calibrated
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  0, 1));}
            }

            public byte cal_mask //Bitmask of compasses being calibrated
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  1, 1));}
            }

            public byte autosaved //0=requires a MAV_CMD_DO_ACCEPT_MAG_CAL, 1=saved to parameters
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  2, 1));}
            }

            public float fitness //RMS milligauss residuals
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  3, 4)));}
            }

            public float ofs_x //X offset
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  7, 4)));}
            }

            public float ofs_y //Y offset
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  11, 4)));}
            }

            public float ofs_z //Z offset
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  15, 4)));}
            }

            public float diag_x //X diagonal (matrix 11)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  19, 4)));}
            }

            public float diag_y //Y diagonal (matrix 22)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  23, 4)));}
            }

            public float diag_z //Z diagonal (matrix 33)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  27, 4)));}
            }

            public float offdiag_x //X off-diagonal (matrix 12 and 21)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  31, 4)));}
            }

            public float offdiag_y //Y off-diagonal (matrix 13 and 31)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  35, 4)));}
            }

            public float offdiag_z //Z off-diagonal (matrix 32 and 23)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  39, 4)));}
            }

            public MAG_CAL_STATUS cal_status //Status (see MAG_CAL_STATUS enum)
            {
                get {  return (MAG_CAL_STATUS)(0 +  BitUtils.get_bits(data, 344, 3));}
            }
        }
        new class EKF_STATUS_REPORT : GroundControl.EKF_STATUS_REPORT
        {
            public float velocity_variance //Velocity variance
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  0, 4)));}
            }

            public float pos_horiz_variance //Horizontal Position variance
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  4, 4)));}
            }

            public float pos_vert_variance //Vertical Position variance
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  8, 4)));}
            }

            public float compass_variance //Compass variance
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  12, 4)));}
            }

            public float terrain_alt_variance //Terrain Altitude variance
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  16, 4)));}
            }

            public EKF_STATUS_FLAGS flags //Flags
            {
                get {  return (EKF_STATUS_FLAGS)(1 +  BitUtils.get_bits(data, 160, 10));}
            }
        }
        new class PID_TUNING : GroundControl.PID_TUNING
        {
            public float desired //desired rate (degrees/s)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  0, 4)));}
            }

            public float achieved //achieved rate (degrees/s)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  4, 4)));}
            }

            public float FF //FF component
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  8, 4)));}
            }

            public float P //P component
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  12, 4)));}
            }

            public float I //I component
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  16, 4)));}
            }

            public float D //D component
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  20, 4)));}
            }

            public PID_TUNING_AXIS axis //axis
            {
                get {  return (PID_TUNING_AXIS)(1 +  BitUtils.get_bits(data, 192, 3));}
            }
        }
        new class GIMBAL_REPORT : GroundControl.GIMBAL_REPORT
        {
            public byte target_system //System ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  0, 1));}
            }

            public byte target_component //Component ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  1, 1));}
            }

            public float delta_time //Time since last update (seconds)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  2, 4)));}
            }

            public float delta_angle_x //Delta angle X (radians)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  6, 4)));}
            }

            public float delta_angle_y //Delta angle Y (radians)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  10, 4)));}
            }

            public float delta_angle_z //Delta angle X (radians)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  14, 4)));}
            }

            public float delta_velocity_x //Delta velocity X (m/s)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  18, 4)));}
            }

            public float delta_velocity_y //Delta velocity Y (m/s)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  22, 4)));}
            }

            public float delta_velocity_z //Delta velocity Z (m/s)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  26, 4)));}
            }

            public float joint_roll //Joint ROLL (radians)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  30, 4)));}
            }

            public float joint_el //Joint EL (radians)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  34, 4)));}
            }

            public float joint_az //Joint AZ (radians)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  38, 4)));}
            }
        }
        new class GIMBAL_CONTROL : GroundControl.GIMBAL_CONTROL
        {
            public byte target_system //System ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  0, 1));}
            }

            public byte target_component //Component ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  1, 1));}
            }

            public float demanded_rate_x //Demanded angular rate X (rad/s)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  2, 4)));}
            }

            public float demanded_rate_y //Demanded angular rate Y (rad/s)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  6, 4)));}
            }

            public float demanded_rate_z //Demanded angular rate Z (rad/s)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  10, 4)));}
            }
        }
        new class GIMBAL_TORQUE_CMD_REPORT : GroundControl.GIMBAL_TORQUE_CMD_REPORT
        {
            public byte target_system //System ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  0, 1));}
            }

            public byte target_component //Component ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  1, 1));}
            }

            public short rl_torque_cmd //Roll Torque Command
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  2, 2));}
            }

            public short el_torque_cmd //Elevation Torque Command
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  4, 2));}
            }

            public short az_torque_cmd //Azimuth Torque Command
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  6, 2));}
            }
        }
        new class GOPRO_HEARTBEAT : GroundControl.GOPRO_HEARTBEAT
        {
            public GOPRO_HEARTBEAT_STATUS status //Status
            {
                get {  return (GOPRO_HEARTBEAT_STATUS)(0 +  BitUtils.get_bits(data, 0, 2));}
            }

            public GOPRO_CAPTURE_MODE capture_mode //Current capture mode
            {
                get
                {
                    switch((int)BitUtils.get_bits(data, 2, 4))
                    {
                        case 0:
                            return GOPRO_CAPTURE_MODE.GOPRO_CAPTURE_MODE_VIDEO;
                        case 1:
                            return GOPRO_CAPTURE_MODE.GOPRO_CAPTURE_MODE_PHOTO;
                        case 2:
                            return GOPRO_CAPTURE_MODE.GOPRO_CAPTURE_MODE_BURST;
                        case 3:
                            return GOPRO_CAPTURE_MODE.GOPRO_CAPTURE_MODE_TIME_LAPSE;
                        case 4:
                            return GOPRO_CAPTURE_MODE.GOPRO_CAPTURE_MODE_MULTI_SHOT;
                        case 5:
                            return GOPRO_CAPTURE_MODE.GOPRO_CAPTURE_MODE_PLAYBACK;
                        case 6:
                            return GOPRO_CAPTURE_MODE.GOPRO_CAPTURE_MODE_SETUP;
                        case 7:
                            return GOPRO_CAPTURE_MODE.GOPRO_CAPTURE_MODE_UNKNOWN;
                    }
                    throw  new ArgumentException("Unknown enum ID ");
                }
            }

            public GOPRO_HEARTBEAT_FLAGS flags //additional status bits
            {
                get {  return (GOPRO_HEARTBEAT_FLAGS)(1 +  BitUtils.get_bits(data, 6, 1));}
            }
        }
        new class GOPRO_GET_REQUEST : GroundControl.GOPRO_GET_REQUEST
        {
            public byte target_system //System ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  0, 1));}
            }

            public byte target_component //Component ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  1, 1));}
            }

            public GOPRO_COMMAND cmd_id //Command ID
            {
                get {  return (GOPRO_COMMAND)(0 +  BitUtils.get_bits(data, 16, 5));}
            }
        }
        new class GOPRO_GET_RESPONSE : GroundControl.GOPRO_GET_RESPONSE
        {
            public byte[] value //Value
            {
                get {return value_GET(new byte[4], 0);}
            }
            public byte[]value_GET(byte[] dst_ch, int pos)  //Value
            {
                for(int BYTE = 0, dst_max = pos + 4; pos < dst_max ; pos++,  BYTE += 1)
                    dst_ch[pos] = (byte)((byte) BitUtils.get_bytes(data,  BYTE, 1));
                return dst_ch;
            }
            public GOPRO_COMMAND cmd_id //Command ID
            {
                get {  return (GOPRO_COMMAND)(0 +  BitUtils.get_bits(data, 32, 5));}
            }

            public GOPRO_REQUEST_STATUS status //Status
            {
                get {  return (GOPRO_REQUEST_STATUS)(0 +  BitUtils.get_bits(data, 37, 1));}
            }
        }
        new class GOPRO_SET_REQUEST : GroundControl.GOPRO_SET_REQUEST
        {
            public byte target_system //System ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  0, 1));}
            }

            public byte target_component //Component ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  1, 1));}
            }

            public byte[] value //Value
            {
                get {return value_GET(new byte[4], 0);}
            }
            public byte[]value_GET(byte[] dst_ch, int pos)  //Value
            {
                for(int BYTE = 2, dst_max = pos + 4; pos < dst_max ; pos++,  BYTE += 1)
                    dst_ch[pos] = (byte)((byte) BitUtils.get_bytes(data,  BYTE, 1));
                return dst_ch;
            }
            public GOPRO_COMMAND cmd_id //Command ID
            {
                get {  return (GOPRO_COMMAND)(0 +  BitUtils.get_bits(data, 48, 5));}
            }
        }
        new class GOPRO_SET_RESPONSE : GroundControl.GOPRO_SET_RESPONSE
        {
            public GOPRO_COMMAND cmd_id //Command ID
            {
                get {  return (GOPRO_COMMAND)(0 +  BitUtils.get_bits(data, 0, 5));}
            }

            public GOPRO_REQUEST_STATUS status //Status
            {
                get {  return (GOPRO_REQUEST_STATUS)(0 +  BitUtils.get_bits(data, 5, 1));}
            }
        }
        new class RPM : GroundControl.RPM
        {
            public float rpm1 //RPM Sensor1
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  0, 4)));}
            }

            public float rpm2 //RPM Sensor2
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  4, 4)));}
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
        new class UAVIONIX_ADSB_OUT_CFG : GroundControl.UAVIONIX_ADSB_OUT_CFG
        {
            public ushort stallSpeed //Aircraft stall speed in cm/s
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
            }

            public uint ICAO //Vehicle address (24 bit)
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  2, 4));}
            }

            public ADSB_EMITTER_TYPE emitterType //Transmitting vehicle type. See ADSB_EMITTER_TYPE enum
            {
                get {  return (ADSB_EMITTER_TYPE)(0 +  BitUtils.get_bits(data, 48, 5));}
            }

            public UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE aircraftSize //Aircraft length and width encoding (table 2-35 of DO-282B)
            {
                get {  return (UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE)(0 +  BitUtils.get_bits(data, 53, 4));}
            }

            public UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT gpsOffsetLat //GPS antenna lateral offset (table 2-36 of DO-282B)
            {
                get {  return (UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT)(0 +  BitUtils.get_bits(data, 57, 3));}
            }

            /**
            *GPS antenna longitudinal offset from nose [if non-zero, take position (in meters) divide by 2 and add
            *	one] (table 2-37 DO-282B*/
            public UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON gpsOffsetLon
            {
                get {  return (UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON)(0 +  BitUtils.get_bits(data, 60, 1));}
            }

            public UAVIONIX_ADSB_OUT_RF_SELECT rfSelect //ADS-B transponder reciever and transmit enable flags
            {
                get {  return (UAVIONIX_ADSB_OUT_RF_SELECT)(0 +  BitUtils.get_bits(data, 61, 2));}
            }
            public string callsign_TRY(Inside ph)//Vehicle identifier (8 characters, null terminated, valid characters are A-Z, 0-9, " " only)
            {
                if(ph.field_bit !=  63 && !try_visit_field(ph, 63)  ||  !try_visit_item(ph, 0)) return null;
                return new string(callsign_GET(ph, new char[ph.items], 0));
            }
            public char[]callsign_GET(Inside ph, char[] dst_ch, int pos) //Vehicle identifier (8 characters, null terminated, valid characters are A-Z, 0-9, " " only)
            {
                for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (char)((char) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }
            public int callsign_LEN(Inside ph)
            {
                return (ph.field_bit !=  63 && !try_visit_field(ph, 63)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
            }
        }
        new class UAVIONIX_ADSB_OUT_DYNAMIC : GroundControl.UAVIONIX_ADSB_OUT_DYNAMIC
        {
            public ushort accuracyVert //Vertical accuracy in cm. If unknown set to UINT16_MAX
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
            }

            public ushort accuracyVel //Velocity accuracy in mm/s (m * 1E-3). If unknown set to UINT16_MAX
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  2, 2));}
            }

            public ushort squawk //Mode A code (typically 1200 [0x04B0] for VFR)
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  4, 2));}
            }

            public uint utcTime //UTC time in seconds since GPS epoch (Jan 6, 1980). If unknown set to UINT32_MAX
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  6, 4));}
            }

            public uint accuracyHor //Horizontal accuracy in mm (m * 1E-3). If unknown set to UINT32_MAX
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  10, 4));}
            }

            public int gpsLat //Latitude WGS84 (deg * 1E7). If unknown set to INT32_MAX
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  14, 4));}
            }

            public int gpsLon //Longitude WGS84 (deg * 1E7). If unknown set to INT32_MAX
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  18, 4));}
            }

            public int gpsAlt //Altitude in mm (m * 1E-3) UP +ve. WGS84 altitude. If unknown set to INT32_MAX
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  22, 4));}
            }

            public byte numSats //Number of satellites visible. If unknown set to UINT8_MAX
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  26, 1));}
            }

            /**
            *Barometric pressure altitude relative to a standard atmosphere of 1013.2 mBar and NOT bar corrected altitude
            *	(m * 1E-3). (up +ve). If unknown set to INT32_MA*/
            public int baroAltMSL
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  27, 4));}
            }

            public short velVert //GPS vertical speed in cm/s. If unknown set to INT16_MAX
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  31, 2));}
            }

            public short velNS //North-South velocity over ground in cm/s North +ve. If unknown set to INT16_MAX
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  33, 2));}
            }

            public short VelEW //East-West velocity over ground in cm/s East +ve. If unknown set to INT16_MAX
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  35, 2));}
            }

            public UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX gpsFix //0-1: no fix, 2: 2D fix, 3: 3D fix, 4: DGPS, 5: RTK
            {
                get {  return (UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX)(0 +  BitUtils.get_bits(data, 296, 3));}
            }

            public UAVIONIX_ADSB_EMERGENCY_STATUS emergencyStatus //Emergency status
            {
                get {  return (UAVIONIX_ADSB_EMERGENCY_STATUS)(0 +  BitUtils.get_bits(data, 299, 3));}
            }

            public UAVIONIX_ADSB_OUT_DYNAMIC_STATE state //ADS-B transponder dynamic input state flags
            {
                get {  return (UAVIONIX_ADSB_OUT_DYNAMIC_STATE)(1 +  BitUtils.get_bits(data, 302, 5));}
            }
        }
        new class UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT : GroundControl.UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT
        {
            public UAVIONIX_ADSB_RF_HEALTH rfHealth //ADS-B transponder messages
            {
                get
                {
                    switch((int)BitUtils.get_bits(data, 0, 3))
                    {
                        case 0:
                            return UAVIONIX_ADSB_RF_HEALTH.UAVIONIX_ADSB_RF_HEALTH_INITIALIZING;
                        case 1:
                            return UAVIONIX_ADSB_RF_HEALTH.UAVIONIX_ADSB_RF_HEALTH_OK;
                        case 2:
                            return UAVIONIX_ADSB_RF_HEALTH.UAVIONIX_ADSB_RF_HEALTH_FAIL_TX;
                        case 3:
                            return UAVIONIX_ADSB_RF_HEALTH.UAVIONIX_ADSB_RF_HEALTH_FAIL_RX;
                    }
                    throw  new ArgumentException("Unknown enum ID ");
                }
            }
        }
        new class DEVICE_OP_READ : GroundControl.DEVICE_OP_READ
        {
            public uint request_id //request ID - copied to reply
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  0, 4));}
            }

            public byte target_system //System ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  4, 1));}
            }

            public byte target_component //Component ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  5, 1));}
            }

            public byte bus //Bus number
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  6, 1));}
            }

            public byte address //Bus address
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  7, 1));}
            }

            public byte regstart //First register to read
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  8, 1));}
            }

            public byte count //count of registers to read
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  9, 1));}
            }

            public DEVICE_OP_BUSTYPE bustype //The bus type
            {
                get {  return (DEVICE_OP_BUSTYPE)(0 +  BitUtils.get_bits(data, 80, 1));}
            }
            public string busname_TRY(Inside ph)//Name of device on bus (for SPI)
            {
                if(ph.field_bit !=  81 && !try_visit_field(ph, 81)  ||  !try_visit_item(ph, 0)) return null;
                return new string(busname_GET(ph, new char[ph.items], 0));
            }
            public char[]busname_GET(Inside ph, char[] dst_ch, int pos) //Name of device on bus (for SPI)
            {
                for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (char)((char) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }
            public int busname_LEN(Inside ph)
            {
                return (ph.field_bit !=  81 && !try_visit_field(ph, 81)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
            }
        }
        new class DEVICE_OP_READ_REPLY : GroundControl.DEVICE_OP_READ_REPLY
        {
            public uint request_id //request ID - copied from request
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  0, 4));}
            }

            public byte result //0 for success, anything else is failure code
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  4, 1));}
            }

            public byte regstart //starting register
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  5, 1));}
            }

            public byte count //count of bytes read
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  6, 1));}
            }

            public byte[] data_ //reply data
            {
                get {return data__GET(new byte[128], 0);}
            }
            public byte[]data__GET(byte[] dst_ch, int pos)  //reply data
            {
                for(int BYTE = 7, dst_max = pos + 128; pos < dst_max ; pos++,  BYTE += 1)
                    dst_ch[pos] = (byte)((byte) BitUtils.get_bytes(data,  BYTE, 1));
                return dst_ch;
            }
        }
        new class DEVICE_OP_WRITE : GroundControl.DEVICE_OP_WRITE
        {
            public uint request_id //request ID - copied to reply
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  0, 4));}
            }

            public byte target_system //System ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  4, 1));}
            }

            public byte target_component //Component ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  5, 1));}
            }

            public byte bus //Bus number
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  6, 1));}
            }

            public byte address //Bus address
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  7, 1));}
            }

            public byte regstart //First register to write
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  8, 1));}
            }

            public byte count //count of registers to write
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  9, 1));}
            }

            public byte[] data_ //write data
            {
                get {return data__GET(new byte[128], 0);}
            }
            public byte[]data__GET(byte[] dst_ch, int pos)  //write data
            {
                for(int BYTE = 10, dst_max = pos + 128; pos < dst_max ; pos++,  BYTE += 1)
                    dst_ch[pos] = (byte)((byte) BitUtils.get_bytes(data,  BYTE, 1));
                return dst_ch;
            }
            public DEVICE_OP_BUSTYPE bustype //The bus type
            {
                get {  return (DEVICE_OP_BUSTYPE)(0 +  BitUtils.get_bits(data, 1104, 1));}
            }
            public string busname_TRY(Inside ph)//Name of device on bus (for SPI)
            {
                if(ph.field_bit !=  1105 && !try_visit_field(ph, 1105)  ||  !try_visit_item(ph, 0)) return null;
                return new string(busname_GET(ph, new char[ph.items], 0));
            }
            public char[]busname_GET(Inside ph, char[] dst_ch, int pos) //Name of device on bus (for SPI)
            {
                for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (char)((char) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }
            public int busname_LEN(Inside ph)
            {
                return (ph.field_bit !=  1105 && !try_visit_field(ph, 1105)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
            }
        }
        new class DEVICE_OP_WRITE_REPLY : GroundControl.DEVICE_OP_WRITE_REPLY
        {
            public uint request_id //request ID - copied from request
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  0, 4));}
            }

            public byte result //0 for success, anything else is failure code
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  4, 1));}
            }
        }
        new class ADAP_TUNING : GroundControl.ADAP_TUNING
        {
            public float desired //desired rate (degrees/s)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  0, 4)));}
            }

            public float achieved //achieved rate (degrees/s)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  4, 4)));}
            }

            public float error //error between model and vehicle
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  8, 4)));}
            }

            public float theta //theta estimated state predictor
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  12, 4)));}
            }

            public float omega //omega estimated state predictor
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  16, 4)));}
            }

            public float sigma //sigma estimated state predictor
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  20, 4)));}
            }

            public float theta_dot //theta derivative
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  24, 4)));}
            }

            public float omega_dot //omega derivative
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  28, 4)));}
            }

            public float sigma_dot //sigma derivative
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  32, 4)));}
            }

            public float f //projection operator value
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  36, 4)));}
            }

            public float f_dot //projection operator derivative
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  40, 4)));}
            }

            public float u //u adaptive controlled output command
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  44, 4)));}
            }

            public PID_TUNING_AXIS axis //axis
            {
                get {  return (PID_TUNING_AXIS)(1 +  BitUtils.get_bits(data, 384, 3));}
            }
        }
        new class VISION_POSITION_DELTA : GroundControl.VISION_POSITION_DELTA
        {
            public ulong time_usec //Timestamp (microseconds, synced to UNIX time or since system boot)
            {
                get {  return (BitUtils.get_bytes(data,  0, 8));}
            }

            public ulong time_delta_usec //Time in microseconds since the last reported camera frame
            {
                get {  return (BitUtils.get_bytes(data,  8, 8));}
            }

            public float[] angle_delta //Defines a rotation vector in body frame that rotates the vehicle from the previous to the current orientatio
            {
                get {return angle_delta_GET(new float[3], 0);}
            }
            public float[]angle_delta_GET(float[] dst_ch, int pos)  //Defines a rotation vector in body frame that rotates the vehicle from the previous to the current orientatio
            {
                for(int BYTE = 16, dst_max = pos + 3; pos < dst_max ; pos++,  BYTE += 4)
                    dst_ch[pos] = (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  BYTE, 4)));
                return dst_ch;
            }
            /**
            *Change in position in meters from previous to current frame rotated into body frame (0=forward, 1=right,
            *	2=down*/
            public float[] position_delta
            {
                get {return position_delta_GET(new float[3], 0);}
            }
            /**
            *Change in position in meters from previous to current frame rotated into body frame (0=forward, 1=right,
            *	2=down*/
            public float[]position_delta_GET(float[] dst_ch, int pos)
            {
                for(int BYTE = 28, dst_max = pos + 3; pos < dst_max ; pos++,  BYTE += 4)
                    dst_ch[pos] = (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  BYTE, 4)));
                return dst_ch;
            }
            public float confidence //normalised confidence value from 0 to 100
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  40, 4)));}
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

            public void OnSENSOR_OFFSETSReceive_direct(Channel src, Inside ph, SENSOR_OFFSETS pack) {OnSENSOR_OFFSETSReceive(this, ph,  pack);}
            public event SENSOR_OFFSETSReceiveHandler OnSENSOR_OFFSETSReceive;
            public delegate void SENSOR_OFFSETSReceiveHandler(Channel src, Inside ph, SENSOR_OFFSETS pack);
            public void OnSET_MAG_OFFSETSReceive_direct(Channel src, Inside ph, SET_MAG_OFFSETS pack) {OnSET_MAG_OFFSETSReceive(this, ph,  pack);}
            public event SET_MAG_OFFSETSReceiveHandler OnSET_MAG_OFFSETSReceive;
            public delegate void SET_MAG_OFFSETSReceiveHandler(Channel src, Inside ph, SET_MAG_OFFSETS pack);
            public void OnMEMINFOReceive_direct(Channel src, Inside ph, MEMINFO pack) {OnMEMINFOReceive(this, ph,  pack);}
            public event MEMINFOReceiveHandler OnMEMINFOReceive;
            public delegate void MEMINFOReceiveHandler(Channel src, Inside ph, MEMINFO pack);
            public void OnAP_ADCReceive_direct(Channel src, Inside ph, AP_ADC pack) {OnAP_ADCReceive(this, ph,  pack);}
            public event AP_ADCReceiveHandler OnAP_ADCReceive;
            public delegate void AP_ADCReceiveHandler(Channel src, Inside ph, AP_ADC pack);
            public void OnDIGICAM_CONFIGUREReceive_direct(Channel src, Inside ph, DIGICAM_CONFIGURE pack) {OnDIGICAM_CONFIGUREReceive(this, ph,  pack);}
            public event DIGICAM_CONFIGUREReceiveHandler OnDIGICAM_CONFIGUREReceive;
            public delegate void DIGICAM_CONFIGUREReceiveHandler(Channel src, Inside ph, DIGICAM_CONFIGURE pack);
            public void OnDIGICAM_CONTROLReceive_direct(Channel src, Inside ph, DIGICAM_CONTROL pack) {OnDIGICAM_CONTROLReceive(this, ph,  pack);}
            public event DIGICAM_CONTROLReceiveHandler OnDIGICAM_CONTROLReceive;
            public delegate void DIGICAM_CONTROLReceiveHandler(Channel src, Inside ph, DIGICAM_CONTROL pack);
            public void OnMOUNT_CONFIGUREReceive_direct(Channel src, Inside ph, MOUNT_CONFIGURE pack) {OnMOUNT_CONFIGUREReceive(this, ph,  pack);}
            public event MOUNT_CONFIGUREReceiveHandler OnMOUNT_CONFIGUREReceive;
            public delegate void MOUNT_CONFIGUREReceiveHandler(Channel src, Inside ph, MOUNT_CONFIGURE pack);
            public void OnMOUNT_CONTROLReceive_direct(Channel src, Inside ph, MOUNT_CONTROL pack) {OnMOUNT_CONTROLReceive(this, ph,  pack);}
            public event MOUNT_CONTROLReceiveHandler OnMOUNT_CONTROLReceive;
            public delegate void MOUNT_CONTROLReceiveHandler(Channel src, Inside ph, MOUNT_CONTROL pack);
            public void OnMOUNT_STATUSReceive_direct(Channel src, Inside ph, MOUNT_STATUS pack) {OnMOUNT_STATUSReceive(this, ph,  pack);}
            public event MOUNT_STATUSReceiveHandler OnMOUNT_STATUSReceive;
            public delegate void MOUNT_STATUSReceiveHandler(Channel src, Inside ph, MOUNT_STATUS pack);
            public void OnFENCE_POINTReceive_direct(Channel src, Inside ph, FENCE_POINT pack) {OnFENCE_POINTReceive(this, ph,  pack);}
            public event FENCE_POINTReceiveHandler OnFENCE_POINTReceive;
            public delegate void FENCE_POINTReceiveHandler(Channel src, Inside ph, FENCE_POINT pack);
            public void OnFENCE_FETCH_POINTReceive_direct(Channel src, Inside ph, FENCE_FETCH_POINT pack) {OnFENCE_FETCH_POINTReceive(this, ph,  pack);}
            public event FENCE_FETCH_POINTReceiveHandler OnFENCE_FETCH_POINTReceive;
            public delegate void FENCE_FETCH_POINTReceiveHandler(Channel src, Inside ph, FENCE_FETCH_POINT pack);
            public void OnFENCE_STATUSReceive_direct(Channel src, Inside ph, FENCE_STATUS pack) {OnFENCE_STATUSReceive(this, ph,  pack);}
            public event FENCE_STATUSReceiveHandler OnFENCE_STATUSReceive;
            public delegate void FENCE_STATUSReceiveHandler(Channel src, Inside ph, FENCE_STATUS pack);
            public void OnAHRSReceive_direct(Channel src, Inside ph, AHRS pack) {OnAHRSReceive(this, ph,  pack);}
            public event AHRSReceiveHandler OnAHRSReceive;
            public delegate void AHRSReceiveHandler(Channel src, Inside ph, AHRS pack);
            public void OnSIMSTATEReceive_direct(Channel src, Inside ph, SIMSTATE pack) {OnSIMSTATEReceive(this, ph,  pack);}
            public event SIMSTATEReceiveHandler OnSIMSTATEReceive;
            public delegate void SIMSTATEReceiveHandler(Channel src, Inside ph, SIMSTATE pack);
            public void OnHWSTATUSReceive_direct(Channel src, Inside ph, HWSTATUS pack) {OnHWSTATUSReceive(this, ph,  pack);}
            public event HWSTATUSReceiveHandler OnHWSTATUSReceive;
            public delegate void HWSTATUSReceiveHandler(Channel src, Inside ph, HWSTATUS pack);
            public void OnRADIOReceive_direct(Channel src, Inside ph, RADIO pack) {OnRADIOReceive(this, ph,  pack);}
            public event RADIOReceiveHandler OnRADIOReceive;
            public delegate void RADIOReceiveHandler(Channel src, Inside ph, RADIO pack);
            public void OnLIMITS_STATUSReceive_direct(Channel src, Inside ph, LIMITS_STATUS pack) {OnLIMITS_STATUSReceive(this, ph,  pack);}
            public event LIMITS_STATUSReceiveHandler OnLIMITS_STATUSReceive;
            public delegate void LIMITS_STATUSReceiveHandler(Channel src, Inside ph, LIMITS_STATUS pack);
            public void OnWINDReceive_direct(Channel src, Inside ph, WIND pack) {OnWINDReceive(this, ph,  pack);}
            public event WINDReceiveHandler OnWINDReceive;
            public delegate void WINDReceiveHandler(Channel src, Inside ph, WIND pack);
            public void OnDATA16Receive_direct(Channel src, Inside ph, DATA16 pack) {OnDATA16Receive(this, ph,  pack);}
            public event DATA16ReceiveHandler OnDATA16Receive;
            public delegate void DATA16ReceiveHandler(Channel src, Inside ph, DATA16 pack);
            public void OnDATA32Receive_direct(Channel src, Inside ph, DATA32 pack) {OnDATA32Receive(this, ph,  pack);}
            public event DATA32ReceiveHandler OnDATA32Receive;
            public delegate void DATA32ReceiveHandler(Channel src, Inside ph, DATA32 pack);
            public void OnDATA64Receive_direct(Channel src, Inside ph, DATA64 pack) {OnDATA64Receive(this, ph,  pack);}
            public event DATA64ReceiveHandler OnDATA64Receive;
            public delegate void DATA64ReceiveHandler(Channel src, Inside ph, DATA64 pack);
            public void OnDATA96Receive_direct(Channel src, Inside ph, DATA96 pack) {OnDATA96Receive(this, ph,  pack);}
            public event DATA96ReceiveHandler OnDATA96Receive;
            public delegate void DATA96ReceiveHandler(Channel src, Inside ph, DATA96 pack);
            public void OnRANGEFINDERReceive_direct(Channel src, Inside ph, RANGEFINDER pack) {OnRANGEFINDERReceive(this, ph,  pack);}
            public event RANGEFINDERReceiveHandler OnRANGEFINDERReceive;
            public delegate void RANGEFINDERReceiveHandler(Channel src, Inside ph, RANGEFINDER pack);
            public void OnAIRSPEED_AUTOCALReceive_direct(Channel src, Inside ph, AIRSPEED_AUTOCAL pack) {OnAIRSPEED_AUTOCALReceive(this, ph,  pack);}
            public event AIRSPEED_AUTOCALReceiveHandler OnAIRSPEED_AUTOCALReceive;
            public delegate void AIRSPEED_AUTOCALReceiveHandler(Channel src, Inside ph, AIRSPEED_AUTOCAL pack);
            public void OnRALLY_POINTReceive_direct(Channel src, Inside ph, RALLY_POINT pack) {OnRALLY_POINTReceive(this, ph,  pack);}
            public event RALLY_POINTReceiveHandler OnRALLY_POINTReceive;
            public delegate void RALLY_POINTReceiveHandler(Channel src, Inside ph, RALLY_POINT pack);
            public void OnRALLY_FETCH_POINTReceive_direct(Channel src, Inside ph, RALLY_FETCH_POINT pack) {OnRALLY_FETCH_POINTReceive(this, ph,  pack);}
            public event RALLY_FETCH_POINTReceiveHandler OnRALLY_FETCH_POINTReceive;
            public delegate void RALLY_FETCH_POINTReceiveHandler(Channel src, Inside ph, RALLY_FETCH_POINT pack);
            public void OnCOMPASSMOT_STATUSReceive_direct(Channel src, Inside ph, COMPASSMOT_STATUS pack) {OnCOMPASSMOT_STATUSReceive(this, ph,  pack);}
            public event COMPASSMOT_STATUSReceiveHandler OnCOMPASSMOT_STATUSReceive;
            public delegate void COMPASSMOT_STATUSReceiveHandler(Channel src, Inside ph, COMPASSMOT_STATUS pack);
            public void OnAHRS2Receive_direct(Channel src, Inside ph, AHRS2 pack) {OnAHRS2Receive(this, ph,  pack);}
            public event AHRS2ReceiveHandler OnAHRS2Receive;
            public delegate void AHRS2ReceiveHandler(Channel src, Inside ph, AHRS2 pack);
            public void OnCAMERA_STATUSReceive_direct(Channel src, Inside ph, CAMERA_STATUS pack) {OnCAMERA_STATUSReceive(this, ph,  pack);}
            public event CAMERA_STATUSReceiveHandler OnCAMERA_STATUSReceive;
            public delegate void CAMERA_STATUSReceiveHandler(Channel src, Inside ph, CAMERA_STATUS pack);
            public void OnCAMERA_FEEDBACKReceive_direct(Channel src, Inside ph, CAMERA_FEEDBACK pack) {OnCAMERA_FEEDBACKReceive(this, ph,  pack);}
            public event CAMERA_FEEDBACKReceiveHandler OnCAMERA_FEEDBACKReceive;
            public delegate void CAMERA_FEEDBACKReceiveHandler(Channel src, Inside ph, CAMERA_FEEDBACK pack);
            public void OnBATTERY2Receive_direct(Channel src, Inside ph, BATTERY2 pack) {OnBATTERY2Receive(this, ph,  pack);}
            public event BATTERY2ReceiveHandler OnBATTERY2Receive;
            public delegate void BATTERY2ReceiveHandler(Channel src, Inside ph, BATTERY2 pack);
            public void OnAHRS3Receive_direct(Channel src, Inside ph, AHRS3 pack) {OnAHRS3Receive(this, ph,  pack);}
            public event AHRS3ReceiveHandler OnAHRS3Receive;
            public delegate void AHRS3ReceiveHandler(Channel src, Inside ph, AHRS3 pack);
            public void OnAUTOPILOT_VERSION_REQUESTReceive_direct(Channel src, Inside ph, AUTOPILOT_VERSION_REQUEST pack) {OnAUTOPILOT_VERSION_REQUESTReceive(this, ph,  pack);}
            public event AUTOPILOT_VERSION_REQUESTReceiveHandler OnAUTOPILOT_VERSION_REQUESTReceive;
            public delegate void AUTOPILOT_VERSION_REQUESTReceiveHandler(Channel src, Inside ph, AUTOPILOT_VERSION_REQUEST pack);
            public void OnREMOTE_LOG_DATA_BLOCKReceive_direct(Channel src, Inside ph, REMOTE_LOG_DATA_BLOCK pack) {OnREMOTE_LOG_DATA_BLOCKReceive(this, ph,  pack);}
            public event REMOTE_LOG_DATA_BLOCKReceiveHandler OnREMOTE_LOG_DATA_BLOCKReceive;
            public delegate void REMOTE_LOG_DATA_BLOCKReceiveHandler(Channel src, Inside ph, REMOTE_LOG_DATA_BLOCK pack);
            public void OnREMOTE_LOG_BLOCK_STATUSReceive_direct(Channel src, Inside ph, REMOTE_LOG_BLOCK_STATUS pack) {OnREMOTE_LOG_BLOCK_STATUSReceive(this, ph,  pack);}
            public event REMOTE_LOG_BLOCK_STATUSReceiveHandler OnREMOTE_LOG_BLOCK_STATUSReceive;
            public delegate void REMOTE_LOG_BLOCK_STATUSReceiveHandler(Channel src, Inside ph, REMOTE_LOG_BLOCK_STATUS pack);
            public void OnLED_CONTROLReceive_direct(Channel src, Inside ph, LED_CONTROL pack) {OnLED_CONTROLReceive(this, ph,  pack);}
            public event LED_CONTROLReceiveHandler OnLED_CONTROLReceive;
            public delegate void LED_CONTROLReceiveHandler(Channel src, Inside ph, LED_CONTROL pack);
            public void OnMAG_CAL_PROGRESSReceive_direct(Channel src, Inside ph, MAG_CAL_PROGRESS pack) {OnMAG_CAL_PROGRESSReceive(this, ph,  pack);}
            public event MAG_CAL_PROGRESSReceiveHandler OnMAG_CAL_PROGRESSReceive;
            public delegate void MAG_CAL_PROGRESSReceiveHandler(Channel src, Inside ph, MAG_CAL_PROGRESS pack);
            public void OnMAG_CAL_REPORTReceive_direct(Channel src, Inside ph, MAG_CAL_REPORT pack) {OnMAG_CAL_REPORTReceive(this, ph,  pack);}
            public event MAG_CAL_REPORTReceiveHandler OnMAG_CAL_REPORTReceive;
            public delegate void MAG_CAL_REPORTReceiveHandler(Channel src, Inside ph, MAG_CAL_REPORT pack);
            public void OnEKF_STATUS_REPORTReceive_direct(Channel src, Inside ph, EKF_STATUS_REPORT pack) {OnEKF_STATUS_REPORTReceive(this, ph,  pack);}
            public event EKF_STATUS_REPORTReceiveHandler OnEKF_STATUS_REPORTReceive;
            public delegate void EKF_STATUS_REPORTReceiveHandler(Channel src, Inside ph, EKF_STATUS_REPORT pack);
            public void OnPID_TUNINGReceive_direct(Channel src, Inside ph, PID_TUNING pack) {OnPID_TUNINGReceive(this, ph,  pack);}
            public event PID_TUNINGReceiveHandler OnPID_TUNINGReceive;
            public delegate void PID_TUNINGReceiveHandler(Channel src, Inside ph, PID_TUNING pack);
            public void OnGIMBAL_REPORTReceive_direct(Channel src, Inside ph, GIMBAL_REPORT pack) {OnGIMBAL_REPORTReceive(this, ph,  pack);}
            public event GIMBAL_REPORTReceiveHandler OnGIMBAL_REPORTReceive;
            public delegate void GIMBAL_REPORTReceiveHandler(Channel src, Inside ph, GIMBAL_REPORT pack);
            public void OnGIMBAL_CONTROLReceive_direct(Channel src, Inside ph, GIMBAL_CONTROL pack) {OnGIMBAL_CONTROLReceive(this, ph,  pack);}
            public event GIMBAL_CONTROLReceiveHandler OnGIMBAL_CONTROLReceive;
            public delegate void GIMBAL_CONTROLReceiveHandler(Channel src, Inside ph, GIMBAL_CONTROL pack);
            public void OnGIMBAL_TORQUE_CMD_REPORTReceive_direct(Channel src, Inside ph, GIMBAL_TORQUE_CMD_REPORT pack) {OnGIMBAL_TORQUE_CMD_REPORTReceive(this, ph,  pack);}
            public event GIMBAL_TORQUE_CMD_REPORTReceiveHandler OnGIMBAL_TORQUE_CMD_REPORTReceive;
            public delegate void GIMBAL_TORQUE_CMD_REPORTReceiveHandler(Channel src, Inside ph, GIMBAL_TORQUE_CMD_REPORT pack);
            public void OnGOPRO_HEARTBEATReceive_direct(Channel src, Inside ph, GOPRO_HEARTBEAT pack) {OnGOPRO_HEARTBEATReceive(this, ph,  pack);}
            public event GOPRO_HEARTBEATReceiveHandler OnGOPRO_HEARTBEATReceive;
            public delegate void GOPRO_HEARTBEATReceiveHandler(Channel src, Inside ph, GOPRO_HEARTBEAT pack);
            public void OnGOPRO_GET_REQUESTReceive_direct(Channel src, Inside ph, GOPRO_GET_REQUEST pack) {OnGOPRO_GET_REQUESTReceive(this, ph,  pack);}
            public event GOPRO_GET_REQUESTReceiveHandler OnGOPRO_GET_REQUESTReceive;
            public delegate void GOPRO_GET_REQUESTReceiveHandler(Channel src, Inside ph, GOPRO_GET_REQUEST pack);
            public void OnGOPRO_GET_RESPONSEReceive_direct(Channel src, Inside ph, GOPRO_GET_RESPONSE pack) {OnGOPRO_GET_RESPONSEReceive(this, ph,  pack);}
            public event GOPRO_GET_RESPONSEReceiveHandler OnGOPRO_GET_RESPONSEReceive;
            public delegate void GOPRO_GET_RESPONSEReceiveHandler(Channel src, Inside ph, GOPRO_GET_RESPONSE pack);
            public void OnGOPRO_SET_REQUESTReceive_direct(Channel src, Inside ph, GOPRO_SET_REQUEST pack) {OnGOPRO_SET_REQUESTReceive(this, ph,  pack);}
            public event GOPRO_SET_REQUESTReceiveHandler OnGOPRO_SET_REQUESTReceive;
            public delegate void GOPRO_SET_REQUESTReceiveHandler(Channel src, Inside ph, GOPRO_SET_REQUEST pack);
            public void OnGOPRO_SET_RESPONSEReceive_direct(Channel src, Inside ph, GOPRO_SET_RESPONSE pack) {OnGOPRO_SET_RESPONSEReceive(this, ph,  pack);}
            public event GOPRO_SET_RESPONSEReceiveHandler OnGOPRO_SET_RESPONSEReceive;
            public delegate void GOPRO_SET_RESPONSEReceiveHandler(Channel src, Inside ph, GOPRO_SET_RESPONSE pack);
            public void OnRPMReceive_direct(Channel src, Inside ph, RPM pack) {OnRPMReceive(this, ph,  pack);}
            public event RPMReceiveHandler OnRPMReceive;
            public delegate void RPMReceiveHandler(Channel src, Inside ph, RPM pack);
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
            public void OnUAVIONIX_ADSB_OUT_CFGReceive_direct(Channel src, Inside ph, UAVIONIX_ADSB_OUT_CFG pack) {OnUAVIONIX_ADSB_OUT_CFGReceive(this, ph,  pack);}
            public event UAVIONIX_ADSB_OUT_CFGReceiveHandler OnUAVIONIX_ADSB_OUT_CFGReceive;
            public delegate void UAVIONIX_ADSB_OUT_CFGReceiveHandler(Channel src, Inside ph, UAVIONIX_ADSB_OUT_CFG pack);
            public void OnUAVIONIX_ADSB_OUT_DYNAMICReceive_direct(Channel src, Inside ph, UAVIONIX_ADSB_OUT_DYNAMIC pack) {OnUAVIONIX_ADSB_OUT_DYNAMICReceive(this, ph,  pack);}
            public event UAVIONIX_ADSB_OUT_DYNAMICReceiveHandler OnUAVIONIX_ADSB_OUT_DYNAMICReceive;
            public delegate void UAVIONIX_ADSB_OUT_DYNAMICReceiveHandler(Channel src, Inside ph, UAVIONIX_ADSB_OUT_DYNAMIC pack);
            public void OnUAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORTReceive_direct(Channel src, Inside ph, UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT pack) {OnUAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORTReceive(this, ph,  pack);}
            public event UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORTReceiveHandler OnUAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORTReceive;
            public delegate void UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORTReceiveHandler(Channel src, Inside ph, UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT pack);
            public void OnDEVICE_OP_READReceive_direct(Channel src, Inside ph, DEVICE_OP_READ pack) {OnDEVICE_OP_READReceive(this, ph,  pack);}
            public event DEVICE_OP_READReceiveHandler OnDEVICE_OP_READReceive;
            public delegate void DEVICE_OP_READReceiveHandler(Channel src, Inside ph, DEVICE_OP_READ pack);
            public void OnDEVICE_OP_READ_REPLYReceive_direct(Channel src, Inside ph, DEVICE_OP_READ_REPLY pack) {OnDEVICE_OP_READ_REPLYReceive(this, ph,  pack);}
            public event DEVICE_OP_READ_REPLYReceiveHandler OnDEVICE_OP_READ_REPLYReceive;
            public delegate void DEVICE_OP_READ_REPLYReceiveHandler(Channel src, Inside ph, DEVICE_OP_READ_REPLY pack);
            public void OnDEVICE_OP_WRITEReceive_direct(Channel src, Inside ph, DEVICE_OP_WRITE pack) {OnDEVICE_OP_WRITEReceive(this, ph,  pack);}
            public event DEVICE_OP_WRITEReceiveHandler OnDEVICE_OP_WRITEReceive;
            public delegate void DEVICE_OP_WRITEReceiveHandler(Channel src, Inside ph, DEVICE_OP_WRITE pack);
            public void OnDEVICE_OP_WRITE_REPLYReceive_direct(Channel src, Inside ph, DEVICE_OP_WRITE_REPLY pack) {OnDEVICE_OP_WRITE_REPLYReceive(this, ph,  pack);}
            public event DEVICE_OP_WRITE_REPLYReceiveHandler OnDEVICE_OP_WRITE_REPLYReceive;
            public delegate void DEVICE_OP_WRITE_REPLYReceiveHandler(Channel src, Inside ph, DEVICE_OP_WRITE_REPLY pack);
            public void OnADAP_TUNINGReceive_direct(Channel src, Inside ph, ADAP_TUNING pack) {OnADAP_TUNINGReceive(this, ph,  pack);}
            public event ADAP_TUNINGReceiveHandler OnADAP_TUNINGReceive;
            public delegate void ADAP_TUNINGReceiveHandler(Channel src, Inside ph, ADAP_TUNING pack);
            public void OnVISION_POSITION_DELTAReceive_direct(Channel src, Inside ph, VISION_POSITION_DELTA pack) {OnVISION_POSITION_DELTAReceive(this, ph,  pack);}
            public event VISION_POSITION_DELTAReceiveHandler OnVISION_POSITION_DELTAReceive;
            public delegate void VISION_POSITION_DELTAReceiveHandler(Channel src, Inside ph, VISION_POSITION_DELTA pack);

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
                    case 87:
                        if(pack == null) return new POSITION_TARGET_GLOBAL_INT();
                        break;
                    case 89:
                        if(pack == null) return new LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
                        break;
                    case 90:
                        if(pack == null) return new HIL_STATE();
                        break;
                    case 91:
                        if(pack == null) return new HIL_CONTROLS();
                        break;
                    case 92:
                        if(pack == null) return new HIL_RC_INPUTS_RAW();
                        break;
                    case 93:
                        if(pack == null) return new HIL_ACTUATOR_CONTROLS();
                        break;
                    case 100:
                        if(pack == null) return new OPTICAL_FLOW();
                        break;
                    case 101:
                        if(pack == null) return new GLOBAL_VISION_POSITION_ESTIMATE();
                        break;
                    case 102:
                        if(pack == null) return new VISION_POSITION_ESTIMATE();
                        break;
                    case 150:
                        if(pack == null) return new SENSOR_OFFSETS();
                        OnSENSOR_OFFSETSReceive(this, ph, (SENSOR_OFFSETS) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 151:
                        if(pack == null) return new SET_MAG_OFFSETS();
                        OnSET_MAG_OFFSETSReceive(this, ph, (SET_MAG_OFFSETS) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 152:
                        if(pack == null) return new MEMINFO();
                        OnMEMINFOReceive(this, ph, (MEMINFO) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 153:
                        if(pack == null) return new AP_ADC();
                        OnAP_ADCReceive(this, ph, (AP_ADC) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 154:
                        if(pack == null) return new DIGICAM_CONFIGURE();
                        OnDIGICAM_CONFIGUREReceive(this, ph, (DIGICAM_CONFIGURE) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 155:
                        if(pack == null) return new DIGICAM_CONTROL();
                        OnDIGICAM_CONTROLReceive(this, ph, (DIGICAM_CONTROL) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 156:
                        if(pack == null) return new MOUNT_CONFIGURE();
                        OnMOUNT_CONFIGUREReceive(this, ph, (MOUNT_CONFIGURE) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 157:
                        if(pack == null) return new MOUNT_CONTROL();
                        OnMOUNT_CONTROLReceive(this, ph, (MOUNT_CONTROL) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 158:
                        if(pack == null) return new MOUNT_STATUS();
                        OnMOUNT_STATUSReceive(this, ph, (MOUNT_STATUS) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 160:
                        if(pack == null) return new FENCE_POINT();
                        OnFENCE_POINTReceive(this, ph, (FENCE_POINT) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 161:
                        if(pack == null) return new FENCE_FETCH_POINT();
                        OnFENCE_FETCH_POINTReceive(this, ph, (FENCE_FETCH_POINT) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 162:
                        if(pack == null) return new FENCE_STATUS();
                        OnFENCE_STATUSReceive(this, ph, (FENCE_STATUS) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 163:
                        if(pack == null) return new AHRS();
                        OnAHRSReceive(this, ph, (AHRS) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 164:
                        if(pack == null) return new SIMSTATE();
                        OnSIMSTATEReceive(this, ph, (SIMSTATE) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 165:
                        if(pack == null) return new HWSTATUS();
                        OnHWSTATUSReceive(this, ph, (HWSTATUS) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 166:
                        if(pack == null) return new RADIO();
                        OnRADIOReceive(this, ph, (RADIO) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 167:
                        if(pack == null) return new LIMITS_STATUS();
                        OnLIMITS_STATUSReceive(this, ph, (LIMITS_STATUS) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 168:
                        if(pack == null) return new WIND();
                        OnWINDReceive(this, ph, (WIND) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 169:
                        if(pack == null) return new DATA16();
                        OnDATA16Receive(this, ph, (DATA16) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 170:
                        if(pack == null) return new DATA32();
                        OnDATA32Receive(this, ph, (DATA32) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 171:
                        if(pack == null) return new DATA64();
                        OnDATA64Receive(this, ph, (DATA64) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 172:
                        if(pack == null) return new DATA96();
                        OnDATA96Receive(this, ph, (DATA96) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 173:
                        if(pack == null) return new RANGEFINDER();
                        OnRANGEFINDERReceive(this, ph, (RANGEFINDER) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 174:
                        if(pack == null) return new AIRSPEED_AUTOCAL();
                        OnAIRSPEED_AUTOCALReceive(this, ph, (AIRSPEED_AUTOCAL) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 175:
                        if(pack == null) return new RALLY_POINT();
                        OnRALLY_POINTReceive(this, ph, (RALLY_POINT) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 176:
                        if(pack == null) return new RALLY_FETCH_POINT();
                        OnRALLY_FETCH_POINTReceive(this, ph, (RALLY_FETCH_POINT) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 177:
                        if(pack == null) return new COMPASSMOT_STATUS();
                        OnCOMPASSMOT_STATUSReceive(this, ph, (COMPASSMOT_STATUS) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 178:
                        if(pack == null) return new AHRS2();
                        OnAHRS2Receive(this, ph, (AHRS2) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 179:
                        if(pack == null) return new CAMERA_STATUS();
                        OnCAMERA_STATUSReceive(this, ph, (CAMERA_STATUS) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 180:
                        if(pack == null) return new CAMERA_FEEDBACK();
                        OnCAMERA_FEEDBACKReceive(this, ph, (CAMERA_FEEDBACK) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 181:
                        if(pack == null) return new BATTERY2();
                        OnBATTERY2Receive(this, ph, (BATTERY2) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 182:
                        if(pack == null) return new AHRS3();
                        OnAHRS3Receive(this, ph, (AHRS3) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 183:
                        if(pack == null) return new AUTOPILOT_VERSION_REQUEST();
                        OnAUTOPILOT_VERSION_REQUESTReceive(this, ph, (AUTOPILOT_VERSION_REQUEST) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 184:
                        if(pack == null) return new REMOTE_LOG_DATA_BLOCK();
                        OnREMOTE_LOG_DATA_BLOCKReceive(this, ph, (REMOTE_LOG_DATA_BLOCK) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 185:
                        if(pack == null) return new REMOTE_LOG_BLOCK_STATUS();
                        OnREMOTE_LOG_BLOCK_STATUSReceive(this, ph, (REMOTE_LOG_BLOCK_STATUS) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 186:
                        if(pack == null) return new LED_CONTROL();
                        OnLED_CONTROLReceive(this, ph, (LED_CONTROL) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 191:
                        if(pack == null) return new MAG_CAL_PROGRESS();
                        OnMAG_CAL_PROGRESSReceive(this, ph, (MAG_CAL_PROGRESS) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 192:
                        if(pack == null) return new MAG_CAL_REPORT();
                        OnMAG_CAL_REPORTReceive(this, ph, (MAG_CAL_REPORT) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 193:
                        if(pack == null) return new EKF_STATUS_REPORT();
                        OnEKF_STATUS_REPORTReceive(this, ph, (EKF_STATUS_REPORT) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 194:
                        if(pack == null) return new PID_TUNING();
                        OnPID_TUNINGReceive(this, ph, (PID_TUNING) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 200:
                        if(pack == null) return new GIMBAL_REPORT();
                        OnGIMBAL_REPORTReceive(this, ph, (GIMBAL_REPORT) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 201:
                        if(pack == null) return new GIMBAL_CONTROL();
                        OnGIMBAL_CONTROLReceive(this, ph, (GIMBAL_CONTROL) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 214:
                        if(pack == null) return new GIMBAL_TORQUE_CMD_REPORT();
                        OnGIMBAL_TORQUE_CMD_REPORTReceive(this, ph, (GIMBAL_TORQUE_CMD_REPORT) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 215:
                        if(pack == null) return new GOPRO_HEARTBEAT();
                        OnGOPRO_HEARTBEATReceive(this, ph, (GOPRO_HEARTBEAT) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 216:
                        if(pack == null) return new GOPRO_GET_REQUEST();
                        OnGOPRO_GET_REQUESTReceive(this, ph, (GOPRO_GET_REQUEST) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 217:
                        if(pack == null) return new GOPRO_GET_RESPONSE();
                        OnGOPRO_GET_RESPONSEReceive(this, ph, (GOPRO_GET_RESPONSE) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 218:
                        if(pack == null) return new GOPRO_SET_REQUEST();
                        OnGOPRO_SET_REQUESTReceive(this, ph, (GOPRO_SET_REQUEST) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 219:
                        if(pack == null) return new GOPRO_SET_RESPONSE();
                        OnGOPRO_SET_RESPONSEReceive(this, ph, (GOPRO_SET_RESPONSE) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 226:
                        if(pack == null) return new RPM();
                        OnRPMReceive(this, ph, (RPM) pack);//no any host channels can receive this pack. Handle it with test channel handler
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
                    case 10001:
                        if(pack == null) return new UAVIONIX_ADSB_OUT_CFG();
                        OnUAVIONIX_ADSB_OUT_CFGReceive(this, ph, (UAVIONIX_ADSB_OUT_CFG) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 10002:
                        if(pack == null) return new UAVIONIX_ADSB_OUT_DYNAMIC();
                        OnUAVIONIX_ADSB_OUT_DYNAMICReceive(this, ph, (UAVIONIX_ADSB_OUT_DYNAMIC) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 10003:
                        if(pack == null) return new UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT();
                        OnUAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORTReceive(this, ph, (UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 11000:
                        if(pack == null) return new DEVICE_OP_READ();
                        OnDEVICE_OP_READReceive(this, ph, (DEVICE_OP_READ) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 11001:
                        if(pack == null) return new DEVICE_OP_READ_REPLY();
                        OnDEVICE_OP_READ_REPLYReceive(this, ph, (DEVICE_OP_READ_REPLY) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 11002:
                        if(pack == null) return new DEVICE_OP_WRITE();
                        OnDEVICE_OP_WRITEReceive(this, ph, (DEVICE_OP_WRITE) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 11003:
                        if(pack == null) return new DEVICE_OP_WRITE_REPLY();
                        OnDEVICE_OP_WRITE_REPLYReceive(this, ph, (DEVICE_OP_WRITE_REPLY) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 11010:
                        if(pack == null) return new ADAP_TUNING();
                        OnADAP_TUNINGReceive(this, ph, (ADAP_TUNING) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 11011:
                        if(pack == null) return new VISION_POSITION_DELTA();
                        OnVISION_POSITION_DELTAReceive(this, ph, (VISION_POSITION_DELTA) pack);//no any host channels can receive this pack. Handle it with test channel handler
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
                Debug.Assert(pack.mavlink_version == (byte)(byte)48);
                Debug.Assert(pack.system_status == MAV_STATE.MAV_STATE_UNINIT);
                Debug.Assert(pack.base_mode == (MAV_MODE_FLAG.MAV_MODE_FLAG_HIL_ENABLED));
                Debug.Assert(pack.custom_mode == (uint)2213815428U);
                Debug.Assert(pack.autopilot == MAV_AUTOPILOT.MAV_AUTOPILOT_SLUGS);
                Debug.Assert(pack.type == MAV_TYPE.MAV_TYPE_OCTOROTOR);
            };
            HEARTBEAT p0 = new HEARTBEAT();
            PH.setPack(p0);
            p0.custom_mode = (uint)2213815428U;
            p0.type = MAV_TYPE.MAV_TYPE_OCTOROTOR;
            p0.mavlink_version = (byte)(byte)48;
            p0.system_status = MAV_STATE.MAV_STATE_UNINIT;
            p0.autopilot = MAV_AUTOPILOT.MAV_AUTOPILOT_SLUGS;
            p0.base_mode = (MAV_MODE_FLAG.MAV_MODE_FLAG_HIL_ENABLED);
            SMP_TEST_CH.send(p0);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSYS_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.voltage_battery == (ushort)(ushort)38454);
                Debug.Assert(pack.errors_comm == (ushort)(ushort)662);
                Debug.Assert(pack.errors_count4 == (ushort)(ushort)46520);
                Debug.Assert(pack.battery_remaining == (sbyte)(sbyte)127);
                Debug.Assert(pack.load == (ushort)(ushort)26498);
                Debug.Assert(pack.errors_count3 == (ushort)(ushort)64356);
                Debug.Assert(pack.onboard_control_sensors_present == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO2 |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_GPS |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_AHRS |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_LOGGING |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE));
                Debug.Assert(pack.errors_count1 == (ushort)(ushort)61904);
                Debug.Assert(pack.drop_rate_comm == (ushort)(ushort)14932);
                Debug.Assert(pack.onboard_control_sensors_health == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO));
                Debug.Assert(pack.onboard_control_sensors_enabled == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_AHRS |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL));
                Debug.Assert(pack.errors_count2 == (ushort)(ushort)5456);
                Debug.Assert(pack.current_battery == (short)(short)20557);
            };
            SYS_STATUS p1 = new SYS_STATUS();
            PH.setPack(p1);
            p1.errors_count4 = (ushort)(ushort)46520;
            p1.current_battery = (short)(short)20557;
            p1.battery_remaining = (sbyte)(sbyte)127;
            p1.voltage_battery = (ushort)(ushort)38454;
            p1.onboard_control_sensors_present = (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO2 |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_GPS |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_AHRS |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_LOGGING |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE);
            p1.onboard_control_sensors_health = (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO);
            p1.errors_comm = (ushort)(ushort)662;
            p1.load = (ushort)(ushort)26498;
            p1.drop_rate_comm = (ushort)(ushort)14932;
            p1.onboard_control_sensors_enabled = (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_AHRS |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL);
            p1.errors_count1 = (ushort)(ushort)61904;
            p1.errors_count3 = (ushort)(ushort)64356;
            p1.errors_count2 = (ushort)(ushort)5456;
            SMP_TEST_CH.send(p1);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSYSTEM_TIMEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_unix_usec == (ulong)4789885133111323800L);
                Debug.Assert(pack.time_boot_ms == (uint)288729695U);
            };
            SYSTEM_TIME p2 = new SYSTEM_TIME();
            PH.setPack(p2);
            p2.time_boot_ms = (uint)288729695U;
            p2.time_unix_usec = (ulong)4789885133111323800L;
            SMP_TEST_CH.send(p2);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPOSITION_TARGET_LOCAL_NEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vx == (float) -9.852065E37F);
                Debug.Assert(pack.vz == (float) -1.4477404E38F);
                Debug.Assert(pack.x == (float) -9.832899E37F);
                Debug.Assert(pack.time_boot_ms == (uint)3969123871U);
                Debug.Assert(pack.vy == (float) -1.3806221E38F);
                Debug.Assert(pack.coordinate_frame == MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT);
                Debug.Assert(pack.afz == (float)1.6901702E38F);
                Debug.Assert(pack.z == (float)1.8765143E38F);
                Debug.Assert(pack.yaw == (float) -9.419351E37F);
                Debug.Assert(pack.type_mask == (ushort)(ushort)49774);
                Debug.Assert(pack.afx == (float) -8.153076E37F);
                Debug.Assert(pack.y == (float) -5.957179E37F);
                Debug.Assert(pack.afy == (float) -2.5960809E38F);
                Debug.Assert(pack.yaw_rate == (float)1.885792E38F);
            };
            POSITION_TARGET_LOCAL_NED p3 = new POSITION_TARGET_LOCAL_NED();
            PH.setPack(p3);
            p3.z = (float)1.8765143E38F;
            p3.yaw = (float) -9.419351E37F;
            p3.time_boot_ms = (uint)3969123871U;
            p3.afz = (float)1.6901702E38F;
            p3.vz = (float) -1.4477404E38F;
            p3.coordinate_frame = MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;
            p3.vx = (float) -9.852065E37F;
            p3.afy = (float) -2.5960809E38F;
            p3.vy = (float) -1.3806221E38F;
            p3.y = (float) -5.957179E37F;
            p3.type_mask = (ushort)(ushort)49774;
            p3.afx = (float) -8.153076E37F;
            p3.yaw_rate = (float)1.885792E38F;
            p3.x = (float) -9.832899E37F;
            SMP_TEST_CH.send(p3);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPINGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)173);
                Debug.Assert(pack.target_system == (byte)(byte)2);
                Debug.Assert(pack.time_usec == (ulong)3882410326389962802L);
                Debug.Assert(pack.seq == (uint)476455018U);
            };
            PING p4 = new PING();
            PH.setPack(p4);
            p4.seq = (uint)476455018U;
            p4.target_system = (byte)(byte)2;
            p4.time_usec = (ulong)3882410326389962802L;
            p4.target_component = (byte)(byte)173;
            SMP_TEST_CH.send(p4);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCHANGE_OPERATOR_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.passkey_LEN(ph) == 15);
                Debug.Assert(pack.passkey_TRY(ph).Equals("dlvorpqoqbjgOub"));
                Debug.Assert(pack.control_request == (byte)(byte)242);
                Debug.Assert(pack.target_system == (byte)(byte)255);
                Debug.Assert(pack.version == (byte)(byte)163);
            };
            CHANGE_OPERATOR_CONTROL p5 = new CHANGE_OPERATOR_CONTROL();
            PH.setPack(p5);
            p5.target_system = (byte)(byte)255;
            p5.control_request = (byte)(byte)242;
            p5.passkey_SET("dlvorpqoqbjgOub", PH) ;
            p5.version = (byte)(byte)163;
            SMP_TEST_CH.send(p5);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCHANGE_OPERATOR_CONTROL_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.gcs_system_id == (byte)(byte)56);
                Debug.Assert(pack.ack == (byte)(byte)59);
                Debug.Assert(pack.control_request == (byte)(byte)140);
            };
            CHANGE_OPERATOR_CONTROL_ACK p6 = new CHANGE_OPERATOR_CONTROL_ACK();
            PH.setPack(p6);
            p6.ack = (byte)(byte)59;
            p6.gcs_system_id = (byte)(byte)56;
            p6.control_request = (byte)(byte)140;
            SMP_TEST_CH.send(p6);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnAUTH_KEYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.key_LEN(ph) == 16);
                Debug.Assert(pack.key_TRY(ph).Equals("mNnganCRjNaudlnp"));
            };
            AUTH_KEY p7 = new AUTH_KEY();
            PH.setPack(p7);
            p7.key_SET("mNnganCRjNaudlnp", PH) ;
            SMP_TEST_CH.send(p7);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_MODEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.custom_mode == (uint)4252766456U);
                Debug.Assert(pack.base_mode == MAV_MODE.MAV_MODE_MANUAL_DISARMED);
                Debug.Assert(pack.target_system == (byte)(byte)236);
            };
            SET_MODE p11 = new SET_MODE();
            PH.setPack(p11);
            p11.target_system = (byte)(byte)236;
            p11.base_mode = MAV_MODE.MAV_MODE_MANUAL_DISARMED;
            p11.custom_mode = (uint)4252766456U;
            SMP_TEST_CH.send(p11);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_REQUEST_READReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_id_LEN(ph) == 7);
                Debug.Assert(pack.param_id_TRY(ph).Equals("ooduvsx"));
                Debug.Assert(pack.param_index == (short)(short) -22497);
                Debug.Assert(pack.target_system == (byte)(byte)227);
                Debug.Assert(pack.target_component == (byte)(byte)237);
            };
            PARAM_REQUEST_READ p20 = new PARAM_REQUEST_READ();
            PH.setPack(p20);
            p20.param_id_SET("ooduvsx", PH) ;
            p20.param_index = (short)(short) -22497;
            p20.target_component = (byte)(byte)237;
            p20.target_system = (byte)(byte)227;
            SMP_TEST_CH.send(p20);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)161);
                Debug.Assert(pack.target_system == (byte)(byte)29);
            };
            PARAM_REQUEST_LIST p21 = new PARAM_REQUEST_LIST();
            PH.setPack(p21);
            p21.target_component = (byte)(byte)161;
            p21.target_system = (byte)(byte)29;
            SMP_TEST_CH.send(p21);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_VALUEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_id_LEN(ph) == 5);
                Debug.Assert(pack.param_id_TRY(ph).Equals("fnqwz"));
                Debug.Assert(pack.param_index == (ushort)(ushort)78);
                Debug.Assert(pack.param_value == (float) -2.9370398E38F);
                Debug.Assert(pack.param_type == MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT64);
                Debug.Assert(pack.param_count == (ushort)(ushort)12380);
            };
            PARAM_VALUE p22 = new PARAM_VALUE();
            PH.setPack(p22);
            p22.param_index = (ushort)(ushort)78;
            p22.param_count = (ushort)(ushort)12380;
            p22.param_type = MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT64;
            p22.param_id_SET("fnqwz", PH) ;
            p22.param_value = (float) -2.9370398E38F;
            SMP_TEST_CH.send(p22);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_SETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_type == MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT8);
                Debug.Assert(pack.target_system == (byte)(byte)115);
                Debug.Assert(pack.target_component == (byte)(byte)97);
                Debug.Assert(pack.param_id_LEN(ph) == 14);
                Debug.Assert(pack.param_id_TRY(ph).Equals("izikKztjcnLhse"));
                Debug.Assert(pack.param_value == (float)3.0669707E38F);
            };
            PARAM_SET p23 = new PARAM_SET();
            PH.setPack(p23);
            p23.target_system = (byte)(byte)115;
            p23.param_type = MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT8;
            p23.param_id_SET("izikKztjcnLhse", PH) ;
            p23.target_component = (byte)(byte)97;
            p23.param_value = (float)3.0669707E38F;
            SMP_TEST_CH.send(p23);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_RAW_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vel == (ushort)(ushort)3409);
                Debug.Assert(pack.cog == (ushort)(ushort)44970);
                Debug.Assert(pack.lon == (int) -2089496397);
                Debug.Assert(pack.v_acc_TRY(ph) == (uint)683446350U);
                Debug.Assert(pack.alt_ellipsoid_TRY(ph) == (int) -1076916395);
                Debug.Assert(pack.alt == (int)1036217372);
                Debug.Assert(pack.vel_acc_TRY(ph) == (uint)1357863935U);
                Debug.Assert(pack.time_usec == (ulong)4980144712112240206L);
                Debug.Assert(pack.fix_type == GPS_FIX_TYPE.GPS_FIX_TYPE_NO_FIX);
                Debug.Assert(pack.epv == (ushort)(ushort)61180);
                Debug.Assert(pack.hdg_acc_TRY(ph) == (uint)674971560U);
                Debug.Assert(pack.h_acc_TRY(ph) == (uint)1522683454U);
                Debug.Assert(pack.lat == (int) -953009244);
                Debug.Assert(pack.satellites_visible == (byte)(byte)220);
                Debug.Assert(pack.eph == (ushort)(ushort)9328);
            };
            GPS_RAW_INT p24 = new GPS_RAW_INT();
            PH.setPack(p24);
            p24.fix_type = GPS_FIX_TYPE.GPS_FIX_TYPE_NO_FIX;
            p24.satellites_visible = (byte)(byte)220;
            p24.h_acc_SET((uint)1522683454U, PH) ;
            p24.vel = (ushort)(ushort)3409;
            p24.lat = (int) -953009244;
            p24.v_acc_SET((uint)683446350U, PH) ;
            p24.alt_ellipsoid_SET((int) -1076916395, PH) ;
            p24.cog = (ushort)(ushort)44970;
            p24.eph = (ushort)(ushort)9328;
            p24.hdg_acc_SET((uint)674971560U, PH) ;
            p24.epv = (ushort)(ushort)61180;
            p24.time_usec = (ulong)4980144712112240206L;
            p24.lon = (int) -2089496397;
            p24.vel_acc_SET((uint)1357863935U, PH) ;
            p24.alt = (int)1036217372;
            SMP_TEST_CH.send(p24);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.satellite_snr.SequenceEqual(new byte[] {(byte)239, (byte)70, (byte)167, (byte)168, (byte)94, (byte)34, (byte)142, (byte)26, (byte)155, (byte)252, (byte)67, (byte)211, (byte)111, (byte)123, (byte)161, (byte)159, (byte)28, (byte)227, (byte)80, (byte)158}));
                Debug.Assert(pack.satellite_azimuth.SequenceEqual(new byte[] {(byte)30, (byte)156, (byte)16, (byte)158, (byte)211, (byte)196, (byte)218, (byte)217, (byte)171, (byte)46, (byte)98, (byte)226, (byte)21, (byte)190, (byte)55, (byte)108, (byte)130, (byte)112, (byte)184, (byte)134}));
                Debug.Assert(pack.satellite_elevation.SequenceEqual(new byte[] {(byte)217, (byte)240, (byte)103, (byte)220, (byte)157, (byte)113, (byte)99, (byte)214, (byte)220, (byte)211, (byte)157, (byte)116, (byte)97, (byte)172, (byte)86, (byte)199, (byte)237, (byte)230, (byte)12, (byte)222}));
                Debug.Assert(pack.satellites_visible == (byte)(byte)25);
                Debug.Assert(pack.satellite_prn.SequenceEqual(new byte[] {(byte)160, (byte)190, (byte)145, (byte)98, (byte)52, (byte)116, (byte)72, (byte)11, (byte)138, (byte)252, (byte)239, (byte)207, (byte)237, (byte)159, (byte)45, (byte)174, (byte)157, (byte)14, (byte)205, (byte)149}));
                Debug.Assert(pack.satellite_used.SequenceEqual(new byte[] {(byte)174, (byte)24, (byte)89, (byte)77, (byte)117, (byte)15, (byte)77, (byte)136, (byte)111, (byte)18, (byte)243, (byte)31, (byte)1, (byte)250, (byte)64, (byte)148, (byte)149, (byte)81, (byte)72, (byte)59}));
            };
            GPS_STATUS p25 = new GPS_STATUS();
            PH.setPack(p25);
            p25.satellite_snr_SET(new byte[] {(byte)239, (byte)70, (byte)167, (byte)168, (byte)94, (byte)34, (byte)142, (byte)26, (byte)155, (byte)252, (byte)67, (byte)211, (byte)111, (byte)123, (byte)161, (byte)159, (byte)28, (byte)227, (byte)80, (byte)158}, 0) ;
            p25.satellite_prn_SET(new byte[] {(byte)160, (byte)190, (byte)145, (byte)98, (byte)52, (byte)116, (byte)72, (byte)11, (byte)138, (byte)252, (byte)239, (byte)207, (byte)237, (byte)159, (byte)45, (byte)174, (byte)157, (byte)14, (byte)205, (byte)149}, 0) ;
            p25.satellite_used_SET(new byte[] {(byte)174, (byte)24, (byte)89, (byte)77, (byte)117, (byte)15, (byte)77, (byte)136, (byte)111, (byte)18, (byte)243, (byte)31, (byte)1, (byte)250, (byte)64, (byte)148, (byte)149, (byte)81, (byte)72, (byte)59}, 0) ;
            p25.satellite_azimuth_SET(new byte[] {(byte)30, (byte)156, (byte)16, (byte)158, (byte)211, (byte)196, (byte)218, (byte)217, (byte)171, (byte)46, (byte)98, (byte)226, (byte)21, (byte)190, (byte)55, (byte)108, (byte)130, (byte)112, (byte)184, (byte)134}, 0) ;
            p25.satellite_elevation_SET(new byte[] {(byte)217, (byte)240, (byte)103, (byte)220, (byte)157, (byte)113, (byte)99, (byte)214, (byte)220, (byte)211, (byte)157, (byte)116, (byte)97, (byte)172, (byte)86, (byte)199, (byte)237, (byte)230, (byte)12, (byte)222}, 0) ;
            p25.satellites_visible = (byte)(byte)25;
            SMP_TEST_CH.send(p25);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_IMUReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.xmag == (short)(short)16177);
                Debug.Assert(pack.zmag == (short)(short) -15849);
                Debug.Assert(pack.ygyro == (short)(short)17002);
                Debug.Assert(pack.xacc == (short)(short)22393);
                Debug.Assert(pack.zacc == (short)(short)11302);
                Debug.Assert(pack.xgyro == (short)(short)3529);
                Debug.Assert(pack.zgyro == (short)(short)26175);
                Debug.Assert(pack.ymag == (short)(short)6598);
                Debug.Assert(pack.yacc == (short)(short)8572);
                Debug.Assert(pack.time_boot_ms == (uint)292027180U);
            };
            SCALED_IMU p26 = new SCALED_IMU();
            PH.setPack(p26);
            p26.time_boot_ms = (uint)292027180U;
            p26.zacc = (short)(short)11302;
            p26.zgyro = (short)(short)26175;
            p26.xmag = (short)(short)16177;
            p26.ygyro = (short)(short)17002;
            p26.zmag = (short)(short) -15849;
            p26.xacc = (short)(short)22393;
            p26.xgyro = (short)(short)3529;
            p26.yacc = (short)(short)8572;
            p26.ymag = (short)(short)6598;
            SMP_TEST_CH.send(p26);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRAW_IMUReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.yacc == (short)(short) -20454);
                Debug.Assert(pack.xmag == (short)(short) -13328);
                Debug.Assert(pack.ymag == (short)(short) -10487);
                Debug.Assert(pack.ygyro == (short)(short)31772);
                Debug.Assert(pack.xacc == (short)(short) -5327);
                Debug.Assert(pack.time_usec == (ulong)4436171244275513697L);
                Debug.Assert(pack.zmag == (short)(short)17291);
                Debug.Assert(pack.xgyro == (short)(short) -14933);
                Debug.Assert(pack.zacc == (short)(short) -18506);
                Debug.Assert(pack.zgyro == (short)(short) -13148);
            };
            RAW_IMU p27 = new RAW_IMU();
            PH.setPack(p27);
            p27.ymag = (short)(short) -10487;
            p27.xacc = (short)(short) -5327;
            p27.zacc = (short)(short) -18506;
            p27.zmag = (short)(short)17291;
            p27.zgyro = (short)(short) -13148;
            p27.yacc = (short)(short) -20454;
            p27.xgyro = (short)(short) -14933;
            p27.ygyro = (short)(short)31772;
            p27.time_usec = (ulong)4436171244275513697L;
            p27.xmag = (short)(short) -13328;
            SMP_TEST_CH.send(p27);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRAW_PRESSUREReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.temperature == (short)(short) -13402);
                Debug.Assert(pack.press_abs == (short)(short) -11823);
                Debug.Assert(pack.press_diff1 == (short)(short)13278);
                Debug.Assert(pack.press_diff2 == (short)(short) -10915);
                Debug.Assert(pack.time_usec == (ulong)7870999345198745871L);
            };
            RAW_PRESSURE p28 = new RAW_PRESSURE();
            PH.setPack(p28);
            p28.temperature = (short)(short) -13402;
            p28.press_diff2 = (short)(short) -10915;
            p28.press_abs = (short)(short) -11823;
            p28.time_usec = (ulong)7870999345198745871L;
            p28.press_diff1 = (short)(short)13278;
            SMP_TEST_CH.send(p28);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_PRESSUREReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.press_abs == (float) -8.3145396E37F);
                Debug.Assert(pack.press_diff == (float)2.4047383E38F);
                Debug.Assert(pack.time_boot_ms == (uint)1596060065U);
                Debug.Assert(pack.temperature == (short)(short) -14635);
            };
            SCALED_PRESSURE p29 = new SCALED_PRESSURE();
            PH.setPack(p29);
            p29.time_boot_ms = (uint)1596060065U;
            p29.press_diff = (float)2.4047383E38F;
            p29.temperature = (short)(short) -14635;
            p29.press_abs = (float) -8.3145396E37F;
            SMP_TEST_CH.send(p29);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATTITUDEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)3595066264U);
                Debug.Assert(pack.rollspeed == (float)7.28532E35F);
                Debug.Assert(pack.pitchspeed == (float)1.2621914E38F);
                Debug.Assert(pack.roll == (float) -4.6754204E37F);
                Debug.Assert(pack.yaw == (float)2.1219396E37F);
                Debug.Assert(pack.yawspeed == (float)3.8563757E36F);
                Debug.Assert(pack.pitch == (float)2.9875928E38F);
            };
            ATTITUDE p30 = new ATTITUDE();
            PH.setPack(p30);
            p30.time_boot_ms = (uint)3595066264U;
            p30.rollspeed = (float)7.28532E35F;
            p30.pitchspeed = (float)1.2621914E38F;
            p30.pitch = (float)2.9875928E38F;
            p30.yaw = (float)2.1219396E37F;
            p30.yawspeed = (float)3.8563757E36F;
            p30.roll = (float) -4.6754204E37F;
            SMP_TEST_CH.send(p30);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATTITUDE_QUATERNIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.q3 == (float) -1.978946E38F);
                Debug.Assert(pack.q4 == (float)6.19508E37F);
                Debug.Assert(pack.q2 == (float) -1.8302337E38F);
                Debug.Assert(pack.time_boot_ms == (uint)1969899966U);
                Debug.Assert(pack.rollspeed == (float) -3.365121E38F);
                Debug.Assert(pack.yawspeed == (float) -3.0514822E38F);
                Debug.Assert(pack.pitchspeed == (float)3.282737E38F);
                Debug.Assert(pack.q1 == (float)2.3447936E38F);
            };
            ATTITUDE_QUATERNION p31 = new ATTITUDE_QUATERNION();
            PH.setPack(p31);
            p31.q4 = (float)6.19508E37F;
            p31.time_boot_ms = (uint)1969899966U;
            p31.pitchspeed = (float)3.282737E38F;
            p31.q1 = (float)2.3447936E38F;
            p31.q2 = (float) -1.8302337E38F;
            p31.yawspeed = (float) -3.0514822E38F;
            p31.rollspeed = (float) -3.365121E38F;
            p31.q3 = (float) -1.978946E38F;
            SMP_TEST_CH.send(p31);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOCAL_POSITION_NEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.z == (float)9.095481E36F);
                Debug.Assert(pack.vz == (float) -2.9473834E38F);
                Debug.Assert(pack.vy == (float) -4.3198277E37F);
                Debug.Assert(pack.y == (float)2.2567385E38F);
                Debug.Assert(pack.time_boot_ms == (uint)3425128957U);
                Debug.Assert(pack.x == (float) -1.4375188E38F);
                Debug.Assert(pack.vx == (float)4.6074876E37F);
            };
            LOCAL_POSITION_NED p32 = new LOCAL_POSITION_NED();
            PH.setPack(p32);
            p32.vz = (float) -2.9473834E38F;
            p32.vx = (float)4.6074876E37F;
            p32.time_boot_ms = (uint)3425128957U;
            p32.z = (float)9.095481E36F;
            p32.y = (float)2.2567385E38F;
            p32.x = (float) -1.4375188E38F;
            p32.vy = (float) -4.3198277E37F;
            SMP_TEST_CH.send(p32);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGLOBAL_POSITION_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.relative_alt == (int)106466507);
                Debug.Assert(pack.time_boot_ms == (uint)1219580306U);
                Debug.Assert(pack.vz == (short)(short)11481);
                Debug.Assert(pack.vy == (short)(short)948);
                Debug.Assert(pack.hdg == (ushort)(ushort)8943);
                Debug.Assert(pack.alt == (int)291703061);
                Debug.Assert(pack.vx == (short)(short) -22256);
                Debug.Assert(pack.lat == (int) -1772696945);
                Debug.Assert(pack.lon == (int)2053063101);
            };
            GLOBAL_POSITION_INT p33 = new GLOBAL_POSITION_INT();
            PH.setPack(p33);
            p33.vz = (short)(short)11481;
            p33.alt = (int)291703061;
            p33.lon = (int)2053063101;
            p33.hdg = (ushort)(ushort)8943;
            p33.relative_alt = (int)106466507;
            p33.vx = (short)(short) -22256;
            p33.vy = (short)(short)948;
            p33.lat = (int) -1772696945;
            p33.time_boot_ms = (uint)1219580306U;
            SMP_TEST_CH.send(p33);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRC_CHANNELS_SCALEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan4_scaled == (short)(short) -22174);
                Debug.Assert(pack.chan6_scaled == (short)(short)22985);
                Debug.Assert(pack.rssi == (byte)(byte)203);
                Debug.Assert(pack.chan3_scaled == (short)(short)24503);
                Debug.Assert(pack.port == (byte)(byte)129);
                Debug.Assert(pack.chan7_scaled == (short)(short) -21489);
                Debug.Assert(pack.time_boot_ms == (uint)636191764U);
                Debug.Assert(pack.chan1_scaled == (short)(short) -23936);
                Debug.Assert(pack.chan2_scaled == (short)(short) -30954);
                Debug.Assert(pack.chan5_scaled == (short)(short) -2089);
                Debug.Assert(pack.chan8_scaled == (short)(short)11786);
            };
            RC_CHANNELS_SCALED p34 = new RC_CHANNELS_SCALED();
            PH.setPack(p34);
            p34.chan7_scaled = (short)(short) -21489;
            p34.rssi = (byte)(byte)203;
            p34.time_boot_ms = (uint)636191764U;
            p34.chan6_scaled = (short)(short)22985;
            p34.chan3_scaled = (short)(short)24503;
            p34.chan4_scaled = (short)(short) -22174;
            p34.chan8_scaled = (short)(short)11786;
            p34.chan2_scaled = (short)(short) -30954;
            p34.chan1_scaled = (short)(short) -23936;
            p34.chan5_scaled = (short)(short) -2089;
            p34.port = (byte)(byte)129;
            SMP_TEST_CH.send(p34);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRC_CHANNELS_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)18883);
                Debug.Assert(pack.port == (byte)(byte)236);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)6417);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)53120);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)22362);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)18780);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)62187);
                Debug.Assert(pack.time_boot_ms == (uint)2744549251U);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)4904);
                Debug.Assert(pack.rssi == (byte)(byte)124);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)12262);
            };
            RC_CHANNELS_RAW p35 = new RC_CHANNELS_RAW();
            PH.setPack(p35);
            p35.time_boot_ms = (uint)2744549251U;
            p35.chan6_raw = (ushort)(ushort)18883;
            p35.rssi = (byte)(byte)124;
            p35.chan3_raw = (ushort)(ushort)18780;
            p35.chan8_raw = (ushort)(ushort)12262;
            p35.chan4_raw = (ushort)(ushort)53120;
            p35.chan2_raw = (ushort)(ushort)6417;
            p35.chan1_raw = (ushort)(ushort)4904;
            p35.port = (byte)(byte)236;
            p35.chan7_raw = (ushort)(ushort)22362;
            p35.chan5_raw = (ushort)(ushort)62187;
            SMP_TEST_CH.send(p35);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSERVO_OUTPUT_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.servo15_raw_TRY(ph) == (ushort)(ushort)45025);
                Debug.Assert(pack.servo5_raw == (ushort)(ushort)11969);
                Debug.Assert(pack.port == (byte)(byte)44);
                Debug.Assert(pack.servo9_raw_TRY(ph) == (ushort)(ushort)19581);
                Debug.Assert(pack.servo3_raw == (ushort)(ushort)60134);
                Debug.Assert(pack.servo13_raw_TRY(ph) == (ushort)(ushort)534);
                Debug.Assert(pack.servo11_raw_TRY(ph) == (ushort)(ushort)28845);
                Debug.Assert(pack.servo1_raw == (ushort)(ushort)61087);
                Debug.Assert(pack.servo6_raw == (ushort)(ushort)35351);
                Debug.Assert(pack.time_usec == (uint)4031505731U);
                Debug.Assert(pack.servo10_raw_TRY(ph) == (ushort)(ushort)7331);
                Debug.Assert(pack.servo7_raw == (ushort)(ushort)56077);
                Debug.Assert(pack.servo14_raw_TRY(ph) == (ushort)(ushort)34846);
                Debug.Assert(pack.servo2_raw == (ushort)(ushort)33294);
                Debug.Assert(pack.servo16_raw_TRY(ph) == (ushort)(ushort)22650);
                Debug.Assert(pack.servo8_raw == (ushort)(ushort)35190);
                Debug.Assert(pack.servo4_raw == (ushort)(ushort)11792);
                Debug.Assert(pack.servo12_raw_TRY(ph) == (ushort)(ushort)17384);
            };
            SERVO_OUTPUT_RAW p36 = new SERVO_OUTPUT_RAW();
            PH.setPack(p36);
            p36.servo2_raw = (ushort)(ushort)33294;
            p36.servo1_raw = (ushort)(ushort)61087;
            p36.servo11_raw_SET((ushort)(ushort)28845, PH) ;
            p36.port = (byte)(byte)44;
            p36.servo3_raw = (ushort)(ushort)60134;
            p36.time_usec = (uint)4031505731U;
            p36.servo9_raw_SET((ushort)(ushort)19581, PH) ;
            p36.servo12_raw_SET((ushort)(ushort)17384, PH) ;
            p36.servo16_raw_SET((ushort)(ushort)22650, PH) ;
            p36.servo5_raw = (ushort)(ushort)11969;
            p36.servo4_raw = (ushort)(ushort)11792;
            p36.servo14_raw_SET((ushort)(ushort)34846, PH) ;
            p36.servo8_raw = (ushort)(ushort)35190;
            p36.servo7_raw = (ushort)(ushort)56077;
            p36.servo15_raw_SET((ushort)(ushort)45025, PH) ;
            p36.servo6_raw = (ushort)(ushort)35351;
            p36.servo13_raw_SET((ushort)(ushort)534, PH) ;
            p36.servo10_raw_SET((ushort)(ushort)7331, PH) ;
            SMP_TEST_CH.send(p36);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_REQUEST_PARTIAL_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.start_index == (short)(short)22691);
                Debug.Assert(pack.end_index == (short)(short)9228);
                Debug.Assert(pack.target_system == (byte)(byte)240);
                Debug.Assert(pack.target_component == (byte)(byte)214);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
            };
            MISSION_REQUEST_PARTIAL_LIST p37 = new MISSION_REQUEST_PARTIAL_LIST();
            PH.setPack(p37);
            p37.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE;
            p37.target_system = (byte)(byte)240;
            p37.start_index = (short)(short)22691;
            p37.target_component = (byte)(byte)214;
            p37.end_index = (short)(short)9228;
            SMP_TEST_CH.send(p37);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_WRITE_PARTIAL_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
                Debug.Assert(pack.target_system == (byte)(byte)135);
                Debug.Assert(pack.end_index == (short)(short)29351);
                Debug.Assert(pack.start_index == (short)(short) -21522);
                Debug.Assert(pack.target_component == (byte)(byte)6);
            };
            MISSION_WRITE_PARTIAL_LIST p38 = new MISSION_WRITE_PARTIAL_LIST();
            PH.setPack(p38);
            p38.target_component = (byte)(byte)6;
            p38.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY;
            p38.end_index = (short)(short)29351;
            p38.target_system = (byte)(byte)135;
            p38.start_index = (short)(short) -21522;
            SMP_TEST_CH.send(p38);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_ITEMReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param3 == (float) -5.335969E36F);
                Debug.Assert(pack.target_component == (byte)(byte)95);
                Debug.Assert(pack.param1 == (float) -1.0436125E38F);
                Debug.Assert(pack.param4 == (float)3.035903E37F);
                Debug.Assert(pack.y == (float)2.056701E38F);
                Debug.Assert(pack.target_system == (byte)(byte)196);
                Debug.Assert(pack.command == MAV_CMD.MAV_CMD_DO_SET_SERVO);
                Debug.Assert(pack.z == (float)2.527882E38F);
                Debug.Assert(pack.current == (byte)(byte)134);
                Debug.Assert(pack.param2 == (float)9.877955E37F);
                Debug.Assert(pack.seq == (ushort)(ushort)62590);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
                Debug.Assert(pack.autocontinue == (byte)(byte)117);
                Debug.Assert(pack.frame == MAV_FRAME.MAV_FRAME_GLOBAL_INT);
                Debug.Assert(pack.x == (float)2.6512892E38F);
            };
            MISSION_ITEM p39 = new MISSION_ITEM();
            PH.setPack(p39);
            p39.param1 = (float) -1.0436125E38F;
            p39.frame = MAV_FRAME.MAV_FRAME_GLOBAL_INT;
            p39.command = MAV_CMD.MAV_CMD_DO_SET_SERVO;
            p39.x = (float)2.6512892E38F;
            p39.param2 = (float)9.877955E37F;
            p39.y = (float)2.056701E38F;
            p39.target_component = (byte)(byte)95;
            p39.autocontinue = (byte)(byte)117;
            p39.seq = (ushort)(ushort)62590;
            p39.param4 = (float)3.035903E37F;
            p39.param3 = (float) -5.335969E36F;
            p39.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION;
            p39.z = (float)2.527882E38F;
            p39.current = (byte)(byte)134;
            p39.target_system = (byte)(byte)196;
            SMP_TEST_CH.send(p39);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)58);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
                Debug.Assert(pack.target_component == (byte)(byte)219);
                Debug.Assert(pack.seq == (ushort)(ushort)20149);
            };
            MISSION_REQUEST p40 = new MISSION_REQUEST();
            PH.setPack(p40);
            p40.target_system = (byte)(byte)58;
            p40.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE;
            p40.seq = (ushort)(ushort)20149;
            p40.target_component = (byte)(byte)219;
            SMP_TEST_CH.send(p40);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_SET_CURRENTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (ushort)(ushort)26180);
                Debug.Assert(pack.target_system == (byte)(byte)3);
                Debug.Assert(pack.target_component == (byte)(byte)18);
            };
            MISSION_SET_CURRENT p41 = new MISSION_SET_CURRENT();
            PH.setPack(p41);
            p41.seq = (ushort)(ushort)26180;
            p41.target_system = (byte)(byte)3;
            p41.target_component = (byte)(byte)18;
            SMP_TEST_CH.send(p41);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_CURRENTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (ushort)(ushort)567);
            };
            MISSION_CURRENT p42 = new MISSION_CURRENT();
            PH.setPack(p42);
            p42.seq = (ushort)(ushort)567;
            SMP_TEST_CH.send(p42);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)100);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
                Debug.Assert(pack.target_system == (byte)(byte)37);
            };
            MISSION_REQUEST_LIST p43 = new MISSION_REQUEST_LIST();
            PH.setPack(p43);
            p43.target_system = (byte)(byte)37;
            p43.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY;
            p43.target_component = (byte)(byte)100;
            SMP_TEST_CH.send(p43);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_COUNTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
                Debug.Assert(pack.count == (ushort)(ushort)9353);
                Debug.Assert(pack.target_component == (byte)(byte)186);
                Debug.Assert(pack.target_system == (byte)(byte)59);
            };
            MISSION_COUNT p44 = new MISSION_COUNT();
            PH.setPack(p44);
            p44.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL;
            p44.target_system = (byte)(byte)59;
            p44.count = (ushort)(ushort)9353;
            p44.target_component = (byte)(byte)186;
            SMP_TEST_CH.send(p44);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_CLEAR_ALLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)228);
                Debug.Assert(pack.target_system == (byte)(byte)238);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
            };
            MISSION_CLEAR_ALL p45 = new MISSION_CLEAR_ALL();
            PH.setPack(p45);
            p45.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION;
            p45.target_component = (byte)(byte)228;
            p45.target_system = (byte)(byte)238;
            SMP_TEST_CH.send(p45);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_ITEM_REACHEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (ushort)(ushort)27742);
            };
            MISSION_ITEM_REACHED p46 = new MISSION_ITEM_REACHED();
            PH.setPack(p46);
            p46.seq = (ushort)(ushort)27742;
            SMP_TEST_CH.send(p46);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)73);
                Debug.Assert(pack.target_system == (byte)(byte)144);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
                Debug.Assert(pack.type == MAV_MISSION_RESULT.MAV_MISSION_NO_SPACE);
            };
            MISSION_ACK p47 = new MISSION_ACK();
            PH.setPack(p47);
            p47.target_system = (byte)(byte)144;
            p47.type = MAV_MISSION_RESULT.MAV_MISSION_NO_SPACE;
            p47.target_component = (byte)(byte)73;
            p47.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL;
            SMP_TEST_CH.send(p47);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_GPS_GLOBAL_ORIGINReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.altitude == (int)911676538);
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)67942415196449503L);
                Debug.Assert(pack.target_system == (byte)(byte)43);
                Debug.Assert(pack.longitude == (int)1297978158);
                Debug.Assert(pack.latitude == (int)103304645);
            };
            SET_GPS_GLOBAL_ORIGIN p48 = new SET_GPS_GLOBAL_ORIGIN();
            PH.setPack(p48);
            p48.target_system = (byte)(byte)43;
            p48.time_usec_SET((ulong)67942415196449503L, PH) ;
            p48.latitude = (int)103304645;
            p48.altitude = (int)911676538;
            p48.longitude = (int)1297978158;
            SMP_TEST_CH.send(p48);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_GLOBAL_ORIGINReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.longitude == (int)257143665);
                Debug.Assert(pack.altitude == (int) -623208903);
                Debug.Assert(pack.latitude == (int)643913369);
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)9200255533626851642L);
            };
            GPS_GLOBAL_ORIGIN p49 = new GPS_GLOBAL_ORIGIN();
            PH.setPack(p49);
            p49.altitude = (int) -623208903;
            p49.latitude = (int)643913369;
            p49.time_usec_SET((ulong)9200255533626851642L, PH) ;
            p49.longitude = (int)257143665;
            SMP_TEST_CH.send(p49);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_MAP_RCReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_value_max == (float)9.620339E37F);
                Debug.Assert(pack.target_system == (byte)(byte)183);
                Debug.Assert(pack.param_value_min == (float) -1.1038044E38F);
                Debug.Assert(pack.target_component == (byte)(byte)48);
                Debug.Assert(pack.scale == (float) -2.0500985E38F);
                Debug.Assert(pack.param_id_LEN(ph) == 3);
                Debug.Assert(pack.param_id_TRY(ph).Equals("ayo"));
                Debug.Assert(pack.param_value0 == (float)3.1862828E38F);
                Debug.Assert(pack.param_index == (short)(short)29515);
                Debug.Assert(pack.parameter_rc_channel_index == (byte)(byte)58);
            };
            PARAM_MAP_RC p50 = new PARAM_MAP_RC();
            PH.setPack(p50);
            p50.target_component = (byte)(byte)48;
            p50.param_index = (short)(short)29515;
            p50.parameter_rc_channel_index = (byte)(byte)58;
            p50.target_system = (byte)(byte)183;
            p50.scale = (float) -2.0500985E38F;
            p50.param_value_min = (float) -1.1038044E38F;
            p50.param_value0 = (float)3.1862828E38F;
            p50.param_value_max = (float)9.620339E37F;
            p50.param_id_SET("ayo", PH) ;
            SMP_TEST_CH.send(p50);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_REQUEST_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)109);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
                Debug.Assert(pack.seq == (ushort)(ushort)6794);
                Debug.Assert(pack.target_component == (byte)(byte)202);
            };
            MISSION_REQUEST_INT p51 = new MISSION_REQUEST_INT();
            PH.setPack(p51);
            p51.target_system = (byte)(byte)109;
            p51.target_component = (byte)(byte)202;
            p51.seq = (ushort)(ushort)6794;
            p51.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION;
            SMP_TEST_CH.send(p51);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSAFETY_SET_ALLOWED_AREAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)228);
                Debug.Assert(pack.p2x == (float)2.5202537E38F);
                Debug.Assert(pack.p2z == (float)2.9541658E38F);
                Debug.Assert(pack.frame == MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT);
                Debug.Assert(pack.p1y == (float) -1.759995E38F);
                Debug.Assert(pack.p1x == (float)3.3887655E38F);
                Debug.Assert(pack.p2y == (float)2.7679114E38F);
                Debug.Assert(pack.p1z == (float) -4.9141495E37F);
                Debug.Assert(pack.target_system == (byte)(byte)120);
            };
            SAFETY_SET_ALLOWED_AREA p54 = new SAFETY_SET_ALLOWED_AREA();
            PH.setPack(p54);
            p54.p1x = (float)3.3887655E38F;
            p54.target_component = (byte)(byte)228;
            p54.p2x = (float)2.5202537E38F;
            p54.p2y = (float)2.7679114E38F;
            p54.p1y = (float) -1.759995E38F;
            p54.frame = MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT;
            p54.p2z = (float)2.9541658E38F;
            p54.target_system = (byte)(byte)120;
            p54.p1z = (float) -4.9141495E37F;
            SMP_TEST_CH.send(p54);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSAFETY_ALLOWED_AREAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.p1y == (float)2.6763297E38F);
                Debug.Assert(pack.p1x == (float) -2.4302418E38F);
                Debug.Assert(pack.p2y == (float) -2.3560067E38F);
                Debug.Assert(pack.frame == MAV_FRAME.MAV_FRAME_BODY_NED);
                Debug.Assert(pack.p2z == (float) -2.7668415E38F);
                Debug.Assert(pack.p1z == (float) -3.2158154E38F);
                Debug.Assert(pack.p2x == (float)5.6124064E37F);
            };
            SAFETY_ALLOWED_AREA p55 = new SAFETY_ALLOWED_AREA();
            PH.setPack(p55);
            p55.p2z = (float) -2.7668415E38F;
            p55.p2y = (float) -2.3560067E38F;
            p55.p2x = (float)5.6124064E37F;
            p55.p1x = (float) -2.4302418E38F;
            p55.frame = MAV_FRAME.MAV_FRAME_BODY_NED;
            p55.p1z = (float) -3.2158154E38F;
            p55.p1y = (float)2.6763297E38F;
            SMP_TEST_CH.send(p55);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATTITUDE_QUATERNION_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.q.SequenceEqual(new float[] {6.289175E37F, -8.562994E37F, -2.658677E38F, -2.8154944E38F}));
                Debug.Assert(pack.pitchspeed == (float)6.9143693E37F);
                Debug.Assert(pack.time_usec == (ulong)7540550029995303761L);
                Debug.Assert(pack.yawspeed == (float)5.66928E37F);
                Debug.Assert(pack.rollspeed == (float) -1.7976458E38F);
                Debug.Assert(pack.covariance.SequenceEqual(new float[] {-5.1665027E37F, 3.3064325E38F, -2.1701032E38F, -2.9052874E38F, -3.1893592E38F, -6.417918E37F, -9.518911E37F, 3.2630833E38F, -7.2542817E37F}));
            };
            ATTITUDE_QUATERNION_COV p61 = new ATTITUDE_QUATERNION_COV();
            PH.setPack(p61);
            p61.yawspeed = (float)5.66928E37F;
            p61.covariance_SET(new float[] {-5.1665027E37F, 3.3064325E38F, -2.1701032E38F, -2.9052874E38F, -3.1893592E38F, -6.417918E37F, -9.518911E37F, 3.2630833E38F, -7.2542817E37F}, 0) ;
            p61.q_SET(new float[] {6.289175E37F, -8.562994E37F, -2.658677E38F, -2.8154944E38F}, 0) ;
            p61.time_usec = (ulong)7540550029995303761L;
            p61.rollspeed = (float) -1.7976458E38F;
            p61.pitchspeed = (float)6.9143693E37F;
            SMP_TEST_CH.send(p61);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnNAV_CONTROLLER_OUTPUTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.xtrack_error == (float) -4.7607526E37F);
                Debug.Assert(pack.wp_dist == (ushort)(ushort)54216);
                Debug.Assert(pack.alt_error == (float) -2.3695083E38F);
                Debug.Assert(pack.aspd_error == (float) -3.2555148E38F);
                Debug.Assert(pack.nav_bearing == (short)(short)19358);
                Debug.Assert(pack.target_bearing == (short)(short) -29979);
                Debug.Assert(pack.nav_roll == (float)3.2815833E38F);
                Debug.Assert(pack.nav_pitch == (float)2.8001221E38F);
            };
            NAV_CONTROLLER_OUTPUT p62 = new NAV_CONTROLLER_OUTPUT();
            PH.setPack(p62);
            p62.nav_pitch = (float)2.8001221E38F;
            p62.alt_error = (float) -2.3695083E38F;
            p62.aspd_error = (float) -3.2555148E38F;
            p62.xtrack_error = (float) -4.7607526E37F;
            p62.target_bearing = (short)(short) -29979;
            p62.nav_bearing = (short)(short)19358;
            p62.wp_dist = (ushort)(ushort)54216;
            p62.nav_roll = (float)3.2815833E38F;
            SMP_TEST_CH.send(p62);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGLOBAL_POSITION_INT_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)1673975492150325753L);
                Debug.Assert(pack.vz == (float) -2.9590707E38F);
                Debug.Assert(pack.lat == (int)168333792);
                Debug.Assert(pack.estimator_type == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VIO);
                Debug.Assert(pack.covariance.SequenceEqual(new float[] {3.3645404E38F, 1.4763555E38F, -3.1660787E38F, -2.3468454E37F, -1.4853761E38F, 3.25082E38F, -1.6417771E37F, -1.9956317E38F, -2.0116122E38F, 1.4728716E38F, -1.6450774E38F, -1.3248705E38F, 2.7701176E38F, -3.561578E37F, -2.9804522E38F, 8.998932E37F, 2.5374124E38F, -7.3985383E37F, 2.74198E38F, 4.254235E37F, -1.3471249E38F, -3.0453868E38F, 8.804733E37F, -2.5838194E38F, -9.038043E37F, 2.9185765E38F, 1.6756232E38F, -1.2675888E38F, -1.0868652E38F, -2.873067E37F, 1.2334545E38F, 2.2854306E38F, -8.165343E37F, 2.408789E38F, 1.1721116E37F, 1.0084474E38F}));
                Debug.Assert(pack.relative_alt == (int)1355090151);
                Debug.Assert(pack.alt == (int) -489661534);
                Debug.Assert(pack.lon == (int)1463053151);
                Debug.Assert(pack.vx == (float)2.1393627E36F);
                Debug.Assert(pack.vy == (float)1.2208179E38F);
            };
            GLOBAL_POSITION_INT_COV p63 = new GLOBAL_POSITION_INT_COV();
            PH.setPack(p63);
            p63.relative_alt = (int)1355090151;
            p63.alt = (int) -489661534;
            p63.vy = (float)1.2208179E38F;
            p63.vx = (float)2.1393627E36F;
            p63.lon = (int)1463053151;
            p63.time_usec = (ulong)1673975492150325753L;
            p63.vz = (float) -2.9590707E38F;
            p63.estimator_type = MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VIO;
            p63.covariance_SET(new float[] {3.3645404E38F, 1.4763555E38F, -3.1660787E38F, -2.3468454E37F, -1.4853761E38F, 3.25082E38F, -1.6417771E37F, -1.9956317E38F, -2.0116122E38F, 1.4728716E38F, -1.6450774E38F, -1.3248705E38F, 2.7701176E38F, -3.561578E37F, -2.9804522E38F, 8.998932E37F, 2.5374124E38F, -7.3985383E37F, 2.74198E38F, 4.254235E37F, -1.3471249E38F, -3.0453868E38F, 8.804733E37F, -2.5838194E38F, -9.038043E37F, 2.9185765E38F, 1.6756232E38F, -1.2675888E38F, -1.0868652E38F, -2.873067E37F, 1.2334545E38F, 2.2854306E38F, -8.165343E37F, 2.408789E38F, 1.1721116E37F, 1.0084474E38F}, 0) ;
            p63.lat = (int)168333792;
            SMP_TEST_CH.send(p63);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOCAL_POSITION_NED_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)6253410289137174200L);
                Debug.Assert(pack.y == (float) -2.761668E38F);
                Debug.Assert(pack.x == (float)1.0867746E38F);
                Debug.Assert(pack.ax == (float) -1.3815318E38F);
                Debug.Assert(pack.z == (float)1.4831496E38F);
                Debug.Assert(pack.az == (float)2.0211873E38F);
                Debug.Assert(pack.vy == (float)6.964771E37F);
                Debug.Assert(pack.vx == (float)2.9857806E38F);
                Debug.Assert(pack.ay == (float) -2.437486E38F);
                Debug.Assert(pack.covariance.SequenceEqual(new float[] {-2.4202634E38F, -2.1668682E38F, 1.259225E38F, -1.4105328E38F, -1.1446718E38F, 1.8085806E38F, -8.137486E37F, -8.438825E37F, 2.8268801E38F, -1.5138397E38F, -2.1473527E38F, 3.3062804E38F, -8.345863E37F, -3.272358E38F, -4.1060734E37F, -2.0112755E38F, 3.5162206E36F, -2.181275E38F, 1.5839204E37F, -3.3111017E38F, -2.8099043E38F, 1.7920426E38F, 2.240826E38F, -4.1834288E37F, 8.1897444E37F, 1.261016E38F, 3.3705673E38F, 8.994582E37F, -1.4359509E38F, 5.1175846E37F, -2.4810918E38F, -2.532714E37F, -1.7595345E38F, -2.9505841E38F, 1.9339241E38F, 1.3151925E38F, 1.9185334E38F, -5.7924493E37F, 2.4075437E38F, 9.465385E37F, 2.767688E38F, -1.1970296E38F, 2.7342781E38F, 2.0717504E38F, -3.1120719E38F}));
                Debug.Assert(pack.estimator_type == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_NAIVE);
                Debug.Assert(pack.vz == (float) -2.9354936E38F);
            };
            LOCAL_POSITION_NED_COV p64 = new LOCAL_POSITION_NED_COV();
            PH.setPack(p64);
            p64.vz = (float) -2.9354936E38F;
            p64.ax = (float) -1.3815318E38F;
            p64.time_usec = (ulong)6253410289137174200L;
            p64.covariance_SET(new float[] {-2.4202634E38F, -2.1668682E38F, 1.259225E38F, -1.4105328E38F, -1.1446718E38F, 1.8085806E38F, -8.137486E37F, -8.438825E37F, 2.8268801E38F, -1.5138397E38F, -2.1473527E38F, 3.3062804E38F, -8.345863E37F, -3.272358E38F, -4.1060734E37F, -2.0112755E38F, 3.5162206E36F, -2.181275E38F, 1.5839204E37F, -3.3111017E38F, -2.8099043E38F, 1.7920426E38F, 2.240826E38F, -4.1834288E37F, 8.1897444E37F, 1.261016E38F, 3.3705673E38F, 8.994582E37F, -1.4359509E38F, 5.1175846E37F, -2.4810918E38F, -2.532714E37F, -1.7595345E38F, -2.9505841E38F, 1.9339241E38F, 1.3151925E38F, 1.9185334E38F, -5.7924493E37F, 2.4075437E38F, 9.465385E37F, 2.767688E38F, -1.1970296E38F, 2.7342781E38F, 2.0717504E38F, -3.1120719E38F}, 0) ;
            p64.y = (float) -2.761668E38F;
            p64.z = (float)1.4831496E38F;
            p64.az = (float)2.0211873E38F;
            p64.ay = (float) -2.437486E38F;
            p64.x = (float)1.0867746E38F;
            p64.vx = (float)2.9857806E38F;
            p64.vy = (float)6.964771E37F;
            p64.estimator_type = MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_NAIVE;
            SMP_TEST_CH.send(p64);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRC_CHANNELSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)58240);
                Debug.Assert(pack.chan11_raw == (ushort)(ushort)52637);
                Debug.Assert(pack.chan16_raw == (ushort)(ushort)22316);
                Debug.Assert(pack.chan9_raw == (ushort)(ushort)20044);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)28352);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)8013);
                Debug.Assert(pack.chan13_raw == (ushort)(ushort)48779);
                Debug.Assert(pack.rssi == (byte)(byte)100);
                Debug.Assert(pack.chan17_raw == (ushort)(ushort)50503);
                Debug.Assert(pack.chan12_raw == (ushort)(ushort)24961);
                Debug.Assert(pack.chan15_raw == (ushort)(ushort)55214);
                Debug.Assert(pack.chancount == (byte)(byte)241);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)10532);
                Debug.Assert(pack.chan10_raw == (ushort)(ushort)59386);
                Debug.Assert(pack.time_boot_ms == (uint)1246773110U);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)32456);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)30885);
                Debug.Assert(pack.chan18_raw == (ushort)(ushort)34349);
                Debug.Assert(pack.chan14_raw == (ushort)(ushort)35912);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)34548);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)10583);
            };
            RC_CHANNELS p65 = new RC_CHANNELS();
            PH.setPack(p65);
            p65.time_boot_ms = (uint)1246773110U;
            p65.chan2_raw = (ushort)(ushort)32456;
            p65.chan8_raw = (ushort)(ushort)58240;
            p65.chan5_raw = (ushort)(ushort)30885;
            p65.chan15_raw = (ushort)(ushort)55214;
            p65.chan3_raw = (ushort)(ushort)34548;
            p65.rssi = (byte)(byte)100;
            p65.chan6_raw = (ushort)(ushort)28352;
            p65.chan9_raw = (ushort)(ushort)20044;
            p65.chan1_raw = (ushort)(ushort)8013;
            p65.chan11_raw = (ushort)(ushort)52637;
            p65.chan16_raw = (ushort)(ushort)22316;
            p65.chan12_raw = (ushort)(ushort)24961;
            p65.chan14_raw = (ushort)(ushort)35912;
            p65.chan4_raw = (ushort)(ushort)10532;
            p65.chan10_raw = (ushort)(ushort)59386;
            p65.chancount = (byte)(byte)241;
            p65.chan17_raw = (ushort)(ushort)50503;
            p65.chan18_raw = (ushort)(ushort)34349;
            p65.chan13_raw = (ushort)(ushort)48779;
            p65.chan7_raw = (ushort)(ushort)10583;
            SMP_TEST_CH.send(p65);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnREQUEST_DATA_STREAMReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.req_message_rate == (ushort)(ushort)19862);
                Debug.Assert(pack.target_component == (byte)(byte)115);
                Debug.Assert(pack.req_stream_id == (byte)(byte)46);
                Debug.Assert(pack.start_stop == (byte)(byte)187);
                Debug.Assert(pack.target_system == (byte)(byte)243);
            };
            REQUEST_DATA_STREAM p66 = new REQUEST_DATA_STREAM();
            PH.setPack(p66);
            p66.target_component = (byte)(byte)115;
            p66.req_stream_id = (byte)(byte)46;
            p66.start_stop = (byte)(byte)187;
            p66.req_message_rate = (ushort)(ushort)19862;
            p66.target_system = (byte)(byte)243;
            SMP_TEST_CH.send(p66);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnDATA_STREAMReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.stream_id == (byte)(byte)2);
                Debug.Assert(pack.message_rate == (ushort)(ushort)1282);
                Debug.Assert(pack.on_off == (byte)(byte)187);
            };
            DATA_STREAM p67 = new DATA_STREAM();
            PH.setPack(p67);
            p67.stream_id = (byte)(byte)2;
            p67.on_off = (byte)(byte)187;
            p67.message_rate = (ushort)(ushort)1282;
            SMP_TEST_CH.send(p67);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMANUAL_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.y == (short)(short)13695);
                Debug.Assert(pack.r == (short)(short) -13572);
                Debug.Assert(pack.x == (short)(short) -5926);
                Debug.Assert(pack.target == (byte)(byte)72);
                Debug.Assert(pack.buttons == (ushort)(ushort)55921);
                Debug.Assert(pack.z == (short)(short) -13423);
            };
            MANUAL_CONTROL p69 = new MANUAL_CONTROL();
            PH.setPack(p69);
            p69.target = (byte)(byte)72;
            p69.z = (short)(short) -13423;
            p69.buttons = (ushort)(ushort)55921;
            p69.y = (short)(short)13695;
            p69.r = (short)(short) -13572;
            p69.x = (short)(short) -5926;
            SMP_TEST_CH.send(p69);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRC_CHANNELS_OVERRIDEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)43491);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)52880);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)40635);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)37423);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)35689);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)29199);
                Debug.Assert(pack.target_system == (byte)(byte)239);
                Debug.Assert(pack.target_component == (byte)(byte)96);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)5290);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)18703);
            };
            RC_CHANNELS_OVERRIDE p70 = new RC_CHANNELS_OVERRIDE();
            PH.setPack(p70);
            p70.chan8_raw = (ushort)(ushort)18703;
            p70.chan2_raw = (ushort)(ushort)43491;
            p70.chan6_raw = (ushort)(ushort)29199;
            p70.target_component = (byte)(byte)96;
            p70.chan1_raw = (ushort)(ushort)52880;
            p70.chan7_raw = (ushort)(ushort)35689;
            p70.chan4_raw = (ushort)(ushort)5290;
            p70.chan3_raw = (ushort)(ushort)37423;
            p70.target_system = (byte)(byte)239;
            p70.chan5_raw = (ushort)(ushort)40635;
            SMP_TEST_CH.send(p70);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_ITEM_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.current == (byte)(byte)244);
                Debug.Assert(pack.y == (int) -1213411937);
                Debug.Assert(pack.param2 == (float)3.029059E38F);
                Debug.Assert(pack.param1 == (float)6.848574E37F);
                Debug.Assert(pack.x == (int)1860037020);
                Debug.Assert(pack.param3 == (float)2.9945333E38F);
                Debug.Assert(pack.command == MAV_CMD.MAV_CMD_NAV_LOITER_TO_ALT);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
                Debug.Assert(pack.autocontinue == (byte)(byte)136);
                Debug.Assert(pack.seq == (ushort)(ushort)59593);
                Debug.Assert(pack.target_system == (byte)(byte)40);
                Debug.Assert(pack.z == (float)4.842393E37F);
                Debug.Assert(pack.param4 == (float)2.2611818E38F);
                Debug.Assert(pack.frame == MAV_FRAME.MAV_FRAME_MISSION);
                Debug.Assert(pack.target_component == (byte)(byte)215);
            };
            MISSION_ITEM_INT p73 = new MISSION_ITEM_INT();
            PH.setPack(p73);
            p73.current = (byte)(byte)244;
            p73.x = (int)1860037020;
            p73.param2 = (float)3.029059E38F;
            p73.seq = (ushort)(ushort)59593;
            p73.target_system = (byte)(byte)40;
            p73.command = MAV_CMD.MAV_CMD_NAV_LOITER_TO_ALT;
            p73.param3 = (float)2.9945333E38F;
            p73.frame = MAV_FRAME.MAV_FRAME_MISSION;
            p73.y = (int) -1213411937;
            p73.param1 = (float)6.848574E37F;
            p73.target_component = (byte)(byte)215;
            p73.autocontinue = (byte)(byte)136;
            p73.z = (float)4.842393E37F;
            p73.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION;
            p73.param4 = (float)2.2611818E38F;
            SMP_TEST_CH.send(p73);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnVFR_HUDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.airspeed == (float) -1.4624814E38F);
                Debug.Assert(pack.heading == (short)(short) -7959);
                Debug.Assert(pack.alt == (float)9.241659E37F);
                Debug.Assert(pack.groundspeed == (float) -1.989728E38F);
                Debug.Assert(pack.throttle == (ushort)(ushort)40304);
                Debug.Assert(pack.climb == (float) -2.7374306E38F);
            };
            VFR_HUD p74 = new VFR_HUD();
            PH.setPack(p74);
            p74.airspeed = (float) -1.4624814E38F;
            p74.heading = (short)(short) -7959;
            p74.climb = (float) -2.7374306E38F;
            p74.alt = (float)9.241659E37F;
            p74.groundspeed = (float) -1.989728E38F;
            p74.throttle = (ushort)(ushort)40304;
            SMP_TEST_CH.send(p74);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCOMMAND_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)75);
                Debug.Assert(pack.command == MAV_CMD.MAV_CMD_USER_1);
                Debug.Assert(pack.param3 == (float) -4.532216E37F);
                Debug.Assert(pack.target_component == (byte)(byte)54);
                Debug.Assert(pack.x == (int) -891548290);
                Debug.Assert(pack.y == (int)536477110);
                Debug.Assert(pack.current == (byte)(byte)153);
                Debug.Assert(pack.z == (float)8.899073E37F);
                Debug.Assert(pack.param2 == (float)1.1324519E38F);
                Debug.Assert(pack.autocontinue == (byte)(byte)73);
                Debug.Assert(pack.frame == MAV_FRAME.MAV_FRAME_MISSION);
                Debug.Assert(pack.param4 == (float)5.7556895E37F);
                Debug.Assert(pack.param1 == (float)2.5722837E38F);
            };
            COMMAND_INT p75 = new COMMAND_INT();
            PH.setPack(p75);
            p75.x = (int) -891548290;
            p75.param1 = (float)2.5722837E38F;
            p75.param4 = (float)5.7556895E37F;
            p75.frame = MAV_FRAME.MAV_FRAME_MISSION;
            p75.target_system = (byte)(byte)75;
            p75.target_component = (byte)(byte)54;
            p75.param2 = (float)1.1324519E38F;
            p75.param3 = (float) -4.532216E37F;
            p75.y = (int)536477110;
            p75.current = (byte)(byte)153;
            p75.command = MAV_CMD.MAV_CMD_USER_1;
            p75.z = (float)8.899073E37F;
            p75.autocontinue = (byte)(byte)73;
            SMP_TEST_CH.send(p75);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCOMMAND_LONGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param6 == (float) -2.2044105E38F);
                Debug.Assert(pack.param3 == (float) -2.1461124E38F);
                Debug.Assert(pack.param7 == (float) -2.8578662E38F);
                Debug.Assert(pack.target_system == (byte)(byte)128);
                Debug.Assert(pack.confirmation == (byte)(byte)175);
                Debug.Assert(pack.param1 == (float)1.9206136E38F);
                Debug.Assert(pack.target_component == (byte)(byte)172);
                Debug.Assert(pack.param4 == (float)8.4396364E37F);
                Debug.Assert(pack.command == MAV_CMD.MAV_CMD_REQUEST_PROTOCOL_VERSION);
                Debug.Assert(pack.param2 == (float) -3.1713388E37F);
                Debug.Assert(pack.param5 == (float) -2.7176867E38F);
            };
            COMMAND_LONG p76 = new COMMAND_LONG();
            PH.setPack(p76);
            p76.command = MAV_CMD.MAV_CMD_REQUEST_PROTOCOL_VERSION;
            p76.param4 = (float)8.4396364E37F;
            p76.param1 = (float)1.9206136E38F;
            p76.param2 = (float) -3.1713388E37F;
            p76.param7 = (float) -2.8578662E38F;
            p76.target_component = (byte)(byte)172;
            p76.confirmation = (byte)(byte)175;
            p76.target_system = (byte)(byte)128;
            p76.param5 = (float) -2.7176867E38F;
            p76.param6 = (float) -2.2044105E38F;
            p76.param3 = (float) -2.1461124E38F;
            SMP_TEST_CH.send(p76);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCOMMAND_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system_TRY(ph) == (byte)(byte)179);
                Debug.Assert(pack.target_component_TRY(ph) == (byte)(byte)52);
                Debug.Assert(pack.command == MAV_CMD.MAV_CMD_OVERRIDE_GOTO);
                Debug.Assert(pack.progress_TRY(ph) == (byte)(byte)130);
                Debug.Assert(pack.result == MAV_RESULT.MAV_RESULT_FAILED);
                Debug.Assert(pack.result_param2_TRY(ph) == (int)1458618351);
            };
            COMMAND_ACK p77 = new COMMAND_ACK();
            PH.setPack(p77);
            p77.result = MAV_RESULT.MAV_RESULT_FAILED;
            p77.result_param2_SET((int)1458618351, PH) ;
            p77.target_component_SET((byte)(byte)52, PH) ;
            p77.target_system_SET((byte)(byte)179, PH) ;
            p77.progress_SET((byte)(byte)130, PH) ;
            p77.command = MAV_CMD.MAV_CMD_OVERRIDE_GOTO;
            SMP_TEST_CH.send(p77);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMANUAL_SETPOINTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)519518855U);
                Debug.Assert(pack.thrust == (float) -2.8904993E38F);
                Debug.Assert(pack.manual_override_switch == (byte)(byte)190);
                Debug.Assert(pack.yaw == (float) -3.6534829E37F);
                Debug.Assert(pack.roll == (float) -5.110413E37F);
                Debug.Assert(pack.pitch == (float) -2.30869E37F);
                Debug.Assert(pack.mode_switch == (byte)(byte)39);
            };
            MANUAL_SETPOINT p81 = new MANUAL_SETPOINT();
            PH.setPack(p81);
            p81.manual_override_switch = (byte)(byte)190;
            p81.roll = (float) -5.110413E37F;
            p81.mode_switch = (byte)(byte)39;
            p81.thrust = (float) -2.8904993E38F;
            p81.time_boot_ms = (uint)519518855U;
            p81.yaw = (float) -3.6534829E37F;
            p81.pitch = (float) -2.30869E37F;
            SMP_TEST_CH.send(p81);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_ATTITUDE_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)130);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-3.1253258E38F, 9.026386E37F, -1.2926062E38F, 3.0059738E37F}));
                Debug.Assert(pack.target_system == (byte)(byte)210);
                Debug.Assert(pack.time_boot_ms == (uint)3655825936U);
                Debug.Assert(pack.type_mask == (byte)(byte)98);
                Debug.Assert(pack.body_yaw_rate == (float)1.9161346E38F);
                Debug.Assert(pack.body_pitch_rate == (float) -2.8588967E38F);
                Debug.Assert(pack.thrust == (float) -2.5268627E38F);
                Debug.Assert(pack.body_roll_rate == (float) -1.6382891E38F);
            };
            SET_ATTITUDE_TARGET p82 = new SET_ATTITUDE_TARGET();
            PH.setPack(p82);
            p82.type_mask = (byte)(byte)98;
            p82.q_SET(new float[] {-3.1253258E38F, 9.026386E37F, -1.2926062E38F, 3.0059738E37F}, 0) ;
            p82.thrust = (float) -2.5268627E38F;
            p82.body_pitch_rate = (float) -2.8588967E38F;
            p82.body_yaw_rate = (float)1.9161346E38F;
            p82.time_boot_ms = (uint)3655825936U;
            p82.target_component = (byte)(byte)130;
            p82.body_roll_rate = (float) -1.6382891E38F;
            p82.target_system = (byte)(byte)210;
            SMP_TEST_CH.send(p82);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATTITUDE_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.body_roll_rate == (float) -3.269951E38F);
                Debug.Assert(pack.type_mask == (byte)(byte)125);
                Debug.Assert(pack.body_yaw_rate == (float) -2.817711E38F);
                Debug.Assert(pack.thrust == (float) -3.390584E38F);
                Debug.Assert(pack.time_boot_ms == (uint)2900286796U);
                Debug.Assert(pack.body_pitch_rate == (float) -3.2578474E38F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {2.220646E38F, 2.9528243E38F, -2.468411E38F, 1.1481269E38F}));
            };
            ATTITUDE_TARGET p83 = new ATTITUDE_TARGET();
            PH.setPack(p83);
            p83.q_SET(new float[] {2.220646E38F, 2.9528243E38F, -2.468411E38F, 1.1481269E38F}, 0) ;
            p83.type_mask = (byte)(byte)125;
            p83.body_roll_rate = (float) -3.269951E38F;
            p83.time_boot_ms = (uint)2900286796U;
            p83.body_pitch_rate = (float) -3.2578474E38F;
            p83.body_yaw_rate = (float) -2.817711E38F;
            p83.thrust = (float) -3.390584E38F;
            SMP_TEST_CH.send(p83);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_POSITION_TARGET_LOCAL_NEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.type_mask == (ushort)(ushort)59875);
                Debug.Assert(pack.yaw_rate == (float) -9.339933E37F);
                Debug.Assert(pack.y == (float)1.744046E38F);
                Debug.Assert(pack.target_component == (byte)(byte)212);
                Debug.Assert(pack.vz == (float)8.593096E37F);
                Debug.Assert(pack.afx == (float) -2.1194376E38F);
                Debug.Assert(pack.target_system == (byte)(byte)111);
                Debug.Assert(pack.yaw == (float) -1.742141E38F);
                Debug.Assert(pack.afy == (float) -3.3480875E38F);
                Debug.Assert(pack.vy == (float) -8.403574E37F);
                Debug.Assert(pack.time_boot_ms == (uint)862372092U);
                Debug.Assert(pack.z == (float)3.4668434E37F);
                Debug.Assert(pack.x == (float)1.8832085E38F);
                Debug.Assert(pack.coordinate_frame == MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED);
                Debug.Assert(pack.afz == (float) -1.0526326E38F);
                Debug.Assert(pack.vx == (float)1.817906E38F);
            };
            SET_POSITION_TARGET_LOCAL_NED p84 = new SET_POSITION_TARGET_LOCAL_NED();
            PH.setPack(p84);
            p84.vy = (float) -8.403574E37F;
            p84.yaw = (float) -1.742141E38F;
            p84.x = (float)1.8832085E38F;
            p84.z = (float)3.4668434E37F;
            p84.y = (float)1.744046E38F;
            p84.vz = (float)8.593096E37F;
            p84.type_mask = (ushort)(ushort)59875;
            p84.coordinate_frame = MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED;
            p84.afz = (float) -1.0526326E38F;
            p84.time_boot_ms = (uint)862372092U;
            p84.vx = (float)1.817906E38F;
            p84.yaw_rate = (float) -9.339933E37F;
            p84.afy = (float) -3.3480875E38F;
            p84.target_component = (byte)(byte)212;
            p84.target_system = (byte)(byte)111;
            p84.afx = (float) -2.1194376E38F;
            SMP_TEST_CH.send(p84);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_POSITION_TARGET_GLOBAL_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.afx == (float)3.9937297E37F);
                Debug.Assert(pack.time_boot_ms == (uint)3288288300U);
                Debug.Assert(pack.coordinate_frame == MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT);
                Debug.Assert(pack.target_system == (byte)(byte)254);
                Debug.Assert(pack.vz == (float) -7.2805413E37F);
                Debug.Assert(pack.yaw == (float) -1.9810424E36F);
                Debug.Assert(pack.afy == (float)1.7150783E38F);
                Debug.Assert(pack.alt == (float) -3.2743137E38F);
                Debug.Assert(pack.vx == (float)1.3767137E38F);
                Debug.Assert(pack.vy == (float)1.3997489E38F);
                Debug.Assert(pack.lon_int == (int) -1359386253);
                Debug.Assert(pack.yaw_rate == (float)3.3576328E38F);
                Debug.Assert(pack.type_mask == (ushort)(ushort)45628);
                Debug.Assert(pack.afz == (float)2.8629812E38F);
                Debug.Assert(pack.lat_int == (int)749942071);
                Debug.Assert(pack.target_component == (byte)(byte)195);
            };
            SET_POSITION_TARGET_GLOBAL_INT p86 = new SET_POSITION_TARGET_GLOBAL_INT();
            PH.setPack(p86);
            p86.time_boot_ms = (uint)3288288300U;
            p86.target_system = (byte)(byte)254;
            p86.target_component = (byte)(byte)195;
            p86.coordinate_frame = MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT;
            p86.vx = (float)1.3767137E38F;
            p86.lon_int = (int) -1359386253;
            p86.vy = (float)1.3997489E38F;
            p86.yaw = (float) -1.9810424E36F;
            p86.yaw_rate = (float)3.3576328E38F;
            p86.type_mask = (ushort)(ushort)45628;
            p86.afx = (float)3.9937297E37F;
            p86.vz = (float) -7.2805413E37F;
            p86.lat_int = (int)749942071;
            p86.afz = (float)2.8629812E38F;
            p86.alt = (float) -3.2743137E38F;
            p86.afy = (float)1.7150783E38F;
            SMP_TEST_CH.send(p86);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPOSITION_TARGET_GLOBAL_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.afx == (float) -1.0474422E38F);
                Debug.Assert(pack.vy == (float)1.9017248E38F);
                Debug.Assert(pack.vz == (float) -1.5360216E38F);
                Debug.Assert(pack.alt == (float) -2.493504E38F);
                Debug.Assert(pack.coordinate_frame == MAV_FRAME.MAV_FRAME_GLOBAL);
                Debug.Assert(pack.lat_int == (int) -649332112);
                Debug.Assert(pack.yaw == (float)3.3743453E38F);
                Debug.Assert(pack.time_boot_ms == (uint)2095846486U);
                Debug.Assert(pack.vx == (float) -1.9966767E38F);
                Debug.Assert(pack.lon_int == (int) -1625402855);
                Debug.Assert(pack.type_mask == (ushort)(ushort)46397);
                Debug.Assert(pack.yaw_rate == (float)5.0033003E37F);
                Debug.Assert(pack.afz == (float) -2.964805E38F);
                Debug.Assert(pack.afy == (float) -2.91676E38F);
            };
            POSITION_TARGET_GLOBAL_INT p87 = new POSITION_TARGET_GLOBAL_INT();
            PH.setPack(p87);
            p87.afx = (float) -1.0474422E38F;
            p87.afy = (float) -2.91676E38F;
            p87.yaw = (float)3.3743453E38F;
            p87.vz = (float) -1.5360216E38F;
            p87.yaw_rate = (float)5.0033003E37F;
            p87.type_mask = (ushort)(ushort)46397;
            p87.coordinate_frame = MAV_FRAME.MAV_FRAME_GLOBAL;
            p87.time_boot_ms = (uint)2095846486U;
            p87.vx = (float) -1.9966767E38F;
            p87.lon_int = (int) -1625402855;
            p87.afz = (float) -2.964805E38F;
            p87.lat_int = (int) -649332112;
            p87.alt = (float) -2.493504E38F;
            p87.vy = (float)1.9017248E38F;
            SMP_TEST_CH.send(p87);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)4114171578U);
                Debug.Assert(pack.yaw == (float)6.1037027E37F);
                Debug.Assert(pack.x == (float)2.9161335E38F);
                Debug.Assert(pack.y == (float) -1.3736649E38F);
                Debug.Assert(pack.roll == (float) -3.3778755E38F);
                Debug.Assert(pack.z == (float) -1.657563E38F);
                Debug.Assert(pack.pitch == (float)2.0182606E38F);
            };
            LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET p89 = new LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
            PH.setPack(p89);
            p89.z = (float) -1.657563E38F;
            p89.yaw = (float)6.1037027E37F;
            p89.x = (float)2.9161335E38F;
            p89.y = (float) -1.3736649E38F;
            p89.pitch = (float)2.0182606E38F;
            p89.time_boot_ms = (uint)4114171578U;
            p89.roll = (float) -3.3778755E38F;
            SMP_TEST_CH.send(p89);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.yawspeed == (float) -1.0062947E38F);
                Debug.Assert(pack.roll == (float)1.0359903E38F);
                Debug.Assert(pack.time_usec == (ulong)3216082929510125028L);
                Debug.Assert(pack.yaw == (float)1.0847905E38F);
                Debug.Assert(pack.yacc == (short)(short)25705);
                Debug.Assert(pack.zacc == (short)(short)23031);
                Debug.Assert(pack.vx == (short)(short)4425);
                Debug.Assert(pack.pitchspeed == (float) -3.4514903E37F);
                Debug.Assert(pack.vz == (short)(short) -2524);
                Debug.Assert(pack.rollspeed == (float) -2.4228529E38F);
                Debug.Assert(pack.alt == (int) -236081519);
                Debug.Assert(pack.vy == (short)(short)9863);
                Debug.Assert(pack.xacc == (short)(short)19105);
                Debug.Assert(pack.pitch == (float) -1.8275341E37F);
                Debug.Assert(pack.lat == (int) -444300378);
                Debug.Assert(pack.lon == (int)933884843);
            };
            HIL_STATE p90 = new HIL_STATE();
            PH.setPack(p90);
            p90.yawspeed = (float) -1.0062947E38F;
            p90.yacc = (short)(short)25705;
            p90.time_usec = (ulong)3216082929510125028L;
            p90.rollspeed = (float) -2.4228529E38F;
            p90.vx = (short)(short)4425;
            p90.vz = (short)(short) -2524;
            p90.lon = (int)933884843;
            p90.vy = (short)(short)9863;
            p90.lat = (int) -444300378;
            p90.alt = (int) -236081519;
            p90.zacc = (short)(short)23031;
            p90.roll = (float)1.0359903E38F;
            p90.pitchspeed = (float) -3.4514903E37F;
            p90.pitch = (float) -1.8275341E37F;
            p90.xacc = (short)(short)19105;
            p90.yaw = (float)1.0847905E38F;
            SMP_TEST_CH.send(p90);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_CONTROLSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)8383262682995776358L);
                Debug.Assert(pack.throttle == (float) -1.7088934E38F);
                Debug.Assert(pack.yaw_rudder == (float) -7.24688E37F);
                Debug.Assert(pack.roll_ailerons == (float)1.5004303E38F);
                Debug.Assert(pack.aux4 == (float) -2.739039E38F);
                Debug.Assert(pack.pitch_elevator == (float)1.2235369E38F);
                Debug.Assert(pack.nav_mode == (byte)(byte)123);
                Debug.Assert(pack.aux3 == (float)1.4231205E38F);
                Debug.Assert(pack.aux2 == (float)2.0841648E38F);
                Debug.Assert(pack.aux1 == (float)8.702796E37F);
                Debug.Assert(pack.mode == MAV_MODE.MAV_MODE_STABILIZE_DISARMED);
            };
            HIL_CONTROLS p91 = new HIL_CONTROLS();
            PH.setPack(p91);
            p91.aux1 = (float)8.702796E37F;
            p91.pitch_elevator = (float)1.2235369E38F;
            p91.roll_ailerons = (float)1.5004303E38F;
            p91.time_usec = (ulong)8383262682995776358L;
            p91.aux2 = (float)2.0841648E38F;
            p91.aux3 = (float)1.4231205E38F;
            p91.throttle = (float) -1.7088934E38F;
            p91.mode = MAV_MODE.MAV_MODE_STABILIZE_DISARMED;
            p91.nav_mode = (byte)(byte)123;
            p91.aux4 = (float) -2.739039E38F;
            p91.yaw_rudder = (float) -7.24688E37F;
            SMP_TEST_CH.send(p91);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_RC_INPUTS_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)27192);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)38663);
                Debug.Assert(pack.chan12_raw == (ushort)(ushort)54532);
                Debug.Assert(pack.rssi == (byte)(byte)146);
                Debug.Assert(pack.time_usec == (ulong)7608031505817144075L);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)11889);
                Debug.Assert(pack.chan11_raw == (ushort)(ushort)1962);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)14652);
                Debug.Assert(pack.chan9_raw == (ushort)(ushort)56882);
                Debug.Assert(pack.chan10_raw == (ushort)(ushort)1009);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)7017);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)60038);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)34186);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)18720);
            };
            HIL_RC_INPUTS_RAW p92 = new HIL_RC_INPUTS_RAW();
            PH.setPack(p92);
            p92.chan12_raw = (ushort)(ushort)54532;
            p92.rssi = (byte)(byte)146;
            p92.chan2_raw = (ushort)(ushort)18720;
            p92.time_usec = (ulong)7608031505817144075L;
            p92.chan8_raw = (ushort)(ushort)11889;
            p92.chan11_raw = (ushort)(ushort)1962;
            p92.chan6_raw = (ushort)(ushort)14652;
            p92.chan4_raw = (ushort)(ushort)7017;
            p92.chan9_raw = (ushort)(ushort)56882;
            p92.chan10_raw = (ushort)(ushort)1009;
            p92.chan3_raw = (ushort)(ushort)60038;
            p92.chan1_raw = (ushort)(ushort)27192;
            p92.chan5_raw = (ushort)(ushort)38663;
            p92.chan7_raw = (ushort)(ushort)34186;
            SMP_TEST_CH.send(p92);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_ACTUATOR_CONTROLSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.controls.SequenceEqual(new float[] {2.3451992E38F, 8.040168E37F, 2.0786121E38F, -1.0119437E38F, -1.5983205E38F, -2.826415E38F, 2.4135927E38F, 1.6270241E38F, -2.1174564E38F, 2.536368E38F, 2.7263893E38F, -2.7886983E38F, 2.354866E38F, -1.1195361E38F, -3.0847035E37F, -1.6601405E38F}));
                Debug.Assert(pack.mode == MAV_MODE.MAV_MODE_GUIDED_ARMED);
                Debug.Assert(pack.time_usec == (ulong)2444910713527807692L);
                Debug.Assert(pack.flags == (ulong)5593546100451500883L);
            };
            HIL_ACTUATOR_CONTROLS p93 = new HIL_ACTUATOR_CONTROLS();
            PH.setPack(p93);
            p93.flags = (ulong)5593546100451500883L;
            p93.time_usec = (ulong)2444910713527807692L;
            p93.controls_SET(new float[] {2.3451992E38F, 8.040168E37F, 2.0786121E38F, -1.0119437E38F, -1.5983205E38F, -2.826415E38F, 2.4135927E38F, 1.6270241E38F, -2.1174564E38F, 2.536368E38F, 2.7263893E38F, -2.7886983E38F, 2.354866E38F, -1.1195361E38F, -3.0847035E37F, -1.6601405E38F}, 0) ;
            p93.mode = MAV_MODE.MAV_MODE_GUIDED_ARMED;
            SMP_TEST_CH.send(p93);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnOPTICAL_FLOWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.flow_rate_y_TRY(ph) == (float) -9.97095E37F);
                Debug.Assert(pack.time_usec == (ulong)2604528233196007841L);
                Debug.Assert(pack.flow_comp_m_y == (float)3.2590245E37F);
                Debug.Assert(pack.ground_distance == (float) -2.094945E38F);
                Debug.Assert(pack.flow_comp_m_x == (float)9.406706E37F);
                Debug.Assert(pack.flow_y == (short)(short) -10402);
                Debug.Assert(pack.flow_rate_x_TRY(ph) == (float)1.3496717E38F);
                Debug.Assert(pack.sensor_id == (byte)(byte)38);
                Debug.Assert(pack.quality == (byte)(byte)114);
                Debug.Assert(pack.flow_x == (short)(short)8018);
            };
            OPTICAL_FLOW p100 = new OPTICAL_FLOW();
            PH.setPack(p100);
            p100.flow_comp_m_x = (float)9.406706E37F;
            p100.flow_comp_m_y = (float)3.2590245E37F;
            p100.flow_rate_x_SET((float)1.3496717E38F, PH) ;
            p100.flow_y = (short)(short) -10402;
            p100.sensor_id = (byte)(byte)38;
            p100.quality = (byte)(byte)114;
            p100.time_usec = (ulong)2604528233196007841L;
            p100.flow_rate_y_SET((float) -9.97095E37F, PH) ;
            p100.ground_distance = (float) -2.094945E38F;
            p100.flow_x = (short)(short)8018;
            SMP_TEST_CH.send(p100);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGLOBAL_VISION_POSITION_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.y == (float) -2.970685E38F);
                Debug.Assert(pack.pitch == (float) -2.5277607E37F);
                Debug.Assert(pack.yaw == (float)1.4610505E38F);
                Debug.Assert(pack.usec == (ulong)1813553190745343231L);
                Debug.Assert(pack.roll == (float)1.3029783E38F);
                Debug.Assert(pack.z == (float) -2.5948546E38F);
                Debug.Assert(pack.x == (float) -3.0157335E38F);
            };
            GLOBAL_VISION_POSITION_ESTIMATE p101 = new GLOBAL_VISION_POSITION_ESTIMATE();
            PH.setPack(p101);
            p101.yaw = (float)1.4610505E38F;
            p101.y = (float) -2.970685E38F;
            p101.z = (float) -2.5948546E38F;
            p101.usec = (ulong)1813553190745343231L;
            p101.roll = (float)1.3029783E38F;
            p101.x = (float) -3.0157335E38F;
            p101.pitch = (float) -2.5277607E37F;
            SMP_TEST_CH.send(p101);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnVISION_POSITION_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.x == (float) -3.0118942E38F);
                Debug.Assert(pack.z == (float) -1.7804911E37F);
                Debug.Assert(pack.usec == (ulong)3693596908237885002L);
                Debug.Assert(pack.yaw == (float) -2.426427E38F);
                Debug.Assert(pack.y == (float) -2.7073297E38F);
                Debug.Assert(pack.pitch == (float) -1.1874082E38F);
                Debug.Assert(pack.roll == (float)1.2381855E38F);
            };
            VISION_POSITION_ESTIMATE p102 = new VISION_POSITION_ESTIMATE();
            PH.setPack(p102);
            p102.pitch = (float) -1.1874082E38F;
            p102.usec = (ulong)3693596908237885002L;
            p102.z = (float) -1.7804911E37F;
            p102.roll = (float)1.2381855E38F;
            p102.x = (float) -3.0118942E38F;
            p102.yaw = (float) -2.426427E38F;
            p102.y = (float) -2.7073297E38F;
            SMP_TEST_CH.send(p102);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnVISION_SPEED_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.z == (float) -6.4913304E37F);
                Debug.Assert(pack.y == (float) -1.8455228E38F);
                Debug.Assert(pack.usec == (ulong)4636894691873977782L);
                Debug.Assert(pack.x == (float)1.7865152E38F);
            };
            GroundControl.VISION_SPEED_ESTIMATE p103 = CommunicationChannel.new_VISION_SPEED_ESTIMATE();
            PH.setPack(p103);
            p103.usec = (ulong)4636894691873977782L;
            p103.y = (float) -1.8455228E38F;
            p103.z = (float) -6.4913304E37F;
            p103.x = (float)1.7865152E38F;
            CommunicationChannel.instance.send(p103);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnVICON_POSITION_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.yaw == (float)2.9011247E38F);
                Debug.Assert(pack.x == (float)2.8069575E38F);
                Debug.Assert(pack.roll == (float) -3.1672419E38F);
                Debug.Assert(pack.z == (float) -2.496464E38F);
                Debug.Assert(pack.y == (float)1.4454929E38F);
                Debug.Assert(pack.pitch == (float)1.1368949E38F);
                Debug.Assert(pack.usec == (ulong)3107330380485455432L);
            };
            GroundControl.VICON_POSITION_ESTIMATE p104 = CommunicationChannel.new_VICON_POSITION_ESTIMATE();
            PH.setPack(p104);
            p104.roll = (float) -3.1672419E38F;
            p104.yaw = (float)2.9011247E38F;
            p104.pitch = (float)1.1368949E38F;
            p104.y = (float)1.4454929E38F;
            p104.x = (float)2.8069575E38F;
            p104.usec = (ulong)3107330380485455432L;
            p104.z = (float) -2.496464E38F;
            CommunicationChannel.instance.send(p104);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIGHRES_IMUReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.xmag == (float)1.521504E38F);
                Debug.Assert(pack.yacc == (float) -1.3476902E38F);
                Debug.Assert(pack.pressure_alt == (float)1.0017846E38F);
                Debug.Assert(pack.abs_pressure == (float) -1.5714984E38F);
                Debug.Assert(pack.zacc == (float)1.0642365E38F);
                Debug.Assert(pack.ygyro == (float)2.1918318E38F);
                Debug.Assert(pack.xgyro == (float)2.1606082E38F);
                Debug.Assert(pack.diff_pressure == (float) -3.200523E37F);
                Debug.Assert(pack.xacc == (float) -1.94535E38F);
                Debug.Assert(pack.zmag == (float) -3.0577408E38F);
                Debug.Assert(pack.fields_updated == (ushort)(ushort)49388);
                Debug.Assert(pack.temperature == (float) -1.1902931E38F);
                Debug.Assert(pack.ymag == (float) -2.089428E38F);
                Debug.Assert(pack.time_usec == (ulong)8476342554897018517L);
                Debug.Assert(pack.zgyro == (float) -3.0481768E38F);
            };
            GroundControl.HIGHRES_IMU p105 = CommunicationChannel.new_HIGHRES_IMU();
            PH.setPack(p105);
            p105.temperature = (float) -1.1902931E38F;
            p105.zgyro = (float) -3.0481768E38F;
            p105.time_usec = (ulong)8476342554897018517L;
            p105.ygyro = (float)2.1918318E38F;
            p105.fields_updated = (ushort)(ushort)49388;
            p105.abs_pressure = (float) -1.5714984E38F;
            p105.pressure_alt = (float)1.0017846E38F;
            p105.xmag = (float)1.521504E38F;
            p105.zmag = (float) -3.0577408E38F;
            p105.ymag = (float) -2.089428E38F;
            p105.zacc = (float)1.0642365E38F;
            p105.xacc = (float) -1.94535E38F;
            p105.xgyro = (float)2.1606082E38F;
            p105.diff_pressure = (float) -3.200523E37F;
            p105.yacc = (float) -1.3476902E38F;
            CommunicationChannel.instance.send(p105);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnOPTICAL_FLOW_RADReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sensor_id == (byte)(byte)147);
                Debug.Assert(pack.quality == (byte)(byte)168);
                Debug.Assert(pack.integrated_xgyro == (float) -2.796735E38F);
                Debug.Assert(pack.integrated_y == (float)1.5656649E38F);
                Debug.Assert(pack.integrated_zgyro == (float) -1.6731372E38F);
                Debug.Assert(pack.time_delta_distance_us == (uint)801722755U);
                Debug.Assert(pack.integration_time_us == (uint)2685026252U);
                Debug.Assert(pack.temperature == (short)(short)5508);
                Debug.Assert(pack.time_usec == (ulong)1157064546246343410L);
                Debug.Assert(pack.distance == (float) -1.0261655E37F);
                Debug.Assert(pack.integrated_x == (float)2.6481515E38F);
                Debug.Assert(pack.integrated_ygyro == (float)1.3033938E38F);
            };
            GroundControl.OPTICAL_FLOW_RAD p106 = CommunicationChannel.new_OPTICAL_FLOW_RAD();
            PH.setPack(p106);
            p106.integrated_x = (float)2.6481515E38F;
            p106.distance = (float) -1.0261655E37F;
            p106.sensor_id = (byte)(byte)147;
            p106.temperature = (short)(short)5508;
            p106.time_delta_distance_us = (uint)801722755U;
            p106.time_usec = (ulong)1157064546246343410L;
            p106.integrated_y = (float)1.5656649E38F;
            p106.quality = (byte)(byte)168;
            p106.integrated_ygyro = (float)1.3033938E38F;
            p106.integrated_xgyro = (float) -2.796735E38F;
            p106.integrated_zgyro = (float) -1.6731372E38F;
            p106.integration_time_us = (uint)2685026252U;
            CommunicationChannel.instance.send(p106);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_SENSORReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.xmag == (float) -2.7993263E38F);
                Debug.Assert(pack.zgyro == (float) -2.746207E38F);
                Debug.Assert(pack.ymag == (float)2.0775418E38F);
                Debug.Assert(pack.xacc == (float) -2.0842263E38F);
                Debug.Assert(pack.diff_pressure == (float)1.2055049E38F);
                Debug.Assert(pack.pressure_alt == (float) -3.3328437E38F);
                Debug.Assert(pack.temperature == (float)1.6126337E38F);
                Debug.Assert(pack.zacc == (float) -1.6006114E38F);
                Debug.Assert(pack.zmag == (float)2.1816775E36F);
                Debug.Assert(pack.abs_pressure == (float) -2.0118033E38F);
                Debug.Assert(pack.time_usec == (ulong)3036310021747183173L);
                Debug.Assert(pack.fields_updated == (uint)521211771U);
                Debug.Assert(pack.yacc == (float) -5.8285794E37F);
                Debug.Assert(pack.ygyro == (float)3.1556851E38F);
                Debug.Assert(pack.xgyro == (float) -5.5621765E37F);
            };
            GroundControl.HIL_SENSOR p107 = CommunicationChannel.new_HIL_SENSOR();
            PH.setPack(p107);
            p107.ygyro = (float)3.1556851E38F;
            p107.zmag = (float)2.1816775E36F;
            p107.pressure_alt = (float) -3.3328437E38F;
            p107.time_usec = (ulong)3036310021747183173L;
            p107.xgyro = (float) -5.5621765E37F;
            p107.temperature = (float)1.6126337E38F;
            p107.abs_pressure = (float) -2.0118033E38F;
            p107.xmag = (float) -2.7993263E38F;
            p107.diff_pressure = (float)1.2055049E38F;
            p107.ymag = (float)2.0775418E38F;
            p107.yacc = (float) -5.8285794E37F;
            p107.zgyro = (float) -2.746207E38F;
            p107.fields_updated = (uint)521211771U;
            p107.zacc = (float) -1.6006114E38F;
            p107.xacc = (float) -2.0842263E38F;
            CommunicationChannel.instance.send(p107);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSIM_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lat == (float) -9.681331E37F);
                Debug.Assert(pack.zgyro == (float) -3.0562873E38F);
                Debug.Assert(pack.zacc == (float)7.3351137E37F);
                Debug.Assert(pack.pitch == (float) -3.25711E38F);
                Debug.Assert(pack.vd == (float)9.470259E37F);
                Debug.Assert(pack.std_dev_vert == (float) -1.2140551E38F);
                Debug.Assert(pack.q1 == (float)2.4156799E37F);
                Debug.Assert(pack.ygyro == (float)8.0569185E37F);
                Debug.Assert(pack.q3 == (float) -1.0658476E38F);
                Debug.Assert(pack.q4 == (float) -9.131111E37F);
                Debug.Assert(pack.vn == (float) -1.8257296E38F);
                Debug.Assert(pack.alt == (float)9.227553E37F);
                Debug.Assert(pack.lon == (float) -1.531833E38F);
                Debug.Assert(pack.std_dev_horz == (float)1.505058E38F);
                Debug.Assert(pack.yaw == (float) -3.6666986E37F);
                Debug.Assert(pack.yacc == (float) -1.0694566E38F);
                Debug.Assert(pack.q2 == (float)3.2001205E38F);
                Debug.Assert(pack.roll == (float)2.779264E38F);
                Debug.Assert(pack.xgyro == (float)2.0880814E37F);
                Debug.Assert(pack.xacc == (float)1.0272464E38F);
                Debug.Assert(pack.ve == (float) -1.5794868E38F);
            };
            GroundControl.SIM_STATE p108 = CommunicationChannel.new_SIM_STATE();
            PH.setPack(p108);
            p108.q4 = (float) -9.131111E37F;
            p108.zacc = (float)7.3351137E37F;
            p108.lon = (float) -1.531833E38F;
            p108.zgyro = (float) -3.0562873E38F;
            p108.xgyro = (float)2.0880814E37F;
            p108.yacc = (float) -1.0694566E38F;
            p108.xacc = (float)1.0272464E38F;
            p108.ve = (float) -1.5794868E38F;
            p108.q2 = (float)3.2001205E38F;
            p108.std_dev_horz = (float)1.505058E38F;
            p108.yaw = (float) -3.6666986E37F;
            p108.roll = (float)2.779264E38F;
            p108.lat = (float) -9.681331E37F;
            p108.std_dev_vert = (float) -1.2140551E38F;
            p108.vn = (float) -1.8257296E38F;
            p108.q3 = (float) -1.0658476E38F;
            p108.vd = (float)9.470259E37F;
            p108.ygyro = (float)8.0569185E37F;
            p108.q1 = (float)2.4156799E37F;
            p108.pitch = (float) -3.25711E38F;
            p108.alt = (float)9.227553E37F;
            CommunicationChannel.instance.send(p108);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRADIO_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.remnoise == (byte)(byte)196);
                Debug.Assert(pack.rxerrors == (ushort)(ushort)38557);
                Debug.Assert(pack.fixed_ == (ushort)(ushort)15504);
                Debug.Assert(pack.remrssi == (byte)(byte)178);
                Debug.Assert(pack.noise == (byte)(byte)245);
                Debug.Assert(pack.rssi == (byte)(byte)115);
                Debug.Assert(pack.txbuf == (byte)(byte)100);
            };
            GroundControl.RADIO_STATUS p109 = CommunicationChannel.new_RADIO_STATUS();
            PH.setPack(p109);
            p109.remrssi = (byte)(byte)178;
            p109.noise = (byte)(byte)245;
            p109.rxerrors = (ushort)(ushort)38557;
            p109.rssi = (byte)(byte)115;
            p109.fixed_ = (ushort)(ushort)15504;
            p109.txbuf = (byte)(byte)100;
            p109.remnoise = (byte)(byte)196;
            CommunicationChannel.instance.send(p109);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnFILE_TRANSFER_PROTOCOLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_network == (byte)(byte)200);
                Debug.Assert(pack.payload.SequenceEqual(new byte[] {(byte)191, (byte)20, (byte)0, (byte)205, (byte)96, (byte)233, (byte)147, (byte)216, (byte)231, (byte)165, (byte)85, (byte)239, (byte)42, (byte)19, (byte)59, (byte)209, (byte)225, (byte)99, (byte)120, (byte)159, (byte)70, (byte)191, (byte)170, (byte)242, (byte)207, (byte)98, (byte)136, (byte)59, (byte)141, (byte)81, (byte)157, (byte)69, (byte)58, (byte)46, (byte)178, (byte)135, (byte)113, (byte)118, (byte)102, (byte)203, (byte)21, (byte)65, (byte)212, (byte)186, (byte)122, (byte)49, (byte)78, (byte)188, (byte)44, (byte)41, (byte)179, (byte)104, (byte)17, (byte)221, (byte)141, (byte)32, (byte)92, (byte)6, (byte)227, (byte)90, (byte)47, (byte)96, (byte)218, (byte)242, (byte)171, (byte)42, (byte)33, (byte)119, (byte)135, (byte)35, (byte)154, (byte)209, (byte)250, (byte)209, (byte)171, (byte)231, (byte)228, (byte)203, (byte)227, (byte)122, (byte)126, (byte)56, (byte)152, (byte)113, (byte)43, (byte)219, (byte)182, (byte)100, (byte)229, (byte)18, (byte)233, (byte)188, (byte)133, (byte)110, (byte)48, (byte)254, (byte)218, (byte)25, (byte)138, (byte)182, (byte)39, (byte)245, (byte)119, (byte)96, (byte)216, (byte)2, (byte)193, (byte)93, (byte)86, (byte)245, (byte)60, (byte)172, (byte)165, (byte)121, (byte)216, (byte)77, (byte)105, (byte)146, (byte)132, (byte)235, (byte)187, (byte)147, (byte)94, (byte)241, (byte)115, (byte)188, (byte)57, (byte)31, (byte)210, (byte)10, (byte)106, (byte)46, (byte)11, (byte)58, (byte)189, (byte)254, (byte)69, (byte)152, (byte)187, (byte)193, (byte)6, (byte)30, (byte)220, (byte)222, (byte)245, (byte)68, (byte)117, (byte)142, (byte)60, (byte)103, (byte)100, (byte)17, (byte)58, (byte)172, (byte)187, (byte)12, (byte)22, (byte)99, (byte)105, (byte)204, (byte)52, (byte)210, (byte)207, (byte)245, (byte)118, (byte)243, (byte)245, (byte)165, (byte)60, (byte)1, (byte)42, (byte)1, (byte)252, (byte)20, (byte)103, (byte)98, (byte)225, (byte)1, (byte)231, (byte)240, (byte)237, (byte)186, (byte)156, (byte)53, (byte)170, (byte)86, (byte)156, (byte)220, (byte)0, (byte)11, (byte)167, (byte)77, (byte)91, (byte)101, (byte)248, (byte)63, (byte)243, (byte)132, (byte)24, (byte)151, (byte)132, (byte)70, (byte)243, (byte)248, (byte)140, (byte)104, (byte)48, (byte)60, (byte)250, (byte)137, (byte)39, (byte)207, (byte)245, (byte)128, (byte)56, (byte)49, (byte)109, (byte)134, (byte)175, (byte)237, (byte)30, (byte)212, (byte)120, (byte)162, (byte)247, (byte)201, (byte)93, (byte)94, (byte)190, (byte)123, (byte)156, (byte)61, (byte)138, (byte)159, (byte)33, (byte)225, (byte)16, (byte)113, (byte)183, (byte)154, (byte)225, (byte)197, (byte)108, (byte)13, (byte)20, (byte)129, (byte)229, (byte)221, (byte)94, (byte)204, (byte)170}));
                Debug.Assert(pack.target_component == (byte)(byte)220);
                Debug.Assert(pack.target_system == (byte)(byte)23);
            };
            GroundControl.FILE_TRANSFER_PROTOCOL p110 = CommunicationChannel.new_FILE_TRANSFER_PROTOCOL();
            PH.setPack(p110);
            p110.payload_SET(new byte[] {(byte)191, (byte)20, (byte)0, (byte)205, (byte)96, (byte)233, (byte)147, (byte)216, (byte)231, (byte)165, (byte)85, (byte)239, (byte)42, (byte)19, (byte)59, (byte)209, (byte)225, (byte)99, (byte)120, (byte)159, (byte)70, (byte)191, (byte)170, (byte)242, (byte)207, (byte)98, (byte)136, (byte)59, (byte)141, (byte)81, (byte)157, (byte)69, (byte)58, (byte)46, (byte)178, (byte)135, (byte)113, (byte)118, (byte)102, (byte)203, (byte)21, (byte)65, (byte)212, (byte)186, (byte)122, (byte)49, (byte)78, (byte)188, (byte)44, (byte)41, (byte)179, (byte)104, (byte)17, (byte)221, (byte)141, (byte)32, (byte)92, (byte)6, (byte)227, (byte)90, (byte)47, (byte)96, (byte)218, (byte)242, (byte)171, (byte)42, (byte)33, (byte)119, (byte)135, (byte)35, (byte)154, (byte)209, (byte)250, (byte)209, (byte)171, (byte)231, (byte)228, (byte)203, (byte)227, (byte)122, (byte)126, (byte)56, (byte)152, (byte)113, (byte)43, (byte)219, (byte)182, (byte)100, (byte)229, (byte)18, (byte)233, (byte)188, (byte)133, (byte)110, (byte)48, (byte)254, (byte)218, (byte)25, (byte)138, (byte)182, (byte)39, (byte)245, (byte)119, (byte)96, (byte)216, (byte)2, (byte)193, (byte)93, (byte)86, (byte)245, (byte)60, (byte)172, (byte)165, (byte)121, (byte)216, (byte)77, (byte)105, (byte)146, (byte)132, (byte)235, (byte)187, (byte)147, (byte)94, (byte)241, (byte)115, (byte)188, (byte)57, (byte)31, (byte)210, (byte)10, (byte)106, (byte)46, (byte)11, (byte)58, (byte)189, (byte)254, (byte)69, (byte)152, (byte)187, (byte)193, (byte)6, (byte)30, (byte)220, (byte)222, (byte)245, (byte)68, (byte)117, (byte)142, (byte)60, (byte)103, (byte)100, (byte)17, (byte)58, (byte)172, (byte)187, (byte)12, (byte)22, (byte)99, (byte)105, (byte)204, (byte)52, (byte)210, (byte)207, (byte)245, (byte)118, (byte)243, (byte)245, (byte)165, (byte)60, (byte)1, (byte)42, (byte)1, (byte)252, (byte)20, (byte)103, (byte)98, (byte)225, (byte)1, (byte)231, (byte)240, (byte)237, (byte)186, (byte)156, (byte)53, (byte)170, (byte)86, (byte)156, (byte)220, (byte)0, (byte)11, (byte)167, (byte)77, (byte)91, (byte)101, (byte)248, (byte)63, (byte)243, (byte)132, (byte)24, (byte)151, (byte)132, (byte)70, (byte)243, (byte)248, (byte)140, (byte)104, (byte)48, (byte)60, (byte)250, (byte)137, (byte)39, (byte)207, (byte)245, (byte)128, (byte)56, (byte)49, (byte)109, (byte)134, (byte)175, (byte)237, (byte)30, (byte)212, (byte)120, (byte)162, (byte)247, (byte)201, (byte)93, (byte)94, (byte)190, (byte)123, (byte)156, (byte)61, (byte)138, (byte)159, (byte)33, (byte)225, (byte)16, (byte)113, (byte)183, (byte)154, (byte)225, (byte)197, (byte)108, (byte)13, (byte)20, (byte)129, (byte)229, (byte)221, (byte)94, (byte)204, (byte)170}, 0) ;
            p110.target_component = (byte)(byte)220;
            p110.target_network = (byte)(byte)200;
            p110.target_system = (byte)(byte)23;
            CommunicationChannel.instance.send(p110);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTIMESYNCReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.tc1 == (long)2069652375342323635L);
                Debug.Assert(pack.ts1 == (long)5891088617928815088L);
            };
            GroundControl.TIMESYNC p111 = CommunicationChannel.new_TIMESYNC();
            PH.setPack(p111);
            p111.tc1 = (long)2069652375342323635L;
            p111.ts1 = (long)5891088617928815088L;
            CommunicationChannel.instance.send(p111);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCAMERA_TRIGGERReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)3728730893453260126L);
                Debug.Assert(pack.seq == (uint)2860936380U);
            };
            GroundControl.CAMERA_TRIGGER p112 = CommunicationChannel.new_CAMERA_TRIGGER();
            PH.setPack(p112);
            p112.seq = (uint)2860936380U;
            p112.time_usec = (ulong)3728730893453260126L;
            CommunicationChannel.instance.send(p112);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_GPSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ve == (short)(short)1175);
                Debug.Assert(pack.lat == (int)1020434379);
                Debug.Assert(pack.cog == (ushort)(ushort)63871);
                Debug.Assert(pack.vn == (short)(short)5934);
                Debug.Assert(pack.eph == (ushort)(ushort)27979);
                Debug.Assert(pack.vel == (ushort)(ushort)20731);
                Debug.Assert(pack.time_usec == (ulong)3428303985216765709L);
                Debug.Assert(pack.satellites_visible == (byte)(byte)24);
                Debug.Assert(pack.vd == (short)(short) -12393);
                Debug.Assert(pack.fix_type == (byte)(byte)77);
                Debug.Assert(pack.epv == (ushort)(ushort)28617);
                Debug.Assert(pack.alt == (int)1347487552);
                Debug.Assert(pack.lon == (int) -1932763010);
            };
            GroundControl.HIL_GPS p113 = CommunicationChannel.new_HIL_GPS();
            PH.setPack(p113);
            p113.lon = (int) -1932763010;
            p113.cog = (ushort)(ushort)63871;
            p113.eph = (ushort)(ushort)27979;
            p113.vn = (short)(short)5934;
            p113.time_usec = (ulong)3428303985216765709L;
            p113.vd = (short)(short) -12393;
            p113.ve = (short)(short)1175;
            p113.lat = (int)1020434379;
            p113.vel = (ushort)(ushort)20731;
            p113.alt = (int)1347487552;
            p113.epv = (ushort)(ushort)28617;
            p113.fix_type = (byte)(byte)77;
            p113.satellites_visible = (byte)(byte)24;
            CommunicationChannel.instance.send(p113);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_OPTICAL_FLOWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)581830020692549949L);
                Debug.Assert(pack.integration_time_us == (uint)2539976368U);
                Debug.Assert(pack.integrated_y == (float)1.9980445E38F);
                Debug.Assert(pack.temperature == (short)(short)1401);
                Debug.Assert(pack.integrated_ygyro == (float) -2.6894637E38F);
                Debug.Assert(pack.integrated_zgyro == (float)3.3463306E37F);
                Debug.Assert(pack.integrated_x == (float)1.8640275E38F);
                Debug.Assert(pack.quality == (byte)(byte)200);
                Debug.Assert(pack.distance == (float)2.6709224E38F);
                Debug.Assert(pack.integrated_xgyro == (float) -1.3878216E38F);
                Debug.Assert(pack.time_delta_distance_us == (uint)843900006U);
                Debug.Assert(pack.sensor_id == (byte)(byte)9);
            };
            GroundControl.HIL_OPTICAL_FLOW p114 = CommunicationChannel.new_HIL_OPTICAL_FLOW();
            PH.setPack(p114);
            p114.integration_time_us = (uint)2539976368U;
            p114.integrated_x = (float)1.8640275E38F;
            p114.integrated_y = (float)1.9980445E38F;
            p114.integrated_xgyro = (float) -1.3878216E38F;
            p114.temperature = (short)(short)1401;
            p114.sensor_id = (byte)(byte)9;
            p114.integrated_ygyro = (float) -2.6894637E38F;
            p114.time_usec = (ulong)581830020692549949L;
            p114.quality = (byte)(byte)200;
            p114.integrated_zgyro = (float)3.3463306E37F;
            p114.time_delta_distance_us = (uint)843900006U;
            p114.distance = (float)2.6709224E38F;
            CommunicationChannel.instance.send(p114);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_STATE_QUATERNIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)2182083464254567278L);
                Debug.Assert(pack.lat == (int) -1424694775);
                Debug.Assert(pack.pitchspeed == (float) -2.6438285E38F);
                Debug.Assert(pack.rollspeed == (float) -1.7163158E38F);
                Debug.Assert(pack.true_airspeed == (ushort)(ushort)13004);
                Debug.Assert(pack.alt == (int) -154243418);
                Debug.Assert(pack.vz == (short)(short)26904);
                Debug.Assert(pack.yawspeed == (float) -2.7176068E38F);
                Debug.Assert(pack.vx == (short)(short)26506);
                Debug.Assert(pack.zacc == (short)(short)31458);
                Debug.Assert(pack.xacc == (short)(short) -32337);
                Debug.Assert(pack.attitude_quaternion.SequenceEqual(new float[] {1.8993489E38F, 3.1652952E38F, -9.209403E37F, -4.6945787E37F}));
                Debug.Assert(pack.yacc == (short)(short)5180);
                Debug.Assert(pack.ind_airspeed == (ushort)(ushort)20160);
                Debug.Assert(pack.vy == (short)(short)22858);
                Debug.Assert(pack.lon == (int)120420085);
            };
            GroundControl.HIL_STATE_QUATERNION p115 = CommunicationChannel.new_HIL_STATE_QUATERNION();
            PH.setPack(p115);
            p115.lat = (int) -1424694775;
            p115.true_airspeed = (ushort)(ushort)13004;
            p115.vy = (short)(short)22858;
            p115.vx = (short)(short)26506;
            p115.attitude_quaternion_SET(new float[] {1.8993489E38F, 3.1652952E38F, -9.209403E37F, -4.6945787E37F}, 0) ;
            p115.yacc = (short)(short)5180;
            p115.lon = (int)120420085;
            p115.xacc = (short)(short) -32337;
            p115.ind_airspeed = (ushort)(ushort)20160;
            p115.zacc = (short)(short)31458;
            p115.yawspeed = (float) -2.7176068E38F;
            p115.alt = (int) -154243418;
            p115.pitchspeed = (float) -2.6438285E38F;
            p115.vz = (short)(short)26904;
            p115.time_usec = (ulong)2182083464254567278L;
            p115.rollspeed = (float) -1.7163158E38F;
            CommunicationChannel.instance.send(p115);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_IMU2Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)3022500215U);
                Debug.Assert(pack.xmag == (short)(short) -24540);
                Debug.Assert(pack.yacc == (short)(short) -24158);
                Debug.Assert(pack.ygyro == (short)(short)13417);
                Debug.Assert(pack.zacc == (short)(short)14726);
                Debug.Assert(pack.xgyro == (short)(short)26360);
                Debug.Assert(pack.ymag == (short)(short) -32664);
                Debug.Assert(pack.xacc == (short)(short) -8970);
                Debug.Assert(pack.zgyro == (short)(short) -29216);
                Debug.Assert(pack.zmag == (short)(short) -31149);
            };
            GroundControl.SCALED_IMU2 p116 = CommunicationChannel.new_SCALED_IMU2();
            PH.setPack(p116);
            p116.xmag = (short)(short) -24540;
            p116.ymag = (short)(short) -32664;
            p116.zacc = (short)(short)14726;
            p116.ygyro = (short)(short)13417;
            p116.time_boot_ms = (uint)3022500215U;
            p116.xacc = (short)(short) -8970;
            p116.yacc = (short)(short) -24158;
            p116.xgyro = (short)(short)26360;
            p116.zgyro = (short)(short) -29216;
            p116.zmag = (short)(short) -31149;
            CommunicationChannel.instance.send(p116);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.start == (ushort)(ushort)20472);
                Debug.Assert(pack.target_system == (byte)(byte)36);
                Debug.Assert(pack.target_component == (byte)(byte)30);
                Debug.Assert(pack.end == (ushort)(ushort)14628);
            };
            GroundControl.LOG_REQUEST_LIST p117 = CommunicationChannel.new_LOG_REQUEST_LIST();
            PH.setPack(p117);
            p117.target_system = (byte)(byte)36;
            p117.end = (ushort)(ushort)14628;
            p117.target_component = (byte)(byte)30;
            p117.start = (ushort)(ushort)20472;
            CommunicationChannel.instance.send(p117);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_ENTRYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.last_log_num == (ushort)(ushort)40562);
                Debug.Assert(pack.id == (ushort)(ushort)18431);
                Debug.Assert(pack.time_utc == (uint)33526910U);
                Debug.Assert(pack.size == (uint)1741276395U);
                Debug.Assert(pack.num_logs == (ushort)(ushort)1040);
            };
            GroundControl.LOG_ENTRY p118 = CommunicationChannel.new_LOG_ENTRY();
            PH.setPack(p118);
            p118.num_logs = (ushort)(ushort)1040;
            p118.last_log_num = (ushort)(ushort)40562;
            p118.id = (ushort)(ushort)18431;
            p118.time_utc = (uint)33526910U;
            p118.size = (uint)1741276395U;
            CommunicationChannel.instance.send(p118);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_REQUEST_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ofs == (uint)9525910U);
                Debug.Assert(pack.target_component == (byte)(byte)252);
                Debug.Assert(pack.count == (uint)3525282743U);
                Debug.Assert(pack.id == (ushort)(ushort)29255);
                Debug.Assert(pack.target_system == (byte)(byte)133);
            };
            GroundControl.LOG_REQUEST_DATA p119 = CommunicationChannel.new_LOG_REQUEST_DATA();
            PH.setPack(p119);
            p119.count = (uint)3525282743U;
            p119.ofs = (uint)9525910U;
            p119.target_system = (byte)(byte)133;
            p119.id = (ushort)(ushort)29255;
            p119.target_component = (byte)(byte)252;
            CommunicationChannel.instance.send(p119);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ofs == (uint)3474918918U);
                Debug.Assert(pack.count == (byte)(byte)243);
                Debug.Assert(pack.id == (ushort)(ushort)13427);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)40, (byte)170, (byte)98, (byte)35, (byte)200, (byte)36, (byte)64, (byte)136, (byte)253, (byte)54, (byte)201, (byte)176, (byte)64, (byte)87, (byte)232, (byte)233, (byte)12, (byte)106, (byte)169, (byte)186, (byte)103, (byte)5, (byte)121, (byte)59, (byte)129, (byte)213, (byte)32, (byte)149, (byte)181, (byte)224, (byte)62, (byte)246, (byte)188, (byte)66, (byte)191, (byte)4, (byte)30, (byte)14, (byte)205, (byte)53, (byte)90, (byte)204, (byte)10, (byte)246, (byte)178, (byte)161, (byte)16, (byte)50, (byte)49, (byte)1, (byte)175, (byte)150, (byte)96, (byte)101, (byte)57, (byte)56, (byte)150, (byte)159, (byte)226, (byte)81, (byte)126, (byte)172, (byte)105, (byte)69, (byte)118, (byte)47, (byte)112, (byte)63, (byte)222, (byte)171, (byte)126, (byte)21, (byte)5, (byte)8, (byte)53, (byte)124, (byte)25, (byte)59, (byte)101, (byte)27, (byte)135, (byte)143, (byte)137, (byte)106, (byte)168, (byte)107, (byte)238, (byte)10, (byte)246, (byte)237}));
            };
            GroundControl.LOG_DATA p120 = CommunicationChannel.new_LOG_DATA();
            PH.setPack(p120);
            p120.id = (ushort)(ushort)13427;
            p120.ofs = (uint)3474918918U;
            p120.count = (byte)(byte)243;
            p120.data__SET(new byte[] {(byte)40, (byte)170, (byte)98, (byte)35, (byte)200, (byte)36, (byte)64, (byte)136, (byte)253, (byte)54, (byte)201, (byte)176, (byte)64, (byte)87, (byte)232, (byte)233, (byte)12, (byte)106, (byte)169, (byte)186, (byte)103, (byte)5, (byte)121, (byte)59, (byte)129, (byte)213, (byte)32, (byte)149, (byte)181, (byte)224, (byte)62, (byte)246, (byte)188, (byte)66, (byte)191, (byte)4, (byte)30, (byte)14, (byte)205, (byte)53, (byte)90, (byte)204, (byte)10, (byte)246, (byte)178, (byte)161, (byte)16, (byte)50, (byte)49, (byte)1, (byte)175, (byte)150, (byte)96, (byte)101, (byte)57, (byte)56, (byte)150, (byte)159, (byte)226, (byte)81, (byte)126, (byte)172, (byte)105, (byte)69, (byte)118, (byte)47, (byte)112, (byte)63, (byte)222, (byte)171, (byte)126, (byte)21, (byte)5, (byte)8, (byte)53, (byte)124, (byte)25, (byte)59, (byte)101, (byte)27, (byte)135, (byte)143, (byte)137, (byte)106, (byte)168, (byte)107, (byte)238, (byte)10, (byte)246, (byte)237}, 0) ;
            CommunicationChannel.instance.send(p120);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_ERASEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)207);
                Debug.Assert(pack.target_system == (byte)(byte)179);
            };
            GroundControl.LOG_ERASE p121 = CommunicationChannel.new_LOG_ERASE();
            PH.setPack(p121);
            p121.target_component = (byte)(byte)207;
            p121.target_system = (byte)(byte)179;
            CommunicationChannel.instance.send(p121);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_REQUEST_ENDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)121);
                Debug.Assert(pack.target_system == (byte)(byte)237);
            };
            GroundControl.LOG_REQUEST_END p122 = CommunicationChannel.new_LOG_REQUEST_END();
            PH.setPack(p122);
            p122.target_system = (byte)(byte)237;
            p122.target_component = (byte)(byte)121;
            CommunicationChannel.instance.send(p122);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_INJECT_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)173);
                Debug.Assert(pack.target_system == (byte)(byte)240);
                Debug.Assert(pack.len == (byte)(byte)99);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)211, (byte)45, (byte)50, (byte)157, (byte)247, (byte)202, (byte)252, (byte)57, (byte)181, (byte)213, (byte)23, (byte)37, (byte)21, (byte)132, (byte)103, (byte)213, (byte)41, (byte)68, (byte)177, (byte)136, (byte)43, (byte)90, (byte)220, (byte)2, (byte)237, (byte)89, (byte)167, (byte)41, (byte)254, (byte)96, (byte)94, (byte)159, (byte)181, (byte)16, (byte)210, (byte)107, (byte)43, (byte)3, (byte)245, (byte)76, (byte)138, (byte)166, (byte)212, (byte)191, (byte)221, (byte)120, (byte)54, (byte)179, (byte)238, (byte)199, (byte)131, (byte)149, (byte)144, (byte)131, (byte)17, (byte)220, (byte)167, (byte)178, (byte)109, (byte)73, (byte)20, (byte)54, (byte)123, (byte)33, (byte)75, (byte)137, (byte)123, (byte)46, (byte)19, (byte)193, (byte)212, (byte)90, (byte)24, (byte)21, (byte)10, (byte)132, (byte)214, (byte)228, (byte)175, (byte)92, (byte)76, (byte)131, (byte)212, (byte)228, (byte)50, (byte)129, (byte)82, (byte)114, (byte)58, (byte)94, (byte)211, (byte)183, (byte)201, (byte)37, (byte)128, (byte)44, (byte)147, (byte)117, (byte)101, (byte)182, (byte)134, (byte)200, (byte)73, (byte)196, (byte)237, (byte)96, (byte)223, (byte)158, (byte)129, (byte)204}));
            };
            GroundControl.GPS_INJECT_DATA p123 = CommunicationChannel.new_GPS_INJECT_DATA();
            PH.setPack(p123);
            p123.data__SET(new byte[] {(byte)211, (byte)45, (byte)50, (byte)157, (byte)247, (byte)202, (byte)252, (byte)57, (byte)181, (byte)213, (byte)23, (byte)37, (byte)21, (byte)132, (byte)103, (byte)213, (byte)41, (byte)68, (byte)177, (byte)136, (byte)43, (byte)90, (byte)220, (byte)2, (byte)237, (byte)89, (byte)167, (byte)41, (byte)254, (byte)96, (byte)94, (byte)159, (byte)181, (byte)16, (byte)210, (byte)107, (byte)43, (byte)3, (byte)245, (byte)76, (byte)138, (byte)166, (byte)212, (byte)191, (byte)221, (byte)120, (byte)54, (byte)179, (byte)238, (byte)199, (byte)131, (byte)149, (byte)144, (byte)131, (byte)17, (byte)220, (byte)167, (byte)178, (byte)109, (byte)73, (byte)20, (byte)54, (byte)123, (byte)33, (byte)75, (byte)137, (byte)123, (byte)46, (byte)19, (byte)193, (byte)212, (byte)90, (byte)24, (byte)21, (byte)10, (byte)132, (byte)214, (byte)228, (byte)175, (byte)92, (byte)76, (byte)131, (byte)212, (byte)228, (byte)50, (byte)129, (byte)82, (byte)114, (byte)58, (byte)94, (byte)211, (byte)183, (byte)201, (byte)37, (byte)128, (byte)44, (byte)147, (byte)117, (byte)101, (byte)182, (byte)134, (byte)200, (byte)73, (byte)196, (byte)237, (byte)96, (byte)223, (byte)158, (byte)129, (byte)204}, 0) ;
            p123.target_component = (byte)(byte)173;
            p123.target_system = (byte)(byte)240;
            p123.len = (byte)(byte)99;
            CommunicationChannel.instance.send(p123);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS2_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.cog == (ushort)(ushort)13256);
                Debug.Assert(pack.dgps_age == (uint)370656372U);
                Debug.Assert(pack.fix_type == GPS_FIX_TYPE.GPS_FIX_TYPE_NO_FIX);
                Debug.Assert(pack.lon == (int)2049379417);
                Debug.Assert(pack.alt == (int)936981521);
                Debug.Assert(pack.vel == (ushort)(ushort)53925);
                Debug.Assert(pack.satellites_visible == (byte)(byte)177);
                Debug.Assert(pack.dgps_numch == (byte)(byte)178);
                Debug.Assert(pack.epv == (ushort)(ushort)13830);
                Debug.Assert(pack.time_usec == (ulong)8253457257032424839L);
                Debug.Assert(pack.lat == (int)1303804377);
                Debug.Assert(pack.eph == (ushort)(ushort)28348);
            };
            GroundControl.GPS2_RAW p124 = CommunicationChannel.new_GPS2_RAW();
            PH.setPack(p124);
            p124.cog = (ushort)(ushort)13256;
            p124.lon = (int)2049379417;
            p124.dgps_age = (uint)370656372U;
            p124.eph = (ushort)(ushort)28348;
            p124.dgps_numch = (byte)(byte)178;
            p124.vel = (ushort)(ushort)53925;
            p124.lat = (int)1303804377;
            p124.fix_type = GPS_FIX_TYPE.GPS_FIX_TYPE_NO_FIX;
            p124.epv = (ushort)(ushort)13830;
            p124.time_usec = (ulong)8253457257032424839L;
            p124.satellites_visible = (byte)(byte)177;
            p124.alt = (int)936981521;
            CommunicationChannel.instance.send(p124);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPOWER_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.flags == (MAV_POWER_STATUS.MAV_POWER_STATUS_SERVO_VALID |
                                            MAV_POWER_STATUS.MAV_POWER_STATUS_USB_CONNECTED));
                Debug.Assert(pack.Vcc == (ushort)(ushort)6804);
                Debug.Assert(pack.Vservo == (ushort)(ushort)2451);
            };
            GroundControl.POWER_STATUS p125 = CommunicationChannel.new_POWER_STATUS();
            PH.setPack(p125);
            p125.Vservo = (ushort)(ushort)2451;
            p125.flags = (MAV_POWER_STATUS.MAV_POWER_STATUS_SERVO_VALID |
                          MAV_POWER_STATUS.MAV_POWER_STATUS_USB_CONNECTED);
            p125.Vcc = (ushort)(ushort)6804;
            CommunicationChannel.instance.send(p125);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSERIAL_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.timeout == (ushort)(ushort)41198);
                Debug.Assert(pack.flags == (SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_MULTI |
                                            SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_RESPOND |
                                            SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_BLOCKING));
                Debug.Assert(pack.device == SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_TELEM1);
                Debug.Assert(pack.baudrate == (uint)843851539U);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)152, (byte)148, (byte)236, (byte)29, (byte)126, (byte)245, (byte)75, (byte)28, (byte)19, (byte)105, (byte)118, (byte)78, (byte)36, (byte)65, (byte)73, (byte)30, (byte)228, (byte)142, (byte)97, (byte)254, (byte)110, (byte)182, (byte)188, (byte)226, (byte)78, (byte)85, (byte)58, (byte)97, (byte)193, (byte)197, (byte)247, (byte)203, (byte)101, (byte)246, (byte)85, (byte)33, (byte)60, (byte)183, (byte)219, (byte)254, (byte)177, (byte)4, (byte)78, (byte)254, (byte)146, (byte)113, (byte)24, (byte)27, (byte)16, (byte)118, (byte)76, (byte)255, (byte)103, (byte)228, (byte)249, (byte)154, (byte)14, (byte)20, (byte)65, (byte)239, (byte)164, (byte)153, (byte)224, (byte)83, (byte)80, (byte)17, (byte)154, (byte)158, (byte)250, (byte)56}));
                Debug.Assert(pack.count == (byte)(byte)195);
            };
            GroundControl.SERIAL_CONTROL p126 = CommunicationChannel.new_SERIAL_CONTROL();
            PH.setPack(p126);
            p126.baudrate = (uint)843851539U;
            p126.device = SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_TELEM1;
            p126.count = (byte)(byte)195;
            p126.timeout = (ushort)(ushort)41198;
            p126.data__SET(new byte[] {(byte)152, (byte)148, (byte)236, (byte)29, (byte)126, (byte)245, (byte)75, (byte)28, (byte)19, (byte)105, (byte)118, (byte)78, (byte)36, (byte)65, (byte)73, (byte)30, (byte)228, (byte)142, (byte)97, (byte)254, (byte)110, (byte)182, (byte)188, (byte)226, (byte)78, (byte)85, (byte)58, (byte)97, (byte)193, (byte)197, (byte)247, (byte)203, (byte)101, (byte)246, (byte)85, (byte)33, (byte)60, (byte)183, (byte)219, (byte)254, (byte)177, (byte)4, (byte)78, (byte)254, (byte)146, (byte)113, (byte)24, (byte)27, (byte)16, (byte)118, (byte)76, (byte)255, (byte)103, (byte)228, (byte)249, (byte)154, (byte)14, (byte)20, (byte)65, (byte)239, (byte)164, (byte)153, (byte)224, (byte)83, (byte)80, (byte)17, (byte)154, (byte)158, (byte)250, (byte)56}, 0) ;
            p126.flags = (SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_MULTI |
                          SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_RESPOND |
                          SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_BLOCKING);
            CommunicationChannel.instance.send(p126);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_RTKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.nsats == (byte)(byte)60);
                Debug.Assert(pack.rtk_health == (byte)(byte)29);
                Debug.Assert(pack.baseline_b_mm == (int) -1242347245);
                Debug.Assert(pack.rtk_rate == (byte)(byte)118);
                Debug.Assert(pack.wn == (ushort)(ushort)11243);
                Debug.Assert(pack.baseline_coords_type == (byte)(byte)117);
                Debug.Assert(pack.iar_num_hypotheses == (int)579721838);
                Debug.Assert(pack.rtk_receiver_id == (byte)(byte)85);
                Debug.Assert(pack.tow == (uint)2527722339U);
                Debug.Assert(pack.accuracy == (uint)1658400330U);
                Debug.Assert(pack.baseline_c_mm == (int) -2070554495);
                Debug.Assert(pack.time_last_baseline_ms == (uint)2033311364U);
                Debug.Assert(pack.baseline_a_mm == (int) -1765824790);
            };
            GroundControl.GPS_RTK p127 = CommunicationChannel.new_GPS_RTK();
            PH.setPack(p127);
            p127.tow = (uint)2527722339U;
            p127.baseline_coords_type = (byte)(byte)117;
            p127.iar_num_hypotheses = (int)579721838;
            p127.baseline_b_mm = (int) -1242347245;
            p127.rtk_health = (byte)(byte)29;
            p127.accuracy = (uint)1658400330U;
            p127.baseline_a_mm = (int) -1765824790;
            p127.baseline_c_mm = (int) -2070554495;
            p127.rtk_rate = (byte)(byte)118;
            p127.nsats = (byte)(byte)60;
            p127.time_last_baseline_ms = (uint)2033311364U;
            p127.rtk_receiver_id = (byte)(byte)85;
            p127.wn = (ushort)(ushort)11243;
            CommunicationChannel.instance.send(p127);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS2_RTKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.baseline_a_mm == (int) -289483023);
                Debug.Assert(pack.iar_num_hypotheses == (int) -908688935);
                Debug.Assert(pack.time_last_baseline_ms == (uint)345403025U);
                Debug.Assert(pack.baseline_coords_type == (byte)(byte)153);
                Debug.Assert(pack.accuracy == (uint)1560634479U);
                Debug.Assert(pack.wn == (ushort)(ushort)12553);
                Debug.Assert(pack.rtk_health == (byte)(byte)93);
                Debug.Assert(pack.baseline_b_mm == (int)544958296);
                Debug.Assert(pack.nsats == (byte)(byte)124);
                Debug.Assert(pack.rtk_receiver_id == (byte)(byte)8);
                Debug.Assert(pack.baseline_c_mm == (int) -1457405241);
                Debug.Assert(pack.tow == (uint)3665229533U);
                Debug.Assert(pack.rtk_rate == (byte)(byte)150);
            };
            GroundControl.GPS2_RTK p128 = CommunicationChannel.new_GPS2_RTK();
            PH.setPack(p128);
            p128.iar_num_hypotheses = (int) -908688935;
            p128.baseline_c_mm = (int) -1457405241;
            p128.wn = (ushort)(ushort)12553;
            p128.rtk_health = (byte)(byte)93;
            p128.time_last_baseline_ms = (uint)345403025U;
            p128.tow = (uint)3665229533U;
            p128.baseline_coords_type = (byte)(byte)153;
            p128.baseline_a_mm = (int) -289483023;
            p128.nsats = (byte)(byte)124;
            p128.baseline_b_mm = (int)544958296;
            p128.rtk_receiver_id = (byte)(byte)8;
            p128.accuracy = (uint)1560634479U;
            p128.rtk_rate = (byte)(byte)150;
            CommunicationChannel.instance.send(p128);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_IMU3Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)3943065434U);
                Debug.Assert(pack.xacc == (short)(short) -6634);
                Debug.Assert(pack.xmag == (short)(short) -16109);
                Debug.Assert(pack.zacc == (short)(short) -25403);
                Debug.Assert(pack.zgyro == (short)(short)2937);
                Debug.Assert(pack.yacc == (short)(short)25504);
                Debug.Assert(pack.ymag == (short)(short) -7343);
                Debug.Assert(pack.xgyro == (short)(short) -19085);
                Debug.Assert(pack.zmag == (short)(short)20729);
                Debug.Assert(pack.ygyro == (short)(short) -1365);
            };
            GroundControl.SCALED_IMU3 p129 = CommunicationChannel.new_SCALED_IMU3();
            PH.setPack(p129);
            p129.xmag = (short)(short) -16109;
            p129.xgyro = (short)(short) -19085;
            p129.yacc = (short)(short)25504;
            p129.xacc = (short)(short) -6634;
            p129.time_boot_ms = (uint)3943065434U;
            p129.zgyro = (short)(short)2937;
            p129.ygyro = (short)(short) -1365;
            p129.zmag = (short)(short)20729;
            p129.zacc = (short)(short) -25403;
            p129.ymag = (short)(short) -7343;
            CommunicationChannel.instance.send(p129);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnDATA_TRANSMISSION_HANDSHAKEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.height == (ushort)(ushort)24793);
                Debug.Assert(pack.type == (byte)(byte)19);
                Debug.Assert(pack.packets == (ushort)(ushort)23304);
                Debug.Assert(pack.width == (ushort)(ushort)36515);
                Debug.Assert(pack.jpg_quality == (byte)(byte)125);
                Debug.Assert(pack.size == (uint)82745406U);
                Debug.Assert(pack.payload == (byte)(byte)177);
            };
            GroundControl.DATA_TRANSMISSION_HANDSHAKE p130 = CommunicationChannel.new_DATA_TRANSMISSION_HANDSHAKE();
            PH.setPack(p130);
            p130.width = (ushort)(ushort)36515;
            p130.size = (uint)82745406U;
            p130.packets = (ushort)(ushort)23304;
            p130.payload = (byte)(byte)177;
            p130.jpg_quality = (byte)(byte)125;
            p130.height = (ushort)(ushort)24793;
            p130.type = (byte)(byte)19;
            CommunicationChannel.instance.send(p130);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnENCAPSULATED_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)21, (byte)178, (byte)98, (byte)21, (byte)160, (byte)91, (byte)25, (byte)164, (byte)153, (byte)243, (byte)100, (byte)104, (byte)108, (byte)112, (byte)214, (byte)4, (byte)1, (byte)188, (byte)196, (byte)133, (byte)147, (byte)156, (byte)114, (byte)190, (byte)159, (byte)155, (byte)87, (byte)229, (byte)199, (byte)185, (byte)124, (byte)29, (byte)209, (byte)11, (byte)161, (byte)220, (byte)202, (byte)102, (byte)6, (byte)175, (byte)111, (byte)78, (byte)206, (byte)139, (byte)250, (byte)100, (byte)58, (byte)156, (byte)183, (byte)127, (byte)126, (byte)188, (byte)115, (byte)221, (byte)121, (byte)166, (byte)225, (byte)131, (byte)238, (byte)117, (byte)126, (byte)93, (byte)223, (byte)12, (byte)144, (byte)23, (byte)165, (byte)140, (byte)183, (byte)239, (byte)37, (byte)236, (byte)245, (byte)224, (byte)0, (byte)156, (byte)208, (byte)211, (byte)255, (byte)250, (byte)80, (byte)238, (byte)42, (byte)104, (byte)4, (byte)248, (byte)121, (byte)247, (byte)151, (byte)81, (byte)247, (byte)225, (byte)31, (byte)169, (byte)103, (byte)178, (byte)158, (byte)59, (byte)220, (byte)43, (byte)165, (byte)199, (byte)102, (byte)211, (byte)69, (byte)74, (byte)76, (byte)107, (byte)128, (byte)161, (byte)209, (byte)192, (byte)74, (byte)170, (byte)118, (byte)74, (byte)4, (byte)187, (byte)24, (byte)14, (byte)18, (byte)34, (byte)141, (byte)126, (byte)94, (byte)228, (byte)94, (byte)170, (byte)5, (byte)105, (byte)218, (byte)64, (byte)141, (byte)113, (byte)208, (byte)203, (byte)138, (byte)137, (byte)83, (byte)242, (byte)152, (byte)72, (byte)0, (byte)67, (byte)228, (byte)3, (byte)115, (byte)61, (byte)222, (byte)111, (byte)144, (byte)230, (byte)26, (byte)154, (byte)167, (byte)83, (byte)138, (byte)170, (byte)47, (byte)120, (byte)169, (byte)180, (byte)183, (byte)19, (byte)17, (byte)198, (byte)101, (byte)185, (byte)89, (byte)246, (byte)144, (byte)130, (byte)13, (byte)226, (byte)83, (byte)110, (byte)57, (byte)62, (byte)50, (byte)0, (byte)47, (byte)82, (byte)238, (byte)243, (byte)199, (byte)116, (byte)106, (byte)107, (byte)42, (byte)53, (byte)83, (byte)184, (byte)250, (byte)165, (byte)251, (byte)28, (byte)0, (byte)244, (byte)66, (byte)200, (byte)199, (byte)12, (byte)160, (byte)146, (byte)244, (byte)88, (byte)68, (byte)126, (byte)14, (byte)119, (byte)97, (byte)253, (byte)14, (byte)72, (byte)75, (byte)146, (byte)51, (byte)63, (byte)204, (byte)162, (byte)111, (byte)88, (byte)14, (byte)126, (byte)58, (byte)82, (byte)26, (byte)182, (byte)201, (byte)94, (byte)243, (byte)110, (byte)203, (byte)58, (byte)246, (byte)68, (byte)27, (byte)45, (byte)203, (byte)211, (byte)52, (byte)56, (byte)130, (byte)63, (byte)57, (byte)186, (byte)215, (byte)195, (byte)214, (byte)174, (byte)26, (byte)100, (byte)111}));
                Debug.Assert(pack.seqnr == (ushort)(ushort)36572);
            };
            GroundControl.ENCAPSULATED_DATA p131 = CommunicationChannel.new_ENCAPSULATED_DATA();
            PH.setPack(p131);
            p131.data__SET(new byte[] {(byte)21, (byte)178, (byte)98, (byte)21, (byte)160, (byte)91, (byte)25, (byte)164, (byte)153, (byte)243, (byte)100, (byte)104, (byte)108, (byte)112, (byte)214, (byte)4, (byte)1, (byte)188, (byte)196, (byte)133, (byte)147, (byte)156, (byte)114, (byte)190, (byte)159, (byte)155, (byte)87, (byte)229, (byte)199, (byte)185, (byte)124, (byte)29, (byte)209, (byte)11, (byte)161, (byte)220, (byte)202, (byte)102, (byte)6, (byte)175, (byte)111, (byte)78, (byte)206, (byte)139, (byte)250, (byte)100, (byte)58, (byte)156, (byte)183, (byte)127, (byte)126, (byte)188, (byte)115, (byte)221, (byte)121, (byte)166, (byte)225, (byte)131, (byte)238, (byte)117, (byte)126, (byte)93, (byte)223, (byte)12, (byte)144, (byte)23, (byte)165, (byte)140, (byte)183, (byte)239, (byte)37, (byte)236, (byte)245, (byte)224, (byte)0, (byte)156, (byte)208, (byte)211, (byte)255, (byte)250, (byte)80, (byte)238, (byte)42, (byte)104, (byte)4, (byte)248, (byte)121, (byte)247, (byte)151, (byte)81, (byte)247, (byte)225, (byte)31, (byte)169, (byte)103, (byte)178, (byte)158, (byte)59, (byte)220, (byte)43, (byte)165, (byte)199, (byte)102, (byte)211, (byte)69, (byte)74, (byte)76, (byte)107, (byte)128, (byte)161, (byte)209, (byte)192, (byte)74, (byte)170, (byte)118, (byte)74, (byte)4, (byte)187, (byte)24, (byte)14, (byte)18, (byte)34, (byte)141, (byte)126, (byte)94, (byte)228, (byte)94, (byte)170, (byte)5, (byte)105, (byte)218, (byte)64, (byte)141, (byte)113, (byte)208, (byte)203, (byte)138, (byte)137, (byte)83, (byte)242, (byte)152, (byte)72, (byte)0, (byte)67, (byte)228, (byte)3, (byte)115, (byte)61, (byte)222, (byte)111, (byte)144, (byte)230, (byte)26, (byte)154, (byte)167, (byte)83, (byte)138, (byte)170, (byte)47, (byte)120, (byte)169, (byte)180, (byte)183, (byte)19, (byte)17, (byte)198, (byte)101, (byte)185, (byte)89, (byte)246, (byte)144, (byte)130, (byte)13, (byte)226, (byte)83, (byte)110, (byte)57, (byte)62, (byte)50, (byte)0, (byte)47, (byte)82, (byte)238, (byte)243, (byte)199, (byte)116, (byte)106, (byte)107, (byte)42, (byte)53, (byte)83, (byte)184, (byte)250, (byte)165, (byte)251, (byte)28, (byte)0, (byte)244, (byte)66, (byte)200, (byte)199, (byte)12, (byte)160, (byte)146, (byte)244, (byte)88, (byte)68, (byte)126, (byte)14, (byte)119, (byte)97, (byte)253, (byte)14, (byte)72, (byte)75, (byte)146, (byte)51, (byte)63, (byte)204, (byte)162, (byte)111, (byte)88, (byte)14, (byte)126, (byte)58, (byte)82, (byte)26, (byte)182, (byte)201, (byte)94, (byte)243, (byte)110, (byte)203, (byte)58, (byte)246, (byte)68, (byte)27, (byte)45, (byte)203, (byte)211, (byte)52, (byte)56, (byte)130, (byte)63, (byte)57, (byte)186, (byte)215, (byte)195, (byte)214, (byte)174, (byte)26, (byte)100, (byte)111}, 0) ;
            p131.seqnr = (ushort)(ushort)36572;
            CommunicationChannel.instance.send(p131);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnDISTANCE_SENSORReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.max_distance == (ushort)(ushort)5486);
                Debug.Assert(pack.orientation == MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_180_PITCH_270);
                Debug.Assert(pack.current_distance == (ushort)(ushort)61236);
                Debug.Assert(pack.covariance == (byte)(byte)202);
                Debug.Assert(pack.time_boot_ms == (uint)2177875414U);
                Debug.Assert(pack.type == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_RADAR);
                Debug.Assert(pack.id == (byte)(byte)207);
                Debug.Assert(pack.min_distance == (ushort)(ushort)34128);
            };
            GroundControl.DISTANCE_SENSOR p132 = CommunicationChannel.new_DISTANCE_SENSOR();
            PH.setPack(p132);
            p132.min_distance = (ushort)(ushort)34128;
            p132.id = (byte)(byte)207;
            p132.covariance = (byte)(byte)202;
            p132.time_boot_ms = (uint)2177875414U;
            p132.max_distance = (ushort)(ushort)5486;
            p132.orientation = MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_180_PITCH_270;
            p132.type = MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_RADAR;
            p132.current_distance = (ushort)(ushort)61236;
            CommunicationChannel.instance.send(p132);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTERRAIN_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mask == (ulong)8372977278719475968L);
                Debug.Assert(pack.grid_spacing == (ushort)(ushort)44747);
                Debug.Assert(pack.lon == (int) -697214823);
                Debug.Assert(pack.lat == (int) -111142458);
            };
            GroundControl.TERRAIN_REQUEST p133 = CommunicationChannel.new_TERRAIN_REQUEST();
            PH.setPack(p133);
            p133.grid_spacing = (ushort)(ushort)44747;
            p133.lat = (int) -111142458;
            p133.lon = (int) -697214823;
            p133.mask = (ulong)8372977278719475968L;
            CommunicationChannel.instance.send(p133);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTERRAIN_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.data_.SequenceEqual(new short[] {(short) -17228, (short)26144, (short) -27500, (short) -25694, (short)15946, (short) -31994, (short)21420, (short) -5263, (short)16480, (short) -31190, (short)31098, (short)21718, (short) -17582, (short)3747, (short) -13779, (short) -19595}));
                Debug.Assert(pack.lat == (int) -1560289622);
                Debug.Assert(pack.gridbit == (byte)(byte)154);
                Debug.Assert(pack.lon == (int) -262414267);
                Debug.Assert(pack.grid_spacing == (ushort)(ushort)65173);
            };
            GroundControl.TERRAIN_DATA p134 = CommunicationChannel.new_TERRAIN_DATA();
            PH.setPack(p134);
            p134.gridbit = (byte)(byte)154;
            p134.lat = (int) -1560289622;
            p134.data__SET(new short[] {(short) -17228, (short)26144, (short) -27500, (short) -25694, (short)15946, (short) -31994, (short)21420, (short) -5263, (short)16480, (short) -31190, (short)31098, (short)21718, (short) -17582, (short)3747, (short) -13779, (short) -19595}, 0) ;
            p134.grid_spacing = (ushort)(ushort)65173;
            p134.lon = (int) -262414267;
            CommunicationChannel.instance.send(p134);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTERRAIN_CHECKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lon == (int)1224545267);
                Debug.Assert(pack.lat == (int)1021194319);
            };
            GroundControl.TERRAIN_CHECK p135 = CommunicationChannel.new_TERRAIN_CHECK();
            PH.setPack(p135);
            p135.lon = (int)1224545267;
            p135.lat = (int)1021194319;
            CommunicationChannel.instance.send(p135);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTERRAIN_REPORTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.terrain_height == (float)4.710307E36F);
                Debug.Assert(pack.current_height == (float)8.0534025E37F);
                Debug.Assert(pack.lat == (int) -329932143);
                Debug.Assert(pack.spacing == (ushort)(ushort)16725);
                Debug.Assert(pack.lon == (int)2049066855);
                Debug.Assert(pack.pending == (ushort)(ushort)4379);
                Debug.Assert(pack.loaded == (ushort)(ushort)61174);
            };
            GroundControl.TERRAIN_REPORT p136 = CommunicationChannel.new_TERRAIN_REPORT();
            PH.setPack(p136);
            p136.spacing = (ushort)(ushort)16725;
            p136.lon = (int)2049066855;
            p136.current_height = (float)8.0534025E37F;
            p136.lat = (int) -329932143;
            p136.terrain_height = (float)4.710307E36F;
            p136.loaded = (ushort)(ushort)61174;
            p136.pending = (ushort)(ushort)4379;
            CommunicationChannel.instance.send(p136);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_PRESSURE2Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.press_diff == (float) -9.612924E37F);
                Debug.Assert(pack.time_boot_ms == (uint)286950654U);
                Debug.Assert(pack.press_abs == (float) -1.1610783E38F);
                Debug.Assert(pack.temperature == (short)(short)16143);
            };
            GroundControl.SCALED_PRESSURE2 p137 = CommunicationChannel.new_SCALED_PRESSURE2();
            PH.setPack(p137);
            p137.time_boot_ms = (uint)286950654U;
            p137.temperature = (short)(short)16143;
            p137.press_diff = (float) -9.612924E37F;
            p137.press_abs = (float) -1.1610783E38F;
            CommunicationChannel.instance.send(p137);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATT_POS_MOCAPReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.z == (float) -3.3308876E38F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-3.4443408E37F, 3.2864988E38F, -3.1817712E38F, -1.3592681E38F}));
                Debug.Assert(pack.time_usec == (ulong)8225154329781359448L);
                Debug.Assert(pack.y == (float)6.7591903E36F);
                Debug.Assert(pack.x == (float) -2.7649603E38F);
            };
            GroundControl.ATT_POS_MOCAP p138 = CommunicationChannel.new_ATT_POS_MOCAP();
            PH.setPack(p138);
            p138.x = (float) -2.7649603E38F;
            p138.y = (float)6.7591903E36F;
            p138.z = (float) -3.3308876E38F;
            p138.time_usec = (ulong)8225154329781359448L;
            p138.q_SET(new float[] {-3.4443408E37F, 3.2864988E38F, -3.1817712E38F, -1.3592681E38F}, 0) ;
            CommunicationChannel.instance.send(p138);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_ACTUATOR_CONTROL_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.controls.SequenceEqual(new float[] {-1.2408517E38F, -2.2636072E38F, 8.7675144E35F, -1.2792853E38F, 3.1641923E37F, 3.0329129E38F, 9.583946E37F, -3.9495903E37F}));
                Debug.Assert(pack.target_component == (byte)(byte)51);
                Debug.Assert(pack.group_mlx == (byte)(byte)254);
                Debug.Assert(pack.target_system == (byte)(byte)70);
                Debug.Assert(pack.time_usec == (ulong)393787733045408587L);
            };
            GroundControl.SET_ACTUATOR_CONTROL_TARGET p139 = CommunicationChannel.new_SET_ACTUATOR_CONTROL_TARGET();
            PH.setPack(p139);
            p139.time_usec = (ulong)393787733045408587L;
            p139.target_system = (byte)(byte)70;
            p139.controls_SET(new float[] {-1.2408517E38F, -2.2636072E38F, 8.7675144E35F, -1.2792853E38F, 3.1641923E37F, 3.0329129E38F, 9.583946E37F, -3.9495903E37F}, 0) ;
            p139.target_component = (byte)(byte)51;
            p139.group_mlx = (byte)(byte)254;
            CommunicationChannel.instance.send(p139);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnACTUATOR_CONTROL_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)8355819359037227800L);
                Debug.Assert(pack.group_mlx == (byte)(byte)161);
                Debug.Assert(pack.controls.SequenceEqual(new float[] {3.0461102E38F, 3.089011E38F, -1.1354305E38F, -1.1153871E38F, -3.193054E38F, -3.377775E38F, 2.4274296E38F, -3.186469E38F}));
            };
            GroundControl.ACTUATOR_CONTROL_TARGET p140 = CommunicationChannel.new_ACTUATOR_CONTROL_TARGET();
            PH.setPack(p140);
            p140.controls_SET(new float[] {3.0461102E38F, 3.089011E38F, -1.1354305E38F, -1.1153871E38F, -3.193054E38F, -3.377775E38F, 2.4274296E38F, -3.186469E38F}, 0) ;
            p140.group_mlx = (byte)(byte)161;
            p140.time_usec = (ulong)8355819359037227800L;
            CommunicationChannel.instance.send(p140);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnALTITUDEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.bottom_clearance == (float)3.046761E38F);
                Debug.Assert(pack.altitude_amsl == (float) -2.1014408E38F);
                Debug.Assert(pack.altitude_relative == (float)7.0979925E37F);
                Debug.Assert(pack.altitude_terrain == (float) -3.0942262E38F);
                Debug.Assert(pack.time_usec == (ulong)3248644481190864153L);
                Debug.Assert(pack.altitude_local == (float)2.9340816E38F);
                Debug.Assert(pack.altitude_monotonic == (float)6.6329103E37F);
            };
            GroundControl.ALTITUDE p141 = CommunicationChannel.new_ALTITUDE();
            PH.setPack(p141);
            p141.altitude_terrain = (float) -3.0942262E38F;
            p141.bottom_clearance = (float)3.046761E38F;
            p141.altitude_local = (float)2.9340816E38F;
            p141.altitude_relative = (float)7.0979925E37F;
            p141.time_usec = (ulong)3248644481190864153L;
            p141.altitude_amsl = (float) -2.1014408E38F;
            p141.altitude_monotonic = (float)6.6329103E37F;
            CommunicationChannel.instance.send(p141);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRESOURCE_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.uri.SequenceEqual(new byte[] {(byte)25, (byte)238, (byte)183, (byte)155, (byte)52, (byte)149, (byte)143, (byte)255, (byte)187, (byte)91, (byte)89, (byte)230, (byte)206, (byte)194, (byte)239, (byte)172, (byte)180, (byte)235, (byte)20, (byte)143, (byte)150, (byte)219, (byte)88, (byte)173, (byte)229, (byte)35, (byte)110, (byte)131, (byte)197, (byte)141, (byte)188, (byte)83, (byte)62, (byte)255, (byte)162, (byte)25, (byte)188, (byte)193, (byte)186, (byte)177, (byte)164, (byte)49, (byte)77, (byte)238, (byte)21, (byte)141, (byte)40, (byte)239, (byte)245, (byte)136, (byte)63, (byte)46, (byte)173, (byte)98, (byte)131, (byte)223, (byte)162, (byte)120, (byte)78, (byte)175, (byte)127, (byte)147, (byte)249, (byte)4, (byte)142, (byte)74, (byte)14, (byte)246, (byte)18, (byte)116, (byte)75, (byte)202, (byte)225, (byte)233, (byte)193, (byte)243, (byte)188, (byte)65, (byte)74, (byte)215, (byte)81, (byte)136, (byte)185, (byte)104, (byte)202, (byte)27, (byte)127, (byte)153, (byte)154, (byte)155, (byte)35, (byte)12, (byte)0, (byte)250, (byte)209, (byte)91, (byte)226, (byte)144, (byte)167, (byte)231, (byte)245, (byte)38, (byte)243, (byte)88, (byte)17, (byte)0, (byte)29, (byte)224, (byte)21, (byte)169, (byte)188, (byte)227, (byte)139, (byte)64, (byte)166, (byte)234, (byte)63, (byte)210, (byte)4, (byte)95}));
                Debug.Assert(pack.transfer_type == (byte)(byte)229);
                Debug.Assert(pack.storage.SequenceEqual(new byte[] {(byte)233, (byte)60, (byte)103, (byte)9, (byte)144, (byte)90, (byte)240, (byte)237, (byte)31, (byte)165, (byte)14, (byte)221, (byte)231, (byte)209, (byte)166, (byte)111, (byte)30, (byte)237, (byte)145, (byte)195, (byte)190, (byte)209, (byte)139, (byte)211, (byte)84, (byte)124, (byte)177, (byte)208, (byte)62, (byte)20, (byte)123, (byte)235, (byte)130, (byte)35, (byte)161, (byte)30, (byte)51, (byte)1, (byte)253, (byte)8, (byte)32, (byte)201, (byte)96, (byte)28, (byte)74, (byte)59, (byte)61, (byte)32, (byte)220, (byte)157, (byte)97, (byte)122, (byte)165, (byte)106, (byte)209, (byte)57, (byte)180, (byte)59, (byte)115, (byte)155, (byte)77, (byte)95, (byte)41, (byte)54, (byte)249, (byte)40, (byte)136, (byte)81, (byte)129, (byte)116, (byte)50, (byte)173, (byte)138, (byte)237, (byte)60, (byte)9, (byte)154, (byte)116, (byte)106, (byte)3, (byte)163, (byte)166, (byte)136, (byte)216, (byte)38, (byte)13, (byte)233, (byte)52, (byte)116, (byte)64, (byte)172, (byte)69, (byte)160, (byte)101, (byte)225, (byte)40, (byte)221, (byte)157, (byte)206, (byte)28, (byte)230, (byte)171, (byte)19, (byte)63, (byte)85, (byte)115, (byte)239, (byte)114, (byte)38, (byte)211, (byte)213, (byte)117, (byte)32, (byte)45, (byte)250, (byte)163, (byte)29, (byte)47, (byte)237, (byte)244}));
                Debug.Assert(pack.uri_type == (byte)(byte)204);
                Debug.Assert(pack.request_id == (byte)(byte)162);
            };
            GroundControl.RESOURCE_REQUEST p142 = CommunicationChannel.new_RESOURCE_REQUEST();
            PH.setPack(p142);
            p142.storage_SET(new byte[] {(byte)233, (byte)60, (byte)103, (byte)9, (byte)144, (byte)90, (byte)240, (byte)237, (byte)31, (byte)165, (byte)14, (byte)221, (byte)231, (byte)209, (byte)166, (byte)111, (byte)30, (byte)237, (byte)145, (byte)195, (byte)190, (byte)209, (byte)139, (byte)211, (byte)84, (byte)124, (byte)177, (byte)208, (byte)62, (byte)20, (byte)123, (byte)235, (byte)130, (byte)35, (byte)161, (byte)30, (byte)51, (byte)1, (byte)253, (byte)8, (byte)32, (byte)201, (byte)96, (byte)28, (byte)74, (byte)59, (byte)61, (byte)32, (byte)220, (byte)157, (byte)97, (byte)122, (byte)165, (byte)106, (byte)209, (byte)57, (byte)180, (byte)59, (byte)115, (byte)155, (byte)77, (byte)95, (byte)41, (byte)54, (byte)249, (byte)40, (byte)136, (byte)81, (byte)129, (byte)116, (byte)50, (byte)173, (byte)138, (byte)237, (byte)60, (byte)9, (byte)154, (byte)116, (byte)106, (byte)3, (byte)163, (byte)166, (byte)136, (byte)216, (byte)38, (byte)13, (byte)233, (byte)52, (byte)116, (byte)64, (byte)172, (byte)69, (byte)160, (byte)101, (byte)225, (byte)40, (byte)221, (byte)157, (byte)206, (byte)28, (byte)230, (byte)171, (byte)19, (byte)63, (byte)85, (byte)115, (byte)239, (byte)114, (byte)38, (byte)211, (byte)213, (byte)117, (byte)32, (byte)45, (byte)250, (byte)163, (byte)29, (byte)47, (byte)237, (byte)244}, 0) ;
            p142.uri_type = (byte)(byte)204;
            p142.request_id = (byte)(byte)162;
            p142.uri_SET(new byte[] {(byte)25, (byte)238, (byte)183, (byte)155, (byte)52, (byte)149, (byte)143, (byte)255, (byte)187, (byte)91, (byte)89, (byte)230, (byte)206, (byte)194, (byte)239, (byte)172, (byte)180, (byte)235, (byte)20, (byte)143, (byte)150, (byte)219, (byte)88, (byte)173, (byte)229, (byte)35, (byte)110, (byte)131, (byte)197, (byte)141, (byte)188, (byte)83, (byte)62, (byte)255, (byte)162, (byte)25, (byte)188, (byte)193, (byte)186, (byte)177, (byte)164, (byte)49, (byte)77, (byte)238, (byte)21, (byte)141, (byte)40, (byte)239, (byte)245, (byte)136, (byte)63, (byte)46, (byte)173, (byte)98, (byte)131, (byte)223, (byte)162, (byte)120, (byte)78, (byte)175, (byte)127, (byte)147, (byte)249, (byte)4, (byte)142, (byte)74, (byte)14, (byte)246, (byte)18, (byte)116, (byte)75, (byte)202, (byte)225, (byte)233, (byte)193, (byte)243, (byte)188, (byte)65, (byte)74, (byte)215, (byte)81, (byte)136, (byte)185, (byte)104, (byte)202, (byte)27, (byte)127, (byte)153, (byte)154, (byte)155, (byte)35, (byte)12, (byte)0, (byte)250, (byte)209, (byte)91, (byte)226, (byte)144, (byte)167, (byte)231, (byte)245, (byte)38, (byte)243, (byte)88, (byte)17, (byte)0, (byte)29, (byte)224, (byte)21, (byte)169, (byte)188, (byte)227, (byte)139, (byte)64, (byte)166, (byte)234, (byte)63, (byte)210, (byte)4, (byte)95}, 0) ;
            p142.transfer_type = (byte)(byte)229;
            CommunicationChannel.instance.send(p142);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_PRESSURE3Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.press_abs == (float) -2.8308676E37F);
                Debug.Assert(pack.temperature == (short)(short) -4510);
                Debug.Assert(pack.time_boot_ms == (uint)1454839693U);
                Debug.Assert(pack.press_diff == (float)2.088296E38F);
            };
            GroundControl.SCALED_PRESSURE3 p143 = CommunicationChannel.new_SCALED_PRESSURE3();
            PH.setPack(p143);
            p143.press_abs = (float) -2.8308676E37F;
            p143.time_boot_ms = (uint)1454839693U;
            p143.press_diff = (float)2.088296E38F;
            p143.temperature = (short)(short) -4510;
            CommunicationChannel.instance.send(p143);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnFOLLOW_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.acc.SequenceEqual(new float[] {1.4321523E38F, 2.8870465E38F, 2.3775006E38F}));
                Debug.Assert(pack.lon == (int)793631235);
                Debug.Assert(pack.timestamp == (ulong)402079410227256721L);
                Debug.Assert(pack.attitude_q.SequenceEqual(new float[] {-2.8629818E35F, -1.2939461E38F, -1.5912773E38F, -6.2609897E37F}));
                Debug.Assert(pack.alt == (float)1.4778212E38F);
                Debug.Assert(pack.vel.SequenceEqual(new float[] {-3.0702035E38F, 2.2865873E38F, 1.8614634E38F}));
                Debug.Assert(pack.rates.SequenceEqual(new float[] {-3.3792794E38F, 3.1771042E38F, 6.895229E37F}));
                Debug.Assert(pack.position_cov.SequenceEqual(new float[] {-1.2393208E37F, 3.3588285E38F, -1.653393E38F}));
                Debug.Assert(pack.est_capabilities == (byte)(byte)149);
                Debug.Assert(pack.lat == (int) -262607286);
                Debug.Assert(pack.custom_state == (ulong)6354509711582432868L);
            };
            GroundControl.FOLLOW_TARGET p144 = CommunicationChannel.new_FOLLOW_TARGET();
            PH.setPack(p144);
            p144.position_cov_SET(new float[] {-1.2393208E37F, 3.3588285E38F, -1.653393E38F}, 0) ;
            p144.attitude_q_SET(new float[] {-2.8629818E35F, -1.2939461E38F, -1.5912773E38F, -6.2609897E37F}, 0) ;
            p144.custom_state = (ulong)6354509711582432868L;
            p144.est_capabilities = (byte)(byte)149;
            p144.lat = (int) -262607286;
            p144.rates_SET(new float[] {-3.3792794E38F, 3.1771042E38F, 6.895229E37F}, 0) ;
            p144.lon = (int)793631235;
            p144.acc_SET(new float[] {1.4321523E38F, 2.8870465E38F, 2.3775006E38F}, 0) ;
            p144.vel_SET(new float[] {-3.0702035E38F, 2.2865873E38F, 1.8614634E38F}, 0) ;
            p144.timestamp = (ulong)402079410227256721L;
            p144.alt = (float)1.4778212E38F;
            CommunicationChannel.instance.send(p144);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCONTROL_SYSTEM_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.y_pos == (float)3.2246652E38F);
                Debug.Assert(pack.pitch_rate == (float)6.0045825E37F);
                Debug.Assert(pack.z_pos == (float) -3.2274323E38F);
                Debug.Assert(pack.y_acc == (float)2.6889227E37F);
                Debug.Assert(pack.roll_rate == (float)1.53291E38F);
                Debug.Assert(pack.z_acc == (float)1.2331914E38F);
                Debug.Assert(pack.x_acc == (float) -3.0243097E38F);
                Debug.Assert(pack.time_usec == (ulong)6657988183660911082L);
                Debug.Assert(pack.airspeed == (float)2.5614797E38F);
                Debug.Assert(pack.z_vel == (float)1.9211356E38F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-2.21723E38F, 1.6903586E38F, 2.5129553E38F, -3.3038013E38F}));
                Debug.Assert(pack.x_pos == (float) -2.1764277E38F);
                Debug.Assert(pack.pos_variance.SequenceEqual(new float[] {-2.2955619E37F, 2.419932E38F, 3.8881268E37F}));
                Debug.Assert(pack.x_vel == (float)2.6632514E38F);
                Debug.Assert(pack.y_vel == (float)3.1294194E37F);
                Debug.Assert(pack.yaw_rate == (float)3.5223074E37F);
                Debug.Assert(pack.vel_variance.SequenceEqual(new float[] {3.3249944E38F, -1.8110928E37F, 1.8111892E38F}));
            };
            GroundControl.CONTROL_SYSTEM_STATE p146 = CommunicationChannel.new_CONTROL_SYSTEM_STATE();
            PH.setPack(p146);
            p146.y_pos = (float)3.2246652E38F;
            p146.time_usec = (ulong)6657988183660911082L;
            p146.x_pos = (float) -2.1764277E38F;
            p146.q_SET(new float[] {-2.21723E38F, 1.6903586E38F, 2.5129553E38F, -3.3038013E38F}, 0) ;
            p146.pitch_rate = (float)6.0045825E37F;
            p146.z_pos = (float) -3.2274323E38F;
            p146.x_acc = (float) -3.0243097E38F;
            p146.z_acc = (float)1.2331914E38F;
            p146.airspeed = (float)2.5614797E38F;
            p146.vel_variance_SET(new float[] {3.3249944E38F, -1.8110928E37F, 1.8111892E38F}, 0) ;
            p146.pos_variance_SET(new float[] {-2.2955619E37F, 2.419932E38F, 3.8881268E37F}, 0) ;
            p146.y_acc = (float)2.6889227E37F;
            p146.x_vel = (float)2.6632514E38F;
            p146.roll_rate = (float)1.53291E38F;
            p146.y_vel = (float)3.1294194E37F;
            p146.z_vel = (float)1.9211356E38F;
            p146.yaw_rate = (float)3.5223074E37F;
            CommunicationChannel.instance.send(p146);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnBATTERY_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.current_consumed == (int) -1845283670);
                Debug.Assert(pack.voltages.SequenceEqual(new ushort[] {(ushort)2218, (ushort)38120, (ushort)51322, (ushort)59814, (ushort)62208, (ushort)8082, (ushort)60998, (ushort)25366, (ushort)60718, (ushort)58786}));
                Debug.Assert(pack.battery_remaining == (sbyte)(sbyte)84);
                Debug.Assert(pack.battery_function == MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_ALL);
                Debug.Assert(pack.id == (byte)(byte)139);
                Debug.Assert(pack.energy_consumed == (int) -1397769773);
                Debug.Assert(pack.type == MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_LIFE);
                Debug.Assert(pack.current_battery == (short)(short) -1011);
                Debug.Assert(pack.temperature == (short)(short) -3218);
            };
            GroundControl.BATTERY_STATUS p147 = CommunicationChannel.new_BATTERY_STATUS();
            PH.setPack(p147);
            p147.battery_function = MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_ALL;
            p147.energy_consumed = (int) -1397769773;
            p147.type = MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_LIFE;
            p147.voltages_SET(new ushort[] {(ushort)2218, (ushort)38120, (ushort)51322, (ushort)59814, (ushort)62208, (ushort)8082, (ushort)60998, (ushort)25366, (ushort)60718, (ushort)58786}, 0) ;
            p147.temperature = (short)(short) -3218;
            p147.id = (byte)(byte)139;
            p147.current_battery = (short)(short) -1011;
            p147.battery_remaining = (sbyte)(sbyte)84;
            p147.current_consumed = (int) -1845283670;
            CommunicationChannel.instance.send(p147);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnAUTOPILOT_VERSIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.board_version == (uint)2897072877U);
                Debug.Assert(pack.os_sw_version == (uint)1577192606U);
                Debug.Assert(pack.vendor_id == (ushort)(ushort)45231);
                Debug.Assert(pack.middleware_sw_version == (uint)437245296U);
                Debug.Assert(pack.capabilities == (MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET |
                                                   MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FLIGHT_INFORMATION |
                                                   MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_PARAM_UNION |
                                                   MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT |
                                                   MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_ACTUATOR_TARGET |
                                                   MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FTP |
                                                   MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT |
                                                   MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION |
                                                   MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_INT |
                                                   MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION |
                                                   MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_FENCE));
                Debug.Assert(pack.flight_sw_version == (uint)4092257281U);
                Debug.Assert(pack.uid2_TRY(ph).SequenceEqual(new byte[] {(byte)102, (byte)200, (byte)121, (byte)62, (byte)62, (byte)143, (byte)41, (byte)177, (byte)232, (byte)255, (byte)111, (byte)145, (byte)152, (byte)174, (byte)18, (byte)113, (byte)57, (byte)117}));
                Debug.Assert(pack.os_custom_version.SequenceEqual(new byte[] {(byte)61, (byte)10, (byte)39, (byte)134, (byte)96, (byte)138, (byte)180, (byte)75}));
                Debug.Assert(pack.product_id == (ushort)(ushort)1923);
                Debug.Assert(pack.uid == (ulong)5892152378978938681L);
                Debug.Assert(pack.middleware_custom_version.SequenceEqual(new byte[] {(byte)190, (byte)202, (byte)189, (byte)109, (byte)45, (byte)210, (byte)105, (byte)147}));
                Debug.Assert(pack.flight_custom_version.SequenceEqual(new byte[] {(byte)215, (byte)100, (byte)8, (byte)5, (byte)188, (byte)17, (byte)244, (byte)190}));
            };
            GroundControl.AUTOPILOT_VERSION p148 = CommunicationChannel.new_AUTOPILOT_VERSION();
            PH.setPack(p148);
            p148.middleware_sw_version = (uint)437245296U;
            p148.flight_sw_version = (uint)4092257281U;
            p148.os_custom_version_SET(new byte[] {(byte)61, (byte)10, (byte)39, (byte)134, (byte)96, (byte)138, (byte)180, (byte)75}, 0) ;
            p148.product_id = (ushort)(ushort)1923;
            p148.board_version = (uint)2897072877U;
            p148.vendor_id = (ushort)(ushort)45231;
            p148.flight_custom_version_SET(new byte[] {(byte)215, (byte)100, (byte)8, (byte)5, (byte)188, (byte)17, (byte)244, (byte)190}, 0) ;
            p148.os_sw_version = (uint)1577192606U;
            p148.uid2_SET(new byte[] {(byte)102, (byte)200, (byte)121, (byte)62, (byte)62, (byte)143, (byte)41, (byte)177, (byte)232, (byte)255, (byte)111, (byte)145, (byte)152, (byte)174, (byte)18, (byte)113, (byte)57, (byte)117}, 0, PH) ;
            p148.uid = (ulong)5892152378978938681L;
            p148.middleware_custom_version_SET(new byte[] {(byte)190, (byte)202, (byte)189, (byte)109, (byte)45, (byte)210, (byte)105, (byte)147}, 0) ;
            p148.capabilities = (MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FLIGHT_INFORMATION |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_PARAM_UNION |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_ACTUATOR_TARGET |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FTP |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_INT |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_FENCE);
            CommunicationChannel.instance.send(p148);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLANDING_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_num == (byte)(byte)26);
                Debug.Assert(pack.angle_x == (float)3.1854847E35F);
                Debug.Assert(pack.size_y == (float) -1.1411029E38F);
                Debug.Assert(pack.position_valid_TRY(ph) == (byte)(byte)222);
                Debug.Assert(pack.type == LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_VISION_OTHER);
                Debug.Assert(pack.z_TRY(ph) == (float) -3.16098E38F);
                Debug.Assert(pack.y_TRY(ph) == (float)1.977145E38F);
                Debug.Assert(pack.angle_y == (float)1.5909189E37F);
                Debug.Assert(pack.time_usec == (ulong)5376195000259058795L);
                Debug.Assert(pack.size_x == (float)1.4864992E37F);
                Debug.Assert(pack.q_TRY(ph).SequenceEqual(new float[] {2.4211135E37F, 2.5187045E38F, 2.7499159E38F, -3.01348E38F}));
                Debug.Assert(pack.x_TRY(ph) == (float) -3.7106542E37F);
                Debug.Assert(pack.distance == (float)1.0370917E38F);
                Debug.Assert(pack.frame == MAV_FRAME.MAV_FRAME_LOCAL_NED);
            };
            GroundControl.LANDING_TARGET p149 = CommunicationChannel.new_LANDING_TARGET();
            PH.setPack(p149);
            p149.x_SET((float) -3.7106542E37F, PH) ;
            p149.frame = MAV_FRAME.MAV_FRAME_LOCAL_NED;
            p149.target_num = (byte)(byte)26;
            p149.q_SET(new float[] {2.4211135E37F, 2.5187045E38F, 2.7499159E38F, -3.01348E38F}, 0, PH) ;
            p149.position_valid_SET((byte)(byte)222, PH) ;
            p149.size_x = (float)1.4864992E37F;
            p149.type = LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_VISION_OTHER;
            p149.size_y = (float) -1.1411029E38F;
            p149.z_SET((float) -3.16098E38F, PH) ;
            p149.y_SET((float)1.977145E38F, PH) ;
            p149.time_usec = (ulong)5376195000259058795L;
            p149.angle_y = (float)1.5909189E37F;
            p149.distance = (float)1.0370917E38F;
            p149.angle_x = (float)3.1854847E35F;
            CommunicationChannel.instance.send(p149);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            ADV_TEST_CH.OnSENSOR_OFFSETSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.gyro_cal_z == (float)2.2592913E38F);
                Debug.Assert(pack.gyro_cal_x == (float)7.1413735E37F);
                Debug.Assert(pack.accel_cal_z == (float)8.642549E37F);
                Debug.Assert(pack.mag_ofs_z == (short)(short) -28429);
                Debug.Assert(pack.mag_ofs_x == (short)(short)14399);
                Debug.Assert(pack.raw_press == (int)136083835);
                Debug.Assert(pack.mag_ofs_y == (short)(short) -21288);
                Debug.Assert(pack.raw_temp == (int) -1434919439);
                Debug.Assert(pack.accel_cal_y == (float)8.591608E37F);
                Debug.Assert(pack.mag_declination == (float) -1.5837997E38F);
                Debug.Assert(pack.gyro_cal_y == (float)3.2842723E37F);
                Debug.Assert(pack.accel_cal_x == (float) -1.4647064E38F);
            };
            GroundControl.SENSOR_OFFSETS p150 = CommunicationChannel.new_SENSOR_OFFSETS();
            PH.setPack(p150);
            p150.mag_ofs_x = (short)(short)14399;
            p150.accel_cal_x = (float) -1.4647064E38F;
            p150.gyro_cal_z = (float)2.2592913E38F;
            p150.accel_cal_y = (float)8.591608E37F;
            p150.accel_cal_z = (float)8.642549E37F;
            p150.raw_temp = (int) -1434919439;
            p150.gyro_cal_y = (float)3.2842723E37F;
            p150.raw_press = (int)136083835;
            p150.mag_ofs_y = (short)(short) -21288;
            p150.mag_ofs_z = (short)(short) -28429;
            p150.gyro_cal_x = (float)7.1413735E37F;
            p150.mag_declination = (float) -1.5837997E38F;
            CommunicationChannel.instance.send(p150);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnSET_MAG_OFFSETSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mag_ofs_z == (short)(short) -23406);
                Debug.Assert(pack.target_component == (byte)(byte)191);
                Debug.Assert(pack.mag_ofs_x == (short)(short) -26104);
                Debug.Assert(pack.target_system == (byte)(byte)212);
                Debug.Assert(pack.mag_ofs_y == (short)(short) -21162);
            };
            GroundControl.SET_MAG_OFFSETS p151 = CommunicationChannel.new_SET_MAG_OFFSETS();
            PH.setPack(p151);
            p151.mag_ofs_x = (short)(short) -26104;
            p151.target_system = (byte)(byte)212;
            p151.mag_ofs_y = (short)(short) -21162;
            p151.target_component = (byte)(byte)191;
            p151.mag_ofs_z = (short)(short) -23406;
            CommunicationChannel.instance.send(p151);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnMEMINFOReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.freemem == (ushort)(ushort)60665);
                Debug.Assert(pack.freemem32_TRY(ph) == (uint)3846929077U);
                Debug.Assert(pack.brkval == (ushort)(ushort)55384);
            };
            GroundControl.MEMINFO p152 = CommunicationChannel.new_MEMINFO();
            PH.setPack(p152);
            p152.freemem32_SET((uint)3846929077U, PH) ;
            p152.freemem = (ushort)(ushort)60665;
            p152.brkval = (ushort)(ushort)55384;
            CommunicationChannel.instance.send(p152);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnAP_ADCReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.adc4 == (ushort)(ushort)11842);
                Debug.Assert(pack.adc3 == (ushort)(ushort)22763);
                Debug.Assert(pack.adc6 == (ushort)(ushort)27049);
                Debug.Assert(pack.adc2 == (ushort)(ushort)61794);
                Debug.Assert(pack.adc5 == (ushort)(ushort)37850);
                Debug.Assert(pack.adc1 == (ushort)(ushort)45390);
            };
            GroundControl.AP_ADC p153 = CommunicationChannel.new_AP_ADC();
            PH.setPack(p153);
            p153.adc1 = (ushort)(ushort)45390;
            p153.adc4 = (ushort)(ushort)11842;
            p153.adc3 = (ushort)(ushort)22763;
            p153.adc2 = (ushort)(ushort)61794;
            p153.adc5 = (ushort)(ushort)37850;
            p153.adc6 = (ushort)(ushort)27049;
            CommunicationChannel.instance.send(p153);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnDIGICAM_CONFIGUREReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.command_id == (byte)(byte)0);
                Debug.Assert(pack.iso == (byte)(byte)37);
                Debug.Assert(pack.target_component == (byte)(byte)151);
                Debug.Assert(pack.shutter_speed == (ushort)(ushort)58602);
                Debug.Assert(pack.aperture == (byte)(byte)231);
                Debug.Assert(pack.target_system == (byte)(byte)165);
                Debug.Assert(pack.exposure_type == (byte)(byte)184);
                Debug.Assert(pack.mode == (byte)(byte)211);
                Debug.Assert(pack.engine_cut_off == (byte)(byte)96);
                Debug.Assert(pack.extra_value == (float)3.694453E37F);
                Debug.Assert(pack.extra_param == (byte)(byte)247);
            };
            GroundControl.DIGICAM_CONFIGURE p154 = CommunicationChannel.new_DIGICAM_CONFIGURE();
            PH.setPack(p154);
            p154.shutter_speed = (ushort)(ushort)58602;
            p154.exposure_type = (byte)(byte)184;
            p154.target_component = (byte)(byte)151;
            p154.target_system = (byte)(byte)165;
            p154.aperture = (byte)(byte)231;
            p154.mode = (byte)(byte)211;
            p154.extra_value = (float)3.694453E37F;
            p154.command_id = (byte)(byte)0;
            p154.engine_cut_off = (byte)(byte)96;
            p154.iso = (byte)(byte)37;
            p154.extra_param = (byte)(byte)247;
            CommunicationChannel.instance.send(p154);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnDIGICAM_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)250);
                Debug.Assert(pack.zoom_step == (sbyte)(sbyte) - 94);
                Debug.Assert(pack.extra_value == (float)2.0977553E38F);
                Debug.Assert(pack.command_id == (byte)(byte)195);
                Debug.Assert(pack.zoom_pos == (byte)(byte)19);
                Debug.Assert(pack.session == (byte)(byte)215);
                Debug.Assert(pack.focus_lock == (byte)(byte)233);
                Debug.Assert(pack.extra_param == (byte)(byte)61);
                Debug.Assert(pack.target_component == (byte)(byte)16);
                Debug.Assert(pack.shot == (byte)(byte)102);
            };
            GroundControl.DIGICAM_CONTROL p155 = CommunicationChannel.new_DIGICAM_CONTROL();
            PH.setPack(p155);
            p155.target_system = (byte)(byte)250;
            p155.focus_lock = (byte)(byte)233;
            p155.target_component = (byte)(byte)16;
            p155.extra_param = (byte)(byte)61;
            p155.shot = (byte)(byte)102;
            p155.zoom_pos = (byte)(byte)19;
            p155.session = (byte)(byte)215;
            p155.extra_value = (float)2.0977553E38F;
            p155.zoom_step = (sbyte)(sbyte) - 94;
            p155.command_id = (byte)(byte)195;
            CommunicationChannel.instance.send(p155);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnMOUNT_CONFIGUREReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.stab_pitch == (byte)(byte)223);
                Debug.Assert(pack.mount_mode == MAV_MOUNT_MODE.MAV_MOUNT_MODE_MAVLINK_TARGETING);
                Debug.Assert(pack.stab_yaw == (byte)(byte)7);
                Debug.Assert(pack.target_system == (byte)(byte)174);
                Debug.Assert(pack.target_component == (byte)(byte)237);
                Debug.Assert(pack.stab_roll == (byte)(byte)182);
            };
            GroundControl.MOUNT_CONFIGURE p156 = CommunicationChannel.new_MOUNT_CONFIGURE();
            PH.setPack(p156);
            p156.mount_mode = MAV_MOUNT_MODE.MAV_MOUNT_MODE_MAVLINK_TARGETING;
            p156.target_system = (byte)(byte)174;
            p156.stab_yaw = (byte)(byte)7;
            p156.stab_roll = (byte)(byte)182;
            p156.target_component = (byte)(byte)237;
            p156.stab_pitch = (byte)(byte)223;
            CommunicationChannel.instance.send(p156);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnMOUNT_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.input_c == (int) -1216413177);
                Debug.Assert(pack.target_system == (byte)(byte)0);
                Debug.Assert(pack.input_a == (int) -2056184571);
                Debug.Assert(pack.input_b == (int)1161465027);
                Debug.Assert(pack.target_component == (byte)(byte)12);
                Debug.Assert(pack.save_position == (byte)(byte)124);
            };
            GroundControl.MOUNT_CONTROL p157 = CommunicationChannel.new_MOUNT_CONTROL();
            PH.setPack(p157);
            p157.input_a = (int) -2056184571;
            p157.target_component = (byte)(byte)12;
            p157.save_position = (byte)(byte)124;
            p157.input_b = (int)1161465027;
            p157.input_c = (int) -1216413177;
            p157.target_system = (byte)(byte)0;
            CommunicationChannel.instance.send(p157);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnMOUNT_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.pointing_c == (int)1977386664);
                Debug.Assert(pack.target_system == (byte)(byte)49);
                Debug.Assert(pack.pointing_b == (int)1878415295);
                Debug.Assert(pack.target_component == (byte)(byte)177);
                Debug.Assert(pack.pointing_a == (int) -215705884);
            };
            GroundControl.MOUNT_STATUS p158 = CommunicationChannel.new_MOUNT_STATUS();
            PH.setPack(p158);
            p158.pointing_c = (int)1977386664;
            p158.target_system = (byte)(byte)49;
            p158.pointing_b = (int)1878415295;
            p158.target_component = (byte)(byte)177;
            p158.pointing_a = (int) -215705884;
            CommunicationChannel.instance.send(p158);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnFENCE_POINTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.idx == (byte)(byte)157);
                Debug.Assert(pack.target_component == (byte)(byte)91);
                Debug.Assert(pack.target_system == (byte)(byte)129);
                Debug.Assert(pack.lat == (float)1.9416908E37F);
                Debug.Assert(pack.lng == (float) -5.9047647E37F);
                Debug.Assert(pack.count == (byte)(byte)193);
            };
            GroundControl.FENCE_POINT p160 = CommunicationChannel.new_FENCE_POINT();
            PH.setPack(p160);
            p160.idx = (byte)(byte)157;
            p160.count = (byte)(byte)193;
            p160.target_component = (byte)(byte)91;
            p160.lng = (float) -5.9047647E37F;
            p160.lat = (float)1.9416908E37F;
            p160.target_system = (byte)(byte)129;
            CommunicationChannel.instance.send(p160);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnFENCE_FETCH_POINTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)215);
                Debug.Assert(pack.target_system == (byte)(byte)121);
                Debug.Assert(pack.idx == (byte)(byte)160);
            };
            GroundControl.FENCE_FETCH_POINT p161 = CommunicationChannel.new_FENCE_FETCH_POINT();
            PH.setPack(p161);
            p161.target_system = (byte)(byte)121;
            p161.idx = (byte)(byte)160;
            p161.target_component = (byte)(byte)215;
            CommunicationChannel.instance.send(p161);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnFENCE_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.breach_count == (ushort)(ushort)23126);
                Debug.Assert(pack.breach_status == (byte)(byte)66);
                Debug.Assert(pack.breach_type == FENCE_BREACH.FENCE_BREACH_BOUNDARY);
                Debug.Assert(pack.breach_time == (uint)2106694924U);
            };
            GroundControl.FENCE_STATUS p162 = CommunicationChannel.new_FENCE_STATUS();
            PH.setPack(p162);
            p162.breach_count = (ushort)(ushort)23126;
            p162.breach_type = FENCE_BREACH.FENCE_BREACH_BOUNDARY;
            p162.breach_time = (uint)2106694924U;
            p162.breach_status = (byte)(byte)66;
            CommunicationChannel.instance.send(p162);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnAHRSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.omegaIz == (float)3.0298502E38F);
                Debug.Assert(pack.omegaIy == (float) -4.5359763E37F);
                Debug.Assert(pack.error_yaw == (float) -1.6304715E38F);
                Debug.Assert(pack.omegaIx == (float)2.5961456E38F);
                Debug.Assert(pack.renorm_val == (float)9.431602E37F);
                Debug.Assert(pack.error_rp == (float) -6.3812E35F);
                Debug.Assert(pack.accel_weight == (float)5.8101183E37F);
            };
            GroundControl.AHRS p163 = CommunicationChannel.new_AHRS();
            PH.setPack(p163);
            p163.error_yaw = (float) -1.6304715E38F;
            p163.error_rp = (float) -6.3812E35F;
            p163.accel_weight = (float)5.8101183E37F;
            p163.omegaIx = (float)2.5961456E38F;
            p163.renorm_val = (float)9.431602E37F;
            p163.omegaIy = (float) -4.5359763E37F;
            p163.omegaIz = (float)3.0298502E38F;
            CommunicationChannel.instance.send(p163);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnSIMSTATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.pitch == (float)3.2227029E38F);
                Debug.Assert(pack.roll == (float)4.637877E37F);
                Debug.Assert(pack.yacc == (float)2.659181E38F);
                Debug.Assert(pack.xgyro == (float) -6.741328E37F);
                Debug.Assert(pack.yaw == (float) -2.08393E38F);
                Debug.Assert(pack.lat == (int) -974944885);
                Debug.Assert(pack.xacc == (float) -8.0107157E37F);
                Debug.Assert(pack.zacc == (float) -8.4692715E37F);
                Debug.Assert(pack.ygyro == (float)1.9025296E38F);
                Debug.Assert(pack.zgyro == (float) -1.4508457E38F);
                Debug.Assert(pack.lng == (int)1987601922);
            };
            GroundControl.SIMSTATE p164 = CommunicationChannel.new_SIMSTATE();
            PH.setPack(p164);
            p164.yacc = (float)2.659181E38F;
            p164.roll = (float)4.637877E37F;
            p164.yaw = (float) -2.08393E38F;
            p164.pitch = (float)3.2227029E38F;
            p164.xacc = (float) -8.0107157E37F;
            p164.zgyro = (float) -1.4508457E38F;
            p164.ygyro = (float)1.9025296E38F;
            p164.zacc = (float) -8.4692715E37F;
            p164.lat = (int) -974944885;
            p164.xgyro = (float) -6.741328E37F;
            p164.lng = (int)1987601922;
            CommunicationChannel.instance.send(p164);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnHWSTATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.Vcc == (ushort)(ushort)63981);
                Debug.Assert(pack.I2Cerr == (byte)(byte)250);
            };
            GroundControl.HWSTATUS p165 = CommunicationChannel.new_HWSTATUS();
            PH.setPack(p165);
            p165.Vcc = (ushort)(ushort)63981;
            p165.I2Cerr = (byte)(byte)250;
            CommunicationChannel.instance.send(p165);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnRADIOReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.txbuf == (byte)(byte)226);
                Debug.Assert(pack.fixed_ == (ushort)(ushort)32582);
                Debug.Assert(pack.rssi == (byte)(byte)108);
                Debug.Assert(pack.remrssi == (byte)(byte)71);
                Debug.Assert(pack.remnoise == (byte)(byte)150);
                Debug.Assert(pack.rxerrors == (ushort)(ushort)58166);
                Debug.Assert(pack.noise == (byte)(byte)45);
            };
            GroundControl.RADIO p166 = CommunicationChannel.new_RADIO();
            PH.setPack(p166);
            p166.fixed_ = (ushort)(ushort)32582;
            p166.txbuf = (byte)(byte)226;
            p166.noise = (byte)(byte)45;
            p166.rxerrors = (ushort)(ushort)58166;
            p166.remrssi = (byte)(byte)71;
            p166.rssi = (byte)(byte)108;
            p166.remnoise = (byte)(byte)150;
            CommunicationChannel.instance.send(p166);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnLIMITS_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.last_action == (uint)4012132568U);
                Debug.Assert(pack.mods_enabled == (LIMIT_MODULE.LIMIT_GEOFENCE |
                                                   LIMIT_MODULE.LIMIT_ALTITUDE));
                Debug.Assert(pack.limits_state == LIMITS_STATE.LIMITS_TRIGGERED);
                Debug.Assert(pack.mods_required == (LIMIT_MODULE.LIMIT_ALTITUDE));
                Debug.Assert(pack.breach_count == (ushort)(ushort)25099);
                Debug.Assert(pack.last_trigger == (uint)1891320859U);
                Debug.Assert(pack.last_clear == (uint)206740290U);
                Debug.Assert(pack.mods_triggered == (LIMIT_MODULE.LIMIT_GEOFENCE));
                Debug.Assert(pack.last_recovery == (uint)966250101U);
            };
            GroundControl.LIMITS_STATUS p167 = CommunicationChannel.new_LIMITS_STATUS();
            PH.setPack(p167);
            p167.mods_enabled = (LIMIT_MODULE.LIMIT_GEOFENCE |
                                 LIMIT_MODULE.LIMIT_ALTITUDE);
            p167.last_trigger = (uint)1891320859U;
            p167.last_action = (uint)4012132568U;
            p167.limits_state = LIMITS_STATE.LIMITS_TRIGGERED;
            p167.breach_count = (ushort)(ushort)25099;
            p167.last_recovery = (uint)966250101U;
            p167.last_clear = (uint)206740290U;
            p167.mods_required = (LIMIT_MODULE.LIMIT_ALTITUDE);
            p167.mods_triggered = (LIMIT_MODULE.LIMIT_GEOFENCE);
            CommunicationChannel.instance.send(p167);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnWINDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.speed_z == (float)1.7286712E38F);
                Debug.Assert(pack.speed == (float) -1.3750439E38F);
                Debug.Assert(pack.direction == (float)1.8902025E38F);
            };
            GroundControl.WIND p168 = CommunicationChannel.new_WIND();
            PH.setPack(p168);
            p168.speed_z = (float)1.7286712E38F;
            p168.direction = (float)1.8902025E38F;
            p168.speed = (float) -1.3750439E38F;
            CommunicationChannel.instance.send(p168);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnDATA16Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.type == (byte)(byte)106);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)104, (byte)250, (byte)69, (byte)214, (byte)201, (byte)155, (byte)226, (byte)119, (byte)29, (byte)123, (byte)112, (byte)45, (byte)233, (byte)135, (byte)226, (byte)118}));
                Debug.Assert(pack.len == (byte)(byte)12);
            };
            GroundControl.DATA16 p169 = CommunicationChannel.new_DATA16();
            PH.setPack(p169);
            p169.data__SET(new byte[] {(byte)104, (byte)250, (byte)69, (byte)214, (byte)201, (byte)155, (byte)226, (byte)119, (byte)29, (byte)123, (byte)112, (byte)45, (byte)233, (byte)135, (byte)226, (byte)118}, 0) ;
            p169.len = (byte)(byte)12;
            p169.type = (byte)(byte)106;
            CommunicationChannel.instance.send(p169);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnDATA32Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)203, (byte)58, (byte)231, (byte)89, (byte)137, (byte)71, (byte)207, (byte)94, (byte)51, (byte)65, (byte)115, (byte)215, (byte)207, (byte)86, (byte)99, (byte)77, (byte)78, (byte)234, (byte)161, (byte)57, (byte)183, (byte)28, (byte)114, (byte)217, (byte)87, (byte)241, (byte)225, (byte)112, (byte)135, (byte)173, (byte)230, (byte)67}));
                Debug.Assert(pack.type == (byte)(byte)201);
                Debug.Assert(pack.len == (byte)(byte)120);
            };
            GroundControl.DATA32 p170 = CommunicationChannel.new_DATA32();
            PH.setPack(p170);
            p170.data__SET(new byte[] {(byte)203, (byte)58, (byte)231, (byte)89, (byte)137, (byte)71, (byte)207, (byte)94, (byte)51, (byte)65, (byte)115, (byte)215, (byte)207, (byte)86, (byte)99, (byte)77, (byte)78, (byte)234, (byte)161, (byte)57, (byte)183, (byte)28, (byte)114, (byte)217, (byte)87, (byte)241, (byte)225, (byte)112, (byte)135, (byte)173, (byte)230, (byte)67}, 0) ;
            p170.len = (byte)(byte)120;
            p170.type = (byte)(byte)201;
            CommunicationChannel.instance.send(p170);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnDATA64Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)231, (byte)51, (byte)191, (byte)70, (byte)161, (byte)175, (byte)6, (byte)169, (byte)77, (byte)255, (byte)23, (byte)201, (byte)142, (byte)154, (byte)128, (byte)138, (byte)128, (byte)62, (byte)219, (byte)105, (byte)98, (byte)160, (byte)93, (byte)133, (byte)4, (byte)237, (byte)137, (byte)82, (byte)121, (byte)250, (byte)29, (byte)184, (byte)246, (byte)2, (byte)147, (byte)227, (byte)38, (byte)148, (byte)63, (byte)64, (byte)30, (byte)232, (byte)183, (byte)122, (byte)13, (byte)191, (byte)71, (byte)25, (byte)72, (byte)160, (byte)205, (byte)174, (byte)68, (byte)225, (byte)94, (byte)157, (byte)217, (byte)2, (byte)100, (byte)63, (byte)154, (byte)47, (byte)242, (byte)164}));
                Debug.Assert(pack.type == (byte)(byte)14);
                Debug.Assert(pack.len == (byte)(byte)189);
            };
            GroundControl.DATA64 p171 = CommunicationChannel.new_DATA64();
            PH.setPack(p171);
            p171.len = (byte)(byte)189;
            p171.data__SET(new byte[] {(byte)231, (byte)51, (byte)191, (byte)70, (byte)161, (byte)175, (byte)6, (byte)169, (byte)77, (byte)255, (byte)23, (byte)201, (byte)142, (byte)154, (byte)128, (byte)138, (byte)128, (byte)62, (byte)219, (byte)105, (byte)98, (byte)160, (byte)93, (byte)133, (byte)4, (byte)237, (byte)137, (byte)82, (byte)121, (byte)250, (byte)29, (byte)184, (byte)246, (byte)2, (byte)147, (byte)227, (byte)38, (byte)148, (byte)63, (byte)64, (byte)30, (byte)232, (byte)183, (byte)122, (byte)13, (byte)191, (byte)71, (byte)25, (byte)72, (byte)160, (byte)205, (byte)174, (byte)68, (byte)225, (byte)94, (byte)157, (byte)217, (byte)2, (byte)100, (byte)63, (byte)154, (byte)47, (byte)242, (byte)164}, 0) ;
            p171.type = (byte)(byte)14;
            CommunicationChannel.instance.send(p171);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnDATA96Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.type == (byte)(byte)66);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)126, (byte)152, (byte)188, (byte)116, (byte)129, (byte)126, (byte)252, (byte)75, (byte)194, (byte)99, (byte)115, (byte)253, (byte)56, (byte)134, (byte)11, (byte)1, (byte)0, (byte)191, (byte)177, (byte)86, (byte)139, (byte)190, (byte)55, (byte)196, (byte)8, (byte)56, (byte)250, (byte)25, (byte)81, (byte)61, (byte)243, (byte)70, (byte)167, (byte)173, (byte)209, (byte)138, (byte)108, (byte)54, (byte)96, (byte)79, (byte)14, (byte)126, (byte)250, (byte)57, (byte)223, (byte)14, (byte)153, (byte)196, (byte)251, (byte)200, (byte)46, (byte)112, (byte)10, (byte)10, (byte)245, (byte)8, (byte)49, (byte)255, (byte)242, (byte)220, (byte)10, (byte)169, (byte)6, (byte)152, (byte)224, (byte)99, (byte)26, (byte)138, (byte)104, (byte)237, (byte)12, (byte)67, (byte)154, (byte)74, (byte)105, (byte)208, (byte)124, (byte)51, (byte)174, (byte)127, (byte)136, (byte)114, (byte)95, (byte)244, (byte)131, (byte)85, (byte)191, (byte)217, (byte)187, (byte)34, (byte)243, (byte)75, (byte)230, (byte)106, (byte)231, (byte)28}));
                Debug.Assert(pack.len == (byte)(byte)154);
            };
            GroundControl.DATA96 p172 = CommunicationChannel.new_DATA96();
            PH.setPack(p172);
            p172.len = (byte)(byte)154;
            p172.type = (byte)(byte)66;
            p172.data__SET(new byte[] {(byte)126, (byte)152, (byte)188, (byte)116, (byte)129, (byte)126, (byte)252, (byte)75, (byte)194, (byte)99, (byte)115, (byte)253, (byte)56, (byte)134, (byte)11, (byte)1, (byte)0, (byte)191, (byte)177, (byte)86, (byte)139, (byte)190, (byte)55, (byte)196, (byte)8, (byte)56, (byte)250, (byte)25, (byte)81, (byte)61, (byte)243, (byte)70, (byte)167, (byte)173, (byte)209, (byte)138, (byte)108, (byte)54, (byte)96, (byte)79, (byte)14, (byte)126, (byte)250, (byte)57, (byte)223, (byte)14, (byte)153, (byte)196, (byte)251, (byte)200, (byte)46, (byte)112, (byte)10, (byte)10, (byte)245, (byte)8, (byte)49, (byte)255, (byte)242, (byte)220, (byte)10, (byte)169, (byte)6, (byte)152, (byte)224, (byte)99, (byte)26, (byte)138, (byte)104, (byte)237, (byte)12, (byte)67, (byte)154, (byte)74, (byte)105, (byte)208, (byte)124, (byte)51, (byte)174, (byte)127, (byte)136, (byte)114, (byte)95, (byte)244, (byte)131, (byte)85, (byte)191, (byte)217, (byte)187, (byte)34, (byte)243, (byte)75, (byte)230, (byte)106, (byte)231, (byte)28}, 0) ;
            CommunicationChannel.instance.send(p172);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnRANGEFINDERReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.voltage == (float) -2.6232634E38F);
                Debug.Assert(pack.distance == (float) -2.222873E38F);
            };
            GroundControl.RANGEFINDER p173 = CommunicationChannel.new_RANGEFINDER();
            PH.setPack(p173);
            p173.voltage = (float) -2.6232634E38F;
            p173.distance = (float) -2.222873E38F;
            CommunicationChannel.instance.send(p173);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnAIRSPEED_AUTOCALReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vz == (float) -2.7758785E37F);
                Debug.Assert(pack.Pax == (float) -4.907291E37F);
                Debug.Assert(pack.state_x == (float)3.127304E38F);
                Debug.Assert(pack.state_z == (float)1.0213407E38F);
                Debug.Assert(pack.state_y == (float)2.9716612E38F);
                Debug.Assert(pack.Pcz == (float) -2.465387E38F);
                Debug.Assert(pack.vx == (float)2.8657917E38F);
                Debug.Assert(pack.diff_pressure == (float) -3.2907408E38F);
                Debug.Assert(pack.vy == (float)3.307809E38F);
                Debug.Assert(pack.Pby == (float) -2.9958387E38F);
                Debug.Assert(pack.ratio == (float) -1.838596E38F);
                Debug.Assert(pack.EAS2TAS == (float)2.8529919E38F);
            };
            GroundControl.AIRSPEED_AUTOCAL p174 = CommunicationChannel.new_AIRSPEED_AUTOCAL();
            PH.setPack(p174);
            p174.state_z = (float)1.0213407E38F;
            p174.Pcz = (float) -2.465387E38F;
            p174.EAS2TAS = (float)2.8529919E38F;
            p174.state_x = (float)3.127304E38F;
            p174.diff_pressure = (float) -3.2907408E38F;
            p174.Pby = (float) -2.9958387E38F;
            p174.vz = (float) -2.7758785E37F;
            p174.vx = (float)2.8657917E38F;
            p174.state_y = (float)2.9716612E38F;
            p174.vy = (float)3.307809E38F;
            p174.ratio = (float) -1.838596E38F;
            p174.Pax = (float) -4.907291E37F;
            CommunicationChannel.instance.send(p174);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnRALLY_POINTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.flags == RALLY_FLAGS.LAND_IMMEDIATELY);
                Debug.Assert(pack.count == (byte)(byte)207);
                Debug.Assert(pack.target_system == (byte)(byte)212);
                Debug.Assert(pack.target_component == (byte)(byte)61);
                Debug.Assert(pack.lng == (int)1287653965);
                Debug.Assert(pack.break_alt == (short)(short) -2796);
                Debug.Assert(pack.land_dir == (ushort)(ushort)17189);
                Debug.Assert(pack.lat == (int) -1389816576);
                Debug.Assert(pack.alt == (short)(short) -256);
                Debug.Assert(pack.idx == (byte)(byte)238);
            };
            GroundControl.RALLY_POINT p175 = CommunicationChannel.new_RALLY_POINT();
            PH.setPack(p175);
            p175.flags = RALLY_FLAGS.LAND_IMMEDIATELY;
            p175.land_dir = (ushort)(ushort)17189;
            p175.target_system = (byte)(byte)212;
            p175.idx = (byte)(byte)238;
            p175.count = (byte)(byte)207;
            p175.lng = (int)1287653965;
            p175.break_alt = (short)(short) -2796;
            p175.lat = (int) -1389816576;
            p175.alt = (short)(short) -256;
            p175.target_component = (byte)(byte)61;
            CommunicationChannel.instance.send(p175);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnRALLY_FETCH_POINTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)17);
                Debug.Assert(pack.idx == (byte)(byte)35);
                Debug.Assert(pack.target_component == (byte)(byte)91);
            };
            GroundControl.RALLY_FETCH_POINT p176 = CommunicationChannel.new_RALLY_FETCH_POINT();
            PH.setPack(p176);
            p176.idx = (byte)(byte)35;
            p176.target_system = (byte)(byte)17;
            p176.target_component = (byte)(byte)91;
            CommunicationChannel.instance.send(p176);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnCOMPASSMOT_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.interference == (ushort)(ushort)33158);
                Debug.Assert(pack.CompensationZ == (float)3.2350103E38F);
                Debug.Assert(pack.current == (float) -1.7296358E38F);
                Debug.Assert(pack.CompensationX == (float)2.689751E38F);
                Debug.Assert(pack.throttle == (ushort)(ushort)61793);
                Debug.Assert(pack.CompensationY == (float)3.7645973E37F);
            };
            GroundControl.COMPASSMOT_STATUS p177 = CommunicationChannel.new_COMPASSMOT_STATUS();
            PH.setPack(p177);
            p177.throttle = (ushort)(ushort)61793;
            p177.CompensationY = (float)3.7645973E37F;
            p177.CompensationZ = (float)3.2350103E38F;
            p177.current = (float) -1.7296358E38F;
            p177.interference = (ushort)(ushort)33158;
            p177.CompensationX = (float)2.689751E38F;
            CommunicationChannel.instance.send(p177);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnAHRS2Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.roll == (float) -1.1548063E38F);
                Debug.Assert(pack.yaw == (float) -2.2985676E38F);
                Debug.Assert(pack.lat == (int) -979156026);
                Debug.Assert(pack.pitch == (float) -2.0453182E37F);
                Debug.Assert(pack.altitude == (float)4.060219E36F);
                Debug.Assert(pack.lng == (int)1541913596);
            };
            GroundControl.AHRS2 p178 = CommunicationChannel.new_AHRS2();
            PH.setPack(p178);
            p178.lat = (int) -979156026;
            p178.altitude = (float)4.060219E36F;
            p178.roll = (float) -1.1548063E38F;
            p178.yaw = (float) -2.2985676E38F;
            p178.pitch = (float) -2.0453182E37F;
            p178.lng = (int)1541913596;
            CommunicationChannel.instance.send(p178);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnCAMERA_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.p2 == (float)2.6639217E38F);
                Debug.Assert(pack.img_idx == (ushort)(ushort)8809);
                Debug.Assert(pack.target_system == (byte)(byte)19);
                Debug.Assert(pack.p3 == (float) -2.56889E38F);
                Debug.Assert(pack.p4 == (float) -3.3015323E38F);
                Debug.Assert(pack.event_id == CAMERA_STATUS_TYPES.CAMERA_STATUS_TYPE_LOWSTORE);
                Debug.Assert(pack.cam_idx == (byte)(byte)129);
                Debug.Assert(pack.time_usec == (ulong)6038780181893468835L);
                Debug.Assert(pack.p1 == (float) -2.1515146E38F);
            };
            GroundControl.CAMERA_STATUS p179 = CommunicationChannel.new_CAMERA_STATUS();
            PH.setPack(p179);
            p179.cam_idx = (byte)(byte)129;
            p179.p2 = (float)2.6639217E38F;
            p179.p1 = (float) -2.1515146E38F;
            p179.time_usec = (ulong)6038780181893468835L;
            p179.img_idx = (ushort)(ushort)8809;
            p179.p3 = (float) -2.56889E38F;
            p179.event_id = CAMERA_STATUS_TYPES.CAMERA_STATUS_TYPE_LOWSTORE;
            p179.p4 = (float) -3.3015323E38F;
            p179.target_system = (byte)(byte)19;
            CommunicationChannel.instance.send(p179);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnCAMERA_FEEDBACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.img_idx == (ushort)(ushort)10405);
                Debug.Assert(pack.alt_msl == (float) -4.84438E35F);
                Debug.Assert(pack.target_system == (byte)(byte)197);
                Debug.Assert(pack.roll == (float)1.9256292E37F);
                Debug.Assert(pack.alt_rel == (float) -1.9295202E38F);
                Debug.Assert(pack.lng == (int) -766220637);
                Debug.Assert(pack.pitch == (float) -2.7509894E38F);
                Debug.Assert(pack.foc_len == (float) -1.0803704E38F);
                Debug.Assert(pack.lat == (int)338129724);
                Debug.Assert(pack.time_usec == (ulong)6027741642353123581L);
                Debug.Assert(pack.cam_idx == (byte)(byte)164);
                Debug.Assert(pack.yaw == (float) -3.1307043E38F);
                Debug.Assert(pack.flags == CAMERA_FEEDBACK_FLAGS.CAMERA_FEEDBACK_CLOSEDLOOP);
            };
            GroundControl.CAMERA_FEEDBACK p180 = CommunicationChannel.new_CAMERA_FEEDBACK();
            PH.setPack(p180);
            p180.target_system = (byte)(byte)197;
            p180.yaw = (float) -3.1307043E38F;
            p180.pitch = (float) -2.7509894E38F;
            p180.alt_rel = (float) -1.9295202E38F;
            p180.lng = (int) -766220637;
            p180.lat = (int)338129724;
            p180.cam_idx = (byte)(byte)164;
            p180.foc_len = (float) -1.0803704E38F;
            p180.img_idx = (ushort)(ushort)10405;
            p180.flags = CAMERA_FEEDBACK_FLAGS.CAMERA_FEEDBACK_CLOSEDLOOP;
            p180.alt_msl = (float) -4.84438E35F;
            p180.time_usec = (ulong)6027741642353123581L;
            p180.roll = (float)1.9256292E37F;
            CommunicationChannel.instance.send(p180);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnBATTERY2Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.voltage == (ushort)(ushort)36006);
                Debug.Assert(pack.current_battery == (short)(short) -14569);
            };
            GroundControl.BATTERY2 p181 = CommunicationChannel.new_BATTERY2();
            PH.setPack(p181);
            p181.voltage = (ushort)(ushort)36006;
            p181.current_battery = (short)(short) -14569;
            CommunicationChannel.instance.send(p181);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnAHRS3Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.v4 == (float)2.4326266E37F);
                Debug.Assert(pack.v1 == (float) -6.3815565E37F);
                Debug.Assert(pack.lat == (int)1616448904);
                Debug.Assert(pack.pitch == (float) -1.8941032E38F);
                Debug.Assert(pack.lng == (int)538259233);
                Debug.Assert(pack.v3 == (float)1.1077289E37F);
                Debug.Assert(pack.yaw == (float)9.252894E37F);
                Debug.Assert(pack.roll == (float)2.363001E38F);
                Debug.Assert(pack.v2 == (float) -7.075054E37F);
                Debug.Assert(pack.altitude == (float) -5.975821E37F);
            };
            GroundControl.AHRS3 p182 = CommunicationChannel.new_AHRS3();
            PH.setPack(p182);
            p182.v4 = (float)2.4326266E37F;
            p182.roll = (float)2.363001E38F;
            p182.lng = (int)538259233;
            p182.pitch = (float) -1.8941032E38F;
            p182.v3 = (float)1.1077289E37F;
            p182.lat = (int)1616448904;
            p182.v1 = (float) -6.3815565E37F;
            p182.v2 = (float) -7.075054E37F;
            p182.altitude = (float) -5.975821E37F;
            p182.yaw = (float)9.252894E37F;
            CommunicationChannel.instance.send(p182);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnAUTOPILOT_VERSION_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)182);
                Debug.Assert(pack.target_component == (byte)(byte)73);
            };
            GroundControl.AUTOPILOT_VERSION_REQUEST p183 = CommunicationChannel.new_AUTOPILOT_VERSION_REQUEST();
            PH.setPack(p183);
            p183.target_component = (byte)(byte)73;
            p183.target_system = (byte)(byte)182;
            CommunicationChannel.instance.send(p183);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnREMOTE_LOG_DATA_BLOCKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)149);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)151, (byte)89, (byte)230, (byte)225, (byte)206, (byte)231, (byte)66, (byte)225, (byte)37, (byte)240, (byte)202, (byte)49, (byte)179, (byte)185, (byte)117, (byte)29, (byte)143, (byte)205, (byte)169, (byte)181, (byte)43, (byte)139, (byte)129, (byte)121, (byte)253, (byte)196, (byte)174, (byte)52, (byte)168, (byte)116, (byte)64, (byte)10, (byte)43, (byte)24, (byte)144, (byte)138, (byte)133, (byte)175, (byte)192, (byte)237, (byte)80, (byte)118, (byte)129, (byte)27, (byte)199, (byte)67, (byte)185, (byte)235, (byte)245, (byte)63, (byte)39, (byte)192, (byte)103, (byte)142, (byte)217, (byte)96, (byte)224, (byte)170, (byte)83, (byte)212, (byte)77, (byte)230, (byte)235, (byte)32, (byte)193, (byte)90, (byte)60, (byte)150, (byte)144, (byte)14, (byte)146, (byte)26, (byte)35, (byte)254, (byte)26, (byte)185, (byte)193, (byte)159, (byte)41, (byte)141, (byte)205, (byte)67, (byte)224, (byte)40, (byte)33, (byte)115, (byte)196, (byte)10, (byte)48, (byte)198, (byte)221, (byte)142, (byte)138, (byte)248, (byte)197, (byte)130, (byte)237, (byte)41, (byte)194, (byte)102, (byte)167, (byte)155, (byte)152, (byte)167, (byte)200, (byte)96, (byte)213, (byte)104, (byte)46, (byte)239, (byte)229, (byte)75, (byte)92, (byte)192, (byte)69, (byte)252, (byte)171, (byte)77, (byte)226, (byte)192, (byte)22, (byte)93, (byte)135, (byte)190, (byte)218, (byte)122, (byte)177, (byte)233, (byte)154, (byte)12, (byte)249, (byte)102, (byte)88, (byte)47, (byte)187, (byte)245, (byte)113, (byte)110, (byte)180, (byte)196, (byte)229, (byte)108, (byte)134, (byte)204, (byte)72, (byte)4, (byte)153, (byte)103, (byte)251, (byte)206, (byte)184, (byte)105, (byte)67, (byte)189, (byte)178, (byte)206, (byte)137, (byte)102, (byte)231, (byte)177, (byte)202, (byte)0, (byte)70, (byte)0, (byte)79, (byte)103, (byte)11, (byte)97, (byte)49, (byte)84, (byte)64, (byte)123, (byte)35, (byte)7, (byte)230, (byte)148, (byte)133, (byte)115, (byte)100, (byte)222, (byte)151, (byte)36, (byte)239, (byte)102, (byte)4, (byte)8, (byte)109, (byte)81, (byte)69, (byte)39, (byte)244, (byte)233, (byte)4, (byte)243, (byte)238, (byte)255, (byte)74, (byte)210, (byte)186, (byte)185}));
                Debug.Assert(pack.seqno == MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS.MAV_REMOTE_LOG_DATA_BLOCK_START);
                Debug.Assert(pack.target_system == (byte)(byte)141);
            };
            GroundControl.REMOTE_LOG_DATA_BLOCK p184 = CommunicationChannel.new_REMOTE_LOG_DATA_BLOCK();
            PH.setPack(p184);
            p184.seqno = MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS.MAV_REMOTE_LOG_DATA_BLOCK_START;
            p184.data__SET(new byte[] {(byte)151, (byte)89, (byte)230, (byte)225, (byte)206, (byte)231, (byte)66, (byte)225, (byte)37, (byte)240, (byte)202, (byte)49, (byte)179, (byte)185, (byte)117, (byte)29, (byte)143, (byte)205, (byte)169, (byte)181, (byte)43, (byte)139, (byte)129, (byte)121, (byte)253, (byte)196, (byte)174, (byte)52, (byte)168, (byte)116, (byte)64, (byte)10, (byte)43, (byte)24, (byte)144, (byte)138, (byte)133, (byte)175, (byte)192, (byte)237, (byte)80, (byte)118, (byte)129, (byte)27, (byte)199, (byte)67, (byte)185, (byte)235, (byte)245, (byte)63, (byte)39, (byte)192, (byte)103, (byte)142, (byte)217, (byte)96, (byte)224, (byte)170, (byte)83, (byte)212, (byte)77, (byte)230, (byte)235, (byte)32, (byte)193, (byte)90, (byte)60, (byte)150, (byte)144, (byte)14, (byte)146, (byte)26, (byte)35, (byte)254, (byte)26, (byte)185, (byte)193, (byte)159, (byte)41, (byte)141, (byte)205, (byte)67, (byte)224, (byte)40, (byte)33, (byte)115, (byte)196, (byte)10, (byte)48, (byte)198, (byte)221, (byte)142, (byte)138, (byte)248, (byte)197, (byte)130, (byte)237, (byte)41, (byte)194, (byte)102, (byte)167, (byte)155, (byte)152, (byte)167, (byte)200, (byte)96, (byte)213, (byte)104, (byte)46, (byte)239, (byte)229, (byte)75, (byte)92, (byte)192, (byte)69, (byte)252, (byte)171, (byte)77, (byte)226, (byte)192, (byte)22, (byte)93, (byte)135, (byte)190, (byte)218, (byte)122, (byte)177, (byte)233, (byte)154, (byte)12, (byte)249, (byte)102, (byte)88, (byte)47, (byte)187, (byte)245, (byte)113, (byte)110, (byte)180, (byte)196, (byte)229, (byte)108, (byte)134, (byte)204, (byte)72, (byte)4, (byte)153, (byte)103, (byte)251, (byte)206, (byte)184, (byte)105, (byte)67, (byte)189, (byte)178, (byte)206, (byte)137, (byte)102, (byte)231, (byte)177, (byte)202, (byte)0, (byte)70, (byte)0, (byte)79, (byte)103, (byte)11, (byte)97, (byte)49, (byte)84, (byte)64, (byte)123, (byte)35, (byte)7, (byte)230, (byte)148, (byte)133, (byte)115, (byte)100, (byte)222, (byte)151, (byte)36, (byte)239, (byte)102, (byte)4, (byte)8, (byte)109, (byte)81, (byte)69, (byte)39, (byte)244, (byte)233, (byte)4, (byte)243, (byte)238, (byte)255, (byte)74, (byte)210, (byte)186, (byte)185}, 0) ;
            p184.target_component = (byte)(byte)149;
            p184.target_system = (byte)(byte)141;
            CommunicationChannel.instance.send(p184);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnREMOTE_LOG_BLOCK_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.status == MAV_REMOTE_LOG_DATA_BLOCK_STATUSES.MAV_REMOTE_LOG_DATA_BLOCK_NACK);
                Debug.Assert(pack.seqno == (uint)2868738705U);
                Debug.Assert(pack.target_system == (byte)(byte)217);
                Debug.Assert(pack.target_component == (byte)(byte)72);
            };
            GroundControl.REMOTE_LOG_BLOCK_STATUS p185 = CommunicationChannel.new_REMOTE_LOG_BLOCK_STATUS();
            PH.setPack(p185);
            p185.status = MAV_REMOTE_LOG_DATA_BLOCK_STATUSES.MAV_REMOTE_LOG_DATA_BLOCK_NACK;
            p185.target_component = (byte)(byte)72;
            p185.seqno = (uint)2868738705U;
            p185.target_system = (byte)(byte)217;
            CommunicationChannel.instance.send(p185);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnLED_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.pattern == (byte)(byte)89);
                Debug.Assert(pack.instance == (byte)(byte)67);
                Debug.Assert(pack.custom_bytes.SequenceEqual(new byte[] {(byte)48, (byte)250, (byte)43, (byte)181, (byte)172, (byte)90, (byte)139, (byte)138, (byte)107, (byte)90, (byte)152, (byte)76, (byte)62, (byte)83, (byte)41, (byte)218, (byte)136, (byte)190, (byte)14, (byte)86, (byte)63, (byte)163, (byte)45, (byte)132}));
                Debug.Assert(pack.custom_len == (byte)(byte)124);
                Debug.Assert(pack.target_system == (byte)(byte)134);
                Debug.Assert(pack.target_component == (byte)(byte)14);
            };
            GroundControl.LED_CONTROL p186 = CommunicationChannel.new_LED_CONTROL();
            PH.setPack(p186);
            p186.pattern = (byte)(byte)89;
            p186.target_component = (byte)(byte)14;
            p186.custom_len = (byte)(byte)124;
            p186.custom_bytes_SET(new byte[] {(byte)48, (byte)250, (byte)43, (byte)181, (byte)172, (byte)90, (byte)139, (byte)138, (byte)107, (byte)90, (byte)152, (byte)76, (byte)62, (byte)83, (byte)41, (byte)218, (byte)136, (byte)190, (byte)14, (byte)86, (byte)63, (byte)163, (byte)45, (byte)132}, 0) ;
            p186.instance = (byte)(byte)67;
            p186.target_system = (byte)(byte)134;
            CommunicationChannel.instance.send(p186);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnMAG_CAL_PROGRESSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.direction_y == (float) -2.7600729E38F);
                Debug.Assert(pack.direction_z == (float) -1.1161556E38F);
                Debug.Assert(pack.completion_pct == (byte)(byte)109);
                Debug.Assert(pack.attempt == (byte)(byte)19);
                Debug.Assert(pack.compass_id == (byte)(byte)30);
                Debug.Assert(pack.direction_x == (float)3.1554075E38F);
                Debug.Assert(pack.completion_mask.SequenceEqual(new byte[] {(byte)200, (byte)224, (byte)127, (byte)70, (byte)45, (byte)144, (byte)204, (byte)125, (byte)235, (byte)97}));
                Debug.Assert(pack.cal_status == MAG_CAL_STATUS.MAG_CAL_RUNNING_STEP_ONE);
                Debug.Assert(pack.cal_mask == (byte)(byte)239);
            };
            GroundControl.MAG_CAL_PROGRESS p191 = CommunicationChannel.new_MAG_CAL_PROGRESS();
            PH.setPack(p191);
            p191.compass_id = (byte)(byte)30;
            p191.direction_y = (float) -2.7600729E38F;
            p191.cal_status = MAG_CAL_STATUS.MAG_CAL_RUNNING_STEP_ONE;
            p191.completion_pct = (byte)(byte)109;
            p191.cal_mask = (byte)(byte)239;
            p191.completion_mask_SET(new byte[] {(byte)200, (byte)224, (byte)127, (byte)70, (byte)45, (byte)144, (byte)204, (byte)125, (byte)235, (byte)97}, 0) ;
            p191.attempt = (byte)(byte)19;
            p191.direction_x = (float)3.1554075E38F;
            p191.direction_z = (float) -1.1161556E38F;
            CommunicationChannel.instance.send(p191);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnMAG_CAL_REPORTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.cal_mask == (byte)(byte)148);
                Debug.Assert(pack.diag_y == (float)1.19834E38F);
                Debug.Assert(pack.compass_id == (byte)(byte)86);
                Debug.Assert(pack.autosaved == (byte)(byte)86);
                Debug.Assert(pack.diag_x == (float)4.6158125E37F);
                Debug.Assert(pack.offdiag_x == (float) -3.391917E38F);
                Debug.Assert(pack.offdiag_y == (float) -2.7468672E38F);
                Debug.Assert(pack.ofs_y == (float)3.3793005E38F);
                Debug.Assert(pack.cal_status == MAG_CAL_STATUS.MAG_CAL_NOT_STARTED);
                Debug.Assert(pack.ofs_x == (float)1.6749739E37F);
                Debug.Assert(pack.diag_z == (float) -6.8497566E37F);
                Debug.Assert(pack.fitness == (float) -2.1889922E38F);
                Debug.Assert(pack.offdiag_z == (float) -1.893803E37F);
                Debug.Assert(pack.ofs_z == (float)1.7003106E38F);
            };
            GroundControl.MAG_CAL_REPORT p192 = CommunicationChannel.new_MAG_CAL_REPORT();
            PH.setPack(p192);
            p192.cal_mask = (byte)(byte)148;
            p192.offdiag_z = (float) -1.893803E37F;
            p192.ofs_x = (float)1.6749739E37F;
            p192.offdiag_x = (float) -3.391917E38F;
            p192.offdiag_y = (float) -2.7468672E38F;
            p192.diag_y = (float)1.19834E38F;
            p192.diag_x = (float)4.6158125E37F;
            p192.ofs_z = (float)1.7003106E38F;
            p192.ofs_y = (float)3.3793005E38F;
            p192.compass_id = (byte)(byte)86;
            p192.autosaved = (byte)(byte)86;
            p192.fitness = (float) -2.1889922E38F;
            p192.diag_z = (float) -6.8497566E37F;
            p192.cal_status = MAG_CAL_STATUS.MAG_CAL_NOT_STARTED;
            CommunicationChannel.instance.send(p192);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnEKF_STATUS_REPORTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.terrain_alt_variance == (float)3.3714324E38F);
                Debug.Assert(pack.flags == (EKF_STATUS_FLAGS.EKF_POS_HORIZ_ABS |
                                            EKF_STATUS_FLAGS.EKF_POS_HORIZ_REL |
                                            EKF_STATUS_FLAGS.EKF_ATTITUDE));
                Debug.Assert(pack.velocity_variance == (float)2.1684017E38F);
                Debug.Assert(pack.pos_vert_variance == (float) -2.0458315E38F);
                Debug.Assert(pack.pos_horiz_variance == (float) -6.5368006E37F);
                Debug.Assert(pack.compass_variance == (float)1.5092788E38F);
            };
            GroundControl.EKF_STATUS_REPORT p193 = CommunicationChannel.new_EKF_STATUS_REPORT();
            PH.setPack(p193);
            p193.compass_variance = (float)1.5092788E38F;
            p193.flags = (EKF_STATUS_FLAGS.EKF_POS_HORIZ_ABS |
                          EKF_STATUS_FLAGS.EKF_POS_HORIZ_REL |
                          EKF_STATUS_FLAGS.EKF_ATTITUDE);
            p193.pos_horiz_variance = (float) -6.5368006E37F;
            p193.pos_vert_variance = (float) -2.0458315E38F;
            p193.terrain_alt_variance = (float)3.3714324E38F;
            p193.velocity_variance = (float)2.1684017E38F;
            CommunicationChannel.instance.send(p193);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnPID_TUNINGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.I == (float)1.2350082E38F);
                Debug.Assert(pack.P == (float) -6.859365E37F);
                Debug.Assert(pack.FF == (float) -1.6385108E38F);
                Debug.Assert(pack.achieved == (float) -3.2557892E38F);
                Debug.Assert(pack.desired == (float)2.5326872E38F);
                Debug.Assert(pack.axis == PID_TUNING_AXIS.PID_TUNING_PITCH);
                Debug.Assert(pack.D == (float) -1.8062196E38F);
            };
            GroundControl.PID_TUNING p194 = CommunicationChannel.new_PID_TUNING();
            PH.setPack(p194);
            p194.axis = PID_TUNING_AXIS.PID_TUNING_PITCH;
            p194.I = (float)1.2350082E38F;
            p194.P = (float) -6.859365E37F;
            p194.D = (float) -1.8062196E38F;
            p194.FF = (float) -1.6385108E38F;
            p194.desired = (float)2.5326872E38F;
            p194.achieved = (float) -3.2557892E38F;
            CommunicationChannel.instance.send(p194);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnGIMBAL_REPORTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)193);
                Debug.Assert(pack.joint_az == (float)2.5739477E38F);
                Debug.Assert(pack.delta_velocity_z == (float)2.357876E38F);
                Debug.Assert(pack.joint_el == (float)1.5861019E36F);
                Debug.Assert(pack.target_component == (byte)(byte)164);
                Debug.Assert(pack.delta_velocity_y == (float)3.0358323E38F);
                Debug.Assert(pack.delta_angle_x == (float) -1.6682445E38F);
                Debug.Assert(pack.delta_time == (float) -3.104189E38F);
                Debug.Assert(pack.delta_velocity_x == (float) -6.666E37F);
                Debug.Assert(pack.delta_angle_z == (float)1.0078742E38F);
                Debug.Assert(pack.joint_roll == (float)3.9983409E37F);
                Debug.Assert(pack.delta_angle_y == (float) -3.2413676E38F);
            };
            GroundControl.GIMBAL_REPORT p200 = CommunicationChannel.new_GIMBAL_REPORT();
            PH.setPack(p200);
            p200.delta_angle_x = (float) -1.6682445E38F;
            p200.delta_velocity_x = (float) -6.666E37F;
            p200.target_system = (byte)(byte)193;
            p200.joint_az = (float)2.5739477E38F;
            p200.target_component = (byte)(byte)164;
            p200.delta_velocity_z = (float)2.357876E38F;
            p200.delta_time = (float) -3.104189E38F;
            p200.delta_velocity_y = (float)3.0358323E38F;
            p200.delta_angle_z = (float)1.0078742E38F;
            p200.delta_angle_y = (float) -3.2413676E38F;
            p200.joint_el = (float)1.5861019E36F;
            p200.joint_roll = (float)3.9983409E37F;
            CommunicationChannel.instance.send(p200);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnGIMBAL_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.demanded_rate_x == (float)1.0920252E38F);
                Debug.Assert(pack.target_component == (byte)(byte)169);
                Debug.Assert(pack.demanded_rate_y == (float)6.4426653E37F);
                Debug.Assert(pack.demanded_rate_z == (float) -8.689538E37F);
                Debug.Assert(pack.target_system == (byte)(byte)109);
            };
            GroundControl.GIMBAL_CONTROL p201 = CommunicationChannel.new_GIMBAL_CONTROL();
            PH.setPack(p201);
            p201.target_component = (byte)(byte)169;
            p201.demanded_rate_y = (float)6.4426653E37F;
            p201.demanded_rate_x = (float)1.0920252E38F;
            p201.demanded_rate_z = (float) -8.689538E37F;
            p201.target_system = (byte)(byte)109;
            CommunicationChannel.instance.send(p201);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnGIMBAL_TORQUE_CMD_REPORTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.el_torque_cmd == (short)(short) -7500);
                Debug.Assert(pack.target_component == (byte)(byte)229);
                Debug.Assert(pack.target_system == (byte)(byte)195);
                Debug.Assert(pack.rl_torque_cmd == (short)(short) -14891);
                Debug.Assert(pack.az_torque_cmd == (short)(short) -22815);
            };
            GroundControl.GIMBAL_TORQUE_CMD_REPORT p214 = CommunicationChannel.new_GIMBAL_TORQUE_CMD_REPORT();
            PH.setPack(p214);
            p214.target_system = (byte)(byte)195;
            p214.az_torque_cmd = (short)(short) -22815;
            p214.target_component = (byte)(byte)229;
            p214.rl_torque_cmd = (short)(short) -14891;
            p214.el_torque_cmd = (short)(short) -7500;
            CommunicationChannel.instance.send(p214);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnGOPRO_HEARTBEATReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.capture_mode == GOPRO_CAPTURE_MODE.GOPRO_CAPTURE_MODE_VIDEO);
                Debug.Assert(pack.flags == GOPRO_HEARTBEAT_FLAGS.GOPRO_FLAG_RECORDING);
                Debug.Assert(pack.status == GOPRO_HEARTBEAT_STATUS.GOPRO_HEARTBEAT_STATUS_CONNECTED);
            };
            GroundControl.GOPRO_HEARTBEAT p215 = CommunicationChannel.new_GOPRO_HEARTBEAT();
            PH.setPack(p215);
            p215.status = GOPRO_HEARTBEAT_STATUS.GOPRO_HEARTBEAT_STATUS_CONNECTED;
            p215.capture_mode = GOPRO_CAPTURE_MODE.GOPRO_CAPTURE_MODE_VIDEO;
            p215.flags = GOPRO_HEARTBEAT_FLAGS.GOPRO_FLAG_RECORDING;
            CommunicationChannel.instance.send(p215);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnGOPRO_GET_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)7);
                Debug.Assert(pack.cmd_id == GOPRO_COMMAND.GOPRO_COMMAND_BATTERY);
                Debug.Assert(pack.target_system == (byte)(byte)55);
            };
            GroundControl.GOPRO_GET_REQUEST p216 = CommunicationChannel.new_GOPRO_GET_REQUEST();
            PH.setPack(p216);
            p216.cmd_id = GOPRO_COMMAND.GOPRO_COMMAND_BATTERY;
            p216.target_component = (byte)(byte)7;
            p216.target_system = (byte)(byte)55;
            CommunicationChannel.instance.send(p216);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnGOPRO_GET_RESPONSEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.cmd_id == GOPRO_COMMAND.GOPRO_COMMAND_PROTUNE_GAIN);
                Debug.Assert(pack.value.SequenceEqual(new byte[] {(byte)139, (byte)68, (byte)248, (byte)125}));
                Debug.Assert(pack.status == GOPRO_REQUEST_STATUS.GOPRO_REQUEST_FAILED);
            };
            GroundControl.GOPRO_GET_RESPONSE p217 = CommunicationChannel.new_GOPRO_GET_RESPONSE();
            PH.setPack(p217);
            p217.value_SET(new byte[] {(byte)139, (byte)68, (byte)248, (byte)125}, 0) ;
            p217.status = GOPRO_REQUEST_STATUS.GOPRO_REQUEST_FAILED;
            p217.cmd_id = GOPRO_COMMAND.GOPRO_COMMAND_PROTUNE_GAIN;
            CommunicationChannel.instance.send(p217);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnGOPRO_SET_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.value.SequenceEqual(new byte[] {(byte)216, (byte)36, (byte)34, (byte)154}));
                Debug.Assert(pack.target_component == (byte)(byte)39);
                Debug.Assert(pack.target_system == (byte)(byte)207);
                Debug.Assert(pack.cmd_id == GOPRO_COMMAND.GOPRO_COMMAND_BATTERY);
            };
            GroundControl.GOPRO_SET_REQUEST p218 = CommunicationChannel.new_GOPRO_SET_REQUEST();
            PH.setPack(p218);
            p218.target_system = (byte)(byte)207;
            p218.value_SET(new byte[] {(byte)216, (byte)36, (byte)34, (byte)154}, 0) ;
            p218.cmd_id = GOPRO_COMMAND.GOPRO_COMMAND_BATTERY;
            p218.target_component = (byte)(byte)39;
            CommunicationChannel.instance.send(p218);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnGOPRO_SET_RESPONSEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.status == GOPRO_REQUEST_STATUS.GOPRO_REQUEST_SUCCESS);
                Debug.Assert(pack.cmd_id == GOPRO_COMMAND.GOPRO_COMMAND_BATTERY);
            };
            GroundControl.GOPRO_SET_RESPONSE p219 = CommunicationChannel.new_GOPRO_SET_RESPONSE();
            PH.setPack(p219);
            p219.status = GOPRO_REQUEST_STATUS.GOPRO_REQUEST_SUCCESS;
            p219.cmd_id = GOPRO_COMMAND.GOPRO_COMMAND_BATTERY;
            CommunicationChannel.instance.send(p219);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnRPMReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.rpm2 == (float)8.1141717E37F);
                Debug.Assert(pack.rpm1 == (float)2.1792303E38F);
            };
            GroundControl.RPM p226 = CommunicationChannel.new_RPM();
            PH.setPack(p226);
            p226.rpm1 = (float)2.1792303E38F;
            p226.rpm2 = (float)8.1141717E37F;
            CommunicationChannel.instance.send(p226);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            CommunicationChannel.instance.OnESTIMATOR_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.pos_vert_accuracy == (float)3.3977979E38F);
                Debug.Assert(pack.mag_ratio == (float) -1.1852974E37F);
                Debug.Assert(pack.pos_horiz_accuracy == (float)2.3321587E38F);
                Debug.Assert(pack.hagl_ratio == (float)2.4880835E38F);
                Debug.Assert(pack.pos_vert_ratio == (float) -1.6818226E38F);
                Debug.Assert(pack.tas_ratio == (float)2.4827698E38F);
                Debug.Assert(pack.pos_horiz_ratio == (float) -1.6144468E38F);
                Debug.Assert(pack.time_usec == (ulong)4327883433454866717L);
                Debug.Assert(pack.flags == (ESTIMATOR_STATUS_FLAGS.ESTIMATOR_PRED_POS_HORIZ_REL |
                                            ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_VERT_AGL |
                                            ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_HORIZ_ABS |
                                            ESTIMATOR_STATUS_FLAGS.ESTIMATOR_GPS_GLITCH |
                                            ESTIMATOR_STATUS_FLAGS.ESTIMATOR_PRED_POS_HORIZ_ABS));
                Debug.Assert(pack.vel_ratio == (float)8.756289E37F);
            };
            GroundControl.ESTIMATOR_STATUS p230 = CommunicationChannel.new_ESTIMATOR_STATUS();
            PH.setPack(p230);
            p230.pos_vert_accuracy = (float)3.3977979E38F;
            p230.pos_horiz_ratio = (float) -1.6144468E38F;
            p230.pos_vert_ratio = (float) -1.6818226E38F;
            p230.time_usec = (ulong)4327883433454866717L;
            p230.hagl_ratio = (float)2.4880835E38F;
            p230.vel_ratio = (float)8.756289E37F;
            p230.flags = (ESTIMATOR_STATUS_FLAGS.ESTIMATOR_PRED_POS_HORIZ_REL |
                          ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_VERT_AGL |
                          ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_HORIZ_ABS |
                          ESTIMATOR_STATUS_FLAGS.ESTIMATOR_GPS_GLITCH |
                          ESTIMATOR_STATUS_FLAGS.ESTIMATOR_PRED_POS_HORIZ_ABS);
            p230.pos_horiz_accuracy = (float)2.3321587E38F;
            p230.mag_ratio = (float) -1.1852974E37F;
            p230.tas_ratio = (float)2.4827698E38F;
            CommunicationChannel.instance.send(p230);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnWIND_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.wind_x == (float) -6.123456E37F);
                Debug.Assert(pack.vert_accuracy == (float)1.3208273E37F);
                Debug.Assert(pack.time_usec == (ulong)1515252373253744495L);
                Debug.Assert(pack.wind_z == (float) -4.0573913E37F);
                Debug.Assert(pack.wind_alt == (float) -3.2234544E38F);
                Debug.Assert(pack.var_vert == (float)1.4293587E38F);
                Debug.Assert(pack.var_horiz == (float) -1.3513527E38F);
                Debug.Assert(pack.horiz_accuracy == (float) -3.3631937E38F);
                Debug.Assert(pack.wind_y == (float)2.5270018E38F);
            };
            GroundControl.WIND_COV p231 = CommunicationChannel.new_WIND_COV();
            PH.setPack(p231);
            p231.var_horiz = (float) -1.3513527E38F;
            p231.wind_alt = (float) -3.2234544E38F;
            p231.vert_accuracy = (float)1.3208273E37F;
            p231.time_usec = (ulong)1515252373253744495L;
            p231.wind_z = (float) -4.0573913E37F;
            p231.var_vert = (float)1.4293587E38F;
            p231.horiz_accuracy = (float) -3.3631937E38F;
            p231.wind_x = (float) -6.123456E37F;
            p231.wind_y = (float)2.5270018E38F;
            CommunicationChannel.instance.send(p231);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_INPUTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.horiz_accuracy == (float) -9.016342E37F);
                Debug.Assert(pack.speed_accuracy == (float) -1.6434819E38F);
                Debug.Assert(pack.vd == (float)7.436338E37F);
                Debug.Assert(pack.lat == (int)1407916548);
                Debug.Assert(pack.time_week == (ushort)(ushort)38076);
                Debug.Assert(pack.time_usec == (ulong)6158621413196698966L);
                Debug.Assert(pack.vdop == (float)1.2624826E38F);
                Debug.Assert(pack.ignore_flags == (GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY |
                                                   GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VEL_HORIZ |
                                                   GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VDOP |
                                                   GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VEL_VERT));
                Debug.Assert(pack.satellites_visible == (byte)(byte)32);
                Debug.Assert(pack.lon == (int) -244478109);
                Debug.Assert(pack.vn == (float)2.2978692E38F);
                Debug.Assert(pack.gps_id == (byte)(byte)247);
                Debug.Assert(pack.ve == (float)2.935685E38F);
                Debug.Assert(pack.time_week_ms == (uint)257095609U);
                Debug.Assert(pack.hdop == (float)1.9341012E38F);
                Debug.Assert(pack.alt == (float) -1.7930437E38F);
                Debug.Assert(pack.fix_type == (byte)(byte)177);
                Debug.Assert(pack.vert_accuracy == (float)2.118743E38F);
            };
            GroundControl.GPS_INPUT p232 = CommunicationChannel.new_GPS_INPUT();
            PH.setPack(p232);
            p232.horiz_accuracy = (float) -9.016342E37F;
            p232.satellites_visible = (byte)(byte)32;
            p232.ve = (float)2.935685E38F;
            p232.alt = (float) -1.7930437E38F;
            p232.hdop = (float)1.9341012E38F;
            p232.lon = (int) -244478109;
            p232.vdop = (float)1.2624826E38F;
            p232.ignore_flags = (GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY |
                                 GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VEL_HORIZ |
                                 GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VDOP |
                                 GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VEL_VERT);
            p232.time_usec = (ulong)6158621413196698966L;
            p232.fix_type = (byte)(byte)177;
            p232.lat = (int)1407916548;
            p232.vert_accuracy = (float)2.118743E38F;
            p232.time_week = (ushort)(ushort)38076;
            p232.vd = (float)7.436338E37F;
            p232.time_week_ms = (uint)257095609U;
            p232.gps_id = (byte)(byte)247;
            p232.vn = (float)2.2978692E38F;
            p232.speed_accuracy = (float) -1.6434819E38F;
            CommunicationChannel.instance.send(p232);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_RTCM_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.flags == (byte)(byte)153);
                Debug.Assert(pack.len == (byte)(byte)100);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)165, (byte)174, (byte)193, (byte)163, (byte)150, (byte)206, (byte)188, (byte)225, (byte)13, (byte)166, (byte)212, (byte)140, (byte)46, (byte)186, (byte)35, (byte)241, (byte)7, (byte)34, (byte)232, (byte)139, (byte)192, (byte)206, (byte)121, (byte)136, (byte)228, (byte)113, (byte)133, (byte)221, (byte)77, (byte)21, (byte)72, (byte)3, (byte)59, (byte)49, (byte)233, (byte)38, (byte)96, (byte)132, (byte)98, (byte)195, (byte)225, (byte)25, (byte)244, (byte)35, (byte)242, (byte)177, (byte)224, (byte)157, (byte)102, (byte)96, (byte)70, (byte)63, (byte)209, (byte)129, (byte)223, (byte)183, (byte)50, (byte)25, (byte)230, (byte)33, (byte)171, (byte)40, (byte)62, (byte)173, (byte)142, (byte)22, (byte)195, (byte)10, (byte)229, (byte)116, (byte)0, (byte)14, (byte)99, (byte)177, (byte)148, (byte)209, (byte)135, (byte)60, (byte)165, (byte)12, (byte)60, (byte)1, (byte)168, (byte)33, (byte)186, (byte)233, (byte)75, (byte)94, (byte)240, (byte)59, (byte)147, (byte)122, (byte)13, (byte)115, (byte)248, (byte)190, (byte)15, (byte)84, (byte)224, (byte)88, (byte)163, (byte)158, (byte)3, (byte)225, (byte)72, (byte)56, (byte)174, (byte)167, (byte)231, (byte)85, (byte)233, (byte)161, (byte)44, (byte)224, (byte)127, (byte)214, (byte)97, (byte)227, (byte)192, (byte)78, (byte)195, (byte)110, (byte)250, (byte)11, (byte)35, (byte)114, (byte)63, (byte)11, (byte)251, (byte)185, (byte)153, (byte)182, (byte)147, (byte)218, (byte)15, (byte)218, (byte)8, (byte)41, (byte)201, (byte)18, (byte)121, (byte)236, (byte)63, (byte)72, (byte)214, (byte)128, (byte)241, (byte)7, (byte)189, (byte)249, (byte)169, (byte)209, (byte)113, (byte)143, (byte)108, (byte)163, (byte)88, (byte)86, (byte)141, (byte)71, (byte)220, (byte)20, (byte)206, (byte)24, (byte)26, (byte)192, (byte)46, (byte)127, (byte)241, (byte)219, (byte)103, (byte)58, (byte)171, (byte)40, (byte)198, (byte)188, (byte)248, (byte)151, (byte)77, (byte)181}));
            };
            GroundControl.GPS_RTCM_DATA p233 = CommunicationChannel.new_GPS_RTCM_DATA();
            PH.setPack(p233);
            p233.data__SET(new byte[] {(byte)165, (byte)174, (byte)193, (byte)163, (byte)150, (byte)206, (byte)188, (byte)225, (byte)13, (byte)166, (byte)212, (byte)140, (byte)46, (byte)186, (byte)35, (byte)241, (byte)7, (byte)34, (byte)232, (byte)139, (byte)192, (byte)206, (byte)121, (byte)136, (byte)228, (byte)113, (byte)133, (byte)221, (byte)77, (byte)21, (byte)72, (byte)3, (byte)59, (byte)49, (byte)233, (byte)38, (byte)96, (byte)132, (byte)98, (byte)195, (byte)225, (byte)25, (byte)244, (byte)35, (byte)242, (byte)177, (byte)224, (byte)157, (byte)102, (byte)96, (byte)70, (byte)63, (byte)209, (byte)129, (byte)223, (byte)183, (byte)50, (byte)25, (byte)230, (byte)33, (byte)171, (byte)40, (byte)62, (byte)173, (byte)142, (byte)22, (byte)195, (byte)10, (byte)229, (byte)116, (byte)0, (byte)14, (byte)99, (byte)177, (byte)148, (byte)209, (byte)135, (byte)60, (byte)165, (byte)12, (byte)60, (byte)1, (byte)168, (byte)33, (byte)186, (byte)233, (byte)75, (byte)94, (byte)240, (byte)59, (byte)147, (byte)122, (byte)13, (byte)115, (byte)248, (byte)190, (byte)15, (byte)84, (byte)224, (byte)88, (byte)163, (byte)158, (byte)3, (byte)225, (byte)72, (byte)56, (byte)174, (byte)167, (byte)231, (byte)85, (byte)233, (byte)161, (byte)44, (byte)224, (byte)127, (byte)214, (byte)97, (byte)227, (byte)192, (byte)78, (byte)195, (byte)110, (byte)250, (byte)11, (byte)35, (byte)114, (byte)63, (byte)11, (byte)251, (byte)185, (byte)153, (byte)182, (byte)147, (byte)218, (byte)15, (byte)218, (byte)8, (byte)41, (byte)201, (byte)18, (byte)121, (byte)236, (byte)63, (byte)72, (byte)214, (byte)128, (byte)241, (byte)7, (byte)189, (byte)249, (byte)169, (byte)209, (byte)113, (byte)143, (byte)108, (byte)163, (byte)88, (byte)86, (byte)141, (byte)71, (byte)220, (byte)20, (byte)206, (byte)24, (byte)26, (byte)192, (byte)46, (byte)127, (byte)241, (byte)219, (byte)103, (byte)58, (byte)171, (byte)40, (byte)198, (byte)188, (byte)248, (byte)151, (byte)77, (byte)181}, 0) ;
            p233.flags = (byte)(byte)153;
            p233.len = (byte)(byte)100;
            CommunicationChannel.instance.send(p233);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIGH_LATENCYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.altitude_amsl == (short)(short) -10980);
                Debug.Assert(pack.airspeed_sp == (byte)(byte)89);
                Debug.Assert(pack.custom_mode == (uint)1610463625U);
                Debug.Assert(pack.groundspeed == (byte)(byte)158);
                Debug.Assert(pack.battery_remaining == (byte)(byte)169);
                Debug.Assert(pack.pitch == (short)(short) -29182);
                Debug.Assert(pack.base_mode == (MAV_MODE_FLAG.MAV_MODE_FLAG_AUTO_ENABLED |
                                                MAV_MODE_FLAG.MAV_MODE_FLAG_STABILIZE_ENABLED));
                Debug.Assert(pack.longitude == (int)1931302713);
                Debug.Assert(pack.landed_state == MAV_LANDED_STATE.MAV_LANDED_STATE_ON_GROUND);
                Debug.Assert(pack.throttle == (sbyte)(sbyte) - 77);
                Debug.Assert(pack.temperature == (sbyte)(sbyte) - 21);
                Debug.Assert(pack.gps_fix_type == GPS_FIX_TYPE.GPS_FIX_TYPE_DGPS);
                Debug.Assert(pack.heading == (ushort)(ushort)8786);
                Debug.Assert(pack.airspeed == (byte)(byte)199);
                Debug.Assert(pack.climb_rate == (sbyte)(sbyte)0);
                Debug.Assert(pack.temperature_air == (sbyte)(sbyte) - 91);
                Debug.Assert(pack.roll == (short)(short) -17567);
                Debug.Assert(pack.wp_distance == (ushort)(ushort)34815);
                Debug.Assert(pack.gps_nsat == (byte)(byte)129);
                Debug.Assert(pack.wp_num == (byte)(byte)144);
                Debug.Assert(pack.failsafe == (byte)(byte)9);
                Debug.Assert(pack.latitude == (int) -2052098740);
                Debug.Assert(pack.altitude_sp == (short)(short) -15476);
                Debug.Assert(pack.heading_sp == (short)(short)473);
            };
            GroundControl.HIGH_LATENCY p234 = CommunicationChannel.new_HIGH_LATENCY();
            PH.setPack(p234);
            p234.temperature = (sbyte)(sbyte) - 21;
            p234.altitude_sp = (short)(short) -15476;
            p234.landed_state = MAV_LANDED_STATE.MAV_LANDED_STATE_ON_GROUND;
            p234.heading = (ushort)(ushort)8786;
            p234.altitude_amsl = (short)(short) -10980;
            p234.longitude = (int)1931302713;
            p234.latitude = (int) -2052098740;
            p234.throttle = (sbyte)(sbyte) - 77;
            p234.pitch = (short)(short) -29182;
            p234.gps_fix_type = GPS_FIX_TYPE.GPS_FIX_TYPE_DGPS;
            p234.heading_sp = (short)(short)473;
            p234.base_mode = (MAV_MODE_FLAG.MAV_MODE_FLAG_AUTO_ENABLED |
                              MAV_MODE_FLAG.MAV_MODE_FLAG_STABILIZE_ENABLED);
            p234.roll = (short)(short) -17567;
            p234.airspeed_sp = (byte)(byte)89;
            p234.custom_mode = (uint)1610463625U;
            p234.gps_nsat = (byte)(byte)129;
            p234.failsafe = (byte)(byte)9;
            p234.wp_num = (byte)(byte)144;
            p234.battery_remaining = (byte)(byte)169;
            p234.airspeed = (byte)(byte)199;
            p234.temperature_air = (sbyte)(sbyte) - 91;
            p234.wp_distance = (ushort)(ushort)34815;
            p234.climb_rate = (sbyte)(sbyte)0;
            p234.groundspeed = (byte)(byte)158;
            CommunicationChannel.instance.send(p234);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnVIBRATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.clipping_2 == (uint)1376906574U);
                Debug.Assert(pack.vibration_y == (float) -8.91632E37F);
                Debug.Assert(pack.vibration_z == (float)1.4705359E38F);
                Debug.Assert(pack.clipping_1 == (uint)234242854U);
                Debug.Assert(pack.vibration_x == (float)7.3038345E36F);
                Debug.Assert(pack.time_usec == (ulong)4648188089499149126L);
                Debug.Assert(pack.clipping_0 == (uint)2869868074U);
            };
            GroundControl.VIBRATION p241 = CommunicationChannel.new_VIBRATION();
            PH.setPack(p241);
            p241.vibration_z = (float)1.4705359E38F;
            p241.clipping_1 = (uint)234242854U;
            p241.vibration_y = (float) -8.91632E37F;
            p241.clipping_2 = (uint)1376906574U;
            p241.vibration_x = (float)7.3038345E36F;
            p241.clipping_0 = (uint)2869868074U;
            p241.time_usec = (ulong)4648188089499149126L;
            CommunicationChannel.instance.send(p241);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHOME_POSITIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.latitude == (int) -2136209204);
                Debug.Assert(pack.approach_x == (float)1.5582151E38F);
                Debug.Assert(pack.x == (float)1.7291439E37F);
                Debug.Assert(pack.y == (float) -1.9874709E38F);
                Debug.Assert(pack.approach_z == (float) -4.094492E37F);
                Debug.Assert(pack.altitude == (int)511293957);
                Debug.Assert(pack.longitude == (int) -2081978245);
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)3829310726214107611L);
                Debug.Assert(pack.approach_y == (float) -1.8808718E38F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {1.5751539E38F, 2.6937036E38F, 2.5834163E38F, 2.7430336E38F}));
                Debug.Assert(pack.z == (float)2.5858805E38F);
            };
            GroundControl.HOME_POSITION p242 = CommunicationChannel.new_HOME_POSITION();
            PH.setPack(p242);
            p242.q_SET(new float[] {1.5751539E38F, 2.6937036E38F, 2.5834163E38F, 2.7430336E38F}, 0) ;
            p242.z = (float)2.5858805E38F;
            p242.y = (float) -1.9874709E38F;
            p242.time_usec_SET((ulong)3829310726214107611L, PH) ;
            p242.longitude = (int) -2081978245;
            p242.x = (float)1.7291439E37F;
            p242.approach_z = (float) -4.094492E37F;
            p242.approach_y = (float) -1.8808718E38F;
            p242.altitude = (int)511293957;
            p242.approach_x = (float)1.5582151E38F;
            p242.latitude = (int) -2136209204;
            CommunicationChannel.instance.send(p242);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_HOME_POSITIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.y == (float) -6.459222E37F);
                Debug.Assert(pack.altitude == (int)845314520);
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)6203925026320578056L);
                Debug.Assert(pack.x == (float)8.907072E37F);
                Debug.Assert(pack.approach_y == (float) -2.8802922E38F);
                Debug.Assert(pack.z == (float) -8.551018E37F);
                Debug.Assert(pack.approach_x == (float) -4.5668943E37F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-3.2360051E38F, 1.7846857E37F, -1.5521152E38F, 2.3962067E38F}));
                Debug.Assert(pack.target_system == (byte)(byte)251);
                Debug.Assert(pack.longitude == (int) -972048534);
                Debug.Assert(pack.latitude == (int)1595420171);
                Debug.Assert(pack.approach_z == (float) -6.177718E37F);
            };
            GroundControl.SET_HOME_POSITION p243 = CommunicationChannel.new_SET_HOME_POSITION();
            PH.setPack(p243);
            p243.longitude = (int) -972048534;
            p243.approach_z = (float) -6.177718E37F;
            p243.approach_x = (float) -4.5668943E37F;
            p243.q_SET(new float[] {-3.2360051E38F, 1.7846857E37F, -1.5521152E38F, 2.3962067E38F}, 0) ;
            p243.approach_y = (float) -2.8802922E38F;
            p243.altitude = (int)845314520;
            p243.target_system = (byte)(byte)251;
            p243.x = (float)8.907072E37F;
            p243.latitude = (int)1595420171;
            p243.z = (float) -8.551018E37F;
            p243.time_usec_SET((ulong)6203925026320578056L, PH) ;
            p243.y = (float) -6.459222E37F;
            CommunicationChannel.instance.send(p243);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMESSAGE_INTERVALReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.interval_us == (int) -1517528923);
                Debug.Assert(pack.message_id == (ushort)(ushort)62080);
            };
            GroundControl.MESSAGE_INTERVAL p244 = CommunicationChannel.new_MESSAGE_INTERVAL();
            PH.setPack(p244);
            p244.message_id = (ushort)(ushort)62080;
            p244.interval_us = (int) -1517528923;
            CommunicationChannel.instance.send(p244);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnEXTENDED_SYS_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.landed_state == MAV_LANDED_STATE.MAV_LANDED_STATE_TAKEOFF);
                Debug.Assert(pack.vtol_state == MAV_VTOL_STATE.MAV_VTOL_STATE_FW);
            };
            GroundControl.EXTENDED_SYS_STATE p245 = CommunicationChannel.new_EXTENDED_SYS_STATE();
            PH.setPack(p245);
            p245.vtol_state = MAV_VTOL_STATE.MAV_VTOL_STATE_FW;
            p245.landed_state = MAV_LANDED_STATE.MAV_LANDED_STATE_TAKEOFF;
            CommunicationChannel.instance.send(p245);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnADSB_VEHICLEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ver_velocity == (short)(short) -20513);
                Debug.Assert(pack.lon == (int) -1180815592);
                Debug.Assert(pack.altitude == (int)1671766923);
                Debug.Assert(pack.squawk == (ushort)(ushort)23910);
                Debug.Assert(pack.flags == (ADSB_FLAGS.ADSB_FLAGS_VALID_ALTITUDE |
                                            ADSB_FLAGS.ADSB_FLAGS_VALID_COORDS |
                                            ADSB_FLAGS.ADSB_FLAGS_VALID_VELOCITY));
                Debug.Assert(pack.callsign_LEN(ph) == 1);
                Debug.Assert(pack.callsign_TRY(ph).Equals("u"));
                Debug.Assert(pack.hor_velocity == (ushort)(ushort)3581);
                Debug.Assert(pack.tslc == (byte)(byte)143);
                Debug.Assert(pack.lat == (int) -56453642);
                Debug.Assert(pack.ICAO_address == (uint)3065829864U);
                Debug.Assert(pack.heading == (ushort)(ushort)56731);
                Debug.Assert(pack.emitter_type == ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_ULTRA_LIGHT);
                Debug.Assert(pack.altitude_type == ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_PRESSURE_QNH);
            };
            GroundControl.ADSB_VEHICLE p246 = CommunicationChannel.new_ADSB_VEHICLE();
            PH.setPack(p246);
            p246.ver_velocity = (short)(short) -20513;
            p246.flags = (ADSB_FLAGS.ADSB_FLAGS_VALID_ALTITUDE |
                          ADSB_FLAGS.ADSB_FLAGS_VALID_COORDS |
                          ADSB_FLAGS.ADSB_FLAGS_VALID_VELOCITY);
            p246.hor_velocity = (ushort)(ushort)3581;
            p246.emitter_type = ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_ULTRA_LIGHT;
            p246.tslc = (byte)(byte)143;
            p246.altitude = (int)1671766923;
            p246.lat = (int) -56453642;
            p246.callsign_SET("u", PH) ;
            p246.lon = (int) -1180815592;
            p246.altitude_type = ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_PRESSURE_QNH;
            p246.heading = (ushort)(ushort)56731;
            p246.squawk = (ushort)(ushort)23910;
            p246.ICAO_address = (uint)3065829864U;
            CommunicationChannel.instance.send(p246);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCOLLISIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.horizontal_minimum_delta == (float) -2.924507E38F);
                Debug.Assert(pack.threat_level == MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_NONE);
                Debug.Assert(pack.time_to_minimum_delta == (float) -4.540544E37F);
                Debug.Assert(pack.action == MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_ASCEND_OR_DESCEND);
                Debug.Assert(pack.altitude_minimum_delta == (float)2.1416346E38F);
                Debug.Assert(pack.src_ == MAV_COLLISION_SRC.MAV_COLLISION_SRC_ADSB);
                Debug.Assert(pack.id == (uint)990484071U);
            };
            GroundControl.COLLISION p247 = CommunicationChannel.new_COLLISION();
            PH.setPack(p247);
            p247.altitude_minimum_delta = (float)2.1416346E38F;
            p247.action = MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_ASCEND_OR_DESCEND;
            p247.src_ = MAV_COLLISION_SRC.MAV_COLLISION_SRC_ADSB;
            p247.id = (uint)990484071U;
            p247.horizontal_minimum_delta = (float) -2.924507E38F;
            p247.time_to_minimum_delta = (float) -4.540544E37F;
            p247.threat_level = MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_NONE;
            CommunicationChannel.instance.send(p247);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnV2_EXTENSIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.payload.SequenceEqual(new byte[] {(byte)119, (byte)67, (byte)166, (byte)26, (byte)241, (byte)161, (byte)221, (byte)106, (byte)251, (byte)127, (byte)220, (byte)200, (byte)199, (byte)68, (byte)238, (byte)106, (byte)59, (byte)184, (byte)207, (byte)112, (byte)26, (byte)73, (byte)80, (byte)138, (byte)166, (byte)70, (byte)138, (byte)232, (byte)40, (byte)218, (byte)134, (byte)18, (byte)81, (byte)173, (byte)229, (byte)27, (byte)32, (byte)167, (byte)228, (byte)251, (byte)243, (byte)5, (byte)52, (byte)190, (byte)151, (byte)252, (byte)113, (byte)228, (byte)63, (byte)239, (byte)97, (byte)1, (byte)30, (byte)157, (byte)198, (byte)57, (byte)79, (byte)5, (byte)181, (byte)239, (byte)84, (byte)26, (byte)101, (byte)125, (byte)172, (byte)153, (byte)207, (byte)118, (byte)154, (byte)195, (byte)95, (byte)241, (byte)47, (byte)9, (byte)109, (byte)134, (byte)106, (byte)71, (byte)216, (byte)56, (byte)231, (byte)99, (byte)7, (byte)15, (byte)140, (byte)204, (byte)74, (byte)252, (byte)89, (byte)29, (byte)40, (byte)176, (byte)197, (byte)71, (byte)58, (byte)96, (byte)93, (byte)131, (byte)62, (byte)160, (byte)124, (byte)137, (byte)157, (byte)7, (byte)97, (byte)56, (byte)64, (byte)44, (byte)43, (byte)204, (byte)78, (byte)124, (byte)247, (byte)162, (byte)220, (byte)156, (byte)203, (byte)252, (byte)234, (byte)206, (byte)11, (byte)48, (byte)24, (byte)133, (byte)89, (byte)194, (byte)207, (byte)82, (byte)184, (byte)37, (byte)12, (byte)134, (byte)117, (byte)177, (byte)14, (byte)73, (byte)13, (byte)184, (byte)86, (byte)78, (byte)252, (byte)74, (byte)208, (byte)77, (byte)91, (byte)226, (byte)39, (byte)119, (byte)107, (byte)218, (byte)18, (byte)229, (byte)148, (byte)0, (byte)249, (byte)230, (byte)194, (byte)131, (byte)60, (byte)93, (byte)106, (byte)23, (byte)34, (byte)53, (byte)14, (byte)246, (byte)170, (byte)233, (byte)147, (byte)127, (byte)86, (byte)32, (byte)83, (byte)218, (byte)98, (byte)227, (byte)95, (byte)194, (byte)222, (byte)174, (byte)48, (byte)106, (byte)224, (byte)74, (byte)117, (byte)161, (byte)86, (byte)101, (byte)73, (byte)94, (byte)136, (byte)132, (byte)162, (byte)170, (byte)124, (byte)227, (byte)26, (byte)95, (byte)55, (byte)22, (byte)200, (byte)253, (byte)195, (byte)207, (byte)14, (byte)220, (byte)85, (byte)202, (byte)148, (byte)77, (byte)254, (byte)211, (byte)12, (byte)213, (byte)226, (byte)252, (byte)90, (byte)155, (byte)195, (byte)183, (byte)8, (byte)232, (byte)140, (byte)206, (byte)82, (byte)18, (byte)161, (byte)43, (byte)172, (byte)213, (byte)98, (byte)85, (byte)181, (byte)215, (byte)6, (byte)204, (byte)53, (byte)98, (byte)102, (byte)247, (byte)192, (byte)203, (byte)129, (byte)104, (byte)52, (byte)210, (byte)141, (byte)236, (byte)211}));
                Debug.Assert(pack.message_type == (ushort)(ushort)38650);
                Debug.Assert(pack.target_network == (byte)(byte)123);
                Debug.Assert(pack.target_component == (byte)(byte)11);
                Debug.Assert(pack.target_system == (byte)(byte)0);
            };
            GroundControl.V2_EXTENSION p248 = CommunicationChannel.new_V2_EXTENSION();
            PH.setPack(p248);
            p248.target_system = (byte)(byte)0;
            p248.target_component = (byte)(byte)11;
            p248.message_type = (ushort)(ushort)38650;
            p248.payload_SET(new byte[] {(byte)119, (byte)67, (byte)166, (byte)26, (byte)241, (byte)161, (byte)221, (byte)106, (byte)251, (byte)127, (byte)220, (byte)200, (byte)199, (byte)68, (byte)238, (byte)106, (byte)59, (byte)184, (byte)207, (byte)112, (byte)26, (byte)73, (byte)80, (byte)138, (byte)166, (byte)70, (byte)138, (byte)232, (byte)40, (byte)218, (byte)134, (byte)18, (byte)81, (byte)173, (byte)229, (byte)27, (byte)32, (byte)167, (byte)228, (byte)251, (byte)243, (byte)5, (byte)52, (byte)190, (byte)151, (byte)252, (byte)113, (byte)228, (byte)63, (byte)239, (byte)97, (byte)1, (byte)30, (byte)157, (byte)198, (byte)57, (byte)79, (byte)5, (byte)181, (byte)239, (byte)84, (byte)26, (byte)101, (byte)125, (byte)172, (byte)153, (byte)207, (byte)118, (byte)154, (byte)195, (byte)95, (byte)241, (byte)47, (byte)9, (byte)109, (byte)134, (byte)106, (byte)71, (byte)216, (byte)56, (byte)231, (byte)99, (byte)7, (byte)15, (byte)140, (byte)204, (byte)74, (byte)252, (byte)89, (byte)29, (byte)40, (byte)176, (byte)197, (byte)71, (byte)58, (byte)96, (byte)93, (byte)131, (byte)62, (byte)160, (byte)124, (byte)137, (byte)157, (byte)7, (byte)97, (byte)56, (byte)64, (byte)44, (byte)43, (byte)204, (byte)78, (byte)124, (byte)247, (byte)162, (byte)220, (byte)156, (byte)203, (byte)252, (byte)234, (byte)206, (byte)11, (byte)48, (byte)24, (byte)133, (byte)89, (byte)194, (byte)207, (byte)82, (byte)184, (byte)37, (byte)12, (byte)134, (byte)117, (byte)177, (byte)14, (byte)73, (byte)13, (byte)184, (byte)86, (byte)78, (byte)252, (byte)74, (byte)208, (byte)77, (byte)91, (byte)226, (byte)39, (byte)119, (byte)107, (byte)218, (byte)18, (byte)229, (byte)148, (byte)0, (byte)249, (byte)230, (byte)194, (byte)131, (byte)60, (byte)93, (byte)106, (byte)23, (byte)34, (byte)53, (byte)14, (byte)246, (byte)170, (byte)233, (byte)147, (byte)127, (byte)86, (byte)32, (byte)83, (byte)218, (byte)98, (byte)227, (byte)95, (byte)194, (byte)222, (byte)174, (byte)48, (byte)106, (byte)224, (byte)74, (byte)117, (byte)161, (byte)86, (byte)101, (byte)73, (byte)94, (byte)136, (byte)132, (byte)162, (byte)170, (byte)124, (byte)227, (byte)26, (byte)95, (byte)55, (byte)22, (byte)200, (byte)253, (byte)195, (byte)207, (byte)14, (byte)220, (byte)85, (byte)202, (byte)148, (byte)77, (byte)254, (byte)211, (byte)12, (byte)213, (byte)226, (byte)252, (byte)90, (byte)155, (byte)195, (byte)183, (byte)8, (byte)232, (byte)140, (byte)206, (byte)82, (byte)18, (byte)161, (byte)43, (byte)172, (byte)213, (byte)98, (byte)85, (byte)181, (byte)215, (byte)6, (byte)204, (byte)53, (byte)98, (byte)102, (byte)247, (byte)192, (byte)203, (byte)129, (byte)104, (byte)52, (byte)210, (byte)141, (byte)236, (byte)211}, 0) ;
            p248.target_network = (byte)(byte)123;
            CommunicationChannel.instance.send(p248);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMEMORY_VECTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.address == (ushort)(ushort)45698);
                Debug.Assert(pack.ver == (byte)(byte)97);
                Debug.Assert(pack.value.SequenceEqual(new sbyte[] {(sbyte) - 2, (sbyte) - 121, (sbyte) - 85, (sbyte) - 24, (sbyte)52, (sbyte)91, (sbyte) - 28, (sbyte) - 15, (sbyte)20, (sbyte) - 64, (sbyte) - 9, (sbyte)63, (sbyte) - 12, (sbyte) - 108, (sbyte) - 105, (sbyte) - 119, (sbyte)80, (sbyte)1, (sbyte)36, (sbyte)45, (sbyte) - 40, (sbyte) - 68, (sbyte)9, (sbyte) - 69, (sbyte) - 8, (sbyte)37, (sbyte)82, (sbyte) - 24, (sbyte) - 6, (sbyte)61, (sbyte)36, (sbyte)68}));
                Debug.Assert(pack.type == (byte)(byte)132);
            };
            GroundControl.MEMORY_VECT p249 = CommunicationChannel.new_MEMORY_VECT();
            PH.setPack(p249);
            p249.type = (byte)(byte)132;
            p249.value_SET(new sbyte[] {(sbyte) - 2, (sbyte) - 121, (sbyte) - 85, (sbyte) - 24, (sbyte)52, (sbyte)91, (sbyte) - 28, (sbyte) - 15, (sbyte)20, (sbyte) - 64, (sbyte) - 9, (sbyte)63, (sbyte) - 12, (sbyte) - 108, (sbyte) - 105, (sbyte) - 119, (sbyte)80, (sbyte)1, (sbyte)36, (sbyte)45, (sbyte) - 40, (sbyte) - 68, (sbyte)9, (sbyte) - 69, (sbyte) - 8, (sbyte)37, (sbyte)82, (sbyte) - 24, (sbyte) - 6, (sbyte)61, (sbyte)36, (sbyte)68}, 0) ;
            p249.ver = (byte)(byte)97;
            p249.address = (ushort)(ushort)45698;
            CommunicationChannel.instance.send(p249);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnDEBUG_VECTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.name_LEN(ph) == 10);
                Debug.Assert(pack.name_TRY(ph).Equals("dpYroOWyns"));
                Debug.Assert(pack.z == (float) -7.540412E37F);
                Debug.Assert(pack.y == (float)8.779743E37F);
                Debug.Assert(pack.x == (float) -1.986322E38F);
                Debug.Assert(pack.time_usec == (ulong)3417227404294860978L);
            };
            GroundControl.DEBUG_VECT p250 = CommunicationChannel.new_DEBUG_VECT();
            PH.setPack(p250);
            p250.z = (float) -7.540412E37F;
            p250.x = (float) -1.986322E38F;
            p250.time_usec = (ulong)3417227404294860978L;
            p250.y = (float)8.779743E37F;
            p250.name_SET("dpYroOWyns", PH) ;
            CommunicationChannel.instance.send(p250);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnNAMED_VALUE_FLOATReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.value == (float) -3.3671575E38F);
                Debug.Assert(pack.name_LEN(ph) == 1);
                Debug.Assert(pack.name_TRY(ph).Equals("q"));
                Debug.Assert(pack.time_boot_ms == (uint)1739530770U);
            };
            GroundControl.NAMED_VALUE_FLOAT p251 = CommunicationChannel.new_NAMED_VALUE_FLOAT();
            PH.setPack(p251);
            p251.value = (float) -3.3671575E38F;
            p251.time_boot_ms = (uint)1739530770U;
            p251.name_SET("q", PH) ;
            CommunicationChannel.instance.send(p251);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnNAMED_VALUE_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)2649201880U);
                Debug.Assert(pack.name_LEN(ph) == 3);
                Debug.Assert(pack.name_TRY(ph).Equals("lmD"));
                Debug.Assert(pack.value == (int) -1050437627);
            };
            GroundControl.NAMED_VALUE_INT p252 = CommunicationChannel.new_NAMED_VALUE_INT();
            PH.setPack(p252);
            p252.name_SET("lmD", PH) ;
            p252.value = (int) -1050437627;
            p252.time_boot_ms = (uint)2649201880U;
            CommunicationChannel.instance.send(p252);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSTATUSTEXTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.text_LEN(ph) == 31);
                Debug.Assert(pack.text_TRY(ph).Equals("dmydXndpgfkapqtjqwgePgoddaOkajp"));
                Debug.Assert(pack.severity == MAV_SEVERITY.MAV_SEVERITY_EMERGENCY);
            };
            GroundControl.STATUSTEXT p253 = CommunicationChannel.new_STATUSTEXT();
            PH.setPack(p253);
            p253.text_SET("dmydXndpgfkapqtjqwgePgoddaOkajp", PH) ;
            p253.severity = MAV_SEVERITY.MAV_SEVERITY_EMERGENCY;
            CommunicationChannel.instance.send(p253);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnDEBUGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ind == (byte)(byte)162);
                Debug.Assert(pack.value == (float)9.005886E37F);
                Debug.Assert(pack.time_boot_ms == (uint)689309029U);
            };
            GroundControl.DEBUG p254 = CommunicationChannel.new_DEBUG();
            PH.setPack(p254);
            p254.time_boot_ms = (uint)689309029U;
            p254.value = (float)9.005886E37F;
            p254.ind = (byte)(byte)162;
            CommunicationChannel.instance.send(p254);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSETUP_SIGNINGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.initial_timestamp == (ulong)8065040902711763077L);
                Debug.Assert(pack.target_system == (byte)(byte)186);
                Debug.Assert(pack.target_component == (byte)(byte)6);
                Debug.Assert(pack.secret_key.SequenceEqual(new byte[] {(byte)114, (byte)103, (byte)76, (byte)54, (byte)213, (byte)124, (byte)181, (byte)56, (byte)113, (byte)228, (byte)161, (byte)116, (byte)229, (byte)93, (byte)207, (byte)116, (byte)58, (byte)134, (byte)81, (byte)156, (byte)132, (byte)54, (byte)220, (byte)246, (byte)143, (byte)118, (byte)97, (byte)134, (byte)47, (byte)157, (byte)67, (byte)141}));
            };
            GroundControl.SETUP_SIGNING p256 = CommunicationChannel.new_SETUP_SIGNING();
            PH.setPack(p256);
            p256.secret_key_SET(new byte[] {(byte)114, (byte)103, (byte)76, (byte)54, (byte)213, (byte)124, (byte)181, (byte)56, (byte)113, (byte)228, (byte)161, (byte)116, (byte)229, (byte)93, (byte)207, (byte)116, (byte)58, (byte)134, (byte)81, (byte)156, (byte)132, (byte)54, (byte)220, (byte)246, (byte)143, (byte)118, (byte)97, (byte)134, (byte)47, (byte)157, (byte)67, (byte)141}, 0) ;
            p256.target_system = (byte)(byte)186;
            p256.initial_timestamp = (ulong)8065040902711763077L;
            p256.target_component = (byte)(byte)6;
            CommunicationChannel.instance.send(p256);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnBUTTON_CHANGEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.last_change_ms == (uint)4001175173U);
                Debug.Assert(pack.time_boot_ms == (uint)1606188807U);
                Debug.Assert(pack.state == (byte)(byte)34);
            };
            GroundControl.BUTTON_CHANGE p257 = CommunicationChannel.new_BUTTON_CHANGE();
            PH.setPack(p257);
            p257.time_boot_ms = (uint)1606188807U;
            p257.state = (byte)(byte)34;
            p257.last_change_ms = (uint)4001175173U;
            CommunicationChannel.instance.send(p257);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPLAY_TUNEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)229);
                Debug.Assert(pack.target_component == (byte)(byte)193);
                Debug.Assert(pack.tune_LEN(ph) == 5);
                Debug.Assert(pack.tune_TRY(ph).Equals("deZIw"));
            };
            GroundControl.PLAY_TUNE p258 = CommunicationChannel.new_PLAY_TUNE();
            PH.setPack(p258);
            p258.target_component = (byte)(byte)193;
            p258.target_system = (byte)(byte)229;
            p258.tune_SET("deZIw", PH) ;
            CommunicationChannel.instance.send(p258);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCAMERA_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sensor_size_v == (float) -2.780039E38F);
                Debug.Assert(pack.resolution_h == (ushort)(ushort)23456);
                Debug.Assert(pack.cam_definition_uri_LEN(ph) == 139);
                Debug.Assert(pack.cam_definition_uri_TRY(ph).Equals("birwuubsuojsFodjbtqjteavfzflzULvNhQoygLgptmsihnosdrpgqnndskyySfpSdsxfmzosustiuZglycmfhsxbxpTpyvghwNpSknesySmcePthIfslrGgzuvlxyjladvWmAkgcfp"));
                Debug.Assert(pack.flags == (CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE |
                                            CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_IMAGE |
                                            CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE));
                Debug.Assert(pack.model_name.SequenceEqual(new byte[] {(byte)46, (byte)104, (byte)155, (byte)155, (byte)99, (byte)247, (byte)173, (byte)5, (byte)194, (byte)4, (byte)0, (byte)119, (byte)140, (byte)65, (byte)163, (byte)19, (byte)53, (byte)139, (byte)118, (byte)8, (byte)41, (byte)209, (byte)183, (byte)208, (byte)192, (byte)17, (byte)38, (byte)235, (byte)154, (byte)113, (byte)151, (byte)36}));
                Debug.Assert(pack.focal_length == (float) -2.9657564E38F);
                Debug.Assert(pack.vendor_name.SequenceEqual(new byte[] {(byte)192, (byte)48, (byte)236, (byte)174, (byte)246, (byte)114, (byte)63, (byte)18, (byte)124, (byte)88, (byte)124, (byte)152, (byte)198, (byte)17, (byte)119, (byte)250, (byte)135, (byte)138, (byte)138, (byte)32, (byte)232, (byte)174, (byte)232, (byte)225, (byte)56, (byte)134, (byte)96, (byte)69, (byte)111, (byte)210, (byte)108, (byte)19}));
                Debug.Assert(pack.sensor_size_h == (float)2.2832702E38F);
                Debug.Assert(pack.lens_id == (byte)(byte)17);
                Debug.Assert(pack.time_boot_ms == (uint)486733367U);
                Debug.Assert(pack.resolution_v == (ushort)(ushort)17597);
                Debug.Assert(pack.cam_definition_version == (ushort)(ushort)65238);
                Debug.Assert(pack.firmware_version == (uint)1573906061U);
            };
            GroundControl.CAMERA_INFORMATION p259 = CommunicationChannel.new_CAMERA_INFORMATION();
            PH.setPack(p259);
            p259.time_boot_ms = (uint)486733367U;
            p259.focal_length = (float) -2.9657564E38F;
            p259.vendor_name_SET(new byte[] {(byte)192, (byte)48, (byte)236, (byte)174, (byte)246, (byte)114, (byte)63, (byte)18, (byte)124, (byte)88, (byte)124, (byte)152, (byte)198, (byte)17, (byte)119, (byte)250, (byte)135, (byte)138, (byte)138, (byte)32, (byte)232, (byte)174, (byte)232, (byte)225, (byte)56, (byte)134, (byte)96, (byte)69, (byte)111, (byte)210, (byte)108, (byte)19}, 0) ;
            p259.flags = (CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE |
                          CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_IMAGE |
                          CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE);
            p259.cam_definition_uri_SET("birwuubsuojsFodjbtqjteavfzflzULvNhQoygLgptmsihnosdrpgqnndskyySfpSdsxfmzosustiuZglycmfhsxbxpTpyvghwNpSknesySmcePthIfslrGgzuvlxyjladvWmAkgcfp", PH) ;
            p259.resolution_v = (ushort)(ushort)17597;
            p259.model_name_SET(new byte[] {(byte)46, (byte)104, (byte)155, (byte)155, (byte)99, (byte)247, (byte)173, (byte)5, (byte)194, (byte)4, (byte)0, (byte)119, (byte)140, (byte)65, (byte)163, (byte)19, (byte)53, (byte)139, (byte)118, (byte)8, (byte)41, (byte)209, (byte)183, (byte)208, (byte)192, (byte)17, (byte)38, (byte)235, (byte)154, (byte)113, (byte)151, (byte)36}, 0) ;
            p259.sensor_size_h = (float)2.2832702E38F;
            p259.resolution_h = (ushort)(ushort)23456;
            p259.lens_id = (byte)(byte)17;
            p259.cam_definition_version = (ushort)(ushort)65238;
            p259.sensor_size_v = (float) -2.780039E38F;
            p259.firmware_version = (uint)1573906061U;
            CommunicationChannel.instance.send(p259);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCAMERA_SETTINGSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mode_id == CAMERA_MODE.CAMERA_MODE_IMAGE_SURVEY);
                Debug.Assert(pack.time_boot_ms == (uint)2107927833U);
            };
            GroundControl.CAMERA_SETTINGS p260 = CommunicationChannel.new_CAMERA_SETTINGS();
            PH.setPack(p260);
            p260.mode_id = CAMERA_MODE.CAMERA_MODE_IMAGE_SURVEY;
            p260.time_boot_ms = (uint)2107927833U;
            CommunicationChannel.instance.send(p260);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSTORAGE_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)412168467U);
                Debug.Assert(pack.status == (byte)(byte)254);
                Debug.Assert(pack.storage_count == (byte)(byte)86);
                Debug.Assert(pack.read_speed == (float)1.73783E38F);
                Debug.Assert(pack.total_capacity == (float) -1.7444748E38F);
                Debug.Assert(pack.used_capacity == (float)2.242307E38F);
                Debug.Assert(pack.storage_id == (byte)(byte)153);
                Debug.Assert(pack.available_capacity == (float)9.78737E37F);
                Debug.Assert(pack.write_speed == (float) -2.9235227E38F);
            };
            GroundControl.STORAGE_INFORMATION p261 = CommunicationChannel.new_STORAGE_INFORMATION();
            PH.setPack(p261);
            p261.available_capacity = (float)9.78737E37F;
            p261.storage_id = (byte)(byte)153;
            p261.status = (byte)(byte)254;
            p261.used_capacity = (float)2.242307E38F;
            p261.total_capacity = (float) -1.7444748E38F;
            p261.read_speed = (float)1.73783E38F;
            p261.storage_count = (byte)(byte)86;
            p261.write_speed = (float) -2.9235227E38F;
            p261.time_boot_ms = (uint)412168467U;
            CommunicationChannel.instance.send(p261);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCAMERA_CAPTURE_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.video_status == (byte)(byte)73);
                Debug.Assert(pack.recording_time_ms == (uint)3817231495U);
                Debug.Assert(pack.image_status == (byte)(byte)25);
                Debug.Assert(pack.time_boot_ms == (uint)1586598991U);
                Debug.Assert(pack.available_capacity == (float) -2.4266323E38F);
                Debug.Assert(pack.image_interval == (float)3.137994E38F);
            };
            GroundControl.CAMERA_CAPTURE_STATUS p262 = CommunicationChannel.new_CAMERA_CAPTURE_STATUS();
            PH.setPack(p262);
            p262.time_boot_ms = (uint)1586598991U;
            p262.image_interval = (float)3.137994E38F;
            p262.available_capacity = (float) -2.4266323E38F;
            p262.image_status = (byte)(byte)25;
            p262.recording_time_ms = (uint)3817231495U;
            p262.video_status = (byte)(byte)73;
            CommunicationChannel.instance.send(p262);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCAMERA_IMAGE_CAPTUREDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.image_index == (int) -2054800563);
                Debug.Assert(pack.camera_id == (byte)(byte)80);
                Debug.Assert(pack.lat == (int) -1705871715);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-1.9018442E38F, -1.1892026E38F, 2.3702198E38F, 3.2970174E38F}));
                Debug.Assert(pack.capture_result == (sbyte)(sbyte) - 101);
                Debug.Assert(pack.alt == (int) -322303093);
                Debug.Assert(pack.relative_alt == (int) -1266446274);
                Debug.Assert(pack.lon == (int)1726037837);
                Debug.Assert(pack.time_utc == (ulong)9207708747713125720L);
                Debug.Assert(pack.file_url_LEN(ph) == 68);
                Debug.Assert(pack.file_url_TRY(ph).Equals("eisfkrhvfyfkerZDElobvgamizztzjplXvAjDsltzhoasgouriwscoakdrxkxvpkscjr"));
                Debug.Assert(pack.time_boot_ms == (uint)2626503171U);
            };
            GroundControl.CAMERA_IMAGE_CAPTURED p263 = CommunicationChannel.new_CAMERA_IMAGE_CAPTURED();
            PH.setPack(p263);
            p263.file_url_SET("eisfkrhvfyfkerZDElobvgamizztzjplXvAjDsltzhoasgouriwscoakdrxkxvpkscjr", PH) ;
            p263.time_boot_ms = (uint)2626503171U;
            p263.alt = (int) -322303093;
            p263.lat = (int) -1705871715;
            p263.camera_id = (byte)(byte)80;
            p263.capture_result = (sbyte)(sbyte) - 101;
            p263.image_index = (int) -2054800563;
            p263.time_utc = (ulong)9207708747713125720L;
            p263.relative_alt = (int) -1266446274;
            p263.lon = (int)1726037837;
            p263.q_SET(new float[] {-1.9018442E38F, -1.1892026E38F, 2.3702198E38F, 3.2970174E38F}, 0) ;
            CommunicationChannel.instance.send(p263);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnFLIGHT_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.arming_time_utc == (ulong)2987757657585844247L);
                Debug.Assert(pack.takeoff_time_utc == (ulong)5723241717379668409L);
                Debug.Assert(pack.flight_uuid == (ulong)1833092389182785301L);
                Debug.Assert(pack.time_boot_ms == (uint)1237509604U);
            };
            GroundControl.FLIGHT_INFORMATION p264 = CommunicationChannel.new_FLIGHT_INFORMATION();
            PH.setPack(p264);
            p264.takeoff_time_utc = (ulong)5723241717379668409L;
            p264.time_boot_ms = (uint)1237509604U;
            p264.flight_uuid = (ulong)1833092389182785301L;
            p264.arming_time_utc = (ulong)2987757657585844247L;
            CommunicationChannel.instance.send(p264);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            ADV_TEST_CH.OnMOUNT_ORIENTATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.yaw == (float)1.2106281E38F);
                Debug.Assert(pack.roll == (float)2.8325089E38F);
                Debug.Assert(pack.time_boot_ms == (uint)2432882924U);
                Debug.Assert(pack.pitch == (float) -1.3566668E37F);
            };
            GroundControl.MOUNT_ORIENTATION p265 = CommunicationChannel.new_MOUNT_ORIENTATION();
            PH.setPack(p265);
            p265.yaw = (float)1.2106281E38F;
            p265.pitch = (float) -1.3566668E37F;
            p265.roll = (float)2.8325089E38F;
            p265.time_boot_ms = (uint)2432882924U;
            CommunicationChannel.instance.send(p265);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnLOGGING_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)184);
                Debug.Assert(pack.length == (byte)(byte)168);
                Debug.Assert(pack.first_message_offset == (byte)(byte)72);
                Debug.Assert(pack.target_component == (byte)(byte)27);
                Debug.Assert(pack.sequence == (ushort)(ushort)16691);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)33, (byte)62, (byte)180, (byte)237, (byte)159, (byte)221, (byte)180, (byte)40, (byte)82, (byte)86, (byte)196, (byte)233, (byte)185, (byte)168, (byte)64, (byte)165, (byte)49, (byte)42, (byte)33, (byte)241, (byte)167, (byte)63, (byte)103, (byte)201, (byte)185, (byte)210, (byte)35, (byte)26, (byte)36, (byte)176, (byte)148, (byte)105, (byte)252, (byte)227, (byte)138, (byte)118, (byte)29, (byte)12, (byte)176, (byte)25, (byte)104, (byte)193, (byte)24, (byte)165, (byte)90, (byte)195, (byte)170, (byte)197, (byte)84, (byte)47, (byte)166, (byte)50, (byte)223, (byte)74, (byte)0, (byte)180, (byte)190, (byte)217, (byte)172, (byte)178, (byte)187, (byte)108, (byte)121, (byte)6, (byte)183, (byte)3, (byte)129, (byte)156, (byte)88, (byte)20, (byte)153, (byte)50, (byte)250, (byte)207, (byte)141, (byte)0, (byte)190, (byte)139, (byte)109, (byte)194, (byte)102, (byte)106, (byte)14, (byte)186, (byte)228, (byte)43, (byte)152, (byte)169, (byte)141, (byte)126, (byte)32, (byte)44, (byte)121, (byte)70, (byte)6, (byte)56, (byte)84, (byte)136, (byte)63, (byte)66, (byte)2, (byte)198, (byte)84, (byte)128, (byte)196, (byte)32, (byte)87, (byte)212, (byte)148, (byte)132, (byte)158, (byte)54, (byte)60, (byte)140, (byte)194, (byte)24, (byte)219, (byte)102, (byte)119, (byte)131, (byte)130, (byte)202, (byte)238, (byte)90, (byte)123, (byte)100, (byte)69, (byte)129, (byte)240, (byte)13, (byte)196, (byte)47, (byte)204, (byte)145, (byte)9, (byte)170, (byte)70, (byte)243, (byte)67, (byte)180, (byte)142, (byte)102, (byte)150, (byte)140, (byte)95, (byte)99, (byte)29, (byte)51, (byte)240, (byte)23, (byte)244, (byte)55, (byte)95, (byte)112, (byte)21, (byte)20, (byte)226, (byte)50, (byte)178, (byte)67, (byte)227, (byte)173, (byte)146, (byte)109, (byte)160, (byte)185, (byte)41, (byte)245, (byte)210, (byte)54, (byte)121, (byte)37, (byte)141, (byte)66, (byte)242, (byte)134, (byte)244, (byte)15, (byte)122, (byte)21, (byte)214, (byte)20, (byte)207, (byte)238, (byte)208, (byte)135, (byte)29, (byte)32, (byte)100, (byte)186, (byte)244, (byte)181, (byte)4, (byte)196, (byte)51, (byte)152, (byte)170, (byte)241, (byte)34, (byte)2, (byte)169, (byte)36, (byte)193, (byte)183, (byte)113, (byte)85, (byte)23, (byte)10, (byte)113, (byte)174, (byte)212, (byte)107, (byte)33, (byte)148, (byte)53, (byte)204, (byte)90, (byte)90, (byte)58, (byte)48, (byte)112, (byte)254, (byte)253, (byte)130, (byte)12, (byte)194, (byte)185, (byte)17, (byte)124, (byte)230, (byte)222, (byte)131, (byte)232, (byte)217, (byte)255, (byte)56, (byte)17, (byte)183, (byte)165, (byte)215, (byte)51, (byte)7, (byte)62, (byte)81, (byte)213, (byte)29, (byte)152, (byte)234, (byte)145}));
            };
            GroundControl.LOGGING_DATA p266 = CommunicationChannel.new_LOGGING_DATA();
            PH.setPack(p266);
            p266.data__SET(new byte[] {(byte)33, (byte)62, (byte)180, (byte)237, (byte)159, (byte)221, (byte)180, (byte)40, (byte)82, (byte)86, (byte)196, (byte)233, (byte)185, (byte)168, (byte)64, (byte)165, (byte)49, (byte)42, (byte)33, (byte)241, (byte)167, (byte)63, (byte)103, (byte)201, (byte)185, (byte)210, (byte)35, (byte)26, (byte)36, (byte)176, (byte)148, (byte)105, (byte)252, (byte)227, (byte)138, (byte)118, (byte)29, (byte)12, (byte)176, (byte)25, (byte)104, (byte)193, (byte)24, (byte)165, (byte)90, (byte)195, (byte)170, (byte)197, (byte)84, (byte)47, (byte)166, (byte)50, (byte)223, (byte)74, (byte)0, (byte)180, (byte)190, (byte)217, (byte)172, (byte)178, (byte)187, (byte)108, (byte)121, (byte)6, (byte)183, (byte)3, (byte)129, (byte)156, (byte)88, (byte)20, (byte)153, (byte)50, (byte)250, (byte)207, (byte)141, (byte)0, (byte)190, (byte)139, (byte)109, (byte)194, (byte)102, (byte)106, (byte)14, (byte)186, (byte)228, (byte)43, (byte)152, (byte)169, (byte)141, (byte)126, (byte)32, (byte)44, (byte)121, (byte)70, (byte)6, (byte)56, (byte)84, (byte)136, (byte)63, (byte)66, (byte)2, (byte)198, (byte)84, (byte)128, (byte)196, (byte)32, (byte)87, (byte)212, (byte)148, (byte)132, (byte)158, (byte)54, (byte)60, (byte)140, (byte)194, (byte)24, (byte)219, (byte)102, (byte)119, (byte)131, (byte)130, (byte)202, (byte)238, (byte)90, (byte)123, (byte)100, (byte)69, (byte)129, (byte)240, (byte)13, (byte)196, (byte)47, (byte)204, (byte)145, (byte)9, (byte)170, (byte)70, (byte)243, (byte)67, (byte)180, (byte)142, (byte)102, (byte)150, (byte)140, (byte)95, (byte)99, (byte)29, (byte)51, (byte)240, (byte)23, (byte)244, (byte)55, (byte)95, (byte)112, (byte)21, (byte)20, (byte)226, (byte)50, (byte)178, (byte)67, (byte)227, (byte)173, (byte)146, (byte)109, (byte)160, (byte)185, (byte)41, (byte)245, (byte)210, (byte)54, (byte)121, (byte)37, (byte)141, (byte)66, (byte)242, (byte)134, (byte)244, (byte)15, (byte)122, (byte)21, (byte)214, (byte)20, (byte)207, (byte)238, (byte)208, (byte)135, (byte)29, (byte)32, (byte)100, (byte)186, (byte)244, (byte)181, (byte)4, (byte)196, (byte)51, (byte)152, (byte)170, (byte)241, (byte)34, (byte)2, (byte)169, (byte)36, (byte)193, (byte)183, (byte)113, (byte)85, (byte)23, (byte)10, (byte)113, (byte)174, (byte)212, (byte)107, (byte)33, (byte)148, (byte)53, (byte)204, (byte)90, (byte)90, (byte)58, (byte)48, (byte)112, (byte)254, (byte)253, (byte)130, (byte)12, (byte)194, (byte)185, (byte)17, (byte)124, (byte)230, (byte)222, (byte)131, (byte)232, (byte)217, (byte)255, (byte)56, (byte)17, (byte)183, (byte)165, (byte)215, (byte)51, (byte)7, (byte)62, (byte)81, (byte)213, (byte)29, (byte)152, (byte)234, (byte)145}, 0) ;
            p266.target_component = (byte)(byte)27;
            p266.length = (byte)(byte)168;
            p266.first_message_offset = (byte)(byte)72;
            p266.sequence = (ushort)(ushort)16691;
            p266.target_system = (byte)(byte)184;
            CommunicationChannel.instance.send(p266);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnLOGGING_DATA_ACKEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)12);
                Debug.Assert(pack.first_message_offset == (byte)(byte)142);
                Debug.Assert(pack.sequence == (ushort)(ushort)6420);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)66, (byte)185, (byte)87, (byte)216, (byte)40, (byte)107, (byte)147, (byte)15, (byte)1, (byte)185, (byte)27, (byte)7, (byte)71, (byte)15, (byte)238, (byte)232, (byte)108, (byte)212, (byte)105, (byte)83, (byte)226, (byte)235, (byte)74, (byte)206, (byte)67, (byte)112, (byte)115, (byte)9, (byte)113, (byte)121, (byte)96, (byte)255, (byte)56, (byte)71, (byte)73, (byte)169, (byte)222, (byte)89, (byte)0, (byte)180, (byte)63, (byte)241, (byte)24, (byte)190, (byte)138, (byte)27, (byte)229, (byte)98, (byte)126, (byte)97, (byte)4, (byte)175, (byte)174, (byte)60, (byte)169, (byte)232, (byte)37, (byte)30, (byte)160, (byte)196, (byte)166, (byte)8, (byte)246, (byte)201, (byte)46, (byte)83, (byte)53, (byte)242, (byte)118, (byte)214, (byte)175, (byte)94, (byte)228, (byte)35, (byte)101, (byte)226, (byte)240, (byte)52, (byte)3, (byte)56, (byte)192, (byte)74, (byte)174, (byte)50, (byte)34, (byte)134, (byte)189, (byte)92, (byte)212, (byte)136, (byte)222, (byte)215, (byte)229, (byte)172, (byte)17, (byte)191, (byte)171, (byte)20, (byte)183, (byte)51, (byte)182, (byte)189, (byte)101, (byte)126, (byte)200, (byte)69, (byte)2, (byte)236, (byte)97, (byte)165, (byte)103, (byte)244, (byte)113, (byte)3, (byte)19, (byte)16, (byte)241, (byte)161, (byte)18, (byte)0, (byte)209, (byte)94, (byte)219, (byte)253, (byte)63, (byte)187, (byte)144, (byte)245, (byte)85, (byte)25, (byte)21, (byte)131, (byte)50, (byte)77, (byte)216, (byte)134, (byte)219, (byte)29, (byte)244, (byte)39, (byte)186, (byte)198, (byte)214, (byte)93, (byte)254, (byte)128, (byte)101, (byte)81, (byte)165, (byte)96, (byte)1, (byte)48, (byte)26, (byte)74, (byte)138, (byte)205, (byte)207, (byte)176, (byte)199, (byte)76, (byte)78, (byte)178, (byte)209, (byte)203, (byte)232, (byte)21, (byte)203, (byte)111, (byte)254, (byte)95, (byte)243, (byte)237, (byte)119, (byte)30, (byte)150, (byte)99, (byte)235, (byte)106, (byte)206, (byte)252, (byte)10, (byte)234, (byte)141, (byte)238, (byte)132, (byte)9, (byte)10, (byte)215, (byte)107, (byte)251, (byte)80, (byte)166, (byte)118, (byte)43, (byte)164, (byte)173, (byte)205, (byte)144, (byte)149, (byte)109, (byte)207, (byte)89, (byte)33, (byte)8, (byte)235, (byte)142, (byte)198, (byte)232, (byte)190, (byte)168, (byte)56, (byte)230, (byte)205, (byte)109, (byte)158, (byte)132, (byte)59, (byte)35, (byte)233, (byte)247, (byte)47, (byte)10, (byte)186, (byte)182, (byte)131, (byte)177, (byte)135, (byte)4, (byte)116, (byte)170, (byte)173, (byte)215, (byte)143, (byte)112, (byte)71, (byte)188, (byte)179, (byte)221, (byte)26, (byte)31, (byte)14, (byte)132, (byte)170, (byte)25, (byte)104, (byte)95, (byte)193, (byte)135, (byte)194}));
                Debug.Assert(pack.target_system == (byte)(byte)47);
                Debug.Assert(pack.length == (byte)(byte)95);
            };
            GroundControl.LOGGING_DATA_ACKED p267 = CommunicationChannel.new_LOGGING_DATA_ACKED();
            PH.setPack(p267);
            p267.data__SET(new byte[] {(byte)66, (byte)185, (byte)87, (byte)216, (byte)40, (byte)107, (byte)147, (byte)15, (byte)1, (byte)185, (byte)27, (byte)7, (byte)71, (byte)15, (byte)238, (byte)232, (byte)108, (byte)212, (byte)105, (byte)83, (byte)226, (byte)235, (byte)74, (byte)206, (byte)67, (byte)112, (byte)115, (byte)9, (byte)113, (byte)121, (byte)96, (byte)255, (byte)56, (byte)71, (byte)73, (byte)169, (byte)222, (byte)89, (byte)0, (byte)180, (byte)63, (byte)241, (byte)24, (byte)190, (byte)138, (byte)27, (byte)229, (byte)98, (byte)126, (byte)97, (byte)4, (byte)175, (byte)174, (byte)60, (byte)169, (byte)232, (byte)37, (byte)30, (byte)160, (byte)196, (byte)166, (byte)8, (byte)246, (byte)201, (byte)46, (byte)83, (byte)53, (byte)242, (byte)118, (byte)214, (byte)175, (byte)94, (byte)228, (byte)35, (byte)101, (byte)226, (byte)240, (byte)52, (byte)3, (byte)56, (byte)192, (byte)74, (byte)174, (byte)50, (byte)34, (byte)134, (byte)189, (byte)92, (byte)212, (byte)136, (byte)222, (byte)215, (byte)229, (byte)172, (byte)17, (byte)191, (byte)171, (byte)20, (byte)183, (byte)51, (byte)182, (byte)189, (byte)101, (byte)126, (byte)200, (byte)69, (byte)2, (byte)236, (byte)97, (byte)165, (byte)103, (byte)244, (byte)113, (byte)3, (byte)19, (byte)16, (byte)241, (byte)161, (byte)18, (byte)0, (byte)209, (byte)94, (byte)219, (byte)253, (byte)63, (byte)187, (byte)144, (byte)245, (byte)85, (byte)25, (byte)21, (byte)131, (byte)50, (byte)77, (byte)216, (byte)134, (byte)219, (byte)29, (byte)244, (byte)39, (byte)186, (byte)198, (byte)214, (byte)93, (byte)254, (byte)128, (byte)101, (byte)81, (byte)165, (byte)96, (byte)1, (byte)48, (byte)26, (byte)74, (byte)138, (byte)205, (byte)207, (byte)176, (byte)199, (byte)76, (byte)78, (byte)178, (byte)209, (byte)203, (byte)232, (byte)21, (byte)203, (byte)111, (byte)254, (byte)95, (byte)243, (byte)237, (byte)119, (byte)30, (byte)150, (byte)99, (byte)235, (byte)106, (byte)206, (byte)252, (byte)10, (byte)234, (byte)141, (byte)238, (byte)132, (byte)9, (byte)10, (byte)215, (byte)107, (byte)251, (byte)80, (byte)166, (byte)118, (byte)43, (byte)164, (byte)173, (byte)205, (byte)144, (byte)149, (byte)109, (byte)207, (byte)89, (byte)33, (byte)8, (byte)235, (byte)142, (byte)198, (byte)232, (byte)190, (byte)168, (byte)56, (byte)230, (byte)205, (byte)109, (byte)158, (byte)132, (byte)59, (byte)35, (byte)233, (byte)247, (byte)47, (byte)10, (byte)186, (byte)182, (byte)131, (byte)177, (byte)135, (byte)4, (byte)116, (byte)170, (byte)173, (byte)215, (byte)143, (byte)112, (byte)71, (byte)188, (byte)179, (byte)221, (byte)26, (byte)31, (byte)14, (byte)132, (byte)170, (byte)25, (byte)104, (byte)95, (byte)193, (byte)135, (byte)194}, 0) ;
            p267.first_message_offset = (byte)(byte)142;
            p267.target_component = (byte)(byte)12;
            p267.target_system = (byte)(byte)47;
            p267.sequence = (ushort)(ushort)6420;
            p267.length = (byte)(byte)95;
            CommunicationChannel.instance.send(p267);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnLOGGING_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)230);
                Debug.Assert(pack.sequence == (ushort)(ushort)3058);
                Debug.Assert(pack.target_system == (byte)(byte)173);
            };
            GroundControl.LOGGING_ACK p268 = CommunicationChannel.new_LOGGING_ACK();
            PH.setPack(p268);
            p268.target_system = (byte)(byte)173;
            p268.sequence = (ushort)(ushort)3058;
            p268.target_component = (byte)(byte)230;
            CommunicationChannel.instance.send(p268);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnVIDEO_STREAM_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.resolution_v == (ushort)(ushort)53279);
                Debug.Assert(pack.resolution_h == (ushort)(ushort)22936);
                Debug.Assert(pack.uri_LEN(ph) == 188);
                Debug.Assert(pack.uri_TRY(ph).Equals("isjryudbjqpmevoocEfiqfxscykuZvnckNvboapeduvgyxRxkuczwqeFqIifpawllkoquefvoukivxHxluvgbvuhoitdambpatsdmvhifetgswhqdBveysmbfaqlsmkvofbrhVNfbrbPbyiunfgyerxPaaknuwvyfIccqkxxeeziVvwxDrhlfppmrNwg"));
                Debug.Assert(pack.rotation == (ushort)(ushort)347);
                Debug.Assert(pack.status == (byte)(byte)220);
                Debug.Assert(pack.camera_id == (byte)(byte)23);
                Debug.Assert(pack.bitrate == (uint)2634772927U);
                Debug.Assert(pack.framerate == (float) -1.2485323E38F);
            };
            GroundControl.VIDEO_STREAM_INFORMATION p269 = CommunicationChannel.new_VIDEO_STREAM_INFORMATION();
            PH.setPack(p269);
            p269.resolution_h = (ushort)(ushort)22936;
            p269.rotation = (ushort)(ushort)347;
            p269.uri_SET("isjryudbjqpmevoocEfiqfxscykuZvnckNvboapeduvgyxRxkuczwqeFqIifpawllkoquefvoukivxHxluvgbvuhoitdambpatsdmvhifetgswhqdBveysmbfaqlsmkvofbrhVNfbrbPbyiunfgyerxPaaknuwvyfIccqkxxeeziVvwxDrhlfppmrNwg", PH) ;
            p269.framerate = (float) -1.2485323E38F;
            p269.resolution_v = (ushort)(ushort)53279;
            p269.camera_id = (byte)(byte)23;
            p269.bitrate = (uint)2634772927U;
            p269.status = (byte)(byte)220;
            CommunicationChannel.instance.send(p269);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnSET_VIDEO_STREAM_SETTINGSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.resolution_h == (ushort)(ushort)59746);
                Debug.Assert(pack.camera_id == (byte)(byte)224);
                Debug.Assert(pack.rotation == (ushort)(ushort)53985);
                Debug.Assert(pack.framerate == (float) -3.0923748E37F);
                Debug.Assert(pack.resolution_v == (ushort)(ushort)53406);
                Debug.Assert(pack.target_system == (byte)(byte)136);
                Debug.Assert(pack.bitrate == (uint)235856834U);
                Debug.Assert(pack.target_component == (byte)(byte)165);
                Debug.Assert(pack.uri_LEN(ph) == 192);
                Debug.Assert(pack.uri_TRY(ph).Equals("jqvnabymgjmZymdmrabajqfolubfktlqOuwdemsxwkpfrsWbwubzigvkdWriurkzpjWMvytlMddtBtmiuZqmijwdmpqJbCjpXfltkhgmfcwhughrxuusAdbUcCmzZxxpxnqkcxdhzisgdqrxfvytocLcxliwQqroxauzXpnrbclkiAvjqvbpabakyxddysgj"));
            };
            GroundControl.SET_VIDEO_STREAM_SETTINGS p270 = CommunicationChannel.new_SET_VIDEO_STREAM_SETTINGS();
            PH.setPack(p270);
            p270.resolution_v = (ushort)(ushort)53406;
            p270.target_system = (byte)(byte)136;
            p270.rotation = (ushort)(ushort)53985;
            p270.target_component = (byte)(byte)165;
            p270.framerate = (float) -3.0923748E37F;
            p270.bitrate = (uint)235856834U;
            p270.uri_SET("jqvnabymgjmZymdmrabajqfolubfktlqOuwdemsxwkpfrsWbwubzigvkdWriurkzpjWMvytlMddtBtmiuZqmijwdmpqJbCjpXfltkhgmfcwhughrxuusAdbUcCmzZxxpxnqkcxdhzisgdqrxfvytocLcxliwQqroxauzXpnrbclkiAvjqvbpabakyxddysgj", PH) ;
            p270.resolution_h = (ushort)(ushort)59746;
            p270.camera_id = (byte)(byte)224;
            CommunicationChannel.instance.send(p270);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnWIFI_CONFIG_APReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.password_LEN(ph) == 36);
                Debug.Assert(pack.password_TRY(ph).Equals("tfqptnnzvhZqgdktshbktqyMhvfEfppwrejp"));
                Debug.Assert(pack.ssid_LEN(ph) == 19);
                Debug.Assert(pack.ssid_TRY(ph).Equals("rlicdrgkgZpxhkgkhpq"));
            };
            GroundControl.WIFI_CONFIG_AP p299 = CommunicationChannel.new_WIFI_CONFIG_AP();
            PH.setPack(p299);
            p299.ssid_SET("rlicdrgkgZpxhkgkhpq", PH) ;
            p299.password_SET("tfqptnnzvhZqgdktshbktqyMhvfEfppwrejp", PH) ;
            CommunicationChannel.instance.send(p299);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnPROTOCOL_VERSIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.max_version == (ushort)(ushort)43228);
                Debug.Assert(pack.library_version_hash.SequenceEqual(new byte[] {(byte)113, (byte)189, (byte)181, (byte)221, (byte)0, (byte)118, (byte)3, (byte)180}));
                Debug.Assert(pack.spec_version_hash.SequenceEqual(new byte[] {(byte)205, (byte)57, (byte)87, (byte)123, (byte)169, (byte)229, (byte)118, (byte)61}));
                Debug.Assert(pack.version == (ushort)(ushort)41733);
                Debug.Assert(pack.min_version == (ushort)(ushort)18162);
            };
            GroundControl.PROTOCOL_VERSION p300 = CommunicationChannel.new_PROTOCOL_VERSION();
            PH.setPack(p300);
            p300.max_version = (ushort)(ushort)43228;
            p300.min_version = (ushort)(ushort)18162;
            p300.spec_version_hash_SET(new byte[] {(byte)205, (byte)57, (byte)87, (byte)123, (byte)169, (byte)229, (byte)118, (byte)61}, 0) ;
            p300.version = (ushort)(ushort)41733;
            p300.library_version_hash_SET(new byte[] {(byte)113, (byte)189, (byte)181, (byte)221, (byte)0, (byte)118, (byte)3, (byte)180}, 0) ;
            CommunicationChannel.instance.send(p300);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnUAVCAN_NODE_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.health == UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_WARNING);
                Debug.Assert(pack.mode == UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_INITIALIZATION);
                Debug.Assert(pack.vendor_specific_status_code == (ushort)(ushort)30382);
                Debug.Assert(pack.time_usec == (ulong)6772183841048829321L);
                Debug.Assert(pack.sub_mode == (byte)(byte)128);
                Debug.Assert(pack.uptime_sec == (uint)3571841324U);
            };
            GroundControl.UAVCAN_NODE_STATUS p310 = CommunicationChannel.new_UAVCAN_NODE_STATUS();
            PH.setPack(p310);
            p310.time_usec = (ulong)6772183841048829321L;
            p310.mode = UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_INITIALIZATION;
            p310.sub_mode = (byte)(byte)128;
            p310.uptime_sec = (uint)3571841324U;
            p310.health = UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_WARNING;
            p310.vendor_specific_status_code = (ushort)(ushort)30382;
            CommunicationChannel.instance.send(p310);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnUAVCAN_NODE_INFOReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sw_version_minor == (byte)(byte)160);
                Debug.Assert(pack.hw_unique_id.SequenceEqual(new byte[] {(byte)87, (byte)238, (byte)237, (byte)222, (byte)189, (byte)25, (byte)95, (byte)62, (byte)224, (byte)149, (byte)255, (byte)164, (byte)88, (byte)22, (byte)33, (byte)86}));
                Debug.Assert(pack.time_usec == (ulong)8696540875694036986L);
                Debug.Assert(pack.sw_version_major == (byte)(byte)23);
                Debug.Assert(pack.hw_version_major == (byte)(byte)106);
                Debug.Assert(pack.uptime_sec == (uint)329060402U);
                Debug.Assert(pack.name_LEN(ph) == 7);
                Debug.Assert(pack.name_TRY(ph).Equals("glteucp"));
                Debug.Assert(pack.hw_version_minor == (byte)(byte)60);
                Debug.Assert(pack.sw_vcs_commit == (uint)310162619U);
            };
            GroundControl.UAVCAN_NODE_INFO p311 = CommunicationChannel.new_UAVCAN_NODE_INFO();
            PH.setPack(p311);
            p311.hw_version_major = (byte)(byte)106;
            p311.sw_version_major = (byte)(byte)23;
            p311.sw_vcs_commit = (uint)310162619U;
            p311.name_SET("glteucp", PH) ;
            p311.uptime_sec = (uint)329060402U;
            p311.hw_version_minor = (byte)(byte)60;
            p311.time_usec = (ulong)8696540875694036986L;
            p311.hw_unique_id_SET(new byte[] {(byte)87, (byte)238, (byte)237, (byte)222, (byte)189, (byte)25, (byte)95, (byte)62, (byte)224, (byte)149, (byte)255, (byte)164, (byte)88, (byte)22, (byte)33, (byte)86}, 0) ;
            p311.sw_version_minor = (byte)(byte)160;
            CommunicationChannel.instance.send(p311);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_REQUEST_READReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)179);
                Debug.Assert(pack.param_index == (short)(short) -31275);
                Debug.Assert(pack.target_component == (byte)(byte)139);
                Debug.Assert(pack.param_id_LEN(ph) == 11);
                Debug.Assert(pack.param_id_TRY(ph).Equals("puzrjDsfuwy"));
            };
            GroundControl.PARAM_EXT_REQUEST_READ p320 = CommunicationChannel.new_PARAM_EXT_REQUEST_READ();
            PH.setPack(p320);
            p320.target_component = (byte)(byte)139;
            p320.param_index = (short)(short) -31275;
            p320.target_system = (byte)(byte)179;
            p320.param_id_SET("puzrjDsfuwy", PH) ;
            CommunicationChannel.instance.send(p320);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)108);
                Debug.Assert(pack.target_component == (byte)(byte)108);
            };
            GroundControl.PARAM_EXT_REQUEST_LIST p321 = CommunicationChannel.new_PARAM_EXT_REQUEST_LIST();
            PH.setPack(p321);
            p321.target_component = (byte)(byte)108;
            p321.target_system = (byte)(byte)108;
            CommunicationChannel.instance.send(p321);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_VALUEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_id_LEN(ph) == 11);
                Debug.Assert(pack.param_id_TRY(ph).Equals("sSFoheCvtjp"));
                Debug.Assert(pack.param_count == (ushort)(ushort)6382);
                Debug.Assert(pack.param_index == (ushort)(ushort)61233);
                Debug.Assert(pack.param_value_LEN(ph) == 3);
                Debug.Assert(pack.param_value_TRY(ph).Equals("uqb"));
                Debug.Assert(pack.param_type == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT8);
            };
            GroundControl.PARAM_EXT_VALUE p322 = CommunicationChannel.new_PARAM_EXT_VALUE();
            PH.setPack(p322);
            p322.param_count = (ushort)(ushort)6382;
            p322.param_index = (ushort)(ushort)61233;
            p322.param_type = MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT8;
            p322.param_value_SET("uqb", PH) ;
            p322.param_id_SET("sSFoheCvtjp", PH) ;
            CommunicationChannel.instance.send(p322);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_SETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_value_LEN(ph) == 13);
                Debug.Assert(pack.param_value_TRY(ph).Equals("abqwjfwdlilcy"));
                Debug.Assert(pack.target_component == (byte)(byte)225);
                Debug.Assert(pack.param_type == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_REAL32);
                Debug.Assert(pack.param_id_LEN(ph) == 8);
                Debug.Assert(pack.param_id_TRY(ph).Equals("gbzLtzqg"));
                Debug.Assert(pack.target_system == (byte)(byte)77);
            };
            GroundControl.PARAM_EXT_SET p323 = CommunicationChannel.new_PARAM_EXT_SET();
            PH.setPack(p323);
            p323.target_system = (byte)(byte)77;
            p323.param_value_SET("abqwjfwdlilcy", PH) ;
            p323.target_component = (byte)(byte)225;
            p323.param_type = MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_REAL32;
            p323.param_id_SET("gbzLtzqg", PH) ;
            CommunicationChannel.instance.send(p323);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_id_LEN(ph) == 13);
                Debug.Assert(pack.param_id_TRY(ph).Equals("FiVtuVdpdxntt"));
                Debug.Assert(pack.param_result == PARAM_ACK.PARAM_ACK_FAILED);
                Debug.Assert(pack.param_value_LEN(ph) == 98);
                Debug.Assert(pack.param_value_TRY(ph).Equals("kMVsngqHzYcqTkknduniyveWmYenyrzryVmvesFSlbmenbbgoHpnzajVzzurusdwzyhphaioycBxwibeexZigszzdyqqnxefse"));
                Debug.Assert(pack.param_type == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT16);
            };
            GroundControl.PARAM_EXT_ACK p324 = CommunicationChannel.new_PARAM_EXT_ACK();
            PH.setPack(p324);
            p324.param_id_SET("FiVtuVdpdxntt", PH) ;
            p324.param_result = PARAM_ACK.PARAM_ACK_FAILED;
            p324.param_value_SET("kMVsngqHzYcqTkknduniyveWmYenyrzryVmvesFSlbmenbbgoHpnzajVzzurusdwzyhphaioycBxwibeexZigszzdyqqnxefse", PH) ;
            p324.param_type = MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT16;
            CommunicationChannel.instance.send(p324);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnOBSTACLE_DISTANCEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.distances.SequenceEqual(new ushort[] {(ushort)2521, (ushort)34588, (ushort)35022, (ushort)50391, (ushort)1028, (ushort)24465, (ushort)8432, (ushort)21034, (ushort)14758, (ushort)30042, (ushort)25736, (ushort)4560, (ushort)16314, (ushort)27796, (ushort)995, (ushort)32052, (ushort)38059, (ushort)46529, (ushort)37228, (ushort)39971, (ushort)38553, (ushort)19160, (ushort)51648, (ushort)40665, (ushort)17658, (ushort)8119, (ushort)6916, (ushort)54213, (ushort)62156, (ushort)32213, (ushort)45578, (ushort)49118, (ushort)46368, (ushort)23617, (ushort)41319, (ushort)3182, (ushort)7808, (ushort)540, (ushort)43959, (ushort)23890, (ushort)774, (ushort)41458, (ushort)6646, (ushort)57511, (ushort)60379, (ushort)3912, (ushort)3605, (ushort)30855, (ushort)13963, (ushort)18278, (ushort)44664, (ushort)5892, (ushort)21658, (ushort)2865, (ushort)43181, (ushort)36389, (ushort)45909, (ushort)31484, (ushort)6334, (ushort)44708, (ushort)22779, (ushort)28439, (ushort)2305, (ushort)14110, (ushort)26030, (ushort)38415, (ushort)55944, (ushort)7642, (ushort)59874, (ushort)46656, (ushort)14960, (ushort)7612}));
                Debug.Assert(pack.max_distance == (ushort)(ushort)46069);
                Debug.Assert(pack.sensor_type == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_ULTRASOUND);
                Debug.Assert(pack.min_distance == (ushort)(ushort)18650);
                Debug.Assert(pack.time_usec == (ulong)4424894582384595349L);
                Debug.Assert(pack.increment == (byte)(byte)12);
            };
            GroundControl.OBSTACLE_DISTANCE p330 = CommunicationChannel.new_OBSTACLE_DISTANCE();
            PH.setPack(p330);
            p330.distances_SET(new ushort[] {(ushort)2521, (ushort)34588, (ushort)35022, (ushort)50391, (ushort)1028, (ushort)24465, (ushort)8432, (ushort)21034, (ushort)14758, (ushort)30042, (ushort)25736, (ushort)4560, (ushort)16314, (ushort)27796, (ushort)995, (ushort)32052, (ushort)38059, (ushort)46529, (ushort)37228, (ushort)39971, (ushort)38553, (ushort)19160, (ushort)51648, (ushort)40665, (ushort)17658, (ushort)8119, (ushort)6916, (ushort)54213, (ushort)62156, (ushort)32213, (ushort)45578, (ushort)49118, (ushort)46368, (ushort)23617, (ushort)41319, (ushort)3182, (ushort)7808, (ushort)540, (ushort)43959, (ushort)23890, (ushort)774, (ushort)41458, (ushort)6646, (ushort)57511, (ushort)60379, (ushort)3912, (ushort)3605, (ushort)30855, (ushort)13963, (ushort)18278, (ushort)44664, (ushort)5892, (ushort)21658, (ushort)2865, (ushort)43181, (ushort)36389, (ushort)45909, (ushort)31484, (ushort)6334, (ushort)44708, (ushort)22779, (ushort)28439, (ushort)2305, (ushort)14110, (ushort)26030, (ushort)38415, (ushort)55944, (ushort)7642, (ushort)59874, (ushort)46656, (ushort)14960, (ushort)7612}, 0) ;
            p330.min_distance = (ushort)(ushort)18650;
            p330.max_distance = (ushort)(ushort)46069;
            p330.increment = (byte)(byte)12;
            p330.sensor_type = MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_ULTRASOUND;
            p330.time_usec = (ulong)4424894582384595349L;
            CommunicationChannel.instance.send(p330);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnUAVIONIX_ADSB_OUT_CFGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.gpsOffsetLat == UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT.UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT_RIGHT_6M);
                Debug.Assert(pack.aircraftSize == UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE.UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L65_59P5M);
                Debug.Assert(pack.stallSpeed == (ushort)(ushort)42460);
                Debug.Assert(pack.emitterType == ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_LIGHT);
                Debug.Assert(pack.callsign_LEN(ph) == 1);
                Debug.Assert(pack.callsign_TRY(ph).Equals("q"));
                Debug.Assert(pack.rfSelect == UAVIONIX_ADSB_OUT_RF_SELECT.UAVIONIX_ADSB_OUT_RF_SELECT_TX_ENABLED);
                Debug.Assert(pack.gpsOffsetLon == UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON.UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON_NO_DATA);
                Debug.Assert(pack.ICAO == (uint)1124267371U);
            };
            GroundControl.UAVIONIX_ADSB_OUT_CFG p10001 = CommunicationChannel.new_UAVIONIX_ADSB_OUT_CFG();
            PH.setPack(p10001);
            p10001.gpsOffsetLon = UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON.UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON_NO_DATA;
            p10001.emitterType = ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_LIGHT;
            p10001.gpsOffsetLat = UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT.UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT_RIGHT_6M;
            p10001.ICAO = (uint)1124267371U;
            p10001.rfSelect = UAVIONIX_ADSB_OUT_RF_SELECT.UAVIONIX_ADSB_OUT_RF_SELECT_TX_ENABLED;
            p10001.callsign_SET("q", PH) ;
            p10001.aircraftSize = UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE.UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L65_59P5M;
            p10001.stallSpeed = (ushort)(ushort)42460;
            CommunicationChannel.instance.send(p10001);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnUAVIONIX_ADSB_OUT_DYNAMICReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.accuracyVert == (ushort)(ushort)36700);
                Debug.Assert(pack.gpsLat == (int) -244117455);
                Debug.Assert(pack.squawk == (ushort)(ushort)40700);
                Debug.Assert(pack.velVert == (short)(short) -28866);
                Debug.Assert(pack.gpsFix == UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX.UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_RTK);
                Debug.Assert(pack.velNS == (short)(short)21114);
                Debug.Assert(pack.gpsLon == (int)1603865785);
                Debug.Assert(pack.VelEW == (short)(short) -13806);
                Debug.Assert(pack.state == (UAVIONIX_ADSB_OUT_DYNAMIC_STATE.UAVIONIX_ADSB_OUT_DYNAMIC_STATE_AUTOPILOT_ENABLED |
                                            UAVIONIX_ADSB_OUT_DYNAMIC_STATE.UAVIONIX_ADSB_OUT_DYNAMIC_STATE_INTENT_CHANGE));
                Debug.Assert(pack.gpsAlt == (int)257903075);
                Debug.Assert(pack.emergencyStatus == UAVIONIX_ADSB_EMERGENCY_STATUS.UAVIONIX_ADSB_OUT_NO_COMM_EMERGENCY);
                Debug.Assert(pack.accuracyHor == (uint)635474154U);
                Debug.Assert(pack.accuracyVel == (ushort)(ushort)42388);
                Debug.Assert(pack.utcTime == (uint)1528704610U);
                Debug.Assert(pack.baroAltMSL == (int)818746744);
                Debug.Assert(pack.numSats == (byte)(byte)250);
            };
            GroundControl.UAVIONIX_ADSB_OUT_DYNAMIC p10002 = CommunicationChannel.new_UAVIONIX_ADSB_OUT_DYNAMIC();
            PH.setPack(p10002);
            p10002.emergencyStatus = UAVIONIX_ADSB_EMERGENCY_STATUS.UAVIONIX_ADSB_OUT_NO_COMM_EMERGENCY;
            p10002.squawk = (ushort)(ushort)40700;
            p10002.accuracyVert = (ushort)(ushort)36700;
            p10002.utcTime = (uint)1528704610U;
            p10002.baroAltMSL = (int)818746744;
            p10002.velNS = (short)(short)21114;
            p10002.VelEW = (short)(short) -13806;
            p10002.accuracyHor = (uint)635474154U;
            p10002.velVert = (short)(short) -28866;
            p10002.gpsAlt = (int)257903075;
            p10002.state = (UAVIONIX_ADSB_OUT_DYNAMIC_STATE.UAVIONIX_ADSB_OUT_DYNAMIC_STATE_AUTOPILOT_ENABLED |
                            UAVIONIX_ADSB_OUT_DYNAMIC_STATE.UAVIONIX_ADSB_OUT_DYNAMIC_STATE_INTENT_CHANGE);
            p10002.gpsLat = (int) -244117455;
            p10002.gpsLon = (int)1603865785;
            p10002.accuracyVel = (ushort)(ushort)42388;
            p10002.gpsFix = UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX.UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_RTK;
            p10002.numSats = (byte)(byte)250;
            CommunicationChannel.instance.send(p10002);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnUAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.rfHealth == UAVIONIX_ADSB_RF_HEALTH.UAVIONIX_ADSB_RF_HEALTH_INITIALIZING);
            };
            GroundControl.UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT p10003 = CommunicationChannel.new_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT();
            PH.setPack(p10003);
            p10003.rfHealth = UAVIONIX_ADSB_RF_HEALTH.UAVIONIX_ADSB_RF_HEALTH_INITIALIZING;
            CommunicationChannel.instance.send(p10003);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnDEVICE_OP_READReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.request_id == (uint)2426106034U);
                Debug.Assert(pack.target_system == (byte)(byte)64);
                Debug.Assert(pack.target_component == (byte)(byte)75);
                Debug.Assert(pack.count == (byte)(byte)246);
                Debug.Assert(pack.bus == (byte)(byte)137);
                Debug.Assert(pack.address == (byte)(byte)231);
                Debug.Assert(pack.busname_LEN(ph) == 21);
                Debug.Assert(pack.busname_TRY(ph).Equals("pbkroprypQoPmnntjrnfl"));
                Debug.Assert(pack.regstart == (byte)(byte)150);
                Debug.Assert(pack.bustype == DEVICE_OP_BUSTYPE.DEVICE_OP_BUSTYPE_SPI);
            };
            GroundControl.DEVICE_OP_READ p11000 = CommunicationChannel.new_DEVICE_OP_READ();
            PH.setPack(p11000);
            p11000.busname_SET("pbkroprypQoPmnntjrnfl", PH) ;
            p11000.regstart = (byte)(byte)150;
            p11000.request_id = (uint)2426106034U;
            p11000.target_system = (byte)(byte)64;
            p11000.address = (byte)(byte)231;
            p11000.target_component = (byte)(byte)75;
            p11000.count = (byte)(byte)246;
            p11000.bustype = DEVICE_OP_BUSTYPE.DEVICE_OP_BUSTYPE_SPI;
            p11000.bus = (byte)(byte)137;
            CommunicationChannel.instance.send(p11000);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnDEVICE_OP_READ_REPLYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.regstart == (byte)(byte)255);
                Debug.Assert(pack.result == (byte)(byte)51);
                Debug.Assert(pack.request_id == (uint)477232725U);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)197, (byte)31, (byte)58, (byte)177, (byte)58, (byte)232, (byte)91, (byte)191, (byte)53, (byte)187, (byte)143, (byte)217, (byte)107, (byte)54, (byte)118, (byte)33, (byte)69, (byte)188, (byte)45, (byte)10, (byte)146, (byte)69, (byte)6, (byte)148, (byte)20, (byte)221, (byte)158, (byte)250, (byte)245, (byte)28, (byte)217, (byte)68, (byte)146, (byte)150, (byte)85, (byte)0, (byte)153, (byte)171, (byte)25, (byte)121, (byte)141, (byte)157, (byte)217, (byte)25, (byte)188, (byte)78, (byte)187, (byte)252, (byte)253, (byte)180, (byte)202, (byte)95, (byte)170, (byte)39, (byte)16, (byte)115, (byte)184, (byte)165, (byte)135, (byte)109, (byte)220, (byte)164, (byte)189, (byte)232, (byte)246, (byte)74, (byte)208, (byte)157, (byte)121, (byte)53, (byte)181, (byte)249, (byte)115, (byte)12, (byte)21, (byte)165, (byte)0, (byte)247, (byte)220, (byte)220, (byte)14, (byte)42, (byte)110, (byte)51, (byte)223, (byte)204, (byte)69, (byte)51, (byte)23, (byte)19, (byte)96, (byte)5, (byte)120, (byte)136, (byte)250, (byte)208, (byte)80, (byte)162, (byte)90, (byte)94, (byte)11, (byte)222, (byte)195, (byte)176, (byte)37, (byte)21, (byte)54, (byte)26, (byte)189, (byte)98, (byte)66, (byte)242, (byte)69, (byte)69, (byte)175, (byte)184, (byte)127, (byte)252, (byte)82, (byte)81, (byte)45, (byte)200, (byte)135, (byte)124, (byte)29, (byte)227, (byte)59, (byte)248}));
                Debug.Assert(pack.count == (byte)(byte)192);
            };
            GroundControl.DEVICE_OP_READ_REPLY p11001 = CommunicationChannel.new_DEVICE_OP_READ_REPLY();
            PH.setPack(p11001);
            p11001.result = (byte)(byte)51;
            p11001.request_id = (uint)477232725U;
            p11001.count = (byte)(byte)192;
            p11001.data__SET(new byte[] {(byte)197, (byte)31, (byte)58, (byte)177, (byte)58, (byte)232, (byte)91, (byte)191, (byte)53, (byte)187, (byte)143, (byte)217, (byte)107, (byte)54, (byte)118, (byte)33, (byte)69, (byte)188, (byte)45, (byte)10, (byte)146, (byte)69, (byte)6, (byte)148, (byte)20, (byte)221, (byte)158, (byte)250, (byte)245, (byte)28, (byte)217, (byte)68, (byte)146, (byte)150, (byte)85, (byte)0, (byte)153, (byte)171, (byte)25, (byte)121, (byte)141, (byte)157, (byte)217, (byte)25, (byte)188, (byte)78, (byte)187, (byte)252, (byte)253, (byte)180, (byte)202, (byte)95, (byte)170, (byte)39, (byte)16, (byte)115, (byte)184, (byte)165, (byte)135, (byte)109, (byte)220, (byte)164, (byte)189, (byte)232, (byte)246, (byte)74, (byte)208, (byte)157, (byte)121, (byte)53, (byte)181, (byte)249, (byte)115, (byte)12, (byte)21, (byte)165, (byte)0, (byte)247, (byte)220, (byte)220, (byte)14, (byte)42, (byte)110, (byte)51, (byte)223, (byte)204, (byte)69, (byte)51, (byte)23, (byte)19, (byte)96, (byte)5, (byte)120, (byte)136, (byte)250, (byte)208, (byte)80, (byte)162, (byte)90, (byte)94, (byte)11, (byte)222, (byte)195, (byte)176, (byte)37, (byte)21, (byte)54, (byte)26, (byte)189, (byte)98, (byte)66, (byte)242, (byte)69, (byte)69, (byte)175, (byte)184, (byte)127, (byte)252, (byte)82, (byte)81, (byte)45, (byte)200, (byte)135, (byte)124, (byte)29, (byte)227, (byte)59, (byte)248}, 0) ;
            p11001.regstart = (byte)(byte)255;
            CommunicationChannel.instance.send(p11001);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnDEVICE_OP_WRITEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.bustype == DEVICE_OP_BUSTYPE.DEVICE_OP_BUSTYPE_I2C);
                Debug.Assert(pack.target_system == (byte)(byte)32);
                Debug.Assert(pack.count == (byte)(byte)148);
                Debug.Assert(pack.bus == (byte)(byte)130);
                Debug.Assert(pack.regstart == (byte)(byte)67);
                Debug.Assert(pack.busname_LEN(ph) == 28);
                Debug.Assert(pack.busname_TRY(ph).Equals("iimjfgnbRoaAOvuofnrzfdUkzzzh"));
                Debug.Assert(pack.target_component == (byte)(byte)243);
                Debug.Assert(pack.address == (byte)(byte)13);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)78, (byte)167, (byte)85, (byte)96, (byte)86, (byte)118, (byte)187, (byte)194, (byte)56, (byte)216, (byte)139, (byte)54, (byte)247, (byte)59, (byte)131, (byte)243, (byte)176, (byte)45, (byte)216, (byte)82, (byte)178, (byte)172, (byte)125, (byte)211, (byte)156, (byte)26, (byte)243, (byte)54, (byte)78, (byte)42, (byte)44, (byte)50, (byte)159, (byte)199, (byte)231, (byte)145, (byte)54, (byte)119, (byte)103, (byte)35, (byte)93, (byte)6, (byte)126, (byte)56, (byte)179, (byte)234, (byte)23, (byte)19, (byte)61, (byte)228, (byte)235, (byte)95, (byte)34, (byte)199, (byte)199, (byte)80, (byte)160, (byte)22, (byte)7, (byte)35, (byte)154, (byte)247, (byte)106, (byte)61, (byte)247, (byte)250, (byte)19, (byte)86, (byte)23, (byte)24, (byte)196, (byte)243, (byte)27, (byte)217, (byte)127, (byte)59, (byte)14, (byte)215, (byte)11, (byte)160, (byte)181, (byte)236, (byte)180, (byte)233, (byte)213, (byte)218, (byte)199, (byte)252, (byte)42, (byte)154, (byte)230, (byte)206, (byte)49, (byte)79, (byte)124, (byte)31, (byte)108, (byte)214, (byte)222, (byte)32, (byte)142, (byte)236, (byte)228, (byte)199, (byte)182, (byte)37, (byte)198, (byte)185, (byte)240, (byte)222, (byte)100, (byte)210, (byte)55, (byte)225, (byte)94, (byte)88, (byte)25, (byte)83, (byte)187, (byte)225, (byte)203, (byte)80, (byte)195, (byte)24, (byte)243, (byte)124, (byte)140, (byte)8}));
                Debug.Assert(pack.request_id == (uint)1607289194U);
            };
            GroundControl.DEVICE_OP_WRITE p11002 = CommunicationChannel.new_DEVICE_OP_WRITE();
            PH.setPack(p11002);
            p11002.target_component = (byte)(byte)243;
            p11002.busname_SET("iimjfgnbRoaAOvuofnrzfdUkzzzh", PH) ;
            p11002.request_id = (uint)1607289194U;
            p11002.bustype = DEVICE_OP_BUSTYPE.DEVICE_OP_BUSTYPE_I2C;
            p11002.count = (byte)(byte)148;
            p11002.regstart = (byte)(byte)67;
            p11002.data__SET(new byte[] {(byte)78, (byte)167, (byte)85, (byte)96, (byte)86, (byte)118, (byte)187, (byte)194, (byte)56, (byte)216, (byte)139, (byte)54, (byte)247, (byte)59, (byte)131, (byte)243, (byte)176, (byte)45, (byte)216, (byte)82, (byte)178, (byte)172, (byte)125, (byte)211, (byte)156, (byte)26, (byte)243, (byte)54, (byte)78, (byte)42, (byte)44, (byte)50, (byte)159, (byte)199, (byte)231, (byte)145, (byte)54, (byte)119, (byte)103, (byte)35, (byte)93, (byte)6, (byte)126, (byte)56, (byte)179, (byte)234, (byte)23, (byte)19, (byte)61, (byte)228, (byte)235, (byte)95, (byte)34, (byte)199, (byte)199, (byte)80, (byte)160, (byte)22, (byte)7, (byte)35, (byte)154, (byte)247, (byte)106, (byte)61, (byte)247, (byte)250, (byte)19, (byte)86, (byte)23, (byte)24, (byte)196, (byte)243, (byte)27, (byte)217, (byte)127, (byte)59, (byte)14, (byte)215, (byte)11, (byte)160, (byte)181, (byte)236, (byte)180, (byte)233, (byte)213, (byte)218, (byte)199, (byte)252, (byte)42, (byte)154, (byte)230, (byte)206, (byte)49, (byte)79, (byte)124, (byte)31, (byte)108, (byte)214, (byte)222, (byte)32, (byte)142, (byte)236, (byte)228, (byte)199, (byte)182, (byte)37, (byte)198, (byte)185, (byte)240, (byte)222, (byte)100, (byte)210, (byte)55, (byte)225, (byte)94, (byte)88, (byte)25, (byte)83, (byte)187, (byte)225, (byte)203, (byte)80, (byte)195, (byte)24, (byte)243, (byte)124, (byte)140, (byte)8}, 0) ;
            p11002.address = (byte)(byte)13;
            p11002.target_system = (byte)(byte)32;
            p11002.bus = (byte)(byte)130;
            CommunicationChannel.instance.send(p11002);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnDEVICE_OP_WRITE_REPLYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.request_id == (uint)3159874842U);
                Debug.Assert(pack.result == (byte)(byte)166);
            };
            GroundControl.DEVICE_OP_WRITE_REPLY p11003 = CommunicationChannel.new_DEVICE_OP_WRITE_REPLY();
            PH.setPack(p11003);
            p11003.request_id = (uint)3159874842U;
            p11003.result = (byte)(byte)166;
            CommunicationChannel.instance.send(p11003);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnADAP_TUNINGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sigma_dot == (float) -2.8071918E38F);
                Debug.Assert(pack.achieved == (float) -2.766013E38F);
                Debug.Assert(pack.theta == (float)3.5570258E37F);
                Debug.Assert(pack.axis == PID_TUNING_AXIS.PID_TUNING_STEER);
                Debug.Assert(pack.omega == (float) -4.0136391E37F);
                Debug.Assert(pack.omega_dot == (float)2.2966476E38F);
                Debug.Assert(pack.error == (float) -2.7543336E38F);
                Debug.Assert(pack.desired == (float)1.9666686E38F);
                Debug.Assert(pack.f_dot == (float) -1.0510079E38F);
                Debug.Assert(pack.f == (float) -3.3315044E38F);
                Debug.Assert(pack.theta_dot == (float)1.4424216E38F);
                Debug.Assert(pack.sigma == (float)2.5955268E37F);
                Debug.Assert(pack.u == (float) -3.1780635E38F);
            };
            GroundControl.ADAP_TUNING p11010 = CommunicationChannel.new_ADAP_TUNING();
            PH.setPack(p11010);
            p11010.omega = (float) -4.0136391E37F;
            p11010.error = (float) -2.7543336E38F;
            p11010.u = (float) -3.1780635E38F;
            p11010.f = (float) -3.3315044E38F;
            p11010.axis = PID_TUNING_AXIS.PID_TUNING_STEER;
            p11010.theta = (float)3.5570258E37F;
            p11010.sigma_dot = (float) -2.8071918E38F;
            p11010.f_dot = (float) -1.0510079E38F;
            p11010.desired = (float)1.9666686E38F;
            p11010.achieved = (float) -2.766013E38F;
            p11010.omega_dot = (float)2.2966476E38F;
            p11010.sigma = (float)2.5955268E37F;
            p11010.theta_dot = (float)1.4424216E38F;
            CommunicationChannel.instance.send(p11010);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnVISION_POSITION_DELTAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.angle_delta.SequenceEqual(new float[] {3.2442016E38F, 3.0223062E38F, 2.9090995E38F}));
                Debug.Assert(pack.time_usec == (ulong)8654725113806901755L);
                Debug.Assert(pack.position_delta.SequenceEqual(new float[] {2.7181102E38F, -2.0566687E37F, 2.6687272E38F}));
                Debug.Assert(pack.confidence == (float)4.149836E37F);
                Debug.Assert(pack.time_delta_usec == (ulong)5305502621403815329L);
            };
            GroundControl.VISION_POSITION_DELTA p11011 = CommunicationChannel.new_VISION_POSITION_DELTA();
            PH.setPack(p11011);
            p11011.position_delta_SET(new float[] {2.7181102E38F, -2.0566687E37F, 2.6687272E38F}, 0) ;
            p11011.time_usec = (ulong)8654725113806901755L;
            p11011.time_delta_usec = (ulong)5305502621403815329L;
            p11011.angle_delta_SET(new float[] {3.2442016E38F, 3.0223062E38F, 2.9090995E38F}, 0) ;
            p11011.confidence = (float)4.149836E37F;
            CommunicationChannel.instance.send(p11011);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
        }
    }
}