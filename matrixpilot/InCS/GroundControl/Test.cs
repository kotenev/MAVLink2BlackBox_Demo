
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
                    ulong id = id__J(value);
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
                    ulong id = 0;
                    switch(value)
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
                            throw  new ArgumentException("Unknown enum " + value);
                    }
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
                    ulong id = 0;
                    switch(value)
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
                            throw  new ArgumentException("Unknown enum " + value);
                    }
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
                    ulong id = 0;
                    switch(value)
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
                            throw  new ArgumentException("Unknown enum " + value);
                    }
                    BitUtils.set_bits(id, 8, data, 276);
                }
            }

            public MAV_MISSION_TYPE mission_type //Mission type, see MAV_MISSION_TYPE
            {
                set
                {
                    ulong id = 0;
                    switch(value)
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
                            throw  new ArgumentException("Unknown enum " + value);
                    }
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
                    ulong id = 0;
                    switch(value)
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
                            throw  new ArgumentException("Unknown enum " + value);
                    }
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
                    ulong id = 0;
                    switch(value)
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
                            throw  new ArgumentException("Unknown enum " + value);
                    }
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
                    ulong id = 0;
                    switch(value)
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
                            throw  new ArgumentException("Unknown enum " + value);
                    }
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
                    ulong id = 0;
                    switch(value)
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
                            throw  new ArgumentException("Unknown enum " + value);
                    }
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
                    ulong id = 0;
                    switch(value)
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
                            throw  new ArgumentException("Unknown enum " + value);
                    }
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
                    ulong id = 0;
                    switch(value)
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
                            throw  new ArgumentException("Unknown enum " + value);
                    }
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
                    ulong id = 0;
                    switch(value)
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
                            throw  new ArgumentException("Unknown enum " + value);
                    }
                    BitUtils.set_bits(id, 8, data, 276);
                }
            }

            public MAV_MISSION_TYPE mission_type //Mission type, see MAV_MISSION_TYPE
            {
                set
                {
                    ulong id = 0;
                    switch(value)
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
                            throw  new ArgumentException("Unknown enum " + value);
                    }
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
                    ulong id = 0;
                    switch(value)
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
                            throw  new ArgumentException("Unknown enum " + value);
                    }
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
                    ulong id = 0;
                    switch(value)
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
                            throw  new ArgumentException("Unknown enum " + value);
                    }
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
                    ulong id = 0;
                    switch(value)
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
                            throw  new ArgumentException("Unknown enum " + value);
                    }
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
                Debug.Assert(pack.autopilot == MAV_AUTOPILOT.MAV_AUTOPILOT_SMACCMPILOT);
                Debug.Assert(pack.type == MAV_TYPE.MAV_TYPE_SURFACE_BOAT);
                Debug.Assert(pack.base_mode == (MAV_MODE_FLAG.MAV_MODE_FLAG_TEST_ENABLED |
                                                MAV_MODE_FLAG.MAV_MODE_FLAG_AUTO_ENABLED |
                                                MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED |
                                                MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED));
                Debug.Assert(pack.mavlink_version == (byte)(byte)186);
                Debug.Assert(pack.custom_mode == (uint)790542260U);
                Debug.Assert(pack.system_status == MAV_STATE.MAV_STATE_STANDBY);
            };
            HEARTBEAT p0 = new HEARTBEAT();
            PH.setPack(p0);
            p0.mavlink_version = (byte)(byte)186;
            p0.custom_mode = (uint)790542260U;
            p0.autopilot = MAV_AUTOPILOT.MAV_AUTOPILOT_SMACCMPILOT;
            p0.base_mode = (MAV_MODE_FLAG.MAV_MODE_FLAG_TEST_ENABLED |
                            MAV_MODE_FLAG.MAV_MODE_FLAG_AUTO_ENABLED |
                            MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED |
                            MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED);
            p0.system_status = MAV_STATE.MAV_STATE_STANDBY;
            p0.type = MAV_TYPE.MAV_TYPE_SURFACE_BOAT;
            ADV_TEST_CH.send(p0);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSYS_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.load == (ushort)(ushort)63430);
                Debug.Assert(pack.errors_count2 == (ushort)(ushort)51594);
                Debug.Assert(pack.errors_count3 == (ushort)(ushort)10198);
                Debug.Assert(pack.errors_count1 == (ushort)(ushort)42476);
                Debug.Assert(pack.errors_count4 == (ushort)(ushort)5335);
                Debug.Assert(pack.battery_remaining == (sbyte)(sbyte) - 4);
                Debug.Assert(pack.current_battery == (short)(short)9452);
                Debug.Assert(pack.voltage_battery == (ushort)(ushort)42273);
                Debug.Assert(pack.onboard_control_sensors_present == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO2 |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_LOGGING |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION));
                Debug.Assert(pack.errors_comm == (ushort)(ushort)30353);
                Debug.Assert(pack.drop_rate_comm == (ushort)(ushort)27456);
                Debug.Assert(pack.onboard_control_sensors_health == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_GPS |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_BATTERY |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE));
                Debug.Assert(pack.onboard_control_sensors_enabled == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_LOGGING |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL));
            };
            SYS_STATUS p1 = new SYS_STATUS();
            PH.setPack(p1);
            p1.voltage_battery = (ushort)(ushort)42273;
            p1.drop_rate_comm = (ushort)(ushort)27456;
            p1.onboard_control_sensors_present = (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO2 |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_LOGGING |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION);
            p1.current_battery = (short)(short)9452;
            p1.errors_count1 = (ushort)(ushort)42476;
            p1.errors_count3 = (ushort)(ushort)10198;
            p1.errors_comm = (ushort)(ushort)30353;
            p1.onboard_control_sensors_enabled = (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_LOGGING |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL);
            p1.onboard_control_sensors_health = (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_GPS |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_BATTERY |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE);
            p1.battery_remaining = (sbyte)(sbyte) - 4;
            p1.errors_count4 = (ushort)(ushort)5335;
            p1.errors_count2 = (ushort)(ushort)51594;
            p1.load = (ushort)(ushort)63430;
            ADV_TEST_CH.send(p1);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSYSTEM_TIMEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_unix_usec == (ulong)3515845924307652412L);
                Debug.Assert(pack.time_boot_ms == (uint)2580395345U);
            };
            SYSTEM_TIME p2 = new SYSTEM_TIME();
            PH.setPack(p2);
            p2.time_boot_ms = (uint)2580395345U;
            p2.time_unix_usec = (ulong)3515845924307652412L;
            ADV_TEST_CH.send(p2);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPOSITION_TARGET_LOCAL_NEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.z == (float)1.584286E38F);
                Debug.Assert(pack.afy == (float) -1.856835E36F);
                Debug.Assert(pack.vy == (float)5.6419275E37F);
                Debug.Assert(pack.yaw == (float)2.9598879E38F);
                Debug.Assert(pack.coordinate_frame == MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT);
                Debug.Assert(pack.type_mask == (ushort)(ushort)29534);
                Debug.Assert(pack.yaw_rate == (float) -2.3888776E38F);
                Debug.Assert(pack.vx == (float) -2.392124E38F);
                Debug.Assert(pack.afz == (float)1.4474702E38F);
                Debug.Assert(pack.vz == (float)2.6321414E38F);
                Debug.Assert(pack.y == (float) -1.2197028E38F);
                Debug.Assert(pack.afx == (float) -7.9655594E37F);
                Debug.Assert(pack.x == (float)2.9871636E37F);
                Debug.Assert(pack.time_boot_ms == (uint)1999915660U);
            };
            POSITION_TARGET_LOCAL_NED p3 = new POSITION_TARGET_LOCAL_NED();
            PH.setPack(p3);
            p3.vy = (float)5.6419275E37F;
            p3.yaw_rate = (float) -2.3888776E38F;
            p3.coordinate_frame = MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT;
            p3.afz = (float)1.4474702E38F;
            p3.vz = (float)2.6321414E38F;
            p3.afx = (float) -7.9655594E37F;
            p3.afy = (float) -1.856835E36F;
            p3.z = (float)1.584286E38F;
            p3.vx = (float) -2.392124E38F;
            p3.yaw = (float)2.9598879E38F;
            p3.type_mask = (ushort)(ushort)29534;
            p3.time_boot_ms = (uint)1999915660U;
            p3.y = (float) -1.2197028E38F;
            p3.x = (float)2.9871636E37F;
            ADV_TEST_CH.send(p3);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPINGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)5175574554305142038L);
                Debug.Assert(pack.target_component == (byte)(byte)242);
                Debug.Assert(pack.seq == (uint)3241649465U);
                Debug.Assert(pack.target_system == (byte)(byte)203);
            };
            PING p4 = new PING();
            PH.setPack(p4);
            p4.seq = (uint)3241649465U;
            p4.target_component = (byte)(byte)242;
            p4.time_usec = (ulong)5175574554305142038L;
            p4.target_system = (byte)(byte)203;
            ADV_TEST_CH.send(p4);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCHANGE_OPERATOR_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.passkey_LEN(ph) == 4);
                Debug.Assert(pack.passkey_TRY(ph).Equals("yxYo"));
                Debug.Assert(pack.control_request == (byte)(byte)182);
                Debug.Assert(pack.target_system == (byte)(byte)173);
                Debug.Assert(pack.version == (byte)(byte)31);
            };
            CHANGE_OPERATOR_CONTROL p5 = new CHANGE_OPERATOR_CONTROL();
            PH.setPack(p5);
            p5.version = (byte)(byte)31;
            p5.passkey_SET("yxYo", PH) ;
            p5.target_system = (byte)(byte)173;
            p5.control_request = (byte)(byte)182;
            ADV_TEST_CH.send(p5);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCHANGE_OPERATOR_CONTROL_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.control_request == (byte)(byte)207);
                Debug.Assert(pack.ack == (byte)(byte)104);
                Debug.Assert(pack.gcs_system_id == (byte)(byte)5);
            };
            CHANGE_OPERATOR_CONTROL_ACK p6 = new CHANGE_OPERATOR_CONTROL_ACK();
            PH.setPack(p6);
            p6.ack = (byte)(byte)104;
            p6.control_request = (byte)(byte)207;
            p6.gcs_system_id = (byte)(byte)5;
            ADV_TEST_CH.send(p6);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnAUTH_KEYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.key_LEN(ph) == 1);
                Debug.Assert(pack.key_TRY(ph).Equals("V"));
            };
            AUTH_KEY p7 = new AUTH_KEY();
            PH.setPack(p7);
            p7.key_SET("V", PH) ;
            ADV_TEST_CH.send(p7);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_MODEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)91);
                Debug.Assert(pack.base_mode == MAV_MODE.MAV_MODE_PREFLIGHT);
                Debug.Assert(pack.custom_mode == (uint)1435876646U);
            };
            SET_MODE p11 = new SET_MODE();
            PH.setPack(p11);
            p11.custom_mode = (uint)1435876646U;
            p11.base_mode = MAV_MODE.MAV_MODE_PREFLIGHT;
            p11.target_system = (byte)(byte)91;
            ADV_TEST_CH.send(p11);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_REQUEST_READReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)174);
                Debug.Assert(pack.param_id_LEN(ph) == 4);
                Debug.Assert(pack.param_id_TRY(ph).Equals("igil"));
                Debug.Assert(pack.target_component == (byte)(byte)156);
                Debug.Assert(pack.param_index == (short)(short)5019);
            };
            PARAM_REQUEST_READ p20 = new PARAM_REQUEST_READ();
            PH.setPack(p20);
            p20.target_system = (byte)(byte)174;
            p20.param_id_SET("igil", PH) ;
            p20.param_index = (short)(short)5019;
            p20.target_component = (byte)(byte)156;
            ADV_TEST_CH.send(p20);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)173);
                Debug.Assert(pack.target_system == (byte)(byte)36);
            };
            PARAM_REQUEST_LIST p21 = new PARAM_REQUEST_LIST();
            PH.setPack(p21);
            p21.target_component = (byte)(byte)173;
            p21.target_system = (byte)(byte)36;
            ADV_TEST_CH.send(p21);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_VALUEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_index == (ushort)(ushort)59906);
                Debug.Assert(pack.param_type == MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT32);
                Debug.Assert(pack.param_id_LEN(ph) == 13);
                Debug.Assert(pack.param_id_TRY(ph).Equals("lrpjtqfMxqpwx"));
                Debug.Assert(pack.param_count == (ushort)(ushort)37168);
                Debug.Assert(pack.param_value == (float) -1.5455312E38F);
            };
            PARAM_VALUE p22 = new PARAM_VALUE();
            PH.setPack(p22);
            p22.param_value = (float) -1.5455312E38F;
            p22.param_id_SET("lrpjtqfMxqpwx", PH) ;
            p22.param_type = MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT32;
            p22.param_count = (ushort)(ushort)37168;
            p22.param_index = (ushort)(ushort)59906;
            ADV_TEST_CH.send(p22);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_SETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_type == MAV_PARAM_TYPE.MAV_PARAM_TYPE_REAL32);
                Debug.Assert(pack.target_component == (byte)(byte)244);
                Debug.Assert(pack.param_id_LEN(ph) == 12);
                Debug.Assert(pack.param_id_TRY(ph).Equals("gWkhgUzeteAd"));
                Debug.Assert(pack.target_system == (byte)(byte)235);
                Debug.Assert(pack.param_value == (float)2.5758013E38F);
            };
            PARAM_SET p23 = new PARAM_SET();
            PH.setPack(p23);
            p23.param_type = MAV_PARAM_TYPE.MAV_PARAM_TYPE_REAL32;
            p23.target_component = (byte)(byte)244;
            p23.target_system = (byte)(byte)235;
            p23.param_id_SET("gWkhgUzeteAd", PH) ;
            p23.param_value = (float)2.5758013E38F;
            ADV_TEST_CH.send(p23);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_RAW_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.hdg_acc_TRY(ph) == (uint)2899896287U);
                Debug.Assert(pack.vel_acc_TRY(ph) == (uint)1324357571U);
                Debug.Assert(pack.time_usec == (ulong)1124843675979525133L);
                Debug.Assert(pack.eph == (ushort)(ushort)27847);
                Debug.Assert(pack.cog == (ushort)(ushort)14007);
                Debug.Assert(pack.h_acc_TRY(ph) == (uint)1658415074U);
                Debug.Assert(pack.fix_type == GPS_FIX_TYPE.GPS_FIX_TYPE_PPP);
                Debug.Assert(pack.epv == (ushort)(ushort)56675);
                Debug.Assert(pack.lat == (int) -1431654962);
                Debug.Assert(pack.satellites_visible == (byte)(byte)142);
                Debug.Assert(pack.v_acc_TRY(ph) == (uint)1378394169U);
                Debug.Assert(pack.alt == (int)595240334);
                Debug.Assert(pack.alt_ellipsoid_TRY(ph) == (int)1589018889);
                Debug.Assert(pack.vel == (ushort)(ushort)34818);
                Debug.Assert(pack.lon == (int) -1073709546);
            };
            GPS_RAW_INT p24 = new GPS_RAW_INT();
            PH.setPack(p24);
            p24.fix_type = GPS_FIX_TYPE.GPS_FIX_TYPE_PPP;
            p24.vel_acc_SET((uint)1324357571U, PH) ;
            p24.cog = (ushort)(ushort)14007;
            p24.time_usec = (ulong)1124843675979525133L;
            p24.h_acc_SET((uint)1658415074U, PH) ;
            p24.alt = (int)595240334;
            p24.eph = (ushort)(ushort)27847;
            p24.lon = (int) -1073709546;
            p24.lat = (int) -1431654962;
            p24.hdg_acc_SET((uint)2899896287U, PH) ;
            p24.vel = (ushort)(ushort)34818;
            p24.epv = (ushort)(ushort)56675;
            p24.alt_ellipsoid_SET((int)1589018889, PH) ;
            p24.satellites_visible = (byte)(byte)142;
            p24.v_acc_SET((uint)1378394169U, PH) ;
            ADV_TEST_CH.send(p24);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.satellite_used.SequenceEqual(new byte[] {(byte)152, (byte)31, (byte)71, (byte)146, (byte)198, (byte)199, (byte)183, (byte)20, (byte)201, (byte)106, (byte)3, (byte)106, (byte)211, (byte)57, (byte)52, (byte)199, (byte)161, (byte)123, (byte)95, (byte)30}));
                Debug.Assert(pack.satellite_prn.SequenceEqual(new byte[] {(byte)173, (byte)231, (byte)173, (byte)97, (byte)137, (byte)255, (byte)121, (byte)217, (byte)235, (byte)140, (byte)135, (byte)22, (byte)206, (byte)101, (byte)95, (byte)142, (byte)208, (byte)239, (byte)78, (byte)176}));
                Debug.Assert(pack.satellite_snr.SequenceEqual(new byte[] {(byte)161, (byte)145, (byte)37, (byte)54, (byte)212, (byte)200, (byte)164, (byte)151, (byte)201, (byte)158, (byte)191, (byte)113, (byte)185, (byte)37, (byte)120, (byte)138, (byte)81, (byte)144, (byte)140, (byte)177}));
                Debug.Assert(pack.satellites_visible == (byte)(byte)9);
                Debug.Assert(pack.satellite_elevation.SequenceEqual(new byte[] {(byte)240, (byte)82, (byte)92, (byte)67, (byte)241, (byte)122, (byte)119, (byte)116, (byte)254, (byte)74, (byte)212, (byte)250, (byte)233, (byte)68, (byte)186, (byte)222, (byte)103, (byte)170, (byte)241, (byte)246}));
                Debug.Assert(pack.satellite_azimuth.SequenceEqual(new byte[] {(byte)154, (byte)98, (byte)175, (byte)40, (byte)205, (byte)187, (byte)13, (byte)220, (byte)111, (byte)137, (byte)181, (byte)92, (byte)123, (byte)16, (byte)218, (byte)53, (byte)183, (byte)122, (byte)13, (byte)249}));
            };
            GPS_STATUS p25 = new GPS_STATUS();
            PH.setPack(p25);
            p25.satellite_prn_SET(new byte[] {(byte)173, (byte)231, (byte)173, (byte)97, (byte)137, (byte)255, (byte)121, (byte)217, (byte)235, (byte)140, (byte)135, (byte)22, (byte)206, (byte)101, (byte)95, (byte)142, (byte)208, (byte)239, (byte)78, (byte)176}, 0) ;
            p25.satellite_snr_SET(new byte[] {(byte)161, (byte)145, (byte)37, (byte)54, (byte)212, (byte)200, (byte)164, (byte)151, (byte)201, (byte)158, (byte)191, (byte)113, (byte)185, (byte)37, (byte)120, (byte)138, (byte)81, (byte)144, (byte)140, (byte)177}, 0) ;
            p25.satellite_used_SET(new byte[] {(byte)152, (byte)31, (byte)71, (byte)146, (byte)198, (byte)199, (byte)183, (byte)20, (byte)201, (byte)106, (byte)3, (byte)106, (byte)211, (byte)57, (byte)52, (byte)199, (byte)161, (byte)123, (byte)95, (byte)30}, 0) ;
            p25.satellite_elevation_SET(new byte[] {(byte)240, (byte)82, (byte)92, (byte)67, (byte)241, (byte)122, (byte)119, (byte)116, (byte)254, (byte)74, (byte)212, (byte)250, (byte)233, (byte)68, (byte)186, (byte)222, (byte)103, (byte)170, (byte)241, (byte)246}, 0) ;
            p25.satellites_visible = (byte)(byte)9;
            p25.satellite_azimuth_SET(new byte[] {(byte)154, (byte)98, (byte)175, (byte)40, (byte)205, (byte)187, (byte)13, (byte)220, (byte)111, (byte)137, (byte)181, (byte)92, (byte)123, (byte)16, (byte)218, (byte)53, (byte)183, (byte)122, (byte)13, (byte)249}, 0) ;
            ADV_TEST_CH.send(p25);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_IMUReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.zacc == (short)(short) -28359);
                Debug.Assert(pack.zmag == (short)(short) -854);
                Debug.Assert(pack.xmag == (short)(short)27157);
                Debug.Assert(pack.zgyro == (short)(short)14657);
                Debug.Assert(pack.xgyro == (short)(short)26165);
                Debug.Assert(pack.ygyro == (short)(short) -991);
                Debug.Assert(pack.yacc == (short)(short) -11142);
                Debug.Assert(pack.xacc == (short)(short) -25894);
                Debug.Assert(pack.time_boot_ms == (uint)512835140U);
                Debug.Assert(pack.ymag == (short)(short) -32755);
            };
            SCALED_IMU p26 = new SCALED_IMU();
            PH.setPack(p26);
            p26.time_boot_ms = (uint)512835140U;
            p26.yacc = (short)(short) -11142;
            p26.xacc = (short)(short) -25894;
            p26.xgyro = (short)(short)26165;
            p26.ymag = (short)(short) -32755;
            p26.zgyro = (short)(short)14657;
            p26.zacc = (short)(short) -28359;
            p26.zmag = (short)(short) -854;
            p26.ygyro = (short)(short) -991;
            p26.xmag = (short)(short)27157;
            ADV_TEST_CH.send(p26);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRAW_IMUReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.xacc == (short)(short)18858);
                Debug.Assert(pack.zacc == (short)(short) -32066);
                Debug.Assert(pack.ygyro == (short)(short)15403);
                Debug.Assert(pack.xgyro == (short)(short)18277);
                Debug.Assert(pack.zgyro == (short)(short)30616);
                Debug.Assert(pack.yacc == (short)(short)27401);
                Debug.Assert(pack.ymag == (short)(short) -658);
                Debug.Assert(pack.xmag == (short)(short)3938);
                Debug.Assert(pack.time_usec == (ulong)4810595649482807406L);
                Debug.Assert(pack.zmag == (short)(short) -21777);
            };
            RAW_IMU p27 = new RAW_IMU();
            PH.setPack(p27);
            p27.zmag = (short)(short) -21777;
            p27.xgyro = (short)(short)18277;
            p27.zacc = (short)(short) -32066;
            p27.xmag = (short)(short)3938;
            p27.zgyro = (short)(short)30616;
            p27.xacc = (short)(short)18858;
            p27.ymag = (short)(short) -658;
            p27.yacc = (short)(short)27401;
            p27.time_usec = (ulong)4810595649482807406L;
            p27.ygyro = (short)(short)15403;
            ADV_TEST_CH.send(p27);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRAW_PRESSUREReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)7001858096199486339L);
                Debug.Assert(pack.press_diff2 == (short)(short)27358);
                Debug.Assert(pack.temperature == (short)(short) -11239);
                Debug.Assert(pack.press_diff1 == (short)(short) -12329);
                Debug.Assert(pack.press_abs == (short)(short) -818);
            };
            RAW_PRESSURE p28 = new RAW_PRESSURE();
            PH.setPack(p28);
            p28.temperature = (short)(short) -11239;
            p28.press_diff2 = (short)(short)27358;
            p28.time_usec = (ulong)7001858096199486339L;
            p28.press_diff1 = (short)(short) -12329;
            p28.press_abs = (short)(short) -818;
            ADV_TEST_CH.send(p28);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_PRESSUREReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.press_abs == (float) -2.4185137E38F);
                Debug.Assert(pack.temperature == (short)(short)3949);
                Debug.Assert(pack.time_boot_ms == (uint)3014922919U);
                Debug.Assert(pack.press_diff == (float)1.2993077E38F);
            };
            SCALED_PRESSURE p29 = new SCALED_PRESSURE();
            PH.setPack(p29);
            p29.press_abs = (float) -2.4185137E38F;
            p29.time_boot_ms = (uint)3014922919U;
            p29.temperature = (short)(short)3949;
            p29.press_diff = (float)1.2993077E38F;
            ADV_TEST_CH.send(p29);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATTITUDEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.yaw == (float) -1.4957065E38F);
                Debug.Assert(pack.yawspeed == (float)2.2743244E38F);
                Debug.Assert(pack.pitch == (float) -3.6244093E37F);
                Debug.Assert(pack.rollspeed == (float) -1.8901938E38F);
                Debug.Assert(pack.time_boot_ms == (uint)3123994604U);
                Debug.Assert(pack.roll == (float)1.9932515E37F);
                Debug.Assert(pack.pitchspeed == (float)2.060617E38F);
            };
            ATTITUDE p30 = new ATTITUDE();
            PH.setPack(p30);
            p30.yaw = (float) -1.4957065E38F;
            p30.roll = (float)1.9932515E37F;
            p30.pitch = (float) -3.6244093E37F;
            p30.yawspeed = (float)2.2743244E38F;
            p30.pitchspeed = (float)2.060617E38F;
            p30.rollspeed = (float) -1.8901938E38F;
            p30.time_boot_ms = (uint)3123994604U;
            ADV_TEST_CH.send(p30);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATTITUDE_QUATERNIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.q1 == (float)2.0902033E38F);
                Debug.Assert(pack.q4 == (float)5.8902567E37F);
                Debug.Assert(pack.yawspeed == (float)2.5786325E38F);
                Debug.Assert(pack.q2 == (float) -3.2018344E37F);
                Debug.Assert(pack.rollspeed == (float) -1.3741665E38F);
                Debug.Assert(pack.pitchspeed == (float) -5.58155E36F);
                Debug.Assert(pack.q3 == (float)1.4736974E38F);
                Debug.Assert(pack.time_boot_ms == (uint)1463676404U);
            };
            ATTITUDE_QUATERNION p31 = new ATTITUDE_QUATERNION();
            PH.setPack(p31);
            p31.q3 = (float)1.4736974E38F;
            p31.yawspeed = (float)2.5786325E38F;
            p31.time_boot_ms = (uint)1463676404U;
            p31.rollspeed = (float) -1.3741665E38F;
            p31.q4 = (float)5.8902567E37F;
            p31.q2 = (float) -3.2018344E37F;
            p31.q1 = (float)2.0902033E38F;
            p31.pitchspeed = (float) -5.58155E36F;
            ADV_TEST_CH.send(p31);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOCAL_POSITION_NEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vy == (float)9.108088E36F);
                Debug.Assert(pack.z == (float)2.8005045E38F);
                Debug.Assert(pack.y == (float) -3.9758238E37F);
                Debug.Assert(pack.vx == (float) -2.16171E38F);
                Debug.Assert(pack.time_boot_ms == (uint)3328701940U);
                Debug.Assert(pack.x == (float)2.9506207E38F);
                Debug.Assert(pack.vz == (float)1.6342397E37F);
            };
            LOCAL_POSITION_NED p32 = new LOCAL_POSITION_NED();
            PH.setPack(p32);
            p32.time_boot_ms = (uint)3328701940U;
            p32.vy = (float)9.108088E36F;
            p32.vx = (float) -2.16171E38F;
            p32.y = (float) -3.9758238E37F;
            p32.x = (float)2.9506207E38F;
            p32.z = (float)2.8005045E38F;
            p32.vz = (float)1.6342397E37F;
            ADV_TEST_CH.send(p32);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGLOBAL_POSITION_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lon == (int) -447758621);
                Debug.Assert(pack.alt == (int) -144204296);
                Debug.Assert(pack.vx == (short)(short) -27784);
                Debug.Assert(pack.vz == (short)(short) -12732);
                Debug.Assert(pack.lat == (int)1933782833);
                Debug.Assert(pack.hdg == (ushort)(ushort)34215);
                Debug.Assert(pack.vy == (short)(short) -4646);
                Debug.Assert(pack.relative_alt == (int) -1699356472);
                Debug.Assert(pack.time_boot_ms == (uint)841840372U);
            };
            GLOBAL_POSITION_INT p33 = new GLOBAL_POSITION_INT();
            PH.setPack(p33);
            p33.hdg = (ushort)(ushort)34215;
            p33.alt = (int) -144204296;
            p33.vy = (short)(short) -4646;
            p33.vz = (short)(short) -12732;
            p33.lon = (int) -447758621;
            p33.time_boot_ms = (uint)841840372U;
            p33.vx = (short)(short) -27784;
            p33.lat = (int)1933782833;
            p33.relative_alt = (int) -1699356472;
            ADV_TEST_CH.send(p33);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRC_CHANNELS_SCALEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)493132164U);
                Debug.Assert(pack.chan4_scaled == (short)(short) -17292);
                Debug.Assert(pack.chan2_scaled == (short)(short) -24546);
                Debug.Assert(pack.chan5_scaled == (short)(short)2900);
                Debug.Assert(pack.chan6_scaled == (short)(short) -31870);
                Debug.Assert(pack.chan1_scaled == (short)(short) -23226);
                Debug.Assert(pack.port == (byte)(byte)140);
                Debug.Assert(pack.rssi == (byte)(byte)150);
                Debug.Assert(pack.chan7_scaled == (short)(short) -12108);
                Debug.Assert(pack.chan3_scaled == (short)(short)22882);
                Debug.Assert(pack.chan8_scaled == (short)(short)18934);
            };
            RC_CHANNELS_SCALED p34 = new RC_CHANNELS_SCALED();
            PH.setPack(p34);
            p34.chan4_scaled = (short)(short) -17292;
            p34.port = (byte)(byte)140;
            p34.rssi = (byte)(byte)150;
            p34.chan2_scaled = (short)(short) -24546;
            p34.time_boot_ms = (uint)493132164U;
            p34.chan5_scaled = (short)(short)2900;
            p34.chan6_scaled = (short)(short) -31870;
            p34.chan7_scaled = (short)(short) -12108;
            p34.chan3_scaled = (short)(short)22882;
            p34.chan1_scaled = (short)(short) -23226;
            p34.chan8_scaled = (short)(short)18934;
            ADV_TEST_CH.send(p34);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRC_CHANNELS_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)43902);
                Debug.Assert(pack.time_boot_ms == (uint)2157464065U);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)29389);
                Debug.Assert(pack.port == (byte)(byte)66);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)3268);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)19942);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)26869);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)40899);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)32688);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)45505);
                Debug.Assert(pack.rssi == (byte)(byte)200);
            };
            RC_CHANNELS_RAW p35 = new RC_CHANNELS_RAW();
            PH.setPack(p35);
            p35.chan7_raw = (ushort)(ushort)26869;
            p35.chan5_raw = (ushort)(ushort)40899;
            p35.chan1_raw = (ushort)(ushort)43902;
            p35.chan8_raw = (ushort)(ushort)19942;
            p35.chan6_raw = (ushort)(ushort)32688;
            p35.chan3_raw = (ushort)(ushort)29389;
            p35.port = (byte)(byte)66;
            p35.chan4_raw = (ushort)(ushort)3268;
            p35.chan2_raw = (ushort)(ushort)45505;
            p35.rssi = (byte)(byte)200;
            p35.time_boot_ms = (uint)2157464065U;
            ADV_TEST_CH.send(p35);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSERVO_OUTPUT_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.servo5_raw == (ushort)(ushort)63199);
                Debug.Assert(pack.servo16_raw_TRY(ph) == (ushort)(ushort)16245);
                Debug.Assert(pack.servo10_raw_TRY(ph) == (ushort)(ushort)27754);
                Debug.Assert(pack.servo15_raw_TRY(ph) == (ushort)(ushort)34377);
                Debug.Assert(pack.servo3_raw == (ushort)(ushort)51253);
                Debug.Assert(pack.port == (byte)(byte)28);
                Debug.Assert(pack.servo6_raw == (ushort)(ushort)44043);
                Debug.Assert(pack.servo4_raw == (ushort)(ushort)49129);
                Debug.Assert(pack.servo12_raw_TRY(ph) == (ushort)(ushort)64038);
                Debug.Assert(pack.servo13_raw_TRY(ph) == (ushort)(ushort)50237);
                Debug.Assert(pack.time_usec == (uint)30805629U);
                Debug.Assert(pack.servo2_raw == (ushort)(ushort)14476);
                Debug.Assert(pack.servo11_raw_TRY(ph) == (ushort)(ushort)32766);
                Debug.Assert(pack.servo8_raw == (ushort)(ushort)65193);
                Debug.Assert(pack.servo7_raw == (ushort)(ushort)35441);
                Debug.Assert(pack.servo9_raw_TRY(ph) == (ushort)(ushort)40159);
                Debug.Assert(pack.servo1_raw == (ushort)(ushort)48001);
                Debug.Assert(pack.servo14_raw_TRY(ph) == (ushort)(ushort)53971);
            };
            SERVO_OUTPUT_RAW p36 = new SERVO_OUTPUT_RAW();
            PH.setPack(p36);
            p36.servo4_raw = (ushort)(ushort)49129;
            p36.servo16_raw_SET((ushort)(ushort)16245, PH) ;
            p36.servo5_raw = (ushort)(ushort)63199;
            p36.servo9_raw_SET((ushort)(ushort)40159, PH) ;
            p36.port = (byte)(byte)28;
            p36.servo12_raw_SET((ushort)(ushort)64038, PH) ;
            p36.servo10_raw_SET((ushort)(ushort)27754, PH) ;
            p36.servo13_raw_SET((ushort)(ushort)50237, PH) ;
            p36.servo2_raw = (ushort)(ushort)14476;
            p36.servo1_raw = (ushort)(ushort)48001;
            p36.servo15_raw_SET((ushort)(ushort)34377, PH) ;
            p36.servo7_raw = (ushort)(ushort)35441;
            p36.servo11_raw_SET((ushort)(ushort)32766, PH) ;
            p36.time_usec = (uint)30805629U;
            p36.servo3_raw = (ushort)(ushort)51253;
            p36.servo8_raw = (ushort)(ushort)65193;
            p36.servo6_raw = (ushort)(ushort)44043;
            p36.servo14_raw_SET((ushort)(ushort)53971, PH) ;
            ADV_TEST_CH.send(p36);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_REQUEST_PARTIAL_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
                Debug.Assert(pack.target_component == (byte)(byte)220);
                Debug.Assert(pack.end_index == (short)(short) -28103);
                Debug.Assert(pack.start_index == (short)(short) -7839);
                Debug.Assert(pack.target_system == (byte)(byte)197);
            };
            MISSION_REQUEST_PARTIAL_LIST p37 = new MISSION_REQUEST_PARTIAL_LIST();
            PH.setPack(p37);
            p37.target_system = (byte)(byte)197;
            p37.start_index = (short)(short) -7839;
            p37.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION;
            p37.target_component = (byte)(byte)220;
            p37.end_index = (short)(short) -28103;
            ADV_TEST_CH.send(p37);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_WRITE_PARTIAL_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)53);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
                Debug.Assert(pack.end_index == (short)(short) -513);
                Debug.Assert(pack.start_index == (short)(short)22047);
                Debug.Assert(pack.target_component == (byte)(byte)228);
            };
            MISSION_WRITE_PARTIAL_LIST p38 = new MISSION_WRITE_PARTIAL_LIST();
            PH.setPack(p38);
            p38.end_index = (short)(short) -513;
            p38.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL;
            p38.start_index = (short)(short)22047;
            p38.target_component = (byte)(byte)228;
            p38.target_system = (byte)(byte)53;
            ADV_TEST_CH.send(p38);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_ITEMReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.z == (float) -2.2127947E38F);
                Debug.Assert(pack.current == (byte)(byte)226);
                Debug.Assert(pack.target_system == (byte)(byte)128);
                Debug.Assert(pack.param2 == (float) -5.4354E37F);
                Debug.Assert(pack.param1 == (float)2.223921E38F);
                Debug.Assert(pack.x == (float) -2.073387E38F);
                Debug.Assert(pack.seq == (ushort)(ushort)24551);
                Debug.Assert(pack.target_component == (byte)(byte)8);
                Debug.Assert(pack.param4 == (float)1.034017E38F);
                Debug.Assert(pack.y == (float) -1.3219214E38F);
                Debug.Assert(pack.param3 == (float)1.9047854E38F);
                Debug.Assert(pack.frame == MAV_FRAME.MAV_FRAME_LOCAL_ENU);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
                Debug.Assert(pack.autocontinue == (byte)(byte)216);
                Debug.Assert(pack.command == MAV_CMD.MAV_CMD_PAYLOAD_PREPARE_DEPLOY);
            };
            MISSION_ITEM p39 = new MISSION_ITEM();
            PH.setPack(p39);
            p39.param4 = (float)1.034017E38F;
            p39.current = (byte)(byte)226;
            p39.target_system = (byte)(byte)128;
            p39.y = (float) -1.3219214E38F;
            p39.target_component = (byte)(byte)8;
            p39.param2 = (float) -5.4354E37F;
            p39.seq = (ushort)(ushort)24551;
            p39.param3 = (float)1.9047854E38F;
            p39.z = (float) -2.2127947E38F;
            p39.command = MAV_CMD.MAV_CMD_PAYLOAD_PREPARE_DEPLOY;
            p39.frame = MAV_FRAME.MAV_FRAME_LOCAL_ENU;
            p39.param1 = (float)2.223921E38F;
            p39.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL;
            p39.autocontinue = (byte)(byte)216;
            p39.x = (float) -2.073387E38F;
            ADV_TEST_CH.send(p39);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)226);
                Debug.Assert(pack.target_system == (byte)(byte)238);
                Debug.Assert(pack.seq == (ushort)(ushort)33259);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
            };
            MISSION_REQUEST p40 = new MISSION_REQUEST();
            PH.setPack(p40);
            p40.target_system = (byte)(byte)238;
            p40.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY;
            p40.seq = (ushort)(ushort)33259;
            p40.target_component = (byte)(byte)226;
            ADV_TEST_CH.send(p40);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_SET_CURRENTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)173);
                Debug.Assert(pack.seq == (ushort)(ushort)2980);
                Debug.Assert(pack.target_component == (byte)(byte)40);
            };
            MISSION_SET_CURRENT p41 = new MISSION_SET_CURRENT();
            PH.setPack(p41);
            p41.target_component = (byte)(byte)40;
            p41.seq = (ushort)(ushort)2980;
            p41.target_system = (byte)(byte)173;
            ADV_TEST_CH.send(p41);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_CURRENTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (ushort)(ushort)62848);
            };
            MISSION_CURRENT p42 = new MISSION_CURRENT();
            PH.setPack(p42);
            p42.seq = (ushort)(ushort)62848;
            ADV_TEST_CH.send(p42);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)187);
                Debug.Assert(pack.target_component == (byte)(byte)63);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
            };
            MISSION_REQUEST_LIST p43 = new MISSION_REQUEST_LIST();
            PH.setPack(p43);
            p43.target_system = (byte)(byte)187;
            p43.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION;
            p43.target_component = (byte)(byte)63;
            ADV_TEST_CH.send(p43);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_COUNTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.count == (ushort)(ushort)21504);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
                Debug.Assert(pack.target_component == (byte)(byte)133);
                Debug.Assert(pack.target_system == (byte)(byte)232);
            };
            MISSION_COUNT p44 = new MISSION_COUNT();
            PH.setPack(p44);
            p44.count = (ushort)(ushort)21504;
            p44.target_component = (byte)(byte)133;
            p44.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY;
            p44.target_system = (byte)(byte)232;
            ADV_TEST_CH.send(p44);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_CLEAR_ALLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
                Debug.Assert(pack.target_system == (byte)(byte)132);
                Debug.Assert(pack.target_component == (byte)(byte)28);
            };
            MISSION_CLEAR_ALL p45 = new MISSION_CLEAR_ALL();
            PH.setPack(p45);
            p45.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE;
            p45.target_component = (byte)(byte)28;
            p45.target_system = (byte)(byte)132;
            ADV_TEST_CH.send(p45);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_ITEM_REACHEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (ushort)(ushort)36503);
            };
            MISSION_ITEM_REACHED p46 = new MISSION_ITEM_REACHED();
            PH.setPack(p46);
            p46.seq = (ushort)(ushort)36503;
            ADV_TEST_CH.send(p46);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.type == MAV_MISSION_RESULT.MAV_MISSION_DENIED);
                Debug.Assert(pack.target_system == (byte)(byte)150);
                Debug.Assert(pack.target_component == (byte)(byte)20);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
            };
            MISSION_ACK p47 = new MISSION_ACK();
            PH.setPack(p47);
            p47.target_system = (byte)(byte)150;
            p47.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL;
            p47.type = MAV_MISSION_RESULT.MAV_MISSION_DENIED;
            p47.target_component = (byte)(byte)20;
            ADV_TEST_CH.send(p47);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_GPS_GLOBAL_ORIGINReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)209);
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)2145614245058885535L);
                Debug.Assert(pack.longitude == (int)617454683);
                Debug.Assert(pack.altitude == (int) -1152485807);
                Debug.Assert(pack.latitude == (int) -1874693102);
            };
            SET_GPS_GLOBAL_ORIGIN p48 = new SET_GPS_GLOBAL_ORIGIN();
            PH.setPack(p48);
            p48.latitude = (int) -1874693102;
            p48.time_usec_SET((ulong)2145614245058885535L, PH) ;
            p48.longitude = (int)617454683;
            p48.altitude = (int) -1152485807;
            p48.target_system = (byte)(byte)209;
            ADV_TEST_CH.send(p48);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_GLOBAL_ORIGINReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)996956407534691805L);
                Debug.Assert(pack.latitude == (int)919796980);
                Debug.Assert(pack.altitude == (int)413263716);
                Debug.Assert(pack.longitude == (int)577931299);
            };
            GPS_GLOBAL_ORIGIN p49 = new GPS_GLOBAL_ORIGIN();
            PH.setPack(p49);
            p49.altitude = (int)413263716;
            p49.time_usec_SET((ulong)996956407534691805L, PH) ;
            p49.longitude = (int)577931299;
            p49.latitude = (int)919796980;
            ADV_TEST_CH.send(p49);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_MAP_RCReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_value_min == (float) -1.149063E38F);
                Debug.Assert(pack.scale == (float)2.5929438E38F);
                Debug.Assert(pack.param_value0 == (float)2.3284168E38F);
                Debug.Assert(pack.parameter_rc_channel_index == (byte)(byte)96);
                Debug.Assert(pack.param_value_max == (float)2.3083745E38F);
                Debug.Assert(pack.target_component == (byte)(byte)189);
                Debug.Assert(pack.param_index == (short)(short) -19863);
                Debug.Assert(pack.param_id_LEN(ph) == 5);
                Debug.Assert(pack.param_id_TRY(ph).Equals("NBinQ"));
                Debug.Assert(pack.target_system == (byte)(byte)26);
            };
            PARAM_MAP_RC p50 = new PARAM_MAP_RC();
            PH.setPack(p50);
            p50.param_value_max = (float)2.3083745E38F;
            p50.target_component = (byte)(byte)189;
            p50.parameter_rc_channel_index = (byte)(byte)96;
            p50.scale = (float)2.5929438E38F;
            p50.param_value0 = (float)2.3284168E38F;
            p50.param_value_min = (float) -1.149063E38F;
            p50.target_system = (byte)(byte)26;
            p50.param_index = (short)(short) -19863;
            p50.param_id_SET("NBinQ", PH) ;
            ADV_TEST_CH.send(p50);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_REQUEST_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)144);
                Debug.Assert(pack.target_system == (byte)(byte)200);
                Debug.Assert(pack.seq == (ushort)(ushort)4086);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
            };
            MISSION_REQUEST_INT p51 = new MISSION_REQUEST_INT();
            PH.setPack(p51);
            p51.seq = (ushort)(ushort)4086;
            p51.target_system = (byte)(byte)200;
            p51.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY;
            p51.target_component = (byte)(byte)144;
            ADV_TEST_CH.send(p51);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSAFETY_SET_ALLOWED_AREAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)109);
                Debug.Assert(pack.p1z == (float) -1.922809E37F);
                Debug.Assert(pack.target_component == (byte)(byte)244);
                Debug.Assert(pack.p2z == (float)1.5238746E38F);
                Debug.Assert(pack.p1y == (float) -6.824728E37F);
                Debug.Assert(pack.p1x == (float) -1.374542E38F);
                Debug.Assert(pack.p2x == (float) -3.1559304E38F);
                Debug.Assert(pack.p2y == (float)4.5440883E37F);
                Debug.Assert(pack.frame == MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED);
            };
            SAFETY_SET_ALLOWED_AREA p54 = new SAFETY_SET_ALLOWED_AREA();
            PH.setPack(p54);
            p54.p2x = (float) -3.1559304E38F;
            p54.frame = MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED;
            p54.p1x = (float) -1.374542E38F;
            p54.p2z = (float)1.5238746E38F;
            p54.target_system = (byte)(byte)109;
            p54.target_component = (byte)(byte)244;
            p54.p1y = (float) -6.824728E37F;
            p54.p2y = (float)4.5440883E37F;
            p54.p1z = (float) -1.922809E37F;
            ADV_TEST_CH.send(p54);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSAFETY_ALLOWED_AREAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.frame == MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT);
                Debug.Assert(pack.p1x == (float)3.2482336E38F);
                Debug.Assert(pack.p2x == (float)6.0051165E37F);
                Debug.Assert(pack.p2z == (float)3.0918637E38F);
                Debug.Assert(pack.p1z == (float)4.624144E37F);
                Debug.Assert(pack.p1y == (float)2.9037816E37F);
                Debug.Assert(pack.p2y == (float) -3.977009E37F);
            };
            SAFETY_ALLOWED_AREA p55 = new SAFETY_ALLOWED_AREA();
            PH.setPack(p55);
            p55.p1x = (float)3.2482336E38F;
            p55.p2x = (float)6.0051165E37F;
            p55.p2y = (float) -3.977009E37F;
            p55.p1y = (float)2.9037816E37F;
            p55.p2z = (float)3.0918637E38F;
            p55.frame = MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT;
            p55.p1z = (float)4.624144E37F;
            ADV_TEST_CH.send(p55);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATTITUDE_QUATERNION_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.pitchspeed == (float)2.830115E37F);
                Debug.Assert(pack.yawspeed == (float)7.285341E37F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {1.727176E38F, 3.306612E38F, -6.9514876E37F, 2.6816487E38F}));
                Debug.Assert(pack.covariance.SequenceEqual(new float[] {-1.3646451E38F, -4.9950482E36F, -2.101692E38F, 2.2474697E37F, 2.860766E38F, 2.303173E37F, 3.9406937E37F, 1.8994014E38F, 3.6968994E37F}));
                Debug.Assert(pack.rollspeed == (float)1.6759526E38F);
                Debug.Assert(pack.time_usec == (ulong)6033099274471234904L);
            };
            ATTITUDE_QUATERNION_COV p61 = new ATTITUDE_QUATERNION_COV();
            PH.setPack(p61);
            p61.covariance_SET(new float[] {-1.3646451E38F, -4.9950482E36F, -2.101692E38F, 2.2474697E37F, 2.860766E38F, 2.303173E37F, 3.9406937E37F, 1.8994014E38F, 3.6968994E37F}, 0) ;
            p61.pitchspeed = (float)2.830115E37F;
            p61.rollspeed = (float)1.6759526E38F;
            p61.time_usec = (ulong)6033099274471234904L;
            p61.yawspeed = (float)7.285341E37F;
            p61.q_SET(new float[] {1.727176E38F, 3.306612E38F, -6.9514876E37F, 2.6816487E38F}, 0) ;
            ADV_TEST_CH.send(p61);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnNAV_CONTROLLER_OUTPUTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.alt_error == (float)1.1238425E38F);
                Debug.Assert(pack.nav_pitch == (float)8.234069E37F);
                Debug.Assert(pack.target_bearing == (short)(short) -11171);
                Debug.Assert(pack.wp_dist == (ushort)(ushort)46188);
                Debug.Assert(pack.xtrack_error == (float)6.5514003E37F);
                Debug.Assert(pack.nav_bearing == (short)(short)10985);
                Debug.Assert(pack.aspd_error == (float) -4.43365E37F);
                Debug.Assert(pack.nav_roll == (float) -2.5748164E38F);
            };
            NAV_CONTROLLER_OUTPUT p62 = new NAV_CONTROLLER_OUTPUT();
            PH.setPack(p62);
            p62.aspd_error = (float) -4.43365E37F;
            p62.wp_dist = (ushort)(ushort)46188;
            p62.nav_bearing = (short)(short)10985;
            p62.nav_roll = (float) -2.5748164E38F;
            p62.target_bearing = (short)(short) -11171;
            p62.alt_error = (float)1.1238425E38F;
            p62.xtrack_error = (float)6.5514003E37F;
            p62.nav_pitch = (float)8.234069E37F;
            ADV_TEST_CH.send(p62);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGLOBAL_POSITION_INT_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.covariance.SequenceEqual(new float[] {1.480038E38F, 2.5247195E38F, -2.6789735E38F, -7.653535E37F, 2.176799E38F, 3.3140843E38F, 6.2477656E37F, 3.0059217E38F, -1.3149654E38F, -3.0100536E38F, -2.8482977E38F, 2.553993E38F, -2.9521362E38F, 2.5254214E38F, -8.336371E37F, -2.7861626E38F, -6.8834787E37F, -1.5329555E38F, -1.2080547E37F, -3.0151447E38F, 1.656632E38F, 1.06286E38F, 2.9036977E38F, 2.6295884E38F, 1.7935948E38F, 3.0473759E38F, -8.636453E37F, 1.92384E38F, -1.8262338E38F, 2.912163E38F, -2.9827383E38F, 2.4320142E38F, -1.6784028E38F, -2.390079E38F, -7.9599574E37F, -1.7118822E38F}));
                Debug.Assert(pack.relative_alt == (int) -1495512612);
                Debug.Assert(pack.estimator_type == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS);
                Debug.Assert(pack.vy == (float) -9.63622E37F);
                Debug.Assert(pack.lon == (int) -197666187);
                Debug.Assert(pack.vx == (float) -5.1627898E36F);
                Debug.Assert(pack.time_usec == (ulong)3690209458134360365L);
                Debug.Assert(pack.alt == (int)449284841);
                Debug.Assert(pack.lat == (int) -2074418511);
                Debug.Assert(pack.vz == (float)1.6910831E38F);
            };
            GLOBAL_POSITION_INT_COV p63 = new GLOBAL_POSITION_INT_COV();
            PH.setPack(p63);
            p63.vz = (float)1.6910831E38F;
            p63.vx = (float) -5.1627898E36F;
            p63.time_usec = (ulong)3690209458134360365L;
            p63.vy = (float) -9.63622E37F;
            p63.estimator_type = MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS;
            p63.covariance_SET(new float[] {1.480038E38F, 2.5247195E38F, -2.6789735E38F, -7.653535E37F, 2.176799E38F, 3.3140843E38F, 6.2477656E37F, 3.0059217E38F, -1.3149654E38F, -3.0100536E38F, -2.8482977E38F, 2.553993E38F, -2.9521362E38F, 2.5254214E38F, -8.336371E37F, -2.7861626E38F, -6.8834787E37F, -1.5329555E38F, -1.2080547E37F, -3.0151447E38F, 1.656632E38F, 1.06286E38F, 2.9036977E38F, 2.6295884E38F, 1.7935948E38F, 3.0473759E38F, -8.636453E37F, 1.92384E38F, -1.8262338E38F, 2.912163E38F, -2.9827383E38F, 2.4320142E38F, -1.6784028E38F, -2.390079E38F, -7.9599574E37F, -1.7118822E38F}, 0) ;
            p63.relative_alt = (int) -1495512612;
            p63.lat = (int) -2074418511;
            p63.lon = (int) -197666187;
            p63.alt = (int)449284841;
            ADV_TEST_CH.send(p63);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOCAL_POSITION_NED_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.x == (float)1.8417272E38F);
                Debug.Assert(pack.ax == (float)3.7573813E37F);
                Debug.Assert(pack.estimator_type == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS_INS);
                Debug.Assert(pack.az == (float) -2.9345635E38F);
                Debug.Assert(pack.vx == (float)1.6733215E38F);
                Debug.Assert(pack.y == (float) -7.6822243E37F);
                Debug.Assert(pack.time_usec == (ulong)8002556452925125630L);
                Debug.Assert(pack.covariance.SequenceEqual(new float[] {-2.8912743E38F, 9.638625E37F, -1.558827E38F, 1.7584723E38F, -2.1884172E38F, -1.9359154E38F, -2.1616584E38F, -2.3457246E38F, -2.8532247E38F, 1.3435548E38F, -2.8263284E36F, -2.6702545E38F, 1.277415E38F, -1.6840911E38F, 2.6017308E38F, 1.9599531E38F, 8.531786E37F, 5.9048433E37F, 2.862075E38F, 1.2115274E38F, 5.845634E37F, -1.942906E38F, 2.8065898E38F, 2.642398E38F, 1.1110024E38F, -2.5455162E38F, 9.581582E37F, -1.2470096E38F, 1.0658038E38F, -1.2796125E38F, 2.8832843E38F, -2.5985586E38F, 3.1145826E38F, -3.1916033E38F, -1.8980502E38F, 3.331073E38F, 2.5566359E38F, -5.736256E37F, -2.667214E38F, 1.431278E38F, 7.7946233E37F, 3.1492104E38F, 9.208075E37F, 2.0814995E38F, 3.0297482E38F}));
                Debug.Assert(pack.vy == (float)8.2746897E37F);
                Debug.Assert(pack.ay == (float)2.6153997E38F);
                Debug.Assert(pack.z == (float) -1.8033482E38F);
                Debug.Assert(pack.vz == (float)1.2573624E38F);
            };
            LOCAL_POSITION_NED_COV p64 = new LOCAL_POSITION_NED_COV();
            PH.setPack(p64);
            p64.vx = (float)1.6733215E38F;
            p64.covariance_SET(new float[] {-2.8912743E38F, 9.638625E37F, -1.558827E38F, 1.7584723E38F, -2.1884172E38F, -1.9359154E38F, -2.1616584E38F, -2.3457246E38F, -2.8532247E38F, 1.3435548E38F, -2.8263284E36F, -2.6702545E38F, 1.277415E38F, -1.6840911E38F, 2.6017308E38F, 1.9599531E38F, 8.531786E37F, 5.9048433E37F, 2.862075E38F, 1.2115274E38F, 5.845634E37F, -1.942906E38F, 2.8065898E38F, 2.642398E38F, 1.1110024E38F, -2.5455162E38F, 9.581582E37F, -1.2470096E38F, 1.0658038E38F, -1.2796125E38F, 2.8832843E38F, -2.5985586E38F, 3.1145826E38F, -3.1916033E38F, -1.8980502E38F, 3.331073E38F, 2.5566359E38F, -5.736256E37F, -2.667214E38F, 1.431278E38F, 7.7946233E37F, 3.1492104E38F, 9.208075E37F, 2.0814995E38F, 3.0297482E38F}, 0) ;
            p64.ax = (float)3.7573813E37F;
            p64.ay = (float)2.6153997E38F;
            p64.estimator_type = MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS_INS;
            p64.az = (float) -2.9345635E38F;
            p64.time_usec = (ulong)8002556452925125630L;
            p64.vz = (float)1.2573624E38F;
            p64.y = (float) -7.6822243E37F;
            p64.vy = (float)8.2746897E37F;
            p64.z = (float) -1.8033482E38F;
            p64.x = (float)1.8417272E38F;
            ADV_TEST_CH.send(p64);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRC_CHANNELSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan17_raw == (ushort)(ushort)58074);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)14125);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)16516);
                Debug.Assert(pack.chan18_raw == (ushort)(ushort)26889);
                Debug.Assert(pack.chan16_raw == (ushort)(ushort)56503);
                Debug.Assert(pack.chan11_raw == (ushort)(ushort)61258);
                Debug.Assert(pack.time_boot_ms == (uint)992111019U);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)14969);
                Debug.Assert(pack.chan15_raw == (ushort)(ushort)37865);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)42444);
                Debug.Assert(pack.chancount == (byte)(byte)127);
                Debug.Assert(pack.chan13_raw == (ushort)(ushort)30456);
                Debug.Assert(pack.rssi == (byte)(byte)41);
                Debug.Assert(pack.chan14_raw == (ushort)(ushort)20363);
                Debug.Assert(pack.chan12_raw == (ushort)(ushort)57486);
                Debug.Assert(pack.chan10_raw == (ushort)(ushort)14757);
                Debug.Assert(pack.chan9_raw == (ushort)(ushort)56654);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)5427);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)52426);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)43224);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)36776);
            };
            RC_CHANNELS p65 = new RC_CHANNELS();
            PH.setPack(p65);
            p65.chancount = (byte)(byte)127;
            p65.chan18_raw = (ushort)(ushort)26889;
            p65.chan3_raw = (ushort)(ushort)5427;
            p65.chan15_raw = (ushort)(ushort)37865;
            p65.chan7_raw = (ushort)(ushort)42444;
            p65.chan12_raw = (ushort)(ushort)57486;
            p65.chan13_raw = (ushort)(ushort)30456;
            p65.chan16_raw = (ushort)(ushort)56503;
            p65.chan17_raw = (ushort)(ushort)58074;
            p65.chan2_raw = (ushort)(ushort)43224;
            p65.chan10_raw = (ushort)(ushort)14757;
            p65.chan11_raw = (ushort)(ushort)61258;
            p65.chan5_raw = (ushort)(ushort)14969;
            p65.chan1_raw = (ushort)(ushort)16516;
            p65.time_boot_ms = (uint)992111019U;
            p65.chan14_raw = (ushort)(ushort)20363;
            p65.rssi = (byte)(byte)41;
            p65.chan6_raw = (ushort)(ushort)52426;
            p65.chan9_raw = (ushort)(ushort)56654;
            p65.chan8_raw = (ushort)(ushort)14125;
            p65.chan4_raw = (ushort)(ushort)36776;
            ADV_TEST_CH.send(p65);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnREQUEST_DATA_STREAMReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.req_message_rate == (ushort)(ushort)32476);
                Debug.Assert(pack.target_component == (byte)(byte)220);
                Debug.Assert(pack.req_stream_id == (byte)(byte)26);
                Debug.Assert(pack.target_system == (byte)(byte)235);
                Debug.Assert(pack.start_stop == (byte)(byte)9);
            };
            REQUEST_DATA_STREAM p66 = new REQUEST_DATA_STREAM();
            PH.setPack(p66);
            p66.target_component = (byte)(byte)220;
            p66.target_system = (byte)(byte)235;
            p66.req_stream_id = (byte)(byte)26;
            p66.start_stop = (byte)(byte)9;
            p66.req_message_rate = (ushort)(ushort)32476;
            ADV_TEST_CH.send(p66);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnDATA_STREAMReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.on_off == (byte)(byte)166);
                Debug.Assert(pack.stream_id == (byte)(byte)186);
                Debug.Assert(pack.message_rate == (ushort)(ushort)60027);
            };
            DATA_STREAM p67 = new DATA_STREAM();
            PH.setPack(p67);
            p67.message_rate = (ushort)(ushort)60027;
            p67.stream_id = (byte)(byte)186;
            p67.on_off = (byte)(byte)166;
            ADV_TEST_CH.send(p67);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMANUAL_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target == (byte)(byte)41);
                Debug.Assert(pack.z == (short)(short)21729);
                Debug.Assert(pack.x == (short)(short)28406);
                Debug.Assert(pack.y == (short)(short)31434);
                Debug.Assert(pack.buttons == (ushort)(ushort)64774);
                Debug.Assert(pack.r == (short)(short) -16935);
            };
            MANUAL_CONTROL p69 = new MANUAL_CONTROL();
            PH.setPack(p69);
            p69.buttons = (ushort)(ushort)64774;
            p69.z = (short)(short)21729;
            p69.r = (short)(short) -16935;
            p69.x = (short)(short)28406;
            p69.target = (byte)(byte)41;
            p69.y = (short)(short)31434;
            ADV_TEST_CH.send(p69);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRC_CHANNELS_OVERRIDEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)6);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)10965);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)61820);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)23379);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)28650);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)61901);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)61492);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)7985);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)48900);
                Debug.Assert(pack.target_component == (byte)(byte)106);
            };
            RC_CHANNELS_OVERRIDE p70 = new RC_CHANNELS_OVERRIDE();
            PH.setPack(p70);
            p70.target_system = (byte)(byte)6;
            p70.chan4_raw = (ushort)(ushort)48900;
            p70.chan7_raw = (ushort)(ushort)23379;
            p70.chan2_raw = (ushort)(ushort)7985;
            p70.chan3_raw = (ushort)(ushort)61820;
            p70.chan5_raw = (ushort)(ushort)61901;
            p70.chan1_raw = (ushort)(ushort)61492;
            p70.chan8_raw = (ushort)(ushort)10965;
            p70.chan6_raw = (ushort)(ushort)28650;
            p70.target_component = (byte)(byte)106;
            ADV_TEST_CH.send(p70);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_ITEM_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param3 == (float)2.4309125E38F);
                Debug.Assert(pack.current == (byte)(byte)152);
                Debug.Assert(pack.param4 == (float) -1.8638731E38F);
                Debug.Assert(pack.frame == MAV_FRAME.MAV_FRAME_GLOBAL);
                Debug.Assert(pack.autocontinue == (byte)(byte)233);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
                Debug.Assert(pack.target_component == (byte)(byte)164);
                Debug.Assert(pack.z == (float) -6.677524E37F);
                Debug.Assert(pack.y == (int) -606350952);
                Debug.Assert(pack.command == MAV_CMD.MAV_CMD_DO_DIGICAM_CONTROL);
                Debug.Assert(pack.seq == (ushort)(ushort)43647);
                Debug.Assert(pack.x == (int)1454369669);
                Debug.Assert(pack.param1 == (float) -1.5913639E38F);
                Debug.Assert(pack.param2 == (float)2.7942133E38F);
                Debug.Assert(pack.target_system == (byte)(byte)162);
            };
            MISSION_ITEM_INT p73 = new MISSION_ITEM_INT();
            PH.setPack(p73);
            p73.seq = (ushort)(ushort)43647;
            p73.command = MAV_CMD.MAV_CMD_DO_DIGICAM_CONTROL;
            p73.target_component = (byte)(byte)164;
            p73.param3 = (float)2.4309125E38F;
            p73.param2 = (float)2.7942133E38F;
            p73.param4 = (float) -1.8638731E38F;
            p73.autocontinue = (byte)(byte)233;
            p73.param1 = (float) -1.5913639E38F;
            p73.target_system = (byte)(byte)162;
            p73.current = (byte)(byte)152;
            p73.z = (float) -6.677524E37F;
            p73.frame = MAV_FRAME.MAV_FRAME_GLOBAL;
            p73.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION;
            p73.x = (int)1454369669;
            p73.y = (int) -606350952;
            ADV_TEST_CH.send(p73);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnVFR_HUDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.airspeed == (float) -2.1283085E38F);
                Debug.Assert(pack.throttle == (ushort)(ushort)57225);
                Debug.Assert(pack.groundspeed == (float) -2.818279E38F);
                Debug.Assert(pack.heading == (short)(short)21251);
                Debug.Assert(pack.climb == (float)5.893062E37F);
                Debug.Assert(pack.alt == (float)3.1304038E37F);
            };
            VFR_HUD p74 = new VFR_HUD();
            PH.setPack(p74);
            p74.alt = (float)3.1304038E37F;
            p74.groundspeed = (float) -2.818279E38F;
            p74.heading = (short)(short)21251;
            p74.throttle = (ushort)(ushort)57225;
            p74.climb = (float)5.893062E37F;
            p74.airspeed = (float) -2.1283085E38F;
            ADV_TEST_CH.send(p74);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCOMMAND_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param1 == (float) -2.2812734E38F);
                Debug.Assert(pack.x == (int)2066259777);
                Debug.Assert(pack.autocontinue == (byte)(byte)90);
                Debug.Assert(pack.z == (float) -2.9827888E38F);
                Debug.Assert(pack.command == MAV_CMD.MAV_CMD_NAV_RALLY_POINT);
                Debug.Assert(pack.target_system == (byte)(byte)57);
                Debug.Assert(pack.frame == MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED);
                Debug.Assert(pack.param2 == (float) -2.3243804E37F);
                Debug.Assert(pack.current == (byte)(byte)184);
                Debug.Assert(pack.target_component == (byte)(byte)140);
                Debug.Assert(pack.param3 == (float)2.3093578E38F);
                Debug.Assert(pack.param4 == (float) -2.1777862E38F);
                Debug.Assert(pack.y == (int) -1234633442);
            };
            COMMAND_INT p75 = new COMMAND_INT();
            PH.setPack(p75);
            p75.target_component = (byte)(byte)140;
            p75.param4 = (float) -2.1777862E38F;
            p75.param1 = (float) -2.2812734E38F;
            p75.param2 = (float) -2.3243804E37F;
            p75.frame = MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED;
            p75.param3 = (float)2.3093578E38F;
            p75.x = (int)2066259777;
            p75.target_system = (byte)(byte)57;
            p75.command = MAV_CMD.MAV_CMD_NAV_RALLY_POINT;
            p75.z = (float) -2.9827888E38F;
            p75.y = (int) -1234633442;
            p75.autocontinue = (byte)(byte)90;
            p75.current = (byte)(byte)184;
            ADV_TEST_CH.send(p75);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCOMMAND_LONGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)248);
                Debug.Assert(pack.param7 == (float) -2.508671E38F);
                Debug.Assert(pack.param2 == (float)1.4722951E38F);
                Debug.Assert(pack.target_system == (byte)(byte)171);
                Debug.Assert(pack.param3 == (float) -2.0391397E38F);
                Debug.Assert(pack.param6 == (float)2.5744505E38F);
                Debug.Assert(pack.param4 == (float) -2.0197702E38F);
                Debug.Assert(pack.param5 == (float)2.5276265E38F);
                Debug.Assert(pack.param1 == (float)1.1731004E38F);
                Debug.Assert(pack.command == MAV_CMD.MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION);
                Debug.Assert(pack.confirmation == (byte)(byte)43);
            };
            COMMAND_LONG p76 = new COMMAND_LONG();
            PH.setPack(p76);
            p76.target_system = (byte)(byte)171;
            p76.confirmation = (byte)(byte)43;
            p76.param2 = (float)1.4722951E38F;
            p76.param4 = (float) -2.0197702E38F;
            p76.param6 = (float)2.5744505E38F;
            p76.param3 = (float) -2.0391397E38F;
            p76.command = MAV_CMD.MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION;
            p76.target_component = (byte)(byte)248;
            p76.param5 = (float)2.5276265E38F;
            p76.param7 = (float) -2.508671E38F;
            p76.param1 = (float)1.1731004E38F;
            ADV_TEST_CH.send(p76);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCOMMAND_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.result == MAV_RESULT.MAV_RESULT_UNSUPPORTED);
                Debug.Assert(pack.progress_TRY(ph) == (byte)(byte)128);
                Debug.Assert(pack.target_component_TRY(ph) == (byte)(byte)88);
                Debug.Assert(pack.command == MAV_CMD.MAV_CMD_USER_1);
                Debug.Assert(pack.target_system_TRY(ph) == (byte)(byte)232);
                Debug.Assert(pack.result_param2_TRY(ph) == (int)269891251);
            };
            COMMAND_ACK p77 = new COMMAND_ACK();
            PH.setPack(p77);
            p77.target_component_SET((byte)(byte)88, PH) ;
            p77.target_system_SET((byte)(byte)232, PH) ;
            p77.command = MAV_CMD.MAV_CMD_USER_1;
            p77.result = MAV_RESULT.MAV_RESULT_UNSUPPORTED;
            p77.result_param2_SET((int)269891251, PH) ;
            p77.progress_SET((byte)(byte)128, PH) ;
            ADV_TEST_CH.send(p77);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMANUAL_SETPOINTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mode_switch == (byte)(byte)93);
                Debug.Assert(pack.thrust == (float)1.5347457E38F);
                Debug.Assert(pack.pitch == (float)1.8071467E38F);
                Debug.Assert(pack.yaw == (float)5.997019E37F);
                Debug.Assert(pack.manual_override_switch == (byte)(byte)189);
                Debug.Assert(pack.roll == (float) -7.0923327E37F);
                Debug.Assert(pack.time_boot_ms == (uint)2004747117U);
            };
            MANUAL_SETPOINT p81 = new MANUAL_SETPOINT();
            PH.setPack(p81);
            p81.manual_override_switch = (byte)(byte)189;
            p81.yaw = (float)5.997019E37F;
            p81.time_boot_ms = (uint)2004747117U;
            p81.roll = (float) -7.0923327E37F;
            p81.pitch = (float)1.8071467E38F;
            p81.mode_switch = (byte)(byte)93;
            p81.thrust = (float)1.5347457E38F;
            ADV_TEST_CH.send(p81);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_ATTITUDE_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.body_pitch_rate == (float)1.8094686E38F);
                Debug.Assert(pack.time_boot_ms == (uint)1971717027U);
                Debug.Assert(pack.thrust == (float)2.433217E38F);
                Debug.Assert(pack.type_mask == (byte)(byte)184);
                Debug.Assert(pack.target_system == (byte)(byte)157);
                Debug.Assert(pack.body_roll_rate == (float) -1.2961264E38F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-1.2235839E38F, -3.9418572E37F, -2.490993E38F, 2.7611032E38F}));
                Debug.Assert(pack.body_yaw_rate == (float) -1.3279399E37F);
                Debug.Assert(pack.target_component == (byte)(byte)96);
            };
            SET_ATTITUDE_TARGET p82 = new SET_ATTITUDE_TARGET();
            PH.setPack(p82);
            p82.target_system = (byte)(byte)157;
            p82.body_pitch_rate = (float)1.8094686E38F;
            p82.time_boot_ms = (uint)1971717027U;
            p82.type_mask = (byte)(byte)184;
            p82.body_yaw_rate = (float) -1.3279399E37F;
            p82.thrust = (float)2.433217E38F;
            p82.body_roll_rate = (float) -1.2961264E38F;
            p82.q_SET(new float[] {-1.2235839E38F, -3.9418572E37F, -2.490993E38F, 2.7611032E38F}, 0) ;
            p82.target_component = (byte)(byte)96;
            ADV_TEST_CH.send(p82);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATTITUDE_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.thrust == (float)2.4226825E38F);
                Debug.Assert(pack.body_roll_rate == (float)3.3923636E38F);
                Debug.Assert(pack.type_mask == (byte)(byte)210);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-1.6376199E38F, 1.8311274E38F, -3.0872614E38F, 4.9621726E37F}));
                Debug.Assert(pack.body_yaw_rate == (float)3.2988502E38F);
                Debug.Assert(pack.body_pitch_rate == (float)8.68098E37F);
                Debug.Assert(pack.time_boot_ms == (uint)3941012794U);
            };
            ATTITUDE_TARGET p83 = new ATTITUDE_TARGET();
            PH.setPack(p83);
            p83.thrust = (float)2.4226825E38F;
            p83.q_SET(new float[] {-1.6376199E38F, 1.8311274E38F, -3.0872614E38F, 4.9621726E37F}, 0) ;
            p83.time_boot_ms = (uint)3941012794U;
            p83.type_mask = (byte)(byte)210;
            p83.body_pitch_rate = (float)8.68098E37F;
            p83.body_yaw_rate = (float)3.2988502E38F;
            p83.body_roll_rate = (float)3.3923636E38F;
            ADV_TEST_CH.send(p83);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_POSITION_TARGET_LOCAL_NEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.yaw == (float) -3.0478805E38F);
                Debug.Assert(pack.afy == (float)3.2753524E38F);
                Debug.Assert(pack.target_system == (byte)(byte)44);
                Debug.Assert(pack.target_component == (byte)(byte)183);
                Debug.Assert(pack.y == (float) -1.231058E38F);
                Debug.Assert(pack.vz == (float) -2.0074547E38F);
                Debug.Assert(pack.vx == (float) -5.6936E37F);
                Debug.Assert(pack.time_boot_ms == (uint)1824233110U);
                Debug.Assert(pack.x == (float) -9.11238E37F);
                Debug.Assert(pack.afz == (float)3.9539941E37F);
                Debug.Assert(pack.afx == (float) -1.4924242E37F);
                Debug.Assert(pack.vy == (float)2.3170436E38F);
                Debug.Assert(pack.z == (float) -2.836605E38F);
                Debug.Assert(pack.yaw_rate == (float)2.496933E38F);
                Debug.Assert(pack.coordinate_frame == MAV_FRAME.MAV_FRAME_MISSION);
                Debug.Assert(pack.type_mask == (ushort)(ushort)22204);
            };
            SET_POSITION_TARGET_LOCAL_NED p84 = new SET_POSITION_TARGET_LOCAL_NED();
            PH.setPack(p84);
            p84.yaw_rate = (float)2.496933E38F;
            p84.afy = (float)3.2753524E38F;
            p84.vz = (float) -2.0074547E38F;
            p84.yaw = (float) -3.0478805E38F;
            p84.y = (float) -1.231058E38F;
            p84.time_boot_ms = (uint)1824233110U;
            p84.x = (float) -9.11238E37F;
            p84.afx = (float) -1.4924242E37F;
            p84.target_component = (byte)(byte)183;
            p84.vx = (float) -5.6936E37F;
            p84.target_system = (byte)(byte)44;
            p84.type_mask = (ushort)(ushort)22204;
            p84.vy = (float)2.3170436E38F;
            p84.afz = (float)3.9539941E37F;
            p84.coordinate_frame = MAV_FRAME.MAV_FRAME_MISSION;
            p84.z = (float) -2.836605E38F;
            ADV_TEST_CH.send(p84);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_POSITION_TARGET_GLOBAL_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vx == (float)3.2905672E38F);
                Debug.Assert(pack.afz == (float)9.761085E37F);
                Debug.Assert(pack.vy == (float)3.0045617E38F);
                Debug.Assert(pack.lat_int == (int) -373185787);
                Debug.Assert(pack.lon_int == (int)1005529501);
                Debug.Assert(pack.yaw == (float) -2.7646271E38F);
                Debug.Assert(pack.coordinate_frame == MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT);
                Debug.Assert(pack.afy == (float) -6.0347313E37F);
                Debug.Assert(pack.yaw_rate == (float)2.8730788E38F);
                Debug.Assert(pack.vz == (float)2.6670478E38F);
                Debug.Assert(pack.target_component == (byte)(byte)86);
                Debug.Assert(pack.alt == (float)3.2268762E38F);
                Debug.Assert(pack.target_system == (byte)(byte)86);
                Debug.Assert(pack.afx == (float)3.4024162E38F);
                Debug.Assert(pack.time_boot_ms == (uint)3873404554U);
                Debug.Assert(pack.type_mask == (ushort)(ushort)62844);
            };
            SET_POSITION_TARGET_GLOBAL_INT p86 = new SET_POSITION_TARGET_GLOBAL_INT();
            PH.setPack(p86);
            p86.alt = (float)3.2268762E38F;
            p86.vy = (float)3.0045617E38F;
            p86.lat_int = (int) -373185787;
            p86.yaw = (float) -2.7646271E38F;
            p86.vx = (float)3.2905672E38F;
            p86.afy = (float) -6.0347313E37F;
            p86.afx = (float)3.4024162E38F;
            p86.coordinate_frame = MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT;
            p86.lon_int = (int)1005529501;
            p86.yaw_rate = (float)2.8730788E38F;
            p86.vz = (float)2.6670478E38F;
            p86.time_boot_ms = (uint)3873404554U;
            p86.afz = (float)9.761085E37F;
            p86.target_component = (byte)(byte)86;
            p86.target_system = (byte)(byte)86;
            p86.type_mask = (ushort)(ushort)62844;
            ADV_TEST_CH.send(p86);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPOSITION_TARGET_GLOBAL_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.afz == (float) -2.0456526E38F);
                Debug.Assert(pack.lon_int == (int)2053369669);
                Debug.Assert(pack.afx == (float) -1.8173844E38F);
                Debug.Assert(pack.yaw == (float)2.6639243E38F);
                Debug.Assert(pack.alt == (float)1.454276E38F);
                Debug.Assert(pack.afy == (float)6.5542277E37F);
                Debug.Assert(pack.type_mask == (ushort)(ushort)15151);
                Debug.Assert(pack.yaw_rate == (float)2.3006597E38F);
                Debug.Assert(pack.lat_int == (int)1769458466);
                Debug.Assert(pack.vz == (float) -1.4912918E38F);
                Debug.Assert(pack.vx == (float)2.4368741E38F);
                Debug.Assert(pack.coordinate_frame == MAV_FRAME.MAV_FRAME_GLOBAL);
                Debug.Assert(pack.time_boot_ms == (uint)1891986642U);
                Debug.Assert(pack.vy == (float)4.7135884E37F);
            };
            GroundControl.POSITION_TARGET_GLOBAL_INT p87 = CommunicationChannel.new_POSITION_TARGET_GLOBAL_INT();
            PH.setPack(p87);
            p87.afx = (float) -1.8173844E38F;
            p87.alt = (float)1.454276E38F;
            p87.afy = (float)6.5542277E37F;
            p87.vz = (float) -1.4912918E38F;
            p87.time_boot_ms = (uint)1891986642U;
            p87.lon_int = (int)2053369669;
            p87.yaw = (float)2.6639243E38F;
            p87.lat_int = (int)1769458466;
            p87.vy = (float)4.7135884E37F;
            p87.type_mask = (ushort)(ushort)15151;
            p87.afz = (float) -2.0456526E38F;
            p87.yaw_rate = (float)2.3006597E38F;
            p87.coordinate_frame = MAV_FRAME.MAV_FRAME_GLOBAL;
            p87.vx = (float)2.4368741E38F;
            CommunicationChannel.instance.send(p87);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)1903470214U);
                Debug.Assert(pack.roll == (float)1.223241E37F);
                Debug.Assert(pack.y == (float) -2.3019782E38F);
                Debug.Assert(pack.yaw == (float) -3.120044E38F);
                Debug.Assert(pack.z == (float)2.7659341E38F);
                Debug.Assert(pack.pitch == (float)3.3406139E38F);
                Debug.Assert(pack.x == (float)2.0705148E38F);
            };
            GroundControl.LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET p89 = CommunicationChannel.new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
            PH.setPack(p89);
            p89.x = (float)2.0705148E38F;
            p89.pitch = (float)3.3406139E38F;
            p89.yaw = (float) -3.120044E38F;
            p89.z = (float)2.7659341E38F;
            p89.y = (float) -2.3019782E38F;
            p89.time_boot_ms = (uint)1903470214U;
            p89.roll = (float)1.223241E37F;
            CommunicationChannel.instance.send(p89);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.rollspeed == (float) -3.1755495E38F);
                Debug.Assert(pack.alt == (int)2105543504);
                Debug.Assert(pack.yawspeed == (float) -1.2741423E38F);
                Debug.Assert(pack.roll == (float) -1.5592174E38F);
                Debug.Assert(pack.yacc == (short)(short)10676);
                Debug.Assert(pack.vx == (short)(short)31507);
                Debug.Assert(pack.lon == (int)295625884);
                Debug.Assert(pack.lat == (int)986104319);
                Debug.Assert(pack.xacc == (short)(short)23421);
                Debug.Assert(pack.pitch == (float) -1.9836937E38F);
                Debug.Assert(pack.yaw == (float) -2.5826266E38F);
                Debug.Assert(pack.time_usec == (ulong)7877941067040227545L);
                Debug.Assert(pack.pitchspeed == (float) -2.4774961E38F);
                Debug.Assert(pack.vy == (short)(short) -989);
                Debug.Assert(pack.zacc == (short)(short) -6050);
                Debug.Assert(pack.vz == (short)(short) -14901);
            };
            GroundControl.HIL_STATE p90 = CommunicationChannel.new_HIL_STATE();
            PH.setPack(p90);
            p90.pitchspeed = (float) -2.4774961E38F;
            p90.yawspeed = (float) -1.2741423E38F;
            p90.pitch = (float) -1.9836937E38F;
            p90.lat = (int)986104319;
            p90.vy = (short)(short) -989;
            p90.rollspeed = (float) -3.1755495E38F;
            p90.xacc = (short)(short)23421;
            p90.vx = (short)(short)31507;
            p90.vz = (short)(short) -14901;
            p90.lon = (int)295625884;
            p90.yaw = (float) -2.5826266E38F;
            p90.yacc = (short)(short)10676;
            p90.alt = (int)2105543504;
            p90.roll = (float) -1.5592174E38F;
            p90.zacc = (short)(short) -6050;
            p90.time_usec = (ulong)7877941067040227545L;
            CommunicationChannel.instance.send(p90);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_CONTROLSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.aux2 == (float) -1.0927231E38F);
                Debug.Assert(pack.mode == MAV_MODE.MAV_MODE_AUTO_ARMED);
                Debug.Assert(pack.time_usec == (ulong)4701431861282089575L);
                Debug.Assert(pack.aux3 == (float) -2.2554725E38F);
                Debug.Assert(pack.yaw_rudder == (float)1.6081375E38F);
                Debug.Assert(pack.aux1 == (float)8.070362E37F);
                Debug.Assert(pack.nav_mode == (byte)(byte)35);
                Debug.Assert(pack.throttle == (float) -2.8314459E38F);
                Debug.Assert(pack.roll_ailerons == (float) -3.2715888E38F);
                Debug.Assert(pack.pitch_elevator == (float)8.51252E37F);
                Debug.Assert(pack.aux4 == (float) -8.63143E37F);
            };
            GroundControl.HIL_CONTROLS p91 = CommunicationChannel.new_HIL_CONTROLS();
            PH.setPack(p91);
            p91.throttle = (float) -2.8314459E38F;
            p91.mode = MAV_MODE.MAV_MODE_AUTO_ARMED;
            p91.time_usec = (ulong)4701431861282089575L;
            p91.nav_mode = (byte)(byte)35;
            p91.pitch_elevator = (float)8.51252E37F;
            p91.roll_ailerons = (float) -3.2715888E38F;
            p91.yaw_rudder = (float)1.6081375E38F;
            p91.aux3 = (float) -2.2554725E38F;
            p91.aux4 = (float) -8.63143E37F;
            p91.aux2 = (float) -1.0927231E38F;
            p91.aux1 = (float)8.070362E37F;
            CommunicationChannel.instance.send(p91);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_RC_INPUTS_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan12_raw == (ushort)(ushort)25188);
                Debug.Assert(pack.time_usec == (ulong)2778847702957578283L);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)795);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)54413);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)49757);
                Debug.Assert(pack.chan9_raw == (ushort)(ushort)47339);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)57791);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)3419);
                Debug.Assert(pack.chan11_raw == (ushort)(ushort)58023);
                Debug.Assert(pack.rssi == (byte)(byte)141);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)26696);
                Debug.Assert(pack.chan10_raw == (ushort)(ushort)13508);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)8508);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)12761);
            };
            GroundControl.HIL_RC_INPUTS_RAW p92 = CommunicationChannel.new_HIL_RC_INPUTS_RAW();
            PH.setPack(p92);
            p92.chan11_raw = (ushort)(ushort)58023;
            p92.chan4_raw = (ushort)(ushort)54413;
            p92.chan9_raw = (ushort)(ushort)47339;
            p92.chan10_raw = (ushort)(ushort)13508;
            p92.chan2_raw = (ushort)(ushort)12761;
            p92.chan5_raw = (ushort)(ushort)795;
            p92.chan8_raw = (ushort)(ushort)26696;
            p92.chan6_raw = (ushort)(ushort)8508;
            p92.chan3_raw = (ushort)(ushort)49757;
            p92.chan7_raw = (ushort)(ushort)3419;
            p92.chan1_raw = (ushort)(ushort)57791;
            p92.time_usec = (ulong)2778847702957578283L;
            p92.rssi = (byte)(byte)141;
            p92.chan12_raw = (ushort)(ushort)25188;
            CommunicationChannel.instance.send(p92);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_ACTUATOR_CONTROLSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mode == MAV_MODE.MAV_MODE_MANUAL_DISARMED);
                Debug.Assert(pack.controls.SequenceEqual(new float[] {1.0213521E38F, -2.0012966E38F, 5.6098224E37F, 3.3119923E38F, 1.779388E38F, 1.4802455E38F, 1.793408E38F, -1.95497E37F, -2.280684E38F, -2.1491513E38F, -1.2273771E38F, 1.893265E38F, 2.9566423E38F, -1.1442093E37F, -2.5653163E38F, 2.1444028E38F}));
                Debug.Assert(pack.flags == (ulong)916023204375925338L);
                Debug.Assert(pack.time_usec == (ulong)5329583541284216493L);
            };
            GroundControl.HIL_ACTUATOR_CONTROLS p93 = CommunicationChannel.new_HIL_ACTUATOR_CONTROLS();
            PH.setPack(p93);
            p93.mode = MAV_MODE.MAV_MODE_MANUAL_DISARMED;
            p93.time_usec = (ulong)5329583541284216493L;
            p93.controls_SET(new float[] {1.0213521E38F, -2.0012966E38F, 5.6098224E37F, 3.3119923E38F, 1.779388E38F, 1.4802455E38F, 1.793408E38F, -1.95497E37F, -2.280684E38F, -2.1491513E38F, -1.2273771E38F, 1.893265E38F, 2.9566423E38F, -1.1442093E37F, -2.5653163E38F, 2.1444028E38F}, 0) ;
            p93.flags = (ulong)916023204375925338L;
            CommunicationChannel.instance.send(p93);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnOPTICAL_FLOWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sensor_id == (byte)(byte)27);
                Debug.Assert(pack.time_usec == (ulong)7239468250747215057L);
                Debug.Assert(pack.flow_rate_x_TRY(ph) == (float) -1.4999225E38F);
                Debug.Assert(pack.flow_comp_m_y == (float)1.0064231E38F);
                Debug.Assert(pack.flow_y == (short)(short)7784);
                Debug.Assert(pack.flow_x == (short)(short)27215);
                Debug.Assert(pack.flow_comp_m_x == (float)2.8794578E38F);
                Debug.Assert(pack.quality == (byte)(byte)142);
                Debug.Assert(pack.ground_distance == (float)3.2911916E37F);
                Debug.Assert(pack.flow_rate_y_TRY(ph) == (float) -8.4923666E37F);
            };
            GroundControl.OPTICAL_FLOW p100 = CommunicationChannel.new_OPTICAL_FLOW();
            PH.setPack(p100);
            p100.flow_y = (short)(short)7784;
            p100.ground_distance = (float)3.2911916E37F;
            p100.flow_x = (short)(short)27215;
            p100.sensor_id = (byte)(byte)27;
            p100.time_usec = (ulong)7239468250747215057L;
            p100.flow_rate_x_SET((float) -1.4999225E38F, PH) ;
            p100.flow_comp_m_y = (float)1.0064231E38F;
            p100.flow_comp_m_x = (float)2.8794578E38F;
            p100.flow_rate_y_SET((float) -8.4923666E37F, PH) ;
            p100.quality = (byte)(byte)142;
            CommunicationChannel.instance.send(p100);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGLOBAL_VISION_POSITION_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.yaw == (float)3.286804E38F);
                Debug.Assert(pack.pitch == (float) -9.230742E37F);
                Debug.Assert(pack.z == (float) -2.9639653E38F);
                Debug.Assert(pack.y == (float)2.7671823E38F);
                Debug.Assert(pack.roll == (float)1.1790282E38F);
                Debug.Assert(pack.usec == (ulong)779938525658669511L);
                Debug.Assert(pack.x == (float)6.5944787E37F);
            };
            GroundControl.GLOBAL_VISION_POSITION_ESTIMATE p101 = CommunicationChannel.new_GLOBAL_VISION_POSITION_ESTIMATE();
            PH.setPack(p101);
            p101.x = (float)6.5944787E37F;
            p101.roll = (float)1.1790282E38F;
            p101.yaw = (float)3.286804E38F;
            p101.pitch = (float) -9.230742E37F;
            p101.z = (float) -2.9639653E38F;
            p101.usec = (ulong)779938525658669511L;
            p101.y = (float)2.7671823E38F;
            CommunicationChannel.instance.send(p101);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnVISION_POSITION_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.y == (float) -9.161092E37F);
                Debug.Assert(pack.yaw == (float)6.406464E37F);
                Debug.Assert(pack.x == (float) -1.9168867E38F);
                Debug.Assert(pack.pitch == (float) -7.9267886E37F);
                Debug.Assert(pack.roll == (float)2.677274E38F);
                Debug.Assert(pack.z == (float) -3.6611248E37F);
                Debug.Assert(pack.usec == (ulong)6450741791249407977L);
            };
            GroundControl.VISION_POSITION_ESTIMATE p102 = CommunicationChannel.new_VISION_POSITION_ESTIMATE();
            PH.setPack(p102);
            p102.pitch = (float) -7.9267886E37F;
            p102.roll = (float)2.677274E38F;
            p102.z = (float) -3.6611248E37F;
            p102.usec = (ulong)6450741791249407977L;
            p102.yaw = (float)6.406464E37F;
            p102.y = (float) -9.161092E37F;
            p102.x = (float) -1.9168867E38F;
            CommunicationChannel.instance.send(p102);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnVISION_SPEED_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.z == (float) -1.153637E38F);
                Debug.Assert(pack.y == (float)2.2512866E38F);
                Debug.Assert(pack.usec == (ulong)6856430693892641789L);
                Debug.Assert(pack.x == (float)1.8063212E38F);
            };
            GroundControl.VISION_SPEED_ESTIMATE p103 = CommunicationChannel.new_VISION_SPEED_ESTIMATE();
            PH.setPack(p103);
            p103.y = (float)2.2512866E38F;
            p103.x = (float)1.8063212E38F;
            p103.z = (float) -1.153637E38F;
            p103.usec = (ulong)6856430693892641789L;
            CommunicationChannel.instance.send(p103);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnVICON_POSITION_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.y == (float)4.780732E37F);
                Debug.Assert(pack.yaw == (float)1.8235391E38F);
                Debug.Assert(pack.z == (float)1.4241977E38F);
                Debug.Assert(pack.usec == (ulong)7515161686353253144L);
                Debug.Assert(pack.x == (float)1.1875222E38F);
                Debug.Assert(pack.roll == (float) -1.3497294E38F);
                Debug.Assert(pack.pitch == (float) -2.8120344E38F);
            };
            GroundControl.VICON_POSITION_ESTIMATE p104 = CommunicationChannel.new_VICON_POSITION_ESTIMATE();
            PH.setPack(p104);
            p104.z = (float)1.4241977E38F;
            p104.yaw = (float)1.8235391E38F;
            p104.usec = (ulong)7515161686353253144L;
            p104.x = (float)1.1875222E38F;
            p104.roll = (float) -1.3497294E38F;
            p104.pitch = (float) -2.8120344E38F;
            p104.y = (float)4.780732E37F;
            CommunicationChannel.instance.send(p104);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIGHRES_IMUReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.zacc == (float) -2.5265122E38F);
                Debug.Assert(pack.ygyro == (float) -2.2008057E38F);
                Debug.Assert(pack.ymag == (float) -2.5881703E38F);
                Debug.Assert(pack.abs_pressure == (float) -1.3715863E38F);
                Debug.Assert(pack.pressure_alt == (float)8.201227E37F);
                Debug.Assert(pack.time_usec == (ulong)5353243079676718519L);
                Debug.Assert(pack.xmag == (float)8.512775E37F);
                Debug.Assert(pack.fields_updated == (ushort)(ushort)20906);
                Debug.Assert(pack.diff_pressure == (float)1.9753384E38F);
                Debug.Assert(pack.yacc == (float) -2.5275219E38F);
                Debug.Assert(pack.temperature == (float) -1.5090368E37F);
                Debug.Assert(pack.zgyro == (float) -7.258465E37F);
                Debug.Assert(pack.zmag == (float) -4.960866E37F);
                Debug.Assert(pack.xacc == (float)3.167621E38F);
                Debug.Assert(pack.xgyro == (float)2.9996457E38F);
            };
            GroundControl.HIGHRES_IMU p105 = CommunicationChannel.new_HIGHRES_IMU();
            PH.setPack(p105);
            p105.ygyro = (float) -2.2008057E38F;
            p105.xacc = (float)3.167621E38F;
            p105.time_usec = (ulong)5353243079676718519L;
            p105.temperature = (float) -1.5090368E37F;
            p105.yacc = (float) -2.5275219E38F;
            p105.fields_updated = (ushort)(ushort)20906;
            p105.xmag = (float)8.512775E37F;
            p105.ymag = (float) -2.5881703E38F;
            p105.zacc = (float) -2.5265122E38F;
            p105.diff_pressure = (float)1.9753384E38F;
            p105.abs_pressure = (float) -1.3715863E38F;
            p105.zmag = (float) -4.960866E37F;
            p105.pressure_alt = (float)8.201227E37F;
            p105.zgyro = (float) -7.258465E37F;
            p105.xgyro = (float)2.9996457E38F;
            CommunicationChannel.instance.send(p105);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnOPTICAL_FLOW_RADReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.integrated_x == (float)2.8245134E38F);
                Debug.Assert(pack.integrated_zgyro == (float) -2.8093157E38F);
                Debug.Assert(pack.integrated_ygyro == (float)9.333529E37F);
                Debug.Assert(pack.time_usec == (ulong)519011913225410916L);
                Debug.Assert(pack.temperature == (short)(short)2528);
                Debug.Assert(pack.integration_time_us == (uint)3178847269U);
                Debug.Assert(pack.distance == (float)3.0857822E38F);
                Debug.Assert(pack.quality == (byte)(byte)195);
                Debug.Assert(pack.sensor_id == (byte)(byte)129);
                Debug.Assert(pack.integrated_xgyro == (float)7.835808E36F);
                Debug.Assert(pack.time_delta_distance_us == (uint)1167713458U);
                Debug.Assert(pack.integrated_y == (float) -1.4633816E38F);
            };
            GroundControl.OPTICAL_FLOW_RAD p106 = CommunicationChannel.new_OPTICAL_FLOW_RAD();
            PH.setPack(p106);
            p106.integrated_x = (float)2.8245134E38F;
            p106.quality = (byte)(byte)195;
            p106.time_delta_distance_us = (uint)1167713458U;
            p106.integrated_y = (float) -1.4633816E38F;
            p106.integrated_xgyro = (float)7.835808E36F;
            p106.integrated_ygyro = (float)9.333529E37F;
            p106.time_usec = (ulong)519011913225410916L;
            p106.temperature = (short)(short)2528;
            p106.integration_time_us = (uint)3178847269U;
            p106.integrated_zgyro = (float) -2.8093157E38F;
            p106.sensor_id = (byte)(byte)129;
            p106.distance = (float)3.0857822E38F;
            CommunicationChannel.instance.send(p106);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_SENSORReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ymag == (float)9.570986E37F);
                Debug.Assert(pack.xgyro == (float)2.0369758E38F);
                Debug.Assert(pack.fields_updated == (uint)4148356206U);
                Debug.Assert(pack.abs_pressure == (float) -7.7369944E37F);
                Debug.Assert(pack.xacc == (float)1.9621712E38F);
                Debug.Assert(pack.time_usec == (ulong)7569977826016428673L);
                Debug.Assert(pack.diff_pressure == (float)1.7208846E38F);
                Debug.Assert(pack.zacc == (float)2.4214733E38F);
                Debug.Assert(pack.pressure_alt == (float)2.7620188E38F);
                Debug.Assert(pack.yacc == (float) -1.2228935E38F);
                Debug.Assert(pack.zgyro == (float) -1.1647898E38F);
                Debug.Assert(pack.xmag == (float)5.7412996E37F);
                Debug.Assert(pack.zmag == (float)3.1755775E38F);
                Debug.Assert(pack.ygyro == (float)1.227117E38F);
                Debug.Assert(pack.temperature == (float) -8.2774506E37F);
            };
            GroundControl.HIL_SENSOR p107 = CommunicationChannel.new_HIL_SENSOR();
            PH.setPack(p107);
            p107.xgyro = (float)2.0369758E38F;
            p107.zgyro = (float) -1.1647898E38F;
            p107.temperature = (float) -8.2774506E37F;
            p107.time_usec = (ulong)7569977826016428673L;
            p107.fields_updated = (uint)4148356206U;
            p107.pressure_alt = (float)2.7620188E38F;
            p107.zmag = (float)3.1755775E38F;
            p107.yacc = (float) -1.2228935E38F;
            p107.ygyro = (float)1.227117E38F;
            p107.abs_pressure = (float) -7.7369944E37F;
            p107.ymag = (float)9.570986E37F;
            p107.xacc = (float)1.9621712E38F;
            p107.zacc = (float)2.4214733E38F;
            p107.xmag = (float)5.7412996E37F;
            p107.diff_pressure = (float)1.7208846E38F;
            CommunicationChannel.instance.send(p107);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSIM_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.roll == (float) -3.3172418E38F);
                Debug.Assert(pack.alt == (float)3.3288884E38F);
                Debug.Assert(pack.xacc == (float)3.0364922E37F);
                Debug.Assert(pack.std_dev_vert == (float)4.5004005E37F);
                Debug.Assert(pack.q4 == (float)1.8391495E38F);
                Debug.Assert(pack.zacc == (float) -2.0726357E38F);
                Debug.Assert(pack.q3 == (float) -1.1306205E37F);
                Debug.Assert(pack.std_dev_horz == (float) -2.293929E38F);
                Debug.Assert(pack.ygyro == (float) -1.8635519E38F);
                Debug.Assert(pack.xgyro == (float)3.2867708E37F);
                Debug.Assert(pack.lat == (float)5.4101746E37F);
                Debug.Assert(pack.pitch == (float)5.179999E36F);
                Debug.Assert(pack.lon == (float) -4.4438465E37F);
                Debug.Assert(pack.vn == (float) -3.0556628E38F);
                Debug.Assert(pack.q2 == (float) -4.0455556E35F);
                Debug.Assert(pack.yacc == (float)9.165665E37F);
                Debug.Assert(pack.vd == (float) -1.7910662E38F);
                Debug.Assert(pack.yaw == (float) -1.2448655E38F);
                Debug.Assert(pack.zgyro == (float) -8.369021E37F);
                Debug.Assert(pack.ve == (float) -1.8281528E38F);
                Debug.Assert(pack.q1 == (float)3.3180647E38F);
            };
            GroundControl.SIM_STATE p108 = CommunicationChannel.new_SIM_STATE();
            PH.setPack(p108);
            p108.lat = (float)5.4101746E37F;
            p108.ygyro = (float) -1.8635519E38F;
            p108.lon = (float) -4.4438465E37F;
            p108.roll = (float) -3.3172418E38F;
            p108.q2 = (float) -4.0455556E35F;
            p108.yacc = (float)9.165665E37F;
            p108.q4 = (float)1.8391495E38F;
            p108.vn = (float) -3.0556628E38F;
            p108.zgyro = (float) -8.369021E37F;
            p108.q1 = (float)3.3180647E38F;
            p108.ve = (float) -1.8281528E38F;
            p108.vd = (float) -1.7910662E38F;
            p108.xacc = (float)3.0364922E37F;
            p108.q3 = (float) -1.1306205E37F;
            p108.zacc = (float) -2.0726357E38F;
            p108.alt = (float)3.3288884E38F;
            p108.pitch = (float)5.179999E36F;
            p108.std_dev_horz = (float) -2.293929E38F;
            p108.std_dev_vert = (float)4.5004005E37F;
            p108.yaw = (float) -1.2448655E38F;
            p108.xgyro = (float)3.2867708E37F;
            CommunicationChannel.instance.send(p108);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRADIO_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.remrssi == (byte)(byte)182);
                Debug.Assert(pack.remnoise == (byte)(byte)18);
                Debug.Assert(pack.txbuf == (byte)(byte)237);
                Debug.Assert(pack.rssi == (byte)(byte)0);
                Debug.Assert(pack.rxerrors == (ushort)(ushort)27013);
                Debug.Assert(pack.fixed_ == (ushort)(ushort)21367);
                Debug.Assert(pack.noise == (byte)(byte)169);
            };
            GroundControl.RADIO_STATUS p109 = CommunicationChannel.new_RADIO_STATUS();
            PH.setPack(p109);
            p109.fixed_ = (ushort)(ushort)21367;
            p109.noise = (byte)(byte)169;
            p109.remrssi = (byte)(byte)182;
            p109.rssi = (byte)(byte)0;
            p109.txbuf = (byte)(byte)237;
            p109.rxerrors = (ushort)(ushort)27013;
            p109.remnoise = (byte)(byte)18;
            CommunicationChannel.instance.send(p109);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnFILE_TRANSFER_PROTOCOLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.payload.SequenceEqual(new byte[] {(byte)196, (byte)105, (byte)82, (byte)216, (byte)104, (byte)124, (byte)113, (byte)118, (byte)222, (byte)228, (byte)36, (byte)167, (byte)130, (byte)32, (byte)243, (byte)114, (byte)165, (byte)16, (byte)244, (byte)246, (byte)188, (byte)136, (byte)239, (byte)191, (byte)72, (byte)184, (byte)173, (byte)197, (byte)15, (byte)3, (byte)15, (byte)27, (byte)107, (byte)239, (byte)223, (byte)43, (byte)75, (byte)57, (byte)116, (byte)28, (byte)151, (byte)44, (byte)114, (byte)130, (byte)70, (byte)57, (byte)93, (byte)109, (byte)243, (byte)248, (byte)51, (byte)76, (byte)104, (byte)82, (byte)176, (byte)49, (byte)130, (byte)135, (byte)223, (byte)47, (byte)59, (byte)41, (byte)63, (byte)187, (byte)170, (byte)1, (byte)246, (byte)229, (byte)131, (byte)222, (byte)28, (byte)236, (byte)151, (byte)109, (byte)238, (byte)255, (byte)80, (byte)97, (byte)92, (byte)21, (byte)152, (byte)52, (byte)143, (byte)126, (byte)132, (byte)8, (byte)229, (byte)158, (byte)123, (byte)134, (byte)58, (byte)45, (byte)218, (byte)31, (byte)187, (byte)62, (byte)213, (byte)79, (byte)92, (byte)22, (byte)172, (byte)60, (byte)150, (byte)97, (byte)37, (byte)171, (byte)97, (byte)245, (byte)21, (byte)99, (byte)138, (byte)170, (byte)70, (byte)228, (byte)19, (byte)58, (byte)110, (byte)24, (byte)200, (byte)235, (byte)152, (byte)36, (byte)78, (byte)43, (byte)119, (byte)84, (byte)81, (byte)239, (byte)138, (byte)128, (byte)82, (byte)75, (byte)143, (byte)127, (byte)15, (byte)101, (byte)204, (byte)53, (byte)63, (byte)134, (byte)238, (byte)6, (byte)28, (byte)195, (byte)114, (byte)16, (byte)149, (byte)188, (byte)173, (byte)133, (byte)56, (byte)221, (byte)184, (byte)8, (byte)170, (byte)40, (byte)193, (byte)114, (byte)52, (byte)70, (byte)83, (byte)243, (byte)92, (byte)209, (byte)243, (byte)3, (byte)42, (byte)161, (byte)43, (byte)50, (byte)104, (byte)46, (byte)207, (byte)144, (byte)180, (byte)44, (byte)229, (byte)8, (byte)141, (byte)86, (byte)211, (byte)56, (byte)3, (byte)160, (byte)198, (byte)155, (byte)187, (byte)77, (byte)139, (byte)23, (byte)250, (byte)203, (byte)34, (byte)36, (byte)98, (byte)244, (byte)88, (byte)193, (byte)24, (byte)45, (byte)185, (byte)167, (byte)235, (byte)39, (byte)221, (byte)207, (byte)70, (byte)192, (byte)204, (byte)172, (byte)237, (byte)36, (byte)3, (byte)13, (byte)68, (byte)253, (byte)75, (byte)26, (byte)126, (byte)126, (byte)101, (byte)138, (byte)140, (byte)141, (byte)239, (byte)88, (byte)118, (byte)220, (byte)76, (byte)176, (byte)2, (byte)242, (byte)250, (byte)191, (byte)15, (byte)40, (byte)57, (byte)177, (byte)152, (byte)56, (byte)5, (byte)215, (byte)244, (byte)109, (byte)59, (byte)215, (byte)56, (byte)31, (byte)42, (byte)209, (byte)218}));
                Debug.Assert(pack.target_component == (byte)(byte)68);
                Debug.Assert(pack.target_system == (byte)(byte)133);
                Debug.Assert(pack.target_network == (byte)(byte)156);
            };
            GroundControl.FILE_TRANSFER_PROTOCOL p110 = CommunicationChannel.new_FILE_TRANSFER_PROTOCOL();
            PH.setPack(p110);
            p110.target_network = (byte)(byte)156;
            p110.payload_SET(new byte[] {(byte)196, (byte)105, (byte)82, (byte)216, (byte)104, (byte)124, (byte)113, (byte)118, (byte)222, (byte)228, (byte)36, (byte)167, (byte)130, (byte)32, (byte)243, (byte)114, (byte)165, (byte)16, (byte)244, (byte)246, (byte)188, (byte)136, (byte)239, (byte)191, (byte)72, (byte)184, (byte)173, (byte)197, (byte)15, (byte)3, (byte)15, (byte)27, (byte)107, (byte)239, (byte)223, (byte)43, (byte)75, (byte)57, (byte)116, (byte)28, (byte)151, (byte)44, (byte)114, (byte)130, (byte)70, (byte)57, (byte)93, (byte)109, (byte)243, (byte)248, (byte)51, (byte)76, (byte)104, (byte)82, (byte)176, (byte)49, (byte)130, (byte)135, (byte)223, (byte)47, (byte)59, (byte)41, (byte)63, (byte)187, (byte)170, (byte)1, (byte)246, (byte)229, (byte)131, (byte)222, (byte)28, (byte)236, (byte)151, (byte)109, (byte)238, (byte)255, (byte)80, (byte)97, (byte)92, (byte)21, (byte)152, (byte)52, (byte)143, (byte)126, (byte)132, (byte)8, (byte)229, (byte)158, (byte)123, (byte)134, (byte)58, (byte)45, (byte)218, (byte)31, (byte)187, (byte)62, (byte)213, (byte)79, (byte)92, (byte)22, (byte)172, (byte)60, (byte)150, (byte)97, (byte)37, (byte)171, (byte)97, (byte)245, (byte)21, (byte)99, (byte)138, (byte)170, (byte)70, (byte)228, (byte)19, (byte)58, (byte)110, (byte)24, (byte)200, (byte)235, (byte)152, (byte)36, (byte)78, (byte)43, (byte)119, (byte)84, (byte)81, (byte)239, (byte)138, (byte)128, (byte)82, (byte)75, (byte)143, (byte)127, (byte)15, (byte)101, (byte)204, (byte)53, (byte)63, (byte)134, (byte)238, (byte)6, (byte)28, (byte)195, (byte)114, (byte)16, (byte)149, (byte)188, (byte)173, (byte)133, (byte)56, (byte)221, (byte)184, (byte)8, (byte)170, (byte)40, (byte)193, (byte)114, (byte)52, (byte)70, (byte)83, (byte)243, (byte)92, (byte)209, (byte)243, (byte)3, (byte)42, (byte)161, (byte)43, (byte)50, (byte)104, (byte)46, (byte)207, (byte)144, (byte)180, (byte)44, (byte)229, (byte)8, (byte)141, (byte)86, (byte)211, (byte)56, (byte)3, (byte)160, (byte)198, (byte)155, (byte)187, (byte)77, (byte)139, (byte)23, (byte)250, (byte)203, (byte)34, (byte)36, (byte)98, (byte)244, (byte)88, (byte)193, (byte)24, (byte)45, (byte)185, (byte)167, (byte)235, (byte)39, (byte)221, (byte)207, (byte)70, (byte)192, (byte)204, (byte)172, (byte)237, (byte)36, (byte)3, (byte)13, (byte)68, (byte)253, (byte)75, (byte)26, (byte)126, (byte)126, (byte)101, (byte)138, (byte)140, (byte)141, (byte)239, (byte)88, (byte)118, (byte)220, (byte)76, (byte)176, (byte)2, (byte)242, (byte)250, (byte)191, (byte)15, (byte)40, (byte)57, (byte)177, (byte)152, (byte)56, (byte)5, (byte)215, (byte)244, (byte)109, (byte)59, (byte)215, (byte)56, (byte)31, (byte)42, (byte)209, (byte)218}, 0) ;
            p110.target_component = (byte)(byte)68;
            p110.target_system = (byte)(byte)133;
            CommunicationChannel.instance.send(p110);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTIMESYNCReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.tc1 == (long) -553470986451010624L);
                Debug.Assert(pack.ts1 == (long) -6964283498152621630L);
            };
            GroundControl.TIMESYNC p111 = CommunicationChannel.new_TIMESYNC();
            PH.setPack(p111);
            p111.tc1 = (long) -553470986451010624L;
            p111.ts1 = (long) -6964283498152621630L;
            CommunicationChannel.instance.send(p111);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCAMERA_TRIGGERReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)9126778970615791279L);
                Debug.Assert(pack.seq == (uint)889070432U);
            };
            GroundControl.CAMERA_TRIGGER p112 = CommunicationChannel.new_CAMERA_TRIGGER();
            PH.setPack(p112);
            p112.seq = (uint)889070432U;
            p112.time_usec = (ulong)9126778970615791279L;
            CommunicationChannel.instance.send(p112);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_GPSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lon == (int)1164051088);
                Debug.Assert(pack.vn == (short)(short)3611);
                Debug.Assert(pack.time_usec == (ulong)9197793687510110747L);
                Debug.Assert(pack.eph == (ushort)(ushort)3275);
                Debug.Assert(pack.epv == (ushort)(ushort)52656);
                Debug.Assert(pack.ve == (short)(short)10795);
                Debug.Assert(pack.vd == (short)(short)775);
                Debug.Assert(pack.vel == (ushort)(ushort)31064);
                Debug.Assert(pack.cog == (ushort)(ushort)61881);
                Debug.Assert(pack.satellites_visible == (byte)(byte)76);
                Debug.Assert(pack.fix_type == (byte)(byte)253);
                Debug.Assert(pack.alt == (int) -463881395);
                Debug.Assert(pack.lat == (int)110270768);
            };
            GroundControl.HIL_GPS p113 = CommunicationChannel.new_HIL_GPS();
            PH.setPack(p113);
            p113.vd = (short)(short)775;
            p113.alt = (int) -463881395;
            p113.satellites_visible = (byte)(byte)76;
            p113.cog = (ushort)(ushort)61881;
            p113.fix_type = (byte)(byte)253;
            p113.lat = (int)110270768;
            p113.lon = (int)1164051088;
            p113.time_usec = (ulong)9197793687510110747L;
            p113.vn = (short)(short)3611;
            p113.vel = (ushort)(ushort)31064;
            p113.eph = (ushort)(ushort)3275;
            p113.ve = (short)(short)10795;
            p113.epv = (ushort)(ushort)52656;
            CommunicationChannel.instance.send(p113);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_OPTICAL_FLOWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.integrated_y == (float)1.457005E38F);
                Debug.Assert(pack.integration_time_us == (uint)870664850U);
                Debug.Assert(pack.quality == (byte)(byte)245);
                Debug.Assert(pack.temperature == (short)(short)25524);
                Debug.Assert(pack.integrated_x == (float) -1.6896316E38F);
                Debug.Assert(pack.time_delta_distance_us == (uint)281531635U);
                Debug.Assert(pack.time_usec == (ulong)8540258989484162792L);
                Debug.Assert(pack.integrated_ygyro == (float)1.7161204E38F);
                Debug.Assert(pack.sensor_id == (byte)(byte)157);
                Debug.Assert(pack.integrated_zgyro == (float) -3.2384487E38F);
                Debug.Assert(pack.distance == (float)1.5655461E38F);
                Debug.Assert(pack.integrated_xgyro == (float)3.1317276E38F);
            };
            GroundControl.HIL_OPTICAL_FLOW p114 = CommunicationChannel.new_HIL_OPTICAL_FLOW();
            PH.setPack(p114);
            p114.sensor_id = (byte)(byte)157;
            p114.integrated_y = (float)1.457005E38F;
            p114.temperature = (short)(short)25524;
            p114.distance = (float)1.5655461E38F;
            p114.time_usec = (ulong)8540258989484162792L;
            p114.integrated_zgyro = (float) -3.2384487E38F;
            p114.integrated_x = (float) -1.6896316E38F;
            p114.time_delta_distance_us = (uint)281531635U;
            p114.integration_time_us = (uint)870664850U;
            p114.integrated_xgyro = (float)3.1317276E38F;
            p114.integrated_ygyro = (float)1.7161204E38F;
            p114.quality = (byte)(byte)245;
            CommunicationChannel.instance.send(p114);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_STATE_QUATERNIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)6304528983165421626L);
                Debug.Assert(pack.yawspeed == (float)2.8183497E38F);
                Debug.Assert(pack.attitude_quaternion.SequenceEqual(new float[] {9.246189E37F, 1.6453512E38F, 1.3290423E38F, 1.715842E38F}));
                Debug.Assert(pack.pitchspeed == (float)1.1524412E38F);
                Debug.Assert(pack.lat == (int)61087257);
                Debug.Assert(pack.vy == (short)(short) -18967);
                Debug.Assert(pack.true_airspeed == (ushort)(ushort)45029);
                Debug.Assert(pack.alt == (int)1983842526);
                Debug.Assert(pack.vx == (short)(short) -32430);
                Debug.Assert(pack.yacc == (short)(short)9848);
                Debug.Assert(pack.ind_airspeed == (ushort)(ushort)7737);
                Debug.Assert(pack.vz == (short)(short) -11962);
                Debug.Assert(pack.xacc == (short)(short)360);
                Debug.Assert(pack.zacc == (short)(short)10);
                Debug.Assert(pack.lon == (int) -1189135313);
                Debug.Assert(pack.rollspeed == (float) -5.435351E33F);
            };
            GroundControl.HIL_STATE_QUATERNION p115 = CommunicationChannel.new_HIL_STATE_QUATERNION();
            PH.setPack(p115);
            p115.true_airspeed = (ushort)(ushort)45029;
            p115.rollspeed = (float) -5.435351E33F;
            p115.lon = (int) -1189135313;
            p115.time_usec = (ulong)6304528983165421626L;
            p115.vz = (short)(short) -11962;
            p115.zacc = (short)(short)10;
            p115.lat = (int)61087257;
            p115.yawspeed = (float)2.8183497E38F;
            p115.vx = (short)(short) -32430;
            p115.alt = (int)1983842526;
            p115.xacc = (short)(short)360;
            p115.yacc = (short)(short)9848;
            p115.ind_airspeed = (ushort)(ushort)7737;
            p115.pitchspeed = (float)1.1524412E38F;
            p115.attitude_quaternion_SET(new float[] {9.246189E37F, 1.6453512E38F, 1.3290423E38F, 1.715842E38F}, 0) ;
            p115.vy = (short)(short) -18967;
            CommunicationChannel.instance.send(p115);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_IMU2Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.xgyro == (short)(short) -31539);
                Debug.Assert(pack.xmag == (short)(short)22434);
                Debug.Assert(pack.xacc == (short)(short) -26591);
                Debug.Assert(pack.yacc == (short)(short)5855);
                Debug.Assert(pack.ymag == (short)(short)28603);
                Debug.Assert(pack.ygyro == (short)(short) -9341);
                Debug.Assert(pack.time_boot_ms == (uint)1674170842U);
                Debug.Assert(pack.zgyro == (short)(short) -17648);
                Debug.Assert(pack.zmag == (short)(short)15612);
                Debug.Assert(pack.zacc == (short)(short)25312);
            };
            GroundControl.SCALED_IMU2 p116 = CommunicationChannel.new_SCALED_IMU2();
            PH.setPack(p116);
            p116.ymag = (short)(short)28603;
            p116.zgyro = (short)(short) -17648;
            p116.zmag = (short)(short)15612;
            p116.xmag = (short)(short)22434;
            p116.xgyro = (short)(short) -31539;
            p116.time_boot_ms = (uint)1674170842U;
            p116.zacc = (short)(short)25312;
            p116.ygyro = (short)(short) -9341;
            p116.yacc = (short)(short)5855;
            p116.xacc = (short)(short) -26591;
            CommunicationChannel.instance.send(p116);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)231);
                Debug.Assert(pack.end == (ushort)(ushort)29718);
                Debug.Assert(pack.target_component == (byte)(byte)205);
                Debug.Assert(pack.start == (ushort)(ushort)40460);
            };
            GroundControl.LOG_REQUEST_LIST p117 = CommunicationChannel.new_LOG_REQUEST_LIST();
            PH.setPack(p117);
            p117.end = (ushort)(ushort)29718;
            p117.target_component = (byte)(byte)205;
            p117.start = (ushort)(ushort)40460;
            p117.target_system = (byte)(byte)231;
            CommunicationChannel.instance.send(p117);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_ENTRYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.size == (uint)737704420U);
                Debug.Assert(pack.id == (ushort)(ushort)30710);
                Debug.Assert(pack.num_logs == (ushort)(ushort)57809);
                Debug.Assert(pack.last_log_num == (ushort)(ushort)40624);
                Debug.Assert(pack.time_utc == (uint)1804897807U);
            };
            GroundControl.LOG_ENTRY p118 = CommunicationChannel.new_LOG_ENTRY();
            PH.setPack(p118);
            p118.id = (ushort)(ushort)30710;
            p118.last_log_num = (ushort)(ushort)40624;
            p118.time_utc = (uint)1804897807U;
            p118.num_logs = (ushort)(ushort)57809;
            p118.size = (uint)737704420U;
            CommunicationChannel.instance.send(p118);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_REQUEST_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)191);
                Debug.Assert(pack.ofs == (uint)2406119363U);
                Debug.Assert(pack.id == (ushort)(ushort)27514);
                Debug.Assert(pack.target_component == (byte)(byte)209);
                Debug.Assert(pack.count == (uint)1306984267U);
            };
            GroundControl.LOG_REQUEST_DATA p119 = CommunicationChannel.new_LOG_REQUEST_DATA();
            PH.setPack(p119);
            p119.ofs = (uint)2406119363U;
            p119.target_component = (byte)(byte)209;
            p119.target_system = (byte)(byte)191;
            p119.id = (ushort)(ushort)27514;
            p119.count = (uint)1306984267U;
            CommunicationChannel.instance.send(p119);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ofs == (uint)2729812238U);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)150, (byte)66, (byte)118, (byte)19, (byte)195, (byte)123, (byte)75, (byte)109, (byte)198, (byte)158, (byte)89, (byte)21, (byte)13, (byte)204, (byte)139, (byte)123, (byte)0, (byte)10, (byte)53, (byte)31, (byte)120, (byte)241, (byte)6, (byte)116, (byte)49, (byte)99, (byte)123, (byte)65, (byte)93, (byte)132, (byte)11, (byte)102, (byte)223, (byte)97, (byte)18, (byte)101, (byte)62, (byte)175, (byte)13, (byte)102, (byte)206, (byte)153, (byte)197, (byte)47, (byte)218, (byte)33, (byte)205, (byte)239, (byte)151, (byte)57, (byte)134, (byte)84, (byte)96, (byte)56, (byte)247, (byte)13, (byte)193, (byte)80, (byte)215, (byte)200, (byte)185, (byte)205, (byte)151, (byte)97, (byte)72, (byte)63, (byte)74, (byte)134, (byte)171, (byte)0, (byte)136, (byte)123, (byte)72, (byte)250, (byte)6, (byte)20, (byte)75, (byte)206, (byte)14, (byte)2, (byte)34, (byte)221, (byte)60, (byte)45, (byte)79, (byte)156, (byte)99, (byte)118, (byte)194, (byte)16}));
                Debug.Assert(pack.count == (byte)(byte)136);
                Debug.Assert(pack.id == (ushort)(ushort)18560);
            };
            GroundControl.LOG_DATA p120 = CommunicationChannel.new_LOG_DATA();
            PH.setPack(p120);
            p120.data__SET(new byte[] {(byte)150, (byte)66, (byte)118, (byte)19, (byte)195, (byte)123, (byte)75, (byte)109, (byte)198, (byte)158, (byte)89, (byte)21, (byte)13, (byte)204, (byte)139, (byte)123, (byte)0, (byte)10, (byte)53, (byte)31, (byte)120, (byte)241, (byte)6, (byte)116, (byte)49, (byte)99, (byte)123, (byte)65, (byte)93, (byte)132, (byte)11, (byte)102, (byte)223, (byte)97, (byte)18, (byte)101, (byte)62, (byte)175, (byte)13, (byte)102, (byte)206, (byte)153, (byte)197, (byte)47, (byte)218, (byte)33, (byte)205, (byte)239, (byte)151, (byte)57, (byte)134, (byte)84, (byte)96, (byte)56, (byte)247, (byte)13, (byte)193, (byte)80, (byte)215, (byte)200, (byte)185, (byte)205, (byte)151, (byte)97, (byte)72, (byte)63, (byte)74, (byte)134, (byte)171, (byte)0, (byte)136, (byte)123, (byte)72, (byte)250, (byte)6, (byte)20, (byte)75, (byte)206, (byte)14, (byte)2, (byte)34, (byte)221, (byte)60, (byte)45, (byte)79, (byte)156, (byte)99, (byte)118, (byte)194, (byte)16}, 0) ;
            p120.id = (ushort)(ushort)18560;
            p120.count = (byte)(byte)136;
            p120.ofs = (uint)2729812238U;
            CommunicationChannel.instance.send(p120);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_ERASEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)191);
                Debug.Assert(pack.target_system == (byte)(byte)255);
            };
            GroundControl.LOG_ERASE p121 = CommunicationChannel.new_LOG_ERASE();
            PH.setPack(p121);
            p121.target_system = (byte)(byte)255;
            p121.target_component = (byte)(byte)191;
            CommunicationChannel.instance.send(p121);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_REQUEST_ENDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)158);
                Debug.Assert(pack.target_system == (byte)(byte)32);
            };
            GroundControl.LOG_REQUEST_END p122 = CommunicationChannel.new_LOG_REQUEST_END();
            PH.setPack(p122);
            p122.target_component = (byte)(byte)158;
            p122.target_system = (byte)(byte)32;
            CommunicationChannel.instance.send(p122);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_INJECT_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.len == (byte)(byte)166);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)10, (byte)115, (byte)31, (byte)190, (byte)235, (byte)184, (byte)107, (byte)31, (byte)72, (byte)80, (byte)237, (byte)72, (byte)91, (byte)128, (byte)188, (byte)155, (byte)55, (byte)95, (byte)157, (byte)94, (byte)159, (byte)46, (byte)144, (byte)119, (byte)43, (byte)147, (byte)149, (byte)1, (byte)186, (byte)7, (byte)243, (byte)187, (byte)173, (byte)91, (byte)104, (byte)108, (byte)82, (byte)253, (byte)178, (byte)228, (byte)99, (byte)114, (byte)162, (byte)7, (byte)134, (byte)181, (byte)24, (byte)244, (byte)242, (byte)186, (byte)110, (byte)43, (byte)3, (byte)154, (byte)150, (byte)225, (byte)109, (byte)185, (byte)105, (byte)53, (byte)243, (byte)89, (byte)198, (byte)77, (byte)62, (byte)118, (byte)203, (byte)23, (byte)37, (byte)152, (byte)141, (byte)101, (byte)253, (byte)136, (byte)51, (byte)238, (byte)47, (byte)218, (byte)237, (byte)48, (byte)10, (byte)211, (byte)59, (byte)147, (byte)53, (byte)138, (byte)122, (byte)1, (byte)209, (byte)3, (byte)246, (byte)22, (byte)170, (byte)150, (byte)74, (byte)249, (byte)200, (byte)208, (byte)137, (byte)51, (byte)80, (byte)168, (byte)205, (byte)251, (byte)200, (byte)4, (byte)80, (byte)233, (byte)77, (byte)245}));
                Debug.Assert(pack.target_system == (byte)(byte)172);
                Debug.Assert(pack.target_component == (byte)(byte)233);
            };
            GroundControl.GPS_INJECT_DATA p123 = CommunicationChannel.new_GPS_INJECT_DATA();
            PH.setPack(p123);
            p123.target_system = (byte)(byte)172;
            p123.len = (byte)(byte)166;
            p123.target_component = (byte)(byte)233;
            p123.data__SET(new byte[] {(byte)10, (byte)115, (byte)31, (byte)190, (byte)235, (byte)184, (byte)107, (byte)31, (byte)72, (byte)80, (byte)237, (byte)72, (byte)91, (byte)128, (byte)188, (byte)155, (byte)55, (byte)95, (byte)157, (byte)94, (byte)159, (byte)46, (byte)144, (byte)119, (byte)43, (byte)147, (byte)149, (byte)1, (byte)186, (byte)7, (byte)243, (byte)187, (byte)173, (byte)91, (byte)104, (byte)108, (byte)82, (byte)253, (byte)178, (byte)228, (byte)99, (byte)114, (byte)162, (byte)7, (byte)134, (byte)181, (byte)24, (byte)244, (byte)242, (byte)186, (byte)110, (byte)43, (byte)3, (byte)154, (byte)150, (byte)225, (byte)109, (byte)185, (byte)105, (byte)53, (byte)243, (byte)89, (byte)198, (byte)77, (byte)62, (byte)118, (byte)203, (byte)23, (byte)37, (byte)152, (byte)141, (byte)101, (byte)253, (byte)136, (byte)51, (byte)238, (byte)47, (byte)218, (byte)237, (byte)48, (byte)10, (byte)211, (byte)59, (byte)147, (byte)53, (byte)138, (byte)122, (byte)1, (byte)209, (byte)3, (byte)246, (byte)22, (byte)170, (byte)150, (byte)74, (byte)249, (byte)200, (byte)208, (byte)137, (byte)51, (byte)80, (byte)168, (byte)205, (byte)251, (byte)200, (byte)4, (byte)80, (byte)233, (byte)77, (byte)245}, 0) ;
            CommunicationChannel.instance.send(p123);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS2_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vel == (ushort)(ushort)9651);
                Debug.Assert(pack.alt == (int)1117797491);
                Debug.Assert(pack.epv == (ushort)(ushort)10338);
                Debug.Assert(pack.satellites_visible == (byte)(byte)26);
                Debug.Assert(pack.dgps_numch == (byte)(byte)162);
                Debug.Assert(pack.lon == (int) -1229687827);
                Debug.Assert(pack.dgps_age == (uint)784607042U);
                Debug.Assert(pack.lat == (int) -2143906198);
                Debug.Assert(pack.eph == (ushort)(ushort)2551);
                Debug.Assert(pack.fix_type == GPS_FIX_TYPE.GPS_FIX_TYPE_NO_FIX);
                Debug.Assert(pack.cog == (ushort)(ushort)28421);
                Debug.Assert(pack.time_usec == (ulong)3175750396463445943L);
            };
            GroundControl.GPS2_RAW p124 = CommunicationChannel.new_GPS2_RAW();
            PH.setPack(p124);
            p124.vel = (ushort)(ushort)9651;
            p124.dgps_numch = (byte)(byte)162;
            p124.fix_type = GPS_FIX_TYPE.GPS_FIX_TYPE_NO_FIX;
            p124.cog = (ushort)(ushort)28421;
            p124.dgps_age = (uint)784607042U;
            p124.satellites_visible = (byte)(byte)26;
            p124.lon = (int) -1229687827;
            p124.eph = (ushort)(ushort)2551;
            p124.time_usec = (ulong)3175750396463445943L;
            p124.lat = (int) -2143906198;
            p124.epv = (ushort)(ushort)10338;
            p124.alt = (int)1117797491;
            CommunicationChannel.instance.send(p124);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPOWER_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.Vcc == (ushort)(ushort)28330);
                Debug.Assert(pack.Vservo == (ushort)(ushort)61842);
                Debug.Assert(pack.flags == (MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_OVERCURRENT |
                                            MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT |
                                            MAV_POWER_STATUS.MAV_POWER_STATUS_BRICK_VALID));
            };
            GroundControl.POWER_STATUS p125 = CommunicationChannel.new_POWER_STATUS();
            PH.setPack(p125);
            p125.flags = (MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_OVERCURRENT |
                          MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT |
                          MAV_POWER_STATUS.MAV_POWER_STATUS_BRICK_VALID);
            p125.Vcc = (ushort)(ushort)28330;
            p125.Vservo = (ushort)(ushort)61842;
            CommunicationChannel.instance.send(p125);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSERIAL_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.flags == (SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_MULTI |
                                            SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_BLOCKING));
                Debug.Assert(pack.timeout == (ushort)(ushort)25964);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)19, (byte)177, (byte)229, (byte)219, (byte)81, (byte)225, (byte)127, (byte)40, (byte)82, (byte)110, (byte)192, (byte)38, (byte)242, (byte)132, (byte)8, (byte)5, (byte)166, (byte)46, (byte)230, (byte)245, (byte)146, (byte)40, (byte)198, (byte)101, (byte)87, (byte)114, (byte)245, (byte)138, (byte)12, (byte)45, (byte)38, (byte)180, (byte)115, (byte)95, (byte)124, (byte)180, (byte)1, (byte)197, (byte)109, (byte)139, (byte)34, (byte)187, (byte)97, (byte)23, (byte)149, (byte)245, (byte)220, (byte)67, (byte)118, (byte)234, (byte)248, (byte)59, (byte)26, (byte)149, (byte)81, (byte)66, (byte)39, (byte)80, (byte)173, (byte)112, (byte)146, (byte)127, (byte)206, (byte)23, (byte)233, (byte)82, (byte)21, (byte)12, (byte)180, (byte)10}));
                Debug.Assert(pack.baudrate == (uint)3580787308U);
                Debug.Assert(pack.count == (byte)(byte)253);
                Debug.Assert(pack.device == SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_TELEM2);
            };
            GroundControl.SERIAL_CONTROL p126 = CommunicationChannel.new_SERIAL_CONTROL();
            PH.setPack(p126);
            p126.timeout = (ushort)(ushort)25964;
            p126.flags = (SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_MULTI |
                          SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_BLOCKING);
            p126.baudrate = (uint)3580787308U;
            p126.count = (byte)(byte)253;
            p126.device = SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_TELEM2;
            p126.data__SET(new byte[] {(byte)19, (byte)177, (byte)229, (byte)219, (byte)81, (byte)225, (byte)127, (byte)40, (byte)82, (byte)110, (byte)192, (byte)38, (byte)242, (byte)132, (byte)8, (byte)5, (byte)166, (byte)46, (byte)230, (byte)245, (byte)146, (byte)40, (byte)198, (byte)101, (byte)87, (byte)114, (byte)245, (byte)138, (byte)12, (byte)45, (byte)38, (byte)180, (byte)115, (byte)95, (byte)124, (byte)180, (byte)1, (byte)197, (byte)109, (byte)139, (byte)34, (byte)187, (byte)97, (byte)23, (byte)149, (byte)245, (byte)220, (byte)67, (byte)118, (byte)234, (byte)248, (byte)59, (byte)26, (byte)149, (byte)81, (byte)66, (byte)39, (byte)80, (byte)173, (byte)112, (byte)146, (byte)127, (byte)206, (byte)23, (byte)233, (byte)82, (byte)21, (byte)12, (byte)180, (byte)10}, 0) ;
            CommunicationChannel.instance.send(p126);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_RTKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.wn == (ushort)(ushort)48800);
                Debug.Assert(pack.tow == (uint)2874745523U);
                Debug.Assert(pack.baseline_a_mm == (int)1902832442);
                Debug.Assert(pack.rtk_receiver_id == (byte)(byte)90);
                Debug.Assert(pack.rtk_rate == (byte)(byte)16);
                Debug.Assert(pack.baseline_b_mm == (int)354042443);
                Debug.Assert(pack.nsats == (byte)(byte)28);
                Debug.Assert(pack.baseline_c_mm == (int) -1981456196);
                Debug.Assert(pack.iar_num_hypotheses == (int)988939068);
                Debug.Assert(pack.baseline_coords_type == (byte)(byte)242);
                Debug.Assert(pack.rtk_health == (byte)(byte)6);
                Debug.Assert(pack.accuracy == (uint)781527591U);
                Debug.Assert(pack.time_last_baseline_ms == (uint)2182748025U);
            };
            GroundControl.GPS_RTK p127 = CommunicationChannel.new_GPS_RTK();
            PH.setPack(p127);
            p127.rtk_rate = (byte)(byte)16;
            p127.rtk_receiver_id = (byte)(byte)90;
            p127.tow = (uint)2874745523U;
            p127.baseline_a_mm = (int)1902832442;
            p127.baseline_b_mm = (int)354042443;
            p127.nsats = (byte)(byte)28;
            p127.accuracy = (uint)781527591U;
            p127.baseline_coords_type = (byte)(byte)242;
            p127.iar_num_hypotheses = (int)988939068;
            p127.baseline_c_mm = (int) -1981456196;
            p127.wn = (ushort)(ushort)48800;
            p127.rtk_health = (byte)(byte)6;
            p127.time_last_baseline_ms = (uint)2182748025U;
            CommunicationChannel.instance.send(p127);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS2_RTKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.rtk_receiver_id == (byte)(byte)54);
                Debug.Assert(pack.rtk_rate == (byte)(byte)187);
                Debug.Assert(pack.baseline_a_mm == (int)975022352);
                Debug.Assert(pack.accuracy == (uint)3033823914U);
                Debug.Assert(pack.wn == (ushort)(ushort)12852);
                Debug.Assert(pack.rtk_health == (byte)(byte)123);
                Debug.Assert(pack.baseline_b_mm == (int)627609042);
                Debug.Assert(pack.iar_num_hypotheses == (int)1935757956);
                Debug.Assert(pack.baseline_coords_type == (byte)(byte)135);
                Debug.Assert(pack.time_last_baseline_ms == (uint)2926521613U);
                Debug.Assert(pack.nsats == (byte)(byte)151);
                Debug.Assert(pack.tow == (uint)1834562276U);
                Debug.Assert(pack.baseline_c_mm == (int) -423698356);
            };
            GroundControl.GPS2_RTK p128 = CommunicationChannel.new_GPS2_RTK();
            PH.setPack(p128);
            p128.iar_num_hypotheses = (int)1935757956;
            p128.baseline_coords_type = (byte)(byte)135;
            p128.tow = (uint)1834562276U;
            p128.baseline_b_mm = (int)627609042;
            p128.rtk_rate = (byte)(byte)187;
            p128.baseline_c_mm = (int) -423698356;
            p128.rtk_receiver_id = (byte)(byte)54;
            p128.nsats = (byte)(byte)151;
            p128.accuracy = (uint)3033823914U;
            p128.baseline_a_mm = (int)975022352;
            p128.wn = (ushort)(ushort)12852;
            p128.rtk_health = (byte)(byte)123;
            p128.time_last_baseline_ms = (uint)2926521613U;
            CommunicationChannel.instance.send(p128);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_IMU3Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.yacc == (short)(short) -22644);
                Debug.Assert(pack.zgyro == (short)(short)20398);
                Debug.Assert(pack.ygyro == (short)(short)9111);
                Debug.Assert(pack.xgyro == (short)(short)29882);
                Debug.Assert(pack.xacc == (short)(short) -307);
                Debug.Assert(pack.xmag == (short)(short) -10774);
                Debug.Assert(pack.ymag == (short)(short) -24524);
                Debug.Assert(pack.time_boot_ms == (uint)4271443740U);
                Debug.Assert(pack.zacc == (short)(short)27202);
                Debug.Assert(pack.zmag == (short)(short)15412);
            };
            GroundControl.SCALED_IMU3 p129 = CommunicationChannel.new_SCALED_IMU3();
            PH.setPack(p129);
            p129.zgyro = (short)(short)20398;
            p129.zmag = (short)(short)15412;
            p129.time_boot_ms = (uint)4271443740U;
            p129.xacc = (short)(short) -307;
            p129.xmag = (short)(short) -10774;
            p129.yacc = (short)(short) -22644;
            p129.xgyro = (short)(short)29882;
            p129.zacc = (short)(short)27202;
            p129.ymag = (short)(short) -24524;
            p129.ygyro = (short)(short)9111;
            CommunicationChannel.instance.send(p129);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnDATA_TRANSMISSION_HANDSHAKEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.payload == (byte)(byte)254);
                Debug.Assert(pack.width == (ushort)(ushort)21164);
                Debug.Assert(pack.height == (ushort)(ushort)25725);
                Debug.Assert(pack.type == (byte)(byte)144);
                Debug.Assert(pack.packets == (ushort)(ushort)40266);
                Debug.Assert(pack.jpg_quality == (byte)(byte)20);
                Debug.Assert(pack.size == (uint)41751827U);
            };
            GroundControl.DATA_TRANSMISSION_HANDSHAKE p130 = CommunicationChannel.new_DATA_TRANSMISSION_HANDSHAKE();
            PH.setPack(p130);
            p130.type = (byte)(byte)144;
            p130.payload = (byte)(byte)254;
            p130.size = (uint)41751827U;
            p130.height = (ushort)(ushort)25725;
            p130.jpg_quality = (byte)(byte)20;
            p130.packets = (ushort)(ushort)40266;
            p130.width = (ushort)(ushort)21164;
            CommunicationChannel.instance.send(p130);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnENCAPSULATED_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)247, (byte)113, (byte)2, (byte)198, (byte)195, (byte)27, (byte)248, (byte)141, (byte)22, (byte)127, (byte)230, (byte)137, (byte)25, (byte)207, (byte)60, (byte)110, (byte)138, (byte)168, (byte)116, (byte)82, (byte)74, (byte)27, (byte)24, (byte)40, (byte)22, (byte)76, (byte)124, (byte)128, (byte)161, (byte)7, (byte)46, (byte)19, (byte)74, (byte)150, (byte)205, (byte)91, (byte)241, (byte)118, (byte)51, (byte)96, (byte)77, (byte)114, (byte)106, (byte)175, (byte)168, (byte)48, (byte)77, (byte)73, (byte)28, (byte)141, (byte)20, (byte)148, (byte)73, (byte)72, (byte)139, (byte)247, (byte)120, (byte)144, (byte)145, (byte)22, (byte)53, (byte)27, (byte)21, (byte)30, (byte)222, (byte)106, (byte)213, (byte)54, (byte)2, (byte)142, (byte)53, (byte)67, (byte)87, (byte)17, (byte)108, (byte)226, (byte)54, (byte)56, (byte)227, (byte)31, (byte)107, (byte)47, (byte)26, (byte)229, (byte)245, (byte)134, (byte)237, (byte)181, (byte)170, (byte)150, (byte)6, (byte)134, (byte)226, (byte)73, (byte)105, (byte)15, (byte)229, (byte)29, (byte)87, (byte)160, (byte)228, (byte)39, (byte)44, (byte)10, (byte)1, (byte)140, (byte)160, (byte)208, (byte)244, (byte)14, (byte)17, (byte)18, (byte)42, (byte)90, (byte)11, (byte)175, (byte)120, (byte)89, (byte)238, (byte)85, (byte)245, (byte)146, (byte)228, (byte)128, (byte)178, (byte)204, (byte)196, (byte)30, (byte)80, (byte)92, (byte)29, (byte)185, (byte)18, (byte)133, (byte)207, (byte)21, (byte)35, (byte)98, (byte)141, (byte)221, (byte)236, (byte)200, (byte)2, (byte)88, (byte)250, (byte)218, (byte)222, (byte)198, (byte)7, (byte)181, (byte)57, (byte)138, (byte)157, (byte)87, (byte)191, (byte)108, (byte)245, (byte)154, (byte)34, (byte)165, (byte)78, (byte)75, (byte)180, (byte)10, (byte)78, (byte)81, (byte)141, (byte)188, (byte)25, (byte)195, (byte)37, (byte)45, (byte)10, (byte)218, (byte)176, (byte)161, (byte)32, (byte)32, (byte)76, (byte)235, (byte)115, (byte)0, (byte)73, (byte)213, (byte)0, (byte)48, (byte)31, (byte)253, (byte)57, (byte)6, (byte)214, (byte)98, (byte)160, (byte)154, (byte)133, (byte)79, (byte)68, (byte)125, (byte)95, (byte)143, (byte)14, (byte)175, (byte)34, (byte)194, (byte)93, (byte)223, (byte)17, (byte)208, (byte)241, (byte)232, (byte)155, (byte)13, (byte)129, (byte)57, (byte)81, (byte)231, (byte)59, (byte)96, (byte)153, (byte)245, (byte)41, (byte)91, (byte)50, (byte)14, (byte)218, (byte)8, (byte)249, (byte)181, (byte)98, (byte)40, (byte)74, (byte)2, (byte)116, (byte)162, (byte)90, (byte)228, (byte)90, (byte)207, (byte)153, (byte)46, (byte)214, (byte)204, (byte)23, (byte)95, (byte)133, (byte)163, (byte)186, (byte)231, (byte)252, (byte)46, (byte)91, (byte)107, (byte)115}));
                Debug.Assert(pack.seqnr == (ushort)(ushort)21832);
            };
            GroundControl.ENCAPSULATED_DATA p131 = CommunicationChannel.new_ENCAPSULATED_DATA();
            PH.setPack(p131);
            p131.data__SET(new byte[] {(byte)247, (byte)113, (byte)2, (byte)198, (byte)195, (byte)27, (byte)248, (byte)141, (byte)22, (byte)127, (byte)230, (byte)137, (byte)25, (byte)207, (byte)60, (byte)110, (byte)138, (byte)168, (byte)116, (byte)82, (byte)74, (byte)27, (byte)24, (byte)40, (byte)22, (byte)76, (byte)124, (byte)128, (byte)161, (byte)7, (byte)46, (byte)19, (byte)74, (byte)150, (byte)205, (byte)91, (byte)241, (byte)118, (byte)51, (byte)96, (byte)77, (byte)114, (byte)106, (byte)175, (byte)168, (byte)48, (byte)77, (byte)73, (byte)28, (byte)141, (byte)20, (byte)148, (byte)73, (byte)72, (byte)139, (byte)247, (byte)120, (byte)144, (byte)145, (byte)22, (byte)53, (byte)27, (byte)21, (byte)30, (byte)222, (byte)106, (byte)213, (byte)54, (byte)2, (byte)142, (byte)53, (byte)67, (byte)87, (byte)17, (byte)108, (byte)226, (byte)54, (byte)56, (byte)227, (byte)31, (byte)107, (byte)47, (byte)26, (byte)229, (byte)245, (byte)134, (byte)237, (byte)181, (byte)170, (byte)150, (byte)6, (byte)134, (byte)226, (byte)73, (byte)105, (byte)15, (byte)229, (byte)29, (byte)87, (byte)160, (byte)228, (byte)39, (byte)44, (byte)10, (byte)1, (byte)140, (byte)160, (byte)208, (byte)244, (byte)14, (byte)17, (byte)18, (byte)42, (byte)90, (byte)11, (byte)175, (byte)120, (byte)89, (byte)238, (byte)85, (byte)245, (byte)146, (byte)228, (byte)128, (byte)178, (byte)204, (byte)196, (byte)30, (byte)80, (byte)92, (byte)29, (byte)185, (byte)18, (byte)133, (byte)207, (byte)21, (byte)35, (byte)98, (byte)141, (byte)221, (byte)236, (byte)200, (byte)2, (byte)88, (byte)250, (byte)218, (byte)222, (byte)198, (byte)7, (byte)181, (byte)57, (byte)138, (byte)157, (byte)87, (byte)191, (byte)108, (byte)245, (byte)154, (byte)34, (byte)165, (byte)78, (byte)75, (byte)180, (byte)10, (byte)78, (byte)81, (byte)141, (byte)188, (byte)25, (byte)195, (byte)37, (byte)45, (byte)10, (byte)218, (byte)176, (byte)161, (byte)32, (byte)32, (byte)76, (byte)235, (byte)115, (byte)0, (byte)73, (byte)213, (byte)0, (byte)48, (byte)31, (byte)253, (byte)57, (byte)6, (byte)214, (byte)98, (byte)160, (byte)154, (byte)133, (byte)79, (byte)68, (byte)125, (byte)95, (byte)143, (byte)14, (byte)175, (byte)34, (byte)194, (byte)93, (byte)223, (byte)17, (byte)208, (byte)241, (byte)232, (byte)155, (byte)13, (byte)129, (byte)57, (byte)81, (byte)231, (byte)59, (byte)96, (byte)153, (byte)245, (byte)41, (byte)91, (byte)50, (byte)14, (byte)218, (byte)8, (byte)249, (byte)181, (byte)98, (byte)40, (byte)74, (byte)2, (byte)116, (byte)162, (byte)90, (byte)228, (byte)90, (byte)207, (byte)153, (byte)46, (byte)214, (byte)204, (byte)23, (byte)95, (byte)133, (byte)163, (byte)186, (byte)231, (byte)252, (byte)46, (byte)91, (byte)107, (byte)115}, 0) ;
            p131.seqnr = (ushort)(ushort)21832;
            CommunicationChannel.instance.send(p131);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnDISTANCE_SENSORReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)1623420820U);
                Debug.Assert(pack.covariance == (byte)(byte)1);
                Debug.Assert(pack.min_distance == (ushort)(ushort)11311);
                Debug.Assert(pack.orientation == MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_270_PITCH_180);
                Debug.Assert(pack.current_distance == (ushort)(ushort)62600);
                Debug.Assert(pack.id == (byte)(byte)173);
                Debug.Assert(pack.type == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_RADAR);
                Debug.Assert(pack.max_distance == (ushort)(ushort)8676);
            };
            GroundControl.DISTANCE_SENSOR p132 = CommunicationChannel.new_DISTANCE_SENSOR();
            PH.setPack(p132);
            p132.covariance = (byte)(byte)1;
            p132.orientation = MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_270_PITCH_180;
            p132.max_distance = (ushort)(ushort)8676;
            p132.min_distance = (ushort)(ushort)11311;
            p132.time_boot_ms = (uint)1623420820U;
            p132.current_distance = (ushort)(ushort)62600;
            p132.type = MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_RADAR;
            p132.id = (byte)(byte)173;
            CommunicationChannel.instance.send(p132);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTERRAIN_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mask == (ulong)6992717119984519115L);
                Debug.Assert(pack.lat == (int) -1219255326);
                Debug.Assert(pack.grid_spacing == (ushort)(ushort)61852);
                Debug.Assert(pack.lon == (int) -147100670);
            };
            GroundControl.TERRAIN_REQUEST p133 = CommunicationChannel.new_TERRAIN_REQUEST();
            PH.setPack(p133);
            p133.mask = (ulong)6992717119984519115L;
            p133.grid_spacing = (ushort)(ushort)61852;
            p133.lat = (int) -1219255326;
            p133.lon = (int) -147100670;
            CommunicationChannel.instance.send(p133);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTERRAIN_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.gridbit == (byte)(byte)239);
                Debug.Assert(pack.data_.SequenceEqual(new short[] {(short) -28105, (short) -14770, (short)18687, (short) -9459, (short)8498, (short)22493, (short)14867, (short)20372, (short)392, (short) -15579, (short)8182, (short)24391, (short)2119, (short) -19662, (short) -28275, (short)30386}));
                Debug.Assert(pack.lon == (int)1860901249);
                Debug.Assert(pack.grid_spacing == (ushort)(ushort)11004);
                Debug.Assert(pack.lat == (int) -753334297);
            };
            GroundControl.TERRAIN_DATA p134 = CommunicationChannel.new_TERRAIN_DATA();
            PH.setPack(p134);
            p134.data__SET(new short[] {(short) -28105, (short) -14770, (short)18687, (short) -9459, (short)8498, (short)22493, (short)14867, (short)20372, (short)392, (short) -15579, (short)8182, (short)24391, (short)2119, (short) -19662, (short) -28275, (short)30386}, 0) ;
            p134.grid_spacing = (ushort)(ushort)11004;
            p134.gridbit = (byte)(byte)239;
            p134.lon = (int)1860901249;
            p134.lat = (int) -753334297;
            CommunicationChannel.instance.send(p134);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTERRAIN_CHECKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lat == (int)321239017);
                Debug.Assert(pack.lon == (int) -302187228);
            };
            GroundControl.TERRAIN_CHECK p135 = CommunicationChannel.new_TERRAIN_CHECK();
            PH.setPack(p135);
            p135.lat = (int)321239017;
            p135.lon = (int) -302187228;
            CommunicationChannel.instance.send(p135);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTERRAIN_REPORTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.terrain_height == (float)3.7663144E37F);
                Debug.Assert(pack.lat == (int) -2046289940);
                Debug.Assert(pack.lon == (int)1897245901);
                Debug.Assert(pack.current_height == (float)3.1698035E38F);
                Debug.Assert(pack.pending == (ushort)(ushort)8712);
                Debug.Assert(pack.spacing == (ushort)(ushort)52746);
                Debug.Assert(pack.loaded == (ushort)(ushort)54419);
            };
            GroundControl.TERRAIN_REPORT p136 = CommunicationChannel.new_TERRAIN_REPORT();
            PH.setPack(p136);
            p136.current_height = (float)3.1698035E38F;
            p136.pending = (ushort)(ushort)8712;
            p136.terrain_height = (float)3.7663144E37F;
            p136.lon = (int)1897245901;
            p136.loaded = (ushort)(ushort)54419;
            p136.lat = (int) -2046289940;
            p136.spacing = (ushort)(ushort)52746;
            CommunicationChannel.instance.send(p136);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_PRESSURE2Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.press_abs == (float)1.3699273E38F);
                Debug.Assert(pack.time_boot_ms == (uint)3582804629U);
                Debug.Assert(pack.temperature == (short)(short)10755);
                Debug.Assert(pack.press_diff == (float) -1.1430072E38F);
            };
            GroundControl.SCALED_PRESSURE2 p137 = CommunicationChannel.new_SCALED_PRESSURE2();
            PH.setPack(p137);
            p137.time_boot_ms = (uint)3582804629U;
            p137.press_diff = (float) -1.1430072E38F;
            p137.press_abs = (float)1.3699273E38F;
            p137.temperature = (short)(short)10755;
            CommunicationChannel.instance.send(p137);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATT_POS_MOCAPReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.q.SequenceEqual(new float[] {3.282178E38F, -2.6524317E38F, 1.2256709E38F, -5.72523E36F}));
                Debug.Assert(pack.y == (float)1.4287081E38F);
                Debug.Assert(pack.z == (float)4.5174443E37F);
                Debug.Assert(pack.time_usec == (ulong)3713724220496364471L);
                Debug.Assert(pack.x == (float) -2.7449132E38F);
            };
            GroundControl.ATT_POS_MOCAP p138 = CommunicationChannel.new_ATT_POS_MOCAP();
            PH.setPack(p138);
            p138.z = (float)4.5174443E37F;
            p138.time_usec = (ulong)3713724220496364471L;
            p138.x = (float) -2.7449132E38F;
            p138.q_SET(new float[] {3.282178E38F, -2.6524317E38F, 1.2256709E38F, -5.72523E36F}, 0) ;
            p138.y = (float)1.4287081E38F;
            CommunicationChannel.instance.send(p138);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_ACTUATOR_CONTROL_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.group_mlx == (byte)(byte)245);
                Debug.Assert(pack.target_system == (byte)(byte)186);
                Debug.Assert(pack.controls.SequenceEqual(new float[] {1.6285262E38F, 3.2489459E38F, -2.094044E38F, 3.2250767E38F, 3.1135922E38F, 5.635345E37F, 3.1200964E38F, 2.1159545E38F}));
                Debug.Assert(pack.time_usec == (ulong)4197961429897681369L);
                Debug.Assert(pack.target_component == (byte)(byte)160);
            };
            GroundControl.SET_ACTUATOR_CONTROL_TARGET p139 = CommunicationChannel.new_SET_ACTUATOR_CONTROL_TARGET();
            PH.setPack(p139);
            p139.controls_SET(new float[] {1.6285262E38F, 3.2489459E38F, -2.094044E38F, 3.2250767E38F, 3.1135922E38F, 5.635345E37F, 3.1200964E38F, 2.1159545E38F}, 0) ;
            p139.target_system = (byte)(byte)186;
            p139.time_usec = (ulong)4197961429897681369L;
            p139.group_mlx = (byte)(byte)245;
            p139.target_component = (byte)(byte)160;
            CommunicationChannel.instance.send(p139);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnACTUATOR_CONTROL_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.group_mlx == (byte)(byte)84);
                Debug.Assert(pack.controls.SequenceEqual(new float[] {3.2286158E38F, 1.3346072E38F, -2.6485452E37F, -2.6257078E38F, 1.8786278E38F, 1.6550425E38F, 2.6839828E38F, -5.6318157E37F}));
                Debug.Assert(pack.time_usec == (ulong)4279545633650345440L);
            };
            GroundControl.ACTUATOR_CONTROL_TARGET p140 = CommunicationChannel.new_ACTUATOR_CONTROL_TARGET();
            PH.setPack(p140);
            p140.controls_SET(new float[] {3.2286158E38F, 1.3346072E38F, -2.6485452E37F, -2.6257078E38F, 1.8786278E38F, 1.6550425E38F, 2.6839828E38F, -5.6318157E37F}, 0) ;
            p140.time_usec = (ulong)4279545633650345440L;
            p140.group_mlx = (byte)(byte)84;
            CommunicationChannel.instance.send(p140);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnALTITUDEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.bottom_clearance == (float)1.2575754E38F);
                Debug.Assert(pack.altitude_relative == (float) -3.886313E37F);
                Debug.Assert(pack.altitude_monotonic == (float)2.5148218E38F);
                Debug.Assert(pack.altitude_local == (float) -1.4455999E38F);
                Debug.Assert(pack.altitude_terrain == (float)2.77866E38F);
                Debug.Assert(pack.time_usec == (ulong)3519154667013881842L);
                Debug.Assert(pack.altitude_amsl == (float) -2.0000638E38F);
            };
            GroundControl.ALTITUDE p141 = CommunicationChannel.new_ALTITUDE();
            PH.setPack(p141);
            p141.bottom_clearance = (float)1.2575754E38F;
            p141.altitude_amsl = (float) -2.0000638E38F;
            p141.time_usec = (ulong)3519154667013881842L;
            p141.altitude_monotonic = (float)2.5148218E38F;
            p141.altitude_relative = (float) -3.886313E37F;
            p141.altitude_local = (float) -1.4455999E38F;
            p141.altitude_terrain = (float)2.77866E38F;
            CommunicationChannel.instance.send(p141);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRESOURCE_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.request_id == (byte)(byte)94);
                Debug.Assert(pack.storage.SequenceEqual(new byte[] {(byte)109, (byte)62, (byte)114, (byte)50, (byte)225, (byte)99, (byte)155, (byte)94, (byte)131, (byte)19, (byte)74, (byte)160, (byte)141, (byte)119, (byte)148, (byte)211, (byte)173, (byte)255, (byte)4, (byte)245, (byte)78, (byte)52, (byte)8, (byte)91, (byte)76, (byte)14, (byte)200, (byte)219, (byte)132, (byte)102, (byte)29, (byte)22, (byte)13, (byte)147, (byte)38, (byte)237, (byte)54, (byte)51, (byte)96, (byte)56, (byte)172, (byte)22, (byte)167, (byte)34, (byte)199, (byte)100, (byte)180, (byte)224, (byte)171, (byte)190, (byte)220, (byte)107, (byte)17, (byte)232, (byte)162, (byte)95, (byte)161, (byte)191, (byte)245, (byte)188, (byte)84, (byte)110, (byte)95, (byte)34, (byte)145, (byte)84, (byte)75, (byte)80, (byte)199, (byte)63, (byte)89, (byte)88, (byte)254, (byte)153, (byte)226, (byte)121, (byte)163, (byte)80, (byte)164, (byte)171, (byte)94, (byte)226, (byte)235, (byte)82, (byte)84, (byte)205, (byte)8, (byte)150, (byte)134, (byte)100, (byte)66, (byte)209, (byte)236, (byte)159, (byte)183, (byte)119, (byte)98, (byte)15, (byte)252, (byte)148, (byte)194, (byte)200, (byte)224, (byte)201, (byte)22, (byte)60, (byte)19, (byte)44, (byte)50, (byte)28, (byte)106, (byte)239, (byte)148, (byte)91, (byte)206, (byte)2, (byte)75, (byte)202, (byte)107, (byte)68}));
                Debug.Assert(pack.uri_type == (byte)(byte)224);
                Debug.Assert(pack.transfer_type == (byte)(byte)212);
                Debug.Assert(pack.uri.SequenceEqual(new byte[] {(byte)127, (byte)158, (byte)61, (byte)63, (byte)127, (byte)163, (byte)93, (byte)98, (byte)153, (byte)143, (byte)29, (byte)217, (byte)230, (byte)179, (byte)239, (byte)146, (byte)98, (byte)144, (byte)25, (byte)228, (byte)198, (byte)19, (byte)25, (byte)38, (byte)214, (byte)195, (byte)156, (byte)65, (byte)81, (byte)86, (byte)71, (byte)124, (byte)48, (byte)0, (byte)249, (byte)129, (byte)5, (byte)155, (byte)152, (byte)239, (byte)175, (byte)64, (byte)102, (byte)166, (byte)202, (byte)110, (byte)32, (byte)146, (byte)243, (byte)54, (byte)150, (byte)152, (byte)25, (byte)29, (byte)96, (byte)19, (byte)30, (byte)195, (byte)16, (byte)71, (byte)108, (byte)191, (byte)173, (byte)10, (byte)120, (byte)148, (byte)169, (byte)14, (byte)232, (byte)11, (byte)222, (byte)38, (byte)175, (byte)50, (byte)176, (byte)228, (byte)2, (byte)187, (byte)17, (byte)143, (byte)146, (byte)30, (byte)142, (byte)26, (byte)66, (byte)155, (byte)24, (byte)71, (byte)198, (byte)147, (byte)233, (byte)114, (byte)174, (byte)42, (byte)5, (byte)24, (byte)41, (byte)226, (byte)86, (byte)79, (byte)0, (byte)186, (byte)232, (byte)223, (byte)186, (byte)74, (byte)212, (byte)41, (byte)146, (byte)23, (byte)110, (byte)208, (byte)48, (byte)6, (byte)100, (byte)10, (byte)138, (byte)166, (byte)199, (byte)119}));
            };
            GroundControl.RESOURCE_REQUEST p142 = CommunicationChannel.new_RESOURCE_REQUEST();
            PH.setPack(p142);
            p142.storage_SET(new byte[] {(byte)109, (byte)62, (byte)114, (byte)50, (byte)225, (byte)99, (byte)155, (byte)94, (byte)131, (byte)19, (byte)74, (byte)160, (byte)141, (byte)119, (byte)148, (byte)211, (byte)173, (byte)255, (byte)4, (byte)245, (byte)78, (byte)52, (byte)8, (byte)91, (byte)76, (byte)14, (byte)200, (byte)219, (byte)132, (byte)102, (byte)29, (byte)22, (byte)13, (byte)147, (byte)38, (byte)237, (byte)54, (byte)51, (byte)96, (byte)56, (byte)172, (byte)22, (byte)167, (byte)34, (byte)199, (byte)100, (byte)180, (byte)224, (byte)171, (byte)190, (byte)220, (byte)107, (byte)17, (byte)232, (byte)162, (byte)95, (byte)161, (byte)191, (byte)245, (byte)188, (byte)84, (byte)110, (byte)95, (byte)34, (byte)145, (byte)84, (byte)75, (byte)80, (byte)199, (byte)63, (byte)89, (byte)88, (byte)254, (byte)153, (byte)226, (byte)121, (byte)163, (byte)80, (byte)164, (byte)171, (byte)94, (byte)226, (byte)235, (byte)82, (byte)84, (byte)205, (byte)8, (byte)150, (byte)134, (byte)100, (byte)66, (byte)209, (byte)236, (byte)159, (byte)183, (byte)119, (byte)98, (byte)15, (byte)252, (byte)148, (byte)194, (byte)200, (byte)224, (byte)201, (byte)22, (byte)60, (byte)19, (byte)44, (byte)50, (byte)28, (byte)106, (byte)239, (byte)148, (byte)91, (byte)206, (byte)2, (byte)75, (byte)202, (byte)107, (byte)68}, 0) ;
            p142.transfer_type = (byte)(byte)212;
            p142.uri_SET(new byte[] {(byte)127, (byte)158, (byte)61, (byte)63, (byte)127, (byte)163, (byte)93, (byte)98, (byte)153, (byte)143, (byte)29, (byte)217, (byte)230, (byte)179, (byte)239, (byte)146, (byte)98, (byte)144, (byte)25, (byte)228, (byte)198, (byte)19, (byte)25, (byte)38, (byte)214, (byte)195, (byte)156, (byte)65, (byte)81, (byte)86, (byte)71, (byte)124, (byte)48, (byte)0, (byte)249, (byte)129, (byte)5, (byte)155, (byte)152, (byte)239, (byte)175, (byte)64, (byte)102, (byte)166, (byte)202, (byte)110, (byte)32, (byte)146, (byte)243, (byte)54, (byte)150, (byte)152, (byte)25, (byte)29, (byte)96, (byte)19, (byte)30, (byte)195, (byte)16, (byte)71, (byte)108, (byte)191, (byte)173, (byte)10, (byte)120, (byte)148, (byte)169, (byte)14, (byte)232, (byte)11, (byte)222, (byte)38, (byte)175, (byte)50, (byte)176, (byte)228, (byte)2, (byte)187, (byte)17, (byte)143, (byte)146, (byte)30, (byte)142, (byte)26, (byte)66, (byte)155, (byte)24, (byte)71, (byte)198, (byte)147, (byte)233, (byte)114, (byte)174, (byte)42, (byte)5, (byte)24, (byte)41, (byte)226, (byte)86, (byte)79, (byte)0, (byte)186, (byte)232, (byte)223, (byte)186, (byte)74, (byte)212, (byte)41, (byte)146, (byte)23, (byte)110, (byte)208, (byte)48, (byte)6, (byte)100, (byte)10, (byte)138, (byte)166, (byte)199, (byte)119}, 0) ;
            p142.uri_type = (byte)(byte)224;
            p142.request_id = (byte)(byte)94;
            CommunicationChannel.instance.send(p142);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_PRESSURE3Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.press_abs == (float) -9.474749E37F);
                Debug.Assert(pack.time_boot_ms == (uint)2977135317U);
                Debug.Assert(pack.press_diff == (float) -1.5073634E38F);
                Debug.Assert(pack.temperature == (short)(short)24456);
            };
            GroundControl.SCALED_PRESSURE3 p143 = CommunicationChannel.new_SCALED_PRESSURE3();
            PH.setPack(p143);
            p143.press_diff = (float) -1.5073634E38F;
            p143.temperature = (short)(short)24456;
            p143.press_abs = (float) -9.474749E37F;
            p143.time_boot_ms = (uint)2977135317U;
            CommunicationChannel.instance.send(p143);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnFOLLOW_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.timestamp == (ulong)7592688374770407478L);
                Debug.Assert(pack.custom_state == (ulong)7267214710140767936L);
                Debug.Assert(pack.rates.SequenceEqual(new float[] {-1.9637697E38F, -6.869321E37F, -4.9005014E37F}));
                Debug.Assert(pack.lon == (int)650452155);
                Debug.Assert(pack.acc.SequenceEqual(new float[] {-8.186431E37F, 2.0529307E38F, 1.8309692E38F}));
                Debug.Assert(pack.attitude_q.SequenceEqual(new float[] {8.859144E37F, 3.3556894E37F, 2.2329665E38F, -2.2069391E38F}));
                Debug.Assert(pack.est_capabilities == (byte)(byte)81);
                Debug.Assert(pack.vel.SequenceEqual(new float[] {5.9277446E37F, -3.3375567E38F, 1.7079681E38F}));
                Debug.Assert(pack.position_cov.SequenceEqual(new float[] {2.7292472E37F, 1.8948494E38F, -1.4156619E38F}));
                Debug.Assert(pack.alt == (float)2.2111013E38F);
                Debug.Assert(pack.lat == (int)1087568935);
            };
            GroundControl.FOLLOW_TARGET p144 = CommunicationChannel.new_FOLLOW_TARGET();
            PH.setPack(p144);
            p144.alt = (float)2.2111013E38F;
            p144.lat = (int)1087568935;
            p144.position_cov_SET(new float[] {2.7292472E37F, 1.8948494E38F, -1.4156619E38F}, 0) ;
            p144.attitude_q_SET(new float[] {8.859144E37F, 3.3556894E37F, 2.2329665E38F, -2.2069391E38F}, 0) ;
            p144.custom_state = (ulong)7267214710140767936L;
            p144.vel_SET(new float[] {5.9277446E37F, -3.3375567E38F, 1.7079681E38F}, 0) ;
            p144.est_capabilities = (byte)(byte)81;
            p144.rates_SET(new float[] {-1.9637697E38F, -6.869321E37F, -4.9005014E37F}, 0) ;
            p144.lon = (int)650452155;
            p144.timestamp = (ulong)7592688374770407478L;
            p144.acc_SET(new float[] {-8.186431E37F, 2.0529307E38F, 1.8309692E38F}, 0) ;
            CommunicationChannel.instance.send(p144);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCONTROL_SYSTEM_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.z_pos == (float)1.1546758E38F);
                Debug.Assert(pack.y_pos == (float) -2.1616392E38F);
                Debug.Assert(pack.yaw_rate == (float)3.0177159E38F);
                Debug.Assert(pack.roll_rate == (float) -7.8730407E37F);
                Debug.Assert(pack.z_vel == (float) -4.7883874E37F);
                Debug.Assert(pack.y_vel == (float)3.2621796E38F);
                Debug.Assert(pack.x_acc == (float)3.1974083E38F);
                Debug.Assert(pack.x_vel == (float) -2.2557043E38F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {1.2591925E38F, 7.3929256E37F, -1.7565135E37F, 2.011905E38F}));
                Debug.Assert(pack.pitch_rate == (float)1.3582137E38F);
                Debug.Assert(pack.z_acc == (float)2.6856823E38F);
                Debug.Assert(pack.vel_variance.SequenceEqual(new float[] {-2.9547907E38F, -3.0161158E38F, -3.9491025E37F}));
                Debug.Assert(pack.time_usec == (ulong)7452355887255773490L);
                Debug.Assert(pack.y_acc == (float) -3.0716807E38F);
                Debug.Assert(pack.pos_variance.SequenceEqual(new float[] {-3.00161E38F, 1.2839658E38F, 3.1028876E38F}));
                Debug.Assert(pack.airspeed == (float)3.1933123E38F);
                Debug.Assert(pack.x_pos == (float)2.2957315E38F);
            };
            GroundControl.CONTROL_SYSTEM_STATE p146 = CommunicationChannel.new_CONTROL_SYSTEM_STATE();
            PH.setPack(p146);
            p146.vel_variance_SET(new float[] {-2.9547907E38F, -3.0161158E38F, -3.9491025E37F}, 0) ;
            p146.x_acc = (float)3.1974083E38F;
            p146.yaw_rate = (float)3.0177159E38F;
            p146.pos_variance_SET(new float[] {-3.00161E38F, 1.2839658E38F, 3.1028876E38F}, 0) ;
            p146.z_vel = (float) -4.7883874E37F;
            p146.roll_rate = (float) -7.8730407E37F;
            p146.airspeed = (float)3.1933123E38F;
            p146.z_acc = (float)2.6856823E38F;
            p146.pitch_rate = (float)1.3582137E38F;
            p146.x_pos = (float)2.2957315E38F;
            p146.z_pos = (float)1.1546758E38F;
            p146.y_acc = (float) -3.0716807E38F;
            p146.q_SET(new float[] {1.2591925E38F, 7.3929256E37F, -1.7565135E37F, 2.011905E38F}, 0) ;
            p146.x_vel = (float) -2.2557043E38F;
            p146.y_pos = (float) -2.1616392E38F;
            p146.y_vel = (float)3.2621796E38F;
            p146.time_usec = (ulong)7452355887255773490L;
            CommunicationChannel.instance.send(p146);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnBATTERY_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.voltages.SequenceEqual(new ushort[] {(ushort)30364, (ushort)4599, (ushort)51885, (ushort)63112, (ushort)58004, (ushort)11292, (ushort)21587, (ushort)30312, (ushort)53673, (ushort)51294}));
                Debug.Assert(pack.id == (byte)(byte)31);
                Debug.Assert(pack.current_consumed == (int)75286167);
                Debug.Assert(pack.temperature == (short)(short)26494);
                Debug.Assert(pack.energy_consumed == (int)325887576);
                Debug.Assert(pack.battery_remaining == (sbyte)(sbyte)7);
                Debug.Assert(pack.battery_function == MAV_BATTERY_FUNCTION.MAV_BATTERY_TYPE_PAYLOAD);
                Debug.Assert(pack.current_battery == (short)(short)32266);
                Debug.Assert(pack.type == MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_LIFE);
            };
            GroundControl.BATTERY_STATUS p147 = CommunicationChannel.new_BATTERY_STATUS();
            PH.setPack(p147);
            p147.battery_remaining = (sbyte)(sbyte)7;
            p147.current_battery = (short)(short)32266;
            p147.temperature = (short)(short)26494;
            p147.id = (byte)(byte)31;
            p147.energy_consumed = (int)325887576;
            p147.voltages_SET(new ushort[] {(ushort)30364, (ushort)4599, (ushort)51885, (ushort)63112, (ushort)58004, (ushort)11292, (ushort)21587, (ushort)30312, (ushort)53673, (ushort)51294}, 0) ;
            p147.battery_function = MAV_BATTERY_FUNCTION.MAV_BATTERY_TYPE_PAYLOAD;
            p147.current_consumed = (int)75286167;
            p147.type = MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_LIFE;
            CommunicationChannel.instance.send(p147);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnAUTOPILOT_VERSIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.flight_custom_version.SequenceEqual(new byte[] {(byte)60, (byte)96, (byte)23, (byte)107, (byte)204, (byte)221, (byte)209, (byte)156}));
                Debug.Assert(pack.middleware_custom_version.SequenceEqual(new byte[] {(byte)52, (byte)28, (byte)163, (byte)171, (byte)100, (byte)17, (byte)185, (byte)105}));
                Debug.Assert(pack.os_sw_version == (uint)1644287647U);
                Debug.Assert(pack.capabilities == (MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_COMMAND_INT |
                                                   MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MAVLINK2 |
                                                   MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_FENCE |
                                                   MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_ACTUATOR_TARGET));
                Debug.Assert(pack.product_id == (ushort)(ushort)13085);
                Debug.Assert(pack.vendor_id == (ushort)(ushort)11684);
                Debug.Assert(pack.middleware_sw_version == (uint)523423666U);
                Debug.Assert(pack.os_custom_version.SequenceEqual(new byte[] {(byte)109, (byte)169, (byte)174, (byte)252, (byte)94, (byte)74, (byte)88, (byte)213}));
                Debug.Assert(pack.uid2_TRY(ph).SequenceEqual(new byte[] {(byte)193, (byte)123, (byte)182, (byte)107, (byte)161, (byte)150, (byte)124, (byte)104, (byte)222, (byte)185, (byte)40, (byte)11, (byte)27, (byte)129, (byte)168, (byte)65, (byte)116, (byte)71}));
                Debug.Assert(pack.board_version == (uint)1668821148U);
                Debug.Assert(pack.uid == (ulong)2249752490742146407L);
                Debug.Assert(pack.flight_sw_version == (uint)2009081359U);
            };
            GroundControl.AUTOPILOT_VERSION p148 = CommunicationChannel.new_AUTOPILOT_VERSION();
            PH.setPack(p148);
            p148.vendor_id = (ushort)(ushort)11684;
            p148.uid = (ulong)2249752490742146407L;
            p148.uid2_SET(new byte[] {(byte)193, (byte)123, (byte)182, (byte)107, (byte)161, (byte)150, (byte)124, (byte)104, (byte)222, (byte)185, (byte)40, (byte)11, (byte)27, (byte)129, (byte)168, (byte)65, (byte)116, (byte)71}, 0, PH) ;
            p148.capabilities = (MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_COMMAND_INT |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MAVLINK2 |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_FENCE |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_ACTUATOR_TARGET);
            p148.flight_sw_version = (uint)2009081359U;
            p148.product_id = (ushort)(ushort)13085;
            p148.middleware_custom_version_SET(new byte[] {(byte)52, (byte)28, (byte)163, (byte)171, (byte)100, (byte)17, (byte)185, (byte)105}, 0) ;
            p148.os_custom_version_SET(new byte[] {(byte)109, (byte)169, (byte)174, (byte)252, (byte)94, (byte)74, (byte)88, (byte)213}, 0) ;
            p148.middleware_sw_version = (uint)523423666U;
            p148.flight_custom_version_SET(new byte[] {(byte)60, (byte)96, (byte)23, (byte)107, (byte)204, (byte)221, (byte)209, (byte)156}, 0) ;
            p148.board_version = (uint)1668821148U;
            p148.os_sw_version = (uint)1644287647U;
            CommunicationChannel.instance.send(p148);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLANDING_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.q_TRY(ph).SequenceEqual(new float[] {1.4943803E38F, 9.317988E37F, -1.0926123E38F, -2.1397356E38F}));
                Debug.Assert(pack.distance == (float) -2.007825E38F);
                Debug.Assert(pack.target_num == (byte)(byte)114);
                Debug.Assert(pack.x_TRY(ph) == (float)2.272034E38F);
                Debug.Assert(pack.frame == MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT);
                Debug.Assert(pack.y_TRY(ph) == (float) -2.2310515E38F);
                Debug.Assert(pack.angle_y == (float) -2.2049833E38F);
                Debug.Assert(pack.position_valid_TRY(ph) == (byte)(byte)149);
                Debug.Assert(pack.type == LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_VISION_FIDUCIAL);
                Debug.Assert(pack.angle_x == (float)3.1735274E38F);
                Debug.Assert(pack.size_x == (float)1.0786177E38F);
                Debug.Assert(pack.z_TRY(ph) == (float)3.3606695E38F);
                Debug.Assert(pack.size_y == (float)1.6680717E38F);
                Debug.Assert(pack.time_usec == (ulong)8885491558521907104L);
            };
            GroundControl.LANDING_TARGET p149 = CommunicationChannel.new_LANDING_TARGET();
            PH.setPack(p149);
            p149.angle_x = (float)3.1735274E38F;
            p149.q_SET(new float[] {1.4943803E38F, 9.317988E37F, -1.0926123E38F, -2.1397356E38F}, 0, PH) ;
            p149.x_SET((float)2.272034E38F, PH) ;
            p149.position_valid_SET((byte)(byte)149, PH) ;
            p149.time_usec = (ulong)8885491558521907104L;
            p149.z_SET((float)3.3606695E38F, PH) ;
            p149.y_SET((float) -2.2310515E38F, PH) ;
            p149.distance = (float) -2.007825E38F;
            p149.size_y = (float)1.6680717E38F;
            p149.type = LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_VISION_FIDUCIAL;
            p149.size_x = (float)1.0786177E38F;
            p149.angle_y = (float) -2.2049833E38F;
            p149.target_num = (byte)(byte)114;
            p149.frame = MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT;
            CommunicationChannel.instance.send(p149);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            ADV_TEST_CH.OnFLEXIFUNCTION_SETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)137);
                Debug.Assert(pack.target_system == (byte)(byte)132);
            };
            GroundControl.FLEXIFUNCTION_SET p150 = CommunicationChannel.new_FLEXIFUNCTION_SET();
            PH.setPack(p150);
            p150.target_system = (byte)(byte)132;
            p150.target_component = (byte)(byte)137;
            CommunicationChannel.instance.send(p150);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnFLEXIFUNCTION_READ_REQReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)216);
                Debug.Assert(pack.read_req_type == (short)(short) -30701);
                Debug.Assert(pack.target_component == (byte)(byte)62);
                Debug.Assert(pack.data_index == (short)(short)31326);
            };
            GroundControl.FLEXIFUNCTION_READ_REQ p151 = CommunicationChannel.new_FLEXIFUNCTION_READ_REQ();
            PH.setPack(p151);
            p151.read_req_type = (short)(short) -30701;
            p151.target_component = (byte)(byte)62;
            p151.data_index = (short)(short)31326;
            p151.target_system = (byte)(byte)216;
            CommunicationChannel.instance.send(p151);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnFLEXIFUNCTION_BUFFER_FUNCTIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.data_.SequenceEqual(new sbyte[] {(sbyte)3, (sbyte) - 80, (sbyte) - 73, (sbyte)75, (sbyte) - 92, (sbyte)10, (sbyte) - 9, (sbyte)13, (sbyte) - 73, (sbyte)89, (sbyte) - 81, (sbyte) - 98, (sbyte)65, (sbyte) - 49, (sbyte) - 11, (sbyte)77, (sbyte)77, (sbyte)54, (sbyte) - 99, (sbyte)123, (sbyte)3, (sbyte)53, (sbyte)50, (sbyte) - 43, (sbyte) - 114, (sbyte)54, (sbyte) - 10, (sbyte)20, (sbyte) - 22, (sbyte)9, (sbyte)19, (sbyte) - 57, (sbyte) - 26, (sbyte)97, (sbyte)10, (sbyte)77, (sbyte) - 107, (sbyte) - 46, (sbyte)68, (sbyte)45, (sbyte) - 52, (sbyte) - 5, (sbyte)67, (sbyte) - 97, (sbyte) - 113, (sbyte)52, (sbyte) - 33, (sbyte)38}));
                Debug.Assert(pack.target_system == (byte)(byte)33);
                Debug.Assert(pack.func_index == (ushort)(ushort)7444);
                Debug.Assert(pack.target_component == (byte)(byte)163);
                Debug.Assert(pack.data_size == (ushort)(ushort)46103);
                Debug.Assert(pack.func_count == (ushort)(ushort)61310);
                Debug.Assert(pack.data_address == (ushort)(ushort)41611);
            };
            GroundControl.FLEXIFUNCTION_BUFFER_FUNCTION p152 = CommunicationChannel.new_FLEXIFUNCTION_BUFFER_FUNCTION();
            PH.setPack(p152);
            p152.func_index = (ushort)(ushort)7444;
            p152.data_address = (ushort)(ushort)41611;
            p152.data__SET(new sbyte[] {(sbyte)3, (sbyte) - 80, (sbyte) - 73, (sbyte)75, (sbyte) - 92, (sbyte)10, (sbyte) - 9, (sbyte)13, (sbyte) - 73, (sbyte)89, (sbyte) - 81, (sbyte) - 98, (sbyte)65, (sbyte) - 49, (sbyte) - 11, (sbyte)77, (sbyte)77, (sbyte)54, (sbyte) - 99, (sbyte)123, (sbyte)3, (sbyte)53, (sbyte)50, (sbyte) - 43, (sbyte) - 114, (sbyte)54, (sbyte) - 10, (sbyte)20, (sbyte) - 22, (sbyte)9, (sbyte)19, (sbyte) - 57, (sbyte) - 26, (sbyte)97, (sbyte)10, (sbyte)77, (sbyte) - 107, (sbyte) - 46, (sbyte)68, (sbyte)45, (sbyte) - 52, (sbyte) - 5, (sbyte)67, (sbyte) - 97, (sbyte) - 113, (sbyte)52, (sbyte) - 33, (sbyte)38}, 0) ;
            p152.target_component = (byte)(byte)163;
            p152.target_system = (byte)(byte)33;
            p152.data_size = (ushort)(ushort)46103;
            p152.func_count = (ushort)(ushort)61310;
            CommunicationChannel.instance.send(p152);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnFLEXIFUNCTION_BUFFER_FUNCTION_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)126);
                Debug.Assert(pack.target_system == (byte)(byte)105);
                Debug.Assert(pack.result == (ushort)(ushort)35788);
                Debug.Assert(pack.func_index == (ushort)(ushort)49788);
            };
            GroundControl.FLEXIFUNCTION_BUFFER_FUNCTION_ACK p153 = CommunicationChannel.new_FLEXIFUNCTION_BUFFER_FUNCTION_ACK();
            PH.setPack(p153);
            p153.target_system = (byte)(byte)105;
            p153.target_component = (byte)(byte)126;
            p153.func_index = (ushort)(ushort)49788;
            p153.result = (ushort)(ushort)35788;
            CommunicationChannel.instance.send(p153);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnFLEXIFUNCTION_DIRECTORYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.count == (byte)(byte)140);
                Debug.Assert(pack.directory_data.SequenceEqual(new sbyte[] {(sbyte)64, (sbyte)115, (sbyte) - 53, (sbyte)93, (sbyte) - 81, (sbyte) - 16, (sbyte) - 125, (sbyte)27, (sbyte)113, (sbyte)101, (sbyte)30, (sbyte)3, (sbyte) - 30, (sbyte)62, (sbyte) - 52, (sbyte)3, (sbyte) - 32, (sbyte) - 35, (sbyte) - 32, (sbyte) - 68, (sbyte)39, (sbyte) - 76, (sbyte)42, (sbyte)107, (sbyte)12, (sbyte) - 88, (sbyte) - 20, (sbyte)81, (sbyte)105, (sbyte) - 111, (sbyte)16, (sbyte) - 71, (sbyte)54, (sbyte)18, (sbyte) - 34, (sbyte) - 116, (sbyte)103, (sbyte) - 4, (sbyte) - 99, (sbyte) - 99, (sbyte)75, (sbyte)50, (sbyte)114, (sbyte)110, (sbyte)77, (sbyte)37, (sbyte)62, (sbyte) - 123}));
                Debug.Assert(pack.start_index == (byte)(byte)148);
                Debug.Assert(pack.directory_type == (byte)(byte)133);
                Debug.Assert(pack.target_component == (byte)(byte)229);
                Debug.Assert(pack.target_system == (byte)(byte)144);
            };
            GroundControl.FLEXIFUNCTION_DIRECTORY p155 = CommunicationChannel.new_FLEXIFUNCTION_DIRECTORY();
            PH.setPack(p155);
            p155.count = (byte)(byte)140;
            p155.directory_data_SET(new sbyte[] {(sbyte)64, (sbyte)115, (sbyte) - 53, (sbyte)93, (sbyte) - 81, (sbyte) - 16, (sbyte) - 125, (sbyte)27, (sbyte)113, (sbyte)101, (sbyte)30, (sbyte)3, (sbyte) - 30, (sbyte)62, (sbyte) - 52, (sbyte)3, (sbyte) - 32, (sbyte) - 35, (sbyte) - 32, (sbyte) - 68, (sbyte)39, (sbyte) - 76, (sbyte)42, (sbyte)107, (sbyte)12, (sbyte) - 88, (sbyte) - 20, (sbyte)81, (sbyte)105, (sbyte) - 111, (sbyte)16, (sbyte) - 71, (sbyte)54, (sbyte)18, (sbyte) - 34, (sbyte) - 116, (sbyte)103, (sbyte) - 4, (sbyte) - 99, (sbyte) - 99, (sbyte)75, (sbyte)50, (sbyte)114, (sbyte)110, (sbyte)77, (sbyte)37, (sbyte)62, (sbyte) - 123}, 0) ;
            p155.target_system = (byte)(byte)144;
            p155.directory_type = (byte)(byte)133;
            p155.start_index = (byte)(byte)148;
            p155.target_component = (byte)(byte)229;
            CommunicationChannel.instance.send(p155);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnFLEXIFUNCTION_DIRECTORY_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.result == (ushort)(ushort)35099);
                Debug.Assert(pack.start_index == (byte)(byte)119);
                Debug.Assert(pack.directory_type == (byte)(byte)135);
                Debug.Assert(pack.target_component == (byte)(byte)135);
                Debug.Assert(pack.target_system == (byte)(byte)203);
                Debug.Assert(pack.count == (byte)(byte)248);
            };
            GroundControl.FLEXIFUNCTION_DIRECTORY_ACK p156 = CommunicationChannel.new_FLEXIFUNCTION_DIRECTORY_ACK();
            PH.setPack(p156);
            p156.count = (byte)(byte)248;
            p156.target_component = (byte)(byte)135;
            p156.directory_type = (byte)(byte)135;
            p156.target_system = (byte)(byte)203;
            p156.start_index = (byte)(byte)119;
            p156.result = (ushort)(ushort)35099;
            CommunicationChannel.instance.send(p156);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnFLEXIFUNCTION_COMMANDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)243);
                Debug.Assert(pack.command_type == (byte)(byte)5);
                Debug.Assert(pack.target_system == (byte)(byte)63);
            };
            GroundControl.FLEXIFUNCTION_COMMAND p157 = CommunicationChannel.new_FLEXIFUNCTION_COMMAND();
            PH.setPack(p157);
            p157.target_system = (byte)(byte)63;
            p157.command_type = (byte)(byte)5;
            p157.target_component = (byte)(byte)243;
            CommunicationChannel.instance.send(p157);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnFLEXIFUNCTION_COMMAND_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.command_type == (ushort)(ushort)31902);
                Debug.Assert(pack.result == (ushort)(ushort)54781);
            };
            GroundControl.FLEXIFUNCTION_COMMAND_ACK p158 = CommunicationChannel.new_FLEXIFUNCTION_COMMAND_ACK();
            PH.setPack(p158);
            p158.command_type = (ushort)(ushort)31902;
            p158.result = (ushort)(ushort)54781;
            CommunicationChannel.instance.send(p158);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSERIAL_UDB_EXTRA_F2_AReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sue_rmat4 == (short)(short) -15066);
                Debug.Assert(pack.sue_latitude == (int)380697046);
                Debug.Assert(pack.sue_waypoint_index == (ushort)(ushort)26212);
                Debug.Assert(pack.sue_sog == (short)(short)13672);
                Debug.Assert(pack.sue_rmat2 == (short)(short)9911);
                Debug.Assert(pack.sue_estimated_wind_0 == (short)(short)31686);
                Debug.Assert(pack.sue_rmat7 == (short)(short)4477);
                Debug.Assert(pack.sue_altitude == (int)916573446);
                Debug.Assert(pack.sue_rmat6 == (short)(short)31678);
                Debug.Assert(pack.sue_estimated_wind_2 == (short)(short)26694);
                Debug.Assert(pack.sue_hdop == (short)(short) -17914);
                Debug.Assert(pack.sue_magFieldEarth1 == (short)(short)2960);
                Debug.Assert(pack.sue_rmat8 == (short)(short) -8198);
                Debug.Assert(pack.sue_rmat1 == (short)(short)2289);
                Debug.Assert(pack.sue_time == (uint)1688706163U);
                Debug.Assert(pack.sue_magFieldEarth0 == (short)(short)31229);
                Debug.Assert(pack.sue_longitude == (int)152606978);
                Debug.Assert(pack.sue_svs == (short)(short)23215);
                Debug.Assert(pack.sue_rmat0 == (short)(short)17795);
                Debug.Assert(pack.sue_magFieldEarth2 == (short)(short) -17637);
                Debug.Assert(pack.sue_air_speed_3DIMU == (ushort)(ushort)33763);
                Debug.Assert(pack.sue_rmat5 == (short)(short) -24379);
                Debug.Assert(pack.sue_cog == (ushort)(ushort)28935);
                Debug.Assert(pack.sue_cpu_load == (ushort)(ushort)40405);
                Debug.Assert(pack.sue_rmat3 == (short)(short)24094);
                Debug.Assert(pack.sue_estimated_wind_1 == (short)(short)1524);
                Debug.Assert(pack.sue_status == (byte)(byte)82);
            };
            GroundControl.SERIAL_UDB_EXTRA_F2_A p170 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F2_A();
            PH.setPack(p170);
            p170.sue_latitude = (int)380697046;
            p170.sue_hdop = (short)(short) -17914;
            p170.sue_rmat5 = (short)(short) -24379;
            p170.sue_magFieldEarth1 = (short)(short)2960;
            p170.sue_rmat3 = (short)(short)24094;
            p170.sue_sog = (short)(short)13672;
            p170.sue_estimated_wind_0 = (short)(short)31686;
            p170.sue_rmat8 = (short)(short) -8198;
            p170.sue_svs = (short)(short)23215;
            p170.sue_rmat2 = (short)(short)9911;
            p170.sue_magFieldEarth2 = (short)(short) -17637;
            p170.sue_rmat6 = (short)(short)31678;
            p170.sue_waypoint_index = (ushort)(ushort)26212;
            p170.sue_altitude = (int)916573446;
            p170.sue_time = (uint)1688706163U;
            p170.sue_estimated_wind_2 = (short)(short)26694;
            p170.sue_air_speed_3DIMU = (ushort)(ushort)33763;
            p170.sue_cpu_load = (ushort)(ushort)40405;
            p170.sue_status = (byte)(byte)82;
            p170.sue_longitude = (int)152606978;
            p170.sue_rmat0 = (short)(short)17795;
            p170.sue_rmat4 = (short)(short) -15066;
            p170.sue_rmat1 = (short)(short)2289;
            p170.sue_rmat7 = (short)(short)4477;
            p170.sue_cog = (ushort)(ushort)28935;
            p170.sue_magFieldEarth0 = (short)(short)31229;
            p170.sue_estimated_wind_1 = (short)(short)1524;
            CommunicationChannel.instance.send(p170);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSERIAL_UDB_EXTRA_F2_BReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sue_waypoint_goal_y == (short)(short)13641);
                Debug.Assert(pack.sue_desired_height == (short)(short) -23300);
                Debug.Assert(pack.sue_time == (uint)3346555308U);
                Debug.Assert(pack.sue_barom_press == (int)1080066895);
                Debug.Assert(pack.sue_pwm_input_11 == (short)(short) -1393);
                Debug.Assert(pack.sue_osc_fails == (short)(short) -18029);
                Debug.Assert(pack.sue_barom_alt == (int)1304461520);
                Debug.Assert(pack.sue_memory_stack_free == (short)(short) -31510);
                Debug.Assert(pack.sue_pwm_input_3 == (short)(short)397);
                Debug.Assert(pack.sue_pwm_output_11 == (short)(short)15673);
                Debug.Assert(pack.sue_pwm_input_10 == (short)(short) -1493);
                Debug.Assert(pack.sue_pwm_output_7 == (short)(short)24267);
                Debug.Assert(pack.sue_pwm_input_9 == (short)(short)25518);
                Debug.Assert(pack.sue_location_error_earth_x == (short)(short) -15440);
                Debug.Assert(pack.sue_pwm_output_5 == (short)(short)2379);
                Debug.Assert(pack.sue_pwm_output_12 == (short)(short) -10872);
                Debug.Assert(pack.sue_imu_location_z == (short)(short)18247);
                Debug.Assert(pack.sue_pwm_input_8 == (short)(short)30767);
                Debug.Assert(pack.sue_aero_y == (short)(short) -10099);
                Debug.Assert(pack.sue_bat_amp_hours == (short)(short) -79);
                Debug.Assert(pack.sue_flags == (uint)1584636002U);
                Debug.Assert(pack.sue_pwm_output_8 == (short)(short) -23336);
                Debug.Assert(pack.sue_waypoint_goal_z == (short)(short)22099);
                Debug.Assert(pack.sue_pwm_output_9 == (short)(short)24406);
                Debug.Assert(pack.sue_pwm_input_4 == (short)(short) -7959);
                Debug.Assert(pack.sue_aero_z == (short)(short)10680);
                Debug.Assert(pack.sue_imu_velocity_y == (short)(short)10649);
                Debug.Assert(pack.sue_pwm_input_7 == (short)(short) -5889);
                Debug.Assert(pack.sue_barom_temp == (short)(short) -20607);
                Debug.Assert(pack.sue_pwm_output_6 == (short)(short) -6818);
                Debug.Assert(pack.sue_pwm_output_4 == (short)(short)21214);
                Debug.Assert(pack.sue_waypoint_goal_x == (short)(short)12176);
                Debug.Assert(pack.sue_bat_volt == (short)(short)3495);
                Debug.Assert(pack.sue_pwm_output_10 == (short)(short)17073);
                Debug.Assert(pack.sue_imu_location_x == (short)(short) -25417);
                Debug.Assert(pack.sue_bat_amp == (short)(short)31632);
                Debug.Assert(pack.sue_aero_x == (short)(short) -15727);
                Debug.Assert(pack.sue_pwm_input_2 == (short)(short) -7380);
                Debug.Assert(pack.sue_imu_velocity_x == (short)(short) -23507);
                Debug.Assert(pack.sue_location_error_earth_y == (short)(short)22460);
                Debug.Assert(pack.sue_imu_velocity_z == (short)(short) -8294);
                Debug.Assert(pack.sue_pwm_output_1 == (short)(short) -3474);
                Debug.Assert(pack.sue_imu_location_y == (short)(short) -1720);
                Debug.Assert(pack.sue_pwm_input_12 == (short)(short) -22270);
                Debug.Assert(pack.sue_location_error_earth_z == (short)(short) -27264);
                Debug.Assert(pack.sue_pwm_output_3 == (short)(short) -28969);
                Debug.Assert(pack.sue_pwm_output_2 == (short)(short)9348);
                Debug.Assert(pack.sue_pwm_input_6 == (short)(short) -13589);
                Debug.Assert(pack.sue_pwm_input_5 == (short)(short) -12910);
                Debug.Assert(pack.sue_pwm_input_1 == (short)(short) -25595);
            };
            GroundControl.SERIAL_UDB_EXTRA_F2_B p171 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F2_B();
            PH.setPack(p171);
            p171.sue_pwm_input_9 = (short)(short)25518;
            p171.sue_pwm_output_8 = (short)(short) -23336;
            p171.sue_imu_location_x = (short)(short) -25417;
            p171.sue_pwm_input_12 = (short)(short) -22270;
            p171.sue_aero_z = (short)(short)10680;
            p171.sue_pwm_output_9 = (short)(short)24406;
            p171.sue_bat_volt = (short)(short)3495;
            p171.sue_aero_x = (short)(short) -15727;
            p171.sue_time = (uint)3346555308U;
            p171.sue_pwm_input_5 = (short)(short) -12910;
            p171.sue_imu_location_z = (short)(short)18247;
            p171.sue_pwm_input_1 = (short)(short) -25595;
            p171.sue_desired_height = (short)(short) -23300;
            p171.sue_barom_press = (int)1080066895;
            p171.sue_waypoint_goal_y = (short)(short)13641;
            p171.sue_waypoint_goal_x = (short)(short)12176;
            p171.sue_pwm_input_2 = (short)(short) -7380;
            p171.sue_location_error_earth_z = (short)(short) -27264;
            p171.sue_pwm_input_4 = (short)(short) -7959;
            p171.sue_pwm_output_4 = (short)(short)21214;
            p171.sue_pwm_output_6 = (short)(short) -6818;
            p171.sue_barom_temp = (short)(short) -20607;
            p171.sue_imu_location_y = (short)(short) -1720;
            p171.sue_flags = (uint)1584636002U;
            p171.sue_pwm_output_10 = (short)(short)17073;
            p171.sue_location_error_earth_y = (short)(short)22460;
            p171.sue_memory_stack_free = (short)(short) -31510;
            p171.sue_pwm_output_11 = (short)(short)15673;
            p171.sue_barom_alt = (int)1304461520;
            p171.sue_bat_amp_hours = (short)(short) -79;
            p171.sue_imu_velocity_x = (short)(short) -23507;
            p171.sue_pwm_output_1 = (short)(short) -3474;
            p171.sue_waypoint_goal_z = (short)(short)22099;
            p171.sue_pwm_output_12 = (short)(short) -10872;
            p171.sue_imu_velocity_z = (short)(short) -8294;
            p171.sue_aero_y = (short)(short) -10099;
            p171.sue_location_error_earth_x = (short)(short) -15440;
            p171.sue_bat_amp = (short)(short)31632;
            p171.sue_pwm_input_10 = (short)(short) -1493;
            p171.sue_pwm_input_7 = (short)(short) -5889;
            p171.sue_pwm_output_3 = (short)(short) -28969;
            p171.sue_pwm_output_5 = (short)(short)2379;
            p171.sue_pwm_input_11 = (short)(short) -1393;
            p171.sue_pwm_input_8 = (short)(short)30767;
            p171.sue_osc_fails = (short)(short) -18029;
            p171.sue_pwm_output_2 = (short)(short)9348;
            p171.sue_pwm_output_7 = (short)(short)24267;
            p171.sue_imu_velocity_y = (short)(short)10649;
            p171.sue_pwm_input_3 = (short)(short)397;
            p171.sue_pwm_input_6 = (short)(short) -13589;
            CommunicationChannel.instance.send(p171);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSERIAL_UDB_EXTRA_F4Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sue_YAW_STABILIZATION_RUDDER == (byte)(byte)27);
                Debug.Assert(pack.sue_ALTITUDEHOLD_WAYPOINT == (byte)(byte)66);
                Debug.Assert(pack.sue_PITCH_STABILIZATION == (byte)(byte)249);
                Debug.Assert(pack.sue_RACING_MODE == (byte)(byte)70);
                Debug.Assert(pack.sue_AILERON_NAVIGATION == (byte)(byte)51);
                Debug.Assert(pack.sue_ALTITUDEHOLD_STABILIZED == (byte)(byte)214);
                Debug.Assert(pack.sue_YAW_STABILIZATION_AILERON == (byte)(byte)74);
                Debug.Assert(pack.sue_ROLL_STABILIZATION_AILERONS == (byte)(byte)208);
                Debug.Assert(pack.sue_RUDDER_NAVIGATION == (byte)(byte)77);
                Debug.Assert(pack.sue_ROLL_STABILIZATION_RUDDER == (byte)(byte)39);
            };
            GroundControl.SERIAL_UDB_EXTRA_F4 p172 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F4();
            PH.setPack(p172);
            p172.sue_RACING_MODE = (byte)(byte)70;
            p172.sue_ROLL_STABILIZATION_AILERONS = (byte)(byte)208;
            p172.sue_AILERON_NAVIGATION = (byte)(byte)51;
            p172.sue_ALTITUDEHOLD_STABILIZED = (byte)(byte)214;
            p172.sue_PITCH_STABILIZATION = (byte)(byte)249;
            p172.sue_ROLL_STABILIZATION_RUDDER = (byte)(byte)39;
            p172.sue_ALTITUDEHOLD_WAYPOINT = (byte)(byte)66;
            p172.sue_YAW_STABILIZATION_AILERON = (byte)(byte)74;
            p172.sue_YAW_STABILIZATION_RUDDER = (byte)(byte)27;
            p172.sue_RUDDER_NAVIGATION = (byte)(byte)77;
            CommunicationChannel.instance.send(p172);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSERIAL_UDB_EXTRA_F5Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sue_YAWKD_AILERON == (float) -1.4362275E38F);
                Debug.Assert(pack.sue_ROLLKP == (float) -2.1532407E37F);
                Debug.Assert(pack.sue_YAWKP_AILERON == (float)8.933318E37F);
                Debug.Assert(pack.sue_ROLLKD == (float)2.4181015E38F);
            };
            GroundControl.SERIAL_UDB_EXTRA_F5 p173 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F5();
            PH.setPack(p173);
            p173.sue_ROLLKD = (float)2.4181015E38F;
            p173.sue_YAWKP_AILERON = (float)8.933318E37F;
            p173.sue_YAWKD_AILERON = (float) -1.4362275E38F;
            p173.sue_ROLLKP = (float) -2.1532407E37F;
            CommunicationChannel.instance.send(p173);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSERIAL_UDB_EXTRA_F6Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sue_ELEVATOR_BOOST == (float)1.1395524E38F);
                Debug.Assert(pack.sue_PITCHKD == (float)2.4782607E37F);
                Debug.Assert(pack.sue_ROLL_ELEV_MIX == (float)2.690581E37F);
                Debug.Assert(pack.sue_PITCHGAIN == (float)1.2494259E38F);
                Debug.Assert(pack.sue_RUDDER_ELEV_MIX == (float) -2.9824281E38F);
            };
            GroundControl.SERIAL_UDB_EXTRA_F6 p174 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F6();
            PH.setPack(p174);
            p174.sue_RUDDER_ELEV_MIX = (float) -2.9824281E38F;
            p174.sue_PITCHKD = (float)2.4782607E37F;
            p174.sue_ELEVATOR_BOOST = (float)1.1395524E38F;
            p174.sue_PITCHGAIN = (float)1.2494259E38F;
            p174.sue_ROLL_ELEV_MIX = (float)2.690581E37F;
            CommunicationChannel.instance.send(p174);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSERIAL_UDB_EXTRA_F7Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sue_YAWKP_RUDDER == (float)1.516986E38F);
                Debug.Assert(pack.sue_RTL_PITCH_DOWN == (float)7.1530385E37F);
                Debug.Assert(pack.sue_ROLLKP_RUDDER == (float)1.6543712E38F);
                Debug.Assert(pack.sue_RUDDER_BOOST == (float) -3.3767754E38F);
                Debug.Assert(pack.sue_ROLLKD_RUDDER == (float)1.6035667E38F);
                Debug.Assert(pack.sue_YAWKD_RUDDER == (float)2.9239852E38F);
            };
            GroundControl.SERIAL_UDB_EXTRA_F7 p175 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F7();
            PH.setPack(p175);
            p175.sue_YAWKD_RUDDER = (float)2.9239852E38F;
            p175.sue_RTL_PITCH_DOWN = (float)7.1530385E37F;
            p175.sue_YAWKP_RUDDER = (float)1.516986E38F;
            p175.sue_ROLLKP_RUDDER = (float)1.6543712E38F;
            p175.sue_ROLLKD_RUDDER = (float)1.6035667E38F;
            p175.sue_RUDDER_BOOST = (float) -3.3767754E38F;
            CommunicationChannel.instance.send(p175);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSERIAL_UDB_EXTRA_F8Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sue_ALT_HOLD_THROTTLE_MIN == (float)1.8958784E38F);
                Debug.Assert(pack.sue_ALT_HOLD_PITCH_MAX == (float) -4.743801E37F);
                Debug.Assert(pack.sue_ALT_HOLD_THROTTLE_MAX == (float)2.556081E37F);
                Debug.Assert(pack.sue_HEIGHT_TARGET_MIN == (float)3.0077475E37F);
                Debug.Assert(pack.sue_HEIGHT_TARGET_MAX == (float)4.5444904E37F);
                Debug.Assert(pack.sue_ALT_HOLD_PITCH_MIN == (float) -2.300007E38F);
                Debug.Assert(pack.sue_ALT_HOLD_PITCH_HIGH == (float)1.2259987E38F);
            };
            GroundControl.SERIAL_UDB_EXTRA_F8 p176 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F8();
            PH.setPack(p176);
            p176.sue_ALT_HOLD_THROTTLE_MIN = (float)1.8958784E38F;
            p176.sue_ALT_HOLD_THROTTLE_MAX = (float)2.556081E37F;
            p176.sue_HEIGHT_TARGET_MAX = (float)4.5444904E37F;
            p176.sue_ALT_HOLD_PITCH_MIN = (float) -2.300007E38F;
            p176.sue_ALT_HOLD_PITCH_MAX = (float) -4.743801E37F;
            p176.sue_ALT_HOLD_PITCH_HIGH = (float)1.2259987E38F;
            p176.sue_HEIGHT_TARGET_MIN = (float)3.0077475E37F;
            CommunicationChannel.instance.send(p176);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSERIAL_UDB_EXTRA_F13Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sue_alt_origin == (int) -66708863);
                Debug.Assert(pack.sue_week_no == (short)(short) -26952);
                Debug.Assert(pack.sue_lon_origin == (int)346214775);
                Debug.Assert(pack.sue_lat_origin == (int) -27415685);
            };
            GroundControl.SERIAL_UDB_EXTRA_F13 p177 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F13();
            PH.setPack(p177);
            p177.sue_lat_origin = (int) -27415685;
            p177.sue_alt_origin = (int) -66708863;
            p177.sue_lon_origin = (int)346214775;
            p177.sue_week_no = (short)(short) -26952;
            CommunicationChannel.instance.send(p177);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSERIAL_UDB_EXTRA_F14Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sue_BOARD_TYPE == (byte)(byte)19);
                Debug.Assert(pack.sue_FLIGHT_PLAN_TYPE == (byte)(byte)24);
                Debug.Assert(pack.sue_AIRFRAME == (byte)(byte)44);
                Debug.Assert(pack.sue_WIND_ESTIMATION == (byte)(byte)39);
                Debug.Assert(pack.sue_DR == (byte)(byte)40);
                Debug.Assert(pack.sue_GPS_TYPE == (byte)(byte)115);
                Debug.Assert(pack.sue_RCON == (short)(short) -6023);
                Debug.Assert(pack.sue_TRAP_SOURCE == (uint)1793447530U);
                Debug.Assert(pack.sue_CLOCK_CONFIG == (byte)(byte)131);
                Debug.Assert(pack.sue_TRAP_FLAGS == (short)(short)24272);
                Debug.Assert(pack.sue_osc_fail_count == (short)(short) -17050);
            };
            GroundControl.SERIAL_UDB_EXTRA_F14 p178 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F14();
            PH.setPack(p178);
            p178.sue_FLIGHT_PLAN_TYPE = (byte)(byte)24;
            p178.sue_DR = (byte)(byte)40;
            p178.sue_WIND_ESTIMATION = (byte)(byte)39;
            p178.sue_TRAP_FLAGS = (short)(short)24272;
            p178.sue_RCON = (short)(short) -6023;
            p178.sue_BOARD_TYPE = (byte)(byte)19;
            p178.sue_AIRFRAME = (byte)(byte)44;
            p178.sue_GPS_TYPE = (byte)(byte)115;
            p178.sue_osc_fail_count = (short)(short) -17050;
            p178.sue_TRAP_SOURCE = (uint)1793447530U;
            p178.sue_CLOCK_CONFIG = (byte)(byte)131;
            CommunicationChannel.instance.send(p178);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSERIAL_UDB_EXTRA_F15Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sue_ID_VEHICLE_MODEL_NAME.SequenceEqual(new byte[] {(byte)61, (byte)104, (byte)7, (byte)21, (byte)144, (byte)79, (byte)145, (byte)59, (byte)207, (byte)108, (byte)97, (byte)46, (byte)170, (byte)229, (byte)175, (byte)72, (byte)45, (byte)101, (byte)44, (byte)117, (byte)163, (byte)106, (byte)215, (byte)100, (byte)171, (byte)198, (byte)83, (byte)7, (byte)44, (byte)147, (byte)37, (byte)15, (byte)157, (byte)63, (byte)150, (byte)168, (byte)178, (byte)109, (byte)190, (byte)71}));
                Debug.Assert(pack.sue_ID_VEHICLE_REGISTRATION.SequenceEqual(new byte[] {(byte)113, (byte)167, (byte)152, (byte)197, (byte)85, (byte)39, (byte)60, (byte)201, (byte)207, (byte)233, (byte)240, (byte)61, (byte)37, (byte)186, (byte)9, (byte)173, (byte)248, (byte)195, (byte)227, (byte)164}));
            };
            GroundControl.SERIAL_UDB_EXTRA_F15 p179 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F15();
            PH.setPack(p179);
            p179.sue_ID_VEHICLE_REGISTRATION_SET(new byte[] {(byte)113, (byte)167, (byte)152, (byte)197, (byte)85, (byte)39, (byte)60, (byte)201, (byte)207, (byte)233, (byte)240, (byte)61, (byte)37, (byte)186, (byte)9, (byte)173, (byte)248, (byte)195, (byte)227, (byte)164}, 0) ;
            p179.sue_ID_VEHICLE_MODEL_NAME_SET(new byte[] {(byte)61, (byte)104, (byte)7, (byte)21, (byte)144, (byte)79, (byte)145, (byte)59, (byte)207, (byte)108, (byte)97, (byte)46, (byte)170, (byte)229, (byte)175, (byte)72, (byte)45, (byte)101, (byte)44, (byte)117, (byte)163, (byte)106, (byte)215, (byte)100, (byte)171, (byte)198, (byte)83, (byte)7, (byte)44, (byte)147, (byte)37, (byte)15, (byte)157, (byte)63, (byte)150, (byte)168, (byte)178, (byte)109, (byte)190, (byte)71}, 0) ;
            CommunicationChannel.instance.send(p179);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSERIAL_UDB_EXTRA_F16Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sue_ID_DIY_DRONES_URL.SequenceEqual(new byte[] {(byte)167, (byte)244, (byte)95, (byte)169, (byte)93, (byte)47, (byte)249, (byte)173, (byte)105, (byte)14, (byte)18, (byte)34, (byte)129, (byte)73, (byte)187, (byte)175, (byte)64, (byte)249, (byte)234, (byte)98, (byte)169, (byte)105, (byte)61, (byte)104, (byte)70, (byte)123, (byte)68, (byte)249, (byte)162, (byte)242, (byte)149, (byte)51, (byte)98, (byte)111, (byte)151, (byte)221, (byte)183, (byte)214, (byte)44, (byte)124, (byte)62, (byte)37, (byte)229, (byte)15, (byte)18, (byte)31, (byte)109, (byte)167, (byte)129, (byte)217, (byte)153, (byte)174, (byte)100, (byte)184, (byte)54, (byte)68, (byte)1, (byte)16, (byte)246, (byte)176, (byte)5, (byte)101, (byte)3, (byte)79, (byte)107, (byte)48, (byte)85, (byte)15, (byte)249, (byte)106}));
                Debug.Assert(pack.sue_ID_LEAD_PILOT.SequenceEqual(new byte[] {(byte)21, (byte)32, (byte)130, (byte)176, (byte)166, (byte)189, (byte)50, (byte)212, (byte)100, (byte)175, (byte)187, (byte)227, (byte)173, (byte)169, (byte)191, (byte)176, (byte)235, (byte)193, (byte)138, (byte)167, (byte)139, (byte)220, (byte)232, (byte)253, (byte)82, (byte)2, (byte)140, (byte)105, (byte)6, (byte)120, (byte)159, (byte)140, (byte)166, (byte)61, (byte)254, (byte)91, (byte)212, (byte)232, (byte)80, (byte)169}));
            };
            GroundControl.SERIAL_UDB_EXTRA_F16 p180 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F16();
            PH.setPack(p180);
            p180.sue_ID_DIY_DRONES_URL_SET(new byte[] {(byte)167, (byte)244, (byte)95, (byte)169, (byte)93, (byte)47, (byte)249, (byte)173, (byte)105, (byte)14, (byte)18, (byte)34, (byte)129, (byte)73, (byte)187, (byte)175, (byte)64, (byte)249, (byte)234, (byte)98, (byte)169, (byte)105, (byte)61, (byte)104, (byte)70, (byte)123, (byte)68, (byte)249, (byte)162, (byte)242, (byte)149, (byte)51, (byte)98, (byte)111, (byte)151, (byte)221, (byte)183, (byte)214, (byte)44, (byte)124, (byte)62, (byte)37, (byte)229, (byte)15, (byte)18, (byte)31, (byte)109, (byte)167, (byte)129, (byte)217, (byte)153, (byte)174, (byte)100, (byte)184, (byte)54, (byte)68, (byte)1, (byte)16, (byte)246, (byte)176, (byte)5, (byte)101, (byte)3, (byte)79, (byte)107, (byte)48, (byte)85, (byte)15, (byte)249, (byte)106}, 0) ;
            p180.sue_ID_LEAD_PILOT_SET(new byte[] {(byte)21, (byte)32, (byte)130, (byte)176, (byte)166, (byte)189, (byte)50, (byte)212, (byte)100, (byte)175, (byte)187, (byte)227, (byte)173, (byte)169, (byte)191, (byte)176, (byte)235, (byte)193, (byte)138, (byte)167, (byte)139, (byte)220, (byte)232, (byte)253, (byte)82, (byte)2, (byte)140, (byte)105, (byte)6, (byte)120, (byte)159, (byte)140, (byte)166, (byte)61, (byte)254, (byte)91, (byte)212, (byte)232, (byte)80, (byte)169}, 0) ;
            CommunicationChannel.instance.send(p180);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnALTITUDESReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)2484792761U);
                Debug.Assert(pack.alt_barometric == (int)472741690);
                Debug.Assert(pack.alt_gps == (int)1459981284);
                Debug.Assert(pack.alt_imu == (int)640661791);
                Debug.Assert(pack.alt_range_finder == (int)236772721);
                Debug.Assert(pack.alt_extra == (int) -1236672386);
                Debug.Assert(pack.alt_optical_flow == (int) -412202388);
            };
            GroundControl.ALTITUDES p181 = CommunicationChannel.new_ALTITUDES();
            PH.setPack(p181);
            p181.alt_barometric = (int)472741690;
            p181.time_boot_ms = (uint)2484792761U;
            p181.alt_imu = (int)640661791;
            p181.alt_extra = (int) -1236672386;
            p181.alt_range_finder = (int)236772721;
            p181.alt_gps = (int)1459981284;
            p181.alt_optical_flow = (int) -412202388;
            CommunicationChannel.instance.send(p181);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnAIRSPEEDSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.airspeed_ultrasonic == (short)(short) -20283);
                Debug.Assert(pack.airspeed_pitot == (short)(short) -23176);
                Debug.Assert(pack.aoy == (short)(short) -13990);
                Debug.Assert(pack.time_boot_ms == (uint)2971361902U);
                Debug.Assert(pack.airspeed_hot_wire == (short)(short)16362);
                Debug.Assert(pack.airspeed_imu == (short)(short) -5017);
                Debug.Assert(pack.aoa == (short)(short) -1618);
            };
            GroundControl.AIRSPEEDS p182 = CommunicationChannel.new_AIRSPEEDS();
            PH.setPack(p182);
            p182.airspeed_ultrasonic = (short)(short) -20283;
            p182.aoa = (short)(short) -1618;
            p182.airspeed_imu = (short)(short) -5017;
            p182.airspeed_hot_wire = (short)(short)16362;
            p182.aoy = (short)(short) -13990;
            p182.time_boot_ms = (uint)2971361902U;
            p182.airspeed_pitot = (short)(short) -23176;
            CommunicationChannel.instance.send(p182);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSERIAL_UDB_EXTRA_F17Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sue_feed_forward == (float)2.0638037E38F);
                Debug.Assert(pack.sue_turn_rate_fbw == (float) -9.002712E37F);
                Debug.Assert(pack.sue_turn_rate_nav == (float)4.8793494E37F);
            };
            GroundControl.SERIAL_UDB_EXTRA_F17 p183 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F17();
            PH.setPack(p183);
            p183.sue_feed_forward = (float)2.0638037E38F;
            p183.sue_turn_rate_fbw = (float) -9.002712E37F;
            p183.sue_turn_rate_nav = (float)4.8793494E37F;
            CommunicationChannel.instance.send(p183);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSERIAL_UDB_EXTRA_F18Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.angle_of_attack_inverted == (float) -6.3090585E37F);
                Debug.Assert(pack.angle_of_attack_normal == (float)1.8879425E37F);
                Debug.Assert(pack.elevator_trim_inverted == (float) -2.643944E38F);
                Debug.Assert(pack.reference_speed == (float) -3.0655087E38F);
                Debug.Assert(pack.elevator_trim_normal == (float)1.9340892E38F);
            };
            GroundControl.SERIAL_UDB_EXTRA_F18 p184 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F18();
            PH.setPack(p184);
            p184.reference_speed = (float) -3.0655087E38F;
            p184.elevator_trim_inverted = (float) -2.643944E38F;
            p184.elevator_trim_normal = (float)1.9340892E38F;
            p184.angle_of_attack_inverted = (float) -6.3090585E37F;
            p184.angle_of_attack_normal = (float)1.8879425E37F;
            CommunicationChannel.instance.send(p184);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSERIAL_UDB_EXTRA_F19Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sue_rudder_reversed == (byte)(byte)95);
                Debug.Assert(pack.sue_aileron_reversed == (byte)(byte)8);
                Debug.Assert(pack.sue_rudder_output_channel == (byte)(byte)55);
                Debug.Assert(pack.sue_elevator_output_channel == (byte)(byte)68);
                Debug.Assert(pack.sue_aileron_output_channel == (byte)(byte)84);
                Debug.Assert(pack.sue_throttle_output_channel == (byte)(byte)29);
                Debug.Assert(pack.sue_elevator_reversed == (byte)(byte)75);
                Debug.Assert(pack.sue_throttle_reversed == (byte)(byte)208);
            };
            GroundControl.SERIAL_UDB_EXTRA_F19 p185 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F19();
            PH.setPack(p185);
            p185.sue_aileron_output_channel = (byte)(byte)84;
            p185.sue_throttle_reversed = (byte)(byte)208;
            p185.sue_rudder_output_channel = (byte)(byte)55;
            p185.sue_throttle_output_channel = (byte)(byte)29;
            p185.sue_elevator_output_channel = (byte)(byte)68;
            p185.sue_elevator_reversed = (byte)(byte)75;
            p185.sue_rudder_reversed = (byte)(byte)95;
            p185.sue_aileron_reversed = (byte)(byte)8;
            CommunicationChannel.instance.send(p185);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSERIAL_UDB_EXTRA_F20Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sue_trim_value_input_2 == (short)(short) -28110);
                Debug.Assert(pack.sue_trim_value_input_4 == (short)(short) -30411);
                Debug.Assert(pack.sue_trim_value_input_3 == (short)(short)24651);
                Debug.Assert(pack.sue_number_of_inputs == (byte)(byte)110);
                Debug.Assert(pack.sue_trim_value_input_6 == (short)(short) -5836);
                Debug.Assert(pack.sue_trim_value_input_7 == (short)(short)22945);
                Debug.Assert(pack.sue_trim_value_input_8 == (short)(short) -23998);
                Debug.Assert(pack.sue_trim_value_input_11 == (short)(short)15214);
                Debug.Assert(pack.sue_trim_value_input_10 == (short)(short) -11641);
                Debug.Assert(pack.sue_trim_value_input_5 == (short)(short) -8533);
                Debug.Assert(pack.sue_trim_value_input_12 == (short)(short)21444);
                Debug.Assert(pack.sue_trim_value_input_1 == (short)(short) -8610);
                Debug.Assert(pack.sue_trim_value_input_9 == (short)(short)15470);
            };
            GroundControl.SERIAL_UDB_EXTRA_F20 p186 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F20();
            PH.setPack(p186);
            p186.sue_trim_value_input_9 = (short)(short)15470;
            p186.sue_trim_value_input_8 = (short)(short) -23998;
            p186.sue_trim_value_input_10 = (short)(short) -11641;
            p186.sue_trim_value_input_4 = (short)(short) -30411;
            p186.sue_trim_value_input_7 = (short)(short)22945;
            p186.sue_trim_value_input_3 = (short)(short)24651;
            p186.sue_trim_value_input_12 = (short)(short)21444;
            p186.sue_trim_value_input_6 = (short)(short) -5836;
            p186.sue_trim_value_input_1 = (short)(short) -8610;
            p186.sue_trim_value_input_11 = (short)(short)15214;
            p186.sue_trim_value_input_2 = (short)(short) -28110;
            p186.sue_number_of_inputs = (byte)(byte)110;
            p186.sue_trim_value_input_5 = (short)(short) -8533;
            CommunicationChannel.instance.send(p186);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSERIAL_UDB_EXTRA_F21Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sue_accel_z_offset == (short)(short) -4380);
                Debug.Assert(pack.sue_gyro_y_offset == (short)(short) -17899);
                Debug.Assert(pack.sue_gyro_x_offset == (short)(short) -18622);
                Debug.Assert(pack.sue_accel_x_offset == (short)(short) -3657);
                Debug.Assert(pack.sue_gyro_z_offset == (short)(short)26685);
                Debug.Assert(pack.sue_accel_y_offset == (short)(short)12229);
            };
            GroundControl.SERIAL_UDB_EXTRA_F21 p187 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F21();
            PH.setPack(p187);
            p187.sue_accel_z_offset = (short)(short) -4380;
            p187.sue_gyro_y_offset = (short)(short) -17899;
            p187.sue_accel_y_offset = (short)(short)12229;
            p187.sue_accel_x_offset = (short)(short) -3657;
            p187.sue_gyro_z_offset = (short)(short)26685;
            p187.sue_gyro_x_offset = (short)(short) -18622;
            CommunicationChannel.instance.send(p187);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSERIAL_UDB_EXTRA_F22Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sue_accel_y_at_calibration == (short)(short) -31661);
                Debug.Assert(pack.sue_accel_z_at_calibration == (short)(short)16207);
                Debug.Assert(pack.sue_gyro_y_at_calibration == (short)(short) -6101);
                Debug.Assert(pack.sue_gyro_z_at_calibration == (short)(short)16520);
                Debug.Assert(pack.sue_gyro_x_at_calibration == (short)(short)8049);
                Debug.Assert(pack.sue_accel_x_at_calibration == (short)(short) -14515);
            };
            GroundControl.SERIAL_UDB_EXTRA_F22 p188 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F22();
            PH.setPack(p188);
            p188.sue_accel_y_at_calibration = (short)(short) -31661;
            p188.sue_gyro_x_at_calibration = (short)(short)8049;
            p188.sue_accel_x_at_calibration = (short)(short) -14515;
            p188.sue_gyro_z_at_calibration = (short)(short)16520;
            p188.sue_accel_z_at_calibration = (short)(short)16207;
            p188.sue_gyro_y_at_calibration = (short)(short) -6101;
            CommunicationChannel.instance.send(p188);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            CommunicationChannel.instance.OnESTIMATOR_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vel_ratio == (float) -2.6267814E38F);
                Debug.Assert(pack.pos_vert_ratio == (float) -5.8715253E37F);
                Debug.Assert(pack.time_usec == (ulong)5656376898200580543L);
                Debug.Assert(pack.hagl_ratio == (float) -2.5307512E38F);
                Debug.Assert(pack.pos_horiz_accuracy == (float)3.0313752E37F);
                Debug.Assert(pack.mag_ratio == (float) -3.1500192E37F);
                Debug.Assert(pack.pos_vert_accuracy == (float) -1.3263796E38F);
                Debug.Assert(pack.pos_horiz_ratio == (float) -2.785298E38F);
                Debug.Assert(pack.flags == (ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_VERT_AGL |
                                            ESTIMATOR_STATUS_FLAGS.ESTIMATOR_PRED_POS_HORIZ_ABS |
                                            ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_VERT_ABS |
                                            ESTIMATOR_STATUS_FLAGS.ESTIMATOR_PRED_POS_HORIZ_REL));
                Debug.Assert(pack.tas_ratio == (float)2.4453408E38F);
            };
            GroundControl.ESTIMATOR_STATUS p230 = CommunicationChannel.new_ESTIMATOR_STATUS();
            PH.setPack(p230);
            p230.pos_horiz_accuracy = (float)3.0313752E37F;
            p230.time_usec = (ulong)5656376898200580543L;
            p230.hagl_ratio = (float) -2.5307512E38F;
            p230.pos_vert_ratio = (float) -5.8715253E37F;
            p230.mag_ratio = (float) -3.1500192E37F;
            p230.pos_horiz_ratio = (float) -2.785298E38F;
            p230.flags = (ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_VERT_AGL |
                          ESTIMATOR_STATUS_FLAGS.ESTIMATOR_PRED_POS_HORIZ_ABS |
                          ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_VERT_ABS |
                          ESTIMATOR_STATUS_FLAGS.ESTIMATOR_PRED_POS_HORIZ_REL);
            p230.tas_ratio = (float)2.4453408E38F;
            p230.pos_vert_accuracy = (float) -1.3263796E38F;
            p230.vel_ratio = (float) -2.6267814E38F;
            CommunicationChannel.instance.send(p230);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnWIND_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)983104539174142199L);
                Debug.Assert(pack.vert_accuracy == (float)6.0649283E37F);
                Debug.Assert(pack.wind_x == (float)1.5805502E38F);
                Debug.Assert(pack.wind_alt == (float)1.6926054E38F);
                Debug.Assert(pack.wind_z == (float) -1.0771409E38F);
                Debug.Assert(pack.horiz_accuracy == (float) -2.662557E38F);
                Debug.Assert(pack.var_vert == (float) -2.6646513E38F);
                Debug.Assert(pack.var_horiz == (float) -4.5526165E37F);
                Debug.Assert(pack.wind_y == (float) -3.221944E38F);
            };
            GroundControl.WIND_COV p231 = CommunicationChannel.new_WIND_COV();
            PH.setPack(p231);
            p231.var_vert = (float) -2.6646513E38F;
            p231.vert_accuracy = (float)6.0649283E37F;
            p231.var_horiz = (float) -4.5526165E37F;
            p231.time_usec = (ulong)983104539174142199L;
            p231.wind_z = (float) -1.0771409E38F;
            p231.wind_alt = (float)1.6926054E38F;
            p231.wind_x = (float)1.5805502E38F;
            p231.wind_y = (float) -3.221944E38F;
            p231.horiz_accuracy = (float) -2.662557E38F;
            CommunicationChannel.instance.send(p231);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_INPUTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ve == (float)1.1014899E38F);
                Debug.Assert(pack.lon == (int) -1706044152);
                Debug.Assert(pack.ignore_flags == (GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VDOP |
                                                   GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_ALT |
                                                   GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VEL_HORIZ));
                Debug.Assert(pack.horiz_accuracy == (float) -2.5182163E38F);
                Debug.Assert(pack.satellites_visible == (byte)(byte)191);
                Debug.Assert(pack.time_week == (ushort)(ushort)50212);
                Debug.Assert(pack.speed_accuracy == (float) -1.1895039E38F);
                Debug.Assert(pack.vn == (float) -1.3775729E38F);
                Debug.Assert(pack.vert_accuracy == (float)2.468399E37F);
                Debug.Assert(pack.time_week_ms == (uint)1908177397U);
                Debug.Assert(pack.alt == (float)1.4149435E38F);
                Debug.Assert(pack.lat == (int) -531956001);
                Debug.Assert(pack.vdop == (float)1.9426918E38F);
                Debug.Assert(pack.gps_id == (byte)(byte)142);
                Debug.Assert(pack.vd == (float) -9.538682E37F);
                Debug.Assert(pack.time_usec == (ulong)7603848248561256661L);
                Debug.Assert(pack.hdop == (float)2.5567815E38F);
                Debug.Assert(pack.fix_type == (byte)(byte)202);
            };
            GroundControl.GPS_INPUT p232 = CommunicationChannel.new_GPS_INPUT();
            PH.setPack(p232);
            p232.gps_id = (byte)(byte)142;
            p232.vert_accuracy = (float)2.468399E37F;
            p232.alt = (float)1.4149435E38F;
            p232.satellites_visible = (byte)(byte)191;
            p232.vn = (float) -1.3775729E38F;
            p232.horiz_accuracy = (float) -2.5182163E38F;
            p232.vdop = (float)1.9426918E38F;
            p232.vd = (float) -9.538682E37F;
            p232.lat = (int) -531956001;
            p232.ignore_flags = (GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VDOP |
                                 GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_ALT |
                                 GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VEL_HORIZ);
            p232.hdop = (float)2.5567815E38F;
            p232.fix_type = (byte)(byte)202;
            p232.time_usec = (ulong)7603848248561256661L;
            p232.speed_accuracy = (float) -1.1895039E38F;
            p232.time_week_ms = (uint)1908177397U;
            p232.lon = (int) -1706044152;
            p232.ve = (float)1.1014899E38F;
            p232.time_week = (ushort)(ushort)50212;
            CommunicationChannel.instance.send(p232);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_RTCM_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.flags == (byte)(byte)111);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)31, (byte)90, (byte)41, (byte)140, (byte)58, (byte)48, (byte)144, (byte)66, (byte)162, (byte)223, (byte)46, (byte)111, (byte)136, (byte)214, (byte)125, (byte)167, (byte)185, (byte)174, (byte)16, (byte)102, (byte)120, (byte)37, (byte)90, (byte)12, (byte)164, (byte)86, (byte)150, (byte)24, (byte)178, (byte)42, (byte)226, (byte)54, (byte)189, (byte)44, (byte)216, (byte)246, (byte)178, (byte)139, (byte)115, (byte)246, (byte)235, (byte)30, (byte)198, (byte)235, (byte)56, (byte)192, (byte)13, (byte)106, (byte)154, (byte)114, (byte)40, (byte)164, (byte)173, (byte)24, (byte)228, (byte)186, (byte)209, (byte)142, (byte)114, (byte)52, (byte)32, (byte)249, (byte)235, (byte)103, (byte)224, (byte)98, (byte)217, (byte)215, (byte)24, (byte)228, (byte)69, (byte)168, (byte)221, (byte)244, (byte)184, (byte)23, (byte)44, (byte)132, (byte)199, (byte)96, (byte)202, (byte)98, (byte)89, (byte)119, (byte)194, (byte)53, (byte)176, (byte)101, (byte)205, (byte)234, (byte)149, (byte)31, (byte)107, (byte)228, (byte)158, (byte)39, (byte)193, (byte)12, (byte)247, (byte)79, (byte)189, (byte)133, (byte)253, (byte)91, (byte)36, (byte)200, (byte)90, (byte)208, (byte)44, (byte)77, (byte)63, (byte)63, (byte)167, (byte)130, (byte)22, (byte)191, (byte)121, (byte)205, (byte)204, (byte)217, (byte)183, (byte)245, (byte)112, (byte)21, (byte)44, (byte)72, (byte)47, (byte)159, (byte)197, (byte)152, (byte)38, (byte)132, (byte)179, (byte)209, (byte)255, (byte)99, (byte)102, (byte)136, (byte)113, (byte)79, (byte)208, (byte)189, (byte)52, (byte)91, (byte)106, (byte)24, (byte)183, (byte)39, (byte)126, (byte)255, (byte)255, (byte)135, (byte)254, (byte)95, (byte)214, (byte)184, (byte)19, (byte)130, (byte)28, (byte)31, (byte)17, (byte)69, (byte)234, (byte)221, (byte)211, (byte)209, (byte)59, (byte)143, (byte)203, (byte)129, (byte)220, (byte)28, (byte)241, (byte)108, (byte)35, (byte)164, (byte)202, (byte)112, (byte)11, (byte)110}));
                Debug.Assert(pack.len == (byte)(byte)217);
            };
            GroundControl.GPS_RTCM_DATA p233 = CommunicationChannel.new_GPS_RTCM_DATA();
            PH.setPack(p233);
            p233.len = (byte)(byte)217;
            p233.data__SET(new byte[] {(byte)31, (byte)90, (byte)41, (byte)140, (byte)58, (byte)48, (byte)144, (byte)66, (byte)162, (byte)223, (byte)46, (byte)111, (byte)136, (byte)214, (byte)125, (byte)167, (byte)185, (byte)174, (byte)16, (byte)102, (byte)120, (byte)37, (byte)90, (byte)12, (byte)164, (byte)86, (byte)150, (byte)24, (byte)178, (byte)42, (byte)226, (byte)54, (byte)189, (byte)44, (byte)216, (byte)246, (byte)178, (byte)139, (byte)115, (byte)246, (byte)235, (byte)30, (byte)198, (byte)235, (byte)56, (byte)192, (byte)13, (byte)106, (byte)154, (byte)114, (byte)40, (byte)164, (byte)173, (byte)24, (byte)228, (byte)186, (byte)209, (byte)142, (byte)114, (byte)52, (byte)32, (byte)249, (byte)235, (byte)103, (byte)224, (byte)98, (byte)217, (byte)215, (byte)24, (byte)228, (byte)69, (byte)168, (byte)221, (byte)244, (byte)184, (byte)23, (byte)44, (byte)132, (byte)199, (byte)96, (byte)202, (byte)98, (byte)89, (byte)119, (byte)194, (byte)53, (byte)176, (byte)101, (byte)205, (byte)234, (byte)149, (byte)31, (byte)107, (byte)228, (byte)158, (byte)39, (byte)193, (byte)12, (byte)247, (byte)79, (byte)189, (byte)133, (byte)253, (byte)91, (byte)36, (byte)200, (byte)90, (byte)208, (byte)44, (byte)77, (byte)63, (byte)63, (byte)167, (byte)130, (byte)22, (byte)191, (byte)121, (byte)205, (byte)204, (byte)217, (byte)183, (byte)245, (byte)112, (byte)21, (byte)44, (byte)72, (byte)47, (byte)159, (byte)197, (byte)152, (byte)38, (byte)132, (byte)179, (byte)209, (byte)255, (byte)99, (byte)102, (byte)136, (byte)113, (byte)79, (byte)208, (byte)189, (byte)52, (byte)91, (byte)106, (byte)24, (byte)183, (byte)39, (byte)126, (byte)255, (byte)255, (byte)135, (byte)254, (byte)95, (byte)214, (byte)184, (byte)19, (byte)130, (byte)28, (byte)31, (byte)17, (byte)69, (byte)234, (byte)221, (byte)211, (byte)209, (byte)59, (byte)143, (byte)203, (byte)129, (byte)220, (byte)28, (byte)241, (byte)108, (byte)35, (byte)164, (byte)202, (byte)112, (byte)11, (byte)110}, 0) ;
            p233.flags = (byte)(byte)111;
            CommunicationChannel.instance.send(p233);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIGH_LATENCYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.climb_rate == (sbyte)(sbyte) - 66);
                Debug.Assert(pack.altitude_sp == (short)(short) -26181);
                Debug.Assert(pack.landed_state == MAV_LANDED_STATE.MAV_LANDED_STATE_TAKEOFF);
                Debug.Assert(pack.airspeed == (byte)(byte)67);
                Debug.Assert(pack.pitch == (short)(short) -1269);
                Debug.Assert(pack.failsafe == (byte)(byte)210);
                Debug.Assert(pack.latitude == (int)1346673767);
                Debug.Assert(pack.battery_remaining == (byte)(byte)203);
                Debug.Assert(pack.groundspeed == (byte)(byte)211);
                Debug.Assert(pack.heading_sp == (short)(short) -31912);
                Debug.Assert(pack.temperature_air == (sbyte)(sbyte)47);
                Debug.Assert(pack.airspeed_sp == (byte)(byte)27);
                Debug.Assert(pack.wp_num == (byte)(byte)59);
                Debug.Assert(pack.custom_mode == (uint)1405375513U);
                Debug.Assert(pack.wp_distance == (ushort)(ushort)58823);
                Debug.Assert(pack.altitude_amsl == (short)(short)6305);
                Debug.Assert(pack.temperature == (sbyte)(sbyte)22);
                Debug.Assert(pack.roll == (short)(short) -13364);
                Debug.Assert(pack.gps_fix_type == GPS_FIX_TYPE.GPS_FIX_TYPE_NO_FIX);
                Debug.Assert(pack.throttle == (sbyte)(sbyte) - 51);
                Debug.Assert(pack.base_mode == (MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED |
                                                MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED |
                                                MAV_MODE_FLAG.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED));
                Debug.Assert(pack.heading == (ushort)(ushort)3351);
                Debug.Assert(pack.longitude == (int) -157395308);
                Debug.Assert(pack.gps_nsat == (byte)(byte)179);
            };
            GroundControl.HIGH_LATENCY p234 = CommunicationChannel.new_HIGH_LATENCY();
            PH.setPack(p234);
            p234.battery_remaining = (byte)(byte)203;
            p234.gps_nsat = (byte)(byte)179;
            p234.groundspeed = (byte)(byte)211;
            p234.climb_rate = (sbyte)(sbyte) - 66;
            p234.gps_fix_type = GPS_FIX_TYPE.GPS_FIX_TYPE_NO_FIX;
            p234.heading_sp = (short)(short) -31912;
            p234.airspeed_sp = (byte)(byte)27;
            p234.longitude = (int) -157395308;
            p234.base_mode = (MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED |
                              MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED |
                              MAV_MODE_FLAG.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED);
            p234.altitude_amsl = (short)(short)6305;
            p234.roll = (short)(short) -13364;
            p234.wp_num = (byte)(byte)59;
            p234.latitude = (int)1346673767;
            p234.heading = (ushort)(ushort)3351;
            p234.throttle = (sbyte)(sbyte) - 51;
            p234.airspeed = (byte)(byte)67;
            p234.altitude_sp = (short)(short) -26181;
            p234.temperature_air = (sbyte)(sbyte)47;
            p234.pitch = (short)(short) -1269;
            p234.landed_state = MAV_LANDED_STATE.MAV_LANDED_STATE_TAKEOFF;
            p234.custom_mode = (uint)1405375513U;
            p234.temperature = (sbyte)(sbyte)22;
            p234.failsafe = (byte)(byte)210;
            p234.wp_distance = (ushort)(ushort)58823;
            CommunicationChannel.instance.send(p234);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnVIBRATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vibration_y == (float) -1.0364714E38F);
                Debug.Assert(pack.clipping_1 == (uint)2235125740U);
                Debug.Assert(pack.vibration_x == (float)3.3404932E38F);
                Debug.Assert(pack.clipping_0 == (uint)1489319275U);
                Debug.Assert(pack.clipping_2 == (uint)3341419681U);
                Debug.Assert(pack.vibration_z == (float)2.8225756E38F);
                Debug.Assert(pack.time_usec == (ulong)4424868105632448734L);
            };
            GroundControl.VIBRATION p241 = CommunicationChannel.new_VIBRATION();
            PH.setPack(p241);
            p241.vibration_y = (float) -1.0364714E38F;
            p241.vibration_x = (float)3.3404932E38F;
            p241.time_usec = (ulong)4424868105632448734L;
            p241.clipping_2 = (uint)3341419681U;
            p241.clipping_0 = (uint)1489319275U;
            p241.vibration_z = (float)2.8225756E38F;
            p241.clipping_1 = (uint)2235125740U;
            CommunicationChannel.instance.send(p241);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHOME_POSITIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.altitude == (int) -728286635);
                Debug.Assert(pack.approach_y == (float) -1.4686186E38F);
                Debug.Assert(pack.approach_x == (float)1.6727564E38F);
                Debug.Assert(pack.approach_z == (float)1.3682008E38F);
                Debug.Assert(pack.x == (float) -1.431634E38F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-2.4741264E38F, 8.944573E37F, -3.1191227E38F, 3.592356E37F}));
                Debug.Assert(pack.y == (float)1.1005049E38F);
                Debug.Assert(pack.longitude == (int)209533257);
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)7636251242621859132L);
                Debug.Assert(pack.latitude == (int)1144110499);
                Debug.Assert(pack.z == (float)7.2058706E37F);
            };
            GroundControl.HOME_POSITION p242 = CommunicationChannel.new_HOME_POSITION();
            PH.setPack(p242);
            p242.time_usec_SET((ulong)7636251242621859132L, PH) ;
            p242.approach_y = (float) -1.4686186E38F;
            p242.altitude = (int) -728286635;
            p242.x = (float) -1.431634E38F;
            p242.approach_x = (float)1.6727564E38F;
            p242.approach_z = (float)1.3682008E38F;
            p242.z = (float)7.2058706E37F;
            p242.q_SET(new float[] {-2.4741264E38F, 8.944573E37F, -3.1191227E38F, 3.592356E37F}, 0) ;
            p242.y = (float)1.1005049E38F;
            p242.latitude = (int)1144110499;
            p242.longitude = (int)209533257;
            CommunicationChannel.instance.send(p242);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_HOME_POSITIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.y == (float)2.7306068E38F);
                Debug.Assert(pack.z == (float)2.6483107E38F);
                Debug.Assert(pack.approach_x == (float) -2.0478601E38F);
                Debug.Assert(pack.latitude == (int) -895827563);
                Debug.Assert(pack.approach_y == (float) -1.19884E38F);
                Debug.Assert(pack.target_system == (byte)(byte)4);
                Debug.Assert(pack.approach_z == (float)2.6582449E38F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {1.5506501E38F, -1.9713516E38F, -6.5515514E37F, -3.3355646E38F}));
                Debug.Assert(pack.x == (float) -3.2398711E38F);
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)7240824485517605809L);
                Debug.Assert(pack.altitude == (int)772577702);
                Debug.Assert(pack.longitude == (int)1675891923);
            };
            GroundControl.SET_HOME_POSITION p243 = CommunicationChannel.new_SET_HOME_POSITION();
            PH.setPack(p243);
            p243.z = (float)2.6483107E38F;
            p243.x = (float) -3.2398711E38F;
            p243.target_system = (byte)(byte)4;
            p243.q_SET(new float[] {1.5506501E38F, -1.9713516E38F, -6.5515514E37F, -3.3355646E38F}, 0) ;
            p243.y = (float)2.7306068E38F;
            p243.approach_y = (float) -1.19884E38F;
            p243.altitude = (int)772577702;
            p243.approach_z = (float)2.6582449E38F;
            p243.latitude = (int) -895827563;
            p243.approach_x = (float) -2.0478601E38F;
            p243.longitude = (int)1675891923;
            p243.time_usec_SET((ulong)7240824485517605809L, PH) ;
            CommunicationChannel.instance.send(p243);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            ADV_TEST_CH.OnMESSAGE_INTERVALReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.interval_us == (int)884661776);
                Debug.Assert(pack.message_id == (ushort)(ushort)9630);
            };
            GroundControl.MESSAGE_INTERVAL p244 = CommunicationChannel.new_MESSAGE_INTERVAL();
            PH.setPack(p244);
            p244.interval_us = (int)884661776;
            p244.message_id = (ushort)(ushort)9630;
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
                Debug.Assert(pack.tslc == (byte)(byte)169);
                Debug.Assert(pack.flags == (ADSB_FLAGS.ADSB_FLAGS_VALID_VELOCITY |
                                            ADSB_FLAGS.ADSB_FLAGS_VALID_HEADING));
                Debug.Assert(pack.ICAO_address == (uint)3219355139U);
                Debug.Assert(pack.heading == (ushort)(ushort)56136);
                Debug.Assert(pack.lon == (int)1285494171);
                Debug.Assert(pack.squawk == (ushort)(ushort)47473);
                Debug.Assert(pack.emitter_type == ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_SPACE);
                Debug.Assert(pack.hor_velocity == (ushort)(ushort)37879);
                Debug.Assert(pack.callsign_LEN(ph) == 6);
                Debug.Assert(pack.callsign_TRY(ph).Equals("ximflt"));
                Debug.Assert(pack.altitude_type == ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_GEOMETRIC);
                Debug.Assert(pack.lat == (int)1661167368);
                Debug.Assert(pack.ver_velocity == (short)(short)3670);
                Debug.Assert(pack.altitude == (int)708357380);
            };
            GroundControl.ADSB_VEHICLE p246 = CommunicationChannel.new_ADSB_VEHICLE();
            PH.setPack(p246);
            p246.lon = (int)1285494171;
            p246.ver_velocity = (short)(short)3670;
            p246.tslc = (byte)(byte)169;
            p246.ICAO_address = (uint)3219355139U;
            p246.altitude_type = ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_GEOMETRIC;
            p246.lat = (int)1661167368;
            p246.hor_velocity = (ushort)(ushort)37879;
            p246.squawk = (ushort)(ushort)47473;
            p246.heading = (ushort)(ushort)56136;
            p246.callsign_SET("ximflt", PH) ;
            p246.altitude = (int)708357380;
            p246.emitter_type = ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_SPACE;
            p246.flags = (ADSB_FLAGS.ADSB_FLAGS_VALID_VELOCITY |
                          ADSB_FLAGS.ADSB_FLAGS_VALID_HEADING);
            CommunicationChannel.instance.send(p246);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnCOLLISIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.horizontal_minimum_delta == (float) -3.0191578E38F);
                Debug.Assert(pack.threat_level == MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_HIGH);
                Debug.Assert(pack.src_ == MAV_COLLISION_SRC.MAV_COLLISION_SRC_ADSB);
                Debug.Assert(pack.time_to_minimum_delta == (float)3.2601058E37F);
                Debug.Assert(pack.action == MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_NONE);
                Debug.Assert(pack.altitude_minimum_delta == (float)1.7436753E38F);
                Debug.Assert(pack.id == (uint)3621125432U);
            };
            GroundControl.COLLISION p247 = CommunicationChannel.new_COLLISION();
            PH.setPack(p247);
            p247.threat_level = MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_HIGH;
            p247.altitude_minimum_delta = (float)1.7436753E38F;
            p247.id = (uint)3621125432U;
            p247.time_to_minimum_delta = (float)3.2601058E37F;
            p247.src_ = MAV_COLLISION_SRC.MAV_COLLISION_SRC_ADSB;
            p247.action = MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_NONE;
            p247.horizontal_minimum_delta = (float) -3.0191578E38F;
            CommunicationChannel.instance.send(p247);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnV2_EXTENSIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)197);
                Debug.Assert(pack.payload.SequenceEqual(new byte[] {(byte)44, (byte)88, (byte)86, (byte)111, (byte)225, (byte)71, (byte)29, (byte)148, (byte)240, (byte)121, (byte)114, (byte)218, (byte)187, (byte)23, (byte)135, (byte)38, (byte)96, (byte)96, (byte)179, (byte)162, (byte)194, (byte)148, (byte)194, (byte)116, (byte)214, (byte)46, (byte)186, (byte)36, (byte)89, (byte)96, (byte)19, (byte)40, (byte)78, (byte)158, (byte)112, (byte)210, (byte)93, (byte)192, (byte)60, (byte)13, (byte)228, (byte)251, (byte)249, (byte)90, (byte)34, (byte)154, (byte)191, (byte)181, (byte)230, (byte)225, (byte)73, (byte)206, (byte)80, (byte)180, (byte)128, (byte)222, (byte)179, (byte)233, (byte)104, (byte)24, (byte)64, (byte)75, (byte)80, (byte)20, (byte)20, (byte)38, (byte)116, (byte)172, (byte)232, (byte)200, (byte)193, (byte)108, (byte)21, (byte)182, (byte)34, (byte)148, (byte)100, (byte)137, (byte)157, (byte)22, (byte)251, (byte)169, (byte)108, (byte)110, (byte)36, (byte)247, (byte)226, (byte)40, (byte)98, (byte)152, (byte)24, (byte)249, (byte)133, (byte)206, (byte)109, (byte)62, (byte)223, (byte)175, (byte)167, (byte)135, (byte)44, (byte)255, (byte)131, (byte)30, (byte)189, (byte)23, (byte)234, (byte)63, (byte)83, (byte)166, (byte)162, (byte)139, (byte)154, (byte)149, (byte)18, (byte)170, (byte)12, (byte)137, (byte)0, (byte)166, (byte)16, (byte)142, (byte)11, (byte)89, (byte)51, (byte)2, (byte)6, (byte)27, (byte)121, (byte)76, (byte)105, (byte)235, (byte)100, (byte)100, (byte)154, (byte)156, (byte)192, (byte)67, (byte)230, (byte)125, (byte)0, (byte)120, (byte)110, (byte)234, (byte)114, (byte)107, (byte)148, (byte)151, (byte)140, (byte)253, (byte)52, (byte)94, (byte)246, (byte)27, (byte)98, (byte)244, (byte)136, (byte)196, (byte)135, (byte)205, (byte)190, (byte)14, (byte)112, (byte)209, (byte)238, (byte)62, (byte)143, (byte)139, (byte)7, (byte)123, (byte)229, (byte)214, (byte)55, (byte)60, (byte)51, (byte)82, (byte)111, (byte)153, (byte)57, (byte)191, (byte)10, (byte)10, (byte)172, (byte)17, (byte)159, (byte)141, (byte)184, (byte)179, (byte)142, (byte)9, (byte)91, (byte)45, (byte)35, (byte)39, (byte)216, (byte)24, (byte)71, (byte)232, (byte)109, (byte)92, (byte)212, (byte)198, (byte)17, (byte)84, (byte)106, (byte)150, (byte)52, (byte)72, (byte)193, (byte)246, (byte)98, (byte)87, (byte)228, (byte)43, (byte)63, (byte)186, (byte)142, (byte)156, (byte)197, (byte)197, (byte)0, (byte)239, (byte)253, (byte)151, (byte)48, (byte)19, (byte)108, (byte)88, (byte)59, (byte)26, (byte)216, (byte)149, (byte)244, (byte)115, (byte)128, (byte)10, (byte)181, (byte)45, (byte)35, (byte)83, (byte)20, (byte)157, (byte)74, (byte)225, (byte)51, (byte)89, (byte)250, (byte)190, (byte)21}));
                Debug.Assert(pack.target_component == (byte)(byte)68);
                Debug.Assert(pack.target_network == (byte)(byte)233);
                Debug.Assert(pack.message_type == (ushort)(ushort)54402);
            };
            GroundControl.V2_EXTENSION p248 = CommunicationChannel.new_V2_EXTENSION();
            PH.setPack(p248);
            p248.target_component = (byte)(byte)68;
            p248.message_type = (ushort)(ushort)54402;
            p248.payload_SET(new byte[] {(byte)44, (byte)88, (byte)86, (byte)111, (byte)225, (byte)71, (byte)29, (byte)148, (byte)240, (byte)121, (byte)114, (byte)218, (byte)187, (byte)23, (byte)135, (byte)38, (byte)96, (byte)96, (byte)179, (byte)162, (byte)194, (byte)148, (byte)194, (byte)116, (byte)214, (byte)46, (byte)186, (byte)36, (byte)89, (byte)96, (byte)19, (byte)40, (byte)78, (byte)158, (byte)112, (byte)210, (byte)93, (byte)192, (byte)60, (byte)13, (byte)228, (byte)251, (byte)249, (byte)90, (byte)34, (byte)154, (byte)191, (byte)181, (byte)230, (byte)225, (byte)73, (byte)206, (byte)80, (byte)180, (byte)128, (byte)222, (byte)179, (byte)233, (byte)104, (byte)24, (byte)64, (byte)75, (byte)80, (byte)20, (byte)20, (byte)38, (byte)116, (byte)172, (byte)232, (byte)200, (byte)193, (byte)108, (byte)21, (byte)182, (byte)34, (byte)148, (byte)100, (byte)137, (byte)157, (byte)22, (byte)251, (byte)169, (byte)108, (byte)110, (byte)36, (byte)247, (byte)226, (byte)40, (byte)98, (byte)152, (byte)24, (byte)249, (byte)133, (byte)206, (byte)109, (byte)62, (byte)223, (byte)175, (byte)167, (byte)135, (byte)44, (byte)255, (byte)131, (byte)30, (byte)189, (byte)23, (byte)234, (byte)63, (byte)83, (byte)166, (byte)162, (byte)139, (byte)154, (byte)149, (byte)18, (byte)170, (byte)12, (byte)137, (byte)0, (byte)166, (byte)16, (byte)142, (byte)11, (byte)89, (byte)51, (byte)2, (byte)6, (byte)27, (byte)121, (byte)76, (byte)105, (byte)235, (byte)100, (byte)100, (byte)154, (byte)156, (byte)192, (byte)67, (byte)230, (byte)125, (byte)0, (byte)120, (byte)110, (byte)234, (byte)114, (byte)107, (byte)148, (byte)151, (byte)140, (byte)253, (byte)52, (byte)94, (byte)246, (byte)27, (byte)98, (byte)244, (byte)136, (byte)196, (byte)135, (byte)205, (byte)190, (byte)14, (byte)112, (byte)209, (byte)238, (byte)62, (byte)143, (byte)139, (byte)7, (byte)123, (byte)229, (byte)214, (byte)55, (byte)60, (byte)51, (byte)82, (byte)111, (byte)153, (byte)57, (byte)191, (byte)10, (byte)10, (byte)172, (byte)17, (byte)159, (byte)141, (byte)184, (byte)179, (byte)142, (byte)9, (byte)91, (byte)45, (byte)35, (byte)39, (byte)216, (byte)24, (byte)71, (byte)232, (byte)109, (byte)92, (byte)212, (byte)198, (byte)17, (byte)84, (byte)106, (byte)150, (byte)52, (byte)72, (byte)193, (byte)246, (byte)98, (byte)87, (byte)228, (byte)43, (byte)63, (byte)186, (byte)142, (byte)156, (byte)197, (byte)197, (byte)0, (byte)239, (byte)253, (byte)151, (byte)48, (byte)19, (byte)108, (byte)88, (byte)59, (byte)26, (byte)216, (byte)149, (byte)244, (byte)115, (byte)128, (byte)10, (byte)181, (byte)45, (byte)35, (byte)83, (byte)20, (byte)157, (byte)74, (byte)225, (byte)51, (byte)89, (byte)250, (byte)190, (byte)21}, 0) ;
            p248.target_system = (byte)(byte)197;
            p248.target_network = (byte)(byte)233;
            CommunicationChannel.instance.send(p248);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnMEMORY_VECTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ver == (byte)(byte)58);
                Debug.Assert(pack.value.SequenceEqual(new sbyte[] {(sbyte)113, (sbyte) - 96, (sbyte) - 122, (sbyte)107, (sbyte) - 15, (sbyte)63, (sbyte)124, (sbyte)114, (sbyte)114, (sbyte)123, (sbyte) - 32, (sbyte)32, (sbyte) - 108, (sbyte) - 21, (sbyte)41, (sbyte) - 99, (sbyte) - 5, (sbyte) - 81, (sbyte) - 59, (sbyte)44, (sbyte)119, (sbyte)105, (sbyte)125, (sbyte)46, (sbyte) - 96, (sbyte)38, (sbyte)21, (sbyte) - 109, (sbyte) - 13, (sbyte) - 76, (sbyte) - 74, (sbyte) - 37}));
                Debug.Assert(pack.address == (ushort)(ushort)57684);
                Debug.Assert(pack.type == (byte)(byte)31);
            };
            GroundControl.MEMORY_VECT p249 = CommunicationChannel.new_MEMORY_VECT();
            PH.setPack(p249);
            p249.type = (byte)(byte)31;
            p249.ver = (byte)(byte)58;
            p249.value_SET(new sbyte[] {(sbyte)113, (sbyte) - 96, (sbyte) - 122, (sbyte)107, (sbyte) - 15, (sbyte)63, (sbyte)124, (sbyte)114, (sbyte)114, (sbyte)123, (sbyte) - 32, (sbyte)32, (sbyte) - 108, (sbyte) - 21, (sbyte)41, (sbyte) - 99, (sbyte) - 5, (sbyte) - 81, (sbyte) - 59, (sbyte)44, (sbyte)119, (sbyte)105, (sbyte)125, (sbyte)46, (sbyte) - 96, (sbyte)38, (sbyte)21, (sbyte) - 109, (sbyte) - 13, (sbyte) - 76, (sbyte) - 74, (sbyte) - 37}, 0) ;
            p249.address = (ushort)(ushort)57684;
            CommunicationChannel.instance.send(p249);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnDEBUG_VECTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.z == (float)2.600006E38F);
                Debug.Assert(pack.x == (float) -2.4235573E38F);
                Debug.Assert(pack.name_LEN(ph) == 2);
                Debug.Assert(pack.name_TRY(ph).Equals("fw"));
                Debug.Assert(pack.time_usec == (ulong)9170299701831629429L);
                Debug.Assert(pack.y == (float)4.398658E37F);
            };
            GroundControl.DEBUG_VECT p250 = CommunicationChannel.new_DEBUG_VECT();
            PH.setPack(p250);
            p250.name_SET("fw", PH) ;
            p250.x = (float) -2.4235573E38F;
            p250.z = (float)2.600006E38F;
            p250.y = (float)4.398658E37F;
            p250.time_usec = (ulong)9170299701831629429L;
            CommunicationChannel.instance.send(p250);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnNAMED_VALUE_FLOATReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.name_LEN(ph) == 5);
                Debug.Assert(pack.name_TRY(ph).Equals("uVwSK"));
                Debug.Assert(pack.time_boot_ms == (uint)704709692U);
                Debug.Assert(pack.value == (float)6.2397287E37F);
            };
            GroundControl.NAMED_VALUE_FLOAT p251 = CommunicationChannel.new_NAMED_VALUE_FLOAT();
            PH.setPack(p251);
            p251.name_SET("uVwSK", PH) ;
            p251.time_boot_ms = (uint)704709692U;
            p251.value = (float)6.2397287E37F;
            CommunicationChannel.instance.send(p251);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnNAMED_VALUE_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.value == (int) -1105928151);
                Debug.Assert(pack.name_LEN(ph) == 7);
                Debug.Assert(pack.name_TRY(ph).Equals("joybkzh"));
                Debug.Assert(pack.time_boot_ms == (uint)258471349U);
            };
            GroundControl.NAMED_VALUE_INT p252 = CommunicationChannel.new_NAMED_VALUE_INT();
            PH.setPack(p252);
            p252.value = (int) -1105928151;
            p252.name_SET("joybkzh", PH) ;
            p252.time_boot_ms = (uint)258471349U;
            CommunicationChannel.instance.send(p252);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSTATUSTEXTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.text_LEN(ph) == 45);
                Debug.Assert(pack.text_TRY(ph).Equals("aplikhomlrrsYetyqkpkliqEyyskgzkAjlzejUktxeazV"));
                Debug.Assert(pack.severity == MAV_SEVERITY.MAV_SEVERITY_ALERT);
            };
            GroundControl.STATUSTEXT p253 = CommunicationChannel.new_STATUSTEXT();
            PH.setPack(p253);
            p253.severity = MAV_SEVERITY.MAV_SEVERITY_ALERT;
            p253.text_SET("aplikhomlrrsYetyqkpkliqEyyskgzkAjlzejUktxeazV", PH) ;
            CommunicationChannel.instance.send(p253);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnDEBUGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.value == (float)1.0373175E38F);
                Debug.Assert(pack.time_boot_ms == (uint)3441700402U);
                Debug.Assert(pack.ind == (byte)(byte)98);
            };
            GroundControl.DEBUG p254 = CommunicationChannel.new_DEBUG();
            PH.setPack(p254);
            p254.value = (float)1.0373175E38F;
            p254.ind = (byte)(byte)98;
            p254.time_boot_ms = (uint)3441700402U;
            CommunicationChannel.instance.send(p254);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSETUP_SIGNINGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.secret_key.SequenceEqual(new byte[] {(byte)187, (byte)40, (byte)212, (byte)198, (byte)164, (byte)167, (byte)235, (byte)63, (byte)64, (byte)178, (byte)147, (byte)1, (byte)239, (byte)67, (byte)65, (byte)228, (byte)164, (byte)139, (byte)8, (byte)20, (byte)187, (byte)134, (byte)28, (byte)6, (byte)138, (byte)23, (byte)117, (byte)46, (byte)61, (byte)250, (byte)156, (byte)57}));
                Debug.Assert(pack.target_system == (byte)(byte)67);
                Debug.Assert(pack.target_component == (byte)(byte)56);
                Debug.Assert(pack.initial_timestamp == (ulong)4791954822348863834L);
            };
            GroundControl.SETUP_SIGNING p256 = CommunicationChannel.new_SETUP_SIGNING();
            PH.setPack(p256);
            p256.secret_key_SET(new byte[] {(byte)187, (byte)40, (byte)212, (byte)198, (byte)164, (byte)167, (byte)235, (byte)63, (byte)64, (byte)178, (byte)147, (byte)1, (byte)239, (byte)67, (byte)65, (byte)228, (byte)164, (byte)139, (byte)8, (byte)20, (byte)187, (byte)134, (byte)28, (byte)6, (byte)138, (byte)23, (byte)117, (byte)46, (byte)61, (byte)250, (byte)156, (byte)57}, 0) ;
            p256.target_component = (byte)(byte)56;
            p256.target_system = (byte)(byte)67;
            p256.initial_timestamp = (ulong)4791954822348863834L;
            CommunicationChannel.instance.send(p256);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnBUTTON_CHANGEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.last_change_ms == (uint)406685927U);
                Debug.Assert(pack.state == (byte)(byte)186);
                Debug.Assert(pack.time_boot_ms == (uint)773692577U);
            };
            GroundControl.BUTTON_CHANGE p257 = CommunicationChannel.new_BUTTON_CHANGE();
            PH.setPack(p257);
            p257.state = (byte)(byte)186;
            p257.time_boot_ms = (uint)773692577U;
            p257.last_change_ms = (uint)406685927U;
            CommunicationChannel.instance.send(p257);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnPLAY_TUNEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.tune_LEN(ph) == 28);
                Debug.Assert(pack.tune_TRY(ph).Equals("krRiytromrfvzwawzfkpiwqkvkst"));
                Debug.Assert(pack.target_component == (byte)(byte)2);
                Debug.Assert(pack.target_system == (byte)(byte)36);
            };
            GroundControl.PLAY_TUNE p258 = CommunicationChannel.new_PLAY_TUNE();
            PH.setPack(p258);
            p258.tune_SET("krRiytromrfvzwawzfkpiwqkvkst", PH) ;
            p258.target_system = (byte)(byte)36;
            p258.target_component = (byte)(byte)2;
            CommunicationChannel.instance.send(p258);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnCAMERA_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.flags == (CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE |
                                            CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE));
                Debug.Assert(pack.resolution_v == (ushort)(ushort)31615);
                Debug.Assert(pack.model_name.SequenceEqual(new byte[] {(byte)65, (byte)154, (byte)158, (byte)93, (byte)54, (byte)174, (byte)44, (byte)19, (byte)160, (byte)193, (byte)149, (byte)246, (byte)133, (byte)127, (byte)91, (byte)55, (byte)83, (byte)60, (byte)162, (byte)12, (byte)5, (byte)56, (byte)65, (byte)121, (byte)231, (byte)79, (byte)44, (byte)122, (byte)236, (byte)7, (byte)181, (byte)224}));
                Debug.Assert(pack.resolution_h == (ushort)(ushort)32972);
                Debug.Assert(pack.lens_id == (byte)(byte)204);
                Debug.Assert(pack.vendor_name.SequenceEqual(new byte[] {(byte)189, (byte)15, (byte)39, (byte)199, (byte)159, (byte)105, (byte)35, (byte)108, (byte)123, (byte)86, (byte)91, (byte)200, (byte)182, (byte)182, (byte)44, (byte)238, (byte)102, (byte)219, (byte)130, (byte)229, (byte)15, (byte)169, (byte)19, (byte)16, (byte)234, (byte)203, (byte)245, (byte)228, (byte)171, (byte)100, (byte)142, (byte)46}));
                Debug.Assert(pack.time_boot_ms == (uint)642032812U);
                Debug.Assert(pack.cam_definition_uri_LEN(ph) == 98);
                Debug.Assert(pack.cam_definition_uri_TRY(ph).Equals("PfmqVvvpfkndnyjxkbhvFpgWuscgWBaybbtrmcebolsyownyjydkxiXggjdpfqxLkmxdggtmcyqnsozitpQtyiwlczillQIngn"));
                Debug.Assert(pack.cam_definition_version == (ushort)(ushort)13453);
                Debug.Assert(pack.sensor_size_v == (float)2.6540385E38F);
                Debug.Assert(pack.sensor_size_h == (float) -2.8294834E37F);
                Debug.Assert(pack.firmware_version == (uint)729682356U);
                Debug.Assert(pack.focal_length == (float)1.8847165E38F);
            };
            GroundControl.CAMERA_INFORMATION p259 = CommunicationChannel.new_CAMERA_INFORMATION();
            PH.setPack(p259);
            p259.cam_definition_version = (ushort)(ushort)13453;
            p259.sensor_size_v = (float)2.6540385E38F;
            p259.model_name_SET(new byte[] {(byte)65, (byte)154, (byte)158, (byte)93, (byte)54, (byte)174, (byte)44, (byte)19, (byte)160, (byte)193, (byte)149, (byte)246, (byte)133, (byte)127, (byte)91, (byte)55, (byte)83, (byte)60, (byte)162, (byte)12, (byte)5, (byte)56, (byte)65, (byte)121, (byte)231, (byte)79, (byte)44, (byte)122, (byte)236, (byte)7, (byte)181, (byte)224}, 0) ;
            p259.firmware_version = (uint)729682356U;
            p259.time_boot_ms = (uint)642032812U;
            p259.cam_definition_uri_SET("PfmqVvvpfkndnyjxkbhvFpgWuscgWBaybbtrmcebolsyownyjydkxiXggjdpfqxLkmxdggtmcyqnsozitpQtyiwlczillQIngn", PH) ;
            p259.lens_id = (byte)(byte)204;
            p259.vendor_name_SET(new byte[] {(byte)189, (byte)15, (byte)39, (byte)199, (byte)159, (byte)105, (byte)35, (byte)108, (byte)123, (byte)86, (byte)91, (byte)200, (byte)182, (byte)182, (byte)44, (byte)238, (byte)102, (byte)219, (byte)130, (byte)229, (byte)15, (byte)169, (byte)19, (byte)16, (byte)234, (byte)203, (byte)245, (byte)228, (byte)171, (byte)100, (byte)142, (byte)46}, 0) ;
            p259.sensor_size_h = (float) -2.8294834E37F;
            p259.focal_length = (float)1.8847165E38F;
            p259.flags = (CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE |
                          CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE);
            p259.resolution_v = (ushort)(ushort)31615;
            p259.resolution_h = (ushort)(ushort)32972;
            CommunicationChannel.instance.send(p259);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnCAMERA_SETTINGSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mode_id == CAMERA_MODE.CAMERA_MODE_VIDEO);
                Debug.Assert(pack.time_boot_ms == (uint)1359951902U);
            };
            GroundControl.CAMERA_SETTINGS p260 = CommunicationChannel.new_CAMERA_SETTINGS();
            PH.setPack(p260);
            p260.mode_id = CAMERA_MODE.CAMERA_MODE_VIDEO;
            p260.time_boot_ms = (uint)1359951902U;
            CommunicationChannel.instance.send(p260);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSTORAGE_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.storage_count == (byte)(byte)81);
                Debug.Assert(pack.read_speed == (float) -3.1640573E38F);
                Debug.Assert(pack.total_capacity == (float)2.4157088E38F);
                Debug.Assert(pack.storage_id == (byte)(byte)161);
                Debug.Assert(pack.time_boot_ms == (uint)1617731122U);
                Debug.Assert(pack.used_capacity == (float) -3.1426845E38F);
                Debug.Assert(pack.status == (byte)(byte)24);
                Debug.Assert(pack.available_capacity == (float)9.055682E37F);
                Debug.Assert(pack.write_speed == (float)2.1803822E38F);
            };
            GroundControl.STORAGE_INFORMATION p261 = CommunicationChannel.new_STORAGE_INFORMATION();
            PH.setPack(p261);
            p261.write_speed = (float)2.1803822E38F;
            p261.storage_id = (byte)(byte)161;
            p261.status = (byte)(byte)24;
            p261.used_capacity = (float) -3.1426845E38F;
            p261.available_capacity = (float)9.055682E37F;
            p261.time_boot_ms = (uint)1617731122U;
            p261.read_speed = (float) -3.1640573E38F;
            p261.storage_count = (byte)(byte)81;
            p261.total_capacity = (float)2.4157088E38F;
            CommunicationChannel.instance.send(p261);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnCAMERA_CAPTURE_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.recording_time_ms == (uint)355633764U);
                Debug.Assert(pack.video_status == (byte)(byte)13);
                Debug.Assert(pack.image_status == (byte)(byte)245);
                Debug.Assert(pack.available_capacity == (float) -1.3612656E38F);
                Debug.Assert(pack.time_boot_ms == (uint)3098457377U);
                Debug.Assert(pack.image_interval == (float)2.7119575E38F);
            };
            GroundControl.CAMERA_CAPTURE_STATUS p262 = CommunicationChannel.new_CAMERA_CAPTURE_STATUS();
            PH.setPack(p262);
            p262.image_interval = (float)2.7119575E38F;
            p262.video_status = (byte)(byte)13;
            p262.available_capacity = (float) -1.3612656E38F;
            p262.recording_time_ms = (uint)355633764U;
            p262.image_status = (byte)(byte)245;
            p262.time_boot_ms = (uint)3098457377U;
            CommunicationChannel.instance.send(p262);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnCAMERA_IMAGE_CAPTUREDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_utc == (ulong)3952040256192091928L);
                Debug.Assert(pack.lat == (int) -106381472);
                Debug.Assert(pack.time_boot_ms == (uint)2855362762U);
                Debug.Assert(pack.relative_alt == (int)910396648);
                Debug.Assert(pack.camera_id == (byte)(byte)138);
                Debug.Assert(pack.capture_result == (sbyte)(sbyte)8);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-2.0071795E38F, 1.0056905E38F, -2.4294418E38F, -5.754455E37F}));
                Debug.Assert(pack.lon == (int)444930241);
                Debug.Assert(pack.alt == (int) -1475524697);
                Debug.Assert(pack.file_url_LEN(ph) == 126);
                Debug.Assert(pack.file_url_TRY(ph).Equals("CcriptmkpRkmdbssiytuopGcjanuzqbmcenmvuxdixwXbfdojkgzQlthyUiusafZaotqxZfbsafquqbwiNTqpluixoEumgsrmpvmyfpnrlekkvukoqsxfkmipoeiit"));
                Debug.Assert(pack.image_index == (int) -1564598040);
            };
            GroundControl.CAMERA_IMAGE_CAPTURED p263 = CommunicationChannel.new_CAMERA_IMAGE_CAPTURED();
            PH.setPack(p263);
            p263.lon = (int)444930241;
            p263.relative_alt = (int)910396648;
            p263.alt = (int) -1475524697;
            p263.capture_result = (sbyte)(sbyte)8;
            p263.file_url_SET("CcriptmkpRkmdbssiytuopGcjanuzqbmcenmvuxdixwXbfdojkgzQlthyUiusafZaotqxZfbsafquqbwiNTqpluixoEumgsrmpvmyfpnrlekkvukoqsxfkmipoeiit", PH) ;
            p263.time_utc = (ulong)3952040256192091928L;
            p263.q_SET(new float[] {-2.0071795E38F, 1.0056905E38F, -2.4294418E38F, -5.754455E37F}, 0) ;
            p263.camera_id = (byte)(byte)138;
            p263.lat = (int) -106381472;
            p263.time_boot_ms = (uint)2855362762U;
            p263.image_index = (int) -1564598040;
            CommunicationChannel.instance.send(p263);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnFLIGHT_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.flight_uuid == (ulong)4697760474365941306L);
                Debug.Assert(pack.arming_time_utc == (ulong)1313695942240391627L);
                Debug.Assert(pack.takeoff_time_utc == (ulong)168252465709586193L);
                Debug.Assert(pack.time_boot_ms == (uint)232247603U);
            };
            GroundControl.FLIGHT_INFORMATION p264 = CommunicationChannel.new_FLIGHT_INFORMATION();
            PH.setPack(p264);
            p264.takeoff_time_utc = (ulong)168252465709586193L;
            p264.time_boot_ms = (uint)232247603U;
            p264.arming_time_utc = (ulong)1313695942240391627L;
            p264.flight_uuid = (ulong)4697760474365941306L;
            CommunicationChannel.instance.send(p264);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnMOUNT_ORIENTATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.roll == (float)1.1143734E37F);
                Debug.Assert(pack.pitch == (float)8.227894E36F);
                Debug.Assert(pack.yaw == (float)1.3048618E38F);
                Debug.Assert(pack.time_boot_ms == (uint)1945259422U);
            };
            GroundControl.MOUNT_ORIENTATION p265 = CommunicationChannel.new_MOUNT_ORIENTATION();
            PH.setPack(p265);
            p265.time_boot_ms = (uint)1945259422U;
            p265.pitch = (float)8.227894E36F;
            p265.roll = (float)1.1143734E37F;
            p265.yaw = (float)1.3048618E38F;
            CommunicationChannel.instance.send(p265);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnLOGGING_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)46, (byte)79, (byte)234, (byte)220, (byte)203, (byte)241, (byte)242, (byte)238, (byte)158, (byte)82, (byte)239, (byte)166, (byte)49, (byte)110, (byte)204, (byte)177, (byte)198, (byte)216, (byte)105, (byte)199, (byte)156, (byte)255, (byte)175, (byte)46, (byte)146, (byte)80, (byte)193, (byte)163, (byte)162, (byte)154, (byte)164, (byte)118, (byte)20, (byte)73, (byte)121, (byte)133, (byte)234, (byte)1, (byte)124, (byte)184, (byte)180, (byte)227, (byte)117, (byte)227, (byte)62, (byte)72, (byte)27, (byte)195, (byte)70, (byte)54, (byte)116, (byte)37, (byte)238, (byte)102, (byte)134, (byte)31, (byte)112, (byte)118, (byte)142, (byte)184, (byte)63, (byte)255, (byte)160, (byte)223, (byte)224, (byte)128, (byte)217, (byte)147, (byte)216, (byte)109, (byte)55, (byte)1, (byte)70, (byte)53, (byte)112, (byte)232, (byte)178, (byte)50, (byte)63, (byte)134, (byte)92, (byte)55, (byte)80, (byte)253, (byte)164, (byte)231, (byte)51, (byte)41, (byte)192, (byte)0, (byte)29, (byte)120, (byte)231, (byte)93, (byte)6, (byte)207, (byte)184, (byte)90, (byte)65, (byte)36, (byte)139, (byte)107, (byte)33, (byte)32, (byte)234, (byte)201, (byte)212, (byte)30, (byte)1, (byte)167, (byte)70, (byte)225, (byte)234, (byte)233, (byte)87, (byte)223, (byte)119, (byte)21, (byte)145, (byte)117, (byte)44, (byte)172, (byte)158, (byte)22, (byte)86, (byte)89, (byte)71, (byte)230, (byte)14, (byte)94, (byte)89, (byte)158, (byte)252, (byte)158, (byte)72, (byte)143, (byte)68, (byte)249, (byte)129, (byte)98, (byte)212, (byte)219, (byte)201, (byte)14, (byte)181, (byte)171, (byte)184, (byte)1, (byte)231, (byte)18, (byte)221, (byte)250, (byte)247, (byte)41, (byte)209, (byte)238, (byte)52, (byte)212, (byte)60, (byte)91, (byte)54, (byte)152, (byte)72, (byte)248, (byte)226, (byte)46, (byte)165, (byte)72, (byte)38, (byte)121, (byte)18, (byte)36, (byte)96, (byte)117, (byte)82, (byte)176, (byte)25, (byte)181, (byte)146, (byte)151, (byte)21, (byte)200, (byte)19, (byte)65, (byte)82, (byte)124, (byte)202, (byte)141, (byte)87, (byte)186, (byte)50, (byte)171, (byte)181, (byte)24, (byte)172, (byte)120, (byte)34, (byte)63, (byte)58, (byte)198, (byte)34, (byte)147, (byte)113, (byte)222, (byte)58, (byte)200, (byte)33, (byte)62, (byte)123, (byte)173, (byte)201, (byte)128, (byte)228, (byte)92, (byte)22, (byte)58, (byte)216, (byte)15, (byte)86, (byte)10, (byte)22, (byte)52, (byte)147, (byte)101, (byte)111, (byte)245, (byte)66, (byte)218, (byte)253, (byte)164, (byte)247, (byte)12, (byte)66, (byte)95, (byte)229, (byte)96, (byte)150, (byte)192, (byte)218, (byte)115, (byte)35, (byte)128, (byte)84, (byte)46, (byte)32, (byte)29, (byte)110, (byte)157, (byte)227}));
                Debug.Assert(pack.sequence == (ushort)(ushort)53662);
                Debug.Assert(pack.first_message_offset == (byte)(byte)192);
                Debug.Assert(pack.target_component == (byte)(byte)94);
                Debug.Assert(pack.target_system == (byte)(byte)31);
                Debug.Assert(pack.length == (byte)(byte)52);
            };
            GroundControl.LOGGING_DATA p266 = CommunicationChannel.new_LOGGING_DATA();
            PH.setPack(p266);
            p266.first_message_offset = (byte)(byte)192;
            p266.target_system = (byte)(byte)31;
            p266.length = (byte)(byte)52;
            p266.target_component = (byte)(byte)94;
            p266.sequence = (ushort)(ushort)53662;
            p266.data__SET(new byte[] {(byte)46, (byte)79, (byte)234, (byte)220, (byte)203, (byte)241, (byte)242, (byte)238, (byte)158, (byte)82, (byte)239, (byte)166, (byte)49, (byte)110, (byte)204, (byte)177, (byte)198, (byte)216, (byte)105, (byte)199, (byte)156, (byte)255, (byte)175, (byte)46, (byte)146, (byte)80, (byte)193, (byte)163, (byte)162, (byte)154, (byte)164, (byte)118, (byte)20, (byte)73, (byte)121, (byte)133, (byte)234, (byte)1, (byte)124, (byte)184, (byte)180, (byte)227, (byte)117, (byte)227, (byte)62, (byte)72, (byte)27, (byte)195, (byte)70, (byte)54, (byte)116, (byte)37, (byte)238, (byte)102, (byte)134, (byte)31, (byte)112, (byte)118, (byte)142, (byte)184, (byte)63, (byte)255, (byte)160, (byte)223, (byte)224, (byte)128, (byte)217, (byte)147, (byte)216, (byte)109, (byte)55, (byte)1, (byte)70, (byte)53, (byte)112, (byte)232, (byte)178, (byte)50, (byte)63, (byte)134, (byte)92, (byte)55, (byte)80, (byte)253, (byte)164, (byte)231, (byte)51, (byte)41, (byte)192, (byte)0, (byte)29, (byte)120, (byte)231, (byte)93, (byte)6, (byte)207, (byte)184, (byte)90, (byte)65, (byte)36, (byte)139, (byte)107, (byte)33, (byte)32, (byte)234, (byte)201, (byte)212, (byte)30, (byte)1, (byte)167, (byte)70, (byte)225, (byte)234, (byte)233, (byte)87, (byte)223, (byte)119, (byte)21, (byte)145, (byte)117, (byte)44, (byte)172, (byte)158, (byte)22, (byte)86, (byte)89, (byte)71, (byte)230, (byte)14, (byte)94, (byte)89, (byte)158, (byte)252, (byte)158, (byte)72, (byte)143, (byte)68, (byte)249, (byte)129, (byte)98, (byte)212, (byte)219, (byte)201, (byte)14, (byte)181, (byte)171, (byte)184, (byte)1, (byte)231, (byte)18, (byte)221, (byte)250, (byte)247, (byte)41, (byte)209, (byte)238, (byte)52, (byte)212, (byte)60, (byte)91, (byte)54, (byte)152, (byte)72, (byte)248, (byte)226, (byte)46, (byte)165, (byte)72, (byte)38, (byte)121, (byte)18, (byte)36, (byte)96, (byte)117, (byte)82, (byte)176, (byte)25, (byte)181, (byte)146, (byte)151, (byte)21, (byte)200, (byte)19, (byte)65, (byte)82, (byte)124, (byte)202, (byte)141, (byte)87, (byte)186, (byte)50, (byte)171, (byte)181, (byte)24, (byte)172, (byte)120, (byte)34, (byte)63, (byte)58, (byte)198, (byte)34, (byte)147, (byte)113, (byte)222, (byte)58, (byte)200, (byte)33, (byte)62, (byte)123, (byte)173, (byte)201, (byte)128, (byte)228, (byte)92, (byte)22, (byte)58, (byte)216, (byte)15, (byte)86, (byte)10, (byte)22, (byte)52, (byte)147, (byte)101, (byte)111, (byte)245, (byte)66, (byte)218, (byte)253, (byte)164, (byte)247, (byte)12, (byte)66, (byte)95, (byte)229, (byte)96, (byte)150, (byte)192, (byte)218, (byte)115, (byte)35, (byte)128, (byte)84, (byte)46, (byte)32, (byte)29, (byte)110, (byte)157, (byte)227}, 0) ;
            CommunicationChannel.instance.send(p266);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnLOGGING_DATA_ACKEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sequence == (ushort)(ushort)14012);
                Debug.Assert(pack.target_system == (byte)(byte)65);
                Debug.Assert(pack.first_message_offset == (byte)(byte)101);
                Debug.Assert(pack.length == (byte)(byte)21);
                Debug.Assert(pack.target_component == (byte)(byte)70);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)6, (byte)152, (byte)158, (byte)237, (byte)48, (byte)52, (byte)143, (byte)129, (byte)55, (byte)220, (byte)49, (byte)128, (byte)166, (byte)252, (byte)0, (byte)9, (byte)101, (byte)168, (byte)190, (byte)116, (byte)243, (byte)86, (byte)1, (byte)124, (byte)22, (byte)243, (byte)79, (byte)17, (byte)222, (byte)2, (byte)101, (byte)249, (byte)183, (byte)189, (byte)50, (byte)110, (byte)75, (byte)112, (byte)1, (byte)97, (byte)140, (byte)25, (byte)145, (byte)132, (byte)57, (byte)158, (byte)200, (byte)208, (byte)253, (byte)94, (byte)164, (byte)72, (byte)50, (byte)54, (byte)169, (byte)100, (byte)165, (byte)138, (byte)115, (byte)253, (byte)206, (byte)29, (byte)50, (byte)44, (byte)94, (byte)213, (byte)211, (byte)177, (byte)153, (byte)234, (byte)43, (byte)247, (byte)126, (byte)82, (byte)107, (byte)103, (byte)42, (byte)36, (byte)199, (byte)198, (byte)103, (byte)146, (byte)177, (byte)93, (byte)83, (byte)83, (byte)57, (byte)107, (byte)123, (byte)226, (byte)115, (byte)142, (byte)105, (byte)70, (byte)138, (byte)128, (byte)158, (byte)253, (byte)57, (byte)169, (byte)233, (byte)204, (byte)154, (byte)22, (byte)178, (byte)12, (byte)161, (byte)32, (byte)62, (byte)159, (byte)34, (byte)245, (byte)199, (byte)72, (byte)72, (byte)217, (byte)11, (byte)170, (byte)105, (byte)33, (byte)129, (byte)175, (byte)226, (byte)77, (byte)44, (byte)228, (byte)221, (byte)10, (byte)106, (byte)51, (byte)241, (byte)56, (byte)218, (byte)105, (byte)45, (byte)120, (byte)147, (byte)30, (byte)66, (byte)76, (byte)245, (byte)119, (byte)6, (byte)55, (byte)250, (byte)41, (byte)143, (byte)36, (byte)159, (byte)103, (byte)46, (byte)4, (byte)31, (byte)8, (byte)168, (byte)181, (byte)4, (byte)16, (byte)181, (byte)235, (byte)231, (byte)9, (byte)215, (byte)35, (byte)101, (byte)132, (byte)12, (byte)117, (byte)62, (byte)137, (byte)147, (byte)237, (byte)115, (byte)113, (byte)70, (byte)35, (byte)202, (byte)126, (byte)37, (byte)109, (byte)150, (byte)254, (byte)210, (byte)88, (byte)251, (byte)36, (byte)153, (byte)75, (byte)132, (byte)184, (byte)169, (byte)145, (byte)254, (byte)129, (byte)198, (byte)80, (byte)17, (byte)11, (byte)57, (byte)24, (byte)251, (byte)38, (byte)157, (byte)145, (byte)235, (byte)9, (byte)55, (byte)46, (byte)67, (byte)222, (byte)157, (byte)240, (byte)39, (byte)255, (byte)197, (byte)227, (byte)74, (byte)245, (byte)193, (byte)217, (byte)96, (byte)225, (byte)166, (byte)86, (byte)11, (byte)174, (byte)243, (byte)217, (byte)213, (byte)243, (byte)249, (byte)233, (byte)94, (byte)79, (byte)75, (byte)73, (byte)192, (byte)126, (byte)43, (byte)245, (byte)2, (byte)239, (byte)195, (byte)151, (byte)159, (byte)166, (byte)125, (byte)244, (byte)184}));
            };
            GroundControl.LOGGING_DATA_ACKED p267 = CommunicationChannel.new_LOGGING_DATA_ACKED();
            PH.setPack(p267);
            p267.first_message_offset = (byte)(byte)101;
            p267.target_system = (byte)(byte)65;
            p267.target_component = (byte)(byte)70;
            p267.data__SET(new byte[] {(byte)6, (byte)152, (byte)158, (byte)237, (byte)48, (byte)52, (byte)143, (byte)129, (byte)55, (byte)220, (byte)49, (byte)128, (byte)166, (byte)252, (byte)0, (byte)9, (byte)101, (byte)168, (byte)190, (byte)116, (byte)243, (byte)86, (byte)1, (byte)124, (byte)22, (byte)243, (byte)79, (byte)17, (byte)222, (byte)2, (byte)101, (byte)249, (byte)183, (byte)189, (byte)50, (byte)110, (byte)75, (byte)112, (byte)1, (byte)97, (byte)140, (byte)25, (byte)145, (byte)132, (byte)57, (byte)158, (byte)200, (byte)208, (byte)253, (byte)94, (byte)164, (byte)72, (byte)50, (byte)54, (byte)169, (byte)100, (byte)165, (byte)138, (byte)115, (byte)253, (byte)206, (byte)29, (byte)50, (byte)44, (byte)94, (byte)213, (byte)211, (byte)177, (byte)153, (byte)234, (byte)43, (byte)247, (byte)126, (byte)82, (byte)107, (byte)103, (byte)42, (byte)36, (byte)199, (byte)198, (byte)103, (byte)146, (byte)177, (byte)93, (byte)83, (byte)83, (byte)57, (byte)107, (byte)123, (byte)226, (byte)115, (byte)142, (byte)105, (byte)70, (byte)138, (byte)128, (byte)158, (byte)253, (byte)57, (byte)169, (byte)233, (byte)204, (byte)154, (byte)22, (byte)178, (byte)12, (byte)161, (byte)32, (byte)62, (byte)159, (byte)34, (byte)245, (byte)199, (byte)72, (byte)72, (byte)217, (byte)11, (byte)170, (byte)105, (byte)33, (byte)129, (byte)175, (byte)226, (byte)77, (byte)44, (byte)228, (byte)221, (byte)10, (byte)106, (byte)51, (byte)241, (byte)56, (byte)218, (byte)105, (byte)45, (byte)120, (byte)147, (byte)30, (byte)66, (byte)76, (byte)245, (byte)119, (byte)6, (byte)55, (byte)250, (byte)41, (byte)143, (byte)36, (byte)159, (byte)103, (byte)46, (byte)4, (byte)31, (byte)8, (byte)168, (byte)181, (byte)4, (byte)16, (byte)181, (byte)235, (byte)231, (byte)9, (byte)215, (byte)35, (byte)101, (byte)132, (byte)12, (byte)117, (byte)62, (byte)137, (byte)147, (byte)237, (byte)115, (byte)113, (byte)70, (byte)35, (byte)202, (byte)126, (byte)37, (byte)109, (byte)150, (byte)254, (byte)210, (byte)88, (byte)251, (byte)36, (byte)153, (byte)75, (byte)132, (byte)184, (byte)169, (byte)145, (byte)254, (byte)129, (byte)198, (byte)80, (byte)17, (byte)11, (byte)57, (byte)24, (byte)251, (byte)38, (byte)157, (byte)145, (byte)235, (byte)9, (byte)55, (byte)46, (byte)67, (byte)222, (byte)157, (byte)240, (byte)39, (byte)255, (byte)197, (byte)227, (byte)74, (byte)245, (byte)193, (byte)217, (byte)96, (byte)225, (byte)166, (byte)86, (byte)11, (byte)174, (byte)243, (byte)217, (byte)213, (byte)243, (byte)249, (byte)233, (byte)94, (byte)79, (byte)75, (byte)73, (byte)192, (byte)126, (byte)43, (byte)245, (byte)2, (byte)239, (byte)195, (byte)151, (byte)159, (byte)166, (byte)125, (byte)244, (byte)184}, 0) ;
            p267.length = (byte)(byte)21;
            p267.sequence = (ushort)(ushort)14012;
            CommunicationChannel.instance.send(p267);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnLOGGING_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)240);
                Debug.Assert(pack.target_component == (byte)(byte)80);
                Debug.Assert(pack.sequence == (ushort)(ushort)25567);
            };
            GroundControl.LOGGING_ACK p268 = CommunicationChannel.new_LOGGING_ACK();
            PH.setPack(p268);
            p268.sequence = (ushort)(ushort)25567;
            p268.target_system = (byte)(byte)240;
            p268.target_component = (byte)(byte)80;
            CommunicationChannel.instance.send(p268);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnVIDEO_STREAM_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.status == (byte)(byte)237);
                Debug.Assert(pack.resolution_h == (ushort)(ushort)7903);
                Debug.Assert(pack.framerate == (float) -2.8995145E38F);
                Debug.Assert(pack.camera_id == (byte)(byte)172);
                Debug.Assert(pack.bitrate == (uint)2945137054U);
                Debug.Assert(pack.rotation == (ushort)(ushort)6992);
                Debug.Assert(pack.resolution_v == (ushort)(ushort)46139);
                Debug.Assert(pack.uri_LEN(ph) == 195);
                Debug.Assert(pack.uri_TRY(ph).Equals("ygejyazlftqlfudxsmeinkmcvpNytdyxhUiscofxeujitowuwnuwveqfjxjvngvowlvtmuvuykbtlDvqtronpIkeqrelsotowgkibfxPttribjzduoyMnsMdbmpfwAbsrkwfprtcLfcrukrfteglhilebxQHPbepdfuprFoShleuhojdrooyiilPghvsanmwlvi"));
            };
            GroundControl.VIDEO_STREAM_INFORMATION p269 = CommunicationChannel.new_VIDEO_STREAM_INFORMATION();
            PH.setPack(p269);
            p269.status = (byte)(byte)237;
            p269.resolution_h = (ushort)(ushort)7903;
            p269.rotation = (ushort)(ushort)6992;
            p269.resolution_v = (ushort)(ushort)46139;
            p269.camera_id = (byte)(byte)172;
            p269.framerate = (float) -2.8995145E38F;
            p269.bitrate = (uint)2945137054U;
            p269.uri_SET("ygejyazlftqlfudxsmeinkmcvpNytdyxhUiscofxeujitowuwnuwveqfjxjvngvowlvtmuvuykbtlDvqtronpIkeqrelsotowgkibfxPttribjzduoyMnsMdbmpfwAbsrkwfprtcLfcrukrfteglhilebxQHPbepdfuprFoShleuhojdrooyiilPghvsanmwlvi", PH) ;
            CommunicationChannel.instance.send(p269);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSET_VIDEO_STREAM_SETTINGSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.camera_id == (byte)(byte)243);
                Debug.Assert(pack.framerate == (float) -3.2341984E38F);
                Debug.Assert(pack.resolution_v == (ushort)(ushort)22639);
                Debug.Assert(pack.uri_LEN(ph) == 114);
                Debug.Assert(pack.uri_TRY(ph).Equals("ixoaxexzewzwoxawyeibzpeehqbasexvdXBRealtykfDeMafmkruisgMrakezukqhmjmviuxavXnxneadwhmuulZhpjhpSgdbppwbobjxrtumhnewu"));
                Debug.Assert(pack.resolution_h == (ushort)(ushort)59808);
                Debug.Assert(pack.rotation == (ushort)(ushort)58116);
                Debug.Assert(pack.target_system == (byte)(byte)66);
                Debug.Assert(pack.bitrate == (uint)387697320U);
                Debug.Assert(pack.target_component == (byte)(byte)6);
            };
            GroundControl.SET_VIDEO_STREAM_SETTINGS p270 = CommunicationChannel.new_SET_VIDEO_STREAM_SETTINGS();
            PH.setPack(p270);
            p270.resolution_h = (ushort)(ushort)59808;
            p270.framerate = (float) -3.2341984E38F;
            p270.target_component = (byte)(byte)6;
            p270.resolution_v = (ushort)(ushort)22639;
            p270.bitrate = (uint)387697320U;
            p270.uri_SET("ixoaxexzewzwoxawyeibzpeehqbasexvdXBRealtykfDeMafmkruisgMrakezukqhmjmviuxavXnxneadwhmuulZhpjhpSgdbppwbobjxrtumhnewu", PH) ;
            p270.target_system = (byte)(byte)66;
            p270.rotation = (ushort)(ushort)58116;
            p270.camera_id = (byte)(byte)243;
            CommunicationChannel.instance.send(p270);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnWIFI_CONFIG_APReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ssid_LEN(ph) == 21);
                Debug.Assert(pack.ssid_TRY(ph).Equals("dmqhcqbgmwIjmcoYrmvyh"));
                Debug.Assert(pack.password_LEN(ph) == 7);
                Debug.Assert(pack.password_TRY(ph).Equals("upIIgrp"));
            };
            GroundControl.WIFI_CONFIG_AP p299 = CommunicationChannel.new_WIFI_CONFIG_AP();
            PH.setPack(p299);
            p299.ssid_SET("dmqhcqbgmwIjmcoYrmvyh", PH) ;
            p299.password_SET("upIIgrp", PH) ;
            CommunicationChannel.instance.send(p299);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnPROTOCOL_VERSIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.version == (ushort)(ushort)54950);
                Debug.Assert(pack.library_version_hash.SequenceEqual(new byte[] {(byte)111, (byte)61, (byte)53, (byte)133, (byte)38, (byte)207, (byte)97, (byte)43}));
                Debug.Assert(pack.max_version == (ushort)(ushort)15859);
                Debug.Assert(pack.min_version == (ushort)(ushort)12375);
                Debug.Assert(pack.spec_version_hash.SequenceEqual(new byte[] {(byte)26, (byte)37, (byte)122, (byte)177, (byte)221, (byte)48, (byte)112, (byte)102}));
            };
            GroundControl.PROTOCOL_VERSION p300 = CommunicationChannel.new_PROTOCOL_VERSION();
            PH.setPack(p300);
            p300.library_version_hash_SET(new byte[] {(byte)111, (byte)61, (byte)53, (byte)133, (byte)38, (byte)207, (byte)97, (byte)43}, 0) ;
            p300.min_version = (ushort)(ushort)12375;
            p300.version = (ushort)(ushort)54950;
            p300.max_version = (ushort)(ushort)15859;
            p300.spec_version_hash_SET(new byte[] {(byte)26, (byte)37, (byte)122, (byte)177, (byte)221, (byte)48, (byte)112, (byte)102}, 0) ;
            CommunicationChannel.instance.send(p300);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnUAVCAN_NODE_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vendor_specific_status_code == (ushort)(ushort)53282);
                Debug.Assert(pack.health == UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_CRITICAL);
                Debug.Assert(pack.mode == UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_SOFTWARE_UPDATE);
                Debug.Assert(pack.uptime_sec == (uint)525141572U);
                Debug.Assert(pack.sub_mode == (byte)(byte)168);
                Debug.Assert(pack.time_usec == (ulong)6062999649787825640L);
            };
            GroundControl.UAVCAN_NODE_STATUS p310 = CommunicationChannel.new_UAVCAN_NODE_STATUS();
            PH.setPack(p310);
            p310.vendor_specific_status_code = (ushort)(ushort)53282;
            p310.uptime_sec = (uint)525141572U;
            p310.mode = UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_SOFTWARE_UPDATE;
            p310.sub_mode = (byte)(byte)168;
            p310.time_usec = (ulong)6062999649787825640L;
            p310.health = UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_CRITICAL;
            CommunicationChannel.instance.send(p310);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnUAVCAN_NODE_INFOReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sw_version_major == (byte)(byte)251);
                Debug.Assert(pack.sw_vcs_commit == (uint)1886475812U);
                Debug.Assert(pack.hw_unique_id.SequenceEqual(new byte[] {(byte)171, (byte)191, (byte)15, (byte)131, (byte)210, (byte)4, (byte)145, (byte)38, (byte)118, (byte)71, (byte)136, (byte)135, (byte)243, (byte)205, (byte)95, (byte)57}));
                Debug.Assert(pack.time_usec == (ulong)8305791404951660800L);
                Debug.Assert(pack.uptime_sec == (uint)2553225165U);
                Debug.Assert(pack.sw_version_minor == (byte)(byte)240);
                Debug.Assert(pack.hw_version_minor == (byte)(byte)124);
                Debug.Assert(pack.name_LEN(ph) == 11);
                Debug.Assert(pack.name_TRY(ph).Equals("bsDyzixgplx"));
                Debug.Assert(pack.hw_version_major == (byte)(byte)110);
            };
            GroundControl.UAVCAN_NODE_INFO p311 = CommunicationChannel.new_UAVCAN_NODE_INFO();
            PH.setPack(p311);
            p311.sw_version_major = (byte)(byte)251;
            p311.hw_version_major = (byte)(byte)110;
            p311.name_SET("bsDyzixgplx", PH) ;
            p311.sw_version_minor = (byte)(byte)240;
            p311.hw_version_minor = (byte)(byte)124;
            p311.sw_vcs_commit = (uint)1886475812U;
            p311.uptime_sec = (uint)2553225165U;
            p311.hw_unique_id_SET(new byte[] {(byte)171, (byte)191, (byte)15, (byte)131, (byte)210, (byte)4, (byte)145, (byte)38, (byte)118, (byte)71, (byte)136, (byte)135, (byte)243, (byte)205, (byte)95, (byte)57}, 0) ;
            p311.time_usec = (ulong)8305791404951660800L;
            CommunicationChannel.instance.send(p311);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_REQUEST_READReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_id_LEN(ph) == 8);
                Debug.Assert(pack.param_id_TRY(ph).Equals("Zvkxxnhp"));
                Debug.Assert(pack.target_system == (byte)(byte)175);
                Debug.Assert(pack.target_component == (byte)(byte)93);
                Debug.Assert(pack.param_index == (short)(short)32622);
            };
            GroundControl.PARAM_EXT_REQUEST_READ p320 = CommunicationChannel.new_PARAM_EXT_REQUEST_READ();
            PH.setPack(p320);
            p320.param_index = (short)(short)32622;
            p320.param_id_SET("Zvkxxnhp", PH) ;
            p320.target_system = (byte)(byte)175;
            p320.target_component = (byte)(byte)93;
            CommunicationChannel.instance.send(p320);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)199);
                Debug.Assert(pack.target_system == (byte)(byte)113);
            };
            GroundControl.PARAM_EXT_REQUEST_LIST p321 = CommunicationChannel.new_PARAM_EXT_REQUEST_LIST();
            PH.setPack(p321);
            p321.target_system = (byte)(byte)113;
            p321.target_component = (byte)(byte)199;
            CommunicationChannel.instance.send(p321);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_VALUEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_id_LEN(ph) == 8);
                Debug.Assert(pack.param_id_TRY(ph).Equals("kIcsgmvr"));
                Debug.Assert(pack.param_index == (ushort)(ushort)51691);
                Debug.Assert(pack.param_value_LEN(ph) == 36);
                Debug.Assert(pack.param_value_TRY(ph).Equals("cuoopjoiratanjjvuiuzcwqxXxgtfYzlHevc"));
                Debug.Assert(pack.param_count == (ushort)(ushort)38909);
                Debug.Assert(pack.param_type == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_CUSTOM);
            };
            GroundControl.PARAM_EXT_VALUE p322 = CommunicationChannel.new_PARAM_EXT_VALUE();
            PH.setPack(p322);
            p322.param_index = (ushort)(ushort)51691;
            p322.param_count = (ushort)(ushort)38909;
            p322.param_type = MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_CUSTOM;
            p322.param_value_SET("cuoopjoiratanjjvuiuzcwqxXxgtfYzlHevc", PH) ;
            p322.param_id_SET("kIcsgmvr", PH) ;
            CommunicationChannel.instance.send(p322);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_SETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_value_LEN(ph) == 6);
                Debug.Assert(pack.param_value_TRY(ph).Equals("xNlwpd"));
                Debug.Assert(pack.target_system == (byte)(byte)71);
                Debug.Assert(pack.param_id_LEN(ph) == 3);
                Debug.Assert(pack.param_id_TRY(ph).Equals("oqu"));
                Debug.Assert(pack.param_type == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT16);
                Debug.Assert(pack.target_component == (byte)(byte)164);
            };
            GroundControl.PARAM_EXT_SET p323 = CommunicationChannel.new_PARAM_EXT_SET();
            PH.setPack(p323);
            p323.param_id_SET("oqu", PH) ;
            p323.param_type = MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT16;
            p323.param_value_SET("xNlwpd", PH) ;
            p323.target_system = (byte)(byte)71;
            p323.target_component = (byte)(byte)164;
            CommunicationChannel.instance.send(p323);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_value_LEN(ph) == 46);
                Debug.Assert(pack.param_value_TRY(ph).Equals("OStcaygaxSmuviuswzhldmibleoJylmsmclvssxvDhzwnn"));
                Debug.Assert(pack.param_type == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT32);
                Debug.Assert(pack.param_id_LEN(ph) == 11);
                Debug.Assert(pack.param_id_TRY(ph).Equals("txPyscjiUsu"));
                Debug.Assert(pack.param_result == PARAM_ACK.PARAM_ACK_FAILED);
            };
            GroundControl.PARAM_EXT_ACK p324 = CommunicationChannel.new_PARAM_EXT_ACK();
            PH.setPack(p324);
            p324.param_value_SET("OStcaygaxSmuviuswzhldmibleoJylmsmclvssxvDhzwnn", PH) ;
            p324.param_type = MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT32;
            p324.param_id_SET("txPyscjiUsu", PH) ;
            p324.param_result = PARAM_ACK.PARAM_ACK_FAILED;
            CommunicationChannel.instance.send(p324);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnOBSTACLE_DISTANCEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.distances.SequenceEqual(new ushort[] {(ushort)32701, (ushort)4066, (ushort)2388, (ushort)34359, (ushort)39815, (ushort)62458, (ushort)24880, (ushort)21190, (ushort)11298, (ushort)38189, (ushort)15878, (ushort)517, (ushort)11485, (ushort)49, (ushort)64134, (ushort)33702, (ushort)40444, (ushort)2382, (ushort)707, (ushort)42957, (ushort)42270, (ushort)16338, (ushort)36273, (ushort)59912, (ushort)24988, (ushort)42197, (ushort)14477, (ushort)35200, (ushort)4907, (ushort)45600, (ushort)334, (ushort)60230, (ushort)9932, (ushort)65200, (ushort)24269, (ushort)26281, (ushort)54496, (ushort)15215, (ushort)13028, (ushort)50851, (ushort)64580, (ushort)16446, (ushort)38527, (ushort)35026, (ushort)22344, (ushort)3544, (ushort)64386, (ushort)60847, (ushort)26184, (ushort)33173, (ushort)19583, (ushort)35527, (ushort)33722, (ushort)41024, (ushort)55744, (ushort)44141, (ushort)7949, (ushort)16753, (ushort)62663, (ushort)40847, (ushort)41761, (ushort)35668, (ushort)21632, (ushort)29636, (ushort)54284, (ushort)61436, (ushort)19339, (ushort)34032, (ushort)17214, (ushort)15614, (ushort)12190, (ushort)21419}));
                Debug.Assert(pack.increment == (byte)(byte)51);
                Debug.Assert(pack.time_usec == (ulong)7394773286028384181L);
                Debug.Assert(pack.max_distance == (ushort)(ushort)5962);
                Debug.Assert(pack.sensor_type == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_RADAR);
                Debug.Assert(pack.min_distance == (ushort)(ushort)50951);
            };
            GroundControl.OBSTACLE_DISTANCE p330 = CommunicationChannel.new_OBSTACLE_DISTANCE();
            PH.setPack(p330);
            p330.increment = (byte)(byte)51;
            p330.min_distance = (ushort)(ushort)50951;
            p330.distances_SET(new ushort[] {(ushort)32701, (ushort)4066, (ushort)2388, (ushort)34359, (ushort)39815, (ushort)62458, (ushort)24880, (ushort)21190, (ushort)11298, (ushort)38189, (ushort)15878, (ushort)517, (ushort)11485, (ushort)49, (ushort)64134, (ushort)33702, (ushort)40444, (ushort)2382, (ushort)707, (ushort)42957, (ushort)42270, (ushort)16338, (ushort)36273, (ushort)59912, (ushort)24988, (ushort)42197, (ushort)14477, (ushort)35200, (ushort)4907, (ushort)45600, (ushort)334, (ushort)60230, (ushort)9932, (ushort)65200, (ushort)24269, (ushort)26281, (ushort)54496, (ushort)15215, (ushort)13028, (ushort)50851, (ushort)64580, (ushort)16446, (ushort)38527, (ushort)35026, (ushort)22344, (ushort)3544, (ushort)64386, (ushort)60847, (ushort)26184, (ushort)33173, (ushort)19583, (ushort)35527, (ushort)33722, (ushort)41024, (ushort)55744, (ushort)44141, (ushort)7949, (ushort)16753, (ushort)62663, (ushort)40847, (ushort)41761, (ushort)35668, (ushort)21632, (ushort)29636, (ushort)54284, (ushort)61436, (ushort)19339, (ushort)34032, (ushort)17214, (ushort)15614, (ushort)12190, (ushort)21419}, 0) ;
            p330.sensor_type = MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_RADAR;
            p330.max_distance = (ushort)(ushort)5962;
            p330.time_usec = (ulong)7394773286028384181L;
            CommunicationChannel.instance.send(p330);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
        }
    }
}