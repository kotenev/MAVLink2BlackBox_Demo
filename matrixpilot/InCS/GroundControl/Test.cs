
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
            *	 (packets that were corrupted on reception on the MAV*/
            public ushort drop_rate_comm
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  4);}
            }

            /**
            *Communication errors (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted
            *	 on reception on the MAV*/
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
            *	 present. Indices defined by ENUM MAV_SYS_STATUS_SENSO*/
            public MAV_SYS_STATUS_SENSOR onboard_control_sensors_present
            {
                set {  BitUtils.set_bits((ulong)(- 1 +   value), 26, data, 152);}
            }

            /**
            *Bitmask showing which onboard controllers and sensors are enabled:  Value of 0: not enabled. Value of
            *	 1: enabled. Indices defined by ENUM MAV_SYS_STATUS_SENSO*/
            public MAV_SYS_STATUS_SENSOR onboard_control_sensors_enabled
            {
                set {  BitUtils.set_bits((ulong)(- 1 +   value), 26, data, 178);}
            }

            /**
            *Bitmask showing which onboard controllers and sensors are operational or have an error:  Value of 0: not
            *	 enabled. Value of 1: enabled. Indices defined by ENUM MAV_SYS_STATUS_SENSO*/
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
            *	 0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. If bit 10 is set
            *	 the floats afx afy afz should be interpreted as force instead of acceleration. Mapping: bit 1: x, bit
            *	 2: y, bit 3: z, bit 4: vx, bit 5: vy, bit 6: vz, bit 7: ax, bit 8: ay, bit 9: az, bit 10: is force setpoint,
            *	 bit 11: yaw, bit 12: yaw rat*/
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
            *	 =*/
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
            *	 the system id of the requesting syste*/
            public byte target_system
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  12);}
            }

            /**
            *0: request ping from all receiving components, if greater than 0: message is a ping response and number
            *	 is the system id of the requesting syste*/
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
            *	 the safest mode possible initially and then gradually move down the encryption level if it gets a NACK
            *	 message indicating an encryption mismatch*/
            public byte version
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  2);}
            }
            /**
            *Password / Key, depending on version plaintext or encrypted. 25 or less characters, NULL terminated. The
            *	 characters may involve A-Z, a-z, 0-9, and "!?,.-*/
            public void passkey_SET(string src, Inside ph)
            {passkey_SET(src.ToCharArray(), 0, src.Length, ph);}/**
*Password / Key, depending on version plaintext or encrypted. 25 or less characters, NULL terminated. The
*	 characters may involve A-Z, a-z, 0-9, and "!?,.-*/
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
            *	 contro*/
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
                    ulong id = id__q(value);
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
            *	 null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes
            *	 storage if the ID is stored as strin*/
            public void param_id_SET(string src, Inside ph)
            {param_id_SET(src.ToCharArray(), 0, src.Length, ph);}/**
*Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT
*	 null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes
*	 storage if the ID is stored as strin*/
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
            *	 null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes
            *	 storage if the ID is stored as strin*/
            public void param_id_SET(string src, Inside ph)
            {param_id_SET(src.ToCharArray(), 0, src.Length, ph);}/**
*Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT
*	 null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes
*	 storage if the ID is stored as strin*/
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
            *	 null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes
            *	 storage if the ID is stored as strin*/
            public void param_id_SET(string src, Inside ph)
            {param_id_SET(src.ToCharArray(), 0, src.Length, ph);}/**
*Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT
*	 null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes
*	 storage if the ID is stored as strin*/
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
            *	 unknown, set to: UINT16_MA*/
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
            *	 the AMSL altitude in addition to the WGS84 altitude*/
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
            *	 provide the AMSL as well*/
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
            *	 8 servos*/
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
            *	 8 servos*/
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
            *	 more than 8 servos*/
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
            *	 send -2 to disable any existing map for this rc_channel_index*/
            public short param_index
            {
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  2);}
            }

            /**
            *Index of parameter RC channel. Not equal to the RC channel id. Typically correpsonds to a potentiometer-knob
            *	 on the RC*/
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
            *	 on implementation*/
            public float param_value_min
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 13);}
            }

            /**
            *Maximum param value. The protocol does not define if this overwrites an onboard maximum value. (Depends
            *	 on implementation*/
            public float param_value_max
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 17);}
            }
            /**
            *Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT
            *	 null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes
            *	 storage if the ID is stored as strin*/
            public void param_id_SET(string src, Inside ph)
            {param_id_SET(src.ToCharArray(), 0, src.Length, ph);}/**
*Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT
*	 null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes
*	 storage if the ID is stored as strin*/
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
            *	 with Z axis up or local, right handed, Z axis down*/
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
            *	 with Z axis up or local, right handed, Z axis down*/
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
            *	 the second row, etc.*/
            public float[] covariance
            {
                set {covariance_SET(value, 0)  ;}
            }
            /**
            *Covariance matrix upper right triangular (first nine entries are the first ROW, next eight entries are
            *	 the second row, etc.*/
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
            *	 are available but not given in this message. This value should be 0 when no RC channels are available*/
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
            *	 bit corresponds to Button 1*/
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
            *	 Generally corresponds to forward(1000)-backward(-1000) movement on a joystick and the pitch of a vehicle*/
            public short x
            {
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  3);}
            }

            /**
            *Y-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid.
            *	 Generally corresponds to left(-1000)-right(1000) movement on a joystick and the roll of a vehicle*/
            public short y
            {
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  5);}
            }

            /**
            *Z-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid.
            *	 Generally corresponds to a separate slider movement with maximum being 1000 and minimum being -1000 on
            *	 a joystick and the thrust of a vehicle. Positive values are positive thrust, negative values are negative
            *	 thrust*/
            public short z
            {
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  7);}
            }

            /**
            *R-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid.
            *	 Generally corresponds to a twisting of the joystick, with counter-clockwise being 1000 and clockwise
            *	 being -1000, and the yaw of a vehicle*/
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
            *	 sequence (0,1,2,3,4)*/
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
            *	 was denied or the progress percentage or 255 if unknown the progress when result is MAV_RESULT_IN_PROGRESS*/
            public void progress_SET(byte src, Inside ph)
            {
                if(ph.field_bit != 11)insert_field(ph, 11, 0);
                BitUtils.set_bytes((ulong)(src), 1, data,  ph.BYTE);
            }/**
*WIP: Additional parameter of the result, example: which parameter of MAV_CMD_NAV_WAYPOINT caused it to
*	 be denied*/
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
            *	 bit 2: body pitch rate, bit 3: body yaw rate. bit 4-bit 6: reserved, bit 7: throttle, bit 8: attitud*/
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
            *	 bit 2: body pitch rate, bit 3: body yaw rate. bit 4-bit 7: reserved, bit 8: attitud*/
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
            *	 0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. If bit 10 is set
            *	 the floats afx afy afz should be interpreted as force instead of acceleration. Mapping: bit 1: x, bit
            *	 2: y, bit 3: z, bit 4: vx, bit 5: vy, bit 6: vz, bit 7: ax, bit 8: ay, bit 9: az, bit 10: is force setpoint,
            *	 bit 11: yaw, bit 12: yaw rat*/
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
            *	 =*/
            public MAV_FRAME coordinate_frame
            {
                set {  BitUtils.set_bits((ulong)(- 0 +   value), 4, data, 416);}
            }
        }
        public class SET_POSITION_TARGET_GLOBAL_INT : GroundControl.SET_POSITION_TARGET_GLOBAL_INT
        {
            /**
            *Bitmask to indicate which dimensions should be ignored by the vehicle: a value of 0b0000000000000000 or
            *	 0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. If bit 10 is set
            *	 the floats afx afy afz should be interpreted as force instead of acceleration. Mapping: bit 1: x, bit
            *	 2: y, bit 3: z, bit 4: vx, bit 5: vy, bit 6: vz, bit 7: ax, bit 8: ay, bit 9: az, bit 10: is force setpoint,
            *	 bit 11: yaw, bit 12: yaw rat*/
            public ushort type_mask
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            /**
            *Timestamp in milliseconds since system boot. The rationale for the timestamp in the setpoint is to allow
            *	 the system to compensate for the transport delay of the setpoint. This allows the system to compensate
            *	 processing latency*/
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
            *	 = 1*/
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
                get {  return (MAV_COLLISION_THREAT_LEVEL)(0 +  BitUtils.get_bits(data, 132, 3));}
            }
        }
        new class V2_EXTENSION : GroundControl.V2_EXTENSION
        {
            /**
            *A code that identifies the software component that understands this message (analogous to usb device classes
            *	 or mime type strings).  If this code is less than 32768, it is considered a 'registered' protocol extension
            *	 and the corresponding entry should be added to https:github.com/mavlink/mavlink/extension-message-ids.xml.
            *	 Software creators can register blocks of message IDs as needed (useful for GCS specific metadata, etc...).
            *	 Message_types greater than 32767 are considered local experiments and should not be checked in to any
            *	 widely distributed codebase*/
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
            *	 and other fields.  The entire content of this block is opaque unless you understand any the encoding
            *	 message_type.  The particular encoding used can be extension specific and might not always be documented
            *	 as part of the mavlink specification*/
            public byte[] payload
            {
                get {return payload_GET(new byte[249], 0);}
            }
            /**
            *Variable length payload. The length is defined by the remaining message length when subtracting the header
            *	 and other fields.  The entire content of this block is opaque unless you understand any the encoding
            *	 message_type.  The particular encoding used can be extension specific and might not always be documented
            *	 as part of the mavlink specification*/
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
                get {  return (CAMERA_MODE)(0 +  BitUtils.get_bits(data, 32, 3));}
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
            *	 set and capture in progress*/
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
            *	 lost (set to 255 if no start exists)*/
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
            *	 lost (set to 255 if no start exists)*/
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
            *	 (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the
            *	 ID is stored as strin*/
            public string param_id_TRY(Inside ph)
            {
                if(ph.field_bit !=  32 && !try_visit_field(ph, 32)  ||  !try_visit_item(ph, 0)) return null;
                return new string(param_id_GET(ph, new char[ph.items], 0));
            }
            /**
            *Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination
            *	 (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the
            *	 ID is stored as strin*/
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
            *	 (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the
            *	 ID is stored as strin*/
            public string param_id_TRY(Inside ph)
            {
                if(ph.field_bit !=  38 && !try_visit_field(ph, 38)  ||  !try_visit_item(ph, 0)) return null;
                return new string(param_id_GET(ph, new char[ph.items], 0));
            }
            /**
            *Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination
            *	 (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the
            *	 ID is stored as strin*/
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
            *	 (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the
            *	 ID is stored as strin*/
            public string param_id_TRY(Inside ph)
            {
                if(ph.field_bit !=  22 && !try_visit_field(ph, 22)  ||  !try_visit_item(ph, 0)) return null;
                return new string(param_id_GET(ph, new char[ph.items], 0));
            }
            /**
            *Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination
            *	 (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the
            *	 ID is stored as strin*/
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
            *	 (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the
            *	 ID is stored as strin*/
            public string param_id_TRY(Inside ph)
            {
                if(ph.field_bit !=  8 && !try_visit_field(ph, 8)  ||  !try_visit_item(ph, 0)) return null;
                return new string(param_id_GET(ph, new char[ph.items], 0));
            }
            /**
            *Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination
            *	 (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the
            *	 ID is stored as strin*/
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
            *	 is right in front of the sensor. A value of max_distance +1 means no obstace is present. A value of UINT16_MAX
            *	 for unknown/not used. In a array element, each unit corresponds to 1cm*/
            public ushort[] distances
            {
                get {return distances_GET(new ushort[72], 0);}
            }
            /**
            *Distance of obstacles in front of the sensor starting on the left side. A value of 0 means that the obstacle
            *	 is right in front of the sensor. A value of max_distance +1 means no obstace is present. A value of UINT16_MAX
            *	 for unknown/not used. In a array element, each unit corresponds to 1cm*/
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
                Debug.Assert(pack.custom_mode == (uint)2393992587U);
                Debug.Assert(pack.type == MAV_TYPE.MAV_TYPE_HEXAROTOR);
                Debug.Assert(pack.autopilot == MAV_AUTOPILOT.MAV_AUTOPILOT_UDB);
                Debug.Assert(pack.system_status == MAV_STATE.MAV_STATE_CALIBRATING);
                Debug.Assert(pack.mavlink_version == (byte)(byte)220);
                Debug.Assert(pack.base_mode == (MAV_MODE_FLAG.MAV_MODE_FLAG_STABILIZE_ENABLED |
                                                MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED));
            };
            HEARTBEAT p0 = new HEARTBEAT();
            PH.setPack(p0);
            p0.autopilot = MAV_AUTOPILOT.MAV_AUTOPILOT_UDB;
            p0.base_mode = (MAV_MODE_FLAG.MAV_MODE_FLAG_STABILIZE_ENABLED |
                            MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED);
            p0.custom_mode = (uint)2393992587U;
            p0.system_status = MAV_STATE.MAV_STATE_CALIBRATING;
            p0.mavlink_version = (byte)(byte)220;
            p0.type = MAV_TYPE.MAV_TYPE_HEXAROTOR;
            ADV_TEST_CH.send(p0);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSYS_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.errors_count3 == (ushort)(ushort)12692);
                Debug.Assert(pack.onboard_control_sensors_present == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_AHRS |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_BATTERY |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL));
                Debug.Assert(pack.onboard_control_sensors_enabled == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO2 |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_GPS |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_AHRS |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_LOGGING |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_BATTERY));
                Debug.Assert(pack.voltage_battery == (ushort)(ushort)13499);
                Debug.Assert(pack.errors_count4 == (ushort)(ushort)37412);
                Debug.Assert(pack.errors_comm == (ushort)(ushort)26749);
                Debug.Assert(pack.onboard_control_sensors_health == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_LOGGING |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL));
                Debug.Assert(pack.drop_rate_comm == (ushort)(ushort)54551);
                Debug.Assert(pack.battery_remaining == (sbyte)(sbyte)43);
                Debug.Assert(pack.errors_count1 == (ushort)(ushort)32987);
                Debug.Assert(pack.errors_count2 == (ushort)(ushort)49014);
                Debug.Assert(pack.current_battery == (short)(short) -12104);
                Debug.Assert(pack.load == (ushort)(ushort)19880);
            };
            SYS_STATUS p1 = new SYS_STATUS();
            PH.setPack(p1);
            p1.drop_rate_comm = (ushort)(ushort)54551;
            p1.onboard_control_sensors_enabled = (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO2 |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_GPS |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_AHRS |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_LOGGING |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_BATTERY);
            p1.current_battery = (short)(short) -12104;
            p1.battery_remaining = (sbyte)(sbyte)43;
            p1.errors_count3 = (ushort)(ushort)12692;
            p1.onboard_control_sensors_present = (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_AHRS |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_BATTERY |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL);
            p1.load = (ushort)(ushort)19880;
            p1.errors_count4 = (ushort)(ushort)37412;
            p1.errors_count1 = (ushort)(ushort)32987;
            p1.errors_comm = (ushort)(ushort)26749;
            p1.onboard_control_sensors_health = (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_LOGGING |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL);
            p1.voltage_battery = (ushort)(ushort)13499;
            p1.errors_count2 = (ushort)(ushort)49014;
            ADV_TEST_CH.send(p1);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSYSTEM_TIMEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_unix_usec == (ulong)7616434037694897091L);
                Debug.Assert(pack.time_boot_ms == (uint)3184873311U);
            };
            SYSTEM_TIME p2 = new SYSTEM_TIME();
            PH.setPack(p2);
            p2.time_boot_ms = (uint)3184873311U;
            p2.time_unix_usec = (ulong)7616434037694897091L;
            ADV_TEST_CH.send(p2);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPOSITION_TARGET_LOCAL_NEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.y == (float)3.2768084E38F);
                Debug.Assert(pack.z == (float) -2.1219946E38F);
                Debug.Assert(pack.time_boot_ms == (uint)3879294595U);
                Debug.Assert(pack.coordinate_frame == MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT);
                Debug.Assert(pack.x == (float)8.30875E37F);
                Debug.Assert(pack.vz == (float)2.0209586E38F);
                Debug.Assert(pack.yaw_rate == (float) -7.5576533E37F);
                Debug.Assert(pack.yaw == (float) -1.4398434E38F);
                Debug.Assert(pack.vx == (float)2.7263434E38F);
                Debug.Assert(pack.afz == (float) -1.2732893E38F);
                Debug.Assert(pack.afx == (float) -2.6615357E38F);
                Debug.Assert(pack.type_mask == (ushort)(ushort)32478);
                Debug.Assert(pack.afy == (float) -2.9919025E38F);
                Debug.Assert(pack.vy == (float) -1.4321039E38F);
            };
            POSITION_TARGET_LOCAL_NED p3 = new POSITION_TARGET_LOCAL_NED();
            PH.setPack(p3);
            p3.x = (float)8.30875E37F;
            p3.vx = (float)2.7263434E38F;
            p3.afz = (float) -1.2732893E38F;
            p3.coordinate_frame = MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT;
            p3.afy = (float) -2.9919025E38F;
            p3.yaw = (float) -1.4398434E38F;
            p3.type_mask = (ushort)(ushort)32478;
            p3.vz = (float)2.0209586E38F;
            p3.time_boot_ms = (uint)3879294595U;
            p3.afx = (float) -2.6615357E38F;
            p3.z = (float) -2.1219946E38F;
            p3.vy = (float) -1.4321039E38F;
            p3.y = (float)3.2768084E38F;
            p3.yaw_rate = (float) -7.5576533E37F;
            ADV_TEST_CH.send(p3);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPINGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)4392896194016484645L);
                Debug.Assert(pack.seq == (uint)2333554126U);
                Debug.Assert(pack.target_system == (byte)(byte)196);
                Debug.Assert(pack.target_component == (byte)(byte)94);
            };
            PING p4 = new PING();
            PH.setPack(p4);
            p4.time_usec = (ulong)4392896194016484645L;
            p4.seq = (uint)2333554126U;
            p4.target_component = (byte)(byte)94;
            p4.target_system = (byte)(byte)196;
            ADV_TEST_CH.send(p4);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCHANGE_OPERATOR_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)113);
                Debug.Assert(pack.control_request == (byte)(byte)155);
                Debug.Assert(pack.passkey_LEN(ph) == 18);
                Debug.Assert(pack.passkey_TRY(ph).Equals("uqxtyeFrsrYwsArfrl"));
                Debug.Assert(pack.version == (byte)(byte)40);
            };
            CHANGE_OPERATOR_CONTROL p5 = new CHANGE_OPERATOR_CONTROL();
            PH.setPack(p5);
            p5.target_system = (byte)(byte)113;
            p5.passkey_SET("uqxtyeFrsrYwsArfrl", PH) ;
            p5.control_request = (byte)(byte)155;
            p5.version = (byte)(byte)40;
            ADV_TEST_CH.send(p5);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCHANGE_OPERATOR_CONTROL_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.control_request == (byte)(byte)214);
                Debug.Assert(pack.ack == (byte)(byte)140);
                Debug.Assert(pack.gcs_system_id == (byte)(byte)18);
            };
            CHANGE_OPERATOR_CONTROL_ACK p6 = new CHANGE_OPERATOR_CONTROL_ACK();
            PH.setPack(p6);
            p6.ack = (byte)(byte)140;
            p6.gcs_system_id = (byte)(byte)18;
            p6.control_request = (byte)(byte)214;
            ADV_TEST_CH.send(p6);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnAUTH_KEYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.key_LEN(ph) == 7);
                Debug.Assert(pack.key_TRY(ph).Equals("Mbogeib"));
            };
            AUTH_KEY p7 = new AUTH_KEY();
            PH.setPack(p7);
            p7.key_SET("Mbogeib", PH) ;
            ADV_TEST_CH.send(p7);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_MODEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)33);
                Debug.Assert(pack.base_mode == MAV_MODE.MAV_MODE_GUIDED_ARMED);
                Debug.Assert(pack.custom_mode == (uint)561353255U);
            };
            SET_MODE p11 = new SET_MODE();
            PH.setPack(p11);
            p11.custom_mode = (uint)561353255U;
            p11.base_mode = MAV_MODE.MAV_MODE_GUIDED_ARMED;
            p11.target_system = (byte)(byte)33;
            ADV_TEST_CH.send(p11);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_REQUEST_READReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)119);
                Debug.Assert(pack.param_id_LEN(ph) == 15);
                Debug.Assert(pack.param_id_TRY(ph).Equals("pryycricwuqwqtn"));
                Debug.Assert(pack.target_system == (byte)(byte)48);
                Debug.Assert(pack.param_index == (short)(short) -29417);
            };
            PARAM_REQUEST_READ p20 = new PARAM_REQUEST_READ();
            PH.setPack(p20);
            p20.param_index = (short)(short) -29417;
            p20.target_system = (byte)(byte)48;
            p20.target_component = (byte)(byte)119;
            p20.param_id_SET("pryycricwuqwqtn", PH) ;
            ADV_TEST_CH.send(p20);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)82);
                Debug.Assert(pack.target_system == (byte)(byte)164);
            };
            PARAM_REQUEST_LIST p21 = new PARAM_REQUEST_LIST();
            PH.setPack(p21);
            p21.target_component = (byte)(byte)82;
            p21.target_system = (byte)(byte)164;
            ADV_TEST_CH.send(p21);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_VALUEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_id_LEN(ph) == 7);
                Debug.Assert(pack.param_id_TRY(ph).Equals("aivalfm"));
                Debug.Assert(pack.param_count == (ushort)(ushort)11942);
                Debug.Assert(pack.param_type == MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT8);
                Debug.Assert(pack.param_index == (ushort)(ushort)47628);
                Debug.Assert(pack.param_value == (float)2.2797388E37F);
            };
            PARAM_VALUE p22 = new PARAM_VALUE();
            PH.setPack(p22);
            p22.param_count = (ushort)(ushort)11942;
            p22.param_value = (float)2.2797388E37F;
            p22.param_id_SET("aivalfm", PH) ;
            p22.param_type = MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT8;
            p22.param_index = (ushort)(ushort)47628;
            ADV_TEST_CH.send(p22);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_SETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_value == (float)3.2730207E38F);
                Debug.Assert(pack.target_system == (byte)(byte)138);
                Debug.Assert(pack.target_component == (byte)(byte)95);
                Debug.Assert(pack.param_type == MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT16);
                Debug.Assert(pack.param_id_LEN(ph) == 16);
                Debug.Assert(pack.param_id_TRY(ph).Equals("gpfybofbxkwcpxRm"));
            };
            PARAM_SET p23 = new PARAM_SET();
            PH.setPack(p23);
            p23.target_system = (byte)(byte)138;
            p23.param_id_SET("gpfybofbxkwcpxRm", PH) ;
            p23.target_component = (byte)(byte)95;
            p23.param_type = MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT16;
            p23.param_value = (float)3.2730207E38F;
            ADV_TEST_CH.send(p23);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_RAW_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.fix_type == GPS_FIX_TYPE.GPS_FIX_TYPE_NO_FIX);
                Debug.Assert(pack.alt_ellipsoid_TRY(ph) == (int) -272512504);
                Debug.Assert(pack.cog == (ushort)(ushort)37820);
                Debug.Assert(pack.vel_acc_TRY(ph) == (uint)3043137155U);
                Debug.Assert(pack.lat == (int)269318666);
                Debug.Assert(pack.h_acc_TRY(ph) == (uint)3635633846U);
                Debug.Assert(pack.v_acc_TRY(ph) == (uint)1215850556U);
                Debug.Assert(pack.epv == (ushort)(ushort)23933);
                Debug.Assert(pack.alt == (int)1898573449);
                Debug.Assert(pack.vel == (ushort)(ushort)53222);
                Debug.Assert(pack.eph == (ushort)(ushort)65008);
                Debug.Assert(pack.hdg_acc_TRY(ph) == (uint)2780264391U);
                Debug.Assert(pack.lon == (int)1405855801);
                Debug.Assert(pack.satellites_visible == (byte)(byte)46);
                Debug.Assert(pack.time_usec == (ulong)4784949784563571260L);
            };
            GPS_RAW_INT p24 = new GPS_RAW_INT();
            PH.setPack(p24);
            p24.lat = (int)269318666;
            p24.vel_acc_SET((uint)3043137155U, PH) ;
            p24.alt_ellipsoid_SET((int) -272512504, PH) ;
            p24.lon = (int)1405855801;
            p24.cog = (ushort)(ushort)37820;
            p24.epv = (ushort)(ushort)23933;
            p24.v_acc_SET((uint)1215850556U, PH) ;
            p24.hdg_acc_SET((uint)2780264391U, PH) ;
            p24.alt = (int)1898573449;
            p24.satellites_visible = (byte)(byte)46;
            p24.fix_type = GPS_FIX_TYPE.GPS_FIX_TYPE_NO_FIX;
            p24.time_usec = (ulong)4784949784563571260L;
            p24.eph = (ushort)(ushort)65008;
            p24.vel = (ushort)(ushort)53222;
            p24.h_acc_SET((uint)3635633846U, PH) ;
            ADV_TEST_CH.send(p24);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.satellite_prn.SequenceEqual(new byte[] {(byte)170, (byte)103, (byte)108, (byte)220, (byte)176, (byte)238, (byte)175, (byte)51, (byte)25, (byte)101, (byte)159, (byte)2, (byte)162, (byte)170, (byte)215, (byte)110, (byte)160, (byte)51, (byte)40, (byte)181}));
                Debug.Assert(pack.satellite_azimuth.SequenceEqual(new byte[] {(byte)127, (byte)72, (byte)30, (byte)231, (byte)182, (byte)129, (byte)128, (byte)48, (byte)240, (byte)2, (byte)158, (byte)8, (byte)162, (byte)79, (byte)199, (byte)112, (byte)135, (byte)117, (byte)181, (byte)106}));
                Debug.Assert(pack.satellites_visible == (byte)(byte)41);
                Debug.Assert(pack.satellite_elevation.SequenceEqual(new byte[] {(byte)167, (byte)57, (byte)30, (byte)90, (byte)200, (byte)161, (byte)80, (byte)39, (byte)165, (byte)8, (byte)148, (byte)208, (byte)20, (byte)207, (byte)210, (byte)34, (byte)148, (byte)255, (byte)47, (byte)15}));
                Debug.Assert(pack.satellite_snr.SequenceEqual(new byte[] {(byte)92, (byte)30, (byte)28, (byte)162, (byte)180, (byte)130, (byte)50, (byte)193, (byte)9, (byte)84, (byte)211, (byte)226, (byte)99, (byte)165, (byte)113, (byte)61, (byte)121, (byte)165, (byte)135, (byte)235}));
                Debug.Assert(pack.satellite_used.SequenceEqual(new byte[] {(byte)148, (byte)93, (byte)160, (byte)172, (byte)240, (byte)148, (byte)130, (byte)73, (byte)26, (byte)186, (byte)113, (byte)74, (byte)31, (byte)179, (byte)190, (byte)166, (byte)56, (byte)130, (byte)148, (byte)34}));
            };
            GPS_STATUS p25 = new GPS_STATUS();
            PH.setPack(p25);
            p25.satellite_prn_SET(new byte[] {(byte)170, (byte)103, (byte)108, (byte)220, (byte)176, (byte)238, (byte)175, (byte)51, (byte)25, (byte)101, (byte)159, (byte)2, (byte)162, (byte)170, (byte)215, (byte)110, (byte)160, (byte)51, (byte)40, (byte)181}, 0) ;
            p25.satellites_visible = (byte)(byte)41;
            p25.satellite_used_SET(new byte[] {(byte)148, (byte)93, (byte)160, (byte)172, (byte)240, (byte)148, (byte)130, (byte)73, (byte)26, (byte)186, (byte)113, (byte)74, (byte)31, (byte)179, (byte)190, (byte)166, (byte)56, (byte)130, (byte)148, (byte)34}, 0) ;
            p25.satellite_snr_SET(new byte[] {(byte)92, (byte)30, (byte)28, (byte)162, (byte)180, (byte)130, (byte)50, (byte)193, (byte)9, (byte)84, (byte)211, (byte)226, (byte)99, (byte)165, (byte)113, (byte)61, (byte)121, (byte)165, (byte)135, (byte)235}, 0) ;
            p25.satellite_azimuth_SET(new byte[] {(byte)127, (byte)72, (byte)30, (byte)231, (byte)182, (byte)129, (byte)128, (byte)48, (byte)240, (byte)2, (byte)158, (byte)8, (byte)162, (byte)79, (byte)199, (byte)112, (byte)135, (byte)117, (byte)181, (byte)106}, 0) ;
            p25.satellite_elevation_SET(new byte[] {(byte)167, (byte)57, (byte)30, (byte)90, (byte)200, (byte)161, (byte)80, (byte)39, (byte)165, (byte)8, (byte)148, (byte)208, (byte)20, (byte)207, (byte)210, (byte)34, (byte)148, (byte)255, (byte)47, (byte)15}, 0) ;
            ADV_TEST_CH.send(p25);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_IMUReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)2730304482U);
                Debug.Assert(pack.xgyro == (short)(short)10483);
                Debug.Assert(pack.ymag == (short)(short)6765);
                Debug.Assert(pack.ygyro == (short)(short) -20658);
                Debug.Assert(pack.yacc == (short)(short)18891);
                Debug.Assert(pack.zacc == (short)(short) -28306);
                Debug.Assert(pack.xmag == (short)(short)25533);
                Debug.Assert(pack.zgyro == (short)(short)9203);
                Debug.Assert(pack.xacc == (short)(short)30290);
                Debug.Assert(pack.zmag == (short)(short) -26876);
            };
            SCALED_IMU p26 = new SCALED_IMU();
            PH.setPack(p26);
            p26.yacc = (short)(short)18891;
            p26.xacc = (short)(short)30290;
            p26.xmag = (short)(short)25533;
            p26.ygyro = (short)(short) -20658;
            p26.ymag = (short)(short)6765;
            p26.zmag = (short)(short) -26876;
            p26.zgyro = (short)(short)9203;
            p26.xgyro = (short)(short)10483;
            p26.time_boot_ms = (uint)2730304482U;
            p26.zacc = (short)(short) -28306;
            ADV_TEST_CH.send(p26);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRAW_IMUReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.zmag == (short)(short) -14459);
                Debug.Assert(pack.zgyro == (short)(short) -13634);
                Debug.Assert(pack.yacc == (short)(short)13592);
                Debug.Assert(pack.time_usec == (ulong)575128190964158538L);
                Debug.Assert(pack.ygyro == (short)(short) -16776);
                Debug.Assert(pack.zacc == (short)(short)7142);
                Debug.Assert(pack.xmag == (short)(short) -25433);
                Debug.Assert(pack.ymag == (short)(short) -22374);
                Debug.Assert(pack.xacc == (short)(short) -30402);
                Debug.Assert(pack.xgyro == (short)(short) -15580);
            };
            RAW_IMU p27 = new RAW_IMU();
            PH.setPack(p27);
            p27.xmag = (short)(short) -25433;
            p27.xacc = (short)(short) -30402;
            p27.zgyro = (short)(short) -13634;
            p27.zacc = (short)(short)7142;
            p27.zmag = (short)(short) -14459;
            p27.yacc = (short)(short)13592;
            p27.time_usec = (ulong)575128190964158538L;
            p27.ymag = (short)(short) -22374;
            p27.ygyro = (short)(short) -16776;
            p27.xgyro = (short)(short) -15580;
            ADV_TEST_CH.send(p27);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRAW_PRESSUREReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.press_diff2 == (short)(short)11159);
                Debug.Assert(pack.press_diff1 == (short)(short) -19451);
                Debug.Assert(pack.temperature == (short)(short) -18292);
                Debug.Assert(pack.press_abs == (short)(short) -17233);
                Debug.Assert(pack.time_usec == (ulong)3712261667540524277L);
            };
            RAW_PRESSURE p28 = new RAW_PRESSURE();
            PH.setPack(p28);
            p28.press_abs = (short)(short) -17233;
            p28.temperature = (short)(short) -18292;
            p28.press_diff1 = (short)(short) -19451;
            p28.press_diff2 = (short)(short)11159;
            p28.time_usec = (ulong)3712261667540524277L;
            ADV_TEST_CH.send(p28);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_PRESSUREReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.press_abs == (float) -2.7220434E38F);
                Debug.Assert(pack.temperature == (short)(short) -32170);
                Debug.Assert(pack.time_boot_ms == (uint)1757939527U);
                Debug.Assert(pack.press_diff == (float)1.8275032E37F);
            };
            SCALED_PRESSURE p29 = new SCALED_PRESSURE();
            PH.setPack(p29);
            p29.press_abs = (float) -2.7220434E38F;
            p29.time_boot_ms = (uint)1757939527U;
            p29.temperature = (short)(short) -32170;
            p29.press_diff = (float)1.8275032E37F;
            ADV_TEST_CH.send(p29);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATTITUDEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.rollspeed == (float)1.2762605E38F);
                Debug.Assert(pack.pitch == (float) -1.0250314E38F);
                Debug.Assert(pack.roll == (float) -2.4868567E38F);
                Debug.Assert(pack.pitchspeed == (float)2.2252961E38F);
                Debug.Assert(pack.time_boot_ms == (uint)2265045438U);
                Debug.Assert(pack.yawspeed == (float) -1.2693269E38F);
                Debug.Assert(pack.yaw == (float)2.6933744E38F);
            };
            ATTITUDE p30 = new ATTITUDE();
            PH.setPack(p30);
            p30.yawspeed = (float) -1.2693269E38F;
            p30.rollspeed = (float)1.2762605E38F;
            p30.pitchspeed = (float)2.2252961E38F;
            p30.yaw = (float)2.6933744E38F;
            p30.roll = (float) -2.4868567E38F;
            p30.pitch = (float) -1.0250314E38F;
            p30.time_boot_ms = (uint)2265045438U;
            ADV_TEST_CH.send(p30);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATTITUDE_QUATERNIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.pitchspeed == (float) -2.8376347E38F);
                Debug.Assert(pack.time_boot_ms == (uint)4193990901U);
                Debug.Assert(pack.rollspeed == (float) -2.6722225E38F);
                Debug.Assert(pack.q3 == (float)1.7497834E38F);
                Debug.Assert(pack.q2 == (float)2.2129896E38F);
                Debug.Assert(pack.yawspeed == (float) -3.0927493E37F);
                Debug.Assert(pack.q4 == (float)5.6463855E37F);
                Debug.Assert(pack.q1 == (float) -3.348882E38F);
            };
            ATTITUDE_QUATERNION p31 = new ATTITUDE_QUATERNION();
            PH.setPack(p31);
            p31.yawspeed = (float) -3.0927493E37F;
            p31.time_boot_ms = (uint)4193990901U;
            p31.q4 = (float)5.6463855E37F;
            p31.rollspeed = (float) -2.6722225E38F;
            p31.pitchspeed = (float) -2.8376347E38F;
            p31.q3 = (float)1.7497834E38F;
            p31.q1 = (float) -3.348882E38F;
            p31.q2 = (float)2.2129896E38F;
            ADV_TEST_CH.send(p31);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOCAL_POSITION_NEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.x == (float)1.4786297E37F);
                Debug.Assert(pack.vy == (float) -9.270547E37F);
                Debug.Assert(pack.time_boot_ms == (uint)1430876601U);
                Debug.Assert(pack.z == (float) -2.4559268E38F);
                Debug.Assert(pack.y == (float)3.0058612E38F);
                Debug.Assert(pack.vx == (float) -2.7844663E38F);
                Debug.Assert(pack.vz == (float)5.1225386E37F);
            };
            LOCAL_POSITION_NED p32 = new LOCAL_POSITION_NED();
            PH.setPack(p32);
            p32.vx = (float) -2.7844663E38F;
            p32.time_boot_ms = (uint)1430876601U;
            p32.vy = (float) -9.270547E37F;
            p32.x = (float)1.4786297E37F;
            p32.vz = (float)5.1225386E37F;
            p32.y = (float)3.0058612E38F;
            p32.z = (float) -2.4559268E38F;
            ADV_TEST_CH.send(p32);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGLOBAL_POSITION_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.hdg == (ushort)(ushort)23760);
                Debug.Assert(pack.vy == (short)(short) -5841);
                Debug.Assert(pack.relative_alt == (int)1519595587);
                Debug.Assert(pack.alt == (int) -845814461);
                Debug.Assert(pack.lat == (int) -133144689);
                Debug.Assert(pack.vx == (short)(short)11075);
                Debug.Assert(pack.vz == (short)(short)22279);
                Debug.Assert(pack.time_boot_ms == (uint)3880369846U);
                Debug.Assert(pack.lon == (int)2057218706);
            };
            GLOBAL_POSITION_INT p33 = new GLOBAL_POSITION_INT();
            PH.setPack(p33);
            p33.lon = (int)2057218706;
            p33.vy = (short)(short) -5841;
            p33.time_boot_ms = (uint)3880369846U;
            p33.relative_alt = (int)1519595587;
            p33.hdg = (ushort)(ushort)23760;
            p33.lat = (int) -133144689;
            p33.vx = (short)(short)11075;
            p33.vz = (short)(short)22279;
            p33.alt = (int) -845814461;
            ADV_TEST_CH.send(p33);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRC_CHANNELS_SCALEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan7_scaled == (short)(short)31736);
                Debug.Assert(pack.chan6_scaled == (short)(short)26501);
                Debug.Assert(pack.chan3_scaled == (short)(short)21621);
                Debug.Assert(pack.chan1_scaled == (short)(short) -1411);
                Debug.Assert(pack.chan5_scaled == (short)(short)11169);
                Debug.Assert(pack.chan4_scaled == (short)(short) -18507);
                Debug.Assert(pack.port == (byte)(byte)134);
                Debug.Assert(pack.chan8_scaled == (short)(short) -1138);
                Debug.Assert(pack.chan2_scaled == (short)(short)4717);
                Debug.Assert(pack.time_boot_ms == (uint)1515013857U);
                Debug.Assert(pack.rssi == (byte)(byte)137);
            };
            RC_CHANNELS_SCALED p34 = new RC_CHANNELS_SCALED();
            PH.setPack(p34);
            p34.chan5_scaled = (short)(short)11169;
            p34.chan4_scaled = (short)(short) -18507;
            p34.time_boot_ms = (uint)1515013857U;
            p34.chan2_scaled = (short)(short)4717;
            p34.chan7_scaled = (short)(short)31736;
            p34.chan6_scaled = (short)(short)26501;
            p34.rssi = (byte)(byte)137;
            p34.port = (byte)(byte)134;
            p34.chan8_scaled = (short)(short) -1138;
            p34.chan1_scaled = (short)(short) -1411;
            p34.chan3_scaled = (short)(short)21621;
            ADV_TEST_CH.send(p34);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRC_CHANNELS_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)29622);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)23244);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)4361);
                Debug.Assert(pack.rssi == (byte)(byte)99);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)47142);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)21204);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)42);
                Debug.Assert(pack.time_boot_ms == (uint)3251604346U);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)45646);
                Debug.Assert(pack.port == (byte)(byte)242);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)32384);
            };
            RC_CHANNELS_RAW p35 = new RC_CHANNELS_RAW();
            PH.setPack(p35);
            p35.port = (byte)(byte)242;
            p35.time_boot_ms = (uint)3251604346U;
            p35.chan3_raw = (ushort)(ushort)29622;
            p35.rssi = (byte)(byte)99;
            p35.chan7_raw = (ushort)(ushort)45646;
            p35.chan1_raw = (ushort)(ushort)32384;
            p35.chan8_raw = (ushort)(ushort)23244;
            p35.chan2_raw = (ushort)(ushort)21204;
            p35.chan5_raw = (ushort)(ushort)42;
            p35.chan4_raw = (ushort)(ushort)4361;
            p35.chan6_raw = (ushort)(ushort)47142;
            ADV_TEST_CH.send(p35);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSERVO_OUTPUT_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.servo13_raw_TRY(ph) == (ushort)(ushort)65470);
                Debug.Assert(pack.servo4_raw == (ushort)(ushort)26873);
                Debug.Assert(pack.servo8_raw == (ushort)(ushort)20899);
                Debug.Assert(pack.servo3_raw == (ushort)(ushort)34127);
                Debug.Assert(pack.servo7_raw == (ushort)(ushort)9271);
                Debug.Assert(pack.port == (byte)(byte)168);
                Debug.Assert(pack.servo5_raw == (ushort)(ushort)30970);
                Debug.Assert(pack.servo15_raw_TRY(ph) == (ushort)(ushort)14266);
                Debug.Assert(pack.servo6_raw == (ushort)(ushort)30839);
                Debug.Assert(pack.servo9_raw_TRY(ph) == (ushort)(ushort)64207);
                Debug.Assert(pack.time_usec == (uint)1152799305U);
                Debug.Assert(pack.servo1_raw == (ushort)(ushort)62589);
                Debug.Assert(pack.servo2_raw == (ushort)(ushort)51880);
                Debug.Assert(pack.servo11_raw_TRY(ph) == (ushort)(ushort)27880);
                Debug.Assert(pack.servo16_raw_TRY(ph) == (ushort)(ushort)47540);
                Debug.Assert(pack.servo12_raw_TRY(ph) == (ushort)(ushort)59550);
                Debug.Assert(pack.servo14_raw_TRY(ph) == (ushort)(ushort)12851);
                Debug.Assert(pack.servo10_raw_TRY(ph) == (ushort)(ushort)19512);
            };
            SERVO_OUTPUT_RAW p36 = new SERVO_OUTPUT_RAW();
            PH.setPack(p36);
            p36.time_usec = (uint)1152799305U;
            p36.servo10_raw_SET((ushort)(ushort)19512, PH) ;
            p36.servo4_raw = (ushort)(ushort)26873;
            p36.servo5_raw = (ushort)(ushort)30970;
            p36.servo8_raw = (ushort)(ushort)20899;
            p36.servo15_raw_SET((ushort)(ushort)14266, PH) ;
            p36.servo2_raw = (ushort)(ushort)51880;
            p36.servo13_raw_SET((ushort)(ushort)65470, PH) ;
            p36.servo11_raw_SET((ushort)(ushort)27880, PH) ;
            p36.servo7_raw = (ushort)(ushort)9271;
            p36.servo1_raw = (ushort)(ushort)62589;
            p36.servo14_raw_SET((ushort)(ushort)12851, PH) ;
            p36.port = (byte)(byte)168;
            p36.servo6_raw = (ushort)(ushort)30839;
            p36.servo16_raw_SET((ushort)(ushort)47540, PH) ;
            p36.servo9_raw_SET((ushort)(ushort)64207, PH) ;
            p36.servo12_raw_SET((ushort)(ushort)59550, PH) ;
            p36.servo3_raw = (ushort)(ushort)34127;
            ADV_TEST_CH.send(p36);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_REQUEST_PARTIAL_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)94);
                Debug.Assert(pack.end_index == (short)(short)29574);
                Debug.Assert(pack.start_index == (short)(short)13241);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
                Debug.Assert(pack.target_system == (byte)(byte)149);
            };
            MISSION_REQUEST_PARTIAL_LIST p37 = new MISSION_REQUEST_PARTIAL_LIST();
            PH.setPack(p37);
            p37.target_component = (byte)(byte)94;
            p37.target_system = (byte)(byte)149;
            p37.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION;
            p37.end_index = (short)(short)29574;
            p37.start_index = (short)(short)13241;
            ADV_TEST_CH.send(p37);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_WRITE_PARTIAL_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
                Debug.Assert(pack.target_system == (byte)(byte)189);
                Debug.Assert(pack.target_component == (byte)(byte)204);
                Debug.Assert(pack.start_index == (short)(short) -1683);
                Debug.Assert(pack.end_index == (short)(short)11086);
            };
            MISSION_WRITE_PARTIAL_LIST p38 = new MISSION_WRITE_PARTIAL_LIST();
            PH.setPack(p38);
            p38.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL;
            p38.start_index = (short)(short) -1683;
            p38.end_index = (short)(short)11086;
            p38.target_component = (byte)(byte)204;
            p38.target_system = (byte)(byte)189;
            ADV_TEST_CH.send(p38);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_ITEMReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.x == (float) -1.1945081E38F);
                Debug.Assert(pack.command == MAV_CMD.MAV_CMD_PREFLIGHT_STORAGE);
                Debug.Assert(pack.target_component == (byte)(byte)26);
                Debug.Assert(pack.current == (byte)(byte)242);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
                Debug.Assert(pack.seq == (ushort)(ushort)22768);
                Debug.Assert(pack.y == (float) -1.5322411E38F);
                Debug.Assert(pack.param2 == (float) -1.5081066E37F);
                Debug.Assert(pack.param3 == (float) -3.0227218E38F);
                Debug.Assert(pack.param4 == (float) -3.1463853E38F);
                Debug.Assert(pack.frame == MAV_FRAME.MAV_FRAME_LOCAL_ENU);
                Debug.Assert(pack.autocontinue == (byte)(byte)195);
                Debug.Assert(pack.param1 == (float) -2.4648056E38F);
                Debug.Assert(pack.target_system == (byte)(byte)63);
                Debug.Assert(pack.z == (float) -1.918636E38F);
            };
            MISSION_ITEM p39 = new MISSION_ITEM();
            PH.setPack(p39);
            p39.frame = MAV_FRAME.MAV_FRAME_LOCAL_ENU;
            p39.x = (float) -1.1945081E38F;
            p39.autocontinue = (byte)(byte)195;
            p39.command = MAV_CMD.MAV_CMD_PREFLIGHT_STORAGE;
            p39.param4 = (float) -3.1463853E38F;
            p39.current = (byte)(byte)242;
            p39.target_component = (byte)(byte)26;
            p39.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL;
            p39.z = (float) -1.918636E38F;
            p39.param2 = (float) -1.5081066E37F;
            p39.target_system = (byte)(byte)63;
            p39.param1 = (float) -2.4648056E38F;
            p39.seq = (ushort)(ushort)22768;
            p39.param3 = (float) -3.0227218E38F;
            p39.y = (float) -1.5322411E38F;
            ADV_TEST_CH.send(p39);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)239);
                Debug.Assert(pack.target_component == (byte)(byte)124);
                Debug.Assert(pack.seq == (ushort)(ushort)12421);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
            };
            MISSION_REQUEST p40 = new MISSION_REQUEST();
            PH.setPack(p40);
            p40.seq = (ushort)(ushort)12421;
            p40.target_system = (byte)(byte)239;
            p40.target_component = (byte)(byte)124;
            p40.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION;
            ADV_TEST_CH.send(p40);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_SET_CURRENTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (ushort)(ushort)7735);
                Debug.Assert(pack.target_system == (byte)(byte)56);
                Debug.Assert(pack.target_component == (byte)(byte)133);
            };
            MISSION_SET_CURRENT p41 = new MISSION_SET_CURRENT();
            PH.setPack(p41);
            p41.seq = (ushort)(ushort)7735;
            p41.target_system = (byte)(byte)56;
            p41.target_component = (byte)(byte)133;
            ADV_TEST_CH.send(p41);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_CURRENTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (ushort)(ushort)25854);
            };
            MISSION_CURRENT p42 = new MISSION_CURRENT();
            PH.setPack(p42);
            p42.seq = (ushort)(ushort)25854;
            ADV_TEST_CH.send(p42);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)150);
                Debug.Assert(pack.target_component == (byte)(byte)156);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
            };
            MISSION_REQUEST_LIST p43 = new MISSION_REQUEST_LIST();
            PH.setPack(p43);
            p43.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE;
            p43.target_component = (byte)(byte)156;
            p43.target_system = (byte)(byte)150;
            ADV_TEST_CH.send(p43);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_COUNTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
                Debug.Assert(pack.count == (ushort)(ushort)13074);
                Debug.Assert(pack.target_system == (byte)(byte)128);
                Debug.Assert(pack.target_component == (byte)(byte)32);
            };
            MISSION_COUNT p44 = new MISSION_COUNT();
            PH.setPack(p44);
            p44.target_system = (byte)(byte)128;
            p44.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY;
            p44.count = (ushort)(ushort)13074;
            p44.target_component = (byte)(byte)32;
            ADV_TEST_CH.send(p44);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_CLEAR_ALLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)40);
                Debug.Assert(pack.target_system == (byte)(byte)30);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
            };
            MISSION_CLEAR_ALL p45 = new MISSION_CLEAR_ALL();
            PH.setPack(p45);
            p45.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL;
            p45.target_system = (byte)(byte)30;
            p45.target_component = (byte)(byte)40;
            ADV_TEST_CH.send(p45);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_ITEM_REACHEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (ushort)(ushort)22403);
            };
            MISSION_ITEM_REACHED p46 = new MISSION_ITEM_REACHED();
            PH.setPack(p46);
            p46.seq = (ushort)(ushort)22403;
            ADV_TEST_CH.send(p46);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)117);
                Debug.Assert(pack.target_system == (byte)(byte)85);
                Debug.Assert(pack.type == MAV_MISSION_RESULT.MAV_MISSION_INVALID_PARAM1);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
            };
            MISSION_ACK p47 = new MISSION_ACK();
            PH.setPack(p47);
            p47.type = MAV_MISSION_RESULT.MAV_MISSION_INVALID_PARAM1;
            p47.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE;
            p47.target_system = (byte)(byte)85;
            p47.target_component = (byte)(byte)117;
            ADV_TEST_CH.send(p47);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_GPS_GLOBAL_ORIGINReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)7027831975024368970L);
                Debug.Assert(pack.target_system == (byte)(byte)204);
                Debug.Assert(pack.longitude == (int) -162142539);
                Debug.Assert(pack.latitude == (int) -1518706148);
                Debug.Assert(pack.altitude == (int) -2080587328);
            };
            SET_GPS_GLOBAL_ORIGIN p48 = new SET_GPS_GLOBAL_ORIGIN();
            PH.setPack(p48);
            p48.target_system = (byte)(byte)204;
            p48.longitude = (int) -162142539;
            p48.time_usec_SET((ulong)7027831975024368970L, PH) ;
            p48.altitude = (int) -2080587328;
            p48.latitude = (int) -1518706148;
            ADV_TEST_CH.send(p48);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_GLOBAL_ORIGINReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.altitude == (int)1151314400);
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)396206872584711446L);
                Debug.Assert(pack.latitude == (int)915257866);
                Debug.Assert(pack.longitude == (int) -1690651068);
            };
            GPS_GLOBAL_ORIGIN p49 = new GPS_GLOBAL_ORIGIN();
            PH.setPack(p49);
            p49.altitude = (int)1151314400;
            p49.latitude = (int)915257866;
            p49.time_usec_SET((ulong)396206872584711446L, PH) ;
            p49.longitude = (int) -1690651068;
            ADV_TEST_CH.send(p49);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_MAP_RCReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)152);
                Debug.Assert(pack.param_value0 == (float) -3.1421479E38F);
                Debug.Assert(pack.target_system == (byte)(byte)116);
                Debug.Assert(pack.param_value_min == (float) -3.3573953E38F);
                Debug.Assert(pack.param_value_max == (float) -2.3901278E38F);
                Debug.Assert(pack.param_index == (short)(short) -10783);
                Debug.Assert(pack.parameter_rc_channel_index == (byte)(byte)238);
                Debug.Assert(pack.scale == (float) -1.3266252E38F);
                Debug.Assert(pack.param_id_LEN(ph) == 4);
                Debug.Assert(pack.param_id_TRY(ph).Equals("mihi"));
            };
            PARAM_MAP_RC p50 = new PARAM_MAP_RC();
            PH.setPack(p50);
            p50.scale = (float) -1.3266252E38F;
            p50.param_id_SET("mihi", PH) ;
            p50.param_value0 = (float) -3.1421479E38F;
            p50.target_system = (byte)(byte)116;
            p50.param_value_max = (float) -2.3901278E38F;
            p50.target_component = (byte)(byte)152;
            p50.parameter_rc_channel_index = (byte)(byte)238;
            p50.param_index = (short)(short) -10783;
            p50.param_value_min = (float) -3.3573953E38F;
            ADV_TEST_CH.send(p50);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_REQUEST_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (ushort)(ushort)34390);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
                Debug.Assert(pack.target_component == (byte)(byte)117);
                Debug.Assert(pack.target_system == (byte)(byte)205);
            };
            MISSION_REQUEST_INT p51 = new MISSION_REQUEST_INT();
            PH.setPack(p51);
            p51.target_component = (byte)(byte)117;
            p51.target_system = (byte)(byte)205;
            p51.seq = (ushort)(ushort)34390;
            p51.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION;
            ADV_TEST_CH.send(p51);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSAFETY_SET_ALLOWED_AREAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.p2y == (float)2.0905187E38F);
                Debug.Assert(pack.p1y == (float)1.4392925E38F);
                Debug.Assert(pack.p1z == (float)1.826312E38F);
                Debug.Assert(pack.p2x == (float)2.8416522E38F);
                Debug.Assert(pack.p1x == (float)2.4894025E38F);
                Debug.Assert(pack.frame == MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT);
                Debug.Assert(pack.target_system == (byte)(byte)122);
                Debug.Assert(pack.target_component == (byte)(byte)137);
                Debug.Assert(pack.p2z == (float)2.230118E37F);
            };
            SAFETY_SET_ALLOWED_AREA p54 = new SAFETY_SET_ALLOWED_AREA();
            PH.setPack(p54);
            p54.p2y = (float)2.0905187E38F;
            p54.p1y = (float)1.4392925E38F;
            p54.p2x = (float)2.8416522E38F;
            p54.target_component = (byte)(byte)137;
            p54.p1z = (float)1.826312E38F;
            p54.target_system = (byte)(byte)122;
            p54.p2z = (float)2.230118E37F;
            p54.frame = MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT;
            p54.p1x = (float)2.4894025E38F;
            ADV_TEST_CH.send(p54);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSAFETY_ALLOWED_AREAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.frame == MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT);
                Debug.Assert(pack.p1x == (float) -4.603467E37F);
                Debug.Assert(pack.p2y == (float) -7.0621145E37F);
                Debug.Assert(pack.p2z == (float)3.0977324E38F);
                Debug.Assert(pack.p1z == (float) -3.4021313E37F);
                Debug.Assert(pack.p1y == (float)1.4123152E38F);
                Debug.Assert(pack.p2x == (float) -3.33832E38F);
            };
            SAFETY_ALLOWED_AREA p55 = new SAFETY_ALLOWED_AREA();
            PH.setPack(p55);
            p55.p1y = (float)1.4123152E38F;
            p55.p1z = (float) -3.4021313E37F;
            p55.p2y = (float) -7.0621145E37F;
            p55.frame = MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT;
            p55.p2x = (float) -3.33832E38F;
            p55.p2z = (float)3.0977324E38F;
            p55.p1x = (float) -4.603467E37F;
            ADV_TEST_CH.send(p55);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATTITUDE_QUATERNION_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)181106444210638623L);
                Debug.Assert(pack.rollspeed == (float)2.479224E38F);
                Debug.Assert(pack.covariance.SequenceEqual(new float[] {-2.2136723E38F, -3.4324353E37F, -1.6666576E38F, 2.1621886E38F, -3.8062365E37F, -1.33423E38F, 3.2120751E38F, 1.977977E38F, -7.5996657E37F}));
                Debug.Assert(pack.yawspeed == (float)2.1850074E38F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {2.6836936E38F, 2.8267107E38F, 1.8147727E38F, -2.2453621E38F}));
                Debug.Assert(pack.pitchspeed == (float) -2.952717E38F);
            };
            ATTITUDE_QUATERNION_COV p61 = new ATTITUDE_QUATERNION_COV();
            PH.setPack(p61);
            p61.time_usec = (ulong)181106444210638623L;
            p61.q_SET(new float[] {2.6836936E38F, 2.8267107E38F, 1.8147727E38F, -2.2453621E38F}, 0) ;
            p61.covariance_SET(new float[] {-2.2136723E38F, -3.4324353E37F, -1.6666576E38F, 2.1621886E38F, -3.8062365E37F, -1.33423E38F, 3.2120751E38F, 1.977977E38F, -7.5996657E37F}, 0) ;
            p61.pitchspeed = (float) -2.952717E38F;
            p61.rollspeed = (float)2.479224E38F;
            p61.yawspeed = (float)2.1850074E38F;
            ADV_TEST_CH.send(p61);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnNAV_CONTROLLER_OUTPUTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.nav_pitch == (float)2.4842149E38F);
                Debug.Assert(pack.nav_bearing == (short)(short)23882);
                Debug.Assert(pack.alt_error == (float)5.1356887E37F);
                Debug.Assert(pack.aspd_error == (float)2.890721E37F);
                Debug.Assert(pack.wp_dist == (ushort)(ushort)60258);
                Debug.Assert(pack.target_bearing == (short)(short)25145);
                Debug.Assert(pack.nav_roll == (float) -3.1099104E38F);
                Debug.Assert(pack.xtrack_error == (float)1.9179087E38F);
            };
            NAV_CONTROLLER_OUTPUT p62 = new NAV_CONTROLLER_OUTPUT();
            PH.setPack(p62);
            p62.nav_roll = (float) -3.1099104E38F;
            p62.nav_pitch = (float)2.4842149E38F;
            p62.target_bearing = (short)(short)25145;
            p62.wp_dist = (ushort)(ushort)60258;
            p62.alt_error = (float)5.1356887E37F;
            p62.nav_bearing = (short)(short)23882;
            p62.xtrack_error = (float)1.9179087E38F;
            p62.aspd_error = (float)2.890721E37F;
            ADV_TEST_CH.send(p62);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGLOBAL_POSITION_INT_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.relative_alt == (int)629119160);
                Debug.Assert(pack.covariance.SequenceEqual(new float[] {-1.8057464E38F, -2.6197346E38F, 9.618999E37F, -2.3409326E38F, -2.1430674E38F, 5.984855E37F, -2.1381574E38F, 1.5350084E38F, -2.0324518E38F, 2.071508E38F, -9.397217E37F, -4.717284E37F, -2.231059E38F, -2.992165E38F, -1.259698E38F, -1.4315713E37F, -2.1507605E38F, 1.7526318E38F, 1.1532092E38F, -1.5164654E38F, 3.4019716E38F, 6.8350463E37F, 1.9120536E38F, 4.476262E37F, -1.5294437E38F, 2.0836405E37F, 1.2414994E38F, 3.5402599E37F, 3.2214373E38F, 3.2725447E38F, 1.4240624E38F, 2.6129674E38F, 4.842786E37F, -1.2816868E38F, 2.4069316E38F, 5.420782E37F}));
                Debug.Assert(pack.vx == (float)2.516956E38F);
                Debug.Assert(pack.lon == (int)888961157);
                Debug.Assert(pack.vy == (float)1.9937783E38F);
                Debug.Assert(pack.alt == (int)914515269);
                Debug.Assert(pack.estimator_type == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_NAIVE);
                Debug.Assert(pack.time_usec == (ulong)7064596412227878756L);
                Debug.Assert(pack.lat == (int) -104443589);
                Debug.Assert(pack.vz == (float)1.9418732E38F);
            };
            GLOBAL_POSITION_INT_COV p63 = new GLOBAL_POSITION_INT_COV();
            PH.setPack(p63);
            p63.vx = (float)2.516956E38F;
            p63.estimator_type = MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_NAIVE;
            p63.vy = (float)1.9937783E38F;
            p63.covariance_SET(new float[] {-1.8057464E38F, -2.6197346E38F, 9.618999E37F, -2.3409326E38F, -2.1430674E38F, 5.984855E37F, -2.1381574E38F, 1.5350084E38F, -2.0324518E38F, 2.071508E38F, -9.397217E37F, -4.717284E37F, -2.231059E38F, -2.992165E38F, -1.259698E38F, -1.4315713E37F, -2.1507605E38F, 1.7526318E38F, 1.1532092E38F, -1.5164654E38F, 3.4019716E38F, 6.8350463E37F, 1.9120536E38F, 4.476262E37F, -1.5294437E38F, 2.0836405E37F, 1.2414994E38F, 3.5402599E37F, 3.2214373E38F, 3.2725447E38F, 1.4240624E38F, 2.6129674E38F, 4.842786E37F, -1.2816868E38F, 2.4069316E38F, 5.420782E37F}, 0) ;
            p63.lat = (int) -104443589;
            p63.lon = (int)888961157;
            p63.time_usec = (ulong)7064596412227878756L;
            p63.vz = (float)1.9418732E38F;
            p63.alt = (int)914515269;
            p63.relative_alt = (int)629119160;
            ADV_TEST_CH.send(p63);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOCAL_POSITION_NED_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vz == (float) -7.1666987E37F);
                Debug.Assert(pack.vy == (float) -5.033453E37F);
                Debug.Assert(pack.y == (float) -3.2271236E38F);
                Debug.Assert(pack.az == (float) -2.4460992E38F);
                Debug.Assert(pack.x == (float) -1.5714718E38F);
                Debug.Assert(pack.vx == (float) -2.9336003E38F);
                Debug.Assert(pack.time_usec == (ulong)3465293262349499887L);
                Debug.Assert(pack.z == (float)3.334488E37F);
                Debug.Assert(pack.ax == (float)1.2597339E38F);
                Debug.Assert(pack.ay == (float) -2.1865143E38F);
                Debug.Assert(pack.covariance.SequenceEqual(new float[] {6.988193E37F, -1.6932324E38F, 1.5637354E38F, 2.1688129E38F, 1.2134674E38F, -1.1423403E38F, -1.4875728E38F, -1.777948E38F, -1.8303041E38F, 9.895894E37F, 3.0097897E38F, 1.5782374E38F, 1.0912358E37F, 9.937752E37F, 1.4960361E38F, -8.546259E37F, -2.8284753E38F, 1.9917823E38F, 1.2870538E38F, -8.607409E37F, -1.4155574E38F, 4.737102E37F, -1.516976E37F, -2.5336313E38F, 2.5899972E38F, -6.68661E37F, 3.688683E37F, 1.6983768E38F, -5.802481E37F, 1.158227E38F, 3.2890876E38F, 1.3142094E37F, 2.0802254E38F, 3.2437435E38F, -2.4094085E37F, 1.9903118E38F, -3.2129738E38F, -1.4720979E38F, -1.4395961E38F, 1.6899076E38F, 5.3720614E37F, -1.3218879E38F, 1.4146147E38F, -8.787189E37F, -2.7091728E38F}));
                Debug.Assert(pack.estimator_type == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VIO);
            };
            LOCAL_POSITION_NED_COV p64 = new LOCAL_POSITION_NED_COV();
            PH.setPack(p64);
            p64.time_usec = (ulong)3465293262349499887L;
            p64.vx = (float) -2.9336003E38F;
            p64.ax = (float)1.2597339E38F;
            p64.vz = (float) -7.1666987E37F;
            p64.estimator_type = MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VIO;
            p64.az = (float) -2.4460992E38F;
            p64.covariance_SET(new float[] {6.988193E37F, -1.6932324E38F, 1.5637354E38F, 2.1688129E38F, 1.2134674E38F, -1.1423403E38F, -1.4875728E38F, -1.777948E38F, -1.8303041E38F, 9.895894E37F, 3.0097897E38F, 1.5782374E38F, 1.0912358E37F, 9.937752E37F, 1.4960361E38F, -8.546259E37F, -2.8284753E38F, 1.9917823E38F, 1.2870538E38F, -8.607409E37F, -1.4155574E38F, 4.737102E37F, -1.516976E37F, -2.5336313E38F, 2.5899972E38F, -6.68661E37F, 3.688683E37F, 1.6983768E38F, -5.802481E37F, 1.158227E38F, 3.2890876E38F, 1.3142094E37F, 2.0802254E38F, 3.2437435E38F, -2.4094085E37F, 1.9903118E38F, -3.2129738E38F, -1.4720979E38F, -1.4395961E38F, 1.6899076E38F, 5.3720614E37F, -1.3218879E38F, 1.4146147E38F, -8.787189E37F, -2.7091728E38F}, 0) ;
            p64.x = (float) -1.5714718E38F;
            p64.z = (float)3.334488E37F;
            p64.y = (float) -3.2271236E38F;
            p64.vy = (float) -5.033453E37F;
            p64.ay = (float) -2.1865143E38F;
            ADV_TEST_CH.send(p64);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRC_CHANNELSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)52951);
                Debug.Assert(pack.chan12_raw == (ushort)(ushort)26535);
                Debug.Assert(pack.chan17_raw == (ushort)(ushort)35031);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)56197);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)49877);
                Debug.Assert(pack.rssi == (byte)(byte)91);
                Debug.Assert(pack.chan15_raw == (ushort)(ushort)41429);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)21107);
                Debug.Assert(pack.chancount == (byte)(byte)74);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)33632);
                Debug.Assert(pack.chan10_raw == (ushort)(ushort)4546);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)27250);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)63691);
                Debug.Assert(pack.chan18_raw == (ushort)(ushort)9221);
                Debug.Assert(pack.time_boot_ms == (uint)2840614185U);
                Debug.Assert(pack.chan16_raw == (ushort)(ushort)61376);
                Debug.Assert(pack.chan13_raw == (ushort)(ushort)20482);
                Debug.Assert(pack.chan11_raw == (ushort)(ushort)36968);
                Debug.Assert(pack.chan14_raw == (ushort)(ushort)7977);
                Debug.Assert(pack.chan9_raw == (ushort)(ushort)62873);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)38716);
            };
            RC_CHANNELS p65 = new RC_CHANNELS();
            PH.setPack(p65);
            p65.chan11_raw = (ushort)(ushort)36968;
            p65.time_boot_ms = (uint)2840614185U;
            p65.chan12_raw = (ushort)(ushort)26535;
            p65.chan17_raw = (ushort)(ushort)35031;
            p65.chan16_raw = (ushort)(ushort)61376;
            p65.chan15_raw = (ushort)(ushort)41429;
            p65.chan13_raw = (ushort)(ushort)20482;
            p65.chan8_raw = (ushort)(ushort)21107;
            p65.rssi = (byte)(byte)91;
            p65.chan6_raw = (ushort)(ushort)33632;
            p65.chan2_raw = (ushort)(ushort)63691;
            p65.chan10_raw = (ushort)(ushort)4546;
            p65.chan18_raw = (ushort)(ushort)9221;
            p65.chan9_raw = (ushort)(ushort)62873;
            p65.chan1_raw = (ushort)(ushort)56197;
            p65.chancount = (byte)(byte)74;
            p65.chan14_raw = (ushort)(ushort)7977;
            p65.chan5_raw = (ushort)(ushort)52951;
            p65.chan3_raw = (ushort)(ushort)27250;
            p65.chan4_raw = (ushort)(ushort)38716;
            p65.chan7_raw = (ushort)(ushort)49877;
            ADV_TEST_CH.send(p65);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnREQUEST_DATA_STREAMReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.start_stop == (byte)(byte)95);
                Debug.Assert(pack.target_system == (byte)(byte)42);
                Debug.Assert(pack.req_message_rate == (ushort)(ushort)45385);
                Debug.Assert(pack.req_stream_id == (byte)(byte)125);
                Debug.Assert(pack.target_component == (byte)(byte)47);
            };
            REQUEST_DATA_STREAM p66 = new REQUEST_DATA_STREAM();
            PH.setPack(p66);
            p66.start_stop = (byte)(byte)95;
            p66.req_message_rate = (ushort)(ushort)45385;
            p66.target_component = (byte)(byte)47;
            p66.req_stream_id = (byte)(byte)125;
            p66.target_system = (byte)(byte)42;
            ADV_TEST_CH.send(p66);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnDATA_STREAMReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.stream_id == (byte)(byte)215);
                Debug.Assert(pack.message_rate == (ushort)(ushort)15813);
                Debug.Assert(pack.on_off == (byte)(byte)116);
            };
            DATA_STREAM p67 = new DATA_STREAM();
            PH.setPack(p67);
            p67.on_off = (byte)(byte)116;
            p67.message_rate = (ushort)(ushort)15813;
            p67.stream_id = (byte)(byte)215;
            ADV_TEST_CH.send(p67);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMANUAL_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.buttons == (ushort)(ushort)48765);
                Debug.Assert(pack.y == (short)(short)12288);
                Debug.Assert(pack.r == (short)(short) -26017);
                Debug.Assert(pack.z == (short)(short)886);
                Debug.Assert(pack.target == (byte)(byte)72);
                Debug.Assert(pack.x == (short)(short) -16066);
            };
            MANUAL_CONTROL p69 = new MANUAL_CONTROL();
            PH.setPack(p69);
            p69.r = (short)(short) -26017;
            p69.y = (short)(short)12288;
            p69.target = (byte)(byte)72;
            p69.buttons = (ushort)(ushort)48765;
            p69.x = (short)(short) -16066;
            p69.z = (short)(short)886;
            ADV_TEST_CH.send(p69);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRC_CHANNELS_OVERRIDEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)35);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)65514);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)15563);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)22376);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)2307);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)49802);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)2971);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)45314);
                Debug.Assert(pack.target_component == (byte)(byte)105);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)14835);
            };
            RC_CHANNELS_OVERRIDE p70 = new RC_CHANNELS_OVERRIDE();
            PH.setPack(p70);
            p70.chan7_raw = (ushort)(ushort)45314;
            p70.chan8_raw = (ushort)(ushort)22376;
            p70.target_system = (byte)(byte)35;
            p70.chan6_raw = (ushort)(ushort)65514;
            p70.chan3_raw = (ushort)(ushort)2307;
            p70.chan5_raw = (ushort)(ushort)14835;
            p70.chan4_raw = (ushort)(ushort)15563;
            p70.chan1_raw = (ushort)(ushort)49802;
            p70.chan2_raw = (ushort)(ushort)2971;
            p70.target_component = (byte)(byte)105;
            ADV_TEST_CH.send(p70);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_ITEM_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
                Debug.Assert(pack.command == MAV_CMD.MAV_CMD_DO_GUIDED_LIMITS);
                Debug.Assert(pack.y == (int) -504937283);
                Debug.Assert(pack.param4 == (float)2.1406783E38F);
                Debug.Assert(pack.x == (int) -1433784394);
                Debug.Assert(pack.target_component == (byte)(byte)107);
                Debug.Assert(pack.param1 == (float)2.5287587E38F);
                Debug.Assert(pack.param2 == (float) -1.5812353E38F);
                Debug.Assert(pack.target_system == (byte)(byte)135);
                Debug.Assert(pack.param3 == (float)1.3946831E37F);
                Debug.Assert(pack.z == (float)1.9005167E38F);
                Debug.Assert(pack.seq == (ushort)(ushort)53860);
                Debug.Assert(pack.autocontinue == (byte)(byte)20);
                Debug.Assert(pack.frame == MAV_FRAME.MAV_FRAME_BODY_NED);
                Debug.Assert(pack.current == (byte)(byte)76);
            };
            MISSION_ITEM_INT p73 = new MISSION_ITEM_INT();
            PH.setPack(p73);
            p73.x = (int) -1433784394;
            p73.command = MAV_CMD.MAV_CMD_DO_GUIDED_LIMITS;
            p73.param4 = (float)2.1406783E38F;
            p73.frame = MAV_FRAME.MAV_FRAME_BODY_NED;
            p73.seq = (ushort)(ushort)53860;
            p73.y = (int) -504937283;
            p73.target_component = (byte)(byte)107;
            p73.current = (byte)(byte)76;
            p73.target_system = (byte)(byte)135;
            p73.z = (float)1.9005167E38F;
            p73.autocontinue = (byte)(byte)20;
            p73.param2 = (float) -1.5812353E38F;
            p73.param1 = (float)2.5287587E38F;
            p73.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION;
            p73.param3 = (float)1.3946831E37F;
            ADV_TEST_CH.send(p73);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnVFR_HUDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.alt == (float)1.426952E38F);
                Debug.Assert(pack.throttle == (ushort)(ushort)31122);
                Debug.Assert(pack.climb == (float)6.44219E37F);
                Debug.Assert(pack.groundspeed == (float)8.4971477E37F);
                Debug.Assert(pack.heading == (short)(short)22113);
                Debug.Assert(pack.airspeed == (float)5.1782614E36F);
            };
            VFR_HUD p74 = new VFR_HUD();
            PH.setPack(p74);
            p74.throttle = (ushort)(ushort)31122;
            p74.groundspeed = (float)8.4971477E37F;
            p74.alt = (float)1.426952E38F;
            p74.heading = (short)(short)22113;
            p74.airspeed = (float)5.1782614E36F;
            p74.climb = (float)6.44219E37F;
            ADV_TEST_CH.send(p74);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCOMMAND_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.current == (byte)(byte)251);
                Debug.Assert(pack.command == MAV_CMD.MAV_CMD_NAV_LOITER_UNLIM);
                Debug.Assert(pack.target_system == (byte)(byte)48);
                Debug.Assert(pack.param2 == (float)2.6849357E38F);
                Debug.Assert(pack.target_component == (byte)(byte)184);
                Debug.Assert(pack.param1 == (float) -1.0555131E38F);
                Debug.Assert(pack.param4 == (float)4.826052E37F);
                Debug.Assert(pack.y == (int)1764591589);
                Debug.Assert(pack.param3 == (float) -2.3744412E38F);
                Debug.Assert(pack.frame == MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT);
                Debug.Assert(pack.z == (float) -2.3408941E38F);
                Debug.Assert(pack.x == (int) -1719645634);
                Debug.Assert(pack.autocontinue == (byte)(byte)109);
            };
            COMMAND_INT p75 = new COMMAND_INT();
            PH.setPack(p75);
            p75.target_component = (byte)(byte)184;
            p75.param3 = (float) -2.3744412E38F;
            p75.param1 = (float) -1.0555131E38F;
            p75.param2 = (float)2.6849357E38F;
            p75.y = (int)1764591589;
            p75.z = (float) -2.3408941E38F;
            p75.param4 = (float)4.826052E37F;
            p75.command = MAV_CMD.MAV_CMD_NAV_LOITER_UNLIM;
            p75.frame = MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT;
            p75.x = (int) -1719645634;
            p75.target_system = (byte)(byte)48;
            p75.autocontinue = (byte)(byte)109;
            p75.current = (byte)(byte)251;
            ADV_TEST_CH.send(p75);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCOMMAND_LONGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param5 == (float)2.1397102E38F);
                Debug.Assert(pack.target_system == (byte)(byte)229);
                Debug.Assert(pack.param7 == (float) -9.412684E37F);
                Debug.Assert(pack.confirmation == (byte)(byte)181);
                Debug.Assert(pack.param4 == (float)2.3080084E38F);
                Debug.Assert(pack.param1 == (float) -2.368563E38F);
                Debug.Assert(pack.param6 == (float) -1.8394172E38F);
                Debug.Assert(pack.param2 == (float)2.448353E38F);
                Debug.Assert(pack.target_component == (byte)(byte)158);
                Debug.Assert(pack.param3 == (float) -3.1620715E38F);
                Debug.Assert(pack.command == MAV_CMD.MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION);
            };
            COMMAND_LONG p76 = new COMMAND_LONG();
            PH.setPack(p76);
            p76.param4 = (float)2.3080084E38F;
            p76.confirmation = (byte)(byte)181;
            p76.param2 = (float)2.448353E38F;
            p76.param6 = (float) -1.8394172E38F;
            p76.param1 = (float) -2.368563E38F;
            p76.target_component = (byte)(byte)158;
            p76.command = MAV_CMD.MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION;
            p76.param3 = (float) -3.1620715E38F;
            p76.param7 = (float) -9.412684E37F;
            p76.param5 = (float)2.1397102E38F;
            p76.target_system = (byte)(byte)229;
            ADV_TEST_CH.send(p76);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCOMMAND_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component_TRY(ph) == (byte)(byte)95);
                Debug.Assert(pack.result == MAV_RESULT.MAV_RESULT_FAILED);
                Debug.Assert(pack.progress_TRY(ph) == (byte)(byte)66);
                Debug.Assert(pack.command == MAV_CMD.MAV_CMD_PREFLIGHT_STORAGE_ADVANCED);
                Debug.Assert(pack.target_system_TRY(ph) == (byte)(byte)6);
                Debug.Assert(pack.result_param2_TRY(ph) == (int) -1888471379);
            };
            COMMAND_ACK p77 = new COMMAND_ACK();
            PH.setPack(p77);
            p77.target_system_SET((byte)(byte)6, PH) ;
            p77.command = MAV_CMD.MAV_CMD_PREFLIGHT_STORAGE_ADVANCED;
            p77.progress_SET((byte)(byte)66, PH) ;
            p77.target_component_SET((byte)(byte)95, PH) ;
            p77.result_param2_SET((int) -1888471379, PH) ;
            p77.result = MAV_RESULT.MAV_RESULT_FAILED;
            ADV_TEST_CH.send(p77);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMANUAL_SETPOINTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mode_switch == (byte)(byte)88);
                Debug.Assert(pack.roll == (float)1.5156544E38F);
                Debug.Assert(pack.time_boot_ms == (uint)2924321722U);
                Debug.Assert(pack.manual_override_switch == (byte)(byte)222);
                Debug.Assert(pack.pitch == (float) -9.114645E37F);
                Debug.Assert(pack.thrust == (float)1.2191367E38F);
                Debug.Assert(pack.yaw == (float)6.931138E36F);
            };
            MANUAL_SETPOINT p81 = new MANUAL_SETPOINT();
            PH.setPack(p81);
            p81.manual_override_switch = (byte)(byte)222;
            p81.time_boot_ms = (uint)2924321722U;
            p81.yaw = (float)6.931138E36F;
            p81.roll = (float)1.5156544E38F;
            p81.thrust = (float)1.2191367E38F;
            p81.mode_switch = (byte)(byte)88;
            p81.pitch = (float) -9.114645E37F;
            ADV_TEST_CH.send(p81);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_ATTITUDE_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.q.SequenceEqual(new float[] {2.2973194E38F, -1.7223655E37F, -2.2685666E38F, -1.4715289E38F}));
                Debug.Assert(pack.target_component == (byte)(byte)187);
                Debug.Assert(pack.thrust == (float)1.9922022E38F);
                Debug.Assert(pack.type_mask == (byte)(byte)80);
                Debug.Assert(pack.time_boot_ms == (uint)1557924698U);
                Debug.Assert(pack.target_system == (byte)(byte)176);
                Debug.Assert(pack.body_roll_rate == (float)2.645831E38F);
                Debug.Assert(pack.body_pitch_rate == (float)1.2179911E38F);
                Debug.Assert(pack.body_yaw_rate == (float)1.8548637E38F);
            };
            SET_ATTITUDE_TARGET p82 = new SET_ATTITUDE_TARGET();
            PH.setPack(p82);
            p82.q_SET(new float[] {2.2973194E38F, -1.7223655E37F, -2.2685666E38F, -1.4715289E38F}, 0) ;
            p82.time_boot_ms = (uint)1557924698U;
            p82.body_yaw_rate = (float)1.8548637E38F;
            p82.body_pitch_rate = (float)1.2179911E38F;
            p82.thrust = (float)1.9922022E38F;
            p82.target_component = (byte)(byte)187;
            p82.target_system = (byte)(byte)176;
            p82.type_mask = (byte)(byte)80;
            p82.body_roll_rate = (float)2.645831E38F;
            ADV_TEST_CH.send(p82);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATTITUDE_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.q.SequenceEqual(new float[] {-2.251322E38F, -2.8252048E38F, 2.5039835E38F, -1.5395163E38F}));
                Debug.Assert(pack.thrust == (float) -9.745907E37F);
                Debug.Assert(pack.body_pitch_rate == (float) -3.3782489E38F);
                Debug.Assert(pack.time_boot_ms == (uint)264524047U);
                Debug.Assert(pack.body_roll_rate == (float)8.44116E37F);
                Debug.Assert(pack.body_yaw_rate == (float)3.2480283E38F);
                Debug.Assert(pack.type_mask == (byte)(byte)83);
            };
            ATTITUDE_TARGET p83 = new ATTITUDE_TARGET();
            PH.setPack(p83);
            p83.type_mask = (byte)(byte)83;
            p83.body_pitch_rate = (float) -3.3782489E38F;
            p83.time_boot_ms = (uint)264524047U;
            p83.body_roll_rate = (float)8.44116E37F;
            p83.q_SET(new float[] {-2.251322E38F, -2.8252048E38F, 2.5039835E38F, -1.5395163E38F}, 0) ;
            p83.thrust = (float) -9.745907E37F;
            p83.body_yaw_rate = (float)3.2480283E38F;
            ADV_TEST_CH.send(p83);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_POSITION_TARGET_LOCAL_NEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.y == (float)1.1889708E38F);
                Debug.Assert(pack.vz == (float)2.4503222E38F);
                Debug.Assert(pack.z == (float) -2.8346045E38F);
                Debug.Assert(pack.yaw == (float) -3.670949E37F);
                Debug.Assert(pack.afy == (float) -3.0997745E38F);
                Debug.Assert(pack.target_system == (byte)(byte)140);
                Debug.Assert(pack.afx == (float)1.8669102E38F);
                Debug.Assert(pack.afz == (float)1.0474817E38F);
                Debug.Assert(pack.yaw_rate == (float) -1.3834493E38F);
                Debug.Assert(pack.time_boot_ms == (uint)141137732U);
                Debug.Assert(pack.target_component == (byte)(byte)174);
                Debug.Assert(pack.coordinate_frame == MAV_FRAME.MAV_FRAME_LOCAL_NED);
                Debug.Assert(pack.x == (float)1.2684479E38F);
                Debug.Assert(pack.type_mask == (ushort)(ushort)56763);
                Debug.Assert(pack.vx == (float)1.8672055E38F);
                Debug.Assert(pack.vy == (float)3.105516E38F);
            };
            SET_POSITION_TARGET_LOCAL_NED p84 = new SET_POSITION_TARGET_LOCAL_NED();
            PH.setPack(p84);
            p84.vy = (float)3.105516E38F;
            p84.yaw = (float) -3.670949E37F;
            p84.afz = (float)1.0474817E38F;
            p84.coordinate_frame = MAV_FRAME.MAV_FRAME_LOCAL_NED;
            p84.target_system = (byte)(byte)140;
            p84.vz = (float)2.4503222E38F;
            p84.x = (float)1.2684479E38F;
            p84.y = (float)1.1889708E38F;
            p84.type_mask = (ushort)(ushort)56763;
            p84.time_boot_ms = (uint)141137732U;
            p84.target_component = (byte)(byte)174;
            p84.yaw_rate = (float) -1.3834493E38F;
            p84.afy = (float) -3.0997745E38F;
            p84.z = (float) -2.8346045E38F;
            p84.afx = (float)1.8669102E38F;
            p84.vx = (float)1.8672055E38F;
            ADV_TEST_CH.send(p84);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_POSITION_TARGET_GLOBAL_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.yaw_rate == (float) -2.8933001E38F);
                Debug.Assert(pack.yaw == (float)1.4564152E38F);
                Debug.Assert(pack.vz == (float)1.0344178E38F);
                Debug.Assert(pack.afz == (float)1.8303094E38F);
                Debug.Assert(pack.vx == (float) -2.055908E38F);
                Debug.Assert(pack.lon_int == (int)1183402020);
                Debug.Assert(pack.afx == (float) -3.16189E38F);
                Debug.Assert(pack.afy == (float)2.8274166E38F);
                Debug.Assert(pack.lat_int == (int)1862518069);
                Debug.Assert(pack.vy == (float) -1.5422237E38F);
                Debug.Assert(pack.coordinate_frame == MAV_FRAME.MAV_FRAME_LOCAL_ENU);
                Debug.Assert(pack.type_mask == (ushort)(ushort)32464);
                Debug.Assert(pack.target_component == (byte)(byte)221);
                Debug.Assert(pack.target_system == (byte)(byte)8);
                Debug.Assert(pack.time_boot_ms == (uint)4072643560U);
                Debug.Assert(pack.alt == (float) -3.0394931E38F);
            };
            SET_POSITION_TARGET_GLOBAL_INT p86 = new SET_POSITION_TARGET_GLOBAL_INT();
            PH.setPack(p86);
            p86.vy = (float) -1.5422237E38F;
            p86.yaw_rate = (float) -2.8933001E38F;
            p86.alt = (float) -3.0394931E38F;
            p86.afy = (float)2.8274166E38F;
            p86.coordinate_frame = MAV_FRAME.MAV_FRAME_LOCAL_ENU;
            p86.lat_int = (int)1862518069;
            p86.afx = (float) -3.16189E38F;
            p86.afz = (float)1.8303094E38F;
            p86.target_component = (byte)(byte)221;
            p86.lon_int = (int)1183402020;
            p86.yaw = (float)1.4564152E38F;
            p86.target_system = (byte)(byte)8;
            p86.vz = (float)1.0344178E38F;
            p86.type_mask = (ushort)(ushort)32464;
            p86.time_boot_ms = (uint)4072643560U;
            p86.vx = (float) -2.055908E38F;
            ADV_TEST_CH.send(p86);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPOSITION_TARGET_GLOBAL_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.afy == (float)5.7695236E37F);
                Debug.Assert(pack.yaw == (float) -2.1067217E38F);
                Debug.Assert(pack.vz == (float) -1.4483534E38F);
                Debug.Assert(pack.vy == (float) -8.927791E37F);
                Debug.Assert(pack.afx == (float) -3.368213E38F);
                Debug.Assert(pack.type_mask == (ushort)(ushort)3964);
                Debug.Assert(pack.lon_int == (int)781762076);
                Debug.Assert(pack.lat_int == (int) -1558295396);
                Debug.Assert(pack.coordinate_frame == MAV_FRAME.MAV_FRAME_GLOBAL);
                Debug.Assert(pack.afz == (float) -2.66253E37F);
                Debug.Assert(pack.alt == (float)4.399187E37F);
                Debug.Assert(pack.vx == (float)1.6354261E38F);
                Debug.Assert(pack.time_boot_ms == (uint)33996481U);
                Debug.Assert(pack.yaw_rate == (float)2.5919757E38F);
            };
            GroundControl.POSITION_TARGET_GLOBAL_INT p87 = CommunicationChannel.new_POSITION_TARGET_GLOBAL_INT();
            PH.setPack(p87);
            p87.lat_int = (int) -1558295396;
            p87.afz = (float) -2.66253E37F;
            p87.yaw = (float) -2.1067217E38F;
            p87.afy = (float)5.7695236E37F;
            p87.time_boot_ms = (uint)33996481U;
            p87.yaw_rate = (float)2.5919757E38F;
            p87.vy = (float) -8.927791E37F;
            p87.afx = (float) -3.368213E38F;
            p87.vz = (float) -1.4483534E38F;
            p87.type_mask = (ushort)(ushort)3964;
            p87.alt = (float)4.399187E37F;
            p87.vx = (float)1.6354261E38F;
            p87.coordinate_frame = MAV_FRAME.MAV_FRAME_GLOBAL;
            p87.lon_int = (int)781762076;
            CommunicationChannel.instance.send(p87);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.roll == (float)2.7421396E38F);
                Debug.Assert(pack.y == (float)1.1171996E38F);
                Debug.Assert(pack.pitch == (float)2.6484205E37F);
                Debug.Assert(pack.yaw == (float) -2.0074799E38F);
                Debug.Assert(pack.time_boot_ms == (uint)4214860040U);
                Debug.Assert(pack.x == (float)4.3450327E37F);
                Debug.Assert(pack.z == (float)2.1052188E38F);
            };
            GroundControl.LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET p89 = CommunicationChannel.new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
            PH.setPack(p89);
            p89.pitch = (float)2.6484205E37F;
            p89.yaw = (float) -2.0074799E38F;
            p89.y = (float)1.1171996E38F;
            p89.x = (float)4.3450327E37F;
            p89.roll = (float)2.7421396E38F;
            p89.z = (float)2.1052188E38F;
            p89.time_boot_ms = (uint)4214860040U;
            CommunicationChannel.instance.send(p89);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vx == (short)(short) -6035);
                Debug.Assert(pack.time_usec == (ulong)8333198966895193694L);
                Debug.Assert(pack.yacc == (short)(short) -13079);
                Debug.Assert(pack.alt == (int)1812570525);
                Debug.Assert(pack.rollspeed == (float) -7.354215E37F);
                Debug.Assert(pack.pitchspeed == (float) -1.0069585E38F);
                Debug.Assert(pack.vz == (short)(short)2027);
                Debug.Assert(pack.xacc == (short)(short) -25010);
                Debug.Assert(pack.yaw == (float) -2.6518161E38F);
                Debug.Assert(pack.lon == (int) -1268566553);
                Debug.Assert(pack.zacc == (short)(short)20229);
                Debug.Assert(pack.vy == (short)(short)19095);
                Debug.Assert(pack.yawspeed == (float) -8.865215E37F);
                Debug.Assert(pack.lat == (int)2095761728);
                Debug.Assert(pack.pitch == (float)1.4390543E37F);
                Debug.Assert(pack.roll == (float) -9.3816736E36F);
            };
            GroundControl.HIL_STATE p90 = CommunicationChannel.new_HIL_STATE();
            PH.setPack(p90);
            p90.pitch = (float)1.4390543E37F;
            p90.yawspeed = (float) -8.865215E37F;
            p90.vy = (short)(short)19095;
            p90.time_usec = (ulong)8333198966895193694L;
            p90.xacc = (short)(short) -25010;
            p90.lat = (int)2095761728;
            p90.yaw = (float) -2.6518161E38F;
            p90.yacc = (short)(short) -13079;
            p90.vx = (short)(short) -6035;
            p90.alt = (int)1812570525;
            p90.roll = (float) -9.3816736E36F;
            p90.pitchspeed = (float) -1.0069585E38F;
            p90.lon = (int) -1268566553;
            p90.rollspeed = (float) -7.354215E37F;
            p90.zacc = (short)(short)20229;
            p90.vz = (short)(short)2027;
            CommunicationChannel.instance.send(p90);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_CONTROLSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)2600918368821109244L);
                Debug.Assert(pack.pitch_elevator == (float)3.0880912E38F);
                Debug.Assert(pack.mode == MAV_MODE.MAV_MODE_TEST_ARMED);
                Debug.Assert(pack.roll_ailerons == (float)8.3235896E37F);
                Debug.Assert(pack.throttle == (float) -2.1249402E38F);
                Debug.Assert(pack.aux4 == (float)1.3144842E37F);
                Debug.Assert(pack.yaw_rudder == (float)2.5461584E38F);
                Debug.Assert(pack.nav_mode == (byte)(byte)79);
                Debug.Assert(pack.aux1 == (float)2.8125723E38F);
                Debug.Assert(pack.aux2 == (float) -1.5865639E38F);
                Debug.Assert(pack.aux3 == (float)2.5111453E38F);
            };
            GroundControl.HIL_CONTROLS p91 = CommunicationChannel.new_HIL_CONTROLS();
            PH.setPack(p91);
            p91.roll_ailerons = (float)8.3235896E37F;
            p91.mode = MAV_MODE.MAV_MODE_TEST_ARMED;
            p91.pitch_elevator = (float)3.0880912E38F;
            p91.aux3 = (float)2.5111453E38F;
            p91.aux2 = (float) -1.5865639E38F;
            p91.aux4 = (float)1.3144842E37F;
            p91.time_usec = (ulong)2600918368821109244L;
            p91.nav_mode = (byte)(byte)79;
            p91.throttle = (float) -2.1249402E38F;
            p91.yaw_rudder = (float)2.5461584E38F;
            p91.aux1 = (float)2.8125723E38F;
            CommunicationChannel.instance.send(p91);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_RC_INPUTS_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)47782);
                Debug.Assert(pack.chan10_raw == (ushort)(ushort)24818);
                Debug.Assert(pack.rssi == (byte)(byte)25);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)1731);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)50366);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)25204);
                Debug.Assert(pack.chan11_raw == (ushort)(ushort)8019);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)25494);
                Debug.Assert(pack.chan12_raw == (ushort)(ushort)26438);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)5543);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)42885);
                Debug.Assert(pack.chan9_raw == (ushort)(ushort)52550);
                Debug.Assert(pack.time_usec == (ulong)1110609850191019686L);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)26856);
            };
            GroundControl.HIL_RC_INPUTS_RAW p92 = CommunicationChannel.new_HIL_RC_INPUTS_RAW();
            PH.setPack(p92);
            p92.chan12_raw = (ushort)(ushort)26438;
            p92.chan6_raw = (ushort)(ushort)5543;
            p92.rssi = (byte)(byte)25;
            p92.chan3_raw = (ushort)(ushort)50366;
            p92.chan7_raw = (ushort)(ushort)42885;
            p92.chan11_raw = (ushort)(ushort)8019;
            p92.chan9_raw = (ushort)(ushort)52550;
            p92.time_usec = (ulong)1110609850191019686L;
            p92.chan4_raw = (ushort)(ushort)1731;
            p92.chan2_raw = (ushort)(ushort)25494;
            p92.chan1_raw = (ushort)(ushort)26856;
            p92.chan8_raw = (ushort)(ushort)25204;
            p92.chan5_raw = (ushort)(ushort)47782;
            p92.chan10_raw = (ushort)(ushort)24818;
            CommunicationChannel.instance.send(p92);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_ACTUATOR_CONTROLSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.flags == (ulong)5943871888806109926L);
                Debug.Assert(pack.controls.SequenceEqual(new float[] {6.8210235E37F, 1.8496383E38F, -1.8243232E38F, -8.240862E37F, 2.636493E38F, 3.2503018E38F, 3.833209E37F, -2.6058521E38F, -2.3622334E38F, -4.990025E37F, -8.74282E37F, -3.3806532E38F, -1.2341666E38F, 3.3676183E38F, 1.5816056E38F, 1.4153049E37F}));
                Debug.Assert(pack.mode == MAV_MODE.MAV_MODE_GUIDED_DISARMED);
                Debug.Assert(pack.time_usec == (ulong)2286363571351175140L);
            };
            GroundControl.HIL_ACTUATOR_CONTROLS p93 = CommunicationChannel.new_HIL_ACTUATOR_CONTROLS();
            PH.setPack(p93);
            p93.controls_SET(new float[] {6.8210235E37F, 1.8496383E38F, -1.8243232E38F, -8.240862E37F, 2.636493E38F, 3.2503018E38F, 3.833209E37F, -2.6058521E38F, -2.3622334E38F, -4.990025E37F, -8.74282E37F, -3.3806532E38F, -1.2341666E38F, 3.3676183E38F, 1.5816056E38F, 1.4153049E37F}, 0) ;
            p93.time_usec = (ulong)2286363571351175140L;
            p93.flags = (ulong)5943871888806109926L;
            p93.mode = MAV_MODE.MAV_MODE_GUIDED_DISARMED;
            CommunicationChannel.instance.send(p93);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnOPTICAL_FLOWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.quality == (byte)(byte)15);
                Debug.Assert(pack.flow_x == (short)(short) -27016);
                Debug.Assert(pack.flow_rate_x_TRY(ph) == (float)1.8696392E38F);
                Debug.Assert(pack.flow_comp_m_x == (float)1.3665042E38F);
                Debug.Assert(pack.sensor_id == (byte)(byte)36);
                Debug.Assert(pack.flow_y == (short)(short) -10720);
                Debug.Assert(pack.flow_rate_y_TRY(ph) == (float)1.1404269E38F);
                Debug.Assert(pack.ground_distance == (float)2.279647E37F);
                Debug.Assert(pack.flow_comp_m_y == (float) -4.3782272E36F);
                Debug.Assert(pack.time_usec == (ulong)7343319042553126430L);
            };
            GroundControl.OPTICAL_FLOW p100 = CommunicationChannel.new_OPTICAL_FLOW();
            PH.setPack(p100);
            p100.sensor_id = (byte)(byte)36;
            p100.quality = (byte)(byte)15;
            p100.flow_rate_y_SET((float)1.1404269E38F, PH) ;
            p100.flow_comp_m_y = (float) -4.3782272E36F;
            p100.time_usec = (ulong)7343319042553126430L;
            p100.ground_distance = (float)2.279647E37F;
            p100.flow_comp_m_x = (float)1.3665042E38F;
            p100.flow_rate_x_SET((float)1.8696392E38F, PH) ;
            p100.flow_x = (short)(short) -27016;
            p100.flow_y = (short)(short) -10720;
            CommunicationChannel.instance.send(p100);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGLOBAL_VISION_POSITION_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.y == (float)2.5459766E38F);
                Debug.Assert(pack.roll == (float) -1.7993724E38F);
                Debug.Assert(pack.pitch == (float)2.6538864E38F);
                Debug.Assert(pack.usec == (ulong)6120239010797408960L);
                Debug.Assert(pack.z == (float)1.2227615E38F);
                Debug.Assert(pack.x == (float) -3.373502E38F);
                Debug.Assert(pack.yaw == (float) -1.4985553E38F);
            };
            GroundControl.GLOBAL_VISION_POSITION_ESTIMATE p101 = CommunicationChannel.new_GLOBAL_VISION_POSITION_ESTIMATE();
            PH.setPack(p101);
            p101.y = (float)2.5459766E38F;
            p101.x = (float) -3.373502E38F;
            p101.z = (float)1.2227615E38F;
            p101.usec = (ulong)6120239010797408960L;
            p101.yaw = (float) -1.4985553E38F;
            p101.roll = (float) -1.7993724E38F;
            p101.pitch = (float)2.6538864E38F;
            CommunicationChannel.instance.send(p101);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnVISION_POSITION_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.roll == (float)1.722899E38F);
                Debug.Assert(pack.yaw == (float)1.7102718E38F);
                Debug.Assert(pack.usec == (ulong)4095470329185091340L);
                Debug.Assert(pack.z == (float) -2.6735169E38F);
                Debug.Assert(pack.x == (float) -4.513149E37F);
                Debug.Assert(pack.y == (float) -1.9480727E38F);
                Debug.Assert(pack.pitch == (float)3.392786E38F);
            };
            GroundControl.VISION_POSITION_ESTIMATE p102 = CommunicationChannel.new_VISION_POSITION_ESTIMATE();
            PH.setPack(p102);
            p102.roll = (float)1.722899E38F;
            p102.y = (float) -1.9480727E38F;
            p102.x = (float) -4.513149E37F;
            p102.yaw = (float)1.7102718E38F;
            p102.z = (float) -2.6735169E38F;
            p102.pitch = (float)3.392786E38F;
            p102.usec = (ulong)4095470329185091340L;
            CommunicationChannel.instance.send(p102);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnVISION_SPEED_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.x == (float)3.1296652E38F);
                Debug.Assert(pack.z == (float) -2.1140907E38F);
                Debug.Assert(pack.usec == (ulong)1553232588362658242L);
                Debug.Assert(pack.y == (float) -1.3734095E38F);
            };
            GroundControl.VISION_SPEED_ESTIMATE p103 = CommunicationChannel.new_VISION_SPEED_ESTIMATE();
            PH.setPack(p103);
            p103.z = (float) -2.1140907E38F;
            p103.x = (float)3.1296652E38F;
            p103.usec = (ulong)1553232588362658242L;
            p103.y = (float) -1.3734095E38F;
            CommunicationChannel.instance.send(p103);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnVICON_POSITION_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.z == (float)9.154966E37F);
                Debug.Assert(pack.y == (float) -1.5324062E38F);
                Debug.Assert(pack.x == (float)2.7037444E38F);
                Debug.Assert(pack.pitch == (float) -2.7633394E38F);
                Debug.Assert(pack.roll == (float)2.398301E38F);
                Debug.Assert(pack.usec == (ulong)7386464769429880225L);
                Debug.Assert(pack.yaw == (float) -2.3849473E38F);
            };
            GroundControl.VICON_POSITION_ESTIMATE p104 = CommunicationChannel.new_VICON_POSITION_ESTIMATE();
            PH.setPack(p104);
            p104.pitch = (float) -2.7633394E38F;
            p104.y = (float) -1.5324062E38F;
            p104.x = (float)2.7037444E38F;
            p104.yaw = (float) -2.3849473E38F;
            p104.roll = (float)2.398301E38F;
            p104.z = (float)9.154966E37F;
            p104.usec = (ulong)7386464769429880225L;
            CommunicationChannel.instance.send(p104);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIGHRES_IMUReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.xmag == (float)1.8322721E38F);
                Debug.Assert(pack.zmag == (float)3.8838446E37F);
                Debug.Assert(pack.ymag == (float) -2.6050342E38F);
                Debug.Assert(pack.temperature == (float) -7.296533E37F);
                Debug.Assert(pack.pressure_alt == (float)2.684322E38F);
                Debug.Assert(pack.fields_updated == (ushort)(ushort)53216);
                Debug.Assert(pack.yacc == (float)3.3211089E38F);
                Debug.Assert(pack.zgyro == (float) -2.0901922E38F);
                Debug.Assert(pack.abs_pressure == (float) -1.7998369E38F);
                Debug.Assert(pack.diff_pressure == (float)7.699182E37F);
                Debug.Assert(pack.xacc == (float) -1.6696507E38F);
                Debug.Assert(pack.xgyro == (float)2.3438647E37F);
                Debug.Assert(pack.zacc == (float)3.0657706E38F);
                Debug.Assert(pack.ygyro == (float) -2.2604847E38F);
                Debug.Assert(pack.time_usec == (ulong)94619446281096029L);
            };
            GroundControl.HIGHRES_IMU p105 = CommunicationChannel.new_HIGHRES_IMU();
            PH.setPack(p105);
            p105.ymag = (float) -2.6050342E38F;
            p105.fields_updated = (ushort)(ushort)53216;
            p105.diff_pressure = (float)7.699182E37F;
            p105.zgyro = (float) -2.0901922E38F;
            p105.yacc = (float)3.3211089E38F;
            p105.xacc = (float) -1.6696507E38F;
            p105.zmag = (float)3.8838446E37F;
            p105.temperature = (float) -7.296533E37F;
            p105.abs_pressure = (float) -1.7998369E38F;
            p105.xgyro = (float)2.3438647E37F;
            p105.ygyro = (float) -2.2604847E38F;
            p105.pressure_alt = (float)2.684322E38F;
            p105.time_usec = (ulong)94619446281096029L;
            p105.zacc = (float)3.0657706E38F;
            p105.xmag = (float)1.8322721E38F;
            CommunicationChannel.instance.send(p105);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnOPTICAL_FLOW_RADReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.integrated_zgyro == (float)2.2453891E38F);
                Debug.Assert(pack.quality == (byte)(byte)101);
                Debug.Assert(pack.time_delta_distance_us == (uint)2716176932U);
                Debug.Assert(pack.integrated_y == (float) -1.3057565E38F);
                Debug.Assert(pack.integrated_xgyro == (float)7.904886E37F);
                Debug.Assert(pack.integrated_x == (float)8.019958E37F);
                Debug.Assert(pack.temperature == (short)(short)4223);
                Debug.Assert(pack.time_usec == (ulong)604571245248422975L);
                Debug.Assert(pack.distance == (float) -6.5614553E37F);
                Debug.Assert(pack.integrated_ygyro == (float)1.0835607E37F);
                Debug.Assert(pack.sensor_id == (byte)(byte)58);
                Debug.Assert(pack.integration_time_us == (uint)1859231606U);
            };
            GroundControl.OPTICAL_FLOW_RAD p106 = CommunicationChannel.new_OPTICAL_FLOW_RAD();
            PH.setPack(p106);
            p106.integrated_zgyro = (float)2.2453891E38F;
            p106.quality = (byte)(byte)101;
            p106.integrated_y = (float) -1.3057565E38F;
            p106.temperature = (short)(short)4223;
            p106.distance = (float) -6.5614553E37F;
            p106.integrated_xgyro = (float)7.904886E37F;
            p106.time_delta_distance_us = (uint)2716176932U;
            p106.integrated_ygyro = (float)1.0835607E37F;
            p106.integration_time_us = (uint)1859231606U;
            p106.sensor_id = (byte)(byte)58;
            p106.integrated_x = (float)8.019958E37F;
            p106.time_usec = (ulong)604571245248422975L;
            CommunicationChannel.instance.send(p106);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_SENSORReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.zacc == (float) -1.2624458E38F);
                Debug.Assert(pack.time_usec == (ulong)207459516659056325L);
                Debug.Assert(pack.ygyro == (float)3.3806968E38F);
                Debug.Assert(pack.ymag == (float) -2.1531575E38F);
                Debug.Assert(pack.yacc == (float) -1.3035251E38F);
                Debug.Assert(pack.zmag == (float)5.7710133E37F);
                Debug.Assert(pack.zgyro == (float)2.8272987E38F);
                Debug.Assert(pack.xacc == (float) -2.1249014E38F);
                Debug.Assert(pack.pressure_alt == (float)3.1062557E38F);
                Debug.Assert(pack.fields_updated == (uint)3379085645U);
                Debug.Assert(pack.abs_pressure == (float) -1.5737681E38F);
                Debug.Assert(pack.diff_pressure == (float)6.3843894E37F);
                Debug.Assert(pack.xmag == (float)3.8658047E37F);
                Debug.Assert(pack.temperature == (float) -2.0307343E38F);
                Debug.Assert(pack.xgyro == (float) -1.4909884E38F);
            };
            GroundControl.HIL_SENSOR p107 = CommunicationChannel.new_HIL_SENSOR();
            PH.setPack(p107);
            p107.xacc = (float) -2.1249014E38F;
            p107.xgyro = (float) -1.4909884E38F;
            p107.ymag = (float) -2.1531575E38F;
            p107.abs_pressure = (float) -1.5737681E38F;
            p107.yacc = (float) -1.3035251E38F;
            p107.zgyro = (float)2.8272987E38F;
            p107.diff_pressure = (float)6.3843894E37F;
            p107.zmag = (float)5.7710133E37F;
            p107.pressure_alt = (float)3.1062557E38F;
            p107.fields_updated = (uint)3379085645U;
            p107.xmag = (float)3.8658047E37F;
            p107.zacc = (float) -1.2624458E38F;
            p107.temperature = (float) -2.0307343E38F;
            p107.time_usec = (ulong)207459516659056325L;
            p107.ygyro = (float)3.3806968E38F;
            CommunicationChannel.instance.send(p107);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSIM_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.zgyro == (float)1.731246E38F);
                Debug.Assert(pack.roll == (float)2.4578344E38F);
                Debug.Assert(pack.q2 == (float) -2.1097704E38F);
                Debug.Assert(pack.yacc == (float)3.3194346E38F);
                Debug.Assert(pack.q4 == (float) -1.4446818E37F);
                Debug.Assert(pack.vd == (float)1.6827067E38F);
                Debug.Assert(pack.q1 == (float)2.9528913E38F);
                Debug.Assert(pack.alt == (float)2.836674E38F);
                Debug.Assert(pack.ve == (float) -1.4766077E38F);
                Debug.Assert(pack.vn == (float)1.990339E37F);
                Debug.Assert(pack.lon == (float) -2.132172E38F);
                Debug.Assert(pack.std_dev_horz == (float) -2.9887569E38F);
                Debug.Assert(pack.zacc == (float)2.7526664E38F);
                Debug.Assert(pack.xgyro == (float) -2.6456797E38F);
                Debug.Assert(pack.lat == (float)5.6058633E37F);
                Debug.Assert(pack.ygyro == (float)2.044502E37F);
                Debug.Assert(pack.xacc == (float) -1.4041648E38F);
                Debug.Assert(pack.std_dev_vert == (float)1.3737852E38F);
                Debug.Assert(pack.pitch == (float) -1.4484785E38F);
                Debug.Assert(pack.yaw == (float) -2.2412726E38F);
                Debug.Assert(pack.q3 == (float) -1.2336035E38F);
            };
            GroundControl.SIM_STATE p108 = CommunicationChannel.new_SIM_STATE();
            PH.setPack(p108);
            p108.ygyro = (float)2.044502E37F;
            p108.vd = (float)1.6827067E38F;
            p108.zgyro = (float)1.731246E38F;
            p108.xgyro = (float) -2.6456797E38F;
            p108.yaw = (float) -2.2412726E38F;
            p108.q2 = (float) -2.1097704E38F;
            p108.vn = (float)1.990339E37F;
            p108.std_dev_vert = (float)1.3737852E38F;
            p108.ve = (float) -1.4766077E38F;
            p108.roll = (float)2.4578344E38F;
            p108.pitch = (float) -1.4484785E38F;
            p108.std_dev_horz = (float) -2.9887569E38F;
            p108.alt = (float)2.836674E38F;
            p108.lon = (float) -2.132172E38F;
            p108.q3 = (float) -1.2336035E38F;
            p108.lat = (float)5.6058633E37F;
            p108.zacc = (float)2.7526664E38F;
            p108.q4 = (float) -1.4446818E37F;
            p108.xacc = (float) -1.4041648E38F;
            p108.q1 = (float)2.9528913E38F;
            p108.yacc = (float)3.3194346E38F;
            CommunicationChannel.instance.send(p108);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRADIO_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.remnoise == (byte)(byte)114);
                Debug.Assert(pack.fixed_ == (ushort)(ushort)30620);
                Debug.Assert(pack.rssi == (byte)(byte)229);
                Debug.Assert(pack.txbuf == (byte)(byte)19);
                Debug.Assert(pack.noise == (byte)(byte)239);
                Debug.Assert(pack.rxerrors == (ushort)(ushort)39512);
                Debug.Assert(pack.remrssi == (byte)(byte)189);
            };
            GroundControl.RADIO_STATUS p109 = CommunicationChannel.new_RADIO_STATUS();
            PH.setPack(p109);
            p109.noise = (byte)(byte)239;
            p109.rssi = (byte)(byte)229;
            p109.remnoise = (byte)(byte)114;
            p109.txbuf = (byte)(byte)19;
            p109.remrssi = (byte)(byte)189;
            p109.rxerrors = (ushort)(ushort)39512;
            p109.fixed_ = (ushort)(ushort)30620;
            CommunicationChannel.instance.send(p109);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnFILE_TRANSFER_PROTOCOLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_network == (byte)(byte)73);
                Debug.Assert(pack.target_system == (byte)(byte)228);
                Debug.Assert(pack.target_component == (byte)(byte)214);
                Debug.Assert(pack.payload.SequenceEqual(new byte[] {(byte)67, (byte)183, (byte)225, (byte)158, (byte)50, (byte)226, (byte)157, (byte)163, (byte)142, (byte)192, (byte)82, (byte)70, (byte)223, (byte)126, (byte)236, (byte)90, (byte)93, (byte)129, (byte)250, (byte)41, (byte)155, (byte)46, (byte)120, (byte)16, (byte)239, (byte)153, (byte)47, (byte)39, (byte)82, (byte)255, (byte)158, (byte)108, (byte)25, (byte)25, (byte)208, (byte)60, (byte)86, (byte)31, (byte)220, (byte)136, (byte)77, (byte)59, (byte)117, (byte)184, (byte)116, (byte)207, (byte)190, (byte)176, (byte)186, (byte)209, (byte)165, (byte)210, (byte)168, (byte)26, (byte)215, (byte)112, (byte)116, (byte)230, (byte)128, (byte)14, (byte)65, (byte)197, (byte)25, (byte)210, (byte)68, (byte)46, (byte)2, (byte)45, (byte)146, (byte)16, (byte)249, (byte)116, (byte)16, (byte)238, (byte)8, (byte)172, (byte)42, (byte)195, (byte)70, (byte)212, (byte)222, (byte)253, (byte)91, (byte)95, (byte)162, (byte)32, (byte)101, (byte)71, (byte)128, (byte)18, (byte)48, (byte)91, (byte)231, (byte)196, (byte)42, (byte)127, (byte)102, (byte)179, (byte)154, (byte)77, (byte)253, (byte)44, (byte)233, (byte)26, (byte)169, (byte)253, (byte)117, (byte)11, (byte)15, (byte)239, (byte)153, (byte)120, (byte)25, (byte)246, (byte)19, (byte)221, (byte)61, (byte)215, (byte)177, (byte)24, (byte)167, (byte)38, (byte)52, (byte)229, (byte)203, (byte)10, (byte)80, (byte)108, (byte)63, (byte)127, (byte)233, (byte)91, (byte)76, (byte)238, (byte)91, (byte)201, (byte)192, (byte)10, (byte)233, (byte)170, (byte)138, (byte)130, (byte)173, (byte)173, (byte)179, (byte)199, (byte)152, (byte)47, (byte)6, (byte)220, (byte)193, (byte)77, (byte)221, (byte)166, (byte)165, (byte)193, (byte)29, (byte)176, (byte)112, (byte)184, (byte)62, (byte)25, (byte)219, (byte)37, (byte)169, (byte)171, (byte)71, (byte)40, (byte)96, (byte)145, (byte)236, (byte)238, (byte)130, (byte)204, (byte)215, (byte)178, (byte)16, (byte)174, (byte)99, (byte)3, (byte)190, (byte)7, (byte)185, (byte)237, (byte)219, (byte)216, (byte)83, (byte)172, (byte)23, (byte)48, (byte)110, (byte)176, (byte)214, (byte)124, (byte)34, (byte)104, (byte)37, (byte)78, (byte)136, (byte)33, (byte)140, (byte)139, (byte)69, (byte)241, (byte)9, (byte)255, (byte)68, (byte)193, (byte)143, (byte)113, (byte)13, (byte)169, (byte)11, (byte)49, (byte)39, (byte)212, (byte)41, (byte)193, (byte)30, (byte)100, (byte)107, (byte)6, (byte)208, (byte)244, (byte)63, (byte)191, (byte)152, (byte)202, (byte)12, (byte)11, (byte)4, (byte)146, (byte)47, (byte)93, (byte)185, (byte)86, (byte)3, (byte)172, (byte)195, (byte)93, (byte)228, (byte)161, (byte)2, (byte)206, (byte)116, (byte)88, (byte)186, (byte)180, (byte)107, (byte)188, (byte)124}));
            };
            GroundControl.FILE_TRANSFER_PROTOCOL p110 = CommunicationChannel.new_FILE_TRANSFER_PROTOCOL();
            PH.setPack(p110);
            p110.target_network = (byte)(byte)73;
            p110.target_component = (byte)(byte)214;
            p110.target_system = (byte)(byte)228;
            p110.payload_SET(new byte[] {(byte)67, (byte)183, (byte)225, (byte)158, (byte)50, (byte)226, (byte)157, (byte)163, (byte)142, (byte)192, (byte)82, (byte)70, (byte)223, (byte)126, (byte)236, (byte)90, (byte)93, (byte)129, (byte)250, (byte)41, (byte)155, (byte)46, (byte)120, (byte)16, (byte)239, (byte)153, (byte)47, (byte)39, (byte)82, (byte)255, (byte)158, (byte)108, (byte)25, (byte)25, (byte)208, (byte)60, (byte)86, (byte)31, (byte)220, (byte)136, (byte)77, (byte)59, (byte)117, (byte)184, (byte)116, (byte)207, (byte)190, (byte)176, (byte)186, (byte)209, (byte)165, (byte)210, (byte)168, (byte)26, (byte)215, (byte)112, (byte)116, (byte)230, (byte)128, (byte)14, (byte)65, (byte)197, (byte)25, (byte)210, (byte)68, (byte)46, (byte)2, (byte)45, (byte)146, (byte)16, (byte)249, (byte)116, (byte)16, (byte)238, (byte)8, (byte)172, (byte)42, (byte)195, (byte)70, (byte)212, (byte)222, (byte)253, (byte)91, (byte)95, (byte)162, (byte)32, (byte)101, (byte)71, (byte)128, (byte)18, (byte)48, (byte)91, (byte)231, (byte)196, (byte)42, (byte)127, (byte)102, (byte)179, (byte)154, (byte)77, (byte)253, (byte)44, (byte)233, (byte)26, (byte)169, (byte)253, (byte)117, (byte)11, (byte)15, (byte)239, (byte)153, (byte)120, (byte)25, (byte)246, (byte)19, (byte)221, (byte)61, (byte)215, (byte)177, (byte)24, (byte)167, (byte)38, (byte)52, (byte)229, (byte)203, (byte)10, (byte)80, (byte)108, (byte)63, (byte)127, (byte)233, (byte)91, (byte)76, (byte)238, (byte)91, (byte)201, (byte)192, (byte)10, (byte)233, (byte)170, (byte)138, (byte)130, (byte)173, (byte)173, (byte)179, (byte)199, (byte)152, (byte)47, (byte)6, (byte)220, (byte)193, (byte)77, (byte)221, (byte)166, (byte)165, (byte)193, (byte)29, (byte)176, (byte)112, (byte)184, (byte)62, (byte)25, (byte)219, (byte)37, (byte)169, (byte)171, (byte)71, (byte)40, (byte)96, (byte)145, (byte)236, (byte)238, (byte)130, (byte)204, (byte)215, (byte)178, (byte)16, (byte)174, (byte)99, (byte)3, (byte)190, (byte)7, (byte)185, (byte)237, (byte)219, (byte)216, (byte)83, (byte)172, (byte)23, (byte)48, (byte)110, (byte)176, (byte)214, (byte)124, (byte)34, (byte)104, (byte)37, (byte)78, (byte)136, (byte)33, (byte)140, (byte)139, (byte)69, (byte)241, (byte)9, (byte)255, (byte)68, (byte)193, (byte)143, (byte)113, (byte)13, (byte)169, (byte)11, (byte)49, (byte)39, (byte)212, (byte)41, (byte)193, (byte)30, (byte)100, (byte)107, (byte)6, (byte)208, (byte)244, (byte)63, (byte)191, (byte)152, (byte)202, (byte)12, (byte)11, (byte)4, (byte)146, (byte)47, (byte)93, (byte)185, (byte)86, (byte)3, (byte)172, (byte)195, (byte)93, (byte)228, (byte)161, (byte)2, (byte)206, (byte)116, (byte)88, (byte)186, (byte)180, (byte)107, (byte)188, (byte)124}, 0) ;
            CommunicationChannel.instance.send(p110);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTIMESYNCReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.tc1 == (long) -779643165414160288L);
                Debug.Assert(pack.ts1 == (long)2777444048409808183L);
            };
            GroundControl.TIMESYNC p111 = CommunicationChannel.new_TIMESYNC();
            PH.setPack(p111);
            p111.ts1 = (long)2777444048409808183L;
            p111.tc1 = (long) -779643165414160288L;
            CommunicationChannel.instance.send(p111);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCAMERA_TRIGGERReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (uint)4271736887U);
                Debug.Assert(pack.time_usec == (ulong)8407758099314616177L);
            };
            GroundControl.CAMERA_TRIGGER p112 = CommunicationChannel.new_CAMERA_TRIGGER();
            PH.setPack(p112);
            p112.time_usec = (ulong)8407758099314616177L;
            p112.seq = (uint)4271736887U;
            CommunicationChannel.instance.send(p112);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_GPSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vel == (ushort)(ushort)34432);
                Debug.Assert(pack.fix_type == (byte)(byte)247);
                Debug.Assert(pack.alt == (int) -1080870394);
                Debug.Assert(pack.lat == (int) -103755026);
                Debug.Assert(pack.vd == (short)(short)317);
                Debug.Assert(pack.vn == (short)(short)31484);
                Debug.Assert(pack.time_usec == (ulong)4580765694053665453L);
                Debug.Assert(pack.epv == (ushort)(ushort)36579);
                Debug.Assert(pack.ve == (short)(short) -5989);
                Debug.Assert(pack.eph == (ushort)(ushort)35700);
                Debug.Assert(pack.cog == (ushort)(ushort)1953);
                Debug.Assert(pack.satellites_visible == (byte)(byte)43);
                Debug.Assert(pack.lon == (int) -2032513594);
            };
            GroundControl.HIL_GPS p113 = CommunicationChannel.new_HIL_GPS();
            PH.setPack(p113);
            p113.eph = (ushort)(ushort)35700;
            p113.satellites_visible = (byte)(byte)43;
            p113.cog = (ushort)(ushort)1953;
            p113.alt = (int) -1080870394;
            p113.fix_type = (byte)(byte)247;
            p113.ve = (short)(short) -5989;
            p113.vn = (short)(short)31484;
            p113.epv = (ushort)(ushort)36579;
            p113.vd = (short)(short)317;
            p113.lon = (int) -2032513594;
            p113.vel = (ushort)(ushort)34432;
            p113.time_usec = (ulong)4580765694053665453L;
            p113.lat = (int) -103755026;
            CommunicationChannel.instance.send(p113);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_OPTICAL_FLOWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.integrated_x == (float) -2.4538939E38F);
                Debug.Assert(pack.integrated_ygyro == (float) -1.6594389E38F);
                Debug.Assert(pack.time_delta_distance_us == (uint)224464437U);
                Debug.Assert(pack.distance == (float) -1.3611681E38F);
                Debug.Assert(pack.integrated_y == (float)2.479682E38F);
                Debug.Assert(pack.integrated_xgyro == (float)1.98662E38F);
                Debug.Assert(pack.integration_time_us == (uint)1466647762U);
                Debug.Assert(pack.temperature == (short)(short)6783);
                Debug.Assert(pack.sensor_id == (byte)(byte)124);
                Debug.Assert(pack.integrated_zgyro == (float)3.039848E38F);
                Debug.Assert(pack.quality == (byte)(byte)75);
                Debug.Assert(pack.time_usec == (ulong)8842997845726409417L);
            };
            GroundControl.HIL_OPTICAL_FLOW p114 = CommunicationChannel.new_HIL_OPTICAL_FLOW();
            PH.setPack(p114);
            p114.quality = (byte)(byte)75;
            p114.temperature = (short)(short)6783;
            p114.integrated_xgyro = (float)1.98662E38F;
            p114.time_usec = (ulong)8842997845726409417L;
            p114.integrated_zgyro = (float)3.039848E38F;
            p114.sensor_id = (byte)(byte)124;
            p114.time_delta_distance_us = (uint)224464437U;
            p114.integrated_ygyro = (float) -1.6594389E38F;
            p114.integrated_y = (float)2.479682E38F;
            p114.distance = (float) -1.3611681E38F;
            p114.integration_time_us = (uint)1466647762U;
            p114.integrated_x = (float) -2.4538939E38F;
            CommunicationChannel.instance.send(p114);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_STATE_QUATERNIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vx == (short)(short) -6374);
                Debug.Assert(pack.ind_airspeed == (ushort)(ushort)5920);
                Debug.Assert(pack.alt == (int) -1827786508);
                Debug.Assert(pack.xacc == (short)(short) -21715);
                Debug.Assert(pack.yacc == (short)(short)25869);
                Debug.Assert(pack.vz == (short)(short)14537);
                Debug.Assert(pack.time_usec == (ulong)8357132671951161313L);
                Debug.Assert(pack.yawspeed == (float)2.0398358E38F);
                Debug.Assert(pack.vy == (short)(short) -12990);
                Debug.Assert(pack.pitchspeed == (float) -1.8994892E38F);
                Debug.Assert(pack.lat == (int) -912532327);
                Debug.Assert(pack.lon == (int) -35295833);
                Debug.Assert(pack.zacc == (short)(short)25625);
                Debug.Assert(pack.true_airspeed == (ushort)(ushort)3213);
                Debug.Assert(pack.rollspeed == (float) -1.4423733E38F);
                Debug.Assert(pack.attitude_quaternion.SequenceEqual(new float[] {-1.2537006E38F, 9.477775E37F, -2.9592837E38F, 1.4641608E38F}));
            };
            GroundControl.HIL_STATE_QUATERNION p115 = CommunicationChannel.new_HIL_STATE_QUATERNION();
            PH.setPack(p115);
            p115.zacc = (short)(short)25625;
            p115.true_airspeed = (ushort)(ushort)3213;
            p115.vz = (short)(short)14537;
            p115.lat = (int) -912532327;
            p115.xacc = (short)(short) -21715;
            p115.ind_airspeed = (ushort)(ushort)5920;
            p115.vy = (short)(short) -12990;
            p115.rollspeed = (float) -1.4423733E38F;
            p115.vx = (short)(short) -6374;
            p115.yacc = (short)(short)25869;
            p115.yawspeed = (float)2.0398358E38F;
            p115.pitchspeed = (float) -1.8994892E38F;
            p115.alt = (int) -1827786508;
            p115.lon = (int) -35295833;
            p115.time_usec = (ulong)8357132671951161313L;
            p115.attitude_quaternion_SET(new float[] {-1.2537006E38F, 9.477775E37F, -2.9592837E38F, 1.4641608E38F}, 0) ;
            CommunicationChannel.instance.send(p115);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_IMU2Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)2656122163U);
                Debug.Assert(pack.xacc == (short)(short) -26093);
                Debug.Assert(pack.ymag == (short)(short) -32532);
                Debug.Assert(pack.ygyro == (short)(short) -31635);
                Debug.Assert(pack.xgyro == (short)(short) -30983);
                Debug.Assert(pack.xmag == (short)(short) -26690);
                Debug.Assert(pack.zmag == (short)(short) -30988);
                Debug.Assert(pack.zacc == (short)(short)24017);
                Debug.Assert(pack.zgyro == (short)(short) -20506);
                Debug.Assert(pack.yacc == (short)(short) -25380);
            };
            GroundControl.SCALED_IMU2 p116 = CommunicationChannel.new_SCALED_IMU2();
            PH.setPack(p116);
            p116.yacc = (short)(short) -25380;
            p116.xgyro = (short)(short) -30983;
            p116.xmag = (short)(short) -26690;
            p116.ymag = (short)(short) -32532;
            p116.zacc = (short)(short)24017;
            p116.zmag = (short)(short) -30988;
            p116.time_boot_ms = (uint)2656122163U;
            p116.ygyro = (short)(short) -31635;
            p116.zgyro = (short)(short) -20506;
            p116.xacc = (short)(short) -26093;
            CommunicationChannel.instance.send(p116);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)182);
                Debug.Assert(pack.end == (ushort)(ushort)10829);
                Debug.Assert(pack.start == (ushort)(ushort)65355);
                Debug.Assert(pack.target_system == (byte)(byte)111);
            };
            GroundControl.LOG_REQUEST_LIST p117 = CommunicationChannel.new_LOG_REQUEST_LIST();
            PH.setPack(p117);
            p117.target_component = (byte)(byte)182;
            p117.target_system = (byte)(byte)111;
            p117.end = (ushort)(ushort)10829;
            p117.start = (ushort)(ushort)65355;
            CommunicationChannel.instance.send(p117);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_ENTRYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_utc == (uint)3740798904U);
                Debug.Assert(pack.last_log_num == (ushort)(ushort)44620);
                Debug.Assert(pack.id == (ushort)(ushort)39166);
                Debug.Assert(pack.num_logs == (ushort)(ushort)30187);
                Debug.Assert(pack.size == (uint)2987681495U);
            };
            GroundControl.LOG_ENTRY p118 = CommunicationChannel.new_LOG_ENTRY();
            PH.setPack(p118);
            p118.id = (ushort)(ushort)39166;
            p118.num_logs = (ushort)(ushort)30187;
            p118.last_log_num = (ushort)(ushort)44620;
            p118.time_utc = (uint)3740798904U;
            p118.size = (uint)2987681495U;
            CommunicationChannel.instance.send(p118);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_REQUEST_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.id == (ushort)(ushort)38923);
                Debug.Assert(pack.target_component == (byte)(byte)160);
                Debug.Assert(pack.ofs == (uint)3110461432U);
                Debug.Assert(pack.target_system == (byte)(byte)190);
                Debug.Assert(pack.count == (uint)1309084180U);
            };
            GroundControl.LOG_REQUEST_DATA p119 = CommunicationChannel.new_LOG_REQUEST_DATA();
            PH.setPack(p119);
            p119.target_component = (byte)(byte)160;
            p119.target_system = (byte)(byte)190;
            p119.count = (uint)1309084180U;
            p119.id = (ushort)(ushort)38923;
            p119.ofs = (uint)3110461432U;
            CommunicationChannel.instance.send(p119);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ofs == (uint)3074031428U);
                Debug.Assert(pack.id == (ushort)(ushort)22945);
                Debug.Assert(pack.count == (byte)(byte)212);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)130, (byte)11, (byte)98, (byte)183, (byte)28, (byte)206, (byte)121, (byte)244, (byte)41, (byte)220, (byte)185, (byte)61, (byte)55, (byte)242, (byte)51, (byte)105, (byte)245, (byte)47, (byte)59, (byte)242, (byte)43, (byte)56, (byte)214, (byte)46, (byte)47, (byte)129, (byte)216, (byte)26, (byte)243, (byte)95, (byte)8, (byte)240, (byte)44, (byte)61, (byte)245, (byte)130, (byte)222, (byte)253, (byte)123, (byte)199, (byte)238, (byte)201, (byte)89, (byte)122, (byte)226, (byte)183, (byte)240, (byte)237, (byte)15, (byte)206, (byte)172, (byte)219, (byte)185, (byte)203, (byte)102, (byte)174, (byte)28, (byte)43, (byte)131, (byte)43, (byte)102, (byte)160, (byte)114, (byte)62, (byte)87, (byte)94, (byte)252, (byte)26, (byte)31, (byte)114, (byte)43, (byte)65, (byte)71, (byte)70, (byte)47, (byte)79, (byte)87, (byte)25, (byte)53, (byte)151, (byte)86, (byte)206, (byte)63, (byte)160, (byte)185, (byte)164, (byte)91, (byte)183, (byte)89, (byte)72}));
            };
            GroundControl.LOG_DATA p120 = CommunicationChannel.new_LOG_DATA();
            PH.setPack(p120);
            p120.ofs = (uint)3074031428U;
            p120.count = (byte)(byte)212;
            p120.id = (ushort)(ushort)22945;
            p120.data__SET(new byte[] {(byte)130, (byte)11, (byte)98, (byte)183, (byte)28, (byte)206, (byte)121, (byte)244, (byte)41, (byte)220, (byte)185, (byte)61, (byte)55, (byte)242, (byte)51, (byte)105, (byte)245, (byte)47, (byte)59, (byte)242, (byte)43, (byte)56, (byte)214, (byte)46, (byte)47, (byte)129, (byte)216, (byte)26, (byte)243, (byte)95, (byte)8, (byte)240, (byte)44, (byte)61, (byte)245, (byte)130, (byte)222, (byte)253, (byte)123, (byte)199, (byte)238, (byte)201, (byte)89, (byte)122, (byte)226, (byte)183, (byte)240, (byte)237, (byte)15, (byte)206, (byte)172, (byte)219, (byte)185, (byte)203, (byte)102, (byte)174, (byte)28, (byte)43, (byte)131, (byte)43, (byte)102, (byte)160, (byte)114, (byte)62, (byte)87, (byte)94, (byte)252, (byte)26, (byte)31, (byte)114, (byte)43, (byte)65, (byte)71, (byte)70, (byte)47, (byte)79, (byte)87, (byte)25, (byte)53, (byte)151, (byte)86, (byte)206, (byte)63, (byte)160, (byte)185, (byte)164, (byte)91, (byte)183, (byte)89, (byte)72}, 0) ;
            CommunicationChannel.instance.send(p120);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_ERASEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)165);
                Debug.Assert(pack.target_system == (byte)(byte)134);
            };
            GroundControl.LOG_ERASE p121 = CommunicationChannel.new_LOG_ERASE();
            PH.setPack(p121);
            p121.target_system = (byte)(byte)134;
            p121.target_component = (byte)(byte)165;
            CommunicationChannel.instance.send(p121);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_REQUEST_ENDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)136);
                Debug.Assert(pack.target_component == (byte)(byte)247);
            };
            GroundControl.LOG_REQUEST_END p122 = CommunicationChannel.new_LOG_REQUEST_END();
            PH.setPack(p122);
            p122.target_component = (byte)(byte)247;
            p122.target_system = (byte)(byte)136;
            CommunicationChannel.instance.send(p122);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_INJECT_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)208);
                Debug.Assert(pack.len == (byte)(byte)26);
                Debug.Assert(pack.target_component == (byte)(byte)20);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)164, (byte)33, (byte)130, (byte)122, (byte)36, (byte)9, (byte)169, (byte)43, (byte)83, (byte)7, (byte)63, (byte)138, (byte)73, (byte)113, (byte)83, (byte)5, (byte)171, (byte)60, (byte)204, (byte)226, (byte)59, (byte)82, (byte)109, (byte)233, (byte)148, (byte)11, (byte)181, (byte)17, (byte)92, (byte)150, (byte)233, (byte)26, (byte)88, (byte)69, (byte)152, (byte)89, (byte)40, (byte)190, (byte)125, (byte)46, (byte)200, (byte)8, (byte)206, (byte)46, (byte)185, (byte)26, (byte)108, (byte)231, (byte)141, (byte)99, (byte)238, (byte)50, (byte)38, (byte)177, (byte)255, (byte)100, (byte)65, (byte)255, (byte)166, (byte)10, (byte)5, (byte)14, (byte)243, (byte)252, (byte)245, (byte)216, (byte)56, (byte)157, (byte)244, (byte)124, (byte)75, (byte)85, (byte)174, (byte)207, (byte)228, (byte)175, (byte)116, (byte)1, (byte)89, (byte)149, (byte)223, (byte)254, (byte)111, (byte)240, (byte)200, (byte)238, (byte)210, (byte)142, (byte)216, (byte)111, (byte)143, (byte)123, (byte)129, (byte)120, (byte)144, (byte)218, (byte)40, (byte)167, (byte)255, (byte)39, (byte)104, (byte)183, (byte)236, (byte)114, (byte)199, (byte)140, (byte)95, (byte)174, (byte)130, (byte)110}));
            };
            GroundControl.GPS_INJECT_DATA p123 = CommunicationChannel.new_GPS_INJECT_DATA();
            PH.setPack(p123);
            p123.target_component = (byte)(byte)20;
            p123.data__SET(new byte[] {(byte)164, (byte)33, (byte)130, (byte)122, (byte)36, (byte)9, (byte)169, (byte)43, (byte)83, (byte)7, (byte)63, (byte)138, (byte)73, (byte)113, (byte)83, (byte)5, (byte)171, (byte)60, (byte)204, (byte)226, (byte)59, (byte)82, (byte)109, (byte)233, (byte)148, (byte)11, (byte)181, (byte)17, (byte)92, (byte)150, (byte)233, (byte)26, (byte)88, (byte)69, (byte)152, (byte)89, (byte)40, (byte)190, (byte)125, (byte)46, (byte)200, (byte)8, (byte)206, (byte)46, (byte)185, (byte)26, (byte)108, (byte)231, (byte)141, (byte)99, (byte)238, (byte)50, (byte)38, (byte)177, (byte)255, (byte)100, (byte)65, (byte)255, (byte)166, (byte)10, (byte)5, (byte)14, (byte)243, (byte)252, (byte)245, (byte)216, (byte)56, (byte)157, (byte)244, (byte)124, (byte)75, (byte)85, (byte)174, (byte)207, (byte)228, (byte)175, (byte)116, (byte)1, (byte)89, (byte)149, (byte)223, (byte)254, (byte)111, (byte)240, (byte)200, (byte)238, (byte)210, (byte)142, (byte)216, (byte)111, (byte)143, (byte)123, (byte)129, (byte)120, (byte)144, (byte)218, (byte)40, (byte)167, (byte)255, (byte)39, (byte)104, (byte)183, (byte)236, (byte)114, (byte)199, (byte)140, (byte)95, (byte)174, (byte)130, (byte)110}, 0) ;
            p123.len = (byte)(byte)26;
            p123.target_system = (byte)(byte)208;
            CommunicationChannel.instance.send(p123);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS2_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.eph == (ushort)(ushort)29319);
                Debug.Assert(pack.lat == (int)280683279);
                Debug.Assert(pack.alt == (int) -1269839627);
                Debug.Assert(pack.cog == (ushort)(ushort)41508);
                Debug.Assert(pack.lon == (int)497211756);
                Debug.Assert(pack.dgps_numch == (byte)(byte)169);
                Debug.Assert(pack.vel == (ushort)(ushort)20990);
                Debug.Assert(pack.dgps_age == (uint)2728204668U);
                Debug.Assert(pack.fix_type == GPS_FIX_TYPE.GPS_FIX_TYPE_RTK_FLOAT);
                Debug.Assert(pack.satellites_visible == (byte)(byte)96);
                Debug.Assert(pack.time_usec == (ulong)4044149686401523520L);
                Debug.Assert(pack.epv == (ushort)(ushort)57898);
            };
            GroundControl.GPS2_RAW p124 = CommunicationChannel.new_GPS2_RAW();
            PH.setPack(p124);
            p124.fix_type = GPS_FIX_TYPE.GPS_FIX_TYPE_RTK_FLOAT;
            p124.lat = (int)280683279;
            p124.dgps_numch = (byte)(byte)169;
            p124.alt = (int) -1269839627;
            p124.dgps_age = (uint)2728204668U;
            p124.epv = (ushort)(ushort)57898;
            p124.satellites_visible = (byte)(byte)96;
            p124.vel = (ushort)(ushort)20990;
            p124.eph = (ushort)(ushort)29319;
            p124.cog = (ushort)(ushort)41508;
            p124.time_usec = (ulong)4044149686401523520L;
            p124.lon = (int)497211756;
            CommunicationChannel.instance.send(p124);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPOWER_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.Vcc == (ushort)(ushort)54350);
                Debug.Assert(pack.flags == (MAV_POWER_STATUS.MAV_POWER_STATUS_CHANGED));
                Debug.Assert(pack.Vservo == (ushort)(ushort)4342);
            };
            GroundControl.POWER_STATUS p125 = CommunicationChannel.new_POWER_STATUS();
            PH.setPack(p125);
            p125.Vcc = (ushort)(ushort)54350;
            p125.Vservo = (ushort)(ushort)4342;
            p125.flags = (MAV_POWER_STATUS.MAV_POWER_STATUS_CHANGED);
            CommunicationChannel.instance.send(p125);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSERIAL_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)158, (byte)134, (byte)189, (byte)5, (byte)195, (byte)226, (byte)209, (byte)51, (byte)147, (byte)115, (byte)254, (byte)48, (byte)183, (byte)44, (byte)218, (byte)217, (byte)109, (byte)68, (byte)129, (byte)249, (byte)17, (byte)34, (byte)124, (byte)241, (byte)149, (byte)4, (byte)84, (byte)199, (byte)225, (byte)61, (byte)204, (byte)220, (byte)31, (byte)224, (byte)204, (byte)209, (byte)166, (byte)143, (byte)81, (byte)192, (byte)221, (byte)129, (byte)83, (byte)95, (byte)96, (byte)4, (byte)89, (byte)81, (byte)202, (byte)238, (byte)16, (byte)153, (byte)213, (byte)210, (byte)165, (byte)175, (byte)141, (byte)226, (byte)124, (byte)224, (byte)189, (byte)173, (byte)15, (byte)225, (byte)228, (byte)247, (byte)218, (byte)42, (byte)154, (byte)210}));
                Debug.Assert(pack.timeout == (ushort)(ushort)24075);
                Debug.Assert(pack.count == (byte)(byte)149);
                Debug.Assert(pack.device == SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_SHELL);
                Debug.Assert(pack.baudrate == (uint)3489091346U);
                Debug.Assert(pack.flags == (SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_REPLY));
            };
            GroundControl.SERIAL_CONTROL p126 = CommunicationChannel.new_SERIAL_CONTROL();
            PH.setPack(p126);
            p126.flags = (SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_REPLY);
            p126.timeout = (ushort)(ushort)24075;
            p126.device = SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_SHELL;
            p126.data__SET(new byte[] {(byte)158, (byte)134, (byte)189, (byte)5, (byte)195, (byte)226, (byte)209, (byte)51, (byte)147, (byte)115, (byte)254, (byte)48, (byte)183, (byte)44, (byte)218, (byte)217, (byte)109, (byte)68, (byte)129, (byte)249, (byte)17, (byte)34, (byte)124, (byte)241, (byte)149, (byte)4, (byte)84, (byte)199, (byte)225, (byte)61, (byte)204, (byte)220, (byte)31, (byte)224, (byte)204, (byte)209, (byte)166, (byte)143, (byte)81, (byte)192, (byte)221, (byte)129, (byte)83, (byte)95, (byte)96, (byte)4, (byte)89, (byte)81, (byte)202, (byte)238, (byte)16, (byte)153, (byte)213, (byte)210, (byte)165, (byte)175, (byte)141, (byte)226, (byte)124, (byte)224, (byte)189, (byte)173, (byte)15, (byte)225, (byte)228, (byte)247, (byte)218, (byte)42, (byte)154, (byte)210}, 0) ;
            p126.baudrate = (uint)3489091346U;
            p126.count = (byte)(byte)149;
            CommunicationChannel.instance.send(p126);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_RTKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.baseline_coords_type == (byte)(byte)50);
                Debug.Assert(pack.accuracy == (uint)746158058U);
                Debug.Assert(pack.rtk_receiver_id == (byte)(byte)116);
                Debug.Assert(pack.baseline_c_mm == (int)1544397756);
                Debug.Assert(pack.iar_num_hypotheses == (int)1918428720);
                Debug.Assert(pack.baseline_b_mm == (int) -1441278128);
                Debug.Assert(pack.rtk_health == (byte)(byte)117);
                Debug.Assert(pack.rtk_rate == (byte)(byte)186);
                Debug.Assert(pack.time_last_baseline_ms == (uint)1784723484U);
                Debug.Assert(pack.wn == (ushort)(ushort)4836);
                Debug.Assert(pack.baseline_a_mm == (int) -449535919);
                Debug.Assert(pack.nsats == (byte)(byte)126);
                Debug.Assert(pack.tow == (uint)4294526552U);
            };
            GroundControl.GPS_RTK p127 = CommunicationChannel.new_GPS_RTK();
            PH.setPack(p127);
            p127.baseline_coords_type = (byte)(byte)50;
            p127.accuracy = (uint)746158058U;
            p127.baseline_b_mm = (int) -1441278128;
            p127.baseline_c_mm = (int)1544397756;
            p127.rtk_health = (byte)(byte)117;
            p127.wn = (ushort)(ushort)4836;
            p127.baseline_a_mm = (int) -449535919;
            p127.nsats = (byte)(byte)126;
            p127.rtk_rate = (byte)(byte)186;
            p127.rtk_receiver_id = (byte)(byte)116;
            p127.tow = (uint)4294526552U;
            p127.time_last_baseline_ms = (uint)1784723484U;
            p127.iar_num_hypotheses = (int)1918428720;
            CommunicationChannel.instance.send(p127);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS2_RTKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.accuracy == (uint)1801749235U);
                Debug.Assert(pack.nsats == (byte)(byte)66);
                Debug.Assert(pack.rtk_receiver_id == (byte)(byte)216);
                Debug.Assert(pack.baseline_coords_type == (byte)(byte)251);
                Debug.Assert(pack.rtk_health == (byte)(byte)129);
                Debug.Assert(pack.iar_num_hypotheses == (int) -494995270);
                Debug.Assert(pack.baseline_a_mm == (int)2111521780);
                Debug.Assert(pack.wn == (ushort)(ushort)59064);
                Debug.Assert(pack.rtk_rate == (byte)(byte)156);
                Debug.Assert(pack.baseline_c_mm == (int)734923107);
                Debug.Assert(pack.baseline_b_mm == (int) -932538454);
                Debug.Assert(pack.time_last_baseline_ms == (uint)1031684824U);
                Debug.Assert(pack.tow == (uint)645886268U);
            };
            GroundControl.GPS2_RTK p128 = CommunicationChannel.new_GPS2_RTK();
            PH.setPack(p128);
            p128.baseline_a_mm = (int)2111521780;
            p128.rtk_receiver_id = (byte)(byte)216;
            p128.baseline_coords_type = (byte)(byte)251;
            p128.baseline_c_mm = (int)734923107;
            p128.wn = (ushort)(ushort)59064;
            p128.tow = (uint)645886268U;
            p128.accuracy = (uint)1801749235U;
            p128.rtk_health = (byte)(byte)129;
            p128.nsats = (byte)(byte)66;
            p128.rtk_rate = (byte)(byte)156;
            p128.baseline_b_mm = (int) -932538454;
            p128.iar_num_hypotheses = (int) -494995270;
            p128.time_last_baseline_ms = (uint)1031684824U;
            CommunicationChannel.instance.send(p128);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_IMU3Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ygyro == (short)(short)11717);
                Debug.Assert(pack.zacc == (short)(short)29888);
                Debug.Assert(pack.xacc == (short)(short)2223);
                Debug.Assert(pack.zgyro == (short)(short) -15789);
                Debug.Assert(pack.xmag == (short)(short)22174);
                Debug.Assert(pack.ymag == (short)(short)22034);
                Debug.Assert(pack.xgyro == (short)(short) -6509);
                Debug.Assert(pack.time_boot_ms == (uint)3930664330U);
                Debug.Assert(pack.zmag == (short)(short)19548);
                Debug.Assert(pack.yacc == (short)(short)12025);
            };
            GroundControl.SCALED_IMU3 p129 = CommunicationChannel.new_SCALED_IMU3();
            PH.setPack(p129);
            p129.zgyro = (short)(short) -15789;
            p129.ygyro = (short)(short)11717;
            p129.time_boot_ms = (uint)3930664330U;
            p129.xgyro = (short)(short) -6509;
            p129.xmag = (short)(short)22174;
            p129.zacc = (short)(short)29888;
            p129.xacc = (short)(short)2223;
            p129.yacc = (short)(short)12025;
            p129.ymag = (short)(short)22034;
            p129.zmag = (short)(short)19548;
            CommunicationChannel.instance.send(p129);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnDATA_TRANSMISSION_HANDSHAKEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.jpg_quality == (byte)(byte)230);
                Debug.Assert(pack.payload == (byte)(byte)182);
                Debug.Assert(pack.height == (ushort)(ushort)29905);
                Debug.Assert(pack.width == (ushort)(ushort)43764);
                Debug.Assert(pack.type == (byte)(byte)81);
                Debug.Assert(pack.packets == (ushort)(ushort)24588);
                Debug.Assert(pack.size == (uint)302865683U);
            };
            GroundControl.DATA_TRANSMISSION_HANDSHAKE p130 = CommunicationChannel.new_DATA_TRANSMISSION_HANDSHAKE();
            PH.setPack(p130);
            p130.width = (ushort)(ushort)43764;
            p130.jpg_quality = (byte)(byte)230;
            p130.height = (ushort)(ushort)29905;
            p130.payload = (byte)(byte)182;
            p130.type = (byte)(byte)81;
            p130.packets = (ushort)(ushort)24588;
            p130.size = (uint)302865683U;
            CommunicationChannel.instance.send(p130);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnENCAPSULATED_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)118, (byte)102, (byte)15, (byte)140, (byte)18, (byte)214, (byte)101, (byte)168, (byte)198, (byte)106, (byte)179, (byte)17, (byte)179, (byte)169, (byte)34, (byte)94, (byte)21, (byte)135, (byte)224, (byte)167, (byte)42, (byte)70, (byte)190, (byte)168, (byte)239, (byte)244, (byte)119, (byte)93, (byte)175, (byte)142, (byte)62, (byte)120, (byte)75, (byte)71, (byte)60, (byte)21, (byte)24, (byte)68, (byte)48, (byte)86, (byte)193, (byte)190, (byte)234, (byte)127, (byte)88, (byte)165, (byte)221, (byte)166, (byte)149, (byte)110, (byte)31, (byte)212, (byte)170, (byte)85, (byte)43, (byte)23, (byte)228, (byte)176, (byte)166, (byte)224, (byte)122, (byte)246, (byte)35, (byte)194, (byte)161, (byte)33, (byte)37, (byte)133, (byte)31, (byte)186, (byte)218, (byte)208, (byte)185, (byte)131, (byte)17, (byte)151, (byte)188, (byte)165, (byte)121, (byte)35, (byte)23, (byte)232, (byte)35, (byte)21, (byte)25, (byte)232, (byte)229, (byte)189, (byte)85, (byte)52, (byte)140, (byte)65, (byte)99, (byte)29, (byte)43, (byte)213, (byte)118, (byte)6, (byte)149, (byte)236, (byte)29, (byte)82, (byte)155, (byte)52, (byte)10, (byte)213, (byte)84, (byte)69, (byte)191, (byte)120, (byte)182, (byte)19, (byte)219, (byte)224, (byte)150, (byte)210, (byte)139, (byte)100, (byte)146, (byte)27, (byte)219, (byte)202, (byte)183, (byte)207, (byte)111, (byte)20, (byte)3, (byte)114, (byte)91, (byte)50, (byte)15, (byte)131, (byte)153, (byte)195, (byte)246, (byte)225, (byte)249, (byte)41, (byte)180, (byte)7, (byte)254, (byte)98, (byte)191, (byte)83, (byte)104, (byte)168, (byte)244, (byte)1, (byte)69, (byte)195, (byte)40, (byte)39, (byte)96, (byte)160, (byte)98, (byte)175, (byte)173, (byte)113, (byte)138, (byte)241, (byte)52, (byte)191, (byte)189, (byte)204, (byte)255, (byte)113, (byte)92, (byte)159, (byte)170, (byte)169, (byte)194, (byte)193, (byte)228, (byte)236, (byte)239, (byte)243, (byte)54, (byte)151, (byte)14, (byte)110, (byte)178, (byte)111, (byte)193, (byte)67, (byte)88, (byte)175, (byte)71, (byte)198, (byte)160, (byte)145, (byte)232, (byte)188, (byte)42, (byte)244, (byte)136, (byte)66, (byte)41, (byte)249, (byte)66, (byte)43, (byte)191, (byte)213, (byte)122, (byte)202, (byte)1, (byte)106, (byte)145, (byte)100, (byte)110, (byte)201, (byte)17, (byte)3, (byte)208, (byte)42, (byte)109, (byte)81, (byte)195, (byte)162, (byte)226, (byte)114, (byte)135, (byte)103, (byte)217, (byte)164, (byte)235, (byte)102, (byte)243, (byte)107, (byte)204, (byte)131, (byte)221, (byte)71, (byte)3, (byte)40, (byte)150, (byte)88, (byte)10, (byte)94, (byte)132, (byte)253, (byte)17, (byte)182, (byte)169, (byte)142, (byte)255, (byte)233, (byte)239, (byte)214, (byte)255, (byte)192, (byte)203, (byte)89, (byte)181}));
                Debug.Assert(pack.seqnr == (ushort)(ushort)62929);
            };
            GroundControl.ENCAPSULATED_DATA p131 = CommunicationChannel.new_ENCAPSULATED_DATA();
            PH.setPack(p131);
            p131.seqnr = (ushort)(ushort)62929;
            p131.data__SET(new byte[] {(byte)118, (byte)102, (byte)15, (byte)140, (byte)18, (byte)214, (byte)101, (byte)168, (byte)198, (byte)106, (byte)179, (byte)17, (byte)179, (byte)169, (byte)34, (byte)94, (byte)21, (byte)135, (byte)224, (byte)167, (byte)42, (byte)70, (byte)190, (byte)168, (byte)239, (byte)244, (byte)119, (byte)93, (byte)175, (byte)142, (byte)62, (byte)120, (byte)75, (byte)71, (byte)60, (byte)21, (byte)24, (byte)68, (byte)48, (byte)86, (byte)193, (byte)190, (byte)234, (byte)127, (byte)88, (byte)165, (byte)221, (byte)166, (byte)149, (byte)110, (byte)31, (byte)212, (byte)170, (byte)85, (byte)43, (byte)23, (byte)228, (byte)176, (byte)166, (byte)224, (byte)122, (byte)246, (byte)35, (byte)194, (byte)161, (byte)33, (byte)37, (byte)133, (byte)31, (byte)186, (byte)218, (byte)208, (byte)185, (byte)131, (byte)17, (byte)151, (byte)188, (byte)165, (byte)121, (byte)35, (byte)23, (byte)232, (byte)35, (byte)21, (byte)25, (byte)232, (byte)229, (byte)189, (byte)85, (byte)52, (byte)140, (byte)65, (byte)99, (byte)29, (byte)43, (byte)213, (byte)118, (byte)6, (byte)149, (byte)236, (byte)29, (byte)82, (byte)155, (byte)52, (byte)10, (byte)213, (byte)84, (byte)69, (byte)191, (byte)120, (byte)182, (byte)19, (byte)219, (byte)224, (byte)150, (byte)210, (byte)139, (byte)100, (byte)146, (byte)27, (byte)219, (byte)202, (byte)183, (byte)207, (byte)111, (byte)20, (byte)3, (byte)114, (byte)91, (byte)50, (byte)15, (byte)131, (byte)153, (byte)195, (byte)246, (byte)225, (byte)249, (byte)41, (byte)180, (byte)7, (byte)254, (byte)98, (byte)191, (byte)83, (byte)104, (byte)168, (byte)244, (byte)1, (byte)69, (byte)195, (byte)40, (byte)39, (byte)96, (byte)160, (byte)98, (byte)175, (byte)173, (byte)113, (byte)138, (byte)241, (byte)52, (byte)191, (byte)189, (byte)204, (byte)255, (byte)113, (byte)92, (byte)159, (byte)170, (byte)169, (byte)194, (byte)193, (byte)228, (byte)236, (byte)239, (byte)243, (byte)54, (byte)151, (byte)14, (byte)110, (byte)178, (byte)111, (byte)193, (byte)67, (byte)88, (byte)175, (byte)71, (byte)198, (byte)160, (byte)145, (byte)232, (byte)188, (byte)42, (byte)244, (byte)136, (byte)66, (byte)41, (byte)249, (byte)66, (byte)43, (byte)191, (byte)213, (byte)122, (byte)202, (byte)1, (byte)106, (byte)145, (byte)100, (byte)110, (byte)201, (byte)17, (byte)3, (byte)208, (byte)42, (byte)109, (byte)81, (byte)195, (byte)162, (byte)226, (byte)114, (byte)135, (byte)103, (byte)217, (byte)164, (byte)235, (byte)102, (byte)243, (byte)107, (byte)204, (byte)131, (byte)221, (byte)71, (byte)3, (byte)40, (byte)150, (byte)88, (byte)10, (byte)94, (byte)132, (byte)253, (byte)17, (byte)182, (byte)169, (byte)142, (byte)255, (byte)233, (byte)239, (byte)214, (byte)255, (byte)192, (byte)203, (byte)89, (byte)181}, 0) ;
            CommunicationChannel.instance.send(p131);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnDISTANCE_SENSORReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)4110086127U);
                Debug.Assert(pack.covariance == (byte)(byte)111);
                Debug.Assert(pack.id == (byte)(byte)2);
                Debug.Assert(pack.min_distance == (ushort)(ushort)40011);
                Debug.Assert(pack.current_distance == (ushort)(ushort)22301);
                Debug.Assert(pack.max_distance == (ushort)(ushort)38271);
                Debug.Assert(pack.orientation == MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_YAW_45);
                Debug.Assert(pack.type == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_RADAR);
            };
            GroundControl.DISTANCE_SENSOR p132 = CommunicationChannel.new_DISTANCE_SENSOR();
            PH.setPack(p132);
            p132.type = MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_RADAR;
            p132.id = (byte)(byte)2;
            p132.time_boot_ms = (uint)4110086127U;
            p132.current_distance = (ushort)(ushort)22301;
            p132.orientation = MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_YAW_45;
            p132.max_distance = (ushort)(ushort)38271;
            p132.min_distance = (ushort)(ushort)40011;
            p132.covariance = (byte)(byte)111;
            CommunicationChannel.instance.send(p132);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTERRAIN_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mask == (ulong)6625902203068172429L);
                Debug.Assert(pack.grid_spacing == (ushort)(ushort)18658);
                Debug.Assert(pack.lon == (int) -1034501436);
                Debug.Assert(pack.lat == (int) -1125144091);
            };
            GroundControl.TERRAIN_REQUEST p133 = CommunicationChannel.new_TERRAIN_REQUEST();
            PH.setPack(p133);
            p133.mask = (ulong)6625902203068172429L;
            p133.grid_spacing = (ushort)(ushort)18658;
            p133.lon = (int) -1034501436;
            p133.lat = (int) -1125144091;
            CommunicationChannel.instance.send(p133);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTERRAIN_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.gridbit == (byte)(byte)9);
                Debug.Assert(pack.grid_spacing == (ushort)(ushort)2656);
                Debug.Assert(pack.data_.SequenceEqual(new short[] {(short) -2048, (short)18972, (short)1574, (short) -9998, (short)3212, (short) -7326, (short) -23350, (short)20712, (short)12759, (short) -790, (short)23807, (short) -2223, (short)26677, (short)22585, (short) -24771, (short) -12111}));
                Debug.Assert(pack.lat == (int)1680974435);
                Debug.Assert(pack.lon == (int)853849857);
            };
            GroundControl.TERRAIN_DATA p134 = CommunicationChannel.new_TERRAIN_DATA();
            PH.setPack(p134);
            p134.grid_spacing = (ushort)(ushort)2656;
            p134.lon = (int)853849857;
            p134.gridbit = (byte)(byte)9;
            p134.data__SET(new short[] {(short) -2048, (short)18972, (short)1574, (short) -9998, (short)3212, (short) -7326, (short) -23350, (short)20712, (short)12759, (short) -790, (short)23807, (short) -2223, (short)26677, (short)22585, (short) -24771, (short) -12111}, 0) ;
            p134.lat = (int)1680974435;
            CommunicationChannel.instance.send(p134);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTERRAIN_CHECKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lat == (int) -1337477586);
                Debug.Assert(pack.lon == (int)1058200218);
            };
            GroundControl.TERRAIN_CHECK p135 = CommunicationChannel.new_TERRAIN_CHECK();
            PH.setPack(p135);
            p135.lat = (int) -1337477586;
            p135.lon = (int)1058200218;
            CommunicationChannel.instance.send(p135);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTERRAIN_REPORTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lat == (int) -1645217138);
                Debug.Assert(pack.current_height == (float)1.7590814E38F);
                Debug.Assert(pack.spacing == (ushort)(ushort)6718);
                Debug.Assert(pack.terrain_height == (float) -6.119747E37F);
                Debug.Assert(pack.lon == (int)1562954218);
                Debug.Assert(pack.loaded == (ushort)(ushort)55073);
                Debug.Assert(pack.pending == (ushort)(ushort)54096);
            };
            GroundControl.TERRAIN_REPORT p136 = CommunicationChannel.new_TERRAIN_REPORT();
            PH.setPack(p136);
            p136.current_height = (float)1.7590814E38F;
            p136.lat = (int) -1645217138;
            p136.lon = (int)1562954218;
            p136.pending = (ushort)(ushort)54096;
            p136.spacing = (ushort)(ushort)6718;
            p136.loaded = (ushort)(ushort)55073;
            p136.terrain_height = (float) -6.119747E37F;
            CommunicationChannel.instance.send(p136);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_PRESSURE2Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.press_diff == (float)1.5939047E38F);
                Debug.Assert(pack.time_boot_ms == (uint)84020900U);
                Debug.Assert(pack.press_abs == (float)5.4114853E37F);
                Debug.Assert(pack.temperature == (short)(short)16143);
            };
            GroundControl.SCALED_PRESSURE2 p137 = CommunicationChannel.new_SCALED_PRESSURE2();
            PH.setPack(p137);
            p137.time_boot_ms = (uint)84020900U;
            p137.temperature = (short)(short)16143;
            p137.press_abs = (float)5.4114853E37F;
            p137.press_diff = (float)1.5939047E38F;
            CommunicationChannel.instance.send(p137);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATT_POS_MOCAPReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)3377357476141483634L);
                Debug.Assert(pack.y == (float) -1.3623819E37F);
                Debug.Assert(pack.x == (float) -2.6233508E38F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {9.356855E37F, -1.1114745E38F, 1.4880621E38F, 7.191345E37F}));
                Debug.Assert(pack.z == (float)2.3116039E38F);
            };
            GroundControl.ATT_POS_MOCAP p138 = CommunicationChannel.new_ATT_POS_MOCAP();
            PH.setPack(p138);
            p138.y = (float) -1.3623819E37F;
            p138.q_SET(new float[] {9.356855E37F, -1.1114745E38F, 1.4880621E38F, 7.191345E37F}, 0) ;
            p138.z = (float)2.3116039E38F;
            p138.time_usec = (ulong)3377357476141483634L;
            p138.x = (float) -2.6233508E38F;
            CommunicationChannel.instance.send(p138);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_ACTUATOR_CONTROL_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)16);
                Debug.Assert(pack.controls.SequenceEqual(new float[] {-1.5272206E38F, -1.1889816E38F, -7.745735E37F, -2.2417245E37F, 1.3818437E38F, 1.8031441E38F, -2.1477429E38F, 1.73613E38F}));
                Debug.Assert(pack.time_usec == (ulong)7410365429622564037L);
                Debug.Assert(pack.group_mlx == (byte)(byte)54);
                Debug.Assert(pack.target_system == (byte)(byte)148);
            };
            GroundControl.SET_ACTUATOR_CONTROL_TARGET p139 = CommunicationChannel.new_SET_ACTUATOR_CONTROL_TARGET();
            PH.setPack(p139);
            p139.group_mlx = (byte)(byte)54;
            p139.target_component = (byte)(byte)16;
            p139.time_usec = (ulong)7410365429622564037L;
            p139.controls_SET(new float[] {-1.5272206E38F, -1.1889816E38F, -7.745735E37F, -2.2417245E37F, 1.3818437E38F, 1.8031441E38F, -2.1477429E38F, 1.73613E38F}, 0) ;
            p139.target_system = (byte)(byte)148;
            CommunicationChannel.instance.send(p139);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnACTUATOR_CONTROL_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.group_mlx == (byte)(byte)11);
                Debug.Assert(pack.controls.SequenceEqual(new float[] {1.0342925E38F, 1.2436221E38F, -1.2794665E37F, 2.0982852E38F, -2.5070287E38F, 6.400031E37F, 2.562361E38F, 1.4428411E38F}));
                Debug.Assert(pack.time_usec == (ulong)4312828625947120449L);
            };
            GroundControl.ACTUATOR_CONTROL_TARGET p140 = CommunicationChannel.new_ACTUATOR_CONTROL_TARGET();
            PH.setPack(p140);
            p140.time_usec = (ulong)4312828625947120449L;
            p140.group_mlx = (byte)(byte)11;
            p140.controls_SET(new float[] {1.0342925E38F, 1.2436221E38F, -1.2794665E37F, 2.0982852E38F, -2.5070287E38F, 6.400031E37F, 2.562361E38F, 1.4428411E38F}, 0) ;
            CommunicationChannel.instance.send(p140);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnALTITUDEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.altitude_amsl == (float)3.162992E38F);
                Debug.Assert(pack.altitude_monotonic == (float) -2.344668E38F);
                Debug.Assert(pack.time_usec == (ulong)7839961837219732492L);
                Debug.Assert(pack.altitude_local == (float)3.063199E38F);
                Debug.Assert(pack.bottom_clearance == (float)2.9026215E38F);
                Debug.Assert(pack.altitude_relative == (float) -1.5310561E38F);
                Debug.Assert(pack.altitude_terrain == (float) -3.778846E37F);
            };
            GroundControl.ALTITUDE p141 = CommunicationChannel.new_ALTITUDE();
            PH.setPack(p141);
            p141.time_usec = (ulong)7839961837219732492L;
            p141.bottom_clearance = (float)2.9026215E38F;
            p141.altitude_relative = (float) -1.5310561E38F;
            p141.altitude_amsl = (float)3.162992E38F;
            p141.altitude_terrain = (float) -3.778846E37F;
            p141.altitude_monotonic = (float) -2.344668E38F;
            p141.altitude_local = (float)3.063199E38F;
            CommunicationChannel.instance.send(p141);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRESOURCE_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.transfer_type == (byte)(byte)171);
                Debug.Assert(pack.storage.SequenceEqual(new byte[] {(byte)31, (byte)27, (byte)104, (byte)123, (byte)110, (byte)132, (byte)230, (byte)136, (byte)253, (byte)18, (byte)21, (byte)135, (byte)139, (byte)147, (byte)120, (byte)167, (byte)83, (byte)243, (byte)0, (byte)86, (byte)83, (byte)233, (byte)106, (byte)190, (byte)87, (byte)144, (byte)139, (byte)117, (byte)109, (byte)130, (byte)160, (byte)134, (byte)4, (byte)195, (byte)239, (byte)116, (byte)62, (byte)100, (byte)64, (byte)209, (byte)116, (byte)233, (byte)169, (byte)220, (byte)102, (byte)53, (byte)157, (byte)57, (byte)194, (byte)78, (byte)162, (byte)168, (byte)217, (byte)247, (byte)89, (byte)46, (byte)95, (byte)16, (byte)22, (byte)89, (byte)83, (byte)16, (byte)26, (byte)104, (byte)14, (byte)171, (byte)95, (byte)188, (byte)105, (byte)172, (byte)126, (byte)141, (byte)233, (byte)168, (byte)17, (byte)242, (byte)179, (byte)117, (byte)212, (byte)82, (byte)236, (byte)37, (byte)44, (byte)92, (byte)157, (byte)184, (byte)73, (byte)250, (byte)43, (byte)171, (byte)171, (byte)68, (byte)211, (byte)247, (byte)96, (byte)46, (byte)94, (byte)200, (byte)146, (byte)30, (byte)0, (byte)138, (byte)111, (byte)97, (byte)141, (byte)33, (byte)35, (byte)156, (byte)47, (byte)24, (byte)248, (byte)101, (byte)112, (byte)104, (byte)1, (byte)117, (byte)62, (byte)66, (byte)43, (byte)82}));
                Debug.Assert(pack.uri.SequenceEqual(new byte[] {(byte)103, (byte)85, (byte)114, (byte)25, (byte)80, (byte)178, (byte)145, (byte)127, (byte)145, (byte)197, (byte)185, (byte)70, (byte)59, (byte)137, (byte)189, (byte)134, (byte)141, (byte)172, (byte)84, (byte)252, (byte)148, (byte)248, (byte)60, (byte)56, (byte)0, (byte)209, (byte)64, (byte)193, (byte)10, (byte)251, (byte)160, (byte)166, (byte)108, (byte)232, (byte)3, (byte)85, (byte)249, (byte)239, (byte)38, (byte)104, (byte)209, (byte)88, (byte)209, (byte)231, (byte)102, (byte)252, (byte)196, (byte)106, (byte)180, (byte)159, (byte)107, (byte)20, (byte)31, (byte)189, (byte)167, (byte)232, (byte)96, (byte)173, (byte)85, (byte)72, (byte)212, (byte)125, (byte)76, (byte)133, (byte)191, (byte)81, (byte)97, (byte)213, (byte)122, (byte)246, (byte)1, (byte)44, (byte)170, (byte)129, (byte)214, (byte)218, (byte)211, (byte)3, (byte)175, (byte)115, (byte)60, (byte)215, (byte)39, (byte)14, (byte)98, (byte)160, (byte)4, (byte)159, (byte)29, (byte)115, (byte)100, (byte)41, (byte)100, (byte)197, (byte)6, (byte)0, (byte)102, (byte)83, (byte)189, (byte)251, (byte)158, (byte)50, (byte)34, (byte)61, (byte)37, (byte)29, (byte)34, (byte)113, (byte)237, (byte)236, (byte)90, (byte)4, (byte)18, (byte)173, (byte)125, (byte)242, (byte)145, (byte)155, (byte)79, (byte)121}));
                Debug.Assert(pack.request_id == (byte)(byte)217);
                Debug.Assert(pack.uri_type == (byte)(byte)121);
            };
            GroundControl.RESOURCE_REQUEST p142 = CommunicationChannel.new_RESOURCE_REQUEST();
            PH.setPack(p142);
            p142.uri_SET(new byte[] {(byte)103, (byte)85, (byte)114, (byte)25, (byte)80, (byte)178, (byte)145, (byte)127, (byte)145, (byte)197, (byte)185, (byte)70, (byte)59, (byte)137, (byte)189, (byte)134, (byte)141, (byte)172, (byte)84, (byte)252, (byte)148, (byte)248, (byte)60, (byte)56, (byte)0, (byte)209, (byte)64, (byte)193, (byte)10, (byte)251, (byte)160, (byte)166, (byte)108, (byte)232, (byte)3, (byte)85, (byte)249, (byte)239, (byte)38, (byte)104, (byte)209, (byte)88, (byte)209, (byte)231, (byte)102, (byte)252, (byte)196, (byte)106, (byte)180, (byte)159, (byte)107, (byte)20, (byte)31, (byte)189, (byte)167, (byte)232, (byte)96, (byte)173, (byte)85, (byte)72, (byte)212, (byte)125, (byte)76, (byte)133, (byte)191, (byte)81, (byte)97, (byte)213, (byte)122, (byte)246, (byte)1, (byte)44, (byte)170, (byte)129, (byte)214, (byte)218, (byte)211, (byte)3, (byte)175, (byte)115, (byte)60, (byte)215, (byte)39, (byte)14, (byte)98, (byte)160, (byte)4, (byte)159, (byte)29, (byte)115, (byte)100, (byte)41, (byte)100, (byte)197, (byte)6, (byte)0, (byte)102, (byte)83, (byte)189, (byte)251, (byte)158, (byte)50, (byte)34, (byte)61, (byte)37, (byte)29, (byte)34, (byte)113, (byte)237, (byte)236, (byte)90, (byte)4, (byte)18, (byte)173, (byte)125, (byte)242, (byte)145, (byte)155, (byte)79, (byte)121}, 0) ;
            p142.transfer_type = (byte)(byte)171;
            p142.request_id = (byte)(byte)217;
            p142.uri_type = (byte)(byte)121;
            p142.storage_SET(new byte[] {(byte)31, (byte)27, (byte)104, (byte)123, (byte)110, (byte)132, (byte)230, (byte)136, (byte)253, (byte)18, (byte)21, (byte)135, (byte)139, (byte)147, (byte)120, (byte)167, (byte)83, (byte)243, (byte)0, (byte)86, (byte)83, (byte)233, (byte)106, (byte)190, (byte)87, (byte)144, (byte)139, (byte)117, (byte)109, (byte)130, (byte)160, (byte)134, (byte)4, (byte)195, (byte)239, (byte)116, (byte)62, (byte)100, (byte)64, (byte)209, (byte)116, (byte)233, (byte)169, (byte)220, (byte)102, (byte)53, (byte)157, (byte)57, (byte)194, (byte)78, (byte)162, (byte)168, (byte)217, (byte)247, (byte)89, (byte)46, (byte)95, (byte)16, (byte)22, (byte)89, (byte)83, (byte)16, (byte)26, (byte)104, (byte)14, (byte)171, (byte)95, (byte)188, (byte)105, (byte)172, (byte)126, (byte)141, (byte)233, (byte)168, (byte)17, (byte)242, (byte)179, (byte)117, (byte)212, (byte)82, (byte)236, (byte)37, (byte)44, (byte)92, (byte)157, (byte)184, (byte)73, (byte)250, (byte)43, (byte)171, (byte)171, (byte)68, (byte)211, (byte)247, (byte)96, (byte)46, (byte)94, (byte)200, (byte)146, (byte)30, (byte)0, (byte)138, (byte)111, (byte)97, (byte)141, (byte)33, (byte)35, (byte)156, (byte)47, (byte)24, (byte)248, (byte)101, (byte)112, (byte)104, (byte)1, (byte)117, (byte)62, (byte)66, (byte)43, (byte)82}, 0) ;
            CommunicationChannel.instance.send(p142);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_PRESSURE3Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.temperature == (short)(short) -23812);
                Debug.Assert(pack.press_diff == (float) -1.1392056E37F);
                Debug.Assert(pack.press_abs == (float)1.7344904E38F);
                Debug.Assert(pack.time_boot_ms == (uint)2502646439U);
            };
            GroundControl.SCALED_PRESSURE3 p143 = CommunicationChannel.new_SCALED_PRESSURE3();
            PH.setPack(p143);
            p143.press_abs = (float)1.7344904E38F;
            p143.time_boot_ms = (uint)2502646439U;
            p143.temperature = (short)(short) -23812;
            p143.press_diff = (float) -1.1392056E37F;
            CommunicationChannel.instance.send(p143);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnFOLLOW_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.est_capabilities == (byte)(byte)121);
                Debug.Assert(pack.lon == (int)237662520);
                Debug.Assert(pack.vel.SequenceEqual(new float[] {7.2664247E37F, 2.8196476E38F, 3.1131038E38F}));
                Debug.Assert(pack.alt == (float)5.846025E37F);
                Debug.Assert(pack.acc.SequenceEqual(new float[] {2.7903764E38F, 2.302405E38F, -1.1716792E38F}));
                Debug.Assert(pack.position_cov.SequenceEqual(new float[] {-1.1209547E38F, 2.2103658E38F, 9.119476E37F}));
                Debug.Assert(pack.rates.SequenceEqual(new float[] {1.0673195E38F, -1.5805551E38F, 2.9379277E38F}));
                Debug.Assert(pack.custom_state == (ulong)3057779116218456888L);
                Debug.Assert(pack.lat == (int)1663735948);
                Debug.Assert(pack.attitude_q.SequenceEqual(new float[] {-1.7025381E38F, -4.1057783E37F, 2.7233755E38F, 9.833517E37F}));
                Debug.Assert(pack.timestamp == (ulong)7196771839631258192L);
            };
            GroundControl.FOLLOW_TARGET p144 = CommunicationChannel.new_FOLLOW_TARGET();
            PH.setPack(p144);
            p144.alt = (float)5.846025E37F;
            p144.custom_state = (ulong)3057779116218456888L;
            p144.rates_SET(new float[] {1.0673195E38F, -1.5805551E38F, 2.9379277E38F}, 0) ;
            p144.position_cov_SET(new float[] {-1.1209547E38F, 2.2103658E38F, 9.119476E37F}, 0) ;
            p144.acc_SET(new float[] {2.7903764E38F, 2.302405E38F, -1.1716792E38F}, 0) ;
            p144.vel_SET(new float[] {7.2664247E37F, 2.8196476E38F, 3.1131038E38F}, 0) ;
            p144.attitude_q_SET(new float[] {-1.7025381E38F, -4.1057783E37F, 2.7233755E38F, 9.833517E37F}, 0) ;
            p144.lon = (int)237662520;
            p144.est_capabilities = (byte)(byte)121;
            p144.lat = (int)1663735948;
            p144.timestamp = (ulong)7196771839631258192L;
            CommunicationChannel.instance.send(p144);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCONTROL_SYSTEM_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.x_vel == (float)1.3143001E38F);
                Debug.Assert(pack.time_usec == (ulong)2018205595192000738L);
                Debug.Assert(pack.x_pos == (float)2.3759216E38F);
                Debug.Assert(pack.y_pos == (float)1.0972523E38F);
                Debug.Assert(pack.y_acc == (float)3.364404E38F);
                Debug.Assert(pack.yaw_rate == (float)2.081258E38F);
                Debug.Assert(pack.vel_variance.SequenceEqual(new float[] {-3.1533644E38F, 2.7260528E38F, 1.546385E38F}));
                Debug.Assert(pack.roll_rate == (float) -2.2306527E38F);
                Debug.Assert(pack.z_pos == (float)1.0487418E38F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {2.0999466E38F, 2.2596553E38F, -3.701008E37F, 2.8128761E38F}));
                Debug.Assert(pack.z_vel == (float) -1.203864E38F);
                Debug.Assert(pack.pos_variance.SequenceEqual(new float[] {1.46177E38F, -1.3088272E38F, -1.6378288E37F}));
                Debug.Assert(pack.y_vel == (float) -2.629172E38F);
                Debug.Assert(pack.x_acc == (float) -6.3219764E37F);
                Debug.Assert(pack.pitch_rate == (float)1.599116E38F);
                Debug.Assert(pack.z_acc == (float)1.2986647E38F);
                Debug.Assert(pack.airspeed == (float)3.0370016E38F);
            };
            GroundControl.CONTROL_SYSTEM_STATE p146 = CommunicationChannel.new_CONTROL_SYSTEM_STATE();
            PH.setPack(p146);
            p146.x_vel = (float)1.3143001E38F;
            p146.y_pos = (float)1.0972523E38F;
            p146.pos_variance_SET(new float[] {1.46177E38F, -1.3088272E38F, -1.6378288E37F}, 0) ;
            p146.airspeed = (float)3.0370016E38F;
            p146.q_SET(new float[] {2.0999466E38F, 2.2596553E38F, -3.701008E37F, 2.8128761E38F}, 0) ;
            p146.z_vel = (float) -1.203864E38F;
            p146.x_acc = (float) -6.3219764E37F;
            p146.z_acc = (float)1.2986647E38F;
            p146.yaw_rate = (float)2.081258E38F;
            p146.vel_variance_SET(new float[] {-3.1533644E38F, 2.7260528E38F, 1.546385E38F}, 0) ;
            p146.y_vel = (float) -2.629172E38F;
            p146.pitch_rate = (float)1.599116E38F;
            p146.z_pos = (float)1.0487418E38F;
            p146.time_usec = (ulong)2018205595192000738L;
            p146.roll_rate = (float) -2.2306527E38F;
            p146.x_pos = (float)2.3759216E38F;
            p146.y_acc = (float)3.364404E38F;
            CommunicationChannel.instance.send(p146);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnBATTERY_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.id == (byte)(byte)246);
                Debug.Assert(pack.battery_function == MAV_BATTERY_FUNCTION.MAV_BATTERY_TYPE_PAYLOAD);
                Debug.Assert(pack.current_consumed == (int) -1114358217);
                Debug.Assert(pack.voltages.SequenceEqual(new ushort[] {(ushort)46510, (ushort)59452, (ushort)29090, (ushort)39624, (ushort)56473, (ushort)7237, (ushort)49309, (ushort)11168, (ushort)38996, (ushort)6562}));
                Debug.Assert(pack.energy_consumed == (int)69220541);
                Debug.Assert(pack.type == MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_LIPO);
                Debug.Assert(pack.battery_remaining == (sbyte)(sbyte) - 20);
                Debug.Assert(pack.temperature == (short)(short)10323);
                Debug.Assert(pack.current_battery == (short)(short)19238);
            };
            GroundControl.BATTERY_STATUS p147 = CommunicationChannel.new_BATTERY_STATUS();
            PH.setPack(p147);
            p147.voltages_SET(new ushort[] {(ushort)46510, (ushort)59452, (ushort)29090, (ushort)39624, (ushort)56473, (ushort)7237, (ushort)49309, (ushort)11168, (ushort)38996, (ushort)6562}, 0) ;
            p147.type = MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_LIPO;
            p147.temperature = (short)(short)10323;
            p147.id = (byte)(byte)246;
            p147.current_battery = (short)(short)19238;
            p147.energy_consumed = (int)69220541;
            p147.current_consumed = (int) -1114358217;
            p147.battery_remaining = (sbyte)(sbyte) - 20;
            p147.battery_function = MAV_BATTERY_FUNCTION.MAV_BATTERY_TYPE_PAYLOAD;
            CommunicationChannel.instance.send(p147);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnAUTOPILOT_VERSIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.middleware_custom_version.SequenceEqual(new byte[] {(byte)239, (byte)198, (byte)37, (byte)0, (byte)52, (byte)194, (byte)23, (byte)3}));
                Debug.Assert(pack.os_sw_version == (uint)1559422395U);
                Debug.Assert(pack.capabilities == (MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET |
                                                   MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_PARAM_UNION |
                                                   MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT |
                                                   MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_ACTUATOR_TARGET |
                                                   MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT |
                                                   MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_TERRAIN |
                                                   MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_INT |
                                                   MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_FENCE));
                Debug.Assert(pack.os_custom_version.SequenceEqual(new byte[] {(byte)215, (byte)181, (byte)153, (byte)168, (byte)38, (byte)162, (byte)44, (byte)255}));
                Debug.Assert(pack.middleware_sw_version == (uint)3234889464U);
                Debug.Assert(pack.product_id == (ushort)(ushort)27604);
                Debug.Assert(pack.vendor_id == (ushort)(ushort)55171);
                Debug.Assert(pack.flight_sw_version == (uint)3217075799U);
                Debug.Assert(pack.board_version == (uint)1815705547U);
                Debug.Assert(pack.uid2_TRY(ph).SequenceEqual(new byte[] {(byte)24, (byte)30, (byte)241, (byte)229, (byte)111, (byte)99, (byte)120, (byte)218, (byte)38, (byte)104, (byte)232, (byte)66, (byte)254, (byte)20, (byte)249, (byte)93, (byte)73, (byte)46}));
                Debug.Assert(pack.uid == (ulong)1637854985050365949L);
                Debug.Assert(pack.flight_custom_version.SequenceEqual(new byte[] {(byte)186, (byte)193, (byte)31, (byte)40, (byte)201, (byte)63, (byte)191, (byte)38}));
            };
            GroundControl.AUTOPILOT_VERSION p148 = CommunicationChannel.new_AUTOPILOT_VERSION();
            PH.setPack(p148);
            p148.os_sw_version = (uint)1559422395U;
            p148.product_id = (ushort)(ushort)27604;
            p148.capabilities = (MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_PARAM_UNION |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_ACTUATOR_TARGET |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_TERRAIN |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_INT |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_FENCE);
            p148.flight_sw_version = (uint)3217075799U;
            p148.uid2_SET(new byte[] {(byte)24, (byte)30, (byte)241, (byte)229, (byte)111, (byte)99, (byte)120, (byte)218, (byte)38, (byte)104, (byte)232, (byte)66, (byte)254, (byte)20, (byte)249, (byte)93, (byte)73, (byte)46}, 0, PH) ;
            p148.flight_custom_version_SET(new byte[] {(byte)186, (byte)193, (byte)31, (byte)40, (byte)201, (byte)63, (byte)191, (byte)38}, 0) ;
            p148.middleware_sw_version = (uint)3234889464U;
            p148.uid = (ulong)1637854985050365949L;
            p148.os_custom_version_SET(new byte[] {(byte)215, (byte)181, (byte)153, (byte)168, (byte)38, (byte)162, (byte)44, (byte)255}, 0) ;
            p148.vendor_id = (ushort)(ushort)55171;
            p148.middleware_custom_version_SET(new byte[] {(byte)239, (byte)198, (byte)37, (byte)0, (byte)52, (byte)194, (byte)23, (byte)3}, 0) ;
            p148.board_version = (uint)1815705547U;
            CommunicationChannel.instance.send(p148);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLANDING_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.angle_x == (float)2.515686E38F);
                Debug.Assert(pack.size_x == (float)7.7546497E37F);
                Debug.Assert(pack.z_TRY(ph) == (float)1.4986291E38F);
                Debug.Assert(pack.x_TRY(ph) == (float) -2.1193564E38F);
                Debug.Assert(pack.angle_y == (float) -2.5742205E38F);
                Debug.Assert(pack.frame == MAV_FRAME.MAV_FRAME_GLOBAL);
                Debug.Assert(pack.position_valid_TRY(ph) == (byte)(byte)204);
                Debug.Assert(pack.type == LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_LIGHT_BEACON);
                Debug.Assert(pack.distance == (float) -2.4356754E38F);
                Debug.Assert(pack.q_TRY(ph).SequenceEqual(new float[] {9.534934E37F, -1.380752E37F, -3.2135857E38F, 3.0432208E38F}));
                Debug.Assert(pack.y_TRY(ph) == (float) -1.4655328E38F);
                Debug.Assert(pack.time_usec == (ulong)6887161780258833203L);
                Debug.Assert(pack.size_y == (float) -2.6236579E38F);
                Debug.Assert(pack.target_num == (byte)(byte)36);
            };
            GroundControl.LANDING_TARGET p149 = CommunicationChannel.new_LANDING_TARGET();
            PH.setPack(p149);
            p149.size_y = (float) -2.6236579E38F;
            p149.frame = MAV_FRAME.MAV_FRAME_GLOBAL;
            p149.time_usec = (ulong)6887161780258833203L;
            p149.angle_x = (float)2.515686E38F;
            p149.position_valid_SET((byte)(byte)204, PH) ;
            p149.q_SET(new float[] {9.534934E37F, -1.380752E37F, -3.2135857E38F, 3.0432208E38F}, 0, PH) ;
            p149.target_num = (byte)(byte)36;
            p149.x_SET((float) -2.1193564E38F, PH) ;
            p149.z_SET((float)1.4986291E38F, PH) ;
            p149.y_SET((float) -1.4655328E38F, PH) ;
            p149.angle_y = (float) -2.5742205E38F;
            p149.distance = (float) -2.4356754E38F;
            p149.size_x = (float)7.7546497E37F;
            p149.type = LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_LIGHT_BEACON;
            CommunicationChannel.instance.send(p149);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            ADV_TEST_CH.OnFLEXIFUNCTION_SETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)134);
                Debug.Assert(pack.target_component == (byte)(byte)117);
            };
            GroundControl.FLEXIFUNCTION_SET p150 = CommunicationChannel.new_FLEXIFUNCTION_SET();
            PH.setPack(p150);
            p150.target_component = (byte)(byte)117;
            p150.target_system = (byte)(byte)134;
            CommunicationChannel.instance.send(p150);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnFLEXIFUNCTION_READ_REQReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.data_index == (short)(short) -23937);
                Debug.Assert(pack.read_req_type == (short)(short)5256);
                Debug.Assert(pack.target_system == (byte)(byte)41);
                Debug.Assert(pack.target_component == (byte)(byte)175);
            };
            GroundControl.FLEXIFUNCTION_READ_REQ p151 = CommunicationChannel.new_FLEXIFUNCTION_READ_REQ();
            PH.setPack(p151);
            p151.data_index = (short)(short) -23937;
            p151.target_component = (byte)(byte)175;
            p151.read_req_type = (short)(short)5256;
            p151.target_system = (byte)(byte)41;
            CommunicationChannel.instance.send(p151);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnFLEXIFUNCTION_BUFFER_FUNCTIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.data_size == (ushort)(ushort)48674);
                Debug.Assert(pack.func_index == (ushort)(ushort)58627);
                Debug.Assert(pack.target_system == (byte)(byte)67);
                Debug.Assert(pack.data_.SequenceEqual(new sbyte[] {(sbyte)23, (sbyte)80, (sbyte)70, (sbyte)27, (sbyte) - 101, (sbyte) - 29, (sbyte)96, (sbyte)17, (sbyte) - 91, (sbyte) - 84, (sbyte)121, (sbyte)45, (sbyte)88, (sbyte) - 110, (sbyte) - 110, (sbyte)117, (sbyte) - 124, (sbyte)62, (sbyte)40, (sbyte) - 119, (sbyte)65, (sbyte) - 121, (sbyte)4, (sbyte) - 95, (sbyte)3, (sbyte) - 52, (sbyte) - 14, (sbyte)105, (sbyte) - 93, (sbyte)33, (sbyte) - 51, (sbyte) - 119, (sbyte) - 21, (sbyte)108, (sbyte)41, (sbyte) - 70, (sbyte)40, (sbyte) - 61, (sbyte)93, (sbyte)122, (sbyte) - 27, (sbyte) - 3, (sbyte) - 40, (sbyte) - 89, (sbyte)91, (sbyte) - 74, (sbyte)68, (sbyte)60}));
                Debug.Assert(pack.data_address == (ushort)(ushort)27045);
                Debug.Assert(pack.target_component == (byte)(byte)108);
                Debug.Assert(pack.func_count == (ushort)(ushort)58100);
            };
            GroundControl.FLEXIFUNCTION_BUFFER_FUNCTION p152 = CommunicationChannel.new_FLEXIFUNCTION_BUFFER_FUNCTION();
            PH.setPack(p152);
            p152.data_size = (ushort)(ushort)48674;
            p152.target_system = (byte)(byte)67;
            p152.data__SET(new sbyte[] {(sbyte)23, (sbyte)80, (sbyte)70, (sbyte)27, (sbyte) - 101, (sbyte) - 29, (sbyte)96, (sbyte)17, (sbyte) - 91, (sbyte) - 84, (sbyte)121, (sbyte)45, (sbyte)88, (sbyte) - 110, (sbyte) - 110, (sbyte)117, (sbyte) - 124, (sbyte)62, (sbyte)40, (sbyte) - 119, (sbyte)65, (sbyte) - 121, (sbyte)4, (sbyte) - 95, (sbyte)3, (sbyte) - 52, (sbyte) - 14, (sbyte)105, (sbyte) - 93, (sbyte)33, (sbyte) - 51, (sbyte) - 119, (sbyte) - 21, (sbyte)108, (sbyte)41, (sbyte) - 70, (sbyte)40, (sbyte) - 61, (sbyte)93, (sbyte)122, (sbyte) - 27, (sbyte) - 3, (sbyte) - 40, (sbyte) - 89, (sbyte)91, (sbyte) - 74, (sbyte)68, (sbyte)60}, 0) ;
            p152.data_address = (ushort)(ushort)27045;
            p152.func_count = (ushort)(ushort)58100;
            p152.target_component = (byte)(byte)108;
            p152.func_index = (ushort)(ushort)58627;
            CommunicationChannel.instance.send(p152);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnFLEXIFUNCTION_BUFFER_FUNCTION_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.result == (ushort)(ushort)6137);
                Debug.Assert(pack.target_system == (byte)(byte)254);
                Debug.Assert(pack.func_index == (ushort)(ushort)36222);
                Debug.Assert(pack.target_component == (byte)(byte)52);
            };
            GroundControl.FLEXIFUNCTION_BUFFER_FUNCTION_ACK p153 = CommunicationChannel.new_FLEXIFUNCTION_BUFFER_FUNCTION_ACK();
            PH.setPack(p153);
            p153.result = (ushort)(ushort)6137;
            p153.target_system = (byte)(byte)254;
            p153.target_component = (byte)(byte)52;
            p153.func_index = (ushort)(ushort)36222;
            CommunicationChannel.instance.send(p153);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnFLEXIFUNCTION_DIRECTORYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.start_index == (byte)(byte)186);
                Debug.Assert(pack.count == (byte)(byte)117);
                Debug.Assert(pack.directory_data.SequenceEqual(new sbyte[] {(sbyte)73, (sbyte)6, (sbyte)13, (sbyte) - 126, (sbyte)97, (sbyte) - 68, (sbyte) - 69, (sbyte) - 17, (sbyte)60, (sbyte)55, (sbyte)26, (sbyte) - 104, (sbyte) - 1, (sbyte) - 80, (sbyte)123, (sbyte) - 91, (sbyte)53, (sbyte)34, (sbyte) - 14, (sbyte)121, (sbyte) - 52, (sbyte) - 54, (sbyte)125, (sbyte) - 23, (sbyte) - 89, (sbyte)25, (sbyte) - 42, (sbyte) - 93, (sbyte) - 36, (sbyte) - 105, (sbyte)119, (sbyte) - 101, (sbyte) - 80, (sbyte) - 111, (sbyte)46, (sbyte) - 59, (sbyte) - 19, (sbyte) - 109, (sbyte)30, (sbyte)104, (sbyte) - 22, (sbyte) - 18, (sbyte) - 59, (sbyte) - 35, (sbyte)22, (sbyte)8, (sbyte) - 54, (sbyte) - 123}));
                Debug.Assert(pack.target_system == (byte)(byte)73);
                Debug.Assert(pack.directory_type == (byte)(byte)15);
                Debug.Assert(pack.target_component == (byte)(byte)244);
            };
            GroundControl.FLEXIFUNCTION_DIRECTORY p155 = CommunicationChannel.new_FLEXIFUNCTION_DIRECTORY();
            PH.setPack(p155);
            p155.target_system = (byte)(byte)73;
            p155.directory_data_SET(new sbyte[] {(sbyte)73, (sbyte)6, (sbyte)13, (sbyte) - 126, (sbyte)97, (sbyte) - 68, (sbyte) - 69, (sbyte) - 17, (sbyte)60, (sbyte)55, (sbyte)26, (sbyte) - 104, (sbyte) - 1, (sbyte) - 80, (sbyte)123, (sbyte) - 91, (sbyte)53, (sbyte)34, (sbyte) - 14, (sbyte)121, (sbyte) - 52, (sbyte) - 54, (sbyte)125, (sbyte) - 23, (sbyte) - 89, (sbyte)25, (sbyte) - 42, (sbyte) - 93, (sbyte) - 36, (sbyte) - 105, (sbyte)119, (sbyte) - 101, (sbyte) - 80, (sbyte) - 111, (sbyte)46, (sbyte) - 59, (sbyte) - 19, (sbyte) - 109, (sbyte)30, (sbyte)104, (sbyte) - 22, (sbyte) - 18, (sbyte) - 59, (sbyte) - 35, (sbyte)22, (sbyte)8, (sbyte) - 54, (sbyte) - 123}, 0) ;
            p155.start_index = (byte)(byte)186;
            p155.directory_type = (byte)(byte)15;
            p155.target_component = (byte)(byte)244;
            p155.count = (byte)(byte)117;
            CommunicationChannel.instance.send(p155);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnFLEXIFUNCTION_DIRECTORY_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.result == (ushort)(ushort)5162);
                Debug.Assert(pack.count == (byte)(byte)134);
                Debug.Assert(pack.target_system == (byte)(byte)197);
                Debug.Assert(pack.directory_type == (byte)(byte)52);
                Debug.Assert(pack.start_index == (byte)(byte)5);
                Debug.Assert(pack.target_component == (byte)(byte)16);
            };
            GroundControl.FLEXIFUNCTION_DIRECTORY_ACK p156 = CommunicationChannel.new_FLEXIFUNCTION_DIRECTORY_ACK();
            PH.setPack(p156);
            p156.directory_type = (byte)(byte)52;
            p156.result = (ushort)(ushort)5162;
            p156.target_system = (byte)(byte)197;
            p156.start_index = (byte)(byte)5;
            p156.count = (byte)(byte)134;
            p156.target_component = (byte)(byte)16;
            CommunicationChannel.instance.send(p156);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnFLEXIFUNCTION_COMMANDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)249);
                Debug.Assert(pack.command_type == (byte)(byte)248);
                Debug.Assert(pack.target_component == (byte)(byte)9);
            };
            GroundControl.FLEXIFUNCTION_COMMAND p157 = CommunicationChannel.new_FLEXIFUNCTION_COMMAND();
            PH.setPack(p157);
            p157.command_type = (byte)(byte)248;
            p157.target_component = (byte)(byte)9;
            p157.target_system = (byte)(byte)249;
            CommunicationChannel.instance.send(p157);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnFLEXIFUNCTION_COMMAND_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.result == (ushort)(ushort)50386);
                Debug.Assert(pack.command_type == (ushort)(ushort)48899);
            };
            GroundControl.FLEXIFUNCTION_COMMAND_ACK p158 = CommunicationChannel.new_FLEXIFUNCTION_COMMAND_ACK();
            PH.setPack(p158);
            p158.result = (ushort)(ushort)50386;
            p158.command_type = (ushort)(ushort)48899;
            CommunicationChannel.instance.send(p158);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSERIAL_UDB_EXTRA_F2_AReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sue_rmat4 == (short)(short)16349);
                Debug.Assert(pack.sue_air_speed_3DIMU == (ushort)(ushort)5797);
                Debug.Assert(pack.sue_waypoint_index == (ushort)(ushort)23180);
                Debug.Assert(pack.sue_rmat0 == (short)(short)27621);
                Debug.Assert(pack.sue_status == (byte)(byte)211);
                Debug.Assert(pack.sue_estimated_wind_0 == (short)(short)7709);
                Debug.Assert(pack.sue_hdop == (short)(short) -19602);
                Debug.Assert(pack.sue_rmat7 == (short)(short)21912);
                Debug.Assert(pack.sue_estimated_wind_1 == (short)(short) -31385);
                Debug.Assert(pack.sue_rmat2 == (short)(short)26049);
                Debug.Assert(pack.sue_cpu_load == (ushort)(ushort)56040);
                Debug.Assert(pack.sue_time == (uint)3403989934U);
                Debug.Assert(pack.sue_magFieldEarth2 == (short)(short) -16639);
                Debug.Assert(pack.sue_latitude == (int) -1846426511);
                Debug.Assert(pack.sue_estimated_wind_2 == (short)(short)24020);
                Debug.Assert(pack.sue_svs == (short)(short) -19019);
                Debug.Assert(pack.sue_rmat8 == (short)(short) -11099);
                Debug.Assert(pack.sue_altitude == (int)56677542);
                Debug.Assert(pack.sue_cog == (ushort)(ushort)6793);
                Debug.Assert(pack.sue_rmat6 == (short)(short)13536);
                Debug.Assert(pack.sue_longitude == (int)1177878456);
                Debug.Assert(pack.sue_rmat5 == (short)(short) -29857);
                Debug.Assert(pack.sue_magFieldEarth0 == (short)(short)28392);
                Debug.Assert(pack.sue_rmat1 == (short)(short)10664);
                Debug.Assert(pack.sue_rmat3 == (short)(short) -15054);
                Debug.Assert(pack.sue_magFieldEarth1 == (short)(short)23517);
                Debug.Assert(pack.sue_sog == (short)(short) -17589);
            };
            GroundControl.SERIAL_UDB_EXTRA_F2_A p170 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F2_A();
            PH.setPack(p170);
            p170.sue_longitude = (int)1177878456;
            p170.sue_altitude = (int)56677542;
            p170.sue_rmat5 = (short)(short) -29857;
            p170.sue_air_speed_3DIMU = (ushort)(ushort)5797;
            p170.sue_sog = (short)(short) -17589;
            p170.sue_cog = (ushort)(ushort)6793;
            p170.sue_magFieldEarth2 = (short)(short) -16639;
            p170.sue_rmat0 = (short)(short)27621;
            p170.sue_waypoint_index = (ushort)(ushort)23180;
            p170.sue_magFieldEarth1 = (short)(short)23517;
            p170.sue_rmat4 = (short)(short)16349;
            p170.sue_time = (uint)3403989934U;
            p170.sue_hdop = (short)(short) -19602;
            p170.sue_estimated_wind_1 = (short)(short) -31385;
            p170.sue_rmat2 = (short)(short)26049;
            p170.sue_cpu_load = (ushort)(ushort)56040;
            p170.sue_latitude = (int) -1846426511;
            p170.sue_status = (byte)(byte)211;
            p170.sue_rmat6 = (short)(short)13536;
            p170.sue_svs = (short)(short) -19019;
            p170.sue_rmat7 = (short)(short)21912;
            p170.sue_rmat3 = (short)(short) -15054;
            p170.sue_rmat8 = (short)(short) -11099;
            p170.sue_estimated_wind_0 = (short)(short)7709;
            p170.sue_estimated_wind_2 = (short)(short)24020;
            p170.sue_magFieldEarth0 = (short)(short)28392;
            p170.sue_rmat1 = (short)(short)10664;
            CommunicationChannel.instance.send(p170);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSERIAL_UDB_EXTRA_F2_BReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sue_pwm_output_7 == (short)(short) -2927);
                Debug.Assert(pack.sue_pwm_input_10 == (short)(short) -25860);
                Debug.Assert(pack.sue_aero_z == (short)(short)1839);
                Debug.Assert(pack.sue_bat_volt == (short)(short)15953);
                Debug.Assert(pack.sue_pwm_input_8 == (short)(short)31270);
                Debug.Assert(pack.sue_memory_stack_free == (short)(short)17621);
                Debug.Assert(pack.sue_pwm_input_4 == (short)(short)27367);
                Debug.Assert(pack.sue_aero_y == (short)(short)10848);
                Debug.Assert(pack.sue_barom_temp == (short)(short)4799);
                Debug.Assert(pack.sue_pwm_input_6 == (short)(short) -12207);
                Debug.Assert(pack.sue_pwm_input_7 == (short)(short) -16053);
                Debug.Assert(pack.sue_pwm_output_1 == (short)(short)28278);
                Debug.Assert(pack.sue_imu_location_y == (short)(short)14151);
                Debug.Assert(pack.sue_location_error_earth_y == (short)(short) -19056);
                Debug.Assert(pack.sue_pwm_input_11 == (short)(short) -9627);
                Debug.Assert(pack.sue_imu_location_z == (short)(short) -18368);
                Debug.Assert(pack.sue_osc_fails == (short)(short)10960);
                Debug.Assert(pack.sue_imu_location_x == (short)(short)9798);
                Debug.Assert(pack.sue_desired_height == (short)(short)18013);
                Debug.Assert(pack.sue_imu_velocity_y == (short)(short) -15546);
                Debug.Assert(pack.sue_pwm_input_5 == (short)(short)950);
                Debug.Assert(pack.sue_pwm_input_3 == (short)(short) -11171);
                Debug.Assert(pack.sue_barom_alt == (int)894885977);
                Debug.Assert(pack.sue_pwm_input_2 == (short)(short)1559);
                Debug.Assert(pack.sue_pwm_input_1 == (short)(short)16819);
                Debug.Assert(pack.sue_pwm_output_9 == (short)(short) -26261);
                Debug.Assert(pack.sue_imu_velocity_z == (short)(short)3293);
                Debug.Assert(pack.sue_pwm_output_5 == (short)(short) -6155);
                Debug.Assert(pack.sue_pwm_output_6 == (short)(short) -23669);
                Debug.Assert(pack.sue_time == (uint)3074114993U);
                Debug.Assert(pack.sue_pwm_output_2 == (short)(short) -28148);
                Debug.Assert(pack.sue_pwm_output_4 == (short)(short) -18290);
                Debug.Assert(pack.sue_location_error_earth_x == (short)(short)6070);
                Debug.Assert(pack.sue_barom_press == (int)980798074);
                Debug.Assert(pack.sue_imu_velocity_x == (short)(short)11497);
                Debug.Assert(pack.sue_waypoint_goal_x == (short)(short)21102);
                Debug.Assert(pack.sue_bat_amp_hours == (short)(short)13329);
                Debug.Assert(pack.sue_waypoint_goal_y == (short)(short) -31825);
                Debug.Assert(pack.sue_bat_amp == (short)(short)9305);
                Debug.Assert(pack.sue_flags == (uint)3108517983U);
                Debug.Assert(pack.sue_location_error_earth_z == (short)(short)5604);
                Debug.Assert(pack.sue_pwm_input_12 == (short)(short) -9348);
                Debug.Assert(pack.sue_pwm_output_3 == (short)(short) -20585);
                Debug.Assert(pack.sue_waypoint_goal_z == (short)(short)8124);
                Debug.Assert(pack.sue_pwm_output_12 == (short)(short) -28834);
                Debug.Assert(pack.sue_pwm_input_9 == (short)(short)4958);
                Debug.Assert(pack.sue_pwm_output_8 == (short)(short) -26912);
                Debug.Assert(pack.sue_aero_x == (short)(short)27864);
                Debug.Assert(pack.sue_pwm_output_11 == (short)(short)17862);
                Debug.Assert(pack.sue_pwm_output_10 == (short)(short)5489);
            };
            GroundControl.SERIAL_UDB_EXTRA_F2_B p171 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F2_B();
            PH.setPack(p171);
            p171.sue_time = (uint)3074114993U;
            p171.sue_pwm_input_3 = (short)(short) -11171;
            p171.sue_pwm_output_5 = (short)(short) -6155;
            p171.sue_pwm_output_4 = (short)(short) -18290;
            p171.sue_pwm_output_7 = (short)(short) -2927;
            p171.sue_location_error_earth_z = (short)(short)5604;
            p171.sue_bat_volt = (short)(short)15953;
            p171.sue_pwm_input_4 = (short)(short)27367;
            p171.sue_aero_x = (short)(short)27864;
            p171.sue_pwm_input_11 = (short)(short) -9627;
            p171.sue_pwm_output_6 = (short)(short) -23669;
            p171.sue_osc_fails = (short)(short)10960;
            p171.sue_bat_amp = (short)(short)9305;
            p171.sue_barom_press = (int)980798074;
            p171.sue_pwm_input_8 = (short)(short)31270;
            p171.sue_pwm_output_3 = (short)(short) -20585;
            p171.sue_waypoint_goal_x = (short)(short)21102;
            p171.sue_pwm_output_1 = (short)(short)28278;
            p171.sue_pwm_output_9 = (short)(short) -26261;
            p171.sue_pwm_output_10 = (short)(short)5489;
            p171.sue_imu_location_z = (short)(short) -18368;
            p171.sue_pwm_output_11 = (short)(short)17862;
            p171.sue_bat_amp_hours = (short)(short)13329;
            p171.sue_pwm_input_9 = (short)(short)4958;
            p171.sue_aero_z = (short)(short)1839;
            p171.sue_pwm_input_5 = (short)(short)950;
            p171.sue_pwm_input_12 = (short)(short) -9348;
            p171.sue_imu_location_x = (short)(short)9798;
            p171.sue_imu_velocity_y = (short)(short) -15546;
            p171.sue_desired_height = (short)(short)18013;
            p171.sue_location_error_earth_x = (short)(short)6070;
            p171.sue_pwm_output_8 = (short)(short) -26912;
            p171.sue_waypoint_goal_y = (short)(short) -31825;
            p171.sue_memory_stack_free = (short)(short)17621;
            p171.sue_pwm_input_6 = (short)(short) -12207;
            p171.sue_imu_velocity_z = (short)(short)3293;
            p171.sue_imu_location_y = (short)(short)14151;
            p171.sue_pwm_input_7 = (short)(short) -16053;
            p171.sue_pwm_input_2 = (short)(short)1559;
            p171.sue_pwm_output_12 = (short)(short) -28834;
            p171.sue_pwm_input_1 = (short)(short)16819;
            p171.sue_barom_temp = (short)(short)4799;
            p171.sue_flags = (uint)3108517983U;
            p171.sue_imu_velocity_x = (short)(short)11497;
            p171.sue_barom_alt = (int)894885977;
            p171.sue_pwm_input_10 = (short)(short) -25860;
            p171.sue_pwm_output_2 = (short)(short) -28148;
            p171.sue_waypoint_goal_z = (short)(short)8124;
            p171.sue_location_error_earth_y = (short)(short) -19056;
            p171.sue_aero_y = (short)(short)10848;
            CommunicationChannel.instance.send(p171);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSERIAL_UDB_EXTRA_F4Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sue_ALTITUDEHOLD_WAYPOINT == (byte)(byte)151);
                Debug.Assert(pack.sue_YAW_STABILIZATION_RUDDER == (byte)(byte)69);
                Debug.Assert(pack.sue_ROLL_STABILIZATION_AILERONS == (byte)(byte)71);
                Debug.Assert(pack.sue_PITCH_STABILIZATION == (byte)(byte)144);
                Debug.Assert(pack.sue_ROLL_STABILIZATION_RUDDER == (byte)(byte)140);
                Debug.Assert(pack.sue_ALTITUDEHOLD_STABILIZED == (byte)(byte)42);
                Debug.Assert(pack.sue_AILERON_NAVIGATION == (byte)(byte)203);
                Debug.Assert(pack.sue_YAW_STABILIZATION_AILERON == (byte)(byte)252);
                Debug.Assert(pack.sue_RUDDER_NAVIGATION == (byte)(byte)178);
                Debug.Assert(pack.sue_RACING_MODE == (byte)(byte)111);
            };
            GroundControl.SERIAL_UDB_EXTRA_F4 p172 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F4();
            PH.setPack(p172);
            p172.sue_AILERON_NAVIGATION = (byte)(byte)203;
            p172.sue_RACING_MODE = (byte)(byte)111;
            p172.sue_PITCH_STABILIZATION = (byte)(byte)144;
            p172.sue_ROLL_STABILIZATION_AILERONS = (byte)(byte)71;
            p172.sue_YAW_STABILIZATION_RUDDER = (byte)(byte)69;
            p172.sue_YAW_STABILIZATION_AILERON = (byte)(byte)252;
            p172.sue_ROLL_STABILIZATION_RUDDER = (byte)(byte)140;
            p172.sue_ALTITUDEHOLD_STABILIZED = (byte)(byte)42;
            p172.sue_RUDDER_NAVIGATION = (byte)(byte)178;
            p172.sue_ALTITUDEHOLD_WAYPOINT = (byte)(byte)151;
            CommunicationChannel.instance.send(p172);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSERIAL_UDB_EXTRA_F5Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sue_YAWKP_AILERON == (float) -1.037948E38F);
                Debug.Assert(pack.sue_ROLLKP == (float) -1.3128547E38F);
                Debug.Assert(pack.sue_YAWKD_AILERON == (float)1.1118226E38F);
                Debug.Assert(pack.sue_ROLLKD == (float) -2.6318984E38F);
            };
            GroundControl.SERIAL_UDB_EXTRA_F5 p173 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F5();
            PH.setPack(p173);
            p173.sue_YAWKD_AILERON = (float)1.1118226E38F;
            p173.sue_ROLLKP = (float) -1.3128547E38F;
            p173.sue_ROLLKD = (float) -2.6318984E38F;
            p173.sue_YAWKP_AILERON = (float) -1.037948E38F;
            CommunicationChannel.instance.send(p173);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSERIAL_UDB_EXTRA_F6Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sue_PITCHGAIN == (float) -3.0653386E38F);
                Debug.Assert(pack.sue_ELEVATOR_BOOST == (float)2.2675304E38F);
                Debug.Assert(pack.sue_PITCHKD == (float)1.3788113E38F);
                Debug.Assert(pack.sue_ROLL_ELEV_MIX == (float) -1.4556641E38F);
                Debug.Assert(pack.sue_RUDDER_ELEV_MIX == (float) -2.8249547E38F);
            };
            GroundControl.SERIAL_UDB_EXTRA_F6 p174 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F6();
            PH.setPack(p174);
            p174.sue_PITCHGAIN = (float) -3.0653386E38F;
            p174.sue_ROLL_ELEV_MIX = (float) -1.4556641E38F;
            p174.sue_RUDDER_ELEV_MIX = (float) -2.8249547E38F;
            p174.sue_PITCHKD = (float)1.3788113E38F;
            p174.sue_ELEVATOR_BOOST = (float)2.2675304E38F;
            CommunicationChannel.instance.send(p174);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSERIAL_UDB_EXTRA_F7Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sue_ROLLKD_RUDDER == (float) -1.8394854E38F);
                Debug.Assert(pack.sue_ROLLKP_RUDDER == (float)1.5735347E38F);
                Debug.Assert(pack.sue_YAWKD_RUDDER == (float) -7.9897026E37F);
                Debug.Assert(pack.sue_RUDDER_BOOST == (float)2.9263617E38F);
                Debug.Assert(pack.sue_RTL_PITCH_DOWN == (float)8.613792E37F);
                Debug.Assert(pack.sue_YAWKP_RUDDER == (float) -2.880925E37F);
            };
            GroundControl.SERIAL_UDB_EXTRA_F7 p175 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F7();
            PH.setPack(p175);
            p175.sue_ROLLKD_RUDDER = (float) -1.8394854E38F;
            p175.sue_RUDDER_BOOST = (float)2.9263617E38F;
            p175.sue_ROLLKP_RUDDER = (float)1.5735347E38F;
            p175.sue_YAWKD_RUDDER = (float) -7.9897026E37F;
            p175.sue_YAWKP_RUDDER = (float) -2.880925E37F;
            p175.sue_RTL_PITCH_DOWN = (float)8.613792E37F;
            CommunicationChannel.instance.send(p175);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSERIAL_UDB_EXTRA_F8Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sue_ALT_HOLD_PITCH_HIGH == (float)1.442168E38F);
                Debug.Assert(pack.sue_ALT_HOLD_THROTTLE_MAX == (float) -5.97217E37F);
                Debug.Assert(pack.sue_ALT_HOLD_PITCH_MAX == (float)3.13909E38F);
                Debug.Assert(pack.sue_ALT_HOLD_PITCH_MIN == (float)4.1844525E37F);
                Debug.Assert(pack.sue_HEIGHT_TARGET_MIN == (float)1.978675E38F);
                Debug.Assert(pack.sue_ALT_HOLD_THROTTLE_MIN == (float) -3.1015575E38F);
                Debug.Assert(pack.sue_HEIGHT_TARGET_MAX == (float) -1.2067804E38F);
            };
            GroundControl.SERIAL_UDB_EXTRA_F8 p176 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F8();
            PH.setPack(p176);
            p176.sue_ALT_HOLD_PITCH_HIGH = (float)1.442168E38F;
            p176.sue_ALT_HOLD_PITCH_MIN = (float)4.1844525E37F;
            p176.sue_HEIGHT_TARGET_MAX = (float) -1.2067804E38F;
            p176.sue_ALT_HOLD_THROTTLE_MAX = (float) -5.97217E37F;
            p176.sue_ALT_HOLD_THROTTLE_MIN = (float) -3.1015575E38F;
            p176.sue_ALT_HOLD_PITCH_MAX = (float)3.13909E38F;
            p176.sue_HEIGHT_TARGET_MIN = (float)1.978675E38F;
            CommunicationChannel.instance.send(p176);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSERIAL_UDB_EXTRA_F13Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sue_lat_origin == (int)912208896);
                Debug.Assert(pack.sue_lon_origin == (int) -1550900932);
                Debug.Assert(pack.sue_week_no == (short)(short)11763);
                Debug.Assert(pack.sue_alt_origin == (int)1359975268);
            };
            GroundControl.SERIAL_UDB_EXTRA_F13 p177 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F13();
            PH.setPack(p177);
            p177.sue_alt_origin = (int)1359975268;
            p177.sue_week_no = (short)(short)11763;
            p177.sue_lat_origin = (int)912208896;
            p177.sue_lon_origin = (int) -1550900932;
            CommunicationChannel.instance.send(p177);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSERIAL_UDB_EXTRA_F14Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sue_FLIGHT_PLAN_TYPE == (byte)(byte)27);
                Debug.Assert(pack.sue_RCON == (short)(short) -16625);
                Debug.Assert(pack.sue_GPS_TYPE == (byte)(byte)63);
                Debug.Assert(pack.sue_BOARD_TYPE == (byte)(byte)1);
                Debug.Assert(pack.sue_AIRFRAME == (byte)(byte)121);
                Debug.Assert(pack.sue_DR == (byte)(byte)250);
                Debug.Assert(pack.sue_CLOCK_CONFIG == (byte)(byte)47);
                Debug.Assert(pack.sue_WIND_ESTIMATION == (byte)(byte)181);
                Debug.Assert(pack.sue_TRAP_FLAGS == (short)(short)5817);
                Debug.Assert(pack.sue_osc_fail_count == (short)(short)31526);
                Debug.Assert(pack.sue_TRAP_SOURCE == (uint)472925721U);
            };
            GroundControl.SERIAL_UDB_EXTRA_F14 p178 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F14();
            PH.setPack(p178);
            p178.sue_GPS_TYPE = (byte)(byte)63;
            p178.sue_osc_fail_count = (short)(short)31526;
            p178.sue_WIND_ESTIMATION = (byte)(byte)181;
            p178.sue_CLOCK_CONFIG = (byte)(byte)47;
            p178.sue_TRAP_FLAGS = (short)(short)5817;
            p178.sue_TRAP_SOURCE = (uint)472925721U;
            p178.sue_FLIGHT_PLAN_TYPE = (byte)(byte)27;
            p178.sue_AIRFRAME = (byte)(byte)121;
            p178.sue_RCON = (short)(short) -16625;
            p178.sue_DR = (byte)(byte)250;
            p178.sue_BOARD_TYPE = (byte)(byte)1;
            CommunicationChannel.instance.send(p178);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSERIAL_UDB_EXTRA_F15Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sue_ID_VEHICLE_REGISTRATION.SequenceEqual(new byte[] {(byte)239, (byte)189, (byte)158, (byte)241, (byte)152, (byte)247, (byte)181, (byte)27, (byte)114, (byte)44, (byte)71, (byte)49, (byte)164, (byte)125, (byte)242, (byte)220, (byte)220, (byte)96, (byte)28, (byte)215}));
                Debug.Assert(pack.sue_ID_VEHICLE_MODEL_NAME.SequenceEqual(new byte[] {(byte)27, (byte)245, (byte)195, (byte)158, (byte)103, (byte)43, (byte)245, (byte)160, (byte)216, (byte)204, (byte)226, (byte)41, (byte)201, (byte)8, (byte)223, (byte)63, (byte)131, (byte)242, (byte)128, (byte)106, (byte)17, (byte)227, (byte)153, (byte)178, (byte)26, (byte)84, (byte)95, (byte)39, (byte)169, (byte)227, (byte)240, (byte)58, (byte)130, (byte)153, (byte)251, (byte)15, (byte)104, (byte)222, (byte)231, (byte)143}));
            };
            GroundControl.SERIAL_UDB_EXTRA_F15 p179 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F15();
            PH.setPack(p179);
            p179.sue_ID_VEHICLE_MODEL_NAME_SET(new byte[] {(byte)27, (byte)245, (byte)195, (byte)158, (byte)103, (byte)43, (byte)245, (byte)160, (byte)216, (byte)204, (byte)226, (byte)41, (byte)201, (byte)8, (byte)223, (byte)63, (byte)131, (byte)242, (byte)128, (byte)106, (byte)17, (byte)227, (byte)153, (byte)178, (byte)26, (byte)84, (byte)95, (byte)39, (byte)169, (byte)227, (byte)240, (byte)58, (byte)130, (byte)153, (byte)251, (byte)15, (byte)104, (byte)222, (byte)231, (byte)143}, 0) ;
            p179.sue_ID_VEHICLE_REGISTRATION_SET(new byte[] {(byte)239, (byte)189, (byte)158, (byte)241, (byte)152, (byte)247, (byte)181, (byte)27, (byte)114, (byte)44, (byte)71, (byte)49, (byte)164, (byte)125, (byte)242, (byte)220, (byte)220, (byte)96, (byte)28, (byte)215}, 0) ;
            CommunicationChannel.instance.send(p179);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSERIAL_UDB_EXTRA_F16Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sue_ID_DIY_DRONES_URL.SequenceEqual(new byte[] {(byte)1, (byte)106, (byte)67, (byte)218, (byte)54, (byte)189, (byte)100, (byte)146, (byte)128, (byte)155, (byte)180, (byte)218, (byte)20, (byte)203, (byte)186, (byte)229, (byte)38, (byte)191, (byte)43, (byte)238, (byte)147, (byte)88, (byte)117, (byte)135, (byte)218, (byte)46, (byte)135, (byte)115, (byte)6, (byte)27, (byte)130, (byte)251, (byte)129, (byte)230, (byte)174, (byte)22, (byte)224, (byte)49, (byte)7, (byte)17, (byte)198, (byte)254, (byte)246, (byte)82, (byte)41, (byte)237, (byte)110, (byte)229, (byte)206, (byte)233, (byte)136, (byte)209, (byte)253, (byte)35, (byte)86, (byte)31, (byte)48, (byte)45, (byte)46, (byte)91, (byte)150, (byte)205, (byte)215, (byte)36, (byte)162, (byte)95, (byte)175, (byte)238, (byte)206, (byte)110}));
                Debug.Assert(pack.sue_ID_LEAD_PILOT.SequenceEqual(new byte[] {(byte)248, (byte)100, (byte)174, (byte)128, (byte)199, (byte)186, (byte)237, (byte)225, (byte)193, (byte)10, (byte)122, (byte)56, (byte)22, (byte)33, (byte)34, (byte)80, (byte)111, (byte)219, (byte)239, (byte)112, (byte)251, (byte)42, (byte)97, (byte)150, (byte)38, (byte)223, (byte)111, (byte)200, (byte)169, (byte)135, (byte)122, (byte)53, (byte)75, (byte)28, (byte)208, (byte)219, (byte)89, (byte)192, (byte)147, (byte)134}));
            };
            GroundControl.SERIAL_UDB_EXTRA_F16 p180 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F16();
            PH.setPack(p180);
            p180.sue_ID_DIY_DRONES_URL_SET(new byte[] {(byte)1, (byte)106, (byte)67, (byte)218, (byte)54, (byte)189, (byte)100, (byte)146, (byte)128, (byte)155, (byte)180, (byte)218, (byte)20, (byte)203, (byte)186, (byte)229, (byte)38, (byte)191, (byte)43, (byte)238, (byte)147, (byte)88, (byte)117, (byte)135, (byte)218, (byte)46, (byte)135, (byte)115, (byte)6, (byte)27, (byte)130, (byte)251, (byte)129, (byte)230, (byte)174, (byte)22, (byte)224, (byte)49, (byte)7, (byte)17, (byte)198, (byte)254, (byte)246, (byte)82, (byte)41, (byte)237, (byte)110, (byte)229, (byte)206, (byte)233, (byte)136, (byte)209, (byte)253, (byte)35, (byte)86, (byte)31, (byte)48, (byte)45, (byte)46, (byte)91, (byte)150, (byte)205, (byte)215, (byte)36, (byte)162, (byte)95, (byte)175, (byte)238, (byte)206, (byte)110}, 0) ;
            p180.sue_ID_LEAD_PILOT_SET(new byte[] {(byte)248, (byte)100, (byte)174, (byte)128, (byte)199, (byte)186, (byte)237, (byte)225, (byte)193, (byte)10, (byte)122, (byte)56, (byte)22, (byte)33, (byte)34, (byte)80, (byte)111, (byte)219, (byte)239, (byte)112, (byte)251, (byte)42, (byte)97, (byte)150, (byte)38, (byte)223, (byte)111, (byte)200, (byte)169, (byte)135, (byte)122, (byte)53, (byte)75, (byte)28, (byte)208, (byte)219, (byte)89, (byte)192, (byte)147, (byte)134}, 0) ;
            CommunicationChannel.instance.send(p180);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnALTITUDESReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.alt_gps == (int) -482297013);
                Debug.Assert(pack.alt_imu == (int) -1914021819);
                Debug.Assert(pack.alt_barometric == (int) -1604084267);
                Debug.Assert(pack.alt_optical_flow == (int)1440010712);
                Debug.Assert(pack.alt_extra == (int)520821311);
                Debug.Assert(pack.alt_range_finder == (int)689299775);
                Debug.Assert(pack.time_boot_ms == (uint)1471658400U);
            };
            GroundControl.ALTITUDES p181 = CommunicationChannel.new_ALTITUDES();
            PH.setPack(p181);
            p181.alt_imu = (int) -1914021819;
            p181.alt_optical_flow = (int)1440010712;
            p181.time_boot_ms = (uint)1471658400U;
            p181.alt_range_finder = (int)689299775;
            p181.alt_extra = (int)520821311;
            p181.alt_barometric = (int) -1604084267;
            p181.alt_gps = (int) -482297013;
            CommunicationChannel.instance.send(p181);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnAIRSPEEDSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)2591953283U);
                Debug.Assert(pack.airspeed_hot_wire == (short)(short)26934);
                Debug.Assert(pack.airspeed_pitot == (short)(short)31071);
                Debug.Assert(pack.aoy == (short)(short)7541);
                Debug.Assert(pack.aoa == (short)(short) -25780);
                Debug.Assert(pack.airspeed_imu == (short)(short) -17904);
                Debug.Assert(pack.airspeed_ultrasonic == (short)(short) -3253);
            };
            GroundControl.AIRSPEEDS p182 = CommunicationChannel.new_AIRSPEEDS();
            PH.setPack(p182);
            p182.airspeed_pitot = (short)(short)31071;
            p182.airspeed_ultrasonic = (short)(short) -3253;
            p182.time_boot_ms = (uint)2591953283U;
            p182.airspeed_imu = (short)(short) -17904;
            p182.airspeed_hot_wire = (short)(short)26934;
            p182.aoa = (short)(short) -25780;
            p182.aoy = (short)(short)7541;
            CommunicationChannel.instance.send(p182);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSERIAL_UDB_EXTRA_F17Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sue_turn_rate_fbw == (float)1.5541983E38F);
                Debug.Assert(pack.sue_turn_rate_nav == (float)2.1449727E38F);
                Debug.Assert(pack.sue_feed_forward == (float) -1.2638519E38F);
            };
            GroundControl.SERIAL_UDB_EXTRA_F17 p183 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F17();
            PH.setPack(p183);
            p183.sue_feed_forward = (float) -1.2638519E38F;
            p183.sue_turn_rate_fbw = (float)1.5541983E38F;
            p183.sue_turn_rate_nav = (float)2.1449727E38F;
            CommunicationChannel.instance.send(p183);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSERIAL_UDB_EXTRA_F18Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.elevator_trim_normal == (float) -4.96109E37F);
                Debug.Assert(pack.elevator_trim_inverted == (float)1.8122795E38F);
                Debug.Assert(pack.angle_of_attack_normal == (float)7.6341286E37F);
                Debug.Assert(pack.angle_of_attack_inverted == (float) -1.4171437E38F);
                Debug.Assert(pack.reference_speed == (float) -1.4569968E38F);
            };
            GroundControl.SERIAL_UDB_EXTRA_F18 p184 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F18();
            PH.setPack(p184);
            p184.angle_of_attack_inverted = (float) -1.4171437E38F;
            p184.elevator_trim_inverted = (float)1.8122795E38F;
            p184.angle_of_attack_normal = (float)7.6341286E37F;
            p184.reference_speed = (float) -1.4569968E38F;
            p184.elevator_trim_normal = (float) -4.96109E37F;
            CommunicationChannel.instance.send(p184);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSERIAL_UDB_EXTRA_F19Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sue_rudder_reversed == (byte)(byte)152);
                Debug.Assert(pack.sue_throttle_output_channel == (byte)(byte)201);
                Debug.Assert(pack.sue_aileron_output_channel == (byte)(byte)204);
                Debug.Assert(pack.sue_rudder_output_channel == (byte)(byte)175);
                Debug.Assert(pack.sue_elevator_reversed == (byte)(byte)224);
                Debug.Assert(pack.sue_elevator_output_channel == (byte)(byte)15);
                Debug.Assert(pack.sue_aileron_reversed == (byte)(byte)64);
                Debug.Assert(pack.sue_throttle_reversed == (byte)(byte)154);
            };
            GroundControl.SERIAL_UDB_EXTRA_F19 p185 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F19();
            PH.setPack(p185);
            p185.sue_aileron_output_channel = (byte)(byte)204;
            p185.sue_aileron_reversed = (byte)(byte)64;
            p185.sue_rudder_output_channel = (byte)(byte)175;
            p185.sue_throttle_output_channel = (byte)(byte)201;
            p185.sue_elevator_reversed = (byte)(byte)224;
            p185.sue_elevator_output_channel = (byte)(byte)15;
            p185.sue_rudder_reversed = (byte)(byte)152;
            p185.sue_throttle_reversed = (byte)(byte)154;
            CommunicationChannel.instance.send(p185);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSERIAL_UDB_EXTRA_F20Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sue_trim_value_input_6 == (short)(short)19286);
                Debug.Assert(pack.sue_trim_value_input_1 == (short)(short) -12210);
                Debug.Assert(pack.sue_trim_value_input_8 == (short)(short) -12692);
                Debug.Assert(pack.sue_trim_value_input_11 == (short)(short)13277);
                Debug.Assert(pack.sue_trim_value_input_4 == (short)(short) -12479);
                Debug.Assert(pack.sue_trim_value_input_12 == (short)(short)21822);
                Debug.Assert(pack.sue_trim_value_input_2 == (short)(short) -3754);
                Debug.Assert(pack.sue_trim_value_input_10 == (short)(short) -30335);
                Debug.Assert(pack.sue_trim_value_input_5 == (short)(short) -12904);
                Debug.Assert(pack.sue_trim_value_input_9 == (short)(short)26974);
                Debug.Assert(pack.sue_trim_value_input_7 == (short)(short)19663);
                Debug.Assert(pack.sue_number_of_inputs == (byte)(byte)71);
                Debug.Assert(pack.sue_trim_value_input_3 == (short)(short)372);
            };
            GroundControl.SERIAL_UDB_EXTRA_F20 p186 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F20();
            PH.setPack(p186);
            p186.sue_trim_value_input_1 = (short)(short) -12210;
            p186.sue_number_of_inputs = (byte)(byte)71;
            p186.sue_trim_value_input_5 = (short)(short) -12904;
            p186.sue_trim_value_input_7 = (short)(short)19663;
            p186.sue_trim_value_input_8 = (short)(short) -12692;
            p186.sue_trim_value_input_10 = (short)(short) -30335;
            p186.sue_trim_value_input_12 = (short)(short)21822;
            p186.sue_trim_value_input_6 = (short)(short)19286;
            p186.sue_trim_value_input_3 = (short)(short)372;
            p186.sue_trim_value_input_11 = (short)(short)13277;
            p186.sue_trim_value_input_2 = (short)(short) -3754;
            p186.sue_trim_value_input_9 = (short)(short)26974;
            p186.sue_trim_value_input_4 = (short)(short) -12479;
            CommunicationChannel.instance.send(p186);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSERIAL_UDB_EXTRA_F21Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sue_gyro_y_offset == (short)(short) -14910);
                Debug.Assert(pack.sue_gyro_z_offset == (short)(short)18414);
                Debug.Assert(pack.sue_accel_z_offset == (short)(short) -5149);
                Debug.Assert(pack.sue_gyro_x_offset == (short)(short) -15553);
                Debug.Assert(pack.sue_accel_x_offset == (short)(short)25108);
                Debug.Assert(pack.sue_accel_y_offset == (short)(short) -1464);
            };
            GroundControl.SERIAL_UDB_EXTRA_F21 p187 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F21();
            PH.setPack(p187);
            p187.sue_accel_y_offset = (short)(short) -1464;
            p187.sue_accel_x_offset = (short)(short)25108;
            p187.sue_gyro_x_offset = (short)(short) -15553;
            p187.sue_gyro_z_offset = (short)(short)18414;
            p187.sue_accel_z_offset = (short)(short) -5149;
            p187.sue_gyro_y_offset = (short)(short) -14910;
            CommunicationChannel.instance.send(p187);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSERIAL_UDB_EXTRA_F22Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sue_accel_z_at_calibration == (short)(short) -26981);
                Debug.Assert(pack.sue_gyro_y_at_calibration == (short)(short) -5862);
                Debug.Assert(pack.sue_accel_x_at_calibration == (short)(short) -26398);
                Debug.Assert(pack.sue_accel_y_at_calibration == (short)(short)6845);
                Debug.Assert(pack.sue_gyro_z_at_calibration == (short)(short) -4200);
                Debug.Assert(pack.sue_gyro_x_at_calibration == (short)(short) -1256);
            };
            GroundControl.SERIAL_UDB_EXTRA_F22 p188 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F22();
            PH.setPack(p188);
            p188.sue_gyro_z_at_calibration = (short)(short) -4200;
            p188.sue_accel_x_at_calibration = (short)(short) -26398;
            p188.sue_gyro_x_at_calibration = (short)(short) -1256;
            p188.sue_accel_z_at_calibration = (short)(short) -26981;
            p188.sue_gyro_y_at_calibration = (short)(short) -5862;
            p188.sue_accel_y_at_calibration = (short)(short)6845;
            CommunicationChannel.instance.send(p188);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            CommunicationChannel.instance.OnESTIMATOR_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vel_ratio == (float)2.2068754E38F);
                Debug.Assert(pack.mag_ratio == (float) -2.233345E38F);
                Debug.Assert(pack.pos_vert_ratio == (float)1.1086946E37F);
                Debug.Assert(pack.tas_ratio == (float)6.2881377E37F);
                Debug.Assert(pack.flags == (ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_VERT |
                                            ESTIMATOR_STATUS_FLAGS.ESTIMATOR_PRED_POS_HORIZ_REL |
                                            ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_VERT_AGL |
                                            ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_HORIZ_ABS |
                                            ESTIMATOR_STATUS_FLAGS.ESTIMATOR_GPS_GLITCH |
                                            ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_HORIZ |
                                            ESTIMATOR_STATUS_FLAGS.ESTIMATOR_PRED_POS_HORIZ_ABS));
                Debug.Assert(pack.time_usec == (ulong)113345794242444728L);
                Debug.Assert(pack.hagl_ratio == (float) -3.3634539E38F);
                Debug.Assert(pack.pos_horiz_ratio == (float) -2.0651766E38F);
                Debug.Assert(pack.pos_horiz_accuracy == (float)1.140243E38F);
                Debug.Assert(pack.pos_vert_accuracy == (float)2.7170675E38F);
            };
            GroundControl.ESTIMATOR_STATUS p230 = CommunicationChannel.new_ESTIMATOR_STATUS();
            PH.setPack(p230);
            p230.pos_vert_accuracy = (float)2.7170675E38F;
            p230.hagl_ratio = (float) -3.3634539E38F;
            p230.mag_ratio = (float) -2.233345E38F;
            p230.time_usec = (ulong)113345794242444728L;
            p230.tas_ratio = (float)6.2881377E37F;
            p230.pos_horiz_accuracy = (float)1.140243E38F;
            p230.pos_vert_ratio = (float)1.1086946E37F;
            p230.pos_horiz_ratio = (float) -2.0651766E38F;
            p230.vel_ratio = (float)2.2068754E38F;
            p230.flags = (ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_VERT |
                          ESTIMATOR_STATUS_FLAGS.ESTIMATOR_PRED_POS_HORIZ_REL |
                          ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_VERT_AGL |
                          ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_HORIZ_ABS |
                          ESTIMATOR_STATUS_FLAGS.ESTIMATOR_GPS_GLITCH |
                          ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_HORIZ |
                          ESTIMATOR_STATUS_FLAGS.ESTIMATOR_PRED_POS_HORIZ_ABS);
            CommunicationChannel.instance.send(p230);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnWIND_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)2870690333891430110L);
                Debug.Assert(pack.vert_accuracy == (float) -9.741987E37F);
                Debug.Assert(pack.wind_y == (float) -2.213926E38F);
                Debug.Assert(pack.var_vert == (float)2.2704411E38F);
                Debug.Assert(pack.wind_z == (float)5.284946E37F);
                Debug.Assert(pack.horiz_accuracy == (float)2.0697225E38F);
                Debug.Assert(pack.wind_x == (float) -1.762383E38F);
                Debug.Assert(pack.var_horiz == (float)3.3617475E38F);
                Debug.Assert(pack.wind_alt == (float)1.1521081E38F);
            };
            GroundControl.WIND_COV p231 = CommunicationChannel.new_WIND_COV();
            PH.setPack(p231);
            p231.var_vert = (float)2.2704411E38F;
            p231.vert_accuracy = (float) -9.741987E37F;
            p231.var_horiz = (float)3.3617475E38F;
            p231.wind_y = (float) -2.213926E38F;
            p231.wind_alt = (float)1.1521081E38F;
            p231.wind_x = (float) -1.762383E38F;
            p231.wind_z = (float)5.284946E37F;
            p231.horiz_accuracy = (float)2.0697225E38F;
            p231.time_usec = (ulong)2870690333891430110L;
            CommunicationChannel.instance.send(p231);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_INPUTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.hdop == (float)2.2429672E38F);
                Debug.Assert(pack.fix_type == (byte)(byte)72);
                Debug.Assert(pack.time_week == (ushort)(ushort)29412);
                Debug.Assert(pack.vd == (float)2.9629483E38F);
                Debug.Assert(pack.vdop == (float) -1.6748981E38F);
                Debug.Assert(pack.lon == (int) -1841891345);
                Debug.Assert(pack.time_week_ms == (uint)121461953U);
                Debug.Assert(pack.ve == (float) -4.3336816E37F);
                Debug.Assert(pack.vn == (float) -1.8769273E38F);
                Debug.Assert(pack.speed_accuracy == (float) -9.575963E37F);
                Debug.Assert(pack.horiz_accuracy == (float) -1.3664729E38F);
                Debug.Assert(pack.alt == (float) -2.5927605E38F);
                Debug.Assert(pack.vert_accuracy == (float) -1.622128E38F);
                Debug.Assert(pack.satellites_visible == (byte)(byte)128);
                Debug.Assert(pack.gps_id == (byte)(byte)192);
                Debug.Assert(pack.ignore_flags == (GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY |
                                                   GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_HDOP |
                                                   GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY |
                                                   GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VEL_VERT));
                Debug.Assert(pack.lat == (int)579344187);
                Debug.Assert(pack.time_usec == (ulong)6880997450513963411L);
            };
            GroundControl.GPS_INPUT p232 = CommunicationChannel.new_GPS_INPUT();
            PH.setPack(p232);
            p232.lon = (int) -1841891345;
            p232.fix_type = (byte)(byte)72;
            p232.ignore_flags = (GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY |
                                 GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_HDOP |
                                 GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY |
                                 GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VEL_VERT);
            p232.gps_id = (byte)(byte)192;
            p232.time_week = (ushort)(ushort)29412;
            p232.vert_accuracy = (float) -1.622128E38F;
            p232.vdop = (float) -1.6748981E38F;
            p232.ve = (float) -4.3336816E37F;
            p232.speed_accuracy = (float) -9.575963E37F;
            p232.horiz_accuracy = (float) -1.3664729E38F;
            p232.vd = (float)2.9629483E38F;
            p232.time_usec = (ulong)6880997450513963411L;
            p232.satellites_visible = (byte)(byte)128;
            p232.hdop = (float)2.2429672E38F;
            p232.alt = (float) -2.5927605E38F;
            p232.vn = (float) -1.8769273E38F;
            p232.time_week_ms = (uint)121461953U;
            p232.lat = (int)579344187;
            CommunicationChannel.instance.send(p232);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_RTCM_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)70, (byte)118, (byte)51, (byte)59, (byte)143, (byte)117, (byte)39, (byte)234, (byte)194, (byte)64, (byte)80, (byte)187, (byte)180, (byte)168, (byte)25, (byte)125, (byte)154, (byte)179, (byte)81, (byte)216, (byte)157, (byte)31, (byte)37, (byte)250, (byte)243, (byte)59, (byte)109, (byte)76, (byte)245, (byte)4, (byte)169, (byte)74, (byte)224, (byte)233, (byte)67, (byte)200, (byte)193, (byte)77, (byte)153, (byte)175, (byte)191, (byte)46, (byte)56, (byte)212, (byte)14, (byte)204, (byte)160, (byte)51, (byte)154, (byte)47, (byte)230, (byte)102, (byte)93, (byte)125, (byte)15, (byte)202, (byte)171, (byte)146, (byte)110, (byte)111, (byte)50, (byte)249, (byte)58, (byte)10, (byte)164, (byte)81, (byte)214, (byte)139, (byte)73, (byte)149, (byte)15, (byte)28, (byte)105, (byte)66, (byte)137, (byte)12, (byte)188, (byte)169, (byte)147, (byte)221, (byte)91, (byte)239, (byte)174, (byte)21, (byte)140, (byte)172, (byte)170, (byte)220, (byte)1, (byte)232, (byte)5, (byte)38, (byte)190, (byte)205, (byte)10, (byte)130, (byte)177, (byte)110, (byte)199, (byte)242, (byte)56, (byte)203, (byte)2, (byte)77, (byte)25, (byte)134, (byte)150, (byte)94, (byte)3, (byte)157, (byte)15, (byte)49, (byte)129, (byte)56, (byte)174, (byte)87, (byte)154, (byte)28, (byte)146, (byte)123, (byte)33, (byte)167, (byte)46, (byte)24, (byte)191, (byte)139, (byte)161, (byte)115, (byte)77, (byte)131, (byte)232, (byte)227, (byte)245, (byte)170, (byte)203, (byte)107, (byte)205, (byte)215, (byte)82, (byte)173, (byte)49, (byte)201, (byte)249, (byte)3, (byte)61, (byte)198, (byte)148, (byte)121, (byte)149, (byte)27, (byte)59, (byte)162, (byte)248, (byte)154, (byte)218, (byte)136, (byte)20, (byte)102, (byte)32, (byte)116, (byte)109, (byte)119, (byte)89, (byte)28, (byte)231, (byte)28, (byte)22, (byte)97, (byte)124, (byte)199, (byte)30, (byte)249, (byte)195, (byte)229, (byte)174, (byte)215, (byte)194, (byte)187, (byte)114, (byte)36}));
                Debug.Assert(pack.len == (byte)(byte)212);
                Debug.Assert(pack.flags == (byte)(byte)183);
            };
            GroundControl.GPS_RTCM_DATA p233 = CommunicationChannel.new_GPS_RTCM_DATA();
            PH.setPack(p233);
            p233.flags = (byte)(byte)183;
            p233.len = (byte)(byte)212;
            p233.data__SET(new byte[] {(byte)70, (byte)118, (byte)51, (byte)59, (byte)143, (byte)117, (byte)39, (byte)234, (byte)194, (byte)64, (byte)80, (byte)187, (byte)180, (byte)168, (byte)25, (byte)125, (byte)154, (byte)179, (byte)81, (byte)216, (byte)157, (byte)31, (byte)37, (byte)250, (byte)243, (byte)59, (byte)109, (byte)76, (byte)245, (byte)4, (byte)169, (byte)74, (byte)224, (byte)233, (byte)67, (byte)200, (byte)193, (byte)77, (byte)153, (byte)175, (byte)191, (byte)46, (byte)56, (byte)212, (byte)14, (byte)204, (byte)160, (byte)51, (byte)154, (byte)47, (byte)230, (byte)102, (byte)93, (byte)125, (byte)15, (byte)202, (byte)171, (byte)146, (byte)110, (byte)111, (byte)50, (byte)249, (byte)58, (byte)10, (byte)164, (byte)81, (byte)214, (byte)139, (byte)73, (byte)149, (byte)15, (byte)28, (byte)105, (byte)66, (byte)137, (byte)12, (byte)188, (byte)169, (byte)147, (byte)221, (byte)91, (byte)239, (byte)174, (byte)21, (byte)140, (byte)172, (byte)170, (byte)220, (byte)1, (byte)232, (byte)5, (byte)38, (byte)190, (byte)205, (byte)10, (byte)130, (byte)177, (byte)110, (byte)199, (byte)242, (byte)56, (byte)203, (byte)2, (byte)77, (byte)25, (byte)134, (byte)150, (byte)94, (byte)3, (byte)157, (byte)15, (byte)49, (byte)129, (byte)56, (byte)174, (byte)87, (byte)154, (byte)28, (byte)146, (byte)123, (byte)33, (byte)167, (byte)46, (byte)24, (byte)191, (byte)139, (byte)161, (byte)115, (byte)77, (byte)131, (byte)232, (byte)227, (byte)245, (byte)170, (byte)203, (byte)107, (byte)205, (byte)215, (byte)82, (byte)173, (byte)49, (byte)201, (byte)249, (byte)3, (byte)61, (byte)198, (byte)148, (byte)121, (byte)149, (byte)27, (byte)59, (byte)162, (byte)248, (byte)154, (byte)218, (byte)136, (byte)20, (byte)102, (byte)32, (byte)116, (byte)109, (byte)119, (byte)89, (byte)28, (byte)231, (byte)28, (byte)22, (byte)97, (byte)124, (byte)199, (byte)30, (byte)249, (byte)195, (byte)229, (byte)174, (byte)215, (byte)194, (byte)187, (byte)114, (byte)36}, 0) ;
            CommunicationChannel.instance.send(p233);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIGH_LATENCYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.pitch == (short)(short) -10952);
                Debug.Assert(pack.heading == (ushort)(ushort)48295);
                Debug.Assert(pack.throttle == (sbyte)(sbyte) - 125);
                Debug.Assert(pack.airspeed == (byte)(byte)248);
                Debug.Assert(pack.groundspeed == (byte)(byte)115);
                Debug.Assert(pack.temperature == (sbyte)(sbyte)67);
                Debug.Assert(pack.landed_state == MAV_LANDED_STATE.MAV_LANDED_STATE_UNDEFINED);
                Debug.Assert(pack.climb_rate == (sbyte)(sbyte)101);
                Debug.Assert(pack.temperature_air == (sbyte)(sbyte)64);
                Debug.Assert(pack.wp_distance == (ushort)(ushort)35808);
                Debug.Assert(pack.roll == (short)(short)26703);
                Debug.Assert(pack.altitude_sp == (short)(short) -30627);
                Debug.Assert(pack.altitude_amsl == (short)(short) -32041);
                Debug.Assert(pack.heading_sp == (short)(short) -7472);
                Debug.Assert(pack.custom_mode == (uint)823346258U);
                Debug.Assert(pack.base_mode == (MAV_MODE_FLAG.MAV_MODE_FLAG_TEST_ENABLED |
                                                MAV_MODE_FLAG.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED));
                Debug.Assert(pack.longitude == (int)1208473824);
                Debug.Assert(pack.latitude == (int) -1189003853);
                Debug.Assert(pack.battery_remaining == (byte)(byte)104);
                Debug.Assert(pack.gps_nsat == (byte)(byte)111);
                Debug.Assert(pack.airspeed_sp == (byte)(byte)18);
                Debug.Assert(pack.gps_fix_type == GPS_FIX_TYPE.GPS_FIX_TYPE_RTK_FLOAT);
                Debug.Assert(pack.wp_num == (byte)(byte)137);
                Debug.Assert(pack.failsafe == (byte)(byte)192);
            };
            GroundControl.HIGH_LATENCY p234 = CommunicationChannel.new_HIGH_LATENCY();
            PH.setPack(p234);
            p234.altitude_amsl = (short)(short) -32041;
            p234.failsafe = (byte)(byte)192;
            p234.battery_remaining = (byte)(byte)104;
            p234.gps_nsat = (byte)(byte)111;
            p234.airspeed = (byte)(byte)248;
            p234.heading_sp = (short)(short) -7472;
            p234.altitude_sp = (short)(short) -30627;
            p234.wp_num = (byte)(byte)137;
            p234.heading = (ushort)(ushort)48295;
            p234.landed_state = MAV_LANDED_STATE.MAV_LANDED_STATE_UNDEFINED;
            p234.climb_rate = (sbyte)(sbyte)101;
            p234.throttle = (sbyte)(sbyte) - 125;
            p234.airspeed_sp = (byte)(byte)18;
            p234.latitude = (int) -1189003853;
            p234.wp_distance = (ushort)(ushort)35808;
            p234.gps_fix_type = GPS_FIX_TYPE.GPS_FIX_TYPE_RTK_FLOAT;
            p234.groundspeed = (byte)(byte)115;
            p234.temperature = (sbyte)(sbyte)67;
            p234.custom_mode = (uint)823346258U;
            p234.roll = (short)(short)26703;
            p234.pitch = (short)(short) -10952;
            p234.temperature_air = (sbyte)(sbyte)64;
            p234.longitude = (int)1208473824;
            p234.base_mode = (MAV_MODE_FLAG.MAV_MODE_FLAG_TEST_ENABLED |
                              MAV_MODE_FLAG.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED);
            CommunicationChannel.instance.send(p234);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnVIBRATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vibration_z == (float) -2.3219244E38F);
                Debug.Assert(pack.clipping_0 == (uint)2627108849U);
                Debug.Assert(pack.clipping_2 == (uint)1616944492U);
                Debug.Assert(pack.clipping_1 == (uint)3023483733U);
                Debug.Assert(pack.time_usec == (ulong)1180276572440409983L);
                Debug.Assert(pack.vibration_x == (float)1.8474496E38F);
                Debug.Assert(pack.vibration_y == (float)1.953819E38F);
            };
            GroundControl.VIBRATION p241 = CommunicationChannel.new_VIBRATION();
            PH.setPack(p241);
            p241.vibration_y = (float)1.953819E38F;
            p241.clipping_0 = (uint)2627108849U;
            p241.clipping_1 = (uint)3023483733U;
            p241.vibration_x = (float)1.8474496E38F;
            p241.vibration_z = (float) -2.3219244E38F;
            p241.clipping_2 = (uint)1616944492U;
            p241.time_usec = (ulong)1180276572440409983L;
            CommunicationChannel.instance.send(p241);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHOME_POSITIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.approach_x == (float)8.043136E37F);
                Debug.Assert(pack.longitude == (int)396169021);
                Debug.Assert(pack.z == (float) -1.5039243E38F);
                Debug.Assert(pack.approach_z == (float)1.4698179E38F);
                Debug.Assert(pack.y == (float) -2.9055363E37F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-2.9681201E38F, 2.6893597E38F, -1.5417413E38F, -2.3176507E38F}));
                Debug.Assert(pack.latitude == (int) -898137631);
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)9158414482931091853L);
                Debug.Assert(pack.altitude == (int)1880478318);
                Debug.Assert(pack.x == (float) -2.2984927E38F);
                Debug.Assert(pack.approach_y == (float) -2.7457486E38F);
            };
            GroundControl.HOME_POSITION p242 = CommunicationChannel.new_HOME_POSITION();
            PH.setPack(p242);
            p242.approach_z = (float)1.4698179E38F;
            p242.q_SET(new float[] {-2.9681201E38F, 2.6893597E38F, -1.5417413E38F, -2.3176507E38F}, 0) ;
            p242.time_usec_SET((ulong)9158414482931091853L, PH) ;
            p242.altitude = (int)1880478318;
            p242.approach_y = (float) -2.7457486E38F;
            p242.z = (float) -1.5039243E38F;
            p242.y = (float) -2.9055363E37F;
            p242.latitude = (int) -898137631;
            p242.approach_x = (float)8.043136E37F;
            p242.x = (float) -2.2984927E38F;
            p242.longitude = (int)396169021;
            CommunicationChannel.instance.send(p242);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_HOME_POSITIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.altitude == (int)102243782);
                Debug.Assert(pack.target_system == (byte)(byte)147);
                Debug.Assert(pack.z == (float) -3.197138E37F);
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)1113304972173902847L);
                Debug.Assert(pack.latitude == (int)111051381);
                Debug.Assert(pack.approach_x == (float) -1.0371227E38F);
                Debug.Assert(pack.longitude == (int)59485374);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-2.8741623E38F, -2.953025E37F, 2.6480748E38F, 2.3514095E38F}));
                Debug.Assert(pack.approach_z == (float) -2.0375366E38F);
                Debug.Assert(pack.x == (float) -2.4967892E38F);
                Debug.Assert(pack.y == (float)1.3612176E38F);
                Debug.Assert(pack.approach_y == (float) -3.0867154E38F);
            };
            GroundControl.SET_HOME_POSITION p243 = CommunicationChannel.new_SET_HOME_POSITION();
            PH.setPack(p243);
            p243.target_system = (byte)(byte)147;
            p243.longitude = (int)59485374;
            p243.approach_z = (float) -2.0375366E38F;
            p243.approach_y = (float) -3.0867154E38F;
            p243.approach_x = (float) -1.0371227E38F;
            p243.y = (float)1.3612176E38F;
            p243.x = (float) -2.4967892E38F;
            p243.q_SET(new float[] {-2.8741623E38F, -2.953025E37F, 2.6480748E38F, 2.3514095E38F}, 0) ;
            p243.time_usec_SET((ulong)1113304972173902847L, PH) ;
            p243.altitude = (int)102243782;
            p243.latitude = (int)111051381;
            p243.z = (float) -3.197138E37F;
            CommunicationChannel.instance.send(p243);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            ADV_TEST_CH.OnMESSAGE_INTERVALReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.message_id == (ushort)(ushort)28758);
                Debug.Assert(pack.interval_us == (int) -332472088);
            };
            GroundControl.MESSAGE_INTERVAL p244 = CommunicationChannel.new_MESSAGE_INTERVAL();
            PH.setPack(p244);
            p244.interval_us = (int) -332472088;
            p244.message_id = (ushort)(ushort)28758;
            CommunicationChannel.instance.send(p244);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnEXTENDED_SYS_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.landed_state == MAV_LANDED_STATE.MAV_LANDED_STATE_LANDING);
                Debug.Assert(pack.vtol_state == MAV_VTOL_STATE.MAV_VTOL_STATE_TRANSITION_TO_MC);
            };
            GroundControl.EXTENDED_SYS_STATE p245 = CommunicationChannel.new_EXTENDED_SYS_STATE();
            PH.setPack(p245);
            p245.landed_state = MAV_LANDED_STATE.MAV_LANDED_STATE_LANDING;
            p245.vtol_state = MAV_VTOL_STATE.MAV_VTOL_STATE_TRANSITION_TO_MC;
            CommunicationChannel.instance.send(p245);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnADSB_VEHICLEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.heading == (ushort)(ushort)2741);
                Debug.Assert(pack.lat == (int) -1830634009);
                Debug.Assert(pack.squawk == (ushort)(ushort)39119);
                Debug.Assert(pack.ver_velocity == (short)(short)28448);
                Debug.Assert(pack.altitude == (int) -146897142);
                Debug.Assert(pack.tslc == (byte)(byte)99);
                Debug.Assert(pack.ICAO_address == (uint)3652085209U);
                Debug.Assert(pack.callsign_LEN(ph) == 2);
                Debug.Assert(pack.callsign_TRY(ph).Equals("wp"));
                Debug.Assert(pack.flags == (ADSB_FLAGS.ADSB_FLAGS_SIMULATED |
                                            ADSB_FLAGS.ADSB_FLAGS_VALID_CALLSIGN |
                                            ADSB_FLAGS.ADSB_FLAGS_VALID_COORDS));
                Debug.Assert(pack.emitter_type == ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_PARACHUTE);
                Debug.Assert(pack.altitude_type == ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_GEOMETRIC);
                Debug.Assert(pack.hor_velocity == (ushort)(ushort)25314);
                Debug.Assert(pack.lon == (int)185834089);
            };
            GroundControl.ADSB_VEHICLE p246 = CommunicationChannel.new_ADSB_VEHICLE();
            PH.setPack(p246);
            p246.hor_velocity = (ushort)(ushort)25314;
            p246.tslc = (byte)(byte)99;
            p246.lon = (int)185834089;
            p246.emitter_type = ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_PARACHUTE;
            p246.squawk = (ushort)(ushort)39119;
            p246.altitude = (int) -146897142;
            p246.flags = (ADSB_FLAGS.ADSB_FLAGS_SIMULATED |
                          ADSB_FLAGS.ADSB_FLAGS_VALID_CALLSIGN |
                          ADSB_FLAGS.ADSB_FLAGS_VALID_COORDS);
            p246.lat = (int) -1830634009;
            p246.heading = (ushort)(ushort)2741;
            p246.altitude_type = ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_GEOMETRIC;
            p246.callsign_SET("wp", PH) ;
            p246.ICAO_address = (uint)3652085209U;
            p246.ver_velocity = (short)(short)28448;
            CommunicationChannel.instance.send(p246);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnCOLLISIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.altitude_minimum_delta == (float)2.0887586E38F);
                Debug.Assert(pack.horizontal_minimum_delta == (float)2.3006883E38F);
                Debug.Assert(pack.src_ == MAV_COLLISION_SRC.MAV_COLLISION_SRC_ADSB);
                Debug.Assert(pack.action == MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_ASCEND_OR_DESCEND);
                Debug.Assert(pack.time_to_minimum_delta == (float) -2.6438397E38F);
                Debug.Assert(pack.threat_level == (MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_LOW |
                                                   MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_NONE));
                Debug.Assert(pack.id == (uint)3641383114U);
            };
            GroundControl.COLLISION p247 = CommunicationChannel.new_COLLISION();
            PH.setPack(p247);
            p247.altitude_minimum_delta = (float)2.0887586E38F;
            p247.action = MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_ASCEND_OR_DESCEND;
            p247.time_to_minimum_delta = (float) -2.6438397E38F;
            p247.id = (uint)3641383114U;
            p247.src_ = MAV_COLLISION_SRC.MAV_COLLISION_SRC_ADSB;
            p247.horizontal_minimum_delta = (float)2.3006883E38F;
            p247.threat_level = (MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_LOW |
                                 MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_NONE);
            CommunicationChannel.instance.send(p247);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnV2_EXTENSIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_network == (byte)(byte)48);
                Debug.Assert(pack.message_type == (ushort)(ushort)55644);
                Debug.Assert(pack.target_component == (byte)(byte)255);
                Debug.Assert(pack.payload.SequenceEqual(new byte[] {(byte)175, (byte)176, (byte)175, (byte)253, (byte)30, (byte)193, (byte)49, (byte)47, (byte)200, (byte)126, (byte)164, (byte)20, (byte)164, (byte)246, (byte)20, (byte)206, (byte)195, (byte)9, (byte)167, (byte)145, (byte)13, (byte)109, (byte)124, (byte)159, (byte)89, (byte)88, (byte)128, (byte)20, (byte)252, (byte)125, (byte)205, (byte)192, (byte)75, (byte)14, (byte)130, (byte)115, (byte)210, (byte)222, (byte)249, (byte)245, (byte)193, (byte)245, (byte)163, (byte)44, (byte)142, (byte)44, (byte)83, (byte)216, (byte)99, (byte)184, (byte)135, (byte)146, (byte)42, (byte)60, (byte)12, (byte)93, (byte)119, (byte)202, (byte)212, (byte)157, (byte)96, (byte)81, (byte)78, (byte)177, (byte)219, (byte)38, (byte)77, (byte)3, (byte)51, (byte)205, (byte)65, (byte)227, (byte)188, (byte)69, (byte)96, (byte)214, (byte)110, (byte)232, (byte)174, (byte)41, (byte)188, (byte)175, (byte)31, (byte)84, (byte)82, (byte)68, (byte)118, (byte)211, (byte)51, (byte)36, (byte)66, (byte)220, (byte)213, (byte)35, (byte)102, (byte)128, (byte)65, (byte)124, (byte)245, (byte)90, (byte)19, (byte)15, (byte)4, (byte)40, (byte)228, (byte)142, (byte)249, (byte)101, (byte)226, (byte)146, (byte)176, (byte)161, (byte)191, (byte)226, (byte)136, (byte)234, (byte)249, (byte)186, (byte)176, (byte)197, (byte)6, (byte)114, (byte)136, (byte)176, (byte)231, (byte)79, (byte)33, (byte)38, (byte)178, (byte)239, (byte)83, (byte)60, (byte)28, (byte)65, (byte)131, (byte)29, (byte)45, (byte)16, (byte)233, (byte)168, (byte)58, (byte)217, (byte)178, (byte)90, (byte)239, (byte)111, (byte)223, (byte)90, (byte)15, (byte)141, (byte)168, (byte)125, (byte)59, (byte)238, (byte)210, (byte)22, (byte)17, (byte)85, (byte)229, (byte)61, (byte)74, (byte)105, (byte)129, (byte)161, (byte)63, (byte)217, (byte)227, (byte)38, (byte)192, (byte)160, (byte)63, (byte)161, (byte)77, (byte)193, (byte)145, (byte)97, (byte)147, (byte)59, (byte)173, (byte)84, (byte)151, (byte)60, (byte)232, (byte)86, (byte)61, (byte)160, (byte)40, (byte)134, (byte)221, (byte)115, (byte)98, (byte)46, (byte)67, (byte)96, (byte)62, (byte)142, (byte)79, (byte)242, (byte)54, (byte)17, (byte)66, (byte)156, (byte)36, (byte)78, (byte)121, (byte)94, (byte)105, (byte)123, (byte)247, (byte)85, (byte)191, (byte)146, (byte)248, (byte)166, (byte)199, (byte)147, (byte)248, (byte)130, (byte)96, (byte)43, (byte)189, (byte)63, (byte)188, (byte)216, (byte)17, (byte)28, (byte)191, (byte)155, (byte)45, (byte)61, (byte)225, (byte)206, (byte)113, (byte)103, (byte)224, (byte)242, (byte)255, (byte)50, (byte)169, (byte)37, (byte)244, (byte)68, (byte)11, (byte)254, (byte)48, (byte)232, (byte)77, (byte)15, (byte)255}));
                Debug.Assert(pack.target_system == (byte)(byte)106);
            };
            GroundControl.V2_EXTENSION p248 = CommunicationChannel.new_V2_EXTENSION();
            PH.setPack(p248);
            p248.message_type = (ushort)(ushort)55644;
            p248.target_component = (byte)(byte)255;
            p248.target_network = (byte)(byte)48;
            p248.target_system = (byte)(byte)106;
            p248.payload_SET(new byte[] {(byte)175, (byte)176, (byte)175, (byte)253, (byte)30, (byte)193, (byte)49, (byte)47, (byte)200, (byte)126, (byte)164, (byte)20, (byte)164, (byte)246, (byte)20, (byte)206, (byte)195, (byte)9, (byte)167, (byte)145, (byte)13, (byte)109, (byte)124, (byte)159, (byte)89, (byte)88, (byte)128, (byte)20, (byte)252, (byte)125, (byte)205, (byte)192, (byte)75, (byte)14, (byte)130, (byte)115, (byte)210, (byte)222, (byte)249, (byte)245, (byte)193, (byte)245, (byte)163, (byte)44, (byte)142, (byte)44, (byte)83, (byte)216, (byte)99, (byte)184, (byte)135, (byte)146, (byte)42, (byte)60, (byte)12, (byte)93, (byte)119, (byte)202, (byte)212, (byte)157, (byte)96, (byte)81, (byte)78, (byte)177, (byte)219, (byte)38, (byte)77, (byte)3, (byte)51, (byte)205, (byte)65, (byte)227, (byte)188, (byte)69, (byte)96, (byte)214, (byte)110, (byte)232, (byte)174, (byte)41, (byte)188, (byte)175, (byte)31, (byte)84, (byte)82, (byte)68, (byte)118, (byte)211, (byte)51, (byte)36, (byte)66, (byte)220, (byte)213, (byte)35, (byte)102, (byte)128, (byte)65, (byte)124, (byte)245, (byte)90, (byte)19, (byte)15, (byte)4, (byte)40, (byte)228, (byte)142, (byte)249, (byte)101, (byte)226, (byte)146, (byte)176, (byte)161, (byte)191, (byte)226, (byte)136, (byte)234, (byte)249, (byte)186, (byte)176, (byte)197, (byte)6, (byte)114, (byte)136, (byte)176, (byte)231, (byte)79, (byte)33, (byte)38, (byte)178, (byte)239, (byte)83, (byte)60, (byte)28, (byte)65, (byte)131, (byte)29, (byte)45, (byte)16, (byte)233, (byte)168, (byte)58, (byte)217, (byte)178, (byte)90, (byte)239, (byte)111, (byte)223, (byte)90, (byte)15, (byte)141, (byte)168, (byte)125, (byte)59, (byte)238, (byte)210, (byte)22, (byte)17, (byte)85, (byte)229, (byte)61, (byte)74, (byte)105, (byte)129, (byte)161, (byte)63, (byte)217, (byte)227, (byte)38, (byte)192, (byte)160, (byte)63, (byte)161, (byte)77, (byte)193, (byte)145, (byte)97, (byte)147, (byte)59, (byte)173, (byte)84, (byte)151, (byte)60, (byte)232, (byte)86, (byte)61, (byte)160, (byte)40, (byte)134, (byte)221, (byte)115, (byte)98, (byte)46, (byte)67, (byte)96, (byte)62, (byte)142, (byte)79, (byte)242, (byte)54, (byte)17, (byte)66, (byte)156, (byte)36, (byte)78, (byte)121, (byte)94, (byte)105, (byte)123, (byte)247, (byte)85, (byte)191, (byte)146, (byte)248, (byte)166, (byte)199, (byte)147, (byte)248, (byte)130, (byte)96, (byte)43, (byte)189, (byte)63, (byte)188, (byte)216, (byte)17, (byte)28, (byte)191, (byte)155, (byte)45, (byte)61, (byte)225, (byte)206, (byte)113, (byte)103, (byte)224, (byte)242, (byte)255, (byte)50, (byte)169, (byte)37, (byte)244, (byte)68, (byte)11, (byte)254, (byte)48, (byte)232, (byte)77, (byte)15, (byte)255}, 0) ;
            CommunicationChannel.instance.send(p248);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnMEMORY_VECTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.type == (byte)(byte)202);
                Debug.Assert(pack.value.SequenceEqual(new sbyte[] {(sbyte)112, (sbyte) - 109, (sbyte)118, (sbyte)37, (sbyte) - 69, (sbyte)28, (sbyte) - 123, (sbyte)22, (sbyte) - 104, (sbyte)15, (sbyte) - 57, (sbyte) - 45, (sbyte) - 119, (sbyte) - 94, (sbyte)79, (sbyte)48, (sbyte) - 86, (sbyte)97, (sbyte)55, (sbyte)13, (sbyte)35, (sbyte) - 31, (sbyte) - 105, (sbyte)77, (sbyte) - 114, (sbyte) - 29, (sbyte) - 34, (sbyte) - 41, (sbyte) - 103, (sbyte) - 2, (sbyte) - 109, (sbyte)29}));
                Debug.Assert(pack.address == (ushort)(ushort)63463);
                Debug.Assert(pack.ver == (byte)(byte)122);
            };
            GroundControl.MEMORY_VECT p249 = CommunicationChannel.new_MEMORY_VECT();
            PH.setPack(p249);
            p249.address = (ushort)(ushort)63463;
            p249.value_SET(new sbyte[] {(sbyte)112, (sbyte) - 109, (sbyte)118, (sbyte)37, (sbyte) - 69, (sbyte)28, (sbyte) - 123, (sbyte)22, (sbyte) - 104, (sbyte)15, (sbyte) - 57, (sbyte) - 45, (sbyte) - 119, (sbyte) - 94, (sbyte)79, (sbyte)48, (sbyte) - 86, (sbyte)97, (sbyte)55, (sbyte)13, (sbyte)35, (sbyte) - 31, (sbyte) - 105, (sbyte)77, (sbyte) - 114, (sbyte) - 29, (sbyte) - 34, (sbyte) - 41, (sbyte) - 103, (sbyte) - 2, (sbyte) - 109, (sbyte)29}, 0) ;
            p249.type = (byte)(byte)202;
            p249.ver = (byte)(byte)122;
            CommunicationChannel.instance.send(p249);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnDEBUG_VECTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)6808373014694874823L);
                Debug.Assert(pack.y == (float) -2.891354E38F);
                Debug.Assert(pack.name_LEN(ph) == 6);
                Debug.Assert(pack.name_TRY(ph).Equals("pnfpFo"));
                Debug.Assert(pack.x == (float) -2.5128514E38F);
                Debug.Assert(pack.z == (float)1.3380686E38F);
            };
            GroundControl.DEBUG_VECT p250 = CommunicationChannel.new_DEBUG_VECT();
            PH.setPack(p250);
            p250.name_SET("pnfpFo", PH) ;
            p250.x = (float) -2.5128514E38F;
            p250.time_usec = (ulong)6808373014694874823L;
            p250.y = (float) -2.891354E38F;
            p250.z = (float)1.3380686E38F;
            CommunicationChannel.instance.send(p250);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnNAMED_VALUE_FLOATReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)755732003U);
                Debug.Assert(pack.name_LEN(ph) == 2);
                Debug.Assert(pack.name_TRY(ph).Equals("rh"));
                Debug.Assert(pack.value == (float)2.9580487E38F);
            };
            GroundControl.NAMED_VALUE_FLOAT p251 = CommunicationChannel.new_NAMED_VALUE_FLOAT();
            PH.setPack(p251);
            p251.time_boot_ms = (uint)755732003U;
            p251.name_SET("rh", PH) ;
            p251.value = (float)2.9580487E38F;
            CommunicationChannel.instance.send(p251);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnNAMED_VALUE_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)2086027969U);
                Debug.Assert(pack.name_LEN(ph) == 5);
                Debug.Assert(pack.name_TRY(ph).Equals("zcTip"));
                Debug.Assert(pack.value == (int) -7983713);
            };
            GroundControl.NAMED_VALUE_INT p252 = CommunicationChannel.new_NAMED_VALUE_INT();
            PH.setPack(p252);
            p252.time_boot_ms = (uint)2086027969U;
            p252.value = (int) -7983713;
            p252.name_SET("zcTip", PH) ;
            CommunicationChannel.instance.send(p252);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSTATUSTEXTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.text_LEN(ph) == 39);
                Debug.Assert(pack.text_TRY(ph).Equals("bfzUpnBznkgpuJfmcybclqpgtmvspSotupvtaio"));
                Debug.Assert(pack.severity == MAV_SEVERITY.MAV_SEVERITY_ALERT);
            };
            GroundControl.STATUSTEXT p253 = CommunicationChannel.new_STATUSTEXT();
            PH.setPack(p253);
            p253.text_SET("bfzUpnBznkgpuJfmcybclqpgtmvspSotupvtaio", PH) ;
            p253.severity = MAV_SEVERITY.MAV_SEVERITY_ALERT;
            CommunicationChannel.instance.send(p253);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnDEBUGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ind == (byte)(byte)220);
                Debug.Assert(pack.time_boot_ms == (uint)2522486456U);
                Debug.Assert(pack.value == (float)1.4354015E38F);
            };
            GroundControl.DEBUG p254 = CommunicationChannel.new_DEBUG();
            PH.setPack(p254);
            p254.value = (float)1.4354015E38F;
            p254.ind = (byte)(byte)220;
            p254.time_boot_ms = (uint)2522486456U;
            CommunicationChannel.instance.send(p254);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSETUP_SIGNINGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)174);
                Debug.Assert(pack.secret_key.SequenceEqual(new byte[] {(byte)159, (byte)36, (byte)69, (byte)213, (byte)14, (byte)39, (byte)171, (byte)180, (byte)184, (byte)158, (byte)8, (byte)63, (byte)197, (byte)114, (byte)140, (byte)193, (byte)14, (byte)163, (byte)101, (byte)42, (byte)67, (byte)218, (byte)26, (byte)122, (byte)210, (byte)244, (byte)63, (byte)226, (byte)113, (byte)16, (byte)94, (byte)229}));
                Debug.Assert(pack.target_system == (byte)(byte)17);
                Debug.Assert(pack.initial_timestamp == (ulong)7063289895825560668L);
            };
            GroundControl.SETUP_SIGNING p256 = CommunicationChannel.new_SETUP_SIGNING();
            PH.setPack(p256);
            p256.secret_key_SET(new byte[] {(byte)159, (byte)36, (byte)69, (byte)213, (byte)14, (byte)39, (byte)171, (byte)180, (byte)184, (byte)158, (byte)8, (byte)63, (byte)197, (byte)114, (byte)140, (byte)193, (byte)14, (byte)163, (byte)101, (byte)42, (byte)67, (byte)218, (byte)26, (byte)122, (byte)210, (byte)244, (byte)63, (byte)226, (byte)113, (byte)16, (byte)94, (byte)229}, 0) ;
            p256.target_system = (byte)(byte)17;
            p256.target_component = (byte)(byte)174;
            p256.initial_timestamp = (ulong)7063289895825560668L;
            CommunicationChannel.instance.send(p256);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnBUTTON_CHANGEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.last_change_ms == (uint)2918748178U);
                Debug.Assert(pack.time_boot_ms == (uint)830104130U);
                Debug.Assert(pack.state == (byte)(byte)251);
            };
            GroundControl.BUTTON_CHANGE p257 = CommunicationChannel.new_BUTTON_CHANGE();
            PH.setPack(p257);
            p257.last_change_ms = (uint)2918748178U;
            p257.time_boot_ms = (uint)830104130U;
            p257.state = (byte)(byte)251;
            CommunicationChannel.instance.send(p257);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnPLAY_TUNEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)106);
                Debug.Assert(pack.target_system == (byte)(byte)204);
                Debug.Assert(pack.tune_LEN(ph) == 8);
                Debug.Assert(pack.tune_TRY(ph).Equals("jpzfltnu"));
            };
            GroundControl.PLAY_TUNE p258 = CommunicationChannel.new_PLAY_TUNE();
            PH.setPack(p258);
            p258.target_component = (byte)(byte)106;
            p258.target_system = (byte)(byte)204;
            p258.tune_SET("jpzfltnu", PH) ;
            CommunicationChannel.instance.send(p258);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnCAMERA_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.resolution_h == (ushort)(ushort)4120);
                Debug.Assert(pack.cam_definition_version == (ushort)(ushort)47458);
                Debug.Assert(pack.resolution_v == (ushort)(ushort)15230);
                Debug.Assert(pack.sensor_size_v == (float) -3.1993655E38F);
                Debug.Assert(pack.focal_length == (float)1.2882985E38F);
                Debug.Assert(pack.time_boot_ms == (uint)2568891040U);
                Debug.Assert(pack.firmware_version == (uint)4094188590U);
                Debug.Assert(pack.vendor_name.SequenceEqual(new byte[] {(byte)181, (byte)25, (byte)169, (byte)140, (byte)112, (byte)218, (byte)96, (byte)199, (byte)107, (byte)232, (byte)211, (byte)166, (byte)170, (byte)78, (byte)138, (byte)154, (byte)101, (byte)142, (byte)149, (byte)35, (byte)48, (byte)130, (byte)251, (byte)20, (byte)195, (byte)199, (byte)13, (byte)31, (byte)52, (byte)39, (byte)147, (byte)176}));
                Debug.Assert(pack.flags == (CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE |
                                            CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_HAS_MODES |
                                            CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE));
                Debug.Assert(pack.sensor_size_h == (float)3.2072096E38F);
                Debug.Assert(pack.lens_id == (byte)(byte)10);
                Debug.Assert(pack.model_name.SequenceEqual(new byte[] {(byte)237, (byte)157, (byte)99, (byte)52, (byte)172, (byte)85, (byte)121, (byte)218, (byte)233, (byte)150, (byte)173, (byte)120, (byte)11, (byte)239, (byte)105, (byte)182, (byte)177, (byte)30, (byte)126, (byte)141, (byte)187, (byte)236, (byte)127, (byte)33, (byte)97, (byte)164, (byte)179, (byte)63, (byte)249, (byte)225, (byte)26, (byte)185}));
                Debug.Assert(pack.cam_definition_uri_LEN(ph) == 45);
                Debug.Assert(pack.cam_definition_uri_TRY(ph).Equals("ibGagtyhflvufuwnmozrZgfdugzshwtsurpkhnrFcjeol"));
            };
            GroundControl.CAMERA_INFORMATION p259 = CommunicationChannel.new_CAMERA_INFORMATION();
            PH.setPack(p259);
            p259.cam_definition_uri_SET("ibGagtyhflvufuwnmozrZgfdugzshwtsurpkhnrFcjeol", PH) ;
            p259.cam_definition_version = (ushort)(ushort)47458;
            p259.vendor_name_SET(new byte[] {(byte)181, (byte)25, (byte)169, (byte)140, (byte)112, (byte)218, (byte)96, (byte)199, (byte)107, (byte)232, (byte)211, (byte)166, (byte)170, (byte)78, (byte)138, (byte)154, (byte)101, (byte)142, (byte)149, (byte)35, (byte)48, (byte)130, (byte)251, (byte)20, (byte)195, (byte)199, (byte)13, (byte)31, (byte)52, (byte)39, (byte)147, (byte)176}, 0) ;
            p259.time_boot_ms = (uint)2568891040U;
            p259.firmware_version = (uint)4094188590U;
            p259.model_name_SET(new byte[] {(byte)237, (byte)157, (byte)99, (byte)52, (byte)172, (byte)85, (byte)121, (byte)218, (byte)233, (byte)150, (byte)173, (byte)120, (byte)11, (byte)239, (byte)105, (byte)182, (byte)177, (byte)30, (byte)126, (byte)141, (byte)187, (byte)236, (byte)127, (byte)33, (byte)97, (byte)164, (byte)179, (byte)63, (byte)249, (byte)225, (byte)26, (byte)185}, 0) ;
            p259.resolution_v = (ushort)(ushort)15230;
            p259.lens_id = (byte)(byte)10;
            p259.flags = (CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE |
                          CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_HAS_MODES |
                          CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE);
            p259.sensor_size_h = (float)3.2072096E38F;
            p259.sensor_size_v = (float) -3.1993655E38F;
            p259.resolution_h = (ushort)(ushort)4120;
            p259.focal_length = (float)1.2882985E38F;
            CommunicationChannel.instance.send(p259);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnCAMERA_SETTINGSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)2010495264U);
                Debug.Assert(pack.mode_id == (CAMERA_MODE.CAMERA_MODE_IMAGE_SURVEY));
            };
            GroundControl.CAMERA_SETTINGS p260 = CommunicationChannel.new_CAMERA_SETTINGS();
            PH.setPack(p260);
            p260.mode_id = (CAMERA_MODE.CAMERA_MODE_IMAGE_SURVEY);
            p260.time_boot_ms = (uint)2010495264U;
            CommunicationChannel.instance.send(p260);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSTORAGE_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.storage_count == (byte)(byte)162);
                Debug.Assert(pack.write_speed == (float) -5.246738E37F);
                Debug.Assert(pack.time_boot_ms == (uint)3527197047U);
                Debug.Assert(pack.available_capacity == (float)1.5417212E38F);
                Debug.Assert(pack.used_capacity == (float) -5.590889E37F);
                Debug.Assert(pack.storage_id == (byte)(byte)53);
                Debug.Assert(pack.total_capacity == (float) -1.0270012E38F);
                Debug.Assert(pack.read_speed == (float)1.1640291E38F);
                Debug.Assert(pack.status == (byte)(byte)122);
            };
            GroundControl.STORAGE_INFORMATION p261 = CommunicationChannel.new_STORAGE_INFORMATION();
            PH.setPack(p261);
            p261.available_capacity = (float)1.5417212E38F;
            p261.time_boot_ms = (uint)3527197047U;
            p261.total_capacity = (float) -1.0270012E38F;
            p261.status = (byte)(byte)122;
            p261.storage_count = (byte)(byte)162;
            p261.storage_id = (byte)(byte)53;
            p261.write_speed = (float) -5.246738E37F;
            p261.used_capacity = (float) -5.590889E37F;
            p261.read_speed = (float)1.1640291E38F;
            CommunicationChannel.instance.send(p261);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnCAMERA_CAPTURE_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)2398992230U);
                Debug.Assert(pack.recording_time_ms == (uint)1666229696U);
                Debug.Assert(pack.image_status == (byte)(byte)174);
                Debug.Assert(pack.video_status == (byte)(byte)112);
                Debug.Assert(pack.image_interval == (float)1.4074223E38F);
                Debug.Assert(pack.available_capacity == (float)1.5231578E38F);
            };
            GroundControl.CAMERA_CAPTURE_STATUS p262 = CommunicationChannel.new_CAMERA_CAPTURE_STATUS();
            PH.setPack(p262);
            p262.recording_time_ms = (uint)1666229696U;
            p262.video_status = (byte)(byte)112;
            p262.available_capacity = (float)1.5231578E38F;
            p262.image_interval = (float)1.4074223E38F;
            p262.image_status = (byte)(byte)174;
            p262.time_boot_ms = (uint)2398992230U;
            CommunicationChannel.instance.send(p262);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnCAMERA_IMAGE_CAPTUREDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.image_index == (int)420575286);
                Debug.Assert(pack.lon == (int)231764658);
                Debug.Assert(pack.camera_id == (byte)(byte)215);
                Debug.Assert(pack.time_boot_ms == (uint)382219259U);
                Debug.Assert(pack.capture_result == (sbyte)(sbyte) - 96);
                Debug.Assert(pack.alt == (int)1584021403);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-1.2723177E38F, 3.317273E37F, -8.2838153E37F, -3.3179844E38F}));
                Debug.Assert(pack.time_utc == (ulong)130819496988195417L);
                Debug.Assert(pack.relative_alt == (int)306791445);
                Debug.Assert(pack.file_url_LEN(ph) == 115);
                Debug.Assert(pack.file_url_TRY(ph).Equals("mRrmpdhTqjxpcfyegwgxskrgldbfHkolpEdovuycujyngVfcfuQKvsyqggykbscvWzhocrfAyYtkrGjhFLPizxdnvbxhmqvwKejwjeyvheinnmguaiu"));
                Debug.Assert(pack.lat == (int) -1260277308);
            };
            GroundControl.CAMERA_IMAGE_CAPTURED p263 = CommunicationChannel.new_CAMERA_IMAGE_CAPTURED();
            PH.setPack(p263);
            p263.image_index = (int)420575286;
            p263.lat = (int) -1260277308;
            p263.alt = (int)1584021403;
            p263.q_SET(new float[] {-1.2723177E38F, 3.317273E37F, -8.2838153E37F, -3.3179844E38F}, 0) ;
            p263.lon = (int)231764658;
            p263.time_boot_ms = (uint)382219259U;
            p263.time_utc = (ulong)130819496988195417L;
            p263.camera_id = (byte)(byte)215;
            p263.relative_alt = (int)306791445;
            p263.capture_result = (sbyte)(sbyte) - 96;
            p263.file_url_SET("mRrmpdhTqjxpcfyegwgxskrgldbfHkolpEdovuycujyngVfcfuQKvsyqggykbscvWzhocrfAyYtkrGjhFLPizxdnvbxhmqvwKejwjeyvheinnmguaiu", PH) ;
            CommunicationChannel.instance.send(p263);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnFLIGHT_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.flight_uuid == (ulong)5226313679661778851L);
                Debug.Assert(pack.arming_time_utc == (ulong)7021555058803440604L);
                Debug.Assert(pack.takeoff_time_utc == (ulong)8799351094751644679L);
                Debug.Assert(pack.time_boot_ms == (uint)747591481U);
            };
            GroundControl.FLIGHT_INFORMATION p264 = CommunicationChannel.new_FLIGHT_INFORMATION();
            PH.setPack(p264);
            p264.arming_time_utc = (ulong)7021555058803440604L;
            p264.time_boot_ms = (uint)747591481U;
            p264.takeoff_time_utc = (ulong)8799351094751644679L;
            p264.flight_uuid = (ulong)5226313679661778851L;
            CommunicationChannel.instance.send(p264);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnMOUNT_ORIENTATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.yaw == (float)1.8112843E38F);
                Debug.Assert(pack.roll == (float)1.5138388E38F);
                Debug.Assert(pack.time_boot_ms == (uint)511029698U);
                Debug.Assert(pack.pitch == (float) -1.3705834E38F);
            };
            GroundControl.MOUNT_ORIENTATION p265 = CommunicationChannel.new_MOUNT_ORIENTATION();
            PH.setPack(p265);
            p265.roll = (float)1.5138388E38F;
            p265.yaw = (float)1.8112843E38F;
            p265.time_boot_ms = (uint)511029698U;
            p265.pitch = (float) -1.3705834E38F;
            CommunicationChannel.instance.send(p265);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnLOGGING_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)221, (byte)62, (byte)17, (byte)202, (byte)217, (byte)183, (byte)216, (byte)101, (byte)30, (byte)116, (byte)143, (byte)100, (byte)142, (byte)222, (byte)67, (byte)96, (byte)41, (byte)90, (byte)175, (byte)152, (byte)65, (byte)150, (byte)175, (byte)240, (byte)144, (byte)51, (byte)2, (byte)249, (byte)2, (byte)3, (byte)45, (byte)27, (byte)82, (byte)31, (byte)73, (byte)77, (byte)152, (byte)46, (byte)111, (byte)209, (byte)195, (byte)94, (byte)223, (byte)62, (byte)52, (byte)3, (byte)123, (byte)120, (byte)92, (byte)151, (byte)218, (byte)90, (byte)49, (byte)118, (byte)71, (byte)48, (byte)240, (byte)163, (byte)223, (byte)43, (byte)51, (byte)200, (byte)189, (byte)159, (byte)216, (byte)147, (byte)142, (byte)37, (byte)53, (byte)129, (byte)246, (byte)86, (byte)94, (byte)6, (byte)157, (byte)233, (byte)115, (byte)179, (byte)101, (byte)250, (byte)172, (byte)75, (byte)244, (byte)189, (byte)254, (byte)208, (byte)183, (byte)110, (byte)132, (byte)111, (byte)184, (byte)70, (byte)78, (byte)34, (byte)55, (byte)145, (byte)130, (byte)61, (byte)107, (byte)25, (byte)69, (byte)53, (byte)198, (byte)236, (byte)92, (byte)193, (byte)19, (byte)36, (byte)168, (byte)88, (byte)3, (byte)85, (byte)182, (byte)22, (byte)78, (byte)192, (byte)137, (byte)185, (byte)92, (byte)66, (byte)225, (byte)105, (byte)33, (byte)96, (byte)157, (byte)70, (byte)1, (byte)163, (byte)146, (byte)152, (byte)10, (byte)102, (byte)13, (byte)169, (byte)197, (byte)79, (byte)149, (byte)204, (byte)18, (byte)65, (byte)251, (byte)215, (byte)77, (byte)194, (byte)238, (byte)60, (byte)168, (byte)42, (byte)132, (byte)132, (byte)237, (byte)213, (byte)168, (byte)206, (byte)54, (byte)76, (byte)93, (byte)41, (byte)149, (byte)72, (byte)240, (byte)103, (byte)252, (byte)160, (byte)214, (byte)255, (byte)65, (byte)9, (byte)148, (byte)169, (byte)138, (byte)2, (byte)252, (byte)35, (byte)104, (byte)164, (byte)20, (byte)166, (byte)67, (byte)1, (byte)122, (byte)175, (byte)46, (byte)0, (byte)90, (byte)250, (byte)231, (byte)100, (byte)220, (byte)175, (byte)12, (byte)49, (byte)4, (byte)13, (byte)179, (byte)22, (byte)48, (byte)127, (byte)87, (byte)239, (byte)101, (byte)198, (byte)75, (byte)249, (byte)55, (byte)89, (byte)159, (byte)123, (byte)146, (byte)242, (byte)249, (byte)140, (byte)245, (byte)29, (byte)133, (byte)181, (byte)149, (byte)170, (byte)135, (byte)18, (byte)19, (byte)158, (byte)211, (byte)97, (byte)151, (byte)66, (byte)98, (byte)129, (byte)49, (byte)164, (byte)178, (byte)206, (byte)8, (byte)101, (byte)162, (byte)233, (byte)111, (byte)228, (byte)44, (byte)32, (byte)246, (byte)39, (byte)18, (byte)191, (byte)167, (byte)152, (byte)68, (byte)42, (byte)109}));
                Debug.Assert(pack.target_component == (byte)(byte)156);
                Debug.Assert(pack.target_system == (byte)(byte)241);
                Debug.Assert(pack.first_message_offset == (byte)(byte)99);
                Debug.Assert(pack.length == (byte)(byte)245);
                Debug.Assert(pack.sequence == (ushort)(ushort)58354);
            };
            GroundControl.LOGGING_DATA p266 = CommunicationChannel.new_LOGGING_DATA();
            PH.setPack(p266);
            p266.target_system = (byte)(byte)241;
            p266.first_message_offset = (byte)(byte)99;
            p266.length = (byte)(byte)245;
            p266.sequence = (ushort)(ushort)58354;
            p266.data__SET(new byte[] {(byte)221, (byte)62, (byte)17, (byte)202, (byte)217, (byte)183, (byte)216, (byte)101, (byte)30, (byte)116, (byte)143, (byte)100, (byte)142, (byte)222, (byte)67, (byte)96, (byte)41, (byte)90, (byte)175, (byte)152, (byte)65, (byte)150, (byte)175, (byte)240, (byte)144, (byte)51, (byte)2, (byte)249, (byte)2, (byte)3, (byte)45, (byte)27, (byte)82, (byte)31, (byte)73, (byte)77, (byte)152, (byte)46, (byte)111, (byte)209, (byte)195, (byte)94, (byte)223, (byte)62, (byte)52, (byte)3, (byte)123, (byte)120, (byte)92, (byte)151, (byte)218, (byte)90, (byte)49, (byte)118, (byte)71, (byte)48, (byte)240, (byte)163, (byte)223, (byte)43, (byte)51, (byte)200, (byte)189, (byte)159, (byte)216, (byte)147, (byte)142, (byte)37, (byte)53, (byte)129, (byte)246, (byte)86, (byte)94, (byte)6, (byte)157, (byte)233, (byte)115, (byte)179, (byte)101, (byte)250, (byte)172, (byte)75, (byte)244, (byte)189, (byte)254, (byte)208, (byte)183, (byte)110, (byte)132, (byte)111, (byte)184, (byte)70, (byte)78, (byte)34, (byte)55, (byte)145, (byte)130, (byte)61, (byte)107, (byte)25, (byte)69, (byte)53, (byte)198, (byte)236, (byte)92, (byte)193, (byte)19, (byte)36, (byte)168, (byte)88, (byte)3, (byte)85, (byte)182, (byte)22, (byte)78, (byte)192, (byte)137, (byte)185, (byte)92, (byte)66, (byte)225, (byte)105, (byte)33, (byte)96, (byte)157, (byte)70, (byte)1, (byte)163, (byte)146, (byte)152, (byte)10, (byte)102, (byte)13, (byte)169, (byte)197, (byte)79, (byte)149, (byte)204, (byte)18, (byte)65, (byte)251, (byte)215, (byte)77, (byte)194, (byte)238, (byte)60, (byte)168, (byte)42, (byte)132, (byte)132, (byte)237, (byte)213, (byte)168, (byte)206, (byte)54, (byte)76, (byte)93, (byte)41, (byte)149, (byte)72, (byte)240, (byte)103, (byte)252, (byte)160, (byte)214, (byte)255, (byte)65, (byte)9, (byte)148, (byte)169, (byte)138, (byte)2, (byte)252, (byte)35, (byte)104, (byte)164, (byte)20, (byte)166, (byte)67, (byte)1, (byte)122, (byte)175, (byte)46, (byte)0, (byte)90, (byte)250, (byte)231, (byte)100, (byte)220, (byte)175, (byte)12, (byte)49, (byte)4, (byte)13, (byte)179, (byte)22, (byte)48, (byte)127, (byte)87, (byte)239, (byte)101, (byte)198, (byte)75, (byte)249, (byte)55, (byte)89, (byte)159, (byte)123, (byte)146, (byte)242, (byte)249, (byte)140, (byte)245, (byte)29, (byte)133, (byte)181, (byte)149, (byte)170, (byte)135, (byte)18, (byte)19, (byte)158, (byte)211, (byte)97, (byte)151, (byte)66, (byte)98, (byte)129, (byte)49, (byte)164, (byte)178, (byte)206, (byte)8, (byte)101, (byte)162, (byte)233, (byte)111, (byte)228, (byte)44, (byte)32, (byte)246, (byte)39, (byte)18, (byte)191, (byte)167, (byte)152, (byte)68, (byte)42, (byte)109}, 0) ;
            p266.target_component = (byte)(byte)156;
            CommunicationChannel.instance.send(p266);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnLOGGING_DATA_ACKEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)144, (byte)37, (byte)86, (byte)158, (byte)141, (byte)112, (byte)246, (byte)96, (byte)117, (byte)198, (byte)119, (byte)161, (byte)11, (byte)54, (byte)45, (byte)120, (byte)64, (byte)230, (byte)220, (byte)219, (byte)53, (byte)19, (byte)203, (byte)119, (byte)197, (byte)77, (byte)49, (byte)187, (byte)230, (byte)133, (byte)185, (byte)70, (byte)93, (byte)37, (byte)239, (byte)229, (byte)64, (byte)20, (byte)178, (byte)175, (byte)10, (byte)92, (byte)189, (byte)202, (byte)131, (byte)210, (byte)49, (byte)47, (byte)200, (byte)170, (byte)253, (byte)147, (byte)105, (byte)208, (byte)209, (byte)4, (byte)57, (byte)197, (byte)68, (byte)210, (byte)223, (byte)215, (byte)53, (byte)145, (byte)34, (byte)221, (byte)198, (byte)181, (byte)106, (byte)74, (byte)50, (byte)231, (byte)53, (byte)251, (byte)211, (byte)111, (byte)22, (byte)54, (byte)150, (byte)225, (byte)123, (byte)46, (byte)242, (byte)66, (byte)210, (byte)33, (byte)113, (byte)224, (byte)122, (byte)178, (byte)60, (byte)219, (byte)3, (byte)157, (byte)126, (byte)1, (byte)126, (byte)187, (byte)17, (byte)178, (byte)168, (byte)135, (byte)143, (byte)70, (byte)105, (byte)104, (byte)9, (byte)121, (byte)243, (byte)229, (byte)101, (byte)148, (byte)71, (byte)165, (byte)66, (byte)40, (byte)206, (byte)231, (byte)201, (byte)227, (byte)94, (byte)134, (byte)213, (byte)89, (byte)108, (byte)148, (byte)153, (byte)93, (byte)213, (byte)88, (byte)35, (byte)134, (byte)127, (byte)173, (byte)177, (byte)151, (byte)252, (byte)34, (byte)149, (byte)192, (byte)99, (byte)213, (byte)247, (byte)234, (byte)162, (byte)94, (byte)107, (byte)18, (byte)16, (byte)87, (byte)119, (byte)58, (byte)72, (byte)248, (byte)10, (byte)194, (byte)115, (byte)31, (byte)151, (byte)36, (byte)97, (byte)190, (byte)214, (byte)198, (byte)129, (byte)41, (byte)115, (byte)136, (byte)169, (byte)82, (byte)173, (byte)216, (byte)250, (byte)176, (byte)91, (byte)30, (byte)94, (byte)113, (byte)215, (byte)21, (byte)246, (byte)146, (byte)95, (byte)253, (byte)103, (byte)40, (byte)128, (byte)151, (byte)223, (byte)202, (byte)3, (byte)8, (byte)168, (byte)79, (byte)204, (byte)71, (byte)134, (byte)173, (byte)57, (byte)77, (byte)82, (byte)112, (byte)243, (byte)216, (byte)59, (byte)67, (byte)61, (byte)146, (byte)205, (byte)217, (byte)33, (byte)90, (byte)113, (byte)220, (byte)99, (byte)74, (byte)132, (byte)255, (byte)160, (byte)166, (byte)247, (byte)244, (byte)36, (byte)6, (byte)5, (byte)58, (byte)220, (byte)77, (byte)180, (byte)52, (byte)231, (byte)254, (byte)244, (byte)224, (byte)99, (byte)197, (byte)185, (byte)87, (byte)195, (byte)16, (byte)231, (byte)115, (byte)224, (byte)137, (byte)14, (byte)188, (byte)222, (byte)107, (byte)167}));
                Debug.Assert(pack.target_system == (byte)(byte)83);
                Debug.Assert(pack.length == (byte)(byte)193);
                Debug.Assert(pack.sequence == (ushort)(ushort)35720);
                Debug.Assert(pack.first_message_offset == (byte)(byte)107);
                Debug.Assert(pack.target_component == (byte)(byte)187);
            };
            GroundControl.LOGGING_DATA_ACKED p267 = CommunicationChannel.new_LOGGING_DATA_ACKED();
            PH.setPack(p267);
            p267.sequence = (ushort)(ushort)35720;
            p267.data__SET(new byte[] {(byte)144, (byte)37, (byte)86, (byte)158, (byte)141, (byte)112, (byte)246, (byte)96, (byte)117, (byte)198, (byte)119, (byte)161, (byte)11, (byte)54, (byte)45, (byte)120, (byte)64, (byte)230, (byte)220, (byte)219, (byte)53, (byte)19, (byte)203, (byte)119, (byte)197, (byte)77, (byte)49, (byte)187, (byte)230, (byte)133, (byte)185, (byte)70, (byte)93, (byte)37, (byte)239, (byte)229, (byte)64, (byte)20, (byte)178, (byte)175, (byte)10, (byte)92, (byte)189, (byte)202, (byte)131, (byte)210, (byte)49, (byte)47, (byte)200, (byte)170, (byte)253, (byte)147, (byte)105, (byte)208, (byte)209, (byte)4, (byte)57, (byte)197, (byte)68, (byte)210, (byte)223, (byte)215, (byte)53, (byte)145, (byte)34, (byte)221, (byte)198, (byte)181, (byte)106, (byte)74, (byte)50, (byte)231, (byte)53, (byte)251, (byte)211, (byte)111, (byte)22, (byte)54, (byte)150, (byte)225, (byte)123, (byte)46, (byte)242, (byte)66, (byte)210, (byte)33, (byte)113, (byte)224, (byte)122, (byte)178, (byte)60, (byte)219, (byte)3, (byte)157, (byte)126, (byte)1, (byte)126, (byte)187, (byte)17, (byte)178, (byte)168, (byte)135, (byte)143, (byte)70, (byte)105, (byte)104, (byte)9, (byte)121, (byte)243, (byte)229, (byte)101, (byte)148, (byte)71, (byte)165, (byte)66, (byte)40, (byte)206, (byte)231, (byte)201, (byte)227, (byte)94, (byte)134, (byte)213, (byte)89, (byte)108, (byte)148, (byte)153, (byte)93, (byte)213, (byte)88, (byte)35, (byte)134, (byte)127, (byte)173, (byte)177, (byte)151, (byte)252, (byte)34, (byte)149, (byte)192, (byte)99, (byte)213, (byte)247, (byte)234, (byte)162, (byte)94, (byte)107, (byte)18, (byte)16, (byte)87, (byte)119, (byte)58, (byte)72, (byte)248, (byte)10, (byte)194, (byte)115, (byte)31, (byte)151, (byte)36, (byte)97, (byte)190, (byte)214, (byte)198, (byte)129, (byte)41, (byte)115, (byte)136, (byte)169, (byte)82, (byte)173, (byte)216, (byte)250, (byte)176, (byte)91, (byte)30, (byte)94, (byte)113, (byte)215, (byte)21, (byte)246, (byte)146, (byte)95, (byte)253, (byte)103, (byte)40, (byte)128, (byte)151, (byte)223, (byte)202, (byte)3, (byte)8, (byte)168, (byte)79, (byte)204, (byte)71, (byte)134, (byte)173, (byte)57, (byte)77, (byte)82, (byte)112, (byte)243, (byte)216, (byte)59, (byte)67, (byte)61, (byte)146, (byte)205, (byte)217, (byte)33, (byte)90, (byte)113, (byte)220, (byte)99, (byte)74, (byte)132, (byte)255, (byte)160, (byte)166, (byte)247, (byte)244, (byte)36, (byte)6, (byte)5, (byte)58, (byte)220, (byte)77, (byte)180, (byte)52, (byte)231, (byte)254, (byte)244, (byte)224, (byte)99, (byte)197, (byte)185, (byte)87, (byte)195, (byte)16, (byte)231, (byte)115, (byte)224, (byte)137, (byte)14, (byte)188, (byte)222, (byte)107, (byte)167}, 0) ;
            p267.target_system = (byte)(byte)83;
            p267.target_component = (byte)(byte)187;
            p267.first_message_offset = (byte)(byte)107;
            p267.length = (byte)(byte)193;
            CommunicationChannel.instance.send(p267);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnLOGGING_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)17);
                Debug.Assert(pack.sequence == (ushort)(ushort)17446);
                Debug.Assert(pack.target_system == (byte)(byte)158);
            };
            GroundControl.LOGGING_ACK p268 = CommunicationChannel.new_LOGGING_ACK();
            PH.setPack(p268);
            p268.sequence = (ushort)(ushort)17446;
            p268.target_system = (byte)(byte)158;
            p268.target_component = (byte)(byte)17;
            CommunicationChannel.instance.send(p268);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnVIDEO_STREAM_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.uri_LEN(ph) == 140);
                Debug.Assert(pack.uri_TRY(ph).Equals("vddmbdRtvxnuafrAvvpuxnmRbbJncrcieaptopmlrvgaVkjyogqrotviegcgMnkvbjveYefCBikyzLOJonnNijOPqmqaznflsljaedjemaeWyrghoPauuszrufyajmrezhewdPfcpjic"));
                Debug.Assert(pack.rotation == (ushort)(ushort)51585);
                Debug.Assert(pack.bitrate == (uint)3000498456U);
                Debug.Assert(pack.resolution_v == (ushort)(ushort)17090);
                Debug.Assert(pack.framerate == (float) -4.0165268E37F);
                Debug.Assert(pack.resolution_h == (ushort)(ushort)27973);
                Debug.Assert(pack.status == (byte)(byte)77);
                Debug.Assert(pack.camera_id == (byte)(byte)18);
            };
            GroundControl.VIDEO_STREAM_INFORMATION p269 = CommunicationChannel.new_VIDEO_STREAM_INFORMATION();
            PH.setPack(p269);
            p269.rotation = (ushort)(ushort)51585;
            p269.framerate = (float) -4.0165268E37F;
            p269.status = (byte)(byte)77;
            p269.uri_SET("vddmbdRtvxnuafrAvvpuxnmRbbJncrcieaptopmlrvgaVkjyogqrotviegcgMnkvbjveYefCBikyzLOJonnNijOPqmqaznflsljaedjemaeWyrghoPauuszrufyajmrezhewdPfcpjic", PH) ;
            p269.bitrate = (uint)3000498456U;
            p269.resolution_h = (ushort)(ushort)27973;
            p269.camera_id = (byte)(byte)18;
            p269.resolution_v = (ushort)(ushort)17090;
            CommunicationChannel.instance.send(p269);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSET_VIDEO_STREAM_SETTINGSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.resolution_v == (ushort)(ushort)1344);
                Debug.Assert(pack.bitrate == (uint)3792892050U);
                Debug.Assert(pack.framerate == (float)1.6897336E38F);
                Debug.Assert(pack.target_component == (byte)(byte)143);
                Debug.Assert(pack.resolution_h == (ushort)(ushort)5374);
                Debug.Assert(pack.uri_LEN(ph) == 142);
                Debug.Assert(pack.uri_TRY(ph).Equals("ttusckvoZzubgjvomifgEqwtnuzyuqtskvjetgGiEpzHykcxpIzfqhthuwdCsuDsyvovjjhsnCpjzemtoxeMcAmRlkyMcpktAcupJovszpmrxspbJeartmwihqmsfaezwtnkfesxclsxjo"));
                Debug.Assert(pack.target_system == (byte)(byte)48);
                Debug.Assert(pack.rotation == (ushort)(ushort)16228);
                Debug.Assert(pack.camera_id == (byte)(byte)123);
            };
            GroundControl.SET_VIDEO_STREAM_SETTINGS p270 = CommunicationChannel.new_SET_VIDEO_STREAM_SETTINGS();
            PH.setPack(p270);
            p270.target_system = (byte)(byte)48;
            p270.uri_SET("ttusckvoZzubgjvomifgEqwtnuzyuqtskvjetgGiEpzHykcxpIzfqhthuwdCsuDsyvovjjhsnCpjzemtoxeMcAmRlkyMcpktAcupJovszpmrxspbJeartmwihqmsfaezwtnkfesxclsxjo", PH) ;
            p270.camera_id = (byte)(byte)123;
            p270.bitrate = (uint)3792892050U;
            p270.resolution_h = (ushort)(ushort)5374;
            p270.framerate = (float)1.6897336E38F;
            p270.target_component = (byte)(byte)143;
            p270.rotation = (ushort)(ushort)16228;
            p270.resolution_v = (ushort)(ushort)1344;
            CommunicationChannel.instance.send(p270);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnWIFI_CONFIG_APReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.password_LEN(ph) == 38);
                Debug.Assert(pack.password_TRY(ph).Equals("lqjdteckDdyycWhaBCkdOapuuoamjkYqqibJrD"));
                Debug.Assert(pack.ssid_LEN(ph) == 30);
                Debug.Assert(pack.ssid_TRY(ph).Equals("rFybxkpTexalushrbnagqEbiwixdbo"));
            };
            GroundControl.WIFI_CONFIG_AP p299 = CommunicationChannel.new_WIFI_CONFIG_AP();
            PH.setPack(p299);
            p299.password_SET("lqjdteckDdyycWhaBCkdOapuuoamjkYqqibJrD", PH) ;
            p299.ssid_SET("rFybxkpTexalushrbnagqEbiwixdbo", PH) ;
            CommunicationChannel.instance.send(p299);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnPROTOCOL_VERSIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.version == (ushort)(ushort)23582);
                Debug.Assert(pack.library_version_hash.SequenceEqual(new byte[] {(byte)187, (byte)84, (byte)31, (byte)252, (byte)112, (byte)255, (byte)38, (byte)173}));
                Debug.Assert(pack.max_version == (ushort)(ushort)14030);
                Debug.Assert(pack.min_version == (ushort)(ushort)34643);
                Debug.Assert(pack.spec_version_hash.SequenceEqual(new byte[] {(byte)244, (byte)95, (byte)93, (byte)27, (byte)241, (byte)180, (byte)172, (byte)161}));
            };
            GroundControl.PROTOCOL_VERSION p300 = CommunicationChannel.new_PROTOCOL_VERSION();
            PH.setPack(p300);
            p300.max_version = (ushort)(ushort)14030;
            p300.spec_version_hash_SET(new byte[] {(byte)244, (byte)95, (byte)93, (byte)27, (byte)241, (byte)180, (byte)172, (byte)161}, 0) ;
            p300.min_version = (ushort)(ushort)34643;
            p300.library_version_hash_SET(new byte[] {(byte)187, (byte)84, (byte)31, (byte)252, (byte)112, (byte)255, (byte)38, (byte)173}, 0) ;
            p300.version = (ushort)(ushort)23582;
            CommunicationChannel.instance.send(p300);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnUAVCAN_NODE_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.health == UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_ERROR);
                Debug.Assert(pack.uptime_sec == (uint)2045821264U);
                Debug.Assert(pack.time_usec == (ulong)5612753432770160579L);
                Debug.Assert(pack.sub_mode == (byte)(byte)142);
                Debug.Assert(pack.vendor_specific_status_code == (ushort)(ushort)56470);
                Debug.Assert(pack.mode == UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_SOFTWARE_UPDATE);
            };
            GroundControl.UAVCAN_NODE_STATUS p310 = CommunicationChannel.new_UAVCAN_NODE_STATUS();
            PH.setPack(p310);
            p310.vendor_specific_status_code = (ushort)(ushort)56470;
            p310.uptime_sec = (uint)2045821264U;
            p310.mode = UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_SOFTWARE_UPDATE;
            p310.health = UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_ERROR;
            p310.time_usec = (ulong)5612753432770160579L;
            p310.sub_mode = (byte)(byte)142;
            CommunicationChannel.instance.send(p310);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnUAVCAN_NODE_INFOReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.uptime_sec == (uint)1811278290U);
                Debug.Assert(pack.name_LEN(ph) == 63);
                Debug.Assert(pack.name_TRY(ph).Equals("yglikcszkqwvhdrgdnduiyHehxuqcebnBPgislkvnxdkhljoAizqjbdkoubyEdD"));
                Debug.Assert(pack.hw_version_major == (byte)(byte)41);
                Debug.Assert(pack.hw_unique_id.SequenceEqual(new byte[] {(byte)251, (byte)59, (byte)69, (byte)106, (byte)36, (byte)86, (byte)195, (byte)93, (byte)34, (byte)54, (byte)151, (byte)200, (byte)117, (byte)207, (byte)209, (byte)208}));
                Debug.Assert(pack.hw_version_minor == (byte)(byte)67);
                Debug.Assert(pack.sw_vcs_commit == (uint)1789439972U);
                Debug.Assert(pack.sw_version_major == (byte)(byte)6);
                Debug.Assert(pack.time_usec == (ulong)1507443918097194362L);
                Debug.Assert(pack.sw_version_minor == (byte)(byte)176);
            };
            GroundControl.UAVCAN_NODE_INFO p311 = CommunicationChannel.new_UAVCAN_NODE_INFO();
            PH.setPack(p311);
            p311.sw_vcs_commit = (uint)1789439972U;
            p311.hw_version_minor = (byte)(byte)67;
            p311.hw_version_major = (byte)(byte)41;
            p311.hw_unique_id_SET(new byte[] {(byte)251, (byte)59, (byte)69, (byte)106, (byte)36, (byte)86, (byte)195, (byte)93, (byte)34, (byte)54, (byte)151, (byte)200, (byte)117, (byte)207, (byte)209, (byte)208}, 0) ;
            p311.time_usec = (ulong)1507443918097194362L;
            p311.uptime_sec = (uint)1811278290U;
            p311.sw_version_major = (byte)(byte)6;
            p311.name_SET("yglikcszkqwvhdrgdnduiyHehxuqcebnBPgislkvnxdkhljoAizqjbdkoubyEdD", PH) ;
            p311.sw_version_minor = (byte)(byte)176;
            CommunicationChannel.instance.send(p311);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_REQUEST_READReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)171);
                Debug.Assert(pack.param_id_LEN(ph) == 7);
                Debug.Assert(pack.param_id_TRY(ph).Equals("xyzonrv"));
                Debug.Assert(pack.target_component == (byte)(byte)83);
                Debug.Assert(pack.param_index == (short)(short)5365);
            };
            GroundControl.PARAM_EXT_REQUEST_READ p320 = CommunicationChannel.new_PARAM_EXT_REQUEST_READ();
            PH.setPack(p320);
            p320.target_system = (byte)(byte)171;
            p320.param_id_SET("xyzonrv", PH) ;
            p320.param_index = (short)(short)5365;
            p320.target_component = (byte)(byte)83;
            CommunicationChannel.instance.send(p320);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)28);
                Debug.Assert(pack.target_system == (byte)(byte)210);
            };
            GroundControl.PARAM_EXT_REQUEST_LIST p321 = CommunicationChannel.new_PARAM_EXT_REQUEST_LIST();
            PH.setPack(p321);
            p321.target_component = (byte)(byte)28;
            p321.target_system = (byte)(byte)210;
            CommunicationChannel.instance.send(p321);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_VALUEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_count == (ushort)(ushort)57795);
                Debug.Assert(pack.param_type == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_REAL32);
                Debug.Assert(pack.param_index == (ushort)(ushort)56486);
                Debug.Assert(pack.param_id_LEN(ph) == 3);
                Debug.Assert(pack.param_id_TRY(ph).Equals("hxJ"));
                Debug.Assert(pack.param_value_LEN(ph) == 2);
                Debug.Assert(pack.param_value_TRY(ph).Equals("qy"));
            };
            GroundControl.PARAM_EXT_VALUE p322 = CommunicationChannel.new_PARAM_EXT_VALUE();
            PH.setPack(p322);
            p322.param_id_SET("hxJ", PH) ;
            p322.param_count = (ushort)(ushort)57795;
            p322.param_index = (ushort)(ushort)56486;
            p322.param_value_SET("qy", PH) ;
            p322.param_type = MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_REAL32;
            CommunicationChannel.instance.send(p322);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_SETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_value_LEN(ph) == 100);
                Debug.Assert(pack.param_value_TRY(ph).Equals("nubhgttzriGzditzjtfzUqaauwwoebicpiscifcegsQrxkisSanraJzssubufjrwjcFvhyuejhyyvvboujyqFGrlIqkbviepduvf"));
                Debug.Assert(pack.param_id_LEN(ph) == 5);
                Debug.Assert(pack.param_id_TRY(ph).Equals("ezZyo"));
                Debug.Assert(pack.target_system == (byte)(byte)192);
                Debug.Assert(pack.param_type == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_REAL32);
                Debug.Assert(pack.target_component == (byte)(byte)73);
            };
            GroundControl.PARAM_EXT_SET p323 = CommunicationChannel.new_PARAM_EXT_SET();
            PH.setPack(p323);
            p323.param_value_SET("nubhgttzriGzditzjtfzUqaauwwoebicpiscifcegsQrxkisSanraJzssubufjrwjcFvhyuejhyyvvboujyqFGrlIqkbviepduvf", PH) ;
            p323.param_id_SET("ezZyo", PH) ;
            p323.target_system = (byte)(byte)192;
            p323.param_type = MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_REAL32;
            p323.target_component = (byte)(byte)73;
            CommunicationChannel.instance.send(p323);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_result == PARAM_ACK.PARAM_ACK_ACCEPTED);
                Debug.Assert(pack.param_type == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT8);
                Debug.Assert(pack.param_id_LEN(ph) == 3);
                Debug.Assert(pack.param_id_TRY(ph).Equals("qtM"));
                Debug.Assert(pack.param_value_LEN(ph) == 5);
                Debug.Assert(pack.param_value_TRY(ph).Equals("xdyuy"));
            };
            GroundControl.PARAM_EXT_ACK p324 = CommunicationChannel.new_PARAM_EXT_ACK();
            PH.setPack(p324);
            p324.param_id_SET("qtM", PH) ;
            p324.param_result = PARAM_ACK.PARAM_ACK_ACCEPTED;
            p324.param_type = MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT8;
            p324.param_value_SET("xdyuy", PH) ;
            CommunicationChannel.instance.send(p324);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnOBSTACLE_DISTANCEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.min_distance == (ushort)(ushort)4049);
                Debug.Assert(pack.sensor_type == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_ULTRASOUND);
                Debug.Assert(pack.time_usec == (ulong)6979548536393023266L);
                Debug.Assert(pack.distances.SequenceEqual(new ushort[] {(ushort)32935, (ushort)60245, (ushort)45327, (ushort)2160, (ushort)42641, (ushort)57475, (ushort)62490, (ushort)16964, (ushort)51707, (ushort)59285, (ushort)34405, (ushort)44729, (ushort)25432, (ushort)42489, (ushort)40645, (ushort)13416, (ushort)48170, (ushort)26446, (ushort)7421, (ushort)64436, (ushort)46752, (ushort)53490, (ushort)7467, (ushort)29476, (ushort)17480, (ushort)48901, (ushort)18931, (ushort)31928, (ushort)8822, (ushort)60974, (ushort)29598, (ushort)31355, (ushort)53354, (ushort)5461, (ushort)48351, (ushort)10082, (ushort)13885, (ushort)19365, (ushort)60416, (ushort)17887, (ushort)7883, (ushort)14303, (ushort)54510, (ushort)29004, (ushort)56383, (ushort)15037, (ushort)12343, (ushort)41972, (ushort)42939, (ushort)10238, (ushort)10709, (ushort)41494, (ushort)12258, (ushort)57653, (ushort)800, (ushort)61445, (ushort)12221, (ushort)55335, (ushort)19348, (ushort)58737, (ushort)53464, (ushort)52704, (ushort)39267, (ushort)60679, (ushort)25364, (ushort)15194, (ushort)5433, (ushort)42658, (ushort)40871, (ushort)52678, (ushort)33662, (ushort)60309}));
                Debug.Assert(pack.increment == (byte)(byte)110);
                Debug.Assert(pack.max_distance == (ushort)(ushort)23026);
            };
            GroundControl.OBSTACLE_DISTANCE p330 = CommunicationChannel.new_OBSTACLE_DISTANCE();
            PH.setPack(p330);
            p330.sensor_type = MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_ULTRASOUND;
            p330.max_distance = (ushort)(ushort)23026;
            p330.distances_SET(new ushort[] {(ushort)32935, (ushort)60245, (ushort)45327, (ushort)2160, (ushort)42641, (ushort)57475, (ushort)62490, (ushort)16964, (ushort)51707, (ushort)59285, (ushort)34405, (ushort)44729, (ushort)25432, (ushort)42489, (ushort)40645, (ushort)13416, (ushort)48170, (ushort)26446, (ushort)7421, (ushort)64436, (ushort)46752, (ushort)53490, (ushort)7467, (ushort)29476, (ushort)17480, (ushort)48901, (ushort)18931, (ushort)31928, (ushort)8822, (ushort)60974, (ushort)29598, (ushort)31355, (ushort)53354, (ushort)5461, (ushort)48351, (ushort)10082, (ushort)13885, (ushort)19365, (ushort)60416, (ushort)17887, (ushort)7883, (ushort)14303, (ushort)54510, (ushort)29004, (ushort)56383, (ushort)15037, (ushort)12343, (ushort)41972, (ushort)42939, (ushort)10238, (ushort)10709, (ushort)41494, (ushort)12258, (ushort)57653, (ushort)800, (ushort)61445, (ushort)12221, (ushort)55335, (ushort)19348, (ushort)58737, (ushort)53464, (ushort)52704, (ushort)39267, (ushort)60679, (ushort)25364, (ushort)15194, (ushort)5433, (ushort)42658, (ushort)40871, (ushort)52678, (ushort)33662, (ushort)60309}, 0) ;
            p330.increment = (byte)(byte)110;
            p330.min_distance = (ushort)(ushort)4049;
            p330.time_usec = (ulong)6979548536393023266L;
            CommunicationChannel.instance.send(p330);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
        }
    }
}