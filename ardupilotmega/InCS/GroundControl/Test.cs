
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
                    ulong id = 0;
                    switch(value)
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
                            throw  new ArgumentException("Unknown enum " + value);
                    }
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
                        case MAV_CMD.MAV_CMD_NAV_ALTITUDE_WAIT:
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
                        case MAV_CMD.MAV_CMD_DO_GRIPPER:
                            id = 57;
                            break;
                        case MAV_CMD.MAV_CMD_DO_AUTOTUNE_ENABLE:
                            id = 58;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_SET_YAW_SPEED:
                            id = 59;
                            break;
                        case MAV_CMD.MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL:
                            id = 60;
                            break;
                        case MAV_CMD.MAV_CMD_DO_MOUNT_CONTROL_QUAT:
                            id = 61;
                            break;
                        case MAV_CMD.MAV_CMD_DO_GUIDED_MASTER:
                            id = 62;
                            break;
                        case MAV_CMD.MAV_CMD_DO_GUIDED_LIMITS:
                            id = 63;
                            break;
                        case MAV_CMD.MAV_CMD_DO_ENGINE_CONTROL:
                            id = 64;
                            break;
                        case MAV_CMD.MAV_CMD_DO_LAST:
                            id = 65;
                            break;
                        case MAV_CMD.MAV_CMD_PREFLIGHT_CALIBRATION:
                            id = 66;
                            break;
                        case MAV_CMD.MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS:
                            id = 67;
                            break;
                        case MAV_CMD.MAV_CMD_PREFLIGHT_UAVCAN:
                            id = 68;
                            break;
                        case MAV_CMD.MAV_CMD_PREFLIGHT_STORAGE:
                            id = 69;
                            break;
                        case MAV_CMD.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
                            id = 70;
                            break;
                        case MAV_CMD.MAV_CMD_OVERRIDE_GOTO:
                            id = 71;
                            break;
                        case MAV_CMD.MAV_CMD_MISSION_START:
                            id = 72;
                            break;
                        case MAV_CMD.MAV_CMD_COMPONENT_ARM_DISARM:
                            id = 73;
                            break;
                        case MAV_CMD.MAV_CMD_GET_HOME_POSITION:
                            id = 74;
                            break;
                        case MAV_CMD.MAV_CMD_START_RX_PAIR:
                            id = 75;
                            break;
                        case MAV_CMD.MAV_CMD_GET_MESSAGE_INTERVAL:
                            id = 76;
                            break;
                        case MAV_CMD.MAV_CMD_SET_MESSAGE_INTERVAL:
                            id = 77;
                            break;
                        case MAV_CMD.MAV_CMD_REQUEST_PROTOCOL_VERSION:
                            id = 78;
                            break;
                        case MAV_CMD.MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES:
                            id = 79;
                            break;
                        case MAV_CMD.MAV_CMD_REQUEST_CAMERA_INFORMATION:
                            id = 80;
                            break;
                        case MAV_CMD.MAV_CMD_REQUEST_CAMERA_SETTINGS:
                            id = 81;
                            break;
                        case MAV_CMD.MAV_CMD_REQUEST_STORAGE_INFORMATION:
                            id = 82;
                            break;
                        case MAV_CMD.MAV_CMD_STORAGE_FORMAT:
                            id = 83;
                            break;
                        case MAV_CMD.MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS:
                            id = 84;
                            break;
                        case MAV_CMD.MAV_CMD_REQUEST_FLIGHT_INFORMATION:
                            id = 85;
                            break;
                        case MAV_CMD.MAV_CMD_RESET_CAMERA_SETTINGS:
                            id = 86;
                            break;
                        case MAV_CMD.MAV_CMD_SET_CAMERA_MODE:
                            id = 87;
                            break;
                        case MAV_CMD.MAV_CMD_IMAGE_START_CAPTURE:
                            id = 88;
                            break;
                        case MAV_CMD.MAV_CMD_IMAGE_STOP_CAPTURE:
                            id = 89;
                            break;
                        case MAV_CMD.MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE:
                            id = 90;
                            break;
                        case MAV_CMD.MAV_CMD_DO_TRIGGER_CONTROL:
                            id = 91;
                            break;
                        case MAV_CMD.MAV_CMD_VIDEO_START_CAPTURE:
                            id = 92;
                            break;
                        case MAV_CMD.MAV_CMD_VIDEO_STOP_CAPTURE:
                            id = 93;
                            break;
                        case MAV_CMD.MAV_CMD_VIDEO_START_STREAMING:
                            id = 94;
                            break;
                        case MAV_CMD.MAV_CMD_VIDEO_STOP_STREAMING:
                            id = 95;
                            break;
                        case MAV_CMD.MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION:
                            id = 96;
                            break;
                        case MAV_CMD.MAV_CMD_LOGGING_START:
                            id = 97;
                            break;
                        case MAV_CMD.MAV_CMD_LOGGING_STOP:
                            id = 98;
                            break;
                        case MAV_CMD.MAV_CMD_AIRFRAME_CONFIGURATION:
                            id = 99;
                            break;
                        case MAV_CMD.MAV_CMD_PANORAMA_CREATE:
                            id = 100;
                            break;
                        case MAV_CMD.MAV_CMD_DO_VTOL_TRANSITION:
                            id = 101;
                            break;
                        case MAV_CMD.MAV_CMD_ARM_AUTHORIZATION_REQUEST:
                            id = 102;
                            break;
                        case MAV_CMD.MAV_CMD_SET_GUIDED_SUBMODE_STANDARD:
                            id = 103;
                            break;
                        case MAV_CMD.MAV_CMD_SET_GUIDED_SUBMODE_CIRCLE:
                            id = 104;
                            break;
                        case MAV_CMD.MAV_CMD_CONDITION_GATE:
                            id = 105;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_FENCE_RETURN_POINT:
                            id = 106;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION:
                            id = 107;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION:
                            id = 108;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION:
                            id = 109;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION:
                            id = 110;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_RALLY_POINT:
                            id = 111;
                            break;
                        case MAV_CMD.MAV_CMD_UAVCAN_GET_NODE_INFO:
                            id = 112;
                            break;
                        case MAV_CMD.MAV_CMD_PAYLOAD_PREPARE_DEPLOY:
                            id = 113;
                            break;
                        case MAV_CMD.MAV_CMD_PAYLOAD_CONTROL_DEPLOY:
                            id = 114;
                            break;
                        case MAV_CMD.MAV_CMD_WAYPOINT_USER_1:
                            id = 115;
                            break;
                        case MAV_CMD.MAV_CMD_WAYPOINT_USER_2:
                            id = 116;
                            break;
                        case MAV_CMD.MAV_CMD_WAYPOINT_USER_3:
                            id = 117;
                            break;
                        case MAV_CMD.MAV_CMD_WAYPOINT_USER_4:
                            id = 118;
                            break;
                        case MAV_CMD.MAV_CMD_WAYPOINT_USER_5:
                            id = 119;
                            break;
                        case MAV_CMD.MAV_CMD_SPATIAL_USER_1:
                            id = 120;
                            break;
                        case MAV_CMD.MAV_CMD_SPATIAL_USER_2:
                            id = 121;
                            break;
                        case MAV_CMD.MAV_CMD_SPATIAL_USER_3:
                            id = 122;
                            break;
                        case MAV_CMD.MAV_CMD_SPATIAL_USER_4:
                            id = 123;
                            break;
                        case MAV_CMD.MAV_CMD_SPATIAL_USER_5:
                            id = 124;
                            break;
                        case MAV_CMD.MAV_CMD_USER_1:
                            id = 125;
                            break;
                        case MAV_CMD.MAV_CMD_USER_2:
                            id = 126;
                            break;
                        case MAV_CMD.MAV_CMD_USER_3:
                            id = 127;
                            break;
                        case MAV_CMD.MAV_CMD_USER_4:
                            id = 128;
                            break;
                        case MAV_CMD.MAV_CMD_USER_5:
                            id = 129;
                            break;
                        case MAV_CMD.MAV_CMD_POWER_OFF_INITIATED:
                            id = 130;
                            break;
                        case MAV_CMD.MAV_CMD_SOLO_BTN_FLY_CLICK:
                            id = 131;
                            break;
                        case MAV_CMD.MAV_CMD_SOLO_BTN_FLY_HOLD:
                            id = 132;
                            break;
                        case MAV_CMD.MAV_CMD_SOLO_BTN_PAUSE_CLICK:
                            id = 133;
                            break;
                        case MAV_CMD.MAV_CMD_DO_START_MAG_CAL:
                            id = 134;
                            break;
                        case MAV_CMD.MAV_CMD_DO_ACCEPT_MAG_CAL:
                            id = 135;
                            break;
                        case MAV_CMD.MAV_CMD_DO_CANCEL_MAG_CAL:
                            id = 136;
                            break;
                        case MAV_CMD.MAV_CMD_SET_FACTORY_TEST_MODE:
                            id = 137;
                            break;
                        case MAV_CMD.MAV_CMD_DO_SEND_BANNER:
                            id = 138;
                            break;
                        case MAV_CMD.MAV_CMD_ACCELCAL_VEHICLE_POS:
                            id = 139;
                            break;
                        case MAV_CMD.MAV_CMD_GIMBAL_RESET:
                            id = 140;
                            break;
                        case MAV_CMD.MAV_CMD_GIMBAL_AXIS_CALIBRATION_STATUS:
                            id = 141;
                            break;
                        case MAV_CMD.MAV_CMD_GIMBAL_REQUEST_AXIS_CALIBRATION:
                            id = 142;
                            break;
                        case MAV_CMD.MAV_CMD_GIMBAL_FULL_RESET:
                            id = 143;
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
                        case MAV_CMD.MAV_CMD_NAV_ALTITUDE_WAIT:
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
                        case MAV_CMD.MAV_CMD_DO_GRIPPER:
                            id = 57;
                            break;
                        case MAV_CMD.MAV_CMD_DO_AUTOTUNE_ENABLE:
                            id = 58;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_SET_YAW_SPEED:
                            id = 59;
                            break;
                        case MAV_CMD.MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL:
                            id = 60;
                            break;
                        case MAV_CMD.MAV_CMD_DO_MOUNT_CONTROL_QUAT:
                            id = 61;
                            break;
                        case MAV_CMD.MAV_CMD_DO_GUIDED_MASTER:
                            id = 62;
                            break;
                        case MAV_CMD.MAV_CMD_DO_GUIDED_LIMITS:
                            id = 63;
                            break;
                        case MAV_CMD.MAV_CMD_DO_ENGINE_CONTROL:
                            id = 64;
                            break;
                        case MAV_CMD.MAV_CMD_DO_LAST:
                            id = 65;
                            break;
                        case MAV_CMD.MAV_CMD_PREFLIGHT_CALIBRATION:
                            id = 66;
                            break;
                        case MAV_CMD.MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS:
                            id = 67;
                            break;
                        case MAV_CMD.MAV_CMD_PREFLIGHT_UAVCAN:
                            id = 68;
                            break;
                        case MAV_CMD.MAV_CMD_PREFLIGHT_STORAGE:
                            id = 69;
                            break;
                        case MAV_CMD.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
                            id = 70;
                            break;
                        case MAV_CMD.MAV_CMD_OVERRIDE_GOTO:
                            id = 71;
                            break;
                        case MAV_CMD.MAV_CMD_MISSION_START:
                            id = 72;
                            break;
                        case MAV_CMD.MAV_CMD_COMPONENT_ARM_DISARM:
                            id = 73;
                            break;
                        case MAV_CMD.MAV_CMD_GET_HOME_POSITION:
                            id = 74;
                            break;
                        case MAV_CMD.MAV_CMD_START_RX_PAIR:
                            id = 75;
                            break;
                        case MAV_CMD.MAV_CMD_GET_MESSAGE_INTERVAL:
                            id = 76;
                            break;
                        case MAV_CMD.MAV_CMD_SET_MESSAGE_INTERVAL:
                            id = 77;
                            break;
                        case MAV_CMD.MAV_CMD_REQUEST_PROTOCOL_VERSION:
                            id = 78;
                            break;
                        case MAV_CMD.MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES:
                            id = 79;
                            break;
                        case MAV_CMD.MAV_CMD_REQUEST_CAMERA_INFORMATION:
                            id = 80;
                            break;
                        case MAV_CMD.MAV_CMD_REQUEST_CAMERA_SETTINGS:
                            id = 81;
                            break;
                        case MAV_CMD.MAV_CMD_REQUEST_STORAGE_INFORMATION:
                            id = 82;
                            break;
                        case MAV_CMD.MAV_CMD_STORAGE_FORMAT:
                            id = 83;
                            break;
                        case MAV_CMD.MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS:
                            id = 84;
                            break;
                        case MAV_CMD.MAV_CMD_REQUEST_FLIGHT_INFORMATION:
                            id = 85;
                            break;
                        case MAV_CMD.MAV_CMD_RESET_CAMERA_SETTINGS:
                            id = 86;
                            break;
                        case MAV_CMD.MAV_CMD_SET_CAMERA_MODE:
                            id = 87;
                            break;
                        case MAV_CMD.MAV_CMD_IMAGE_START_CAPTURE:
                            id = 88;
                            break;
                        case MAV_CMD.MAV_CMD_IMAGE_STOP_CAPTURE:
                            id = 89;
                            break;
                        case MAV_CMD.MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE:
                            id = 90;
                            break;
                        case MAV_CMD.MAV_CMD_DO_TRIGGER_CONTROL:
                            id = 91;
                            break;
                        case MAV_CMD.MAV_CMD_VIDEO_START_CAPTURE:
                            id = 92;
                            break;
                        case MAV_CMD.MAV_CMD_VIDEO_STOP_CAPTURE:
                            id = 93;
                            break;
                        case MAV_CMD.MAV_CMD_VIDEO_START_STREAMING:
                            id = 94;
                            break;
                        case MAV_CMD.MAV_CMD_VIDEO_STOP_STREAMING:
                            id = 95;
                            break;
                        case MAV_CMD.MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION:
                            id = 96;
                            break;
                        case MAV_CMD.MAV_CMD_LOGGING_START:
                            id = 97;
                            break;
                        case MAV_CMD.MAV_CMD_LOGGING_STOP:
                            id = 98;
                            break;
                        case MAV_CMD.MAV_CMD_AIRFRAME_CONFIGURATION:
                            id = 99;
                            break;
                        case MAV_CMD.MAV_CMD_PANORAMA_CREATE:
                            id = 100;
                            break;
                        case MAV_CMD.MAV_CMD_DO_VTOL_TRANSITION:
                            id = 101;
                            break;
                        case MAV_CMD.MAV_CMD_ARM_AUTHORIZATION_REQUEST:
                            id = 102;
                            break;
                        case MAV_CMD.MAV_CMD_SET_GUIDED_SUBMODE_STANDARD:
                            id = 103;
                            break;
                        case MAV_CMD.MAV_CMD_SET_GUIDED_SUBMODE_CIRCLE:
                            id = 104;
                            break;
                        case MAV_CMD.MAV_CMD_CONDITION_GATE:
                            id = 105;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_FENCE_RETURN_POINT:
                            id = 106;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION:
                            id = 107;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION:
                            id = 108;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION:
                            id = 109;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION:
                            id = 110;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_RALLY_POINT:
                            id = 111;
                            break;
                        case MAV_CMD.MAV_CMD_UAVCAN_GET_NODE_INFO:
                            id = 112;
                            break;
                        case MAV_CMD.MAV_CMD_PAYLOAD_PREPARE_DEPLOY:
                            id = 113;
                            break;
                        case MAV_CMD.MAV_CMD_PAYLOAD_CONTROL_DEPLOY:
                            id = 114;
                            break;
                        case MAV_CMD.MAV_CMD_WAYPOINT_USER_1:
                            id = 115;
                            break;
                        case MAV_CMD.MAV_CMD_WAYPOINT_USER_2:
                            id = 116;
                            break;
                        case MAV_CMD.MAV_CMD_WAYPOINT_USER_3:
                            id = 117;
                            break;
                        case MAV_CMD.MAV_CMD_WAYPOINT_USER_4:
                            id = 118;
                            break;
                        case MAV_CMD.MAV_CMD_WAYPOINT_USER_5:
                            id = 119;
                            break;
                        case MAV_CMD.MAV_CMD_SPATIAL_USER_1:
                            id = 120;
                            break;
                        case MAV_CMD.MAV_CMD_SPATIAL_USER_2:
                            id = 121;
                            break;
                        case MAV_CMD.MAV_CMD_SPATIAL_USER_3:
                            id = 122;
                            break;
                        case MAV_CMD.MAV_CMD_SPATIAL_USER_4:
                            id = 123;
                            break;
                        case MAV_CMD.MAV_CMD_SPATIAL_USER_5:
                            id = 124;
                            break;
                        case MAV_CMD.MAV_CMD_USER_1:
                            id = 125;
                            break;
                        case MAV_CMD.MAV_CMD_USER_2:
                            id = 126;
                            break;
                        case MAV_CMD.MAV_CMD_USER_3:
                            id = 127;
                            break;
                        case MAV_CMD.MAV_CMD_USER_4:
                            id = 128;
                            break;
                        case MAV_CMD.MAV_CMD_USER_5:
                            id = 129;
                            break;
                        case MAV_CMD.MAV_CMD_POWER_OFF_INITIATED:
                            id = 130;
                            break;
                        case MAV_CMD.MAV_CMD_SOLO_BTN_FLY_CLICK:
                            id = 131;
                            break;
                        case MAV_CMD.MAV_CMD_SOLO_BTN_FLY_HOLD:
                            id = 132;
                            break;
                        case MAV_CMD.MAV_CMD_SOLO_BTN_PAUSE_CLICK:
                            id = 133;
                            break;
                        case MAV_CMD.MAV_CMD_DO_START_MAG_CAL:
                            id = 134;
                            break;
                        case MAV_CMD.MAV_CMD_DO_ACCEPT_MAG_CAL:
                            id = 135;
                            break;
                        case MAV_CMD.MAV_CMD_DO_CANCEL_MAG_CAL:
                            id = 136;
                            break;
                        case MAV_CMD.MAV_CMD_SET_FACTORY_TEST_MODE:
                            id = 137;
                            break;
                        case MAV_CMD.MAV_CMD_DO_SEND_BANNER:
                            id = 138;
                            break;
                        case MAV_CMD.MAV_CMD_ACCELCAL_VEHICLE_POS:
                            id = 139;
                            break;
                        case MAV_CMD.MAV_CMD_GIMBAL_RESET:
                            id = 140;
                            break;
                        case MAV_CMD.MAV_CMD_GIMBAL_AXIS_CALIBRATION_STATUS:
                            id = 141;
                            break;
                        case MAV_CMD.MAV_CMD_GIMBAL_REQUEST_AXIS_CALIBRATION:
                            id = 142;
                            break;
                        case MAV_CMD.MAV_CMD_GIMBAL_FULL_RESET:
                            id = 143;
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
                        case MAV_CMD.MAV_CMD_NAV_ALTITUDE_WAIT:
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
                        case MAV_CMD.MAV_CMD_DO_GRIPPER:
                            id = 57;
                            break;
                        case MAV_CMD.MAV_CMD_DO_AUTOTUNE_ENABLE:
                            id = 58;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_SET_YAW_SPEED:
                            id = 59;
                            break;
                        case MAV_CMD.MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL:
                            id = 60;
                            break;
                        case MAV_CMD.MAV_CMD_DO_MOUNT_CONTROL_QUAT:
                            id = 61;
                            break;
                        case MAV_CMD.MAV_CMD_DO_GUIDED_MASTER:
                            id = 62;
                            break;
                        case MAV_CMD.MAV_CMD_DO_GUIDED_LIMITS:
                            id = 63;
                            break;
                        case MAV_CMD.MAV_CMD_DO_ENGINE_CONTROL:
                            id = 64;
                            break;
                        case MAV_CMD.MAV_CMD_DO_LAST:
                            id = 65;
                            break;
                        case MAV_CMD.MAV_CMD_PREFLIGHT_CALIBRATION:
                            id = 66;
                            break;
                        case MAV_CMD.MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS:
                            id = 67;
                            break;
                        case MAV_CMD.MAV_CMD_PREFLIGHT_UAVCAN:
                            id = 68;
                            break;
                        case MAV_CMD.MAV_CMD_PREFLIGHT_STORAGE:
                            id = 69;
                            break;
                        case MAV_CMD.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
                            id = 70;
                            break;
                        case MAV_CMD.MAV_CMD_OVERRIDE_GOTO:
                            id = 71;
                            break;
                        case MAV_CMD.MAV_CMD_MISSION_START:
                            id = 72;
                            break;
                        case MAV_CMD.MAV_CMD_COMPONENT_ARM_DISARM:
                            id = 73;
                            break;
                        case MAV_CMD.MAV_CMD_GET_HOME_POSITION:
                            id = 74;
                            break;
                        case MAV_CMD.MAV_CMD_START_RX_PAIR:
                            id = 75;
                            break;
                        case MAV_CMD.MAV_CMD_GET_MESSAGE_INTERVAL:
                            id = 76;
                            break;
                        case MAV_CMD.MAV_CMD_SET_MESSAGE_INTERVAL:
                            id = 77;
                            break;
                        case MAV_CMD.MAV_CMD_REQUEST_PROTOCOL_VERSION:
                            id = 78;
                            break;
                        case MAV_CMD.MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES:
                            id = 79;
                            break;
                        case MAV_CMD.MAV_CMD_REQUEST_CAMERA_INFORMATION:
                            id = 80;
                            break;
                        case MAV_CMD.MAV_CMD_REQUEST_CAMERA_SETTINGS:
                            id = 81;
                            break;
                        case MAV_CMD.MAV_CMD_REQUEST_STORAGE_INFORMATION:
                            id = 82;
                            break;
                        case MAV_CMD.MAV_CMD_STORAGE_FORMAT:
                            id = 83;
                            break;
                        case MAV_CMD.MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS:
                            id = 84;
                            break;
                        case MAV_CMD.MAV_CMD_REQUEST_FLIGHT_INFORMATION:
                            id = 85;
                            break;
                        case MAV_CMD.MAV_CMD_RESET_CAMERA_SETTINGS:
                            id = 86;
                            break;
                        case MAV_CMD.MAV_CMD_SET_CAMERA_MODE:
                            id = 87;
                            break;
                        case MAV_CMD.MAV_CMD_IMAGE_START_CAPTURE:
                            id = 88;
                            break;
                        case MAV_CMD.MAV_CMD_IMAGE_STOP_CAPTURE:
                            id = 89;
                            break;
                        case MAV_CMD.MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE:
                            id = 90;
                            break;
                        case MAV_CMD.MAV_CMD_DO_TRIGGER_CONTROL:
                            id = 91;
                            break;
                        case MAV_CMD.MAV_CMD_VIDEO_START_CAPTURE:
                            id = 92;
                            break;
                        case MAV_CMD.MAV_CMD_VIDEO_STOP_CAPTURE:
                            id = 93;
                            break;
                        case MAV_CMD.MAV_CMD_VIDEO_START_STREAMING:
                            id = 94;
                            break;
                        case MAV_CMD.MAV_CMD_VIDEO_STOP_STREAMING:
                            id = 95;
                            break;
                        case MAV_CMD.MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION:
                            id = 96;
                            break;
                        case MAV_CMD.MAV_CMD_LOGGING_START:
                            id = 97;
                            break;
                        case MAV_CMD.MAV_CMD_LOGGING_STOP:
                            id = 98;
                            break;
                        case MAV_CMD.MAV_CMD_AIRFRAME_CONFIGURATION:
                            id = 99;
                            break;
                        case MAV_CMD.MAV_CMD_PANORAMA_CREATE:
                            id = 100;
                            break;
                        case MAV_CMD.MAV_CMD_DO_VTOL_TRANSITION:
                            id = 101;
                            break;
                        case MAV_CMD.MAV_CMD_ARM_AUTHORIZATION_REQUEST:
                            id = 102;
                            break;
                        case MAV_CMD.MAV_CMD_SET_GUIDED_SUBMODE_STANDARD:
                            id = 103;
                            break;
                        case MAV_CMD.MAV_CMD_SET_GUIDED_SUBMODE_CIRCLE:
                            id = 104;
                            break;
                        case MAV_CMD.MAV_CMD_CONDITION_GATE:
                            id = 105;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_FENCE_RETURN_POINT:
                            id = 106;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION:
                            id = 107;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION:
                            id = 108;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION:
                            id = 109;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION:
                            id = 110;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_RALLY_POINT:
                            id = 111;
                            break;
                        case MAV_CMD.MAV_CMD_UAVCAN_GET_NODE_INFO:
                            id = 112;
                            break;
                        case MAV_CMD.MAV_CMD_PAYLOAD_PREPARE_DEPLOY:
                            id = 113;
                            break;
                        case MAV_CMD.MAV_CMD_PAYLOAD_CONTROL_DEPLOY:
                            id = 114;
                            break;
                        case MAV_CMD.MAV_CMD_WAYPOINT_USER_1:
                            id = 115;
                            break;
                        case MAV_CMD.MAV_CMD_WAYPOINT_USER_2:
                            id = 116;
                            break;
                        case MAV_CMD.MAV_CMD_WAYPOINT_USER_3:
                            id = 117;
                            break;
                        case MAV_CMD.MAV_CMD_WAYPOINT_USER_4:
                            id = 118;
                            break;
                        case MAV_CMD.MAV_CMD_WAYPOINT_USER_5:
                            id = 119;
                            break;
                        case MAV_CMD.MAV_CMD_SPATIAL_USER_1:
                            id = 120;
                            break;
                        case MAV_CMD.MAV_CMD_SPATIAL_USER_2:
                            id = 121;
                            break;
                        case MAV_CMD.MAV_CMD_SPATIAL_USER_3:
                            id = 122;
                            break;
                        case MAV_CMD.MAV_CMD_SPATIAL_USER_4:
                            id = 123;
                            break;
                        case MAV_CMD.MAV_CMD_SPATIAL_USER_5:
                            id = 124;
                            break;
                        case MAV_CMD.MAV_CMD_USER_1:
                            id = 125;
                            break;
                        case MAV_CMD.MAV_CMD_USER_2:
                            id = 126;
                            break;
                        case MAV_CMD.MAV_CMD_USER_3:
                            id = 127;
                            break;
                        case MAV_CMD.MAV_CMD_USER_4:
                            id = 128;
                            break;
                        case MAV_CMD.MAV_CMD_USER_5:
                            id = 129;
                            break;
                        case MAV_CMD.MAV_CMD_POWER_OFF_INITIATED:
                            id = 130;
                            break;
                        case MAV_CMD.MAV_CMD_SOLO_BTN_FLY_CLICK:
                            id = 131;
                            break;
                        case MAV_CMD.MAV_CMD_SOLO_BTN_FLY_HOLD:
                            id = 132;
                            break;
                        case MAV_CMD.MAV_CMD_SOLO_BTN_PAUSE_CLICK:
                            id = 133;
                            break;
                        case MAV_CMD.MAV_CMD_DO_START_MAG_CAL:
                            id = 134;
                            break;
                        case MAV_CMD.MAV_CMD_DO_ACCEPT_MAG_CAL:
                            id = 135;
                            break;
                        case MAV_CMD.MAV_CMD_DO_CANCEL_MAG_CAL:
                            id = 136;
                            break;
                        case MAV_CMD.MAV_CMD_SET_FACTORY_TEST_MODE:
                            id = 137;
                            break;
                        case MAV_CMD.MAV_CMD_DO_SEND_BANNER:
                            id = 138;
                            break;
                        case MAV_CMD.MAV_CMD_ACCELCAL_VEHICLE_POS:
                            id = 139;
                            break;
                        case MAV_CMD.MAV_CMD_GIMBAL_RESET:
                            id = 140;
                            break;
                        case MAV_CMD.MAV_CMD_GIMBAL_AXIS_CALIBRATION_STATUS:
                            id = 141;
                            break;
                        case MAV_CMD.MAV_CMD_GIMBAL_REQUEST_AXIS_CALIBRATION:
                            id = 142;
                            break;
                        case MAV_CMD.MAV_CMD_GIMBAL_FULL_RESET:
                            id = 143;
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
                        case MAV_CMD.MAV_CMD_NAV_ALTITUDE_WAIT:
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
                        case MAV_CMD.MAV_CMD_DO_GRIPPER:
                            id = 57;
                            break;
                        case MAV_CMD.MAV_CMD_DO_AUTOTUNE_ENABLE:
                            id = 58;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_SET_YAW_SPEED:
                            id = 59;
                            break;
                        case MAV_CMD.MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL:
                            id = 60;
                            break;
                        case MAV_CMD.MAV_CMD_DO_MOUNT_CONTROL_QUAT:
                            id = 61;
                            break;
                        case MAV_CMD.MAV_CMD_DO_GUIDED_MASTER:
                            id = 62;
                            break;
                        case MAV_CMD.MAV_CMD_DO_GUIDED_LIMITS:
                            id = 63;
                            break;
                        case MAV_CMD.MAV_CMD_DO_ENGINE_CONTROL:
                            id = 64;
                            break;
                        case MAV_CMD.MAV_CMD_DO_LAST:
                            id = 65;
                            break;
                        case MAV_CMD.MAV_CMD_PREFLIGHT_CALIBRATION:
                            id = 66;
                            break;
                        case MAV_CMD.MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS:
                            id = 67;
                            break;
                        case MAV_CMD.MAV_CMD_PREFLIGHT_UAVCAN:
                            id = 68;
                            break;
                        case MAV_CMD.MAV_CMD_PREFLIGHT_STORAGE:
                            id = 69;
                            break;
                        case MAV_CMD.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
                            id = 70;
                            break;
                        case MAV_CMD.MAV_CMD_OVERRIDE_GOTO:
                            id = 71;
                            break;
                        case MAV_CMD.MAV_CMD_MISSION_START:
                            id = 72;
                            break;
                        case MAV_CMD.MAV_CMD_COMPONENT_ARM_DISARM:
                            id = 73;
                            break;
                        case MAV_CMD.MAV_CMD_GET_HOME_POSITION:
                            id = 74;
                            break;
                        case MAV_CMD.MAV_CMD_START_RX_PAIR:
                            id = 75;
                            break;
                        case MAV_CMD.MAV_CMD_GET_MESSAGE_INTERVAL:
                            id = 76;
                            break;
                        case MAV_CMD.MAV_CMD_SET_MESSAGE_INTERVAL:
                            id = 77;
                            break;
                        case MAV_CMD.MAV_CMD_REQUEST_PROTOCOL_VERSION:
                            id = 78;
                            break;
                        case MAV_CMD.MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES:
                            id = 79;
                            break;
                        case MAV_CMD.MAV_CMD_REQUEST_CAMERA_INFORMATION:
                            id = 80;
                            break;
                        case MAV_CMD.MAV_CMD_REQUEST_CAMERA_SETTINGS:
                            id = 81;
                            break;
                        case MAV_CMD.MAV_CMD_REQUEST_STORAGE_INFORMATION:
                            id = 82;
                            break;
                        case MAV_CMD.MAV_CMD_STORAGE_FORMAT:
                            id = 83;
                            break;
                        case MAV_CMD.MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS:
                            id = 84;
                            break;
                        case MAV_CMD.MAV_CMD_REQUEST_FLIGHT_INFORMATION:
                            id = 85;
                            break;
                        case MAV_CMD.MAV_CMD_RESET_CAMERA_SETTINGS:
                            id = 86;
                            break;
                        case MAV_CMD.MAV_CMD_SET_CAMERA_MODE:
                            id = 87;
                            break;
                        case MAV_CMD.MAV_CMD_IMAGE_START_CAPTURE:
                            id = 88;
                            break;
                        case MAV_CMD.MAV_CMD_IMAGE_STOP_CAPTURE:
                            id = 89;
                            break;
                        case MAV_CMD.MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE:
                            id = 90;
                            break;
                        case MAV_CMD.MAV_CMD_DO_TRIGGER_CONTROL:
                            id = 91;
                            break;
                        case MAV_CMD.MAV_CMD_VIDEO_START_CAPTURE:
                            id = 92;
                            break;
                        case MAV_CMD.MAV_CMD_VIDEO_STOP_CAPTURE:
                            id = 93;
                            break;
                        case MAV_CMD.MAV_CMD_VIDEO_START_STREAMING:
                            id = 94;
                            break;
                        case MAV_CMD.MAV_CMD_VIDEO_STOP_STREAMING:
                            id = 95;
                            break;
                        case MAV_CMD.MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION:
                            id = 96;
                            break;
                        case MAV_CMD.MAV_CMD_LOGGING_START:
                            id = 97;
                            break;
                        case MAV_CMD.MAV_CMD_LOGGING_STOP:
                            id = 98;
                            break;
                        case MAV_CMD.MAV_CMD_AIRFRAME_CONFIGURATION:
                            id = 99;
                            break;
                        case MAV_CMD.MAV_CMD_PANORAMA_CREATE:
                            id = 100;
                            break;
                        case MAV_CMD.MAV_CMD_DO_VTOL_TRANSITION:
                            id = 101;
                            break;
                        case MAV_CMD.MAV_CMD_ARM_AUTHORIZATION_REQUEST:
                            id = 102;
                            break;
                        case MAV_CMD.MAV_CMD_SET_GUIDED_SUBMODE_STANDARD:
                            id = 103;
                            break;
                        case MAV_CMD.MAV_CMD_SET_GUIDED_SUBMODE_CIRCLE:
                            id = 104;
                            break;
                        case MAV_CMD.MAV_CMD_CONDITION_GATE:
                            id = 105;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_FENCE_RETURN_POINT:
                            id = 106;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION:
                            id = 107;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION:
                            id = 108;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION:
                            id = 109;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION:
                            id = 110;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_RALLY_POINT:
                            id = 111;
                            break;
                        case MAV_CMD.MAV_CMD_UAVCAN_GET_NODE_INFO:
                            id = 112;
                            break;
                        case MAV_CMD.MAV_CMD_PAYLOAD_PREPARE_DEPLOY:
                            id = 113;
                            break;
                        case MAV_CMD.MAV_CMD_PAYLOAD_CONTROL_DEPLOY:
                            id = 114;
                            break;
                        case MAV_CMD.MAV_CMD_WAYPOINT_USER_1:
                            id = 115;
                            break;
                        case MAV_CMD.MAV_CMD_WAYPOINT_USER_2:
                            id = 116;
                            break;
                        case MAV_CMD.MAV_CMD_WAYPOINT_USER_3:
                            id = 117;
                            break;
                        case MAV_CMD.MAV_CMD_WAYPOINT_USER_4:
                            id = 118;
                            break;
                        case MAV_CMD.MAV_CMD_WAYPOINT_USER_5:
                            id = 119;
                            break;
                        case MAV_CMD.MAV_CMD_SPATIAL_USER_1:
                            id = 120;
                            break;
                        case MAV_CMD.MAV_CMD_SPATIAL_USER_2:
                            id = 121;
                            break;
                        case MAV_CMD.MAV_CMD_SPATIAL_USER_3:
                            id = 122;
                            break;
                        case MAV_CMD.MAV_CMD_SPATIAL_USER_4:
                            id = 123;
                            break;
                        case MAV_CMD.MAV_CMD_SPATIAL_USER_5:
                            id = 124;
                            break;
                        case MAV_CMD.MAV_CMD_USER_1:
                            id = 125;
                            break;
                        case MAV_CMD.MAV_CMD_USER_2:
                            id = 126;
                            break;
                        case MAV_CMD.MAV_CMD_USER_3:
                            id = 127;
                            break;
                        case MAV_CMD.MAV_CMD_USER_4:
                            id = 128;
                            break;
                        case MAV_CMD.MAV_CMD_USER_5:
                            id = 129;
                            break;
                        case MAV_CMD.MAV_CMD_POWER_OFF_INITIATED:
                            id = 130;
                            break;
                        case MAV_CMD.MAV_CMD_SOLO_BTN_FLY_CLICK:
                            id = 131;
                            break;
                        case MAV_CMD.MAV_CMD_SOLO_BTN_FLY_HOLD:
                            id = 132;
                            break;
                        case MAV_CMD.MAV_CMD_SOLO_BTN_PAUSE_CLICK:
                            id = 133;
                            break;
                        case MAV_CMD.MAV_CMD_DO_START_MAG_CAL:
                            id = 134;
                            break;
                        case MAV_CMD.MAV_CMD_DO_ACCEPT_MAG_CAL:
                            id = 135;
                            break;
                        case MAV_CMD.MAV_CMD_DO_CANCEL_MAG_CAL:
                            id = 136;
                            break;
                        case MAV_CMD.MAV_CMD_SET_FACTORY_TEST_MODE:
                            id = 137;
                            break;
                        case MAV_CMD.MAV_CMD_DO_SEND_BANNER:
                            id = 138;
                            break;
                        case MAV_CMD.MAV_CMD_ACCELCAL_VEHICLE_POS:
                            id = 139;
                            break;
                        case MAV_CMD.MAV_CMD_GIMBAL_RESET:
                            id = 140;
                            break;
                        case MAV_CMD.MAV_CMD_GIMBAL_AXIS_CALIBRATION_STATUS:
                            id = 141;
                            break;
                        case MAV_CMD.MAV_CMD_GIMBAL_REQUEST_AXIS_CALIBRATION:
                            id = 142;
                            break;
                        case MAV_CMD.MAV_CMD_GIMBAL_FULL_RESET:
                            id = 143;
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
                        case MAV_CMD.MAV_CMD_NAV_ALTITUDE_WAIT:
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
                        case MAV_CMD.MAV_CMD_DO_GRIPPER:
                            id = 57;
                            break;
                        case MAV_CMD.MAV_CMD_DO_AUTOTUNE_ENABLE:
                            id = 58;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_SET_YAW_SPEED:
                            id = 59;
                            break;
                        case MAV_CMD.MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL:
                            id = 60;
                            break;
                        case MAV_CMD.MAV_CMD_DO_MOUNT_CONTROL_QUAT:
                            id = 61;
                            break;
                        case MAV_CMD.MAV_CMD_DO_GUIDED_MASTER:
                            id = 62;
                            break;
                        case MAV_CMD.MAV_CMD_DO_GUIDED_LIMITS:
                            id = 63;
                            break;
                        case MAV_CMD.MAV_CMD_DO_ENGINE_CONTROL:
                            id = 64;
                            break;
                        case MAV_CMD.MAV_CMD_DO_LAST:
                            id = 65;
                            break;
                        case MAV_CMD.MAV_CMD_PREFLIGHT_CALIBRATION:
                            id = 66;
                            break;
                        case MAV_CMD.MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS:
                            id = 67;
                            break;
                        case MAV_CMD.MAV_CMD_PREFLIGHT_UAVCAN:
                            id = 68;
                            break;
                        case MAV_CMD.MAV_CMD_PREFLIGHT_STORAGE:
                            id = 69;
                            break;
                        case MAV_CMD.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
                            id = 70;
                            break;
                        case MAV_CMD.MAV_CMD_OVERRIDE_GOTO:
                            id = 71;
                            break;
                        case MAV_CMD.MAV_CMD_MISSION_START:
                            id = 72;
                            break;
                        case MAV_CMD.MAV_CMD_COMPONENT_ARM_DISARM:
                            id = 73;
                            break;
                        case MAV_CMD.MAV_CMD_GET_HOME_POSITION:
                            id = 74;
                            break;
                        case MAV_CMD.MAV_CMD_START_RX_PAIR:
                            id = 75;
                            break;
                        case MAV_CMD.MAV_CMD_GET_MESSAGE_INTERVAL:
                            id = 76;
                            break;
                        case MAV_CMD.MAV_CMD_SET_MESSAGE_INTERVAL:
                            id = 77;
                            break;
                        case MAV_CMD.MAV_CMD_REQUEST_PROTOCOL_VERSION:
                            id = 78;
                            break;
                        case MAV_CMD.MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES:
                            id = 79;
                            break;
                        case MAV_CMD.MAV_CMD_REQUEST_CAMERA_INFORMATION:
                            id = 80;
                            break;
                        case MAV_CMD.MAV_CMD_REQUEST_CAMERA_SETTINGS:
                            id = 81;
                            break;
                        case MAV_CMD.MAV_CMD_REQUEST_STORAGE_INFORMATION:
                            id = 82;
                            break;
                        case MAV_CMD.MAV_CMD_STORAGE_FORMAT:
                            id = 83;
                            break;
                        case MAV_CMD.MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS:
                            id = 84;
                            break;
                        case MAV_CMD.MAV_CMD_REQUEST_FLIGHT_INFORMATION:
                            id = 85;
                            break;
                        case MAV_CMD.MAV_CMD_RESET_CAMERA_SETTINGS:
                            id = 86;
                            break;
                        case MAV_CMD.MAV_CMD_SET_CAMERA_MODE:
                            id = 87;
                            break;
                        case MAV_CMD.MAV_CMD_IMAGE_START_CAPTURE:
                            id = 88;
                            break;
                        case MAV_CMD.MAV_CMD_IMAGE_STOP_CAPTURE:
                            id = 89;
                            break;
                        case MAV_CMD.MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE:
                            id = 90;
                            break;
                        case MAV_CMD.MAV_CMD_DO_TRIGGER_CONTROL:
                            id = 91;
                            break;
                        case MAV_CMD.MAV_CMD_VIDEO_START_CAPTURE:
                            id = 92;
                            break;
                        case MAV_CMD.MAV_CMD_VIDEO_STOP_CAPTURE:
                            id = 93;
                            break;
                        case MAV_CMD.MAV_CMD_VIDEO_START_STREAMING:
                            id = 94;
                            break;
                        case MAV_CMD.MAV_CMD_VIDEO_STOP_STREAMING:
                            id = 95;
                            break;
                        case MAV_CMD.MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION:
                            id = 96;
                            break;
                        case MAV_CMD.MAV_CMD_LOGGING_START:
                            id = 97;
                            break;
                        case MAV_CMD.MAV_CMD_LOGGING_STOP:
                            id = 98;
                            break;
                        case MAV_CMD.MAV_CMD_AIRFRAME_CONFIGURATION:
                            id = 99;
                            break;
                        case MAV_CMD.MAV_CMD_PANORAMA_CREATE:
                            id = 100;
                            break;
                        case MAV_CMD.MAV_CMD_DO_VTOL_TRANSITION:
                            id = 101;
                            break;
                        case MAV_CMD.MAV_CMD_ARM_AUTHORIZATION_REQUEST:
                            id = 102;
                            break;
                        case MAV_CMD.MAV_CMD_SET_GUIDED_SUBMODE_STANDARD:
                            id = 103;
                            break;
                        case MAV_CMD.MAV_CMD_SET_GUIDED_SUBMODE_CIRCLE:
                            id = 104;
                            break;
                        case MAV_CMD.MAV_CMD_CONDITION_GATE:
                            id = 105;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_FENCE_RETURN_POINT:
                            id = 106;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION:
                            id = 107;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION:
                            id = 108;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION:
                            id = 109;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION:
                            id = 110;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_RALLY_POINT:
                            id = 111;
                            break;
                        case MAV_CMD.MAV_CMD_UAVCAN_GET_NODE_INFO:
                            id = 112;
                            break;
                        case MAV_CMD.MAV_CMD_PAYLOAD_PREPARE_DEPLOY:
                            id = 113;
                            break;
                        case MAV_CMD.MAV_CMD_PAYLOAD_CONTROL_DEPLOY:
                            id = 114;
                            break;
                        case MAV_CMD.MAV_CMD_WAYPOINT_USER_1:
                            id = 115;
                            break;
                        case MAV_CMD.MAV_CMD_WAYPOINT_USER_2:
                            id = 116;
                            break;
                        case MAV_CMD.MAV_CMD_WAYPOINT_USER_3:
                            id = 117;
                            break;
                        case MAV_CMD.MAV_CMD_WAYPOINT_USER_4:
                            id = 118;
                            break;
                        case MAV_CMD.MAV_CMD_WAYPOINT_USER_5:
                            id = 119;
                            break;
                        case MAV_CMD.MAV_CMD_SPATIAL_USER_1:
                            id = 120;
                            break;
                        case MAV_CMD.MAV_CMD_SPATIAL_USER_2:
                            id = 121;
                            break;
                        case MAV_CMD.MAV_CMD_SPATIAL_USER_3:
                            id = 122;
                            break;
                        case MAV_CMD.MAV_CMD_SPATIAL_USER_4:
                            id = 123;
                            break;
                        case MAV_CMD.MAV_CMD_SPATIAL_USER_5:
                            id = 124;
                            break;
                        case MAV_CMD.MAV_CMD_USER_1:
                            id = 125;
                            break;
                        case MAV_CMD.MAV_CMD_USER_2:
                            id = 126;
                            break;
                        case MAV_CMD.MAV_CMD_USER_3:
                            id = 127;
                            break;
                        case MAV_CMD.MAV_CMD_USER_4:
                            id = 128;
                            break;
                        case MAV_CMD.MAV_CMD_USER_5:
                            id = 129;
                            break;
                        case MAV_CMD.MAV_CMD_POWER_OFF_INITIATED:
                            id = 130;
                            break;
                        case MAV_CMD.MAV_CMD_SOLO_BTN_FLY_CLICK:
                            id = 131;
                            break;
                        case MAV_CMD.MAV_CMD_SOLO_BTN_FLY_HOLD:
                            id = 132;
                            break;
                        case MAV_CMD.MAV_CMD_SOLO_BTN_PAUSE_CLICK:
                            id = 133;
                            break;
                        case MAV_CMD.MAV_CMD_DO_START_MAG_CAL:
                            id = 134;
                            break;
                        case MAV_CMD.MAV_CMD_DO_ACCEPT_MAG_CAL:
                            id = 135;
                            break;
                        case MAV_CMD.MAV_CMD_DO_CANCEL_MAG_CAL:
                            id = 136;
                            break;
                        case MAV_CMD.MAV_CMD_SET_FACTORY_TEST_MODE:
                            id = 137;
                            break;
                        case MAV_CMD.MAV_CMD_DO_SEND_BANNER:
                            id = 138;
                            break;
                        case MAV_CMD.MAV_CMD_ACCELCAL_VEHICLE_POS:
                            id = 139;
                            break;
                        case MAV_CMD.MAV_CMD_GIMBAL_RESET:
                            id = 140;
                            break;
                        case MAV_CMD.MAV_CMD_GIMBAL_AXIS_CALIBRATION_STATUS:
                            id = 141;
                            break;
                        case MAV_CMD.MAV_CMD_GIMBAL_REQUEST_AXIS_CALIBRATION:
                            id = 142;
                            break;
                        case MAV_CMD.MAV_CMD_GIMBAL_FULL_RESET:
                            id = 143;
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
        public class POSITION_TARGET_GLOBAL_INT : GroundControl.POSITION_TARGET_GLOBAL_INT
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
            *	 = 1*/
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
                    ulong id = 0;
                    switch(value)
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
                            throw  new ArgumentException("Unknown enum " + value);
                    }
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
                    ulong id = 0;
                    switch(value)
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
                            throw  new ArgumentException("Unknown enum " + value);
                    }
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
            *	 just onc*/
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
            *	 just onc*/
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
            *	 no CCB*/
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
            *	 one] (table 2-37 DO-282B*/
            public UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON gpsOffsetLon
            {
                get {  return (UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON)(0 +  BitUtils.get_bits(data, 60, 1));}
            }

            public UAVIONIX_ADSB_OUT_RF_SELECT rfSelect //ADS-B transponder reciever and transmit enable flags
            {
                get {  return (UAVIONIX_ADSB_OUT_RF_SELECT)(0 +  BitUtils.get_bits(data, 61, 3));}
            }
            public string callsign_TRY(Inside ph)//Vehicle identifier (8 characters, null terminated, valid characters are A-Z, 0-9, " " only)
            {
                if(ph.field_bit !=  64 && !try_visit_field(ph, 64)  ||  !try_visit_item(ph, 0)) return null;
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
                return (ph.field_bit !=  64 && !try_visit_field(ph, 64)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
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
            *	 (m * 1E-3). (up +ve). If unknown set to INT32_MA*/
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
                get {  return (UAVIONIX_ADSB_RF_HEALTH)(0 +  BitUtils.get_bits(data, 0, 6));}
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
            *	 2=down*/
            public float[] position_delta
            {
                get {return position_delta_GET(new float[3], 0);}
            }
            /**
            *Change in position in meters from previous to current frame rotated into body frame (0=forward, 1=right,
            *	 2=down*/
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
                Debug.Assert(pack.base_mode == (MAV_MODE_FLAG.MAV_MODE_FLAG_HIL_ENABLED));
                Debug.Assert(pack.custom_mode == (uint)3339864721U);
                Debug.Assert(pack.type == MAV_TYPE.MAV_TYPE_QUADROTOR);
                Debug.Assert(pack.mavlink_version == (byte)(byte)52);
                Debug.Assert(pack.system_status == MAV_STATE.MAV_STATE_FLIGHT_TERMINATION);
                Debug.Assert(pack.autopilot == MAV_AUTOPILOT.MAV_AUTOPILOT_PPZ);
            };
            HEARTBEAT p0 = new HEARTBEAT();
            PH.setPack(p0);
            p0.base_mode = (MAV_MODE_FLAG.MAV_MODE_FLAG_HIL_ENABLED);
            p0.custom_mode = (uint)3339864721U;
            p0.system_status = MAV_STATE.MAV_STATE_FLIGHT_TERMINATION;
            p0.mavlink_version = (byte)(byte)52;
            p0.autopilot = MAV_AUTOPILOT.MAV_AUTOPILOT_PPZ;
            p0.type = MAV_TYPE.MAV_TYPE_QUADROTOR;
            SMP_TEST_CH.send(p0);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSYS_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.drop_rate_comm == (ushort)(ushort)41165);
                Debug.Assert(pack.errors_count2 == (ushort)(ushort)37268);
                Debug.Assert(pack.onboard_control_sensors_health == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO2 |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_GPS |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_BATTERY |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE));
                Debug.Assert(pack.load == (ushort)(ushort)31637);
                Debug.Assert(pack.errors_count4 == (ushort)(ushort)22988);
                Debug.Assert(pack.voltage_battery == (ushort)(ushort)16552);
                Debug.Assert(pack.errors_count1 == (ushort)(ushort)15133);
                Debug.Assert(pack.onboard_control_sensors_present == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_LOGGING |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL));
                Debug.Assert(pack.battery_remaining == (sbyte)(sbyte) - 52);
                Debug.Assert(pack.onboard_control_sensors_enabled == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_GPS |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_AHRS |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE));
                Debug.Assert(pack.current_battery == (short)(short) -20569);
                Debug.Assert(pack.errors_comm == (ushort)(ushort)54425);
                Debug.Assert(pack.errors_count3 == (ushort)(ushort)4522);
            };
            SYS_STATUS p1 = new SYS_STATUS();
            PH.setPack(p1);
            p1.errors_comm = (ushort)(ushort)54425;
            p1.load = (ushort)(ushort)31637;
            p1.errors_count3 = (ushort)(ushort)4522;
            p1.onboard_control_sensors_present = (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_LOGGING |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL);
            p1.voltage_battery = (ushort)(ushort)16552;
            p1.battery_remaining = (sbyte)(sbyte) - 52;
            p1.errors_count1 = (ushort)(ushort)15133;
            p1.onboard_control_sensors_enabled = (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_GPS |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_AHRS |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE);
            p1.current_battery = (short)(short) -20569;
            p1.onboard_control_sensors_health = (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO2 |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_GPS |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_BATTERY |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE);
            p1.errors_count4 = (ushort)(ushort)22988;
            p1.errors_count2 = (ushort)(ushort)37268;
            p1.drop_rate_comm = (ushort)(ushort)41165;
            SMP_TEST_CH.send(p1);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSYSTEM_TIMEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)1286726711U);
                Debug.Assert(pack.time_unix_usec == (ulong)9107615540322272746L);
            };
            SYSTEM_TIME p2 = new SYSTEM_TIME();
            PH.setPack(p2);
            p2.time_unix_usec = (ulong)9107615540322272746L;
            p2.time_boot_ms = (uint)1286726711U;
            SMP_TEST_CH.send(p2);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPOSITION_TARGET_LOCAL_NEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.x == (float) -3.198967E38F);
                Debug.Assert(pack.afy == (float)1.4431647E38F);
                Debug.Assert(pack.z == (float) -3.3047716E38F);
                Debug.Assert(pack.afx == (float)1.7572306E38F);
                Debug.Assert(pack.y == (float) -2.0541306E38F);
                Debug.Assert(pack.yaw == (float)1.5181603E38F);
                Debug.Assert(pack.vz == (float) -1.24344E38F);
                Debug.Assert(pack.afz == (float)1.574233E38F);
                Debug.Assert(pack.vx == (float)2.2661942E38F);
                Debug.Assert(pack.type_mask == (ushort)(ushort)35303);
                Debug.Assert(pack.time_boot_ms == (uint)524963689U);
                Debug.Assert(pack.vy == (float)4.363026E37F);
                Debug.Assert(pack.coordinate_frame == MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT);
                Debug.Assert(pack.yaw_rate == (float) -2.0435112E37F);
            };
            POSITION_TARGET_LOCAL_NED p3 = new POSITION_TARGET_LOCAL_NED();
            PH.setPack(p3);
            p3.afx = (float)1.7572306E38F;
            p3.x = (float) -3.198967E38F;
            p3.afz = (float)1.574233E38F;
            p3.yaw = (float)1.5181603E38F;
            p3.z = (float) -3.3047716E38F;
            p3.afy = (float)1.4431647E38F;
            p3.vz = (float) -1.24344E38F;
            p3.vx = (float)2.2661942E38F;
            p3.vy = (float)4.363026E37F;
            p3.type_mask = (ushort)(ushort)35303;
            p3.time_boot_ms = (uint)524963689U;
            p3.coordinate_frame = MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;
            p3.y = (float) -2.0541306E38F;
            p3.yaw_rate = (float) -2.0435112E37F;
            SMP_TEST_CH.send(p3);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPINGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)203);
                Debug.Assert(pack.seq == (uint)3226894920U);
                Debug.Assert(pack.target_component == (byte)(byte)146);
                Debug.Assert(pack.time_usec == (ulong)3450884310301018098L);
            };
            PING p4 = new PING();
            PH.setPack(p4);
            p4.time_usec = (ulong)3450884310301018098L;
            p4.target_system = (byte)(byte)203;
            p4.target_component = (byte)(byte)146;
            p4.seq = (uint)3226894920U;
            SMP_TEST_CH.send(p4);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCHANGE_OPERATOR_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.version == (byte)(byte)126);
                Debug.Assert(pack.control_request == (byte)(byte)10);
                Debug.Assert(pack.target_system == (byte)(byte)234);
                Debug.Assert(pack.passkey_LEN(ph) == 4);
                Debug.Assert(pack.passkey_TRY(ph).Equals("yenh"));
            };
            CHANGE_OPERATOR_CONTROL p5 = new CHANGE_OPERATOR_CONTROL();
            PH.setPack(p5);
            p5.passkey_SET("yenh", PH) ;
            p5.control_request = (byte)(byte)10;
            p5.target_system = (byte)(byte)234;
            p5.version = (byte)(byte)126;
            SMP_TEST_CH.send(p5);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCHANGE_OPERATOR_CONTROL_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.gcs_system_id == (byte)(byte)153);
                Debug.Assert(pack.ack == (byte)(byte)9);
                Debug.Assert(pack.control_request == (byte)(byte)155);
            };
            CHANGE_OPERATOR_CONTROL_ACK p6 = new CHANGE_OPERATOR_CONTROL_ACK();
            PH.setPack(p6);
            p6.ack = (byte)(byte)9;
            p6.gcs_system_id = (byte)(byte)153;
            p6.control_request = (byte)(byte)155;
            SMP_TEST_CH.send(p6);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnAUTH_KEYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.key_LEN(ph) == 10);
                Debug.Assert(pack.key_TRY(ph).Equals("jvogbohzfe"));
            };
            AUTH_KEY p7 = new AUTH_KEY();
            PH.setPack(p7);
            p7.key_SET("jvogbohzfe", PH) ;
            SMP_TEST_CH.send(p7);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_MODEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.base_mode == MAV_MODE.MAV_MODE_AUTO_ARMED);
                Debug.Assert(pack.custom_mode == (uint)991548267U);
                Debug.Assert(pack.target_system == (byte)(byte)38);
            };
            SET_MODE p11 = new SET_MODE();
            PH.setPack(p11);
            p11.target_system = (byte)(byte)38;
            p11.custom_mode = (uint)991548267U;
            p11.base_mode = MAV_MODE.MAV_MODE_AUTO_ARMED;
            SMP_TEST_CH.send(p11);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_REQUEST_READReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)119);
                Debug.Assert(pack.param_index == (short)(short) -17156);
                Debug.Assert(pack.target_component == (byte)(byte)244);
                Debug.Assert(pack.param_id_LEN(ph) == 3);
                Debug.Assert(pack.param_id_TRY(ph).Equals("afv"));
            };
            PARAM_REQUEST_READ p20 = new PARAM_REQUEST_READ();
            PH.setPack(p20);
            p20.param_index = (short)(short) -17156;
            p20.param_id_SET("afv", PH) ;
            p20.target_component = (byte)(byte)244;
            p20.target_system = (byte)(byte)119;
            SMP_TEST_CH.send(p20);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)212);
                Debug.Assert(pack.target_component == (byte)(byte)159);
            };
            PARAM_REQUEST_LIST p21 = new PARAM_REQUEST_LIST();
            PH.setPack(p21);
            p21.target_component = (byte)(byte)159;
            p21.target_system = (byte)(byte)212;
            SMP_TEST_CH.send(p21);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_VALUEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_type == MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT64);
                Debug.Assert(pack.param_count == (ushort)(ushort)8956);
                Debug.Assert(pack.param_index == (ushort)(ushort)46187);
                Debug.Assert(pack.param_value == (float)1.4564204E38F);
                Debug.Assert(pack.param_id_LEN(ph) == 1);
                Debug.Assert(pack.param_id_TRY(ph).Equals("w"));
            };
            PARAM_VALUE p22 = new PARAM_VALUE();
            PH.setPack(p22);
            p22.param_count = (ushort)(ushort)8956;
            p22.param_type = MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT64;
            p22.param_value = (float)1.4564204E38F;
            p22.param_id_SET("w", PH) ;
            p22.param_index = (ushort)(ushort)46187;
            SMP_TEST_CH.send(p22);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_SETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)165);
                Debug.Assert(pack.param_value == (float) -1.6479313E38F);
                Debug.Assert(pack.target_system == (byte)(byte)108);
                Debug.Assert(pack.param_type == MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT8);
                Debug.Assert(pack.param_id_LEN(ph) == 7);
                Debug.Assert(pack.param_id_TRY(ph).Equals("pohggns"));
            };
            PARAM_SET p23 = new PARAM_SET();
            PH.setPack(p23);
            p23.param_id_SET("pohggns", PH) ;
            p23.target_system = (byte)(byte)108;
            p23.param_type = MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT8;
            p23.param_value = (float) -1.6479313E38F;
            p23.target_component = (byte)(byte)165;
            SMP_TEST_CH.send(p23);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_RAW_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lat == (int)376315947);
                Debug.Assert(pack.satellites_visible == (byte)(byte)197);
                Debug.Assert(pack.alt_ellipsoid_TRY(ph) == (int)1179456469);
                Debug.Assert(pack.eph == (ushort)(ushort)53428);
                Debug.Assert(pack.fix_type == GPS_FIX_TYPE.GPS_FIX_TYPE_NO_GPS);
                Debug.Assert(pack.v_acc_TRY(ph) == (uint)2907182487U);
                Debug.Assert(pack.vel == (ushort)(ushort)13157);
                Debug.Assert(pack.alt == (int)2021116454);
                Debug.Assert(pack.vel_acc_TRY(ph) == (uint)3871217750U);
                Debug.Assert(pack.epv == (ushort)(ushort)56787);
                Debug.Assert(pack.cog == (ushort)(ushort)58681);
                Debug.Assert(pack.time_usec == (ulong)2359577176374685364L);
                Debug.Assert(pack.hdg_acc_TRY(ph) == (uint)128481677U);
                Debug.Assert(pack.h_acc_TRY(ph) == (uint)995228588U);
                Debug.Assert(pack.lon == (int) -485632645);
            };
            GPS_RAW_INT p24 = new GPS_RAW_INT();
            PH.setPack(p24);
            p24.eph = (ushort)(ushort)53428;
            p24.alt_ellipsoid_SET((int)1179456469, PH) ;
            p24.v_acc_SET((uint)2907182487U, PH) ;
            p24.hdg_acc_SET((uint)128481677U, PH) ;
            p24.lat = (int)376315947;
            p24.epv = (ushort)(ushort)56787;
            p24.cog = (ushort)(ushort)58681;
            p24.alt = (int)2021116454;
            p24.lon = (int) -485632645;
            p24.time_usec = (ulong)2359577176374685364L;
            p24.vel = (ushort)(ushort)13157;
            p24.fix_type = GPS_FIX_TYPE.GPS_FIX_TYPE_NO_GPS;
            p24.h_acc_SET((uint)995228588U, PH) ;
            p24.vel_acc_SET((uint)3871217750U, PH) ;
            p24.satellites_visible = (byte)(byte)197;
            SMP_TEST_CH.send(p24);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.satellite_prn.SequenceEqual(new byte[] {(byte)35, (byte)122, (byte)246, (byte)239, (byte)16, (byte)235, (byte)12, (byte)209, (byte)133, (byte)163, (byte)222, (byte)112, (byte)205, (byte)117, (byte)34, (byte)43, (byte)65, (byte)242, (byte)52, (byte)168}));
                Debug.Assert(pack.satellite_used.SequenceEqual(new byte[] {(byte)68, (byte)71, (byte)27, (byte)224, (byte)136, (byte)88, (byte)166, (byte)147, (byte)55, (byte)230, (byte)106, (byte)88, (byte)194, (byte)49, (byte)137, (byte)118, (byte)220, (byte)89, (byte)100, (byte)196}));
                Debug.Assert(pack.satellites_visible == (byte)(byte)184);
                Debug.Assert(pack.satellite_elevation.SequenceEqual(new byte[] {(byte)112, (byte)144, (byte)73, (byte)16, (byte)186, (byte)199, (byte)102, (byte)125, (byte)248, (byte)55, (byte)168, (byte)252, (byte)25, (byte)63, (byte)170, (byte)124, (byte)247, (byte)42, (byte)20, (byte)6}));
                Debug.Assert(pack.satellite_azimuth.SequenceEqual(new byte[] {(byte)248, (byte)153, (byte)128, (byte)212, (byte)240, (byte)179, (byte)248, (byte)202, (byte)149, (byte)61, (byte)22, (byte)254, (byte)101, (byte)168, (byte)101, (byte)4, (byte)211, (byte)62, (byte)194, (byte)46}));
                Debug.Assert(pack.satellite_snr.SequenceEqual(new byte[] {(byte)178, (byte)205, (byte)191, (byte)100, (byte)173, (byte)114, (byte)195, (byte)93, (byte)162, (byte)75, (byte)28, (byte)139, (byte)95, (byte)179, (byte)254, (byte)177, (byte)249, (byte)17, (byte)92, (byte)193}));
            };
            GPS_STATUS p25 = new GPS_STATUS();
            PH.setPack(p25);
            p25.satellites_visible = (byte)(byte)184;
            p25.satellite_azimuth_SET(new byte[] {(byte)248, (byte)153, (byte)128, (byte)212, (byte)240, (byte)179, (byte)248, (byte)202, (byte)149, (byte)61, (byte)22, (byte)254, (byte)101, (byte)168, (byte)101, (byte)4, (byte)211, (byte)62, (byte)194, (byte)46}, 0) ;
            p25.satellite_prn_SET(new byte[] {(byte)35, (byte)122, (byte)246, (byte)239, (byte)16, (byte)235, (byte)12, (byte)209, (byte)133, (byte)163, (byte)222, (byte)112, (byte)205, (byte)117, (byte)34, (byte)43, (byte)65, (byte)242, (byte)52, (byte)168}, 0) ;
            p25.satellite_snr_SET(new byte[] {(byte)178, (byte)205, (byte)191, (byte)100, (byte)173, (byte)114, (byte)195, (byte)93, (byte)162, (byte)75, (byte)28, (byte)139, (byte)95, (byte)179, (byte)254, (byte)177, (byte)249, (byte)17, (byte)92, (byte)193}, 0) ;
            p25.satellite_used_SET(new byte[] {(byte)68, (byte)71, (byte)27, (byte)224, (byte)136, (byte)88, (byte)166, (byte)147, (byte)55, (byte)230, (byte)106, (byte)88, (byte)194, (byte)49, (byte)137, (byte)118, (byte)220, (byte)89, (byte)100, (byte)196}, 0) ;
            p25.satellite_elevation_SET(new byte[] {(byte)112, (byte)144, (byte)73, (byte)16, (byte)186, (byte)199, (byte)102, (byte)125, (byte)248, (byte)55, (byte)168, (byte)252, (byte)25, (byte)63, (byte)170, (byte)124, (byte)247, (byte)42, (byte)20, (byte)6}, 0) ;
            SMP_TEST_CH.send(p25);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_IMUReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.zmag == (short)(short) -28247);
                Debug.Assert(pack.xacc == (short)(short) -16871);
                Debug.Assert(pack.zgyro == (short)(short) -7518);
                Debug.Assert(pack.time_boot_ms == (uint)2619552090U);
                Debug.Assert(pack.ymag == (short)(short)27509);
                Debug.Assert(pack.zacc == (short)(short) -26971);
                Debug.Assert(pack.yacc == (short)(short)27387);
                Debug.Assert(pack.xmag == (short)(short) -30776);
                Debug.Assert(pack.xgyro == (short)(short) -29733);
                Debug.Assert(pack.ygyro == (short)(short)11726);
            };
            SCALED_IMU p26 = new SCALED_IMU();
            PH.setPack(p26);
            p26.xmag = (short)(short) -30776;
            p26.yacc = (short)(short)27387;
            p26.zgyro = (short)(short) -7518;
            p26.xgyro = (short)(short) -29733;
            p26.xacc = (short)(short) -16871;
            p26.zacc = (short)(short) -26971;
            p26.time_boot_ms = (uint)2619552090U;
            p26.zmag = (short)(short) -28247;
            p26.ygyro = (short)(short)11726;
            p26.ymag = (short)(short)27509;
            SMP_TEST_CH.send(p26);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRAW_IMUReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.yacc == (short)(short)16014);
                Debug.Assert(pack.zacc == (short)(short)10028);
                Debug.Assert(pack.ymag == (short)(short) -13322);
                Debug.Assert(pack.time_usec == (ulong)198506634823785360L);
                Debug.Assert(pack.xgyro == (short)(short)2897);
                Debug.Assert(pack.ygyro == (short)(short) -11601);
                Debug.Assert(pack.zgyro == (short)(short) -1680);
                Debug.Assert(pack.zmag == (short)(short) -10377);
                Debug.Assert(pack.xmag == (short)(short)7530);
                Debug.Assert(pack.xacc == (short)(short) -9743);
            };
            RAW_IMU p27 = new RAW_IMU();
            PH.setPack(p27);
            p27.yacc = (short)(short)16014;
            p27.zmag = (short)(short) -10377;
            p27.ymag = (short)(short) -13322;
            p27.xacc = (short)(short) -9743;
            p27.zgyro = (short)(short) -1680;
            p27.zacc = (short)(short)10028;
            p27.xgyro = (short)(short)2897;
            p27.time_usec = (ulong)198506634823785360L;
            p27.xmag = (short)(short)7530;
            p27.ygyro = (short)(short) -11601;
            SMP_TEST_CH.send(p27);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRAW_PRESSUREReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.press_diff2 == (short)(short)1317);
                Debug.Assert(pack.time_usec == (ulong)4146678563743034258L);
                Debug.Assert(pack.temperature == (short)(short)1631);
                Debug.Assert(pack.press_diff1 == (short)(short)19918);
                Debug.Assert(pack.press_abs == (short)(short) -10283);
            };
            RAW_PRESSURE p28 = new RAW_PRESSURE();
            PH.setPack(p28);
            p28.press_diff2 = (short)(short)1317;
            p28.temperature = (short)(short)1631;
            p28.press_abs = (short)(short) -10283;
            p28.press_diff1 = (short)(short)19918;
            p28.time_usec = (ulong)4146678563743034258L;
            SMP_TEST_CH.send(p28);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_PRESSUREReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.press_diff == (float)3.0674007E38F);
                Debug.Assert(pack.time_boot_ms == (uint)3459726693U);
                Debug.Assert(pack.temperature == (short)(short)7462);
                Debug.Assert(pack.press_abs == (float) -1.953893E38F);
            };
            SCALED_PRESSURE p29 = new SCALED_PRESSURE();
            PH.setPack(p29);
            p29.press_abs = (float) -1.953893E38F;
            p29.time_boot_ms = (uint)3459726693U;
            p29.press_diff = (float)3.0674007E38F;
            p29.temperature = (short)(short)7462;
            SMP_TEST_CH.send(p29);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATTITUDEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.pitch == (float)2.215235E38F);
                Debug.Assert(pack.yaw == (float) -6.667058E37F);
                Debug.Assert(pack.yawspeed == (float)2.2104405E38F);
                Debug.Assert(pack.time_boot_ms == (uint)3440972232U);
                Debug.Assert(pack.pitchspeed == (float) -2.2945626E38F);
                Debug.Assert(pack.roll == (float)2.423373E38F);
                Debug.Assert(pack.rollspeed == (float) -2.8514928E38F);
            };
            ATTITUDE p30 = new ATTITUDE();
            PH.setPack(p30);
            p30.pitchspeed = (float) -2.2945626E38F;
            p30.pitch = (float)2.215235E38F;
            p30.rollspeed = (float) -2.8514928E38F;
            p30.time_boot_ms = (uint)3440972232U;
            p30.yawspeed = (float)2.2104405E38F;
            p30.yaw = (float) -6.667058E37F;
            p30.roll = (float)2.423373E38F;
            SMP_TEST_CH.send(p30);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATTITUDE_QUATERNIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.q3 == (float) -8.4463656E37F);
                Debug.Assert(pack.q4 == (float) -2.2371624E38F);
                Debug.Assert(pack.rollspeed == (float)1.894921E38F);
                Debug.Assert(pack.yawspeed == (float) -2.6586842E38F);
                Debug.Assert(pack.time_boot_ms == (uint)1790116033U);
                Debug.Assert(pack.pitchspeed == (float)7.4498756E37F);
                Debug.Assert(pack.q1 == (float)3.2407374E38F);
                Debug.Assert(pack.q2 == (float)8.963502E37F);
            };
            ATTITUDE_QUATERNION p31 = new ATTITUDE_QUATERNION();
            PH.setPack(p31);
            p31.pitchspeed = (float)7.4498756E37F;
            p31.rollspeed = (float)1.894921E38F;
            p31.q4 = (float) -2.2371624E38F;
            p31.time_boot_ms = (uint)1790116033U;
            p31.q2 = (float)8.963502E37F;
            p31.q1 = (float)3.2407374E38F;
            p31.yawspeed = (float) -2.6586842E38F;
            p31.q3 = (float) -8.4463656E37F;
            SMP_TEST_CH.send(p31);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOCAL_POSITION_NEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)3868808627U);
                Debug.Assert(pack.vz == (float) -1.4410286E38F);
                Debug.Assert(pack.z == (float) -1.5956765E38F);
                Debug.Assert(pack.x == (float)3.7327095E37F);
                Debug.Assert(pack.vx == (float) -4.740506E37F);
                Debug.Assert(pack.y == (float)1.8515406E38F);
                Debug.Assert(pack.vy == (float)1.6639426E38F);
            };
            LOCAL_POSITION_NED p32 = new LOCAL_POSITION_NED();
            PH.setPack(p32);
            p32.y = (float)1.8515406E38F;
            p32.vz = (float) -1.4410286E38F;
            p32.vx = (float) -4.740506E37F;
            p32.time_boot_ms = (uint)3868808627U;
            p32.x = (float)3.7327095E37F;
            p32.z = (float) -1.5956765E38F;
            p32.vy = (float)1.6639426E38F;
            SMP_TEST_CH.send(p32);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGLOBAL_POSITION_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lat == (int) -514495474);
                Debug.Assert(pack.lon == (int) -1537660277);
                Debug.Assert(pack.vy == (short)(short) -17104);
                Debug.Assert(pack.hdg == (ushort)(ushort)22724);
                Debug.Assert(pack.time_boot_ms == (uint)1508326970U);
                Debug.Assert(pack.vz == (short)(short) -12129);
                Debug.Assert(pack.alt == (int)1825850229);
                Debug.Assert(pack.vx == (short)(short)9430);
                Debug.Assert(pack.relative_alt == (int) -1913963986);
            };
            GLOBAL_POSITION_INT p33 = new GLOBAL_POSITION_INT();
            PH.setPack(p33);
            p33.relative_alt = (int) -1913963986;
            p33.vx = (short)(short)9430;
            p33.lon = (int) -1537660277;
            p33.hdg = (ushort)(ushort)22724;
            p33.lat = (int) -514495474;
            p33.alt = (int)1825850229;
            p33.vy = (short)(short) -17104;
            p33.vz = (short)(short) -12129;
            p33.time_boot_ms = (uint)1508326970U;
            SMP_TEST_CH.send(p33);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRC_CHANNELS_SCALEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.rssi == (byte)(byte)134);
                Debug.Assert(pack.port == (byte)(byte)116);
                Debug.Assert(pack.chan5_scaled == (short)(short) -22964);
                Debug.Assert(pack.chan7_scaled == (short)(short)18456);
                Debug.Assert(pack.chan3_scaled == (short)(short) -29694);
                Debug.Assert(pack.chan8_scaled == (short)(short) -3363);
                Debug.Assert(pack.chan6_scaled == (short)(short)6835);
                Debug.Assert(pack.chan2_scaled == (short)(short)14551);
                Debug.Assert(pack.chan4_scaled == (short)(short) -14088);
                Debug.Assert(pack.chan1_scaled == (short)(short) -7158);
                Debug.Assert(pack.time_boot_ms == (uint)1704523491U);
            };
            RC_CHANNELS_SCALED p34 = new RC_CHANNELS_SCALED();
            PH.setPack(p34);
            p34.rssi = (byte)(byte)134;
            p34.chan7_scaled = (short)(short)18456;
            p34.port = (byte)(byte)116;
            p34.time_boot_ms = (uint)1704523491U;
            p34.chan3_scaled = (short)(short) -29694;
            p34.chan4_scaled = (short)(short) -14088;
            p34.chan5_scaled = (short)(short) -22964;
            p34.chan1_scaled = (short)(short) -7158;
            p34.chan2_scaled = (short)(short)14551;
            p34.chan6_scaled = (short)(short)6835;
            p34.chan8_scaled = (short)(short) -3363;
            SMP_TEST_CH.send(p34);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRC_CHANNELS_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)21058);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)59264);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)28328);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)5584);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)22784);
                Debug.Assert(pack.time_boot_ms == (uint)581226713U);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)57762);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)8644);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)18796);
                Debug.Assert(pack.port == (byte)(byte)185);
                Debug.Assert(pack.rssi == (byte)(byte)97);
            };
            RC_CHANNELS_RAW p35 = new RC_CHANNELS_RAW();
            PH.setPack(p35);
            p35.chan4_raw = (ushort)(ushort)59264;
            p35.chan2_raw = (ushort)(ushort)21058;
            p35.chan7_raw = (ushort)(ushort)22784;
            p35.chan1_raw = (ushort)(ushort)28328;
            p35.time_boot_ms = (uint)581226713U;
            p35.chan5_raw = (ushort)(ushort)5584;
            p35.chan3_raw = (ushort)(ushort)8644;
            p35.port = (byte)(byte)185;
            p35.rssi = (byte)(byte)97;
            p35.chan8_raw = (ushort)(ushort)18796;
            p35.chan6_raw = (ushort)(ushort)57762;
            SMP_TEST_CH.send(p35);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSERVO_OUTPUT_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.servo12_raw_TRY(ph) == (ushort)(ushort)20735);
                Debug.Assert(pack.port == (byte)(byte)109);
                Debug.Assert(pack.servo10_raw_TRY(ph) == (ushort)(ushort)24816);
                Debug.Assert(pack.servo8_raw == (ushort)(ushort)36023);
                Debug.Assert(pack.servo13_raw_TRY(ph) == (ushort)(ushort)53479);
                Debug.Assert(pack.servo15_raw_TRY(ph) == (ushort)(ushort)43987);
                Debug.Assert(pack.servo5_raw == (ushort)(ushort)29711);
                Debug.Assert(pack.time_usec == (uint)3479594794U);
                Debug.Assert(pack.servo2_raw == (ushort)(ushort)56915);
                Debug.Assert(pack.servo3_raw == (ushort)(ushort)56303);
                Debug.Assert(pack.servo9_raw_TRY(ph) == (ushort)(ushort)55273);
                Debug.Assert(pack.servo16_raw_TRY(ph) == (ushort)(ushort)20942);
                Debug.Assert(pack.servo4_raw == (ushort)(ushort)9913);
                Debug.Assert(pack.servo11_raw_TRY(ph) == (ushort)(ushort)31934);
                Debug.Assert(pack.servo14_raw_TRY(ph) == (ushort)(ushort)18159);
                Debug.Assert(pack.servo7_raw == (ushort)(ushort)39189);
                Debug.Assert(pack.servo1_raw == (ushort)(ushort)13061);
                Debug.Assert(pack.servo6_raw == (ushort)(ushort)52295);
            };
            SERVO_OUTPUT_RAW p36 = new SERVO_OUTPUT_RAW();
            PH.setPack(p36);
            p36.port = (byte)(byte)109;
            p36.time_usec = (uint)3479594794U;
            p36.servo16_raw_SET((ushort)(ushort)20942, PH) ;
            p36.servo14_raw_SET((ushort)(ushort)18159, PH) ;
            p36.servo15_raw_SET((ushort)(ushort)43987, PH) ;
            p36.servo9_raw_SET((ushort)(ushort)55273, PH) ;
            p36.servo11_raw_SET((ushort)(ushort)31934, PH) ;
            p36.servo1_raw = (ushort)(ushort)13061;
            p36.servo5_raw = (ushort)(ushort)29711;
            p36.servo8_raw = (ushort)(ushort)36023;
            p36.servo2_raw = (ushort)(ushort)56915;
            p36.servo10_raw_SET((ushort)(ushort)24816, PH) ;
            p36.servo4_raw = (ushort)(ushort)9913;
            p36.servo12_raw_SET((ushort)(ushort)20735, PH) ;
            p36.servo7_raw = (ushort)(ushort)39189;
            p36.servo6_raw = (ushort)(ushort)52295;
            p36.servo13_raw_SET((ushort)(ushort)53479, PH) ;
            p36.servo3_raw = (ushort)(ushort)56303;
            SMP_TEST_CH.send(p36);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_REQUEST_PARTIAL_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.start_index == (short)(short) -31555);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
                Debug.Assert(pack.target_component == (byte)(byte)59);
                Debug.Assert(pack.target_system == (byte)(byte)136);
                Debug.Assert(pack.end_index == (short)(short) -30173);
            };
            MISSION_REQUEST_PARTIAL_LIST p37 = new MISSION_REQUEST_PARTIAL_LIST();
            PH.setPack(p37);
            p37.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE;
            p37.target_component = (byte)(byte)59;
            p37.end_index = (short)(short) -30173;
            p37.start_index = (short)(short) -31555;
            p37.target_system = (byte)(byte)136;
            SMP_TEST_CH.send(p37);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_WRITE_PARTIAL_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
                Debug.Assert(pack.target_system == (byte)(byte)12);
                Debug.Assert(pack.target_component == (byte)(byte)128);
                Debug.Assert(pack.start_index == (short)(short) -23427);
                Debug.Assert(pack.end_index == (short)(short) -15362);
            };
            MISSION_WRITE_PARTIAL_LIST p38 = new MISSION_WRITE_PARTIAL_LIST();
            PH.setPack(p38);
            p38.end_index = (short)(short) -15362;
            p38.target_component = (byte)(byte)128;
            p38.start_index = (short)(short) -23427;
            p38.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY;
            p38.target_system = (byte)(byte)12;
            SMP_TEST_CH.send(p38);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_ITEMReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param1 == (float) -2.2593805E38F);
                Debug.Assert(pack.param3 == (float) -3.1143119E38F);
                Debug.Assert(pack.y == (float) -2.784129E38F);
                Debug.Assert(pack.seq == (ushort)(ushort)16155);
                Debug.Assert(pack.param4 == (float)2.652912E38F);
                Debug.Assert(pack.target_system == (byte)(byte)188);
                Debug.Assert(pack.param2 == (float) -2.9899448E38F);
                Debug.Assert(pack.frame == MAV_FRAME.MAV_FRAME_GLOBAL);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
                Debug.Assert(pack.x == (float)1.8474385E38F);
                Debug.Assert(pack.autocontinue == (byte)(byte)138);
                Debug.Assert(pack.command == MAV_CMD.MAV_CMD_SET_CAMERA_MODE);
                Debug.Assert(pack.z == (float)1.1585383E38F);
                Debug.Assert(pack.current == (byte)(byte)3);
                Debug.Assert(pack.target_component == (byte)(byte)123);
            };
            MISSION_ITEM p39 = new MISSION_ITEM();
            PH.setPack(p39);
            p39.autocontinue = (byte)(byte)138;
            p39.y = (float) -2.784129E38F;
            p39.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY;
            p39.current = (byte)(byte)3;
            p39.param1 = (float) -2.2593805E38F;
            p39.target_component = (byte)(byte)123;
            p39.x = (float)1.8474385E38F;
            p39.param4 = (float)2.652912E38F;
            p39.command = MAV_CMD.MAV_CMD_SET_CAMERA_MODE;
            p39.frame = MAV_FRAME.MAV_FRAME_GLOBAL;
            p39.target_system = (byte)(byte)188;
            p39.param3 = (float) -3.1143119E38F;
            p39.z = (float)1.1585383E38F;
            p39.param2 = (float) -2.9899448E38F;
            p39.seq = (ushort)(ushort)16155;
            SMP_TEST_CH.send(p39);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
                Debug.Assert(pack.target_system == (byte)(byte)151);
                Debug.Assert(pack.seq == (ushort)(ushort)17675);
                Debug.Assert(pack.target_component == (byte)(byte)159);
            };
            MISSION_REQUEST p40 = new MISSION_REQUEST();
            PH.setPack(p40);
            p40.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL;
            p40.target_component = (byte)(byte)159;
            p40.target_system = (byte)(byte)151;
            p40.seq = (ushort)(ushort)17675;
            SMP_TEST_CH.send(p40);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_SET_CURRENTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)154);
                Debug.Assert(pack.target_component == (byte)(byte)246);
                Debug.Assert(pack.seq == (ushort)(ushort)18817);
            };
            MISSION_SET_CURRENT p41 = new MISSION_SET_CURRENT();
            PH.setPack(p41);
            p41.seq = (ushort)(ushort)18817;
            p41.target_system = (byte)(byte)154;
            p41.target_component = (byte)(byte)246;
            SMP_TEST_CH.send(p41);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_CURRENTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (ushort)(ushort)31631);
            };
            MISSION_CURRENT p42 = new MISSION_CURRENT();
            PH.setPack(p42);
            p42.seq = (ushort)(ushort)31631;
            SMP_TEST_CH.send(p42);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
                Debug.Assert(pack.target_component == (byte)(byte)14);
                Debug.Assert(pack.target_system == (byte)(byte)101);
            };
            MISSION_REQUEST_LIST p43 = new MISSION_REQUEST_LIST();
            PH.setPack(p43);
            p43.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION;
            p43.target_component = (byte)(byte)14;
            p43.target_system = (byte)(byte)101;
            SMP_TEST_CH.send(p43);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_COUNTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.count == (ushort)(ushort)20445);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
                Debug.Assert(pack.target_system == (byte)(byte)41);
                Debug.Assert(pack.target_component == (byte)(byte)146);
            };
            MISSION_COUNT p44 = new MISSION_COUNT();
            PH.setPack(p44);
            p44.target_system = (byte)(byte)41;
            p44.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY;
            p44.count = (ushort)(ushort)20445;
            p44.target_component = (byte)(byte)146;
            SMP_TEST_CH.send(p44);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_CLEAR_ALLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)78);
                Debug.Assert(pack.target_component == (byte)(byte)169);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
            };
            MISSION_CLEAR_ALL p45 = new MISSION_CLEAR_ALL();
            PH.setPack(p45);
            p45.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION;
            p45.target_component = (byte)(byte)169;
            p45.target_system = (byte)(byte)78;
            SMP_TEST_CH.send(p45);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_ITEM_REACHEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (ushort)(ushort)46965);
            };
            MISSION_ITEM_REACHED p46 = new MISSION_ITEM_REACHED();
            PH.setPack(p46);
            p46.seq = (ushort)(ushort)46965;
            SMP_TEST_CH.send(p46);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.type == MAV_MISSION_RESULT.MAV_MISSION_INVALID_PARAM6_Y);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
                Debug.Assert(pack.target_component == (byte)(byte)255);
                Debug.Assert(pack.target_system == (byte)(byte)17);
            };
            MISSION_ACK p47 = new MISSION_ACK();
            PH.setPack(p47);
            p47.target_component = (byte)(byte)255;
            p47.type = MAV_MISSION_RESULT.MAV_MISSION_INVALID_PARAM6_Y;
            p47.target_system = (byte)(byte)17;
            p47.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL;
            SMP_TEST_CH.send(p47);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_GPS_GLOBAL_ORIGINReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)169);
                Debug.Assert(pack.longitude == (int)1919817256);
                Debug.Assert(pack.latitude == (int)478607761);
                Debug.Assert(pack.altitude == (int) -1611519377);
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)2486801012879551034L);
            };
            SET_GPS_GLOBAL_ORIGIN p48 = new SET_GPS_GLOBAL_ORIGIN();
            PH.setPack(p48);
            p48.time_usec_SET((ulong)2486801012879551034L, PH) ;
            p48.altitude = (int) -1611519377;
            p48.latitude = (int)478607761;
            p48.target_system = (byte)(byte)169;
            p48.longitude = (int)1919817256;
            SMP_TEST_CH.send(p48);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_GLOBAL_ORIGINReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.longitude == (int) -232578909);
                Debug.Assert(pack.latitude == (int) -1951484880);
                Debug.Assert(pack.altitude == (int) -838874868);
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)8735354368730287133L);
            };
            GPS_GLOBAL_ORIGIN p49 = new GPS_GLOBAL_ORIGIN();
            PH.setPack(p49);
            p49.time_usec_SET((ulong)8735354368730287133L, PH) ;
            p49.longitude = (int) -232578909;
            p49.altitude = (int) -838874868;
            p49.latitude = (int) -1951484880;
            SMP_TEST_CH.send(p49);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_MAP_RCReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)22);
                Debug.Assert(pack.param_value_min == (float) -2.5634144E38F);
                Debug.Assert(pack.param_id_LEN(ph) == 14);
                Debug.Assert(pack.param_id_TRY(ph).Equals("efsfwaymufwHug"));
                Debug.Assert(pack.scale == (float)1.6471395E38F);
                Debug.Assert(pack.param_value_max == (float) -8.982579E36F);
                Debug.Assert(pack.target_system == (byte)(byte)49);
                Debug.Assert(pack.parameter_rc_channel_index == (byte)(byte)115);
                Debug.Assert(pack.param_index == (short)(short) -17535);
                Debug.Assert(pack.param_value0 == (float)1.511091E38F);
            };
            PARAM_MAP_RC p50 = new PARAM_MAP_RC();
            PH.setPack(p50);
            p50.target_system = (byte)(byte)49;
            p50.param_id_SET("efsfwaymufwHug", PH) ;
            p50.target_component = (byte)(byte)22;
            p50.param_value_min = (float) -2.5634144E38F;
            p50.scale = (float)1.6471395E38F;
            p50.parameter_rc_channel_index = (byte)(byte)115;
            p50.param_index = (short)(short) -17535;
            p50.param_value0 = (float)1.511091E38F;
            p50.param_value_max = (float) -8.982579E36F;
            SMP_TEST_CH.send(p50);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_REQUEST_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (ushort)(ushort)52161);
                Debug.Assert(pack.target_component == (byte)(byte)86);
                Debug.Assert(pack.target_system == (byte)(byte)70);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
            };
            MISSION_REQUEST_INT p51 = new MISSION_REQUEST_INT();
            PH.setPack(p51);
            p51.target_system = (byte)(byte)70;
            p51.target_component = (byte)(byte)86;
            p51.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION;
            p51.seq = (ushort)(ushort)52161;
            SMP_TEST_CH.send(p51);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSAFETY_SET_ALLOWED_AREAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.p2z == (float)2.3492659E37F);
                Debug.Assert(pack.frame == MAV_FRAME.MAV_FRAME_LOCAL_NED);
                Debug.Assert(pack.p2x == (float)1.597338E38F);
                Debug.Assert(pack.p1y == (float) -2.630987E38F);
                Debug.Assert(pack.target_component == (byte)(byte)3);
                Debug.Assert(pack.p1x == (float)2.7881715E38F);
                Debug.Assert(pack.p2y == (float) -7.125794E36F);
                Debug.Assert(pack.target_system == (byte)(byte)157);
                Debug.Assert(pack.p1z == (float) -1.6389596E38F);
            };
            SAFETY_SET_ALLOWED_AREA p54 = new SAFETY_SET_ALLOWED_AREA();
            PH.setPack(p54);
            p54.p1z = (float) -1.6389596E38F;
            p54.frame = MAV_FRAME.MAV_FRAME_LOCAL_NED;
            p54.p2z = (float)2.3492659E37F;
            p54.target_component = (byte)(byte)3;
            p54.p2y = (float) -7.125794E36F;
            p54.p1x = (float)2.7881715E38F;
            p54.p2x = (float)1.597338E38F;
            p54.target_system = (byte)(byte)157;
            p54.p1y = (float) -2.630987E38F;
            SMP_TEST_CH.send(p54);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSAFETY_ALLOWED_AREAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.frame == MAV_FRAME.MAV_FRAME_GLOBAL);
                Debug.Assert(pack.p1z == (float)1.9783509E38F);
                Debug.Assert(pack.p1y == (float)2.3429596E37F);
                Debug.Assert(pack.p2y == (float)3.1136853E38F);
                Debug.Assert(pack.p2z == (float) -2.870769E38F);
                Debug.Assert(pack.p1x == (float) -2.1053852E37F);
                Debug.Assert(pack.p2x == (float)2.6428442E38F);
            };
            SAFETY_ALLOWED_AREA p55 = new SAFETY_ALLOWED_AREA();
            PH.setPack(p55);
            p55.p1z = (float)1.9783509E38F;
            p55.p2z = (float) -2.870769E38F;
            p55.p2x = (float)2.6428442E38F;
            p55.p2y = (float)3.1136853E38F;
            p55.frame = MAV_FRAME.MAV_FRAME_GLOBAL;
            p55.p1x = (float) -2.1053852E37F;
            p55.p1y = (float)2.3429596E37F;
            SMP_TEST_CH.send(p55);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATTITUDE_QUATERNION_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.covariance.SequenceEqual(new float[] {2.488078E38F, -1.6778232E38F, 2.078556E38F, -2.2702892E38F, -7.3702915E37F, 2.851148E38F, 2.0273203E38F, -3.317172E38F, -3.1386102E38F}));
                Debug.Assert(pack.time_usec == (ulong)5122208568366972041L);
                Debug.Assert(pack.pitchspeed == (float) -9.593744E37F);
                Debug.Assert(pack.yawspeed == (float) -2.4620468E38F);
                Debug.Assert(pack.rollspeed == (float)2.536877E38F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {3.4510256E37F, 2.4838891E38F, -2.8257372E38F, 4.360056E37F}));
            };
            ATTITUDE_QUATERNION_COV p61 = new ATTITUDE_QUATERNION_COV();
            PH.setPack(p61);
            p61.yawspeed = (float) -2.4620468E38F;
            p61.covariance_SET(new float[] {2.488078E38F, -1.6778232E38F, 2.078556E38F, -2.2702892E38F, -7.3702915E37F, 2.851148E38F, 2.0273203E38F, -3.317172E38F, -3.1386102E38F}, 0) ;
            p61.rollspeed = (float)2.536877E38F;
            p61.q_SET(new float[] {3.4510256E37F, 2.4838891E38F, -2.8257372E38F, 4.360056E37F}, 0) ;
            p61.pitchspeed = (float) -9.593744E37F;
            p61.time_usec = (ulong)5122208568366972041L;
            SMP_TEST_CH.send(p61);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnNAV_CONTROLLER_OUTPUTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_bearing == (short)(short) -16932);
                Debug.Assert(pack.aspd_error == (float) -2.8430379E38F);
                Debug.Assert(pack.nav_pitch == (float)2.1223816E38F);
                Debug.Assert(pack.xtrack_error == (float)3.024377E38F);
                Debug.Assert(pack.wp_dist == (ushort)(ushort)59184);
                Debug.Assert(pack.nav_bearing == (short)(short)2117);
                Debug.Assert(pack.alt_error == (float) -2.3640814E38F);
                Debug.Assert(pack.nav_roll == (float)1.1045312E38F);
            };
            NAV_CONTROLLER_OUTPUT p62 = new NAV_CONTROLLER_OUTPUT();
            PH.setPack(p62);
            p62.nav_pitch = (float)2.1223816E38F;
            p62.alt_error = (float) -2.3640814E38F;
            p62.wp_dist = (ushort)(ushort)59184;
            p62.target_bearing = (short)(short) -16932;
            p62.xtrack_error = (float)3.024377E38F;
            p62.nav_roll = (float)1.1045312E38F;
            p62.aspd_error = (float) -2.8430379E38F;
            p62.nav_bearing = (short)(short)2117;
            SMP_TEST_CH.send(p62);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGLOBAL_POSITION_INT_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lat == (int)1744438289);
                Debug.Assert(pack.time_usec == (ulong)2285438602282984450L);
                Debug.Assert(pack.estimator_type == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_NAIVE);
                Debug.Assert(pack.relative_alt == (int) -243512527);
                Debug.Assert(pack.covariance.SequenceEqual(new float[] {1.9964418E38F, 2.7695425E38F, -1.7693476E38F, -2.669125E38F, -2.477825E38F, 1.1528638E38F, -1.6569136E38F, -2.2344486E38F, -7.5389235E37F, 1.1579037E38F, -2.1208318E38F, 2.86907E38F, -6.21896E37F, 1.2757799E38F, -1.369239E38F, -2.1386754E38F, -3.101383E38F, 5.863406E37F, -1.0664387E38F, -1.7025576E36F, 6.0750005E37F, -1.9260748E38F, -1.4887211E37F, -4.963421E37F, -3.0160146E38F, 1.2683829E38F, -2.0402439E38F, -2.7078153E38F, 2.3917999E37F, 1.5952882E38F, 9.842022E37F, -9.631772E37F, 1.6240866E38F, 2.9964435E38F, 2.8852066E38F, -2.0700542E38F}));
                Debug.Assert(pack.vx == (float) -2.8404182E38F);
                Debug.Assert(pack.alt == (int)1163164197);
                Debug.Assert(pack.lon == (int) -803567944);
                Debug.Assert(pack.vy == (float)1.3890833E37F);
                Debug.Assert(pack.vz == (float) -1.0956371E38F);
            };
            GLOBAL_POSITION_INT_COV p63 = new GLOBAL_POSITION_INT_COV();
            PH.setPack(p63);
            p63.alt = (int)1163164197;
            p63.relative_alt = (int) -243512527;
            p63.time_usec = (ulong)2285438602282984450L;
            p63.lon = (int) -803567944;
            p63.vy = (float)1.3890833E37F;
            p63.estimator_type = MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_NAIVE;
            p63.covariance_SET(new float[] {1.9964418E38F, 2.7695425E38F, -1.7693476E38F, -2.669125E38F, -2.477825E38F, 1.1528638E38F, -1.6569136E38F, -2.2344486E38F, -7.5389235E37F, 1.1579037E38F, -2.1208318E38F, 2.86907E38F, -6.21896E37F, 1.2757799E38F, -1.369239E38F, -2.1386754E38F, -3.101383E38F, 5.863406E37F, -1.0664387E38F, -1.7025576E36F, 6.0750005E37F, -1.9260748E38F, -1.4887211E37F, -4.963421E37F, -3.0160146E38F, 1.2683829E38F, -2.0402439E38F, -2.7078153E38F, 2.3917999E37F, 1.5952882E38F, 9.842022E37F, -9.631772E37F, 1.6240866E38F, 2.9964435E38F, 2.8852066E38F, -2.0700542E38F}, 0) ;
            p63.vx = (float) -2.8404182E38F;
            p63.vz = (float) -1.0956371E38F;
            p63.lat = (int)1744438289;
            SMP_TEST_CH.send(p63);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOCAL_POSITION_NED_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vy == (float)1.9693366E38F);
                Debug.Assert(pack.ay == (float) -1.5470074E38F);
                Debug.Assert(pack.vx == (float)3.3356353E38F);
                Debug.Assert(pack.z == (float)1.1232583E38F);
                Debug.Assert(pack.az == (float) -2.7427716E38F);
                Debug.Assert(pack.estimator_type == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VISION);
                Debug.Assert(pack.vz == (float) -3.3484275E38F);
                Debug.Assert(pack.time_usec == (ulong)3473183780583395286L);
                Debug.Assert(pack.ax == (float)6.601542E37F);
                Debug.Assert(pack.y == (float) -2.7653017E38F);
                Debug.Assert(pack.x == (float) -2.0976723E38F);
                Debug.Assert(pack.covariance.SequenceEqual(new float[] {-1.899704E37F, -2.8603864E38F, -1.7429816E38F, 2.2586623E38F, 8.346998E37F, 1.6695296E37F, -1.0774929E37F, 1.8459595E38F, 6.9698675E37F, 5.2372767E37F, 2.7728816E38F, 1.0715074E38F, -2.4977534E38F, -2.7868753E38F, -3.1617942E38F, -1.093876E38F, -4.525877E37F, 4.4675835E37F, 4.588509E36F, 1.9872474E38F, 1.7770054E38F, 6.2447796E36F, 3.213285E38F, -2.5380048E38F, -1.9016876E38F, -2.9238373E38F, 2.3335608E38F, -2.6920343E38F, -2.700055E36F, 7.199995E36F, -2.4810969E38F, -3.3941367E38F, 1.7664738E37F, 2.603461E38F, -2.729562E37F, 2.2529897E38F, -3.1088689E38F, -1.9677941E38F, -4.664957E37F, -1.9571737E37F, 2.2446786E38F, -2.0624996E38F, 8.0595197E37F, -1.749438E37F, -9.92784E37F}));
            };
            LOCAL_POSITION_NED_COV p64 = new LOCAL_POSITION_NED_COV();
            PH.setPack(p64);
            p64.ax = (float)6.601542E37F;
            p64.z = (float)1.1232583E38F;
            p64.x = (float) -2.0976723E38F;
            p64.time_usec = (ulong)3473183780583395286L;
            p64.y = (float) -2.7653017E38F;
            p64.ay = (float) -1.5470074E38F;
            p64.vz = (float) -3.3484275E38F;
            p64.vx = (float)3.3356353E38F;
            p64.az = (float) -2.7427716E38F;
            p64.estimator_type = MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VISION;
            p64.covariance_SET(new float[] {-1.899704E37F, -2.8603864E38F, -1.7429816E38F, 2.2586623E38F, 8.346998E37F, 1.6695296E37F, -1.0774929E37F, 1.8459595E38F, 6.9698675E37F, 5.2372767E37F, 2.7728816E38F, 1.0715074E38F, -2.4977534E38F, -2.7868753E38F, -3.1617942E38F, -1.093876E38F, -4.525877E37F, 4.4675835E37F, 4.588509E36F, 1.9872474E38F, 1.7770054E38F, 6.2447796E36F, 3.213285E38F, -2.5380048E38F, -1.9016876E38F, -2.9238373E38F, 2.3335608E38F, -2.6920343E38F, -2.700055E36F, 7.199995E36F, -2.4810969E38F, -3.3941367E38F, 1.7664738E37F, 2.603461E38F, -2.729562E37F, 2.2529897E38F, -3.1088689E38F, -1.9677941E38F, -4.664957E37F, -1.9571737E37F, 2.2446786E38F, -2.0624996E38F, 8.0595197E37F, -1.749438E37F, -9.92784E37F}, 0) ;
            p64.vy = (float)1.9693366E38F;
            SMP_TEST_CH.send(p64);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRC_CHANNELSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan14_raw == (ushort)(ushort)10240);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)4733);
                Debug.Assert(pack.chancount == (byte)(byte)204);
                Debug.Assert(pack.rssi == (byte)(byte)15);
                Debug.Assert(pack.chan17_raw == (ushort)(ushort)46799);
                Debug.Assert(pack.chan11_raw == (ushort)(ushort)18647);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)15915);
                Debug.Assert(pack.chan9_raw == (ushort)(ushort)61782);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)61162);
                Debug.Assert(pack.chan13_raw == (ushort)(ushort)50615);
                Debug.Assert(pack.chan12_raw == (ushort)(ushort)51654);
                Debug.Assert(pack.chan18_raw == (ushort)(ushort)42275);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)44943);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)60524);
                Debug.Assert(pack.chan16_raw == (ushort)(ushort)49491);
                Debug.Assert(pack.chan15_raw == (ushort)(ushort)64713);
                Debug.Assert(pack.chan10_raw == (ushort)(ushort)30977);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)2681);
                Debug.Assert(pack.time_boot_ms == (uint)2480953570U);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)15863);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)62966);
            };
            RC_CHANNELS p65 = new RC_CHANNELS();
            PH.setPack(p65);
            p65.chan3_raw = (ushort)(ushort)44943;
            p65.chan6_raw = (ushort)(ushort)60524;
            p65.chan8_raw = (ushort)(ushort)2681;
            p65.chan10_raw = (ushort)(ushort)30977;
            p65.chan2_raw = (ushort)(ushort)61162;
            p65.chan12_raw = (ushort)(ushort)51654;
            p65.chan9_raw = (ushort)(ushort)61782;
            p65.chan16_raw = (ushort)(ushort)49491;
            p65.chancount = (byte)(byte)204;
            p65.chan7_raw = (ushort)(ushort)4733;
            p65.chan17_raw = (ushort)(ushort)46799;
            p65.rssi = (byte)(byte)15;
            p65.chan13_raw = (ushort)(ushort)50615;
            p65.chan15_raw = (ushort)(ushort)64713;
            p65.chan14_raw = (ushort)(ushort)10240;
            p65.chan18_raw = (ushort)(ushort)42275;
            p65.chan11_raw = (ushort)(ushort)18647;
            p65.chan5_raw = (ushort)(ushort)15915;
            p65.chan4_raw = (ushort)(ushort)62966;
            p65.time_boot_ms = (uint)2480953570U;
            p65.chan1_raw = (ushort)(ushort)15863;
            SMP_TEST_CH.send(p65);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnREQUEST_DATA_STREAMReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)131);
                Debug.Assert(pack.start_stop == (byte)(byte)47);
                Debug.Assert(pack.req_stream_id == (byte)(byte)134);
                Debug.Assert(pack.req_message_rate == (ushort)(ushort)60551);
                Debug.Assert(pack.target_component == (byte)(byte)42);
            };
            REQUEST_DATA_STREAM p66 = new REQUEST_DATA_STREAM();
            PH.setPack(p66);
            p66.target_component = (byte)(byte)42;
            p66.req_message_rate = (ushort)(ushort)60551;
            p66.req_stream_id = (byte)(byte)134;
            p66.start_stop = (byte)(byte)47;
            p66.target_system = (byte)(byte)131;
            SMP_TEST_CH.send(p66);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnDATA_STREAMReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.on_off == (byte)(byte)185);
                Debug.Assert(pack.message_rate == (ushort)(ushort)10336);
                Debug.Assert(pack.stream_id == (byte)(byte)160);
            };
            DATA_STREAM p67 = new DATA_STREAM();
            PH.setPack(p67);
            p67.on_off = (byte)(byte)185;
            p67.stream_id = (byte)(byte)160;
            p67.message_rate = (ushort)(ushort)10336;
            SMP_TEST_CH.send(p67);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMANUAL_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.x == (short)(short)8540);
                Debug.Assert(pack.r == (short)(short)18169);
                Debug.Assert(pack.z == (short)(short) -20537);
                Debug.Assert(pack.y == (short)(short)28273);
                Debug.Assert(pack.target == (byte)(byte)244);
                Debug.Assert(pack.buttons == (ushort)(ushort)29742);
            };
            MANUAL_CONTROL p69 = new MANUAL_CONTROL();
            PH.setPack(p69);
            p69.r = (short)(short)18169;
            p69.buttons = (ushort)(ushort)29742;
            p69.z = (short)(short) -20537;
            p69.target = (byte)(byte)244;
            p69.y = (short)(short)28273;
            p69.x = (short)(short)8540;
            SMP_TEST_CH.send(p69);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRC_CHANNELS_OVERRIDEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)53734);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)64320);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)14214);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)18543);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)21424);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)41015);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)51907);
                Debug.Assert(pack.target_component == (byte)(byte)149);
                Debug.Assert(pack.target_system == (byte)(byte)40);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)43289);
            };
            RC_CHANNELS_OVERRIDE p70 = new RC_CHANNELS_OVERRIDE();
            PH.setPack(p70);
            p70.chan5_raw = (ushort)(ushort)43289;
            p70.chan7_raw = (ushort)(ushort)14214;
            p70.chan3_raw = (ushort)(ushort)21424;
            p70.chan6_raw = (ushort)(ushort)53734;
            p70.target_system = (byte)(byte)40;
            p70.target_component = (byte)(byte)149;
            p70.chan4_raw = (ushort)(ushort)51907;
            p70.chan8_raw = (ushort)(ushort)41015;
            p70.chan2_raw = (ushort)(ushort)18543;
            p70.chan1_raw = (ushort)(ushort)64320;
            SMP_TEST_CH.send(p70);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_ITEM_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.z == (float) -3.9101448E37F);
                Debug.Assert(pack.current == (byte)(byte)60);
                Debug.Assert(pack.y == (int) -1326066384);
                Debug.Assert(pack.param3 == (float) -1.7362345E38F);
                Debug.Assert(pack.target_system == (byte)(byte)197);
                Debug.Assert(pack.autocontinue == (byte)(byte)65);
                Debug.Assert(pack.command == MAV_CMD.MAV_CMD_USER_4);
                Debug.Assert(pack.x == (int) -1682863122);
                Debug.Assert(pack.param1 == (float) -1.3022983E38F);
                Debug.Assert(pack.seq == (ushort)(ushort)33087);
                Debug.Assert(pack.target_component == (byte)(byte)102);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
                Debug.Assert(pack.param4 == (float)3.5745217E37F);
                Debug.Assert(pack.param2 == (float)7.8125266E37F);
                Debug.Assert(pack.frame == MAV_FRAME.MAV_FRAME_MISSION);
            };
            MISSION_ITEM_INT p73 = new MISSION_ITEM_INT();
            PH.setPack(p73);
            p73.param3 = (float) -1.7362345E38F;
            p73.z = (float) -3.9101448E37F;
            p73.param1 = (float) -1.3022983E38F;
            p73.seq = (ushort)(ushort)33087;
            p73.autocontinue = (byte)(byte)65;
            p73.y = (int) -1326066384;
            p73.current = (byte)(byte)60;
            p73.frame = MAV_FRAME.MAV_FRAME_MISSION;
            p73.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE;
            p73.command = MAV_CMD.MAV_CMD_USER_4;
            p73.target_system = (byte)(byte)197;
            p73.param2 = (float)7.8125266E37F;
            p73.target_component = (byte)(byte)102;
            p73.x = (int) -1682863122;
            p73.param4 = (float)3.5745217E37F;
            SMP_TEST_CH.send(p73);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnVFR_HUDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.climb == (float) -2.2722302E38F);
                Debug.Assert(pack.throttle == (ushort)(ushort)28604);
                Debug.Assert(pack.alt == (float)1.2824161E38F);
                Debug.Assert(pack.heading == (short)(short) -25403);
                Debug.Assert(pack.groundspeed == (float) -3.3858025E38F);
                Debug.Assert(pack.airspeed == (float)2.8727685E38F);
            };
            VFR_HUD p74 = new VFR_HUD();
            PH.setPack(p74);
            p74.airspeed = (float)2.8727685E38F;
            p74.heading = (short)(short) -25403;
            p74.climb = (float) -2.2722302E38F;
            p74.groundspeed = (float) -3.3858025E38F;
            p74.throttle = (ushort)(ushort)28604;
            p74.alt = (float)1.2824161E38F;
            SMP_TEST_CH.send(p74);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCOMMAND_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param4 == (float)2.6452035E37F);
                Debug.Assert(pack.param1 == (float) -3.1375113E38F);
                Debug.Assert(pack.param2 == (float) -1.5730155E38F);
                Debug.Assert(pack.z == (float)2.5686491E38F);
                Debug.Assert(pack.command == MAV_CMD.MAV_CMD_CONDITION_LAST);
                Debug.Assert(pack.current == (byte)(byte)78);
                Debug.Assert(pack.param3 == (float) -2.8551037E38F);
                Debug.Assert(pack.target_component == (byte)(byte)186);
                Debug.Assert(pack.y == (int) -906327668);
                Debug.Assert(pack.frame == MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT);
                Debug.Assert(pack.target_system == (byte)(byte)224);
                Debug.Assert(pack.x == (int)87921314);
                Debug.Assert(pack.autocontinue == (byte)(byte)120);
            };
            COMMAND_INT p75 = new COMMAND_INT();
            PH.setPack(p75);
            p75.y = (int) -906327668;
            p75.param2 = (float) -1.5730155E38F;
            p75.frame = MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT;
            p75.param3 = (float) -2.8551037E38F;
            p75.target_component = (byte)(byte)186;
            p75.param4 = (float)2.6452035E37F;
            p75.z = (float)2.5686491E38F;
            p75.autocontinue = (byte)(byte)120;
            p75.current = (byte)(byte)78;
            p75.command = MAV_CMD.MAV_CMD_CONDITION_LAST;
            p75.param1 = (float) -3.1375113E38F;
            p75.x = (int)87921314;
            p75.target_system = (byte)(byte)224;
            SMP_TEST_CH.send(p75);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCOMMAND_LONGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param7 == (float)1.597059E38F);
                Debug.Assert(pack.param4 == (float)3.0791765E37F);
                Debug.Assert(pack.param5 == (float)9.246787E37F);
                Debug.Assert(pack.confirmation == (byte)(byte)73);
                Debug.Assert(pack.param3 == (float)1.7513986E38F);
                Debug.Assert(pack.target_system == (byte)(byte)218);
                Debug.Assert(pack.target_component == (byte)(byte)172);
                Debug.Assert(pack.command == MAV_CMD.MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS);
                Debug.Assert(pack.param6 == (float) -1.1711716E38F);
                Debug.Assert(pack.param1 == (float) -2.7398773E38F);
                Debug.Assert(pack.param2 == (float)2.2740567E38F);
            };
            COMMAND_LONG p76 = new COMMAND_LONG();
            PH.setPack(p76);
            p76.target_component = (byte)(byte)172;
            p76.param6 = (float) -1.1711716E38F;
            p76.param2 = (float)2.2740567E38F;
            p76.target_system = (byte)(byte)218;
            p76.param3 = (float)1.7513986E38F;
            p76.param7 = (float)1.597059E38F;
            p76.param1 = (float) -2.7398773E38F;
            p76.param5 = (float)9.246787E37F;
            p76.command = MAV_CMD.MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS;
            p76.param4 = (float)3.0791765E37F;
            p76.confirmation = (byte)(byte)73;
            SMP_TEST_CH.send(p76);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCOMMAND_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.result_param2_TRY(ph) == (int) -743177092);
                Debug.Assert(pack.result == MAV_RESULT.MAV_RESULT_TEMPORARILY_REJECTED);
                Debug.Assert(pack.target_system_TRY(ph) == (byte)(byte)116);
                Debug.Assert(pack.target_component_TRY(ph) == (byte)(byte)63);
                Debug.Assert(pack.progress_TRY(ph) == (byte)(byte)246);
                Debug.Assert(pack.command == MAV_CMD.MAV_CMD_NAV_VTOL_LAND);
            };
            COMMAND_ACK p77 = new COMMAND_ACK();
            PH.setPack(p77);
            p77.target_component_SET((byte)(byte)63, PH) ;
            p77.progress_SET((byte)(byte)246, PH) ;
            p77.target_system_SET((byte)(byte)116, PH) ;
            p77.command = MAV_CMD.MAV_CMD_NAV_VTOL_LAND;
            p77.result = MAV_RESULT.MAV_RESULT_TEMPORARILY_REJECTED;
            p77.result_param2_SET((int) -743177092, PH) ;
            SMP_TEST_CH.send(p77);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMANUAL_SETPOINTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.pitch == (float)2.8885502E38F);
                Debug.Assert(pack.mode_switch == (byte)(byte)24);
                Debug.Assert(pack.thrust == (float) -2.9333033E38F);
                Debug.Assert(pack.roll == (float)2.676834E38F);
                Debug.Assert(pack.time_boot_ms == (uint)4068160696U);
                Debug.Assert(pack.yaw == (float)1.3262786E38F);
                Debug.Assert(pack.manual_override_switch == (byte)(byte)45);
            };
            MANUAL_SETPOINT p81 = new MANUAL_SETPOINT();
            PH.setPack(p81);
            p81.pitch = (float)2.8885502E38F;
            p81.manual_override_switch = (byte)(byte)45;
            p81.roll = (float)2.676834E38F;
            p81.thrust = (float) -2.9333033E38F;
            p81.time_boot_ms = (uint)4068160696U;
            p81.yaw = (float)1.3262786E38F;
            p81.mode_switch = (byte)(byte)24;
            SMP_TEST_CH.send(p81);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_ATTITUDE_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)1838879457U);
                Debug.Assert(pack.type_mask == (byte)(byte)124);
                Debug.Assert(pack.body_roll_rate == (float)7.5281414E37F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-1.8544846E38F, -1.6861499E38F, -3.2471268E38F, -1.0392594E37F}));
                Debug.Assert(pack.target_system == (byte)(byte)233);
                Debug.Assert(pack.body_pitch_rate == (float) -2.9510249E38F);
                Debug.Assert(pack.target_component == (byte)(byte)155);
                Debug.Assert(pack.body_yaw_rate == (float)2.6014294E38F);
                Debug.Assert(pack.thrust == (float)1.883599E38F);
            };
            SET_ATTITUDE_TARGET p82 = new SET_ATTITUDE_TARGET();
            PH.setPack(p82);
            p82.q_SET(new float[] {-1.8544846E38F, -1.6861499E38F, -3.2471268E38F, -1.0392594E37F}, 0) ;
            p82.body_roll_rate = (float)7.5281414E37F;
            p82.thrust = (float)1.883599E38F;
            p82.target_system = (byte)(byte)233;
            p82.type_mask = (byte)(byte)124;
            p82.body_pitch_rate = (float) -2.9510249E38F;
            p82.body_yaw_rate = (float)2.6014294E38F;
            p82.target_component = (byte)(byte)155;
            p82.time_boot_ms = (uint)1838879457U;
            SMP_TEST_CH.send(p82);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATTITUDE_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.body_yaw_rate == (float) -1.4912026E38F);
                Debug.Assert(pack.body_roll_rate == (float)1.0524939E38F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-1.2438382E38F, 1.2732473E38F, 2.4819003E38F, 2.4468648E38F}));
                Debug.Assert(pack.body_pitch_rate == (float)7.174976E36F);
                Debug.Assert(pack.time_boot_ms == (uint)1775707608U);
                Debug.Assert(pack.type_mask == (byte)(byte)21);
                Debug.Assert(pack.thrust == (float) -4.2077334E37F);
            };
            ATTITUDE_TARGET p83 = new ATTITUDE_TARGET();
            PH.setPack(p83);
            p83.time_boot_ms = (uint)1775707608U;
            p83.thrust = (float) -4.2077334E37F;
            p83.q_SET(new float[] {-1.2438382E38F, 1.2732473E38F, 2.4819003E38F, 2.4468648E38F}, 0) ;
            p83.body_roll_rate = (float)1.0524939E38F;
            p83.body_pitch_rate = (float)7.174976E36F;
            p83.body_yaw_rate = (float) -1.4912026E38F;
            p83.type_mask = (byte)(byte)21;
            SMP_TEST_CH.send(p83);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_POSITION_TARGET_LOCAL_NEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)147);
                Debug.Assert(pack.target_system == (byte)(byte)240);
                Debug.Assert(pack.z == (float)2.2558763E38F);
                Debug.Assert(pack.afz == (float) -3.5326925E37F);
                Debug.Assert(pack.yaw == (float)9.552018E37F);
                Debug.Assert(pack.coordinate_frame == MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT);
                Debug.Assert(pack.vy == (float) -2.4077548E37F);
                Debug.Assert(pack.vx == (float)1.9783034E38F);
                Debug.Assert(pack.x == (float)2.0792082E38F);
                Debug.Assert(pack.afy == (float)1.9309505E38F);
                Debug.Assert(pack.afx == (float)2.924804E38F);
                Debug.Assert(pack.yaw_rate == (float)1.7623633E38F);
                Debug.Assert(pack.y == (float) -3.2542725E38F);
                Debug.Assert(pack.vz == (float)1.349819E38F);
                Debug.Assert(pack.type_mask == (ushort)(ushort)39298);
                Debug.Assert(pack.time_boot_ms == (uint)422997861U);
            };
            SET_POSITION_TARGET_LOCAL_NED p84 = new SET_POSITION_TARGET_LOCAL_NED();
            PH.setPack(p84);
            p84.afy = (float)1.9309505E38F;
            p84.z = (float)2.2558763E38F;
            p84.vx = (float)1.9783034E38F;
            p84.afx = (float)2.924804E38F;
            p84.vy = (float) -2.4077548E37F;
            p84.type_mask = (ushort)(ushort)39298;
            p84.target_system = (byte)(byte)240;
            p84.yaw_rate = (float)1.7623633E38F;
            p84.coordinate_frame = MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT;
            p84.vz = (float)1.349819E38F;
            p84.y = (float) -3.2542725E38F;
            p84.yaw = (float)9.552018E37F;
            p84.afz = (float) -3.5326925E37F;
            p84.time_boot_ms = (uint)422997861U;
            p84.target_component = (byte)(byte)147;
            p84.x = (float)2.0792082E38F;
            SMP_TEST_CH.send(p84);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_POSITION_TARGET_GLOBAL_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)225);
                Debug.Assert(pack.alt == (float)3.2472683E38F);
                Debug.Assert(pack.yaw == (float)1.6121906E38F);
                Debug.Assert(pack.vz == (float)2.8678896E36F);
                Debug.Assert(pack.vx == (float)2.1462836E38F);
                Debug.Assert(pack.lat_int == (int)161172140);
                Debug.Assert(pack.coordinate_frame == MAV_FRAME.MAV_FRAME_MISSION);
                Debug.Assert(pack.afy == (float) -1.593971E38F);
                Debug.Assert(pack.target_component == (byte)(byte)1);
                Debug.Assert(pack.afx == (float)7.7916707E37F);
                Debug.Assert(pack.time_boot_ms == (uint)1023211194U);
                Debug.Assert(pack.type_mask == (ushort)(ushort)14213);
                Debug.Assert(pack.lon_int == (int)715879245);
                Debug.Assert(pack.vy == (float)9.707746E37F);
                Debug.Assert(pack.yaw_rate == (float)2.7382143E38F);
                Debug.Assert(pack.afz == (float)2.603332E38F);
            };
            SET_POSITION_TARGET_GLOBAL_INT p86 = new SET_POSITION_TARGET_GLOBAL_INT();
            PH.setPack(p86);
            p86.alt = (float)3.2472683E38F;
            p86.coordinate_frame = MAV_FRAME.MAV_FRAME_MISSION;
            p86.target_component = (byte)(byte)1;
            p86.afx = (float)7.7916707E37F;
            p86.target_system = (byte)(byte)225;
            p86.vx = (float)2.1462836E38F;
            p86.vz = (float)2.8678896E36F;
            p86.type_mask = (ushort)(ushort)14213;
            p86.lat_int = (int)161172140;
            p86.yaw = (float)1.6121906E38F;
            p86.yaw_rate = (float)2.7382143E38F;
            p86.afz = (float)2.603332E38F;
            p86.lon_int = (int)715879245;
            p86.afy = (float) -1.593971E38F;
            p86.time_boot_ms = (uint)1023211194U;
            p86.vy = (float)9.707746E37F;
            SMP_TEST_CH.send(p86);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPOSITION_TARGET_GLOBAL_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vx == (float) -1.1979326E38F);
                Debug.Assert(pack.coordinate_frame == MAV_FRAME.MAV_FRAME_BODY_NED);
                Debug.Assert(pack.time_boot_ms == (uint)2461144177U);
                Debug.Assert(pack.vz == (float)1.5677893E38F);
                Debug.Assert(pack.yaw_rate == (float)3.1345207E38F);
                Debug.Assert(pack.alt == (float) -2.8793475E38F);
                Debug.Assert(pack.lon_int == (int) -1774993923);
                Debug.Assert(pack.afy == (float) -1.7371438E38F);
                Debug.Assert(pack.type_mask == (ushort)(ushort)28755);
                Debug.Assert(pack.vy == (float)2.8037111E38F);
                Debug.Assert(pack.afx == (float)6.211827E37F);
                Debug.Assert(pack.lat_int == (int)954801066);
                Debug.Assert(pack.yaw == (float) -1.2441117E38F);
                Debug.Assert(pack.afz == (float) -2.338877E38F);
            };
            POSITION_TARGET_GLOBAL_INT p87 = new POSITION_TARGET_GLOBAL_INT();
            PH.setPack(p87);
            p87.vz = (float)1.5677893E38F;
            p87.type_mask = (ushort)(ushort)28755;
            p87.vy = (float)2.8037111E38F;
            p87.lat_int = (int)954801066;
            p87.vx = (float) -1.1979326E38F;
            p87.yaw_rate = (float)3.1345207E38F;
            p87.coordinate_frame = MAV_FRAME.MAV_FRAME_BODY_NED;
            p87.yaw = (float) -1.2441117E38F;
            p87.afx = (float)6.211827E37F;
            p87.lon_int = (int) -1774993923;
            p87.alt = (float) -2.8793475E38F;
            p87.afz = (float) -2.338877E38F;
            p87.time_boot_ms = (uint)2461144177U;
            p87.afy = (float) -1.7371438E38F;
            SMP_TEST_CH.send(p87);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.x == (float)1.5683781E38F);
                Debug.Assert(pack.roll == (float) -2.9831871E38F);
                Debug.Assert(pack.y == (float)2.5447362E38F);
                Debug.Assert(pack.time_boot_ms == (uint)3076402331U);
                Debug.Assert(pack.z == (float)2.963245E38F);
                Debug.Assert(pack.yaw == (float)2.5731581E38F);
                Debug.Assert(pack.pitch == (float) -1.2164175E38F);
            };
            LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET p89 = new LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
            PH.setPack(p89);
            p89.yaw = (float)2.5731581E38F;
            p89.z = (float)2.963245E38F;
            p89.x = (float)1.5683781E38F;
            p89.y = (float)2.5447362E38F;
            p89.roll = (float) -2.9831871E38F;
            p89.time_boot_ms = (uint)3076402331U;
            p89.pitch = (float) -1.2164175E38F;
            SMP_TEST_CH.send(p89);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lon == (int)1296490875);
                Debug.Assert(pack.time_usec == (ulong)3970784881625203517L);
                Debug.Assert(pack.lat == (int) -1399169581);
                Debug.Assert(pack.vx == (short)(short)7657);
                Debug.Assert(pack.zacc == (short)(short)26167);
                Debug.Assert(pack.roll == (float)2.5522905E37F);
                Debug.Assert(pack.rollspeed == (float) -1.5584848E38F);
                Debug.Assert(pack.yawspeed == (float) -7.7320855E37F);
                Debug.Assert(pack.xacc == (short)(short)14936);
                Debug.Assert(pack.alt == (int) -2144442417);
                Debug.Assert(pack.pitch == (float) -2.8739961E37F);
                Debug.Assert(pack.vy == (short)(short)6934);
                Debug.Assert(pack.yaw == (float)2.8049341E38F);
                Debug.Assert(pack.yacc == (short)(short)25338);
                Debug.Assert(pack.pitchspeed == (float) -3.1245644E38F);
                Debug.Assert(pack.vz == (short)(short) -6340);
            };
            HIL_STATE p90 = new HIL_STATE();
            PH.setPack(p90);
            p90.rollspeed = (float) -1.5584848E38F;
            p90.lat = (int) -1399169581;
            p90.xacc = (short)(short)14936;
            p90.lon = (int)1296490875;
            p90.roll = (float)2.5522905E37F;
            p90.zacc = (short)(short)26167;
            p90.alt = (int) -2144442417;
            p90.time_usec = (ulong)3970784881625203517L;
            p90.pitch = (float) -2.8739961E37F;
            p90.pitchspeed = (float) -3.1245644E38F;
            p90.yaw = (float)2.8049341E38F;
            p90.yacc = (short)(short)25338;
            p90.vx = (short)(short)7657;
            p90.yawspeed = (float) -7.7320855E37F;
            p90.vy = (short)(short)6934;
            p90.vz = (short)(short) -6340;
            SMP_TEST_CH.send(p90);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_CONTROLSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.aux2 == (float)1.9897052E38F);
                Debug.Assert(pack.throttle == (float)3.2999488E37F);
                Debug.Assert(pack.yaw_rudder == (float) -1.0593955E38F);
                Debug.Assert(pack.aux3 == (float) -1.8888298E38F);
                Debug.Assert(pack.aux1 == (float)2.64267E38F);
                Debug.Assert(pack.mode == MAV_MODE.MAV_MODE_PREFLIGHT);
                Debug.Assert(pack.time_usec == (ulong)2419204240475870275L);
                Debug.Assert(pack.nav_mode == (byte)(byte)117);
                Debug.Assert(pack.roll_ailerons == (float) -1.9426784E38F);
                Debug.Assert(pack.pitch_elevator == (float)2.0903228E38F);
                Debug.Assert(pack.aux4 == (float)1.0619122E38F);
            };
            HIL_CONTROLS p91 = new HIL_CONTROLS();
            PH.setPack(p91);
            p91.nav_mode = (byte)(byte)117;
            p91.aux2 = (float)1.9897052E38F;
            p91.pitch_elevator = (float)2.0903228E38F;
            p91.roll_ailerons = (float) -1.9426784E38F;
            p91.time_usec = (ulong)2419204240475870275L;
            p91.yaw_rudder = (float) -1.0593955E38F;
            p91.throttle = (float)3.2999488E37F;
            p91.aux4 = (float)1.0619122E38F;
            p91.aux3 = (float) -1.8888298E38F;
            p91.aux1 = (float)2.64267E38F;
            p91.mode = MAV_MODE.MAV_MODE_PREFLIGHT;
            SMP_TEST_CH.send(p91);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_RC_INPUTS_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)15619);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)46490);
                Debug.Assert(pack.chan12_raw == (ushort)(ushort)65331);
                Debug.Assert(pack.rssi == (byte)(byte)111);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)24188);
                Debug.Assert(pack.chan11_raw == (ushort)(ushort)42681);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)34097);
                Debug.Assert(pack.chan9_raw == (ushort)(ushort)57682);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)278);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)9850);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)19708);
                Debug.Assert(pack.time_usec == (ulong)2997928145303580693L);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)28390);
                Debug.Assert(pack.chan10_raw == (ushort)(ushort)21621);
            };
            HIL_RC_INPUTS_RAW p92 = new HIL_RC_INPUTS_RAW();
            PH.setPack(p92);
            p92.chan6_raw = (ushort)(ushort)19708;
            p92.chan9_raw = (ushort)(ushort)57682;
            p92.chan5_raw = (ushort)(ushort)34097;
            p92.time_usec = (ulong)2997928145303580693L;
            p92.chan10_raw = (ushort)(ushort)21621;
            p92.chan3_raw = (ushort)(ushort)15619;
            p92.chan12_raw = (ushort)(ushort)65331;
            p92.rssi = (byte)(byte)111;
            p92.chan2_raw = (ushort)(ushort)28390;
            p92.chan1_raw = (ushort)(ushort)9850;
            p92.chan11_raw = (ushort)(ushort)42681;
            p92.chan7_raw = (ushort)(ushort)46490;
            p92.chan8_raw = (ushort)(ushort)24188;
            p92.chan4_raw = (ushort)(ushort)278;
            SMP_TEST_CH.send(p92);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_ACTUATOR_CONTROLSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mode == MAV_MODE.MAV_MODE_MANUAL_ARMED);
                Debug.Assert(pack.flags == (ulong)9199555358530478399L);
                Debug.Assert(pack.controls.SequenceEqual(new float[] {2.3585994E38F, 1.2183388E38F, 2.1619223E38F, 6.1759314E37F, 3.3305602E36F, 1.8302173E38F, 2.2550046E38F, 1.9573842E38F, 2.8748354E38F, -2.1785519E38F, 2.8672802E38F, 1.3538543E38F, 2.3158234E38F, -1.6341162E38F, -2.753872E37F, 2.9934894E38F}));
                Debug.Assert(pack.time_usec == (ulong)6387559662428557596L);
            };
            HIL_ACTUATOR_CONTROLS p93 = new HIL_ACTUATOR_CONTROLS();
            PH.setPack(p93);
            p93.mode = MAV_MODE.MAV_MODE_MANUAL_ARMED;
            p93.flags = (ulong)9199555358530478399L;
            p93.time_usec = (ulong)6387559662428557596L;
            p93.controls_SET(new float[] {2.3585994E38F, 1.2183388E38F, 2.1619223E38F, 6.1759314E37F, 3.3305602E36F, 1.8302173E38F, 2.2550046E38F, 1.9573842E38F, 2.8748354E38F, -2.1785519E38F, 2.8672802E38F, 1.3538543E38F, 2.3158234E38F, -1.6341162E38F, -2.753872E37F, 2.9934894E38F}, 0) ;
            SMP_TEST_CH.send(p93);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnOPTICAL_FLOWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.flow_rate_x_TRY(ph) == (float) -2.2252032E38F);
                Debug.Assert(pack.flow_comp_m_x == (float)3.3441763E38F);
                Debug.Assert(pack.flow_rate_y_TRY(ph) == (float)2.152395E38F);
                Debug.Assert(pack.quality == (byte)(byte)250);
                Debug.Assert(pack.time_usec == (ulong)5913676282590462814L);
                Debug.Assert(pack.sensor_id == (byte)(byte)166);
                Debug.Assert(pack.flow_x == (short)(short)10409);
                Debug.Assert(pack.flow_comp_m_y == (float)4.3547895E37F);
                Debug.Assert(pack.ground_distance == (float) -3.15095E38F);
                Debug.Assert(pack.flow_y == (short)(short) -11855);
            };
            OPTICAL_FLOW p100 = new OPTICAL_FLOW();
            PH.setPack(p100);
            p100.flow_rate_y_SET((float)2.152395E38F, PH) ;
            p100.flow_comp_m_y = (float)4.3547895E37F;
            p100.time_usec = (ulong)5913676282590462814L;
            p100.flow_x = (short)(short)10409;
            p100.sensor_id = (byte)(byte)166;
            p100.flow_comp_m_x = (float)3.3441763E38F;
            p100.flow_y = (short)(short) -11855;
            p100.ground_distance = (float) -3.15095E38F;
            p100.flow_rate_x_SET((float) -2.2252032E38F, PH) ;
            p100.quality = (byte)(byte)250;
            SMP_TEST_CH.send(p100);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGLOBAL_VISION_POSITION_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.yaw == (float) -1.8865466E38F);
                Debug.Assert(pack.roll == (float) -4.0458861E37F);
                Debug.Assert(pack.usec == (ulong)7329446983633484899L);
                Debug.Assert(pack.x == (float) -2.2982475E38F);
                Debug.Assert(pack.pitch == (float)2.6802669E38F);
                Debug.Assert(pack.y == (float)1.95368E38F);
                Debug.Assert(pack.z == (float) -6.7761624E37F);
            };
            GLOBAL_VISION_POSITION_ESTIMATE p101 = new GLOBAL_VISION_POSITION_ESTIMATE();
            PH.setPack(p101);
            p101.usec = (ulong)7329446983633484899L;
            p101.y = (float)1.95368E38F;
            p101.x = (float) -2.2982475E38F;
            p101.z = (float) -6.7761624E37F;
            p101.pitch = (float)2.6802669E38F;
            p101.yaw = (float) -1.8865466E38F;
            p101.roll = (float) -4.0458861E37F;
            SMP_TEST_CH.send(p101);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnVISION_POSITION_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.yaw == (float) -2.2824917E38F);
                Debug.Assert(pack.usec == (ulong)1771073969800832894L);
                Debug.Assert(pack.pitch == (float) -6.6046315E37F);
                Debug.Assert(pack.y == (float) -1.2527515E38F);
                Debug.Assert(pack.z == (float)2.5741457E38F);
                Debug.Assert(pack.roll == (float)7.416646E37F);
                Debug.Assert(pack.x == (float) -8.3281663E37F);
            };
            VISION_POSITION_ESTIMATE p102 = new VISION_POSITION_ESTIMATE();
            PH.setPack(p102);
            p102.z = (float)2.5741457E38F;
            p102.pitch = (float) -6.6046315E37F;
            p102.x = (float) -8.3281663E37F;
            p102.usec = (ulong)1771073969800832894L;
            p102.yaw = (float) -2.2824917E38F;
            p102.y = (float) -1.2527515E38F;
            p102.roll = (float)7.416646E37F;
            SMP_TEST_CH.send(p102);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnVISION_SPEED_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.y == (float)1.3431725E36F);
                Debug.Assert(pack.usec == (ulong)8186062769909481406L);
                Debug.Assert(pack.z == (float) -1.5572852E38F);
                Debug.Assert(pack.x == (float)1.6525591E38F);
            };
            GroundControl.VISION_SPEED_ESTIMATE p103 = CommunicationChannel.new_VISION_SPEED_ESTIMATE();
            PH.setPack(p103);
            p103.x = (float)1.6525591E38F;
            p103.y = (float)1.3431725E36F;
            p103.z = (float) -1.5572852E38F;
            p103.usec = (ulong)8186062769909481406L;
            CommunicationChannel.instance.send(p103);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnVICON_POSITION_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.x == (float) -3.1363759E38F);
                Debug.Assert(pack.roll == (float)9.211906E37F);
                Debug.Assert(pack.z == (float)2.7279873E38F);
                Debug.Assert(pack.usec == (ulong)5287244611089968752L);
                Debug.Assert(pack.yaw == (float) -2.5072243E38F);
                Debug.Assert(pack.pitch == (float)2.1515546E38F);
                Debug.Assert(pack.y == (float) -3.335362E37F);
            };
            GroundControl.VICON_POSITION_ESTIMATE p104 = CommunicationChannel.new_VICON_POSITION_ESTIMATE();
            PH.setPack(p104);
            p104.z = (float)2.7279873E38F;
            p104.roll = (float)9.211906E37F;
            p104.pitch = (float)2.1515546E38F;
            p104.x = (float) -3.1363759E38F;
            p104.usec = (ulong)5287244611089968752L;
            p104.yaw = (float) -2.5072243E38F;
            p104.y = (float) -3.335362E37F;
            CommunicationChannel.instance.send(p104);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIGHRES_IMUReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.zmag == (float)1.8538183E38F);
                Debug.Assert(pack.time_usec == (ulong)1569251225271581796L);
                Debug.Assert(pack.xacc == (float)2.7066275E38F);
                Debug.Assert(pack.abs_pressure == (float)1.0717574E38F);
                Debug.Assert(pack.ymag == (float) -1.4610165E38F);
                Debug.Assert(pack.diff_pressure == (float) -3.013892E38F);
                Debug.Assert(pack.yacc == (float)5.1853365E36F);
                Debug.Assert(pack.zacc == (float)2.2776047E38F);
                Debug.Assert(pack.xgyro == (float)1.5382535E38F);
                Debug.Assert(pack.xmag == (float)2.1907734E38F);
                Debug.Assert(pack.pressure_alt == (float) -6.6037383E36F);
                Debug.Assert(pack.temperature == (float)2.5079942E38F);
                Debug.Assert(pack.zgyro == (float)6.5642814E36F);
                Debug.Assert(pack.ygyro == (float)1.9250126E38F);
                Debug.Assert(pack.fields_updated == (ushort)(ushort)11684);
            };
            GroundControl.HIGHRES_IMU p105 = CommunicationChannel.new_HIGHRES_IMU();
            PH.setPack(p105);
            p105.ygyro = (float)1.9250126E38F;
            p105.fields_updated = (ushort)(ushort)11684;
            p105.abs_pressure = (float)1.0717574E38F;
            p105.time_usec = (ulong)1569251225271581796L;
            p105.diff_pressure = (float) -3.013892E38F;
            p105.pressure_alt = (float) -6.6037383E36F;
            p105.zacc = (float)2.2776047E38F;
            p105.yacc = (float)5.1853365E36F;
            p105.temperature = (float)2.5079942E38F;
            p105.xacc = (float)2.7066275E38F;
            p105.xgyro = (float)1.5382535E38F;
            p105.zmag = (float)1.8538183E38F;
            p105.ymag = (float) -1.4610165E38F;
            p105.zgyro = (float)6.5642814E36F;
            p105.xmag = (float)2.1907734E38F;
            CommunicationChannel.instance.send(p105);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnOPTICAL_FLOW_RADReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.temperature == (short)(short) -31263);
                Debug.Assert(pack.integrated_ygyro == (float)1.0563965E38F);
                Debug.Assert(pack.integrated_xgyro == (float) -1.963328E38F);
                Debug.Assert(pack.quality == (byte)(byte)57);
                Debug.Assert(pack.time_usec == (ulong)2688680238529694517L);
                Debug.Assert(pack.distance == (float)1.3233045E38F);
                Debug.Assert(pack.integration_time_us == (uint)1624945130U);
                Debug.Assert(pack.integrated_y == (float)3.2912733E38F);
                Debug.Assert(pack.sensor_id == (byte)(byte)23);
                Debug.Assert(pack.time_delta_distance_us == (uint)4261461670U);
                Debug.Assert(pack.integrated_zgyro == (float) -1.3476849E38F);
                Debug.Assert(pack.integrated_x == (float) -2.1162684E37F);
            };
            GroundControl.OPTICAL_FLOW_RAD p106 = CommunicationChannel.new_OPTICAL_FLOW_RAD();
            PH.setPack(p106);
            p106.integrated_x = (float) -2.1162684E37F;
            p106.integrated_xgyro = (float) -1.963328E38F;
            p106.integration_time_us = (uint)1624945130U;
            p106.integrated_zgyro = (float) -1.3476849E38F;
            p106.quality = (byte)(byte)57;
            p106.time_usec = (ulong)2688680238529694517L;
            p106.integrated_y = (float)3.2912733E38F;
            p106.sensor_id = (byte)(byte)23;
            p106.temperature = (short)(short) -31263;
            p106.time_delta_distance_us = (uint)4261461670U;
            p106.integrated_ygyro = (float)1.0563965E38F;
            p106.distance = (float)1.3233045E38F;
            CommunicationChannel.instance.send(p106);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_SENSORReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.abs_pressure == (float)2.3974617E37F);
                Debug.Assert(pack.fields_updated == (uint)788757244U);
                Debug.Assert(pack.time_usec == (ulong)2707334716034648782L);
                Debug.Assert(pack.yacc == (float) -9.656685E37F);
                Debug.Assert(pack.zacc == (float)3.134495E38F);
                Debug.Assert(pack.zmag == (float) -3.143203E38F);
                Debug.Assert(pack.diff_pressure == (float) -1.1634013E38F);
                Debug.Assert(pack.xmag == (float) -4.2149616E37F);
                Debug.Assert(pack.temperature == (float) -2.057055E38F);
                Debug.Assert(pack.ymag == (float)2.8501132E38F);
                Debug.Assert(pack.pressure_alt == (float)2.7944528E38F);
                Debug.Assert(pack.ygyro == (float) -2.8511152E38F);
                Debug.Assert(pack.xgyro == (float) -2.8885074E38F);
                Debug.Assert(pack.zgyro == (float)3.3298344E38F);
                Debug.Assert(pack.xacc == (float) -1.5521286E38F);
            };
            GroundControl.HIL_SENSOR p107 = CommunicationChannel.new_HIL_SENSOR();
            PH.setPack(p107);
            p107.xacc = (float) -1.5521286E38F;
            p107.zacc = (float)3.134495E38F;
            p107.xmag = (float) -4.2149616E37F;
            p107.ygyro = (float) -2.8511152E38F;
            p107.pressure_alt = (float)2.7944528E38F;
            p107.diff_pressure = (float) -1.1634013E38F;
            p107.xgyro = (float) -2.8885074E38F;
            p107.ymag = (float)2.8501132E38F;
            p107.abs_pressure = (float)2.3974617E37F;
            p107.fields_updated = (uint)788757244U;
            p107.zgyro = (float)3.3298344E38F;
            p107.zmag = (float) -3.143203E38F;
            p107.temperature = (float) -2.057055E38F;
            p107.yacc = (float) -9.656685E37F;
            p107.time_usec = (ulong)2707334716034648782L;
            CommunicationChannel.instance.send(p107);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSIM_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.q1 == (float)3.1727867E37F);
                Debug.Assert(pack.lon == (float)4.413384E35F);
                Debug.Assert(pack.yaw == (float) -2.956897E38F);
                Debug.Assert(pack.roll == (float) -3.306345E38F);
                Debug.Assert(pack.xgyro == (float)1.08506336E36F);
                Debug.Assert(pack.pitch == (float) -1.7958993E38F);
                Debug.Assert(pack.lat == (float) -3.3136087E38F);
                Debug.Assert(pack.q4 == (float)8.931771E37F);
                Debug.Assert(pack.std_dev_horz == (float)2.5049212E38F);
                Debug.Assert(pack.zacc == (float)3.249381E38F);
                Debug.Assert(pack.alt == (float)4.756313E37F);
                Debug.Assert(pack.xacc == (float)2.9039147E38F);
                Debug.Assert(pack.zgyro == (float) -2.0608587E38F);
                Debug.Assert(pack.std_dev_vert == (float)3.0129106E38F);
                Debug.Assert(pack.yacc == (float) -2.0919442E38F);
                Debug.Assert(pack.ygyro == (float)1.4620825E38F);
                Debug.Assert(pack.vd == (float) -1.00036444E37F);
                Debug.Assert(pack.ve == (float) -3.2864306E38F);
                Debug.Assert(pack.q3 == (float) -1.6455292E38F);
                Debug.Assert(pack.vn == (float) -7.1662697E37F);
                Debug.Assert(pack.q2 == (float)1.1948539E38F);
            };
            GroundControl.SIM_STATE p108 = CommunicationChannel.new_SIM_STATE();
            PH.setPack(p108);
            p108.zgyro = (float) -2.0608587E38F;
            p108.ygyro = (float)1.4620825E38F;
            p108.pitch = (float) -1.7958993E38F;
            p108.vn = (float) -7.1662697E37F;
            p108.q1 = (float)3.1727867E37F;
            p108.alt = (float)4.756313E37F;
            p108.yaw = (float) -2.956897E38F;
            p108.xacc = (float)2.9039147E38F;
            p108.std_dev_horz = (float)2.5049212E38F;
            p108.q2 = (float)1.1948539E38F;
            p108.lat = (float) -3.3136087E38F;
            p108.yacc = (float) -2.0919442E38F;
            p108.roll = (float) -3.306345E38F;
            p108.zacc = (float)3.249381E38F;
            p108.q4 = (float)8.931771E37F;
            p108.vd = (float) -1.00036444E37F;
            p108.xgyro = (float)1.08506336E36F;
            p108.std_dev_vert = (float)3.0129106E38F;
            p108.lon = (float)4.413384E35F;
            p108.ve = (float) -3.2864306E38F;
            p108.q3 = (float) -1.6455292E38F;
            CommunicationChannel.instance.send(p108);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRADIO_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.remrssi == (byte)(byte)118);
                Debug.Assert(pack.remnoise == (byte)(byte)74);
                Debug.Assert(pack.rxerrors == (ushort)(ushort)13695);
                Debug.Assert(pack.rssi == (byte)(byte)90);
                Debug.Assert(pack.fixed_ == (ushort)(ushort)23989);
                Debug.Assert(pack.noise == (byte)(byte)117);
                Debug.Assert(pack.txbuf == (byte)(byte)66);
            };
            GroundControl.RADIO_STATUS p109 = CommunicationChannel.new_RADIO_STATUS();
            PH.setPack(p109);
            p109.noise = (byte)(byte)117;
            p109.rssi = (byte)(byte)90;
            p109.remnoise = (byte)(byte)74;
            p109.txbuf = (byte)(byte)66;
            p109.remrssi = (byte)(byte)118;
            p109.fixed_ = (ushort)(ushort)23989;
            p109.rxerrors = (ushort)(ushort)13695;
            CommunicationChannel.instance.send(p109);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnFILE_TRANSFER_PROTOCOLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.payload.SequenceEqual(new byte[] {(byte)144, (byte)94, (byte)190, (byte)57, (byte)113, (byte)252, (byte)25, (byte)232, (byte)84, (byte)151, (byte)41, (byte)66, (byte)237, (byte)128, (byte)108, (byte)183, (byte)227, (byte)63, (byte)18, (byte)97, (byte)128, (byte)97, (byte)71, (byte)189, (byte)222, (byte)195, (byte)54, (byte)164, (byte)159, (byte)253, (byte)109, (byte)21, (byte)91, (byte)30, (byte)111, (byte)238, (byte)75, (byte)44, (byte)79, (byte)130, (byte)242, (byte)245, (byte)182, (byte)117, (byte)178, (byte)109, (byte)4, (byte)75, (byte)111, (byte)39, (byte)120, (byte)107, (byte)140, (byte)131, (byte)106, (byte)40, (byte)102, (byte)18, (byte)231, (byte)24, (byte)54, (byte)190, (byte)104, (byte)198, (byte)49, (byte)170, (byte)254, (byte)141, (byte)238, (byte)17, (byte)3, (byte)111, (byte)251, (byte)139, (byte)175, (byte)207, (byte)239, (byte)251, (byte)128, (byte)218, (byte)150, (byte)147, (byte)1, (byte)137, (byte)225, (byte)35, (byte)75, (byte)22, (byte)42, (byte)208, (byte)117, (byte)106, (byte)211, (byte)161, (byte)54, (byte)41, (byte)137, (byte)179, (byte)251, (byte)70, (byte)1, (byte)252, (byte)11, (byte)113, (byte)160, (byte)6, (byte)14, (byte)152, (byte)55, (byte)25, (byte)153, (byte)116, (byte)90, (byte)194, (byte)219, (byte)228, (byte)247, (byte)123, (byte)226, (byte)157, (byte)205, (byte)157, (byte)14, (byte)99, (byte)203, (byte)82, (byte)162, (byte)203, (byte)220, (byte)128, (byte)236, (byte)33, (byte)144, (byte)73, (byte)174, (byte)79, (byte)194, (byte)197, (byte)19, (byte)36, (byte)201, (byte)102, (byte)181, (byte)226, (byte)129, (byte)46, (byte)227, (byte)111, (byte)233, (byte)41, (byte)148, (byte)55, (byte)76, (byte)135, (byte)168, (byte)175, (byte)208, (byte)177, (byte)91, (byte)171, (byte)20, (byte)225, (byte)153, (byte)4, (byte)26, (byte)113, (byte)64, (byte)232, (byte)112, (byte)176, (byte)204, (byte)69, (byte)106, (byte)6, (byte)241, (byte)250, (byte)24, (byte)79, (byte)140, (byte)189, (byte)74, (byte)91, (byte)142, (byte)191, (byte)141, (byte)185, (byte)52, (byte)67, (byte)17, (byte)147, (byte)117, (byte)77, (byte)147, (byte)227, (byte)226, (byte)96, (byte)210, (byte)146, (byte)222, (byte)27, (byte)206, (byte)26, (byte)215, (byte)221, (byte)72, (byte)119, (byte)63, (byte)99, (byte)36, (byte)226, (byte)187, (byte)88, (byte)95, (byte)219, (byte)248, (byte)121, (byte)97, (byte)20, (byte)114, (byte)196, (byte)54, (byte)35, (byte)200, (byte)56, (byte)54, (byte)109, (byte)41, (byte)50, (byte)146, (byte)16, (byte)198, (byte)79, (byte)27, (byte)33, (byte)142, (byte)255, (byte)146, (byte)176, (byte)239, (byte)27, (byte)21, (byte)137, (byte)30, (byte)216, (byte)25, (byte)225, (byte)223, (byte)8, (byte)59, (byte)21, (byte)48}));
                Debug.Assert(pack.target_network == (byte)(byte)45);
                Debug.Assert(pack.target_component == (byte)(byte)76);
                Debug.Assert(pack.target_system == (byte)(byte)198);
            };
            GroundControl.FILE_TRANSFER_PROTOCOL p110 = CommunicationChannel.new_FILE_TRANSFER_PROTOCOL();
            PH.setPack(p110);
            p110.target_system = (byte)(byte)198;
            p110.target_network = (byte)(byte)45;
            p110.target_component = (byte)(byte)76;
            p110.payload_SET(new byte[] {(byte)144, (byte)94, (byte)190, (byte)57, (byte)113, (byte)252, (byte)25, (byte)232, (byte)84, (byte)151, (byte)41, (byte)66, (byte)237, (byte)128, (byte)108, (byte)183, (byte)227, (byte)63, (byte)18, (byte)97, (byte)128, (byte)97, (byte)71, (byte)189, (byte)222, (byte)195, (byte)54, (byte)164, (byte)159, (byte)253, (byte)109, (byte)21, (byte)91, (byte)30, (byte)111, (byte)238, (byte)75, (byte)44, (byte)79, (byte)130, (byte)242, (byte)245, (byte)182, (byte)117, (byte)178, (byte)109, (byte)4, (byte)75, (byte)111, (byte)39, (byte)120, (byte)107, (byte)140, (byte)131, (byte)106, (byte)40, (byte)102, (byte)18, (byte)231, (byte)24, (byte)54, (byte)190, (byte)104, (byte)198, (byte)49, (byte)170, (byte)254, (byte)141, (byte)238, (byte)17, (byte)3, (byte)111, (byte)251, (byte)139, (byte)175, (byte)207, (byte)239, (byte)251, (byte)128, (byte)218, (byte)150, (byte)147, (byte)1, (byte)137, (byte)225, (byte)35, (byte)75, (byte)22, (byte)42, (byte)208, (byte)117, (byte)106, (byte)211, (byte)161, (byte)54, (byte)41, (byte)137, (byte)179, (byte)251, (byte)70, (byte)1, (byte)252, (byte)11, (byte)113, (byte)160, (byte)6, (byte)14, (byte)152, (byte)55, (byte)25, (byte)153, (byte)116, (byte)90, (byte)194, (byte)219, (byte)228, (byte)247, (byte)123, (byte)226, (byte)157, (byte)205, (byte)157, (byte)14, (byte)99, (byte)203, (byte)82, (byte)162, (byte)203, (byte)220, (byte)128, (byte)236, (byte)33, (byte)144, (byte)73, (byte)174, (byte)79, (byte)194, (byte)197, (byte)19, (byte)36, (byte)201, (byte)102, (byte)181, (byte)226, (byte)129, (byte)46, (byte)227, (byte)111, (byte)233, (byte)41, (byte)148, (byte)55, (byte)76, (byte)135, (byte)168, (byte)175, (byte)208, (byte)177, (byte)91, (byte)171, (byte)20, (byte)225, (byte)153, (byte)4, (byte)26, (byte)113, (byte)64, (byte)232, (byte)112, (byte)176, (byte)204, (byte)69, (byte)106, (byte)6, (byte)241, (byte)250, (byte)24, (byte)79, (byte)140, (byte)189, (byte)74, (byte)91, (byte)142, (byte)191, (byte)141, (byte)185, (byte)52, (byte)67, (byte)17, (byte)147, (byte)117, (byte)77, (byte)147, (byte)227, (byte)226, (byte)96, (byte)210, (byte)146, (byte)222, (byte)27, (byte)206, (byte)26, (byte)215, (byte)221, (byte)72, (byte)119, (byte)63, (byte)99, (byte)36, (byte)226, (byte)187, (byte)88, (byte)95, (byte)219, (byte)248, (byte)121, (byte)97, (byte)20, (byte)114, (byte)196, (byte)54, (byte)35, (byte)200, (byte)56, (byte)54, (byte)109, (byte)41, (byte)50, (byte)146, (byte)16, (byte)198, (byte)79, (byte)27, (byte)33, (byte)142, (byte)255, (byte)146, (byte)176, (byte)239, (byte)27, (byte)21, (byte)137, (byte)30, (byte)216, (byte)25, (byte)225, (byte)223, (byte)8, (byte)59, (byte)21, (byte)48}, 0) ;
            CommunicationChannel.instance.send(p110);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTIMESYNCReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ts1 == (long)5395761942338736143L);
                Debug.Assert(pack.tc1 == (long) -5005903779980495427L);
            };
            GroundControl.TIMESYNC p111 = CommunicationChannel.new_TIMESYNC();
            PH.setPack(p111);
            p111.tc1 = (long) -5005903779980495427L;
            p111.ts1 = (long)5395761942338736143L;
            CommunicationChannel.instance.send(p111);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCAMERA_TRIGGERReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)50825119746893358L);
                Debug.Assert(pack.seq == (uint)964450856U);
            };
            GroundControl.CAMERA_TRIGGER p112 = CommunicationChannel.new_CAMERA_TRIGGER();
            PH.setPack(p112);
            p112.time_usec = (ulong)50825119746893358L;
            p112.seq = (uint)964450856U;
            CommunicationChannel.instance.send(p112);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_GPSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.cog == (ushort)(ushort)14217);
                Debug.Assert(pack.vn == (short)(short) -4528);
                Debug.Assert(pack.ve == (short)(short)2742);
                Debug.Assert(pack.eph == (ushort)(ushort)21724);
                Debug.Assert(pack.lat == (int)1833958593);
                Debug.Assert(pack.vel == (ushort)(ushort)49848);
                Debug.Assert(pack.time_usec == (ulong)610684699306545453L);
                Debug.Assert(pack.lon == (int) -1651711670);
                Debug.Assert(pack.alt == (int) -1583963432);
                Debug.Assert(pack.fix_type == (byte)(byte)85);
                Debug.Assert(pack.satellites_visible == (byte)(byte)30);
                Debug.Assert(pack.epv == (ushort)(ushort)11854);
                Debug.Assert(pack.vd == (short)(short)16919);
            };
            GroundControl.HIL_GPS p113 = CommunicationChannel.new_HIL_GPS();
            PH.setPack(p113);
            p113.vd = (short)(short)16919;
            p113.vn = (short)(short) -4528;
            p113.ve = (short)(short)2742;
            p113.cog = (ushort)(ushort)14217;
            p113.lon = (int) -1651711670;
            p113.eph = (ushort)(ushort)21724;
            p113.lat = (int)1833958593;
            p113.vel = (ushort)(ushort)49848;
            p113.fix_type = (byte)(byte)85;
            p113.satellites_visible = (byte)(byte)30;
            p113.time_usec = (ulong)610684699306545453L;
            p113.alt = (int) -1583963432;
            p113.epv = (ushort)(ushort)11854;
            CommunicationChannel.instance.send(p113);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_OPTICAL_FLOWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_delta_distance_us == (uint)1485349461U);
                Debug.Assert(pack.integrated_ygyro == (float) -1.7044124E38F);
                Debug.Assert(pack.integrated_y == (float) -3.2379259E38F);
                Debug.Assert(pack.integrated_xgyro == (float) -1.2117845E38F);
                Debug.Assert(pack.distance == (float) -3.148194E38F);
                Debug.Assert(pack.quality == (byte)(byte)55);
                Debug.Assert(pack.temperature == (short)(short) -25061);
                Debug.Assert(pack.time_usec == (ulong)6454790734636730546L);
                Debug.Assert(pack.integrated_x == (float) -6.532966E37F);
                Debug.Assert(pack.sensor_id == (byte)(byte)196);
                Debug.Assert(pack.integrated_zgyro == (float) -2.7789169E38F);
                Debug.Assert(pack.integration_time_us == (uint)780120478U);
            };
            GroundControl.HIL_OPTICAL_FLOW p114 = CommunicationChannel.new_HIL_OPTICAL_FLOW();
            PH.setPack(p114);
            p114.integrated_ygyro = (float) -1.7044124E38F;
            p114.integrated_y = (float) -3.2379259E38F;
            p114.distance = (float) -3.148194E38F;
            p114.quality = (byte)(byte)55;
            p114.integrated_xgyro = (float) -1.2117845E38F;
            p114.integrated_zgyro = (float) -2.7789169E38F;
            p114.sensor_id = (byte)(byte)196;
            p114.integrated_x = (float) -6.532966E37F;
            p114.temperature = (short)(short) -25061;
            p114.time_delta_distance_us = (uint)1485349461U;
            p114.time_usec = (ulong)6454790734636730546L;
            p114.integration_time_us = (uint)780120478U;
            CommunicationChannel.instance.send(p114);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_STATE_QUATERNIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vy == (short)(short) -6656);
                Debug.Assert(pack.time_usec == (ulong)346271224020496318L);
                Debug.Assert(pack.yawspeed == (float) -2.6430998E38F);
                Debug.Assert(pack.alt == (int)1362121051);
                Debug.Assert(pack.attitude_quaternion.SequenceEqual(new float[] {1.0159408E38F, -3.2101686E38F, -2.3985256E38F, 2.6900517E38F}));
                Debug.Assert(pack.rollspeed == (float)8.2511205E37F);
                Debug.Assert(pack.pitchspeed == (float) -1.6201994E38F);
                Debug.Assert(pack.zacc == (short)(short) -23362);
                Debug.Assert(pack.xacc == (short)(short)24554);
                Debug.Assert(pack.vx == (short)(short)26453);
                Debug.Assert(pack.vz == (short)(short) -23395);
                Debug.Assert(pack.yacc == (short)(short)1531);
                Debug.Assert(pack.ind_airspeed == (ushort)(ushort)50322);
                Debug.Assert(pack.lon == (int) -503844800);
                Debug.Assert(pack.true_airspeed == (ushort)(ushort)15981);
                Debug.Assert(pack.lat == (int) -1143690812);
            };
            GroundControl.HIL_STATE_QUATERNION p115 = CommunicationChannel.new_HIL_STATE_QUATERNION();
            PH.setPack(p115);
            p115.time_usec = (ulong)346271224020496318L;
            p115.vz = (short)(short) -23395;
            p115.zacc = (short)(short) -23362;
            p115.attitude_quaternion_SET(new float[] {1.0159408E38F, -3.2101686E38F, -2.3985256E38F, 2.6900517E38F}, 0) ;
            p115.vy = (short)(short) -6656;
            p115.alt = (int)1362121051;
            p115.xacc = (short)(short)24554;
            p115.rollspeed = (float)8.2511205E37F;
            p115.vx = (short)(short)26453;
            p115.pitchspeed = (float) -1.6201994E38F;
            p115.ind_airspeed = (ushort)(ushort)50322;
            p115.lat = (int) -1143690812;
            p115.true_airspeed = (ushort)(ushort)15981;
            p115.yacc = (short)(short)1531;
            p115.lon = (int) -503844800;
            p115.yawspeed = (float) -2.6430998E38F;
            CommunicationChannel.instance.send(p115);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_IMU2Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ymag == (short)(short) -2252);
                Debug.Assert(pack.time_boot_ms == (uint)1760077403U);
                Debug.Assert(pack.xgyro == (short)(short)26525);
                Debug.Assert(pack.zmag == (short)(short)9292);
                Debug.Assert(pack.zacc == (short)(short)29818);
                Debug.Assert(pack.xmag == (short)(short) -761);
                Debug.Assert(pack.xacc == (short)(short)31173);
                Debug.Assert(pack.zgyro == (short)(short) -1683);
                Debug.Assert(pack.ygyro == (short)(short)19448);
                Debug.Assert(pack.yacc == (short)(short)9926);
            };
            GroundControl.SCALED_IMU2 p116 = CommunicationChannel.new_SCALED_IMU2();
            PH.setPack(p116);
            p116.xgyro = (short)(short)26525;
            p116.time_boot_ms = (uint)1760077403U;
            p116.ygyro = (short)(short)19448;
            p116.xacc = (short)(short)31173;
            p116.yacc = (short)(short)9926;
            p116.xmag = (short)(short) -761;
            p116.zmag = (short)(short)9292;
            p116.zgyro = (short)(short) -1683;
            p116.zacc = (short)(short)29818;
            p116.ymag = (short)(short) -2252;
            CommunicationChannel.instance.send(p116);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)46);
                Debug.Assert(pack.target_system == (byte)(byte)179);
                Debug.Assert(pack.end == (ushort)(ushort)24205);
                Debug.Assert(pack.start == (ushort)(ushort)29540);
            };
            GroundControl.LOG_REQUEST_LIST p117 = CommunicationChannel.new_LOG_REQUEST_LIST();
            PH.setPack(p117);
            p117.start = (ushort)(ushort)29540;
            p117.target_system = (byte)(byte)179;
            p117.end = (ushort)(ushort)24205;
            p117.target_component = (byte)(byte)46;
            CommunicationChannel.instance.send(p117);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_ENTRYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.num_logs == (ushort)(ushort)19151);
                Debug.Assert(pack.size == (uint)252793893U);
                Debug.Assert(pack.id == (ushort)(ushort)4786);
                Debug.Assert(pack.last_log_num == (ushort)(ushort)32376);
                Debug.Assert(pack.time_utc == (uint)3733714616U);
            };
            GroundControl.LOG_ENTRY p118 = CommunicationChannel.new_LOG_ENTRY();
            PH.setPack(p118);
            p118.num_logs = (ushort)(ushort)19151;
            p118.time_utc = (uint)3733714616U;
            p118.last_log_num = (ushort)(ushort)32376;
            p118.id = (ushort)(ushort)4786;
            p118.size = (uint)252793893U;
            CommunicationChannel.instance.send(p118);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_REQUEST_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)123);
                Debug.Assert(pack.target_system == (byte)(byte)172);
                Debug.Assert(pack.ofs == (uint)505246413U);
                Debug.Assert(pack.id == (ushort)(ushort)64552);
                Debug.Assert(pack.count == (uint)3162092676U);
            };
            GroundControl.LOG_REQUEST_DATA p119 = CommunicationChannel.new_LOG_REQUEST_DATA();
            PH.setPack(p119);
            p119.target_component = (byte)(byte)123;
            p119.ofs = (uint)505246413U;
            p119.target_system = (byte)(byte)172;
            p119.id = (ushort)(ushort)64552;
            p119.count = (uint)3162092676U;
            CommunicationChannel.instance.send(p119);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ofs == (uint)1298619649U);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)4, (byte)176, (byte)94, (byte)218, (byte)242, (byte)231, (byte)119, (byte)196, (byte)147, (byte)21, (byte)190, (byte)186, (byte)84, (byte)35, (byte)54, (byte)47, (byte)99, (byte)228, (byte)225, (byte)72, (byte)21, (byte)213, (byte)131, (byte)173, (byte)233, (byte)19, (byte)81, (byte)15, (byte)214, (byte)19, (byte)73, (byte)38, (byte)221, (byte)216, (byte)42, (byte)36, (byte)0, (byte)109, (byte)33, (byte)204, (byte)129, (byte)88, (byte)221, (byte)56, (byte)144, (byte)245, (byte)17, (byte)45, (byte)102, (byte)133, (byte)42, (byte)103, (byte)82, (byte)217, (byte)246, (byte)133, (byte)74, (byte)54, (byte)229, (byte)50, (byte)87, (byte)105, (byte)37, (byte)209, (byte)47, (byte)162, (byte)173, (byte)0, (byte)43, (byte)160, (byte)234, (byte)135, (byte)121, (byte)174, (byte)83, (byte)177, (byte)45, (byte)215, (byte)210, (byte)249, (byte)30, (byte)176, (byte)123, (byte)228, (byte)6, (byte)174, (byte)45, (byte)93, (byte)247, (byte)197}));
                Debug.Assert(pack.count == (byte)(byte)104);
                Debug.Assert(pack.id == (ushort)(ushort)47202);
            };
            GroundControl.LOG_DATA p120 = CommunicationChannel.new_LOG_DATA();
            PH.setPack(p120);
            p120.id = (ushort)(ushort)47202;
            p120.data__SET(new byte[] {(byte)4, (byte)176, (byte)94, (byte)218, (byte)242, (byte)231, (byte)119, (byte)196, (byte)147, (byte)21, (byte)190, (byte)186, (byte)84, (byte)35, (byte)54, (byte)47, (byte)99, (byte)228, (byte)225, (byte)72, (byte)21, (byte)213, (byte)131, (byte)173, (byte)233, (byte)19, (byte)81, (byte)15, (byte)214, (byte)19, (byte)73, (byte)38, (byte)221, (byte)216, (byte)42, (byte)36, (byte)0, (byte)109, (byte)33, (byte)204, (byte)129, (byte)88, (byte)221, (byte)56, (byte)144, (byte)245, (byte)17, (byte)45, (byte)102, (byte)133, (byte)42, (byte)103, (byte)82, (byte)217, (byte)246, (byte)133, (byte)74, (byte)54, (byte)229, (byte)50, (byte)87, (byte)105, (byte)37, (byte)209, (byte)47, (byte)162, (byte)173, (byte)0, (byte)43, (byte)160, (byte)234, (byte)135, (byte)121, (byte)174, (byte)83, (byte)177, (byte)45, (byte)215, (byte)210, (byte)249, (byte)30, (byte)176, (byte)123, (byte)228, (byte)6, (byte)174, (byte)45, (byte)93, (byte)247, (byte)197}, 0) ;
            p120.count = (byte)(byte)104;
            p120.ofs = (uint)1298619649U;
            CommunicationChannel.instance.send(p120);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_ERASEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)77);
                Debug.Assert(pack.target_component == (byte)(byte)8);
            };
            GroundControl.LOG_ERASE p121 = CommunicationChannel.new_LOG_ERASE();
            PH.setPack(p121);
            p121.target_component = (byte)(byte)8;
            p121.target_system = (byte)(byte)77;
            CommunicationChannel.instance.send(p121);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_REQUEST_ENDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)234);
                Debug.Assert(pack.target_system == (byte)(byte)33);
            };
            GroundControl.LOG_REQUEST_END p122 = CommunicationChannel.new_LOG_REQUEST_END();
            PH.setPack(p122);
            p122.target_system = (byte)(byte)33;
            p122.target_component = (byte)(byte)234;
            CommunicationChannel.instance.send(p122);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_INJECT_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)46, (byte)171, (byte)199, (byte)34, (byte)252, (byte)63, (byte)226, (byte)190, (byte)71, (byte)152, (byte)110, (byte)46, (byte)186, (byte)93, (byte)100, (byte)230, (byte)23, (byte)96, (byte)168, (byte)14, (byte)179, (byte)196, (byte)63, (byte)186, (byte)190, (byte)161, (byte)75, (byte)92, (byte)83, (byte)196, (byte)13, (byte)204, (byte)99, (byte)10, (byte)17, (byte)166, (byte)40, (byte)23, (byte)81, (byte)11, (byte)120, (byte)42, (byte)153, (byte)244, (byte)123, (byte)161, (byte)120, (byte)135, (byte)153, (byte)50, (byte)203, (byte)56, (byte)40, (byte)134, (byte)217, (byte)180, (byte)63, (byte)92, (byte)138, (byte)202, (byte)159, (byte)155, (byte)3, (byte)135, (byte)53, (byte)136, (byte)229, (byte)82, (byte)185, (byte)124, (byte)221, (byte)84, (byte)67, (byte)74, (byte)113, (byte)46, (byte)201, (byte)176, (byte)76, (byte)253, (byte)120, (byte)7, (byte)148, (byte)185, (byte)24, (byte)155, (byte)170, (byte)9, (byte)56, (byte)211, (byte)246, (byte)92, (byte)120, (byte)183, (byte)89, (byte)135, (byte)246, (byte)64, (byte)250, (byte)156, (byte)221, (byte)92, (byte)126, (byte)234, (byte)215, (byte)36, (byte)143, (byte)121, (byte)44, (byte)69}));
                Debug.Assert(pack.target_component == (byte)(byte)36);
                Debug.Assert(pack.len == (byte)(byte)108);
                Debug.Assert(pack.target_system == (byte)(byte)97);
            };
            GroundControl.GPS_INJECT_DATA p123 = CommunicationChannel.new_GPS_INJECT_DATA();
            PH.setPack(p123);
            p123.target_component = (byte)(byte)36;
            p123.len = (byte)(byte)108;
            p123.data__SET(new byte[] {(byte)46, (byte)171, (byte)199, (byte)34, (byte)252, (byte)63, (byte)226, (byte)190, (byte)71, (byte)152, (byte)110, (byte)46, (byte)186, (byte)93, (byte)100, (byte)230, (byte)23, (byte)96, (byte)168, (byte)14, (byte)179, (byte)196, (byte)63, (byte)186, (byte)190, (byte)161, (byte)75, (byte)92, (byte)83, (byte)196, (byte)13, (byte)204, (byte)99, (byte)10, (byte)17, (byte)166, (byte)40, (byte)23, (byte)81, (byte)11, (byte)120, (byte)42, (byte)153, (byte)244, (byte)123, (byte)161, (byte)120, (byte)135, (byte)153, (byte)50, (byte)203, (byte)56, (byte)40, (byte)134, (byte)217, (byte)180, (byte)63, (byte)92, (byte)138, (byte)202, (byte)159, (byte)155, (byte)3, (byte)135, (byte)53, (byte)136, (byte)229, (byte)82, (byte)185, (byte)124, (byte)221, (byte)84, (byte)67, (byte)74, (byte)113, (byte)46, (byte)201, (byte)176, (byte)76, (byte)253, (byte)120, (byte)7, (byte)148, (byte)185, (byte)24, (byte)155, (byte)170, (byte)9, (byte)56, (byte)211, (byte)246, (byte)92, (byte)120, (byte)183, (byte)89, (byte)135, (byte)246, (byte)64, (byte)250, (byte)156, (byte)221, (byte)92, (byte)126, (byte)234, (byte)215, (byte)36, (byte)143, (byte)121, (byte)44, (byte)69}, 0) ;
            p123.target_system = (byte)(byte)97;
            CommunicationChannel.instance.send(p123);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS2_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lat == (int)1577634707);
                Debug.Assert(pack.fix_type == GPS_FIX_TYPE.GPS_FIX_TYPE_PPP);
                Debug.Assert(pack.epv == (ushort)(ushort)14244);
                Debug.Assert(pack.dgps_numch == (byte)(byte)94);
                Debug.Assert(pack.lon == (int)1624887337);
                Debug.Assert(pack.satellites_visible == (byte)(byte)102);
                Debug.Assert(pack.eph == (ushort)(ushort)63154);
                Debug.Assert(pack.time_usec == (ulong)8910937442076553385L);
                Debug.Assert(pack.cog == (ushort)(ushort)63641);
                Debug.Assert(pack.vel == (ushort)(ushort)1572);
                Debug.Assert(pack.alt == (int) -211670080);
                Debug.Assert(pack.dgps_age == (uint)2629970601U);
            };
            GroundControl.GPS2_RAW p124 = CommunicationChannel.new_GPS2_RAW();
            PH.setPack(p124);
            p124.time_usec = (ulong)8910937442076553385L;
            p124.fix_type = GPS_FIX_TYPE.GPS_FIX_TYPE_PPP;
            p124.lon = (int)1624887337;
            p124.dgps_numch = (byte)(byte)94;
            p124.eph = (ushort)(ushort)63154;
            p124.lat = (int)1577634707;
            p124.dgps_age = (uint)2629970601U;
            p124.vel = (ushort)(ushort)1572;
            p124.cog = (ushort)(ushort)63641;
            p124.satellites_visible = (byte)(byte)102;
            p124.epv = (ushort)(ushort)14244;
            p124.alt = (int) -211670080;
            CommunicationChannel.instance.send(p124);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPOWER_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.flags == (MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_OVERCURRENT |
                                            MAV_POWER_STATUS.MAV_POWER_STATUS_USB_CONNECTED |
                                            MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT));
                Debug.Assert(pack.Vcc == (ushort)(ushort)13917);
                Debug.Assert(pack.Vservo == (ushort)(ushort)31967);
            };
            GroundControl.POWER_STATUS p125 = CommunicationChannel.new_POWER_STATUS();
            PH.setPack(p125);
            p125.Vcc = (ushort)(ushort)13917;
            p125.flags = (MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_OVERCURRENT |
                          MAV_POWER_STATUS.MAV_POWER_STATUS_USB_CONNECTED |
                          MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT);
            p125.Vservo = (ushort)(ushort)31967;
            CommunicationChannel.instance.send(p125);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSERIAL_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)220, (byte)41, (byte)39, (byte)137, (byte)6, (byte)223, (byte)30, (byte)210, (byte)245, (byte)236, (byte)162, (byte)15, (byte)233, (byte)129, (byte)54, (byte)67, (byte)167, (byte)34, (byte)169, (byte)240, (byte)34, (byte)165, (byte)79, (byte)240, (byte)7, (byte)74, (byte)6, (byte)24, (byte)145, (byte)102, (byte)205, (byte)63, (byte)36, (byte)3, (byte)189, (byte)89, (byte)46, (byte)219, (byte)36, (byte)26, (byte)59, (byte)246, (byte)243, (byte)156, (byte)113, (byte)184, (byte)249, (byte)23, (byte)156, (byte)227, (byte)207, (byte)199, (byte)146, (byte)203, (byte)155, (byte)248, (byte)116, (byte)239, (byte)229, (byte)134, (byte)248, (byte)217, (byte)140, (byte)222, (byte)105, (byte)66, (byte)29, (byte)98, (byte)58, (byte)23}));
                Debug.Assert(pack.baudrate == (uint)1263926293U);
                Debug.Assert(pack.flags == (SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_MULTI |
                                            SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_RESPOND |
                                            SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_REPLY));
                Debug.Assert(pack.timeout == (ushort)(ushort)905);
                Debug.Assert(pack.device == SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_GPS2);
                Debug.Assert(pack.count == (byte)(byte)240);
            };
            GroundControl.SERIAL_CONTROL p126 = CommunicationChannel.new_SERIAL_CONTROL();
            PH.setPack(p126);
            p126.timeout = (ushort)(ushort)905;
            p126.flags = (SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_MULTI |
                          SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_RESPOND |
                          SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_REPLY);
            p126.data__SET(new byte[] {(byte)220, (byte)41, (byte)39, (byte)137, (byte)6, (byte)223, (byte)30, (byte)210, (byte)245, (byte)236, (byte)162, (byte)15, (byte)233, (byte)129, (byte)54, (byte)67, (byte)167, (byte)34, (byte)169, (byte)240, (byte)34, (byte)165, (byte)79, (byte)240, (byte)7, (byte)74, (byte)6, (byte)24, (byte)145, (byte)102, (byte)205, (byte)63, (byte)36, (byte)3, (byte)189, (byte)89, (byte)46, (byte)219, (byte)36, (byte)26, (byte)59, (byte)246, (byte)243, (byte)156, (byte)113, (byte)184, (byte)249, (byte)23, (byte)156, (byte)227, (byte)207, (byte)199, (byte)146, (byte)203, (byte)155, (byte)248, (byte)116, (byte)239, (byte)229, (byte)134, (byte)248, (byte)217, (byte)140, (byte)222, (byte)105, (byte)66, (byte)29, (byte)98, (byte)58, (byte)23}, 0) ;
            p126.device = SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_GPS2;
            p126.count = (byte)(byte)240;
            p126.baudrate = (uint)1263926293U;
            CommunicationChannel.instance.send(p126);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_RTKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.accuracy == (uint)475123770U);
                Debug.Assert(pack.rtk_health == (byte)(byte)148);
                Debug.Assert(pack.baseline_a_mm == (int)1997149434);
                Debug.Assert(pack.nsats == (byte)(byte)150);
                Debug.Assert(pack.baseline_b_mm == (int)1221913803);
                Debug.Assert(pack.baseline_coords_type == (byte)(byte)111);
                Debug.Assert(pack.wn == (ushort)(ushort)44974);
                Debug.Assert(pack.rtk_receiver_id == (byte)(byte)18);
                Debug.Assert(pack.baseline_c_mm == (int)1570472922);
                Debug.Assert(pack.tow == (uint)3710350643U);
                Debug.Assert(pack.time_last_baseline_ms == (uint)4173014701U);
                Debug.Assert(pack.rtk_rate == (byte)(byte)167);
                Debug.Assert(pack.iar_num_hypotheses == (int) -1832826069);
            };
            GroundControl.GPS_RTK p127 = CommunicationChannel.new_GPS_RTK();
            PH.setPack(p127);
            p127.wn = (ushort)(ushort)44974;
            p127.rtk_rate = (byte)(byte)167;
            p127.rtk_receiver_id = (byte)(byte)18;
            p127.baseline_b_mm = (int)1221913803;
            p127.nsats = (byte)(byte)150;
            p127.time_last_baseline_ms = (uint)4173014701U;
            p127.baseline_a_mm = (int)1997149434;
            p127.rtk_health = (byte)(byte)148;
            p127.baseline_c_mm = (int)1570472922;
            p127.baseline_coords_type = (byte)(byte)111;
            p127.accuracy = (uint)475123770U;
            p127.tow = (uint)3710350643U;
            p127.iar_num_hypotheses = (int) -1832826069;
            CommunicationChannel.instance.send(p127);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS2_RTKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.rtk_receiver_id == (byte)(byte)100);
                Debug.Assert(pack.time_last_baseline_ms == (uint)49572236U);
                Debug.Assert(pack.baseline_a_mm == (int) -1965059545);
                Debug.Assert(pack.iar_num_hypotheses == (int)1319112635);
                Debug.Assert(pack.baseline_c_mm == (int) -1100656371);
                Debug.Assert(pack.accuracy == (uint)2139275197U);
                Debug.Assert(pack.wn == (ushort)(ushort)1110);
                Debug.Assert(pack.rtk_rate == (byte)(byte)67);
                Debug.Assert(pack.baseline_b_mm == (int)935819034);
                Debug.Assert(pack.rtk_health == (byte)(byte)203);
                Debug.Assert(pack.tow == (uint)3700202384U);
                Debug.Assert(pack.nsats == (byte)(byte)4);
                Debug.Assert(pack.baseline_coords_type == (byte)(byte)116);
            };
            GroundControl.GPS2_RTK p128 = CommunicationChannel.new_GPS2_RTK();
            PH.setPack(p128);
            p128.rtk_rate = (byte)(byte)67;
            p128.baseline_coords_type = (byte)(byte)116;
            p128.wn = (ushort)(ushort)1110;
            p128.baseline_a_mm = (int) -1965059545;
            p128.nsats = (byte)(byte)4;
            p128.time_last_baseline_ms = (uint)49572236U;
            p128.baseline_c_mm = (int) -1100656371;
            p128.rtk_receiver_id = (byte)(byte)100;
            p128.rtk_health = (byte)(byte)203;
            p128.baseline_b_mm = (int)935819034;
            p128.accuracy = (uint)2139275197U;
            p128.iar_num_hypotheses = (int)1319112635;
            p128.tow = (uint)3700202384U;
            CommunicationChannel.instance.send(p128);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_IMU3Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)1373070633U);
                Debug.Assert(pack.xacc == (short)(short) -28892);
                Debug.Assert(pack.xmag == (short)(short) -22403);
                Debug.Assert(pack.zgyro == (short)(short) -17039);
                Debug.Assert(pack.zmag == (short)(short) -7412);
                Debug.Assert(pack.yacc == (short)(short)11086);
                Debug.Assert(pack.ymag == (short)(short) -4579);
                Debug.Assert(pack.ygyro == (short)(short) -9136);
                Debug.Assert(pack.xgyro == (short)(short) -29699);
                Debug.Assert(pack.zacc == (short)(short)27480);
            };
            GroundControl.SCALED_IMU3 p129 = CommunicationChannel.new_SCALED_IMU3();
            PH.setPack(p129);
            p129.zmag = (short)(short) -7412;
            p129.time_boot_ms = (uint)1373070633U;
            p129.xacc = (short)(short) -28892;
            p129.zgyro = (short)(short) -17039;
            p129.zacc = (short)(short)27480;
            p129.xmag = (short)(short) -22403;
            p129.ymag = (short)(short) -4579;
            p129.ygyro = (short)(short) -9136;
            p129.xgyro = (short)(short) -29699;
            p129.yacc = (short)(short)11086;
            CommunicationChannel.instance.send(p129);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnDATA_TRANSMISSION_HANDSHAKEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.size == (uint)3106003767U);
                Debug.Assert(pack.height == (ushort)(ushort)29080);
                Debug.Assert(pack.payload == (byte)(byte)209);
                Debug.Assert(pack.width == (ushort)(ushort)65089);
                Debug.Assert(pack.packets == (ushort)(ushort)52053);
                Debug.Assert(pack.jpg_quality == (byte)(byte)195);
                Debug.Assert(pack.type == (byte)(byte)141);
            };
            GroundControl.DATA_TRANSMISSION_HANDSHAKE p130 = CommunicationChannel.new_DATA_TRANSMISSION_HANDSHAKE();
            PH.setPack(p130);
            p130.jpg_quality = (byte)(byte)195;
            p130.width = (ushort)(ushort)65089;
            p130.height = (ushort)(ushort)29080;
            p130.payload = (byte)(byte)209;
            p130.type = (byte)(byte)141;
            p130.size = (uint)3106003767U;
            p130.packets = (ushort)(ushort)52053;
            CommunicationChannel.instance.send(p130);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnENCAPSULATED_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)8, (byte)69, (byte)16, (byte)216, (byte)64, (byte)118, (byte)202, (byte)82, (byte)22, (byte)137, (byte)186, (byte)16, (byte)82, (byte)55, (byte)156, (byte)255, (byte)210, (byte)138, (byte)191, (byte)110, (byte)232, (byte)107, (byte)109, (byte)215, (byte)80, (byte)248, (byte)214, (byte)174, (byte)163, (byte)96, (byte)35, (byte)218, (byte)218, (byte)91, (byte)95, (byte)57, (byte)165, (byte)227, (byte)154, (byte)14, (byte)233, (byte)153, (byte)72, (byte)52, (byte)10, (byte)239, (byte)202, (byte)4, (byte)89, (byte)43, (byte)177, (byte)2, (byte)246, (byte)93, (byte)254, (byte)76, (byte)242, (byte)46, (byte)13, (byte)55, (byte)112, (byte)193, (byte)122, (byte)56, (byte)255, (byte)163, (byte)67, (byte)131, (byte)209, (byte)242, (byte)76, (byte)15, (byte)166, (byte)202, (byte)145, (byte)110, (byte)26, (byte)83, (byte)254, (byte)90, (byte)125, (byte)207, (byte)226, (byte)199, (byte)30, (byte)165, (byte)186, (byte)175, (byte)127, (byte)15, (byte)18, (byte)89, (byte)183, (byte)2, (byte)162, (byte)72, (byte)167, (byte)230, (byte)62, (byte)96, (byte)96, (byte)198, (byte)249, (byte)156, (byte)108, (byte)136, (byte)63, (byte)204, (byte)9, (byte)163, (byte)243, (byte)231, (byte)82, (byte)141, (byte)59, (byte)128, (byte)246, (byte)190, (byte)114, (byte)174, (byte)182, (byte)71, (byte)113, (byte)250, (byte)42, (byte)164, (byte)157, (byte)84, (byte)170, (byte)255, (byte)27, (byte)90, (byte)186, (byte)242, (byte)46, (byte)168, (byte)112, (byte)17, (byte)9, (byte)139, (byte)151, (byte)86, (byte)56, (byte)122, (byte)230, (byte)80, (byte)178, (byte)190, (byte)54, (byte)3, (byte)111, (byte)173, (byte)215, (byte)97, (byte)234, (byte)230, (byte)135, (byte)89, (byte)238, (byte)75, (byte)190, (byte)147, (byte)159, (byte)31, (byte)41, (byte)229, (byte)17, (byte)137, (byte)9, (byte)12, (byte)10, (byte)146, (byte)206, (byte)195, (byte)83, (byte)177, (byte)156, (byte)99, (byte)207, (byte)98, (byte)119, (byte)226, (byte)185, (byte)29, (byte)115, (byte)58, (byte)202, (byte)106, (byte)133, (byte)48, (byte)71, (byte)185, (byte)23, (byte)157, (byte)7, (byte)88, (byte)111, (byte)207, (byte)125, (byte)135, (byte)59, (byte)9, (byte)206, (byte)135, (byte)91, (byte)66, (byte)77, (byte)37, (byte)65, (byte)147, (byte)237, (byte)88, (byte)197, (byte)61, (byte)189, (byte)238, (byte)249, (byte)232, (byte)78, (byte)117, (byte)155, (byte)168, (byte)14, (byte)246, (byte)22, (byte)214, (byte)249, (byte)109, (byte)111, (byte)8, (byte)99, (byte)88, (byte)29, (byte)18, (byte)175, (byte)199, (byte)62, (byte)98, (byte)3, (byte)31, (byte)78, (byte)75, (byte)173, (byte)217, (byte)99, (byte)132, (byte)15, (byte)147, (byte)253, (byte)249, (byte)195, (byte)142, (byte)36}));
                Debug.Assert(pack.seqnr == (ushort)(ushort)3979);
            };
            GroundControl.ENCAPSULATED_DATA p131 = CommunicationChannel.new_ENCAPSULATED_DATA();
            PH.setPack(p131);
            p131.data__SET(new byte[] {(byte)8, (byte)69, (byte)16, (byte)216, (byte)64, (byte)118, (byte)202, (byte)82, (byte)22, (byte)137, (byte)186, (byte)16, (byte)82, (byte)55, (byte)156, (byte)255, (byte)210, (byte)138, (byte)191, (byte)110, (byte)232, (byte)107, (byte)109, (byte)215, (byte)80, (byte)248, (byte)214, (byte)174, (byte)163, (byte)96, (byte)35, (byte)218, (byte)218, (byte)91, (byte)95, (byte)57, (byte)165, (byte)227, (byte)154, (byte)14, (byte)233, (byte)153, (byte)72, (byte)52, (byte)10, (byte)239, (byte)202, (byte)4, (byte)89, (byte)43, (byte)177, (byte)2, (byte)246, (byte)93, (byte)254, (byte)76, (byte)242, (byte)46, (byte)13, (byte)55, (byte)112, (byte)193, (byte)122, (byte)56, (byte)255, (byte)163, (byte)67, (byte)131, (byte)209, (byte)242, (byte)76, (byte)15, (byte)166, (byte)202, (byte)145, (byte)110, (byte)26, (byte)83, (byte)254, (byte)90, (byte)125, (byte)207, (byte)226, (byte)199, (byte)30, (byte)165, (byte)186, (byte)175, (byte)127, (byte)15, (byte)18, (byte)89, (byte)183, (byte)2, (byte)162, (byte)72, (byte)167, (byte)230, (byte)62, (byte)96, (byte)96, (byte)198, (byte)249, (byte)156, (byte)108, (byte)136, (byte)63, (byte)204, (byte)9, (byte)163, (byte)243, (byte)231, (byte)82, (byte)141, (byte)59, (byte)128, (byte)246, (byte)190, (byte)114, (byte)174, (byte)182, (byte)71, (byte)113, (byte)250, (byte)42, (byte)164, (byte)157, (byte)84, (byte)170, (byte)255, (byte)27, (byte)90, (byte)186, (byte)242, (byte)46, (byte)168, (byte)112, (byte)17, (byte)9, (byte)139, (byte)151, (byte)86, (byte)56, (byte)122, (byte)230, (byte)80, (byte)178, (byte)190, (byte)54, (byte)3, (byte)111, (byte)173, (byte)215, (byte)97, (byte)234, (byte)230, (byte)135, (byte)89, (byte)238, (byte)75, (byte)190, (byte)147, (byte)159, (byte)31, (byte)41, (byte)229, (byte)17, (byte)137, (byte)9, (byte)12, (byte)10, (byte)146, (byte)206, (byte)195, (byte)83, (byte)177, (byte)156, (byte)99, (byte)207, (byte)98, (byte)119, (byte)226, (byte)185, (byte)29, (byte)115, (byte)58, (byte)202, (byte)106, (byte)133, (byte)48, (byte)71, (byte)185, (byte)23, (byte)157, (byte)7, (byte)88, (byte)111, (byte)207, (byte)125, (byte)135, (byte)59, (byte)9, (byte)206, (byte)135, (byte)91, (byte)66, (byte)77, (byte)37, (byte)65, (byte)147, (byte)237, (byte)88, (byte)197, (byte)61, (byte)189, (byte)238, (byte)249, (byte)232, (byte)78, (byte)117, (byte)155, (byte)168, (byte)14, (byte)246, (byte)22, (byte)214, (byte)249, (byte)109, (byte)111, (byte)8, (byte)99, (byte)88, (byte)29, (byte)18, (byte)175, (byte)199, (byte)62, (byte)98, (byte)3, (byte)31, (byte)78, (byte)75, (byte)173, (byte)217, (byte)99, (byte)132, (byte)15, (byte)147, (byte)253, (byte)249, (byte)195, (byte)142, (byte)36}, 0) ;
            p131.seqnr = (ushort)(ushort)3979;
            CommunicationChannel.instance.send(p131);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnDISTANCE_SENSORReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.covariance == (byte)(byte)79);
                Debug.Assert(pack.time_boot_ms == (uint)131119040U);
                Debug.Assert(pack.id == (byte)(byte)80);
                Debug.Assert(pack.type == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_LASER);
                Debug.Assert(pack.min_distance == (ushort)(ushort)10671);
                Debug.Assert(pack.current_distance == (ushort)(ushort)13863);
                Debug.Assert(pack.orientation == MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_YAW_135);
                Debug.Assert(pack.max_distance == (ushort)(ushort)5088);
            };
            GroundControl.DISTANCE_SENSOR p132 = CommunicationChannel.new_DISTANCE_SENSOR();
            PH.setPack(p132);
            p132.id = (byte)(byte)80;
            p132.min_distance = (ushort)(ushort)10671;
            p132.time_boot_ms = (uint)131119040U;
            p132.current_distance = (ushort)(ushort)13863;
            p132.orientation = MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_YAW_135;
            p132.type = MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_LASER;
            p132.covariance = (byte)(byte)79;
            p132.max_distance = (ushort)(ushort)5088;
            CommunicationChannel.instance.send(p132);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTERRAIN_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lon == (int)909040730);
                Debug.Assert(pack.mask == (ulong)5025813004490915959L);
                Debug.Assert(pack.grid_spacing == (ushort)(ushort)37002);
                Debug.Assert(pack.lat == (int)1106974937);
            };
            GroundControl.TERRAIN_REQUEST p133 = CommunicationChannel.new_TERRAIN_REQUEST();
            PH.setPack(p133);
            p133.grid_spacing = (ushort)(ushort)37002;
            p133.mask = (ulong)5025813004490915959L;
            p133.lat = (int)1106974937;
            p133.lon = (int)909040730;
            CommunicationChannel.instance.send(p133);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTERRAIN_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.data_.SequenceEqual(new short[] {(short)32616, (short) -21668, (short)22448, (short)13548, (short) -21966, (short)28338, (short)12540, (short) -13489, (short) -24623, (short)27552, (short) -976, (short)18391, (short) -19749, (short)25702, (short)23125, (short)11486}));
                Debug.Assert(pack.gridbit == (byte)(byte)42);
                Debug.Assert(pack.lon == (int)219222245);
                Debug.Assert(pack.lat == (int) -168046801);
                Debug.Assert(pack.grid_spacing == (ushort)(ushort)24033);
            };
            GroundControl.TERRAIN_DATA p134 = CommunicationChannel.new_TERRAIN_DATA();
            PH.setPack(p134);
            p134.data__SET(new short[] {(short)32616, (short) -21668, (short)22448, (short)13548, (short) -21966, (short)28338, (short)12540, (short) -13489, (short) -24623, (short)27552, (short) -976, (short)18391, (short) -19749, (short)25702, (short)23125, (short)11486}, 0) ;
            p134.gridbit = (byte)(byte)42;
            p134.grid_spacing = (ushort)(ushort)24033;
            p134.lat = (int) -168046801;
            p134.lon = (int)219222245;
            CommunicationChannel.instance.send(p134);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTERRAIN_CHECKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lat == (int)1523581280);
                Debug.Assert(pack.lon == (int) -943149001);
            };
            GroundControl.TERRAIN_CHECK p135 = CommunicationChannel.new_TERRAIN_CHECK();
            PH.setPack(p135);
            p135.lat = (int)1523581280;
            p135.lon = (int) -943149001;
            CommunicationChannel.instance.send(p135);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTERRAIN_REPORTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.terrain_height == (float)7.7481173E37F);
                Debug.Assert(pack.loaded == (ushort)(ushort)12110);
                Debug.Assert(pack.spacing == (ushort)(ushort)44179);
                Debug.Assert(pack.pending == (ushort)(ushort)63196);
                Debug.Assert(pack.lat == (int)1474262490);
                Debug.Assert(pack.current_height == (float)9.096218E37F);
                Debug.Assert(pack.lon == (int)1980093128);
            };
            GroundControl.TERRAIN_REPORT p136 = CommunicationChannel.new_TERRAIN_REPORT();
            PH.setPack(p136);
            p136.pending = (ushort)(ushort)63196;
            p136.current_height = (float)9.096218E37F;
            p136.spacing = (ushort)(ushort)44179;
            p136.loaded = (ushort)(ushort)12110;
            p136.lat = (int)1474262490;
            p136.terrain_height = (float)7.7481173E37F;
            p136.lon = (int)1980093128;
            CommunicationChannel.instance.send(p136);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_PRESSURE2Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.temperature == (short)(short) -8354);
                Debug.Assert(pack.time_boot_ms == (uint)81287993U);
                Debug.Assert(pack.press_abs == (float)1.096146E38F);
                Debug.Assert(pack.press_diff == (float) -2.8549648E38F);
            };
            GroundControl.SCALED_PRESSURE2 p137 = CommunicationChannel.new_SCALED_PRESSURE2();
            PH.setPack(p137);
            p137.temperature = (short)(short) -8354;
            p137.press_abs = (float)1.096146E38F;
            p137.time_boot_ms = (uint)81287993U;
            p137.press_diff = (float) -2.8549648E38F;
            CommunicationChannel.instance.send(p137);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATT_POS_MOCAPReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.x == (float)2.8550248E38F);
                Debug.Assert(pack.time_usec == (ulong)5833655074342331684L);
                Debug.Assert(pack.z == (float)4.0871347E36F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-3.9284753E37F, -1.9512271E37F, -3.308054E38F, 5.498204E37F}));
                Debug.Assert(pack.y == (float)6.8615047E37F);
            };
            GroundControl.ATT_POS_MOCAP p138 = CommunicationChannel.new_ATT_POS_MOCAP();
            PH.setPack(p138);
            p138.y = (float)6.8615047E37F;
            p138.z = (float)4.0871347E36F;
            p138.q_SET(new float[] {-3.9284753E37F, -1.9512271E37F, -3.308054E38F, 5.498204E37F}, 0) ;
            p138.time_usec = (ulong)5833655074342331684L;
            p138.x = (float)2.8550248E38F;
            CommunicationChannel.instance.send(p138);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_ACTUATOR_CONTROL_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)159);
                Debug.Assert(pack.controls.SequenceEqual(new float[] {-1.570754E38F, 8.674378E37F, 2.8991119E38F, 2.5511321E37F, 1.4782629E38F, -5.955929E37F, 2.5573066E38F, -3.1767552E37F}));
                Debug.Assert(pack.target_system == (byte)(byte)229);
                Debug.Assert(pack.group_mlx == (byte)(byte)66);
                Debug.Assert(pack.time_usec == (ulong)2018807222995401755L);
            };
            GroundControl.SET_ACTUATOR_CONTROL_TARGET p139 = CommunicationChannel.new_SET_ACTUATOR_CONTROL_TARGET();
            PH.setPack(p139);
            p139.target_system = (byte)(byte)229;
            p139.controls_SET(new float[] {-1.570754E38F, 8.674378E37F, 2.8991119E38F, 2.5511321E37F, 1.4782629E38F, -5.955929E37F, 2.5573066E38F, -3.1767552E37F}, 0) ;
            p139.time_usec = (ulong)2018807222995401755L;
            p139.group_mlx = (byte)(byte)66;
            p139.target_component = (byte)(byte)159;
            CommunicationChannel.instance.send(p139);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnACTUATOR_CONTROL_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.group_mlx == (byte)(byte)45);
                Debug.Assert(pack.time_usec == (ulong)1663298794438432398L);
                Debug.Assert(pack.controls.SequenceEqual(new float[] {6.1535284E36F, -1.3218191E38F, -1.6215748E38F, -4.7763143E37F, -8.1572814E37F, 2.0505788E38F, 2.121281E38F, -1.823982E38F}));
            };
            GroundControl.ACTUATOR_CONTROL_TARGET p140 = CommunicationChannel.new_ACTUATOR_CONTROL_TARGET();
            PH.setPack(p140);
            p140.time_usec = (ulong)1663298794438432398L;
            p140.group_mlx = (byte)(byte)45;
            p140.controls_SET(new float[] {6.1535284E36F, -1.3218191E38F, -1.6215748E38F, -4.7763143E37F, -8.1572814E37F, 2.0505788E38F, 2.121281E38F, -1.823982E38F}, 0) ;
            CommunicationChannel.instance.send(p140);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnALTITUDEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)1430255960188902611L);
                Debug.Assert(pack.altitude_amsl == (float) -1.3824569E38F);
                Debug.Assert(pack.bottom_clearance == (float)2.3371311E38F);
                Debug.Assert(pack.altitude_relative == (float) -1.9816212E38F);
                Debug.Assert(pack.altitude_terrain == (float)2.1483593E36F);
                Debug.Assert(pack.altitude_local == (float) -1.2635044E37F);
                Debug.Assert(pack.altitude_monotonic == (float) -9.224315E37F);
            };
            GroundControl.ALTITUDE p141 = CommunicationChannel.new_ALTITUDE();
            PH.setPack(p141);
            p141.bottom_clearance = (float)2.3371311E38F;
            p141.altitude_relative = (float) -1.9816212E38F;
            p141.altitude_monotonic = (float) -9.224315E37F;
            p141.altitude_amsl = (float) -1.3824569E38F;
            p141.time_usec = (ulong)1430255960188902611L;
            p141.altitude_local = (float) -1.2635044E37F;
            p141.altitude_terrain = (float)2.1483593E36F;
            CommunicationChannel.instance.send(p141);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRESOURCE_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.uri.SequenceEqual(new byte[] {(byte)34, (byte)165, (byte)38, (byte)243, (byte)104, (byte)135, (byte)52, (byte)89, (byte)19, (byte)139, (byte)45, (byte)41, (byte)5, (byte)195, (byte)174, (byte)206, (byte)153, (byte)188, (byte)200, (byte)137, (byte)66, (byte)103, (byte)70, (byte)215, (byte)62, (byte)62, (byte)162, (byte)73, (byte)16, (byte)49, (byte)126, (byte)15, (byte)166, (byte)182, (byte)54, (byte)89, (byte)188, (byte)187, (byte)96, (byte)237, (byte)175, (byte)192, (byte)208, (byte)87, (byte)242, (byte)44, (byte)232, (byte)75, (byte)60, (byte)22, (byte)26, (byte)57, (byte)41, (byte)0, (byte)144, (byte)137, (byte)252, (byte)203, (byte)246, (byte)198, (byte)154, (byte)126, (byte)236, (byte)210, (byte)226, (byte)3, (byte)192, (byte)38, (byte)182, (byte)217, (byte)91, (byte)164, (byte)19, (byte)100, (byte)83, (byte)209, (byte)254, (byte)70, (byte)5, (byte)236, (byte)211, (byte)58, (byte)131, (byte)115, (byte)178, (byte)103, (byte)168, (byte)163, (byte)197, (byte)80, (byte)106, (byte)45, (byte)51, (byte)6, (byte)148, (byte)222, (byte)115, (byte)198, (byte)34, (byte)10, (byte)24, (byte)110, (byte)21, (byte)10, (byte)79, (byte)45, (byte)237, (byte)113, (byte)234, (byte)97, (byte)63, (byte)180, (byte)54, (byte)151, (byte)165, (byte)12, (byte)219, (byte)192, (byte)93, (byte)238}));
                Debug.Assert(pack.uri_type == (byte)(byte)138);
                Debug.Assert(pack.request_id == (byte)(byte)28);
                Debug.Assert(pack.storage.SequenceEqual(new byte[] {(byte)115, (byte)155, (byte)193, (byte)190, (byte)68, (byte)101, (byte)185, (byte)78, (byte)86, (byte)32, (byte)212, (byte)53, (byte)195, (byte)148, (byte)211, (byte)33, (byte)73, (byte)139, (byte)32, (byte)27, (byte)56, (byte)74, (byte)153, (byte)125, (byte)216, (byte)213, (byte)124, (byte)237, (byte)96, (byte)205, (byte)160, (byte)251, (byte)170, (byte)158, (byte)42, (byte)103, (byte)231, (byte)78, (byte)114, (byte)36, (byte)40, (byte)159, (byte)25, (byte)168, (byte)110, (byte)72, (byte)58, (byte)112, (byte)246, (byte)66, (byte)222, (byte)171, (byte)82, (byte)238, (byte)101, (byte)6, (byte)214, (byte)27, (byte)231, (byte)244, (byte)105, (byte)129, (byte)46, (byte)51, (byte)79, (byte)187, (byte)147, (byte)38, (byte)232, (byte)157, (byte)29, (byte)2, (byte)181, (byte)10, (byte)190, (byte)0, (byte)153, (byte)76, (byte)199, (byte)129, (byte)74, (byte)137, (byte)181, (byte)249, (byte)83, (byte)235, (byte)22, (byte)217, (byte)73, (byte)78, (byte)177, (byte)56, (byte)86, (byte)237, (byte)78, (byte)87, (byte)230, (byte)89, (byte)209, (byte)154, (byte)173, (byte)76, (byte)249, (byte)83, (byte)199, (byte)87, (byte)197, (byte)174, (byte)119, (byte)253, (byte)30, (byte)8, (byte)126, (byte)240, (byte)29, (byte)103, (byte)243, (byte)81, (byte)33, (byte)89}));
                Debug.Assert(pack.transfer_type == (byte)(byte)51);
            };
            GroundControl.RESOURCE_REQUEST p142 = CommunicationChannel.new_RESOURCE_REQUEST();
            PH.setPack(p142);
            p142.storage_SET(new byte[] {(byte)115, (byte)155, (byte)193, (byte)190, (byte)68, (byte)101, (byte)185, (byte)78, (byte)86, (byte)32, (byte)212, (byte)53, (byte)195, (byte)148, (byte)211, (byte)33, (byte)73, (byte)139, (byte)32, (byte)27, (byte)56, (byte)74, (byte)153, (byte)125, (byte)216, (byte)213, (byte)124, (byte)237, (byte)96, (byte)205, (byte)160, (byte)251, (byte)170, (byte)158, (byte)42, (byte)103, (byte)231, (byte)78, (byte)114, (byte)36, (byte)40, (byte)159, (byte)25, (byte)168, (byte)110, (byte)72, (byte)58, (byte)112, (byte)246, (byte)66, (byte)222, (byte)171, (byte)82, (byte)238, (byte)101, (byte)6, (byte)214, (byte)27, (byte)231, (byte)244, (byte)105, (byte)129, (byte)46, (byte)51, (byte)79, (byte)187, (byte)147, (byte)38, (byte)232, (byte)157, (byte)29, (byte)2, (byte)181, (byte)10, (byte)190, (byte)0, (byte)153, (byte)76, (byte)199, (byte)129, (byte)74, (byte)137, (byte)181, (byte)249, (byte)83, (byte)235, (byte)22, (byte)217, (byte)73, (byte)78, (byte)177, (byte)56, (byte)86, (byte)237, (byte)78, (byte)87, (byte)230, (byte)89, (byte)209, (byte)154, (byte)173, (byte)76, (byte)249, (byte)83, (byte)199, (byte)87, (byte)197, (byte)174, (byte)119, (byte)253, (byte)30, (byte)8, (byte)126, (byte)240, (byte)29, (byte)103, (byte)243, (byte)81, (byte)33, (byte)89}, 0) ;
            p142.uri_type = (byte)(byte)138;
            p142.uri_SET(new byte[] {(byte)34, (byte)165, (byte)38, (byte)243, (byte)104, (byte)135, (byte)52, (byte)89, (byte)19, (byte)139, (byte)45, (byte)41, (byte)5, (byte)195, (byte)174, (byte)206, (byte)153, (byte)188, (byte)200, (byte)137, (byte)66, (byte)103, (byte)70, (byte)215, (byte)62, (byte)62, (byte)162, (byte)73, (byte)16, (byte)49, (byte)126, (byte)15, (byte)166, (byte)182, (byte)54, (byte)89, (byte)188, (byte)187, (byte)96, (byte)237, (byte)175, (byte)192, (byte)208, (byte)87, (byte)242, (byte)44, (byte)232, (byte)75, (byte)60, (byte)22, (byte)26, (byte)57, (byte)41, (byte)0, (byte)144, (byte)137, (byte)252, (byte)203, (byte)246, (byte)198, (byte)154, (byte)126, (byte)236, (byte)210, (byte)226, (byte)3, (byte)192, (byte)38, (byte)182, (byte)217, (byte)91, (byte)164, (byte)19, (byte)100, (byte)83, (byte)209, (byte)254, (byte)70, (byte)5, (byte)236, (byte)211, (byte)58, (byte)131, (byte)115, (byte)178, (byte)103, (byte)168, (byte)163, (byte)197, (byte)80, (byte)106, (byte)45, (byte)51, (byte)6, (byte)148, (byte)222, (byte)115, (byte)198, (byte)34, (byte)10, (byte)24, (byte)110, (byte)21, (byte)10, (byte)79, (byte)45, (byte)237, (byte)113, (byte)234, (byte)97, (byte)63, (byte)180, (byte)54, (byte)151, (byte)165, (byte)12, (byte)219, (byte)192, (byte)93, (byte)238}, 0) ;
            p142.transfer_type = (byte)(byte)51;
            p142.request_id = (byte)(byte)28;
            CommunicationChannel.instance.send(p142);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_PRESSURE3Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.press_diff == (float)1.0909202E38F);
                Debug.Assert(pack.temperature == (short)(short) -22776);
                Debug.Assert(pack.time_boot_ms == (uint)3135622296U);
                Debug.Assert(pack.press_abs == (float)2.8311128E38F);
            };
            GroundControl.SCALED_PRESSURE3 p143 = CommunicationChannel.new_SCALED_PRESSURE3();
            PH.setPack(p143);
            p143.temperature = (short)(short) -22776;
            p143.press_diff = (float)1.0909202E38F;
            p143.press_abs = (float)2.8311128E38F;
            p143.time_boot_ms = (uint)3135622296U;
            CommunicationChannel.instance.send(p143);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnFOLLOW_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.acc.SequenceEqual(new float[] {-6.422682E37F, 1.4290472E38F, -4.8099738E36F}));
                Debug.Assert(pack.lat == (int) -284818445);
                Debug.Assert(pack.alt == (float) -1.8756673E38F);
                Debug.Assert(pack.est_capabilities == (byte)(byte)105);
                Debug.Assert(pack.custom_state == (ulong)8590562920809016685L);
                Debug.Assert(pack.rates.SequenceEqual(new float[] {-2.389113E38F, -2.0366715E38F, -1.4893487E37F}));
                Debug.Assert(pack.attitude_q.SequenceEqual(new float[] {-1.0535906E38F, 2.3321852E38F, 1.0404321E37F, -2.0595702E38F}));
                Debug.Assert(pack.vel.SequenceEqual(new float[] {-1.8123173E38F, -3.0364341E38F, -1.3475683E38F}));
                Debug.Assert(pack.lon == (int)1127684214);
                Debug.Assert(pack.timestamp == (ulong)7144670659852323504L);
                Debug.Assert(pack.position_cov.SequenceEqual(new float[] {-1.5444474E38F, -1.8109925E37F, 3.0541863E38F}));
            };
            GroundControl.FOLLOW_TARGET p144 = CommunicationChannel.new_FOLLOW_TARGET();
            PH.setPack(p144);
            p144.rates_SET(new float[] {-2.389113E38F, -2.0366715E38F, -1.4893487E37F}, 0) ;
            p144.position_cov_SET(new float[] {-1.5444474E38F, -1.8109925E37F, 3.0541863E38F}, 0) ;
            p144.est_capabilities = (byte)(byte)105;
            p144.lat = (int) -284818445;
            p144.alt = (float) -1.8756673E38F;
            p144.vel_SET(new float[] {-1.8123173E38F, -3.0364341E38F, -1.3475683E38F}, 0) ;
            p144.lon = (int)1127684214;
            p144.timestamp = (ulong)7144670659852323504L;
            p144.custom_state = (ulong)8590562920809016685L;
            p144.acc_SET(new float[] {-6.422682E37F, 1.4290472E38F, -4.8099738E36F}, 0) ;
            p144.attitude_q_SET(new float[] {-1.0535906E38F, 2.3321852E38F, 1.0404321E37F, -2.0595702E38F}, 0) ;
            CommunicationChannel.instance.send(p144);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCONTROL_SYSTEM_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.roll_rate == (float) -2.7478304E38F);
                Debug.Assert(pack.x_vel == (float)1.4547591E38F);
                Debug.Assert(pack.z_pos == (float) -2.545065E38F);
                Debug.Assert(pack.y_acc == (float)2.826889E38F);
                Debug.Assert(pack.x_acc == (float) -6.7589954E37F);
                Debug.Assert(pack.yaw_rate == (float) -1.5762305E38F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-1.2169129E38F, 2.25343E38F, -2.3473017E38F, 2.4590087E38F}));
                Debug.Assert(pack.z_vel == (float) -3.0236448E38F);
                Debug.Assert(pack.pos_variance.SequenceEqual(new float[] {6.3438413E37F, -1.5205175E38F, -3.3674542E38F}));
                Debug.Assert(pack.y_vel == (float)1.9913554E38F);
                Debug.Assert(pack.z_acc == (float) -4.3933585E37F);
                Debug.Assert(pack.x_pos == (float)1.4314396E38F);
                Debug.Assert(pack.time_usec == (ulong)7281621481508914547L);
                Debug.Assert(pack.airspeed == (float) -2.1660725E38F);
                Debug.Assert(pack.y_pos == (float) -2.287363E38F);
                Debug.Assert(pack.vel_variance.SequenceEqual(new float[] {4.6791387E37F, 9.283258E37F, 1.0378929E38F}));
                Debug.Assert(pack.pitch_rate == (float) -2.034721E38F);
            };
            GroundControl.CONTROL_SYSTEM_STATE p146 = CommunicationChannel.new_CONTROL_SYSTEM_STATE();
            PH.setPack(p146);
            p146.pos_variance_SET(new float[] {6.3438413E37F, -1.5205175E38F, -3.3674542E38F}, 0) ;
            p146.z_acc = (float) -4.3933585E37F;
            p146.time_usec = (ulong)7281621481508914547L;
            p146.airspeed = (float) -2.1660725E38F;
            p146.x_vel = (float)1.4547591E38F;
            p146.pitch_rate = (float) -2.034721E38F;
            p146.q_SET(new float[] {-1.2169129E38F, 2.25343E38F, -2.3473017E38F, 2.4590087E38F}, 0) ;
            p146.y_acc = (float)2.826889E38F;
            p146.y_pos = (float) -2.287363E38F;
            p146.x_acc = (float) -6.7589954E37F;
            p146.z_pos = (float) -2.545065E38F;
            p146.z_vel = (float) -3.0236448E38F;
            p146.yaw_rate = (float) -1.5762305E38F;
            p146.x_pos = (float)1.4314396E38F;
            p146.y_vel = (float)1.9913554E38F;
            p146.roll_rate = (float) -2.7478304E38F;
            p146.vel_variance_SET(new float[] {4.6791387E37F, 9.283258E37F, 1.0378929E38F}, 0) ;
            CommunicationChannel.instance.send(p146);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnBATTERY_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.energy_consumed == (int)1557946143);
                Debug.Assert(pack.current_consumed == (int)413087016);
                Debug.Assert(pack.type == MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_LION);
                Debug.Assert(pack.battery_function == MAV_BATTERY_FUNCTION.MAV_BATTERY_TYPE_PAYLOAD);
                Debug.Assert(pack.current_battery == (short)(short) -5685);
                Debug.Assert(pack.battery_remaining == (sbyte)(sbyte)47);
                Debug.Assert(pack.temperature == (short)(short) -26376);
                Debug.Assert(pack.voltages.SequenceEqual(new ushort[] {(ushort)48500, (ushort)50267, (ushort)21139, (ushort)11757, (ushort)18922, (ushort)63794, (ushort)38170, (ushort)54855, (ushort)51034, (ushort)41555}));
                Debug.Assert(pack.id == (byte)(byte)95);
            };
            GroundControl.BATTERY_STATUS p147 = CommunicationChannel.new_BATTERY_STATUS();
            PH.setPack(p147);
            p147.battery_function = MAV_BATTERY_FUNCTION.MAV_BATTERY_TYPE_PAYLOAD;
            p147.battery_remaining = (sbyte)(sbyte)47;
            p147.current_consumed = (int)413087016;
            p147.type = MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_LION;
            p147.id = (byte)(byte)95;
            p147.energy_consumed = (int)1557946143;
            p147.voltages_SET(new ushort[] {(ushort)48500, (ushort)50267, (ushort)21139, (ushort)11757, (ushort)18922, (ushort)63794, (ushort)38170, (ushort)54855, (ushort)51034, (ushort)41555}, 0) ;
            p147.current_battery = (short)(short) -5685;
            p147.temperature = (short)(short) -26376;
            CommunicationChannel.instance.send(p147);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnAUTOPILOT_VERSIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.uid == (ulong)1591586801121562340L);
                Debug.Assert(pack.flight_sw_version == (uint)1305869284U);
                Debug.Assert(pack.board_version == (uint)3307772572U);
                Debug.Assert(pack.capabilities == (MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET |
                                                   MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED |
                                                   MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_PARAM_UNION |
                                                   MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT |
                                                   MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_TERRAIN |
                                                   MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION |
                                                   MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_FENCE));
                Debug.Assert(pack.os_sw_version == (uint)4136421274U);
                Debug.Assert(pack.middleware_custom_version.SequenceEqual(new byte[] {(byte)121, (byte)103, (byte)151, (byte)6, (byte)209, (byte)167, (byte)229, (byte)44}));
                Debug.Assert(pack.flight_custom_version.SequenceEqual(new byte[] {(byte)175, (byte)162, (byte)148, (byte)93, (byte)18, (byte)101, (byte)54, (byte)27}));
                Debug.Assert(pack.vendor_id == (ushort)(ushort)29144);
                Debug.Assert(pack.product_id == (ushort)(ushort)24236);
                Debug.Assert(pack.middleware_sw_version == (uint)3469563935U);
                Debug.Assert(pack.os_custom_version.SequenceEqual(new byte[] {(byte)229, (byte)104, (byte)172, (byte)71, (byte)224, (byte)99, (byte)103, (byte)173}));
                Debug.Assert(pack.uid2_TRY(ph).SequenceEqual(new byte[] {(byte)44, (byte)148, (byte)250, (byte)53, (byte)120, (byte)130, (byte)130, (byte)4, (byte)191, (byte)48, (byte)84, (byte)138, (byte)67, (byte)35, (byte)8, (byte)249, (byte)89, (byte)204}));
            };
            GroundControl.AUTOPILOT_VERSION p148 = CommunicationChannel.new_AUTOPILOT_VERSION();
            PH.setPack(p148);
            p148.os_sw_version = (uint)4136421274U;
            p148.uid2_SET(new byte[] {(byte)44, (byte)148, (byte)250, (byte)53, (byte)120, (byte)130, (byte)130, (byte)4, (byte)191, (byte)48, (byte)84, (byte)138, (byte)67, (byte)35, (byte)8, (byte)249, (byte)89, (byte)204}, 0, PH) ;
            p148.uid = (ulong)1591586801121562340L;
            p148.capabilities = (MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_PARAM_UNION |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_TERRAIN |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_FENCE);
            p148.middleware_custom_version_SET(new byte[] {(byte)121, (byte)103, (byte)151, (byte)6, (byte)209, (byte)167, (byte)229, (byte)44}, 0) ;
            p148.flight_custom_version_SET(new byte[] {(byte)175, (byte)162, (byte)148, (byte)93, (byte)18, (byte)101, (byte)54, (byte)27}, 0) ;
            p148.os_custom_version_SET(new byte[] {(byte)229, (byte)104, (byte)172, (byte)71, (byte)224, (byte)99, (byte)103, (byte)173}, 0) ;
            p148.flight_sw_version = (uint)1305869284U;
            p148.vendor_id = (ushort)(ushort)29144;
            p148.board_version = (uint)3307772572U;
            p148.middleware_sw_version = (uint)3469563935U;
            p148.product_id = (ushort)(ushort)24236;
            CommunicationChannel.instance.send(p148);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLANDING_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_num == (byte)(byte)105);
                Debug.Assert(pack.angle_x == (float) -6.765539E35F);
                Debug.Assert(pack.y_TRY(ph) == (float) -3.2016337E38F);
                Debug.Assert(pack.x_TRY(ph) == (float) -5.2715666E37F);
                Debug.Assert(pack.distance == (float) -2.2245093E37F);
                Debug.Assert(pack.size_y == (float) -2.4359793E38F);
                Debug.Assert(pack.type == LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_VISION_OTHER);
                Debug.Assert(pack.frame == MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT);
                Debug.Assert(pack.q_TRY(ph).SequenceEqual(new float[] {-1.6968625E38F, -2.9367599E38F, 2.6514851E38F, 2.5399288E38F}));
                Debug.Assert(pack.time_usec == (ulong)4349498623449262832L);
                Debug.Assert(pack.angle_y == (float) -2.7588314E38F);
                Debug.Assert(pack.z_TRY(ph) == (float)1.7215042E38F);
                Debug.Assert(pack.position_valid_TRY(ph) == (byte)(byte)102);
                Debug.Assert(pack.size_x == (float)9.199755E37F);
            };
            GroundControl.LANDING_TARGET p149 = CommunicationChannel.new_LANDING_TARGET();
            PH.setPack(p149);
            p149.angle_x = (float) -6.765539E35F;
            p149.distance = (float) -2.2245093E37F;
            p149.z_SET((float)1.7215042E38F, PH) ;
            p149.size_y = (float) -2.4359793E38F;
            p149.type = LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_VISION_OTHER;
            p149.y_SET((float) -3.2016337E38F, PH) ;
            p149.size_x = (float)9.199755E37F;
            p149.position_valid_SET((byte)(byte)102, PH) ;
            p149.time_usec = (ulong)4349498623449262832L;
            p149.x_SET((float) -5.2715666E37F, PH) ;
            p149.target_num = (byte)(byte)105;
            p149.angle_y = (float) -2.7588314E38F;
            p149.frame = MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT;
            p149.q_SET(new float[] {-1.6968625E38F, -2.9367599E38F, 2.6514851E38F, 2.5399288E38F}, 0, PH) ;
            CommunicationChannel.instance.send(p149);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            ADV_TEST_CH.OnSENSOR_OFFSETSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.gyro_cal_x == (float) -1.7929638E38F);
                Debug.Assert(pack.mag_declination == (float) -1.0657306E38F);
                Debug.Assert(pack.mag_ofs_x == (short)(short)26862);
                Debug.Assert(pack.accel_cal_z == (float) -3.103214E38F);
                Debug.Assert(pack.raw_press == (int)445205050);
                Debug.Assert(pack.mag_ofs_z == (short)(short)13291);
                Debug.Assert(pack.raw_temp == (int)761796188);
                Debug.Assert(pack.gyro_cal_z == (float)3.2230302E38F);
                Debug.Assert(pack.gyro_cal_y == (float)1.5688998E37F);
                Debug.Assert(pack.accel_cal_y == (float)9.665618E37F);
                Debug.Assert(pack.accel_cal_x == (float)2.986002E38F);
                Debug.Assert(pack.mag_ofs_y == (short)(short)26756);
            };
            GroundControl.SENSOR_OFFSETS p150 = CommunicationChannel.new_SENSOR_OFFSETS();
            PH.setPack(p150);
            p150.accel_cal_z = (float) -3.103214E38F;
            p150.accel_cal_x = (float)2.986002E38F;
            p150.gyro_cal_x = (float) -1.7929638E38F;
            p150.gyro_cal_y = (float)1.5688998E37F;
            p150.mag_declination = (float) -1.0657306E38F;
            p150.mag_ofs_y = (short)(short)26756;
            p150.mag_ofs_x = (short)(short)26862;
            p150.raw_press = (int)445205050;
            p150.mag_ofs_z = (short)(short)13291;
            p150.accel_cal_y = (float)9.665618E37F;
            p150.raw_temp = (int)761796188;
            p150.gyro_cal_z = (float)3.2230302E38F;
            CommunicationChannel.instance.send(p150);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnSET_MAG_OFFSETSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)89);
                Debug.Assert(pack.mag_ofs_y == (short)(short) -9187);
                Debug.Assert(pack.target_component == (byte)(byte)66);
                Debug.Assert(pack.mag_ofs_x == (short)(short) -25339);
                Debug.Assert(pack.mag_ofs_z == (short)(short)16083);
            };
            GroundControl.SET_MAG_OFFSETS p151 = CommunicationChannel.new_SET_MAG_OFFSETS();
            PH.setPack(p151);
            p151.mag_ofs_y = (short)(short) -9187;
            p151.mag_ofs_x = (short)(short) -25339;
            p151.target_component = (byte)(byte)66;
            p151.mag_ofs_z = (short)(short)16083;
            p151.target_system = (byte)(byte)89;
            CommunicationChannel.instance.send(p151);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnMEMINFOReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.freemem32_TRY(ph) == (uint)221778852U);
                Debug.Assert(pack.freemem == (ushort)(ushort)11563);
                Debug.Assert(pack.brkval == (ushort)(ushort)55347);
            };
            GroundControl.MEMINFO p152 = CommunicationChannel.new_MEMINFO();
            PH.setPack(p152);
            p152.brkval = (ushort)(ushort)55347;
            p152.freemem = (ushort)(ushort)11563;
            p152.freemem32_SET((uint)221778852U, PH) ;
            CommunicationChannel.instance.send(p152);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnAP_ADCReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.adc6 == (ushort)(ushort)50290);
                Debug.Assert(pack.adc1 == (ushort)(ushort)8643);
                Debug.Assert(pack.adc4 == (ushort)(ushort)24984);
                Debug.Assert(pack.adc2 == (ushort)(ushort)22149);
                Debug.Assert(pack.adc3 == (ushort)(ushort)6777);
                Debug.Assert(pack.adc5 == (ushort)(ushort)15360);
            };
            GroundControl.AP_ADC p153 = CommunicationChannel.new_AP_ADC();
            PH.setPack(p153);
            p153.adc4 = (ushort)(ushort)24984;
            p153.adc3 = (ushort)(ushort)6777;
            p153.adc5 = (ushort)(ushort)15360;
            p153.adc6 = (ushort)(ushort)50290;
            p153.adc1 = (ushort)(ushort)8643;
            p153.adc2 = (ushort)(ushort)22149;
            CommunicationChannel.instance.send(p153);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnDIGICAM_CONFIGUREReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)68);
                Debug.Assert(pack.target_system == (byte)(byte)64);
                Debug.Assert(pack.extra_param == (byte)(byte)133);
                Debug.Assert(pack.mode == (byte)(byte)146);
                Debug.Assert(pack.exposure_type == (byte)(byte)213);
                Debug.Assert(pack.iso == (byte)(byte)132);
                Debug.Assert(pack.engine_cut_off == (byte)(byte)236);
                Debug.Assert(pack.aperture == (byte)(byte)168);
                Debug.Assert(pack.extra_value == (float)2.773975E38F);
                Debug.Assert(pack.command_id == (byte)(byte)80);
                Debug.Assert(pack.shutter_speed == (ushort)(ushort)57208);
            };
            GroundControl.DIGICAM_CONFIGURE p154 = CommunicationChannel.new_DIGICAM_CONFIGURE();
            PH.setPack(p154);
            p154.mode = (byte)(byte)146;
            p154.shutter_speed = (ushort)(ushort)57208;
            p154.aperture = (byte)(byte)168;
            p154.iso = (byte)(byte)132;
            p154.engine_cut_off = (byte)(byte)236;
            p154.extra_value = (float)2.773975E38F;
            p154.exposure_type = (byte)(byte)213;
            p154.extra_param = (byte)(byte)133;
            p154.command_id = (byte)(byte)80;
            p154.target_component = (byte)(byte)68;
            p154.target_system = (byte)(byte)64;
            CommunicationChannel.instance.send(p154);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnDIGICAM_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.command_id == (byte)(byte)251);
                Debug.Assert(pack.focus_lock == (byte)(byte)251);
                Debug.Assert(pack.target_component == (byte)(byte)230);
                Debug.Assert(pack.extra_param == (byte)(byte)165);
                Debug.Assert(pack.target_system == (byte)(byte)168);
                Debug.Assert(pack.session == (byte)(byte)194);
                Debug.Assert(pack.shot == (byte)(byte)61);
                Debug.Assert(pack.extra_value == (float) -9.260794E37F);
                Debug.Assert(pack.zoom_step == (sbyte)(sbyte) - 99);
                Debug.Assert(pack.zoom_pos == (byte)(byte)241);
            };
            GroundControl.DIGICAM_CONTROL p155 = CommunicationChannel.new_DIGICAM_CONTROL();
            PH.setPack(p155);
            p155.shot = (byte)(byte)61;
            p155.zoom_pos = (byte)(byte)241;
            p155.target_component = (byte)(byte)230;
            p155.focus_lock = (byte)(byte)251;
            p155.extra_param = (byte)(byte)165;
            p155.zoom_step = (sbyte)(sbyte) - 99;
            p155.command_id = (byte)(byte)251;
            p155.extra_value = (float) -9.260794E37F;
            p155.target_system = (byte)(byte)168;
            p155.session = (byte)(byte)194;
            CommunicationChannel.instance.send(p155);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnMOUNT_CONFIGUREReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)95);
                Debug.Assert(pack.target_system == (byte)(byte)19);
                Debug.Assert(pack.stab_yaw == (byte)(byte)103);
                Debug.Assert(pack.stab_roll == (byte)(byte)158);
                Debug.Assert(pack.mount_mode == MAV_MOUNT_MODE.MAV_MOUNT_MODE_RETRACT);
                Debug.Assert(pack.stab_pitch == (byte)(byte)113);
            };
            GroundControl.MOUNT_CONFIGURE p156 = CommunicationChannel.new_MOUNT_CONFIGURE();
            PH.setPack(p156);
            p156.stab_yaw = (byte)(byte)103;
            p156.target_component = (byte)(byte)95;
            p156.mount_mode = MAV_MOUNT_MODE.MAV_MOUNT_MODE_RETRACT;
            p156.stab_roll = (byte)(byte)158;
            p156.stab_pitch = (byte)(byte)113;
            p156.target_system = (byte)(byte)19;
            CommunicationChannel.instance.send(p156);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnMOUNT_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)61);
                Debug.Assert(pack.target_component == (byte)(byte)207);
                Debug.Assert(pack.input_a == (int)1517306946);
                Debug.Assert(pack.save_position == (byte)(byte)23);
                Debug.Assert(pack.input_b == (int)754989382);
                Debug.Assert(pack.input_c == (int)287965892);
            };
            GroundControl.MOUNT_CONTROL p157 = CommunicationChannel.new_MOUNT_CONTROL();
            PH.setPack(p157);
            p157.input_a = (int)1517306946;
            p157.save_position = (byte)(byte)23;
            p157.input_c = (int)287965892;
            p157.target_component = (byte)(byte)207;
            p157.input_b = (int)754989382;
            p157.target_system = (byte)(byte)61;
            CommunicationChannel.instance.send(p157);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnMOUNT_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)246);
                Debug.Assert(pack.pointing_a == (int) -753492246);
                Debug.Assert(pack.pointing_b == (int)1236593124);
                Debug.Assert(pack.target_component == (byte)(byte)171);
                Debug.Assert(pack.pointing_c == (int)33622387);
            };
            GroundControl.MOUNT_STATUS p158 = CommunicationChannel.new_MOUNT_STATUS();
            PH.setPack(p158);
            p158.target_component = (byte)(byte)171;
            p158.pointing_b = (int)1236593124;
            p158.pointing_a = (int) -753492246;
            p158.target_system = (byte)(byte)246;
            p158.pointing_c = (int)33622387;
            CommunicationChannel.instance.send(p158);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnFENCE_POINTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.idx == (byte)(byte)200);
                Debug.Assert(pack.lng == (float)8.914866E37F);
                Debug.Assert(pack.lat == (float) -6.31089E36F);
                Debug.Assert(pack.target_system == (byte)(byte)166);
                Debug.Assert(pack.count == (byte)(byte)193);
                Debug.Assert(pack.target_component == (byte)(byte)49);
            };
            GroundControl.FENCE_POINT p160 = CommunicationChannel.new_FENCE_POINT();
            PH.setPack(p160);
            p160.target_system = (byte)(byte)166;
            p160.target_component = (byte)(byte)49;
            p160.lng = (float)8.914866E37F;
            p160.idx = (byte)(byte)200;
            p160.count = (byte)(byte)193;
            p160.lat = (float) -6.31089E36F;
            CommunicationChannel.instance.send(p160);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnFENCE_FETCH_POINTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.idx == (byte)(byte)102);
                Debug.Assert(pack.target_component == (byte)(byte)71);
                Debug.Assert(pack.target_system == (byte)(byte)47);
            };
            GroundControl.FENCE_FETCH_POINT p161 = CommunicationChannel.new_FENCE_FETCH_POINT();
            PH.setPack(p161);
            p161.target_system = (byte)(byte)47;
            p161.idx = (byte)(byte)102;
            p161.target_component = (byte)(byte)71;
            CommunicationChannel.instance.send(p161);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnFENCE_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.breach_status == (byte)(byte)8);
                Debug.Assert(pack.breach_type == FENCE_BREACH.FENCE_BREACH_BOUNDARY);
                Debug.Assert(pack.breach_count == (ushort)(ushort)15994);
                Debug.Assert(pack.breach_time == (uint)950293936U);
            };
            GroundControl.FENCE_STATUS p162 = CommunicationChannel.new_FENCE_STATUS();
            PH.setPack(p162);
            p162.breach_count = (ushort)(ushort)15994;
            p162.breach_time = (uint)950293936U;
            p162.breach_status = (byte)(byte)8;
            p162.breach_type = FENCE_BREACH.FENCE_BREACH_BOUNDARY;
            CommunicationChannel.instance.send(p162);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnAHRSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.omegaIy == (float) -1.5763003E38F);
                Debug.Assert(pack.error_yaw == (float)7.8731375E37F);
                Debug.Assert(pack.error_rp == (float) -6.4597695E37F);
                Debug.Assert(pack.accel_weight == (float)3.183541E37F);
                Debug.Assert(pack.renorm_val == (float)1.7896281E38F);
                Debug.Assert(pack.omegaIz == (float)1.7827958E38F);
                Debug.Assert(pack.omegaIx == (float) -2.106959E38F);
            };
            GroundControl.AHRS p163 = CommunicationChannel.new_AHRS();
            PH.setPack(p163);
            p163.accel_weight = (float)3.183541E37F;
            p163.error_yaw = (float)7.8731375E37F;
            p163.omegaIx = (float) -2.106959E38F;
            p163.error_rp = (float) -6.4597695E37F;
            p163.renorm_val = (float)1.7896281E38F;
            p163.omegaIz = (float)1.7827958E38F;
            p163.omegaIy = (float) -1.5763003E38F;
            CommunicationChannel.instance.send(p163);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnSIMSTATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.xacc == (float)1.015566E38F);
                Debug.Assert(pack.lng == (int) -1227321592);
                Debug.Assert(pack.xgyro == (float) -1.6978324E38F);
                Debug.Assert(pack.yaw == (float) -3.184909E38F);
                Debug.Assert(pack.zgyro == (float) -2.4075119E38F);
                Debug.Assert(pack.pitch == (float)3.7854204E37F);
                Debug.Assert(pack.roll == (float) -3.3790228E37F);
                Debug.Assert(pack.zacc == (float)1.3877079E38F);
                Debug.Assert(pack.yacc == (float)2.5211181E38F);
                Debug.Assert(pack.lat == (int)1256535465);
                Debug.Assert(pack.ygyro == (float)6.593369E37F);
            };
            GroundControl.SIMSTATE p164 = CommunicationChannel.new_SIMSTATE();
            PH.setPack(p164);
            p164.zacc = (float)1.3877079E38F;
            p164.xgyro = (float) -1.6978324E38F;
            p164.zgyro = (float) -2.4075119E38F;
            p164.xacc = (float)1.015566E38F;
            p164.lat = (int)1256535465;
            p164.pitch = (float)3.7854204E37F;
            p164.yacc = (float)2.5211181E38F;
            p164.ygyro = (float)6.593369E37F;
            p164.lng = (int) -1227321592;
            p164.roll = (float) -3.3790228E37F;
            p164.yaw = (float) -3.184909E38F;
            CommunicationChannel.instance.send(p164);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnHWSTATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.I2Cerr == (byte)(byte)251);
                Debug.Assert(pack.Vcc == (ushort)(ushort)38109);
            };
            GroundControl.HWSTATUS p165 = CommunicationChannel.new_HWSTATUS();
            PH.setPack(p165);
            p165.Vcc = (ushort)(ushort)38109;
            p165.I2Cerr = (byte)(byte)251;
            CommunicationChannel.instance.send(p165);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnRADIOReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.rssi == (byte)(byte)14);
                Debug.Assert(pack.remrssi == (byte)(byte)24);
                Debug.Assert(pack.rxerrors == (ushort)(ushort)33273);
                Debug.Assert(pack.fixed_ == (ushort)(ushort)55457);
                Debug.Assert(pack.noise == (byte)(byte)32);
                Debug.Assert(pack.remnoise == (byte)(byte)189);
                Debug.Assert(pack.txbuf == (byte)(byte)213);
            };
            GroundControl.RADIO p166 = CommunicationChannel.new_RADIO();
            PH.setPack(p166);
            p166.rxerrors = (ushort)(ushort)33273;
            p166.rssi = (byte)(byte)14;
            p166.txbuf = (byte)(byte)213;
            p166.remnoise = (byte)(byte)189;
            p166.fixed_ = (ushort)(ushort)55457;
            p166.remrssi = (byte)(byte)24;
            p166.noise = (byte)(byte)32;
            CommunicationChannel.instance.send(p166);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnLIMITS_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.breach_count == (ushort)(ushort)62655);
                Debug.Assert(pack.last_trigger == (uint)3057540901U);
                Debug.Assert(pack.last_clear == (uint)1476596194U);
                Debug.Assert(pack.mods_required == (LIMIT_MODULE.LIMIT_GPSLOCK));
                Debug.Assert(pack.last_recovery == (uint)3585253751U);
                Debug.Assert(pack.limits_state == LIMITS_STATE.LIMITS_TRIGGERED);
                Debug.Assert(pack.mods_enabled == (LIMIT_MODULE.LIMIT_GPSLOCK));
                Debug.Assert(pack.mods_triggered == (LIMIT_MODULE.LIMIT_GPSLOCK));
                Debug.Assert(pack.last_action == (uint)1100274906U);
            };
            GroundControl.LIMITS_STATUS p167 = CommunicationChannel.new_LIMITS_STATUS();
            PH.setPack(p167);
            p167.limits_state = LIMITS_STATE.LIMITS_TRIGGERED;
            p167.mods_required = (LIMIT_MODULE.LIMIT_GPSLOCK);
            p167.mods_enabled = (LIMIT_MODULE.LIMIT_GPSLOCK);
            p167.last_action = (uint)1100274906U;
            p167.breach_count = (ushort)(ushort)62655;
            p167.last_clear = (uint)1476596194U;
            p167.mods_triggered = (LIMIT_MODULE.LIMIT_GPSLOCK);
            p167.last_trigger = (uint)3057540901U;
            p167.last_recovery = (uint)3585253751U;
            CommunicationChannel.instance.send(p167);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnWINDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.speed_z == (float)2.4385947E38F);
                Debug.Assert(pack.speed == (float) -8.854161E37F);
                Debug.Assert(pack.direction == (float) -1.4275013E38F);
            };
            GroundControl.WIND p168 = CommunicationChannel.new_WIND();
            PH.setPack(p168);
            p168.speed = (float) -8.854161E37F;
            p168.direction = (float) -1.4275013E38F;
            p168.speed_z = (float)2.4385947E38F;
            CommunicationChannel.instance.send(p168);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnDATA16Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)143, (byte)20, (byte)107, (byte)98, (byte)159, (byte)254, (byte)210, (byte)137, (byte)79, (byte)114, (byte)11, (byte)199, (byte)166, (byte)77, (byte)252, (byte)139}));
                Debug.Assert(pack.len == (byte)(byte)19);
                Debug.Assert(pack.type == (byte)(byte)222);
            };
            GroundControl.DATA16 p169 = CommunicationChannel.new_DATA16();
            PH.setPack(p169);
            p169.type = (byte)(byte)222;
            p169.data__SET(new byte[] {(byte)143, (byte)20, (byte)107, (byte)98, (byte)159, (byte)254, (byte)210, (byte)137, (byte)79, (byte)114, (byte)11, (byte)199, (byte)166, (byte)77, (byte)252, (byte)139}, 0) ;
            p169.len = (byte)(byte)19;
            CommunicationChannel.instance.send(p169);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnDATA32Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.type == (byte)(byte)54);
                Debug.Assert(pack.len == (byte)(byte)135);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)56, (byte)145, (byte)102, (byte)188, (byte)72, (byte)92, (byte)3, (byte)94, (byte)136, (byte)191, (byte)45, (byte)130, (byte)242, (byte)72, (byte)17, (byte)69, (byte)161, (byte)81, (byte)129, (byte)180, (byte)205, (byte)238, (byte)141, (byte)40, (byte)42, (byte)50, (byte)192, (byte)22, (byte)13, (byte)7, (byte)51, (byte)126}));
            };
            GroundControl.DATA32 p170 = CommunicationChannel.new_DATA32();
            PH.setPack(p170);
            p170.type = (byte)(byte)54;
            p170.data__SET(new byte[] {(byte)56, (byte)145, (byte)102, (byte)188, (byte)72, (byte)92, (byte)3, (byte)94, (byte)136, (byte)191, (byte)45, (byte)130, (byte)242, (byte)72, (byte)17, (byte)69, (byte)161, (byte)81, (byte)129, (byte)180, (byte)205, (byte)238, (byte)141, (byte)40, (byte)42, (byte)50, (byte)192, (byte)22, (byte)13, (byte)7, (byte)51, (byte)126}, 0) ;
            p170.len = (byte)(byte)135;
            CommunicationChannel.instance.send(p170);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnDATA64Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.type == (byte)(byte)252);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)46, (byte)255, (byte)112, (byte)2, (byte)177, (byte)167, (byte)154, (byte)215, (byte)238, (byte)210, (byte)150, (byte)55, (byte)120, (byte)28, (byte)11, (byte)227, (byte)129, (byte)101, (byte)131, (byte)238, (byte)218, (byte)195, (byte)90, (byte)79, (byte)56, (byte)149, (byte)49, (byte)159, (byte)209, (byte)21, (byte)170, (byte)183, (byte)251, (byte)56, (byte)16, (byte)8, (byte)16, (byte)187, (byte)243, (byte)196, (byte)72, (byte)196, (byte)205, (byte)224, (byte)183, (byte)76, (byte)97, (byte)72, (byte)5, (byte)167, (byte)240, (byte)32, (byte)244, (byte)136, (byte)186, (byte)123, (byte)15, (byte)225, (byte)147, (byte)230, (byte)122, (byte)139, (byte)74, (byte)191}));
                Debug.Assert(pack.len == (byte)(byte)111);
            };
            GroundControl.DATA64 p171 = CommunicationChannel.new_DATA64();
            PH.setPack(p171);
            p171.data__SET(new byte[] {(byte)46, (byte)255, (byte)112, (byte)2, (byte)177, (byte)167, (byte)154, (byte)215, (byte)238, (byte)210, (byte)150, (byte)55, (byte)120, (byte)28, (byte)11, (byte)227, (byte)129, (byte)101, (byte)131, (byte)238, (byte)218, (byte)195, (byte)90, (byte)79, (byte)56, (byte)149, (byte)49, (byte)159, (byte)209, (byte)21, (byte)170, (byte)183, (byte)251, (byte)56, (byte)16, (byte)8, (byte)16, (byte)187, (byte)243, (byte)196, (byte)72, (byte)196, (byte)205, (byte)224, (byte)183, (byte)76, (byte)97, (byte)72, (byte)5, (byte)167, (byte)240, (byte)32, (byte)244, (byte)136, (byte)186, (byte)123, (byte)15, (byte)225, (byte)147, (byte)230, (byte)122, (byte)139, (byte)74, (byte)191}, 0) ;
            p171.len = (byte)(byte)111;
            p171.type = (byte)(byte)252;
            CommunicationChannel.instance.send(p171);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnDATA96Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)103, (byte)4, (byte)211, (byte)117, (byte)117, (byte)152, (byte)83, (byte)57, (byte)239, (byte)49, (byte)20, (byte)107, (byte)229, (byte)185, (byte)113, (byte)251, (byte)197, (byte)61, (byte)169, (byte)41, (byte)58, (byte)0, (byte)47, (byte)38, (byte)83, (byte)125, (byte)148, (byte)212, (byte)139, (byte)198, (byte)187, (byte)51, (byte)28, (byte)120, (byte)227, (byte)91, (byte)211, (byte)152, (byte)99, (byte)34, (byte)76, (byte)19, (byte)7, (byte)139, (byte)37, (byte)251, (byte)70, (byte)207, (byte)45, (byte)93, (byte)29, (byte)53, (byte)103, (byte)108, (byte)103, (byte)196, (byte)96, (byte)17, (byte)82, (byte)132, (byte)128, (byte)218, (byte)237, (byte)21, (byte)119, (byte)120, (byte)230, (byte)30, (byte)125, (byte)96, (byte)11, (byte)212, (byte)51, (byte)175, (byte)122, (byte)189, (byte)86, (byte)226, (byte)40, (byte)62, (byte)249, (byte)148, (byte)118, (byte)10, (byte)49, (byte)72, (byte)94, (byte)60, (byte)99, (byte)228, (byte)180, (byte)187, (byte)17, (byte)1, (byte)123, (byte)98}));
                Debug.Assert(pack.len == (byte)(byte)132);
                Debug.Assert(pack.type == (byte)(byte)40);
            };
            GroundControl.DATA96 p172 = CommunicationChannel.new_DATA96();
            PH.setPack(p172);
            p172.len = (byte)(byte)132;
            p172.data__SET(new byte[] {(byte)103, (byte)4, (byte)211, (byte)117, (byte)117, (byte)152, (byte)83, (byte)57, (byte)239, (byte)49, (byte)20, (byte)107, (byte)229, (byte)185, (byte)113, (byte)251, (byte)197, (byte)61, (byte)169, (byte)41, (byte)58, (byte)0, (byte)47, (byte)38, (byte)83, (byte)125, (byte)148, (byte)212, (byte)139, (byte)198, (byte)187, (byte)51, (byte)28, (byte)120, (byte)227, (byte)91, (byte)211, (byte)152, (byte)99, (byte)34, (byte)76, (byte)19, (byte)7, (byte)139, (byte)37, (byte)251, (byte)70, (byte)207, (byte)45, (byte)93, (byte)29, (byte)53, (byte)103, (byte)108, (byte)103, (byte)196, (byte)96, (byte)17, (byte)82, (byte)132, (byte)128, (byte)218, (byte)237, (byte)21, (byte)119, (byte)120, (byte)230, (byte)30, (byte)125, (byte)96, (byte)11, (byte)212, (byte)51, (byte)175, (byte)122, (byte)189, (byte)86, (byte)226, (byte)40, (byte)62, (byte)249, (byte)148, (byte)118, (byte)10, (byte)49, (byte)72, (byte)94, (byte)60, (byte)99, (byte)228, (byte)180, (byte)187, (byte)17, (byte)1, (byte)123, (byte)98}, 0) ;
            p172.type = (byte)(byte)40;
            CommunicationChannel.instance.send(p172);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnRANGEFINDERReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.voltage == (float)2.0167621E38F);
                Debug.Assert(pack.distance == (float)2.9615764E38F);
            };
            GroundControl.RANGEFINDER p173 = CommunicationChannel.new_RANGEFINDER();
            PH.setPack(p173);
            p173.distance = (float)2.9615764E38F;
            p173.voltage = (float)2.0167621E38F;
            CommunicationChannel.instance.send(p173);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnAIRSPEED_AUTOCALReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.state_y == (float) -1.2308783E38F);
                Debug.Assert(pack.diff_pressure == (float)4.784667E37F);
                Debug.Assert(pack.Pby == (float)1.8215427E38F);
                Debug.Assert(pack.vy == (float)1.5419969E38F);
                Debug.Assert(pack.state_z == (float)2.7432344E38F);
                Debug.Assert(pack.Pcz == (float) -2.424482E38F);
                Debug.Assert(pack.state_x == (float)2.090924E38F);
                Debug.Assert(pack.Pax == (float) -2.8573859E38F);
                Debug.Assert(pack.vx == (float)8.4747026E37F);
                Debug.Assert(pack.EAS2TAS == (float) -3.1806682E38F);
                Debug.Assert(pack.vz == (float) -2.9932326E38F);
                Debug.Assert(pack.ratio == (float)2.662629E38F);
            };
            GroundControl.AIRSPEED_AUTOCAL p174 = CommunicationChannel.new_AIRSPEED_AUTOCAL();
            PH.setPack(p174);
            p174.Pax = (float) -2.8573859E38F;
            p174.Pby = (float)1.8215427E38F;
            p174.vy = (float)1.5419969E38F;
            p174.vz = (float) -2.9932326E38F;
            p174.diff_pressure = (float)4.784667E37F;
            p174.state_x = (float)2.090924E38F;
            p174.state_z = (float)2.7432344E38F;
            p174.ratio = (float)2.662629E38F;
            p174.state_y = (float) -1.2308783E38F;
            p174.vx = (float)8.4747026E37F;
            p174.Pcz = (float) -2.424482E38F;
            p174.EAS2TAS = (float) -3.1806682E38F;
            CommunicationChannel.instance.send(p174);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnRALLY_POINTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.alt == (short)(short) -3951);
                Debug.Assert(pack.count == (byte)(byte)70);
                Debug.Assert(pack.target_component == (byte)(byte)238);
                Debug.Assert(pack.land_dir == (ushort)(ushort)42718);
                Debug.Assert(pack.flags == RALLY_FLAGS.LAND_IMMEDIATELY);
                Debug.Assert(pack.break_alt == (short)(short)20005);
                Debug.Assert(pack.lng == (int)164202527);
                Debug.Assert(pack.lat == (int) -1747812421);
                Debug.Assert(pack.idx == (byte)(byte)60);
                Debug.Assert(pack.target_system == (byte)(byte)113);
            };
            GroundControl.RALLY_POINT p175 = CommunicationChannel.new_RALLY_POINT();
            PH.setPack(p175);
            p175.target_component = (byte)(byte)238;
            p175.target_system = (byte)(byte)113;
            p175.idx = (byte)(byte)60;
            p175.alt = (short)(short) -3951;
            p175.land_dir = (ushort)(ushort)42718;
            p175.break_alt = (short)(short)20005;
            p175.flags = RALLY_FLAGS.LAND_IMMEDIATELY;
            p175.lat = (int) -1747812421;
            p175.lng = (int)164202527;
            p175.count = (byte)(byte)70;
            CommunicationChannel.instance.send(p175);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnRALLY_FETCH_POINTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)53);
                Debug.Assert(pack.idx == (byte)(byte)233);
                Debug.Assert(pack.target_component == (byte)(byte)177);
            };
            GroundControl.RALLY_FETCH_POINT p176 = CommunicationChannel.new_RALLY_FETCH_POINT();
            PH.setPack(p176);
            p176.target_system = (byte)(byte)53;
            p176.idx = (byte)(byte)233;
            p176.target_component = (byte)(byte)177;
            CommunicationChannel.instance.send(p176);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnCOMPASSMOT_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.current == (float)2.575143E38F);
                Debug.Assert(pack.throttle == (ushort)(ushort)6456);
                Debug.Assert(pack.CompensationY == (float) -3.1706607E38F);
                Debug.Assert(pack.CompensationZ == (float) -2.21425E38F);
                Debug.Assert(pack.CompensationX == (float) -2.105371E38F);
                Debug.Assert(pack.interference == (ushort)(ushort)37892);
            };
            GroundControl.COMPASSMOT_STATUS p177 = CommunicationChannel.new_COMPASSMOT_STATUS();
            PH.setPack(p177);
            p177.throttle = (ushort)(ushort)6456;
            p177.CompensationZ = (float) -2.21425E38F;
            p177.interference = (ushort)(ushort)37892;
            p177.CompensationY = (float) -3.1706607E38F;
            p177.current = (float)2.575143E38F;
            p177.CompensationX = (float) -2.105371E38F;
            CommunicationChannel.instance.send(p177);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnAHRS2Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.altitude == (float)1.1727122E38F);
                Debug.Assert(pack.lng == (int) -1059959201);
                Debug.Assert(pack.pitch == (float) -2.2236563E38F);
                Debug.Assert(pack.yaw == (float)3.3544603E38F);
                Debug.Assert(pack.roll == (float)6.6003656E37F);
                Debug.Assert(pack.lat == (int) -1607375468);
            };
            GroundControl.AHRS2 p178 = CommunicationChannel.new_AHRS2();
            PH.setPack(p178);
            p178.yaw = (float)3.3544603E38F;
            p178.altitude = (float)1.1727122E38F;
            p178.pitch = (float) -2.2236563E38F;
            p178.roll = (float)6.6003656E37F;
            p178.lat = (int) -1607375468;
            p178.lng = (int) -1059959201;
            CommunicationChannel.instance.send(p178);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnCAMERA_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.cam_idx == (byte)(byte)45);
                Debug.Assert(pack.img_idx == (ushort)(ushort)25606);
                Debug.Assert(pack.p1 == (float)2.9552515E37F);
                Debug.Assert(pack.time_usec == (ulong)3448071664187463885L);
                Debug.Assert(pack.p3 == (float) -2.1357639E38F);
                Debug.Assert(pack.event_id == CAMERA_STATUS_TYPES.CAMERA_STATUS_TYPE_TRIGGER);
                Debug.Assert(pack.p4 == (float)3.559578E37F);
                Debug.Assert(pack.p2 == (float)1.6324105E38F);
                Debug.Assert(pack.target_system == (byte)(byte)246);
            };
            GroundControl.CAMERA_STATUS p179 = CommunicationChannel.new_CAMERA_STATUS();
            PH.setPack(p179);
            p179.p1 = (float)2.9552515E37F;
            p179.p3 = (float) -2.1357639E38F;
            p179.p2 = (float)1.6324105E38F;
            p179.target_system = (byte)(byte)246;
            p179.time_usec = (ulong)3448071664187463885L;
            p179.event_id = CAMERA_STATUS_TYPES.CAMERA_STATUS_TYPE_TRIGGER;
            p179.cam_idx = (byte)(byte)45;
            p179.img_idx = (ushort)(ushort)25606;
            p179.p4 = (float)3.559578E37F;
            CommunicationChannel.instance.send(p179);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnCAMERA_FEEDBACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)83);
                Debug.Assert(pack.img_idx == (ushort)(ushort)16758);
                Debug.Assert(pack.alt_rel == (float)3.2302246E38F);
                Debug.Assert(pack.yaw == (float) -2.1371078E38F);
                Debug.Assert(pack.lng == (int) -1432633150);
                Debug.Assert(pack.alt_msl == (float) -3.1624906E37F);
                Debug.Assert(pack.cam_idx == (byte)(byte)113);
                Debug.Assert(pack.flags == CAMERA_FEEDBACK_FLAGS.CAMERA_FEEDBACK_CLOSEDLOOP);
                Debug.Assert(pack.lat == (int) -235376201);
                Debug.Assert(pack.foc_len == (float) -2.3682464E38F);
                Debug.Assert(pack.roll == (float)2.278683E38F);
                Debug.Assert(pack.time_usec == (ulong)5519900788081468331L);
                Debug.Assert(pack.pitch == (float)2.2371767E37F);
            };
            GroundControl.CAMERA_FEEDBACK p180 = CommunicationChannel.new_CAMERA_FEEDBACK();
            PH.setPack(p180);
            p180.time_usec = (ulong)5519900788081468331L;
            p180.pitch = (float)2.2371767E37F;
            p180.flags = CAMERA_FEEDBACK_FLAGS.CAMERA_FEEDBACK_CLOSEDLOOP;
            p180.cam_idx = (byte)(byte)113;
            p180.alt_msl = (float) -3.1624906E37F;
            p180.lng = (int) -1432633150;
            p180.roll = (float)2.278683E38F;
            p180.lat = (int) -235376201;
            p180.foc_len = (float) -2.3682464E38F;
            p180.alt_rel = (float)3.2302246E38F;
            p180.target_system = (byte)(byte)83;
            p180.img_idx = (ushort)(ushort)16758;
            p180.yaw = (float) -2.1371078E38F;
            CommunicationChannel.instance.send(p180);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnBATTERY2Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.current_battery == (short)(short)22192);
                Debug.Assert(pack.voltage == (ushort)(ushort)9644);
            };
            GroundControl.BATTERY2 p181 = CommunicationChannel.new_BATTERY2();
            PH.setPack(p181);
            p181.voltage = (ushort)(ushort)9644;
            p181.current_battery = (short)(short)22192;
            CommunicationChannel.instance.send(p181);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnAHRS3Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lng == (int) -865435705);
                Debug.Assert(pack.yaw == (float)8.0693354E37F);
                Debug.Assert(pack.v2 == (float)2.3214865E38F);
                Debug.Assert(pack.v1 == (float)1.3071731E38F);
                Debug.Assert(pack.altitude == (float) -3.2893004E38F);
                Debug.Assert(pack.roll == (float) -8.8405015E36F);
                Debug.Assert(pack.pitch == (float) -4.781831E37F);
                Debug.Assert(pack.v3 == (float) -3.9560264E37F);
                Debug.Assert(pack.v4 == (float) -5.847742E36F);
                Debug.Assert(pack.lat == (int)1166132524);
            };
            GroundControl.AHRS3 p182 = CommunicationChannel.new_AHRS3();
            PH.setPack(p182);
            p182.altitude = (float) -3.2893004E38F;
            p182.lat = (int)1166132524;
            p182.v2 = (float)2.3214865E38F;
            p182.lng = (int) -865435705;
            p182.yaw = (float)8.0693354E37F;
            p182.v4 = (float) -5.847742E36F;
            p182.v3 = (float) -3.9560264E37F;
            p182.roll = (float) -8.8405015E36F;
            p182.pitch = (float) -4.781831E37F;
            p182.v1 = (float)1.3071731E38F;
            CommunicationChannel.instance.send(p182);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnAUTOPILOT_VERSION_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)225);
                Debug.Assert(pack.target_system == (byte)(byte)160);
            };
            GroundControl.AUTOPILOT_VERSION_REQUEST p183 = CommunicationChannel.new_AUTOPILOT_VERSION_REQUEST();
            PH.setPack(p183);
            p183.target_system = (byte)(byte)160;
            p183.target_component = (byte)(byte)225;
            CommunicationChannel.instance.send(p183);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnREMOTE_LOG_DATA_BLOCKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)207, (byte)141, (byte)50, (byte)251, (byte)224, (byte)2, (byte)106, (byte)30, (byte)83, (byte)220, (byte)203, (byte)230, (byte)204, (byte)48, (byte)229, (byte)2, (byte)102, (byte)109, (byte)199, (byte)38, (byte)20, (byte)86, (byte)157, (byte)25, (byte)222, (byte)87, (byte)124, (byte)65, (byte)169, (byte)121, (byte)30, (byte)12, (byte)103, (byte)86, (byte)241, (byte)140, (byte)170, (byte)159, (byte)6, (byte)11, (byte)174, (byte)52, (byte)93, (byte)152, (byte)240, (byte)69, (byte)62, (byte)246, (byte)8, (byte)116, (byte)166, (byte)228, (byte)183, (byte)86, (byte)125, (byte)143, (byte)107, (byte)109, (byte)13, (byte)148, (byte)22, (byte)208, (byte)190, (byte)118, (byte)64, (byte)133, (byte)76, (byte)121, (byte)81, (byte)100, (byte)132, (byte)191, (byte)74, (byte)36, (byte)148, (byte)199, (byte)187, (byte)140, (byte)96, (byte)212, (byte)29, (byte)198, (byte)109, (byte)140, (byte)118, (byte)199, (byte)147, (byte)10, (byte)96, (byte)111, (byte)1, (byte)194, (byte)168, (byte)129, (byte)170, (byte)71, (byte)169, (byte)10, (byte)240, (byte)140, (byte)39, (byte)104, (byte)71, (byte)222, (byte)142, (byte)166, (byte)133, (byte)19, (byte)127, (byte)125, (byte)24, (byte)38, (byte)170, (byte)1, (byte)19, (byte)104, (byte)183, (byte)71, (byte)192, (byte)219, (byte)219, (byte)25, (byte)212, (byte)48, (byte)221, (byte)22, (byte)51, (byte)236, (byte)252, (byte)185, (byte)223, (byte)121, (byte)116, (byte)202, (byte)214, (byte)94, (byte)55, (byte)188, (byte)3, (byte)243, (byte)186, (byte)140, (byte)26, (byte)243, (byte)191, (byte)221, (byte)82, (byte)145, (byte)184, (byte)197, (byte)8, (byte)16, (byte)193, (byte)188, (byte)215, (byte)74, (byte)195, (byte)104, (byte)125, (byte)83, (byte)55, (byte)61, (byte)1, (byte)227, (byte)203, (byte)19, (byte)177, (byte)142, (byte)2, (byte)11, (byte)202, (byte)172, (byte)144, (byte)67, (byte)126, (byte)188, (byte)10, (byte)208, (byte)176, (byte)39, (byte)20, (byte)65, (byte)221, (byte)73, (byte)240, (byte)104, (byte)55, (byte)228, (byte)38, (byte)108, (byte)6, (byte)46, (byte)32, (byte)211, (byte)74, (byte)140, (byte)202, (byte)206, (byte)24, (byte)46}));
                Debug.Assert(pack.target_component == (byte)(byte)172);
                Debug.Assert(pack.target_system == (byte)(byte)80);
                Debug.Assert(pack.seqno == MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS.MAV_REMOTE_LOG_DATA_BLOCK_STOP);
            };
            GroundControl.REMOTE_LOG_DATA_BLOCK p184 = CommunicationChannel.new_REMOTE_LOG_DATA_BLOCK();
            PH.setPack(p184);
            p184.data__SET(new byte[] {(byte)207, (byte)141, (byte)50, (byte)251, (byte)224, (byte)2, (byte)106, (byte)30, (byte)83, (byte)220, (byte)203, (byte)230, (byte)204, (byte)48, (byte)229, (byte)2, (byte)102, (byte)109, (byte)199, (byte)38, (byte)20, (byte)86, (byte)157, (byte)25, (byte)222, (byte)87, (byte)124, (byte)65, (byte)169, (byte)121, (byte)30, (byte)12, (byte)103, (byte)86, (byte)241, (byte)140, (byte)170, (byte)159, (byte)6, (byte)11, (byte)174, (byte)52, (byte)93, (byte)152, (byte)240, (byte)69, (byte)62, (byte)246, (byte)8, (byte)116, (byte)166, (byte)228, (byte)183, (byte)86, (byte)125, (byte)143, (byte)107, (byte)109, (byte)13, (byte)148, (byte)22, (byte)208, (byte)190, (byte)118, (byte)64, (byte)133, (byte)76, (byte)121, (byte)81, (byte)100, (byte)132, (byte)191, (byte)74, (byte)36, (byte)148, (byte)199, (byte)187, (byte)140, (byte)96, (byte)212, (byte)29, (byte)198, (byte)109, (byte)140, (byte)118, (byte)199, (byte)147, (byte)10, (byte)96, (byte)111, (byte)1, (byte)194, (byte)168, (byte)129, (byte)170, (byte)71, (byte)169, (byte)10, (byte)240, (byte)140, (byte)39, (byte)104, (byte)71, (byte)222, (byte)142, (byte)166, (byte)133, (byte)19, (byte)127, (byte)125, (byte)24, (byte)38, (byte)170, (byte)1, (byte)19, (byte)104, (byte)183, (byte)71, (byte)192, (byte)219, (byte)219, (byte)25, (byte)212, (byte)48, (byte)221, (byte)22, (byte)51, (byte)236, (byte)252, (byte)185, (byte)223, (byte)121, (byte)116, (byte)202, (byte)214, (byte)94, (byte)55, (byte)188, (byte)3, (byte)243, (byte)186, (byte)140, (byte)26, (byte)243, (byte)191, (byte)221, (byte)82, (byte)145, (byte)184, (byte)197, (byte)8, (byte)16, (byte)193, (byte)188, (byte)215, (byte)74, (byte)195, (byte)104, (byte)125, (byte)83, (byte)55, (byte)61, (byte)1, (byte)227, (byte)203, (byte)19, (byte)177, (byte)142, (byte)2, (byte)11, (byte)202, (byte)172, (byte)144, (byte)67, (byte)126, (byte)188, (byte)10, (byte)208, (byte)176, (byte)39, (byte)20, (byte)65, (byte)221, (byte)73, (byte)240, (byte)104, (byte)55, (byte)228, (byte)38, (byte)108, (byte)6, (byte)46, (byte)32, (byte)211, (byte)74, (byte)140, (byte)202, (byte)206, (byte)24, (byte)46}, 0) ;
            p184.seqno = MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS.MAV_REMOTE_LOG_DATA_BLOCK_STOP;
            p184.target_system = (byte)(byte)80;
            p184.target_component = (byte)(byte)172;
            CommunicationChannel.instance.send(p184);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnREMOTE_LOG_BLOCK_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)61);
                Debug.Assert(pack.seqno == (uint)574884152U);
                Debug.Assert(pack.target_system == (byte)(byte)140);
                Debug.Assert(pack.status == MAV_REMOTE_LOG_DATA_BLOCK_STATUSES.MAV_REMOTE_LOG_DATA_BLOCK_NACK);
            };
            GroundControl.REMOTE_LOG_BLOCK_STATUS p185 = CommunicationChannel.new_REMOTE_LOG_BLOCK_STATUS();
            PH.setPack(p185);
            p185.target_component = (byte)(byte)61;
            p185.seqno = (uint)574884152U;
            p185.target_system = (byte)(byte)140;
            p185.status = MAV_REMOTE_LOG_DATA_BLOCK_STATUSES.MAV_REMOTE_LOG_DATA_BLOCK_NACK;
            CommunicationChannel.instance.send(p185);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnLED_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.custom_bytes.SequenceEqual(new byte[] {(byte)19, (byte)69, (byte)207, (byte)154, (byte)19, (byte)187, (byte)69, (byte)27, (byte)17, (byte)195, (byte)76, (byte)32, (byte)12, (byte)45, (byte)51, (byte)88, (byte)97, (byte)224, (byte)31, (byte)62, (byte)70, (byte)144, (byte)144, (byte)11}));
                Debug.Assert(pack.target_system == (byte)(byte)56);
                Debug.Assert(pack.instance == (byte)(byte)241);
                Debug.Assert(pack.pattern == (byte)(byte)61);
                Debug.Assert(pack.target_component == (byte)(byte)61);
                Debug.Assert(pack.custom_len == (byte)(byte)149);
            };
            GroundControl.LED_CONTROL p186 = CommunicationChannel.new_LED_CONTROL();
            PH.setPack(p186);
            p186.target_component = (byte)(byte)61;
            p186.instance = (byte)(byte)241;
            p186.custom_len = (byte)(byte)149;
            p186.target_system = (byte)(byte)56;
            p186.custom_bytes_SET(new byte[] {(byte)19, (byte)69, (byte)207, (byte)154, (byte)19, (byte)187, (byte)69, (byte)27, (byte)17, (byte)195, (byte)76, (byte)32, (byte)12, (byte)45, (byte)51, (byte)88, (byte)97, (byte)224, (byte)31, (byte)62, (byte)70, (byte)144, (byte)144, (byte)11}, 0) ;
            p186.pattern = (byte)(byte)61;
            CommunicationChannel.instance.send(p186);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnMAG_CAL_PROGRESSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.attempt == (byte)(byte)50);
                Debug.Assert(pack.direction_y == (float)7.610753E36F);
                Debug.Assert(pack.cal_mask == (byte)(byte)172);
                Debug.Assert(pack.cal_status == MAG_CAL_STATUS.MAG_CAL_NOT_STARTED);
                Debug.Assert(pack.direction_z == (float) -2.5340792E38F);
                Debug.Assert(pack.direction_x == (float) -7.97578E35F);
                Debug.Assert(pack.completion_mask.SequenceEqual(new byte[] {(byte)252, (byte)208, (byte)176, (byte)122, (byte)197, (byte)124, (byte)8, (byte)89, (byte)226, (byte)202}));
                Debug.Assert(pack.compass_id == (byte)(byte)94);
                Debug.Assert(pack.completion_pct == (byte)(byte)216);
            };
            GroundControl.MAG_CAL_PROGRESS p191 = CommunicationChannel.new_MAG_CAL_PROGRESS();
            PH.setPack(p191);
            p191.direction_x = (float) -7.97578E35F;
            p191.cal_mask = (byte)(byte)172;
            p191.attempt = (byte)(byte)50;
            p191.direction_y = (float)7.610753E36F;
            p191.cal_status = MAG_CAL_STATUS.MAG_CAL_NOT_STARTED;
            p191.completion_mask_SET(new byte[] {(byte)252, (byte)208, (byte)176, (byte)122, (byte)197, (byte)124, (byte)8, (byte)89, (byte)226, (byte)202}, 0) ;
            p191.direction_z = (float) -2.5340792E38F;
            p191.completion_pct = (byte)(byte)216;
            p191.compass_id = (byte)(byte)94;
            CommunicationChannel.instance.send(p191);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnMAG_CAL_REPORTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ofs_x == (float) -5.840176E37F);
                Debug.Assert(pack.offdiag_x == (float)2.8843356E37F);
                Debug.Assert(pack.fitness == (float)2.344175E38F);
                Debug.Assert(pack.offdiag_z == (float) -2.134073E38F);
                Debug.Assert(pack.diag_z == (float)2.3111347E38F);
                Debug.Assert(pack.diag_y == (float) -1.4389175E38F);
                Debug.Assert(pack.cal_mask == (byte)(byte)182);
                Debug.Assert(pack.cal_status == MAG_CAL_STATUS.MAG_CAL_NOT_STARTED);
                Debug.Assert(pack.autosaved == (byte)(byte)233);
                Debug.Assert(pack.compass_id == (byte)(byte)167);
                Debug.Assert(pack.ofs_y == (float) -2.33844E38F);
                Debug.Assert(pack.diag_x == (float)1.778914E38F);
                Debug.Assert(pack.ofs_z == (float)3.2089468E38F);
                Debug.Assert(pack.offdiag_y == (float) -2.0572858E38F);
            };
            GroundControl.MAG_CAL_REPORT p192 = CommunicationChannel.new_MAG_CAL_REPORT();
            PH.setPack(p192);
            p192.offdiag_z = (float) -2.134073E38F;
            p192.diag_x = (float)1.778914E38F;
            p192.diag_y = (float) -1.4389175E38F;
            p192.diag_z = (float)2.3111347E38F;
            p192.ofs_z = (float)3.2089468E38F;
            p192.offdiag_x = (float)2.8843356E37F;
            p192.autosaved = (byte)(byte)233;
            p192.cal_mask = (byte)(byte)182;
            p192.fitness = (float)2.344175E38F;
            p192.ofs_x = (float) -5.840176E37F;
            p192.offdiag_y = (float) -2.0572858E38F;
            p192.compass_id = (byte)(byte)167;
            p192.cal_status = MAG_CAL_STATUS.MAG_CAL_NOT_STARTED;
            p192.ofs_y = (float) -2.33844E38F;
            CommunicationChannel.instance.send(p192);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnEKF_STATUS_REPORTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.pos_vert_variance == (float)3.0943718E38F);
                Debug.Assert(pack.pos_horiz_variance == (float)3.3897993E38F);
                Debug.Assert(pack.terrain_alt_variance == (float)2.1381807E37F);
                Debug.Assert(pack.velocity_variance == (float) -2.3683457E38F);
                Debug.Assert(pack.flags == (EKF_STATUS_FLAGS.EKF_POS_VERT_AGL |
                                            EKF_STATUS_FLAGS.EKF_POS_HORIZ_ABS |
                                            EKF_STATUS_FLAGS.EKF_VELOCITY_VERT |
                                            EKF_STATUS_FLAGS.EKF_CONST_POS_MODE));
                Debug.Assert(pack.compass_variance == (float) -2.8511652E38F);
            };
            GroundControl.EKF_STATUS_REPORT p193 = CommunicationChannel.new_EKF_STATUS_REPORT();
            PH.setPack(p193);
            p193.velocity_variance = (float) -2.3683457E38F;
            p193.pos_vert_variance = (float)3.0943718E38F;
            p193.flags = (EKF_STATUS_FLAGS.EKF_POS_VERT_AGL |
                          EKF_STATUS_FLAGS.EKF_POS_HORIZ_ABS |
                          EKF_STATUS_FLAGS.EKF_VELOCITY_VERT |
                          EKF_STATUS_FLAGS.EKF_CONST_POS_MODE);
            p193.pos_horiz_variance = (float)3.3897993E38F;
            p193.compass_variance = (float) -2.8511652E38F;
            p193.terrain_alt_variance = (float)2.1381807E37F;
            CommunicationChannel.instance.send(p193);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnPID_TUNINGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.D == (float) -2.8035328E38F);
                Debug.Assert(pack.desired == (float)2.7613018E38F);
                Debug.Assert(pack.I == (float) -6.7264477E37F);
                Debug.Assert(pack.FF == (float)6.762405E37F);
                Debug.Assert(pack.axis == PID_TUNING_AXIS.PID_TUNING_YAW);
                Debug.Assert(pack.P == (float) -3.702159E37F);
                Debug.Assert(pack.achieved == (float)2.628422E38F);
            };
            GroundControl.PID_TUNING p194 = CommunicationChannel.new_PID_TUNING();
            PH.setPack(p194);
            p194.P = (float) -3.702159E37F;
            p194.FF = (float)6.762405E37F;
            p194.desired = (float)2.7613018E38F;
            p194.achieved = (float)2.628422E38F;
            p194.axis = PID_TUNING_AXIS.PID_TUNING_YAW;
            p194.I = (float) -6.7264477E37F;
            p194.D = (float) -2.8035328E38F;
            CommunicationChannel.instance.send(p194);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnGIMBAL_REPORTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.delta_angle_y == (float) -9.849839E37F);
                Debug.Assert(pack.joint_roll == (float)1.2718951E38F);
                Debug.Assert(pack.joint_el == (float)1.8029306E38F);
                Debug.Assert(pack.delta_velocity_x == (float)1.0574846E38F);
                Debug.Assert(pack.delta_velocity_z == (float)4.8582263E37F);
                Debug.Assert(pack.delta_time == (float) -1.5546639E38F);
                Debug.Assert(pack.target_system == (byte)(byte)200);
                Debug.Assert(pack.delta_angle_z == (float) -2.8778078E38F);
                Debug.Assert(pack.joint_az == (float) -9.439661E37F);
                Debug.Assert(pack.delta_angle_x == (float) -6.87369E35F);
                Debug.Assert(pack.delta_velocity_y == (float) -2.4846096E37F);
                Debug.Assert(pack.target_component == (byte)(byte)42);
            };
            GroundControl.GIMBAL_REPORT p200 = CommunicationChannel.new_GIMBAL_REPORT();
            PH.setPack(p200);
            p200.joint_roll = (float)1.2718951E38F;
            p200.target_system = (byte)(byte)200;
            p200.delta_velocity_z = (float)4.8582263E37F;
            p200.delta_velocity_y = (float) -2.4846096E37F;
            p200.delta_time = (float) -1.5546639E38F;
            p200.delta_velocity_x = (float)1.0574846E38F;
            p200.target_component = (byte)(byte)42;
            p200.delta_angle_z = (float) -2.8778078E38F;
            p200.joint_el = (float)1.8029306E38F;
            p200.delta_angle_y = (float) -9.849839E37F;
            p200.delta_angle_x = (float) -6.87369E35F;
            p200.joint_az = (float) -9.439661E37F;
            CommunicationChannel.instance.send(p200);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnGIMBAL_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)29);
                Debug.Assert(pack.demanded_rate_z == (float)7.972681E37F);
                Debug.Assert(pack.demanded_rate_y == (float) -5.3326E37F);
                Debug.Assert(pack.demanded_rate_x == (float)2.4621843E38F);
                Debug.Assert(pack.target_component == (byte)(byte)51);
            };
            GroundControl.GIMBAL_CONTROL p201 = CommunicationChannel.new_GIMBAL_CONTROL();
            PH.setPack(p201);
            p201.target_system = (byte)(byte)29;
            p201.demanded_rate_y = (float) -5.3326E37F;
            p201.demanded_rate_x = (float)2.4621843E38F;
            p201.target_component = (byte)(byte)51;
            p201.demanded_rate_z = (float)7.972681E37F;
            CommunicationChannel.instance.send(p201);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnGIMBAL_TORQUE_CMD_REPORTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.rl_torque_cmd == (short)(short) -19720);
                Debug.Assert(pack.el_torque_cmd == (short)(short) -6706);
                Debug.Assert(pack.az_torque_cmd == (short)(short) -20738);
                Debug.Assert(pack.target_component == (byte)(byte)249);
                Debug.Assert(pack.target_system == (byte)(byte)102);
            };
            GroundControl.GIMBAL_TORQUE_CMD_REPORT p214 = CommunicationChannel.new_GIMBAL_TORQUE_CMD_REPORT();
            PH.setPack(p214);
            p214.target_component = (byte)(byte)249;
            p214.el_torque_cmd = (short)(short) -6706;
            p214.az_torque_cmd = (short)(short) -20738;
            p214.rl_torque_cmd = (short)(short) -19720;
            p214.target_system = (byte)(byte)102;
            CommunicationChannel.instance.send(p214);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnGOPRO_HEARTBEATReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.status == GOPRO_HEARTBEAT_STATUS.GOPRO_HEARTBEAT_STATUS_CONNECTED);
                Debug.Assert(pack.flags == GOPRO_HEARTBEAT_FLAGS.GOPRO_FLAG_RECORDING);
                Debug.Assert(pack.capture_mode == GOPRO_CAPTURE_MODE.GOPRO_CAPTURE_MODE_MULTI_SHOT);
            };
            GroundControl.GOPRO_HEARTBEAT p215 = CommunicationChannel.new_GOPRO_HEARTBEAT();
            PH.setPack(p215);
            p215.flags = GOPRO_HEARTBEAT_FLAGS.GOPRO_FLAG_RECORDING;
            p215.status = GOPRO_HEARTBEAT_STATUS.GOPRO_HEARTBEAT_STATUS_CONNECTED;
            p215.capture_mode = GOPRO_CAPTURE_MODE.GOPRO_CAPTURE_MODE_MULTI_SHOT;
            CommunicationChannel.instance.send(p215);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnGOPRO_GET_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)78);
                Debug.Assert(pack.cmd_id == GOPRO_COMMAND.GOPRO_COMMAND_CHARGING);
                Debug.Assert(pack.target_component == (byte)(byte)246);
            };
            GroundControl.GOPRO_GET_REQUEST p216 = CommunicationChannel.new_GOPRO_GET_REQUEST();
            PH.setPack(p216);
            p216.cmd_id = GOPRO_COMMAND.GOPRO_COMMAND_CHARGING;
            p216.target_component = (byte)(byte)246;
            p216.target_system = (byte)(byte)78;
            CommunicationChannel.instance.send(p216);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnGOPRO_GET_RESPONSEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.cmd_id == GOPRO_COMMAND.GOPRO_COMMAND_PROTUNE_GAIN);
                Debug.Assert(pack.status == GOPRO_REQUEST_STATUS.GOPRO_REQUEST_SUCCESS);
                Debug.Assert(pack.value.SequenceEqual(new byte[] {(byte)244, (byte)158, (byte)170, (byte)108}));
            };
            GroundControl.GOPRO_GET_RESPONSE p217 = CommunicationChannel.new_GOPRO_GET_RESPONSE();
            PH.setPack(p217);
            p217.cmd_id = GOPRO_COMMAND.GOPRO_COMMAND_PROTUNE_GAIN;
            p217.value_SET(new byte[] {(byte)244, (byte)158, (byte)170, (byte)108}, 0) ;
            p217.status = GOPRO_REQUEST_STATUS.GOPRO_REQUEST_SUCCESS;
            CommunicationChannel.instance.send(p217);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnGOPRO_SET_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)169);
                Debug.Assert(pack.target_system == (byte)(byte)232);
                Debug.Assert(pack.value.SequenceEqual(new byte[] {(byte)91, (byte)198, (byte)30, (byte)137}));
                Debug.Assert(pack.cmd_id == GOPRO_COMMAND.GOPRO_COMMAND_VIDEO_SETTINGS);
            };
            GroundControl.GOPRO_SET_REQUEST p218 = CommunicationChannel.new_GOPRO_SET_REQUEST();
            PH.setPack(p218);
            p218.target_system = (byte)(byte)232;
            p218.value_SET(new byte[] {(byte)91, (byte)198, (byte)30, (byte)137}, 0) ;
            p218.target_component = (byte)(byte)169;
            p218.cmd_id = GOPRO_COMMAND.GOPRO_COMMAND_VIDEO_SETTINGS;
            CommunicationChannel.instance.send(p218);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnGOPRO_SET_RESPONSEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.cmd_id == GOPRO_COMMAND.GOPRO_COMMAND_CHARGING);
                Debug.Assert(pack.status == GOPRO_REQUEST_STATUS.GOPRO_REQUEST_FAILED);
            };
            GroundControl.GOPRO_SET_RESPONSE p219 = CommunicationChannel.new_GOPRO_SET_RESPONSE();
            PH.setPack(p219);
            p219.status = GOPRO_REQUEST_STATUS.GOPRO_REQUEST_FAILED;
            p219.cmd_id = GOPRO_COMMAND.GOPRO_COMMAND_CHARGING;
            CommunicationChannel.instance.send(p219);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnRPMReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.rpm1 == (float) -3.001679E38F);
                Debug.Assert(pack.rpm2 == (float)2.6782232E38F);
            };
            GroundControl.RPM p226 = CommunicationChannel.new_RPM();
            PH.setPack(p226);
            p226.rpm1 = (float) -3.001679E38F;
            p226.rpm2 = (float)2.6782232E38F;
            CommunicationChannel.instance.send(p226);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            CommunicationChannel.instance.OnESTIMATOR_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.flags == (ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_HORIZ_ABS));
                Debug.Assert(pack.pos_horiz_ratio == (float)1.0746153E38F);
                Debug.Assert(pack.pos_horiz_accuracy == (float) -1.8595657E38F);
                Debug.Assert(pack.time_usec == (ulong)8431090286010670498L);
                Debug.Assert(pack.pos_vert_ratio == (float) -8.66818E37F);
                Debug.Assert(pack.vel_ratio == (float) -1.6600645E38F);
                Debug.Assert(pack.pos_vert_accuracy == (float)4.0551222E37F);
                Debug.Assert(pack.hagl_ratio == (float)1.773713E38F);
                Debug.Assert(pack.mag_ratio == (float) -1.4564522E38F);
                Debug.Assert(pack.tas_ratio == (float)2.6202537E38F);
            };
            GroundControl.ESTIMATOR_STATUS p230 = CommunicationChannel.new_ESTIMATOR_STATUS();
            PH.setPack(p230);
            p230.flags = (ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_HORIZ_ABS);
            p230.time_usec = (ulong)8431090286010670498L;
            p230.tas_ratio = (float)2.6202537E38F;
            p230.mag_ratio = (float) -1.4564522E38F;
            p230.pos_vert_accuracy = (float)4.0551222E37F;
            p230.pos_horiz_accuracy = (float) -1.8595657E38F;
            p230.hagl_ratio = (float)1.773713E38F;
            p230.vel_ratio = (float) -1.6600645E38F;
            p230.pos_horiz_ratio = (float)1.0746153E38F;
            p230.pos_vert_ratio = (float) -8.66818E37F;
            CommunicationChannel.instance.send(p230);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnWIND_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.wind_y == (float) -2.5845253E37F);
                Debug.Assert(pack.var_vert == (float) -2.2591006E38F);
                Debug.Assert(pack.wind_x == (float) -2.001806E38F);
                Debug.Assert(pack.wind_alt == (float) -1.0637644E38F);
                Debug.Assert(pack.horiz_accuracy == (float)1.1838765E38F);
                Debug.Assert(pack.time_usec == (ulong)1955950908138220228L);
                Debug.Assert(pack.wind_z == (float) -2.872788E36F);
                Debug.Assert(pack.var_horiz == (float) -1.73734E38F);
                Debug.Assert(pack.vert_accuracy == (float) -3.0891033E37F);
            };
            GroundControl.WIND_COV p231 = CommunicationChannel.new_WIND_COV();
            PH.setPack(p231);
            p231.vert_accuracy = (float) -3.0891033E37F;
            p231.wind_y = (float) -2.5845253E37F;
            p231.time_usec = (ulong)1955950908138220228L;
            p231.wind_alt = (float) -1.0637644E38F;
            p231.wind_z = (float) -2.872788E36F;
            p231.wind_x = (float) -2.001806E38F;
            p231.var_horiz = (float) -1.73734E38F;
            p231.var_vert = (float) -2.2591006E38F;
            p231.horiz_accuracy = (float)1.1838765E38F;
            CommunicationChannel.instance.send(p231);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_INPUTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vn == (float) -1.4801128E38F);
                Debug.Assert(pack.gps_id == (byte)(byte)230);
                Debug.Assert(pack.vdop == (float) -1.1347633E38F);
                Debug.Assert(pack.ignore_flags == (GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY |
                                                   GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY |
                                                   GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VDOP |
                                                   GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VEL_HORIZ |
                                                   GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_HDOP));
                Debug.Assert(pack.speed_accuracy == (float) -1.5425346E38F);
                Debug.Assert(pack.lon == (int)342927668);
                Debug.Assert(pack.satellites_visible == (byte)(byte)179);
                Debug.Assert(pack.alt == (float)2.9852164E38F);
                Debug.Assert(pack.vd == (float)2.98871E38F);
                Debug.Assert(pack.vert_accuracy == (float) -2.2026332E38F);
                Debug.Assert(pack.time_week_ms == (uint)4020248905U);
                Debug.Assert(pack.time_week == (ushort)(ushort)48849);
                Debug.Assert(pack.hdop == (float) -2.6355411E38F);
                Debug.Assert(pack.fix_type == (byte)(byte)136);
                Debug.Assert(pack.horiz_accuracy == (float) -1.061246E37F);
                Debug.Assert(pack.lat == (int)1379038520);
                Debug.Assert(pack.ve == (float)2.5911113E37F);
                Debug.Assert(pack.time_usec == (ulong)3104845128705913635L);
            };
            GroundControl.GPS_INPUT p232 = CommunicationChannel.new_GPS_INPUT();
            PH.setPack(p232);
            p232.hdop = (float) -2.6355411E38F;
            p232.speed_accuracy = (float) -1.5425346E38F;
            p232.lon = (int)342927668;
            p232.alt = (float)2.9852164E38F;
            p232.vd = (float)2.98871E38F;
            p232.lat = (int)1379038520;
            p232.time_week_ms = (uint)4020248905U;
            p232.ve = (float)2.5911113E37F;
            p232.ignore_flags = (GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY |
                                 GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY |
                                 GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VDOP |
                                 GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VEL_HORIZ |
                                 GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_HDOP);
            p232.gps_id = (byte)(byte)230;
            p232.satellites_visible = (byte)(byte)179;
            p232.time_usec = (ulong)3104845128705913635L;
            p232.horiz_accuracy = (float) -1.061246E37F;
            p232.time_week = (ushort)(ushort)48849;
            p232.vdop = (float) -1.1347633E38F;
            p232.fix_type = (byte)(byte)136;
            p232.vert_accuracy = (float) -2.2026332E38F;
            p232.vn = (float) -1.4801128E38F;
            CommunicationChannel.instance.send(p232);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_RTCM_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.flags == (byte)(byte)115);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)64, (byte)41, (byte)138, (byte)170, (byte)86, (byte)137, (byte)31, (byte)125, (byte)98, (byte)127, (byte)9, (byte)25, (byte)82, (byte)220, (byte)45, (byte)58, (byte)84, (byte)44, (byte)165, (byte)32, (byte)36, (byte)74, (byte)16, (byte)39, (byte)195, (byte)224, (byte)13, (byte)116, (byte)89, (byte)144, (byte)205, (byte)104, (byte)183, (byte)164, (byte)123, (byte)62, (byte)36, (byte)42, (byte)102, (byte)140, (byte)180, (byte)8, (byte)62, (byte)36, (byte)177, (byte)222, (byte)212, (byte)131, (byte)215, (byte)7, (byte)223, (byte)198, (byte)225, (byte)197, (byte)247, (byte)169, (byte)52, (byte)132, (byte)77, (byte)216, (byte)49, (byte)7, (byte)121, (byte)54, (byte)158, (byte)170, (byte)64, (byte)198, (byte)18, (byte)118, (byte)215, (byte)218, (byte)192, (byte)105, (byte)94, (byte)192, (byte)38, (byte)243, (byte)69, (byte)59, (byte)148, (byte)143, (byte)212, (byte)151, (byte)106, (byte)129, (byte)58, (byte)201, (byte)81, (byte)23, (byte)64, (byte)16, (byte)5, (byte)125, (byte)39, (byte)133, (byte)98, (byte)78, (byte)29, (byte)206, (byte)249, (byte)8, (byte)47, (byte)5, (byte)245, (byte)19, (byte)104, (byte)69, (byte)70, (byte)223, (byte)176, (byte)155, (byte)49, (byte)183, (byte)160, (byte)29, (byte)177, (byte)250, (byte)194, (byte)245, (byte)63, (byte)113, (byte)85, (byte)31, (byte)47, (byte)26, (byte)24, (byte)76, (byte)118, (byte)47, (byte)203, (byte)73, (byte)76, (byte)151, (byte)89, (byte)104, (byte)242, (byte)160, (byte)57, (byte)30, (byte)88, (byte)49, (byte)21, (byte)222, (byte)152, (byte)69, (byte)244, (byte)1, (byte)168, (byte)34, (byte)33, (byte)171, (byte)72, (byte)129, (byte)36, (byte)35, (byte)229, (byte)186, (byte)140, (byte)192, (byte)80, (byte)106, (byte)12, (byte)136, (byte)87, (byte)96, (byte)243, (byte)14, (byte)76, (byte)50, (byte)102, (byte)89, (byte)115, (byte)149, (byte)154, (byte)76, (byte)0, (byte)207, (byte)72, (byte)91}));
                Debug.Assert(pack.len == (byte)(byte)55);
            };
            GroundControl.GPS_RTCM_DATA p233 = CommunicationChannel.new_GPS_RTCM_DATA();
            PH.setPack(p233);
            p233.data__SET(new byte[] {(byte)64, (byte)41, (byte)138, (byte)170, (byte)86, (byte)137, (byte)31, (byte)125, (byte)98, (byte)127, (byte)9, (byte)25, (byte)82, (byte)220, (byte)45, (byte)58, (byte)84, (byte)44, (byte)165, (byte)32, (byte)36, (byte)74, (byte)16, (byte)39, (byte)195, (byte)224, (byte)13, (byte)116, (byte)89, (byte)144, (byte)205, (byte)104, (byte)183, (byte)164, (byte)123, (byte)62, (byte)36, (byte)42, (byte)102, (byte)140, (byte)180, (byte)8, (byte)62, (byte)36, (byte)177, (byte)222, (byte)212, (byte)131, (byte)215, (byte)7, (byte)223, (byte)198, (byte)225, (byte)197, (byte)247, (byte)169, (byte)52, (byte)132, (byte)77, (byte)216, (byte)49, (byte)7, (byte)121, (byte)54, (byte)158, (byte)170, (byte)64, (byte)198, (byte)18, (byte)118, (byte)215, (byte)218, (byte)192, (byte)105, (byte)94, (byte)192, (byte)38, (byte)243, (byte)69, (byte)59, (byte)148, (byte)143, (byte)212, (byte)151, (byte)106, (byte)129, (byte)58, (byte)201, (byte)81, (byte)23, (byte)64, (byte)16, (byte)5, (byte)125, (byte)39, (byte)133, (byte)98, (byte)78, (byte)29, (byte)206, (byte)249, (byte)8, (byte)47, (byte)5, (byte)245, (byte)19, (byte)104, (byte)69, (byte)70, (byte)223, (byte)176, (byte)155, (byte)49, (byte)183, (byte)160, (byte)29, (byte)177, (byte)250, (byte)194, (byte)245, (byte)63, (byte)113, (byte)85, (byte)31, (byte)47, (byte)26, (byte)24, (byte)76, (byte)118, (byte)47, (byte)203, (byte)73, (byte)76, (byte)151, (byte)89, (byte)104, (byte)242, (byte)160, (byte)57, (byte)30, (byte)88, (byte)49, (byte)21, (byte)222, (byte)152, (byte)69, (byte)244, (byte)1, (byte)168, (byte)34, (byte)33, (byte)171, (byte)72, (byte)129, (byte)36, (byte)35, (byte)229, (byte)186, (byte)140, (byte)192, (byte)80, (byte)106, (byte)12, (byte)136, (byte)87, (byte)96, (byte)243, (byte)14, (byte)76, (byte)50, (byte)102, (byte)89, (byte)115, (byte)149, (byte)154, (byte)76, (byte)0, (byte)207, (byte)72, (byte)91}, 0) ;
            p233.len = (byte)(byte)55;
            p233.flags = (byte)(byte)115;
            CommunicationChannel.instance.send(p233);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIGH_LATENCYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.failsafe == (byte)(byte)142);
                Debug.Assert(pack.climb_rate == (sbyte)(sbyte)4);
                Debug.Assert(pack.base_mode == (MAV_MODE_FLAG.MAV_MODE_FLAG_AUTO_ENABLED |
                                                MAV_MODE_FLAG.MAV_MODE_FLAG_TEST_ENABLED |
                                                MAV_MODE_FLAG.MAV_MODE_FLAG_STABILIZE_ENABLED |
                                                MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED |
                                                MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED));
                Debug.Assert(pack.heading_sp == (short)(short)8962);
                Debug.Assert(pack.gps_fix_type == GPS_FIX_TYPE.GPS_FIX_TYPE_3D_FIX);
                Debug.Assert(pack.wp_num == (byte)(byte)167);
                Debug.Assert(pack.throttle == (sbyte)(sbyte) - 3);
                Debug.Assert(pack.longitude == (int)1851592620);
                Debug.Assert(pack.latitude == (int) -1887364608);
                Debug.Assert(pack.roll == (short)(short)18921);
                Debug.Assert(pack.gps_nsat == (byte)(byte)48);
                Debug.Assert(pack.temperature == (sbyte)(sbyte) - 37);
                Debug.Assert(pack.battery_remaining == (byte)(byte)22);
                Debug.Assert(pack.groundspeed == (byte)(byte)146);
                Debug.Assert(pack.custom_mode == (uint)3524871651U);
                Debug.Assert(pack.altitude_amsl == (short)(short)31728);
                Debug.Assert(pack.pitch == (short)(short)8843);
                Debug.Assert(pack.altitude_sp == (short)(short)32068);
                Debug.Assert(pack.wp_distance == (ushort)(ushort)25144);
                Debug.Assert(pack.airspeed_sp == (byte)(byte)115);
                Debug.Assert(pack.landed_state == MAV_LANDED_STATE.MAV_LANDED_STATE_IN_AIR);
                Debug.Assert(pack.heading == (ushort)(ushort)39161);
                Debug.Assert(pack.airspeed == (byte)(byte)19);
                Debug.Assert(pack.temperature_air == (sbyte)(sbyte)31);
            };
            GroundControl.HIGH_LATENCY p234 = CommunicationChannel.new_HIGH_LATENCY();
            PH.setPack(p234);
            p234.failsafe = (byte)(byte)142;
            p234.temperature_air = (sbyte)(sbyte)31;
            p234.gps_nsat = (byte)(byte)48;
            p234.custom_mode = (uint)3524871651U;
            p234.temperature = (sbyte)(sbyte) - 37;
            p234.airspeed = (byte)(byte)19;
            p234.landed_state = MAV_LANDED_STATE.MAV_LANDED_STATE_IN_AIR;
            p234.climb_rate = (sbyte)(sbyte)4;
            p234.roll = (short)(short)18921;
            p234.altitude_sp = (short)(short)32068;
            p234.groundspeed = (byte)(byte)146;
            p234.gps_fix_type = GPS_FIX_TYPE.GPS_FIX_TYPE_3D_FIX;
            p234.wp_num = (byte)(byte)167;
            p234.battery_remaining = (byte)(byte)22;
            p234.airspeed_sp = (byte)(byte)115;
            p234.wp_distance = (ushort)(ushort)25144;
            p234.altitude_amsl = (short)(short)31728;
            p234.longitude = (int)1851592620;
            p234.latitude = (int) -1887364608;
            p234.base_mode = (MAV_MODE_FLAG.MAV_MODE_FLAG_AUTO_ENABLED |
                              MAV_MODE_FLAG.MAV_MODE_FLAG_TEST_ENABLED |
                              MAV_MODE_FLAG.MAV_MODE_FLAG_STABILIZE_ENABLED |
                              MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED |
                              MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED);
            p234.heading = (ushort)(ushort)39161;
            p234.heading_sp = (short)(short)8962;
            p234.pitch = (short)(short)8843;
            p234.throttle = (sbyte)(sbyte) - 3;
            CommunicationChannel.instance.send(p234);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnVIBRATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)548018999232611835L);
                Debug.Assert(pack.vibration_y == (float)5.576237E37F);
                Debug.Assert(pack.vibration_z == (float)2.3063495E38F);
                Debug.Assert(pack.vibration_x == (float) -1.42842E37F);
                Debug.Assert(pack.clipping_2 == (uint)3886725252U);
                Debug.Assert(pack.clipping_1 == (uint)3359028326U);
                Debug.Assert(pack.clipping_0 == (uint)3610688568U);
            };
            GroundControl.VIBRATION p241 = CommunicationChannel.new_VIBRATION();
            PH.setPack(p241);
            p241.vibration_x = (float) -1.42842E37F;
            p241.vibration_z = (float)2.3063495E38F;
            p241.time_usec = (ulong)548018999232611835L;
            p241.vibration_y = (float)5.576237E37F;
            p241.clipping_2 = (uint)3886725252U;
            p241.clipping_1 = (uint)3359028326U;
            p241.clipping_0 = (uint)3610688568U;
            CommunicationChannel.instance.send(p241);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHOME_POSITIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.z == (float)7.964864E37F);
                Debug.Assert(pack.approach_y == (float) -1.1354042E38F);
                Debug.Assert(pack.approach_z == (float)1.7896085E38F);
                Debug.Assert(pack.x == (float) -3.2934652E38F);
                Debug.Assert(pack.altitude == (int)1337466216);
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)8948412196787226633L);
                Debug.Assert(pack.latitude == (int) -609912353);
                Debug.Assert(pack.approach_x == (float) -2.1032686E38F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {3.3365677E38F, -1.2432586E38F, 2.9669851E38F, -2.22495E37F}));
                Debug.Assert(pack.y == (float) -2.3539996E38F);
                Debug.Assert(pack.longitude == (int)1200636368);
            };
            GroundControl.HOME_POSITION p242 = CommunicationChannel.new_HOME_POSITION();
            PH.setPack(p242);
            p242.time_usec_SET((ulong)8948412196787226633L, PH) ;
            p242.latitude = (int) -609912353;
            p242.y = (float) -2.3539996E38F;
            p242.x = (float) -3.2934652E38F;
            p242.approach_x = (float) -2.1032686E38F;
            p242.approach_z = (float)1.7896085E38F;
            p242.altitude = (int)1337466216;
            p242.q_SET(new float[] {3.3365677E38F, -1.2432586E38F, 2.9669851E38F, -2.22495E37F}, 0) ;
            p242.longitude = (int)1200636368;
            p242.approach_y = (float) -1.1354042E38F;
            p242.z = (float)7.964864E37F;
            CommunicationChannel.instance.send(p242);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_HOME_POSITIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)151);
                Debug.Assert(pack.z == (float)7.2888855E37F);
                Debug.Assert(pack.latitude == (int)1041161828);
                Debug.Assert(pack.approach_y == (float) -2.7224373E38F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-2.2322414E38F, -1.6576409E38F, -1.0335928E38F, -2.574021E38F}));
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)2746790828482661431L);
                Debug.Assert(pack.x == (float)1.6995901E38F);
                Debug.Assert(pack.longitude == (int)581570126);
                Debug.Assert(pack.approach_z == (float) -2.4864285E38F);
                Debug.Assert(pack.altitude == (int) -1424797148);
                Debug.Assert(pack.y == (float)2.1305493E38F);
                Debug.Assert(pack.approach_x == (float)2.4790228E38F);
            };
            GroundControl.SET_HOME_POSITION p243 = CommunicationChannel.new_SET_HOME_POSITION();
            PH.setPack(p243);
            p243.target_system = (byte)(byte)151;
            p243.approach_z = (float) -2.4864285E38F;
            p243.time_usec_SET((ulong)2746790828482661431L, PH) ;
            p243.z = (float)7.2888855E37F;
            p243.q_SET(new float[] {-2.2322414E38F, -1.6576409E38F, -1.0335928E38F, -2.574021E38F}, 0) ;
            p243.approach_x = (float)2.4790228E38F;
            p243.approach_y = (float) -2.7224373E38F;
            p243.longitude = (int)581570126;
            p243.x = (float)1.6995901E38F;
            p243.altitude = (int) -1424797148;
            p243.y = (float)2.1305493E38F;
            p243.latitude = (int)1041161828;
            CommunicationChannel.instance.send(p243);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMESSAGE_INTERVALReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.message_id == (ushort)(ushort)54551);
                Debug.Assert(pack.interval_us == (int) -2126446594);
            };
            GroundControl.MESSAGE_INTERVAL p244 = CommunicationChannel.new_MESSAGE_INTERVAL();
            PH.setPack(p244);
            p244.message_id = (ushort)(ushort)54551;
            p244.interval_us = (int) -2126446594;
            CommunicationChannel.instance.send(p244);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnEXTENDED_SYS_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vtol_state == MAV_VTOL_STATE.MAV_VTOL_STATE_TRANSITION_TO_MC);
                Debug.Assert(pack.landed_state == MAV_LANDED_STATE.MAV_LANDED_STATE_TAKEOFF);
            };
            GroundControl.EXTENDED_SYS_STATE p245 = CommunicationChannel.new_EXTENDED_SYS_STATE();
            PH.setPack(p245);
            p245.landed_state = MAV_LANDED_STATE.MAV_LANDED_STATE_TAKEOFF;
            p245.vtol_state = MAV_VTOL_STATE.MAV_VTOL_STATE_TRANSITION_TO_MC;
            CommunicationChannel.instance.send(p245);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnADSB_VEHICLEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.callsign_LEN(ph) == 2);
                Debug.Assert(pack.callsign_TRY(ph).Equals("df"));
                Debug.Assert(pack.tslc == (byte)(byte)141);
                Debug.Assert(pack.ICAO_address == (uint)3791149222U);
                Debug.Assert(pack.altitude_type == ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_GEOMETRIC);
                Debug.Assert(pack.squawk == (ushort)(ushort)34859);
                Debug.Assert(pack.altitude == (int)1980874123);
                Debug.Assert(pack.lon == (int)2140142272);
                Debug.Assert(pack.lat == (int)1663130676);
                Debug.Assert(pack.ver_velocity == (short)(short) -15660);
                Debug.Assert(pack.emitter_type == ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_LARGE);
                Debug.Assert(pack.heading == (ushort)(ushort)31831);
                Debug.Assert(pack.hor_velocity == (ushort)(ushort)26878);
                Debug.Assert(pack.flags == (ADSB_FLAGS.ADSB_FLAGS_VALID_COORDS |
                                            ADSB_FLAGS.ADSB_FLAGS_SIMULATED));
            };
            GroundControl.ADSB_VEHICLE p246 = CommunicationChannel.new_ADSB_VEHICLE();
            PH.setPack(p246);
            p246.altitude = (int)1980874123;
            p246.lat = (int)1663130676;
            p246.callsign_SET("df", PH) ;
            p246.heading = (ushort)(ushort)31831;
            p246.squawk = (ushort)(ushort)34859;
            p246.tslc = (byte)(byte)141;
            p246.flags = (ADSB_FLAGS.ADSB_FLAGS_VALID_COORDS |
                          ADSB_FLAGS.ADSB_FLAGS_SIMULATED);
            p246.lon = (int)2140142272;
            p246.hor_velocity = (ushort)(ushort)26878;
            p246.ICAO_address = (uint)3791149222U;
            p246.emitter_type = ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_LARGE;
            p246.altitude_type = ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_GEOMETRIC;
            p246.ver_velocity = (short)(short) -15660;
            CommunicationChannel.instance.send(p246);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCOLLISIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.horizontal_minimum_delta == (float) -2.1347985E38F);
                Debug.Assert(pack.altitude_minimum_delta == (float) -2.9187168E38F);
                Debug.Assert(pack.id == (uint)469553919U);
                Debug.Assert(pack.time_to_minimum_delta == (float)8.86501E37F);
                Debug.Assert(pack.src_ == MAV_COLLISION_SRC.MAV_COLLISION_SRC_ADSB);
                Debug.Assert(pack.action == MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_RTL);
                Debug.Assert(pack.threat_level == (MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_LOW |
                                                   MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_NONE));
            };
            GroundControl.COLLISION p247 = CommunicationChannel.new_COLLISION();
            PH.setPack(p247);
            p247.altitude_minimum_delta = (float) -2.9187168E38F;
            p247.src_ = MAV_COLLISION_SRC.MAV_COLLISION_SRC_ADSB;
            p247.id = (uint)469553919U;
            p247.action = MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_RTL;
            p247.time_to_minimum_delta = (float)8.86501E37F;
            p247.horizontal_minimum_delta = (float) -2.1347985E38F;
            p247.threat_level = (MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_LOW |
                                 MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_NONE);
            CommunicationChannel.instance.send(p247);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnV2_EXTENSIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.message_type == (ushort)(ushort)18621);
                Debug.Assert(pack.target_component == (byte)(byte)56);
                Debug.Assert(pack.payload.SequenceEqual(new byte[] {(byte)61, (byte)52, (byte)169, (byte)106, (byte)174, (byte)35, (byte)3, (byte)174, (byte)215, (byte)191, (byte)78, (byte)195, (byte)41, (byte)67, (byte)179, (byte)211, (byte)136, (byte)18, (byte)113, (byte)196, (byte)209, (byte)90, (byte)46, (byte)245, (byte)233, (byte)41, (byte)116, (byte)143, (byte)24, (byte)205, (byte)193, (byte)28, (byte)0, (byte)245, (byte)203, (byte)92, (byte)204, (byte)226, (byte)184, (byte)66, (byte)87, (byte)36, (byte)128, (byte)155, (byte)203, (byte)3, (byte)133, (byte)45, (byte)135, (byte)244, (byte)141, (byte)212, (byte)223, (byte)91, (byte)44, (byte)8, (byte)190, (byte)134, (byte)218, (byte)67, (byte)88, (byte)186, (byte)73, (byte)11, (byte)174, (byte)204, (byte)179, (byte)167, (byte)81, (byte)45, (byte)58, (byte)80, (byte)203, (byte)211, (byte)125, (byte)156, (byte)108, (byte)199, (byte)118, (byte)63, (byte)156, (byte)205, (byte)82, (byte)182, (byte)32, (byte)42, (byte)242, (byte)69, (byte)175, (byte)127, (byte)74, (byte)98, (byte)14, (byte)125, (byte)207, (byte)13, (byte)108, (byte)119, (byte)37, (byte)223, (byte)88, (byte)255, (byte)229, (byte)103, (byte)207, (byte)69, (byte)117, (byte)219, (byte)191, (byte)227, (byte)135, (byte)116, (byte)70, (byte)82, (byte)70, (byte)65, (byte)180, (byte)23, (byte)115, (byte)199, (byte)232, (byte)86, (byte)81, (byte)42, (byte)112, (byte)223, (byte)28, (byte)253, (byte)57, (byte)38, (byte)65, (byte)252, (byte)244, (byte)83, (byte)94, (byte)175, (byte)5, (byte)98, (byte)95, (byte)107, (byte)70, (byte)88, (byte)201, (byte)89, (byte)232, (byte)193, (byte)49, (byte)112, (byte)162, (byte)213, (byte)173, (byte)157, (byte)185, (byte)229, (byte)98, (byte)163, (byte)192, (byte)229, (byte)67, (byte)168, (byte)46, (byte)229, (byte)64, (byte)174, (byte)125, (byte)224, (byte)202, (byte)158, (byte)193, (byte)211, (byte)85, (byte)28, (byte)52, (byte)25, (byte)219, (byte)168, (byte)66, (byte)138, (byte)76, (byte)54, (byte)135, (byte)73, (byte)5, (byte)200, (byte)88, (byte)93, (byte)71, (byte)158, (byte)44, (byte)115, (byte)10, (byte)32, (byte)40, (byte)38, (byte)141, (byte)203, (byte)184, (byte)36, (byte)90, (byte)88, (byte)193, (byte)23, (byte)28, (byte)120, (byte)128, (byte)219, (byte)9, (byte)28, (byte)165, (byte)97, (byte)46, (byte)188, (byte)146, (byte)31, (byte)242, (byte)90, (byte)139, (byte)236, (byte)133, (byte)212, (byte)154, (byte)234, (byte)161, (byte)166, (byte)7, (byte)157, (byte)180, (byte)78, (byte)196, (byte)123, (byte)0, (byte)145, (byte)10, (byte)179, (byte)11, (byte)229, (byte)255, (byte)221, (byte)249, (byte)16, (byte)166, (byte)86, (byte)86, (byte)153, (byte)66, (byte)41, (byte)36, (byte)171, (byte)180}));
                Debug.Assert(pack.target_network == (byte)(byte)22);
                Debug.Assert(pack.target_system == (byte)(byte)134);
            };
            GroundControl.V2_EXTENSION p248 = CommunicationChannel.new_V2_EXTENSION();
            PH.setPack(p248);
            p248.target_component = (byte)(byte)56;
            p248.target_system = (byte)(byte)134;
            p248.payload_SET(new byte[] {(byte)61, (byte)52, (byte)169, (byte)106, (byte)174, (byte)35, (byte)3, (byte)174, (byte)215, (byte)191, (byte)78, (byte)195, (byte)41, (byte)67, (byte)179, (byte)211, (byte)136, (byte)18, (byte)113, (byte)196, (byte)209, (byte)90, (byte)46, (byte)245, (byte)233, (byte)41, (byte)116, (byte)143, (byte)24, (byte)205, (byte)193, (byte)28, (byte)0, (byte)245, (byte)203, (byte)92, (byte)204, (byte)226, (byte)184, (byte)66, (byte)87, (byte)36, (byte)128, (byte)155, (byte)203, (byte)3, (byte)133, (byte)45, (byte)135, (byte)244, (byte)141, (byte)212, (byte)223, (byte)91, (byte)44, (byte)8, (byte)190, (byte)134, (byte)218, (byte)67, (byte)88, (byte)186, (byte)73, (byte)11, (byte)174, (byte)204, (byte)179, (byte)167, (byte)81, (byte)45, (byte)58, (byte)80, (byte)203, (byte)211, (byte)125, (byte)156, (byte)108, (byte)199, (byte)118, (byte)63, (byte)156, (byte)205, (byte)82, (byte)182, (byte)32, (byte)42, (byte)242, (byte)69, (byte)175, (byte)127, (byte)74, (byte)98, (byte)14, (byte)125, (byte)207, (byte)13, (byte)108, (byte)119, (byte)37, (byte)223, (byte)88, (byte)255, (byte)229, (byte)103, (byte)207, (byte)69, (byte)117, (byte)219, (byte)191, (byte)227, (byte)135, (byte)116, (byte)70, (byte)82, (byte)70, (byte)65, (byte)180, (byte)23, (byte)115, (byte)199, (byte)232, (byte)86, (byte)81, (byte)42, (byte)112, (byte)223, (byte)28, (byte)253, (byte)57, (byte)38, (byte)65, (byte)252, (byte)244, (byte)83, (byte)94, (byte)175, (byte)5, (byte)98, (byte)95, (byte)107, (byte)70, (byte)88, (byte)201, (byte)89, (byte)232, (byte)193, (byte)49, (byte)112, (byte)162, (byte)213, (byte)173, (byte)157, (byte)185, (byte)229, (byte)98, (byte)163, (byte)192, (byte)229, (byte)67, (byte)168, (byte)46, (byte)229, (byte)64, (byte)174, (byte)125, (byte)224, (byte)202, (byte)158, (byte)193, (byte)211, (byte)85, (byte)28, (byte)52, (byte)25, (byte)219, (byte)168, (byte)66, (byte)138, (byte)76, (byte)54, (byte)135, (byte)73, (byte)5, (byte)200, (byte)88, (byte)93, (byte)71, (byte)158, (byte)44, (byte)115, (byte)10, (byte)32, (byte)40, (byte)38, (byte)141, (byte)203, (byte)184, (byte)36, (byte)90, (byte)88, (byte)193, (byte)23, (byte)28, (byte)120, (byte)128, (byte)219, (byte)9, (byte)28, (byte)165, (byte)97, (byte)46, (byte)188, (byte)146, (byte)31, (byte)242, (byte)90, (byte)139, (byte)236, (byte)133, (byte)212, (byte)154, (byte)234, (byte)161, (byte)166, (byte)7, (byte)157, (byte)180, (byte)78, (byte)196, (byte)123, (byte)0, (byte)145, (byte)10, (byte)179, (byte)11, (byte)229, (byte)255, (byte)221, (byte)249, (byte)16, (byte)166, (byte)86, (byte)86, (byte)153, (byte)66, (byte)41, (byte)36, (byte)171, (byte)180}, 0) ;
            p248.target_network = (byte)(byte)22;
            p248.message_type = (ushort)(ushort)18621;
            CommunicationChannel.instance.send(p248);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMEMORY_VECTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.value.SequenceEqual(new sbyte[] {(sbyte)12, (sbyte)44, (sbyte)32, (sbyte)31, (sbyte)42, (sbyte)55, (sbyte)68, (sbyte) - 49, (sbyte)24, (sbyte)89, (sbyte)45, (sbyte) - 35, (sbyte) - 28, (sbyte) - 74, (sbyte)85, (sbyte)101, (sbyte) - 62, (sbyte)24, (sbyte)76, (sbyte) - 107, (sbyte)2, (sbyte) - 8, (sbyte)89, (sbyte) - 74, (sbyte)0, (sbyte) - 39, (sbyte) - 23, (sbyte)36, (sbyte)0, (sbyte) - 77, (sbyte) - 36, (sbyte)125}));
                Debug.Assert(pack.type == (byte)(byte)46);
                Debug.Assert(pack.address == (ushort)(ushort)32284);
                Debug.Assert(pack.ver == (byte)(byte)171);
            };
            GroundControl.MEMORY_VECT p249 = CommunicationChannel.new_MEMORY_VECT();
            PH.setPack(p249);
            p249.ver = (byte)(byte)171;
            p249.value_SET(new sbyte[] {(sbyte)12, (sbyte)44, (sbyte)32, (sbyte)31, (sbyte)42, (sbyte)55, (sbyte)68, (sbyte) - 49, (sbyte)24, (sbyte)89, (sbyte)45, (sbyte) - 35, (sbyte) - 28, (sbyte) - 74, (sbyte)85, (sbyte)101, (sbyte) - 62, (sbyte)24, (sbyte)76, (sbyte) - 107, (sbyte)2, (sbyte) - 8, (sbyte)89, (sbyte) - 74, (sbyte)0, (sbyte) - 39, (sbyte) - 23, (sbyte)36, (sbyte)0, (sbyte) - 77, (sbyte) - 36, (sbyte)125}, 0) ;
            p249.address = (ushort)(ushort)32284;
            p249.type = (byte)(byte)46;
            CommunicationChannel.instance.send(p249);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnDEBUG_VECTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.name_LEN(ph) == 2);
                Debug.Assert(pack.name_TRY(ph).Equals("ux"));
                Debug.Assert(pack.z == (float) -2.8087927E38F);
                Debug.Assert(pack.x == (float) -3.286439E38F);
                Debug.Assert(pack.y == (float)2.254512E38F);
                Debug.Assert(pack.time_usec == (ulong)2820747069097901969L);
            };
            GroundControl.DEBUG_VECT p250 = CommunicationChannel.new_DEBUG_VECT();
            PH.setPack(p250);
            p250.z = (float) -2.8087927E38F;
            p250.x = (float) -3.286439E38F;
            p250.time_usec = (ulong)2820747069097901969L;
            p250.name_SET("ux", PH) ;
            p250.y = (float)2.254512E38F;
            CommunicationChannel.instance.send(p250);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnNAMED_VALUE_FLOATReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)2236020060U);
                Debug.Assert(pack.value == (float) -1.664965E38F);
                Debug.Assert(pack.name_LEN(ph) == 3);
                Debug.Assert(pack.name_TRY(ph).Equals("vad"));
            };
            GroundControl.NAMED_VALUE_FLOAT p251 = CommunicationChannel.new_NAMED_VALUE_FLOAT();
            PH.setPack(p251);
            p251.name_SET("vad", PH) ;
            p251.value = (float) -1.664965E38F;
            p251.time_boot_ms = (uint)2236020060U;
            CommunicationChannel.instance.send(p251);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnNAMED_VALUE_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)2334601303U);
                Debug.Assert(pack.name_LEN(ph) == 4);
                Debug.Assert(pack.name_TRY(ph).Equals("dmJu"));
                Debug.Assert(pack.value == (int)349724205);
            };
            GroundControl.NAMED_VALUE_INT p252 = CommunicationChannel.new_NAMED_VALUE_INT();
            PH.setPack(p252);
            p252.name_SET("dmJu", PH) ;
            p252.value = (int)349724205;
            p252.time_boot_ms = (uint)2334601303U;
            CommunicationChannel.instance.send(p252);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSTATUSTEXTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.severity == MAV_SEVERITY.MAV_SEVERITY_INFO);
                Debug.Assert(pack.text_LEN(ph) == 48);
                Debug.Assert(pack.text_TRY(ph).Equals("vugjvtooxpjzuBDthlffonJcoDzkIvlypySifnkkiOfhrqiO"));
            };
            GroundControl.STATUSTEXT p253 = CommunicationChannel.new_STATUSTEXT();
            PH.setPack(p253);
            p253.severity = MAV_SEVERITY.MAV_SEVERITY_INFO;
            p253.text_SET("vugjvtooxpjzuBDthlffonJcoDzkIvlypySifnkkiOfhrqiO", PH) ;
            CommunicationChannel.instance.send(p253);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnDEBUGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ind == (byte)(byte)44);
                Debug.Assert(pack.value == (float) -1.2872983E37F);
                Debug.Assert(pack.time_boot_ms == (uint)4256969071U);
            };
            GroundControl.DEBUG p254 = CommunicationChannel.new_DEBUG();
            PH.setPack(p254);
            p254.time_boot_ms = (uint)4256969071U;
            p254.value = (float) -1.2872983E37F;
            p254.ind = (byte)(byte)44;
            CommunicationChannel.instance.send(p254);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSETUP_SIGNINGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.initial_timestamp == (ulong)1477956430925097564L);
                Debug.Assert(pack.target_component == (byte)(byte)54);
                Debug.Assert(pack.target_system == (byte)(byte)5);
                Debug.Assert(pack.secret_key.SequenceEqual(new byte[] {(byte)109, (byte)128, (byte)231, (byte)160, (byte)158, (byte)66, (byte)233, (byte)126, (byte)16, (byte)243, (byte)162, (byte)190, (byte)5, (byte)210, (byte)224, (byte)127, (byte)25, (byte)193, (byte)158, (byte)92, (byte)178, (byte)92, (byte)185, (byte)4, (byte)5, (byte)164, (byte)112, (byte)48, (byte)197, (byte)158, (byte)119, (byte)222}));
            };
            GroundControl.SETUP_SIGNING p256 = CommunicationChannel.new_SETUP_SIGNING();
            PH.setPack(p256);
            p256.initial_timestamp = (ulong)1477956430925097564L;
            p256.target_component = (byte)(byte)54;
            p256.secret_key_SET(new byte[] {(byte)109, (byte)128, (byte)231, (byte)160, (byte)158, (byte)66, (byte)233, (byte)126, (byte)16, (byte)243, (byte)162, (byte)190, (byte)5, (byte)210, (byte)224, (byte)127, (byte)25, (byte)193, (byte)158, (byte)92, (byte)178, (byte)92, (byte)185, (byte)4, (byte)5, (byte)164, (byte)112, (byte)48, (byte)197, (byte)158, (byte)119, (byte)222}, 0) ;
            p256.target_system = (byte)(byte)5;
            CommunicationChannel.instance.send(p256);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnBUTTON_CHANGEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)1253186604U);
                Debug.Assert(pack.last_change_ms == (uint)2363955918U);
                Debug.Assert(pack.state == (byte)(byte)249);
            };
            GroundControl.BUTTON_CHANGE p257 = CommunicationChannel.new_BUTTON_CHANGE();
            PH.setPack(p257);
            p257.state = (byte)(byte)249;
            p257.last_change_ms = (uint)2363955918U;
            p257.time_boot_ms = (uint)1253186604U;
            CommunicationChannel.instance.send(p257);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPLAY_TUNEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)171);
                Debug.Assert(pack.target_system == (byte)(byte)231);
                Debug.Assert(pack.tune_LEN(ph) == 13);
                Debug.Assert(pack.tune_TRY(ph).Equals("gvpdinXmtpfmu"));
            };
            GroundControl.PLAY_TUNE p258 = CommunicationChannel.new_PLAY_TUNE();
            PH.setPack(p258);
            p258.tune_SET("gvpdinXmtpfmu", PH) ;
            p258.target_system = (byte)(byte)231;
            p258.target_component = (byte)(byte)171;
            CommunicationChannel.instance.send(p258);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCAMERA_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sensor_size_v == (float)5.230528E37F);
                Debug.Assert(pack.flags == (CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_HAS_MODES |
                                            CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_VIDEO));
                Debug.Assert(pack.focal_length == (float) -2.5421729E38F);
                Debug.Assert(pack.resolution_h == (ushort)(ushort)33735);
                Debug.Assert(pack.cam_definition_version == (ushort)(ushort)7068);
                Debug.Assert(pack.vendor_name.SequenceEqual(new byte[] {(byte)247, (byte)143, (byte)156, (byte)230, (byte)3, (byte)40, (byte)54, (byte)249, (byte)25, (byte)32, (byte)227, (byte)109, (byte)48, (byte)128, (byte)141, (byte)75, (byte)197, (byte)193, (byte)133, (byte)176, (byte)13, (byte)148, (byte)192, (byte)102, (byte)97, (byte)85, (byte)239, (byte)38, (byte)213, (byte)109, (byte)75, (byte)50}));
                Debug.Assert(pack.sensor_size_h == (float) -5.417836E37F);
                Debug.Assert(pack.lens_id == (byte)(byte)103);
                Debug.Assert(pack.firmware_version == (uint)2885575404U);
                Debug.Assert(pack.time_boot_ms == (uint)2099189947U);
                Debug.Assert(pack.cam_definition_uri_LEN(ph) == 13);
                Debug.Assert(pack.cam_definition_uri_TRY(ph).Equals("wbnwmfimuonzg"));
                Debug.Assert(pack.model_name.SequenceEqual(new byte[] {(byte)9, (byte)47, (byte)97, (byte)180, (byte)163, (byte)201, (byte)199, (byte)76, (byte)85, (byte)103, (byte)226, (byte)44, (byte)3, (byte)25, (byte)177, (byte)104, (byte)151, (byte)165, (byte)132, (byte)227, (byte)53, (byte)247, (byte)223, (byte)180, (byte)252, (byte)16, (byte)63, (byte)232, (byte)213, (byte)102, (byte)13, (byte)208}));
                Debug.Assert(pack.resolution_v == (ushort)(ushort)46797);
            };
            GroundControl.CAMERA_INFORMATION p259 = CommunicationChannel.new_CAMERA_INFORMATION();
            PH.setPack(p259);
            p259.resolution_h = (ushort)(ushort)33735;
            p259.focal_length = (float) -2.5421729E38F;
            p259.cam_definition_uri_SET("wbnwmfimuonzg", PH) ;
            p259.vendor_name_SET(new byte[] {(byte)247, (byte)143, (byte)156, (byte)230, (byte)3, (byte)40, (byte)54, (byte)249, (byte)25, (byte)32, (byte)227, (byte)109, (byte)48, (byte)128, (byte)141, (byte)75, (byte)197, (byte)193, (byte)133, (byte)176, (byte)13, (byte)148, (byte)192, (byte)102, (byte)97, (byte)85, (byte)239, (byte)38, (byte)213, (byte)109, (byte)75, (byte)50}, 0) ;
            p259.resolution_v = (ushort)(ushort)46797;
            p259.sensor_size_v = (float)5.230528E37F;
            p259.firmware_version = (uint)2885575404U;
            p259.flags = (CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_HAS_MODES |
                          CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_VIDEO);
            p259.sensor_size_h = (float) -5.417836E37F;
            p259.model_name_SET(new byte[] {(byte)9, (byte)47, (byte)97, (byte)180, (byte)163, (byte)201, (byte)199, (byte)76, (byte)85, (byte)103, (byte)226, (byte)44, (byte)3, (byte)25, (byte)177, (byte)104, (byte)151, (byte)165, (byte)132, (byte)227, (byte)53, (byte)247, (byte)223, (byte)180, (byte)252, (byte)16, (byte)63, (byte)232, (byte)213, (byte)102, (byte)13, (byte)208}, 0) ;
            p259.cam_definition_version = (ushort)(ushort)7068;
            p259.lens_id = (byte)(byte)103;
            p259.time_boot_ms = (uint)2099189947U;
            CommunicationChannel.instance.send(p259);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCAMERA_SETTINGSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mode_id == (CAMERA_MODE.CAMERA_MODE_IMAGE |
                                              CAMERA_MODE.CAMERA_MODE_IMAGE_SURVEY));
                Debug.Assert(pack.time_boot_ms == (uint)853670183U);
            };
            GroundControl.CAMERA_SETTINGS p260 = CommunicationChannel.new_CAMERA_SETTINGS();
            PH.setPack(p260);
            p260.time_boot_ms = (uint)853670183U;
            p260.mode_id = (CAMERA_MODE.CAMERA_MODE_IMAGE |
                            CAMERA_MODE.CAMERA_MODE_IMAGE_SURVEY);
            CommunicationChannel.instance.send(p260);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSTORAGE_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.available_capacity == (float) -4.93624E37F);
                Debug.Assert(pack.storage_count == (byte)(byte)14);
                Debug.Assert(pack.used_capacity == (float) -6.2594624E37F);
                Debug.Assert(pack.time_boot_ms == (uint)347018478U);
                Debug.Assert(pack.storage_id == (byte)(byte)77);
                Debug.Assert(pack.status == (byte)(byte)130);
                Debug.Assert(pack.read_speed == (float)1.7833726E38F);
                Debug.Assert(pack.total_capacity == (float) -3.1876583E37F);
                Debug.Assert(pack.write_speed == (float)2.6108354E37F);
            };
            GroundControl.STORAGE_INFORMATION p261 = CommunicationChannel.new_STORAGE_INFORMATION();
            PH.setPack(p261);
            p261.used_capacity = (float) -6.2594624E37F;
            p261.total_capacity = (float) -3.1876583E37F;
            p261.write_speed = (float)2.6108354E37F;
            p261.available_capacity = (float) -4.93624E37F;
            p261.time_boot_ms = (uint)347018478U;
            p261.status = (byte)(byte)130;
            p261.read_speed = (float)1.7833726E38F;
            p261.storage_id = (byte)(byte)77;
            p261.storage_count = (byte)(byte)14;
            CommunicationChannel.instance.send(p261);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCAMERA_CAPTURE_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)3750302855U);
                Debug.Assert(pack.image_interval == (float) -1.0941286E38F);
                Debug.Assert(pack.image_status == (byte)(byte)173);
                Debug.Assert(pack.available_capacity == (float)2.1649671E38F);
                Debug.Assert(pack.video_status == (byte)(byte)213);
                Debug.Assert(pack.recording_time_ms == (uint)2953434366U);
            };
            GroundControl.CAMERA_CAPTURE_STATUS p262 = CommunicationChannel.new_CAMERA_CAPTURE_STATUS();
            PH.setPack(p262);
            p262.time_boot_ms = (uint)3750302855U;
            p262.available_capacity = (float)2.1649671E38F;
            p262.recording_time_ms = (uint)2953434366U;
            p262.image_status = (byte)(byte)173;
            p262.video_status = (byte)(byte)213;
            p262.image_interval = (float) -1.0941286E38F;
            CommunicationChannel.instance.send(p262);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCAMERA_IMAGE_CAPTUREDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lon == (int)1306424045);
                Debug.Assert(pack.lat == (int) -975074640);
                Debug.Assert(pack.relative_alt == (int) -608757355);
                Debug.Assert(pack.alt == (int) -1877396262);
                Debug.Assert(pack.capture_result == (sbyte)(sbyte) - 75);
                Debug.Assert(pack.time_boot_ms == (uint)3153129890U);
                Debug.Assert(pack.image_index == (int)1904251495);
                Debug.Assert(pack.file_url_LEN(ph) == 114);
                Debug.Assert(pack.file_url_TRY(ph).Equals("dqKVnbycwzuuklegclgdYAvrrkcjhlnytqtnMcxpxdxdzbhyuymwazbfyLkjmrgszrktuyensevbtDpvrnrxYaxwmdjrcusznDcnkzursgrwYotrgf"));
                Debug.Assert(pack.time_utc == (ulong)1192944105467787476L);
                Debug.Assert(pack.q.SequenceEqual(new float[] {2.472684E38F, 5.346991E37F, -7.6121506E37F, -2.6757772E38F}));
                Debug.Assert(pack.camera_id == (byte)(byte)161);
            };
            GroundControl.CAMERA_IMAGE_CAPTURED p263 = CommunicationChannel.new_CAMERA_IMAGE_CAPTURED();
            PH.setPack(p263);
            p263.alt = (int) -1877396262;
            p263.lon = (int)1306424045;
            p263.camera_id = (byte)(byte)161;
            p263.relative_alt = (int) -608757355;
            p263.time_boot_ms = (uint)3153129890U;
            p263.file_url_SET("dqKVnbycwzuuklegclgdYAvrrkcjhlnytqtnMcxpxdxdzbhyuymwazbfyLkjmrgszrktuyensevbtDpvrnrxYaxwmdjrcusznDcnkzursgrwYotrgf", PH) ;
            p263.time_utc = (ulong)1192944105467787476L;
            p263.q_SET(new float[] {2.472684E38F, 5.346991E37F, -7.6121506E37F, -2.6757772E38F}, 0) ;
            p263.lat = (int) -975074640;
            p263.image_index = (int)1904251495;
            p263.capture_result = (sbyte)(sbyte) - 75;
            CommunicationChannel.instance.send(p263);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnFLIGHT_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)1959525586U);
                Debug.Assert(pack.arming_time_utc == (ulong)5950621428468839213L);
                Debug.Assert(pack.takeoff_time_utc == (ulong)2292820824543552457L);
                Debug.Assert(pack.flight_uuid == (ulong)4044106653557364166L);
            };
            GroundControl.FLIGHT_INFORMATION p264 = CommunicationChannel.new_FLIGHT_INFORMATION();
            PH.setPack(p264);
            p264.flight_uuid = (ulong)4044106653557364166L;
            p264.time_boot_ms = (uint)1959525586U;
            p264.arming_time_utc = (ulong)5950621428468839213L;
            p264.takeoff_time_utc = (ulong)2292820824543552457L;
            CommunicationChannel.instance.send(p264);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            ADV_TEST_CH.OnMOUNT_ORIENTATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)1135120484U);
                Debug.Assert(pack.yaw == (float) -5.086356E37F);
                Debug.Assert(pack.roll == (float) -5.74179E37F);
                Debug.Assert(pack.pitch == (float)2.8308145E38F);
            };
            GroundControl.MOUNT_ORIENTATION p265 = CommunicationChannel.new_MOUNT_ORIENTATION();
            PH.setPack(p265);
            p265.time_boot_ms = (uint)1135120484U;
            p265.pitch = (float)2.8308145E38F;
            p265.roll = (float) -5.74179E37F;
            p265.yaw = (float) -5.086356E37F;
            CommunicationChannel.instance.send(p265);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnLOGGING_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)201, (byte)234, (byte)56, (byte)46, (byte)19, (byte)233, (byte)213, (byte)217, (byte)224, (byte)142, (byte)66, (byte)34, (byte)39, (byte)36, (byte)109, (byte)90, (byte)101, (byte)182, (byte)190, (byte)211, (byte)200, (byte)122, (byte)29, (byte)145, (byte)219, (byte)196, (byte)177, (byte)207, (byte)236, (byte)4, (byte)27, (byte)15, (byte)190, (byte)194, (byte)193, (byte)108, (byte)225, (byte)45, (byte)64, (byte)152, (byte)41, (byte)23, (byte)238, (byte)82, (byte)133, (byte)90, (byte)157, (byte)213, (byte)121, (byte)93, (byte)248, (byte)153, (byte)112, (byte)68, (byte)243, (byte)18, (byte)178, (byte)109, (byte)126, (byte)128, (byte)203, (byte)191, (byte)219, (byte)243, (byte)236, (byte)76, (byte)246, (byte)218, (byte)243, (byte)73, (byte)1, (byte)199, (byte)14, (byte)38, (byte)224, (byte)167, (byte)0, (byte)16, (byte)208, (byte)55, (byte)252, (byte)198, (byte)15, (byte)119, (byte)184, (byte)68, (byte)1, (byte)200, (byte)73, (byte)40, (byte)141, (byte)163, (byte)163, (byte)241, (byte)46, (byte)113, (byte)199, (byte)126, (byte)192, (byte)227, (byte)6, (byte)4, (byte)73, (byte)74, (byte)47, (byte)248, (byte)46, (byte)142, (byte)111, (byte)229, (byte)4, (byte)212, (byte)104, (byte)133, (byte)95, (byte)129, (byte)232, (byte)64, (byte)196, (byte)65, (byte)126, (byte)248, (byte)171, (byte)44, (byte)244, (byte)157, (byte)246, (byte)92, (byte)179, (byte)150, (byte)5, (byte)227, (byte)45, (byte)214, (byte)209, (byte)101, (byte)231, (byte)208, (byte)192, (byte)176, (byte)156, (byte)98, (byte)122, (byte)161, (byte)95, (byte)173, (byte)54, (byte)81, (byte)4, (byte)13, (byte)142, (byte)233, (byte)102, (byte)0, (byte)163, (byte)140, (byte)231, (byte)56, (byte)143, (byte)149, (byte)96, (byte)68, (byte)27, (byte)109, (byte)139, (byte)38, (byte)9, (byte)254, (byte)206, (byte)142, (byte)111, (byte)240, (byte)172, (byte)149, (byte)215, (byte)239, (byte)145, (byte)96, (byte)155, (byte)181, (byte)6, (byte)149, (byte)108, (byte)177, (byte)195, (byte)41, (byte)152, (byte)133, (byte)195, (byte)65, (byte)127, (byte)99, (byte)175, (byte)190, (byte)197, (byte)187, (byte)167, (byte)36, (byte)189, (byte)110, (byte)180, (byte)193, (byte)72, (byte)249, (byte)129, (byte)92, (byte)45, (byte)79, (byte)47, (byte)123, (byte)146, (byte)165, (byte)134, (byte)158, (byte)224, (byte)143, (byte)204, (byte)80, (byte)190, (byte)200, (byte)207, (byte)57, (byte)173, (byte)12, (byte)158, (byte)64, (byte)145, (byte)185, (byte)79, (byte)133, (byte)135, (byte)37, (byte)214, (byte)17, (byte)167, (byte)32, (byte)140, (byte)27, (byte)132, (byte)237, (byte)219, (byte)159, (byte)249, (byte)206, (byte)64, (byte)65, (byte)55, (byte)175, (byte)34}));
                Debug.Assert(pack.length == (byte)(byte)224);
                Debug.Assert(pack.target_component == (byte)(byte)57);
                Debug.Assert(pack.target_system == (byte)(byte)165);
                Debug.Assert(pack.first_message_offset == (byte)(byte)62);
                Debug.Assert(pack.sequence == (ushort)(ushort)7378);
            };
            GroundControl.LOGGING_DATA p266 = CommunicationChannel.new_LOGGING_DATA();
            PH.setPack(p266);
            p266.target_component = (byte)(byte)57;
            p266.sequence = (ushort)(ushort)7378;
            p266.target_system = (byte)(byte)165;
            p266.data__SET(new byte[] {(byte)201, (byte)234, (byte)56, (byte)46, (byte)19, (byte)233, (byte)213, (byte)217, (byte)224, (byte)142, (byte)66, (byte)34, (byte)39, (byte)36, (byte)109, (byte)90, (byte)101, (byte)182, (byte)190, (byte)211, (byte)200, (byte)122, (byte)29, (byte)145, (byte)219, (byte)196, (byte)177, (byte)207, (byte)236, (byte)4, (byte)27, (byte)15, (byte)190, (byte)194, (byte)193, (byte)108, (byte)225, (byte)45, (byte)64, (byte)152, (byte)41, (byte)23, (byte)238, (byte)82, (byte)133, (byte)90, (byte)157, (byte)213, (byte)121, (byte)93, (byte)248, (byte)153, (byte)112, (byte)68, (byte)243, (byte)18, (byte)178, (byte)109, (byte)126, (byte)128, (byte)203, (byte)191, (byte)219, (byte)243, (byte)236, (byte)76, (byte)246, (byte)218, (byte)243, (byte)73, (byte)1, (byte)199, (byte)14, (byte)38, (byte)224, (byte)167, (byte)0, (byte)16, (byte)208, (byte)55, (byte)252, (byte)198, (byte)15, (byte)119, (byte)184, (byte)68, (byte)1, (byte)200, (byte)73, (byte)40, (byte)141, (byte)163, (byte)163, (byte)241, (byte)46, (byte)113, (byte)199, (byte)126, (byte)192, (byte)227, (byte)6, (byte)4, (byte)73, (byte)74, (byte)47, (byte)248, (byte)46, (byte)142, (byte)111, (byte)229, (byte)4, (byte)212, (byte)104, (byte)133, (byte)95, (byte)129, (byte)232, (byte)64, (byte)196, (byte)65, (byte)126, (byte)248, (byte)171, (byte)44, (byte)244, (byte)157, (byte)246, (byte)92, (byte)179, (byte)150, (byte)5, (byte)227, (byte)45, (byte)214, (byte)209, (byte)101, (byte)231, (byte)208, (byte)192, (byte)176, (byte)156, (byte)98, (byte)122, (byte)161, (byte)95, (byte)173, (byte)54, (byte)81, (byte)4, (byte)13, (byte)142, (byte)233, (byte)102, (byte)0, (byte)163, (byte)140, (byte)231, (byte)56, (byte)143, (byte)149, (byte)96, (byte)68, (byte)27, (byte)109, (byte)139, (byte)38, (byte)9, (byte)254, (byte)206, (byte)142, (byte)111, (byte)240, (byte)172, (byte)149, (byte)215, (byte)239, (byte)145, (byte)96, (byte)155, (byte)181, (byte)6, (byte)149, (byte)108, (byte)177, (byte)195, (byte)41, (byte)152, (byte)133, (byte)195, (byte)65, (byte)127, (byte)99, (byte)175, (byte)190, (byte)197, (byte)187, (byte)167, (byte)36, (byte)189, (byte)110, (byte)180, (byte)193, (byte)72, (byte)249, (byte)129, (byte)92, (byte)45, (byte)79, (byte)47, (byte)123, (byte)146, (byte)165, (byte)134, (byte)158, (byte)224, (byte)143, (byte)204, (byte)80, (byte)190, (byte)200, (byte)207, (byte)57, (byte)173, (byte)12, (byte)158, (byte)64, (byte)145, (byte)185, (byte)79, (byte)133, (byte)135, (byte)37, (byte)214, (byte)17, (byte)167, (byte)32, (byte)140, (byte)27, (byte)132, (byte)237, (byte)219, (byte)159, (byte)249, (byte)206, (byte)64, (byte)65, (byte)55, (byte)175, (byte)34}, 0) ;
            p266.first_message_offset = (byte)(byte)62;
            p266.length = (byte)(byte)224;
            CommunicationChannel.instance.send(p266);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnLOGGING_DATA_ACKEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.first_message_offset == (byte)(byte)34);
                Debug.Assert(pack.sequence == (ushort)(ushort)29864);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)114, (byte)159, (byte)18, (byte)93, (byte)107, (byte)242, (byte)105, (byte)168, (byte)190, (byte)214, (byte)94, (byte)140, (byte)87, (byte)122, (byte)246, (byte)35, (byte)82, (byte)219, (byte)3, (byte)115, (byte)40, (byte)135, (byte)108, (byte)11, (byte)46, (byte)242, (byte)193, (byte)224, (byte)134, (byte)233, (byte)79, (byte)130, (byte)104, (byte)30, (byte)185, (byte)42, (byte)37, (byte)160, (byte)160, (byte)237, (byte)132, (byte)158, (byte)73, (byte)40, (byte)66, (byte)204, (byte)105, (byte)116, (byte)46, (byte)219, (byte)138, (byte)43, (byte)70, (byte)67, (byte)23, (byte)150, (byte)35, (byte)198, (byte)27, (byte)7, (byte)250, (byte)99, (byte)200, (byte)50, (byte)216, (byte)126, (byte)120, (byte)42, (byte)144, (byte)159, (byte)185, (byte)224, (byte)156, (byte)183, (byte)91, (byte)54, (byte)181, (byte)105, (byte)206, (byte)62, (byte)30, (byte)211, (byte)10, (byte)138, (byte)11, (byte)221, (byte)236, (byte)60, (byte)31, (byte)13, (byte)75, (byte)171, (byte)137, (byte)96, (byte)75, (byte)211, (byte)217, (byte)86, (byte)69, (byte)197, (byte)82, (byte)220, (byte)215, (byte)70, (byte)179, (byte)206, (byte)42, (byte)114, (byte)104, (byte)2, (byte)32, (byte)82, (byte)87, (byte)5, (byte)137, (byte)76, (byte)37, (byte)80, (byte)187, (byte)99, (byte)42, (byte)204, (byte)192, (byte)0, (byte)236, (byte)26, (byte)198, (byte)23, (byte)101, (byte)173, (byte)243, (byte)93, (byte)78, (byte)119, (byte)146, (byte)89, (byte)152, (byte)158, (byte)88, (byte)100, (byte)8, (byte)161, (byte)59, (byte)165, (byte)167, (byte)242, (byte)130, (byte)165, (byte)148, (byte)174, (byte)35, (byte)175, (byte)196, (byte)155, (byte)219, (byte)250, (byte)235, (byte)40, (byte)20, (byte)186, (byte)203, (byte)172, (byte)119, (byte)176, (byte)65, (byte)3, (byte)236, (byte)242, (byte)2, (byte)149, (byte)205, (byte)54, (byte)136, (byte)210, (byte)83, (byte)201, (byte)91, (byte)249, (byte)232, (byte)3, (byte)157, (byte)8, (byte)40, (byte)205, (byte)222, (byte)162, (byte)254, (byte)203, (byte)64, (byte)187, (byte)55, (byte)16, (byte)2, (byte)113, (byte)40, (byte)248, (byte)153, (byte)12, (byte)203, (byte)111, (byte)84, (byte)177, (byte)109, (byte)43, (byte)211, (byte)245, (byte)152, (byte)124, (byte)64, (byte)193, (byte)237, (byte)128, (byte)254, (byte)74, (byte)166, (byte)198, (byte)217, (byte)0, (byte)83, (byte)129, (byte)187, (byte)89, (byte)92, (byte)215, (byte)62, (byte)197, (byte)97, (byte)87, (byte)111, (byte)230, (byte)63, (byte)100, (byte)50, (byte)106, (byte)34, (byte)140, (byte)218, (byte)243, (byte)86, (byte)37, (byte)64, (byte)233, (byte)19, (byte)197, (byte)221, (byte)35, (byte)220, (byte)184, (byte)13}));
                Debug.Assert(pack.length == (byte)(byte)88);
                Debug.Assert(pack.target_component == (byte)(byte)211);
                Debug.Assert(pack.target_system == (byte)(byte)24);
            };
            GroundControl.LOGGING_DATA_ACKED p267 = CommunicationChannel.new_LOGGING_DATA_ACKED();
            PH.setPack(p267);
            p267.target_system = (byte)(byte)24;
            p267.first_message_offset = (byte)(byte)34;
            p267.data__SET(new byte[] {(byte)114, (byte)159, (byte)18, (byte)93, (byte)107, (byte)242, (byte)105, (byte)168, (byte)190, (byte)214, (byte)94, (byte)140, (byte)87, (byte)122, (byte)246, (byte)35, (byte)82, (byte)219, (byte)3, (byte)115, (byte)40, (byte)135, (byte)108, (byte)11, (byte)46, (byte)242, (byte)193, (byte)224, (byte)134, (byte)233, (byte)79, (byte)130, (byte)104, (byte)30, (byte)185, (byte)42, (byte)37, (byte)160, (byte)160, (byte)237, (byte)132, (byte)158, (byte)73, (byte)40, (byte)66, (byte)204, (byte)105, (byte)116, (byte)46, (byte)219, (byte)138, (byte)43, (byte)70, (byte)67, (byte)23, (byte)150, (byte)35, (byte)198, (byte)27, (byte)7, (byte)250, (byte)99, (byte)200, (byte)50, (byte)216, (byte)126, (byte)120, (byte)42, (byte)144, (byte)159, (byte)185, (byte)224, (byte)156, (byte)183, (byte)91, (byte)54, (byte)181, (byte)105, (byte)206, (byte)62, (byte)30, (byte)211, (byte)10, (byte)138, (byte)11, (byte)221, (byte)236, (byte)60, (byte)31, (byte)13, (byte)75, (byte)171, (byte)137, (byte)96, (byte)75, (byte)211, (byte)217, (byte)86, (byte)69, (byte)197, (byte)82, (byte)220, (byte)215, (byte)70, (byte)179, (byte)206, (byte)42, (byte)114, (byte)104, (byte)2, (byte)32, (byte)82, (byte)87, (byte)5, (byte)137, (byte)76, (byte)37, (byte)80, (byte)187, (byte)99, (byte)42, (byte)204, (byte)192, (byte)0, (byte)236, (byte)26, (byte)198, (byte)23, (byte)101, (byte)173, (byte)243, (byte)93, (byte)78, (byte)119, (byte)146, (byte)89, (byte)152, (byte)158, (byte)88, (byte)100, (byte)8, (byte)161, (byte)59, (byte)165, (byte)167, (byte)242, (byte)130, (byte)165, (byte)148, (byte)174, (byte)35, (byte)175, (byte)196, (byte)155, (byte)219, (byte)250, (byte)235, (byte)40, (byte)20, (byte)186, (byte)203, (byte)172, (byte)119, (byte)176, (byte)65, (byte)3, (byte)236, (byte)242, (byte)2, (byte)149, (byte)205, (byte)54, (byte)136, (byte)210, (byte)83, (byte)201, (byte)91, (byte)249, (byte)232, (byte)3, (byte)157, (byte)8, (byte)40, (byte)205, (byte)222, (byte)162, (byte)254, (byte)203, (byte)64, (byte)187, (byte)55, (byte)16, (byte)2, (byte)113, (byte)40, (byte)248, (byte)153, (byte)12, (byte)203, (byte)111, (byte)84, (byte)177, (byte)109, (byte)43, (byte)211, (byte)245, (byte)152, (byte)124, (byte)64, (byte)193, (byte)237, (byte)128, (byte)254, (byte)74, (byte)166, (byte)198, (byte)217, (byte)0, (byte)83, (byte)129, (byte)187, (byte)89, (byte)92, (byte)215, (byte)62, (byte)197, (byte)97, (byte)87, (byte)111, (byte)230, (byte)63, (byte)100, (byte)50, (byte)106, (byte)34, (byte)140, (byte)218, (byte)243, (byte)86, (byte)37, (byte)64, (byte)233, (byte)19, (byte)197, (byte)221, (byte)35, (byte)220, (byte)184, (byte)13}, 0) ;
            p267.length = (byte)(byte)88;
            p267.target_component = (byte)(byte)211;
            p267.sequence = (ushort)(ushort)29864;
            CommunicationChannel.instance.send(p267);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnLOGGING_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)3);
                Debug.Assert(pack.target_system == (byte)(byte)38);
                Debug.Assert(pack.sequence == (ushort)(ushort)30547);
            };
            GroundControl.LOGGING_ACK p268 = CommunicationChannel.new_LOGGING_ACK();
            PH.setPack(p268);
            p268.target_component = (byte)(byte)3;
            p268.target_system = (byte)(byte)38;
            p268.sequence = (ushort)(ushort)30547;
            CommunicationChannel.instance.send(p268);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnVIDEO_STREAM_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.status == (byte)(byte)123);
                Debug.Assert(pack.rotation == (ushort)(ushort)38550);
                Debug.Assert(pack.resolution_h == (ushort)(ushort)13903);
                Debug.Assert(pack.uri_LEN(ph) == 206);
                Debug.Assert(pack.uri_TRY(ph).Equals("ondtmibnqxgbKidyouxaoqhWdoCwwnIbnzkjpojEyluhkrgGejozjNvrpbuRgeAphtwcLaOgsvcrtxdazuwbovypehwsxseZlhdsTafegokiblqrixnwiokqrgbpuyAanqvgzasjzmnmwxlvnmfbnniaqqOcimbwqkxmvFWUpdlmxmAxAwEQafLcfadkcylLhxrcuqzreadmgT"));
                Debug.Assert(pack.resolution_v == (ushort)(ushort)29634);
                Debug.Assert(pack.bitrate == (uint)445322940U);
                Debug.Assert(pack.camera_id == (byte)(byte)120);
                Debug.Assert(pack.framerate == (float) -3.2516067E38F);
            };
            GroundControl.VIDEO_STREAM_INFORMATION p269 = CommunicationChannel.new_VIDEO_STREAM_INFORMATION();
            PH.setPack(p269);
            p269.bitrate = (uint)445322940U;
            p269.uri_SET("ondtmibnqxgbKidyouxaoqhWdoCwwnIbnzkjpojEyluhkrgGejozjNvrpbuRgeAphtwcLaOgsvcrtxdazuwbovypehwsxseZlhdsTafegokiblqrixnwiokqrgbpuyAanqvgzasjzmnmwxlvnmfbnniaqqOcimbwqkxmvFWUpdlmxmAxAwEQafLcfadkcylLhxrcuqzreadmgT", PH) ;
            p269.rotation = (ushort)(ushort)38550;
            p269.camera_id = (byte)(byte)120;
            p269.resolution_h = (ushort)(ushort)13903;
            p269.status = (byte)(byte)123;
            p269.framerate = (float) -3.2516067E38F;
            p269.resolution_v = (ushort)(ushort)29634;
            CommunicationChannel.instance.send(p269);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnSET_VIDEO_STREAM_SETTINGSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)90);
                Debug.Assert(pack.resolution_h == (ushort)(ushort)13408);
                Debug.Assert(pack.bitrate == (uint)2807990474U);
                Debug.Assert(pack.camera_id == (byte)(byte)26);
                Debug.Assert(pack.rotation == (ushort)(ushort)50014);
                Debug.Assert(pack.uri_LEN(ph) == 3);
                Debug.Assert(pack.uri_TRY(ph).Equals("Vqi"));
                Debug.Assert(pack.resolution_v == (ushort)(ushort)19539);
                Debug.Assert(pack.target_component == (byte)(byte)46);
                Debug.Assert(pack.framerate == (float) -2.5143184E38F);
            };
            GroundControl.SET_VIDEO_STREAM_SETTINGS p270 = CommunicationChannel.new_SET_VIDEO_STREAM_SETTINGS();
            PH.setPack(p270);
            p270.resolution_v = (ushort)(ushort)19539;
            p270.camera_id = (byte)(byte)26;
            p270.rotation = (ushort)(ushort)50014;
            p270.target_component = (byte)(byte)46;
            p270.framerate = (float) -2.5143184E38F;
            p270.target_system = (byte)(byte)90;
            p270.bitrate = (uint)2807990474U;
            p270.resolution_h = (ushort)(ushort)13408;
            p270.uri_SET("Vqi", PH) ;
            CommunicationChannel.instance.send(p270);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnWIFI_CONFIG_APReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.password_LEN(ph) == 22);
                Debug.Assert(pack.password_TRY(ph).Equals("mgafxtwkuvwbteovvjTzly"));
                Debug.Assert(pack.ssid_LEN(ph) == 25);
                Debug.Assert(pack.ssid_TRY(ph).Equals("esQagdzfecmcgcaxkpStlacvm"));
            };
            GroundControl.WIFI_CONFIG_AP p299 = CommunicationChannel.new_WIFI_CONFIG_AP();
            PH.setPack(p299);
            p299.password_SET("mgafxtwkuvwbteovvjTzly", PH) ;
            p299.ssid_SET("esQagdzfecmcgcaxkpStlacvm", PH) ;
            CommunicationChannel.instance.send(p299);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnPROTOCOL_VERSIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.min_version == (ushort)(ushort)60065);
                Debug.Assert(pack.version == (ushort)(ushort)22685);
                Debug.Assert(pack.library_version_hash.SequenceEqual(new byte[] {(byte)159, (byte)91, (byte)34, (byte)23, (byte)175, (byte)187, (byte)220, (byte)242}));
                Debug.Assert(pack.max_version == (ushort)(ushort)40437);
                Debug.Assert(pack.spec_version_hash.SequenceEqual(new byte[] {(byte)33, (byte)179, (byte)178, (byte)197, (byte)49, (byte)38, (byte)197, (byte)195}));
            };
            GroundControl.PROTOCOL_VERSION p300 = CommunicationChannel.new_PROTOCOL_VERSION();
            PH.setPack(p300);
            p300.spec_version_hash_SET(new byte[] {(byte)33, (byte)179, (byte)178, (byte)197, (byte)49, (byte)38, (byte)197, (byte)195}, 0) ;
            p300.max_version = (ushort)(ushort)40437;
            p300.version = (ushort)(ushort)22685;
            p300.min_version = (ushort)(ushort)60065;
            p300.library_version_hash_SET(new byte[] {(byte)159, (byte)91, (byte)34, (byte)23, (byte)175, (byte)187, (byte)220, (byte)242}, 0) ;
            CommunicationChannel.instance.send(p300);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnUAVCAN_NODE_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.uptime_sec == (uint)3139804668U);
                Debug.Assert(pack.health == UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_ERROR);
                Debug.Assert(pack.vendor_specific_status_code == (ushort)(ushort)48293);
                Debug.Assert(pack.time_usec == (ulong)6914505159702156346L);
                Debug.Assert(pack.mode == UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_OPERATIONAL);
                Debug.Assert(pack.sub_mode == (byte)(byte)203);
            };
            GroundControl.UAVCAN_NODE_STATUS p310 = CommunicationChannel.new_UAVCAN_NODE_STATUS();
            PH.setPack(p310);
            p310.sub_mode = (byte)(byte)203;
            p310.mode = UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_OPERATIONAL;
            p310.time_usec = (ulong)6914505159702156346L;
            p310.uptime_sec = (uint)3139804668U;
            p310.vendor_specific_status_code = (ushort)(ushort)48293;
            p310.health = UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_ERROR;
            CommunicationChannel.instance.send(p310);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnUAVCAN_NODE_INFOReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sw_version_minor == (byte)(byte)121);
                Debug.Assert(pack.name_LEN(ph) == 13);
                Debug.Assert(pack.name_TRY(ph).Equals("fiZnlcckupvnt"));
                Debug.Assert(pack.uptime_sec == (uint)3167994307U);
                Debug.Assert(pack.time_usec == (ulong)4986575982472890805L);
                Debug.Assert(pack.hw_unique_id.SequenceEqual(new byte[] {(byte)50, (byte)61, (byte)200, (byte)53, (byte)32, (byte)69, (byte)246, (byte)80, (byte)216, (byte)92, (byte)33, (byte)4, (byte)103, (byte)102, (byte)205, (byte)72}));
                Debug.Assert(pack.sw_version_major == (byte)(byte)90);
                Debug.Assert(pack.hw_version_major == (byte)(byte)12);
                Debug.Assert(pack.sw_vcs_commit == (uint)1950404269U);
                Debug.Assert(pack.hw_version_minor == (byte)(byte)198);
            };
            GroundControl.UAVCAN_NODE_INFO p311 = CommunicationChannel.new_UAVCAN_NODE_INFO();
            PH.setPack(p311);
            p311.sw_vcs_commit = (uint)1950404269U;
            p311.hw_version_major = (byte)(byte)12;
            p311.name_SET("fiZnlcckupvnt", PH) ;
            p311.time_usec = (ulong)4986575982472890805L;
            p311.hw_unique_id_SET(new byte[] {(byte)50, (byte)61, (byte)200, (byte)53, (byte)32, (byte)69, (byte)246, (byte)80, (byte)216, (byte)92, (byte)33, (byte)4, (byte)103, (byte)102, (byte)205, (byte)72}, 0) ;
            p311.sw_version_minor = (byte)(byte)121;
            p311.sw_version_major = (byte)(byte)90;
            p311.uptime_sec = (uint)3167994307U;
            p311.hw_version_minor = (byte)(byte)198;
            CommunicationChannel.instance.send(p311);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_REQUEST_READReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_index == (short)(short)369);
                Debug.Assert(pack.target_component == (byte)(byte)142);
                Debug.Assert(pack.param_id_LEN(ph) == 1);
                Debug.Assert(pack.param_id_TRY(ph).Equals("W"));
                Debug.Assert(pack.target_system == (byte)(byte)105);
            };
            GroundControl.PARAM_EXT_REQUEST_READ p320 = CommunicationChannel.new_PARAM_EXT_REQUEST_READ();
            PH.setPack(p320);
            p320.target_component = (byte)(byte)142;
            p320.target_system = (byte)(byte)105;
            p320.param_id_SET("W", PH) ;
            p320.param_index = (short)(short)369;
            CommunicationChannel.instance.send(p320);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)94);
                Debug.Assert(pack.target_component == (byte)(byte)228);
            };
            GroundControl.PARAM_EXT_REQUEST_LIST p321 = CommunicationChannel.new_PARAM_EXT_REQUEST_LIST();
            PH.setPack(p321);
            p321.target_system = (byte)(byte)94;
            p321.target_component = (byte)(byte)228;
            CommunicationChannel.instance.send(p321);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_VALUEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_count == (ushort)(ushort)39203);
                Debug.Assert(pack.param_id_LEN(ph) == 6);
                Debug.Assert(pack.param_id_TRY(ph).Equals("sbztZi"));
                Debug.Assert(pack.param_value_LEN(ph) == 2);
                Debug.Assert(pack.param_value_TRY(ph).Equals("ft"));
                Debug.Assert(pack.param_index == (ushort)(ushort)48076);
                Debug.Assert(pack.param_type == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT64);
            };
            GroundControl.PARAM_EXT_VALUE p322 = CommunicationChannel.new_PARAM_EXT_VALUE();
            PH.setPack(p322);
            p322.param_type = MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT64;
            p322.param_value_SET("ft", PH) ;
            p322.param_id_SET("sbztZi", PH) ;
            p322.param_index = (ushort)(ushort)48076;
            p322.param_count = (ushort)(ushort)39203;
            CommunicationChannel.instance.send(p322);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_SETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_id_LEN(ph) == 8);
                Debug.Assert(pack.param_id_TRY(ph).Equals("vuZigtkx"));
                Debug.Assert(pack.target_component == (byte)(byte)212);
                Debug.Assert(pack.param_value_LEN(ph) == 103);
                Debug.Assert(pack.param_value_TRY(ph).Equals("hxvwdzqwGsxzakfgmiujkbxewiOcjyeecJkmjQonQvdfkoJsszkueoPnwoayofrhcleuypfcbkaaeyuiuwlezktFlkcxAqhLoOklfwn"));
                Debug.Assert(pack.param_type == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT8);
                Debug.Assert(pack.target_system == (byte)(byte)83);
            };
            GroundControl.PARAM_EXT_SET p323 = CommunicationChannel.new_PARAM_EXT_SET();
            PH.setPack(p323);
            p323.target_component = (byte)(byte)212;
            p323.param_value_SET("hxvwdzqwGsxzakfgmiujkbxewiOcjyeecJkmjQonQvdfkoJsszkueoPnwoayofrhcleuypfcbkaaeyuiuwlezktFlkcxAqhLoOklfwn", PH) ;
            p323.target_system = (byte)(byte)83;
            p323.param_id_SET("vuZigtkx", PH) ;
            p323.param_type = MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT8;
            CommunicationChannel.instance.send(p323);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_result == PARAM_ACK.PARAM_ACK_IN_PROGRESS);
                Debug.Assert(pack.param_type == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT32);
                Debug.Assert(pack.param_id_LEN(ph) == 7);
                Debug.Assert(pack.param_id_TRY(ph).Equals("allljnv"));
                Debug.Assert(pack.param_value_LEN(ph) == 104);
                Debug.Assert(pack.param_value_TRY(ph).Equals("XrcmtstGaswjxpptegWxtkFfrxzfAyBakexkikgcqcomvltsdbaybfwfcITEdxomtguOzeiwqyrsuuwaDsjoayHjopuqsnudutxzemhq"));
            };
            GroundControl.PARAM_EXT_ACK p324 = CommunicationChannel.new_PARAM_EXT_ACK();
            PH.setPack(p324);
            p324.param_value_SET("XrcmtstGaswjxpptegWxtkFfrxzfAyBakexkikgcqcomvltsdbaybfwfcITEdxomtguOzeiwqyrsuuwaDsjoayHjopuqsnudutxzemhq", PH) ;
            p324.param_type = MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT32;
            p324.param_result = PARAM_ACK.PARAM_ACK_IN_PROGRESS;
            p324.param_id_SET("allljnv", PH) ;
            CommunicationChannel.instance.send(p324);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnOBSTACLE_DISTANCEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sensor_type == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_ULTRASOUND);
                Debug.Assert(pack.min_distance == (ushort)(ushort)3568);
                Debug.Assert(pack.increment == (byte)(byte)132);
                Debug.Assert(pack.time_usec == (ulong)2473180393616130214L);
                Debug.Assert(pack.max_distance == (ushort)(ushort)60301);
                Debug.Assert(pack.distances.SequenceEqual(new ushort[] {(ushort)55213, (ushort)3486, (ushort)34292, (ushort)34468, (ushort)34151, (ushort)19814, (ushort)36047, (ushort)31631, (ushort)28012, (ushort)663, (ushort)58695, (ushort)14853, (ushort)24791, (ushort)14835, (ushort)26122, (ushort)58410, (ushort)20739, (ushort)26389, (ushort)24323, (ushort)38204, (ushort)63717, (ushort)5166, (ushort)29729, (ushort)4878, (ushort)33854, (ushort)26726, (ushort)34184, (ushort)24510, (ushort)52764, (ushort)7182, (ushort)31086, (ushort)9304, (ushort)3634, (ushort)41312, (ushort)15551, (ushort)21612, (ushort)41681, (ushort)27003, (ushort)11735, (ushort)53362, (ushort)3709, (ushort)19712, (ushort)65139, (ushort)56266, (ushort)19922, (ushort)47186, (ushort)10687, (ushort)41891, (ushort)37091, (ushort)52307, (ushort)46856, (ushort)10924, (ushort)3828, (ushort)37545, (ushort)23672, (ushort)55554, (ushort)29466, (ushort)62925, (ushort)65463, (ushort)63509, (ushort)17104, (ushort)60501, (ushort)59105, (ushort)56377, (ushort)5861, (ushort)41290, (ushort)50071, (ushort)36382, (ushort)11548, (ushort)14010, (ushort)59271, (ushort)11456}));
            };
            GroundControl.OBSTACLE_DISTANCE p330 = CommunicationChannel.new_OBSTACLE_DISTANCE();
            PH.setPack(p330);
            p330.distances_SET(new ushort[] {(ushort)55213, (ushort)3486, (ushort)34292, (ushort)34468, (ushort)34151, (ushort)19814, (ushort)36047, (ushort)31631, (ushort)28012, (ushort)663, (ushort)58695, (ushort)14853, (ushort)24791, (ushort)14835, (ushort)26122, (ushort)58410, (ushort)20739, (ushort)26389, (ushort)24323, (ushort)38204, (ushort)63717, (ushort)5166, (ushort)29729, (ushort)4878, (ushort)33854, (ushort)26726, (ushort)34184, (ushort)24510, (ushort)52764, (ushort)7182, (ushort)31086, (ushort)9304, (ushort)3634, (ushort)41312, (ushort)15551, (ushort)21612, (ushort)41681, (ushort)27003, (ushort)11735, (ushort)53362, (ushort)3709, (ushort)19712, (ushort)65139, (ushort)56266, (ushort)19922, (ushort)47186, (ushort)10687, (ushort)41891, (ushort)37091, (ushort)52307, (ushort)46856, (ushort)10924, (ushort)3828, (ushort)37545, (ushort)23672, (ushort)55554, (ushort)29466, (ushort)62925, (ushort)65463, (ushort)63509, (ushort)17104, (ushort)60501, (ushort)59105, (ushort)56377, (ushort)5861, (ushort)41290, (ushort)50071, (ushort)36382, (ushort)11548, (ushort)14010, (ushort)59271, (ushort)11456}, 0) ;
            p330.max_distance = (ushort)(ushort)60301;
            p330.min_distance = (ushort)(ushort)3568;
            p330.increment = (byte)(byte)132;
            p330.sensor_type = MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_ULTRASOUND;
            p330.time_usec = (ulong)2473180393616130214L;
            CommunicationChannel.instance.send(p330);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnUAVIONIX_ADSB_OUT_CFGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.aircraftSize == UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE.UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L45_45M);
                Debug.Assert(pack.gpsOffsetLon == UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON.UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON_NO_DATA);
                Debug.Assert(pack.stallSpeed == (ushort)(ushort)9659);
                Debug.Assert(pack.rfSelect == (UAVIONIX_ADSB_OUT_RF_SELECT.UAVIONIX_ADSB_OUT_RF_SELECT_TX_ENABLED));
                Debug.Assert(pack.gpsOffsetLat == UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT.UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT_RIGHT_0M);
                Debug.Assert(pack.ICAO == (uint)3325197749U);
                Debug.Assert(pack.emitterType == ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_LIGHT);
                Debug.Assert(pack.callsign_LEN(ph) == 2);
                Debug.Assert(pack.callsign_TRY(ph).Equals("iv"));
            };
            GroundControl.UAVIONIX_ADSB_OUT_CFG p10001 = CommunicationChannel.new_UAVIONIX_ADSB_OUT_CFG();
            PH.setPack(p10001);
            p10001.rfSelect = (UAVIONIX_ADSB_OUT_RF_SELECT.UAVIONIX_ADSB_OUT_RF_SELECT_TX_ENABLED);
            p10001.emitterType = ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_LIGHT;
            p10001.gpsOffsetLat = UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT.UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT_RIGHT_0M;
            p10001.ICAO = (uint)3325197749U;
            p10001.aircraftSize = UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE.UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L45_45M;
            p10001.stallSpeed = (ushort)(ushort)9659;
            p10001.gpsOffsetLon = UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON.UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON_NO_DATA;
            p10001.callsign_SET("iv", PH) ;
            CommunicationChannel.instance.send(p10001);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnUAVIONIX_ADSB_OUT_DYNAMICReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.velVert == (short)(short)10981);
                Debug.Assert(pack.gpsAlt == (int)1856112114);
                Debug.Assert(pack.accuracyVel == (ushort)(ushort)18556);
                Debug.Assert(pack.squawk == (ushort)(ushort)44831);
                Debug.Assert(pack.accuracyVert == (ushort)(ushort)42402);
                Debug.Assert(pack.emergencyStatus == UAVIONIX_ADSB_EMERGENCY_STATUS.UAVIONIX_ADSB_OUT_RESERVED);
                Debug.Assert(pack.velNS == (short)(short) -1390);
                Debug.Assert(pack.gpsFix == UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX.UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_NONE_0);
                Debug.Assert(pack.gpsLon == (int)812114970);
                Debug.Assert(pack.numSats == (byte)(byte)63);
                Debug.Assert(pack.accuracyHor == (uint)1679872439U);
                Debug.Assert(pack.VelEW == (short)(short)5926);
                Debug.Assert(pack.state == (UAVIONIX_ADSB_OUT_DYNAMIC_STATE.UAVIONIX_ADSB_OUT_DYNAMIC_STATE_NICBARO_CROSSCHECKED |
                                            UAVIONIX_ADSB_OUT_DYNAMIC_STATE.UAVIONIX_ADSB_OUT_DYNAMIC_STATE_AUTOPILOT_ENABLED));
                Debug.Assert(pack.gpsLat == (int)889523357);
                Debug.Assert(pack.utcTime == (uint)1585289757U);
                Debug.Assert(pack.baroAltMSL == (int)1305377265);
            };
            GroundControl.UAVIONIX_ADSB_OUT_DYNAMIC p10002 = CommunicationChannel.new_UAVIONIX_ADSB_OUT_DYNAMIC();
            PH.setPack(p10002);
            p10002.emergencyStatus = UAVIONIX_ADSB_EMERGENCY_STATUS.UAVIONIX_ADSB_OUT_RESERVED;
            p10002.utcTime = (uint)1585289757U;
            p10002.accuracyHor = (uint)1679872439U;
            p10002.velVert = (short)(short)10981;
            p10002.state = (UAVIONIX_ADSB_OUT_DYNAMIC_STATE.UAVIONIX_ADSB_OUT_DYNAMIC_STATE_NICBARO_CROSSCHECKED |
                            UAVIONIX_ADSB_OUT_DYNAMIC_STATE.UAVIONIX_ADSB_OUT_DYNAMIC_STATE_AUTOPILOT_ENABLED);
            p10002.baroAltMSL = (int)1305377265;
            p10002.accuracyVert = (ushort)(ushort)42402;
            p10002.gpsAlt = (int)1856112114;
            p10002.velNS = (short)(short) -1390;
            p10002.numSats = (byte)(byte)63;
            p10002.gpsLat = (int)889523357;
            p10002.VelEW = (short)(short)5926;
            p10002.squawk = (ushort)(ushort)44831;
            p10002.accuracyVel = (ushort)(ushort)18556;
            p10002.gpsLon = (int)812114970;
            p10002.gpsFix = UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX.UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_NONE_0;
            CommunicationChannel.instance.send(p10002);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnUAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.rfHealth == (UAVIONIX_ADSB_RF_HEALTH.UAVIONIX_ADSB_RF_HEALTH_INITIALIZING |
                                               UAVIONIX_ADSB_RF_HEALTH.UAVIONIX_ADSB_RF_HEALTH_OK |
                                               UAVIONIX_ADSB_RF_HEALTH.UAVIONIX_ADSB_RF_HEALTH_FAIL_RX));
            };
            GroundControl.UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT p10003 = CommunicationChannel.new_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT();
            PH.setPack(p10003);
            p10003.rfHealth = (UAVIONIX_ADSB_RF_HEALTH.UAVIONIX_ADSB_RF_HEALTH_INITIALIZING |
                               UAVIONIX_ADSB_RF_HEALTH.UAVIONIX_ADSB_RF_HEALTH_OK |
                               UAVIONIX_ADSB_RF_HEALTH.UAVIONIX_ADSB_RF_HEALTH_FAIL_RX);
            CommunicationChannel.instance.send(p10003);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnDEVICE_OP_READReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)111);
                Debug.Assert(pack.count == (byte)(byte)151);
                Debug.Assert(pack.busname_LEN(ph) == 10);
                Debug.Assert(pack.busname_TRY(ph).Equals("gqotusxzko"));
                Debug.Assert(pack.regstart == (byte)(byte)1);
                Debug.Assert(pack.address == (byte)(byte)164);
                Debug.Assert(pack.bustype == DEVICE_OP_BUSTYPE.DEVICE_OP_BUSTYPE_I2C);
                Debug.Assert(pack.bus == (byte)(byte)50);
                Debug.Assert(pack.request_id == (uint)991813630U);
                Debug.Assert(pack.target_component == (byte)(byte)197);
            };
            GroundControl.DEVICE_OP_READ p11000 = CommunicationChannel.new_DEVICE_OP_READ();
            PH.setPack(p11000);
            p11000.target_system = (byte)(byte)111;
            p11000.bus = (byte)(byte)50;
            p11000.bustype = DEVICE_OP_BUSTYPE.DEVICE_OP_BUSTYPE_I2C;
            p11000.regstart = (byte)(byte)1;
            p11000.address = (byte)(byte)164;
            p11000.target_component = (byte)(byte)197;
            p11000.count = (byte)(byte)151;
            p11000.request_id = (uint)991813630U;
            p11000.busname_SET("gqotusxzko", PH) ;
            CommunicationChannel.instance.send(p11000);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnDEVICE_OP_READ_REPLYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.request_id == (uint)2170611626U);
                Debug.Assert(pack.regstart == (byte)(byte)115);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)120, (byte)186, (byte)214, (byte)245, (byte)67, (byte)6, (byte)96, (byte)177, (byte)70, (byte)101, (byte)116, (byte)217, (byte)225, (byte)53, (byte)44, (byte)238, (byte)206, (byte)227, (byte)155, (byte)104, (byte)68, (byte)5, (byte)112, (byte)147, (byte)112, (byte)106, (byte)126, (byte)128, (byte)114, (byte)37, (byte)93, (byte)132, (byte)186, (byte)236, (byte)36, (byte)175, (byte)119, (byte)108, (byte)36, (byte)107, (byte)151, (byte)216, (byte)254, (byte)11, (byte)248, (byte)152, (byte)217, (byte)170, (byte)65, (byte)149, (byte)72, (byte)160, (byte)130, (byte)178, (byte)93, (byte)92, (byte)70, (byte)236, (byte)184, (byte)132, (byte)242, (byte)216, (byte)34, (byte)130, (byte)152, (byte)60, (byte)8, (byte)83, (byte)119, (byte)14, (byte)195, (byte)109, (byte)127, (byte)145, (byte)139, (byte)82, (byte)92, (byte)46, (byte)151, (byte)116, (byte)223, (byte)44, (byte)182, (byte)72, (byte)201, (byte)23, (byte)8, (byte)45, (byte)252, (byte)209, (byte)227, (byte)204, (byte)18, (byte)236, (byte)118, (byte)19, (byte)245, (byte)32, (byte)26, (byte)95, (byte)101, (byte)79, (byte)186, (byte)146, (byte)178, (byte)189, (byte)83, (byte)198, (byte)58, (byte)199, (byte)231, (byte)30, (byte)17, (byte)109, (byte)90, (byte)186, (byte)241, (byte)173, (byte)51, (byte)120, (byte)142, (byte)50, (byte)17, (byte)116, (byte)238, (byte)147, (byte)50, (byte)123}));
                Debug.Assert(pack.count == (byte)(byte)46);
                Debug.Assert(pack.result == (byte)(byte)72);
            };
            GroundControl.DEVICE_OP_READ_REPLY p11001 = CommunicationChannel.new_DEVICE_OP_READ_REPLY();
            PH.setPack(p11001);
            p11001.regstart = (byte)(byte)115;
            p11001.request_id = (uint)2170611626U;
            p11001.data__SET(new byte[] {(byte)120, (byte)186, (byte)214, (byte)245, (byte)67, (byte)6, (byte)96, (byte)177, (byte)70, (byte)101, (byte)116, (byte)217, (byte)225, (byte)53, (byte)44, (byte)238, (byte)206, (byte)227, (byte)155, (byte)104, (byte)68, (byte)5, (byte)112, (byte)147, (byte)112, (byte)106, (byte)126, (byte)128, (byte)114, (byte)37, (byte)93, (byte)132, (byte)186, (byte)236, (byte)36, (byte)175, (byte)119, (byte)108, (byte)36, (byte)107, (byte)151, (byte)216, (byte)254, (byte)11, (byte)248, (byte)152, (byte)217, (byte)170, (byte)65, (byte)149, (byte)72, (byte)160, (byte)130, (byte)178, (byte)93, (byte)92, (byte)70, (byte)236, (byte)184, (byte)132, (byte)242, (byte)216, (byte)34, (byte)130, (byte)152, (byte)60, (byte)8, (byte)83, (byte)119, (byte)14, (byte)195, (byte)109, (byte)127, (byte)145, (byte)139, (byte)82, (byte)92, (byte)46, (byte)151, (byte)116, (byte)223, (byte)44, (byte)182, (byte)72, (byte)201, (byte)23, (byte)8, (byte)45, (byte)252, (byte)209, (byte)227, (byte)204, (byte)18, (byte)236, (byte)118, (byte)19, (byte)245, (byte)32, (byte)26, (byte)95, (byte)101, (byte)79, (byte)186, (byte)146, (byte)178, (byte)189, (byte)83, (byte)198, (byte)58, (byte)199, (byte)231, (byte)30, (byte)17, (byte)109, (byte)90, (byte)186, (byte)241, (byte)173, (byte)51, (byte)120, (byte)142, (byte)50, (byte)17, (byte)116, (byte)238, (byte)147, (byte)50, (byte)123}, 0) ;
            p11001.count = (byte)(byte)46;
            p11001.result = (byte)(byte)72;
            CommunicationChannel.instance.send(p11001);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnDEVICE_OP_WRITEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)77);
                Debug.Assert(pack.target_component == (byte)(byte)117);
                Debug.Assert(pack.request_id == (uint)1010091232U);
                Debug.Assert(pack.count == (byte)(byte)17);
                Debug.Assert(pack.address == (byte)(byte)108);
                Debug.Assert(pack.bustype == DEVICE_OP_BUSTYPE.DEVICE_OP_BUSTYPE_I2C);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)143, (byte)241, (byte)157, (byte)94, (byte)210, (byte)208, (byte)72, (byte)146, (byte)202, (byte)80, (byte)132, (byte)254, (byte)181, (byte)236, (byte)99, (byte)249, (byte)179, (byte)110, (byte)143, (byte)148, (byte)86, (byte)163, (byte)128, (byte)142, (byte)55, (byte)204, (byte)189, (byte)186, (byte)145, (byte)155, (byte)78, (byte)224, (byte)29, (byte)81, (byte)111, (byte)137, (byte)72, (byte)105, (byte)74, (byte)247, (byte)118, (byte)89, (byte)193, (byte)176, (byte)166, (byte)106, (byte)92, (byte)119, (byte)235, (byte)61, (byte)121, (byte)199, (byte)132, (byte)35, (byte)62, (byte)97, (byte)235, (byte)68, (byte)54, (byte)36, (byte)152, (byte)75, (byte)1, (byte)50, (byte)255, (byte)13, (byte)153, (byte)120, (byte)43, (byte)221, (byte)15, (byte)189, (byte)167, (byte)114, (byte)19, (byte)155, (byte)14, (byte)26, (byte)24, (byte)181, (byte)202, (byte)177, (byte)127, (byte)146, (byte)202, (byte)54, (byte)94, (byte)109, (byte)115, (byte)174, (byte)168, (byte)72, (byte)8, (byte)222, (byte)79, (byte)214, (byte)150, (byte)105, (byte)80, (byte)233, (byte)189, (byte)81, (byte)218, (byte)246, (byte)154, (byte)252, (byte)48, (byte)104, (byte)250, (byte)117, (byte)197, (byte)243, (byte)45, (byte)67, (byte)248, (byte)117, (byte)255, (byte)225, (byte)209, (byte)27, (byte)91, (byte)220, (byte)90, (byte)161, (byte)237, (byte)247, (byte)232, (byte)225}));
                Debug.Assert(pack.bus == (byte)(byte)3);
                Debug.Assert(pack.regstart == (byte)(byte)123);
                Debug.Assert(pack.busname_LEN(ph) == 6);
                Debug.Assert(pack.busname_TRY(ph).Equals("JWYpfS"));
            };
            GroundControl.DEVICE_OP_WRITE p11002 = CommunicationChannel.new_DEVICE_OP_WRITE();
            PH.setPack(p11002);
            p11002.data__SET(new byte[] {(byte)143, (byte)241, (byte)157, (byte)94, (byte)210, (byte)208, (byte)72, (byte)146, (byte)202, (byte)80, (byte)132, (byte)254, (byte)181, (byte)236, (byte)99, (byte)249, (byte)179, (byte)110, (byte)143, (byte)148, (byte)86, (byte)163, (byte)128, (byte)142, (byte)55, (byte)204, (byte)189, (byte)186, (byte)145, (byte)155, (byte)78, (byte)224, (byte)29, (byte)81, (byte)111, (byte)137, (byte)72, (byte)105, (byte)74, (byte)247, (byte)118, (byte)89, (byte)193, (byte)176, (byte)166, (byte)106, (byte)92, (byte)119, (byte)235, (byte)61, (byte)121, (byte)199, (byte)132, (byte)35, (byte)62, (byte)97, (byte)235, (byte)68, (byte)54, (byte)36, (byte)152, (byte)75, (byte)1, (byte)50, (byte)255, (byte)13, (byte)153, (byte)120, (byte)43, (byte)221, (byte)15, (byte)189, (byte)167, (byte)114, (byte)19, (byte)155, (byte)14, (byte)26, (byte)24, (byte)181, (byte)202, (byte)177, (byte)127, (byte)146, (byte)202, (byte)54, (byte)94, (byte)109, (byte)115, (byte)174, (byte)168, (byte)72, (byte)8, (byte)222, (byte)79, (byte)214, (byte)150, (byte)105, (byte)80, (byte)233, (byte)189, (byte)81, (byte)218, (byte)246, (byte)154, (byte)252, (byte)48, (byte)104, (byte)250, (byte)117, (byte)197, (byte)243, (byte)45, (byte)67, (byte)248, (byte)117, (byte)255, (byte)225, (byte)209, (byte)27, (byte)91, (byte)220, (byte)90, (byte)161, (byte)237, (byte)247, (byte)232, (byte)225}, 0) ;
            p11002.regstart = (byte)(byte)123;
            p11002.bustype = DEVICE_OP_BUSTYPE.DEVICE_OP_BUSTYPE_I2C;
            p11002.request_id = (uint)1010091232U;
            p11002.target_system = (byte)(byte)77;
            p11002.busname_SET("JWYpfS", PH) ;
            p11002.address = (byte)(byte)108;
            p11002.bus = (byte)(byte)3;
            p11002.count = (byte)(byte)17;
            p11002.target_component = (byte)(byte)117;
            CommunicationChannel.instance.send(p11002);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnDEVICE_OP_WRITE_REPLYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.request_id == (uint)456405154U);
                Debug.Assert(pack.result == (byte)(byte)164);
            };
            GroundControl.DEVICE_OP_WRITE_REPLY p11003 = CommunicationChannel.new_DEVICE_OP_WRITE_REPLY();
            PH.setPack(p11003);
            p11003.request_id = (uint)456405154U;
            p11003.result = (byte)(byte)164;
            CommunicationChannel.instance.send(p11003);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnADAP_TUNINGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.error == (float) -4.344054E37F);
                Debug.Assert(pack.theta == (float)3.0849529E38F);
                Debug.Assert(pack.axis == PID_TUNING_AXIS.PID_TUNING_YAW);
                Debug.Assert(pack.sigma_dot == (float) -2.7085635E38F);
                Debug.Assert(pack.f == (float)9.951676E37F);
                Debug.Assert(pack.theta_dot == (float)2.7883545E38F);
                Debug.Assert(pack.f_dot == (float)2.1271262E38F);
                Debug.Assert(pack.omega_dot == (float) -3.0222881E38F);
                Debug.Assert(pack.omega == (float) -1.674294E38F);
                Debug.Assert(pack.achieved == (float)2.7188254E38F);
                Debug.Assert(pack.desired == (float)4.0528597E37F);
                Debug.Assert(pack.u == (float)3.3900973E38F);
                Debug.Assert(pack.sigma == (float) -1.1967338E38F);
            };
            GroundControl.ADAP_TUNING p11010 = CommunicationChannel.new_ADAP_TUNING();
            PH.setPack(p11010);
            p11010.omega = (float) -1.674294E38F;
            p11010.sigma_dot = (float) -2.7085635E38F;
            p11010.u = (float)3.3900973E38F;
            p11010.error = (float) -4.344054E37F;
            p11010.achieved = (float)2.7188254E38F;
            p11010.theta = (float)3.0849529E38F;
            p11010.sigma = (float) -1.1967338E38F;
            p11010.axis = PID_TUNING_AXIS.PID_TUNING_YAW;
            p11010.desired = (float)4.0528597E37F;
            p11010.theta_dot = (float)2.7883545E38F;
            p11010.f_dot = (float)2.1271262E38F;
            p11010.omega_dot = (float) -3.0222881E38F;
            p11010.f = (float)9.951676E37F;
            CommunicationChannel.instance.send(p11010);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnVISION_POSITION_DELTAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.angle_delta.SequenceEqual(new float[] {-3.1630304E38F, 1.433858E38F, -2.0647728E38F}));
                Debug.Assert(pack.time_delta_usec == (ulong)2961130291619475850L);
                Debug.Assert(pack.time_usec == (ulong)8668252793052617006L);
                Debug.Assert(pack.position_delta.SequenceEqual(new float[] {1.821127E38F, -2.651462E37F, -3.1980492E37F}));
                Debug.Assert(pack.confidence == (float) -5.412094E36F);
            };
            GroundControl.VISION_POSITION_DELTA p11011 = CommunicationChannel.new_VISION_POSITION_DELTA();
            PH.setPack(p11011);
            p11011.time_delta_usec = (ulong)2961130291619475850L;
            p11011.confidence = (float) -5.412094E36F;
            p11011.angle_delta_SET(new float[] {-3.1630304E38F, 1.433858E38F, -2.0647728E38F}, 0) ;
            p11011.position_delta_SET(new float[] {1.821127E38F, -2.651462E37F, -3.1980492E37F}, 0) ;
            p11011.time_usec = (ulong)8668252793052617006L;
            CommunicationChannel.instance.send(p11011);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
        }
    }
}